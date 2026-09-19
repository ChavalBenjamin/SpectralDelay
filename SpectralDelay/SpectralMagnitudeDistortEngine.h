#pragma once

#include <vector>
#include <complex>
#include <cmath>
#include <algorithm>

// ============================================================================
// SpectralMagnitudeDistortEngine
//
// 3 mecanismes combinables :
//
//  1. Distorsion en magnitude : nouvelle_magnitude = magnitude ^ exposant,
//     par bande - redistribue l'energie deja presente.
//  2. Injection harmonique : copie une partie de l'energie de chaque
//     bande vers ses multiples (x2, x3, x4) - vraie nouvelle richesse
//     spectrale.
//  3. Distorsion temporelle : waveshaping (tanh) sur le signal
//     RECONSTRUIT final (apres overlap-add).
//
// Lissage par bande (attaque/relachement) applique au GAIN EFFECTIF de
// chaque bande (rapport magnitude finale / magnitude d'origine) - sans
// ca, chaque hop recalcule tout independamment, creant des discontinuites
// audibles (clics) a chaque saut de bloc.
//
// La latence de traitement (environ une fenetre FFT complete) est
// exposee via GetLatencySamples(), pour que le plugin puisse a la fois
// informer l'hote (PDC) et compenser son propre signal sec (Dry/Wet).
// ============================================================================

class SpectralMagnitudeDistortEngine
{
public:
  using cplx = std::complex<float>;

  void Init(int fftSize, int overlapFactor, double sampleRate)
  {
    mFFTSize = fftSize;
    mOverlap = overlapFactor;
    mHopSize = mFFTSize / mOverlap;
    mSampleRate = sampleRate;

    mRing.assign(mFFTSize, 0.f);
    mRingOut.assign(mFFTSize, 0.f);
    mWindow.resize(mFFTSize);
    mTime.resize(mFFTSize);
    mCplx.assign(mFFTSize, cplx(0.f, 0.f));
    mMagBuf.assign(mFFTSize, 0.f);
    mMagInjected.assign(mFFTSize, 0.f);
    mMaxInjectionBuf.assign(mFFTSize, 0.f);
    mPhaseBuf.assign(mFFTSize, 0.f);
    mOrigMagBuf.assign(mFFTSize, 0.f);

    int numBins = mFFTSize / 2 + 1;
    mMagSmoothDb.assign(numBins, kFloorDb);

    for (int i = 0; i < mFFTSize; i++)
      mWindow[i] = 0.5f - 0.5f * std::cos(2.f * kPi * i / (mFFTSize - 1));

    // Coefficients d'attaque/relachement du lissage de gain par bande.
    float hopMs = (float)mHopSize / (float)mSampleRate * 1000.f;
    mAttackCoeff = 1.f - std::exp(-hopMs / kSmoothAttackMs);
    mReleaseCoeff = 1.f - std::exp(-hopMs / kSmoothReleaseMs);

    mWritePos = 0;
    mReadPos = 0;
    mSamplesUntilHop = mHopSize;
  }

  void SetCurve(const float* curve, int curveSize) { mCurve = curve; mCurveSize = curveSize; }
  void SetHarmonicInjection(float amount) { mHarmonicInjection = std::clamp(amount, 0.f, 1.f); }
  void SetDecayExponent(float exponent) { mDecayExponent = std::clamp(exponent, 0.2f, 1.f); }

  // Courbe non-lineaire : 75% du parcours du bouton couvre les 15%
  // premiers de Drive (la zone la plus interessante, dilatee pour plus
  // de precision), le reste suit une courbe exponentielle.
  // Prend le drive DEJA deforme (0-1), aucune deformation ici - la
  // courbe non-lineaire se fait UNE SEULE FOIS, cote plugin, pour eviter
  // toute confusion/double application.
  void SetTempDrive(float drive) { mTempDrive = std::clamp(drive, 0.f, 1.f); }

  // Latence de traitement introduite (en echantillons) - environ une
  // fenetre FFT complete pour ce type d'architecture (ring buffer STFT).
  int GetLatencySamples() const { return mFFTSize; }

  void Process(const float* in, float* out, int nFrames)
  {
    for (int i = 0; i < nFrames; i++)
    {
      mRing[mWritePos] = in[i];

      float wetSample = mRingOut[mReadPos];
      mRingOut[mReadPos] = 0.f;

      // Distorsion temporelle (waveshaping) : appliquee ICI, sur le
      // signal RECONSTRUIT final (apres overlap-add) - jamais avant,
      // sinon la distorsion serait appliquee plusieurs fois dans les
      // zones de chevauchement entre hops.
      out[i] = Waveshape(wetSample, mTempDrive);

      mWritePos = (mWritePos + 1) % mFFTSize;
      mReadPos = (mReadPos + 1) % mFFTSize;

      if (--mSamplesUntilHop == 0)
      {
        mSamplesUntilHop = mHopSize;
        ProcessHop();
      }
    }
  }

private:
  static float Waveshape(float x, float drive)
  {
    if (drive <= 0.f) return x;
    // Melange LINEAIRE entre x (identite) et une forme SATUREE FIXE - a
    // drive=0, ca redonne x EXACTEMENT (propriete mathematique garantie,
    // pas juste un cas special qui masquait une vraie discontinuite avec
    // la formule generale juste au-dessus de 0).
    constexpr float kFixedK = 1.f + kMaxDrive;
    float saturated = std::tanh(x * kFixedK) / std::tanh(kFixedK);
    return x + drive * (saturated - x);
  }

  void ReadRingIntoLinear(const std::vector<float>& ring, std::vector<float>& dst)
  {
    int start = mWritePos;
    for (int i = 0; i < mFFTSize; i++)
      dst[i] = ring[(start + i) % mFFTSize];
  }

  static void FFT(std::vector<cplx>& a, bool invert)
  {
    int n = (int)a.size();
    for (int i = 1, j = 0; i < n; i++)
    {
      int bit = n >> 1;
      for (; j & bit; bit >>= 1)
        j ^= bit;
      j ^= bit;
      if (i < j) std::swap(a[i], a[j]);
    }

    for (int len = 2; len <= n; len <<= 1)
    {
      float ang = 2.f * kPi / (float)len * (invert ? 1.f : -1.f);
      cplx wlen(std::cos(ang), std::sin(ang));
      for (int i = 0; i < n; i += len)
      {
        cplx w(1.f, 0.f);
        for (int k = 0; k < len / 2; k++)
        {
          cplx u = a[i + k];
          cplx v = a[i + k + len / 2] * w;
          a[i + k] = u + v;
          a[i + k + len / 2] = u - v;
          w *= wlen;
        }
      }
    }
    if (invert)
      for (auto& x : a) x /= (float)n;
  }

  float GetExponentForBin(int binIdx) const
  {
    if (!mCurve || mCurveSize < 2) return 1.f;

    float freq = (float)binIdx * (float)mSampleRate / (float)mFFTSize;
    freq = std::clamp(freq, 20.f, 20000.f);
    float logPos = std::log(freq / 20.f) / std::log(20000.f / 20.f);

    float pos = logPos * (float)(mCurveSize - 1);
    int idx0 = (int)pos;
    int idx1 = std::min(idx0 + 1, mCurveSize - 1);
    float frac = pos - (float)idx0;
    float curveVal = mCurve[idx0] * (1.f - frac) + mCurve[idx1] * frac; // -1..1

    return std::pow(kMaxExponent, curveVal);
  }

  void ProcessHop()
  {
    ReadRingIntoLinear(mRing, mTime);

    for (int i = 0; i < mFFTSize; i++)
      mCplx[i] = cplx(mTime[i] * mWindow[i], 0.f);

    FFT(mCplx, false);

    int numBins = mFFTSize / 2;

    // Passe 1 : distorsion en magnitude par bande. Garde la magnitude
    // D'ORIGINE (avant tout traitement) pour le lissage de gain plus bas.
    float energyBefore = 0.f;
    for (int k = 0; k <= numBins; k++)
    {
      float mag = std::abs(mCplx[k]);
      mOrigMagBuf[k] = mag;
      float exponent = GetExponentForBin(k);
      mMagBuf[k] = std::pow(std::max(mag, 1e-9f), exponent);
      mPhaseBuf[k] = std::arg(mCplx[k]);
      energyBefore += mag * mag;
    }

    // Passe 2 : injection harmonique. Le nombre d'harmoniques injectees
    // grandit lui aussi avec Decroissance : 3 harmoniques (x2/x3/x4) a
    // Decroissance=1, jusqu'a 11 (x2..x12) a Decroissance=0.2 - une
    // harmonique de plus tous les 0.1 - combine avec la chute
    // d'amplitude plus plate, l'effet devient bien plus audible sur
    // toute la course du bouton.
    if (mHarmonicInjection > 0.001f)
    {
      int maxHarmonic = 4 + (int)std::round((1.f - mDecayExponent) * 10.f);
      std::copy(mMagBuf.begin(), mMagBuf.begin() + numBins + 1, mMagInjected.begin());
      std::fill(mMaxInjectionBuf.begin(), mMaxInjectionBuf.begin() + numBins + 1, 0.f);

      for (int k = 1; k <= numBins; k++)
      {
        float srcMag = mMagBuf[k];
        if (srcMag < 1e-6f) continue;
        for (int h = 2; h <= maxHarmonic; h++)
        {
          int targetBin = k * h;
          if (targetBin > numBins) break;
          // Decroissance reglable : exposant 1 = decroissance actuelle
          // (÷n), exposant plus bas (jusque 0.2) = beaucoup plus plat,
          // les harmoniques elevees restent presque aussi presentes que
          // les basses - plus de complexite, plus d'energie globale.
          float contribution = srcMag * (mHarmonicInjection / std::pow((float)h, mDecayExponent));

          // MAX plutot que SOMME : certaines bandes cibles (12, 24, 60...
          // les indices "hautement composes") recoivent des contributions
          // de PLUSIEURS sources en meme temps (x2 ET x3 ET x4 a la fois)
          // - les additionner faisait exploser ces bandes precises en
          // pics, d'autant plus visibles que la fenetre FFT est grande
          // (plus de bandes = plus de ces points chauds). Ne garder que
          // la contribution la plus forte evite l'accumulation.
          mMaxInjectionBuf[targetBin] = std::max(mMaxInjectionBuf[targetBin], contribution);
        }
      }

      for (int k = 0; k <= numBins; k++)
        mMagInjected[k] = mMagBuf[k] + mMaxInjectionBuf[k];
      std::copy(mMagInjected.begin(), mMagInjected.begin() + numBins + 1, mMagBuf.begin());
    }

    // Passe 3 : compensation de gain GLOBALE (apres distorsion ET
    // injection). Plancher abaisse a 0.01 (au lieu de 0.1) : l'injection
    // peut ajouter beaucoup d'energie, l'ancien plancher ne suffisait
    // plus a compenser dans les cas extremes.
    float energyAfter = 0.f;
    for (int k = 0; k <= numBins; k++)
      energyAfter += mMagBuf[k] * mMagBuf[k];

    float globalGain = std::sqrt(energyBefore / std::max(energyAfter, 1e-9f));
    globalGain = std::clamp(globalGain, 0.01f, 10.f);

    // Passe 4 : lissage PAR BANDE du niveau ABSOLU final (pas un rapport
    // a l'origine) - avec l'Injection, une bande quasi silencieuse a
    // l'origine peut recevoir beaucoup d'energie injectee : diviser par
    // cette origine quasi-nulle rendait le rapport de gain numeriquement
    // instable, causant des sauts brutaux precisement sur les bandes les
    // plus interessantes (celles qui recoivent l'injection).
    for (int k = 0; k <= numBins; k++)
    {
      float finalMag = mMagBuf[k] * globalGain;
      float targetDb = 20.f * std::log10(std::max(finalMag, 1e-9f));
      targetDb = std::max(targetDb, kFloorDb);

      if (targetDb > mMagSmoothDb[k])
        mMagSmoothDb[k] += (targetDb - mMagSmoothDb[k]) * mAttackCoeff;
      else
        mMagSmoothDb[k] += (targetDb - mMagSmoothDb[k]) * mReleaseCoeff;

      float outMag = std::pow(10.f, mMagSmoothDb[k] / 20.f);
      float outPhase = mPhaseBuf[k];

      cplx val(outMag * std::cos(outPhase), outMag * std::sin(outPhase));
      mCplx[k] = val;
      if (k > 0 && k < numBins)
        mCplx[mFFTSize - k] = std::conj(val);
    }

    FFT(mCplx, true);

    // Securite supplementaire : mesure le pic REEL de ce hop reconstruit,
    // et le ramene a une valeur raisonnable si besoin - deuxieme ligne
    // de defense, AVANT meme le limiteur final. Des bandes VOISINES
    // (proches en frequence) peuvent chacune recevoir une injection
    // individuellement raisonnable, mais se combiner bien plus fort une
    // fois reconverties en son reel - le plafond par bande (max au lieu
    // de somme) ne protege pas contre CE cas precis.
    float hopPeak = 0.f;
    for (int i = 0; i < mFFTSize; i++)
      hopPeak = std::max(hopPeak, std::abs(mCplx[i].real()));

    float safetyGain = 1.f;
    if (hopPeak > kHopPeakLimit)
      safetyGain = kHopPeakLimit / hopPeak;

    float normOverlap = 1.f / (float)mOverlap * 2.f * safetyGain;
    int start = mWritePos;
    for (int i = 0; i < mFFTSize; i++)
    {
      int idx = (start + i) % mFFTSize;
      mRingOut[idx] += mCplx[i].real() * mWindow[i] * normOverlap;
    }
  }

  static constexpr float kPi = 3.14159265358979323846f;
  static constexpr float kMaxExponent = 8.f;
  static constexpr float kMaxDrive = 30.f;
  static constexpr float kSmoothAttackMs = 5.f;
  static constexpr float kSmoothReleaseMs = 30.f; // allonge (etait 15ms), plus doux globalement
  static constexpr float kFloorDb = -100.f;
  static constexpr float kHopPeakLimit = 1.5f;

  int mFFTSize = 1024;
  int mOverlap = 4;
  int mHopSize = 256;
  int mSamplesUntilHop = 256;
  int mWritePos = 0;
  int mReadPos = 0;
  double mSampleRate = 44100.0;

  const float* mCurve = nullptr;
  int mCurveSize = 0;
  float mHarmonicInjection = 0.f;
  float mDecayExponent = 1.f; // 1 = decroissance actuelle, 0.2 = presque plat
  float mTempDrive = 0.f;

  float mAttackCoeff = 0.5f;
  float mReleaseCoeff = 0.2f;
  std::vector<float> mMagSmoothDb;

  std::vector<float> mRing, mRingOut, mWindow, mTime;
  std::vector<cplx> mCplx;
  std::vector<float> mMagBuf, mMagInjected, mPhaseBuf, mOrigMagBuf, mMaxInjectionBuf;
};
