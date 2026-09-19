#pragma once

#include <vector>
#include <complex>
#include <cmath>
#include <algorithm>

// ============================================================================
// SpectralDelayEngine
//
// Delay spectral "multibandes" : chaque bande FFT a son PROPRE temps de
// delai (5ms a 2.5s), pilote par la courbe partagee (-1 a 1, mappee sur
// une echelle log). Feedback commun a toutes les bandes (peut depasser
// 100% - le limiteur final protege). Mode Sync : le temps de chaque bande
// se cale sur la division rythmique la plus proche (binaire ou ternaire).
//
// Implementation : un historique circulaire de valeurs complexes PAR
// BANDE (assez long pour couvrir 2.5s au hop courant), mixe avec le
// signal entrant a chaque etape - un vrai banc de lignes a retard
// independantes, pas un simple decalage de phase.
// ============================================================================

class SpectralDelayEngine
{
public:
  using cplx = std::complex<float>;

  void Init(int fftSize, int overlapFactor, double sampleRate)
  {
    mFFTSize = fftSize;
    mOverlap = overlapFactor;
    mHopSize = mFFTSize / mOverlap;
    mSampleRate = sampleRate;
    mHopDurationMs = (float)mHopSize / (float)mSampleRate * 1000.f;

    mRing.assign(mFFTSize, 0.f);
    mRingOut.assign(mFFTSize, 0.f);
    mWindow.resize(mFFTSize);
    mTime.resize(mFFTSize);
    mCplx.assign(mFFTSize, cplx(0.f, 0.f));

    for (int i = 0; i < mFFTSize; i++)
      mWindow[i] = 0.5f - 0.5f * std::cos(2.f * kPi * i / (mFFTSize - 1));

    int numBins = mFFTSize / 2 + 1;
    mMaxDelayHops = std::max(1, (int)std::ceil(kMaxDelayMs / mHopDurationMs));
    mBinHistory.assign(numBins, std::vector<cplx>(mMaxDelayHops, cplx(0.f, 0.f)));
    mHistoryWritePos.assign(numBins, 0);

    mWritePos = 0;
    mReadPos = 0;
    mSamplesUntilHop = mHopSize;
  }

  void SetSampleRate(double sr) { mSampleRate = sr; }
  void SetCurve(const float* curve, int curveSize) { mCurve = curve; mCurveSize = curveSize; }
  void SetFeedback(float fb) { mFeedback = std::max(0.f, fb); } // peut depasser 1.0
  void SetSyncMode(bool sync) { mSyncMode = sync; }

  // Latence de traitement introduite (en echantillons) - environ une
  // fenetre FFT complete, meme principe que SpectralMagnitudeDistortEngine
  // (tampon circulaire STFT).
  int GetLatencySamples() const { return mFFTSize; }
  void SetBPM(double bpm) { mBPM = bpm; }

  void Process(const float* in, float* out, int nFrames)
  {
    for (int i = 0; i < nFrames; i++)
    {
      mRing[mWritePos] = in[i];

      out[i] = mRingOut[mReadPos];
      mRingOut[mReadPos] = 0.f;

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
    {
      for (auto& x : a)
        x /= (float)n;
    }
  }

  // Case (binaire 64..1, et son equivalent ternaire) la duree ms la plus
  // proche de la valeur demandee.
  float SnapToNearestSyncMs(float ms) const
  {
    double quarterMs = 60000.0 / std::max(20.0, mBPM);
    double wholeMs = quarterMs * 4.0;
    static const float divisors[] = { 1.f, 2.f, 4.f, 8.f, 16.f, 32.f, 64.f };

    // 0 est une cible valide au meme titre que les divisions rythmiques -
    // sans ca, le plafond bas en mode Sync accroche toujours sur la plus
    // petite division (64T) au lieu de vraiment atteindre "aucun delai".
    float best = 0.f;
    float bestDist = std::abs(ms);
    for (float d : divisors)
    {
      float straightMs = (float)(wholeMs / d);
      float tripletMs = straightMs * (2.f / 3.f);
      for (float candidate : { straightMs, tripletMs })
      {
        if (candidate < kMinDelayMs || candidate > kMaxDelayMs) continue;
        float dist = std::abs(candidate - ms);
        if (dist < bestDist) { bestDist = dist; best = candidate; }
      }
    }
    return best;
  }

  float GetDelayMsForBin(int binIdx) const
  {
    if (!mCurve || mCurveSize < 2) return kMinDelayMs;

    float freq = (float)binIdx * (float)mSampleRate / (float)mFFTSize;
    freq = std::clamp(freq, 20.f, 20000.f);
    float logPos = std::log(freq / 20.f) / std::log(20000.f / 20.f);

    float pos = logPos * (float)(mCurveSize - 1);
    int idx0 = (int)pos;
    int idx1 = std::min(idx0 + 1, mCurveSize - 1);
    float frac = pos - (float)idx0;
    float curveVal = mCurve[idx0] * (1.f - frac) + mCurve[idx1] * frac; // -1..1

    // -1..1 -> 0ms..2500ms via une courbe en puissance : garde un
    // "ressenti" log (beaucoup de resolution en bas), tout en touchant
    // vraiment 0 (impossible avec un vrai log, qui ne peut pas atteindre 0).
    float t = (curveVal + 1.f) * 0.5f; // 0..1
    float ms = kMaxDelayMs * std::pow(t, kDelayCurveExponent);

    if (mSyncMode) ms = SnapToNearestSyncMs(ms);

    return std::clamp(ms, kMinDelayMs, kMaxDelayMs);
  }

  void ProcessHop()
  {
    ReadRingIntoLinear(mRing, mTime);

    for (int i = 0; i < mFFTSize; i++)
      mCplx[i] = cplx(mTime[i] * mWindow[i], 0.f);

    FFT(mCplx, false);

    int numBins = mFFTSize / 2;
    for (int k = 0; k <= numBins; k++)
    {
      float delayMs = GetDelayMsForBin(k);

      if (delayMs < mHopDurationMs * 0.5f)
      {
        // "Pas de delai" reel (pas juste le plus petit delai possible) :
        // la bande reste totalement seche, aucun feedback. On alimente
        // quand meme l'historique (valeur brute, sans feedback) pour que
        // les bandes voisines avec delai restent coherentes si la courbe
        // change de forme.
        mBinHistory[k][mHistoryWritePos[k]] = mCplx[k];
        mHistoryWritePos[k] = (mHistoryWritePos[k] + 1) % mMaxDelayHops;
      }
      else
      {
        int delayHops = std::clamp((int)std::round(delayMs / mHopDurationMs), 1, mMaxDelayHops - 1);

        int readIdx = (mHistoryWritePos[k] - delayHops + mMaxDelayHops) % mMaxDelayHops;
        cplx delayed = mBinHistory[k][readIdx];

        cplx mixed = mCplx[k] + delayed * mFeedback;

        // Ecrit ce qui vient d'etre mixe (pas seulement l'entree) - c'est
        // ce qui permet au feedback de s'accumuler au fil des repetitions.
        mBinHistory[k][mHistoryWritePos[k]] = mixed;
        mHistoryWritePos[k] = (mHistoryWritePos[k] + 1) % mMaxDelayHops;

        // SORTIE = uniquement l'echo retarde (delayed), jamais l'entree
        // directe de ce hop - "vrai Wet" isole. L'entree ne rejoindra
        // la sortie qu'une fois qu'elle aura vraiment traverse la ligne
        // a retard, pas immediatement melangee comme avant.
        mCplx[k] = delayed;
      }

      if (k > 0 && k < numBins)
        mCplx[mFFTSize - k] = std::conj(mCplx[k]);
    }

    FFT(mCplx, true);

    float normOverlap = 1.f / (float)mOverlap * 2.f;
    int start = mWritePos;
    for (int i = 0; i < mFFTSize; i++)
    {
      int idx = (start + i) % mFFTSize;
      mRingOut[idx] += mCplx[i].real() * mWindow[i] * normOverlap;
    }
  }

  static constexpr float kPi = 3.14159265358979323846f;
  static constexpr float kMinDelayMs = 0.f;
  static constexpr float kDelayCurveExponent = 3.f; // courbe en puissance (pas log, qui ne peut pas toucher 0)
  static constexpr float kMaxDelayMs = 2500.f;

  int mFFTSize = 1024;
  int mOverlap = 4;
  int mHopSize = 256;
  int mSamplesUntilHop = 256;
  int mWritePos = 0;
  int mReadPos = 0;
  double mSampleRate = 44100.0;
  float mHopDurationMs = 10.f;

  const float* mCurve = nullptr;
  int mCurveSize = 0;
  float mFeedback = 0.f;
  bool mSyncMode = false;
  double mBPM = 120.0;

  int mMaxDelayHops = 1;
  std::vector<std::vector<cplx>> mBinHistory;
  std::vector<int> mHistoryWritePos;

  std::vector<float> mRing, mRingOut, mWindow, mTime;
  std::vector<cplx> mCplx;
};
