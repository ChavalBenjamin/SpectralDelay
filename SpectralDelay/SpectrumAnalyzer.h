#pragma once

#include <vector>
#include <complex>
#include <cmath>
#include <algorithm>

// ============================================================================
// SpectrumAnalyzer
//
// Analyseur de spectre leger, purement visuel - independant du moteur de
// traitement (taille FFT fixe, ne participe pas au signal). Calcule la
// magnitude par bande (en dB), avec un lissage "attaque rapide, relachement
// progressif" pour un affichage lisible plutot que nerveux.
// ============================================================================

class SpectrumAnalyzer
{
public:
  using cplx = std::complex<float>;

  void Init(double sampleRate)
  {
    mSampleRate = sampleRate;
    mHopSize = kFFTSize / kOverlap;

    mRing.assign(kFFTSize, 0.f);
    mWindow.resize(kFFTSize);
    mTime.resize(kFFTSize);
    mCplx.assign(kFFTSize, cplx(0.f, 0.f));
    mMagDb.assign(kFFTSize / 2 + 1, kFloorDb);

    for (int i = 0; i < kFFTSize; i++)
      mWindow[i] = 0.5f - 0.5f * std::cos(2.f * kPi * i / (kFFTSize - 1));

    mWritePos = 0;
    mSamplesUntilHop = mHopSize;
  }

  // Alimente l'analyseur (mono - deja mixe L+R en amont si besoin).
  void Process(const float* in, int nFrames)
  {
    for (int i = 0; i < nFrames; i++)
    {
      mRing[mWritePos] = in[i];
      mWritePos = (mWritePos + 1) % kFFTSize;

      if (--mSamplesUntilHop == 0)
      {
        mSamplesUntilHop = mHopSize;
        ProcessHop();
      }
    }
  }

  const float* GetMagnitudeDb() const { return mMagDb.data(); }
  int GetNumBins() const { return kFFTSize / 2 + 1; }
  int GetFFTSize() const { return kFFTSize; }
  double GetSampleRate() const { return mSampleRate; }

private:
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

  void ProcessHop()
  {
    int start = mWritePos;
    for (int i = 0; i < kFFTSize; i++)
      mTime[i] = mRing[(start + i) % kFFTSize];

    for (int i = 0; i < kFFTSize; i++)
      mCplx[i] = cplx(mTime[i] * mWindow[i], 0.f);

    FFT(mCplx, false);

    int numBins = kFFTSize / 2;
    for (int k = 0; k <= numBins; k++)
    {
      float mag = std::abs(mCplx[k]) / (float)kFFTSize;
      float db = 20.f * std::log10(std::max(mag, 1e-9f));
      db = std::max(db, kFloorDb);

      // Attaque rapide, relachement progressif - lisible sans etre nerveux.
      if (db > mMagDb[k]) mMagDb[k] = db;
      else mMagDb[k] = mMagDb[k] + (db - mMagDb[k]) * kReleaseCoeff;
    }
  }

  static constexpr float kPi = 3.14159265358979323846f;
  static constexpr int kFFTSize = 2048;
  static constexpr int kOverlap = 4;
  static constexpr float kFloorDb = -80.f;
  static constexpr float kReleaseCoeff = 0.15f;

  double mSampleRate = 44100.0;
  int mHopSize = 512;
  int mSamplesUntilHop = 512;
  int mWritePos = 0;

  std::vector<float> mRing, mWindow, mTime;
  std::vector<cplx> mCplx;
  std::vector<float> mMagDb;
};
