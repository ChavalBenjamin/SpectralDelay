#pragma once

#include <vector>
#include <complex>
#include <cmath>
#include <algorithm>

// ============================================================================
// SpectralFilterEngine
//
// Applique la courbe generee par SpectralCurveEngine comme un gain par
// bande de frequence (en dB), sur un signal reel via FFT/IFFT (STFT
// classique, fenetre de Hann, overlap-add). Un instance par canal
// (utilisee x2 pour le stereo).
//
// La courbe (-1 a 1) est interpolee sur une echelle LOGARITHMIQUE entre
// 20Hz et 20kHz - pas une simple correspondance lineaire bande-par-bande -
// pour correspondre a la facon dont l'oreille percoit les frequences.
// -1 = kMaxDb en coupe, +1 = kMaxDb en boost, 0 = inchange (0dB).
// ============================================================================

class SpectralFilterEngine
{
public:
  using cplx = std::complex<float>;

  void Init(int fftSize, int overlapFactor)
  {
    mFFTSize = fftSize;
    mOverlap = overlapFactor;
    mHopSize = mFFTSize / mOverlap;

    mRing.assign(mFFTSize, 0.f);
    mRingOut.assign(mFFTSize, 0.f);
    mWindow.resize(mFFTSize);
    mTime.resize(mFFTSize);
    mCplx.assign(mFFTSize, cplx(0.f, 0.f));

    for (int i = 0; i < mFFTSize; i++)
      mWindow[i] = 0.5f - 0.5f * std::cos(2.f * kPi * i / (mFFTSize - 1));

    mWritePos = 0;
    mReadPos = 0;
    mSamplesUntilHop = mHopSize;
  }

  void SetSampleRate(double sr) { mSampleRate = sr; }

  // Pointeur vers la courbe partagee (-1 a 1) - pas copiee, juste
  // referencee le temps du traitement du bloc courant.
  void SetCurve(const float* curve, int curveSize)
  {
    mCurve = curve;
    mCurveSize = curveSize;
  }

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

  // Interpole la courbe partagee sur une echelle LOG entre 20Hz et 20kHz,
  // pour la bande FFT donnee, et convertit en gain lineaire.
  float GetGainLinearForBin(int binIdx) const
  {
    if (!mCurve || mCurveSize < 2) return 1.f;

    float freq = (float)binIdx * (float)mSampleRate / (float)mFFTSize;
    freq = std::clamp(freq, kMinFreq, kMaxFreq);
    float logPos = std::log(freq / kMinFreq) / std::log(kMaxFreq / kMinFreq); // 0..1

    float pos = logPos * (float)(mCurveSize - 1);
    int idx0 = (int)pos;
    int idx1 = std::min(idx0 + 1, mCurveSize - 1);
    float frac = pos - (float)idx0;
    float curveVal = mCurve[idx0] * (1.f - frac) + mCurve[idx1] * frac;

    float db = curveVal * kMaxDb;
    return std::pow(10.f, db / 20.f);
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
      float gain = GetGainLinearForBin(k);
      mCplx[k] *= gain;
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
  static constexpr float kMinFreq = 20.f;
  static constexpr float kMaxFreq = 20000.f;
  static constexpr float kMaxDb = 24.f; // -1..1 -> -24dB..+24dB

  int mFFTSize = 1024;
  int mOverlap = 4;
  int mHopSize = 256;
  int mSamplesUntilHop = 256;
  int mWritePos = 0;
  int mReadPos = 0;
  double mSampleRate = 44100.0;

  const float* mCurve = nullptr;
  int mCurveSize = 0;

  std::vector<float> mRing, mRingOut, mWindow, mTime;
  std::vector<cplx> mCplx;
};
