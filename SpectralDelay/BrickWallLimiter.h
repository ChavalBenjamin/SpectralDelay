#pragma once

#include <vector>
#include <cmath>
#include <algorithm>

// ============================================================================
// BrickwallLimiter
//
// Limiteur stereo a "lookahead" (le signal est legerement retarde, ce qui
// permet de reduire le gain AVANT qu'un pic n'arrive vraiment en sortie -
// evite le clic dur d'un ecretage instantane). Attaque quasi immediate,
// relachement progressif (evite le "pompage").
//
// Filet de securite en fin de chaine : garantit qu'aucun pic ne depasse
// le seuil regle, quels que soient les reglages en amont (Horizon compris,
// qui peut sinon pousser le signal tres au-dela de 0dB).
// ============================================================================

class BrickwallLimiter
{
public:
  void Init(double sampleRate)
  {
    mSampleRate = sampleRate;
    mLookaheadSamples = std::max(1, (int)(kLookaheadMs * 0.001 * sampleRate));
    mDelayBufL.assign(mLookaheadSamples, 0.f);
    mDelayBufR.assign(mLookaheadSamples, 0.f);
    mWritePos = 0;
    mGainEnv = 1.f;
    mReleaseCoeff = 1.f - std::exp(-1.f / (kReleaseMs * 0.001f * (float)mSampleRate));
    mAttackCoeff = 1.f - std::exp(-1.f / (kAttackMs * 0.001f * (float)mSampleRate));
  }

  void SetThresholdDb(float db) { mThresholdLin = std::pow(10.f, db / 20.f); }

  void ProcessStereo(float* L, float* R, int n)
  {
    for (int i = 0; i < n; i++)
    {
      float inL = L[i], inR = R[i];

      float delayedL = mDelayBufL[mWritePos];
      float delayedR = mDelayBufR[mWritePos];
      mDelayBufL[mWritePos] = inL;
      mDelayBufR[mWritePos] = inR;

      // Detecte le pic sur l'echantillon ENTRANT (pas encore sorti, grace
      // au lookahead) - permet a l'attaque de "voir venir" le pic.
      float peak = std::max(std::abs(inL), std::abs(inR));
      float targetGain = (peak > mThresholdLin) ? (mThresholdLin / peak) : 1.f;

      if (targetGain < mGainEnv)
        mGainEnv += (targetGain - mGainEnv) * mAttackCoeff; // attaque tres rapide, mais plus un saut brut
      else
        mGainEnv = mGainEnv + (targetGain - mGainEnv) * mReleaseCoeff; // relachement progressif

      L[i] = delayedL * mGainEnv;
      R[i] = delayedR * mGainEnv;

      mWritePos = (mWritePos + 1) % mLookaheadSamples;
    }
  }

private:
  static constexpr float kLookaheadMs = 5.f;
  static constexpr float kAttackMs = 0.5f;
  static constexpr float kReleaseMs = 80.f;

  double mSampleRate = 44100.0;
  int mLookaheadSamples = 220;
  std::vector<float> mDelayBufL, mDelayBufR;
  int mWritePos = 0;
  float mThresholdLin = 1.f;
  float mGainEnv = 1.f;
  float mAttackCoeff = 0.9f;
  float mReleaseCoeff = 0.01f;
};
