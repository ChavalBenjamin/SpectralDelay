#pragma once

#include "IPlug_include_in_plug_hdr.h"
#include "SpectralCurveEngine.h"
#include "SpectralCurvePreviewControl.h"
#include "SpectralMagnitudeDistortEngine.h"
#include "BrickwallLimiter.h"
#include "SpectrumAnalyzer.h"
#include <atomic>
#include <mutex>

// ============================================================================
// Etape 5bis : Distorsion en magnitude - nouvelle_magnitude = magnitude ^
// exposant, par bande, exposant pilote par la courbe partagee. Continu (pas
// de seuil), compensation de gain. Stereo.
// ============================================================================

enum EParams
{
  kParamFFTSize = 0,   // 0=512, 1=1024, 2=2048, 3=4096, 4=8192
  kParamOverlap,       // 0=2x, 1=4x
  kParamCycles,
  kParamQ,
  kParamBallade,
  kParamHorizon,
  kParamSkew,
  kParamShapeMode,        // 0 = Type (sinus), 1 = Dessin libre
  kParamHarmonicInjection, // 0-100% : injection harmonique (x2/x3/x4)
  kParamDecayExponent,    // 0.2-1.0 : decroissance de l'injection (1 = actuel, 0.2 = presque plat)
  kParamTempDrive,        // 0-100% : distorsion temporelle (waveshaping)
  kParamDryWet,           // 0-100% : melange signal sec (retarde) / traite
  kParamLimiterThreshold, // dB - seuil du limiteur Brickwall final (securite)
  kNumParams
};

using namespace iplug;
using namespace igraphics;

class SpectralDelay final : public iplug::Plugin
{
public:
  SpectralDelay(const InstanceInfo& info);

  void OnIdle() override;
  void OnUIOpen() override { SyncUIToState(); }
  void OnUIClose() override { mCurveView = nullptr; for (auto& c : mParamControls) c = nullptr; }

  // SerializeState/UnserializeState RETIRES : provoquaient un crash grave
  // de Reaper (VST3_SaveState, allocation memoire demesuree) - la
  // sauvegarde du dessin sur disque est desactivee pour l'instant,
  // a reprendre plus tard avec une approche plus prudente.

#if IPLUG_DSP
  void ProcessBlock(sample** inputs, sample** outputs, int nFrames) override;
  void OnParamChange(int paramIdx) override;
  void OnReset() override;
#endif

private:
  SpectralCurvePreviewControl* mCurveView = nullptr;
  IControl* mParamControls[kNumParams] = { nullptr };

  std::vector<float> mDrawnShapeStorage;

  void ApplyAllState();
  void SyncUIToState();

#if IPLUG_DSP
  void UpdateFFTConfig();
  void UpdateEngine();
  void UpdateYAxisMarks();

  SpectralCurveEngine mEngine;
  SpectralMagnitudeDistortEngine mDistortL, mDistortR;
  BrickwallLimiter mLimiter;
  SpectrumAnalyzer mAnalyzer;

  // Ligne a retard pour le signal SEC, alignee sur la latence du
  // traitement (environ une fenetre FFT) - sans ca, melanger sec (instantane)
  // et traite (retarde) creerait un decalage temporel audible.
  std::vector<float> mDryDelayL, mDryDelayR;
  int mDryDelayPos = 0;
  int mDryDelaySize = 1;

  std::mutex mCurveMutex;
  std::vector<float> mSharedCurve;

  std::mutex mSpectrumMutex;
  std::atomic<bool> mSpectrumUIUpdated { false };
  float mSpectrumUIBuf[1100] = { -80.f };
  int mSpectrumUISize = 0;

  std::atomic<bool> mCurveUIUpdated { false };
  float mCurveUIBuf[512] = { 0.f };
  int mCurveUISize = 0;
#endif
};
