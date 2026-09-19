#include "SpectralDelay.h"
#include "IPlug_include_in_plug_src.h"
#include "IControls.h"
#include <cmath>
#include <string>

SpectralDelay::SpectralDelay(const InstanceInfo& info)
: iplug::Plugin(info, MakeConfig(kNumParams, 1))
{
  GetParam(kParamFFTSize)->InitEnum("FFT Size", 2, 5, "", IParam::kFlagsNone, "", "512", "1024", "2048", "4096", "8192");
  GetParam(kParamOverlap)->InitEnum("Overlap", 1, 2, "", IParam::kFlagsNone, "", "2x (50%)", "4x (75%)");
  GetParam(kParamCycles)->InitDouble("Cycles", 1., 0., 24., 0.01);
  GetParam(kParamQ)->InitPercentage("Q", 50.);
  GetParam(kParamBallade)->InitPercentage("Ballade", 0.);
  GetParam(kParamHorizon)->InitPercentage("Horizon", 50.);
  GetParam(kParamSkew)->InitDouble("Skew", 1., 0.1, 6., 0.01);
  GetParam(kParamShapeMode)->InitEnum("Forme", 0, 2, "", IParam::kFlagsNone, "", "Type", "Dessin");
  GetParam(kParamFeedback)->InitDouble("Feedback", 0., 0., 95., 0.1, "%");
  GetParam(kParamSyncMode)->InitBool("Sync BPM", false);
  GetParam(kParamDryWet)->InitDouble("Dry/Wet", 100., 0., 100., 0.1, "%");
  GetParam(kParamLimiterThreshold)->InitDouble("Limiteur", 0., -24., 0., 0.1, "dB");

  mDrawnShapeStorage.assign(128, 0.f);

#if IPLUG_EDITOR
  mMakeGraphicsFunc = [&]() {
    return MakeGraphics(*this, PLUG_WIDTH, PLUG_HEIGHT, PLUG_FPS,
                         GetScaleForScreen(PLUG_WIDTH, PLUG_HEIGHT));
  };

  mLayoutFunc = [&](IGraphics* pGraphics) {
    pGraphics->AttachCornerResizer(EUIResizerMode::Scale, false);
    pGraphics->AttachPanelBackground(COLOR_GRAY);
    pGraphics->LoadFont("Roboto-Regular", ROBOTO_FN);

    const IVStyle knobStyle = DEFAULT_STYLE.WithLabelText(IText(11.f, COLOR_WHITE));
    const IVStyle bonusStyle = DEFAULT_STYLE.WithLabelText(IText(11.f, COLOR_WHITE))
                                             .WithColor(EVColor::kFG, IColor(255, 220, 160, 60))
                                             .WithColor(EVColor::kPR, IColor(255, 240, 190, 90));

    const IRECT bounds = pGraphics->GetBounds();

    // --- Rangee du haut : reglages generaux ---
    IRECT topRow = bounds.GetFromTop(100.f).GetPadded(-10.f);
    mParamControls[kParamFFTSize] = new IVMenuButtonControl(topRow.GetGridCell(0, 0, 1, 7).GetCentredInside(110.f, 44.f), kParamFFTSize, "FFT Size");
    pGraphics->AttachControl(mParamControls[kParamFFTSize]);
    mParamControls[kParamOverlap] = new IVMenuButtonControl(topRow.GetGridCell(0, 1, 1, 7).GetCentredInside(110.f, 44.f), kParamOverlap, "Overlap");
    pGraphics->AttachControl(mParamControls[kParamOverlap]);
    mParamControls[kParamFeedback] = new IVKnobControl(topRow.GetGridCell(0, 2, 1, 7).GetCentredInside(90.f), kParamFeedback, "Feedback", bonusStyle);
    pGraphics->AttachControl(mParamControls[kParamFeedback]);
    mParamControls[kParamSyncMode] = new IVToggleControl(topRow.GetGridCell(0, 3, 1, 7).GetCentredInside(120.f, 44.f), kParamSyncMode, "Sync BPM");
    pGraphics->AttachControl(mParamControls[kParamSyncMode]);
    mParamControls[kParamDryWet] = new IVKnobControl(topRow.GetGridCell(0, 4, 1, 7).GetCentredInside(90.f), kParamDryWet, "Dry/Wet", bonusStyle);
    pGraphics->AttachControl(mParamControls[kParamDryWet]);

    IVStyle limiterStyle = DEFAULT_STYLE.WithLabelText(IText(12.f, COLOR_WHITE))
                                         .WithColor(EVColor::kFG, IColor(255, 200, 30, 30))
                                         .WithColor(EVColor::kPR, IColor(255, 230, 50, 50));
    mParamControls[kParamLimiterThreshold] = new IVKnobControl(topRow.GetGridCell(0, 5, 1, 7).GetCentredInside(90.f), kParamLimiterThreshold, "LIMITEUR", limiterStyle);
    pGraphics->AttachControl(mParamControls[kParamLimiterThreshold]);

    // --- Rangee des 6 parametres de courbe ---
    IRECT controlsRow = IRECT(bounds.L, bounds.T + 100.f, bounds.R, bounds.T + 230.f).GetPadded(-15.f);
    mParamControls[kParamCycles] = new IVKnobControl(controlsRow.GetGridCell(0, 0, 1, 6).GetCentredInside(90.f), kParamCycles, "Cycles", knobStyle);
    pGraphics->AttachControl(mParamControls[kParamCycles]);
    mParamControls[kParamQ] = new IVKnobControl(controlsRow.GetGridCell(0, 1, 1, 6).GetCentredInside(90.f), kParamQ, "Q", knobStyle);
    pGraphics->AttachControl(mParamControls[kParamQ]);
    mParamControls[kParamBallade] = new IVKnobControl(controlsRow.GetGridCell(0, 2, 1, 6).GetCentredInside(90.f), kParamBallade, "Ballade", knobStyle);
    pGraphics->AttachControl(mParamControls[kParamBallade]);
    mParamControls[kParamHorizon] = new IVKnobControl(controlsRow.GetGridCell(0, 3, 1, 6).GetCentredInside(90.f), kParamHorizon, "Horizon", knobStyle);
    pGraphics->AttachControl(mParamControls[kParamHorizon]);
    mParamControls[kParamSkew] = new IVKnobControl(controlsRow.GetGridCell(0, 4, 1, 6).GetCentredInside(90.f), kParamSkew, "Skew", knobStyle);
    pGraphics->AttachControl(mParamControls[kParamSkew]);
    mParamControls[kParamShapeMode] = new IVMenuButtonControl(controlsRow.GetGridCell(0, 5, 1, 6).GetCentredInside(120.f, 48.f), kParamShapeMode, "Forme");
    pGraphics->AttachControl(mParamControls[kParamShapeMode]);

    // --- Courbe : le reste de l'espace, aussi grand que possible ---
    IRECT curveArea = IRECT(bounds.L, bounds.T + 230.f, bounds.R, bounds.B).GetPadded(-20.f);
    mCurveView = new SpectralCurvePreviewControl(curveArea, [this](const float* data, int size) {
      mDrawnShapeStorage.assign(data, data + size);
      mEngine.SetDrawnShape(data, size);
      UpdateEngine();
    });
    pGraphics->AttachControl(mCurveView);
  };
#endif

#if IPLUG_DSP
  OnReset();
#endif
}

void SpectralDelay::OnIdle()
{
#if IPLUG_DSP
  if (mCurveView && mCurveUIUpdated.exchange(false))
  {
    mCurveView->SetCurve(mCurveUIBuf, mCurveUISize);
    mCurveView->SetDirty(false);
  }
  if (mCurveView && mSpectrumUIUpdated.exchange(false))
  {
    mCurveView->SetSpectrumData(mSpectrumUIBuf, mSpectrumUISize, mAnalyzer.GetSampleRate(), mAnalyzer.GetFFTSize());
  }

  // Rafraichit l'affichage BPM en Sync en continu (le tempo peut changer
  // pendant la lecture) - seulement quand Sync est actif, pour ne pas
  // reconstruire l'axe inutilement sinon.
  if (mCurveView && GetParam(kParamSyncMode)->Value() != 0.)
    UpdateYAxisMarks();
#endif
}

void SpectralDelay::SyncUIToState()
{
  for (int i = 0; i < kNumParams; i++)
    if (mParamControls[i])
      mParamControls[i]->SetValueFromDelegate(GetParam(i)->GetNormalized());

  if (!mCurveView) return;

  bool drawMode = (int)GetParam(kParamShapeMode)->Value() != 0;
  mCurveView->SetDrawMode(drawMode);

  if (!mDrawnShapeStorage.empty())
    mCurveView->SetDrawnShapeExternal(mDrawnShapeStorage.data(), (int)mDrawnShapeStorage.size());

#if IPLUG_DSP
  UpdateYAxisMarks();
#endif
}

void SpectralDelay::ApplyAllState()
{
#if IPLUG_DSP
  UpdateFFTConfig();
  UpdateEngine();
  UpdateYAxisMarks();
  mLimiter.Init(GetSampleRate());
  mLimiter.SetThresholdDb((float)GetParam(kParamLimiterThreshold)->Value());
  mAnalyzer.Init(GetSampleRate());
#endif
}

#if IPLUG_DSP

void SpectralDelay::UpdateFFTConfig()
{
  int fftSizeIdx = (int)GetParam(kParamFFTSize)->Value();
  int fftSize = 512 << fftSizeIdx;
  int overlapIdx = (int)GetParam(kParamOverlap)->Value();
  int overlap = (overlapIdx == 0) ? 2 : 4;

  {
    std::lock_guard<std::mutex> lock(mEngineMutex);
    mDelayL.Init(fftSize, overlap, GetSampleRate());
    mDelayR.Init(fftSize, overlap, GetSampleRate());
  }

  // Ligne a retard du signal sec, alignee EXACTEMENT sur la latence du
  // traitement - verrouillee separement (redimensionnee ici, thread
  // principal ; lue/ecrite par ProcessBlock, thread audio).
  {
    std::lock_guard<std::mutex> lock(mDryDelayMutex);
    mDryDelaySize = std::max(1, mDelayL.GetLatencySamples());
    mDryDelayL.assign(mDryDelaySize, 0.f);
    mDryDelayR.assign(mDryDelaySize, 0.f);
    mDryDelayPos = 0;
  }

  // SetLatency() REMIS : le crash precedent (projet fusionne) venait
  // probablement du mutex de securite entre threads (corrige depuis),
  // pas de cette methode - elle avait ete retiree par precaution au
  // mauvais moment. Informe l'hote (Reaper) du vrai retard, pour qu'il
  // compense sur l'ensemble de la session (PDC) - a tester avec
  // attention (changements de FFT Size pendant la lecture) malgre cette
  // conviction, vu qu'elle avait deja ete soupconnee une fois.
  SetLatency(mDryDelaySize);
}

void SpectralDelay::UpdateEngine()
{
  mEngine.SetSize(512);
  mEngine.SetShapeMode((int)GetParam(kParamShapeMode)->Value() == 0
                          ? SpectralCurveEngine::ShapeMode::Type
                          : SpectralCurveEngine::ShapeMode::Draw);
  mEngine.SetCycles((float)GetParam(kParamCycles)->Value());
  mEngine.SetQ((float)(GetParam(kParamQ)->Value() / 100.0));
  mEngine.SetBallade((float)(GetParam(kParamBallade)->Value() / 100.0));
  mEngine.SetHorizon((float)(GetParam(kParamHorizon)->Value() / 100.0));
  mEngine.SetSkew((float)GetParam(kParamSkew)->Value());
  mEngine.RebuildIfNeeded();

  const float* curve = mEngine.GetCurve();
  int size = mEngine.GetSize();

  {
    std::lock_guard<std::mutex> lock(mCurveMutex);
    mSharedCurve.assign(curve, curve + size);
  }
  mCurveUISize = size;
  for (int i = 0; i < size; i++) mCurveUIBuf[i] = curve[i];
  mCurveUIUpdated.store(true);
}

void SpectralDelay::UpdateYAxisMarks()
{
  if (!mCurveView) return;

  std::vector<SpectralCurvePreviewControl::AxisMark> marks;

  bool syncOn = GetParam(kParamSyncMode)->Value() != 0.;
  if (syncOn)
  {
    // Grille complete : 1/4 a 1/64, binaire ET ternaire, reactive au
    // BPM en direct (le tempo peut changer pendant la lecture).
    double bpm = mLastBPM.load();
    double quarterMs = 60000.0 / std::max(bpm, 1.0);

    struct Division { const char* name; double divisor; };
    static const Division kDivisions[5] = {
      { "1/4", 1.0 }, { "1/8", 2.0 }, { "1/16", 4.0 }, { "1/32", 8.0 }, { "1/64", 16.0 }
    };

    for (const auto& div : kDivisions)
    {
      double binaryMs = quarterMs / div.divisor;
      float axisVal = mDelayL.MsToAxisValue((float)binaryMs);
      char buf[24];
      snprintf(buf, sizeof(buf), "%s = %.0fms", div.name, binaryMs);
      marks.push_back({ axisVal, buf, div.divisor == 1.0 });

      double ternaryMs = binaryMs * 2.0 / 3.0;
      float ternaryAxisVal = mDelayL.MsToAxisValue((float)ternaryMs);
      char bufT[24];
      snprintf(bufT, sizeof(bufT), "%sT = %.0fms", div.name, ternaryMs);
      marks.push_back({ ternaryAxisVal, bufT, false });
    }
  }
  else
  {
    marks.push_back({ -1.f, "0 ms", false });
    marks.push_back({ 0.f, "1.25 s", true });
    marks.push_back({ 1.f, "2.5 s", false });
  }

  // Repere ROUGE : seuil reel en dessous duquel une bande reste en
  // passthrough total (aucun delai) - grandit avec la taille FFT/
  // Overlap, utile pour se reperer surtout en mode Dessin libre.
  float dangerAxisVal = mDelayL.GetEffectiveMinDelayAxisValue();
  char bufDanger[48];
  snprintf(bufDanger, sizeof(bufDanger), "min effectif: %.1fms", mDelayL.GetEffectiveMinDelayMs());
  marks.push_back({ dangerAxisVal, bufDanger, false, true });

  mCurveView->SetYAxisMarks(marks);
}

void SpectralDelay::OnReset()
{
  ApplyAllState();
}

void SpectralDelay::OnParamChange(int paramIdx)
{
  switch (paramIdx)
  {
    case kParamFFTSize:
    case kParamOverlap:
      UpdateFFTConfig();
      UpdateYAxisMarks();
      break;

    case kParamShapeMode:
      if (mCurveView) mCurveView->SetDrawMode((int)GetParam(kParamShapeMode)->Value() != 0);
      UpdateEngine();
      break;

    case kParamCycles:
    case kParamQ:
    case kParamBallade:
    case kParamHorizon:
    case kParamSkew:
      UpdateEngine();
      break;

    case kParamSyncMode:
      UpdateYAxisMarks();
      break;

    case kParamLimiterThreshold:
      mLimiter.SetThresholdDb((float)GetParam(kParamLimiterThreshold)->Value());
      break;

    default:
      break;
  }
}

void SpectralDelay::ProcessBlock(sample** inputs, sample** outputs, int nFrames)
{
  static float bufL[8192], bufR[8192], outL[8192], outR[8192];
  int n = std::min(nFrames, 8192);

  for (int i = 0; i < n; i++) { bufL[i] = (float)inputs[0][i]; bufR[i] = (float)inputs[1][i]; }

  static float bufMix[8192];
  for (int i = 0; i < n; i++) bufMix[i] = (bufL[i] + bufR[i]) * 0.5f;
  mAnalyzer.Process(bufMix, n);
  {
    std::lock_guard<std::mutex> lock(mSpectrumMutex);
    int numBins = std::min(mAnalyzer.GetNumBins(), 1100);
    for (int i = 0; i < numBins; i++) mSpectrumUIBuf[i] = mAnalyzer.GetMagnitudeDb()[i];
    mSpectrumUISize = numBins;
  }
  mSpectrumUIUpdated.store(true);

  float feedback = (float)(GetParam(kParamFeedback)->Value() / 100.0);
  bool syncMode = GetParam(kParamSyncMode)->Value() != 0.;
  double bpm = GetTempo(); // confirmee fonctionnelle (utilisee dans MagniPhase)
  mLastBPM.store(bpm);

  {
    std::lock_guard<std::mutex> curveLock(mCurveMutex);
    std::lock_guard<std::mutex> engineLock(mEngineMutex);

    mDelayL.SetCurve(mSharedCurve.data(), (int)mSharedCurve.size());
    mDelayR.SetCurve(mSharedCurve.data(), (int)mSharedCurve.size());
    mDelayL.SetFeedback(feedback); mDelayR.SetFeedback(feedback);
    mDelayL.SetSyncMode(syncMode); mDelayR.SetSyncMode(syncMode);
    mDelayL.SetBPM(bpm); mDelayR.SetBPM(bpm);

    mDelayL.Process(bufL, outL, n);
    mDelayR.Process(bufR, outR, n);
  }

  // Dry/Wet : le signal sec passe par sa PROPRE ligne a retard (alignee
  // sur la latence du traitement) avant d'etre melange - necessaire
  // maintenant que le moteur produit un vrai Wet isole (plus d'entree
  // directe melangee automatiquement dans le traitement lui-meme).
  {
    float wetRaw = (float)(GetParam(kParamDryWet)->Value() / 100.0);
    constexpr float kExponent = 3.f;
    float wetAmount = std::pow(wetRaw, kExponent);

    std::lock_guard<std::mutex> lock(mDryDelayMutex);
    for (int i = 0; i < n; i++)
    {
      float dryL = mDryDelayL[mDryDelayPos];
      float dryR = mDryDelayR[mDryDelayPos];
      mDryDelayL[mDryDelayPos] = bufL[i];
      mDryDelayR[mDryDelayPos] = bufR[i];
      mDryDelayPos = (mDryDelayPos + 1) % mDryDelaySize;

      outL[i] = dryL * (1.f - wetAmount) + outL[i] * wetAmount;
      outR[i] = dryR * (1.f - wetAmount) + outR[i] * wetAmount;
    }
  }

  mLimiter.ProcessStereo(outL, outR, n);

  for (int i = 0; i < n; i++) { outputs[0][i] = outL[i]; outputs[1][i] = outR[i]; }
}

#endif
