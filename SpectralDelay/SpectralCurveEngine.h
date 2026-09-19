#pragma once

#include <vector>
#include <cmath>
#include <algorithm>

// ============================================================================
// SpectralCurveEngine
//
// Moteur de courbe partage, reutilise par les 3 modules (Filtre, Delay,
// Inverse Comp) des "Operateurs Spectraux". Genere une courbe 2D
// (spectre en X, valeur en Y, -1 a 1) a partir de 5 parametres, appliques
// a une forme de base qui peut etre soit un sinus (mode Type), soit un
// cycle dessine a la souris (mode Draw) - les 5 parametres agissent de
// la meme facon dans les deux cas :
//
//  - Cycles  : nombre de repetitions de la forme a travers le spectre (jusqu'a 24)
//  - Q       : 0 = plat, milieu = forme pleine, max = notchs etroits
//  - Ballade : phase (0..1 = un tour complet) - boucle proprement pour un LFO
//  - Horizon : asymetrie - 0.5 = symetrique, vers 0 = ne garde que les
//              bosses positives, vers 1 = ne garde que les creux negatifs
//              (avec decalage +1 progressif au-dela de 50%)
//  - Skew    : redistribue la POSITION des cycles sur l'axe X (pas leur
//              hauteur) - resserre d'un cote, etire de l'autre
// ============================================================================

class SpectralCurveEngine
{
public:
  enum class ShapeMode { Type, Draw };

  void SetSize(int numPoints)
  {
    numPoints = std::max(2, numPoints);
    // "|| mCurve.empty()" est essentiel : sans ca, le tout premier appel
    // (quand numPoints correspond deja a la valeur par defaut du membre)
    // ne redimensionne jamais le vecteur, qui reste vide alors que le
    // reste du code croit qu'il contient mNumPoints elements - ecriture
    // hors limites garantie au premier RebuildIfNeeded().
    if (numPoints != mNumPoints || mCurve.empty())
    {
      mNumPoints = numPoints;
      mCurve.assign(mNumPoints, 0.f);
      mDirty = true;
    }
  }

  void SetShapeMode(ShapeMode mode)
  {
    if (mode != mShapeMode) { mShapeMode = mode; mDirty = true; }
  }

  // Fournit un cycle dessine a la souris (valeurs -1 a 1, resolution
  // libre). Recalcule immediatement le raccord de boucle en fonction de
  // Cycles actuel.
  void SetDrawnShape(const float* data, int size)
  {
    mDrawnShapeRaw.assign(data, data + size);
    RebuildDrawnShapeForLoop();
    mDirty = true;
  }

  void SetCycles(float cycles)
  {
    float clamped = std::clamp(cycles, 0.f, 24.f);
    bool crossedOneThreshold = (clamped > 1.0001f) != (mCycles > 1.0001f);
    SetIfChanged(mCycles, clamped);
    // Le raccord de boucle n'a de sens que si la forme se repete
    // (Cycles > 1) - on ne recalcule que si on vient de franchir ce seuil.
    if (crossedOneThreshold) RebuildDrawnShapeForLoop();
  }

  void SetQ(float q) { SetIfChanged(mQ, std::clamp(q, 0.f, 1.f)); }
  void SetBallade(float ballade) { SetIfChanged(mBallade, std::clamp(ballade, 0.f, 1.f)); }
  void SetHorizon(float horizon) { SetIfChanged(mHorizon, std::clamp(horizon, 0.f, 1.f)); }
  void SetSkew(float skew) { SetIfChanged(mSkew, std::clamp(skew, 0.1f, 6.f)); }

  const float* GetCurve() const { return mCurve.data(); }
  int GetSize() const { return mNumPoints; }

  void RebuildIfNeeded()
  {
    if (!mDirty) return;
    mDirty = false;

    for (int i = 0; i < mNumPoints; i++)
    {
      float x = (float)i / (float)(mNumPoints - 1); // 0..1

      // 0. Skew : redistribue la POSITION des cycles sur l'axe X (pas leur
      // hauteur) - resserre les cycles d'un cote du spectre, les etire de
      // l'autre. Skew=1 = lineaire (neutre), <1 et >1 divergent dans des
      // sens opposes. Applique AVANT le calcul de phase, sur x directement.
      float xWarped = std::pow(x, mSkew);

      // 1. Forme de base - "tours" (pas radians) pilotes par Cycles et
      // Ballade, boucle proprement (Ballade parcourt exactement un tour
      // complet, 0 a 1). phaseFraction (0..1) se repete a chaque cycle,
      // que la forme soit un sinus ou un dessin.
      float rawTurns = mCycles * xWarped + mBallade;
      float phaseFraction = rawTurns - std::floor(rawTurns); // 0..1
      float s = GetBaseShape(phaseFraction);

      // 2. Q : deux regimes - 0..milieu monte l'amplitude (plat -> forme
      // pleine), milieu..max resserre les pics (forme pleine -> notchs).
      float y;
      if (mQ <= 0.5f)
      {
        float amt = mQ / 0.5f;
        y = s * amt;
      }
      else
      {
        float amt = (mQ - 0.5f) / 0.5f;
        float power = 1.f + amt * 11.f; // durete croissante des notchs
        float signS = (s >= 0.f) ? 1.f : -1.f;
        y = signS * std::pow(std::abs(s), power); // pointe (pas 1/power, qui aplatissait)
      }

      // 3. Horizon : asymetrie - 0.5 = symetrique (rien ne change).
      float posGain, negGain;
      float shiftUp = 0.f; // decalage progressif +1 au-dela de 50% (voir plus bas)
      if (mHorizon <= 0.5f)
      {
        posGain = 1.f;
        negGain = mHorizon / 0.5f;
      }
      else
      {
        negGain = 1.f;
        posGain = (1.f - mHorizon) / 0.5f;
        // Au-dela de 50%, decale progressivement vers le haut (0 a
        // horizon=0.5, +1 complet a horizon=1) - a l'extreme, les creux
        // negatifs (-1 a 0) deviennent des bosses positives (0 a +1),
        // sans changer leur forme (toujours pilotee par Q).
        shiftUp = (mHorizon - 0.5f) / 0.5f;
      }
      y *= (y >= 0.f) ? posGain : negGain;
      y += shiftUp;

      mCurve[i] = y;
    }
  }

private:
  // Renvoie la forme de base pour une fraction de cycle (0..1, se repete
  // a chaque cycle) - soit un sinus (mode Type), soit interpole dans le
  // cycle dessine a la souris (mode Draw).
  float GetBaseShape(float phaseFraction) const
  {
    if (mShapeMode == ShapeMode::Type || mDrawnShape.empty())
      return std::sin(2.f * kPi * phaseFraction);

    float pos = phaseFraction * (float)(mDrawnShape.size() - 1);
    int idx0 = (int)pos;
    int idx1 = std::min(idx0 + 1, (int)mDrawnShape.size() - 1);
    float frac = pos - (float)idx0;
    return mDrawnShape[idx0] * (1.f - frac) + mDrawnShape[idx1] * frac;
  }

  // Recopie la forme dessinee brute, en lissant le raccord fin->debut
  // UNIQUEMENT si Cycles > 1 (la forme va etre repetee - sans ca, un saut
  // audible/visuel apparaitrait a chaque jonction). Si Cycles == 1, la
  // forme brute est utilisee telle quelle, sans aucune retouche.
  void RebuildDrawnShapeForLoop()
  {
    mDrawnShape = mDrawnShapeRaw;
    int n = (int)mDrawnShape.size();
    if (mCycles > 1.0001f && n > 4)
    {
      int fadeLen = std::max(2, n / 10);
      float startVal = mDrawnShape[0];
      for (int i = 0; i < fadeLen; i++)
      {
        float t = (float)i / (float)(fadeLen - 1); // 0..1
        int idx = n - fadeLen + i;
        mDrawnShape[idx] = mDrawnShape[idx] * (1.f - t) + startVal * t;
      }
    }
  }

  void SetIfChanged(float& member, float value)
  {
    if (value != member) { member = value; mDirty = true; }
  }

  static constexpr float kPi = 3.14159265358979323846f;

  int mNumPoints = 512;
  float mCycles = 1.f;
  float mQ = 0.5f;       // forme pleine par defaut
  float mBallade = 0.f;
  float mHorizon = 0.5f; // symetrique par defaut
  float mSkew = 1.f;     // neutre par defaut

  ShapeMode mShapeMode = ShapeMode::Type;
  std::vector<float> mDrawnShapeRaw;  // telle que dessinee, jamais modifiee
  std::vector<float> mDrawnShape;     // copie utilisee, avec raccord si besoin

  bool mDirty = true;
  std::vector<float> mCurve;
};
