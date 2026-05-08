/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     EncAdaptiveLoopFilterBaseDefs.h
 \brief    Class templates for ALF encoder (definitions).
 */
#include "EncAdaptiveLoopFilterBase.h"
#if defined(TARGET_SIMD_X86) && ENABLE_SIMD_OPT_ALF
#include "CommonLib/x86/CommonDefX86.h"
#endif

template<class ParamsT> template<bool useEnableLessClip> float
  AlfCovarianceBase<ParamsT>::optimizeFilterClip(const int size, int *clip, const bool enableLessClip /*= false*/) const
{
  Ty f;
  if (useEnableLessClip)
  {
    return optimizeFilter<true>(size, clip, f, true, enableLessClip);
  }
  else
  {
    return optimizeFilter<false>(size, clip, f, true);
  }
}

template<class ParamsT> template<bool useEnableLessClip>
float AlfCovarianceBase<ParamsT>::optimizeFilter(const int size, int *clip, float *f, bool optimizeClip,
                                                 const bool enableLessClip /*= false*/) const
{
  const int copySize = sizeof(float) * size;
  float     fBest[MAX_NUM_ALF_LUMA_COEFF];
  int       clipMax[MAX_NUM_ALF_LUMA_COEFF];

  float errBest, errLast;

  TE kE;
  Ty ky;

  if (optimizeClip)
  {
    // Start by looking for min clipping that has no impact => max_clipping
    getClipMax(clipMax);
    for (int k = 0; k < size; ++k)
    {
      clip[k] = std::max(clipMax[k], clip[k]);
      clip[k] = std::min(clip[k], numBins - 1);
    }
  }

  setEyFromClip(clip, kE, ky, size);

  gnsSolveByChol(kE, ky, f, size);
  errBest = calculateError(clip, f, size);
  memcpy(fBest, f, copySize);
  int step = optimizeClip ? (numBins + 1) / 2 : 0;

  int iter = 0;
  while ((useEnableLessClip && enableLessClip) ? ((step > 0) && (++iter <= 32)) : (step > 0))
  {
    float errMin = errBest;
    int   idxMin = -1;
    int   incMin = 0;

    for (int k = 0; k < size - 1; ++k)
    {
      if (clip[k] - step >= clipMax[k])
      {
        clip[k] -= step;
        ky[k] = y(clip[k], k);
        for (int l = 0; l < size; l++)
        {
          kE[k][l] = E(clip[k], clip[l], k, l);
          kE[l][k] = E(clip[l], clip[k], l, k);
        }

        gnsSolveByChol(kE, ky, f, size);
        errLast = calculateError(clip, f, size);

        if (errLast < errMin)
        {
          errMin = errLast;
          idxMin = k;
          incMin = -step;
          memcpy(fBest, f, copySize);
        }
        clip[k] += step;
      }
      if (clip[k] + step < numBins)
      {
        clip[k] += step;
        ky[k] = y(clip[k], k);
        for (int l = 0; l < size; l++)
        {
          kE[k][l] = E(clip[k], clip[l], k, l);
          kE[l][k] = E(clip[l], clip[k], l, k);
        }

        gnsSolveByChol(kE, ky, f, size);
        errLast = calculateError(clip, f, size);

        if (errLast < errMin)
        {
          errMin = errLast;
          idxMin = k;
          incMin = step;
          memcpy(fBest, f, copySize);
        }
        clip[k] -= step;
      }
      ky[k] = y(clip[k], k);
      for (int l = 0; l < size; l++)
      {
        kE[k][l] = E(clip[k], clip[l], k, l);
        kE[l][k] = E(clip[l], clip[k], l, k);
      }
    }

    if (idxMin >= 0)
    {
      errBest = errMin;
      clip[idxMin] += incMin;
      ky[idxMin] = y(clip[idxMin], idxMin);
      for (int l = 0; l < size; l++)
      {
        kE[idxMin][l] = E(clip[idxMin], clip[l], idxMin, l);
        kE[l][idxMin] = E(clip[l], clip[idxMin], l, idxMin);
      }
    }
    else
    {
      --step;
    }
  }

  if (optimizeClip)
  {
    // test all max
    for (int k = 0; k < size - 1; ++k)
    {
      clipMax[k] = 0;
    }
    TE kEMax;
    Ty kyMax;
    setEyFromClip(clipMax, kEMax, kyMax, size);

    gnsSolveByChol(kEMax, kyMax, f, size);
    errLast = calculateError(clipMax, f, size);
    if (errLast < errBest)
    {
      errBest = errLast;
      for (int k = 0; k < size; ++k)
      {
        clip[k] = clipMax[k];
      }
    }
    else
    {
      // update clip to reduce coding cost
      reduceClipCost(clip);

      // update f with best solution
      memcpy(f, fBest, copySize);
    }
  }

  return errBest;
}

template<class ParamsT> float AlfCovarianceBase<ParamsT>::calculateError(const int *clip) const
{
  Ty c;

  return optimizeFilter(clip, c, numCoeff);
}

//********************************
// Cholesky decomposition
//********************************

// Find filter coeff related
template<class ParamsT> int AlfCovarianceBase<ParamsT>::gnsCholeskyDec(TE inpMatr, TE outMatr, int numEq) const
{
  for (int i = 0; i < numEq; i++)
  {
    float scale = inpMatr[i][i];

    for (int k = 0; k < i; k++)
    {
      scale -= outMatr[k][i] * outMatr[k][i];
    }

    if (scale <= REG_SQR)   // inpMatr is singular
    {
      return 0;
    }

    outMatr[i][i] = sqrt(scale);
    float tmp     = 1 / outMatr[i][i];

    int j = i + 1;
#if defined(TARGET_SIMD_X86) && ENABLE_SIMD_OPT_ALF
    bool useSimd = read_x86_extension_flags() > SCALAR;
    if (useSimd)
    {
      for (; j + 4 < numEq; j += 4)
      {
        __m128 scale = _mm_loadu_ps(inpMatr[i] + j);
        for (int k = 0; k < i; k++)
        {
          __m128 outMatrJ = _mm_loadu_ps(outMatr[k] + j);
          __m128 outMatrI = _mm_set1_ps(outMatr[k][i]);
          scale           = _mm_sub_ps(scale, _mm_mul_ps(outMatrJ, outMatrI));
        }
        scale = _mm_mul_ps(scale, _mm_set1_ps(tmp));
        _mm_storeu_ps(outMatr[i] + j, scale); // Upper triangular
        /*for (int addToJ = 0; addToJ < 4; addToJ++)
        {
          outMatr[j + addToJ][i] = 0.0;      // Lower triangular part
        }*/
      }
    }
#endif
    for (; j < numEq; ++j)
    {
      scale = inpMatr[i][j];
      for (int k = 0; k < i; k++)
      {
        scale -= outMatr[k][j] * outMatr[k][i];
      }

      outMatr[i][j] = scale * tmp;   // Upper triangular
      // outMatr[j][i] = 0.0;      // Lower triangular part
    }
  }

  return 1;   // Signal that Cholesky factorization is successfully performed
}

template<class ParamsT>
void AlfCovarianceBase<ParamsT>::gnsTransposeBacksubstitution(TE U, float *rhs, float *x, int order) const
{
  /* Backsubstitution starts */
  x[0] = rhs[0] / U[0][0]; /* First row of U'                   */
  for (int i = 1; i < order; i++)
  { /* For the rows 1..order-1           */

    float sum = 0;   // Holds backsubstitution from already handled rows

    for (int j = 0; j < i; j++) /* Backsubst already solved unknowns */
    {
      sum += x[j] * U[j][i];
    }

    x[i] = (rhs[i] - sum) / U[i][i]; /* i'th component of solution vect.  */
  }
}

template<class ParamsT> void AlfCovarianceBase<ParamsT>::gnsBacksubstitution(TE R, float *z, int size, float *A) const
{
  size--;
  A[size] = z[size] / R[size][size];

  for (int i = size - 1; i >= 0; i--)
  {
    float sum = 0;

    for (int j = i + 1; j <= size; j++)
    {
      sum += R[i][j] * A[j];
    }

    A[i] = (z[i] - sum) / R[i][i];
  }
}

template<class ParamsT> int AlfCovarianceBase<ParamsT>::gnsSolveByChol(const int *clip, float *x, int numEq) const
{
  TE LHS;
  Ty rhs;

  setEyFromClip(clip, LHS, rhs, numEq);
  return gnsSolveByChol(LHS, rhs, x, numEq);
}

template<class ParamsT> int AlfCovarianceBase<ParamsT>::gnsSolveByChol(TE LHS, float *rhs, float *x, int numEq) const
{
  Ty aux; /* Auxiliary vector */
  TE U;   /* Upper triangular Cholesky factor of LHS */

  int res = 1;   // Signal that Cholesky factorization is successfully performed

  /* The equation to be solved is LHSx = rhs */

  /* Compute upper triangular U such that U'*U = LHS */

  if (gnsCholeskyDec(LHS, U, numEq)) /* If Cholesky decomposition has been successful */
  {
    /* Now, the equation is  U'*U*x = rhs, where U is upper triangular
     * Solve U'*aux = rhs for aux
     */
    gnsTransposeBacksubstitution(U, rhs, aux, numEq);

    /* The equation is now U*x = aux, solve it for x (new motion coefficients) */
    gnsBacksubstitution(U, aux, numEq, x);
  }
  else /* LHS was singular */
  {
    res = 0;

    /* Regularize LHS */
    for (int i = 0; i < numEq; i++)
    {
      LHS[i][i] += REG;
    }

    /* Compute upper triangular U such that U'*U = regularized LHS */
    res = gnsCholeskyDec(LHS, U, numEq);

    if (!res)
    {
      std::memset(x, 0, sizeof(float) * numEq);
      return 0;
    }

    /* Solve  U'*aux = rhs for aux */
    gnsTransposeBacksubstitution(U, rhs, aux, numEq);

    /* Solve U*x = aux for x */
    gnsBacksubstitution(U, aux, numEq, x);
  }
  return res;
}

struct FilterIdxCount
{
  uint64_t count;
  uint8_t  filterIdx;
};

inline bool compareCounts(FilterIdxCount a, FilterIdxCount b) { return a.count > b.count; }

template<class ParamsT>
void EncAdaptiveLoopFilterBase<ParamsT>::roundFiltCoeff(int *const filterCoeffQuant, const float *const filterCoeff,
                                                        const int numCoeff, const int factor) const
{
  static_assert(ParamsT::MAX_NUM_ALF_LUMA_COEFF > ParamsT::MAX_NUM_ALF_CHROMA_COEFF,
                "Expecting the more coefficients for luma than for chroma.");
  const bool isLumaFilter = numCoeff > ParamsT::MAX_NUM_ALF_CHROMA_COEFF ? 1 : 0;
  double     alfStrength  = isLumaFilter ? m_encCfg->m_alfStrengthLuma : m_encCfg->m_alfStrengthChroma;
  for (int i = 0; i < numCoeff; i++)
  {
    const int sign      = sgn2(filterCoeff[i]);
    filterCoeffQuant[i] = int((filterCoeff[i] * alfStrength) * sign * factor + 0.5) * sign;
  }
}

template<class ParamsT> int EncAdaptiveLoopFilterBase<ParamsT>::getCoeffRateCcAlf(
  short chromaCoeff[ParamsT::MAX_NUM_CC_ALF_FILTERS][ParamsT::MAX_NUM_CC_ALF_CHROMA_COEFF],
  bool filterEnabled[ParamsT::MAX_NUM_CC_ALF_FILTERS], uint8_t filterCount, CompID compID) const
{
  int bits = 0;

  if (filterCount > 0)
  {
    bits += lengthUvlc(filterCount - 1);
    int signaledFilterCount = 0;
    for (int filterIdx = 0; filterIdx < ParamsT::MAX_NUM_CC_ALF_FILTERS; filterIdx++)
    {
      if (filterEnabled[filterIdx])
      {
        typename ParamsT::AlfFilterShape alfShape(ParamsT::AlfFilterType::CC_ALF);
        // Filter coefficients
        for (int i = 0; i < alfShape.numCoeff - 1; i++)
        {
          bits += CCALF_BITS_PER_COEFF_LEVEL + (chromaCoeff[filterIdx][i] == 0 ? 0 : 1);
        }

        signaledFilterCount++;
      }
    }
    CHECK(signaledFilterCount != filterCount, "Number of filter signaled not same as indicated");
  }

  return bits;
}

template<class ParamsT> void EncAdaptiveLoopFilterBase<ParamsT>::deriveCcAlfFilterCoeff(
  CompID compID, const PelUnitBuf &recYuv, const PelUnitBuf &recYuvExt,
  short filterCoeff[ParamsT::MAX_NUM_CC_ALF_FILTERS][ParamsT::MAX_NUM_CC_ALF_CHROMA_COEFF], const uint8_t filterIdx,
  const AlfCovarianceBase<ParamsT> &alfCovarianceFrameCcAlf, const int numCoeff) const
{
  int forwardTab[CCALF_CANDS_COEFF_NR * 2 - 1] = { 0 };
  for (int i = 0; i < CCALF_CANDS_COEFF_NR; i++)
  {
    forwardTab[CCALF_CANDS_COEFF_NR - 1 + i] = CCALF_SMALL_TAB[i];
    forwardTab[CCALF_CANDS_COEFF_NR - 1 - i] = (-1) * CCALF_SMALL_TAB[i];
  }
  using TE = float[ParamsT::MAX_NUM_ALF_LUMA_COEFF][ParamsT::MAX_NUM_ALF_LUMA_COEFF];
  using Ty = float[ParamsT::MAX_NUM_ALF_LUMA_COEFF];

  float   filterCoeffDbl[ParamsT::MAX_NUM_CC_ALF_CHROMA_COEFF];
  int16_t filterCoeffInt[ParamsT::MAX_NUM_CC_ALF_CHROMA_COEFF];

  std::fill_n(filterCoeffInt, ParamsT::MAX_NUM_CC_ALF_CHROMA_COEFF, 0);

  TE        kE;
  Ty        ky;
  const int size = numCoeff - 1;

  for (int k = 0; k < size; k++)
  {
    ky[k] = alfCovarianceFrameCcAlf.y(0, k);
    for (int l = 0; l < size; l++)
    {
      kE[k][l] = alfCovarianceFrameCcAlf.E(0, 0, k, l);
    }
  }

  alfCovarianceFrameCcAlf.gnsSolveByChol(kE, ky, filterCoeffDbl, size);
  roundFiltCoeffCCALF(filterCoeffInt, filterCoeffDbl, size, 1 << COEFF_SCALE_BITS_CCALF);

  for (int k = 0; k < size; k++)
  {
    CHECK(filterCoeffInt[k] < -(1 << CCALF_DYNAMIC_RANGE),
          "this is not possible: filterCoeffInt[k] <  -(1 << CCALF_DYNAMIC_RANGE)");
    CHECK(filterCoeffInt[k] > (1 << CCALF_DYNAMIC_RANGE),
          "this is not possible: filterCoeffInt[k] >  (1 << CCALF_DYNAMIC_RANGE)");
  }

  // Refine quanitzation
  int modified = 1;
  if (m_encCfg->m_ccalfStrength != 1.0)
  {
    modified = 0;
  }
  float errRef = alfCovarianceFrameCcAlf.calcErrorForCcAlfCoeffs(filterCoeffInt, size, COEFF_SCALE_BITS_CCALF + 1);

  while (modified)
  {
    modified = 0;
    for (int delta: { 1, -1 })
    {
      float errMin   = MAX_FLOAT;
      int   idxMin   = -1;
      int   minIndex = -1;

      for (int k = 0; k < size; k++)
      {
        int orgIdx = -1;
        for (int i = 0; i < CCALF_CANDS_COEFF_NR * 2 - 1; i++)
        {
          if (forwardTab[i] == filterCoeffInt[k])
          {
            orgIdx = i;
            break;
          }
        }
        CHECK(orgIdx < 0, "this is wrong, does not find coeff from forward_tab");
        if ((orgIdx - delta < 0) || (orgIdx - delta >= CCALF_CANDS_COEFF_NR * 2 - 1))
        {
          continue;
        }

        filterCoeffInt[k] = forwardTab[orgIdx - delta];

        float error = alfCovarianceFrameCcAlf.calcErrorForCcAlfCoeffs(filterCoeffInt, size, COEFF_SCALE_BITS_CCALF + 1);
        if (error < errMin)
        {
          errMin   = error;
          idxMin   = k;
          minIndex = orgIdx;
        }
        filterCoeffInt[k] = forwardTab[orgIdx];
      }
      if (errMin < errRef)
      {
        minIndex -= delta;
        CHECK(minIndex < 0, "this is wrong, index - delta < 0");
        CHECK(minIndex >= CCALF_CANDS_COEFF_NR * 2 - 1, "this is wrong, index - delta >= CCALF_CANDS_COEFF_NR * 2 - 1");
        filterCoeffInt[idxMin] = forwardTab[minIndex];
        modified++;
        errRef = errMin;
      }
    }
  }

  for (int k = 0; k < (size + 1); k++)
  {
    CHECK((filterCoeffInt[k] < -(1 << CCALF_DYNAMIC_RANGE)) || (filterCoeffInt[k] > (1 << CCALF_DYNAMIC_RANGE)),
          "Exceeded valid range for CC ALF coefficient");
    filterCoeff[filterIdx][k] = filterCoeffInt[k];
  }
}

template<class ParamsT> void EncAdaptiveLoopFilterBase<ParamsT>::determineControlIdcValues(
  CodingStructure &cs, const CompID compID, const PelBuf *buf, const int ctuWidthC, const int ctuHeightC,
  const int picWidthC, const int picHeightC, float **unfilteredDistortion,
  uint64_t *trainingDistortion[ParamsT::MAX_NUM_CC_ALF_FILTERS], uint64_t *lumaSwingGreaterThanThresholdCount,
  uint64_t *chromaSampleCountNearMidPoint, bool reuseTemporalFilterCoeff, uint8_t *trainingCovControl,
  uint8_t *filterControl, uint64_t &curTotalDistortion, float &curTotalRate,
  bool    filterEnabled[ParamsT::MAX_NUM_CC_ALF_FILTERS],
  uint8_t mapFilterIdxToFilterIdc[ParamsT::MAX_NUM_CC_ALF_FILTERS + 1], uint8_t &ccAlfFilterCount)
{
  constexpr auto &MAX_NUM_CC_ALF_FILTERS = ParamsT::MAX_NUM_CC_ALF_FILTERS;

  bool curFilterEnabled[MAX_NUM_CC_ALF_FILTERS];
  std::fill_n(curFilterEnabled, MAX_NUM_CC_ALF_FILTERS, false);

  FilterIdxCount filterIdxCount[MAX_NUM_CC_ALF_FILTERS];
  for (int i = 0; i < MAX_NUM_CC_ALF_FILTERS; i++)
  {
    filterIdxCount[i].count     = 0;
    filterIdxCount[i].filterIdx = i;
  }

  float prevRate = curTotalRate;

  TempCtx ctxInitial(m_ctxPool);
  TempCtx ctxBest(m_ctxPool);
  TempCtx ctxStart(m_ctxPool);
  ctxInitial = SubCtx(Ctx::CcAlfFilterControlFlag, m_CABACEstimator->getCtx());
  ctxBest    = SubCtx(Ctx::CcAlfFilterControlFlag, m_CABACEstimator->getCtx());

  int ctuIdx = 0;
  for (int yCtu = 0; yCtu < buf->height; yCtu += ctuHeightC)
  {
    for (int xCtu = 0; xCtu < buf->width; xCtu += ctuWidthC)
    {
      uint64_t ssd;
      float    rate;
      float    cost;

      uint64_t       bestSSD       = MAX_UINT64;
      float          bestRate      = MAX_FLOAT;
      float          bestCost      = MAX_FLOAT;
      uint8_t        bestFilterIdc = 0;
      uint8_t        bestFilterIdx = 0;
      const uint32_t thresholdS =
        (std::min<int>(buf->height - yCtu, ctuHeightC) << getComponentScaleY(COMP_Cb, m_chromaFormat));
      const uint32_t numberOfChromaSamples =
        std::min<int>(buf->height - yCtu, ctuHeightC) * std::min<int>(buf->width - xCtu, ctuWidthC);
      const uint32_t thresholdC = (numberOfChromaSamples >> 2);

      m_CABACEstimator->getCtx() = ctxBest;
      ctxStart                   = SubCtx(Ctx::CcAlfFilterControlFlag, m_CABACEstimator->getCtx());

      for (int filterIdx = 0; filterIdx <= MAX_NUM_CC_ALF_FILTERS; filterIdx++)
      {
        uint8_t filterIdc = mapFilterIdxToFilterIdc[filterIdx];
        if (filterIdx < MAX_NUM_CC_ALF_FILTERS && !filterEnabled[filterIdx])
        {
          continue;
        }

        if (filterIdx == MAX_NUM_CC_ALF_FILTERS)
        {
          ssd = (uint64_t)unfilteredDistortion[compID][ctuIdx];   // restore saved distortion computation
        }
        else
        {
          ssd = trainingDistortion[filterIdx][ctuIdx];
        }
        m_CABACEstimator->getCtx() = ctxStart;
        m_CABACEstimator->resetBits();
        const Position lumaPos = Position({ xCtu << getComponentScaleX(compID, cs.pcv->chrFormat),
                                            yCtu << getComponentScaleY(compID, cs.pcv->chrFormat) });
        m_CABACEstimator->codeCcAlfFilterControlIdc(filterIdc, cs, compID, ctuIdx, filterControl, lumaPos,
                                                    ccAlfFilterCount);
        rate = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
        cost = rate * m_lambda[compID] + ssd;

        bool limitationExceeded = false;
        if (m_limitCcAlf && filterIdx < MAX_NUM_CC_ALF_FILTERS)
        {
          limitationExceeded = limitationExceeded || (lumaSwingGreaterThanThresholdCount[ctuIdx] >= thresholdS);
          limitationExceeded = limitationExceeded || (chromaSampleCountNearMidPoint[ctuIdx] >= thresholdC);
        }
        if (cost < bestCost && !limitationExceeded)
        {
          bestCost      = cost;
          bestRate      = rate;
          bestSSD       = ssd;
          bestFilterIdc = filterIdc;
          bestFilterIdx = filterIdx;

          ctxBest = SubCtx(Ctx::CcAlfFilterControlFlag, m_CABACEstimator->getCtx());

          trainingCovControl[ctuIdx] = (filterIdx == MAX_NUM_CC_ALF_FILTERS) ? 0 : (filterIdx + 1);
          filterControl[ctuIdx]      = (filterIdx == MAX_NUM_CC_ALF_FILTERS) ? 0 : (filterIdx + 1);
        }
      }
      if (bestFilterIdc != 0)
      {
        curFilterEnabled[bestFilterIdx] = true;
        if (MAX_NUM_CC_ALF_FILTERS > 1)
        {
          filterIdxCount[bestFilterIdx].count++;
        }
      }
      curTotalRate += bestRate;
      curTotalDistortion += bestSSD;
      ctuIdx++;
    }
  }

  if ((MAX_NUM_CC_ALF_FILTERS > 1) && !reuseTemporalFilterCoeff)
  {
    std::copy_n(curFilterEnabled, MAX_NUM_CC_ALF_FILTERS, filterEnabled);

    std::stable_sort(filterIdxCount, filterIdxCount + MAX_NUM_CC_ALF_FILTERS, compareCounts);

    int filterIdc    = 1;
    ccAlfFilterCount = 0;
    for (FilterIdxCount &s: filterIdxCount)
    {
      const int filterIdx = s.filterIdx;
      if (filterEnabled[filterIdx])
      {
        mapFilterIdxToFilterIdc[filterIdx] = filterIdc;
        filterIdc++;
        ccAlfFilterCount++;
      }
    }

    curTotalRate               = prevRate;
    m_CABACEstimator->getCtx() = ctxInitial;
    m_CABACEstimator->resetBits();
    int ctuIdx = 0;
    for (int y = 0; y < buf->height; y += ctuHeightC)
    {
      for (int x = 0; x < buf->width; x += ctuWidthC)
      {
        const int filterIdxPlus1 = filterControl[ctuIdx];

        const Position lumaPos = Position(
          { x << getComponentScaleX(compID, cs.pcv->chrFormat), y << getComponentScaleY(compID, cs.pcv->chrFormat) });

        m_CABACEstimator->codeCcAlfFilterControlIdc(filterIdxPlus1 == 0 ? 0
                                                                        : mapFilterIdxToFilterIdc[filterIdxPlus1 - 1],
                                                    cs, compID, ctuIdx, filterControl, lumaPos, ccAlfFilterCount);

        ctuIdx++;
      }
    }
    curTotalRate += FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
  }

  // restore for next iteration
  m_CABACEstimator->getCtx() = ctxInitial;
}

template<class ParamsT> template<class Covariance>
void EncAdaptiveLoopFilterBase<ParamsT>::getFrameStatsCcalf(std::vector<Covariance>    &alfCovarianceFrameCcAlf,
                                                            const vector2D<Covariance> &alfCovarianceCcAlf,
                                                            CompID compIdx, int filterIdc, const int numShapesCcAlf)
{
  // init Frame stats buffers
  for (int shape = 0; shape != numShapesCcAlf; ++shape)
  {
    alfCovarianceFrameCcAlf[shape].reset();
  }

  for (int yPos = 0, ctuRsAddr = 0; yPos < m_picHeight; yPos += m_maxCUHeight)
  {
    for (int xPos = 0; xPos < m_picWidth; xPos += m_maxCUWidth, ++ctuRsAddr)
    {
      if (m_trainingCovControl[ctuRsAddr] == filterIdc)
      {
        for (int shape = 0; shape != numShapesCcAlf; ++shape)
        {
          alfCovarianceFrameCcAlf[shape] += alfCovarianceCcAlf[shape][ctuRsAddr];
        }
      }
    }
  }
}
