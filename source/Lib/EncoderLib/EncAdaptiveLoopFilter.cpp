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

/** \file     EncAdaptiveLoopFilter.cpp
 \brief    estimation part of adaptive loop filter class
 */
#include "EncAdaptiveLoopFilter.h"
#include "EncCfg.h"

#include "CommonLib/Picture.h"
#include "CommonLib/CodingStructure.h"
#if defined(TARGET_SIMD_X86) && ENABLE_SIMD_OPT_ALF
#include "CommonLib/x86/CommonDefX86.h"
#endif

#define AlfCtx(c) SubCtx(Ctx::Alf, c)

#include <algorithm>

// static inline bool essentiallyEqual(double a, double b)
//{
// #if 0
//   constexpr double REL_EPSILON = 0x1p-20;   // 2^-20 or about 1e-6
//   constexpr double ABS_EPSILON = 0x1p-30;   // 2^-30 or about 1e-9
//   return std::abs(a - b) < std::max(ABS_EPSILON, REL_EPSILON * std::max(std::abs(a), std::abs(b)));
// #else
//   return a == b;
// #endif
// }

static inline bool essentiallyEqual(float a, float b)
{
#if 0
  constexpr float REL_EPSILON = 0x1p-20f;   // 2^-20
  constexpr float ABS_EPSILON = 0x1p-30f;   // 2^-30
  return std::abs(a - b) < std::max(ABS_EPSILON, REL_EPSILON * std::max(std::abs(a), std::abs(b)));
#else
  return a == b;
#endif
}

void AlfCovariance::getClipMax(int *clip_max) const
{
  for (int k = 0; k < numCoeff - 1; ++k)
  {
    clip_max[k] = 0;

    bool inc = true;
    while (inc && clip_max[k] + 1 < numBins && essentiallyEqual(y(clip_max[k] + 1, k), y(clip_max[k], k)))
    {
      for (int l = 0; inc && l < numCoeff; ++l)
      {
        if (!essentiallyEqual(E(clip_max[k], 0, k, l), E(clip_max[k] + 1, 0, k, l)))
        {
          inc = false;
        }
      }
      if (inc)
      {
        ++clip_max[k];
      }
    }
  }
  clip_max[numCoeff - 1] = 0;
}

void AlfCovariance::reduceClipCost(int *clip) const
{
  for (int k = 0; k < numCoeff - 1; ++k)
  {
    bool dec = true;
    while (dec && clip[k] > 0 && essentiallyEqual(y(clip[k] - 1, k), y(clip[k], k)))
    {
      for (int l = 0; dec && l < numCoeff; ++l)
      {
        if (!essentiallyEqual(E(clip[k], clip[l], k, l), E(clip[k] - 1, clip[l], k, l)))
        {
          dec = false;
        }
      }
      if (dec)
      {
        --clip[k];
      }
    }
  }
}

float AlfCovariance::calcErrorForCoeffs(const int *clip, const int *coeff, const int numCoeff, const int scale,
                                        const int fractionalBits) const
{
  const float factor = 1 << fractionalBits;

  float error = 0;

  if (numBins == 1)
  {
    static constexpr int N = 40;
    CHECK(numCoeff > N, "Need to increase max number of bins!");

#if defined(TARGET_SIMD_X86) && ENABLE_SIMD_OPT_ALF
    bool useSimd = read_x86_extension_flags() > SCALAR;

    if (useSimd)
    {
      float arreij[N], arrsum[N], arry[N];

      for (ptrdiff_t i = 0; i < numCoeff; ++i)   // diagonal
      {
        const float *Eij = &Efast(0, 0, i, 0);
        const int   *cff = coeff;

        __m128 xsum = _mm_setzero_ps();

        ptrdiff_t j = 3;
        __m128    xcff, xeij;

        for (; j < i; j += 4)
        {
          xcff = _mm_cvtepi32_ps(_mm_loadu_si128((const __m128i *)cff));
          xeij = _mm_loadu_ps(Eij);
          // E[j][i] = E[i][j], sum will be multiplied by 2 later
          Eij += 4;
          cff += 4;

          xsum = _mm_add_ps(xsum, _mm_mul_ps(xcff, xeij));
        }

        ptrdiff_t r = i - j + 3;

        float c0 = 0.0f, c1 = 0.0f, c2 = 0.0f, e0 = 0.0f, e1 = 0.0f, e2 = 0.0f;

        switch (r)
        {
        case 3:
          c2 = cff[2];
          e2 = Eij[2];
        case 2:
          c1 = cff[1];
          e1 = Eij[1];
        case 1:
          c0 = cff[0];
          e0 = Eij[0];
        case 0:
        default:;
        }

        xcff = _mm_set_ps(c0, c1, c2, 0.0f);
        xeij = _mm_set_ps(e0, e1, e2, 0.0f);
        xsum = _mm_add_ps(xsum, _mm_mul_ps(xcff, xeij));

        xsum      = _mm_hadd_ps(xsum, xsum);
        xsum      = _mm_hadd_ps(xsum, xsum);
        float sum = _mm_cvtss_f32(xsum);

        arreij[i] = Eij[r];
        arrsum[i] = sum;
        arry[i]   = y(0, i);

        // error += ((*Eij * coeff[i] + sum * 2) / factor - 2 * y(0, i)) * coeff[i];
      }

      __m128 xerr = _mm_setzero_ps();
      __m128 xtwo = _mm_set1_ps(2.0f);
      __m128 xscl = _mm_set1_ps((float)scale);
      __m128 xftr = _mm_set1_ps(factor);

      int i = 0;
      for (; i < (numCoeff - 3); i += 4)
      {
        __m128 eij = _mm_loadu_ps(&arreij[i]);
        __m128 cff = _mm_cvtepi32_ps(_mm_loadu_si128((const __m128i *)&coeff[i]));
        __m128 sum = _mm_loadu_ps(&arrsum[i]);
        __m128 y0i = _mm_loadu_ps(&arry[i]);

        sum = _mm_mul_ps(xtwo, sum);
        y0i = _mm_mul_ps(xtwo, y0i);

        __m128 tmp = _mm_mul_ps(eij, cff);
        tmp        = _mm_add_ps(tmp, sum);
        tmp        = _mm_mul_ps(tmp, xscl);
        tmp        = _mm_div_ps(tmp, xftr);
        tmp        = _mm_sub_ps(tmp, y0i);
        tmp        = _mm_mul_ps(tmp, cff);
        tmp        = _mm_mul_ps(tmp, xscl);
        xerr       = _mm_add_ps(tmp, xerr);
      }
      xerr = _mm_hadd_ps(xerr, xerr);
      xerr = _mm_hadd_ps(xerr, xerr);

      error = _mm_cvtss_f32(xerr);

      for (; i < numCoeff; ++i)
      {
        error += ((arreij[i] * coeff[i] + arrsum[i] * 2) * scale / factor - 2 * arry[i]) * coeff[i] * scale;
      }
    }
    else
#endif
    {
      // calculate scalar sum in same order as SIMD code to achieve identical accuracy
      float arrsum[N];

      for (ptrdiff_t i = 0; i < numCoeff; ++i)   // diagonal
      {
        const float *Eij = &Efast(0, 0, i, 0);
        const int   *cff = coeff;

        float sum4[4];
        memset(sum4, 0, 4 * sizeof(float));

        ptrdiff_t j = 3;
        for (; j < i; j += 4)
        {
          for (int n = 0; n < 4; n++)
          {
            sum4[n] += cff[n] * Eij[n];
          }
          Eij += 4;
          cff += 4;
        }
        for (int n = 0; n < (i - j + 3); n++)
        {
          sum4[3 - n] += *Eij++ * *cff++;
        }
        float sum = sum4[0] + sum4[1];
        sum += sum4[2] + sum4[3];
        arrsum[i] = sum;
        // error += ((*Eij * coeff[i] + sum * 2) / factor - 2 * y(0, i)) * coeff[i];
      }
      int   i = 0;
      float err[4];
      memset(err, 0, 4 * sizeof(float));
      for (; i < (numCoeff - 3); i += 4)
      {
        for (int n = 0; n < 4; n++)
        {
          float sum = arrsum[i + n] * 2;
          float y01 = 2 * y(0, i + n);
          float tmp = Efast(0, 0, i + n, i + n) * coeff[i + n];
          tmp       = tmp + sum;
          tmp       = tmp * scale / factor;
          tmp       = tmp - y01;
          tmp       = tmp * coeff[i + n] * scale;
          err[n] += tmp;
        }
      }
      error = err[0] + err[1];
      error += err[2] + err[3];

      for (; i < numCoeff; ++i)
      {
        error += ((Efast(0, 0, i, i) * coeff[i] + arrsum[i] * 2) * scale / factor - 2 * y(0, i)) * coeff[i] * scale;
      }
    }
  }
  else
  {
    for (ptrdiff_t i = 0; i < numCoeff; ++i)   // diagonal
    {
      float sum = 0;
      for (ptrdiff_t j = 0; j < i; ++j)
      {
        // E[j][i] = E[i][j], sum will be multiplied by 2 later
        sum += E(clip[i], clip[j], i, j) * coeff[j];
      }
      error +=
        ((E(clip[i], clip[i], i, i) * coeff[i] + sum * 2) * scale / factor - 2 * y(clip[i], i)) * coeff[i] * scale;
    }
  }

  return error / factor;
}

float AlfCovariance::calcErrorForCcAlfCoeffs(const int16_t *coeff, const int numCoeff, const int bitDepth) const
{
  float factor = 1 << (bitDepth - 1);
  float error  = 0;

  for (int i = 0; i < numCoeff; i++)   // diagonal
  {
    float sum = 0;
    for (int j = i + 1; j < numCoeff; j++)
    {
      // E[j][i] = E[i][j], sum will be multiplied by 2 later
      sum += E(0, 0, i, j) * coeff[j];
    }
    error += ((E(0, 0, i, i) * coeff[i] + sum * 2) / factor - 2 * y(0, i)) * coeff[i];
  }

  return error / factor;
}

float AlfCovariance::calculateError(const int *clip, const float *coeff, const int numCoeff) const
{
  float sum = 0;
  for (int i = 0; i < numCoeff; i++)
  {
    sum += coeff[i] * y(clip[i], i);
  }

  return pixAcc - sum;
}

float AlfCovariance::initCachedCalcErrorForCoeffs(const int *clip, const int *coeff, const int numCoeff,
                                                  const int scale, const int fractionalBits, float &quadratic,
                                                  float *dotProduct, float &constTermSum) const
{
  const float factor = 1 << fractionalBits;
  quadratic          = 0;
  constTermSum       = 0;

  for (ptrdiff_t i = 0; i < numCoeff; i++)   // diagonal
  {
    float sum = 0;
    for (ptrdiff_t j = 0; j < numCoeff; j++)
    {
      sum += E(clip[i], clip[j], i, j) * coeff[j];
    }
    sum *= scale;
    dotProduct[i] = 2 * sum / factor;
    quadratic += sum * coeff[i];
    constTermSum += 2 * coeff[i] * y(clip[i], i);
  }

  quadratic *= scale;
  quadratic /= factor * factor;
  constTermSum *= scale;
  constTermSum /= factor;

  return quadratic - constTermSum;
}

float AlfCovariance::calcCachedCalcErrorForCoeffs(const int *clip, const float quadratic, const float *dotProduct,
                                                  const float constTermSum, const int coeffIdx,
                                                  const float coeffDelta) const
{
  float error = quadratic - constTermSum;
  error += coeffDelta * dotProduct[coeffIdx];
  error += coeffDelta * coeffDelta * E(clip[coeffIdx], clip[coeffIdx], coeffIdx, coeffIdx);
  error -= 2 * y(clip[coeffIdx], coeffIdx) * coeffDelta;
  return error;
}

void AlfCovariance::updateCachedErrorForCoeffs(const int *clip, const int numCoeff, float &quadratic, float *dotProduct,
                                               float &constTermSum, const int coeffIdx, const float coeffDelta) const
{
  quadratic += coeffDelta * dotProduct[coeffIdx];
  quadratic += coeffDelta * coeffDelta * E(clip[coeffIdx], clip[coeffIdx], coeffIdx, coeffIdx);
  constTermSum += 2 * y(clip[coeffIdx], coeffIdx) * coeffDelta;
  for (ptrdiff_t i = 0; i < numCoeff; i++)
  {
    dotProduct[i] += 2 * coeffDelta * E(clip[coeffIdx], clip[i], coeffIdx, i);
  }
}

EncAdaptiveLoopFilter::EncAdaptiveLoopFilter() : m_CABACEstimator(nullptr)
{
  m_filterCoeffSet = nullptr;
  m_filterClippSet = nullptr;
}

void EncAdaptiveLoopFilter::createSharedEncMembers(const EncCfg *encCfg)
{
  m_encCfg = encCfg;

  for (int comp = 0; comp < MAX_NUM_COMP; comp++)
  {
    m_ctbDistortionUnfilter[comp] = new float[m_numCTUsInPic];
  }
  m_indexTmp.resize(m_numCTUsInPic);

  m_apsIdCcAlfStart[0] = MAX_NUM_APS(ApsType::ALF);
  m_apsIdCcAlfStart[1] = MAX_NUM_APS(ApsType::ALF);

  m_trainingCovControl = new uint8_t[m_numCTUsInPic];
  m_filterControl      = new uint8_t[m_numCTUsInPic];
  m_bestFilterControl  = new uint8_t[m_numCTUsInPic];
  m_bestFilterCount    = 0;
  uint32_t area        = (m_picWidth >> getComponentScaleX(COMP_Cb, m_chromaFormat)) *
    (m_picHeight >> getComponentScaleY(COMP_Cb, m_chromaFormat));
  m_bufOrigin = (Pel *)xMalloc(Pel, area);
  m_buf       = new PelBuf(m_bufOrigin, m_picWidth >> getComponentScaleX(COMP_Cb, m_chromaFormat),
                           m_picWidth >> getComponentScaleX(COMP_Cb, m_chromaFormat),
                           m_picHeight >> getComponentScaleY(COMP_Cb, m_chromaFormat));
  m_lumaSwingGreaterThanThresholdCount = new uint64_t[m_numCTUsInPic];
  m_chromaSampleCountNearMidPoint      = new uint64_t[m_numCTUsInPic];
}

void EncAdaptiveLoopFilter::destroySharedEncMembers()
{
  for (int comp = 0; comp < MAX_NUM_COMP; comp++)
  {
    if (m_ctbDistortionUnfilter[comp])
    {
      delete[] m_ctbDistortionUnfilter[comp];
      m_ctbDistortionUnfilter[comp] = nullptr;
    }
  }

  if (m_trainingCovControl)
  {
    delete[] m_trainingCovControl;
    m_trainingCovControl = nullptr;
  }

  if (m_filterControl)
  {
    delete[] m_filterControl;
    m_filterControl = nullptr;
  }

  if (m_bestFilterControl)
  {
    delete[] m_bestFilterControl;
    m_bestFilterControl = nullptr;
  }

  if (m_bufOrigin)
  {
    xFree(m_bufOrigin);
    m_bufOrigin = nullptr;
  }

  if (m_buf)
  {
    delete m_buf;
    m_buf = nullptr;
  }

  if (m_lumaSwingGreaterThanThresholdCount)
  {
    delete[] m_lumaSwingGreaterThanThresholdCount;
    m_lumaSwingGreaterThanThresholdCount = nullptr;
  }
  if (m_chromaSampleCountNearMidPoint)
  {
    delete[] m_chromaSampleCountNearMidPoint;
    m_chromaSampleCountNearMidPoint = nullptr;
  }
}

void EncAdaptiveLoopFilter::initCABACEstimator(CABACEncoder *cabacEncoder, CtxPool *ctxPool, Slice *pcSlice,
                                               ParameterSetMap<APS> *apsMap)
{
  m_apsMap         = apsMap;
  m_CABACEstimator = cabacEncoder->getCABACEstimator(pcSlice->m_sps);
  m_ctxPool        = ctxPool;
  m_CABACEstimator->initCtxModels(*pcSlice);
  m_CABACEstimator->resetBits();
}

int EncAdaptiveLoopFilter::getCostFilterClipp(const int numCoeffs, const int *const *const pDiffQFilterCoeffIntPP,
                                              const int numFilters)
{
  for (int filterIdx = 0; filterIdx < numFilters; ++filterIdx)
  {
    for (int i = 0; i < numCoeffs - 1; i++)
    {
      if (!abs(pDiffQFilterCoeffIntPP[filterIdx][i]))
      {
        m_filterClippSet[filterIdx][i] = 0;
      }
    }
  }
  return (numFilters * (numCoeffs - 1)) << 1;
}

float EncAdaptiveLoopFilter::getDistCoeffForce0(bool *codedVarBins, float errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2],
                                                int *bitsVarBin, int zeroBitsVarBin, const int numFilters) const
{
  float distForce0 = 0;
  std::memset(codedVarBins, 0, sizeof(*codedVarBins) * MAX_NUM_ALF_CLASSES);

  for (int filtIdx = 0; filtIdx < numFilters; filtIdx++)
  {
    float costDiff = (errorForce0CoeffTab[filtIdx][0] + m_lambda[COMP_Y] * zeroBitsVarBin) -
      (errorForce0CoeffTab[filtIdx][1] + m_lambda[COMP_Y] * bitsVarBin[filtIdx]);
    codedVarBins[filtIdx] = costDiff > 0 ? true : false;
    distForce0 += errorForce0CoeffTab[filtIdx][codedVarBins[filtIdx] ? 1 : 0];
  }

  return distForce0;
}

int EncAdaptiveLoopFilter::lengthUvlc(int code)
{
  CHECK(code < 0, "Unsigned VLC cannot be negative");
  CHECK(code == MAX_INT, "Maximum supported UVLC code is MAX_INT-1");

  int length = 1;
  int temp   = ++code;

  while (1 != temp)
  {
    temp >>= 1;
    length += 2;
  }
  // Take care of cases where length > 32
  return (length >> 1) + ((length + 1) >> 1);
}

void EncAdaptiveLoopFilter::roundFiltCoeffCCALF(int16_t *const filterCoeffQuant, const float *const filterCoeff,
                                                const int numCoeff, const int factor) const
{
  for (int i = 0; i < numCoeff; i++)
  {
    const int sign = sgn2(filterCoeff[i]);

    float bestErr   = factor * factor;
    int   bestIndex = 0;

    const float val = filterCoeff[i] * m_encCfg->m_ccalfStrength * sign * factor;

    for (int k = 0; k < CCALF_CANDS_COEFF_NR; k++)
    {
      const float diff = val - CCALF_SMALL_TAB[k];
      const float err  = diff * diff;

      if (err < bestErr)
      {
        bestErr   = err;
        bestIndex = k;
      }
    }

    filterCoeffQuant[i] = CCALF_SMALL_TAB[bestIndex] * sign;
  }
}

void EncAdaptiveLoopFilter::setSliceEnabledFlag(AlfParamBase &alfSlicePara, ChannelType channel, bool val)
{
  if (isLuma(channel))
  {
    alfSlicePara.enabledFlag[COMP_Y] = val;
  }
  else
  {
    alfSlicePara.enabledFlag[COMP_Cb] = val;
    alfSlicePara.enabledFlag[COMP_Cr] = val;
  }
}

void EncAdaptiveLoopFilter::setSliceEnabledFlag(AlfParamBase &alfSlicePara, ChannelType channel,
                                                const CtuModes &ctuModes)
{
  const CompID compIDFirst = isLuma(channel) ? COMP_Y : COMP_Cb;
  const CompID compIDLast  = isLuma(channel) ? COMP_Y : COMP_Cr;
  for (int compId = compIDFirst; compId <= compIDLast; compId++)
  {
    alfSlicePara.enabledFlag[compId] = std::any_of(ctuModes[compId].begin(), ctuModes[compId].end(),
                                                   [](const int x) { return CtbModeHandler::isEnabled(x); });
  }
}

void EncAdaptiveLoopFilter::copyIndices(CtuModes &ctuModesDst, const CtuModes &ctuModesSrc, ChannelType channel)
{
  if (isLuma(channel))
  {
    CHECKD(ctuModesSrc[CompID::COMP_Y].size() != m_numCTUsInPic, "Unexpected number of mode entries for Y (source).");
    CHECKD(ctuModesDst[CompID::COMP_Y].size() != m_numCTUsInPic, "Unexpected number of mode entries for Y (target).");
    ctuModesDst[CompID::COMP_Y] = ctuModesSrc[CompID::COMP_Y];
  }
  else
  {
    CHECKD(ctuModesSrc[CompID::COMP_Cb].size() != m_numCTUsInPic, "Unexpected number of mode entries for Cb (source).");
    CHECKD(ctuModesDst[CompID::COMP_Cb].size() != m_numCTUsInPic, "Unexpected number of mode entries for Cb (target).");
    CHECKD(ctuModesSrc[CompID::COMP_Cr].size() != m_numCTUsInPic, "Unexpected number of mode entries for Cr (source).");
    CHECKD(ctuModesDst[CompID::COMP_Cr].size() != m_numCTUsInPic, "Unexpected number of mode entries for Cr (target).");
    ctuModesDst[CompID::COMP_Cb] = ctuModesSrc[CompID::COMP_Cb];
    ctuModesDst[CompID::COMP_Cr] = ctuModesSrc[CompID::COMP_Cr];
  }
}

void EncAdaptiveLoopFilter::setCtuEnableFlag(CtuModes &ctuModes, ChannelType channel, const int val)
{
  if (isLuma(channel))
  {
    CHECKD(ctuModes[COMP_Y].size() != m_numCTUsInPic, "Unexpected number of mode entries for Y.");
    std::fill_n(ctuModes[COMP_Y].data(), m_numCTUsInPic, val);
  }
  else
  {
    CHECKD(ctuModes[COMP_Cb].size() != m_numCTUsInPic, "Unexpected number of mode entries for Cb.");
    CHECKD(ctuModes[COMP_Cr].size() != m_numCTUsInPic, "Unexpected number of mode entries for Cr.");
    std::fill_n(ctuModes[COMP_Cb].data(), m_numCTUsInPic, val);
    std::fill_n(ctuModes[COMP_Cr].data(), m_numCTUsInPic, val);
  }
}

int EncAdaptiveLoopFilter::getAvailableApsIdsLuma(CodingStructure &cs)
{
  APS     **apss       = cs.slice->m_alfApss;
  const int firstApsId = m_encCfg->m_alfapsIDShift;
  const int lastApsId  = firstApsId + m_encCfg->m_maxNumAlfAps;

  for (int i = firstApsId; i < lastApsId; i++)
  {
    apss[i] = m_apsMap->getPS(i);
  }

  AlfApsList result;

  int curApsId = m_apsIdStart;

  if (curApsId < lastApsId && !cs.slice->isIRAP() && !cs.slice->m_pendingRasInit)
  {
    for (int i = 0; i < m_encCfg->m_maxNumAlfAps; i++)
    {
      APS *curAPS = apss[curApsId];

      if (curAPS != nullptr && curAPS->m_layerId == cs.slice->m_pic->m_layerId &&
          curAPS->m_temporalId <= cs.slice->m_uiTLayer &&
          curAPS->m_alfAPSParam.getParam().newFilterFlag[ChannelType::LUMA])
      {
        result.push_back(curApsId);
      }
      if (++curApsId >= lastApsId)
      {
        curApsId = firstApsId;
      }
    }
  }

  cs.slice->m_numAlfApsIdsLuma = (int)result.size();
  cs.slice->m_alfApsIdsLuma    = result;

  int newApsId = m_apsIdStart - 1;
  if (newApsId < firstApsId)
  {
    newApsId = lastApsId - 1;
  }
  CHECK(newApsId >= lastApsId, "Wrong APS index assignment");

  return newApsId;
}

int EncAdaptiveLoopFilter::getMaxNumAlternativesChroma()
{
  return std::min<int>(m_numCTUsInPic * 2, m_encCfg->m_maxNumAlfAlternativesChroma);
}

std::vector<int> EncAdaptiveLoopFilter::getAvailableCcAlfApsIds(CodingStructure &cs, CompID compID)
{
  APS **apss = cs.slice->m_alfApss;
  for (int i = m_encCfg->m_alfapsIDShift; i < m_encCfg->m_alfapsIDShift + m_encCfg->m_maxNumAlfAps; i++)
  {
    apss[i] = m_apsMap->getPS(i);
  }

  std::vector<int> result;
  int              numApsIdsChecked = 0, curApsId = m_apsIdStart;
  if (curApsId < m_encCfg->m_alfapsIDShift + m_encCfg->m_maxNumAlfAps)
  {
    while ((numApsIdsChecked < m_encCfg->m_maxNumAlfAps) && !cs.slice->isIRAP() &&
           (result.size() < m_encCfg->m_maxNumAlfAps) && !cs.slice->m_pendingRasInit)
    {
      APS *curAPS = apss[curApsId];
      if (curAPS && curAPS->m_layerId == cs.slice->m_pic->m_layerId && curAPS->m_temporalId <= cs.slice->m_uiTLayer &&
          curAPS->m_ccAlfAPSParam.getParam().newCcAlfFilter[compID - 1])
      {
        result.push_back(curApsId);
      }
      numApsIdsChecked++;
      curApsId++;
      if (curApsId >= m_encCfg->m_alfapsIDShift + m_encCfg->m_maxNumAlfAps)
      {
        curApsId = m_encCfg->m_alfapsIDShift;
      }
    }
  }
  return result;
}

void EncAdaptiveLoopFilter::countChromaSampleValueNearMidPoint(const Pel *chroma, ptrdiff_t chromaStride, int height,
                                                               int width, int log2BlockWidth, int log2BlockHeight,
                                                               uint64_t *chromaSampleCountNearMidPoint,
                                                               int       chromaSampleCountNearMidPointStride)
{
  const int midPoint  = (1 << m_inputBitDepth[ChannelType::CHROMA]) >> 1;
  const int threshold = 16;

  for (int y = 0; y < height; y += (1 << log2BlockHeight))
  {
    for (int x = 0; x < width; x += (1 << log2BlockWidth))
    {
      chromaSampleCountNearMidPoint[(y >> log2BlockHeight) * chromaSampleCountNearMidPointStride +
                                    (x >> log2BlockWidth)] = 0;

      for (int yOff = 0; yOff < (1 << log2BlockHeight); yOff++)
      {
        for (int xOff = 0; xOff < (1 << log2BlockWidth); xOff++)
        {
          if ((y + yOff) >= height || (x + xOff) >= width)
          {
            continue;
          }

          int distanceToMidPoint = abs(chroma[yOff * chromaStride + x + xOff] - midPoint);
          if (distanceToMidPoint < threshold)
          {
            chromaSampleCountNearMidPoint[(y >> log2BlockHeight) * chromaSampleCountNearMidPointStride +
                                          (x >> log2BlockWidth)]++;
          }
        }
      }
    }
    chroma += (chromaStride << log2BlockHeight);
  }
}
