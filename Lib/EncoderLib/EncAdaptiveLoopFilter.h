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

/** \file     EncAdaptiveLoopFilter.h
 \brief    estimation part of adaptive loop filter class (header)
 */

#ifndef __ENCADAPTIVELOOPFILTER__
#define __ENCADAPTIVELOOPFILTER__

#include "CommonLib/AdaptiveLoopFilter.h"
#include "CommonLib/ParameterSetManager.h"

#include "CABACWriter.h"

struct EncCfg;

struct AlfCovariance
{
  std::vector<float> data;

  ptrdiff_t offsetE;
  ptrdiff_t stridey;

  size_t sizeY() const { return offsetE; }
  size_t sizeE() const { return data.size() - offsetE; }

  ptrdiff_t getOffsetY(ptrdiff_t i, ptrdiff_t j) const { return stridey * i + j; }

  ptrdiff_t getOffsetEfast(ptrdiff_t i, ptrdiff_t j, ptrdiff_t k, ptrdiff_t l) const
  {
    const ptrdiff_t v0 = getOffsetY(i, k);
    const ptrdiff_t v1 = getOffsetY(j, l);

    assert(v1 <= v0);

    return offsetE + (v0 * (v0 + 1) >> 1) + v1;
  }

  ptrdiff_t getOffsetE(ptrdiff_t i, ptrdiff_t j, ptrdiff_t k, ptrdiff_t l) const
  {
    const ptrdiff_t v0 = getOffsetY(i, k);
    const ptrdiff_t v1 = getOffsetY(j, l);

    return offsetE + (v1 <= v0 ? (v0 * (v0 + 1) >> 1) + v1 : (v1 * (v1 + 1) >> 1) + v0);
  }

  float       &y(ptrdiff_t i, ptrdiff_t j) { return data[getOffsetY(i, j)]; }
  const float &y(ptrdiff_t i, ptrdiff_t j) const { return data[getOffsetY(i, j)]; }

  float       &E(ptrdiff_t i, ptrdiff_t j, ptrdiff_t k, ptrdiff_t l) { return data[getOffsetE(i, j, k, l)]; }
  const float &E(ptrdiff_t i, ptrdiff_t j, ptrdiff_t k, ptrdiff_t l) const { return data[getOffsetE(i, j, k, l)]; }

  float       &Efast(ptrdiff_t i, ptrdiff_t j, ptrdiff_t k, ptrdiff_t l) { return data[getOffsetEfast(i, j, k, l)]; }
  const float &Efast(ptrdiff_t i, ptrdiff_t j, ptrdiff_t k, ptrdiff_t l) const
  {
    return data[getOffsetEfast(i, j, k, l)];
  }

  int   numCoeff;
  int   numBins;
  float pixAcc;

  void create(int size, int _numBins)
  {
    numCoeff          = size;
    numBins           = _numBins;
    const int numCols = numCoeff * numBins;
    stridey           = numCoeff;
    offsetE           = stridey * numBins;
    data.resize(offsetE + (numCols * (numCols + 1) >> 1));
    std::fill(data.begin(), data.end(), 0.0);
  }

  void reset()
  {
    pixAcc = 0;
    std::fill(data.begin(), data.end(), 0.0);
  }

  const AlfCovariance &operator=(const AlfCovariance &src)
  {
    numCoeff = src.numCoeff;
    numBins  = src.numBins;
    data     = src.data;
    stridey  = src.stridey;
    offsetE  = src.offsetE;
    pixAcc   = src.pixAcc;

    return *this;
  }

  bool sameSizeAs(const AlfCovariance &x) { return x.numCoeff == numCoeff && x.numBins == numBins; }

  void add(const AlfCovariance &lhs, const AlfCovariance &rhs)
  {
    numCoeff = lhs.numCoeff;
    numBins  = lhs.numBins;

    CHECK(!sameSizeAs(lhs), "AlfCovariance size mismatch");
    CHECK(!sameSizeAs(rhs), "AlfCovariance size mismatch");

    for (ptrdiff_t i = 0; i < data.size(); i++)
    {
      data[i] = lhs.data[i] + rhs.data[i];
    }
    pixAcc = lhs.pixAcc + rhs.pixAcc;
  }

  const AlfCovariance &operator+=(const AlfCovariance &src)
  {
    CHECK(src.numCoeff != numCoeff, "AlfCovariance size mismatch");
    // There may be a case where we accumulate only bin #0
    CHECK(src.numBins != numBins && numBins != 1, "AlfCovariance size mismatch");

    for (ptrdiff_t i = 0; i < sizeY(); i++)
    {
      data[i] += src.data[i];
    }

    if (src.numBins != numBins)
    {
      // numBins == 1 (see CHECK above)
      for (ptrdiff_t i = 0; i < numCoeff; i++)
      {
        for (ptrdiff_t j = 0; j <= i; j++)
        {
          data[getOffsetEfast(0, 0, i, j)] += src.data[src.getOffsetEfast(0, 0, i, j)];
        }
      }
    }
    else
    {
      for (ptrdiff_t i = 0; i < sizeE(); i++)
      {
        data[offsetE + i] += src.data[src.offsetE + i];
      }
    }

    pixAcc += src.pixAcc;

    return *this;
  }

  const AlfCovariance &operator-=(const AlfCovariance &src)
  {
    CHECK(src.numCoeff != numCoeff, "AlfCovariance size mismatch");
    // There may be a case where we accumulate only bin #0
    CHECK(src.numBins != numBins && numBins != 1, "AlfCovariance size mismatch");

    for (ptrdiff_t i = 0; i < sizeY(); i++)
    {
      data[i] -= src.data[i];
    }
    if (src.numBins != numBins)
    {
      // numBins == 1 (see CHECK above)
      for (ptrdiff_t i = 0; i < numCoeff; i++)
      {
        for (ptrdiff_t j = 0; j <= i; j++)
        {
          data[getOffsetEfast(0, 0, i, j)] -= src.data[src.getOffsetEfast(0, 0, i, j)];
        }
      }
    }
    else
    {
      for (ptrdiff_t i = 0; i < sizeE(); i++)
      {
        data[offsetE + i] -= src.data[src.offsetE + i];
      }
    }

    pixAcc -= src.pixAcc;

    return *this;
  }

  float calculateError(const int *clip, const float *coeff) const { return calculateError(clip, coeff, numCoeff); }
  float calculateError(const int *clip, const float *coeff, const int numCoeff) const;
  float calcErrorForCoeffs(const int *clip, const int *coeff, const int numCoeff, const int scale,
                           const int fractionalBits) const;
  float calcErrorForCcAlfCoeffs(const int16_t *coeff, const int numCoeff, const int bitDepth) const;

  float initCachedCalcErrorForCoeffs(const int *clip, const int *coeff, const int numCoeff, const int scale,
                                     const int fractionalBits, float &quadratic, float *dotProduct,
                                     float &constTermSum) const;
  float calcCachedCalcErrorForCoeffs(const int *clip, const float quadratic, const float *dotProduct,
                                     const float constTermSum, const int coeffIdx, const float coeffDelta) const;
  void  updateCachedErrorForCoeffs(const int *clip, const int numCoeff, float &quadratic, float *dotProduct,
                                   float &constTermSum, const int coeffIdx, const float coeffDelta) const;

  void getClipMax(int *clip_max) const;
  void reduceClipCost(int *clip) const;
};

class EncAdaptiveLoopFilter : virtual public AdaptiveLoopFilter
{
public:
  void setAlfWSSD(bool alfWSSD) { m_alfWSSD = alfWSSD; }
  void setLumaLevelWeightTable(const std::vector<double> &weightTable) { m_lumaLevelToWeightPLUT = weightTable; }

protected:
  bool                m_alfWSSD { false };
  std::vector<double> m_lumaLevelToWeightPLUT;

  const EncCfg *m_encCfg;

  // for RDO
  ParameterSetMap<APS> *m_apsMap;
  CABACWriter          *m_CABACEstimator;
  CtxPool              *m_ctxPool;
  double                m_lambda[MAX_NUM_COMP];

  int **m_filterCoeffSet;   // [lumaClassIdx/chromaAltIdx][coeffIdx]
  int **m_filterClippSet;   // [lumaClassIdx/chromaAltIdx][coeffIdx]
  short m_filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES];

  EnumArray<unsigned, ChannelType> m_bitsNewFilter;

  int      m_apsIdStart;
  float   *m_ctbDistortionUnfilter[MAX_NUM_COMP];
  float    m_unFiltDistCompnent[MAX_NUM_COMP];
  CtuModes m_indexTmp;

  int m_apsIdCcAlfStart[2];

  uint8_t   m_bestFilterCount;
  uint8_t  *m_trainingCovControl;
  Pel      *m_bufOrigin;
  PelBuf   *m_buf;
  uint64_t *m_lumaSwingGreaterThanThresholdCount;
  uint64_t *m_chromaSampleCountNearMidPoint;
  uint8_t  *m_filterControl;       // current iterations filter control
  uint8_t  *m_bestFilterControl;   // best saved filter control
  int       m_reuseApsId[2];
  bool      m_limitCcAlf;

public:
  PelUnitBuf *m_newOrgBuf;

public:
  EncAdaptiveLoopFilter();

  int  getAvailableApsIdsLuma(CodingStructure &cs);
  void initCABACEstimator(CABACEncoder *cabacEncoder, CtxPool *ctxPool, Slice *pcSlice, ParameterSetMap<APS> *apsMap);
  void setApsIdStart(int i) { m_apsIdStart = i; }

protected:
  void roundFiltCoeffCCALF(int16_t *const filterCoeffQuant, const float *const filterCoeff, const int numCoeff,
                           int factor) const;

  float      getDistCoeffForce0(bool *codedVarBins, float errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2], int *bitsVarBin,
                                int zeroBitsVarBin, const int numFilters) const;
  static int lengthUvlc(int uiCode);

  int getCostFilterClipp(const int numCoeffs, const int *const *const pDiffQFilterCoeffIntPP, const int numFilters);

  template<class AlfCovarianceT>
  static float getUnfilteredDistortion(const std::vector<AlfCovarianceT> &cov, const int numClasses);

  void        setSliceEnabledFlag(AlfParamBase &alfSlicePara, ChannelType channel, bool val);
  void        setSliceEnabledFlag(AlfParamBase &alfSlicePara, ChannelType channel, const CtuModes &ctuModes);
  inline void setSliceEnabledFlag(AlfParamBase &alfSlicePara, ChannelType channel, const CtuModes *const ctuModes);

  void             setCtuEnableFlag(CtuModes &ctuModes, ChannelType channel, const int val);
  inline void      setCtuEnableFlag(CtuModes *const ctuModes, ChannelType channel, const int val);
  void             copyIndices(CtuModes &ctuModesDst, const CtuModes &ctuModesSrc, ChannelType channel);
  inline void      copyIndices(CtuModes *const ctuModesDst, const CtuModes &ctuModesSrc, ChannelType channel);
  inline void      copyIndices(CtuModes &ctuModesDst, const CtuModes *const ctuModesSrc, ChannelType channel);
  inline void      copyIndices(CtuModes *const ctuModesDst, const CtuModes *const ctuModesSrc, ChannelType channel);
  int              getMaxNumAlternativesChroma();
  std::vector<int> getAvailableCcAlfApsIds(CodingStructure &cs, CompID compID);

  void countChromaSampleValueNearMidPoint(const Pel *chroma, ptrdiff_t chromaStride, int height, int width,
                                          int log2BlockWidth, int log2BlockHeight,
                                          uint64_t *chromaSampleCountNearMidPoint,
                                          int       chromaSampleCountNearMidPointStride);

  /// Create the encoder members that are shared by all ALF implementations.
  void createSharedEncMembers(const EncCfg *encCfg);
  /// Destroy the encoder members that are shared by all ALF implementations.
  void destroySharedEncMembers();
};

void EncAdaptiveLoopFilter::copyIndices(CtuModes *const ctuModesDst, const CtuModes &ctuModesSrc, ChannelType channel)
{
  copyIndices(*ctuModesDst, ctuModesSrc, channel);
}

void EncAdaptiveLoopFilter::copyIndices(CtuModes &ctuModesDst, const CtuModes *const ctuModesSrc, ChannelType channel)
{
  copyIndices(ctuModesDst, *ctuModesSrc, channel);
}

void EncAdaptiveLoopFilter::copyIndices(CtuModes *const ctuModesDst, const CtuModes *const ctuModesSrc,
                                        ChannelType channel)
{
  copyIndices(*ctuModesDst, *ctuModesSrc, channel);
}

template<class AlfCovarianceT>
float EncAdaptiveLoopFilter::getUnfilteredDistortion(const std::vector<AlfCovarianceT> &cov, const int numClasses)
{
  CHECKD(cov.size() != numClasses, "Covariance vector size does not match number of classes.");

  float dist = 0;
  for (int classIdx = 0; classIdx < numClasses; classIdx++)
  {
    dist += cov[classIdx].pixAcc;
  }
  return dist;
}

void EncAdaptiveLoopFilter::setCtuEnableFlag(CtuModes *const ctuModes, ChannelType channel, const int val)
{
  setCtuEnableFlag(*ctuModes, channel, val);
}

void EncAdaptiveLoopFilter::setSliceEnabledFlag(AlfParamBase &alfSlicePara, ChannelType channel,
                                                const CtuModes *const ctuModes)
{
  setSliceEnabledFlag(alfSlicePara, channel, *ctuModes);
}

#endif
