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

/** \file     EncAdaptiveLoopFilterVtm.h
 \brief    estimation part of adaptive loop filter class (header)
 */

#ifndef __ENCADAPTIVELOOPFILTERVTM__
#define __ENCADAPTIVELOOPFILTERVTM__

#include "CommonLib/AdaptiveLoopFilterVtm.h"
#include "EncAdaptiveLoopFilterBase.h"

struct AlfCovarianceVtm : public AlfCovarianceBase<AlfParametersVtm>
{
  inline float optimizeFilter(const int size, int *clip, float *f, bool optimizeClip) const;
  inline float optimizeFilterClip(const int size, int *clip) const;

  AlfCovarianceVtm()                         = default;
  AlfCovarianceVtm(const AlfCovarianceVtm &) = default;

private:
  using BaseT = AlfCovarianceBase<AlfParametersVtm>;
};

#ifdef _MSC_VER
// Temporarily disable Visual Studio warning 4250 (inheritance via dominance).
#pragma warning(push)
#pragma warning(disable : 4250)
#endif

class EncAdaptiveLoopFilterVtm : virtual public EncAdaptiveLoopFilterBase<AlfParametersVtm>,
                                 virtual public AdaptiveLoopFilterVtm
{
public:
  using AlfCovariance = AlfCovarianceVtm;

private:
  using CovarianceVec1D = std::vector<AlfCovariance>;
  using CovarianceVec2D = std::vector<CovarianceVec1D>;
  using CovarianceVec3D = std::vector<CovarianceVec2D>;

  using CovMergedArr1D = std::array<AlfCovariance, MAX_NUM_ALF_CLASSES + 2>;
  using CovMergedArr2D = std::array<CovMergedArr1D, ALF_NUM_OF_FILTER_TYPES>;

  // clang-format off
  std::array<CovarianceVec3D, MAX_NUM_COMP> m_alfCovariance;             // [compIdx][shapeIdx][ctbAddr][classIdx]
  EnumArray<CovarianceVec2D, ChannelType>   m_alfCovarianceFrame;        // [CHANNEL][shapeIdx][lumaClassIdx/chromaAltIdx]
  CovarianceVec2D                           m_alfCovarianceCcAlf;        // [shapeIdx][filterIdx][ctbAddr]
  CovarianceVec1D                           m_alfCovarianceFrameCcAlf;   // [shapeIdx][filterIdx]
  // clang-format on

  // for RDO
  AlfParam       m_alfParamTemp;
  CovMergedArr2D m_alfCovarianceMerged;
  int m_alfClipMerged[ALF_NUM_OF_FILTER_TYPES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF];

  float   *m_distCtbApsLuma[ALF_CTB_MAX_NUM_APS];
  float   *m_distCtbLumaNewFilt;
  float   *m_ctbDistortionFixedFilter[ALF_NUM_FIXED_FILTER_SETS];
  int      m_clipDefaultEnc[MAX_NUM_ALF_LUMA_COEFF];
  AlfParam m_alfParamTempNL;
  int      m_filterTmp[MAX_NUM_ALF_LUMA_COEFF];
  int      m_clipTmp[MAX_NUM_ALF_LUMA_COEFF];

  short     m_bestFilterCoeffSet[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF];
  bool      m_bestFilterIdxEnabled[MAX_NUM_CC_ALF_FILTERS];
  uint64_t *m_trainingDistortion[MAX_NUM_CC_ALF_FILTERS];   // for current block size

public:
  EncAdaptiveLoopFilterVtm();

  void         create(const EncCfg *encCfg, const int picWidth, const int picHeight, const ChromaFormat chromaFormatIdc,
                      const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const BitDepths &inputBitDepth);
  virtual void destroy() override;

  template<bool alfWSSD> void initDistortion();
  void                        alfEncoderCtb(CodingStructure &cs, AlfParam &alfParamNewFilters
#if ENABLE_QPA
                     ,
                     const double lambdaChromaWeight
#endif
  );
  void alfReconstructor(CodingStructure &cs, const PelUnitBuf &recExtBuf);
  void ALFProcess(CodingStructure &cs, const double *lambdas,
#if ENABLE_QPA
                  const double lambdaChromaWeight,
#endif
                  Picture *pic, uint32_t numSliceSegments);

  void getDistApsFilter(const CodingStructure &cs, const AlfApsList &apsIds);
  void getDistNewFilter(AlfParam &alfParam);

private:
  void firstPass(CodingStructure &cs, AlfParam &alfParam, const PelUnitBuf &orgUnitBuf, const PelUnitBuf &recExtBuf,
                 const PelUnitBuf &recBuf, const ChannelType channel
#if ENABLE_QPA
                 ,
                 const double lambdaChromaWeight = 0.0
#endif
  );

  void  copyAlfParam(AlfParam &alfParamDst, AlfParam &alfParamSrc, ChannelType channel);
  float mergeFiltersAndCost(AlfParam &alfParam, AlfFilterShape &alfShape, const CovarianceVec1D &covFrame,
                            CovMergedArr1D &covMerged,
                            int  clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF],
                            int &coeffBitsFinal);

  void getFrameStats(ChannelType channel, int shapeIdx, int altIdx);

  void getFrameStat(CovarianceVec1D &frameCov, const CovarianceVec2D &ctbCov, const CtbModes &ctbModes,
                    const int numClasses, int altIdx) const;

  template<bool alfWSSD> void deriveStatsForFiltering(PelUnitBuf &orgYuv, PelUnitBuf &recYuv, CodingStructure &cs);

  void getBlkStats(CovarianceVec1D &alfCovariance, const AlfFilterShape &shape, const ClassBuf *const classifier,
                   const Pel *org, const ptrdiff_t orgStride, const Pel *orgLuma, const ptrdiff_t orgLumaStride,
                   const Pel *rec, const ptrdiff_t recStride, const CompArea &areaDst, const CompArea &area,
                   const ChannelType channel, int vbCTUHeight, int vbPos);

  void calcCovariance(Pel ELocal[MAX_NUM_ALF_LUMA_COEFF][MAX_ALF_NUM_CLIP_VALS], const Pel *rec, const ptrdiff_t stride,
                      const AlfFilterShape &shape, const int transposeIdx, const ChannelType channel, int vbDistance);

  template<bool alfWSSD> void   deriveStatsForCcAlfFiltering(const PelUnitBuf &orgYuv, const PelUnitBuf &recYuv,
                                                             const int compIdx, CodingStructure &cs);
  template<bool m_alfWSSD> void getBlkStatsCcAlf(AlfCovariance &alfCovariance, const AlfFilterShape &shape,
                                                 const PelUnitBuf &orgYuv, const PelUnitBuf &recYuv,
                                                 const UnitArea &areaDst, const UnitArea &area, const CompID compID,
                                                 const int yPos);

  void calcCovarianceCcAlf(Pel ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][1], const Pel *rec, const ptrdiff_t stride,
                           const AlfFilterShape &shape, int vbDistance);

  void mergeClasses(const AlfFilterShape &alfShape, const CovarianceVec1D &cov, CovMergedArr1D &covMerged,
                    int       clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF],
                    const int numClasses, short filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES]) const;

  float getFilterCoeffAndCost(CodingStructure &cs, float distUnfilter, ChannelType channel, bool bReCollectStat,
                              int shapeIdx, int &coeffBits, bool onlyFilterCost = false);

  float deriveFilterCoeffs(const CovarianceVec1D &cov, CovMergedArr1D &covMerged,
                           int             clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF],
                           AlfFilterShape &alfShape, const short *const filterIndices, const int numFilters,
                           float errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2]);

  int deriveFilterCoefficientsPredictionMode(const AlfFilterShape &alfShape, const int *const *const filterSet,
                                             const int numFilters);

  float deriveCoeffQuant(int *const filterClipp, int *const filterCoeffQuant, const AlfCovariance &cov,
                         const AlfFilterShape &shape, const int fractionalBits, const bool optimizeClip);

  float deriveCtbAlfEnableFlags(CodingStructure &cs, const int shapeIdx, ChannelType channel,
#if ENABLE_QPA
                                const double chromaWeight,
#endif
                                const int numClasses, const int numCoeff, float &distUnfilter);

  int getNonFilterCoeffRate(AlfParam &alfParam);

  int getCostFilterCoeffForce0(AlfFilterShape &alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters,
                               bool *codedVarBins);
  int lengthFilterCoeffs(const AlfFilterShape &alfShape, const int numFilters,
                         const int *const *const filterCoeff) const;

  float getDistForce0(AlfFilterShape &alfShape, const int numFilters, float errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2],
                      bool *codedVarBins);

  int getChromaCoeffRate(AlfParam &alfParam, int altIdx) const;

  float getFilteredDistortion(const CovarianceVec1D &cov, const int numClasses, const int numFiltersMinus1,
                              const int numCoeff) const;

  void        initCtuAlternativeChroma(CtuModes &ctuModes) const;
  inline void initCtuAlternativeChroma(CtuModes *const ctuModes) const;
  void        deriveCcAlfFilterCoeff(CompID compID, const PelUnitBuf &recYuv, const PelUnitBuf &recYuvExt,
                                     short         filterCoeff[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF],
                                     const uint8_t filterIdx);
  void deriveCcAlfFilter(CodingStructure &cs, CompID compID, const PelUnitBuf &orgYuv, const PelUnitBuf &tempDecYuvBuf,
                         const PelUnitBuf &dstYuv);
  inline void xSetupCcAlfAPS(const CodingStructure &cs);
  void        countLumaSwingGreaterThanThreshold(const Pel *luma, ptrdiff_t lumaStride, int height, int width,
                                                 int log2BlockWidth, int log2BlockHeight,
                                                 uint64_t *lumaSwingGreaterThanThresholdCount, int lumaCountStride);
  void        getFrameStatsCcalf(CompID compIdx, int filterIdc);
  void        initDistortionCcalf(const CompID compId);
};

#ifdef _MSC_VER
// Restore all Visual Studio warning settings.
#pragma warning(pop)
#endif

float AlfCovarianceVtm::optimizeFilter(const int size, int *clip, float *f, bool optimizeClip) const
{
  return BaseT::optimizeFilter<false>(size, clip, f, optimizeClip);
}

float AlfCovarianceVtm::optimizeFilterClip(const int size, int *clip) const
{
  return BaseT::optimizeFilterClip<false>(size, clip);
}

void EncAdaptiveLoopFilterVtm::initCtuAlternativeChroma(CtuModes *const ctuModes) const
{
  initCtuAlternativeChroma(*ctuModes);
}

#endif // __ENCADAPTIVELOOPFILTERVTM__
