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

/**
 \file     EncSampleAdaptiveOffset.h
 \brief    estimation part of sample adaptive offset class (header)
 */

#ifndef __ENCSAMPLEADAPTIVEOFFSET__
#define __ENCSAMPLEADAPTIVEOFFSET__

#include "CommonLib/SampleAdaptiveOffset.h"

#include "CABACWriter.h"

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

struct SAOStatData // data structure for SAO statistics
{
  int64_t diff[MAX_NUM_SAO_CLASSES];
  int64_t count[MAX_NUM_SAO_CLASSES];

  SAOStatData() {}
  ~SAOStatData() {}
  void reset()
  {
    ::memset(diff, 0, sizeof(int64_t) * MAX_NUM_SAO_CLASSES);
    ::memset(count, 0, sizeof(int64_t) * MAX_NUM_SAO_CLASSES);
  }
  const SAOStatData &operator=(const SAOStatData &src)
  {
    ::memcpy(diff, src.diff, sizeof(int64_t) * MAX_NUM_SAO_CLASSES);
    ::memcpy(count, src.count, sizeof(int64_t) * MAX_NUM_SAO_CLASSES);
    return *this;
  }
  const SAOStatData &operator+=(const SAOStatData &src)
  {
    for (int i = 0; i < MAX_NUM_SAO_CLASSES; i++)
    {
      diff[i] += src.diff[i];
      count[i] += src.count[i];
    }
    return *this;
  }
};

struct CcSaoStatData
{
  int64_t  diff[MAX_CCSAO_CLASS_NUM];
  uint32_t count[MAX_CCSAO_CLASS_NUM];

  CcSaoStatData() {}
  ~CcSaoStatData() {}
  void reset()
  {
    ::memset(diff, 0, sizeof(int64_t) * MAX_CCSAO_CLASS_NUM);
    ::memset(count, 0, sizeof(uint32_t) * MAX_CCSAO_CLASS_NUM);
  }
  const CcSaoStatData &operator=(const CcSaoStatData &src)
  {
    ::memcpy(diff, src.diff, sizeof(int64_t) * MAX_CCSAO_CLASS_NUM);
    ::memcpy(count, src.count, sizeof(uint32_t) * MAX_CCSAO_CLASS_NUM);
    return *this;
  }
  const CcSaoStatData &operator+=(const CcSaoStatData &src)
  {
    for (int i = 0; i < MAX_CCSAO_CLASS_NUM; i++)
    {
      diff[i] += src.diff[i];
      count[i] += src.count[i];
    }
    return *this;
  }
};

struct CcSaoEncParam
{
  bool     reusePrv;
  int      reusePrvId;
  uint8_t  setNum;
  bool     setEnabled[MAX_CCSAO_SET_NUM];
  uint8_t  setType[MAX_CCSAO_SET_NUM];
  uint16_t candPos[MAX_CCSAO_SET_NUM][MAX_NUM_COMP];
  uint16_t bandNum[MAX_CCSAO_SET_NUM][MAX_NUM_COMP];
  short    offset[MAX_CCSAO_SET_NUM][MAX_CCSAO_CLASS_NUM];
  uint8_t  mapIdxToIdc[MAX_CCSAO_SET_NUM + 1];

  CcSaoEncParam() {}
  ~CcSaoEncParam() {}
  void reset()
  {
    reusePrv   = false;
    reusePrvId = 0;
    setNum     = 0;
    ::memset(setEnabled, false, sizeof(setEnabled));
    ::memset(setType, 0, sizeof(setType));
    ::memset(candPos, 0, sizeof(candPos));
    ::memset(bandNum, 0, sizeof(bandNum));
    ::memset(offset, 0, sizeof(offset));
    ::memset(mapIdxToIdc, 0, sizeof(mapIdxToIdc));
  }
  const CcSaoEncParam &operator=(const CcSaoEncParam &src)
  {
    reusePrv   = src.reusePrv;
    reusePrvId = src.reusePrvId;
    setNum     = src.setNum;
    ::memcpy(setEnabled, src.setEnabled, sizeof(setEnabled));
    ::memcpy(setType, src.setType, sizeof(setType));
    ::memcpy(candPos, src.candPos, sizeof(candPos));
    ::memcpy(bandNum, src.bandNum, sizeof(bandNum));
    ::memcpy(offset, src.offset, sizeof(offset));
    ::memcpy(mapIdxToIdc, src.mapIdxToIdc, sizeof(mapIdxToIdc));
    return *this;
  }
};

class EncSampleAdaptiveOffset : public SampleAdaptiveOffset
{
  using StatDataArray = EnumArray<SAOStatData, SAOModeNewTypes>;

public:
  EncSampleAdaptiveOffset();
  virtual ~EncSampleAdaptiveOffset();

  // interface
  void createEncData(bool isPreDBFSamplesUsed, uint32_t numCTUsPic);
  void destroyEncData();
  void initCABACEstimator(CABACEncoder *cabacEncoder, CtxPool *ctxPool, Slice *pcSlice);
  void SAOProcess(CodingStructure &cs, bool *sliceEnabled, const double *lambdas,
#if ENABLE_QPA
                  const double lambdaChromaWeight,
#endif
                  const bool testSAODisableAtPictureLevel, const double saoEncodingRate,
                  const double saoEncodingRateChroma, const bool isPreDBFSamplesUsed, bool isGreedyMergeEncoding,
                  bool usingTrueOrg, BIFCabacEst *bifCABACEstimator);

  void CCSAOProcess(CodingStructure &cs, const double *lambdas, const int intraPeriod, const int ccsao_mode);

  void disabledRate(CodingStructure &cs, SAOBlkParam *reconParams, const double saoEncodingRate,
                    const double saoEncodingRateChroma);
  void getPreDBFStatistics(CodingStructure &cs, bool usingTrueOrg);

private:   // methods
  void deriveLoopFilterBoundaryAvailability(CodingStructure &cs, const Position &pos, bool &isLeftAvail,
                                            bool &isAboveAvail, bool &isAboveLeftAvail) const;
  void getStatistics(std::vector<StatDataArray *> &blkStats, PelUnitBuf &orgYuv, PelUnitBuf &srcYuv, PelUnitBuf &bifYuv,
                     CodingStructure &cs, bool isCalculatePreDeblockSamples = false);
  void decidePicParams(const Slice &slice, bool *sliceEnabled, const double saoEncodingRate,
                       const double saoEncodingRateChroma);
  void decideBlkParams(CodingStructure &cs, bool *sliceEnabled, std::vector<StatDataArray *> &blkStats,
                       PelUnitBuf &srcYuv, PelUnitBuf &resYuv, SAOBlkParam *reconParams, SAOBlkParam *codedParams,
                       const bool testSAODisableAtPictureLevel,
#if ENABLE_QPA
                       const double chromaWeight,
#endif
                       const double saoEncodingRate, const double saoEncodingRateChroma,
                       const bool isGreedymergeEncoding);
  void    getBlkStats(const CompID compIdx, const int channelBitDepth, StatDataArray &statsDataTypes, Pel *srcBlk,
                      Pel *orgBlk, Pel *bifBlk, ptrdiff_t bifStride, ptrdiff_t srcStride, ptrdiff_t orgStride, int width,
                      int height, bool isLeftAvail, bool isRightAvail, bool isAboveAvail, bool isBelowAvail,
                      bool isAboveLeftAvail, bool isAboveRightAvail, bool isCalculatePreDeblockSamples,
                      int horVirBndryPos[], int verVirBndryPos[], int numHorVirBndry, int numVerVirBndry);
  void    deriveModeNewRDO(const BitDepths &bitDepths, int ctuRsAddr, MergeBlkParams &mergeList, bool *sliceEnabled,
                           std::vector<StatDataArray *> &blkStats, SAOBlkParam &modeParam, double &modeNormCost);
  void    deriveModeMergeRDO(const BitDepths &bitDepths, int ctuRsAddr, MergeBlkParams &mergeList, bool *sliceEnabled,
                             std::vector<StatDataArray *> &blkStats, SAOBlkParam &modeParam, double &modeNormCost);
  int64_t getDistortion(const int channelBitDepth, SAOModeNewTypes typeIdc, int typeAuxInfo, int *offsetVal,
                        SAOStatData &statData);
  void    deriveOffsets(CompID compIdx, const int channelBitDepth, SAOModeNewTypes typeIdc, SAOStatData &statData,
                        int *quantOffsets, int &typeAuxInfo);

  int64_t estSaoDist(int64_t count, int64_t offset, int64_t diffSum, int shift)
  {
    return (count * offset * offset - diffSum * offset * 2) >> shift;
  }

  int  estIterOffset(SAOModeNewTypes typeIdx, double lambda, int offsetInput, int64_t count, int64_t diffSum, int shift,
                     int bitIncrease, int64_t &bestDist, double &bestCost, int offsetTh);
  void addPreDBFStatistics(std::vector<StatDataArray *> &blkStats);

  void       setupCcSaoLambdas(CodingStructure &cs, const double *lambdas);
  void       setupCcSaoSH(CodingStructure &cs, const CPelUnitBuf &orgYuv);
  void       deriveCcSao(CodingStructure &cs, const CompID compID, const CPelUnitBuf &orgYuv, const CPelUnitBuf &srcYuv,
                         const CPelUnitBuf &dstYuv);
  void       setupInitCcSaoParam(CodingStructure &cs, const CompID compID, const int setNum,
                                 int64_t *trainingDistortion[MAX_CCSAO_SET_NUM],
                                 std::vector<CcSaoStatData> (&blkStats)[MAX_CCSAO_SET_NUM],
                                 CcSaoStatData frameStats[MAX_CCSAO_SET_NUM],
                                 std::vector<CcSaoStatData> (&blkStatsEdge)[MAX_CCSAO_SET_NUM],
                                 CcSaoStatData frameStatsEdge[MAX_CCSAO_SET_NUM], CcSaoEncParam &initCcSaoParam,
                                 CcSaoEncParam &bestCcSaoParam, uint8_t *initCcSaoControl, uint8_t *bestCcSaoControl);
  void       setupTempCcSaoParam(CodingStructure &cs, const CompID compID, const int setNum, const int edgeCmp,
                                 const int candPosY, const int bandNumY, const int bandNumU, const int bandNumV,
                                 CcSaoEncParam &tempCcSaoParam, CcSaoEncParam &initCcSaoParam, uint8_t *tempCcSaoControl,
                                 uint8_t *initCcSaoControl, int setType = CCSAO_SET_TYPE_BAND);
  void       setupTempCcSaoParamFromPrv(CodingStructure &cs, const CompID compID, const int prvId,
                                        CcSaoEncParam &tempCcSaoParam, CcSaoPrvParam &prvCcSaoParam,
                                        uint8_t *tempCcSaoControl);
  void       getCcSaoStatistics(CodingStructure &cs, const CompID compID, const CPelUnitBuf &orgYuv,
                                const CPelUnitBuf &srcYuv, const CPelUnitBuf                                   &dstYuv,
                                std::vector<CcSaoStatData> (&blkStats)[MAX_CCSAO_SET_NUM], const CcSaoEncParam &ccSaoParam);
  inline int getCcSaoEdgeStatIdx(const int ctuRsAddr, const int bandIdc, const int edgeCmp, const int edgeIdc,
                                 int edgeDir, const int edgeThr);
  void       resetCcSaoEdgeStats(std::vector<CcSaoStatData>(&blkStatsEdge));
  void       prepareCcSaoEdgeStats(CodingStructure &cs, const CompID compID, const CPelUnitBuf &orgYuv,
                                   const CPelUnitBuf &srcYuv, const CPelUnitBuf               &dstYuv,
                                   std::vector<CcSaoStatData>(&blkStats), const CcSaoEncParam &ccSaoParam);

  void getCcSaoStatisticsEdge(CodingStructure &cs, const CompID compID, const CPelUnitBuf &orgYuv,
                              const CPelUnitBuf &srcYuv, const CPelUnitBuf &dstYuv,
                              std::vector<CcSaoStatData> (&blkStats)[MAX_CCSAO_SET_NUM],
                              std::vector<CcSaoStatData>(&blkStatsEdgePre), const CcSaoEncParam &ccSaoParam);

  inline void getCcSaomSampleStatsEdgePre(int startx, int endx, const ptrdiff_t srcStrideE, const int edgeIdcNum,
                                          const int bitDepth, const int edgePosXA, const int edgePosXB,
                                          const int edgePosYA, const int                       edgePosYB,
                                          std::vector<CcSaoStatData>(&blkStatsEdge), const int ctuRsAddr,
                                          const Pel *srcY, const Pel *srcU, const Pel *srcV, const int chromaScaleX,
                                          int edgeCmp, SAOModeNewTypes edgeDir, const Pel *org, const Pel *dst);

  inline void getCcSaomChromaSampleStatsEdgePre(int startx, int endx, const ptrdiff_t srcStrideE, const int edgeIdcNum,
                                                const int bitDepth, const int edgePosXA, const int edgePosXB,
                                                const int edgePosYA, const int                       edgePosYB,
                                                std::vector<CcSaoStatData>(&blkStatsEdge), const int ctuRsAddr,
                                                const Pel *srcY, const Pel *srcU, const Pel *srcV,
                                                const int chromaScaleX, int edgeCmp, SAOModeNewTypes edgeDir,
                                                const Pel *org, const Pel *dst);

  void getCcSaoBlkStatsEdgePre(CodingStructure &cs, const CompID compID, const ChromaFormat chromaFormat,
                               const int bitDepth, std::vector<CcSaoStatData>(&blkStats), const int ctuRsAddr,
                               const Pel *srcY, const Pel *srcU, const Pel *srcV, const Pel *org, const Pel *dst,
                               const ptrdiff_t srcStrideY, const ptrdiff_t srcStrideU, const ptrdiff_t srcStrideV,
                               const ptrdiff_t orgStride, const ptrdiff_t dstStride, const int width, const int height,
                               bool isLeftAvail, bool isRightAvail, bool isAboveAvail, bool isBelowAvail,
                               bool isAboveLeftAvail, bool isAboveRightAvail);
  void getCcSaoBlkStats(const CompID compID, const ChromaFormat chromaFormat, const int bitDepth, const int setIdx,
                        std::vector<CcSaoStatData> (&blkStats)[MAX_CCSAO_SET_NUM], const int ctuRsAddr,
                        const uint16_t candPosY, const uint16_t bandNumY, const uint16_t bandNumU,
                        const uint16_t bandNumV, const Pel *srcY, const Pel *srcU, const Pel *srcV, const Pel *org,
                        const Pel *dst, const ptrdiff_t srcStrideY, const ptrdiff_t srcStrideU,
                        const ptrdiff_t srcStrideV, const ptrdiff_t orgStride, const ptrdiff_t dstStride,
                        const int width, const int height, bool isLeftAvail, bool isRightAvail, bool isAboveAvail,
                        bool isBelowAvail, bool isAboveLeftAvail, bool isAboveRightAvail);
  void getCcSaoFrameStats(const CompID compID, const int setIdx, const uint8_t *ccSaoControl,
                          std::vector<CcSaoStatData> (&blkStats)[MAX_CCSAO_SET_NUM],
                          CcSaoStatData frameStats[MAX_CCSAO_SET_NUM],
                          std::vector<CcSaoStatData> (&blkStatsEdge)[MAX_CCSAO_SET_NUM],
                          CcSaoStatData frameStatsEdge[MAX_CCSAO_SET_NUM], const uint8_t setType);
  void deriveCcSaoOffsets(const CompID compID, const int bitDepth, const int setIdx,
                          CcSaoStatData frameStats[MAX_CCSAO_SET_NUM],
                          short         offset[MAX_CCSAO_SET_NUM][MAX_CCSAO_CLASS_NUM]);
  inline int estCcSaoIterOffset(const double lambda, const int offsetInput, const int64_t count, const int64_t diffSum,
                                const int shift, const int bitIncrease, int64_t &bestDist, double &bestCost,
                                const int offsetTh);
  void       getCcSaoDistortion(const CompID compID, const int setIdx,
                                std::vector<CcSaoStatData> (&blkStats)[MAX_CCSAO_SET_NUM],
                                short    offset[MAX_CCSAO_SET_NUM][MAX_CCSAO_CLASS_NUM],
                                int64_t *trainingDistortion[MAX_CCSAO_SET_NUM]);
  void       deriveCcSaoRDO(CodingStructure &cs, const CompID compID, int64_t *trainingDistortion[MAX_CCSAO_SET_NUM],
                            std::vector<CcSaoStatData> (&blkStats)[MAX_CCSAO_SET_NUM],
                            CcSaoStatData frameStats[MAX_CCSAO_SET_NUM],
                            std::vector<CcSaoStatData> (&blkStatsEdge)[MAX_CCSAO_SET_NUM],
                            CcSaoStatData frameStatsEdge[MAX_CCSAO_SET_NUM], CcSaoEncParam &bestCcSaoParam,
                            CcSaoEncParam &tempCcSaoParam, uint8_t *bestCcSaoControl, uint8_t *tempCcSaoControl,
                            double &bestCost, double &tempCost);
  void determineCcSaoControlIdc(CodingStructure &cs, const CompID compID, const int ctuWidthC, const int ctuHeightC,
                                const int picWidthC, const int picHeightC, CcSaoEncParam &ccSaoParam,
                                uint8_t *ccSaoControl, int64_t *trainingDistorsion[MAX_CCSAO_SET_NUM],
                                int64_t &curTotalDist, double &curTotalRate);
  int  getCcSaoParamRate(const CompID compID, const CcSaoEncParam &ccSaoParam);
  int  lengthUvlc(int uiCode);
  int  getCcSaoClassNumEnc(const int setIdx, const CcSaoEncParam &ccSaoParam);   // for CcSaoEncParam
public:
  void setupCcSaoPrv(CodingStructure &cs);

private: // members
  // for RDO
  CABACWriter *m_CABACEstimator { nullptr };
  CtxPool     *m_ctxPool { nullptr };
  double       m_lambda[MAX_NUM_COMP];

  // statistics
  std::vector<StatDataArray *> m_statData;   //[ctu][comp][classes]
  std::vector<StatDataArray *> m_preDBFstatData;

  double m_saoDisabledRate[MAX_NUM_COMP][MAX_TLAYER];

  EnumArray<int, SAOModeNewTypes> m_skipLinesR[MAX_NUM_COMP];
  EnumArray<int, SAOModeNewTypes> m_skipLinesB[MAX_NUM_COMP];

  bool                       m_createdEnc = false;
  int                        m_intraPeriod;
  bool                       m_extChroma = false;
  std::vector<CcSaoStatData> m_ccSaoStatData[MAX_CCSAO_SET_NUM];
  CcSaoStatData              m_ccSaoStatFrame[MAX_CCSAO_SET_NUM];
  std::vector<CcSaoStatData> m_ccSaoStatDataEdge[MAX_CCSAO_SET_NUM];
  CcSaoStatData              m_ccSaoStatFrameEdge[MAX_CCSAO_SET_NUM];
  std::vector<CcSaoStatData> m_ccSaoStatDataEdgePre;
  CcSaoEncParam              m_bestCcSaoParam;
  CcSaoEncParam              m_tempCcSaoParam;
  CcSaoEncParam              m_initCcSaoParam;
  uint8_t                   *m_bestCcSaoControl;
  uint8_t                   *m_tempCcSaoControl;
  uint8_t                   *m_initCcSaoControl;
  int64_t                   *m_trainingDistortion[MAX_CCSAO_SET_NUM];
  int                        m_CCSaoMode = 0;

  std::vector<CcSaoPrvParam> m_ccSaoPrvParamEnc[MAX_NUM_COMP];
};
//! \}

#endif
