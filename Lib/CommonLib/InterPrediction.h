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

/** \file     InterPrediction.h
    \brief    inter prediction class (header)
*/

#ifndef __INTERPREDICTION__
#define __INTERPREDICTION__

// Include files
#include "CommonDef.h"
#include "InterpolationFilter.h"
#include "WeightPrediction.h"

#include "Buffer.h"
#include "Unit.h"
#include "Picture.h"

#include "RdCost.h"
// forward declaration
class Mv;
class IntraPrediction;
class Reshape;
//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class MergeCtx
{
public:
  MergeCtx() : numValidMergeCand(0), numCandToTestEnc(0), hasMergedCandList(false) {}
  ~MergeCtx() {}

public:
  MvField mvFieldNeighbours[MRG_MAX_NUM_CANDS][2];
  uint8_t bcwIdx[MRG_MAX_NUM_CANDS];
  uint8_t interDirNeighbours[MRG_MAX_NUM_CANDS];
  int     numValidMergeCand;
  int     numCandToTestEnc;
  bool    hasMergedCandList;

  MvField mmvdBaseMv[MmvdIdx::BASE_MV_NUM][2];
  bool    mmvdUseAltHpelIf[MmvdIdx::BASE_MV_NUM];
  bool    useAltHpelIf[MRG_MAX_NUM_CANDS];
  bool    LICFlags[MRG_MAX_NUM_CANDS];

  void setMmvdMergeCandiInfo(CodingUnit &cu, MmvdIdx candIdx);
  void getMmvdDeltaMv(const Slice &slice, const MmvdIdx candIdx, Mv deltaMv[NUM_RPL01]) const;
  void setMergeInfo(CodingUnit &cu, int mergeIdx) const;
  void setGeoMmvdMergeInfo(CodingUnit &cu, int mergeIdx, int mmvdIdx) const;
  bool checkSimilarMotion(int mergeCandIndex, uint32_t mvdSimilarityThresh = 1) const;
  void initMrgCand(int mergeCandIndex);
};

using GeoMergeCtx = MergeCtx[GEO_NUM_TM_MV_CAND];

class AffineMergeCtx
{
public:
  AffineMergeCtx() : numValidMergeCand(0)
  {
    for (int i = 0; i < RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE; i++)
    {
      affineType[i] = AffineModel::_4_PARAMS;
    }
  }
  ~AffineMergeCtx() {}

public:
  std::array<MvField[2], AFFINE_MAX_NUM_CP> mvFieldNeighbours[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE];
  uint8_t                                   interDirNeighbours[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE];
  Distortion                                candCost[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE];
  AffineModel                               affineType[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE];
  uint8_t                                   bcwIdx[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE];
  bool                                      LICFlags[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE];
  int                                       numValidMergeCand;
  int                                       maxNumMergeCand;
  int                                       numAffCandToTestEnc;

  MergeType mergeType[RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE];
  MotionBuf subPuMvpMiBuf;
  bool      xCheckSimilarMotion(int mergeCandIndex, uint32_t mvdSimilarityThresh = 1) const;
};

struct BMSubBlkInfo : Area
{
  bool    m_predReady;
  Mv      m_mv[2];
  Mv      m_mvRefine[2];
  Mv      m_mvRefineL0[2];
  Mv      m_mvRefineL1[2];
  PosType m_cXInPU; // x coordinate of center relative to PU's top left sample
  PosType m_cYInPU; // y coordinate of center relative to PU's top left sample
};

class InterPrediction : public WeightPrediction
{
private:
  const Picture *m_bmRefPic[2];
  CPelBuf        m_bmRefBuf[2];
  PelBuf         m_bmInterpolationTmpBuf;
  int            m_bmFilterSize;
  int            m_bmInterpolationHOfst;
  int            m_bmInterpolationVOfst;

  ClpRng       m_bmClpRng;
  ChromaFormat m_bmChFmt;
  PelBuf       m_bmPredBuf[2];
  DistParam    m_bmDistParam;
  int32_t      m_bmCostShift;   // bilateral matching cost shift

  int32_t                   m_bmSubBlkW;
  int32_t                   m_bmSubBlkH;
  std::vector<BMSubBlkInfo> m_bmSubBlkList;

protected:
  InterpolationFilter *m_if;

  Pel *m_acYuvPred[NUM_RPL01][MAX_NUM_COMP];
  Pel *m_filteredBlock[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL]
                      [LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL][MAX_NUM_COMP];
  Pel *m_filteredBlockTmp[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL][MAX_NUM_COMP];

  // one vector for each subblock
  Pel *m_dmvrRightBoundary[MAX_CU_SIZE / DMVR_SUBCU_HEIGHT * MAX_CU_SIZE / DMVR_SUBCU_WIDTH];
  Pel *m_dmvrBottomBoundary[MAX_CU_SIZE / DMVR_SUBCU_HEIGHT * MAX_CU_SIZE / DMVR_SUBCU_WIDTH];

  static constexpr int TMP_RPR_WIDTH  = MAX_CU_SIZE + 16;
  static constexpr int TMP_RPR_HEIGHT = MAX_CU_SIZE * MAX_SCALING_RATIO + 16;

  Pel *m_filteredBlockTmpRPR;

  ChromaFormat m_currChromaFormat;

  CompID m_maxCompIDToPred;   ///< tells the predictor to only process the components up to (inklusive) this one -
                              ///< useful to skip chroma components during RD-search

  RdCost *m_pcRdCost;

  PelStorage m_geoPartBuf[2];

  static constexpr int MVBUFFER_SIZE = MAX_CU_SIZE / MIN_PU_SIZE;

  Mv    *m_storedMv;
  // buffers for initial prediction that is used to calculate DMVR refinement
  Pel   *m_yuvPredTempDmvr[NUM_RPL01];
  PelBuf m_dmvrInitialPred[NUM_RPL01];

  // buffers for padded data, initially filled by xDmvrPrefetch(), padded by xDmvrPad()
  Pel       *m_refSamplesDmvr[NUM_RPL01][MAX_NUM_COMP];
  PelUnitBuf m_yuvRefBufDmvr[NUM_RPL01];

  Mv       m_pSearchEnlargeOffset_bilMrg[5][BDMVR_INTME_AREA];
  uint16_t m_pSearchEnlargeOffsetToIdx[5][BDMVR_INTME_AREA];
  uint16_t m_pSearchEnlargeOffsetNum[5];
  uint64_t m_SADsEnlargeArray_bilMrg[BDMVR_INTME_AREA];
  int      m_costShift_1_bilMrg[BDMVR_INTME_AREA];
  int      m_costShift_2_bilMrg[BDMVR_INTME_AREA];

  static constexpr int AFFINE_SUBBLOCK_WIDTH_EXT  = AFFINE_SUBBLOCK_SIZE + 2 * PROF_BORDER_EXT_W;
  static constexpr int AFFINE_SUBBLOCK_HEIGHT_EXT = AFFINE_SUBBLOCK_SIZE + 2 * PROF_BORDER_EXT_H;

  // PROF skip flags for encoder speedup
  bool m_skipProf { false };
  bool m_skipProfCond { false };
  bool m_biPredSearchAffine { false };

  Pel *m_gradX0;
  Pel *m_gradX1;
  Pel *m_gradY0;
  Pel *m_gradY1;
  Pel *m_absGx;
  Pel *m_absGy;
  Pel *m_dIx;
  Pel *m_dIy;
  Pel *m_dI;
  Pel *m_signGxGy;
  int *m_tmpxSample32bit;
  int *m_tmpySample32bit;
  int *m_sumAbsGxSample32bit;
  int *m_sumAbsGySample32bit;
  int *m_sumDIXSample32bit;
  int *m_sumDIYSample32bit;
  int *m_sumSignGyGxSample32bit;
  bool m_bdofMvRefined;
  Mv   m_bdofSubPuMvOffset[BDOF_SUBPU_MAX_NUM];
  Mv   m_bdofSubPuMvOffse2[BDOF_SUBPU_MAX_NUM];
  bool m_subPuMC;

  PelStorage m_obmcTmp1;
  PelStorage m_obmcTmp2;
  PelStorage m_obmcTmp3;
  PelUnitBuf m_obmcSubPred[4];

  int        m_ibcBufferWidth;
  int        m_ibcBufferHeight;
  PelStorage m_ibcBuffer;
  int32_t   *m_piDotProduct[5];

  bool  m_isOOB[NUM_RPL01];
  bool *m_mcMask[NUM_RPL01];
  bool *m_mcMaskChroma[NUM_RPL01];

  Reshape *m_reshape;

  void xIntraBlockCopy(CodingUnit &cu, PelUnitBuf &predBuf, const CompID compID);
  int  rightShiftMSB(int numer, int denom);
  template<bool dmvdBDOFExt>
  inline void applyBiOptFlow(const CodingUnit &cu, const bool isBdofMvRefine, const int bdofBlockOffset,
                             const CPelUnitBuf &yuvSrc0, const CPelUnitBuf &yuvSrc1, const int &refIdx0,
                             const int &refIdx1, PelUnitBuf &yuvDst, const BitDepths &clipBitDepths,
                             bool *mcMask[NUM_RPL01], bool *mcMaskChroma[NUM_RPL01], bool *isOOB,
                             const bool scaleMvBDOF, const bool reduceSubPuSizeBDOF, int iter = 0);
  void        xPredInterUni(const CodingUnit &cu, const RefPicList &eRefPicList, PelUnitBuf &pcYuvPred, const bool bi,
                            const bool bioApplied, const bool luma, const bool chroma, const bool isBdofMvRefine = false);
  void        xPredInterBiSubPuBDOF(const CodingUnit &cu, PelUnitBuf &pcYuvPred, const bool luma, const bool chroma);
  void        xPredInterBiBDMVR(CodingUnit &cu, PelUnitBuf &pcYuvPred, const bool luma, const bool chroma,
                                PelUnitBuf *yuvPredTmp = NULL);
  void        xDoPredBiBDMVR(const CodingUnit &cu, CodingUnit &subPu, PelUnitBuf &pcYuvPred, PelUnitBuf &srcPred0,
                             PelUnitBuf &srcPred1, PelUnitBuf *yuvPredTmp, bool bioApplied, bool luma, bool chroma,
                             int bioSubPuIdx, const bool scaleMvBDOF, const bool reduceSubPuSizeBDOF, int iter = 0);
  void        xPredInterBiBDMVR2(CodingUnit &pu, PelUnitBuf &pcYuvPred, const bool luma, const bool chroma,
                                 PelUnitBuf *yuvPredTmp = NULL, int iter = 1);
  void        xPredInterBi(CodingUnit &cu, PelUnitBuf &pcYuvPred, bool luma, bool chroma, PelUnitBuf *yuvPredTmp);

  void xPredInterBlk(const CompID compID, const CodingUnit &cu, const Picture *refPic, const Mv &_mv,
                     PelUnitBuf &dstPic, bool bi, const ClpRng &clpRng, bool bioApplied, bool isIBC, RefPicList l,
                     const ScalingRatio scalingRatio = SCALE_1X);
  void xPredIBCBlkPadding(const CodingUnit &cu, const CompID compID, const Picture *refPic, const ClpRng &clpRng,
                          CPelBuf &refBufBeforePadding, const Position &refOffsetByIntBv, int xFrac, int yFrac,
                          int width, int height, InterpolationFilter::Filter filterIdx);

  void subBlockBiOptFlow(Pel *dstY, const int dstStride, const Pel *src0, const int src0Stride, const Pel *src1,
                         const int src1Stride, int bioParamOffset, const int bioParamStride, int width, int height,
                         const ClpRng &clpRng, const int shiftNum, const int offset, const int limit, bool *mcMask[2],
                         int mcStride, bool *isOOB = NULL);
  void xBioGradFilter(Pel *pSrc, ptrdiff_t srcStride, int width, int height, ptrdiff_t gradStride, Pel *gradX,
                      Pel *gradY, int bitDepth);

  void        xWeightedAverage(const CodingUnit &cu, const bool isBdofMvRefine, const int bdofBlockOffset,
                               const CPelUnitBuf &pcYuvSrc0, const CPelUnitBuf &pcYuvSrc1, PelUnitBuf &pcYuvDst,
                               bool bioApplied, bool lumaOnly, bool chromaOnly, PelUnitBuf *yuvDstTmp, bool *mcMask[NUM_RPL01],
                               int mcStride, bool *mcMaskChroma[NUM_RPL01], int mcCStride, bool *isOOB,
                               const bool scaleMvBDOF = false, const bool reduceSubPuSizeBDOF = false, int iter = 0);
  void        xPredAffineBlk(const CompID &compID, const CodingUnit &cu, const Picture *refPic, const Mv *_mv,
                             PelUnitBuf &dstPic, const bool bi, const ClpRng &clpRng, RefPicList eRefPicList,
                             const bool genChromaMv = false, const ScalingRatio = SCALE_1X, const bool calGradient = false);
  static bool xCheckIdenticalMotion(const CodingUnit &cu);
  template<bool dmvdBDOFExt> inline void xStoreBDOFMvDataExt(Mv &bioMv, const int bdofBlockOffset,
                                                             const int bioSubPuMvIndex, const int bioDx,
                                                             const int bioDy, const bool scaleMvBDOF, const int iter);
  template<bool dmvdBDOFExt> void        xResetBDOFMvData(const int bdofBlockOffset, const int bioSubPuMvIndex,
                                                          const int bioDx, const int bioDy, int iter);
  void xSubPuMC(CodingUnit &cu, PelUnitBuf &predBuf, const RefPicList &eRefPicList, bool luma, bool chroma);
  void destroy();

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  CacheModel *m_cacheModel;
#endif
  PelStorage m_colorTransResiBuf;
  bool       m_encMotionEstimation = false;

public:
  InterPrediction();
  virtual ~InterPrediction();

  void init(RdCost *pcRdCost, Reshape *pcReshape, ChromaFormat chromaFormatIdc, const int ctuSize, const int picWidth,
            InterpolationFilter *pcInterpolationFilter);

  bool isMvOOB(const Mv &rcMv, const Position pos, const Size size, const SPS *sps, const PPS *pps, bool *mcMask,
               bool *mcMaskChroma, bool lumaOnly = false);
  bool isMvOOBSubBlk(const Mv &rcMv, const Position pos, const Size size, const SPS *sps, const PPS *pps, bool *mcMask,
                     int mcStride, bool *mcMaskChroma, int mcCStride, bool lumaOnly = false);

  // inter
  void motionCompensation(CodingUnit &cu, PelUnitBuf &predBuf, RefPicList eRefPicList, bool luma, bool chroma,
                          PelUnitBuf *predBufWOBIO, bool obmc);

  void motionCompensation(CodingUnit &cu, PelUnitBuf &predBuf, RefPicList eRefPicList)
  {
    bool chromaEnabled = isChromaEnabled(cu.chromaFormat);
    motionCompensation(cu, predBuf, eRefPicList, true, chromaEnabled, nullptr, false);
  }

  void motionCompensationGeo(CodingUnit &cu, const GeoMergeCtx &geoMrgCtx, IntraPrediction *pcIntraPred,
                             std::vector<Pel> *reshapeLUT);
  void weightedGeo(CodingUnit &cu, const ChannelType channel, PelUnitBuf &predDst, PelUnitBuf &predSrc0,
                   PelUnitBuf &predSrc1);
  void weightedGeoRounded(CodingUnit &pu, const ChannelType channel, PelUnitBuf &predDst, PelUnitBuf &predSrc0,
                          PelUnitBuf &predSrc1);

  bool        obmcFilter(const CodingUnit &cu, PelUnitBuf &predBuf, bool luma, bool chroma);
  inline void getPredIBCBlk(const CodingUnit &cu, const CompID comp, const Picture *refPic, const Mv &_mv,
                            PelUnitBuf &dstPic)
  {
    xPredInterBlk(comp, cu, refPic, _mv, dstPic, false, cu.slice->clpRng(comp), false, true, RPL0);
  }
  uint32_t checkValidBvPU(const CodingUnit &cu, CompID compID, Mv mv, bool ignoreFracMv = false,
                          InterpolationFilter::Filter filterIdx = InterpolationFilter::Filter::DEFAULT);
  uint32_t checkValidBv(const CodingUnit &cu, CompID compID, int compWidth, int compHeight, Mv mv,
                        bool                        ignoreFracMv = false,
                        InterpolationFilter::Filter filterIdx    = InterpolationFilter::Filter::DEFAULT,
                        bool                        isFinalMC    = false   // this flag is for non-normative SW speedup
                        ,
                        bool checkAllRefValid = false);
  bool searchBv(const CodingUnit &cu, int xPos, int yPos, int width, int height, int picWidth, int picHeight, int xBv,
                int yBv, int ctuSize, int xFilterTap = 1, int yFilterTap = 1, CompID compID = COMP_Y);
  // DMVR related definitions
  using DmvrDist = int32_t;

private:
  void xObmcIsSkip(const CodingUnit &cu, const PelUnitBuf &predBuf, bool skipObmc[3]);
  bool xObmcCheckSkip(const CodingUnit &cu, CompID comp, const PelUnitBuf &predBuf, PelUnitBuf &tmpBuf);
  void xObmcFetchPred(const CodingUnit &cu, PelUnitBuf &tmpBuf, const bool skipObmc[3]);
  void xObmcOverlap(const CodingUnit &subCu, int dir, PelUnitBuf &predBuf, PelUnitBuf &tmpBuf, const bool skipObmc[3]);
  bool xObmcSubBlock(const CodingUnit &cu, CodingUnit &subCu, PelUnitBuf &predBuf, const bool doVer[2],
                     const bool doHor[2], bool luma, bool chroma);
  void xObmcSubBlockBlend(const CodingUnit &subCu, PelUnitBuf &predBuf, CompID comp, const int weights[4][2][4]);

public:
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  void cacheAssign(CacheModel *cache);
#endif

  void xFillIBCBuffer(CodingUnit &cu);
  void resetIBCBuffer(const ChromaFormat chromaFormatIdc, const int ctuSize);
  void resetVPDUforIBC(const ChromaFormat chromaFormatIdc, const int ctuSize, const int vSize, const int xPos,
                       const int yPos);
  bool isLumaBvValid(const int ctuSize, const int xCb, const int yCb, const int width, const int height, const int xBv,
                     const int yBv);

  bool xPredInterBlkRPR(const ScalingRatio scalingRatio, const PPS &pps, const CompArea &blk, const Picture *refPic,
                        const Mv &mv, Pel *dst, const ptrdiff_t dstStride, const bool bi, const bool wrapRef,
                        const ClpRng &clpRng, const InterpolationFilter::Filter filterIndex,
                        const bool useAltHpelIf = false);

  static Distortion getDecoderSideDerivedMvCost(const Mv &mvStart, const Mv &mvCur, int searchRangeInFullPel,
                                                int weight);
  void              xBDMVRUpdateSquareSearchCostLog(Distortion *costLog, int bestDirect);

private:
  void xBDMVRFillBlkPredPelBuffer(const CodingUnit &cu, const Picture &refPic, const Mv &_mv, PelBuf &dstBuf,
                                  const ClpRng &clpRng);
  template<uint8_t dir> void xBDMVRPreInterpolation(const CodingUnit &cu, const Mv (&mvCenter)[2],
                                                    bool doPreInterpolationFP, bool doPreInterpolationHP);

  Distortion xBDMVRGetMatchingError(const CodingUnit &cu, const Mv (&mv)[2], bool useMR, int useHadmard = 0);
  template<uint8_t dir>
  Distortion xBDMVRGetMatchingError(const CodingUnit &cu, const Mv (&mv)[2], const int subPuOffset, int useHadmard,
                                    bool useMR, bool &doPreInterpolation, int32_t          searchStepShift,
                                    const Mv (&mvCenter)[2], const Mv (&mvInitial)[2], int nDirect);

  template<bool adaptRange, bool useHadamard>
  Distortion xBDMVRMvIntPelFullSearch(Mv &mvOffset, Distortion curBestCost, const Mv (&initialMv)[2],
                                      const int32_t maxSearchRounds, const int maxHorOffset, const int maxVerOffset,
                                      const bool earlySkip, const Distortion earlyTerminateTh, DistParam &cDistParam,
                                      Pel *pelBuffer[2], const int stride);
  template<bool hPel> Distortion   xBDMVRMvSquareSearch(Mv (&curBestMv)[2], Distortion curBestCost, CodingUnit &cu,
                                                        const Mv (&initialMv)[2], int32_t maxSearchRounds,
                                                        int32_t searchStepShift, bool useMR, int useHadmard);
  template<uint8_t dir> Distortion xBDMVRMvOneTemplateHPelSquareSearch(Mv (&curBestMv)[2], Distortion curBestCost,
                                                                       CodingUnit &cu, const Mv (&initialMv)[2],
                                                                       int32_t maxSearchRounds, int32_t searchStepShift,
                                                                       bool useMR, int useHadmard);

  void xBmAffineInit(const CodingUnit &pu);
  void xBmInitAffineSubBlocks(const Position puPos, const int width, const int height, const int dx, const int dy,
                              int mvScaleHor[2], int mvScaleVer[2], int deltaMvHorX[2], int deltaMvHorY[2],
                              int deltaMvVerX[2], int deltaMvVerY[2]);
  void xBmAffineIntSearch(
    const CodingUnit &pu, Mv (&mvOffset)[2], Distortion &minCost,
    Distortion totalCost[(AFFINE_DMVR_INT_SRCH_RANGE << 1) + 1][(AFFINE_DMVR_INT_SRCH_RANGE << 1) + 1]);
  void xBmAffineHPelSearch(const CodingUnit &pu, Mv (&mvOffset)[2], Distortion &minCost, Distortion localCostArray[9]);

  void xInitBilateralMatching(const int width, const int height, const int bitDepth, const bool useMR,
                              const int useHadmard);
  void xDeriveCPMV(CodingUnit &pu, const Mv (&curShiftMv)[2][3], int deltaMvHorX, int deltaMvHorY, int deltaMvVerX,
                   int deltaMvVerY, int baseCP, Mv (&cpMV)[2][3]);
  Distortion xBDMVRMv6ParameterSearchAffine(Distortion curBestCost, CodingUnit &pu);
  Distortion xGetBilateralMatchingErrorAffine(const CodingUnit &pu, Mv (&mvAffi)[2][3]);
  Distortion xGetBilateralMatchingErrorAffineForMvOffset(const CodingUnit &pu, Mv (&mvOffset)[2]);
  template<bool checkMv> Distortion xGetBilateralMatchingErrorAffineCheckMv(const CodingUnit &pu, Mv (&mvAffi)[2][3]);
  bool                              xBmAffineRegression(CodingUnit &pu, Distortion &minCost);
  Mv                               *m_bdmvrSubPuMvBuf[2];
  void xSetBoundarySamples(const CodingUnit &pu, const Mv (&mv)[2], int dx, int dy, int xx, int yy, int widthInSubPu);
  Distortion getBoundaryDistortion(CodingUnit &pu, int xx, int yy, const int widthInSubPu, int theWidth, int theHeight,
                                   bool &lowSpatAct);
  void       xBDMVRMvRefinement(int xx, int yy, int widthInSubPu, Mv mvOffset, Mv (&curBestMv)[2][2], CodingUnit &pu,
                                const Mv (&initialMv)[2], bool useMR, int useHadmard);

public:
  bool isBDOFMvRefined() { return m_bdofMvRefined; }
  Mv  *getBdofSubPuMvOffset() { return m_bdofSubPuMvOffset; }
  void setBdmvrSubPuMvBuf(Mv *mvBuf0, Mv *mvBuf1)
  {
    m_bdmvrSubPuMvBuf[0] = mvBuf0;
    m_bdmvrSubPuMvBuf[1] = mvBuf1;
  }
  bool processBDMVR(CodingUnit &cu);
  bool processBDMVR4Affine(CodingUnit &pu);

  void mcFramePad(Picture *pcCurPic, Slice &slice);

public:
  PelUnitBuf m_predictionBeforeLIC;
  bool       m_storeBeforeLIC;

private:
  static const int m_LICShift     = 5;
  static const int m_LICRegShift  = 7;
  static const int m_LICShiftDiff = 12;

  // buffer size for left/above current templates and left/above reference templates
  Pel *m_pcLICRefLeftTemplate[2][MAX_NUM_COMP];
  Pel *m_pcLICRefAboveTemplate[2][MAX_NUM_COMP];
  Pel *m_pcLICRecLeftTemplate[MAX_NUM_COMP];
  Pel *m_pcLICRecAboveTemplate[MAX_NUM_COMP];

  Pel *m_curLICRefLeftTemplate[2][MAX_NUM_COMP];
  Pel *m_curLICRefAboveTemplate[2][MAX_NUM_COMP];
  Pel *m_curLICRecLeftTemplate[MAX_NUM_COMP];
  Pel *m_curLICRecAboveTemplate[MAX_NUM_COMP];

  bool m_templateAvailable[MAX_NUM_COMP][2];
  bool m_fillLicTpl[MAX_NUM_COMP];

  PelStorage m_acPredBeforeLICBuffer[2];
  Pel       *m_Gx;
  Pel       *m_Gy;

  void xLicPredBlk(const CodingUnit &cu, const CompID compID, const Picture *refPic, const Mv *mv, const bool bi,
                   PelBuf dstBuf, const RefPicList refPicList);

  void xGetLICParamGeneral(const CodingUnit &cu, const CompID compID, const bool *templateAvailable,
                           Pel *refLeftTemplate, Pel *refAboveTemplate, Pel *recLeftTemplate, Pel *recAboveTemplate,
                           int &scale, int &offset);
  void xGetSublkTemplate(const CodingUnit &cu, const CompID compID, const Picture &refPic, const Mv &mv,
                         const int sublkWidth, const int sublkHeight, const int posW, const int posH,
                         bool *templateAvailable, Pel *refLeftTemplate, Pel *refAboveTemplate, Pel *recLeftTemplate,
                         Pel *recAboveTemplate);
  void xLocalIlluComp(const CodingUnit &cu, const CompID compID, const Picture *refPic, const Mv *mv, const bool biPred,
                      PelBuf &dstBuf, const RefPicList refPicList);

  template<bool TrueA_FalseL> void xGetPredBlkTpl(const CodingUnit &cu, const CompID compID, const CPelBuf &refBuf,
                                                  const Mv &mv, const int posW, const int posH, const int tplSize,
                                                  Pel *predBlkTpl);

  void xLicRemHighFreq(const CodingUnit &cu, const CompID compID, const int licIdx);
  void xLicCompAdj(CodingUnit &pu, const bool lumaOnly, const bool chromaOnly);
  void xLicBiDerive(CodingUnit &cu, const bool lumaOnly, const bool chromaOnly);
  void xLicBiApply(const CodingUnit &cu, const bool lumaOnly, const bool chromaOnly);
  void xLicCopyPredBeforeLic(CodingUnit &cu, PelUnitBuf &pcYuvPred, bool luma, bool chroma, PelUnitBuf *yuvPredTmp,
                             bool *isOOB);

public:
  void resetFillLicTpl() { m_fillLicTpl[COMP_Y] = m_fillLicTpl[COMP_Cb] = m_fillLicTpl[COMP_Cr] = false; }

  template<boundaryDirection T> void mcFramePadOneSide(Picture *pcCurPic, Slice &slice, CodingUnit *blkDataTmp,
                                                       PelStorage *pPadYUVContainerDyn, const UnitArea blkUnitAreaBuff,
                                                       PelStorage *pCurBuffYUV);

#define mcFramePadTop    mcFramePadOneSide<boundaryDirection::BD_TOP>
#define mcFramePadBottom mcFramePadOneSide<boundaryDirection::BD_BOTTOM>
#define mcFramePadLeft   mcFramePadOneSide<boundaryDirection::BD_LEFT>
#define mcFramePadRight  mcFramePadOneSide<boundaryDirection::BD_RIGHT>
};

//! \}

#endif   // __INTERPREDICTION__
