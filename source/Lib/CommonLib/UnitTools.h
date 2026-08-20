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

/** \file     UnitTool.h
 *  \brief    defines operations for basic units
 */

#ifndef __UNITTOOLS__
#define __UNITTOOLS__

#include "Unit.h"
#include "UnitPartitioner.h"
#include "ContextModelling.h"
#include "InterPrediction.h"

// CS tools
namespace CS
{
UnitArea getArea(const CodingStructure &cs, const UnitArea &area, const ChannelType chType);
bool     isDualITree(const CodingStructure &cs);
void     saveTemporalEipModel(CodingStructure &cs);
}   // namespace CS

// CU tools
namespace CU
{
static inline bool isIntra(const CodingUnit &cu) { return cu.predMode == MODE_INTRA; }
static inline bool isInter(const CodingUnit &cu) { return cu.predMode == MODE_INTER; }
static inline bool isIBC(const CodingUnit &cu) { return cu.predMode == MODE_IBC; }
static inline bool isPLT(const CodingUnit &cu) { return cu.predMode == MODE_PLT; }

bool     isSameCtu(const CodingUnit &cu, const CodingUnit &cu2);
bool     isSameSlice(const CodingUnit &cu, const CodingUnit &cu2);
bool     isSameTile(const CodingUnit &cu, const CodingUnit &cu2);
bool     isSameSliceAndTile(const CodingUnit &cu, const CodingUnit &cu2);
bool     isSameSubPic(const CodingUnit &cu, const CodingUnit &cu2);
bool     isLastSubCUOfCtu(const CodingUnit &cu);
bool     isOnCtuBottom(const CodingUnit &cu);
uint32_t getCtuAddr(const CodingUnit &cu);
int      predictQP(const CodingUnit &cu, const int prevQP);

void saveMotionForHmvp(const CodingUnit &cu);

PartSplit getSplitAtDepth(const CodingUnit &cu, const unsigned depth);

bool isSgpmCoded(const CodingUnit &cu);

bool    isBcwIdxCoded(const CodingUnit &cu);
uint8_t getValidBcwIdx(const CodingUnit &cu);
bool    bdpcmAllowed(const CodingUnit &cu, const CompID compID);
bool    pltAllowed(const CodingUnit &cu);
bool    canUseGPM(const CodingUnit &cu);

bool hasAffineNb(const CodingUnit &cu);
bool isAffineAllowed(const CodingUnit &cu);
bool affineCtxInc(const CodingUnit &cu);

TUTraverser  traverseTUs(CodingUnit &cu);
cTUTraverser traverseTUs(const CodingUnit &cu);

bool hasSubCUNonZeroMVd(const CodingUnit &cu);
bool hasSubCUNonZeroAffineMVd(const CodingUnit &cu);

uint8_t getSbtInfo(uint8_t idx, uint8_t pos);
uint8_t getSbtIdx(const uint8_t sbtInfo);
uint8_t getSbtPos(const uint8_t sbtInfo);
uint8_t getSbtMode(const uint8_t sbtIdx, const uint8_t sbtPos);
uint8_t getSbtIdxFromSbtMode(const uint8_t sbtMode);
uint8_t getSbtPosFromSbtMode(const uint8_t sbtMode);
uint8_t targetSbtAllowed(uint8_t idx, uint8_t sbtAllowed);
uint8_t numSbtModeRdo(uint8_t sbtAllowed);
bool    isSbtMode(const uint8_t sbtInfo);
bool    isSameSbtSize(const uint8_t sbtInfo1, const uint8_t sbtInfo2);
bool    getRprScaling(const SPS *sps, const PPS *curPPS, Picture *refPic, ScalingRatio &scalingRatio);
void    checkConformanceILRP(Slice *slice);
bool    allowTimdSad(const CodingUnit &cu);
bool    canTimdMerge(const CodingUnit &cu);
bool    hasTimdMergeCandidate(const CodingUnit &cu);
int     canTimdMergeImplicitDst7(const TransformUnit &tu);
std::array<const CodingUnit *, TIMD_MERGE_MAX_NONADJACENT> timdMergeNonAdjacentNeighbours(const CodingUnit &cu);
const TimdData &getActiveTimdData(const CodingUnit &cu);
const std::array<TransType, 2> &getActiveTimdTrTypes(const CodingUnit &cu);

bool isObmcAllowed(const CodingUnit &cu);

struct TimdRefTypePositionAndSize
{
  TemplateType                  eTemplateType { NO_NEIGHBOR };
  std::pair<int, int>           iRefPosition { -1, -1 };
  std::pair<uint32_t, uint32_t> uiRefSize {};
};
TimdRefTypePositionAndSize deriveTimdRefTypePositionAndSize(const CodingUnit &cu, const int iTemplateWidth,
                                                            const int iTemplateHeight);

bool isLICFlagPresent(const CodingUnit &cu);
bool hasDecoderDerivedCCP(const CodingUnit &cu);
bool hasNonLocalCCP(const CodingUnit &cu);
int  getCCPModelCandidateList(const CodingUnit &cu, CrossCompModels candList[], int selIdx = -1);
void saveModelsInHCCP(const CodingUnit &cu);
bool hasCcFilterFlag(const CodingUnit &cu);
bool eipAllowed(const CodingUnit &cu, const CompID &compId);
bool eipMergeAllowed(const CodingUnit &cu, const CompID &compId);
void eipCurAllowed(const CodingUnit &cu, const CompID &compId, static_vector<EipInfo, NUM_DERIVED_EIP> &eipInfoList,
                   bool isMultiModel);
void saveModelsInHEIP(const CodingUnit &cu);
bool hasOppositeLICFlag(const CodingUnit &cu);
}   // namespace CU

// PU tools
namespace PU
{
int      getLMSymbolList(const CodingUnit &cu, int *modeList);
int      getIntraMPMs(const CodingUnit &pu, uint8_t *mpm, uint8_t *non_mpm);
void     getGeoIntraMPMs(const CodingUnit &cu, uint8_t *mpm, uint8_t splitDir, uint8_t shape);
void     getSgpmIntraMPMs(const CodingUnit &cu, uint8_t *mpm, uint8_t splitDir, uint8_t shape, int dimdMode);
bool     isMIP(const CodingUnit &cu, const ChannelType chType = ChannelType::LUMA);
bool     isDIMD(const CodingUnit &cu, const ChannelType chType = ChannelType::LUMA);
bool     isDIMDChroma(const CodingUnit &cu, const ChannelType chType = ChannelType::CHROMA);
bool     isTIMD(const CodingUnit &cu, const ChannelType chType = ChannelType::LUMA);
bool     isOBIC(const CodingUnit &cu, const ChannelType chType = ChannelType::LUMA);
bool     isEIP(const CodingUnit &cu, const ChannelType chType = ChannelType::LUMA);
void     interpretLumaIntraMode(CodingUnit &cu);
void     interpretChromaIntraMode(CodingUnit &cu);
void     setLumaIntraModeFlags(CodingUnit &cu, const CUCtxIntra &cuCtxt);
void     setChromaIntraModeFlag(CodingUnit &cu);
bool     directionalPlanarAvailable(const CodingUnit &cu, const ChannelType &chType = ChannelType::LUMA);
void     trafoIntraDirPlanar(const CodingUnit &cu, const ChannelType &chType, uint32_t &intraDir);
bool     isDMChromaMIP(const CodingUnit &cu);
bool     isSgpm(const CodingUnit &cu, const ChannelType &chType = ChannelType::LUMA);
bool     isDMChromaSgpm(const CodingUnit &cu);
uint32_t getIntraDirLuma(const CodingUnit &cu);
void     getIntraChromaCandModes(const CodingUnit &cu, unsigned modeList[NUM_CHROMA_MODE]);
uint32_t getFinalIntraMode(const CodingUnit &cu, const ChannelType &chType);
int      getLFNSTMatrixDim(const int width, const int height);
bool     getUseLFNST8(const int width, const int height);
bool     getUseLFNST16(const int width, const int height);
int      getNSPTMatrixDim(const int width, const int height);
int      getNSPTBucket(const TransformUnit &tu);
std::pair<int8_t, int8_t> getFinalIntraModesTrafo(const TransformUnit &tu, const CompID compID);
uint32_t                  getWANonSepTrafoIntraMode(int wideAngPredMode);
uint32_t                  getCoLocatedIntraLumaMode(const CodingUnit &cu);
int                       getWideAngle(const TransformUnit &tu, const uint32_t dirMode, const CompID compID);
uint32_t                  getBDMVRMvdThreshold(const CodingUnit &cu);
bool                      hasCclmDeltaFlag(const CodingUnit &cu, const int mode = -1);
void                      getCccmRefLineNum(const CodingUnit &cu, int &th, int &tv);
NeighAreaType             crossCompNeighType(int intraMode);
int                       cccmMultiFilterIndex(const ConvModelType modelType);
bool cccmAvailable(const CodingUnit &cu, const int intraMode, const ConvModelType modelType = CONV_MODEL_UNDEFINED);
bool hasBvgCccmFlag(const CodingUnit &cu);
bool bvgCccmModeAvail(const CodingUnit &cu);
bool bvgCccmMultiModeAvail(const CodingUnit &cu, int intraMode);
void getBvgCccmCands(CodingUnit &cu);
bool isBvgCccmCand(const CodingUnit &cu, Mv &chromaBv, int &rrIbcType, int candIdx = 0);
bool checkIsChromaBvCandidateValid(const CodingUnit &cu, const Mv chromaBv, int &iWidth, int &iHeight,
                                   bool isRefTemplate = false, bool isRefAbove = false);
const CodingUnit &getCoLocatedLumaCU(const CodingUnit &cu);
uint32_t          getBDMVRMvdThreshold(const CodingUnit &cu);
uint32_t          getBiGpmThreshold(const CodingUnit &cu);
void getInterMergeCandidates(const CodingUnit &cu, MergeCtx &mrgCtx, int mmvdList, const int &mrgCandIdx = -1,
                             int simThr = 1);
void getIBCMergeCandidates(const CodingUnit &cu, MergeCtx &mrgCtx, const int &mrgCandIdx = -1);
void getInterMMVDMergeCandidates(const CodingUnit &cu, MergeCtx &mrgCtx);
int  getDistScaleFactor(const int &currPOC, const int &currRefPOC, const int &colPOC, const int &colRefPOC);
bool isDiffMER(const Position &pos1, const Position &pos2, const unsigned plevel);
bool getColocatedMVP(const CodingUnit &cu, const RefPicList &eRefPicList, const Position &pos, Mv &rcMv,
                     const int &refIdx, bool sbFlag);
void fillMvpCand(CodingUnit &cu, const RefPicList &eRefPicList, const int &refIdx, AMVPInfo &amvpInfo);
void fillIBCMvpCand(CodingUnit &cu, AMVPInfo &amvpInfo);
void fillAffineMvpCand(CodingUnit &cu, const RefPicList &eRefPicList, const int &refIdx, AffineAMVPInfo &affiAMVPInfo);
bool addMVPCandUnscaled(const CodingUnit &cu, const RefPicList &eRefPicList, const int &refIdx, const Position &pos,
                        const MvpDir &eDir, AMVPInfo &amvpInfo);
void xInheritedAffineMv(const CodingUnit &cu, const CodingUnit *puNeighbour, RefPicList eRefPicList, Mv rcMv[3]);
bool addMergeHmvpCand(const CodingStructure &cs, MergeCtx &mrgCtx, const int &mrgCandIdx,
                      const uint32_t maxNumMergeCandMin1, int &cnt, const bool isAvailableA1, const MotionInfo &miLeft,
                      const bool isAvailableB1, const MotionInfo &miAbove, const bool ibcFlag, const bool isGt4x4);
void addAMVPHMVPCand(const CodingUnit &cu, const RefPicList eRefPicList, const int currRefPOC, AMVPInfo &info);
void xGetAffineMvFromLUT(AffineMotionInfo *affHistInfo, int rParameters[4]);
bool checkLastAffineMergeCandRedundancy(const CodingUnit &cu, AffineMergeCtx &affMrgCtx);

bool addOneAffineMergeHMVPCand(const CodingUnit &cu, AffineMergeCtx &affMrgCtx,
                               static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS> *lutAff, int affHMVPIdx,
                               const MotionInfo &mvInfo, Position neiPosition, int iGBiIdx, bool LICFlag = false);
bool addSpatialAffineMergeHMVPCand(const CodingUnit &cu, AffineMergeCtx &affMrgCtx,
                                   static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS> *lutAff, int affHMVPIdx,
                                   const CodingUnit *neiPUs[], Position neiPositions[], int iNeiNum,
                                   const int mrgCandIdx = -1);
bool addSpatialAffineAMVPHMVPCand(CodingUnit &cu, const RefPicList &eRefPicList, const int &refIdx,
                                  AffineAMVPInfo                                          &affiAMVPInfo,
                                  static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS> *lutAff, int iHMVPlistIdx,
                                  int neiIdx[], int iNeiNum, int aiNeibeInherited[], bool bFoundOne);
bool addMergeHMVPCandFromAffModel(const CodingUnit &cu, MergeCtx &mrgCtx, const int &mrgCandIdx, int &cnt);
bool addOneMergeHMVPCandFromAffModel(const CodingUnit &cu, MergeCtx &mrgCtx, int &cnt,
                                     static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS> *lutAff, int listIdx,
                                     const MotionInfo &mvInfo, Position neiPosition, int iGBiIdx);
void deriveAffineCandFromMvField(Position posLT, const int width, const int height, std::vector<RMVFInfo> mvInfoVec,
                                 Mv mvAffi[3]);

bool addOneInheritedHMVPAffineMergeCand(const CodingUnit &cu, AffineMergeCtx &affMrgCtx,
                                        static_vector<AffineInheritInfo, MAX_NUM_AFF_INHERIT_HMVP_CANDS> &lutAffInherit,
                                        int                                                               affHMVPIdx);
void xGetAffineMvFromLUT(short affineParameters[4], int rParameters[4]);
void deriveAffineParametersFromMVs(const CodingUnit &cu, const Mv acMvTemp[3], int *affinePara, AffineModel affModel);
void deriveMVsFromAffineParameters(const CodingUnit &cu, Mv rcMv[3], int *affinePara, const Mv &cBaseMv,
                                   const Position &cBasePos);
void storeAffParas(int *affinePara);
void deriveCenterMVFromAffineParameters(const CodingUnit &cu, Mv &rcMv, int *affinePara, const Mv &cBaseMv,
                                        const Position &cBasePos);
bool checkLastAffineAMVPCandRedundancy(const CodingUnit &cu, AffineAMVPInfo &affiAMVPInfo);
bool addAffineMVPCandUnscaled(const CodingUnit &cu, const RefPicList &refPicList, const int &refIdx,
                              const Position &pos, const MvpDir &dir, AffineAMVPInfo &affiAmvpInfo,
                              int aiNeibeInherited[5]);
bool isBipredRestriction(const CodingUnit &cu);
void spanMotionInfo(CodingUnit &cu, const AffineMergeCtx *mrgCtx = nullptr);
void spanBdmvrMotionInfo(const CodingUnit &cu, MotionBuf &mb, Mv *bdmvrSubPuMv0, Mv *bdmvrSubPuMv1,
                         Mv *bdofSubPuMvOffset);
void applyImv(CodingUnit &cu, InterPrediction *interPred = nullptr);
bool getAffineControlPointCand(const CodingUnit &cu, MotionInfo mi[4], bool isAvailable[4], int verIdx[4],
                               int8_t bcwIdx, int modelIdx, int verNum, AffineMergeCtx &affMrgCtx);
bool xCPMVSimCheck(const CodingUnit &cu, AffineMergeCtx &affMrgCtx, Mv curCpmv[2][3], uint8_t curdir,
                   int8_t curRefIdx[2], AffineModel curType, int bcwIdx, bool LICFlag);
int  getMvDiffThresholdByWidthAndHeight(const CodingUnit &cu, bool width);
bool addNonAdjAffineConstructedCPMV(const CodingUnit &cu, MotionInfo miNew[4], bool isAvaNew[4], Position pos[4],
                                    int8_t bcwId, bool LICFlag, AffineMergeCtx &affMrgCtx, int mrgCandIdx);
bool addNonAdjCstAffineMVPCandUnscaled(const CodingUnit &cu, const RefPicList &refPicList, const int &refIdx,
                                       AffineAMVPInfo &affiAmvpInfo);
bool addNonAdjCstAffineMVPConstructedCPMV(const CodingUnit &cu, MotionInfo miNew[3], bool isAvaNew[3], Position pos[3],
                                          const RefPicList &refPicList, const int &refIdx,
                                          AffineAMVPInfo &affiAmvpInfo);
void getNonAdjCstMergeCand(const CodingUnit &cu, AffineMergeCtx &affMrgCtx, const int mrgCandIdx = -1,
                           bool isInitialized = false);
int  getNonAdjAffParaDivFun(int num1, int num2);
void getAffineMergeCand(const CodingUnit &cu, AffineMergeCtx &affMrgCtx, const int mrgCandIdx = -1,
                        bool isAffMmvd = false);
void getAffMmvdMvf(const CodingUnit &pu, const AffineMergeCtx &affineMergeCtx,
                   std::array<MvField[2], AFFINE_MAX_NUM_CP> &mvfMmvd, const uint16_t affMmvdBaseIdx,
                   const uint16_t offsetStep, const uint16_t offsetDir);
uint8_t getMergeIdxFromAffMmvdBaseIdx(const AffineMergeCtx &affMrgCtx, uint16_t affMmvdBaseIdx);
void    setAllAffineMvField(CodingUnit &cu, std::array<MvField[2], AFFINE_MAX_NUM_CP> &mvField, RefPicList eRefList);
void    setAllAffineMv(CodingUnit &cu, Mv affLT, Mv affRT, Mv affLB, RefPicList eRefList, bool clipCPMVs = false);
bool    getInterMergeSubPuMvpCand(const CodingUnit &cu, AffineMergeCtx &mrgCtx, const int count);
bool    isSimpleSymmetricBiPred(const CodingUnit &cu);
void    restrictBiPredMergeCandsOne(CodingUnit &cu);

bool isLMCMode(unsigned mode);
bool isMultiModeLM(unsigned mode);
bool isLMCModeEnabled(const CodingUnit &cu, unsigned mode);
void setGpmDirMode(CodingUnit &cu);
void getGeoMergeCandidates(const CodingUnit &cu, MergeCtx &GeoMrgCtx);
void spanGeoMMVDMotionInfo(CodingUnit &cu, const GeoMergeCtx &geoMrgCtx);

bool     addNeighborMv(const Mv &currMv, static_vector<Mv, IBC_NUM_CANDIDATES> &neighborMvs);
void     getIbcMVPsEncOnly(CodingUnit &cu, static_vector<Mv, IBC_NUM_CANDIDATES> &mvPred);
bool     getDerivedBV(CodingUnit &cu, const Mv &currentMv, Mv &derivedMv);
bool     isBiRefScaled(const Slice &slice, const int refIdx0, const int refIdx1);
bool     isBiPredFromDifferentDirEqDistPoc(const CodingUnit &cu);
bool     isBiPredFromDifferentDirEqDistPoc(const Slice &slice, int refIdx0, int refIdx1);
bool     isBiPredFromDifferentDirGenDistPoc(const CodingUnit &pu);
bool     checkBDMVR4Affine(const CodingUnit &pu);
bool     checkBDMVRCpmvRefinementPuUsage(const CodingUnit &pu);
bool     checkBDMVRCondition(const CodingUnit &cu);
bool     isBIOApplied(const CodingUnit &cu, const Slice &slice, const PPS &pps, const bool subPuMC);
void     getNeighborAffineInfo(const CodingUnit &cu, int &numNeighborAvai, int &numNeighborAffine);
bool     isBMMergeFlagCoded(const CodingUnit &cu);
bool     addBMMergeHMVPCand(const CodingStructure &cs, MergeCtx &mrgCtx, const int &mrgCandIdx,
                            const uint32_t maxNumMergeCandMin1, int &cnt, const bool isAvailableA1,
                            const MotionInfo &miLeft, const bool isAvailableB1, const MotionInfo &miAbove,
                            const bool ibcFlag, const uint32_t mvdSimilarityThresh = 1);
void     getInterBMCandidates(const CodingUnit &pu, MergeCtx &mrgCtx, const int &mrgCandIdx = -1);
int      getSameMotionNeighbor(const CodingUnit &cu, int dir, int startBlock, int maxBlocks, MotionInfo &curMi,
                               MotionInfo &neighMi, const CodingUnit *&neighCu, bool &isValid);
bool     checkRprLicCondition(const CodingUnit &cu);
void     spanLicFlags(CodingUnit &cu, const bool LICFlag);
void     xCalcRMVFParameters(std::vector<RMVFInfo> &mvpInfoVec, int64_t dMatrix[2][4], int64_t sumbb[2][3][3],
                             int64_t sumeb[2][3], uint8_t shift, uint16_t addedSize);
void     xReturnMvpVec(std::vector<RMVFInfo> mvp[2][4], const CodingUnit &pu, const Position &pos);
void     getRMVFAffineGuideCand(const CodingUnit &pu, const CodingUnit &abovePU, AffineMergeCtx &affMrgCtx,
                                std::vector<RMVFInfo> mvp[2][4], int mrgCandIdx = -1);
Position convertNonAdjAffineBlkPos(const Position &pos, int curCtuX, int curCtuY);
void     collectNeiMotionInfo(std::vector<RMVFInfo> mvpInfoVec[2][4], const CodingUnit &pu);
int      getObicNeighbours(const CodingUnit &cu, std::vector<const CodingUnit *> &cuNeighbours);
bool     isObicAvail(const CodingUnit &cu);
}   // namespace PU

// TU tools
namespace TU
{
bool getCbf(const TransformUnit &tu, const CompID &compID);
bool getCbfAtDepth(const TransformUnit &tu, const CompID &compID, const unsigned &depth);
void setCbfAtDepth(TransformUnit &tu, const CompID &compID, const unsigned &depth, const bool &cbf);
bool isTSAllowed(const TransformUnit &tu, const CompID &compID);
bool isMTSAllowed(const TransformUnit &tu, const CompID &compID);
bool lfnstAllowed(const TransformUnit &tu, const CompID &compID);
int  getNstIdx(const TransformUnit &tu, const CompID &compID);
bool isNSPTAllowed(int width, int height);
bool nsptApplyCond(const TransformUnit &tu, const CompID compID, bool allowNSPT);

bool           needsSqrt2Scale(const TransformUnit &tu, const CompID &compID);
bool           needsBlockSizeTrafoScale(const TransformUnit &tu, const CompID &compID);
TransformUnit *getPrevTU(const TransformUnit &tu, const CompID compID);
int            getICTMode(const TransformUnit &tu, int jointCbCr = -1);
TransList getTransCandIntra(const TransformUnit &tu, const CompID &compID, const bool lossless, const bool chromaTS);
TransList getTransCandInter(const TransformUnit &tu, const CompID &compID, const bool lossless, const bool chromaTS,
                            const MtsType bestHistMTS, const uint8_t bestHistSBT);

Size getMaxLog2SignPredArea(const TransformUnit &tu, const CompID compID);
Size getSignPredArea(const TransformUnit &tu, const CompID compID);
bool getDelayedSignCoding(const TransformUnit &tu, const CompID compID);
bool getUseSignPred(const TransformUnit &tu, const CompID compID);
}   // namespace TU

bool     shouldUse2x2EdgeOperator(const Area &area);
bool     storeContexts(const Slice *slice, const int ctuXPosInCtus, const int ctuYPosInCtus);
uint32_t getCtuAddr(const Position &pos, const PreCalcValues &pcv);
template<typename T, size_t N> uint32_t updateCandList(T mode, double uiCost, static_vector<T, N> &candModeList,
                                                       static_vector<double, N> &candCostList, size_t uiFastCandNum = N,
                                                       int *iserttPos = nullptr)
{
  CHECK(std::min(uiFastCandNum, candModeList.size()) != std::min(uiFastCandNum, candCostList.size()),
        "Sizes do not match!");
  CHECK(uiFastCandNum > candModeList.capacity(), "The vector is to small to hold all the candidates!");

  size_t i;
  size_t shift    = 0;
  size_t currSize = std::min(uiFastCandNum, candCostList.size());

  while (shift < uiFastCandNum && shift < currSize && uiCost < candCostList[currSize - 1 - shift])
  {
    shift++;
  }

  if (candModeList.size() >= uiFastCandNum && shift != 0)
  {
    for (i = 1; i < shift; i++)
    {
      candModeList[currSize - i] = candModeList[currSize - 1 - i];
      candCostList[currSize - i] = candCostList[currSize - 1 - i];
    }
    candModeList[currSize - shift] = mode;
    candCostList[currSize - shift] = uiCost;
    if (iserttPos != nullptr)
    {
      *iserttPos = int(currSize - shift);
    }
    return 1;
  }
  else if (currSize < uiFastCandNum)
  {
    candModeList.insert(candModeList.end() - shift, mode);
    candCostList.insert(candCostList.end() - shift, uiCost);
    if (iserttPos != nullptr)
    {
      *iserttPos = int(candModeList.size() - shift - 1);
    }
    return 1;
  }
  if (iserttPos != nullptr)
  {
    *iserttPos = -1;
  }
  return 0;
}

int deriveAffineSubBlkSize(const int sz, const int minSbSz, const int deltaMvX, const int deltaMvY, const int shift);

#endif
