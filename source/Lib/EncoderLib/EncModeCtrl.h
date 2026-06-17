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

/** \file     EncModeCtrl.h
    \brief    Encoder controller for trying out specific modes
*/

#ifndef __ENCMODECTRL__
#define __ENCMODECTRL__

// Include files
#include "CommonLib/CommonDef.h"
#include "CommonLib/CodingStructure.h"
#include "InterSearch.h"

#include <typeinfo>
#include <vector>

//////////////////////////////////////////////////////////////////////////
// Encoder modes to try out
//////////////////////////////////////////////////////////////////////////

enum EncTestModeType
{
  ETM_HASH_INTER,
  ETM_MERGE_SKIP,
  ETM_INTER_ME,
  ETM_INTRA,
  ETM_PALETTE,
  ETM_SPLIT_QT,
  ETM_SPLIT_BT_H,
  ETM_SPLIT_BT_V,
  ETM_SPLIT_TT_H,
  ETM_SPLIT_TT_V,
  ETM_POST_DONT_SPLIT, // dummy mode to collect the data from the unsplit coding
#if REUSE_CU_RESULTS
  ETM_RECO_CACHED,
#endif
  ETM_IBC,    // ibc mode
  ETM_IBC_MERGE, // ibc merge mode
  ETM_INVALID
};

enum EncTestModeOpts
{
  ETO_STANDARD    = 0,                   // empty      (standard option)
  ETO_FORCE_MERGE = 1 << 0,                // bit   0    (indicates forced merge)
  ETO_IMV_SHIFT   = 1,                // bits  1-3  (imv parameter starts at bit 1)
  ETO_IMV         = 7 << ETO_IMV_SHIFT,    // bits  1-3  (imv parameter uses 3 bits)
  ETO_LIC         = 1 << 4,               // bit   4    (local illumination compensation)
  ETO_DUMMY       = 1 << 5,                // bit   5    (dummy)
  ETO_INVALID     = 0xffffffff            // bits 0-31  (invalid option)
};

static void getAreaIdx(const Area &area, const PreCalcValues &pcv, unsigned &idxX, unsigned &idxY, unsigned &idxW,
                       unsigned &idxH)
{
  idxX = (area.x & pcv.maxCUWidthMask) >> MIN_CU_LOG2;
  idxY = (area.y & pcv.maxCUHeightMask) >> MIN_CU_LOG2;
  idxW = gp_sizeIdxInfo->idxFrom(area.width);
  idxH = gp_sizeIdxInfo->idxFrom(area.height);
}

struct EncTestMode
{
  enum struct AmvrSearchMode
  {
    NONE,
    FULL_PEL,
    FOUR_PEL,
    FOUR_PEL_FAST,
    HALF_PEL
  };

  EncTestMode() : type(ETM_INVALID), opts(ETO_INVALID), qp(-1), deltaQPForLambda(0) {}
  EncTestMode(EncTestModeType _type) : type(_type), opts(ETO_STANDARD), qp(-1), deltaQPForLambda(0) {}
  EncTestMode(EncTestModeType _type, int _qp, double _deltaQPForLambda)
    : type(_type)
    , opts(ETO_STANDARD)
    , qp(_qp)
    , deltaQPForLambda(_deltaQPForLambda)
  {}
  EncTestMode(EncTestModeType _type, EncTestModeOpts _opts, int _qp, double _deltaQPForLambda)
    : type(_type)
    , opts(_opts)
    , qp(_qp)
    , deltaQPForLambda(_deltaQPForLambda)
  {}
  EncTestMode(EncTestModeType _type, EncTestModeOpts _opts, int _qp, double _deltaQPForLambda, double maxCost)
    : type(_type)
    , opts(_opts)
    , qp(_qp)
    , deltaQPForLambda(_deltaQPForLambda)
    , maxCostAllowed(maxCost)
  {}

  EncTestModeType type;
  EncTestModeOpts opts;
  int             qp;
  double          deltaQPForLambda;
  double          maxCostAllowed;

  AmvrSearchMode getAmvrSearchMode() const { return AmvrSearchMode((opts & ETO_IMV) >> ETO_IMV_SHIFT); }
};

inline bool isModeSplit(const EncTestMode &encTestmode)
{
  switch (encTestmode.type)
  {
  case ETM_SPLIT_QT:
  case ETM_SPLIT_BT_H:
  case ETM_SPLIT_BT_V:
  case ETM_SPLIT_TT_H:
  case ETM_SPLIT_TT_V:
    return true;
  default:
    return false;
  }
}

inline bool isModeNoSplit(const EncTestMode &encTestmode)
{
  return !isModeSplit(encTestmode) && encTestmode.type != ETM_POST_DONT_SPLIT;
}

inline bool isModeInter(const EncTestMode &encTestmode) // perhaps remove
{
  return (encTestmode.type == ETM_INTER_ME || encTestmode.type == ETM_MERGE_SKIP || encTestmode.type == ETM_HASH_INTER);
}

inline PartSplit getPartSplit(const EncTestMode &encTestmode)
{
  switch (encTestmode.type)
  {
  case ETM_SPLIT_QT:
    return CU_QUAD_SPLIT;
  case ETM_SPLIT_BT_H:
    return CU_HORZ_SPLIT;
  case ETM_SPLIT_BT_V:
    return CU_VERT_SPLIT;
  case ETM_SPLIT_TT_H:
    return CU_TRIH_SPLIT;
  case ETM_SPLIT_TT_V:
    return CU_TRIV_SPLIT;
  default:
    return CU_DONT_SPLIT;
  }
}

inline EncTestMode getCSEncMode(const CodingStructure &cs)
{
  return EncTestMode(EncTestModeType(cs.etmType), EncTestModeOpts(cs.etmOpts), false);
}

//////////////////////////////////////////////////////////////////////////
// EncModeCtrl controls if specific modes should be tested
//////////////////////////////////////////////////////////////////////////

struct ComprCUCtx
{
  ComprCUCtx() : testModes() {}

  ComprCUCtx(const CodingStructure &cs, const uint32_t _minDepth, const uint32_t _maxDepth)
    : minDepth(_minDepth)
    , maxDepth(_maxDepth)
    , testModes()
    , lastTestMode()
  {}

  unsigned                 minDepth;
  unsigned                 maxDepth;
  std::vector<EncTestMode> testModes;
  EncTestMode              lastTestMode;
  EncTestMode              currTestMode;
  CodingStructure         *bestCS { nullptr };
  CodingUnit              *bestCU { nullptr };
  TransformUnit           *bestTU { nullptr };

  Distortion  interHad { std::numeric_limits<Distortion>::max() };
  bool        earlySkip { false };
  bool        isHashPerfectMatch { false };
  EncTestMode bestMode;
  double      bestNonSplitCost { MAX_DOUBLE };
  double      bestCostVertSplit { MAX_DOUBLE };
  double      bestCostHorzSplit { MAX_DOUBLE };
  double      bestCostTriVertSplit { MAX_DOUBLE };
  double      bestCostTriHorzSplit { MAX_DOUBLE };
  double      bestCostImv { MAX_DOUBLE };
  double      bestCostNoImv { MAX_DOUBLE };
  double      bestCostGPM { MAX_DOUBLE };
  double      bestCostLIC { MAX_DOUBLE };
  int         maxQtSubDepth { 0 };
  bool        isReusingCu { false };
  bool        qtBeforeBt { false };
  bool        doTriHorzSplit { false };
  bool        doTriVertSplit { false };
  bool        didQuadSplit { false };
  bool        didHorzSplit { false };
  bool        didVertSplit { false };
  bool        doHorChromaSplit { false };
  bool        doVerChromaSplit { false };
  bool        doQtChromaSplit { false };
  int         doMoreSplits { 3 };
  bool        isBestNoSplitSkip { false };
  bool        skipSecondMTSPass { false };
  bool        intraWasTested { false };
  bool        isIntra { false };
  bool        nonSkipWasTested { false };
  int         bestIntraMode { 0 };
  int         baseQp { -1 };
  int         bestIntraNzCnt { 0 };
};

//////////////////////////////////////////////////////////////////////////
// some utility interfaces that expose some functionality that can be used without concerning about which particular
// controller is used
//////////////////////////////////////////////////////////////////////////
struct SaveLoadStructSbt
{
  uint8_t  numPuInfoStored;
  uint32_t puSse[SBT_NUM_SL];
  uint8_t  puSbt[SBT_NUM_SL];
  MtsType  puTrs[SBT_NUM_SL];
};

class SaveLoadEncInfoSbt
{
protected:
  void init(const Slice &slice);
  void create();
  void destroy();

private:
  SaveLoadStructSbt  **m_saveLoadSbt[MAX_CU_SIZE_IN_PARTS][MAX_CU_SIZE_IN_PARTS];
  PreCalcValues const *m_pcv;

public:
  virtual ~SaveLoadEncInfoSbt() {}
  void resetSaveloadSbt(int maxSbtSize);

  struct BestSbt
  {
    uint8_t sbt;
    MtsType trs;
  };

  BestSbt findBestSbt(const UnitArea &area, const uint32_t curPuSse) const;
  bool    saveBestSbt(const UnitArea &area, const uint32_t curPuSse, const uint8_t curPuSbt, const MtsType curPuTrs);
};

static constexpr int MAX_STORED_CU_INFO_REFS = 4;

struct CodedCUInfo
{
  bool isInter;
  bool isIntra;
  bool isSkip;
  bool isMMVDSkip;
  bool isIBC;
  bool isSkipGPM;
  int  geoDirCandList[GEO_MAX_TRY_WEIGHTED_SATD];
  int  numGeoDirCand;
  bool validMv[NUM_RPL01][MAX_STORED_CU_INFO_REFS];
  Mv   saveMv[NUM_RPL01][MAX_STORED_CU_INFO_REFS];

  uint8_t bcwIdx;
  double  bestCost;
  bool    skipLIC;
};

class CacheBlkInfoCtrl
{
private:
  PreCalcValues const *m_pcv;
  // x in CTU, y in CTU, width, height
  CodedCUInfo       ***m_codedCUInfo[MAX_CU_SIZE_IN_PARTS][MAX_CU_SIZE_IN_PARTS];

protected:
  void create();
  void destroy();
  void init(const Slice &slice);
  virtual ~CacheBlkInfoCtrl() {}

public:
  CodedCUInfo &getCodedCUInfo(const UnitArea &area);

public:
  const CodedCUInfo &getBlkInfo(const UnitArea &area) const;

  bool getMv(const UnitArea &area, const RefPicList refPicList, const int refIdx, Mv &rMv) const;
  void setMv(const UnitArea &area, const RefPicList refPicList, const int refIdx, const Mv &rMv);
};

#if REUSE_CU_RESULTS
struct BestEncodingInfo
{
  CodingUnit cu;
#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS
  TransformUnit tus[REUSE_CU_RESULTS_MAX_NUM_TUS];
  size_t        numTus;
#else
  TransformUnit tu;
#endif
  EncTestMode testMode;
  Distortion  dist;
  double      encDbOptCost;

  int poc;
};

class BestEncInfoCache
{
private:
  const PreCalcValues *m_pcv { nullptr };
  BestEncodingInfo  ***m_bestEncInfo[MAX_CU_SIZE_IN_PARTS][MAX_CU_SIZE_IN_PARTS] {};
  TCoeff              *m_pCoeff { nullptr };
  uint8_t             *m_pSignsPredArea { nullptr };
  Pel                 *m_pltIdx { nullptr };
  bool                *m_runType { nullptr };
  XuPool               m_dummyPool {};
  CodingStructure      m_dummyCS { m_dummyPool };
  int                  m_sliceQp { -128 };

protected:
  BestEncInfoCache() {}
  virtual ~BestEncInfoCache() {}

  void create(const ChromaFormat chFmt);
  void destroy();
  void init(const Slice &slice);

  bool setFromCs(const CodingStructure &cs, const Partitioner &partitioner);

public:
  bool isValid(const CodingStructure &cs, const Partitioner &partitioner, int qp) const;
  bool setCsFrom(CodingStructure &cs, EncTestMode &testMode, const Partitioner &partitioner) const;
};

#endif
//////////////////////////////////////////////////////////////////////////
// EncModeCtrlMTnoRQT - allows and controls modes introduced by QTBT (inkl. multi-type-tree)
//                    - only 2Nx2N, no RQT, additional binary/triary CU splits
//////////////////////////////////////////////////////////////////////////

struct EncCfg;
class RateCtrl;
class RdCost;

class EncModeCtrl : public CacheBlkInfoCtrl
#if REUSE_CU_RESULTS
  ,
                    public BestEncInfoCache
#endif
  ,
                    public SaveLoadEncInfoSbt
{
public:
  ComprCUCtx *comprCUCtx { nullptr };
  bool        doPlt { false };
  bool        useHashME { false };  // th put this somewhere else ?
  bool        fastDeltaQp { false };

protected:
  static_vector<ComprCUCtx, (4 * MAX_CU_DEPTH)> m_ComprCUCtxList;
  const EncCfg                                 *m_encCfg { nullptr };
  const RateCtrl                               *m_pcRateCtrl { nullptr };
  RdCost                                       *m_pcRdCost { nullptr };
  const Slice                                  *m_slice { nullptr };
  std::map<int, double *>                      *m_adaptQPmap { nullptr };
#if SHARP_LUMA_DELTA_QP
  int m_lumaLevelToDeltaQPLUT[LUMA_LEVEL_TO_DQP_LUT_MAXSIZE];
  int m_lumaQPOffset { 0 };
#endif
  unsigned m_skipSplitThreshold { SKIP_DEPTH };

public:
  virtual ~EncModeCtrl() {}

  void create(const EncCfg *encCfg);
  void destroy();
  void init(const EncCfg *encCfg, RateCtrl *pRateCtrl, RdCost *pRdCost, std::map<int, double *> *adaptQPmap);
  void initCTUEncoding(const Slice &slice);
  void initCULevel(Partitioner &partitioner, const CodingStructure &cs);
  void finishCULevel(Partitioner &partitioner);
  const CodingUnit *getBestCU(int depthOffset = 0) const;
  bool              tryMode(const EncTestMode &encTestmode, const CodingStructure &cs, Partitioner &partitioner);
  bool              trySplit(const EncTestMode &encTestmode, const CodingStructure &cs, Partitioner &partitioner);
  bool              finishNonSplitModes(const CodingStructure &cs, Partitioner &partitioner);
  bool              useModeResult(const EncTestMode &encTestmode, CodingStructure *&tempCS, Partitioner &partitioner,
                                  bool useEncDbOpt);

  void getMinMaxQP(int &iMinQP, int &iMaxQP, double &deltaQPForLambda, const CodingStructure &cs, const Partitioner &pm,
                   const int baseQP, const PartSplit splitMode);

  void setEarlySkipDetected();

  //  const ComprCUCtx& getComprCUCtx   () { CHECK( nullptr == comprCUCtx, "Accessing empty list!"); return *comprCUCtx;
  //  }

#if SHARP_LUMA_DELTA_QP
  void initLumaDeltaQpLUT();
  int  calculateLumaDQP(const CPelBuf &rcOrg);
#endif
  int calculateLumaDQPsmooth(const CPelBuf &rcOrg, int baseQP, double threshold, double scale, double offset,
                             int limit);

private:
  bool xSkipTreeCandidate(const PartSplit split, const ComprCUCtx &cuECtx, const SliceType sliceType) const;
  int  xComputeDQP(const CodingStructure &cs, const Partitioner &pm);
  bool xTryModeInternal(const EncTestMode &encTestmode, const CodingStructure &cs, Partitioner &partitioner);
  bool xTrySplitInternal(const EncTestMode &encTestmode, const CodingStructure &cs, Partitioner &partitioner);
};

//! \}

#endif   // __ENCMODECTRL__
