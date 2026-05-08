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

/** \file     InterSearch.h
    \brief    inter search class (header)
 */

#ifndef __INTERSEARCH__
#define __INTERSEARCH__

// Include files
#include "CABACWriter.h"

#include "CommonLib/MotionInfo.h"
#include "CommonLib/InterPrediction.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/Unit.h"
#include "CommonLib/UnitPartitioner.h"
#include "CommonLib/RdCost.h"
#include "CommonLib/BilateralFilter.h"
#include "CommonLib/AffineGradientSearch.h"
#include "CommonLib/IbcHashMap.h"
#include "CommonLib/Hash.h"
#include <unordered_map>
#include <vector>
#include "EncReshape.h"
//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

static constexpr uint32_t MAX_NUM_REF_LIST_ADAPT_SR = NUM_RPL01;
static constexpr uint32_t MAX_IDX_ADAPT_SR          = MAX_NUM_REF;
static constexpr uint32_t NUM_MV_PREDICTORS         = 3;
struct BlkRecord
{
  std::unordered_map<Mv, Distortion> bvRecord;
};
class EncModeCtrl;

struct AffineMVInfo
{
  RefSetArray<Mv[3]> affMVs;
  int                x, y, w, h;
};

struct BlkUniMvInfo
{
  RefSetArray<Mv> uniMvs;
  int             x, y, w, h;
};

typedef struct
{
  Mv         acMvAffine4Para[2][3];
  Mv         acMvAffine6Para[2][3];
  int16_t    affine4ParaRefIdx[2];
  int16_t    affine6ParaRefIdx[2];
  Distortion hevcCost[3];
  Distortion affineCost[3];
  bool       affine4ParaAvail;
  bool       affine6ParaAvail;

} EncAffineMotion;

struct EncCfg;

template<int N> struct SrchCostBv
{
  static const int maxSize  = N;
  static const int capacity = (N) + 1;

  uint32_t cnt;
  bool     enableFracIBC;         // Just to make sure cfg-off results and macro-off results will match
  bool enableMultiCandSrch;   // True: search best fracBv among best N integer BVs; False: search best fracBv among best
                              // integer BVs and best fracBv of previous round
  Distortion costList[capacity];
  Mv         mvList[capacity];
  uint8_t    imvList[capacity];
  uint8_t    mvpIdxList[capacity];
#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  int bvTypeList[capacity];
#endif
#if JVET_AA0070_RRIBC && JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  bool bvFlipList[capacity];
#endif

  SrchCostBv() : cnt(0), enableFracIBC(false), enableMultiCandSrch(false) { mvList[maxSize].setZero(); }

  void init(bool resetHistoryMv = false, bool _enableFracIBC = false)
  {
    cnt                 = 0;
    enableFracIBC       = _enableFracIBC;
    enableMultiCandSrch = _enableFracIBC;
    if (resetHistoryMv)
    {
      mvList[maxSize].setZero();
    }
  }

  void cutoff(double ratio)
  {
    if (N >= 2)
    {
      if (cnt >= 2)
      {
        double th = ratio * (double)costList[0];

        for (int i = 1; i < cnt; ++i)
        {
          if ((double)costList[i] > th)
          {
            cnt = i;
            return;
          }
        }
      }
    }
  }

  int find(int mvx, int mvy
#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
           ,
           int bvType
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
           ,
           bool bvFlip
#endif
  )
  {
    for (int i = 0; i < (int)cnt; ++i)
    {
      if (mvList[i].getHor() == mvx && mvList[i].getVer() == mvy
#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
          && bvTypeList[i] == bvType
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
          && bvFlipList[i] == bvFlip
#endif
      )
      {
        return i;
      }
    }
    return NOT_VALID;
  }

  void replaceAt(uint32_t idxSrc, uint32_t idxDst)
  {
    if (idxSrc != idxDst)
    {
      costList[idxDst]   = costList[idxSrc];
      mvList[idxDst]     = mvList[idxSrc];
      imvList[idxDst]    = imvList[idxSrc];
      mvpIdxList[idxDst] = mvpIdxList[idxSrc];
#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
      bvTypeList[idxDst] = bvTypeList[idxSrc];
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
      bvFlipList[idxDst] = bvFlipList[idxSrc];
#endif
    }
  }

  int insert(Distortion cost, int mvx, int mvy
#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
             ,
             int bvType = 0
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
             ,
             bool bvFlip = true
#endif
  )
  {
#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
    // Ignore 1-D BV's
    if (bvType != 0)
    {
      return NOT_VALID;
    }
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
    bvFlip = bvType == 0 ? false : bvFlip;
#endif

    // Find insertion index
    int insertIdx = NOT_VALID;
    for (int i = 0; i < (int)cnt; ++i)
    {
      if (cost <= costList[i])
      {
        if (cost == costList[i])
        {
          if (mvList[i].getHor() == mvx && mvList[i].getVer() == mvy
#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
              && bvTypeList[i] == bvType
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
              && bvFlipList[i] == bvFlip
#endif
          )
          {
            return NOT_VALID;
          }
        }
        else
        {
          insertIdx = i;
          break;
        }
      }
    }

    // Do insertion
    auto setAt = [&](uint32_t idx)
    {
      costList[idx] = cost;
      mvList[idx].set(mvx, mvy);
#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
      bvTypeList[idx] = bvType;
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
      bvFlipList[idx] = bvFlip;
#endif
    };

    auto replaceNext = [&](uint32_t idx)
    {
      costList[idx + 1] = costList[idx];
      mvList[idx + 1].set(mvList[idx].getHor(), mvList[idx].getVer());
#if JVET_AA0070_RRIBC || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
      bvTypeList[idx + 1] = bvTypeList[idx];
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
      bvFlipList[idx + 1] = bvFlipList[idx];
#endif
    };

    if (insertIdx != NOT_VALID)
    {
      for (int i = (cnt < N ? cnt - 1 : N - 2); i >= insertIdx; --i)
      {
        replaceNext(i);
      }
      setAt(insertIdx);

      if (cnt < N)
      {
        ++cnt;
      }
      return insertIdx;
    }
    else if (cnt < N)
    {
      insertIdx = cnt;
      setAt(insertIdx);

      ++cnt;
      return insertIdx;
    }
    else
    {
      return NOT_VALID;
    }
  }
};

typedef SrchCostBv<4> SrchCostIntBv;

class EncFastLICCtrl
{
  double m_amvpRdBeforeLIC[NUM_IMV_MODES];

public:
  EncFastLICCtrl() { init(); }

  void init()
  {
    m_amvpRdBeforeLIC[IMV_OFF]  = std::numeric_limits<double>::max();
    m_amvpRdBeforeLIC[IMV_FPEL] = std::numeric_limits<double>::max();
    m_amvpRdBeforeLIC[IMV_4PEL] = std::numeric_limits<double>::max();
    m_amvpRdBeforeLIC[IMV_HPEL] = std::numeric_limits<double>::max();
  }

  bool skipRDCheckForLIC(bool isLIC, int imv, double curBestRd, uint32_t cuNumPel)
  {
    bool skipLIC = false;
    if (isLIC)
    {
      skipLIC |= skipLicBasedOnBestAmvpRDBeforeLIC(imv, curBestRd);
      skipLIC |= (cuNumPel < LIC_MIN_CU_PIXELS);
    }
    return skipLIC;
  }

public:
  void setBestAmvpRDBeforeLIC(const CodingUnit &cu, double curCuRdCost)
  {
    m_amvpRdBeforeLIC[cu.imv] = !cu.mergeFlag && cu.predMode != MODE_IBC && !cu.licFlag
      ? std::min(curCuRdCost, m_amvpRdBeforeLIC[cu.imv])
      : m_amvpRdBeforeLIC[cu.imv];
  }

private:
  bool skipLicBasedOnBestAmvpRDBeforeLIC(uint8_t curCuimvIdx, double curBestRdCost)
  {
    return m_amvpRdBeforeLIC[curCuimvIdx] != std::numeric_limits<double>::max() &&
      m_amvpRdBeforeLIC[curCuimvIdx] > curBestRdCost * LIC_AMVP_SKIP_TH;
  }
};

/// encoder search class
class InterSearch : public InterPrediction, AffineGradientSearch
{
private:
  struct MvRefSetArray
  {
    RefSetArray<Mv> entry;
    bool            valid = 0;
  };

private:
  PelStorage m_tmpPredStorage[NUM_RPL01];
  PelStorage m_tmpStorageCtu;
  PelStorage m_tmpStorageCUflipH;
  PelStorage m_tmpStorageCUflipV;

public:
  SrchCostIntBv m_bestSrchCostIntBv;

private:
  PelStorage m_tmpAffiStorage;
  Pel       *m_tmpAffiError;
  Pel       *m_tmpAffiDeri[2];

  CodingStructure **m_pSaveCS;

  ClpRng                                                            m_lumaClpRng;
  uint32_t                                                          m_estWeightIdxBits[BCW_NUM];
  BcwMotionParam                                                    m_uniMotions;
  bool                                                              m_affineModeSelected;
  bool                                                              m_doAffineLic;
  std::unordered_map<Position, std::unordered_map<Size, BlkRecord>> m_ctuRecord;
  AffineMVInfo                                                     *m_affMVList;
  int                                                               m_affMVListIdx;
  int                                                               m_affMVListSize;
  int                                                               m_affMVListMaxSize;
  BlkUniMvInfo                                                     *m_uniMvList;
  int                                                               m_uniMvListIdx;
  int                                                               m_uniMvListSize;
  int                                                               m_uniMvListMaxSize;
  BlkUniMvInfo                                                     *m_uniMvListLIC;
  int                                                               m_uniMvListIdxLIC;
  int                                                               m_uniMvListSizeLIC;
  Distortion                                                        m_hevcCost;
  EncAffineMotion                                                   m_affineMotion;
  static_vector<Mv, IBC_NUM_CANDIDATES>                             m_defaultCachedBvs;

  size_t         m_numReusedUniMvBufs = 0;
  MvRefSetArray *m_reusedUniMvBuf[2]  = { nullptr, nullptr };
  MvRefSetArray *m_reusedUniMVs[MAX_CU_SIZE_IN_PARTS][MAX_CU_SIZE_IN_PARTS][MAX_NUM_SIZES][MAX_NUM_SIZES];
  MvRefSetArray *m_reusedUniMVsLIC[MAX_CU_SIZE_IN_PARTS][MAX_CU_SIZE_IN_PARTS][MAX_NUM_SIZES][MAX_NUM_SIZES];

  struct PreTrList
  {
    PreTrList() {}
    ALIGN_DATA(MEMORY_ALIGN_DEF_SIZE, Pel res[4][MAX_TB_SIZEY * MAX_TB_SIZEY]) = {};
    TransList   trTypes                                                        = {};
    TransList   trTypesAdd                                                     = {};
    TransBuffer trCoeffs                                                       = {};
  };
  PreTrList m_trCandList;

protected:
  // interface to option
  const EncCfg    *m_encCfg;
  BilateralFilter *m_bilateralFilter;
  EncModeCtrl     *m_modeCtrl;

  // interface to classes
  TrQuant    *m_pcTrQuant;
  EncReshape *m_pcReshape;

  // ME parameters
  int            m_searchRange;
  int            m_bipredSearchRange;   // Search range for bi-prediction
  MESearchMethod m_motionEstimationSearchMethod;
  int            m_adaptSR[MAX_NUM_REF_LIST_ADAPT_SR][MAX_IDX_ADAPT_SR];

  // RD computation
  CABACWriter *m_CABACEstimator;
  CtxPool     *m_ctxPool;
  DistParam    m_cDistParam;

  RefPicList m_currRefPicList;
  int        m_currRefPicIndex;
  bool       m_skipFracME;

  RefSetArray<int>                         m_numHashMVStoreds;
  RefSetArray<Mv[Hash::NUM_LOG_BLK_SIZES]> m_hashMVStoreds;

  // Misc.
  Pel *m_pTempPel;

  // AMVP cost computation
  uint32_t m_auiMVPIdxCost[AMVP_MAX_NUM_CANDS + 1][AMVP_MAX_NUM_CANDS + 1];   // th array bounds

  RefSetArray<Mv> m_integerMv2Nx2N;

  bool m_isInitialized;

  static_vector<Mv, 2 * IBC_NUM_CANDIDATES> m_acBVs;
  bool                                      m_useCompositeRef;
  Distortion m_estMinDistSbt[NUMBER_SBT_MODE + 1];   // estimated minimum SSE value of the PU if using a SBT mode
  uint8_t    m_sbtRdoOrder[NUMBER_SBT_MODE];   // order of SBT mode in RDO
  bool       m_skipSbtAll;   // to skip all SBT modes for the current PU
  uint8_t    m_histBestSbt;   // historical best SBT mode for PU of certain SSE values
  MtsType    m_histBestMtsIdx;   // historical best MTS idx  for PU of certain SSE values
  bool       m_clipMvInSubPic;

public:
  EncFastLICCtrl m_fastLicCtrl;

public:
  InterSearch();
  virtual ~InterSearch();

  void init(const EncCfg *encCfg, BilateralFilter *bilateralFilter, TrQuant *pcTrQuant, EncModeCtrl *pcEncModeCtrl,
            int searchRange, int bipredSearchRange, MESearchMethod motionEstimationSearchMethod, bool useCompositeRef,
            const uint32_t maxCUWidth, const uint32_t maxCUHeight, const uint32_t maxTotalCUDepth, RdCost *pcRdCost,
            CABACWriter *CABACEstimator, CtxPool *ctxPool, EncReshape *m_pcReshape, const uint32_t curPicWidthY,
            InterpolationFilter *pcInterpolationFilter);

  void destroy();

  void    calcMinDistSbt(CodingStructure &cs, const CodingUnit &cu, const uint8_t sbtAllowed);
  uint8_t skipSbtByRDCost(int width, int height, int mtDepth, uint8_t sbtIdx, uint8_t sbtPos, double bestCost,
                          Distortion distSbtOff, double costSbtOff, bool rootCbfSbtOff);
  bool    getSkipSbtAll() { return m_skipSbtAll; }
  void    setSkipSbtAll(bool skipAll) { m_skipSbtAll = skipAll; }
  uint8_t getSbtRdoOrder(uint8_t idx)
  {
    assert(m_sbtRdoOrder[idx] < NUMBER_SBT_MODE);
    assert((uint32_t)(m_estMinDistSbt[m_sbtRdoOrder[idx]] >> 2) < (MAX_UINT >> 1));
    return m_sbtRdoOrder[idx];
  }
  Distortion getEstDistSbt(uint8_t sbtMode) { return m_estMinDistSbt[sbtMode]; }
  void       initTuAnalyzer()
  {
    m_estMinDistSbt[NUMBER_SBT_MODE] = std::numeric_limits<uint64_t>::max();
    m_skipSbtAll                     = false;
  }
  void setHistBestTrs(uint8_t sbtInfo, MtsType mtsIdx)
  {
    m_histBestSbt    = sbtInfo;
    m_histBestMtsIdx = mtsIdx;
  }
  void initSbtRdoOrder(uint8_t sbtMode)
  {
    m_sbtRdoOrder[0]   = sbtMode;
    m_estMinDistSbt[0] = m_estMinDistSbt[sbtMode];
  }

  void setTempBuffers(CodingStructure **pSaveCS);
  void resetCtuRecord() { m_ctuRecord.clear(); }
  void setAffineModeSelected(bool flag) { m_affineModeSelected = flag; }
  void setDoAffineLic(bool flag) { m_doAffineLic = flag; }
  void resetAffineMVList()
  {
    m_affMVListIdx  = 0;
    m_affMVListSize = 0;
  }
  void savePrevAffMVInfo(int idx, AffineMVInfo &tmpMVInfo, bool &isSaved);
  void addAffMVInfo(AffineMVInfo &tmpMVInfo);
  void swapUniMvBuffer()   // simply swap the MvInfo buffer in order to not over-change the functions ME,
                           // insertUniMvCands, savePrevUniMvInfo and addUniMvInfo.
  {
    std::swap(m_uniMvList, m_uniMvListLIC);
    std::swap(m_uniMvListIdx, m_uniMvListIdxLIC);
    std::swap(m_uniMvListSize, m_uniMvListSizeLIC);
  }
  void resetUniMvList()
  {
    m_uniMvListIdx     = 0;
    m_uniMvListSize    = 0;
    m_uniMvListIdxLIC  = 0;
    m_uniMvListSizeLIC = 0;
  }
  void xCreateReusedUniMvs();
  void xDestroyReusedUniMvs();
  void resetReusedUniMvs();
  void resetGradBuffers()
  {
    memset(m_gradX0, 2 + 8 + 32 + 128, BDOF_TEMP_BUFFER_SIZE * sizeof(Pel));
    memset(m_gradX1, 2 + 8 + 32 + 128, BDOF_TEMP_BUFFER_SIZE * sizeof(Pel));
    memset(m_gradY0, 2 + 8 + 32 + 128, BDOF_TEMP_BUFFER_SIZE * sizeof(Pel));
    memset(m_gradY1, 2 + 8 + 32 + 128, BDOF_TEMP_BUFFER_SIZE * sizeof(Pel));
  }

  void insertUniMvCands(const Area &blkArea, RefSetArray<Mv> &cMvTemp);
  void savePrevUniMvInfo(CompArea blkArea, BlkUniMvInfo &tmpUniMvInfo, bool &isUniMvInfoSaved);
  void addUniMvInfo(BlkUniMvInfo &tmpUniMVInfo);
  void resetSavedAffineMotion();

  void storeAffineMotion(Mv acAffineMv[2][3], int16_t affineRefIdx[2], AffineModel affineType, int bcwIdx);

  void setClipMvInSubPic(bool flag) { m_clipMvInSubPic = flag; }

protected:
  /// sub-function for motion vector refinement used in fractional-pel accuracy
  Distortion xPatternRefinement(const CPelBuf *pcPatternKey, Mv baseRefMv, int iFrac, Mv &rcMvFrac,
                                bool bAllowUseOfHadamard);

  void xEstBvdBitCosts(EstBvdBitsStruct *p, unsigned useIBCFrac = 0);

  typedef struct
  {
    int left;
    int right;
    int top;
    int bottom;
  } SearchRange;

  typedef struct
  {
    SearchRange    searchRange;
    const CPelBuf *pcPatternKey;
    const Pel     *piRefY;
    ptrdiff_t      iRefStride;
    int            iBestX;
    int            iBestY;
    uint32_t       uiBestRound;
    uint32_t       uiBestDistance;
    Distortion     uiBestSad;
    uint8_t        ucPointNr;
    int            subShiftMode;
    unsigned       imvShift;
    bool           useAltHpelIf;
    bool           inCtuSearch;
    bool           zeroMV;
  } IntTZSearchStruct;

  // sub-functions for ME
  inline void xTZSearchHelp(IntTZSearchStruct &rcStruct, const int iSearchX, const int iSearchY,
                            const uint8_t ucPointNr, const uint32_t uiDistance);
  inline void xTZ2PointSearch(IntTZSearchStruct &rcStruct);
  inline void xTZ8PointSquareSearch(IntTZSearchStruct &rcStruct, const int iStartX, const int iStartY, const int iDist);
  inline void xTZ8PointDiamondSearch(IntTZSearchStruct &rcStruct, const int iStartX, const int iStartY, const int iDist,
                                     const bool bCheckCornersAtDist1);

public:
  /// encoder estimation - inter prediction (non-skip)

  void predInterSearch(CodingUnit &cu, Partitioner &partitioner);
  bool predIBCSearch(CodingUnit &cu, Partitioner &partitioner, const int localSearchRangeX, const int localSearchRangeY,
                     IbcHashMap &ibcHashMap);
  void resetIbcSearch() { m_defaultCachedBvs.clear(); }
  bool predInterHashSearch(CodingUnit &cu, Partitioner &partitioner, bool &isPerfectMatch);
  /// set ME search range
  void setAdaptiveSearchRange(int dir, int refIdx, int searchRange)
  {
    CHECK(dir >= MAX_NUM_REF_LIST_ADAPT_SR || refIdx >= int(MAX_IDX_ADAPT_SR), "Invalid index");
    m_adaptSR[dir][refIdx] = searchRange;
  }

  void insertUniMvCandsReuseMv(const Area &blkArea, const PreCalcValues &pcv);

protected:
  void xIntraPatternSearch(CodingUnit &cu, IntTZSearchStruct &cStruct, Mv &rcMv, Distortion &ruiCost, Mv *cMvSrchRngLT,
                           Mv *cMvSrchRngRB, Mv *pcMvPred);
  template<int N> Distortion xPredIBCFracPelSearch(CodingUnit &cu, SrchCostBv<N> &intBvList, AMVPInfo *amvpInfoQPel,
                                                   AMVPInfo *amvpInfoHPel, AMVPInfo *amvpInfoFPel,
                                                   AMVPInfo *amvpInfo4Pel);
  void xSetIntraSearchRange(CodingUnit &cu, int iRoiWidth, int iRoiHeight, const int localSearchRangeX,
                            const int localSearchRangeY, Mv &rcMvSrchRngLT, Mv &rcMvSrchRngRB);

  void xIBCEstimation(CodingUnit &cu, PelUnitBuf &origBuf, Mv *pcMvPred, Mv &rcMv, Distortion &ruiCost,
                      const int localSearchRangeX, const int localSearchRangeY);
  void xIBCSearchMVCandUpdate(Distortion uiSad, int x, int y, Distortion *uiSadBestCand,
                              static_vector<Mv, CHROMA_REFINEMENT_CANDIDATES> &cMVCand);
  int  xIBCSearchMVChromaRefine(CodingUnit &cu, int iRoiWidth, int iRoiHeight, int cuPelX, int cuPelY,
                                Distortion *uiSadBestCand, static_vector<Mv, CHROMA_REFINEMENT_CANDIDATES> &cMVCand);
  void addToSortList(std::list<BlockHash> &listBlockHash, std::list<int> &listCost, int cost,
                     const BlockHash &blockHash);

  bool xHashInterEstimation(CodingUnit &cu, RefPicList &bestRefPicList, int &bestRefIndex, Mv &bestMv, Mv &bestMvd,
                            int &bestMVPIndex, bool &isPerfectMatch);
  bool xRectHashInterEstimation(CodingUnit &cu, RefPicList &bestRefPicList, int &bestRefIndex, Mv &bestMv, Mv &bestMvd,
                                int &bestMVPIndex, bool &isPerfectMatch);
  void xSelectRectangleMatchesInter(const MapIterator &itBegin, int count, std::list<BlockHash> &listBlockHash,
                                    const BlockHash &currBlockHash, int width, int height, int idxNonSimple,
                                    unsigned int *&hashValues, int baseNum, int picWidth, int picHeight,
                                    bool isHorizontal, uint16_t *curHashPic);
  void xSelectMatchesInter(const MapIterator &itBegin, int count, std::list<BlockHash> &vecBlockHash,
                           const BlockHash &currBlockHash);

  // -------------------------------------------------------------------------------------------------------------------
  // Inter search (AMP)
  // -------------------------------------------------------------------------------------------------------------------

  void xEstimateMvPredAMVP(CodingUnit &cu, PelUnitBuf &origBuf, RefPicList eRefPicList, int refIdx, Mv &rcMvPred,
                           AMVPInfo &amvpInfo, bool bFilled = false, Distortion *puiDistBiP = nullptr);

  void xCheckBestMVP(RefPicList eRefPicList, Mv cMv, Mv &rcMvPred, int &riMVPIdx, AMVPInfo &amvpInfo, uint32_t &ruiBits,
                     Distortion &ruiCost, const uint8_t imv);

  Distortion xGetTemplateCost(const CodingUnit &cu, PelUnitBuf &origBuf, PelUnitBuf &predBuf, Mv cMvCand, int mvpIdx,
                              int mvpNum, RefPicList eRefPicList, int refIdx);
  uint32_t   xCalcAffineMVBits(CodingUnit &cu, Mv mvCand[3], Mv mvPred[3]);

  void     xCopyAMVPInfo(AMVPInfo *pSrc, AMVPInfo *pDst);
  uint32_t xGetMvpIdxBits(int idx, int num);
  void     xGetBlkBits(bool isPSlice, uint32_t blkBit[3]);

  // -------------------------------------------------------------------------------------------------------------------
  // motion estimation
  // -------------------------------------------------------------------------------------------------------------------

  void xMotionEstimation(CodingUnit &cu, PelUnitBuf &origBuf, RefPicList eRefPicList, Mv &rcMvPred, int refIdxPred,
                         Mv &rcMv, int &riMVPIdx, uint32_t &ruiBits, Distortion &ruiCost, const AMVPInfo &amvpInfo,
                         bool bBi = false);
  void xTZSearch(const CodingUnit &cu, RefPicList eRefPicList, int refIdxPred, IntTZSearchStruct &cStruct, Mv &rcMv,
                 Distortion &ruiSAD, const Mv *const pIntegerMv2Nx2NPred, const bool bExtendedSettings,
                 const bool bFastSettings = false);

  void xTZSearchSelective(const CodingUnit &cu, RefPicList eRefPicList, int refIdxPred, IntTZSearchStruct &cStruct,
                          Mv &rcMv, Distortion &ruiSAD, const Mv *const pIntegerMv2Nx2NPred);

  void xSetSearchRange(const CodingUnit &cu, const Mv &cMvPred, const int iSrchRng, SearchRange &sr,
                       IntTZSearchStruct &cStruct);

  void xPatternSearchFast(const CodingUnit &cu, RefPicList eRefPicList, int refIdxPred, IntTZSearchStruct &cStruct,
                          Mv &rcMv, Distortion &ruiSAD, const Mv *const pIntegerMv2Nx2NPred);

  void xPatternSearch(IntTZSearchStruct &cStruct, Mv &rcMv, Distortion &ruiSAD);

  void xPatternSearchIntRefine(CodingUnit &cu, IntTZSearchStruct &cStruct, Mv &rcMv, Mv &rcMvPred, int &riMVPIdx,
                               uint32_t &ruiBits, Distortion &ruiCost, const AMVPInfo &amvpInfo, double fWeight);

  void xPatternSearchFracDIF(const CodingUnit &cu, RefPicList eRefPicList, int refIdx, IntTZSearchStruct &cStruct,
                             const Mv &rcMvInt, Mv &rcMvHalf, Mv &rcMvQter, Distortion &ruiCost);

  void xPredAffineInterSearch(CodingUnit &cu, PelUnitBuf &origBuf, int puIdx, uint32_t &lastMode,
                              Distortion &affineCost, RefSetArray<Mv> &hevcMv, RefSetArray<Mv[3]> &mvAffine4Para,
                              int refIdx4Para[NUM_RPL01], uint8_t bcwIdx = BCW_DEFAULT, bool enforceBcwPred = false,
                              uint32_t bcwIdxBits = 0);

  void xAffineMotionEstimation(CodingUnit &cu, PelUnitBuf &origBuf, RefPicList eRefPicList, Mv acMvPred[3],
                               int refIdxPred, Mv acMv[3], uint32_t &ruiBits, Distortion &ruiCost, int &mvpIdx,
                               const AffineAMVPInfo &aamvpi, bool bBi = false);

  void xEstimateAffineAMVP(CodingUnit &cu, AffineAMVPInfo &affineAMVPInfo, PelUnitBuf &origBuf, RefPicList eRefPicList,
                           int refIdx, Mv acMvPred[3], Distortion *puiDistBiP);

  Distortion xGetAffineTemplateCost(CodingUnit &cu, PelUnitBuf &origBuf, PelUnitBuf &predBuf, Mv acMvCand[3],
                                    int mvpIdx, int mvpNum, RefPicList eRefPicList, int refIdx);

  void xCopyAffineAMVPInfo(AffineAMVPInfo &src, AffineAMVPInfo &dst);
  void xCheckBestAffineMVP(CodingUnit &cu, AffineAMVPInfo &affineAMVPInfo, RefPicList eRefPicList, Mv acMv[3],
                           Mv acMvPred[3], int &riMVPIdx, uint32_t &ruiBits, Distortion &ruiCost);

  Distortion xGetSymmetricCost(CodingUnit &cu, PelUnitBuf &origBuf, RefPicList eCurRefPicList,
                               const MvField &cCurMvField, MvField &cTarMvField, int bcwIdx);

  Distortion xSymmeticRefineMvSearch(CodingUnit &cu, PelUnitBuf &origBuf, Mv &rcMvCurPred, Mv &rcMvTarPred,
                                     RefPicList eRefPicList, MvField &rCurMvField, MvField &rTarMvField,
                                     Distortion uiMinCost, int searchPattern, int nSearchStepShift,
                                     uint32_t uiMaxSearchRounds, int bcwIdx);

  void xSymmetricMotionEstimation(CodingUnit &cu, PelUnitBuf &origBuf, Mv &rcMvCurPred, Mv &rcMvTarPred,
                                  RefPicList eRefPicList, MvField &rCurMvField, MvField &rTarMvField,
                                  Distortion &ruiCost, int bcwIdx);

  bool   xReadBufferedAffineUniMv(CodingUnit &cu, RefPicList eRefPicList, int32_t refIdx, Mv acMvPred[3], Mv acMv[3],
                                  uint32_t &ruiBits, Distortion &ruiCost, int &mvpIdx, const AffineAMVPInfo &aamvpi);
  double xGetMEDistortionWeight(uint8_t bcwIdx, RefPicList eRefPicList);
  bool   xReadBufferedUniMv(CodingUnit &cu, RefPicList eRefPicList, int32_t refIdx, Mv &pcMvPred, Mv &rcMv,
                            uint32_t &ruiBits, Distortion &ruiCost);
  void   xClipMv(Mv &rcMv, const Position &pos, const Size &size, const SPS &sps, const PPS &pps);

public:
  void     resetBufferedUniMotions() { m_uniMotions.reset(); }
  uint32_t getWeightIdxBits(uint8_t bcwIdx) { return m_estWeightIdxBits[bcwIdx]; }
  void     initWeightIdxBits();
  void     symmvdCheckBestMvp(CodingUnit &cu, PelUnitBuf &origBuf, Mv curMv, RefPicList curRefList,
                              RefSetArray<AMVPInfo> &amvpInfo, int32_t bcwIdx, Mv cMvPredSym[NUM_RPL01],
                              int32_t mvpIdxSym[NUM_RPL01], Distortion &bestCost, bool skip = false);

protected:
  void     xExtDIFUpSamplingH(CPelBuf *pcPattern, bool useAltHpelIf);
  void     xExtDIFUpSamplingQ(CPelBuf *pcPatternKey, Mv halfPelRef);
  uint32_t xDetermineBestMvp(CodingUnit &cu, Mv acMvTemp[3], int &mvpIdx, const AffineAMVPInfo &aamvpi);
  void     xSetWpScalingDistParam(int refIdx, RefPicList eRefPicListCur, Slice *slice);

  void xIBCHashSearch(CodingUnit &cu, Mv *mvPred, int numMvPred, Mv &mv, int &idxMvPred, IbcHashMap &ibcHashMap);
  // -------------------------------------------------------------------------------------------------------------------
  // compute symbol bits
  // -------------------------------------------------------------------------------------------------------------------

public:
  void     encodeResAndCalcRdInterCU(CodingStructure &cs, Partitioner &partitioner, const bool &skipResidual,
                                     const bool luma = true, const bool chroma = true);
  uint64_t calcPuMeBits(CodingUnit &cu);

protected:
  void     xEncodeInterResidualQT(CodingStructure &cs, Partitioner &partitioner, const CompID &compID);
  void     xEstimateInterResidualQT(CodingStructure &cs, Partitioner &partitioner, Distortion *puiZeroDist = nullptr,
                                    const bool luma = true, const bool chroma = true, PelUnitBuf *orgResi = nullptr);
  uint64_t xGetSymbolFracBitsInter(CodingStructure &cs, Partitioner &partitioner);

private:
  TrEst::Cost xPreCalcTrans(TransformUnit &tu, const CompID compID, PreTrList &tl);
  void        xPreCalcTransUpd(TransformUnit &tu, const CompID compID, PreTrList &tl, TrEst::Cost &costDCT);
  CbfMaskList xPreCalcTransJCCR(TransformUnit &tu, PreTrList &tl);
  TrEst::Cost xSelectTrCand(TransformUnit &tu, const CompID compID, PreTrList &tl);
  void        xSelectTrCandUpd(TransformUnit &tu, const CompID compID, PreTrList &tl, TrEst::Cost &costDCT);

  class PreCost : public TrEst::PreCostBase
  {
  public:
    PreCost(TransformUnit &tu, const CompID cId, CABACWriter *cabacEst, RdCost *rdCost);
    void        init(const PreTrList &tl);
    TrEst::Cost operator()(const MtsType tr);

  private:
    double estTransBits(const MtsType tr);

  private:
    CABACWriter     *bitEst = nullptr;
    const PreTrList *trList = nullptr;
  };

};   // END CLASS DEFINITION EncSearch

//! \}

#endif   // __ENCSEARCH__
