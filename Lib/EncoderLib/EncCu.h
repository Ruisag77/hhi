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

/** \file     EncCu.h
    \brief    Coding Unit (CU) encoder class (header)
*/

#ifndef __ENCCU__
#define __ENCCU__

// Include files
#include "CommonLib/CommonDef.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/InterPrediction.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/Unit.h"
#include "CommonLib/UnitPartitioner.h"
#include "CommonLib/IbcHashMap.h"
#include "CommonLib/DeblockingFilter.h"
#include "CommonLib/BilateralFilter.h"
#include "DecoderLib/DecCu.h"

#include "CABACWriter.h"
#include "IntraSearch.h"
#include "InterSearch.h"
#include "RateCtrl.h"
#include "EncModeCtrl.h"
//! \ingroup EncoderLib
//! \{

class EncLib;
class HLSWriter;
class EncSlice;
class EncGOP;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// CU encoder class
struct GeoMergeCombo
{
  int          splitDir;
  int          bldIdx;
  MergeIdxPair mergeIdx;
  double       cost;
  GeoMergeCombo() : splitDir(0), bldIdx(0), mergeIdx { 0, 0 }, cost(0.0) {};
  GeoMergeCombo(int _splitDir, int _bldIdx, const MergeIdxPair &idx, double _cost)
    : splitDir(_splitDir)
    , bldIdx(_bldIdx)
    , mergeIdx(idx)
    , cost(_cost) {};
};

class GeoComboCostList
{
public:
  GeoComboCostList() {};
  ~GeoComboCostList() {};
  std::vector<GeoMergeCombo> list;

  void sortByCost()
  {
    std::stable_sort(list.begin(), list.end(),
                     [](const GeoMergeCombo &a, const GeoMergeCombo &b) { return a.cost < b.cost; });
  };
};

struct GeoCost
{
  double mode[GEO_NUM_PARTITION_MODE];
  double mergeIdx[MRG_MAX_NUM_CANDS];
  double MMVDFlag[2];
  double MMVDIdx[GPM_EXT_MMVD_MAX_REFINE_NUM2];
  double TMFlag[2];
  double IntraFlag[2];
  double BldFlag[GEO_NUM_BLD];
};

class FastGeoCostList
{
  int m_maxNumGeoCand { 0 };

  using CostArray = double[GEO_NUM_PARTITION_MODE][2];

  CostArray *m_singleDistList { nullptr };

public:
  FastGeoCostList() {}
  ~FastGeoCostList()
  {
    delete[] m_singleDistList;
    m_singleDistList = nullptr;
  }

  void init(int maxNumGeoCand)
  {
    if (m_maxNumGeoCand != maxNumGeoCand)
    {
      delete[] m_singleDistList;
      m_singleDistList = nullptr;

      CHECK(maxNumGeoCand > GPM_MODES_TOTAL, "Too many candidates");
      m_singleDistList = new CostArray[maxNumGeoCand];
      m_maxNumGeoCand  = maxNumGeoCand;
    }
  }

  void insert(int geoIdx, int partIdx, int mergeIdx, double cost)
  {
    CHECKD(cost < 0, "invalid cost");
    CHECKD(geoIdx >= GEO_NUM_PARTITION_MODE, "geoIdx is too large");
    CHECKD(mergeIdx >= m_maxNumGeoCand, "mergeIdx is too large");
    CHECKD(partIdx >= 2, "partIdx is too large");

    m_singleDistList[mergeIdx][geoIdx][partIdx] = cost;
  }

  double getCost(const int splitDir, const MergeIdxPair &mergeCand)
  {
    return m_singleDistList[mergeCand[0]][splitDir][0] + m_singleDistList[mergeCand[1]][splitDir][1];
  }
};

class MergeItem
{
private:
  PelStorage              m_pelStorage;
  std::vector<MotionInfo> m_mvStorage;

public:
  enum class MergeItemType
  {
    REGULAR,
    SBTMVP,
    AFFINE,
    AFF_MMVD,
    MMVD,
    CIIP,
    GPM,
    IBC,
    NUM,
  };

  double                                    cost;
  std::array<MvField[2], AFFINE_MAX_NUM_CP> mvField;
  int                                       mergeIdx;
  uint8_t                                   bcwIdx;
  uint8_t                                   interDir;
  bool                                      useAltHpelIf;
  AffineModel                               affineType;
  CIIP_Type                                 ciipMode;
  bool                                      mergeIdxAlreadyCheckByCIIP;

  bool noResidual;
  bool noBdofRefine;

  bool lumaPredReady;
  bool chromaPredReady;

  bool obmcDone;

  MergeItemType mergeItemType;
  MotionBuf     mvBuf;
  bool          bdmvrRefine;

  bool   bmMergeFlag;
  int8_t bmDir;

  bool              licFlag;
  LicScaleAndOffset licScaleAndOffset;
  bool              oppositeLicFlag;

  MergeItem();
  ~MergeItem();

  void create(ChromaFormat chromaFormat, const Area &area);
  void importMergeInfo(const MergeCtx &mergeCtx, int _mergeIdx, MergeItemType _mergeItemType, CodingUnit &cu);
  void importMergeInfo(const GeoMergeCtx &mergeCtx, int _mergeIdx, MergeItemType _mergeItemType, CodingUnit &cu);
  void importMergeInfo(const AffineMergeCtx &mergeCtx, int _mergeIdx, MergeItemType _mergeItemType, CodingUnit &cu);
  bool exportMergeInfo(CodingUnit &cu, bool forceNoResidual);
  PelUnitBuf getPredBuf(const UnitArea &unitArea) { return m_pelStorage.getCompactBuf(unitArea); }
  MotionBuf  getMvBuf(const UnitArea &unitArea)
  {
    return MotionBuf(m_mvStorage.data(), g_miScaling.scale(unitArea.lumaSize()));
  }
  CMotionBuf getMvBuf(const UnitArea &unitArea) const
  {
    return CMotionBuf(m_mvStorage.data(), g_miScaling.scale(unitArea.lumaSize()));
  }

  static unsigned getGpmUnfiedIndex(unsigned splitDir, unsigned bldIdx, const MergeIdxPair &geoMergeIdx)
  {
    CHECKD(splitDir > 63, "invalid split dir");
    CHECKD(bldIdx > 5, "invalid bld idx");
    CHECKD(geoMergeIdx[0] > 0x7ff, "invalid bld idx");
    CHECKD(geoMergeIdx[1] > 0x7ff, "invalid bld idx");
    return (bldIdx << 28) | (splitDir << 22) | (geoMergeIdx[0] << 11) | geoMergeIdx[1];
  }

  static void updateGpmIdx(unsigned mergeIdx, uint8_t &splitDir, uint8_t &bldIdx, MergeIdxPair &geoMergeIdx)
  {
    bldIdx         = (mergeIdx >> 28) & 0x7;
    splitDir       = (mergeIdx >> 22) & 0x3F;
    geoMergeIdx[0] = (mergeIdx >> 11) & 0x7FF;
    geoMergeIdx[1] = mergeIdx & 0x7FF;
  }
};

class MergeItemList
{
private:
  Pool<MergeItem>          m_mergeItemPool;
  std::vector<MergeItem *> m_list;
  size_t                   m_maxTrackingNum = 0;
  ChromaFormat             m_chromaFormat;
  Area                     m_ctuArea;

public:
  MergeItemList();
  ~MergeItemList();

  void       init(size_t maxSize, ChromaFormat chromaFormat, int ctuWidth, int ctuHeight);
  MergeItem *allocateNewMergeItem();
  void       insertMergeItemToList(MergeItem *p, const size_t startPos = 0);
  void       resetList(size_t maxTrackingNum);
  size_t     getListSize() const { return m_maxTrackingNum; }
  void       setListSize(size_t s)
  {
    shrink(s);
    m_maxTrackingNum = s;
  }
  MergeItem *getMergeItemInList(size_t index);
  size_t     size() { return m_list.size(); }
  void       shrink(size_t maxTrackingNum);
};

class EncCu : DecCu
{
private:
  struct CtxPair
  {
    Ctx start;
    Ctx best;
  };

  std::vector<CtxPair> m_ctxBuffer;
  CtxPair             *m_CurrCtx;
  CtxPool             *m_ctxPool;

  //  Data : encoder control
  int m_cuChromaQpOffsetIdxPlus1;   // if 0, then cu_chroma_qp_offset_flag will be 0, otherwise cu_chroma_qp_offset_flag
                                    // will be 1.

  XuPool         m_unitPool;
  PelUnitBufPool m_pelUnitBufPool;

  static const int maxCuDepth = (MAX_CU_DEPTH + 1 - MIN_CU_LOG2) << 1;

  CodingStructure  *m_pTempCS[maxCuDepth];
  CodingStructure  *m_pBestCS[maxCuDepth];
  CodingStructure  *m_obmcCS[maxCuDepth];
  //  Access channel
  const EncCfg     *m_encCfg;
  IntraSearch      *m_pcIntraSearch;
  InterSearch      *m_pcInterSearch;
  TrQuant          *m_pcTrQuant;
  RdCost           *m_pcRdCost;
  EncSlice         *m_pcSliceEncoder;
  DeblockingFilter *m_deblockingFilter;
  BilateralFilter  *m_bilateralFilter;
  EncGOP           *m_pcGOPEncoder;

  CABACWriter *m_CABACEstimator;
  RateCtrl    *m_pcRateCtrl;
  IbcHashMap   m_ibcHashMap;
  EncModeCtrl *m_modeCtrl;

  PelStorage  m_acMergeTmpBuffer[MRG_MAX_NUM_CANDS];
  PelStorage *m_predWoObmcTmp;
  PelStorage *m_predWoObmcBest;
  EncTestMode m_etmObmcBest;

  std::vector<GeoMergeCombo> m_geoLists[GEO_NUM_PARTITION_MODE];
  FastGeoCostList            m_geoCostList;
  double                     m_AFFBestSATDCost;
  double                     m_mergeBestSATDCost;

  bool m_fastGpmMmvdSearch;
  bool m_includeMoreMMVDCandFirstPass;
  int  m_maxNumGPMDirFirstPass;

  MotionInfo m_SubPuMiBuf[(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)];

  int m_ctuIbcSearchRangeX;
  int m_ctuIbcSearchRangeY;

  std::array<int, 2>    m_bestBcwIdx;
  std::array<double, 2> m_bestBcwCost;

#if SHARP_LUMA_DELTA_QP || ENABLE_QPA_SUB_CTU
  void updateLambda(Slice *slice, const double dQP,
#if WCG_EXT && ER_CHROMA_QP_WCG_PPS
                    const bool useWCGChromaControl,
#endif
                    const bool updateRdCostLambda);
#endif
  double m_sbtCostSave[2];

  GeoComboCostList m_comboList;
  MergeItemList    m_mergeItemList;
  MergeItemList    m_mergeItemListGeoMMVD;

public:
  /// copy parameters from encoder class
  void init(EncLib *pcEncLib, EncModeCtrl *pcEncModeCtrl, const SPS &sps);

  void setDecCuReshaperInEncCU(EncReshape *pcReshape, ChromaFormat chromaFormatIdc)
  {
    initDecCuReshaper((Reshape *)pcReshape, chromaFormatIdc);
  }
  /// create internal buffers
  void create(const EncCfg *encCfg);

  /// destroy internal buffers
  void destroy();

  /// CTU analysis function
  void compressCtu(CodingStructure &cs, const UnitArea &area, const unsigned ctuRsAddr,
                   const EnumArray<int, ChannelType> &prevQP, const EnumArray<int, ChannelType> &currQP);
  /// CTU encoding function
  int  updateCtuDataISlice(const CPelBuf buf);

  EncModeCtrl *getModeCtrl() { return m_modeCtrl; }

  IbcHashMap &getIbcHashMap() { return m_ibcHashMap; }

  EncCu();
  ~EncCu();

protected:
  void       xCalDebCost(CodingStructure &cs, Partitioner &partitioner);
  Distortion getDistortionDb(CodingStructure &cs, CPelBuf org, CPelBuf reco, CompID compID, const CompArea &compArea,
                             bool afterDb);

  void prepare(CodingStructure *&tempCS, Partitioner &partitioner, EncTestMode &currTestMode);

  void xCompressCU(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm,
                   double maxCostAllowed = MAX_DOUBLE);
  void xCheckSplitModes(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, double &maxCostAllowed);
  void xCheckNonSplitModes(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, double &maxCostAllowed);

  bool xCheckBestMode(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm,
                      const EncTestMode &encTestmode, bool useEncDbOpt = false);

  void xCheckModeSplit(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm,
                       const EncTestMode &encTestMode);
  void xCheckModeSplitCore(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner,
                           const EncTestMode &encTestMode);

  bool xCheckRDCostIntra(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm,
                         const EncTestMode &encTestMode);

  void xCheckDQP(CodingStructure &cs, Partitioner &partitioner, bool bKeepCtx = false);
  void xCheckChromaQPOffset(CodingStructure &cs, Partitioner &partitioner);

  void xCheckRDCostHashInter(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm,
                             const EncTestMode &encTestMode);
  void xCheckRDCostInter(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm,
                         const EncTestMode &encTestMode);
  bool xCheckRDCostInterAmvr(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm,
                             const EncTestMode &encTestMode, double &bestIntPelCost);
  void xEncodeDontSplit(CodingStructure &cs, Partitioner &partitioner);

  bool xApplyObmc(CodingStructure &cs, CodingUnit &cu);
  void xUpdateBestObmcCS(CodingStructure &cs, Partitioner &pm, const EncTestMode &encTestMode);
  void xCheckRDCostInterWoObmc(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm);

  void xCheckRDCostUnifiedMerge(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm,
                                const EncTestMode &encTestMode);

  bool xEncodeInterResidual(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner,
                            const EncTestMode &encTestMode, int residualPass, bool *bestHasNonResi, double *equBcwCost);
#if REUSE_CU_RESULTS
  void xReuseCachedResult(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner);
#endif
  bool xIsBcwSkip(const CodingUnit &cu)
  {
    if (cu.slice->m_eSliceType != B_SLICE)
    {
      return true;
    }
    return ((m_encCfg->m_iQP > 32) &&
            ((cu.slice->m_uiTLayer >= 4) ||
             ((cu.refIdxBi[0] >= 0 && cu.refIdxBi[1] >= 0) &&
              (abs(cu.slice->m_poc - cu.slice->getRefPOC(RPL0, cu.refIdxBi[0])) == 1 ||
               abs(cu.slice->m_poc - cu.slice->getRefPOC(RPL1, cu.refIdxBi[1])) == 1))));
  }
  void xCheckRDCostIBCMode(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm,
                           const EncTestMode &encTestMode);
  void xCheckRDCostIBCModeMerge2Nx2N(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner,
                                     const EncTestMode &encTestMode);

  void xCheckPLT(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner,
                 const EncTestMode &encTestMode);

  CodingUnit  *getPuForInterPrediction(CodingStructure *cs);
  unsigned int updateRdCheckingNum(MergeItemList &mergeItemList, double threshold, unsigned int numMergeSatdCand);

  void   generateMergePrediction(const UnitArea &unitArea, MergeItem *mergeItem, CodingUnit &cu, bool luma, bool chroma,
                                 PelUnitBuf &dstBuf, bool finalRd, bool forceNoResidual, PelUnitBuf *predBuf1,
                                 PelUnitBuf *predBuf2, const MergeCtx *mrgCtx, bool obmc = false);
  double calcLumaCost4MergePrediction(const TempCtx &ctxStart, const PelUnitBuf &predBuf, double lambda, CodingUnit &cu,
                                      DistParam &distParam);
  double calcRate4MergePrediction(const TempCtx &ctxStart, const PelUnitBuf &predBuf, double lambda, CodingUnit &cu,
                                  DistParam &distParam);
  void   addRegularCandsToPruningList(MergeItemList &mergeItemList, const MergeCtx &mergeCtx,
                                      const UnitArea &localUnitArea, double sqrtLambdaForFirstPassIntra,
                                      const TempCtx &ctxStart, PelUnitBufVector<MRG_MAX_NUM_CANDS> *mrgPredBufNoCiip,
                                      PelUnitBufVector<MRG_MAX_NUM_CANDS> *mrgPredBufNoMvRefine, DistParam &distParam,
                                      CodingUnit *cu);
  void addCiipCandsToPruningList(MergeItemList &mergeItemList, const MergeCtx &mergeCtx, const UnitArea &localUnitArea,
                                 double sqrtLambdaForFirstPassIntra, const TempCtx &ctxStart,
                                 PelUnitBufVector<MRG_MAX_NUM_CANDS> &mrgPredBufNoCiip,
                                 PelUnitBufVector<MRG_MAX_NUM_CANDS> &mrgPredBufNoMvRefine, DistParam &distParam,
                                 CodingUnit *cu);
#if 0   // this function is not used
  void addMmvdCandsToPruningList(MergeItemList& mergeItemList, const GeoMergeCtx& geoMergeCtx, const UnitArea& localUnitArea, double sqrtLambdaForFirstPassIntra,
    const TempCtx& ctxStart, DistParam& distParam, CodingUnit* cu);
#endif
  void addMmvdCandsToPruningList(MergeItemList &mergeItemList, const MergeCtx &mergeCtx, const UnitArea &localUnitArea,
                                 double sqrtLambdaForFirstPassIntra, const TempCtx &ctxStart, DistParam &distParam,
                                 CodingUnit *cu);
  void addAffineCandsToPruningList(MergeItemList &mergeItemList, AffineMergeCtx &affineMergeCtx,
                                   const UnitArea &localUnitArea, double sqrtLambdaForFirstPass,
                                   const TempCtx &ctxStart, DistParam &distParam, CodingUnit *cu);
  void addAffineMmvdCandsToPruningList(MergeItemList &mergeItemList, AffineMergeCtx &affineMergeCtx,
                                       const UnitArea &localUnitArea, double sqrtLambdaForFirstPass,
                                       const TempCtx &ctxStart, DistParam &distParam, CodingUnit *cu);
  void addBMCandsToPruningList(MergeItemList &mergeItemList, const MergeCtx &mergeCtx, const UnitArea &localUnitArea,
                               double sqrtLambdaForFirstPassIntra, const TempCtx &ctxStart, DistParam &distParam,
                               CodingUnit *cu);

  struct GeoHelper
  {
    int16_t setGeoBufIntraPred(int intraPred)
    {
      int16_t i = 0;
      for (; i < (int16_t)intraPredBufs.size(); i++)
      {
        if (intraPredBufs[i] == intraPred)
        {
          return i;
        }
      }

      intraPredBufs.push_back(intraPred);
      return i;
    }

    int16_t getGeoBufIntraIdx(int splitDir, int partIdx, int intraCandIdx) const
    {
      int intraPred = geoIntraMPMList(splitDir, partIdx, intraCandIdx);

      for (int16_t i = 0; i < (int16_t)intraPredBufs.size(); i++)
      {
        if (intraPredBufs[i] == intraPred)
        {
          return i;
        }
      }
      THROW("no valid entry found");
      return -1;
    }

    int geoIntraMPMList(int splitDir, int partIdx, int intraCandIdx) const
    {
      CHECK(intraCandIdx >= GEO_MAX_NUM_INTRA_CANDS, "intraIdx out of range");
      CHECK(partIdx >= 2, "partIdx out of range");
      CHECK(splitDir >= GEO_NUM_PARTITION_MODE, "splitDir out of range");
      int intraPredMode = m_geoIntraMPMList[splitDir][partIdx][intraCandIdx];
      CHECK(intraPredMode >= NUM_INTRA_MODE, "intraPredMode out of range");
      return intraPredMode;
    }

    static_vector<int8_t, GPM_EXT_MMVD_MAX_REFINE_NUM> intraPredBufs;
    uint8_t m_geoIntraMPMList[GEO_NUM_PARTITION_MODE][2][GEO_MAX_NUM_INTRA_CANDS];
    GeoCost geoCost;
  };

  template<size_t N> double addGpmCandsToPruningList(MergeItemList &mergeItemList, const GeoMergeCtx &geoMergeCtx,
                                                     const UnitArea &localUnitArea, double sqrtLambdaForFirstPass,
                                                     const TempCtx &ctxStart, const GeoComboCostList &comboList,
                                                     const GeoHelper &gh, PelUnitBufVector<N> &geoBuffer,
                                                     DistParam &distParamSAD2, CodingUnit *cu, size_t pos);

  template<size_t N> bool prepareGpmComboList(GeoMergeCtx &geoMergeCtx, const UnitArea &localUnitArea,
                                              double sqrtLambdaForFirstPass, GeoComboCostList &comboList,
                                              PelUnitBufVector<N> &geoBuffer, GeoHelper &gh, CodingUnit *cu,
                                              bool useMMVDS);
  void                    checkEarlySkip(const CodingStructure *bestCS, const Partitioner &partitioner);
};

//! \}

#endif   // __ENCMB__
