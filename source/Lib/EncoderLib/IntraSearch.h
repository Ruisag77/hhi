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

/** \file     IntraSearch.h
    \brief    intra search class (header)
*/

#ifndef __INTRASEARCH__
#define __INTRASEARCH__

// Include files

#include "CABACWriter.h"

#include "CommonLib/IntraPrediction.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/Unit.h"
#include "CommonLib/RdCost.h"
#include "CommonLib/BilateralFilter.h"
#include "EncReshape.h"

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================
class EncModeCtrl;
struct EncCfg;

enum PLTScanMode
{
  PLT_SCAN_HORTRAV = 0,
  PLT_SCAN_VERTRAV = 1,
  NUM_PLT_SCAN     = 2
};

class SortingElement
{
public:
  SortingElement()
  {
    cnt[0] = cnt[1] = cnt[2] = cnt[3] = 0;
    shift[0] = shift[1] = shift[2] = 0;
    lastCnt[0] = lastCnt[1] = lastCnt[2] = 0;
    data[0] = data[1] = data[2] = 0;
    sumData[0] = sumData[1] = sumData[2] = 0;
  }
  uint32_t getCnt(int idx) const { return cnt[idx]; }
  void     setCnt(uint32_t val, int idx) { cnt[idx] = val; }
  int      getSumData(int id) const { return sumData[id]; }

  void resetAll(CompID compBegin, uint32_t numComp)
  {
    shift[0] = shift[1] = shift[2] = 0;
    lastCnt[0] = lastCnt[1] = lastCnt[2] = 0;
    for (int ch = compBegin; ch < (compBegin + numComp); ch++)
    {
      data[ch]    = 0;
      sumData[ch] = 0;
    }
  }
  void setAll(uint32_t *ui, CompID compBegin, uint32_t numComp)
  {
    for (int ch = compBegin; ch < (compBegin + numComp); ch++)
    {
      data[ch] = ui[ch];
    }
  }
  bool almostEqualData(SortingElement element, int errorLimit, const BitDepths &bitDepths, CompID compBegin,
                       uint32_t numComp, bool lossless)
  {
    bool almostEqual = true;
    for (int comp = compBegin; comp < (compBegin + numComp); comp++)
    {
      if (lossless)
      {
        if ((std::abs(data[comp] - element.data[comp])) > errorLimit)
        {
          almostEqual = false;
          break;
        }
      }
      else
      {
        uint32_t absError = 0;
        if (isChroma((CompID)comp))
        {
          absError += int(double(std::abs(data[comp] - element.data[comp])) * PLT_CHROMA_WEIGHTING) >>
            (bitDepths[ChannelType::CHROMA] - PLT_ENCBITDEPTH);
        }
        else
        {
          absError += (std::abs(data[comp] - element.data[comp])) >> (bitDepths[ChannelType::LUMA] - PLT_ENCBITDEPTH);
        }
        if (absError > errorLimit)
        {
          almostEqual = false;
          break;
        }
      }
    }
    return almostEqual;
  }
  uint32_t getSAD(SortingElement element, const BitDepths &bitDepths, CompID compBegin, uint32_t numComp, bool lossless)
  {
    uint32_t sumAd = 0;
    for (int comp = compBegin; comp < (compBegin + numComp); comp++)
    {
      ChannelType chType = (comp > 0) ? ChannelType::CHROMA : ChannelType::LUMA;
      if (lossless)
      {
        sumAd += (std::abs(data[comp] - element.data[comp]));
      }
      else
      {
        sumAd += (std::abs(data[comp] - element.data[comp]) >> (bitDepths[chType] - PLT_ENCBITDEPTH));
      }
    }
    return sumAd;
  }
  void copyDataFrom(SortingElement element, CompID compBegin, uint32_t numComp)
  {
    for (int comp = compBegin; comp < (compBegin + numComp); comp++)
    {
      data[comp]    = element.data[comp];
      sumData[comp] = data[comp];
      shift[comp]   = 0;
      lastCnt[comp] = 1;
    }
  }
  void copyAllFrom(SortingElement element, CompID compBegin, uint32_t numComp)
  {
    copyDataFrom(element, compBegin, numComp);
    for (int comp = compBegin; comp < (compBegin + numComp); comp++)
    {
      sumData[comp] = element.sumData[comp];
      cnt[comp]     = element.cnt[comp];
      shift[comp]   = element.shift[comp];
      lastCnt[comp] = element.lastCnt[comp];
    }
    cnt[MAX_NUM_COMP] = element.cnt[MAX_NUM_COMP];
  }
  void addElement(const SortingElement &element, CompID compBegin, uint32_t numComp)
  {
    for (int i = compBegin; i < (compBegin + numComp); i++)
    {
      sumData[i] += element.data[i];
      cnt[i]++;
      if (cnt[i] > 1 && cnt[i] == 2 * lastCnt[i])
      {
        uint32_t rnd = 1 << shift[i];
        shift[i]++;
        data[i]    = (sumData[i] + rnd) >> shift[i];
        lastCnt[i] = cnt[i];
      }
    }
  }

private:
  uint32_t cnt[MAX_NUM_COMP + 1];
  int      shift[MAX_NUM_COMP];
  int      lastCnt[MAX_NUM_COMP];
  int      data[MAX_NUM_COMP];
  int      sumData[MAX_NUM_COMP];
};

/// encoder search class
class IntraSearch : public IntraPrediction
{
private:
  XuPool m_unitPool;

  CodingStructure *m_pTempCS;
  CodingStructure *m_pBestCS;

  CodingStructure **m_pSaveCS;

  struct ModeInfo
  {
    bool          mipFlg { false };       // CU::mipFlag
    bool          mipTrFlg { false };     // CU::mipTransposedFlag
    bool          bvgMipFlg { false };    // CU::bvgMipFlag
    uint8_t       mRefId { 0 };       // CU::multiRefIdx
    uint32_t      modeId { 0 };       // CU::intraDir[ChannelType::LUMA]
    BdpcmMode     bdpcm { BdpcmMode::NONE };        // CU::bdpcnMode[0]
    PlanarDirType plIdx { PlanarDirType::NO_DIR };    // CU::planarIdx
    int8_t        derivedIpm[2] { -1, -1 };// CU::derivedIpm[2]
    bool          dimdFlg { false };      // CU::dimdFlg
    bool          timdFlg { false };      // CU::timdFlag
    bool          timdSadFlg { false };   // CU::timdSadFlag
    bool          obicFlg { false };      // CU::obicFlg
    bool          obicAvailFlg { false }; // CU::obicAvailFlag
    bool          eipFlg { false };       // CU::eipFlg
    bool          eipMergeFlg { false };  // CU::eipMergeFlg
    EipModels     eipModels {};
    uint8_t       inferredDimdMode { 0 };
    bool          sgpmFlg { false };      // CU::sgmp
    SgpmInfo      sgpmInfo {};
    int           bufferIdx { 0 };

    ModeInfo() = default;

    ModeInfo(const bool mipf, const bool miptf, const int mrid, const uint32_t mode, PlanarDirType plDirid,
             const int bufIdx, const bool bvgMipf = false)
      : mipFlg(mipf)
      , mipTrFlg(miptf)
      , bvgMipFlg(bvgMipf)
      , mRefId(mrid)
      , modeId(mode)
      , plIdx(plDirid)
      , bufferIdx(bufIdx)
    {}
    ModeInfo(const bool mipf, const bool miptf, const int mrid, const uint32_t mode, const int bufIdx,
             const bool bvgMipf = false)
      : mipFlg(mipf)
      , mipTrFlg(miptf)
      , bvgMipFlg(bvgMipf)
      , mRefId(mrid)
      , modeId(mode)
      , bufferIdx(bufIdx)
    {}
    ModeInfo(const BdpcmMode bdpcmMode)
      : modeId(bdpcmMode == BdpcmMode::VER ? VER_IDX : HOR_IDX)
      , bdpcm(bdpcmMode)
      , bufferIdx(-1)
    {}
    ModeInfo(const PlanarDirType _plIdx, const int bufIdx) : modeId(PLANAR_IDX), plIdx(_plIdx), bufferIdx(bufIdx) {}
    ModeInfo(const bool _sgpm, const int _sgpmIdx, const SgpmInfo &_sgpmInfo, int _bufIdx)
      : modeId(_sgpmIdx)
      , sgpmFlg { _sgpm }
      , sgpmInfo { _sgpmInfo }
      , bufferIdx(_bufIdx)
    {}
    bool operator==(const ModeInfo &cmp) const
    {
      return (mipFlg == cmp.mipFlg && mipTrFlg == cmp.mipTrFlg && bvgMipFlg == cmp.bvgMipFlg &&
              mRefId == cmp.mRefId && modeId == cmp.modeId && bdpcm == cmp.bdpcm && plIdx == cmp.plIdx &&
              dimdFlg == cmp.dimdFlg && timdFlg == cmp.timdFlg && obicFlg == cmp.obicFlg &&
              obicAvailFlg == cmp.obicAvailFlg && eipFlg == cmp.eipFlg && sgpmFlg == cmp.sgpmFlg &&
              sgpmInfo == cmp.sgpmInfo);
    }
  };

  using ModeInfoList = static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM>;

  enum IntraPredCategory
  {
    IPRED_CAT_SPATIAL = 0,
    IPRED_CAT_CROSSCOMP,
    IPRED_CAT_CROSSCOMP_MERGE,
    IPRED_CAT_CROSSCOMP_MERGE_FUSION,
    IPRED_CAT_CROSSCOMP_DEC_DERIVED,
  };

  struct ChromaModeInfo
  {
    int32_t         modeId            = 0;   // CU::intraDir[ChannelType::Chroma]
    bool            cccmFlag          = false;
    ConvModelType   cccmType          = CONV_MODEL_UNDEFINED;
    CclmOffsets     cclmOffsets       = {};
    CrossCompModels ccModels          = {};
    int             ccMergeInd        = 0;
    int             ccFusionInd       = 0;
    CrossCompModels ccModels1         = {};
    bool            ccFilterFlag      = false;
    bool            decDerivedCcpMode = false;
    bool            dimdFlag          = false;

    int64_t satdCost = 0;

    ChromaModeInfo() {}
    ChromaModeInfo(int32_t _modeId) : modeId(_modeId) {}
    ChromaModeInfo(int32_t _modeId, int64_t _satdCost) : modeId(_modeId), satdCost(_satdCost) {}
    ChromaModeInfo(int _modeId, CrossCompModels &_ccModels, int _ccMergeInd, bool _ccFilterFlag, int64_t _satdCost)
      : modeId(_modeId)
      , ccModels(_ccModels)
      , ccMergeInd(_ccMergeInd)
      , ccFilterFlag(_ccFilterFlag)
      , satdCost(_satdCost)
    {
      cccmType = ccModels.modelCb[0].modelType;
      cccmFlag = cccmType >= CONV_MODEL_CCCM_INTRA_FIRST && cccmType <= CONV_MODEL_CCCM_INTRA_LAST ? 1 : 0;
    }
    ChromaModeInfo(int32_t _modeId, ConvModelType _cccmType, CrossCompModels &_ccModels, bool _ccFilterFlag,
                   int64_t _satdCost)
      : modeId(_modeId)
      , cccmFlag(true)
      , cccmType(_cccmType)
      , ccModels(_ccModels)
      , ccFilterFlag(_ccFilterFlag)
      , satdCost(_satdCost)
    {}
    ChromaModeInfo(int32_t _modeId, CclmOffsets _cclmOffsets, CrossCompModels &_ccModels, int64_t _satdCost)
      : modeId(_modeId)
      , cclmOffsets(_cclmOffsets)
      , ccModels(_ccModels)
      , satdCost(_satdCost)
    {}
    ChromaModeInfo(IntraPredCategory predCat, int _modeId, CrossCompModels &_ccModels0, CrossCompModels &_ccModels1,
                   int _candInd, int64_t _satdCost)
      : modeId(_modeId)
      , ccModels(_ccModels0)
      , ccModels1(_ccModels1)
      , satdCost(_satdCost)
    {
      cccmType = ccModels.modelCb[0].modelType;
      cccmFlag = cccmType >= CONV_MODEL_CCCM_INTRA_FIRST && cccmType <= CONV_MODEL_CCCM_INTRA_LAST ? 1 : 0;

      if (predCat == IPRED_CAT_CROSSCOMP_DEC_DERIVED)
      {
        decDerivedCcpMode = true;
      }
      else if (predCat == IPRED_CAT_CROSSCOMP_MERGE_FUSION)
      {
        ccMergeInd  = 1;
        ccFusionInd = _candInd;
      }
    }
  };

  struct PreTrListLuma
  {
    PreTrListLuma() {}
    ALIGN_DATA(MEMORY_ALIGN_DEF_SIZE, Pel prd[MAX_TB_SIZEY * MAX_TB_SIZEY]) = {};
    ALIGN_DATA(MEMORY_ALIGN_DEF_SIZE, Pel res[MAX_TB_SIZEY * MAX_TB_SIZEY]) = {};
    TransList                 trTypes                                       = {};
    TransBuffer               trCoeffs                                      = {};
    std::pair<int8_t, int8_t> derivedIntraDirs                              = { 0, 1 };
    bool                      valid                                         = false;
  };

  struct PreTrListChroma
  {
    PreTrListChroma() {}
    ALIGN_DATA(MEMORY_ALIGN_DEF_SIZE, Pel prd[2][MAX_TB_SIZEY * MAX_TB_SIZEY]) = {};
    ALIGN_DATA(MEMORY_ALIGN_DEF_SIZE, Pel res[5][MAX_TB_SIZEY * MAX_TB_SIZEY]) = {};
    TransList             trTypesSep                                           = {};
    TransList             trTypesJnt                                           = {};
    TransList             trTypesCmb                                           = {};
    TransList             trTypesJCCR                                          = {};
    CbfMaskList           cbfMaskJCCR                                          = {};
    TransBuffer           trCoeffs[5]                                          = {};
    int                   chromaResScale                                       = 0;
    std::array<int8_t, 4> derivedIntraDirs                                     = { 0, 0, 0, 0 };
    bool                  valid                                                = false;
  };

  struct IModeTrCandLuma : public ModeInfo, public PreTrListLuma
  {
    IModeTrCandLuma() : ModeInfo(), PreTrListLuma() {}
  };

  struct IModeTrCandChroma : public PreTrListChroma
  {
    ChromaModeInfo chromaMode = ChromaModeInfo(-1);
    operator ChromaModeInfo() { return chromaMode; }
    IModeTrCandChroma() : PreTrListChroma() {}
  };

  struct IModeTrCandListLuma : public static_vector<IModeTrCandLuma, 17>
  {
    IModeTrCandListLuma() {}
    IModeTrCandLuma &add(const ModeInfo &m)
    {
      resize_noinit(size() + 1);
      IModeTrCandLuma &last         = back();
      static_cast<ModeInfo &>(last) = m;
      last.trTypes.clear();
      last.valid = false;
      return back();
    }
  };

  struct IModeTrCandListChroma
    : public static_vector<IModeTrCandChroma, ENCODER_INTRA_CHROMA_NUM_RDO + 2> // +2 for BDPCM modes
  {
    IModeTrCandListChroma() {}
    IModeTrCandChroma &add(const ChromaModeInfo m)
    {
      resize_noinit(size() + 1);
      IModeTrCandChroma &last = back();
      last.chromaMode         = m;
      last.trTypesSep.clear();
      last.trTypesJnt.clear();
      last.trTypesCmb.clear();
      last.trTypesJCCR.clear();
      last.cbfMaskJCCR.clear();
      last.valid = false;
      return back();
    }
  };

  // uint8_t m_mpmList[NUM_MOST_PROBABLE_MODES];
  // uint8_t m_nonMPMList[NUM_NON_MPM_MODES];

  PelStorage m_tmpStorageCtu;
  PelStorage m_colorTransResiBuf;

  std::vector<TransformUnit *> m_orgTUs;

  struct SatdCheckerCbCr
  {
    DistParam distParamSadCb;
    DistParam distParamSatdCb;
    DistParam distParamSadCr;
    DistParam distParamSatdCr;

    void init(RdCost &rdCost, CPelBuf &origCb, CPelBuf &origCr, CPelBuf &predCb, CPelBuf &predCr, int bitdepth)
    {
      rdCost.setDistParam(distParamSadCb, origCb, predCb, bitdepth, COMP_Cb, 0);
      rdCost.setDistParam(distParamSatdCb, origCb, predCb, bitdepth, COMP_Cb, 1);
      rdCost.setDistParam(distParamSadCr, origCr, predCr, bitdepth, COMP_Cr, 0);
      rdCost.setDistParam(distParamSatdCr, origCr, predCr, bitdepth, COMP_Cr, 1);

      distParamSadCb.applyWeight  = false;
      distParamSatdCb.applyWeight = false;
      distParamSadCr.applyWeight  = false;
      distParamSatdCr.applyWeight = false;
    }

    int64_t getCost()
    {
      int64_t sadCb  = distParamSadCb.distFunc(distParamSadCb) * 2;
      int64_t satdCb = distParamSatdCb.distFunc(distParamSatdCb);
      int64_t sadCr  = distParamSadCr.distFunc(distParamSadCr) * 2;
      int64_t satdCr = distParamSatdCr.distFunc(distParamSatdCr);

      return std::min(sadCb, satdCb) + std::min(sadCr, satdCr);
    }
  };

  SatdCheckerCbCr m_satdCheckerCbCr;

  void tryAddingToModeList(std::vector<ChromaModeInfo> &modeList, ChromaModeInfo mode, int maxListSize);

protected:
  // interface to option
  const EncCfg *m_encCfg;
  EncModeCtrl  *m_modeCtrl;

  // interface to classes
  BilateralFilter *m_bilateralFilter;
  TrQuant         *m_pcTrQuant;
  RdCost          *m_pcRdCost;
  EncReshape      *m_pcReshape;

  // RD computation
  CABACWriter *m_CABACEstimator;
  CtxPool     *m_ctxPool;

  bool     m_isInitialized;
  bool     m_bestEscape;
  double  *m_indexError[MAXPLTSIZE + 1];
  uint8_t *m_minErrorIndexMap; // store the best index in terms of distortion for each pixel
  uint8_t  m_indexMapRDOQ[2][NUM_TRELLIS_STATE][2 * MAX_CU_BLKSIZE_PLT];
  bool     m_runMapRDOQ[2][NUM_TRELLIS_STATE][2 * MAX_CU_BLKSIZE_PLT];
  uint8_t *m_statePtRDOQ[NUM_TRELLIS_STATE];
  bool     m_prevRunTypeRDOQ[2][NUM_TRELLIS_STATE];
  int      m_prevRunPosRDOQ[2][NUM_TRELLIS_STATE];
  double   m_stateCostRDOQ[2][NUM_TRELLIS_STATE];

  IModeTrCandListLuma   m_modeTrCandListLuma;
  IModeTrCandListChroma m_modeTrCandListChroma;

public:
  IntraSearch();
  ~IntraSearch();

  void init(const EncCfg *encCfg, BilateralFilter *bilateralFilter, TrQuant *pcTrQuant, RdCost *pcRdCost,
            InterpolationFilter *pIf, CABACWriter *CABACEstimator, EncModeCtrl *pcEncModeCtrl, CtxPool *ctxPool,
            const uint32_t maxCUWidth, const uint32_t maxCUHeight, const uint32_t maxTotalCUDepth,
            EncReshape *m_pcReshape, const unsigned bitDepthY);

  void destroy();

  CodingStructure **getSaveCSBuf() { return m_pSaveCS; }

public:
  bool estIntraPredLumaQT(CodingUnit &cu, Partitioner &pm, CUCtxIntra &cuCtxIntra,
                          const double bestCostSoFar = MAX_DOUBLE, CodingStructure *bestCS = nullptr,
                          PelUnitBufPool *pelUnitBufPool = nullptr);
  void estIntraPredChromaQT(CodingUnit &cu, Partitioner &pm, const double maxCostAllowed = MAX_DOUBLE);
  void PLTSearch(CodingStructure &cs, Partitioner &partitioner, CompID compBegin, uint32_t numComp);

  void        sortRdModeListFirstColorSpace(ModeInfo mode, double cost, BdpcmMode bdpcmMode, ModeInfo *rdModeList,
                                            double *rdCostList, BdpcmMode *bdpcmModeList, int &candNum);
  static void setCuPredDataLuma(CodingUnit &cu, const ModeInfo &mi);

  static void setCuPredDataChroma(CodingUnit &cu, const ChromaModeInfo &mi, const uint32_t *chromaCandModes);

protected:
  uint64_t xFracModeBitsIntra(CodingUnit &cu, const uint32_t &mode, const ChannelType &compID,
                              const CUCtxIntra &cuCtxIntra);
  void     xSortRdModeListFirstColorSpace(ModeInfo mode, double cost, BdpcmMode bdpcmMode, ModeInfo *rdModeList,
                                          double *rdCostList, BdpcmMode *bdpcmModeList, int &candNum);

  // -------------------------------------------------------------------------------------------------------------------
  // T & Q & Q-1 & T-1
  // -------------------------------------------------------------------------------------------------------------------

  // -------------------------------------------------------------------------------------------------------------------
  // Intra search
  // -------------------------------------------------------------------------------------------------------------------

  void     xEncIntraHeader(CodingStructure &cs, Partitioner &pm, const bool &luma, const bool &chroma,
                           const CUCtxIntra *cuCtxIntra = nullptr);
  void     xEncSubdivCbfQT(CodingStructure &cs, Partitioner &pm, const bool &luma, const bool &chroma);
  uint64_t xGetIntraFracBitsQT(CodingStructure &cs, Partitioner &pm, const bool &luma, const bool &chroma,
                               CUCtx *cuCtx = nullptr, const CUCtxIntra *cuCtxIntra = nullptr);

  uint64_t xGetIntraFracBitsQTChroma(TransformUnit &tu, const CompID &compID, CUCtx *cuCtx = nullptr);
  void     xEncCoeffQT(CodingStructure &cs, Partitioner &pm, const CompID compID, CUCtx *cuCtx = nullptr);

  void xPredTuLuma(TransformUnit &tu, PelBuf &pred);
  void xPredTuChroma(TransformUnit &tu, PelBuf &predCb, PelBuf &predCr, IModeTrCandChroma &modeCand);

  bool xIntraCodingTUBlockLuma(TransformUnit &tu, Distortion &dist, PreTrListLuma &ptList, TCoeff &absSum);
  void xIntraCodingTUBlockChroma(TransformUnit &tu, const CompID compID, Distortion &dist, PreTrListChroma &ptList);
  bool xRecurIntraCodingLumaQT(CodingStructure &cs, Partitioner &pm, PreTrListLuma &ptList, CUCtxIntra &cuCtxIntra);
  ChromaCbfs xRecurIntraCodingChromaQT(CodingStructure &cs, Partitioner &pm, IModeTrCandChroma &ptList);

  template<typename T, size_t N>
  void xReduceHadCandList(static_vector<T, N> &candModeList, static_vector<double, N> &candCostList,
                          SortedPelUnitBufs &sortedPelBuffer, int &numModesForFullRD, const double thresholdHadCost,
                          const double *mipHadCost, const bool *mipHadBvgFlag, const CodingUnit &cu,
                          const bool fastMip);
  void xDerivePLTLossy(CodingStructure &cs, Partitioner &partitioner, CompID compBegin, uint32_t numComp);
  void xCalcPixelPred(CodingStructure &cs, Partitioner &partitioner, uint32_t yPos, uint32_t xPos, CompID compBegin,
                      uint32_t numComp);
  void xPreCalcPLTIndexRD(CodingStructure &cs, Partitioner &partitioner, CompID compBegin, uint32_t numComp);
  void xCalcPixelPredRD(CodingStructure &cs, Partitioner &partitioner, Pel *orgBuf, Pel *pixelValue, Pel *recoValue,
                        CompID compBegin, uint32_t numComp);
  void xDeriveIndexMap(CodingStructure &cs, Partitioner &partitioner, CompID compBegin, uint32_t numComp,
                       PLTScanMode pltScanMode, double &cost, bool *idxExist);
  bool xDeriveSubblockIndexMap(CodingStructure &cs, Partitioner &partitioner, CompID compBegin, PLTScanMode pltScanMode,
                               int minSubPos, int maxSubPos, const BinFracBits &fracBitsPltRunType,
                               const BinFracBits *fracBitsPltIndexINDEX, const BinFracBits *fracBitsPltIndexCOPY,
                               const double minCost, bool useRotate);
  double xRateDistOptPLT(bool RunType, uint8_t RunIndex, bool prevRunType, uint8_t prevRunIndex, uint8_t aboveRunIndex,
                         bool &prevCodedRunType, int &prevCodedRunPos, int scanPos, uint32_t width, int dist,
                         int indexMaxValue, const BinFracBits *IndexfracBits, const BinFracBits &TypefracBits);
  uint32_t xGetTruncBinBits(uint32_t symbol, uint32_t numSymbols);

  void xPreCalcPrdTransLuma(TransformUnit &tu, PreTrListLuma &tl, const PelBuf *prdBuf = nullptr);
  void xPreCalcPrdTransChroma(TransformUnit &tu, IModeTrCandChroma &tl);
  void xPreCalcPrdTransJCCR(TransformUnit &tu, IModeTrCandChroma &tl);
  void xSelectMTCandLuma(CodingStructure &cs, Partitioner &pt, IModeTrCandListLuma &modeTrCandList,
                         const ModeInfoList &imodeList, const CUCtxIntra &cuCtxIntra,
                         const SortedPelUnitBufs *sortedBufs = nullptr);
  void xSelectMTCandChroma(CodingStructure &cs, Partitioner &pt, IModeTrCandListChroma &modeTrCandList,
                           const std::vector<ChromaModeInfo> &imodeList, const uint32_t *chromaCandModes);

  void xSelectPrTrCandLuma(TransformUnit &tu, IModeTrCandListLuma &ptl, const CUCtxIntra &cuCtxIntra);
  void xSelectPrTrCandChroma(TransformUnit &tu, IModeTrCandListChroma &ptl, const uint32_t *chromaCandModes);
  void xSelectTrCandLuma(TransformUnit &tu, PreTrListLuma &tl);
  void xSelectTrCandChroma(TransformUnit &tu, PreTrListChroma &tl);

  void xAddChromaCands(CodingUnit &cu, std::vector<ChromaModeInfo> &candList);
  void xAddCccmCands(CodingUnit &cu, std::vector<ChromaModeInfo> &candList);
  void xAddCclmDeltaSlopeCands(CodingUnit &cu, std::vector<ChromaModeInfo> &candList);
  void xFindBestCclmDeltaSlopeSATD(CodingUnit &cu, CompID compID, CclmModel &cclmModelBase, int cclmModelInd,
                                   int &deltaBest, int64_t &satdBest);
  void xAddCcMergeCands(CodingUnit &cu, std::vector<ChromaModeInfo> &candList);

  void addDimdChromaAsCandidate(CodingUnit &cu, CompArea &areaCb, CompArea &areaCr, PelBuf &predCb, PelBuf &predCr,
                                std::vector<ChromaModeInfo> &candList);

  class PreCostLuma : public TrEst::PreCostBase
  {
  public:
    PreCostLuma(TransformUnit &tu, CABACWriter *cabacEst, RdCost *rdCost);
    void        init(const IModeTrCandListLuma &il, const int id, const CUCtxIntra &cuCtxIntra);
    void        init(const PreTrListLuma &tl);
    TrEst::Cost operator()(const MtsType tr);

  private:
    double estIModeBits(const ModeInfo &mi, const CUCtxIntra &cuCtxIntra);
    double estTransBits(const MtsType tr);

  private:
    CABACWriter         *bitEst    = nullptr;
    const PreTrListLuma *trList    = nullptr;
    int                  imodeId   = 0;
    double               imodeBits = 0;
  };

  class PreCostChroma : public TrEst::PreCostBase
  {
  public:
    PreCostChroma(TransformUnit &tu, CABACWriter *cabacEst, RdCost *rdCost);
    void        init(const IModeTrCandListChroma &il, const int id, const uint32_t *chromaCandModes);
    void        init(const PreTrListChroma &tl);
    TrEst::Cost operator()(const MtsType tr);

  private:
    double estIModeBits(const ChromaModeInfo mi, const uint32_t *chromaCandModes);
    double estTransBits(const MtsType tr);

  private:
    CABACWriter           *bitEst    = nullptr;
    const PreTrListChroma *trList    = nullptr;
    int                    imodeId   = 0;
    double                 imodeBits = 0;
  };

  static uint32_t getEpExGolombNumBins(uint32_t symbol, uint32_t count);
};   // END CLASS DEFINITION EncSearch

//! \}

#endif   // __ENCSEARCH__
