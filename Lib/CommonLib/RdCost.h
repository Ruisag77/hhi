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

/** \file     RdCost.h
    \brief    RD cost computation classes (header)
*/

#ifndef __RDCOST__
#define __RDCOST__

#include "CommonDef.h"
#include "Mv.h"
#include "Unit.h"
#include "Buffer.h"
#include "Slice.h"
#include "RdCostWeightPrediction.h"
#include <math.h>
#include <functional>

//! \ingroup CommonLib
//! \{

struct EstBvdBitsStruct
{
  uint32_t bitsGt0FlagH[2];
  uint32_t bitsGt0FlagV[2];

  uint32_t bitsH[BVD_IBC_MAX_PREFIX];
  uint32_t bitsV[BVD_IBC_MAX_PREFIX];

  uint32_t bitsIdx[2];
  uint32_t bitsImv[2];
  uint32_t bitsFracImv[4];
};

struct DistParam;

// ====================================================================================================================
// Type definition
// ====================================================================================================================

using DistFunc = std::function<Distortion(const DistParam &)>;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// distortion parameter class
struct DistParam
{
  CPelBuf org {};
  CPelBuf cur {};
#if WCG_EXT
  CPelBuf orgLuma {};
#endif
  const Pel *mask { nullptr };
  ptrdiff_t  maskStride { 0 };
  int        stepX { 0 };
  ptrdiff_t  maskStride2 { 0 };
  int        step { 1 };
  DistFunc   distFunc { nullptr };
  int        bitDepth { 0 };
  int        inputDepth {};

  bool useMR { false };
  bool applyWeight { false };     // whether weighted prediction is used or not
  bool isBiPred { false };

  const WPScalingParam *wpCur { nullptr };           // weighted prediction scaling parameters for current ref
  CompID                compID { MAX_NUM_COMP };
  Distortion            maximumDistortionForEarlyExit {
    std::numeric_limits<Distortion>::max()
  }; /// During cost calculations, if distortion exceeds this value, cost calculations may early-terminate.

  // (vertical) subsampling shift (for reducing complexity)
  // - 0 = no subsampling, 1 = even rows, 2 = every 4th, etc.
  int subShift { 0 };
  int cShiftX { -1 };
  int cShiftY { -1 };
  int tmWeightIdx { 0 };
};

/// RD cost computation class
class RdCost
{
private:
  // for distortion

  static EnumArray<DistFunc, DFunc> m_distortionFunc;
  CostMode                          m_costMode;
  double                            m_distortionWeight[MAX_NUM_COMP]; // only chroma values are used.
  double                            m_dLambda;
  bool                              m_isLosslessRDCost;

#if WCG_EXT
  double m_dLambda_unadjusted; // TODO: check is necessary
  double m_distScaleUnadjusted;

  static std::vector<int32_t> m_reshapeLumaLevelToWeightPLUT;   // scaled by MSE_WEIGHT_ONE
  static std::vector<double>  m_lumaLevelToWeightPLUT;

  static int32_t  m_chromaWeight;   // scaled by MSE_WEIGHT_ONE
  static uint32_t m_signalType;
  static int      m_lumaBD;

  ChromaFormat m_cf;
#endif
  double m_distScale;
  double m_dLambdaMotionSAD;
  double m_lambdaStore[2][3];   // 0-org; 1-act
  double m_distScaleStore[2][3];   // 0-org; 1-act
  bool   m_resetStore;
  int    m_pairCheck;

  // for motion cost
  Mv     m_mvPredictor;
  Mv     m_bvPredictors[2];
  double m_motionLambda;
  int    m_iCostScale;

  double           m_dCost; // for ibc
  EstBvdBitsStruct m_cBvdBitCosts;
  bool             m_useFullPelImvForZeroBvd;

  template<bool allowOddSizes> static DFuncDiff sizeOffset(const uint32_t width)
  {
    if (isPowerOf2(width) && width <= 64)
    {
      return static_cast<DFuncDiff>(std::min(7, floorLog2(width)));
    }
    else if (width == 128 || width == 256 || width % 16 == 0)
    {
      return DFunc::SAD16N - DFunc::SAD;
    }
    else if (allowOddSizes && width == 12)
    {
      return DFunc::SAD12 - DFunc::SAD;
    }
    else if (allowOddSizes && width == 24)
    {
      return DFunc::SAD24 - DFunc::SAD;
    }
    else if (allowOddSizes && width == 48)
    {
      return DFunc::SAD48 - DFunc::SAD;
    }
    else
    {
      return DFunc::SAD - DFunc::SAD;
    }
  }

public:
  RdCost();
  virtual ~RdCost();

#if WCG_EXT
  void   setChromaFormat(const ChromaFormat &_cf) { m_cf = _cf; }
  double calcRdCost(uint64_t fracBits, Distortion distortion, bool useUnadjustedLambda = true);
#else
  double calcRdCost(uint64_t fracBits, Distortion distortion);
#endif

  void setDistortionWeight(const CompID compID, const double distortionWeight)
  {
    m_distortionWeight[compID] = distortionWeight;
  }
  void setLambda(double dLambda, const BitDepths &bitDepths);

#if WCG_EXT
  double getLambda(bool unadj = false) { return unadj ? m_dLambda_unadjusted : m_dLambda; }
#else
  double getLambda() { return m_dLambda; }
#endif
  double getChromaWeight() { return ((m_distortionWeight[COMP_Cb] + m_distortionWeight[COMP_Cr]) / 2.0); }
#if RDOQ_CHROMA_LAMBDA
  double getDistortionWeight(const CompID compID) const { return m_distortionWeight[compID % MAX_NUM_COMP]; }
#endif

  void setCostMode(CostMode m) { m_costMode = m; }
  void setLosslessRDCost(bool m) { m_isLosslessRDCost = m; }

  // Distortion Functions
  void init();
#ifdef TARGET_SIMD_X86
  void                         initRdCostX86();
  template<X86_VEXT vext> void _initRdCostX86();
#endif

  void setDistParam(DistParam &rcDP, const CPelBuf &org, const Pel *piRefY, ptrdiff_t iRefStride, int bitDepth,
                    CompID compID, int subShiftMode = 0, int step = 1, int useHadamard = 0);
  void setDistParam(DistParam &rcDP, const CPelBuf &org, const CPelBuf &cur, int bitDepth, CompID compID,
                    int useHadamard = 0);
  void setDistParam(DistParam &rcDP, const Pel *pOrg, const Pel *piRefY, ptrdiff_t iOrgStride, ptrdiff_t iRefStride,
                    int bitDepth, CompID compID, int width, int height, int subShiftMode = 0, int step = 1,
                    int useHadamard = 0, bool bioApplied = false);
  void setDistParam(DistParam &rcDP, const CPelBuf &org, const Pel *piRefY, ptrdiff_t iRefStride, const Pel *mask,
                    ptrdiff_t iMaskStride, int stepX, ptrdiff_t iMaskStride2, int bitDepth, CompID compID);
  void setTimdDistParam(DistParam &rcDP, const Pel *pOrg, const Pel *piRefY, ptrdiff_t iOrgStride, ptrdiff_t iRefStride,
                        int bitDepth, CompID compID, int width, int height, int subShiftMode = 0, int step = 1,
                        int useHadamard = 0);
  double            getMotionLambda() { return m_dLambdaMotionSAD; }
  void              selectMotionLambda() { m_motionLambda = getMotionLambda(); }
  void              setPredictor(const Mv &rcMv) { m_mvPredictor = rcMv; }
  void              setCostScale(int iCostScale) { m_iCostScale = iCostScale; }
  Distortion        getCost(uint32_t b) { return Distortion(m_motionLambda * b); }
  // for ibc
  void              getMotionCost(int add) { m_dCost = m_dLambdaMotionSAD + add; }
  inline Distortion getBvCostSingle(Mv mv, AMVPInfo &amvpInfo, uint8_t imv, uint8_t imvForZeroBvd, bool zeroMvdCheckX,
                                    bool zeroMvdCheckY, uint32_t addExtraBits, uint8_t &mvpIdx)
  {
    Mv bv = mv;
    bv.changeIbcPrecInternal2Amvr(imv);

    auto getMvBinCost = [&](int mvpIdx)
    {
      Mv bvp = amvpInfo.mvCand[mvpIdx];
      bvp.changeIbcPrecInternal2Amvr(imv);
      Mv bvd = bv - bvp;

      if (imv != imvForZeroBvd && (zeroMvdCheckX || bvd.getHor() == 0) &&
          (zeroMvdCheckY || bvd.getVer() == 0)) // note: zero mvd is allowed only for default IMV mode
      {
        return std::numeric_limits<uint32_t>::max();
      }
      else
      {
        uint32_t binCost = (zeroMvdCheckX ? 0 : xGetExpGolombNumberOfBitsIBCH(bvd.getHor())) +
          (zeroMvdCheckY ? 0 : xGetExpGolombNumberOfBitsIBCV(bvd.getVer())) + m_cBvdBitCosts.bitsIdx[mvpIdx];
        if (!(imv == imvForZeroBvd && (zeroMvdCheckX || bvd.getHor() == 0) && (zeroMvdCheckY || bvd.getVer() == 0)))
        {
          binCost += m_cBvdBitCosts.bitsFracImv[imv];
        }
        return binCost;
      }
    };

    uint32_t b0       = getMvBinCost(0);
    uint32_t b1       = getMvBinCost(1);
    uint32_t bBest    = (b1 < b0) ? b1 : b0;
    int      bBestIdx = (b1 < b0) ? 1 : 0;

    mvpIdx = bBestIdx;
    return Distortion(m_dCost * bBest) >> SCALE_BITS;
  }
  void setFullPelImvForZeroBvd(bool b) { m_useFullPelImvForZeroBvd = b; }

  void setPredictors(Mv *pcMv)
  {
    for (int i = 0; i < 2; i++)
    {
      m_bvPredictors[i] = pcMv[i];
    }
  }

  EstBvdBitsStruct *getBvdBitCosts() { return &m_cBvdBitCosts; }
  inline Distortion getBvCostMultiplePreds(int x, int y, bool useIMV, uint8_t *bvImvResBest = NULL,
                                           int *bvpIdxBest = NULL)
  {
    uint32_t b0 = xGetExpGolombNumberOfBitsIBCH(x - m_bvPredictors[0].getHor()) +
      xGetExpGolombNumberOfBitsIBCV(y - m_bvPredictors[0].getVer()) + m_cBvdBitCosts.bitsIdx[0];
    uint32_t b1 = xGetExpGolombNumberOfBitsIBCH(x - m_bvPredictors[1].getHor()) +
      xGetExpGolombNumberOfBitsIBCV(y - m_bvPredictors[1].getVer()) + m_cBvdBitCosts.bitsIdx[1];

    if (useIMV)
    {
      b0 += (x != m_bvPredictors[0].getHor() || y != m_bvPredictors[0].getVer()) ? m_cBvdBitCosts.bitsImv[0] : 0;
      b1 += (x != m_bvPredictors[1].getHor() || y != m_bvPredictors[1].getVer()) ? m_cBvdBitCosts.bitsImv[0] : 0;
    }
    uint32_t bBest    = (b1 < b0) ? b1 : b0;
    int      bBestIdx = (b1 < b0) ? 1 : 0;
    uint8_t  bestRes =
      (useIMV && (x != m_bvPredictors[bBestIdx].getHor() || y != m_bvPredictors[bBestIdx].getVer())) ? 1 : 0;
    if (bvImvResBest)
    {
      *bvImvResBest = IMV_FPEL;
      *bvpIdxBest   = bBestIdx;
    }

    if (bestRes && useIMV && x % 4 == 0 && y % 4 == 0)
    {
      Mv cMv(x >> 2, y >> 2);

      Mv tmpBv0 = m_bvPredictors[0];
      Mv tmpBv1 = m_bvPredictors[1];
      tmpBv0.changePrecision(MvPrecision::ONE /*MV_PRECISION_INT*/, MvPrecision::FOUR /*MV_PRECISION_4PEL*/);
      tmpBv1.changePrecision(MvPrecision::ONE /*MV_PRECISION_INT*/, MvPrecision::FOUR /*MV_PRECISION_4PEL*/);

      uint32_t bQ0 = (cMv == tmpBv0)
        ? std::numeric_limits<uint32_t>::max()
        : (xGetExpGolombNumberOfBitsIBCH(cMv.getHor() - tmpBv0.getHor()) +
           xGetExpGolombNumberOfBitsIBCV(cMv.getVer() - tmpBv0.getVer()) + m_cBvdBitCosts.bitsIdx[0]);
      uint32_t bQ1 = (cMv == tmpBv1)
        ? std::numeric_limits<uint32_t>::max()
        : (xGetExpGolombNumberOfBitsIBCH(cMv.getHor() - tmpBv1.getHor()) +
           xGetExpGolombNumberOfBitsIBCV(cMv.getVer() - tmpBv1.getVer()) + m_cBvdBitCosts.bitsIdx[1]);

      uint32_t bQBest = (bQ1 < bQ0) ? bQ1 : bQ0;
      bQBest += (bQBest < std::numeric_limits<uint32_t>::max()) ? m_cBvdBitCosts.bitsImv[1] : 0;

      if (bQBest < bBest)
      {
        if (bvImvResBest)
        {
          *bvImvResBest = 2;
          *bvpIdxBest   = (bQ1 < bQ0) ? 1 : 0;
        }
        bBest = bQBest;
      }
    }
    return Distortion(m_dCost * bBest) >> SCALE_BITS;
  }

  // for motion cost
  static uint32_t xGetExpGolombNumberOfBits(int iVal)
  {
    CHECKD(iVal == std::numeric_limits<int>::min(), "Wrong value");
    unsigned uiLength2 = 1, uiTemp2 = (iVal <= 0) ? (unsigned(-iVal) << 1) + 1 : unsigned(iVal << 1);

    while (uiTemp2 > MAX_CU_SIZE)
    {
      uiLength2 += (MAX_CU_DEPTH << 1);
      uiTemp2 >>= MAX_CU_DEPTH;
    }

    return uiLength2 + (floorLog2(uiTemp2) << 1);
  }
  Distortion getCostOfVectorWithPredictor(const int x, const int y, const unsigned imvShift)
  {
    return Distortion(m_motionLambda * getBitsOfVectorWithPredictor(x, y, imvShift));
  }
  uint32_t getBitsOfVectorWithPredictor(const int x, const int y, const unsigned imvShift)
  {
    return xGetExpGolombNumberOfBits(((x * (1 << m_iCostScale)) - m_mvPredictor.getHor()) >> imvShift) +
      xGetExpGolombNumberOfBits(((y * (1 << m_iCostScale)) - m_mvPredictor.getVer()) >> imvShift);
  }
  // for block vector cost
  uint32_t xGetExpGolombNumberOfBitsIBCH(int iVal)
  {
    CHECKD(iVal == std::numeric_limits<int>::min(), "Wrong value");

    unsigned int temp = (iVal <= 0) ? -iVal : iVal;
    if (!temp)
    {
      return m_cBvdBitCosts.bitsGt0FlagH[0];
    }
    else
    {
      unsigned int order = BVD_CODING_GOLOMB_ORDER;
      unsigned int bins  = 0;
      temp -= 1;

      while (temp >= (1 << order))
      {
        temp -= (1 << order);
        order += 1;
        bins += 1;
      }
      CHECKD(bins >= BVD_IBC_MAX_PREFIX, "Prefix is too large");
      return m_cBvdBitCosts.bitsGt0FlagH[1] + m_cBvdBitCosts.bitsH[bins] + (1 << SCALE_BITS);
    }
  }

  uint32_t xGetExpGolombNumberOfBitsIBCV(int iVal)
  {
    CHECKD(iVal == std::numeric_limits<int>::min(), "Wrong value");

    unsigned int temp = (iVal <= 0) ? -iVal : iVal;
    if (!temp)
    {
      return m_cBvdBitCosts.bitsGt0FlagV[0];
    }
    else
    {
      unsigned int order = BVD_CODING_GOLOMB_ORDER;
      unsigned int bins  = 0;
      temp -= 1;

      while (temp >= (1 << order))
      {
        temp -= (1 << order);
        order += 1;
        bins += 1;
      }
      CHECKD(bins >= BVD_IBC_MAX_PREFIX, "Prefix is too large");
      return m_cBvdBitCosts.bitsGt0FlagV[1] + m_cBvdBitCosts.bitsV[bins] + (1 << SCALE_BITS);
    }
  }

  Distortion getCostOfVectorWithPredictorIBC(const int x, const int y, const unsigned imvShift)
  {
    return Distortion(m_motionLambda * getBitsOfVectorWithPredictorIBC(x, y, imvShift)) >> SCALE_BITS;
  }
  uint32_t getBitsOfVectorWithPredictorIBC(const int x, const int y, const unsigned imvShift)
  {
    return xGetExpGolombNumberOfBitsIBCH(((x << m_iCostScale) - m_mvPredictor.getHor()) >> imvShift) +
      xGetExpGolombNumberOfBitsIBCV(((y << m_iCostScale) - m_mvPredictor.getVer()) >> imvShift);
  }
#if WCG_EXT
  void saveUnadjustedLambda();
  void initLumaLevelToWeightTable(int bitDepth);
  void initLumaLevelToWeightTableReshape();
  void updateReshapeLumaLevelToWeightTableChromaMD(std::vector<Pel> &ILUT);
  void restoreReshapeLumaLevelToWeightTable();
  void updateReshapeLumaLevelToWeightTable(SliceReshapeInfo &sliceReshape, Pel *wtTable, double cwt);

  void setReshapeInfo(uint32_t type, int lumaBD)
  {
    m_signalType = type;
    m_lumaBD     = lumaBD;
  }

  double               getWPSNRLumaLevelWeight(int val) { return m_lumaLevelToWeightPLUT[val]; }
  std::vector<double> &getLumaLevelWeightTable() { return m_lumaLevelToWeightPLUT; }
#endif

  void resetStore() { m_resetStore = true; }

private:
  static Distortion xGetSSE(const DistParam &pcDtParam);
  static Distortion xGetSSE4(const DistParam &pcDtParam);
  static Distortion xGetSSE8(const DistParam &pcDtParam);
  static Distortion xGetSSE16(const DistParam &pcDtParam);
  static Distortion xGetSSE32(const DistParam &pcDtParam);
  static Distortion xGetSSE64(const DistParam &pcDtParam);
  static Distortion xGetSSE16N(const DistParam &pcDtParam);

#if WCG_EXT
  static inline Distortion getWeightedMSE(int compIdx, const Pel org, const Pel cur, const uint32_t shift,
                                          const Pel orgLuma);
  static Distortion        xGetSSE_WTD(const DistParam &pcDtParam);
  static Distortion        xGetSSE2_WTD(const DistParam &pcDtParam);
  static Distortion        xGetSSE4_WTD(const DistParam &pcDtParam);
  static Distortion        xGetSSE8_WTD(const DistParam &pcDtParam);
  static Distortion        xGetSSE16_WTD(const DistParam &pcDtParam);
  static Distortion        xGetSSE32_WTD(const DistParam &pcDtParam);
  static Distortion        xGetSSE64_WTD(const DistParam &pcDtParam);
  static Distortion        xGetSSE16N_WTD(const DistParam &pcDtParam);
#endif

  static Distortion xGetSAD(const DistParam &pcDtParam);
  static Distortion xGetSAD4(const DistParam &pcDtParam);
  static Distortion xGetSAD8(const DistParam &pcDtParam);
  static Distortion xGetSAD16(const DistParam &pcDtParam);
  static Distortion xGetSAD32(const DistParam &pcDtParam);
  static Distortion xGetSAD64(const DistParam &pcDtParam);
  static Distortion xGetSAD16N(const DistParam &pcDtParam);

  static Distortion xGetSAD12(const DistParam &pcDtParam);
  static Distortion xGetSAD24(const DistParam &pcDtParam);
  static Distortion xGetSAD48(const DistParam &pcDtParam);

  static Distortion xGetSAD_full(const DistParam &pcDtParam);
  static Distortion xGetSADwMask(const DistParam &pcDtParam);

  static Distortion                               xGetMRSAD(const DistParam &pcDtParam);
  static Distortion                               xGetMRSAD4(const DistParam &pcDtParam);
  static Distortion                               xGetMRSAD8(const DistParam &pcDtParam);
  static Distortion                               xGetMRSAD16(const DistParam &pcDtParam);
  static Distortion                               xGetMRSAD32(const DistParam &pcDtParam);
  static Distortion                               xGetMRSAD64(const DistParam &pcDtParam);
  static Distortion                               xGetMRSAD16N(const DistParam &pcDtParam);
  static Distortion                               xGetMRSAD12(const DistParam &pcDtParam);
  static Distortion                               xGetMRSAD24(const DistParam &pcDtParam);
  static Distortion                               xGetMRSAD48(const DistParam &pcDtParam);
  template<bool isFast = false> static Distortion xGetMRHADs(const DistParam &pcDtParam);

  template<bool isFast = false> static Distortion xGetHADs(const DistParam &pcDtParam);
  static Distortion xCalcHADs4x4(const Pel *piOrg, const Pel *piCurr, ptrdiff_t strideOrg, ptrdiff_t strideCur,
                                 int step);
  static Distortion xCalcHADs8x8(const Pel *piOrg, const Pel *piCurr, ptrdiff_t strideOrg, ptrdiff_t strideCur,
                                 int step);
  static Distortion xCalcHADs2x2(const Pel *piOrg, const Pel *piCurr, ptrdiff_t strideOrg, ptrdiff_t strideCur,
                                 int step);

  static Distortion xCalcHADs16x8(const Pel *piOrg, const Pel *piCur, ptrdiff_t strideOrg, ptrdiff_t strideCur);
  static Distortion xCalcHADs8x16(const Pel *piOrg, const Pel *piCur, ptrdiff_t strideOrg, ptrdiff_t strideCur);
  static Distortion xCalcHADs4x8(const Pel *piOrg, const Pel *piCur, ptrdiff_t strideOrg, ptrdiff_t strideCur);
  static Distortion xCalcHADs8x4(const Pel *piOrg, const Pel *piCur, ptrdiff_t strideOrg, ptrdiff_t strideCur);

#ifdef TARGET_SIMD_X86
  template<X86_VEXT vext> static Distortion            xGetSSE_SIMD(const DistParam &pcDtParam);
  template<int width, X86_VEXT vext> static Distortion xGetSSE_NxN_SIMD(const DistParam &pcDtParam);

  template<X86_VEXT vext> static Distortion            xGetSAD_SIMD(const DistParam &pcDtParam);
  template<int width, X86_VEXT vext> static Distortion xGetSAD_NxN_SIMD(const DistParam &pcDtParam);
  template<X86_VEXT vext> static Distortion            xGetSAD_IBD_SIMD(const DistParam &pcDtParam);
  template<X86_VEXT vext> static Distortion            xGetHADs_SIMD(const DistParam &pcDtParam);
  template<X86_VEXT vext> static Distortion            xGetHADsFst_SIMD(const DistParam &pcDtParam);

  template<X86_VEXT vext> static Distortion xGetSADwMask_SIMD(const DistParam &pcDtParam);
  template<X86_VEXT vext> static Distortion xGetMRSAD_SIMD(const DistParam &rcDtParam);
#endif

public:
#if WCG_EXT
  Distortion getDistPart(const CPelBuf &org, const CPelBuf &cur, int bitDepth, const CompID compID, DFunc distFunc,
                         const CPelBuf *orgLuma = nullptr, int inputDepth = 0);
#else
  Distortion getDistPart(const CPelBuf &org, const CPelBuf &cur, int bitDepth, const CompID compID, DFunc distFunc);
#endif

  Distortion getDistPart(const CPelBuf &org, const CPelBuf &cur, const Pel *mask, int bitDepth, const CompID compID,
                         DFunc distFunc);
};// END CLASS DEFINITION RdCost

//! \}

#endif // __RDCOST__
