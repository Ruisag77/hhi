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

/** \file     ContextModelling.h
 *  \brief    Classes providing probability descriptions and contexts (header)
 */

#ifndef __CONTEXTMODELLING__
#define __CONTEXTMODELLING__

#include "Rom.h"
#include "CommonDef.h"
#include "Contexts.h"
#include "Slice.h"
#include "Unit.h"
#include "UnitPartitioner.h"
#include "CodingStructure.h"

#include <bitset>

struct CoeffCodingContext
{
public:
  static const int prefixCtx[9];

  CoeffCodingContext(const TransformUnit &tu, CompID component, bool signHide, const BdpcmMode bdpcm);

public:
  void initSubblock(int SubsetId, bool sigGroupFlag = false);

public:
  void resetSigGroup() { m_sigCoeffGroupFlag.reset(m_subSetPos); }
  void setSigGroup() { m_sigCoeffGroupFlag.set(m_subSetPos); }
  bool noneSigGroup() { return m_sigCoeffGroupFlag.none(); }
  int  lastSubSet() { return (maxNumCoeff() - 1) >> log2CGSize(); }
  bool isLastSubSet() { return lastSubSet() == m_subSetId; }
  bool only1stSigGroup() { return m_sigCoeffGroupFlag.count() - m_sigCoeffGroupFlag[lastSubSet()] == 0; }
  void setScanPosLast(int posLast) { m_scanPosLast = posLast; }

  const Size &getSignPredArea() { return m_signPredArea; }
  bool        isSignPredCG()
  {
    return ((m_subSetPosX << m_log2CGWidth) < m_signPredArea.width &&
            (m_subSetPosY << m_log2CGHeight) < m_signPredArea.height);
  }
  uint32_t &numSignsPredArea() { return m_numSignsPredArea; }

  static inline bool getSwitchCondition(const CodingUnit &cu, ChannelType channelType)
  {
    bool condition = cu.predMode == MODE_INTRA && cu.slice->m_eSliceType != I_SLICE;
    //&& ( ( isLuma(channelType) && !cu.tmpFlag) || ( isChroma(channelType) && cu.intraDir[ChannelType::CHROMA] !=
    // DBV_CHROMA_IDX));

    CHECK(cu.slice->m_sps->m_dualITree && cu.slice->m_eSliceType == I_SLICE && channelType != cu.chType,
          "not supported");
    // int tmpMaxSize = cu.cs->sps->getIntraTMPMaxSize();
    condition = condition || (cu.predMode == MODE_IBC && isLuma(channelType) && cu.slice->m_eSliceType == I_SLICE);
    //|| (cu.predMode == MODE_INTRA &&
    //   ((isLuma(channelType) && cu.tmpFlag && !cu.bdpcmMode && !cu.dimd && cu.lwidth() <= tmpMaxSize && cu.lheight()
    //   <= tmpMaxSize)
    //   || (isChroma(channelType) && cu.intraDir[ChannelType::CHROMA] == DBV_CHROMA_IDX)) && cu.slice->m_eSliceType ==
    //   I_SLICE);
    return condition;
  }

public:
  CompID   compID() const { return m_compID; }
  int      subSetId() const { return m_subSetId; }
  int      subSetPos() const { return m_subSetPos; }
  int      cgPosY() const { return m_subSetPosY; }
  int      cgPosX() const { return m_subSetPosX; }
  int      nstIdx() const { return m_nstIdx; }
  unsigned width() const { return m_width; }
  unsigned height() const { return m_height; }
  unsigned log2CGWidth() const { return m_log2CGWidth; }
  unsigned log2CGHeight() const { return m_log2CGHeight; }
  unsigned log2CGSize() const { return m_log2CGSize; }
  bool     extPrec() const { return m_extendedPrecision; }
  int      maxLog2TrDRange() const { return m_maxLog2TrDynamicRange; }
  unsigned maxNumCoeff() const { return m_maxNumCoeff; }
  int      scanPosLast() const { return m_scanPosLast; }
  int      minSubPos() const { return m_minSubPos; }
  int      maxSubPos() const { return m_maxSubPos; }
  bool     isLast() const { return ((m_scanPosLast >> m_log2CGSize) == m_subSetId); }
  bool     isNotFirst() const { return (m_subSetId != 0); }
  bool     isSigGroup(int scanPosCG) const { return m_sigCoeffGroupFlag[m_scanCG[scanPosCG].idx]; }
  bool     isSigGroup() const { return m_sigCoeffGroupFlag[m_subSetPos]; }
  bool     signHiding() const { return m_signHiding; }
  bool     hideSign(int posFirst, int posLast) const { return (m_signHiding && (posLast - posFirst >= SBH_THRESHOLD)); }
  unsigned blockPos(int scanPos) const { return m_scan[scanPos].idx; }
  unsigned posX(int scanPos) const { return m_scan[scanPos].x; }
  unsigned posY(int scanPos) const { return m_scan[scanPos].y; }
  unsigned maxLastPosX() const { return m_maxLastPosX; }
  unsigned maxLastPosY() const { return m_maxLastPosY; }
  unsigned lastXCtxId(unsigned posLastX) const { return m_CtxSetLastX(m_lastOffsetX + (posLastX >> m_lastShiftX)); }
  unsigned lastYCtxId(unsigned posLastY) const { return m_CtxSetLastY(m_lastOffsetY + (posLastY >> m_lastShiftY)); }
  unsigned sigGroupCtxId(bool ts = false) const
  {
    return (m_switchCondition ? (ts ? m_sigGroupCtxIdTSSwitch : m_sigGroupCtxIdSwitch)
                              : (ts ? m_sigGroupCtxIdTS : m_sigGroupCtxId));
  }
  BdpcmMode bdpcm() const { return m_bdpcm; }

  TCoeff minCoeff() const { return m_minCoeff; }
  TCoeff maxCoeff() const { return m_maxCoeff; }

  unsigned sigCtxIdAbs(int scanPos, const TCoeff *coeff, const int state)
  {
    const uint32_t posY   = m_scan[scanPos].y;
    const uint32_t posX   = m_scan[scanPos].x;
    const TCoeff  *pData  = coeff + posX + posY * m_width;
    const int      diag   = posX + posY;
    int            numPos = 0;
    TCoeff         sumAbs = 0;
#define UPDATE(x)                               \
  {                                             \
    TCoeff a = abs(x);                          \
    sumAbs += std::min(GTN_LEVEL + (a & 1), a); \
    numPos += int(!!a);                         \
  }
    if (m_nstIdx)
    {
      const int mpos = std::min<int>(scanPos + 5, scanPosLast());
      for (int spos = scanPos + 1; spos <= mpos; spos++)
      {
        UPDATE(coeff[blockPos(spos)]);
      }
    }
    else
    {
      if (posX < m_width - 1)
      {
        UPDATE(pData[1]);
        if (posX < m_width - 2)
        {
          UPDATE(pData[2]);
        }
        if (posY < m_height - 1)
        {
          UPDATE(pData[m_width + 1]);
        }
      }
      if (posY < m_height - 1)
      {
        UPDATE(pData[m_width]);
        if (posY < m_height - 2)
        {
          UPDATE(pData[m_width << 1]);
        }
      }
    }
#undef UPDATE

    int ctxOfs = int(std::min<TCoeff>((sumAbs + 1) >> 1, NSIGCTX - 1)) + (diag < 2 ? NSIGCTX : 0);

    if (isLuma(m_chType))
    {
      ctxOfs += diag < 5 ? NSIGCTX : 0;
    }

    m_tmplCpDiag = diag;
    m_tmplCpSum1 = sumAbs - numPos;
    return m_sigFlagCtxSet[state & 3](ctxOfs);
  }

  uint8_t ctxOffsetAbs()
  {
    int offset = 0;
    if (m_tmplCpDiag != -1)
    {
      offset = int(std::min<TCoeff>(m_tmplCpSum1, NGTXCTX - 1)) + 1;
      offset += (!m_tmplCpDiag        ? (isLuma(m_chType) ? NGTXCTX * 3 : NGTXCTX)
                   : isLuma(m_chType) ? m_tmplCpDiag < 3 ? NGTXCTX * 2 : (m_tmplCpDiag < 10 ? NGTXCTX : 0)
                                      : 0);
    }
    return uint8_t(offset);
  }

  unsigned greaterXCtxIdAbs(int k, uint8_t offset) const
  {
    static const uint8_t ctxMapping[GTN + 1] = {
      0 /*unused*/, 1, 2, 0, 3, 3, 3, 3
    };   // could reorder ctx init arrays instead
    return m_gtxFlagCtxSet[ctxMapping[k]](offset);
  }

  unsigned templateAbsSum(int scanPos, const TCoeff *coeff, int baseLevel)
  {
    const uint32_t posY  = m_scan[scanPos].y;
    const uint32_t posX  = m_scan[scanPos].x;
    const TCoeff  *pData = coeff + posX + posY * m_width;
    TCoeff         sum   = 0;
    if (m_nstIdx)
    {
      const int mpos = std::min<int>(scanPos + 5, scanPosLast());
      for (int spos = scanPos + 1; spos <= mpos; spos++)
      {
        sum += abs(coeff[blockPos(spos)]);
      }
    }
    else
    {
      if (posX < m_width - 1)
      {
        sum += abs(pData[1]);
        if (posX < m_width - 2)
        {
          sum += abs(pData[2]);
        }
        else
        {
          sum += m_histValue;
        }
        if (posY < m_height - 1)
        {
          sum += abs(pData[m_width + 1]);
        }
        else
        {
          sum += m_histValue;
        }
      }
      else
      {
        sum += 2 * m_histValue;
      }

      if (posY < m_height - 1)
      {
        sum += abs(pData[m_width]);
        if (posY < m_height - 2)
        {
          sum += abs(pData[m_width << 1]);
        }
        else
        {
          sum += m_histValue;
        }
      }
      else
      {
        sum += m_histValue;
      }
    }
    return unsigned(std::max<TCoeff>(std::min<TCoeff>(sum - 5 * baseLevel, 31), 0));
  }

  unsigned templateAbsSumGTN(int scanPos, const TCoeff *coeff)
  {
    const uint32_t posY  = m_scan[scanPos].y;
    const uint32_t posX  = m_scan[scanPos].x;
    const TCoeff  *pData = coeff + posX + posY * m_width;
    TCoeff         sum   = 0;
    TCoeff         c     = 0;

    if (m_nstIdx)
    {
      const int mpos = std::min<int>(scanPos + 5, scanPosLast());
      for (int spos = scanPos + 1; spos <= mpos; spos++)
      {
        c = abs(coeff[blockPos(spos)]);
        sum += (c - (TCoeff) !!c);
      }
    }
    else
    {
      if (posX < m_width - 1)
      {
        c = abs(pData[1]);
        sum += (c - (TCoeff) !!c);
        if (posX < m_width - 2)
        {
          c = abs(pData[2]);
          sum += (c - (TCoeff) !!c);
        }
        if (posY < m_height - 1)
        {
          c = abs(pData[m_width + 1]);
          sum += (c - (TCoeff) !!c);
        }
      }
      if (posY < m_height - 1)
      {
        c = abs(pData[m_width]);
        sum += (c - (TCoeff) !!c);
        if (posY < m_height - 2)
        {
          c = abs(pData[m_width << 1]);
          sum += (c - (TCoeff) !!c);
        }
      }
    }
    return unsigned(std::max<TCoeff>(std::min<TCoeff>(sum, GTN_MAXSUM - 1), 0));
  }

  void updateRiceStat(unsigned &riceStat, TCoeff rem, int remainderFlag)
  {
    if (remainderFlag)
    {
      riceStat = (riceStat + floorLog2((uint32_t)rem) + 2) >> 1;
    }
    else
    {
      riceStat = (riceStat + floorLog2((uint32_t)rem)) >> 1;
    }
  }

  static unsigned templateAbsCompare(TCoeff sum)
  {
    for (int rangeIdx = 0; rangeIdx < g_riceThreshold.size(); rangeIdx++)
    {
      if (sum < g_riceThreshold[rangeIdx])
      {
        return g_riceShift[rangeIdx];
      }
    }

    return g_riceShift[g_riceThreshold.size()];
  }

  unsigned templateAbsSumExt(int scanPos, const TCoeff *coeff, int baseLevel)
  {
    unsigned       riceParam;
    const uint32_t posY = m_scan[scanPos].y;
    const uint32_t posX = m_scan[scanPos].x;
    const TCoeff  *data = coeff + posX + posY * m_width;
    TCoeff         sum  = 0;
    if (m_nstIdx)
    {
      const int mpos = std::min<int>(scanPos + 5, scanPosLast());
      for (int spos = scanPos + 1; spos <= mpos; spos++)
      {
        sum += abs(coeff[blockPos(spos)]);
      }
    }
    else
    {
      if (posX < m_width - 1)
      {
        sum += abs(data[1]);
        if (posX < m_width - 2)
        {
          sum += abs(data[2]);
        }
        else
        {
          sum += m_histValue;
        }

        if (posY < m_height - 1)
        {
          sum += abs(data[m_width + 1]);
        }
        else
        {
          sum += m_histValue;
        }
      }
      else
      {
        sum += 2 * m_histValue;
      }
      if (posY < m_height - 1)
      {
        sum += abs(data[m_width]);
        if (posY < m_height - 2)
        {
          sum += abs(data[m_width << 1]);
        }
        else
        {
          sum += m_histValue;
        }
      }
      else
      {
        sum += m_histValue;
      }
    }

    const int currentShift = templateAbsCompare(sum);
    sum                    = sum >> currentShift;
    if (baseLevel == 0)
    {
      riceParam = unsigned(std::min<TCoeff>(sum, 31));
    }
    else
    {
      riceParam = unsigned(std::max<TCoeff>(std::min<TCoeff>(sum - baseLevel, 31), 0));
    }

    riceParam = g_goRiceParsCoeff[riceParam] + currentShift;

    return riceParam;
  }

  unsigned (CoeffCodingContext::*deriveRiceRRC)(int scanPos, const TCoeff *coeff, int baseLevel);

  unsigned deriveRice(int scanPos, const TCoeff *coeff, int baseLevel)
  {
    unsigned sumAbs    = templateAbsSum(scanPos, coeff, baseLevel);
    unsigned riceParam = g_goRiceParsCoeff[sumAbs];
    return riceParam;
  }

  unsigned deriveRiceExt(int scanPos, const TCoeff *coeff, int baseLevel)
  {
    unsigned riceParam = templateAbsSumExt(scanPos, coeff, baseLevel);
    return riceParam;
  }

  unsigned deriveRiceGTN(int scanPos, const TCoeff *coeff)
  {
    unsigned sumAbs    = templateAbsSumGTN(scanPos, coeff);
    unsigned riceParam = g_goRiceParsCoeffGTN[sumAbs];
    return riceParam;
  }

  unsigned sigCtxIdAbsTS(int scanPos, const TCoeff *coeff)
  {
    const uint32_t posY   = m_scan[scanPos].y;
    const uint32_t posX   = m_scan[scanPos].x;
    const TCoeff  *posC   = coeff + posX + posY * m_width;
    int            numPos = 0;
#define UPDATE(x)       \
  {                     \
    TCoeff a = abs(x);  \
    numPos += int(!!a); \
  }
    if (posX > 0)
    {
      UPDATE(posC[-1]);
    }
    if (posY > 0)
    {
      UPDATE(posC[-(int)m_width]);
    }
#undef UPDATE

    return m_tsSigFlagCtxSet(numPos);
  }

  unsigned parityCtxIdAbsTS() const { return m_tsParFlagCtxSet(0); }
  unsigned greaterXCtxIdAbsTS(uint8_t offset) const { return m_tsGtxFlagCtxSet(offset); }

  unsigned lrg1CtxIdAbsTS(int scanPos, const TCoeff *coeff, const BdpcmMode bdpcm)
  {
    const uint32_t posY = m_scan[scanPos].y;
    const uint32_t posX = m_scan[scanPos].x;
    const TCoeff  *posC = coeff + posX + posY * m_width;

    int numPos = 0;
#define UPDATE(x)       \
  {                     \
    TCoeff a = abs(x);  \
    numPos += int(!!a); \
  }

    if (bdpcm != BdpcmMode::NONE)
    {
      numPos = 3;
    }
    else
    {
      if (posX > 0)
      {
        UPDATE(posC[-1]);
      }
      if (posY > 0)
      {
        UPDATE(posC[-(int)m_width]);
      }
    }

#undef UPDATE
    return m_tsLrg1FlagCtxSet(numPos);
  }

  unsigned signCtxIdAbsTS(int scanPos, const TCoeff *coeff, const BdpcmMode bdpcm)
  {
    const uint32_t posY  = m_scan[scanPos].y;
    const uint32_t posX  = m_scan[scanPos].x;
    const TCoeff  *pData = coeff + posX + posY * m_width;

    int      rightSign = 0, belowSign = 0;
    unsigned signCtx = 0;

    if (posX > 0)
    {
      rightSign = sgn(pData[-1]);
    }
    if (posY > 0)
    {
      belowSign = sgn(pData[-(int)m_width]);
    }

    if ((rightSign == 0 && belowSign == 0) || ((rightSign * belowSign) < 0))
    {
      signCtx = 0;
    }
    else if (rightSign >= 0 && belowSign >= 0)
    {
      signCtx = 1;
    }
    else
    {
      signCtx = 2;
    }
    if (bdpcm != BdpcmMode::NONE)
    {
      signCtx += 3;
    }
    return m_tsSignFlagCtxSet(signCtx);
  }

  void neighTS(int &rightPixel, int &belowPixel, int scanPos, const TCoeff *coeff)
  {
    const uint32_t posY = m_scan[scanPos].y;
    const uint32_t posX = m_scan[scanPos].x;
    const TCoeff  *data = coeff + posX + posY * m_width;

    rightPixel = belowPixel = 0;

    if (posX > 0)
    {
      rightPixel = int(data[-1]);
    }
    if (posY > 0)
    {
      belowPixel = int(data[-(int)m_width]);
    }
  }

  int deriveModCoeff(int rightPixel, int belowPixel, TCoeff absCoeff, const bool bdpcm)
  {

    if (absCoeff == 0)
    {
      return 0;
    }
    int pred1, absBelow = abs(belowPixel), absRight = abs(rightPixel);

    int absCoeffMod = int(absCoeff);

    if (!bdpcm)
    {
      pred1 = std::max(absBelow, absRight);

      if (absCoeffMod == pred1)
      {
        absCoeffMod = 1;
      }
      else
      {
        absCoeffMod = absCoeffMod < pred1 ? absCoeffMod + 1 : absCoeffMod;
      }
    }

    return (absCoeffMod);
  }

  TCoeff decDeriveModCoeff(int rightPixel, int belowPixel, TCoeff absCoeff)
  {

    if (absCoeff == 0)
    {
      return 0;
    }

    int pred1, absBelow = abs(belowPixel), absRight = abs(rightPixel);
    pred1 = std::max(absBelow, absRight);

    TCoeff absCoeffMod;

    if (absCoeff == 1 && pred1 > 0)
    {
      absCoeffMod = pred1;
    }
    else
    {
      absCoeffMod = absCoeff - (absCoeff <= pred1);
    }
    return (absCoeffMod);
  }

  unsigned templateAbsSumTS(int scanPos, const TCoeff *coeff) { return 1; }

  int remRegBins;

  unsigned getBaseLevel() { return m_cctxBaseLevel; };
  void     setBaseLevel(int value) { m_cctxBaseLevel = value; };
  TCoeff   getHistValue() { return m_histValue; };
  void     setHistValue(TCoeff value) { m_histValue = value; };
  bool     getUpdateHist() { return m_updateHist; };
  void     setUpdateHist(bool value) { m_updateHist = value; };

private:
  // constant
  const CompID             m_compID;
  const ChannelType        m_chType;
  const int                m_nstIdx;
  const unsigned           m_width;
  const unsigned           m_height;
  const unsigned           m_log2CGWidth;
  const unsigned           m_log2CGHeight;
  const unsigned           m_log2CGSize;
  const unsigned           m_widthInGroups;
  const unsigned           m_heightInGroups;
  const unsigned           m_log2BlockWidth;
  const unsigned           m_log2BlockHeight;
  const unsigned           m_maxNumCoeff;
  const bool               m_signHiding;
  const bool               m_extendedPrecision;
  const int                m_maxLog2TrDynamicRange;
  const ScanElement       *m_scan;
  const ScanElement       *m_scanCG;
  const bool               m_switchCondition;
  const CtxSet             m_CtxSetLastX;
  const CtxSet             m_CtxSetLastY;
  const unsigned           m_maxLastPosX;
  const unsigned           m_maxLastPosY;
  const int                m_lastOffsetX;
  const int                m_lastOffsetY;
  const int                m_lastShiftX;
  const int                m_lastShiftY;
  const TCoeff             m_minCoeff;
  const TCoeff             m_maxCoeff;
  // modified
  int                      m_scanPosLast;
  int                      m_subSetId;
  int                      m_subSetPos;
  int                      m_subSetPosX;
  int                      m_subSetPosY;
  int                      m_minSubPos;
  int                      m_maxSubPos;
  unsigned                 m_sigGroupCtxId;
  TCoeff                   m_tmplCpSum1;
  int                      m_tmplCpDiag;
  unsigned                 m_sigGroupCtxIdSwitch;
  unsigned                 m_sigGroupCtxIdTSSwitch;
  CtxSet                   m_sigFlagCtxSet[4];
  CtxSet                   m_gtxFlagCtxSet[4];
  unsigned                 m_sigGroupCtxIdTS;
  CtxSet                   m_tsSigFlagCtxSet;
  CtxSet                   m_tsParFlagCtxSet;
  CtxSet                   m_tsGtxFlagCtxSet;
  CtxSet                   m_tsLrg1FlagCtxSet;
  CtxSet                   m_tsSignFlagCtxSet;
  std::bitset<MLS_GRP_NUM> m_sigCoeffGroupFlag;
  const BdpcmMode          m_bdpcm;
  Size                     m_signPredArea;
  uint32_t                 m_numSignsPredArea;
  int                      m_cctxBaseLevel;
  TCoeff                   m_histValue;
  bool                     m_updateHist;
};

class CUCtxIntra
{
public:
  CUCtxIntra() : mpmListSize(0) {}
  ~CUCtxIntra() {}
  uint8_t mpmList[NUM_MOST_PROBABLE_MODES];
  uint8_t nonMPMList[NUM_NON_MPM_MODES];
  int     mpmListSize;
};

class CUCtx : public CUCtxIntra
{
public:
  CUCtx() : isDQPCoded(false), isChromaQpAdjCoded(false), qgStart(false)
  {
    violatesLfnstConstrained.fill(false);
    lfnstLastScanPos           = false;
    violatesMtsCoeffConstraint = false;
    mtsLastScanPos             = false;
    mtsCoeffAbsSum             = 0;
    mpmListSize                = 0;
  }
  CUCtx(int _qp) : CUCtxIntra(), isDQPCoded(false), isChromaQpAdjCoded(false), qgStart(false), qp(_qp)
  {
    violatesLfnstConstrained.fill(false);
    lfnstLastScanPos           = false;
    violatesMtsCoeffConstraint = false;
    mtsLastScanPos             = false;
    mtsCoeffAbsSum             = 0;
    mpmListSize                = 0;
  }
  ~CUCtx() {}

public:
  bool                         isDQPCoded;
  bool                         isChromaQpAdjCoded;
  bool                         qgStart;
  bool                         lfnstLastScanPos;
  int8_t                       qp;   // used as a previous(last) QP and for QP prediction
  EnumArray<bool, ChannelType> violatesLfnstConstrained;
  bool                         violatesMtsCoeffConstraint;
  bool                         mtsLastScanPos;
  int64_t                      mtsCoeffAbsSum;
};

struct DeriveCtx
{
  const CodingUnit *cuRestrictedLeft[to_underlying(ChannelType::NUM)]  = { nullptr, nullptr };
  const CodingUnit *cuRestrictedAbove[to_underlying(ChannelType::NUM)] = { nullptr, nullptr };

  void setNeighbourCus(const CodingStructure &cs, const UnitArea &ua, const ChannelType ch);
  void CtxSplit(const CodingStructure &cs, Partitioner &partitioner, unsigned &ctxSpl, unsigned &ctxQt, unsigned &ctxHv,
                unsigned &ctxHorBt, unsigned &ctxVerBt, bool *canSplit = nullptr) const;
  static unsigned CtxQtCbf(const CompID compID, const bool prevCbf = false);
  unsigned        CtxInterDir(const CodingUnit &cu) const;
  unsigned        CtxSkipFlag(const CodingUnit &cu) const;
  unsigned        CtxAffineFlag(const CodingUnit &cu) const;
  unsigned        CtxBMMrgFlag(const CodingUnit &cu) const;
  unsigned        CtxPredModeFlag(const CodingUnit &cu) const;
  unsigned        CtxIBCFlag(const CodingUnit &cu) const;
  unsigned        CtxMipFlag(const CodingUnit &cu) const;
  unsigned        CtxDimdFlag(const CodingUnit &cu) const;
  unsigned        CtxTimdFlag(const CodingUnit &cu) const;
  unsigned        CtxSgpmFlag(const CodingUnit &cu) const;
  static unsigned CtxPltCopyFlag(const unsigned prevRunType, const unsigned dist);
};

#endif   // __CONTEXTMODELLING__
