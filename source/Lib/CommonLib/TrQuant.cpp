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

/** \file     TrQuant.cpp
    \brief    transform and quantization class
*/

#include "TrQuant.h"
#include "TrQuant_EMT.h"

#include "UnitTools.h"
#include "ContextModelling.h"
#include "CodingStructure.h"

#include "dtrace_buffer.h"

#include <stdlib.h>
#include <limits>
#include <memory.h>

#include "QuantRDOQ.h"
#include "DepQuant.h"
#include "Reshape.h"

#if RExt__DECODER_DEBUG_TOOL_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif

struct coeffGroupRDStats
{
  int    iNNZbeforePos0;
  double d64CodedLevelandDist; // distortion and level cost only
  double d64UncodedDist;    // all zero coded block distortion
  double d64SigCost;
  double d64SigCost_0;
};

using FwdTransList = std::array<FwdTrans *, NUM_TRANSFORM_MATRIX_SIZES>;
using InvTransList = std::array<InvTrans *, NUM_TRANSFORM_MATRIX_SIZES>;

static const EnumArray<FwdTransList, TransType> fastFwdTrans = { {
  FwdTransList { fastForwardDCT2_B2, fastForwardDCT2_B4, fastForwardDCT2_B8, fastForwardDCT2_B16, fastForwardDCT2_B32,
                 fastForwardDCT2_B64, fastForwardDCT2_B128, fastForwardDCT2_B256 },
  FwdTransList { nullptr, fastForwardDCT8_B4, fastForwardDCT8_B8, fastForwardDCT8_B16, fastForwardDCT8_B32,
                 fastForwardDCT8_B64, fastForwardDCT8_B128, fastForwardDCT8_B256 },
  FwdTransList { nullptr, fastForwardDST7_B4, fastForwardDST7_B8, fastForwardDST7_B16, fastForwardDST7_B32,
                 fastForwardDST7_B64, fastForwardDST7_B128, fastForwardDST7_B256 },
  FwdTransList { nullptr, fastForwardDCT5_B4, fastForwardDCT5_B8, fastForwardDCT5_B16, fastForwardDCT5_B32,
                 fastForwardDCT5_B64, fastForwardDCT5_B128, fastForwardDCT5_B256 },
  FwdTransList { nullptr, fastForwardDST4_B4, fastForwardDST4_B8, fastForwardDST4_B16, fastForwardDST4_B32,
                 fastForwardDST4_B64, fastForwardDST4_B128, fastForwardDST4_B256 },
  FwdTransList { nullptr, fastForwardDST1_B4, fastForwardDST1_B8, fastForwardDST1_B16, fastForwardDST1_B32,
                 fastForwardDST1_B64, fastForwardDST1_B128, fastForwardDST1_B256 },
  FwdTransList { nullptr, fastForwardIDTR_B4, fastForwardIDTR_B8, fastForwardIDTR_B16, fastForwardIDTR_B32,
                 fastForwardIDTR_B64, fastForwardIDTR_B128, fastForwardIDTR_B256 },
  FwdTransList { nullptr, fastForwardKLT0_B4, fastForwardKLT0_B8, fastForwardKLT0_B16, nullptr, nullptr, nullptr,
                 nullptr },
  FwdTransList { nullptr, fastForwardKLT1_B4, fastForwardKLT1_B8, fastForwardKLT1_B16, nullptr, nullptr, nullptr,
                 nullptr },
} };

static const EnumArray<InvTransList, TransType> fastInvTrans = { {
  InvTransList { fastInverseDCT2_B2, fastInverseDCT2_B4, fastInverseDCT2_B8, fastInverseDCT2_B16, fastInverseDCT2_B32,
                 fastInverseDCT2_B64, fastInverseDCT2_B128, fastInverseDCT2_B256 },
  InvTransList { nullptr, fastInverseDCT8_B4, fastInverseDCT8_B8, fastInverseDCT8_B16, fastInverseDCT8_B32,
                 fastInverseDCT8_B64, fastInverseDCT8_B128, fastInverseDCT8_B256 },
  InvTransList { nullptr, fastInverseDST7_B4, fastInverseDST7_B8, fastInverseDST7_B16, fastInverseDST7_B32,
                 fastInverseDST7_B64, fastInverseDST7_B128, fastInverseDST7_B256 },
  InvTransList { nullptr, fastInverseDCT5_B4, fastInverseDCT5_B8, fastInverseDCT5_B16, fastInverseDCT5_B32,
                 fastInverseDCT5_B64, fastInverseDCT5_B128, fastInverseDCT5_B256 },
  InvTransList { nullptr, fastInverseDST4_B4, fastInverseDST4_B8, fastInverseDST4_B16, fastInverseDST4_B32,
                 fastInverseDST4_B64, fastInverseDST4_B128, fastInverseDST4_B256 },
  InvTransList { nullptr, fastInverseDST1_B4, fastInverseDST1_B8, fastInverseDST1_B16, fastInverseDST1_B32,
                 fastInverseDST1_B64, fastInverseDST1_B128, fastInverseDST1_B256 },
  InvTransList { nullptr, fastInverseIDTR_B4, fastInverseIDTR_B8, fastInverseIDTR_B16, fastInverseIDTR_B32,
                 fastInverseIDTR_B64, fastInverseIDTR_B128, fastInverseIDTR_B256 },
  InvTransList { nullptr, fastInverseKLT0_B4, fastInverseKLT0_B8, fastInverseKLT0_B16, nullptr, nullptr, nullptr,
                 nullptr },
  InvTransList { nullptr, fastInverseKLT1_B4, fastInverseKLT1_B8, fastInverseKLT1_B16, nullptr, nullptr, nullptr,
                 nullptr },
} };

//! \ingroup CommonLib
//! \{

static inline int64_t square(const int d) { return d * (int64_t)d; }

template<int signedMode>
std::pair<int64_t, int64_t> fwdTransformCbCr(const CPelBuf &resCb, const CPelBuf &resCr, PelBuf &resJCCR)
{
  const Pel *cb = resCb.buf;
  const Pel *cr = resCr.buf;
  Pel       *cc = resJCCR.buf;
  int64_t    d1 = 0;
  int64_t    d2 = 0;
  for (SizeType y = 0; y < resCb.height; y++, cb += resCb.stride, cr += resCr.stride, cc += resJCCR.stride)
  {
    for (SizeType x = 0; x < resCb.width; x++)
    {
      int cbx = cb[x], crx = cr[x];
      if (signedMode == 1)
      {
        cc[x] = Pel((4 * cbx + 2 * crx) / 5);
        d1 += square(cbx - cc[x]) + square(crx - (cc[x] >> 1));
      }
      else if (signedMode == -1)
      {
        cc[x] = Pel((4 * cbx - 2 * crx) / 5);
        d1 += square(cbx - cc[x]) + square(crx - (-cc[x] >> 1));
      }
      else if (signedMode == 2)
      {
        cc[x] = Pel((cbx + crx) / 2);
        d1 += square(cbx - cc[x]) + square(crx - cc[x]);
      }
      else if (signedMode == -2)
      {
        cc[x] = Pel((cbx - crx) / 2);
        d1 += square(cbx - cc[x]) + square(crx + cc[x]);
      }
      else if (signedMode == 3)
      {
        cc[x] = Pel((4 * crx + 2 * cbx) / 5);
        d1 += square(cbx - (cc[x] >> 1)) + square(crx - cc[x]);
      }
      else if (signedMode == -3)
      {
        cc[x] = Pel((4 * crx - 2 * cbx) / 5);
        d1 += square(cbx - (-cc[x] >> 1)) + square(crx - cc[x]);
      }
      else
      {
        d1 += square(cbx);
        d2 += square(crx);
      }
    }
  }
  return std::make_pair(d1, d2);
}

template<int signedMode> void invTransformCbCr(PelBuf &resCb, PelBuf &resCr)
{
  Pel *cb = resCb.buf;
  Pel *cr = resCr.buf;
  for (SizeType y = 0; y < resCb.height; y++, cb += resCb.stride, cr += resCr.stride)
  {
    for (SizeType x = 0; x < resCb.width; x++)
    {
      if (signedMode == 1)
      {
        cr[x] = cb[x] >> 1;
      }
      else if (signedMode == -1)
      {
        cr[x] = -cb[x] >> 1;
      }
      else if (signedMode == 2)
      {
        cr[x] = cb[x];
      }
      else if (signedMode == -2)
      {
        // non-normative clipping to prevent 16-bit overflow
        cr[x] = (cb[x] == -32768 && sizeof(Pel) == 2) ? 32767 : -cb[x];
      }
      else if (signedMode == 3)
      {
        cb[x] = cr[x] >> 1;
      }
      else if (signedMode == -3)
      {
        cb[x] = -cr[x] >> 1;
      }
    }
  }
}

void xFwdLfnstNxNCore(TCoeff *src, TCoeff *dst, const uint32_t mode, const uint32_t index, const uint32_t size,
                      int zeroOutSize)
{
  const int8_t *trMat  = (size > 8) ? g_fwdLfnst16x16[mode][index][0]
                                    : ((size > 4) ? g_fwdLfnst8x8[mode][index][0] : g_fwdLfnst4x4[mode][index][0]);
  const int     trSize = (size > 8) ? L16W_ZO : ((size > 4) ? L8W_ZO : 16);
  TCoeff        coef;
  TCoeff       *out = dst;
  assert(index < 4);

  for (int j = 0; j < zeroOutSize; j++)
  {
    TCoeff       *srcPtr   = src;
    const int8_t *trMatTmp = trMat;
    coef                   = 0;
    for (int i = 0; i < trSize; i++)
    {
      coef += *srcPtr++ * *trMatTmp++;
    }
    *out++ = (coef + 64) >> 7;
    trMat += trSize;
  }

  std::fill_n(out, trSize - zeroOutSize, 0);
}

void xInvLfnstNxNCore(TCoeff *src, TCoeff *dst, const uint32_t mode, const uint32_t index, const uint32_t size,
                      int zeroOutSize, const int maxLog2TrDynamicRange)
{
  const TCoeff  outputMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff  outputMaximum = (1 << maxLog2TrDynamicRange) - 1;
  const int8_t *trMat         = (size > 8) ? g_invLfnst16x16[mode][index][0]
                                           : ((size > 4) ? g_invLfnst8x8[mode][index][0] : g_invLfnst4x4[mode][index][0]);
  const int     trSize        = (size > 8) ? L16W_ZO : ((size > 4) ? L8W_ZO : 16);
  const int     trMatInc      = (size > 8) ? L16H_ZO : ((size > 4) ? L8H_ZO : 16);
  int           resi;
  TCoeff       *out = dst;

  assert(index < 4);

  for (int j = 0; j < trSize; j++, trMat += trMatInc)
  {
    resi                   = 0;
    const int8_t *trMatTmp = trMat;
    TCoeff       *srcPtr   = src;

    for (int i = 0; i < zeroOutSize; i++)
    {
      resi += *srcPtr++ * *trMatTmp++;
    }

    *out++ = Clip3(outputMinimum, outputMaximum, (TCoeff)(resi + 64) >> 7);
  }
}

void xFwdNsptNxNCore(TCoeff *src, TCoeff *dst, const uint32_t mode, const uint32_t width, const uint32_t height,
                     const int _shift, int zeroOutSize, int nsptIdx, int setIdx)
{
  const int8_t *trMat = TrQuant::getNsptMatrix(mode, width, height, nsptIdx, setIdx);

  const int trSize = width * height;
  const int shift  = _shift - 7;
  const int rnd    = 1 << (shift - 1);
  TCoeff   *out    = dst;

  for (int j = 0; j < zeroOutSize; j++)
  {
    const int8_t *trMatTmp = trMat;
    TCoeff       *srcPtr   = src;
    TCoeff        coef     = 0;
    for (int i = 0; i < trSize; i++)
    {
      coef += *srcPtr++ * *trMatTmp++;
    }
    *out++ = (coef + rnd) >> shift;
    trMat += trSize;
  }
  std::fill_n(out, trSize - zeroOutSize, 0);
}

void xInvNsptNxNCore(TCoeff *src, TCoeff *dst, const uint32_t mode, const uint32_t width, const uint32_t height,
                     const int _shift, int zeroOutSize, int nsptIdx, int setIdx, const int maxLog2TrDynamicRange)
{
  const TCoeff outputMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff outputMaximum = (1 << maxLog2TrDynamicRange) - 1;

  const int8_t *trMat = TrQuant::getNsptMatrix(mode, width, height, nsptIdx, setIdx);

  const int trSize = width * height;
  const int shift  = _shift - 7;
  const int rnd    = 1 << (shift - 1);
  TCoeff   *out    = dst;

  for (int j = 0; j < trSize; j++)
  {
    const int8_t *trMatTmp = trMat;
    TCoeff       *srcPtr   = src;
    TCoeff        resi     = 0;

    for (int i = 0; i < zeroOutSize; i++)
    {
      resi += *srcPtr++ * *trMatTmp;
      trMatTmp += trSize;
    }

    *out++ = Clip3<TCoeff>(outputMinimum, outputMaximum, (resi + rnd) >> shift);
    trMat++;
  }
}

// ====================================================================================================================
// TrQuant class member functions
// ====================================================================================================================
TrQuant::TrQuant() : m_quant(nullptr)
{
  // allocate temporary buffers
  m_tempCoeff = (TCoeff *)xMalloc(TCoeff, MAX_TB_SIZEY * MAX_TB_SIZEY);
  m_tmp       = (TCoeff *)xMalloc(TCoeff, MAX_TB_SIZEY * MAX_TB_SIZEY);
  m_blk       = (TCoeff *)xMalloc(TCoeff, MAX_TB_SIZEY * MAX_TB_SIZEY);
  m_tempResi  = (Pel *)xMalloc(Pel, SIGN_PRED_MAX_BS * SIGN_PRED_MAX_BS);

  // allocate temporary buffers
  {
    m_invICT     = m_invICTMem + maxAbsIctMode;
    m_invICT[0]  = invTransformCbCr<0>;
    m_invICT[1]  = invTransformCbCr<1>;
    m_invICT[-1] = invTransformCbCr<-1>;
    m_invICT[2]  = invTransformCbCr<2>;
    m_invICT[-2] = invTransformCbCr<-2>;
    m_invICT[3]  = invTransformCbCr<3>;
    m_invICT[-3] = invTransformCbCr<-3>;
    m_fwdICT     = m_fwdICTMem + maxAbsIctMode;
    m_fwdICT[0]  = fwdTransformCbCr<0>;
    m_fwdICT[1]  = fwdTransformCbCr<1>;
    m_fwdICT[-1] = fwdTransformCbCr<-1>;
    m_fwdICT[2]  = fwdTransformCbCr<2>;
    m_fwdICT[-2] = fwdTransformCbCr<-2>;
    m_fwdICT[3]  = fwdTransformCbCr<3>;
    m_fwdICT[-3] = fwdTransformCbCr<-3>;
  }

  m_invLfnstNxN = xInvLfnstNxNCore;
  m_fwdLfnstNxN = xFwdLfnstNxNCore;

  m_invNsptNxN = xInvNsptNxNCore;
  m_fwdNsptNxN = xFwdNsptNxNCore;

#if defined(TARGET_SIMD_X86) && ENABLE_SIMD_TRAFO
  initTrQuantX86();
#endif
}

TrQuant::~TrQuant()
{
  if (m_quant)
  {
    delete m_quant;
    m_quant = nullptr;
  }

  // delete temporary buffers
  if (m_tempCoeff)
  {
    xFree(m_tempCoeff);
    m_tempCoeff = nullptr;
  }

  if (m_blk)
  {
    xFree(m_blk);
    m_blk = nullptr;
  }

  if (m_tmp)
  {
    xFree(m_tmp);
    m_tmp = nullptr;
  }

  if (m_tempResi)
  {
    xFree(m_tempResi);
    m_tempResi = nullptr;
  }
}

void TrQuant::xDeQuant(const TransformUnit &tu, CoeffBuf &dstCoeff, const CompID &compID, const QpParam &cQP)
{
  PROFILER_SCOPE(TP_ENABLE_TRQUANT_STAGES, g_timeProfiler, P_QUANT);
  m_quant->dequant(tu, dstCoeff, compID, cQP);
}

void TrQuant::init(const Quant *otherQuant, const uint32_t uiMaxTrSize, const bool bUseRDOQ, const bool bUseRDOQTS,
                   const bool useSelectiveRDOQ, const bool bEnc)
{
  delete m_quant;
  m_quant = nullptr;

  m_quant = new DepQuant(otherQuant, bEnc);

  if (m_quant)
  {
    m_quant->init(uiMaxTrSize, bUseRDOQ, bUseRDOQTS, useSelectiveRDOQ);
  }
}

bool TrQuant::getTransposeFlag(uint32_t intraMode)
{
  return ((intraMode >= NUM_LUMA_MODE) && (intraMode >= (NUM_LUMA_MODE + (NUM_EXT_LUMA_MODE >> 1)))) ||
    ((intraMode < NUM_LUMA_MODE) && (intraMode > DIA_IDX));
}

const int8_t *TrQuant::getNsptMatrix(const uint32_t mode, const uint32_t width, const uint32_t height, int nsptIdx,
                                     int setIdx)
{
  const int8_t *trMat = nullptr;
  if (width == 4 && height == 4)
  {
    trMat = g_nspt4x4[g_nsptIdx4x4[mode][setIdx][nsptIdx]][0];
  }
  else if (width == 8 && height == 8)
  {
    trMat = g_nspt8x8[g_nsptIdx8x8[mode][setIdx][nsptIdx]][0];
  }
  else if (width == 4 && height == 8)
  {
    trMat = g_nspt4x8[g_nsptIdx4x8[mode][setIdx][nsptIdx]][0];
  }
  else if (width == 8 && height == 4)
  {
    trMat = g_nspt8x4[g_nsptIdx8x4[mode][setIdx][nsptIdx]][0];
  }
  else if (width == 4 && height == 16)
  {
    trMat = g_nspt4x16[g_nsptIdx4x16[mode][setIdx][nsptIdx]][0];
  }
  else if (width == 16 && height == 4)
  {
    trMat = g_nspt16x4[g_nsptIdx16x4[mode][setIdx][nsptIdx]][0];
  }
  else if (width == 8 && height == 16)
  {
    trMat = g_nspt8x16[g_nsptIdx8x16[mode][setIdx][nsptIdx]][0];
  }
  else if (width == 16 && height == 8)
  {
    trMat = g_nspt16x8[g_nsptIdx16x8[mode][setIdx][nsptIdx]][0];
  }
  else if (width == 4 && height == 32)
  {
    trMat = g_nspt4x32[g_nsptIdx4x32[mode][setIdx][nsptIdx]][0];
  }
  else if (width == 32 && height == 4)
  {
    trMat = g_nspt32x4[g_nsptIdx32x4[mode][setIdx][nsptIdx]][0];
  }
  else if (width == 8 && height == 32)
  {
    trMat = g_nspt8x32[g_nsptIdx8x32[mode][setIdx][nsptIdx]][0];
  }
  else if (width == 32 && height == 8)
  {
    trMat = g_nspt32x8[g_nsptIdx32x8[mode][setIdx][nsptIdx]][0];
  }
  return trMat;
}

void TrQuant::xInvLfnst(const TransformUnit &tu, const CompID compID)
{
  const int       maxLog2TrDynamicRange = tu.cs->sps->getMaxLog2TrDynamicRange(toChannelType(compID));
  const CompArea &area                  = tu.blocks[compID];
  const uint32_t  width                 = area.width;
  const uint32_t  height                = area.height;
  const uint32_t  lfnstIdx              = TU::getNstIdx(tu, compID);
  if (lfnstIdx && tu.mtsIdx[compID] != MtsType::SKIP && (CS::isDualITree(*tu.cs) ? true : isLuma(compID)))
  {
    if (TU::isNSPTAllowed(width, height))
    {
      return;
    }
    const bool         whge4     = PU::getUseLFNST16(width, height);
    const bool         whge3     = PU::getUseLFNST8(width, height);
    const int          widthIdx  = gp_sizeIdxInfo->idxFrom(width);
    const int          heightIdx = gp_sizeIdxInfo->idxFrom(height);
    const ScanElement *scan      = whge4 ? g_coefTopLeftDiagScan16x16[widthIdx]
                                         : (whge3 ? g_coefTopLeftDiagScan8x8[widthIdx]
                                                  : g_scanOrder[SCAN_GROUPED_4x4][CoeffScanType::DIAG][widthIdx][heightIdx]);
    const uint32_t     intraMode = PU::getFinalIntraModesTrafo(tu, compID).first;
    if (lfnstIdx < 4)
    {
#if RExt__DECODER_DEBUG_TOOL_STATISTICS
      CodingStatistics::IncrementStatisticTool(CodingStatisticsClassType { STATS__TOOL_LFNST, width, height, compID });
#endif
      bool      transposeFlag = getTransposeFlag(intraMode);
      const int sbSize        = whge4 ? 16 : (whge3 ? 8 : 4);
      TCoeff   *lfnstTemp;
      TCoeff   *coeffTemp;
      int       y;
      lfnstTemp   = m_tempInMatrix;   // inverse spectral rearrangement
      coeffTemp   = m_tempCoeff;
      TCoeff *dst = lfnstTemp;

      const ScanElement *scanPtr       = scan;
      int                numLfnstCoeff = PU::getLFNSTMatrixDim(width, height);
      for (y = 0; y < numLfnstCoeff; y++)
      {
        *dst++ = coeffTemp[scanPtr->idx];
        scanPtr++;
      }

      m_invLfnstNxN(m_tempInMatrix, m_tempOutMatrix, g_lfnstLut[intraMode], lfnstIdx - 1, sbSize, numLfnstCoeff,
                    maxLog2TrDynamicRange);
      lfnstTemp = m_tempOutMatrix;   // inverse spectral rearrangement

      if (transposeFlag)
      {
        if (sbSize == 4)
        {
          for (y = 0; y < 4; y++)
          {
            coeffTemp[0] = lfnstTemp[0];
            coeffTemp[1] = lfnstTemp[4];
            coeffTemp[2] = lfnstTemp[8];
            coeffTemp[3] = lfnstTemp[12];
            lfnstTemp++;
            coeffTemp += width;
          }
        }
        else if (sbSize == 8)
        {
          for (y = 0; y < 8; y++)
          {
            coeffTemp[0] = lfnstTemp[0];
            coeffTemp[1] = lfnstTemp[8];
            coeffTemp[2] = lfnstTemp[16];
            coeffTemp[3] = lfnstTemp[24];
            coeffTemp[4] = lfnstTemp[32];
            coeffTemp[5] = lfnstTemp[40];
            coeffTemp[6] = lfnstTemp[48];
            coeffTemp[7] = lfnstTemp[56];
            lfnstTemp++;
            coeffTemp += width;
          }
        }
        else // (sbSize == 16)
        {
          for (y = 0; y < 12; y++)
          {
            coeffTemp[0] = lfnstTemp[0];
            coeffTemp[1] = lfnstTemp[12];
            coeffTemp[2] = lfnstTemp[24];
            coeffTemp[3] = lfnstTemp[36];

            if (y < 8)
            {
              coeffTemp[4] = lfnstTemp[48];
              coeffTemp[5] = lfnstTemp[56];
              coeffTemp[6] = lfnstTemp[64];
              coeffTemp[7] = lfnstTemp[72];
            }

            if (y < 4)
            {
              coeffTemp[8]  = lfnstTemp[80];
              coeffTemp[9]  = lfnstTemp[84];
              coeffTemp[10] = lfnstTemp[88];
              coeffTemp[11] = lfnstTemp[92];
            }
            lfnstTemp++;
            coeffTemp += width;
          }
        }
      }
      else
      {
        if (sbSize == 16)
        {
          for (y = 0; y < 12; y++)
          {
            uint32_t uiStride = (y < 4) ? 12 : ((y < 8) ? 8 : 4);
            ::memcpy(coeffTemp, lfnstTemp, uiStride * sizeof(TCoeff));
            lfnstTemp += uiStride;
            coeffTemp += width;
          }
        }
        else
        {
          for (y = 0; y < sbSize; y++)
          {
            uint32_t uiStride = sbSize;
            ::memcpy(coeffTemp, lfnstTemp, uiStride * sizeof(TCoeff));
            lfnstTemp += uiStride;
            coeffTemp += width;
          }
        }
      }
    }
  }
}

void TrQuant::xFwdLfnst(const TransformUnit &tu, const CompID compID, TCoeff *buf)
{
  const CompArea &area     = tu.blocks[compID];
  const uint32_t  width    = area.width;
  const uint32_t  height   = area.height;
  const uint32_t  lfnstIdx = TU::getNstIdx(tu, compID);
  if (lfnstIdx && tu.mtsIdx[compID] != MtsType::SKIP && (CS::isDualITree(*tu.cs) ? true : isLuma(compID)))
  {
    if (TU::isNSPTAllowed(width, height))
    {
      return;
    }
    const bool         whge4     = PU::getUseLFNST16(width, height);  // width >= 16 && height >= 16;
    const bool         whge3     = PU::getUseLFNST8(width, height);
    const int          widthIdx  = gp_sizeIdxInfo->idxFrom(width);
    const int          heightIdx = gp_sizeIdxInfo->idxFrom(height);
    const ScanElement *scan      = whge4 ? g_coefTopLeftDiagScan16x16[widthIdx]
                                         : (whge3 ? g_coefTopLeftDiagScan8x8[widthIdx]
                                                  : g_scanOrder[SCAN_GROUPED_4x4][CoeffScanType::DIAG][widthIdx][heightIdx]);
    const uint32_t     intraMode = PU::getFinalIntraModesTrafo(tu, compID).first;
    if (lfnstIdx < 4)
    {
      bool      transposeFlag = getTransposeFlag(intraMode);
      const int sbSize        = whge4 ? 16 : (whge3 ? 8 : 4);
      TCoeff   *lfnstTemp;
      TCoeff   *coeffTemp;
      TCoeff   *tempCoeff = buf == nullptr ? m_tempCoeff : buf;

      int y;
      lfnstTemp = m_tempInMatrix;   // forward low frequency non-separable transform
      coeffTemp = tempCoeff;

      if (transposeFlag)
      {
        if (sbSize == 4)
        {
          for (y = 0; y < 4; y++)
          {
            lfnstTemp[0]  = coeffTemp[0];
            lfnstTemp[4]  = coeffTemp[1];
            lfnstTemp[8]  = coeffTemp[2];
            lfnstTemp[12] = coeffTemp[3];
            lfnstTemp++;
            coeffTemp += width;
          }
        }
        else if (sbSize == 8)
        {
          for (y = 0; y < 8; y++)
          {
            lfnstTemp[0]  = coeffTemp[0];
            lfnstTemp[8]  = coeffTemp[1];
            lfnstTemp[16] = coeffTemp[2];
            lfnstTemp[24] = coeffTemp[3];
            lfnstTemp[32] = coeffTemp[4];
            lfnstTemp[40] = coeffTemp[5];
            lfnstTemp[48] = coeffTemp[6];
            lfnstTemp[56] = coeffTemp[7];
            lfnstTemp++;
            coeffTemp += width;
          }
        }
        else // (sbSize == 16)
        {
          for (y = 0; y < 12; y++)
          {
            lfnstTemp[0]  = coeffTemp[0];
            lfnstTemp[12] = coeffTemp[1];
            lfnstTemp[24] = coeffTemp[2];
            lfnstTemp[36] = coeffTemp[3];

            if (y < 8)
            {
              lfnstTemp[48] = coeffTemp[4];
              lfnstTemp[56] = coeffTemp[5];
              lfnstTemp[64] = coeffTemp[6];
              lfnstTemp[72] = coeffTemp[7];
            }

            if (y < 4)
            {
              lfnstTemp[80] = coeffTemp[8];
              lfnstTemp[84] = coeffTemp[9];
              lfnstTemp[88] = coeffTemp[10];
              lfnstTemp[92] = coeffTemp[11];
            }
            lfnstTemp++;
            coeffTemp += width;
          }
        }
      }
      else
      {
        if (sbSize == 16)
        {
          for (y = 0; y < 16; y++)
          {
            uint32_t uiStride = (y < 4) ? 12 : ((y < 8) ? 8 : 4);
            ::memcpy(lfnstTemp, coeffTemp, uiStride * sizeof(TCoeff));
            lfnstTemp += uiStride;
            coeffTemp += width;
          }
        }
        else
        {
          for (y = 0; y < sbSize; y++)
          {
            uint32_t uiStride = sbSize;
            ::memcpy(lfnstTemp, coeffTemp, uiStride * sizeof(TCoeff));
            lfnstTemp += uiStride;
            coeffTemp += width;
          }
        }
      }

      int lfnstCoeffNum = PU::getLFNSTMatrixDim(width, height);
      m_fwdLfnstNxN(m_tempInMatrix, m_tempOutMatrix, g_lfnstLut[intraMode], lfnstIdx - 1, sbSize, lfnstCoeffNum);

      lfnstTemp = m_tempOutMatrix;   // forward spectral rearrangement
      coeffTemp = tempCoeff;

      const ScanElement *scanPtr = scan;

      ::memset(coeffTemp, 0, sizeof(TCoeff) * width * height); // required for simple calculation of l1 norm
      for (y = 0; y < lfnstCoeffNum; y++)
      {
        coeffTemp[scanPtr->idx] = *lfnstTemp++;
        scanPtr++;
      }
    }
  }
}

void TrQuant::xFwdNspt(const TransformUnit &tu, TCoeff *src, TCoeff *dst, const CompID compID, const int shift,
                       int lfnstIdx)
{
  const CompArea    &area   = tu.blocks[compID];
  const uint32_t     width  = area.width;
  const uint32_t     height = area.height;
  const ScanElement *scan =
    g_scanOrder[SCAN_GROUPED_4x4][CoeffScanType::DIAG][gp_sizeIdxInfo->idxFrom(width)][gp_sizeIdxInfo->idxFrom(height)];
  const uint32_t intraMode     = PU::getFinalIntraModesTrafo(tu, compID).first;
  const int      nsptBucket    = PU::getNSPTBucket(tu);
  const bool     transposeFlag = getTransposeFlag(intraMode);
  TCoeff        *nsptIn        = m_nsptTempInMatrix;
  TCoeff        *nsptOut       = m_nsptTempOutMatrix;
  const int      nsptIdx       = lfnstIdx - 1;
  const uint8_t  nsptSetIdx    = g_nsptLut[intraMode];
  const int      nsptCoeffNum  = PU::getNSPTMatrixDim(transposeFlag ? height : width, transposeFlag ? width : height);

  if (transposeFlag)
  {
    TCoeff *srcTemp = src;
    for (int y = 0; y < height; y++)
    {
      TCoeff *nsptInTemp = nsptIn++;
      for (int x = 0; x < width; x++)
      {
        *nsptInTemp = *srcTemp++;
        nsptInTemp += height;
      }
    }
  }
  else // transposeFlag = 0
  {
    ::memcpy(nsptIn, src, width * height * sizeof(TCoeff));
  }

  m_fwdNsptNxN(m_nsptTempInMatrix, m_nsptTempOutMatrix, nsptSetIdx, transposeFlag ? height : width,
               transposeFlag ? width : height, shift, nsptCoeffNum, nsptIdx, nsptBucket);

  ::memset(dst, 0, sizeof(TCoeff) * width * height); // required for simple calculation of l1 norm
  const ScanElement *scanPtr = scan;
  for (int y = 0; y < nsptCoeffNum; y++)
  {
    dst[scanPtr->idx] = *nsptOut++;
    scanPtr++;
  }
}

void TrQuant::xInvNspt(const TransformUnit &tu, const TCoeff *src, TCoeff *dst, const CompID compID, const int shift,
                       int lfnstIdx)
{
  const int          maxLog2TrDynamicRange = tu.cs->sps->getMaxLog2TrDynamicRange(toChannelType(compID));
  const CompArea    &area                  = tu.blocks[compID];
  const uint32_t     width                 = area.width;
  const uint32_t     height                = area.height;
  const ScanElement *scan =
    g_scanOrder[SCAN_GROUPED_4x4][CoeffScanType::DIAG][gp_sizeIdxInfo->idxFrom(width)][gp_sizeIdxInfo->idxFrom(height)];
  const uint32_t intraMode     = PU::getFinalIntraModesTrafo(tu, compID).first;
  const int      nsptBucket    = PU::getNSPTBucket(tu);
  const bool     transposeFlag = getTransposeFlag(intraMode);
  TCoeff        *invNsptIn     = m_nsptTempInMatrix;
  TCoeff        *nsptOut       = m_nsptTempOutMatrix;
  const int      nsptCoeffNum  = PU::getNSPTMatrixDim(transposeFlag ? height : width, transposeFlag ? width : height);
  const int      nsptIdx       = lfnstIdx - 1;
  const uint8_t  nsptSetIdx    = g_nsptLut[intraMode];

#if RExt__DECODER_DEBUG_TOOL_STATISTICS
  CodingStatistics::IncrementStatisticTool(CodingStatisticsClassType { STATS__TOOL_LFNST, width, height, compID });
#endif

  const ScanElement *scanPtr = scan;
  for (int i = 0; i < nsptCoeffNum; i++)
  {
    *invNsptIn++ = src[scanPtr->idx];
    scanPtr++;
  }

  m_invNsptNxN(m_nsptTempInMatrix, m_nsptTempOutMatrix, nsptSetIdx, transposeFlag ? height : width,
               transposeFlag ? width : height, shift, nsptCoeffNum, nsptIdx, nsptBucket, maxLog2TrDynamicRange);

  if (transposeFlag)
  {
    TCoeff *dstTemp = dst;
    for (int y = 0; y < height; y++)
    {
      TCoeff *nsptOutTemp = nsptOut++;
      for (int x = 0; x < width; x++)
      {
        *dstTemp++ = *nsptOutTemp;
        nsptOutTemp += height;
      }
    }
  }
  else
  {
    ::memcpy(dst, nsptOut, width * height * sizeof(TCoeff));
  }
}

void TrQuant::invTransformNxN(TransformUnit &tu, const CompID &compID, PelBuf &pResi, const QpParam &cQP,
                              bool skipDequant)
{
  PROFILER_SCOPE(0, g_timeProfiler, P_TRAFO_QUANT);
  const CompArea &area     = tu.blocks[compID];
  const uint32_t  uiWidth  = area.width;
  const uint32_t  uiHeight = area.height;

  CHECK(uiWidth > tu.cs->sps->getMaxTbSize() || uiHeight > tu.cs->sps->getMaxTbSize(),
        "Maximal allowed transformation size exceeded!");
  CoeffBuf tempCoeff = CoeffBuf(m_tempCoeff, area);
  if (!skipDequant)
  {
    xDeQuant(tu, tempCoeff, compID, cQP);
  }

  DTRACE_COEFF_BUF(D_TCOEFF, tempCoeff, tu, tu.cu->predMode, compID);

  if (tu.mtsIdx[compID] == MtsType::SKIP)
  {
    xITransformSkip(tempCoeff, pResi, tu, compID);
  }
  else
  {
    if (TU::getNstIdx(tu, compID))
    {
      xInvLfnst(tu, compID);
    }
    xIT(tu, compID, tempCoeff, pResi);
  }

  // DTRACE_BLOCK_COEFF(tu.getCoeffs(compID), tu, tu.cu->predMode, compID);
  DTRACE_PEL_BUF(D_RESIDUALS, pResi, tu, tu.cu->predMode, compID);
}

std::pair<int64_t, int64_t> TrQuant::fwdTransformICT(const TransformUnit &tu, const PelBuf **resCbCr, PelBuf &resJCCR,
                                                     int jointCbCr)
{
  CHECK(Size(*resCbCr[0]) != Size(*resCbCr[1]), "resCb and resCr have different sizes");
  CHECK(Size(*resCbCr[0]) != Size(resJCCR), "resCb and resJCCR have different sizes");
  return (*m_fwdICT[TU::getICTMode(tu, jointCbCr)])(*resCbCr[0], *resCbCr[1], resJCCR);
}

void TrQuant::invTransformICT(const TransformUnit &tu, PelBuf &resCb, PelBuf &resCr)
{
  CHECK(Size(resCb) != Size(resCr), "resCb and resCr have different sizes");
  (*m_invICT[TU::getICTMode(tu)])(resCb, resCr);
}

CbfMaskList TrQuant::selectICTCandidates(const TransformUnit &tu, const PelBuf **resOrg, PelBuf **resJCCR)
{
  CbfMaskList cbfMasksToTest;

  if (!CU::isIntra(*tu.cu))
  {
    int cbfMask = CBF_MASK_CBCR;
    fwdTransformICT(tu, resOrg, *resJCCR[cbfMask - 1], cbfMask);
    cbfMasksToTest.push_back(cbfMask);
    return cbfMasksToTest;
  }

  std::pair<int64_t, int64_t> pairDist[4];
  for (int cbfMask = 0; cbfMask < 4; cbfMask++)
  {
    pairDist[cbfMask] = fwdTransformICT(tu, resOrg, *resJCCR[std::max<int>(0, cbfMask - 1)], cbfMask);
  }

  int64_t minDist1 = std::min<int64_t>(pairDist[0].first, pairDist[0].second);
  int64_t minDist2 = std::numeric_limits<int64_t>::max();
  int     cbfMask1 = 0;
  int     cbfMask2 = 0;
  for (int cbfMask: { CBF_MASK_CB, CBF_MASK_CR, CBF_MASK_CBCR })
  {
    if (pairDist[cbfMask].first < minDist1)
    {
      cbfMask2 = cbfMask1;
      minDist2 = minDist1;
      cbfMask1 = cbfMask;
      minDist1 = pairDist[cbfMask1].first;
    }
    else if (pairDist[cbfMask].first < minDist2)
    {
      cbfMask2 = cbfMask;
      minDist2 = pairDist[cbfMask2].first;
    }
  }
  if (cbfMask1)
  {
    cbfMasksToTest.push_back(cbfMask1);
  }
  if (cbfMask2 && ((minDist2 < (9 * minDist1) / 8) || (!cbfMask1 && minDist2 < (3 * minDist1) / 2)))
  {
    cbfMasksToTest.push_back(cbfMask2);
  }
  return cbfMasksToTest;
}

// ------------------------------------------------------------------------------------------------
// Logical transform
// ------------------------------------------------------------------------------------------------

void TrQuant::getTrTypes(const TransformUnit &tu, const CompID compID, TransType &trTypeHor, TransType &trTypeVer)
{
  const bool isExplicitMTS =
    (CU::isIntra(*tu.cu) ? tu.cs->sps->m_explicitMtsIntra : tu.cs->sps->m_explicitMtsInter && CU::isInter(*tu.cu)) &&
    isLuma(compID);
  const bool isImplicitMTS = CU::isIntra(*tu.cu) && tu.cs->sps->m_mtsEnabled && !tu.cs->sps->m_explicitMtsIntra &&
    isLuma(compID) && !TU::getNstIdx(tu, compID);
  const bool isSBT = CU::isInter(*tu.cu) && tu.cu->sbtInfo && isLuma(compID);

  trTypeHor = TransType::DCT2;
  trTypeVer = TransType::DCT2;

  if (isSBT && isNST(tu.mtsIdx[compID]))
  {
    return;
  }
  if (!tu.cs->sps->m_mtsEnabled)
  {
    return;
  }

  if (isImplicitMTS)
  {
    const int width  = tu.blocks[compID].width;
    const int height = tu.blocks[compID].height;

    if (width < 4 || height < 4)
    {
      const bool widthDstOk  = (width >= 4 && width <= 16);
      const bool heightDstOk = (height >= 4 && height <= 16);
      if (widthDstOk)
      {
        trTypeHor = TransType::DST7;
      }
      if (heightDstOk)
      {
        trTypeVer = TransType::DST7;
      }
      return;
    }

    const bool useDerivedIntraModes =
      tu.cu->dimdFlag || tu.cu->timdFlag || tu.cu->obicFlag || tu.cu->mipFlag || tu.cu->sgpm || tu.cu->eipFlag;
    if (!useDerivedIntraModes)
    {
      int predMode = PU::getWideAngle(tu, PU::getFinalIntraMode(*tu.cu, toChannelType(compID)), compID);
      CHECK(predMode < -(NUM_EXT_LUMA_MODE >> 1) || predMode >= NUM_LUMA_MODE + (NUM_EXT_LUMA_MODE >> 1),
            "luma mode out of range");
      if (predMode == PLANAR_IDX)
      {
        if (tu.cu->plDir == PlanarDirType::VER)
        {
          predMode = VER_IDX;
        }
        else if (tu.cu->plDir == PlanarDirType::HOR)
        {
          predMode = HOR_IDX;
        }
      }
      const int     modeImplicit   = predMode < 0 ? predMode + NUM_LUMA_MODE
              : predMode >= NUM_LUMA_MODE         ? predMode - NUM_LUMA_MODE + 2
                                                  : predMode;
      const int     modeIdx        = modeImplicit > DIA_IDX ? (NUM_LUMA_MODE + 1 - modeImplicit) : modeImplicit;
      const bool    isTrTransposed = modeImplicit > DIA_IDX;
      const uint8_t nSzIdxW        = std::min(3, (floorLog2(width) - 2));
      const uint8_t nSzIdxH        = std::min(3, (floorLog2(height) - 2));
      const uint8_t nSzIdx         = isTrTransposed ? (nSzIdxH * 4 + nSzIdxW) : (nSzIdxW * 4 + nSzIdxH);
      const uint8_t nTrType        = g_aucImplicitToTrSet[nSzIdx][modeIdx];
      trTypeHor                    = g_aucImplicitTrIdxToTr[nTrType][isTrTransposed ? 1 : 0];
      trTypeVer                    = g_aucImplicitTrIdxToTr[nTrType][isTrTransposed ? 0 : 1];
      return;
    }

    const auto    predModes       = PU::getFinalIntraModesTrafo(tu, compID);
    const int     idxMap[18]      = { 0, 1, 2, 3, -1, 4, 5, 6, -1, -1, 7, 8, -1, -1, -1, 9, 10, 11 };
    const bool    blockSym        = (height > width);
    const bool    predModeSym     = blockSym && int(predModes.first) > 1;
    const int     log2BlockWidth  = floorLog2(blockSym ? height : width) - 2;
    const int     log2BlockHeight = floorLog2(blockSym ? width : height) - 2;
    const int     blIndSize    = idxMap[log2BlockWidth < 4 && log2BlockHeight < 4 ? log2BlockHeight * 4 + log2BlockWidth
                                                                                  : 16 + log2BlockWidth - 4];
    const int     absPModeDiff = abs(int(predModes.first) - int(predModes.second));
    const int     diffClass    = (absPModeDiff <= 8 ? 0 : absPModeDiff <= 16 ? 1 : 2);
    const int     pmodeIndex   = (predModeSym ? 34 * 2 - int(predModes.first) : int(predModes.first));
    const bool    useDimdLikeTrSet = tu.cu->dimdFlag || tu.cu->obicFlag || tu.cu->eipFlag;
    const uint8_t trIndex =
      (useDimdLikeTrSet    ? g_aucIpmToTrSetModDimd[diffClass][std::min(11, blIndSize)][pmodeIndex]
         : tu.cu->timdFlag ? g_aucIpmToTrSetModTimd[diffClass][std::min(9, blIndSize)][pmodeIndex]
         : tu.cu->mipFlag  ? g_aucIpmToTrSetModMip[diffClass][std::min(11, blIndSize)][pmodeIndex]
         : tu.cu->sgpm     ? g_aucIpmToTrSetModSgpm[diffClass][std::min(10, blIndSize)][pmodeIndex]
                           : 0);
    const uint8_t trFirst  = trIndex / uint8_t { 6 };
    const uint8_t trSecond = trIndex - uint8_t { 6 } * trFirst;
    trTypeVer              = TransType(blockSym ? trFirst : trSecond);
    trTypeHor              = TransType(blockSym ? trSecond : trFirst);
    return;
  }

  if (isSBT)
  {
    uint8_t sbtIdx = tu.cu->getSbtIdx();
    uint8_t sbtPos = tu.cu->getSbtPos();

    if (sbtIdx == SBT_QUAD || sbtIdx == SBT_QUARTER)
    {
      if (tu.lwidth() > MTS_INTER_MAX_CU_SIZE || tu.lheight() > MTS_INTER_MAX_CU_SIZE)
      {
        trTypeHor = trTypeVer = TransType::DCT2;
      }
      else if (sbtPos == 0)
      {
        trTypeHor = TransType::DCT8;
        trTypeVer = TransType::DCT8;
      }
      else if (sbtPos == 1)
      {
        trTypeHor = TransType::DST7;
        trTypeVer = TransType::DCT8;
      }
      else if (sbtPos == 2)
      {
        trTypeHor = TransType::DCT8;
        trTypeVer = TransType::DST7;
      }
      else if (sbtPos == 3)
      {
        trTypeHor = TransType::DST7;
        trTypeVer = TransType::DST7;
      }
      else
      {
        CHECK(true, "Wrong SBT QUAD position");
      }
    }
    else if (sbtIdx == SBT_VER_HALF || sbtIdx == SBT_VER_QUAD)
    {
      assert(tu.lwidth() <= MTS_INTER_MAX_CU_SIZE);
      if (tu.lheight() > MTS_INTER_MAX_CU_SIZE)
      {
        trTypeHor = trTypeVer = TransType::DCT2;
      }
      else
      {
        if (sbtPos == SBT_POS0)
        {
          trTypeHor = TransType::DCT8;
          trTypeVer = TransType::DST7;
        }
        else
        {
          trTypeHor = TransType::DST7;
          trTypeVer = TransType::DST7;
        }
      }
    }
    else
    {
      assert(tu.lheight() <= MTS_INTER_MAX_CU_SIZE);
      if (tu.lwidth() > MTS_INTER_MAX_CU_SIZE)
      {
        trTypeHor = trTypeVer = TransType::DCT2;
      }
      else
      {
        if (sbtPos == SBT_POS0)
        {
          trTypeHor = TransType::DST7;
          trTypeVer = TransType::DCT8;
        }
        else
        {
          trTypeHor = TransType::DST7;
          trTypeVer = TransType::DST7;
        }
      }
    }
    return;
  }

  if (isExplicitMTS)
  {
    if (isMTS(tu.mtsIdx[compID]) && CU::isIntra(*tu.cu))
    {
      CHECK(compID != CompID::COMP_Y, "MTS activated for chroma");
      const uint32_t width  = tu.blocks[compID].width;
      const uint32_t height = tu.blocks[compID].height;
      const int      trIdx  = int(tu.mtsIdx[compID] - MtsType::MTS_1);
      CHECK(width < 4 || height < 4, "width < 4 || height < 4 for MTS");
      const uint8_t     nSzIdxW  = std::min(3, (floorLog2(width) - 2));
      const uint8_t     nSzIdxH  = std::min(3, (floorLog2(height) - 2));
      const CodingUnit &cu       = *tu.cu;
      int               predMode = PU::getFinalIntraMode(cu, toChannelType(compID));
      int               ucMode;
      int               nMdIdx;
      bool              isTrTransposed = false;
      if (tu.cu->sgpm)
      {
        predMode = g_geoAngle2IntraAng[g_geoParams[tu.cu->sgpmSplitDir].angleIdx];
      }
      if (tu.cu->eipFlag && compID == COMP_Y)
      {
        predMode = tu.cu->inferredDimdMode;
      }
      if (tu.cu->mipFlag) // MIP is treated as planar.
      {
        ucMode         = 0;
        nMdIdx         = 35;
        isTrTransposed = cu.mipTransposedFlag;
      }
      else
      {
        uint32_t uipredMode = static_cast<uint32_t>(predMode);
        PU::trafoIntraDirPlanar(cu, toChannelType(compID), uipredMode);
        predMode = static_cast<int>(uipredMode);
        ucMode   = predMode;
        predMode = PU::getWideAngle(tu, (uint32_t)predMode, compID);
        CHECK(predMode < -(NUM_EXT_LUMA_MODE >> 1) || predMode >= NUM_LUMA_MODE + (NUM_EXT_LUMA_MODE >> 1),
              "luma mode out of range");
        predMode       = (predMode < 0) ? 2 : (predMode >= NUM_LUMA_MODE) ? 66 : predMode;
        nMdIdx         = predMode > DIA_IDX ? (NUM_LUMA_MODE + 1 - predMode) : predMode;
        isTrTransposed = (predMode > DIA_IDX) ? true : false;
      }
      const uint8_t nSzIdx = isTrTransposed ? (nSzIdxH * 4 + nSzIdxW) : (nSzIdxW * 4 + nSzIdxH);
      CHECK(nSzIdx >= 16, "nSzIdx >= 16");
      CHECK(nMdIdx >= 36, "nMdIdx >= 36");
      const uint8_t nTrSet = g_aucIpmToTrSet[nSzIdx][nMdIdx];
      CHECK(nTrSet >= 80, "nTrSet >= 80");
      trTypeVer = g_aucTrIdxToTr[g_aucTrSet[nTrSet][trIdx]][predMode > DIA_IDX ? 1 : 0];
      trTypeHor = g_aucTrIdxToTr[g_aucTrSet[nTrSet][trIdx]][predMode > DIA_IDX ? 0 : 1];
      predMode  = ucMode; // to Check IDTR criteria, signaled mode should be used to check the difference
      if (trIdx == 3 && width <= 16 && height <= 16)
      {
        if (abs(predMode - HOR_IDX) <= g_aiIdLut[floorLog2(width) - 2][floorLog2(height) - 2])
        {
          trTypeVer = TransType::IDTR;
        }
        if (abs(predMode - VER_IDX) <= g_aiIdLut[floorLog2(width) - 2][floorLog2(height) - 2])
        {
          trTypeHor = TransType::IDTR;
        }
      }
    }
    else if (isMTS(tu.mtsIdx[compID]))
    {
      int trIdx = (tu.mtsIdx[compID] - MtsType::MTS_1);
      CHECK(trIdx < 0 || trIdx > 3, "Invalid Inter MTS");
      int indHor      = trIdx & 1;
      int indVer      = trIdx >> 1;
      trTypeHor       = indHor ? TransType::DCT8 : TransType::DST7;
      trTypeVer       = indVer ? TransType::DCT8 : TransType::DST7;
      uint32_t width  = tu.blocks[compID].width;
      uint32_t height = tu.blocks[compID].height;
      CHECK(width < 4 || height < 4, "width < 4 || height < 4 for KLT");
      if (width <= 16 && height <= 16)
      {
        trTypeHor = indHor ? TransType::KLT1 : TransType::KLT0;
        trTypeVer = indVer ? TransType::KLT1 : TransType::KLT0;
      }
    }
  }
}

void TrQuant::xT(const TransformUnit &tu, const CompID &compID, const CPelBuf &resi, CoeffBuf &dstCoeff,
                 const int width, const int height)
{
  PROFILER_SCOPE(TP_ENABLE_TRQUANT_STAGES, g_timeProfiler, P_TRAFO);
  const unsigned maxLog2TrDynamicRange  = tu.cs->sps->getMaxLog2TrDynamicRange(toChannelType(compID));
  const unsigned bitDepth               = tu.cs->sps->m_bitDepths[toChannelType(compID)];
  const int      TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_FORWARD];
  const uint32_t transformWidthIndex = floorLog2(width) - 1;   // nLog2WidthMinus1, since transform start from 2-point
  const uint32_t transformHeightIndex =
    floorLog2(height) - 1;   // nLog2HeightMinus1, since transform start from 2-point

  auto trTypeHor = TransType::DCT2;
  auto trTypeVer = TransType::DCT2;

  getTrTypes(tu, compID, trTypeHor, trTypeVer);

  int skipWidth  = width > MAX_NONZERO_TU_SIZE ? width - MAX_NONZERO_TU_SIZE : 0;
  int skipHeight = height > MAX_NONZERO_TU_SIZE ? height - MAX_NONZERO_TU_SIZE : 0;

  const bool allowNSPT = TU::isNSPTAllowed(width, height);
  const int  lfnstIdx  = TU::getNstIdx(tu, compID);

  if (lfnstIdx && !allowNSPT)
  {
    if ((width == 4 && height > 4) || (width > 4 && height == 4))
    {
      skipWidth  = std::max<int>(0, width - 4);
      skipHeight = std::max<int>(0, height - 4);
    }
    else if ((width >= 16 && height >= 16))
    {
      skipWidth  = width - 16;
      skipHeight = height - 16;
    }
    else if ((width >= 8 && height >= 8))
    {
      skipWidth  = width - 8;
      skipHeight = height - 8;
    }
  }

#if RExt__DECODER_DEBUG_TOOL_STATISTICS
  if (trTypeHor != TransType::DCT2)
  {
    CodingStatistics::IncrementStatisticTool(
      CodingStatisticsClassType { STATS__TOOL_EMT, uint32_t(width), uint32_t(height), compID });
  }
#endif

  TCoeff *block = m_blk;
  TCoeff *tmp   = m_tmp;

  const Pel      *resiBuf    = resi.buf;
  const ptrdiff_t resiStride = resi.stride;

#if ENABLE_SIMD_TRAFO
  if (width & 3)
#endif
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        block[(y * width) + x] = resiBuf[(y * resiStride) + x];
      }
    }
  }
#if ENABLE_SIMD_TRAFO
  else if (width & 7)
  {
    g_tCoeffOps.cpyCoeff4(resiBuf, resiStride, block, width, height);
  }
  else
  {
    g_tCoeffOps.cpyCoeff8(resiBuf, resiStride, block, width, height);
  }
#endif   // ENABLE_SIMD_TRAFO

  if (width > 1 && height > 1)   // 2-D transform
  {
    const int shift1st =
      ((floorLog2(width)) + bitDepth + TRANSFORM_MATRIX_SHIFT) - maxLog2TrDynamicRange + COM16_C806_TRANS_PREC;
    const int shift2nd = (floorLog2(height)) + TRANSFORM_MATRIX_SHIFT + COM16_C806_TRANS_PREC;
    CHECK(shift1st < 0, "Negative shift");
    CHECK(shift2nd < 0, "Negative shift");

    if (TU::nsptApplyCond(tu, compID, allowNSPT))
    {
      xFwdNspt(tu, block, dstCoeff.buf, compID, shift1st + shift2nd, lfnstIdx);
    }
    else
    {
      fastFwdTrans[trTypeHor][transformWidthIndex](block, tmp, shift1st, height, 0, skipWidth);
      fastFwdTrans[trTypeVer][transformHeightIndex](tmp, dstCoeff.buf, shift2nd, width, skipWidth, skipHeight);
    }
  }
  else if (height == 1)   // 1-D horizontal transform
  {
    const int shift =
      ((floorLog2(width)) + bitDepth + TRANSFORM_MATRIX_SHIFT) - maxLog2TrDynamicRange + COM16_C806_TRANS_PREC;
    CHECK(shift < 0, "Negative shift");
    CHECKD((transformWidthIndex < 0), "There is a problem with the width.");
    fastFwdTrans[trTypeHor][transformWidthIndex](block, dstCoeff.buf, shift, 1, 0, skipWidth);
  }
  else   // if (width == 1) //1-D vertical transform
  {
    int shift =
      ((floorLog2(height)) + bitDepth + TRANSFORM_MATRIX_SHIFT) - maxLog2TrDynamicRange + COM16_C806_TRANS_PREC;
    CHECK(shift < 0, "Negative shift");
    CHECKD((transformHeightIndex < 0), "There is a problem with the height.");
    fastFwdTrans[trTypeVer][transformHeightIndex](block, dstCoeff.buf, shift, 1, 0, skipHeight);
  }
}

void TrQuant::xIT(const TransformUnit &tu, const CompID &compID, const CCoeffBuf &pCoeff, PelBuf &pResidual)
{
  PROFILER_SCOPE(TP_ENABLE_TRQUANT_STAGES, g_timeProfiler, P_TRAFO);
  const int      width                  = pCoeff.width;
  const int      height                 = pCoeff.height;
  const unsigned maxLog2TrDynamicRange  = tu.cs->sps->getMaxLog2TrDynamicRange(toChannelType(compID));
  const unsigned bitDepth               = tu.cs->sps->m_bitDepths[toChannelType(compID)];
  const int      TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_INVERSE];
  const TCoeff   clipMinimum            = -(1 << maxLog2TrDynamicRange);
  const TCoeff   clipMaximum            = (1 << maxLog2TrDynamicRange) - 1;
  const TCoeff   pelMinimum             = std::numeric_limits<Pel>::min();
  const TCoeff   pelMaximum             = std::numeric_limits<Pel>::max();
  const uint32_t transformWidthIndex = floorLog2(width) - 1;   // nLog2WidthMinus1, since transform start from 2-point
  const uint32_t transformHeightIndex =
    floorLog2(height) - 1;   // nLog2HeightMinus1, since transform start from 2-point

  auto trTypeHor = TransType::DCT2;
  auto trTypeVer = TransType::DCT2;

  getTrTypes(tu, compID, trTypeHor, trTypeVer);

  int skipWidth  = width > MAX_NONZERO_TU_SIZE ? width - MAX_NONZERO_TU_SIZE : 0;
  int skipHeight = height > MAX_NONZERO_TU_SIZE ? height - MAX_NONZERO_TU_SIZE : 0;

  const bool allowNSPT = TU::isNSPTAllowed(width, height);
  const int  lfnstIdx  = TU::getNstIdx(tu, compID);

  if (lfnstIdx && !allowNSPT)
  {
    if ((width == 4 && height > 4) || (width > 4 && height == 4))
    {
      skipWidth  = std::max<int>(0, width - 4);
      skipHeight = std::max<int>(0, height - 4);
    }
    else if ((width >= 16 && height >= 16))
    {
      skipWidth  = width - 16;
      skipHeight = height - 16;
    }
    else if ((width >= 8 && height >= 8))
    {
      skipWidth  = width - 8;
      skipHeight = height - 8;
    }
  }

  TCoeff *block = m_blk;
  TCoeff *tmp   = m_tmp;

  if (width > 1 && height > 1)   // 2-D transform
  {
    const int shift1st =
      TRANSFORM_MATRIX_SHIFT + 1 + COM16_C806_TRANS_PREC;   // 1 has been added to shift_1st at the expense of shift_2nd
    const int shift2nd = (TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1) - bitDepth + COM16_C806_TRANS_PREC;
    CHECK(shift1st < 0, "Negative shift");
    CHECK(shift2nd < 0, "Negative shift");
    if (TU::nsptApplyCond(tu, compID, allowNSPT))
    {
      xInvNspt(tu, pCoeff.buf, block, compID, shift1st + shift2nd, lfnstIdx);
    }
    else
    {
      fastInvTrans[trTypeVer][transformHeightIndex](pCoeff.buf, tmp, shift1st, width, skipWidth, skipHeight,
                                                    clipMinimum, clipMaximum);
      fastInvTrans[trTypeHor][transformWidthIndex](tmp, block, shift2nd, height, 0, skipWidth, pelMinimum, pelMaximum);
    }
  }
  else if (width == 1)   // 1-D vertical transform
  {
    int shift = (TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1) - bitDepth + COM16_C806_TRANS_PREC;
    CHECK(shift < 0, "Negative shift");
    CHECK((transformHeightIndex < 0), "There is a problem with the height.");
    fastInvTrans[trTypeVer][transformHeightIndex](pCoeff.buf, block, shift + 1, 1, 0, skipHeight, pelMinimum,
                                                  pelMaximum);
  }
  else   // if(height == 1) //1-D horizontal transform
  {
    const int shift = (TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1) - bitDepth + COM16_C806_TRANS_PREC;
    CHECK(shift < 0, "Negative shift");
    CHECK((transformWidthIndex < 0), "There is a problem with the width.");
    fastInvTrans[trTypeHor][transformWidthIndex](pCoeff.buf, block, shift + 1, 1, 0, skipWidth, pelMinimum, pelMaximum);
  }

#if ENABLE_SIMD_TRAFO
  if (width & 3)
#endif   // ENABLE_SIMD_TRAFO
  {
    Pel      *resiBuf    = pResidual.buf;
    ptrdiff_t resiStride = pResidual.stride;

    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        resiBuf[x] = (Pel)*block++;
      }

      resiBuf += resiStride;
    }
  }
#if ENABLE_SIMD_TRAFO
  else if (width & 7)
  {
    g_tCoeffOps.cpyResi4(block, pResidual.buf, pResidual.stride, width, height);
  }
  else
  {
    g_tCoeffOps.cpyResi8(block, pResidual.buf, pResidual.stride, width, height);
  }
#endif   // ENABLE_SIMD_TRAFO
}

/** Wrapper function between HM interface and core NxN transform skipping
 */
void TrQuant::xITransformSkip(const CCoeffBuf &pCoeff, PelBuf &pResidual, const TransformUnit &tu, const CompID &compID)
{
  const CompArea &area   = tu.blocks[compID];
  const int       width  = area.width;
  const int       height = area.height;

  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      pResidual.at(x, y) = Pel(pCoeff.at(x, y));
    }
  }
}

void TrQuant::xQuant(TransformUnit &tu, const CompID &compID, const CCoeffBuf &pSrc, TCoeff &absSum, const QpParam &cQP,
                     const Ctx &ctx)
{
  PROFILER_SCOPE(TP_ENABLE_TRQUANT_STAGES, g_timeProfiler, P_QUANT);
  m_quant->quant(tu, compID, pSrc, absSum, cQP, ctx);
}

void TrQuant::preCalcTrans(TransformUnit &tu, const CompID &compID, const CPelBuf &res, const TransList &tl,
                           TransBuffer &tbuf, const bool dctAvailable)
{
  const CompArea &area      = tu.blocks[compID];
  const MtsType   trOrg     = tu.mtsIdx[compID];
  const bool      allowNSPT = TU::isNSPTAllowed(area.width, area.height);
  bool            haveDCT   = dctAvailable;

  for (auto &tr: tl)
  {
    TCoeff *tc        = tbuf[tr];
    tu.mtsIdx[compID] = tr;

    if (tr == MtsType::SKIP)
    {
      xTransformSkip(tu, compID, res, tc);
    }
    else if (isNST(tr) && !allowNSPT)
    {
      if (!haveDCT)
      {
        CoeffBuf tcoeff(tbuf[MtsType::DCT2_DCT2], area);
        xT(tu, compID, res, tcoeff, area.width, area.height);
        haveDCT = true;
      }
      ::memcpy(tc, tbuf[MtsType::DCT2_DCT2], sizeof(TCoeff) * area.area());
      xFwdLfnst(tu, compID, tc);
    }
    else if (isNST(tr) || isMTS(tr) || !haveDCT)
    {
      CoeffBuf tcoeff(tc, area);
      xT(tu, compID, res, tcoeff, area.width, area.height);
      if (tr == MtsType::DCT2_DCT2)
      {
        haveDCT = true;
      }
    }
  }
  tu.mtsIdx[compID] = trOrg;
}

void TrQuant::quantNxN(TransformUnit &tu, const CompID &compID, const QpParam &cQP, TCoeff &absSum, const Ctx &ctx,
                       TransBuffer &tcoeff)
{
  absSum = 0;
  if (tu.noResidual)
  {
    TU::setCbfAtDepth(tu, compID, tu.depth, absSum > 0);
    return;
  }

  const CompArea &area  = tu.blocks[compID];
  const MtsType   ttype = tu.mtsIdx[compID];
  const CCoeffBuf orgCoeff(tcoeff[ttype], area);

  DTRACE_COEFF_BUF(D_TCOEFF, orgCoeff, tu, tu.cu->predMode, compID);

  xQuant(tu, compID, orgCoeff, absSum, cQP, ctx);

  DTRACE_COEFF_BUF(D_TCOEFF, tu.getCoeffs(compID), tu, tu.cu->predMode, compID);

  // set coded block flag (CBF)
  TU::setCbfAtDepth(tu, compID, tu.depth, absSum > 0);
}

void TrQuant::xTransformSkip(const TransformUnit &tu, const CompID &compID, const CPelBuf &resi, TCoeff *psCoeff)
{
  const CompArea &rect   = tu.blocks[compID];
  const uint32_t  width  = rect.width;
  const uint32_t  height = rect.height;

  for (uint32_t y = 0, coefficientIndex = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++, coefficientIndex++)
    {
      psCoeff[coefficientIndex] = TCoeff(resi.at(x, y));
    }
  }
}

namespace TrEst
{
std::vector<Cost> &sortCL(std::vector<Cost> &cl)
{
  std::stable_sort(cl.begin(), cl.end(), CostCmp());
  return cl;
}

std::vector<Cost> &addCL(std::vector<Cost> &cl, const std::vector<Cost> &add)
{
  cl.insert(cl.end(), add.cbegin(), add.cend());
  return cl;
}

ZeroOut::ZeroOut(const Size &sz, const MtsType tr) : nzW(sz.width), nzH(sz.height), num(0)
{
  if (isNST(tr))
  {
    int d = TU::isNSPTAllowed(sz.width, sz.height) ? PU::getNSPTMatrixDim(sz.width, sz.height)
                                                   : PU::getLFNSTMatrixDim(sz.width, sz.height);
    nzW   = std::min<uint32_t>(8, sz.width);
    nzH   = std::min<uint32_t>(8, sz.height);
    num   = sz.area() - d;
  }
  else if (isMTS(tr) || tr == MtsType::DCT2_DCT2)
  {
    nzW = std::min<uint32_t>(MAX_NONZERO_TU_SIZE, sz.width);
    nzH = std::min<uint32_t>(MAX_NONZERO_TU_SIZE, sz.height);
    num = sz.area() - nzW * nzH;
  }
}

PreCostBase::PreCostBase(const CompID c, TransformUnit &tu, const double l1Lambda, const double l1Scale,
                         const double skipScale)
  : compId(c)
  , tu(tu)
  , cuCtx()
  , sz(tu.blocks[compId == CompID::JOINT_CbCr ? CompID::COMP_Cb : compId].size())
  , num(sz.area())
  , orthoScaleAbs(calcOrthoScale())
  , orthoScaleSqr(orthoScaleAbs * orthoScaleAbs)
  , invL1Lambda(1.0 / l1Lambda)
  , distScaleAbs(l1Scale * invL1Lambda)
  , distScaleSqr(invL1Lambda * invL1Lambda)
  , skipFactor(skipScale)
  , bsizeId(std::max<int>(0, floorLog2(sz.area()) - 4))
{
  cuCtx.lfnstLastScanPos = true;
  cuCtx.mtsLastScanPos   = true;
  cuCtx.mtsCoeffAbsSum   = MTS_TH_COEFF[1] << 1;
}

double PreCostBase::calcOrthoScale()
{
  const int    bd = tu.cu->slice->m_sps->m_bitDepths[toChannelType(compId)];
  const int    dr = tu.cu->slice->m_sps->getMaxLog2TrDynamicRange(toChannelType(compId));
  const int    ep = (int(sz.width > 1) + int(sz.height > 1)) * COM16_C806_TRANS_PREC;
  const int    ls = bd - dr - ep;
  const double os = sqrt(double(num)) * exp2(double(ls));
  return os;
}

bool PreCostBase::includesZeroOut(const TransList &tl)
{
  for (const MtsType tr: tl)
  {
    if (ZeroOut(sz, tr).num)
    {
      return true;
    }
  }
  return false;
}

void PreCostBase::initResidual(const TransList &tl, const Pel *p)
{
  if (includesZeroOut(tl))
  {
    sumSqrRes = sumResSqr(p);
    return;
  }
  sumSqrRes = 0.0;
}

void PreCostBase::initResidual(const TransList &tl, const Pel *pa, const Pel *pb)
{
  if (includesZeroOut(tl))
  {
    sumSqrRes = sumResSqr(pa) + sumResSqr(pb);
    return;
  }
  sumSqrRes = 0.0;
}

double PreCostBase::getEstTrCost(const MtsType tr, const TransBuffer &tb)
{
  if (tr == MtsType::SKIP)
  {
    double d1 = sumResAbs(tb[tr]);
    return d1 * distScaleAbs * skipFactor;
  }
  ZeroOut zo(sz, tr);
  if (zo.num)
  {
    auto   ds = sumTrnZO(tb[tr], zo);
    double d1 = ds.abs;
    double d2 = std::max<double>(sumSqrRes - ds.sqr, 0.0);
    return d1 * distScaleAbs + d2 * distScaleSqr;
  }
  double d1 = sumTrnAbs(tb[tr]);
  return d1 * distScaleAbs;
}

double PreCostBase::getEstTrCost(const MtsType tr, const TransBuffer &tba, const TransBuffer &tbb)
{
  if (tr == MtsType::SKIP)
  {
    double d1 = sumResAbs(tba[tr]) + sumResAbs(tbb[tr]);
    return d1 * distScaleAbs * skipFactor;
  }
  ZeroOut zo(sz, tr);
  if (zo.num)
  {
    auto   da = sumTrnZO(tba[tr], zo);
    auto   db = sumTrnZO(tbb[tr], zo);
    double d1 = da.abs + db.abs;
    double d2 = std::max<double>(sumSqrRes - da.sqr - db.sqr, 0.0);
    return d1 * distScaleAbs + d2 * distScaleSqr;
  }
  double d1 = sumTrnAbs(tba[tr]) + sumTrnAbs(tbb[tr]);
  return d1 * distScaleAbs;
}

template<typename T> double PreCostBase::sumResAbs(const T *p)
{
  int32_t sum = 0;
  for (uint32_t k = 0; k < num; k++)
  {
    sum += abs(p[k]);
  }
  return double(sum);
}

template<typename T> double PreCostBase::sumResSqr(const T *p)
{
  int64_t sum = 0;
  for (uint32_t k = 0; k < num; k++)
  {
    sum += p[k] * p[k];
  }
  return double(sum);
}

template<typename T> double PreCostBase::sumTrnAbs(const T *p)
{
  int64_t sum = 0;
  for (uint32_t k = 0; k < num; k++)
  {
    sum += abs(p[k]);
  }
  return double(sum) * orthoScaleAbs;
}

template<typename T> DSums PreCostBase::sumTrnZO(const T *p, const ZeroOut &zo)
{
  int32_t sum1 = 0;
  int64_t sum2 = 0;
  for (uint32_t y = 0; y < zo.nzH; y++, p += sz.width)
  {
    for (uint32_t x = 0; x < zo.nzW; x++)
    {
      sum1 += abs(p[x]);
      sum2 += p[x] * p[x];
    }
  }
  return DSums(double(sum1) * orthoScaleAbs, double(sum2) * orthoScaleSqr);
}
}   // namespace TrEst

namespace SignPred
{
typedef Pel sprd_brd_t;

void _predLeft(const CPelBuf &recoBuf, const CPelBuf &predBuf, const std::vector<Pel> *fwdLUT, sprd_brd_t *dst)
{
  const Pel *reco = recoBuf.buf;
  const Pel *pred = predBuf.buf;
  sprd_brd_t avg, sgn;
  if (fwdLUT)
  {
    const auto lut = *fwdLUT;
    for (int32_t y = 0; y < predBuf.height; y += 2, reco += 2 * recoBuf.stride, pred += 2 * predBuf.stride)
    {
      avg = (reco[0 - 1] << 1) - reco[0 - 2] - lut[pred[0]];
      avg += (reco[recoBuf.stride - 1] << 1) - reco[recoBuf.stride - 2] - lut[pred[predBuf.stride]];
      sgn            = avg < 0 ? -1 : 1;
      avg            = (std::abs(avg) + 1) >> 1;
      dst[~(y >> 1)] = sgn * avg;
    }
  }
  else
  {
    for (int32_t y = 0; y < predBuf.height; y += 2, reco += 2 * recoBuf.stride, pred += 2 * predBuf.stride)
    {
      avg = (reco[0 - 1] << 1) - reco[0 - 2] - pred[0];
      avg += (reco[recoBuf.stride - 1] << 1) - reco[recoBuf.stride - 2] - pred[predBuf.stride];
      sgn            = avg < 0 ? -1 : 1;
      avg            = (std::abs(avg) + 1) >> 1;
      dst[~(y >> 1)] = sgn * avg;
    }
  }
}

void _predLeft(const Pel defaultVal, const CPelBuf &predBuf, const std::vector<Pel> *fwdLUT, sprd_brd_t *dst)
{
  const Pel *pred = predBuf.buf;
  sprd_brd_t avg, sgn;
  if (fwdLUT)
  {
    const auto lut = *fwdLUT;
    for (int32_t y = 0; y < predBuf.height; y += 2, pred += 2 * predBuf.stride)
    {
      avg = defaultVal - lut[pred[0]];
      avg += defaultVal - lut[pred[predBuf.stride]];
      sgn            = avg < 0 ? -1 : 1;
      avg            = (std::abs(avg) + 1) >> 1;
      dst[~(y >> 1)] = sgn * avg;
    }
  }
  else
  {
    for (int32_t y = 0; y < predBuf.height; y += 2, pred += 2 * predBuf.stride)
    {
      avg = defaultVal - pred[0];
      avg += defaultVal - pred[predBuf.stride];
      sgn            = avg < 0 ? -1 : 1;
      avg            = (std::abs(avg) + 1) >> 1;
      dst[~(y >> 1)] = sgn * avg;
    }
  }
}

void _predTop(const CPelBuf &recoBuf, const CPelBuf &predBuf, const std::vector<Pel> *fwdLUT, sprd_brd_t *dst)
{
  const Pel *pred  = predBuf.buf;
  const Pel *reco1 = recoBuf.buf - recoBuf.stride;
  const Pel *reco2 = reco1 - recoBuf.stride;
  sprd_brd_t avg, sgn;
  if (fwdLUT)
  {
    const auto lut = *fwdLUT;
    for (int32_t x = 0; x < predBuf.width; x += 2)
    {
      avg = (reco1[x + 0] << 1) - reco2[x + 0] - lut[pred[x + 0]];
      avg += (reco1[x + 1] << 1) - reco2[x + 1] - lut[pred[x + 1]];
      sgn         = avg < 0 ? -1 : 1;
      avg         = (std::abs(avg) + 1) >> 1;
      dst[x >> 1] = sgn * avg;
    }
  }
  else
  {
    for (int32_t x = 0; x < predBuf.width; x += 2)
    {
      avg = (reco1[x + 0] << 1) - reco2[x + 0] - pred[x + 0];
      avg += (reco1[x + 1] << 1) - reco2[x + 1] - pred[x + 1];
      sgn         = avg < 0 ? -1 : 1;
      avg         = (std::abs(avg) + 1) >> 1;
      dst[x >> 1] = sgn * avg;
    }
  }
}

void _predTop(const Pel defaultVal, const CPelBuf &predBuf, const std::vector<Pel> *fwdLUT, sprd_brd_t *dst)
{
  const Pel *pred = predBuf.buf;
  sprd_brd_t avg, sgn;
  if (fwdLUT)
  {
    const auto lut = *fwdLUT;
    for (int32_t x = 0; x < predBuf.width; x += 2)
    {
      avg = defaultVal - lut[pred[x + 0]];
      avg += defaultVal - lut[pred[x + 1]];
      sgn         = avg < 0 ? -1 : 1;
      avg         = (std::abs(avg) + 1) >> 1;
      dst[x >> 1] = sgn * avg;
    }
  }
  else
  {
    for (int32_t x = 0; x < predBuf.width; x += 2)
    {
      avg = defaultVal - pred[x + 0];
      avg += defaultVal - pred[x + 1];
      sgn         = avg < 0 ? -1 : 1;
      avg         = (std::abs(avg) + 1) >> 1;
      dst[x >> 1] = sgn * avg;
    }
  }
}

void predBorder(const CompArea area, const CPelBuf &recoBuf, const CPelBuf &predBuf, const CompID compID,
                sprd_brd_t *predResiBorder, const sprd_brd_t defaultVal, const std::vector<Pel> *fwdLUT)
{
  const bool  useLeft = area.x != 0 || area.y == 0;
  const bool  useTop  = area.y != 0 || area.x == 0;
  sprd_brd_t *dst     = predResiBorder + (area.height >> 1);

  if (useLeft)
  {
    if (area.x)
    {
      _predLeft(recoBuf, predBuf, fwdLUT, dst);
    }
    else
    {
      _predLeft(defaultVal, predBuf, fwdLUT, dst);
    }
  }
  if (useTop)
  {
    if (area.y)
    {
      _predTop(recoBuf, predBuf, fwdLUT, dst);
    }
    else
    {
      _predTop(defaultVal, predBuf, fwdLUT, dst);
    }
  }
}

uint32_t getBestPattern(const static_vector<uint32_t, 1 << SIGN_PRED_MAX_NUM> &costs, const uint32_t first,
                        const uint32_t last)
{
  uint32_t minVal = ~0u;
  for (uint32_t idx = first; idx < last; idx++)
  {
    minVal = std::min(minVal, costs[idx]);
  }
  return (minVal & ~(~0u << SIGN_PRED_MAX_NUM));
}

bool hasPredSigns(const TransformUnit &tu, const CompID compID)
{
  const bool   isJCCR    = tu.jointCbCr && isChroma(compID);
  const CompID resCompID = isJCCR && !(tu.jointCbCr >> 1) ? CompID::COMP_Cr : compID;
  if (!TU::getUseSignPred(tu, resCompID) || !(TU::getCbf(tu, compID) || isJCCR))
  {
    return false;
  }
  if (isJCCR && compID == CompID::COMP_Cr)
  {
    return false;
  }
  return true;
}
}   // namespace SignPred

TrQuant::TrBrdTemplate TrQuant::xGetSignPredTemplate(const TransformUnit &tu, const CompID compID)
{
  const int      signPredShift = SIGN_PRED_RESIDUAL_BITS + 10 - tu.cs->sps->m_bitDepths[toChannelType(compID)];
  const TCoeff   signPredCoeff = TCoeff(1) << signPredShift;
  const Size     maxlog2SPArea = TU::getMaxLog2SignPredArea(tu, compID);
  const Size     maxSPArea { 1u << maxlog2SPArea.width, 1u << maxlog2SPArea.height };
  const int      uniTransIdx   = xGetUniTransformIdx(tu, compID);
  const Size     blkSize       = tu.blocks[compID].size();
  const SizeType log2BlkWidth  = floorLog2(blkSize.width);
  const SizeType log2BlkHeight = floorLog2(blkSize.height);
  const Size     bufferSize =
    Size((blkSize.width + blkSize.height) >> 1u, 1u << (maxlog2SPArea.width + maxlog2SPArea.height));
  int8_t *&templateData = g_signPredTemplate[log2BlkWidth - 2][log2BlkHeight - 2][uniTransIdx];
  if (!templateData)
  {
    const bool nonSep = TU::getNstIdx(tu, compID) && tu.mtsIdx[compID] != MtsType::SKIP &&
      (CS::isDualITree(*tu.cs) ? true : isLuma(compID));
    const bool isLfnst = nonSep && !TU::isNSPTAllowed(blkSize.width, blkSize.height);

    templateData           = (int8_t *)xMalloc(int8_t, bufferSize.area());
    CoeffBuf  coeff        = CoeffBuf(m_tempCoeff, blkSize);
    PelBuf    resi         = PelBuf(m_tempResi, blkSize);
    Position  prevPos      = Position();
    const int posShiftY    = maxlog2SPArea.width;
    const int posMaskX     = (1 << posShiftY) - 1;
    int8_t   *currTemplate = templateData;
    if (!isLfnst)
    {
      coeff.fill(0);   // for Lfnst, we have to always reset all coefficients
    }
    for (int k = 0; k < bufferSize.height; k++, currTemplate += bufferSize.width)
    {
      const Position currPos(k & posMaskX, k >> posShiftY);
      if (isLfnst)
      {
        coeff.fill(0);
      }
      else
      {
        coeff.at(prevPos) = 0;
        prevPos           = currPos;
      }
      coeff.at(currPos) = signPredCoeff;

      xInvLfnst(tu, compID);
      xIT(tu, compID, coeff, resi);

      Pel *resiData = resi.bufAt(0, blkSize.height - 1);
      Pel  avg, sign;
      for (SizeType y = 0; y < blkSize.height; y += 2, resiData -= 2 * resi.stride)
      {
        CHECK(resiData[0] < -128 || resiData[0] > 127, "value exceeds 8-bit range");
        CHECK(resiData[-resi.stride] < -128 || resiData[-resi.stride] > 127, "value exceeds 8-bit range");
        avg = resiData[0];
        avg += resiData[-resi.stride];
        sign                 = avg < 0 ? -1 : 1;
        avg                  = (std::abs(avg) + 1) >> 1;
        currTemplate[y >> 1] = (int8_t)sign * avg;
      }
      resiData        = resi.buf;
      int8_t *tmplTop = currTemplate + (blkSize.height >> 1);
      for (SizeType x = 0; x < blkSize.width; x += 2)
      {
        CHECK(resiData[x + 0] < -128 || resiData[x + 0] > 127, "value exceeds 8-bit range");
        CHECK(resiData[x + 1] < -128 || resiData[x + 1] > 127, "value exceeds 8-bit range");
        avg             = resiData[x] + resiData[x + 1];
        sign            = avg < 0 ? -1 : 1;
        avg             = (std::abs(avg) + 1) >> 1;
        tmplTop[x >> 1] = (int8_t)sign * avg;
      }
    }
  }
  return TrQuant::TrBrdTemplate(AreaBuf(templateData, bufferSize), maxlog2SPArea);
}

static_vector<uint32_t, 1 << SIGN_PRED_MAX_NUM> TrQuant::xCalcSignPredCosts(TransformUnit &tu, const CompID compID,
                                                                            const std::vector<Position> &sprdPos,
                                                                            const std::vector<Pel>      *fwdLUT)
{
  //===== get prediction for top-left residual border (inside block)
  CodingStructure     &cs         = *tu.cs;
  const CompArea       area       = tu.blocks[compID];
  const CPelBuf        recoBuf    = cs.picture->getRecoBuf(area);
  const CPelBuf        predBuf    = cs.getPredBuf(area);
  const Pel            defaultVal = (1 << (tu.cs->sps->m_bitDepths[toChannelType(compID)] - 1));
  SignPred::sprd_brd_t predResiBorder[2 * SIGN_PRED_MAX_BUF_SIZE];
  SignPred::predBorder(area, recoBuf, predBuf, compID, predResiBorder, defaultVal, fwdLUT);

  //===== create (if required) and set normalized sign prediction template for block size and transform =====
  const auto normalizedSignPredTemplate = xGetSignPredTemplate(tu, compID);

  //===== get dequantized coefficients (using positive values for sign predicted positions) ======
  const Size    maxlog2SPArea = TU::getMaxLog2SignPredArea(tu, compID);
  const Size    maxSPArea { 1u << maxlog2SPArea.width, 1u << maxlog2SPArea.height };
  const QpParam cQP(tu, compID);
  CoeffBuf      recCoeff(m_tempCoeff, area);
  xDeQuant(tu, recCoeff, compID, cQP);

  for (auto &p: sprdPos)
  {
    TCoeff &coeff = recCoeff.at(p);
    if (coeff < 0)
    {
      coeff = -coeff;
    }
  }

  //===== calculate impact of predicted coefficients on residual border =====
  SignPred::sprd_brd_t scaledSignPredTemplate[SIGN_PRED_MAX_NUM * SIGN_PRED_MAX_BUF_SIZE * 2];
  const uint32_t       borderSize     = (area.width + area.height) >> 1;
  const bool           useLeft        = area.x != 0 || area.y == 0;
  const bool           useTop         = area.y != 0 || area.x == 0;
  const int            first          = useLeft ? 0 : (area.height >> 1);
  const int            last           = useTop ? borderSize : (area.height >> 1);
  const int            numel          = last - first;
  const int            signPredShift  = SIGN_PRED_RESIDUAL_BITS + 10 - tu.cs->sps->m_bitDepths[toChannelType(compID)];
  const TCoeff         signPredOffset = (TCoeff(1) << signPredShift) >> 1;
  for (int posIdx = 0; posIdx < sprdPos.size(); posIdx++)
  {
    const Position &coeffPos  = sprdPos[posIdx];
    const TCoeff    absCoeff  = recCoeff.at(coeffPos);
    const int8_t   *nrmBrdVec = normalizedSignPredTemplate[coeffPos] + first;
    auto           *sclBrdVec = scaledSignPredTemplate + posIdx * borderSize + first;
    CHECKD(absCoeff <= 0, "coefficient value should be positive");
    for (int k = 0; k < numel; k++)
    {
      // absCoeff should be in -32768..32767 range and nrmBrdVec[j] in -63..63 ---> output range should be about
      // -8064..8064 (i.e. 15bit)
      *sclBrdVec++ = (absCoeff * *nrmBrdVec++ + signPredOffset) >> signPredShift;
    }
  }

  //===== calculated actual residual border (using positive values for sign predicted positions) =====
  SignPred::sprd_brd_t resiBorder[2 * SIGN_PRED_MAX_BUF_SIZE];
  memset(resiBorder, 0, sizeof(resiBorder));

  for (int y = 0; y < maxSPArea.height; y++)
  {
    for (int x = 0; x < maxSPArea.width; x++)
    {
      const TCoeff coeff = recCoeff.at(x, y);
      if (coeff)
      {
        const int8_t *nrmBrdVec = normalizedSignPredTemplate[Position { x, y }] + first;
        auto          resi      = resiBorder + first;
        const TCoeff  absCoeff  = std::abs(coeff);
        const TCoeff  sgnCoeff  = (coeff < 0 ? -1 : 1);
        for (int k = 0; k < numel; k++)
        {
          // TODO: SIMD
          *resi++ += sgnCoeff * ((absCoeff * *nrmBrdVec++ + signPredOffset) >> signPredShift);
        }
      }
    }
  }

  //===== calculate border costs for case that all predicted signs are positive =====
  const bool    reshapeChroma = isChroma(compID) && tu.getChromaAdj();
  const int16_t chromaScale   = tu.getChromaAdj();
  const Pel     maxVal        = (1 << tu.cu->cs->slice->clpRng(compID).bd) - 1;
  uint32_t      allPosCost    = 0;

  SignPred::sprd_brd_t *pred = predResiBorder + first;
  SignPred::sprd_brd_t *resi = resiBorder + first;
  if (reshapeChroma)
  {
    for (int k = 0; k < numel; k++)
    {
      allPosCost += abs(*pred++ - Reshape::scalePel(*resi++, chromaScale, maxVal));
    }
  }
  else
  {
    for (int k = 0; k < numel; k++)
    {
      allPosCost += abs(*pred++ - *resi++);
    }
  }

  //===== determine costs and sign patterns for all combinations =====
  const int                                       numPredSigns     = (int)sprdPos.size();
  const uint32_t                                  numSignPredCombs = 1 << numPredSigns;
  static_vector<uint32_t, 1 << SIGN_PRED_MAX_NUM> prdCosts(numSignPredCombs);
  prdCosts[0] = allPosCost << SIGN_PRED_MAX_NUM;
  for (uint32_t combIdx = 1; combIdx < numSignPredCombs; combIdx++)
  {
    const uint32_t signXor  = combIdx & ~(combIdx - 1);
    const uint32_t curSigns = combIdx ^ (combIdx >> 1);
    const bool     negative = (curSigns & signXor);
    const int      signIdx  = numPredSigns - 1 - floorLog2(signXor);
    const auto    *spTempl  = scaledSignPredTemplate + signIdx * borderSize + first;
    uint32_t       cost     = 0;
    resi                    = resiBorder + first;
    if (negative)
    {
      for (int k = 0; k < numel; k++)
      {
        *resi++ -= *spTempl++ << 1;
      }
    }
    else
    {
      for (int k = 0; k < numel; k++)
      {
        *resi++ += *spTempl++ << 1;
      }
    }

    pred = predResiBorder + first;
    resi = resiBorder + first;

    if (reshapeChroma)
    {
      for (int k = 0; k < numel; k++)
      {
        cost += abs(*pred++ - Reshape::scalePel(*resi++, chromaScale, maxVal));
      }
    }
    else
    {
      for (int k = 0; k < numel; k++)
      {
        cost += abs(*pred++ - *resi++);
      }
    }
    prdCosts[curSigns] = (cost << SIGN_PRED_MAX_NUM) | curSigns;
  }
  return prdCosts;
}

#define SKIP_SIGN_PREDICTION_IN_REUSE_CU \
  0   // if enabled, it doesn't do anything when called via EncCu::xReuseCachedResult()
      // "1" doesn't seem to be faster, also it seems to give same results
      // --> keep "0" for now

bool TrQuant::prdCoeffSigns(TransformUnit &tu, const CompID compID, const std::vector<Pel> *fwdLUT)
{
  if (!SignPred::hasPredSigns(tu, compID))
  {
    return false;
  }
  const CompID           resCompID         = isChroma(compID) && tu.jointCbCr == 1 ? CompID::COMP_Cr : compID;
  const size_t           maxNumPredSigns   = tu.maxNumPredSigns(resCompID);
  uint8_t               *signsPredArea     = tu.getSignsPredArea(resCompID);
  std::vector<Position> &predSignPositions = m_predSignPos;
  m_quant->getSignPosPredArea(tu, resCompID, predSignPositions);
  tu.numPredAreaSigns[resCompID] = 0;
  if (predSignPositions.empty())
  {
    return false;
  }

  //===== set non-predicted signs and shorten sign position list =====
  if (predSignPositions.size() > maxNumPredSigns)
  {
    const CCoeffBuf qIndexBuf = tu.getCoeffs(resCompID);
    uint32_t        signCount = 0;
    for (size_t k = maxNumPredSigns; k < predSignPositions.size(); k++)
    {
      const TCoeff qIndex = qIndexBuf.at(predSignPositions[k]);
      signsPredArea[k]    = (qIndex < 0);
      signCount += (abs(qIndex) > 1 ? (1 << 16) + 1 : 1);
    }
    tu.numPredAreaSigns[resCompID] += signCount;
    predSignPositions.resize(maxNumPredSigns);
  }

  //===== set predicted signs for quantization indexes =====
  const auto      sgnPrdCosts = xCalcSignPredCosts(tu, resCompID, predSignPositions, fwdLUT);
  CoeffBuf        qCoefffs    = CoeffBuf(m_tempCoeff, tu.blocks[resCompID].size());
  const CCoeffBuf qIndexes    = tu.getCoeffs(resCompID);
  uint32_t        bestPattern = 0, signIdx = 0, base = 0, signMask = 1 << predSignPositions.size();
  bool            resiSign = true;
  uint32_t        numGt1   = 0;
  for (const auto &signPos: predSignPositions)
  {
    if (resiSign)
    {
      bestPattern = SignPred::getBestPattern(sgnPrdCosts, base, base + signMask);
    }
    signMask >>= 1;
    const TCoeff qIndex      = qIndexes.at(signPos);
    const bool   realSign    = qIndex < 0;
    const bool   predSign    = bestPattern & signMask;
    resiSign                 = predSign ^ realSign;
    signsPredArea[signIdx++] = resiSign;
    numGt1 += abs(qIndex) > 1;
    if (realSign)
    {
      base += signMask;

      TCoeff &qCoeff = qCoefffs.at(signPos);
#if (REUSE_CU_RESULTS && !SKIP_SIGN_PREDICTION_IN_REUSE_CU)
      qCoeff = -abs(qCoeff);   // abs(), because it could be called by EncCu::xReuseCachedResult()
#else
      qCoeff = -qCoeff;
#endif
    }
  }
  tu.numPredAreaSigns[resCompID] += (numGt1 << 16) + (uint32_t)predSignPositions.size();

  return true;
}

bool TrQuant::recCoeffSigns(TransformUnit &tu, const CompID compID, const std::vector<Pel> *fwdLUT)
{
#if (REUSE_CU_RESULTS && SKIP_SIGN_PREDICTION_IN_REUSE_CU)
  if (tu.cs->pcv->isEncoder)
  {
    return false;   // note: don't do anything when called via EncCu::xReuseCachedResult(..)
  }
#endif
  if (!SignPred::hasPredSigns(tu, compID))
  {
    return false;
  }
  const CompID           resCompID           = isChroma(compID) && tu.jointCbCr == 1 ? CompID::COMP_Cr : compID;
  const size_t           maxNumPredSigns     = tu.maxNumPredSigns(resCompID);
  const size_t           numDecodedPredSigns = tu.numPredAreaSigns[resCompID] & ((1 << 16) - 1);
  const uint8_t         *signsPredArea       = tu.getSignsPredArea(resCompID);
  std::vector<Position> &predSignPositions   = m_predSignPos;
  predSignPositions.clear();
  m_quant->getSignPosPredArea(tu, resCompID, predSignPositions);
  CHECK(predSignPositions.size() != numDecodedPredSigns, "number of signs do not match");
  if (predSignPositions.empty())
  {
    return false;
  }

  //===== set non-predicted signs and shorten sign position list =====
  if (predSignPositions.size() > maxNumPredSigns)
  {
    CoeffBuf qIndexBuf = tu.getCoeffs(resCompID);
    for (size_t k = maxNumPredSigns; k < predSignPositions.size(); k++)
    {
      if (signsPredArea[k])
      {
        TCoeff &qIndex = qIndexBuf.at(predSignPositions[k]);
#if (REUSE_CU_RESULTS && !SKIP_SIGN_PREDICTION_IN_REUSE_CU)
        qIndex = -abs(qIndex);   // abs(), because it could be called by EncCu::xReuseCachedResult()
#else
        qIndex = -qIndex;
#endif
      }
    }
    predSignPositions.resize(maxNumPredSigns);
  }

  //===== correct signs of quantization indexes =====
  const auto sgnPrdCosts = xCalcSignPredCosts(tu, resCompID, predSignPositions, fwdLUT);
  CoeffBuf   qCoefffs    = CoeffBuf(m_tempCoeff, tu.blocks[resCompID].size());
  CoeffBuf   qIndexes    = tu.getCoeffs(resCompID);
  uint32_t   bestPattern = 0, signIdx = 0, base = 0, signMask = 1 << predSignPositions.size();
  bool       resiSign = true;
  for (const auto &signPos: predSignPositions)
  {
    if (resiSign)
    {
      bestPattern = SignPred::getBestPattern(sgnPrdCosts, base, base + signMask);
    }
    signMask >>= 1;
    resiSign            = signsPredArea[signIdx++];
    const bool predSign = bestPattern & signMask;
    const bool realSign = predSign ^ resiSign;
    if (realSign)
    {
      TCoeff &qIndex = qIndexes.at(signPos);
#if (REUSE_CU_RESULTS && !SKIP_SIGN_PREDICTION_IN_REUSE_CU)
      qIndex = -abs(qIndex);   // abs(), because it could be called by EncCu::xReuseCachedResult()
#else
      qIndex = -qIndex;
#endif
      TCoeff &qCoeff = qCoefffs.at(signPos);
#if (REUSE_CU_RESULTS && !SKIP_SIGN_PREDICTION_IN_REUSE_CU)
      qCoeff = -abs(qCoeff);   // abs(), because it could be called by EncCu::xReuseCachedResult()
#else
      qCoeff = -qCoeff;
#endif
      base += signMask;
    }
  }
  return true;
}

#undef SKIP_SIGN_PREDICTION_IN_REUSE_CU

int TrQuant::xGetUniTransformIdx(const TransformUnit &tu, CompID compID)
{
  int trIdx = to_underlying(TransType::NUM) * to_underlying(TransType::NUM);
  if (!tu.checkNSTApplied(compID))
  {
    // separable transform
    TransType trHor, trVer;
    getTrTypes(tu, compID, trHor, trVer);
    trIdx = to_underlying(trHor) * to_underlying(TransType::NUM) + to_underlying(trVer);
  }
  else
  {
    // non-separable transform
    constexpr int   scaleNstIdx    = NUM_LFNST_SETS;
    constexpr int   scaleTranspose = NUM_LFNST_SETS * (NUM_LFNST_NUM_PER_SET - 1);
    constexpr int   scaleNstBucket = NUM_LFNST_SETS * (NUM_LFNST_NUM_PER_SET - 1) * 2;
    const CompArea &area           = tu.blocks[compID];
    const int       nstIdx         = TU::getNstIdx(tu, compID) - 1u;
    const uint32_t  intraMode      = PU::getFinalIntraModesTrafo(tu, compID).first;
    CHECK(nstIdx < 0, "Invalid nst idx");
    CHECK(nstIdx >= NUM_LFNST_NUM_PER_SET - 1, "Invalid nst idx");
    if (TU::isNSPTAllowed(area.width, area.height))
    {
      trIdx += PU::getNSPTBucket(tu) * scaleNstBucket + g_nsptLut[intraMode];
    }
    else
    {
      trIdx += g_lfnstLut[intraMode];
    }
    if (getTransposeFlag(intraMode))
    {
      trIdx += scaleTranspose;
    }
    trIdx += nstIdx * scaleNstIdx;
  }
  return trIdx;
}

//! \}
