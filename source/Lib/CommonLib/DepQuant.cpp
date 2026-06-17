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

#include "DepQuant.h"
#include "TrQuant.h"
#include "CodingStructure.h"
#include "UnitTools.h"

#include <bitset>

#include "ContextModelling.h"

#if ENABLE_SIMD_OPT_QUANT
#include "CommonDefX86.h"
#endif

namespace DQIntern
{
  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   R A T E   E S T I M A T O R                                        =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

struct NbInfoSbb
{
  uint8_t numInv;
  uint8_t invInPos[5];
};
struct NbInfoOut
{
  uint16_t maxDist;
  uint16_t num;
  uint16_t outPos[5];
};
struct CoeffFracBits
{
  int32_t bits[GTN + 3];
};

enum ScanPosType
{
  SCAN_ISCSBB = 0,
  SCAN_SOCSBB = 1,
  SCAN_EOCSBB = 2
};

struct ScanInfo
{
  ScanInfo() {}
  int         sbbSize;
  int         numSbb;
  int         scanIdx;
  int         rasterPos;
  int         sbbPos;
  int         insidePos;
  bool        eosbb;
  ScanPosType spt;
  unsigned    sigCtxOffsetNext;
  unsigned    gtxCtxOffsetNext;
  int         nextInsidePos;
  NbInfoSbb   currNbInfoSbb;
  int         nextSbbRight;
  int         nextSbbBelow;
  int         posX;
  int         posY;
  ChannelType chType;
  int         sbtInfo;
  int         tuWidth;
  int         tuHeight;
};

class Rom;
struct TUParameters
{
  TUParameters(const Rom &rom, const unsigned width, const unsigned height, const ChannelType chType, const int isNst);
  ~TUParameters() { delete[] m_scanInfo; }

  ChannelType        m_chType;
  unsigned           m_width;
  unsigned           m_height;
  unsigned           m_numCoeff;
  unsigned           m_numSbb;
  unsigned           m_log2SbbWidth;
  unsigned           m_log2SbbHeight;
  unsigned           m_log2SbbSize;
  unsigned           m_sbbSize;
  unsigned           m_sbbMask;
  unsigned           m_widthInSbb;
  unsigned           m_heightInSbb;
  const ScanElement *m_scanSbbId2SbbPos;
  const ScanElement *m_scanId2BlkPos;
  const NbInfoSbb   *m_scanId2NbInfoSbb;
  const NbInfoOut   *m_scanId2NbInfoOut;
  ScanInfo          *m_scanInfo;

private:
  void xSetScanInfo(ScanInfo &scanInfo, int scanIdx);
};

class Rom
{
public:
  Rom() : m_scansInitialized(false) {}
  ~Rom() { xUninitScanArrays(); }
  void                init() { xInitScanArrays(); }
  const NbInfoSbb    *getNbInfoSbb(int hd, int vd, int isNst) const { return m_scanId2NbInfoSbbArray[hd][vd][isNst]; }
  const NbInfoOut    *getNbInfoOut(int hd, int vd, int isNst) const { return m_scanId2NbInfoOutArray[hd][vd][isNst]; }
  const TUParameters *getTUPars(const CompArea &area, const CompID compID, const int nstIdx) const
  {
    return m_tuParameters[floorLog2(area.width)][floorLog2(area.height)][to_underlying(toChannelType(compID))]
                         [nstIdx ? 1 : 0];
  }

private:
  void xInitScanArrays();
  void xUninitScanArrays();

private:
  bool          m_scansInitialized;
  NbInfoSbb    *m_scanId2NbInfoSbbArray[MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1][2];
  NbInfoOut    *m_scanId2NbInfoOutArray[MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1][2];
  TUParameters *m_tuParameters[MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1][MAX_NUM_CHANNEL_TYPE][2];
};

void Rom::xInitScanArrays()
{
  if (m_scansInitialized)
  {
    return;
  }
  ::memset(m_scanId2NbInfoSbbArray, 0, sizeof(m_scanId2NbInfoSbbArray));
  ::memset(m_scanId2NbInfoOutArray, 0, sizeof(m_scanId2NbInfoOutArray));
  ::memset(m_tuParameters, 0, sizeof(m_tuParameters));

  uint32_t raster2id[MAX_CU_SIZE * MAX_CU_SIZE];
  ::memset(raster2id, 0, sizeof(raster2id));

  for (int nst = 0; nst <= 1; nst++)
  {
    for (int hd = 0; hd <= MAX_CU_DEPTH; hd++)
    {
      for (int vd = 0; vd <= MAX_CU_DEPTH; vd++)
      {
        if ((hd == 0 && vd <= 1) || (hd <= 1 && vd == 0))
        {
          continue;
        }
        const uint32_t     blockWidth   = (1 << hd);
        const uint32_t     blockHeight  = (1 << vd);
        const uint32_t     log2CGWidth  = g_log2TxSubblockSize[hd][vd].width;
        const uint32_t     log2CGHeight = g_log2TxSubblockSize[hd][vd].height;
        const uint32_t     groupWidth   = 1 << log2CGWidth;
        const uint32_t     groupHeight  = 1 << log2CGHeight;
        const uint32_t     groupSize    = groupWidth * groupHeight;
        const SizeType     blkWidthIdx  = gp_sizeIdxInfo->idxFrom(blockWidth);
        const SizeType     blkHeightIdx = gp_sizeIdxInfo->idxFrom(blockHeight);
        const ScanElement *scanId2RP    = g_scanOrder[SCAN_GROUPED_4x4][CoeffScanType::DIAG][blkWidthIdx][blkHeightIdx];
        NbInfoSbb        *&sId2NbSbb    = m_scanId2NbInfoSbbArray[hd][vd][nst];
        NbInfoOut        *&sId2NbOut    = m_scanId2NbInfoOutArray[hd][vd][nst];
          // consider only non-zero-out region
        const uint32_t     blkWidthNZOut  = getNonzeroTuSize(blockWidth);
        const uint32_t     blkHeightNZOut = getNonzeroTuSize(blockHeight);
        const uint32_t     totalValues    = blkWidthNZOut * blkHeightNZOut;

        sId2NbSbb = new NbInfoSbb[totalValues];
        sId2NbOut = new NbInfoOut[totalValues];

        for (uint32_t scanId = 0; scanId < totalValues; scanId++)
        {
          raster2id[scanId2RP[scanId].idx] = scanId;
          sId2NbSbb[scanId].numInv         = 0;
        }

        for (unsigned scanId = 0; scanId < totalValues; scanId++)
        {
          const int posX = scanId2RP[scanId].x;
          const int posY = scanId2RP[scanId].y;
          const int rpos = scanId2RP[scanId].idx;
          {
              //===== inside subband neighbours =====
            const int begSbb = scanId - (scanId & (groupSize - 1)); // first pos in current subblock
            int       cpos[5];

            if (nst)
            {
              for (int k = 0; k < 5; k++)
              {
                const uint32_t rId = scanId + k + 1;
                cpos[k]            = (rId < totalValues) && (rId < groupSize + begSbb) ? rId - begSbb : 0;
              }
            }
            else
            {
              cpos[0] = (posX + 1 < blkWidthNZOut
                           ? (raster2id[rpos + 1] < groupSize + begSbb ? raster2id[rpos + 1] - begSbb : 0)
                           : 0);
              cpos[1] = (posX + 2 < blkWidthNZOut
                           ? (raster2id[rpos + 2] < groupSize + begSbb ? raster2id[rpos + 2] - begSbb : 0)
                           : 0);
              cpos[2] =
                (posX + 1 < blkWidthNZOut && posY + 1 < blkHeightNZOut
                   ? (raster2id[rpos + 1 + blockWidth] < groupSize + begSbb ? raster2id[rpos + 1 + blockWidth] - begSbb
                                                                            : 0)
                   : 0);
              cpos[3] =
                (posY + 1 < blkHeightNZOut
                   ? (raster2id[rpos + blockWidth] < groupSize + begSbb ? raster2id[rpos + blockWidth] - begSbb : 0)
                   : 0);
              cpos[4] = (posY + 2 < blkHeightNZOut ? (raster2id[rpos + 2 * blockWidth] < groupSize + begSbb
                                                        ? raster2id[rpos + 2 * blockWidth] - begSbb
                                                        : 0)
                                                   : 0);
            }

            int num      = 0;
            int inPos[5] = {
              0,
            };

            while (true)
            {
              int nk = -1;
              for (int k = 0; k < 5; k++)
              {
                if (cpos[k] != 0 && (nk < 0 || cpos[k] < cpos[nk]))
                {
                  nk = k;
                }
              }
              if (nk < 0)
              {
                break;
              }
              inPos[num++] = uint8_t(cpos[nk]);
              cpos[nk]     = 0;
            }
            for (int k = num; k < 5; k++)
            {
              inPos[k] = 0;
            }
            for (int k = 0; k < num; k++)
            {
              CHECK(sId2NbSbb[begSbb + inPos[k]].numInv >= 5, "");
              sId2NbSbb[begSbb + inPos[k]].invInPos[sId2NbSbb[begSbb + inPos[k]].numInv++] = scanId & (groupSize - 1);
            }
          }
          {
              //===== outside subband neighbours =====
            NbInfoOut &nbOut  = sId2NbOut[scanId];
            const int  begSbb = scanId - (scanId & (groupSize - 1)); // first pos in current subblock
            int        cpos[5];

            if (nst)
            {
              for (int k = 0; k < 5; k++)
              {
                const uint32_t rId = scanId + k + 1;
                cpos[k]            = (rId < totalValues) && (rId >= groupSize + begSbb) ? rId : 0;
              }
            }
            else
            {
              cpos[0] =
                (posX + 1 < blkWidthNZOut ? (raster2id[rpos + 1] >= groupSize + begSbb ? raster2id[rpos + 1] : 0) : 0);
              cpos[1] =
                (posX + 2 < blkWidthNZOut ? (raster2id[rpos + 2] >= groupSize + begSbb ? raster2id[rpos + 2] : 0) : 0);
              cpos[2] =
                (posX + 1 < blkWidthNZOut && posY + 1 < blkHeightNZOut
                   ? (raster2id[rpos + 1 + blockWidth] >= groupSize + begSbb ? raster2id[rpos + 1 + blockWidth] : 0)
                   : 0);
              cpos[3] = (posY + 1 < blkHeightNZOut
                           ? (raster2id[rpos + blockWidth] >= groupSize + begSbb ? raster2id[rpos + blockWidth] : 0)
                           : 0);
              cpos[4] =
                (posY + 2 < blkHeightNZOut
                   ? (raster2id[rpos + 2 * blockWidth] >= groupSize + begSbb ? raster2id[rpos + 2 * blockWidth] : 0)
                   : 0);
            }

            for (nbOut.num = 0; true;)
            {
              int nk = -1;
              for (int k = 0; k < 5; k++)
              {
                if (cpos[k] != 0 && (nk < 0 || cpos[k] < cpos[nk]))
                {
                  nk = k;
                }
              }
              if (nk < 0)
              {
                break;
              }
              nbOut.outPos[nbOut.num++] = uint16_t(cpos[nk]);
              cpos[nk]                  = 0;
            }
            for (int k = nbOut.num; k < 5; k++)
            {
              nbOut.outPos[k] = 0;
            }
            nbOut.maxDist = (scanId == 0 ? 0 : sId2NbOut[scanId - 1].maxDist);
            for (int k = 0; k < nbOut.num; k++)
            {
              if (nbOut.outPos[k] > nbOut.maxDist)
              {
                nbOut.maxDist = nbOut.outPos[k];
              }
            }
          }
        }

          // make it relative
        for (unsigned scanId = 0; scanId < totalValues; scanId++)
        {
          NbInfoOut &nbOut  = sId2NbOut[scanId];
          const int  begSbb = scanId - (scanId & (groupSize - 1)); // first pos in current subblock
          for (int k = 0; k < nbOut.num; k++)
          {
            CHECK(begSbb > nbOut.outPos[k], "Position must be past sub block begin");
            nbOut.outPos[k] -= begSbb;
          }
          nbOut.maxDist -= scanId;
        }

        for (int chId = 0; chId < MAX_NUM_CHANNEL_TYPE; chId++)
        {
          m_tuParameters[hd][vd][chId][nst] = new TUParameters(*this, blockWidth, blockHeight, ChannelType(chId), nst);
        }
      }
    }
  }
  m_scansInitialized = true;
}

void Rom::xUninitScanArrays()
{
  if (!m_scansInitialized)
  {
    return;
  }
  for (int nst = 0; nst <= 1; nst++)
  {
    for (int hd = 0; hd <= MAX_CU_DEPTH; hd++)
    {
      for (int vd = 0; vd <= MAX_CU_DEPTH; vd++)
      {
        NbInfoSbb *&sId2NbSbb = m_scanId2NbInfoSbbArray[hd][vd][nst];
        NbInfoOut *&sId2NbOut = m_scanId2NbInfoOutArray[hd][vd][nst];
        if (sId2NbSbb)
        {
          delete[] sId2NbSbb;
        }
        if (sId2NbOut)
        {
          delete[] sId2NbOut;
        }
        for (int chId = 0; chId < MAX_NUM_CHANNEL_TYPE; chId++)
        {
          TUParameters *&tuPars = m_tuParameters[hd][vd][chId][nst];
          if (tuPars)
          {
            delete tuPars;
          }
        }
      }
    }
  }
  m_scansInitialized = false;
}

static Rom g_Rom;

TUParameters::TUParameters(const Rom &rom, const unsigned width, const unsigned height, const ChannelType chType,
                           const int isNst)
{
  m_chType                     = chType;
  m_width                      = width;
  m_height                     = height;
  const uint32_t nonzeroWidth  = getNonzeroTuSize(m_width);
  const uint32_t nonzeroHeight = getNonzeroTuSize(m_height);
  m_numCoeff                   = nonzeroWidth * nonzeroHeight;

  const int log2W = floorLog2(m_width);
  const int log2H = floorLog2(m_height);

  m_log2SbbWidth  = g_log2TxSubblockSize[log2W][log2H].width;
  m_log2SbbHeight = g_log2TxSubblockSize[log2W][log2H].height;
  m_log2SbbSize   = m_log2SbbWidth + m_log2SbbHeight;
  m_sbbSize       = 1 << m_log2SbbSize;
  m_sbbMask       = m_sbbSize - 1;
  m_widthInSbb    = nonzeroWidth >> m_log2SbbWidth;
  m_heightInSbb   = nonzeroHeight >> m_log2SbbHeight;
  m_numSbb        = m_widthInSbb * m_heightInSbb;

  SizeType hsbb      = gp_sizeIdxInfo->idxFrom(m_widthInSbb);
  SizeType vsbb      = gp_sizeIdxInfo->idxFrom(m_heightInSbb);
  SizeType hsId      = gp_sizeIdxInfo->idxFrom(m_width);
  SizeType vsId      = gp_sizeIdxInfo->idxFrom(m_height);
  m_scanSbbId2SbbPos = g_scanOrder[SCAN_UNGROUPED][CoeffScanType::DIAG][hsbb][vsbb];
  m_scanId2BlkPos    = g_scanOrder[SCAN_GROUPED_4x4][CoeffScanType::DIAG][hsId][vsId];
  m_scanId2NbInfoSbb = rom.getNbInfoSbb(log2W, log2H, isNst);
  m_scanId2NbInfoOut = rom.getNbInfoOut(log2W, log2H, isNst);
  m_scanInfo         = new ScanInfo[m_numCoeff];
  for (int scanIdx = 0; scanIdx < m_numCoeff; scanIdx++)
  {
    xSetScanInfo(m_scanInfo[scanIdx], scanIdx);
  }
}

void TUParameters::xSetScanInfo(ScanInfo &scanInfo, int scanIdx)
{
  scanInfo.chType    = m_chType;
  scanInfo.tuWidth   = m_width;
  scanInfo.tuHeight  = m_height;
  scanInfo.sbbSize   = m_sbbSize;
  scanInfo.numSbb    = m_numSbb;
  scanInfo.scanIdx   = scanIdx;
  scanInfo.rasterPos = m_scanId2BlkPos[scanIdx].idx;
  scanInfo.sbbPos    = m_scanSbbId2SbbPos[scanIdx >> m_log2SbbSize].idx;
  scanInfo.insidePos = scanIdx & m_sbbMask;
  scanInfo.eosbb     = (scanInfo.insidePos == 0);
  scanInfo.spt       = SCAN_ISCSBB;
  if (scanInfo.insidePos == m_sbbMask && scanIdx > scanInfo.sbbSize && scanIdx < m_numCoeff - 1)
  {
    scanInfo.spt = SCAN_SOCSBB;
  }
  else if (scanInfo.eosbb && scanIdx > 0 && scanIdx < m_numCoeff - m_sbbSize)
  {
    scanInfo.spt = SCAN_EOCSBB;
  }
  scanInfo.posX = m_scanId2BlkPos[scanIdx].x;
  scanInfo.posY = m_scanId2BlkPos[scanIdx].y;
  if (scanIdx)
  {
    const int nextScanIdx = scanIdx - 1;
    const int diag        = m_scanId2BlkPos[nextScanIdx].x + m_scanId2BlkPos[nextScanIdx].y;
    if (isLuma(m_chType))
    {
      scanInfo.sigCtxOffsetNext = (diag < 2 ? NSIGCTX * 2 : diag < 5 ? NSIGCTX : 0);
      scanInfo.gtxCtxOffsetNext = (diag < 1      ? 1 + (NGTXCTX * 3)
                                     : diag < 3  ? 1 + (NGTXCTX * 2)
                                     : diag < 10 ? 1 + NGTXCTX
                                                 : 1);
    }
    else
    {
      scanInfo.sigCtxOffsetNext = (diag < 2 ? NSIGCTX : 0);
      scanInfo.gtxCtxOffsetNext = (diag < 1 ? 1 + NGTXCTX : 1);
    }
    scanInfo.nextInsidePos = nextScanIdx & m_sbbMask;
    scanInfo.currNbInfoSbb = m_scanId2NbInfoSbb[scanIdx];
    if (scanInfo.eosbb)
    {
      const int nextSbbPos  = m_scanSbbId2SbbPos[nextScanIdx >> m_log2SbbSize].idx;
      const int nextSbbPosY = nextSbbPos / m_widthInSbb;
      const int nextSbbPosX = nextSbbPos - nextSbbPosY * m_widthInSbb;
      scanInfo.nextSbbRight = (nextSbbPosX < m_widthInSbb - 1 ? nextSbbPos + 1 : 0);
      scanInfo.nextSbbBelow = (nextSbbPosY < m_heightInSbb - 1 ? nextSbbPos + m_widthInSbb : 0);
    }
  }
}

class RateEstimator
{
public:
  RateEstimator() {}
  ~RateEstimator() {}
  void initCtx(const TUParameters &tuPars, const TransformUnit &tu, const CompID compID,
               const FracBitsAccess &fracBitsAccess);

  inline const BinFracBits *sigSbbFracBits() const { return m_sigSbbFracBits; }
  inline const BinFracBits *sigFlagBits(unsigned stateId) const
  {
    return m_sigFracBits[(stateId & 1) ? 1 + ((stateId & 3) >> 1) : 0];
  }
  inline const CoeffFracBits *gtxFracBits(unsigned stateId) const { return m_gtxFracBits; }
  inline int32_t              lastOffset(unsigned scanIdx, int effWidth, int effHeight, bool reverseLast) const
  {
    if (reverseLast)
    {
      return m_lastBitsX[effWidth - 1 - m_scanId2Pos[scanIdx].x] + m_lastBitsY[effHeight - 1 - m_scanId2Pos[scanIdx].y];
    }
    else
    {
      return m_lastBitsX[m_scanId2Pos[scanIdx].x] + m_lastBitsY[m_scanId2Pos[scanIdx].y];
    }
  }

private:
  void xSetLastCoeffOffset(const FracBitsAccess &fracBitsAccess, const TUParameters &tuPars, const TransformUnit &tu,
                           const CompID compID);
  void xSetSigSbbFracBits(const FracBitsAccess &fracBitsAccess, ChannelType chType);
  void xSetSigFlagBits(const FracBitsAccess &fracBitsAccess, ChannelType chType, const bool isNST);
  void xSetGtxFlagBits(const FracBitsAccess &fracBitsAccess, ChannelType chType, const bool isNST);

private:
  static const unsigned sm_numCtxSetsSig   = 3;
  static const unsigned sm_numCtxSetsGtx   = 2;
  static const unsigned sm_maxNumSigSbbCtx = 2;
  static const unsigned sm_maxNumSigCtx    = NSIGCTX * 3;
  static const unsigned sm_maxNumGtxCtx    = 1 + NGTXCTX * 4;

private:
  const ScanElement *m_scanId2Pos;
  bool               m_condition = false;
  SliceType          m_sliceType = NUMBER_OF_SLICE_TYPES;
  int32_t            m_lastBitsX[MAX_TB_SIZEY];
  int32_t            m_lastBitsY[MAX_TB_SIZEY];
  BinFracBits        m_sigSbbFracBits[sm_maxNumSigSbbCtx];
  BinFracBits        m_sigFracBits[sm_numCtxSetsSig][sm_maxNumSigCtx];
  CoeffFracBits      m_gtxFracBits[sm_maxNumGtxCtx];
};

void RateEstimator::initCtx(const TUParameters &tuPars, const TransformUnit &tu, const CompID compID,
                            const FracBitsAccess &fracBitsAccess)
{
  CHECK(tuPars.m_chType != toChannelType(compID), "not supported");
  m_condition  = CoeffCodingContext::getSwitchCondition(*tu.cu, tuPars.m_chType);
  m_sliceType  = tu.cu->slice->m_eSliceType;
  m_scanId2Pos = tuPars.m_scanId2BlkPos;
  xSetSigSbbFracBits(fracBitsAccess, tuPars.m_chType);
  const bool isNST = TU::getNstIdx(tu, compID);
  xSetSigFlagBits(fracBitsAccess, tuPars.m_chType, isNST);
  xSetGtxFlagBits(fracBitsAccess, tuPars.m_chType, isNST);
  xSetLastCoeffOffset(fracBitsAccess, tuPars, tu, compID);
}

void RateEstimator::xSetLastCoeffOffset(const FracBitsAccess &fracBitsAccess, const TUParameters &tuPars,
                                        const TransformUnit &tu, const CompID compID)
{
  const ChannelType chType = toChannelType(compID);

  BinFracBits bits = { 0, 0 };

  if (isLuma(chType) && !CU::isIntra(*tu.cu) && !tu.depth)
  {
    bits = fracBitsAccess.getFracBitsArray(Ctx::QtRootCbf());
  }
  else
  {
    bits = fracBitsAccess.getFracBitsArray(Ctx::QtCbf[compID](DeriveCtx::CtxQtCbf(compID, tu.cbf[COMP_Cb])));
  }

  const int32_t cbfDeltaBits = int32_t(bits.intBits[1]) - int32_t(bits.intBits[0]);

  for (int xy = 0; xy < 2; xy++)
  {
    const bool isY = xy != 0;

    const unsigned size       = isY ? tuPars.m_height : tuPars.m_width;
    const int      log2Size   = floorLog2(size);
    const CtxSet  &ctxSetLast = (m_condition ? (isY ? Ctx::LastYCtxSetSwitch : Ctx::LastXCtxSetSwitch)
                                             : (isY ? Ctx::LastY : Ctx::LastX))[to_underlying(chType)];
    const unsigned lastShift  = isLuma(chType) ? (log2Size + 1) >> 2 : Clip3<unsigned>(0, 2, size >> 3);
    const unsigned lastOffset = isLuma(chType) ? CoeffCodingContext::prefixCtx[log2Size] : 0;
    const int      nzSize     = getNonzeroTuSize(size);
    const int      maxCtxId   = g_groupIdx[nzSize - 1];
    const bool     maxCoded   = (log2Size >= LOG2_SECONDARY_PREFIX_START_SIZE);

    std::array<int32_t, LAST_SIGNIFICANT_GROUPS> prefixBits;
    int                                          sumPrefixBits = isY ? cbfDeltaBits : 0;
    for (int ctxId = 0; ctxId <= maxCtxId; ctxId++)
    {
      prefixBits[ctxId] = sumPrefixBits;
      if (ctxId < maxCtxId)
      {
        const BinFracBits bits = fracBitsAccess.getFracBitsArray(ctxSetLast(lastOffset + (ctxId >> lastShift)));
        prefixBits[ctxId] += bits.intBits[0];
        sumPrefixBits += bits.intBits[1];
      }
    }
    const CtxSet     &ctxMax      = (isY ? Ctx::lastYSecondaryPrefix : Ctx::lastXSecondaryPrefix);
    const CtxSet     &ctxSuf      = (isY ? Ctx::lastYSuffix[compID] : Ctx::lastXSuffix[compID]);
    const int         ctxIdMax    = (compID == CompID::COMP_Y ? std::max(0, log2Size - LOG2_SECONDARY_PREFIX_START_SIZE)
                                                              : 5 + to_underlying(compID) - 1);
    const int         ctxIdSuf    = std::max(0, log2Size - LOG2_ID_SUFFIX_CTX_START_SIZE);
    const BinFracBits bitsMax     = fracBitsAccess.getFracBitsArray(ctxMax(ctxIdMax));
    const BinFracBits bitsSuf     = fracBitsAccess.getFracBitsArray(ctxSuf(ctxIdSuf));
    const int         maxGrpNoMax = (maxCoded ? maxCtxId - 1 : maxCtxId);
    const int         maxGrpNoSufCtx  = std::min(5, maxGrpNoMax);
    const int         maxGrpSinglePos = std::min(3, maxGrpNoMax);
    int32_t          *lastBits        = (isY ? m_lastBitsY : m_lastBitsX);
    for (int grpId = 0; grpId <= maxGrpSinglePos; grpId++)
    {
      lastBits[grpId] = prefixBits[grpId];
    }
    for (int grpId = maxGrpSinglePos + 1; grpId <= maxGrpNoSufCtx; grpId++)
    {
      const int log2Size = (grpId - 2) >> 1;
      const int posBeg   = g_minInGroup[grpId];
      const int posEnd   = posBeg + (1 << log2Size);
      for (int pos = posBeg; pos < posEnd; pos++)
      {
        lastBits[pos] = prefixBits[grpId] + (log2Size << SCALE_BITS);
      }
    }
    for (int grpId = maxGrpNoSufCtx + 1; grpId <= maxGrpNoMax; grpId++)
    {
      const int log2Half = ((grpId - 2) >> 1) - 1;
      const int halfSize = 1 << log2Half;
      const int posBeg   = g_minInGroup[grpId];
      const int posMid   = posBeg + halfSize;
      const int posEnd   = posMid + halfSize;
      const int bitsGrp  = prefixBits[grpId] + (log2Half << SCALE_BITS);
      const int bits1st  = bitsGrp + bitsSuf.intBits[0];
      const int bits2nd  = bitsGrp + bitsSuf.intBits[1];
      for (int pos = posBeg; pos < posMid; pos++)
      {
        lastBits[pos] = bits1st;
      }
      for (int pos = posMid; pos < posEnd; pos++)
      {
        lastBits[pos] = bits2nd;
      }
    }
    if (maxCtxId > maxGrpNoMax)
    {
      if (maxCtxId > 5)
      {
        const int log2Half = ((maxCtxId - 2) >> 1) - 1;
        const int halfSize = 1 << log2Half;
        const int posBeg   = g_minInGroup[maxCtxId];
        const int posMid   = posBeg + halfSize;
        const int posEnd   = posMid + halfSize;
        const int bitsGrp  = prefixBits[maxCtxId] + (log2Half << SCALE_BITS) + bitsMax.intBits[0];
        const int bits1st  = bitsGrp + bitsSuf.intBits[0];
        const int bits2nd  = bitsGrp + bitsSuf.intBits[1];
        for (int pos = posBeg; pos < posMid; pos++)
        {
          lastBits[pos] = bits1st;
        }
        for (int pos = posMid; pos < posEnd - 2; pos++)
        {
          lastBits[pos] = bits2nd;
        }
        lastBits[posEnd - 2] = bits2nd - (1 << SCALE_BITS);
        lastBits[posEnd - 1] = prefixBits[maxCtxId] + bitsMax.intBits[1];
      }
      else
      {
        const int posBeg     = g_minInGroup[maxCtxId];
        lastBits[posBeg]     = prefixBits[maxCtxId] + bitsMax.intBits[0];
        lastBits[posBeg + 1] = prefixBits[maxCtxId] + bitsMax.intBits[1];
      }
    }
  }
}

void RateEstimator::xSetSigSbbFracBits(const FracBitsAccess &fracBitsAccess, ChannelType chType)
{
  const CtxSet &ctxSet = (m_condition ? Ctx::SigCoeffGroupCtxSetSwitch : Ctx::SigCoeffGroup)[to_underlying(chType)];
  for (unsigned ctxId = 0; ctxId < sm_maxNumSigSbbCtx; ctxId++)
  {
    m_sigSbbFracBits[ctxId] = fracBitsAccess.getFracBitsArray(ctxSet(ctxId));
  }
}

void RateEstimator::xSetSigFlagBits(const FracBitsAccess &fracBitsAccess, ChannelType chType, const bool isNST)
{
  for (unsigned ctxSetId = 0; ctxSetId < sm_numCtxSetsSig; ctxSetId++)
  {
    BinFracBits   *bits   = m_sigFracBits[ctxSetId];
    const unsigned ctxCh  = 2 * ctxSetId + to_underlying(chType);
    const CtxSet  &ctxSet = (isNST ? Ctx::SigFlagNST : (m_condition ? Ctx::SigFlagCtxSetSwitch : Ctx::SigFlag))[ctxCh];
    const unsigned numCtx = (isLuma(chType) ? NSIGCTX * 3 : NSIGCTX * 2);
    for (unsigned ctxId = 0; ctxId < numCtx; ctxId++)
    {
      bits[ctxId] = fracBitsAccess.getFracBitsArray(ctxSet(ctxId));
    }
  }
}

void RateEstimator::xSetGtxFlagBits(const FracBitsAccess &fracBitsAccess, const ChannelType chType, const bool isNST)
{
  const auto    chIdx = to_underlying(chType);
  const CtxSet &ctxSetGt1 =
    (isNST ? Ctx::GtxFlagNST : (m_condition ? Ctx::GtxFlagCtxSetSwitch : Ctx::GtxFlag))[chIdx + 2];
  const CtxSet &ctxSetGt2 =
    (isNST ? Ctx::GtxFlagNST : (m_condition ? Ctx::GtxFlagCtxSetSwitch : Ctx::GtxFlag))[chIdx + 4];
  const CtxSet &ctxSetGt3 = (isNST ? Ctx::GtxFlagNST : (m_condition ? Ctx::GtxFlagCtxSetSwitch : Ctx::GtxFlag))[chIdx];
  const CtxSet &ctxSetGt4 =
    (isNST ? Ctx::GtxFlagNST : (m_condition ? Ctx::GtxFlagCtxSetSwitch : Ctx::GtxFlag))[chIdx + 6];
  const unsigned numCtx = (isLuma(chType) ? 1 + NGTXCTX * 4 : 1 + NGTXCTX * 2);
  for (unsigned ctxId = 0; ctxId < numCtx; ctxId++)
  {
    BinFracBits    fbGt1 = fracBitsAccess.getFracBitsArray(ctxSetGt1(ctxId));
    BinFracBits    fbGt2 = fracBitsAccess.getFracBitsArray(ctxSetGt2(ctxId));
    BinFracBits    fbGt3 = fracBitsAccess.getFracBitsArray(ctxSetGt3(ctxId));
    BinFracBits    fbGtN = fracBitsAccess.getFracBitsArray(ctxSetGt4(ctxId));
    int32_t        gt1   = (1 << SCALE_BITS) + int32_t(fbGt1.intBits[1]);
    int32_t        gt2   = gt1 + fbGt2.intBits[1];
    int32_t        gt3   = gt2 + fbGt3.intBits[1];
    int32_t        gtN0  = fbGtN.intBits[0];
    int32_t        gtN1  = fbGtN.intBits[1];
    CoeffFracBits &cb    = m_gtxFracBits[ctxId];
    cb.bits[0]           = 0;
    cb.bits[1]           = fbGt1.intBits[0] + (1 << SCALE_BITS);
    cb.bits[2]           = gt1 + fbGt2.intBits[0];
    cb.bits[3]           = gt2 + fbGt3.intBits[0];
    cb.bits[4]           = gt3 + gtN0;
    cb.bits[GTN + 2]     = gt3 + gtN1 * (GTN - 3) + (1 << SCALE_BITS);
    cb.bits[GTN + 1]     = gt3 + gtN1 * (GTN - 3) + (1 << SCALE_BITS);
    int diff             = GTN - 4;
    for (int i = 0; diff > 0; i++, diff--)
    {
      cb.bits[GTN - i] = gt3 + gtN1 * diff + gtN0;
    }
  }
}

  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   D A T A   S T R U C T U R E S                                      =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

struct PQData
{
  TCoeff  absLevel;
  int64_t deltaDist;
};

struct Decision
{
  int64_t rdCost;
  TCoeff  absLevel;
  int     prevId;
};

  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   P R E - Q U A N T I Z E R                                          =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

class Quantizer
{
public:
  Quantizer() {}
  void           dequantBlock(const TransformUnit &tu, const CompID compID, const QpParam &cQP, CoeffBuf &recCoeff,
                              bool enableScalingLists, int *piDequantCoef, const uint64_t stateTransTab) const;
  void           initQuantBlock(const TransformUnit &tu, const CompID compID, const QpParam &cQP, const double lambda,
                                int gValue);
  inline bool    preQuantAbsTCQ(const TCoeff absCoeff, PQData *pqData, TCoeff quanCoeff) const;
  inline bool    preQuantAbsSQ(const TCoeff absCoeff, PQData *pqData, TCoeff quanCoeff) const;
  inline TCoeff  getLastThreshold() const { return m_thresLast; }
  inline int64_t getQScale() const { return m_QScale; }

private:
    // quantization
  int     m_QShift;
  int64_t m_QAdd;
  int64_t m_QScale;
  TCoeff  m_maxQIdx;
  TCoeff  m_thresLast;
    // distortion normalization
  int     m_DistShift;
  int64_t m_DistAdd;
  int64_t m_DistStepAdd;
  int64_t m_DistOrgFact;
};

inline int ceil_log2(uint64_t x)
{
  static const uint64_t t[6] = { 0xFFFFFFFF00000000ull, 0x00000000FFFF0000ull, 0x000000000000FF00ull,
                                 0x00000000000000F0ull, 0x000000000000000Cull, 0x0000000000000002ull };
  int                   y    = (((x & (x - 1)) == 0) ? 0 : 1);
  int                   j    = 32;
  for (int i = 0; i < 6; i++)
  {
    int k = (((x & t[i]) == 0) ? 0 : j);
    y += k;
    x >>= k;
    j >>= 1;
  }
  return y;
}
void Quantizer::initQuantBlock(const TransformUnit &tu, const CompID compID, const QpParam &cQP, const double lambda,
                               int gValue = -1)
{
  CHECKD(lambda <= 0.0, "Lambda must be greater than 0");

  const int         isTCQ                 = !!tu.cs->slice->m_depQuantEnabledIdc;
  const int         qpDQ                  = cQP.Qp(tu.mtsIdx[compID] == MtsType::SKIP) + isTCQ;
  const int         qpPer                 = qpDQ / 6;
  const int         qpRem                 = qpDQ - 6 * qpPer;
  const SPS        &sps                   = *tu.cs->sps;
  const CompArea   &area                  = tu.blocks[compID];
  const ChannelType chType                = toChannelType(compID);
  const int         channelBitDepth       = sps.m_bitDepths[chType];
  const int         maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange(chType);
  const int         nomTransformShift     = getTransformShift(channelBitDepth, area.size(), maxLog2TrDynamicRange);

  const bool clipTransformShift =
    tu.mtsIdx[compID] == MtsType::SKIP && sps.m_spsRangeExtension.m_extendedPrecisionProcessingFlag;

  const bool needsSqrt2ScaleAdjustment = TU::needsSqrt2Scale(tu, compID);
  const int  transformShift = (clipTransformShift ? std::max<int>(0, nomTransformShift) : nomTransformShift) +
    (needsSqrt2ScaleAdjustment ? -1 : 0);
    // quant parameters
  if (isTCQ)
  {
    m_QShift     = QUANT_SHIFT - 1 + qpPer + transformShift;
    m_QAdd       = -((3 << m_QShift) >> 1);
    int invShift = IQUANT_SHIFT + 1 - qpPer - transformShift;
    m_QScale     = g_quantScales[needsSqrt2ScaleAdjustment ? 1 : 0][qpRem];
    const unsigned qIdxBD =
      std::min<unsigned>(maxLog2TrDynamicRange + 1, 8 * sizeof(Intermediate_Int) + invShift - IQUANT_SHIFT - 1);
    m_maxQIdx   = (1 << (qIdxBD - 1)) - 4;
    m_thresLast = TCoeff((int64_t(4) << m_QShift));
  }
  else
  {
    m_QShift     = QUANT_SHIFT + qpPer + transformShift;
    m_QAdd       = -((1 << m_QShift) >> 1);
    int invShift = IQUANT_SHIFT - qpPer - transformShift;
    m_QScale     = g_quantScales[needsSqrt2ScaleAdjustment ? 1 : 0][qpRem];
    const unsigned qIdxBD =
      std::min<unsigned>(maxLog2TrDynamicRange + 1, 8 * sizeof(Intermediate_Int) + invShift - IQUANT_SHIFT - 1);
    m_maxQIdx   = (1 << (qIdxBD - 1)) - 2;
    m_thresLast = TCoeff((int64_t(2) << m_QShift));
  }
    // distortion calculation parameters
  const int64_t qScale    = (gValue == -1) ? m_QScale : gValue;
  const int     nomDShift = SCALE_BITS - 2 * (nomTransformShift + DISTORTION_PRECISION_ADJUSTMENT(channelBitDepth)) +
    m_QShift + (needsSqrt2ScaleAdjustment ? 1 : 0);
  const double  qScale2       = double(qScale * qScale);
  const double  nomDistFactor = (nomDShift < 0 ? 1.0 / (double(int64_t(1) << (-nomDShift)) * qScale2 * lambda)
                                               : double(int64_t(1) << nomDShift) / (qScale2 * lambda));
  const int64_t pow2dfShift   = (int64_t)(nomDistFactor * qScale2) + 1;
  const int     dfShift       = ceil_log2(pow2dfShift);
  m_DistShift                 = 62 + m_QShift - 2 * maxLog2TrDynamicRange - dfShift;
  m_DistAdd                   = (int64_t(1) << m_DistShift) >> 1;
  m_DistStepAdd =
    (int64_t)(nomDistFactor * double(int64_t(1) << (m_DistShift)) * double(int64_t(1) << (m_QShift)) + .5);
  m_DistOrgFact = (int64_t)(nomDistFactor * double(int64_t(1) << (m_DistShift + 1)) + .5);
}

static const int coeffShiftArray[64] = { 0, 63, 31, 21, 15, 12, 10, 9, 7, 7, 6, 5, 5, 4, 4, 4, 3, 3, 3, 3, 3, 3,
                                         2, 2,  2,  2,  2,  2,  2,  2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                                         1, 1,  1,  1,  1,  1,  1,  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

void Quantizer::dequantBlock(const TransformUnit &tu, const CompID compID, const QpParam &cQP, CoeffBuf &recCoeff,
                             bool enableScalingLists, int *piDequantCoef, const uint64_t stateTransTab) const
{
    //----- set basic parameters -----
  const CompArea    &area     = tu.blocks[compID];
  const int          numCoeff = area.area();
  const SizeType     hsId     = gp_sizeIdxInfo->idxFrom(area.width);
  const SizeType     vsId     = gp_sizeIdxInfo->idxFrom(area.height);
  const ScanElement *scan     = g_scanOrder[SCAN_GROUPED_4x4][CoeffScanType::DIAG][hsId][vsId];
  const TCoeff      *qCoeff   = tu.getCoeffs(compID).buf;
  TCoeff            *tCoeff   = recCoeff.buf;

    //----- reset coefficients and get last scan index -----
  ::memset(tCoeff, 0, numCoeff * sizeof(TCoeff));
  int lastScanIdx = -1;
  for (int scanIdx = numCoeff - 1; scanIdx >= 0; scanIdx--)
  {
    if (qCoeff[scan[scanIdx].idx])
    {
      lastScanIdx = scanIdx;
      break;
    }
  }
  if (lastScanIdx < 0)
  {
    return;
  }

    //----- set dequant parameters -----
  const int         qpDQ                  = cQP.Qp(tu.mtsIdx[compID] == MtsType::SKIP) + 1;
  const int         qpPer                 = qpDQ / 6;
  const int         qpRem                 = qpDQ - 6 * qpPer;
  const SPS        &sps                   = *tu.cs->sps;
  const ChannelType chType                = toChannelType(compID);
  const int         channelBitDepth       = sps.m_bitDepths[chType];
  const int         maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange(chType);
  const TCoeff      minTCoeff             = -(1 << maxLog2TrDynamicRange);
  const TCoeff      maxTCoeff             = (1 << maxLog2TrDynamicRange) - 1;
  const int         nomTransformShift     = getTransformShift(channelBitDepth, area.size(), maxLog2TrDynamicRange);

  const bool clipTransformShift =
    tu.mtsIdx[compID] == MtsType::SKIP && sps.m_spsRangeExtension.m_extendedPrecisionProcessingFlag;

  const bool needsSqrt2ScaleAdjustment = TU::needsSqrt2Scale(tu, compID);
  const int  transformShift = (clipTransformShift ? std::max<int>(0, nomTransformShift) : nomTransformShift) +
    (needsSqrt2ScaleAdjustment ? -1 : 0);
  Intermediate_Int shift =
    IQUANT_SHIFT + 1 - qpPer - transformShift + (enableScalingLists ? LOG2_SCALING_LIST_NEUTRAL_VALUE : 0);
  Intermediate_Int invQScale = g_invQuantScales[needsSqrt2ScaleAdjustment ? 1 : 0][qpRem];
  Intermediate_Int add       = (shift < 0) ? 0 : ((1 << shift) >> 1);
    //----- dequant coefficients -----
  for (int state = 0, scanIdx = lastScanIdx; scanIdx >= 0; scanIdx--)
  {
    const unsigned rasterPos = scan[scanIdx].idx;
    const TCoeff  &level     = qCoeff[rasterPos];
    if (level)
    {
      if (enableScalingLists)
      {
        invQScale = piDequantCoef[rasterPos];// scalingfactor*levelScale
      }
      if (shift < 0 && (enableScalingLists || scanIdx == lastScanIdx))
      {
        invQScale <<= -shift;
      }
      const Intermediate_Int qIdx = 2 * level + (level > 0 ? -(state & 1) : (state & 1));
      CHECK(qIdx < minTCoeff || qIdx > maxTCoeff, "TransCoeffLevel outside allowable range");
      const Intermediate_Int qSgn      = qIdx < 0 ? -1 : 1;
      const Intermediate_Int qAbs      = qIdx * qSgn;
      int64_t                nomTCoeff = ((int64_t)qAbs * (int64_t)invQScale + add) >> ((shift < 0) ? 0 : shift);
      if (qAbs < 64)
      {
        const int     qAbs2      = qAbs + 1;
        const int64_t nomTCoeff2 = ((int64_t)qAbs2 * (int64_t)invQScale + add) >> ((shift < 0) ? 0 : shift);
        const int     coefShift  = coeffShiftArray[qAbs];
        nomTCoeff                = (int64_t)((1024 - coefShift) * nomTCoeff + coefShift * nomTCoeff2) >> 10;
      }
      tCoeff[rasterPos] = (TCoeff)Clip3<int64_t>(minTCoeff, maxTCoeff, qSgn * nomTCoeff);
    }
    state = int((stateTransTab >> ((state << 3) + ((level & 1) << 2))) & 15);
  }
}

inline bool Quantizer::preQuantAbsTCQ(const TCoeff absCoeff, PQData *pqData, TCoeff quanCoeff) const
{
  int64_t scaledOrg = int64_t(absCoeff) * quanCoeff;
  TCoeff  qIdx      = TCoeff((scaledOrg + m_QAdd) >> m_QShift);

  if (qIdx < 0)
  {
    int64_t scaledAdd = m_DistStepAdd - scaledOrg * m_DistOrgFact;
    PQData &pqA       = pqData[1];
    PQData &pqB       = pqData[2];

    pqA.deltaDist = ((scaledAdd + 0 * m_DistStepAdd) * 1 + m_DistAdd) >> m_DistShift;
    pqA.absLevel  = 1;

    pqB.deltaDist = ((scaledAdd + 1 * m_DistStepAdd) * 2 + m_DistAdd) >> m_DistShift;
    pqB.absLevel  = 1;

    return true;
  }

  qIdx              = std::max<TCoeff>(1, std::min<TCoeff>(m_maxQIdx, qIdx));
  int64_t scaledAdd = qIdx * m_DistStepAdd - scaledOrg * m_DistOrgFact;
  PQData &pqA       = pqData[qIdx & 3];
  pqA.deltaDist     = (scaledAdd * qIdx + m_DistAdd) >> m_DistShift;
  pqA.absLevel      = (++qIdx) >> 1;
  scaledAdd += m_DistStepAdd;
  PQData &pqB   = pqData[qIdx & 3];
  pqB.deltaDist = (scaledAdd * qIdx + m_DistAdd) >> m_DistShift;
  pqB.absLevel  = (++qIdx) >> 1;
  scaledAdd += m_DistStepAdd;
  PQData &pqC   = pqData[qIdx & 3];
  pqC.deltaDist = (scaledAdd * qIdx + m_DistAdd) >> m_DistShift;
  pqC.absLevel  = (++qIdx) >> 1;
  scaledAdd += m_DistStepAdd;
  PQData &pqD   = pqData[qIdx & 3];
  pqD.deltaDist = (scaledAdd * qIdx + m_DistAdd) >> m_DistShift;
  pqD.absLevel  = (++qIdx) >> 1;

  return false;
}

inline bool Quantizer::preQuantAbsSQ(const TCoeff absCoeff, PQData *pqData, TCoeff quanCoeff) const
{
  int64_t scaledOrg = int64_t(absCoeff) * quanCoeff;
  TCoeff  qIdx      = TCoeff((scaledOrg + m_QAdd) >> m_QShift);

  if (qIdx < 0)
  {
    int64_t scaledAdd = m_DistStepAdd - scaledOrg * m_DistOrgFact;
    PQData &pqA       = pqData[0];

    pqA.deltaDist = (scaledAdd + m_DistAdd) >> m_DistShift;
    pqA.absLevel  = 1;

    return true;
  }

  qIdx              = std::max<TCoeff>(1, std::min<TCoeff>(m_maxQIdx, qIdx));
  int64_t scaledAdd = qIdx * m_DistStepAdd - scaledOrg * m_DistOrgFact;
  PQData &pqA       = pqData[0];
  pqA.deltaDist     = (scaledAdd * qIdx + m_DistAdd) >> m_DistShift;
  pqA.absLevel      = qIdx++;
  scaledAdd += m_DistStepAdd;
  PQData &pqB   = pqData[1];
  pqB.deltaDist = (scaledAdd * qIdx + m_DistAdd) >> m_DistShift;
  pqB.absLevel  = qIdx;

  return false;
}

  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   T C Q   S T A T E                                                  =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

class State;

struct SbbCtx
{
  uint8_t *sbbFlags;
  uint8_t *levels;
};

class CommonCtx
{
public:
  CommonCtx() : m_currSbbCtx(m_allSbbCtx), m_prevSbbCtx(m_currSbbCtx + 8) {}

  inline void swap() { std::swap(m_currSbbCtx, m_prevSbbCtx); }

  inline void reset(const TUParameters &tuPars, const RateEstimator &rateEst)
  {
    m_nbInfo = tuPars.m_scanId2NbInfoOut;
    ::memcpy(m_sbbFlagBits, rateEst.sigSbbFracBits(), 2 * sizeof(BinFracBits));
    const int numSbb    = tuPars.m_numSbb;
    const int chunkSize = numSbb + tuPars.m_numCoeff;
    uint8_t  *nextMem   = m_memory;
    for (int k = 0; k < 16; k++, nextMem += chunkSize)
    {
      m_allSbbCtx[k].sbbFlags = nextMem;
      m_allSbbCtx[k].levels   = nextMem + numSbb;
    }
  }

  template<bool cpy> inline void update(const ScanInfo &scanInfo, const State *prevState, State &currState);

private:
  const NbInfoOut *m_nbInfo;
  BinFracBits      m_sbbFlagBits[2];
  SbbCtx           m_allSbbCtx[16];
  SbbCtx          *m_currSbbCtx;
  SbbCtx          *m_prevSbbCtx;
  uint8_t          m_memory[16 * (MAX_TB_SIZEY * MAX_TB_SIZEY + MLS_GRP_NUM)];
};

#define RICEMAX 32
const int32_t g_goRiceBits[4][RICEMAX] = {
  { 32768,  65536,  98304,  131072, 163840, 196608, 262144, 262144, 327680, 327680, 327680,
    327680, 393216, 393216, 393216, 393216, 393216, 393216, 393216, 393216, 458752, 458752,
    458752, 458752, 458752, 458752, 458752, 458752, 458752, 458752, 458752, 458752 },
  { 65536,  65536,  98304,  98304,  131072, 131072, 163840, 163840, 196608, 196608, 229376,
    229376, 294912, 294912, 294912, 294912, 360448, 360448, 360448, 360448, 360448, 360448,
    360448, 360448, 425984, 425984, 425984, 425984, 425984, 425984, 425984, 425984 },
  { 98304,  98304,  98304,  98304,  131072, 131072, 131072, 131072, 163840, 163840, 163840,
    163840, 196608, 196608, 196608, 196608, 229376, 229376, 229376, 229376, 262144, 262144,
    262144, 262144, 327680, 327680, 327680, 327680, 327680, 327680, 327680, 327680 },
  { 131072, 131072, 131072, 131072, 131072, 131072, 131072, 131072, 163840, 163840, 163840,
    163840, 163840, 163840, 163840, 163840, 196608, 196608, 196608, 196608, 196608, 196608,
    196608, 196608, 229376, 229376, 229376, 229376, 229376, 229376, 229376, 229376 }
};

class State
{
  friend class CommonCtx;

public:
  State(const RateEstimator &rateEst, CommonCtx &commonCtx, const int stateId);

  template<bool cpy> inline void updateState(const ScanInfo &scanInfo, const State *prevStates,
                                             const Decision &decision, const int baseLevel, const bool extRiceFlag);
  template<bool cpy> inline void updateStateEOS(const ScanInfo &scanInfo, const State *prevStates,
                                                const State *skipStates, const Decision &decision, const int skipOffset,
                                                const bool extRiceFlag);

  inline void init(int effectiveWidth, int effectiveHeight)
  {
    m_rdCost        = rdCostInit;
    m_numSigSbb     = 0;
    m_remRegBins    = MAX_REG_BINS;  // just large enough for last scan pos
    m_refSbbCtxId   = -1;
    m_sigFracBits   = m_sigFracBitsArray[0];
    m_coeffFracBits = m_gtxFracBitsArray[0];
    m_goRicePar     = 0;
    m_goRiceZero    = 0;
    effWidth        = effectiveWidth;
    effHeight       = effectiveHeight;
    ::memset(&m_sbb, 0, sizeof(m_sbb));
  }

  void checkRdCosts(const ScanPosType spt, const PQData &pqDataA, const PQData &pqDataB, Decision &decisionA,
                    Decision &decisionB) const
  {
    const int32_t *goRiceTab = g_goRiceBits[m_goRicePar];
    int64_t        rdCostA   = m_rdCost + pqDataA.deltaDist;
    int64_t        rdCostB   = m_rdCost + pqDataB.deltaDist;
    int64_t        rdCostZ   = m_rdCost;
    if (m_remRegBins >= MAX_REG_BINS)
    {
      if (pqDataA.absLevel < GTN_LEVEL)
      {
        rdCostA += m_coeffFracBits.bits[pqDataA.absLevel];
      }
      else
      {
        const TCoeff value = (pqDataA.absLevel - GTN_LEVEL) >> 1;
        rdCostA +=
          m_coeffFracBits.bits[pqDataA.absLevel - (value << 1)] + goRiceTab[value < RICEMAX ? value : RICEMAX - 1];
      }
      if (pqDataB.absLevel < GTN_LEVEL)
      {
        rdCostB += m_coeffFracBits.bits[pqDataB.absLevel];
      }
      else
      {
        const TCoeff value = (pqDataB.absLevel - GTN_LEVEL) >> 1;
        rdCostB +=
          m_coeffFracBits.bits[pqDataB.absLevel - (value << 1)] + goRiceTab[value < RICEMAX ? value : RICEMAX - 1];
      }
      if (spt == SCAN_ISCSBB)
      {
        rdCostA += m_sigFracBits.intBits[1];
        rdCostB += m_sigFracBits.intBits[1];
        rdCostZ += m_sigFracBits.intBits[0];
      }
      else if (spt == SCAN_SOCSBB)
      {
        rdCostA += m_sbbFracBits.intBits[1] + m_sigFracBits.intBits[1];
        rdCostB += m_sbbFracBits.intBits[1] + m_sigFracBits.intBits[1];
        rdCostZ += m_sbbFracBits.intBits[1] + m_sigFracBits.intBits[0];
      }
      else if (m_numSigSbb)
      {
        rdCostA += m_sigFracBits.intBits[1];
        rdCostB += m_sigFracBits.intBits[1];
        rdCostZ += m_sigFracBits.intBits[0];
      }
      else
      {
        rdCostZ = decisionA.rdCost;
      }
    }
    else
    {
      rdCostA += (1 << SCALE_BITS) +
        goRiceTab[pqDataA.absLevel <= m_goRiceZero ? pqDataA.absLevel - 1
                                                   : (pqDataA.absLevel < RICEMAX ? pqDataA.absLevel : RICEMAX - 1)];
      rdCostB += (1 << SCALE_BITS) +
        goRiceTab[pqDataB.absLevel <= m_goRiceZero ? pqDataB.absLevel - 1
                                                   : (pqDataB.absLevel < RICEMAX ? pqDataB.absLevel : RICEMAX - 1)];
      rdCostZ += goRiceTab[m_goRiceZero];
    }
    if (rdCostA < decisionA.rdCost)
    {
      decisionA.rdCost   = rdCostA;
      decisionA.absLevel = pqDataA.absLevel;
      decisionA.prevId   = m_stateId;
    }
    if (rdCostZ < decisionA.rdCost)
    {
      decisionA.rdCost   = rdCostZ;
      decisionA.absLevel = 0;
      decisionA.prevId   = m_stateId;
    }
    if (rdCostB < decisionB.rdCost)
    {
      decisionB.rdCost   = rdCostB;
      decisionB.absLevel = pqDataB.absLevel;
      decisionB.prevId   = m_stateId;
    }
  }

  void checkRdCostsOdd1(const ScanPosType spt, const PQData &pqDataA, Decision &decisionA, Decision &decisionZ) const
  {
    CHECKD(pqDataA.absLevel != 1, "");

    const int32_t *goRiceTab = g_goRiceBits[m_goRicePar];
    int64_t        rdCostA   = m_rdCost + pqDataA.deltaDist;
    int64_t        rdCostZ   = m_rdCost;

    if (m_remRegBins >= MAX_REG_BINS)
    {
      rdCostA += m_coeffFracBits.bits[1];

      if (spt == SCAN_ISCSBB)
      {
        rdCostA += m_sigFracBits.intBits[1];
        rdCostZ += m_sigFracBits.intBits[0];
      }
      else if (spt == SCAN_SOCSBB)
      {
        rdCostA += m_sbbFracBits.intBits[1] + m_sigFracBits.intBits[1];
        rdCostZ += m_sbbFracBits.intBits[1] + m_sigFracBits.intBits[0];
      }
      else if (m_numSigSbb)
      {
        rdCostA += m_sigFracBits.intBits[1];
        rdCostZ += m_sigFracBits.intBits[0];
      }
      else
      {
        rdCostZ = decisionZ.rdCost;
      }
    }
    else
    {
      rdCostA += (1 << SCALE_BITS) + goRiceTab[0];
      rdCostZ += goRiceTab[m_goRiceZero];
    }

    if (rdCostA < decisionA.rdCost)
    {
      decisionA.rdCost   = rdCostA;
      decisionA.absLevel = 1;
      decisionA.prevId   = m_stateId;
    }

    if (rdCostZ < decisionZ.rdCost)
    {
      decisionZ.rdCost   = rdCostZ;
      decisionZ.absLevel = 0;
      decisionZ.prevId   = m_stateId;
    }
  }

  inline void checkRdCostStart(int32_t lastOffset, const PQData &pqData, Decision &decision) const
  {
    int64_t rdCost = pqData.deltaDist + lastOffset;
    if (pqData.absLevel < GTN_LEVEL)
    {
      rdCost += m_coeffFracBits.bits[pqData.absLevel];
    }
    else
    {
      const TCoeff value = (pqData.absLevel - GTN_LEVEL) >> 1;
      rdCost += m_coeffFracBits.bits[pqData.absLevel - (value << 1)] +
        g_goRiceBits[m_goRicePar][value < RICEMAX ? value : RICEMAX - 1];
    }
    if (rdCost < decision.rdCost)
    {
      decision.rdCost   = rdCost;
      decision.absLevel = pqData.absLevel;
      decision.prevId   = -1;
    }
  }

  inline void checkRdCostSkipSbb(Decision &decision, const int skipOffset) const
  {
    int64_t rdCost = m_rdCost + m_sbbFracBits.intBits[0];
    if (rdCost < decision.rdCost)
    {
      decision.rdCost   = rdCost;
      decision.absLevel = 0;
      decision.prevId   = skipOffset + m_stateId;
    }
  }

  struct CtxAcc
  {
    uint8_t sumAbs1, numPos, sumAbs;
  };

private:
  int64_t m_rdCost;
  struct
  {
    uint8_t absLevels[16];
    CtxAcc  ctx[16];
  } m_sbb;
  int8_t                     m_numSigSbb;
  int                        m_remRegBins;
  int8_t                     m_refSbbCtxId;
  BinFracBits                m_sbbFracBits;
  BinFracBits                m_sigFracBits;
  CoeffFracBits              m_coeffFracBits;
  int8_t                     m_goRicePar;
  int8_t                     m_goRiceZero;
  const int8_t               m_stateId;
  const BinFracBits *const   m_sigFracBitsArray;
  const CoeffFracBits *const m_gtxFracBitsArray;
  CommonCtx                 &m_commonCtx;
  unsigned                   effWidth;
  unsigned                   effHeight;

public:
  static const int64_t rdCostInit = std::numeric_limits<int64_t>::max() >> 1;
};

State::State(const RateEstimator &rateEst, CommonCtx &commonCtx, const int stateId)
  : m_sbbFracBits { { 0, 0 } }
  , m_stateId(stateId)
  , m_sigFracBitsArray(rateEst.sigFlagBits(stateId))
  , m_gtxFracBitsArray(rateEst.gtxFracBits(stateId))
  , m_commonCtx(commonCtx)
{}

template<bool cpy> inline void State::updateState(const ScanInfo &scanInfo, const State *prevStates,
                                                  const Decision &decision, const int baseLevel, const bool extRiceFlag)
{
  m_rdCost = decision.rdCost;
  if (decision.prevId > -2)
  {
    if (decision.prevId >= 0)
    {
      if (cpy)
      {
        const State *prvState = prevStates + decision.prevId;
        m_numSigSbb           = prvState->m_numSigSbb + !!decision.absLevel;
        m_refSbbCtxId         = prvState->m_refSbbCtxId;
        m_sbbFracBits         = prvState->m_sbbFracBits;
        m_remRegBins          = prvState->m_remRegBins - 1;
        m_goRicePar           = prvState->m_goRicePar;
        ::memcpy(&m_sbb, &prvState->m_sbb, sizeof(m_sbb));
      }
      else
      {
        m_numSigSbb += !!decision.absLevel;
        m_remRegBins--;
      }
      if (m_remRegBins >= MAX_REG_BINS)
      {
        m_remRegBins -= decision.absLevel <= (GTN_LEVEL - 1) ? (int)decision.absLevel : (GTN_LEVEL - 1);
      }
    }
    else
    {
      m_numSigSbb           = 1;
      m_refSbbCtxId         = -1;
      int ctxBinSampleRatio = isLuma(scanInfo.chType) ? MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_LUMA
                                                      : MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_CHROMA;
      m_remRegBins          = (effWidth * effHeight * ctxBinSampleRatio) / 16;
      m_remRegBins -= decision.absLevel <= (GTN_LEVEL - 1) ? (int)decision.absLevel : (GTN_LEVEL - 1);

      ::memset(&m_sbb, 0, sizeof(m_sbb));
    }

    if (decision.absLevel)
    {
      const uint8_t clippedAbslevel       = (uint8_t)std::min<TCoeff>(255, decision.absLevel);
      m_sbb.absLevels[scanInfo.insidePos] = clippedAbslevel;

      if (scanInfo.currNbInfoSbb.numInv)
      {
        uint8_t absPass1 = (uint8_t)std::min<TCoeff>(GTN_LEVEL + (decision.absLevel & 1), decision.absLevel);

        auto adds8 = [](uint8_t a, uint8_t b)
        {
          uint8_t c = a + b;
          if (c < a)
          {
            c = -1;
          }
          return c;
        };

        auto update_deps = [&](int k)
        {
          auto &ctx = m_sbb.ctx[scanInfo.currNbInfoSbb.invInPos[k]];
          ctx.sumAbs1 += absPass1;
          ctx.numPos++;
          ctx.sumAbs = adds8(ctx.sumAbs, clippedAbslevel);
        };

        switch (scanInfo.currNbInfoSbb.numInv)
        {
        default:
        case 5:
          update_deps(4);
        case 4:
          update_deps(3);
        case 3:
          update_deps(2);
        case 2:
          update_deps(1);
        case 1:
          update_deps(0);
        }
      }
    }

    if (m_remRegBins >= MAX_REG_BINS)
    {
      TCoeff sumAbs1 = m_sbb.ctx[scanInfo.nextInsidePos].sumAbs1;
      TCoeff sumNum  = m_sbb.ctx[scanInfo.nextInsidePos].numPos;
      TCoeff sumGt1  = sumAbs1 - sumNum;
      m_sigFracBits = m_sigFracBitsArray[scanInfo.sigCtxOffsetNext + std::min<TCoeff>((sumAbs1 + 1) >> 1, NSIGCTX - 1)];
      m_coeffFracBits =
        m_gtxFracBitsArray[scanInfo.gtxCtxOffsetNext + (sumGt1 < (NGTXCTX - 1) ? sumGt1 : (NGTXCTX - 1))];

      TCoeff sumAbs = m_sbb.ctx[scanInfo.nextInsidePos].sumAbs;

      int sumAll  = std::max(std::min(GTN_MAXSUM - 1, (int)sumAbs - sumNum), 0);
      m_goRicePar = g_goRiceParsCoeffGTN[sumAll];
    }
    else
    {
      TCoeff sumAbs = m_sbb.ctx[scanInfo.nextInsidePos].sumAbs;
      if (extRiceFlag)
      {
        unsigned currentShift = CoeffCodingContext::templateAbsCompare(sumAbs);
        sumAbs                = sumAbs >> currentShift;
        sumAbs                = std::min<TCoeff>(31, sumAbs);
        m_goRicePar           = g_goRiceParsCoeff[sumAbs];
        m_goRicePar += currentShift;
      }
      else
      {
        sumAbs      = std::min<TCoeff>(31, sumAbs);
        m_goRicePar = g_goRiceParsCoeff[sumAbs];
      }
      m_goRiceZero = g_goRicePosCoeff0(m_stateId, m_goRicePar);
    }
  }
}

template<bool cpy> inline void State::updateStateEOS(const ScanInfo &scanInfo, const State *prevStates,
                                                     const State *skipStates, const Decision &decision,
                                                     const int skipOffset, const bool extRiceFlag)
{
  m_rdCost = decision.rdCost;
  if (decision.prevId > -2)
  {
    const State *prvState = 0;
    if (decision.prevId >= skipOffset)
    {
      CHECK(decision.absLevel != 0, "cannot happen");
      prvState     = skipStates + (decision.prevId - skipOffset);
      m_numSigSbb  = 0;
      m_remRegBins = prvState->m_remRegBins;
      ::memset(m_sbb.absLevels, 0, sizeof(m_sbb.absLevels));
    }
    else if (decision.prevId >= 0)
    {
      prvState     = prevStates + decision.prevId;
      m_numSigSbb  = prvState->m_numSigSbb + !!decision.absLevel;
      m_remRegBins = prvState->m_remRegBins - 1;
      if (m_remRegBins >= MAX_REG_BINS)
      {
        m_remRegBins -= decision.absLevel <= (GTN_LEVEL - 1) ? (int)decision.absLevel : (GTN_LEVEL - 1);
      }
      if (cpy)
      {
        ::memcpy(m_sbb.absLevels, prvState->m_sbb.absLevels, sizeof(m_sbb.absLevels));
      }
    }
    else
    {
      int ctxBinSampleRatio = isLuma(scanInfo.chType) ? MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_LUMA
                                                      : MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_CHROMA;
      m_numSigSbb           = 1;
      m_remRegBins          = (effWidth * effHeight * ctxBinSampleRatio) / 16;
      m_remRegBins -= decision.absLevel <= (GTN_LEVEL - 1) ? (int)decision.absLevel : (GTN_LEVEL - 1);
      ::memset(m_sbb.absLevels, 0, sizeof(m_sbb.absLevels));
    }

    m_sbb.absLevels[scanInfo.insidePos] = (uint8_t)std::min<TCoeff>(255, decision.absLevel);

    m_commonCtx.update<cpy>(scanInfo, prvState, *this);

    if (m_remRegBins >= MAX_REG_BINS)
    {
      TCoeff sumAbs1 = m_sbb.ctx[scanInfo.nextInsidePos].sumAbs1;
      TCoeff sumNum  = m_sbb.ctx[scanInfo.nextInsidePos].numPos;
      TCoeff sumGt1  = sumAbs1 - sumNum;
      m_sigFracBits = m_sigFracBitsArray[scanInfo.sigCtxOffsetNext + std::min<TCoeff>((sumAbs1 + 1) >> 1, NSIGCTX - 1)];
      m_coeffFracBits =
        m_gtxFracBitsArray[scanInfo.gtxCtxOffsetNext + (sumGt1 < (NGTXCTX - 1) ? sumGt1 : (NGTXCTX - 1))];

      TCoeff sumAbs = m_sbb.ctx[scanInfo.nextInsidePos].sumAbs;

      int sumAll  = std::max(std::min(GTN_MAXSUM - 1, (int)sumAbs - sumNum), 0);
      m_goRicePar = g_goRiceParsCoeffGTN[sumAll];
    }
    else
    {
      TCoeff sumAbs = m_sbb.ctx[scanInfo.nextInsidePos].sumAbs;
      if (extRiceFlag)
      {
        unsigned currentShift = CoeffCodingContext::templateAbsCompare(sumAbs);
        sumAbs                = sumAbs >> currentShift;
        sumAbs                = std::min<TCoeff>(31, sumAbs);
        m_goRicePar           = g_goRiceParsCoeff[sumAbs];
        m_goRicePar += currentShift;
      }
      else
      {
        sumAbs      = std::min<TCoeff>(31, sumAbs);
        m_goRicePar = g_goRiceParsCoeff[sumAbs];
      }
      m_goRiceZero = g_goRicePosCoeff0(m_stateId, m_goRicePar);
    }
  }
}

template<bool cpy> inline void CommonCtx::update(const ScanInfo &scanInfo, const State *prevState, State &currState)
{
  uint8_t    *sbbFlags  = m_currSbbCtx[currState.m_stateId].sbbFlags;
  uint8_t    *levels    = m_currSbbCtx[currState.m_stateId].levels;
  std::size_t setCpSize = m_nbInfo[scanInfo.scanIdx - 1].maxDist * sizeof(uint8_t);
  if (prevState && prevState->m_refSbbCtxId >= 0)
  {
    if (cpy)
    {
      ::memcpy(sbbFlags, m_prevSbbCtx[prevState->m_refSbbCtxId].sbbFlags, scanInfo.numSbb * sizeof(uint8_t));
      ::memcpy(levels + scanInfo.scanIdx, m_prevSbbCtx[prevState->m_refSbbCtxId].levels + scanInfo.scanIdx, setCpSize);
    }
  }
  else
  {
    ::memset(sbbFlags, 0, scanInfo.numSbb * sizeof(uint8_t));
    ::memset(levels + scanInfo.scanIdx, 0, setCpSize);
  }
  sbbFlags[scanInfo.sbbPos] = !!currState.m_numSigSbb;
  ::memcpy(levels + scanInfo.scanIdx, currState.m_sbb.absLevels, scanInfo.sbbSize * sizeof(uint8_t));

  const int sigNSbb       = ((scanInfo.nextSbbRight ? sbbFlags[scanInfo.nextSbbRight] : false) ||
                           (scanInfo.nextSbbBelow ? sbbFlags[scanInfo.nextSbbBelow] : false)
                               ? 1
                               : 0);
  currState.m_numSigSbb   = 0;
  currState.m_refSbbCtxId = currState.m_stateId;
  currState.m_sbbFracBits = m_sbbFlagBits[sigNSbb];

  ::memset(&currState.m_sbb, 0, sizeof(currState.m_sbb));

  if (sigNSbb || ((scanInfo.nextSbbRight && scanInfo.nextSbbBelow) ? sbbFlags[scanInfo.nextSbbBelow + 1] : false))
  {
    const int        scanBeg   = scanInfo.scanIdx - scanInfo.sbbSize;
    const NbInfoOut *nbOut     = m_nbInfo + scanBeg;
    const uint8_t   *absLevels = levels + scanBeg;
    for (int id = 0; id < scanInfo.sbbSize; id++, nbOut++)
    {
      if (nbOut->num)
      {
        TCoeff sumAbs = 0, sumAbs1 = 0, sumNum = 0;
#define UPDATE(k)                                        \
  {                                                      \
    TCoeff t = absLevels[nbOut->outPos[k]];              \
    sumAbs += t;                                         \
    sumAbs1 += std::min<TCoeff>(GTN_LEVEL + (t & 1), t); \
    sumNum += !!t;                                       \
  }
        switch (nbOut->num)
        {
        default:
        case 5:
          UPDATE(4);
        case 4:
          UPDATE(3);
        case 3:
          UPDATE(2);
        case 2:
          UPDATE(1);
        case 1:
          UPDATE(0);
        }
#undef UPDATE
        currState.m_sbb.ctx[id].sumAbs1 = sumAbs1;
        currState.m_sbb.ctx[id].numPos  = sumNum;
        currState.m_sbb.ctx[id].sumAbs  = (uint8_t)std::min<TCoeff>(127, sumAbs);
      }
    }
  }
}

// clang-format off
#define DI(l,p) {std::numeric_limits<int64_t>::max()>>2,l,p}
#define DX      {std::numeric_limits<int64_t>::max()>>2,0,0}
  static const Decision s_startDec[3][16] = {
    {
      DI(-1,-2),
      DI( 0, 1),
      DX, DX, DX, DX, DX, DX, DX, DX, DX, DX, DX, DX, DX, DX,
    },
    {
      DI(-1,-2),DI(-1,-2),DI(-1,-2),DI(-1,-2),
      DI( 0, 4),DI( 0, 5),DI( 0, 6),DI( 0, 7),
      DX, DX, DX, DX, DX, DX, DX, DX, 
    },
    {
      DI(-1,-2),DI(-1,-2),DI(-1,-2),DI(-1,-2),DI(-1,-2),DI(-1,-2),DI(-1,-2),DI(-1,-2),
      DI( 0, 8),DI( 0, 9),DI( 0,10),DI( 0,11),DI( 0,12),DI( 0,13),DI( 0,14),DI( 0,15),
    }
  };
#undef DX
#undef DI
// clang-format on

  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   T C Q                                                              =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/
class DepQuant : private RateEstimator
{
public:
  DepQuant();

  void quant(TransformUnit &tu, const CCoeffBuf &srcCoeff, const CompID compID, const QpParam &cQP, const double lambda,
             const Ctx &ctx, TCoeff &absSum, bool enableScalingLists, int *quantCoeff);
  void dequant(const TransformUnit &tu, CoeffBuf &recCoeff, const CompID compID, const QpParam &cQP,
               bool enableScalingLists, int *quantCoeff);

  int  m_baseLevel;
  bool m_extRiceRRCFlag;

private:
  template<int qm>
  void xDecideAndUpdate(const TCoeff absCoeff, const ScanInfo &scanInfo, TCoeff quantCoeff, bool reverseLast);
  template<int qm> void xDecide(const ScanPosType spt, const TCoeff absCoeff, const int lastOffset, Decision *decisions,
                                TCoeff quantCoeff);

private:
  CommonCtx m_commonCtx;
  State     m_allStates[24];
  State    *m_currStates;
  State    *m_prevStates;
  State    *m_skipStates;
  State     m_startState;
  Quantizer m_quant;
  Decision  m_trellis[MAX_TB_SIZEY * MAX_TB_SIZEY * 16];
};

#define TINIT(x) { *this, m_commonCtx, x }
DepQuant::DepQuant()
  : RateEstimator()
  , m_commonCtx()
  , m_allStates { TINIT(0), TINIT(1), TINIT(2), TINIT(3), TINIT(4), TINIT(5), TINIT(6), TINIT(7),
                  TINIT(0), TINIT(1), TINIT(2), TINIT(3), TINIT(4), TINIT(5), TINIT(6), TINIT(7),
                  TINIT(0), TINIT(1), TINIT(2), TINIT(3), TINIT(4), TINIT(5), TINIT(6), TINIT(7) }
  , m_currStates(m_allStates)
  , m_prevStates(m_currStates + 8)
  , m_skipStates(m_prevStates + 8)
  , m_startState TINIT(0)
{}
#undef TINIT

void DepQuant::dequant(const TransformUnit &tu, CoeffBuf &recCoeff, const CompID compID, const QpParam &cQP,
                       bool enableScalingLists, int *piDequantCoef)
{
  m_quant.dequantBlock(tu, compID, cQP, recCoeff, enableScalingLists, piDequantCoef,
                       g_stateTransTab[tu.cs->slice->m_depQuantEnabledIdc]);
}

template<int qm> void DepQuant::xDecide(const ScanPosType spt, const TCoeff absCoeff, const int lastOffset,
                                        Decision *decisions, TCoeff quanCoeff)
{
  constexpr int numStates = (qm > 1 ? 8 : qm ? 4 : 1);
  ::memcpy(decisions, s_startDec[qm], (numStates << 1) * sizeof(Decision));

  PQData pqData[4];
  bool   near0 = false;
  if constexpr (qm > 0)
  {
    near0 = m_quant.preQuantAbsTCQ(absCoeff, pqData, quanCoeff);
  }
  else
  {
    near0 = m_quant.preQuantAbsSQ(absCoeff, pqData, quanCoeff);
  }

  State *prevStates = qm || spt == SCAN_SOCSBB ? m_prevStates : m_currStates;

  if constexpr (qm > 1)
  {
      // TCQ with 8 states
    if (near0)
    {
      prevStates[0].checkRdCostsOdd1(spt, pqData[2], decisions[2], decisions[0]);
      prevStates[1].checkRdCostsOdd1(spt, pqData[1], decisions[7], decisions[5]);
      prevStates[2].checkRdCostsOdd1(spt, pqData[2], decisions[3], decisions[1]);
      prevStates[3].checkRdCostsOdd1(spt, pqData[1], decisions[4], decisions[6]);
      prevStates[4].checkRdCostsOdd1(spt, pqData[2], decisions[0], decisions[2]);
      prevStates[5].checkRdCostsOdd1(spt, pqData[1], decisions[6], decisions[4]);
      prevStates[6].checkRdCostsOdd1(spt, pqData[2], decisions[1], decisions[3]);
      prevStates[7].checkRdCostsOdd1(spt, pqData[1], decisions[5], decisions[7]);

      m_startState.checkRdCostStart(lastOffset, pqData[2], decisions[2]);
    }
    else
    {
      prevStates[0].checkRdCosts(spt, pqData[0], pqData[2], decisions[0], decisions[2]);
      prevStates[1].checkRdCosts(spt, pqData[3], pqData[1], decisions[5], decisions[7]);
      prevStates[2].checkRdCosts(spt, pqData[0], pqData[2], decisions[1], decisions[3]);
      prevStates[3].checkRdCosts(spt, pqData[3], pqData[1], decisions[6], decisions[4]);
      prevStates[4].checkRdCosts(spt, pqData[0], pqData[2], decisions[2], decisions[0]);
      prevStates[5].checkRdCosts(spt, pqData[3], pqData[1], decisions[4], decisions[6]);
      prevStates[6].checkRdCosts(spt, pqData[0], pqData[2], decisions[3], decisions[1]);
      prevStates[7].checkRdCosts(spt, pqData[3], pqData[1], decisions[7], decisions[5]);

      m_startState.checkRdCostStart(lastOffset, pqData[0], decisions[0]);
      m_startState.checkRdCostStart(lastOffset, pqData[2], decisions[2]);
    }
  }
  else if constexpr (qm > 0)
  {
      // TCQ with 4 states
    if (near0)
    {
      prevStates[0].checkRdCostsOdd1(spt, pqData[2], decisions[1], decisions[0]);
      prevStates[1].checkRdCostsOdd1(spt, pqData[1], decisions[3], decisions[2]);
      prevStates[2].checkRdCostsOdd1(spt, pqData[2], decisions[0], decisions[1]);
      prevStates[3].checkRdCostsOdd1(spt, pqData[1], decisions[2], decisions[3]);

      m_startState.checkRdCostStart(lastOffset, pqData[2], decisions[1]);
    }
    else
    {
      prevStates[0].checkRdCosts(spt, pqData[0], pqData[2], decisions[0], decisions[1]);
      prevStates[1].checkRdCosts(spt, pqData[3], pqData[1], decisions[2], decisions[3]);
      prevStates[2].checkRdCosts(spt, pqData[0], pqData[2], decisions[1], decisions[0]);
      prevStates[3].checkRdCosts(spt, pqData[3], pqData[1], decisions[3], decisions[2]);

      m_startState.checkRdCostStart(lastOffset, pqData[0], decisions[0]);
      m_startState.checkRdCostStart(lastOffset, pqData[2], decisions[1]);
    }
  }
  else
  {
      // scalar quantization
    if (near0)
    {
      prevStates[0].checkRdCostsOdd1(spt, pqData[0], decisions[0], decisions[0]);

      m_startState.checkRdCostStart(lastOffset, pqData[0], decisions[0]);
    }
    else
    {
      prevStates[0].checkRdCosts(spt, pqData[0], pqData[1], decisions[0], decisions[0]);

      m_startState.checkRdCostStart(lastOffset, pqData[0], decisions[0]);
      m_startState.checkRdCostStart(lastOffset, pqData[1], decisions[0]);
    }
  }

  if (spt == SCAN_EOCSBB)
  {
    {
      m_skipStates[0].checkRdCostSkipSbb(decisions[0], numStates);
    }
    if constexpr (numStates > 1)
    {
      m_skipStates[1].checkRdCostSkipSbb(decisions[1], numStates);
      m_skipStates[2].checkRdCostSkipSbb(decisions[2], numStates);
      m_skipStates[3].checkRdCostSkipSbb(decisions[3], numStates);
    }
    if constexpr (numStates > 4)
    {
      m_skipStates[4].checkRdCostSkipSbb(decisions[4], numStates);
      m_skipStates[5].checkRdCostSkipSbb(decisions[5], numStates);
      m_skipStates[6].checkRdCostSkipSbb(decisions[6], numStates);
      m_skipStates[7].checkRdCostSkipSbb(decisions[7], numStates);
    }
  }
}

template<int qm>
void DepQuant::xDecideAndUpdate(const TCoeff absCoeff, const ScanInfo &scanInfo, TCoeff quantCoeff, bool reverseLast)
{
  constexpr int numStates = (qm > 1 ? 8 : qm ? 4 : 1);
  Decision     *decisions = m_trellis + scanInfo.scanIdx * (numStates << 1);

  if (qm || scanInfo.spt == SCAN_SOCSBB)
  {
    std::swap(m_prevStates, m_currStates);
  }

  xDecide<qm>(scanInfo.spt, absCoeff, lastOffset(scanInfo.scanIdx, scanInfo.tuWidth, scanInfo.tuHeight, reverseLast),
              decisions, quantCoeff);

  if (scanInfo.scanIdx)
  {
    State *prevStates = qm || scanInfo.spt == SCAN_SOCSBB ? m_prevStates : m_currStates;

    if (scanInfo.eosbb)
    {
      if (qm)
      {
        m_commonCtx.swap();
      }
      {
        m_currStates[0].updateStateEOS<!!qm>(scanInfo, prevStates, m_skipStates, decisions[0], numStates,
                                             m_extRiceRRCFlag);
      }
      if constexpr (numStates > 1)
      {
        m_currStates[1].updateStateEOS<!!qm>(scanInfo, prevStates, m_skipStates, decisions[1], numStates,
                                             m_extRiceRRCFlag);
        m_currStates[2].updateStateEOS<!!qm>(scanInfo, prevStates, m_skipStates, decisions[2], numStates,
                                             m_extRiceRRCFlag);
        m_currStates[3].updateStateEOS<!!qm>(scanInfo, prevStates, m_skipStates, decisions[3], numStates,
                                             m_extRiceRRCFlag);
      }
      if constexpr (numStates > 4)
      {
        m_currStates[4].updateStateEOS<!!qm>(scanInfo, prevStates, m_skipStates, decisions[4], numStates,
                                             m_extRiceRRCFlag);
        m_currStates[5].updateStateEOS<!!qm>(scanInfo, prevStates, m_skipStates, decisions[5], numStates,
                                             m_extRiceRRCFlag);
        m_currStates[6].updateStateEOS<!!qm>(scanInfo, prevStates, m_skipStates, decisions[6], numStates,
                                             m_extRiceRRCFlag);
        m_currStates[7].updateStateEOS<!!qm>(scanInfo, prevStates, m_skipStates, decisions[7], numStates,
                                             m_extRiceRRCFlag);
      }
      ::memcpy(decisions + numStates, decisions, numStates * sizeof(Decision));
    }
    else
    {
      if (qm || scanInfo.spt == SCAN_SOCSBB)
      {
        m_currStates[0].updateState<true>(scanInfo, prevStates, decisions[0], m_baseLevel, m_extRiceRRCFlag);
      }
      else
      {
        m_currStates[0].updateState<false>(scanInfo, prevStates, decisions[0], m_baseLevel, m_extRiceRRCFlag);
      }
      if constexpr (numStates > 1)
      {
        m_currStates[1].updateState<true>(scanInfo, prevStates, decisions[1], m_baseLevel, m_extRiceRRCFlag);
        m_currStates[2].updateState<true>(scanInfo, prevStates, decisions[2], m_baseLevel, m_extRiceRRCFlag);
        m_currStates[3].updateState<true>(scanInfo, prevStates, decisions[3], m_baseLevel, m_extRiceRRCFlag);
      }
      if constexpr (numStates > 4)
      {
        m_currStates[4].updateState<true>(scanInfo, prevStates, decisions[4], m_baseLevel, m_extRiceRRCFlag);
        m_currStates[5].updateState<true>(scanInfo, prevStates, decisions[5], m_baseLevel, m_extRiceRRCFlag);
        m_currStates[6].updateState<true>(scanInfo, prevStates, decisions[6], m_baseLevel, m_extRiceRRCFlag);
        m_currStates[7].updateState<true>(scanInfo, prevStates, decisions[7], m_baseLevel, m_extRiceRRCFlag);
      }
    }

    if (scanInfo.spt == SCAN_SOCSBB)
    {
      std::swap(m_prevStates, m_skipStates);
    }
  }
}

void DepQuant::quant(TransformUnit &tu, const CCoeffBuf &srcCoeff, const CompID compID, const QpParam &cQP,
                     const double lambda, const Ctx &ctx, TCoeff &absSum, bool enableScalingLists, int *quantCoeff)
{
  CHECKD(tu.cs->sps->m_spsRangeExtension.m_extendedPrecisionProcessingFlag, "ext precision is not supported");

    //===== reset / pre-init =====
  const int           qmethod   = tu.cs->slice->m_depQuantEnabledIdc;
  const int           numStates = (qmethod > 1 ? 8 : qmethod ? 4 : 1);
  const uint32_t      nstIdx    = TU::getNstIdx(tu, compID);
  const TUParameters &tuPars    = *g_Rom.getTUPars(tu.blocks[compID], compID, nstIdx);
  m_quant.initQuantBlock(tu, compID, cQP, lambda);
  m_baseLevel            = ctx.getBaseLevel();
  m_extRiceRRCFlag       = tu.cs->sps->m_spsRangeExtension.m_rrcRiceExtensionEnableFlag;
  TCoeff       *qCoeff   = tu.getCoeffs(compID).buf;
  const TCoeff *tCoeff   = srcCoeff.buf;
  const int     numCoeff = tu.blocks[compID].area();
  ::memset(tu.getCoeffs(compID).buf, 0x00, numCoeff * sizeof(TCoeff));
  absSum = 0;

  const CompArea &area   = tu.blocks[compID];
  const uint32_t  width  = area.width;
  const uint32_t  height = area.height;

    //===== find first test position =====
  int firstTestPos = numCoeff - 1;
  if (nstIdx > 0 && tu.mtsIdx[compID] != MtsType::SKIP && width >= 4 && height >= 4)
  {
    bool allowNSPT = TU::isNSPTAllowed(width, height);
    firstTestPos   = (allowNSPT ? PU::getNSPTMatrixDim(width, height) : PU::getLFNSTMatrixDim(width, height)) - 1;
  }
  const TCoeff defaultQuantisationCoefficient = (TCoeff)m_quant.getQScale();
  const TCoeff thres                          = m_quant.getLastThreshold();

  if (enableScalingLists)
  {
    for (; firstTestPos >= 0; firstTestPos--)
    {
      TCoeff thresTmp = (enableScalingLists)
        ? TCoeff(thres / (4 * quantCoeff[tuPars.m_scanId2BlkPos[firstTestPos].idx]))
        : TCoeff(thres / (4 * defaultQuantisationCoefficient));

      if (abs(tCoeff[tuPars.m_scanId2BlkPos[firstTestPos].idx]) > thresTmp)
      {
        break;
      }
    }
  }
  else
  {
    const TCoeff defaultTh = TCoeff(thres / (defaultQuantisationCoefficient << 2));

#if ENABLE_SIMD_OPT_QUANT && defined(TARGET_SIMD_X86)
    // if more than one 4x4 coding subblock is available, use SIMD to find first subblock with coefficient larger than
    // threshold
    if ((firstTestPos & 15) == 15 && firstTestPos >= 16 && tuPars.m_log2SbbWidth == 2 && tuPars.m_log2SbbHeight == 2 &&
        read_x86_extension_flags() > SCALAR)
    {
      const int sbbSize = tuPars.m_sbbSize;
      // move the pointer to the beginning of the current subblock
      firstTestPos -= (sbbSize - 1);

      const __m128i xdfTh = _mm_set1_epi32(defaultTh);

      // for each subblock
      for (; firstTestPos >= 0; firstTestPos -= sbbSize)
      {
        // read first line of the subblock and check for coefficients larger than the threshold
        // assumming the subblocks are dense 4x4 blocks in raster scan order with the stride of tuPars.m_width
        int     pos = tuPars.m_scanId2BlkPos[firstTestPos].idx;
        __m128i xl0 = _mm_abs_epi32(_mm_loadu_si128((const __m128i *)&tCoeff[pos]));
        __m128i xdf = _mm_cmpgt_epi32(xl0, xdfTh);

        // same for the next line in the subblock
        pos += tuPars.m_width;
        xl0 = _mm_abs_epi32(_mm_loadu_si128((const __m128i *)&tCoeff[pos]));
        xdf = _mm_or_si128(xdf, _mm_cmpgt_epi32(xl0, xdfTh));

        // and the third line
        pos += tuPars.m_width;
        xl0 = _mm_abs_epi32(_mm_loadu_si128((const __m128i *)&tCoeff[pos]));
        xdf = _mm_or_si128(xdf, _mm_cmpgt_epi32(xl0, xdfTh));

        // and the last line
        pos += tuPars.m_width;
        xl0 = _mm_abs_epi32(_mm_loadu_si128((const __m128i *)&tCoeff[pos]));
        xdf = _mm_or_si128(xdf, _mm_cmpgt_epi32(xl0, xdfTh));

        // if any of the 16 comparisons were true, break, because this subblock contains a coefficient larger than
        // threshold
        if (!_mm_testz_si128(xdf, xdf))
        {
          break;
        }
      }

      if (firstTestPos >= 0)
      {
        // if a coefficient was found, advance the pointer to the end of the current subblock
        // for the subsequent coefficient-wise refinement (C-impl after endif)
        firstTestPos += sbbSize - 1;
      }
    }

#endif

    for (; firstTestPos >= 0; firstTestPos--)
    {
      if (abs(tCoeff[tuPars.m_scanId2BlkPos[firstTestPos].idx]) > defaultTh)
      {
        break;
      }
    }
  }
  if (firstTestPos < 0)
  {
    return;
  }

  //===== real init =====
  const int effectWidth  = tuPars.m_width;
  const int effectHeight = tuPars.m_height;
  RateEstimator::initCtx(tuPars, tu, compID, ctx.getFracBitsAcess());
  m_commonCtx.reset(tuPars, *this);
  for (int k = 0; k < numStates; k++)
  {
    m_allStates[k].init(effectWidth, effectHeight);
    m_allStates[k + 8].init(effectWidth, effectHeight);
    m_allStates[k + 16].init(effectWidth, effectHeight);
  }
  m_startState.init(effectWidth, effectHeight);

  //===== populate trellis =====
  void (DepQuant::*fdecide)(const TCoeff, const ScanInfo &, TCoeff, bool) = nullptr;
  switch (qmethod)
  {
  case 2:
    fdecide = &DepQuant::xDecideAndUpdate<2>;
    break;
  case 1:
    fdecide = &DepQuant::xDecideAndUpdate<1>;
    break;
  case 0:
    fdecide = &DepQuant::xDecideAndUpdate<0>;
    break;
  default:
    THROW("intern error");
  }
  for (int scanIdx = firstTestPos; scanIdx >= 0; scanIdx--)
  {
    const ScanInfo &scanInfo = tuPars.m_scanInfo[scanIdx];
    if (enableScalingLists)
    {
      m_quant.initQuantBlock(tu, compID, cQP, lambda, quantCoeff[scanInfo.rasterPos]);
      (this->*fdecide)(abs(tCoeff[scanInfo.rasterPos]), scanInfo, quantCoeff[scanInfo.rasterPos],
                       tu.cu->slice->m_reverseLastSigCoeffFlag);
    }
    else
    {
      (this->*fdecide)(abs(tCoeff[scanInfo.rasterPos]), scanInfo, defaultQuantisationCoefficient,
                       tu.cu->slice->m_reverseLastSigCoeffFlag);
    }
  }

  //===== find best path =====
  Decision decision    = { std::numeric_limits<int64_t>::max(), -1, -2 };
  int64_t  minPathCost = 0;
  for (int8_t stateId = 0; stateId < numStates; stateId++)
  {
    int64_t pathCost = m_trellis[stateId].rdCost;
    if (pathCost < minPathCost)
    {
      decision.prevId = stateId;
      minPathCost     = pathCost;
    }
  }

  //===== backward scanning =====
  int scanIdx = 0;
  for (const Decision *trellisStage = m_trellis; decision.prevId >= 0; scanIdx++, trellisStage += (numStates << 1))
  {
    decision       = trellisStage[decision.prevId];
    int32_t blkpos = tuPars.m_scanId2BlkPos[scanIdx].idx;
    qCoeff[blkpos] = (tCoeff[blkpos] < 0 ? -decision.absLevel : decision.absLevel);
    absSum += decision.absLevel;
  }

  tu.lastPos[compID] = scanIdx - 1;
}

};   // namespace DQIntern

//===== interface class =====
DepQuant::DepQuant(const Quant *other, bool enc) : QuantRDOQ(other)
{
  const DepQuant *dq = dynamic_cast<const DepQuant *>(other);
  CHECK(other && !dq, "The DepQuant cast must be successfull!");
  p = new DQIntern::DepQuant();
  if (enc)
  {
    DQIntern::g_Rom.init();
  }
}

DepQuant::~DepQuant() { delete static_cast<DQIntern::DepQuant *>(p); }

void DepQuant::quant(TransformUnit &tu, const CompID &compID, const CCoeffBuf &pSrc, TCoeff &absSum, const QpParam &cQP,
                     const Ctx &ctx)
{
  tu.lastPos[compID] = -1;

  const bool preferDQ = true;   // set to false if normal RDOQ should be used
  const bool useRegularResidualCoding =
    tu.cu->slice->m_tsResidualCodingDisabledFlag || tu.mtsIdx[compID] != MtsType::SKIP;
  const bool useDQEnc =
    (tu.cs->slice->m_depQuantEnabledIdc ||
     (preferDQ && m_useRDOQ && !tu.cs->slice->m_signDataHidingEnabledFlag && tu.mtsIdx[compID] != MtsType::SKIP));
  if (useDQEnc && useRegularResidualCoding)
  {
    //===== scaling matrix ====
    const int       qpDQ            = cQP.Qp(tu.mtsIdx[compID] == MtsType::SKIP) + !!tu.cs->slice->m_depQuantEnabledIdc;
    const int       qpPer           = qpDQ / 6;
    const int       qpRem           = qpDQ - 6 * qpPer;
    const CompArea &rect            = tu.blocks[compID];
    const int       width           = rect.width;
    const int       height          = rect.height;
    uint32_t        scalingListType = getScalingListType(tu.cu->predMode, compID);
    CHECK(scalingListType >= SCALING_LIST_NUM, "Invalid scaling list");
    const uint32_t log2TrWidth  = floorLog2(width);
    const uint32_t log2TrHeight = floorLog2(height);

    const bool disableSMForLFNST =
      tu.cs->slice->m_explicitScalingListUsed ? tu.cs->slice->m_sps->m_disableScalingMatrixForLfnstBlks : false;
    const bool isLfnstApplied = TU::getNstIdx(tu, compID) > 0 && (CS::isDualITree(*tu.cs) ? true : isLuma(compID));

    const bool enableScalingLists =
      getUseScalingList(width, height, tu.mtsIdx[compID] == MtsType::SKIP, isLfnstApplied, disableSMForLFNST);

    static_cast<DQIntern::DepQuant *>(p)->quant(
      tu, pSrc, compID, cQP, Quant::m_dLambda, ctx, absSum, enableScalingLists,
      Quant::getQuantCoeff(scalingListType, qpRem, log2TrWidth, log2TrHeight));
  }
  else
  {
    QuantRDOQ::quant(tu, compID, pSrc, absSum, cQP, ctx);
  }
}

void DepQuant::dequant(const TransformUnit &tu, CoeffBuf &dstCoeff, const CompID &compID, const QpParam &cQP)
{
  const bool useRegularResidualCoding =
    tu.cu->slice->m_tsResidualCodingDisabledFlag || tu.mtsIdx[compID] != MtsType::SKIP;
  if (tu.cs->slice->m_depQuantEnabledIdc && useRegularResidualCoding)
  {
    const int       qpDQ            = cQP.Qp(tu.mtsIdx[compID] == MtsType::SKIP) + 1;
    const int       qpPer           = qpDQ / 6;
    const int       qpRem           = qpDQ - 6 * qpPer;
    const CompArea &rect            = tu.blocks[compID];
    const int       width           = rect.width;
    const int       height          = rect.height;
    uint32_t        scalingListType = getScalingListType(tu.cu->predMode, compID);
    CHECK(scalingListType >= SCALING_LIST_NUM, "Invalid scaling list");
    const uint32_t log2TrWidth  = floorLog2(width);
    const uint32_t log2TrHeight = floorLog2(height);

    const bool disableSMForLFNST =
      tu.cs->slice->m_explicitScalingListUsed ? tu.cs->slice->m_sps->m_disableScalingMatrixForLfnstBlks : false;
    const bool isLfnstApplied = TU::getNstIdx(tu, compID) > 0 && (CS::isDualITree(*tu.cs) ? true : isLuma(compID));
    const bool enableScalingLists =
      getUseScalingList(width, height, tu.mtsIdx[compID] == MtsType::SKIP, isLfnstApplied, disableSMForLFNST);
    static_cast<DQIntern::DepQuant *>(p)->dequant(
      tu, dstCoeff, compID, cQP, enableScalingLists,
      Quant::getDequantCoeff(scalingListType, qpRem, log2TrWidth, log2TrHeight));
  }
  else
  {
    QuantRDOQ::dequant(tu, dstCoeff, compID, cQP);
  }
}
