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

/** \file     SampleAdaptiveOffset.h
    \brief    sample adaptive offset class (header)
*/

#ifndef __SAMPLEADAPTIVEOFFSET__
#define __SAMPLEADAPTIVEOFFSET__

#include "CommonDef.h"
#include "Unit.h"
#include "Reshape.h"
#include "BilateralFilter.h"
//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

static constexpr int MAX_SAO_TRUNCATED_BITDEPTH = 10;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class SampleAdaptiveOffset
{
public:
  SampleAdaptiveOffset();
  virtual ~SampleAdaptiveOffset();

  void SAOProcess(CodingStructure &cs, SAOBlkParam *saoBlkParams);
  void create(int picWidth, int picHeight, ChromaFormat format, uint32_t maxCUWidth, uint32_t maxCUHeight,
              uint32_t maxCUDepth, uint32_t lumaBitShift, uint32_t chromaBitShift);
  void setReshaper(Reshape *p) { m_pcReshape = p; }
  void destroy();

  static int getMaxOffsetQVal(const int channelBitDepth)
  {
    return (1 << (std::min<int>(channelBitDepth, MAX_SAO_TRUNCATED_BITDEPTH) - 5)) - 1;
  }   // Table 9-32, inclusive

  void           CCSAOProcess(CodingStructure &cs);
  CcSaoComParam &getCcSaoComParam() { return m_ccSaoComParam; }
  uint8_t       *getCcSaoControlIdc(const CompID compID) { return m_ccSaoControl[compID]; }
  PelStorage    &getCcSaoBuf() { return m_ccSaoBuf; }
  void           jointClipSaoBifCcSao(CodingStructure &cs);
  static int     getCcSaoClassNum(const int compIdx, const int setIdx, const CcSaoComParam &ccSaoParam);
  inline int     getCcSaoEdgeIdx(const Pel c, const Pel a, const int t, const int edgeIdc)
  {
    const int d = c - a;
    if (edgeIdc == 0)
    {
      return (d < 0 ? (d < -t ? 0 : 1) : (d < t ? 2 : 3));
    }
    else /*edgeIdc == 1*/
    {
      return (d < t ? 0 : 1);
    }
  }

  void loadOrStoreCCSaoTemporalPredictor(Slice *pcSlice, CcSaoComParam &ccSaoParam);

protected:
  using MergeBlkParams = EnumArray<SAOBlkParam *, SAOModeMergeTypes>;

  void deriveLoopFilterBoundaryAvailability(CodingStructure &cs, const Position &pos, bool &isLeftAvail,
                                            bool &isRightAvail, bool &isAboveAvail, bool &isBelowAvail,
                                            bool &isAboveLeftAvail, bool &isAboveRightAvail, bool &isBelowLeftAvail,
                                            bool &isBelowRightAvail) const;

  void offsetBlock(const int channelBitDepth, const ClpRng &clpRng, SAOModeNewTypes typeIdx, int *offset,
                   const Pel *srcBlk, Pel *resBlk, ptrdiff_t srcStride, ptrdiff_t resStride, int width, int height,
                   bool isLeftAvail, bool isRightAvail, bool isAboveAvail, bool isBelowAvail, bool isAboveLeftAvail,
                   bool isAboveRightAvail, bool isBelowLeftAvail, bool isBelowRightAvail, int horVirBndryPos[],
                   int verVirBndryPos[], int numHorVirBndry, int numVerVirBndry);

  void offsetBlockNoClip(const int channelBitDepth, const ClpRng &clpRng, SAOModeNewTypes typeIdx, int *offset,
                         const Pel *srcBlk, Pel *resBlk, ptrdiff_t srcStride, ptrdiff_t resStride, int width,
                         int height, bool isLeftAvail, bool isRightAvail, bool isAboveAvail, bool isBelowAvail,
                         bool isAboveLeftAvail, bool isAboveRightAvail, bool isBelowLeftAvail, bool isBelowRightAvail,
                         int horVirBndryPos[], int verVirBndryPos[], int numHorVirBndry, int numVerVirBndry);

  void invertQuantOffsets(CompID compIdx, SAOModeNewTypes typeIdc, int typeAuxInfo, int *dstOffsets, int *srcOffsets);
  void reconstructBlkSAOParam(SAOBlkParam &recParam, MergeBlkParams &mergeList);
  int  getMergeList(CodingStructure &cs, int ctuRsAddr, SAOBlkParam *blkParams, MergeBlkParams &mergeList);
  void offsetCTU(const UnitArea &area, const CPelUnitBuf &src, PelUnitBuf &res, SAOBlkParam &saoblkParam,
                 CodingStructure &cs);

  void offsetCTUnoClip(const UnitArea &area, const CPelUnitBuf &src, PelUnitBuf &res, SAOBlkParam &saoblkParam,
                       CodingStructure &cs);
  void clipCTU(CodingStructure &cs, PelUnitBuf &dstYuv, const UnitArea &area, const CompID compID);
  void xReconstructBlkSAOParams(CodingStructure &cs, SAOBlkParam *saoBlkParams);
  bool isProcessDisabled(int xPos, int yPos, int numVerVirBndry, int numHorVirBndry, int verVirBndryPos[],
                         int horVirBndryPos[])
  {
    bool disabledFlag = false;
    for (int i = 0; i < numVerVirBndry; i++)
    {
      if ((xPos == verVirBndryPos[i]) || (xPos == verVirBndryPos[i] - 1))
      {
        disabledFlag = true;
        break;
      }
    }
    for (int i = 0; i < numHorVirBndry; i++)
    {
      if ((yPos == horVirBndryPos[i]) || (yPos == horVirBndryPos[i] - 1))
      {
        disabledFlag = true;
        break;
      }
    }
    return disabledFlag;
  }
  Reshape        *m_pcReshape;
  BilateralFilter m_bilateralFilter;

  void applyCcSao(CodingStructure &cs, const PreCalcValues &pcv, const CPelUnitBuf &srcYuv, PelUnitBuf &dstYuv);

  inline void offsetmSampleCcSaoNoClipEdge(int startx, int endx, const Pel *srcY, const Pel *srcU, const Pel *srcV,
                                           const int chromaScaleX, const uint16_t edgeCmp, const ptrdiff_t srcStrideE,
                                           const int edgePosYA, const int edgePosXA, const int edgePosYB,
                                           const int edgePosXB, const int edgeThrVal, const uint16_t edgeIdc,
                                           const int edgeNumUni, const int bandCmp, const int bandNum,
                                           const int edgeNum, const int bitDepth, const short *offset, Pel *dst);
  inline void offsetmChromaSampleCcSaoNoClipEdge(int startx, int endx, const Pel *srcY, const Pel *srcU,
                                                 const Pel *srcV, const int chromaScaleX, const uint16_t edgeCmp,
                                                 const ptrdiff_t srcStrideE, const int edgePosYA, const int edgePosXA,
                                                 const int edgePosYB, const int edgePosXB, const int edgeThrVal,
                                                 const uint16_t edgeIdc, const int edgeNumUni, const int bandCmp,
                                                 const int bandNum, const int edgeNum, const int bitDepth,
                                                 const short *offset, Pel *dst);

  void offsetBlockCcSaoNoClipEdge(const CompID compID, const ChromaFormat chromaFormat, const int bitDepth,
                                  const ClpRng &clpRng, const uint16_t edgeCmp, uint16_t edgeDir,
                                  const uint16_t bandIdc, const uint16_t edgeThr, const uint16_t edgeIdc,
                                  const short *offset, const Pel *srcY, const Pel *srcU, const Pel *srcV, Pel *dst,
                                  const ptrdiff_t srcStrideY, const ptrdiff_t srcStrideU, const ptrdiff_t srcStrideV,
                                  const ptrdiff_t dstStride, const int width, const int height, bool isLeftAvail,
                                  bool isRightAvail, bool isAboveAvail, bool isBelowAvail, bool isAboveLeftAvail,
                                  bool isAboveRightAvail, bool isBelowLeftAvail, bool isBelowRightAvail);
  void offsetCTUCcSaoNoClip(CodingStructure &cs, const UnitArea &area, const CPelUnitBuf &srcYuv, PelUnitBuf &dstYuv,
                            const int ctuRsAddr);

  void offsetBlockCcSaoNoClip(const CompID compID, const ChromaFormat chromaFormat, const int bitDepth,
                              const ClpRng &clpRng, const uint16_t candPosY, const uint16_t bandNumY,
                              const uint16_t bandNumU, const uint16_t bandNumV, const short *offset, const Pel *srcY,
                              const Pel *srcU, const Pel *srcV, Pel *dst, const ptrdiff_t srcStrideY,
                              const ptrdiff_t srcStrideU, const ptrdiff_t srcStrideV, const ptrdiff_t dstStride,
                              const int width, const int height, bool isLeftAvail, bool isRightAvail, bool isAboveAvail,
                              bool isBelowAvail, bool isAboveLeftAvail, bool isAboveRightAvail, bool isBelowLeftAvail,
                              bool isBelowRightAvail);

  inline void offsetmSampleCcSaoNoClip(int startx, int endx, const Pel *srcY, const Pel *srcU, const Pel *srcV,
                                       const int chromaScaleX, const ptrdiff_t srcStrideY, const int candPosYX,
                                       const int candPosYY, const uint16_t bandNumY, const uint16_t bandNumU,
                                       const uint16_t bandNumV, const int bitDepth, const short *offset, Pel *dst);

  inline void offsetmChromaSampleCcSaoNoClip(int startx, int endx, const Pel *srcY, const Pel *srcU, const Pel *srcV,
                                             const int chromaScaleX, const ptrdiff_t srcStrideY, const int candPosYX,
                                             const int candPosYY, const uint16_t bandNumY, const uint16_t bandNumU,
                                             const uint16_t bandNumV, const int bitDepth, const short *offset,
                                             Pel *dst);
  // for CCSAO
  void (*m_getCcSaomSampleStats)(int startx, int endx, const int bitDepth, int64_t *diff, uint32_t *count,
                                 const int chromaScaleX, const uint16_t bandNumY, const uint16_t bandNumU,
                                 const uint16_t bandNumV, const Pel *srcY, const Pel *srcU, const Pel *srcV,
                                 const Pel *org, const Pel *dst);

  void (*m_getCcSaomChromaSampleStats)(int startx, int endx, const int bitDepth, int64_t *diff, uint32_t *count,
                                       const int chromaScaleX, const uint16_t bandNumY, const uint16_t bandNumU,
                                       const uint16_t bandNumV, const Pel *srcY, const Pel *srcU, const Pel *srcV,
                                       const Pel *org, const Pel *dst);

#ifdef TARGET_SIMD_X86
  void                         initCCSAOX86();
  template<X86_VEXT vext> void _initCCSAOX86();
#endif

protected:
  uint32_t   m_offsetStepLog2[MAX_NUM_COMP]; // offset step
  PelStorage m_tempBuf;
  uint32_t   m_numberOfComponents;

  std::vector<int8_t> m_signLineBuf1;
  std::vector<int8_t> m_signLineBuf2;

  bool       m_created = false;
  PelStorage m_ccSaoBuf;
  int        m_picWidth;
  int        m_picHeight;
  int        m_maxCUWidth;
  int        m_maxCUHeight;
  int        m_numCTUsInWidth;
  int        m_numCTUsInHeight;
  int        m_numCTUsInPic;

  CcSaoComParam m_ccSaoComParam;
  uint8_t      *m_ccSaoControl[MAX_NUM_COMP];

private:
  bool m_picSAOEnabled[MAX_NUM_COMP];

  std::vector<CcSaoPrvParam> m_ccSaoPrvParamDec[MAX_NUM_COMP];
};
//! \}
#endif
