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

/** \file     AdaptiveLoopFilter.cpp
    \brief    adaptive loop filter class
*/

#include "AdaptiveLoopFilter.h"
#include "CodingStructure.h"
#include "Picture.h"
#include <array>
#include <cmath>

const EnumArray<int, ChannelType> AdaptiveLoopFilter::ALF_NUM_CLIP_VALS = { 4, 4 };

AdaptiveLoopFilter::AdaptiveLoopFilter() : m_modes(nullptr) {}

bool AdaptiveLoopFilter::isCrossedBySubPicBoundaries(const CodingStructure &cs, const int xPos, const int yPos,
                                                     const int width, const int height, bool &clipTop, bool &clipBottom,
                                                     bool &clipLeft, bool &clipRight, int &rasterSliceAlfPad)
{
  clipTop        = false;
  clipBottom     = false;
  clipLeft       = false;
  clipRight      = false;
  const PPS *pps = cs.pps;

  const Slice      &slice   = *(cs.slice);
  int               ctuSize = slice.m_sps->m_ctuSize;
  const Position    currCtuPos(xPos, yPos);
  const CodingUnit *currCtu   = cs.getCU(currCtuPos, ChannelType::LUMA);
  const SubPic     &curSubPic = slice.m_pps->getSubPicFromPos(currCtuPos);

  bool loopFilterAcrossSubPicEnabledFlag = curSubPic.m_loopFilterAcrossSubPicEnabledFlag;
  // top
  if (yPos >= ctuSize && clipTop == false)
  {
    const Position    prevCtuPos(xPos, yPos - ctuSize);
    const CodingUnit *prevCtu = cs.getCU(prevCtuPos, ChannelType::LUMA);
    if ((!pps->m_loopFilterAcrossSlicesEnabledFlag && !CU::isSameSlice(*currCtu, *prevCtu)) ||
        (!pps->m_loopFilterAcrossTilesEnabledFlag && !CU::isSameTile(*currCtu, *prevCtu)) ||
        (!loopFilterAcrossSubPicEnabledFlag && !CU::isSameSubPic(*currCtu, *prevCtu)))
    {
      clipTop = true;
    }
  }

  // bottom
  if (yPos + ctuSize < cs.pcv->lumaHeight && clipBottom == false)
  {
    const Position    nextCtuPos(xPos, yPos + ctuSize);
    const CodingUnit *nextCtu = cs.getCU(nextCtuPos, ChannelType::LUMA);
    if ((!pps->m_loopFilterAcrossSlicesEnabledFlag && !CU::isSameSlice(*currCtu, *nextCtu)) ||
        (!pps->m_loopFilterAcrossTilesEnabledFlag && !CU::isSameTile(*currCtu, *nextCtu)) ||
        (!loopFilterAcrossSubPicEnabledFlag && !CU::isSameSubPic(*currCtu, *nextCtu)))
    {
      clipBottom = true;
    }
  }

  // left
  if (xPos >= ctuSize && clipLeft == false)
  {
    const Position    prevCtuPos(xPos - ctuSize, yPos);
    const CodingUnit *prevCtu = cs.getCU(prevCtuPos, ChannelType::LUMA);
    if ((!pps->m_loopFilterAcrossSlicesEnabledFlag && !CU::isSameSlice(*currCtu, *prevCtu)) ||
        (!pps->m_loopFilterAcrossTilesEnabledFlag && !CU::isSameTile(*currCtu, *prevCtu)) ||
        (!loopFilterAcrossSubPicEnabledFlag && !CU::isSameSubPic(*currCtu, *prevCtu)))
    {
      clipLeft = true;
    }
  }

  // right
  if (xPos + ctuSize < cs.pcv->lumaWidth && clipRight == false)
  {
    const Position    nextCtuPos(xPos + ctuSize, yPos);
    const CodingUnit *nextCtu = cs.getCU(nextCtuPos, ChannelType::LUMA);
    if ((!pps->m_loopFilterAcrossSlicesEnabledFlag && !CU::isSameSlice(*currCtu, *nextCtu)) ||
        (!pps->m_loopFilterAcrossTilesEnabledFlag && !CU::isSameTile(*currCtu, *nextCtu)) ||
        (!loopFilterAcrossSubPicEnabledFlag && !CU::isSameSubPic(*currCtu, *nextCtu)))
    {
      clipRight = true;
    }
  }

  rasterSliceAlfPad = 0;
  if (!clipTop && !clipLeft)
  {
    // top-left CTU
    if (xPos >= ctuSize && yPos >= ctuSize)
    {
      const Position    prevCtuPos(xPos - ctuSize, yPos - ctuSize);
      const CodingUnit *prevCtu = cs.getCU(prevCtuPos, ChannelType::LUMA);
      if (!pps->m_loopFilterAcrossSlicesEnabledFlag && !CU::isSameSlice(*currCtu, *prevCtu))
      {
        rasterSliceAlfPad = 1;
      }
    }
  }

  if (!clipBottom && !clipRight)
  {
    // bottom-right CTU
    if (xPos + ctuSize < cs.pcv->lumaWidth && yPos + ctuSize < cs.pcv->lumaHeight)
    {
      const Position    nextCtuPos(xPos + ctuSize, yPos + ctuSize);
      const CodingUnit *nextCtu = cs.getCU(nextCtuPos, ChannelType::LUMA);
      if (!pps->m_loopFilterAcrossSlicesEnabledFlag && !CU::isSameSlice(*currCtu, *nextCtu))
      {
        rasterSliceAlfPad += 2;
      }
    }
  }

  return clipTop || clipBottom || clipLeft || clipRight || rasterSliceAlfPad;
}

void AdaptiveLoopFilter::reconstructCoeffSingleFilter(const short *const coeffSrc, const Pel *const clippSrc,
                                                      short *const coeffDst, Pel *const clippDst,
                                                      const std::array<Pel, MAX_ALF_NUM_CLIP_VALS> &clipVals,
                                                      const size_t numCoeffMinus1, const bool nonLinearFlag,
                                                      const bool isRdo, const short factor)
{
  for (size_t coeffIdx = 0; coeffIdx < numCoeffMinus1; ++coeffIdx)
  {
    const Pel clipIdx = nonLinearFlag ? clippSrc[coeffIdx] : 0;
    CHECK(!(clipIdx >= 0 && clipIdx < MAX_ALF_NUM_CLIP_VALS), "Bad clip idx in ALF");

    coeffDst[coeffIdx] = coeffSrc[coeffIdx];
    clippDst[coeffIdx] = isRdo ? clipIdx : clipVals[clipIdx];
  }
  coeffDst[numCoeffMinus1] = factor;
  clippDst[numCoeffMinus1] = isRdo ? 0 : clipVals[0];
}

void AdaptiveLoopFilter::create(const int picWidth, const int picHeight, const ChromaFormat format,
                                const int maxCUWidth, const int maxCUHeight, const int maxCUDepth,
                                const BitDepths &inputBitDepth)
{
  destroy();
  m_inputBitDepth   = inputBitDepth;
  m_picWidth        = picWidth;
  m_picHeight       = picHeight;
  m_picWidthChroma  = m_picWidth >> getChannelTypeScaleX(ChannelType::CHROMA, format);
  m_picHeightChroma = m_picHeight >> getChannelTypeScaleY(ChannelType::CHROMA, format);
  m_maxCUWidth      = maxCUWidth;
  m_maxCUHeight     = maxCUHeight;
  m_maxCUDepth      = maxCUDepth;
  m_chromaFormat    = format;

  m_numCTUsInWidth = (m_picWidth + (m_maxCUWidth - 1)) / m_maxCUWidth;
  m_numCTUsInPic   = ((m_picHeight + (m_maxCUHeight - 1)) / m_maxCUHeight) * m_numCTUsInWidth;

  CHECK(ALF_NUM_CLIP_VALS[ChannelType::LUMA] < 1, "ALF_NUM_CLIP_VALS[ChannelType::LUMA] must be at least one");
  m_alfClippingValues[ChannelType::LUMA][0] = 1 << m_inputBitDepth[ChannelType::LUMA];
  int shiftLuma                             = m_inputBitDepth[ChannelType::LUMA] - 8;
  for (int i = 1; i < ALF_NUM_CLIP_VALS[ChannelType::LUMA]; ++i)
  {
    m_alfClippingValues[ChannelType::LUMA][i] = 1 << (7 - 2 * i + shiftLuma);
  }
  CHECK(ALF_NUM_CLIP_VALS[ChannelType::CHROMA] < 1, "ALF_NUM_CLIP_VALS[ChannelType::CHROMA] must be at least one");
  m_alfClippingValues[ChannelType::CHROMA][0] = 1 << m_inputBitDepth[ChannelType::CHROMA];
  int shiftChroma                             = m_inputBitDepth[ChannelType::CHROMA] - 8;
  for (int i = 1; i < ALF_NUM_CLIP_VALS[ChannelType::CHROMA]; ++i)
  {
    m_alfClippingValues[ChannelType::CHROMA][i] = 1 << (7 - 2 * i + shiftChroma);
  }
  if (m_created)
  {
    return;
  }

  m_created = true;

  m_ccAlfFilterControl[0] = new uint8_t[m_numCTUsInPic];
  m_ccAlfFilterControl[1] = new uint8_t[m_numCTUsInPic];
}

void AdaptiveLoopFilter::destroy()
{
  if (!m_created)
  {
    return;
  }

  m_tempBuf.destroy();
  m_tempBuf2.destroy();

  m_created = false;

  if (m_ccAlfFilterControl[0])
  {
    delete[] m_ccAlfFilterControl[0];
    m_ccAlfFilterControl[0] = nullptr;
  }

  if (m_ccAlfFilterControl[1])
  {
    delete[] m_ccAlfFilterControl[1];
    m_ccAlfFilterControl[1] = nullptr;
  }
}
