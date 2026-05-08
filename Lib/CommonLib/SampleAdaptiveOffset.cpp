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

/** \file     SampleAdaptiveOffset.cpp
    \brief    sample adaptive offset class
*/

#include "SampleAdaptiveOffset.h"

#include "UnitTools.h"
#include "UnitPartitioner.h"
#include "CodingStructure.h"
#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/dtrace_buffer.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

//! \ingroup CommonLib
//! \{

SAOOffset::SAOOffset() { reset(); }

SAOOffset::~SAOOffset() {}

void SAOOffset::reset()
{
  modeIdc         = SAOMode::OFF;
  typeIdc.newType = SAOModeNewTypes::NONE;
  typeAuxInfo     = -1;
  ::memset(offset, 0, sizeof(int) * MAX_NUM_SAO_CLASSES);
}

const SAOOffset &SAOOffset::operator=(const SAOOffset &src)
{
  modeIdc     = src.modeIdc;
  typeIdc     = src.typeIdc;
  typeAuxInfo = src.typeAuxInfo;
  ::memcpy(offset, src.offset, sizeof(int) * MAX_NUM_SAO_CLASSES);

  return *this;
}

SAOBlkParam::SAOBlkParam() { reset(); }

SAOBlkParam::~SAOBlkParam() {}

void SAOBlkParam::reset()
{
  for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
  {
    offsetParam[compIdx].reset();
  }
}

const SAOBlkParam &SAOBlkParam::operator=(const SAOBlkParam &src)
{
  for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
  {
    offsetParam[compIdx] = src.offsetParam[compIdx];
  }
  return *this;
}

SampleAdaptiveOffset::SampleAdaptiveOffset()
{
  m_numberOfComponents = 0;
  m_ccSaoControl[0] = m_ccSaoControl[1] = m_ccSaoControl[2] = nullptr;
}

SampleAdaptiveOffset::~SampleAdaptiveOffset()
{
  destroy();

  m_signLineBuf1.clear();
  m_signLineBuf2.clear();
}

void SampleAdaptiveOffset::create(int picWidth, int picHeight, ChromaFormat format, uint32_t maxCUWidth,
                                  uint32_t maxCUHeight, uint32_t maxCUDepth, uint32_t lumaBitShift,
                                  uint32_t chromaBitShift)
{
  // temporary picture buffer
  UnitArea picArea(format, Area(0, 0, picWidth, picHeight));

  m_tempBuf.destroy();
  m_tempBuf.create(picArea);

  // bit-depth related
  for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
  {
    m_offsetStepLog2[compIdx] = isLuma(CompID(compIdx)) ? lumaBitShift : chromaBitShift;
  }
  m_numberOfComponents = getNumberValidComponents(format);
  if (m_created)
  {
    return;
  }
  m_created = true;

  m_ccSaoBuf.destroy();
  m_ccSaoBuf.create(format, Area(0, 0, picWidth, picHeight), maxCUWidth, MAX_CCSAO_FILTER_LENGTH >> 1, 0, false);

  m_picWidth    = picWidth;
  m_picHeight   = picHeight;
  m_maxCUWidth  = maxCUWidth;
  m_maxCUHeight = maxCUHeight;

  m_numCTUsInWidth  = (m_picWidth / m_maxCUWidth) + ((m_picWidth % m_maxCUWidth) ? 1 : 0);
  m_numCTUsInHeight = (m_picHeight / m_maxCUHeight) + ((m_picHeight % m_maxCUHeight) ? 1 : 0);
  m_numCTUsInPic    = m_numCTUsInHeight * m_numCTUsInWidth;

  for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
  {
    if (m_ccSaoControl[compIdx])
    {
      delete[] m_ccSaoControl[compIdx];
      m_ccSaoControl[compIdx] = nullptr;
    }
  }

  for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
  {
    if (m_ccSaoControl[compIdx] == nullptr)
    {
      m_ccSaoControl[compIdx] = new uint8_t[m_numCTUsInPic];
    }
    ::memset(m_ccSaoControl[compIdx], 0, sizeof(uint8_t) * m_numCTUsInPic);
  }
  m_bilateralFilter.create();
}

void SampleAdaptiveOffset::destroy()
{
  m_tempBuf.destroy();

  if (!m_created)
  {
    return;
  }
  m_created = false;

  m_ccSaoBuf.destroy();

  for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
  {
    if (m_ccSaoControl[compIdx])
    {
      delete[] m_ccSaoControl[compIdx];
      m_ccSaoControl[compIdx] = nullptr;
    }
  }
  m_bilateralFilter.destroy();
}

void SampleAdaptiveOffset::invertQuantOffsets(CompID compIdx, SAOModeNewTypes typeIdc, int typeAuxInfo, int *dstOffsets,
                                              int *srcOffsets)
{
  int codedOffset[MAX_NUM_SAO_CLASSES];

  ::memcpy(codedOffset, srcOffsets, sizeof(int) * MAX_NUM_SAO_CLASSES);
  ::memset(dstOffsets, 0, sizeof(int) * MAX_NUM_SAO_CLASSES);

  if (typeIdc == SAOModeNewTypes::START_BO)
  {
    for (int i = 0; i < 4; i++)
    {
      dstOffsets[(typeAuxInfo + i) % NUM_SAO_BO_CLASSES] =
        codedOffset[(typeAuxInfo + i) % NUM_SAO_BO_CLASSES] * (1 << m_offsetStepLog2[compIdx]);
    }
  }
  else // EO
  {
    for (int i = 0; i < NUM_SAO_EO_CLASSES; i++)
    {
      dstOffsets[i] = codedOffset[i] * (1 << m_offsetStepLog2[compIdx]);
    }
    CHECK(dstOffsets[SAO_CLASS_EO_PLAIN] != 0, "EO offset is not '0'"); // keep EO plain offset as zero
  }
}

int SampleAdaptiveOffset::getMergeList(CodingStructure &cs, int ctuRsAddr, SAOBlkParam *blkParams,
                                       MergeBlkParams &mergeList)
{
  const PreCalcValues &pcv = *cs.pcv;

  int               ctuX = ctuRsAddr % pcv.widthInCtus;
  int               ctuY = ctuRsAddr / pcv.widthInCtus;
  const CodingUnit &cu   = *cs.getCU(Position(ctuX * pcv.maxCUWidth, ctuY * pcv.maxCUHeight), ChannelType::LUMA);
  int               mergedCTUPos;
  int               numValidMergeCandidates = 0;

  for (const auto mergeType: { SAOModeMergeTypes::LEFT, SAOModeMergeTypes::ABOVE })
  {
    SAOBlkParam *mergeCandidate = nullptr;

    switch (mergeType)
    {
    case SAOModeMergeTypes::ABOVE:
      if (ctuY > 0)
      {
        mergedCTUPos = ctuRsAddr - pcv.widthInCtus;
        if (cs.getCURestricted(Position(ctuX * pcv.maxCUWidth, (ctuY - 1) * pcv.maxCUHeight), cu, cu.chType))
        {
          mergeCandidate = &(blkParams[mergedCTUPos]);
        }
      }
      break;
    case SAOModeMergeTypes::LEFT:
      if (ctuX > 0)
      {
        mergedCTUPos = ctuRsAddr - 1;
        if (cs.getCURestricted(Position((ctuX - 1) * pcv.maxCUWidth, ctuY * pcv.maxCUHeight), cu, cu.chType))
        {
          mergeCandidate = &(blkParams[mergedCTUPos]);
        }
      }
      break;
    default:
      THROW("not a supported merge type");
      break;
    }

    mergeList[mergeType] = mergeCandidate;
    if (mergeCandidate != nullptr)
    {
      numValidMergeCandidates++;
    }
  }

  return numValidMergeCandidates;
}

void SampleAdaptiveOffset::reconstructBlkSAOParam(SAOBlkParam &recParam, MergeBlkParams &mergeList)
{
  const int numberOfComponents = m_numberOfComponents;
  for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    const CompID component   = CompID(compIdx);
    SAOOffset   &offsetParam = recParam[component];

    if (offsetParam.modeIdc == SAOMode::OFF)
    {
      continue;
    }

    switch (offsetParam.modeIdc)
    {
    case SAOMode::NEW:
      invertQuantOffsets(component, offsetParam.typeIdc.newType, offsetParam.typeAuxInfo, offsetParam.offset,
                         offsetParam.offset);
      break;
    case SAOMode::MERGE:
      {
        SAOBlkParam *mergeTarget = mergeList[offsetParam.typeIdc.mergeType];
        CHECK(mergeTarget == nullptr, "Merge target does not exist");

        offsetParam = (*mergeTarget)[component];
        break;
      }
    default:
      THROW("Not a supported mode");
      break;
    }
  }
}

void SampleAdaptiveOffset::xReconstructBlkSAOParams(CodingStructure &cs, SAOBlkParam *saoBlkParams)
{
  for (uint32_t compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
  {
    m_picSAOEnabled[compIdx] = false;
  }

  const uint32_t numberOfComponents = getNumberValidComponents(cs.pcv->chrFormat);

  for (int ctuRsAddr = 0; ctuRsAddr < cs.pcv->sizeInCtus; ctuRsAddr++)
  {
    MergeBlkParams mergeList;
    mergeList.fill(nullptr);
    getMergeList(cs, ctuRsAddr, saoBlkParams, mergeList);

    reconstructBlkSAOParam(saoBlkParams[ctuRsAddr], mergeList);

    for (uint32_t compIdx = 0; compIdx < numberOfComponents; compIdx++)
    {
      if (saoBlkParams[ctuRsAddr][compIdx].modeIdc != SAOMode::OFF)
      {
        m_picSAOEnabled[compIdx] = true;
      }
    }
  }
}

void SampleAdaptiveOffset::offsetBlock(const int channelBitDepth, const ClpRng &clpRng, SAOModeNewTypes typeIdx,
                                       int *offset, const Pel *srcBlk, Pel *resBlk, ptrdiff_t srcStride,
                                       ptrdiff_t resStride, int width, int height, bool isLeftAvail, bool isRightAvail,
                                       bool isAboveAvail, bool isBelowAvail, bool isAboveLeftAvail,
                                       bool isAboveRightAvail, bool isBelowLeftAvail, bool isBelowRightAvail,
                                       int horVirBndryPos[], int verVirBndryPos[], int numHorVirBndry,
                                       int numVerVirBndry)
{
  int    x, y, startX, startY, endX, endY, edgeType;
  int    firstLineStartX, firstLineEndX, lastLineStartX, lastLineEndX;
  int8_t signLeft, signRight, signDown;

  const Pel *srcLine = srcBlk;
  Pel       *resLine = resBlk;

  switch (typeIdx)
  {
  case SAOModeNewTypes::EO_0:
    {
      offset += 2;
      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width - 1);
      for (y = 0; y < height; y++)
      {
        signLeft = (int8_t)sgn(srcLine[startX] - srcLine[startX - 1]);
        for (x = startX; x < endX; x++)
        {
          signRight = (int8_t)sgn(srcLine[x] - srcLine[x + 1]);
          edgeType  = signRight + signLeft;
          signLeft  = -signRight;

          resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
        }
        srcLine += srcStride;
        resLine += resStride;
      }
    }
    break;
  case SAOModeNewTypes::EO_90:
    {
      offset += 2;
      int8_t *signUpLine = m_signLineBuf1.data();

      startY = isAboveAvail ? 0 : 1;
      endY   = isBelowAvail ? height : height - 1;
      if (!isAboveAvail)
      {
        srcLine += srcStride;
        resLine += resStride;
      }

      const Pel *srcLineAbove = srcLine - srcStride;
      for (x = 0; x < width; x++)
      {
        signUpLine[x] = (int8_t)sgn(srcLine[x] - srcLineAbove[x]);
      }

      const Pel *srcLineBelow;
      for (y = startY; y < endY; y++)
      {
        srcLineBelow = srcLine + srcStride;

        for (x = 0; x < width; x++)
        {
          signDown      = (int8_t)sgn(srcLine[x] - srcLineBelow[x]);
          edgeType      = signDown + signUpLine[x];
          signUpLine[x] = -signDown;

          resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
        }
        srcLine += srcStride;
        resLine += resStride;
      }
    }
    break;
  case SAOModeNewTypes::EO_135:
    {
      offset += 2;
      int8_t *signTmpLine;

      int8_t *signUpLine   = m_signLineBuf1.data();
      int8_t *signDownLine = m_signLineBuf2.data();

      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width - 1);

      // prepare 2nd line's upper sign
      const Pel *srcLineBelow = srcLine + srcStride;
      for (x = startX; x < endX + 1; x++)
      {
        signUpLine[x] = (int8_t)sgn(srcLineBelow[x] - srcLine[x - 1]);
      }

      // 1st line
      const Pel *srcLineAbove = srcLine - srcStride;
      firstLineStartX         = isAboveLeftAvail ? 0 : 1;
      firstLineEndX           = isAboveAvail ? endX : 1;
      for (x = firstLineStartX; x < firstLineEndX; x++)
      {
        edgeType = sgn(srcLine[x] - srcLineAbove[x - 1]) - signUpLine[x + 1];

        resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
      }
      srcLine += srcStride;
      resLine += resStride;

      // middle lines
      for (y = 1; y < height - 1; y++)
      {
        srcLineBelow = srcLine + srcStride;

        for (x = startX; x < endX; x++)
        {
          signDown   = (int8_t)sgn(srcLine[x] - srcLineBelow[x + 1]);
          edgeType   = signDown + signUpLine[x];
          resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);

          signDownLine[x + 1] = -signDown;
        }
        signDownLine[startX] = (int8_t)sgn(srcLineBelow[startX] - srcLine[startX - 1]);

        signTmpLine  = signUpLine;
        signUpLine   = signDownLine;
        signDownLine = signTmpLine;

        srcLine += srcStride;
        resLine += resStride;
      }

      // last line
      srcLineBelow   = srcLine + srcStride;
      lastLineStartX = isBelowAvail ? startX : (width - 1);
      lastLineEndX   = isBelowRightAvail ? width : (width - 1);
      for (x = lastLineStartX; x < lastLineEndX; x++)
      {
        edgeType   = sgn(srcLine[x] - srcLineBelow[x + 1]) + signUpLine[x];
        resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
      }
    }
    break;
  case SAOModeNewTypes::EO_45:
    {
      offset += 2;
      int8_t *signUpLine = m_signLineBuf1.data() + 1;

      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width - 1);

      // prepare 2nd line upper sign
      const Pel *srcLineBelow = srcLine + srcStride;
      for (x = startX - 1; x < endX; x++)
      {
        signUpLine[x] = (int8_t)sgn(srcLineBelow[x] - srcLine[x + 1]);
      }

      // first line
      const Pel *srcLineAbove = srcLine - srcStride;
      firstLineStartX         = isAboveAvail ? startX : (width - 1);
      firstLineEndX           = isAboveRightAvail ? width : (width - 1);
      for (x = firstLineStartX; x < firstLineEndX; x++)
      {
        edgeType   = sgn(srcLine[x] - srcLineAbove[x + 1]) - signUpLine[x - 1];
        resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
      }
      srcLine += srcStride;
      resLine += resStride;

      // middle lines
      for (y = 1; y < height - 1; y++)
      {
        srcLineBelow = srcLine + srcStride;

        for (x = startX; x < endX; x++)
        {
          signDown          = (int8_t)sgn(srcLine[x] - srcLineBelow[x - 1]);
          edgeType          = signDown + signUpLine[x];
          resLine[x]        = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
          signUpLine[x - 1] = -signDown;
        }
        signUpLine[endX - 1] = (int8_t)sgn(srcLineBelow[endX - 1] - srcLine[endX]);
        srcLine += srcStride;
        resLine += resStride;
      }

      // last line
      srcLineBelow   = srcLine + srcStride;
      lastLineStartX = isBelowLeftAvail ? 0 : 1;
      lastLineEndX   = isBelowAvail ? endX : 1;
      for (x = lastLineStartX; x < lastLineEndX; x++)
      {
        edgeType   = sgn(srcLine[x] - srcLineBelow[x - 1]) + signUpLine[x];
        resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
      }
    }
    break;
  case SAOModeNewTypes::BO:
    {
      const int shiftBits = channelBitDepth - NUM_SAO_BO_CLASSES_LOG2;
      for (y = 0; y < height; y++)
      {
        for (x = 0; x < width; x++)
        {
          resLine[x] = ClipPel<int>(srcLine[x] + offset[srcLine[x] >> shiftBits], clpRng);
        }
        srcLine += srcStride;
        resLine += resStride;
      }
    }
    break;
  default:
    {
      THROW("Not a supported SAO types\n");
    }
  }
}

void SampleAdaptiveOffset::offsetBlockNoClip(const int channelBitDepth, const ClpRng &clpRng, SAOModeNewTypes typeIdx,
                                             int *offset, const Pel *srcBlk, Pel *resBlk, ptrdiff_t srcStride,
                                             ptrdiff_t resStride, int width, int height, bool isLeftAvail,
                                             bool isRightAvail, bool isAboveAvail, bool isBelowAvail,
                                             bool isAboveLeftAvail, bool isAboveRightAvail, bool isBelowLeftAvail,
                                             bool isBelowRightAvail, int horVirBndryPos[], int verVirBndryPos[],
                                             int numHorVirBndry, int numVerVirBndry)
{
  int    x, y, startX, startY, endX, endY, edgeType;
  int    firstLineStartX, firstLineEndX, lastLineStartX, lastLineEndX;
  int8_t signLeft, signRight, signDown;

  const Pel *srcLine = srcBlk;
  Pel       *resLine = resBlk;

  switch (typeIdx)
  {
  case SAOModeNewTypes::EO_0:
    {
      offset += 2;
      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width - 1);
      for (y = 0; y < height; y++)
      {
        signLeft = (int8_t)sgn(srcLine[startX] - srcLine[startX - 1]);
        for (x = startX; x < endX; x++)
        {
          signRight = (int8_t)sgn(srcLine[x] - srcLine[x + 1]);
          edgeType  = signRight + signLeft;
          signLeft  = -signRight;

          resLine[x] = srcLine[x] + offset[edgeType];
        }
        srcLine += srcStride;
        resLine += resStride;
      }
    }
    break;
  case SAOModeNewTypes::EO_90:
    {
      offset += 2;
      int8_t *signUpLine = &m_signLineBuf1[0];

      startY = isAboveAvail ? 0 : 1;
      endY   = isBelowAvail ? height : height - 1;
      if (!isAboveAvail)
      {
        srcLine += srcStride;
        resLine += resStride;
      }

      const Pel *srcLineAbove = srcLine - srcStride;
      for (x = 0; x < width; x++)
      {
        signUpLine[x] = (int8_t)sgn(srcLine[x] - srcLineAbove[x]);
      }

      const Pel *srcLineBelow;
      for (y = startY; y < endY; y++)
      {
        srcLineBelow = srcLine + srcStride;

        for (x = 0; x < width; x++)
        {
          signDown      = (int8_t)sgn(srcLine[x] - srcLineBelow[x]);
          edgeType      = signDown + signUpLine[x];
          signUpLine[x] = -signDown;

          resLine[x] = srcLine[x] + offset[edgeType];
        }
        srcLine += srcStride;
        resLine += resStride;
      }
    }
    break;
  case SAOModeNewTypes::EO_135:
    {
      offset += 2;
      int8_t *signUpLine, *signDownLine, *signTmpLine;

      signUpLine   = &m_signLineBuf1[0];
      signDownLine = &m_signLineBuf2[0];

      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width - 1);

    // prepare 2nd line's upper sign
      const Pel *srcLineBelow = srcLine + srcStride;
      for (x = startX; x < endX + 1; x++)
      {
        signUpLine[x] = (int8_t)sgn(srcLineBelow[x] - srcLine[x - 1]);
      }

    // 1st line
      const Pel *srcLineAbove = srcLine - srcStride;
      firstLineStartX         = isAboveLeftAvail ? 0 : 1;
      firstLineEndX           = isAboveAvail ? endX : 1;
      for (x = firstLineStartX; x < firstLineEndX; x++)
      {
        edgeType = sgn(srcLine[x] - srcLineAbove[x - 1]) - signUpLine[x + 1];

        resLine[x] = srcLine[x] + offset[edgeType];
      }
      srcLine += srcStride;
      resLine += resStride;

    // middle lines
      for (y = 1; y < height - 1; y++)
      {
        srcLineBelow = srcLine + srcStride;

        for (x = startX; x < endX; x++)
        {
          signDown   = (int8_t)sgn(srcLine[x] - srcLineBelow[x + 1]);
          edgeType   = signDown + signUpLine[x];
          resLine[x] = srcLine[x] + offset[edgeType];

          signDownLine[x + 1] = -signDown;
        }
        signDownLine[startX] = (int8_t)sgn(srcLineBelow[startX] - srcLine[startX - 1]);

        signTmpLine  = signUpLine;
        signUpLine   = signDownLine;
        signDownLine = signTmpLine;

        srcLine += srcStride;
        resLine += resStride;
      }

    // last line
      srcLineBelow   = srcLine + srcStride;
      lastLineStartX = isBelowAvail ? startX : (width - 1);
      lastLineEndX   = isBelowRightAvail ? width : (width - 1);
      for (x = lastLineStartX; x < lastLineEndX; x++)
      {
        edgeType   = sgn(srcLine[x] - srcLineBelow[x + 1]) + signUpLine[x];
        resLine[x] = srcLine[x] + offset[edgeType];
      }
    }
    break;
  case SAOModeNewTypes::EO_45:
    {
      offset += 2;
      int8_t *signUpLine = &m_signLineBuf1[1];

      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width - 1);

    // prepare 2nd line upper sign
      const Pel *srcLineBelow = srcLine + srcStride;
      for (x = startX - 1; x < endX; x++)
      {
        signUpLine[x] = (int8_t)sgn(srcLineBelow[x] - srcLine[x + 1]);
      }

    // first line
      const Pel *srcLineAbove = srcLine - srcStride;
      firstLineStartX         = isAboveAvail ? startX : (width - 1);
      firstLineEndX           = isAboveRightAvail ? width : (width - 1);
      for (x = firstLineStartX; x < firstLineEndX; x++)
      {
        edgeType   = sgn(srcLine[x] - srcLineAbove[x + 1]) - signUpLine[x - 1];
        resLine[x] = srcLine[x] + offset[edgeType];
      }
      srcLine += srcStride;
      resLine += resStride;

    // middle lines
      for (y = 1; y < height - 1; y++)
      {
        srcLineBelow = srcLine + srcStride;

        for (x = startX; x < endX; x++)
        {
          signDown          = (int8_t)sgn(srcLine[x] - srcLineBelow[x - 1]);
          edgeType          = signDown + signUpLine[x];
          resLine[x]        = srcLine[x] + offset[edgeType];
          signUpLine[x - 1] = -signDown;
        }
        signUpLine[endX - 1] = (int8_t)sgn(srcLineBelow[endX - 1] - srcLine[endX]);
        srcLine += srcStride;
        resLine += resStride;
      }

    // last line
      srcLineBelow   = srcLine + srcStride;
      lastLineStartX = isBelowLeftAvail ? 0 : 1;
      lastLineEndX   = isBelowAvail ? endX : 1;
      for (x = lastLineStartX; x < lastLineEndX; x++)
      {
        edgeType   = sgn(srcLine[x] - srcLineBelow[x - 1]) + signUpLine[x];
        resLine[x] = srcLine[x] + offset[edgeType];
      }
    }
    break;
  case SAOModeNewTypes::BO:
    {
      const int shiftBits = channelBitDepth - NUM_SAO_BO_CLASSES_LOG2;
      for (y = 0; y < height; y++)
      {
        for (x = 0; x < width; x++)
        {
          resLine[x] = srcLine[x] + offset[srcLine[x] >> shiftBits];
        }
        srcLine += srcStride;
        resLine += resStride;
      }
    }
    break;
  default:
    {
      THROW("Not a supported SAO types\n");
    }
  }
}

void SampleAdaptiveOffset::offsetCTU(const UnitArea &area, const CPelUnitBuf &src, PelUnitBuf &res,
                                     SAOBlkParam &saoblkParam, CodingStructure &cs)
{
  const uint32_t numberOfComponents = getNumberValidComponents(area.chromaFormat);

  bool allOff = true;
  for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    if (saoblkParam[compIdx].modeIdc != SAOMode::OFF)
    {
      allOff = false;
    }
  }
  if (allOff)
  {
    return;
  }

  bool isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail, isBelowLeftAvail,
    isBelowRightAvail;

  // block boundary availability
  deriveLoopFilterBoundaryAvailability(cs, area.Y(), isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail,
                                       isAboveLeftAvail, isAboveRightAvail, isBelowLeftAvail, isBelowRightAvail);

  const size_t lineBufferSize = area.lwidth() + 1;
  if (m_signLineBuf1.size() < lineBufferSize)
  {
    m_signLineBuf1.resize(lineBufferSize);
    m_signLineBuf2.resize(lineBufferSize);
  }

  int numHorVirBndry = 0, numVerVirBndry = 0;
  int horVirBndryPos[]     = { -1, -1, -1 };
  int verVirBndryPos[]     = { -1, -1, -1 };
  int horVirBndryPosComp[] = { -1, -1, -1 };
  int verVirBndryPosComp[] = { -1, -1, -1 };

  for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    const CompID    compID    = CompID(compIdx);
    const CompArea &compArea  = area.block(compID);
    SAOOffset      &ctbOffset = saoblkParam[compIdx];

    if (ctbOffset.modeIdc != SAOMode::OFF)
    {
      const ptrdiff_t srcStride = src.get(compID).stride;
      const ptrdiff_t resStride = res.get(compID).stride;

      const Pel *srcBlk = src.get(compID).bufAt(compArea);
      Pel       *resBlk = res.get(compID).bufAt(compArea);
      for (int i = 0; i < numHorVirBndry; i++)
      {
        horVirBndryPosComp[i] = (horVirBndryPos[i] >> ::getComponentScaleY(compID, area.chromaFormat)) - compArea.y;
      }
      for (int i = 0; i < numVerVirBndry; i++)
      {
        verVirBndryPosComp[i] = (verVirBndryPos[i] >> ::getComponentScaleX(compID, area.chromaFormat)) - compArea.x;
      }
      offsetBlock(cs.sps->m_bitDepths[toChannelType(compID)], cs.slice->clpRng(compID), ctbOffset.typeIdc.newType,
                  ctbOffset.offset, srcBlk, resBlk, srcStride, resStride, compArea.width, compArea.height, isLeftAvail,
                  isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail, isBelowLeftAvail,
                  isBelowRightAvail, horVirBndryPosComp, verVirBndryPosComp, numHorVirBndry, numVerVirBndry);
    }
  } // compIdx
}

void SampleAdaptiveOffset::offsetCTUnoClip(const UnitArea &area, const CPelUnitBuf &src, PelUnitBuf &res,
                                           SAOBlkParam &saoblkParam, CodingStructure &cs)
{
  const uint32_t numberOfComponents = getNumberValidComponents(area.chromaFormat);
  bool           bAllOff            = true;
  for (uint32_t compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    if (saoblkParam[compIdx].modeIdc != SAOMode::OFF)
    {
      bAllOff = false;
    }
  }
  if (bAllOff)
  {
    return;
  }

  bool isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail, isBelowLeftAvail,
    isBelowRightAvail;

  // block boundary availability
  deriveLoopFilterBoundaryAvailability(cs, area.Y(), isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail,
                                       isAboveLeftAvail, isAboveRightAvail, isBelowLeftAvail, isBelowRightAvail);
  const size_t lineBufferSize = area.lwidth() + 1;
  if (m_signLineBuf1.size() < lineBufferSize)
  {
    m_signLineBuf1.resize(lineBufferSize);
    m_signLineBuf2.resize(lineBufferSize);
  }
  int numHorVirBndry = 0, numVerVirBndry = 0;
  int horVirBndryPosComp[] = { -1, -1, -1 };
  int verVirBndryPosComp[] = { -1, -1, -1 };
  int horVirBndryPos[]     = { -1, -1, -1 };
  int verVirBndryPos[]     = { -1, -1, -1 };

  for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    const CompID    compID    = CompID(compIdx);
    const CompArea &compArea  = area.block(compID);
    SAOOffset      &ctbOffset = saoblkParam[compIdx];

    if (ctbOffset.modeIdc != SAOMode::OFF)
    {
      ptrdiff_t  srcStride = src.get(compID).stride;
      const Pel *srcBlk    = src.get(compID).bufAt(compArea);
      ptrdiff_t  resStride = res.get(compID).stride;
      Pel       *resBlk    = res.get(compID).bufAt(compArea);
      for (int i = 0; i < numHorVirBndry; i++)
      {
        horVirBndryPosComp[i] = (horVirBndryPos[i] >> ::getComponentScaleY(compID, area.chromaFormat)) - compArea.y;
      }
      for (int i = 0; i < numVerVirBndry; i++)
      {
        verVirBndryPosComp[i] = (verVirBndryPos[i] >> ::getComponentScaleX(compID, area.chromaFormat)) - compArea.x;
      }

      if (isLuma(compID) || isChroma(compID))
      {
        // If it is luma we should not clip, since we will clip
        // after BIF has been added.

        offsetBlockNoClip(cs.sps->m_bitDepths[toChannelType(compID)], cs.slice->clpRng(compID),
                          ctbOffset.typeIdc.newType, ctbOffset.offset, srcBlk, resBlk, srcStride, resStride,
                          compArea.width, compArea.height, isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail,
                          isAboveLeftAvail, isAboveRightAvail, isBelowLeftAvail, isBelowRightAvail, horVirBndryPosComp,
                          verVirBndryPosComp, numHorVirBndry, numVerVirBndry);
      }
      else
      {
        // If it is chroma we should clip as normal, since
        // chroma is not bilaterally filtered.
        offsetBlock(cs.sps->m_bitDepths[toChannelType(compID)], cs.slice->clpRng(compID), ctbOffset.typeIdc.newType,
                    ctbOffset.offset, srcBlk, resBlk, srcStride, resStride, compArea.width, compArea.height,
                    isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail,
                    isBelowLeftAvail, isBelowRightAvail, horVirBndryPosComp, verVirBndryPosComp, numHorVirBndry,
                    numVerVirBndry);
      }
    }
  } // compIdx
}

void SampleAdaptiveOffset::SAOProcess(CodingStructure &cs, SAOBlkParam *saoBlkParams)
{
  CHECK(!saoBlkParams, "No parameters present");

  // In code without BIF, SAOProcess would not be run if 'SAO=0'.
  // However, in the BIF-enabled code, we still might go here if 'SAO=0' and 'BIF=1'.
  // Hence we must check getSAOEnabledFlag() for some of the function calls.
  if (cs.sps->m_saoEnabledFlag)
  {
    xReconstructBlkSAOParams(cs, saoBlkParams);
  }
  const uint32_t numberOfComponents = getNumberValidComponents(cs.area.chromaFormat);

  bool allDisabled = true;
  // If 'SAO=0' we would not normally get here. However, now we might get
  // here if 'SAO=0' and 'BIF=1'. Hence we should only run this if
  // getSAOEnabledFlag() is true. Note that if getSAOEnabledFlag() is false,
  // we will not run the code and bAllDisabled will stay true, which will
  // give the correct behavior.
  if (cs.sps->m_saoEnabledFlag)
  {
    for (uint32_t compIdx = 0; compIdx < numberOfComponents; compIdx++)
    {
      if (m_picSAOEnabled[compIdx])
      {
        allDisabled = false;
      }
    }
  }
  if (allDisabled)
  {
    if (!cs.pps->m_BIF && !cs.pps->m_chromaBIF)
    {
      // However, if we are not doing BIF it is safe to return.
      return;
    }
  }

  const PreCalcValues &pcv = *cs.pcv;
  PelUnitBuf           rec = cs.getRecoBuf();
  m_tempBuf.copyFrom(rec);

  int ctuRsAddr = 0;

  for (uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
  {
    for (uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth, ctuRsAddr++)
    {
      const uint32_t width  = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
      const uint32_t height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
      const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));

      if (cs.pps->m_BIF || cs.pps->m_chromaBIF)
      {
        // We are using BIF, so we run SAO without clipping
        // However, if 'SAO=0', bAllDisabled=true and we should not run offsetCTUnoClip.
        if (!allDisabled)
        {
          offsetCTUnoClip(area, m_tempBuf, rec, cs.picture->getSAO()[ctuRsAddr], cs);
        }

        // We don't need to clip if SAO was not performed on luma.
        SAOBlkParam mySAOblkParam = cs.picture->getSAO()[ctuRsAddr];
        SAOOffset  &myCtbOffset   = mySAOblkParam[0];
        BifParams  &bifParams     = cs.picture->getBifParam(COMP_Y);

        bool clipLumaIfNoBilat = false;
        if (!allDisabled && myCtbOffset.modeIdc != SAOMode::OFF)
        {
          clipLumaIfNoBilat = true;
        }
        SAOOffset &myCtbOffsetCb = mySAOblkParam[1];
        SAOOffset &myCtbOffsetCr = mySAOblkParam[2];

        bool clipChromaIfNoBilat[MAX_NUM_COMP] = { false };

        if (!allDisabled && myCtbOffsetCb.modeIdc != SAOMode::OFF)
        {
          clipChromaIfNoBilat[COMP_Cb] = true;
        }
        if (!allDisabled && myCtbOffsetCr.modeIdc != SAOMode::OFF)
        {
          clipChromaIfNoBilat[COMP_Cr] = true;
        }
        if (cs.pps->m_BIF)
        {

          // And now we traverse the CTU to do BIF
          for (auto &currCU: cs.traverseCUs(CS::getArea(cs, area, ChannelType::LUMA), ChannelType::LUMA))
          {
            for (auto &currTU: CU::traverseTUs(currCU))
            {

              bool applyBIF = bifParams.ctuOn[ctuRsAddr] && m_bilateralFilter.getApplyBIF(currTU, COMP_Y);
              if (applyBIF)
              {
                bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
                int  numHorVirBndry = 0, numVerVirBndry = 0;
                int  horVirBndryPos[]               = { 0, 0, 0 };
                int  verVirBndryPos[]               = { 0, 0, 0 };
                bool isTUCrossedByVirtualBoundaries = m_bilateralFilter.isCrossedByVirtualBoundaries(
                  cs, currTU.lx(), currTU.ly(), currTU.lumaSize().width, currTU.lumaSize().height, clipTop, clipBottom,
                  clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);

                m_bilateralFilter.bilateralFilterDiamond5x5(
                  COMP_Y, m_tempBuf, rec, currTU.cu->qp, cs.slice->clpRng(COMP_Y), currTU, false,
                  isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry, numVerVirBndry,
                  clipTop, clipBottom, clipLeft, clipRight);
                // count_BIF++ ;
              }
              else
              {
                // We don't need to clip if SAO was not performed on luma.
                if (clipLumaIfNoBilat)
                {
                  m_bilateralFilter.clipNotBilaterallyFilteredBlocks(COMP_Y, m_tempBuf, rec, cs.slice->clpRng(COMP_Y),
                                                                     currTU);
                  // count_clip_noBIF++;
                }
                // count_noBIF++;
              }
            }
          }
        } // BIF LUMA is disabled
        else
        {
          for (auto &currCU: cs.traverseCUs(CS::getArea(cs, area, ChannelType::LUMA), ChannelType::LUMA))
          {
            for (auto &currTU: CU::traverseTUs(currCU))
            {
              if (clipLumaIfNoBilat)
              {
                m_bilateralFilter.clipNotBilaterallyFilteredBlocks(COMP_Y, m_tempBuf, rec, cs.slice->clpRng(COMP_Y),
                                                                   currTU);
              }
            }
          }
        }
        if (isChromaEnabled(cs.sps->m_chromaFormatIdc))
        {
          if (cs.pps->m_chromaBIF)
          {
            bool        isDualTree = CS::isDualITree(cs);
            ChannelType chType     = isDualTree ? ChannelType::CHROMA : ChannelType::LUMA;
            // And now we traverse the CTU to do BIF
            for (auto &currCU: cs.traverseCUs(CS::getArea(cs, area, chType), chType))
            {
              bool chromaValid = currCU.Cb().valid() && currCU.Cr().valid();
              if (!chromaValid)
              {
                continue;
              }
              for (auto &currTU: CU::traverseTUs(currCU))
              {
                for (int compIdx = COMP_Cb; compIdx < MAX_NUM_COMP; compIdx++)
                {
                  CompID     compID          = CompID(compIdx);
                  BifParams &chromaBifParams = cs.picture->getBifParam(compID);
                  bool       applyChromaBIF =
                    chromaBifParams.ctuOn[ctuRsAddr] && m_bilateralFilter.getApplyBIF(currTU, compID);
                  if (applyChromaBIF)
                  {
                    bool      clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
                    int       numHorVirBndry = 0, numVerVirBndry = 0;
                    int       horVirBndryPos[] = { 0, 0, 0 };
                    int       verVirBndryPos[] = { 0, 0, 0 };
                    CompArea &myArea           = currTU.block(compID);
                    const int chromaScaleX     = getComponentScaleX(compID, currTU.cu->cs->pcv->chrFormat);
                    const int chromaScaleY     = getComponentScaleY(compID, currTU.cu->cs->pcv->chrFormat);
                    int       yPos             = myArea.y << chromaScaleY;
                    int       xPos             = myArea.x << chromaScaleX;
                    bool      isTUCrossedByVirtualBoundaries = m_bilateralFilter.isCrossedByVirtualBoundaries(
                      cs, xPos, yPos, myArea.width << chromaScaleX, myArea.height << chromaScaleY, clipTop, clipBottom,
                      clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);

                    m_bilateralFilter.bilateralFilterDiamond5x5(
                      compID, m_tempBuf, rec, currTU.cu->qp, cs.slice->clpRng(compID), currTU, false,
                      isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry, numVerVirBndry,
                      clipTop, clipBottom, clipLeft, clipRight);
                  }
                  else
                  {
                    bool useClip = clipChromaIfNoBilat[compID];
                    if (useClip && currTU.blocks[compIdx].valid())
                    {
                      m_bilateralFilter.clipNotBilaterallyFilteredBlocks(compID, m_tempBuf, rec,
                                                                         cs.slice->clpRng(compID), currTU);
                    }
                  }
                }
              }
            }
          }   // BIF chroma is disabled
          else
          {
            bool        isDualTree = CS::isDualITree(cs);
            ChannelType chType     = isDualTree ? ChannelType::CHROMA : ChannelType::LUMA;

            for (auto &currCU: cs.traverseCUs(CS::getArea(cs, area, chType), chType))
            {
              bool chromaValid = currCU.Cb().valid() && currCU.Cr().valid();
              if (!chromaValid)
              {
                continue;
              }
              for (auto &currTU: CU::traverseTUs(currCU))
              {
                if (clipChromaIfNoBilat[COMP_Cb] && currTU.blocks[COMP_Cb].valid())
                {
                  m_bilateralFilter.clipNotBilaterallyFilteredBlocks(COMP_Cb, m_tempBuf, rec, cs.slice->clpRng(COMP_Cb),
                                                                     currTU);
                }
                if (clipChromaIfNoBilat[COMP_Cr] && currTU.blocks[COMP_Cr].valid())
                {
                  m_bilateralFilter.clipNotBilaterallyFilteredBlocks(COMP_Cr, m_tempBuf, rec, cs.slice->clpRng(COMP_Cr),
                                                                     currTU);
                }
              }
            }
          }
        }
      }
      else
      {
        // BIF is not used, use old SAO code
        offsetCTU(area, m_tempBuf, rec, cs.picture->getSAO()[ctuRsAddr], cs);
      }
    }
  }
  DTRACE_UPDATE(g_trace_ctx, (std::make_pair("poc", cs.slice->m_poc)));
  DTRACE_PIC_COMP(D_REC_CB_LUMA_SAO, cs, cs.getRecoBuf(), COMP_Y);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_SAO, cs, cs.getRecoBuf(), COMP_Cb);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_SAO, cs, cs.getRecoBuf(), COMP_Cr);

  DTRACE(g_trace_ctx, D_CRC, "SAO");
  DTRACE_CRC(g_trace_ctx, D_CRC, cs, cs.getRecoBuf());
}

void SampleAdaptiveOffset::CCSAOProcess(CodingStructure &cs)
{
  const uint32_t numberOfComponents = getNumberValidComponents(cs.area.chromaFormat);
  bool           bAllDisabled       = true;
  for (uint32_t compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    if (m_ccSaoComParam.enabled[compIdx])
    {
      bAllDisabled = false;
    }
  }
  if (bAllDisabled)
  {
    return;
  }

  const PreCalcValues &pcv    = *cs.pcv;
  PelUnitBuf           dstYuv = cs.getRecoBuf();
  PelUnitBuf           srcYuv = m_ccSaoBuf.getBuf(cs.area);
  srcYuv.extendBorderPel(MAX_CCSAO_FILTER_LENGTH >> 1);

  applyCcSao(cs, pcv, srcYuv, dstYuv);

  DTRACE(g_trace_ctx, D_CRC, "CCSAO");
  DTRACE_CRC(g_trace_ctx, D_CRC, cs, cs.getRecoBuf());
}

void SampleAdaptiveOffset::applyCcSao(CodingStructure &cs, const PreCalcValues &pcv, const CPelUnitBuf &srcYuv,
                                      PelUnitBuf &dstYuv)
{
  int ctuRsAddr = 0;
  for (uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
  {
    for (uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
    {
      const uint32_t width  = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
      const uint32_t height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
      const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));
      offsetCTUCcSaoNoClip(cs, area, srcYuv, dstYuv, ctuRsAddr);
      ctuRsAddr++;
    }
  }
}

void SampleAdaptiveOffset::jointClipSaoBifCcSao(CodingStructure &cs)
{
  if (!cs.sps->m_saoEnabledFlag && !cs.pps->m_BIF && !cs.pps->m_chromaBIF && !cs.sps->m_ccSaoEnabledFlag)
  {
    return;
  }

  const PreCalcValues &pcv    = *cs.pcv;
  PelUnitBuf           dstYuv = cs.getRecoBuf();

  // Iterate all CTUs and check if any of the filters is on for a given component
  int ctuRsAddr = 0;
  for (uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
  {
    for (uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
    {
      const uint32_t width  = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
      const uint32_t height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
      const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));
      const uint32_t numberOfComponents = getNumberValidComponents(area.chromaFormat);

      for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
      {
        CompID compID  = CompID(compIdx);
        bool   saoOn   = false;
        bool   ccsaoOn = false;
        if (cs.sps->m_saoEnabledFlag)
        {
          SAOBlkParam mySAOblkParam = cs.picture->getSAO()[ctuRsAddr];
          SAOOffset  &myCtbOffset   = mySAOblkParam[compIdx];
          saoOn                     = myCtbOffset.modeIdc != SAOMode::OFF;
        }
        if (cs.sps->m_ccSaoEnabledFlag)
        {
          const int setIdc = m_ccSaoControl[compIdx][ctuRsAddr];
          ccsaoOn          = m_ccSaoComParam.enabled[compIdx] && setIdc != 0;
        }
        if (ccsaoOn || saoOn)
        {
          // We definitely need to clip if either SAO or CCSAO is on for the given component of the CTU
          clipCTU(cs, dstYuv, area, compID);
        }
        else
        {
          // When BIF is on, the luma component might need to be clipped
          if (cs.pps->m_BIF && isLuma(compID))
          {
            BifParams &bifParams = cs.picture->getBifParam(compID);

            // And now we traverse the CTU to do clipping
            for (auto &currCU: cs.traverseCUs(CS::getArea(cs, area, ChannelType::LUMA), ChannelType::LUMA))
            {
              for (auto &currTU: CU::traverseTUs(currCU))
              {
                bool applyBIF = bifParams.ctuOn[ctuRsAddr] && m_bilateralFilter.getApplyBIF(currTU, compID);
                if (applyBIF)
                {
                  m_bilateralFilter.clipNotBilaterallyFilteredBlocks(compID, m_tempBuf, dstYuv,
                                                                     cs.slice->clpRng(compID), currTU);
                }
              }
            }
          }
          if (cs.pps->m_chromaBIF && isChroma(compID))
          {
            bool        isDualTree = CS::isDualITree(cs);
            ChannelType chType     = isDualTree ? ChannelType::CHROMA : ChannelType::LUMA;
            for (auto &currCU: cs.traverseCUs(CS::getArea(cs, area, chType), chType))
            {
              bool chromaValid = currCU.Cb().valid() && currCU.Cr().valid();
              if (!chromaValid)
              {
                continue;
              }
              for (auto &currTU: CU::traverseTUs(currCU))
              {
                // Cb or Cr
                BifParams &chromaBifParams = cs.picture->getBifParam(compID);
                bool applyChromaBIF = chromaBifParams.ctuOn[ctuRsAddr] && m_bilateralFilter.getApplyBIF(currTU, compID);
                if (applyChromaBIF)
                {
                  m_bilateralFilter.clipNotBilaterallyFilteredBlocks(compID, m_tempBuf, dstYuv,
                                                                     cs.slice->clpRng(compID), currTU);
                }
              }
            }
          }
        }
      }
      ctuRsAddr++;
    }
  }
  DTRACE(g_trace_ctx, D_CRC, "JCLP");
  DTRACE_CRC(g_trace_ctx, D_CRC, cs, cs.getRecoBuf());
}

void SampleAdaptiveOffset::clipCTU(CodingStructure &cs, PelUnitBuf &dstYuv, const UnitArea &area, const CompID compID)
{
  const CompArea &compArea  = area.block(compID);
  const uint32_t  height    = compArea.height;
  const uint32_t  width     = compArea.width;
  Pel            *dst       = dstYuv.get(compID).bufAt(area.block(compID));
  const ptrdiff_t dstStride = dstYuv.get(compID).stride;

  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      // new result = old result (which is SAO-treated already) + clipping
      dst[x] = ClipPel<int>(dst[x], cs.slice->clpRng(compID));
    }
    dst += dstStride;
  }
}

void SampleAdaptiveOffset::offsetCTUCcSaoNoClip(CodingStructure &cs, const UnitArea &area, const CPelUnitBuf &srcYuv,
                                                PelUnitBuf &dstYuv, const int ctuRsAddr)
{
  const uint32_t numberOfComponents = getNumberValidComponents(area.chromaFormat);
  bool           bAllOff            = true;
  for (uint32_t compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    if (m_ccSaoComParam.enabled[compIdx])
    {
      bAllOff = false;
    }
  }
  if (bAllOff)
  {
    return;
  }

  bool isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail, isBelowLeftAvail,
    isBelowRightAvail;
  deriveLoopFilterBoundaryAvailability(cs, area.Y(), isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail,
                                       isAboveLeftAvail, isAboveRightAvail, isBelowLeftAvail, isBelowRightAvail);

  for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    if (m_ccSaoComParam.enabled[compIdx])
    {
      const int setIdc = m_ccSaoControl[compIdx][ctuRsAddr];

      if (setIdc == 0)
      {
        continue;
      }

      const int setType = m_ccSaoComParam.setType[compIdx][setIdc - 1];
      if (setType == CCSAO_SET_TYPE_EDGE)
      {
        const CompID    compID     = CompID(compIdx);
        const CompArea &compArea   = area.block(compID);
        const ptrdiff_t srcStrideY = srcYuv.get(COMP_Y).stride;
        const ptrdiff_t srcStrideU = srcYuv.get(COMP_Cb).stride;
        const ptrdiff_t srcStrideV = srcYuv.get(COMP_Cr).stride;
        const Pel      *srcBlkY    = srcYuv.get(COMP_Y).bufAt(area.block(COMP_Y));
        const Pel      *srcBlkU    = srcYuv.get(COMP_Cb).bufAt(area.block(COMP_Cb));
        const Pel      *srcBlkV    = srcYuv.get(COMP_Cr).bufAt(area.block(COMP_Cr));
        const ptrdiff_t dstStride  = dstYuv.get(compID).stride;
        Pel            *dstBlk     = dstYuv.get(compID).bufAt(compArea);

        const uint16_t edgeCmp = m_ccSaoComParam.candPos[compIdx][setIdc - 1][COMP_Cb];
        const uint16_t edgeIdc = m_ccSaoComParam.bandNum[compIdx][setIdc - 1][COMP_Cr];
        uint16_t       edgeDir = m_ccSaoComParam.candPos[compIdx][setIdc - 1][COMP_Y];
        const uint16_t edgeThr = m_ccSaoComParam.bandNum[compIdx][setIdc - 1][COMP_Cb];
        const uint16_t bandIdc = m_ccSaoComParam.bandNum[compIdx][setIdc - 1][COMP_Y];
        const short   *offset  = m_ccSaoComParam.offset[compIdx][setIdc - 1];

        offsetBlockCcSaoNoClipEdge(
          compID, cs.sps->m_chromaFormatIdc, cs.sps->m_bitDepths[toChannelType(compID)], cs.slice->clpRng(compID),
          edgeCmp, edgeDir, bandIdc, edgeThr, edgeIdc, offset, srcBlkY, srcBlkU, srcBlkV, dstBlk, srcStrideY,
          srcStrideU, srcStrideV, dstStride, compArea.width, compArea.height, isLeftAvail, isRightAvail, isAboveAvail,
          isBelowAvail, isAboveLeftAvail, isAboveRightAvail, isBelowLeftAvail, isBelowRightAvail);
      }
      else
      {
        const CompID    compID     = CompID(compIdx);
        const CompArea &compArea   = area.block(compID);
        const ptrdiff_t srcStrideY = srcYuv.get(COMP_Y).stride;
        const ptrdiff_t srcStrideU = srcYuv.get(COMP_Cb).stride;
        const ptrdiff_t srcStrideV = srcYuv.get(COMP_Cr).stride;
        const Pel      *srcBlkY    = srcYuv.get(COMP_Y).bufAt(area.block(COMP_Y));
        const Pel      *srcBlkU    = srcYuv.get(COMP_Cb).bufAt(area.block(COMP_Cb));
        const Pel      *srcBlkV    = srcYuv.get(COMP_Cr).bufAt(area.block(COMP_Cr));
        const ptrdiff_t dstStride  = dstYuv.get(compID).stride;
        Pel            *dstBlk     = dstYuv.get(compID).bufAt(compArea);

        const uint16_t candPosY = m_ccSaoComParam.candPos[compIdx][setIdc - 1][COMP_Y];
        const uint16_t bandNumY = m_ccSaoComParam.bandNum[compIdx][setIdc - 1][COMP_Y];
        const uint16_t bandNumU = m_ccSaoComParam.bandNum[compIdx][setIdc - 1][COMP_Cb];
        const uint16_t bandNumV = m_ccSaoComParam.bandNum[compIdx][setIdc - 1][COMP_Cr];
        const short   *offset   = m_ccSaoComParam.offset[compIdx][setIdc - 1];

        offsetBlockCcSaoNoClip(compID, cs.sps->m_chromaFormatIdc, cs.sps->m_bitDepths[toChannelType(compID)],
                               cs.slice->clpRng(compID), candPosY, bandNumY, bandNumU, bandNumV, offset, srcBlkY,
                               srcBlkU, srcBlkV, dstBlk, srcStrideY, srcStrideU, srcStrideV, dstStride, compArea.width,
                               compArea.height, isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail,
                               isAboveRightAvail, isBelowLeftAvail, isBelowRightAvail);
      }
    }
  }
}

int SampleAdaptiveOffset::getCcSaoClassNum(const int compIdx, const int setIdx, const CcSaoComParam &ccSaoParam)
{
  int classNum = 0;

  if (ccSaoParam.setType[compIdx][setIdx] == CCSAO_SET_TYPE_EDGE)
  {
    int bandIdc = ccSaoParam.bandNum[compIdx][setIdx][COMP_Y], bandNum = g_ccSaoBandTab[bandIdc][1];
    int edgeIdc = ccSaoParam.bandNum[compIdx][setIdx][COMP_Cr], edgeNum = g_ccSaoEdgeNum[edgeIdc][0];
    classNum = bandNum * edgeNum;
  }
  else
  {
    classNum = ccSaoParam.bandNum[compIdx][setIdx][COMP_Y] * ccSaoParam.bandNum[compIdx][setIdx][COMP_Cb] *
      ccSaoParam.bandNum[compIdx][setIdx][COMP_Cr];
  }

  return classNum;
}

inline void SampleAdaptiveOffset::offsetmSampleCcSaoNoClipEdge(
  int startx, int endx, const Pel *srcY, const Pel *srcU, const Pel *srcV, const int chromaScaleX,
  const uint16_t edgeCmp, const ptrdiff_t srcStrideE, const int edgePosYA, const int edgePosXA, const int edgePosYB,
  const int edgePosXB, const int edgeThrVal, const uint16_t edgeIdc, const int edgeNumUni, const int bandCmp,
  const int bandNum, const int edgeNum, const int bitDepth, const short *offset, Pel *dst)
{
  for (int x = startx; x < endx; x++)
  {
    const Pel *colY              = srcY + x;
    const Pel *colU              = srcU + (x >> chromaScaleX);
    const Pel *colV              = srcV + (x >> chromaScaleX);
    const Pel *col[MAX_NUM_COMP] = { colY, colU, colV };
    const Pel *colE              = col[edgeCmp];
    const Pel *colA              = colE + srcStrideE * edgePosYA + edgePosXA;
    const Pel *colB              = colE + srcStrideE * edgePosYB + edgePosXB;

    const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
    const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
    const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
    const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;

    const int classIdx = bandIdx * edgeNum + edgeIdx;
    dst[x]             = dst[x] + offset[classIdx];
  }
}

inline void SampleAdaptiveOffset::offsetmChromaSampleCcSaoNoClipEdge(
  int startx, int endx, const Pel *srcY, const Pel *srcU, const Pel *srcV, const int chromaScaleX,
  const uint16_t edgeCmp, const ptrdiff_t srcStrideE, const int edgePosYA, const int edgePosXA, const int edgePosYB,
  const int edgePosXB, const int edgeThrVal, const uint16_t edgeIdc, const int edgeNumUni, const int bandCmp,
  const int bandNum, const int edgeNum, const int bitDepth, const short *offset, Pel *dst)
{
  for (int x = startx; x < endx; x++)
  {
    const Pel *colY              = srcY + (x << chromaScaleX);
    const Pel *colU              = srcU + x;
    const Pel *colV              = srcV + x;
    const Pel *col[MAX_NUM_COMP] = { colY, colU, colV };
    const Pel *colE              = col[edgeCmp];
    const Pel *colA              = colE + srcStrideE * edgePosYA + edgePosXA;
    const Pel *colB              = colE + srcStrideE * edgePosYB + edgePosXB;

    const int edgeIdxA = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
    const int edgeIdxB = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
    const int edgeIdx  = edgeIdxA * edgeNumUni + edgeIdxB;
    const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;

    const int classIdx = bandIdx * edgeNum + edgeIdx;
    dst[x]             = dst[x] + offset[classIdx];
  }
}

void SampleAdaptiveOffset::offsetBlockCcSaoNoClipEdge(
  const CompID compID, const ChromaFormat chromaFormat, const int bitDepth, const ClpRng &clpRng,
  const uint16_t edgeCmp, uint16_t edgeDir, const uint16_t bandIdc, const uint16_t edgeThr, const uint16_t edgeIdc,
  const short *offset, const Pel *srcY, const Pel *srcU, const Pel *srcV, Pel *dst, const ptrdiff_t srcStrideY,
  const ptrdiff_t srcStrideU, const ptrdiff_t srcStrideV, const ptrdiff_t dstStride, const int width, const int height,
  bool isLeftAvail, bool isRightAvail, bool isAboveAvail, bool isBelowAvail, bool isAboveLeftAvail,
  bool isAboveRightAvail, bool isBelowLeftAvail, bool isBelowRightAvail)
{
  const int       edgePosXA = g_ccSaoEdgePosX[edgeDir][0], edgePosYA = g_ccSaoEdgePosY[edgeDir][0];
  const int       edgePosXB = g_ccSaoEdgePosX[edgeDir][1], edgePosYB = g_ccSaoEdgePosY[edgeDir][1];
  const int       bandCmp        = g_ccSaoBandTab[bandIdc][0];
  const int       bandNum        = g_ccSaoBandTab[bandIdc][1];
  const int       edgeThrVal     = g_ccSaoEdgeThr[edgeIdc][edgeThr];
  const int       edgeNum        = g_ccSaoEdgeNum[edgeIdc][0];
  const int       edgeNumUni     = g_ccSaoEdgeNum[edgeIdc][1];
  const ptrdiff_t srcStrideE     = edgeCmp == COMP_Y ? srcStrideY : edgeCmp == COMP_Cb ? srcStrideU : srcStrideV;
  const int       chromaScaleX   = getChannelTypeScaleX(ChannelType::CHROMA, chromaFormat);
  const int       chromaScaleY   = getChannelTypeScaleY(ChannelType::CHROMA, chromaFormat);
  const int       chromaScaleYM1 = 1 - chromaScaleY;

  int y, startX, startY, endX, endY;
  int firstLineStartX, firstLineEndX, lastLineStartX, lastLineEndX;
  switch (compID)
  {
  case COMP_Y:
    {
      switch ((SAOModeNewTypes)edgeDir)
      {
      case SAOModeNewTypes::NONE:
      case SAOModeNewTypes::BO:
      case SAOModeNewTypes::NUM:
      case SAOModeNewTypes::EO_0:
        {
          startX = isLeftAvail ? 0 : 1;
          endX   = isRightAvail ? width : (width - 1);
          for (y = 0; y < height; y++)
          {
            offsetmSampleCcSaoNoClipEdge(startX, endX, srcY, srcU, srcV, chromaScaleX, edgeCmp, srcStrideE, edgePosYA,
                                         edgePosXA, edgePosYB, edgePosXB, edgeThrVal, edgeIdc, edgeNumUni, bandCmp,
                                         bandNum, edgeNum, bitDepth, offset, dst);
            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            dst += dstStride;
          }
        }
        break;
      case SAOModeNewTypes::EO_90:
        {
          startY = isAboveAvail ? 0 : 1;
          endY   = isBelowAvail ? height : (height - 1);
          if (!isAboveAvail)
          {
            srcY += srcStrideY;
            srcU += srcStrideU * chromaScaleYM1;
            srcV += srcStrideV * chromaScaleYM1;
            dst += dstStride;
          }
          for (y = startY; y < endY; y++)
          {
            offsetmSampleCcSaoNoClipEdge(0, width, srcY, srcU, srcV, chromaScaleX, edgeCmp, srcStrideE, edgePosYA,
                                         edgePosXA, edgePosYB, edgePosXB, edgeThrVal, edgeIdc, edgeNumUni, bandCmp,
                                         bandNum, edgeNum, bitDepth, offset, dst);

            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            dst += dstStride;
          }
        }
        break;
      case SAOModeNewTypes::EO_135:
        {
          startX = isLeftAvail ? 0 : 1;
          endX   = isRightAvail ? width : (width - 1);

          // 1st line
          firstLineStartX = isAboveLeftAvail ? 0 : 1;
          firstLineEndX   = isAboveAvail ? endX : 1;
          offsetmSampleCcSaoNoClipEdge(firstLineStartX, firstLineEndX, srcY, srcU, srcV, chromaScaleX, edgeCmp,
                                       srcStrideE, edgePosYA, edgePosXA, edgePosYB, edgePosXB, edgeThrVal, edgeIdc,
                                       edgeNumUni, bandCmp, bandNum, edgeNum, bitDepth, offset, dst);

          srcY += srcStrideY;
          srcU += srcStrideU * chromaScaleYM1;
          srcV += srcStrideV * chromaScaleYM1;
          dst += dstStride;

          // middle lines
          for (y = 1; y < height - 1; y++)
          {
            offsetmSampleCcSaoNoClipEdge(startX, endX, srcY, srcU, srcV, chromaScaleX, edgeCmp, srcStrideE, edgePosYA,
                                         edgePosXA, edgePosYB, edgePosXB, edgeThrVal, edgeIdc, edgeNumUni, bandCmp,
                                         bandNum, edgeNum, bitDepth, offset, dst);

            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            dst += dstStride;
          }

          // last line
          lastLineStartX = isBelowAvail ? startX : (width - 1);
          lastLineEndX   = isBelowRightAvail ? width : (width - 1);
          offsetmSampleCcSaoNoClipEdge(lastLineStartX, lastLineEndX, srcY, srcU, srcV, chromaScaleX, edgeCmp,
                                       srcStrideE, edgePosYA, edgePosXA, edgePosYB, edgePosXB, edgeThrVal, edgeIdc,
                                       edgeNumUni, bandCmp, bandNum, edgeNum, bitDepth, offset, dst);
        }
        break;
      case SAOModeNewTypes::EO_45:
        {
          startX = isLeftAvail ? 0 : 1;
          endX   = isRightAvail ? width : (width - 1);

          // first line
          firstLineStartX = isAboveAvail ? startX : (width - 1);
          firstLineEndX   = isAboveRightAvail ? width : (width - 1);
          offsetmSampleCcSaoNoClipEdge(firstLineStartX, firstLineEndX, srcY, srcU, srcV, chromaScaleX, edgeCmp,
                                       srcStrideE, edgePosYA, edgePosXA, edgePosYB, edgePosXB, edgeThrVal, edgeIdc,
                                       edgeNumUni, bandCmp, bandNum, edgeNum, bitDepth, offset, dst);

          srcY += srcStrideY;
          srcU += srcStrideU * chromaScaleYM1;
          srcV += srcStrideV * chromaScaleYM1;
          dst += dstStride;

          // middle lines
          for (y = 1; y < height - 1; y++)
          {
            offsetmSampleCcSaoNoClipEdge(startX, endX, srcY, srcU, srcV, chromaScaleX, edgeCmp, srcStrideE, edgePosYA,
                                         edgePosXA, edgePosYB, edgePosXB, edgeThrVal, edgeIdc, edgeNumUni, bandCmp,
                                         bandNum, edgeNum, bitDepth, offset, dst);

            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            dst += dstStride;
          }

          // last line
          lastLineStartX = isBelowLeftAvail ? 0 : 1;
          lastLineEndX   = isBelowAvail ? endX : 1;
          offsetmSampleCcSaoNoClipEdge(lastLineStartX, lastLineEndX, srcY, srcU, srcV, chromaScaleX, edgeCmp,
                                       srcStrideE, edgePosYA, edgePosXA, edgePosYB, edgePosXB, edgeThrVal, edgeIdc,
                                       edgeNumUni, bandCmp, bandNum, edgeNum, bitDepth, offset, dst);
        }
        break;
      }
      break;
    }

  case COMP_Cb:
  case COMP_Cr:
    {
      switch ((SAOModeNewTypes)edgeDir)
      {
      case SAOModeNewTypes::NONE:
      case SAOModeNewTypes::BO:
      case SAOModeNewTypes::NUM:
      case SAOModeNewTypes::EO_0:
        {
          startX = isLeftAvail ? 0 : 1;
          endX   = isRightAvail ? width : (width - 1);
          for (y = 0; y < height; y++)
          {
            offsetmChromaSampleCcSaoNoClipEdge(startX, endX, srcY, srcU, srcV, chromaScaleX, edgeCmp, srcStrideE,
                                               edgePosYA, edgePosXA, edgePosYB, edgePosXB, edgeThrVal, edgeIdc,
                                               edgeNumUni, bandCmp, bandNum, edgeNum, bitDepth, offset, dst);

            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            dst += dstStride;
          }
        }
        break;
      case SAOModeNewTypes::EO_90:
        {
          startY = isAboveAvail ? 0 : 1;
          endY   = isBelowAvail ? height : (height - 1);
          if (!isAboveAvail)
          {
            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            dst += dstStride;
          }
          for (y = startY; y < endY; y++)
          {
            offsetmChromaSampleCcSaoNoClipEdge(0, width, srcY, srcU, srcV, chromaScaleX, edgeCmp, srcStrideE, edgePosYA,
                                               edgePosXA, edgePosYB, edgePosXB, edgeThrVal, edgeIdc, edgeNumUni,
                                               bandCmp, bandNum, edgeNum, bitDepth, offset, dst);

            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            dst += dstStride;
          }
        }
        break;
      case SAOModeNewTypes::EO_135:
        {
          startX = isLeftAvail ? 0 : 1;
          endX   = isRightAvail ? width : (width - 1);

          // 1st line
          firstLineStartX = isAboveLeftAvail ? 0 : 1;
          firstLineEndX   = isAboveAvail ? endX : 1;
          offsetmChromaSampleCcSaoNoClipEdge(firstLineStartX, firstLineEndX, srcY, srcU, srcV, chromaScaleX, edgeCmp,
                                             srcStrideE, edgePosYA, edgePosXA, edgePosYB, edgePosXB, edgeThrVal,
                                             edgeIdc, edgeNumUni, bandCmp, bandNum, edgeNum, bitDepth, offset, dst);

          srcY += srcStrideY << chromaScaleY;
          srcU += srcStrideU;
          srcV += srcStrideV;
          dst += dstStride;

          // middle lines
          for (y = 1; y < height - 1; y++)
          {
            offsetmChromaSampleCcSaoNoClipEdge(startX, endX, srcY, srcU, srcV, chromaScaleX, edgeCmp, srcStrideE,
                                               edgePosYA, edgePosXA, edgePosYB, edgePosXB, edgeThrVal, edgeIdc,
                                               edgeNumUni, bandCmp, bandNum, edgeNum, bitDepth, offset, dst);

            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            dst += dstStride;
          }

          // last line
          lastLineStartX = isBelowAvail ? startX : (width - 1);
          lastLineEndX   = isBelowRightAvail ? width : (width - 1);
          offsetmChromaSampleCcSaoNoClipEdge(lastLineStartX, lastLineEndX, srcY, srcU, srcV, chromaScaleX, edgeCmp,
                                             srcStrideE, edgePosYA, edgePosXA, edgePosYB, edgePosXB, edgeThrVal,
                                             edgeIdc, edgeNumUni, bandCmp, bandNum, edgeNum, bitDepth, offset, dst);
        }
        break;
      case SAOModeNewTypes::EO_45:
        {
          startX = isLeftAvail ? 0 : 1;
          endX   = isRightAvail ? width : (width - 1);

          // first line
          firstLineStartX = isAboveAvail ? startX : (width - 1);
          firstLineEndX   = isAboveRightAvail ? width : (width - 1);
          offsetmChromaSampleCcSaoNoClipEdge(firstLineStartX, firstLineEndX, srcY, srcU, srcV, chromaScaleX, edgeCmp,
                                             srcStrideE, edgePosYA, edgePosXA, edgePosYB, edgePosXB, edgeThrVal,
                                             edgeIdc, edgeNumUni, bandCmp, bandNum, edgeNum, bitDepth, offset, dst);

          srcY += srcStrideY << chromaScaleY;
          srcU += srcStrideU;
          srcV += srcStrideV;
          dst += dstStride;

          // middle lines
          for (y = 1; y < height - 1; y++)
          {
            offsetmChromaSampleCcSaoNoClipEdge(startX, endX, srcY, srcU, srcV, chromaScaleX, edgeCmp, srcStrideE,
                                               edgePosYA, edgePosXA, edgePosYB, edgePosXB, edgeThrVal, edgeIdc,
                                               edgeNumUni, bandCmp, bandNum, edgeNum, bitDepth, offset, dst);

            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            dst += dstStride;
          }

          // last line
          lastLineStartX = isBelowLeftAvail ? 0 : 1;
          lastLineEndX   = isBelowAvail ? endX : 1;
          offsetmChromaSampleCcSaoNoClipEdge(lastLineStartX, lastLineEndX, srcY, srcU, srcV, chromaScaleX, edgeCmp,
                                             srcStrideE, edgePosYA, edgePosXA, edgePosYB, edgePosXB, edgeThrVal,
                                             edgeIdc, edgeNumUni, bandCmp, bandNum, edgeNum, bitDepth, offset, dst);
        }
        break;
      }
      break;
    }
  default:
    {
      THROW("Not a supported CCSAO compID\n");
    }
  }
}

inline void SampleAdaptiveOffset::offsetmSampleCcSaoNoClip(int startx, int endx, const Pel *srcY, const Pel *srcU,
                                                           const Pel *srcV, const int chromaScaleX,
                                                           const ptrdiff_t srcStrideY, const int candPosYX,
                                                           const int candPosYY, const uint16_t bandNumY,
                                                           const uint16_t bandNumU, const uint16_t bandNumV,
                                                           const int bitDepth, const short *offset, Pel *dst)
{
  for (int x = startx; x < endx; x++)
  {
    const Pel *colY = srcY + x + srcStrideY * candPosYY + candPosYX;
    const Pel *colU = srcU + (x >> chromaScaleX);
    const Pel *colV = srcV + (x >> chromaScaleX);

    const int bandY    = (*colY * bandNumY) >> bitDepth;
    const int bandU    = (*colU * bandNumU) >> bitDepth;
    const int bandV    = (*colV * bandNumV) >> bitDepth;
    const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
    const int classIdx = bandIdx;

    // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
    dst[x] = dst[x] + offset[classIdx];
  }
}

inline void SampleAdaptiveOffset::offsetmChromaSampleCcSaoNoClip(int startx, int endx, const Pel *srcY, const Pel *srcU,
                                                                 const Pel *srcV, const int chromaScaleX,
                                                                 const ptrdiff_t srcStrideY, const int candPosYX,
                                                                 const int candPosYY, const uint16_t bandNumY,
                                                                 const uint16_t bandNumU, const uint16_t bandNumV,
                                                                 const int bitDepth, const short *offset, Pel *dst)
{
  for (int x = startx; x < endx; x++)
  {
    const Pel *colY = srcY + (x << chromaScaleX) + srcStrideY * candPosYY + candPosYX;
    const Pel *colU = srcU + x;
    const Pel *colV = srcV + x;

    const int bandY    = (*colY * bandNumY) >> bitDepth;
    const int bandU    = (*colU * bandNumU) >> bitDepth;
    const int bandV    = (*colV * bandNumV) >> bitDepth;
    const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
    const int classIdx = bandIdx;

    // dst[x] = ClipPel<int>(dst[x] + offset[classIdx], clpRng);
    dst[x] = dst[x] + offset[classIdx];
  }
}

void SampleAdaptiveOffset::offsetBlockCcSaoNoClip(
  const CompID compID, const ChromaFormat chromaFormat, const int bitDepth, const ClpRng &clpRng,
  const uint16_t candPosY, const uint16_t bandNumY, const uint16_t bandNumU, const uint16_t bandNumV,
  const short *offset, const Pel *srcY, const Pel *srcU, const Pel *srcV, Pel *dst, const ptrdiff_t srcStrideY,
  const ptrdiff_t srcStrideU, const ptrdiff_t srcStrideV, const ptrdiff_t dstStride, const int width, const int height,
  bool isLeftAvail, bool isRightAvail, bool isAboveAvail, bool isBelowAvail, bool isAboveLeftAvail,
  bool isAboveRightAvail, bool isBelowLeftAvail, bool isBelowRightAvail)
{
  const int candPosYX = g_ccSaoCandPosX[COMP_Y][candPosY];
  const int candPosYY = g_ccSaoCandPosY[COMP_Y][candPosY];

  const int chromaScaleX   = getChannelTypeScaleX(ChannelType::CHROMA, chromaFormat);
  const int chromaScaleY   = getChannelTypeScaleY(ChannelType::CHROMA, chromaFormat);
  const int chromaScaleYM1 = 1 - chromaScaleY;

  int y, startX, startY, endX, endY;
  int firstLineStartX, firstLineEndX, lastLineStartX, lastLineEndX;

  switch (compID)
  {
  case COMP_Y:
    {
      switch (candPosY)
      {
      case 0:   // top left (-1, -1), unlike SAO, CCSAO BO only uses one spatial neighbor sample to derive band
                // information
        /* total 9 cases will come up here
        for (-1,-1) just use the top and middle lines and check for vb */
        {
          startX          = isLeftAvail ? 0 : 1;
          endX            = width;
          // 1st line
          firstLineStartX = isAboveLeftAvail ? 0 : 1;
          firstLineEndX   = isAboveAvail ? endX : 1;
          offsetmSampleCcSaoNoClip(firstLineStartX, firstLineEndX, srcY, srcU, srcV, chromaScaleX, srcStrideY,
                                   candPosYX, candPosYY, bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

          srcY += srcStrideY;
          srcU += srcStrideU * chromaScaleYM1;
          srcV += srcStrideV * chromaScaleYM1;
          dst += dstStride;

          // middle lines
          for (y = 1; y < height; y++)
          {
            offsetmSampleCcSaoNoClip(startX, endX, srcY, srcU, srcV, chromaScaleX, srcStrideY, candPosYX, candPosYY,
                                     bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            dst += dstStride;
          }
        }
        break;
      case 1: /*(0, -1)  top sample */
        {
          startY = isAboveAvail ? 0 : 1;
          endY   = height;
          if (!isAboveAvail)
          {
            srcY += srcStrideY;
            srcU += srcStrideU * chromaScaleYM1;
            srcV += srcStrideV * chromaScaleYM1;
            dst += dstStride;
          }
          for (y = startY; y < endY; y++)
          {
            offsetmSampleCcSaoNoClip(0, width, srcY, srcU, srcV, chromaScaleX, srcStrideY, candPosYX, candPosYY,
                                     bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            dst += dstStride;
          }
          break;
        }
      case 2: /*(0, -1)  top right sample */
        {
          startX          = isLeftAvail ? 0 : 1;
          endX            = width;
          // first line
          firstLineStartX = isAboveAvail ? startX : (width - 1);
          firstLineEndX   = isAboveRightAvail ? width : (width - 1);
          offsetmSampleCcSaoNoClip(firstLineStartX, firstLineEndX, srcY, srcU, srcV, chromaScaleX, srcStrideY,
                                   candPosYX, candPosYY, bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

          srcY += srcStrideY;
          srcU += srcStrideU * chromaScaleYM1;
          srcV += srcStrideV * chromaScaleYM1;
          dst += dstStride;

          // middle lines
          for (y = 1; y < height; y++)
          {
            offsetmSampleCcSaoNoClip(startX, endX, srcY, srcU, srcV, chromaScaleX, srcStrideY, candPosYX, candPosYY,
                                     bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            dst += dstStride;
          }
          break;
        }
      case 3: /*(-1, 0)  left sample */
        {
          startX = isLeftAvail ? 0 : 1;
          endX   = width;

          for (y = 0; y < height; y++)
          {
            offsetmSampleCcSaoNoClip(startX, endX, srcY, srcU, srcV, chromaScaleX, srcStrideY, candPosYX, candPosYY,
                                     bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            dst += dstStride;
          }
          break;
        }
      case 4: /*(0, 0)  current sample */
        {     /* when current sample is choosen there is no more dependency on neighbor samples*/

          for (y = 0; y < height; y++)
          {
            offsetmSampleCcSaoNoClip(0, width, srcY, srcU, srcV, chromaScaleX, srcStrideY, candPosYX, candPosYY,
                                     bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            dst += dstStride;
          }
          break;
        }
      case 5: /*(1, 0)  right sample */
        {
          startX = 0;
          endX   = isRightAvail ? width : (width - 1);

          for (y = 0; y < height; y++)
          {
            offsetmSampleCcSaoNoClip(startX, endX, srcY, srcU, srcV, chromaScaleX, srcStrideY, candPosYX, candPosYY,
                                     bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            dst += dstStride;
          }
          break;
        }
      case 6: /*(-1, 1)  below left sample */
        {
          startX = isLeftAvail ? 0 : 1;
          endX   = width;

          for (y = 1; y < height; y++)
          {
            offsetmSampleCcSaoNoClip(startX, endX, srcY, srcU, srcV, chromaScaleX, srcStrideY, candPosYX, candPosYY,
                                     bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            dst += dstStride;
          }

          // last line
          lastLineStartX = isBelowLeftAvail ? 0 : 1;
          lastLineEndX   = isBelowAvail ? endX : 1;
          offsetmSampleCcSaoNoClip(lastLineStartX, lastLineEndX, srcY, srcU, srcV, chromaScaleX, srcStrideY, candPosYX,
                                   candPosYY, bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

          break;
        }
      case 7: /*(0, 1)  below sample */
        {
          startY = 0;
          endY   = isBelowAvail ? height : height - 1;

          for (y = startY; y < endY; y++)
          {
            offsetmSampleCcSaoNoClip(0, width, srcY, srcU, srcV, chromaScaleX, srcStrideY, candPosYX, candPosYY,
                                     bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            dst += dstStride;
          }
          break;
        }
      case 8: /*(1, 1)  below right sample */
        {
          startX = 0;
          endX   = isRightAvail ? width : (width - 1);

          for (y = 0; y < height - 1; y++)
          {
            offsetmSampleCcSaoNoClip(startX, endX, srcY, srcU, srcV, chromaScaleX, srcStrideY, candPosYX, candPosYY,
                                     bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            dst += dstStride;
          }

          // last line
          lastLineStartX = isBelowAvail ? startX : (width - 1);
          lastLineEndX   = isBelowRightAvail ? width : (width - 1);
          offsetmSampleCcSaoNoClip(lastLineStartX, lastLineEndX, srcY, srcU, srcV, chromaScaleX, srcStrideY, candPosYX,
                                   candPosYY, bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

          break;
        }
      }
      break;
    }
  case COMP_Cb:
  case COMP_Cr:
    {
      switch (candPosY)
      {
      case 0:   // top left (-1, -1), unlike SAO, CCSAO BO only uses one spatial neighbor sample to derive band
                // information
        /* total 9 cases will come up here
        for (-1,-1) just use the top and middle lines and check for vb */
        {
          startX          = isLeftAvail ? 0 : 1;
          endX            = width;
          // 1st line
          firstLineStartX = isAboveLeftAvail ? 0 : 1;
          firstLineEndX   = isAboveAvail ? endX : 1;
          offsetmChromaSampleCcSaoNoClip(firstLineStartX, firstLineEndX, srcY, srcU, srcV, chromaScaleX, srcStrideY,
                                         candPosYX, candPosYY, bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

          srcY += srcStrideY << chromaScaleY;
          srcU += srcStrideU;
          srcV += srcStrideV;
          dst += dstStride;

          // middle lines
          for (y = 1; y < height; y++)
          {
            offsetmChromaSampleCcSaoNoClip(startX, endX, srcY, srcU, srcV, chromaScaleX, srcStrideY, candPosYX,
                                           candPosYY, bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            dst += dstStride;
          }
        }
        break;
      case 1: /*(0, -1)  top sample */
        {
          startY = isAboveAvail ? 0 : 1;
          endY   = height;
          if (!isAboveAvail)
          {
            srcY += srcStrideY;
            srcU += srcStrideU * chromaScaleYM1;
            srcV += srcStrideV * chromaScaleYM1;
            dst += dstStride;
          }
          for (y = startY; y < endY; y++)
          {
            offsetmChromaSampleCcSaoNoClip(0, width, srcY, srcU, srcV, chromaScaleX, srcStrideY, candPosYX, candPosYY,
                                           bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            dst += dstStride;
          }
          break;
        }
      case 2: /*(0, -1)  top right sample */
        {
          startX          = isLeftAvail ? 0 : 1;
          endX            = width;
          // first line
          firstLineStartX = isAboveAvail ? startX : (width - 1);
          firstLineEndX   = isAboveRightAvail ? width : (width - 1);
          offsetmChromaSampleCcSaoNoClip(firstLineStartX, firstLineEndX, srcY, srcU, srcV, chromaScaleX, srcStrideY,
                                         candPosYX, candPosYY, bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

          srcY += srcStrideY << chromaScaleY;
          srcU += srcStrideU;
          srcV += srcStrideV;
          dst += dstStride;

          // middle lines
          for (y = 1; y < height; y++)
          {
            offsetmChromaSampleCcSaoNoClip(startX, endX, srcY, srcU, srcV, chromaScaleX, srcStrideY, candPosYX,
                                           candPosYY, bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            dst += dstStride;
          }
          break;
        }
      case 3: /*(-1, 0)  left sample */
        {
          startX = isLeftAvail ? 0 : 1;
          endX   = width;

          for (y = 0; y < height; y++)
          {
            offsetmChromaSampleCcSaoNoClip(startX, endX, srcY, srcU, srcV, chromaScaleX, srcStrideY, candPosYX,
                                           candPosYY, bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            dst += dstStride;
          }
          break;
        }
      case 4: /*(0, 0)  current sample */
        {     /* when current sample is choosen there is no more dependency on neighbor samples*/

          for (y = 0; y < height; y++)
          {
            offsetmChromaSampleCcSaoNoClip(0, width, srcY, srcU, srcV, chromaScaleX, srcStrideY, candPosYX, candPosYY,
                                           bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            dst += dstStride;
          }
          break;
        }
      case 5: /*(1, 0)  right sample */
        {
          startX = 0;
          endX   = isRightAvail ? width : (width - 1);

          for (y = 0; y < height; y++)
          {
            offsetmChromaSampleCcSaoNoClip(startX, endX, srcY, srcU, srcV, chromaScaleX, srcStrideY, candPosYX,
                                           candPosYY, bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            dst += dstStride;
          }
          break;
        }
      case 6: /*(-1, 1)  below left sample */
        {
          startX = isLeftAvail ? 0 : 1;
          endX   = width;

          for (y = 1; y < height; y++)
          {
            offsetmChromaSampleCcSaoNoClip(startX, endX, srcY, srcU, srcV, chromaScaleX, srcStrideY, candPosYX,
                                           candPosYY, bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            dst += dstStride;
          }

          // last line
          lastLineStartX = isBelowLeftAvail ? 0 : 1;
          lastLineEndX   = isBelowAvail ? endX : 1;
          offsetmChromaSampleCcSaoNoClip(lastLineStartX, lastLineEndX, srcY, srcU, srcV, chromaScaleX, srcStrideY,
                                         candPosYX, candPosYY, bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

          break;
        }
      case 7: /*(0, 1)  below sample */
        {
          startY = 0;
          endY   = isBelowAvail ? height : height - 1;

          for (y = startY; y < endY; y++)
          {
            offsetmChromaSampleCcSaoNoClip(0, width, srcY, srcU, srcV, chromaScaleX, srcStrideY, candPosYX, candPosYY,
                                           bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            dst += dstStride;
          }
          break;
        }
      case 8: /*(1, 1)  below right sample */
        {
          startX = 0;
          endX   = isRightAvail ? width : (width - 1);

          for (y = 0; y < height - 1; y++)
          {
            offsetmChromaSampleCcSaoNoClip(startX, endX, srcY, srcU, srcV, chromaScaleX, srcStrideY, candPosYX,
                                           candPosYY, bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);

            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            dst += dstStride;
          }
          // last line
          lastLineStartX = isBelowAvail ? startX : (width - 1);
          lastLineEndX   = isBelowRightAvail ? width : (width - 1);
          offsetmChromaSampleCcSaoNoClip(lastLineStartX, lastLineEndX, srcY, srcU, srcV, chromaScaleX, srcStrideY,
                                         candPosYX, candPosYY, bandNumY, bandNumU, bandNumV, bitDepth, offset, dst);
        }
        break;
      }
      break;
    }
  default:
    {
      THROW("Not a supported CCSAO compID\n");
    }
  }
}

void SampleAdaptiveOffset::deriveLoopFilterBoundaryAvailability(CodingStructure &cs, const Position &pos,
                                                                bool &isLeftAvail, bool &isRightAvail,
                                                                bool &isAboveAvail, bool &isBelowAvail,
                                                                bool &isAboveLeftAvail, bool &isAboveRightAvail,
                                                                bool &isBelowLeftAvail, bool &isBelowRightAvail) const
{
  const int         width        = cs.pcv->maxCUWidth;
  const int         height       = cs.pcv->maxCUHeight;
  const CodingUnit *cuCurr       = cs.getCU(pos, ChannelType::LUMA);
  const CodingUnit *cuLeft       = cs.getCU(pos.offset(-width, 0), ChannelType::LUMA);
  const CodingUnit *cuRight      = cs.getCU(pos.offset(width, 0), ChannelType::LUMA);
  const CodingUnit *cuAbove      = cs.getCU(pos.offset(0, -height), ChannelType::LUMA);
  const CodingUnit *cuBelow      = cs.getCU(pos.offset(0, height), ChannelType::LUMA);
  const CodingUnit *cuAboveLeft  = cs.getCU(pos.offset(-width, -height), ChannelType::LUMA);
  const CodingUnit *cuAboveRight = cs.getCU(pos.offset(width, -height), ChannelType::LUMA);
  const CodingUnit *cuBelowLeft  = cs.getCU(pos.offset(-width, height), ChannelType::LUMA);
  const CodingUnit *cuBelowRight = cs.getCU(pos.offset(width, height), ChannelType::LUMA);

  // check cross slice flags
  const bool isLoopFilterAcrossSlicePPS = cs.pps->m_loopFilterAcrossSlicesEnabledFlag;
  if (!isLoopFilterAcrossSlicePPS)
  {
    isLeftAvail       = (cuLeft == nullptr) ? false : CU::isSameSlice(*cuCurr, *cuLeft);
    isAboveAvail      = (cuAbove == nullptr) ? false : CU::isSameSlice(*cuCurr, *cuAbove);
    isRightAvail      = (cuRight == nullptr) ? false : CU::isSameSlice(*cuCurr, *cuRight);
    isBelowAvail      = (cuBelow == nullptr) ? false : CU::isSameSlice(*cuCurr, *cuBelow);
    isAboveLeftAvail  = (cuAboveLeft == nullptr) ? false : CU::isSameSlice(*cuCurr, *cuAboveLeft);
    isAboveRightAvail = (cuAboveRight == nullptr) ? false : CU::isSameSlice(*cuCurr, *cuAboveRight);
    isBelowLeftAvail  = (cuBelowLeft == nullptr) ? false : CU::isSameSlice(*cuCurr, *cuBelowLeft);
    isBelowRightAvail = (cuBelowRight == nullptr) ? false : CU::isSameSlice(*cuCurr, *cuBelowRight);
  }
  else
  {
    isLeftAvail       = (cuLeft != nullptr);
    isAboveAvail      = (cuAbove != nullptr);
    isRightAvail      = (cuRight != nullptr);
    isBelowAvail      = (cuBelow != nullptr);
    isAboveLeftAvail  = (cuAboveLeft != nullptr);
    isAboveRightAvail = (cuAboveRight != nullptr);
    isBelowLeftAvail  = (cuBelowLeft != nullptr);
    isBelowRightAvail = (cuBelowRight != nullptr);
  }

  // check cross tile flags
  const bool isLoopFilterAcrossTilePPS = cs.pps->m_loopFilterAcrossTilesEnabledFlag;
  if (!isLoopFilterAcrossTilePPS)
  {
    isLeftAvail       = (!isLeftAvail) ? false : CU::isSameTile(*cuCurr, *cuLeft);
    isAboveAvail      = (!isAboveAvail) ? false : CU::isSameTile(*cuCurr, *cuAbove);
    isRightAvail      = (!isRightAvail) ? false : CU::isSameTile(*cuCurr, *cuRight);
    isBelowAvail      = (!isBelowAvail) ? false : CU::isSameTile(*cuCurr, *cuBelow);
    isAboveLeftAvail  = (!isAboveLeftAvail) ? false : CU::isSameTile(*cuCurr, *cuAboveLeft);
    isAboveRightAvail = (!isAboveRightAvail) ? false : CU::isSameTile(*cuCurr, *cuAboveRight);
    isBelowLeftAvail  = (!isBelowLeftAvail) ? false : CU::isSameTile(*cuCurr, *cuBelowLeft);
    isBelowRightAvail = (!isBelowRightAvail) ? false : CU::isSameTile(*cuCurr, *cuBelowRight);
  }

  // check cross subpic flags
  const SubPic &curSubPic = cs.pps->getSubPicFromCU(*cuCurr);
  if (!curSubPic.m_loopFilterAcrossSubPicEnabledFlag)
  {
    isLeftAvail       = (!isLeftAvail) ? false : CU::isSameSubPic(*cuCurr, *cuLeft);
    isAboveAvail      = (!isAboveAvail) ? false : CU::isSameSubPic(*cuCurr, *cuAbove);
    isRightAvail      = (!isRightAvail) ? false : CU::isSameSubPic(*cuCurr, *cuRight);
    isBelowAvail      = (!isBelowAvail) ? false : CU::isSameSubPic(*cuCurr, *cuBelow);
    isAboveLeftAvail  = (!isAboveLeftAvail) ? false : CU::isSameSubPic(*cuCurr, *cuAboveLeft);
    isAboveRightAvail = (!isAboveRightAvail) ? false : CU::isSameSubPic(*cuCurr, *cuAboveRight);
    isBelowLeftAvail  = (!isBelowLeftAvail) ? false : CU::isSameSubPic(*cuCurr, *cuBelowLeft);
    isBelowRightAvail = (!isBelowRightAvail) ? false : CU::isSameSubPic(*cuCurr, *cuBelowRight);
  }
}

void SampleAdaptiveOffset::loadOrStoreCCSaoTemporalPredictor(Slice *pcSlice, CcSaoComParam &ccSaoParam)
{
  // Should be before CcSaoControlIdc to assign setNum
  if (pcSlice->isIDRorBLA() || pcSlice->m_pendingRasInit)
  {
    m_ccSaoPrvParamDec[COMP_Y].clear();
    m_ccSaoPrvParamDec[COMP_Cb].clear();
    m_ccSaoPrvParamDec[COMP_Cr].clear();
  }

  // loadCcSaoPrvParam
  for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
  {
    if (ccSaoParam.enabled[compIdx] && ccSaoParam.reusePrv[compIdx])
    {
      int prvId = ccSaoParam.reusePrvId[compIdx];

      CcSaoPrvParam prvParam     = m_ccSaoPrvParamDec[compIdx][prvId];
      ccSaoParam.setNum[compIdx] = prvParam.setNum;
      std::memcpy(ccSaoParam.setEnabled[compIdx], prvParam.setEnabled, sizeof(ccSaoParam.setEnabled[compIdx]));
      std::memcpy(ccSaoParam.setType[compIdx], prvParam.setType, sizeof(ccSaoParam.setType[compIdx]));
      std::memcpy(ccSaoParam.candPos[compIdx], prvParam.candPos, sizeof(ccSaoParam.candPos[compIdx]));
      std::memcpy(ccSaoParam.bandNum[compIdx], prvParam.bandNum, sizeof(ccSaoParam.bandNum[compIdx]));
      std::memcpy(ccSaoParam.offset[compIdx], prvParam.offset, sizeof(ccSaoParam.offset[compIdx]));
    }
  }

  // setup/saveCcSaoPrvParam
  for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
  {
    if (ccSaoParam.enabled[compIdx] && !ccSaoParam.reusePrv[compIdx])
    {
      if (m_ccSaoPrvParamDec[compIdx].size() == MAX_CCSAO_PRV_NUM)
      {
        m_ccSaoPrvParamDec[compIdx].pop_back();
      }

      CcSaoPrvParam prvParam;
      prvParam.temporalId = pcSlice->m_uiTLayer;
      prvParam.setNum     = ccSaoParam.setNum[compIdx];
      std::memcpy(prvParam.setEnabled, ccSaoParam.setEnabled[compIdx], sizeof(prvParam.setEnabled));
      std::memcpy(prvParam.setType, ccSaoParam.setType[compIdx], sizeof(prvParam.setType));
      std::memcpy(prvParam.candPos, ccSaoParam.candPos[compIdx], sizeof(prvParam.candPos));
      std::memcpy(prvParam.bandNum, ccSaoParam.bandNum[compIdx], sizeof(prvParam.bandNum));
      std::memcpy(prvParam.offset, ccSaoParam.offset[compIdx], sizeof(prvParam.offset));

      m_ccSaoPrvParamDec[compIdx].insert(m_ccSaoPrvParamDec[compIdx].begin(), prvParam);
    }
  }
}

//! \}
