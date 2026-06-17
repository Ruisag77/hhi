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

#include "AdaptiveLoopFilterVtm.h"
#include "CodingStructure.h"
#include "Picture.h"
#include <array>
#include <cmath>

AdaptiveLoopFilterVtm::AdaptiveLoopFilterVtm() : AdaptiveLoopFilter()
{
  initAdaptiveLoopFilter();
#if ENABLE_SIMD_OPT_ALF
#ifdef TARGET_SIMD_X86
  initAdaptiveLoopFilterX86();
#endif
#endif
}

void AdaptiveLoopFilterVtm::initAdaptiveLoopFilter()
{
  m_deriveClassificationBlk = deriveClassificationBlk;

  m_filterCcAlf = filterBlkCcAlf;

  m_filter5x5Blk = filterBlk<ALF_FILTER_5>;
  m_filter7x7Blk = filterBlk<ALF_FILTER_7>;
}

// clang-format off
const int AdaptiveLoopFilterVtm::m_fixedFilterSetCoeff[ALF_FIXED_FILTER_NUM][MAX_NUM_ALF_LUMA_COEFF] =
{
  {   0,   0,   2,  -3,   1,  -4,   1,   7,  -1,   1,  -1,   5,   0 },
  {   0,   0,   0,   0,   0,  -1,   0,   1,   0,   0,  -1,   2,   0 },
  {   0,   0,   0,   0,   0,   0,   0,   1,   0,   0,   0,   0,   0 },
  {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  -1,   1,   0 },
  {   2,   2,  -7,  -3,   0,  -5,  13,  22,  12,  -3,  -3,  17,   0 },
  {  -1,   0,   6,  -8,   1,  -5,   1,  23,   0,   2,  -5,  10,   0 },
  {   0,   0,  -1,  -1,   0,  -1,   2,   1,   0,   0,  -1,   4,   0 },
  {   0,   0,   3, -11,   1,   0,  -1,  35,   5,   2,  -9,   9,   0 },
  {   0,   0,   8,  -8,  -2,  -7,   4,   4,   2,   1,  -1,  25,   0 },
  {   0,   0,   1,  -1,   0,  -3,   1,   3,  -1,   1,  -1,   3,   0 },
  {   0,   0,   3,  -3,   0,  -6,   5,  -1,   2,   1,  -4,  21,   0 },
  {  -7,   1,   5,   4,  -3,   5,  11,  13,  12,  -8,  11,  12,   0 },
  {  -5,  -3,   6,  -2,  -3,   8,  14,  15,   2,  -7,  11,  16,   0 },
  {   2,  -1,  -6,  -5,  -2,  -2,  20,  14,  -4,   0,  -3,  25,   0 },
  {   3,   1,  -8,  -4,   0,  -8,  22,   5,  -3,   2, -10,  29,   0 },
  {   2,   1,  -7,  -1,   2, -11,  23,  -5,   0,   2, -10,  29,   0 },
  {  -6,  -3,   8,   9,  -4,   8,   9,   7,  14,  -2,   8,   9,   0 },
  {   2,   1,  -4,  -7,   0,  -8,  17,  22,   1,  -1,  -4,  23,   0 },
  {   3,   0,  -5,  -7,   0,  -7,  15,  18,  -5,   0,  -5,  27,   0 },
  {   2,   0,   0,  -7,   1, -10,  13,  13,  -4,   2,  -7,  24,   0 },
  {   3,   3, -13,   4,  -2,  -5,   9,  21,  25,  -2,  -3,  12,   0 },
  {  -5,  -2,   7,  -3,  -7,   9,   8,   9,  16,  -2,  15,  12,   0 },
  {   0,  -1,   0,  -7,  -5,   4,  11,  11,   8,  -6,  12,  21,   0 },
  {   3,  -2,  -3,  -8,  -4,  -1,  16,  15,  -2,  -3,   3,  26,   0 },
  {   2,   1,  -5,  -4,  -1,  -8,  16,   4,  -2,   1,  -7,  33,   0 },
  {   2,   1,  -4,  -2,   1, -10,  17,  -2,   0,   2, -11,  33,   0 },
  {   1,  -2,   7, -15, -16,  10,   8,   8,  20,  11,  14,  11,   0 },
  {   2,   2,   3, -13, -13,   4,   8,  12,   2,  -3,  16,  24,   0 },
  {   1,   4,   0,  -7,  -8,  -4,   9,   9,  -2,  -2,   8,  29,   0 },
  {   1,   1,   2,  -4,  -1,  -6,   6,   3,  -1,  -1,  -3,  30,   0 },
  {  -7,   3,   2,  10,  -2,   3,   7,  11,  19,  -7,   8,  10,   0 },
  {   0,  -2,  -5,  -3,  -2,   4,  20,  15,  -1,  -3,  -1,  22,   0 },
  {   3,  -1,  -8,  -4,  -1,  -4,  22,   8,  -4,   2,  -8,  28,   0 },
  {   0,   3, -14,   3,   0,   1,  19,  17,   8,  -3,  -7,  20,   0 },
  {   0,   2,  -1,  -8,   3,  -6,   5,  21,   1,   1,  -9,  13,   0 },
  {  -4,  -2,   8,  20,  -2,   2,   3,   5,  21,   4,   6,   1,   0 },
  {   2,  -2,  -3,  -9,  -4,   2,  14,  16,   3,  -6,   8,  24,   0 },
  {   2,   1,   5, -16,  -7,   2,   3,  11,  15,  -3,  11,  22,   0 },
  {   1,   2,   3, -11,  -2,  -5,   4,   8,   9,  -3,  -2,  26,   0 },
  {   0,  -1,  10,  -9,  -1,  -8,   2,   3,   4,   0,   0,  29,   0 },
  {   1,   2,   0,  -5,   1,  -9,   9,   3,   0,   1,  -7,  20,   0 },
  {  -2,   8,  -6,  -4,   3,  -9,  -8,  45,  14,   2, -13,   7,   0 },
  {   1,  -1,  16, -19,  -8,  -4,  -3,   2,  19,   0,   4,  30,   0 },
  {   1,   1,  -3,   0,   2, -11,  15,  -5,   1,   2,  -9,  24,   0 },
  {   0,   1,  -2,   0,   1,  -4,   4,   0,   0,   1,  -4,   7,   0 },
  {   0,   1,   2,  -5,   1,  -6,   4,  10,  -2,   1,  -4,  10,   0 },
  {   3,   0,  -3,  -6,  -2,  -6,  14,   8,  -1,  -1,  -3,  31,   0 },
  {   0,   1,   0,  -2,   1,  -6,   5,   1,   0,   1,  -5,  13,   0 },
  {   3,   1,   9, -19, -21,   9,   7,   6,  13,   5,  15,  21,   0 },
  {   2,   4,   3, -12, -13,   1,   7,   8,   3,   0,  12,  26,   0 },
  {   3,   1,  -8,  -2,   0,  -6,  18,   2,  -2,   3, -10,  23,   0 },
  {   1,   1,  -4,  -1,   1,  -5,   8,   1,  -1,   2,  -5,  10,   0 },
  {   0,   1,  -1,   0,   0,  -2,   2,   0,   0,   1,  -2,   3,   0 },
  {   1,   1,  -2,  -7,   1,  -7,  14,  18,   0,   0,  -7,  21,   0 },
  {   0,   1,   0,  -2,   0,  -7,   8,   1,  -2,   0,  -3,  24,   0 },
  {   0,   1,   1,  -2,   2, -10,  10,   0,  -2,   1,  -7,  23,   0 },
  {   0,   2,   2, -11,   2,  -4,  -3,  39,   7,   1, -10,   9,   0 },
  {   1,   0,  13, -16,  -5,  -6,  -1,   8,   6,   0,   6,  29,   0 },
  {   1,   3,   1,  -6,  -4,  -7,   9,   6,  -3,  -2,   3,  33,   0 },
  {   4,   0, -17,  -1,  -1,   5,  26,   8,  -2,   3, -15,  30,   0 },
  {   0,   1,  -2,   0,   2,  -8,  12,  -6,   1,   1,  -6,  16,   0 },
  {   0,   0,   0,  -1,   1,  -4,   4,   0,   0,   0,  -3,  11,   0 },
  {   0,   1,   2,  -8,   2,  -6,   5,  15,   0,   2,  -7,   9,   0 },
  {   1,  -1,  12, -15,  -7,  -2,   3,   6,   6,  -1,   7,  30,   0 },
};

const int AdaptiveLoopFilterVtm::m_classToFilterMapping[ALF_NUM_FIXED_FILTER_SETS][MAX_NUM_ALF_CLASSES] =
{
  {  8,   2,   2,   2,   3,   4,  53,   9,   9,  52,   4,   4,   5,   9,   2,   8,  10,   9,   1,   3,  39,  39,  10,   9,  52 },
  { 11,  12,  13,  14,  15,  30,  11,  17,  18,  19,  16,  20,  20,   4,  53,  21,  22,  23,  14,  25,  26,  26,  27,  28,  10 },
  { 16,  12,  31,  32,  14,  16,  30,  33,  53,  34,  35,  16,  20,   4,   7,  16,  21,  36,  18,  19,  21,  26,  37,  38,  39 },
  { 35,  11,  13,  14,  43,  35,  16,   4,  34,  62,  35,  35,  30,  56,   7,  35,  21,  38,  24,  40,  16,  21,  48,  57,  39 },
  { 11,  31,  32,  43,  44,  16,   4,  17,  34,  45,  30,  20,  20,   7,   5,  21,  22,  46,  40,  47,  26,  48,  63,  58,  10 },
  { 12,  13,  50,  51,  52,  11,  17,  53,  45,   9,  30,   4,  53,  19,   0,  22,  23,  25,  43,  44,  37,  27,  28,  10,  55 },
  { 30,  33,  62,  51,  44,  20,  41,  56,  34,  45,  20,  41,  41,  56,   5,  30,  56,  38,  40,  47,  11,  37,  42,  57,   8 },
  { 35,  11,  23,  32,  14,  35,  20,   4,  17,  18,  21,  20,  20,  20,   4,  16,  21,  36,  46,  25,  41,  26,  48,  49,  58 },
  { 12,  31,  59,  59,   3,  33,  33,  59,  59,  52,   4,  33,  17,  59,  55,  22,  36,  59,  59,  60,  22,  36,  59,  25,  55 },
  { 31,  25,  15,  60,  60,  22,  17,  19,  55,  55,  20,  20,  53,  19,  55,  22,  46,  25,  43,  60,  37,  28,  10,  55,  52 },
  { 12,  31,  32,  50,  51,  11,  33,  53,  19,  45,  16,   4,   4,  53,   5,  22,  36,  18,  25,  43,  26,  27,  27,  28,  10 },
  {  5,   2,  44,  52,   3,   4,  53,  45,   9,   3,   4,  56,   5,   0,   2,   5,  10,  47,  52,   3,  63,  39,  10,   9,  52 },
  { 12,  34,  44,  44,   3,  56,  56,  62,  45,   9,  56,  56,   7,   5,   0,  22,  38,  40,  47,  52,  48,  57,  39,  10,   9 },
  { 35,  11,  23,  14,  51,  35,  20,  41,  56,  62,  16,  20,  41,  56,   7,  16,  21,  38,  24,  40,  26,  26,  42,  57,  39 },
  { 33,  34,  51,  51,  52,  41,  41,  34,  62,   0,  41,  41,  56,   7,   5,  56,  38,  38,  40,  44,  37,  42,  57,  39,  10 },
  { 16,  31,  32,  15,  60,  30,   4,  17,  19,  25,  22,  20,   4,  53,  19,  21,  22,  46,  25,  55,  26,  48,  63,  58,  55 },
};
// clang-format on

void AdaptiveLoopFilterVtm::applyCcAlfFilter(CodingStructure &cs, CompID compID, const PelBuf &dstBuf,
                                             const PelUnitBuf &recYuvExt, uint8_t *filterControl,
                                             const short filterSet[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF],
                                             const int   selectedFilterIdx)
{
  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;

  int ctuIdx = 0;
  for (int yPos = 0; yPos < m_picHeight; yPos += m_maxCUHeight)
  {
    for (int xPos = 0; xPos < m_picWidth; xPos += m_maxCUWidth)
    {
      int  filterIdx     = (filterControl == nullptr)
             ? selectedFilterIdx
             : filterControl[(yPos >> cs.pcv->maxCUHeightLog2) * cs.pcv->widthInCtus + (xPos >> cs.pcv->maxCUWidthLog2)];
      bool skipFiltering = (filterControl != nullptr && filterIdx == 0) ? true : false;
      if (!skipFiltering)
      {
        if (filterControl != nullptr)
        {
          filterIdx--;
        }

        const int16_t *filterCoeff = filterSet[filterIdx];

        const int width        = (xPos + m_maxCUWidth > m_picWidth) ? (m_picWidth - xPos) : m_maxCUWidth;
        const int height       = (yPos + m_maxCUHeight > m_picHeight) ? (m_picHeight - yPos) : m_maxCUHeight;
        const int chromaScaleX = getComponentScaleX(compID, m_chromaFormat);
        const int chromaScaleY = getComponentScaleY(compID, m_chromaFormat);

        int rasterSliceAlfPad = 0;
        if (isCrossedBySubPicBoundaries(cs, xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight,
                                        rasterSliceAlfPad))
        {
          const bool clipT = clipTop || (yPos == 0);
          const bool clipB = clipBottom || (yPos + height == m_picHeight);
          const bool clipL = clipLeft || (xPos == 0);
          const bool clipR = clipRight || (xPos + width == m_picWidth);
          const int  wBuf  = width + (clipL ? 0 : MAX_ALF_PADDING_SIZE) + (clipR ? 0 : MAX_ALF_PADDING_SIZE);
          const int  hBuf  = height + (clipT ? 0 : MAX_ALF_PADDING_SIZE) + (clipB ? 0 : MAX_ALF_PADDING_SIZE);
          PelUnitBuf buf   = m_tempBuf2.subBuf(UnitArea(cs.area.chromaFormat, Area(0, 0, wBuf, hBuf)));
          buf.copyFrom(recYuvExt.subBuf(UnitArea(
            cs.area.chromaFormat,
            Area(xPos - (clipL ? 0 : MAX_ALF_PADDING_SIZE), yPos - (clipT ? 0 : MAX_ALF_PADDING_SIZE), wBuf, hBuf))));
          // pad top-left unavailable samples for raster slice
          if (rasterSliceAlfPad & 1)
          {
            buf.padBorderPel(MAX_ALF_PADDING_SIZE, 1);
          }

          // pad bottom-right unavailable samples for raster slice
          if (rasterSliceAlfPad & 2)
          {
            buf.padBorderPel(MAX_ALF_PADDING_SIZE, 2);
          }
          buf.extendBorderPel(MAX_ALF_PADDING_SIZE);
          buf = buf.subBuf(
            UnitArea(cs.area.chromaFormat,
                     Area(clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, width, height)));

          const Area blkSrc(0, 0, width, height);

          const Area blkDst(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY);
          m_filterCcAlf(dstBuf, buf, blkDst, blkSrc, compID, filterCoeff, m_clpRngs, cs, m_alfVBLumaCTUHeight,
                        m_alfVBLumaPos);
        }
        else
        {
          const UnitArea area(m_chromaFormat, Area(xPos, yPos, width, height));

          Area blkDst(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY);
          Area blkSrc(xPos, yPos, width, height);

          m_filterCcAlf(dstBuf, recYuvExt, blkDst, blkSrc, compID, filterCoeff, m_clpRngs, cs, m_alfVBLumaCTUHeight,
                        m_alfVBLumaPos);
        }
      }
      ctuIdx++;
    }
  }
}

void AdaptiveLoopFilterVtm::ALFProcess(CodingStructure &cs)
{

  // set clipping range
  m_clpRngs = cs.slice->m_clpRngs;

  // set CTU enable flags
  CHECKD(cs.picture->getAlfModes(CompID::COMP_Y).size() != m_numCTUsInPic,
         "Unexpected number of CTB mode entries (Y).");
  CHECKD(cs.picture->getAlfModes(CompID::COMP_Cb).size() != m_numCTUsInPic,
         "Unexpected number of CTB mode entries (Cb).");
  CHECKD(cs.picture->getAlfModes(CompID::COMP_Cr).size() != m_numCTUsInPic,
         "Unexpected number of CTB mode entries (Cr).");
  m_modes = &cs.picture->getAlfModes();

  CtbModes *lumaModes    = nullptr;
  uint32_t  lastSliceIdx = 0xFFFFFFFF;

  PelUnitBuf recYuv = cs.getRecoBuf();

  // Setup the frame buffer for the reconstruction after SAO.
  m_tempBuf.copyFrom(recYuv);
  PelUnitBuf tmpYuv = m_tempBuf.getBuf(cs.area);
  tmpYuv.extendBorderPel(MAX_ALF_FILTER_LENGTH >> 1);

  const PreCalcValues &pcv = *cs.pcv;

  int  ctuIdx  = 0;
  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;

  for (int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
  {
    for (int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
    {
      // get first CU in CTU
      const CodingUnit *cu = cs.getCU(Position(xPos, yPos), ChannelType::LUMA);

      // skip this CTU if ALF is disabled
      if (!cu->slice->m_alfEnabledFlag[COMP_Y] && !cu->slice->m_alfEnabledFlag[COMP_Cb] &&
          !cu->slice->m_alfEnabledFlag[COMP_Cr])
      {
        ctuIdx++;
        continue;
      }

      const CcAlfFilterParam &sliceCcAlfFilterParam = cu->slice->m_ccAlfFilterParam.getVtmParam();

      // reload ALF APS each time the slice changes during raster scan filtering
      if (ctuIdx == 0 || lastSliceIdx != cu->slice->getSliceID() || lumaModes == nullptr)
      {
        cs.slice = cu->slice;
        reconstructCoeffAPSs(cs, true, cu->slice->m_alfEnabledFlag[COMP_Cb] || cu->slice->m_alfEnabledFlag[COMP_Cr],
                             false);
        CHECKD(cu->slice->m_pic->getAlfModes(COMP_Y).size() != m_numCTUsInPic,
               "Unexpected number of luma CTB mode entries.");
        lumaModes          = &cu->slice->m_pic->getAlfModes(COMP_Y);
        m_ccAlfFilterParam = sliceCcAlfFilterParam;
      }
      lastSliceIdx = cu->slice->getSliceID();

      const int width         = std::min(pcv.maxCUWidth, pcv.lumaWidth - static_cast<unsigned>(xPos));
      const int height        = std::min(pcv.maxCUHeight, pcv.lumaHeight - static_cast<unsigned>(yPos));
      bool      ctuEnableFlag = CtbModeHandler::isEnabled((*m_modes)[COMP_Y][ctuIdx]);
      for (int compIdx = 1; compIdx < MAX_NUM_COMP; compIdx++)
      {
        ctuEnableFlag |= CtbModeHandler::isEnabled((*m_modes)[compIdx][ctuIdx]);
        if (sliceCcAlfFilterParam.ccAlfFilterEnabled[compIdx - 1])
        {
          ctuEnableFlag |= m_ccAlfFilterControl[compIdx - 1][ctuIdx] > 0;
        }
      }

      // CTU of size (width, height) at position (yPos, xPos).

      int rasterSliceAlfPad = 0;
      if (ctuEnableFlag &&
          isCrossedBySubPicBoundaries(cs, xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight,
                                      rasterSliceAlfPad))
      {
        // Sub-unit of size (width, height) at position (xPos, yPos).

        const bool clipT = clipTop || (yPos == 0);
        const bool clipB = clipBottom || (yPos + height == pcv.lumaHeight);
        const bool clipL = clipLeft || (xPos == 0);
        const bool clipR = clipRight || (xPos + width == pcv.lumaWidth);
        const int  wBuf  = width + (clipL ? 0 : MAX_ALF_PADDING_SIZE) + (clipR ? 0 : MAX_ALF_PADDING_SIZE);
        const int  hBuf  = height + (clipT ? 0 : MAX_ALF_PADDING_SIZE) + (clipB ? 0 : MAX_ALF_PADDING_SIZE);

        // Setup reco unit buffer for sub-unit (sub-unit area, extended by neighbour samples and mirroring).
        PelUnitBuf buf = m_tempBuf2.subBuf(UnitArea(cs.area.chromaFormat, Area(0, 0, wBuf, hBuf)));
        buf.copyFrom(tmpYuv.subBuf(UnitArea(
          cs.area.chromaFormat,
          Area(xPos - (clipL ? 0 : MAX_ALF_PADDING_SIZE), yPos - (clipT ? 0 : MAX_ALF_PADDING_SIZE), wBuf, hBuf))));
        // pad top-left unavailable samples for raster slice
        if (rasterSliceAlfPad & 1)
        {
          buf.padBorderPel(MAX_ALF_PADDING_SIZE, 1);
        }

        // pad bottom-right unavailable samples for raster slice
        if (rasterSliceAlfPad & 2)
        {
          buf.padBorderPel(MAX_ALF_PADDING_SIZE, 2);
        }
        buf.extendBorderPel(MAX_ALF_PADDING_SIZE);
        buf =
          buf.subBuf(UnitArea(cs.area.chromaFormat,
                              Area(clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, width, height)));

        if (CtbModeHandler::isEnabled((*m_modes)[COMP_Y][ctuIdx]))
        {
          const Area blkSrc(0, 0, width, height);
          const Area blkDst(xPos, yPos, width, height);
          const int  m = (*lumaModes)[ctuIdx];

          deriveClassification(buf.get(COMP_Y), blkDst, blkSrc);

          m_filter7x7Blk(m_classifier, recYuv, buf, blkDst, blkSrc, COMP_Y, getCoeffVals(m), getClipVals(m),
                         m_clpRngs.comp[COMP_Y], m_alfVBLumaCTUHeight, m_alfVBLumaPos);
        }

        for (int compIdx = 1; compIdx < MAX_NUM_COMP; compIdx++)
        {
          CompID    compID       = CompID(compIdx);
          const int chromaScaleX = getComponentScaleX(compID, tmpYuv.chromaFormat);
          const int chromaScaleY = getComponentScaleY(compID, tmpYuv.chromaFormat);

          if (CtbModeHandler::isEnabled((*m_modes)[compIdx][ctuIdx]))
          {
            const Area     blkSrc(0, 0, width >> chromaScaleX, height >> chromaScaleY);
            const Area     blkDst(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX,
                                  height >> chromaScaleY);
            const unsigned altIdx = ChromaCtbModeHandler::getAlternative((*m_modes)[compIdx][ctuIdx]);

            m_filter5x5Blk(m_classifier, recYuv, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[altIdx],
                           m_chromaClippFinal[altIdx], m_clpRngs.comp[compIdx], m_alfVBChmaCTUHeight, m_alfVBChmaPos);
          }
          if (sliceCcAlfFilterParam.ccAlfFilterEnabled[compIdx - 1])
          {
            const int filterIdx = m_ccAlfFilterControl[compIdx - 1][ctuIdx];

            if (filterIdx != 0)
            {
              const Area blkSrc(0, 0, width, height);
              Area blkDst(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY);

              const int16_t *filterCoeff = m_ccAlfFilterParam.ccAlfCoeff[compIdx - 1][filterIdx - 1];
              m_filterCcAlf(recYuv.get(compID), buf, blkDst, blkSrc, compID, filterCoeff, m_clpRngs, cs,
                            m_alfVBLumaCTUHeight, m_alfVBLumaPos);
            }
          }
        }
      }
      else
      {
        const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));
        if (CtbModeHandler::isEnabled((*m_modes)[COMP_Y][ctuIdx]))
        {
          Area      blk(xPos, yPos, width, height);
          const int m = (*lumaModes)[ctuIdx];

          deriveClassification(tmpYuv.get(COMP_Y), blk, blk);

          m_filter7x7Blk(m_classifier, recYuv, tmpYuv, blk, blk, COMP_Y, getCoeffVals(m), getClipVals(m),
                         m_clpRngs.comp[COMP_Y], m_alfVBLumaCTUHeight, m_alfVBLumaPos);
        }

        for (int compIdx = 1; compIdx < MAX_NUM_COMP; compIdx++)
        {
          CompID    compID       = CompID(compIdx);
          const int chromaScaleX = getComponentScaleX(compID, tmpYuv.chromaFormat);
          const int chromaScaleY = getComponentScaleY(compID, tmpYuv.chromaFormat);

          if (CtbModeHandler::isEnabled((*m_modes)[compIdx][ctuIdx]))
          {
            Area blk(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY);
            const unsigned altIdx = ChromaCtbModeHandler::getAlternative((*m_modes)[compIdx][ctuIdx]);

            m_filter5x5Blk(m_classifier, recYuv, tmpYuv, blk, blk, compID, m_chromaCoeffFinal[altIdx],
                           m_chromaClippFinal[altIdx], m_clpRngs.comp[compIdx], m_alfVBChmaCTUHeight, m_alfVBChmaPos);
          }
          if (sliceCcAlfFilterParam.ccAlfFilterEnabled[compIdx - 1])
          {
            const int filterIdx = m_ccAlfFilterControl[compIdx - 1][ctuIdx];

            if (filterIdx != 0)
            {
              Area blkDst(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY);
              Area blkSrc(xPos, yPos, width, height);

              const int16_t *filterCoeff = m_ccAlfFilterParam.ccAlfCoeff[compIdx - 1][filterIdx - 1];
              m_filterCcAlf(recYuv.get(compID), tmpYuv, blkDst, blkSrc, compID, filterCoeff, m_clpRngs, cs,
                            m_alfVBLumaCTUHeight, m_alfVBLumaPos);
            }
          }
        }
      }
      ctuIdx++;
    }
  }
}

void AdaptiveLoopFilterVtm::reconstructCoeffAPSs(CodingStructure &cs, bool luma, bool chroma, bool isRdo)
{
  // luma
  APS    **aps = cs.slice->m_alfApss;
  AlfParam alfParamTmp;
  APS     *curAPS;
  if (luma)
  {
    for (int i = 0; i < cs.slice->m_numAlfApsIdsLuma; i++)
    {
      int apsIdx = cs.slice->m_alfApsIdsLuma[i];
      curAPS     = aps[apsIdx];
      CHECK(curAPS == nullptr, "invalid APS");
      alfParamTmp = curAPS->m_alfAPSParam.getVtmParam();
      reconstructLumaCoeff(alfParamTmp, isRdo);
      memcpy(m_coeffApsLuma[i], m_coeffFinal, sizeof(m_coeffFinal));
      memcpy(m_clippApsLuma[i], m_clippFinal, sizeof(m_clippFinal));
    }
  }

  // chroma
  if (chroma)
  {
    int apsIdxChroma = cs.slice->m_alfApsIdChroma;
    curAPS           = aps[apsIdxChroma];
    alfParamTmp      = curAPS->m_alfAPSParam.getVtmParam();
    reconstructChromaCoeff(alfParamTmp, isRdo);
  }
}

void AdaptiveLoopFilterVtm::reconstructLumaCoeff(AlfParam &alfParam, const bool isRdo)
{
  const int    factor         = isRdo ? 0 : 1 << COEFF_SCALE_BITS_ALF;
  const int    numCoeff       = 13;
  const bool   nonLinearFlag  = alfParam.lumaNonLinearFlag;
  const int    numCoeffMinus1 = numCoeff - 1;
  const auto  &coeffDeltaIdx  = alfParam.filterCoeffDeltaIdx;
  const int    numFilters     = alfParam.numLumaFilters;
  auto        &coeffSrc       = alfParam.lumaCoeff;
  const auto  &clippSrc       = alfParam.lumaClipp;
  short *const coeffDst       = m_coeffFinal;
  Pel *const   clippDst       = m_clippFinal;

  // TODO: ALF: Why do we change the source coefficients? BS 2023-12-06
  for (int filterIdx = 0; filterIdx < numFilters; ++filterIdx)
  {
    coeffSrc[filterIdx * MAX_NUM_ALF_LUMA_COEFF + numCoeffMinus1] = factor;
  }

  for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; ++classIdx)
  {
    const int filterIdx = coeffDeltaIdx[classIdx];
    CHECK(!(filterIdx >= 0 && filterIdx < numFilters), "Bad coeff delta idx in ALF");

    reconstructCoeffSingleFilter(
      &coeffSrc[filterIdx * MAX_NUM_ALF_LUMA_COEFF], &clippSrc[filterIdx * MAX_NUM_ALF_LUMA_COEFF],
      &coeffDst[classIdx * MAX_NUM_ALF_LUMA_COEFF], &clippDst[classIdx * MAX_NUM_ALF_LUMA_COEFF],
      m_alfClippingValues[ChannelType::LUMA], numCoeffMinus1, nonLinearFlag, isRdo, factor);
  }
}

void AdaptiveLoopFilterVtm::reconstructChromaCoeff(AlfParam &alfParam, const bool isRdo)
{
  const int  factor         = isRdo ? 0 : 1 << COEFF_SCALE_BITS_ALF;
  const int  numCoeff       = 7;
  const bool nonLinearFlag  = alfParam.chromaNonLinearFlag;
  const int  numAlts        = alfParam.numAlternativesChroma;
  const int  numCoeffMinus1 = numCoeff - 1;

  for (int altIdx = 0; altIdx < numAlts; ++altIdx)
  {
    // TODO: ALF: Why do we change the source coefficient? BS 2023-12-06
    alfParam.chromaCoeff[altIdx][numCoeffMinus1] = factor;

    reconstructCoeffSingleFilter(alfParam.chromaCoeff[altIdx].data(), alfParam.chromaClipp[altIdx].data(),
                                 m_chromaCoeffFinal[altIdx], m_chromaClippFinal[altIdx],
                                 m_alfClippingValues[ChannelType::CHROMA], numCoeffMinus1, nonLinearFlag, isRdo,
                                 factor);
  }
}

void AdaptiveLoopFilterVtm::create(const int picWidth, const int picHeight, const ChromaFormat format,
                                   const int maxCUWidth, const int maxCUHeight, const int maxCUDepth,
                                   const BitDepths &inputBitDepth)
{
  AdaptiveLoopFilter::create(picWidth, picHeight, format, maxCUWidth, maxCUHeight, maxCUDepth, inputBitDepth);

  m_filterShapesCcAlf.push_back(AlfFilterShape(CC_ALF));

  m_filterShapes[ChannelType::LUMA].push_back(AlfFilterShape(ALF_FILTER_7));

  m_filterShapes[ChannelType::CHROMA].push_back(AlfFilterShape(ALF_FILTER_5));

  m_alfVBLumaPos = m_maxCUHeight - ALF_VB_POS_ABOVE_CTUROW_LUMA;
  m_alfVBChmaPos = (m_maxCUHeight >> ((m_chromaFormat == ChromaFormat::_420) ? 1 : 0)) - ALF_VB_POS_ABOVE_CTUROW_CHMA;

  m_alfVBLumaCTUHeight = m_maxCUHeight;
  m_alfVBChmaCTUHeight = (m_maxCUHeight >> ((m_chromaFormat == ChromaFormat::_420) ? 1 : 0));

  m_tempBuf.destroy();
  // NOTE: make border 1 sample wider to avoid out-of-bounds memory access in SIMD code (simdDeriveClassificationBlk
  // function)
  m_tempBuf.create(format, Area(0, 0, picWidth, picHeight), maxCUWidth, (MAX_ALF_FILTER_LENGTH + 1) >> 1, 0, false);
  m_tempBuf2.destroy();
  m_tempBuf2.create(format,
                    Area(0, 0, maxCUWidth + (MAX_ALF_PADDING_SIZE << 1), maxCUHeight + (MAX_ALF_PADDING_SIZE << 1)),
                    maxCUWidth, MAX_ALF_PADDING_SIZE, 0, false);

  // Classification
  m_classifier.create(Size(picWidth, picHeight));

  for (int filterSetIndex = 0; filterSetIndex < ALF_NUM_FIXED_FILTER_SETS; filterSetIndex++)
  {
    for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
    {
      int fixedFilterIdx = m_classToFilterMapping[filterSetIndex][classIdx];
      for (int i = 0; i < MAX_NUM_ALF_LUMA_COEFF - 1; i++)
      {
        m_fixedFilterSetCoeffDec[filterSetIndex][classIdx * MAX_NUM_ALF_LUMA_COEFF + i] =
          m_fixedFilterSetCoeff[fixedFilterIdx][i];
      }
      m_fixedFilterSetCoeffDec[filterSetIndex][classIdx * MAX_NUM_ALF_LUMA_COEFF + MAX_NUM_ALF_LUMA_COEFF - 1] = 1
        << COEFF_SCALE_BITS_ALF;
    }
  }
  for (int i = 0; i < MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES; i++)
  {
    m_clipDefault[i] = m_alfClippingValues[ChannelType::LUMA][0];
  }
}

void AdaptiveLoopFilterVtm::destroy()
{
  if (!m_created)
  {
    return;
  }
  AdaptiveLoopFilter::destroy();

  m_classifier.destroy();

  m_filterShapes[ChannelType::LUMA].clear();
  m_filterShapes[ChannelType::CHROMA].clear();
  m_filterShapesCcAlf.clear();
}

void AdaptiveLoopFilterVtm::deriveClassification(const CPelBuf &srcLuma, const Area &blkDst, const Area &blk)
{
  int height = blk.pos().y + blk.height;
  int width  = blk.pos().x + blk.width;

  for (int i = blk.pos().y; i < height; i += CLASSIFICATION_BLK_SIZE)
  {
    int nHeight = std::min(i + CLASSIFICATION_BLK_SIZE, height) - i;

    for (int j = blk.pos().x; j < width; j += CLASSIFICATION_BLK_SIZE)
    {
      int nWidth = std::min(j + CLASSIFICATION_BLK_SIZE, width) - j;

      m_deriveClassificationBlk(
        m_classifier, m_laplacian, srcLuma,
        Area(j - blk.pos().x + blkDst.pos().x, i - blk.pos().y + blkDst.pos().y, nWidth, nHeight),
        Area(j, i, nWidth, nHeight), m_inputBitDepth[ChannelType::LUMA] + 4, m_alfVBLumaCTUHeight, m_alfVBLumaPos);
    }
  }
}

void AdaptiveLoopFilterVtm::deriveClassificationBlk(ClassBuf &classifier, LaplacianBuf &laplacian,
                                                    const CPelBuf &srcLuma, const Area &blkDst, const Area &blk,
                                                    const int shift, const int vbCTUHeight, int vbPos)
{
  CHECK((vbCTUHeight & (vbCTUHeight - 1)) != 0, "vbCTUHeight must be a power of 2");

  static const int th[16] = { 0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4 };

  const ptrdiff_t stride = srcLuma.stride;

  const Pel *src         = srcLuma.buf;
  const int  maxActivity = 15;

  int fl   = 2;
  int flP1 = fl + 1;
  int fl2  = 2 * fl;

  int mainDirection, secondaryDirection, dirTempHV, dirTempD;

  int pixY;
  int height      = blk.height + fl2;
  int width       = blk.width + fl2;
  int posX        = blk.pos().x;
  int posY        = blk.pos().y;
  int startHeight = posY - flP1;

  for (int i = 0; i < height; i += 2)
  {
    const ptrdiff_t yoffset = (i + 1 + startHeight) * stride - flP1;

    const Pel *src0 = &src[yoffset - stride];
    const Pel *src1 = &src[yoffset];
    const Pel *src2 = &src[yoffset + stride];
    const Pel *src3 = &src[yoffset + stride * 2];

    const int y = blkDst.pos().y - 2 + i;
    if (y > 0 && (y & (vbCTUHeight - 1)) == vbPos - 2)
    {
      src3 = &src[yoffset + stride];
    }
    else if (y > 0 && (y & (vbCTUHeight - 1)) == vbPos)
    {
      src0 = &src[yoffset];
    }
    int *pYver  = laplacian[VER][i];
    int *pYhor  = laplacian[HOR][i];
    int *pYdig0 = laplacian[DIAG0][i];
    int *pYdig1 = laplacian[DIAG1][i];

    for (int j = 0; j < width; j += 2)
    {
      pixY              = j + 1 + posX;
      const Pel *pY     = src1 + pixY;
      const Pel *pYdown = src0 + pixY;
      const Pel *pYup   = src2 + pixY;
      const Pel *pYup2  = src3 + pixY;

      const Pel y0   = pY[0] << 1;
      const Pel yup1 = pYup[1] << 1;

      pYver[j]  = abs(y0 - pYdown[0] - pYup[0]) + abs(yup1 - pY[1] - pYup2[1]);
      pYhor[j]  = abs(y0 - pY[1] - pY[-1]) + abs(yup1 - pYup[2] - pYup[0]);
      pYdig0[j] = abs(y0 - pYdown[-1] - pYup[1]) + abs(yup1 - pY[0] - pYup2[2]);
      pYdig1[j] = abs(y0 - pYup[-1] - pYdown[1]) + abs(yup1 - pYup2[0] - pY[2]);

      if (j > 4 && (j - 6) % 4 == 0)
      {
        int jM6 = j - 6;
        int jM4 = j - 4;
        int jM2 = j - 2;
        pYver[jM6] += pYver[jM4] + pYver[jM2] + pYver[j];
        pYhor[jM6] += pYhor[jM4] + pYhor[jM2] + pYhor[j];
        pYdig0[jM6] += pYdig0[jM4] + pYdig0[jM2] + pYdig0[j];
        pYdig1[jM6] += pYdig1[jM4] + pYdig1[jM2] + pYdig1[j];
      }
    }
  }

  // classification block size
  const int clsSizeY = 4;
  const int clsSizeX = 4;

  for (int i = 0; i < blk.height; i += clsSizeY)
  {
    int *pYver  = laplacian[VER][i];
    int *pYver2 = laplacian[VER][i + 2];
    int *pYver4 = laplacian[VER][i + 4];
    int *pYver6 = laplacian[VER][i + 6];

    int *pYhor  = laplacian[HOR][i];
    int *pYhor2 = laplacian[HOR][i + 2];
    int *pYhor4 = laplacian[HOR][i + 4];
    int *pYhor6 = laplacian[HOR][i + 6];

    int *pYdig0  = laplacian[DIAG0][i];
    int *pYdig02 = laplacian[DIAG0][i + 2];
    int *pYdig04 = laplacian[DIAG0][i + 4];
    int *pYdig06 = laplacian[DIAG0][i + 6];

    int *pYdig1  = laplacian[DIAG1][i];
    int *pYdig12 = laplacian[DIAG1][i + 2];
    int *pYdig14 = laplacian[DIAG1][i + 4];
    int *pYdig16 = laplacian[DIAG1][i + 6];

    for (int j = 0; j < blk.width; j += clsSizeX)
    {
      int sumV  = 0;
      int sumH  = 0;
      int sumD0 = 0;
      int sumD1 = 0;
      if (((i + blkDst.pos().y) % vbCTUHeight) == (vbPos - 4))
      {
        sumV  = pYver[j] + pYver2[j] + pYver4[j];
        sumH  = pYhor[j] + pYhor2[j] + pYhor4[j];
        sumD0 = pYdig0[j] + pYdig02[j] + pYdig04[j];
        sumD1 = pYdig1[j] + pYdig12[j] + pYdig14[j];
      }
      else if (((i + blkDst.pos().y) % vbCTUHeight) == vbPos)
      {
        sumV  = pYver2[j] + pYver4[j] + pYver6[j];
        sumH  = pYhor2[j] + pYhor4[j] + pYhor6[j];
        sumD0 = pYdig02[j] + pYdig04[j] + pYdig06[j];
        sumD1 = pYdig12[j] + pYdig14[j] + pYdig16[j];
      }
      else
      {
        sumV  = pYver[j] + pYver2[j] + pYver4[j] + pYver6[j];
        sumH  = pYhor[j] + pYhor2[j] + pYhor4[j] + pYhor6[j];
        sumD0 = pYdig0[j] + pYdig02[j] + pYdig04[j] + pYdig06[j];
        sumD1 = pYdig1[j] + pYdig12[j] + pYdig14[j] + pYdig16[j];
      }

      int tempAct  = sumV + sumH;
      int activity = 0;

      const int y = (i + blkDst.pos().y) & (vbCTUHeight - 1);
      if (y == vbPos - 4 || y == vbPos)
      {
        activity = (Pel)Clip3<int>(0, maxActivity, (tempAct * 96) >> shift);
      }
      else
      {
        activity = (Pel)Clip3<int>(0, maxActivity, (tempAct * 64) >> shift);
      }
      int classIdx = th[activity];

      int hv1, hv0, d1, d0, hvd1, hvd0;

      if (sumV > sumH)
      {
        hv1       = sumV;
        hv0       = sumH;
        dirTempHV = 1;
      }
      else
      {
        hv1       = sumH;
        hv0       = sumV;
        dirTempHV = 3;
      }
      if (sumD0 > sumD1)
      {
        d1       = sumD0;
        d0       = sumD1;
        dirTempD = 0;
      }
      else
      {
        d1       = sumD1;
        d0       = sumD0;
        dirTempD = 2;
      }
      if ((uint32_t)d1 * (uint32_t)hv0 > (uint32_t)hv1 * (uint32_t)d0)
      {
        hvd1               = d1;
        hvd0               = d0;
        mainDirection      = dirTempD;
        secondaryDirection = dirTempHV;
      }
      else
      {
        hvd1               = hv1;
        hvd0               = hv0;
        mainDirection      = dirTempHV;
        secondaryDirection = dirTempD;
      }

      int directionStrength = 0;
      if (hvd1 > 2 * hvd0)
      {
        directionStrength = 1;
      }
      if (hvd1 * 2 > 9 * hvd0)
      {
        directionStrength = 2;
      }

      if (directionStrength)
      {
        classIdx += (((mainDirection & 0x1) << 1) + directionStrength) * 5;
      }

      static const int transposeTable[8] = { 0, 1, 0, 2, 2, 3, 1, 3 };
      int              transposeIdx      = transposeTable[mainDirection * 2 + (secondaryDirection >> 1)];

      int yOffset = i + blkDst.pos().y;
      int xOffset = j + blkDst.pos().x;

      AlfClassifier *cl0 = classifier[yOffset] + xOffset;
      AlfClassifier *cl1 = classifier[yOffset + 1] + xOffset;
      AlfClassifier *cl2 = classifier[yOffset + 2] + xOffset;
      AlfClassifier *cl3 = classifier[yOffset + 3] + xOffset;
      cl0[0] = cl0[1] = cl0[2] = cl0[3] = cl1[0] = cl1[1] = cl1[2] = cl1[3] = cl2[0] = cl2[1] = cl2[2] = cl2[3] =
        cl3[0] = cl3[1] = cl3[2] = cl3[3] = AlfClassifier(classIdx, transposeIdx);
    }
  }
}

template<AdaptiveLoopFilterVtm::AlfFilterType filtType>
void AdaptiveLoopFilterVtm::filterBlk(const ClassBuf &classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc,
                                      const Area &blkDst, const Area &blk, const CompID compId, const short *filterSet,
                                      const Pel *fClipSet, const ClpRng &clpRng, const int vbCTUHeight, int vbPos)
{
  CHECK((vbCTUHeight & (vbCTUHeight - 1)) != 0, "vbCTUHeight must be a power of 2");

  CHECK(isLuma(compId) && (filtType != AlfFilterType::ALF_FILTER_7), "Invalid filter type for luma.");
  CHECK(isChroma(compId) && (filtType != AlfFilterType::ALF_FILTER_5), "Invalid filter type for chroma.");

  CHECK(static_cast<Size>(blkDst) != static_cast<Size>(blk), "Size mismatch between source and target area.");

  const CPelBuf &srcComp = recSrc.get(compId);
  PelBuf         dstComp = recDst.get(compId); // TODO: ALF: No need for a copy but for non-const. BS 2023-09-27

  const ptrdiff_t srcStride = srcComp.stride;
  const ptrdiff_t dstStride = dstComp.stride;

  const short *coef = filterSet;
  const Pel   *clip = fClipSet;

  const int shift  = COEFF_SCALE_BITS_ALF;
  const int offset = 1 << (shift - 1);

  int       transposeIdx = 0;
  const int clsSizeY     = 4;
  const int clsSizeX     = 4;

  CHECK(blk.y % clsSizeY, "Wrong y-coordinate in filtering");
  CHECK(blk.x % clsSizeX, "Wrong x-coordinate in filtering");
  CHECK(blk.height % clsSizeY, "Wrong height in filtering");
  CHECK(blk.width % clsSizeX, "Wrong width in filtering");

  const AlfClassifier *pClass = nullptr;

  const ptrdiff_t dstStride2 = dstStride * clsSizeY;
  const ptrdiff_t srcStride2 = srcStride * clsSizeY;

  // TODO: ALF: These arrays are much too large. BS 2023-09-26
  std::array<int, MAX_NUM_ALF_LUMA_COEFF> filterCoeff;
  std::array<int, MAX_NUM_ALF_LUMA_COEFF> filterClipp;

  constexpr unsigned filterSizeReco = (filtType == ALF_FILTER_7) ? 7 : 5;
  using RowPtrs                     = FilterRowPtrs<const Pel, filterSizeReco>;

  const Pel *pRecSrc0 = srcComp.bufAt(blk.pos());
  Pel       *pRecDst0 = dstComp.bufAt(blkDst.pos());

  for (int i = 0; i < blk.height; i += clsSizeY)
  {
    if (isLuma(compId))
    {
      pClass = classifier[blkDst.y + i] + blkDst.x;
    }

    for (int j = 0; j < blk.width; j += clsSizeX)
    {
      if (isLuma(compId))
      {
        const AlfClassifier &cl = pClass[j];
        transposeIdx            = cl.transposeIdx;
        coef                    = filterSet + cl.classIdx * MAX_NUM_ALF_LUMA_COEFF;
        clip                    = fClipSet + cl.classIdx * MAX_NUM_ALF_LUMA_COEFF;
      }

      // Transpose the filters according to the current classification.
      if (filtType == ALF_FILTER_7)
      {
        if (transposeIdx == 1)
        {
          filterCoeff = { coef[9], coef[4], coef[10], coef[8], coef[1], coef[5], coef[11],
                          coef[7], coef[3], coef[0],  coef[2], coef[6], coef[12] };
          filterClipp = { clip[9], clip[4], clip[10], clip[8], clip[1], clip[5], clip[11],
                          clip[7], clip[3], clip[0],  clip[2], clip[6], clip[12] };
        }
        else if (transposeIdx == 2)
        {
          filterCoeff = { coef[0], coef[3], coef[2], coef[1],  coef[8],  coef[7], coef[6],
                          coef[5], coef[4], coef[9], coef[10], coef[11], coef[12] };
          filterClipp = { clip[0], clip[3], clip[2], clip[1],  clip[8],  clip[7], clip[6],
                          clip[5], clip[4], clip[9], clip[10], clip[11], clip[12] };
        }
        else if (transposeIdx == 3)
        {
          filterCoeff = { coef[9], coef[8], coef[10], coef[4], coef[3], coef[7], coef[11],
                          coef[5], coef[1], coef[0],  coef[2], coef[6], coef[12] };
          filterClipp = { clip[9], clip[8], clip[10], clip[4], clip[3], clip[7], clip[11],
                          clip[5], clip[1], clip[0],  clip[2], clip[6], clip[12] };
        }
        else
        {
          filterCoeff = { coef[0], coef[1], coef[2], coef[3],  coef[4],  coef[5], coef[6],
                          coef[7], coef[8], coef[9], coef[10], coef[11], coef[12] };
          filterClipp = { clip[0], clip[1], clip[2], clip[3],  clip[4],  clip[5], clip[6],
                          clip[7], clip[8], clip[9], clip[10], clip[11], clip[12] };
        }
      }
      else
      {
        if (transposeIdx == 1)
        {
          filterCoeff = { coef[4], coef[1], coef[5], coef[3], coef[0], coef[2], coef[6] };
          filterClipp = { clip[4], clip[1], clip[5], clip[3], clip[0], clip[2], clip[6] };
        }
        else if (transposeIdx == 2)
        {
          filterCoeff = { coef[0], coef[3], coef[2], coef[1], coef[4], coef[5], coef[6] };
          filterClipp = { clip[0], clip[3], clip[2], clip[1], clip[4], clip[5], clip[6] };
        }
        else if (transposeIdx == 3)
        {
          filterCoeff = { coef[4], coef[3], coef[5], coef[1], coef[0], coef[2], coef[6] };
          filterClipp = { clip[4], clip[3], clip[5], clip[1], clip[0], clip[2], clip[6] };
        }
        else
        {
          filterCoeff = { coef[0], coef[1], coef[2], coef[3], coef[4], coef[5], coef[6] };
          filterClipp = { clip[0], clip[1], clip[2], clip[3], clip[4], clip[5], clip[6] };
        }
      }

      for (int ii = 0; ii < clsSizeY; ii++)
      {
        const int yVb = (blkDst.y + i + ii) & (vbCTUHeight - 1);
        int       maxNumNeighborRows;
        if (yVb < vbPos && (yVb >= vbPos - (isChroma(compId) ? 2 : 4)))   // above
        {
          maxNumNeighborRows = vbPos - 1 - yVb;
        }
        else if (yVb >= vbPos && (yVb <= vbPos + (isChroma(compId) ? 1 : 3)))   // bottom
        {
          maxNumNeighborRows = yVb - vbPos;
        }
        else
        {
          maxNumNeighborRows = typename RowPtrs::MaxNumNeighborRows();
        }
        bool isNearVBabove = yVb < vbPos && (yVb >= vbPos - 1);
        bool isNearVBbelow = yVb >= vbPos && (yVb <= vbPos);

        RowPtrs pRecSrc1Rows(pRecSrc0 + ii * srcStride + j, srcStride, maxNumNeighborRows);
        Pel    *pRecDst1 = pRecDst0 + ii * dstStride + j;

        for (int jj = 0; jj < clsSizeX; jj++)
        {
          int32_t   sum  = 0;
          const Pel curr = pRecSrc1Rows[0][0];
          if (filtType == ALF_FILTER_7)
          {
            // 7x7 diamond-shaped filter for the ALF input reconstruction.
            sum += filterCoeff[0] * (clipALF(filterClipp[0], curr, pRecSrc1Rows[3][+0], pRecSrc1Rows[-3][+0]));

            sum += filterCoeff[1] * (clipALF(filterClipp[1], curr, pRecSrc1Rows[2][+1], pRecSrc1Rows[-2][-1]));
            sum += filterCoeff[2] * (clipALF(filterClipp[2], curr, pRecSrc1Rows[2][+0], pRecSrc1Rows[-2][+0]));
            sum += filterCoeff[3] * (clipALF(filterClipp[3], curr, pRecSrc1Rows[2][-1], pRecSrc1Rows[-2][+1]));

            sum += filterCoeff[4] * (clipALF(filterClipp[4], curr, pRecSrc1Rows[1][+2], pRecSrc1Rows[-1][-2]));
            sum += filterCoeff[5] * (clipALF(filterClipp[5], curr, pRecSrc1Rows[1][+1], pRecSrc1Rows[-1][-1]));
            sum += filterCoeff[6] * (clipALF(filterClipp[6], curr, pRecSrc1Rows[1][+0], pRecSrc1Rows[-1][+0]));
            sum += filterCoeff[7] * (clipALF(filterClipp[7], curr, pRecSrc1Rows[1][-1], pRecSrc1Rows[-1][+1]));
            sum += filterCoeff[8] * (clipALF(filterClipp[8], curr, pRecSrc1Rows[1][-2], pRecSrc1Rows[-1][+2]));

            sum += filterCoeff[9] * (clipALF(filterClipp[9], curr, pRecSrc1Rows[0][+3], pRecSrc1Rows[0][-3]));
            sum += filterCoeff[10] * (clipALF(filterClipp[10], curr, pRecSrc1Rows[0][+2], pRecSrc1Rows[0][-2]));
            sum += filterCoeff[11] * (clipALF(filterClipp[11], curr, pRecSrc1Rows[0][+1], pRecSrc1Rows[0][-1]));
          }
          else
          {
            // 5x5 diamond-shaped filter for the ALF input reconstruction.
            sum += filterCoeff[0] * (clipALF(filterClipp[0], curr, pRecSrc1Rows[2][+0], pRecSrc1Rows[-2][+0]));

            sum += filterCoeff[1] * (clipALF(filterClipp[1], curr, pRecSrc1Rows[1][+1], pRecSrc1Rows[-1][-1]));
            sum += filterCoeff[2] * (clipALF(filterClipp[2], curr, pRecSrc1Rows[1][+0], pRecSrc1Rows[-1][+0]));
            sum += filterCoeff[3] * (clipALF(filterClipp[3], curr, pRecSrc1Rows[1][-1], pRecSrc1Rows[-1][+1]));

            sum += filterCoeff[4] * (clipALF(filterClipp[4], curr, pRecSrc1Rows[0][+2], pRecSrc1Rows[0][-2]));
            sum += filterCoeff[5] * (clipALF(filterClipp[5], curr, pRecSrc1Rows[0][+1], pRecSrc1Rows[0][-1]));
          }

          if (!(isNearVBabove || isNearVBbelow))
          {
            sum = (sum + offset) >> shift;
          }
          else
          {
            sum = (sum + (1 << ((shift + 3) - 1))) >> (shift + 3);
          }

          sum += curr;
          pRecDst1[jj] = ClipPel(sum, clpRng);

          ++pRecSrc1Rows;
        }
      }
    }

    pRecSrc0 += srcStride2;
    pRecDst0 += dstStride2;
  }
}

void AdaptiveLoopFilterVtm::filterBlkCcAlf(const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst,
                                           const Area &blkSrc, const CompID compId, const int16_t *filterCoeff,
                                           const ClpRngs &clpRngs, CodingStructure &cs, int vbCTUHeight, int vbPos)
{
  CHECK(1 << floorLog2(vbCTUHeight) != vbCTUHeight, "Not a power of 2");

  CHECK(!isChroma(compId), "Must be chroma");

  const int clsSizeY = 4;
  const int clsSizeX = 4;

  const int scaleX = getComponentScaleX(compId, cs.slice->m_sps->m_chromaFormatIdc);
  const int scaleY = getComponentScaleY(compId, cs.slice->m_sps->m_chromaFormatIdc);

  CHECK(blkDst.y % clsSizeY, "Wrong y-coordinate in filtering");
  CHECK(blkDst.x % clsSizeX, "Wrong x-coordinate in filtering");
  CHECK(blkDst.height % clsSizeY, "Wrong height in filtering");
  CHECK(blkDst.width % clsSizeX, "Wrong width in filtering");

  CPelBuf srcBuf = recSrc.get(COMP_Y);

  const ptrdiff_t lumaStride   = srcBuf.stride;
  const ptrdiff_t chromaStride = dstBuf.stride;

  const Pel *lumaPtr   = srcBuf.buf + blkSrc.y * lumaStride + blkSrc.x;
  Pel       *chromaPtr = dstBuf.buf + blkDst.y * chromaStride + blkDst.x;

  for (int i = 0; i < blkDst.height; i += clsSizeY)
  {
    for (int j = 0; j < blkDst.width; j += clsSizeX)
    {
      for (int ii = 0; ii < clsSizeY; ii++)
      {
        int  row     = ii;
        int  col     = j;
        Pel *srcSelf = chromaPtr + col + row * chromaStride;

        // clang-format off
        ptrdiff_t       offsetM1 = -1 * lumaStride;
        const ptrdiff_t offset0 =   0;
        ptrdiff_t       offsetP1 =  1 * lumaStride;
        ptrdiff_t       offsetP2 =  2 * lumaStride;
        // clang-format on

        row <<= scaleY;
        col <<= scaleX;
        const Pel *srcCross = lumaPtr + col + row * lumaStride;

        int pos = ((blkDst.y + i + ii) << scaleY) & (vbCTUHeight - 1);
        if (scaleY == 0 && (pos == vbPos || pos == vbPos + 1))
        {
          continue;
        }
        if (pos == (vbPos - 2) || pos == (vbPos + 1))
        {
          offsetP2 = offsetP1;
        }
        else if (pos == (vbPos - 1) || pos == vbPos)
        {
          offsetM1 = offset0;
          offsetP1 = offset0;
          offsetP2 = offset0;
        }

        for (int jj = 0; jj < clsSizeX; jj++)
        {
          const int jj2     = (jj << scaleX);
          const int offset0 = 0;

          int       sum          = 0;
          const Pel currSrcCross = srcCross[offset0 + jj2];

          // clang-format off
          sum += filterCoeff[0] * (srcCross[offsetM1 + jj2    ] - currSrcCross);
          sum += filterCoeff[1] * (srcCross[offset0  + jj2 - 1] - currSrcCross);
          sum += filterCoeff[2] * (srcCross[offset0  + jj2 + 1] - currSrcCross);
          sum += filterCoeff[3] * (srcCross[offsetP1 + jj2 - 1] - currSrcCross);
          sum += filterCoeff[4] * (srcCross[offsetP1 + jj2    ] - currSrcCross);
          sum += filterCoeff[5] * (srcCross[offsetP1 + jj2 + 1] - currSrcCross);
          sum += filterCoeff[6] * (srcCross[offsetP2 + jj2    ] - currSrcCross);
          // clang-format on

          sum = (sum + (1 << COEFF_SCALE_BITS_CCALF >> 1)) >> COEFF_SCALE_BITS_CCALF;

          const int offset = 1 << clpRngs.comp[compId].bd >> 1;
          sum              = ClipPel(sum + offset, clpRngs.comp[compId]) - offset;
          sum += srcSelf[jj];
          srcSelf[jj] = ClipPel(sum, clpRngs.comp[compId]);
        }
      }
    }

    chromaPtr += chromaStride * clsSizeY;

    lumaPtr += lumaStride * clsSizeY << scaleY;
  }
}
