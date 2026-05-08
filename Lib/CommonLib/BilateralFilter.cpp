/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
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

#include "BilateralFilter.h"
#include "Unit.h"
#include "UnitTools.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <algorithm>

#include "CommonLib/UnitTools.h"

BilateralFilter::BilateralFilter()
{
  m_bilateralFilterDiamond5x5 = blockBilateralFilterDiamond5x5;
  m_calcMAD                   = calcMAD;

#if ENABLE_SIMD_BILATERAL_FILTER
#ifdef TARGET_SIMD_X86
  initBilateralFilterX86();
#endif
#endif
}

BilateralFilter::~BilateralFilter() {}

void BilateralFilter::create()
{
  for (int qp = 0; qp < 26; qp++)
  {
    for (int i = 0; i < 16; i++)
    {
      m_lut[qp][i]            = ((int)m_wBIF[qp][i] * m_distFactor[0] + 4) >> 3;
      m_lut[qp][16 + i]       = ((int)m_wBIF[qp][i] * m_distFactor[1] + 4) >> 3;
      m_lut[qp][32 + i]       = ((int)m_wBIF[qp][i] * m_distFactor[2] + 4) >> 3;
      m_lutChroma[qp][i]      = ((int)m_wBIFChroma[qp][i] * m_distFactorChroma[0] + 4) >> 3;
      m_lutChroma[qp][16 + i] = ((int)m_wBIFChroma[qp][i] * m_distFactorChroma[1] + 4) >> 3;
      m_lutChroma[qp][32 + i] = ((int)m_wBIFChroma[qp][i] * m_distFactorChroma[2] + 4) >> 3;
    }
  }
}

void BilateralFilter::destroy() {}

inline void bifApplyLut(int diff, int &res, int cutBitsNum, int bitsRound, int bitsRound2, int shift,
                        const int8_t *lutRowPtr, int lutShift)
{
  int sg0  = diff >> shift;
  int v0   = (diff + sg0) ^ sg0;
  int idx  = (v0 + bitsRound) >> cutBitsNum;
  idx      = 15 + ((idx - 15) & ((idx - 15) >> shift));
  int idx2 = (v0 + bitsRound2) >> cutBitsNum;
  idx2     = 15 + ((idx2 - 15) & ((idx2 - 15) >> shift));
  int w0   = (lutRowPtr[lutShift + idx] + lutRowPtr[lutShift + idx2] + 1) >> 1;
  res      = (w0 + sg0) ^ sg0;
}

void BilateralFilter::blockBilateralFilterDiamond5x5(uint32_t uiWidth, uint32_t uiHeight, int16_t block[],
                                                     int16_t blkFilt[], const ClpRng &clpRng, Pel *recPtr,
                                                     ptrdiff_t recStride, int iWidthExtSIMD, int bfac, int bifRoundAdd,
                                                     int bifRoundShift, bool isRDO, const int8_t *lutRowPtr,
                                                     bool noClip, int cutBitsNum)
{
  int pad = NUMBER_PADDED_SAMPLES;

  int padwidth = iWidthExtSIMD;
  int downbuffer[128];
  int downleftbuffer[129];
  int downrightbuffer[2][129];
  int shift     = sizeof(int) * 8 - 1;
  int lutShift1 = 0, lutShift2 = 16, lutShift3 = 32;
  int bitsRound  = 1 << (cutBitsNum - 2);
  int bitsRound2 = bitsRound + (1 << (cutBitsNum - 1));

  for (int x = 0; x < uiWidth; x++)
  {
    int pixel = block[(-1 + pad) * padwidth + x + pad];
    int below = block[(-1 + pad + 1) * padwidth + x + pad];
    bifApplyLut(below - pixel, downbuffer[x], cutBitsNum, bitsRound, bitsRound2, shift, lutRowPtr, lutShift1);

    int belowright = block[(-1 + pad + 1) * padwidth + x + pad + 1];
    bifApplyLut(belowright - pixel, downrightbuffer[1][x + 1], cutBitsNum, bitsRound, bitsRound2, shift, lutRowPtr,
                lutShift2);

    int belowleft = block[(-1 + pad + 1) * padwidth + x + pad - 1];
    bifApplyLut(belowleft - pixel, downleftbuffer[x], cutBitsNum, bitsRound, bitsRound2, shift, lutRowPtr, lutShift2);
  }
  int width = uiWidth;
  for (int y = 0; y < uiHeight; y++)
  {
    int16_t *rowStart = &block[(y + pad) * padwidth + pad];

    int pixel = rowStart[-1];

    int right = rowStart[0], rightmod = 0;
    bifApplyLut(right - pixel, rightmod, cutBitsNum, bitsRound, bitsRound2, shift, lutRowPtr, lutShift1);

    pixel          = rowStart[-padwidth - 1];
    int belowright = right;
    bifApplyLut(belowright - pixel, downrightbuffer[(y + 1) & 1][0], cutBitsNum, bitsRound, bitsRound2, shift,
                lutRowPtr, lutShift2);

    pixel         = rowStart[-padwidth + width];
    int belowleft = rowStart[width - 1];
    bifApplyLut(belowleft - pixel, downleftbuffer[width], cutBitsNum, bitsRound, bitsRound2, shift, lutRowPtr,
                lutShift2);

    for (int x = 0; x < uiWidth; x++)
    {
      pixel      = rowStart[x];
      int modsum = 0, mod = 0;

      int abovemod = -downbuffer[x];
      modsum += abovemod;

      int leftmod = -rightmod;
      modsum += leftmod;

      right = rowStart[x + 1];
      bifApplyLut(right - pixel, mod, cutBitsNum, bitsRound, bitsRound2, shift, lutRowPtr, lutShift1);
      modsum += mod;
      rightmod = mod;

      int below = rowStart[x + padwidth];
      bifApplyLut(below - pixel, mod, cutBitsNum, bitsRound, bitsRound2, shift, lutRowPtr, lutShift1);
      modsum += mod;
      downbuffer[x] = mod;

      int aboverightmod = -downleftbuffer[x + 1];
      modsum += aboverightmod;

      int aboveleftmod = -downrightbuffer[(y + 1) & 1][x];
      modsum += aboveleftmod;

      int belowleft = rowStart[x + padwidth - 1];
      bifApplyLut(belowleft - pixel, mod, cutBitsNum, bitsRound, bitsRound2, shift, lutRowPtr, lutShift2);
      modsum += mod;
      downleftbuffer[x] = mod;

      int belowright = rowStart[x + padwidth + 1];
      bifApplyLut(belowright - pixel, mod, cutBitsNum, bitsRound, bitsRound2, shift, lutRowPtr, lutShift2);
      modsum += mod;
      downrightbuffer[y % 2][x + 1] = mod;

      // For samples two pixels out, we do not reuse previously calculated
      // values even though that is possible. Doing so would likely increase
      // speed when SIMD is turned off.

      int above = rowStart[x - 2 * padwidth];
      bifApplyLut(above - pixel, mod, cutBitsNum, bitsRound, bitsRound2, shift, lutRowPtr, lutShift3);
      modsum += mod;

      below = rowStart[x + 2 * padwidth];
      bifApplyLut(below - pixel, mod, cutBitsNum, bitsRound, bitsRound2, shift, lutRowPtr, lutShift3);
      modsum += mod;

      int left = rowStart[x - 2];
      bifApplyLut(left - pixel, mod, cutBitsNum, bitsRound, bitsRound2, shift, lutRowPtr, lutShift3);
      modsum += mod;

      right = rowStart[x + 2];
      bifApplyLut(right - pixel, mod, cutBitsNum, bitsRound, bitsRound2, shift, lutRowPtr, lutShift3);
      modsum += mod;

      blkFilt[(y + pad) * padwidth + x + pad] =
        ((int16_t)((uint16_t)((modsum * bfac + (bifRoundAdd << 3)) >> (bifRoundShift + 3))));
    }
  }

  // Copy back
  Pel *tempBlockPtr = blkFilt + pad * padwidth + pad;
  if (isRDO)
  {
    Pel *srcBlockPtr = block + pad * padwidth + pad;
    for (uint32_t yy = 0; yy < uiHeight; yy++)
    {
      for (uint32_t xx = 0; xx < uiWidth; xx++)
      {
        recPtr[xx] = ClipPel(srcBlockPtr[xx] + tempBlockPtr[xx], clpRng);
      }
      recPtr += recStride;
      tempBlockPtr += padwidth;
      srcBlockPtr += padwidth;
    }
  }
  else if (noClip)
  {
    for (uint32_t yy = 0; yy < uiHeight; yy++)
    {
      for (uint32_t xx = 0; xx < uiWidth; xx++)
      {
        // new result = old result (which is SAO-treated already) + diff due to bilateral filtering
        recPtr[xx] = recPtr[xx] + tempBlockPtr[xx]; // clipping is done jointly for SAO/BIF/CCSAO
      }
      recPtr += recStride;
      tempBlockPtr += padwidth;
    }
  }
  else
  {
    for (uint32_t yy = 0; yy < uiHeight; yy++)
    {
      for (uint32_t xx = 0; xx < uiWidth; xx++)
      {
        // new result = old result (which is SAO-treated already) + diff due to bilateral filtering
        recPtr[xx] = ClipPel<int>(recPtr[xx] + tempBlockPtr[xx], clpRng);
      }
      recPtr += recStride;
      tempBlockPtr += padwidth;
    }
  }
}

int BilateralFilter::calcMAD(int16_t *block, ptrdiff_t stride, int width, int height, int whlog2)
{
  int average = 0;
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      average += block[j];
    }
    block += stride;
  }
  block -= stride * height;
  average = (average + (1 << (whlog2 - 1))) >> whlog2;
  int mad = 0;
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      mad += std::abs(block[j] - average);
    }
    block += stride;
  }
  mad = (mad + (1 << (whlog2 - 1))) >> whlog2;
  return mad;
}

void BilateralFilter::bilateralFilterRDOdiamond5x5(const CompID compID, PelBuf &resiBuf, const CPelBuf &predBuf,
                                                   PelBuf &recoBuf, int32_t qp, const CPelBuf &recIPredBuf,
                                                   const CPelBuf &recIPredBuf1, const ClpRng &clpRng,
                                                   TransformUnit &currTU, bool useReco, bool doReshape,
                                                   std::vector<Pel> *pLUT)
{
  uint32_t uiWidth  = predBuf.width;
  uint32_t uiHeight = predBuf.height;

  const ptrdiff_t uiPredStride = predBuf.stride;
  const ptrdiff_t uiStrideRes  = resiBuf.stride;
  const ptrdiff_t uiRecStride  = recoBuf.stride;
  const Pel      *piPred       = predBuf.buf;
  Pel            *piResi       = resiBuf.buf;
  Pel            *piReco       = recoBuf.buf;

  const Pel      *piRecIPred       = recIPredBuf.buf;
  const ptrdiff_t uiRecIPredStride = recIPredBuf.stride;

  const Pel *piPredTemp = piPred;
  Pel       *piResiTemp = piResi;
  Pel       *piRecoTemp = piReco;
  // Reco = Pred + Resi

  const uint32_t uiWidthExt  = uiWidth + (NUMBER_PADDED_SAMPLES << 1);
  const uint32_t uiHeightExt = uiHeight + (NUMBER_PADDED_SAMPLES << 1);

  int iWidthExtSIMD = uiWidthExt + 16;

  memset(tempblock, 0, iWidthExtSIMD * uiHeightExt * sizeof(Pel));
  Pel *tempBlockPtr = tempblock + NUMBER_PADDED_SAMPLES * iWidthExtSIMD + NUMBER_PADDED_SAMPLES;

  //// Clip and move block to temporary block
  if (useReco)
  {
    for (uint32_t uiY = 0; uiY < uiHeight; ++uiY)
    {
      std::memcpy(tempBlockPtr, piReco, uiWidth * sizeof(Pel));
      piReco += uiRecStride;
      tempBlockPtr += iWidthExtSIMD;
    }
    piReco = piRecoTemp;
  }
  else
  {
    for (uint32_t uiY = 0; uiY < uiHeight; ++uiY)
    {
      for (uint32_t uiX = 0; uiX < uiWidth; ++uiX)
      {
        tempBlockPtr[uiX] = ClipPel(piPred[uiX] + piResi[uiX], clpRng);
      }
      piPred += uiPredStride;
      piResi += uiStrideRes;
      piReco += uiRecStride;
      tempBlockPtr += iWidthExtSIMD;
    }
  }

  piPred = piPredTemp;
  piResi = piResiTemp;
  piReco = piRecoTemp;

  // Now do non-local filtering
  //
  // If a surrounding block is available, use samples from that block.
  // Otherwise, use block padded using odd padding:  . . a b c -> c b a b c

  // Pad entire block first and then overwrite with samples from surrounding blocks
  // if they pass the test.

  CHECK(NUMBER_PADDED_SAMPLES != 2, "Current implementation is done only for padding size equal to 2");

  for (int yy = 1; yy < uiHeightExt - 1; yy++)
  {
    tempblock[yy * iWidthExtSIMD + NUMBER_PADDED_SAMPLES - 1] = tempblock[yy * iWidthExtSIMD + NUMBER_PADDED_SAMPLES];
    tempblock[yy * iWidthExtSIMD + uiWidthExt - NUMBER_PADDED_SAMPLES] =
      tempblock[yy * iWidthExtSIMD + uiWidthExt - NUMBER_PADDED_SAMPLES - 1];
  }

  std::copy(tempblock + NUMBER_PADDED_SAMPLES * iWidthExtSIMD,
            tempblock + NUMBER_PADDED_SAMPLES * iWidthExtSIMD + uiWidthExt - 1, tempblock + iWidthExtSIMD);
  std::copy(tempblock + (uiHeightExt - NUMBER_PADDED_SAMPLES - 1) * iWidthExtSIMD,
            tempblock + (uiHeightExt - NUMBER_PADDED_SAMPLES - 1) * iWidthExtSIMD + uiWidthExt - 1,
            tempblock + (uiHeightExt - 2) * iWidthExtSIMD);

  const CompArea &area = currTU.blocks[compID];

  bool subTuVer = (area.x > currTU.cu->blocks[compID].x ? true : false);
  bool subTuHor = (area.y > currTU.cu->blocks[compID].y ? true : false);

  uint32_t scaleY = getComponentScaleY(compID, currTU.cu->chromaFormat);

  const bool isCTUBoundary = area.y % (currTU.cs->slice->m_sps->m_ctuSize >> scaleY) == 0;

  bool topAvailable = ((area.y - NUMBER_PADDED_SAMPLES >= 0) && area.y == currTU.cu->blocks[compID].y);
  topAvailable &= !isCTUBoundary;

  bool leftAvailable = ((area.x - NUMBER_PADDED_SAMPLES >= 0) && area.x == currTU.cu->blocks[compID].x);

  // if not 420, then don't use rec for padding
  if (isChroma(compID) && currTU.cu->chromaFormat != ChromaFormat::_420)
  {
    subTuHor      = false;
    subTuVer      = false;
    leftAvailable = false;
    topAvailable  = false;
  }

  int posX = area.x - currTU.cu->blocks[compID].x;
  int posY = area.y - currTU.cu->blocks[compID].y;

  // Fill in samples from blocks that pass the test
  if (topAvailable || leftAvailable || subTuVer || subTuHor)
  {
    if (topAvailable && leftAvailable)
    {
      // top left pixels
      Pel &tmp = tempblock[iWidthExtSIMD + 1];
      tmp      = *(piRecIPred - uiRecIPredStride - 1);

      // Reshape copied pixels if necessary.
      if (doReshape)
      {
        tmp = pLUT->at(tmp);
      }
    }

    // top row
    Pel *tmp = tempblock + NUMBER_PADDED_SAMPLES + iWidthExtSIMD;

    if (topAvailable)
    {
      std::copy(piRecIPred - uiRecIPredStride, piRecIPred - uiRecIPredStride + area.width, tmp);
      if (doReshape)
      {
        for (int xx = 0; xx < area.width; xx++)
        {
          tmp[xx] = pLUT->at(tmp[xx]);
        }
      }
    }
    else if (subTuHor)
    {
      for (int xx = 0; xx < area.width; xx++)
      {
        tmp[xx] = recIPredBuf1.at(posX + xx, posY - 1);
      }
      if (doReshape)
      {
        for (int xx = 0; xx < area.width; xx++)
        {
          tmp[xx] = pLUT->at(tmp[xx]);
        }
      }
    }

    // left column
    tmp = tempblock + iWidthExtSIMD * NUMBER_PADDED_SAMPLES + NUMBER_PADDED_SAMPLES - 1;

    if (leftAvailable)
    {
      for (int yy = 0; yy < area.height; yy++)
      {
        tmp[yy * iWidthExtSIMD] = *(piRecIPred + yy * uiRecIPredStride - 1); // 1 pel out
      }

      if (doReshape)
      {
        for (int yy = 0; yy < area.height; yy++)
        {
          tmp[yy * iWidthExtSIMD] = pLUT->at(tmp[yy * iWidthExtSIMD]);
        }
      }
    }
    else if (subTuVer)
    {

      for (int yy = 0; yy < area.height; yy++)
      {
        tmp[yy * iWidthExtSIMD] = recIPredBuf1.at(posX - 1, posY + yy);
      }

      if (doReshape)
      {
        for (int yy = 0; yy < area.height; yy++)
        {
          tmp[yy * iWidthExtSIMD] = pLUT->at(tmp[yy * iWidthExtSIMD]);
        }
      }
    }
  }

  // copying of outer layer
  for (int yy = 0; yy < uiHeight + NUMBER_PADDED_SAMPLES; yy++)
  {
    tempblock[iWidthExtSIMD + yy * iWidthExtSIMD] = tempblock[iWidthExtSIMD + yy * iWidthExtSIMD + 1];
    tempblock[iWidthExtSIMD + uiWidthExt - 1 + yy * iWidthExtSIMD] =
      tempblock[iWidthExtSIMD + uiWidthExt - 2 + yy * iWidthExtSIMD];
  }
  std::copy(tempblock + iWidthExtSIMD, tempblock + iWidthExtSIMD + uiWidthExt, tempblock);
  std::copy(tempblock + iWidthExtSIMD * (uiHeightExt - 2), tempblock + iWidthExtSIMD * (uiHeightExt - 2) + uiWidthExt,
            tempblock + iWidthExtSIMD * (uiHeightExt - 1));

  int       bfac          = 1;
  const int bifRoundAdd   = BIF_ROUND_ADD >> currTU.cs->pps->m_BIFStrength;
  const int bifRoundShift = BIF_ROUND_SHIFT - currTU.cs->pps->m_BIFStrength;
  const int madShift      = BIF_MAD_SHIFT - currTU.cs->pps->m_BIFStrength;

  const int8_t *lutRowPtr  = nullptr;
  int           cutBitsNum = 3;

  if (isLuma(compID))
  {
    lutRowPtr = getFilterLutParameters(tempblock + iWidthExtSIMD * NUMBER_PADDED_SAMPLES + NUMBER_PADDED_SAMPLES,
                                       iWidthExtSIMD, uiWidth, uiHeight, currTU.cu->predMode,
                                       qp + currTU.cs->pps->m_BIFQPOffset, bfac, madShift);
  }
  else
  {
    cutBitsNum            = 2;
    int widthForStrength  = currTU.blocks[compID].width;
    int heightForStrength = currTU.blocks[compID].height;

    if (currTU.blocks[COMP_Y].valid())
    {
      widthForStrength  = currTU.blocks[COMP_Y].width;
      heightForStrength = currTU.blocks[COMP_Y].height;
    }

    lutRowPtr = getFilterLutParametersChroma(tempblock + iWidthExtSIMD * NUMBER_PADDED_SAMPLES + NUMBER_PADDED_SAMPLES,
                                             iWidthExtSIMD, uiWidth, uiHeight, currTU.cu->predMode,
                                             qp + currTU.cs->pps->m_chromaBIFQPOffset, bfac, madShift, widthForStrength,
                                             heightForStrength, currTU.blocks[COMP_Y].valid());

    CHECK(doReshape, "Reshape domain is not used for chroma");
  }
  m_bilateralFilterDiamond5x5(uiWidth, uiHeight, tempblock, tempblockFiltered, clpRng, piReco, uiRecStride,
                              iWidthExtSIMD, bfac, bifRoundAdd, bifRoundShift, true, lutRowPtr, false, cutBitsNum);

  if (!useReco)
  {
    // need to be performed if residual  is used
    // Resi' = Reco' - Pred
    for (uint32_t uiY = 0; uiY < uiHeight; ++uiY)
    {
      for (uint32_t uiX = 0; uiX < uiWidth; ++uiX)
      {
        piResi[uiX] = piReco[uiX] - piPred[uiX];
      }
      piPred += uiPredStride;
      piResi += uiStrideRes;
      piReco += uiRecStride;
    }
  }
}

void BilateralFilter::bilateralFilterDiamond5x5(const CompID compID, const CPelUnitBuf &src, PelUnitBuf &rec,
                                                int32_t qp, const ClpRng &clpRng, TransformUnit &currTU, bool noClip,
                                                bool isTUCrossedByVirtualBoundaries, int horVirBndryPos[],
                                                int verVirBndryPos[], int numHorVirBndry, int numVerVirBndry,
                                                bool clipTop, bool clipBottom, bool clipLeft, bool clipRight)
{
  const int scaleX = getChannelTypeScaleX(toChannelType(compID), currTU.cu->cs->pcv->chrFormat);
  const int scaleY = getChannelTypeScaleY(toChannelType(compID), currTU.cu->cs->pcv->chrFormat);

  const uint32_t curPicWidth  = currTU.cu->slice->m_pps->m_picWidthInLumaSamples >> scaleX;
  const uint32_t curPicHeight = currTU.cu->slice->m_pps->m_picHeightInLumaSamples >> scaleY;

  if (isTUCrossedByVirtualBoundaries)
  {
    CompArea &compArea = currTU.block(compID);

    const unsigned width  = compArea.width;
    const unsigned height = compArea.height;

    const CompArea &myArea = currTU.blocks[compID];
    int             yPos   = myArea.y;
    int             xPos   = myArea.x;
    int             yStart = yPos;

    for (int i = 0; i <= numHorVirBndry; i++)
    {
      const int  yEnd   = i == numHorVirBndry ? yPos + height : (horVirBndryPos[i] >> scaleY);
      const int  h      = yEnd - yStart;
      const bool clipT  = (i == 0 && clipTop) || (i > 0) || (yStart - 2 < 0);
      const bool clipB  = (i == numHorVirBndry && clipBottom) || (i < numHorVirBndry) || (yEnd + 2 >= curPicHeight);
      int        xStart = xPos;
      for (int j = 0; j <= numVerVirBndry; j++)
      {
        const int  xEnd  = j == numVerVirBndry ? xPos + width : (verVirBndryPos[j] >> scaleX);
        const int  w     = xEnd - xStart;
        const bool clipL = (j == 0 && clipLeft) || (j > 0) || (xStart - 2 < 0);
        const bool clipR = (j == numVerVirBndry && clipRight) || (j < numVerVirBndry) || (xEnd + 2 >= curPicWidth);

        const unsigned uiWidth  = w;
        const unsigned uiHeight = h;

        const Area blkDst(xStart, yStart, uiWidth, uiHeight);
        ptrdiff_t  srcStride  = src.get(compID).stride;
        const Pel *srcPtr     = src.get(compID).bufAt(blkDst);
        const Pel *srcPtrTemp = srcPtr;

        ptrdiff_t recStride = rec.get(compID).stride;
        Pel      *recPtr    = rec.get(compID).bufAt(blkDst);

        bool topAltAvailable  = !clipT;
        bool leftAltAvailable = !clipL;

        bool bottomAltAvailable = !clipB;
        bool rightAltAvailable  = !clipR;

        topAltAvailable    = topAltAvailable && (blkDst.y - NUMBER_PADDED_SAMPLES >= 0);
        leftAltAvailable   = leftAltAvailable && (blkDst.x - NUMBER_PADDED_SAMPLES >= 0);
        bottomAltAvailable = bottomAltAvailable && (blkDst.y + blkDst.height + 1 < curPicHeight);
        rightAltAvailable  = rightAltAvailable && (blkDst.x + blkDst.width + 1 < curPicWidth);

        bool allAvail = topAltAvailable && bottomAltAvailable && leftAltAvailable && rightAltAvailable;

        // if not 420, then don't use rec for padding
        if (isChroma(compID) && currTU.cu->chromaFormat != ChromaFormat::_420)
        {
          topAltAvailable    = false;
          bottomAltAvailable = false;
          leftAltAvailable   = false;
          rightAltAvailable  = false;
          allAvail           = false;
        }

        uint32_t uiWidthExt  = uiWidth + (NUMBER_PADDED_SAMPLES << 1);
        uint32_t uiHeightExt = uiHeight + (NUMBER_PADDED_SAMPLES << 1);

        int iWidthExtSIMD = uiWidthExt + 16;

        Pel *tempBlockPtr;

        memset(tempblock, 0, iWidthExtSIMD * uiHeightExt * sizeof(Pel));

        if (allAvail)
        {
          // set pointer two rows up and two pixels to the left from the start of the block
          tempBlockPtr = tempblock;

          // same with image data
          srcPtr = srcPtr - 2 * srcStride - 2;

          //// Move block to temporary block

          // Check if the block a the top block of a CTU.
          bool isCTUboundary = blkDst.y % (currTU.cs->slice->m_sps->m_ctuSize >> scaleY) == 0;
          if (isCTUboundary)
          {
            // The samples two lines up are out of bounds. (One line above the CTU is OK, since SAO uses that line.)
            // Hence the top line of tempblock is unavailable if the block is the top block of a CTU.
            // Therefore, copy samples from one line up instead of from two lines up by updating srcPtr *before* copy.
            srcPtr += srcStride;
            std::memcpy(tempBlockPtr, srcPtr, uiWidthExt * sizeof(Pel));
          }
          else
          {
            std::memcpy(tempBlockPtr, srcPtr, uiWidthExt * sizeof(Pel));
            srcPtr += srcStride;
          }
          tempBlockPtr += iWidthExtSIMD;
          // Copy samples that are not out of bounds.
          for (uint32_t uiY = 1; uiY < uiHeightExt - 1; ++uiY)
          {
            std::memcpy(tempBlockPtr, srcPtr, uiWidthExt * sizeof(Pel));
            srcPtr += srcStride;
            tempBlockPtr += iWidthExtSIMD;
          }
          // Check if the block is a bottom block of a CTU.
          isCTUboundary = (blkDst.y + uiHeight) % (currTU.cs->slice->m_sps->m_ctuSize >> scaleY) == 0;
          if (isCTUboundary)
          {
            // The samples two lines down are out of bounds. (One line below the CTU is OK, since SAO uses that line.)
            // Hence the bottom line of tempblock is unavailable if the block at the bottom of a CTU.
            // Therefore, copy samples from the second to last line instead of the last line by subtracting srcPtr
            // before copy.
            srcPtr -= srcStride;
            std::memcpy(tempBlockPtr, srcPtr, uiWidthExt * sizeof(Pel));
          }
          else
          {
            std::memcpy(tempBlockPtr, srcPtr, uiWidthExt * sizeof(Pel));
          }
        }
        else
        {
          tempBlockPtr = tempblock + NUMBER_PADDED_SAMPLES * iWidthExtSIMD + NUMBER_PADDED_SAMPLES;

          //// Move block to temporary block
          for (uint32_t uiY = 0; uiY < uiHeight; ++uiY)
          {
            std::memcpy(tempBlockPtr, srcPtr, uiWidth * sizeof(Pel));
            srcPtr += srcStride;
            tempBlockPtr += iWidthExtSIMD;
          }
          srcPtr = srcPtrTemp;

          if (topAltAvailable)
          {
            std::copy(srcPtr - 2 * srcStride, srcPtr - 2 * srcStride + uiWidth, tempblock + 2);
            std::copy(srcPtr - srcStride, srcPtr - srcStride + uiWidth, tempblock + iWidthExtSIMD + 2);
          }
          if (bottomAltAvailable)
          {
            std::copy(srcPtr + (uiHeight + 1) * srcStride, srcPtr + (uiHeight + 1) * srcStride + uiWidth,
                      tempblock + (uiHeightExt - 1) * iWidthExtSIMD + 2);
            std::copy(srcPtr + uiHeight * srcStride, srcPtr + uiHeight * srcStride + uiWidth,
                      tempblock + (uiHeightExt - 2) * iWidthExtSIMD + 2);
          }
          if (leftAltAvailable)
          {
            for (int yy = 0; yy < uiHeight; yy++)
            {
              tempblock[(iWidthExtSIMD << 1) + yy * iWidthExtSIMD + 0] = *(srcPtr + yy * srcStride - 2);
              tempblock[(iWidthExtSIMD << 1) + yy * iWidthExtSIMD + 1] = *(srcPtr + yy * srcStride - 1);
            }
          }
          if (rightAltAvailable)
          {
            for (int yy = 0; yy < uiHeight; yy++)
            {
              tempblock[(iWidthExtSIMD << 1) + uiWidthExt - 1 + yy * iWidthExtSIMD] =
                *(srcPtr + uiWidth + yy * srcStride + 1);
              tempblock[(iWidthExtSIMD << 1) + uiWidthExt - 2 + yy * iWidthExtSIMD] =
                *(srcPtr + uiWidth + yy * srcStride);
            }
          }

          // if not all available, copy from inside tempbuffer
          if (!topAltAvailable)
          {
            std::copy(tempblock + iWidthExtSIMD * 2 + 2, tempblock + iWidthExtSIMD * 2 + 2 + uiWidth, tempblock + 2);
            std::copy(tempblock + iWidthExtSIMD * 2 + 2, tempblock + iWidthExtSIMD * 2 + 2 + uiWidth,
                      tempblock + iWidthExtSIMD + 2);
          }
          if (!bottomAltAvailable)
          {
            std::copy(tempblock + (uiHeightExt - 3) * iWidthExtSIMD + 2,
                      tempblock + (uiHeightExt - 3) * iWidthExtSIMD + 2 + uiWidth,
                      tempblock + (uiHeightExt - 2) * iWidthExtSIMD + 2);
            std::copy(tempblock + (uiHeightExt - 3) * iWidthExtSIMD + 2,
                      tempblock + (uiHeightExt - 3) * iWidthExtSIMD + 2 + uiWidth,
                      tempblock + (uiHeightExt - 1) * iWidthExtSIMD + 2);
          }
          if (!leftAltAvailable)
          {
            for (int yy = 0; yy < uiHeight; yy++)
            {
              tempblock[(iWidthExtSIMD << 1) + yy * iWidthExtSIMD + 0] =
                tempblock[(iWidthExtSIMD << 1) + yy * iWidthExtSIMD + 2];
              tempblock[(iWidthExtSIMD << 1) + yy * iWidthExtSIMD + 1] =
                tempblock[(iWidthExtSIMD << 1) + yy * iWidthExtSIMD + 2];
            }
          }
          if (!rightAltAvailable)
          {
            for (int yy = 0; yy < uiHeight; yy++)
            {
              tempblock[(iWidthExtSIMD << 1) + uiWidthExt - 2 + yy * iWidthExtSIMD] =
                tempblock[(iWidthExtSIMD << 1) + uiWidthExt - 2 + yy * iWidthExtSIMD - 1];
              tempblock[(iWidthExtSIMD << 1) + uiWidthExt - 1 + yy * iWidthExtSIMD] =
                tempblock[(iWidthExtSIMD << 1) + uiWidthExt - 2 + yy * iWidthExtSIMD - 1];
            }
          }

          // All sides are available, easy to just copy corners also.
          if (topAltAvailable && leftAltAvailable)
          {
            tempblock[0]                 = *(srcPtr - 2 * srcStride - 2);   // a     top left corner
            tempblock[1]                 = *(srcPtr - 2 * srcStride - 1);   // b     a b|x x
            tempblock[iWidthExtSIMD + 0] = *(srcPtr - srcStride - 2);   // c     c d|x x
            tempblock[iWidthExtSIMD + 1] = *(srcPtr - srcStride - 1);   // d     -------
          }
          else
          {
            tempblock[0]                 = tempblock[iWidthExtSIMD * 2 + 2];   // extend top left
            tempblock[1]                 = tempblock[iWidthExtSIMD * 2 + 2];   // extend top left
            tempblock[iWidthExtSIMD + 0] = tempblock[iWidthExtSIMD * 2 + 2];   // extend top left
            tempblock[iWidthExtSIMD + 1] = tempblock[iWidthExtSIMD * 2 + 2];   // extend top left
          }

          if (topAltAvailable && rightAltAvailable)
          {
            tempblock[iWidthExtSIMD - 2]              = *(srcPtr - 2 * srcStride + uiWidth);   // a
            tempblock[iWidthExtSIMD - 1]              = *(srcPtr - 2 * srcStride + uiWidth + 1);   // b
            tempblock[iWidthExtSIMD + uiWidthExt - 2] = *(srcPtr - srcStride + uiWidth);   // c
            tempblock[iWidthExtSIMD + uiWidthExt - 1] = *(srcPtr - srcStride + uiWidth + 1);   // d
          }
          else
          {
            tempblock[iWidthExtSIMD - 2] = tempblock[iWidthExtSIMD * 2 + uiWidthExt - 3];   // extend top right
            tempblock[iWidthExtSIMD - 1] = tempblock[iWidthExtSIMD * 2 + uiWidthExt - 3];   // extend top right
            tempblock[iWidthExtSIMD + uiWidthExt - 2] =
              tempblock[iWidthExtSIMD * 2 + uiWidthExt - 3];   // extend top right
            tempblock[iWidthExtSIMD + uiWidthExt - 1] =
              tempblock[iWidthExtSIMD * 2 + uiWidthExt - 3];   // extend top right
          }

          if (bottomAltAvailable && leftAltAvailable)
          {
            tempblock[iWidthExtSIMD * (uiHeightExt - 2) + 0] = *(srcPtr + uiHeight * srcStride - 2);   // a
            tempblock[iWidthExtSIMD * (uiHeightExt - 2) + 1] = *(srcPtr + uiHeight * srcStride - 1);   // b
            tempblock[iWidthExtSIMD * (uiHeightExt - 1) + 0] = *(srcPtr + (uiHeight + 1) * srcStride - 2);   // c
            tempblock[iWidthExtSIMD * (uiHeightExt - 1) + 1] = *(srcPtr + (uiHeight + 1) * srcStride - 1);   // d
          }
          else
          {
            tempblock[iWidthExtSIMD * (uiHeightExt - 2) + 0] =
              tempblock[iWidthExtSIMD * (uiHeightExt - 3) + 2];   // bot avail: mirror left/right
            tempblock[iWidthExtSIMD * (uiHeightExt - 2) + 1] =
              tempblock[iWidthExtSIMD * (uiHeightExt - 3) + 2];   // bot avail: mirror left/right
            tempblock[iWidthExtSIMD * (uiHeightExt - 1) + 0] =
              tempblock[iWidthExtSIMD * (uiHeightExt - 3) + 2];   // bot avail: mirror left/right
            tempblock[iWidthExtSIMD * (uiHeightExt - 1) + 1] =
              tempblock[iWidthExtSIMD * (uiHeightExt - 3) + 2];   // bot avail: mirror left/right
          }

          if (bottomAltAvailable && rightAltAvailable)
          {
            tempblock[iWidthExtSIMD * (uiHeightExt - 2) + uiWidthExt - 2] =
              *(srcPtr + uiHeight * srcStride + uiWidth);   // a
            tempblock[iWidthExtSIMD * (uiHeightExt - 2) + uiWidthExt - 1] =
              *(srcPtr + uiHeight * srcStride + uiWidth + 1);   // b
            tempblock[iWidthExtSIMD * (uiHeightExt - 1) + uiWidthExt - 2] =
              *(srcPtr + (uiHeight + 1) * srcStride + uiWidth);   // c
            tempblock[iWidthExtSIMD * (uiHeightExt - 1) + uiWidthExt - 1] =
              *(srcPtr + (uiHeight + 1) * srcStride + uiWidth + 1);   // d
          }
          else
          {
            tempblock[iWidthExtSIMD * (uiHeightExt - 2) + uiWidthExt - 2] =
              tempblock[iWidthExtSIMD * (uiHeightExt - 3) + uiWidthExt - 3];
            tempblock[iWidthExtSIMD * (uiHeightExt - 2) + uiWidthExt - 1] =
              tempblock[iWidthExtSIMD * (uiHeightExt - 3) + uiWidthExt - 3];
            tempblock[iWidthExtSIMD * (uiHeightExt - 1) + uiWidthExt - 2] =
              tempblock[iWidthExtSIMD * (uiHeightExt - 3) + uiWidthExt - 3];
            tempblock[iWidthExtSIMD * (uiHeightExt - 1) + uiWidthExt - 1] =
              tempblock[iWidthExtSIMD * (uiHeightExt - 3) + uiWidthExt - 3];
          }
        }

        int           bfac       = 1;
        const int8_t *lutRowPtr  = nullptr;
        int           cutBitsNum = 3;
        const int     madShift   = BIF_MAD_SHIFT - currTU.cs->pps->m_BIFStrength;
        if (isLuma(compID))
        {
          lutRowPtr = getFilterLutParameters(tempblock + iWidthExtSIMD * NUMBER_PADDED_SAMPLES + NUMBER_PADDED_SAMPLES,
                                             iWidthExtSIMD, uiWidth, uiHeight, currTU.cu->predMode,
                                             qp + currTU.cs->pps->m_BIFQPOffset, bfac, madShift);
        }
        else
        {
          cutBitsNum            = 2;
          int widthForStrength  = currTU.blocks[compID].width;
          int heightForStrength = currTU.blocks[compID].height;

          if (currTU.blocks[COMP_Y].valid())
          {
            widthForStrength  = currTU.blocks[COMP_Y].width;
            heightForStrength = currTU.blocks[COMP_Y].height;
          }

          lutRowPtr = getFilterLutParametersChroma(
            tempblock + iWidthExtSIMD * NUMBER_PADDED_SAMPLES + NUMBER_PADDED_SAMPLES, iWidthExtSIMD, uiWidth, uiHeight,
            currTU.cu->predMode, qp + currTU.cs->pps->m_chromaBIFQPOffset, bfac, madShift, widthForStrength,
            heightForStrength, currTU.blocks[COMP_Y].valid());
        }

        int bifRoundAdd   = BIF_ROUND_ADD >> currTU.cs->pps->m_BIFStrength;
        int bifRoundShift = BIF_ROUND_SHIFT - currTU.cs->pps->m_BIFStrength;

        m_bilateralFilterDiamond5x5(uiWidth, uiHeight, tempblock, tempblockFiltered, clpRng, recPtr, recStride,
                                    iWidthExtSIMD, bfac, bifRoundAdd, bifRoundShift, false, lutRowPtr, false,
                                    cutBitsNum);

        xStart = xEnd;
      }

      yStart = yEnd;
    }
  }
  else
  {
    CompArea &compArea = currTU.block(compID);

    const unsigned uiWidth  = compArea.width;
    const unsigned uiHeight = compArea.height;

    bool topAltAvailable;
    bool leftAltAvailable;

    ptrdiff_t  srcStride  = src.get(compID).stride;
    const Pel *srcPtr     = src.get(compID).bufAt(compArea);
    const Pel *srcPtrTemp = srcPtr;

    ptrdiff_t recStride = rec.get(compID).stride;
    Pel      *recPtr    = rec.get(compID).bufAt(compArea);

    const CompArea &myArea  = currTU.blocks[compID];
    topAltAvailable         = myArea.y - NUMBER_PADDED_SAMPLES >= 0;
    leftAltAvailable        = myArea.x - NUMBER_PADDED_SAMPLES >= 0;
    bool bottomAltAvailable = myArea.y + myArea.height + 1 < curPicHeight;
    bool rightAltAvailable  = myArea.x + myArea.width + 1 < curPicWidth;

    uint32_t uiWidthExt  = uiWidth + (NUMBER_PADDED_SAMPLES << 1);
    uint32_t uiHeightExt = uiHeight + (NUMBER_PADDED_SAMPLES << 1);

    int iWidthExtSIMD = uiWidthExt + 16;

    Pel *tempBlockPtr;

    bool allAvail = topAltAvailable && bottomAltAvailable && leftAltAvailable && rightAltAvailable;

    // if not 420, then don't use rec for padding
    if (isChroma(compID) && currTU.cu->chromaFormat != ChromaFormat::_420)
    {
      topAltAvailable    = false;
      bottomAltAvailable = false;
      leftAltAvailable   = false;
      rightAltAvailable  = false;
      allAvail           = false;
    }

    memset(tempblock, 0, iWidthExtSIMD * uiHeightExt * sizeof(Pel));

    if (allAvail)
    {
      // set pointer two rows up and two pixels to the left from the start of the block
      tempBlockPtr = tempblock;

      // same with image data
      srcPtr = srcPtr - 2 * srcStride - 2;

      //// Move block to temporary block

      // Check if the block a the top block of a CTU.
      bool isCTUboundary = myArea.y % (currTU.cs->slice->m_sps->m_ctuSize >> scaleX) == 0;
      if (isCTUboundary)
      {
        // The samples two lines up are out of bounds. (One line above the CTU is OK, since SAO uses that line.)
        // Hence the top line of tempblock is unavailable if the block is the top block of a CTU.
        // Therefore, copy samples from one line up instead of from two lines up by updating srcPtr *before* copy.
        srcPtr += srcStride;
        std::memcpy(tempBlockPtr, srcPtr, uiWidthExt * sizeof(Pel));
      }
      else
      {
        std::memcpy(tempBlockPtr, srcPtr, uiWidthExt * sizeof(Pel));
        srcPtr += srcStride;
      }
      tempBlockPtr += iWidthExtSIMD;
      // Copy samples that are not out of bounds.
      for (uint32_t uiY = 1; uiY < uiHeightExt - 1; ++uiY)
      {
        std::memcpy(tempBlockPtr, srcPtr, uiWidthExt * sizeof(Pel));
        srcPtr += srcStride;
        tempBlockPtr += iWidthExtSIMD;
      }
      // Check if the block is a bottom block of a CTU.
      isCTUboundary = (myArea.y + uiHeight) % (currTU.cs->slice->m_sps->m_ctuSize >> scaleY) == 0;
      if (isCTUboundary)
      {
        // The samples two lines down are out of bounds. (One line below the CTU is OK, since SAO uses that line.)
        // Hence the bottom line of tempblock is unavailable if the block at the bottom of a CTU.
        // Therefore, copy samples from the second to last line instead of the last line by subtracting srcPtr before
        // copy.
        srcPtr -= srcStride;
        std::memcpy(tempBlockPtr, srcPtr, uiWidthExt * sizeof(Pel));
      }
      else
      {
        std::memcpy(tempBlockPtr, srcPtr, uiWidthExt * sizeof(Pel));
      }
    }
    else
    {
      tempBlockPtr = tempblock + NUMBER_PADDED_SAMPLES * iWidthExtSIMD + NUMBER_PADDED_SAMPLES;

      //// Move block to temporary block
      for (uint32_t uiY = 0; uiY < uiHeight; ++uiY)
      {
        std::memcpy(tempBlockPtr, srcPtr, uiWidth * sizeof(Pel));
        srcPtr += srcStride;
        tempBlockPtr += iWidthExtSIMD;
      }
      srcPtr = srcPtrTemp;

      if (topAltAvailable)
      {
        std::copy(srcPtr - 2 * srcStride, srcPtr - 2 * srcStride + uiWidth, tempblock + 2);
        std::copy(srcPtr - srcStride, srcPtr - srcStride + uiWidth, tempblock + iWidthExtSIMD + 2);
      }
      if (bottomAltAvailable)
      {
        std::copy(srcPtr + (uiHeight + 1) * srcStride, srcPtr + (uiHeight + 1) * srcStride + uiWidth,
                  tempblock + (uiHeightExt - 1) * iWidthExtSIMD + 2);
        std::copy(srcPtr + uiHeight * srcStride, srcPtr + uiHeight * srcStride + uiWidth,
                  tempblock + (uiHeightExt - 2) * iWidthExtSIMD + 2);
      }
      if (leftAltAvailable)
      {
        for (int yy = 0; yy < uiHeight; yy++)
        {
          tempblock[(iWidthExtSIMD << 1) + yy * iWidthExtSIMD + 0] = *(srcPtr + yy * srcStride - 2);
          tempblock[(iWidthExtSIMD << 1) + yy * iWidthExtSIMD + 1] = *(srcPtr + yy * srcStride - 1);
        }
      }
      if (rightAltAvailable)
      {
        for (int yy = 0; yy < uiHeight; yy++)
        {
          tempblock[(iWidthExtSIMD << 1) + uiWidthExt - 1 + yy * iWidthExtSIMD] =
            *(srcPtr + uiWidth + yy * srcStride + 1);
          tempblock[(iWidthExtSIMD << 1) + uiWidthExt - 2 + yy * iWidthExtSIMD] = *(srcPtr + uiWidth + yy * srcStride);
        }
      }

      // if not all available, copy from inside tempbuffer
      if (!topAltAvailable)
      {
        std::copy(tempblock + iWidthExtSIMD * 2 + 2, tempblock + iWidthExtSIMD * 2 + 2 + uiWidth, tempblock + 2);
        std::copy(tempblock + iWidthExtSIMD * 2 + 2, tempblock + iWidthExtSIMD * 2 + 2 + uiWidth,
                  tempblock + iWidthExtSIMD + 2);
      }
      if (!bottomAltAvailable)
      {
        std::copy(tempblock + (uiHeightExt - 3) * iWidthExtSIMD + 2,
                  tempblock + (uiHeightExt - 3) * iWidthExtSIMD + 2 + uiWidth,
                  tempblock + (uiHeightExt - 2) * iWidthExtSIMD + 2);
        std::copy(tempblock + (uiHeightExt - 3) * iWidthExtSIMD + 2,
                  tempblock + (uiHeightExt - 3) * iWidthExtSIMD + 2 + uiWidth,
                  tempblock + (uiHeightExt - 1) * iWidthExtSIMD + 2);
      }
      if (!leftAltAvailable)
      {
        for (int yy = 0; yy < uiHeight; yy++)
        {
          tempblock[(iWidthExtSIMD << 1) + yy * iWidthExtSIMD + 0] =
            tempblock[(iWidthExtSIMD << 1) + yy * iWidthExtSIMD + 2];
          tempblock[(iWidthExtSIMD << 1) + yy * iWidthExtSIMD + 1] =
            tempblock[(iWidthExtSIMD << 1) + yy * iWidthExtSIMD + 2];
        }
      }
      if (!rightAltAvailable)
      {
        for (int yy = 0; yy < uiHeight; yy++)
        {
          tempblock[(iWidthExtSIMD << 1) + uiWidthExt - 2 + yy * iWidthExtSIMD] =
            tempblock[(iWidthExtSIMD << 1) + uiWidthExt - 2 + yy * iWidthExtSIMD - 1];
          tempblock[(iWidthExtSIMD << 1) + uiWidthExt - 1 + yy * iWidthExtSIMD] =
            tempblock[(iWidthExtSIMD << 1) + uiWidthExt - 2 + yy * iWidthExtSIMD - 1];
        }
      }

      // All sides are available, easy to just copy corners also.
      if (topAltAvailable && leftAltAvailable)
      {
        tempblock[0]                 = *(srcPtr - 2 * srcStride - 2);   // a     top left corner
        tempblock[1]                 = *(srcPtr - 2 * srcStride - 1);   // b     a b|x x
        tempblock[iWidthExtSIMD + 0] = *(srcPtr - srcStride - 2);   // c     c d|x x
        tempblock[iWidthExtSIMD + 1] = *(srcPtr - srcStride - 1);   // d     -------
      }
      else
      {
        tempblock[0]                 = tempblock[iWidthExtSIMD * 2 + 2];   // extend top left
        tempblock[1]                 = tempblock[iWidthExtSIMD * 2 + 2];   // extend top left
        tempblock[iWidthExtSIMD + 0] = tempblock[iWidthExtSIMD * 2 + 2];   // extend top left
        tempblock[iWidthExtSIMD + 1] = tempblock[iWidthExtSIMD * 2 + 2];   // extend top left
      }

      if (topAltAvailable && rightAltAvailable)
      {
        tempblock[iWidthExtSIMD - 2]              = *(srcPtr - 2 * srcStride + uiWidth);   // a
        tempblock[iWidthExtSIMD - 1]              = *(srcPtr - 2 * srcStride + uiWidth + 1);   // b
        tempblock[iWidthExtSIMD + uiWidthExt - 2] = *(srcPtr - srcStride + uiWidth);   // c
        tempblock[iWidthExtSIMD + uiWidthExt - 1] = *(srcPtr - srcStride + uiWidth + 1);   // d
      }
      else
      {
        tempblock[iWidthExtSIMD - 2]              = tempblock[iWidthExtSIMD * 2 + uiWidthExt - 3];   // extend top right
        tempblock[iWidthExtSIMD - 1]              = tempblock[iWidthExtSIMD * 2 + uiWidthExt - 3];   // extend top right
        tempblock[iWidthExtSIMD + uiWidthExt - 2] = tempblock[iWidthExtSIMD * 2 + uiWidthExt - 3];   // extend top right
        tempblock[iWidthExtSIMD + uiWidthExt - 1] = tempblock[iWidthExtSIMD * 2 + uiWidthExt - 3];   // extend top right
      }

      if (bottomAltAvailable && leftAltAvailable)
      {
        tempblock[iWidthExtSIMD * (uiHeightExt - 2) + 0] = *(srcPtr + uiHeight * srcStride - 2);   // a
        tempblock[iWidthExtSIMD * (uiHeightExt - 2) + 1] = *(srcPtr + uiHeight * srcStride - 1);   // b
        tempblock[iWidthExtSIMD * (uiHeightExt - 1) + 0] = *(srcPtr + (uiHeight + 1) * srcStride - 2);   // c
        tempblock[iWidthExtSIMD * (uiHeightExt - 1) + 1] = *(srcPtr + (uiHeight + 1) * srcStride - 1);   // d
      }
      else
      {
        tempblock[iWidthExtSIMD * (uiHeightExt - 2) + 0] =
          tempblock[iWidthExtSIMD * (uiHeightExt - 3) + 2];   // bot avail: mirror left/right
        tempblock[iWidthExtSIMD * (uiHeightExt - 2) + 1] =
          tempblock[iWidthExtSIMD * (uiHeightExt - 3) + 2];   // bot avail: mirror left/right
        tempblock[iWidthExtSIMD * (uiHeightExt - 1) + 0] =
          tempblock[iWidthExtSIMD * (uiHeightExt - 3) + 2];   // bot avail: mirror left/right
        tempblock[iWidthExtSIMD * (uiHeightExt - 1) + 1] =
          tempblock[iWidthExtSIMD * (uiHeightExt - 3) + 2];   // bot avail: mirror left/right
      }

      if (bottomAltAvailable && rightAltAvailable)
      {
        tempblock[iWidthExtSIMD * (uiHeightExt - 2) + uiWidthExt - 2] =
          *(srcPtr + uiHeight * srcStride + uiWidth);   // a
        tempblock[iWidthExtSIMD * (uiHeightExt - 2) + uiWidthExt - 1] =
          *(srcPtr + uiHeight * srcStride + uiWidth + 1);   // b
        tempblock[iWidthExtSIMD * (uiHeightExt - 1) + uiWidthExt - 2] =
          *(srcPtr + (uiHeight + 1) * srcStride + uiWidth);   // c
        tempblock[iWidthExtSIMD * (uiHeightExt - 1) + uiWidthExt - 1] =
          *(srcPtr + (uiHeight + 1) * srcStride + uiWidth + 1);   // d
      }
      else
      {
        tempblock[iWidthExtSIMD * (uiHeightExt - 2) + uiWidthExt - 2] =
          tempblock[iWidthExtSIMD * (uiHeightExt - 3) + uiWidthExt - 3];
        tempblock[iWidthExtSIMD * (uiHeightExt - 2) + uiWidthExt - 1] =
          tempblock[iWidthExtSIMD * (uiHeightExt - 3) + uiWidthExt - 3];
        tempblock[iWidthExtSIMD * (uiHeightExt - 1) + uiWidthExt - 2] =
          tempblock[iWidthExtSIMD * (uiHeightExt - 3) + uiWidthExt - 3];
        tempblock[iWidthExtSIMD * (uiHeightExt - 1) + uiWidthExt - 1] =
          tempblock[iWidthExtSIMD * (uiHeightExt - 3) + uiWidthExt - 3];
      }
    }

    int           bfac       = 1;
    const int8_t *lutRowPtr  = nullptr;
    int           cutBitsNum = 3;
    const int     madShift   = BIF_MAD_SHIFT - currTU.cs->pps->m_BIFStrength;
    if (isLuma(compID))
    {
      lutRowPtr = getFilterLutParameters(tempblock + iWidthExtSIMD * NUMBER_PADDED_SAMPLES + NUMBER_PADDED_SAMPLES,
                                         iWidthExtSIMD, uiWidth, uiHeight, currTU.cu->predMode,
                                         qp + currTU.cs->pps->m_BIFQPOffset, bfac, madShift);
    }
    else
    {
      cutBitsNum            = 2;
      int widthForStrength  = currTU.blocks[compID].width;
      int heightForStrength = currTU.blocks[compID].height;

      if (currTU.blocks[COMP_Y].valid())
      {
        widthForStrength  = currTU.blocks[COMP_Y].width;
        heightForStrength = currTU.blocks[COMP_Y].height;
      }

      lutRowPtr = getFilterLutParametersChroma(
        tempblock + iWidthExtSIMD * NUMBER_PADDED_SAMPLES + NUMBER_PADDED_SAMPLES, iWidthExtSIMD, uiWidth, uiHeight,
        currTU.cu->predMode, qp + currTU.cs->pps->m_chromaBIFQPOffset, bfac, madShift, widthForStrength,
        heightForStrength, currTU.blocks[COMP_Y].valid());
    }

    int bifRoundAdd   = BIF_ROUND_ADD >> currTU.cs->pps->m_BIFStrength;
    int bifRoundShift = BIF_ROUND_SHIFT - currTU.cs->pps->m_BIFStrength;

    m_bilateralFilterDiamond5x5(uiWidth, uiHeight, tempblock, tempblockFiltered, clpRng, recPtr, recStride,
                                iWidthExtSIMD, bfac, bifRoundAdd, bifRoundShift, false, lutRowPtr, noClip, cutBitsNum);
  }
}
void BilateralFilter::clipNotBilaterallyFilteredBlocks(const CompID compID, const CPelUnitBuf &src, PelUnitBuf &rec,
                                                       const ClpRng &clpRng, TransformUnit &currTU)
{
  PelUnitBuf myRecBuf = currTU.cs->getRecoBuf(currTU);
  if (myRecBuf.bufs[compID].width > 1)
  {
    // new result = old result (which is SAO-treated already) + diff due to bilateral filtering
    myRecBuf.bufs[compID].copyClip(myRecBuf.bufs[compID], clpRng);
  }
  else
  {
    CompArea      &compArea  = currTU.block(compID);
    const unsigned uiHeight  = compArea.height;
    ptrdiff_t      recStride = rec.get(compID).stride;
    Pel           *recPtr    = rec.get(compID).bufAt(compArea);

    for (uint32_t yy = 0; yy < uiHeight; yy++)
    {
      // new result = old result (which is SAO-treated already) + diff due to bilateral filtering
      recPtr[0] = ClipPel<int>(recPtr[0], clpRng);
      recPtr += recStride;
    }
  }
}

static double getDist(PelBuf &recBuf, PelBuf &origBuf)
{
  double dist = 0;
  assert(recBuf.height == origBuf.height);
  assert(recBuf.width == origBuf.width);

  ptrdiff_t  recStride = recBuf.stride;
  const Pel *recPtr    = recBuf.buf;

  ptrdiff_t  orgStride = origBuf.stride;
  const Pel *orgPtr    = origBuf.buf;

  for (int yy = 0; yy < recBuf.height; yy++)
  {
    for (int xx = 0; xx < recBuf.width; xx++)
    {
      int diff = (*recPtr++) - (*orgPtr++);
      dist     = dist + ((double)diff) * ((double)diff);
    }
    recPtr += recStride - recBuf.width;
    orgPtr += orgStride - recBuf.width;
  }
  return dist;
}

void copyBack(PelBuf &srcBuf, PelBuf &dstBuf)
{
  assert((srcBuf.height == dstBuf.height));
  assert((srcBuf.width == dstBuf.width));

  ptrdiff_t  srcStride = srcBuf.stride;
  const Pel *srcPtr    = srcBuf.buf;

  ptrdiff_t dstStride = dstBuf.stride;
  Pel      *dstPtr    = dstBuf.buf;

  for (int yy = 0; yy < srcBuf.height; yy++)
  {
    for (int xx = 0; xx < srcBuf.width; xx++)
    {
      (*dstPtr++) = (*srcPtr++);
    }
    srcPtr += srcStride - srcBuf.width;
    dstPtr += dstStride - srcBuf.width;
  }
}

bool BilateralFilter::isCrossedByVirtualBoundaries(const CodingStructure &cs, const int xPos, const int yPos,
                                                   const int width, const int height, bool &clipTop, bool &clipBottom,
                                                   bool &clipLeft, bool &clipRight, int &numHorVirBndry,
                                                   int &numVerVirBndry, int horVirBndryPos[], int verVirBndryPos[],
                                                   bool isEncoderRDO)
{
  clipTop        = false;
  clipBottom     = false;
  clipLeft       = false;
  clipRight      = false;
  numHorVirBndry = 0;
  numVerVirBndry = 0;
  const PPS *pps = cs.pps;

  if (!isEncoderRDO)
  {
    const Slice      &slice   = *(cs.slice);
    int               ctuSize = slice.m_sps->m_ctuSize;
    const Position    currCtuPos(xPos, yPos);
    const CodingUnit *currCtu                           = cs.getCU(currCtuPos, ChannelType::LUMA);
    const SubPic     &curSubPic                         = slice.m_pps->getSubPicFromPos(currCtuPos);
    bool              loopFilterAcrossSubPicEnabledFlag = curSubPic.m_loopFilterAcrossSubPicEnabledFlag;
    // top -> dont need clipping for top of the frame/picture
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

    // bottom -> dont need clipping for bottom of the frame/picture
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

    // left ->  dont need clipping for left of the frame/picture
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

    // right -> dont need clipping for right of the frame/picture
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
  }
  return numHorVirBndry > 0 || numVerVirBndry > 0 || clipTop || clipBottom || clipLeft || clipRight;
}

void BilateralFilter::bilateralFilterPicRDOperCTU(const CompID compID, CodingStructure &cs, PelUnitBuf &src,
                                                  BIFCabacEst *bifCABACEstimator)
{
  // We must have already copied recobuf into src before running this
  // such as src.copyFrom(rec);

  const PreCalcValues &pcv = *cs.pcv;

  PelUnitBuf rec = cs.getRecoBuf();

  const int scaleX = getChannelTypeScaleX(toChannelType(compID), cs.pcv->chrFormat);
  const int scaleY = getChannelTypeScaleY(toChannelType(compID), cs.pcv->chrFormat);

  double frameMseBifOff    = 0;
  double frameMseBifAllOn  = 0;
  double frameMseBifSwitch = 0;

  BifParams &bifParams = cs.picture->getBifParam(compID);
  int        ctuIdx    = 0;

  for (int y = 0; y < pcv.heightInCtus; y++)
  {
    for (int x = 0; x < pcv.widthInCtus; x++)
    {
      UnitArea ctuArea(pcv.chrFormat,
                       Area(x << pcv.maxCUWidthLog2, y << pcv.maxCUHeightLog2, pcv.maxCUWidth, pcv.maxCUWidth));

      ctuArea          = clipArea(ctuArea, *cs.slice->m_pic);
      PelBuf piOrg     = cs.getOrgBuf(ctuArea).bufs[compID];
      PelBuf piSrc     = src.subBuf(ctuArea).bufs[compID];
      double mseBifOff = getDist(piSrc, piOrg);

      bool        isDualTree = CS::isDualITree(cs);
      ChannelType chType     = (isDualTree && isChroma(compID)) ? ChannelType::CHROMA : ChannelType::LUMA;

      for (auto &currCU: cs.traverseCUs(CS::getArea(cs, ctuArea, chType), chType))
      {
        bool valid = isLuma(compID) ? true : currCU.blocks[compID].valid();
        if (!valid)
        {
          continue;
        }

        for (auto &currTU: CU::traverseTUs(currCU))
        {
          bool applyBIF = getApplyBIF(currTU, compID);

          // We should ideally also check the CTU-BIF-flag here. However, given that this function
          // is only called by the encoder, and the encoder always has CTU-BIF-flag on, there is no
          // need to check.

          if (applyBIF)
          {
            int  numHorVirBndry = 0, numVerVirBndry = 0;
            int  horVirBndryPos[] = { 0, 0, 0 };
            int  verVirBndryPos[] = { 0, 0, 0 };
            bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;

            CompArea &myArea = currTU.block(compID);
            int       yPos   = myArea.y << scaleY;
            int       xPos   = myArea.x << scaleX;

            bool isTUCrossedByVirtualBoundaries = isCrossedByVirtualBoundaries(
              cs, xPos, yPos, myArea.width << scaleX, myArea.height << scaleY, clipTop, clipBottom, clipLeft, clipRight,
              numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);
            bilateralFilterDiamond5x5(compID, src, rec, currTU.cu->qp, cs.slice->clpRng(compID), currTU, false,
                                      isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry,
                                      numVerVirBndry, clipTop, clipBottom, clipLeft, clipRight);
          }
        }
      }

      PelBuf piRec    = rec.subBuf(ctuArea).bufs[compID];
      double mseBifOn = getDist(piRec, piOrg);
      frameMseBifOff += mseBifOff;
      frameMseBifAllOn += mseBifOn;

      if (mseBifOff < mseBifOn)
      {
        frameMseBifSwitch += mseBifOff;
        bifParams.ctuOn[ctuIdx] = 0;
      }
      else
      {
        frameMseBifSwitch += mseBifOn;
        bifParams.ctuOn[ctuIdx] = 1;
      }
      ctuIdx++;
    }
  }

  double lambda = cs.slice->getLambdas()[compID];
  double costAllCTUsBIF =
    frameMseBifAllOn + lambda * 1;   // To turn everything on, only slice_bif_all_ctb_enabled_flag = 1, so one bit.
  double costNoCTUsBIF = frameMseBifOff +
    lambda * 2;   // To turn everything off, slice_bif_all_ctb_enabled = 0 && slice_bif_enabled_flag = 0, so two bits.
  double costSwitchCTUsBIF;

  // Does CABAC estimation instead
  const double fracBitsScale = 1.0 / double(1 << SCALE_BITS);

  bifParams.frmOn    = 1;
  bifParams.allCtuOn = 0;

  double ctuSwitchBits = fracBitsScale * bifCABACEstimator->getBits(compID, *cs.slice, bifParams);
  costSwitchCTUsBIF    = frameMseBifSwitch + lambda * ctuSwitchBits;

  double bestCost = MAX_DOUBLE;
  if (costAllCTUsBIF < bestCost)
  {
    // If everything should be BIF-filtered, we do not need to change any of the samples,
    // since they are already filtered.
    bestCost           = costAllCTUsBIF;
    bifParams.frmOn    = 1;
    bifParams.allCtuOn = 1;
  }
  if (costSwitchCTUsBIF < bestCost)
  {
    bestCost           = costSwitchCTUsBIF;
    bifParams.frmOn    = 1;
    bifParams.allCtuOn = 0;

    // If only some CTUs should be BIF-filtered, we need to restore the ones
    // that should not be filtered. This test must be done before the above one
    // since it is partly destroying our filtered data.
    ctuIdx = 0;
    for (int y = 0; y < pcv.heightInCtus; y++)
    {
      for (int x = 0; x < pcv.widthInCtus; x++)
      {
        UnitArea ctuArea(pcv.chrFormat,
                         Area(x << pcv.maxCUWidthLog2, y << pcv.maxCUHeightLog2, pcv.maxCUWidth, pcv.maxCUWidth));
        ctuArea      = clipArea(ctuArea, *cs.slice->m_pic);
        PelBuf piRec = rec.subBuf(ctuArea).bufs[compID];
        PelBuf piSrc = src.subBuf(ctuArea).bufs[compID];

        if (!bifParams.ctuOn[ctuIdx])
        {
          copyBack(piSrc, piRec);   // Copy unfiltered samples back to rec
        }

        ctuIdx++;
      }
    }
  }
  if (costNoCTUsBIF < bestCost)
  {
    bestCost           = costNoCTUsBIF;
    bifParams.frmOn    = 0;
    bifParams.allCtuOn = 0;
    // If no CTUs should be BIF-filtered, we need to restore all CTUs.
    // Note that this test must be done last since it is destroying all
    // of our filtered data.
    if (cs.pps->m_chromaBIF)
    {
      for (int y = 0; y < pcv.heightInCtus; y++)
      {
        for (int x = 0; x < pcv.widthInCtus; x++)
        {
          UnitArea ctuArea(pcv.chrFormat,
                           Area(x << pcv.maxCUWidthLog2, y << pcv.maxCUHeightLog2, pcv.maxCUWidth, pcv.maxCUWidth));
          ctuArea      = clipArea(ctuArea, *cs.slice->m_pic);
          PelBuf piRec = rec.subBuf(ctuArea).bufs[compID];
          PelBuf piSrc = src.subBuf(ctuArea).bufs[compID];
          copyBack(piSrc, piRec);   // Copy unfiltered samples back to rec
        }
      }
    }
    else
    {
      rec.copyFrom(src);
    }
  }
  bifParams.initCTUflag(pcv.heightInCtus * pcv.widthInCtus, bifParams.frmOn, bifParams.allCtuOn);
}

const int8_t *BilateralFilter::getFilterLutParameters(int16_t *block, const ptrdiff_t stride, const int width,
                                                      const int height, const PredMode predMode, const int32_t qp,
                                                      int &bfac, int madShift)
{
  int w = floorLog2(width);
  int h = floorLog2(height);

  bfac = m_tuSizeFactor[predMode == MODE_INTER][std::min(w, 7) * 8 + std::min(h, 7)];
  if (bfac)   // BIF is not applied if tuSizeFactor[(w, h)] = 0
  {
    int mad = m_calcMAD(block, stride, width, height, w + h);
    bfac += m_tuMADFactor[predMode == MODE_INTER][std::min(mad >> madShift, 15)];
  }

  int sqp = qp - BifParams::BIF_QP;
  sqp     = std::min(sqp, 25);
  sqp     = std::max(sqp, 0);
  return m_lut[sqp];
}

const int8_t *BilateralFilter::getFilterLutParametersChroma(int16_t *block, const ptrdiff_t stride, const int width,
                                                            const int height, const PredMode predMode, const int32_t qp,
                                                            int &bfac, int madShift, int widthForStrength,
                                                            int heightForStrength, bool isLumaValid)
{
  int w = floorLog2(widthForStrength);
  int h = floorLog2(heightForStrength);

  bfac = m_tuSizeFactorChroma[predMode == MODE_INTER][std::min(w, 7) * 8 + std::min(h, 7)];
  if (bfac)   // BIF is not applied if tuSizeFactor[(w, h)] = 0
  {
    int mad = m_calcMAD(block, stride, width, height, floorLog2(width) + floorLog2(height));
    bfac += m_tuMADFactorChroma[predMode == MODE_INTER][std::min(mad >> madShift, 15)];
  }

  int sqp = qp - BifParams::BIF_QP;
  sqp     = std::min(sqp, 25);
  sqp     = std::max(sqp, 0);
  return m_lutChroma[sqp];
}

bool BilateralFilter::getApplyBIF(const TransformUnit &currTU, CompID compID)
{
  bool applyBIF = currTU.blocks[compID].valid() && (TU::getCbf(currTU, compID) || currTU.cu->predMode != MODE_INTER) &&
    (currTU.cu->qp > BifParams::BIF_QP);
  if (applyBIF)
  {
    int w = currTU.blocks[compID].width, h = currTU.blocks[compID].height;
    int8_t (*factor)[64] = nullptr;
    if (isLuma(compID))
    {
      factor = m_tuSizeFactor;
    }
    if (!isLuma(compID))
    {
      if (currTU.blocks[COMP_Y].valid())
      {
        w = currTU.blocks[COMP_Y].width, h = currTU.blocks[COMP_Y].height;
      }
      factor = m_tuSizeFactorChroma;
    }
    w = floorLog2(w), h = floorLog2(h);
    w = std::min(w, 7), h = std::min(h, 7);
    applyBIF = factor && factor[currTU.cu->predMode == MODE_INTER][w * 8 + h];
  }
  return applyBIF;
}
