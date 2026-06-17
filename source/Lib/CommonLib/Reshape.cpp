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

/** \file     Reshape.cpp
    \brief    common reshaper class
*/
#include "Reshape.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <UnitTools.h>
 //! \ingroup CommonLib
 //! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

void Reshape::createDec(int bitDepth)
{
  m_lumaBD         = bitDepth;
  m_reshapeLUTSize = 1 << m_lumaBD;
  m_initCW         = m_reshapeLUTSize / PIC_CODE_CW_BINS;
  m_fwdLUT.resize(m_reshapeLUTSize, 0);
  m_invLUT.resize(m_reshapeLUTSize, 0);
  if (m_binCW.empty())
  {
    m_binCW.resize(PIC_CODE_CW_BINS, 0);
  }
  if (m_inputPivot.empty())
  {
    m_inputPivot.resize(PIC_CODE_CW_BINS + 1, 0);
  }
  if (m_fwdScaleCoef.empty())
  {
    m_fwdScaleCoef.resize(PIC_CODE_CW_BINS, 1 << FP_PREC);
  }
  if (m_invScaleCoef.empty())
  {
    m_invScaleCoef.resize(PIC_CODE_CW_BINS, 1 << FP_PREC);
  }
  if (m_reshapePivot.empty())
  {
    m_reshapePivot.resize(PIC_CODE_CW_BINS + 1, 0);
  }
  if (m_chromaAdjHelpLUT.empty())
  {
    m_chromaAdjHelpLUT.resize(PIC_CODE_CW_BINS, 1 << CSCALE_FP_PREC);
  }
}

void Reshape::destroy() {}

/** compute chroma residuce scale for TU
 * \param average luma pred of TU
 * \return chroma residue scale
 */
int Reshape::calculateChromaAdj(Pel avgLuma)
{
  int iAdj = m_chromaAdjHelpLUT[getPWLIdxInv(avgLuma)];
  return (iAdj);
}

/** compute chroma residuce scale for TU
 * \param average luma pred of TU
 * \return chroma residue scale
 */
int Reshape::calculateChromaAdjVpduNei(TransformUnit &tu, const CompArea &areaY)
{
  CodingStructure &cs               = *tu.cs;
  int              xPos             = areaY.lumaPos().x;
  int              yPos             = areaY.lumaPos().y;
  int              numNeighborAbove = areaY.width, numNeighborLeft = areaY.height;

  Position          topLeft(xPos, yPos);
  CodingUnit       *topLeftLuma;
  const CodingUnit *cuAbove, *cuLeft;
  if (CS::isDualITree(cs) && cs.slice->m_eSliceType == I_SLICE)
  {
    topLeftLuma = tu.cs->picture->m_cs->getCU(topLeft, ChannelType::LUMA);
    cuAbove     = cs.picture->m_cs->getCURestricted(topLeft.offset(0, -1), *topLeftLuma, ChannelType::LUMA);
    cuLeft      = cs.picture->m_cs->getCURestricted(topLeft.offset(-1, 0), *topLeftLuma, ChannelType::LUMA);
  }
  else
  {
    topLeftLuma = cs.getCU(topLeft, ChannelType::LUMA);
    cuAbove     = cs.getCURestricted(topLeft.offset(0, -1), *topLeftLuma, ChannelType::LUMA);
    cuLeft      = cs.getCURestricted(topLeft.offset(-1, 0), *topLeftLuma, ChannelType::LUMA);
  }

  CompArea  lumaArea    = CompArea(CompID::COMP_Y, tu.chromaFormat, topLeft, areaY, true);
  PelBuf    piRecoY     = cs.picture->getRecoBuf(lumaArea);
  ptrdiff_t strideY     = piRecoY.stride;
  int       chromaScale = (1 << CSCALE_FP_PREC);
  int       lumaValue   = -1;

  Pel           *recSrc0 = piRecoY.bufAt(0, 0);
  const uint32_t picH    = tu.cs->picture->lheight();
  const uint32_t picW    = tu.cs->picture->lwidth();
  const Pel      valueDC = 1 << (tu.cs->sps->m_bitDepths[ChannelType::LUMA] - 1);
  int32_t        recLuma = 0;
  int            pelnum  = 0;
  if (cuLeft != nullptr)
  {
    for (int i = 0; i < numNeighborLeft; i++)
    {
      int k = (yPos + i) >= picH ? (picH - yPos - 1) : i;
      recLuma += recSrc0[-1 + k * strideY];
      pelnum++;
    }
  }
  if (cuAbove != nullptr)
  {
    for (int i = 0; i < numNeighborAbove; i++)
    {
      int k = (xPos + i) >= picW ? (picW - xPos - 1) : i;
      recLuma += recSrc0[-strideY + k];
      pelnum++;
    }
  }
  if (pelnum)
  {
    lumaValue = ClipPel((recLuma + (pelnum >> 1)) / pelnum, tu.cs->slice->clpRng(CompID::COMP_Y));
  }
  else
  {
    CHECK(pelnum != 0, "");
    lumaValue = valueDC;
  }
  chromaScale = calculateChromaAdj(lumaValue);
  return (chromaScale);
}
/** find inx of PWL for inverse mapping
 * \param average luma pred of TU
 * \return idx of PWL for inverse mapping
 */
int Reshape::getPWLIdxInv(int lumaVal)
{
  int idxS = 0;
  for (idxS = m_sliceReshapeInfo.reshaperModelMinBinIdx; (idxS <= m_sliceReshapeInfo.reshaperModelMaxBinIdx); idxS++)
  {
    if (lumaVal < m_reshapePivot[idxS + 1])
    {
      break;
    }
  }
  return std::min(idxS, PIC_CODE_CW_BINS - 1);
}

/** Construct reshaper from syntax
 * \param void
 * \return void
 */
void Reshape::constructReshaper()
{
  int pwlFwdLUTsize = PIC_CODE_CW_BINS;
  int pwlFwdBinLen  = m_reshapeLUTSize / PIC_CODE_CW_BINS;

  for (int i = 0; i < m_sliceReshapeInfo.reshaperModelMinBinIdx; i++)
  {
    m_binCW[i] = 0;
  }
  for (int i = m_sliceReshapeInfo.reshaperModelMaxBinIdx + 1; i < PIC_CODE_CW_BINS; i++)
  {
    m_binCW[i] = 0;
  }
  for (int i = m_sliceReshapeInfo.reshaperModelMinBinIdx; i <= m_sliceReshapeInfo.reshaperModelMaxBinIdx; i++)
  {
    m_binCW[i] = (uint16_t)(m_sliceReshapeInfo.reshaperModelBinCWDelta[i] + (int)m_initCW);
  }

  for (int i = 0; i < pwlFwdLUTsize; i++)
  {
    m_reshapePivot[i + 1] = m_reshapePivot[i] + m_binCW[i];
    m_inputPivot[i + 1]   = m_inputPivot[i] + m_initCW;
    m_fwdScaleCoef[i] =
      ((int32_t)m_binCW[i] * (1 << FP_PREC) + (1 << (floorLog2(pwlFwdBinLen) - 1))) >> floorLog2(pwlFwdBinLen);
    if (m_binCW[i] == 0)
    {
      m_invScaleCoef[i]     = 0;
      m_chromaAdjHelpLUT[i] = 1 << CSCALE_FP_PREC;
    }
    else
    {
      m_invScaleCoef[i] = (int32_t)(m_initCW * (1 << FP_PREC) / m_binCW[i]);
      m_chromaAdjHelpLUT[i] =
        (int32_t)(m_initCW * (1 << FP_PREC) / (m_binCW[i] + m_sliceReshapeInfo.chrResScalingOffset));
    }
  }

  int sumBinCW = 0;
  for (int i = m_sliceReshapeInfo.reshaperModelMinBinIdx; i <= m_sliceReshapeInfo.reshaperModelMaxBinIdx; i++)
  {
    sumBinCW += m_binCW[i];
    if (m_binCW[i] != 0)
    {
      CHECK((m_binCW[i] + m_sliceReshapeInfo.chrResScalingOffset) < (m_initCW >> 3) ||
              (m_binCW[i] + m_sliceReshapeInfo.chrResScalingOffset) > ((m_initCW << 3) - 1),
            "It is a requirement of bitstream conformance that, when lmcsCW[ i ] is not equal to 0, ( lmcsCW[ i ] + "
            "lmcsDeltaCrs ) shall be in the range of ( OrgCW >> 3 ) to ( ( OrgCW << 3 ) - 1 ), inclusive.");
    }
    CHECK(m_binCW[i] < (m_initCW >> 3) || m_binCW[i] > ((m_initCW << 3) - 1),
          " lmcsCW[ i ] shall be in the range of ( OrgCW >> 3 ) to ( ( OrgCW << 3 ) - 1 if not equal to 0 ).");
    CHECK((((m_reshapePivot[i] % (1 << (m_lumaBD - 5))) != 0) &&
           ((m_reshapePivot[i] >> (m_lumaBD - 5)) == (m_reshapePivot[i + 1] >> (m_lumaBD - 5)))),
          "It is a requirement of bitstream conformance that, for i = lmcs_min_bin_idx..LmcsMaxBinIdx, when the value "
          "of LmcsPivot[ i ] is not a multiple of 1 << ( BitDepth - 5 ), the value of(LmcsPivot[i] >> (BitDepth - 5)) "
          "shall not be equal to the value of(LmcsPivot[i + 1] >> (BitDepth - 5)).");
  }
  CHECK(sumBinCW > ((1 << m_lumaBD) - 1),
        "It is a requirement of bitstream conformance that the following condition is true: Sum_(i = 0) ^ 15 "
        "[lmcsCW[i]] <= (1 << BitDepth) - 1.");

  for (int lumaSample = 0; lumaSample < m_reshapeLUTSize; lumaSample++)
  {
    int idxY    = lumaSample / m_initCW;
    int tempVal = m_reshapePivot[idxY] +
      ((m_fwdScaleCoef[idxY] * (lumaSample - m_inputPivot[idxY]) + (1 << (FP_PREC - 1))) >> FP_PREC);
    m_fwdLUT[lumaSample] = Clip3((Pel)0, (Pel)((1 << m_lumaBD) - 1), (Pel)(tempVal));

    int idxYInv   = getPWLIdxInv(lumaSample);
    int invSample = m_inputPivot[idxYInv] +
      ((m_invScaleCoef[idxYInv] * (lumaSample - m_reshapePivot[idxYInv]) + (1 << (FP_PREC - 1))) >> FP_PREC);
    m_invLUT[lumaSample] = Clip3((Pel)0, (Pel)((1 << m_lumaBD) - 1), (Pel)(invSample));
  }
}

//
//! \}
