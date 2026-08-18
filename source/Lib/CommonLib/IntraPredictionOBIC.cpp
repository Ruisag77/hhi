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

#include "IntraPrediction.h"

#include "UnitTools.h"

void IntraPrediction::deriveObicMode(const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu)
{
  /* -------------------------------------------------------------------
  Step 1: Build Histogram of oCcurrence (HoC) from remaining neighbours
  Step 2: Get top 6 amplitudes from the HoC
  Step 3: Compute the fusion weights from amplitudes and store in CU
  ---------------------------------------------------------------------- */
  auto &obicData   = cu.obicData;
  obicData.isBlend = false;
  for (int i = 0; i < OBIC_FUSION_NUM; i++)
  {
    obicData.blendMode[i] = PLANAR_IDX;
    obicData.relWeight[i] = 0;
  }

  /* -----------------------------------------------------------------
  --------------------------- Step 1: --------------------------------
  ---------------- Build Histogram of oCcurrence (HoC) ---------------
  --------------------- from remaining neighbours --------------------
  ----------------------------------------------------------------- */
  int histogram[NUM_LUMA_MODE];
  int histoLocDep[NUM_LUMA_MODE][3];
  for (int i = 0; i < NUM_LUMA_MODE; i++)
  {
    histogram[i] = 0;
    for (int j = 0; j < 3; j++)
    {
      histoLocDep[i][j] = 0;
    }
  }
  auto &cuNeighbours = cu.obicNeighbours;
  for (int i = 0; i < NUM_OBIC_CUS; i++)
  {
    if (!cuNeighbours[i])
    {
      continue;
    }

    const int weight[NUM_OBIC_CUS] = { 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 2, 2, 2,
                                       2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
    const int numSamples =
      (weight[i] * (cuNeighbours[i]->lumaSize().width * cuNeighbours[i]->lumaSize().height) + 2) / 4;
    if (cuNeighbours[i]->timdFlag)
    {
      const auto &neighbourTimd = cuNeighbours[i]->ofTimdFlag    ? cuNeighbours[i]->ofTimdData
                                  : cuNeighbours[i]->timdSadFlag ? cuNeighbours[i]->timdSadData
                                                                 : cuNeighbours[i]->timdData;
      int   m             = MAP131TO67(neighbourTimd.blendMode[0]);
      int   w[3]          = { 0 };
      w[0]                = neighbourTimd.relWeight[0];
      w[1] = (neighbourTimd.isBlend && (neighbourTimd.relWeight[1] > 0)) ? neighbourTimd.relWeight[1] : 0;
      w[2] = (neighbourTimd.isBlend && (neighbourTimd.relWeight[2] > 0)) ? neighbourTimd.relWeight[2] : 0;
      histoLocDep[m][neighbourTimd.locDep[0]] += numSamples;

      if (neighbourTimd.isBlend && neighbourTimd.relWeight[1] > 0 &&
          (neighbourTimd.blendMode[0] != neighbourTimd.blendMode[1]))
      {
        int m = MAP131TO67(neighbourTimd.blendMode[1]);
        CHECK(!w[0], "Division by zero!");
        histoLocDep[m][neighbourTimd.locDep[1]] += numSamples * w[1] / w[0];
        if (neighbourTimd.relWeight[2] > 0)
        {
          int m = MAP131TO67(neighbourTimd.blendMode[2]);
          histoLocDep[m][neighbourTimd.locDep[2]] += numSamples * w[2] / w[0];
        }
      }
    }
    else if (cuNeighbours[i]->dimdFlag && !cuNeighbours[i]->obicFlag)
    {
      auto &neighbourDimd      = cuNeighbours[i]->dimdData;
      int   m                  = neighbourDimd.blendMode[0];
      int   w[DIMD_FUSION_NUM] = { 0 };
      w[0]                     = neighbourDimd.relWeight[0];
      w[1]                     = (neighbourDimd.relWeight[1] > 0) ? neighbourDimd.relWeight[1] : 0;
      for (int j = 2; j < DIMD_FUSION_NUM; j++)
      {
        w[j] = (neighbourDimd.isBlend && (neighbourDimd.relWeight[j] > 0)) ? neighbourDimd.relWeight[j] : 0;
      }
      if (m >= 0 && m < NUM_LUMA_MODE)
      {
        histoLocDep[m][neighbourDimd.locDep[1]] += numSamples; // todo: check locdep: 1 or 0?
      }
      if (neighbourDimd.relWeight[1] > 0)
      {
        CHECK(!w[0], "Division by zero!");
        histoLocDep[0][0] += numSamples * w[1] / w[0];
      }
      if (neighbourDimd.isBlend)
      {
        for (int idx = 2; idx < DIMD_FUSION_NUM; idx++)
        {
          m = neighbourDimd.blendMode[idx];
          if (m >= 0 && m < NUM_LUMA_MODE && neighbourDimd.relWeight[idx] > 0)
          {
            CHECK(!w[0], "Division by zero!");
            histoLocDep[m][neighbourDimd.locDep[idx]] += numSamples * w[idx] / w[0];
          }
        }
      }
    }
    else if (cuNeighbours[i]->obicFlag)
    {
      auto &neighbourObic = cuNeighbours[i]->obicData;
      int   m             = neighbourObic.blendMode[0];
      histoLocDep[m][neighbourObic.locDep[0]] += numSamples;
      if (neighbourObic.isBlend)
      {
        for (int idx = 1; idx < OBIC_FUSION_NUM; idx++)
        {
          m = neighbourObic.blendMode[idx];
          if (m >= 0 && neighbourObic.relWeight[idx] > 0)
          {
            histoLocDep[m][neighbourObic.locDep[idx]] += numSamples;
          }
        }
      }
    }
    else if (cuNeighbours[i]->sgpm)
    {
      int m1 = cuNeighbours[i]->sgpmMode0;
      int m2 = cuNeighbours[i]->sgpmMode1;
      if (m1 >= 0 && m1 < NUM_LUMA_MODE)
      {
        histoLocDep[m1][0] += numSamples;
      }
      if (m2 >= 0 && m2 < NUM_LUMA_MODE)
      {
        histoLocDep[m2][0] += numSamples;
      }
    }
    else if (cuNeighbours[i]->eipFlag && cu.slice->m_eSliceType != I_SLICE)
    {
      int m = cuNeighbours[i]->inferredDimdMode;
      histoLocDep[m][0] += numSamples;
    }
    else if (CU::isIntra(*cuNeighbours[i]) /*&& !cuNeighbours[i]->tmpFlag*/ && !cuNeighbours[i]->mipFlag &&
             !cuNeighbours[i]->eipFlag && !CU::isIBC(*cuNeighbours[i]) && !CU::isPLT(*cuNeighbours[i]))
    {
      int m = cuNeighbours[i]->intraDir[ChannelType::LUMA];
      histoLocDep[m][0] += numSamples;
    }
    else if (cuNeighbours[i] && (CU::isInter(*cuNeighbours[i])))
    {
      if (cuNeighbours[i]->geoFlag)
      {
        int ipm = g_geoAngle2IntraAng[g_geoParams[cuNeighbours[i]->geoSplitDir].angleIdx];
        if (ipm > PLANAR_IDX && ipm < NUM_LUMA_MODE)
        {
          histoLocDep[ipm][0] += numSamples;
        }
      }
    }
  }
  for (int i = 0; i < NUM_LUMA_MODE; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      histogram[i] += histoLocDep[i][j];
    }
  }
  // Penalize Dimd modes to impose diversity between OBIC and DIMD
  if (cu.dimdData.blendMode[0] >= 0 && cu.dimdData.blendMode[0] < NUM_LUMA_MODE)
  {
    histogram[cu.dimdData.blendMode[0]] >>= 1;
  }
  if (cu.dimdData.isBlend && cu.dimdData.blendMode[1] >= 0 && cu.dimdData.blendMode[1] < NUM_LUMA_MODE &&
      cu.dimdData.relWeight[1] > 0)
  {
    histogram[cu.dimdData.blendMode[1]] >>= 1;
  }

/* -----------------------------------------------------------------
    ------------------- Step 2: Get top 6 amplitudes -------------------
    -------------------------- from the HoC ----------------------------
    ----------------------------------------------------------------- */

  int bestModes[OBIC_FUSION_NUM], bestAmps[OBIC_FUSION_NUM];
  for (int i = 0; i < OBIC_FUSION_NUM; i++)
  {
    bestModes[i] = -1;
    bestAmps[i]  = 0;
  }
  for (int i = 0; i < NUM_LUMA_MODE; i++)
  {
    int curMode = i;
    if (curMode == PLANAR_IDX)
    {
      continue;
    }
    if (histogram[curMode] > bestAmps[0])
    {
      bestAmps[5]  = bestAmps[4];
      bestAmps[4]  = bestAmps[3];
      bestAmps[3]  = bestAmps[2];
      bestAmps[2]  = bestAmps[1];
      bestAmps[1]  = bestAmps[0];
      bestAmps[0]  = histogram[curMode];
      bestModes[5] = bestModes[4];
      bestModes[4] = bestModes[3];
      bestModes[3] = bestModes[2];
      bestModes[2] = bestModes[1];
      bestModes[1] = bestModes[0];
      bestModes[0] = curMode;
    }
    else if (histogram[curMode] > bestAmps[1])
    {
      bestAmps[5]  = bestAmps[4];
      bestAmps[4]  = bestAmps[3];
      bestAmps[3]  = bestAmps[2];
      bestAmps[2]  = bestAmps[1];
      bestAmps[1]  = histogram[curMode];
      bestModes[5] = bestModes[4];
      bestModes[4] = bestModes[3];
      bestModes[3] = bestModes[2];
      bestModes[2] = bestModes[1];
      bestModes[1] = curMode;
    }
    else if (histogram[curMode] > bestAmps[2])
    {
      bestAmps[5]  = bestAmps[4];
      bestAmps[4]  = bestAmps[3];
      bestAmps[3]  = bestAmps[2];
      bestAmps[2]  = histogram[curMode];
      bestModes[5] = bestModes[4];
      bestModes[4] = bestModes[3];
      bestModes[3] = bestModes[2];
      bestModes[2] = curMode;
    }
    else if (histogram[curMode] > bestAmps[3])
    {
      bestAmps[5]  = bestAmps[4];
      bestAmps[4]  = bestAmps[3];
      bestAmps[3]  = histogram[curMode];
      bestModes[5] = bestModes[4];
      bestModes[4] = bestModes[3];
      bestModes[3] = curMode;
    }
    else if (histogram[curMode] > bestAmps[4])
    {
      bestAmps[5]  = bestAmps[4];
      bestAmps[4]  = histogram[curMode];
      bestModes[5] = bestModes[4];
      bestModes[4] = curMode;
    }
    else if (histogram[curMode] > bestAmps[5])
    {
      bestAmps[5]  = histogram[curMode];
      bestModes[5] = curMode;
    }
  }
  if (bestModes[0] < 0)
  {
    return;
  }

  int count = 0;
  for (int i = 0; i < OBIC_FUSION_NUM; i++)
  {
    count += bestModes[i] >= 0 ? 1 : 0;
  }

// For each selected IPM in the generation of OBIC predictor, associate the dominant LocDep
  for (int i = 0; i < OBIC_FUSION_NUM; i++)
  {
    int secondMode = bestModes[i];
    int bestLocDep[3];
    int bestLocDepAmp[3];
    for (int i = 0; i < 3; i++)
    {
      bestLocDep[i]    = -1;
      bestLocDepAmp[i] = 0;
    }
    obicData.locDep[i] = 0;

    if (secondMode > DC_IDX)
    {
      for (int j = 0; j < 3; j++)
      {
        if (histoLocDep[secondMode][j] > bestLocDepAmp[0])
        {
          bestLocDepAmp[2] = bestLocDepAmp[1];
          bestLocDepAmp[1] = bestLocDepAmp[0];
          bestLocDepAmp[0] = histoLocDep[secondMode][j];
          bestLocDep[2]    = bestLocDep[1];
          bestLocDep[1]    = bestLocDep[0];
          bestLocDep[0]    = j;
        }
        else if (histoLocDep[secondMode][j] > bestLocDepAmp[1])
        {
          bestLocDepAmp[2] = bestLocDepAmp[1];
          bestLocDepAmp[1] = histoLocDep[secondMode][j];
          bestLocDep[2]    = bestLocDep[1];
          bestLocDep[1]    = j;
        }
        else if (histoLocDep[secondMode][j] > bestLocDepAmp[2])
        {
          bestLocDepAmp[2] = histoLocDep[secondMode][j];
          bestLocDep[2]    = j;
        }
      }
      obicData.locDep[i] = bestLocDep[0];
    }
  }
  /* -----------------------------------------------------------------
  -------------- Step 3: Compute the fusion weights ------------------
  ---------------- from amplitudes and store in CU -------------------
  ----------------------------------------------------------------- */

  int planarWeight    = count == 1 ? 21 : 64 / 4;
  int log2BlendWeight = 6;
  int sumWeight       = (1 << log2BlendWeight);
  if (bestModes[1] < 0)
  {
    obicData.blendMode[0] = bestModes[0];
    obicData.isBlend      = false;
    obicData.relWeight[0] = sumWeight;
    for (int i = 1; i < OBIC_FUSION_NUM; i++)
    {
      obicData.blendMode[i] = -1;
      obicData.relWeight[i] = 0;
    }
    obicData.isBlend      = true;
    obicData.blendMode[1] = PLANAR_IDX;
    obicData.relWeight[0] = sumWeight - planarWeight;
    obicData.relWeight[1] = planarWeight;
    obicData.locDep[1]    = 0;
  }
  else
  {
    sumWeight = sumWeight - planarWeight;
    if (count == OBIC_FUSION_NUM)
    {
      bestAmps[count - 1] = 0;
    }

    int s1 = 0;
    for (int i = 0; i < OBIC_FUSION_NUM; i++)
    {
      bestAmps[i] = 10 * bestAmps[i];
    }
    for (int i = 0; i < OBIC_FUSION_NUM; i++)
    {
      s1 = s1 + bestAmps[i];
    }
    int x = floorLog2(s1);
    CHECK(x < 0, "floor log2 value should be no negative");
    int normS1 = (s1 << 4 >> x) & 15;
    int v      = g_gradDivTable[normS1] | 8;
    x += (normS1 != 0);
    int shift                   = x + 3;
    int add                     = (1 << (shift - 1));
    int iRatio[OBIC_FUSION_NUM] = { 0 };
    for (int i = 0; i < OBIC_FUSION_NUM; i++)
    {
      iRatio[i] =
        (int)((((uint64_t)bestAmps[i] * (uint64_t)v * (uint64_t)sumWeight + (uint64_t)add)) >> (uint64_t)shift);
      if (bestAmps[i] == 0)
      {
        iRatio[i] = 0;
      }
      if (iRatio[i] > sumWeight)
      {
        iRatio[i] = sumWeight;
      }
      CHECK(iRatio[i] > sumWeight, "Wrong ratio in OBIC");
    }
    int sumTmp = 0;
    for (int i = 0; i < OBIC_FUSION_NUM; i++)
    {
      sumTmp += iRatio[i];
      obicData.blendMode[i] = bestModes[i];
      obicData.relWeight[i] = iRatio[i];
    }
    if (sumTmp != sumWeight)
    {
      int d = sumTmp - sumWeight;
      if (d > 0)
      {
        for (int i = OBIC_FUSION_NUM - 1; i >= 0; i--)
        {
          int diff = d;
          for (int k = 0; k < diff; k++)
          {
            if (obicData.relWeight[i] > 0)
            {
              obicData.relWeight[i] -= 1;
              d -= 1;
            }
          }
        }
      }
      else
      {
        for (int i = 0; i < OBIC_FUSION_NUM; i++) // i < OBIC_FUSION_NUM && d != 0
        {
          int diff = d;
          if (obicData.relWeight[i] > 0 && (obicData.relWeight[i] + d) < sumWeight)
          {
            for (int k = 0; k < abs(diff); k++)
            {
              if (d == 0)
              {
                break;
              }
              if (obicData.relWeight[i] > 0)
              {
                obicData.relWeight[i] += 1;
                d += 1;
              }
            }
          }
        }
      }
    }
    sumTmp = 0;
    for (int i = 0; i < OBIC_FUSION_NUM; i++)
    {
      sumTmp += obicData.relWeight[i];
    }
    CHECKD(sumTmp != sumWeight, "Wrong sum!");
    if (count == OBIC_FUSION_NUM)
    {
      obicData.blendMode[count - 1] = PLANAR_IDX;
      obicData.relWeight[count - 1] = planarWeight;
      obicData.locDep[count - 1]    = 0;
    }
    else
    {
      obicData.blendMode[count] = PLANAR_IDX;
      obicData.relWeight[count] = planarWeight;
      obicData.locDep[count]    = 0;
    }
    obicData.isBlend = true;
  }
}

// OBIC prediction
void IntraPrediction::predIntraObic(PelBuf &piPred, CodingUnit &cu, const CompArea &area, bool skipDerivation)
{
  const auto &obicData = cu.obicData;
  if (!skipDerivation)
  {
    deriveObicMode(cu.cs->picture->getRecoBuf(area), area, cu);
    cu.derivedIpm[0] = obicData.blendMode[0]; // todo: check
    cu.derivedIpm[1] = obicData.isBlend ? obicData.blendMode[1] : cu.derivedIpm[0];
  }
  PROFILER_SCOPE(1, g_timeProfiler, P_INTRA_EST_CAND_LUMA_OBIC);

  const CodingStructure &cs     = *cu.cs;
  int                    width  = piPred.width;
  int                    height = piPred.height;
  const UnitArea         localUnitArea(cu.chromaFormat, Area(0, 0, width, height));
  bool                   blendModes[OBIC_FUSION_NUM - 1] = { false };
  PelBuf                 predFusion[OBIC_FUSION_NUM - 1];
  const bool             applyPdpc = m_ipaParam.applyPDPC;
  cu.intraDir[ChannelType::LUMA]   = obicData.blendMode[0];
  initIntraPatternChType(cu, area, true);
  // Do First Prediction
  initPredIntraParams(cu, area, *cs.sps);
  predIntraAng(COMP_Y, piPred, cu, true, false);
  if (!obicData.isBlend)
  {
    return;
  }

  for (int i = 0; i < OBIC_FUSION_NUM - 1; i++)
  {
    blendModes[i] = false;
    predFusion[i] = m_tempBufferDIMD[i].getBuf(localUnitArea.Y());
    if (obicData.blendMode[i + 1] >= 0)
    {
      blendModes[i]                  = true;
      cu.intraDir[ChannelType::LUMA] = obicData.blendMode[i + 1];
      initPredIntraParams(cu, area, *cs.sps);
      predIntraAng(COMP_Y, predFusion[i], cu, true, false);
    }
  }

  cu.intraDir[ChannelType::LUMA] = obicData.blendMode[0];
  m_ipaParam.applyPDPC           = applyPdpc;

  PelBuf predAngNonLocDep = m_tempBufferDIMD[7].getBuf(localUnitArea.Y());
  PelBuf predAngVer       = m_tempBufferDIMD[5].getBuf(localUnitArea.Y());
  PelBuf predAngHor       = m_tempBufferDIMD[6].getBuf(localUnitArea.Y());

  Pel      *pelVer          = predAngVer.buf;
  ptrdiff_t strideVer       = predAngVer.stride;
  Pel      *pelHor          = predAngHor.buf;
  ptrdiff_t strideHor       = predAngHor.stride;
  Pel      *pelNonLocDep    = predAngNonLocDep.buf;
  ptrdiff_t strideNonLocDep = predAngNonLocDep.stride;

  bool useLocDepBlending = false;
  int  weightVer = 0, weightHor = 0, weightNonLocDep = 0;

  for (int i = 0; i < OBIC_FUSION_NUM; i++)
  {
    if (i == 0 || blendModes[i - 1])
    {
      if (obicData.locDep[i] == 1)
      {
        weightVer += obicData.relWeight[i];
      }
      else if (obicData.locDep[i] == 2)
      {
        weightHor += obicData.relWeight[i];
      }
      else
      {
        weightNonLocDep += obicData.relWeight[i];
      }
    }
  }

  const int log2WeightSum = 6;
  const int weightSum     = 1 << log2WeightSum;
  if ((weightHor & (weightSum - 1)) || (weightVer & (weightSum - 1))) // either weight is != 0 or 64
  {
    useLocDepBlending = true;
  }

  if (!useLocDepBlending)
  {
    pelNonLocDep    = piPred.buf;
    strideNonLocDep = piPred.stride;
  }
  for (int locDep = 0; locDep < 3; locDep++)
  {
    int totWeight = (locDep == 0 ? weightNonLocDep : (locDep == 1 ? weightVer : weightHor));
    if (totWeight == 0)
    {
      continue;
    }

    int weights[OBIC_FUSION_NUM] = { 0 };
    weights[0]                   = (obicData.locDep[0] == locDep) ? obicData.relWeight[0] : 0;
    for (int i = 1; i < OBIC_FUSION_NUM; i++)
    {
      weights[i] = (blendModes[i - 1] && obicData.locDep[i] == locDep) ? obicData.relWeight[i] : 0;
    }

    int num2blend                     = 0;
    int blendIndexes[OBIC_FUSION_NUM] = { 0 };
    for (int i = 0; i < OBIC_FUSION_NUM; i++)
    {
      if (weights[i] != 0)
      {
        blendIndexes[num2blend] = i;
        num2blend++;
      }
    }

    bool weightIsPowerOf2 = (totWeight & (totWeight - 1)) == 0;
    if ((num2blend == 1) || (num2blend <= 3 && weightIsPowerOf2))
    {
      int index = blendIndexes[0];
      if (locDep == 0)
      {
        pelNonLocDep    = (index == 0 ? piPred.buf : predFusion[index - 1].buf);
        strideNonLocDep = (index == 0 ? piPred.stride : predFusion[index - 1].stride);
      }
      else if (locDep == 1)
      {
        pelVer    = (index == 0 ? piPred.buf : predFusion[index - 1].buf);
        strideVer = (index == 0 ? piPred.stride : predFusion[index - 1].stride);
      }
      else
      {
        pelHor    = (index == 0 ? piPred.buf : predFusion[index - 1].buf);
        strideHor = (index == 0 ? piPred.stride : predFusion[index - 1].stride);
      }
      Pel      *pCur      = (locDep == 0 ? pelNonLocDep : (locDep == 1 ? pelVer : pelHor));
      ptrdiff_t strideCur = (locDep == 0 ? strideNonLocDep : (locDep == 1 ? strideVer : strideHor));

      int factor = weightSum / totWeight;
      if (num2blend == 2)
      {
        int       index1  = blendIndexes[1];
        Pel      *p1      = (index1 == 0 ? piPred.buf : predFusion[index1 - 1].buf);
        ptrdiff_t stride1 = (index1 == 0 ? piPred.stride : predFusion[index1 - 1].stride);

        int w0 = (weights[index] * factor);
        int w1 = weightSum - w0;
        for (int y = 0; y < height; y++)
        {
          for (int x = 0; x < width; x++)
          {
            int blend = pCur[x] * w0;
            blend += p1[x] * w1;
            pCur[x] = (Pel)(blend >> log2WeightSum);
          }
          pCur += strideCur;
          p1 += stride1;
        }
      }
      else if (num2blend == 3)
      {
        int       index1  = blendIndexes[1];
        Pel      *p1      = (index1 == 0 ? piPred.buf : predFusion[index1 - 1].buf);
        ptrdiff_t stride1 = (index1 == 0 ? piPred.stride : predFusion[index1 - 1].stride);

        int       index2  = blendIndexes[2];
        Pel      *p2      = (index2 == 0 ? piPred.buf : predFusion[index2 - 1].buf);
        ptrdiff_t stride2 = (index2 == 0 ? piPred.stride : predFusion[index2 - 1].stride);

        int w0 = (weights[index] * factor);
        int w1 = (weights[index1] * factor);
        int w2 = weightSum - w0 - w1;
        for (int y = 0; y < height; y++)
        {
          for (int x = 0; x < width; x++)
          {
            int blend = pCur[x] * w0;
            blend += p1[x] * w1;
            blend += p2[x] * w2;
            pCur[x] = (Pel)(blend >> log2WeightSum);
          }

          pCur += strideCur;
          p1 += stride1;
          p2 += stride2;
        }
      }
    }
    else
    {
      Pel      *pCur      = (locDep == 0 ? pelNonLocDep : (locDep == 1 ? pelVer : pelHor));
      ptrdiff_t strideCur = (locDep == 0 ? strideNonLocDep : (locDep == 1 ? strideVer : strideHor));

      Pel *pelPred = piPred.buf;
      Pel *pelFusion[OBIC_FUSION_NUM - 1];

      for (int i = 0; i < OBIC_FUSION_NUM - 1; i++)
      {
        pelFusion[i] = predFusion[i].buf;
      }

      auto [scale, round, shift] = lutDivideGetScaleRoundShift(totWeight, INTRA_FUSION_BITS);
      for (int y = 0; y < height; y++)
      {
        for (int x = 0; x < width; x++)
        {
          int blend = pelPred[x] * weights[0];
          for (int i = 0; i < OBIC_FUSION_NUM - 1; i++)
          {
            blend += blendModes[i] ? pelFusion[i][x] * weights[i + 1] : 0;
          }
          pCur[x] = (Pel)((uint64_t(blend) * scale + round) >> (INTRA_FUSION_BITS + shift));
        }
        pCur += strideCur;
        pelPred += piPred.stride;
        for (int i = 0; i < OBIC_FUSION_NUM - 1; i++)
        {
          pelFusion[i] += predFusion[i].stride;
        }
      }
    }
  }

  if (useLocDepBlending)
  {
    int       mode      = ((weightHor > 0 && weightVer > 0) ? 0 : (weightVer > 0 ? 1 : 2));
    Pel      *pelDst    = piPred.buf;
    ptrdiff_t strideDst = piPred.stride;
    locDepBlending(pelDst, strideDst, pelVer, strideVer, pelHor, strideHor, pelNonLocDep, strideNonLocDep, width,
                   height, mode, weightVer, weightHor, weightNonLocDep);
  }
}
