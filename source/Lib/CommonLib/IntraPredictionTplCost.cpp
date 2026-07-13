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

/** \file     Prediction.cpp
    \brief    prediction class
*/

#include "IntraPrediction.h"

#include "Unit.h"
#include "UnitTools.h"
#include "Buffer.h"

#include "dtrace_next.h"
#include "Rom.h"

#include <memory.h>

#include "CommonLib/InterpolationFilter.h"
//! \ingroup CommonLib
//! \{

void IntraPrediction::findDecoderDerivedCcpModel(CodingUnit &cu, CrossCompModels &ccModel0, CrossCompModels &ccModel1)
{
  CompArea &areaCb = cu.blocks[COMP_Cb];
  CompArea &areaCr = cu.blocks[COMP_Cr];

  CclmModel       cclmModelCb, cclmModelCr;
  CrossCompModels ccModels[6] = {};

  ConvModelType candModelType[4]  = { CONV_MODEL_CCLM, CONV_MODEL_CCCM, CONV_MODEL_CCCM, CONV_MODEL_CCCM_GRADLOC };
  int           candIntraMode[4]  = { LM_CHROMA_IDX, LM_CHROMA_IDX, MMLM_CHROMA_IDX, LM_CHROMA_IDX };
  bool          candTestFilter[4] = { false, true, true, false };

  const int width     = areaCb.width;
  const int height    = areaCb.height;
  const int sizeThr   = (cu.slice->m_eSliceType == I_SLICE) ? 32 : 8;
  const int numModels = width * height > sizeThr ? 4 : 3;

  std::vector<int> candCost;

  int candInd     = 0;
  int costBest    = -1;
  int candIndBest = 0;

  // Non-blended candidates
  for (int modelIndex = 0; modelIndex < numModels; modelIndex++)
  {
    const int numFilterCands = candTestFilter[modelIndex] ? 2 : 1;

    for (int filterIndex = 0; filterIndex < numFilterCands; filterIndex++)
    {
      CrossCompModels &ccModelCand = cu.ccModels;

      // For a filtered candidate using the already created models
      if (filterIndex)
      {
        ccModelCand        = ccModels[candInd - 1];
        ccModelCand.filter = true;
      }
      else
      {
        cu.intraDir[ChannelType::CHROMA] = candIntraMode[modelIndex];
        cu.cccmFlag                      = candModelType[modelIndex] == CONV_MODEL_CCLM ? false : true;
        cu.cccmType                      = candModelType[modelIndex];
        cu.decDerivedCcpMode             = true;

        if (cu.cccmFlag)
        {
          ccModelCand.init(PU::isMultiModeLM(cu.intraDir[ChannelType::CHROMA]));
          ccModelCand.modelCb[0] = ConvModel(cu.cccmType, cu.cs->sps->m_bitDepths[ChannelType::LUMA]);
          ccModelCand.modelCr[0] = ConvModel(cu.cccmType, cu.cs->sps->m_bitDepths[ChannelType::LUMA]);
          ccModelCand.lumaOffset = m_cccmLumaOffset;
          ccModelCand.filter     = false;

          if (!ccModelCand.dualModel)
          {
            if (cu.cccmType == CONV_MODEL_CCCM)
            {
              xCccmCalcModels(cu, ccModelCand.modelCb[0], ccModelCand.modelCr[0], 0, 0, 2);
            }
            else
            {
              xCccmCalcModels(cu, ccModelCand.modelCb[0], ccModelCand.modelCr[0], 0, 0);
            }
          }
          else
          {
            ccModelCand.modelCb[1] = ConvModel(cu.cccmType, cu.cs->sps->m_bitDepths[ChannelType::LUMA]);
            ccModelCand.modelCr[1] = ConvModel(cu.cccmType, cu.cs->sps->m_bitDepths[ChannelType::LUMA]);
            ccModelCand.lumaThres  = xCccmCalcRefAver(cu);

            xCccmCalcModels(cu, ccModelCand.modelCb[0], ccModelCand.modelCr[0], 1, ccModelCand.lumaThres);
            xCccmCalcModels(cu, ccModelCand.modelCb[1], ccModelCand.modelCr[1], 2, ccModelCand.lumaThres);
          }
        }
        else
        {
          xGetLMParameters_LMS(cu, COMP_Cb, areaCb, cclmModelCb);
          xGetLMParameters_LMS(cu, COMP_Cr, areaCr, cclmModelCr);

          ccModelCand.setCclmModel(cclmModelCb, COMP_Cb, PU::isMultiModeLM(cu.intraDir[ChannelType::CHROMA]));
          ccModelCand.setCclmModel(cclmModelCr, COMP_Cr, PU::isMultiModeLM(cu.intraDir[ChannelType::CHROMA]));
        }
      }

      // Template cost
      int cost = xGetOneCCPCandCost(cu, ccModelCand, candInd, ccModelCand.filter);

      candCost.push_back(cost);

      ccModels[candInd] = cu.ccModels;

      if (cost < costBest || costBest < 0)
      {
        candIndBest = candInd;
        costBest    = cost;
      }

      candInd++;
    }
  }

  ccModel0       = ccModels[candIndBest];
  ccModel1.valid = false;

  int numCands = candInd;

  // Blended candidates
  if (costBest > (width + height) >> 1 && numCands > 1)
  {
    int bestIdx1       = 0;
    int bestIdx2       = 1;
    int bestFusionCost = MAX_INT;

    for (int i = 0; i < numCands - 1; i++)
    {
      for (int j = i + 1; j < numCands; j++)
      {
        int cost = xDecDerivedFusionTemplateCost(cu, i, j, candCost[i], candCost[j]);

        if (cost < bestFusionCost)
        {
          bestIdx1       = i;
          bestIdx2       = j;
          bestFusionCost = cost;
        }
      }
    }

    if (bestFusionCost < costBest)
    {
      if (candCost[bestIdx2] < candCost[bestIdx1])
      {
        ccModel0 = ccModels[bestIdx2];
        ccModel1 = ccModels[bestIdx1];
      }
      else
      {
        ccModel0 = ccModels[bestIdx1];
        ccModel1 = ccModels[bestIdx2];
      }
    }
  }
}

int IntraPrediction::xDecDerivedFusionTemplateCost(const CodingUnit &cu, int candIdxOrig0, int candIdxOrig1, int cost0,
                                                   int cost1)
{
  int candIdx0 = cost0 < cost1 ? candIdxOrig0 : candIdxOrig1;
  int candIdx1 = cost0 < cost1 ? candIdxOrig1 : candIdxOrig0;
  int fusionWeight;
  return xGetCCPFusionTemplateCost(cu, candIdx0, candIdx1, fusionWeight, 48);
}

int IntraPrediction::xGetCCPFusionTemplateCost(const CodingUnit &cu, int candIdx0, int candIdx1, int &fusionWeight,
                                               int defaultWeight)
{
  const bool aboveAvailable = cu.cs->getCU(cu.blocks[COMP_Cb].pos().offset(0, -1), ChannelType::CHROMA) != nullptr;
  const bool leftAvailable  = cu.cs->getCU(cu.blocks[COMP_Cb].pos().offset(-1, 0), ChannelType::CHROMA) != nullptr;

  int64_t  numerator   = 0;
  uint64_t denominator = 0;

  auto accumulateModelStats = [&](const Pel org, const Pel pred0, const Pel pred1)
  {
    const int64_t predDiff = int64_t(pred0) - pred1;
    numerator += (int64_t(org) - pred1) * predDiff;
    denominator += uint64_t(predDiff * predDiff);
  };

  for (const CompID compId: { COMP_Cb, COMP_Cr })
  {
    const CompArea compArea = cu.blocks[compId];
    const int      width    = compArea.width;
    const int      height   = compArea.height;
    const PelBuf   reco     = cu.cs->picture->getRecoBuf(compArea);

    if (aboveAvailable)
    {
      const PelBuf pred0 = xGetCcTemplPredBuf(compId, candIdx0, true, compArea);
      const PelBuf pred1 = xGetCcTemplPredBuf(compId, candIdx1, true, compArea);
      for (int x = 0; x < width; x++)
      {
        accumulateModelStats(reco.at(x, -1), pred0.at(x, 0), pred1.at(x, 0));
      }
    }

    if (leftAvailable)
    {
      const PelBuf pred0 = xGetCcTemplPredBuf(compId, candIdx0, false, compArea);
      const PelBuf pred1 = xGetCcTemplPredBuf(compId, candIdx1, false, compArea);
      for (int y = 0; y < height; y++)
      {
        accumulateModelStats(reco.at(-1, y), pred0.at(0, y), pred1.at(0, y));
      }
    }
  }

  fusionWeight = defaultWeight;
  if (denominator != 0)
  {
    fusionWeight = numerator <= 0 ? 0
      : uint64_t(numerator) >= denominator ? 64
                                           : int((uint64_t(numerator) * 64 + (denominator >> 1)) / denominator);
  }

  int cost = 0;
  for (const CompID compId: { COMP_Cb, COMP_Cr })
  {
    const CompArea compArea = cu.blocks[compId];
    const int      width    = compArea.width;
    const int      height   = compArea.height;
    const PelBuf   reco     = cu.cs->picture->getRecoBuf(compArea);

    if (aboveAvailable)
    {
      const PelBuf pred0 = xGetCcTemplPredBuf(compId, candIdx0, true, compArea);
      const PelBuf pred1 = xGetCcTemplPredBuf(compId, candIdx1, true, compArea);
      for (int x = 0; x < width; x++)
      {
        const Pel pred = Pel((fusionWeight * pred0.at(x, 0) + (64 - fusionWeight) * pred1.at(x, 0) + 32) >> 6);
        cost += abs(reco.at(x, -1) - pred);
      }
    }

    if (leftAvailable)
    {
      const PelBuf pred0 = xGetCcTemplPredBuf(compId, candIdx0, false, compArea);
      const PelBuf pred1 = xGetCcTemplPredBuf(compId, candIdx1, false, compArea);
      for (int y = 0; y < height; y++)
      {
        const Pel pred = Pel((fusionWeight * pred0.at(0, y) + (64 - fusionWeight) * pred1.at(0, y) + 32) >> 6);
        cost += abs(reco.at(-1, y) - pred);
      }
    }
  }

  return cost;
}

void IntraPrediction::setAndPredCcMergeCand(CodingUnit &cu, CrossCompModels &ccpCand, PelBuf &predCb, PelBuf &predCr)
{
  cu.ccModels = ccpCand;
  cu.cccmType = cu.ccModels.modelCb[0].modelType;
  cu.cccmFlag = cu.cccmType >= CONV_MODEL_CCCM_INTRA_FIRST && cu.cccmType <= CONV_MODEL_CCCM_INTRA_LAST ? 1 : 0;

  cu.intraDir[ChannelType::CHROMA] = cu.ccModels.dualModel ? MMLM_CHROMA_IDX : LM_CHROMA_IDX;

  if (cu.cccmFlag)
  {
    predIntraCCCM(cu, predCb, predCr, false);
  }
  else
  {
    predIntraChromaLM(COMP_Cb, predCb, cu, cu.Cb(), cu.intraDir[ChannelType::CHROMA], false);
    predIntraChromaLM(COMP_Cr, predCr, cu, cu.Cr(), cu.intraDir[ChannelType::CHROMA], false);
  }
}

void IntraPrediction::setAndPredCcMergeFusionCand(CodingUnit &cu, CrossCompModels model0, CrossCompModels model1,
                                                  PelBuf &predCb, PelBuf &predCr)
{
  const int      width  = predCb.width;
  const int      height = predCb.height;
  const UnitArea localUnitArea(cu.chromaFormat, Area(0, 0, width, height));

  PelBuf predCb1 = m_tempBufferDIMD[0].getBuf(localUnitArea.Y());
  PelBuf predCr1 = m_tempBufferDIMD[1].getBuf(localUnitArea.Y());

  if (cu.decDerivedCcpMode && !model1.valid)
  {
    // Non-blended decoder derived cross-component model
    setAndPredCcMergeCand(cu, model0, predCb, predCr);

    return;
  }

  xGetOneCCPCandCost(cu, model0, 0, model0.filter);
  xGetOneCCPCandCost(cu, model1, 1, model1.filter);
  int fusionWeight;
  xGetCCPFusionTemplateCost(cu, 0, 1, fusionWeight, cu.decDerivedCcpMode ? 48 : 32);

  setAndPredCcMergeCand(cu, model1, predCb1, predCr1);
  setAndPredCcMergeCand(cu, model0, predCb, predCr);

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      predCb.at(x, y) = Pel((fusionWeight * predCb.at(x, y) + (64 - fusionWeight) * predCb1.at(x, y) + 32) >> 6);
      predCr.at(x, y) = Pel((fusionWeight * predCr.at(x, y) + (64 - fusionWeight) * predCr1.at(x, y) + 32) >> 6);
    }
  }
}

void IntraPrediction::reorderCCPCandidates(CodingUnit &cu, CrossCompModels candList[], int reorderlistSize,
                                           int *fusionList)
{
  int candCost[MAX_CCP_CAND_LIST_SIZE];
  int ccpCandIndex[MAX_CCP_CAND_LIST_SIZE];

  for (int i = 0; i < reorderlistSize; i++)
  {
    candCost[i]     = xGetOneCCPCandCost(cu, candList[i], i);
    ccpCandIndex[i] = i;
  }

  // Inserting sorting
  for (int i = 1; i < reorderlistSize; i++)
  {
    for (int j = 0; j < i; j++)
    {
      if (candCost[i] < candCost[j])
      {
        CrossCompModels tmpCand   = candList[i];
        int             tmpCost   = candCost[i];
        int             tempIndex = ccpCandIndex[i];

        for (int k = i; k > j; k--)
        {
          candList[k]     = candList[k - 1];
          candCost[k]     = candCost[k - 1];
          ccpCandIndex[k] = ccpCandIndex[k - 1];
        }
        candList[j]     = tmpCand;
        candCost[j]     = tmpCost;
        ccpCandIndex[j] = tempIndex;
        break;
      }
    }
  }

  if (fusionList && cu.cs->slice->m_sps->m_ccMergeFusion)
  {
    int fusionCandCost[MAX_CCP_FUSION_NUM] = { MAX_INT };
    int maxCandIdxForCcpFusion             = std::min(reorderlistSize, 9);

    for (int i = 0; i < maxCandIdxForCcpFusion - 1; i++)
    {
      for (int j = i + 1; j < maxCandIdxForCcpFusion; j++)
      {
        int fusionWeight;
        int sad = xGetCCPFusionTemplateCost(cu, ccpCandIndex[i], ccpCandIndex[j], fusionWeight, 32);
        for (int m = 0; m < MAX_CCP_FUSION_NUM; m++)
        {
          if (sad < fusionCandCost[m])
          {
            for (int n = MAX_CCP_FUSION_NUM - 2; n >= m; n--)
            {
              const int nextIdx           = n + 1;
              fusionCandCost[nextIdx]     = fusionCandCost[n];
              fusionList[2 * nextIdx]     = fusionList[2 * n];
              fusionList[2 * nextIdx + 1] = fusionList[2 * n + 1];
            }
            fusionCandCost[m]     = sad;
            fusionList[2 * m]     = i;
            fusionList[2 * m + 1] = j;

            break;
          }
        }
      }
    }
  }
}

void IntraPrediction::xDecDerivedCcpFusionTemplFilter(Pel *predBuf, int numSamples)
{
  // Todo: ECM does 2D filtering here
  static Pel srcBuffer[MAX_CU_SIZE + 2];
  Pel       *srcBuf = srcBuffer + 1; // For padding

  for (int i = 0; i < numSamples; i++)
  {
    srcBuf[i] = predBuf[i];
  }

  srcBuf[-1]         = srcBuf[0];
  srcBuf[numSamples] = srcBuf[numSamples - 1];

  for (int i = 0; i < numSamples; i++)
  {
    predBuf[i] = (2 * srcBuf[i] + srcBuf[i - 1] + srcBuf[i + 1] + 2) >> 2;
  }
}

int IntraPrediction::xGetOneCCPCandCost(CodingUnit &cu, CrossCompModels &ccpCand, const int candIdx, const bool filter)
{
  int cost = 0;

  if (ccpCand.modelCb[0].modelType == CONV_MODEL_CCLM)
  {
    CclmModel cclmModelCb;
    CclmModel cclmModelCr;

    ccpCand.getCclmModel(cclmModelCb, COMP_Cb);
    ccpCand.getCclmModel(cclmModelCr, COMP_Cr);

    cost += xTemplateCostCCLM(cu, COMP_Cb, cu.Cb(), cclmModelCb, 1, candIdx);
    cost += xTemplateCostCCLM(cu, COMP_Cr, cu.Cr(), cclmModelCr, 1, candIdx);
  }
  else if (ccpCand.modelCb[0].modelType >= CONV_MODEL_CCCM_INTRA_FIRST &&
           ccpCand.modelCb[0].modelType <= CONV_MODEL_CCCM_INTRA_LAST)
  {
    const bool aboveAvailable =
      cu.cs->getCU(cu.blocks[COMP_Cb].pos().offset(0, -1), ChannelType::CHROMA) ? true : false;
    const bool leftAvailable = cu.cs->getCU(cu.blocks[COMP_Cb].pos().offset(-1, 0), ChannelType::CHROMA) ? true : false;

    PelBuf predCbTop  = xGetCcTemplPredBuf(COMP_Cb, candIdx, true, cu.Cb());
    PelBuf predCbLeft = xGetCcTemplPredBuf(COMP_Cb, candIdx, false, cu.Cb());
    PelBuf predCrTop  = xGetCcTemplPredBuf(COMP_Cr, candIdx, true, cu.Cr());
    PelBuf predCrLeft = xGetCcTemplPredBuf(COMP_Cr, candIdx, false, cu.Cr());

    PelBuf recoCb = cu.cs->picture->getRecoBuf(cu.Cb());
    PelBuf recoCr = cu.cs->picture->getRecoBuf(cu.Cr());

    const int width  = cu.Cb().width;
    const int height = cu.Cb().height;

    if (aboveAvailable)
    {
      xCccmApplyModels(cu, ccpCand, predCbTop, predCrTop, true, true);

      Pel *cbTop = predCbTop.buf;
      Pel *crTop = predCrTop.buf;

      if (filter)
      {
        xDecDerivedCcpFusionTemplFilter(cbTop, width);
        xDecDerivedCcpFusionTemplFilter(crTop, width);
      }

      for (int x = 0; x < width; x++)
      {
        cost += abs(recoCb.at(x, -1) - cbTop[x]);
        cost += abs(recoCr.at(x, -1) - crTop[x]);
      }
    }

    if (leftAvailable)
    {
      xCccmApplyModels(cu, ccpCand, predCbLeft, predCrLeft, true, false);

      Pel *cbLeft = predCbLeft.buf;
      Pel *crLeft = predCrLeft.buf;

      if (filter)
      {
        xDecDerivedCcpFusionTemplFilter(cbLeft, height);
        xDecDerivedCcpFusionTemplFilter(crLeft, height);
      }

      for (int y = 0; y < height; y++)
      {
        cost += abs(recoCb.at(-1, y) - cbLeft[y]);
        cost += abs(recoCr.at(-1, y) - crLeft[y]);
      }
    }
  }
  else
  {
    CHECK(true, "Unknown mode");
  }

  return cost;
}

PelBuf IntraPrediction::xGetCcTemplPredBuf(const CompID compID, const int candIdx, const bool top, const Area &area)
{
  int  offset = top ? 0 : area.width;
  int  stride = top ? area.width : 1;
  Size size   = top ? Size(area.width, 1) : Size(1, area.height);

  return PelBuf((compID == COMP_Cb ? m_ccTemplPredCb[candIdx] : m_ccTemplPredCr[candIdx]) + offset, stride, size);
}

// ECM: xUpdateOffsetsAndGetCostCCLM
int IntraPrediction::xTemplateCostCCLM(const CodingUnit &cu, const CompID compID, const CompArea &chromaArea,
                                       const CclmModel &cclmModel, const int modelNum, const int candIdx)
{
  int    sad       = 0;
  int    srcStride = 2 * MAX_CU_SIZE + 1;
  PelBuf temp      = PelBuf(m_piTemp + srcStride + 1, srcStride, Size(chromaArea));

  /*
  if (glmIdc > 0)
  {
    Pel *glmTemp = m_glmTempCb[glmIdc];
    temp         = PelBuf(glmTemp + srcStride + 1, srcStride, Size(chromaArea));
  }
  */

  const SizeType cWidth  = chromaArea.width;
  const SizeType cHeight = chromaArea.height;

  CodingStructure &cs = *(cu.cs);

  const bool aboveAvailable = cs.getCU(cu.blocks[compID].pos().offset(0, -1), ChannelType::CHROMA) ? true : false;
  const bool leftAvailable  = cs.getCU(cu.blocks[compID].pos().offset(-1, 0), ChannelType::CHROMA) ? true : false;

  Pel *srcColor0, *curChroma0;

  srcColor0 = temp.bufAt(0, 0);

  PelBuf    chromaReco   = cs.picture->getRecoBuf(chromaArea);
  Pel      *curChromaBuf = chromaReco.buf;
  const int curStride    = int(chromaReco.stride);

  PelBuf predTop  = xGetCcTemplPredBuf(compID, candIdx, true, chromaArea);
  PelBuf predLeft = xGetCcTemplPredBuf(compID, candIdx, false, chromaArea);

  if (aboveAvailable)
  {
    curChroma0 = curChromaBuf - curStride;
    Pel *src   = srcColor0 - srcStride;
    Pel  predChroma;
    for (int pos = 0; pos < cWidth; pos++)
    {
      if (modelNum == 2 && src[pos] > cclmModel.multiModelLumaThr)
      {
        predChroma = rightShift(cclmModel.model[1].a * src[pos], cclmModel.model[1].shift) + cclmModel.model[1].b;
      }
      else
      {
        predChroma = rightShift(cclmModel.model[0].a * src[pos], cclmModel.model[0].shift) + cclmModel.model[0].b;
      }
      predTop.at(pos, 0) = predChroma;
      sad += abs(curChroma0[pos] - predChroma);
    }
  }

  if (leftAvailable)
  {
    curChroma0 = curChromaBuf - 1;
    Pel *src   = srcColor0 - 1;

    Pel predChroma;
    for (int pos = 0; pos < cHeight; pos++)
    {
      if (modelNum == 2 && src[pos * srcStride] > cclmModel.multiModelLumaThr)
      {
        predChroma =
          rightShift(cclmModel.model[1].a * src[pos * srcStride], cclmModel.model[1].shift) + cclmModel.model[1].b;
      }
      else
      {
        predChroma =
          rightShift(cclmModel.model[0].a * src[pos * srcStride], cclmModel.model[0].shift) + cclmModel.model[0].b;
      }
      predLeft.at(0, pos) = predChroma;
      sad += abs(curChroma0[pos * curStride] - predChroma);
    }
  }

  return sad;
}

//! \}
