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

#include "UnitTools.h"

// EIP prediction
Position IntraPrediction::getEipRecoPosition(const CodingUnit &cu, const CompID compId) const
{
  const int numRecoLines = std::min(cu.blocks[compId].width, cu.blocks[compId].height);
  const int above        = std::min(cu.blocks[compId].y - EIP_FILTER_SIZE, numRecoLines);
  const int left         = std::min(cu.blocks[compId].x - EIP_FILTER_SIZE, numRecoLines);

  CHECK(above < 1 || left < 1, "no reconstruction lines above or left.");

  return Position(above, left);
}

Pel IntraPrediction::getEipInputsAvg(Pel *inputs, ConvModelType filterShape) const
{
  if (filterShape == CONV_MODEL_EIP_S)
  {
    return (inputs[0] + inputs[3] + 1) >> 1;
  }
  else
  {
    return inputs[1];
  }
}

void IntraPrediction::setEipInputVector(PelBuf &reco, int w, int h, int filterShape, Pel *inputs)
{
  inputs[EIP_FILTER_TAP - 1] = m_eipBias;

  for (int idx = 0; idx < EIP_FILTER_TAP - 1; idx++)
  {
    inputs[idx] = reco.at(w + g_eipFilter[filterShape][idx].x, h + g_eipFilter[filterShape][idx].y);
  }
}

int IntraPrediction::getCurEipCands(const CodingUnit &cu, static_vector<EipModels, NUM_DERIVED_EIP> &candList,
                                    const CompID compId)
{
  if (!CU::eipAllowed(cu, compId))
  {
    return 0;
  }

  const Position                          tplSize   = getEipRecoPosition(cu, compId);
  const int                               refSizeY  = m_eipBlkArea.y;
  const int                               refSizeX  = m_eipBlkArea.x;
  const int                               refHeight = m_eipBlkArea.height;
  const int                               refWidth  = m_eipBlkArea.width;
  PelBuf                                  refBuf(m_eipBuffer, refWidth, refHeight);
  const int                               numInputs = EIP_FILTER_TAP;
  static_vector<EipInfo, NUM_DERIVED_EIP> eipInfoList;

  CU::eipCurAllowed(cu, compId, eipInfoList, false);
  const int numNonMultiModel       = int(eipInfoList.size());
  int       remainingNonMultiModel = numNonMultiModel;
  CU::eipCurAllowed(cu, compId, eipInfoList,
                    true); // eventually completes eipInfoList with multi-model candidates if relevant

  if (!cu.cs->pcv->isEncoder) // decoder derives only one set of coefficients.
  {
    int candIdx = cu.intraDir[ChannelType::LUMA];
    if (cu.eipMultiModel)
    {
      candIdx += numNonMultiModel;
      remainingNonMultiModel = 0;
    }

    const auto eipInfoBackup = eipInfoList[candIdx];
    eipInfoList.clear();
    eipInfoList.push_back(eipInfoBackup);
  }

  for (auto mode: eipInfoList)
  {
    EipInfo::REFERENCE_TYPE recoType    = mode.recoType;
    ConvModelType           filterShape = mode.filterShape;
    const int               bd          = cu.slice->m_sps->m_bitDepths[toChannelType(compId)];
    const int  filterIdx    = filterShape - CONV_MODEL_EIP_FIRST;   //  CONV_MODEL_EIP_S => 0, CONV_MODEL_EIP_V => 1 ...
    const bool isMultiModel = remainingNonMultiModel <= 0;
    EipModels  models(filterShape, bd, isMultiModel);
    remainingNonMultiModel -= 1;

    int numSamples0 = 0;
    int numSamples1 = 0;

    auto fillSamples = [&](int startX, int startY, int endX, int endY, bool multiModel)
    {
      if (multiModel == true)
      {
        for (int y = startY; y < endY; y++)
        {
          for (int x = startX; x < endX; x++)
          {
            int ref;
            // compute reference for classification
            if (filterShape == CONV_MODEL_EIP_S)
            {
              const int ref1 = refBuf.at(x + g_eipFilter[filterIdx][0].x, y + g_eipFilter[filterIdx][0].y);
              const int ref2 = refBuf.at(x + g_eipFilter[filterIdx][3].x, y + g_eipFilter[filterIdx][3].y);
              ref            = (ref1 + ref2 + 1) >> 1;
            }
            else
            {
              ref = refBuf.at(x + g_eipFilter[filterIdx][1].x, y + g_eipFilter[filterIdx][1].y);
            }

            // classify
            const int thrd      = m_eipAvg;
            int       modelIdx  = (ref <= thrd) ? 0 : 1;
            int      &sampleIdx = (ref <= thrd) ? numSamples0 : numSamples1;

            for (int inputIdx = 0; inputIdx < numInputs - 1; inputIdx++)
            {
              m_eipRefMatrixA[modelIdx][inputIdx * strideEipRefMatrixA + sampleIdx] =
                refBuf.at(x + g_eipFilter[filterIdx][inputIdx].x, y + g_eipFilter[filterIdx][inputIdx].y);
            }
            m_eipRefMatrixA[modelIdx][(numInputs - 1) * strideEipRefMatrixA + sampleIdx] = m_eipBias;

            m_eipTargetVectorY[modelIdx][sampleIdx] = refBuf.at(x, y);

            sampleIdx++;
          }
        }
      }
      else
      {
        for (int y = startY; y < endY; y++)
        {
          for (int x = startX; x < endX; x++)
          {
            for (int inputIdx = 0; inputIdx < numInputs - 1; inputIdx++)
            {
              m_eipRefMatrixA[0][inputIdx * strideEipRefMatrixA + numSamples0] =
                refBuf.at(x + g_eipFilter[filterIdx][inputIdx].x, y + g_eipFilter[filterIdx][inputIdx].y);
            }
            m_eipRefMatrixA[0][(numInputs - 1) * strideEipRefMatrixA + numSamples0] = m_eipBias;

            m_eipTargetVectorY[0][numSamples0] = refBuf.at(x, y);

            numSamples0++;
          }
        }
      }
    };

    auto recoIncludes = [&](EipInfo::REFERENCE_TYPE refType)
    { return (to_underlying(recoType) & to_underlying(refType)) == to_underlying(refType); };

    if (recoIncludes(EipInfo::REFERENCE_TYPE::AL_A))
    {
      fillSamples(refSizeX - tplSize.x, refSizeY - tplSize.y, refWidth, refSizeY, isMultiModel);
    }
    else if (recoIncludes(EipInfo::REFERENCE_TYPE::AL))
    {
      fillSamples(refSizeX - tplSize.x, refSizeY - tplSize.y, refSizeX, refSizeY, isMultiModel);
    }
    else if (recoIncludes(EipInfo::REFERENCE_TYPE::A))
    {
      fillSamples(refSizeX, refSizeY - tplSize.y, refWidth, refSizeY, isMultiModel);
    }

    if (recoIncludes(EipInfo::REFERENCE_TYPE::L))
    {
      fillSamples(refSizeX - tplSize.x, refSizeY, refSizeX, refHeight, isMultiModel);
    }

    auto getRegularizationParam = [](int numSamples, bool isMultiModel) -> int
    {
      if (isMultiModel)
      {
        if (numSamples <= EIP_L2_MM_REGULARIZATION_SAMPLE_THRESHOLD[0])
        {
          return EIP_L2_MM_REGULARIZATION[0] * EIP_FILTER_TAP;
        }
        else if (numSamples <= EIP_L2_MM_REGULARIZATION_SAMPLE_THRESHOLD[1])
        {
          return EIP_L2_MM_REGULARIZATION[1] * EIP_FILTER_TAP;
        }
        else if (numSamples <= EIP_L2_MM_REGULARIZATION_SAMPLE_THRESHOLD[2])
        {
          return EIP_L2_MM_REGULARIZATION[2] * EIP_FILTER_TAP;
        }
        else if (numSamples <= EIP_L2_MM_REGULARIZATION_SAMPLE_THRESHOLD[3])
        {
          return EIP_L2_MM_REGULARIZATION[3] * EIP_FILTER_TAP;
        }
        else if (numSamples <= EIP_L2_MM_REGULARIZATION_SAMPLE_THRESHOLD[4])
        {
          return EIP_L2_MM_REGULARIZATION[4] * EIP_FILTER_TAP;
        }
        else
        {
          return EIP_L2_MM_REGULARIZATION[5] * EIP_FILTER_TAP;
        }
      }
      else
      {
        return (numSamples <= EIP_L2_REGULARIZATION_SAMPLE_THRESHOLD) ? EIP_L2_REGULARIZATION_SMALL * EIP_FILTER_TAP
                                                                      : EIP_L2_REGULARIZATION_LARGE * EIP_FILTER_TAP;
      }
    };

    const int regularizationParam = getRegularizationParam(numSamples0, isMultiModel);
    m_cccmSolver.solve(*this, numSamples0, m_eipRefMatrixA[0], strideEipRefMatrixA, regularizationParam,
                       m_eipTargetVectorY[0], m_eipLumaOffset, &models.model[0]);

    if (isMultiModel == true)
    {
      models.threshold              = m_eipAvg;
      const int regularizationParam = getRegularizationParam(numSamples1, isMultiModel);
      m_cccmSolver.solve(*this, numSamples1, m_eipRefMatrixA[1], strideEipRefMatrixA, regularizationParam,
                         m_eipTargetVectorY[1], m_eipLumaOffset, &models.model[1]);
    }

    candList.push_back(models);
  }

  return numNonMultiModel;
}

void IntraPrediction::getNeighborsEipCands(const CodingUnit &cu, static_vector<EipModels, MAX_MERGE_EIP> &candList,
                                           const CompID compId)
{
  if (!CU::eipMergeAllowed(cu, compId))
  {
    return;
  }

  const CompArea        &area        = cu.blocks[compId];
  const ChannelType      channelType = toChannelType(compId);
  int                    numCand     = 0;
  const int              maxCands    = MAX_MERGE_EIP;
  const CodingStructure &cs          = *cu.cs;
  const Position         topLeft     = area.topLeft();
  const Position         posCand[5]  = {
    topLeft.offset(-1, area.height - 1), topLeft.offset(area.width - 1, -1), topLeft.offset(-1, -1),
    topLeft.offset(area.width, -1),      topLeft.offset(-1, area.height),
  };

  auto getEipModel = [&](Position pos) -> void
  {
    const CodingUnit *neighborCu = cs.getCURestricted(pos, cu, channelType);
    if (neighborCu && neighborCu->eipFlag)
    {
      EipModels cand           = neighborCu->eipModels;
      bool      bIncludedModel = false;
      for (auto i = 0; i < candList.size(); i++)
      {
        if (cand == candList[i])
        {
          bIncludedModel = true;
          break;
        }
      }
      if (!bIncludedModel)
      {
        candList.push_back(cand);
        numCand++;
      }
    }
    if (numCand >= maxCands)
    {
      return;
    }
  };

  for (int posIdx = 0; posIdx < 5; posIdx++)
  {
    getEipModel(posCand[posIdx]);
  }

  auto tryToAddOneModel = [&](const EipModels &cand)
  {
    bool bIncludedModel = false;
    for (auto i = 0; i < candList.size(); i++)
    {
      if (cand == candList[i])
      {
        bIncludedModel = true;
        break;
      }
    }
    if (!bIncludedModel)
    {
      candList.push_back(cand);
      numCand++;
    }
    if (numCand >= maxCands)
    {
      return;
    }
  };

  const Slice         &slice      = *cu.slice;
  const PreCalcValues &pcv        = *cs.pcv;
  const SubPic        &curSubPic  = slice.m_pps->getSubPicFromPos(cu.lumaPos());
  int                  lumaScaleX = getChannelTypeScaleX(channelType, cu.chromaFormat);
  int                  lumaScaleY = getChannelTypeScaleY(channelType, cu.chromaFormat);

  auto checkBoundaryCondition = [&curSubPic, &pcv, &lumaScaleX, &lumaScaleY](int x, int y) -> bool
  {
    bool boundaryCond;
    if (curSubPic.m_treatedAsPicFlag)
    {
      boundaryCond = (x <= (curSubPic.m_subPicRight >> lumaScaleX)) && (y <= (curSubPic.m_subPicBottom >> lumaScaleY));
    }
    else
    {
      boundaryCond = (x < (pcv.lumaWidth >> lumaScaleX)) && (y < (pcv.lumaHeight >> lumaScaleY));
    }
    return boundaryCond;
  };

  if (!slice.isIntra() && compId == COMP_Y)
  {
    const Picture *const pColPic =
      slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.m_colFromL0Flag : 0), slice.m_colRefIdx);
    bool isRefScaled = pColPic->isRefScaled(slice.m_pps);
    if (pColPic && !isRefScaled)
    {
      bool c0Avail;
      bool c1Avail;

      Position posRB     = area.bottomRight().offset(-1, -1);
      Position posCenter = area.center();
      Position posC0;
      Position posC1;

      int offsetX0 = 0, offsetX1 = 0, offsetX2 = 0, offsetX3 = area.width >> 1;
      int offsetY0 = 0, offsetY1 = 0, offsetY2 = 0, offsetY3 = area.height >> 1;

      const int numNACandidate[5] = { 2, 2, 2, 2, 2 };
      const int idxMap[5][2]      = { { 0, 1 }, { 0, 2 }, { 0, 2 }, { 0, 2 }, { 0, 2 } };

      for (int iDistanceIndex = 0; iDistanceIndex < 5 && numCand < maxCands; iDistanceIndex++)
      {
        const int iNADistanceHor = area.width * iDistanceIndex;
        const int iNADistanceVer = area.height * iDistanceIndex;

        for (int naspIdx = 0; naspIdx < numNACandidate[iDistanceIndex] && numCand < maxCands; naspIdx++)
        {
          switch (idxMap[iDistanceIndex][naspIdx])
          {
          case 0:
            offsetX0 = offsetX2 = 2 + iNADistanceHor;
            offsetY0 = offsetY2 = 2 + iNADistanceVer;
            offsetX1            = iNADistanceHor;
            offsetY1            = iNADistanceVer;
            break;
          case 1:
            offsetX0 = 2;
            offsetY0 = 0;
            offsetX1 = 0;
            offsetY1 = 2;
            break;
          case 2:
            offsetX0 = offsetX2;
            offsetY0 = 2 - offsetY3;
            offsetX1 = 2 - offsetX3;
            offsetY1 = offsetY2;
            break;
          default:
            THROW("error!");
            break;
          }

          c0Avail                   = false;
          const bool c0BoundaryCond = checkBoundaryCondition(posRB.x + offsetX0, posRB.y + offsetY0);
          if (c0BoundaryCond)
          {
            posC0   = posRB.offset(offsetX0, offsetY0);
            c0Avail = true;
          }

          if (idxMap[iDistanceIndex][naspIdx] == 0)
          {
            c1Avail                   = false;
            const bool c1BoundaryCond = checkBoundaryCondition(posCenter.x + offsetX1, posCenter.y + offsetY1);
            if (c1BoundaryCond)
            {
              posC1   = posCenter.offset(offsetX1, offsetY1);
              c1Avail = true;
            }
          }
          else
          {
            c1Avail                   = false;
            const bool c1BoundaryCond = checkBoundaryCondition(posRB.x + offsetX1, posRB.y + offsetY1);
            if (c1BoundaryCond)
            {
              posC1   = posRB.offset(offsetX1, offsetY1);
              c1Avail = true;
            }
          }

          if (c0Avail || c1Avail)
          {
            int modelIdx = c0Avail ? pColPic->m_cs->getEipIdxInfo(posC0) : pColPic->m_cs->getEipIdxInfo(posC1);
            if (modelIdx > 0)
            {
              const EipModels currEipModel = pColPic->m_cs->m_eipModelLUT[modelIdx - 1];
              tryToAddOneModel(currEipModel);
            }
          }
        }
      }
    }
  }

  // Non-adjacent candidates round 1
  int       offsetX  = 0;
  int       offsetY  = 0;
  int       offsetX0 = 0;
  int       offsetX1 = 0;
  int       offsetY0 = 0;
  int       offsetY1 = 0;
  const int offsetX2 = area.width >> 1;
  const int offsetY2 = area.height >> 1;

  const int horNAInterval     = std::max<int>((area.width * 2) >> 1, 4);
  const int verNAInterval     = std::max<int>((area.height * 2) >> 1, 4);
  const int numNACandidate[7] = { 5, 9, 9, 9, 9, 9, 9 };
  const int idxMap[7][9]      = { { 0, 1, 2, 3, 4 },
                                  { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
                                  { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
                                  { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
                                  { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
                                  { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
                                  { 0, 1, 2, 3, 4, 5, 6, 7, 8 } };

  for (int iDistanceIndex = 0; iDistanceIndex < 7 && numCand < maxCands; iDistanceIndex++)
  {
    const int iNADistanceHor = horNAInterval * (iDistanceIndex + 1);
    const int iNADistanceVer = verNAInterval * (iDistanceIndex + 1);

    for (int naspIdx = 0; naspIdx < numNACandidate[iDistanceIndex] && numCand < maxCands; naspIdx++)
    {
      switch (idxMap[iDistanceIndex][naspIdx])
      {
      case 0:
        offsetX = offsetX0 = -iNADistanceHor - 1;
        offsetY = offsetY0 = verNAInterval + iNADistanceVer - 1;
        break;
      case 1:
        offsetX = offsetX1 = horNAInterval + iNADistanceHor - 1;
        offsetY = offsetY1 = -iNADistanceVer - 1;
        break;
      case 2:
        offsetX = offsetX2;
        offsetY = offsetY1;
        break;
      case 3:
        offsetX = offsetX0;
        offsetY = offsetY2;
        break;
      case 4:
        offsetX = offsetX0;
        offsetY = offsetY1;
        break;
      case 5:
        offsetX = -1;
        offsetY = offsetY0;
        break;
      case 6:
        offsetX = offsetX1;
        offsetY = -1;
        break;
      case 7:
        offsetX = offsetX0 >> 1;
        offsetY = offsetY0;
        break;
      case 8:
        offsetX = offsetX1;
        offsetY = offsetY1 >> 1;
        break;
      default:
        THROW("error!");
        break;
      }

      getEipModel(topLeft.offset(offsetX, offsetY));
    }
  }

  // Non-adjacent candidates round 2
  const int numNACandidate2[7] = { 4, 4, 4, 4, 4, 4, 4 };
  const int idxMap2[7][5]      = { { 0, 1, 2, 3 }, { 0, 1, 2, 3 }, { 0, 1, 2, 3 }, { 0, 1, 2, 3 },
                                   { 0, 1, 2, 3 }, { 0, 1, 2, 3 }, { 0, 1, 2, 3 } };

  for (int iDistanceIndex = 0; iDistanceIndex < 7 && numCand < maxCands; iDistanceIndex++)
  {
    const int horNADistance = horNAInterval * (iDistanceIndex + 1);
    const int verNADistance = verNAInterval * (iDistanceIndex + 1);

    for (int naspIdx = 0; naspIdx < numNACandidate2[iDistanceIndex] && numCand < maxCands; naspIdx++)
    {
      switch (idxMap2[iDistanceIndex][naspIdx])
      {
      case 0:
        offsetX = offsetX0 = -horNADistance - 1;
        offsetY            = offsetY2 + ((verNAInterval + verNADistance - 1 - offsetY2) >> 1);
        break;
      case 1:
        offsetX = offsetX2 + ((horNAInterval + horNADistance - 1 - offsetX2) >> 1);
        offsetY = offsetY0 = -verNADistance - 1;
        break;
      case 2:
        offsetX = offsetX0;
        offsetY = offsetY0 + ((offsetY2 - offsetY0) >> 1);
        break;
      case 3:
        offsetX = offsetX0 + ((offsetX2 - offsetX0) >> 1);
        offsetY = offsetY0;
        break;
      default:
        THROW("error!");
        break;
      }

      getEipModel(topLeft.offset(offsetX, offsetY));
    }
  }

  // Shifted temporal eip models
  if (!slice.isIntra() && compId == COMP_Y)
  {
    const Picture *const pColPic =
      slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.m_colFromL0Flag : 0), slice.m_colRefIdx);
    bool isRefScaled = pColPic->isRefScaled(slice.m_pps);

    if (pColPic && !isRefScaled)
    {
      bool     c0Avail;
      bool     c1Avail;
      Position posRB     = area.bottomRight().offset(-1, -1);
      Position posCenter = area.center();

      Position posC0;
      Position posC1;

      int offsetX0 = 0, offsetX1 = 0, offsetX2 = 0, offsetX3 = area.width >> 1;
      int offsetY0 = 0, offsetY1 = 0, offsetY2 = 0, offsetY3 = area.height >> 1;

      const int numNACandidate[5] = { 2, 2, 2, 2, 2 };
      const int idxMap[5][2]      = { { 0, 1 }, { 0, 2 }, { 0, 2 }, { 0, 2 }, { 0, 2 } };

      const unsigned plevel = cu.cs->sps->m_log2ParallelMergeLevelMinus2 + 2;

      MotionInfo miNeigh;
      bool       foundNeighMV = false;
      bool       useL0;
      const int  colPOC = pColPic->m_poc;

      for (int posIdx = 0; posIdx < 5 && foundNeighMV == false; posIdx++)
      {
        const CodingUnit *cuRef = cs.getCURestricted(posCand[posIdx], cu, cu.chType);
        bool              isAvailableNeigh =
          cuRef && PU::isDiffMER(cu.lumaPos(), posCand[posIdx], plevel) && cu != *cuRef && CU::isInter(*cuRef);

        if (isAvailableNeigh)
        {
          miNeigh = cuRef->getMotionInfo(posCand[posIdx]);
          for (int i = 0; i < 2 && foundNeighMV == false; i++)
          {
            int refIdx = miNeigh.refIdx[i];

            if (refIdx != -1)
            {
              const int currRefPOC = slice.getRefPic(RefPicList(i), refIdx)->m_poc;
              if (currRefPOC == colPOC)
              {
                foundNeighMV = true;
                useL0        = i == 0 ? 1 : 0;
              }
            }
          }
        }
      }

      Mv shiftMv;
      if (foundNeighMV)
      {
        shiftMv = useL0 ? miNeigh.mv[0] : miNeigh.mv[1];
        shiftMv.changePrecision(MvPrecision::INTERNAL, MvPrecision::ONE);
        shiftMv.hor = shiftMv.hor >> lumaScaleX;
        shiftMv.ver = shiftMv.ver >> lumaScaleY;
      }
      else
      {
        shiftMv.set(0, 0);
      }

      for (int iDistanceIndex = 0; iDistanceIndex < 5 && numCand < maxCands; iDistanceIndex++)
      {
        const int iNADistanceHor = area.width * iDistanceIndex;
        const int iNADistanceVer = area.height * iDistanceIndex;

        for (int naspIdx = 0; naspIdx < numNACandidate[iDistanceIndex] && numCand < maxCands; naspIdx++)
        {
          switch (idxMap[iDistanceIndex][naspIdx])
          {
          case 0:
            offsetX0 = offsetX2 = 2 + iNADistanceHor;
            offsetY0 = offsetY2 = 2 + iNADistanceVer;
            offsetX1            = iNADistanceHor;
            offsetY1            = iNADistanceVer;
            break;
          case 1:
            offsetX0 = 2;
            offsetY0 = 0;
            offsetX1 = 0;
            offsetY1 = 2;
            break;
          case 2:
            offsetX0 = offsetX2;
            offsetY0 = 2 - offsetY3;
            offsetX1 = 2 - offsetX3;
            offsetY1 = offsetY2;
            break;
          default:
            THROW("error!");
            break;
          }

          c0Avail = false;
          const bool c0BoundaryCond =
            checkBoundaryCondition(posRB.x + shiftMv.hor + offsetX0, posRB.y + shiftMv.ver + offsetY0);
          if (c0BoundaryCond)
          {
            posC0   = posRB.offset(shiftMv.hor + offsetX0, shiftMv.ver + offsetY0);
            c0Avail = true;
          }

          if (idxMap[iDistanceIndex][naspIdx] == 0)
          {
            c1Avail = false;
            const bool c1BoundaryCond =
              checkBoundaryCondition(posCenter.x + shiftMv.hor + offsetX1, posCenter.y + shiftMv.ver + offsetY1);
            if (c1BoundaryCond)
            {
              posC1   = posCenter.offset(shiftMv.hor + offsetX1, shiftMv.ver + offsetY1);
              c1Avail = true;
            }
          }
          else
          {
            c1Avail = false;
            const bool c1BoundaryCond =
              checkBoundaryCondition(posRB.x + shiftMv.hor + offsetX1, posRB.y + shiftMv.ver + offsetY1);
            if (c1BoundaryCond)
            {
              posC1   = posRB.offset(shiftMv.hor + offsetX1, shiftMv.ver + offsetY1);
              c1Avail = true;
            }
          }
          if (c0Avail || c1Avail)
          {
            int modelIdx = c0Avail ? pColPic->m_cs->getEipIdxInfo(posC0) : pColPic->m_cs->getEipIdxInfo(posC1);
            if (modelIdx > 0)
            {
              const EipModels currEipModel = pColPic->m_cs->m_eipModelLUT[modelIdx - 1];
              tryToAddOneModel(currEipModel);
            }
          }
        }
      }
    }
  }

  auto tryHistEip = [&](const static_vector<EipModels, MAX_NUM_HEIP_CANDS> &lut) -> void
  {
    for (int idx = 0; idx < lut.size() && numCand < maxCands; idx++)
    {
      EipModels cand;
      cu.cs->getOneModelFromEipLut(lut, cand, idx);
      bool duplication = false;

      for (int j = 0; j < candList.size(); j++)
      {
        if (cand == candList[j])
        {
          duplication = true;
          // THROW("Should not duplicated");
          break;
        }
      }
      if (!duplication)
      {
        candList.push_back(cand);
        numCand++;
      }
      if (numCand >= maxCands)
      {
        return;
      }
    }
  };

  tryHistEip(cu.cs->eipLut.lutEip);
}

void IntraPrediction::reorderEipCands(const CodingUnit &cu, static_vector<EipModels, MAX_MERGE_EIP> &candList,
                                      const CompID compId)
{
  if (candList.size() <= 1)
  {
    return;
  }

  CHECK(candList.size() > MAX_MERGE_EIP, "candlist size is error.");
  const int refSizeY    = m_eipBlkArea.y;
  const int refSizeX    = m_eipBlkArea.x;
  const int stride      = m_eipBlkArea.width;
  const int blockHeight = cu.blocks[compId].height;
  const int blockWidth  = cu.blocks[compId].width;
  const int topOffset   = stride * (refSizeY - EIP_TPL_SIZE) + refSizeX;
  const int leftOffset  = stride * refSizeY + refSizeX - EIP_TPL_SIZE;
  PelBuf    recoTop(m_eipBuffer + topOffset, stride, blockWidth, EIP_TPL_SIZE);
  PelBuf    recoLeft(m_eipBuffer + leftOffset, stride, EIP_TPL_SIZE, blockHeight);
  PelBuf    predTop(m_eipPredTpl[0], blockWidth, EIP_TPL_SIZE);
  PelBuf    predLeft(m_eipPredTpl[1], EIP_TPL_SIZE, blockHeight);
  const int bd = cu.slice->m_sps->m_bitDepths[toChannelType(compId)];

  DistParam cDistParam;
  cDistParam.applyWeight = false;
  static_vector<double, MAX_MERGE_EIP>    candCostList;
  static_vector<EipModels, MAX_MERGE_EIP> tmpCandList;
  Pel                                     inputs[EIP_FILTER_TAP];

  ClpRng clipRng = cu.slice->m_clpRngs.comp[compId];
  clipRng.min    = std::max(clipRng.min, m_eipClipMin);
  clipRng.max    = std::min(clipRng.max, m_eipClipMax);

  for (auto models: candList)
  {
    Distortion cost = 0;
    const int  filterIdx =
      models.model[0].modelType - CONV_MODEL_EIP_FIRST;   //  CONV_MODEL_EIP_S => 0, CONV_MODEL_EIP_V => 1 ...

    if (models.isMultiModel())
    {
      for (int h = 0; h < EIP_TPL_SIZE; h++)
      {
        for (int w = 0; w < blockWidth; w++)
        {
          setEipInputVector(recoTop, w, h, filterIdx, inputs);
          int modelIdx     = (getEipInputsAvg(inputs, models.model[0].modelType) <= models.threshold) ? 0 : 1;
          predTop.at(w, h) = ClipPel(models.model[modelIdx].convolve(inputs), clipRng);
        }
      }
    }
    else
    {
      for (int h = 0; h < EIP_TPL_SIZE; h++)
      {
        for (int w = 0; w < blockWidth; w++)
        {
          setEipInputVector(recoTop, w, h, filterIdx, inputs);
          predTop.at(w, h) = ClipPel(models.model[0].convolve(inputs), clipRng);
        }
      }
    }

    // #else JVET_AJ0096_SATD_REORDER_INTRA
    m_rdCost.setDistParam(cDistParam, predTop, recoTop, bd, compId, false);
    cost += cDistParam.distFunc(cDistParam);

    if (models.isMultiModel())
    {
      for (int h = 0; h < blockHeight; h++)
      {
        for (int w = 0; w < EIP_TPL_SIZE; w++)
        {
          setEipInputVector(recoLeft, w, h, filterIdx, inputs);
          int modelIdx      = (getEipInputsAvg(inputs, models.model[0].modelType) <= models.threshold) ? 0 : 1;
          predLeft.at(w, h) = ClipPel(models.model[modelIdx].convolve(inputs), clipRng);
        }
      }
    }
    else
    {
      for (int h = 0; h < blockHeight; h++)
      {
        for (int w = 0; w < EIP_TPL_SIZE; w++)
        {
          setEipInputVector(recoLeft, w, h, filterIdx, inputs);
          predLeft.at(w, h) = ClipPel(models.model[0].convolve(inputs), clipRng);
        }
      }
    }

    // #else JVET_AJ0096_SATD_REORDER_INTRA
    m_rdCost.setDistParam(cDistParam, predLeft, recoLeft, bd, compId, false);
    cost += cDistParam.distFunc(cDistParam);

    updateCandList(models, (double)cost, tmpCandList, candCostList, NUM_EIP_MERGE_SIGNAL);
  }
  candList.clear();
  for (auto model: tmpCandList)
  {
    candList.push_back(model);
  }
}

void IntraPrediction::eipPred(const CodingUnit &cu, PelBuf &pred, const CompID compId)
{
  EipModels models = cu.eipModels;

  const int refSizeY    = m_eipBlkArea.y;
  const int refSizeX    = m_eipBlkArea.x;
  const int stride      = m_eipBlkArea.width;
  const int blockHeight = pred.height;
  const int blockWidth  = pred.width;

  PelBuf             predBuf(m_eipBuffer + refSizeY * stride + refSizeX, stride, blockWidth, blockHeight);
  const ScanElement *scan = g_scanOrder[SCAN_UNGROUPED][CoeffScanType::DIAG][gp_sizeIdxInfo->idxFrom(blockWidth)]
                                       [gp_sizeIdxInfo->idxFrom(blockHeight)];
  const int num = blockWidth * blockHeight;
  Pel       inputs[EIP_FILTER_TAP];

  ClpRng clipRng = cu.slice->m_clpRngs.comp[compId];
  clipRng.min    = std::max(clipRng.min, m_eipClipMin);
  clipRng.max    = std::min(clipRng.max, m_eipClipMax);

  if (models.isMultiModel())
  {
    const int filterIdx =
      models.model[0].modelType - CONV_MODEL_EIP_FIRST;   //  CONV_MODEL_EIP_S => 0, CONV_MODEL_EIP_V => 1 ...

    for (int scanIdx = 0; scanIdx < num; scanIdx++)
    {
      setEipInputVector(predBuf, scan[scanIdx].x, scan[scanIdx].y, filterIdx, inputs);
      int modelIdx = (getEipInputsAvg(inputs, models.model[0].modelType) <= models.threshold) ? 0 : 1;
      predBuf.at(scan[scanIdx].x, scan[scanIdx].y) = ClipPel(models.model[modelIdx].convolve(inputs), clipRng);
    }
  }
  else
  {
    ConvModel &model    = models.model[0];
    const int filterIdx = model.modelType - CONV_MODEL_EIP_FIRST;   //  CONV_MODEL_EIP_S => 0, CONV_MODEL_EIP_V => 1 ...

    for (int scanIdx = 0; scanIdx < num; scanIdx++)
    {
      setEipInputVector(predBuf, scan[scanIdx].x, scan[scanIdx].y, filterIdx, inputs);
      predBuf.at(scan[scanIdx].x, scan[scanIdx].y) = ClipPel(model.convolve(inputs), clipRng);
    }
  }

  pred.copyFrom(predBuf);
}
