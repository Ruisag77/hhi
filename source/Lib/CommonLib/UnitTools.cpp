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

/** \file     UnitTool.cpp
 *  \brief    defines operations for basic units
 */

#include "UnitTools.h"

#include "dtrace_next.h"
#include "dtrace_buffer.h"

#include "Unit.h"
#include "Slice.h"
#include "Picture.h"

#include <utility>
#include <algorithm>

// CS tools

bool CS::isDualITree(const CodingStructure &cs) { return cs.slice->isIntra() && !cs.pcv->ISingleTree; }

UnitArea CS::getArea(const CodingStructure &cs, const UnitArea &area, const ChannelType chType)
{
  return isDualITree(cs) ? area.singleChan(chType) : area;
}

void CS::saveTemporalEipModel(CodingStructure &cs)
{
  if (cs.area.Y().area() > 0)
  {
    EipModelIdxBuf eipIdxBuf = cs.getEipIdxBuf(cs.area.Y());
    eipIdxBuf.fill(0);
  }
  cs.m_eipModelLUT.resize(0);

  int idx = 0;
  for (CodingUnit *cu: cs.cus)
  {
    if (cu->Y().valid() && cu->eipFlag)
    {
      EipModels eipModels = cu->eipModels;
      cs.m_eipModelLUT.push_back(eipModels);
      idx++;
      EipModelIdxBuf eipIdxBuf = cu->cs->getEipIdxBuf(cu->Y());
      eipIdxBuf.fill(idx);
    }
  }
}

// CU tools

bool CU::getRprScaling(const SPS *sps, const PPS *curPPS, Picture *refPic, ScalingRatio &scalingRatio)
{
  const int subWidthC  = SPS::getWinUnitX(sps->m_chromaFormatIdc);
  const int subHeightC = SPS::getWinUnitY(sps->m_chromaFormatIdc);

  const Window &curScalingWindow = curPPS->m_scalingWindow;

  const int curLeftOffset   = subWidthC * curScalingWindow.m_winLeftOffset;
  const int curRightOffset  = subWidthC * curScalingWindow.m_winRightOffset;
  const int curTopOffset    = subHeightC * curScalingWindow.m_winTopOffset;
  const int curBottomOffset = subHeightC * curScalingWindow.m_winBottomOffset;

  // Note: 64-bit integers are used for sizes such as to avoid possible overflows in corner cases
  const int64_t curPicScalWinWidth  = curPPS->m_picWidthInLumaSamples - (curLeftOffset + curRightOffset);
  const int64_t curPicScalWinHeight = curPPS->m_picHeightInLumaSamples - (curTopOffset + curBottomOffset);

  const Window &refScalingWindow = refPic->m_scalingWindow;

  const int refLeftOffset   = subWidthC * refScalingWindow.m_winLeftOffset;
  const int refRightOffset  = subWidthC * refScalingWindow.m_winRightOffset;
  const int refTopOffset    = subHeightC * refScalingWindow.m_winTopOffset;
  const int refBottomOffset = subHeightC * refScalingWindow.m_winBottomOffset;

  const int64_t refPicScalWinWidth  = refPic->getPicWidthInLumaSamples() - (refLeftOffset + refRightOffset);
  const int64_t refPicScalWinHeight = refPic->getPicHeightInLumaSamples() - (refTopOffset + refBottomOffset);

  CHECK(curPicScalWinWidth * 2 < refPicScalWinWidth,
        "curPicScalWinWidth * 2 shall be greater than or equal to refPicScalWinWidth");
  CHECK(curPicScalWinHeight * 2 < refPicScalWinHeight,
        "curPicScalWinHeight * 2 shall be greater than or equal to refPicScalWinHeight");
  CHECK(curPicScalWinWidth > refPicScalWinWidth * 8,
        "curPicScalWinWidth shall be less than or equal to refPicScalWinWidth * 8");
  CHECK(curPicScalWinHeight > refPicScalWinHeight * 8,
        "curPicScalWinHeight shall be less than or equal to refPicScalWinHeight * 8");

  scalingRatio.x = (int)(((refPicScalWinWidth << ScalingRatio::BITS) + (curPicScalWinWidth >> 1)) / curPicScalWinWidth);
  scalingRatio.y =
    (int)(((refPicScalWinHeight << ScalingRatio::BITS) + (curPicScalWinHeight >> 1)) / curPicScalWinHeight);

  const int maxPicWidth  = sps->m_maxWidthInLumaSamples;    // sps_pic_width_max_in_luma_samples
  const int maxPicHeight = sps->m_maxHeightInLumaSamples;   // sps_pic_height_max_in_luma_samples
  const int curPicWidth  = curPPS->m_picWidthInLumaSamples;    // pps_pic_width_in_luma_samples
  const int curPicHeight = curPPS->m_picHeightInLumaSamples;   // pps_pic_height_in_luma_samples

  const int picSizeIncrement = std::max((int)8, (1 << sps->m_log2MinCodingBlockSize));   // Max(8, MinCbSizeY)

  CHECK((curPicScalWinWidth * maxPicWidth) < refPicScalWinWidth * (curPicWidth - picSizeIncrement),
        "(curPicScalWinWidth * maxPicWidth) should be greater than or equal to refPicScalWinWidth * (curPicWidth - "
        "picSizeIncrement))");
  CHECK((curPicScalWinHeight * maxPicHeight) < refPicScalWinHeight * (curPicHeight - picSizeIncrement),
        "(curPicScalWinHeight * maxPicHeight) should be greater than or equal to refPicScalWinHeight * (curPicHeight - "
        "picSizeIncrement))");

  CHECK(curLeftOffset < -curPicWidth * 15,
        "The value of SubWidthC * pps_scaling_win_left_offset shall be greater "
        "than or equal to -pps_pic_width_in_luma_samples * 15");
  CHECK(curLeftOffset >= curPicWidth,
        "The value of SubWidthC * pps_scaling_win_left_offset shall be less than pps_pic_width_in_luma_samples");
  CHECK(curRightOffset < -curPicWidth * 15,
        "The value of SubWidthC * pps_scaling_win_right_offset shall be greater "
        "than or equal to -pps_pic_width_in_luma_samples * 15");
  CHECK(curRightOffset >= curPicWidth,
        "The value of SubWidthC * pps_scaling_win_right_offset shall be less than pps_pic_width_in_luma_samples");

  CHECK(curTopOffset < -curPicHeight * 15,
        "The value of SubHeightC * pps_scaling_win_top_offset shall be greater "
        "than or equal to -pps_pic_height_in_luma_samples * 15");
  CHECK(curTopOffset >= curPicHeight,
        "The value of SubHeightC * pps_scaling_win_top_offset shall be less than pps_pic_height_in_luma_samples");
  CHECK(curBottomOffset < (-curPicHeight) * 15,
        "The value of SubHeightC * pps_scaling_win_bottom_offset shall be "
        "greater than or equal to -pps_pic_height_in_luma_samples * 15");
  CHECK(curBottomOffset >= curPicHeight,
        "The value of SubHeightC * pps_scaling_win_bottom_offset shall be less than pps_pic_height_in_luma_samples");

  CHECK(curLeftOffset + curRightOffset < -curPicWidth * 15,
        "The value of SubWidthC * ( pps_scaling_win_left_offset + pps_scaling_win_right_offset ) shall be greater than "
        "or equal to -pps_pic_width_in_luma_samples * 15");
  CHECK(curLeftOffset + curRightOffset >= curPicWidth,
        "The value of SubWidthC * ( pps_scaling_win_left_offset + pps_scaling_win_right_offset ) shall be less than "
        "pps_pic_width_in_luma_samples");
  CHECK(curTopOffset + curBottomOffset < -curPicHeight * 15,
        "The value of SubHeightC * ( pps_scaling_win_top_offset + pps_scaling_win_bottom_offset ) shall be greater "
        "than or equal to -pps_pic_height_in_luma_samples * 15");
  CHECK(curTopOffset + curBottomOffset >= curPicHeight,
        "The value of SubHeightC * ( pps_scaling_win_top_offset + pps_scaling_win_bottom_offset ) shall be less than "
        "pps_pic_height_in_luma_samples");

  return refPic->isRefScaled(curPPS);
}

void CU::checkConformanceILRP(Slice *slice)
{
  const int numRefList = slice->isInterB() ? 2 : 1;

  int currentSubPicIdx = NOT_VALID;

  // derive sub-picture index for the current slice
  for (int subPicIdx = 0; subPicIdx < slice->m_pic->m_cs->sps->m_numSubPics; subPicIdx++)
  {
    if (slice->m_pic->m_cs->pps->m_subPics[subPicIdx].m_subPicID == slice->m_sliceSubPicId)
    {
      currentSubPicIdx = subPicIdx;
      break;
    }
  }

  CHECK(currentSubPicIdx == NOT_VALID, "Sub-picture was not found");

  if (!slice->m_pic->m_cs->sps->m_subPicTreatedAsPicFlag[currentSubPicIdx])
  {
    return;
  }

  // constraint 1: The picture referred to by each active entry in RefPicList[ 0 ] or RefPicList[ 1 ] has the same
  // subpicture layout as the current picture
  bool isAllRefSameSubpicLayout = true;
  for (int refList = 0; refList < numRefList; refList++) // loop over l0 and l1
  {
    RefPicList eRefPicList = (refList ? RPL1 : RPL0);

    for (int refIdx = 0; refIdx < slice->m_numRefIdx[eRefPicList]; refIdx++)
    {
      const Picture *refPic = slice->getRefPic(eRefPicList, refIdx);

      if (refPic->m_subPictures.size() != slice->m_pic->m_cs->pps->m_numSubPics)
      {
        isAllRefSameSubpicLayout = false;
        refList                  = numRefList;
        break;
      }
      else
      {
        for (int i = 0; i < refPic->m_subPictures.size(); i++)
        {
          const SubPic &refSubPic = refPic->m_subPictures[i];
          const SubPic &curSubPic = slice->m_pic->m_cs->pps->m_subPics[i];

          if (refSubPic.m_subPicWidthInCTUs != curSubPic.m_subPicWidthInCTUs ||
              refSubPic.m_subPicHeightInCTUs != curSubPic.m_subPicHeightInCTUs ||
              refSubPic.m_subPicCtuTopLeftX != curSubPic.m_subPicCtuTopLeftX ||
              refSubPic.m_subPicCtuTopLeftY != curSubPic.m_subPicCtuTopLeftY ||
              (refPic->m_layerId != slice->m_pic->m_layerId && refSubPic.m_subPicID != curSubPic.m_subPicID) ||
              refSubPic.m_treatedAsPicFlag != curSubPic.m_treatedAsPicFlag)
          {
            isAllRefSameSubpicLayout = false;
            refIdx                   = slice->m_numRefIdx[eRefPicList];
            refList                  = numRefList;
            break;
          }
        }

        // A picture with different sub-picture ID of the collocated sub-picture cannot be used as an active reference
        // picture in the same layer
        if (refPic->m_layerId == slice->m_pic->m_layerId)
        {
          isAllRefSameSubpicLayout =
            isAllRefSameSubpicLayout && refPic->m_subPictures[currentSubPicIdx].m_subPicID == slice->m_sliceSubPicId;
        }
      }
    }
  }

  // constraint 2: The picture referred to by each active entry in RefPicList[ 0 ] or RefPicList[ 1 ] is an ILRP for
  // which the value of sps_num_subpics_minus1 is equal to 0
  if (!isAllRefSameSubpicLayout)
  {
    for (int refList = 0; refList < numRefList; refList++)   // loop over l0 and l1
    {
      const RefPicList eRefPicList = refList ? RPL1 : RPL0;
      for (int refIdx = 0; refIdx < slice->m_numRefIdx[eRefPicList]; refIdx++)
      {
        const Picture *refPic = slice->getRefPic(eRefPicList, refIdx);
        CHECK(refPic->m_layerId == slice->m_pic->m_layerId || refPic->m_subPictures.size() > 1,
              "The inter-layer reference shall contain a single subpicture or have same subpicture layout with the "
              "current picture");
      }
    }
  }

  return;
}

bool CU::allowTimdSad(const CodingUnit &cu)
{
  if (!cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType) || cu.bdpcmMode[0] != BdpcmMode::NONE)
  {
    return false;
  }

  const auto cuArea  = cu.lwidth() * cu.lheight();
  const auto minSize = std::min(cu.lwidth(), cu.lheight());
  if (cu.slice->isIntra() && (cuArea > 1024 || minSize == 4))
  {
    return false;
  }

  return cu.cs->sps->m_useTIMD;
}

bool CU::isSameSlice(const CodingUnit &cu, const CodingUnit &cu2)
{
  return cu.slice->m_independentSliceIdx == cu2.slice->m_independentSliceIdx;
}

bool CU::isSameTile(const CodingUnit &cu, const CodingUnit &cu2) { return cu.tileIdx == cu2.tileIdx; }

bool CU::isSameSliceAndTile(const CodingUnit &cu, const CodingUnit &cu2)
{
  return (cu.slice->m_independentSliceIdx == cu2.slice->m_independentSliceIdx) && (cu.tileIdx == cu2.tileIdx);
}

bool CU::isSameSubPic(const CodingUnit &cu, const CodingUnit &cu2)
{
  return (cu.slice->m_pps->getSubPicFromCU(cu).m_subPicIdx == cu2.slice->m_pps->getSubPicFromCU(cu2).m_subPicIdx);
}

bool CU::isSameCtu(const CodingUnit &cu, const CodingUnit &cu2)
{
  const uint32_t ctuSizeBit = floorLog2(cu.cs->sps->m_maxCuWidth);

  Position pos1Ctu(cu.lumaPos().x >> ctuSizeBit, cu.lumaPos().y >> ctuSizeBit);
  Position pos2Ctu(cu2.lumaPos().x >> ctuSizeBit, cu2.lumaPos().y >> ctuSizeBit);

  return pos1Ctu.x == pos2Ctu.x && pos1Ctu.y == pos2Ctu.y;
}

bool CU::isLastSubCUOfCtu(const CodingUnit &cu)
{
  const Area cuAreaY = CS::isDualITree(*cu.cs)
    ? Area(recalcPosition(cu.chromaFormat, cu.chType, ChannelType::LUMA, cu.block(cu.chType).pos()),
           recalcSize(cu.chromaFormat, cu.chType, ChannelType::LUMA, cu.block(cu.chType).size()))
    : (const Area &)cu.Y();

  return ((((cuAreaY.x + cuAreaY.width) & cu.cs->pcv->maxCUWidthMask) == 0 ||
           cuAreaY.x + cuAreaY.width == cu.cs->pps->m_picWidthInLumaSamples) &&
          (((cuAreaY.y + cuAreaY.height) & cu.cs->pcv->maxCUHeightMask) == 0 ||
           cuAreaY.y + cuAreaY.height == cu.cs->pps->m_picHeightInLumaSamples));
}

bool CU::isOnCtuBottom(const CodingUnit &cu)
{
  const CodingStructure &cs     = *cu.cs;
  const CompID           compID = cu.chType == ChannelType::CHROMA ? COMP_Cb : COMP_Y;
  const Area            &cuArea = cu.chType == ChannelType::CHROMA ? cu.Cb() : cu.Y();

  const int ctuHeight = cs.pcv->maxCUHeight >> getComponentScaleY(compID, cs.pcv->chrFormat);
  const int cuBottomY = cuArea.y + cuArea.height;

  return cuBottomY % ctuHeight == 0;
}

uint32_t CU::getCtuAddr(const CodingUnit &cu) { return getCtuAddr(cu.block(cu.chType).lumaPos(), *cu.cs->pcv); }

int CU::predictQP(const CodingUnit &cu, const int prevQP)
{
  const CodingStructure &cs = *cu.cs;

  const uint32_t  ctuRsAddr      = getCtuAddr(cu);
  const uint32_t  ctuXPosInCtus  = ctuRsAddr % cs.pcv->widthInCtus;
  const uint32_t  tileColIdx     = cu.slice->m_pps->ctuToTileCol(ctuXPosInCtus);
  const uint32_t  tileXPosInCtus = cu.slice->m_pps->getTileColumnBd(tileColIdx);
  const CompArea &area           = cu.block(cu.chType);
  if (ctuXPosInCtus == tileXPosInCtus &&
      !(area.x & (cs.pcv->maxCUWidthMask >> getChannelTypeScaleX(cu.chType, cu.chromaFormat))) &&
      !(area.y & (cs.pcv->maxCUHeightMask >> getChannelTypeScaleY(cu.chType, cu.chromaFormat))) &&
      (cs.getCU(area.pos().offset(0, -1), cu.chType) != nullptr) &&
      CU::isSameSliceAndTile(*cs.getCU(area.pos().offset(0, -1), cu.chType), cu))
  {
    return ((cs.getCU(area.pos().offset(0, -1), cu.chType))->qp);
  }
  else
  {
    const int a = (area.y & (cs.pcv->maxCUHeightMask >> getChannelTypeScaleY(cu.chType, cu.chromaFormat)))
      ? (cs.getCU(area.pos().offset(0, -1), cu.chType))->qp
      : prevQP;
    const int b = (area.x & (cs.pcv->maxCUWidthMask >> getChannelTypeScaleX(cu.chType, cu.chromaFormat)))
      ? (cs.getCU(area.pos().offset(-1, 0), cu.chType))->qp
      : prevQP;

    return (a + b + 1) >> 1;
  }
}

void CU::saveMotionForHmvp(const CodingUnit &cu)
{
  if (cu.affine)
  {
    if (cu.mergeType != MergeType::DEFAULT_N)
    {
      return;
    }

    AffineMotionInfo addMi[2];
    int              addRefIdx[2];

    cu.getAffineMotionInfo(addMi, addRefIdx);

    cu.cs->addAffMiToLut(cu.cs->motionLut.lutAff, addMi, addRefIdx);

    AffineInheritInfo addAffInherit;
    addAffInherit.basePos                        = cu.lumaPos();
    addAffInherit.baseMV[0]                      = MvField(cu.mvAffi[0][0], cu.refIdx[0]);
    addAffInherit.baseMV[1]                      = MvField(cu.mvAffi[1][0], cu.refIdx[1]);
    addAffInherit.oneSetAffineParametersPattern0 = addMi[0].oneSetAffineParametersPattern;
    addAffInherit.oneSetAffineParametersPattern1 = addMi[1].oneSetAffineParametersPattern;
    if (addAffInherit.oneSetAffineParametersPattern0 == 0)
    {
      addAffInherit.baseMV[0].refIdx = -1;
    }
    if (addAffInherit.oneSetAffineParametersPattern1 == 0)
    {
      addAffInherit.baseMV[1].refIdx = -1;
    }
    if (addAffInherit.baseMV[0].refIdx != -1 || addAffInherit.baseMV[1].refIdx != -1)
    {
      cu.cs->addAffInheritToLut(cu.cs->motionLut.lutAffInherit, addAffInherit);
    }
    return;
  }

  if (!cu.geoFlag && !cu.affine && !(CU::isIBC(cu) && cu.lwidth() * cu.lheight() <= 16))
  {
    MotionInfo mi = cu.getMotionInfo();

    mi.bcwIdx = mi.interDir == 3 ? cu.bcwIdx : BCW_DEFAULT;
    CHECK(mi.usesLIC != cu.licFlag, "mismatch");
    CHECK(!cu.cs->sps->m_biLicEnabledFlag && mi.usesLIC && mi.interDir == 3, "bi-predicted LIC is disabled");

    if (CU::isIBC(cu))
    {
      cu.cs->addMiToLut(cu.cs->motionLut.lutIbc, mi);
    }
    else
    {
      const uint32_t  mask = ~0u << (cu.cs->sps->m_log2ParallelMergeLevelMinus2 + 2);
      const CompArea &area = cu.Y();

      if ((((area.x + area.width) ^ area.x) & mask) != 0 && (((area.y + area.height) ^ area.y) & mask) != 0)
      {
        cu.cs->addMiToLut(cu.cs->motionLut.lut, mi);
      }
    }
  }
}

PartSplit CU::getSplitAtDepth(const CodingUnit &cu, const unsigned depth)
{
  if (depth >= cu.depth)
  {
    return CU_DONT_SPLIT;
  }

  const PartSplit cuSplitType = PartSplit((cu.splitSeries >> (depth * SPLIT_DMULT)) & SPLIT_MASK);

  if (cuSplitType == CU_QUAD_SPLIT)
  {
    return CU_QUAD_SPLIT;
  }
  else if (cuSplitType == CU_HORZ_SPLIT)
  {
    return CU_HORZ_SPLIT;
  }
  else if (cuSplitType == CU_VERT_SPLIT)
  {
    return CU_VERT_SPLIT;
  }
  else if (cuSplitType == CU_TRIH_SPLIT)
  {
    return CU_TRIH_SPLIT;
  }
  else if (cuSplitType == CU_TRIV_SPLIT)
  {
    return CU_TRIV_SPLIT;
  }
  else
  {
    THROW("Unknown split mode");
    return CU_QUAD_SPLIT;
  }
}

bool CU::canUseGPM(const CodingUnit &cu)
{
  if (cu.cs->sps->m_useGeo && !cu.slice->isIntra() && cu.cs->sps->m_maxNumGeoCand > 0 &&
      GEO_MIN_CU_SIZE <= cu.lwidth() && cu.lwidth() <= GEO_MAX_CU_SIZE && cu.lwidth() < 8 * cu.lheight() &&
      GEO_MIN_CU_SIZE <= cu.lheight() && cu.lheight() <= GEO_MAX_CU_SIZE && cu.lheight() < 8 * cu.lwidth())
  {
    return true;
  }
  return false;
}

bool CU::hasAffineNb(const CodingUnit &cu)
{
  int      affineNb = 0;
  Position pos[7]   = { cu.lumaPos().offset(-1, 0),           cu.lumaPos().offset(0, -1),
                        cu.lumaPos().offset(-1, -1),          cu.lumaPos().offset(cu.lwidth() - 1, -1),
                        cu.lumaPos().offset(cu.lwidth(), -1), cu.lumaPos().offset(-1, cu.lheight() - 1),
                        cu.lumaPos().offset(-1, cu.lheight()) };
  for (int i = 0; i < 7; i++)
  {
    const CodingUnit *cuNb = cu.cs->getCURestricted(pos[i], cu, ChannelType::LUMA);
    affineNb += (cuNb && cuNb->affine) ? 1 : 0;
    if (affineNb)
    {
      return true;
    }
  }

  int offsetX  = 0;
  int offsetY  = 0;
  int offsetX0 = 0;
  int offsetX1 = 0;
  int offsetX2 = cu.lwidth() >> 1;
  int offsetY0 = 0;
  int offsetY1 = 0;
  int offsetY2 = cu.lheight() >> 1;

  const int numNACandidate = 5;
  const int idxMap[5]      = { 0, 1, 2, 3, 4 };
  int       iDistanceIndex = 0;
  const int iNADistanceHor = cu.lwidth() * (iDistanceIndex + 1);
  const int iNADistanceVer = cu.lheight() * (iDistanceIndex + 1);

  for (int iNASPIdx = 0; iNASPIdx < numNACandidate; iNASPIdx++)
  {
    switch (idxMap[iNASPIdx])
    {
    case 0:
      offsetX = offsetX0 = -iNADistanceHor - 1;
      offsetY = offsetY0 = cu.lheight() + iNADistanceVer - 1;
      break;
    case 1:
      offsetX = offsetX1 = cu.lwidth() + iNADistanceHor - 1;
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
      printf("error!");
      exit(0);
      break;
    }
    const CodingUnit *cuNb = cu.cs->getCURestricted(cu.lumaPos().offset(offsetX, offsetY), cu, ChannelType::LUMA);
    affineNb += (cuNb && (cuNb->affine || (cuNb->geoFlag))) ? 1 : 0;
    if (affineNb)
    {
      return true;
    }
  }

  return false;
}

bool CU::isAffineAllowed(const CodingUnit &cu)
{
  if ((cu.lumaSize().width >= 8 && cu.lumaSize().height >= 8) ||
      (cu.lumaSize().width * cu.lumaSize().height >= 32 && CU::hasAffineNb(cu) && cu.slice->m_sps->m_affineSbMrgExt))
  {
    return true;
  }
  return false;
}

bool CU::affineCtxInc(const CodingUnit &cu)
{
  if (!(cu.lumaSize().width >= 8 && cu.lumaSize().height >= 8))
  {
    return true;
  }
  return false;
}

TUTraverser CU::traverseTUs(CodingUnit &cu) { return TUTraverser(cu.firstTU, cu.lastTU->next); }

cTUTraverser CU::traverseTUs(const CodingUnit &cu) { return cTUTraverser(cu.firstTU, cu.lastTU->next); }

// PU tools

int PU::getIntraMPMs(const CodingUnit &cu, uint8_t *mpm, uint8_t *non_mpm)
{
  const ChannelType channelType = ChannelType::LUMA;

  bool includedMode[NUM_INTRA_MODE];
  memset(includedMode, false, sizeof(includedMode));

  int numValidMPM          = 0;
  mpm[numValidMPM++]       = PLANAR_IDX;
  includedMode[PLANAR_IDX] = true;

  const CompArea &area  = cu.block(getFirstComponentOfChannel(channelType));
  const Position  posRT = area.topRight();
  const Position  posLB = area.bottomLeft();

  // Get intra direction of left PU
  const CodingUnit *puLeft = (cu.lheight() >= cu.lwidth())
    ? cu.cs->getCURestricted(posRT.offset(0, -1), cu, channelType)
    : cu.cs->getCURestricted(posLB.offset(-1, 0), cu, channelType);
  if (puLeft && CU::isIntra(*puLeft))
  {
    mpm[numValidMPM] = PU::getIntraDirLuma(*puLeft);
    if (!includedMode[mpm[numValidMPM]])
    {
      includedMode[mpm[numValidMPM++]] = true;
    }
  }

  // Get intra direction of above PU
  const CodingUnit *puAbove = (cu.lheight() >= cu.lwidth())
    ? cu.cs->getCURestricted(posLB.offset(-1, 0), cu, channelType)
    : cu.cs->getCURestricted(posRT.offset(0, -1), cu, channelType);
  if (puAbove && CU::isIntra(*puAbove) && CU::isSameCtu(cu, *puAbove))
  {
    mpm[numValidMPM] = PU::getIntraDirLuma(*puAbove);
    if (!includedMode[mpm[numValidMPM]])
    {
      includedMode[mpm[numValidMPM++]] = true;
    }
  }

  int       numCand = -1;
  const int numMPMs = NUM_MOST_PROBABLE_MODES;
  CHECK(2 >= numMPMs, "Invalid number of most probable modes");

  const int offset = (int)NUM_LUMA_MODE - 6;
  const int mod    = offset + 3;

  {
    numCand             = numValidMPM;
    bool checkDCEnabled = false;

    // Derived modes of mpm[1]
    if (numCand >= 2)
    {
      if (mpm[1] > DC_IDX)
      {
        for (int i = 0; i < 4 && numValidMPM < numMPMs; i++)
        {
          mpm[numValidMPM] = ((mpm[1] + offset - i) % mod) + 2;
          if (!includedMode[mpm[numValidMPM]])
          {
            includedMode[mpm[numValidMPM++]] = true;
          }

          if (numValidMPM >= numMPMs)
          {
            break;
          }

          mpm[numValidMPM] = ((mpm[1] - 1 + i) % mod) + 2;
          if (!includedMode[mpm[numValidMPM]])
          {
            includedMode[mpm[numValidMPM++]] = true;
          }
        }
      }
      else if (mpm[1] == DC_IDX)
      {
        checkDCEnabled = true;
      }
    }

    // Derived modes of mpm[2]
    if (numCand >= 3)
    {
      if (mpm[2] > DC_IDX)
      {
        for (int i = 0; i < 4 && numValidMPM < numMPMs; i++)
        {
          mpm[numValidMPM] = ((mpm[2] + offset - i) % mod) + 2;
          if (!includedMode[mpm[numValidMPM]])
          {
            includedMode[mpm[numValidMPM++]] = true;
          }

          if (numValidMPM >= numMPMs)
          {
            break;
          }

          mpm[numValidMPM] = ((mpm[2] - 1 + i) % mod) + 2;
          if (!includedMode[mpm[numValidMPM]])
          {
            includedMode[mpm[numValidMPM++]] = true;
          }
        }
      }
      else if (mpm[2] == DC_IDX)
      {
        checkDCEnabled = true;
      }
    }

    // Derived modes of mpm[3]
    if (checkDCEnabled && numCand >= 4 && mpm[3] > DC_IDX)
    {
      for (int i = 0; i < 3 && numValidMPM < numMPMs; i++)
      {
        mpm[numValidMPM] = ((mpm[3] + offset - i) % mod) + 2;
        if (!includedMode[mpm[numValidMPM]])
        {
          includedMode[mpm[numValidMPM++]] = true;
        }

        if (numValidMPM >= numMPMs)
        {
          break;
        }

        mpm[numValidMPM] = ((mpm[3] - 1 + i) % mod) + 2;
        if (!includedMode[mpm[numValidMPM]])
        {
          includedMode[mpm[numValidMPM++]] = true;
        }
      }
    }

    unsigned mpmDefault[numMPMs - 1] = { DC_IDX, VER_IDX, HOR_IDX, VER_IDX - 4, VER_IDX + 4, 14, 22, 42, 58, 10, 26,
                                         38,     62,      6,       30,          34,          66, 2,  48, 52, 16 };
    for (int idx = 0; (idx < numMPMs - 1) && numValidMPM < numMPMs; idx++)
    {
      mpm[numValidMPM] = mpmDefault[idx];
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
      }
    }

    int numNonMPM = 0;
    for (int idx = 0; idx < NUM_LUMA_MODE; idx++)
    {
      if (!includedMode[idx])
      {
        non_mpm[numNonMPM++] = idx;
      }
    }
  }
  for (int i = 0; i < numMPMs; i++)
  {
    CHECK(mpm[i] >= NUM_LUMA_MODE, "Invalid MPM");
  }
  CHECK(numCand == 0, "No candidates found");
  return numCand;
}

void PU::getGeoIntraMPMs(const CodingUnit &cu, uint8_t *mpm, uint8_t splitDir, uint8_t shape)
{
  bool includedMode[NUM_INTRA_MODE];
  memset(includedMode, false, sizeof(includedMode));

  int numValidMPM      = 0;
  mpm[numValidMPM++]   = g_geoAngle2IntraAng[g_geoParams[splitDir].angleIdx];
  includedMode[mpm[0]] = true;

  const CompArea &area  = cu.block(COMP_Y);
  const Position  posA  = area.topRight().offset(0, -1);
  const Position  posAR = area.topRight().offset(1, -1);
  const Position  posL  = area.bottomLeft().offset(-1, 0);
  const Position  posBL = area.bottomLeft().offset(-1, 1);
  const Position  posAL = area.topLeft().offset(-1, -1);

  if (shape == GEO_TM_SHAPE_L || shape == GEO_TM_SHAPE_AL)
  {
    const CodingUnit *puLeft = cu.cs->getCURestricted(posL, cu, ChannelType::LUMA);
    if (puLeft && CU::isIntra(*puLeft))
    {
      mpm[numValidMPM] = PU::getIntraDirLuma(*puLeft);
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == GEO_MAX_NUM_INTRA_CANDS)
        {
          return;
        }
      }
    }
  }

  if (shape == GEO_TM_SHAPE_A || shape == GEO_TM_SHAPE_AL)
  {
    const CodingUnit *puAbove = cu.cs->getCURestricted(posA, cu, ChannelType::LUMA);
    if (puAbove && CU::isIntra(*puAbove))
    {
      mpm[numValidMPM] = PU::getIntraDirLuma(*puAbove);
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == GEO_MAX_NUM_INTRA_CANDS)
        {
          return;
        }
      }
    }
  }

  if (shape == GEO_TM_SHAPE_L || shape == GEO_TM_SHAPE_AL)
  {
    const CodingUnit *puBelowLeft = cu.cs->getCURestricted(posBL, cu, ChannelType::LUMA);
    if (puBelowLeft && CU::isIntra(*puBelowLeft))
    {
      mpm[numValidMPM] = PU::getIntraDirLuma(*puBelowLeft);
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == GEO_MAX_NUM_INTRA_CANDS)
        {
          return;
        }
      }
    }
  }

  if (shape == GEO_TM_SHAPE_A || shape == GEO_TM_SHAPE_AL)
  {
    const CodingUnit *puAboveRight = cu.cs->getCURestricted(posAR, cu, ChannelType::LUMA);
    if (puAboveRight && CU::isIntra(*puAboveRight))
    {
      mpm[numValidMPM] = PU::getIntraDirLuma(*puAboveRight);
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == GEO_MAX_NUM_INTRA_CANDS)
        {
          return;
        }
      }
    }
  }

  {
    const CodingUnit *puAboveLeft = cu.cs->getCURestricted(posAL, cu, ChannelType::LUMA);
    if (puAboveLeft && CU::isIntra(*puAboveLeft))
    {
      mpm[numValidMPM] = PU::getIntraDirLuma(*puAboveLeft);
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == GEO_MAX_NUM_INTRA_CANDS)
        {
          return;
        }
      }
    }
  }

  mpm[numValidMPM] = (mpm[0] > DIA_IDX) ? (mpm[0] - 32) : (mpm[0] + 32);
  if (!includedMode[mpm[numValidMPM]])
  {
    includedMode[mpm[numValidMPM++]] = true;
    if (numValidMPM == GEO_MAX_NUM_INTRA_CANDS)
    {
      return;
    }
  }
  mpm[numValidMPM] = PLANAR_IDX;
}

void PU::getSgpmIntraMPMs(const CodingUnit &cu, uint8_t *mpm, uint8_t splitDir, uint8_t shape, int dimdMode)
{
  bool includedMode[NUM_INTRA_MODE];
  memset(includedMode, false, sizeof(includedMode));

  int  numValidMPM = 0;
  bool timdDerived = !(cu.lwidth() * cu.lheight() > 1024);
  if (timdDerived)
  {
    if (cu.timdHor > DC_IDX && includedMode[MAP131TO67(cu.timdHor)] == false)
    {
      mpm[numValidMPM] = MAP131TO67(cu.timdHor);
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == SGPM_NUM_MPM)
        {
          return;
        }
      }
    }

    if (cu.timdVer > DC_IDX && includedMode[MAP131TO67(cu.timdVer)] == false)
    {
      mpm[numValidMPM] = MAP131TO67(cu.timdVer);
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == SGPM_NUM_MPM)
        {
          return;
        }
      }
    }
  }

  mpm[numValidMPM] = g_geoAngle2IntraAng[g_geoParams[splitDir].angleIdx];
  if (!includedMode[mpm[numValidMPM]])
  {
    includedMode[mpm[numValidMPM++]] = true;
    if (numValidMPM == SGPM_NUM_MPM)
    {
      return;
    }
  }

  if (cu.slice->m_sps->m_useDIMD)
  {
    if (dimdMode != -1)
    {
      mpm[numValidMPM] = dimdMode;
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == SGPM_NUM_MPM)
        {
          return;
        }
      }
    }
  }

  const CompArea &area  = cu.block(COMP_Y);
  const Position  posA  = area.topRight().offset(0, -1);
  const Position  posAR = area.topRight().offset(1, -1);
  const Position  posL  = area.bottomLeft().offset(-1, 0);
  const Position  posBL = area.bottomLeft().offset(-1, 1);
  const Position  posAL = area.topLeft().offset(-1, -1);

  if (shape == GEO_TM_SHAPE_L || shape == GEO_TM_SHAPE_AL)
  {
    const CodingUnit *cuLeft = cu.cs->getCURestricted(posL, cu, ChannelType::LUMA);
    if (cuLeft && CU::isIntra(*cuLeft))
    {
      mpm[numValidMPM] = PU::getIntraDirLuma(*cuLeft);
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == SGPM_NUM_MPM)
        {
          return;
        }
      }
    }
  }

  if (shape == GEO_TM_SHAPE_A || shape == GEO_TM_SHAPE_AL)
  {
    const CodingUnit *cuAbove = cu.cs->getCURestricted(posA, cu, ChannelType::LUMA);
    if (cuAbove && CU::isIntra(*cuAbove))
    {
      mpm[numValidMPM] = PU::getIntraDirLuma(*cuAbove);
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == SGPM_NUM_MPM)
        {
          return;
        }
      }
    }
  }

  if (shape == GEO_TM_SHAPE_L || shape == GEO_TM_SHAPE_AL)
  {
    const CodingUnit *cuBelowLeft = cu.cs->getCURestricted(posBL, cu, ChannelType::LUMA);
    if (cuBelowLeft && CU::isIntra(*cuBelowLeft))
    {
      mpm[numValidMPM] = PU::getIntraDirLuma(*cuBelowLeft);
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == SGPM_NUM_MPM)
        {
          return;
        }
      }
    }
  }

  if (shape == GEO_TM_SHAPE_A || shape == GEO_TM_SHAPE_AL)
  {
    const CodingUnit *cuAboveRight = cu.cs->getCURestricted(posAR, cu, ChannelType::LUMA);
    if (cuAboveRight && CU::isIntra(*cuAboveRight))
    {
      mpm[numValidMPM] = PU::getIntraDirLuma(*cuAboveRight);
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == SGPM_NUM_MPM)
        {
          return;
        }
      }
    }
  }

  {
    const CodingUnit *cuAboveLeft = cu.cs->getCURestricted(posAL, cu, ChannelType::LUMA);
    if (cuAboveLeft && CU::isIntra(*cuAboveLeft))
    {
      mpm[numValidMPM] = PU::getIntraDirLuma(*cuAboveLeft);
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == SGPM_NUM_MPM)
        {
          return;
        }
      }
    }
  }

  mpm[numValidMPM] = (mpm[0] > DIA_IDX) ? (mpm[0] - 32) : (mpm[0] + 32);
  if (!includedMode[mpm[numValidMPM]])
  {
    includedMode[mpm[numValidMPM++]] = true;
    if (numValidMPM == SGPM_NUM_MPM)
    {
      return;
    }
  }
  mpm[numValidMPM] = PLANAR_IDX;
}

bool PU::isMIP(const CodingUnit &cu, const ChannelType chType)
{
  if (isLuma(chType))
  {
    // Default case if chType is omitted.
    return cu.mipFlag;
  }
  else
  {
    return isDMChromaMIP(cu) && (cu.intraDir[ChannelType::CHROMA] == DM_CHROMA_IDX);
  }
}

bool PU::isDIMD(const CodingUnit &cu, const ChannelType chType)
{
  return cu.dimdFlag && !cu.obicFlag && isLuma(chType);
}

bool PU::isDIMDChroma(const CodingUnit &cu, const ChannelType chType)
{
  return cu.slice->m_sps->m_useDIMDChroma && cu.dimdChromaFlag && isChroma(chType);
}

bool PU::isTIMD(const CodingUnit &cu, const ChannelType chType) { return cu.timdFlag && isLuma(chType); }

bool PU::isOBIC(const CodingUnit &cu, const ChannelType chType) { return cu.obicFlag && isLuma(chType); }

bool PU::isEIP(const CodingUnit &cu, const ChannelType chType) { return cu.eipFlag && isLuma(chType); }

void PU::interpretLumaIntraMode(CodingUnit &cu)
{
  if (cu.mipFlag || cu.timdFlag || cu.dimdFlag || cu.obicFlag || !cu.Y().valid() || (cu.predMode != MODE_INTRA) ||
      (cu.bdpcmMode[0] != BdpcmMode::NONE) || cu.sgpm || cu.eipFlag)
  {
    return;
  }
  uint8_t mpmPred[NUM_MOST_PROBABLE_MODES];   // mpm_idx / rem_intra_luma_pred_mode
  uint8_t nonMpmPred[NUM_NON_MPM_MODES];
  PU::getIntraMPMs(cu, mpmPred, nonMpmPred);
  if (cu.mpmFlag)
  {
    cu.intraDir[ChannelType::LUMA] = mpmPred[cu.lumaModeIdx];
  }
  else
  {
    if (cu.secondMpmFlag)
    {
      cu.intraDir[ChannelType::LUMA] = mpmPred[cu.lumaModeIdx + NUM_PRIMARY_MOST_PROBABLE_MODES];
    }
    else
    {
      cu.intraDir[ChannelType::LUMA] = nonMpmPred[cu.lumaModeIdx];
    }
  }
}
void PU::interpretChromaIntraMode(CodingUnit &cu)
{
  if ((cu.chromaModeIdx == -1) || (cu.predMode != MODE_INTRA) ||
      (cu.bdpcmMode[1] != BdpcmMode::NONE))   // LM-Modes, DM-Mode, MIP (for 444);
  {
    return;
  }
  unsigned chromaCandModes[NUM_CHROMA_MODE];
  PU::getIntraChromaCandModes(cu, chromaCandModes);
  cu.intraDir[ChannelType::CHROMA] = chromaCandModes[cu.chromaModeIdx];
  CHECK(PU::isLMCMode(chromaCandModes[cu.chromaModeIdx]), "The intra dir cannot be LM_CHROMA for this path");
  CHECK(chromaCandModes[cu.chromaModeIdx] == DM_CHROMA_IDX, "The intra dir cannot be DM_CHROMA for this path");
}
void PU::setLumaIntraModeFlags(CodingUnit &cu, const CUCtxIntra &cuCtxIntra)
{
  if (cu.mipFlag || cu.timdFlag || cu.dimdFlag || cu.obicFlag || cu.bdpcmMode[0] != BdpcmMode::NONE || cu.eipFlag)
  {
    return;
  }
  const int      numMPMs   = NUM_PRIMARY_MOST_PROBABLE_MODES;
  const uint8_t *mpmPred   = cuCtxIntra.mpmList;
  const uint32_t intraMode = cu.intraDir[ChannelType::LUMA];
  for (int idx = 0; idx < numMPMs; idx++)
  {
    if (intraMode == mpmPred[idx])
    {
      cu.mpmFlag     = true;
      cu.lumaModeIdx = idx;
      return;
    }
  }
  auto secondMpmPred = mpmPred + NUM_PRIMARY_MOST_PROBABLE_MODES;
  for (unsigned idx = 0; idx < NUM_SECONDARY_MOST_PROBABLE_MODES; idx++)
  {
    if (intraMode == secondMpmPred[idx])
    {
      cu.mpmFlag       = false;
      cu.secondMpmFlag = true;
      cu.lumaModeIdx   = idx;
      return;
    }
  }
  for (unsigned idx = 0; idx < NUM_NON_MPM_MODES; idx++)
  {
    if (intraMode == cuCtxIntra.nonMPMList[idx])
    {
      cu.mpmFlag       = false;
      cu.secondMpmFlag = false;
      cu.lumaModeIdx   = idx;
      return;
    }
  }
}
void PU::setChromaIntraModeFlag(CodingUnit &cu)
{
  const unsigned intraDir = cu.intraDir[ChannelType::CHROMA];
  if ((cu.bdpcmMode[1] != BdpcmMode::NONE) || PU::isLMCMode(intraDir) || (intraDir == DM_CHROMA_IDX) ||
      (cu.dimdChromaFlag))
  {
    cu.chromaModeIdx = -1;
    return;
  }
  unsigned chromaCandModes[NUM_CHROMA_MODE];
  PU::getIntraChromaCandModes(cu, chromaCandModes);
  int candId = 0;
  for (; candId < NUM_CHROMA_MODE; candId++)
  {
    if (intraDir == chromaCandModes[candId])
    {
      break;
    }
  }
  CHECK(candId >= NUM_CHROMA_MODE, "Chroma prediction mode index out of bounds");
  CHECK(chromaCandModes[candId] == DM_CHROMA_IDX, "The intra dir cannot be DM_CHROMA for this path");
  cu.chromaModeIdx = candId;
}

bool PU::directionalPlanarAvailable(const CodingUnit &cu, const ChannelType &chType)
{
  return (cu.cs->sps->m_usedirPlanar) && (cu.mipFlag == false) && isLuma(chType) && (cu.intraDir[chType] == PLANAR_IDX);
}
void PU::trafoIntraDirPlanar(const CodingUnit &cu, const ChannelType &chType, uint32_t &intraDir)
{
  if (directionalPlanarAvailable(cu, chType) && (cu.plDir != PlanarDirType::NO_DIR))
  {
    intraDir = (cu.plDir == PlanarDirType::HOR) ? HOR_IDX : VER_IDX;
  }
}

bool PU::isDMChromaMIP(const CodingUnit &cu)
{
  return !CS::isDualITree(*cu.cs) && cu.chromaFormat == ChromaFormat::_444 && getCoLocatedLumaCU(cu).mipFlag;
}

bool PU::isSgpm(const CodingUnit &cu, const ChannelType &chType)
{
  if (isLuma(chType))
  {
    // Default case if chType is omitted.
    return cu.sgpm;
  }
  else
  {
    return isDMChromaSgpm(cu) && (cu.intraDir[ChannelType::CHROMA] == DM_CHROMA_IDX);
  }
}

bool PU::isDMChromaSgpm(const CodingUnit &cu) { return false; }

uint32_t PU::getIntraDirLuma(const CodingUnit &cu)
{
  if (isMIP(cu) || isEIP(cu))
  {
    return PLANAR_IDX;
  }
  else
  {
    return cu.intraDir[ChannelType::LUMA];
  }
}

void PU::getIntraChromaCandModes(const CodingUnit &cu, unsigned modeList[NUM_CHROMA_MODE])
{
  modeList[0]  = PLANAR_IDX;
  modeList[1]  = VER_IDX;
  modeList[2]  = HOR_IDX;
  modeList[3]  = DC_IDX;
  modeList[4]  = LM_CHROMA_IDX;
  modeList[5]  = MDLM_L_IDX;
  modeList[6]  = MDLM_T_IDX;
  modeList[7]  = MMLM_CHROMA_IDX;
  modeList[8]  = MMLM_L_IDX;
  modeList[9]  = MMLM_T_IDX;
  modeList[10] = DM_CHROMA_IDX;

  // If Direct Mode is MIP, mode cannot be already in the list.
  if (isDMChromaMIP(cu))
  {
    return;
  }

  if (isDMChromaSgpm(cu))
  {
    return;
  }

  const uint32_t lumaMode = getCoLocatedIntraLumaMode(cu);
  for (int i = 0; i < 4; i++)
  {
    if (lumaMode == modeList[i])
    {
      modeList[i] = VDIA_IDX;
      break;
    }
  }
}

bool PU::isLMCMode(unsigned mode) { return (mode >= LM_CHROMA_IDX && mode <= MMLM_T_IDX); }
bool PU::isMultiModeLM(unsigned mode) { return (mode == MMLM_CHROMA_IDX || mode == MMLM_L_IDX || mode == MMLM_T_IDX); }
bool PU::isLMCModeEnabled(const CodingUnit &cu, unsigned mode)
{
  if (cu.cs->sps->m_LMChroma)
  {
    return true;
  }
  return false;
}

int PU::getLMSymbolList(const CodingUnit &cu, int *modeList)
{
  int idx = 0;

  modeList[idx++] = LM_CHROMA_IDX;
  modeList[idx++] = MMLM_CHROMA_IDX;
  modeList[idx++] = MDLM_L_IDX;
  modeList[idx++] = MDLM_T_IDX;
  modeList[idx++] = MMLM_L_IDX;
  modeList[idx++] = MMLM_T_IDX;
  return idx;
}

uint32_t PU::getFinalIntraMode(const CodingUnit &cu, const ChannelType &chType)
{
  uint32_t intraMode = cu.intraDir[chType];

  if (!isLuma(chType))
  {
    if (intraMode == DM_CHROMA_IDX)
    {
      intraMode = getCoLocatedIntraLumaMode(cu);
    }
    if (cu.chromaFormat == ChromaFormat::_422 && intraMode < NUM_LUMA_MODE)
    {   // map directional, planar and dc
      intraMode = g_chroma422IntraAngleMappingTable[intraMode];
    }
  }
  return intraMode;
}

const CodingUnit &PU::getCoLocatedLumaCU(const CodingUnit &cu)
{
  Position topLeftPos = cu.block(cu.chType).lumaPos();

  Position refPos =
    topLeftPos.offset(cu.block(cu.chType).lumaSize().width >> 1, cu.block(cu.chType).lumaSize().height >> 1);

  const CodingUnit &lumaPU = CS::isDualITree(*cu.cs) ? *cu.cs->picture->m_cs->getLumaCU(refPos) : cu;

  return lumaPU;
}

uint32_t PU::getCoLocatedIntraLumaMode(const CodingUnit &cu)
{
  if (PU::getCoLocatedLumaCU(cu).eipFlag)
  {
    return PU::getCoLocatedLumaCU(cu).inferredDimdMode;
  }
  return PU::getIntraDirLuma(PU::getCoLocatedLumaCU(cu));
}

int PU::getWideAngle(const TransformUnit &tu, const uint32_t dirMode, const CompID compID)
{
  // This function returns a wide angle index taking into account that the values 0 and 1 are reserved
  // for Planar and DC respectively, as defined in the Spec. Text.
  if (dirMode < 2)
  {
    return (int)dirMode;
  }

  const CompArea &area        = tu.blocks[compID];
  int             width       = area.width;
  int             height      = area.height;
  int             modeShift[] = { 0, 6, 10, 12, 14, 15 };
  int             deltaSize   = abs(floorLog2(width) - floorLog2(height));
  int             predMode    = dirMode;

  if (width > height && dirMode < 2 + modeShift[deltaSize])
  {
    predMode += (VDIA_IDX - 1);
  }
  else if (height > width && predMode > VDIA_IDX - modeShift[deltaSize])
  {
    predMode -= (VDIA_IDX + 1);
  }

  return predMode;
}

bool PU::hasCclmDeltaFlag(const CodingUnit &cu, const int mode)
{
  const Area area         = cu.blocks[COMP_Cb];
  const int  chrMode      = mode < 0 ? cu.intraDir[ChannelType::CHROMA] : mode;
  bool       hasDeltaFlag = chrMode == LM_CHROMA_IDX || chrMode == MMLM_CHROMA_IDX;
  hasDeltaFlag &= area.width * area.height >= 128;
  hasDeltaFlag &= area.width <= 128 && area.height <= 128;

  return hasDeltaFlag;
}

void PU::getCccmRefLineNum(const CodingUnit &cu, int &th, int &tv)
{
  const Area area = cu.blocks[COMP_Cb];

  th = area.x < CCCM_WINDOW_SIZE ? area.x : CCCM_WINDOW_SIZE;
  tv = area.y < CCCM_WINDOW_SIZE ? area.y : CCCM_WINDOW_SIZE;

  if (CCCM_REF_LINES_ABOVE_CTU)
  {
    int ctuHeight  = cu.cs->sps->m_maxCuHeight >> getComponentScaleY(COMP_Cb, cu.chromaFormat);
    int borderDist = area.y % ctuHeight;
    int tvMax      = borderDist + CCCM_REF_LINES_ABOVE_CTU;

    tv = tv > tvMax ? tvMax : tv;
  }
}

NeighAreaType PU::crossCompNeighType(int intraMode)
{
  return intraMode == LM_CHROMA_IDX || intraMode == MMLM_CHROMA_IDX ? NEIGH_AREA_TOPLEFT
    : intraMode == MDLM_L_IDX || intraMode == MMLM_L_IDX            ? NEIGH_AREA_LEFT
                                                                    : NEIGH_AREA_TOP;
}

int PU::cccmMultiFilterIndex(const ConvModelType modelType)
{
  return modelType == CONV_MODEL_CCCM_MULTIF_1 ? 1
    : modelType == CONV_MODEL_CCCM_MULTIF_2    ? 2
    : modelType == CONV_MODEL_CCCM_MULTIF_3    ? 3
                                               : 0;
}

bool PU::cccmAvailable(const CodingUnit &cu, const int intraMode, const ConvModelType modelType)
{
  if (modelType == CONV_MODEL_CCCM_BVG)
  {
    if (!((intraMode == LM_CHROMA_IDX) || (intraMode == MMLM_CHROMA_IDX)) || !PU::bvgCccmMultiModeAvail(cu, intraMode))
    {
      return false;
    }
  }

  const Area area   = cu.blocks[COMP_Cb];
  const int  width  = area.width;
  const int  height = area.height;
  const int  size   = width * height;

  const bool fullRef = intraMode == LM_CHROMA_IDX || intraMode == MMLM_CHROMA_IDX;
  const bool leftRef = intraMode == MDLM_L_IDX || intraMode == MMLM_L_IDX;

  int wLeftRef, hTopRef;

  PU::getCccmRefLineNum(cu, wLeftRef, hTopRef);

  int refSamples = fullRef ? ((width + wLeftRef) * (height + hTopRef) - size)
    : leftRef              ? height * wLeftRef
                           : width * hTopRef;
  int refThr     = intraMode == LM_CHROMA_IDX ? 0 : intraMode == MMLM_CHROMA_IDX ? 128 : 16;

  if (PU::cccmMultiFilterIndex(modelType) && PU::isMultiModeLM(intraMode))
  {
    refThr = intraMode == MMLM_CHROMA_IDX ? 128 : 256;
  }

  bool modeIsOk = intraMode >= LM_CHROMA_IDX && intraMode <= MMLM_T_IDX;

  modeIsOk &= cu.cs->sps->m_CCCM;
  modeIsOk &= size >= CCCM_MIN_PU_SIZE;
  modeIsOk &= intraMode == LM_CHROMA_IDX ? (area.x || area.y)
    : intraMode == MMLM_CHROMA_IDX       ? true
                                         : (area.x && area.y);
  modeIsOk &= refSamples >= refThr;

  return modeIsOk;
}

bool PU::hasBvgCccmFlag(const CodingUnit &cu)
{
  if (CS::isDualITree(*cu.cs) && cu.cs->slice->m_sps->m_bvgCccm)
  {
    return bvgCccmModeAvail(cu);
  }
  return false;
}

bool PU::bvgCccmMultiModeAvail(const CodingUnit &cu, int intraMode)
{
  if (intraMode == MMLM_CHROMA_IDX && cu.chromaSize().height * cu.chromaSize().width <= 16)
  {
    return false;
  }
  return true;
}

void PU::getBvgCccmCands(CodingUnit &cu)
{
  for (int posIdx = 0; posIdx < NUM_BVG_CCCM_CANDS; posIdx++)
  {
    Mv  chromaBv      = Mv(0, 0);
    int rrIbcType     = 0;
    cu.bvList[posIdx] = Mv(0, 0);
    if (PU::isBvgCccmCand(cu, chromaBv, rrIbcType, posIdx))
    {
      if (chromaBv == Mv(0, 0))
      {
        continue;
      }
      bool bvExist = false;
      for (int i = 0; i < cu.numBvgCands; i++)
      {
        if (chromaBv == cu.bvList[i])
        {
          bvExist = true;
          break;
        }
      }
      if (!bvExist)
      {
        cu.bvList[cu.numBvgCands] = chromaBv;
        // pu.rrIbcList[pu.numBvgCands] = rrIbcType;
        cu.numBvgCands++;
      }
    }
  }
}

bool PU::bvgCccmModeAvail(const CodingUnit &cu)
{

  CompArea lumaArea                    = CompArea(COMP_Y, cu.chromaFormat, cu.Cb().lumaPos(),
                                                  recalcSize(cu.chromaFormat, ChannelType::CHROMA, ChannelType::LUMA, cu.Cb().size()));
  lumaArea                             = clipArea(lumaArea, cu.cs->picture->block(COMP_Y));
  Position posList[NUM_BVG_CCCM_CANDS] = { lumaArea.center(), lumaArea.topLeft(), lumaArea.topRight(),
                                           lumaArea.bottomLeft(), lumaArea.bottomRight() };
  int      checkOffset                 = BVG_CCCM_POS_OFFSET >> 1;

  for (int n = 0; n < NUM_BVG_CCCM_CANDS; n++)
  {
    const CodingUnit *lumaCU = cu.cs->picture->m_cs->getCU(posList[n], ChannelType::LUMA);

    if (lumaCU && (CU::isIBC(*lumaCU)))   //|| isTmp(*lumaPU))) TMP not yet
    {
      return true;
    }
    else
    {
      // check more locations
      for (int a = 0; a < 3; a++)
      {
        int      offsetX = (a == 0) ? 1 : (a >> 1);
        int      offsetY = (a == 0) ? 1 : (a % 2);
        Position temp(posList[n].x - offsetX * checkOffset, posList[n].y - offsetY * checkOffset);

        const CodingUnit *alumaCU = cu.cs->picture->m_cs->getCU(temp, ChannelType::LUMA);
        if (alumaCU && (CU::isIBC(*alumaCU)))   // || isTmp(*lumaPU)))
        {
          return true;
        }
      }
    }
  }
  return false;
}

bool PU::isBvgCccmCand(const CodingUnit &cu, Mv &chromaBv, int &rrIbcType, int candIdx)
{
  // rrIbcType = 0;
  chromaBv                             = Mv(0, 0);
  // assuming both chroma components have the same bitdepth
  const int shiftHor                   = ::getComponentScaleX(COMP_Cb, cu.chromaFormat);
  const int shiftVer                   = ::getComponentScaleY(COMP_Cr, cu.chromaFormat);
  int       checkOffset                = BVG_CCCM_POS_OFFSET >> 1;
  bool      isBvgFound                 = true;
  CompArea  lumaArea                   = CompArea(COMP_Y, cu.chromaFormat, cu.Cb().lumaPos(),
                                                  recalcSize(cu.chromaFormat, ChannelType::CHROMA, ChannelType::LUMA, cu.Cb().size()));
  lumaArea                             = clipArea(lumaArea, cu.cs->picture->block(COMP_Y));
  Position posList[NUM_BVG_CCCM_CANDS] = { lumaArea.center(), lumaArea.topLeft(), lumaArea.topRight(),
                                           lumaArea.bottomLeft(), lumaArea.bottomRight() };

  const CodingUnit *lumaCU = cu.cs->picture->m_cs->getCU(posList[candIdx], ChannelType::LUMA);

  if (lumaCU && (CU::isIBC(*lumaCU)))   // || isTmp(*lumaPU)))
  {
    if (candIdx > 0)
    {
      for (int n = 0; n < candIdx; n++)
      {
        const CodingUnit *prevLumaCU = cu.cs->picture->m_cs->getCU(posList[n], ChannelType::LUMA);
        if (prevLumaCU == lumaCU)
        {
          isBvgFound = false;
        }
      }
    }
    if (isBvgFound)
    {
      Mv lumaBv = lumaCU->bv;
#if 0   // JVET_AA0070_RRIBC
      lumaBv = adjustChromaBv(*lumaPU, lumaArea);
#endif
      // Bv is already one pel precision
      /*
      const int bvShiftHor = MV_FRACTIONAL_BITS_INTERNAL + shiftHor;
      const int bvShiftVer = MV_FRACTIONAL_BITS_INTERNAL + shiftVer;
      lumaBv.hor = (lumaBv.hor >> bvShiftHor) << shiftHor;
      lumaBv.ver = (lumaBv.ver >> bvShiftVer) << shiftVer;*/
      chromaBv      = Mv(lumaBv.hor >> shiftHor, lumaBv.ver >> shiftVer);
      int maxWidth  = cu.chromaSize().width;
      int maxHeight = cu.chromaSize().height;
      if (PU::checkIsChromaBvCandidateValid(cu, chromaBv, maxWidth, maxHeight))
      {
        // rrIbcType = (int)lumaPU->cu->rribcFlipType;
        return true;
      }
    }
  }
  // check for more
  for (int a = 0; a < 3; a++)
  {
    int               offsetX = (a == 0) ? 1 : (a >> 1);
    int               offsetY = (a == 0) ? 1 : (a % 2);
    Position          temp(posList[candIdx].x - offsetX * checkOffset, posList[candIdx].y - offsetY * checkOffset);
    const CodingUnit *alumaCU = cu.cs->picture->m_cs->getCU(temp, ChannelType::LUMA);
    if (alumaCU && (CU::isIBC(*alumaCU)))   // || isTmp(*lumaPU)))
    {
      if (candIdx > 0)
      {
        for (int n = 0; n < candIdx; n++)
        {
          const CodingUnit *prevLumaCU = cu.cs->picture->m_cs->getCU(posList[n], ChannelType::LUMA);
          if (prevLumaCU == alumaCU)
          {
            isBvgFound = false;
          }
        }
      }
      if (isBvgFound)
      {
        Mv lumaBv = alumaCU->bv;
#if 0   // JVET_AA0070_RRIBC
        lumaBv = adjustChromaBv(*lumaPU, lumaArea);
#endif
        // Bv is already one pel precision
        /*
        const int bvShiftHor = MV_FRACTIONAL_BITS_INTERNAL + shiftHor;
        const int bvShiftVer = MV_FRACTIONAL_BITS_INTERNAL + shiftVer;
        lumaBv.hor = (lumaBv.hor >> bvShiftHor) << shiftHor;
        lumaBv.ver = (lumaBv.ver >> bvShiftVer) << shiftVer;*/
        chromaBv      = Mv(lumaBv.hor >> shiftHor, lumaBv.ver >> shiftVer);
        int maxWidth  = cu.chromaSize().width;
        int maxHeight = cu.chromaSize().height;
        if (PU::checkIsChromaBvCandidateValid(cu, chromaBv, maxWidth, maxHeight))
        {
          // rrIbcType = (int)lumaPU->cu->rribcFlipType;
          return true;
        }
      }
    }
  }
  return false;
}

bool PU::checkIsChromaBvCandidateValid(const CodingUnit &cu, const Mv chromaBv, int &maxWidth, int &maxHeight,
                                       bool isRefTemplate, bool isRefAbove)
{
  const int cuPelX  = cu.Cb().x;
  const int cuPelY  = cu.Cb().y;
  // DBV_TEMPLATE_SIZE
  int       tplSize = 1;

  int roiWidth  = (isRefTemplate && !isRefAbove) ? tplSize : cu.Cb().width;
  int roiHeight = (isRefTemplate && isRefAbove) ? tplSize : cu.Cb().height;
  int xPred     = chromaBv.getHor();
  int yPred     = chromaBv.getVer();
  if ((xPred + roiWidth) > 0 && (yPred + roiHeight) > 0)
  {
    maxWidth  = 0;
    maxHeight = 0;
    return false;
  }
  int            refRightX  = cuPelX + xPred + roiWidth - 1;
  int            refLeftX   = cuPelX + xPred;
  int            refBottomY = cuPelY + yPred + roiHeight - 1;
  int            refTopY    = cuPelY + yPred;
  const Position refPosLT(refLeftX, refTopY);
  Position       refPosBR(refRightX, refBottomY);
  if (!cu.cs->isDecomp(refPosLT, cu.chType))
  {
    return false;
  }
  if (!cu.cs->isDecomp(refPosBR, cu.chType))
  {
    Position tmpPosBR(refRightX, refTopY);
    int      validX = refRightX;
    int      validY = refBottomY;
    for (int x = tmpPosBR.x; x >= 0; x -= 1)
    {
      if (!cu.cs->isDecomp(tmpPosBR, cu.chType))
      {
        tmpPosBR.x -= 1;
      }
      else
      {
        validX = tmpPosBR.x;
        break;
      }
    }
    tmpPosBR.x = refLeftX;
    tmpPosBR.y = refBottomY;
    for (int y = tmpPosBR.y; y >= 0; y -= 1)
    {
      if (!cu.cs->isDecomp(tmpPosBR, cu.chType))
      {
        tmpPosBR.y -= 1;
      }
      else
      {
        validY = tmpPosBR.y;
        break;
      }
    }
    tmpPosBR.x = validX;
    tmpPosBR.y = validY;
    if (cu.cs->isDecomp(tmpPosBR, cu.chType))
    {
      maxHeight = maxHeight - (refBottomY - validY);
      maxWidth  = maxWidth - (refRightX - validX);
      return true;
    }
    return false;
  }
  return true;
}

int PU::getLFNSTMatrixDim(const int width, const int height)
{
  if (getUseLFNST16(width, height))
  {
    return L16H;
  }
  if (getUseLFNST8(width, height))
  {
    return L8H;
  }
  return 16;
}

bool PU::getUseLFNST8(const int width, const int height) { return (width >= 8) && (height >= 8); }

bool PU::getUseLFNST16(const int width, int const height) { return (width >= 16) && (height >= 16); }

int PU::getNSPTMatrixDim(const int width, const int height)
{
  int dimension =
    (width == 8 && height == 8) ? 32 : (((width == 4 && height == 8) || (width == 8 && height == 4)) ? 20 : 16);
  dimension = ((width == 4 && height == 16) || (width == 16 && height == 4))
    ? 24
    : (((width == 8 && height == 16) || (width == 16 && height == 8)) ? 40 : dimension);
  dimension = ((width == 4 && height == 32) || (width == 32 && height == 4))
    ? 20
    : (((width == 8 && height == 32) || (width == 32 && height == 8)) ? 24 : dimension);
  return dimension;
}

int PU::getNSPTBucket(const TransformUnit &tu)
{
  const CodingUnit &cu = *tu.cu;
  if (cu.timdFlag || cu.dimdFlag || cu.obicFlag || cu.eipFlag || cu.mipFlag || cu.sgpm)
  {
    return 1;
  }
  if (/*cu.tmpFlag ||*/ CU::isInter(cu))
  {
    return 2;
  }
  // Conventional intra mode
  return 0;
}

std::pair<int8_t, int8_t> PU::getFinalIntraModesTrafo(const TransformUnit &tu, const CompID compID)
{
  const CodingUnit         &cu         = *tu.cu;
  std::pair<int8_t, int8_t> intraModes = std::make_pair<int8_t, int8_t>(-1, -1);

  if (CU::isInter(*tu.cu))
  {
    intraModes = tu.derivedIntraDirsLuma;
  }
  else if (PU::isLMCMode(cu.intraDir[toChannelType(compID)]))
  {
    intraModes.first = tu.derivedIntraDirChroma;
  }
  else if (PU::isMIP(cu, toChannelType(compID)))
  {
    intraModes = (compID == CompID::COMP_Y ? tu.derivedIntraDirsLuma
                                           : std::make_pair<int8_t, int8_t>(int8_t { PLANAR_IDX }, int8_t { DC_IDX }));
  }
  else if (PU::isSgpm(cu, toChannelType(compID)))
  {
    if (compID == CompID::COMP_Y)
    {
      if (cu.lumaSize().area() < MAX_SGPM_VIPM_SIZE)
      {
        bool isMode1Invalid = abs(tu.derivedIntraDirsLuma.first - tu.cu->sgpmMode0) > SGPM_VIPM_TH &&
            abs(tu.derivedIntraDirsLuma.first - tu.cu->sgpmMode1) > SGPM_VIPM_TH
          ? true
          : false;
        bool isMode2Invalid = abs(tu.derivedIntraDirsLuma.second - tu.cu->sgpmMode0) > SGPM_VIPM_TH &&
            abs(tu.derivedIntraDirsLuma.second - tu.cu->sgpmMode1) > SGPM_VIPM_TH
          ? true
          : false;
        if (isMode1Invalid && isMode2Invalid)
        {
          intraModes.first = g_geoAngle2IntraAng[g_geoParams[cu.sgpmSplitDir].angleIdx];
        }
        else
        {
          intraModes.first = (isMode1Invalid ? tu.derivedIntraDirsLuma.second : tu.derivedIntraDirsLuma.first);
        }
        intraModes.second = tu.derivedIntraDirsLuma.second;
      }
      else
      {
        intraModes = tu.derivedIntraDirsLuma;
      }
    }
    else
    {
      intraModes.first = g_geoAngle2IntraAng[g_geoParams[cu.sgpmSplitDir].angleIdx];
    }
  }
  else if (PU::isEIP(cu, toChannelType(compID)))
  {
    intraModes.first = cu.inferredDimdMode;
  }
  else
  {
    intraModes.first = PU::getFinalIntraMode(cu, toChannelType(compID));
    if (PU::directionalPlanarAvailable(cu, toChannelType(compID)) && (cu.plDir != PlanarDirType::NO_DIR))
    {
      intraModes.first = int8_t(cu.plDir == PlanarDirType::HOR ? HOR_IDX : VER_IDX);
    }
    if (compID == CompID::COMP_Y && (cu.timdFlag || cu.dimdFlag || cu.obicFlag))
    {
      intraModes.second = tu.cu->derivedIpm[1];
    }
  }

  if (tu.cs->sps->m_mtsEnabled && !tu.cs->sps->m_explicitMtsIntra && compID == CompID::COMP_Y && CU::isIntra(*tu.cu) &&
      !isNST(tu.mtsIdx[compID]))
  {
    if (intraModes.second == intraModes.first)
    {
      intraModes.second = -1;
    }
    if (intraModes.second < 0)
    {
      const std::array<int8_t, 4> stuffingModes = { int8_t { PLANAR_IDX }, int8_t { DC_IDX }, int8_t { HOR_IDX },
                                                    int8_t { VER_IDX } };
      for (auto mode: stuffingModes)
      {
        if (mode != intraModes.first)
        {
          intraModes.second = mode;
          break;
        }
      }
    }
    return intraModes;
  }

  intraModes.first  = getWANonSepTrafoIntraMode(PU::getWideAngle(tu, uint32_t(intraModes.first), compID));
  intraModes.second = -1;
  return intraModes;
}

uint32_t PU::getWANonSepTrafoIntraMode(int wideAngPredMode)
{
  uint32_t intraMode;
  if (wideAngPredMode < 0)
  {
    intraMode = (uint32_t)(wideAngPredMode + (NUM_EXT_LUMA_MODE >> 1) + NUM_LUMA_MODE);
  }
  else if (wideAngPredMode >= NUM_LUMA_MODE)
  {
    intraMode = (uint32_t)(wideAngPredMode + (NUM_EXT_LUMA_MODE >> 1));
  }
  else
  {
    intraMode = (uint32_t)wideAngPredMode;
  }
  return intraMode;
}

bool TU::isNSPTAllowed(int width, int height)
{
  bool allowNSPT = ((width == 4 && height == 4) || (width == 8 && height == 8) || (width == 4 && height == 8) ||
                    (width == 8 && height == 4) || (width == 4 && height == 16) || (width == 16 && height == 4) ||
                    (width == 8 && height == 16) || (width == 16 && height == 8) || (width == 4 && height == 32) ||
                    (width == 32 && height == 4) || (width == 8 && height == 32) || (width == 32 && height == 8));
  return allowNSPT;
}

bool TU::nsptApplyCond(const TransformUnit &tu, CompID compID, bool allowNSPT)
{
  bool cond = allowNSPT && TU::getNstIdx(tu, compID) > 0 && (CS::isDualITree(*tu.cs) ? true : isLuma(compID));

  return cond;
}

bool PU::addMergeHmvpCand(const CodingStructure &cs, MergeCtx &mrgCtx, const int &mrgCandIdx,
                          const uint32_t maxNumMergeCandMin1, int &cnt, const bool isAvailableA1,
                          const MotionInfo &miLeft, const bool isAvailableB1, const MotionInfo &miAbove,
                          const bool ibcFlag, const bool isGt4x4)
{
  const Slice &slice = *cs.slice;

  const auto &lut = ibcFlag ? cs.motionLut.lutIbc : cs.motionLut.lut;

  const int numAvailCandInLut = (int)lut.size();

  for (int mrgIdx = 0; mrgIdx < numAvailCandInLut; mrgIdx++)
  {
    const MotionInfo &miNeighbor = lut[numAvailCandInLut - 1 - mrgIdx];

    if (mrgIdx > 1 || ((mrgIdx > 0 || !isGt4x4) && ibcFlag) ||
        ((!isAvailableA1 || miLeft != miNeighbor) && (!isAvailableB1 || miAbove != miNeighbor)))
    {
      mrgCtx.LICFlags[cnt] = miNeighbor.usesLIC;

      if (ibcFlag)
      {
        CHECK(mrgCtx.LICFlags[cnt], "addMergeHMVPCand: LIC is not used with IBC mode")
      }
      mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
      mrgCtx.useAltHpelIf[cnt]       = miNeighbor.useAltHpelIf;
      mrgCtx.bcwIdx[cnt]             = miNeighbor.interDir == 3 ? miNeighbor.bcwIdx : BCW_DEFAULT;

      for (const auto l: { RPL0, RPL1 })
      {
        if (l != RPL0 && !slice.isInterB())
        {
          continue;
        }
        mrgCtx.mvFieldNeighbours[cnt][l].setMvField(miNeighbor.mv[l], miNeighbor.refIdx[l]);
      }
      if (mrgCtx.checkSimilarMotion(cnt))
      {
        continue;
      }

      if (mrgCandIdx == cnt)
      {
        return true;
      }

      if (++cnt == maxNumMergeCandMin1)
      {
        break;
      }
    }
  }

  if (cnt < maxNumMergeCandMin1)
  {
    mrgCtx.useAltHpelIf[cnt] = false;
  }

  return false;
}

void PU::getIBCMergeCandidates(const CodingUnit &cu, MergeCtx &mrgCtx, const int &mrgCandIdx)
{
  const CodingStructure &cs              = *cu.cs;
  const uint32_t         maxNumMergeCand = cu.cs->sps->m_maxNumIBCMergeCand;

  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    mrgCtx.bcwIdx[ui]                      = BCW_DEFAULT;
    mrgCtx.interDirNeighbours[ui]          = 0;
    mrgCtx.mvFieldNeighbours[ui][0].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[ui][1].refIdx = NOT_VALID;
    mrgCtx.useAltHpelIf[ui]                = false;
    mrgCtx.mvFieldNeighbours[ui][0].setMvField(Mv(), -1);
    mrgCtx.mvFieldNeighbours[ui][1].setMvField(Mv(), -1);
    mrgCtx.LICFlags[ui] = false;
  }

  mrgCtx.numValidMergeCand = maxNumMergeCand;
  // compute the location of the current PU

  int cnt = 0;

  const Position posRT = cu.Y().topRight();
  const Position posLB = cu.Y().bottomLeft();

  MotionInfo miAbove, miLeft, miAboveLeft, miAboveRight, miBelowLeft;

  // left
  const CodingUnit *puLeft        = cs.getCURestricted(posLB.offset(-1, 0), cu, cu.chType);
  bool              isGt4x4       = cu.lwidth() * cu.lheight() > 16;
  const bool        isAvailableA1 = puLeft && &cu != puLeft && CU::isIBC(*puLeft);
  if (isGt4x4 && isAvailableA1)
  {
    miLeft              = puLeft->getMotionInfo(posLB.offset(-1, 0));
    miLeft.useAltHpelIf = cu.mergeFlag ? miLeft.useAltHpelIf : (cu.imv == IMV_HPEL);

    // get Inter Dir
    mrgCtx.interDirNeighbours[cnt] = miLeft.interDir;
    // get Mv from Left
    mrgCtx.mvFieldNeighbours[cnt][0].setMvField(miLeft.mv[0], miLeft.refIdx[0]);
    mrgCtx.useAltHpelIf[cnt] = miLeft.useAltHpelIf;
    if (mrgCandIdx == cnt)
    {
      return;
    }
    cnt++;
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  // above
  const CodingUnit *puAbove       = cs.getCURestricted(posRT.offset(0, -1), cu, cu.chType);
  bool              isAvailableB1 = puAbove && &cu != puAbove && CU::isIBC(*puAbove);
  if (isGt4x4 && isAvailableB1)
  {
    miAbove              = puAbove->getMotionInfo(posRT.offset(0, -1));
    miAbove.useAltHpelIf = cu.mergeFlag ? miAbove.useAltHpelIf : (cu.imv == IMV_HPEL);

    if (!isAvailableA1 || (miAbove != miLeft))
    {
      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miAbove.interDir;
      // get Mv from Above
      mrgCtx.mvFieldNeighbours[cnt][0].setMvField(miAbove.mv[0], miAbove.refIdx[0]);
      mrgCtx.useAltHpelIf[cnt] = miAbove.useAltHpelIf;
      if (mrgCandIdx == cnt)
      {
        return;
      }

      cnt++;
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  if (cnt != maxNumMergeCand)
  {
    bool found = addMergeHmvpCand(cs, mrgCtx, mrgCandIdx, maxNumMergeCand, cnt, isAvailableA1, miLeft, isAvailableB1,
                                  miAbove, true, isGt4x4);

    if (found)
    {
      return;
    }
  }

  while (cnt < maxNumMergeCand)
  {
    mrgCtx.mvFieldNeighbours[cnt][0].setMvField(Mv(0, 0), IBC_REF_IDX);
    mrgCtx.interDirNeighbours[cnt] = 1;
    if (mrgCandIdx == cnt)
    {
      return;
    }
    cnt++;
  }

  mrgCtx.numValidMergeCand = cnt;
}

uint32_t PU::getBiGpmThreshold(const CodingUnit &cu)
{
  uint32_t numPixels = cu.lwidth() * cu.lheight();

  if (numPixels < 256)
  {
    return 1;
  }
  else
  {
    return (1 << MV_FRACTIONAL_BITS_INTERNAL);
  }
}

void PU::getInterMergeCandidates(const CodingUnit &cu, MergeCtx &mrgCtx, int mmvdList, const int &mrgCandIdx,
                                 int simThr)
{
  const unsigned         plevel          = cu.cs->sps->m_log2ParallelMergeLevelMinus2 + 2;
  const CodingStructure &cs              = *cu.cs;
  const Slice           &slice           = *cu.cs->slice;
  const uint32_t         maxNumMergeCand = cu.cs->sps->m_maxNumMergeCand;
  CHECK(maxNumMergeCand > MRG_MAX_NUM_CANDS, "selected maximum number of merge candidate exceeds global limit");
  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    mrgCtx.bcwIdx[ui]                      = BCW_DEFAULT;
    mrgCtx.interDirNeighbours[ui]          = 0;
    mrgCtx.mvFieldNeighbours[ui][0].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[ui][1].refIdx = NOT_VALID;
    mrgCtx.useAltHpelIf[ui]                = false;
    mrgCtx.LICFlags[ui]                    = false;
  }

  mrgCtx.numValidMergeCand = maxNumMergeCand;
  mrgCtx.numCandToTestEnc  = maxNumMergeCand;

  int  blkSizeThres = 128;
  bool useAdditionalCMVP =
    cu.cs->sps->m_useAdditionalCMVP && (cu.lumaSize().width + cu.lumaSize().height < blkSizeThres);
  uint32_t numCmvpCands = useAdditionalCMVP ? NUM_CMVP_CANDS : 0;
  // compute the location of the current PU

  int cnt = 0;

  const Position posLT = cu.Y().topLeft();
  const Position posRT = cu.Y().topRight();
  const Position posLB = cu.Y().bottomLeft();
  MotionInfo     miAbove, miLeft, miAboveLeft, miAboveRight, miBelowLeft;

  // above
  const CodingUnit *puAbove = cs.getCURestricted(posRT.offset(0, -1), cu, cu.chType);

  bool isAvailableB1 = puAbove && isDiffMER(cu.lumaPos(), posRT.offset(0, -1), plevel) && &cu != puAbove &&
    CU::isInter(*puAbove) && puAbove->getMotionInfo(posRT.offset(0, -1)).isInter;
  if (isAvailableB1)
  {
    miAbove = puAbove->getMotionInfo(posRT.offset(0, -1));

    // get Inter Dir
    mrgCtx.interDirNeighbours[cnt] = miAbove.interDir;
    mrgCtx.useAltHpelIf[cnt]       = miAbove.useAltHpelIf;
    // get Mv from Above
    mrgCtx.bcwIdx[cnt]             = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAbove->bcwIdx : BCW_DEFAULT;
    CHECK(miAbove.usesLIC != puAbove->licFlag, "LIC flag mismatch");
    mrgCtx.LICFlags[cnt] = miAbove.usesLIC;

    for (const auto l: { RPL0, RPL1 })
    {
      if (l != RPL0 && !slice.isInterB())
      {
        continue;
      }

      mrgCtx.mvFieldNeighbours[cnt][l].setMvField(miAbove.mv[l], miAbove.refIdx[l]);
    }
    if (!mrgCtx.checkSimilarMotion(cnt, simThr))
    {
      if (mrgCandIdx == cnt)
      {
        return;
      }

      cnt++;
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  // left
  const CodingUnit *puLeft = cs.getCURestricted(posLB.offset(-1, 0), cu, cu.chType);

  const bool isAvailableA1 = puLeft && isDiffMER(cu.lumaPos(), posLB.offset(-1, 0), plevel) && &cu != puLeft &&
    CU::isInter(*puLeft) && puLeft->getMotionInfo(posLB.offset(-1, 0)).isInter;

  if (isAvailableA1)
  {
    miLeft = puLeft->getMotionInfo(posLB.offset(-1, 0));

    if (!isAvailableB1 || (miAbove != miLeft))
    {
      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miLeft.interDir;
      mrgCtx.useAltHpelIf[cnt]       = miLeft.useAltHpelIf;
      mrgCtx.bcwIdx[cnt]             = (mrgCtx.interDirNeighbours[cnt] == 3) ? puLeft->bcwIdx : BCW_DEFAULT;
      CHECK(miLeft.usesLIC != puLeft->licFlag, "LIC flag mismatch");
      mrgCtx.LICFlags[cnt] = miLeft.usesLIC;
      // get Mv from Left
      mrgCtx.mvFieldNeighbours[cnt][0].setMvField(miLeft.mv[0], miLeft.refIdx[0]);

      if (slice.isInterB())
      {
        mrgCtx.mvFieldNeighbours[cnt][1].setMvField(miLeft.mv[1], miLeft.refIdx[1]);
      }
      if (!mrgCtx.checkSimilarMotion(cnt, simThr))
      {
        if (mrgCandIdx == cnt)
        {
          return;
        }

        cnt++;
      }
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  // above right
  const CodingUnit *puAboveRight = cs.getCURestricted(posRT.offset(1, -1), cu, cu.chType);

  bool isAvailableB0 = puAboveRight && isDiffMER(cu.lumaPos(), posRT.offset(1, -1), plevel) &&
    CU::isInter(*puAboveRight) && puAboveRight->getMotionInfo(posRT.offset(1, -1)).isInter;
  if (isAvailableB0)
  {
    miAboveRight = puAboveRight->getMotionInfo(posRT.offset(1, -1));

    if (!isAvailableB1 || (miAbove != miAboveRight))
    {

      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miAboveRight.interDir;
      mrgCtx.useAltHpelIf[cnt]       = miAboveRight.useAltHpelIf;
      // get Mv from Above-right
      mrgCtx.bcwIdx[cnt]             = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAboveRight->bcwIdx : BCW_DEFAULT;
      CHECK(miAboveRight.usesLIC != puAboveRight->licFlag, "LIC flag mismatch");
      mrgCtx.LICFlags[cnt] = miAboveRight.usesLIC;
      mrgCtx.mvFieldNeighbours[cnt][0].setMvField(miAboveRight.mv[0], miAboveRight.refIdx[0]);

      if (slice.isInterB())
      {
        mrgCtx.mvFieldNeighbours[cnt][1].setMvField(miAboveRight.mv[1], miAboveRight.refIdx[1]);
      }

      if (!mrgCtx.checkSimilarMotion(cnt, simThr))
      {
        if (mrgCandIdx == cnt)
        {
          return;
        }

        cnt++;
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  // left bottom
  const CodingUnit *puLeftBottom = cs.getCURestricted(posLB.offset(-1, 1), cu, cu.chType);

  bool isAvailableA0 = puLeftBottom && isDiffMER(cu.lumaPos(), posLB.offset(-1, 1), plevel) &&
    CU::isInter(*puLeftBottom) && puLeftBottom->getMotionInfo(posLB.offset(-1, 1)).isInter;

  if (isAvailableA0)
  {
    miBelowLeft = puLeftBottom->getMotionInfo(posLB.offset(-1, 1));

    if (!isAvailableA1 || (miBelowLeft != miLeft))
    {
      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miBelowLeft.interDir;
      mrgCtx.useAltHpelIf[cnt]       = miBelowLeft.useAltHpelIf;
      mrgCtx.bcwIdx[cnt]             = (mrgCtx.interDirNeighbours[cnt] == 3) ? puLeftBottom->bcwIdx : BCW_DEFAULT;
      CHECK(miBelowLeft.usesLIC != puLeftBottom->licFlag, "LIC flag mismatch");
      mrgCtx.LICFlags[cnt] = miBelowLeft.usesLIC;
      // get Mv from Bottom-Left
      mrgCtx.mvFieldNeighbours[cnt][0].setMvField(miBelowLeft.mv[0], miBelowLeft.refIdx[0]);

      if (slice.isInterB())
      {
        mrgCtx.mvFieldNeighbours[cnt][1].setMvField(miBelowLeft.mv[1], miBelowLeft.refIdx[1]);
      }

      if (!mrgCtx.checkSimilarMotion(cnt, simThr))
      {
        if (mrgCandIdx == cnt)
        {
          return;
        }

        cnt++;
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  // above left
  if (cnt < 4)
  {
    const CodingUnit *puAboveLeft = cs.getCURestricted(posLT.offset(-1, -1), cu, cu.chType);

    bool isAvailableB2 = puAboveLeft && isDiffMER(cu.lumaPos(), posLT.offset(-1, -1), plevel) &&
      CU::isInter(*puAboveLeft) && puAboveLeft->getMotionInfo(posLT.offset(-1, -1)).isInter;
    if (isAvailableB2)
    {
      miAboveLeft = puAboveLeft->getMotionInfo(posLT.offset(-1, -1));

      if ((!isAvailableA1 || (miLeft != miAboveLeft)) && (!isAvailableB1 || (miAbove != miAboveLeft)))
      {
        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miAboveLeft.interDir;
        mrgCtx.useAltHpelIf[cnt]       = miAboveLeft.useAltHpelIf;
        mrgCtx.bcwIdx[cnt]             = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAboveLeft->bcwIdx : BCW_DEFAULT;
        CHECK(miAboveLeft.usesLIC != puAboveLeft->licFlag, "LIC flag mismatch");
        mrgCtx.LICFlags[cnt] = miAboveLeft.usesLIC;
        // get Mv from Above-Left
        mrgCtx.mvFieldNeighbours[cnt][0].setMvField(miAboveLeft.mv[0], miAboveLeft.refIdx[0]);

        if (slice.isInterB())
        {
          mrgCtx.mvFieldNeighbours[cnt][1].setMvField(miAboveLeft.mv[1], miAboveLeft.refIdx[1]);
        }

        if (!mrgCtx.checkSimilarMotion(cnt, simThr))
        {
          if (mrgCandIdx == cnt)
          {
            return;
          }

          cnt++;
        }
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  if (slice.m_picHeader->m_enableTMVPFlag)
  {
    //>> MTK colocated-RightBottom
    // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to
    Position             posRB = cu.Y().bottomRight().offset(-3, -3);
    const PreCalcValues &pcv   = *cs.pcv;

    Position posC0;
    Position posC1    = cu.Y().center();
    bool     C0Avail  = false;
    bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
    const SubPic &curSubPic = cu.cs->slice->m_pps->getSubPicFromPos(cu.lumaPos());
    if (curSubPic.m_treatedAsPicFlag)
    {
      boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.m_subPicRight &&
                      (posRB.y + pcv.minCUHeight) <= curSubPic.m_subPicBottom);
    }
    if (boundaryCond)
    {
      int posYInCtu = posRB.y & pcv.maxCUHeightMask;
      if (posYInCtu + 4 < pcv.maxCUHeight)
      {
        posC0   = posRB.offset(4, 4);
        C0Avail = true;
      }
    }

    Mv       cColMv;
    int      refIdx    = 0;
    int      dir       = 0;
    unsigned arrayAddr = cnt;
    bool     existMV   = (C0Avail && getColocatedMVP(cu, RPL0, posC0, cColMv, refIdx, false)) ||
      getColocatedMVP(cu, RPL0, posC1, cColMv, refIdx, false);
    if (existMV)
    {
      dir |= 1;
      mrgCtx.mvFieldNeighbours[arrayAddr][0].setMvField(cColMv, refIdx);
    }

    if (slice.isInterB())
    {
      existMV = (C0Avail && getColocatedMVP(cu, RPL1, posC0, cColMv, refIdx, false)) ||
        getColocatedMVP(cu, RPL1, posC1, cColMv, refIdx, false);
      if (existMV)
      {
        dir |= 2;
        mrgCtx.mvFieldNeighbours[arrayAddr][1].setMvField(cColMv, refIdx);
      }
    }

    if (dir != 0)
    {
      bool addTMvp = true;
      if (addTMvp)
      {
        mrgCtx.interDirNeighbours[arrayAddr] = dir;
        mrgCtx.bcwIdx[arrayAddr]             = BCW_DEFAULT;
        mrgCtx.LICFlags[arrayAddr]           = false;
        mrgCtx.useAltHpelIf[arrayAddr]       = false;
        if (!mrgCtx.checkSimilarMotion(cnt, simThr))
        {
          if (mrgCandIdx == cnt)
          {
            return;
          }

          cnt++;
        }
      }
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  MotionInfo miNeighbor;
  int        offsetX           = 0;
  int        offsetY           = 0;
  const int  numNACandidate[4] = { 3, 5, 5, 5 };
  const int  idxMap[4][5]      = { { 0, 1, 4 }, { 0, 1, 2, 3, 4 }, { 0, 1, 2, 3, 4 }, { 0, 1, 2, 3, 4 } };
  for (int iDistanceIndex = 0; iDistanceIndex < NADISTANCE_LEVEL && cnt < maxNumMergeCand - numCmvpCands - 1;
       iDistanceIndex++)
  {
    const int iNADistanceHor = cu.lwidth() * (iDistanceIndex + 1);
    const int iNADistanceVer = cu.lheight() * (iDistanceIndex + 1);
    for (int iNASPIdx = 0; iNASPIdx < numNACandidate[iDistanceIndex] && cnt < maxNumMergeCand - numCmvpCands - 1;
         iNASPIdx++)
    {
      switch (idxMap[iDistanceIndex][iNASPIdx])
      {
      case 0:
        offsetX = -iNADistanceHor - 1;
        offsetY = cu.lheight() + iNADistanceVer - 1;
        break;
      case 1:
        offsetX = cu.lwidth() + iNADistanceHor - 1;
        offsetY = -iNADistanceVer - 1;
        break;
      case 2:
        offsetX = cu.lwidth() >> 1;
        offsetY = -iNADistanceVer - 1;
        break;
      case 3:
        offsetX = -iNADistanceHor - 1;
        offsetY = cu.lheight() >> 1;
        break;
      case 4:
        offsetX = -iNADistanceHor - 1;
        offsetY = -iNADistanceVer - 1;
        break;
      default:
        printf("error!");
        exit(0);
        break;
      }

      const CodingUnit *puNonAdjacent = cs.getCURestricted(posLT.offset(offsetX, offsetY), cu, cu.chType);

      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(cu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) &&
        CU::isInter(*puNonAdjacent) && puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY)).isInter;
      if (isAvailableNonAdjacent)
      {
        miNeighbor = puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY));

        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
        // get Mv from Above-Left
        mrgCtx.mvFieldNeighbours[cnt][0].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
        mrgCtx.useAltHpelIf[cnt] = miNeighbor.useAltHpelIf;
        // get Mv from Above-right
        mrgCtx.bcwIdx[cnt]       = (mrgCtx.interDirNeighbours[cnt] == 3) ? puNonAdjacent->bcwIdx : BCW_DEFAULT;
        CHECK(miNeighbor.usesLIC != puNonAdjacent->licFlag, "LIC flag mismatch");
        mrgCtx.LICFlags[cnt] = miNeighbor.usesLIC;

        if (slice.isInterB())
        {
          mrgCtx.mvFieldNeighbours[cnt][1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);
        }

        if (!mrgCtx.checkSimilarMotion(cnt, simThr))
        {
          if (mrgCandIdx == cnt)
          {
            return;
          }
          cnt++;
        }
      }
    }
  }

  int maxNumMergeCandMin1 = maxNumMergeCand - 1;

  if (cnt < (maxNumMergeCandMin1 - numCmvpCands))
  {
    bool isGt4x4 = true;
    bool found   = addMergeHmvpCand(cs, mrgCtx, mrgCandIdx, maxNumMergeCandMin1 - numCmvpCands, cnt, isAvailableA1,
                                    miLeft, isAvailableB1, miAbove, CU::isIBC(cu), isGt4x4);

    if (found)
    {
      return;
    }
  }
  int checkNum = cnt;
  if (cnt < maxNumMergeCandMin1)
  {
    if (slice.m_picHeader->m_enableTMVPFlag)
    {
      const unsigned scale      = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
      const unsigned mask       = ~(scale - 1);
      const Position basePos[5] = { cu.Y().center(), cu.Y().topLeft(), cu.Y().topRight(), cu.Y().bottomLeft(),
                                    cu.Y().bottomRight() };
      const SubPic  &curSubPic  = cu.cs->pps->getSubPicFromPos(cu.lumaPos());

      for (int mergeIndex = 0; mergeIndex < checkNum && cnt < maxNumMergeCandMin1; mergeIndex++)
      {
        for (int list = 0; list < NUM_RPL01 && cnt < maxNumMergeCandMin1; list++)
        {
          if (!(mrgCtx.interDirNeighbours[mergeIndex] & (1 << list)))
          {
            continue;
          }

          MvField &mvField = mrgCtx.mvFieldNeighbours[mergeIndex][list];
          Mv       cMv     = mvField.mv;
          cMv.changePrecision(MvPrecision::SIXTEENTH, MvPrecision::ONE);
          const Picture *const pColPic = slice.getRefPic(RefPicList(list), mvField.refIdx);

          if (!pColPic || pColPic->isRefScaled(cu.cs->pps))
          {
            continue;
          }

          for (const auto &pos: basePos)
          {
            Position posCand(pos.x + cMv.getHor(), pos.y + cMv.getVer());
            posCand.x = Clip3(0, (int)pColPic->getPicWidthInLumaSamples() - 1, posCand.x);
            posCand.y = Clip3(0, (int)pColPic->getPicHeightInLumaSamples() - 1, posCand.y);
            posCand.x = posCand.x & mask;
            posCand.y = posCand.y & mask;

            // Check the position of colocated block is within a subpicture
            if (curSubPic.m_treatedAsPicFlag)
            {
              if (!curSubPic.isContainingPos(posCand))
              {
                continue;
              }
            }

            const MotionInfo &miCascaded = pColPic->m_cs->getMotionInfo(posCand);
            if (!miCascaded.isInter)   // TODO || (miCascaded.isIBCmot && miCascaded.rribcFlipType))
            {
              continue;
            }
            int    interDir                 = 0;
            int8_t chainedRefIdx[NUM_RPL01] = { -1, -1 };
            Mv     chainedMv[NUM_RPL01];
            chainedMv[RPL0].setZero();
            chainedMv[RPL1].setZero();

            const Slice *pColSlice = nullptr;
            for (const auto s: pColPic->m_slices)
            {
              if (s->m_independentSliceIdx == miCascaded.sliceIdx)
              {
                pColSlice = s;
                break;
              }
            }
            CHECK(pColSlice == nullptr, "Slice segment not found");

            for (int colList = 0; colList < 2; colList++)
            {
              if (miCascaded.interDir & (1 << colList))
              {
                const int *refRefIdxList =
                  slice.getRefRefIdx(RefPicList(list), mvField.refIdx, RefPicList(colList), miCascaded.refIdx[colList]);
                int    curList   = -1;
                int8_t curRefIdx = -1;
                if (miCascaded.isIBCmot)
                {
                  if (chainedRefIdx[list] < 0)
                  {
                    curList   = list;
                    curRefIdx = mvField.refIdx;
                  }
                  else   // BiPredictive IBC
                  {
                    int l     = 1 - list;
                    curList   = l;
                    curRefIdx = refRefIdxList[l];
                  }
                }
                else
                {
                  for (int l = 0; l < (slice.isInterB() ? 2 : 1) && curRefIdx < 0; l++)
                  {
                    if (interDir & (1 << l))
                    {
                      continue;
                    }
                    curList   = l;
                    curRefIdx = refRefIdxList[l];
                  }
                }
                if (curRefIdx >= 0)
                {
                  CHECK(interDir & (1 << curList), "Already use list");
                  interDir |= 1 << curList;
                  chainedRefIdx[curList] = curRefIdx;
                  chainedMv[curList]     = mvField.mv + miCascaded.mv[colList];
                  if (!slice.isInterB())
                  {
                    break;
                  }
                }
              }
            }
            if (interDir == 0)
            {
              continue;
            }

            mrgCtx.interDirNeighbours[cnt] = interDir;
            mrgCtx.mvFieldNeighbours[cnt][0].setMvField(chainedMv[0], chainedRefIdx[0]);
            if (slice.isInterB())
            {
              mrgCtx.mvFieldNeighbours[cnt][1].setMvField(chainedMv[1], chainedRefIdx[1]);
            }

            mrgCtx.useAltHpelIf[cnt] = false;
            mrgCtx.LICFlags[cnt]     = false;
            mrgCtx.bcwIdx[cnt]       = BCW_DEFAULT;
#if JVET_AC0112_IBC_LIC   // TODO
            mrgCtx.ibcLicFlags[cnt] = false;
#endif
#if JVET_AE0159_FIBC   // TODO
            mrgCtx.ibcFilterFlags[cnt] = false;
#endif

            if (!mrgCtx.checkSimilarMotion(cnt, simThr))
            {
              if (mrgCandIdx == cnt)
              {
                return;
              }
              cnt++;
              if (cnt == maxNumMergeCandMin1)
              {
                break;
              }
            }
            else
            {
              mrgCtx.initMrgCand(cnt);
            }
          }
        }
      }
    }
  }
  // pairwise-average candidates
  {
    if (cnt > 1 && cnt < maxNumMergeCand)
    {
      mrgCtx.mvFieldNeighbours[cnt][0].setMvField(Mv(0, 0), NOT_VALID);
      mrgCtx.mvFieldNeighbours[cnt][1].setMvField(Mv(0, 0), NOT_VALID);
      mrgCtx.LICFlags[cnt] = false;
      // calculate average MV for L0 and L1 seperately
      uint8_t interDir     = 0;
      bool    averageUsed  = false;

      mrgCtx.useAltHpelIf[cnt] = (mrgCtx.useAltHpelIf[0] == mrgCtx.useAltHpelIf[1]) ? mrgCtx.useAltHpelIf[0] : false;
      for (int refListId = 0; refListId < (slice.isInterB() ? 2 : 1); refListId++)
      {
        const short refIdxI = mrgCtx.mvFieldNeighbours[0][refListId].refIdx;
        const short refIdxJ = mrgCtx.mvFieldNeighbours[1][refListId].refIdx;

        // both MVs are invalid, skip
        if ((refIdxI == NOT_VALID) && (refIdxJ == NOT_VALID))
        {
          continue;
        }

        interDir += 1 << refListId;
        // both MVs are valid, average these two MVs
        if ((refIdxI != NOT_VALID) && (refIdxJ != NOT_VALID))
        {
          const Mv &mvI = mrgCtx.mvFieldNeighbours[0][refListId].mv;
          const Mv &mvJ = mrgCtx.mvFieldNeighbours[1][refListId].mv;

          // average two MVs
          Mv avgMv = mvI;
          avgMv += mvJ;
          avgMv >>= 1;

          mrgCtx.mvFieldNeighbours[cnt][refListId].setMvField(avgMv, refIdxI);
          mrgCtx.LICFlags[cnt] = false;
          averageUsed          = true;
        }
        // only one MV is valid, take the only one MV
        else if (refIdxI != NOT_VALID)
        {
          Mv singleMv = mrgCtx.mvFieldNeighbours[0][refListId].mv;
          mrgCtx.mvFieldNeighbours[cnt][refListId].setMvField(singleMv, refIdxI);
          if (!averageUsed)
          {
            if (interDir == 3)
            {
              mrgCtx.LICFlags[cnt] &= mrgCtx.LICFlags[0];
            }
            else
            {
              mrgCtx.LICFlags[cnt] |= mrgCtx.LICFlags[0];
            }
          }
        }
        else if (refIdxJ != NOT_VALID)
        {
          Mv singleMv = mrgCtx.mvFieldNeighbours[1][refListId].mv;
          mrgCtx.mvFieldNeighbours[cnt][refListId].setMvField(singleMv, refIdxJ);
          if (!averageUsed)
          {
            if (interDir == 3)
            {
              mrgCtx.LICFlags[cnt] &= mrgCtx.LICFlags[1];
            }
            else
            {
              mrgCtx.LICFlags[cnt] |= mrgCtx.LICFlags[1];
            }
          }
        }
      }

      mrgCtx.interDirNeighbours[cnt] = interDir;
      if (interDir > 0)
      {
        if (!cu.cs->sps->m_biLicEnabledFlag && interDir == 3)
        {
          mrgCtx.LICFlags[cnt] = false;
        }
        if (!mrgCtx.checkSimilarMotion(cnt, simThr))
        {
          cnt++;
        }
      }
    }

    // early termination
    if (cnt == maxNumMergeCand)
    {
      return;
    }
  }

  if (addMergeHMVPCandFromAffModel(cu, mrgCtx, mrgCandIdx, cnt))
  {
    return;
  }

  if (cnt < maxNumMergeCand)
  {
    if (slice.m_picHeader->m_enableTMVPFlag && useAdditionalCMVP)
    {
      const unsigned scale     = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
      const unsigned mask      = ~(scale - 1);
      const Position basePos   = cu.Y().center();
      const SubPic  &curSubPic = cu.cs->pps->getSubPicFromPos(cu.lumaPos());
      int            cntEnd    = cnt;
      for (int mergeIndex = checkNum; mergeIndex < cntEnd && cnt < maxNumMergeCand; mergeIndex++)
      {
        for (int list = 0; list < NUM_RPL01 && cnt < maxNumMergeCand; list++)
        {
          if (!(mrgCtx.interDirNeighbours[mergeIndex] & (1 << list)))
          {
            continue;
          }

          MvField &mvField = mrgCtx.mvFieldNeighbours[mergeIndex][list];
          Mv       cMv     = mvField.mv;
          cMv.changePrecision(MvPrecision::SIXTEENTH, MvPrecision::ONE);
          const Picture *const pColPic = slice.getRefPic(RefPicList(list), mvField.refIdx);

          if (!pColPic || pColPic->isRefScaled(cu.cs->pps))
          {
            continue;
          }

          Position posCand(basePos.x + cMv.getHor(), basePos.y + cMv.getVer());
          posCand.x = Clip3(0, (int)pColPic->getPicWidthInLumaSamples() - 1, posCand.x);
          posCand.y = Clip3(0, (int)pColPic->getPicHeightInLumaSamples() - 1, posCand.y);
          posCand.x = posCand.x & mask;
          posCand.y = posCand.y & mask;

          // Check the position of colocated block is within a subpicture
          if (curSubPic.m_treatedAsPicFlag)
          {
            if (!curSubPic.isContainingPos(posCand))
            {
              continue;
            }
          }

          const MotionInfo &miCascaded = pColPic->m_cs->getMotionInfo(posCand);
          if (!miCascaded.isInter)   //|| (miCascaded.isIBCmot && miCascaded.rribcFlipType))
          {
            continue;
          }
          int    interDir                 = 0;
          int8_t chainedRefIdx[NUM_RPL01] = { -1, -1 };
          Mv     chainedMv[NUM_RPL01];
          chainedMv[RPL0].setZero();
          chainedMv[RPL1].setZero();

          const Slice *pColSlice = nullptr;
          for (const auto s: pColPic->m_slices)
          {
            if (s->m_independentSliceIdx == miCascaded.sliceIdx)
            {
              pColSlice = s;
              break;
            }
          }
          CHECK(pColSlice == nullptr, "Slice segment not found");

          for (int colList = 0; colList < 2; colList++)
          {
            if (miCascaded.interDir & (1 << colList))
            {
              const int *refRefIdxList =
                slice.getRefRefIdx(RefPicList(list), mvField.refIdx, RefPicList(colList), miCascaded.refIdx[colList]);
              int    curList   = -1;
              int8_t curRefIdx = -1;
              if (miCascaded.isIBCmot)
              {
                if (chainedRefIdx[list] < 0)
                {
                  curList   = list;
                  curRefIdx = mvField.refIdx;
                }
                else   // BiPredictive IBC
                {
                  int l     = 1 - list;
                  curList   = l;
                  curRefIdx = refRefIdxList[l];
                }
              }
              else
              {
                for (int l = 0; l < (slice.isInterB() ? 2 : 1) && curRefIdx < 0; l++)
                {
                  if (interDir & (1 << l))
                  {
                    continue;
                  }
                  curList   = l;
                  curRefIdx = refRefIdxList[l];
                }
              }
              if (curRefIdx >= 0)
              {
                CHECK(interDir & (1 << curList), "Already use list");
                interDir |= 1 << curList;
                chainedRefIdx[curList] = curRefIdx;
                chainedMv[curList]     = mvField.mv + miCascaded.mv[colList];
                if (!slice.isInterB())
                {
                  break;
                }
              }
            }
          }
          if (interDir == 0)
          {
            continue;
          }

          mrgCtx.interDirNeighbours[cnt] = interDir;
          mrgCtx.mvFieldNeighbours[cnt][0].setMvField(chainedMv[0], chainedRefIdx[0]);
          if (slice.isInterB())
          {
            mrgCtx.mvFieldNeighbours[cnt][1].setMvField(chainedMv[1], chainedRefIdx[1]);
          }

          mrgCtx.useAltHpelIf[cnt] = false;
          mrgCtx.LICFlags[cnt]     = false;
          mrgCtx.bcwIdx[cnt]       = BCW_DEFAULT;
#if JVET_AA0070_RRIBC   // TO DO
          mrgCtx.rribcFlipTypes[cnt] = 0;
#endif
#if JVET_AC0112_IBC_LIC   // TO DO
          mrgCtx.ibcLicFlags[cnt] = false;
#endif
#if JVET_AE0159_FIBC   // TO DO
          mrgCtx.ibcFilterFlags[cnt] = false;
#endif

          if (!mrgCtx.checkSimilarMotion(cnt, simThr))
          {
            if (mrgCandIdx == cnt)
            {
              return;
            }
            cnt++;
            if (cnt == maxNumMergeCand)
            {
              break;
            }
          }
          else
          {
            mrgCtx.initMrgCand(cnt);
          }
        }
      }
    }
  }

  int arrayAddr = cnt;

  int numRefIdx =
    slice.isInterB() ? std::min(slice.m_numRefIdx[RPL0], slice.m_numRefIdx[RPL1]) : slice.m_numRefIdx[RPL0];

  int r      = 0;
  int refcnt = 0;
  while (arrayAddr < maxNumMergeCand)
  {
    mrgCtx.interDirNeighbours[arrayAddr] = 1;
    mrgCtx.bcwIdx[arrayAddr]             = BCW_DEFAULT;
    mrgCtx.LICFlags[arrayAddr]           = false;
    mrgCtx.mvFieldNeighbours[arrayAddr][0].setMvField(Mv(0, 0), r);
    mrgCtx.useAltHpelIf[arrayAddr] = false;

    if (slice.isInterB())
    {
      mrgCtx.interDirNeighbours[arrayAddr] = 3;
      mrgCtx.mvFieldNeighbours[arrayAddr][1].setMvField(Mv(0, 0), r);
    }

    arrayAddr++;

    if (refcnt == numRefIdx - 1)
    {
      r = 0;
    }
    else
    {
      ++r;
      ++refcnt;
    }
    if (arrayAddr < maxNumMergeCand && arrayAddr == cnt + 1)
    {
      if (addMergeHMVPCandFromAffModel(cu, mrgCtx, mrgCandIdx, arrayAddr))
      {
        return;
      }
      if (arrayAddr >= mrgCtx.numValidMergeCand)
      {
        return;
      }
    }
  }
  CHECK(mrgCtx.numValidMergeCand != arrayAddr, "not enough number of merge candidates!");
}

bool PU::isBiRefScaled(const Slice &slice, const int refIdx0, const int refIdx1)
{
  const bool isResamplingPossible = slice.m_sps->m_rprEnabledFlag;
  if (!isResamplingPossible)
  {
    return false;
  }

  const bool ref0IsScaled = refIdx0 < 0 || refIdx0 >= MAX_NUM_REF
    ? false
    : isResamplingPossible && slice.getRefPic(RPL0, refIdx0)->isRefScaled(slice.m_pps);
  const bool ref1IsScaled = refIdx1 < 0 || refIdx1 >= MAX_NUM_REF
    ? false
    : isResamplingPossible && slice.getRefPic(RPL1, refIdx1)->isRefScaled(slice.m_pps);

  return (ref0IsScaled || ref1IsScaled);
}

bool PU::isBiPredFromDifferentDirEqDistPoc(const CodingUnit &cu)
{
  return isBiPredFromDifferentDirEqDistPoc(*cu.cs->slice, cu.refIdx[0], cu.refIdx[1]);
}

bool PU::isBiPredFromDifferentDirEqDistPoc(const Slice &slice, int refIdx0, int refIdx1)
{
  if (refIdx0 >= 0 && refIdx1 >= 0)
  {
    if (PU::isBiRefScaled(slice, refIdx0, refIdx1))
    {
      return false;
    }

    if (slice.getRefPic(RPL0, refIdx0)->m_longTerm || slice.getRefPic(RPL1, refIdx1)->m_longTerm)
    {
      return false;
    }

    const int poc0 = slice.getRefPOC(RPL0, refIdx0);
    const int poc1 = slice.getRefPOC(RPL1, refIdx1);
    const int poc  = slice.m_poc;

    if ((poc - poc0) * (poc - poc1) < 0)
    {
      if (abs(poc - poc0) == abs(poc - poc1))
      {
        return true;
      }
    }
  }
  return false;
}

bool PU::isBiPredFromDifferentDirGenDistPoc(const CodingUnit &pu)
{
  if (pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0)
  {
    if (pu.slice->getRefPic(RPL0, pu.refIdx[0])->m_longTerm || pu.slice->getRefPic(RPL1, pu.refIdx[1])->m_longTerm)
    {
      return false;
    }
    if (PU::isBiRefScaled(*pu.slice, pu.refIdx[0], pu.refIdx[1]))
    {
      return false;
    }
    const int poc0 = pu.slice->getRefPOC(RPL0, pu.refIdx[0]);
    const int poc1 = pu.slice->getRefPOC(RPL1, pu.refIdx[1]);
    const int poc  = pu.slice->m_poc;
    if ((poc - poc0) * (poc - poc1) < 0)
    {
      {
        return true;
      }
    }
  }
  return false;
}

bool PU::checkBDMVR4Affine(const CodingUnit &pu)
{
  // Different conditions for square and rectangular shapes
  if (pu.lwidth() == pu.lheight())
  {
    const int base32 = ((uint32_t)(pu.cs->picture->lwidth() * pu.cs->picture->lheight() * 0.00003546 + 55)) >> 5;
    const int maxSquareCuSizeABDMVR = std::max(32, std::min(base32 << 5, 256));

    if (pu.lwidth() > maxSquareCuSizeABDMVR)
    {
      return false;
    }
  }
  else
  {
    const int maxRectCuSizebase32 =
      ((uint32_t)(pu.cs->picture->lwidth() * pu.cs->picture->lheight() * 0.00001722 + 95)) >> 5;
    const int maxRectCuSizeABDMVR = std::max(32, std::min(maxRectCuSizebase32 << 5, 128));
    const int maxRectCuSizeRatiobase32 =
      ((uint32_t)(pu.cs->picture->lwidth() * pu.cs->picture->lheight() * 0.00000090 + 15)) >> 3;
    const int maxRectCuSizeRatio = std::max(8, std::min(maxRectCuSizeRatiobase32 << 3, 16));
    uint32_t  maxLength          = std::max(pu.lwidth(), pu.lheight());
    uint32_t  minLength          = std::min(pu.lwidth(), pu.lheight());

    if (maxLength > maxRectCuSizeABDMVR)
    {
      return false;
    }
    if (maxLength / minLength >= maxRectCuSizeRatio)
    {
      return false;
    }
  }
  return true;
}

bool PU::checkBDMVRCpmvRefinementPuUsage(const CodingUnit &pu)
{
  // Different conditions for square and rectangular shapes
  if (pu.lwidth() == pu.lheight())
  {
    const int base32 = ((uint32_t)(pu.cs->picture->lwidth() * pu.cs->picture->lheight() * 0.000010558 + 60)) >> 5;
    const int maxSquareCuSizeABDMVR = std::max(32, std::min(base32 << 5, 128));

    if (pu.lwidth() > maxSquareCuSizeABDMVR)
    {
      return false;
    }
  }
  else
  {
    const int base32 = ((uint32_t)(pu.cs->picture->lwidth() * pu.cs->picture->lheight() * 0.0000100751 + 44)) >> 5;
    const int maxRectCuSizeABDMVR = std::max(32, std::min(base32 << 5, 128));
    const int maxRectCuSizeRatio  = 8;
    uint32_t  maxLength           = std::max(pu.lwidth(), pu.lheight());
    uint32_t  minLength           = std::min(pu.lwidth(), pu.lheight());

    if (maxLength > maxRectCuSizeABDMVR)
    {
      return false;
    }
    if (maxLength / minLength >= maxRectCuSizeRatio)
    {
      return false;
    }
  }
  return true;
}

bool PU::checkBDMVRCondition(const CodingUnit &cu)
{
  if (cu.cs->sps->m_useDMVD && CU::isInter(cu))
  {
    const int refIdx0 = cu.refIdx[RPL0];
    const int refIdx1 = cu.refIdx[RPL1];

    const WPScalingParam *wp0 = cu.slice->getWpScaling(RPL0, refIdx0);
    const WPScalingParam *wp1 = cu.slice->getWpScaling(RPL1, refIdx1);

    const bool isResamplingPossible = cu.cs->sps->m_rprEnabledFlag;

    const bool ref0IsScaled = refIdx0 < 0 || refIdx0 >= MAX_NUM_REF
      ? false
      : isResamplingPossible && cu.slice->getRefPic(RPL0, refIdx0)->isRefScaled(cu.cs->pps);
    const bool ref1IsScaled = refIdx1 < 0 || refIdx1 >= MAX_NUM_REF
      ? false
      : isResamplingPossible && cu.slice->getRefPic(RPL1, refIdx1)->isRefScaled(cu.cs->pps);

    return cu.mergeFlag && cu.mergeType == MergeType::DEFAULT_N && !cu.ciipFlag && !cu.mmvdMergeFlag && !cu.mmvdSkip &&
      (PU::isBiPredFromDifferentDirEqDistPoc(cu) ||
       (cu.cs->sps->m_dmvdBDOFExt && PU::isBiPredFromDifferentDirGenDistPoc(cu) && !cu.geoFlag)) &&
      cu.bcwIdx == BCW_DEFAULT && !cu.licFlag && !WPScalingParam::isWeighted(wp0) && !WPScalingParam::isWeighted(wp1) &&
      !ref0IsScaled && !ref1IsScaled;
  }
  else
  {
    return false;
  }
}

uint32_t PU::getBDMVRMvdThreshold(const CodingUnit &cu)
{
  const int numPixels = cu.Y().area();

  if (numPixels < 64)
  {
    return (1 << MV_FRACTIONAL_BITS_INTERNAL) >> 2;
  }
  else if (numPixels < 256)
  {
    return (1 << MV_FRACTIONAL_BITS_INTERNAL) >> 1;
  }
  else
  {
    return (1 << MV_FRACTIONAL_BITS_INTERNAL) >> 0;
  }
}

bool PU::isBIOApplied(const CodingUnit &cu, const Slice &slice, const PPS &pps, const bool subPuMC)
{
  int                   refIdx0 = cu.refIdx[RPL0];
  int                   refIdx1 = cu.refIdx[RPL1];
  const WPScalingParam *wp0     = cu.cs->slice->getWpScaling(RPL0, refIdx0);
  const WPScalingParam *wp1     = cu.cs->slice->getWpScaling(RPL1, refIdx1);

  bool bioApplied = false;
  if (cu.cs->sps->m_bdofEnabledFlag && !cu.cs->picHeader->m_bdofDisabledFlag)
  {
    if (cu.affine || subPuMC)
    {
      bioApplied = false;
    }
    else
    {
      const bool biocheck0 =
        !(slice.m_eSliceType == B_SLICE && (WPScalingParam::isWeighted(wp0) || WPScalingParam::isWeighted(wp1)));
      const bool biocheck1 = !(pps.m_useWP && slice.m_eSliceType == P_SLICE);

      if (biocheck0 && biocheck1 && PU::isBiPredFromDifferentDirEqDistPoc(cu))
      {
        bioApplied = true;
      }
    }

    if (bioApplied && cu.ciipFlag)
    {
      bioApplied = false;
    }

    if (bioApplied && cu.smvdMode)
    {
      bioApplied = false;
    }

    if (cu.cs->sps->m_useBcw && bioApplied && cu.bcwIdx != BCW_DEFAULT)
    {
      bioApplied = false;
    }

    if (bioApplied && cu.licFlag)
    {
      bioApplied = false;
    }
  }
  if (cu.mmvdEncOptMode == 2 && cu.mmvdMergeFlag)
  {
    bioApplied = false;
  }

  const bool isResamplingPossible = cu.cs->sps->m_rprEnabledFlag;
  const bool refIsScaled          = isResamplingPossible &&
    ((refIdx0 < 0 ? false : cu.slice->getRefPic(RPL0, refIdx0)->isRefScaled(cu.cs->pps)) ||
     (refIdx1 < 0 ? false : cu.slice->getRefPic(RPL1, refIdx1)->isRefScaled(cu.cs->pps)));
  bioApplied = bioApplied && !refIsScaled;
  return bioApplied;
}

static int xGetDistScaleFactor(const int &currPoc, const int &currRefPoc, const int &colPoc, const int &colRefPoc)
{
  const int diffPocD = colPoc - colRefPoc;
  const int diffPocB = currPoc - currRefPoc;

  if (diffPocD == diffPocB)
  {
    return 4096;
  }
  else
  {
    const int tdB   = Clip3(-128, 127, diffPocB);
    const int tdD   = Clip3(-128, 127, diffPocD);
    const int x     = (0x4000 + abs(tdD / 2)) / tdD;
    const int scale = Clip3(-4096, 4095, (tdB * x + 32) >> 6);
    return scale;
  }
}

int convertMvFixedToFloat(int32_t val)
{
  const int sign  = val >> 31;
  const int scale = floorLog2((val ^ sign) | MV_MANTISSA_UPPER_LIMIT) - (MV_MANTISSA_BITCOUNT - 1);

  int exponent;
  int mantissa;
  if (scale >= 0)
  {
    int round = (1 << scale) >> 1;
    int n     = (val + round) >> scale;
    exponent  = scale + ((n ^ sign) >> (MV_MANTISSA_BITCOUNT - 1));
    mantissa  = (n & MV_MANTISSA_UPPER_LIMIT) | (sign * (1 << (MV_MANTISSA_BITCOUNT - 1)));
  }
  else
  {
    exponent = 0;
    mantissa = val;
  }

  return exponent | (mantissa * (1 << MV_EXPONENT_BITCOUNT));
}

int convertMvFloatToFixed(int val)
{
  const int exponent = val & MV_EXPONENT_MASK;
  const int mantissa = val >> MV_EXPONENT_BITCOUNT;
  return exponent == 0 ? mantissa : (mantissa ^ MV_MANTISSA_LIMIT) * (1 << (exponent - 1));
}

int roundMvComp(int x) { return convertMvFloatToFixed(convertMvFixedToFloat(x)); }

int PU::getDistScaleFactor(const int &currPOC, const int &currRefPOC, const int &colPOC, const int &colRefPOC)
{
  return xGetDistScaleFactor(currPOC, currRefPOC, colPOC, colRefPOC);
}

void PU::getInterMMVDMergeCandidates(const CodingUnit &cu, MergeCtx &mrgCtx)
{
  int            k;
  int            currBaseNum     = 0;
  const uint16_t maxNumMergeCand = mrgCtx.numValidMergeCand;

  for (k = 0; k < maxNumMergeCand; k++)
  {
    for (const auto l: { RPL0, RPL1 })
    {
      const int refIdx                  = mrgCtx.mvFieldNeighbours[k][l].refIdx;
      mrgCtx.mmvdBaseMv[currBaseNum][l] = refIdx >= 0 ? mrgCtx.mvFieldNeighbours[k][l] : MvField(Mv(0, 0), -1);
    }

    mrgCtx.mmvdUseAltHpelIf[currBaseNum] = mrgCtx.useAltHpelIf[k];

    currBaseNum++;

    if (currBaseNum == MmvdIdx::BASE_MV_NUM)
    {
      break;
    }
  }
}

bool PU::getColocatedMVP(const CodingUnit &cu, const RefPicList &eRefPicList, const Position &_pos, Mv &rcMv,
                         const int &refIdx, bool sbFlag)
{
  // don't perform MV compression when generally disabled or subPuMvp is used
  const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
  const unsigned mask  = ~(scale - 1);

  const Position pos = Position { PosType(_pos.x & mask), PosType(_pos.y & mask) };

  const Slice &slice = *cu.cs->slice;

  // use coldir.
  const Picture *const pColPic =
    slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.m_colFromL0Flag : 0), slice.m_colRefIdx);
  if (!pColPic || pColPic->isRefScaled(cu.cs->pps))
  {
    return false;
  }

  // Check the position of colocated block is within a subpicture
  const SubPic &curSubPic = cu.cs->slice->m_pps->getSubPicFromPos(cu.lumaPos());
  if (curSubPic.m_treatedAsPicFlag)
  {
    if (!curSubPic.isContainingPos(pos))
    {
      return false;
    }
  }
  RefPicList eColRefPicList = slice.m_checkLdc ? eRefPicList : RefPicList(slice.m_colFromL0Flag);

  const MotionInfo &mi = pColPic->m_cs->getMotionInfo(pos);

  if (!mi.isInter)
  {
    return false;
  }
  if (mi.isIBCmot)
  {
    return false;
  }
  if (CU::isIBC(cu))
  {
    return false;
  }
  int colRefIdx = mi.refIdx[eColRefPicList];

  if (sbFlag && !slice.m_checkLdc)
  {
    eColRefPicList = eRefPicList;
    colRefIdx      = mi.refIdx[eColRefPicList];
    if (colRefIdx < 0)
    {
      return false;
    }
  }
  else
  {
    if (colRefIdx < 0)
    {
      eColRefPicList = RefPicList(1 - eColRefPicList);
      colRefIdx      = mi.refIdx[eColRefPicList];

      if (colRefIdx < 0)
      {
        return false;
      }
    }
  }

  const Slice *pColSlice = nullptr;

  for (const auto s: pColPic->m_slices)
  {
    if (s->m_independentSliceIdx == mi.sliceIdx)
    {
      pColSlice = s;
      break;
    }
  }

  CHECK(pColSlice == nullptr, "Slice segment not found");

  const Slice &colSlice = *pColSlice;

  const bool isCurrRefLongTerm = slice.getRefPic(eRefPicList, refIdx)->m_longTerm;
  const bool isColRefLongTerm  = colSlice.m_isUsedAsLongTerm[eColRefPicList][colRefIdx];

  if (isCurrRefLongTerm != isColRefLongTerm)
  {
    return false;
  }

  // Scale the vector.
  Mv cColMv = mi.mv[eColRefPicList];
  cColMv.setHor(roundMvComp(cColMv.getHor()));
  cColMv.setVer(roundMvComp(cColMv.getVer()));

  if (isCurrRefLongTerm /*|| isColRefLongTerm*/)
  {
    rcMv = cColMv;
    rcMv.clipToStorageBitDepth();
  }
  else
  {
    const int currPOC    = slice.m_poc;
    const int colPOC     = colSlice.m_poc;
    const int colRefPOC  = colSlice.getRefPOC(eColRefPicList, colRefIdx);
    const int currRefPOC = slice.getRefPic(eRefPicList, refIdx)->m_poc;
    const int distscale  = xGetDistScaleFactor(currPOC, currRefPOC, colPOC, colRefPOC);

    if (distscale == 4096)
    {
      rcMv = cColMv;
      rcMv.clipToStorageBitDepth();
    }
    else
    {
      rcMv = cColMv.getScaledMv(distscale);
    }
  }

  return true;
}

bool PU::isDiffMER(const Position &pos1, const Position &pos2, const unsigned plevel)
{
  const unsigned xN = pos1.x;
  const unsigned yN = pos1.y;
  const unsigned xP = pos2.x;
  const unsigned yP = pos2.y;

  if ((xN >> plevel) != (xP >> plevel))
  {
    return true;
  }

  if ((yN >> plevel) != (yP >> plevel))
  {
    return true;
  }

  return false;
}

bool PU::addNeighborMv(const Mv &currMv, static_vector<Mv, IBC_NUM_CANDIDATES> &neighborMvs)
{
  for (const auto &cand: neighborMvs)
  {
    if (currMv == cand)
    {
      return false;
    }
  }
  neighborMvs.push_back(currMv);
  return true;
}

void PU::getIbcMVPsEncOnly(CodingUnit &cu, static_vector<Mv, IBC_NUM_CANDIDATES> &mvPred)
{
  const PreCalcValues &pcv             = *cu.cs->pcv;
  const int            cuWidth         = cu.blocks[COMP_Y].width;
  const int            cuHeight        = cu.blocks[COMP_Y].height;
  const int            log2UnitWidth   = floorLog2(pcv.minCUWidth);
  const int            log2UnitHeight  = floorLog2(pcv.minCUHeight);
  const int            totalAboveUnits = (cuWidth >> log2UnitWidth) + 1;
  const int            totalLeftUnits  = (cuHeight >> log2UnitHeight) + 1;

  Position posLT = cu.Y().topLeft();

  // above-left
  const CodingUnit *aboveLeftPU = cu.cs->getCURestricted(posLT.offset(-1, -1), cu, ChannelType::LUMA);
  if (aboveLeftPU && CU::isIBC(*aboveLeftPU))
  {
    addNeighborMv(aboveLeftPU->bv, mvPred);
  }

  // above neighbors
  for (uint32_t dx = 0; dx < totalAboveUnits && mvPred.size() < mvPred.max_size(); dx++)
  {
    const CodingUnit *tmpPU = cu.cs->getCURestricted(posLT.offset((dx << log2UnitWidth), -1), cu, ChannelType::LUMA);
    if (tmpPU && CU::isIBC(*tmpPU))
    {
      addNeighborMv(tmpPU->bv, mvPred);
    }
  }

  // left neighbors
  for (uint32_t dy = 0; dy < totalLeftUnits && mvPred.size() < mvPred.max_size(); dy++)
  {
    const CodingUnit *tmpPU = cu.cs->getCURestricted(posLT.offset(-1, (dy << log2UnitHeight)), cu, ChannelType::LUMA);
    if (tmpPU && CU::isIBC(*tmpPU))
    {
      addNeighborMv(tmpPU->bv, mvPred);
    }
  }

  size_t numAvaiCandInLUT = cu.cs->motionLut.lutIbc.size();
  for (uint32_t cand = 0; cand < numAvaiCandInLUT && mvPred.size() < mvPred.max_size(); cand++)
  {
    MotionInfo neibMi = cu.cs->motionLut.lutIbc[cand];
    addNeighborMv(neibMi.bv, mvPred);
  }

  std::array<bool, IBC_NUM_CANDIDATES> isBvCandDerived;
  isBvCandDerived.fill(false);

  auto curNbPred = mvPred.size();
  if (curNbPred < mvPred.max_size())
  {
    do
    {
      curNbPred = mvPred.size();
      for (uint32_t idx = 0; idx < curNbPred && mvPred.size() < mvPred.max_size(); idx++)
      {
        if (!isBvCandDerived[idx])
        {
          Mv derivedBv;
          if (getDerivedBV(cu, mvPred[idx], derivedBv))
          {
            addNeighborMv(derivedBv, mvPred);
          }
          isBvCandDerived[idx] = true;
        }
      }
    } while (mvPred.size() > curNbPred && mvPred.size() < mvPred.max_size());
  }
}

bool PU::getDerivedBV(CodingUnit &cu, const Mv &currentMv, Mv &derivedMv)
{
  int cuPelX  = cu.lumaPos().x;
  int cuPelY  = cu.lumaPos().y;
  int rX      = cuPelX + currentMv.getHor();
  int rY      = cuPelY + currentMv.getVer();
  int offsetX = currentMv.getHor();
  int offsetY = currentMv.getVer();

  if (rX < 0 || rY < 0 || rX >= cu.cs->slice->m_pps->m_picWidthInLumaSamples ||
      rY >= cu.cs->slice->m_pps->m_picHeightInLumaSamples)
  {
    return false;
  }

  const CodingUnit *neibRefPU = nullptr;
  neibRefPU                   = cu.cs->getCURestricted(cu.lumaPos().offset(offsetX, offsetY), cu, ChannelType::LUMA);

  bool isIBC = (neibRefPU) ? CU::isIBC(*neibRefPU) : 0;
  if (isIBC)
  {
    derivedMv = neibRefPU->bv;
    derivedMv += currentMv;
  }
  return isIBC;
}

/**
 * Constructs a list of candidates for IBC AMVP (See specification, section "Derivation process for motion vector
 * predictor candidates")
 */
void PU::fillIBCMvpCand(CodingUnit &cu, AMVPInfo &amvpInfo)
{
  AMVPInfo *pInfo = &amvpInfo;

  pInfo->numCand = 0;

  MergeCtx mergeCtx;
  PU::getIBCMergeCandidates(cu, mergeCtx, AMVP_MAX_NUM_CANDS - 1);
  int candIdx = 0;
  while (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    pInfo->mvCand[pInfo->numCand] = mergeCtx.mvFieldNeighbours[candIdx][0].mv;
    ;
    pInfo->numCand++;
    candIdx++;
  }

  for (Mv &mv: pInfo->mvCand)
  {
    mv.roundIbcPrecInternal2Amvr(cu.imv);
  }
}

/** Constructs a list of candidates for AMVP (See specification, section "Derivation process for motion vector predictor
 * candidates") \param uiPartIdx \param uiPartAddr \param eRefPicList \param refIdx \param pInfo
 */
void PU::fillMvpCand(CodingUnit &cu, const RefPicList &eRefPicList, const int &refIdx, AMVPInfo &amvpInfo)
{
  CodingStructure &cs = *cu.cs;

  AMVPInfo *pInfo = &amvpInfo;

  pInfo->numCand = 0;

  if (refIdx < 0)
  {
    return;
  }

  //-- Get Spatial MV
  Position posLT = cu.Y().topLeft();
  Position posRT = cu.Y().topRight();
  Position posLB = cu.Y().bottomLeft();

  {
    bool added = addMVPCandUnscaled(cu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, *pInfo);

    if (!added)
    {
      added = addMVPCandUnscaled(cu, eRefPicList, refIdx, posLB, MD_LEFT, *pInfo);
    }
  }

  // Above predictor search
  {
    bool added = addMVPCandUnscaled(cu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, *pInfo);

    if (!added)
    {
      added = addMVPCandUnscaled(cu, eRefPicList, refIdx, posRT, MD_ABOVE, *pInfo);

      if (!added)
      {
        addMVPCandUnscaled(cu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, *pInfo);
      }
    }
  }

  for (int i = 0; i < pInfo->numCand; i++)
  {
    pInfo->mvCand[i].roundTransPrecInternal2Amvr(cu.imv);
  }

  if (pInfo->numCand == 2)
  {
    if (pInfo->mvCand[0] == pInfo->mvCand[1])
    {
      pInfo->numCand = 1;
    }
  }

  if (cs.picHeader->m_enableTMVPFlag && pInfo->numCand < AMVP_MAX_NUM_CANDS &&
      (cu.lumaSize().width + cu.lumaSize().height > 12))
  {
    // Get Temporal Motion Predictor
    const int refIdxCol = refIdx;

    Position posRB = cu.Y().bottomRight().offset(-3, -3);

    const PreCalcValues &pcv = *cs.pcv;

    Position posC0;
    bool     C0Avail = false;
    Position posC1   = cu.Y().center();
    Mv       cColMv;

    bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
    const SubPic &curSubPic = cu.cs->slice->m_pps->getSubPicFromPos(cu.lumaPos());
    if (curSubPic.m_treatedAsPicFlag)
    {
      boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.m_subPicRight &&
                      (posRB.y + pcv.minCUHeight) <= curSubPic.m_subPicBottom);
    }
    if (boundaryCond)
    {
      int posYInCtu = posRB.y & pcv.maxCUHeightMask;
      if (posYInCtu + 4 < pcv.maxCUHeight)
      {
        posC0   = posRB.offset(4, 4);
        C0Avail = true;
      }
    }
    if ((C0Avail && getColocatedMVP(cu, eRefPicList, posC0, cColMv, refIdxCol, false)) ||
        getColocatedMVP(cu, eRefPicList, posC1, cColMv, refIdxCol, false))
    {
      cColMv.roundTransPrecInternal2Amvr(cu.imv);
      pInfo->mvCand[pInfo->numCand++] = cColMv;
    }
  }

  if (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    const int currRefPOC = cs.slice->getRefPic(eRefPicList, refIdx)->m_poc;
    addAMVPHMVPCand(cu, eRefPicList, currRefPOC, *pInfo);
  }

  if (pInfo->numCand > AMVP_MAX_NUM_CANDS)
  {
    pInfo->numCand = AMVP_MAX_NUM_CANDS;
  }

  while (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    pInfo->mvCand[pInfo->numCand] = Mv(0, 0);
    pInfo->numCand++;
  }

  for (Mv &mv: pInfo->mvCand)
  {
    mv.roundTransPrecInternal2Amvr(cu.imv);
  }
}

bool PU::addAffineMVPCandUnscaled(const CodingUnit &cu, const RefPicList &refPicList, const int &refIdx,
                                  const Position &pos, const MvpDir &dir, AffineAMVPInfo &affiAMVPInfo,
                                  int aiNeibeInherited[5])
{
  CodingStructure  &cs     = *cu.cs;
  const CodingUnit *neibPU = nullptr;
  Position          neibPos;

  switch (dir)
  {
  case MD_LEFT:
    neibPos = pos.offset(-1, 0);
    break;
  case MD_ABOVE:
    neibPos = pos.offset(0, -1);
    break;
  case MD_ABOVE_RIGHT:
    neibPos = pos.offset(1, -1);
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset(-1, 1);
    break;
  case MD_ABOVE_LEFT:
    neibPos = pos.offset(-1, -1);
    break;
  default:
    break;
  }

  neibPU = cs.getCURestricted(neibPos, cu, cu.chType);

  if (neibPU == nullptr || !neibPU->isAffineBlock())
  {
    return false;
  }

  Mv                outputAffineMv[3];
  const MotionInfo &neibMi = neibPU->getMotionInfo(neibPos);

  const int        currRefPOC    = cs.slice->getRefPic(refPicList, refIdx)->m_poc;
  const RefPicList refPicList2nd = (refPicList == RPL0) ? RPL1 : RPL0;

  for (int predictorSource = 0; predictorSource < 2;
       predictorSource++)   // examine the indicated reference picture list, then if not available, examine the other
                            // list.
  {
    const RefPicList eRefPicListIndex = (predictorSource == 0) ? refPicList : refPicList2nd;
    const int        neibRefIdx       = neibMi.refIdx[eRefPicListIndex];

    if (((neibPU->interDir & (eRefPicListIndex + 1)) == 0) ||
        cu.slice->getRefPOC(eRefPicListIndex, neibRefIdx) != currRefPOC)
    {
      continue;
    }

    xInheritedAffineMv(cu, neibPU, eRefPicListIndex, outputAffineMv);
    outputAffineMv[0].roundAffinePrecInternal2Amvr(cu.imv);
    outputAffineMv[1].roundAffinePrecInternal2Amvr(cu.imv);
    affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[0];
    affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[1];
    if (cu.affineType == AffineModel::_6_PARAMS)
    {
      outputAffineMv[2].roundAffinePrecInternal2Amvr(cu.imv);
      affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[2];
    }
    if (!checkLastAffineAMVPCandRedundancy(cu, affiAMVPInfo))
    {
      continue;
    }
    affiAMVPInfo.numCand++;
    aiNeibeInherited[dir] = 1;
    return true;
  }

  return false;
}

void PU::xInheritedAffineMv(const CodingUnit &cu, const CodingUnit *puNeighbour, RefPicList eRefPicList, Mv rcMv[3])
{
  int posNeiX = puNeighbour->Y().pos().x;
  int posNeiY = puNeighbour->Y().pos().y;
  int posCurX = cu.Y().pos().x;
  int posCurY = cu.Y().pos().y;

  int neiW = puNeighbour->lwidth();
  int curW = cu.lwidth();
  int neiH = puNeighbour->lheight();
  int curH = cu.lheight();

  Mv mvLT, mvRT, mvLB;
  mvLT = puNeighbour->mvAffi[eRefPicList][0];
  mvRT = puNeighbour->mvAffi[eRefPicList][1];
  mvLB = puNeighbour->mvAffi[eRefPicList][2];

  int shift = MAX_CU_DEPTH;
  int dmvHorX, dmvHorY, dmvVerX, dmvVerY;

  dmvHorX = (mvRT - mvLT).getHor() * (1 << (shift - floorLog2(neiW)));
  dmvHorY = (mvRT - mvLT).getVer() * (1 << (shift - floorLog2(neiW)));

  if (puNeighbour->affineType == AffineModel::_6_PARAMS)
  {
    dmvVerX = (mvLB - mvLT).getHor() * (1 << (shift - floorLog2(neiH)));
    dmvVerY = (mvLB - mvLT).getVer() * (1 << (shift - floorLog2(neiH)));
  }
  else
  {
    dmvVerX = -dmvHorY;
    dmvVerY = dmvHorX;
  }

  int mvScaleHor = mvLT.getHor() * (1 << shift);
  int mvScaleVer = mvLT.getVer() * (1 << shift);

  // v0
  rcMv[0].hor = mvScaleHor + dmvHorX * (posCurX - posNeiX) + dmvVerX * (posCurY - posNeiY);
  rcMv[0].ver = mvScaleVer + dmvHorY * (posCurX - posNeiX) + dmvVerY * (posCurY - posNeiY);
  rcMv[0] >>= shift;
  rcMv[0].clipToStorageBitDepth();

  // v1
  rcMv[1].hor = mvScaleHor + dmvHorX * (posCurX + curW - posNeiX) + dmvVerX * (posCurY - posNeiY);
  rcMv[1].ver = mvScaleVer + dmvHorY * (posCurX + curW - posNeiX) + dmvVerY * (posCurY - posNeiY);
  rcMv[1] >>= shift;
  rcMv[1].clipToStorageBitDepth();

  // v2
  {
    rcMv[2].hor = mvScaleHor + dmvHorX * (posCurX - posNeiX) + dmvVerX * (posCurY + curH - posNeiY);
    rcMv[2].ver = mvScaleVer + dmvHorY * (posCurX - posNeiX) + dmvVerY * (posCurY + curH - posNeiY);
    rcMv[2] >>= shift;
    rcMv[2].clipToStorageBitDepth();
  }
}

void PU::deriveAffineParametersFromMVs(const CodingUnit &cu, const Mv acMvTemp[3], int *affinePara,
                                       AffineModel affModel)
{
  Mv        mvLT = acMvTemp[0];
  Mv        mvRT = acMvTemp[1];
  Mv        mvLB = acMvTemp[2];
  const int iBit = MAX_CU_DEPTH;
  int       iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;
  iDMvHorX = (mvRT - mvLT).getHor() << (iBit - floorLog2(cu.lwidth()));
  iDMvHorY = (mvRT - mvLT).getVer() << (iBit - floorLog2(cu.lwidth()));
  if (affModel == AffineModel::_6_PARAMS)
  {
    iDMvVerX = (mvLB - mvLT).getHor() << (iBit - floorLog2(cu.lheight()));
    iDMvVerY = (mvLB - mvLT).getVer() << (iBit - floorLog2(cu.lheight()));
  }
  else
  {
    iDMvVerX = -iDMvHorY;
    iDMvVerY = iDMvHorX;
  }

  affinePara[0] = iDMvHorX;
  affinePara[1] = iDMvVerX;
  affinePara[2] = iDMvHorY;
  affinePara[3] = iDMvVerY;
}
void PU::deriveMVsFromAffineParameters(const CodingUnit &cu, Mv rcMv[3], int *affinePara, const Mv &cBaseMv,
                                       const Position &cBasePos)
{
  int posCurX = cu.Y().pos().x;
  int posCurY = cu.Y().pos().y;

  int curW = cu.lwidth();
  int curH = cu.lheight();

  int shift = MAX_CU_DEPTH;
  int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;

  iDMvHorX = affinePara[0];
  iDMvVerX = affinePara[1];
  iDMvHorY = affinePara[2];
  iDMvVerY = affinePara[3];

  int iMvScaleHor = cBaseMv.getHor() << shift;
  int iMvScaleVer = cBaseMv.getVer() << shift;

  int horTmp, verTmp;

  // v0
  horTmp = iMvScaleHor + iDMvHorX * (posCurX - cBasePos.x) + iDMvVerX * (posCurY - cBasePos.y);
  verTmp = iMvScaleVer + iDMvHorY * (posCurX - cBasePos.x) + iDMvVerY * (posCurY - cBasePos.y);
  roundAffineMv(horTmp, verTmp, shift);

  rcMv[0].hor = horTmp;
  rcMv[0].ver = verTmp;

  // v1
  horTmp = iMvScaleHor + iDMvHorX * (posCurX + curW - cBasePos.x) + iDMvVerX * (posCurY - cBasePos.y);
  verTmp = iMvScaleVer + iDMvHorY * (posCurX + curW - cBasePos.x) + iDMvVerY * (posCurY - cBasePos.y);
  roundAffineMv(horTmp, verTmp, shift);

  rcMv[1].hor = horTmp;
  rcMv[1].ver = verTmp;

  // v2

  horTmp = iMvScaleHor + iDMvHorX * (posCurX - cBasePos.x) + iDMvVerX * (posCurY + curH - cBasePos.y);
  verTmp = iMvScaleVer + iDMvHorY * (posCurX - cBasePos.x) + iDMvVerY * (posCurY + curH - cBasePos.y);
  roundAffineMv(horTmp, verTmp, shift);

  rcMv[2].hor = horTmp;
  rcMv[2].ver = verTmp;
}

void PU::deriveCenterMVFromAffineParameters(const CodingUnit &cu, Mv &rcMv, int *affinePara, const Mv &cBaseMv,
                                            const Position &cBasePos)
{

  int posCenterX = cu.Y().pos().x + (cu.lwidth() >> 1);
  int posCenterY = cu.Y().pos().y + (cu.lheight() >> 1);

  int shift = MAX_CU_DEPTH;
  int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;

  iDMvHorX = affinePara[0];
  iDMvVerX = affinePara[1];
  iDMvHorY = affinePara[2];
  iDMvVerY = affinePara[3];

  int iMvScaleHor = cBaseMv.getHor() << shift;
  int iMvScaleVer = cBaseMv.getVer() << shift;

  int horTmp, verTmp;

  horTmp = iMvScaleHor + iDMvHorX * (posCenterX - cBasePos.x) + iDMvVerX * (posCenterY - cBasePos.y);
  verTmp = iMvScaleVer + iDMvHorY * (posCenterX - cBasePos.x) + iDMvVerY * (posCenterY - cBasePos.y);
  roundAffineMv(horTmp, verTmp, shift);

  rcMv.hor = horTmp;
  rcMv.ver = verTmp;
}

void PU::storeAffParas(int *affinePara)
{
  const int minV        = -(1 << (AFF_PARA_STORE_BITS - 1));
  const int maxV        = (1 << (AFF_PARA_STORE_BITS - 1)) - 1;
  const int roundingAdd = (1 << AFF_PARA_SHIFT) >> 1;
  affinePara[0]         = Clip3(minV, maxV,
                        affinePara[0] >= 0 ? ((affinePara[0] + roundingAdd) >> AFF_PARA_SHIFT)
                                                   : -((-affinePara[0] + roundingAdd) >> AFF_PARA_SHIFT));
  affinePara[1]         = Clip3(minV, maxV,
                        affinePara[1] >= 0 ? ((affinePara[1] + roundingAdd) >> AFF_PARA_SHIFT)
                                                   : -((-affinePara[1] + roundingAdd) >> AFF_PARA_SHIFT));
  affinePara[2]         = Clip3(minV, maxV,
                        affinePara[2] >= 0 ? ((affinePara[2] + roundingAdd) >> AFF_PARA_SHIFT)
                                                   : -((-affinePara[2] + roundingAdd) >> AFF_PARA_SHIFT));
  affinePara[3]         = Clip3(minV, maxV,
                        affinePara[3] >= 0 ? ((affinePara[3] + roundingAdd) >> AFF_PARA_SHIFT)
                                                   : -((-affinePara[3] + roundingAdd) >> AFF_PARA_SHIFT));
}

void PU::xGetAffineMvFromLUT(AffineMotionInfo *affHistInfo, int rParameters[4])
{
  rParameters[0] = affHistInfo->oneSetAffineParameters[0] << AFF_PARA_SHIFT;
  rParameters[1] = affHistInfo->oneSetAffineParameters[1] << AFF_PARA_SHIFT;
  rParameters[2] = affHistInfo->oneSetAffineParameters[2] << AFF_PARA_SHIFT;
  rParameters[3] = affHistInfo->oneSetAffineParameters[3] << AFF_PARA_SHIFT;
}

void PU::xGetAffineMvFromLUT(short affineParameters[4], int rParameters[4])
{
  rParameters[0] = affineParameters[0] << AFF_PARA_SHIFT;
  rParameters[1] = affineParameters[1] << AFF_PARA_SHIFT;
  rParameters[2] = affineParameters[2] << AFF_PARA_SHIFT;
  rParameters[3] = affineParameters[3] << AFF_PARA_SHIFT;
}

// Add affine candidates from LUT
// return true to early terminate

bool PU::checkLastAffineMergeCandRedundancy(const CodingUnit &cu, AffineMergeCtx &affMrgCtx)
{
  if (affMrgCtx.numValidMergeCand < 1)
  {
    return true;
  }

  const int CPMV_SIMILARITY_THREH = 1;
  const int PARA_SIMILARITY_THREH = 1;

  const int lastIdx = affMrgCtx.numValidMergeCand;
  if (affMrgCtx.mergeType[lastIdx] == MergeType::SUBPU_ATMVP)
  {
    return true;
  }

  for (int idx = 0; idx < affMrgCtx.numValidMergeCand; idx++)
  {
    if (affMrgCtx.mergeType[idx] == MergeType::SUBPU_ATMVP)
    {
      continue;
    }
    if (affMrgCtx.affineType[idx] != affMrgCtx.affineType[lastIdx])
    {
      continue;
    }
    CHECK(affMrgCtx.affineType[idx] != AffineModel::_6_PARAMS && affMrgCtx.affineType[idx] != AffineModel::_4_PARAMS,
          "Invalid parameter");

    if (affMrgCtx.interDirNeighbours[idx] != affMrgCtx.interDirNeighbours[lastIdx])
    {
      continue;
    }
    if (affMrgCtx.interDirNeighbours[idx] == 3 && affMrgCtx.bcwIdx[idx] != affMrgCtx.bcwIdx[lastIdx])
    {
      continue;
    }

    if (affMrgCtx.LICFlags[idx] != affMrgCtx.LICFlags[lastIdx])
    {
      continue;
    }

    if ((affMrgCtx.interDirNeighbours[lastIdx] & 1) != 0)
    {
      if (affMrgCtx.mvFieldNeighbours[idx][0][RPL0].refIdx != affMrgCtx.mvFieldNeighbours[lastIdx][0][RPL0].refIdx)
      {
        continue;
      }
      Mv  acMvTemp[3];
      int affinePara[4], affineParaLast[4];
      acMvTemp[0] = affMrgCtx.mvFieldNeighbours[idx][0][0].mv;
      acMvTemp[1] = affMrgCtx.mvFieldNeighbours[idx][1][0].mv;
      acMvTemp[2] = affMrgCtx.mvFieldNeighbours[idx][2][0].mv;
      deriveAffineParametersFromMVs(cu, acMvTemp, affinePara, affMrgCtx.affineType[idx]);
      acMvTemp[0] = affMrgCtx.mvFieldNeighbours[lastIdx][0][0].mv;
      acMvTemp[1] = affMrgCtx.mvFieldNeighbours[lastIdx][1][0].mv;
      acMvTemp[2] = affMrgCtx.mvFieldNeighbours[lastIdx][2][0].mv;
      deriveAffineParametersFromMVs(cu, acMvTemp, affineParaLast, affMrgCtx.affineType[idx]);

      if (abs(affMrgCtx.mvFieldNeighbours[idx][0][0].mv.getHor() -
              affMrgCtx.mvFieldNeighbours[lastIdx][0][0].mv.getHor()) > CPMV_SIMILARITY_THREH ||
          abs(affMrgCtx.mvFieldNeighbours[idx][0][0].mv.getVer() -
              affMrgCtx.mvFieldNeighbours[lastIdx][0][0].mv.getVer()) > CPMV_SIMILARITY_THREH ||
          abs(affinePara[0] - affineParaLast[0]) > PARA_SIMILARITY_THREH ||
          abs(affinePara[2] - affineParaLast[2]) > PARA_SIMILARITY_THREH)
      {
        continue;
      }

      if (affMrgCtx.affineType[idx] == AffineModel::_6_PARAMS)
      {
        if (abs(affinePara[1] - affineParaLast[1]) > PARA_SIMILARITY_THREH ||
            abs(affinePara[3] - affineParaLast[3]) > PARA_SIMILARITY_THREH)
        {
          continue;
        }
      }
    }

    if ((affMrgCtx.interDirNeighbours[lastIdx] & 2) != 0)
    {
      if (affMrgCtx.mvFieldNeighbours[idx][0][RPL1].refIdx != affMrgCtx.mvFieldNeighbours[lastIdx][0][RPL1].refIdx)
      {
        continue;
      }
      Mv  acMvTemp[3];
      int affinePara[4], affineParaLast[4];
      acMvTemp[0] = affMrgCtx.mvFieldNeighbours[idx][0][RPL1].mv;
      acMvTemp[1] = affMrgCtx.mvFieldNeighbours[idx][1][RPL1].mv;
      acMvTemp[2] = affMrgCtx.mvFieldNeighbours[idx][2][RPL1].mv;
      deriveAffineParametersFromMVs(cu, acMvTemp, affinePara, affMrgCtx.affineType[idx]);
      acMvTemp[0] = affMrgCtx.mvFieldNeighbours[lastIdx][0][RPL1].mv;
      acMvTemp[1] = affMrgCtx.mvFieldNeighbours[lastIdx][1][RPL1].mv;
      acMvTemp[2] = affMrgCtx.mvFieldNeighbours[lastIdx][2][RPL1].mv;
      deriveAffineParametersFromMVs(cu, acMvTemp, affineParaLast, affMrgCtx.affineType[idx]);

      if (abs(affMrgCtx.mvFieldNeighbours[idx][0][RPL1].mv.getHor() -
              affMrgCtx.mvFieldNeighbours[lastIdx][0][RPL1].mv.getHor()) > CPMV_SIMILARITY_THREH ||
          abs(affMrgCtx.mvFieldNeighbours[idx][0][RPL1].mv.getVer() -
              affMrgCtx.mvFieldNeighbours[lastIdx][0][RPL1].mv.getVer()) > CPMV_SIMILARITY_THREH ||
          abs(affinePara[0] - affineParaLast[0]) > PARA_SIMILARITY_THREH ||
          abs(affinePara[2] - affineParaLast[2]) > PARA_SIMILARITY_THREH)
      {
        continue;
      }

      if (affMrgCtx.affineType[idx] == AffineModel::_6_PARAMS)
      {
        if (abs(affinePara[1] - affineParaLast[1]) > PARA_SIMILARITY_THREH ||
            abs(affinePara[3] - affineParaLast[3]) > PARA_SIMILARITY_THREH)
        {
          continue;
        }
      }
    }

    for (int mvNum = 0; mvNum < 3; mvNum++)
    {
      affMrgCtx.mvFieldNeighbours[lastIdx][mvNum][RPL0].setMvField(Mv(), -1);
      affMrgCtx.mvFieldNeighbours[lastIdx][mvNum][RPL1].setMvField(Mv(), -1);
    }
    affMrgCtx.interDirNeighbours[lastIdx] = 0;
    affMrgCtx.affineType[lastIdx]         = AffineModel::_4_PARAMS;
    affMrgCtx.mergeType[lastIdx]          = MergeType::DEFAULT_N;
    affMrgCtx.bcwIdx[lastIdx]             = BCW_DEFAULT;
    affMrgCtx.LICFlags[lastIdx]           = false;
    return false;
  }
  return true;
}

bool PU::addMergeHMVPCandFromAffModel(const CodingUnit &cu, MergeCtx &mrgCtx, const int &mrgCandIdx, int &cnt)
{
  if ((cu.slice->m_picHeader->m_mvdL1ZeroFlag || cu.slice->m_numRefIdx[RPL1] == 0) &&
      abs(cu.slice->getRefPOC(RPL0, 0) - cu.slice->m_poc) == 1)
  {
    return false;
  }

  const CodingUnit *npuGroup1[5];
  const CodingUnit *npuGroup2[5];
  Position          posGroup1[5];
  Position          posGroup2[5];
  int               numGroup1, numGroup2;
  numGroup1 = numGroup2 = 0;

  const int CHECKED_NEI_NUM = 5;

  const Position posLB = cu.Y().bottomLeft();
  const Position posLT = cu.Y().topLeft();
  const Position posRT = cu.Y().topRight();

  Position neiPositions[5] = { posLB.offset(-2, -1), posRT.offset(-1, -2), posRT.offset(3, -2), posLB.offset(-2, 3),
                               posLT.offset(-2, -2) };

  int       iTotalAffHMVPCandNum      = 0;
  const int MAX_ALLOWED_AFF_HMVP_CAND = 1;

  for (int nei = 0; nei < CHECKED_NEI_NUM; nei++)
  {
    const CodingUnit *cuNei = cu.cs->getCURestricted(neiPositions[nei], cu, cu.chType);
    if (!cuNei)
    {
      continue;
    }
    if (cuNei->predMode != MODE_INTER)
    {
      continue;
    }
    MotionInfo mvInfo = cuNei->getMotionInfo(neiPositions[nei]);
    if (!mvInfo.isInter || mvInfo.interDir <= 0 || mvInfo.interDir > 3)
    {
      continue;
    }
    if (cuNei->affine && !(cuNei->mergeFlag && cuNei->mergeType != MergeType::DEFAULT_N))
    {
      posGroup1[numGroup1]   = neiPositions[nei];
      npuGroup1[numGroup1++] = cuNei;
    }
    else
    {
      posGroup2[numGroup2]   = neiPositions[nei];
      npuGroup2[numGroup2++] = cuNei;
    }
  }
  for (int iAffListIdx = 0; iAffListIdx < MAX_NUM_AFF_HMVP_CANDS && iTotalAffHMVPCandNum < MAX_ALLOWED_AFF_HMVP_CAND;
       iAffListIdx++)
  {
    for (int i = 0; i < numGroup1 && iTotalAffHMVPCandNum < MAX_ALLOWED_AFF_HMVP_CAND; i++)
    {
      const CodingUnit *cuNei  = npuGroup1[i];
      MotionInfo        mvInfo = cuNei->getMotionInfo(posGroup1[i]);

      if (addOneMergeHMVPCandFromAffModel(cu, mrgCtx, cnt, cu.cs->motionLut.lutAff, iAffListIdx, mvInfo, posGroup1[i],
                                          mvInfo.interDir == 3 ? cuNei->bcwIdx : BCW_DEFAULT))
      {

        if (cnt == mrgCandIdx)
        {
          return true;
        }
        cnt++;
        if (cnt == mrgCtx.numValidMergeCand)
        {
          return true;
        }
        iTotalAffHMVPCandNum++;
      }
    }
    for (int i = 0; i < numGroup2 && iTotalAffHMVPCandNum < MAX_ALLOWED_AFF_HMVP_CAND; i++)
    {
      const CodingUnit *cuNei  = npuGroup2[i];
      MotionInfo        mvInfo = cuNei->getMotionInfo(posGroup2[i]);

      if (addOneMergeHMVPCandFromAffModel(cu, mrgCtx, cnt, cu.cs->motionLut.lutAff, iAffListIdx, mvInfo, posGroup2[i],
                                          mvInfo.interDir == 3 ? cuNei->bcwIdx : BCW_DEFAULT))
      {
        if (cnt == mrgCandIdx)
        {
          return true;
        }
        cnt++;
        if (cnt == mrgCtx.numValidMergeCand)
        {
          return true;
        }
        iTotalAffHMVPCandNum++;
      }
    }
  }
  CHECK(iTotalAffHMVPCandNum > MAX_ALLOWED_AFF_HMVP_CAND, "Invalid number of aff-HMVP based merge candidates");
  return false;
}

bool PU::addOneMergeHMVPCandFromAffModel(const CodingUnit &cu, MergeCtx &mrgCtx, int &cnt,
                                         static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS> *lutAff, int listIdx,
                                         const MotionInfo &mvInfo, Position neiPosition, int iGBiIdx)
{
  Mv  cMv[2];
  int histParameters[2][4];
  mrgCtx.interDirNeighbours[cnt] = 0;
  mrgCtx.bcwIdx[cnt]             = BCW_DEFAULT;
  mrgCtx.LICFlags[cnt]           = false;
  mrgCtx.mvFieldNeighbours[cnt][RPL0].setMvField(Mv(), -1);
  mrgCtx.mvFieldNeighbours[cnt][RPL1].setMvField(Mv(), -1);

  if ((mvInfo.interDir & 1) != 0)
  {
    CHECK(mvInfo.refIdx[0] == -1, "invalid Refidx");
    int idxInLUT = std::min((int)mvInfo.refIdx[0], MAX_NUM_AFFHMVP_ENTRIES_ONELIST - 1);
    int lutSize  = (int)lutAff[idxInLUT].size();
    if (listIdx >= lutSize || lutAff[idxInLUT][lutSize - 1 - listIdx].oneSetAffineParametersPattern == 0)
    {
      mrgCtx.mvFieldNeighbours[cnt][RPL0].setMvField(Mv(), -1);
    }
    else
    {
      AffineMotionInfo &affHistInfo = lutAff[idxInLUT][lutSize - 1 - listIdx];
      mrgCtx.interDirNeighbours[cnt] |= 1;
      xGetAffineMvFromLUT(&affHistInfo, histParameters[0]);
      deriveCenterMVFromAffineParameters(cu, cMv[0], histParameters[0], mvInfo.mv[0], neiPosition);
      mrgCtx.mvFieldNeighbours[cnt][RPL0].setMvField(cMv[0], mvInfo.refIdx[0]);
    }
  }
  if ((mvInfo.interDir & 2) != 0)
  {
    CHECK(mvInfo.refIdx[1] == -1, "invalid Refidx");
    int idxInLUT =
      MAX_NUM_AFFHMVP_ENTRIES_ONELIST + std::min((int)mvInfo.refIdx[1], MAX_NUM_AFFHMVP_ENTRIES_ONELIST - 1);
    int lutSize = (int)lutAff[idxInLUT].size();
    if (listIdx >= lutSize || lutAff[idxInLUT][lutSize - 1 - listIdx].oneSetAffineParametersPattern == 0)
    {
      mrgCtx.mvFieldNeighbours[cnt][RPL1].setMvField(Mv(), -1);
    }
    else
    {
      AffineMotionInfo &affHistInfo = lutAff[idxInLUT][lutSize - 1 - listIdx];
      mrgCtx.interDirNeighbours[cnt] |= 2;
      xGetAffineMvFromLUT(&affHistInfo, histParameters[1]);
      deriveCenterMVFromAffineParameters(cu, cMv[1], histParameters[1], mvInfo.mv[1], neiPosition);
      mrgCtx.mvFieldNeighbours[cnt][RPL1].setMvField(cMv[1], mvInfo.refIdx[1]);
    }
  }
  if (mrgCtx.interDirNeighbours[cnt] == 0)
  {
    mrgCtx.bcwIdx[cnt] = BCW_DEFAULT;
    mrgCtx.mvFieldNeighbours[cnt][RPL0].setMvField(Mv(), -1);
    mrgCtx.mvFieldNeighbours[cnt][RPL1].setMvField(Mv(), -1);
    return false;
  }
  mrgCtx.bcwIdx[cnt] = mrgCtx.interDirNeighbours[cnt] == 3 ? iGBiIdx : BCW_DEFAULT;

  if (mrgCtx.checkSimilarMotion(cnt))
  {
    mrgCtx.interDirNeighbours[cnt] = 0;
    mrgCtx.bcwIdx[cnt]             = BCW_DEFAULT;
    mrgCtx.mvFieldNeighbours[cnt][RPL0].setMvField(Mv(), -1);
    mrgCtx.mvFieldNeighbours[cnt][RPL1].setMvField(Mv(), -1);
    return false;
  }
  return true;
}

bool PU::addOneAffineMergeHMVPCand(const CodingUnit &cu, AffineMergeCtx &affMrgCtx,
                                   static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS> *lutAff, int listIdx,
                                   const MotionInfo &mvInfo, Position neiPosition, int iGBiIdx, bool LICFlag)
{
  Mv  cMv[2][3];
  int aiHistParameters[2][4];
  affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = 0;
  if ((mvInfo.interDir & 1) != 0)
  {
    CHECK(mvInfo.refIdx[0] == -1, "invalid Refidx");
    int idxInLUT = std::min((int)mvInfo.refIdx[0], MAX_NUM_AFFHMVP_ENTRIES_ONELIST - 1);
    int lutSize  = (int)lutAff[idxInLUT].size();
    if (listIdx >= lutSize || lutAff[idxInLUT][lutSize - 1 - listIdx].oneSetAffineParametersPattern == 0)
    {
      affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][0][RPL0].setMvField(Mv(), -1);
      affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][1][RPL0].setMvField(Mv(), -1);
      affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][2][RPL0].setMvField(Mv(), -1);
    }
    else
    {
      AffineMotionInfo &affHistInfo = lutAff[idxInLUT][lutSize - 1 - listIdx];
      affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] |= 1;
      xGetAffineMvFromLUT(&affHistInfo, aiHistParameters[0]);
      deriveMVsFromAffineParameters(cu, cMv[0], aiHistParameters[0], mvInfo.mv[0], neiPosition);
      for (int mvNum = 0; mvNum < 3; mvNum++)
      {
        affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][mvNum][RPL0].setMvField(cMv[0][mvNum],
                                                                                         mvInfo.refIdx[0]);
      }
    }
  }
  if ((mvInfo.interDir & 2) != 0)
  {
    CHECK(mvInfo.refIdx[1] == -1, "invalid Refidx");
    int idxInLUT =
      MAX_NUM_AFFHMVP_ENTRIES_ONELIST + std::min((int)mvInfo.refIdx[1], MAX_NUM_AFFHMVP_ENTRIES_ONELIST - 1);
    int lutSize = (int)lutAff[idxInLUT].size();
    if (listIdx >= lutSize || lutAff[idxInLUT][lutSize - 1 - listIdx].oneSetAffineParametersPattern == 0)
    {
      affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][0][RPL1].setMvField(Mv(), -1);
      affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][1][RPL1].setMvField(Mv(), -1);
      affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][2][RPL1].setMvField(Mv(), -1);
    }
    else
    {
      AffineMotionInfo &affHistInfo = lutAff[idxInLUT][lutSize - 1 - listIdx];
      affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] |= 2;
      xGetAffineMvFromLUT(&affHistInfo, aiHistParameters[1]);
      deriveMVsFromAffineParameters(cu, cMv[1], aiHistParameters[1], mvInfo.mv[1], neiPosition);
      for (int mvNum = 0; mvNum < 3; mvNum++)
      {
        affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][mvNum][RPL1].setMvField(cMv[1][mvNum],
                                                                                         mvInfo.refIdx[1]);
      }
    }
  }
  if (affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] == 0)
  {
    return false;
  }
  affMrgCtx.affineType[affMrgCtx.numValidMergeCand] = AffineModel::_6_PARAMS;
  affMrgCtx.bcwIdx[affMrgCtx.numValidMergeCand]     = iGBiIdx;
  CHECK(!cu.cs->sps->m_biLicEnabledFlag && affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] == 3 && LICFlag,
        "LIC is not allowed with bi");
  affMrgCtx.LICFlags[affMrgCtx.numValidMergeCand] = LICFlag;

  if (checkLastAffineMergeCandRedundancy(cu, affMrgCtx))
  {
    return true;
  }
  return false;
}

bool PU::addOneInheritedHMVPAffineMergeCand(
  const CodingUnit &cu, AffineMergeCtx &affMrgCtx,
  static_vector<AffineInheritInfo, MAX_NUM_AFF_INHERIT_HMVP_CANDS> &lutAffInherit, int listIdx)
{
  Mv  cMv[2][3];
  int aiHistParameters[2][4];

  int lutSize = (int)lutAffInherit.size();

  if (listIdx >= lutSize)
  {
    return false;
  }
  AffineInheritInfo &affHistInfo = lutAffInherit[lutSize - 1 - listIdx];

  affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = 0;

  affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][0][RPL0].setMvField(Mv(), -1);
  affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][1][RPL0].setMvField(Mv(), -1);
  affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][2][RPL0].setMvField(Mv(), -1);
  affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][0][RPL1].setMvField(Mv(), -1);
  affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][1][RPL1].setMvField(Mv(), -1);
  affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][2][RPL1].setMvField(Mv(), -1);

  if (affHistInfo.oneSetAffineParametersPattern0 != 0 && affHistInfo.baseMV[0].refIdx != -1)
  {
    affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] |= 1;
    xGetAffineMvFromLUT(affHistInfo.oneSetAffineParameters0, aiHistParameters[0]);
    deriveMVsFromAffineParameters(cu, cMv[0], aiHistParameters[0], affHistInfo.baseMV[0].mv, affHistInfo.basePos);
    for (int mvNum = 0; mvNum < 3; mvNum++)
    {
      affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][mvNum][RPL0].setMvField(cMv[0][mvNum],
                                                                                       affHistInfo.baseMV[0].refIdx);
    }
  }

  if (affHistInfo.oneSetAffineParametersPattern1 != 0 && affHistInfo.baseMV[1].refIdx != -1)
  {
    affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] |= 2;
    xGetAffineMvFromLUT(affHistInfo.oneSetAffineParameters1, aiHistParameters[1]);
    deriveMVsFromAffineParameters(cu, cMv[1], aiHistParameters[1], affHistInfo.baseMV[1].mv, affHistInfo.basePos);
    for (int mvNum = 0; mvNum < 3; mvNum++)
    {
      affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][mvNum][RPL1].setMvField(cMv[1][mvNum],
                                                                                       affHistInfo.baseMV[1].refIdx);
    }
  }

  if (affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] == 0)
  {
    return false;
  }
  affMrgCtx.affineType[affMrgCtx.numValidMergeCand] = AffineModel::_6_PARAMS;

  affMrgCtx.bcwIdx[affMrgCtx.numValidMergeCand] = BCW_DEFAULT;

  affMrgCtx.LICFlags[affMrgCtx.numValidMergeCand] = false;

  if (checkLastAffineMergeCandRedundancy(cu, affMrgCtx))
  {
    return true;
  }
  return false;
}

bool PU::addSpatialAffineMergeHMVPCand(const CodingUnit &cu, AffineMergeCtx &affMrgCtx,
                                       static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS> *lutAff, int affHMVPIdx,
                                       const CodingUnit *neiCUs[], Position neiPositions[], int iNeiNum,
                                       const int mrgCandIdx)
{
  const Slice   &slice                 = *cu.cs->slice;
  const uint32_t maxNumAffineMergeCand = slice.m_picHeader->m_maxNumAffineMergeCand;

  for (int nei = 0; nei < iNeiNum; nei++)
  {
    const CodingUnit *cuNei = neiCUs[nei];
    CHECK(!neiCUs[nei], "Invalid neighbour PU");
    MotionInfo mvInfo = cuNei->getMotionInfo(neiPositions[nei]);
    if (mvInfo.isIBCmot)
    {
      continue;
    }
    CHECK(!cu.cs->sps->m_biLicEnabledFlag && cuNei->interDir == 3 && cuNei->licFlag,
          "cuNei has interDir==3 and LIC flag set");
    if (addOneAffineMergeHMVPCand(cu, affMrgCtx, lutAff, affHMVPIdx, mvInfo, neiPositions[nei],
                                  cuNei->interDir == 3 ? cuNei->bcwIdx : BCW_DEFAULT, cuNei->licFlag))
    {
      if (affMrgCtx.numValidMergeCand == mrgCandIdx)   // for decoder
      {
        return true;
      }

      affMrgCtx.numValidMergeCand++;

      // early termination
      if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
      {
        return true;
      }
    }
  }   // for nei
  return false;
}

bool PU::addSpatialAffineAMVPHMVPCand(CodingUnit &cu, const RefPicList &eRefPicList, const int &refIdx,
                                      AffineAMVPInfo                                          &affiAMVPInfo,
                                      static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS> *lutAff, int iHMVPlistIdx,
                                      int neiIdx[], int iNeiNum, int aiNeibeInherited[], bool bFoundOne)
{
  if (affiAMVPInfo.numCand >= AMVP_MAX_NUM_CANDS)
  {
    return false;
  }

  const Slice   &slice = *cu.cs->slice;
  const Position posLB = cu.Y().bottomLeft();
  const Position posLT = cu.Y().topLeft();
  const Position posRT = cu.Y().topRight();

  Position neiPositions[5] = { posLB.offset(-2, -1), posRT.offset(-1, -2), posRT.offset(3, -2), posLB.offset(-2, 3),
                               posLT.offset(-2, -2) };

  const int currRefPOC = cu.cs->slice->getRefPic(eRefPicList, refIdx)->m_poc;

  AffineMotionInfo affHistInfo;
  int              idxInLUT =
    (int)eRefPicList * MAX_NUM_AFFHMVP_ENTRIES_ONELIST + std::min(refIdx, MAX_NUM_AFFHMVP_ENTRIES_ONELIST - 1);
  if (iHMVPlistIdx >= lutAff[idxInLUT].size())
  {
    return false;
  }
  affHistInfo = lutAff[idxInLUT][lutAff[idxInLUT].size() - 1 - iHMVPlistIdx];
  if (affHistInfo.oneSetAffineParametersPattern == 0)
  {
    return false;
  }

  Mv  cMv[3];
  int aiHistParameters[4];

  for (int idx = 0; idx < iNeiNum; idx++)
  {
    int nei = neiIdx[idx];
    if (aiNeibeInherited[nei])
    {
      continue;
    }
    aiNeibeInherited[nei]   = 1;
    const CodingUnit *cuNei = cu.cs->getCURestricted(neiPositions[nei], cu, cu.chType);
    if (!cuNei || !CU::isInter(cu))
    {
      continue;
    }

    MotionInfo mvInfo = cuNei->getMotionInfo(neiPositions[nei]);
    if (mvInfo.isIBCmot)
    {
      continue;
    }
    if (!mvInfo.isInter || mvInfo.interDir <= 0 || mvInfo.interDir > 3)
    {
      continue;
    }
    RefPicList selRefPicListIndex = eRefPicList;
    if (((mvInfo.interDir & (selRefPicListIndex + 1)) == 0) ||
        ((slice.getRefPic(selRefPicListIndex, mvInfo.refIdx[selRefPicListIndex])->m_poc) != currRefPOC))
    {
      selRefPicListIndex = RefPicList(1 - eRefPicList);
      if (((mvInfo.interDir & (selRefPicListIndex + 1)) == 0) ||
          ((slice.getRefPic(selRefPicListIndex, mvInfo.refIdx[selRefPicListIndex])->m_poc) != currRefPOC))
      {
        continue;
      }
    }

    xGetAffineMvFromLUT(&affHistInfo, aiHistParameters);
    deriveMVsFromAffineParameters(cu, cMv, aiHistParameters, mvInfo.mv[selRefPicListIndex], neiPositions[nei]);

    if (cu.imv == 0)
    {
      cMv[0].roundToPrecision(MvPrecision::INTERNAL, MvPrecision::QUARTER);
      cMv[1].roundToPrecision(MvPrecision::INTERNAL, MvPrecision::QUARTER);
      cMv[2].roundToPrecision(MvPrecision::INTERNAL, MvPrecision::QUARTER);
    }
    else if (cu.imv == 2)
    {
      cMv[0].roundToPrecision(MvPrecision::INTERNAL, MvPrecision::ONE);
      cMv[1].roundToPrecision(MvPrecision::INTERNAL, MvPrecision::ONE);
      cMv[2].roundToPrecision(MvPrecision::INTERNAL, MvPrecision::ONE);
    }

    affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = cMv[0];
    affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = cMv[1];
    affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = cMv[2];

    if (!checkLastAffineAMVPCandRedundancy(cu, affiAMVPInfo))
    {
      continue;
    }

    affiAMVPInfo.numCand++;

    if (affiAMVPInfo.numCand == AMVP_MAX_NUM_CANDS)
    {
      return true;
    }
    if (bFoundOne)
    {
      return false;
    }
  }

  return false;
}

bool PU::checkLastAffineAMVPCandRedundancy(const CodingUnit &cu, AffineAMVPInfo &affiAMVPInfo)
{
  if (affiAMVPInfo.numCand < 1)
  {
    return true;
  }

  const int CPMV_SIMILARITY_THREH = 0;
  const int lastIdx               = affiAMVPInfo.numCand;

  for (int idx = 0; idx < affiAMVPInfo.numCand; idx++)
  {

    if (abs(affiAMVPInfo.mvCandLT[idx].getHor() - affiAMVPInfo.mvCandLT[lastIdx].getHor()) > CPMV_SIMILARITY_THREH ||
        abs(affiAMVPInfo.mvCandLT[idx].getVer() - affiAMVPInfo.mvCandLT[lastIdx].getVer()) > CPMV_SIMILARITY_THREH ||
        abs(affiAMVPInfo.mvCandRT[idx].getHor() - affiAMVPInfo.mvCandRT[lastIdx].getHor()) > CPMV_SIMILARITY_THREH ||
        abs(affiAMVPInfo.mvCandRT[idx].getVer() - affiAMVPInfo.mvCandRT[lastIdx].getVer()) > CPMV_SIMILARITY_THREH)
    {
      continue;
    }
    if (cu.affineType == AffineModel::_6_PARAMS)
    {
      if (abs(affiAMVPInfo.mvCandLB[idx].getHor() - affiAMVPInfo.mvCandLB[lastIdx].getHor()) > CPMV_SIMILARITY_THREH ||
          abs(affiAMVPInfo.mvCandLB[idx].getVer() - affiAMVPInfo.mvCandLB[lastIdx].getVer()) > CPMV_SIMILARITY_THREH)
      {
        continue;
      }
    }
    return false;
  }
  return true;
}

void PU::fillAffineMvpCand(CodingUnit &cu, const RefPicList &eRefPicList, const int &refIdx,
                           AffineAMVPInfo &affiAMVPInfo)
{
  affiAMVPInfo.numCand = 0;

  if (refIdx < 0)
  {
    return;
  }

  // insert inherited affine candidates
  Mv       outputAffineMv[3];
  Position posLT = cu.Y().topLeft();
  Position posRT = cu.Y().topRight();
  Position posLB = cu.Y().bottomLeft();

  int aiNeibeInherited[5];
  memset(aiNeibeInherited, 0, sizeof(aiNeibeInherited));
  Position neiPositions[5] = { posLB.offset(-2, -1), posRT.offset(-1, -2), posRT.offset(3, -2), posLB.offset(-2, 3),
                               posLT.offset(-2, -2) };
  const CodingUnit *neiCU[5];
  for (int nei = 0; nei < 5; nei++)
  {
    neiCU[nei] = cu.cs->getCURestricted(neiPositions[nei], cu, cu.chType);
  }
  int neiIdx[5] = {
    0, 1, 2, 3, 4,
  };
  int leftNeiIdx[2];
  int leftAffNeiNum = 0;

  int aboveNeiIdx[3];
  int aboveAffNeiNum = 0;

  if (neiCU[3] && CU::isInter(*neiCU[3]) && neiCU[3]->affine && neiCU[3]->mergeType == MergeType::DEFAULT_N)
  {
    MotionInfo mvInfo = neiCU[3]->getMotionInfo(neiPositions[3]);
    if (mvInfo.isInter && mvInfo.interDir > 1 && mvInfo.interDir <= 3)
    {
      leftNeiIdx[leftAffNeiNum++] = 3;
    }
  }
  if (neiCU[0] && CU::isInter(*neiCU[0]) && neiCU[0]->affine && neiCU[0]->mergeType == MergeType::DEFAULT_N)
  {
    MotionInfo mvInfo = neiCU[0]->getMotionInfo(neiPositions[0]);
    if (mvInfo.isInter && mvInfo.interDir > 1 && mvInfo.interDir <= 3)
    {
      leftNeiIdx[leftAffNeiNum++] = 0;
    }
  }
  if (neiCU[2] && CU::isInter(*neiCU[2]) && neiCU[2]->affine && neiCU[2]->mergeType == MergeType::DEFAULT_N)
  {
    MotionInfo mvInfo = neiCU[2]->getMotionInfo(neiPositions[2]);
    if (mvInfo.isInter && mvInfo.interDir > 1 && mvInfo.interDir <= 3)
    {
      aboveNeiIdx[aboveAffNeiNum++] = 2;
    }
  }
  if (neiCU[1] && CU::isInter(*neiCU[1]) && neiCU[1]->affine && neiCU[1]->mergeType == MergeType::DEFAULT_N)
  {
    MotionInfo mvInfo = neiCU[1]->getMotionInfo(neiPositions[1]);
    if (mvInfo.isInter && mvInfo.interDir > 1 && mvInfo.interDir <= 3)
    {
      aboveNeiIdx[aboveAffNeiNum++] = 1;
    }
  }
  if (neiCU[4] && CU::isInter(*neiCU[4]) && neiCU[4]->affine && neiCU[4]->mergeType == MergeType::DEFAULT_N)
  {
    MotionInfo mvInfo = neiCU[4]->getMotionInfo(neiPositions[4]);
    if (mvInfo.isInter && mvInfo.interDir > 1 && mvInfo.interDir <= 3)
    {
      aboveNeiIdx[aboveAffNeiNum++] = 4;
    }
  }
  // check left neighbor
  if (!addAffineMVPCandUnscaled(cu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, affiAMVPInfo, aiNeibeInherited))
  {
    addAffineMVPCandUnscaled(cu, eRefPicList, refIdx, posLB, MD_LEFT, affiAMVPInfo, aiNeibeInherited);
  }
  leftAffNeiNum = 0;

  // check above neighbor
  if (!addAffineMVPCandUnscaled(cu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, affiAMVPInfo, aiNeibeInherited))
  {
    if (!addAffineMVPCandUnscaled(cu, eRefPicList, refIdx, posRT, MD_ABOVE, affiAMVPInfo, aiNeibeInherited))
    {
      addAffineMVPCandUnscaled(cu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, affiAMVPInfo, aiNeibeInherited);
    }
  }
  aboveAffNeiNum = 0;

  for (int affHMVPIdx = 0; affHMVPIdx < 1; affHMVPIdx++)
  {
    if (affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS && leftAffNeiNum > 0)
    {
      addSpatialAffineAMVPHMVPCand(cu, eRefPicList, refIdx, affiAMVPInfo, cu.cs->motionLut.lutAff, 0, leftNeiIdx,
                                   leftAffNeiNum, aiNeibeInherited, true);
    }
    if (affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS && aboveAffNeiNum > 0)
    {
      addSpatialAffineAMVPHMVPCand(cu, eRefPicList, refIdx, affiAMVPInfo, cu.cs->motionLut.lutAff, 0, aboveNeiIdx,
                                   aboveAffNeiNum, aiNeibeInherited, true);
    }
  }

  if (affiAMVPInfo.numCand >= AMVP_MAX_NUM_CANDS)
  {
    for (int i = 0; i < affiAMVPInfo.numCand; i++)
    {
      affiAMVPInfo.mvCandLT[i].roundAffinePrecInternal2Amvr(cu.imv);
      affiAMVPInfo.mvCandRT[i].roundAffinePrecInternal2Amvr(cu.imv);
      affiAMVPInfo.mvCandLB[i].roundAffinePrecInternal2Amvr(cu.imv);
    }
    return;
  }

  // insert constructed affine candidates
  int cornerMVPattern = 0;

  //-------------------  V0 (START) -------------------//
  AMVPInfo amvpInfo0;
  amvpInfo0.numCand = 0;

  // A->C: Above Left, Above, Left
  addMVPCandUnscaled(cu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, amvpInfo0);
  if (amvpInfo0.numCand < 1)
  {
    addMVPCandUnscaled(cu, eRefPicList, refIdx, posLT, MD_ABOVE, amvpInfo0);
  }
  if (amvpInfo0.numCand < 1)
  {
    addMVPCandUnscaled(cu, eRefPicList, refIdx, posLT, MD_LEFT, amvpInfo0);
  }
  cornerMVPattern = cornerMVPattern | amvpInfo0.numCand;

  //-------------------  V1 (START) -------------------//
  AMVPInfo amvpInfo1;
  amvpInfo1.numCand = 0;

  // D->E: Above, Above Right
  addMVPCandUnscaled(cu, eRefPicList, refIdx, posRT, MD_ABOVE, amvpInfo1);
  if (amvpInfo1.numCand < 1)
  {
    addMVPCandUnscaled(cu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, amvpInfo1);
  }
  cornerMVPattern = cornerMVPattern | (amvpInfo1.numCand << 1);

  //-------------------  V2 (START) -------------------//
  AMVPInfo amvpInfo2;
  amvpInfo2.numCand = 0;

  // F->G: Left, Below Left
  addMVPCandUnscaled(cu, eRefPicList, refIdx, posLB, MD_LEFT, amvpInfo2);
  if (amvpInfo2.numCand < 1)
  {
    addMVPCandUnscaled(cu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, amvpInfo2);
  }
  cornerMVPattern = cornerMVPattern | (amvpInfo2.numCand << 2);

  outputAffineMv[0] = amvpInfo0.mvCand[0];
  outputAffineMv[1] = amvpInfo1.mvCand[0];
  outputAffineMv[2] = amvpInfo2.mvCand[0];

  outputAffineMv[0].roundAffinePrecInternal2Amvr(cu.imv);
  outputAffineMv[1].roundAffinePrecInternal2Amvr(cu.imv);
  outputAffineMv[2].roundAffinePrecInternal2Amvr(cu.imv);

  if (cornerMVPattern == 7 || (cornerMVPattern == 3 && cu.affineType == AffineModel::_4_PARAMS))
  {
    affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[0];
    affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[1];
    affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[2];
    if (checkLastAffineAMVPCandRedundancy(cu, affiAMVPInfo))
    {
      affiAMVPInfo.numCand++;
    }
  }

  for (int affHMVPIdx = 1; affHMVPIdx < MAX_NUM_AFF_HMVP_CANDS; affHMVPIdx++)
  {
    if (affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS && leftAffNeiNum > 0)
    {
      addSpatialAffineAMVPHMVPCand(cu, eRefPicList, refIdx, affiAMVPInfo, cu.cs->motionLut.lutAff, 0, leftNeiIdx,
                                   leftAffNeiNum, aiNeibeInherited, true);
    }
    if (affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS && aboveAffNeiNum > 0)
    {
      addSpatialAffineAMVPHMVPCand(cu, eRefPicList, refIdx, affiAMVPInfo, cu.cs->motionLut.lutAff, 0, aboveNeiIdx,
                                   aboveAffNeiNum, aiNeibeInherited, true);
    }
  }

  if (affiAMVPInfo.numCand < 2)
  {
    // check corner MVs
    for (int i = 2; i >= 0 && affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS; i--)
    {
      if (cornerMVPattern & (1 << i))   // MV i exist
      {
        affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[i];
        affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[i];
        affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[i];
        if (checkLastAffineAMVPCandRedundancy(cu, affiAMVPInfo))
        {
          affiAMVPInfo.numCand++;
        }
      }
    }

    // Get Temporal Motion Predictor
    if (affiAMVPInfo.numCand < 2 && cu.cs->picHeader->m_enableTMVPFlag)
    {
      const int refIdxCol = refIdx;

      Position posRB = cu.Y().bottomRight().offset(-3, -3);

      const PreCalcValues &pcv = *cu.cs->pcv;

      Position posC0;
      bool     C0Avail = false;
      Position posC1   = cu.Y().center();
      Mv       cColMv;
      bool     boundaryCond =
        ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
      const SubPic &curSubPic = cu.cs->slice->m_pps->getSubPicFromPos(cu.lumaPos());
      if (curSubPic.m_treatedAsPicFlag)
      {
        boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.m_subPicRight &&
                        (posRB.y + pcv.minCUHeight) <= curSubPic.m_subPicBottom);
      }
      if (boundaryCond)
      {
        int posYInCtu = posRB.y & pcv.maxCUHeightMask;
        if (posYInCtu + 4 < pcv.maxCUHeight)
        {
          posC0   = posRB.offset(4, 4);
          C0Avail = true;
        }
      }
      if ((C0Avail && getColocatedMVP(cu, eRefPicList, posC0, cColMv, refIdxCol, false)) ||
          getColocatedMVP(cu, eRefPicList, posC1, cColMv, refIdxCol, false))
      {
        cColMv.roundAffinePrecInternal2Amvr(cu.imv);
        affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = cColMv;
        affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = cColMv;
        affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = cColMv;
        if (checkLastAffineAMVPCandRedundancy(cu, affiAMVPInfo))
        {
          affiAMVPInfo.numCand++;
        }
      }
    }

    if (affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS)
    {
      addNonAdjCstAffineMVPCandUnscaled(cu, eRefPicList, refIdx, affiAMVPInfo);
    }

    if (affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS)
    {
      for (int affHMVPIdx = 0; affHMVPIdx < MAX_NUM_AFF_HMVP_CANDS; affHMVPIdx++)
      {
        addSpatialAffineAMVPHMVPCand(cu, eRefPicList, refIdx, affiAMVPInfo, cu.cs->motionLut.lutAff, affHMVPIdx, neiIdx,
                                     5, aiNeibeInherited, false);
      }
    }
    if (affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS)
    {
      // add zero MV
      for (int i = affiAMVPInfo.numCand; i < AMVP_MAX_NUM_CANDS; i++)
      {
        affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand].setZero();
        affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand].setZero();
        affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand].setZero();
        affiAMVPInfo.numCand++;
      }
    }
  }

  for (int i = 0; i < affiAMVPInfo.numCand; i++)
  {
    affiAMVPInfo.mvCandLT[i].roundAffinePrecInternal2Amvr(cu.imv);
    affiAMVPInfo.mvCandRT[i].roundAffinePrecInternal2Amvr(cu.imv);
    affiAMVPInfo.mvCandLB[i].roundAffinePrecInternal2Amvr(cu.imv);
  }
}

bool PU::addMVPCandUnscaled(const CodingUnit &cu, const RefPicList &eRefPicList, const int &refIdx, const Position &pos,
                            const MvpDir &eDir, AMVPInfo &info)
{
  CodingStructure &cs = *cu.cs;

  const CodingUnit *neibPU = nullptr;
  Position          neibPos;

  switch (eDir)
  {
  case MD_LEFT:
    neibPos = pos.offset(-1, 0);
    break;
  case MD_ABOVE:
    neibPos = pos.offset(0, -1);
    break;
  case MD_ABOVE_RIGHT:
    neibPos = pos.offset(1, -1);
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset(-1, 1);
    break;
  case MD_ABOVE_LEFT:
    neibPos = pos.offset(-1, -1);
    break;
  default:
    break;
  }

  neibPU = cs.getCURestricted(neibPos, cu, cu.chType);

  if (neibPU == NULL || !CU::isInter(*neibPU) || !neibPU->getMotionInfo(neibPos).isInter)
  {
    return false;
  }

  const MotionInfo &neibMi = neibPU->getMotionInfo(neibPos);

  const int        currRefPOC     = cs.slice->getRefPic(eRefPicList, refIdx)->m_poc;
  const RefPicList eRefPicList2nd = (eRefPicList == RPL0) ? RPL1 : RPL0;

  for (int predictorSource = 0; predictorSource < 2;
       predictorSource++)   // examine the indicated reference picture list, then if not available, examine the other
                            // list.
  {
    const RefPicList eRefPicListIndex = (predictorSource == 0) ? eRefPicList : eRefPicList2nd;
    const int        neibRefIdx       = neibMi.refIdx[eRefPicListIndex];

    if (neibRefIdx >= 0 && currRefPOC == cs.slice->getRefPOC(eRefPicListIndex, neibRefIdx))
    {
      info.mvCand[info.numCand++] = neibMi.mv[eRefPicListIndex];
      return true;
    }
  }

  return false;
}

void PU::addAMVPHMVPCand(const CodingUnit &cu, const RefPicList eRefPicList, const int currRefPOC, AMVPInfo &info)
{
  const Slice &slice = *(*cu.cs).slice;

  MotionInfo       neibMi;
  auto            &lut               = CU::isIBC(cu) ? cu.cs->motionLut.lutIbc : cu.cs->motionLut.lut;
  int              numAvailCandInLut = (int)lut.size();
  int              numAllowedCand    = std::min(MAX_NUM_HMVP_AVMPCANDS, numAvailCandInLut);
  const RefPicList eRefPicList2nd    = (eRefPicList == RPL0) ? RPL1 : RPL0;

  for (int mrgIdx = 1; mrgIdx <= numAllowedCand; mrgIdx++)
  {
    if (info.numCand >= AMVP_MAX_NUM_CANDS)
    {
      return;
    }
    neibMi = lut[mrgIdx - 1];

    for (int predictorSource = 0; predictorSource < 2; predictorSource++)
    {
      const RefPicList eRefPicListIndex = (predictorSource == 0) ? eRefPicList : eRefPicList2nd;
      const int        neibRefIdx       = neibMi.refIdx[eRefPicListIndex];

      if (neibRefIdx >= 0 && (CU::isIBC(cu) || (currRefPOC == slice.getRefPOC(eRefPicListIndex, neibRefIdx))))
      {
        Mv pmv = neibMi.mv[eRefPicListIndex];
        pmv.roundTransPrecInternal2Amvr(cu.imv);

        info.mvCand[info.numCand++] = pmv;
        if (info.numCand >= AMVP_MAX_NUM_CANDS)
        {
          return;
        }
      }
    }
  }
}

bool PU::isBipredRestriction(const CodingUnit &cu) { return false; }

bool PU::getAffineControlPointCand(const CodingUnit &cu, MotionInfo mi[4], bool isAvailable[4], int verIdx[4],
                                   int8_t bcwIdx, int modelIdx, int verNum, AffineMergeCtx &affMrgType)
{
  int cuW = cu.lwidth();
  int cuH = cu.lheight();
  int vx, vy;
  int shift     = MAX_CU_DEPTH;
  int shiftHtoW = shift + floorLog2(cuW) - floorLog2(cuH);

  // motion info
  Mv          cMv[2][4];
  int         refIdx[2] = { -1, -1 };
  int         dir       = 0;
  AffineModel curType   = (verNum == 2) ? AffineModel::_4_PARAMS : AffineModel::_6_PARAMS;
  bool        LICFlag   = false;

  if (verNum == 2)
  {
    int idx0 = verIdx[0], idx1 = verIdx[1];
    if (!isAvailable[idx0] || !isAvailable[idx1])
    {
      return false;
    }

    for (int l = 0; l < 2; l++)
    {
      if (mi[idx0].refIdx[l] >= 0 && mi[idx1].refIdx[l] >= 0)
      {
        // check same refidx and different mv
        if (mi[idx0].refIdx[l] == mi[idx1].refIdx[l])
        {
          dir |= (l + 1);
          refIdx[l] = mi[idx0].refIdx[l];
        }
      }
    }

    LICFlag = mi[idx0].usesLIC || mi[idx1].usesLIC;
  }
  else if (verNum == 3)
  {
    int idx0 = verIdx[0], idx1 = verIdx[1], idx2 = verIdx[2];
    if (!isAvailable[idx0] || !isAvailable[idx1] || !isAvailable[idx2])
    {
      return false;
    }

    for (int l = 0; l < 2; l++)
    {
      if (mi[idx0].refIdx[l] >= 0 && mi[idx1].refIdx[l] >= 0 && mi[idx2].refIdx[l] >= 0)
      {
        // check same refidx and different mv
        if (mi[idx0].refIdx[l] == mi[idx1].refIdx[l] && mi[idx0].refIdx[l] == mi[idx2].refIdx[l])
        {
          dir |= (l + 1);
          refIdx[l] = mi[idx0].refIdx[l];
        }
      }
    }

    LICFlag = mi[idx0].usesLIC || mi[idx1].usesLIC || mi[idx2].usesLIC;
  }

  if (dir == 0)
  {
    return false;
  }

  for (int l = 0; l < 2; l++)
  {
    if (dir & (l + 1))
    {
      for (int i = 0; i < verNum; i++)
      {
        cMv[l][verIdx[i]] = mi[verIdx[i]].mv[l];
      }

      // convert to LT, RT[, [LB]]
      switch (modelIdx)
      {
      case 0:   // 0 : LT, RT, LB
        break;

      case 1:   // 1 : LT, RT, RB
        cMv[l][2].hor = cMv[l][3].hor + cMv[l][0].hor - cMv[l][1].hor;
        cMv[l][2].ver = cMv[l][3].ver + cMv[l][0].ver - cMv[l][1].ver;
        cMv[l][2].clipToStorageBitDepth();
        break;

      case 2:   // 2 : LT, LB, RB
        cMv[l][1].hor = cMv[l][3].hor + cMv[l][0].hor - cMv[l][2].hor;
        cMv[l][1].ver = cMv[l][3].ver + cMv[l][0].ver - cMv[l][2].ver;
        cMv[l][1].clipToStorageBitDepth();
        break;

      case 3:   // 3 : RT, LB, RB
        cMv[l][0].hor = cMv[l][1].hor + cMv[l][2].hor - cMv[l][3].hor;
        cMv[l][0].ver = cMv[l][1].ver + cMv[l][2].ver - cMv[l][3].ver;
        cMv[l][0].clipToStorageBitDepth();
        break;

      case 4:   // 4 : LT, RT
        {
          Mv mvLT, mvRT, mvLB;
          mvLT      = cMv[l][0];
          mvRT      = cMv[l][1];
          int shift = MAX_CU_DEPTH;
          int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;

          iDMvHorX = (mvRT - mvLT).getHor() << (shift - floorLog2(cu.lwidth()));
          iDMvHorY = (mvRT - mvLT).getVer() << (shift - floorLog2(cu.lwidth()));
          iDMvVerX = -iDMvHorY;
          iDMvVerY = iDMvHorX;

          int iMvScaleHor = mvLT.getHor() << shift;
          int iMvScaleVer = mvLT.getVer() << shift;
          int horTmp, verTmp;

          horTmp = iMvScaleHor + iDMvVerX * cu.lheight();
          verTmp = iMvScaleVer + iDMvVerY * cu.lheight();
          roundAffineMv(horTmp, verTmp, shift);
          cMv[l][2].hor = horTmp;
          cMv[l][2].ver = verTmp;
          cMv[l][2].clipToStorageBitDepth();
        }
        break;

      case 5:   // 5 : LT, LB
        vx = (cMv[l][0].hor * (1 << shift)) + ((cMv[l][2].ver - cMv[l][0].ver) * (1 << shiftHtoW));
        vy = (cMv[l][0].ver * (1 << shift)) - ((cMv[l][2].hor - cMv[l][0].hor) * (1 << shiftHtoW));
        cMv[l][1].set(vx, vy);
        cMv[l][1] >>= shift;
        cMv[l][1].clipToStorageBitDepth();
        break;

      default:
        THROW("Invalid model index!");
        break;
      }
    }
    else
    {
      for (int i = 0; i < 4; i++)
      {
        cMv[l][i].hor = 0;
        cMv[l][i].ver = 0;
      }
    }
  }

  for (int i = 0; i < 3; i++)
  {
    affMrgType.mvFieldNeighbours[affMrgType.numValidMergeCand][i][0].mv     = cMv[0][i];
    affMrgType.mvFieldNeighbours[affMrgType.numValidMergeCand][i][0].refIdx = refIdx[0];

    affMrgType.mvFieldNeighbours[affMrgType.numValidMergeCand][i][1].mv     = cMv[1][i];
    affMrgType.mvFieldNeighbours[affMrgType.numValidMergeCand][i][1].refIdx = refIdx[1];
  }
  affMrgType.interDirNeighbours[affMrgType.numValidMergeCand] = dir;
  affMrgType.affineType[affMrgType.numValidMergeCand]         = curType;
  affMrgType.bcwIdx[affMrgType.numValidMergeCand]             = (dir == 3) ? bcwIdx : BCW_DEFAULT;
  affMrgType.LICFlags[affMrgType.numValidMergeCand]           = LICFlag;
  if (!checkLastAffineMergeCandRedundancy(cu, affMrgType))
  {
    return false;
  }
  return true;
}

const int getNonAdjAvailableAffineNeighboursByDistance(const CodingUnit &pu, const CodingUnit *npu[],
                                                       AffineMergeCtx &affMrgCtx, const int mrgCandStart,
                                                       const int mrgCandIdx)
{
  const Position posLT        = pu.Y().topLeft();
  const unsigned plevel       = pu.cs->sps->m_log2ParallelMergeLevelMinus2 + 2;
  int            num          = mrgCandStart;
  uint8_t        cntLeftCand  = 0;
  uint8_t        cntAboveCand = 0;
  int            log2CtuSize  = floorLog2(pu.cs->sps->m_ctuSize);
  int            ctuX         = ((posLT.x >> log2CtuSize) << log2CtuSize);
  int            ctuY         = ((posLT.y >> log2CtuSize) << log2CtuSize);

  int offsetX = 0;
  int offsetY = 0;
  for (int pos = 1; pos < (AFF_NON_ADJACENT_DIST + 1); pos++)
  {
    bool isAboveAva = (cntAboveCand < AFF_MAX_NON_ADJACENT_INHERITED_CANDS);
    bool isLeftAva  = (cntLeftCand < AFF_MAX_NON_ADJACENT_INHERITED_CANDS);
    for (int posA = 0;; posA++)
    {
      offsetX = ((int)pu.lwidth()) * (pos + 1 - posA);
      offsetY = -((int)pu.lheight() * pos) - 1;
      if (offsetX < (-((int)pu.lwidth() * pos) - 1))
      {
        break;
      }

      const Position posTemp = PU::convertNonAdjAffineBlkPos(posLT.offset(offsetX, offsetY), ctuX, ctuY);
      if (posTemp == Position(-1, -1))
      {
        break;
      }
      const CodingUnit *puTemp = pu.cs->getCURestricted(posTemp, pu, pu.chType);
      if (puTemp && puTemp->affine && puTemp->mergeType == MergeType::DEFAULT_N &&
          PU::isDiffMER(pu.lumaPos(), posTemp, plevel))
      {
        bool redudant = false;
        for (int i = 0; i < num; i++)
        {
          if (puTemp == npu[i])
          {
            redudant = true;
            break;
          }
        }
        if (!redudant && isAboveAva)
        {
          npu[num++] = puTemp;
          cntAboveCand++;

          isAboveAva = (cntAboveCand < AFF_MAX_NON_ADJACENT_INHERITED_CANDS);
          if (!isAboveAva)
          {
            break;
          }
        }
      }
    }

    for (int posL = 0;; posL++)
    {
      offsetX = -((int)pu.lwidth() * pos);
      offsetY = ((int)pu.lheight()) * (pos + 1 - posL) - 1;
      if (offsetY < (-((int)pu.lheight() * pos) - 1))
      {
        break;
      }

      const Position posTemp = PU::convertNonAdjAffineBlkPos(posLT.offset(offsetX, offsetY), ctuX, ctuY);
      if (posTemp == Position(-1, -1))
      {
        break;
      }
      const CodingUnit *puTemp = pu.cs->getCURestricted(posTemp, pu, pu.chType);
      if (puTemp && puTemp->affine && puTemp->mergeType == MergeType::DEFAULT_N &&
          PU::isDiffMER(pu.lumaPos(), posTemp, plevel))
      {
        bool redudant = false;
        for (int i = 0; i < num; i++)
        {
          if (puTemp == npu[i])
          {
            redudant = true;
            break;
          }
        }
        if (!redudant && isLeftAva)
        {
          npu[num++] = puTemp;
          cntLeftCand++;

          isLeftAva = (cntLeftCand < AFF_MAX_NON_ADJACENT_INHERITED_CANDS);
          if (!isLeftAva)
          {
            break;
          }
        }
      }
    }
  }
  int lutSize = (int)pu.cs->motionLut.lutAffInherit.size();
  for (int listIdx = 0; listIdx < lutSize; listIdx++)
  {
    AffineInheritInfo &affHistInfo = pu.cs->motionLut.lutAffInherit[lutSize - 1 - listIdx];
    const Position     posTemp     = affHistInfo.basePos;
    const CodingUnit  *puTemp      = pu.cs->getCURestricted(posTemp, pu, pu.chType);
    if (puTemp && puTemp->affine && puTemp->mergeType == MergeType::DEFAULT_N &&
        PU::isDiffMER(pu.lumaPos(), posTemp, plevel))
    {
      bool redudant = false;
      for (int i = 0; i < num; i++)
      {
        if (puTemp == npu[i])
        {
          redudant = true;
          break;
        }
      }
      if (!redudant)
      {
        npu[num++] = puTemp;
      }
    }
  }
  return num;
}

Position PU::convertNonAdjAffineBlkPos(const Position &pos, int curCtuX, int curCtuY)
{
  if (pos.x < 0 || pos.y < 0)
  {
    return Position(-1, -1);
  }

  PosType newX = pos.x;
  PosType newY = pos.y;
  if (newY < curCtuY && newX < curCtuX)
  {
    return Position(curCtuX - 1, curCtuY - 1);
  }

  if (newY < curCtuY && newX >= curCtuX)
  {
    return Position(newX, curCtuY - 1);
  }

  if (newY >= curCtuY && newX < curCtuX)
  {
    return Position(curCtuX - 1, newY);
  }

  newX = ((newX >> 4) << 4);
  newY = ((newY >> 4) << 4);

  return Position(newX, newY);
}

int PU::getMvDiffThresholdByWidthAndHeight(const CodingUnit &cu, bool width)
{
  uint32_t numPixels = (width ? cu.lwidth() : cu.lheight());
  if (numPixels <= 8)
  {
    return (1 << MV_FRACTIONAL_BITS_INTERNAL) >> 4;
  }
  else if (numPixels <= 32)
  {
    return (1 << MV_FRACTIONAL_BITS_INTERNAL) >> 3;
  }
  else if (numPixels <= 64)
  {
    return (1 << MV_FRACTIONAL_BITS_INTERNAL) >> 2;
  }
  else
  {
    return (1 << MV_FRACTIONAL_BITS_INTERNAL) >> 1;
  }
}

bool PU::addNonAdjCstAffineMVPCandUnscaled(const CodingUnit &cu, const RefPicList &refPicList, const int &refIdx,
                                           AffineAMVPInfo &affiAmvpInfo)
{
  const Position posLT[3] = { cu.Y().topLeft().offset(-1, -1), cu.Y().topLeft().offset(0, -1),
                              cu.Y().topLeft().offset(-1, 0) };
  const Position posRT[2] = { cu.Y().topRight().offset(0, -1), cu.Y().topRight().offset(1, -1) };
  const Position posLB[2] = { cu.Y().bottomLeft().offset(-1, 0), cu.Y().bottomLeft().offset(-1, 1) };
  const unsigned plevel   = cu.cs->sps->m_log2ParallelMergeLevelMinus2 + 2;

  for (int i = 1; i < (AFF_NON_ADJACENT_DIST + 1); i++)
  {
    MotionInfo miNew[3];
    Position   posNew[3];
    bool       isAvailableNew[3] = { false, false, false };

    const Position    posTRNew     = Position(posRT[0].x, posRT[0].y - (i * ((int)cu.lheight())));
    const CodingUnit *puNeighTRNew = cu.cs->getCURestricted(posTRNew, cu, cu.chType);

    if (puNeighTRNew && CU::isInter(*puNeighTRNew) && PU::isDiffMER(cu.lumaPos(), posTRNew, plevel))
    {
      isAvailableNew[1] = true;
      miNew[1]          = puNeighTRNew->getMotionInfo(posTRNew);
      posNew[1]         = posTRNew;
      CHECK(posTRNew.x < 0 || posTRNew.y < 0, "posTRNew < 0");
    }

    for (int j = 1; j < (AFF_NON_ADJACENT_DIST + 1); j++)
    {
      isAvailableNew[0]              = false;
      isAvailableNew[2]              = false;
      const Position    posLBNew     = Position(posLB[0].x - (j * ((int)cu.lwidth())), posLB[0].y);
      const CodingUnit *puNeighLBNew = cu.cs->getCURestricted(posLBNew, cu, cu.chType);

      if (puNeighLBNew && CU::isInter(*puNeighLBNew) && PU::isDiffMER(cu.lumaPos(), posLBNew, plevel))
      {
        isAvailableNew[2] = true;
        miNew[2]          = puNeighLBNew->getMotionInfo(posLBNew);
        posNew[2]         = posLBNew;
        CHECK(posLBNew.x < 0 || posLBNew.y < 0, "posLBNew < 0");
      }

      PosType posX = isAvailableNew[2] ? posNew[2].x : (posLT[0].x - (j * ((int)cu.lwidth())));
      PosType posY = isAvailableNew[1] ? posNew[1].y : (posLT[0].y - (i * ((int)cu.lheight())));
      if (posX < 0)
      {
        posX = isAvailableNew[1] ? posLT[1].x : -1;
      }
      if (posY < 0)
      {
        posY = isAvailableNew[2] ? posLT[2].y : -1;
      }

      const Position    posLTNew     = Position(posX, posY);
      const CodingUnit *puNeighLTNew = cu.cs->getCURestricted(posLTNew, cu, cu.chType);

      if (puNeighLTNew && CU::isInter(*puNeighLTNew) && PU::isDiffMER(cu.lumaPos(), posLTNew, plevel))
      {
        isAvailableNew[0] = true;
        miNew[0]          = puNeighLTNew->getMotionInfo(posLTNew);
        posNew[0]         = posLTNew;
        CHECK(posLTNew.x < 0 || posLTNew.y < 0, "posLTNew < 0");
      }

      if (addNonAdjCstAffineMVPConstructedCPMV(cu, miNew, isAvailableNew, posNew, refPicList, refIdx, affiAmvpInfo))
      {
        if (affiAmvpInfo.numCand >= AMVP_MAX_NUM_CANDS)
        {
          return true;
        }
      }
    }
  }

  return false;
}

int PU::getNonAdjAffParaDivFun(int num1, int num2)
{
  int divTable[16] = { 0, 7, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 1, 1, 0 };
  int x            = floorLog2(num2);
  int normNum1     = (num2 << 4 >> x) & 15;
  int v            = divTable[normNum1] | 8;
  x += (normNum1 != 0);
  int shift  = 13 - x;
  int retVal = 0;
  if (shift < 0)
  {
    shift   = -shift;
    int add = (1 << (shift - 1));
    retVal  = (num1 * v + add) >> shift;
  }
  else
  {
    retVal = (num1 * v) << shift;
  }
  return (retVal >> (16 - MAX_CU_DEPTH));
}

bool PU::addNonAdjCstAffineMVPConstructedCPMV(const CodingUnit &cu, MotionInfo miNew[3], bool isAvaNew[3],
                                              Position pos[3], const RefPicList &refPicList, const int &refIdx,
                                              AffineAMVPInfo &affiAmvpInfo)
{
  if (!isAvaNew[0] || (!isAvaNew[1] && !isAvaNew[2]))
  {
    return false;
  }

  const int        currRefPOC    = cu.cs->slice->getRefPic(refPicList, refIdx)->m_poc;
  const RefPicList refPicList2nd = (refPicList == RPL0) ? RPL1 : RPL0;

  int shift   = MAX_CU_DEPTH;
  int posNeiX = pos[0].x;
  int posNeiY = pos[0].y;
  int posCurX = cu.Y().pos().x;
  int posCurY = cu.Y().pos().y;

  int curW = cu.lwidth();
  int curH = cu.lheight();
  int neiW = pos[1].x - pos[0].x;
  int neiH = pos[2].y - pos[0].y;
  if (!isAvaNew[1])
  {
    neiW = cu.Y().topRight().x - pos[0].x;
  }
  if (!isAvaNew[2])
  {
    neiH = cu.Y().bottomLeft().y - pos[0].y;
  }

  bool isConverted = false;

  for (int predictorSource = 0; predictorSource < 2; predictorSource++)
  {
    const RefPicList refListindex = (predictorSource == 0) ? refPicList : refPicList2nd;
    const int        neibRefIdx   = miNew[0].refIdx[refListindex];
    if (neibRefIdx < 0 || cu.slice->getRefPOC(refListindex, neibRefIdx) != currRefPOC)
    {
      continue;
    }
    for (int modelIdx = 0; modelIdx < 3; modelIdx++)
    {
      if ((modelIdx == 1 && !isAvaNew[1]) || (modelIdx == 2 && !isAvaNew[2]))
      {
        continue;
      }
      Mv  outputAffineMv[3];
      Mv  mvLT, mvRT, mvLB;
      int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;
      int horTmp, verTmp;

      mvLT = miNew[0].mv[refListindex];
      mvRT = miNew[1].mv[refListindex];
      mvLB = miNew[2].mv[refListindex];

      int iMvScaleHor = mvLT.getHor() << shift;
      int iMvScaleVer = mvLT.getVer() << shift;
      iDMvHorX        = getNonAdjAffParaDivFun((mvRT - mvLT).getHor(), neiW);
      iDMvHorY        = getNonAdjAffParaDivFun((mvRT - mvLT).getVer(), neiW);
      iDMvVerX        = getNonAdjAffParaDivFun((mvLB - mvLT).getHor(), neiH);
      iDMvVerY        = getNonAdjAffParaDivFun((mvLB - mvLT).getVer(), neiH);

      if (!modelIdx && isAvaNew[0] && isAvaNew[1] && isAvaNew[2] &&
          miNew[0].refIdx[refListindex] == miNew[1].refIdx[refListindex] &&
          miNew[0].refIdx[refListindex] == miNew[2].refIdx[refListindex])
      {}
      else if (modelIdx == 1 && isAvaNew[0] && isAvaNew[1] &&
               miNew[0].refIdx[refListindex] == miNew[1].refIdx[refListindex])
      {
        iDMvVerX = -iDMvHorY;
        iDMvVerY = iDMvHorX;
      }
      else if (modelIdx == 2 && isAvaNew[0] && isAvaNew[2] &&
               miNew[0].refIdx[refListindex] == miNew[2].refIdx[refListindex])
      {
        iDMvHorX = iDMvVerY;
        iDMvHorY = -iDMvVerX;
      }
      else
      {
        continue;
      }

      horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY - posNeiY);
      verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY - posNeiY);
      roundAffineMv(horTmp, verTmp, shift);
      outputAffineMv[0].hor = horTmp;
      outputAffineMv[0].ver = verTmp;
      outputAffineMv[0].clipToStorageBitDepth();

      horTmp = iMvScaleHor + iDMvHorX * (posCurX + curW - posNeiX) + iDMvVerX * (posCurY - posNeiY);
      verTmp = iMvScaleVer + iDMvHorY * (posCurX + curW - posNeiX) + iDMvVerY * (posCurY - posNeiY);
      roundAffineMv(horTmp, verTmp, shift);
      outputAffineMv[1].hor = horTmp;
      outputAffineMv[1].ver = verTmp;
      outputAffineMv[1].clipToStorageBitDepth();

      if (cu.affineType == AffineModel::_6_PARAMS)
      {
        horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY + curH - posNeiY);
        verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY + curH - posNeiY);
        roundAffineMv(horTmp, verTmp, shift);
        outputAffineMv[2].hor = horTmp;
        outputAffineMv[2].ver = verTmp;
        outputAffineMv[2].clipToStorageBitDepth();
      }

      outputAffineMv[0].roundAffinePrecInternal2Amvr(cu.imv);
      outputAffineMv[1].roundAffinePrecInternal2Amvr(cu.imv);
      if (cu.affineType == AffineModel::_6_PARAMS)
      {
        outputAffineMv[2].roundAffinePrecInternal2Amvr(cu.imv);
      }
      affiAmvpInfo.mvCandLT[affiAmvpInfo.numCand] = outputAffineMv[0];
      affiAmvpInfo.mvCandRT[affiAmvpInfo.numCand] = outputAffineMv[1];
      affiAmvpInfo.mvCandLB[affiAmvpInfo.numCand] = Mv();
      if (cu.affineType == AffineModel::_6_PARAMS)
      {
        affiAmvpInfo.mvCandLB[affiAmvpInfo.numCand] = outputAffineMv[2];
      }
      if (!checkLastAffineAMVPCandRedundancy(cu, affiAmvpInfo))
      {
        affiAmvpInfo.mvCandLT[affiAmvpInfo.numCand] = Mv();
        affiAmvpInfo.mvCandRT[affiAmvpInfo.numCand] = Mv();
        affiAmvpInfo.mvCandLB[affiAmvpInfo.numCand] = Mv();
        continue;
      }
      affiAmvpInfo.numCand++;
      isConverted = true;
      if (affiAmvpInfo.numCand >= AMVP_MAX_NUM_CANDS)
      {
        return true;
      }
    }
  }

  return isConverted;
}

bool PU::xCPMVSimCheck(const CodingUnit &cu, AffineMergeCtx &affMrgCtx, Mv curCpmv[2][3], uint8_t curdir,
                       int8_t curRefIdx[2], AffineModel curType, int bcwIdx, bool LICFlag)
{
  if (affMrgCtx.numValidMergeCand == 0 || affMrgCtx.mergeType[affMrgCtx.numValidMergeCand - 1] != MergeType::DEFAULT_N)
  {
    return false;
  }
  int iDMvHorX[2], iDMvHorY[2], iDMvVerX[2], iDMvVerY[2], iMvScaleHor[2], iMvScaleVer[2];
  int iDMvHorXCand, iDMvHorYCand, iDMvVerXCand, iDMvVerYCand, iMvScaleHorCand, iMvScaleVerCand;
  Mv  mvLT, mvRT, mvLB;

  const int mvDifThldWidth  = getMvDiffThresholdByWidthAndHeight(cu, true);
  const int mvDifThldHeight = getMvDiffThresholdByWidthAndHeight(cu, false);

  for (int l = 0; l < 2; l++)
  {
    if (curdir & (l + 1))
    {
      mvLT        = curCpmv[l][0];
      mvRT        = curCpmv[l][1];
      mvLB        = curCpmv[l][2];
      iDMvHorX[l] = (mvRT - mvLT).getHor();
      iDMvHorY[l] = (mvRT - mvLT).getVer();
      if (curType == AffineModel::_6_PARAMS)
      {
        iDMvVerX[l] = (mvLB - mvLT).getHor();
        iDMvVerY[l] = (mvLB - mvLT).getVer();
      }
      else
      {
        iDMvVerX[l] = -iDMvHorY[l];
        iDMvVerY[l] = iDMvHorX[l];
      }

      iMvScaleHor[l] = mvLT.getHor();
      iMvScaleVer[l] = mvLT.getVer();
    }
    else
    {
      iDMvHorX[l] = iDMvHorY[l] = 0;
      iDMvVerX[l] = iDMvVerY[l] = 0;
      iMvScaleHor[l] = iMvScaleVer[l] = 0;
    }
  }

  for (uint32_t ui = 0; ui < affMrgCtx.numValidMergeCand; ui++)
  {
    bool    isSimilar = true;
    uint8_t candDir   = affMrgCtx.interDirNeighbours[ui];
    int8_t  candRefIdx[2];
    if ((affMrgCtx.mergeType[ui] != MergeType::DEFAULT_N) || (candDir != curdir))
    {
      continue;
    }
    candRefIdx[0] = affMrgCtx.mvFieldNeighbours[ui][0][RPL0].refIdx;
    candRefIdx[1] = affMrgCtx.mvFieldNeighbours[ui][0][RPL1].refIdx;

    if ((candDir == 3 && (curRefIdx[0] != candRefIdx[0] || curRefIdx[1] != candRefIdx[1])) ||
        (candDir == 1 && curRefIdx[0] != candRefIdx[0]) || (candDir == 2 && curRefIdx[1] != candRefIdx[1]))
    {
      continue;
    }
    AffineModel candAffType = affMrgCtx.affineType[ui];
    for (int l = 0; l < 2; l++)
    {
      if (curdir & (l + 1))
      {
        mvLT         = affMrgCtx.mvFieldNeighbours[ui][0][l].mv;
        mvRT         = affMrgCtx.mvFieldNeighbours[ui][1][l].mv;
        mvLB         = affMrgCtx.mvFieldNeighbours[ui][2][l].mv;
        iDMvHorXCand = (mvRT - mvLT).getHor();
        iDMvHorYCand = (mvRT - mvLT).getVer();
        if (candAffType == AffineModel::_6_PARAMS)
        {
          iDMvVerXCand = (mvLB - mvLT).getHor();
          iDMvVerYCand = (mvLB - mvLT).getVer();
        }
        else
        {
          iDMvVerXCand = -iDMvHorYCand;
          iDMvVerYCand = iDMvHorXCand;
        }

        iMvScaleHorCand = mvLT.getHor();
        iMvScaleVerCand = mvLT.getVer();

        int diffHorX        = iDMvHorX[l] - iDMvHorXCand;
        int diffHorY        = iDMvHorY[l] - iDMvHorYCand;
        int diffVerX        = iDMvVerX[l] - iDMvVerXCand;
        int diffVerY        = iDMvVerY[l] - iDMvVerYCand;
        int diffiMvScaleHor = iMvScaleHor[l] - iMvScaleHorCand;
        int diffiMvScaleVer = iMvScaleVer[l] - iMvScaleVerCand;

        if (abs(diffHorX) >= mvDifThldWidth || abs(diffHorY) >= mvDifThldWidth || abs(diffVerX) >= mvDifThldHeight ||
            abs(diffVerY) >= mvDifThldHeight || abs(diffiMvScaleHor) >= 1 || abs(diffiMvScaleVer) >= 1)
        {
          isSimilar = false;
        }
        if (!isSimilar)
        {
          break;
        }
      }
    }
    if (isSimilar)
    {
      return true;
    }
  }

  return false;
}

bool PU::addNonAdjAffineConstructedCPMV(const CodingUnit &cu, MotionInfo miNew[4], bool isAvaNew[4], Position pos[4],
                                        int8_t bcwId, bool LICFlag, AffineMergeCtx &affMrgCtx, int mrgCandIdx)
{
  if (!isAvaNew[0] || (!isAvaNew[1] && !isAvaNew[2]))
  {
    return true;
  }
  int shift = MAX_CU_DEPTH;

  int posNeiX = pos[0].x;
  int posNeiY = pos[0].y;
  int posCurX = cu.Y().pos().x;
  int posCurY = cu.Y().pos().y;

  int curW = cu.lwidth();
  int curH = cu.lheight();
  int neiW = pos[1].x - pos[0].x;
  int neiH = pos[2].y - pos[0].y;
  if (!isAvaNew[1])
  {
    neiW = cu.Y().topRight().x - pos[0].x;
  }
  if (!isAvaNew[2])
  {
    neiH = cu.Y().bottomLeft().y - pos[0].y;
  }

  bool isConverted = false;
  for (int modelIdx = 0; modelIdx < 3; modelIdx++)
  {
    if ((modelIdx == 1 && !isAvaNew[1]) || (modelIdx == 2 && !isAvaNew[2]))
    {
      continue;
    }
    Mv          cMv[2][3];
    int8_t      refIdx[2] = { -1, -1 };
    int         dir       = 0;
    AffineModel curType   = AffineModel::_6_PARAMS;
    bool        bLICFlag  = false;

    for (int l = 0; l < 2; l++)
    {
      Mv  mvLT, mvRT, mvLB;
      int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;
      int horTmp, verTmp;

      mvLT = miNew[0].mv[l];
      mvRT = miNew[1].mv[l];
      mvLB = miNew[2].mv[l];

      int iMvScaleHor = mvLT.getHor() << shift;
      int iMvScaleVer = mvLT.getVer() << shift;
      iDMvHorX        = getNonAdjAffParaDivFun((mvRT - mvLT).getHor(), neiW);
      iDMvHorY        = getNonAdjAffParaDivFun((mvRT - mvLT).getVer(), neiW);
      iDMvVerX        = getNonAdjAffParaDivFun((mvLB - mvLT).getHor(), neiH);
      iDMvVerY        = getNonAdjAffParaDivFun((mvLB - mvLT).getVer(), neiH);

      if (!modelIdx && isAvaNew[0] && isAvaNew[1] && isAvaNew[2] && miNew[0].refIdx[l] >= 0 &&
          miNew[0].refIdx[l] == miNew[1].refIdx[l] && miNew[0].refIdx[l] == miNew[2].refIdx[l])
      {}
      else if (modelIdx == 1 && isAvaNew[0] && isAvaNew[1] && miNew[0].refIdx[l] >= 0 &&
               miNew[0].refIdx[l] == miNew[1].refIdx[l])
      {
        iDMvVerX = -iDMvHorY;
        iDMvVerY = iDMvHorX;
      }
      else if (modelIdx == 2 && isAvaNew[0] && isAvaNew[2] && miNew[0].refIdx[l] >= 0 &&
               miNew[0].refIdx[l] == miNew[2].refIdx[l])
      {
        iDMvHorX = iDMvVerY;
        iDMvHorY = -iDMvVerX;
      }
      else
      {
        continue;
      }

      dir |= (l + 1);
      refIdx[l] = miNew[0].refIdx[l];

      horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY - posNeiY);
      verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY - posNeiY);
      roundAffineMv(horTmp, verTmp, shift);
      cMv[l][0].hor = horTmp;
      cMv[l][0].ver = verTmp;
      cMv[l][0].clipToStorageBitDepth();

      horTmp = iMvScaleHor + iDMvHorX * (posCurX + curW - posNeiX) + iDMvVerX * (posCurY - posNeiY);
      verTmp = iMvScaleVer + iDMvHorY * (posCurX + curW - posNeiX) + iDMvVerY * (posCurY - posNeiY);
      roundAffineMv(horTmp, verTmp, shift);
      cMv[l][1].hor = horTmp;
      cMv[l][1].ver = verTmp;
      cMv[l][1].clipToStorageBitDepth();

      {
        horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY + curH - posNeiY);
        verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY + curH - posNeiY);
        roundAffineMv(horTmp, verTmp, shift);
        cMv[l][2].hor = horTmp;
        cMv[l][2].ver = verTmp;
        cMv[l][2].clipToStorageBitDepth();
      }
    }

    if (!dir)
    {
      continue;
    }
    if (xCPMVSimCheck(cu, affMrgCtx, cMv, dir, refIdx, curType, (dir == 3) ? bcwId : BCW_DEFAULT,
                      (dir != 3) ? bLICFlag : false))
    {
      continue;
    }
    isConverted = true;
    for (int i = 0; i < 3; i++)
    {
      affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][i][RPL0].mv     = cMv[0][i];
      affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][i][RPL0].refIdx = refIdx[0];

      affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][i][RPL1].mv     = cMv[1][i];
      affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][i][RPL1].refIdx = refIdx[1];
    }
    affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = dir;
    affMrgCtx.affineType[affMrgCtx.numValidMergeCand]         = curType;
    affMrgCtx.mergeType[affMrgCtx.numValidMergeCand]          = MergeType::DEFAULT_N;
    affMrgCtx.bcwIdx[affMrgCtx.numValidMergeCand]             = (dir == 3) ? bcwId : BCW_DEFAULT;
    affMrgCtx.LICFlags[affMrgCtx.numValidMergeCand]           = LICFlag;

    affMrgCtx.numValidMergeCand++;

    if (affMrgCtx.numValidMergeCand != 0 && affMrgCtx.numValidMergeCand - 1 == mrgCandIdx)
    {
      return false;
    }

    if (affMrgCtx.numValidMergeCand == affMrgCtx.maxNumMergeCand)
    {
      return false;
    }
    break;
  }

  return (!isConverted);
}

void PU::getNonAdjCstMergeCand(const CodingUnit &cu, AffineMergeCtx &affMrgCtx, const int mrgCandIdx,
                               bool isInitialized)
{
  const CodingStructure &cs                    = *cu.cs;
  const Slice           &slice                 = *cu.cs->slice;
  const uint32_t         maxNumAffineMergeCand = slice.m_picHeader->m_maxNumAffineMergeCand;
  const unsigned         plevel                = cu.cs->sps->m_log2ParallelMergeLevelMinus2 + 2;
  const Position         posLT[3]              = { cu.Y().topLeft().offset(-1, -1), cu.Y().topLeft().offset(0, -1),
                                                   cu.Y().topLeft().offset(-1, 0) };
  const Position         posRT[2]              = { cu.Y().topRight().offset(0, -1), cu.Y().topRight().offset(1, -1) };
  const Position         posLB[2] = { cu.Y().bottomLeft().offset(-1, 0), cu.Y().bottomLeft().offset(-1, 1) };
  if (!isInitialized)
  {
    for (int i = 0; i < maxNumAffineMergeCand; i++)
    {
      for (int mvNum = 0; mvNum < 3; mvNum++)
      {
        affMrgCtx.mvFieldNeighbours[i][mvNum][RPL0].setMvField(Mv(), -1);
        affMrgCtx.mvFieldNeighbours[i][mvNum][RPL1].setMvField(Mv(), -1);
      }
      affMrgCtx.interDirNeighbours[i] = 0;
      affMrgCtx.affineType[i]         = AffineModel::_4_PARAMS;
      affMrgCtx.mergeType[i]          = MergeType::DEFAULT_N;
      affMrgCtx.bcwIdx[i]             = BCW_DEFAULT;
      affMrgCtx.LICFlags[i]           = false;
    }

    affMrgCtx.numValidMergeCand = 0;
    affMrgCtx.maxNumMergeCand   = maxNumAffineMergeCand;
  }

  for (int i = 1; i < (AFF_NON_ADJACENT_DIST + 1); i++)
  {
    MotionInfo miNew[4];
    Position   posNew[4];
    bool       isAvailableNew[4] = { false, false, false, false };
    int8_t     neighBcwNew[2]    = { BCW_DEFAULT, BCW_DEFAULT };
    bool       neighLicNew       = false;

    const Position    posTRNew     = Position(posRT[0].x, posRT[0].y - (i * ((int)cu.lheight())));
    const CodingUnit *puNeighTRNew = cs.getCURestricted(posTRNew, cu, cu.chType);

    if (puNeighTRNew && CU::isInter(*puNeighTRNew) && PU::isDiffMER(cu.lumaPos(), posTRNew, plevel))
    {
      isAvailableNew[1] = true;
      miNew[1]          = puNeighTRNew->getMotionInfo(posTRNew);
      neighBcwNew[1]    = puNeighTRNew->bcwIdx;
      posNew[1]         = posTRNew;
      CHECK(posTRNew.x < 0 || posTRNew.y < 0, "posTRNew < 0");
    }

    for (int j = 1; j < (AFF_NON_ADJACENT_DIST + 1); j++)
    {
      isAvailableNew[0]              = false;
      isAvailableNew[2]              = false;
      const Position    posLBNew     = Position(posLB[0].x - (j * ((int)cu.lwidth())), posLB[0].y);
      const CodingUnit *puNeighLBNew = cs.getCURestricted(posLBNew, cu, cu.chType);

      if (puNeighLBNew && CU::isInter(*puNeighLBNew) && PU::isDiffMER(cu.lumaPos(), posLBNew, plevel))
      {
        isAvailableNew[2] = true;
        miNew[2]          = puNeighLBNew->getMotionInfo(posLBNew);
        posNew[2]         = posLBNew;
        CHECK(posLBNew.x < 0 || posLBNew.y < 0, "posLBNew < 0");
      }

      PosType posX = isAvailableNew[2] ? posNew[2].x : (posLT[0].x - (j * ((int)cu.lwidth())));
      PosType posY = isAvailableNew[1] ? posNew[1].y : (posLT[0].y - (i * ((int)cu.lheight())));
      if (posX < 0)
      {
        posX = isAvailableNew[1] ? posLT[1].x : -1;
      }
      if (posY < 0)
      {
        posY = isAvailableNew[2] ? posLT[2].y : -1;
      }

      const Position    posLTNew     = Position(posX, posY);
      const CodingUnit *puNeighLTNew = cs.getCURestricted(posLTNew, cu, cu.chType);

      if (puNeighLTNew && CU::isInter(*puNeighLTNew) && PU::isDiffMER(cu.lumaPos(), posLTNew, plevel))
      {
        isAvailableNew[0] = true;
        miNew[0]          = puNeighLTNew->getMotionInfo(posLTNew);
        neighBcwNew[0]    = puNeighLTNew->bcwIdx;
        neighLicNew       = puNeighLTNew->licFlag;
        posNew[0]         = posLTNew;
        CHECK(posLTNew.x < 0 || posLTNew.y < 0, "posLTNew < 0");
      }

      if (!addNonAdjAffineConstructedCPMV(cu, miNew, isAvailableNew, posNew, neighBcwNew[0], neighLicNew, affMrgCtx,
                                          mrgCandIdx))
      {
        if (affMrgCtx.numValidMergeCand != 0 && affMrgCtx.numValidMergeCand - 1 == mrgCandIdx)
        {
          return;
        }

        if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
        {
          return;
        }
      }
    }
  }
}

const int getAvailableAffineNeighboursForLeftPredictor(const CodingUnit &cu, const CodingUnit *npu[], int neiIdx[])
{
  const Position posLB  = cu.Y().bottomLeft();
  int            num    = 0;
  const unsigned plevel = cu.cs->sps->m_log2ParallelMergeLevelMinus2 + 2;

  const CodingUnit *puLeftBottom = cu.cs->getCURestricted(posLB.offset(-1, 1), cu, cu.chType);
  if (puLeftBottom && puLeftBottom->isAffineBlock() && PU::isDiffMER(cu.lumaPos(), posLB.offset(-1, 1), plevel))
  {
    neiIdx[num] = 3;
    npu[num++]  = puLeftBottom;
    return num;
  }

  const CodingUnit *puLeft = cu.cs->getCURestricted(posLB.offset(-1, 0), cu, cu.chType);
  if (puLeft && puLeft->isAffineBlock() && PU::isDiffMER(cu.lumaPos(), posLB.offset(-1, 0), plevel))
  {
    neiIdx[num] = 0;
    npu[num++]  = puLeft;
    return num;
  }

  return num;
}

const int getAvailableAffineNeighboursForAbovePredictor(const CodingUnit &cu, const CodingUnit *npu[],
                                                        int numAffNeighLeft, int neiIdx[])
{
  const Position posLT  = cu.Y().topLeft();
  const Position posRT  = cu.Y().topRight();
  const unsigned plevel = cu.cs->sps->m_log2ParallelMergeLevelMinus2 + 2;
  int            num    = numAffNeighLeft;

  const CodingUnit *puAboveRight = cu.cs->getCURestricted(posRT.offset(1, -1), cu, cu.chType);
  if (puAboveRight && puAboveRight->isAffineBlock() && PU::isDiffMER(cu.lumaPos(), posRT.offset(1, -1), plevel))
  {
    neiIdx[num] = 2;
    npu[num++]  = puAboveRight;
    return num;
  }

  const CodingUnit *puAbove = cu.cs->getCURestricted(posRT.offset(0, -1), cu, cu.chType);
  if (puAbove && puAbove->isAffineBlock() && PU::isDiffMER(cu.lumaPos(), posRT.offset(0, -1), plevel))
  {
    neiIdx[num] = 1;
    npu[num++]  = puAbove;
    return num;
  }

  const CodingUnit *puAboveLeft = cu.cs->getCURestricted(posLT.offset(-1, -1), cu, cu.chType);
  if (puAboveLeft && puAboveLeft->isAffineBlock() && PU::isDiffMER(cu.lumaPos(), posLT.offset(-1, -1), plevel))
  {
    neiIdx[num] = 4;
    npu[num++]  = puAboveLeft;
    return num;
  }

  return num;
}

void PU::getAffineMergeCand(const CodingUnit &cu, AffineMergeCtx &affMrgCtx, const int mrgCandIdx, bool isAffMmvd)
{
  const CodingStructure &cs                    = *cu.cs;
  const Slice           &slice                 = *cu.cs->slice;
  const uint32_t         maxNumAffineMergeCand = slice.m_picHeader->m_maxNumAffineMergeCand;
  const unsigned         plevel                = cu.cs->sps->m_log2ParallelMergeLevelMinus2 + 2;

  for (int i = 0; i < RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE; i++)
  {
    for (int mvNum = 0; mvNum < 3; mvNum++)
    {
      affMrgCtx.mvFieldNeighbours[i][mvNum][0].setMvField(Mv(), -1);
      affMrgCtx.mvFieldNeighbours[i][mvNum][1].setMvField(Mv(), -1);
    }
    affMrgCtx.interDirNeighbours[i] = 0;
    affMrgCtx.affineType[i]         = AffineModel::_4_PARAMS;
    affMrgCtx.mergeType[i]          = MergeType::DEFAULT_N;
    affMrgCtx.bcwIdx[i]             = BCW_DEFAULT;
    affMrgCtx.LICFlags[i]           = false;
    affMrgCtx.numAffCandToTestEnc   = maxNumAffineMergeCand;
    affMrgCtx.candCost[i]           = MAX_UINT64;
  }

  affMrgCtx.numValidMergeCand = 0;
  affMrgCtx.maxNumMergeCand   = maxNumAffineMergeCand;

  bool sbTmvpEnableFlag =
    slice.m_sps->m_sbtmvpEnabledFlag && !(slice.m_poc == slice.getRefPic(RPL0, 0)->m_poc && slice.isIRAP());
  bool isAvailableSubPu = false;
  if (sbTmvpEnableFlag && slice.m_picHeader->m_enableTMVPFlag)
  {
    int pos = 0;

    CHECK(affMrgCtx.subPuMvpMiBuf.area() == 0 || !affMrgCtx.subPuMvpMiBuf.buf, "Buffer not initialized");
    affMrgCtx.subPuMvpMiBuf.fill(MotionInfo());
    // Get spatial MV
    const Position posCurLB = cu.Y().bottomLeft();
    MotionInfo     miLeft;

    // left
    const CodingUnit *puLeft = cs.getCURestricted(posCurLB.offset(-1, 0), cu, cu.chType);
    const bool isAvailableA1 = puLeft && isDiffMER(cu.lumaPos(), posCurLB.offset(-1, 0), plevel) && &cu != puLeft &&
      CU::isInter(*puLeft) && puLeft->getMotionInfo(posCurLB.offset(-1, 0)).isInter;
    if (isAvailableA1)
    {
      miLeft                            = puLeft->getMotionInfo(posCurLB.offset(-1, 0));
      // get Inter Dir
      affMrgCtx.interDirNeighbours[pos] = miLeft.interDir;

      // get Mv from Left
      affMrgCtx.mvFieldNeighbours[pos][0][0].setMvField(miLeft.mv[0], miLeft.refIdx[0]);

      if (slice.isInterB())
      {
        affMrgCtx.mvFieldNeighbours[pos][1][0].setMvField(miLeft.mv[1], miLeft.refIdx[1]);
      }
      pos++;
    }

    isAvailableSubPu = getInterMergeSubPuMvpCand(cu, affMrgCtx, pos);
    if (isAvailableSubPu)
    {
      for (int mvNum = 1; mvNum < 3; mvNum++)
      {
        affMrgCtx.mvFieldNeighbours[0][mvNum][0].setMvField(affMrgCtx.mvFieldNeighbours[0][0][0].mv,
                                                            affMrgCtx.mvFieldNeighbours[0][0][0].refIdx);
        affMrgCtx.mvFieldNeighbours[0][mvNum][1].setMvField(affMrgCtx.mvFieldNeighbours[0][0][1].mv,
                                                            affMrgCtx.mvFieldNeighbours[0][0][1].refIdx);
      }

      affMrgCtx.affineType[affMrgCtx.numValidMergeCand] = AffineModel::NUM;
      affMrgCtx.mergeType[affMrgCtx.numValidMergeCand]  = MergeType::SUBPU_ATMVP;
      CHECK(affMrgCtx.LICFlags[affMrgCtx.numValidMergeCand], "LIC flag is set for ATMVP");

      affMrgCtx.numValidMergeCand++;
      const int affMmvdIdxOffset = (isAffMmvd && mrgCandIdx >= 0 ? 1 : 0);

      if (affMrgCtx.numValidMergeCand == mrgCandIdx + 1 + affMmvdIdxOffset)
      {
        return;
      }

      // early termination
      if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
      {
        return;
      }
    }
    else
    {
      for (int mvNum = 0; mvNum < 3; mvNum++)
      {
        affMrgCtx.mvFieldNeighbours[0][mvNum][0].setMvField(Mv(), -1);
        affMrgCtx.mvFieldNeighbours[0][mvNum][1].setMvField(Mv(), -1);
      }
      affMrgCtx.interDirNeighbours[0] = 0;
      affMrgCtx.affineType[0]         = AffineModel::_4_PARAMS;
      affMrgCtx.mergeType[0]          = MergeType::DEFAULT_N;
      affMrgCtx.bcwIdx[0]             = BCW_DEFAULT;
      affMrgCtx.LICFlags[0]           = false;
    }
  }

  if (slice.m_sps->m_useAffine)
  {
    ///> Start: inherited affine candidates
    const int         CHECKED_NEI_NUM = 7;
    const CodingUnit *npu[CHECKED_NEI_NUM + AFF_MAX_NON_ADJACENT_INHERITED_CANDS + MAX_NUM_AFF_INHERIT_HMVP_CANDS];
    const CodingUnit *npuGroup2[CHECKED_NEI_NUM];
    Position          posGroup2[CHECKED_NEI_NUM];
    int               numGroup2;
    numGroup2 = 0;
    int neiIdx[CHECKED_NEI_NUM];
    int aiNeibeInherited[CHECKED_NEI_NUM];
    memset(aiNeibeInherited, 0, sizeof(aiNeibeInherited));

    const Position posLB = cu.Y().bottomLeft();
    const Position posLT = cu.Y().topLeft();
    const Position posRT = cu.Y().topRight();

    Position neiPositions[CHECKED_NEI_NUM] = { posLB.offset(-2, -1), posRT.offset(-1, -2), posRT.offset(3, -2),
                                               posLB.offset(-2, 3),  posLT.offset(-2, -2), posLT.offset(-2, 1),
                                               posLT.offset(1, -2) };

    int numAffNeighLeft = getAvailableAffineNeighboursForLeftPredictor(cu, npu, neiIdx);
    if (numAffNeighLeft > 0)
    {
      aiNeibeInherited[neiIdx[0]] = 1;
    }
    int numAffNeigh = getAvailableAffineNeighboursForAbovePredictor(cu, npu, numAffNeighLeft, neiIdx);
    if (numAffNeigh > 0)
    {
      aiNeibeInherited[neiIdx[numAffNeigh - 1]] = 1;
    }
    for (int nei = 0; nei < CHECKED_NEI_NUM; nei++)
    {
      const CodingUnit *puNei = cu.cs->getCURestricted(neiPositions[nei], cu, cu.chType);
      if (aiNeibeInherited[nei])
      {
        continue;
      }
      if (!puNei)
      {
        continue;
      }
      if (puNei->predMode != MODE_INTER)
      {
        continue;
      }
      MotionInfo mvInfo = puNei->getMotionInfo(neiPositions[nei]);
      if (!mvInfo.isInter || mvInfo.interDir <= 0 || mvInfo.interDir > 3)
      {
        continue;
      }
      posGroup2[numGroup2]   = neiPositions[nei];
      npuGroup2[numGroup2++] = puNei;
    }
    for (int idx = 0; idx < numAffNeigh; idx++)
    {
      // derive Mv from Neigh affine PU
      Mv                cMv[2][3];
      const CodingUnit *puNeigh               = npu[idx];
      const_cast<CodingUnit &>(cu).affineType = puNeigh->affineType;
      if (puNeigh->interDir != 2)
      {
        xInheritedAffineMv(cu, puNeigh, RPL0, cMv[0]);
      }
      if (slice.isInterB())
      {
        if (puNeigh->interDir != 1)
        {
          xInheritedAffineMv(cu, puNeigh, RPL1, cMv[1]);
        }
      }

      for (int mvNum = 0; mvNum < 3; mvNum++)
      {
        affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][mvNum][0].setMvField(cMv[0][mvNum],
                                                                                      puNeigh->refIdx[0]);
        affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][mvNum][1].setMvField(cMv[1][mvNum],
                                                                                      puNeigh->refIdx[1]);
      }
      affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = puNeigh->interDir;
      affMrgCtx.affineType[affMrgCtx.numValidMergeCand]         = puNeigh->affineType;
      affMrgCtx.bcwIdx[affMrgCtx.numValidMergeCand]             = puNeigh->bcwIdx;
      CHECK(!cu.cs->sps->m_biLicEnabledFlag && puNeigh->interDir == 3 && puNeigh->licFlag,
            "LIC should not be enabled for affine bi-pred");
      affMrgCtx.LICFlags[affMrgCtx.numValidMergeCand] = puNeigh->licFlag;

      if (!checkLastAffineMergeCandRedundancy(cu, affMrgCtx))
      {
        continue;
      }
      if (affMrgCtx.numValidMergeCand == mrgCandIdx)
      {
        return;
      }

      // early termination
      affMrgCtx.numValidMergeCand++;
      if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
      {
        return;
      }
    }
    int numAffNeighExtend2 = getNonAdjAvailableAffineNeighboursByDistance(cu, npu, affMrgCtx, 0, -1);
    if (numAffNeighExtend2 > 0)
    {
#if 0   // variable affMrgCtxTemp is unused
      AffineMergeCtx affMrgCtxTemp;
      for (int i = 0; i < RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE; i++)
      {
        for (int mvNum = 0; mvNum < 3; mvNum++)
        {
          affMrgCtxTemp.mvFieldNeighbours[i][mvNum][0].setMvField(Mv(), -1);
          affMrgCtxTemp.mvFieldNeighbours[i][mvNum][1].setMvField(Mv(), -1);
        }
        affMrgCtxTemp.interDirNeighbours[i] = 0;
        affMrgCtxTemp.affineType[i] = AffineModel::_4_PARAMS;
        affMrgCtxTemp.mergeType[i] = MergeType::DEFAULT_N;
        affMrgCtxTemp.bcwIdx[i] = BCW_DEFAULT;
        affMrgCtxTemp.LICFlags[i] = false;
        affMrgCtxTemp.candCost[i] = MAX_UINT64;
      }
      affMrgCtxTemp.numValidMergeCand = 0;
      affMrgCtxTemp.maxNumMergeCand = RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE;
#endif

      std::vector<RMVFInfo> mvpInfoVec[2][4];
      collectNeiMotionInfo(mvpInfoVec, cu);

      int counter  = 0;
      int numAdded = 0;
      for (int i = 0; i < numAffNeighExtend2; i++)
      {
        numAdded = affMrgCtx.numValidMergeCand;
        getRMVFAffineGuideCand(cu, *npu[i], affMrgCtx, mvpInfoVec, mrgCandIdx);
        if (affMrgCtx.numValidMergeCand != 0 && (affMrgCtx.numValidMergeCand - 1 == mrgCandIdx))
        {
          return;
        }
        if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
        {
          return;
        }
        counter += affMrgCtx.numValidMergeCand - numAdded;
        if (counter >= numAffNeighExtend2)
        {
          break;
        }
      }
    }

    MotionInfo tmvpInfo;
    tmvpInfo.interDir  = 0;
    tmvpInfo.refIdx[0] = tmvpInfo.refIdx[1] = -1;
    bool isTmvpAvailable                    = false;
    ///> End: inherited affine candidates

    ///> Start: Constructed affine candidates
    {
      MotionInfo mi[4];
      bool       isAvailable[4] = { false };

      int8_t         neighBcw[2] = { BCW_DEFAULT, BCW_DEFAULT };
      // control point: LT B2->B3->A2
      const Position posLT[3]    = { cu.Y().topLeft().offset(-1, -1), cu.Y().topLeft().offset(0, -1),
                                     cu.Y().topLeft().offset(-1, 0) };
      for (int i = 0; i < 3; i++)
      {
        const Position    pos     = posLT[i];
        const CodingUnit *puNeigh = cs.getCURestricted(pos, cu, cu.chType);

        if (puNeigh && CU::isInter(*puNeigh) && puNeigh->getMotionInfo(pos).isInter &&
            PU::isDiffMER(cu.lumaPos(), pos, plevel))
        {
          isAvailable[0] = true;
          mi[0]          = puNeigh->getMotionInfo(pos);
          neighBcw[0]    = puNeigh->bcwIdx;
          if (puNeigh->interDir != 3)
          {
            neighBcw[0] = BCW_DEFAULT;
          }
          break;
        }
      }

      // control point: RT B1->B0
      const Position posRT[2] = { cu.Y().topRight().offset(0, -1), cu.Y().topRight().offset(1, -1) };
      for (int i = 0; i < 2; i++)
      {
        const Position    pos     = posRT[i];
        const CodingUnit *puNeigh = cs.getCURestricted(pos, cu, cu.chType);

        if (puNeigh && CU::isInter(*puNeigh) && puNeigh->getMotionInfo(pos).isInter &&
            PU::isDiffMER(cu.lumaPos(), pos, plevel))
        {
          isAvailable[1] = true;
          mi[1]          = puNeigh->getMotionInfo(pos);
          neighBcw[1]    = puNeigh->bcwIdx;
          if (puNeigh->interDir != 3)
          {
            neighBcw[1] = BCW_DEFAULT;
          }
          break;
        }
      }

      // control point: LB A1->A0
      const Position posLB[2] = { cu.Y().bottomLeft().offset(-1, 0), cu.Y().bottomLeft().offset(-1, 1) };
      for (int i = 0; i < 2; i++)
      {
        const Position    pos     = posLB[i];
        const CodingUnit *puNeigh = cs.getCURestricted(pos, cu, cu.chType);

        if (puNeigh && CU::isInter(*puNeigh) && puNeigh->getMotionInfo(pos).isInter &&
            PU::isDiffMER(cu.lumaPos(), pos, plevel))
        {
          isAvailable[2] = true;
          mi[2]          = puNeigh->getMotionInfo(pos);
          break;
        }
      }

      // control point: RB
      if (slice.m_picHeader->m_enableTMVPFlag)
      {
        //>> MTK colocated-RightBottom
        // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to
        Position posRB = cu.Y().bottomRight().offset(-3, -3);

        const PreCalcValues &pcv = *cs.pcv;
        Position             posC0;
        bool                 C0Avail = false;

        bool boundaryCond =
          ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
        const SubPic &curSubPic = cu.cs->slice->m_pps->getSubPicFromPos(cu.lumaPos());
        if (curSubPic.m_treatedAsPicFlag)
        {
          boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.m_subPicRight &&
                          (posRB.y + pcv.minCUHeight) <= curSubPic.m_subPicBottom);
        }
        if (boundaryCond)
        {
          int posYInCtu = posRB.y & pcv.maxCUHeightMask;
          if (posYInCtu + 4 < pcv.maxCUHeight)
          {
            posC0   = posRB.offset(4, 4);
            C0Avail = true;
          }
        }

        Mv   cColMv;
        int  refIdx  = 0;
        bool existMV = C0Avail && getColocatedMVP(cu, RPL0, posC0, cColMv, refIdx, false);

        if (existMV)
        {
          mi[3].mv[0]        = cColMv;
          mi[3].refIdx[0]    = refIdx;
          mi[3].interDir     = 1;
          isAvailable[3]     = true;
          tmvpInfo.mv[0]     = cColMv;
          tmvpInfo.refIdx[0] = refIdx;
          tmvpInfo.interDir |= 1;
          isTmvpAvailable = true;
        }

        if (slice.isInterB())
        {
          existMV = C0Avail && getColocatedMVP(cu, RPL1, posC0, cColMv, refIdx, false);
          if (existMV)
          {
            mi[3].mv[1]     = cColMv;
            mi[3].refIdx[1] = refIdx;
            mi[3].interDir |= 2;
            isAvailable[3]     = true;
            tmvpInfo.mv[1]     = cColMv;
            tmvpInfo.refIdx[1] = refIdx;
            tmvpInfo.interDir |= 2;
            isTmvpAvailable = true;
          }
        }
        mi[3].usesLIC = false;
      }

      //-------------------  insert model  -------------------//
      int order[6]    = { 0, 1, 2, 3, 4, 5 };
      int modelNum    = 6;
      int model[6][4] = {
        { 0, 1, 2 },   // 0:  LT, RT, LB
        { 0, 1, 3 },   // 1:  LT, RT, RB
        { 0, 2, 3 },   // 2:  LT, LB, RB
        { 1, 2, 3 },   // 3:  RT, LB, RB
        { 0, 1 },   // 4:  LT, RT
        { 0, 2 },   // 5:  LT, LB
      };

      int verNum[6] = { 3, 3, 3, 3, 2, 2 };
      int startIdx  = cu.cs->sps->m_AffineType ? 0 : 4;
      for (int idx = startIdx; idx < modelNum; idx++)
      {
        int modelIdx = order[idx];
        if (getAffineControlPointCand(cu, mi, isAvailable, model[modelIdx],
                                      ((modelIdx == 3) ? neighBcw[1] : neighBcw[0]), modelIdx, verNum[modelIdx],
                                      affMrgCtx))
        {
          if (affMrgCtx.numValidMergeCand == mrgCandIdx)
          {
            return;
          }
          affMrgCtx.numValidMergeCand++;
          if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
          {
            return;
          }
        }
        if (idx == startIdx)
        {
          if (addSpatialAffineMergeHMVPCand(cu, affMrgCtx, cu.cs->motionLut.lutAff, 0, npuGroup2, posGroup2, numGroup2,
                                            mrgCandIdx))
          {
            return;
          }

          if (addOneInheritedHMVPAffineMergeCand(cu, affMrgCtx, cu.cs->motionLut.lutAffInherit, 0))
          {
            if (affMrgCtx.numValidMergeCand == mrgCandIdx)   // for decoder
            {
              return;
            }

            affMrgCtx.numValidMergeCand++;

            // early termination
            if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
            {
              return;
            }
          }
        }
        if (!((cu.slice->m_poc - cu.slice->getRefPOC(RPL0, 0)) == 1 && cu.slice->m_picHeader->m_mvdL1ZeroFlag))
        {
          if (idx == startIdx)
          {
            getNonAdjCstMergeCand(cu, affMrgCtx, mrgCandIdx, true);
            if (affMrgCtx.numValidMergeCand != 0 && affMrgCtx.numValidMergeCand - 1 == mrgCandIdx)
            {
              return;
            }

            if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
            {
              return;
            }
          }
        }
      }
      if ((cu.slice->m_poc - cu.slice->getRefPOC(RPL0, 0)) == 1 && cu.slice->m_picHeader->m_mvdL1ZeroFlag)
      {
        getNonAdjCstMergeCand(cu, affMrgCtx, mrgCandIdx, true);
        if (affMrgCtx.numValidMergeCand != 0 && affMrgCtx.numValidMergeCand - 1 == mrgCandIdx)
        {
          return;
        }

        if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
        {
          return;
        }
      }
    }
    ///> End: Constructed affine candidates

    if (isTmvpAvailable)
    {
      Position posRB = cu.Y().bottomRight().offset(3, 3);
      if (addOneAffineMergeHMVPCand(cu, affMrgCtx, cu.cs->motionLut.lutAff, 0, tmvpInfo, posRB, BCW_DEFAULT))
      {
        if (affMrgCtx.numValidMergeCand == mrgCandIdx)   // for decoder
        {
          return;
        }

        affMrgCtx.numValidMergeCand++;

        // early termination
        if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
        {
          return;
        }
      }
    }

    for (int iAffListIdx = 1; iAffListIdx < MAX_NUM_AFF_HMVP_CANDS; iAffListIdx++)
    {
      if (addSpatialAffineMergeHMVPCand(cu, affMrgCtx, cu.cs->motionLut.lutAff, iAffListIdx, npuGroup2, posGroup2,
                                        numGroup2, mrgCandIdx))
      {
        return;
      }
      if (isTmvpAvailable)
      {
        Position posRB = cu.Y().bottomRight().offset(3, 3);
        if (addOneAffineMergeHMVPCand(cu, affMrgCtx, cu.cs->motionLut.lutAff, iAffListIdx, tmvpInfo, posRB,
                                      BCW_DEFAULT))
        {
          if (affMrgCtx.numValidMergeCand == mrgCandIdx)   // for decoder
          {
            return;
          }

          affMrgCtx.numValidMergeCand++;

          // early termination
          if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
          {
            return;
          }
        }
      }
    }

    for (int iAffListIdx = 1; iAffListIdx < MAX_NUM_AFF_INHERIT_HMVP_CANDS; iAffListIdx++)
    {
      if (addOneInheritedHMVPAffineMergeCand(cu, affMrgCtx, cu.cs->motionLut.lutAffInherit, iAffListIdx))
      {
        if (affMrgCtx.numValidMergeCand == mrgCandIdx)   // for decoder
        {
          return;
        }

        affMrgCtx.numValidMergeCand++;

        // early termination
        if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
        {
          return;
        }
      }
    }
    const int MAX_PAIRWISE_NUM                     = 9;
    const int preDefinedPairs[MAX_PAIRWISE_NUM][2] = { { 0, 1 }, { 0, 2 }, { 1, 2 }, { 0, 3 }, { 1, 3 },
                                                       { 2, 3 }, { 0, 4 }, { 1, 4 }, { 2, 4 } };
    int       iATMVPoffset                         = affMrgCtx.mergeType[0] == MergeType::SUBPU_ATMVP ? 1 : 0;
    int       currSize                             = affMrgCtx.numValidMergeCand;
    for (int pairIdx = 0; pairIdx < MAX_PAIRWISE_NUM; pairIdx++)
    {
      int idx0 = preDefinedPairs[pairIdx][0] + iATMVPoffset;
      int idx1 = preDefinedPairs[pairIdx][1] + iATMVPoffset;

      CHECK(affMrgCtx.mergeType[idx0] == MergeType::SUBPU_ATMVP || affMrgCtx.mergeType[idx1] == MergeType::SUBPU_ATMVP,
            "Invalid Index");

      if (idx0 >= currSize || idx1 >= currSize)
      {
        break;
      }
      if ((affMrgCtx.interDirNeighbours[idx0] & affMrgCtx.interDirNeighbours[idx1]) == 0)
      {
        continue;
      }

      affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = 0;

      affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][0][RPL0].setMvField(Mv(), -1);
      affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][1][RPL0].setMvField(Mv(), -1);
      affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][2][RPL0].setMvField(Mv(), -1);
      affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][0][RPL1].setMvField(Mv(), -1);
      affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][1][RPL1].setMvField(Mv(), -1);
      affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][2][RPL1].setMvField(Mv(), -1);

      affMrgCtx.bcwIdx[affMrgCtx.numValidMergeCand] = BCW_DEFAULT;

      affMrgCtx.LICFlags[affMrgCtx.numValidMergeCand] = false;

      affMrgCtx.affineType[affMrgCtx.numValidMergeCand] = AffineModel::_6_PARAMS;

      int numCPMVs = 3;

      if (affMrgCtx.mvFieldNeighbours[idx0][0][RPL0].refIdx != -1 &&
          affMrgCtx.mvFieldNeighbours[idx1][0][RPL0].refIdx == affMrgCtx.mvFieldNeighbours[idx0][0][RPL0].refIdx)
      {
        affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] |= 1;

        for (int mvIdx = 0; mvIdx < numCPMVs; mvIdx++)
        {
          Mv avgMv = affMrgCtx.mvFieldNeighbours[idx0][mvIdx][RPL0].mv;
          avgMv += affMrgCtx.mvFieldNeighbours[idx1][mvIdx][RPL0].mv;
          roundAffineMv(avgMv.hor, avgMv.ver, 1);
          affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][mvIdx][RPL0].setMvField(
            avgMv, affMrgCtx.mvFieldNeighbours[idx0][0][RPL0].refIdx);
        }
      }
      if (affMrgCtx.mvFieldNeighbours[idx0][0][RPL1].refIdx != -1 &&
          affMrgCtx.mvFieldNeighbours[idx1][0][RPL1].refIdx == affMrgCtx.mvFieldNeighbours[idx0][0][RPL1].refIdx)
      {
        affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] |= 2;

        for (int mvIdx = 0; mvIdx < numCPMVs; mvIdx++)
        {
          Mv avgMv = affMrgCtx.mvFieldNeighbours[idx0][mvIdx][RPL1].mv;
          avgMv += affMrgCtx.mvFieldNeighbours[idx1][mvIdx][RPL1].mv;
          roundAffineMv(avgMv.hor, avgMv.ver, 1);
          affMrgCtx.mvFieldNeighbours[affMrgCtx.numValidMergeCand][mvIdx][RPL1].setMvField(
            avgMv, affMrgCtx.mvFieldNeighbours[idx0][0][RPL1].refIdx);
        }
      }

      if (affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] == 0)
      {
        continue;
      }
      if (checkLastAffineMergeCandRedundancy(cu, affMrgCtx))
      {
        if (affMrgCtx.numValidMergeCand == mrgCandIdx)   // for decoder
        {
          return;
        }

        affMrgCtx.numValidMergeCand++;

        // early termination
        if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
        {
          return;
        }
      }
    }
  }

  ///> zero padding
  int cnt = affMrgCtx.numValidMergeCand;
  while (cnt < maxNumAffineMergeCand)
  {
    for (int mvNum = 0; mvNum < 3; mvNum++)
    {
      affMrgCtx.mvFieldNeighbours[cnt][mvNum][0].setMvField(Mv(0, 0), 0);
    }
    affMrgCtx.interDirNeighbours[cnt] = 1;
    CHECK(affMrgCtx.LICFlags[cnt], "LIC flag is already set");

    if (slice.isInterB())
    {
      for (int mvNum = 0; mvNum < 3; mvNum++)
      {
        affMrgCtx.mvFieldNeighbours[cnt][mvNum][1].setMvField(Mv(0, 0), 0);
      }
      affMrgCtx.interDirNeighbours[cnt] = 3;
    }
    affMrgCtx.affineType[cnt] = AffineModel::_4_PARAMS;

    if (cnt == mrgCandIdx)
    {
      return;
    }
    cnt++;
    affMrgCtx.numValidMergeCand++;
  }
}

uint8_t PU::getMergeIdxFromAffMmvdBaseIdx(const AffineMergeCtx &affMrgCtx, uint16_t affMmvdBaseIdx)
{
  uint8_t mergeIdx = (uint8_t)affMmvdBaseIdx;
  for (int i = 0; i < affMrgCtx.numValidMergeCand; i++)
  {
    if (affMrgCtx.mergeType[i] != MergeType::DEFAULT_N)
    {
      ++mergeIdx;
    }
    else
    {
      break;
    }
  }

  return mergeIdx;
}

void PU::getAffMmvdMvf(const CodingUnit &pu, const AffineMergeCtx &affineMergeCtx,
                       std::array<MvField[2], AFFINE_MAX_NUM_CP> &mvfMmvd, const uint16_t affMmvdBase,
                       const uint16_t offsetStep, const uint16_t offsetDir)
{
  CHECK(!pu.affine, "Affine flag is not on for Affine MMVD mode!");
  CHECK(affineMergeCtx.mergeType[affMmvdBase] != MergeType::DEFAULT_N,
        "AFF_MMVD base candidate type is not regular Affine!");

  static const int32_t refMvdCands[AF_MMVD_STEP_NUM] = { 1, 2, 4, 8, 16 };
  const int32_t        iPicSize                      = pu.slice->m_pic->lumaSize().area();
  const int32_t        mvShift                       = iPicSize < 921600
                                 ? 0
                                 : (iPicSize < 4096000 ? 2 : MV_FRACTIONAL_BITS_INTERNAL - 1);   // 921600 = 1280x720, 4096000 = 2560x1600
  int                  step                          = refMvdCands[offsetStep] << mvShift;
  AffineModel          affineType                    = affineMergeCtx.affineType[affMmvdBase];

  uint8_t interDir = affineMergeCtx.interDirNeighbours[affMmvdBase];
  int8_t  refIdxL0 = affineMergeCtx.mvFieldNeighbours[affMmvdBase][0][RPL0].refIdx;
  int8_t  refIdxL1 = affineMergeCtx.mvFieldNeighbours[affMmvdBase][0][RPL1].refIdx;

  Mv baseMv[3][2];
  for (int i = 0; i < 3; i++)
  {
    baseMv[i][RPL0]         = affineMergeCtx.mvFieldNeighbours[affMmvdBase][i][RPL0].mv;
    baseMv[i][RPL1]         = affineMergeCtx.mvFieldNeighbours[affMmvdBase][i][RPL1].mv;
    mvfMmvd[i][RPL0].refIdx = refIdxL0;
    mvfMmvd[i][RPL1].refIdx = refIdxL1;
    mvfMmvd[i][RPL0].mv     = Mv();
    mvfMmvd[i][RPL1].mv     = Mv();
  }

  int magY    = (offsetDir >> 1) & 0x1;
  int sign    = (offsetDir & 0x1) ? -1 : 1;
  int offsetX = (1 - magY) * sign * step;
  int offsetY = (magY)*sign * step;
  Mv  offsetMv(offsetX, offsetY);

  int numCp = (affineType == AffineModel::_4_PARAMS) ? 2 : 3;
  for (int cpIdx = 0; cpIdx < numCp; cpIdx++)
  {
    if (interDir == 1)
    {
      mvfMmvd[cpIdx][RPL0].mv = baseMv[cpIdx][RPL0] + offsetMv;
    }
    else if (interDir == 2)
    {
      mvfMmvd[cpIdx][RPL1].mv = baseMv[cpIdx][RPL1] + offsetMv;
    }
    else if (interDir == 3)
    {
      int pocCur = pu.slice->m_poc;
      int pocL0  = pu.slice->getRefPOC(RPL0, refIdxL0);
      int pocL1  = pu.slice->getRefPOC(RPL1, refIdxL1);

      int distL0              = pocL0 - pocCur;
      int distL1              = pocL1 - pocCur;
      mvfMmvd[cpIdx][RPL0].mv = baseMv[cpIdx][RPL0] + offsetMv;
      mvfMmvd[cpIdx][RPL1].mv = distL0 * distL1 < 0 ? baseMv[cpIdx][RPL1] - offsetMv : baseMv[cpIdx][RPL1] + offsetMv;
    }
  }
}

void PU::setAllAffineMvField(CodingUnit &cu, std::array<MvField[2], AFFINE_MAX_NUM_CP> &mvField, RefPicList eRefList)
{
  // Set Mv
  std::array<Mv, AFFINE_MAX_NUM_CP> mv;

  for (int i = 0; i < mv.size(); i++)
  {
    mv[i] = mvField[i][eRefList].mv;
  }
  setAllAffineMv(cu, mv[0], mv[1], mv[2], eRefList);

  // Set RefIdx
  CHECK(mvField[0][eRefList].refIdx != mvField[1][eRefList].refIdx ||
          mvField[0][eRefList].refIdx != mvField[2][eRefList].refIdx,
        "Affine mv corners don't have the same refIdx.");
  cu.refIdx[eRefList] = mvField[0][eRefList].refIdx;
}

void PU::setAllAffineMv(CodingUnit &cu, Mv affLT, Mv affRT, Mv affLB, RefPicList eRefList, bool clipCPMVs)
{
  int width = cu.lwidth();
  int shift = MAX_CU_DEPTH;
  if (clipCPMVs)
  {
    affLT.foldToStorageBitDepth();
    affRT.foldToStorageBitDepth();
    if (cu.affineType == AffineModel::_6_PARAMS)
    {
      affLB.foldToStorageBitDepth();
    }
  }
  int deltaMvHorX, deltaMvHorY, deltaMvVerX, deltaMvVerY;
  deltaMvHorX = (affRT - affLT).getHor() * (1 << (shift - floorLog2(width)));
  deltaMvHorY = (affRT - affLT).getVer() * (1 << (shift - floorLog2(width)));
  int height  = cu.lheight();
  if (cu.affineType == AffineModel::_6_PARAMS)
  {
    deltaMvVerX = (affLB - affLT).getHor() * (1 << (shift - floorLog2(height)));
    deltaMvVerY = (affLB - affLT).getVer() * (1 << (shift - floorLog2(height)));
  }
  else
  {
    deltaMvVerX = -deltaMvHorY;
    deltaMvVerY = deltaMvHorX;
  }

  const int mvScaleHor = affLT.getHor() * (1 << shift);
  const int mvScaleVer = affLT.getVer() * (1 << shift);

  int       blockWidth  = AFFINE_SUBBLOCK_SIZE;
  int       blockHeight = AFFINE_SUBBLOCK_SIZE;
  const int halfBW      = blockWidth >> 1;
  const int halfBH      = blockHeight >> 1;

  MotionBuf mb = cu.getMotionBuf();
  int       mvScaleTmpHor, mvScaleTmpVer;

  for (int h = 0; h < cu.lheight(); h += blockHeight)
  {
    for (int w = 0; w < cu.lwidth(); w += blockWidth)
    {
      mvScaleTmpHor = mvScaleHor + deltaMvHorX * (halfBW + w) + deltaMvVerX * (halfBH + h);
      mvScaleTmpVer = mvScaleVer + deltaMvHorY * (halfBW + w) + deltaMvVerY * (halfBH + h);

      Mv curMv(mvScaleTmpHor, mvScaleTmpVer);
      curMv >>= shift;
      curMv.clipToStorageBitDepth();

      for (int y = (h >> MIN_CU_LOG2); y < ((h + blockHeight) >> MIN_CU_LOG2); y++)
      {
        for (int x = (w >> MIN_CU_LOG2); x < ((w + blockWidth) >> MIN_CU_LOG2); x++)
        {
          mb.at(x, y).mv[eRefList] = curMv;
        }
      }
    }
  }

  cu.mvAffi[eRefList][0] = affLT;
  cu.mvAffi[eRefList][1] = affRT;
  cu.mvAffi[eRefList][2] = affLB;
}

void clipColPos(int &posX, int &posY, const CodingUnit &cu)
{
  Position      puPos       = cu.lumaPos();
  int           log2CtuSize = floorLog2(cu.cs->sps->m_ctuSize);
  int           ctuX        = ((puPos.x >> log2CtuSize) << log2CtuSize);
  int           ctuY        = ((puPos.y >> log2CtuSize) << log2CtuSize);
  int           horMax;
  const SubPic &curSubPic = cu.slice->m_pps->getSubPicFromPos(puPos);
  if (curSubPic.m_treatedAsPicFlag)
  {
    horMax = std::min((int)curSubPic.m_subPicRight, ctuX + (int)cu.cs->sps->m_ctuSize + 3);
  }
  else
  {
    horMax = std::min((int)cu.cs->pps->m_picWidthInLumaSamples - 1, ctuX + (int)cu.cs->sps->m_ctuSize + 3);
  }
  int horMin = std::max((int)0, ctuX);
  int verMax = std::min((int)cu.cs->pps->m_picHeightInLumaSamples - 1, ctuY + (int)cu.cs->sps->m_ctuSize - 1);
  int verMin = std::max((int)0, ctuY);

  posX = std::min(horMax, std::max(horMin, posX));
  posY = std::min(verMax, std::max(verMin, posY));
}

bool PU::getInterMergeSubPuMvpCand(const CodingUnit &cu, AffineMergeCtx &mrgCtx, const int count)
{
  const Slice   &slice = *cu.cs->slice;
  const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
  const unsigned mask  = ~(scale - 1);

  const Picture *pColPic =
    slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.m_colFromL0Flag : 0), slice.m_colRefIdx);
  if (pColPic->isRefScaled(cu.cs->pps))
  {
    return false;
  }
  Mv cTMv;

  if (count)
  {
    if ((mrgCtx.interDirNeighbours[0] & (1 << RPL0)) &&
        slice.getRefPic(RPL0, mrgCtx.mvFieldNeighbours[0][RPL0][0].refIdx) == pColPic)
    {
      cTMv = mrgCtx.mvFieldNeighbours[0][RPL0][0].mv;
    }
    else if (slice.isInterB() && (mrgCtx.interDirNeighbours[0] & (1 << RPL1)) &&
             slice.getRefPic(RPL1, mrgCtx.mvFieldNeighbours[0][RPL1][0].refIdx) == pColPic)
    {
      cTMv = mrgCtx.mvFieldNeighbours[0][RPL1][0].mv;
    }
  }

  ///////////////////////////////////////////////////////////////////////
  ////////          GET Initial Temporal Vector                  ////////
  ///////////////////////////////////////////////////////////////////////

  Mv cTempVector = cTMv;

  // compute the location of the current PU
  Position puPos       = cu.lumaPos();
  Size     puSize      = cu.lumaSize();
  int      numPartLine = std::max(puSize.width >> ATMVP_SUB_BLOCK_SIZE, 1u);
  int      numPartCol  = std::max(puSize.height >> ATMVP_SUB_BLOCK_SIZE, 1u);
  int      puHeight    = numPartCol == 1 ? puSize.height : 1 << ATMVP_SUB_BLOCK_SIZE;
  int      puWidth     = numPartLine == 1 ? puSize.width : 1 << ATMVP_SUB_BLOCK_SIZE;

  Mv         cColMv;
  int        refIdx   = 0;
  // use coldir.
  const bool isBSlice = slice.isInterB();

  Position centerPos;

  bool found  = false;
  cTempVector = cTMv;

  cTempVector.changePrecision(MvPrecision::SIXTEENTH, MvPrecision::ONE);
  int tempX = cTempVector.getHor();
  int tempY = cTempVector.getVer();

  centerPos.x = puPos.x + (puSize.width >> 1) + tempX;
  centerPos.y = puPos.y + (puSize.height >> 1) + tempY;

  clipColPos(centerPos.x, centerPos.y, cu);

  centerPos = Position { PosType(centerPos.x & mask), PosType(centerPos.y & mask) };

  // derivation of center motion parameters from the collocated CU
  const MotionInfo &mi = pColPic->m_cs->getMotionInfo(centerPos);

  if (mi.isInter && mi.isIBCmot == false)
  {
    mrgCtx.interDirNeighbours[0] = 0;

    for (unsigned currRefListId = 0; currRefListId < (isBSlice ? 2 : 1); currRefListId++)
    {
      RefPicList currRefPicList = RefPicList(currRefListId);

      if (getColocatedMVP(cu, currRefPicList, centerPos, cColMv, refIdx, true))
      {
        // set as default, for further motion vector field spanning
        mrgCtx.mvFieldNeighbours[0][currRefListId][0].setMvField(cColMv, 0);
        mrgCtx.interDirNeighbours[0] |= (1 << currRefListId);
        mrgCtx.bcwIdx[0]   = BCW_DEFAULT;
        mrgCtx.LICFlags[0] = false;
        found              = true;
      }
      else
      {
        mrgCtx.mvFieldNeighbours[0][currRefListId][0].setMvField(Mv(), NOT_VALID);
        mrgCtx.interDirNeighbours[0] &= ~(1 << currRefListId);
      }
    }
  }

  if (!found)
  {
    return false;
  }

  int xOff = (puWidth >> 1) + tempX;
  int yOff = (puHeight >> 1) + tempY;

  MotionBuf &mb = mrgCtx.subPuMvpMiBuf;

  const bool isBiPred = isBipredRestriction(cu);

  for (int y = puPos.y; y < puPos.y + puSize.height; y += puHeight)
  {
    for (int x = puPos.x; x < puPos.x + puSize.width; x += puWidth)
    {
      Position colPos { x + xOff, y + yOff };

      clipColPos(colPos.x, colPos.y, cu);

      colPos = Position { PosType(colPos.x & mask), PosType(colPos.y & mask) };

      const MotionInfo &colMi = pColPic->m_cs->getMotionInfo(colPos);

      MotionInfo mi;

      found       = false;
      mi.isInter  = true;
      mi.sliceIdx = slice.m_independentSliceIdx;
      mi.isIBCmot = false;
      if (colMi.isInter && colMi.isIBCmot == false)
      {
        for (unsigned currRefListId = 0; currRefListId < (isBSlice ? 2 : 1); currRefListId++)
        {
          RefPicList currRefPicList = RefPicList(currRefListId);
          if (getColocatedMVP(cu, currRefPicList, colPos, cColMv, refIdx, true))
          {
            mi.refIdx[currRefListId] = 0;
            mi.mv[currRefListId]     = cColMv;
            found                    = true;
          }
        }
      }
      if (!found)
      {
        mi.mv[0]     = mrgCtx.mvFieldNeighbours[0][0][0].mv;
        mi.mv[1]     = mrgCtx.mvFieldNeighbours[0][1][0].mv;
        mi.refIdx[0] = mrgCtx.mvFieldNeighbours[0][0][0].refIdx;
        mi.refIdx[1] = mrgCtx.mvFieldNeighbours[0][1][0].refIdx;
      }

      mi.interDir = (mi.refIdx[0] != -1 ? 1 : 0) + (mi.refIdx[1] != -1 ? 2 : 0);

      if (isBiPred && mi.interDir == 3)
      {
        mi.interDir  = 1;
        mi.mv[1]     = Mv();
        mi.refIdx[1] = NOT_VALID;
      }

      CHECK(mi.usesLIC, "mi.usesLIC is set");

      mb.subBuf(g_miScaling.scale(Position { x, y } - cu.lumaPos()), g_miScaling.scale(Size(puWidth, puHeight)))
        .fill(mi);
    }
  }

  return true;
}

void PU::spanBdmvrMotionInfo(const CodingUnit &cu, MotionBuf &mb, Mv *bdmvrSubPuMv0, Mv *bdmvrSubPuMv1,
                             Mv *bdofSubPuMvOffset)
{
  MotionInfo mi;

  CHECK(CU::isIBC(cu), "BDMVR and IBC not possible!");
  CHECK(cu.affine, "BDMVR and Affine not possible!");

  mi.isInter  = !CU::isIntra(cu);
  mi.isIBCmot = CU::isIBC(cu);
  mi.sliceIdx = cu.slice->m_independentSliceIdx;

  if (mi.isInter)
  {
    mi.interDir     = cu.interDir;
    mi.useAltHpelIf = cu.imv == IMV_HPEL;

    for (int i = 0; i < NUM_RPL01; i++)
    {
      mi.mv[i]     = cu.mv[i];
      mi.refIdx[i] = cu.refIdx[i];
    }
  }

  const int dx                    = std::min<int>(cu.lwidth(), BDOF_SUBPU_DIM);
  const int dy                    = std::min<int>(cu.lheight(), BDOF_SUBPU_DIM);
  int       subPuIdx              = 0;
  const int bioSubPuIdxStrideIncr = BDOF_SUBPU_STRIDE - std::max(1, (int)(cu.lwidth() >> BDOF_SUBPU_DIM_LOG2));

  if (bdofSubPuMvOffset)
  {
    for (int yStart = 0; yStart < cu.lheight(); yStart += dy)
    {
      for (int xStart = 0; xStart < cu.lwidth(); xStart += dx)
      {
        const int bdmvrSubPuIdx =
          (yStart >> DMVR_SUBCU_HEIGHT_LOG2) * DMVR_SUBPU_STRIDE + (xStart >> DMVR_SUBCU_WIDTH_LOG2);
        mi.mv[0] = bdmvrSubPuMv0[bdmvrSubPuIdx] + bdofSubPuMvOffset[subPuIdx];
        mi.mv[1] = bdmvrSubPuMv1[bdmvrSubPuIdx] - bdofSubPuMvOffset[subPuIdx];

        subPuIdx++;

        mb.subBuf(g_miScaling.scale(Position(xStart, yStart)), g_miScaling.scale(Size(dx, dy))).fill(mi);
      }
      subPuIdx += bioSubPuIdxStrideIncr;
    }
  }
  else
  {
    for (int yStart = 0; yStart < cu.lheight(); yStart += dy)
    {
      for (int xStart = 0; xStart < cu.lwidth(); xStart += dx)
      {
        const int bdmvrSubPuIdx =
          (yStart >> DMVR_SUBCU_HEIGHT_LOG2) * DMVR_SUBPU_STRIDE + (xStart >> DMVR_SUBCU_WIDTH_LOG2);
        mi.mv[0] = bdmvrSubPuMv0[bdmvrSubPuIdx];
        mi.mv[1] = bdmvrSubPuMv1[bdmvrSubPuIdx];
        mb.subBuf(g_miScaling.scale(Position(xStart, yStart)), g_miScaling.scale(Size(dx, dy))).fill(mi);
      }
    }
  }
}

void PU::spanMotionInfo(CodingUnit &cu, const AffineMergeCtx *affMrgCtx)
{
  MotionBuf mb = cu.getMotionBuf();

  if (cu.mergeFlag && cu.mergeType == MergeType::SUBPU_ATMVP)
  {
    CHECK(affMrgCtx->subPuMvpMiBuf.area() == 0 || !affMrgCtx->subPuMvpMiBuf.buf, "Buffer not initialized");
    mb.copyFrom(affMrgCtx->subPuMvpMiBuf);
  }
  else if (cu.mergeFlag && cu.bdmvrRefine)
  {
    THROW("Span motion info should not be called for BDMVR refined CUs");
  }
  else if (!cu.mergeFlag || cu.mergeType == MergeType::DEFAULT_N || cu.mergeType == MergeType::IBC)
  {
    MotionInfo mi;

    mi.isInter  = !CU::isIntra(cu);
    mi.isIBCmot = CU::isIBC(cu);
    mi.sliceIdx = cu.slice->m_independentSliceIdx;
    mi.usesLIC  = cu.licFlag;

    if (mi.isInter)
    {
      mi.interDir     = cu.interDir;
      mi.useAltHpelIf = cu.imv == IMV_HPEL;

      for (int i = 0; i < NUM_RPL01; i++)
      {
        mi.mv[i]     = cu.mv[i];
        mi.refIdx[i] = cu.refIdx[i];
      }
      if (mi.isIBCmot)
      {
        mi.bv = cu.bv;
      }
    }

    if (cu.affine)
    {
      for (int y = 0; y < mb.height; y++)
      {
        for (int x = 0; x < mb.width; x++)
        {
          MotionInfo &dest = mb.at(x, y);
          dest.isInter     = mi.isInter;
          dest.isIBCmot    = false;
          dest.interDir    = mi.interDir;
          dest.sliceIdx    = mi.sliceIdx;
          for (int i = 0; i < NUM_RPL01; i++)
          {
            if (mi.refIdx[i] == -1)
            {
              dest.mv[i] = Mv();
            }
            dest.refIdx[i] = mi.refIdx[i];
          }
          dest.usesLIC = mi.usesLIC;
        }
      }
    }
    else
    {
      mb.fill(mi);
    }
  }
  else
  {
    if (isBipredRestriction(cu))
    {
      for (int y = 0; y < mb.height; y++)
      {
        for (int x = 0; x < mb.width; x++)
        {
          MotionInfo &mi = mb.at(x, y);
          if (mi.interDir == 3)
          {
            mi.interDir  = 1;
            mi.mv[1]     = Mv();
            mi.refIdx[1] = NOT_VALID;
          }
        }
      }
    }
  }
}

void PU::applyImv(CodingUnit &cu, InterPrediction *interPred)
{
  CHECK(cu.mergeFlag, "Unexpected")

  if (cu.interDir != 2 /* PRED_L1 */)
  {
    cu.mvd[0].changeTransPrecAmvr2Internal(cu.imv);
    unsigned mvpIdx = cu.mvpIdx[0];
    AMVPInfo amvpInfo;
    if (CU::isIBC(cu))
    {
      PU::fillIBCMvpCand(cu, amvpInfo);
    }
    else
    {
      PU::fillMvpCand(cu, RPL0, cu.refIdx[0], amvpInfo);
    }
    cu.mvpNum[0] = amvpInfo.numCand;
    cu.mvpIdx[0] = mvpIdx;
    cu.mv[0]     = amvpInfo.mvCand[mvpIdx] + cu.mvd[0];
    cu.mv[0].foldToStorageBitDepth();
  }

  if (cu.interDir != 1 /* PRED_L0 */)
  {
    if (!(cu.cs->picHeader->m_mvdL1ZeroFlag && cu.interDir == 3) && cu.imv) /* PRED_BI */
    {
      cu.mvd[1].changeTransPrecAmvr2Internal(cu.imv);
    }
    unsigned mvpIdx = cu.mvpIdx[1];
    AMVPInfo amvpInfo;
    PU::fillMvpCand(cu, RPL1, cu.refIdx[1], amvpInfo);
    cu.mvpNum[1] = amvpInfo.numCand;
    cu.mvpIdx[1] = mvpIdx;
    cu.mv[1]     = amvpInfo.mvCand[mvpIdx] + cu.mvd[1];
    cu.mv[1].foldToStorageBitDepth();
  }

  PU::spanMotionInfo(cu);
}

bool PU::isSimpleSymmetricBiPred(const CodingUnit &cu)
{
  const int refIdx0 = cu.refIdx[RPL0];
  const int refIdx1 = cu.refIdx[RPL1];

  if (refIdx0 >= 0 && refIdx1 >= 0)
  {
    const Slice *slice = cu.slice;

    if (slice->getRefPic(RPL0, refIdx0)->m_longTerm || slice->getRefPic(RPL1, refIdx1)->m_longTerm)
    {
      return false;
    }

    if (cu.bcwIdx != BCW_DEFAULT)
    {
      return false;
    }

    if (WPScalingParam::isWeighted(slice->getWpScaling(RPL0, refIdx0)) ||
        WPScalingParam::isWeighted(slice->getWpScaling(RPL1, refIdx1)))
    {
      return false;
    }

    const int poc0 = slice->getRefPOC(RPL0, refIdx0);
    const int poc1 = slice->getRefPOC(RPL1, refIdx1);
    const int poc  = slice->m_poc;

    return poc - poc0 == poc1 - poc;
  }

  return false;
}

void PU::restrictBiPredMergeCandsOne(CodingUnit &cu)
{
  if (PU::isBipredRestriction(cu))
  {
    if (cu.interDir == 3)
    {
      cu.interDir  = 1;
      cu.refIdx[1] = -1;
      cu.mv[1]     = Mv(0, 0);
      cu.bcwIdx    = BCW_DEFAULT;
    }
  }
}

void PU::setGpmDirMode(CodingUnit &cu)
{
  cu.gpmDirMode = 1;

  bool triggerUni = ((cu.lwidth() * cu.lheight()) < 256) ? true : false;

  if (triggerUni)
  {
    cu.gpmDirMode = 0;
  }
}

void PU::getGeoMergeCandidates(const CodingUnit &cu, MergeCtx &geoMrgCtx)
{
  MergeCtx        tempMergeCtx;
  const MergeCtx *mergeCtx;

  const uint32_t maxNumMergeCand = cu.cs->sps->m_maxNumMergeCand;
  geoMrgCtx.numValidMergeCand    = 0;

  for (int32_t i = 0; i < cu.cs->sps->m_maxNumGeoCand; i++)
  {
    geoMrgCtx.bcwIdx[i]                      = BCW_DEFAULT;
    geoMrgCtx.interDirNeighbours[i]          = 0;
    geoMrgCtx.mvFieldNeighbours[i][0].refIdx = NOT_VALID;
    geoMrgCtx.mvFieldNeighbours[i][1].refIdx = NOT_VALID;
    geoMrgCtx.mvFieldNeighbours[i][0].mv     = Mv();
    geoMrgCtx.mvFieldNeighbours[i][1].mv     = Mv();
    geoMrgCtx.useAltHpelIf[i]                = false;
    geoMrgCtx.LICFlags[i]                    = false;
  }

  PU::getInterMergeCandidates(cu, tempMergeCtx, 0, -1, getBiGpmThreshold(cu));
  mergeCtx = &tempMergeCtx;

  if (cu.gpmDirMode)
  {
    CHECK(!cu.geoFlag, "is4GPM should be on when gpmDirMode is not 0");

    geoMrgCtx                   = tempMergeCtx;
    geoMrgCtx.numValidMergeCand = maxNumMergeCand;

    for (int32_t i = 0; i < maxNumMergeCand; i++)
    {
      geoMrgCtx.useAltHpelIf[i] = false;
      geoMrgCtx.bcwIdx[i]       = BCW_DEFAULT;
      geoMrgCtx.LICFlags[i]     = false;
    }
    return;
  }

  const int mvdSimilarityThresh = PU::getBDMVRMvdThreshold(cu);

  for (int32_t i = 0; i < maxNumMergeCand; i++)
  {
    int parity = i & 1;
    if (mergeCtx->interDirNeighbours[i] & (0x01 + parity))
    {
      geoMrgCtx.interDirNeighbours[geoMrgCtx.numValidMergeCand]            = 1 + parity;
      geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand][!parity].mv = Mv(0, 0);
      geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand][parity].mv  = mergeCtx->mvFieldNeighbours[i][parity].mv;
      geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand][!parity].refIdx = -1;
      geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand][parity].refIdx =
        mergeCtx->mvFieldNeighbours[i][parity].refIdx;
      if (geoMrgCtx.checkSimilarMotion(geoMrgCtx.numValidMergeCand, mvdSimilarityThresh))
      {
        continue;
      }
      geoMrgCtx.numValidMergeCand++;
      if (geoMrgCtx.numValidMergeCand == GEO_MAX_NUM_UNI_CANDS)
      {
        return;
      }
      continue;
    }

    if (mergeCtx->interDirNeighbours[i] & (0x02 - parity))
    {
      geoMrgCtx.interDirNeighbours[geoMrgCtx.numValidMergeCand]            = 2 - parity;
      geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand][!parity].mv = mergeCtx->mvFieldNeighbours[i][!parity].mv;
      geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand][parity].mv  = Mv(0, 0);
      geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand][!parity].refIdx =
        mergeCtx->mvFieldNeighbours[i][!parity].refIdx;
      geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand][parity].refIdx = -1;
      if (geoMrgCtx.checkSimilarMotion(geoMrgCtx.numValidMergeCand, mvdSimilarityThresh))
      {
        continue;
      }
      geoMrgCtx.numValidMergeCand++;
      if (geoMrgCtx.numValidMergeCand == GEO_MAX_NUM_UNI_CANDS)
      {
        return;
      }
    }
  }

  // add more parity based geo candidates, in an opposite parity rule
  if (geoMrgCtx.numValidMergeCand < cu.cs->sps->m_maxNumGeoCand)
  {
    for (int32_t i = 0; i < maxNumMergeCand; i++)
    {
      int parity = i & 1;
      if (mergeCtx->interDirNeighbours[i] == 3)
      {
        geoMrgCtx.interDirNeighbours[geoMrgCtx.numValidMergeCand] = 2 - parity;
        geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand][!parity].mv =
          mergeCtx->mvFieldNeighbours[i][!parity].mv;
        geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand][parity].mv = Mv(0, 0);
        geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand][!parity].refIdx =
          mergeCtx->mvFieldNeighbours[i][!parity].refIdx;
        geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand][parity].refIdx = -1;
        if (geoMrgCtx.checkSimilarMotion(geoMrgCtx.numValidMergeCand, mvdSimilarityThresh))
        {
          continue;
        }
        geoMrgCtx.numValidMergeCand++;
        if (geoMrgCtx.numValidMergeCand == cu.cs->sps->m_maxNumGeoCand)
        {
          return;
        }
      }
    }
  }

  // add at most two average based geo candidates
  if (geoMrgCtx.numValidMergeCand < cu.cs->sps->m_maxNumGeoCand)
  {
    // add one L0 cand by averaging the first two available L0 candidates
    int cnt              = 0;
    int firstAvailRefIdx = -1;
    Mv  avgMv;
    avgMv.setZero();
    for (int i = 0; i < geoMrgCtx.numValidMergeCand; i++)
    {
      if (cnt == 2)
      {
        break;
      }
      if (geoMrgCtx.interDirNeighbours[i] == 1)
      {
        avgMv += geoMrgCtx.mvFieldNeighbours[i][0].mv;
        if (firstAvailRefIdx == -1)
        {
          firstAvailRefIdx = geoMrgCtx.mvFieldNeighbours[i][0].refIdx;
        }
        cnt++;
      }
    }
    if (cnt == 2)
    {
      roundAffineMv(avgMv.hor, avgMv.ver, 1);
      geoMrgCtx.interDirNeighbours[geoMrgCtx.numValidMergeCand] = 1;
      geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand][0].setMvField(avgMv, firstAvailRefIdx);
      geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand][1].setMvField(Mv(0, 0), NOT_VALID);

      if (!geoMrgCtx.checkSimilarMotion(geoMrgCtx.numValidMergeCand, mvdSimilarityThresh))
      {
        geoMrgCtx.numValidMergeCand++;
      }
      if (geoMrgCtx.numValidMergeCand == cu.cs->sps->m_maxNumGeoCand)
      {
        return;
      }
    }

    // add one L1 cand by averaging the first two available L1 candidates
    cnt              = 0;
    firstAvailRefIdx = -1;
    avgMv.setZero();
    for (int i = 0; i < geoMrgCtx.numValidMergeCand; i++)
    {
      if (cnt == 2)
      {
        break;
      }
      if (geoMrgCtx.interDirNeighbours[i] == 2)
      {
        avgMv += geoMrgCtx.mvFieldNeighbours[i][1].mv;
        if (firstAvailRefIdx == -1)
        {
          firstAvailRefIdx = geoMrgCtx.mvFieldNeighbours[i][1].refIdx;
        }
        cnt++;
      }
    }
    if (cnt == 2)
    {
      roundAffineMv(avgMv.hor, avgMv.ver, 1);
      geoMrgCtx.interDirNeighbours[geoMrgCtx.numValidMergeCand] = 2;
      geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand][1].setMvField(avgMv, firstAvailRefIdx);
      geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand][0].setMvField(Mv(0, 0), NOT_VALID);
      if (!geoMrgCtx.checkSimilarMotion(geoMrgCtx.numValidMergeCand, mvdSimilarityThresh))
      {
        geoMrgCtx.numValidMergeCand++;
      }
      if (geoMrgCtx.numValidMergeCand == cu.cs->sps->m_maxNumGeoCand)
      {
        return;
      }
    }
  }
  if (geoMrgCtx.numValidMergeCand < cu.cs->sps->m_maxNumGeoCand)
  {
    const Slice &slice = *cu.cs->slice;
    int          iNumRefIdx =
      slice.isInterP() ? slice.m_numRefIdx[RPL0] : std::min(slice.m_numRefIdx[RPL0], slice.m_numRefIdx[RPL1]);
    int r      = 0;
    int refcnt = 0;

    for (int32_t i = geoMrgCtx.numValidMergeCand; i < cu.cs->sps->m_maxNumGeoCand; i++)
    {
      int parity = slice.isInterP() ? 0 : (i & 1);
      if (0x01 + parity)
      {
        geoMrgCtx.interDirNeighbours[geoMrgCtx.numValidMergeCand]                = 1 + parity;
        geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand][!parity].mv     = Mv(0, 0);
        geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand][parity].mv      = Mv(0, 0);
        geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand][!parity].refIdx = -1;
        geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand][parity].refIdx  = r;

        if (refcnt == iNumRefIdx - 1)
        {
          r = 0;
        }
        else
        {
          ++r;
          ++refcnt;
        }

        geoMrgCtx.numValidMergeCand++;
        if (geoMrgCtx.numValidMergeCand == cu.cs->sps->m_maxNumGeoCand)
        {
          return;
        }
      }
    }
  }
}

void PU::spanGeoMMVDMotionInfo(CodingUnit &cu, const GeoMergeCtx &geoMrgCtx)
{
  const MergeCtx *mrgCtx[2];
  mrgCtx[0] = mrgCtx[1]      = &geoMrgCtx[0];
  const int  mvShift         = MV_FRACTIONAL_BITS_DIFF;
  const bool extMMVD         = cu.cs->picHeader->m_gpmMMVDTableFlag;
  const int  mmvdCands[8]    = { 1 << mvShift,  2 << mvShift,  4 << mvShift,  8 << mvShift,
                                 16 << mvShift, 32 << mvShift, 64 << mvShift, 128 << mvShift };
  const int  extMmvdCands[9] = { 1 << mvShift,  2 << mvShift,  4 << mvShift,  8 << mvShift, 12 << mvShift,
                                 16 << mvShift, 24 << mvShift, 32 << mvShift, 64 << mvShift };
  const int *refMvdCands     = (extMMVD ? extMmvdCands : mmvdCands);
  Mv         mvOffset[2][2], deltaMv;
  int        interDirIdx[2];

  for (auto part: { 0, 1 })
  {
    interDirIdx[part] = mrgCtx[part]->interDirNeighbours[cu.geoMergeIdx[part]];
    if (cu.geoMMVDFlag[part])
    {
      if (mrgCtx[part]->interDirNeighbours[cu.geoMergeIdx[part]] == 3)
      {
        if (!cu.slice->m_checkLdc)
        {
          interDirIdx[part] = (cu.geoMergeIdx[part] % 2) + 1;
        }
      }
    }

    if (cu.geoMMVDFlag[part])
    {
      int fPosStep     = (extMMVD ? (cu.geoMMVDIdx[part] >> 3) : (cu.geoMMVDIdx[part] >> 2));
      int fPosPosition = (extMMVD ? (cu.geoMMVDIdx[part] - (fPosStep << 3)) : (cu.geoMMVDIdx[part] - (fPosStep << 2)));

      if (fPosPosition == 0)
      {
        deltaMv = Mv(refMvdCands[fPosStep], 0);
      }
      else if (fPosPosition == 1)
      {
        deltaMv = Mv(-refMvdCands[fPosStep], 0);
      }
      else if (fPosPosition == 2)
      {
        deltaMv = Mv(0, refMvdCands[fPosStep]);
      }
      else if (fPosPosition == 3)
      {
        deltaMv = Mv(0, -refMvdCands[fPosStep]);
      }
      else if (fPosPosition == 4)
      {
        deltaMv = Mv(refMvdCands[fPosStep], refMvdCands[fPosStep]);
      }
      else if (fPosPosition == 5)
      {
        deltaMv = Mv(refMvdCands[fPosStep], -refMvdCands[fPosStep]);
      }
      else if (fPosPosition == 6)
      {
        deltaMv = Mv(-refMvdCands[fPosStep], refMvdCands[fPosStep]);
      }
      else if (fPosPosition == 7)
      {
        deltaMv = Mv(-refMvdCands[fPosStep], -refMvdCands[fPosStep]);
      }

      if (interDirIdx[part] == 1)
      {
        mvOffset[part][0] = deltaMv;
      }
      else if (interDirIdx[part] == 2)
      {
        mvOffset[part][1] = deltaMv;
      }
      else
      {
        CHECK(mrgCtx[part]->interDirNeighbours[cu.geoMergeIdx[part]] == 2, "Error in inter dir when setting MMVD GPM");
        const int refListIdx0 = mrgCtx[part]->mvFieldNeighbours[cu.geoMergeIdx[part]][RPL0].refIdx;
        const int refListIdx1 = mrgCtx[part]->mvFieldNeighbours[cu.geoMergeIdx[part]][RPL1].refIdx;

        const int poc0    = cu.slice->getRefPOC(RPL0, refListIdx0);
        const int poc1    = cu.slice->getRefPOC(RPL1, refListIdx1);
        const int currPoc = cu.slice->m_poc;

        mvOffset[part][0] = deltaMv;

        if ((poc0 - currPoc) == (poc1 - currPoc))
        {
          mvOffset[part][1] = mvOffset[part][0];
        }
        else if (abs(poc1 - currPoc) > abs(poc0 - currPoc))
        {
          const int scale   = PU::getDistScaleFactor(currPoc, poc0, currPoc, poc1);
          mvOffset[part][1] = mvOffset[part][0];

          const bool isL0RefLongTerm = cu.slice->getRefPic(RPL0, refListIdx0)->m_longTerm;
          const bool isL1RefLongTerm = cu.slice->getRefPic(RPL1, refListIdx1)->m_longTerm;

          if (isL0RefLongTerm || isL1RefLongTerm)
          {
            if ((poc1 - currPoc) * (poc0 - currPoc) > 0)
            {
              mvOffset[part][0] = mvOffset[part][1];
            }
            else
            {
              mvOffset[part][0].set(-1 * mvOffset[part][1].getHor(), -1 * mvOffset[part][1].getVer());
            }
          }
          else
          {
            mvOffset[part][0] = mvOffset[part][1].getScaledMv(scale);
          }
        }
        else
        {
          const int  scale           = PU::getDistScaleFactor(currPoc, poc1, currPoc, poc0);
          const bool isL0RefLongTerm = cu.slice->getRefPic(RPL0, refListIdx0)->m_longTerm;
          const bool isL1RefLongTerm = cu.slice->getRefPic(RPL1, refListIdx1)->m_longTerm;
          if (isL0RefLongTerm || isL1RefLongTerm)
          {
            if ((poc1 - currPoc) * (poc0 - currPoc) > 0)
            {
              mvOffset[part][1] = mvOffset[part][0];
            }
            else
            {
              mvOffset[part][1].set(-1 * mvOffset[part][0].getHor(), -1 * mvOffset[part][0].getVer());
            }
          }
          else
          {
            mvOffset[part][1] = mvOffset[part][0].getScaledMv(scale);
          }
        }
      }
    }
  }

  MotionBuf mb = cu.getMotionBuf();

  uint32_t sliceIdx = cu.cs->slice->m_independentSliceIdx;
  bool     isIntra0 = cu.geoMergeIdx[0] >= GEO_MAX_NUM_UNI_CANDS;
  bool     isIntra1 = cu.geoMergeIdx[1] >= GEO_MAX_NUM_UNI_CANDS;
  int16_t  angle    = g_geoParams[cu.geoSplitDir].angleIdx;
  int      tpmMask  = 0;
  int      lookUpY = 0, motionIdx = 0;
  bool     isFlip      = angle >= 13 && angle <= 27;
  int      distanceIdx = g_geoParams[cu.geoSplitDir].distanceIdx;
  int      distanceX   = angle;
  int      distanceY   = (distanceX + (GEO_NUM_ANGLES >> 2)) % GEO_NUM_ANGLES;
  int      offsetX     = (-(int)cu.lwidth()) >> 1;
  int      offsetY     = (-(int)cu.lheight()) >> 1;
  if (distanceIdx > 0)
  {
    if (angle % 16 == 8 || (angle % 16 != 0 && cu.lheight() >= cu.lwidth()))
    {
      offsetY += angle < 16 ? ((distanceIdx * cu.lheight()) >> 3) : -(int)((distanceIdx * cu.lheight()) >> 3);
    }
    else
    {
      offsetX += angle < 16 ? ((distanceIdx * cu.lwidth()) >> 3) : -(int)((distanceIdx * cu.lwidth()) >> 3);
    }
  }
  for (int y = 0; y < mb.height; y++)
  {
    lookUpY = (((4 * y + offsetY) << 1) + 5) * g_dis[distanceY];
    for (int x = 0; x < mb.width; x++)
    {
      MotionInfo &mi = mb.at(x, y);
      mi.sliceIdx    = sliceIdx;
      mi.bcwIdx      = BCW_DEFAULT;
      mi.usesLIC     = false;

      motionIdx = (((4 * x + offsetX) << 1) + 5) * g_dis[distanceX] + lookUpY;
      tpmMask   = (motionIdx <= 0 ? (1 - isFlip) : isFlip);
      if (tpmMask == 0)
      {
        if (isIntra0)
        {
          mi.isInter   = false;
          mi.interDir  = MAX_UCHAR;
          mi.refIdx[0] = -1;
          mi.refIdx[1] = -1;
          mi.mv[0]     = Mv(0, 0);
          mi.mv[1]     = Mv(0, 0);
        }
        else
        {
          mi.isInter   = true;
          mi.interDir  = interDirIdx[0];
          mi.refIdx[0] = (interDirIdx[0] == 1 || interDirIdx[0] == 3)
            ? mrgCtx[0]->mvFieldNeighbours[cu.geoMergeIdx[0]][RPL0].refIdx
            : -1;
          mi.refIdx[1] = (interDirIdx[0] == 2 || interDirIdx[0] == 3)
            ? mrgCtx[0]->mvFieldNeighbours[cu.geoMergeIdx[0]][RPL1].refIdx
            : -1;
          mi.mv[0]     = mrgCtx[0]->mvFieldNeighbours[cu.geoMergeIdx[0]][RPL0].mv + mvOffset[0][0];
          mi.mv[1]     = mrgCtx[0]->mvFieldNeighbours[cu.geoMergeIdx[0]][RPL1].mv + mvOffset[0][1];
          if (interDirIdx[0] == 1)
          {
            mi.mv[1] = Mv();
          }
          else if (interDirIdx[0] == 2)
          {
            mi.mv[0] = Mv();
          }
        }
      }
      else
      {
        if (isIntra1)
        {
          mi.isInter   = false;
          mi.interDir  = MAX_UCHAR;
          mi.refIdx[0] = -1;
          mi.refIdx[1] = -1;
          mi.mv[0]     = Mv(0, 0);
          mi.mv[1]     = Mv(0, 0);
        }
        else
        {
          mi.isInter   = true;
          mi.interDir  = interDirIdx[1];
          mi.refIdx[0] = (interDirIdx[1] == 1 || interDirIdx[1] == 3)
            ? mrgCtx[1]->mvFieldNeighbours[cu.geoMergeIdx[1]][RPL0].refIdx
            : -1;
          mi.refIdx[1] = (interDirIdx[1] == 2 || interDirIdx[1] == 3)
            ? mrgCtx[1]->mvFieldNeighbours[cu.geoMergeIdx[1]][RPL1].refIdx
            : -1;
          mi.mv[0]     = mrgCtx[1]->mvFieldNeighbours[cu.geoMergeIdx[1]][RPL0].mv + mvOffset[1][0];
          mi.mv[1]     = mrgCtx[1]->mvFieldNeighbours[cu.geoMergeIdx[1]][RPL1].mv + mvOffset[1][1];
          if (interDirIdx[1] == 1)
          {
            mi.mv[1] = Mv();
          }
          else if (interDirIdx[1] == 2)
          {
            mi.mv[0] = Mv();
          }
        }
      }
    }
  }
}

void PU::getNeighborAffineInfo(const CodingUnit &cu, int &numNeighborAvai, int &numNeighborAffine)
{
  const Position   &posLT       = cu.Y().topLeft();
  const Position   &posRT       = cu.Y().topRight();
  const Position   &posLB       = cu.Y().bottomLeft();
  const int         neighborNum = 5;
  const CodingUnit *neighbor[neighborNum];
  neighbor[0]       = cu.cs->getCURestricted(posRT.offset(0, -1), cu, cu.chType);   // above
  neighbor[1]       = cu.cs->getCURestricted(posLB.offset(-1, 0), cu, cu.chType);   // left
  neighbor[2]       = cu.cs->getCURestricted(posRT.offset(1, -1), cu, cu.chType);   // above-right
  neighbor[3]       = cu.cs->getCURestricted(posLB.offset(-1, 1), cu, cu.chType);   // left-bottom
  neighbor[4]       = cu.cs->getCURestricted(posLT.offset(-1, -1), cu, cu.chType);   // above-left
  numNeighborAvai   = 0;
  numNeighborAffine = 0;
  for (int i = 0; i < neighborNum; i++)
  {
    if (neighbor[i] != nullptr)
    {
      numNeighborAvai++;
      numNeighborAffine += neighbor[i]->affine;
    }
  }
}

int PU::getSameMotionNeighbor(const CodingUnit &cu, int dir, int startBlock, int maxBlocks, MotionInfo &curMi,
                              MotionInfo &neighMi, const CodingUnit *&neighCuOut, bool &isValid)
{
  Position blockPos;
  Position neighPos;

  switch (dir)
  {
  case 0:   // top border
    blockPos = cu.lumaPos().offset(startBlock * cu.cs->pcv->minCUWidth, 0);
    neighPos = blockPos.offset(0, -1);
    break;
  case 1:   // left border
    blockPos = cu.lumaPos().offset(0, startBlock * cu.cs->pcv->minCUHeight);
    neighPos = blockPos.offset(-1, 0);
    break;
  default:
    THROW("unknown direction");
  }

  const CodingUnit *neighCu = cu.cs->getCU(neighPos, cu.chType);
  neighCuOut                = neighCu;
  if (!neighCu)
  {
    isValid = false;
    return maxBlocks;
  }

  const bool isNeighIntra = CU::isIntra(*neighCu);
  curMi                   = cu.getMotionInfo(blockPos);
  neighMi                 = neighCu->getMotionInfo(neighPos);
  isValid =
    !isNeighIntra && !neighMi.isIBCmot && neighMi.isInter && !CU::isIntra(cu) && !curMi.isIBCmot && curMi.isInter;

  int l;
  // loop until motion info deviates for border sub-blocks (inside CU and left/top adjacent CU's)
  for (l = 1; l < maxBlocks; l++)
  {
    switch (dir)
    {
    case 0:
      blockPos.x += cu.cs->pcv->minCUWidth;
      neighPos.x += cu.cs->pcv->minCUWidth;
      break;   // top border
    case 1:
      blockPos.y += cu.cs->pcv->minCUHeight;
      neighPos.y += cu.cs->pcv->minCUHeight;
      break;   // left border
    default:
      THROW("unknown direction");
    }

    if (!curMi.isSameMiObmc(cu.getMotionInfo(blockPos)))
    {
      break;
    }
    const CodingUnit *nextCu = cu.cs->getCU(neighPos, cu.chType);
    CHECK(nextCu == nullptr, "miss adjacent CU");
    if (isNeighIntra != CU::isIntra(*nextCu) || !neighMi.isSameMiObmc(nextCu->getMotionInfo(neighPos)))
    {
      break;
    }
    if (!nextCu->hasSameLicParameters(*neighCu))
    {
      break;
    }
  }

  return l;
}

bool PU::isBMMergeFlagCoded(const CodingUnit &pu)
{
  if (pu.cs->slice->m_sps->m_useDMVD && !pu.cs->slice->m_checkLdc)
  {
    return (pu.cs->sps->m_maxNumBMMergeCand > 0);
  }
  return false;
}

bool PU::addBMMergeHMVPCand(const CodingStructure &cs, MergeCtx &mrgCtx, const int &mrgCandIdx,
                            const uint32_t maxNumMergeCandMin1, int &cnt, const bool isAvailableA1,
                            const MotionInfo &miLeft, const bool isAvailableB1, const MotionInfo &miAbove,
                            const bool ibcFlag, const uint32_t mvdSimilarityThresh)
{
  const Slice &slice = *cs.slice;
  MotionInfo   miNeighbor;

  auto &lut               = ibcFlag ? cs.motionLut.lutIbc : cs.motionLut.lut;
  int   numAvaiLCandInLut = (int)lut.size();

  for (int mrgIdx = 1; mrgIdx <= numAvaiLCandInLut; mrgIdx++)
  {
    miNeighbor = lut[numAvaiLCandInLut - mrgIdx];

    if (mrgIdx > 2 || (mrgIdx > 1 && ibcFlag) ||
        ((!isAvailableA1 || (miLeft != miNeighbor)) && (!isAvailableB1 || (miAbove != miNeighbor))))
    {
      int refIdx0 = miNeighbor.refIdx[0];
      int refIdx1 = miNeighbor.refIdx[1];

      if (miNeighbor.bcwIdx != BCW_DEFAULT || !isBiPredFromDifferentDirEqDistPoc(slice, refIdx0, refIdx1))
      {
        continue;
      }

      mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
      mrgCtx.useAltHpelIf[cnt]       = !ibcFlag && miNeighbor.useAltHpelIf;
      mrgCtx.mvFieldNeighbours[cnt][0].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
      mrgCtx.mvFieldNeighbours[cnt][1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);

      if (mrgCtx.checkSimilarMotion(cnt, mvdSimilarityThresh))
      {
        continue;
      }
      if (mrgCandIdx == cnt)
      {
        return true;
      }
      cnt++;

      if (cnt == maxNumMergeCandMin1)
      {
        break;
      }
    }
  }

  if (cnt < maxNumMergeCandMin1)
  {
    mrgCtx.useAltHpelIf[cnt] = false;
  }

  return false;
}

void PU::getInterBMCandidates(const CodingUnit &cu, MergeCtx &mrgCtx, const int &mrgCandIdx)
{
  const unsigned         plevel          = cu.cs->sps->m_log2ParallelMergeLevelMinus2 + 2;
  const CodingStructure &cs              = *cu.cs;
  const Slice           &slice           = *cu.cs->slice;
  const uint32_t         maxNumMergeCand = cu.cs->sps->m_maxNumBMMergeCand;

  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    mrgCtx.bcwIdx[ui]                      = BCW_DEFAULT;
    mrgCtx.interDirNeighbours[ui]          = 0;
    mrgCtx.mvFieldNeighbours[ui][0].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[ui][1].refIdx = NOT_VALID;
    mrgCtx.useAltHpelIf[ui]                = false;
    mrgCtx.LICFlags[ui]                    = false;
  }

  mrgCtx.numValidMergeCand = maxNumMergeCand;
  mrgCtx.numCandToTestEnc  = maxNumMergeCand;
  // compute the location of the current PU

  int mvThreshod = 1;

  int cnt = 0;

  const Position posLT = cu.Y().topLeft();
  const Position posRT = cu.Y().topRight();
  const Position posLB = cu.Y().bottomLeft();
  MotionInfo     miAbove, miLeft, miAboveLeft, miAboveRight, miBelowLeft;

  // above
  const CodingUnit *puAbove = cs.getCURestricted(posRT.offset(0, -1), cu, cu.chType);

  bool isAvailableB1 = puAbove && isDiffMER(cu.lumaPos(), posRT.offset(0, -1), plevel) && CU::isInter(*puAbove);

  if (isAvailableB1)
  {
    miAbove = puAbove->getMotionInfo(posRT.offset(0, -1));

    if (isBiPredFromDifferentDirEqDistPoc(slice, miAbove.refIdx[0], miAbove.refIdx[1]))
    {
      mrgCtx.interDirNeighbours[cnt] = miAbove.interDir;
      mrgCtx.useAltHpelIf[cnt]       = miAbove.useAltHpelIf;
      mrgCtx.mvFieldNeighbours[cnt][0].setMvField(miAbove.mv[0], miAbove.refIdx[0]);
      mrgCtx.mvFieldNeighbours[cnt][1].setMvField(miAbove.mv[1], miAbove.refIdx[1]);

      if (!mrgCtx.checkSimilarMotion(cnt, mvThreshod))
      {
        if (mrgCandIdx == cnt)
        {
          mrgCtx.numValidMergeCand = cnt + 1;
          return;
        }
        cnt++;
      }
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    mrgCtx.numValidMergeCand = cnt;
    return;
  }

  // left
  const CodingUnit *puLeft = cs.getCURestricted(posLB.offset(-1, 0), cu, cu.chType);

  const bool isAvailableA1 = puLeft && isDiffMER(cu.lumaPos(), posLB.offset(-1, 0), plevel) && CU::isInter(*puLeft);

  if (isAvailableA1)
  {
    miLeft = puLeft->getMotionInfo(posLB.offset(-1, 0));

    if (!isAvailableB1 || (miAbove != miLeft))
    {
      if (isBiPredFromDifferentDirEqDistPoc(slice, miLeft.refIdx[0], miLeft.refIdx[1]))
      {
        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miLeft.interDir;
        mrgCtx.useAltHpelIf[cnt]       = miLeft.useAltHpelIf;
        // get Mv from Left
        mrgCtx.mvFieldNeighbours[cnt][0].setMvField(miLeft.mv[0], miLeft.refIdx[0]);
        mrgCtx.mvFieldNeighbours[cnt][1].setMvField(miLeft.mv[1], miLeft.refIdx[1]);

        if (!mrgCtx.checkSimilarMotion(cnt, mvThreshod))
        {
          if (mrgCandIdx == cnt)
          {
            mrgCtx.numValidMergeCand = cnt + 1;
            return;
          }

          cnt++;
        }
      }
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    mrgCtx.numValidMergeCand = cnt;
    return;
  }

  // above right
  const CodingUnit *puAboveRight = cs.getCURestricted(posRT.offset(1, -1), cu, cu.chType);

  bool isAvailableB0 =
    puAboveRight && isDiffMER(cu.lumaPos(), posRT.offset(1, -1), plevel) && CU::isInter(*puAboveRight);

  if (isAvailableB0)
  {
    miAboveRight = puAboveRight->getMotionInfo(posRT.offset(1, -1));

    if (!isAvailableB1 || (miAbove != miAboveRight))
    {
      if (isBiPredFromDifferentDirEqDistPoc(slice, miAboveRight.refIdx[0], miAboveRight.refIdx[1]))
      {
        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miAboveRight.interDir;
        mrgCtx.useAltHpelIf[cnt]       = miAboveRight.useAltHpelIf;
        mrgCtx.mvFieldNeighbours[cnt][0].setMvField(miAboveRight.mv[0], miAboveRight.refIdx[0]);
        mrgCtx.mvFieldNeighbours[cnt][1].setMvField(miAboveRight.mv[1], miAboveRight.refIdx[1]);

        if (!mrgCtx.checkSimilarMotion(cnt, mvThreshod))
        {
          if (mrgCandIdx == cnt)
          {
            mrgCtx.numValidMergeCand = cnt + 1;
            return;
          }
          cnt++;
        }
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    mrgCtx.numValidMergeCand = cnt;
    return;
  }

  // left bottom
  const CodingUnit *puLeftBottom = cs.getCURestricted(posLB.offset(-1, 1), cu, cu.chType);

  bool isAvailableA0 =
    puLeftBottom && isDiffMER(cu.lumaPos(), posLB.offset(-1, 1), plevel) && CU::isInter(*puLeftBottom);

  if (isAvailableA0)
  {
    miBelowLeft = puLeftBottom->getMotionInfo(posLB.offset(-1, 1));

    if (!isAvailableA1 || (miBelowLeft != miLeft))
    {
      if (isBiPredFromDifferentDirEqDistPoc(slice, miBelowLeft.refIdx[0], miBelowLeft.refIdx[1]))
      {
        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miBelowLeft.interDir;
        mrgCtx.useAltHpelIf[cnt]       = miBelowLeft.useAltHpelIf;
        // get Mv from Bottom-Left
        mrgCtx.mvFieldNeighbours[cnt][0].setMvField(miBelowLeft.mv[0], miBelowLeft.refIdx[0]);
        mrgCtx.mvFieldNeighbours[cnt][1].setMvField(miBelowLeft.mv[1], miBelowLeft.refIdx[1]);

        if (!mrgCtx.checkSimilarMotion(cnt, mvThreshod))
        {
          if (mrgCandIdx == cnt)
          {
            return;
          }
          cnt++;
        }
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    mrgCtx.numValidMergeCand = cnt;
    return;
  }

  const CodingUnit *puAboveLeft = cs.getCURestricted(posLT.offset(-1, -1), cu, cu.chType);

  bool isAvailableB2 =
    puAboveLeft && isDiffMER(cu.lumaPos(), posLT.offset(-1, -1), plevel) && CU::isInter(*puAboveLeft);

  if (isAvailableB2)
  {
    miAboveLeft = puAboveLeft->getMotionInfo(posLT.offset(-1, -1));

    if ((!isAvailableA1 || (miLeft != miAboveLeft)) && (!isAvailableB1 || (miAbove != miAboveLeft)))
    {
      if (isBiPredFromDifferentDirEqDistPoc(slice, miAboveLeft.refIdx[0], miAboveLeft.refIdx[1]))
      {
        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miAboveLeft.interDir;
        mrgCtx.useAltHpelIf[cnt]       = miAboveLeft.useAltHpelIf;
        // get Mv from Above-Left
        mrgCtx.mvFieldNeighbours[cnt][0].setMvField(miAboveLeft.mv[0], miAboveLeft.refIdx[0]);
        mrgCtx.mvFieldNeighbours[cnt][1].setMvField(miAboveLeft.mv[1], miAboveLeft.refIdx[1]);

        if (!mrgCtx.checkSimilarMotion(cnt, mvThreshod))
        {
          if (mrgCandIdx == cnt)
          {
            mrgCtx.numValidMergeCand = cnt + 1;
            return;
          }

          cnt++;
        }
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    mrgCtx.numValidMergeCand = cnt;
    return;
  }

  if (slice.m_picHeader->m_enableTMVPFlag)
  {
    //>> MTK colocated-RightBottom
    // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to
    Position             posRB = cu.Y().bottomRight().offset(-3, -3);
    const PreCalcValues &pcv   = *cs.pcv;

    Position posC0;
    Position posC1    = cu.Y().center();
    bool     C0Avail  = false;
    bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
    const SubPic &curSubPic = cu.cs->slice->m_pps->getSubPicFromPos(cu.lumaPos());
    if (curSubPic.m_treatedAsPicFlag)
    {
      boundaryCond = (posRB.x + pcv.minCUWidth) <= curSubPic.m_subPicRight &&
        (posRB.y + pcv.minCUHeight) <= curSubPic.m_subPicBottom;
    }
    if (boundaryCond)
    {
      int posYInCtu = posRB.y & pcv.maxCUHeightMask;
      if (posYInCtu + 4 < pcv.maxCUHeight)
      {
        posC0   = posRB.offset(4, 4);
        C0Avail = true;
      }
    }

    Mv       cColMv;
    int      iRefIdx     = 0;
    int      dir         = 0;
    unsigned uiArrayAddr = cnt;
    bool     bExistMV    = (C0Avail && getColocatedMVP(cu, RPL0, posC0, cColMv, iRefIdx, false)) ||
      getColocatedMVP(cu, RPL0, posC1, cColMv, iRefIdx, false);
    if (bExistMV)
    {
      dir |= 1;
      mrgCtx.mvFieldNeighbours[uiArrayAddr][0].setMvField(cColMv, iRefIdx);
    }

    if (slice.isInterB())
    {
      bExistMV = (C0Avail && getColocatedMVP(cu, RPL1, posC0, cColMv, iRefIdx, false)) ||
        getColocatedMVP(cu, RPL1, posC1, cColMv, iRefIdx, false);
      if (bExistMV)
      {
        dir |= 2;
        mrgCtx.mvFieldNeighbours[uiArrayAddr][1].setMvField(cColMv, iRefIdx);
      }
    }

    if (dir != 0)
    {
      bool addTMvp = isBiPredFromDifferentDirEqDistPoc(slice, mrgCtx.mvFieldNeighbours[uiArrayAddr][0].refIdx,
                                                       mrgCtx.mvFieldNeighbours[uiArrayAddr][1].refIdx);
      if (addTMvp)
      {
        mrgCtx.interDirNeighbours[uiArrayAddr] = dir;
        mrgCtx.useAltHpelIf[uiArrayAddr]       = false;

        if (!mrgCtx.checkSimilarMotion(cnt, mvThreshod))
        {
          if (mrgCandIdx == cnt)
          {
            mrgCtx.numValidMergeCand = cnt + 1;
            return;
          }

          cnt++;
        }
      }
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    mrgCtx.numValidMergeCand = cnt;
    return;
  }

  int        maxNumMergeCandMin1 = maxNumMergeCand - 1;
  MotionInfo miNeighbor;
  int        offsetX             = 0;
  int        offsetY             = 0;
  const int  iNACANDIDATE_NUM[4] = { 3, 5, 5, 5 };
  const int  idxMap[4][5]        = { { 0, 1, 4 }, { 0, 1, 2, 3, 4 }, { 0, 1, 2, 3, 4 }, { 0, 1, 2, 3, 4 } };

  for (int iDistanceIndex = 0; iDistanceIndex < NADISTANCE_LEVEL && cnt < maxNumMergeCandMin1; iDistanceIndex++)
  {
    const int iNADistanceHor = cu.lwidth() * (iDistanceIndex + 1);
    const int iNADistanceVer = cu.lheight() * (iDistanceIndex + 1);

    for (int NASPIdx = 0; NASPIdx < iNACANDIDATE_NUM[iDistanceIndex] && cnt < maxNumMergeCandMin1; NASPIdx++)
    {
      switch (idxMap[iDistanceIndex][NASPIdx])
      {
      case 0:
        offsetX = -iNADistanceHor - 1;
        offsetY = cu.lheight() + iNADistanceVer - 1;
        break;
      case 1:
        offsetX = cu.lwidth() + iNADistanceHor - 1;
        offsetY = -iNADistanceVer - 1;
        break;
      case 2:
        offsetX = cu.lwidth() >> 1;
        offsetY = -iNADistanceVer - 1;
        break;
      case 3:
        offsetX = -iNADistanceHor - 1;
        offsetY = cu.lheight() >> 1;
        break;
      case 4:
        offsetX = -iNADistanceHor - 1;
        offsetY = -iNADistanceVer - 1;
        break;
      default:
        THROW("error!");
        break;
      }

      const CodingUnit *puNonAdjacent = cs.getCURestricted(posLT.offset(offsetX, offsetY), cu, cu.chType);

      bool isAvailableNonAdjacent =
        puNonAdjacent && isDiffMER(cu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent);

      if (isAvailableNonAdjacent)
      {
        miNeighbor = puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY));

        if (isBiPredFromDifferentDirEqDistPoc(slice, miNeighbor.refIdx[0], miNeighbor.refIdx[1]))
        {
          // get Inter Dir
          mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
          mrgCtx.useAltHpelIf[cnt]       = miNeighbor.useAltHpelIf;
          // get Mv from Above-Left
          mrgCtx.mvFieldNeighbours[cnt][0].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
          mrgCtx.mvFieldNeighbours[cnt][1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);

          if (!mrgCtx.checkSimilarMotion(cnt, mvThreshod))
          {
            if (mrgCandIdx == cnt)
            {
              mrgCtx.numValidMergeCand = cnt + 1;
              return;
            }
            cnt++;
          }
        }
      }
    }
  }

  if (cnt != maxNumMergeCandMin1)
  {
    bool bFound = addBMMergeHMVPCand(cs, mrgCtx, mrgCandIdx, maxNumMergeCandMin1, cnt, isAvailableA1, miLeft,
                                     isAvailableB1, miAbove, CU::isIBC(cu), mvThreshod);

    if (bFound)
    {
      return;
    }
  }

  {
    if (cnt > 1 && cnt < maxNumMergeCand)
    {
      mrgCtx.mvFieldNeighbours[cnt][0].setMvField(Mv(0, 0), NOT_VALID);
      mrgCtx.mvFieldNeighbours[cnt][1].setMvField(Mv(0, 0), NOT_VALID);
      mrgCtx.bcwIdx[cnt]       = BCW_DEFAULT;
      // calculate average MV for L0 and L1 seperately
      uint8_t interDir         = 0;
      mrgCtx.useAltHpelIf[cnt] = (mrgCtx.useAltHpelIf[0] == mrgCtx.useAltHpelIf[1]) ? mrgCtx.useAltHpelIf[0] : false;
      for (int refListId = 0; refListId < (slice.isInterB() ? 2 : 1); refListId++)
      {
        const short refIdxI = mrgCtx.mvFieldNeighbours[0][refListId].refIdx;
        const short refIdxJ = mrgCtx.mvFieldNeighbours[1][refListId].refIdx;

        // both MVs are invalid, skip
        if (refIdxI != refIdxJ)
        {
          continue;
        }

        interDir += 1 << refListId;
        const Mv &mvI = mrgCtx.mvFieldNeighbours[0][refListId].mv;
        const Mv &mvJ = mrgCtx.mvFieldNeighbours[1][refListId].mv;

        // average two MVs
        Mv avgMv = mvI;
        avgMv += mvJ;
        roundAffineMv(avgMv.hor, avgMv.ver, 1);

        mrgCtx.mvFieldNeighbours[cnt][refListId].setMvField(avgMv, refIdxI);
      }

      mrgCtx.interDirNeighbours[cnt] = interDir;
      if (interDir == 3 &&
          isBiPredFromDifferentDirEqDistPoc(slice, mrgCtx.mvFieldNeighbours[cnt][0].refIdx,
                                            mrgCtx.mvFieldNeighbours[cnt][1].refIdx))
      {
        if (!mrgCtx.checkSimilarMotion(cnt, mvThreshod))
        {
          cnt++;
        }
      }
    }

    // early termination
    if (cnt == maxNumMergeCand)
    {
      mrgCtx.numValidMergeCand = cnt;
      return;
    }
  }

  mrgCtx.numCandToTestEnc = cnt;

  if (cnt < maxNumMergeCand)
  {
    mrgCtx.interDirNeighbours[cnt] = 3;
    mrgCtx.bcwIdx[cnt]             = BCW_DEFAULT;
    mrgCtx.useAltHpelIf[cnt]       = false;
    mrgCtx.mvFieldNeighbours[cnt][0].setMvField(Mv(0, 0), 0);
    mrgCtx.mvFieldNeighbours[cnt][1].setMvField(Mv(0, 0), 0);

    if (!mrgCtx.checkSimilarMotion(cnt, mvThreshod))
    {
      if (mrgCandIdx == cnt)
      {
        mrgCtx.numValidMergeCand = cnt + 1;
        return;
      }
      cnt++;
    }
  }

  mrgCtx.numValidMergeCand = cnt;
}

bool PU::checkRprLicCondition(const CodingUnit &cu)
{
  CHECK(cu.interDir != (((cu.refIdx[1] >= 0) << 1) | (cu.refIdx[0] >= 0)), "interDir/refIdx inconsistency");

  if (!cu.cs->sps->m_licEnabledFlag)
  {
    return false;
  }

  if (cu.interDir != 1 && cu.interDir != 2 && (!cu.cs->sps->m_biLicEnabledFlag || cu.interDir != 3))
  {
    return false;
  }

  const bool isResamplingPossible = cu.cs->sps->m_rprEnabledFlag;
  for (const auto eRefPicList: { RPL0, RPL1 })
  {
    if ((cu.interDir & (1 << eRefPicList)) && isResamplingPossible &&
        cu.slice->getRefPic(eRefPicList, cu.refIdx[eRefPicList])->isRefScaled(cu.cs->pps))
    {
      return false;
    }
  }

  return true;
}

void computeDeltaAndShift(const Position posLT, Mv firstMv, std::vector<RMVFInfo> &mvpInfoVecOri)
{
  g_pelBufOP.computeDeltaAndShift(posLT, firstMv, mvpInfoVecOri);
}
void computeDeltaAndShiftAddi(const Position posLT, Mv firstMv, std::vector<RMVFInfo> &mvpInfoVecOri,
                              std::vector<RMVFInfo> &mvpInfoVecRes)
{
  g_pelBufOP.computeDeltaAndShiftAddi(posLT, firstMv, mvpInfoVecOri, mvpInfoVecRes);
}
void buildRegressionMatrix(std::vector<RMVFInfo> &mvpInfoVecOri, int64_t sumbb[2][3][3], int64_t sumeb[2][3],
                           uint16_t addedSize)
{
  g_pelBufOP.buildRegressionMatrix(mvpInfoVecOri, sumbb, sumeb, addedSize);
}
static int getRMVFMSB(int64_t x)
{
  int     msb = 0, bits = (sizeof(int64_t) << 3);
  int64_t y = 1;
  while (x > 1)
  {
    bits >>= 1;
    y = x >> bits;
    if (y)
    {
      x = y;
      msb += bits;
    }
  }
  msb += (int)y;
  return msb;
}

int64_t divideRMVF(int64_t numer, int64_t denom, int scale, bool isoffset)   // out = numer/denom
{
  if (numer == 0 || denom == 1)
  {
    return numer;
  }
  int64_t       d;
  const int64_t iShiftA2       = 6;
  const int64_t iAccuracyShift = 15;
  const int64_t iMaxVal        = 63;
  int64_t       iScaleShiftA2  = 0;
  int64_t       iScaleShiftA1  = 0;

  uint8_t signA1 = numer < 0;
  uint8_t signA2 = denom < 0;

  numer = (signA1) ? -numer : numer;
  denom = (signA2) ? -denom : denom;
  if (isoffset)
  {
    denom = denom >> (3 * scale);
    if (!denom)
    {
      return numer;
    }
  }
  iScaleShiftA2 = getRMVFMSB(denom) - iShiftA2;

  if (iScaleShiftA2 < 0)
  {
    iScaleShiftA2 = 0;
  }
  int numerBitlength = getRMVFMSB(abs(numer));
  if (numerBitlength > 48)
  {
    iScaleShiftA1 = numerBitlength - 48;
  }
  int64_t iScaleShiftA = iScaleShiftA2 + iAccuracyShift - iScaleShiftA1;

  int64_t a2s = (denom >> iScaleShiftA2) > iMaxVal ? iMaxVal : (denom >> iScaleShiftA2);
  int64_t a1s = (numer >> iScaleShiftA1);

  int64_t aI64 = (a1s * (int64_t)g_rmvfMultApproxTbl[a2s]) >> (iScaleShiftA - 8 < 0 ? 0 : iScaleShiftA - 8);

  d = (signA1 + signA2 == 1) ? -aI64 : aI64;
  return Clip3(int64_t(-RMVF_PARAM_THRED), int64_t(RMVF_PARAM_THRED - 1), d);
}
void PU::xCalcRMVFParameters(std::vector<RMVFInfo> &mvpInfoVec, int64_t dMatrix[2][4], int64_t sumbbfinal[2][3][3],
                             int64_t sumebfinal[2][3], uint8_t shift, uint16_t addedSize)
{
  int     iNum       = int(mvpInfoVec.size());
  uint8_t initShift  = 3 * shift;
  uint8_t rightShift = 0;
  int64_t sumbb[2][3][3];
  int64_t sumeb[2][3];
  int64_t md[3][3];
  ////////////////// Extract statistics: Start
  // initialize values to zero
  if (!addedSize)
  {
    for (int c = 0; c < 2; c++)
    {
      for (int d = 0; d < 3; d++)
      {
        sumebfinal[c][d] = 0;
      }
      for (int d1 = 0; d1 < 3; d1++)
      {
        for (int d = 0; d < 3; d++)
        {
          sumbbfinal[c][d1][d] = 0;
        }
      }
    }
  }
  buildRegressionMatrix(mvpInfoVec, sumbbfinal, sumebfinal, addedSize);

  for (int c = 0; c < 2; c++)
  {
    for (int d = 0; d < 3; d++)
    {
      sumeb[c][d] = sumebfinal[c][d];
    }
    for (int d1 = 0; d1 < 3; d1++)
    {
      for (int d = 0; d < 3; d++)
      {
        sumbb[c][d1][d] = sumbbfinal[c][d1][d];
      }
    }
  }
  ////////////////// Extract statistics: End
  ////////////////// Extract Weight: Start
  bool    bBadMatrix = false;
  int64_t det1       = 0;
  int64_t det2       = (sumbb[0][2][0]) * ((sumbb[0][0][1] * sumbb[0][1][2] - sumbb[0][1][1] * sumbb[0][0][2])) -
    (sumbb[0][2][1]) * ((sumbb[0][0][0] * sumbb[0][1][2] - sumbb[0][1][0] * sumbb[0][0][2])) +
    (sumbb[0][2][2]) * ((sumbb[0][0][0] * sumbb[0][1][1] - sumbb[0][1][0] * sumbb[0][0][1]));
  if (det2 == 0)
  {
    bBadMatrix = true;
  }
  for (int c = 0; c < 2; c++)
  {
    // 1) find matrix parameters with cross-component model
    // Find w[c][d]
    if (!bBadMatrix)
    {
      for (int d = 0; d < 3; d++)
      {
        // Initialize md matrix
        rightShift = d == 2 ? initShift : 0;
        for (int i = 0; i < 3; i++)
        {
          for (int j = 0; j < 3; j++)
          {
            md[i][j] = sumbb[0][i][j];
          }
        }
        // Replace coloumn D in md matrix
        for (int j = 0; j < 3; j++)
        {
          md[j][d] = sumeb[c][j];
        }
        det1 = (md[2][0]) * ((md[0][1] * md[1][2] - md[1][1] * md[0][2]) >> rightShift) -
          (md[2][1]) * ((md[0][0] * md[1][2] - md[1][0] * md[0][2]) >> rightShift) +
          (md[2][2]) * ((md[0][0] * md[1][1] - md[1][0] * md[0][1]) >> rightShift);
        dMatrix[c][d] = det1;

        // calcualte offset
        if (d == 2)
        {
          dMatrix[c][3] = det2;
        }
      }   // for d
    }

    if (bBadMatrix)
    {
      // 2) Find simple weight and offset, with non-corss component between x and y, i.e.: MVx=weight[0]*x+offset[0],
      // MVy=weight[1]*y+offset[1]
      for (int d = 0; d < 3; d++)
      {
        dMatrix[c][d] = 0;
      }
      int64_t det1 = (iNum * sumeb[c][c] - sumbb[c][c][2] * sumeb[c][2]);
      int64_t det2 = (iNum * sumbb[c][c][c] - sumbb[c][c][2] * sumbb[c][c][2]);
      if (det2 == 0)
      {
        dMatrix[c][c] = 0;
        det1          = 0;
        det2          = 1;
        dMatrix[c][3] = iNum;
      }
      else
      {
        dMatrix[c][c] = iNum * det1;
        dMatrix[c][3] = iNum * det2;
      }
      dMatrix[c][2] = det2 * sumeb[c][2] - det1 * sumbb[c][c][2];
    }   // End "bBadMatrix"

  }   // end "for c"
      ////////////////// Extract Weights: End
  return;
}
void PU::getRMVFAffineGuideCand(const CodingUnit &pu, const CodingUnit &abovePU, AffineMergeCtx &affMrgCtx,
                                std::vector<RMVFInfo> mvp[2][4], int mrgCandIdx)
{
  const CodingStructure &cs = *pu.cs;

  int iNumPredDir = cs.slice->isInterP() ? 1 : 2;
  Mv  cMV[2][3];
  Mv  cMVOri[2][3];

  std::vector<RMVFInfo> mvpInfoVec[2];
  std::vector<RMVFInfo> mvpInfoVecOri;

  bool     available[2]   = { false, false };
  //-- Collect non-adj affine subblock info
  Position anchorPosAbove = abovePU.Y().topLeft();
  Position neibPos;
  int      stepsize     = 1 << ATMVP_SUB_BLOCK_SIZE;
  int      hSearchAbove = abovePU.lwidth();
  int      vSearchAbove = abovePU.lheight();
  size_t   blkCnt       = 0;
  size_t   thred[2];

  const Position anchorPos    = pu.Y().topLeft();
  bool           enoughBlk[2] = { false, false };
  int            startX =
    anchorPos.x - anchorPosAbove.x > RMVF_DISTANCE_THRED ? anchorPos.x - RMVF_DISTANCE_THRED - anchorPosAbove.x : 0;
  int endX = abovePU.Y().topRight().x - anchorPos.x > RMVF_DISTANCE_THRED
    ? hSearchAbove - (abovePU.Y().topRight().x - anchorPos.x - RMVF_DISTANCE_THRED) - 1
    : hSearchAbove;
  int startY =
    anchorPos.y - anchorPosAbove.y > RMVF_DISTANCE_THRED ? anchorPos.y - RMVF_DISTANCE_THRED - anchorPosAbove.y : 0;
  int  endY     = abovePU.Y().bottomLeft().y - anchorPos.y > RMVF_DISTANCE_THRED
         ? vSearchAbove - (abovePU.Y().bottomLeft().y - anchorPos.y - RMVF_DISTANCE_THRED) - 1
         : vSearchAbove;
  bool outsideX = anchorPos.x - abovePU.Y().topRight().x > RMVF_DISTANCE_THRED ||
    anchorPosAbove.x - anchorPos.x >= RMVF_DISTANCE_THRED;
  bool outsideY = anchorPos.y - abovePU.Y().bottomLeft().y > RMVF_DISTANCE_THRED ||
    anchorPosAbove.y - anchorPos.y >= RMVF_DISTANCE_THRED;
  CodingUnit  tempPU = abovePU;
  MotionBuf   mb     = tempPU.getMotionBuf();
  MotionInfo *mi     = mb.buf;
  mi += (startY >> MIN_CU_LOG2) * mb.stride;
  if (!outsideX && !outsideY)
  {
    if (abovePU.interDir & 1)
    {
      thred[0] = RMVF_NUM_SUBBLK_THRED - mvp[RPL0][abovePU.refIdx[RPL0]].size();
      for (int i = startY; i < endY; i += stepsize)
      {
        for (int j = startX; j < endX; j += stepsize)
        {
          neibPos = anchorPosAbove.offset(j + 2, i + 2);
          mvpInfoVec[RPL0].push_back(RMVFInfo(mi[j >> MIN_CU_LOG2].mv[RPL0], neibPos, -1));
          blkCnt++;
          if (blkCnt == thred[0])
          {
            break;
          }
        }
        if (blkCnt == thred[0])
        {
          break;
        }
        mi += mb.stride;
      }
    }
    blkCnt = 0;
    mi     = mb.buf;
    mi += (startY >> MIN_CU_LOG2) * mb.stride;
    if (abovePU.interDir & 2)
    {
      thred[1] = RMVF_NUM_SUBBLK_THRED - mvp[RPL1][abovePU.refIdx[RPL1]].size();
      for (int i = startY; i < endY; i += stepsize)
      {
        for (int j = startX; j < endX; j += stepsize)
        {
          neibPos = anchorPosAbove.offset(j + 2, i + 2);
          mvpInfoVec[RPL1].push_back(RMVFInfo(mi[j >> MIN_CU_LOG2].mv[RPL1], neibPos, -1));
          blkCnt++;
          if (blkCnt == thred[1])
          {
            break;
          }
        }
        if (blkCnt == thred[1])
        {
          break;
        }
        mi += mb.stride;
      }
    }
  }
  if (mvpInfoVec[RPL0].size() > 3)
  {
    enoughBlk[0] = true;
  }
  if (mvpInfoVec[RPL1].size() > 3)
  {
    enoughBlk[1] = true;
  }
  for (unsigned int list = 0; list < iNumPredDir; ++list)
  {
    RefPicList eRefPicList = list == 0 ? RPL0 : RPL1;
    if (abovePU.interDir == 1 && eRefPicList == RPL1)
    {
      continue;
    }
    if (abovePU.interDir == 2 && eRefPicList == RPL0)
    {
      continue;
    }
    if (!enoughBlk[eRefPicList])
    {
      continue;
    }
    const Position posLT = pu.Y().topLeft();

    int64_t cMvX = 0, cMvY = 0;
    Mv      cMv;
    Mv      firstMv;
    int64_t parametersRMVF[2][4];
    firstMv.set(mvpInfoVec[eRefPicList][0].mvp.getHor(), mvpInfoVec[eRefPicList][0].mvp.getVer());
    computeDeltaAndShift(posLT, firstMv, mvpInfoVec[eRefPicList]);

    //-- Model with Linear Regression
    //-- Calculate RMVF parameters:
    int64_t sumbb[2][3][3];
    int64_t sumeb[2][3];
    int     denumShift = (getRMVFMSB(mvpInfoVec[eRefPicList].size()) - 5);
    if (denumShift < 0)
    {
      denumShift = 0;
    }

    // top-left CPMV
    xCalcRMVFParameters(mvpInfoVec[eRefPicList], parametersRMVF, sumbb, sumeb, denumShift, 0);
    for (int i = 0; i < 2; i++)
    {
      parametersRMVF[0][i] = divideRMVF(parametersRMVF[0][i], parametersRMVF[0][3], denumShift, false);
      parametersRMVF[1][i] = divideRMVF(parametersRMVF[1][i], parametersRMVF[1][3], denumShift, false);
    }
    parametersRMVF[0][2] = divideRMVF(parametersRMVF[0][2], parametersRMVF[0][3], denumShift, true);
    parametersRMVF[1][2] = divideRMVF(parametersRMVF[1][2], parametersRMVF[1][3], denumShift, true);
    cMvX                 = parametersRMVF[0][2];
    cMvY                 = parametersRMVF[1][2];
    cMvX                 = (firstMv.getHor() << MAX_CU_DEPTH) + cMvX;
    cMvY                 = (firstMv.getVer() << MAX_CU_DEPTH) + cMvY;
    int iMvX             = int(cMvX);
    int iMvY             = int(cMvY);
    roundAffineMv(iMvX, iMvY, MAX_CU_DEPTH);
    cMv             = Mv(iMvX, iMvY);
    cMVOri[list][0] = cMv;
    cMVOri[list][0].clipToStorageBitDepth();

    // top-right CPMV
    cMvX = parametersRMVF[0][0] * pu.lumaSize().width + parametersRMVF[0][2];
    cMvY = parametersRMVF[1][0] * pu.lumaSize().width + parametersRMVF[1][2];
    cMvX = (firstMv.getHor() << MAX_CU_DEPTH) + cMvX;
    cMvY = (firstMv.getVer() << MAX_CU_DEPTH) + cMvY;
    iMvX = int(cMvX);
    iMvY = int(cMvY);
    roundAffineMv(iMvX, iMvY, MAX_CU_DEPTH);
    cMv = Mv(iMvX, iMvY);

    cMVOri[list][1] = cMv;
    cMVOri[list][1].clipToStorageBitDepth();

    // bottom-left CPMV
    cMvX = parametersRMVF[0][1] * pu.lumaSize().height + parametersRMVF[0][2];
    cMvY = parametersRMVF[1][1] * pu.lumaSize().height + parametersRMVF[1][2];

    cMvX = (firstMv.getHor() << MAX_CU_DEPTH) + cMvX;
    cMvY = (firstMv.getVer() << MAX_CU_DEPTH) + cMvY;
    iMvX = int(cMvX);
    iMvY = int(cMvY);
    roundAffineMv(iMvX, iMvY, MAX_CU_DEPTH);
    cMv = Mv(iMvX, iMvY);

    cMVOri[list][2] = cMv;
    cMVOri[list][2].clipToStorageBitDepth();
    int refIdx = 0;

    refIdx = abovePU.refIdx[eRefPicList];
    if ((int)mvp[eRefPicList][refIdx].size() < 3)
    {
      available[eRefPicList] = true;
      cMV[list][0]           = cMVOri[list][0];
      cMV[list][1]           = cMVOri[list][1];
      cMV[list][2]           = cMVOri[list][2];
      mvpInfoVec[eRefPicList].clear();
      mvpInfoVecOri.clear();
      continue;
    }
    for (int i = 0; i < (int)(mvp[eRefPicList][refIdx]).size(); i++)
    {
      mvpInfoVecOri.push_back(mvp[eRefPicList][refIdx][i]);
    }

    uint16_t addedSize = (uint16_t)mvpInfoVecOri.size();

    computeDeltaAndShiftAddi(posLT, firstMv, mvpInfoVecOri, mvpInfoVec[eRefPicList]);
    denumShift = (getRMVFMSB(mvpInfoVec[eRefPicList].size()) - 5);
    if (denumShift < 0)
    {
      denumShift = 0;
    }
    //-- Model with Linear Regression
    //-- Calculate RMVF parameters:
    //-- Generate prediction signal
    cMvX = 0, cMvY = 0;
    // top-left CPMV
    xCalcRMVFParameters(mvpInfoVec[eRefPicList], parametersRMVF, sumbb, sumeb, denumShift, addedSize);
    for (int i = 0; i < 2; i++)
    {
      parametersRMVF[0][i] = divideRMVF(parametersRMVF[0][i], parametersRMVF[0][3], denumShift, false);
      parametersRMVF[1][i] = divideRMVF(parametersRMVF[1][i], parametersRMVF[1][3], denumShift, false);
    }
    parametersRMVF[0][2] = divideRMVF(parametersRMVF[0][2], parametersRMVF[0][3], denumShift, true);
    parametersRMVF[1][2] = divideRMVF(parametersRMVF[1][2], parametersRMVF[1][3], denumShift, true);
    cMvX                 = parametersRMVF[0][2];
    cMvY                 = parametersRMVF[1][2];
    cMvX                 = (firstMv.getHor() << MAX_CU_DEPTH) + cMvX;
    cMvY                 = (firstMv.getVer() << MAX_CU_DEPTH) + cMvY;
    iMvX                 = int(cMvX);
    iMvY                 = int(cMvY);
    roundAffineMv(iMvX, iMvY, MAX_CU_DEPTH);
    cMv          = Mv(iMvX, iMvY);
    cMV[list][0] = cMv;
    cMV[list][0].clipToStorageBitDepth();

    // top-right CPMV
    cMvX = parametersRMVF[0][0] * pu.lumaSize().width + parametersRMVF[0][2];
    cMvY = parametersRMVF[1][0] * pu.lumaSize().width + parametersRMVF[1][2];

    cMvX = (firstMv.getHor() << MAX_CU_DEPTH) + cMvX;
    cMvY = (firstMv.getVer() << MAX_CU_DEPTH) + cMvY;
    iMvX = int(cMvX);
    iMvY = int(cMvY);
    roundAffineMv(iMvX, iMvY, MAX_CU_DEPTH);
    cMv = Mv(iMvX, iMvY);

    cMV[list][1] = cMv;
    cMV[list][1].clipToStorageBitDepth();
    // bottom-left CPMV
    cMvX = parametersRMVF[0][1] * pu.lumaSize().height + parametersRMVF[0][2];
    cMvY = parametersRMVF[1][1] * pu.lumaSize().height + parametersRMVF[1][2];

    cMvX = (firstMv.getHor() << MAX_CU_DEPTH) + cMvX;
    cMvY = (firstMv.getVer() << MAX_CU_DEPTH) + cMvY;
    iMvX = int(cMvX);
    iMvY = int(cMvY);
    roundAffineMv(iMvX, iMvY, MAX_CU_DEPTH);
    cMv = Mv(iMvX, iMvY);

    cMV[list][2] = cMv;
    cMV[list][2].clipToStorageBitDepth();
    available[eRefPicList] = true;

    mvpInfoVec[eRefPicList].clear();
    mvpInfoVecOri.clear();
  }
  int8_t referenceidx[2];
  referenceidx[0] = abovePU.refIdx[0];
  referenceidx[1] = abovePU.refIdx[1];
  if (enoughBlk[0] || enoughBlk[1])
  {
    if (!xCPMVSimCheck(pu, affMrgCtx, cMVOri, abovePU.interDir, referenceidx, AffineModel::_6_PARAMS, abovePU.bcwIdx,
                       false))
    {
      int i = affMrgCtx.numValidMergeCand;
      for (int mvNum = 0; mvNum < 3; mvNum++)
      {
        affMrgCtx.mvFieldNeighbours[i][mvNum][0].setMvField(cMVOri[0][mvNum], referenceidx[0]);
        affMrgCtx.mvFieldNeighbours[i][mvNum][1].setMvField(cMVOri[1][mvNum], referenceidx[1]);
      }
      affMrgCtx.interDirNeighbours[i] = abovePU.interDir;
      affMrgCtx.affineType[i]         = AffineModel::_6_PARAMS;
      affMrgCtx.mergeType[i]          = MergeType::DEFAULT_N;
      affMrgCtx.bcwIdx[i]             = abovePU.bcwIdx;
      CHECK(!pu.cs->sps->m_biLicEnabledFlag && abovePU.licFlag && abovePU.interDir == 3, "abovePU has biPred and LIC");
      affMrgCtx.LICFlags[i] = abovePU.licFlag;
      if (affMrgCtx.numValidMergeCand == mrgCandIdx)
      {
        affMrgCtx.numValidMergeCand++;
        return;
      }
      affMrgCtx.numValidMergeCand++;
      if (affMrgCtx.numValidMergeCand == affMrgCtx.maxNumMergeCand)
      {
        return;
      }
    }
  }
  if (available[RPL0] || available[RPL1])
  {
    if (!xCPMVSimCheck(pu, affMrgCtx, cMV, abovePU.interDir, referenceidx, AffineModel::_6_PARAMS, BCW_DEFAULT, false))
    {
      int i = affMrgCtx.numValidMergeCand;
      for (int mvNum = 0; mvNum < 3; mvNum++)
      {
        affMrgCtx.mvFieldNeighbours[i][mvNum][0].setMvField(cMV[0][mvNum], referenceidx[0]);
        affMrgCtx.mvFieldNeighbours[i][mvNum][1].setMvField(cMV[1][mvNum], referenceidx[1]);
      }
      affMrgCtx.interDirNeighbours[i] = abovePU.interDir;
      affMrgCtx.affineType[i]         = AffineModel::_6_PARAMS;
      affMrgCtx.mergeType[i]          = MergeType::DEFAULT_N;
      affMrgCtx.bcwIdx[i]             = BCW_DEFAULT;
      affMrgCtx.LICFlags[i]           = false;
      if (affMrgCtx.numValidMergeCand == mrgCandIdx)
      {
        affMrgCtx.numValidMergeCand++;
        return;
      }
      affMrgCtx.numValidMergeCand++;
      if (affMrgCtx.numValidMergeCand == affMrgCtx.maxNumMergeCand)
      {
        return;
      }
    }
  }
}
void PU::xReturnMvpVec(std::vector<RMVFInfo> mvp[2][4], const CodingUnit &pu, const Position &pos)
{
  CodingStructure  &cs      = *pu.cs;
  const CodingUnit *neibPU  = NULL;
  Position          neibPos = pos;
  neibPU                    = cs.getCURestricted(neibPos, pu, pu.chType);
  if (neibPU == NULL || !CU::isInter(*neibPU))
  {
    return;
  }
  const MotionInfo &neibMi = neibPU->getMotionInfo(neibPos);
  if (neibMi.refIdx[0] >= 0)
  {
    mvp[0][neibMi.refIdx[0]].push_back(RMVFInfo(neibMi.mv[0], neibPos, neibMi.refIdx[0]));
  }
  if (neibMi.refIdx[1] >= 0)
  {
    mvp[1][neibMi.refIdx[1]].push_back(RMVFInfo(neibMi.mv[1], neibPos, neibMi.refIdx[1]));
  }
}
void PU::collectNeiMotionInfo(std::vector<RMVFInfo> mvpInfoVec[NUM_RPL01][4], const CodingUnit &pu)
{
  Position anchorPos = pu.Y().topLeft();
  Position neibPos;

  int imHeight  = pu.cs->slice->m_pic->lheight();
  int imWidth   = pu.cs->slice->m_pic->lwidth();
  int hSearch   = pu.lwidth();
  int vSearch   = pu.lheight();
  int horSearch = hSearch >> 1;
  int verSearch = vSearch >> 1;
  int stepsize  = 4;
  if (pu.lumaSize().width > RMVF_CUSIZE_THRED)
  {
    stepsize = 8;
  }
  neibPos = anchorPos;
  if (neibPos.y - 2 >= 0)
  {
    neibPos = anchorPos.offset(-2, -2);
    for (int j = 0; j < hSearch; j += stepsize)
    {
      neibPos.x += stepsize;
      xReturnMvpVec(mvpInfoVec, pu, neibPos

      );
    }
    neibPos     = anchorPos.offset(2, -2);
    int leftend = anchorPos.x - horSearch < 0 ? anchorPos.x : horSearch;
    for (int j = 0; j < leftend; j += stepsize)
    {
      neibPos.x -= stepsize;
      xReturnMvpVec(mvpInfoVec, pu, neibPos);
    }
    neibPos      = anchorPos.offset(hSearch - 2, -2);
    int rightend = pu.Y().topRight().x + horSearch > imWidth - 1 ? imWidth - 1 - pu.Y().topRight().x : horSearch;
    rightend     = pu.lumaSize().width == RMVF_DISTANCE_THRED ? 0 : rightend;
    for (int j = 0; j < rightend; j += stepsize)
    {
      neibPos.x += stepsize;
      xReturnMvpVec(mvpInfoVec, pu, neibPos);
    }
  }
  neibPos  = anchorPos;
  stepsize = 4;
  if (pu.lumaSize().height > RMVF_CUSIZE_THRED)
  {
    stepsize = 8;
  }
  if (neibPos.x - 2 >= 0)
  {
    neibPos = anchorPos.offset(-2, -2);
    for (int i = 0; i < vSearch; i += stepsize)
    {
      neibPos.y += stepsize;
      xReturnMvpVec(mvpInfoVec, pu, neibPos);
    }
    neibPos      = anchorPos.offset(-2, vSearch - 2);
    int belowend = pu.Y().bottomLeft().y + verSearch > imHeight - 1 ? imHeight - 1 - pu.Y().bottomLeft().y : verSearch;
    belowend     = pu.lumaSize().height == RMVF_DISTANCE_THRED ? 0 : belowend;
    for (int j = 0; j < belowend; j += stepsize)
    {
      neibPos.y += stepsize;
      xReturnMvpVec(mvpInfoVec, pu, neibPos);
    }
  }
}

bool CU::hasSubCUNonZeroMVd(const CodingUnit &_cu)
{
  bool nonZeroMvd = false;

  CodingUnit &cu = const_cast<CodingUnit &>(_cu);

  if ((!cu.mergeFlag) && (!cu.skip))
  {
    if (cu.interDir != 2 /* PRED_L1 */)
    {
      nonZeroMvd |= cu.mvd[RPL0].getHor() != 0;
      nonZeroMvd |= cu.mvd[RPL0].getVer() != 0;
    }
    if (cu.interDir != 1 /* PRED_L0 */)
    {
      if (!cu.cs->picHeader->m_mvdL1ZeroFlag || cu.interDir != 3 /* PRED_BI */)
      {
        nonZeroMvd |= cu.mvd[RPL1].getHor() != 0;
        nonZeroMvd |= cu.mvd[RPL1].getVer() != 0;
      }
    }
  }

  return nonZeroMvd;
}

bool CU::hasSubCUNonZeroAffineMVd(const CodingUnit &_cu)
{
  bool nonZeroAffineMvd = false;

  if (!_cu.affine || _cu.mergeFlag)
  {
    return false;
  }

  CodingUnit &cu = const_cast<CodingUnit &>(_cu);

  if ((!cu.mergeFlag) && (!cu.skip))
  {
    if (cu.interDir != 2 /* PRED_L1 */)
    {
      for (int i = 0; i < cu.getNumAffineMvs(); i++)
      {
        nonZeroAffineMvd |= cu.mvdAffi[RPL0][i].getHor() != 0;
        nonZeroAffineMvd |= cu.mvdAffi[RPL0][i].getVer() != 0;
      }
    }

    if (cu.interDir != 1 /* PRED_L0 */)
    {
      if (!cu.cs->picHeader->m_mvdL1ZeroFlag || cu.interDir != 3 /* PRED_BI */)
      {
        for (int i = 0; i < cu.getNumAffineMvs(); i++)
        {
          nonZeroAffineMvd |= cu.mvdAffi[RPL1][i].getHor() != 0;
          nonZeroAffineMvd |= cu.mvdAffi[RPL1][i].getVer() != 0;
        }
      }
    }
  }

  return nonZeroAffineMvd;
}

uint8_t CU::getSbtInfo(uint8_t idx, uint8_t pos) { return (pos << 4) + (idx << 0); }

uint8_t CU::getSbtIdx(const uint8_t sbtInfo) { return (sbtInfo >> 0) & 0xf; }

uint8_t CU::getSbtPos(const uint8_t sbtInfo) { return (sbtInfo >> 4) & 0x3; }

uint8_t CU::getSbtMode(uint8_t sbtIdx, uint8_t sbtPos)
{
  uint8_t sbtMode = 0;
  switch (sbtIdx)
  {
  case SBT_VER_HALF:
    sbtMode = sbtPos + SBT_VER_H0;
    break;
  case SBT_HOR_HALF:
    sbtMode = sbtPos + SBT_HOR_H0;
    break;
  case SBT_VER_QUAD:
    sbtMode = sbtPos + SBT_VER_Q0;
    break;
  case SBT_HOR_QUAD:
    sbtMode = sbtPos + SBT_HOR_Q0;
    break;
  case SBT_QUAD:
    sbtMode = sbtPos + SBT_Q0;
    break;
  case SBT_QUARTER:
    sbtMode = sbtPos + SBT_QT0;
    break;
  default:
    assert(0);
  }

  assert(sbtMode < NUMBER_SBT_MODE);
  return sbtMode;
}

uint8_t CU::getSbtIdxFromSbtMode(uint8_t sbtMode)
{
  if (sbtMode <= SBT_VER_H1)
  {
    return SBT_VER_HALF;
  }
  else if (sbtMode <= SBT_HOR_H1)
  {
    return SBT_HOR_HALF;
  }
  else if (sbtMode <= SBT_VER_Q1)
  {
    return SBT_VER_QUAD;
  }
  else if (sbtMode <= SBT_HOR_Q1)
  {
    return SBT_HOR_QUAD;
  }
  else if (sbtMode <= SBT_Q3)
  {
    return SBT_QUAD;
  }
  else if (sbtMode <= SBT_QT3)
  {
    return SBT_QUARTER;
  }
  else
  {
    assert(0);
    return 0;
  }
}

uint8_t CU::getSbtPosFromSbtMode(uint8_t sbtMode)
{
  if (sbtMode <= SBT_VER_H1)
  {
    return sbtMode - SBT_VER_H0;
  }
  else if (sbtMode <= SBT_HOR_H1)
  {
    return sbtMode - SBT_HOR_H0;
  }
  else if (sbtMode <= SBT_VER_Q1)
  {
    return sbtMode - SBT_VER_Q0;
  }
  else if (sbtMode <= SBT_HOR_Q1)
  {
    return sbtMode - SBT_HOR_Q0;
  }
  else if (sbtMode <= SBT_Q3)
  {
    return sbtMode - SBT_Q0;
  }
  else if (sbtMode <= SBT_QT3)
  {
    return sbtMode - SBT_QT0;
  }
  else
  {
    assert(0);
    return 0;
  }
}

uint8_t CU::targetSbtAllowed(uint8_t sbtIdx, uint8_t sbtAllowed)
{
  uint8_t val = 0;
  switch (sbtIdx)
  {
  case SBT_VER_HALF:
    val = ((sbtAllowed >> SBT_VER_HALF) & 0x1);
    break;
  case SBT_HOR_HALF:
    val = ((sbtAllowed >> SBT_HOR_HALF) & 0x1);
    break;
  case SBT_VER_QUAD:
    val = ((sbtAllowed >> SBT_VER_QUAD) & 0x1);
    break;
  case SBT_HOR_QUAD:
    val = ((sbtAllowed >> SBT_HOR_QUAD) & 0x1);
    break;
  case SBT_QUAD:
    val = ((sbtAllowed >> SBT_QUAD) & 0x1);
    break;
  case SBT_QUARTER:
    val = ((sbtAllowed >> SBT_QUARTER) & 0x1);
    break;
  default:
    THROW("unknown SBT type");
  }
  return val;
}

uint8_t CU::numSbtModeRdo(uint8_t sbtAllowed)
{
  uint8_t num = 0;
  uint8_t sum = 0;
  num         = targetSbtAllowed(SBT_VER_HALF, sbtAllowed) + targetSbtAllowed(SBT_HOR_HALF, sbtAllowed);
  sum += std::min(SBT_NUM_RDO, (num << 1));

  num = ((targetSbtAllowed(SBT_VER_QUAD, sbtAllowed) + targetSbtAllowed(SBT_HOR_QUAD, sbtAllowed)) << 1) +
    (CU::targetSbtAllowed(SBT_QUAD, sbtAllowed) << 2);
  sum += std::min<uint8_t>(SBT_NUM_RDO, num);

  num = targetSbtAllowed(SBT_QUARTER, sbtAllowed);
  sum += std::min(SBT_NUM_RDO, (num << 2));
  return sum;
}

bool CU::isSbtMode(const uint8_t sbtInfo)
{
  const uint8_t sbtIdx = getSbtIdx(sbtInfo);
  return sbtIdx >= SBT_VER_HALF && sbtIdx < NUMBER_SBT_IDX;
}

bool CU::isSameSbtSize(const uint8_t sbtInfo1, const uint8_t sbtInfo2)
{
  const uint8_t sbtIdx1 = getSbtIdxFromSbtMode(sbtInfo1);
  const uint8_t sbtIdx2 = getSbtIdxFromSbtMode(sbtInfo2);
  if (sbtIdx1 == SBT_HOR_HALF || sbtIdx1 == SBT_VER_HALF)
  {
    return sbtIdx2 == SBT_HOR_HALF || sbtIdx2 == SBT_VER_HALF;
  }
  else if (sbtIdx1 == SBT_VER_QUAD || sbtIdx1 == SBT_HOR_QUAD || sbtIdx1 == SBT_QUAD)
  {
    return (sbtIdx2 == SBT_VER_QUAD || sbtIdx2 == SBT_HOR_QUAD || sbtIdx2 == SBT_QUAD);
  }
  else if (sbtIdx1 == SBT_QUARTER)
  {
    return sbtIdx2 == SBT_QUARTER;
  }
  else
  {
    return false;
  }
}

bool CU::isSgpmCoded(const CodingUnit &cu)
{
  if (!cu.cs->sps->m_useSgpm)
  {
    return false;
  }
  if (!(cu.lwidth() >= GEO_MIN_CU_SIZE_EX && cu.lheight() >= GEO_MIN_CU_SIZE_EX && cu.lwidth() <= GEO_MAX_CU_SIZE_EX &&
        cu.lheight() <= GEO_MAX_CU_SIZE_EX && cu.lwidth() < 8 * cu.lheight() && cu.lheight() < 8 * cu.lwidth() &&
        cu.lwidth() * cu.lheight() >= SGPM_MIN_PIX))
  {
    return false;
  }
  if (!(cu.lx() && cu.ly()))
  {
    return false;
  }
  return true;
}

bool CU::isBcwIdxCoded(const CodingUnit &cu)
{
  if (cu.cs->sps->m_useBcw == false)
  {
    CHECK(cu.bcwIdx != BCW_DEFAULT, "Error: cu.bcwIdx != BCW_DEFAULT");
    return false;
  }

  if (CU::isIBC(cu))
  {
    return false;
  }

  if (CU::isIntra(cu) || cu.cs->slice->isInterP())
  {
    return false;
  }

  if (cu.lwidth() * cu.lheight() < BCW_SIZE_CONSTRAINT)
  {
    return false;
  }

  if (!cu.mergeFlag)
  {
    if (cu.interDir == 3)
    {
      const int refIdx0 = cu.refIdx[RPL0];
      const int refIdx1 = cu.refIdx[RPL1];

      const WPScalingParam *wp0 = cu.cs->slice->getWpScaling(RPL0, refIdx0);
      const WPScalingParam *wp1 = cu.cs->slice->getWpScaling(RPL1, refIdx1);

      return !(WPScalingParam::isWeighted(wp0) || WPScalingParam::isWeighted(wp1));
    }
  }

  return false;
}

uint8_t CU::getValidBcwIdx(const CodingUnit &cu)
{
  if (cu.interDir == 3 && !cu.mergeFlag)
  {
    return cu.bcwIdx;
  }
  else if (cu.interDir == 3 && cu.mergeFlag && cu.mergeType == MergeType::DEFAULT_N)
  {
    // This is intended to do nothing here.
  }
  else if (cu.mergeFlag && cu.mergeType == MergeType::SUBPU_ATMVP)
  {
    CHECK(cu.bcwIdx != BCW_DEFAULT, " cu.bcwIdx != BCW_DEFAULT ");
  }
  else
  {
    CHECK(cu.bcwIdx != BCW_DEFAULT, " cu.bcwIdx != BCW_DEFAULT ");
  }

  return BCW_DEFAULT;
}

bool CU::bdpcmAllowed(const CodingUnit &cu, const CompID compID)
{
  SizeType tsMaxSize    = 1 << cu.cs->sps->m_log2MaxTransformSkipBlockSize;
  bool     bdpcmAllowed = cu.cs->sps->m_bdpcmEnabledFlag && CU::isIntra(cu);

  if (isLuma(compID))
  {
    bdpcmAllowed &= (cu.lwidth() <= tsMaxSize && cu.lheight() <= tsMaxSize);
  }
  else
  {
    bdpcmAllowed &= (cu.chromaSize().width <= tsMaxSize && cu.chromaSize().height <= tsMaxSize);
  }

  return bdpcmAllowed;
}

bool CU::pltAllowed(const CodingUnit &cu)
{
  bool pltAllowed = cu.cs->slice->m_sps->m_PLTMode && cu.lwidth() <= 64 && cu.lheight() <= 64;
  pltAllowed &= CS::isDualITree(*cu.cs) || isLuma(cu.chType);

  if (isLuma(cu.chType))
  {
    pltAllowed &= cu.Y().area() > 16;
  }
  else
  {
    pltAllowed &= cu.Cb().area() > 16;
  }

  return pltAllowed;
}

bool CU::isObmcAllowed(const CodingUnit &cu)
{
  if (!cu.cs->sps->m_useObmc || CU::isIBC(cu) || CU::isIntra(cu) || cu.lwidth() * cu.lheight() < 32)
  {
    return false;
  }
  return true;
}

CU::TimdRefTypePositionAndSize CU::deriveTimdRefTypePositionAndSize(const CodingUnit &cu, const int iTemplateWidth,
                                                                    const int iTemplateHeight)
{
  const auto iCurX = cu.lx();
  const auto iCurY = cu.ly();

  if (iCurX == 0 && iCurY == 0)
  {
    return {};
  }

  TimdRefTypePositionAndSize tps;
  if (iCurX > 0 && iCurY > 0)
  {
    tps.iRefPosition  = { iCurX - iTemplateWidth, iCurY - iTemplateHeight };
    tps.uiRefSize     = { cu.lwidth() + iTemplateWidth, cu.lheight() + iTemplateHeight };
    tps.eTemplateType = LEFT_ABOVE_NEIGHBOR;
  }
  else if (iCurX == 0 && iCurY > 0)
  {
    tps.iRefPosition  = { iCurX, iCurY - iTemplateHeight };
    tps.uiRefSize     = { cu.lwidth(), cu.lheight() };
    tps.eTemplateType = ABOVE_NEIGHBOR;
  }
  else if (iCurX > 0 && iCurY == 0)
  {
    tps.iRefPosition  = { iCurX - iTemplateWidth, iCurY };
    tps.uiRefSize     = { cu.lwidth(), cu.lheight() };
    tps.eTemplateType = LEFT_NEIGHBOR;
  }
  else
  {
    assert(0);
  }
  return tps;
}

bool CU::isLICFlagPresent(const CodingUnit &cu)
{
  if (CU::isIntra(cu) || !cu.cs->slice->m_useLic)
  {
    return false;
  }

  if (cu.geoFlag || cu.mergeFlag || (!cu.cs->sps->m_biLicEnabledFlag && cu.interDir == 3) || CU::isIBC(cu) ||
      cu.Y().area() < LIC_MIN_CU_PIXELS)
  {
    return false;
  }

  if (!PU::checkRprLicCondition(cu))
  {
    return false;
  }

  return true;
}

bool CU::hasOppositeLICFlag(const CodingUnit &cu)
{
  if (cu.cs->sps->m_mergeOppositeLic && cu.slice->m_useLic && cu.cs->slice->m_uiTLayer < 5 && cu.mergeFlag)
  {
    int blkArea = cu.lumaSize().area();
    if (cu.affine)
    {
      return !cu.mmvdMergeFlag && !cu.bmMergeFlag && blkArea > 64 && blkArea < 8192 && cu.cs->sps->m_useAffine &&
        cu.cs->sps->m_maxNumAffineOppositeLicMergeCand > 0;
    }
    else
    {
      return !cu.geoFlag && cu.regularMergeFlag && !cu.bmMergeFlag && blkArea >= 32 && blkArea < 16384;
    }
  }
  return false;
}

void PU::spanLicFlags(CodingUnit &cu, const bool LICFlag)
{
  MotionBuf   mb         = cu.getMotionBuf();
  MotionInfo *motionInfo = mb.buf;

  for (int y = 0; y < mb.height; y++)
  {
    for (int x = 0; x < mb.width; x++)
    {
      motionInfo[x].usesLIC = LICFlag;
    }
    motionInfo += mb.stride;
  }
}

int deriveAffineSubBlkSize(const int sz, const int minSbSz, const int deltaMvX, const int deltaMvY, const int shift)
{
  int sbSz = minSbSz;
  if (deltaMvX == 0 && deltaMvY == 0)
  {
    sbSz = sz;
  }
  else
  {
    int maxDmv = std::max(abs(deltaMvX), abs(deltaMvY)) * sbSz;
    int thred  = 1 << (shift - 1);
    while (maxDmv < thred && sbSz < sz)
    {
      sbSz <<= 1;
      maxDmv <<= 1;
    }
  }
  return sbSz;
}

void PU::deriveAffineCandFromMvField(Position posLT, const int width, const int height, std::vector<RMVFInfo> mvInfoVec,
                                     Mv mvAffi[3])
{
  int64_t cMvX = 0, cMvY = 0;
  Mv      cMv;
  Mv      firstMv;
  int64_t parametersRMVF[2][4];

  firstMv.set(mvInfoVec[0].mvp.getHor(), mvInfoVec[0].mvp.getVer());
  computeDeltaAndShift(posLT, firstMv, mvInfoVec);

  //-- Model with Linear Regression
  //-- Calculate RMVF parameters:
  int64_t sumbb[2][3][3];
  int64_t sumeb[2][3];

  int denumShift = (getRMVFMSB(mvInfoVec.size()) - 5);
  if (denumShift < 0)
  {
    denumShift = 0;
  }

  xCalcRMVFParameters(mvInfoVec, parametersRMVF, sumbb, sumeb, denumShift, 0);
  for (int i = 0; i < 2; i++)
  {
    parametersRMVF[0][i] = divideRMVF(parametersRMVF[0][i], parametersRMVF[0][3], denumShift, false);
    parametersRMVF[1][i] = divideRMVF(parametersRMVF[1][i], parametersRMVF[1][3], denumShift, false);
  }
  parametersRMVF[0][2] = divideRMVF(parametersRMVF[0][2], parametersRMVF[0][3], denumShift, true);
  parametersRMVF[1][2] = divideRMVF(parametersRMVF[1][2], parametersRMVF[1][3], denumShift, true);
  cMvX                 = parametersRMVF[0][2];
  cMvY                 = parametersRMVF[1][2];
  cMvX                 = (firstMv.getHor() << MAX_CU_DEPTH) + cMvX;
  cMvY                 = (firstMv.getVer() << MAX_CU_DEPTH) + cMvY;
  int iMvX             = int(cMvX);
  int iMvY             = int(cMvY);
  roundAffineMv(iMvX, iMvY, MAX_CU_DEPTH);
  cMv       = Mv(iMvX, iMvY);
  mvAffi[0] = cMv;
  mvAffi[0].clipToStorageBitDepth();

  cMvX = parametersRMVF[0][0] * width + parametersRMVF[0][2];
  cMvY = parametersRMVF[1][0] * width + parametersRMVF[1][2];
  cMvX = (firstMv.getHor() << MAX_CU_DEPTH) + cMvX;
  cMvY = (firstMv.getVer() << MAX_CU_DEPTH) + cMvY;
  iMvX = int(cMvX);
  iMvY = int(cMvY);
  roundAffineMv(iMvX, iMvY, MAX_CU_DEPTH);
  cMv = Mv(iMvX, iMvY);

  mvAffi[1] = cMv;
  mvAffi[1].clipToStorageBitDepth();

  // bottom-left CPMV
  cMvX = parametersRMVF[0][1] * height + parametersRMVF[0][2];
  cMvY = parametersRMVF[1][1] * height + parametersRMVF[1][2];

  cMvX = (firstMv.getHor() << MAX_CU_DEPTH) + cMvX;
  cMvY = (firstMv.getVer() << MAX_CU_DEPTH) + cMvY;
  iMvX = int(cMvX);
  iMvY = int(cMvY);
  roundAffineMv(iMvX, iMvY, MAX_CU_DEPTH);
  cMv = Mv(iMvX, iMvY);

  mvAffi[2] = cMv;
  mvAffi[2].clipToStorageBitDepth();
}

bool CU::hasCcFilterFlag(const CodingUnit &cu)
{
  if (cu.cccmType == CONV_MODEL_CCCM_BVG)
  {
    return false;
  }

  if (cu.cclmOffsets.isActive())
  {
    return false;
  }

  if (!cu.cs->slice->m_sps->m_ccBoostFilter)
  {
    return false;
  }

  return cu.intraDir[ChannelType::CHROMA] == MMLM_CHROMA_IDX;
}

bool CU::hasDecoderDerivedCCP(const CodingUnit &cu)
{
  if (!cu.cs->slice->m_sps->m_ccDecDerivedMode || cu.chromaSize().width * cu.chromaSize().height <= 16)
  {
    return false;
  }

  const Area area = cu.blocks[COMP_Cb];

  return (area.x > 0 || area.y > 0);
}

bool CU::hasNonLocalCCP(const CodingUnit &cu)
{
  if (!cu.cs->slice->m_sps->m_ccMerge || cu.chromaSize().width * cu.chromaSize().height <= 16)
  {
    return false;
  }

  if (!CS::isDualITree(*cu.cs))
  {
    return false;
  }

  return true;
}

int CU::getCCPModelCandidateList(const CodingUnit &cu, CrossCompModels candList[], int selIdx)
{
  //
  // NOTE: Missing inter + temporal stuff still
  //

  int     maxCandIdx   = 0;
  bool    found1stCCLM = false;
  int64_t scaleCclm[2] = { 0 };
  int64_t shiftCclm[2] = { 3 };

  int iW = cu.blocks[1].width;
  int iH = cu.blocks[1].height;

  auto tryToAddNewModels = [&](const CrossCompModels &candModels)
  {
    for (int j = 0; j < maxCandIdx; j++)
    {
      if (candModels == candList[j])
      {
        return maxCandIdx;
      }
    }

    if (!found1stCCLM && candModels.modelCb[0].modelType == CONV_MODEL_CCLM)
    {
      scaleCclm[0] = candModels.modelCb[0].params[0];
      shiftCclm[0] = candModels.modelCb[0].params[2];
      scaleCclm[1] = candModels.modelCr[0].params[0];
      shiftCclm[1] = candModels.modelCr[0].params[2];
      found1stCCLM = true;
    }

    candList[maxCandIdx++] = candModels;

    return maxCandIdx;
  };

  const Position posCand[5] = { cu.chromaPos().offset(-1, iH - 1), cu.chromaPos().offset(iW - 1, -1),
                                cu.chromaPos().offset(-1, iH), cu.chromaPos().offset(iW, -1),
                                cu.chromaPos().offset(-1, -1) };

  for (const Position &posLT: posCand)
  {
    const CodingUnit *cuRef = cu.cs->getCURestricted(posLT, cu, ChannelType::CHROMA);

    if (cuRef != nullptr && cuRef->ccModels.valid)
    {
      if (tryToAddNewModels(cuRef->ccModels) == MAX_CCP_CAND_LIST_SIZE)
      {
        return maxCandIdx;
      }
    }
  }

  int offsetX  = 0;
  int offsetY  = 0;
  int offsetX0 = 0;
  int offsetX1 = 0;
  int offsetX2 = cu.chType == ChannelType::LUMA ? cu.lwidth() >> 1 : cu.Cb().width >> 1;
  int offsetY0 = 0;
  int offsetY1 = 0;
  int offsetY2 = cu.chType == ChannelType::LUMA ? cu.lheight() >> 1 : cu.Cb().height >> 1;

  const int horNAInterval =
    std::max((int)(cu.chType == ChannelType::LUMA ? cu.lwidth() * 2 : cu.Cb().width * 2) >> 1, 4);
  const int verNAInterval =
    std::max((int)(cu.chType == ChannelType::LUMA ? cu.lheight() * 2 : cu.Cb().height * 2) >> 1, 4);
  const int numNACandidate[7] = { 5, 9, 9, 9, 9, 9, 9 };
  const int idxMap[7][9]      = { { 0, 1, 2, 3, 4 },
                                  { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
                                  { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
                                  { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
                                  { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
                                  { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
                                  { 0, 1, 2, 3, 4, 5, 6, 7, 8 } };

  for (int iDistanceIndex = 0; iDistanceIndex < 7 && maxCandIdx < MAX_CCP_CAND_LIST_SIZE; iDistanceIndex++)
  {
    const int iNADistanceHor = horNAInterval * (iDistanceIndex + 1);
    const int iNADistanceVer = verNAInterval * (iDistanceIndex + 1);

    for (int naspIdx = 0; naspIdx < numNACandidate[iDistanceIndex] && maxCandIdx < MAX_CCP_CAND_LIST_SIZE; naspIdx++)
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
        printf("error!");
        exit(0);
        break;
      }

      Position posLT(cu.chromaPos().x + offsetX, cu.chromaPos().y + offsetY);

      const CodingUnit *cuRef = cu.cs->getCURestricted(posLT, cu, ChannelType::CHROMA);

      if (cuRef != nullptr && cuRef->ccModels.valid)
      {
        if (tryToAddNewModels(cuRef->ccModels) == MAX_CCP_CAND_LIST_SIZE)
        {
          return maxCandIdx;
        }
      }
    }
  }

  // Non-adjacent candidates round 2
  const int numNACandidate2[7] = { 4, 4, 4, 4, 4, 4, 4 };
  const int idxMap2[7][5]      = { { 0, 1, 2, 3 }, { 0, 1, 2, 3 }, { 0, 1, 2, 3 }, { 0, 1, 2, 3 },
                                   { 0, 1, 2, 3 }, { 0, 1, 2, 3 }, { 0, 1, 2, 3 } };

  for (int iDistanceIndex = 0; iDistanceIndex < 7 && maxCandIdx < MAX_CCP_CAND_LIST_SIZE; iDistanceIndex++)
  {
    const int horNADistance = horNAInterval * (iDistanceIndex + 1);
    const int verNADistance = verNAInterval * (iDistanceIndex + 1);

    for (int naspIdx = 0; naspIdx < numNACandidate2[iDistanceIndex] && maxCandIdx < MAX_CCP_CAND_LIST_SIZE; naspIdx++)
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
        printf("error!");
        exit(0);
        break;
      }

      Position posLT(cu.chromaPos().x + offsetX, cu.chromaPos().y + offsetY);

      const CodingUnit *cuRef = cu.cs->getCURestricted(posLT, cu, ChannelType::CHROMA);

      if (cuRef != nullptr && cuRef->ccModels.valid)
      {
        if (tryToAddNewModels(cuRef->ccModels) == MAX_CCP_CAND_LIST_SIZE)
        {
          return maxCandIdx;
        }
      }
    }
  }

  auto tryHistCCP = [&](const LutCCP &ccpLut)
  {
    for (int idx = 0; idx < ccpLut.lutCCP.size(); idx++)
    {
      CrossCompModels curModel;
      cu.cs->getOneModelFromCCPLut(ccpLut.lutCCP, curModel, idx);
      if (tryToAddNewModels(curModel) == MAX_CCP_CAND_LIST_SIZE)
      {
        return maxCandIdx;
      }
    }
    return -1;
  };

  int ret = tryHistCCP(cu.cs->ccpLut);

  if (ret != -1)
  {
    return ret;
  }

  if (maxCandIdx < MAX_CCP_CAND_LIST_SIZE)
  {
    unsigned  uiInternalBitDepth               = cu.cs->sps->m_bitDepths[ChannelType::CHROMA];
    const int defaultA[MAX_CCP_CAND_LIST_SIZE] = { 0, 1, -1, 2, -2, 3, -3, 4, -4, 5, -5, 6 };
    const int defaultB                         = 1 << (uiInternalBitDepth - 1);
    const int defaultShift                     = 3;

    for (int posIdx = 0; posIdx < MAX_CCP_CAND_LIST_SIZE; posIdx++)
    {
      CclmModelSingle cclmModelCb(int(scaleCclm[0]), defaultB, int(shiftCclm[0]));
      CclmModelSingle cclmModelCr(int(scaleCclm[1]), defaultB, int(shiftCclm[1]));

      if (found1stCCLM && defaultA[posIdx])
      {
        int dCb = cclmModelCb.a > 0 ? -defaultA[posIdx] : defaultA[posIdx];
        if (cclmModelCb.shift < defaultShift)
        {
          cclmModelCb.a <<= (defaultShift - cclmModelCb.shift);
          cclmModelCb.shift = defaultShift;
        }
        else if (cclmModelCb.shift > defaultShift)
        {
          dCb <<= (cclmModelCb.shift - defaultShift);
        }
        cclmModelCb.a += dCb;

        int dCr = cclmModelCr.a > 0 ? -defaultA[posIdx] : defaultA[posIdx];
        if (cclmModelCr.shift < defaultShift)
        {
          cclmModelCr.a <<= (defaultShift - cclmModelCr.shift);
          cclmModelCr.shift = defaultShift;
        }
        else if (cclmModelCr.shift > defaultShift)
        {
          dCr <<= (cclmModelCr.shift - defaultShift);
        }
        cclmModelCr.a += dCr;
      }
      else
      {
        cclmModelCb.a = cclmModelCr.a = defaultA[posIdx];
        cclmModelCb.shift = cclmModelCr.shift = defaultShift;
      }

      CrossCompModels ccModelCand;

      ccModelCand.init(false);
      ccModelCand.modelCb[0] = ConvModel(cclmModelCb);
      ccModelCand.modelCr[0] = ConvModel(cclmModelCr);

      bool duplication = false;
      for (int j = 0; j < maxCandIdx; j++)
      {
        if (candList[j] == ccModelCand)
        {
          duplication = true;
          // THROW("Should not duplicaten");
          break;
        }
      }
      if (duplication)
      {
        continue;
      }
      candList[maxCandIdx] = ccModelCand;
      if (selIdx == maxCandIdx)
      {
        return maxCandIdx + 1;
      }
      maxCandIdx++;
      if (maxCandIdx == MAX_CCP_CAND_LIST_SIZE)
      {
        return maxCandIdx;
      }
    }
  }

  CHECK(maxCandIdx > MAX_CCP_CAND_LIST_SIZE, "Invlid number of non-adj CCCM candidates");

  return maxCandIdx;
}

void CU::saveModelsInHCCP(const CodingUnit &cu)
{
  if (cu.chromaFormat == ChromaFormat::_400 || (CS::isDualITree(*cu.cs) && cu.chType == ChannelType::LUMA) ||
      !CU::isIntra(cu))
  {
    return;
  }

  CodingStructure &cs = *cu.cs;

  if (PU::isLMCMode(cu.intraDir[ChannelType::CHROMA]) && cu.ccModels.valid)
  {
    cs.addCCPToLut(cs.ccpLut.lutCCP, cu.ccModels, -1);
  }
}

bool PU::isObicAvail(const CodingUnit &cu)
{
  for (auto &neighbour: cu.obicNeighbours)
  {
    if (neighbour == nullptr)
    {
      continue;
    }
    if (neighbour->timdFlag || neighbour->dimdFlag || neighbour->sgpm)
    {
      return true;
    }
    else if (neighbour->eipFlag && cu.slice->m_eSliceType != I_SLICE)
    {
      return true;
    }
    else if (CU::isIntra(*neighbour) && !CU::isIBC(*neighbour) && !neighbour->mipFlag &&
             /*!neighbour->tmpFlag && !neighbour->eipFlag &&*/ !CU::isPLT(*neighbour))
    {
      return true;
    }
  }

  return false;
}

int PU::getObicNeighbours(const CodingUnit &cu, std::vector<const CodingUnit *> &cuNeighbours)
{
  const int step = 4;
  if (!cu.lumaPos().x && !cu.lumaPos().y)
  {
    return 0;
  }
  if (!cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType) || !cu.slice->m_sps->m_useDIMD ||
      cu.Y().area() <= 32)
  {
    return 0;
  }

  const int numCUs = NUM_OBIC_CUS;
  cuNeighbours.resize(numCUs);
  /* -----------------------------------------------------------------
  Step 1: Collect adjacent neighbour cands
  Step 2: Collect non-adjacent neighbour cands
  Step 3: Sort neighbours by distance
  ----------------------------------------------------------------- */
  /* -----------------------------------------------------------------
  ----------- Step 1: Collect adjacent neighbour cands ---------------
  ----------------------------------------------------------------- */
  cuNeighbours[0] = cu.cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, ChannelType::LUMA);
  cuNeighbours[1] = cu.cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, ChannelType::LUMA);
  cuNeighbours[2] = cu.cs->getCURestricted(cu.lumaPos().offset(-1, -1), cu, ChannelType::LUMA);

  const CodingUnit *cuTemp;
  for (int i = 0; i <= cu.lheight(); i += step)
  {
    cuTemp = cu.cs->getCURestricted(cu.lumaPos().offset(-1, i), cu, ChannelType::LUMA);
    if (cuTemp && CU::isIntra(*cuTemp))
    {
      cuNeighbours[0] = cuTemp;
      break;
    }
  }
  for (int i = 0; i <= cu.lwidth(); i += step)
  {
    cuTemp = cu.cs->getCURestricted(cu.lumaPos().offset(i, -1), cu, ChannelType::LUMA);
    if (cuTemp && CU::isIntra(*cuTemp))
    {
      cuNeighbours[1] = cuTemp;
      break;
    }
  }

  const CodingUnit *cuLeft = cu.cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, ChannelType::LUMA);
  const CodingUnit *cuTop  = cu.cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, ChannelType::LUMA);
  cuNeighbours[3]          = cuLeft
             ? cu.cs->getCURestricted(cuLeft->lumaPos().offset(cuLeft->lwidth() - 1, cuLeft->lheight()), cu, ChannelType::LUMA)
             : nullptr;
  cuNeighbours[4]          = cuTop
             ? cu.cs->getCURestricted(cuTop->lumaPos().offset(cuTop->lwidth(), cuTop->lheight() - 1), cu, ChannelType::LUMA)
             : nullptr;
  cuNeighbours[5]          = cuNeighbours[3]
             ? cu.cs->getCURestricted(
        cuNeighbours[3]->lumaPos().offset(cuNeighbours[3]->lwidth() - 1, cuNeighbours[3]->lheight()), cu,
        ChannelType::LUMA)
             : nullptr;
  cuNeighbours[6]          = cuNeighbours[4]
             ? cu.cs->getCURestricted(
        cuNeighbours[4]->lumaPos().offset(cuNeighbours[4]->lwidth(), cuNeighbours[4]->lheight() - 1), cu,
        ChannelType::LUMA)
             : nullptr;
  cuNeighbours[7] = cuLeft ? cu.cs->getCURestricted(cuLeft->lumaPos().offset(-1, 0), cu, ChannelType::LUMA) : nullptr;
  cuNeighbours[8] = cuTop ? cu.cs->getCURestricted(cuTop->lumaPos().offset(0, -1), cu, ChannelType::LUMA) : nullptr;
  cuNeighbours[9] =
    cuNeighbours[3] ? cu.cs->getCURestricted(cuNeighbours[3]->lumaPos().offset(-1, 0), cu, ChannelType::LUMA) : nullptr;
  cuNeighbours[10] =
    cuNeighbours[4] ? cu.cs->getCURestricted(cuNeighbours[4]->lumaPos().offset(0, -1), cu, ChannelType::LUMA) : nullptr;
  cuNeighbours[11] =
    cuNeighbours[5] ? cu.cs->getCURestricted(cuNeighbours[5]->lumaPos().offset(-1, 0), cu, ChannelType::LUMA) : nullptr;
  cuNeighbours[12] =
    cuNeighbours[6] ? cu.cs->getCURestricted(cuNeighbours[6]->lumaPos().offset(0, -1), cu, ChannelType::LUMA) : nullptr;

  if ((!cuNeighbours[9]) && cuNeighbours[2])
  {
    cuNeighbours[9] = cu.cs->getCURestricted(cuNeighbours[2]->lumaPos().offset(-1, cuNeighbours[2]->lheight() - 1), cu,
                                             ChannelType::LUMA);
  }
  if ((!cuNeighbours[10]) && cuNeighbours[2])
  {
    cuNeighbours[10] = cu.cs->getCURestricted(cuNeighbours[2]->lumaPos().offset(cuNeighbours[2]->lwidth() - 1, -1), cu,
                                              ChannelType::LUMA);
  }
  if ((!cuNeighbours[11]) && cuLeft && cuNeighbours[7])
  {
    cuNeighbours[11] = cu.cs->getCURestricted(cuNeighbours[7]->lumaPos().offset(-1, 0), cu, ChannelType::LUMA);
  }
  if ((!cuNeighbours[12]) && cuTop && cuNeighbours[8])
  {
    cuNeighbours[12] = cu.cs->getCURestricted(cuNeighbours[8]->lumaPos().offset(0, -1), cu, ChannelType::LUMA);
  }

  /* -----------------------------------------------------------------
  ---------- Step 2: Collect non-adjacent neighbour cands-------------
  ----------------------------------------------------------------- */
  const CompArea &area              = cu.Y();
  const Position  topLeft           = area.topLeft();
  int             offsetX           = 0;
  int             offsetY           = 0;
  int             cout              = 13;
  const int       numNACandidate[4] = { 3, 5, 5, 5 };
  const int       idxMap[4][5]      = { { 0, 1, 4 }, { 0, 1, 2, 3, 4 }, { 0, 1, 2, 3, 4 }, { 0, 1, 2, 3, 4 } };
  for (int iDistanceIndex = 0; iDistanceIndex < NADISTANCE_LEVEL; iDistanceIndex++)
  {
    const int iNADistanceHor = cu.lwidth() * (iDistanceIndex + 1);
    const int iNADistanceVer = cu.lheight() * (iDistanceIndex + 1);
    for (int iNASPIdx = 0; iNASPIdx < numNACandidate[iDistanceIndex]; iNASPIdx++)
    {
      switch (idxMap[iDistanceIndex][iNASPIdx])
      {
      case 0:
        offsetX = -iNADistanceHor - 1;
        offsetY = cu.lheight() + iNADistanceVer - 1;
        break;
      case 1:
        offsetX = cu.lwidth() + iNADistanceHor - 1;
        offsetY = -iNADistanceVer - 1;
        break;
      case 2:
        offsetX = cu.lwidth() >> 1;
        offsetY = -iNADistanceVer - 1;
        break;
      case 3:
        offsetX = -iNADistanceHor - 1;
        offsetY = cu.lheight() >> 1;
        break;
      case 4:
        offsetX = -iNADistanceHor - 1;
        offsetY = -iNADistanceVer - 1;
        break;
      default:
        THROW("error!");
        break;
      }
      cuNeighbours[cout++] = cu.cs->getCURestricted(topLeft.offset(offsetX, offsetY), cu, ChannelType::LUMA);
    }
  }

  /* -----------------------------------------------------------------
  ---------------- Step 3: Sort neighbours by distance ---------------
  ----------------------------------------------------------------- */

  bool useNeighbour[numCUs] {};
  int  neighboursInDistOrder[numCUs] {};
  int  dists[numCUs];
  std::fill_n(dists, numCUs, std::numeric_limits<int>::max());

  for (int i = 0; i < numCUs; i++)
  {
    useNeighbour[i] = (cuNeighbours[i] && (CU::isIntra(*cuNeighbours[i]) || CU::isInter(*cuNeighbours[i])));
    if (cuNeighbours[i] && (CU::isInter(*cuNeighbours[i])))
    {
      bool useInter = false;
      if (cuNeighbours[i]->geoFlag)
      {
        int ipm = g_geoAngle2IntraAng[g_geoParams[cuNeighbours[i]->geoSplitDir].angleIdx];
        if (ipm > PLANAR_IDX && ipm < NUM_LUMA_MODE)
        {
          useInter = true;
        }
      }
      useNeighbour[i] &= useInter;
    }
    if (useNeighbour[i])
    {
      for (int j = i - 1; j >= 0; j--)
      {
        if (!useNeighbour[j])
        {
          continue;
        }
        useNeighbour[i] &=
          (cuNeighbours[i]->lx() != cuNeighbours[j]->lx() || cuNeighbours[i]->ly() != cuNeighbours[j]->ly());
      }
    }
    int curNeigh = i;
    int curDist  = (useNeighbour[i] ? abs((int)(cu.lx()) - (int)(cuNeighbours[i]->lx())) +
                       abs((int)(cu.ly()) - (int)(cuNeighbours[i]->ly()))
                                    : 0);

    for (int j = 0; j < numCUs; j++)
    {
      if (curDist < dists[j])
      {
        for (int k = numCUs - 1; k > j; k--)
        {
          dists[k]                 = dists[k - 1];
          neighboursInDistOrder[k] = neighboursInDistOrder[k - 1];
        }
        dists[j]                 = curDist;
        neighboursInDistOrder[j] = curNeigh;
        break;
      }
    }
  }
  // keep at most 20 of the closest neighbours:
  // remove neighbours with the farthest top left pixel
  int       numToMix      = 0;
  const int limitMaxNeigh = 20;
  for (int i = 0; i < numCUs; i++)
  {
    int j = neighboursInDistOrder[i];
    if (limitMaxNeigh > 0 && numToMix >= limitMaxNeigh)
    {
      cuNeighbours[j] = nullptr;
      useNeighbour[j] = false;
      continue;
    }
    if (useNeighbour[j])
    {
      numToMix++;
    }
    else
    {
      cuNeighbours[j] = nullptr;
    }
  }
  return numToMix;
}

bool CU::eipAllowed(const CodingUnit &cu, const CompID &compId)
{
  if (!cu.cs->sps->m_useEIP)
  {
    return false;
  }

  const uint32_t width  = cu.blocks[compId].width;
  const uint32_t height = cu.blocks[compId].height;

  if (width > MAX_EIP_SIZE || height > MAX_EIP_SIZE)
  {
    return false;
  }

  if (cu.blocks[compId].area() < 64)
  {
    return false;
  }

  const int numAboveAvail = cu.blocks[compId].y;
  const int numLeftAvail  = cu.blocks[compId].x;
  const int tplSize       = std::min(cu.blocks[compId].height, cu.blocks[compId].width) + EIP_FILTER_SIZE;
  if (numAboveAvail < tplSize || numLeftAvail < tplSize)
  {
    return false;
  }

  return true;
}

bool CU::eipMergeAllowed(const CodingUnit &cu, const CompID &compId)
{
  if (!cu.cs->sps->m_useEIP)
  {
    return false;
  }

  const uint32_t width  = cu.blocks[compId].width;
  const uint32_t height = cu.blocks[compId].height;

  if (width > MAX_EIP_SIZE || height > MAX_EIP_SIZE)
  {
    return false;
  }

  const int numAboveAvail = cu.blocks[compId].y;
  const int numLeftAvail  = cu.blocks[compId].x;

  if ((numAboveAvail >= (EIP_FILTER_SIZE + EIP_TPL_SIZE)) && (numLeftAvail >= (EIP_FILTER_SIZE + EIP_TPL_SIZE)))
  {
    return true;
  }

  return false;
}

void CU::eipCurAllowed(const CodingUnit &cu, const CompID &compId, static_vector<EipInfo, NUM_DERIVED_EIP> &eipInfoList,
                       bool isMultiModel)
{
  const int log2Wm2 = floorLog2(cu.blocks[compId].width) - 2;
  const int log2Hm2 = floorLog2(cu.blocks[compId].height) - 2;
  if (cu.cs->sps->m_useMMEIP)
  {
    if (!isMultiModel)
    {
      const int numOfCombEIP[4][4] = {
        { 0, 0, 1, 1 },
        { 0, 2, 2, 2 },
        { 1, 2, 3, 3 },
        { 1, 2, 3, 3 },
      };
      for (int i = 0; i < numOfCombEIP[log2Wm2][log2Hm2]; i++)
      {
        eipInfoList.push_back(g_eipInfoLutMultiModelOnFalse[log2Wm2][log2Hm2][i]);
      }
    }
    else
    {
      const int numOfCombMmEIP[4][4] = {
        { 0, 0, 2, 1 },
        { 0, 3, 3, 3 },
        { 2, 3, 5, 5 },
        { 1, 3, 5, 5 },
      };
      for (int i = 0; i < numOfCombMmEIP[log2Wm2][log2Hm2]; i++)
      {
        eipInfoList.push_back(g_eipInfoLutMultiModelOnTrue[log2Wm2][log2Hm2][i]);
      }
    }
  }
  else
  {
    if (isMultiModel)
    {
      return;
    }

    const int numOfCombEIP[4][4] = {
      { 3, 3, 3, 2 },
      { 3, 5, 5, 5 },
      { 3, 5, 9, 9 },
      { 2, 5, 9, 9 },
    };

    for (int i = 0; i < numOfCombEIP[log2Wm2][log2Hm2]; i++)
    {
      eipInfoList.push_back(g_eipInfoLutMultiModelOff[log2Wm2][log2Hm2][i]);
    }
  }
}

void CU::saveModelsInHEIP(const CodingUnit &cu)
{
  if (!cu.Y().valid())
  {
    return;
  }

  CodingStructure &cs = *cu.cs;
  if (cu.eipFlag)
  {
    cs.addEipToLut(cs.eipLut.lutEip, cu.eipModels, -1);
  }
}
// TU tools

bool TU::getCbf(const TransformUnit &tu, const CompID &compID) { return getCbfAtDepth(tu, compID, tu.depth); }

bool TU::getCbfAtDepth(const TransformUnit &tu, const CompID &compID, const unsigned &depth)
{
  return ((tu.cbf[compID] >> depth) & 1) == 1;
}

void TU::setCbfAtDepth(TransformUnit &tu, const CompID &compID, const unsigned &depth, const bool &cbf)
{
  // first clear the CBF at the depth
  tu.cbf[compID] &= ~(1 << depth);
  // then set the CBF
  tu.cbf[compID] |= ((cbf ? 1 : 0) << depth);
}

bool TU::isTSAllowed(const TransformUnit &tu, const CompID &compID)
{
  const int maxSize              = tu.cs->sps->m_log2MaxTransformSkipBlockSize;
  SizeType  transformSkipMaxSize = 1 << maxSize;

  bool tsAllowed = tu.cs->sps->m_transformSkipEnabledFlag;
  tsAllowed &= tu.blocks[compID].width <= transformSkipMaxSize && tu.blocks[compID].height <= transformSkipMaxSize;
  tsAllowed &= !tu.cu->sbtInfo;

  return tsAllowed;
}

bool TU::isMTSAllowed(const TransformUnit &tu, const CompID &compID)
{
  SizeType  tsMaxSize  = 1 << tu.cs->sps->m_log2MaxTransformSkipBlockSize;
  const int maxSize    = CU::isIntra(*tu.cu) ? MTS_INTRA_MAX_CU_SIZE : tu.cs->sps->m_interMTSMaxSize;
  const int cuWidth    = tu.cu->blocks[0].lumaSize().width;
  const int cuHeight   = tu.cu->blocks[0].lumaSize().height;
  bool      mtsAllowed = isLuma(tu.cu->chType) && compID == COMP_Y;

  mtsAllowed &= CU::isIntra(*tu.cu) ? tu.cs->sps->m_explicitMtsIntra : tu.cs->sps->m_explicitMtsInter;
  mtsAllowed &= cuWidth <= maxSize && cuHeight <= maxSize;
  mtsAllowed &= !tu.cu->sbtInfo;
  mtsAllowed &= !(tu.cu->bdpcmMode[0] != BdpcmMode::NONE && cuWidth <= tsMaxSize && cuHeight <= tsMaxSize);
  return mtsAllowed;
}

bool TU::lfnstAllowed(const TransformUnit &tu, const CompID &compID)
{
  const CodingUnit &cu       = *tu.cu;
  const Slice      &slice    = *cu.slice;
  const SPS        &sps      = *slice.m_sps;
  const bool        dualTree = CS::isDualITree(*tu.cs);
  const int         cidx     = dualTree && isChroma(cu.chType) ? 1 : 0;
  const bool        luma     = (compID == CompID::COMP_Y);
  bool              allow    = false;

  if (CU::isIntra(cu))
  {
    //--- intra blocks ---
    allow |= (slice.m_eSliceType == I_SLICE &&
              sps.m_useIntraLFNSTinISlice);   // enable if intra slice and sps enable flag for intra slices is set
    allow |= (slice.m_eSliceType != I_SLICE &&
              sps.m_useIntraLFNSTinPBSlice);   // enable if inter slice and sps enable flag for inter slices is set
    allow &= (!cu.mipFlag || (cu.lumaSize().width >= 4 && cu.lumaSize().height >= 4));   // disable for small MIP blocks
    allow &= (!cu.timdFlag || !cu.multiRefIdx);   // disable for TIMD with multi-ref index
    allow &= (cu.getBdpcmMode(compID) == BdpcmMode::NONE);   // disable for BDPCM
  }
  else if (CU::isInter(cu))
  {
    //--- inter blocks ---
    allow |= (cu.slice->m_sps->m_useInterLFNST);   // enable if sps flag is set
    allow &= (!cu.sbtInfo || cu.slice->m_sps->m_useInterLFNSTSBT);   // disable for SBT when inter LFNST-SBT is disabled
  }
  //--- general constraints ---
  allow &= (luma || dualTree);   // disable for single chroma tree
  allow &= (luma || !dualTree ||
            std::min(cu.blocks[1].width, cu.blocks[1].height) >= 4);   // disable for small chroma blocks (in dual tree)
  allow &= (cu.blocks[cidx].lumaSize().width <= cu.slice->m_sps->getMaxTbSize());   // disable for too large blocks
  allow &= (cu.blocks[cidx].lumaSize().height <= cu.slice->m_sps->getMaxTbSize());   // disable for tool large blocks
  return allow;
}

int TU::getNstIdx(const TransformUnit &tu, const CompID &compID)
{
  if (tu.mtsIdx[compID] == MtsType::DCT2_NST1)
  {
    return 1;
  }
  else if (tu.mtsIdx[compID] == MtsType::DCT2_NST2)
  {
    return 2;
  }
  else if (tu.mtsIdx[compID] == MtsType::DCT2_NST3)
  {
    return 3;
  }
  else
  {
    return 0;
  }
}

int TU::getICTMode(const TransformUnit &tu, int jointCbCr)
{
  if (jointCbCr < 0)
  {
    jointCbCr = tu.jointCbCr;
  }
  return g_ictModes[tu.cs->picHeader->m_jointCbCrSignFlag][jointCbCr];
}

TransList TU::getTransCandIntra(const TransformUnit &tu, const CompID &compID, const bool lossless, const bool chromaTS)
{
  TransList         tl           = {};
  const CodingUnit &cu           = *tu.cu;
  const bool        isTSAllowed  = TU::isTSAllowed(tu, compID);
  const bool        isMTSAllowed = TU::isMTSAllowed(tu, compID);
  const bool        lfnstAllowed = TU::lfnstAllowed(tu, compID);

  if (isLuma(compID))
  {
    if (lossless)
    {
      CHECK(!isTSAllowed, "transform skip should be enabled for LS");
      tl.push_back(MtsType::SKIP);
    }
    else
    {
      if (cu.getBdpcmMode(compID) == BdpcmMode::NONE)
      {
        tl.push_back(MtsType::DCT2_DCT2);
        if (isTSAllowed)
        {
          tl.push_back(MtsType::SKIP);
        }
        if (isMTSAllowed)
        {
          for (MtsType mtsIdx = MtsType(MtsType::SKIP + 1); mtsIdx < MtsType::NUM; mtsIdx++)
          {
            tl.push_back(mtsIdx);
          }
        }
        if (lfnstAllowed)
        {
          for (MtsType mtsIdx = MtsType(MtsType::DCT2_DCT2 + 1); mtsIdx < MtsType::SKIP; mtsIdx++)
          {
            tl.push_back(mtsIdx);
          }
        }
      }
      else
      {
        CHECK(!isTSAllowed, "transform skip should be enabled for BDPCM");
        tl.push_back(MtsType::SKIP);
      }
    }
  }
  else
  {
    const bool isTSAllowedChroma = isTSAllowed && chromaTS;

    if (lossless)
    {
      CHECK(!isTSAllowedChroma, "transform skip (chroma) should be enabled for LS");
      tl.push_back(MtsType::SKIP);
    }
    else
    {
      if (cu.getBdpcmMode(compID) == BdpcmMode::NONE)
      {
        tl.push_back(MtsType::DCT2_DCT2);
        if (isTSAllowedChroma)
        {
          tl.push_back(MtsType::SKIP);
        }
        if (lfnstAllowed)
        {
          for (MtsType mtsIdx = MtsType(MtsType::DCT2_DCT2 + 1); mtsIdx < MtsType::SKIP; mtsIdx++)
          {
            tl.push_back(mtsIdx);
          }
        }
      }
      else
      {
        CHECK(!isTSAllowedChroma, "transform skip (chroma) should be enabled for BDPCM");
        tl.push_back(MtsType::SKIP);
      }
    }
  }
  return tl;
}

TransList TU::getTransCandInter(const TransformUnit &tu, const CompID &compID, const bool lossless, const bool chromaTS,
                                const MtsType bestHistMTS, const uint8_t bestHistSBT)
{
  TransList  tl           = {};
  const bool isTSAllowed  = TU::isTSAllowed(tu, compID) && (isLuma(compID) || chromaTS);
  const bool isMTSAllowed = TU::isMTSAllowed(tu, compID);
  const bool lfnstAllowed = TU::lfnstAllowed(tu, compID);

  if (lossless)
  {
    CHECK(!isTSAllowed, "transform skip should be enabled for lossless");
    CHECK(tu.noResidual, "lossless cannot be coded with NoResidual");
    tl.push_back(MtsType::SKIP);
    return tl;
  }

  tl.push_back(MtsType::DCT2_DCT2);
  if (!tu.noResidual)
  {
    if (isTSAllowed)
    {
      tl.push_back(MtsType::SKIP);
    }
#if APPLY_SBT_SL_ON_MTS
    // skip MTS if DCT2 is the best
    if (isMTSAllowed && (!tu.cu->slice->m_sps->m_useSBT || CU::getSbtIdx(bestHistSBT) != SBT_OFF_DCT))
#else
    if (isMTSAllowed)
#endif
    {
      for (MtsType tr = MtsType::MTS_1; tr <= MtsType::MTS_4; tr++)
      {
#if APPLY_SBT_SL_ON_MTS
        // skip the non-best Mts mode
        if (!tu.cu->slice->m_sps->m_useSBT || (bestHistMTS == MtsType::NONE || bestHistMTS == tr))
#endif
        {
          tl.push_back(tr);
        }
      }
    }
    if (lfnstAllowed)
    {
      for (MtsType mtsIdx = MtsType(MtsType::DCT2_DCT2 + 1); mtsIdx < MtsType::SKIP; mtsIdx++)
      {
        tl.push_back(mtsIdx);
      }
    }
  }
  return tl;
}

bool TU::needsSqrt2Scale(const TransformUnit &tu, const CompID &compID)
{
  const Size &size            = tu.blocks[compID];
  const bool  isTransformSkip = tu.mtsIdx[compID] == MtsType::SKIP;
  return (!isTransformSkip) && (((floorLog2(size.width) + floorLog2(size.height)) & 1) == 1);
}

bool TU::needsBlockSizeTrafoScale(const TransformUnit &tu, const CompID &compID) { return needsSqrt2Scale(tu, compID); }

TransformUnit *TU::getPrevTU(const TransformUnit &tu, const CompID compID)
{
  TransformUnit *prevTU = tu.prev;

  if (prevTU != nullptr && (prevTU->cu != tu.cu || !prevTU->blocks[compID].valid()))
  {
    prevTU = nullptr;
  }

  return prevTU;
}

// other tools
bool storeContexts(const Slice *slice, const int ctuXPosInCtus, const int ctuYPosInCtus)
{
  if (slice->m_sps->m_tempCabacInitMode && !slice->isIntra())
  {
    const PreCalcValues &pcv       = *slice->m_pps->pcv;
    const int            ctuRsAddr = ctuXPosInCtus + ctuYPosInCtus * pcv.widthInCtus;
    return ctuRsAddr == pcv.sizeInCtus - 1;
  }
  return false;
}

bool shouldUse2x2EdgeOperator(const Area &area) { return area.area() <= 32; }

uint32_t getCtuAddr(const Position &pos, const PreCalcValues &pcv)
{
  return (pos.x >> pcv.maxCUWidthLog2) + (pos.y >> pcv.maxCUHeightLog2) * pcv.widthInCtus;
}

Size TU::getMaxLog2SignPredArea(const TransformUnit &tu, const CompID compID)
{
  const bool     isNST         = tu.checkNSTApplied(compID);
  const SizeType maxLog2SPArea = isNST ? MAX_LOG2_SIGN_PRED_SIZE_NST : MAX_LOG2_SIGN_PRED_SIZE;
  const SizeType log2Width     = floorLog2(tu.blocks[compID].width);
  const SizeType log2Height    = floorLog2(tu.blocks[compID].height);
  const SizeType log2SPWidth   = std::min<SizeType>(log2Width, maxLog2SPArea);
  const SizeType log2SPHeight  = std::min<SizeType>(log2Height, maxLog2SPArea);
  return Size(log2SPWidth, log2SPHeight);
}

Size TU::getSignPredArea(const TransformUnit &tu, const CompID compID)
{
  Size     maxLog2SPArea = TU::getMaxLog2SignPredArea(tu, compID);
  SizeType log2SPWidth   = std::min<SizeType>(maxLog2SPArea.width, tu.cs->sps->m_log2SignPredArea);
  SizeType log2SPHeight  = std::min<SizeType>(maxLog2SPArea.height, tu.cs->sps->m_log2SignPredArea);
  // make sure its not smaller than residual coding subblock size
  log2SPWidth  = std::min(log2SPWidth, std::max<SizeType>(MLS_CG_SIZE >> 1, floorLog2(tu.blocks[compID].width) - 1));
  log2SPHeight = std::min(log2SPHeight, std::max<SizeType>(MLS_CG_SIZE >> 1, floorLog2(tu.blocks[compID].height) - 1));
  return Size(1 << log2SPWidth, 1 << log2SPHeight);
}

bool TU::getDelayedSignCoding(const TransformUnit &tu, const CompID compID)
{
  if (tu.cs->sps->m_numPredSign <= 0 || tu.mtsIdx[compID] == MtsType::SKIP ||
      tu.cu->getBdpcmMode(compID) != BdpcmMode::NONE)
  {
    return false;
  }
  const uint32_t maxSize = CU::isIntra(*tu.cu) ? SIGN_PRED_MAX_BS_INTRA : SIGN_PRED_MAX_BS_INTER;
  const uint32_t width   = tu.blocks[compID].width;
  const uint32_t height  = tu.blocks[compID].height;
  if (width < 4 || height < 4 || width > maxSize || height > maxSize)
  {
    return false;
  }
  return true;
}

bool TU::getUseSignPred(const TransformUnit &tu, const CompID compID)
{
  return TU::getDelayedSignCoding(tu, compID) && (!isNST(tu.mtsIdx[compID]) || true /*!tu.cu->tmpFlag*/);
}
