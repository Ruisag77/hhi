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

/** \file     ContextModelling.cpp
    \brief    Classes providing probability descriptions and contexts
*/

#include <algorithm>

#include "ContextModelling.h"
#include "UnitTools.h"
#include "CodingStructure.h"
#include "Picture.h"

const int CoeffCodingContext::prefixCtx[] = { 0, 0, 0, 3, 6, 10, 15, 21, 28 };

CoeffCodingContext::CoeffCodingContext(const TransformUnit &tu, CompID component, bool signHide, const BdpcmMode bdpcm)
  : m_compID(component)
  , m_chType(toChannelType(m_compID))
  , m_nstIdx(TU::getNstIdx(tu, m_compID))
  , m_width(tu.block(m_compID).width)
  , m_height(tu.block(m_compID).height)
  , m_log2CGWidth(g_log2TxSubblockSize[floorLog2(m_width)][floorLog2(m_height)].width)
  , m_log2CGHeight(g_log2TxSubblockSize[floorLog2(m_width)][floorLog2(m_height)].height)
  , m_log2CGSize(m_log2CGWidth + m_log2CGHeight)
  , m_widthInGroups(getNonzeroTuSize(m_width) >> m_log2CGWidth)
  , m_heightInGroups(getNonzeroTuSize(m_height) >> m_log2CGHeight)
  , m_log2BlockWidth((unsigned)floorLog2(m_width))
  , m_log2BlockHeight((unsigned)floorLog2(m_height))
  , m_maxNumCoeff(m_width * m_height)
  , m_signHiding(signHide)
  , m_extendedPrecision(tu.cs->sps->m_spsRangeExtension.m_extendedPrecisionProcessingFlag)
  , m_maxLog2TrDynamicRange(tu.cs->sps->getMaxLog2TrDynamicRange(m_chType))
  , m_scan(g_scanOrder[SCAN_GROUPED_4x4][CoeffScanType::DIAG][gp_sizeIdxInfo->idxFrom(m_width)]
                      [gp_sizeIdxInfo->idxFrom(m_height)])
  , m_scanCG(g_scanOrder[SCAN_UNGROUPED][CoeffScanType::DIAG][gp_sizeIdxInfo->idxFrom(m_widthInGroups)]
                        [gp_sizeIdxInfo->idxFrom(m_heightInGroups)])
  , m_switchCondition(getSwitchCondition(*tu.cu, m_chType))
  , m_CtxSetLastX((m_switchCondition ? Ctx::LastXCtxSetSwitch : Ctx::LastX)[to_underlying(m_chType)])
  , m_CtxSetLastY((m_switchCondition ? Ctx::LastYCtxSetSwitch : Ctx::LastY)[to_underlying(m_chType)])
  , m_maxLastPosX(g_groupIdx[getNonzeroTuSize(m_width) - 1])
  , m_maxLastPosY(g_groupIdx[getNonzeroTuSize(m_height) - 1])
  , m_lastOffsetX(0)
  , m_lastOffsetY(0)
  , m_lastShiftX(0)
  , m_lastShiftY(0)
  , m_minCoeff(-(1 << tu.cs->sps->getMaxLog2TrDynamicRange(m_chType)))
  , m_maxCoeff((1 << tu.cs->sps->getMaxLog2TrDynamicRange(m_chType)) - 1)
  , m_scanPosLast(-1)
  , m_subSetId(-1)
  , m_subSetPos(-1)
  , m_subSetPosX(-1)
  , m_subSetPosY(-1)
  , m_minSubPos(-1)
  , m_maxSubPos(-1)
  , m_sigGroupCtxId(-1)
  , m_tmplCpSum1(-1)
  , m_tmplCpDiag(-1)
  , m_sigGroupCtxIdSwitch(-1)
  , m_sigGroupCtxIdTSSwitch(-1)
  , m_sigFlagCtxSet { (m_nstIdx              ? Ctx::SigFlagNST
                         : m_switchCondition ? Ctx::SigFlagCtxSetSwitch
                                             : Ctx::SigFlag)[to_underlying(m_chType)],
                      (m_nstIdx              ? Ctx::SigFlagNST
                         : m_switchCondition ? Ctx::SigFlagCtxSetSwitch
                                             : Ctx::SigFlag)[to_underlying(m_chType) + 2],
                      (m_nstIdx              ? Ctx::SigFlagNST
                         : m_switchCondition ? Ctx::SigFlagCtxSetSwitch
                                             : Ctx::SigFlag)[to_underlying(m_chType)],
                      (m_nstIdx              ? Ctx::SigFlagNST
                         : m_switchCondition ? Ctx::SigFlagCtxSetSwitch
                                             : Ctx::SigFlag)[to_underlying(m_chType) + 4] }
  , m_gtxFlagCtxSet { (m_nstIdx              ? Ctx::GtxFlagNST
                         : m_switchCondition ? Ctx::GtxFlagCtxSetSwitch
                                             : Ctx::GtxFlag)[to_underlying(m_chType)],
                      (m_nstIdx              ? Ctx::GtxFlagNST
                         : m_switchCondition ? Ctx::GtxFlagCtxSetSwitch
                                             : Ctx::GtxFlag)[to_underlying(m_chType) + 2],
                      (m_nstIdx              ? Ctx::GtxFlagNST
                         : m_switchCondition ? Ctx::GtxFlagCtxSetSwitch
                                             : Ctx::GtxFlag)[to_underlying(m_chType) + 4],
                      (m_nstIdx              ? Ctx::GtxFlagNST
                         : m_switchCondition ? Ctx::GtxFlagCtxSetSwitch
                                             : Ctx::GtxFlag)[to_underlying(m_chType) + 6] }
  , m_sigGroupCtxIdTS(-1)
  , m_tsSigFlagCtxSet(m_switchCondition ? Ctx::TsSigFlagCtxSetSwitch : Ctx::TsSigFlag)
  , m_tsParFlagCtxSet(m_switchCondition ? Ctx::TsParFlagCtxSetSwitch : Ctx::TsParFlag)
  , m_tsGtxFlagCtxSet(m_switchCondition ? Ctx::TsGtxFlagCtxSetSwitch : Ctx::TsGtxFlag)
  , m_tsLrg1FlagCtxSet(m_switchCondition ? Ctx::TsLrg1FlagCtxSetSwitch : Ctx::TsLrg1Flag)
  , m_tsSignFlagCtxSet(m_switchCondition ? Ctx::TsResidualSignCtxSetSwitch : Ctx::TsResidualSign)
  , m_sigCoeffGroupFlag()
  , m_bdpcm(bdpcm)
  , m_signPredArea()
  , m_numSignsPredArea(0)
{
  if (TU::getDelayedSignCoding(tu, component))
  {
    m_signPredArea = TU::getSignPredArea(tu, component);
  }

  // LOGTODO
  unsigned log2sizeX = m_log2BlockWidth;
  unsigned log2sizeY = m_log2BlockHeight;
  if (m_chType == ChannelType::CHROMA)
  {
    const_cast<int &>(m_lastShiftX) = Clip3(0, 2, int(m_width >> 3));
    const_cast<int &>(m_lastShiftY) = Clip3(0, 2, int(m_height >> 3));
  }
  else
  {
    const_cast<int &>(m_lastOffsetX) = prefixCtx[log2sizeX];
    const_cast<int &>(m_lastOffsetY) = prefixCtx[log2sizeY];

    const_cast<int &>(m_lastShiftX) = (log2sizeX + 1) >> 2;
    const_cast<int &>(m_lastShiftY) = (log2sizeY + 1) >> 2;
  }

  m_cctxBaseLevel = 4;   // default value for RRC rice derivation in VVCv1, is updated for extended RRC rice derivation
  m_histValue  = 0;   // default value for RRC rice derivation in VVCv1, is updated for history-based extention of RRC
                     // rice derivation
  m_updateHist = 0;   // default value for RRC rice derivation (history update is disabled), is updated for
                      // history-based extention of RRC rice derivation

  if (tu.cs->sps->m_spsRangeExtension.m_rrcRiceExtensionEnableFlag)
  {
    deriveRiceRRC = &CoeffCodingContext::deriveRiceExt;
  }
  else
  {
    deriveRiceRRC = &CoeffCodingContext::deriveRice;
  }
}

void CoeffCodingContext::initSubblock(int SubsetId, bool sigGroupFlag)
{
  m_subSetId   = SubsetId;
  m_subSetPos  = m_scanCG[m_subSetId].idx;
  m_subSetPosY = m_subSetPos / m_widthInGroups;
  m_subSetPosX = m_subSetPos - (m_subSetPosY * m_widthInGroups);
  m_minSubPos  = m_subSetId << m_log2CGSize;
  m_maxSubPos  = m_minSubPos + (1 << m_log2CGSize) - 1;
  if (sigGroupFlag)
  {
    m_sigCoeffGroupFlag.set(m_subSetPos);
  }
  unsigned CGPosY   = m_subSetPosY;
  unsigned CGPosX   = m_subSetPosX;
  unsigned sigRight = unsigned((CGPosX + 1) < m_widthInGroups ? m_sigCoeffGroupFlag[m_subSetPos + 1] : false);
  unsigned sigLower =
    unsigned((CGPosY + 1) < m_heightInGroups ? m_sigCoeffGroupFlag[m_subSetPos + m_widthInGroups] : false);
  m_sigGroupCtxId         = Ctx::SigCoeffGroup[to_underlying(m_chType)](sigRight | sigLower);
  unsigned sigLeft        = unsigned(CGPosX > 0 ? m_sigCoeffGroupFlag[m_subSetPos - 1] : false);
  unsigned sigAbove       = unsigned(CGPosY > 0 ? m_sigCoeffGroupFlag[m_subSetPos - m_widthInGroups] : false);
  m_sigGroupCtxIdTS       = Ctx::TsSigCoeffGroup(sigLeft + sigAbove);
  m_sigGroupCtxIdSwitch   = Ctx::SigCoeffGroupCtxSetSwitch[to_underlying(m_chType)](sigRight | sigLower);
  m_sigGroupCtxIdTSSwitch = Ctx::TsSigCoeffGroupCtxSetSwitch(sigLeft + sigAbove);
}

void DeriveCtx::CtxSplit(const CodingStructure &cs, Partitioner &partitioner, unsigned &ctxSpl, unsigned &ctxQt,
                         unsigned &ctxHv, unsigned &ctxHorBt, unsigned &ctxVerBt, bool *_canSplit /*= nullptr */) const
{
  const CodingUnit *cuLeft  = cuRestrictedLeft[to_underlying(partitioner.chType)];
  const CodingUnit *cuAbove = cuRestrictedAbove[to_underlying(partitioner.chType)];

  bool canSplit[6];

  if (_canSplit == nullptr)
  {
    partitioner.canSplit(cs, canSplit[0], canSplit[1], canSplit[2], canSplit[3], canSplit[4], canSplit[5]);
  }
  else
  {
    memcpy(canSplit, _canSplit, 6 * sizeof(bool));
  }

  ///////////////////////
  // CTX do split (0-8)
  ///////////////////////
  const unsigned widthCurr  = partitioner.currArea().block(partitioner.chType).width;
  const unsigned heightCurr = partitioner.currArea().block(partitioner.chType).height;

  ctxSpl = 0;

  if (cuLeft)
  {
    const unsigned heightLeft = cuLeft->block(partitioner.chType).height;
    ctxSpl += (heightLeft < heightCurr ? 1 : 0);
  }
  if (cuAbove)
  {
    const unsigned widthAbove = cuAbove->block(partitioner.chType).width;
    ctxSpl += (widthAbove < widthCurr ? 1 : 0);
  }

  unsigned numSplit = 0;
  if (canSplit[1])
  {
    numSplit += 2;
  }
  if (canSplit[2])
  {
    numSplit += 1;
  }
  if (canSplit[3])
  {
    numSplit += 1;
  }
  if (canSplit[4])
  {
    numSplit += 1;
  }
  if (canSplit[5])
  {
    numSplit += 1;
  }

  if (numSplit > 0)
  {
    numSplit--;
  }

  ctxSpl += 3 * (numSplit >> 1);

  int maxWidthHeight = std::max(partitioner.currArea().lwidth(), partitioner.currArea().lheight());
  if (partitioner.chType == ChannelType::LUMA && partitioner.currPartIdx() == 1 && partitioner.currBtDepth == 1 &&
      partitioner.currArea().lx() + maxWidthHeight <= cs.picture->lwidth() &&
      partitioner.currArea().ly() + maxWidthHeight <= cs.picture->lheight())
  {
    const PartLevel &partLevel = partitioner.currPartLevel();
    if ((partLevel.split == CU_HORZ_SPLIT && partLevel.firstSubPartSplit == CU_VERT_SPLIT) ||
        (partLevel.split == CU_VERT_SPLIT && partLevel.firstSubPartSplit == CU_HORZ_SPLIT))
    {
      ctxSpl = 9;
    }
  }

  //////////////////////////
  // CTX is qt split (0-5)
  //////////////////////////
  ctxQt = (cuLeft && cuLeft->qtDepth > partitioner.currQtDepth) ? 1 : 0;
  ctxQt += (cuAbove && cuAbove->qtDepth > partitioner.currQtDepth) ? 1 : 0;
  ctxQt += partitioner.currQtDepth < 2 ? 0 : 3;

  ////////////////////////////
  // CTX is ver split (0-4)
  ////////////////////////////
  ctxHv = 0;

  const unsigned numHor = (canSplit[2] ? 1 : 0) + (canSplit[4] ? 1 : 0);
  const unsigned numVer = (canSplit[3] ? 1 : 0) + (canSplit[5] ? 1 : 0);

  if (numVer == numHor)
  {
    const Area &area = partitioner.currArea().block(partitioner.chType);

    const unsigned wAbove = cuAbove ? cuAbove->block(partitioner.chType).width : 1;
    const unsigned hLeft  = cuLeft ? cuLeft->block(partitioner.chType).height : 1;

    const unsigned depAbove = area.width / wAbove;
    const unsigned depLeft  = area.height / hLeft;

    if (depAbove == depLeft || !cuLeft || !cuAbove)
    {
      ctxHv = 0;
    }
    else if (depAbove < depLeft)
    {
      ctxHv = 1;
    }
    else
    {
      ctxHv = 2;
    }
  }
  else if (numVer < numHor)
  {
    ctxHv = 3;
  }
  else
  {
    ctxHv = 4;
  }

  //////////////////////////
  // CTX is h/v bt (0-3)
  //////////////////////////
  ctxHorBt = (partitioner.currMtDepth <= 1 ? 1 : 0);
  ctxVerBt = (partitioner.currMtDepth <= 1 ? 3 : 2);
}

unsigned DeriveCtx::CtxQtCbf(const CompID compID, const bool prevCbf)
{
  if (compID == COMP_Cr)
  {
    return (prevCbf ? 1 : 0);
  }
  return 0;
}

unsigned DeriveCtx::CtxInterDir(const CodingUnit &cu) const
{
  return (MAX_CU_DEPTH - ((floorLog2(cu.lumaSize().width) + floorLog2(cu.lumaSize().height) + 1) >> 1));
}

unsigned DeriveCtx::CtxAffineFlag(const CodingUnit &cu) const
{
  unsigned ctxId = 0;

  const CodingUnit *cuLeft  = cuRestrictedLeft[to_underlying(ChannelType::LUMA)];
  const CodingUnit *cuAbove = cuRestrictedAbove[to_underlying(ChannelType::LUMA)];

  ctxId = (cuLeft && cuLeft->affine) ? 1 : 0;
  ctxId += (cuAbove && cuAbove->affine) ? 1 : 0;

  if (CU::affineCtxInc(cu))
  {
    ctxId += 3;
  }

  return ctxId;
}

unsigned DeriveCtx::CtxBMMrgFlag(const CodingUnit &cu) const
{
  unsigned ctxId = 0;

  const CodingUnit *cuLeft  = cuRestrictedLeft[to_underlying(ChannelType::LUMA)];
  const CodingUnit *cuAbove = cuRestrictedAbove[to_underlying(ChannelType::LUMA)];

  ctxId = (cuLeft && (cuLeft->bmMergeFlag || (!cuLeft->mergeFlag && cuLeft->interDir == 3))) ? 1 : 0;
  ctxId += (cuAbove && (cuAbove->bmMergeFlag || (!cuAbove->mergeFlag && cuAbove->interDir == 3))) ? 1 : 0;

  return ctxId;
}

unsigned DeriveCtx::CtxSkipFlag(const CodingUnit &cu) const
{
  unsigned ctxId = 0;

  const CodingUnit *cuLeft  = cuRestrictedLeft[to_underlying(ChannelType::LUMA)];
  const CodingUnit *cuAbove = cuRestrictedAbove[to_underlying(ChannelType::LUMA)];

  ctxId = (cuLeft && cuLeft->skip) ? 1 : 0;
  ctxId += (cuAbove && cuAbove->skip) ? 1 : 0;

  return ctxId;
}

unsigned DeriveCtx::CtxSgpmFlag(const CodingUnit &cu) const
{
  unsigned ctxId = 0;

  const CodingUnit *cuLeft  = cuRestrictedLeft[to_underlying(ChannelType::LUMA)];
  const CodingUnit *cuAbove = cuRestrictedAbove[to_underlying(ChannelType::LUMA)];

  ctxId = (cuLeft && cuLeft->sgpm) ? 1 : 0;
  ctxId += (cuAbove && cuAbove->sgpm) ? 1 : 0;
  return ctxId;
}

unsigned DeriveCtx::CtxPredModeFlag(const CodingUnit &cu) const
{
  const CodingUnit *cuLeft  = cuRestrictedLeft[to_underlying(ChannelType::LUMA)];
  const CodingUnit *cuAbove = cuRestrictedAbove[to_underlying(ChannelType::LUMA)];

  unsigned ctxId = ((cuAbove && CU::isIntra(*cuAbove)) || (cuLeft && CU::isIntra(*cuLeft))) ? 1 : 0;

  return ctxId;
}

unsigned DeriveCtx::CtxIBCFlag(const CodingUnit &cu) const
{
  unsigned ctxId = 0;

  const CodingUnit *cuLeft  = cuRestrictedLeft[to_underlying(cu.chType)];
  const CodingUnit *cuAbove = cuRestrictedAbove[to_underlying(cu.chType)];

  ctxId += (cuLeft && CU::isIBC(*cuLeft)) ? 1 : 0;
  ctxId += (cuAbove && CU::isIBC(*cuAbove)) ? 1 : 0;

  return ctxId;
}

void MergeCtx::setGeoMmvdMergeInfo(CodingUnit &cu, int mergeIdx, int mmvdIdx) const
{
  bool extMMVD = cu.cs->picHeader->m_gpmMMVDTableFlag;
  CHECK(mergeIdx >= numValidMergeCand, "Merge candidate does not exist");
  CHECK(mmvdIdx >= (extMMVD ? GPM_EXT_MMVD_MAX_REFINE_NUM : GPM_MMVD_MAX_REFINE_NUM), "GPM MMVD index is invalid");
  CHECK(mmvdIdx < 0, "GPM MMVD index is invalid");
  CHECK(!cu.geoFlag || CU::isIBC(cu), "incorrect GPM setting")

  cu.regularMergeFlag = !(cu.ciipFlag || cu.geoFlag);
  cu.mergeFlag        = true;
  cu.mmvdMergeFlag    = false;
  int desiredDir      = interDirNeighbours[mergeIdx];

  if (interDirNeighbours[mergeIdx] == 3)
  {
    if (!cu.cs->slice->m_checkLdc)
    {
      desiredDir = (mergeIdx % 2) + 1;
    }
  }

  cu.interDir  = desiredDir;
  cu.imv       = 0;
  cu.mergeIdx  = mergeIdx;
  cu.mergeType = MergeType::DEFAULT_N;

  constexpr int mvShift           = MV_FRACTIONAL_BITS_DIFF;
  constexpr int refMvdCands[8]    = { 1 << mvShift,  2 << mvShift,  4 << mvShift,  8 << mvShift,
                                      16 << mvShift, 32 << mvShift, 64 << mvShift, 128 << mvShift };
  constexpr int refExtMvdCands[9] = { 1 << mvShift,  2 << mvShift,  4 << mvShift,  8 << mvShift, 12 << mvShift,
                                      16 << mvShift, 24 << mvShift, 32 << mvShift, 64 << mvShift };
  int           fPosStep          = (extMMVD ? (mmvdIdx >> 3) : (mmvdIdx >> 2));
  int           fPosPosition      = (extMMVD ? (mmvdIdx - (fPosStep << 3)) : (mmvdIdx - (fPosStep << 2)));
  int           offset            = (extMMVD ? refExtMvdCands[fPosStep] : refMvdCands[fPosStep]);
  Mv            mvOffset;

  if (fPosPosition == 0)
  {
    mvOffset = Mv(offset, 0);
  }
  else if (fPosPosition == 1)
  {
    mvOffset = Mv(-offset, 0);
  }
  else if (fPosPosition == 2)
  {
    mvOffset = Mv(0, offset);
  }
  else if (fPosPosition == 3)
  {
    mvOffset = Mv(0, -offset);
  }
  else if (fPosPosition == 4)
  {
    mvOffset = Mv(offset, offset);
  }
  else if (fPosPosition == 5)
  {
    mvOffset = Mv(offset, -offset);
  }
  else if (fPosPosition == 6)
  {
    mvOffset = Mv(-offset, offset);
  }
  else if (fPosPosition == 7)
  {
    mvOffset = Mv(-offset, -offset);
  }
  if (desiredDir == 3)
  {
    cu.refIdx[RPL0] = mvFieldNeighbours[mergeIdx][RPL0].refIdx;
    cu.refIdx[RPL1] = mvFieldNeighbours[mergeIdx][RPL1].refIdx;
  }
  else
  {
    int listTarget = desiredDir - 1;
    int listEmpty  = 1 - listTarget;

    cu.refIdx[RefPicList(listTarget)] = mvFieldNeighbours[mergeIdx][listTarget].refIdx;
    cu.refIdx[RefPicList(listEmpty)]  = -1;
  }

  if (cu.refIdx[RPL0] >= 0 && cu.refIdx[RPL1] >= 0)
  {
    Mv tempMv[2];

    const int refListIdx0 = cu.refIdx[RPL0];
    const int refListIdx1 = cu.refIdx[RPL1];

    const int poc0    = cu.cs->slice->getRefPOC(RPL0, refListIdx0);
    const int poc1    = cu.cs->slice->getRefPOC(RPL1, refListIdx1);
    const int currPoc = cu.cs->slice->m_poc;

    tempMv[0] = mvOffset;

    if ((poc0 - currPoc) == (poc1 - currPoc))
    {
      tempMv[1] = tempMv[0];
    }
    else if (abs(poc1 - currPoc) > abs(poc0 - currPoc))
    {
      const int scale = PU::getDistScaleFactor(currPoc, poc0, currPoc, poc1);
      tempMv[1]       = tempMv[0];

      const bool isL0RefLongTerm = cu.cs->slice->getRefPic(RPL0, refListIdx0)->m_longTerm;
      const bool isL1RefLongTerm = cu.cs->slice->getRefPic(RPL1, refListIdx1)->m_longTerm;

      if (isL0RefLongTerm || isL1RefLongTerm)
      {
        if ((poc1 - currPoc) * (poc0 - currPoc) > 0)
        {
          tempMv[0] = tempMv[1];
        }
        else
        {
          tempMv[0].set(-1 * tempMv[1].getHor(), -1 * tempMv[1].getVer());
        }
      }
      else
      {
        tempMv[0] = tempMv[1].getScaledMv(scale);
      }
    }
    else
    {
      const int  scale           = PU::getDistScaleFactor(currPoc, poc1, currPoc, poc0);
      const bool isL0RefLongTerm = cu.cs->slice->getRefPic(RPL0, refListIdx0)->m_longTerm;
      const bool isL1RefLongTerm = cu.cs->slice->getRefPic(RPL1, refListIdx1)->m_longTerm;
      if (isL0RefLongTerm || isL1RefLongTerm)
      {
        if ((poc1 - currPoc) * (poc0 - currPoc) > 0)
        {
          tempMv[1] = tempMv[0];
        }
        else
        {
          tempMv[1].set(-1 * tempMv[0].getHor(), -1 * tempMv[0].getVer());
        }
      }
      else
      {
        tempMv[1] = tempMv[0].getScaledMv(scale);
      }
    }

    cu.mv[RPL0] = mvFieldNeighbours[mergeIdx][RPL0].mv + tempMv[0];
    cu.mv[RPL1] = mvFieldNeighbours[mergeIdx][RPL1].mv + tempMv[1];
  }
  else
  {
    cu.mv[RPL0] = (cu.refIdx[RPL0] >= 0) ? mvFieldNeighbours[mergeIdx][RPL0].mv + mvOffset : Mv();
    cu.mv[RPL1] = (cu.refIdx[RPL1] >= 0) ? mvFieldNeighbours[mergeIdx][RPL1].mv + mvOffset : Mv();
  }

  cu.bdmvrRefine  = false;
  cu.mvd[RPL0]    = Mv();
  cu.mvd[RPL1]    = Mv();
  cu.mvpIdx[RPL0] = NOT_VALID;
  cu.mvpIdx[RPL1] = NOT_VALID;
  cu.mvpNum[RPL0] = NOT_VALID;
  cu.mvpNum[RPL1] = NOT_VALID;
  cu.bcwIdx       = (interDirNeighbours[mergeIdx] == 3) ? bcwIdx[mergeIdx] : BCW_DEFAULT;

  PU::restrictBiPredMergeCandsOne(cu);
  cu.mmvdEncOptMode = 0;
}

void DeriveCtx::setNeighbourCus(const CodingStructure &cs, const UnitArea &ua, const ChannelType ch)
{
  const Position &posLuma     = ua.lumaPos();
  const Position &pos         = isLuma(ch) ? posLuma : ua.chromaPos();
  const uint32_t  curSliceIdx = cs.slice->m_independentSliceIdx;
  const uint32_t  curTileIdx  = cs.pps->getTileIdx(posLuma);

  cuRestrictedLeft[to_underlying(ch)]  = cs.getCURestricted(pos.offset(-1, 0), pos, curSliceIdx, curTileIdx, ch);
  cuRestrictedAbove[to_underlying(ch)] = cs.getCURestricted(pos.offset(0, -1), pos, curSliceIdx, curTileIdx, ch);
}

unsigned DeriveCtx::CtxMipFlag(const CodingUnit &cu) const
{
  unsigned ctxId = 0;

  const CodingUnit *cuLeft  = cuRestrictedLeft[to_underlying(ChannelType::LUMA)];
  const CodingUnit *cuAbove = cuRestrictedAbove[to_underlying(ChannelType::LUMA)];

  ctxId = (cuLeft && cuLeft->mipFlag) ? 1 : 0;
  ctxId += (cuAbove && cuAbove->mipFlag) ? 1 : 0;

  ctxId = (cu.lwidth() > 2 * cu.lheight() || cu.lheight() > 2 * cu.lwidth()) ? 3 : ctxId;

  return ctxId;
}

unsigned DeriveCtx::CtxDimdFlag(const CodingUnit &cu) const
{
  unsigned ctxId = 0;

  const CodingUnit *cuLeft  = cuRestrictedLeft[to_underlying(ChannelType::LUMA)];
  const CodingUnit *cuAbove = cuRestrictedAbove[to_underlying(ChannelType::LUMA)];

  ctxId = (cuLeft && cuLeft->dimdFlag) ? 1 : 0;
  ctxId += (cuAbove && cuAbove->dimdFlag) ? 1 : 0;

  return ctxId;
}

unsigned DeriveCtx::CtxTimdFlag(const CodingUnit &cu) const
{
  unsigned ctxId = 0;

  const CodingUnit *cuLeft  = cuRestrictedLeft[to_underlying(ChannelType::LUMA)];
  const CodingUnit *cuAbove = cuRestrictedAbove[to_underlying(ChannelType::LUMA)];

  ctxId = (cuLeft && cuLeft->timdFlag) ? 1 : 0;
  ctxId += (cuAbove && cuAbove->timdFlag) ? 1 : 0;

  return ctxId;
}

unsigned DeriveCtx::CtxPltCopyFlag(const unsigned prevRunType, const unsigned dist)
{
  uint8_t *ucCtxLut = (prevRunType == PLT_RUN_INDEX) ? g_paletteRunLeftLut : g_paletteRunTopLut;
  if (dist <= RUN_IDX_THRE)
  {
    return ucCtxLut[dist];
  }
  else
  {
    return ucCtxLut[RUN_IDX_THRE];
  }
}
