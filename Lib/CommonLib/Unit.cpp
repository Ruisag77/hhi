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

/** \file     Unit.cpp
 *  \brief    defines unit as a set of blocks and basic unit types (coding, prediction, transform)
 */

#include <array>

#include "Unit.h"

#include "Buffer.h"
#include "Picture.h"
#include "ChromaFormat.h"

#include "UnitTools.h"
#include "UnitPartitioner.h"

#include "ChromaFormat.h"

 // ---------------------------------------------------------------------------
 // block method definitions
 // ---------------------------------------------------------------------------

void CompArea::xRecalcLumaToChroma()
{
  const uint32_t csx = getComponentScaleX(compID, chromaFormat);
  const uint32_t csy = getComponentScaleY(compID, chromaFormat);

  x >>= csx;
  y >>= csy;
  width >>= csx;
  height >>= csy;
}

Position CompArea::chromaPos() const
{
  if (isLuma(compID))
  {
    uint32_t scaleX = getComponentScaleX(compID, chromaFormat);
    uint32_t scaleY = getComponentScaleY(compID, chromaFormat);

    return Position(x >> scaleX, y >> scaleY);
  }
  else
  {
    return *this;
  }
}

Size CompArea::lumaSize() const
{
  if (isChroma(compID))
  {
    uint32_t scaleX = getComponentScaleX(compID, chromaFormat);
    uint32_t scaleY = getComponentScaleY(compID, chromaFormat);

    return Size(width << scaleX, height << scaleY);
  }
  else
  {
    return *this;
  }
}

Size CompArea::chromaSize() const
{
  if (isLuma(compID))
  {
    uint32_t scaleX = getComponentScaleX(compID, chromaFormat);
    uint32_t scaleY = getComponentScaleY(compID, chromaFormat);

    return Size(width >> scaleX, height >> scaleY);
  }
  else
  {
    return *this;
  }
}

Position CompArea::lumaPos() const
{
  if (isChroma(compID))
  {
    uint32_t scaleX = getComponentScaleX(compID, chromaFormat);
    uint32_t scaleY = getComponentScaleY(compID, chromaFormat);

    return Position(x << scaleX, y << scaleY);
  }
  else
  {
    return *this;
  }
}

Position CompArea::compPos(const CompID compID) const { return isLuma(compID) ? lumaPos() : chromaPos(); }

Position CompArea::chanPos(const ChannelType chType) const { return isLuma(chType) ? lumaPos() : chromaPos(); }

// ---------------------------------------------------------------------------
// unit method definitions
// ---------------------------------------------------------------------------

UnitArea::UnitArea(const ChromaFormat _chromaFormat) : chromaFormat(_chromaFormat) {}

UnitArea::UnitArea(const ChromaFormat _chromaFormat, const Area &_area)
  : chromaFormat(_chromaFormat)
  , blocks(getNumberValidComponents(_chromaFormat))
{
  const uint32_t numCh = getNumberValidComponents(chromaFormat);

  for (uint32_t i = 0; i < numCh; i++)
  {
    blocks[i] = CompArea(CompID(i), chromaFormat, _area, true);
  }
}

UnitArea::UnitArea(const ChromaFormat _chromaFormat, const CompArea &blkY)
  : chromaFormat(_chromaFormat)
  , blocks { blkY }
{}

UnitArea::UnitArea(const ChromaFormat _chromaFormat, CompArea &&blkY)
  : chromaFormat(_chromaFormat)
  , blocks { std::forward<CompArea>(blkY) }
{}

UnitArea::UnitArea(const ChromaFormat _chromaFormat, const CompArea &blkY, const CompArea &blkCb, const CompArea &blkCr)
  : chromaFormat(_chromaFormat)
  , blocks { blkY, blkCb, blkCr }
{}

UnitArea::UnitArea(const ChromaFormat _chromaFormat, CompArea &&blkY, CompArea &&blkCb, CompArea &&blkCr)
  : chromaFormat(_chromaFormat)
  , blocks { std::forward<CompArea>(blkY), std::forward<CompArea>(blkCb), std::forward<CompArea>(blkCr) }
{}

bool UnitArea::contains(const UnitArea &other) const
{
  bool ret = true;
  bool any = false;

  for (const auto &blk: other.blocks)
  {
    if (blk.valid() && blocks[blk.compID].valid())
    {
      ret &= blocks[blk.compID].contains(blk);
      any = true;
    }
  }

  return any && ret;
}

bool UnitArea::contains(const UnitArea &other, const ChannelType chType) const
{
  bool ret = true;
  bool any = false;

  for (const auto &blk: other.blocks)
  {
    if (toChannelType(blk.compID) == chType && blk.valid() && blocks[blk.compID].valid())
    {
      ret &= blocks[blk.compID].contains(blk);
      any = true;
    }
  }

  return any && ret;
}

#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS
void UnitArea::resizeTo(const UnitArea &unitArea)
{
  for (uint32_t i = 0; i < blocks.size(); i++)
  {
    blocks[i].resizeTo(unitArea.blocks[i]);
  }
}
#endif

void UnitArea::repositionTo(const UnitArea &unitArea)
{
  for (uint32_t i = 0; i < blocks.size(); i++)
  {
    blocks[i].repositionTo(unitArea.blocks[i]);
  }
}

UnitArea UnitArea::singleChan(const ChannelType chType) const
{
  UnitArea ret(chromaFormat);

  for (const auto &blk: blocks)
  {
    ret.blocks.push_back(toChannelType(blk.compID) == chType ? blk : CompArea());
  }

  return ret;
}

// ---------------------------------------------------------------------------
// coding unit method definitions
// ---------------------------------------------------------------------------

CodingUnit::CodingUnit(const UnitArea &unit)
  : UnitArea(unit)
  , cs(nullptr)
  , slice(nullptr)
  , chType(ChannelType::LUMA)
  , next(nullptr)
  , firstTU(nullptr)
  , lastTU(nullptr)
{
  initData();
}
CodingUnit::CodingUnit(const ChromaFormat _chromaFormat, const Area &_area)
  : UnitArea(_chromaFormat, _area)
  , cs(nullptr)
  , slice(nullptr)
  , chType(ChannelType::LUMA)
  , next(nullptr)
  , firstTU(nullptr)
  , lastTU(nullptr)
{
  initData();
}

CodingUnit &CodingUnit::operator=(const CodingUnit &other)
{
  // CU data
  slice           = other.slice;
  predMode        = other.predMode;
  qtDepth         = other.qtDepth;
  depth           = other.depth;
  btDepth         = other.btDepth;
  mtDepth         = other.mtDepth;
  mtImplicitDepth = other.mtImplicitDepth;
  splitSeries     = other.splitSeries;
  skip            = other.skip;
  mmvdSkip        = other.mmvdSkip;
  affine          = other.affine;
  affineType      = other.affineType;
  geoFlag         = other.geoFlag;
  bdpcmMode[0]    = other.bdpcmMode[0];
  bdpcmMode[1]    = other.bdpcmMode[1];
  qp              = other.qp;
  chromaQpAdj     = other.chromaQpAdj;
  rootCbf         = other.rootCbf;
  sbtInfo         = other.sbtInfo;
  tileIdx         = other.tileIdx;
  imv             = other.imv;
  bcwIdx          = other.bcwIdx;
  for (int i = 0; i < 2; i++)
  {
    refIdxBi[i] = other.refIdxBi[i];
  }
  derivedIpm[0]    = other.derivedIpm[0];
  derivedIpm[1]    = other.derivedIpm[1];
  smvdMode         = other.smvdMode;
  mipFlag          = other.mipFlag;
  dimdFlag         = other.dimdFlag;
  dimdData         = other.dimdData;
  dimdChromaFlag   = other.dimdChromaFlag;
  inferredDimdMode = other.inferredDimdMode;
  timdFlag         = other.timdFlag;
  timdData         = other.timdData;
  timdSadFlag      = other.timdSadFlag;
  timdSadData      = other.timdSadData;
  obicAvailFlag    = other.obicAvailFlag;
  obicFlag         = other.obicFlag;
  obicData         = other.obicData;
  obicNeighbours   = other.obicNeighbours;
  eipFlag          = other.eipFlag;
  eipMerge         = other.eipMerge;
  eipModels        = other.eipModels;
  eipMultiModel    = other.eipMultiModel;
  mpmFlag          = other.mpmFlag;
  secondMpmFlag    = other.secondMpmFlag;
  lumaModeIdx      = other.lumaModeIdx;
  chromaModeIdx    = other.chromaModeIdx;
  plDir            = other.plDir;

  for (int idx = 0; idx < MAX_NUM_CHANNEL_TYPE; idx++)
  {
    curPLTSize[idx]   = other.curPLTSize[idx];
    useEscape[idx]    = other.useEscape[idx];
    useRotation[idx]  = other.useRotation[idx];
    reusePLTSize[idx] = other.reusePLTSize[idx];
    lastPLTSize[idx]  = other.lastPLTSize[idx];
    if (slice->m_sps->m_PLTMode)
    {
      std::copy_n(other.reuseflag[idx], MAXPLTPREDSIZE, reuseflag[idx]);
    }
  }

  if (slice->m_sps->m_PLTMode)
  {
    for (int idx = 0; idx < MAX_NUM_COMP; idx++)
    {
      std::copy_n(other.curPLT[idx], MAXPLTSIZE, curPLT[idx]);
    }
  }

  // PU data
  intraDir          = other.intraDir;
  mipTransposedFlag = other.mipTransposedFlag;
  multiRefIdx       = other.multiRefIdx;
  cccmFlag          = other.cccmFlag;
  cccmType          = other.cccmType;
  ccModels          = other.ccModels;
  idxNonLocalCCP    = other.idxNonLocalCCP;
  ccMergeFusionIdx  = other.ccMergeFusionIdx;
  decDerivedCcpMode = other.decDerivedCcpMode;
  ccFilterFlag      = other.ccFilterFlag;
  plDir             = other.plDir;
  cclmOffsets       = other.cclmOffsets;
  numBvgCands       = other.numBvgCands;
  for (int candIdx = 0; candIdx < NUM_BVG_CCCM_CANDS; candIdx++)
  {
    bvList[candIdx] = other.bvList[candIdx];
  }
  mergeFlag        = other.mergeFlag;
  regularMergeFlag = other.regularMergeFlag;
  mergeIdx         = other.mergeIdx;
  geoSplitDir      = other.geoSplitDir;
  geoMergeIdx      = other.geoMergeIdx;
  geoMMVDFlag      = other.geoMMVDFlag;
  geoMMVDIdx       = other.geoMMVDIdx;
  gpmIntraFlag     = other.gpmIntraFlag;
  geoBldIdx        = other.geoBldIdx;
  mmvdMergeFlag    = other.mmvdMergeFlag;
  mmvdMergeIdx     = other.mmvdMergeIdx;
  bmMergeFlag      = other.bmMergeFlag;
  bmDir            = other.bmDir;

  interDir  = other.interDir;
  mergeType = other.mergeType;
  bv        = other.bv;
  bvd       = other.bvd;
  ciipFlag  = other.ciipFlag;
  ciipMode  = other.ciipMode;
  obmcFlag  = other.obmcFlag;

  bdmvrRefine = other.bdmvrRefine;

  for (uint32_t i = 0; i < NUM_RPL01; i++)
  {
    mvpIdx[i] = other.mvpIdx[i];
    mvpNum[i] = other.mvpNum[i];
    mv[i]     = other.mv[i];
    mvd[i]    = other.mvd[i];
    refIdx[i] = other.refIdx[i];
    for (uint32_t j = 0; j < 3; j++)
    {
      mvdAffi[i][j] = other.mvdAffi[i][j];
    }
    for (uint32_t j = 0; j < 3; j++)
    {
      mvAffi[i][j] = other.mvAffi[i][j];
    }
  }

  gpmDirMode   = other.gpmDirMode;
  timdHor      = other.timdHor;
  timdVer      = other.timdVer;
  sgpm         = other.sgpm;
  sgpmIdx      = other.sgpmIdx;
  sgpmSplitDir = other.sgpmSplitDir;
  sgpmMode0    = other.sgpmMode0;
  sgpmMode1    = other.sgpmMode1;

  licFlag           = other.licFlag;
  licScaleAndOffset = other.licScaleAndOffset;
  oppositeLicFlag   = other.oppositeLicFlag;

  return *this;
}

void CodingUnit::initCuData()
{
  predMode        = NUMBER_OF_PREDICTION_MODES;
  qtDepth         = 0;
  depth           = 0;
  btDepth         = 0;
  mtDepth         = 0;
  mtImplicitDepth = 0;
  splitSeries     = 0;
  skip            = false;
  mmvdSkip        = false;
  affine          = false;
  affineType      = AffineModel::_4_PARAMS;
  geoFlag         = false;
  bdpcmMode[0]    = BdpcmMode::NONE;
  bdpcmMode[1]    = BdpcmMode::NONE;
  qp              = 0;
  chromaQpAdj     = 0;
  rootCbf         = true;
  sbtInfo         = 0;
  tileIdx         = 0;
  imv             = 0;
  bcwIdx          = BCW_DEFAULT;
  for (int i = 0; i < 2; i++)
  {
    refIdxBi[i] = -1;
  }
  derivedIpm[0]  = -1;
  derivedIpm[1]  = -1;
  smvdMode       = 0;
  mipFlag        = false;
  dimdFlag       = false;
  dimdData       = {};
  dimdChromaFlag = false;
  timdFlag       = false;
  timdData       = {};
  timdSadFlag    = false;
  timdSadData    = {};
  obicAvailFlag  = false;
  obicFlag       = false;
  obicData       = {};
  obicNeighbours = {};
  mpmFlag        = false;
  secondMpmFlag  = false;
  lumaModeIdx    = -1;
  chromaModeIdx  = -1;
  plDir          = PlanarDirType::NO_DIR;

  for (int idx = 0; idx < MAX_NUM_CHANNEL_TYPE; idx++)
  {
    curPLTSize[idx]   = 0;
    reusePLTSize[idx] = 0;
    lastPLTSize[idx]  = 0;
    useEscape[idx]    = false;
    useRotation[idx]  = false;
    std::fill_n(reuseflag[idx], MAXPLTPREDSIZE, false);
  }

  for (int idx = 0; idx < MAX_NUM_COMP; idx++)
  {
    std::fill_n(curPLT[idx], MAXPLTSIZE, 0);
  }

  timdHor         = -1;
  timdVer         = -1;
  sgpm            = false;
  sgpmIdx         = -1;
  sgpmSplitDir    = -1;
  sgpmMode0       = -1;
  sgpmMode1       = -1;
  oppositeLicFlag = false;
}

const uint8_t CodingUnit::checkAllowedSbt() const
{
  if (!slice->m_sps->m_useSBT)
  {
    return 0;
  }

  // check on prediction mode
  if (predMode == MODE_INTRA || predMode == MODE_IBC || predMode == MODE_PLT)   // intra, palette or IBC
  {
    return 0;
  }
  if (ciipFlag)
  {
    return 0;
  }
  if (gpmIntraFlag)
  {
    return 0;
  }

  const int cuWidth  = lwidth();
  const int cuHeight = lheight();

  // parameter
  const int maxSbtCUSize = cs->sps->getMaxTbSize();
  const int minSbtCUSize = 1 << (MIN_CU_LOG2 + 1);

  // check on size
  if (cuWidth > maxSbtCUSize || cuHeight > maxSbtCUSize)
  {
    return 0;
  }

  std::array<bool, NUMBER_SBT_IDX> allowType;

  allowType.fill(false);
  allowType[SBT_VER_HALF] = cuWidth >= minSbtCUSize;
  allowType[SBT_HOR_HALF] = cuHeight >= minSbtCUSize;
  allowType[SBT_VER_QUAD] = cuWidth >= 2 * minSbtCUSize;
  allowType[SBT_HOR_QUAD] = cuHeight >= 2 * minSbtCUSize;
  allowType[SBT_QUAD]     = cuWidth >= minSbtCUSize && cuHeight >= minSbtCUSize && cuWidth >= SBT_QUAD_MIN_BLOCK_SIZE &&
    cuHeight >= SBT_QUAD_MIN_BLOCK_SIZE;
  allowType[SBT_QUARTER] = cuWidth >= minSbtCUSize && cuHeight >= minSbtCUSize &&
    cuWidth >= SBT_QUARTER_MIN_BLOCK_SIZE && cuHeight >= SBT_QUARTER_MIN_BLOCK_SIZE;

  uint8_t sbtAllowed = 0;

  for (int i = 0; i < allowType.size(); i++)
  {
    sbtAllowed += (uint8_t)allowType[i] << i;
  }

  return sbtAllowed;
}

uint8_t CodingUnit::getSbtTuSplit() const
{
  uint8_t sbtTuSplitType = 0;

  switch (getSbtIdx())
  {
  case SBT_VER_HALF:
    sbtTuSplitType = (getSbtPos() == SBT_POS0 ? 0 : 1) + SBT_VER_HALF_POS0_SPLIT;
    break;
  case SBT_HOR_HALF:
    sbtTuSplitType = (getSbtPos() == SBT_POS0 ? 0 : 1) + SBT_HOR_HALF_POS0_SPLIT;
    break;
  case SBT_VER_QUAD:
    sbtTuSplitType = (getSbtPos() == SBT_POS0 ? 0 : 1) + SBT_VER_QUAD_POS0_SPLIT;
    break;
  case SBT_HOR_QUAD:
    sbtTuSplitType = (getSbtPos() == SBT_POS0 ? 0 : 1) + SBT_HOR_QUAD_POS0_SPLIT;
    break;
  case SBT_QUAD:
    sbtTuSplitType = getSbtPos() + SBT_QUAD_POSTL_SPLIT;
    break;
  case SBT_QUARTER:
    sbtTuSplitType = getSbtPos() + SBT_QUARTER_POSTL_SPLIT;
    break;
  default:
    assert(0);
    break;
  }

  assert(sbtTuSplitType >= SBT_VER_HALF_POS0_SPLIT && sbtTuSplitType < NUM_PART_SPLIT);
  return sbtTuSplitType;
}

int CodingUnit::getSbtTuIdx() const
{
  int idx = 0;

  if (sbtInfo)
  {
    const int sbtIdx = getSbtIdx();
    const int sbtPos = getSbtPos();

    if (sbtIdx == SBT_QUARTER)
    {
      if (sbtPos == 0)
      {
        idx = 0;
      }
      else if (sbtPos == 1)
      {
        idx = 3;
      }
      else if (sbtPos == 2)
      {
        idx = 12;
      }
      else if (sbtPos == 3)
      {
        idx = 15;
      }
      else
      {
        CHECKD(true, "Wrong SBT position");
      }
    }
    else
    {
      idx = sbtPos;
    }
  }

  return idx;
}

// ---------------------------------------------------------------------------
// prediction unit method definitions
// ---------------------------------------------------------------------------

void CodingUnit::initPuData()
{
  // intra data - need this default initialization for PCM

  intraDir[ChannelType::LUMA]   = DC_IDX;
  intraDir[ChannelType::CHROMA] = PLANAR_IDX;
  mipTransposedFlag             = false;
  multiRefIdx                   = 0;
  cccmFlag                      = 0;
  cccmType                      = ConvModelType::CONV_MODEL_UNDEFINED;
  plDir                         = PlanarDirType::NO_DIR;
  cclmOffsets                   = {};
  ccModels                      = CrossCompModels();
  idxNonLocalCCP                = 0;
  ccMergeFusionIdx              = 0;
  decDerivedCcpMode             = false;
  ccFilterFlag                  = false;
  numBvgCands                   = 0;
  eipFlag                       = false;
  eipMerge                      = false;
  eipMultiModel                 = false;
  eipModels                     = EipModels();
  inferredDimdMode              = 0;
  for (int candIdx = 0; candIdx < NUM_BVG_CCCM_CANDS; candIdx++)
  {
    bvList[candIdx] = Mv(0, 0);
  }

  // inter data
  mergeFlag        = false;
  regularMergeFlag = false;
  mergeIdx         = MAX_UCHAR;
  geoSplitDir      = MAX_UCHAR;
  geoMergeIdx.fill(MAX_UCHAR);
  geoMMVDFlag.fill(false);
  geoMMVDIdx.fill(MAX_UCHAR);
  gpmIntraFlag     = false;
  geoBldIdx        = MAX_UCHAR;
  mmvdMergeFlag    = false;
  mmvdMergeIdx.val = MmvdIdx::INVALID;
  bmMergeFlag      = false;
  bmDir            = 0;
  interDir         = MAX_UCHAR;
  mergeType        = MergeType::DEFAULT_N;
  bv.setZero();
  bvd.setZero();
  bdmvrRefine = false;
  for (uint32_t i = 0; i < NUM_RPL01; i++)
  {
    mvpIdx[i] = MAX_UCHAR;
    mvpNum[i] = MAX_UCHAR;
    refIdx[i] = -1;
    mv[i].setZero();
    mvd[i].setZero();
    for (uint32_t j = 0; j < 3; j++)
    {
      mvdAffi[i][j].setZero();
    }
    for (uint32_t j = 0; j < 3; j++)
    {
      mvAffi[i][j].setZero();
    }
  }
  ciipFlag       = false;
  ciipMode       = CIIP_Type::NORMAL;
  obmcFlag       = false;
  mmvdEncOptMode = 0;

  gpmDirMode = 0;

  licFlag           = false;
  licScaleAndOffset = LicScaleAndOffset();
  oppositeLicFlag   = false;
}

CodingUnit &CodingUnit::operator=(const IntraPredictionData &predData)
{
  intraDir          = predData.intraDir;
  mipTransposedFlag = predData.mipTransposedFlag;
  multiRefIdx       = predData.multiRefIdx;
  cccmFlag          = predData.cccmFlag;
  cccmType          = predData.cccmType;
  cclmOffsets       = predData.cclmOffsets;
  ccModels          = predData.ccModels;
  idxNonLocalCCP    = predData.idxNonLocalCCP;
  ccMergeFusionIdx  = predData.ccMergeFusionIdx;
  decDerivedCcpMode = predData.decDerivedCcpMode;
  ccFilterFlag      = predData.ccFilterFlag;
  numBvgCands       = predData.numBvgCands;
  for (int candIdx = 0; candIdx < NUM_BVG_CCCM_CANDS; candIdx++)
  {
    bvList[candIdx] = predData.bvList[candIdx];
  }

  return *this;
}

CodingUnit &CodingUnit::operator=(const InterPredictionData &predData)
{
  mergeFlag        = predData.mergeFlag;
  regularMergeFlag = predData.regularMergeFlag;
  mergeIdx         = predData.mergeIdx;
  geoSplitDir      = predData.geoSplitDir;
  geoMergeIdx      = predData.geoMergeIdx;
  geoMMVDFlag      = predData.geoMMVDFlag;
  geoMMVDIdx       = predData.geoMMVDIdx;
  gpmIntraFlag     = predData.gpmIntraFlag;
  geoBldIdx        = predData.geoBldIdx;
  bmMergeFlag      = predData.bmMergeFlag;
  bmDir            = predData.bmDir;

  mmvdMergeFlag = predData.mmvdMergeFlag;
  mmvdMergeIdx  = predData.mmvdMergeIdx;
  interDir      = predData.interDir;
  mergeType     = predData.mergeType;
  bv            = predData.bv;
  bvd           = predData.bvd;
  ciipFlag      = predData.ciipFlag;
  ciipMode      = predData.ciipMode;
  obmcFlag      = predData.obmcFlag;
  bdmvrRefine   = predData.bdmvrRefine;

  for (uint32_t i = 0; i < NUM_RPL01; i++)
  {
    mvpIdx[i] = predData.mvpIdx[i];
    mvpNum[i] = predData.mvpNum[i];
    mv[i]     = predData.mv[i];
    mvd[i]    = predData.mvd[i];
    refIdx[i] = predData.refIdx[i];
    for (uint32_t j = 0; j < 3; j++)
    {
      mvdAffi[i][j] = predData.mvdAffi[i][j];
    }
    for (uint32_t j = 0; j < 3; j++)
    {
      mvAffi[i][j] = predData.mvAffi[i][j];
    }
  }

  gpmDirMode = predData.gpmDirMode;

  licFlag           = predData.licFlag;
  licScaleAndOffset = predData.licScaleAndOffset;
  oppositeLicFlag   = predData.oppositeLicFlag;

  return *this;
}

CodingUnit &CodingUnit::operator=(const MotionInfo &mi)
{
  interDir = mi.interDir;

  for (uint32_t i = 0; i < NUM_RPL01; i++)
  {
    refIdx[i] = mi.refIdx[i];
    mv[i]     = mi.mv[i];
  }

  return *this;
}

void CodingUnit::getAffineMotionInfo(AffineMotionInfo affineMiOut[2], int refIdxOut[2]) const
{

  for (int list = 0; list < 2; list++)
  {
    RefPicList eRefList                             = (list == 0) ? RPL0 : RPL1;
    refIdxOut[list]                                 = NOT_VALID;
    affineMiOut[list].oneSetAffineParametersPattern = 0;

    if ((interDir == 1 && list == 1) || (interDir == 2 && list == 0))
    {
      continue;
    }
    refIdxOut[list] = refIdx[list];

    int affpara[4];
    PU::deriveAffineParametersFromMVs(*this, mvAffi[eRefList], affpara, this->affineType);
    PU::storeAffParas(affpara);

    affineMiOut[list].oneSetAffineParameters[0] = (short)(affpara[0]);
    affineMiOut[list].oneSetAffineParameters[1] = (short)(affpara[1]);
    affineMiOut[list].oneSetAffineParameters[2] = (short)(affpara[2]);
    affineMiOut[list].oneSetAffineParameters[3] = (short)(affpara[3]);
  }
}

const MotionInfo &CodingUnit::getMotionInfo() const { return cs->getMotionInfo(lumaPos()); }

const MotionInfo &CodingUnit::getMotionInfo(const Position &pos) const
{
  CHECKD(!Y().contains(pos), "Trying to access motion info outsied of PU");
  return cs->getMotionInfo(pos);
}

MotionBuf CodingUnit::getMotionBuf() { return cs->getMotionBuf(*this); }

CMotionBuf CodingUnit::getMotionBuf() const { return cs->getMotionBuf(*this); }

bool CodingUnit::hasSameLicParameters(const CodingUnit &other) const
{
  if (this == &other) // same CU
  {
    return true;
  }

  const bool ourEffectiveLicFlag    = licFlag && !ciipFlag;
  const bool othersEffectiveLicFlag = other.licFlag && !other.ciipFlag;
  if (ourEffectiveLicFlag != othersEffectiveLicFlag)
  {
    return false;
  }

  if (!ourEffectiveLicFlag)
  {
    return true;
  }

  CHECK(refIdx[0] != other.refIdx[0], "refIdx[0] should already have been checked");
  CHECK(refIdx[1] != other.refIdx[1], "refIdx[1] should already have been checked");
  CHECK(interDir != other.interDir, "interDir should already have been checked");

  CHECK(interDir != (((refIdx[1] >= 0) << 1) | (refIdx[0] >= 0)), "interDir/refIdx inconsistency");
  CHECK(other.interDir != (((other.refIdx[1] >= 0) << 1) | (other.refIdx[0] >= 0)),
        "other.interDir/other.refIdx inconsistency");

  for (const int ref: { 0, 1 })
  {
    if (refIdx[ref] >= 0)
    {
      for (const int comp: { 0, 1, 2 })
      {
        if (licScaleAndOffset.scale[ref][comp] != other.licScaleAndOffset.scale[ref][comp] ||
            licScaleAndOffset.offset[ref][comp] != other.licScaleAndOffset.offset[ref][comp])
        {
          return false;
        }
      }
    }
  }

  return true;
}

// ---------------------------------------------------------------------------
// transform unit method definitions
// ---------------------------------------------------------------------------

TransformUnit::TransformUnit(const UnitArea &unit)
  : UnitArea(unit)
  , cu(nullptr)
  , cs(nullptr)
  , chType(ChannelType::LUMA)
  , next(nullptr)
{
  for (unsigned i = 0; i < MAX_NUM_TBLOCKS; i++)
  {
    m_coeffs[i]        = nullptr;
    m_signsPredArea[i] = nullptr;
  }

  m_pltIdx.fill(nullptr);
  m_runType.fill(nullptr);

  initData();
}

TransformUnit::TransformUnit(const ChromaFormat _chromaFormat, const Area &_area)
  : UnitArea(_chromaFormat, _area)
  , cu(nullptr)
  , cs(nullptr)
  , chType(ChannelType::LUMA)
  , next(nullptr)
{
  for (unsigned i = 0; i < MAX_NUM_TBLOCKS; i++)
  {
    m_coeffs[i]        = nullptr;
    m_signsPredArea[i] = nullptr;
  }

  m_pltIdx.fill(nullptr);
  m_runType.fill(nullptr);

  initData();
}

void TransformUnit::initData()
{
  for (unsigned i = 0; i < MAX_NUM_TBLOCKS; i++)
  {
    cbf[i]              = 0;
    mtsIdx[i]           = MtsType::DCT2_DCT2;
    lastPos[i]          = -1;
    numPredAreaSigns[i] = 0;
  }
  depth                 = 0;
  noResidual            = false;
  jointCbCr             = 0;
  m_chromaResScaleInv   = 0;
  derivedIntraDirsLuma  = std::make_pair<int8_t, int8_t>(0, 1);
  derivedIntraDirChroma = int8_t { 0 };
}

void TransformUnit::init(TCoeff **coeffs, uint8_t **signsPredArea, EnumArray<Pel *, ChannelType> &pltIdx,
                         EnumArray<bool *, ChannelType> &runType)
{
  uint32_t numBlocks = getNumberValidTBlocks(*cs->pcv);

  for (uint32_t i = 0; i < numBlocks; i++)
  {
    m_coeffs[i]        = coeffs[i];
    m_signsPredArea[i] = signsPredArea[i];
  }

  for (auto chType = ChannelType::LUMA; chType <= ::getLastChannel(cs->pcv->chrFormat); chType++)
  {
    m_pltIdx[chType]  = pltIdx[chType];
    m_runType[chType] = runType[chType];
  }
}

TransformUnit &TransformUnit::operator=(const TransformUnit &other)
{
  CHECK(chromaFormat != other.chromaFormat, "Incompatible formats");

  const int numBlocks = ::getNumberValidTBlocks(*cs->pcv);

  for (int i = 0; i < numBlocks; i++)
  {
    CHECKD(blocks[i].area() != other.blocks[i].area(), "Transformation units cover different areas");

    const uint32_t area = blocks[i].area();

    if (m_coeffs[i] && other.m_coeffs[i] && m_coeffs[i] != other.m_coeffs[i])
    {
      std::copy_n(other.m_coeffs[i], area, m_coeffs[i]);
    }
    if (m_signsPredArea[i] && other.m_signsPredArea[i] && m_signsPredArea[i] != other.m_signsPredArea[i])
    {
      std::copy_n(other.m_signsPredArea[i], area, m_signsPredArea[i]);
    }

    cbf[i]              = other.cbf[i];
    mtsIdx[i]           = other.mtsIdx[i];
    lastPos[i]          = other.lastPos[i];
    numPredAreaSigns[i] = other.numPredAreaSigns[i];
  }

  if (cu->slice->m_sps->m_PLTMode)
  {
    for (auto chType = ChannelType::LUMA; chType <= ::getLastChannel(cs->pcv->chrFormat); chType++)
    {
      if (m_pltIdx[chType] != nullptr && other.m_pltIdx[chType] != nullptr &&
          m_pltIdx[chType] != other.m_pltIdx[chType])
      {
        const uint32_t area = block(chType).area();
        std::copy_n(other.m_pltIdx[chType], area, m_pltIdx[chType]);
      }
      if (m_runType[chType] != nullptr && other.m_runType[chType] != nullptr &&
          m_runType[chType] != other.m_runType[chType])
      {
        const uint32_t area = block(chType).area();
        std::copy_n(other.m_runType[chType], area, m_runType[chType]);
      }
    }
  }

  depth                 = other.depth;
  noResidual            = other.noResidual;
  jointCbCr             = other.jointCbCr;
  derivedIntraDirsLuma  = other.derivedIntraDirsLuma;
  derivedIntraDirChroma = other.derivedIntraDirChroma;

  return *this;
}

void TransformUnit::copyComponentFrom(const TransformUnit &other, const CompID i)
{
  CHECK(chromaFormat != other.chromaFormat, "Incompatible formats");
  CHECKD(blocks[i].area() != other.blocks[i].area(), "Transformation units cover different areas");

  const uint32_t area = blocks[i].area();

  if (m_coeffs[i] && other.m_coeffs[i] && m_coeffs[i] != other.m_coeffs[i])
  {
    std::copy_n(other.m_coeffs[i], area, m_coeffs[i]);
  }
  if (m_signsPredArea[i] && other.m_signsPredArea[i] && m_signsPredArea[i] != other.m_signsPredArea[i])
  {
    std::copy_n(other.m_signsPredArea[i], area, m_signsPredArea[i]);
  }

  const ChannelType chType = toChannelType(i);
  if (i == getFirstComponentOfChannel(chType))
  {
    if (m_pltIdx[chType] != nullptr && other.m_pltIdx[chType] != nullptr && m_pltIdx[chType] != other.m_pltIdx[chType])
    {
      std::copy_n(other.m_pltIdx[chType], area, m_pltIdx[chType]);
    }
    if (m_runType[chType] != nullptr && other.m_runType[chType] != nullptr &&
        m_runType[chType] != other.m_runType[chType])
    {
      std::copy_n(other.m_runType[chType], area, m_runType[chType]);
    }
  }

  cbf[i]                = other.cbf[i];
  mtsIdx[i]             = other.mtsIdx[i];
  lastPos[i]            = other.lastPos[i];
  numPredAreaSigns[i]   = other.numPredAreaSigns[i];
  depth                 = other.depth;
  noResidual            = other.noResidual;
  jointCbCr             = isChroma(i) ? other.jointCbCr : jointCbCr;
  derivedIntraDirsLuma  = isLuma(i) ? other.derivedIntraDirsLuma : derivedIntraDirsLuma;
  derivedIntraDirChroma = isChroma(i) ? other.derivedIntraDirChroma : derivedIntraDirChroma;
}

CoeffBuf        TransformUnit::getCoeffs(const CompID id) { return CoeffBuf(m_coeffs[id], blocks[id]); }
const CCoeffBuf TransformUnit::getCoeffs(const CompID id) const { return CCoeffBuf(m_coeffs[id], blocks[id]); }

PelBuf        TransformUnit::getcurPLTIdx(const ChannelType id) { return PelBuf(m_pltIdx[id], block(id)); }
const CPelBuf TransformUnit::getcurPLTIdx(const ChannelType id) const { return CPelBuf(m_pltIdx[id], block(id)); }

PLTtypeBuf        TransformUnit::getrunType(const ChannelType id) { return PLTtypeBuf(m_runType[id], block(id)); }
const CPLTtypeBuf TransformUnit::getrunType(const ChannelType id) const
{
  return CPLTtypeBuf(m_runType[id], block(id));
}

PLTescapeBuf        TransformUnit::getescapeValue(const CompID id) { return PLTescapeBuf(m_coeffs[id], blocks[id]); }
const CPLTescapeBuf TransformUnit::getescapeValue(const CompID id) const
{
  return CPLTescapeBuf(m_coeffs[id], blocks[id]);
}

Pel  *TransformUnit::getPLTIndex(const ChannelType id) { return m_pltIdx[id]; }
bool *TransformUnit::getRunTypes(const ChannelType id) { return m_runType[id]; }

int TransformUnit::countNonZero()
{
  if (!cbf[0])
  {
    return 0;
  }
  int  count   = 0;
  auto areaY   = Y();
  auto coeffY  = m_coeffs[0];
  int  blksize = areaY.width * areaY.height;
  for (auto i = 0; i < blksize; ++i)
  {
    if (coeffY[i])
    {
      count++;
    }
  }
  return count;
}

void TransformUnit::checkTuNoResidual(unsigned idx)
{
  if (CU::getSbtIdx(cu->sbtInfo) == SBT_OFF_DCT)
  {
    return;
  }

  if ((CU::getSbtPos(cu->sbtInfo) == SBT_POS0 && idx == 1) || (CU::getSbtPos(cu->sbtInfo) == SBT_POS1 && idx == 0))
  {
    noResidual = true;
  }
  else if (CU::getSbtIdx(cu->sbtInfo) == SBT_QUAD)
  {
    noResidual = (CU::getSbtPos(cu->sbtInfo) != idx);
  }
  else if (CU::getSbtIdx(cu->sbtInfo) == SBT_QUARTER)
  {
    if ((CU::getSbtPos(cu->sbtInfo) == 0 && idx == 0) || (CU::getSbtPos(cu->sbtInfo) == 1 && idx == 3) ||
        (CU::getSbtPos(cu->sbtInfo) == 2 && idx == 12) || (CU::getSbtPos(cu->sbtInfo) == 3 && idx == 15))
    {
      noResidual = false;
    }
    else
    {
      noResidual = true;
    }
  }
}

int TransformUnit::getTbAreaAfterCoefZeroOut(CompID compID) const
{
  int tbZeroOutWidth  = blocks[compID].width;
  int tbZeroOutHeight = blocks[compID].height;
  int tbArea          = tbZeroOutWidth * tbZeroOutHeight;

  tbZeroOutWidth  = getNonzeroTuSize(tbZeroOutWidth);
  tbZeroOutHeight = getNonzeroTuSize(tbZeroOutHeight);
  tbArea          = tbZeroOutWidth * tbZeroOutHeight;
  return tbArea;
}

int  TransformUnit::getChromaAdj() const { return m_chromaResScaleInv; }
void TransformUnit::setChromaAdj(int i) { m_chromaResScaleInv = i; }

bool TransformUnit::checkNSTApplied(CompID compID) const
{
  bool lfnstApplied = isNST(mtsIdx[compID]) && (CS::isDualITree(*this->cs) ? true : isLuma(compID));
  return lfnstApplied;
}

int TransformUnit::maxNumPredSigns(CompID compID) const
{
  return checkNSTApplied(compID) ? std::min<int>(SIGN_PRED_MAX_NUM_NST, cs->sps->m_numPredSign)
                                 : cs->sps->m_numPredSign;
}
