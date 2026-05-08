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

/** \file     CodingStructure.h
 *  \brief    A class managing the coding information for a specific image part
 */

#include "CodingStructure.h"

#include "Unit.h"
#include "Slice.h"
#include "Picture.h"
#include "UnitTools.h"
#include "UnitPartitioner.h"

XuPool g_xuPool = XuPool();

// ---------------------------------------------------------------------------
// coding structure method definitions
// ---------------------------------------------------------------------------

CodingStructure::CodingStructure(XuPool &xuPool)
  : area()
  , picture(nullptr)
  , parent(nullptr)
  , m_isTuEnc(false)
  , m_cuPool(xuPool.cuPool)
  , m_tuPool(xuPool.tuPool)
  , resetIBCBuffer(true)
{
  for (uint32_t i = 0; i < MAX_NUM_COMP; i++)
  {
    m_coeffs[i]        = nullptr;
    m_signsPredArea[i] = nullptr;
    m_offsets[i]       = 0;
  }

  m_pltIdx.fill(nullptr);
  m_runType.fill(nullptr);
  m_cuArr.fill(nullptr);
  m_isDecomp.fill(nullptr);

  m_motionBuf      = nullptr;
  picHeader        = nullptr;
  m_currQtDepthBuf = nullptr;

  etmType = -1;
  etmOpts = -1;

  m_eipIdxBuf = nullptr;
  m_eipModelLUT.clear();
}

void CodingStructure::destroy()
{
  picture = nullptr;
  parent  = nullptr;

  m_pred.destroy();
  m_resi.destroy();
  m_reco.destroy();
  m_orgr.destroy();
#if ENABLE_NNLF
  m_predCustom.destroy();
#endif

  destroyCoeffs();

  for (auto &ptr: m_isDecomp)
  {
    delete[] ptr;
    ptr = nullptr;
  }

  for (auto &ptr: m_cuArr)
  {
    delete[] ptr;
    ptr = nullptr;
  }

  delete[] m_motionBuf;
  m_motionBuf = nullptr;

  delete[] m_currQtDepthBuf;
  m_currQtDepthBuf = nullptr;

  delete[] m_eipIdxBuf;
  m_eipIdxBuf = nullptr;
  m_eipModelLUT.clear();

  m_tuPool.giveBack(tus);
  m_cuPool.giveBack(cus);
}

void CodingStructure::releaseIntermediateData()
{
  clearTUs();
  clearCUs();
}

bool CodingStructure::isDecomp(const Position &pos, const ChannelType effChType)
{
  const CompArea &_blk = area.block(effChType);

  if (_blk.contains(pos))
  {
    return m_isDecomp[effChType][rsAddr(pos, _blk, _blk.width, unitScale[getFirstComponentOfChannel(effChType)])];
  }
  else if (parent)
  {
    return parent->isDecomp(pos, effChType);
  }
  else
  {
    return false;
  }
}

bool CodingStructure::isDecomp(const Position &pos, const ChannelType effChType) const
{
  const CompArea &_blk = area.block(effChType);

  if (_blk.contains(pos))
  {
    return m_isDecomp[effChType][rsAddr(pos, _blk, _blk.width, unitScale[getFirstComponentOfChannel(effChType)])];
  }
  else if (parent)
  {
    return parent->isDecomp(pos, effChType);
  }
  else
  {
    return false;
  }
}

void CodingStructure::setDecomp(const CompArea &_area, const bool _isCoded /*= true*/)
{
  const UnitScale &scale = unitScale[_area.compID];

  AreaBuf<bool> isCodedBlk(m_isDecomp[toChannelType(_area.compID)] +
                             rsAddr(_area, area.blocks[_area.compID].pos(), area.blocks[_area.compID].width, scale),
                           area.blocks[_area.compID].width >> scale.posx, _area.width >> scale.posx,
                           _area.height >> scale.posy);
  isCodedBlk.fill(_isCoded);
}

void CodingStructure::setDecomp(const UnitArea &_area, const bool _isCoded /*= true*/)
{
  for (uint32_t i = 0; i < _area.blocks.size(); i++)
  {
    if (_area.blocks[i].valid())
    {
      setDecomp(_area.blocks[i], _isCoded);
    }
  }
}

CodingUnit *CodingStructure::getLumaCU(const Position &pos)
{
  const CompArea &_blk = area.block(ChannelType::LUMA);
  CHECK(!_blk.contains(pos), "must contain the pos");

  return m_cuArr[ChannelType::LUMA][rsAddr(pos, _blk.pos(), _blk.width, unitScale[COMP_Y])];
}

CodingUnit *CodingStructure::getCU(const Position &pos, const ChannelType effChType)
{
  const CodingStructure *cs = this;

  while (cs && !cs->area.block(effChType).contains(pos))
  {
    cs = cs->parent;
  }

  if (!cs)
  {
    return nullptr;
  }

  const CompArea &_blk = cs->area.block(effChType);

  return cs->m_cuArr[effChType][rsAddr(pos, _blk.pos(), _blk.width, unitScale[getFirstComponentOfChannel(effChType)])];
}

const CodingUnit *CodingStructure::getCU(const Position &pos, const ChannelType effChType) const
{
  const CodingStructure *cs = this;

  while (cs && !cs->area.block(effChType).contains(pos))
  {
    cs = cs->parent;
  }

  if (!cs)
  {
    return nullptr;
  }

  const CompArea &_blk = cs->area.block(effChType);

  return cs->m_cuArr[effChType][rsAddr(pos, _blk.pos(), _blk.width, unitScale[getFirstComponentOfChannel(effChType)])];
}

TransformUnit *CodingStructure::getTU(const Position &pos, const ChannelType effChType)
{
  const CodingStructure *cs = this;

  while (cs && !cs->area.block(effChType).contains(pos))
  {
    cs = cs->parent;
  }

  if (!cs)
  {
    return nullptr;
  }

  const CompArea &_blk = cs->area.block(effChType);

  const CodingUnit *cu =
    cs->m_cuArr[effChType][rsAddr(pos, _blk.pos(), _blk.width, unitScale[getFirstComponentOfChannel(effChType)])];

  if (!cu)
  {
    return nullptr;
  }

  TransformUnit *ptu = cu->firstTU;

  while (ptu && !ptu->block(effChType).contains(pos))
  {
    ptu = ptu->next;
  }

  return ptu;
}

const TransformUnit *CodingStructure::getTU(const Position &pos, const ChannelType effChType) const
{
  const CodingStructure *cs = this;

  while (cs && !cs->area.block(effChType).contains(pos))
  {
    cs = cs->parent;
  }

  if (!cs)
  {
    return nullptr;
  }

  const CompArea &_blk = cs->area.block(effChType);

  const CodingUnit *cu =
    cs->m_cuArr[effChType][rsAddr(pos, _blk.pos(), _blk.width, unitScale[getFirstComponentOfChannel(effChType)])];

  if (!cu)
  {
    return nullptr;
  }

  const TransformUnit *ptu = cu->firstTU;

  while (ptu && !ptu->block(effChType).contains(pos))
  {
    ptu = ptu->next;
  }

  return ptu;
}

CodingUnit &CodingStructure::addCU(const UnitArea &unit, const ChannelType chType)
{
  CodingUnit *cu = m_cuPool.get();

  cu->UnitArea::operator=(unit);
  cu->initData();
  cu->cs      = this;
  cu->slice   = nullptr;
  cu->next    = nullptr;
  cu->firstTU = nullptr;
  cu->lastTU  = nullptr;
  cu->chType  = chType;

  CodingUnit *prevCU = m_numCUs > 0 ? cus.back() : nullptr;

  if (prevCU)
  {
    prevCU->next = cu;
  }

  cus.push_back(cu);

  uint32_t idx = ++m_numCUs;
  cu->idx      = idx;

  for (auto chType = ChannelType::LUMA; chType <= ::getLastChannel(area.chromaFormat); chType++)
  {
    if (!cu->block(chType).valid())
    {
      continue;
    }

    const CompArea &_selfBlk = area.block(chType);
    const CompArea &_blk     = cu->block(chType);

    const UnitScale &scale      = unitScale[_blk.compID];
    const Area       scaledSelf = scale.scale(_selfBlk);
    const Area       scaledBlk  = scale.scale(_blk);
    CodingUnit     **cuPtr      = m_cuArr[chType] + rsAddr(scaledBlk.pos(), scaledSelf.pos(), scaledSelf.width);
    CHECK(*cuPtr, "Overwriting a pre-existing value, should be '0'!");
    AreaBuf<CodingUnit *>(cuPtr, scaledSelf.width, scaledBlk.size()).fill(cu);
  }

  return *cu;
}

TransformUnit &CodingStructure::addTU(const UnitArea &unit, const ChannelType chType)
{
  TransformUnit *tu = m_tuPool.get();

  tu->UnitArea::operator=(unit);
  tu->initData();
  tu->next   = nullptr;
  tu->prev   = nullptr;
  tu->cs     = this;
  tu->cu     = m_isTuEnc ? cus[0] : getCU(unit.block(chType).pos(), chType);
  tu->chType = chType;

  TransformUnit *prevTU = m_numTUs > 0 ? tus.back() : nullptr;

  if (prevTU && prevTU->cu == tu->cu)
  {
    prevTU->next = tu;
    tu->prev     = prevTU;
  }

  tus.push_back(tu);

  if (tu->cu)
  {
    if (tu->cu->firstTU == nullptr)
    {
      tu->cu->firstTU = tu;
    }
    tu->cu->lastTU = tu;
  }

  uint32_t idx = ++m_numTUs;
  tu->idx      = idx;

  TCoeff                        *coeffs[5]        = { nullptr, nullptr, nullptr, nullptr, nullptr };
  uint8_t                       *signsPredArea[5] = { nullptr, nullptr, nullptr, nullptr, nullptr };
  EnumArray<Pel *, ChannelType>  pltIdx;
  EnumArray<bool *, ChannelType> runType;

  for (auto chType = ChannelType::LUMA; chType <= ::getLastChannel(area.chromaFormat); chType++)
  {
    if (!tu->block(chType).valid())
    {
      continue;
    }

    if (m_runType[chType] != nullptr)
    {
      runType[chType] = m_runType[chType] + m_offsets[getFirstComponentOfChannel(chType)];
    }
    if (m_pltIdx[chType] != nullptr)
    {
      pltIdx[chType] = m_pltIdx[chType] + m_offsets[getFirstComponentOfChannel(chType)];
    }
  }

  uint32_t numComp = ::getNumberValidComponents(area.chromaFormat);

  for (uint32_t i = 0; i < numComp; i++)
  {
    if (!tu->blocks[i].valid())
    {
      continue;
    }

    coeffs[i]        = m_coeffs[i] + m_offsets[i];
    signsPredArea[i] = m_signsPredArea[i] + m_offsets[i];

    unsigned areaSize = tu->blocks[i].area();
    m_offsets[i] += areaSize;
  }
  tu->init(coeffs, signsPredArea, pltIdx, runType);

  return *tu;
}

void CodingStructure::addEmptyTUs(Partitioner &partitioner)
{
  const UnitArea &area    = partitioner.currArea();
  bool            split   = partitioner.canSplit(TU_MAX_TR_SPLIT, *this);
  const unsigned  trDepth = partitioner.currTrDepth;

  if (split)
  {
    partitioner.splitCurrArea(TU_MAX_TR_SPLIT, *this);
    do
    {
      addEmptyTUs(partitioner);
    } while (partitioner.nextPart(*this));

    partitioner.exitCurrSplit();
  }
  else
  {
    TransformUnit &tu        = this->addTU(CS::getArea(*this, area, partitioner.chType), partitioner.chType);
    unsigned       numBlocks = ::getNumberValidTBlocks(*this->pcv);
    for (unsigned compID = COMP_Y; compID < numBlocks; compID++)
    {
      if (tu.blocks[compID].valid())
      {
        tu.getCoeffs(CompID(compID)).fill(0);
      }
    }
    tu.depth = trDepth;
  }
}

CUTraverser CodingStructure::traverseCUs(const UnitArea &unit, const ChannelType effChType)
{
  CodingUnit *firstCU = getCU(isLuma(effChType) ? unit.lumaPos() : unit.chromaPos(), effChType);
  CodingUnit *lastCU  = firstCU;

  do
  {
  } while (lastCU && (lastCU = lastCU->next) && unit.contains(*lastCU));

  return CUTraverser(firstCU, lastCU);
}

TUTraverser CodingStructure::traverseTUs(const UnitArea &unit, const ChannelType effChType)
{
  TransformUnit *firstTU = getTU(isLuma(effChType) ? unit.lumaPos() : unit.chromaPos(), effChType);
  TransformUnit *lastTU  = firstTU;

  do
  {
  } while (lastTU && (lastTU = lastTU->next) && unit.contains(*lastTU));

  return TUTraverser(firstTU, lastTU);
}

cCUTraverser CodingStructure::traverseCUs(const UnitArea &unit, const ChannelType effChType) const
{
  const CodingUnit *firstCU = getCU(isLuma(effChType) ? unit.lumaPos() : unit.chromaPos(), effChType);
  const CodingUnit *lastCU  = firstCU;

  do
  {
  } while (lastCU && (lastCU = lastCU->next) && unit.contains(*lastCU));

  return cCUTraverser(firstCU, lastCU);
}

cTUTraverser CodingStructure::traverseTUs(const UnitArea &unit, const ChannelType effChType) const
{
  const TransformUnit *firstTU = getTU(isLuma(effChType) ? unit.lumaPos() : unit.chromaPos(), effChType);
  const TransformUnit *lastTU  = firstTU;

  do
  {
  } while (lastTU && (lastTU = lastTU->next) && unit.contains(*lastTU));

  return cTUTraverser(firstTU, lastTU);
}

// coding utilities

void CodingStructure::allocateVectorsAtPicLevel()
{
  const int twice     = !pcv->ISingleTree && slice->isIRAP() && isChromaEnabled(pcv->chrFormat) ? 2 : 1;
  size_t    allocSize = twice * unitScale[COMP_Y].scale(area.blocks[COMP_Y].size()).area();

  cus.reserve(allocSize);
  tus.reserve(allocSize);
}

void CodingStructure::create(const ChromaFormat &_chromaFormat, const Area &_area, const bool isTopLayer,
                             const bool isPLTused, const bool isTempPartPredUsed)
{
  createInternals(UnitArea(_chromaFormat, _area), isTopLayer, isPLTused, isTempPartPredUsed);

  if (isTopLayer)
  {
    return;
  }

  m_reco.create(area);
  m_pred.create(area);
  m_resi.create(area);
  m_orgr.create(area);
#if ENABLE_NNLF
  m_predCustom.create(area);
#endif
}

void CodingStructure::create(const UnitArea &_unit, const bool isTopLayer, const bool isPLTused,
                             const bool isTempPartPredUsed)
{
  createInternals(_unit, isTopLayer, isPLTused, isTempPartPredUsed);

  if (isTopLayer)
  {
    return;
  }

  m_reco.create(area);
  m_pred.create(area);
  m_resi.create(area);
  m_orgr.create(area);
#if ENABLE_NNLF
  m_predCustom.create(area);
#endif
}

void CodingStructure::createInternals(const UnitArea &_unit, const bool isTopLayer, const bool isPLTused,
                                      const bool isTempPartPredUsed)
{
  area      = _unit;
  m_maxArea = _unit;

  for (int i = 0; i < unitScale.size(); i++)
  {
    const auto c       = CompID(i);
    const bool present = isLuma(c) || isChromaEnabled(area.chromaFormat);
    const int  scaleX  = present ? 2 >> getComponentScaleX(c, area.chromaFormat) : 0;
    const int  scaleY  = present ? 2 >> getComponentScaleY(c, area.chromaFormat) : 0;
    unitScale[i]       = UnitScale(scaleX, scaleY);
  }

  picture = nullptr;
  parent  = nullptr;

  for (auto chType = ChannelType::LUMA; chType <= ::getLastChannel(area.chromaFormat); chType++)
  {
    unsigned _area = unitScale[getFirstComponentOfChannel(chType)].scale(area.block(chType).size()).area();

    m_cuArr[chType]    = _area > 0 ? new CodingUnit *[_area] : nullptr;
    m_isDecomp[chType] = _area > 0 ? new bool[_area] : nullptr;
  }

  const int numComp = getNumberValidComponents(area.chromaFormat);

  for (int i = 0; i < numComp; i++)
  {
    m_offsets[i] = 0;
  }

  if (!isTopLayer)
  {
    createCoeffs(isPLTused);
  }

  unsigned _lumaAreaScaled = g_miScaling.scale(area.lumaSize()).area();
  m_motionBuf              = new MotionInfo[_lumaAreaScaled];
  if (isTopLayer && isTempPartPredUsed)
  {
    m_currQtDepthBuf = new SplitPred[_lumaAreaScaled];
  }

  m_eipIdxBuf = new int[_lumaAreaScaled];
  m_eipModelLUT.resize(0);

  initStructData();
}

void CodingStructure::addMiToLut(static_vector<MotionInfo, MAX_NUM_HMVP_CANDS> &lut, const MotionInfo &mi)
{
  size_t currCnt = lut.size();

  bool pruned      = false;
  int  sameCandIdx = 0;

  for (int idx = 0; idx < currCnt; idx++)
  {
    if (lut[idx] == mi)
    {
      sameCandIdx = idx;
      pruned      = true;
      break;
    }
  }

  if (pruned || currCnt == lut.capacity())
  {
    lut.erase(lut.begin() + sameCandIdx);
  }

  lut.push_back(mi);
}

void CodingStructure::addAffMiToLut(static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS> *lutSet,
                                    const AffineMotionInfo addMi[2], int refIdx[2])
{
  for (int reflist = 0; reflist < 2; reflist++)
  {
    if (refIdx[reflist] != -1 && addMi[reflist].oneSetAffineParametersPattern != 0)
    {
      int idxInLUT =
        reflist * MAX_NUM_AFFHMVP_ENTRIES_ONELIST + std::min(refIdx[reflist], MAX_NUM_AFFHMVP_ENTRIES_ONELIST - 1);

      static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS> &lut = lutSet[idxInLUT];

      size_t currCnt = lut.size();

      bool pruned      = false;
      int  sameCandIdx = 0;

      for (int idx = 0; idx < currCnt; idx++)
      {
        if (lut[idx] == addMi[reflist])
        {
          sameCandIdx = idx;
          pruned      = true;
          break;
        }
      }

      if (pruned || currCnt == lut.capacity())
      {
        lut.erase(lut.begin() + sameCandIdx);
      }

      lut.push_back(addMi[reflist]);
    }
  }
}

void CodingStructure::addAffInheritToLut(static_vector<AffineInheritInfo, MAX_NUM_AFF_INHERIT_HMVP_CANDS> &lut,
                                         const AffineInheritInfo                                          &mi)
{
  size_t currCnt = lut.size();

  bool pruned      = false;
  int  sameCandIdx = 0;
  for (int idx = 0; idx < currCnt; idx++)
  {
    if (lut[idx] == mi)
    {
      sameCandIdx = idx;
      pruned      = true;
      break;
    }
  }

  if (pruned || currCnt == lut.capacity())
  {
    lut.erase(lut.begin() + sameCandIdx);
  }

  lut.push_back(mi);
}

void CodingStructure::resetPrevPLT(PLTBuf &prevPLT)
{
  for (int ch = 0; ch < MAX_NUM_CHANNEL_TYPE; ch++)
  {
    prevPLT.curPLTSize[ch] = 0;
  }

  for (int comp = 0; comp < MAX_NUM_COMP; comp++)
  {
    memset(prevPLT.curPLT[comp], 0, MAXPLTPREDSIZE * sizeof(Pel));
  }
}

void CodingStructure::reorderPrevPLT(PLTBuf &prevPLT, uint8_t curPLTSize[MAX_NUM_CHANNEL_TYPE],
                                     Pel  curPLT[MAX_NUM_COMP][MAXPLTSIZE],
                                     bool reuseflag[MAX_NUM_CHANNEL_TYPE][MAXPLTPREDSIZE], uint32_t compBegin,
                                     uint32_t numComp, bool jointPLT)
{
  Pel     stuffedPLT[MAX_NUM_COMP][MAXPLTPREDSIZE];
  uint8_t tempCurPLTsize[MAX_NUM_CHANNEL_TYPE];
  uint8_t stuffPLTsize[MAX_NUM_COMP];

  uint32_t maxPredPltSize = jointPLT ? MAXPLTPREDSIZE : MAXPLTPREDSIZE_DUALTREE;

  for (int i = compBegin; i < (compBegin + numComp); i++)
  {
    CompID comID          = jointPLT ? (CompID)compBegin : ((i > 0) ? COMP_Cb : COMP_Y);
    tempCurPLTsize[comID] = curPLTSize[comID];
    stuffPLTsize[i]       = 0;
    memcpy(stuffedPLT[i], curPLT[i], curPLTSize[comID] * sizeof(Pel));
  }

  for (int ch = compBegin; ch < (compBegin + numComp); ch++)
  {
    CompID comID = jointPLT ? (CompID)compBegin : ((ch > 0) ? COMP_Cb : COMP_Y);
    if (ch > 1)
    {
      break;
    }
    for (int i = 0; i < prevPLT.curPLTSize[comID]; i++)
    {
      if (tempCurPLTsize[comID] + stuffPLTsize[ch] >= maxPredPltSize)
      {
        break;
      }

      if (!reuseflag[comID][i])
      {
        if (ch == COMP_Y)
        {
          stuffedPLT[0][tempCurPLTsize[comID] + stuffPLTsize[ch]] = prevPLT.curPLT[0][i];
        }
        else
        {
          stuffedPLT[1][tempCurPLTsize[comID] + stuffPLTsize[ch]] = prevPLT.curPLT[1][i];
          stuffedPLT[2][tempCurPLTsize[comID] + stuffPLTsize[ch]] = prevPLT.curPLT[2][i];
        }
        stuffPLTsize[ch]++;
      }
    }
  }

  for (int i = compBegin; i < (compBegin + numComp); i++)
  {
    CompID comID              = jointPLT ? (CompID)compBegin : ((i > 0) ? COMP_Cb : COMP_Y);
    prevPLT.curPLTSize[comID] = curPLTSize[comID] + stuffPLTsize[comID];
    memcpy(prevPLT.curPLT[i], stuffedPLT[i], prevPLT.curPLTSize[comID] * sizeof(Pel));
    CHECK(prevPLT.curPLTSize[comID] > maxPredPltSize, " Maximum palette predictor size exceed limit");
  }
}

void CodingStructure::setPrevPLT(PLTBuf predictor)
{
  for (int comp = 0; comp < MAX_NUM_CHANNEL_TYPE; comp++)
  {
    prevPLT.curPLTSize[comp] = predictor.curPLTSize[comp];
  }
  for (int comp = 0; comp < MAX_NUM_COMP; comp++)
  {
    memcpy(prevPLT.curPLT[comp], predictor.curPLT[comp], MAXPLTPREDSIZE * sizeof(Pel));
  }
}

void CodingStructure::storePrevPLT(PLTBuf &predictor)
{
  for (int comp = 0; comp < MAX_NUM_CHANNEL_TYPE; comp++)
  {
    predictor.curPLTSize[comp] = prevPLT.curPLTSize[comp];
  }
  for (int comp = 0; comp < MAX_NUM_COMP; comp++)
  {
    memcpy(predictor.curPLT[comp], prevPLT.curPLT[comp], MAXPLTPREDSIZE * sizeof(Pel));
  }
}

void CodingStructure::rebindPicBufs()
{
  CHECK(parent, "rebindPicBufs can only be used for the top level CodingStructure");

  if (!picture->m_bufs[PIC_RECONSTRUCTION].bufs.empty())
  {
    m_reco.createFromBuf(picture->m_bufs[PIC_RECONSTRUCTION]);
  }
  else
  {
    m_reco.destroy();
  }
  if (!picture->m_bufs[PIC_PREDICTION].bufs.empty())
  {
    m_pred.createFromBuf(picture->m_bufs[PIC_PREDICTION]);
  }
  else
  {
    m_pred.destroy();
  }
  if (!picture->m_bufs[PIC_RESIDUAL].bufs.empty())
  {
    m_resi.createFromBuf(picture->m_bufs[PIC_RESIDUAL]);
  }
  else
  {
    m_resi.destroy();
  }
  if (pcv->isEncoder)
  {
    if (!picture->m_bufs[PIC_RESIDUAL].bufs.empty())
    {
      m_orgr.create(area.chromaFormat, area.blocks[0], pcv->maxCUWidth);
    }
    else
    {
      m_orgr.destroy();
    }
  }
#if ENABLE_NNLF
  if (!picture->m_bufs[PIC_PREDICTION_CUSTOM].bufs.empty())
  {
    m_predCustom.createFromBuf(picture->m_bufs[PIC_PREDICTION_CUSTOM]);
  }
  else
  {
    m_predCustom.destroy();
  }
#endif
}

void CodingStructure::createCoeffs(const bool isPLTused)
{
  const unsigned numCh = getNumberValidComponents(area.chromaFormat);

  for (unsigned i = 0; i < numCh; i++)
  {
    unsigned _area = area.blocks[i].area();

    m_coeffs[i]        = _area > 0 ? (TCoeff *)xMalloc(TCoeff, _area) : nullptr;
    m_signsPredArea[i] = _area > 0 ? (uint8_t *)xMalloc(uint8_t, _area) : nullptr;
  }

  if (isPLTused)
  {
    for (auto chType = ChannelType::LUMA; chType <= ::getLastChannel(area.chromaFormat); chType++)
    {
      unsigned _area = area.block(chType).area();

      m_pltIdx[chType]  = _area > 0 ? (Pel *)xMalloc(Pel, _area) : nullptr;
      m_runType[chType] = _area > 0 ? (bool *)xMalloc(bool, _area) : nullptr;
    }
  }
}

void CodingStructure::destroyCoeffs()
{
  for (uint32_t i = 0; i < MAX_NUM_COMP; i++)
  {
    if (m_coeffs[i])
    {
      xFree(m_coeffs[i]);
      m_coeffs[i] = nullptr;
    }
    if (m_signsPredArea[i])
    {
      xFree(m_signsPredArea[i]);
      m_signsPredArea[i] = nullptr;
    }
  }

  for (auto &ptr: m_pltIdx)
  {
    if (ptr != nullptr)
    {
      xFree(ptr);
      ptr = nullptr;
    }
  }
  for (auto &ptr: m_runType)
  {
    if (ptr != nullptr)
    {
      xFree(ptr);
      ptr = nullptr;
    }
  }
}

void CodingStructure::initSubStructure(CodingStructure &subStruct, const ChannelType _chType, const UnitArea &subArea,
                                       const bool &isTuEnc)
{
  CHECK(this == &subStruct, "Trying to init self as sub-structure");

  subStruct.costDbOffset = 0;

  if (parent)
  {
    // allow this to be false at the top level (need for edge CTU's)
    CHECKD(!area.contains(subArea), "Trying to init sub-structure not contained in the parent");
  }

  subStruct.compactResize(subArea);

  subStruct.parent  = this;
  subStruct.picture = picture;

  subStruct.sps       = sps;
  subStruct.vps       = vps;
  subStruct.pps       = pps;
  subStruct.picHeader = picHeader;

  memcpy(subStruct.alfApss, alfApss, sizeof(alfApss));

  subStruct.lmcsAps        = lmcsAps;
  subStruct.scalinglistAps = scalinglistAps;

  subStruct.slice           = slice;
  subStruct.baseQP          = baseQP;
  subStruct.prevQP[_chType] = prevQP[_chType];
  subStruct.pcv             = pcv;

  subStruct.m_isTuEnc = isTuEnc;

  subStruct.motionLut = motionLut;
  subStruct.ccpLut    = ccpLut;
  subStruct.prevPLT   = prevPLT;
  subStruct.eipLut    = eipLut;

  subStruct.initStructData(currQP[_chType]);

  if (isTuEnc)
  {
    CHECKD(area != subStruct.area, "Trying to init sub-structure for TU-encoding of incompatible size");

    for (const auto &pcu: cus)
    {
      CodingUnit &cu = subStruct.addCU(*pcu, _chType);

      cu = *pcu;
    }

    for (auto chType = ChannelType::LUMA; chType <= ::getLastChannel(area.chromaFormat); chType++)
    {
      std::copy_n(m_isDecomp[chType],
                  unitScale[getFirstComponentOfChannel(chType)].scale(area.block(chType).size()).area(),
                  subStruct.m_isDecomp[chType]);
    }
  }
}

void CodingStructure::useSubStructure(const CodingStructure &subStruct, const ChannelType chType,
                                      const UnitArea &subArea, const bool cpyPred /*= true*/,
                                      const bool cpyReco /*= true*/, const bool cpyOrgResi /*= true*/,
                                      const bool cpyResi /*= true*/, const bool updateCost /*= true*/)
{
  UnitArea clippedArea = clipArea(subArea, *picture);

  setDecomp(clippedArea);

  CPelUnitBuf subPredBuf = cpyPred ? subStruct.getPredBuf(clippedArea) : CPelUnitBuf();
  CPelUnitBuf subResiBuf = cpyResi ? subStruct.getResiBuf(clippedArea) : CPelUnitBuf();
  CPelUnitBuf subRecoBuf = cpyReco ? subStruct.getRecoBuf(clippedArea) : CPelUnitBuf();

  if (parent)
  {
    // copy data to picture
#if ENABLE_NNLF
    if (cpyPred || sps->m_nnlfStore)
    {
      getPredBuf(clippedArea).copyFrom(subStruct.getPredBuf(clippedArea));
    }
#else
    if (cpyPred)
    {
      getPredBuf(clippedArea).copyFrom(subPredBuf);
    }
#endif
    if (cpyResi)
    {
      getResiBuf(clippedArea).copyFrom(subResiBuf);
    }
    if (cpyReco)
    {
      getRecoBuf(clippedArea).copyFrom(subRecoBuf);
    }
    if (cpyOrgResi)
    {
      getOrgResiBuf(clippedArea).copyFrom(subStruct.getOrgResiBuf(clippedArea));
    }
  }

  if (cpyPred)
  {
    picture->getPredBuf(clippedArea).copyFrom(subPredBuf);
  }
  if (cpyResi)
  {
    picture->getResiBuf(clippedArea).copyFrom(subResiBuf);
  }
  if (cpyReco)
  {
    picture->getRecoBuf(clippedArea).copyFrom(subRecoBuf);
  }

#if ENABLE_NNLF
  if (sps->m_nnlfStore && !parent)
  {
    getPredBufCustom(clippedArea).copyFrom(subStruct.getPredBuf(clippedArea));
  }
#endif

  if (!subStruct.m_isTuEnc && ((!slice->isIntra() || slice->m_ibcFlag) && chType != ChannelType::CHROMA))
  {
    // copy motion buffer
    MotionBuf  ownMB = getMotionBuf(clippedArea);
    CMotionBuf subMB = subStruct.getMotionBuf(clippedArea);

    ownMB.copyFrom(subMB);

    motionLut = subStruct.motionLut;
  }

  ccpLut  = subStruct.ccpLut;
  prevPLT = subStruct.prevPLT;
  if ((slice->isIntra() || (!slice->isIntra() && !subStruct.m_isTuEnc)) && chType != ChannelType::CHROMA)
  {
    eipLut = subStruct.eipLut;
  }

  if (updateCost)
  {
    fracBits += subStruct.fracBits;
    dist += subStruct.dist;
    cost += subStruct.cost;
    costDbOffset += subStruct.costDbOffset;
  }
  if (parent)
  {
    // allow this to be false at the top level
    CHECKD(!area.contains(subArea), "Trying to use a sub-structure not contained in self");
  }

  // copy the CUs over
  if (subStruct.m_isTuEnc)
  {
    // don't copy if the substruct was created for encoding of the TUs
  }
  else
  {
    for (const auto &pcu: subStruct.cus)
    {
      // add an analogue CU into own CU store
      const UnitArea &cuPatch = *pcu;
      CodingUnit     &cu      = addCU(cuPatch, chType);

      // copy the CU info from subPatch
      cu = *pcu;
    }
  }

  // copy the TUs over
  for (const auto &ptu: subStruct.tus)
  {
    // add an analogue TU into own TU store
    const UnitArea &tuPatch = *ptu;
    TransformUnit  &tu      = addTU(tuPatch, chType);

    // copy the TU info from subPatch
    tu = *ptu;
  }
}

void CodingStructure::copyStructure(const CodingStructure &other, const ChannelType chType, const bool copyTUs,
                                    const bool copyRecoBuf)
{
  fracBits     = other.fracBits;
  dist         = other.dist;
  cost         = other.cost;
  costDbOffset = other.costDbOffset;
  CHECKD(area != other.area, "Incompatible sizes");

  const UnitArea dualITreeArea = CS::getArea(*this, this->area, chType);

  // copy the CUs over
  for (const auto &pcu: other.cus)
  {
    if (!dualITreeArea.contains(*pcu))
    {
      continue;
    }
    // add an analogue CU into own CU store
    const UnitArea &cuPatch = *pcu;

    CodingUnit &cu = addCU(cuPatch, pcu->chType);

    // copy the CU info from subPatch
    cu = *pcu;
  }

  if (!other.slice->isIntra() || other.slice->m_ibcFlag)
  {
    // copy motion buffer
    MotionBuf  ownMB = getMotionBuf();
    CMotionBuf subMB = other.getMotionBuf();

    ownMB.copyFrom(subMB);

    motionLut = other.motionLut;
  }

  ccpLut                     = other.ccpLut;
  prevPLT                    = other.prevPLT;
  eipLut                     = other.eipLut;
  EipModelIdxBuf  ownEipIdxB = getEipIdxBuf();
  CEipModelIdxBuf subEipIdxB = other.getEipIdxBuf();
  ownEipIdxB.copyFrom(subEipIdxB);
  m_eipModelLUT = other.m_eipModelLUT;

  if (copyTUs)
  {
    // copy the TUs over
    for (const auto &ptu: other.tus)
    {
      if (!dualITreeArea.contains(*ptu))
      {
        continue;
      }
      // add an analogue TU into own TU store
      const UnitArea &tuPatch = *ptu;
      TransformUnit  &tu      = addTU(tuPatch, ptu->chType);
      // copy the TU info from subPatch
      tu                      = *ptu;
    }
  }

  if (copyRecoBuf)
  {
    CPelUnitBuf recoBuf = other.getRecoBuf(area);

    if (parent)
    {
      // copy data to self for neighbors
      getRecoBuf(area).copyFrom(recoBuf);
    }

    // copy data to picture
    picture->getRecoBuf(area).copyFrom(recoBuf);
    if (other.pcv->isEncoder)
    {
      CPelUnitBuf predBuf = other.getPredBuf(area);
      if (parent)
      {
        getPredBuf(area).copyFrom(predBuf);
      }
      picture->getPredBuf(area).copyFrom(predBuf);
    }

    // required for DebugCTU
    for (auto chType = ChannelType::LUMA; chType <= ::getLastChannel(area.chromaFormat); chType++)
    {
      const size_t _area = unitScale[getFirstComponentOfChannel(chType)].scaleArea(area.block(chType).area());
      std::copy_n(other.m_isDecomp[chType], _area, m_isDecomp[chType]);
    }
  }
}

void CodingStructure::initStructData(const int &QP, const bool &skipMotBuf, const UnitArea *_area)
{
  clearTUs();
  clearCUs();

  if (_area)
  {
    compactResize(*_area);
  }

  if (QP < MAX_INT)
  {
    currQP.fill(QP);
  }

  if (!skipMotBuf && (!parent || ((!slice->isIntra() || slice->m_ibcFlag) && !m_isTuEnc)))
  {
    getMotionBuf().memset(0);
  }

  if (m_currQtDepthBuf != nullptr)
  {
    getQTDepthBuf().memset(0);
  }

  fracBits     = 0;
  dist         = 0;
  cost         = MAX_DOUBLE;
  lumaCost     = MAX_DOUBLE;
  costDbOffset = 0;
  interHad     = std::numeric_limits<Distortion>::max();
}

void CodingStructure::compactResize(const UnitArea &_area)
{
  m_pred.compactResize(_area);
  m_reco.compactResize(_area);
  m_resi.compactResize(_area);
  m_orgr.compactResize(_area);
#if ENABLE_NNLF
  m_predCustom.compactResize(_area);
#endif

  for (uint32_t i = 0; i < _area.blocks.size(); i++)
  {
    CHECK(m_maxArea.blocks[i].area() < _area.blocks[i].area(), "Trying to init sub-structure of incompatible size");
  }

  area = _area;
}

void CodingStructure::clearTUs()
{
  for (auto chType = ChannelType::LUMA; chType <= ::getLastChannel(area.chromaFormat); chType++)
  {
    size_t _area = (area.block(chType).area() >> unitScale[getFirstComponentOfChannel(chType)].area);
    std::fill_n(m_isDecomp[chType], _area, false);
  }

  const int numComp = getNumberValidComponents(area.chromaFormat);
  for (int i = 0; i < numComp; i++)
  {
    m_offsets[i] = 0;
  }

  for (auto &pcu: cus)
  {
    pcu->firstTU = pcu->lastTU = nullptr;
  }

  m_tuPool.giveBack(tus);
  m_numTUs = 0;
}

void CodingStructure::clearCUs()
{
  for (auto chType = ChannelType::LUMA; chType <= ::getLastChannel(area.chromaFormat); chType++)
  {
    std::fill_n(m_cuArr[chType], unitScale[getFirstComponentOfChannel(chType)].scaleArea(area.block(chType).area()),
                nullptr);
  }

  m_cuPool.giveBack(cus);
  m_numCUs = 0;
}

MotionBuf CodingStructure::getMotionBuf(const Area &_area)
{
  const CompArea &_luma = area.Y();

  CHECKD(!_luma.contains(_area), "Trying to access motion information outside of this coding structure");

  const Area miArea   = g_miScaling.scale(_area);
  const Area selfArea = g_miScaling.scale(_luma);

  return MotionBuf(m_motionBuf + rsAddr(miArea.pos(), selfArea.pos(), selfArea.width), selfArea.width, miArea.size());
}

const CMotionBuf CodingStructure::getMotionBuf(const Area &_area) const
{
  const CompArea &_luma = area.Y();

  CHECKD(!_luma.contains(_area), "Trying to access motion information outside of this coding structure");

  const Area miArea   = g_miScaling.scale(_area);
  const Area selfArea = g_miScaling.scale(_luma);

  return MotionBuf(m_motionBuf + rsAddr(miArea.pos(), selfArea.pos(), selfArea.width), selfArea.width, miArea.size());
}

MotionInfo &CodingStructure::getMotionInfo(const Position &pos)
{
  CHECKD(!area.Y().contains(pos), "Trying to access motion information outside of this coding structure");

  // return getMotionBuf().at( g_miScaling.scale( pos - area.lumaPos() ) );
  //  bypass the motion buf calling and get the value directly
  const unsigned stride = g_miScaling.scaleHor(area.lumaSize().width);
  const Position miPos  = g_miScaling.scale(pos - area.lumaPos());

  return *(m_motionBuf + miPos.y * stride + miPos.x);
}

const MotionInfo &CodingStructure::getMotionInfo(const Position &pos) const
{
  CHECKD(!area.Y().contains(pos), "Trying to access motion information outside of this coding structure");

  // return getMotionBuf().at( g_miScaling.scale( pos - area.lumaPos() ) );
  //  bypass the motion buf calling and get the value directly
  const unsigned stride = g_miScaling.scaleHor(area.lumaSize().width);
  const Position miPos  = g_miScaling.scale(pos - area.lumaPos());

  return *(m_motionBuf + miPos.y * stride + miPos.x);
}

void CodingStructure::setSplitPred()
{
  const Picture *pic    = picture->m_unscaledPic;
  const int      width  = pic->getPicWidthInLumaSamples();
  const int      height = pic->getPicHeightInLumaSamples();
  Position       pos;
  CodingUnit    *cuColAll = NULL;
  QTDepthBuf     mb;
  SplitPred      sp;

  int offset                    = slice->m_pps->m_ctuSize >> 1;
  picture->m_maxTemporalBtDepth = slice->m_picHeader->getMaxMTTHierarchyDepth(slice->m_eSliceType, ChannelType::LUMA);
  const uint32_t roundVal       = ((slice->m_pps->m_ctuSize >> 2) * (slice->m_pps->m_ctuSize >> 2)) *
    (slice->m_pps->m_ctuSize >> LOG2_BLOCK_RESOLUTION) * (slice->m_pps->m_ctuSize >> LOG2_BLOCK_RESOLUTION);

  uint32_t sumQtdepthColArea = 0;
  uint32_t nbSamples         = 0;
  uint32_t nbSamplesMTT      = 0;

  const uint8_t nbPos = (offset << 1) >> LOG2_BLOCK_RESOLUTION;
  bool          parsedPositions[256 >> LOG2_BLOCK_RESOLUTION][256 >> LOG2_BLOCK_RESOLUTION];
  for (int h = 0; h < height; h = h + QTBTTT_TEMPO_PRED_BUFFER_SIZE)
  {
    for (int w = 0; w < width; w = w + QTBTTT_TEMPO_PRED_BUFFER_SIZE)
    {
      sumQtdepthColArea = 0;
      nbSamples         = 0;

      uint8_t  maxBtdepthColArea  = 0;
      uint32_t sumMttdepthColArea = 0;
      nbSamplesMTT                = 0;
      uint8_t minQtdepthColArea   = UINT8_MAX;

      pos = Position { PosType(w), PosType(h) };

      int topLeftWindowX     = pos.x - offset;
      int topLeftWindowY     = pos.y - offset;
      int bottomRightWindowX = pos.x + offset;
      int bottomRightWindowY = pos.y + offset;

      for (int i = 0; i < nbPos; i++)
      {
        for (int j = 0; j < nbPos; j++)
        {
          parsedPositions[i][j] = false;
        }
      }

      if ((topLeftWindowX < 0) || (topLeftWindowY < 0) || (bottomRightWindowX >= width) ||
          (bottomRightWindowY >= height))
      {
        for (int i = 0; i < nbPos; i++)
        {
          for (int j = 0; j < nbPos; j++)
          {
            if (topLeftWindowX + i * QTBTTT_TEMPO_PRED_BLOCK_RESOLUTION < 0)
            {
              parsedPositions[i][j] = true;
            }
            if (topLeftWindowY + j * QTBTTT_TEMPO_PRED_BLOCK_RESOLUTION < 0)
            {
              parsedPositions[i][j] = true;
            }
            if (topLeftWindowX + i * QTBTTT_TEMPO_PRED_BLOCK_RESOLUTION >= width)
            {
              parsedPositions[i][j] = true;
            }
            if (topLeftWindowY + j * QTBTTT_TEMPO_PRED_BLOCK_RESOLUTION >= height)
            {
              parsedPositions[i][j] = true;
            }
          }
        }
      }

      for (int x = 0; x < 0 + (offset << 1); x = x + QTBTTT_TEMPO_PRED_BLOCK_RESOLUTION)
      {
        for (int y = 0; y < 0 + (offset << 1); y = y + QTBTTT_TEMPO_PRED_BLOCK_RESOLUTION)
        {
          if (!parsedPositions[x >> LOG2_BLOCK_RESOLUTION][y >> LOG2_BLOCK_RESOLUTION])
          {
            cuColAll = getCU(pos.offset(x - offset, y - offset), ChannelType::LUMA);
            if (!cuColAll)
            {
              continue;
            }

            int widthBlockRes = (cuColAll->lumaPos().x - (pos.x + x - offset) + cuColAll->lwidth() +
                                 QTBTTT_TEMPO_PRED_BLOCK_RESOLUTION - 1) >>
              LOG2_BLOCK_RESOLUTION;
            int heightBlockRes = (cuColAll->lumaPos().y - (pos.y + y - offset) + cuColAll->lheight() +
                                  QTBTTT_TEMPO_PRED_BLOCK_RESOLUTION - 1) >>
              LOG2_BLOCK_RESOLUTION;

            if (((x >> LOG2_BLOCK_RESOLUTION) + widthBlockRes > ((offset << 1) >> LOG2_BLOCK_RESOLUTION)))
            {
              widthBlockRes = ((offset << 1) >> LOG2_BLOCK_RESOLUTION) - (x >> LOG2_BLOCK_RESOLUTION);
            }
            if (((y >> LOG2_BLOCK_RESOLUTION) + heightBlockRes > ((offset << 1) >> LOG2_BLOCK_RESOLUTION)))
            {
              heightBlockRes = ((offset << 1) >> LOG2_BLOCK_RESOLUTION) - (y >> LOG2_BLOCK_RESOLUTION);
            }

            for (int i = x >> LOG2_BLOCK_RESOLUTION; i < (x >> LOG2_BLOCK_RESOLUTION) + widthBlockRes; i++)
            {
              for (int j = y >> LOG2_BLOCK_RESOLUTION; j < (y >> LOG2_BLOCK_RESOLUTION) + heightBlockRes; j++)
              {
                parsedPositions[i][j] = true;
              }
            }

            int sizeBlock = (cuColAll->lheight()) * (cuColAll->lwidth());
            sumQtdepthColArea += cuColAll->qtDepth * ((roundVal * (heightBlockRes * widthBlockRes)) / sizeBlock);
            nbSamples += ((roundVal * (heightBlockRes * widthBlockRes)) / sizeBlock);

            if (cuColAll->mtDepth - cuColAll->mtImplicitDepth > maxBtdepthColArea)
            {
              maxBtdepthColArea = cuColAll->mtDepth - cuColAll->mtImplicitDepth;
            }

            if (cuColAll->mtImplicitDepth == 0)
            {
              sumMttdepthColArea += (cuColAll->mtDepth - cuColAll->mtImplicitDepth) *
                ((roundVal * (heightBlockRes * widthBlockRes)) / sizeBlock);
              nbSamplesMTT += ((roundVal * (heightBlockRes * widthBlockRes)) / sizeBlock);
            }

            if (cuColAll->qtDepth < minQtdepthColArea)
            {
              minQtdepthColArea = cuColAll->qtDepth;
            }
          }
        }
      }

      if (nbSamples > 0)
      {
        sumQtdepthColArea = (sumQtdepthColArea + (nbSamples >> 1)) / nbSamples;
      }
      sp.qtDepth    = (uint8_t)sumQtdepthColArea;
      sp.maxBtDepth = maxBtdepthColArea;

      if (nbSamplesMTT > 0)
      {
        sumMttdepthColArea = (sumMttdepthColArea + (nbSamplesMTT >> 1)) / nbSamplesMTT;
      }
      sp.mttDepth = (uint8_t)sumMttdepthColArea;

      sp.minQtDepth = minQtdepthColArea;

      mb = getQtDepthBuf(Area(pos.x, pos.y, std::min(width - w, QTBTTT_TEMPO_PRED_BUFFER_SIZE),
                              std::min(height - h, QTBTTT_TEMPO_PRED_BUFFER_SIZE)));
      mb.fill(sp);
    }
  }
}

QTDepthBuf CodingStructure::getQtDepthBuf(const Area &_area)
{
  const CompArea &_luma = area.Y();
  CHECKD(!_luma.contains(_area), "Trying to access motion information outside of this coding structure");

  const Area miArea   = g_miScaling.scale(_area);
  const Area selfArea = g_miScaling.scale(_luma);

  return QTDepthBuf(m_currQtDepthBuf + rsAddr(miArea.pos(), selfArea.pos(), selfArea.width), selfArea.width,
                    miArea.size());
}

SplitPred &CodingStructure::getQtDepthInfo(const Position &pos)
{
  CHECKD(!area.Y().contains(pos),
         "Trying to access TEMPORAL partitionning information outside of this coding structure");
  const unsigned stride = g_miScaling.scaleHor(area.lumaSize().width);
  const Position miPos  = g_miScaling.scale(pos - area.lumaPos());

  return *(m_currQtDepthBuf + miPos.y * stride + miPos.x);
}

EipModelIdxBuf CodingStructure::getEipIdxBuf(const Area &bufArea)
{
  const CompArea &lumaArea = area.Y();
  CHECK(!lumaArea.contains(bufArea), "Trying to access Eip model information outside of this coding structure");
  const Area miArea   = g_miScaling.scale(bufArea);
  const Area selfArea = g_miScaling.scale(lumaArea);

  return EipModelIdxBuf(m_eipIdxBuf + rsAddr(miArea.pos(), selfArea.pos(), selfArea.width), selfArea.width,
                        miArea.size());
}

const CEipModelIdxBuf CodingStructure::getEipIdxBuf(const Area &bufArea) const
{
  const CompArea &lumaArea = area.Y();
  CHECK(!lumaArea.contains(bufArea), "Trying to access Eip model information outside of this coding structure");
  const Area miArea   = g_miScaling.scale(bufArea);
  const Area selfArea = g_miScaling.scale(lumaArea);

  return EipModelIdxBuf(m_eipIdxBuf + rsAddr(miArea.pos(), selfArea.pos(), selfArea.width), selfArea.width,
                        miArea.size());
}

int &CodingStructure::getEipIdxInfo(const Position &pos)
{
  CHECK(!area.Y().contains(pos), "Trying to access Eip model information outside of this coding structure");

  const unsigned stride = g_miScaling.scaleHor(area.lumaSize().width);
  const Position miPos  = g_miScaling.scale(pos - area.lumaPos());

  return *(m_eipIdxBuf + miPos.y * stride + miPos.x);
}

const int &CodingStructure::getEipIdxInfo(const Position &pos) const
{
  CHECK(!area.Y().contains(pos), "Trying to access Eip model information outside of this coding structure");

  const unsigned stride = g_miScaling.scaleHor(area.lumaSize().width);
  const Position miPos  = g_miScaling.scale(pos - area.lumaPos());

  return *(m_eipIdxBuf + miPos.y * stride + miPos.x);
}

// data accessors
PelBuf            CodingStructure::getPredBuf(const CompArea &blk) { return getBuf(blk, PIC_PREDICTION); }
const CPelBuf     CodingStructure::getPredBuf(const CompArea &blk) const { return getBuf(blk, PIC_PREDICTION); }
PelUnitBuf        CodingStructure::getPredBuf(const UnitArea &unit) { return getBuf(unit, PIC_PREDICTION); }
const CPelUnitBuf CodingStructure::getPredBuf(const UnitArea &unit) const { return getBuf(unit, PIC_PREDICTION); }

PelBuf            CodingStructure::getResiBuf(const CompArea &blk) { return getBuf(blk, PIC_RESIDUAL); }
const CPelBuf     CodingStructure::getResiBuf(const CompArea &blk) const { return getBuf(blk, PIC_RESIDUAL); }
PelUnitBuf        CodingStructure::getResiBuf(const UnitArea &unit) { return getBuf(unit, PIC_RESIDUAL); }
const CPelUnitBuf CodingStructure::getResiBuf(const UnitArea &unit) const { return getBuf(unit, PIC_RESIDUAL); }

PelBuf            CodingStructure::getRecoBuf(const CompArea &blk) { return getBuf(blk, PIC_RECONSTRUCTION); }
const CPelBuf     CodingStructure::getRecoBuf(const CompArea &blk) const { return getBuf(blk, PIC_RECONSTRUCTION); }
PelUnitBuf        CodingStructure::getRecoBuf(const UnitArea &unit) { return getBuf(unit, PIC_RECONSTRUCTION); }
const CPelUnitBuf CodingStructure::getRecoBuf(const UnitArea &unit) const { return getBuf(unit, PIC_RECONSTRUCTION); }

PelBuf            CodingStructure::getOrgResiBuf(const CompArea &blk) { return getBuf(blk, PIC_ORG_RESI); }
const CPelBuf     CodingStructure::getOrgResiBuf(const CompArea &blk) const { return getBuf(blk, PIC_ORG_RESI); }
PelUnitBuf        CodingStructure::getOrgResiBuf(const UnitArea &unit) { return getBuf(unit, PIC_ORG_RESI); }
const CPelUnitBuf CodingStructure::getOrgResiBuf(const UnitArea &unit) const { return getBuf(unit, PIC_ORG_RESI); }

PelBuf            CodingStructure::getOrgBuf(const CompArea &blk) { return getBuf(blk, PIC_ORIGINAL); }
const CPelBuf     CodingStructure::getOrgBuf(const CompArea &blk) const { return getBuf(blk, PIC_ORIGINAL); }
PelUnitBuf        CodingStructure::getOrgBuf(const UnitArea &unit) { return getBuf(unit, PIC_ORIGINAL); }
const CPelUnitBuf CodingStructure::getOrgBuf(const UnitArea &unit) const { return getBuf(unit, PIC_ORIGINAL); }

PelBuf CodingStructure::getOrgBuf(const CompID &compID) { return picture->getBuf(area.blocks[compID], PIC_ORIGINAL); }
const CPelBuf CodingStructure::getOrgBuf(const CompID &compID) const
{
  return picture->getBuf(area.blocks[compID], PIC_ORIGINAL);
}
PelUnitBuf        CodingStructure::getOrgBuf() { return picture->getBuf(area, PIC_ORIGINAL); }
const CPelUnitBuf CodingStructure::getOrgBuf() const { return picture->getBuf(area, PIC_ORIGINAL); }
PelUnitBuf        CodingStructure::getTrueOrgBuf() { return picture->getBuf(area, PIC_TRUE_ORIGINAL); }
const CPelUnitBuf CodingStructure::getTrueOrgBuf() const { return picture->getBuf(area, PIC_TRUE_ORIGINAL); }

#if ENABLE_NNLF
PelBuf        CodingStructure::getPredBufCustom(const CompArea &blk) { return getBuf(blk, PIC_PREDICTION_CUSTOM); }
const CPelBuf CodingStructure::getPredBufCustom(const CompArea &blk) const
{
  return getBuf(blk, PIC_PREDICTION_CUSTOM);
}
PelUnitBuf CodingStructure::getPredBufCustom(const UnitArea &unit) { return getBuf(unit, PIC_PREDICTION_CUSTOM); }
const CPelUnitBuf CodingStructure::getPredBufCustom(const UnitArea &unit) const
{
  return getBuf(unit, PIC_PREDICTION_CUSTOM);
}
#endif

PelBuf CodingStructure::getBuf(const CompArea &blk, const PictureType &type)
{
  if (!blk.valid())
  {
    return PelBuf();
  }

  if (type == PIC_ORIGINAL)
  {
    return picture->getBuf(blk, type);
  }

  const CompID compID = blk.compID;

  PelStorage *buf = type == PIC_PREDICTION
    ? &m_pred
    : (type == PIC_RESIDUAL ? &m_resi
                            : (type == PIC_RECONSTRUCTION ? &m_reco : (type == PIC_ORG_RESI ? &m_orgr : nullptr)));
#if ENABLE_NNLF
  if (type == PIC_PREDICTION_CUSTOM)
  {
    buf = &m_predCustom;
  }
#endif

  CHECK(!buf, "Unknown buffer requested");

  CHECKD(!area.blocks[compID].contains(blk), "Buffer not contained in self requested");

  CompArea cFinal = blk;
  cFinal.relativeTo(area.blocks[compID]);

#if !KEEP_PRED_AND_RESI_SIGNALS
  if (!parent && (type == PIC_RESIDUAL || type == PIC_PREDICTION))
  {
    cFinal.x &= (pcv->maxCUWidthMask >> getComponentScaleX(blk.compID, blk.chromaFormat));
    cFinal.y &= (pcv->maxCUHeightMask >> getComponentScaleY(blk.compID, blk.chromaFormat));
  }
#endif

  return buf->getBuf(cFinal);
}

const CPelBuf CodingStructure::getBuf(const CompArea &blk, const PictureType &type) const
{
  if (!blk.valid())
  {
    return PelBuf();
  }

  if (type == PIC_ORIGINAL)
  {
    return picture->getBuf(blk, type);
  }

  const CompID compID = blk.compID;

  const PelStorage *buf = type == PIC_PREDICTION
    ? &m_pred
    : (type == PIC_RESIDUAL ? &m_resi
                            : (type == PIC_RECONSTRUCTION ? &m_reco : (type == PIC_ORG_RESI ? &m_orgr : nullptr)));
#if ENABLE_NNLF
  if (type == PIC_PREDICTION_CUSTOM)
  {
    buf = &m_predCustom;
  }
#endif

  CHECK(!buf, "Unknown buffer requested");

  CHECKD(!area.blocks[compID].contains(blk), "Buffer not contained in self requested");

  CompArea cFinal = blk;
  cFinal.relativeTo(area.blocks[compID]);

#if !KEEP_PRED_AND_RESI_SIGNALS
  if (!parent && (type == PIC_RESIDUAL || type == PIC_PREDICTION))
  {
    cFinal.x &= (pcv->maxCUWidthMask >> getComponentScaleX(blk.compID, blk.chromaFormat));
    cFinal.y &= (pcv->maxCUHeightMask >> getComponentScaleY(blk.compID, blk.chromaFormat));
  }
#endif

  return buf->getBuf(cFinal);
}

PelUnitBuf CodingStructure::getBuf(const UnitArea &unit, const PictureType &type)
{
  // no parent fetching for buffers
  if (!isChromaEnabled(area.chromaFormat))
  {
    return PelUnitBuf(area.chromaFormat, getBuf(unit.Y(), type));
  }
  else
  {
    return PelUnitBuf(area.chromaFormat, getBuf(unit.Y(), type), getBuf(unit.Cb(), type), getBuf(unit.Cr(), type));
  }
}

const CPelUnitBuf CodingStructure::getBuf(const UnitArea &unit, const PictureType &type) const
{
  // no parent fetching for buffers
  if (!isChromaEnabled(area.chromaFormat))
  {
    return CPelUnitBuf(area.chromaFormat, getBuf(unit.Y(), type));
  }
  else
  {
    return CPelUnitBuf(area.chromaFormat, getBuf(unit.Y(), type), getBuf(unit.Cb(), type), getBuf(unit.Cr(), type));
  }
}

const CodingUnit *CodingStructure::getCURestricted(const Position &pos, const CodingUnit &curCu,
                                                   const ChannelType _chType) const
{
  const int csx    = getChannelTypeScaleX(_chType, area.chromaFormat);
  const int csy    = getChannelTypeScaleY(_chType, area.chromaFormat);
  const int xshift = pcv->maxCUWidthLog2 - csx;
  const int yshift = pcv->maxCUHeightLog2 - csy;
  const int ydiff  = (pos.y >> yshift) - (curCu.block(_chType).y >> yshift);
  const int xdiff  = (pos.x >> xshift) - (curCu.block(_chType).x >> xshift);

  if (!xdiff && !ydiff)
  {
    const CodingUnit *cu = getCU(pos, _chType);

    return (cu && (cu->cs != curCu.cs || cu->idx <= curCu.idx)) ? cu : nullptr;
  }

  if (ydiff > 0 || (ydiff == 0 && xdiff > 0) || (ydiff == -1 && xdiff > (sps->m_entropyCodingSyncEnabledFlag ? 0 : 1)))
  {
    return nullptr;
  }

  if (pos.x < 0 || pos.y < 0 || (pos.x * (1 << csx)) >= pcv->lumaWidth ||
      pps->getTileIdx(pos.x >> xshift, pos.y >> yshift) != curCu.tileIdx)
  {
    return nullptr;
  }

  const CodingUnit *cu = getCU(pos, _chType);

  return (cu && CU::isSameSlice(*cu, curCu)) ? cu : nullptr;
}

const CodingUnit *CodingStructure::getCURestricted(const Position &pos, const Position curPos,
                                                   const unsigned curSliceIdx, const unsigned curTileIdx,
                                                   const ChannelType _chType) const
{
  const int csx    = getChannelTypeScaleX(_chType, area.chromaFormat);
  const int csy    = getChannelTypeScaleY(_chType, area.chromaFormat);
  const int xshift = pcv->maxCUWidthLog2 - csx;
  const int yshift = pcv->maxCUHeightLog2 - csy;
  const int ydiff  = (pos.y >> yshift) - (curPos.y >> yshift);
  const int xdiff  = (pos.x >> xshift) - (curPos.x >> xshift);

  if (!xdiff && !ydiff)
  {
    return getCU(pos, _chType);
  }

  if (ydiff > 0 || (ydiff == 0 && xdiff > 0) || (ydiff == -1 && xdiff > (sps->m_entropyCodingSyncEnabledFlag ? 0 : 1)))
  {
    return nullptr;
  }

  if (pos.x < 0 || pos.y < 0 || (pos.x << csx) >= pcv->lumaWidth ||
      pps->getTileIdx(pos.x >> xshift, pos.y >> yshift) != curTileIdx)
  {
    return nullptr;
  }

  const CodingUnit *cu = getCU(pos, _chType);

  return (cu && cu->slice->m_independentSliceIdx == curSliceIdx && cu->tileIdx == curTileIdx) ? cu : nullptr;
}

const TransformUnit *CodingStructure::getTURestricted(const Position &pos, const TransformUnit &curTu,
                                                      const ChannelType _chType) const
{
  if (sps->m_entropyCodingSyncEnabledFlag)
  {
    const int xshift = pcv->maxCUWidthLog2 - getChannelTypeScaleX(_chType, curTu.chromaFormat);
    const int yshift = pcv->maxCUHeightLog2 - getChannelTypeScaleY(_chType, curTu.chromaFormat);
    if ((pos.x >> xshift) > (curTu.block(_chType).x >> xshift) ||
        (pos.y >> yshift) > (curTu.block(_chType).y >> yshift))
    {
      return nullptr;
    }
  }
  const TransformUnit *tu = getTU(pos, _chType);
  return (tu && CU::isSameSliceAndTile(*tu->cu, *curTu.cu) && (tu->cs != curTu.cs || tu->idx <= curTu.idx)) ? tu
                                                                                                            : nullptr;
}
