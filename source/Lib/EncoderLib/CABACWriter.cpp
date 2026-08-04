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

/** \file     CABACWriter.cpp
 *  \brief    Writer for low level syntax
 */

#include "CommonLib/Contexts.h"
#include "CABACWriter.h"

#include "EncLib.h"

#include "CommonLib/AlfParameters.h"
#include "CommonLib/AlfParametersEcm.h"
#include "CommonLib/AlfParametersVtm.h"

#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_buffer.h"

#include <map>
#include <algorithm>
#include <limits>

//! \ingroup EncoderLib
//! \{

#if ENABLE_CABAC_DUMP
void CABACWriter::initCtxModels(Slice &slice)
#else
void CABACWriter::initCtxModels(const Slice &slice)
#endif
{
  int       qp               = slice.m_iSliceQp;
  SliceType sliceType        = slice.m_eSliceType;
  SliceType encCABACTableIdx = slice.m_encCABACTableIdx;
  if (!slice.isIntra() && (encCABACTableIdx == B_SLICE || encCABACTableIdx == P_SLICE) &&
      slice.m_pps->m_cabacInitPresentFlag)
  {
    sliceType = encCABACTableIdx;
  }

  if (sliceType == B_SLICE && slice.m_checkLdc)
  {
    sliceType = L_SLICE;
  }

#if ENABLE_CABAC_DUMP
  slice.m_cabacInitSliceType = sliceType;
#endif

  m_binEncoder.reset(qp, (int)sliceType);
  m_binEncoder.setBaseLevel(slice.m_riceBaseLevelValue);
  m_binEncoder.riceStatReset(slice.m_sps->m_bitDepths[ChannelType::LUMA],
                             slice.m_sps->m_spsRangeExtension.m_persistentRiceAdaptationEnabledFlag);
  if (slice.m_sps->m_tempCabacInitMode)
  {
    m_CABACDataStore->loadCtxStates(&slice, getCtx());
  }
}

template<class BinProbModel> SliceType xGetCtxInitId(const Slice &slice, const BinEncIf &binEncoder, Ctx &ctxTest)
{
  const CtxStore<BinProbModel> &ctxStoreTest = static_cast<const CtxStore<BinProbModel> &>(ctxTest);
  const CtxStore<BinProbModel> &ctxStoreRef  = static_cast<const CtxStore<BinProbModel> &>(binEncoder.getCtx());
  int                           qp           = slice.m_iSliceQp;
  if (!slice.isIntra())
  {
    SliceType aSliceTypeChoices[] = { B_SLICE, P_SLICE };
    uint64_t  bestCost            = std::numeric_limits<uint64_t>::max();
    SliceType bestSliceType       = aSliceTypeChoices[0];
    for (uint32_t idx = 0; idx < 2; idx++)
    {
      uint64_t  curCost      = 0;
      SliceType curSliceType = aSliceTypeChoices[idx];
      if (curSliceType == B_SLICE && slice.m_checkLdc)
      {
        ctxTest.init(qp, (int)L_SLICE);
      }
      else
      {
        ctxTest.init(qp, (int)curSliceType);
      }
      for (int k = 0; k < Ctx::NumberOfContexts; k++)
      {
        if (binEncoder.getNumBins(k) > 0)
        {
          curCost += uint64_t(binEncoder.getNumBins(k)) * ctxStoreRef[k].estFracExcessBits(ctxStoreTest[k]);
        }
      }
      if (curCost < bestCost)
      {
        bestSliceType = curSliceType;
        bestCost      = curCost;
      }
    }
    return bestSliceType;
  }
  else
  {
    return I_SLICE;
  }
}

SliceType CABACWriter::getCtxInitId(const Slice &slice)
{
  switch (m_testCtx.getBpmType())
  {
  case BpmType::STD:
    return xGetCtxInitId<BinProbModel_Std>(slice, m_binEncoder, m_testCtx);
  default:
    return NUMBER_OF_SLICE_TYPES;
  }
}

unsigned estBits(BinEncIf &binEnc, const std::vector<bool> &bins, const Ctx &ctx, const int ctxId,
                 const uint8_t winSize)
{
  binEnc.initCtxAndWinSize(ctxId, ctx, winSize);
  binEnc.start();
  const std::size_t numBins   = bins.size();
  unsigned          startBits = binEnc.getNumWrittenBits();
  for (std::size_t binId = 0; binId < numBins; binId++)
  {
    unsigned bin = (bins[binId] ? 1 : 0);
    binEnc.encodeBin(bin, ctxId);
  }
  unsigned endBits   = binEnc.getNumWrittenBits();
  unsigned codedBits = endBits - startBits;
  return codedBits;
}

//================================================================================
//  clause 7.3.8.1
//--------------------------------------------------------------------------------
//    void  end_of_slice()
//================================================================================

void CABACWriter::end_of_slice()
{
  m_binEncoder.encodeBinTrm(1);
  m_binEncoder.finish();
}

void CABACWriter::bif(const CompID compID, const Slice &slice, const BifParams &bifParams)
{
  for (int i = 0; i < bifParams.numBlocks; ++i)
  {
    bif(compID, slice, bifParams, i);
  }
}

void CABACWriter::bif(const CompID compID, const Slice &slice, const BifParams &bifParams, unsigned ctuRsAddr)
{
  const PPS &pps = *slice.m_pps;

  if (isLuma(compID) && !pps.m_BIF)
  {
    return;
  }

  if (isChroma(compID) && !pps.m_chromaBIF)
  {
    return;
  }

  if (ctuRsAddr == 0)
  {
    m_binEncoder.encodeBinEP(bifParams.allCtuOn);
    if (bifParams.allCtuOn == 0)
    {
      m_binEncoder.encodeBinEP(bifParams.frmOn);
    }
  }

  if (bifParams.allCtuOn == 0 && bifParams.frmOn)
  {
    m_binEncoder.encodeBin(bifParams.ctuOn[ctuRsAddr], Ctx::BifCtrlFlags[compID]());
  }
}

//================================================================================
//  clause 7.3.8.2
//--------------------------------------------------------------------------------
//    bool  coding_tree_unit( cs, area, qp, ctuRsAddr, skipSao, skipAlf, skipLfCccm, skipNnlf )
//================================================================================

void CABACWriter::coding_tree_unit(CodingStructure &cs, const UnitArea &area, EnumArray<int, ChannelType> &qps,
                                   unsigned ctuRsAddr, bool skipSao /* = false */, bool skipAlf /* = false */,
                                   bool skipLfCccm /* = false */
#if ENABLE_NNLF
                                   ,
                                   bool skipNnlf /* = false */
#endif
)
{
  CUCtx           cuCtx(qps[ChannelType::LUMA]);
  QTBTPartitioner partitioner;

  partitioner.initCtu(area, ChannelType::LUMA, cs);

#if ENABLE_NNLF
  if (!skipNnlf && cs.sps->m_nnlf && !cs.picHeader->m_nnlfDisabled && ctuRsAddr == 0)
  {
    writeNnlfUnifiedParameters(cs);
  }
#endif

  if (!skipSao)
  {
    sao(*cs.slice, ctuRsAddr);
  }

  if (!skipSao)
  {
    for (int compIdx = 0; compIdx < getNumberValidComponents(cs.pcv->chrFormat); compIdx++)
    {
      if (cs.slice->m_ccSaoComParam.enabled[compIdx])
      {
        const int setNum = cs.slice->m_ccSaoComParam.setNum[compIdx];

        const int      ry = ctuRsAddr / cs.pcv->widthInCtus;
        const int      rx = ctuRsAddr % cs.pcv->widthInCtus;
        const Position lumaPos(rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight);
        codeCcSaoControlIdc(cs.slice->m_ccSaoControl[compIdx][ctuRsAddr], cs, CompID(compIdx), ctuRsAddr,
                            cs.slice->m_ccSaoControl[compIdx], lumaPos, setNum);
      }
    }
  }

  if (!skipLfCccm)
  {
    lfCccm(cs, ctuRsAddr);
  }

  if (!skipAlf)
  {
    for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
    {
      if (!cs.slice->m_alfEnabledFlag[static_cast<CompID>(compIdx)])
      {
        continue;
      }

      codeAlfCtuEnableFlag(cs, ctuRsAddr, compIdx, nullptr);

      if (isLuma(CompID(compIdx)))
      {
        codeAlfCtuFilterIndex(cs, ctuRsAddr, true);

        if (cs.sps->m_alfImprovementsEnabledFlag)
        {
          codeAlfCtuLumaAlternative(cs, ctuRsAddr);
        }
      }
      if (isChroma(CompID(compIdx)))
      {
        codeAlfCtuChromaAlternative(cs, ctuRsAddr, static_cast<CompID>(compIdx));
      }
    }
  }

  if (!skipAlf && cs.sps->m_ccalfEnabledFlag)
  {
    const AlfParameters::CcAlfFilterParamBase &sliceCcAlfFilterParam = cs.slice->m_ccAlfFilterParam.getParam();
    for (int compIdx = 1; compIdx < getNumberValidComponents(cs.pcv->chrFormat); compIdx++)
    {
      if (sliceCcAlfFilterParam.ccAlfFilterEnabled[compIdx - 1])
      {
        const int filterCount = sliceCcAlfFilterParam.ccAlfFilterCount[compIdx - 1];

        const int      ry = ctuRsAddr / cs.pcv->widthInCtus;
        const int      rx = ctuRsAddr % cs.pcv->widthInCtus;
        const Position lumaPos(rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight);

        codeCcAlfFilterControlIdc(cs.slice->m_ccAlfFilterControl[compIdx - 1][ctuRsAddr], cs, CompID(compIdx),
                                  ctuRsAddr, cs.slice->m_ccAlfFilterControl[compIdx - 1], lumaPos, filterCount);
      }
    }
  }

  if (CS::isDualITree(cs) && isChromaEnabled(cs.pcv->chrFormat) &&
      cs.pcv->maxCUWidth > std::min<int>(MAX_TB_SIZEY, MAX_INTRA_SIZE))
  {
    CUCtx           chromaCuCtx(qps[ChannelType::CHROMA]);
    QTBTPartitioner chromaPartitioner;
    chromaPartitioner.initCtu(area, ChannelType::CHROMA, cs);
    coding_tree(cs, partitioner, cuCtx, &chromaPartitioner, &chromaCuCtx);
    qps[ChannelType::LUMA]   = cuCtx.qp;
    qps[ChannelType::CHROMA] = chromaCuCtx.qp;
  }
  else
  {
    coding_tree(cs, partitioner, cuCtx);
    qps[ChannelType::LUMA] = cuCtx.qp;
    if (CS::isDualITree(cs) && isChromaEnabled(cs.pcv->chrFormat))
    {
      CUCtx cuCtxChroma(qps[ChannelType::CHROMA]);
      partitioner.initCtu(area, ChannelType::CHROMA, cs);
      coding_tree(cs, partitioner, cuCtxChroma);
      qps[ChannelType::CHROMA] = cuCtxChroma.qp;
    }
  }
}

//================================================================================
//  clause 7.3.8.3
//--------------------------------------------------------------------------------
//    void  sao             ( slice, ctuRsAddr )
//    void  sao_block_params  ( saoPars, bitDepths, sliceEnabled, leftMergeAvail, aboveMergeAvail, onlyEstMergeInfo )
//    void  sao_offset_params ( ctbPars, compID, sliceEnabled, bitDepth )
//================================================================================

void CABACWriter::sao(const Slice &slice, unsigned ctuRsAddr)
{
  const SPS &sps = *slice.m_sps;
  if (!sps.m_saoEnabledFlag)
  {
    return;
  }

  CodingStructure     &cs           = *slice.m_pic->m_cs;
  const PreCalcValues &pcv          = *cs.pcv;
  const SAOBlkParam   &saoCtuParams = cs.picture->getSAO()[ctuRsAddr];

  const bool sliceSaoLumaFlag   = slice.m_saoEnabledFlag[ChannelType::LUMA];
  const bool sliceSaoChromaFlag = slice.m_saoEnabledFlag[ChannelType::CHROMA] && isChromaEnabled(sps.m_chromaFormatIdc);

  if (!sliceSaoLumaFlag && !sliceSaoChromaFlag)
  {
    return;
  }

  const bool sliceEnabled[3] = { sliceSaoLumaFlag, sliceSaoChromaFlag, sliceSaoChromaFlag };

  const int frameWidthInCtus = pcv.widthInCtus;

  const int ry = ctuRsAddr / frameWidthInCtus;
  const int rx = ctuRsAddr - ry * frameWidthInCtus;

  const Position pos(rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight);

  const unsigned curSliceIdx = slice.m_independentSliceIdx;
  const unsigned curTileIdx  = cs.pps->getTileIdx(pos);

  const bool leftMergeAvail =
    cs.getCURestricted(pos.offset(-(int)pcv.maxCUWidth, 0), pos, curSliceIdx, curTileIdx, ChannelType::LUMA) != nullptr;
  const bool aboveMergeAvail = cs.getCURestricted(pos.offset(0, -(int)pcv.maxCUHeight), pos, curSliceIdx, curTileIdx,
                                                  ChannelType::LUMA) != nullptr;

  sao_block_params(saoCtuParams, sps.m_bitDepths, sliceEnabled, leftMergeAvail, aboveMergeAvail, false);
}

void CABACWriter::sao_block_params(const SAOBlkParam &saoPars, const BitDepths &bitDepths, const bool *sliceEnabled,
                                   bool leftMergeAvail, bool aboveMergeAvail, bool onlyEstMergeInfo)
{
  bool isLeftMerge  = false;
  bool isAboveMerge = false;
  if (leftMergeAvail)
  {
    // sao_merge_left_flag
    isLeftMerge =
      (saoPars[COMP_Y].modeIdc == SAOMode::MERGE && saoPars[COMP_Y].typeIdc.mergeType == SAOModeMergeTypes::LEFT);
    m_binEncoder.encodeBin((isLeftMerge), Ctx::SaoMergeFlag());
  }
  if (aboveMergeAvail && !isLeftMerge)
  {
    // sao_merge_above_flag
    isAboveMerge =
      (saoPars[COMP_Y].modeIdc == SAOMode::MERGE && saoPars[COMP_Y].typeIdc.mergeType == SAOModeMergeTypes::ABOVE);
    m_binEncoder.encodeBin((isAboveMerge), Ctx::SaoMergeFlag());
  }
  if (onlyEstMergeInfo)
  {
    return; // only for RDO
  }
  if (!isLeftMerge && !isAboveMerge)
  {
    // explicit parameters
    for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
    {
      sao_offset_params(saoPars[compIdx], CompID(compIdx), sliceEnabled[compIdx],
                        bitDepths[toChannelType(CompID(compIdx))]);
    }
  }
}

void CABACWriter::sao_offset_params(const SAOOffset &ctbPars, CompID compID, bool sliceEnabled, int bitDepth)
{
  if (!sliceEnabled)
  {
    CHECK(ctbPars.modeIdc != SAOMode::OFF, "Sao must be off, if it is disabled on slice level");
    return;
  }
  const bool isFirstCompOfChType = (getFirstComponentOfChannel(toChannelType(compID)) == compID);

  if (isFirstCompOfChType)
  {
    // sao_type_idx_luma / sao_type_idx_chroma
    if (ctbPars.modeIdc == SAOMode::OFF)
    {
      m_binEncoder.encodeBin(0, Ctx::SaoTypeIdx());
    }
    else if (ctbPars.typeIdc.newType == SAOModeNewTypes::BO)
    {
      m_binEncoder.encodeBin(1, Ctx::SaoTypeIdx());
      m_binEncoder.encodeBinEP(0);
    }
    else
    {
      CHECK(!(ctbPars.typeIdc.newType < SAOModeNewTypes::START_BO), "Unspecified error");
      m_binEncoder.encodeBin(1, Ctx::SaoTypeIdx());
      m_binEncoder.encodeBinEP(1);
    }
  }

  if (ctbPars.modeIdc == SAOMode::NEW)
  {
    const int maxOffsetQVal = SampleAdaptiveOffset::getMaxOffsetQVal(bitDepth);
    int       numClasses    = (ctbPars.typeIdc.newType == SAOModeNewTypes::BO ? 4 : NUM_SAO_EO_CLASSES);
    int       k             = 0;
    int       offset[4];
    for (int i = 0; i < numClasses; i++)
    {
      if (ctbPars.typeIdc.newType != SAOModeNewTypes::BO && i == SAO_CLASS_EO_PLAIN)
      {
        continue;
      }
      int classIdx =
        (ctbPars.typeIdc.newType == SAOModeNewTypes::BO ? (ctbPars.typeAuxInfo + i) % NUM_SAO_BO_CLASSES : i);
      offset[k++] = ctbPars.offset[classIdx];
    }

    // sao_offset_abs
    for (int i = 0; i < 4; i++)
    {
      unsigned absOffset = (offset[i] < 0 ? -offset[i] : offset[i]);
      unary_max_eqprob(absOffset, maxOffsetQVal);
    }

    // band offset mode
    if (ctbPars.typeIdc.newType == SAOModeNewTypes::BO)
    {
      // sao_offset_sign
      for (int i = 0; i < 4; i++)
      {
        if (offset[i])
        {
          m_binEncoder.encodeBinEP((offset[i] < 0));
        }
      }
      // sao_band_position
      m_binEncoder.encodeBinsEP(ctbPars.typeAuxInfo, NUM_SAO_BO_CLASSES_LOG2);
    }
    // edge offset mode
    else
    {
      if (isFirstCompOfChType)
      {
        // sao_eo_class_luma / sao_eo_class_chroma
        CHECK(ctbPars.typeIdc.newType < SAOModeNewTypes::START_EO, "sao edge offset class is outside valid range");
        m_binEncoder.encodeBinsEP(to_underlying(ctbPars.typeIdc.newType) - to_underlying(SAOModeNewTypes::START_EO),
                                  NUM_SAO_EO_TYPES_LOG2);
      }
    }
  }
}

void CABACWriter::codeCcSaoControlIdc(uint8_t idcVal, CodingStructure &cs, const CompID compID, const int curIdx,
                                      const uint8_t *controlIdc, Position lumaPos, const int setNum)
{
  CHECK(idcVal > setNum, "Set index is too large");

  const uint32_t curSliceIdx  = cs.slice->m_independentSliceIdx;
  const uint32_t curTileIdx   = cs.pps->getTileIdx(lumaPos);
  Position       leftLumaPos  = lumaPos.offset(-(int)cs.pcv->maxCUWidth, 0);
  Position       aboveLumaPos = lumaPos.offset(0, -(int)cs.pcv->maxCUWidth);
  bool leftAvail = cs.getCURestricted(leftLumaPos, lumaPos, curSliceIdx, curTileIdx, ChannelType::LUMA) ? true : false;
  bool aboveAvail =
    cs.getCURestricted(aboveLumaPos, lumaPos, curSliceIdx, curTileIdx, ChannelType::LUMA) ? true : false;
  int ctxt = 0;

  if (leftAvail)
  {
    ctxt += (controlIdc[curIdx - 1]) ? 1 : 0;
  }
  if (aboveAvail)
  {
    ctxt += (controlIdc[curIdx - cs.pcv->widthInCtus]) ? 1 : 0;
  }
  ctxt += (compID == COMP_Y) ? 0 : (compID == COMP_Cb) ? 3 : 6;
  m_binEncoder.encodeBin((idcVal == 0) ? 0 : 1, Ctx::CcSaoControlIdc(ctxt));   // ON/OFF flag is context coded

  if (idcVal > 0)
  {
    int val = (idcVal - 1);
    while (val)
    {
      m_binEncoder.encodeBinEP(1);
      val--;
    }
    if (idcVal < setNum)
    {
      m_binEncoder.encodeBinEP(0);
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "cc_sao_control_idc() compID=%d pos=(%d,%d) ctxt=%d, setNum=%d, idcVal=%d\n", compID,
         lumaPos.x, lumaPos.y, ctxt, setNum, idcVal);
}

//================================================================================
//  clause 7.3.8.4
//--------------------------------------------------------------------------------
//    void  coding_tree       ( cs, partitioner, cuCtx )
//    void  split_cu_flag     ( split, cs, partitioner )
//    void  split_cu_mode_mt  ( split, cs, partitioner )
//================================================================================

void CABACWriter::coding_tree(const CodingStructure &cs, Partitioner &partitioner, CUCtx &cuCtx,
                              Partitioner *pPartitionerChroma, CUCtx *pCuCtxChroma)
{
  const PPS        &pps      = *cs.pps;
  const UnitArea   &currArea = partitioner.currArea();
  const CodingUnit &cu       = *cs.getCU(currArea.block(partitioner.chType), partitioner.chType);

  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
  // Note: do not reset qg at chroma CU
  if (pps.m_useDQP && partitioner.currQgEnable())
  {
    cuCtx.qgStart    = true;
    cuCtx.isDQPCoded = false;
  }
  if (cs.slice->m_chromaQpAdjEnabled && partitioner.currQgChromaEnable())
  {
    cuCtx.isChromaQpAdjCoded = false;
  }
  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
  if (CS::isDualITree(cs) && pPartitionerChroma != nullptr)
  {
    if (pps.m_useDQP && pPartitionerChroma->currQgEnable())
    {
      pCuCtxChroma->qgStart    = true;
      pCuCtxChroma->isDQPCoded = false;
    }
    if (cs.slice->m_chromaQpAdjEnabled && pPartitionerChroma->currQgChromaEnable())
    {
      pCuCtxChroma->isChromaQpAdjCoded = false;
    }
  }

  DeriveCtx::setNeighbourCus(cs, partitioner.currArea(), partitioner.chType);

  const PartSplit splitMode = CU::getSplitAtDepth(cu, partitioner.currDepth);

  split_cu_mode(splitMode, cs, partitioner);

  CHECK(!partitioner.canSplit(splitMode, cs), "The chosen split mode is invalid!");

  if (splitMode != CU_DONT_SPLIT)
  {
    const int maxSize = std::min<int>(MAX_TB_SIZEY, MAX_INTRA_SIZE);

    if (CS::isDualITree(cs) && pPartitionerChroma != nullptr &&
        (partitioner.currArea().lwidth() >= maxSize || partitioner.currArea().lheight() >= maxSize))
    {
      partitioner.splitCurrArea(CU_QUAD_SPLIT, cs);
      pPartitionerChroma->splitCurrArea(CU_QUAD_SPLIT, cs);
      bool beContinue     = true;
      bool lumaContinue   = true;
      bool chromaContinue = true;

      while (beContinue)
      {
        if (partitioner.currArea().lwidth() > maxSize || partitioner.currArea().lheight() > maxSize)
        {
          if (cs.picture->block(partitioner.chType).contains(partitioner.currArea().block(partitioner.chType).pos()))
          {
            coding_tree(cs, partitioner, cuCtx, pPartitionerChroma, pCuCtxChroma);
          }
          lumaContinue   = partitioner.nextPart(cs);
          chromaContinue = pPartitionerChroma->nextPart(cs);
          CHECK(lumaContinue != chromaContinue, "luma chroma partition should be matched");
          beContinue = lumaContinue;
        }
        else
        {
          // dual tree coding under 64x64 block
          if (cs.picture->block(partitioner.chType).contains(partitioner.currArea().block(partitioner.chType).pos()))
          {
            coding_tree(cs, partitioner, cuCtx);
          }
          lumaContinue = partitioner.nextPart(cs);
          if (cs.picture->block(pPartitionerChroma->chType)
                .contains(pPartitionerChroma->currArea().block(pPartitionerChroma->chType).pos()))
          {
            coding_tree(cs, *pPartitionerChroma, *pCuCtxChroma);
          }
          chromaContinue = pPartitionerChroma->nextPart(cs);
          CHECK(lumaContinue != chromaContinue, "luma chroma partition should be matched");
          beContinue = lumaContinue;
        }
      }
      partitioner.exitCurrSplit();
      pPartitionerChroma->exitCurrSplit();
    }
    else
    {
      partitioner.splitCurrArea(splitMode, cs);

      do
      {
        if (cs.picture->block(partitioner.chType).contains(partitioner.currArea().block(partitioner.chType).pos()))
        {
          coding_tree(cs, partitioner, cuCtx);
        }
      } while (partitioner.nextPart(cs));

      partitioner.exitCurrSplit();
    }
    return;
  }

  // Predict QP on start of quantization group
  if (cuCtx.qgStart)
  {
    cuCtx.qgStart = false;
    cuCtx.qp      = CU::predictQP(cu, cuCtx.qp);
  }

  if (m_binEncoder
        .getBinBuffer()) // if context data collection is active and if on bottom of the CTU, start the counters
  {
    m_binEncoder.setBinBufferActive(CU::isOnCtuBottom(cu));
  }

  // coding unit
  coding_unit(cu, partitioner, cuCtx);

  if (m_binEncoder.getBinBuffer()) // done with the data collection for this CU
  {
    m_binEncoder.setBinBufferActive(false);
  }

  DTRACE_COND((isEncoding()), g_trace_ctx, D_QP, "x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.lx(), cu.ly(), cu.lwidth(),
              cu.lheight(), cu.qp);
  DTRACE_BLOCK_REC_COND((!isEncoding()), cs.picture->getRecoBuf(cu), cu, cu.predMode);
}

void CABACWriter::split_cu_mode(const PartSplit split, const CodingStructure &cs, Partitioner &partitioner)
{
  bool canNo, canQt, canBh, canBv, canTh, canTv;
  partitioner.canSplit(cs, canNo, canQt, canBh, canBv, canTh, canTv);

  bool canSpl[6] = { canNo, canQt, canBh, canBv, canTh, canTv };

  unsigned ctxSplit = 0, ctxQtSplit = 0, ctxBttHV = 0, ctxBttH12 = 0, ctxBttV12;
  DeriveCtx::CtxSplit(cs, partitioner, ctxSplit, ctxQtSplit, ctxBttHV, ctxBttH12, ctxBttV12, canSpl);

  bool       canSplit = canBh || canBv || canTh || canTv || canQt;
  const bool isNo     = split == CU_DONT_SPLIT;
  const bool canBtt   = canBh || canBv || canTh || canTv;
  const bool isQt     = split == CU_QUAD_SPLIT;
  bool       qtFrst   = false;

  if (canNo && canSplit)
  {
    bool      colSplitPredExist = false;
    SplitPred currentSplitPred;

    const Position centerPos = partitioner.currArea().Y().center();

    Picture *pColPic =
      cs.slice->getRefPic(RefPicList(cs.slice->isInterB() ? 1 - cs.slice->m_colFromL0Flag : 0), cs.slice->m_colRefIdx);

    if (!cs.slice->isIntra() && pColPic && !pColPic->isRefScaled(cs.pps) && pColPic->m_cs->slice &&
        pColPic->m_cs->area.Y().contains(centerPos) && isLuma(partitioner.chType) &&
        cs.slice->m_sps->m_tempPartPredEnabledFlag)
    {
      currentSplitPred  = pColPic->m_cs->getQtDepthInfo(centerPos);
      colSplitPredExist = true;
    }

    if (colSplitPredExist && (partitioner.currQtDepth < currentSplitPred.qtDepth))
    {
      qtFrst = true;

      if (canQt && canBtt)
      {
        m_binEncoder.encodeBin(isQt, Ctx::SplitQtFlag(ctxQtSplit));
        if (isQt)
        {
          return;
        }
        else
        {
          canSplit = canBtt;
        }
      }
    }

    if (canNo && canSplit)
    {
      m_binEncoder.encodeBin(!isNo, Ctx::SplitFlag(ctxSplit));
    }
  }

  DTRACE(g_trace_ctx, D_SYNTAX, "split_cu_mode() depth=%d ctx=%d split=%d\n", partitioner.currDepth, ctxSplit, !isNo);

  if (isNo)
  {
    return;
  }

  if (canQt && canBtt && !qtFrst)
  {
    m_binEncoder.encodeBin(isQt, Ctx::SplitQtFlag(ctxQtSplit));
  }

  DTRACE(g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d qt=%d\n", ctxQtSplit, isQt);

  if (isQt)
  {
    return;
  }

  const bool canHor = canBh || canTh;
  const bool canVer = canBv || canTv;
  const bool isVer  = split == CU_VERT_SPLIT || split == CU_TRIV_SPLIT;

  if (canVer && canHor)
  {
    m_binEncoder.encodeBin(isVer, Ctx::SplitHvFlag(ctxBttHV));
  }

  const bool can14 = isVer ? canTv : canTh;
  const bool can12 = isVer ? canBv : canBh;
  const bool is12  = isVer ? (split == CU_VERT_SPLIT) : (split == CU_HORZ_SPLIT);

  if (can12 && can14)
  {
    m_binEncoder.encodeBin(is12, Ctx::Split12Flag(isVer ? ctxBttV12 : ctxBttH12));
  }

  DTRACE(g_trace_ctx, D_SYNTAX, "split_cu_mode() ctxHv=%d ctx12=%d mode=%d\n", ctxBttHV, isVer ? ctxBttV12 : ctxBttH12,
         split);
}

//================================================================================
//  clause 7.3.8.5
//--------------------------------------------------------------------------------
//    void  coding_unit               ( cu, partitioner, cuCtx )
//    void  cu_skip_flag              ( cu )
//    void  pred_mode                 ( cu )
//    void  part_mode                 ( cu )
//    void  cu_pred_data              ( pus )
//    void  cu_lic_flag               ( cu )
//    void  intra_luma_pred_mode      ( cu )
//    void  intra_chroma_pred_mode    ( cu )
//    void  cu_residual               ( cu, partitioner, cuCtx )
//    void  rqt_root_cbf              ( cu )
//    void  end_of_ctu                ( cu, cuCtx )
//================================================================================

void CABACWriter::coding_unit(const CodingUnit &cu, Partitioner &partitioner, CUCtx &cuCtx)
{
  DTRACE(g_trace_ctx, D_SYNTAX, "coding_unit() %dx%d @ (%d,%d)\n", cu.blocks[int(cu.chType)].width,
         cu.blocks[int(cu.chType)].height, cu.blocks[int(cu.chType)].x, cu.blocks[int(cu.chType)].y);
  CodingStructure &cs = *cu.cs;

  // skip flag
  if ((!cs.slice->isIntra() || cs.slice->m_ibcFlag) && cu.Y().valid())
  {
    cu_skip_flag(cu);
  }

  // skip data
  if (cu.skip)
  {
    CHECK(!cu.mergeFlag, "Merge flag has to be on!");
    CodingUnit &mcu = const_cast<CodingUnit &>(cu);
    prediction_unit(mcu);
    end_of_ctu(cu, cuCtx);
    return;
  }

  // prediction mode and partitioning data
  pred_mode(cu);
  if (CU::isPLT(cu))
  {
    if (CS::isDualITree(cs))
    {
      if (isLuma(partitioner.chType))
      {
        cu_palette_info(cu, COMP_Y, 1, cuCtx);
      }
      if (isChromaEnabled(cu.chromaFormat) && partitioner.chType == ChannelType::CHROMA)
      {
        cu_palette_info(cu, COMP_Cb, 2, cuCtx);
      }
    }
    else
    {
      cu_palette_info(cu, COMP_Y, getNumberValidComponents(cu.chromaFormat), cuCtx);
    }
    end_of_ctu(cu, cuCtx);
    return;
  }

  // prediction data ( intra prediction modes / reference indexes + motion vectors )
  cu_pred_data(cu);

  // residual data ( coded block flags + transform coefficient levels )
  cu_residual(cu, partitioner, cuCtx);

  // end of cu
  end_of_ctu(cu, cuCtx);
}

void CABACWriter::cu_skip_flag(const CodingUnit &cu)
{
  unsigned ctxId = DeriveCtx::CtxSkipFlag(cu);

  if (cu.slice->isIntra() && cu.cs->slice->m_ibcFlag)
  {
    if (!cu.slice->m_sps->m_ibcMerge)
    {
      return;
    }
    if (cu.lwidth() <= IBC_MAX_CU_SIZE && cu.lheight() <= IBC_MAX_CU_SIZE)   // disable IBC mode larger than 64x64
    {
      m_binEncoder.encodeBin((cu.skip), Ctx::SkipFlag(ctxId));
      DTRACE(g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, cu.skip ? 1 : 0);
    }
    else
    {
      DTRACE(g_trace_ctx, D_SYNTAX, "cu_skip_flag() skip=%d (uncoded)\n", cu.skip ? 1 : 0);
    }
    return;
  }
  m_binEncoder.encodeBin((cu.skip), Ctx::SkipFlag(ctxId));

  DTRACE(g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, cu.skip ? 1 : 0);
  if (cu.skip && cu.cs->slice->m_ibcFlag)
  {
    // disable IBC mode larger than 64x64 and disable IBC when only allowing inter mode
    if (!cu.slice->m_sps->m_ibcMerge)
    {
      CHECK(CU::isIBC(cu), "IBC skip shall not be used with IBC merge disabled");
    }
    else if (cu.lwidth() <= IBC_MAX_CU_SIZE && cu.lheight() <= IBC_MAX_CU_SIZE)
    {
      unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
      m_binEncoder.encodeBin(CU::isIBC(cu) ? 1 : 0, Ctx::IBCFlag(ctxidx));
      DTRACE(g_trace_ctx, D_SYNTAX, "ibc() ctx=%d cu.predMode=%d\n", ctxidx, cu.predMode);
    }
  }
}

void CABACWriter::pred_mode(const CodingUnit &cu)
{
  if (cu.cs->slice->m_ibcFlag && cu.chType != ChannelType::CHROMA)
  {
    if (cu.cs->slice->isIntra())
    {
      if (cu.lwidth() <= IBC_MAX_CU_SIZE && cu.lheight() <= IBC_MAX_CU_SIZE)
      {
        unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
        m_binEncoder.encodeBin(CU::isIBC(cu), Ctx::IBCFlag(ctxidx));
      }
      if (!CU::isIBC(cu) && CU::pltAllowed(cu))
      {
        m_binEncoder.encodeBin(CU::isPLT(cu), Ctx::PLTFlag(0));
      }
    }
    else
    {
      m_binEncoder.encodeBin((CU::isIntra(cu) || CU::isPLT(cu)), Ctx::PredMode(DeriveCtx::CtxPredModeFlag(cu)));
      if (CU::isIntra(cu) || CU::isPLT(cu))
      {
        if (CU::pltAllowed(cu))
        {
          m_binEncoder.encodeBin(CU::isPLT(cu), Ctx::PLTFlag(0));
        }
      }
      else
      {
        if (cu.lwidth() <= IBC_MAX_CU_SIZE && cu.lheight() <= IBC_MAX_CU_SIZE)   // disable IBC mode larger than 64x64
        {
          unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
          m_binEncoder.encodeBin(CU::isIBC(cu), Ctx::IBCFlag(ctxidx));
        }
      }
    }
  }
  else
  {
    if (cu.cs->slice->isIntra())
    {
      if (CU::pltAllowed(cu))
      {
        m_binEncoder.encodeBin((CU::isPLT(cu)), Ctx::PLTFlag(0));
      }
    }
    else
    {
      m_binEncoder.encodeBin((CU::isIntra(cu) || CU::isPLT(cu)), Ctx::PredMode(DeriveCtx::CtxPredModeFlag(cu)));
      if ((CU::isIntra(cu) || CU::isPLT(cu)) && CU::pltAllowed(cu))
      {
        m_binEncoder.encodeBin((CU::isPLT(cu)), Ctx::PLTFlag(0));
      }
    }
  }

  if (cu.Y().valid())
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "pred_mode(%d) x=%d, y=%d, w=%d, h=%d, mode=%d\n", ChannelType::LUMA, cu.lumaPos().x,
           cu.lumaPos().y, cu.lumaSize().width, cu.lumaSize().height, cu.predMode);
  }
  else
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "pred_mode(%d) x=%d, y=%d, w=%d, h=%d, mode=%d\n", ChannelType::CHROMA,
           cu.chromaPos().x, cu.chromaPos().y, cu.chromaSize().width, cu.chromaSize().height, cu.predMode);
  }
}

void CABACWriter::bdpcm_mode(const CodingUnit &cu, const CompID compID)
{
  if (!cu.cs->sps->m_bdpcmEnabledFlag || !CU::bdpcmAllowed(cu, compID))
  {
    return;
  }

  const BdpcmMode bdpcmMode = cu.getBdpcmMode(compID);

  unsigned ctxId = isLuma(compID) ? 0 : 2;
  m_binEncoder.encodeBin(bdpcmMode != BdpcmMode::NONE ? 1 : 0, Ctx::BDPCMMode(ctxId));

  if (bdpcmMode != BdpcmMode::NONE)
  {
    m_binEncoder.encodeBin(bdpcmMode != BdpcmMode::HOR ? 1 : 0, Ctx::BDPCMMode(ctxId + 1));
  }
  if (isLuma(compID))
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "bdpcm_mode(%d) x=%d, y=%d, w=%d, h=%d, bdpcm=%d\n", ChannelType::LUMA,
           cu.lumaPos().x, cu.lumaPos().y, cu.lwidth(), cu.lheight(), cu.bdpcmMode[0]);
  }
  else
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "bdpcm_mode(%d) x=%d, y=%d, w=%d, h=%d, bdpcm=%d\n", ChannelType::CHROMA,
           cu.chromaPos().x, cu.chromaPos().y, cu.chromaSize().width, cu.chromaSize().height, cu.bdpcmMode[1]);
  }
}

void CABACWriter::cu_pred_data_intra(const CodingUnit &cu, const CUCtxIntra &cuCtxIntra)
{
  CHECKD(!CU::isIntra(cu), "Not expected");
  if (cu.Y().valid())
  {
    bdpcm_mode(cu, COMP_Y);
  }

  intra_luma_pred_mode(cu, cuCtxIntra);
  if ((!cu.Y().valid() || (!CS::isDualITree(*cu.cs) && cu.Y().valid())) && isChromaEnabled(cu.chromaFormat))
  {
    bdpcm_mode(cu, CompID(ChannelType::CHROMA));
  }
  intra_chroma_pred_mode(cu);
}

void CABACWriter::cu_pred_data(const CodingUnit &cu)
{
  if (CU::isIntra(cu))
  {
    CUCtxIntra cuCtxIntra;
    cuCtxIntra.mpmListSize = PU::getIntraMPMs(cu, cuCtxIntra.mpmList, cuCtxIntra.nonMPMList);
    cu_pred_data_intra(cu, cuCtxIntra);
    return;
  }
  if (!cu.Y().valid())   // dual tree chroma CU
  {
    return;
  }

  prediction_unit(cu);

  imv_mode(cu);
  affine_amvr_mode(cu);
  cu_lic_flag(cu);
  cu_bcw_flag(cu);
  obmc_flag(cu);
}

void CABACWriter::cu_bcw_flag(const CodingUnit &cu)
{
  if (!CU::isBcwIdxCoded(cu))
  {
    return;
  }

  CHECK(!(BCW_NUM > 1 && (BCW_NUM == 2 || (BCW_NUM & 0x01) == 1)),
        " !( BCW_NUM > 1 && ( BCW_NUM == 2 || ( BCW_NUM & 0x01 ) == 1 ) ) ");
  const uint8_t bcwCodingIdx = (uint8_t)g_BcwCodingOrder[CU::getValidBcwIdx(cu)];

  const int32_t numBcw = (cu.slice->m_checkLdc) ? 5 : 3;
  m_binEncoder.encodeBin((bcwCodingIdx == 0 ? 0 : 1), Ctx::bcwIdx(0));
  if (numBcw > 2 && bcwCodingIdx != 0)
  {
    const uint32_t prefixNumBits = numBcw - 2;
    const uint32_t step          = 1;

    uint8_t idx = 1;
    for (int ui = 0; ui < prefixNumBits; ++ui)
    {
      if (bcwCodingIdx == idx)
      {
        m_binEncoder.encodeBinEP(0);
        break;
      }
      else
      {
        m_binEncoder.encodeBinEP(1);
        idx += step;
      }
    }
  }

  DTRACE(g_trace_ctx, D_SYNTAX, "cu_bcw_flag() bcw_idx=%d\n", cu.bcwIdx ? 1 : 0);
}

void CABACWriter::obmc_flag(const CodingUnit &cu)
{
  if (!CU::isObmcAllowed(cu))
  {
    CHECKD(cu.obmcFlag == true, "obmc flag set but not allowed");
    return;
  }
  if (cu.mergeFlag)
  {
    return;
  }

  const int ctxId = cu.affine ? 1 : 0;
  m_binEncoder.encodeBin(cu.obmcFlag, Ctx::ObmcFlag(ctxId));
  DTRACE(g_trace_ctx, D_SYNTAX, "obmc_flag() pos=(%d,%d) size=(%d,%d) obmcFlag=%d\n", cu.lx(), cu.ly(),
         cu.lumaSize().width, cu.lumaSize().height, cu.obmcFlag);
}

void CABACWriter::xWriteTruncBinCode(const uint32_t symbol, const uint32_t numSymbols)
{
  CHECKD(symbol >= numSymbols, "symbol must be less than numSymbols");

  const int thresh = floorLog2(numSymbols);

  const int val = 1 << thresh;

  const int b = numSymbols - val;

  if (symbol < val - b)
  {
    m_binEncoder.encodeBinsEP(symbol, thresh);
  }
  else
  {
    m_binEncoder.encodeBinsEP(symbol + val - b, thresh + 1);
  }
}

void CABACWriter::cclmDelta(const int8_t delta)
{
  if (delta)
  {
    int deltaAbs = abs(delta);

    if (deltaAbs == 1)
    {
      m_binEncoder.encodeBinsEP(0, 1);
    }
    else if (deltaAbs == 2)
    {
      m_binEncoder.encodeBinsEP(2, 2);
    }
    else if (deltaAbs == 3)
    {
      m_binEncoder.encodeBinsEP(6, 3);
    }
    else
    {
      m_binEncoder.encodeBinsEP(7, 3);
    }

    m_binEncoder.encodeBin(delta < 0 ? 1 : 0, Ctx::CclmDeltaFlags(4));
  }
}

void CABACWriter::cclmDeltaSlope(const CodingUnit &cu)
{
  if (cu.cccmFlag)
  {
    return;
  }

  if (PU::hasCclmDeltaFlag(cu))
  {
    const int  chrMode     = cu.intraDir[ChannelType::CHROMA];
    const bool deltaActive = cu.cclmOffsets.isActive();

    m_binEncoder.encodeBin(deltaActive ? 1 : 0, Ctx::CclmDeltaFlags(0));

    if (deltaActive)
    {
      const bool bothActive = cu.cclmOffsets.cb0 && cu.cclmOffsets.cr0;

      m_binEncoder.encodeBin(bothActive ? 1 : 0, Ctx::CclmDeltaFlags(3));

      if (!bothActive)
      {
        m_binEncoder.encodeBin(cu.cclmOffsets.cb0 ? 1 : 0, Ctx::CclmDeltaFlags(1));

        if (PU::isMultiModeLM(chrMode) && !cu.cclmOffsets.cb0)
        {
          m_binEncoder.encodeBin(cu.cclmOffsets.cr0 ? 1 : 0, Ctx::CclmDeltaFlags(2));
        }
      }

      cclmDelta(cu.cclmOffsets.cb0);
      cclmDelta(cu.cclmOffsets.cr0);

      // Now the same for the second model (if applicable)
      if (PU::isMultiModeLM(chrMode))
      {
        const bool bothActive = cu.cclmOffsets.cb1 && cu.cclmOffsets.cr1;

        m_binEncoder.encodeBin(bothActive ? 1 : 0, Ctx::CclmDeltaFlags(3));

        if (!bothActive)
        {
          m_binEncoder.encodeBin(cu.cclmOffsets.cb1 ? 1 : 0, Ctx::CclmDeltaFlags(1));

          if (!cu.cclmOffsets.cb1)
          {
            if (cu.cclmOffsets.cb0 || cu.cclmOffsets.cr0 || cu.cclmOffsets.cb1)
            {
              m_binEncoder.encodeBin(cu.cclmOffsets.cr1 ? 1 : 0, Ctx::CclmDeltaFlags(2));
            }
          }
        }

        cclmDelta(cu.cclmOffsets.cb1);
        cclmDelta(cu.cclmOffsets.cr1);
      }
    }
  }
}

void CABACWriter::extend_ref_line(const CodingUnit &cu)
{
  if (!cu.Y().valid() || !CU::isIntra(cu) || !isLuma(cu.chType) || cu.bdpcmMode[0] != BdpcmMode::NONE || cu.sgpm)
  {
    return;
  }
  if (!cu.cs->sps->m_useMRL)
  {
    return;
  }

  int multiRefIdx = cu.multiRefIdx;
  if (MRL_NUM_REF_LINES > 1)
  {
    m_binEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[0], Ctx::MultiRefLineIdx(0));
    if (MRL_NUM_REF_LINES > 2 && multiRefIdx != MULTI_REF_LINE_IDX[0])
    {
      m_binEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[1], Ctx::MultiRefLineIdx(1));
      if (MRL_NUM_REF_LINES > 3 && multiRefIdx != MULTI_REF_LINE_IDX[1])
      {
        int mrlIdxToPos = MULTI_REF_LINE_2_IDX[multiRefIdx];
        xWriteTruncBinCode(mrlIdxToPos - 2, (int)MRL_NUM_REF_LINES - 2);
      }
    }
  }
}

void CABACWriter::sgpm_flag(const CodingUnit &cu)
{
  if (!CU::isSgpmCoded(cu))
  {
    return;
  }

  if (cu.dimdFlag || cu.timdFlag || cu.mipFlag /*|| cu.tmpFlag*/)
  {
    return;
  }
  if (!cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType))
  {
    return;
  }

  unsigned ctxId = DeriveCtx::CtxSgpmFlag(cu);
  m_binEncoder.encodeBin(cu.sgpm, Ctx::SgpmFlag(ctxId));

  if (cu.sgpm)
  {
    xWriteTruncBinCode(cu.sgpmIdx, SGPM_NUM);
  }
}

void CABACWriter::intra_luma_pred_mode(const CodingUnit &cu, const CUCtxIntra &cuCtxIntra)
{
  if (!cu.Y().valid())
  {
    return;
  }
  if (cu.bdpcmMode[0] != BdpcmMode::NONE)
  {
    const_cast<CodingUnit &>(cu).intraDir[ChannelType::LUMA] = cu.bdpcmMode[0] == BdpcmMode::VER ? VER_IDX : HOR_IDX;
    return;
  }

  mip_flag(cu);
  if (cu.mipFlag)
  {
    mip_pred_mode(cu);
    return;
  }
  dimd_flag(cu);
  if (cu.dimdFlag)
  {
    return;
  }
  timd_flag(cu);
  if (cu.timdFlag)
  {
    timd_sad_flag(cu);
    return;
  }
  eip_flag(cu);
  if (cu.eipFlag)
  {
    return;
  }
  sgpm_flag(cu);
  if (cu.sgpm)
  {
    return;
  }
  extend_ref_line(cu);

  // prev_intra_luma_pred_flag
  const int      numMPMs = NUM_PRIMARY_MOST_PROBABLE_MODES;
  const uint8_t *mpmPred = cuCtxIntra.mpmList;

  unsigned predMode = cu.intraDir[ChannelType::LUMA];
  unsigned mpmIdx   = numMPMs;

  for (int idx = 0; idx < numMPMs; idx++)
  {
    if (predMode == mpmPred[idx])
    {
      mpmIdx = idx;
      break;
    }
  }
  if (cu.multiRefIdx)
  {
    CHECK(mpmIdx >= numMPMs, "use of non-MPM");
  }
  else
  {
    m_binEncoder.encodeBin(mpmIdx < numMPMs, Ctx::IntraLumaMpmFlag());
  }
  // mpm_idx / rem_intra_luma_pred_mode
#if ENABLE_TRACING
  bool isMpm = mpmIdx < numMPMs, mpm2nd = false;
  int  lumaModeIdx = mpmIdx;
#endif
  if (mpmIdx < numMPMs)
  {
    unsigned ctx  = 0;
    unsigned ctx2 = (cu.multiRefIdx == 0 ? 2 : 1);
    if (cu.multiRefIdx == 0)
    {
      m_binEncoder.encodeBin(mpmIdx > 0, Ctx::IntraLumaPlanarFlag(ctx));
    }
    if (mpmIdx)
    {
      m_binEncoder.encodeBin(mpmIdx > 1, Ctx::IntraLumaMPMIdx(0 + ctx2));
    }
    if (mpmIdx > 1)
    {
      m_binEncoder.encodeBinEP(mpmIdx > 2);
    }
    if (mpmIdx > 2)
    {
      m_binEncoder.encodeBinEP(mpmIdx > 3);
    }
    if (mpmIdx > 3)
    {
      m_binEncoder.encodeBinEP(mpmIdx > 4);
    }
  }
  else
  {
    auto     secondMpmPred = mpmPred + NUM_PRIMARY_MOST_PROBABLE_MODES;
    unsigned secondMpmIdx  = NUM_SECONDARY_MOST_PROBABLE_MODES;

    for (unsigned idx = 0; idx < NUM_SECONDARY_MOST_PROBABLE_MODES; idx++)
    {
      if (predMode == secondMpmPred[idx])
      {
        secondMpmIdx = idx;
        break;
      }
    }

    if (secondMpmIdx < NUM_SECONDARY_MOST_PROBABLE_MODES)
    {
#if ENABLE_TRACING
      isMpm       = false;
      mpm2nd      = true;
      lumaModeIdx = secondMpmIdx;
#endif
      m_binEncoder.encodeBin(1, Ctx::IntraLumaSecondMpmFlag());
      m_binEncoder.encodeBinsEP(secondMpmIdx, 4);
    }
    else
    {
      m_binEncoder.encodeBin(0, Ctx::IntraLumaSecondMpmFlag());

      unsigned nonMpmIdx = NUM_NON_MPM_MODES;
      for (unsigned idx = 0; idx < NUM_NON_MPM_MODES; idx++)
      {
        if (predMode == cuCtxIntra.nonMPMList[idx])
        {
          nonMpmIdx = idx;
          break;
        }
      }
      xWriteTruncBinCode(nonMpmIdx,
                         NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES);   // Remaining mode is truncated binary coded
#if ENABLE_TRACING
      isMpm       = false;
      mpm2nd      = false;
      lumaModeIdx = nonMpmIdx;
#endif
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) mpm1st=%d mpm2nd=%d idx=%d\n", 0,
         cu.lumaPos().x, cu.lumaPos().y, isMpm, mpm2nd, lumaModeIdx);
  planarDir(cu);
}

void CABACWriter::nonLocalCCPIndex(const CodingUnit &cu)
{
  if (CU::hasNonLocalCCP(cu))
  {
    CHECK(cu.idxNonLocalCCP < 0 || cu.idxNonLocalCCP > MAX_CCP_CAND_LIST_SIZE, "Invalid idxNonLocalCCP");
    {
      m_binEncoder.encodeBin(cu.idxNonLocalCCP ? 1 : 0, Ctx::nonLocalCCP(0));

      if (cu.idxNonLocalCCP)
      {
        if (cu.cs->slice->m_sps->m_ccMergeFusion)
        {
          m_binEncoder.encodeBin(cu.ccMergeFusionIdx ? 1 : 0, Ctx::CcMergeFusionFlag(0));
        }

        if (cu.ccMergeFusionIdx)
        {
          unary_max_eqprob(cu.ccMergeFusionIdx - 1, MAX_CCP_FUSION_NUM - 1);
        }
        else
        {
          unary_max_eqprob(cu.idxNonLocalCCP - 1, MAX_CCP_CAND_LIST_SIZE - 1);
        }
      }
    }
  }
}

void CABACWriter::decoderDerivedCcpModes(const CodingUnit &cu)
{
  if (CU::hasDecoderDerivedCCP(cu))
  {
    m_binEncoder.encodeBin(cu.decDerivedCcpMode ? 1 : 0, Ctx::decoderDerivedCCP(0));
  }
}

void CABACWriter::ccFilterFlag(const CodingUnit &cu)
{
  if (CU::hasCcFilterFlag(cu))
  {
    m_binEncoder.encodeBin(cu.ccFilterFlag ? 1 : 0, Ctx::CcInsideFilterFlag(0));
  }
}

void CABACWriter::intra_chroma_lmc_mode(const CodingUnit &cu)
{
  decoderDerivedCcpModes(cu);

  if (cu.decDerivedCcpMode)
  {
    return;
  }

  nonLocalCCPIndex(cu);

  if (cu.idxNonLocalCCP)
  {
    return;
  }

  const unsigned intraDir = cu.intraDir[ChannelType::CHROMA];
  int            lmModeList[NUM_CHROMA_MODE];
  PU::getLMSymbolList(cu, lmModeList);
  int symbol = -1;
  for (int k = 0; k < LM_SYMBOL_NUM; k++)
  {
    if (lmModeList[k] == intraDir)
    {
      symbol = k;
      break;
    }
  }
  CHECK(symbol < 0, "invalid symbol found");

  m_binEncoder.encodeBin(symbol == 0 ? 0 : 1, Ctx::CclmModeIdx(0));

  if (symbol > 0)
  {
    CHECK(symbol > 5, "invalid symbol for MMLM");
    m_binEncoder.encodeBin(symbol == 1 ? 0 : 1, Ctx::MMLMFlag(0));

    if (symbol > 1)
    {
      m_binEncoder.encodeBinEP(symbol > 2);
    }

    if (symbol > 2)
    {
      m_binEncoder.encodeBinEP(symbol > 3);
    }
    if (symbol > 3)
    {
      m_binEncoder.encodeBinEP(symbol > 4);
    }
  }

  cccmFlag(cu);
  cclmDeltaSlope(cu);
  ccFilterFlag(cu);
}

void CABACWriter::intra_chroma_pred_mode(const CodingUnit &cu)
{
  if (!isChromaEnabled(cu.chromaFormat) || (CS::isDualITree(*cu.cs) && isLuma(cu.chType)))
  {
    return;
  }
  if (cu.bdpcmMode[1] != BdpcmMode::NONE)
  {
    const_cast<CodingUnit &>(cu).intraDir[ChannelType::CHROMA] = cu.bdpcmMode[1] == BdpcmMode::VER ? VER_IDX : HOR_IDX;
    return;
  }
  const unsigned intraDir = cu.intraDir[ChannelType::CHROMA];
  if (cu.cs->sps->m_LMChroma)
  {
    m_binEncoder.encodeBin(PU::isLMCMode(intraDir) ? 1 : 0, Ctx::CclmModeFlag(0));
    if (PU::isLMCMode(intraDir))
    {
      intra_chroma_lmc_mode(cu);
      return;
    }
  }

  const bool isDerivedMode = intraDir == DM_CHROMA_IDX;
  m_binEncoder.encodeBin(isDerivedMode ? 0 : 1, Ctx::IntraChromaPredMode(0));
  if (isDerivedMode)
  {
    return;
  }

  // chroma candidate index
  unsigned chromaCandModes[NUM_CHROMA_MODE];
  PU::getIntraChromaCandModes(cu, chromaCandModes);

  dimdChromaFlag(cu);
  if (cu.dimdChromaFlag)
  {
    return;
  }
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
  {
    m_binEncoder.encodeBinsEP(candId, 2);
  }
}

void CABACWriter::cccmFlag(const CodingUnit &cu)
{
  const unsigned intraDir = cu.intraDir[ChannelType::CHROMA];

  if (PU::cccmAvailable(cu, intraDir))
  {
    if (PU::cccmAvailable(cu, intraDir, CONV_MODEL_CCCM_BVG) && PU::hasBvgCccmFlag(cu))
    {
      m_binEncoder.encodeBin((cu.cccmType == CONV_MODEL_CCCM_BVG) ? 1 : 0, Ctx::BvgCccmFlag(0));

      if (cu.cccmType == CONV_MODEL_CCCM_BVG)
      {
        return;
      }
    }

    m_binEncoder.encodeBin(cu.cccmFlag ? 1 : 0, Ctx::CccmFlag(0));

    if (cu.cccmFlag)
    {
      int mpfIndex = PU::cccmMultiFilterIndex(cu.cccmType);

      if (PU::cccmAvailable(cu, intraDir, CONV_MODEL_CCCM_MULTIF_1))
      {
        m_binEncoder.encodeBin(mpfIndex ? 1 : 0, Ctx::CccmMpfFlag(0));
      }

      if (mpfIndex)
      {
        m_binEncoder.encodeBin(mpfIndex > 1 ? 1 : 0, Ctx::CccmMpfFlag(1));

        if (mpfIndex > 1)
        {
          m_binEncoder.encodeBin(mpfIndex > 2 ? 1 : 0, Ctx::CccmMpfFlag(2));
        }
      }
      else
      {
        m_binEncoder.encodeBin(cu.cccmType == CONV_MODEL_CCCM_NOSUBS ? 1 : 0, Ctx::CccmFlag(1));

        if (cu.cccmType != CONV_MODEL_CCCM_NOSUBS)
        {
          m_binEncoder.encodeBin(cu.cccmType == CONV_MODEL_CCCM_GRADLOC ? 1 : 0, Ctx::CccmFlag(2));
        }
      }
    }
  }
}

void CABACWriter::cu_residual(const CodingUnit &cu, Partitioner &partitioner, CUCtx &cuCtx)
{
  if (!CU::isIntra(cu))
  {
    if (!cu.mergeFlag)
    {
      rqt_root_cbf(cu);
    }
    if (cu.rootCbf)
    {
      sbt_mode(cu);
    }

    if (!cu.rootCbf)
    {
      return;
    }
  }

  cuCtx.violatesLfnstConstrained.fill(false);
  cuCtx.lfnstLastScanPos           = false;
  cuCtx.violatesMtsCoeffConstraint = false;
  cuCtx.mtsLastScanPos             = false;
  cuCtx.mtsCoeffAbsSum             = 0;

  transform_tree(*cu.cs, partitioner, cuCtx);
}

void CABACWriter::rqt_root_cbf(const CodingUnit &cu)
{
  m_binEncoder.encodeBin(cu.rootCbf, Ctx::QtRootCbf());

  DTRACE(g_trace_ctx, D_SYNTAX, "rqt_root_cbf() ctx=0 root_cbf=%d pos=(%d,%d)\n", cu.rootCbf ? 1 : 0, cu.lumaPos().x,
         cu.lumaPos().y);
}

void CABACWriter::sbt_mode(const CodingUnit &cu)
{
  uint8_t sbtAllowed = cu.checkAllowedSbt();
  if (!sbtAllowed)
  {
    return;
  }

  SizeType cuWidth  = cu.lwidth();
  SizeType cuHeight = cu.lheight();
  uint8_t  sbtIdx   = cu.getSbtIdx();
  uint8_t  sbtPos   = cu.getSbtPos();

  // sbt flag
  bool    sbtFlag = cu.sbtInfo != 0;
  uint8_t ctxIdx  = (cuWidth * cuHeight <= 256) ? 1 : 0;
  m_binEncoder.encodeBin(sbtFlag, Ctx::SbtFlag(ctxIdx));
  if (!sbtFlag)
  {
    return;
  }

  const bool sbtVerHalfAllowed = CU::targetSbtAllowed(SBT_VER_HALF, sbtAllowed);
  const bool sbtHorHalfAllowed = CU::targetSbtAllowed(SBT_HOR_HALF, sbtAllowed);
  const bool sbtVerQuadAllowed = CU::targetSbtAllowed(SBT_VER_QUAD, sbtAllowed);
  const bool sbtHorQuadAllowed = CU::targetSbtAllowed(SBT_HOR_QUAD, sbtAllowed);
  const bool sbtQuadAllowed    = CU::targetSbtAllowed(SBT_QUAD, sbtAllowed);
  const bool sbtQuarterAllowed = CU::targetSbtAllowed(SBT_QUARTER, sbtAllowed);
  const bool sbtRectAllowed    = (sbtVerHalfAllowed || sbtHorHalfAllowed || sbtVerQuadAllowed || sbtHorQuadAllowed);
  const int  lastTypeIdx       = int(sbtRectAllowed) + int(sbtQuadAllowed) + int(sbtQuarterAllowed) - 1;

  auto sbtRectMode = [&](const bool isLast, int &typeIdx)
  {
    if (sbtRectAllowed)
    {
      typeIdx++;
      const bool sbtRectFlag = (sbtIdx <= SBT_HOR_QUAD);
      if (!isLast)
      {
        m_binEncoder.encodeBin(sbtRectFlag, Ctx::SbtQuadFlag(1));
      }
      if (sbtRectFlag)
      {
        const bool sbtRectQuadFlag = (sbtIdx == SBT_HOR_QUAD || sbtIdx == SBT_VER_QUAD);
        const bool sbtRectHorFlag  = (sbtIdx == SBT_HOR_HALF || sbtIdx == SBT_HOR_QUAD);
        const bool sbtRectPosFlag  = (sbtPos == SBT_POS1);

        // rectangular type
        if ((sbtHorHalfAllowed || sbtVerHalfAllowed) && (sbtHorQuadAllowed || sbtVerQuadAllowed))
        {
          m_binEncoder.encodeBin(sbtRectQuadFlag, Ctx::SbtQuadFlag(0));
        }

        // rectangular direction
        if ((sbtRectQuadFlag && sbtVerQuadAllowed && sbtHorQuadAllowed) ||
            (!sbtRectQuadFlag && sbtVerHalfAllowed && sbtHorHalfAllowed))   // both direction allowed
        {
          uint8_t ctxIdx = (cuWidth == cuHeight) ? 0 : (cuWidth < cuHeight ? 1 : 2);
          m_binEncoder.encodeBin(sbtRectHorFlag, Ctx::SbtHorFlag(ctxIdx));
        }

        // rectangular position
        m_binEncoder.encodeBin(sbtRectPosFlag, Ctx::SbtPosFlag(0));

        DTRACE(g_trace_ctx, D_SYNTAX, "sbt_mode() pos=(%d,%d) sbtInfo=%d\n", cu.lx(), cu.ly(), (int)cu.sbtInfo);
        return true;
      }
    }
    return false;
  };

  auto sbtQuadMode = [&](const bool isLast, int &typeIdx)
  {
    if (sbtQuadAllowed)
    {
      typeIdx++;
      const bool bSbtQuad = (sbtIdx == SBT_QUAD);
      if (!isLast)
      {
        m_binEncoder.encodeBin(bSbtQuad, Ctx::SbtQuadFlag(2));
      }
      if (bSbtQuad)
      {
        // position
        const uint8_t horIdx = (sbtPos >> 0) & 0x1;
        const uint8_t verIdx = (sbtPos >> 1) & 0x1;
        m_binEncoder.encodeBin(horIdx, Ctx::SbtPosFlag(1));
        m_binEncoder.encodeBin(verIdx, Ctx::SbtPosFlag(2));
        DTRACE(g_trace_ctx, D_SYNTAX, "sbt_mode() pos=(%d,%d) sbtInfo=%d\n", cu.lx(), cu.ly(), (int)cu.sbtInfo);
        return true;
      }
    }
    return false;
  };

  auto sbtQuarterMode = [&](const bool isLast, int &typeIdx)
  {
    if (sbtQuarterAllowed)
    {
      typeIdx++;
      const bool bSbtQuarter = (sbtIdx == SBT_QUARTER);
      if (!isLast)
      {
        m_binEncoder.encodeBin(bSbtQuarter, Ctx::SbtQuadFlag(3));
      }
      if (bSbtQuarter)
      {
        // position
        const uint8_t horIdx = (sbtPos >> 0) & 0x1;
        const uint8_t verIdx = (sbtPos >> 1) & 0x1;
        m_binEncoder.encodeBin(horIdx, Ctx::SbtPosFlag(3));
        m_binEncoder.encodeBin(verIdx, Ctx::SbtPosFlag(4));
        DTRACE(g_trace_ctx, D_SYNTAX, "sbt_mode() pos=(%d,%d) sbtInfo=%d\n", cu.lx(), cu.ly(), (int)cu.sbtInfo);
        return true;
      }
    }
    return false;
  };

  int typeIdx = 0;
  if (sbtRectMode(typeIdx == lastTypeIdx, typeIdx))
  {
    return;
  }
  if (sbtQuadMode(typeIdx == lastTypeIdx, typeIdx))
  {
    return;
  }
  if (sbtQuarterMode(typeIdx == lastTypeIdx, typeIdx))
  {
    return;
  }
}

void CABACWriter::end_of_ctu(const CodingUnit &cu, CUCtx &cuCtx)
{
  const bool isLastSubCUOfCtu = CU::isLastSubCUOfCtu(cu);

  if (isLastSubCUOfCtu && (!CS::isDualITree(*cu.cs) || !isChromaEnabled(cu.chromaFormat) || isChroma(cu.chType)))
  {
    cuCtx.isDQPCoded = (cu.cs->pps->m_useDQP && !cuCtx.isDQPCoded);
  }
}

void CABACWriter::cu_palette_info(const CodingUnit &cu, CompID compBegin, uint32_t numComp, CUCtx &cuCtx)
{
  const SPS     &sps          = *(cu.cs->sps);
  TransformUnit &tu           = *cu.firstTU;
  uint32_t       indexMaxSize = cu.useEscape[compBegin] ? (cu.curPLTSize[compBegin] + 1) : cu.curPLTSize[compBegin];

  int maxPltSize = CS::isDualITree(*cu.cs) ? MAXPLTSIZE_DUALTREE : MAXPLTSIZE;

  if (cu.lastPLTSize[compBegin])
  {
    xEncodePLTPredIndicator(cu, maxPltSize, compBegin);
  }

  uint32_t reusedPLTnum = 0;
  for (int idx = 0; idx < cu.lastPLTSize[compBegin]; idx++)
  {
    if (cu.reuseflag[compBegin][idx])
    {
      reusedPLTnum++;
    }
  }
  if (reusedPLTnum < maxPltSize)
  {
    exp_golomb_eqprob(cu.curPLTSize[compBegin] - reusedPLTnum, 0);
  }

  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    for (int idx = reusedPLTnum; idx < cu.curPLTSize[compBegin]; idx++)
    {
      CompID    compID          = (CompID)comp;
      const int channelBitDepth = sps.m_bitDepths[toChannelType(compID)];
      m_binEncoder.encodeBinsEP(cu.curPLT[comp][idx], channelBitDepth);
    }
  }
  uint32_t signalEscape = (cu.useEscape[compBegin]) ? 1 : 0;
  if (cu.curPLTSize[compBegin] > 0)
  {
    m_binEncoder.encodeBinEP(signalEscape);
  }
  // encode index map
  uint32_t height = cu.block(compBegin).height;
  uint32_t width  = cu.block(compBegin).width;

  m_scanOrder =
    g_scanOrder[SCAN_UNGROUPED][(cu.useRotation[compBegin]) ? CoeffScanType::TRAV_VER : CoeffScanType::TRAV_HOR]
               [gp_sizeIdxInfo->idxFrom(width)][gp_sizeIdxInfo->idxFrom(height)];
  uint32_t total = height * width;
  if (indexMaxSize > 1)
  {
    codeScanRotationModeFlag(cu, compBegin);
  }
  else
  {
    assert(!cu.useRotation[compBegin]);
  }

  if (cu.useEscape[compBegin] && cu.cs->pps->m_useDQP && !cuCtx.isDQPCoded)
  {
    if (!CS::isDualITree(*cu.cs) || isLuma(tu.chType))
    {
      cu_qp_delta(cu, cuCtx.qp, cu.qp);
      cuCtx.qp         = cu.qp;
      cuCtx.isDQPCoded = true;
    }
  }
  if (cu.useEscape[compBegin] && cu.cs->slice->m_chromaQpAdjEnabled && !cuCtx.isChromaQpAdjCoded)
  {
    if (!CS::isDualITree(*tu.cs) || isChroma(tu.chType))
    {
      cu_chroma_qp_offset(cu);
      cuCtx.isChromaQpAdjCoded = true;
    }
  }

  uint32_t prevRunPos  = 0;
  unsigned prevRunType = 0;
  for (int subSetId = 0; subSetId <= (total - 1) >> LOG2_PALETTE_CG_SIZE; subSetId++)
  {
    cuPaletteSubblockInfo(cu, compBegin, numComp, subSetId, prevRunPos, prevRunType);
  }
  CHECK(cu.curPLTSize[compBegin] > maxPltSize, " Current palette size is larger than maximum palette size");
}

void CABACWriter::cuPaletteSubblockInfo(const CodingUnit &cu, CompID compBegin, uint32_t numComp, int subSetId,
                                        uint32_t &prevRunPos, unsigned &prevRunType)
{
  const SPS     &sps          = *(cu.cs->sps);
  TransformUnit &tu           = *cu.firstTU;
  PLTtypeBuf     runType      = tu.getrunType(toChannelType(compBegin));
  PelBuf         curPLTIdx    = tu.getcurPLTIdx(toChannelType(compBegin));
  uint32_t       indexMaxSize = cu.useEscape[compBegin] ? (cu.curPLTSize[compBegin] + 1) : cu.curPLTSize[compBegin];
  uint32_t       totalPel     = cu.block(compBegin).height * cu.block(compBegin).width;

  int minSubPos = subSetId << LOG2_PALETTE_CG_SIZE;
  int maxSubPos = minSubPos + (1 << LOG2_PALETTE_CG_SIZE);
  maxSubPos     = (maxSubPos > totalPel) ? totalPel : maxSubPos;   // if last position is out of the current CU size

  unsigned runCopyFlag[(1 << LOG2_PALETTE_CG_SIZE)];
  for (int i = 0; i < (1 << LOG2_PALETTE_CG_SIZE); i++)
  {
    runCopyFlag[i] = MAX_INT;
  }

  if (minSubPos == 0)
  {
    runCopyFlag[0] = 0;
  }

  // PLT runCopy flag and runType - context coded
  int curPos = minSubPos;
  for (; curPos < maxSubPos && indexMaxSize > 1; curPos++)
  {
    uint32_t posy     = m_scanOrder[curPos].y;
    uint32_t posx     = m_scanOrder[curPos].x;
    uint32_t posyprev = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].y;
    uint32_t posxprev = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].x;
    // encode runCopyFlag
    bool     identityFlag =
      !((runType.at(posx, posy) != runType.at(posxprev, posyprev)) ||
        ((runType.at(posx, posy) == PLT_RUN_INDEX) && (curPLTIdx.at(posx, posy) != curPLTIdx.at(posxprev, posyprev))));

    const CtxSet &ctxSet = (prevRunType == PLT_RUN_INDEX) ? Ctx::IdxRunModel : Ctx::CopyRunModel;
    if (curPos > 0)
    {
      int            dist             = curPos - prevRunPos - 1;
      const unsigned ctxId            = DeriveCtx::CtxPltCopyFlag(prevRunType, dist);
      runCopyFlag[curPos - minSubPos] = identityFlag;
      m_binEncoder.encodeBin(identityFlag, ctxSet(ctxId));
      DTRACE(g_trace_ctx, D_SYNTAX, "plt_copy_flag() bin=%d ctx=%d\n", identityFlag, ctxId);
    }
    // encode run_type
    if (!identityFlag || curPos == 0)
    {
      prevRunPos  = curPos;
      prevRunType = runType.at(posx, posy);
      if (((posy == 0) && !cu.useRotation[compBegin]) || ((posx == 0) && cu.useRotation[compBegin]))
      {
        assert(runType.at(posx, posy) == PLT_RUN_INDEX);
      }
      else if (curPos != 0 && runType.at(posxprev, posyprev) == PLT_RUN_COPY)
      {
        assert(runType.at(posx, posy) == PLT_RUN_INDEX);
      }
      else
      {
        m_binEncoder.encodeBin(runType.at(posx, posy), Ctx::RunTypeFlag());
      }
      DTRACE(g_trace_ctx, D_SYNTAX, "plt_type_flag() bin=%d sp=%d\n", runType.at(posx, posy), curPos);
    }
  }

  // PLT index values - bypass coded
  if (indexMaxSize > 1)
  {
    curPos = minSubPos;
    for (; curPos < maxSubPos; curPos++)
    {
      uint32_t posy = m_scanOrder[curPos].y;
      uint32_t posx = m_scanOrder[curPos].x;
      if (runCopyFlag[curPos - minSubPos] == 0 && runType.at(posx, posy) == PLT_RUN_INDEX)
      {
        writePLTIndex(cu, curPos, curPLTIdx, runType, indexMaxSize, compBegin);
        DTRACE(g_trace_ctx, D_SYNTAX, "plt_idx_idc() value=%d sp=%d\n", curPLTIdx.at(posx, posy), curPos);
      }
    }
  }

  // Quantized escape colors - bypass coded
  uint32_t scaleX = getComponentScaleX(COMP_Cb, sps.m_chromaFormatIdc);
  uint32_t scaleY = getComponentScaleY(COMP_Cb, sps.m_chromaFormatIdc);
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    CompID compID = (CompID)comp;
    for (curPos = minSubPos; curPos < maxSubPos; curPos++)
    {
      uint32_t posy = m_scanOrder[curPos].y;
      uint32_t posx = m_scanOrder[curPos].x;
      if (curPLTIdx.at(posx, posy) == cu.curPLTSize[compBegin])
      {
        PLTescapeBuf escapeValue = tu.getescapeValue((CompID)comp);
        if (compID == COMP_Y || compBegin != COMP_Y)
        {
          exp_golomb_eqprob((unsigned)escapeValue.at(posx, posy), 5);
          DTRACE(g_trace_ctx, D_SYNTAX, "plt_escape_val() value=%d etype=%d sp=%d\n", escapeValue.at(posx, posy), comp,
                 curPos);
        }
        if (compBegin == COMP_Y && compID != COMP_Y && posy % (1 << scaleY) == 0 && posx % (1 << scaleX) == 0)
        {
          uint32_t posxC = posx >> scaleX;
          uint32_t posyC = posy >> scaleY;
          exp_golomb_eqprob((unsigned)escapeValue.at(posxC, posyC), 5);
          DTRACE(g_trace_ctx, D_SYNTAX, "plt_escape_val() value=%d etype=%d sp=%d\n", escapeValue.at(posx, posy), comp,
                 curPos);
        }
      }
    }
  }
}

void CABACWriter::codeScanRotationModeFlag(const CodingUnit &cu, CompID compBegin)
{
  m_binEncoder.encodeBin((cu.useRotation[compBegin]), Ctx::RotationFlag());
}

void CABACWriter::xEncodePLTPredIndicator(const CodingUnit &cu, uint32_t maxPLTSize, CompID compBegin)
{
  int      lastPredIdx     = -1;
  uint32_t run             = 0;
  uint32_t numPLTPredicted = 0;
  for (uint32_t idx = 0; idx < cu.lastPLTSize[compBegin]; idx++)
  {
    if (cu.reuseflag[compBegin][idx])
    {
      numPLTPredicted++;
      lastPredIdx = idx;
    }
  }

  int idx = 0;
  while (idx <= lastPredIdx)
  {
    if (cu.reuseflag[compBegin][idx])
    {
      exp_golomb_eqprob(run ? run + 1 : run, 0);
      run = 0;
    }
    else
    {
      run++;
    }
    idx++;
  }
  if ((numPLTPredicted < maxPLTSize && lastPredIdx + 1 < cu.lastPLTSize[compBegin]) || !numPLTPredicted)
  {
    exp_golomb_eqprob(1, 0);
  }
}

Pel CABACWriter::writePLTIndex(const CodingUnit &cu, uint32_t idx, PelBuf &paletteIdx, PLTtypeBuf &paletteRunType,
                               int maxSymbol, CompID compBegin)
{
  uint32_t posy = m_scanOrder[idx].y;
  uint32_t posx = m_scanOrder[idx].x;
  Pel curLevel  = (paletteIdx.at(posx, posy) == cu.curPLTSize[compBegin]) ? (maxSymbol - 1) : paletteIdx.at(posx, posy);
  if (idx)   // R0348: remove index redundancy
  {
    uint32_t prevposy = m_scanOrder[idx - 1].y;
    uint32_t prevposx = m_scanOrder[idx - 1].x;
    if (paletteRunType.at(prevposx, prevposy) == PLT_RUN_INDEX)
    {
      Pel leftLevel = paletteIdx.at(prevposx, prevposy);   // left index
      if (leftLevel == cu.curPLTSize[compBegin])   // escape mode
      {
        leftLevel = maxSymbol - 1;
      }
      assert(leftLevel != curLevel);
      if (curLevel > leftLevel)
      {
        curLevel--;
      }
    }
    else
    {
      Pel aboveLevel;
      if (cu.useRotation[compBegin])
      {
        assert(prevposx > 0);
        aboveLevel = paletteIdx.at(posx - 1, posy);
        if (paletteIdx.at(posx - 1, posy) == cu.curPLTSize[compBegin])   // escape mode
        {
          aboveLevel = maxSymbol - 1;
        }
      }
      else
      {
        assert(prevposy > 0);
        aboveLevel = paletteIdx.at(posx, posy - 1);
        if (paletteIdx.at(posx, posy - 1) == cu.curPLTSize[compBegin])   // escape mode
        {
          aboveLevel = maxSymbol - 1;
        }
      }
      assert(curLevel != aboveLevel);
      if (curLevel > aboveLevel)
      {
        curLevel--;
      }
    }
    maxSymbol--;
  }
  assert(maxSymbol > 0);
  assert(curLevel >= 0);
  assert(maxSymbol > curLevel);
  if (maxSymbol > 1)
  {
    xWriteTruncBinCode(curLevel, maxSymbol);
  }
  return curLevel;
}

//================================================================================
//  clause 7.3.8.6
//--------------------------------------------------------------------------------
//    void  prediction_unit ( cu );
//    void  merge_flag      ( cu );
//    void  merge_idx       ( cu );
//    void  inter_pred_idc  ( cu );
//    void  ref_idx         ( cu, refList );
//    void  mvp_flag        ( cu, refList );
//================================================================================

void CABACWriter::prediction_unit(const CodingUnit &cu)
{
  if (cu.skip)
  {
    CHECK(!cu.mergeFlag, "merge_flag must be true for skipped CUs");
  }
  else
  {
    merge_flag(cu);
  }
  if (cu.mergeFlag)
  {
    merge_data(cu);
  }
  else if (CU::isIBC(cu))
  {
    ref_idx(cu, RPL0);
    Mv mvd = cu.mvd[RPL0];
    mvd.changeIbcPrecInternal2Amvr(cu.imv);
    bvdCoding(mvd);   // already changed to signaling precision
    if (cu.cs->sps->m_maxNumIBCMergeCand == 1)
    {
      CHECK(cu.mvpIdx[RPL0], "mvpIdx for IBC mode should be 0");
    }
    else
    {
      mvp_flag(cu, RPL0);
    }
  }
  else
  {
    inter_pred_idc(cu);
    affine_flag(cu);
    smvd_mode(cu);
    if (cu.interDir != 2 /* PRED_L1 */)
    {
      ref_idx(cu, RPL0);
      if (cu.affine)
      {
        for (int i = 0; i < cu.getNumAffineMvs(); i++)
        {
          mvd_coding(cu, cu.mvdAffi[RPL0][i], cu.imv);
        }
      }
      else
      {
        mvd_coding(cu, cu.mvd[RPL0], cu.imv);
      }
      mvp_flag(cu, RPL0);
    }
    if (cu.interDir != 1 /* PRED_L0 */)
    {
      if (cu.smvdMode != 1)
      {
        ref_idx(cu, RPL1);
        if (!cu.cs->picHeader->m_mvdL1ZeroFlag || cu.interDir != 3 /* PRED_BI */)
        {
          if (cu.affine)
          {
            for (int i = 0; i < cu.getNumAffineMvs(); i++)
            {
              mvd_coding(cu, cu.mvdAffi[RPL1][i], cu.imv);
            }
          }
          else
          {
            mvd_coding(cu, cu.mvd[RPL1], cu.imv);
          }
        }
      }
      mvp_flag(cu, RPL1);
    }
  }
}

void CABACWriter::smvd_mode(const CodingUnit &cu)
{
  if (cu.interDir != 3 || cu.affine)
  {
    return;
  }

  if (cu.cs->slice->getBiDirPred() == false)
  {
    return;
  }

  m_binEncoder.encodeBin(cu.smvdMode ? 1 : 0, Ctx::SmvdFlag());

  DTRACE(g_trace_ctx, D_SYNTAX, "symmvd_flag() symmvd=%d pos=(%d,%d) size=%dx%d\n", cu.smvdMode ? 1 : 0, cu.lumaPos().x,
         cu.lumaPos().y, cu.lumaSize().width, cu.lumaSize().height);
}

void CABACWriter::subblock_merge_flag(const CodingUnit &cu)
{
  if (!cu.cs->slice->isIntra() && (cu.slice->m_picHeader->m_maxNumAffineMergeCand > 0) && CU::isAffineAllowed(cu))
  {
    unsigned ctxId = DeriveCtx::CtxAffineFlag(cu);
    m_binEncoder.encodeBin(cu.affine, Ctx::SubblockMergeFlag(ctxId));
    DTRACE(g_trace_ctx, D_SYNTAX, "subblock_merge_flag() subblock_merge_flag=%d ctx=%d pos=(%d,%d)\n",
           cu.affine ? 1 : 0, ctxId, cu.lx(), cu.ly());
  }
}

void CABACWriter::affine_flag(const CodingUnit &cu)
{
  if (!cu.cs->slice->isIntra() && cu.cs->sps->m_useAffine &&
      std::min(cu.lumaSize().width, cu.lumaSize().height) >=
        (1 << (cu.cs->slice->m_sps->m_log2MinAffineBlkSizeMinus3 + 3)))
  {
    unsigned ctxId = DeriveCtx::CtxAffineFlag(cu);
    m_binEncoder.encodeBin(cu.affine, Ctx::AffineFlag(ctxId));
    DTRACE(g_trace_ctx, D_SYNTAX, "affine_flag() affine=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.lx(),
           cu.ly());

    if (cu.affine && cu.cs->sps->m_AffineType)
    {
      unsigned ctxId = 0;
      m_binEncoder.encodeBin(cu.affineType != AffineModel::_4_PARAMS ? 1 : 0, Ctx::AffineType(ctxId));
      DTRACE(g_trace_ctx, D_SYNTAX, "affine_type() affine_type=%d ctx=%d pos=(%d,%d)\n",
             cu.affineType != AffineModel::_4_PARAMS ? 1 : 0, ctxId, cu.lx(), cu.ly());
    }
  }
}

void CABACWriter::affine_mmvd_data(const CodingUnit &pu)
{
  if (!pu.cs->sps->m_AffineMmvdMode || !pu.mergeFlag || !pu.affine)
  {
    return;
  }

  m_binEncoder.encodeBin(pu.mmvdMergeFlag, Ctx::AffMmvdFlag());
  DTRACE(g_trace_ctx, D_SYNTAX, "affine_mmvd_flag() af_mmvd_merge=%d pos=(%d,%d) size=%dx%d\n",
         pu.mmvdMergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);

  if (!pu.mmvdMergeFlag)
  {
    return;
  }

  // Base affine merge candidate idx
  uint8_t affMmvdBaseIdx    = pu.mmvdMergeIdx.pos.baseIdx;
  int     numCandminus1Base = AF_MMVD_BASE_NUM - 1;
  if (numCandminus1Base > 0)
  {
    // to support more base candidates
    m_binEncoder.encodeBin((affMmvdBaseIdx == 0 ? 0 : 1), Ctx::AffMmvdIdx());

    if (affMmvdBaseIdx > 0)
    {
      for (unsigned idx = 1; idx < numCandminus1Base; idx++)
      {
        m_binEncoder.encodeBinEP(affMmvdBaseIdx == idx ? 0 : 1);
        if (affMmvdBaseIdx == idx)
        {
          break;
        }
      }
    }
  }

  // Code Step Value
  uint8_t step              = pu.mmvdMergeIdx.pos.step;
  int     numCandminus1Step = AF_MMVD_STEP_NUM - 1;
  if (numCandminus1Step > 0)
  {
    m_binEncoder.encodeBin((step == 0 ? 0 : 1), Ctx::AffMmvdOffsetStep());

    if (step > 0)
    {
      for (unsigned idx = 1; idx < numCandminus1Step; idx++)
      {
        m_binEncoder.encodeBinEP(step == idx ? 0 : 1);
        if (step == idx)
        {
          break;
        }
      }
    }
  }

  // Code Dir Value
  uint8_t offsetDir = pu.mmvdMergeIdx.pos.position;
  uint8_t b0        = offsetDir & 0x1;
  uint8_t b1        = (offsetDir >> 1) & 0x1;
  m_binEncoder.encodeBinEP(b0);
  m_binEncoder.encodeBinEP(b1);

  DTRACE(g_trace_ctx, D_SYNTAX, "affine_mmvd_data() base=%d dir=%d step=%d\n", pu.mmvdMergeIdx.pos.baseIdx,
         pu.mmvdMergeIdx.pos.position, pu.mmvdMergeIdx.pos.step);
}

void CABACWriter::merge_flag(const CodingUnit &cu)
{
  if (CU::isIBC(cu) && !cu.slice->m_sps->m_ibcMerge)
  {
    CHECK(cu.mergeFlag, "IBC merge flag shall be disabled");
    return;
  }
  m_binEncoder.encodeBin(cu.mergeFlag, Ctx::MergeFlag());

  DTRACE(g_trace_ctx, D_SYNTAX, "merge_flag() merge=%d pos=(%d,%d) size=%dx%d\n", cu.mergeFlag ? 1 : 0, cu.lumaPos().x,
         cu.lumaPos().y, cu.lumaSize().width, cu.lumaSize().height);
}

void CABACWriter::merge_data(const CodingUnit &cu)
{
  if (CU::isIBC(cu))
  {
    merge_idx(cu);
    return;
  }
  subblock_merge_flag(cu);
  if (cu.affine)
  {
    affine_mmvd_data(cu);
    if (CU::hasOppositeLICFlag(cu))
    {
      m_binEncoder.encodeBin(cu.oppositeLicFlag, Ctx::AffineFlagOppositeLic(0));
    }
    merge_idx(cu);
    return;
  }
  const int  maxSize       = std::min<int>(MAX_TB_SIZEY, MAX_INTRA_SIZE);
  const bool ciipAvailable = cu.cs->sps->m_useCiip && !cu.skip && cu.lwidth() <= maxSize && cu.lheight() <= maxSize &&
    cu.lwidth() * cu.lheight() >= CIIP_MIN_AREA;
  const bool geoAvailable = CU::canUseGPM(cu);

  if (geoAvailable || ciipAvailable)
  {
    m_binEncoder.encodeBin(cu.regularMergeFlag, Ctx::RegularMergeFlag(cu.skip ? 0 : 1));
  }
  if (cu.regularMergeFlag)
  {
    bm_merge_flag(cu);
    if (cu.cs->sps->m_useMMVD && !cu.bmMergeFlag)
    {
      m_binEncoder.encodeBin(cu.mmvdMergeFlag, Ctx::MmvdFlag(0));
      DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_flag() mmvd_merge=%d pos=(%d,%d) size=%dx%d\n",
             cu.mmvdMergeFlag ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y, cu.lumaSize().width, cu.lumaSize().height);
    }
    if (cu.mmvdMergeFlag || cu.mmvdSkip)
    {
      mmvd_merge_idx(cu);
    }
    else
    {
      if (CU::hasOppositeLICFlag(cu))
      {
        m_binEncoder.encodeBin(cu.oppositeLicFlag, Ctx::MergeFlagOppositeLic(0));
      }
      merge_idx(cu);
    }
  }
  else
  {
    if (geoAvailable && ciipAvailable)
    {
      ciip_flag(cu, false);
    }
    else if (ciipAvailable)
    {
      CHECK(cu.ciipFlag == false, "error in CIIP flag!")
      ciip_flag(cu, true);
    }
    merge_idx(cu);
  }
}

void CABACWriter::imv_mode(const CodingUnit &cu)
{
  const SPS *sps = cu.cs->sps;

  if (!sps->m_AMVREnabledFlag)
  {
    if (CU::isIBC(cu) && !cu.mergeFlag)
    {
      CHECK(cu.imv != IMV_FPEL, "Error on default IMV flag of IBC AMVP mode")
    }
    return;
  }
  if (cu.affine)
  {
    return;
  }

  bool useIBCFrac = CU::isIBC(cu) && !cu.mergeFlag && cu.slice->m_sps->m_ibcFracFlag;
  bool nonZeroMvd = CU::hasSubCUNonZeroMVd(cu);
  if (!nonZeroMvd)
  {
    if (CU::isIBC(cu) && !cu.mergeFlag)
    {
      CHECK(cu.imv != (useIBCFrac ? IBC_SUBPEL_AMVR_MODE_FOR_ZERO_MVD : 1),
            "Error on default IMV flag of IBC AMVP mode")
    }
    return;
  }

  const CtxSet &imvCtx = CU::isIBC(cu) && cu.cs->sps->m_ibcFracFlag ? Ctx::ImvFlagIBC : Ctx::ImvFlag;
  if (CU::isIBC(cu) && cu.cs->sps->m_ibcFracFlag)
  {
    CHECK(cu.imv == IMV_HPEL, "IBC does not support IMV_HPEL");
  }

  if (CU::isIBC(cu) == false || useIBCFrac)
  {
    m_binEncoder.encodeBin((cu.imv > 0), imvCtx(0));
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", (cu.imv > 0), 0);

  if (sps->m_AMVREnabledFlag && cu.imv > 0)
  {
    if (!CU::isIBC(cu))
    {
      m_binEncoder.encodeBin(cu.imv < IMV_HPEL, imvCtx(4));
      DTRACE(g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", cu.imv < 3, 4);
    }
    if (cu.imv < IMV_HPEL)
    {
      m_binEncoder.encodeBin((cu.imv > 1), imvCtx(1));
      DTRACE(g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", (cu.imv > 1), 1);
    }
  }

  if (CU::isIBC(cu))
  {
    CHECK(cu.slice->m_sps->m_ibcFracFlag && cu.imv == IMV_HPEL, "IBC does not support IMV_HPEL");
    CHECK(!useIBCFrac && (cu.imv == IMV_OFF || cu.imv == IMV_HPEL),
          "Fractiona IBC is not enabled to support fractional BVD coding");
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "imv_mode() IMVFlag=%d\n", cu.imv);
}

void CABACWriter::affine_amvr_mode(const CodingUnit &cu)
{
  const SPS *sps = cu.slice->m_sps;

  if (!sps->m_affineAmvrEnabledFlag || !cu.affine)
  {
    return;
  }

  if (!CU::hasSubCUNonZeroAffineMVd(cu))
  {
    return;
  }

  m_binEncoder.encodeBin((cu.imv > 0), Ctx::ImvFlag(2));
  DTRACE(g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", (cu.imv > 0), 2);

  if (cu.imv > 0)
  {
    m_binEncoder.encodeBin((cu.imv > 1), Ctx::ImvFlag(3));
    DTRACE(g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", (cu.imv > 1), 3);
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "affine_amvr_mode() IMVFlag=%d\n", cu.imv);
}

void CABACWriter::merge_idx(const CodingUnit &cu)
{
  if (cu.affine)
  {
    if (cu.mmvdMergeFlag)
    {
      return;
    }
    int numCandminus1 = int(cu.cs->picHeader->m_maxNumAffineMergeCand) - 1;
    if (cu.oppositeLicFlag)
    {
      numCandminus1 = int(cu.cs->sps->m_maxNumAffineOppositeLicMergeCand) - 1;
    }
    if (numCandminus1 > 0)
    {
      unsigned int unaryIdx = 0;
      for (; unaryIdx < numCandminus1; ++unaryIdx)
      {
        unsigned int symbol = cu.mergeIdx == unaryIdx ? 0 : 1;
        m_binEncoder.encodeBin(symbol, Ctx::AffMergeIdx((unaryIdx > 2 ? 2 : unaryIdx)));
        if (symbol == 0)
        {
          break;
        }
      }
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "aff_merge_idx() aff_merge_idx=%d\n", cu.mergeIdx);
  }
  else if (cu.geoFlag)
  {
    geo_adaptive_blending_idx(cu.geoBldIdx);

    bool isIntra0          = (cu.geoMergeIdx[0] >= GEO_MAX_NUM_UNI_CANDS);
    bool isIntra1          = (cu.geoMergeIdx[1] >= GEO_MAX_NUM_UNI_CANDS);
    bool bUseOnlyOneVector = cu.slice->isInterP() || cu.cs->sps->m_maxNumGeoCand == 1;
    m_binEncoder.encodeBin(cu.geoMMVDFlag[0], Ctx::GeoMmvdFlag());
    DTRACE(g_trace_ctx, D_SYNTAX, "geo_mmvd_flag() L%d : %d\n", 0, cu.geoMMVDFlag[0]);
    if (cu.geoMMVDFlag[0])
    {
      geo_mmvd_idx(cu, RPL0);
    }
    else
    {
      m_binEncoder.encodeBin(isIntra0 ? 1 : 0, Ctx::GPMIntraFlag());
      DTRACE(g_trace_ctx, D_SYNTAX, "geo_intra_flag() L%d : %d\n", 0, isIntra0);
    }

    if (!bUseOnlyOneVector || isIntra0)
    {
      m_binEncoder.encodeBin(cu.geoMMVDFlag[1], Ctx::GeoMmvdFlag());
      DTRACE(g_trace_ctx, D_SYNTAX, "geo_mmvd_flag() L%d : %d\n", 1, cu.geoMMVDFlag[1]);
      if (cu.geoMMVDFlag[1])
      {
        geo_mmvd_idx(cu, RPL1);
      }
      else if (!isIntra0)
      {
        m_binEncoder.encodeBin(isIntra1 ? 1 : 0, Ctx::GPMIntraFlag());
        DTRACE(g_trace_ctx, D_SYNTAX, "geo_intra_flag() L%d : %d\n", 1, isIntra1);
      }
    }

    if (!cu.geoMMVDFlag[0] && !cu.geoMMVDFlag[1])
    {
      if (!isIntra0 && !isIntra1)
      {
        geo_merge_idx(cu);
      }
      else
      {
        geo_merge_idx1(cu);
      }
    }
    else if (cu.geoMMVDFlag[0] && cu.geoMMVDFlag[1])
    {
      if (cu.geoMMVDIdx[0] == cu.geoMMVDIdx[1])
      {
        geo_merge_idx(cu);
      }
      else
      {
        geo_merge_idx1(cu);
      }
    }
    else
    {
      geo_merge_idx1(cu);
    }
  }
  else
  {
    int     numCandminus1;
    uint8_t mergeIdx = cu.mergeIdx;

    if (CU::isIBC(cu))
    {
      numCandminus1 = int(cu.cs->sps->m_maxNumIBCMergeCand) - 1;
    }
    else if (cu.bmMergeFlag)
    {
      numCandminus1 = int(cu.cs->sps->m_maxNumBMMergeCand) - 1;
    }
    else if (cu.oppositeLicFlag)
    {
      numCandminus1 = int(cu.cs->sps->m_maxNumOppositeLicMergeCand) - 1;
    }
    else
    {
      numCandminus1 = int(cu.cs->sps->m_maxNumMergeCand) - 1;
    }
    if (numCandminus1 > 0)
    {
      unsigned int uiUnaryIdx = 0;
      for (; uiUnaryIdx < numCandminus1; ++uiUnaryIdx)
      {
        unsigned int uiSymbol = mergeIdx == uiUnaryIdx ? 0 : 1;
        m_binEncoder.encodeBin(
          uiSymbol, Ctx::MergeIdx((uiUnaryIdx > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : uiUnaryIdx)));
        if (uiSymbol == 0)
        {
          break;
        }
      }
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", mergeIdx);
  }
}

void CABACWriter::mmvd_merge_idx(const CodingUnit &cu)
{
  const int mvdBaseIdx  = cu.mmvdMergeIdx.pos.baseIdx;
  const int mvdStep     = cu.mmvdMergeIdx.pos.step;
  const int mvdPosition = cu.mmvdMergeIdx.pos.position;

  if (cu.cs->sps->m_maxNumMergeCand > 1)
  {
    static_assert(MmvdIdx::BASE_MV_NUM == 2, "");
    assert(mvdBaseIdx < 2);
    m_binEncoder.encodeBin(mvdBaseIdx, Ctx::MmvdMergeIdx());
  }
  // DTRACE(g_trace_ctx, D_SYNTAX, "base_mvp_idx() base_mvp_idx=%d\n", mvdBaseIdx);

  int numStepCandMinus1 = MmvdIdx::REFINE_STEP - 1;
  if (numStepCandMinus1 > 0)
  {
    if (mvdStep == 0)
    {
      m_binEncoder.encodeBin(0, Ctx::MmvdStepMvpIdx());
    }
    else
    {
      m_binEncoder.encodeBin(1, Ctx::MmvdStepMvpIdx());
      for (unsigned idx = 1; idx < numStepCandMinus1; idx++)
      {
        m_binEncoder.encodeBinEP(mvdStep == idx ? 0 : 1);
        if (mvdStep == idx)
        {
          break;
        }
      }
    }
  }
  // DTRACE(g_trace_ctx, D_SYNTAX, "MmvdStepMvpIdx() MmvdStepMvpIdx=%d\n", mvdStep);

  m_binEncoder.encodeBinsEP(mvdPosition, 2);

  // DTRACE(g_trace_ctx, D_SYNTAX, "pos() pos=%d\n", mvdPosition);
  DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_idx() base=%d pos=%d step=%d\n", cu.mmvdMergeIdx.pos.baseIdx,
         cu.mmvdMergeIdx.pos.position, cu.mmvdMergeIdx.pos.step);
}

void CABACWriter::geo_mmvd_idx(const CodingUnit &cu, RefPicList eRefPicList)
{
  int  geoMMVDIdx = cu.geoMMVDIdx[eRefPicList];
  bool extMMVD    = cu.cs->picHeader->m_gpmMMVDTableFlag;
  CHECK(geoMMVDIdx >= (extMMVD ? GPM_EXT_MMVD_MAX_REFINE_NUM : GPM_MMVD_MAX_REFINE_NUM),
        "invalid GPM MMVD index exist");

  int step      = (extMMVD ? (geoMMVDIdx >> 3) : (geoMMVDIdx >> 2));
  int direction = (extMMVD ? (geoMMVDIdx - (step << 3)) : (geoMMVDIdx - (step << 2)));

  int mmvdStepToIdx[GPM_EXT_MMVD_REFINE_STEP] = { 5, 0, 1, 2, 3, 4, 6, 7, 8 };
  step                                        = mmvdStepToIdx[step];

  int numCandminus1Step = (extMMVD ? GPM_EXT_MMVD_REFINE_STEP : GPM_MMVD_REFINE_STEP) - 1;
  if (numCandminus1Step > 0)
  {
    if (step == 0)
    {
      m_binEncoder.encodeBin(0, Ctx::GeoMmvdStepMvpIdx());
    }
    else
    {
      m_binEncoder.encodeBin(1, Ctx::GeoMmvdStepMvpIdx());
      for (unsigned idx = 1; idx < numCandminus1Step; idx++)
      {
        m_binEncoder.encodeBinEP(step == idx ? 0 : 1);
        if (step == idx)
        {
          break;
        }
      }
    }
  }

  int maxMMVDDir = (extMMVD ? GPM_EXT_MMVD_REFINE_DIRECTION : GPM_MMVD_REFINE_DIRECTION);
  m_binEncoder.encodeBinsEP(direction, maxMMVDDir > 4 ? 3 : 2);
  DTRACE(g_trace_ctx, D_SYNTAX, "geo_mmvd_idx() L%d: %d\n", eRefPicList, cu.geoMMVDIdx[eRefPicList]);
}

void CABACWriter::geo_merge_idx(const CodingUnit &cu)
{
  uint8_t candIdx0 = cu.geoMergeIdx[0];
  uint8_t candIdx1 = cu.geoMergeIdx[1];
  CHECK(candIdx0 == candIdx1, "invalid combination");

  xWriteTruncBinCode(cu.geoSplitDir, GEO_NUM_PARTITION_MODE);
  DTRACE(g_trace_ctx, D_SYNTAX, "geo_split_dir %d\n", cu.geoSplitDir);

  candIdx1 -= candIdx1 < candIdx0 ? 0 : 1;

  const int numCandminus2 = cu.cs->sps->m_maxNumGeoCand - 2;
  m_binEncoder.encodeBin(candIdx0 == 0 ? 0 : 1, Ctx::MergeIdx());
  if (candIdx0 > 0)
  {
    unary_max_eqprob(candIdx0 - 1, numCandminus2);
  }
  if (numCandminus2 > 0)
  {
    m_binEncoder.encodeBin(candIdx1 == 0 ? 0 : 1, Ctx::MergeIdx());
    if (candIdx1 > 0)
    {
      unary_max_eqprob(candIdx1 - 1, numCandminus2 - 1);
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "geo_merge_idx() (%d, %d)\n", cu.geoMergeIdx[0], cu.geoMergeIdx[1]);
}

void CABACWriter::geo_merge_idx1(const CodingUnit &cu)
{
  uint8_t candIdx0 = cu.geoMergeIdx[0];
  uint8_t candIdx1 = cu.geoMergeIdx[1];
  uint8_t splitDir = cu.geoSplitDir;
  bool    isIntra0 = (candIdx0 >= GEO_MAX_NUM_UNI_CANDS);
  bool    isIntra1 = (candIdx1 >= GEO_MAX_NUM_UNI_CANDS);

  xWriteTruncBinCode(splitDir, GEO_NUM_PARTITION_MODE);
  DTRACE(g_trace_ctx, D_SYNTAX, "geo_split_dir %d\n", cu.geoSplitDir);

  int numCandminus2 = cu.cs->sps->m_maxNumGeoCand - 2;
  if (isIntra0)
  {
    unary_max_eqprob(candIdx0 - GEO_MAX_NUM_UNI_CANDS, GEO_MAX_NUM_INTRA_CANDS - 1);
  }
  else if (numCandminus2 >= 0)
  {
    m_binEncoder.encodeBin(candIdx0 == 0 ? 0 : 1, Ctx::MergeIdx());
    if (candIdx0 > 0)
    {
      unary_max_eqprob(candIdx0 - 1, numCandminus2);
    }
  }

  if (isIntra1)
  {
    unary_max_eqprob(candIdx1 - GEO_MAX_NUM_UNI_CANDS, GEO_MAX_NUM_INTRA_CANDS - 1);
  }
  else if (numCandminus2 >= 0)
  {
    m_binEncoder.encodeBin(candIdx1 == 0 ? 0 : 1, Ctx::MergeIdx());
    if (candIdx1 > 0)
    {
      unary_max_eqprob(candIdx1 - 1, numCandminus2);
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "geo_merge_idx1() (%d, %d)\n", cu.geoMergeIdx[0], cu.geoMergeIdx[1]);
}

double CABACWriter::geo_mode_est(const TempCtx &ctxStart, const int geoMode)
{
  getCtx() = ctxStart;
  resetBits();

  xWriteTruncBinCode(geoMode, GEO_NUM_PARTITION_MODE);

  return (double)getEstFracBits();
}

double CABACWriter::geo_mergeIdx_est(const TempCtx &ctxStart, const int candIdx, const int maxNumGeoCand)
{
  getCtx() = ctxStart;
  resetBits();

  int numCandminus2 = maxNumGeoCand - 2;
  m_binEncoder.encodeBin(candIdx == 0 ? 0 : 1, Ctx::MergeIdx());
  if (candIdx > 0)
  {
    unary_max_eqprob(candIdx - 1, numCandminus2);
  }

  return (double)getEstFracBits();
}

double CABACWriter::geo_mmvdFlag_est(const TempCtx &ctxStart, const int flag)
{
  getCtx() = ctxStart;
  resetBits();

  m_binEncoder.encodeBin(flag, Ctx::GeoMmvdFlag());

  return (double)getEstFracBits();
}

double CABACWriter::geo_mmvdIdx_est(const TempCtx &ctxStart, const int geoMMVDIdx, const bool extMMVD)
{
  getCtx() = ctxStart;
  resetBits();

  CHECK(geoMMVDIdx >= (extMMVD ? GPM_EXT_MMVD_MAX_REFINE_NUM : GPM_MMVD_MAX_REFINE_NUM),
        "invalid GPM MMVD index exist");
  int step      = (extMMVD ? (geoMMVDIdx >> 3) : (geoMMVDIdx >> 2));
  int direction = (extMMVD ? (geoMMVDIdx - (step << 3)) : (geoMMVDIdx - (step << 2)));

  int mmvdStepToIdx[GPM_EXT_MMVD_REFINE_STEP] = { 5, 0, 1, 2, 3, 4, 6, 7, 8 };
  step                                        = mmvdStepToIdx[step];

  int numCandminus1Step = (extMMVD ? GPM_EXT_MMVD_REFINE_STEP : GPM_MMVD_REFINE_STEP) - 1;
  if (numCandminus1Step > 0)
  {
    if (step == 0)
    {
      m_binEncoder.encodeBin(0, Ctx::GeoMmvdStepMvpIdx());
    }
    else
    {
      m_binEncoder.encodeBin(1, Ctx::GeoMmvdStepMvpIdx());
      for (unsigned idx = 1; idx < numCandminus1Step; idx++)
      {
        m_binEncoder.encodeBinEP(step == idx ? 0 : 1);
        if (step == idx)
        {
          break;
        }
      }
    }
  }
  int maxMMVDDir = (extMMVD ? GPM_EXT_MMVD_REFINE_DIRECTION : GPM_MMVD_REFINE_DIRECTION);
  m_binEncoder.encodeBinsEP(direction, maxMMVDDir > 4 ? 3 : 2);
  return (double)getEstFracBits();
}

double CABACWriter::geo_intraFlag_est(const TempCtx &ctxStart, const int intraFlag)
{
  getCtx() = ctxStart;
  resetBits();
  m_binEncoder.encodeBin(intraFlag, Ctx::GPMIntraFlag());
  return (double)getEstFracBits();
}

uint64_t CABACWriter::geo_bld_flag_est(const TempCtx &ctxStart, const int flag)
{
  getCtx() = ctxStart;
  resetBits();

  geo_adaptive_blending_idx(flag);

  return getEstFracBits();
}

void CABACWriter::geo_adaptive_blending_idx(const int flag)
{
  if (flag == 2)
  {
    m_binEncoder.encodeBin(1, Ctx::GeoBldFlag(0));
  }
  else
  {
    m_binEncoder.encodeBin(0, Ctx::GeoBldFlag(0));
    if (flag == 0 || flag == 1)
    {
      m_binEncoder.encodeBin(1, Ctx::GeoBldFlag(1));
      m_binEncoder.encodeBin(flag == 0, Ctx::GeoBldFlag(2));
    }
    else
    {
      m_binEncoder.encodeBin(0, Ctx::GeoBldFlag(1));
      m_binEncoder.encodeBin(flag == 3, Ctx::GeoBldFlag(3));
    }
  }
}

void CABACWriter::bm_merge_flag(const CodingUnit &cu)
{
  if (!PU::isBMMergeFlagCoded(cu))
  {
    return;
  }

  unsigned ctxId = DeriveCtx::CtxBMMrgFlag(cu);
  m_binEncoder.encodeBin(cu.bmMergeFlag, Ctx::BMMergeFlag(ctxId));
  if (cu.bmMergeFlag)
  {
    CHECK(cu.bmDir != 1 && cu.bmDir != 2, "cu.bmDir != 1 && cu.bmDir != 2");
    m_binEncoder.encodeBin(cu.bmDir >> 1, Ctx::BMMergeFlag(3));
  }

  DTRACE(g_trace_ctx, D_SYNTAX, "bm_merge_flag() bmMergeFlag=%d, bmDir=%d\n", cu.bmMergeFlag ? 1 : 0, cu.bmDir);
}

void CABACWriter::inter_pred_idc(const CodingUnit &cu)
{
  if (!cu.cs->slice->isInterB())
  {
    return;
  }
  if (!(PU::isBipredRestriction(cu)))
  {
    unsigned ctxId = DeriveCtx::CtxInterDir(cu);
    if (cu.interDir == 3)
    {
      m_binEncoder.encodeBin(1, Ctx::InterDir(ctxId));
      DTRACE(g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=%d value=%d pos=(%d,%d)\n", ctxId, cu.interDir,
             cu.lumaPos().x, cu.lumaPos().y);
      return;
    }
    else
    {
      m_binEncoder.encodeBin(0, Ctx::InterDir(ctxId));
    }
  }
  m_binEncoder.encodeBin((cu.interDir == 2), Ctx::InterDir(7));
  DTRACE(g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=7 value=%d pos=(%d,%d)\n", cu.interDir, cu.lumaPos().x,
         cu.lumaPos().y);
}

void CABACWriter::ref_idx(const CodingUnit &cu, RefPicList eRefList)
{
  if (cu.smvdMode)
  {
    CHECK(cu.refIdx[eRefList] != cu.cs->slice->getSymRefIdx(eRefList), "Invalid reference index!\n");
    return;
  }

  int numRef = cu.cs->slice->m_numRefIdx[eRefList];

  if (eRefList == RPL0 && cu.slice->m_ibcFlag)
  {
    if (CU::isIBC(cu))
    {
      return;
    }
  }

  if (numRef <= 1)
  {
    return;
  }
  int refIdx = cu.refIdx[eRefList];
  m_binEncoder.encodeBin((refIdx > 0), Ctx::RefPic());
  if (numRef <= 2 || refIdx == 0)
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, cu.lumaPos().x, cu.lumaPos().y);
    return;
  }
  m_binEncoder.encodeBin((refIdx > 1), Ctx::RefPic(1));
  if (numRef <= 3 || refIdx == 1)
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, cu.lumaPos().x, cu.lumaPos().y);
    return;
  }
  for (int idx = 3; idx < numRef; idx++)
  {
    if (refIdx > idx - 1)
    {
      m_binEncoder.encodeBinEP(1);
    }
    else
    {
      m_binEncoder.encodeBinEP(0);
      break;
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, cu.lumaPos().x, cu.lumaPos().y);
}

void CABACWriter::mvp_flag(const CodingUnit &cu, RefPicList eRefList)
{
  m_binEncoder.encodeBin(cu.mvpIdx[eRefList], Ctx::MVPIdx());
  DTRACE(g_trace_ctx, D_SYNTAX, "mvp_flag() value=%d pos=(%d,%d)\n", cu.mvpIdx[eRefList], cu.lumaPos().x,
         cu.lumaPos().y);
  DTRACE(g_trace_ctx, D_SYNTAX, "mvpIdx(refList:%d)=%d\n", eRefList, cu.mvpIdx[eRefList]);
}

void CABACWriter::ciip_flag(const CodingUnit &cu, bool usageIsInferred)
{
  if (!cu.cs->sps->m_useCiip || cu.skip)
  {
    CHECK(cu.ciipFlag == true, "invalid ciip SPS or skip");
    DTRACE(g_trace_ctx, D_SYNTAX, "ciip_flag() ciip=0\n");
    return;
  }

  if (!usageIsInferred)
  {
    m_binEncoder.encodeBin(cu.ciipFlag == true, Ctx::CiipFlag(0));
  }
  if (cu.ciipFlag)
  {
    m_binEncoder.encodeBin(cu.ciipMode != CIIP_Type::NORMAL, Ctx::CiipFlag(1));
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "ciip_flag() ciip=%d\n", cu.ciipFlag ? 1 : 0);
}

void CABACWriter::lfCccm(const CodingStructure &cs, const uint32_t ctuRsAddr)
{
  if (!cs.sps->m_lfCccmEnabledFlag)
  {
    return;
  }
  if (!cs.slice->m_lfCccmEnabledFlag)
  {
    return;
  }
  if (ctuRsAddr == 0 && !cs.slice->isIntra() && cs.slice->lfCccmGetReferencePicture())
  {
    m_binEncoder.encodeBin(cs.slice->m_lfCccmFrameLevelInherit ? 1 : 0, Ctx::LfCccmFlag(1));
  }
  if (cs.slice->m_lfCccmFrameLevelInherit)
  {
    return;
  }
  m_binEncoder.encodeBin(cs.slice->m_lfCccmEnabled.at(ctuRsAddr) ? 1 : 0, Ctx::LfCccmFlag(0));
  if (cs.slice->m_lfCccmEnabled.at(ctuRsAddr))
  {
    const int nCand = (int)cs.slice->lfCccmGetMergeCandidates(ctuRsAddr).size();
    if (nCand)
    {
      m_binEncoder.encodeBin(cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr) ? 1 : 0, Ctx::LfCccmFlag(2));
      if (cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr) && nCand > 1)
      {
        m_binEncoder.encodeBin(cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr) == 3 ? 1 : 0, Ctx::LfCccmFlag(3));
      }
      if (cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr))
      {
        return;
      }
    }

    m_binEncoder.encodeBin(cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr) & 1, Ctx::LfCccmFlag(4));
    m_binEncoder.encodeBin((cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr) >> 1) & 1, Ctx::LfCccmFlag(5));
    m_binEncoder.encodeBin((cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr) >> 2) & 1, Ctx::LfCccmFlag(6));

    m_binEncoder.encodeBin(cs.slice->m_lfCccmModelType.at(ctuRsAddr) & 1, Ctx::LfCccmFlag(7));
    m_binEncoder.encodeBin((cs.slice->m_lfCccmModelType.at(ctuRsAddr) >> 1) & 1, Ctx::LfCccmFlag(8));
    m_binEncoder.encodeBin((cs.slice->m_lfCccmModelType.at(ctuRsAddr) >> 2) & 1, Ctx::LfCccmFlag(9));
  }
}

//================================================================================
//  clause 7.3.8.8
//--------------------------------------------------------------------------------
//    void  transform_tree      ( cs, area, cuCtx, chromaCbfs )
//    bool  split_transform_flag( split, depth )
//    bool  cbf_comp            ( cbf, area, depth )
//================================================================================
void CABACWriter::transform_tree(const CodingStructure &cs, Partitioner &partitioner, CUCtx &cuCtx)
{
  const UnitArea      &area    = partitioner.currArea();
  const TransformUnit &tu      = *cs.getTU(area.block(partitioner.chType).pos(), partitioner.chType);
  const CodingUnit    &cu      = *tu.cu;
  const unsigned       trDepth = partitioner.currTrDepth;
  const bool           split   = (tu.depth > trDepth);

  // split_transform_flag
  if (partitioner.canSplit(TU_MAX_TR_SPLIT, cs))
  {
    CHECK(!split, "transform split implied");
  }
  else if (cu.sbtInfo && partitioner.canSplit(PartSplit(cu.getSbtTuSplit()), cs))
  {
    CHECK(!split, "transform split implied - sbt");
  }
  else
  {
    CHECK(split, "transform split not allowed with QTBT");
  }

  if (split)
  {
    if (partitioner.canSplit(TU_MAX_TR_SPLIT, cs))
    {
#if ENABLE_TRACING
      const CompArea &tuArea = partitioner.currArea().block(partitioner.chType);
      DTRACE(g_trace_ctx, D_SYNTAX, "transform_tree() maxTrSplit chType=%d pos=(%d,%d) size=%dx%d\n",
             partitioner.chType, tuArea.x, tuArea.y, tuArea.width, tuArea.height);

#endif
      partitioner.splitCurrArea(TU_MAX_TR_SPLIT, cs);
    }
    else if (cu.sbtInfo && partitioner.canSplit(PartSplit(cu.getSbtTuSplit()), cs))
    {
      partitioner.splitCurrArea(PartSplit(cu.getSbtTuSplit()), cs);
    }
    else
    {
      THROW("Implicit TU split not available");
    }

    do
    {
      transform_tree(cs, partitioner, cuCtx);
    } while (partitioner.nextPart(cs));

    partitioner.exitCurrSplit();
  }
  else
  {
    transform_unit_last(tu, cuCtx, partitioner);
    nst_idx(tu, cuCtx);
    transform_unit_coef(tu, cuCtx, partitioner);
    mts_idx(tu, cuCtx);
    transform_unit_sign(tu, partitioner);
  }
}

void CABACWriter::cbf_comp(bool cbf, const CompArea &area, unsigned depth, const bool prevCbf,
                           const BdpcmMode bdpcmMode)
{
  unsigned      ctxId  = DeriveCtx::CtxQtCbf(area.compID, prevCbf);
  const CtxSet &ctxSet = Ctx::QtCbf[area.compID];

  if (bdpcmMode != BdpcmMode::NONE)
  {
    ctxId = area.compID == COMP_Cr ? 2 : 1;
  }

  m_binEncoder.encodeBin(cbf ? 1 : 0, ctxSet(ctxId));

  DTRACE(g_trace_ctx, D_SYNTAX, "cbf_comp() etype=%d pos=(%d,%d) ctx=%d cbf=%d\n", area.compID, area.x, area.y, ctxId,
         cbf ? 1 : 0);
}

//================================================================================
//  clause 7.3.8.9
//--------------------------------------------------------------------------------
//================================================================================
void CABACWriter::mvd_coding(const CodingUnit &cu, Mv mvd, int amvr)
{
  if (CU::isIBC(cu))
  {
    mvd.changeIbcPrecInternal2Amvr(amvr);
  }
  else if (cu.affine)
  {
    mvd.changeAffinePrecInternal2Amvr(amvr);
  }
  else
  {
    mvd.changeTransPrecInternal2Amvr(amvr);
  }
  const int          horMvd = mvd.getHor();
  const int          verMvd = mvd.getVer();
  const unsigned int horAbs = std::abs(horMvd);
  const unsigned int verAbs = std::abs(verMvd);

  // abs_mvd_greater0_flag[ 0 | 1 ]
  m_binEncoder.encodeBin((horAbs > 0), Ctx::Mvd());
  m_binEncoder.encodeBin((verAbs > 0), Ctx::Mvd());

  // abs_mvd_greater1_flag[ 0 | 1 ]
  if (horAbs > 0)
  {
    m_binEncoder.encodeBin((horAbs > 1), Ctx::Mvd(1));
  }
  if (verAbs > 0)
  {
    m_binEncoder.encodeBin((verAbs > 1), Ctx::Mvd(1));
  }

  // abs_mvd_minus2[ 0 | 1 ] and mvd_sign_flag[ 0 | 1 ]
  if (horAbs > 0)
  {
    if (horAbs > 1)
    {
      m_binEncoder.encodeRemAbsEP(horAbs - 2, 1, 0, MV_BITS - 1);
    }
    m_binEncoder.encodeBinEP((horMvd < 0));
  }
  if (verAbs > 0)
  {
    if (verAbs > 1)
    {
      m_binEncoder.encodeRemAbsEP(verAbs - 2, 1, 0, MV_BITS - 1);
    }
    m_binEncoder.encodeBinEP((verMvd < 0));
  }
}

void CABACWriter::xWriteBvdContext(unsigned uiSymbol, unsigned ctxT, int offset, int param)
{
  unsigned bins    = 0;
  unsigned numBins = 0;
  while (uiSymbol >= (unsigned)(1 << param))
  {
    bins <<= 1;
    bins++;
    numBins++;
    uiSymbol -= 1 << param;
    param++;
  }

  bins <<= 1;
  numBins++;

  unsigned temp     = 0;
  unsigned bitCount = 0;
  for (int i = numBins - 1; i >= 0; i--)
  {
    temp = bins >> i;
    if (bitCount >= ctxT)
    {
      m_binEncoder.encodeBinEP(temp);
    }
    else
    {
      m_binEncoder.encodeBin(temp, Ctx::Bvd(offset + bitCount + 1));
    }
    bins -= (temp << i);
    bitCount++;
  }
  m_binEncoder.encodeBinsEP(uiSymbol, param);
}

void CABACWriter::bvdCoding(const Mv &rMvd)
{
  int horMvd = rMvd.getHor();
  int verMvd = rMvd.getVer();

  unsigned horAbs = unsigned(horMvd < 0 ? -horMvd : horMvd);
  unsigned verAbs = unsigned(verMvd < 0 ? -verMvd : verMvd);

  m_binEncoder.encodeBin((horAbs > 0), Ctx::Bvd(HOR_BVD_CTX_OFFSET));
  m_binEncoder.encodeBin((verAbs > 0), Ctx::Bvd(VER_BVD_CTX_OFFSET));

  if (horAbs > 0)
  {
    xWriteBvdContext(horAbs - 1, NUM_HOR_BVD_CTX, HOR_BVD_CTX_OFFSET, BVD_CODING_GOLOMB_ORDER);
    m_binEncoder.encodeBinEP((horMvd < 0));
  }
  if (verAbs > 0)
  {
    xWriteBvdContext(verAbs - 1, NUM_VER_BVD_CTX, VER_BVD_CTX_OFFSET, BVD_CODING_GOLOMB_ORDER);
    m_binEncoder.encodeBinEP((verMvd < 0));
  }
}
//================================================================================
//  clause 7.3.8.10
//--------------------------------------------------------------------------------
//    void  transform_unit      ( tu, cuCtx, chromaCbfs )
//    void  cu_qp_delta         ( cu )
//    void  cu_chroma_qp_offset ( cu )
//================================================================================
void CABACWriter::transform_unit_last(const TransformUnit &tu, CUCtx &cuCtx, Partitioner &partitioner)
{
  DTRACE(g_trace_ctx, D_SYNTAX, "transform_unit_last() pos=(%d,%d) size=%dx%d depth=%d trDepth=%d\n",
         tu.block(tu.chType).x, tu.block(tu.chType).y, tu.block(tu.chType).width, tu.block(tu.chType).height,
         tu.cu->depth, partitioner.currTrDepth);

  const CodingUnit &cu      = *tu.cu;
  const UnitArea   &area    = partitioner.currArea();
  const unsigned    trDepth = partitioner.currTrDepth;
  ChromaCbfs        chromaCbfs;
  CHECK(tu.depth != trDepth, " transform unit should be not be futher partitioned");

  // cbf_cb & cbf_cr
  if (isChromaEnabled(area.chromaFormat))
  {
    if (area.blocks[COMP_Cb].valid() && (!CS::isDualITree(*cu.cs) || partitioner.chType == ChannelType::CHROMA))
    {
      chromaCbfs.Cb = TU::getCbfAtDepth(tu, COMP_Cb, trDepth);
      if (!(cu.sbtInfo && tu.noResidual))
      {
        cbf_comp(chromaCbfs.Cb, area.blocks[COMP_Cb], trDepth, false, cu.getBdpcmMode(COMP_Cb));
      }

      chromaCbfs.Cr = TU::getCbfAtDepth(tu, COMP_Cr, trDepth);
      if (!(cu.sbtInfo && tu.noResidual))
      {
        cbf_comp(chromaCbfs.Cr, area.blocks[COMP_Cr], trDepth, chromaCbfs.Cb, cu.getBdpcmMode(COMP_Cr));
      }
    }
    else if (CS::isDualITree(*cu.cs))
    {
      chromaCbfs = ChromaCbfs(false);
    }
  }
  else if (CS::isDualITree(*cu.cs))
  {
    chromaCbfs = ChromaCbfs(false);
  }

  if (!isChroma(partitioner.chType))
  {
    if (!CU::isIntra(cu) && trDepth == 0 && !chromaCbfs.sigChroma(area.chromaFormat))
    {
      CHECK(!TU::getCbfAtDepth(tu, COMP_Y, trDepth), "Luma cbf must be true for inter units with no chroma coeffs");
    }
    else if (cu.sbtInfo && tu.noResidual)
    {
      CHECK(TU::getCbfAtDepth(tu, COMP_Y, trDepth), "Luma cbf must be false for inter sbt no-residual tu");
    }
    else if (cu.sbtInfo && !chromaCbfs.sigChroma(area.chromaFormat))
    {
      assert(!tu.noResidual);
      CHECK(!TU::getCbfAtDepth(tu, COMP_Y, trDepth), "Luma cbf must be true for inter sbt residual tu");
    }
    else
    {
      cbf_comp(TU::getCbfAtDepth(tu, COMP_Y, trDepth), tu.Y(), trDepth, false, cu.getBdpcmMode(COMP_Y));
    }
  }
  bool lumaOnly  = !isChromaEnabled(cu.chromaFormat) || !tu.blocks[COMP_Cb].valid();
  bool cbf[3]    = { TU::getCbf(tu, COMP_Y), chromaCbfs.Cb, chromaCbfs.Cr };
  bool cbfLuma   = (cbf[COMP_Y] != 0);
  bool cbfChroma = false;

  if (!lumaOnly)
  {
    if (tu.blocks[COMP_Cb].valid())
    {
      cbf[COMP_Cb] = TU::getCbf(tu, COMP_Cb);
      cbf[COMP_Cr] = TU::getCbf(tu, COMP_Cr);
    }
    cbfChroma = (cbf[COMP_Cb] || cbf[COMP_Cr]);
  }

  if ((cu.lwidth() > MAX_TB_SIZEY || cu.lheight() > MAX_TB_SIZEY || cbfLuma || cbfChroma) &&
      (!CS::isDualITree(*cu.cs) || isLuma(tu.chType)))
  {
    if (cu.cs->pps->m_useDQP && !cuCtx.isDQPCoded)
    {
      cu_qp_delta(cu, cuCtx.qp, cu.qp);
      cuCtx.qp         = cu.qp;
      cuCtx.isDQPCoded = true;
    }
  }
  if (!CS::isDualITree(*tu.cs) || isChroma(tu.chType))   // !DUAL_TREE_LUMA
  {
    SizeType channelWidth  = !CS::isDualITree(*tu.cs) ? cu.lwidth() : cu.chromaSize().width;
    SizeType channelHeight = !CS::isDualITree(*tu.cs) ? cu.lheight() : cu.chromaSize().height;

    if (cu.cs->slice->m_chromaQpAdjEnabled &&
        (channelWidth > MAX_TB_SIZEY || channelHeight > MAX_TB_SIZEY || cbfChroma) && !cuCtx.isChromaQpAdjCoded)
    {
      cu_chroma_qp_offset(cu);
      cuCtx.isChromaQpAdjCoded = true;
    }
  }

  if (!lumaOnly)
  {
    joint_cb_cr(tu, (cbf[COMP_Cb] ? 2 : 0) + (cbf[COMP_Cr] ? 1 : 0));
  }

  if (cbfLuma)
  {
    residual_coding_last(tu, COMP_Y, &cuCtx);
  }
  if (!lumaOnly)
  {
    for (CompID compID = COMP_Cb; compID <= COMP_Cr; compID = CompID(compID + 1))
    {
      if (cbf[compID])
      {
        residual_coding_last(tu, compID, &cuCtx);
      }
    }
  }
}

void CABACWriter::transform_unit_coef(const TransformUnit &tu, CUCtx &cuCtx, Partitioner &partitioner)
{
  DTRACE(g_trace_ctx, D_SYNTAX, "transform_unit_coef() pos=(%d,%d) size=%dx%d depth=%d trDepth=%d\n",
         tu.block(tu.chType).x, tu.block(tu.chType).y, tu.block(tu.chType).width, tu.block(tu.chType).height,
         tu.cu->depth, partitioner.currTrDepth);

  if (TU::getCbf(tu, COMP_Y))
  {
    residual_coding_coef(tu, COMP_Y, &cuCtx);
  }
  if (!(tu.cu->chromaFormat == ChromaFormat::_400 || !tu.blocks[COMP_Cb].valid()))
  {
    for (CompID compID = COMP_Cb; compID <= COMP_Cr; compID = CompID(compID + 1))
    {
      if (TU::getCbf(tu, compID))
      {
        residual_coding_coef(tu, compID, &cuCtx);
      }
    }
  }
}

void CABACWriter::transform_unit_sign(const TransformUnit &tu, Partitioner &partitioner)
{
  DTRACE(g_trace_ctx, D_SYNTAX, "transform_unit_sign() pos=(%d,%d) size=%dx%d depth=%d trDepth=%d\n",
         tu.block(tu.chType).x, tu.block(tu.chType).y, tu.block(tu.chType).width, tu.block(tu.chType).height,
         tu.cu->depth, partitioner.currTrDepth);

  for (int compIdx = 0; compIdx < tu.blocks.size(); compIdx++)
  {
    CompID compID = CompID(compIdx);
    if (tu.jointCbCr)
    {
      if (!(tu.jointCbCr >> 1) && compID == CompID::COMP_Cb)
      {
        continue;
      }
      if ((tu.jointCbCr >> 1) && compID == CompID::COMP_Cr)
      {
        continue;
      }
    }
    if (tu.blocks[compID].valid() && TU::getCbf(tu, compID) && TU::getDelayedSignCoding(tu, compID))
    {
      residual_coding_sign(tu, compID);
    }
  }
}

void CABACWriter::cu_qp_delta(const CodingUnit &cu, int predQP, const int8_t qp)
{
  CHECK(!(predQP != std::numeric_limits<int>::max()), "Unspecified error");
  int DQp         = qp - predQP;
  int qpBdOffsetY = cu.cs->sps->m_qpBDOffset[ChannelType::LUMA];
  DQp = (DQp + (MAX_QP + 1) + (MAX_QP + 1) / 2 + qpBdOffsetY + (qpBdOffsetY / 2)) % ((MAX_QP + 1) + qpBdOffsetY) -
    (MAX_QP + 1) / 2 - (qpBdOffsetY / 2);
  unsigned absDQP   = unsigned(DQp < 0 ? -DQp : DQp);
  unsigned unaryDQP = std::min<unsigned>(absDQP, CU_DQP_TU_CMAX);

  unary_max_symbol(unaryDQP, Ctx::DeltaQP(), Ctx::DeltaQP(1), CU_DQP_TU_CMAX);
  if (absDQP >= CU_DQP_TU_CMAX)
  {
    exp_golomb_eqprob(absDQP - CU_DQP_TU_CMAX, CU_DQP_EG_k);
  }
  if (absDQP > 0)
  {
    m_binEncoder.encodeBinEP(DQp < 0);
  }

  DTRACE_COND((isEncoding()), g_trace_ctx, D_DQP, "x=%d, y=%d, d=%d, pred_qp=%d, DQp=%d, qp=%d\n",
              cu.block(cu.chType).lumaPos().x, cu.block(cu.chType).lumaPos().y, cu.qtDepth, predQP, DQp, qp);
}

void CABACWriter::cu_chroma_qp_offset(const CodingUnit &cu)
{
  // cu_chroma_qp_offset_flag
  unsigned qpAdj = cu.chromaQpAdj;
  if (qpAdj == 0)
  {
    m_binEncoder.encodeBin(0, Ctx::ChromaQpAdjFlag());
  }
  else
  {
    m_binEncoder.encodeBin(1, Ctx::ChromaQpAdjFlag());
    int length = cu.cs->pps->m_chromaQpOffsetListLen;
    if (length > 1)
    {
      unary_max_symbol(qpAdj - 1, Ctx::ChromaQpAdjIdc(), Ctx::ChromaQpAdjIdc(), length - 1);
    }
  }
}

//================================================================================
//  clause 7.3.8.11
//--------------------------------------------------------------------------------
//    void        residual_coding         ( tu, compID )
//    void        transform_skip_flag     ( tu, compID )
//    void        last_sig_coeff          ( coeffCtx )
//    void        residual_coding_subblock( coeffCtx )
//================================================================================

void CABACWriter::joint_cb_cr(const TransformUnit &tu, const int cbfMask)
{
  if (!tu.cu->slice->m_sps->m_jointCbCrEnabledFlag)
  {
    return;
  }

  CHECK(tu.jointCbCr && tu.jointCbCr != cbfMask,
        "wrong value of jointCbCr (" << (int)tu.jointCbCr << " vs " << (int)cbfMask << ")");
  if ((CU::isIntra(*tu.cu) && cbfMask != 0) || cbfMask == CBF_MASK_CBCR)
  {
    m_binEncoder.encodeBin(tu.jointCbCr ? 1 : 0, Ctx::JointCbCrFlag(cbfMask - 1));
  }
}

void CABACWriter::residual_coding_last(const TransformUnit &tu, CompID compID, CUCtx *cuCtx)
{
  const CodingUnit &cu = *tu.cu;
  DTRACE(g_trace_ctx, D_SYNTAX, "residual_coding_last() etype=%d pos=(%d,%d) size=%dx%d predMode=%d\n",
         tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width,
         tu.blocks[compID].height, cu.predMode);

  if (compID == COMP_Cr && tu.jointCbCr == 3)
  {
    return;
  }

  ts_flag(tu, compID);

  if (tu.mtsIdx[compID] == MtsType::SKIP && !tu.cs->slice->m_tsResidualCodingDisabledFlag)
  {
    return;
  }

  // determine sign hiding
  bool signHiding = cu.cs->slice->m_signDataHidingEnabledFlag;

  // init coeff coding context
  CoeffCodingContext cctx(tu, compID, signHiding, BdpcmMode::NONE);
  const TCoeff      *coeff = tu.getCoeffs(compID).buf;

  if (tu.lastPos[compID] < 0)
  {
    int lastPos = -1;
    for (int scanPos = 0; scanPos < cctx.maxNumCoeff(); scanPos++)
    {
      unsigned blkPos = cctx.blockPos(scanPos);
      if (coeff[blkPos])
      {
        lastPos = scanPos;
      }
    }
    const_cast<TransformUnit &>(tu).lastPos[compID] = lastPos;
  }
  CHECK(tu.lastPos[compID] < 0, "Coefficient coding called for empty TU");
  cctx.setScanPosLast(tu.lastPos[compID]);

  if (cuCtx && tu.mtsIdx[compID] != MtsType::SKIP && tu.blocks[compID].height >= 4 && tu.blocks[compID].width >= 4)
  {
    const uint32_t width     = tu.blocks[compID].width;
    const uint32_t height    = tu.blocks[compID].height;
    const bool     allowNSPT = TU::isNSPTAllowed(width, height);
    const int      maxLfnstPos =
      (allowNSPT ? PU::getNSPTMatrixDim(width, height) : PU::getLFNSTMatrixDim(width, height)) - 1;
    cuCtx->violatesLfnstConstrained[toChannelType(compID)] |= cctx.scanPosLast() > maxLfnstPos;
  }
  if (cuCtx && tu.mtsIdx[compID] != MtsType::SKIP && tu.blocks[compID].height >= 4 && tu.blocks[compID].width >= 4)
  {
    const int lfnstLastScanPosTh = isLuma(compID) ? LFNST_LAST_SIG_LUMA : LFNST_LAST_SIG_CHROMA;
    cuCtx->lfnstLastScanPos |= cctx.scanPosLast() >= lfnstLastScanPosTh;
  }
  if (cuCtx && isLuma(compID) && tu.mtsIdx[compID] != MtsType::SKIP)
  {
    cuCtx->mtsLastScanPos |= cctx.scanPosLast() >= 1;
    cuCtx->mtsCoeffAbsSum = tu.getCoeffs(compID).computeAbsSum();
  }

  // code last coeff position
  last_sig_coeff(cctx, tu, compID);
}

void CABACWriter::residual_coding_coef(const TransformUnit &tu, CompID compID, CUCtx *cuCtx)
{
  const CodingUnit &cu = *tu.cu;
  DTRACE(g_trace_ctx, D_SYNTAX, "residual_coding_coef() etype=%d pos=(%d,%d) size=%dx%d predMode=%d\n",
         tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width,
         tu.blocks[compID].height, cu.predMode);

  if (compID == COMP_Cr && tu.jointCbCr == 3)
  {
    return;
  }

  if (tu.mtsIdx[compID] == MtsType::SKIP && !tu.cs->slice->m_tsResidualCodingDisabledFlag)
  {
    residual_codingTS(tu, compID);
    return;
  }

  // determine sign hiding
  bool signHiding = cu.cs->slice->m_signDataHidingEnabledFlag;

  // init coeff coding context
  CoeffCodingContext cctx(tu, compID, signHiding, BdpcmMode::NONE);
  const TCoeff      *coeff = tu.getCoeffs(compID).buf;

  // determine and set last coeff position and sig group flags
  std::bitset<MLS_GRP_NUM> sigGroupFlags;
  int                      scanPos = 0;
  int                      sbbSize = 1 << cctx.log2CGWidth();
  while (scanPos <= tu.lastPos[compID])
  {
    unsigned blkPos = cctx.blockPos(scanPos);
    if (coeff[blkPos])
    {
      sigGroupFlags.set(scanPos >> cctx.log2CGSize());
      scanPos += sbbSize - (scanPos & (sbbSize - 1));
    }
    else
    {
      scanPos++;
    }
  }
  CHECK(tu.lastPos[compID] < 0, "Coefficient coding called for empty TU");
  cctx.setScanPosLast(tu.lastPos[compID]);

  // code subblocks
  const uint64_t stateTransTab = g_stateTransTab[tu.cs->slice->m_depQuantEnabledIdc];
  int            state         = 0;

  int ctxBinSampleRatio =
    (compID == COMP_Y) ? MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_LUMA : MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_CHROMA;
  cctx.remRegBins = (tu.getTbAreaAfterCoefZeroOut(compID) * ctxBinSampleRatio) >> 4;

  int baseLevel = m_binEncoder.getCtx().getBaseLevel();
  cctx.setBaseLevel(baseLevel);
  if (tu.cs->slice->m_sps->m_spsRangeExtension.m_persistentRiceAdaptationEnabledFlag)
  {
    cctx.setUpdateHist(1);
    unsigned riceStats    = m_binEncoder.getCtx().getGRAdaptStats((unsigned)compID);
    TCoeff   historyValue = (TCoeff)1 << riceStats;
    cctx.setHistValue(historyValue);
  }
  for (int subSetId = (cctx.scanPosLast() >> cctx.log2CGSize()); subSetId >= 0; subSetId--)
  {
    cctx.initSubblock(subSetId, sigGroupFlags[subSetId]);

    residual_coding_subblock(cctx, coeff, stateTransTab, state);
  }
}

void CABACWriter::residual_coding_sign(const TransformUnit &tu, CompID compID)
{
  if (!TU::getDelayedSignCoding(tu, compID) || (compID == COMP_Cr && tu.jointCbCr == 3))
  {
    return;
  }
  const CtxSet  &ctx              = Ctx::SignPredFlag[to_underlying(toChannelType(compID))];
  const uint16_t ctxIdxGt1        = ctx(CU::isIntra(*tu.cu) ? 1 : 3);
  const uint16_t ctxIdxEq1        = ctx(CU::isIntra(*tu.cu) ? 0 : 2);
  const uint16_t numSignsPredArea = tu.numPredAreaSigns[compID] & ((1 << 16) - 1);
  const int32_t  maxNumPredSigns  = tu.maxNumPredSigns(compID);
  const uint8_t *signsPredArea    = tu.getSignsPredArea(compID);
#if ENABLE_TRACING
  const Size signPredArea = TU::getSignPredArea(tu, compID);
#endif

  //----- signs inside pred area -----
  const int numPredSignsGt1 = std::min<int>(maxNumPredSigns, tu.numPredAreaSigns[compID] >> 16);
  const int numPredSigns    = std::min<int>(maxNumPredSigns, numSignsPredArea);

  DTRACE(g_trace_ctx, D_SYNTAX, "residual_coding_sign() size=%dx%d signsGt1=%d signsEq1=%d signsMax=%d\n",
         signPredArea.width, signPredArea.height, numPredSignsGt1, maxNumPredSigns, maxNumPredSigns);

  for (int k = 0; k < numPredSignsGt1; k++)
  {
    m_binEncoder.encodeBin(signsPredArea[k], ctxIdxGt1);
  }
  for (int k = numPredSignsGt1; k < numPredSigns; k++)
  {
    m_binEncoder.encodeBin(signsPredArea[k], ctxIdxEq1);
  }
  for (int k = numPredSigns; k < numSignsPredArea; k++)
  {
    m_binEncoder.encodeBinEP(signsPredArea[k]);
  }
}

void CABACWriter::ts_flag(const TransformUnit &tu, CompID compID)
{
  const int tsFlag = tu.mtsIdx[compID] == MtsType::SKIP ? 1 : 0;
  int       ctxIdx = isLuma(compID) ? 0 : 1;

  if (TU::isTSAllowed(tu, compID) && tu.cu->getBdpcmMode(compID) == BdpcmMode::NONE)
  {
    m_binEncoder.encodeBin(tsFlag, Ctx::TransformSkipFlag(ctxIdx));
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "ts_flag() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMP_Y, tu.cu->lx(), tu.cu->ly(), tsFlag);
}

void CABACWriter::nst_idx(const TransformUnit &tu, CUCtx &cuCtx)
{
  CodingUnit &cu     = *tu.cu;
  ChannelType ch     = (CS::isDualITree(*cu.cs) && isChroma(cu.chType) ? ChannelType::CHROMA : ChannelType::LUMA);
  CompID      compId = (ch == ChannelType::LUMA ? CompID::COMP_Y : CompID::COMP_Cb);

  if (!TU::lfnstAllowed(tu, compId))
  {
    return;
  }
  if (ch == ChannelType::LUMA && tu.mtsIdx[COMP_Y] == MtsType::SKIP)
  {
    return;
  }
  if (ch == ChannelType::CHROMA && (tu.mtsIdx[COMP_Cb] == MtsType::SKIP || tu.mtsIdx[COMP_Cr] == MtsType::SKIP))
  {
    return;
  }
  if (!cuCtx.lfnstLastScanPos || cuCtx.violatesLfnstConstrained[ch])
  {
    return;
  }

  const uint32_t nstIdx = TU::getNstIdx(tu, compId);
  if (CU::isInter(cu))
  {
    assert(nstIdx < 4);
    m_binEncoder.encodeBin(nstIdx > 0, Ctx::LFNSTIdxInter(0));
    if (nstIdx > 0)
    {
      m_binEncoder.encodeBin(nstIdx > 1, Ctx::LFNSTIdxInter(1));
    }
    if (nstIdx > 1)
    {
      m_binEncoder.encodeBin(nstIdx > 2, Ctx::LFNSTIdxInter(2));
    }
  }
  else
  {
    assert(nstIdx < 4);
    uint32_t ctxIdx    = CS::isDualITree(*cu.cs) ? 1 : 0;
    uint32_t firstBin  = nstIdx & 1;
    uint32_t secondBin = (nstIdx >> 1) & 1;
    m_binEncoder.encodeBin(firstBin, Ctx::LFNSTIdxIntra(ctxIdx));
    m_binEncoder.encodeBin(secondBin, Ctx::LFNSTIdxIntra(2 + firstBin));
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "nst_idx() etype=%d pos=(%d,%d) nstIdx=%d\n", compId, cu.lx(), cu.ly(),
         TU::getNstIdx(tu, compId));
}

void CABACWriter::mts_idx(const TransformUnit &tu, CUCtx &cuCtx)
{
  CodingUnit &cu     = *tu.cu;
  MtsType     mtsIdx = tu.mtsIdx[COMP_Y];

  if (TU::isMTSAllowed(tu, COMP_Y) && !cuCtx.violatesMtsCoeffConstraint && cuCtx.mtsLastScanPos &&
      mtsIdx != MtsType::SKIP && TU::getNstIdx(tu, COMP_Y) == 0 && tu.cbf[COMP_Y] != 0)
  {
    int symbol = mtsIdx != MtsType::DCT2_DCT2 ? 1 : 0;
    int ctxIdx = 0;
    if (CU::isIntra(cu))
    {
      ctxIdx = (cuCtx.mtsCoeffAbsSum > MTS_TH_COEFF[1]) ? 2 : (cuCtx.mtsCoeffAbsSum > MTS_TH_COEFF[0]) ? 1 : 0;
    }

    m_binEncoder.encodeBin(symbol, Ctx::MTSIdx(ctxIdx));

    if (symbol)
    {
      int trIdx  = (tu.mtsIdx[CompID::COMP_Y] - MtsType::MTS_1);
      int nCands = CU::isIntra(cu) ? MTS_NCANDS[ctxIdx] : 4;
      CHECK(trIdx < 0 || trIdx >= nCands, "trIdx outside range");
      xWriteTruncBinCode(trIdx, nCands);
    }
  }

  DTRACE(g_trace_ctx, D_SYNTAX, "mts_idx() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMP_Y, tu.cu->lx(), tu.cu->ly(), mtsIdx);
}

void CABACWriter::last_sig_coeff(CoeffCodingContext &cctx, const TransformUnit &tu, CompID compID)
{
  unsigned blkPos = cctx.blockPos(cctx.scanPosLast());
  unsigned posX, posY;
  {
    posY = blkPos / cctx.width();
    posX = blkPos - (posY * cctx.width());
  }

  unsigned ctxLast;
  unsigned groupIdxX = g_groupIdx[posX];
  unsigned groupIdxY = g_groupIdx[posY];

  unsigned maxLastPosX = cctx.maxLastPosX();
  unsigned maxLastPosY = cctx.maxLastPosY();

  unsigned zoTbWdith  = getNonzeroTuSize(cctx.width());
  unsigned zoTbHeight = getNonzeroTuSize(cctx.height());

  if (isEncoding())
  {
    if ((posX + posY) > ((zoTbWdith + zoTbHeight + 2) / 2))
    {
      tu.cu->slice->m_cntRightBottom += 1;
    }
    else
    {
      tu.cu->slice->m_cntRightBottom -= 1;
    }
  }
  if (tu.cu->slice->m_reverseLastSigCoeffFlag)
  {
    posX = zoTbWdith - 1 - posX;
    posY = zoTbHeight - 1 - posY;

    groupIdxX = g_groupIdx[posX];
    groupIdxY = g_groupIdx[posY];
  }

  for (ctxLast = 0; ctxLast < groupIdxX; ctxLast++)
  {
    m_binEncoder.encodeBin(1, cctx.lastXCtxId(ctxLast));
  }
  if (groupIdxX < maxLastPosX)
  {
    m_binEncoder.encodeBin(0, cctx.lastXCtxId(ctxLast));
  }
  for (ctxLast = 0; ctxLast < groupIdxY; ctxLast++)
  {
    m_binEncoder.encodeBin(1, cctx.lastYCtxId(ctxLast));
  }
  if (groupIdxY < maxLastPosY)
  {
    m_binEncoder.encodeBin(0, cctx.lastYCtxId(ctxLast));
  }
  if (groupIdxX > 3)
  {
    const int  log2Width        = floorLog2(cctx.width());
    const bool maxValueSignaled = (log2Width >= LOG2_SECONDARY_PREFIX_START_SIZE && groupIdxX == maxLastPosX);
    bool       isMaximum        = false;
    if (maxValueSignaled)
    {
      int ctxId =
        (compID == CompID::COMP_Y ? log2Width - LOG2_SECONDARY_PREFIX_START_SIZE : 5 + to_underlying(compID) - 1);
      isMaximum = (posX == (g_minInGroup[groupIdxX + 1] - 1));
      m_binEncoder.encodeBin((unsigned)isMaximum, Ctx::lastXSecondaryPrefix(ctxId));
      DTRACE(g_trace_ctx, D_SYNTAX, "last_sig_coeff() isMaxX=%d\n", int(isMaximum));
    }
    if (!isMaximum)
    {
      posX -= g_minInGroup[groupIdxX];
      const int ctxId  = std::max<int>(0, log2Width - LOG2_ID_SUFFIX_CTX_START_SIZE);
      const int count  = (((int)groupIdxX - 2) >> 1) - 1;
      const int maxRem = ((1 << count) - 1) << 1;
      for (int i = count; i >= 1; i--)
      {
        if (i == count)
        {
          m_binEncoder.encodeBin((posX >> i) & 1, Ctx::lastXSuffix[compID](ctxId));
        }
        else
        {
          m_binEncoder.encodeBinEP((posX >> i) & 1);
        }
        DTRACE(g_trace_ctx, D_SYNTAX, "last_sig_coeff() lastXSuffixBin(%d)=%d\n", i, (posX >> i) & 1);
      }
      if (!maxValueSignaled || posX != maxRem)
      {
        m_binEncoder.encodeBinEP(posX & 1);
        DTRACE(g_trace_ctx, D_SYNTAX, "last_sig_coeff() lastXSuffixBin(%d)=%d\n", 0, posX & 1);
      }
    }
  }
  if (groupIdxY > 3)
  {
    const int  log2Height       = floorLog2(cctx.height());
    const bool maxValueSignaled = (log2Height >= LOG2_SECONDARY_PREFIX_START_SIZE && groupIdxY == maxLastPosY);
    bool       isMaximum        = false;
    if (maxValueSignaled)
    {
      int ctxId =
        (compID == CompID::COMP_Y ? log2Height - LOG2_SECONDARY_PREFIX_START_SIZE : 5 + to_underlying(compID) - 1);
      isMaximum = (posY == (g_minInGroup[groupIdxY + 1] - 1));
      m_binEncoder.encodeBin((unsigned)isMaximum, Ctx::lastYSecondaryPrefix(ctxId));
      DTRACE(g_trace_ctx, D_SYNTAX, "last_sig_coeff() isMaxY=%d\n", int(isMaximum));
    }
    if (!isMaximum)
    {
      posY -= g_minInGroup[groupIdxY];
      const int ctxId  = std::max<int>(0, log2Height - LOG2_ID_SUFFIX_CTX_START_SIZE);
      const int count  = (((int)groupIdxY - 2) >> 1) - 1;
      const int maxRem = ((1 << count) - 1) << 1;
      for (int i = count; i >= 1; i--)
      {
        if (i == count)
        {
          m_binEncoder.encodeBin((posY >> i) & 1, Ctx::lastYSuffix[compID](ctxId));
        }
        else
        {
          m_binEncoder.encodeBinEP((posY >> i) & 1);
        }
        DTRACE(g_trace_ctx, D_SYNTAX, "last_sig_coeff() lastYSuffixBin(%d)=%d\n", i, (posY >> i) & 1);
      }
      if (!maxValueSignaled || posY != maxRem)
      {
        m_binEncoder.encodeBinEP(posY & 1);
        DTRACE(g_trace_ctx, D_SYNTAX, "last_sig_coeff() lastYSuffixBin(%d)=%d\n", 0, posY & 1);
      }
    }
  }
}

void CABACWriter::residual_coding_subblock(CoeffCodingContext &cctx, const TCoeff *coeff,
                                           const uint64_t stateTransTable, int &state)
{
  //===== init =====
  const int  minSubPos     = cctx.minSubPos();
  const bool isLast        = cctx.isLast();
  int        firstSigPos   = (isLast ? cctx.scanPosLast() : cctx.maxSubPos());
  int        nextSigPos    = firstSigPos;
  bool       updateHistory = cctx.getUpdateHist();

  //===== encode significant_coeffgroup_flag =====
  if (!isLast && cctx.isNotFirst())
  {
    if (cctx.isSigGroup())
    {
      m_binEncoder.encodeBin(1, cctx.sigGroupCtxId());
    }
    else
    {
      m_binEncoder.encodeBin(0, cctx.sigGroupCtxId());
      return;
    }
  }

  uint8_t ctxOffset[16];

  //===== encode absolute values =====
  const int inferSigPos     = nextSigPos != cctx.scanPosLast() ? (cctx.isNotFirst() ? minSubPos : -1) : nextSigPos;
  int       firstNZPos      = nextSigPos;
  int       lastNZPos       = -1;
  int       numNonZero      = 0;
  unsigned  signPattern     = 0;
  int       remRegBins      = cctx.remRegBins;
  int       firstPos2ndPass = minSubPos - 1;

  for (; nextSigPos >= minSubPos && remRegBins >= MAX_REG_BINS; nextSigPos--)
  {
    const TCoeff level   = coeff[cctx.blockPos(nextSigPos)];
    unsigned     sigFlag = (level != 0);
    if (numNonZero || nextSigPos != inferSigPos)
    {
      const unsigned sigCtxId = cctx.sigCtxIdAbs(nextSigPos, coeff, state);
      m_binEncoder.encodeBin(sigFlag, sigCtxId);
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId);
      remRegBins--;
    }
    else if (nextSigPos != cctx.scanPosLast())
    {
      cctx.sigCtxIdAbs(nextSigPos, coeff,
                       state);   // required for setting variables that are needed for gtx/par context selection
    }

    if (sigFlag)
    {
      uint8_t &ctxOff = ctxOffset[nextSigPos - minSubPos];
      ctxOff          = cctx.ctxOffsetAbs();
      numNonZero++;
      firstNZPos = nextSigPos;
      lastNZPos  = std::max<int>(lastNZPos, nextSigPos);

      if (nextSigPos != cctx.scanPosLast())
      {
        signPattern <<= 1;
      }
      if (level < 0)
      {
        signPattern++;
      }

      TCoeff   remAbsLevel = abs(level);
      unsigned gtX         = 1;
      for (int k = 1; k <= GTN && gtX; k++)
      {
        unsigned ctxId = cctx.greaterXCtxIdAbs(k, ctxOff);
        gtX            = !!(--remAbsLevel);
        m_binEncoder.encodeBin(gtX, ctxId);
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "gt%d_flag() bin=%d ctx=%d\n", k, gtX, ctxId);
        remRegBins--;
      }
      if (gtX)
      {
        // start 2nd pass with first coeff that has all gtX == 1
        firstPos2ndPass = std::max(firstPos2ndPass, nextSigPos);
        unsigned parity = ((--remAbsLevel) & 1);
        m_binEncoder.encodeBinEP(parity);
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "par_flag() bin=%d \n", parity);
      }
    }

    state = int((stateTransTable >> ((state << 3) + ((level & 1) << 2))) & 15);
  }
  int minPos2ndPass = nextSigPos;
  cctx.remRegBins   = remRegBins;

  //===== 2nd PASS: Go-rice codes =====
  for (int scanPos = firstPos2ndPass; scanPos > minPos2ndPass; scanPos--)
  {
    unsigned absLevel = (unsigned)abs(coeff[cctx.blockPos(scanPos)]);
    if (absLevel >= GTN_LEVEL)
    {
      const unsigned ricePar = cctx.deriveRiceGTN(scanPos, coeff);
      const unsigned rem     = (absLevel - GTN_LEVEL) >> 1;
      m_binEncoder.encodeRemAbsEP(rem, ricePar, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange());
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, ricePar);
    }
  }

  //===== coeff bypass ====
  for (int scanPos = minPos2ndPass; scanPos >= minSubPos; scanPos--)
  {
    TCoeff   coeffVal = coeff[cctx.blockPos(scanPos)];
    unsigned absLevel = (unsigned)abs(coeffVal);
    int      rice     = (cctx.*(cctx.deriveRiceRRC))(scanPos, coeff, 0);
    int      pos0     = g_goRicePosCoeff0(state, rice);
    unsigned rem      = (absLevel == 0 ? pos0 : absLevel <= pos0 ? absLevel - 1 : absLevel);
    m_binEncoder.encodeRemAbsEP(rem, rice, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange());
    DTRACE(g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, rice);
    state = int((stateTransTable >> ((state << 3) + ((absLevel & 1) << 2))) & 15);
    if ((updateHistory) && (rem > 0))
    {
      unsigned &riceStats = m_binEncoder.getCtx().getGRAdaptStats((unsigned)cctx.compID());
      cctx.updateRiceStat(riceStats, rem, 0);
      cctx.setUpdateHist(0);
      updateHistory = 0;
    }
    if (absLevel)
    {
      numNonZero++;
      firstNZPos = scanPos;
      lastNZPos  = std::max<int>(lastNZPos, scanPos);
      signPattern <<= 1;
      if (coeffVal < 0)
      {
        signPattern++;
      }
    }
  }

  //===== encode sign's =====
  if (cctx.isSignPredCG())
  {
    return;
  }
  unsigned numSigns = numNonZero;
  if (cctx.hideSign(firstNZPos, lastNZPos))
  {
    numSigns--;
    signPattern >>= 1;
  }
  m_binEncoder.encodeBinsEP(signPattern, numSigns);
}

void CABACWriter::residual_codingTS(const TransformUnit &tu, CompID compID)
{
  DTRACE(g_trace_ctx, D_SYNTAX, "residual_codingTS() etype=%d pos=(%d,%d) size=%dx%d\n", tu.blocks[compID].compID,
         tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height);

  // init coeff coding context
  CoeffCodingContext cctx(tu, compID, false, tu.cu->bdpcmMode[isLuma(compID) ? 0 : 1]);
  const TCoeff      *coeff      = tu.getCoeffs(compID).buf;
  int                maxCtxBins = (cctx.maxNumCoeff() * 7) >> 2;
  cctx.remRegBins               = maxCtxBins;

  // determine and set last coeff position and sig group flags
  std::bitset<MLS_GRP_NUM> sigGroupFlags;
  for (int scanPos = 0; scanPos < cctx.maxNumCoeff(); scanPos++)
  {
    unsigned blkPos = cctx.blockPos(scanPos);
    if (coeff[blkPos])
    {
      sigGroupFlags.set(scanPos >> cctx.log2CGSize());
    }
  }

  // code subblocks
  for (int subSetId = 0; subSetId <= (cctx.maxNumCoeff() - 1) >> cctx.log2CGSize(); subSetId++)
  {
    cctx.initSubblock(subSetId, sigGroupFlags[subSetId]);
    int      goRiceParam     = 1;
    bool     ricePresentFlag = false;
    unsigned riceBit[8]      = { 0, 0, 0, 0, 0, 0, 0, 0 };
    if (tu.cu->slice->m_sps->m_spsRangeExtension.m_tsrcRicePresentFlag && tu.mtsIdx[compID] == MtsType::SKIP)
    {
      goRiceParam = goRiceParam + tu.cu->slice->m_tsrcIndex;
      if (isEncoding())
      {
        ricePresentFlag = true;
        for (int i = 0; i < MAX_TSRC_RICE; i++)
        {
          riceBit[i] = tu.cu->slice->m_riceBit[i];
        }
      }
    }
    residual_coding_subblockTS(cctx, coeff, riceBit, goRiceParam, ricePresentFlag);
    if (tu.cu->slice->m_sps->m_spsRangeExtension.m_tsrcRicePresentFlag && tu.mtsIdx[compID] == MtsType::SKIP &&
        isEncoding())
    {
      for (int i = 0; i < MAX_TSRC_RICE; i++)
      {
        tu.cu->slice->m_riceBit[i] = riceBit[i];
      }
    }
  }
}

void CABACWriter::residual_coding_subblockTS(CoeffCodingContext &cctx, const TCoeff *coeff, unsigned (&RiceBit)[8],
                                             const int riceParam, bool ricePresentFlag)
{
  //===== init =====
  const int minSubPos   = cctx.maxSubPos();
  int       firstSigPos = cctx.minSubPos();
  int       nextSigPos  = firstSigPos;

  //===== encode significant_coeffgroup_flag =====
  if (!cctx.isLastSubSet() || !cctx.only1stSigGroup())
  {
    if (cctx.isSigGroup())
    {
      m_binEncoder.encodeBin(1, cctx.sigGroupCtxId(true));
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_sigGroup() bin=%d ctx=%d\n", 1, cctx.sigGroupCtxId());
    }
    else
    {
      m_binEncoder.encodeBin(0, cctx.sigGroupCtxId(true));
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_sigGroup() bin=%d ctx=%d\n", 0, cctx.sigGroupCtxId());
      return;
    }
  }

  //===== encode absolute values =====
  const int inferSigPos = minSubPos;
  int       remAbsLevel = -1;
  int       numNonZero  = 0;

  int rightPixel, belowPixel, modAbsCoeff;

  int lastScanPosPass1 = -1;
  int lastScanPosPass2 = -1;
  for (; nextSigPos <= minSubPos && cctx.remRegBins >= 4; nextSigPos++)
  {
    TCoeff   coeffVal = coeff[cctx.blockPos(nextSigPos)];
    unsigned sigFlag  = (coeffVal != 0);
    if (numNonZero || nextSigPos != inferSigPos)
    {
      const unsigned sigCtxId = cctx.sigCtxIdAbsTS(nextSigPos, coeff);
      m_binEncoder.encodeBin(sigFlag, sigCtxId);
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId);
      cctx.remRegBins--;
    }

    if (sigFlag)
    {
      //===== encode sign's =====
      int            sign      = coeffVal < 0;
      const unsigned signCtxId = cctx.signCtxIdAbsTS(nextSigPos, coeff, cctx.bdpcm());
      m_binEncoder.encodeBin(sign, signCtxId);
      cctx.remRegBins--;
      numNonZero++;
      cctx.neighTS(rightPixel, belowPixel, nextSigPos, coeff);
      modAbsCoeff = cctx.deriveModCoeff(rightPixel, belowPixel, abs(coeffVal), cctx.bdpcm() != BdpcmMode::NONE);
      remAbsLevel = modAbsCoeff - 1;

      unsigned       gt1      = !!remAbsLevel;
      const unsigned gt1CtxId = cctx.lrg1CtxIdAbsTS(nextSigPos, coeff, cctx.bdpcm());
      m_binEncoder.encodeBin(gt1, gt1CtxId);
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_gt1_flag() bin=%d ctx=%d\n", gt1, gt1CtxId);
      cctx.remRegBins--;

      if (gt1)
      {
        remAbsLevel -= 1;
        m_binEncoder.encodeBin(remAbsLevel & 1, cctx.parityCtxIdAbsTS());
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_par_flag() bin=%d ctx=%d\n", remAbsLevel & 1, cctx.parityCtxIdAbsTS());
        cctx.remRegBins--;
      }
    }
    lastScanPosPass1 = nextSigPos;
  }

  int cutoffVal = 2;
  int numGtBins = 4;
  for (int scanPos = firstSigPos; scanPos <= minSubPos && cctx.remRegBins >= 4; scanPos++)
  {
    unsigned absLevel;
    cctx.neighTS(rightPixel, belowPixel, scanPos, coeff);
    absLevel =
      cctx.deriveModCoeff(rightPixel, belowPixel, abs(coeff[cctx.blockPos(scanPos)]), cctx.bdpcm() != BdpcmMode::NONE);
    cutoffVal = 2;
    for (int i = 0; i < numGtBins; i++)
    {
      if (absLevel >= cutoffVal)
      {
        unsigned gt2 = (absLevel >= (cutoffVal + 2));
        m_binEncoder.encodeBin(gt2, cctx.greaterXCtxIdAbsTS(cutoffVal >> 1));
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_gt%d_flag() bin=%d ctx=%d sp=%d coeff=%d\n", i + 2, gt2,
               cctx.greaterXCtxIdAbsTS(cutoffVal >> 1), scanPos,
               std::min<int>(absLevel, cutoffVal) + 2 * gt2 + (absLevel & 1));
        cctx.remRegBins--;
      }
      cutoffVal += 2;
    }
    lastScanPosPass2 = scanPos;
  }

  //===== coeff bypass ====
  for (int scanPos = firstSigPos; scanPos <= minSubPos; scanPos++)
  {
    unsigned absLevel;
    cctx.neighTS(rightPixel, belowPixel, scanPos, coeff);
    cutoffVal = (scanPos <= lastScanPosPass2 ? 10 : (scanPos <= lastScanPosPass1 ? 2 : 0));
    absLevel  = cctx.deriveModCoeff(rightPixel, belowPixel, abs(coeff[cctx.blockPos(scanPos)]),
                                    cctx.bdpcm() != BdpcmMode::NONE || cutoffVal == 0);

    if (absLevel >= cutoffVal)
    {
      unsigned rem = scanPos <= lastScanPosPass1 ? (absLevel - cutoffVal) >> 1 : absLevel;
      m_binEncoder.encodeRemAbsEP(rem, riceParam, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange());
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_rem_val() bin=%d ctx=%d sp=%d\n", rem, riceParam, scanPos);
      if (ricePresentFlag && (isEncoding()) && (cctx.compID() == COMP_Y))
      {
        for (int idx = 1; idx < 9; idx++)
        {
          uint32_t length;
          uint32_t symbol = rem;
          if (rem < (5 << idx))
          {
            length = rem >> idx;
            RiceBit[idx - 1] += (length + 1 + idx);
          }
          else
          {
            length = idx;
            symbol = symbol - (5 << idx);
            while (symbol >= (1 << length))
            {
              symbol -= (1 << (length++));
            }
            RiceBit[idx - 1] += (5 + length + 1 - idx + length);
          }
        }
      }

      if (absLevel && scanPos > lastScanPosPass1)
      {
        const int sign = coeff[cctx.blockPos(scanPos)] < 0 ? 1 : 0;
        m_binEncoder.encodeBinEP(sign);
      }
    }
  }
}

//================================================================================
//  helper functions
//--------------------------------------------------------------------------------
//    void  unary_max_symbol  ( symbol, ctxId0, ctxIdN, maxSymbol )
//    void  unary_max_eqprob  ( symbol,                 maxSymbol )
//    void  exp_golomb_eqprob ( symbol, count )
//================================================================================

void CABACWriter::unary_max_symbol(unsigned symbol, unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol)
{
  CHECK(symbol > maxSymbol, "symbol > maxSymbol");
  const unsigned totalBinsToWrite = std::min(symbol + 1, maxSymbol);
  for (unsigned binsWritten = 0; binsWritten < totalBinsToWrite; ++binsWritten)
  {
    const unsigned nextBin = symbol > binsWritten;
    m_binEncoder.encodeBin(nextBin, binsWritten == 0 ? ctxId0 : ctxIdN);
  }
}

void CABACWriter::unary_max_eqprob(unsigned symbol, unsigned maxSymbol)
{
  if (maxSymbol == 0)
  {
    return;
  }
  bool     codeLast = (maxSymbol > symbol);
  unsigned bins     = 0;
  unsigned numBins  = 0;
  while (symbol--)
  {
    bins <<= 1;
    bins++;
    numBins++;
  }
  if (codeLast)
  {
    bins <<= 1;
    numBins++;
  }
  CHECK(!(numBins <= 32), "Unspecified error");
  m_binEncoder.encodeBinsEP(bins, numBins);
}

void CABACWriter::exp_golomb_eqprob(unsigned symbol, unsigned count)
{
  unsigned bins    = 0;
  unsigned numBins = 0;
  while (symbol >= (unsigned)(1 << count))
  {
    bins <<= 1;
    bins++;
    numBins++;
    symbol -= 1 << count;
    count++;
  }
  bins <<= 1;
  numBins++;
  // CHECK(!( numBins + count <= 32 ), "Unspecified error");
  m_binEncoder.encodeBinsEP(bins, numBins);
  m_binEncoder.encodeBinsEP(symbol, count);
}

void CABACWriter::codeAlfCtuEnableFlags(CodingStructure &cs, ChannelType channel,
                                        const AlfParameters::AlfParamBase *alfParam)
{
  if (isLuma(channel))
  {
    if (alfParam->enabledFlag[COMP_Y])
    {
      codeAlfCtuEnableFlags(cs, COMP_Y, alfParam);
    }
  }
  else
  {
    if (alfParam->enabledFlag[COMP_Cb])
    {
      codeAlfCtuEnableFlags(cs, COMP_Cb, alfParam);
    }
    if (alfParam->enabledFlag[COMP_Cr])
    {
      codeAlfCtuEnableFlags(cs, COMP_Cr, alfParam);
    }
  }
}
void CABACWriter::codeAlfCtuEnableFlags(CodingStructure &cs, CompID compID, const AlfParameters::AlfParamBase *alfParam)
{
  uint32_t numCTUs = cs.pcv->sizeInCtus;

  for (int ctuIdx = 0; ctuIdx < numCTUs; ctuIdx++)
  {
    codeAlfCtuEnableFlag(cs, ctuIdx, compID, alfParam);
  }
}

void CABACWriter::codeAlfCtuEnableFlag(CodingStructure &cs, uint32_t ctuRsAddr, const int compIdx,
                                       const AlfParameters::AlfParamBase *alfParam)
{
  const bool alfComponentEnabled =
    (alfParam != nullptr) ? alfParam->enabledFlag[compIdx] : cs.slice->m_alfEnabledFlag[(CompID)compIdx];

  if (cs.sps->m_alfEnabledFlag && alfComponentEnabled)
  {
    const PreCalcValues &pcv = *cs.pcv;

    const int frameWidthInCtus = pcv.widthInCtus;

    const int ry = ctuRsAddr / frameWidthInCtus;
    const int rx = ctuRsAddr - ry * frameWidthInCtus;

    const Position pos(rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight);

    const uint32_t curSliceIdx = cs.slice->m_independentSliceIdx;
    const uint32_t curTileIdx  = cs.pps->getTileIdx(pos);

    const bool leftAvail  = cs.getCURestricted(pos.offset(-(int)pcv.maxCUWidth, 0), pos, curSliceIdx, curTileIdx,
                                               ChannelType::LUMA) != nullptr;
    const bool aboveAvail = cs.getCURestricted(pos.offset(0, -(int)pcv.maxCUHeight), pos, curSliceIdx, curTileIdx,
                                               ChannelType::LUMA) != nullptr;

    const int leftCTUAddr  = leftAvail ? ctuRsAddr - 1 : -1;
    const int aboveCTUAddr = aboveAvail ? ctuRsAddr - frameWidthInCtus : -1;

    const AlfParameters::CtbModes &alfModes = cs.slice->m_pic->getAlfModes(compIdx);
    const int ctx = (leftCTUAddr > -1 ? (AlfParameters::CtbModeHandler::isEnabled(alfModes[leftCTUAddr]) ? 1 : 0) : 0) +
      (aboveCTUAddr > -1 ? (AlfParameters::CtbModeHandler::isEnabled(alfModes[aboveCTUAddr]) ? 1 : 0) : 0);
    m_binEncoder.encodeBin(AlfParameters::CtbModeHandler::isEnabled(alfModes[ctuRsAddr]),
                           Ctx::alfCtbFlag(compIdx * 3 + ctx));
  }
}

void CABACWriter::codeCcAlfFilterControlIdc(uint8_t idcVal, CodingStructure &cs, const CompID compID, const int curIdx,
                                            const uint8_t *filterControlIdc, Position lumaPos, const int filterCount)
{
  CHECK(idcVal > filterCount, "Filter index is too large");

  const uint32_t curSliceIdx  = cs.slice->m_independentSliceIdx;
  const uint32_t curTileIdx   = cs.pps->getTileIdx(lumaPos);
  Position       leftLumaPos  = lumaPos.offset(-(int)cs.pcv->maxCUWidth, 0);
  Position       aboveLumaPos = lumaPos.offset(0, -(int)cs.pcv->maxCUWidth);
  bool leftAvail = cs.getCURestricted(leftLumaPos, lumaPos, curSliceIdx, curTileIdx, ChannelType::LUMA) ? true : false;
  bool aboveAvail =
    cs.getCURestricted(aboveLumaPos, lumaPos, curSliceIdx, curTileIdx, ChannelType::LUMA) ? true : false;

  int ctxt = 0;
  if (cs.sps->m_alfImprovementsEnabledFlag)
  {
    if (leftAvail)
    {
      ctxt += (filterControlIdc[curIdx - 1] != 1) ? 1 : 0;
    }
    if (aboveAvail)
    {
      ctxt += (filterControlIdc[curIdx - cs.pcv->widthInCtus] != 1) ? 1 : 0;
    }
  }
  else
  {
    if (leftAvail)
    {
      ctxt += (filterControlIdc[curIdx - 1]) ? 1 : 0;
    }
    if (aboveAvail)
    {
      ctxt += (filterControlIdc[curIdx - cs.pcv->widthInCtus]) ? 1 : 0;
    }
  }
  ctxt += (compID == COMP_Cr) ? 3 : 0;

  const auto encode = [this, ctxt, filterCount](const uint8_t idcVal)
  {
    m_binEncoder.encodeBin((idcVal == 0) ? 0 : 1, Ctx::CcAlfFilterControlFlag(ctxt));   // ON/OFF flag is context coded

    if (idcVal > 0)
    {
      int val = (idcVal - 1);
      while (val)
      {
        m_binEncoder.encodeBinEP(1);
        val--;
      }
      if (idcVal < filterCount)
      {
        m_binEncoder.encodeBinEP(0);
      }
    }
  };

  if (cs.sps->m_alfImprovementsEnabledFlag)
  {
    int      pos0      = 1;
    unsigned mappedIdc = (idcVal == 0 ? pos0 : idcVal <= pos0 ? idcVal - 1 : idcVal);
    encode(mappedIdc);
  }
  else
  {
    encode(idcVal);
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "ccAlfFilterControlIdc() compID=%d pos=(%d,%d) ctxt=%d, filterCount=%d, idcVal=%d\n",
         compID, lumaPos.x, lumaPos.y, ctxt, filterCount, idcVal);
}

#if ENABLE_NNLF
void CABACWriter::writeNnlfUnifiedParameters(const CodingStructure &cs)
{
  // signal parameter id of each block
  NnlfFilterParameters &prm = cs.picture->m_picprm;

  int cpt    = 0;
  int prmNum = prm.prmNum;

  if (prm.sprm.mode < prmNum)   // no block level signalling
  {
    return;
  }

  for (int y = 0; y < prm.nb_blocks_height; ++y)
  {
    for (int x = 0; x < prm.nb_blocks_width; ++x, ++cpt)
    {
      int code = prm.prmId[cpt];

      m_binEncoder.encodeBin(code != -1, Ctx::nnlfUnifiedParams(0));

      if (prmNum > 1 && code != -1)
      {
        m_binEncoder.encodeBin(code == 0, Ctx::nnlfUnifiedParams(1));
      }
      if (prmNum > 2 && code > 0)
      {
        xWriteTruncBinCode(code - 1, prmNum - 1);
      }
    }
  }
}
#endif

void CABACWriter::mip_flag(const CodingUnit &cu)
{
  if (!cu.Y().valid())
  {
    return;
  }
  if (!cu.cs->sps->m_useMIP)
  {
    return;
  }

  unsigned ctxId = DeriveCtx::CtxMipFlag(cu);
  m_binEncoder.encodeBin(cu.mipFlag, Ctx::MipFlag(ctxId));
  DTRACE(g_trace_ctx, D_SYNTAX, "mip_flag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.mipFlag ? 1 : 0);
}

void CABACWriter::mip_pred_mode(const CodingUnit &cu)
{
  bvg_mip_flag(cu);
  m_binEncoder.encodeBinEP((cu.mipTransposedFlag ? 1 : 0));

  const int numModes = MatrixIntraPrediction::getNumModesMip(cu.Y());
  CHECKD(cu.intraDir[ChannelType::LUMA] < 0 || cu.intraDir[ChannelType::LUMA] >= numModes, "Invalid MIP mode");
  xWriteTruncBinCode(cu.intraDir[ChannelType::LUMA], numModes);

  DTRACE(g_trace_ctx, D_SYNTAX, "mip_pred_mode() pos=(%d,%d) mode=%d transposed=%d\n", cu.lumaPos().x, cu.lumaPos().y,
         cu.intraDir[ChannelType::LUMA], cu.mipTransposedFlag ? 1 : 0);
}

void CABACWriter::bvg_mip_flag(const CodingUnit &cu)
{
  const bool available = PU::bvgMipAvailable(cu);
  CHECK(cu.bvgMipFlag && !available, "BVG-MIP is selected without a valid block-vector reference.");
  if (!available)
  {
    return;
  }

  m_binEncoder.encodeBinEP(cu.bvgMipFlag ? 1 : 0);
  DTRACE(g_trace_ctx, D_SYNTAX, "bvg_mip_flag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y,
         cu.bvgMipFlag ? 1 : 0);
}

void CABACWriter::planarDir(const CodingUnit &cu)
{
  if (!PU::directionalPlanarAvailable(cu) || !cu.Y().valid())
  {
    return;
  }
  m_binEncoder.encodeBin((cu.plDir == PlanarDirType::NO_DIR) ? 0 : 1, Ctx::PlanarDir(0));
  if (cu.plDir != PlanarDirType::NO_DIR)
  {
    m_binEncoder.encodeBin((cu.plDir == PlanarDirType::HOR) ? 0 : 1, Ctx::PlanarDir(1));
  }
}

void CABACWriter::dimd_flag(const CodingUnit &cu)
{
  if (!cu.Y().valid() || !cu.cs->sps->m_useDIMD)
  {
    return;
  }
  unsigned ctxId = DeriveCtx::CtxDimdFlag(cu);
  m_binEncoder.encodeBin(cu.dimdFlag, Ctx::DimdFlag(ctxId));
  obic_flag(cu);
}

void CABACWriter::dimdChromaFlag(const CodingUnit &cu)
{
  bool chromaChannelValid = cu.Cb().valid() && cu.Cr().valid();
  if (chromaChannelValid && cu.slice->m_sps->m_useDIMDChroma)
  {
    const unsigned int invertedFlag = cu.dimdChromaFlag ? 0 : 1;
    m_binEncoder.encodeBin(invertedFlag, Ctx::DimdChromaFlag());
  }
}

void CABACWriter::timd_flag(const CodingUnit &cu)
{
  if (!cu.Y().valid() || !cu.cs->sps->m_useTIMD)
  {
    return;
  }
  unsigned ctxId = DeriveCtx::CtxTimdFlag(cu);
  m_binEncoder.encodeBin(cu.timdFlag, Ctx::TimdFlag(ctxId));
  DTRACE(g_trace_ctx, D_SYNTAX, "timd_flag() pos=(%d,%d) size=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y,
         cu.lumaSize().width, cu.lumaSize().height, cu.timdFlag);
}

void CABACWriter::timd_sad_flag(const CodingUnit &cu)
{
  if (!cu.Y().valid() || !cu.cs->sps->m_useTIMD || !cu.cs->sps->m_useTIMDSAD || !cu.timdFlag || !CU::allowTimdSad(cu))
  {
    return;
  }

  m_binEncoder.encodeBin(cu.timdSadFlag, Ctx::TimdSadFlag());
}

void CABACWriter::obic_flag(const CodingUnit &cu)
{
  if (!cu.Y().valid() || !cu.cs->sps->m_useOBIC || !cu.dimdFlag)
  {
    return;
  }
  if (!cu.obicAvailFlag)
  {
    return;
  }
  m_binEncoder.encodeBin(cu.obicFlag, Ctx::ObicFlag(0));
  DTRACE(g_trace_ctx, D_SYNTAX, "cu_obic_flag() pos=(%d,%d) obic=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.obicFlag);
}
void CABACWriter::eip_flag(const CodingUnit &cu)
{
  if (!cu.Y().valid() || !isLuma(cu.chType) || !cu.cs->sps->m_useEIP)
  {
    return;
  }

  const bool bCanUseEip = CU::eipAllowed(cu, CompID::COMP_Y) || CU::eipMergeAllowed(cu, CompID::COMP_Y);
  if (bCanUseEip)
  {
    m_binEncoder.encodeBin(cu.eipFlag, Ctx::EipFlag(0));
    if (cu.eipFlag)
    {
      if (CU::eipAllowed(cu, CompID::COMP_Y) && CU::eipMergeAllowed(cu, CompID::COMP_Y))
      {
        m_binEncoder.encodeBin(cu.eipMerge, Ctx::EipFlag(1));
      }
      if (cu.eipMerge)
      {
        unary_max_eqprob(cu.intraDir[ChannelType::LUMA], NUM_EIP_MERGE_SIGNAL - 1);
      }
      else
      {
        static_vector<EipInfo, NUM_DERIVED_EIP> eipInfoList;
        if (cu.cs->sps->m_useMMEIP)
        {
          m_binEncoder.encodeBin(cu.eipMultiModel, Ctx::EipFlag(2));
        }
        CU::eipCurAllowed(cu, CompID::COMP_Y, eipInfoList, cu.eipMultiModel);
        xWriteTruncBinCode(cu.intraDir[ChannelType::LUMA], uint32_t(eipInfoList.size()));
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "eip_flag() pos=(%d,%d) mode=%d idx= %d merge=%d mm=%d\n", cu.lumaPos().x,
         cu.lumaPos().y, cu.eipFlag ? 1 : 0, cu.intraDir[ChannelType::LUMA], cu.eipMerge ? 1 : 0,
         cu.eipMultiModel ? 1 : 0);
}

void CABACWriter::codeAlfCtuFilterIndex(CodingStructure &cs, uint32_t ctuRsAddr, bool alfEnableLuma)
{
  if (!cs.sps->m_alfEnabledFlag || !alfEnableLuma)
  {
    return;
  }

  const AlfParameters::CtbModes &alfModes = cs.slice->m_pic->getAlfModes(COMP_Y);

  const int m = alfModes[ctuRsAddr];
  if (!AlfParameters::CtbModeHandler::isEnabled(m))
  {
    return;
  }

  const int  numAps        = cs.slice->m_numAlfApsIdsLuma;
  const bool alfUseApsFlag = cs.sps->m_alfImprovementsEnabledFlag
    ? AlfParametersEcm::LumaCtbModeHandler::isApsFilter(m)
    : AlfParametersVtm::LumaCtbModeHandler::isApsFilter(m);

  if (numAps > 0)
  {
    m_binEncoder.encodeBin(alfUseApsFlag ? 1 : 0, Ctx::alfUseApsFlag());
  }
  if (alfUseApsFlag)
  {
    const uint32_t alfLumaPrevFilterIdx = cs.sps->m_alfImprovementsEnabledFlag
      ? AlfParametersEcm::LumaCtbModeHandler::getApsIdx(m)
      : AlfParametersVtm::LumaCtbModeHandler::getApsIdx(m);
    CHECK(alfLumaPrevFilterIdx >= numAps, "alfLumaPrevFilterIdx is too large");

    if (numAps > 1)
    {
      xWriteTruncBinCode(alfLumaPrevFilterIdx, numAps);
    }
  }
  else
  {
    if (cs.sps->m_alfImprovementsEnabledFlag)
    {
      const uint32_t alfLumaFixedFilterIdx = AlfParametersEcm::LumaCtbModeHandler::getFixedFilterIdx(m);
      xWriteTruncBinCode(alfLumaFixedFilterIdx, AlfParametersEcm::NUM_FIXED_FILTERS);
    }
    else
    {
      const uint32_t alfLumaFixedFilterIdx = AlfParametersVtm::LumaCtbModeHandler::getFixedFilterIdx(m);
      xWriteTruncBinCode(alfLumaFixedFilterIdx, AlfParametersVtm::ALF_NUM_FIXED_FILTER_SETS);
    }
  }
}

void CABACWriter::codeAlfCtuChromaAlternatives(const CodingStructure &cs, const AlfParameters::AlfParamBase &alfParam)
{
  if (alfParam.enabledFlag[COMP_Cb])
  {
    codeAlfCtuChromaAlternatives(cs, COMP_Cb, alfParam);
  }
  if (alfParam.enabledFlag[COMP_Cr])
  {
    codeAlfCtuChromaAlternatives(cs, COMP_Cr, alfParam);
  }
}

void CABACWriter::codeAlfCtuChromaAlternatives(const CodingStructure &cs, const CompID compID,
                                               const AlfParameters::AlfParamBase &alfParam)
{
  CHECK(!isChroma(compID), "A chroma component is needed.");

  uint32_t                       numCTUs  = cs.pcv->sizeInCtus;
  const AlfParameters::CtbModes &alfModes = cs.slice->m_pic->getAlfModes(compID);

  for (int ctuIdx = 0; ctuIdx < numCTUs; ctuIdx++)
  {
    if (AlfParameters::CtbModeHandler::isEnabled(alfModes[ctuIdx]))
    {
      codeAlfCtuChromaAlternative(cs, ctuIdx, compID, alfParam);
    }
  }
}

void CABACWriter::codeAlfCtuAlternative(const CodingStructure &cs, const int altIdx, const int numAlts,
                                        const CompID compId)
{
  CHECKD(altIdx >= numAlts, "Invalid ALF alternative index.");

  const auto ctxId = cs.sps->m_alfImprovementsEnabledFlag ? Ctx::ctbAlfAlternativeEcm(static_cast<int>(compId))
                                                          : Ctx::ctbAlfAlternativeVtm(static_cast<int>(compId) - 1);
  for (int i = 0; i < altIdx; ++i)
  {
    m_binEncoder.encodeBin(1, ctxId);
  }
  if (altIdx < numAlts - 1)
  {
    m_binEncoder.encodeBin(0, ctxId);
  }
}

/// @brief Code the ALF CTU chroma alternative getting the number of alternatives from the slice.
void CABACWriter::codeAlfCtuChromaAlternative(const CodingStructure &cs, const uint32_t ctuRsAddr, const CompID compId)
{
  CHECK(!isChroma(compId), "A chroma component is needed.");

  if (isAlfEnabledInSpsAndSlice(cs, compId))
  {
    const auto &ctbMode = getAlfCtbMode(cs, ctuRsAddr, compId);
    if (AlfParameters::CtbModeHandler::isEnabled(ctbMode))
    {
      const int  apsId   = cs.slice->m_alfApsIdChroma;
      const APS &aps     = *cs.slice->m_alfApss[apsId];
      const int  numAlts = aps.m_alfAPSParam.getParam().numAlternativesChroma;
      codeAlfCtuAlternative(cs, AlfParameters::ChromaCtbModeHandler::getAlternative(ctbMode), numAlts, compId);
    }
  }
}

/// @brief Code the ALF CTU chroma alternative getting the number of alternatives from the given parameters.
void CABACWriter::codeAlfCtuChromaAlternative(const CodingStructure &cs, const uint32_t ctuRsAddr, const CompID compId,
                                              const AlfParameters::AlfParamBase &alfParam)
{
  CHECK(!isChroma(compId), "A chroma component is needed.");

  const auto &ctbMode = getAlfCtbMode(cs, ctuRsAddr, compId);
  if (AlfParameters::CtbModeHandler::isEnabled(ctbMode))
  {
    codeAlfCtuAlternative(cs, AlfParameters::ChromaCtbModeHandler::getAlternative(ctbMode),
                          alfParam.numAlternativesChroma, compId);
  }
}

/// @brief Code the ALF CTU luma alternative getting the number of alternatives from the slice.
void CABACWriter::codeAlfCtuLumaAlternative(const CodingStructure &cs, const uint32_t ctuRsAddr)
{
  if (isAlfEnabledInSpsAndSlice(cs, CompID::COMP_Y))
  {
    const auto &ctbMode = getAlfCtbMode(cs, ctuRsAddr, CompID::COMP_Y);
    if (AlfParametersEcm::LumaCtbModeHandler::isApsFilter(ctbMode))
    {
      const int  apsId   = cs.slice->m_alfApsIdsLuma[AlfParametersEcm::LumaCtbModeHandler::getApsIdx(ctbMode)];
      const APS &aps     = *cs.slice->m_alfApss[apsId];
      const int  numAlts = aps.m_alfAPSParam.getEcmParam().numAlternativesLuma;
      codeAlfCtuAlternative(cs, AlfParametersEcm::LumaCtbModeHandler::getAlternative(ctbMode), numAlts, CompID::COMP_Y);
    }
  }
}

/// @brief Code the ALF CTU luma alternative using the given number of alternatives.
void CABACWriter::codeAlfCtuLumaAlternative(const CodingStructure &cs, const uint32_t ctuRsAddr, const int numAlts)
{
  const auto &ctbMode = getAlfCtbMode(cs, ctuRsAddr, CompID::COMP_Y);
  if (AlfParametersEcm::LumaCtbModeHandler::isApsFilter(ctbMode))
  {
    codeAlfCtuAlternative(cs, AlfParametersEcm::LumaCtbModeHandler::getAlternative(ctbMode), numAlts, CompID::COMP_Y);
  }
}

const int &CABACWriter::getAlfCtbMode(const CodingStructure &cs, const uint32_t ctuRsAddr, const CompID compId)
{
  return cs.slice->m_pic->getAlfModes(static_cast<int>(compId))[ctuRsAddr];
}

bool CABACWriter::isAlfEnabledInSpsAndSlice(const CodingStructure &cs, const CompID compId)
{
  return cs.sps->m_alfEnabledFlag && cs.slice->m_alfEnabledFlag[compId];
}

void CABACWriter::cu_lic_flag(const CodingUnit &cu)
{
  if (CU::isLICFlagPresent(cu))
  {
    m_binEncoder.encodeBin(cu.licFlag ? 1 : 0, Ctx::LICFlag(0));
    DTRACE(g_trace_ctx, D_SYNTAX, "cu_lic_flag() lic_flag=%d\n", cu.licFlag ? 1 : 0);
  }
}
//! \}
