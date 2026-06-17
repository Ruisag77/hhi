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

/** \file     CABACReader.cpp
 *  \brief    Reader for low level syntax
 */

#include "CABACReader.h"

#include "CommonLib/AlfParameters.h"
#include "CommonLib/AlfParametersEcm.h"
#include "CommonLib/AlfParametersVtm.h"
#include "CommonLib/CodingStructure.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/SampleAdaptiveOffset.h"
#include "CommonLib/dtrace_next.h"
#include "CommonLib/Picture.h"
#include "CommonLib/MatrixIntraPrediction.h"

#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif

#if RExt__DECODER_DEBUG_BIT_STATISTICS
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(x) \
  const CodingStatisticsClassType CSCT(x);               \
  m_binDecoder.set(CSCT)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET2(x, y) \
  const CodingStatisticsClassType CSCT(x, y);                \
  m_binDecoder.set(CSCT)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(x, s) \
  const CodingStatisticsClassType CSCT(x, s.width, s.height);    \
  m_binDecoder.set(CSCT)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(x, s, z) \
  const CodingStatisticsClassType CSCT(x, s.width, s.height, z);     \
  m_binDecoder.set(CSCT)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_SET(x) m_binDecoder.set(x);
#else
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(x)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET2(x, y)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(x, s)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(x, s, z)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_SET(x)
#endif

#if ENABLE_CABAC_DUMP
namespace CabacRetrain
{
extern SliceType sliceReport;
}
#endif

void CABACReader::initCtxModels(Slice &slice)
{
  SliceType sliceType = slice.m_eSliceType;
  int       qp        = slice.m_iSliceQp;
  if (slice.m_pps->m_cabacInitPresentFlag && slice.m_cabacInitFlag)
  {
    switch (sliceType)
    {
    case P_SLICE:           // change initialization table to B_SLICE initialization
      sliceType = B_SLICE;
      break;
    case B_SLICE:           // change initialization table to P_SLICE initialization
      sliceType = P_SLICE;
      break;
    default:           // should not occur
      THROW("Invalid slice type");
      break;
    }
  }

  if (sliceType == B_SLICE && slice.m_checkLdc)
  {
    sliceType = L_SLICE;
  }

  m_binDecoder.reset(qp, (int)sliceType);
  m_binDecoder.setBaseLevel(slice.m_riceBaseLevelValue);
  m_binDecoder.riceStatReset(slice.m_sps->m_bitDepths[ChannelType::LUMA],
                             slice.m_sps->m_spsRangeExtension.m_persistentRiceAdaptationEnabledFlag);
#if ENABLE_CABAC_DUMP
  slice.m_cabacInitSliceType = sliceType;
#endif
  if (slice.m_sps->m_tempCabacInitMode)
  {
    m_CABACDataStore->loadCtxStates(&slice, getCtx());
  }
#if ENABLE_CABAC_DUMP
  CabacRetrain::sliceReport = slice.m_cabacInitSliceType;
#endif
}

//================================================================================
//  clause 7.3.8.1
//--------------------------------------------------------------------------------
//    bool  terminating_bit()
//    void  remaining_bytes( noTrailingBytesExpected )
//================================================================================

bool CABACReader::terminating_bit()
{
  if (m_binDecoder.decodeBinTrm())
  {
    m_binDecoder.finish();
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    CodingStatistics::IncrementStatisticEP(STATS__TRAILING_BITS, m_bitstream->readOutTrailingBits(), 0);
#else
    m_bitstream->readOutTrailingBits();
#endif
    return true;
  }
  return false;
}

void CABACReader::remaining_bytes(bool noTrailingBytesExpected)
{
  if (noTrailingBytesExpected)
  {
    CHECK(0 != m_bitstream->getNumBitsLeft(), "Bits left when not supposed");
  }
  else
  {
    while (m_bitstream->getNumBitsLeft())
    {
      unsigned trailingNullByte = m_bitstream->readByte();
      if (trailingNullByte != 0)
      {
        THROW("Trailing byte should be '0', but has a value of " << std::hex << trailingNullByte << std::dec << "\n");
      }
    }
  }
}

//================================================================================
//  clause 7.3.8.2
//--------------------------------------------------------------------------------
//    void  coding_tree_unit( cs, area, qpL, qpC, ctuRsAddr )
//================================================================================

void CABACReader::coding_tree_unit(CodingStructure &cs, const UnitArea &area, EnumArray<int, ChannelType> &qps,
                                   unsigned ctuRsAddr)
{
  PROFILER_SCOPE(1, g_timeProfiler, P_PARSE_CTU);
  CUCtx           cuCtx(qps[ChannelType::LUMA]);
  QTBTPartitioner partitioner;

  partitioner.initCtu(area, ChannelType::LUMA, cs);

#if ENABLE_NNLF
  if (cs.sps->m_nnlf && !cs.picHeader->m_nnlfDisabled && ctuRsAddr == 0)
  {
    readNnlfUnifiedParameters(cs);
  }
#endif

  sao(cs, ctuRsAddr);
  if (cs.sps->m_ccSaoEnabledFlag)
  {
    for (int compIdx = 0; compIdx < getNumberValidComponents(cs.pcv->chrFormat); compIdx++)
    {
      if (cs.slice->m_ccSaoComParam.enabled[compIdx])
      {
        const int setNum = cs.slice->m_ccSaoComParam.setNum[compIdx];

        const int      ry = ctuRsAddr / cs.pcv->widthInCtus;
        const int      rx = ctuRsAddr % cs.pcv->widthInCtus;
        const Position lumaPos(rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight);

        ccSaoControlIdc(cs, CompID(compIdx), ctuRsAddr, cs.slice->m_ccSaoControl[compIdx], lumaPos, setNum);
      }
    }
  }
  lfCccm(cs, ctuRsAddr);

  if (cs.sps->m_alfEnabledFlag && (cs.slice->m_alfEnabledFlag[COMP_Y]))
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

    for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
    {
      if (cs.slice->m_alfEnabledFlag[(CompID)compIdx])
      {
        AlfParameters::CtbModes &alfModes = cs.slice->m_pic->getAlfModes(compIdx);
        const int                ctx =
          (leftCTUAddr > -1 ? (AlfParameters::CtbModeHandler::isEnabled(alfModes[leftCTUAddr]) ? 1 : 0) : 0) +
          (aboveCTUAddr > -1 ? (AlfParameters::CtbModeHandler::isEnabled(alfModes[aboveCTUAddr]) ? 1 : 0) : 0);

        RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__ALF);
        const bool enabled = m_binDecoder.decodeBin(Ctx::alfCtbFlag(compIdx * 3 + ctx)) != 0;

        if (!enabled)
        {
          AlfParameters::CtbModeHandler::setDisabled(alfModes[ctuRsAddr]);
        }
        else
        {
          if (isLuma((CompID)compIdx))
          {
            readAlfCtuFilterIndex(cs, ctuRsAddr);

            if (cs.sps->m_alfImprovementsEnabledFlag &&
                AlfParametersEcm::LumaCtbModeHandler::isApsFilter(alfModes[ctuRsAddr]))
            {
              AlfParametersEcm::LumaCtbModeHandler::setAlternative(
                alfModes[ctuRsAddr],
                readAlfCtuAlternative(
                  cs, cs.slice->m_alfApsIdsLuma[AlfParametersEcm::LumaCtbModeHandler::getApsIdx(alfModes[ctuRsAddr])],
                  CompID::COMP_Y));
            }
          }
          else
          {
            AlfParameters::ChromaCtbModeHandler::setAlternative(
              alfModes[ctuRsAddr], readAlfCtuAlternative(cs, cs.slice->m_alfApsIdChroma, static_cast<CompID>(compIdx)));
          }
        }
      }
    }
  }
  if (cs.sps->m_ccalfEnabledFlag)
  {
    const AlfParameters::CcAlfFilterParamBase sliceCcAlfFilterParam = cs.slice->m_ccAlfFilterParam.getParam();
    for (int compIdx = 1; compIdx < getNumberValidComponents(cs.pcv->chrFormat); compIdx++)
    {
      if (sliceCcAlfFilterParam.ccAlfFilterEnabled[compIdx - 1])
      {
        const int filterCount = sliceCcAlfFilterParam.ccAlfFilterCount[compIdx - 1];

        const int ry = ctuRsAddr / cs.pcv->widthInCtus;
        const int rx = ctuRsAddr % cs.pcv->widthInCtus;

        const Position lumaPos(rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight);

        ccAlfFilterControlIdc(cs, CompID(compIdx), ctuRsAddr, cs.slice->m_ccAlfFilterControl[compIdx - 1], lumaPos,
                              filterCount);
      }
    }
  }

  if (CS::isDualITree(cs) && isChromaEnabled(cs.pcv->chrFormat) &&
      cs.pcv->maxCUWidth > std::min<int>(MAX_TB_SIZEY, MAX_INTRA_SIZE))
  {
    QTBTPartitioner chromaPartitioner;
    chromaPartitioner.initCtu(area, ChannelType::CHROMA, cs);
    CUCtx cuCtxChroma(qps[ChannelType::CHROMA]);
    coding_tree(cs, partitioner, cuCtx, &chromaPartitioner, &cuCtxChroma);
    qps[ChannelType::LUMA]   = cuCtx.qp;
    qps[ChannelType::CHROMA] = cuCtxChroma.qp;
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

  DTRACE_COND(ctuRsAddr == 0, g_trace_ctx, D_QP_PER_CTU, "\n%4d %2d", cs.picture->m_poc, cs.slice->m_iSliceQpBase);
  DTRACE(g_trace_ctx, D_QP_PER_CTU, " %3d", qps[ChannelType::LUMA] - cs.slice->m_iSliceQpBase);
}

#if ENABLE_NNLF
void CABACReader::readNnlfUnifiedParameters(CodingStructure &cs)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__NNLF);
  // parse parameter id of each block
  NnlfFilterParameters      &prm    = cs.picture->m_picprm;
  const NnlfSliceParameters &sprm   = cs.picture->m_slices[0]->m_nnlfUnifiedParam;
  int                        cpt    = 0;
  int                        prmNum = prm.prmNum;
  for (int y = 0; y < prm.nb_blocks_height; ++y)
  {
    for (int x = 0; x < prm.nb_blocks_width; ++x, ++cpt)
    {
      if (sprm.mode < prmNum)
      {
        prm.prmId[cpt] = sprm.mode;
        continue;
      }
      bool useNnlf, useFirstParam = false;
      useNnlf = m_binDecoder.decodeBin(Ctx::nnlfUnifiedParams(0));
      if (prmNum == 1)
      {
        prm.prmId[cpt] = useNnlf ? 0 : -1;
      }
      else if (!useNnlf)
      {
        prm.prmId[cpt] = -1;
      }
      else
      {
        useFirstParam = m_binDecoder.decodeBin(Ctx::nnlfUnifiedParams(1));
        if (prmNum == 2)
        {
          prm.prmId[cpt] = useFirstParam ? 0 : 1;
        }
        else if (useFirstParam)
        {
          prm.prmId[cpt] = 0;
        }
        else
        {
          uint32_t nnlfPrmIdMinus1 = 0;
          xReadTruncBinCode(nnlfPrmIdMinus1, prmNum - 1);
          prm.prmId[cpt] = nnlfPrmIdMinus1 + 1;
        }
      }
    }
  }
}
#endif

void CABACReader::readAlfCtuFilterIndex(CodingStructure &cs, unsigned ctuRsAddr)
{
  auto      &alfMode       = cs.slice->m_pic->getAlfModes(COMP_Y)[ctuRsAddr];
  const int  numAps        = cs.slice->m_numAlfApsIdsLuma;
  const bool alfUseApsFlag = numAps > 0 && m_binDecoder.decodeBin(Ctx::alfUseApsFlag()) != 0;

  if (alfUseApsFlag)
  {
    uint32_t alfLumaPrevFilterIdx = 0;
    if (numAps > 1)
    {
      xReadTruncBinCode(alfLumaPrevFilterIdx, numAps);
    }
    if (cs.sps->m_alfImprovementsEnabledFlag)
    {
      // Set APS index and dummy alternative index. The real alternative index is read later.
      AlfParametersEcm::LumaCtbModeHandler::setApsFilter(alfMode, alfLumaPrevFilterIdx, 0);
    }
    else
    {
      AlfParametersVtm::LumaCtbModeHandler::setApsFilter(alfMode, alfLumaPrevFilterIdx);
    }
  }
  else
  {
    uint32_t alfLumaFixedFilterIdx = 0;
    if (cs.sps->m_alfImprovementsEnabledFlag)
    {
      xReadTruncBinCode(alfLumaFixedFilterIdx, AlfParametersEcm::NUM_FIXED_FILTERS);
      AlfParametersEcm::LumaCtbModeHandler::setFixedFilter(alfMode, alfLumaFixedFilterIdx);
    }
    else
    {
      xReadTruncBinCode(alfLumaFixedFilterIdx, AlfParametersVtm::ALF_NUM_FIXED_FILTER_SETS);
      AlfParametersVtm::LumaCtbModeHandler::setFixedFilter(alfMode, alfLumaFixedFilterIdx);
    }
  }
}

uint8_t CABACReader::readAlfCtuAlternative(const CodingStructure &cs, const int apsId, const CompID compId)
{
  CHECK(cs.slice->m_alfApss[apsId] == nullptr, "APS not initialized");
  CHECK(!cs.sps->m_alfImprovementsEnabledFlag && isLuma(compId), "VTM ALF does not have alternatives for luma.");

  const auto &sliceAlfAPSParam = cs.slice->m_alfApss[apsId]->m_alfAPSParam;
  const int   numAlts          = cs.sps->m_alfImprovementsEnabledFlag
               ? (isLuma(compId) ? sliceAlfAPSParam.getEcmParam().numAlternativesLuma
                                 : sliceAlfAPSParam.getEcmParam().numAlternativesChroma)
               : sliceAlfAPSParam.getVtmParam().numAlternativesChroma;
  const auto  ctxId = cs.sps->m_alfImprovementsEnabledFlag ? Ctx::ctbAlfAlternativeEcm(static_cast<int>(compId))
                                                           : Ctx::ctbAlfAlternativeVtm(static_cast<int>(compId) - 1);

  uint8_t decoded = 0;
  while (decoded < numAlts - 1 && m_binDecoder.decodeBin(ctxId))
  {
    ++decoded;
  }

  return decoded;
}

void CABACReader::ccAlfFilterControlIdc(CodingStructure &cs, const CompID compID, const int curIdx,
                                        uint8_t *filterControlIdc, Position lumaPos, int filterCount)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__CROSS_COMP_ALF_BLOCK_LEVEL_IDC);

  const Position leftLumaPos  = lumaPos.offset(-(int)cs.pcv->maxCUWidth, 0);
  const Position aboveLumaPos = lumaPos.offset(0, -(int)cs.pcv->maxCUWidth);

  const uint32_t curSliceIdx = cs.slice->m_independentSliceIdx;
  const uint32_t curTileIdx  = cs.pps->getTileIdx(lumaPos);

  const bool leftAvail =
    cs.getCURestricted(leftLumaPos, lumaPos, curSliceIdx, curTileIdx, ChannelType::LUMA) != nullptr;
  const bool aboveAvail =
    cs.getCURestricted(aboveLumaPos, lumaPos, curSliceIdx, curTileIdx, ChannelType::LUMA) != nullptr;

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

  int idcVal = m_binDecoder.decodeBin(Ctx::CcAlfFilterControlFlag(ctxt));
  if (idcVal)
  {
    while ((idcVal != filterCount) && m_binDecoder.decodeBinEP())
    {
      idcVal++;
    }
  }

  if (cs.sps->m_alfImprovementsEnabledFlag)
  {
    const int pos0 = 1;
    idcVal         = (idcVal == pos0 ? 0 : idcVal < pos0 ? idcVal + 1 : idcVal);
  }

  filterControlIdc[curIdx] = idcVal;

  DTRACE(g_trace_ctx, D_SYNTAX, "ccAlfFilterControlIdc() compID=%d pos=(%d,%d) ctxt=%d, filterCount=%d, idcVal=%d\n",
         compID, lumaPos.x, lumaPos.y, ctxt, filterCount, idcVal);
}

void CABACReader::lfCccm(CodingStructure &cs, const uint32_t ctuRsAddr)
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
    cs.slice->m_lfCccmFrameLevelInherit = m_binDecoder.decodeBin(Ctx::LfCccmFlag(1));
  }
  if (cs.slice->m_lfCccmFrameLevelInherit)
  {
    return;
  }
  cs.slice->m_lfCccmEnabled.at(ctuRsAddr) = m_binDecoder.decodeBin(Ctx::LfCccmFlag(0));
  if (cs.slice->m_lfCccmEnabled.at(ctuRsAddr))
  {
    const int nCand = (int)cs.slice->lfCccmGetMergeCandidates(ctuRsAddr).size();
    if (nCand)
    {
      cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr) = m_binDecoder.decodeBin(Ctx::LfCccmFlag(2));
      if (cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr) && nCand > 1)
      {
        cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr) += 2 * m_binDecoder.decodeBin(Ctx::LfCccmFlag(3));
      }
      if (cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr))
      {
        cs.slice->lfCccmMerge(ctuRsAddr);
        return;
      }
    }
    cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr) = 0;

    cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr) += m_binDecoder.decodeBin(Ctx::LfCccmFlag(4));
    cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr) += 2 * m_binDecoder.decodeBin(Ctx::LfCccmFlag(5));
    cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr) += 4 * m_binDecoder.decodeBin(Ctx::LfCccmFlag(6));

    cs.slice->m_lfCccmModelType.at(ctuRsAddr) = m_binDecoder.decodeBin(Ctx::LfCccmFlag(7));
    cs.slice->m_lfCccmModelType.at(ctuRsAddr) += 2 * m_binDecoder.decodeBin(Ctx::LfCccmFlag(8));
    cs.slice->m_lfCccmModelType.at(ctuRsAddr) += 4 * m_binDecoder.decodeBin(Ctx::LfCccmFlag(9));
  }
}

//================================================================================
//  clause 7.3.8.3
//--------------------------------------------------------------------------------
//    void  sao( slice, ctuRsAddr )
//================================================================================

void CABACReader::sao(CodingStructure &cs, unsigned ctuRsAddr)
{
  const SPS &sps = *cs.sps;

  if (!(cs.pps->m_BIF || cs.sps->m_saoEnabledFlag || cs.pps->m_chromaBIF))
  {
    return;
  }
  // At least one is enabled, it is safe to assume we can do getSAO().
  SAOBlkParam &saoCtuParams = cs.picture->getSAO()[ctuRsAddr];

  if (!sps.m_saoEnabledFlag)
  {
    return;
  }

  const Slice &slice = *cs.slice;

  const bool sliceSaoLumaFlag   = slice.m_saoEnabledFlag[ChannelType::LUMA];
  const bool sliceSaoChromaFlag = slice.m_saoEnabledFlag[ChannelType::CHROMA] && isChromaEnabled(sps.m_chromaFormatIdc);

  saoCtuParams[COMP_Y].modeIdc  = SAOMode::OFF;
  saoCtuParams[COMP_Cb].modeIdc = SAOMode::OFF;
  saoCtuParams[COMP_Cr].modeIdc = SAOMode::OFF;

  if (!sliceSaoLumaFlag && !sliceSaoChromaFlag)
  {
    return;
  }

  // merge
  const int frameWidthInCtus = cs.pcv->widthInCtus;

  const int ry = ctuRsAddr / frameWidthInCtus;
  const int rx = ctuRsAddr - ry * frameWidthInCtus;

  const Position pos(rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight);
  const unsigned curSliceIdx = cs.slice->m_independentSliceIdx;

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__SAO);

  auto mergeType = SAOModeMergeTypes::NONE;

  const unsigned curTileIdx = cs.pps->getTileIdx(pos);

  if (cs.getCURestricted(pos.offset(-(int)cs.pcv->maxCUWidth, 0), pos, curSliceIdx, curTileIdx, ChannelType::LUMA))
  {
    // sao_merge_left_flag
    mergeType = m_binDecoder.decodeBin(Ctx::SaoMergeFlag()) ? SAOModeMergeTypes::LEFT : SAOModeMergeTypes::NONE;
  }

  if (mergeType == SAOModeMergeTypes::NONE &&
      cs.getCURestricted(pos.offset(0, -(int)cs.pcv->maxCUHeight), pos, curSliceIdx, curTileIdx, ChannelType::LUMA))
  {
    // sao_merge_above_flag
    mergeType = m_binDecoder.decodeBin(Ctx::SaoMergeFlag()) ? SAOModeMergeTypes::ABOVE : SAOModeMergeTypes::NONE;
  }

  if (mergeType != SAOModeMergeTypes::NONE)
  {
    if (sliceSaoLumaFlag || sliceSaoChromaFlag)
    {
      saoCtuParams[COMP_Y].modeIdc           = SAOMode::MERGE;
      saoCtuParams[COMP_Y].typeIdc.mergeType = mergeType;
    }
    if (sliceSaoChromaFlag)
    {
      saoCtuParams[COMP_Cb].modeIdc           = SAOMode::MERGE;
      saoCtuParams[COMP_Cr].modeIdc           = SAOMode::MERGE;
      saoCtuParams[COMP_Cb].typeIdc.mergeType = mergeType;
      saoCtuParams[COMP_Cr].typeIdc.mergeType = mergeType;
    }
    return;
  }

  // explicit parameters
  CompID firstComp = sliceSaoLumaFlag ? COMP_Y : COMP_Cb;
  CompID lastComp  = sliceSaoChromaFlag ? COMP_Cr : COMP_Y;
  for (CompID compID = firstComp; compID <= lastComp; compID = CompID(compID + 1))
  {
    SAOOffset &saoPars = saoCtuParams[compID];

    // sao_type_idx_luma / sao_type_idx_chroma
    if (compID != COMP_Cr)
    {
      if (m_binDecoder.decodeBin(Ctx::SaoTypeIdx()))
      {
        if (m_binDecoder.decodeBinEP())
        {
          // edge offset
          saoPars.modeIdc         = SAOMode::NEW;
          saoPars.typeIdc.newType = SAOModeNewTypes::START_EO;
        }
        else
        {
          // band offset
          saoPars.modeIdc         = SAOMode::NEW;
          saoPars.typeIdc.newType = SAOModeNewTypes::START_BO;
        }
      }
    }
    else // Cr, follow Cb SAO type
    {
      saoPars.modeIdc = saoCtuParams[COMP_Cb].modeIdc;
      saoPars.typeIdc = saoCtuParams[COMP_Cb].typeIdc;
    }
    if (saoPars.modeIdc == SAOMode::OFF)
    {
      continue;
    }

    // sao_offset_abs
    int       offset[4];
    const int maxOffsetQVal = SampleAdaptiveOffset::getMaxOffsetQVal(sps.m_bitDepths[toChannelType(compID)]);

    offset[0] = (int)unary_max_eqprob(maxOffsetQVal);
    offset[1] = (int)unary_max_eqprob(maxOffsetQVal);
    offset[2] = (int)unary_max_eqprob(maxOffsetQVal);
    offset[3] = (int)unary_max_eqprob(maxOffsetQVal);

    // band offset mode
    if (saoPars.typeIdc.newType == SAOModeNewTypes::START_BO)
    {
      // sao_offset_sign
      for (int k = 0; k < 4; k++)
      {
        if (offset[k] && m_binDecoder.decodeBinEP())
        {
          offset[k] = -offset[k];
        }
      }
      // sao_band_position
      saoPars.typeAuxInfo = m_binDecoder.decodeBinsEP(NUM_SAO_BO_CLASSES_LOG2);
      for (int k = 0; k < 4; k++)
      {
        saoPars.offset[(saoPars.typeAuxInfo + k) % MAX_NUM_SAO_CLASSES] = offset[k];
      }
      continue;
    }

    // edge offset mode
    saoPars.typeAuxInfo = 0;
    if (compID != COMP_Cr)
    {
      // sao_eo_class_luma / sao_eo_class_chroma
      saoPars.typeIdc.newType =
        SAOModeNewTypes(to_underlying(saoPars.typeIdc.newType) + m_binDecoder.decodeBinsEP(NUM_SAO_EO_TYPES_LOG2));
    }
    else
    {
      saoPars.typeIdc = saoCtuParams[COMP_Cb].typeIdc;
    }
    saoPars.offset[SAO_CLASS_EO_FULL_VALLEY] = offset[0];
    saoPars.offset[SAO_CLASS_EO_HALF_VALLEY] = offset[1];
    saoPars.offset[SAO_CLASS_EO_PLAIN]       = 0;
    saoPars.offset[SAO_CLASS_EO_HALF_PEAK]   = -offset[2];
    saoPars.offset[SAO_CLASS_EO_FULL_PEAK]   = -offset[3];
  }
}

void CABACReader::bif(const CompID compID, CodingStructure &cs)
{
  int width  = cs.picture->lwidth();
  int height = cs.picture->lheight();

  int blockWidth  = cs.pcv->maxCUWidth;
  int blockHeight = cs.pcv->maxCUHeight;

  int widthInBlocks  = width / blockWidth + (width % blockWidth != 0);
  int heightInBlocks = height / blockHeight + (height % blockHeight != 0);

  for (int i = 0; i < widthInBlocks * heightInBlocks; i++)
  {
    bif(compID, cs, i);
  }
}

void CABACReader::ccSaoControlIdc(CodingStructure &cs, const CompID compID, const int curIdx, uint8_t *controlIdc,
                                  Position lumaPos, int setNum)
{
  // RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__CROSS_COMPONENT_SAO_BLOCK_LEVEL_IDC );

  Position       leftLumaPos  = lumaPos.offset(-(int)cs.pcv->maxCUWidth, 0);
  Position       aboveLumaPos = lumaPos.offset(0, -(int)cs.pcv->maxCUWidth);
  const uint32_t curSliceIdx  = cs.slice->m_independentSliceIdx;
  const uint32_t curTileIdx   = cs.pps->getTileIdx(lumaPos);
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
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET2(STATS__CABAC_BITS__SAO, compID);
  int idcVal = m_binDecoder.decodeBin(Ctx::CcSaoControlIdc(ctxt));

  if (idcVal)
  {
    while ((idcVal != setNum) && m_binDecoder.decodeBinEP())
    {
      idcVal++;
    }
  }
  controlIdc[curIdx] = idcVal;

  DTRACE(g_trace_ctx, D_SYNTAX, "cc_sao_control_idc() compID=%d pos=(%d,%d) ctxt=%d, setNum=%d, idcVal=%d\n", compID,
         lumaPos.x, lumaPos.y, ctxt, setNum, idcVal);
}

void CABACReader::bif(const CompID compID, CodingStructure &cs, unsigned ctuRsAddr)
{
  //  const SPS&   sps = *cs.sps;
  const PPS &pps = *cs.pps;
  if (isLuma(compID) && !pps.m_BIF)
  {
    return;
  }
  if (isChroma(compID) && !pps.m_chromaBIF)
  {
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET2(STATS__CABAC_BITS__BIF, compID);

  BifParams &bifParams = cs.picture->getBifParam(compID);

  if (ctuRsAddr == 0)
  {
    int width       = cs.picture->lwidth();
    int height      = cs.picture->lheight();
    int blockWidth  = cs.pcv->maxCUWidth;
    int blockHeight = cs.pcv->maxCUHeight;

    int widthInBlocks   = width / blockWidth + (width % blockWidth != 0);
    int heightInBlocks  = height / blockHeight + (height % blockHeight != 0);
    bifParams.numBlocks = widthInBlocks * heightInBlocks;
    bifParams.ctuOn.resize(bifParams.numBlocks);
    std::fill(bifParams.ctuOn.begin(), bifParams.ctuOn.end(), 0);

    bifParams.allCtuOn = m_binDecoder.decodeBinEP();

    if (bifParams.allCtuOn == 0)
    {
      bifParams.frmOn = m_binDecoder.decodeBinEP();
    }
    else
    {
      bifParams.frmOn = 0;
    }
  }

  if (bifParams.allCtuOn)
  {
    bifParams.ctuOn[ctuRsAddr] = 1;
  }
  else
  {
    if (bifParams.frmOn)
    {
      bifParams.ctuOn[ctuRsAddr] = m_binDecoder.decodeBin(Ctx::BifCtrlFlags[compID]());
    }
    else
    {
      bifParams.ctuOn[ctuRsAddr] = 0;
    }
  }
}

//================================================================================
//  clause 7.3.8.4
//--------------------------------------------------------------------------------
//    void  coding_tree       ( cs, partitioner, cuCtx )
//    bool  split_cu_flag     ( cs, partitioner )
//    split split_cu_mode_mt  ( cs, partitioner )
//================================================================================

void CABACReader::coding_tree(CodingStructure &cs, Partitioner &partitioner, CUCtx &cuCtx,
                              Partitioner *pPartitionerChroma, CUCtx *pCuCtxChroma)
{
  const PPS      &pps      = *cs.pps;
  const UnitArea &currArea = partitioner.currArea();

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
    cs.chromaQpAdj           = 0;
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
      cs.chromaQpAdj                   = 0;
    }
  }

  DeriveCtx::setNeighbourCus(cs, partitioner.currArea(), partitioner.chType);

  const PartSplit splitMode = split_cu_mode(cs, partitioner);

  CHECK(!partitioner.canSplit(splitMode, cs), "Got an invalid split!");

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
          if (cs.area.block(partitioner.chType).contains(partitioner.currArea().block(partitioner.chType).pos()))
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
          if (cs.area.block(partitioner.chType).contains(partitioner.currArea().block(partitioner.chType).pos()))
          {
            coding_tree(cs, partitioner, cuCtx);
          }
          lumaContinue = partitioner.nextPart(cs);
          if (cs.area.block(pPartitionerChroma->chType)
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

      // cat the chroma CUs together
      CodingUnit *currentCu        = cs.getCU(partitioner.currArea().lumaPos(), ChannelType::LUMA);
      CodingUnit *nextCu           = nullptr;
      CodingUnit *tempLastLumaCu   = nullptr;
      CodingUnit *tempLastChromaCu = nullptr;
      ChannelType currentChType    = currentCu->chType;
      while (currentCu->next != nullptr)
      {
        nextCu = currentCu->next;
        if (currentChType != nextCu->chType && isLuma(currentChType))
        {
          tempLastLumaCu = currentCu;
          if (tempLastChromaCu != nullptr)   // swap
          {
            tempLastChromaCu->next = nextCu;
          }
        }
        else if (currentChType != nextCu->chType && currentChType == ChannelType::CHROMA)
        {
          tempLastChromaCu = currentCu;
          if (tempLastLumaCu != nullptr)   // swap
          {
            tempLastLumaCu->next = nextCu;
          }
        }
        currentCu     = nextCu;
        currentChType = currentCu->chType;
      }

      CodingUnit *chromaFirstCu = cs.getCU(pPartitionerChroma->currArea().chromaPos(), ChannelType::CHROMA);
      tempLastLumaCu->next      = chromaFirstCu;
    }
    else
    {
      partitioner.splitCurrArea(splitMode, cs);
      do
      {
        if (cs.area.block(partitioner.chType).contains(partitioner.currArea().block(partitioner.chType).pos()))
        {
          coding_tree(cs, partitioner, cuCtx);
        }
      } while (partitioner.nextPart(cs));

      partitioner.exitCurrSplit();
    }
    return;
  }

  CodingUnit &cu = cs.addCU(CS::getArea(cs, currArea, partitioner.chType), partitioner.chType);

  partitioner.setCUData(cu);
  cu.slice   = cs.slice;
  cu.tileIdx = cs.pps->getTileIdx(currArea.lumaPos());

  // Predict QP on start of quantization group
  if (cuCtx.qgStart)
  {
    cuCtx.qgStart = false;
    cuCtx.qp      = CU::predictQP(cu, cuCtx.qp);
  }

  if (pps.m_useDQP && CS::isDualITree(cs) && isChroma(cu.chType))
  {
    const Position    chromaCentral(cu.chromaPos().offset(cu.chromaSize().width >> 1, cu.chromaSize().height >> 1));
    const Position    lumaRefPos(chromaCentral.x << getComponentScaleX(COMP_Cb, cu.chromaFormat),
                                 chromaCentral.y << getComponentScaleY(COMP_Cb, cu.chromaFormat));
    // derive chroma qp, but the chroma qp is saved in cuCtx.qp which is used for luma qp
    // therefore, after decoding the chroma CU, the cuCtx.qp shall be recovered to luma qp in order to decode next luma
    // cu qp
    const CodingUnit *colLumaCu = cs.getLumaCU(lumaRefPos);

    if (colLumaCu)
    {
      cuCtx.qp = colLumaCu->qp;
    }
  }

  cu.qp          = cuCtx.qp;   // NOTE: CU QP can be changed by deltaQP signaling at TU level
  cu.chromaQpAdj = cs.chromaQpAdj;   // NOTE: CU chroma QP adjustment can be changed by adjustment signaling at TU level

  if (m_binDecoder
        .getBinBuffer())   // if context data collection is active and if on bottom of the CTU, start the counters
  {
    m_binDecoder.setBinBufferActive(CU::isOnCtuBottom(cu));
  }

  // coding unit
  coding_unit(cu, partitioner, cuCtx);

  if (m_binDecoder.getBinBuffer())   // done with the data collection for this CU
  {
    m_binDecoder.setBinBufferActive(false);
  }

  uint32_t compBegin;
  uint32_t numComp;
  bool     jointPLT = false;
  if (CS::isDualITree(*cu.cs))
  {
    if (isLuma(partitioner.chType))
    {
      compBegin = COMP_Y;
      numComp   = 1;
    }
    else
    {
      compBegin = COMP_Cb;
      numComp   = 2;
    }
  }
  else
  {
    compBegin = COMP_Y;
    numComp   = getNumberValidComponents(cu.chromaFormat);
    jointPLT  = true;
  }
  if (CU::isPLT(cu))
  {
    cs.reorderPrevPLT(cs.prevPLT, cu.curPLTSize, cu.curPLT, cu.reuseflag, compBegin, numComp, jointPLT);
  }

  DTRACE(g_trace_ctx, D_QP, "x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.lx(), cu.ly(), cu.lwidth(), cu.lheight(), cu.qp);
}

PartSplit CABACReader::split_cu_mode(CodingStructure &cs, Partitioner &partitioner)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(
    STATS__CABAC_BITS__SPLIT_FLAG, partitioner.currArea().block(partitioner.chType).size(), partitioner.chType);

  PartSplit mode = CU_DONT_SPLIT;

  bool canNo, canQt, canBh, canBv, canTh, canTv;
  partitioner.canSplit(cs, canNo, canQt, canBh, canBv, canTh, canTv);

  bool canSpl[6] = { canNo, canQt, canBh, canBv, canTh, canTv };

  unsigned ctxSplit = 0, ctxQtSplit = 0, ctxBttHV = 0, ctxBttH12 = 0, ctxBttV12;
  DeriveCtx::CtxSplit(cs, partitioner, ctxSplit, ctxQtSplit, ctxBttHV, ctxBttH12, ctxBttV12, canSpl);

  bool isSplit = canBh || canBv || canTh || canTv || canQt;
  bool canBtt  = canBh || canBv || canTh || canTv;
  bool isQt    = canQt;
  bool qtFrst  = false;

  if (canNo && isSplit)
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

      if (isQt && canBtt)
      {
        isQt = m_binDecoder.decodeBin(Ctx::SplitQtFlag(ctxQtSplit));

        if (isQt)
        {
          return CU_QUAD_SPLIT;
        }
        else
        {
          isSplit = canBtt;
        }
      }
    }

    if (canNo && isSplit)
    {
      isSplit = m_binDecoder.decodeBin(Ctx::SplitFlag(ctxSplit));
    }
    else
    {
      isSplit = false;
    }

    DTRACE(g_trace_ctx, D_SYNTAX, "split_cu_mode() depth=%d ctx=%d split=%d\n", partitioner.currDepth, ctxSplit,
           isSplit);
  }
#if ENABLE_TRACING
  else
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "split_cu_mode() depth=%d ctx=%d split=%d\n", partitioner.currDepth, ctxSplit,
           isSplit);
  }
#endif

  if (!isSplit)
  {
    return CU_DONT_SPLIT;
  }

  if (isQt && canBtt && !qtFrst)
  {
    isQt = m_binDecoder.decodeBin(Ctx::SplitQtFlag(ctxQtSplit));
  }

  DTRACE(g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d qt=%d\n", ctxQtSplit, isQt);

  if (isQt)
  {
    return CU_QUAD_SPLIT;
  }

  const bool canHor = canBh || canTh;
  bool       isVer  = canBv || canTv;

  if (isVer && canHor)
  {
    isVer = m_binDecoder.decodeBin(Ctx::SplitHvFlag(ctxBttHV));
  }

  const bool can14 = isVer ? canTv : canTh;
  bool       is12  = isVer ? canBv : canBh;

  if (is12 && can14)
  {
    is12 = m_binDecoder.decodeBin(Ctx::Split12Flag(isVer ? ctxBttV12 : ctxBttH12));
  }

  if (isVer && is12)
  {
    mode = CU_VERT_SPLIT;
  }
  else if (isVer && !is12)
  {
    mode = CU_TRIV_SPLIT;
  }
  else if (!isVer && is12)
  {
    mode = CU_HORZ_SPLIT;
  }
  else
  {
    mode = CU_TRIH_SPLIT;
  }

  DTRACE(g_trace_ctx, D_SYNTAX, "split_cu_mode() ctxHv=%d ctx12=%d mode=%d\n", ctxBttHV, isVer ? ctxBttV12 : ctxBttH12,
         mode);

  return mode;
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

void CABACReader::coding_unit(CodingUnit &cu, Partitioner &partitioner, CUCtx &cuCtx)
{
  CodingStructure &cs = *cu.cs;
  DTRACE(g_trace_ctx, D_SYNTAX, "coding_unit() %dx%d @ (%d,%d)\n", cu.blocks[int(cu.chType)].width,
         cu.blocks[int(cu.chType)].height, cu.blocks[int(cu.chType)].x, cu.blocks[int(cu.chType)].y);
  // skip flag
  if ((!cs.slice->isIntra() || cs.slice->m_ibcFlag) && cu.Y().valid())
  {
    cu_skip_flag(cu);
  }

  // skip data
  if (cu.skip)
  {
    cs.addEmptyTUs(partitioner);
    prediction_unit(cu);
    obmc_flag(cu);
    end_of_ctu(cu, cuCtx);
    return;
  }

  // prediction mode and partitioning data
  pred_mode(cu);
  if (CU::isPLT(cu))
  {
    cs.addTU(cu, partitioner.chType);
    if (CS::isDualITree(*cu.cs))
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

  // --> create PUs

  // prediction data ( intra prediction modes / reference indexes + motion vectors )
  cu_pred_data(cu);

  // residual data ( coded block flags + transform coefficient levels )
  cu_residual(cu, partitioner, cuCtx);

  // check end of cu
  end_of_ctu(cu, cuCtx);
}

void CABACReader::cu_skip_flag(CodingUnit &cu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__SKIP_FLAG);

  if (cu.slice->isIntra() && cu.cs->slice->m_ibcFlag)
  {
    cu.skip     = false;
    cu.rootCbf  = false;
    cu.predMode = MODE_INTRA;
    cu.mmvdSkip = false;

    if (!cu.slice->m_sps->m_ibcMerge)
    {
      return;
    }

    if (cu.lwidth() <= IBC_MAX_CU_SIZE && cu.lheight() <= IBC_MAX_CU_SIZE)   // disable IBC mode larger than 64x64
    {
      unsigned ctxId = DeriveCtx::CtxSkipFlag(cu);
      unsigned skip  = m_binDecoder.decodeBin(Ctx::SkipFlag(ctxId));
      DTRACE(g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, skip ? 1 : 0);
      if (skip)
      {
        cu.skip     = true;
        cu.rootCbf  = false;
        cu.predMode = MODE_IBC;
        cu.mmvdSkip = false;
      }
    }
    else
    {
      DTRACE(g_trace_ctx, D_SYNTAX, "cu_skip_flag() skip=0 (uncoded)\n");
    }
    return;
  }

  unsigned ctxId = DeriveCtx::CtxSkipFlag(cu);
  unsigned skip  = m_binDecoder.decodeBin(Ctx::SkipFlag(ctxId));

  DTRACE(g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, skip ? 1 : 0);

  if (skip && cu.cs->slice->m_ibcFlag)
  {
    if (!cu.slice->m_sps->m_ibcMerge)
    {
      cu.predMode = MODE_INTER;
    }
    else
      // disable IBC mode larger than 64x64 and disable IBC when only allowing inter mode
      if (cu.lwidth() <= IBC_MAX_CU_SIZE && cu.lheight() <= IBC_MAX_CU_SIZE)
      {
        unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
        if (m_binDecoder.decodeBin(Ctx::IBCFlag(ctxidx)))
        {
          cu.skip             = true;
          cu.rootCbf          = false;
          cu.predMode         = MODE_IBC;
          cu.mmvdSkip         = false;
          cu.regularMergeFlag = false;
        }
        else
        {
          cu.predMode = MODE_INTER;
        }
        DTRACE(g_trace_ctx, D_SYNTAX, "ibc() ctx=%d cu.predMode=%d\n", ctxidx, cu.predMode);
      }
      else
      {
        cu.predMode = MODE_INTER;
      }
  }
  if ((skip && CU::isInter(cu) && cu.cs->slice->m_ibcFlag) || (skip && !cu.cs->slice->m_ibcFlag))
  {
    cu.skip     = true;
    cu.rootCbf  = false;
    cu.predMode = MODE_INTER;
  }
}

void CABACReader::imv_mode(CodingUnit &cu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__IMV_FLAG);

  if (!cu.cs->sps->m_AMVREnabledFlag)
  {
    cu.imv = CU::isIBC(cu) && !cu.mergeFlag ? 1 : cu.imv;
    return;
  }
  bool useIBCFrac = CU::isIBC(cu) && !cu.mergeFlag && cu.cs->sps->m_ibcFracFlag;
  bool nonZeroMvd = CU::hasSubCUNonZeroMVd(cu);
  if (!nonZeroMvd)
  {
    cu.imv = CU::isIBC(cu) && !cu.mergeFlag ? (useIBCFrac ? IBC_SUBPEL_AMVR_MODE_FOR_ZERO_MVD : 1) : cu.imv;
    return;
  }

  if (cu.affine)
  {
    return;
  }

  const SPS    *sps    = cu.cs->sps;
  const CtxSet &imvCtx = CU::isIBC(cu) && cu.cs->sps->m_ibcFracFlag ? Ctx::ImvFlagIBC : Ctx::ImvFlag;

  unsigned value = 0;
  if (CU::isIBC(cu) && !useIBCFrac)
  {
    value = 1;
  }
  else
  {
    value = m_binDecoder.decodeBin(imvCtx(0));
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, 0);

  cu.imv = value;
  if (sps->m_AMVREnabledFlag && value)
  {
    if (!CU::isIBC(cu))
    {
      value = m_binDecoder.decodeBin(imvCtx(4));
      DTRACE(g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, 4);
      cu.imv = value ? 1 : IMV_HPEL;
    }
    if (value)
    {
      value = m_binDecoder.decodeBin(imvCtx(1));
      DTRACE(g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, 1);
      value++;
      cu.imv = value;
    }
  }

  if (CU::isIBC(cu))
  {
    CHECK(cu.cs->sps->m_ibcFracFlag && cu.imv == IMV_HPEL, "IBC does not support IMV_HPEL");
    CHECK(!useIBCFrac && (cu.imv == IMV_OFF || cu.imv == IMV_HPEL),
          "Fractiona IBC is not enabled to support fractional BVD coding");
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "imv_mode() IMVFlag=%d\n", cu.imv);
}

void CABACReader::affine_amvr_mode(CodingUnit &cu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__AFFINE_AMVR);

  const SPS *sps = cu.slice->m_sps;

  if (!sps->m_affineAmvrEnabledFlag || !cu.affine)
  {
    return;
  }

  if (!CU::hasSubCUNonZeroAffineMVd(cu))
  {
    return;
  }

  unsigned value = 0;
  value          = m_binDecoder.decodeBin(Ctx::ImvFlag(2));
  DTRACE(g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", value, 2);

  if (value)
  {
    value = m_binDecoder.decodeBin(Ctx::ImvFlag(3));
    DTRACE(g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", value, 3);
    value++;
  }

  cu.imv = value;
  DTRACE(g_trace_ctx, D_SYNTAX, "affine_amvr_mode() IMVFlag=%d\n", cu.imv);
}

void CABACReader::pred_mode(CodingUnit &cu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__PRED_MODE);
  if (cu.cs->slice->m_ibcFlag && cu.chType != ChannelType::CHROMA)
  {
    if (cu.cs->slice->isIntra())
    {
      cu.predMode = MODE_INTRA;
      if (cu.lwidth() <= IBC_MAX_CU_SIZE && cu.lheight() <= IBC_MAX_CU_SIZE)   // disable IBC mode larger than 64x64
      {
        unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
        if (m_binDecoder.decodeBin(Ctx::IBCFlag(ctxidx)))
        {
          cu.predMode = MODE_IBC;
        }
      }
      if (!CU::isIBC(cu) && CU::pltAllowed(cu))
      {
        if (m_binDecoder.decodeBin(Ctx::PLTFlag(0)))
        {
          cu.predMode = MODE_PLT;
        }
      }
    }
    else
    {
      if (m_binDecoder.decodeBin(Ctx::PredMode(DeriveCtx::CtxPredModeFlag(cu))))
      {
        cu.predMode = MODE_INTRA;
        if (CU::pltAllowed(cu))
        {
          if (m_binDecoder.decodeBin(Ctx::PLTFlag(0)))
          {
            cu.predMode = MODE_PLT;
          }
        }
      }
      else
      {
        cu.predMode = MODE_INTER;
        if (cu.lwidth() <= IBC_MAX_CU_SIZE && cu.lheight() <= IBC_MAX_CU_SIZE)   // disable IBC mode larger than 64x64
        {
          unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
          if (m_binDecoder.decodeBin(Ctx::IBCFlag(ctxidx)))
          {
            cu.predMode = MODE_IBC;
          }
        }
      }
    }
  }
  else
  {
    if (cu.cs->slice->isIntra())
    {
      cu.predMode = MODE_INTRA;
      if (CU::pltAllowed(cu))
      {
        if (m_binDecoder.decodeBin(Ctx::PLTFlag(0)))
        {
          cu.predMode = MODE_PLT;
        }
      }
    }
    else
    {
      cu.predMode = m_binDecoder.decodeBin(Ctx::PredMode(DeriveCtx::CtxPredModeFlag(cu))) ? MODE_INTRA : MODE_INTER;
      if (CU::isIntra(cu) && CU::pltAllowed(cu))
      {
        if (m_binDecoder.decodeBin(Ctx::PLTFlag(0)))
        {
          cu.predMode = MODE_PLT;
        }
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

void CABACReader::bdpcm_mode(CodingUnit &cu, const CompID compID)
{
  if (!CU::bdpcmAllowed(cu, compID))
  {
    if (isLuma(compID))
    {
      cu.bdpcmMode[0] = BdpcmMode::NONE;
    }
    if (!isLuma(compID) || (isLuma(compID) && !CS::isDualITree(*cu.cs)))
    {
      cu.bdpcmMode[1] = BdpcmMode::NONE;
    }
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__BDPCM_MODE, cu.block(compID).lumaSize(),
                                                      compID);

  BdpcmMode bdpcmMode = BdpcmMode::NONE;
  unsigned  ctxId     = isLuma(compID) ? 0 : 2;
  if (m_binDecoder.decodeBin(Ctx::BDPCMMode(ctxId)))
  {
    bdpcmMode = m_binDecoder.decodeBin(Ctx::BDPCMMode(ctxId + 1)) ? BdpcmMode::VER : BdpcmMode::HOR;
  }

  if (isLuma(compID))
  {
    cu.bdpcmMode[0] = bdpcmMode;
  }
  else
  {
    cu.bdpcmMode[1] = bdpcmMode;
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

void CABACReader::cu_pred_data(CodingUnit &cu)
{
  if (CU::isIntra(cu))
  {
    if (cu.Y().valid())
    {
      bdpcm_mode(cu, COMP_Y);
    }
    intra_luma_pred_mode(cu);

    if ((!cu.Y().valid() || (!CS::isDualITree(*cu.cs) && cu.Y().valid())) && isChromaEnabled(cu.chromaFormat))
    {
      bdpcm_mode(cu, CompID(ChannelType::CHROMA));
    }
    intra_chroma_pred_mode(cu);
    return;
  }
  if (!cu.Y().valid())   // dual tree chroma CU
  {
    cu.predMode = MODE_IBC;
    return;
  }

  prediction_unit(cu);

  imv_mode(cu);
  affine_amvr_mode(cu);
  cu_lic_flag(cu);
  cu_bcw_flag(cu);
  obmc_flag(cu);
}

void CABACReader::cu_bcw_flag(CodingUnit &cu)
{
  if (!CU::isBcwIdxCoded(cu))
  {
    return;
  }

  CHECK(!(BCW_NUM > 1 && (BCW_NUM == 2 || (BCW_NUM & 0x01) == 1)),
        " !( BCW_NUM > 1 && ( BCW_NUM == 2 || ( BCW_NUM & 0x01 ) == 1 ) ) ");

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__BCW_IDX);

  uint32_t idx = 0;

  uint32_t symbol = m_binDecoder.decodeBin(Ctx::bcwIdx(0));

  int32_t numBcw = (cu.slice->m_checkLdc) ? 5 : 3;
  if (symbol == 1)
  {
    uint32_t prefixNumBits = numBcw - 2;
    uint32_t step          = 1;

    idx = 1;

    for (int ui = 0; ui < prefixNumBits; ++ui)
    {
      symbol = m_binDecoder.decodeBinEP();
      if (symbol == 0)
      {
        break;
      }
      idx += step;
    }
  }

  cu.bcwIdx = (uint8_t)g_BcwParsingOrder[idx];

  DTRACE(g_trace_ctx, D_SYNTAX, "cu_bcw_flag() bcw_idx=%d\n", cu.bcwIdx ? 1 : 0);
}

void CABACReader::obmc_flag(CodingUnit &cu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__OBMC);

  if (!CU::isObmcAllowed(cu))
  {
    cu.obmcFlag = false;
    return;
  }
  if (cu.mergeFlag)
  {
    cu.obmcFlag = true;
    return;
  }

  const int ctxId = cu.affine ? 1 : 0;
  cu.obmcFlag     = m_binDecoder.decodeBin(Ctx::ObmcFlag(ctxId));
  DTRACE(g_trace_ctx, D_SYNTAX, "obmc_flag() pos=(%d,%d) size=(%d,%d) obmcFlag=%d\n", cu.lx(), cu.ly(),
         cu.lumaSize().width, cu.lumaSize().height, cu.obmcFlag);
}

void CABACReader::xReadTruncBinCode(uint32_t &symbol, uint32_t numSymbols)
{
  const int thresh = floorLog2(numSymbols);
  const int val    = 1 << thresh;
  const int b      = numSymbols - val;

  symbol = m_binDecoder.decodeBinsEP(thresh);
  if (symbol >= val - b)
  {
    symbol = 2 * symbol - (val - b) + m_binDecoder.decodeBinEP();
  }
}

void CABACReader::extend_ref_line(CodingUnit &cu)
{
  if (!cu.Y().valid() || !CU::isIntra(cu) || !isLuma(cu.chType) || cu.bdpcmMode[0] != BdpcmMode::NONE || cu.sgpm)
  {
    cu.multiRefIdx = 0;
    return;
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__MULTI_REF_LINE);

  if (!cu.cs->sps->m_useMRL)
  {
    cu.multiRefIdx = 0;
    return;
  }

  int multiRefIdx = 0;

  if (MRL_NUM_REF_LINES > 1)
  {
    multiRefIdx = m_binDecoder.decodeBin(Ctx::MultiRefLineIdx(0)) == 1 ? MULTI_REF_LINE_IDX[1] : MULTI_REF_LINE_IDX[0];
    if (MRL_NUM_REF_LINES > 2 && multiRefIdx != MULTI_REF_LINE_IDX[0])
    {
      multiRefIdx =
        m_binDecoder.decodeBin(Ctx::MultiRefLineIdx(1)) == 1 ? MULTI_REF_LINE_IDX[2] : MULTI_REF_LINE_IDX[1];
      if (MRL_NUM_REF_LINES > 3 && multiRefIdx != MULTI_REF_LINE_IDX[1])
      {
        uint32_t mrlOffset;
        xReadTruncBinCode(mrlOffset, (int)MRL_NUM_REF_LINES - 2);
        multiRefIdx = MULTI_REF_LINE_IDX[2 + mrlOffset];
      }
    }
  }
  cu.multiRefIdx = multiRefIdx;
}

void CABACReader::cclmDelta(int8_t &delta)
{
  if (delta)
  {
    int flag = 1;

    while (flag && delta < 4)
    {
      flag = m_binDecoder.decodeBinEP();
      delta += flag;
    }

    delta *= m_binDecoder.decodeBin(Ctx::CclmDeltaFlags(4)) ? -1 : +1;
  }
}

void CABACReader::cclmDeltaSlope(CodingUnit &cu)
{
  if (cu.cccmFlag)
  {
    return;
  }

  if (PU::hasCclmDeltaFlag(cu))
  {
    const int  chrMode     = cu.intraDir[ChannelType::CHROMA];
    const bool deltaActive = m_binDecoder.decodeBin(Ctx::CclmDeltaFlags(0));

    if (deltaActive)
    {
      bool bothActive = m_binDecoder.decodeBin(Ctx::CclmDeltaFlags(3));

      cu.cclmOffsets.cb0 = bothActive;
      cu.cclmOffsets.cr0 = bothActive;

      if (!bothActive)
      {
        cu.cclmOffsets.cb0 = m_binDecoder.decodeBin(Ctx::CclmDeltaFlags(1));

        if (PU::isMultiModeLM(chrMode) && !cu.cclmOffsets.cb0)
        {
          cu.cclmOffsets.cr0 = m_binDecoder.decodeBin(Ctx::CclmDeltaFlags(2));
        }
        else
        {
          cu.cclmOffsets.cr0 = !cu.cclmOffsets.cb0;
        }
      }

      cclmDelta(cu.cclmOffsets.cb0);
      cclmDelta(cu.cclmOffsets.cr0);

      // Now the same for the second model (if applicable)
      if (PU::isMultiModeLM(chrMode))
      {
        const bool bothActive = m_binDecoder.decodeBin(Ctx::CclmDeltaFlags(3));

        cu.cclmOffsets.cb1 = bothActive;
        cu.cclmOffsets.cr1 = bothActive;

        if (!bothActive)
        {
          cu.cclmOffsets.cb1 = m_binDecoder.decodeBin(Ctx::CclmDeltaFlags(1));

          if (cu.cclmOffsets.cb1)
          {
            cu.cclmOffsets.cr1 = 0;
          }
          else if (cu.cclmOffsets.cb0 == 0 && cu.cclmOffsets.cr0 == 0 && cu.cclmOffsets.cb1 == 0)
          {
            cu.cclmOffsets.cr1 = 1;
          }
          else
          {
            cu.cclmOffsets.cr1 = m_binDecoder.decodeBin(Ctx::CclmDeltaFlags(2));
          }
        }

        cclmDelta(cu.cclmOffsets.cb1);
        cclmDelta(cu.cclmOffsets.cr1);
      }
    }
  }
}

void CABACReader::intra_luma_pred_mode(CodingUnit &cu)
{
  if (!cu.Y().valid())
  {
    return;
  }
  cu.plDir = PlanarDirType::NO_DIR;
  if (cu.bdpcmMode[0] != BdpcmMode::NONE)
  {
    cu.intraDir[ChannelType::LUMA] = cu.bdpcmMode[0] == BdpcmMode::VER ? VER_IDX : HOR_IDX;
    return;
  }

  mip_flag(cu);
  if (cu.mipFlag)
  {
    mip_pred_modes(cu);
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

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__INTRA_DIR_ANG, cu.lumaSize(),
                                                      ChannelType::LUMA);

  // prev_intra_luma_pred_flag
  int mpmFlag;
  if (cu.multiRefIdx)
  {
    mpmFlag = true;
  }
  else
  {
    mpmFlag = m_binDecoder.decodeBin(Ctx::IntraLumaMpmFlag());
  }

  cu.mpmFlag    = mpmFlag;
  bool isPlanar = false;
  if (mpmFlag)
  {
    uint32_t predIdx = 0;
    {
      unsigned ctx  = 0;
      unsigned ctx2 = (cu.multiRefIdx == 0 ? 2 : 1);
      if (cu.multiRefIdx == 0)
      {
        predIdx = m_binDecoder.decodeBin(Ctx::IntraLumaPlanarFlag(ctx));
      }
      else
      {
        predIdx = 1;
      }
      if (predIdx)
      {
        predIdx += m_binDecoder.decodeBin(Ctx::IntraLumaMPMIdx(0 + ctx2));
      }
      if (predIdx > 1)
      {
        predIdx += m_binDecoder.decodeBinEP();
      }
      if (predIdx > 2)
      {
        predIdx += m_binDecoder.decodeBinEP();
      }
      if (predIdx > 3)
      {
        predIdx += m_binDecoder.decodeBinEP();
      }
    }
    cu.lumaModeIdx = predIdx;
    isPlanar       = (predIdx == 0);   // First MPM is always planar (not depnding on neighboring blocks)
  }
  else
  {
    cu.secondMpmFlag = m_binDecoder.decodeBin(Ctx::IntraLumaSecondMpmFlag());
    if (cu.secondMpmFlag)
    {
      cu.lumaModeIdx = m_binDecoder.decodeBinsEP(4);
    }
    else
    {
      xReadTruncBinCode(cu.lumaModeIdx, NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES);
    }
  }

  DTRACE(g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) mpm1st=%d mpm2nd=%d idx=%d\n", 0,
         cu.lumaPos().x, cu.lumaPos().y, cu.mpmFlag, cu.secondMpmFlag, cu.lumaModeIdx);
  if (isPlanar)
  {
    cu.intraDir[ChannelType::LUMA] = PLANAR_IDX;
    planarDir(cu);
  }
}

void CABACReader::decoderDerivedCcpModes(CodingUnit &cu)
{
  if (CU::hasDecoderDerivedCCP(cu))
  {
    cu.decDerivedCcpMode             = m_binDecoder.decodeBin(Ctx::decoderDerivedCCP(0));
    cu.intraDir[ChannelType::CHROMA] = LM_CHROMA_IDX;
  }
}

void CABACReader::nonLocalCCPIndex(CodingUnit &cu)
{
  cu.idxNonLocalCCP = 0;

  if (CU::hasNonLocalCCP(cu))
  {
    cu.idxNonLocalCCP = m_binDecoder.decodeBin(Ctx::nonLocalCCP(0));

    if (cu.idxNonLocalCCP)
    {
      if (cu.cs->slice->m_sps->m_ccMergeFusion)
      {
        cu.ccMergeFusionIdx = m_binDecoder.decodeBin(Ctx::CcMergeFusionFlag(0));
      }

      if (cu.ccMergeFusionIdx)
      {
        cu.ccMergeFusionIdx += unary_max_eqprob(MAX_CCP_FUSION_NUM - 1);
      }
      else
      {
        cu.idxNonLocalCCP += unary_max_eqprob(MAX_CCP_CAND_LIST_SIZE - 1);
      }
      cu.cccmFlag                      = 0;
      cu.intraDir[ChannelType::CHROMA] = LM_CHROMA_IDX;
    }
  }
}

void CABACReader::ccFilterFlag(CodingUnit &cu)
{
  cu.ccFilterFlag = false;

  if (CU::hasCcFilterFlag(cu))
  {
    cu.ccFilterFlag = m_binDecoder.decodeBin(Ctx::CcInsideFilterFlag(0));
  }
}

bool CABACReader::intra_chroma_lmc_mode(CodingUnit &cu)
{
  decoderDerivedCcpModes(cu);

  if (cu.decDerivedCcpMode)
  {
    return true;
  }

  nonLocalCCPIndex(cu);

  if (cu.idxNonLocalCCP)
  {
    return true;
  }

  int lmModeList[NUM_CHROMA_MODE];
  PU::getLMSymbolList(cu, lmModeList);

  int symbol = m_binDecoder.decodeBin(Ctx::CclmModeIdx(0));

  if (symbol == 0)
  {
    cu.intraDir[ChannelType::CHROMA] = lmModeList[symbol];
    CHECK(cu.intraDir[ChannelType::CHROMA] != LM_CHROMA_IDX, "should be LM_CHROMA");
  }
  else
  {
    int modeIdx = 1;   // MMLM_Chroma
    modeIdx += m_binDecoder.decodeBin(Ctx::MMLMFlag(0));

    if (modeIdx > 1)   // = 2 means MDLM_L
    {
      modeIdx += m_binDecoder.decodeBinEP();
    }

    if (modeIdx > 2)   // == 3 means MDLM_T
    {
      modeIdx += m_binDecoder.decodeBinEP();
    }
    if (modeIdx > 3)   // == 4 MMLM_L
    {
      modeIdx += m_binDecoder.decodeBinEP();
    }   // == 5 mean MMLM_T

    cu.intraDir[ChannelType::CHROMA] = lmModeList[modeIdx];
  }

  cccmFlag(cu);
  cclmDeltaSlope(cu);
  ccFilterFlag(cu);

  return true;   // it will only enter this function for LMC modes, so always return true ;
}

void CABACReader::sgpm_flag(CodingUnit &cu)
{
  if (!CU::isSgpmCoded(cu))
  {
    cu.sgpm = false;
    return;
  }

  if (cu.dimdFlag || cu.timdFlag || cu.mipFlag /*|| cu.tmpFlag*/)
  {
    cu.sgpm = false;
    return;
  }
  if (!cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType))
  {
    cu.sgpm = false;
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__SGPM);

  unsigned ctxId = DeriveCtx::CtxSgpmFlag(cu);
  cu.sgpm        = m_binDecoder.decodeBin(Ctx::SgpmFlag(ctxId));

  if (cu.sgpm)
  {
    uint32_t sgpmIdx = 0;
    xReadTruncBinCode(sgpmIdx, SGPM_NUM);
    cu.sgpmIdx = sgpmIdx;
  }
}

void CABACReader::intra_chroma_pred_mode(CodingUnit &cu)
{
  if (!isChromaEnabled(cu.chromaFormat) || (CS::isDualITree(*cu.cs) && isLuma(cu.chType)))
  {
    return;
  }
  cu.chromaModeIdx = -1;
  if (cu.bdpcmMode[1] != BdpcmMode::NONE)
  {
    cu.intraDir[ChannelType::CHROMA] = cu.bdpcmMode[1] == BdpcmMode::VER ? VER_IDX : HOR_IDX;
    return;
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__INTRA_DIR_ANG, cu.block(cu.chType).lumaSize(),
                                                      ChannelType::CHROMA);
  // LM chroma mode

  if (cu.cs->sps->m_LMChroma)
  {
    bool isLMCMode = m_binDecoder.decodeBin(Ctx::CclmModeFlag(0)) ? true : false;
    if (isLMCMode)
    {
      intra_chroma_lmc_mode(cu);
      return;
    }
  }

  if (m_binDecoder.decodeBin(Ctx::IntraChromaPredMode(0)) == 0)
  {
    cu.intraDir[ChannelType::CHROMA] = DM_CHROMA_IDX;
    return;
  }
  dimdChromaFlag(cu);
  if (cu.dimdChromaFlag)
  {
    return;
  }

  unsigned candId  = m_binDecoder.decodeBinsEP(2);
  cu.chromaModeIdx = candId;
  CHECK(candId >= NUM_CHROMA_MODE, "Chroma prediction mode index out of bounds");
}

void CABACReader::cccmFlag(CodingUnit &cu)
{
  const unsigned intraDir = cu.intraDir[ChannelType::CHROMA];

  if (PU::cccmAvailable(cu, intraDir))
  {
    if (PU::cccmAvailable(cu, intraDir, CONV_MODEL_CCCM_BVG) && PU::hasBvgCccmFlag(cu))
    {
      cu.cccmFlag = m_binDecoder.decodeBin(Ctx::BvgCccmFlag(0));

      if (cu.cccmFlag)
      {
        cu.cccmType = CONV_MODEL_CCCM_BVG;
        return;
      }
    }

    cu.cccmFlag = m_binDecoder.decodeBin(Ctx::CccmFlag(0));
    cu.cccmType = CONV_MODEL_UNDEFINED;

    if (cu.cccmFlag)
    {
      int mpfIndex = 0;
      cu.cccmType  = CONV_MODEL_CCCM;
      if (PU::cccmAvailable(cu, intraDir, CONV_MODEL_CCCM_MULTIF_1))
      {
        mpfIndex = m_binDecoder.decodeBin(Ctx::CccmMpfFlag(0));
      }

      if (mpfIndex)
      {
        mpfIndex += m_binDecoder.decodeBin(Ctx::CccmMpfFlag(1));
        mpfIndex += mpfIndex == 2 ? m_binDecoder.decodeBin(Ctx::CccmMpfFlag(2)) : 0;

        cu.cccmType = mpfIndex == 1 ? CONV_MODEL_CCCM_MULTIF_1
          : mpfIndex == 2           ? CONV_MODEL_CCCM_MULTIF_2
                                    : CONV_MODEL_CCCM_MULTIF_3;
      }
      else
      {
        cu.cccmType = m_binDecoder.decodeBin(Ctx::CccmFlag(1)) ? CONV_MODEL_CCCM_NOSUBS : cu.cccmType;

        if (cu.cccmType != CONV_MODEL_CCCM_NOSUBS)
        {
          cu.cccmType = m_binDecoder.decodeBin(Ctx::CccmFlag(2)) ? CONV_MODEL_CCCM_GRADLOC : cu.cccmType;
        }
      }
    }
  }
}

void CABACReader::cu_residual(CodingUnit &cu, Partitioner &partitioner, CUCtx &cuCtx)
{
  if (!CU::isIntra(cu))
  {
    if (!cu.mergeFlag)
    {
      rqt_root_cbf(cu);
    }
    else
    {
      cu.rootCbf = true;
    }
    if (cu.rootCbf)
    {
      sbt_mode(cu);
    }
    if (!cu.rootCbf)
    {
      cu.cs->addEmptyTUs(partitioner);
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

void CABACReader::rqt_root_cbf(CodingUnit &cu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__QT_ROOT_CBF);

  cu.rootCbf = (m_binDecoder.decodeBin(Ctx::QtRootCbf()));

  DTRACE(g_trace_ctx, D_SYNTAX, "rqt_root_cbf() ctx=0 root_cbf=%d pos=(%d,%d)\n", cu.rootCbf ? 1 : 0, cu.lumaPos().x,
         cu.lumaPos().y);
}

void CABACReader::sbt_mode(CodingUnit &cu)
{
  const uint8_t sbtAllowed = cu.checkAllowedSbt();
  if (!sbtAllowed)
  {
    return;
  }

  SizeType cuWidth  = cu.lwidth();
  SizeType cuHeight = cu.lheight();

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__SBT_MODE);

  // sbt flag
  uint8_t ctxIdx  = (cuWidth * cuHeight <= 256) ? 1 : 0;
  bool    sbtFlag = m_binDecoder.decodeBin(Ctx::SbtFlag(ctxIdx));
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
      const bool sbtRectFlag = (isLast || m_binDecoder.decodeBin(Ctx::SbtQuadFlag(1)));
      if (sbtRectFlag)
      {
        // rectangular type
        bool sbtRectQuadFlag = false;
        if ((sbtHorHalfAllowed || sbtVerHalfAllowed) && (sbtHorQuadAllowed || sbtVerQuadAllowed))
        {
          sbtRectQuadFlag = m_binDecoder.decodeBin(Ctx::SbtQuadFlag(0));
        }

        // rectangular direction
        bool sbtRectHorFlag = false;
        if ((sbtRectQuadFlag && sbtVerQuadAllowed && sbtHorQuadAllowed) ||
            (!sbtRectQuadFlag && sbtVerHalfAllowed && sbtHorHalfAllowed))   // both direction allowed
        {
          uint8_t ctxIdx = (cuWidth == cuHeight) ? 0 : (cuWidth < cuHeight ? 1 : 2);
          sbtRectHorFlag = m_binDecoder.decodeBin(Ctx::SbtHorFlag(ctxIdx));
        }
        else
        {
          sbtRectHorFlag = ((sbtRectQuadFlag && sbtHorQuadAllowed) || (!sbtRectQuadFlag && sbtHorHalfAllowed));
        }
        cu.setSbtIdx(sbtRectHorFlag ? (sbtRectQuadFlag ? SBT_HOR_QUAD : SBT_HOR_HALF)
                                    : (sbtRectQuadFlag ? SBT_VER_QUAD : SBT_VER_HALF));

        // rectangular position
        bool sbtRectPosFlag = m_binDecoder.decodeBin(Ctx::SbtPosFlag(0));
        cu.setSbtPos(sbtRectPosFlag ? SBT_POS1 : SBT_POS0);

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
      const bool bSbtQuad = (isLast || m_binDecoder.decodeBin(Ctx::SbtQuadFlag(2)));
      if (bSbtQuad)
      {
        cu.setSbtIdx(SBT_QUAD);
        // position
        const uint8_t horIdx = m_binDecoder.decodeBin(Ctx::SbtPosFlag(1));
        const uint8_t verIdx = m_binDecoder.decodeBin(Ctx::SbtPosFlag(2));
        cu.setSbtPos((verIdx << 1) + horIdx);
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
      const bool bSbtQuarter = (isLast || m_binDecoder.decodeBin(Ctx::SbtQuadFlag(3)));
      if (bSbtQuarter)
      {
        cu.setSbtIdx(SBT_QUARTER);
        // position
        const uint8_t horIdx = m_binDecoder.decodeBin(Ctx::SbtPosFlag(3));
        const uint8_t verIdx = m_binDecoder.decodeBin(Ctx::SbtPosFlag(4));
        cu.setSbtPos((verIdx << 1) + horIdx);
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

void CABACReader::end_of_ctu(CodingUnit &cu, CUCtx &cuCtx)
{
  const Position rbPos =
    recalcPosition(cu.chromaFormat, cu.chType, ChannelType::LUMA, cu.block(cu.chType).bottomRight().offset(1, 1));

  if (((rbPos.x & cu.cs->pcv->maxCUWidthMask) == 0 || rbPos.x == cu.cs->pps->m_picWidthInLumaSamples) &&
      ((rbPos.y & cu.cs->pcv->maxCUHeightMask) == 0 || rbPos.y == cu.cs->pps->m_picHeightInLumaSamples) &&
      (!CS::isDualITree(*cu.cs) || !isChromaEnabled(cu.chromaFormat) || isChroma(cu.chType)))
  {
    cuCtx.isDQPCoded = (cu.cs->pps->m_useDQP && !cuCtx.isDQPCoded);
  }
}

void CABACReader::cu_palette_info(CodingUnit &cu, CompID compBegin, uint32_t numComp, CUCtx &cuCtx)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__PLT_MODE);

  const SPS     &sps       = *(cu.cs->sps);
  TransformUnit &tu        = *cu.firstTU;
  int            curPLTidx = 0;

  cu.lastPLTSize[compBegin] = cu.cs->prevPLT.curPLTSize[compBegin];

  int maxPltSize = CS::isDualITree(*cu.cs) ? MAXPLTSIZE_DUALTREE : MAXPLTSIZE;

  if (cu.lastPLTSize[compBegin])
  {
    xDecodePLTPredIndicator(cu, maxPltSize, compBegin);
  }

  for (int idx = 0; idx < cu.lastPLTSize[compBegin]; idx++)
  {
    if (cu.reuseflag[compBegin][idx])
    {
      for (int comp = compBegin; comp < (compBegin + numComp); comp++)
      {
        cu.curPLT[comp][curPLTidx] = cu.cs->prevPLT.curPLT[comp][idx];
      }
      curPLTidx++;
    }
  }

  uint32_t recievedPLTnum = 0;
  if (curPLTidx < maxPltSize)
  {
    recievedPLTnum = exp_golomb_eqprob(0);
  }

  cu.curPLTSize[compBegin] = curPLTidx + recievedPLTnum;
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    for (int idx = curPLTidx; idx < cu.curPLTSize[compBegin]; idx++)
    {
      CompID    compID          = (CompID)comp;
      const int channelBitDepth = sps.m_bitDepths[toChannelType(compID)];
      cu.curPLT[compID][idx]    = m_binDecoder.decodeBinsEP(channelBitDepth);
    }
  }

  cu.useEscape[compBegin] = true;
  if (cu.curPLTSize[compBegin] > 0)
  {
    uint32_t escCode        = 0;
    escCode                 = m_binDecoder.decodeBinEP();
    cu.useEscape[compBegin] = (escCode != 0);
  }
  uint32_t indexMaxSize = cu.useEscape[compBegin] ? (cu.curPLTSize[compBegin] + 1) : cu.curPLTSize[compBegin];
  // encode index map
  uint32_t height       = cu.block(compBegin).height;
  uint32_t width        = cu.block(compBegin).width;

  uint32_t total = height * width;
  if (indexMaxSize > 1)
  {
    parseScanRotationModeFlag(cu, compBegin);
  }
  else
  {
    cu.useRotation[compBegin] = false;
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
    if (!CS::isDualITree(*cu.cs) || isChroma(tu.chType))
    {
      cu_chroma_qp_offset(cu);
      cuCtx.isChromaQpAdjCoded = true;
    }
  }

  m_scanOrder =
    g_scanOrder[SCAN_UNGROUPED][(cu.useRotation[compBegin]) ? CoeffScanType::TRAV_VER : CoeffScanType::TRAV_HOR]
               [gp_sizeIdxInfo->idxFrom(width)][gp_sizeIdxInfo->idxFrom(height)];
  uint32_t prevRunPos  = 0;
  unsigned prevRunType = 0;
  for (int subSetId = 0; subSetId <= (total - 1) >> LOG2_PALETTE_CG_SIZE; subSetId++)
  {
    cuPaletteSubblockInfo(cu, compBegin, numComp, subSetId, prevRunPos, prevRunType);
  }
  CHECK(cu.curPLTSize[compBegin] > maxPltSize, " Current palette size is larger than maximum palette size");
}

void CABACReader::cuPaletteSubblockInfo(CodingUnit &cu, CompID compBegin, uint32_t numComp, int subSetId,
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
    uint32_t posy         = m_scanOrder[curPos].y;
    uint32_t posx         = m_scanOrder[curPos].x;
    uint32_t posyprev     = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].y;
    uint32_t posxprev     = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].x;
    unsigned identityFlag = 1;

    const CtxSet &ctxSet = (prevRunType == PLT_RUN_INDEX) ? Ctx::IdxRunModel : Ctx::CopyRunModel;
    if (curPos > 0)
    {
      int            dist  = curPos - prevRunPos - 1;
      const unsigned ctxId = DeriveCtx::CtxPltCopyFlag(prevRunType, dist);
      identityFlag         = m_binDecoder.decodeBin(ctxSet(ctxId));
      DTRACE(g_trace_ctx, D_SYNTAX, "plt_copy_flag() bin=%d ctx=%d\n", identityFlag, ctxId);
      runCopyFlag[curPos - minSubPos] = identityFlag;
    }

    if (identityFlag == 0 || curPos == 0)
    {
      if (((posy == 0) && !cu.useRotation[compBegin]) || ((posx == 0) && cu.useRotation[compBegin]))
      {
        runType.at(posx, posy) = PLT_RUN_INDEX;
      }
      else if (curPos != 0 && runType.at(posxprev, posyprev) == PLT_RUN_COPY)
      {
        runType.at(posx, posy) = PLT_RUN_INDEX;
      }
      else
      {
        runType.at(posx, posy) = (m_binDecoder.decodeBin(Ctx::RunTypeFlag()));
      }
      DTRACE(g_trace_ctx, D_SYNTAX, "plt_type_flag() bin=%d sp=%d\n", runType.at(posx, posy), curPos);
      prevRunType = runType.at(posx, posy);
      prevRunPos  = curPos;
    }
    else   // assign run information
    {
      runType.at(posx, posy) = runType.at(posxprev, posyprev);
    }
  }

  // PLT index values - bypass coded
  uint32_t adjust;
  uint32_t symbol = 0;
  curPos          = minSubPos;
  if (indexMaxSize > 1)
  {
    for (; curPos < maxSubPos; curPos++)
    {
      if (curPos > 0)
      {
        adjust = 1;
      }
      else
      {
        adjust = 0;
      }

      uint32_t posy     = m_scanOrder[curPos].y;
      uint32_t posx     = m_scanOrder[curPos].x;
      uint32_t posyprev = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].y;
      uint32_t posxprev = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].x;
      if (runCopyFlag[curPos - minSubPos] == 0 && runType.at(posx, posy) == PLT_RUN_INDEX)
      {
        xReadTruncBinCode(symbol, indexMaxSize - adjust);
        xAdjustPLTIndex(cu, symbol, curPos, curPLTIdx, runType, indexMaxSize, compBegin);
        DTRACE(g_trace_ctx, D_SYNTAX, "plt_idx_idc() value=%d sp=%d\n", curPLTIdx.at(posx, posy), curPos);
      }
      else if (runType.at(posx, posy) == PLT_RUN_INDEX)
      {
        curPLTIdx.at(posx, posy) = curPLTIdx.at(posxprev, posyprev);
      }
      else
      {
        curPLTIdx.at(posx, posy) =
          (cu.useRotation[compBegin]) ? curPLTIdx.at(posx - 1, posy) : curPLTIdx.at(posx, posy - 1);
      }
    }
  }
  else
  {
    for (; curPos < maxSubPos; curPos++)
    {
      uint32_t posy          = m_scanOrder[curPos].y;
      uint32_t posx          = m_scanOrder[curPos].x;
      uint32_t posyprev      = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].y;
      uint32_t posxprev      = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].x;
      runType.at(posx, posy) = PLT_RUN_INDEX;
      if (runCopyFlag[curPos - minSubPos] == 0 && runType.at(posx, posy) == PLT_RUN_INDEX)
      {
        curPLTIdx.at(posx, posy) = 0;
      }
      else
      {
        curPLTIdx.at(posx, posy) = curPLTIdx.at(posxprev, posyprev);
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
          escapeValue.at(posx, posy) = exp_golomb_eqprob(5);
          assert(escapeValue.at(posx, posy) <
                 (TCoeff(1) << (cu.cs->sps->m_bitDepths[toChannelType((CompID)comp)] + 1)));
          DTRACE(g_trace_ctx, D_SYNTAX, "plt_escape_val() value=%d etype=%d sp=%d\n", escapeValue.at(posx, posy), comp,
                 curPos);
        }
        if (compBegin == COMP_Y && compID != COMP_Y && posy % (1 << scaleY) == 0 && posx % (1 << scaleX) == 0)
        {
          uint32_t posxC               = posx >> scaleX;
          uint32_t posyC               = posy >> scaleY;
          escapeValue.at(posxC, posyC) = exp_golomb_eqprob(5);
          assert(escapeValue.at(posxC, posyC) < (TCoeff(1) << (cu.cs->sps->m_bitDepths[toChannelType(compID)] + 1)));
          DTRACE(g_trace_ctx, D_SYNTAX, "plt_escape_val() value=%d etype=%d sp=%d\n", escapeValue.at(posx, posy), comp,
                 curPos);
        }
      }
    }
  }
}

void CABACReader::parseScanRotationModeFlag(CodingUnit &cu, CompID compBegin)
{
  cu.useRotation[compBegin] = m_binDecoder.decodeBin(Ctx::RotationFlag());
}

void CABACReader::xDecodePLTPredIndicator(CodingUnit &cu, uint32_t maxPLTSize, CompID compBegin)
{
  uint32_t symbol, numPltPredicted = 0, idx = 0;

  symbol = exp_golomb_eqprob(0);

  if (symbol != 1)
  {
    while (idx < cu.lastPLTSize[compBegin] && numPltPredicted < maxPLTSize)
    {
      if (idx > 0)
      {
        symbol = exp_golomb_eqprob(0);
      }
      if (symbol == 1)
      {
        break;
      }

      if (symbol)
      {
        idx += symbol - 1;
      }
      cu.reuseflag[compBegin][idx] = 1;
      numPltPredicted++;
      idx++;
    }
  }
}
void CABACReader::xAdjustPLTIndex(CodingUnit &cu, Pel curLevel, uint32_t idx, PelBuf &paletteIdx,
                                  PLTtypeBuf &paletteRunType, int maxSymbol, CompID compBegin)
{
  uint32_t symbol;
  int      refLevel = MAX_INT;
  uint32_t posy     = m_scanOrder[idx].y;
  uint32_t posx     = m_scanOrder[idx].x;
  if (idx)
  {
    uint32_t prevposy = m_scanOrder[idx - 1].y;
    uint32_t prevposx = m_scanOrder[idx - 1].x;
    if (paletteRunType.at(prevposx, prevposy) == PLT_RUN_INDEX)
    {
      refLevel = paletteIdx.at(prevposx, prevposy);
      if (paletteIdx.at(prevposx, prevposy) == cu.curPLTSize[compBegin])   // escape
      {
        refLevel = maxSymbol - 1;
      }
    }
    else
    {
      if (cu.useRotation[compBegin])
      {
        assert(prevposx > 0);
        refLevel = paletteIdx.at(posx - 1, posy);
        if (paletteIdx.at(posx - 1, posy) == cu.curPLTSize[compBegin])   // escape mode
        {
          refLevel = maxSymbol - 1;
        }
      }
      else
      {
        assert(prevposy > 0);
        refLevel = paletteIdx.at(posx, posy - 1);
        if (paletteIdx.at(posx, posy - 1) == cu.curPLTSize[compBegin])   // escape mode
        {
          refLevel = maxSymbol - 1;
        }
      }
    }
    maxSymbol--;
  }
  symbol = curLevel;
  if (curLevel >= refLevel)   // include escape mode
  {
    symbol++;
  }
  paletteIdx.at(posx, posy) = symbol;
}

//================================================================================
//  clause 7.3.8.6
//--------------------------------------------------------------------------------
//    void  prediction_unit ( cu, mrgCtx );
//    void  merge_flag      ( cu );
//    void  merge_data      ( cu, mrgCtx );
//    void  merge_idx       ( cu );
//    void  inter_pred_idc  ( cu );
//    void  ref_idx         ( cu, refList );
//    void  mvp_flag        ( cu, refList );
//================================================================================

void CABACReader::prediction_unit(CodingUnit &cu)
{
  if (cu.skip)
  {
    cu.mergeFlag = true;
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
    cu.interDir     = 1;
    cu.affine       = false;
    cu.refIdx[RPL0] = IBC_REF_IDX;
    bvdCoding(cu.mvd[RPL0]);
    if (cu.cs->sps->m_maxNumIBCMergeCand == 1)
    {
      cu.mvpIdx[RPL0] = 0;
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
          mvd_coding(cu.mvdAffi[RPL0][i]);
        }
      }
      else
      {
        mvd_coding(cu.mvd[RPL0]);
      }
      mvp_flag(cu, RPL0);
    }

    if (cu.interDir != 1 /* PRED_L0 */)
    {
      if (cu.smvdMode != 1)
      {
        ref_idx(cu, RPL1);
        if (cu.cs->picHeader->m_mvdL1ZeroFlag && cu.interDir == 3 /* PRED_BI */)
        {
          cu.mvd[RPL1]        = Mv();
          cu.mvdAffi[RPL1][0] = Mv();
          cu.mvdAffi[RPL1][1] = Mv();
          cu.mvdAffi[RPL1][2] = Mv();
        }
        else if (cu.affine)
        {
          for (int i = 0; i < cu.getNumAffineMvs(); i++)
          {
            mvd_coding(cu.mvdAffi[RPL1][i]);
          }
        }
        else
        {
          mvd_coding(cu.mvd[RPL1]);
        }
      }
      mvp_flag(cu, RPL1);
    }
  }
  if (cu.interDir == 3 /* PRED_BI */ && PU::isBipredRestriction(cu))
  {
    cu.mv[RPL1]     = Mv(0, 0);
    cu.refIdx[RPL1] = -1;
    cu.interDir     = 1;
    cu.bcwIdx       = BCW_DEFAULT;
  }

  if (cu.smvdMode)
  {
    RefPicList eCurRefList = (RefPicList)(cu.smvdMode - 1);
    cu.mvd[1 - eCurRefList].set(-cu.mvd[eCurRefList].hor, -cu.mvd[eCurRefList].ver);
    CHECK(!((cu.mvd[1 - eCurRefList].getHor() >= MVD_MIN) && (cu.mvd[1 - eCurRefList].getHor() <= MVD_MAX)) ||
            !((cu.mvd[1 - eCurRefList].getVer() >= MVD_MIN) && (cu.mvd[1 - eCurRefList].getVer() <= MVD_MAX)),
          "Illegal MVD value");
    cu.refIdx[1 - eCurRefList] = cu.cs->slice->getSymRefIdx(1 - eCurRefList);
  }
}

void CABACReader::smvd_mode(CodingUnit &cu)
{
  cu.smvdMode = 0;
  if (cu.interDir != 3 || cu.affine)
  {
    return;
  }

  if (cu.cs->slice->getBiDirPred() == false)
  {
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__SYMMVD_FLAG);

  cu.smvdMode = m_binDecoder.decodeBin(Ctx::SmvdFlag()) ? 1 : 0;

  DTRACE(g_trace_ctx, D_SYNTAX, "symmvd_flag() symmvd=%d pos=(%d,%d) size=%dx%d\n", cu.smvdMode ? 1 : 0, cu.lumaPos().x,
         cu.lumaPos().y, cu.lumaSize().width, cu.lumaSize().height);
}

void CABACReader::subblock_merge_flag(CodingUnit &cu)
{
  cu.affine = false;

  if (!cu.cs->slice->isIntra() && (cu.slice->m_picHeader->m_maxNumAffineMergeCand > 0) && CU::isAffineAllowed(cu))
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__AFFINE_FLAG);

    unsigned ctxId = DeriveCtx::CtxAffineFlag(cu);
    cu.affine      = m_binDecoder.decodeBin(Ctx::SubblockMergeFlag(ctxId));
    DTRACE(g_trace_ctx, D_SYNTAX, "subblock_merge_flag() subblock_merge_flag=%d ctx=%d pos=(%d,%d)\n",
           cu.affine ? 1 : 0, ctxId, cu.lx(), cu.ly());
  }
}

void CABACReader::affine_flag(CodingUnit &cu)
{
  if (!cu.cs->slice->isIntra() && cu.cs->sps->m_useAffine &&
      std::min(cu.lumaSize().width, cu.lumaSize().height) >=
        (1 << (cu.cs->slice->m_sps->m_log2MinAffineBlkSizeMinus3 + 3)))
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__AFFINE_FLAG);

    unsigned ctxId = DeriveCtx::CtxAffineFlag(cu);
    cu.affine      = m_binDecoder.decodeBin(Ctx::AffineFlag(ctxId));
    DTRACE(g_trace_ctx, D_SYNTAX, "affine_flag() affine=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.lx(),
           cu.ly());

    if (cu.affine && cu.cs->sps->m_AffineType)
    {
      ctxId         = 0;
      cu.affineType = m_binDecoder.decodeBin(Ctx::AffineType(ctxId)) ? AffineModel::_6_PARAMS : AffineModel::_4_PARAMS;
      DTRACE(g_trace_ctx, D_SYNTAX, "affine_type() affine_type=%d ctx=%d pos=(%d,%d)\n",
             cu.affineType == AffineModel::_6_PARAMS ? 1 : 0, ctxId, cu.lx(), cu.ly());
    }
    else
    {
      cu.affineType = AffineModel::_4_PARAMS;
    }
  }
}

void CABACReader::affine_mmvd_data(CodingUnit &pu)
{
  if (!pu.cs->sps->m_AffineMmvdMode || !pu.mergeFlag || !pu.affine)
  {
    return;
  }

  pu.mmvdMergeFlag = (m_binDecoder.decodeBin(Ctx::AffMmvdFlag()));
  DTRACE(g_trace_ctx, D_SYNTAX, "affine_mmvd_flag() af_mmvd_merge=%d pos=(%d,%d) size=%dx%d\n",
         pu.mmvdMergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);

  if (!pu.mmvdMergeFlag)
  {
    return;
  }

  // Base affine merge candidate idx
  uint8_t affMmvdBaseIdx    = 0;
  int     numCandminus1Base = AF_MMVD_BASE_NUM - 1;
  if (numCandminus1Base > 0)
  {
    // to support more base candidates
    if (m_binDecoder.decodeBin(Ctx::AffMmvdIdx()))
    {
      affMmvdBaseIdx++;
      for (; affMmvdBaseIdx < numCandminus1Base; affMmvdBaseIdx++)
      {
        if (!m_binDecoder.decodeBinEP())
        {
          break;
        }
      }
    }
  }

  // Decode Step Value
  uint8_t stepOffset        = 0;
  int     numCandminus1Step = AF_MMVD_STEP_NUM - 1;
  if (numCandminus1Step > 0)
  {
    if (m_binDecoder.decodeBin(Ctx::AffMmvdOffsetStep()))
    {
      stepOffset++;
      for (; stepOffset < numCandminus1Step; stepOffset++)
      {
        if (!m_binDecoder.decodeBinEP())
        {
          break;
        }
      }
    }
  }

  // Decode offset direction
  uint8_t b0 = 0, b1 = 0;
  b0             = m_binDecoder.decodeBinEP();
  b1             = m_binDecoder.decodeBinEP();
  uint8_t offDir = (b1 << 1) | b0;

  pu.mmvdMergeIdx.pos.position = offDir;
  pu.mmvdMergeIdx.pos.step     = stepOffset;
  pu.mmvdMergeIdx.pos.baseIdx  = affMmvdBaseIdx;
  DTRACE(g_trace_ctx, D_SYNTAX, "affine_mmvd_data() base=%d dir=%d step=%d\n", pu.mmvdMergeIdx.pos.baseIdx,
         pu.mmvdMergeIdx.pos.position, pu.mmvdMergeIdx.pos.step);
}

void CABACReader::bm_merge_flag(CodingUnit &cu)
{
  cu.bmDir       = 0;
  cu.bmMergeFlag = false;

  if (!PU::isBMMergeFlagCoded(cu))
  {
    return;
  }

  unsigned ctxId = DeriveCtx::CtxBMMrgFlag(cu);
  cu.bmMergeFlag = m_binDecoder.decodeBin(Ctx::BMMergeFlag(ctxId));

  if (cu.bmMergeFlag)
  {
    cu.bmDir = 1 << m_binDecoder.decodeBin(Ctx::BMMergeFlag(3));
  }

  DTRACE(g_trace_ctx, D_SYNTAX, "bm_merge_flag() bmMergeFlag=%d, bmDir=%d\n", cu.bmMergeFlag ? 1 : 0, cu.bmDir);
}

void CABACReader::merge_flag(CodingUnit &cu)
{
  if (CU::isIBC(cu) && !cu.slice->m_sps->m_ibcMerge)
  {
    cu.mergeFlag = false;
    return;
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__MERGE_FLAG);

  cu.mergeFlag = (m_binDecoder.decodeBin(Ctx::MergeFlag()));

  DTRACE(g_trace_ctx, D_SYNTAX, "merge_flag() merge=%d pos=(%d,%d) size=%dx%d\n", cu.mergeFlag ? 1 : 0, cu.lumaPos().x,
         cu.lumaPos().y, cu.lumaSize().width, cu.lumaSize().height);

  if (cu.mergeFlag && CU::isIBC(cu))
  {
    cu.mmvdMergeFlag    = false;
    cu.regularMergeFlag = false;
    return;
  }
}

void CABACReader::merge_data(CodingUnit &cu)
{
  if (CU::isIBC(cu))
  {
    merge_idx(cu);
    return;
  }
  else
  {
    subblock_merge_flag(cu);
    if (cu.affine)
    {
      RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__AFFINE_DATA);
      affine_mmvd_data(cu);
      if (CU::hasOppositeLICFlag(cu))
      {
        cu.oppositeLicFlag = m_binDecoder.decodeBin(Ctx::AffineFlagOppositeLic(0));
      }
      merge_idx(cu);
      cu.regularMergeFlag = false;
      return;
    }

    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__MERGE_FLAG);

    const int  maxSize       = std::min<int>(MAX_TB_SIZEY, MAX_INTRA_SIZE);
    const bool ciipAvailable = cu.cs->sps->m_useCiip && !cu.skip && cu.lwidth() <= maxSize && cu.lheight() <= maxSize &&
      cu.lwidth() * cu.lheight() >= CIIP_MIN_AREA;
    const bool geoAvailable = CU::canUseGPM(cu);

    if (geoAvailable || ciipAvailable)
    {
      cu.regularMergeFlag = m_binDecoder.decodeBin(Ctx::RegularMergeFlag(cu.skip ? 0 : 1));
    }
    else
    {
      cu.regularMergeFlag = true;
    }
    if (cu.regularMergeFlag)
    {
      bm_merge_flag(cu);
      if (cu.cs->slice->m_sps->m_useMMVD && !cu.bmMergeFlag)
      {
        cu.mmvdMergeFlag = m_binDecoder.decodeBin(Ctx::MmvdFlag(0));
        DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_flag() mmvd_merge=%d pos=(%d,%d) size=%dx%d\n",
               cu.mmvdMergeFlag ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y, cu.lumaSize().width, cu.lumaSize().height);
      }
      else
      {
        cu.mmvdMergeFlag = false;
      }
      if (cu.skip)
      {
        cu.mmvdSkip = cu.mmvdMergeFlag;
      }
    }
    else
    {
      cu.mmvdMergeFlag = false;
      cu.mmvdSkip      = false;
      cu.bmMergeFlag   = false;
      if (geoAvailable && ciipAvailable)
      {
        ciip_flag(cu, false);
      }
      else if (ciipAvailable)
      {
        ciip_flag(cu, true);
      }
      else
      {
        cu.ciipFlag = false;
      }
      if (cu.ciipFlag)
      {
        cu.intraDir[ChannelType::LUMA]   = PLANAR_IDX;
        cu.intraDir[ChannelType::CHROMA] = DM_CHROMA_IDX;
      }
      else
      {
        cu.geoFlag = true;
        PU::setGpmDirMode(cu);
      }
    }
  }
  if (cu.mmvdMergeFlag || cu.mmvdSkip)
  {
    mmvd_merge_idx(cu);
  }
  else
  {
    if (CU::hasOppositeLICFlag(cu))
    {
      cu.oppositeLicFlag = m_binDecoder.decodeBin(Ctx::MergeFlagOppositeLic(0));
    }
    merge_idx(cu);
  }
}

void CABACReader::merge_idx(CodingUnit &cu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__MERGE_INDEX);

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
    cu.mergeIdx = 0;
    if (numCandminus1 > 0)
    {
      unsigned int unaryIdx = 0;
      for (; unaryIdx < numCandminus1; ++unaryIdx)
      {
        if (!m_binDecoder.decodeBin(Ctx::AffMergeIdx((unaryIdx > 2 ? 2 : unaryIdx))))
        {
          break;
        }
      }
      cu.mergeIdx = unaryIdx;
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "aff_merge_idx() aff_merge_idx=%d\n", cu.mergeIdx);
  }
  else
  {
    int numCandminus1 = int(cu.cs->sps->m_maxNumMergeCand) - 1;
    cu.mergeIdx       = 0;

    if (cu.geoFlag)
    {
      RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__GEO_INDEX);
      geo_adaptive_blending_idx(cu);
      bool isIntra0          = false;
      bool isIntra1          = false;
      bool bUseOnlyOneVector = cu.slice->isInterP() || cu.slice->m_sps->m_maxNumGeoCand == 1;

      cu.geoMMVDFlag[0] = m_binDecoder.decodeBin(Ctx::GeoMmvdFlag());
      DTRACE(g_trace_ctx, D_SYNTAX, "geo_mmvd_flag() L%d : %d\n", 0, cu.geoMMVDFlag[0]);
      if (cu.geoMMVDFlag[0])
      {
        geo_mmvd_idx(cu, RPL0);
      }
      else
      {
        isIntra0 = m_binDecoder.decodeBin(Ctx::GPMIntraFlag()) ? true : false;
        DTRACE(g_trace_ctx, D_SYNTAX, "geo_intra_flag() L%d : %d\n", 0, isIntra0);
      }

      if (!bUseOnlyOneVector || isIntra0)
      {
        cu.geoMMVDFlag[1] = m_binDecoder.decodeBin(Ctx::GeoMmvdFlag());
        DTRACE(g_trace_ctx, D_SYNTAX, "geo_mmvd_flag() L%d : %d\n", 1, cu.geoMMVDFlag[1]);
        if (cu.geoMMVDFlag[1])
        {
          geo_mmvd_idx(cu, RPL1);
        }
        else if (!isIntra0)
        {
          isIntra1 = m_binDecoder.decodeBin(Ctx::GPMIntraFlag()) ? true : false;
          DTRACE(g_trace_ctx, D_SYNTAX, "geo_intra_flag() L%d : %d\n", 1, isIntra1);
        }
      }
      else
      {
        isIntra1 = true;
      }
      cu.gpmIntraFlag = (isIntra0 || isIntra1);

      if (isIntra0 || isIntra1)
      {
        geo_merge_idx1(cu, isIntra0, isIntra1);
      }
      else if (!cu.geoMMVDFlag[0] && !cu.geoMMVDFlag[1])
      {
        geo_merge_idx(cu);
      }
      else if (cu.geoMMVDFlag[0] && cu.geoMMVDFlag[1] && cu.geoMMVDIdx[0] == cu.geoMMVDIdx[1])
      {
        geo_merge_idx(cu);
      }
      else
      {
        geo_merge_idx1(cu, isIntra0, isIntra1);
      }

      return;
    }

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

    if (numCandminus1 > 0)
    {
      unsigned int uiUnaryIdx = 0;
      for (; uiUnaryIdx < numCandminus1; ++uiUnaryIdx)
      {
        if (!m_binDecoder.decodeBin(
              Ctx::MergeIdx((uiUnaryIdx > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : uiUnaryIdx))))
        {
          break;
        }
      }
      cu.mergeIdx = uiUnaryIdx;
    }

    DTRACE(g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", cu.mergeIdx);
  }
}

void CABACReader::geo_mmvd_idx(CodingUnit &cu, RefPicList eRefPicList)
{
  bool extMMVD           = cu.cs->picHeader->m_gpmMMVDTableFlag;
  int  numCandminus1Step = (extMMVD ? GPM_EXT_MMVD_REFINE_STEP : GPM_MMVD_REFINE_STEP) - 1;
  int  step              = 0;
  if (m_binDecoder.decodeBin(Ctx::GeoMmvdStepMvpIdx()))
  {
    step++;
    for (; step < numCandminus1Step; step++)
    {
      if (!m_binDecoder.decodeBinEP())
      {
        break;
      }
    }
  }

  const static int idxToMMVDStep[GPM_EXT_MMVD_REFINE_STEP] = { 1, 2, 3, 4, 5, 0, 6, 7, 8 };
  step                                                     = idxToMMVDStep[step];

  int maxMMVDDir             = (extMMVD ? GPM_EXT_MMVD_REFINE_DIRECTION : GPM_MMVD_REFINE_DIRECTION);
  int direction              = m_binDecoder.decodeBinsEP(maxMMVDDir > 4 ? 3 : 2);
  cu.geoMMVDIdx[eRefPicList] = (step * maxMMVDDir + direction);
  DTRACE(g_trace_ctx, D_SYNTAX, "geo_mmvd_idx() L%d: %d\n", eRefPicList, cu.geoMMVDIdx[eRefPicList]);
}

void CABACReader::geo_merge_idx(CodingUnit &cu)
{
  uint32_t splitDir = 0;
  xReadTruncBinCode(splitDir, GEO_NUM_PARTITION_MODE);
  cu.geoSplitDir = splitDir;
  DTRACE(g_trace_ctx, D_SYNTAX, "geo_split_dir %d\n", cu.geoSplitDir);
  const int maxNumGeoCand = cu.cs->sps->m_maxNumGeoCand;
  CHECK(maxNumGeoCand < 2, "Incorrect max number of geo candidates");
  CHECK(cu.lheight() > 64 || cu.lwidth() > 64, "Incorrect block size of geo flag");
  int numCandminus2 = maxNumGeoCand - 2;
  cu.mergeIdx       = 0;
  int mergeCand0    = 0;
  int mergeCand1    = 0;
  if (m_binDecoder.decodeBin(Ctx::MergeIdx()))
  {
    mergeCand0 += unary_max_eqprob(numCandminus2) + 1;
  }
  if (numCandminus2 > 0)
  {
    if (m_binDecoder.decodeBin(Ctx::MergeIdx()))
    {
      mergeCand1 += unary_max_eqprob(numCandminus2 - 1) + 1;
    }
  }
  mergeCand1 += mergeCand1 >= mergeCand0 ? 1 : 0;
  cu.geoMergeIdx[0] = mergeCand0;
  cu.geoMergeIdx[1] = mergeCand1;
  DTRACE(g_trace_ctx, D_SYNTAX, "geo_merge_idx() (%d, %d)\n", cu.geoMergeIdx[0], cu.geoMergeIdx[1]);
}

void CABACReader::geo_merge_idx1(CodingUnit &cu, bool isIntra0, bool isIntra1)
{
  uint32_t splitDir = 0;
  xReadTruncBinCode(splitDir, GEO_NUM_PARTITION_MODE);
  cu.geoSplitDir = splitDir;
  DTRACE(g_trace_ctx, D_SYNTAX, "geo_split_dir %d\n", cu.geoSplitDir);
  const int maxNumGeoCand = cu.cs->sps->m_maxNumGeoCand;
  CHECK(maxNumGeoCand < 1, "Incorrect max number of geo candidates");
  CHECK(cu.lheight() > 64 || cu.lwidth() > 64, "Incorrect block size of geo flag");
  int numCandminus2 = maxNumGeoCand - 2;
  cu.mergeIdx       = 0;
  int mergeCand0    = 0;
  int mergeCand1    = 0;
  if (isIntra0)
  {
    mergeCand0 = GEO_MAX_NUM_UNI_CANDS + unary_max_eqprob(GEO_MAX_NUM_INTRA_CANDS - 1);
  }
  else if (numCandminus2 >= 0)
  {
    if (m_binDecoder.decodeBin(Ctx::MergeIdx()))
    {
      mergeCand0 += unary_max_eqprob(numCandminus2) + 1;
    }
  }

  if (isIntra1)
  {
    mergeCand1 = GEO_MAX_NUM_UNI_CANDS + unary_max_eqprob(GEO_MAX_NUM_INTRA_CANDS - 1);
  }
  else if (numCandminus2 >= 0)
  {
    if (m_binDecoder.decodeBin(Ctx::MergeIdx()))
    {
      mergeCand1 += unary_max_eqprob(numCandminus2) + 1;
    }
  }

  cu.geoMergeIdx[0] = mergeCand0;
  cu.geoMergeIdx[1] = mergeCand1;
  DTRACE(g_trace_ctx, D_SYNTAX, "geo_merge_idx1() (%d, %d)\n", cu.geoMergeIdx[0], cu.geoMergeIdx[1]);
}

void CABACReader::geo_adaptive_blending_idx(CodingUnit &cu)
{
  int bin0 = m_binDecoder.decodeBin(Ctx::GeoBldFlag(0));
  if (bin0 == 1)
  {
    cu.geoBldIdx = 2;   // 1
  }
  else
  {
    int bin1 = m_binDecoder.decodeBin(Ctx::GeoBldFlag(1));
    if (bin1 == 0)
    {
      int bin2 = m_binDecoder.decodeBin(Ctx::GeoBldFlag(3));
      if (bin2 == 0)
      {
        cu.geoBldIdx = 4;   // 000
      }
      else
      {
        cu.geoBldIdx = 3;   // 001
      }
    }
    else
    {
      int bin2 = m_binDecoder.decodeBin(Ctx::GeoBldFlag(2));
      if (bin2 == 0)
      {
        cu.geoBldIdx = 1;   // 010
      }
      else
      {
        cu.geoBldIdx = 0;   // 011
      }
    }
  }
}

void CABACReader::mmvd_merge_idx(CodingUnit &cu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__MERGE_INDEX);

  int mvdBaseIdx = 0;
  if (cu.cs->sps->m_maxNumMergeCand > 1)
  {
    static_assert(MmvdIdx::BASE_MV_NUM == 2, "");
    mvdBaseIdx = m_binDecoder.decodeBin(Ctx::MmvdMergeIdx());
  }
  // DTRACE(g_trace_ctx, D_SYNTAX, "base_mvp_idx() base_mvp_idx=%d\n", mvdBaseIdx);
  int numStepCandMinus1 = MmvdIdx::REFINE_STEP - 1;
  int mvdStep           = 0;
  if (m_binDecoder.decodeBin(Ctx::MmvdStepMvpIdx()))
  {
    mvdStep++;
    for (; mvdStep < numStepCandMinus1; mvdStep++)
    {
      if (!m_binDecoder.decodeBinEP())
      {
        break;
      }
    }
  }
  // DTRACE(g_trace_ctx, D_SYNTAX, "MmvdStepMvpIdx() MmvdStepMvpIdx=%d\n", mvdStep);
  const int mvdPosition        = m_binDecoder.decodeBinsEP(2);
  // DTRACE(g_trace_ctx, D_SYNTAX, "pos() pos=%d\n", mvdPosition);
  cu.mmvdMergeIdx.pos.position = mvdPosition;
  cu.mmvdMergeIdx.pos.step     = mvdStep;
  cu.mmvdMergeIdx.pos.baseIdx  = mvdBaseIdx;
  DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_idx() base=%d pos=%d step=%d\n", cu.mmvdMergeIdx.pos.baseIdx,
         cu.mmvdMergeIdx.pos.position, cu.mmvdMergeIdx.pos.step);
}

void CABACReader::inter_pred_idc(CodingUnit &cu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__INTER_DIR);

  if (cu.cs->slice->isInterP())
  {
    cu.interDir = 1;
    return;
  }
  if (!(PU::isBipredRestriction(cu)))
  {
    unsigned ctxId = DeriveCtx::CtxInterDir(cu);
    if (m_binDecoder.decodeBin(Ctx::InterDir(ctxId)))
    {
      DTRACE(g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=%d value=%d pos=(%d,%d)\n", ctxId, 3, cu.lumaPos().x,
             cu.lumaPos().y);
      cu.interDir = 3;
      return;
    }
  }
  if (m_binDecoder.decodeBin(Ctx::InterDir(7)))
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=7 value=%d pos=(%d,%d)\n", 2, cu.lumaPos().x, cu.lumaPos().y);
    cu.interDir = 2;
    return;
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=7 value=%d pos=(%d,%d)\n", 1, cu.lumaPos().x, cu.lumaPos().y);
  cu.interDir = 1;
  return;
}

void CABACReader::ref_idx(CodingUnit &cu, RefPicList eRefList)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__REF_FRM_IDX);

  if (cu.smvdMode)
  {
    cu.refIdx[eRefList] = cu.cs->slice->getSymRefIdx(eRefList);
    return;
  }

  int numRef = cu.cs->slice->m_numRefIdx[eRefList];

  if (numRef <= 1 || !m_binDecoder.decodeBin(Ctx::RefPic()))
  {
    if (numRef > 1)
    {
      DTRACE(g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", 0, cu.lumaPos().x, cu.lumaPos().y);
    }
    cu.refIdx[eRefList] = 0;
    return;
  }
  if (numRef <= 2 || !m_binDecoder.decodeBin(Ctx::RefPic(1)))
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", 1, cu.lumaPos().x, cu.lumaPos().y);
    cu.refIdx[eRefList] = 1;
    return;
  }
  for (int idx = 3;; idx++)
  {
    if (numRef <= idx || !m_binDecoder.decodeBinEP())
    {
      cu.refIdx[eRefList] = (int8_t)(idx - 1);
      DTRACE(g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", idx - 1, cu.lumaPos().x, cu.lumaPos().y);
      return;
    }
  }
}

void CABACReader::mvp_flag(CodingUnit &cu, RefPicList eRefList)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__MVP_IDX);

  unsigned mvpIdx = m_binDecoder.decodeBin(Ctx::MVPIdx());
  DTRACE(g_trace_ctx, D_SYNTAX, "mvp_flag() value=%d pos=(%d,%d)\n", mvpIdx, cu.lumaPos().x, cu.lumaPos().y);
  cu.mvpIdx[eRefList] = mvpIdx;
  DTRACE(g_trace_ctx, D_SYNTAX, "mvpIdx(refList:%d)=%d\n", eRefList, mvpIdx);
}

void CABACReader::ciip_flag(CodingUnit &cu, bool usageIsInferred)
{
  if (!cu.cs->sps->m_useCiip || cu.skip)
  {
    cu.ciipFlag = false;
    DTRACE(g_trace_ctx, D_SYNTAX, "ciip_flag() ciip=0\n");
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__MH_INTRA_FLAG);

  cu.ciipFlag = false;
  if (usageIsInferred)
  {
    cu.ciipFlag = true;
  }
  else
  {
    cu.ciipFlag = (bool)m_binDecoder.decodeBin(Ctx::CiipFlag(0));
  }
  if (cu.ciipFlag)
  {
    cu.ciipMode = (CIIP_Type)m_binDecoder.decodeBin(Ctx::CiipFlag(1));
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "ciip_flag() ciip=%d\n", cu.ciipFlag ? 1 : 0);
}

//================================================================================
//  clause 7.3.8.8
//--------------------------------------------------------------------------------
//    void  transform_tree      ( cs, area, cuCtx, chromaCbfs )
//    bool  split_transform_flag( depth )
//    bool  cbf_comp            ( area, depth )
//================================================================================

void CABACReader::transform_tree(CodingStructure &cs, Partitioner &partitioner, CUCtx &cuCtx)
{
  const UnitArea &area = partitioner.currArea();
  CodingUnit     &cu   = *cs.getCU(area.block(partitioner.chType), partitioner.chType);

  // split_transform_flag
  bool           split   = partitioner.canSplit(TU_MAX_TR_SPLIT, cs);
  const unsigned trDepth = partitioner.currTrDepth;

  if (cu.sbtInfo && partitioner.canSplit(PartSplit(cu.getSbtTuSplit()), cs))
  {
    split = true;
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
      THROW("Implicit TU split not available!");
    }

    do
    {
      transform_tree(cs, partitioner, cuCtx);
    } while (partitioner.nextPart(cs));

    partitioner.exitCurrSplit();
  }
  else
  {
    TransformUnit &tu        = cs.addTU(CS::getArea(cs, area, partitioner.chType), partitioner.chType);
    unsigned       numBlocks = ::getNumberValidTBlocks(*cs.pcv);
    tu.checkTuNoResidual(partitioner.currPartIdx());

    for (unsigned compID = COMP_Y; compID < numBlocks; compID++)
    {
      if (tu.blocks[compID].valid())
      {
        tu.getCoeffs(CompID(compID)).fill(0);
      }
    }
    tu.depth = trDepth;

    transform_unit_last(tu, cuCtx, partitioner);
    nst_idx(tu, cuCtx);
    transform_unit_coef(tu, cuCtx, partitioner);
    mts_idx(tu, cuCtx);
    transform_unit_sign(tu, partitioner);
  }
}

bool CABACReader::cbf_comp(const CompArea &area, unsigned depth, const bool prevCbf, const BdpcmMode bdpcmMode)
{
  unsigned      ctxId  = DeriveCtx::CtxQtCbf(area.compID, prevCbf);
  const CtxSet &ctxSet = Ctx::QtCbf[area.compID];

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__QT_CBF, area.size(), area.compID);

  if (bdpcmMode != BdpcmMode::NONE)
  {
    ctxId = area.compID == COMP_Cr ? 2 : 1;
  }

  const bool cbf = m_binDecoder.decodeBin(ctxSet(ctxId)) != 0;

  DTRACE(g_trace_ctx, D_SYNTAX, "cbf_comp() etype=%d pos=(%d,%d) ctx=%d cbf=%d\n", area.compID, area.x, area.y, ctxId,
         cbf ? 1 : 0);

  return cbf;
}

//================================================================================
//  clause 7.3.8.9
//--------------------------------------------------------------------------------
//    void  mvd_coding( cu, refList )
//================================================================================

void CABACReader::mvd_coding(Mv &rMvd)
{
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatisticsClassType ctype_mvd(STATS__CABAC_BITS__MVD);
  CodingStatisticsClassType ctype_mvd_ep(STATS__CABAC_BITS__MVD_EP);
#endif

  RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_mvd);

  // abs_mvd_greater0_flag[ 0 | 1 ]
  int horAbs = (int)m_binDecoder.decodeBin(Ctx::Mvd());
  int verAbs = (int)m_binDecoder.decodeBin(Ctx::Mvd());

  // abs_mvd_greater1_flag[ 0 | 1 ]
  if (horAbs)
  {
    horAbs += (int)m_binDecoder.decodeBin(Ctx::Mvd(1));
  }
  if (verAbs)
  {
    verAbs += (int)m_binDecoder.decodeBin(Ctx::Mvd(1));
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_mvd_ep);

  // abs_mvd_minus2[ 0 | 1 ] and mvd_sign_flag[ 0 | 1 ]
  if (horAbs)
  {
    if (horAbs > 1)
    {
      horAbs += m_binDecoder.decodeRemAbsEP(1, 0, MV_BITS - 1);
    }
    if (m_binDecoder.decodeBinEP())
    {
      horAbs = -horAbs;
    }
  }
  if (verAbs)
  {
    if (verAbs > 1)
    {
      verAbs += m_binDecoder.decodeRemAbsEP(1, 0, MV_BITS - 1);
    }
    if (m_binDecoder.decodeBinEP())
    {
      verAbs = -verAbs;
    }
  }
  rMvd = Mv(horAbs, verAbs);
  CHECK(!((horAbs >= MVD_MIN) && (horAbs <= MVD_MAX)) || !((verAbs >= MVD_MIN) && (verAbs <= MVD_MAX)),
        "Illegal MVD value");
}

unsigned CABACReader::xReadBvdContext(unsigned ctxT, int offset, int param)
{
  unsigned symbol = 0;
  unsigned bit    = 1;
  unsigned uiIdx  = 0;
  while (bit)
  {
    if (uiIdx >= ctxT)
    {
      bit = m_binDecoder.decodeBinEP();
    }
    else
    {
      bit = m_binDecoder.decodeBin(Ctx::Bvd(offset + uiIdx + 1));
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "uiIdx: %d, bit: %d\n", uiIdx, bit);
    uiIdx++;
    symbol += bit << param++;
  }
  if (--param)
  {
    bit = m_binDecoder.decodeBinsEP(param);
    symbol += bit;
  }
  return symbol;
}

void CABACReader::bvdCoding(Mv &rMvd)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__BVD);

  int horAbs = (int)m_binDecoder.decodeBin(Ctx::Bvd(HOR_BVD_CTX_OFFSET));
  int verAbs = (int)m_binDecoder.decodeBin(Ctx::Bvd(VER_BVD_CTX_OFFSET));

  if (horAbs)
  {
    horAbs += xReadBvdContext(NUM_HOR_BVD_CTX, HOR_BVD_CTX_OFFSET, BVD_CODING_GOLOMB_ORDER);
    if (m_binDecoder.decodeBinEP())
    {
      horAbs = -horAbs;
    }
  }
  if (verAbs)
  {
    verAbs += xReadBvdContext(NUM_VER_BVD_CTX, VER_BVD_CTX_OFFSET, BVD_CODING_GOLOMB_ORDER);
    if (m_binDecoder.decodeBinEP())
    {
      verAbs = -verAbs;
    }
  }
  rMvd = Mv(horAbs, verAbs);

  CHECK(!((horAbs >= MVD_MIN) && (horAbs <= MVD_MAX)) || !((verAbs >= MVD_MIN) && (verAbs <= MVD_MAX)),
        "Illegal BVD value");
}

//================================================================================
//  clause 7.3.8.10
//--------------------------------------------------------------------------------
//    void  transform_unit      ( tu, cuCtx, chromaCbfs )
//    void  cu_qp_delta         ( cu )
//    void  cu_chroma_qp_offset ( cu )
//================================================================================
void CABACReader::transform_unit_last(TransformUnit &tu, CUCtx &cuCtx, Partitioner &partitioner)
{
  DTRACE(g_trace_ctx, D_SYNTAX, "transform_unit_last() pos=(%d,%d) size=%dx%d depth=%d trDepth=%d\n",
         tu.block(tu.chType).x, tu.block(tu.chType).y, tu.block(tu.chType).width, tu.block(tu.chType).height,
         tu.cu->depth, partitioner.currTrDepth);

  const UnitArea &area    = partitioner.currArea();
  const unsigned  trDepth = partitioner.currTrDepth;

  CodingUnit &cu = *tu.cu;
  ChromaCbfs  chromaCbfs;
  chromaCbfs.Cb = chromaCbfs.Cr = false;

  // cbf_cb & cbf_cr
  if (isChromaEnabled(area.chromaFormat) && area.blocks[COMP_Cb].valid() &&
      (!CS::isDualITree(*cu.cs) || partitioner.chType == ChannelType::CHROMA))
  {
    if (!(cu.sbtInfo && tu.noResidual))
    {
      chromaCbfs.Cb = cbf_comp(area.blocks[COMP_Cb], trDepth, false, cu.bdpcmMode[1]);
    }

    if (!(cu.sbtInfo && tu.noResidual))
    {
      chromaCbfs.Cr = cbf_comp(area.blocks[COMP_Cr], trDepth, chromaCbfs.Cb, cu.bdpcmMode[1]);
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
      TU::setCbfAtDepth(tu, COMP_Y, trDepth, 1);
    }
    else if (cu.sbtInfo && tu.noResidual)
    {
      TU::setCbfAtDepth(tu, COMP_Y, trDepth, 0);
    }
    else if (cu.sbtInfo && !chromaCbfs.sigChroma(area.chromaFormat))
    {
      assert(!tu.noResidual);
      TU::setCbfAtDepth(tu, COMP_Y, trDepth, 1);
    }
    else
    {
      bool cbfY = cbf_comp(tu.Y(), trDepth, false, cu.bdpcmMode[0]);
      TU::setCbfAtDepth(tu, COMP_Y, trDepth, (cbfY ? 1 : 0));
    }
  }
  if (isChromaEnabled(area.chromaFormat))
  {
    TU::setCbfAtDepth(tu, COMP_Cb, trDepth, (chromaCbfs.Cb ? 1 : 0));
    TU::setCbfAtDepth(tu, COMP_Cr, trDepth, (chromaCbfs.Cr ? 1 : 0));
  }
  bool lumaOnly  = !isChromaEnabled(cu.chromaFormat) || !tu.blocks[COMP_Cb].valid();
  bool cbfLuma   = (tu.cbf[COMP_Y] != 0);
  bool cbfChroma = (lumaOnly ? false : (chromaCbfs.Cb || chromaCbfs.Cr));

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
  if (!CS::isDualITree(*cu.cs) || isChroma(tu.chType))   // !DUAL_TREE_LUMA
  {
    SizeType channelWidth  = !CS::isDualITree(*cu.cs) ? cu.lwidth() : cu.chromaSize().width;
    SizeType channelHeight = !CS::isDualITree(*cu.cs) ? cu.lheight() : cu.chromaSize().height;

    if (cu.cs->slice->m_chromaQpAdjEnabled &&
        (channelWidth > MAX_TB_SIZEY || channelHeight > MAX_TB_SIZEY || cbfChroma) && !cuCtx.isChromaQpAdjCoded)
    {
      cu_chroma_qp_offset(cu);
      cuCtx.isChromaQpAdjCoded = true;
    }
  }

  if (!lumaOnly)
  {
    joint_cb_cr(tu, (tu.cbf[COMP_Cb] ? 2 : 0) + (tu.cbf[COMP_Cr] ? 1 : 0));
  }

  if (cbfLuma)
  {
    residual_coding_last(tu, COMP_Y, cuCtx);
  }
  if (!lumaOnly)
  {
    for (CompID compID = COMP_Cb; compID <= COMP_Cr; compID = CompID(compID + 1))
    {
      if (tu.cbf[compID])
      {
        residual_coding_last(tu, compID, cuCtx);
      }
    }
  }
}

void CABACReader::transform_unit_coef(TransformUnit &tu, CUCtx &cuCtx, Partitioner &partitioner)
{
  DTRACE(g_trace_ctx, D_SYNTAX, "transform_unit_coef() pos=(%d,%d) size=%dx%d depth=%d trDepth=%d\n",
         tu.block(tu.chType).x, tu.block(tu.chType).y, tu.block(tu.chType).width, tu.block(tu.chType).height,
         tu.cu->depth, partitioner.currTrDepth);

  if (tu.cbf[COMP_Y])
  {
    residual_coding_coef(tu, COMP_Y, cuCtx);
  }
  if (!(tu.cu->chromaFormat == ChromaFormat::_400 || !tu.blocks[COMP_Cb].valid()))
  {
    for (CompID compID = COMP_Cb; compID <= COMP_Cr; compID = CompID(compID + 1))
    {
      if (tu.cbf[compID])
      {
        residual_coding_coef(tu, compID, cuCtx);
      }
    }
  }
}

void CABACReader::transform_unit_sign(TransformUnit &tu, Partitioner &partitioner)
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

void CABACReader::cu_qp_delta(CodingUnit &cu, int predQP, int8_t &qp)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__DELTA_QP_EP);

  CHECK(predQP == std::numeric_limits<int>::max(), "Invalid predicted QP");
  int qpY = predQP;
  int DQp = unary_max_symbol(Ctx::DeltaQP(), Ctx::DeltaQP(1), CU_DQP_TU_CMAX);
  if (DQp >= CU_DQP_TU_CMAX)
  {
    DQp += exp_golomb_eqprob(CU_DQP_EG_k);
  }
  if (DQp > 0)
  {
    if (m_binDecoder.decodeBinEP())
    {
      DQp = -DQp;
    }
    int qpBdOffsetY = cu.cs->sps->m_qpBDOffset[ChannelType::LUMA];
    qpY             = ((predQP + DQp + (MAX_QP + 1) + 2 * qpBdOffsetY) % ((MAX_QP + 1) + qpBdOffsetY)) - qpBdOffsetY;
  }
  qp = (int8_t)qpY;

  DTRACE(g_trace_ctx, D_DQP, "x=%d, y=%d, d=%d, pred_qp=%d, DQp=%d, qp=%d\n", cu.block(cu.chType).lumaPos().x,
         cu.block(cu.chType).lumaPos().y, cu.qtDepth, predQP, DQp, qp);
}

void CABACReader::cu_chroma_qp_offset(CodingUnit &cu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__CHROMA_QP_ADJUSTMENT,
                                                      cu.block(cu.chType).lumaSize(), ChannelType::CHROMA);

  // cu_chroma_qp_offset_flag
  int      length = cu.cs->pps->m_chromaQpOffsetListLen;
  unsigned qpAdj  = m_binDecoder.decodeBin(Ctx::ChromaQpAdjFlag());
  if (qpAdj && length > 1)
  {
    // cu_chroma_qp_offset_idx
    qpAdj += unary_max_symbol(Ctx::ChromaQpAdjIdc(), Ctx::ChromaQpAdjIdc(), length - 1);
  }
  /* NB, symbol = 0 if outer flag is not set,
   *              1 if outer flag is set and there is no inner flag
   *              1+ otherwise */
  cu.chromaQpAdj = cu.cs->chromaQpAdj = qpAdj;
}

//================================================================================
//  clause 7.3.8.11
//--------------------------------------------------------------------------------
//    void        residual_coding         ( tu, compID )
//    bool        transform_skip_flag     ( tu, compID )
//    int         last_sig_coeff          ( coeffCtx )
//    void        residual_coding_subblock( coeffCtx )
//================================================================================

void CABACReader::joint_cb_cr(TransformUnit &tu, const int cbfMask)
{
  if (!tu.cu->slice->m_sps->m_jointCbCrEnabledFlag)
  {
    return;
  }

  if ((CU::isIntra(*tu.cu) && cbfMask != 0) || cbfMask == CBF_MASK_CBCR)
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__JOINT_CB_CR, tu.blocks[COMP_Cr].lumaSize(),
                                                        ChannelType::CHROMA);
    tu.jointCbCr = (m_binDecoder.decodeBin(Ctx::JointCbCrFlag(cbfMask - 1)) ? cbfMask : 0);
  }
}

void CABACReader::residual_coding_last(TransformUnit &tu, CompID compID, CUCtx &cuCtx)
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
  CoeffCodingContext cctx(tu, compID, signHiding, cu.getBdpcmMode(compID));

  // parse last coeff position
  cctx.setScanPosLast(last_sig_coeff(cctx, tu, compID));
  tu.lastPos[compID] = cctx.scanPosLast();

  if (tu.mtsIdx[compID] != MtsType::SKIP && tu.blocks[compID].height >= 4 && tu.blocks[compID].width >= 4)
  {
    const uint32_t width     = tu.blocks[compID].width;
    const uint32_t height    = tu.blocks[compID].height;
    const bool     allowNSPT = TU::isNSPTAllowed(width, height);
    const int      maxLfnstPos =
      (allowNSPT ? PU::getNSPTMatrixDim(width, height) : PU::getLFNSTMatrixDim(width, height)) - 1;
    cuCtx.violatesLfnstConstrained[toChannelType(compID)] |= cctx.scanPosLast() > maxLfnstPos;
  }
  if (tu.mtsIdx[compID] != MtsType::SKIP && tu.blocks[compID].height >= 4 && tu.blocks[compID].width >= 4)
  {
    const int lfnstLastScanPosTh = isLuma(compID) ? LFNST_LAST_SIG_LUMA : LFNST_LAST_SIG_CHROMA;
    cuCtx.lfnstLastScanPos |= cctx.scanPosLast() >= lfnstLastScanPosTh;
  }
  if (isLuma(compID) && tu.mtsIdx[compID] != MtsType::SKIP)
  {
    cuCtx.mtsLastScanPos |= cctx.scanPosLast() >= 1;
  }
}

void CABACReader::residual_coding_coef(TransformUnit &tu, CompID compID, CUCtx &cuCtx)
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
  CoeffCodingContext cctx(tu, compID, signHiding, cu.getBdpcmMode(compID));
  TCoeff            *coeff = tu.getCoeffs(compID).buf;

  // set last coeff position
  cctx.setScanPosLast(tu.lastPos[compID]);

  // parse subblocks
  const uint64_t stateTransTab = g_stateTransTab[tu.cs->slice->m_depQuantEnabledIdc];
  int            state         = 0;

  int ctxBinSampleRatio =
    (compID == COMP_Y) ? MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_LUMA : MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_CHROMA;
  cctx.remRegBins = (tu.getTbAreaAfterCoefZeroOut(compID) * ctxBinSampleRatio) >> 4;

  int baseLevel = m_binDecoder.getCtx().getBaseLevel();
  cctx.setBaseLevel(baseLevel);
  if (tu.cs->slice->m_sps->m_spsRangeExtension.m_persistentRiceAdaptationEnabledFlag)
  {
    cctx.setUpdateHist(1);
    unsigned riceStats    = m_binDecoder.getCtx().getGRAdaptStats((unsigned)compID);
    TCoeff   historyValue = (TCoeff)1 << riceStats;
    cctx.setHistValue(historyValue);
  }
  for (int subSetId = (cctx.scanPosLast() >> cctx.log2CGSize()); subSetId >= 0; subSetId--)
  {
    cctx.initSubblock(subSetId);

    residual_coding_subblock(cctx, coeff, stateTransTab, state);
  }
  tu.numPredAreaSigns[compID] = cctx.numSignsPredArea();
  if (isLuma(compID) && tu.mtsIdx[compID] != MtsType::SKIP)
  {
    cuCtx.mtsCoeffAbsSum = tu.getCoeffs(compID).computeAbsSum();
  }
}

void CABACReader::residual_coding_sign(TransformUnit &tu, CompID compID)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__SIGN_BIT, tu.cu->block(compID).lumaSize(),
                                                      compID);
  const CtxSet  &ctx              = Ctx::SignPredFlag[to_underlying(toChannelType(compID))];
  const uint16_t ctxIdxGt1        = ctx(CU::isIntra(*tu.cu) ? 1 : 3);
  const uint16_t ctxIdxEq1        = ctx(CU::isIntra(*tu.cu) ? 0 : 2);
  const uint16_t numSignsPredArea = tu.numPredAreaSigns[compID] & ((1 << 16) - 1);
  const int32_t  maxNumPredSigns  = tu.maxNumPredSigns(compID);
  uint8_t       *signsPredArea    = tu.getSignsPredArea(compID);
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
    signsPredArea[k] = m_binDecoder.decodeBin(ctxIdxGt1);
  }
  for (int k = numPredSignsGt1; k < numPredSigns; k++)
  {
    signsPredArea[k] = m_binDecoder.decodeBin(ctxIdxEq1);
  }
  for (int k = numPredSigns; k < numSignsPredArea; k++)
  {
    signsPredArea[k] = m_binDecoder.decodeBinEP();
  }
}

void CABACReader::ts_flag(TransformUnit &tu, CompID compID)
{
  CodingUnit &cu     = *tu.cu;
  int         tsFlag = cu.getBdpcmMode(compID) != BdpcmMode::NONE || tu.mtsIdx[compID] == MtsType::SKIP ? 1 : 0;
  int         ctxIdx = isLuma(compID) ? 0 : 1;

  if (TU::isTSAllowed(tu, compID) && tu.cu->getBdpcmMode(compID) == BdpcmMode::NONE)
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__MTS_FLAGS, tu.blocks[compID], compID);
    tsFlag = m_binDecoder.decodeBin(Ctx::TransformSkipFlag(ctxIdx)) != 0;
  }

  tu.mtsIdx[compID] = tsFlag || cu.getBdpcmMode(compID) != BdpcmMode::NONE ? MtsType::SKIP : MtsType::DCT2_DCT2;

  DTRACE(g_trace_ctx, D_SYNTAX, "ts_flag() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMP_Y, cu.lx(), cu.ly(), tsFlag);
}

void CABACReader::nst_idx(TransformUnit &tu, CUCtx &cuCtx)
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

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__LFNST);
  uint32_t nstIdx = 0;
  if (CU::isInter(cu))
  {
    nstIdx = m_binDecoder.decodeBin(Ctx::LFNSTIdxInter(0));
    if (nstIdx > 0)
    {
      nstIdx += m_binDecoder.decodeBin(Ctx::LFNSTIdxInter(1));
    }
    if (nstIdx > 1)
    {
      nstIdx += m_binDecoder.decodeBin(Ctx::LFNSTIdxInter(2));
    }
  }
  else
  {
    uint32_t ctxIdx    = CS::isDualITree(*cu.cs) ? 1 : 0;
    uint32_t firstBin  = m_binDecoder.decodeBin(Ctx::LFNSTIdxIntra(ctxIdx));
    uint32_t secondBin = m_binDecoder.decodeBin(Ctx::LFNSTIdxIntra(2 + firstBin));
    nstIdx             = firstBin + (secondBin << 1);
  }
  if (ch == ChannelType::LUMA)
  {
    if (nstIdx == 1)
    {
      tu.mtsIdx[COMP_Y] = MtsType::DCT2_NST1;
    }
    else if (nstIdx == 2)
    {
      tu.mtsIdx[COMP_Y] = MtsType::DCT2_NST2;
    }
    else if (nstIdx == 3)
    {
      tu.mtsIdx[COMP_Y] = MtsType::DCT2_NST3;
    }
  }
  else
  {
    if (nstIdx == 1)
    {
      tu.mtsIdx[COMP_Cr] = tu.mtsIdx[COMP_Cb] = MtsType::DCT2_NST1;
    }
    else if (nstIdx == 2)
    {
      tu.mtsIdx[COMP_Cr] = tu.mtsIdx[COMP_Cb] = MtsType::DCT2_NST2;
    }
    else if (nstIdx == 3)
    {
      tu.mtsIdx[COMP_Cr] = tu.mtsIdx[COMP_Cb] = MtsType::DCT2_NST3;
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "nst_idx() etype=%d pos=(%d,%d) nstIdx=%d\n", compId, cu.lx(), cu.ly(),
         TU::getNstIdx(tu, compId));
}

void CABACReader::mts_idx(TransformUnit &tu, CUCtx &cuCtx)
{
  CodingUnit &cu     = *tu.cu;
  MtsType     mtsIdx = tu.mtsIdx[COMP_Y];

  if (TU::isMTSAllowed(tu, COMP_Y) && !cuCtx.violatesMtsCoeffConstraint && cuCtx.mtsLastScanPos &&
      mtsIdx != MtsType::SKIP && TU::getNstIdx(tu, COMP_Y) == 0 && tu.cbf[COMP_Y] != 0)
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__MTS_FLAGS, tu.blocks[COMP_Y], COMP_Y);
    int ctxIdx = 0;
    if (CU::isIntra(cu))
    {
      ctxIdx = (cuCtx.mtsCoeffAbsSum > MTS_TH_COEFF[1]) ? 2 : (cuCtx.mtsCoeffAbsSum > MTS_TH_COEFF[0]) ? 1 : 0;
    }
    int symbol = m_binDecoder.decodeBin(Ctx::MTSIdx(ctxIdx));

    if (symbol)
    {
      int      nCands = CU::isIntra(cu) ? MTS_NCANDS[ctxIdx] : 4;
      uint32_t val;
      if (nCands > 1)
      {
        xReadTruncBinCode(val, nCands);
      }
      else
      {
        val = 0;
      }
      mtsIdx = MtsType::MTS_1 + val;
    }
  }

  tu.mtsIdx[COMP_Y] = mtsIdx;
  DTRACE(g_trace_ctx, D_SYNTAX, "mts_idx() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMP_Y, tu.cu->lx(), tu.cu->ly(), mtsIdx);
}

int CABACReader::last_sig_coeff(CoeffCodingContext &cctx, TransformUnit &tu, CompID compID)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__LAST_SIG_X_Y,
                                                      Size(cctx.width(), cctx.height()), cctx.compID());

  unsigned posLastX = 0, posLastY = 0;
  unsigned maxLastPosX = cctx.maxLastPosX();
  unsigned maxLastPosY = cctx.maxLastPosY();
  unsigned zoTbWdith   = getNonzeroTuSize(cctx.width());
  unsigned zoTbHeight  = getNonzeroTuSize(cctx.height());

  for (; posLastX < maxLastPosX; posLastX++)
  {
    if (!m_binDecoder.decodeBin(cctx.lastXCtxId(posLastX)))
    {
      break;
    }
  }
  for (; posLastY < maxLastPosY; posLastY++)
  {
    if (!m_binDecoder.decodeBin(cctx.lastYCtxId(posLastY)))
    {
      break;
    }
  }
  if (posLastX > 3)
  {
    const int  log2Width        = floorLog2(cctx.width());
    const bool maxValueSignaled = (log2Width >= LOG2_SECONDARY_PREFIX_START_SIZE && posLastX == maxLastPosX);
    bool       isMaximum        = false;
    if (maxValueSignaled)
    {
      int ctxId =
        (compID == CompID::COMP_Y ? log2Width - LOG2_SECONDARY_PREFIX_START_SIZE : 5 + to_underlying(compID) - 1);
      isMaximum = (bool)m_binDecoder.decodeBin(Ctx::lastXSecondaryPrefix(ctxId));
      DTRACE(g_trace_ctx, D_SYNTAX, "last_sig_coeff() isMaxX=%d\n", int(isMaximum));
    }
    if (isMaximum)
    {
      posLastX = g_minInGroup[posLastX + 1] - 1;
    }
    else
    {
      const int ctxId  = std::max<int>(0, log2Width - LOG2_ID_SUFFIX_CTX_START_SIZE);
      const int count  = (((int)posLastX - 2) >> 1) - 1;
      const int maxTmp = ((1 << count) - 1) << 1;
      int       temp   = 0;
      for (int i = count; i >= 1; i--)
      {
        if (i == count)
        {
          temp = m_binDecoder.decodeBin(Ctx::lastXSuffix[compID](ctxId)) << i;
        }
        else
        {
          temp += m_binDecoder.decodeBinEP() << i;
        }
        DTRACE(g_trace_ctx, D_SYNTAX, "last_sig_coeff() lastXSuffixBin(%d)=%d\n", i, (temp >> i) & 1);
      }
      if (!maxValueSignaled || temp != maxTmp)
      {
        temp += m_binDecoder.decodeBinEP();
        DTRACE(g_trace_ctx, D_SYNTAX, "last_sig_coeff() lastXSuffixBin(%d)=%d\n", 0, temp & 1);
      }
      posLastX = g_minInGroup[posLastX] + temp;
    }
  }
  if (posLastY > 3)
  {
    const int  log2Height       = floorLog2(cctx.height());
    const bool maxValueSignaled = (log2Height >= LOG2_SECONDARY_PREFIX_START_SIZE && posLastY == maxLastPosY);
    bool       isMaximum        = false;
    if (maxValueSignaled)
    {
      int ctxId =
        (compID == CompID::COMP_Y ? log2Height - LOG2_SECONDARY_PREFIX_START_SIZE : 5 + to_underlying(compID) - 1);
      isMaximum = (bool)m_binDecoder.decodeBin(Ctx::lastYSecondaryPrefix(ctxId));
      DTRACE(g_trace_ctx, D_SYNTAX, "last_sig_coeff() isMaxY=%d\n", int(isMaximum));
    }
    if (isMaximum)
    {
      posLastY = g_minInGroup[posLastY + 1] - 1;
    }
    else
    {
      const int ctxId  = std::max<int>(0, log2Height - LOG2_ID_SUFFIX_CTX_START_SIZE);
      const int count  = (((int)posLastY - 2) >> 1) - 1;
      const int maxTmp = ((1 << count) - 1) << 1;
      int       temp   = 0;
      for (int i = count; i >= 1; i--)
      {
        if (i == count)
        {
          temp = m_binDecoder.decodeBin(Ctx::lastYSuffix[compID](ctxId)) << i;
        }
        else
        {
          temp += m_binDecoder.decodeBinEP() << i;
        }
        DTRACE(g_trace_ctx, D_SYNTAX, "last_sig_coeff() lastYSuffixBin(%d)=%d\n", i, (temp >> i) & 1);
      }
      if (!maxValueSignaled || temp != maxTmp)
      {
        temp += m_binDecoder.decodeBinEP();
        DTRACE(g_trace_ctx, D_SYNTAX, "last_sig_coeff() lastYSuffixBin(%d)=%d\n", 0, temp & 1);
      }
      posLastY = g_minInGroup[posLastY] + temp;
    }
  }

  if (tu.cu->slice->m_reverseLastSigCoeffFlag)
  {
    posLastX = zoTbWdith - 1 - posLastX;
    posLastY = zoTbHeight - 1 - posLastY;
  }
  int blkPos;
  blkPos = posLastX + (posLastY * cctx.width());

  int scanPos = 0;
  for (; scanPos < cctx.maxNumCoeff() - 1; scanPos++)
  {
    if (blkPos == cctx.blockPos(scanPos))
    {
      break;
    }
  }
  return scanPos;
}

static void checkCoeffInRange(const CoeffCodingContext &cctx, const TCoeff coeff)
{
  CHECK(coeff < cctx.minCoeff() || coeff > cctx.maxCoeff(), "TransCoeffLevel outside allowable range");
}

void CABACReader::residual_coding_subblock(CoeffCodingContext &cctx, TCoeff *coeff, const uint64_t stateTransTable,
                                           int &state)
{
  // NOTE: All coefficients of the subblock must be set to zero before calling this function
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatisticsClassType ctype_group(STATS__CABAC_BITS__SIG_COEFF_GROUP_FLAG, cctx.width(), cctx.height(),
                                        cctx.compID());
  CodingStatisticsClassType ctype_map(STATS__CABAC_BITS__SIG_COEFF_MAP_FLAG, cctx.width(), cctx.height(),
                                      cctx.compID());
  CodingStatisticsClassType ctype_par(STATS__CABAC_BITS__PAR_FLAG, cctx.width(), cctx.height(), cctx.compID());
  CodingStatisticsClassType ctype_gt1(STATS__CABAC_BITS__GT1_FLAG, cctx.width(), cctx.height(), cctx.compID());
  CodingStatisticsClassType ctype_gt2(STATS__CABAC_BITS__GT2_FLAG, cctx.width(), cctx.height(), cctx.compID());
  CodingStatisticsClassType ctype_gt3(STATS__CABAC_BITS__GT3_FLAG, cctx.width(), cctx.height(), cctx.compID());
  CodingStatisticsClassType ctype_gt4(STATS__CABAC_BITS__GT4_FLAG, cctx.width(), cctx.height(), cctx.compID());
  CodingStatisticsClassType ctype_escs(STATS__CABAC_BITS__ESCAPE_BITS, cctx.width(), cctx.height(), cctx.compID());
#endif

  //===== init =====
  const int  minSubPos     = cctx.minSubPos();
  const bool isLast        = cctx.isLast();
  int        firstSigPos   = (isLast ? cctx.scanPosLast() : cctx.maxSubPos());
  int        nextSigPos    = firstSigPos;
  bool       updateHistory = cctx.getUpdateHist();

  //===== decode significant_coeffgroup_flag =====
  RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_group);
  bool sigGroup = (isLast || !minSubPos);
  if (!sigGroup)
  {
    sigGroup = m_binDecoder.decodeBin(cctx.sigGroupCtxId());
  }
  if (sigGroup)
  {
    cctx.setSigGroup();
  }
  else
  {
    return;
  }

  uint8_t ctxOffset[16];

  //===== decode absolute values =====
  const int inferSigPos   = nextSigPos != cctx.scanPosLast() ? (cctx.isNotFirst() ? minSubPos : -1) : nextSigPos;
  int       firstNZPos    = nextSigPos;
  int       lastNZPos     = -1;
  int       numNonZero    = 0;
  int       numGt1        = 0;
  int       remRegBins    = cctx.remRegBins;
  int       firstPosMode2 = minSubPos - 1;
  int       sigBlkPos[1 << MLS_CG_SIZE];

  for (; nextSigPos >= minSubPos && remRegBins >= MAX_REG_BINS; nextSigPos--)
  {
    int      blkPos  = cctx.blockPos(nextSigPos);
    unsigned sigFlag = (!numNonZero && nextSigPos == inferSigPos);
    if (!sigFlag)
    {
      RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_map);
      const unsigned sigCtxId = cctx.sigCtxIdAbs(nextSigPos, coeff, state);
      sigFlag                 = m_binDecoder.decodeBin(sigCtxId);
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
      uint8_t &ctxOff         = ctxOffset[nextSigPos - minSubPos];
      ctxOff                  = cctx.ctxOffsetAbs();
      sigBlkPos[numNonZero++] = blkPos;
      firstNZPos              = nextSigPos;
      lastNZPos               = std::max<int>(lastNZPos, nextSigPos);

      unsigned gtX     = 1;
      unsigned sumGtX  = 0;
      unsigned parFlag = 0;
      for (int k = 1; k <= GTN && gtX; k++)
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_SET(k == 1     ? ctype_gt1
                                                 : k == 2 ? ctype_gt2
                                                 : k == 3 ? ctype_gt3
                                                          : ctype_gt4);
        unsigned ctxId = cctx.greaterXCtxIdAbs(k, ctxOff);
        gtX            = m_binDecoder.decodeBin(ctxId);
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "gt%d_flag() bin=%d ctx=%d\n", k, gtX, ctxId);
        sumGtX += gtX;
        remRegBins--;
      }
      if (gtX)
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_par);
        parFlag = m_binDecoder.decodeBinEP();
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "par_flag() bin=%d \n", parFlag);
      }
      coeff[blkPos] += 1 + sumGtX + parFlag;
      numGt1 += (coeff[blkPos] > 1);
    }

    state = int((stateTransTable >> ((state << 3) + ((coeff[blkPos] & 1) << 2))) & 15);
  }
  firstPosMode2   = nextSigPos;
  cctx.remRegBins = remRegBins;

  //===== 2nd PASS: Go-rice codes =====
  for (int scanPos = firstSigPos; scanPos > firstPosMode2; scanPos--)
  {
    TCoeff &tcoeff = coeff[cctx.blockPos(scanPos)];
    if (tcoeff >= GTN_LEVEL)
    {
      const unsigned ricePar = cctx.deriveRiceGTN(scanPos, coeff);
      RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_escs);
      int rem = m_binDecoder.decodeRemAbsEP(ricePar, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange());
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, ricePar);
      tcoeff += (rem << 1);
    }
  }

  //===== coeff bypass ====
  for (int scanPos = firstPosMode2; scanPos >= minSubPos; scanPos--)
  {
    int rice = (cctx.*(cctx.deriveRiceRRC))(scanPos, coeff, 0);
    int pos0 = g_goRicePosCoeff0(state, rice);
    RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_escs);
    int rem = m_binDecoder.decodeRemAbsEP(rice, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange());
    DTRACE(g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, rice);
    TCoeff tcoeff = (rem == pos0 ? 0 : rem < pos0 ? rem + 1 : rem);
    state         = int((stateTransTable >> ((state << 3) + ((tcoeff & 1) << 2))) & 15);
    if ((updateHistory) && (rem > 0))
    {
      unsigned &riceStats = m_binDecoder.getCtx().getGRAdaptStats((unsigned)(cctx.compID()));
      cctx.updateRiceStat(riceStats, rem, 0);
      cctx.setUpdateHist(0);
      updateHistory = 0;
    }
    if (tcoeff)
    {
      int blkPos              = cctx.blockPos(scanPos);
      sigBlkPos[numNonZero++] = blkPos;
      firstNZPos              = scanPos;
      lastNZPos               = std::max<int>(lastNZPos, scanPos);
      coeff[blkPos]           = tcoeff;
      numGt1 += (tcoeff > 1);
    }
  }

  //===== decode sign's =====
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__SIGN_BIT, Size(cctx.width(), cctx.height()),
                                                      cctx.compID());
  const unsigned numSigns    = (cctx.hideSign(firstNZPos, lastNZPos) ? numNonZero - 1 : numNonZero);
  const bool     signPred    = cctx.isSignPredCG();
  unsigned       signPattern = (signPred ? 0 : m_binDecoder.decodeBinsEP(numSigns) << (32 - numSigns));

  //===== set final coefficents =====
  TCoeff sumAbs = 0;
  for (unsigned k = 0; k < numSigns; k++)
  {
    TCoeff absCoeff = coeff[sigBlkPos[k]];
    sumAbs += absCoeff;
    coeff[sigBlkPos[k]] = (signPattern & (1u << 31) ? -absCoeff : absCoeff);
    signPattern <<= 1;

    checkCoeffInRange(cctx, coeff[sigBlkPos[k]]);
    // NOTE: when Slice::m_depQuantEnabledIdc is true, additional checks are required to determine
    // whether coeff is in valid range (see DQIntern::Quantizer::dequantBlock)
  }
  if (signPred)
  {
    numGt1 -= (numNonZero > numSigns && coeff[sigBlkPos[numSigns]] > 1);
    cctx.numSignsPredArea() += (numGt1 << 16) + numSigns;
  }
  if (numNonZero > numSigns)
  {
    int    k        = numSigns;
    TCoeff absCoeff = coeff[sigBlkPos[k]];
    sumAbs += absCoeff;
    coeff[sigBlkPos[k]] = (sumAbs & 1 ? -absCoeff : absCoeff);
    checkCoeffInRange(cctx, coeff[sigBlkPos[k]]);
  }
}

void CABACReader::residual_codingTS(TransformUnit &tu, CompID compID)
{
  DTRACE(g_trace_ctx, D_SYNTAX, "residual_codingTS() etype=%d pos=(%d,%d) size=%dx%d\n", tu.blocks[compID].compID,
         tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height);

  // init coeff coding context
  CoeffCodingContext cctx(tu, compID, false, tu.cu->getBdpcmMode(compID));
  TCoeff            *coeff      = tu.getCoeffs(compID).buf;
  int                maxCtxBins = (cctx.maxNumCoeff() * 7) >> 2;
  cctx.remRegBins               = maxCtxBins;

  for (int subSetId = 0; subSetId <= (cctx.maxNumCoeff() - 1) >> cctx.log2CGSize(); subSetId++)
  {
    cctx.initSubblock(subSetId);
    int goRiceParam = 1;
    if (tu.cu->slice->m_sps->m_spsRangeExtension.m_tsrcRicePresentFlag && tu.mtsIdx[compID] == MtsType::SKIP)
    {
      goRiceParam = goRiceParam + tu.cu->slice->m_tsrcIndex;
    }
    residual_coding_subblockTS(cctx, coeff, goRiceParam);
  }
}

void CABACReader::residual_coding_subblockTS(CoeffCodingContext &cctx, TCoeff *coeff, const int riceParam)
{
  // NOTE: All coefficients of the subblock must be set to zero before calling this function
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatisticsClassType ctype_group(STATS__CABAC_BITS__SIG_COEFF_GROUP_FLAG, cctx.width(), cctx.height(),
                                        cctx.compID());
#if TR_ONLY_COEFF_STATS
  CodingStatisticsClassType ctype_map(STATS__CABAC_BITS__SIG_COEFF_MAP_FLAG_TS, cctx.width(), cctx.height(),
                                      cctx.compID());
  CodingStatisticsClassType ctype_par(STATS__CABAC_BITS__PAR_FLAG_TS, cctx.width(), cctx.height(), cctx.compID());
  CodingStatisticsClassType ctype_gt1(STATS__CABAC_BITS__GT1_FLAG_TS, cctx.width(), cctx.height(), cctx.compID());
  CodingStatisticsClassType ctype_gt2(STATS__CABAC_BITS__GT2_FLAG_TS, cctx.width(), cctx.height(), cctx.compID());
  CodingStatisticsClassType ctype_escs(STATS__CABAC_BITS__ESCAPE_BITS_TS, cctx.width(), cctx.height(), cctx.compID());
#else
  CodingStatisticsClassType ctype_map(STATS__CABAC_BITS__SIG_COEFF_MAP_FLAG, cctx.width(), cctx.height(),
                                      cctx.compID());
  CodingStatisticsClassType ctype_par(STATS__CABAC_BITS__PAR_FLAG, cctx.width(), cctx.height(), cctx.compID());
  CodingStatisticsClassType ctype_gt1(STATS__CABAC_BITS__GT1_FLAG, cctx.width(), cctx.height(), cctx.compID());
  CodingStatisticsClassType ctype_gt2(STATS__CABAC_BITS__GT2_FLAG, cctx.width(), cctx.height(), cctx.compID());
  CodingStatisticsClassType ctype_escs(STATS__CABAC_BITS__ESCAPE_BITS, cctx.width(), cctx.height(), cctx.compID());
#endif

#endif

  //===== init =====
  const int minSubPos   = cctx.maxSubPos();
  int       firstSigPos = cctx.minSubPos();
  int       nextSigPos  = firstSigPos;
  unsigned  signPattern = 0;

  //===== decode significant_coeffgroup_flag =====
  RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_group);
  bool sigGroup = cctx.isLastSubSet() && cctx.noneSigGroup();
  if (!sigGroup)
  {
    sigGroup = m_binDecoder.decodeBin(cctx.sigGroupCtxId(true));
    DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_sigGroup() bin=%d ctx=%d\n", sigGroup, cctx.sigGroupCtxId());
  }
  if (sigGroup)
  {
    cctx.setSigGroup();
  }
  else
  {
    return;
  }

  //===== decode absolute values =====
  const int inferSigPos = minSubPos;
  int       numNonZero  = 0;
  int       sigBlkPos[1 << MLS_CG_SIZE];

  int lastScanPosPass1 = -1;
  int lastScanPosPass2 = -1;
  for (; nextSigPos <= minSubPos && cctx.remRegBins >= 4; nextSigPos++)
  {
    int      blkPos  = cctx.blockPos(nextSigPos);
    unsigned sigFlag = (!numNonZero && nextSigPos == inferSigPos);
    if (!sigFlag)
    {
      RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_map);
      const unsigned sigCtxId = cctx.sigCtxIdAbsTS(nextSigPos, coeff);
      sigFlag                 = m_binDecoder.decodeBin(sigCtxId);
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId);
      cctx.remRegBins--;
    }

    if (sigFlag)
    {
      //===== decode sign's =====
#if TR_ONLY_COEFF_STATS
      RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__SIGN_BIT_TS,
                                                          Size(cctx.width(), cctx.height()), cctx.compID());
#else
      RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__SIGN_BIT,
                                                          Size(cctx.width(), cctx.height()), cctx.compID());
#endif
      int            sign;
      const unsigned signCtxId = cctx.signCtxIdAbsTS(nextSigPos, coeff, cctx.bdpcm());
      sign                     = m_binDecoder.decodeBin(signCtxId);
      cctx.remRegBins--;

      signPattern += (sign << numNonZero);

      sigBlkPos[numNonZero++] = blkPos;

      RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_gt1);
      unsigned       gt1Flag;
      const unsigned gt1CtxId = cctx.lrg1CtxIdAbsTS(nextSigPos, coeff, cctx.bdpcm());
      gt1Flag                 = m_binDecoder.decodeBin(gt1CtxId);
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_gt1_flag() bin=%d ctx=%d\n", gt1Flag, gt1CtxId);
      cctx.remRegBins--;

      unsigned parFlag = 0;
      if (gt1Flag)
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_par);
        parFlag = m_binDecoder.decodeBin(cctx.parityCtxIdAbsTS());
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_par_flag() bin=%d ctx=%d\n", parFlag, cctx.parityCtxIdAbsTS());
        cctx.remRegBins--;
      }
      coeff[blkPos] = (sign ? -1 : 1) * (TCoeff)(1 + parFlag + gt1Flag);
    }
    lastScanPosPass1 = nextSigPos;
  }

  int       cutoffVal = 2;
  const int numGtBins = 4;

  //===== 2nd PASS: gt2 =====
  for (int scanPos = firstSigPos; scanPos <= minSubPos && cctx.remRegBins >= 4; scanPos++)
  {
    TCoeff &tcoeff = coeff[cctx.blockPos(scanPos)];
    cutoffVal      = 2;
    for (int i = 0; i < numGtBins; i++)
    {
      if (tcoeff < 0)
      {
        tcoeff = -tcoeff;
      }
      if (tcoeff >= cutoffVal)
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_gt2);
        unsigned gt2Flag;
        gt2Flag = m_binDecoder.decodeBin(cctx.greaterXCtxIdAbsTS(cutoffVal >> 1));
        tcoeff += (gt2Flag << 1);
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_gt%d_flag() bin=%d ctx=%d sp=%d coeff=%d\n", i + 2, gt2Flag,
               cctx.greaterXCtxIdAbsTS(cutoffVal >> 1), scanPos, tcoeff);
        cctx.remRegBins--;
      }
      cutoffVal += 2;
    }
    lastScanPosPass2 = scanPos;
  }
  //===== 3rd PASS: Go-rice codes =====
  for (int scanPos = firstSigPos; scanPos <= minSubPos; scanPos++)
  {
    TCoeff &tcoeff = coeff[cctx.blockPos(scanPos)];
    RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_escs);

    cutoffVal = (scanPos <= lastScanPosPass2 ? 10 : (scanPos <= lastScanPosPass1 ? 2 : 0));
    if (tcoeff < 0)
    {
      tcoeff = -tcoeff;
    }
    if (tcoeff >= cutoffVal)
    {
      int rice = riceParam;
      int rem  = m_binDecoder.decodeRemAbsEP(rice, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange());
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_rem_val() bin=%d ctx=%d sp=%d\n", rem, rice, scanPos);
      tcoeff += (scanPos <= lastScanPosPass1) ? (rem << 1) : rem;
      if (tcoeff && scanPos > lastScanPosPass1)
      {
        int blkPos = cctx.blockPos(scanPos);
        int sign   = m_binDecoder.decodeBinEP();
        signPattern += (sign << numNonZero);
        sigBlkPos[numNonZero++] = blkPos;
      }
    }
    if (cctx.bdpcm() == BdpcmMode::NONE && cutoffVal)
    {
      if (tcoeff > 0)
      {
        int rightPixel, belowPixel;
        cctx.neighTS(rightPixel, belowPixel, scanPos, coeff);
        tcoeff = cctx.decDeriveModCoeff(rightPixel, belowPixel, tcoeff);
      }
    }
  }

  //===== set final coefficents =====
  for (unsigned k = 0; k < numNonZero; k++)
  {
    TCoeff absCoeff     = coeff[sigBlkPos[k]];
    coeff[sigBlkPos[k]] = (signPattern & 1 ? -absCoeff : absCoeff);
    signPattern >>= 1;
    checkCoeffInRange(cctx, coeff[sigBlkPos[k]]);
  }
}

//================================================================================
//  helper functions
//--------------------------------------------------------------------------------
//    unsigned  unary_max_symbol ( ctxId0, ctxId1, maxSymbol )
//    unsigned  unary_max_eqprob (                 maxSymbol )
//    unsigned  exp_golomb_eqprob( count )
//================================================================================

unsigned CABACReader::unary_max_symbol(unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol)
{
  unsigned onesRead = 0;
  while (onesRead < maxSymbol && m_binDecoder.decodeBin(onesRead == 0 ? ctxId0 : ctxIdN) == 1)
  {
    ++onesRead;
  }
  return onesRead;
}

unsigned CABACReader::unary_max_eqprob(unsigned maxSymbol)
{
  for (unsigned k = 0; k < maxSymbol; k++)
  {
    if (!m_binDecoder.decodeBinEP())
    {
      return k;
    }
  }
  return maxSymbol;
}

unsigned CABACReader::exp_golomb_eqprob(unsigned count)
{
  unsigned symbol = 0;
  unsigned bit    = 1;
  while (bit)
  {
    bit = m_binDecoder.decodeBinEP();
    symbol += bit << count++;
  }
  if (--count)
  {
    symbol += m_binDecoder.decodeBinsEP(count);
  }
  return symbol;
}

void CABACReader::mip_flag(CodingUnit &cu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MIP, cu.lumaSize());

  if (!cu.Y().valid())
  {
    return;
  }
  if (!cu.cs->sps->m_useMIP)
  {
    cu.mipFlag = false;
    return;
  }

  unsigned ctxId = DeriveCtx::CtxMipFlag(cu);
  cu.mipFlag     = m_binDecoder.decodeBin(Ctx::MipFlag(ctxId));
  DTRACE(g_trace_ctx, D_SYNTAX, "mip_flag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.mipFlag ? 1 : 0);
}

void CABACReader::mip_pred_modes(CodingUnit &cu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MIP, cu.lumaSize());

  if (!cu.Y().valid())
  {
    return;
  }

  mip_pred_mode(cu);
}

void CABACReader::mip_pred_mode(CodingUnit &cu)
{
  cu.mipTransposedFlag = bool(m_binDecoder.decodeBinEP());

  uint32_t  mipMode;
  const int numModes = MatrixIntraPrediction::getNumModesMip(cu.Y());
  xReadTruncBinCode(mipMode, numModes);
  cu.intraDir[ChannelType::LUMA] = mipMode;
  CHECKD(cu.intraDir[ChannelType::LUMA] < 0 || cu.intraDir[ChannelType::LUMA] >= numModes, "Invalid MIP mode");

  DTRACE(g_trace_ctx, D_SYNTAX, "mip_pred_mode() pos=(%d,%d) mode=%d transposed=%d\n", cu.lumaPos().x, cu.lumaPos().y,
         cu.intraDir[ChannelType::LUMA], cu.mipTransposedFlag ? 1 : 0);
}

void CABACReader::planarDir(CodingUnit &cu)
{
  cu.plDir = PlanarDirType::NO_DIR;
  if (!PU::directionalPlanarAvailable(cu) || !cu.Y().valid())
  {
    return;
  }
  if (m_binDecoder.decodeBin(Ctx::PlanarDir(0)))
  {
    cu.plDir = (m_binDecoder.decodeBin(Ctx::PlanarDir(1)) == 0) ? PlanarDirType::HOR : PlanarDirType::VER;
  }
}

void CABACReader::dimd_flag(CodingUnit &cu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__DIMD_FLAG, cu.lumaSize());

  if (!cu.Y().valid() || !cu.cs->sps->m_useDIMD)
  {
    CHECK(cu.dimdFlag, "Error, dimd supported only for luma and when enabled in sps");
    return;
  }
  unsigned ctxId = DeriveCtx::CtxDimdFlag(cu);
  cu.dimdFlag    = m_binDecoder.decodeBin(Ctx::DimdFlag(ctxId));
  obic_flag(cu);
}

void CABACReader::dimdChromaFlag(CodingUnit &cu)
{
  const bool isChromaChannelValid = cu.Cb().valid() && cu.Cr().valid();
  if (isChromaChannelValid && cu.cs->sps->m_useDIMDChroma)
  {
    bool dimdSignalled = (m_binDecoder.decodeBin(Ctx::DimdChromaFlag()) == 0);
    cu.dimdChromaFlag  = dimdSignalled;
  }
}

void CABACReader::timd_flag(CodingUnit &cu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__TIMD_FLAG, cu.lumaSize());

  if (!cu.Y().valid() || !cu.cs->sps->m_useTIMD)
  {
    CHECK(cu.timdFlag, "Error, timd supported only for luma and when enabled in sps");
    return;
  }
  unsigned ctxId = DeriveCtx::CtxTimdFlag(cu);
  cu.timdFlag    = m_binDecoder.decodeBin(Ctx::TimdFlag(ctxId));
  DTRACE(g_trace_ctx, D_SYNTAX, "timd_flag() pos=(%d,%d) size=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y,
         cu.lumaSize().width, cu.lumaSize().height, cu.timdFlag);
}

void CABACReader::timd_sad_flag(CodingUnit &cu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__TIMDSAD_FLAG, cu.lumaSize());

  if (!cu.Y().valid() || !cu.cs->sps->m_useTIMD || !cu.cs->sps->m_useTIMDSAD)
  {
    CHECK(!cu.timdFlag, "Error, timd SAD supported only for luma and when TIMD and TIMDSAD is enabled in sps");
    return;
  }

  CHECK(!cu.timdFlag, "Error, timd must be aneabled for timdSAD");

  cu.timdSadFlag = false;
  if (cu.timdFlag && CU::allowTimdSad(cu))
  {
    cu.timdSadFlag = m_binDecoder.decodeBin(Ctx::TimdSadFlag());
    DTRACE(g_trace_ctx, D_SYNTAX, "timd_sad_flag() pos=(%d,%d) size=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y,
           cu.lumaSize().width, cu.lumaSize().height, cu.timdSadFlag);
  }
}

void CABACReader::eip_flag(CodingUnit &cu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__EIP_FLAG, cu.lumaSize());

  if (!cu.Y().valid() || !isLuma(cu.chType) || !cu.cs->sps->m_useEIP)
  {
    cu.eipFlag = false;
    return;
  }

  const bool bCanUseEip = CU::eipAllowed(cu, CompID::COMP_Y) || CU::eipMergeAllowed(cu, CompID::COMP_Y);
  if (bCanUseEip)
  {
    cu.eipFlag = m_binDecoder.decodeBin(Ctx::EipFlag(0));
    if (cu.eipFlag)
    {
      if (CU::eipAllowed(cu, CompID::COMP_Y) && CU::eipMergeAllowed(cu, CompID::COMP_Y))
      {
        cu.eipMerge = m_binDecoder.decodeBin(Ctx::EipFlag(1));
      }
      else if (CU::eipMergeAllowed(cu, CompID::COMP_Y))
      {
        cu.eipMerge = true;
      }
      else
      {
        cu.eipMerge = false;
      }

      if (cu.eipMerge)
      {
        cu.intraDir[ChannelType::LUMA] = unary_max_eqprob(NUM_EIP_MERGE_SIGNAL - 1);
      }
      else
      {
        uint32_t                                symbol = 0;
        static_vector<EipInfo, NUM_DERIVED_EIP> eipInfoList;
        if (cu.cs->sps->m_useMMEIP)
        {
          cu.eipMultiModel = m_binDecoder.decodeBin(Ctx::EipFlag(2));
        }
        else
        {
          cu.eipMultiModel = false;
        }
        CU::eipCurAllowed(cu, CompID::COMP_Y, eipInfoList, cu.eipMultiModel);
        xReadTruncBinCode(symbol, uint32_t(eipInfoList.size()));
        cu.intraDir[ChannelType::LUMA] = symbol;
      }
    }
  }
  else
  {
    cu.eipFlag = false;
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "eip_flag() pos=(%d,%d) mode=%d idx= %d merge=%d mm=%d\n", cu.lumaPos().x,
         cu.lumaPos().y, cu.eipFlag ? 1 : 0, cu.intraDir[ChannelType::LUMA], cu.eipMerge ? 1 : 0,
         cu.eipMultiModel ? 1 : 0);
}

void CABACReader::obic_flag(CodingUnit &cu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__OBIC_FLAG, cu.lumaSize());

  if (!cu.Y().valid() || !cu.cs->sps->m_useOBIC || !cu.dimdFlag)
  {
    CHECK(cu.obicFlag, "Error, obic supported only for luma and when enabled in sps");
    return;
  }
  PU::getObicNeighbours(cu, cu.obicNeighbours);
  cu.obicAvailFlag = PU::isObicAvail(cu);
  if (!cu.obicAvailFlag)
  {
    return;
  }
  cu.obicFlag = m_binDecoder.decodeBin(Ctx::ObicFlag(0));
  DTRACE(g_trace_ctx, D_SYNTAX, "cu_obic_flag() pos=(%d,%d) obic=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.obicFlag);
}

void CABACReader::cu_lic_flag(CodingUnit &cu)
{
  if (CU::isLICFlagPresent(cu))
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__LIC_FLAG, cu.lumaSize());
    cu.licFlag = m_binDecoder.decodeBin(Ctx::LICFlag(0));
    DTRACE(g_trace_ctx, D_SYNTAX, "cu_lic_flag() lic_flag=%d\n", cu.licFlag ? 1 : 0);
  }
}
