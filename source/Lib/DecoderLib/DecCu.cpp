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

/** \file     DecCu.cpp
    \brief    CU decoder class
*/

#include "DecCu.h"

#include "CommonLib/InterPrediction.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"

#include "CommonLib/dtrace_buffer.h"

#if RExt__DECODER_DEBUG_TOOL_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif
#if K0149_BLOCK_STATISTICS
#include "CommonLib/ChromaFormat.h"
#include "CommonLib/dtrace_blockstatistics.h"
#endif

//! \ingroup DecoderLib
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

DecCu::DecCu() : m_tmpStorageCtu(nullptr), m_tmpStorageCtu2(nullptr) {}

DecCu::~DecCu() { destoryDecCuReshaprBuf(); }

void DecCu::init(TrQuant *pcTrQuant, IntraPrediction *pcIntra, InterPrediction *pcInter)
{
  m_pcTrQuant   = pcTrQuant;
  m_pcIntraPred = pcIntra;
  m_pcInterPred = pcInter;
}

void DecCu::initDecCuReshaper(Reshape *pcReshape, ChromaFormat chromaFormatIdc)
{
  m_pcReshape = pcReshape;
  if (m_tmpStorageCtu == nullptr)
  {
    m_tmpStorageCtu = new PelStorage;
    m_tmpStorageCtu->create(UnitArea(chromaFormatIdc, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
  }

  if (m_tmpStorageCtu2 == nullptr)
  {
    m_tmpStorageCtu2 = new PelStorage;
    m_tmpStorageCtu2->create(UnitArea(chromaFormatIdc, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
  }
}
void DecCu::destoryDecCuReshaprBuf()
{
  if (m_tmpStorageCtu)
  {
    m_tmpStorageCtu->destroy();
    delete m_tmpStorageCtu;
    m_tmpStorageCtu = nullptr;
  }

  if (m_tmpStorageCtu2)
  {
    m_tmpStorageCtu2->destroy();
    delete m_tmpStorageCtu2;
    m_tmpStorageCtu2 = nullptr;
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void DecCu::decompressCtu(CodingStructure &cs, const UnitArea &ctuArea)
{
  PROFILER_SCOPE(1, g_timeProfiler, P_REC_CTU);
  const int maxNumChannelType = isChromaEnabled(cs.pcv->chrFormat) && CS::isDualITree(cs) ? 2 : 1;

  if (cs.resetIBCBuffer)
  {
    m_pcInterPred->resetIBCBuffer(cs.pcv->chrFormat, cs.slice->m_sps->m_maxCuHeight);
    cs.resetIBCBuffer = false;
  }
  else
  {
    const int ctuSize = cs.slice->m_sps->m_maxCuHeight;
    m_pcInterPred->resetVPDUforIBC(cs.pcv->chrFormat, ctuSize, ctuSize, ctuArea.lx(), ctuArea.ly());
  }
  for (int ch = 0; ch < maxNumChannelType; ch++)
  {
    const ChannelType chType = ChannelType(ch);
    Position          prevTmpPos;
    prevTmpPos.x = -1;
    prevTmpPos.y = -1;

    for (auto &currCU: cs.traverseCUs(CS::getArea(cs, ctuArea, chType), chType))
    {
      if (!CU::isIntra(currCU) && !CU::isPLT(currCU) && currCU.Y().valid())
      {
        m_pcInterPred->resetFillLicTpl();
        xDeriveCuMvs(currCU);
#if K0149_BLOCK_STATISTICS
        if (currCU.geoFlag)
        {
          for (auto &it: m_geoMrgCtx)
          {
            storeGeoMergeCtx(it);
          }
        }
#endif
      }
      switch (currCU.predMode)
      {
      case MODE_INTER:
      case MODE_IBC:
        xReconInter(currCU);
        break;
      case MODE_PLT:
      case MODE_INTRA:
        xReconIntraQT(currCU);
        break;
      default:
        THROW("Invalid prediction mode");
        break;
      }

      m_pcInterPred->xFillIBCBuffer(currCU);

      DTRACE_BLOCK_REC(cs.picture->getRecoBuf(currCU), currCU, currCU.predMode);
    }
  }
#if K0149_BLOCK_STATISTICS
  getAndStoreBlockStatistics(cs, ctuArea);
#endif
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

void DecCu::xIntraRecBlk(TransformUnit &tu, const CompID compID)
{
  if (!tu.blocks[compID].valid())
  {
    return;
  }
  CodingStructure &cs   = *tu.cs;
  const CompArea  &area = tu.blocks[compID];

  const ChannelType chType = toChannelType(compID);

  PelBuf piPred = cs.getPredBuf(area);
  if (isLuma(chType))
  {
    PU::interpretLumaIntraMode(*tu.cu);
  }
  else
  {
    PU::interpretChromaIntraMode(*tu.cu);
  }

  const CodingUnit &cu          = *tu.cu;
  const uint32_t    chFinalMode = PU::getFinalIntraMode(cu, chType);
  PelBuf            pReco       = cs.getRecoBuf(area);

  const bool isJCCR = tu.jointCbCr && isChroma(compID);

  //===== init availability pattern =====
  CompArea areaPredReg(COMP_Y, tu.chromaFormat, area);
  if (!isJCCR || compID != CompID::COMP_Cr)
  {
    if (!PU::isDIMD(cu, chType) && !PU::isDIMDChroma(cu, chType) && !PU::isTIMD(cu, chType) &&
        !PU::isOBIC(cu, chType) && !PU::isSgpm(cu, chType))
    {
      m_pcIntraPred->initIntraPatternChType(*tu.cu, area);
    }

    //===== get prediction signal =====
    if (compID != COMP_Y && cu.decDerivedCcpMode)
    {
      if (compID == COMP_Cb)
      {
        CodingUnit &cu     = *tu.cu;
        PelBuf      predCr = cs.getPredBuf(tu.blocks[COMP_Cr]);

        cu.cccmFlag = false;

        m_pcIntraPred->initIntraPatternChType(cu, tu.blocks[COMP_Cr]);
        m_pcIntraPred->cccmCreateLumaRefs(cu, false, CONV_MODEL_CCCM);
        m_pcIntraPred->cccmCreateLumaRefs(cu, false, CONV_MODEL_CCCM_GRADLOC);
        m_pcIntraPred->xGetLumaRecPixels(cu, area, true);

        CrossCompModels ccModels[2] = {};

        m_pcIntraPred->findDecoderDerivedCcpModel(cu, ccModels[0], ccModels[1]);
        m_pcIntraPred->setAndPredCcMergeFusionCand(cu, ccModels[0], ccModels[1], piPred, predCr);
      }
    }
    else if (compID != COMP_Y && cu.idxNonLocalCCP)
    {
      if (compID == COMP_Cb)
      {
        CrossCompModels mergeList[MAX_CCP_CAND_LIST_SIZE]  = {};
        int             fusionList[MAX_CCP_FUSION_NUM * 2] = { MAX_CCP_FUSION_NUM };

        CodingUnit &cu     = *tu.cu;
        PelBuf      predCr = cs.getPredBuf(tu.blocks[COMP_Cr]);

        cu.cccmFlag = false;

        int mergeListSize = CU::getCCPModelCandidateList(cu, mergeList);

        m_pcIntraPred->cccmCreateLumaRefsForMergeList(cu, mergeList, mergeListSize);
        m_pcIntraPred->xGetLumaRecPixels(cu, area, true);
        m_pcIntraPred->reorderCCPCandidates(cu, mergeList, mergeListSize, cu.ccMergeFusionIdx ? fusionList : nullptr);

        if (cu.ccMergeFusionIdx > 0)
        {
          int i = 2 * (cu.ccMergeFusionIdx - 1);

          m_pcIntraPred->setAndPredCcMergeFusionCand(cu, mergeList[fusionList[i]], mergeList[fusionList[i + 1]], piPred,
                                                     predCr);
        }
        else
        {
          m_pcIntraPred->setAndPredCcMergeCand(cu, mergeList[cu.idxNonLocalCCP - 1], piPred, predCr);
        }
      }
    }
    else if (compID != COMP_Y && cu.cccmFlag)
    {
      // Create both Cb and Cr predictions when here for Cb
      if (compID == COMP_Cb)
      {
        CodingUnit &cu     = *tu.cu;
        PelBuf      predCr = cs.getPredBuf(tu.blocks[COMP_Cr]);

        if (cu.cccmType == CONV_MODEL_CCCM_BVG)
        {
          PU::getBvgCccmCands(cu);
          CHECK(!cu.numBvgCands, "No valid BV found!");
        }

        m_pcIntraPred->cccmCreateLumaRefs(cu);
        m_pcIntraPred->predIntraCCCM(cu, piPred, predCr);
      }
    }
    else if (compID != COMP_Y && PU::isLMCMode(chFinalMode))
    {
      const CompArea &areaCr   = tu.blocks[CompID::COMP_Cr];
      PelBuf          piPredCr = cs.getPredBuf(areaCr);
      CodingUnit     &cu       = *tu.cu;

      m_pcIntraPred->xGetLumaRecPixels(cu, area);
      m_pcIntraPred->predIntraChromaLM(compID, piPred, cu, area, chFinalMode);

      m_pcIntraPred->initIntraPatternChType(*tu.cu, areaCr);
      m_pcIntraPred->xGetLumaRecPixels(cu, areaCr);
      m_pcIntraPred->predIntraChromaLM(CompID::COMP_Cr, piPredCr, cu, areaCr, chFinalMode);
    }
    else if (PU::isDIMDChroma(cu, chType))
    {
      m_pcIntraPred->predIntraDimdChroma(piPred, *tu.cu, compID, area);
    }
    else
    {
      if (PU::isMIP(cu, chType))
      {
        PelBuf tempPredBuf = m_tmpStorageCtu2->getCompactBuf(area);
        m_pcIntraPred->initIntraMip(cu, area);
        m_pcIntraPred->predIntraMip(compID, tempPredBuf, cu);
        piPred.copyFrom(tempPredBuf);
#if RExt__DECODER_DEBUG_TOOL_STATISTICS
        CodingStatistics::IncrementStatisticTool(
          CodingStatisticsClassType { STATS__TOOL_MIP, cu.lwidth(), cu.lheight(), compID });
#endif
      }
      else if (PU::isDIMD(cu, chType))
      {
        CodingUnit &cuNonConst = *tu.cu;
        CHECK(cuNonConst.lumaSize() != tu.lumaSize(), "Error");
        m_pcIntraPred->predIntraDimd(piPred, cuNonConst, area);
#if RExt__DECODER_DEBUG_TOOL_STATISTICS
        CodingStatistics::IncrementStatisticTool(
          CodingStatisticsClassType { STATS__TOOL_DIMD, cu.lwidth(), cu.lheight(), compID });
#endif
      }
      else if (PU::isTIMD(cu, chType))
      {
        CodingUnit &cuNonConst = *tu.cu;
        if (cu.timdSadFlag)
        {
          m_pcIntraPred->deriveDimdMode(cuNonConst.dimdData, cu.cs->picture->getRecoBuf(area), area, *tu.cu);
          m_pcIntraPred->predIntraTimd(piPred, cuNonConst, area, false, IntraPrediction::TimdMode::SAD, false);
#if RExt__DECODER_DEBUG_TOOL_STATISTICS
          CodingStatistics::IncrementStatisticTool(
            CodingStatisticsClassType { STATS__TOOL_TIMDSAD, cu.lwidth(), cu.lheight(), compID });
#endif
        }
        else
        {
          m_pcIntraPred->predIntraTimd(piPred, cuNonConst, area, false, IntraPrediction::TimdMode::Normal, false);
#if RExt__DECODER_DEBUG_TOOL_STATISTICS
          CodingStatistics::IncrementStatisticTool(
            CodingStatisticsClassType { STATS__TOOL_TIMD, cu.lwidth(), cu.lheight(), compID });
#endif
        }
      }
      else if (PU::isOBIC(cu, chType))
      {
        CodingUnit &cuNonConst = *tu.cu;
        m_pcIntraPred->deriveDimdMode(cuNonConst.dimdData, cu.cs->picture->getRecoBuf(area), area, *tu.cu);
        cuNonConst.dimdFlag = true;
        m_pcIntraPred->predIntraObic(piPred, cuNonConst, area, false);
#if RExt__DECODER_DEBUG_TOOL_STATISTICS
        CodingStatistics::IncrementStatisticTool(
          CodingStatisticsClassType { STATS__TOOL_OBIC, cu.lwidth(), cu.lheight(), compID });
#endif
      }
      else if (PU::isSgpm(cu, chType))
      {
        CodingUnit &cuNonConst = *tu.cu;
        m_pcIntraPred->predIntraSGPM(piPred, cuNonConst, area, false);
      }
      else if (PU::isEIP(cu, chType))
      {
        CodingUnit &cuNonConst = *tu.cu;

        m_pcIntraPred->initIntraEip(cuNonConst, area);
        if (!cs.pcv->isEncoder)
        {
          if (cu.eipMerge)
          {
            static_vector<EipModels, MAX_MERGE_EIP> eipMergeCandList;
            m_pcIntraPred->getNeighborsEipCands(cu, eipMergeCandList, compID);
            m_pcIntraPred->reorderEipCands(cu, eipMergeCandList);
            cuNonConst.eipModels = eipMergeCandList[cu.intraDir[ChannelType::LUMA]];
          }
          else
          {
            static_vector<EipModels, NUM_DERIVED_EIP> eipModelCandList;
            m_pcIntraPred->getCurEipCands(cu, eipModelCandList, compID);
            cuNonConst.eipModels = eipModelCandList[0];
          }
        }
        m_pcIntraPred->eipPred(cu, piPred, compID);
        if (!cs.pcv->isEncoder)
        {
          const auto derivedIPrdModes = IntraPrediction::deriveIpmForTransform(piPred, cuNonConst);
          cuNonConst.inferredDimdMode = derivedIPrdModes.first;
        }
#if RExt__DECODER_DEBUG_TOOL_STATISTICS
        CodingStatistics::IncrementStatisticTool(
          CodingStatisticsClassType { STATS__TOOL_EIP, cu.lwidth(), cu.lheight(), compID });
#endif
      }
      else
      {
        m_pcIntraPred->predIntraAng(compID, piPred, cu, true, false);
      }
    }
    if (isJCCR && compID == CompID::COMP_Cb && !PU::isLMCMode(chFinalMode))
    {
      const CompArea &areaCr   = tu.blocks[CompID::COMP_Cr];
      PelBuf          piPredCr = cs.getPredBuf(areaCr);

      m_pcIntraPred->initIntraPatternChType(*tu.cu, areaCr);
      if (PU::isMIP(cu, chType))
      {
        m_pcIntraPred->initIntraMip(cu, areaCr);
        m_pcIntraPred->predIntraMip(CompID::COMP_Cr, piPredCr, cu);
      }
      else
      {
        m_pcIntraPred->predIntraAng(CompID::COMP_Cr, piPredCr, cu, true, false);
      }
    }
  }

  const Slice &slice = *cs.slice;
  bool         flag  = slice.m_lmcsEnabledFlag && (slice.isIntra() || (!slice.isIntra() && m_pcReshape->m_ctuFlag));
  if (flag && slice.m_picHeader->m_lmcsChromaResidualScaleFlag && (compID != COMP_Y) &&
      (tu.cbf[COMP_Cb] || tu.cbf[COMP_Cr]))
  {
    const Area      area  = tu.cu->Y().valid()
            ? tu.cu->Y()
            : Area(recalcPosition(tu.chromaFormat, tu.chType, ChannelType::LUMA, tu.cu->block(tu.chType).pos()),
                   recalcSize(tu.chromaFormat, tu.chType, ChannelType::LUMA, tu.cu->block(tu.chType).size()));
    const CompArea &areaY = CompArea(COMP_Y, tu.chromaFormat, area);
    int             adj   = m_pcReshape->calculateChromaAdjVpduNei(tu, areaY);
    tu.setChromaAdj(adj);
  }
  flag = flag && (tu.blocks[compID].width * tu.blocks[compID].height > 4);

  //===== derive transform =====
  if (compID == CompID::COMP_Y &&
      (isNST(tu.mtsIdx[compID]) || (tu.cs->sps->m_mtsEnabled && !tu.cs->sps->m_explicitMtsIntra)))
  {
    if (PU::isMIP(*tu.cu, toChannelType(compID)) || PU::isSgpm(*tu.cu, toChannelType(compID)))
    {
      tu.derivedIntraDirsLuma = IntraPrediction::deriveIpmForTransform(piPred, *tu.cu);
    }
  }
  else if (compID == CompID::COMP_Cb && PU::isLMCMode(chFinalMode) && isNST(tu.mtsIdx[compID]))
  {
    const CompArea &areaCr   = tu.blocks[CompID::COMP_Cr];
    PelBuf          piPredCr = cs.getPredBuf(areaCr);
    tu.derivedIntraDirChroma = IntraPrediction::deriveIpmForChromaTransform(piPred, piPredCr, *tu.cu);
  }

  //===== inverse transform =====
  PelBuf piResi = cs.getResiBuf(area);

  const QpParam cQP(tu, compID);

  const bool hasSignPred = m_pcTrQuant->recCoeffSigns(tu, compID);

  if (tu.jointCbCr && isChroma(compID))
  {
    if (compID == COMP_Cb)
    {
      PelBuf resiCr = cs.getResiBuf(tu.blocks[COMP_Cr]);
      if (tu.jointCbCr >> 1)
      {
        m_pcTrQuant->invTransformNxN(tu, COMP_Cb, piResi, cQP, hasSignPred);
      }
      else
      {
        const QpParam qpCr(tu, COMP_Cr);
        m_pcTrQuant->invTransformNxN(tu, COMP_Cr, resiCr, qpCr, hasSignPred);
      }
      m_pcTrQuant->invTransformICT(tu, piResi, resiCr);
    }
  }
  else if (TU::getCbf(tu, compID))
  {
    m_pcTrQuant->invTransformNxN(tu, compID, piResi, cQP, hasSignPred);
  }
  else
  {
    piResi.fill(0);
  }

  //===== reconstruction =====
  if (flag && (TU::getCbf(tu, compID) || tu.jointCbCr) && isChroma(compID) &&
      slice.m_picHeader->m_lmcsChromaResidualScaleFlag)
  {
    piResi.scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(compID));
  }

  cs.setDecomp(area);

#if ENABLE_NNLF
  if (slice.m_sps->m_nnlfStore)
  {
    PelBuf piPredCustom = cs.getPredBufCustom(area);
    piPredCustom.copyFrom(piPred);
  }
#endif

#if REUSE_CU_RESULTS
  CompArea tmpArea(COMP_Y, area.chromaFormat, Position(0, 0), area.size());
  PelBuf   tmpPred;
#endif
  if (slice.m_lmcsEnabledFlag && (m_pcReshape->m_ctuFlag || slice.isIntra()) && compID == COMP_Y)
  {
#if REUSE_CU_RESULTS
    tmpPred = m_tmpStorageCtu->getBuf(tmpArea);
    tmpPred.copyFrom(piPred);
#endif
  }
  ClpRng clpRng = tu.cu->cs->slice->setNewClipRange(true, &m_pcReshape->m_fwdLUT, compID);
#if KEEP_PRED_AND_RESI_SIGNALS || ENABLE_NNLF
  pReco.reconstruct(piPred, piResi, clpRng);
#else
  piPred.reconstruct(piPred, piResi, clpRng);
  pReco.copyFrom(piPred);
#endif
  if (slice.m_lmcsEnabledFlag && (m_pcReshape->m_ctuFlag || slice.isIntra()) && compID == COMP_Y)
  {
#if REUSE_CU_RESULTS
    piPred.copyFrom(tmpPred);
#endif
  }
#if REUSE_CU_RESULTS
  cs.picture->getRecoBuf(area).copyFrom(pReco);
  cs.picture->getPredBuf(area).copyFrom(piPred);
#endif
}

void DecCu::xReconIntraQT(CodingUnit &cu)
{
  PROFILER_SCOPE(0, g_timeProfiler, P_REC_INTRA);
  if (CU::isPLT(cu))
  {
    if (CS::isDualITree(*cu.cs))
    {
      if (isLuma(cu.chType))
      {
        xReconPLT(cu, COMP_Y, 1);
      }
      if (isChromaEnabled(cu.chromaFormat) && cu.chType == ChannelType::CHROMA)
      {
        xReconPLT(cu, COMP_Cb, 2);
      }
    }
    else
    {
      xReconPLT(cu, COMP_Y, getNumberValidComponents(cu.chromaFormat));
    }
    return;
  }
  for (auto chType = ChannelType::LUMA; chType <= ::getLastChannel(cu.chromaFormat); chType++)
  {
    if (cu.block(chType).valid())
    {
      xIntraRecQT(cu, chType);
    }
  }

  CU::saveModelsInHCCP(cu);

  CU::saveModelsInHEIP(cu);
}

void DecCu::xReconPLT(CodingUnit &cu, CompID compBegin, uint32_t numComp)
{
  const SPS     &sps       = *(cu.cs->sps);
  TransformUnit &tu        = *cu.firstTU;
  PelBuf         curPLTIdx = tu.getcurPLTIdx(toChannelType(compBegin));

  uint32_t height = cu.block(compBegin).height;
  uint32_t width  = cu.block(compBegin).width;

  // recon. pixels
  uint32_t scaleX = getComponentScaleX(COMP_Cb, sps.m_chromaFormatIdc);
  uint32_t scaleY = getComponentScaleY(COMP_Cb, sps.m_chromaFormatIdc);
  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      for (uint32_t compID = compBegin; compID < (compBegin + numComp); compID++)
      {
        const int       channelBitDepth = cu.cs->sps->m_bitDepths[toChannelType((CompID)compID)];
        const CompArea &area            = cu.blocks[compID];

        PelBuf       picReco     = cu.cs->getRecoBuf(area);
        PLTescapeBuf escapeValue = tu.getescapeValue((CompID)compID);
        if (curPLTIdx.at(x, y) == cu.curPLTSize[compBegin])
        {
          TCoeff  value;
          QpParam cQP(tu, (CompID)compID);
          int     qp    = cQP.Qp(true);
          int     qpRem = qp % 6;
          int     qpPer = qp / 6;
          if (compBegin != COMP_Y || compID == COMP_Y)
          {
            int invquantiserRightShift = IQUANT_SHIFT;
            int add                    = 1 << (invquantiserRightShift - 1);
            value = ((((escapeValue.at(x, y) * g_invQuantScales[0][qpRem]) << qpPer) + add) >> invquantiserRightShift);
            value = ClipBD<TCoeff>(value, channelBitDepth);
            picReco.at(x, y) = Pel(value);
          }
          else if (compBegin == COMP_Y && compID != COMP_Y && y % (1 << scaleY) == 0 && x % (1 << scaleX) == 0)
          {
            uint32_t posYC                  = y >> scaleY;
            uint32_t posXC                  = x >> scaleX;
            int      invquantiserRightShift = IQUANT_SHIFT;
            int      add                    = 1 << (invquantiserRightShift - 1);
            value = ((((escapeValue.at(posXC, posYC) * g_invQuantScales[0][qpRem]) << qpPer) + add) >>
                     invquantiserRightShift);
            value = ClipBD<TCoeff>(value, channelBitDepth);
            picReco.at(posXC, posYC) = Pel(value);
          }
        }
        else
        {
          uint32_t curIdx = curPLTIdx.at(x, y);
          if (compBegin != COMP_Y || compID == COMP_Y)
          {
            picReco.at(x, y) = cu.curPLT[compID][curIdx];
          }
          else if (compBegin == COMP_Y && compID != COMP_Y && y % (1 << scaleY) == 0 && x % (1 << scaleX) == 0)
          {
            uint32_t posYC           = y >> scaleY;
            uint32_t posXC           = x >> scaleX;
            picReco.at(posXC, posYC) = cu.curPLT[compID][curIdx];
          }
        }
      }
    }
  }
  for (uint32_t compID = compBegin; compID < (compBegin + numComp); compID++)
  {
    const CompArea &area    = cu.blocks[compID];
    PelBuf          picReco = cu.cs->getRecoBuf(area);
    cu.cs->picture->getRecoBuf(area).copyFrom(picReco);
#if ENABLE_NNLF
    if (cu.cs->sps->m_nnlfStore)
    {
      cu.cs->getPredBuf(area).fill(0);
      if (!cu.cs->parent)
      {
        cu.cs->getPredBufCustom(area).fill(0);
      }
    }
#endif
    cu.cs->setDecomp(area);
  }
}

/** Function for deriving reconstructed PU/CU chroma samples with QTree structure
* \param pcRecoYuv pointer to reconstructed sample arrays
* \param pcPredYuv pointer to prediction sample arrays
* \param pcResiYuv pointer to residue sample arrays
* \param chType    texture channel type (luma/chroma)
* \param rTu       reference to transform data
*
\ This function derives reconstructed PU/CU chroma samples with QTree recursive structure
*/

void DecCu::xIntraRecQT(CodingUnit &cu, const ChannelType chType)
{
  for (auto &currTU: CU::traverseTUs(cu))
  {
    if (isLuma(chType))
    {
      xIntraRecBlk(currTU, COMP_Y);
    }
    else
    {
      const uint32_t numValidComp = getNumberValidComponents(cu.chromaFormat);

      for (uint32_t compID = COMP_Cb; compID < numValidComp; compID++)
      {
        xIntraRecBlk(currTU, CompID(compID));
      }
    }
  }
}

#include "CommonLib/dtrace_buffer.h"

void DecCu::xReconInter(CodingUnit &cu)
{
  PROFILER_SCOPE(0, g_timeProfiler, P_REC_INTER);
  if (cu.geoFlag)
  {
    auto *reshapeLUT = (cu.slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag ? &m_pcReshape->m_fwdLUT : nullptr);
    m_pcInterPred->motionCompensationGeo(cu, m_geoMrgCtx, m_pcIntraPred, reshapeLUT);
  }
  else
  {
    m_pcIntraPred->geneIntrainterPred(cu);

    // inter prediction
    CHECK(CU::isIBC(cu) && cu.ciipFlag, "IBC and Ciip cannot be used together");
    CHECK(CU::isIBC(cu) && cu.affine, "IBC and Affine cannot be used together");
    CHECK(CU::isIBC(cu) && cu.geoFlag, "IBC and geo cannot be used together");
    CHECK(CU::isIBC(cu) && cu.mmvdMergeFlag, "IBC and MMVD cannot be used together");
    const bool luma    = cu.Y().valid();
    const bool chroma  = isChromaEnabled(cu.chromaFormat) && cu.Cb().valid();
    PelUnitBuf predBuf = cu.cs->getPredBuf(cu);
    if (luma && (chroma || !isChromaEnabled(cu.chromaFormat)))
    {
      m_pcInterPred->motionCompensation(cu, predBuf, RPLX, true, chroma, nullptr, false);
    }
    else
    {
      // IBC
      m_pcInterPred->motionCompensation(cu, predBuf, RPL0, luma, chroma, nullptr, false);
    }

    if (cu.bdmvrRefine)
    {
      // now getBdofSubPuMvOffsets is completed
      MotionBuf mb = cu.getMotionBuf();
      PU::spanBdmvrMotionInfo(cu, mb, m_mvBufBDMVR[0], m_mvBufBDMVR[1],
                              m_pcInterPred->isBDOFMvRefined() ? m_pcInterPred->getBdofSubPuMvOffset() : nullptr);
    }

    // OBMC can only be applied after spanning the motion info
    m_pcInterPred->obmcFilter(cu, predBuf, luma, chroma);
  }
  if (cu.Y().valid())
  {
    CU::saveMotionForHmvp(cu);
  }

  if (cu.ciipFlag)
  {
    if (cu.cs->slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag)
    {
      cu.cs->getPredBuf(cu).Y().rspSignal(m_pcReshape->m_fwdLUT);
    }
    m_pcIntraPred->geneWeightedPred(cu.cs->getPredBuf(cu).Y(), cu,
                                    m_pcIntraPred->getPredictorPtr2(COMP_Y, (uint32_t)cu.ciipMode), COMP_Y);
    if (isChromaEnabled(cu.chromaFormat))
    {
      m_pcIntraPred->geneWeightedPred(cu.cs->getPredBuf(cu).Cb(), cu,
                                      m_pcIntraPred->getPredictorPtr2(COMP_Cb, (uint32_t)cu.ciipMode), COMP_Cb);
      m_pcIntraPred->geneWeightedPred(cu.cs->getPredBuf(cu).Cr(), cu,
                                      m_pcIntraPred->getPredictorPtr2(COMP_Cr, (uint32_t)cu.ciipMode), COMP_Cr);
    }
  }

  DTRACE(g_trace_ctx, D_TMP, "pred ");
  DTRACE_CRC(g_trace_ctx, D_TMP, *cu.cs, cu.cs->getPredBuf(cu), &cu.Y());

  // inter recon
  xDecodeInterTexture(cu);

#if ENABLE_NNLF
  if (cu.cs->sps->m_nnlfStore && !cu.cs->parent)
  {
    if (cu.rootCbf)
    {
      cu.cs->getPredBufCustom(cu).copyFrom(cu.cs->getPredBuf(cu));
    }
    else
    {
      cu.cs->getPredBufCustom(cu).copyClip(cu.cs->getPredBuf(cu), cu.cs->slice->m_clpRngs);
    }
  }
#endif

  DTRACE(g_trace_ctx, D_TMP, "reco ");
  DTRACE_CRC(g_trace_ctx, D_TMP, *cu.cs, cu.cs->getRecoBuf(cu), &cu.Y());

  cu.cs->setDecomp(cu);
}

void DecCu::xDecodeInterTU(TransformUnit &currTU, const CompID compID)
{
  if (!currTU.blocks[compID].valid())
  {
    return;
  }

  const CompArea &area = currTU.blocks[compID];

  CodingStructure &cs = *currTU.cs;

  //===== inverse transform =====
  PelBuf resiBuf = cs.getResiBuf(area);

  QpParam cQP(currTU, compID);

  const bool lumaReshape =
    (currTU.cs->slice->m_picHeader->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag && isLuma(compID) &&
     !currTU.cu->ciipFlag && !currTU.cu->gpmIntraFlag && !CU::isIBC(*currTU.cu));
  const bool hasSignPred = m_pcTrQuant->recCoeffSigns(currTU, compID, lumaReshape ? &m_pcReshape->m_fwdLUT : nullptr);

  if (currTU.jointCbCr && isChroma(compID))
  {
    if (compID == COMP_Cb)
    {
      PelBuf resiCr = cs.getResiBuf(currTU.blocks[COMP_Cr]);
      if (currTU.jointCbCr >> 1)
      {
        m_pcTrQuant->invTransformNxN(currTU, COMP_Cb, resiBuf, cQP, hasSignPred);
      }
      else
      {
        QpParam qpCr(currTU, COMP_Cr);
        m_pcTrQuant->invTransformNxN(currTU, COMP_Cr, resiCr, qpCr, hasSignPred);
      }
      m_pcTrQuant->invTransformICT(currTU, resiBuf, resiCr);
    }
  }
  else if (TU::getCbf(currTU, compID))
  {
    m_pcTrQuant->invTransformNxN(currTU, compID, resiBuf, cQP, hasSignPred);
  }
  else
  {
    resiBuf.fill(0);
  }

  //===== reconstruction =====
  const Slice &slice = *cs.slice;
  if (slice.m_lmcsEnabledFlag && isChroma(compID) && (TU::getCbf(currTU, compID) || currTU.jointCbCr) &&
      slice.m_picHeader->m_lmcsChromaResidualScaleFlag &&
      currTU.blocks[compID].width * currTU.blocks[compID].height > 4)
  {
    resiBuf.scaleSignal(currTU.getChromaAdj(), 0, currTU.cu->cs->slice->clpRng(compID));
  }

  int firstComponent = compID;
  int lastComponent  = compID;
  if (currTU.jointCbCr && isChroma(compID))
  {
    if (compID == COMP_Cb)
    {
      firstComponent = MAX_INT;
    }
    else
    {
      firstComponent -= 1;
    }
  }
  for (auto i = firstComponent; i <= lastComponent; ++i)
  {
    CompID currCompID  = (CompID)i;
    PelBuf compResiBuf = cs.getResiBuf(currTU.blocks[currCompID]);
    ClpRng clpRng      = currTU.cu->cs->slice->setNewClipRange(true, &m_pcReshape->m_fwdLUT, currCompID);
    if (cs.picHeader->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag && isLuma(currCompID) && !currTU.cu->ciipFlag &&
        !currTU.cu->gpmIntraFlag && !CU::isIBC(*currTU.cu))
    {
      PelBuf picRecoBuff = currTU.cs->picture->getRecoBuf(currTU.blocks[currCompID]);
      picRecoBuff.copyFrom(cs.getPredBuf(currTU.blocks[currCompID]));
      picRecoBuff.rspSignal(m_pcReshape->m_fwdLUT);
      currTU.cs->getRecoBuf(currTU.blocks[currCompID]).reconstruct(picRecoBuff, compResiBuf, clpRng);
    }
    else
    {
      currTU.cs->getRecoBuf(currTU.blocks[currCompID])
        .reconstruct(cs.getPredBuf(currTU.blocks[currCompID]), compResiBuf, clpRng);
    }
  }
}

void DecCu::xDecodeInterTexture(CodingUnit &cu)
{
  if (!cu.rootCbf)
  {
    CodingStructure &cs = *cu.cs;
    cs.getResiBuf(cu).bufs[0].fill(0);
    cs.getRecoBuf(cu).copyClip(cs.getPredBuf(cu), cs.slice->m_clpRngs);
    if (cs.picHeader->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag && !cu.ciipFlag && !cu.gpmIntraFlag && !CU::isIBC(cu))
    {
      cs.getRecoBuf(cu).get(CompID::COMP_Y).rspSignal(m_pcReshape->m_fwdLUT);
    }
    ClpRng clpRng = cs.slice->setNewClipRange(true, &m_pcReshape->m_fwdLUT, COMP_Y);
    cs.getRecoBuf(cu).get(COMP_Y).reconstruct(cs.getRecoBuf(cu).get(COMP_Y), cs.getResiBuf(cu).get(COMP_Y), clpRng);
    return;
  }

  const uint32_t uiNumVaildComp = getNumberValidComponents(cu.chromaFormat);

  for (auto &currTU: CU::traverseTUs(cu))
  {
    if (isNST(currTU.mtsIdx[CompID::COMP_Y]))
    {
      currTU.derivedIntraDirsLuma = IntraPrediction::deriveIpmForTransform(cu.cs->getPredBuf(currTU).Y(), cu);
    }

    for (uint32_t ch = 0; ch < uiNumVaildComp; ch++)
    {
      const CompID     compID = CompID(ch);
      CodingStructure &cs     = *cu.cs;
      const Slice     &slice  = *cs.slice;
      if (slice.m_lmcsEnabledFlag && slice.m_picHeader->m_lmcsChromaResidualScaleFlag && (compID == COMP_Y) &&
          (currTU.cbf[COMP_Cb] || currTU.cbf[COMP_Cr]))
      {
        const CompArea &areaY = cu.blocks[COMP_Y];
        int             adj   = m_pcReshape->calculateChromaAdjVpduNei(currTU, areaY);
        currTU.setChromaAdj(adj);
      }
      xDecodeInterTU(currTU, compID);
      if (cs.pcv->isEncoder && cs.sps->m_numPredSign > 0)
      {
        PelBuf picRecoBuff = currTU.cs->picture->getRecoBuf(currTU.blocks[compID]);
        picRecoBuff.copyFrom(currTU.cs->getRecoBuf(currTU.blocks[compID]));
      }
    }
  }
}
void setAllAffineMotion(CodingUnit &cu, Mv mv0, Mv mv1, Mv mv2, int16_t refIdx, RefPicList eRefList)
{
  // Set Mv
  PU::setAllAffineMv(cu, mv0, mv1, mv2, eRefList);

  // Set RefIdx
  cu.refIdx[eRefList] = refIdx;
}

void DecCu::xDeriveCuMvs(CodingUnit &cu)
{
  PROFILER_SCOPE(0, g_timeProfiler, P_REC_INTER);
  {
    if (cu.cs->pcv->isEncoder && !cu.geoFlag && !(!cu.affine && PU::checkBDMVRCondition(cu)) &&
        cu.mergeType != MergeType::SUBPU_ATMVP)
    {
      if (cu.affine)
      {
        for (const auto l: { RPL0, RPL1 })
        {
          if (cu.cs->slice->m_numRefIdx[l] > 0)
          {
            setAllAffineMotion(cu, cu.mvAffi[l][0], cu.mvAffi[l][1], cu.mvAffi[l][2], cu.refIdx[l], l);
          }
        }
      }
      PU::spanMotionInfo(cu);
      return;
    }
#if RExt__DECODER_DEBUG_TOOL_STATISTICS
    if (cu.affine)
    {
      CodingStatistics::IncrementStatisticTool(
        CodingStatisticsClassType { STATS__TOOL_AFF, cu.lwidth(), cu.lheight() });
    }
#endif

    if (cu.mergeFlag)
    {
      MergeCtx &mrgCtx = m_mrgCtx;
      new (&mrgCtx) MergeCtx;

      if ((cu.mmvdMergeFlag && !cu.affine) || cu.mmvdSkip)
      {
        CHECK(cu.ciipFlag, "invalid Ciip");

        PU::getInterMergeCandidates(cu, mrgCtx, 1, cu.mmvdMergeIdx.pos.baseIdx + 1, 1);
        PU::getInterMMVDMergeCandidates(cu, mrgCtx);
        mrgCtx.setMmvdMergeCandiInfo(cu, cu.mmvdMergeIdx);

        PU::spanMotionInfo(cu);
      }
      else if (cu.geoFlag)
      {
        PU::getGeoMergeCandidates(cu, m_geoMrgCtx[0]);
      }
      else if (cu.affine)
      {
        AffineMergeCtx affineMergeCtx;
        if (cu.cs->sps->m_sbtmvpEnabledFlag)
        {
          Size      bufSize = g_miScaling.scale(cu.lumaSize());
          MotionBuf subPuMiBuf(m_SubPuMiBuf, bufSize);
          affineMergeCtx.subPuMvpMiBuf = subPuMiBuf;
        }
        PU::getAffineMergeCand(cu, affineMergeCtx, cu.mergeIdx, cu.mmvdMergeFlag);

        if (cu.mmvdMergeFlag)
        {
          cu.mergeIdx = PU::getMergeIdxFromAffMmvdBaseIdx(affineMergeCtx, cu.mmvdMergeIdx.pos.baseIdx);
          CHECK(cu.mergeIdx >= cu.slice->m_picHeader->m_maxNumAffineMergeCand,
                "Affine MMVD mode doesn't have a valid base candidate!");
          PU::getAffMmvdMvf(cu, affineMergeCtx, affineMergeCtx.mvFieldNeighbours[cu.mergeIdx], cu.mergeIdx,
                            cu.mmvdMergeIdx.pos.step, cu.mmvdMergeIdx.pos.position);
        }
        cu.interDir   = affineMergeCtx.interDirNeighbours[cu.mergeIdx];
        cu.affineType = affineMergeCtx.affineType[cu.mergeIdx];
        cu.bcwIdx     = affineMergeCtx.bcwIdx[cu.mergeIdx];
#if !ENABLE_POST_CFE_CHANGES
        cu.licFlag = affineMergeCtx.LICFlags[cu.mergeIdx] ^ cu.oppositeLicFlag;
#endif
        CHECK(!cu.cs->sps->m_biLicEnabledFlag && cu.interDir == 3 && cu.licFlag, "LIC flag is not allowed with bi");
        cu.obmcFlag  = CU::isObmcAllowed(cu);
        cu.mergeType = affineMergeCtx.mergeType[cu.mergeIdx];
        if (cu.mergeType == MergeType::SUBPU_ATMVP)
        {
          cu.refIdx[0] = affineMergeCtx.mvFieldNeighbours[cu.mergeIdx][0][0].refIdx;
          cu.refIdx[1] = affineMergeCtx.mvFieldNeighbours[cu.mergeIdx][0][1].refIdx;
#if ENABLE_POST_CFE_CHANGES
          cu.licFlag = affineMergeCtx.LICFlags[cu.mergeIdx];
#endif
        }
        else
        {
#if ENABLE_POST_CFE_CHANGES
          cu.licFlag = affineMergeCtx.LICFlags[cu.mergeIdx] ^ cu.oppositeLicFlag;
#endif
          auto &mvField = affineMergeCtx.mvFieldNeighbours[cu.mergeIdx];
          for (const auto l: { RPL0, RPL1 })
          {
            if (cu.cs->slice->m_numRefIdx[l] > 0)
            {
              cu.mvpIdx[l]    = 0;
              cu.mvpNum[l]    = 0;
              cu.mvd[l]       = Mv();
              cu.refIdx[l]    = mvField[0][l].refIdx;
              cu.mvAffi[l][0] = mvField[0][l].mv;
              cu.mvAffi[l][1] = mvField[1][l].mv;
              cu.mvAffi[l][2] = mvField[2][l].mv;
            }
          }
          if (PU::checkBDMVRCondition(cu))
          {
            cu.bdmvrRefine = false;
            if (!affineMergeCtx.xCheckSimilarMotion(cu.mergeIdx, PU::getBDMVRMvdThreshold(cu)))
            {
              if (PU::checkBDMVR4Affine(cu))
              {
                m_pcInterPred->processBDMVR4Affine(cu);
              }
              mvField[0][RPL0].mv = cu.mvAffi[RPL0][0];
              mvField[1][RPL0].mv = cu.mvAffi[RPL0][1];
              mvField[2][RPL0].mv = cu.mvAffi[RPL0][2];
              mvField[0][RPL1].mv = cu.mvAffi[RPL1][0];
              mvField[1][RPL1].mv = cu.mvAffi[RPL1][1];
              mvField[2][RPL1].mv = cu.mvAffi[RPL1][2];
            }
          }
          for (const auto l: { RPL0, RPL1 })
          {
            if (cu.cs->slice->m_numRefIdx[l] > 0)
            {
              PU::setAllAffineMvField(cu, mvField, l);
            }
          }
        }
        PU::spanMotionInfo(cu, &affineMergeCtx);
      }
      else
      {
        if (CU::isIBC(cu))
        {
          PU::getIBCMergeCandidates(cu, mrgCtx, cu.mergeIdx);
        }
        else if (cu.bmMergeFlag)
        {
          PU::getInterBMCandidates(cu, mrgCtx, cu.mergeIdx);
        }
        else
        {
          PU::getInterMergeCandidates(cu, mrgCtx, 0, cu.mergeIdx, 1);
        }

        mrgCtx.setMergeInfo(cu, cu.mergeIdx);

        if (PU::checkBDMVRCondition(cu))
        {
          m_pcInterPred->setBdmvrSubPuMvBuf(m_mvBufBDMVR[0], m_mvBufBDMVR[1]);
          cu.bdmvrRefine = true;

          CHECK(mrgCtx.numValidMergeCand <= 0, "this is not possible");

          if (!cu.bmMergeFlag && mrgCtx.checkSimilarMotion(cu.mergeIdx, PU::getBDMVRMvdThreshold(cu)))
          {
            // span motion to subPU
            for (int subPuIdx = 0; subPuIdx < MAX_NUM_SUBCU_DMVR; subPuIdx++)
            {
              m_mvBufBDMVR[0][subPuIdx] = cu.mv[0];
              m_mvBufBDMVR[1][subPuIdx] = cu.mv[1];
            }
          }
          else
          {
            cu.bdmvrRefine = m_pcInterPred->processBDMVR(cu);
          }
        }

        if (!cu.bdmvrRefine)
        {
          // BDOF MV offsets are only calculated during reconstruction
          PU::spanMotionInfo(cu);
        }
      }
    }
    else
    {
      if (CU::isIBC(cu) && cu.interDir == 1)
      {
        AMVPInfo amvpInfo;
        PU::fillIBCMvpCand(cu, amvpInfo);
        cu.mvpNum[RPL0] = amvpInfo.numCand;
        Mv &mvd         = cu.mvd[RPL0];
#if REUSE_CU_RESULTS
        if (!cu.cs->pcv->isEncoder)
#endif
        {
          mvd.changeIbcPrecAmvr2Internal(cu.imv);
        }
        if (cu.cs->sps->m_maxNumIBCMergeCand == 1)
        {
          CHECK(cu.mvpIdx[RPL0], "mvpIdx for IBC mode should be 0");
        }
        cu.mv[RPL0] = amvpInfo.mvCand[cu.mvpIdx[RPL0]] + mvd;
        cu.mv[RPL0].foldToStorageBitDepth();
        PU::spanMotionInfo(cu);
      }
      else

#if REUSE_CU_RESULTS
        if (cu.imv && !cu.affine && !cu.cs->pcv->isEncoder)
#else
        if (cu.imv && !cu.affine)
#endif
      {
        PU::applyImv(cu, m_pcInterPred);
      }
      else
      {
        if (cu.affine)
        {
          for (uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++)
          {
            RefPicList eRefList = RefPicList(uiRefListIdx);
            if (cu.cs->slice->m_numRefIdx[eRefList] > 0 && (cu.interDir & (1 << uiRefListIdx)))
            {
              AffineAMVPInfo affineAMVPInfo;
              PU::fillAffineMvpCand(cu, eRefList, cu.refIdx[eRefList], affineAMVPInfo);

              const unsigned mvpIdx = cu.mvpIdx[eRefList];

              cu.mvpNum[eRefList] = affineAMVPInfo.numCand;

              //    Mv mv[3];
              CHECK(cu.refIdx[eRefList] < 0, "Unexpected negative refIdx.");
              if (!cu.cs->pcv->isEncoder)
              {
                for (int i = 0; i < cu.getNumAffineMvs(); i++)
                {
                  cu.mvdAffi[eRefList][i].changeAffinePrecAmvr2Internal(cu.imv);
                }
              }

              Mv mvLT = affineAMVPInfo.mvCandLT[mvpIdx] + cu.mvdAffi[eRefList][0];
              Mv mvRT = affineAMVPInfo.mvCandRT[mvpIdx] + cu.mvdAffi[eRefList][1];
              mvRT += cu.mvdAffi[eRefList][0];

              Mv mvLB;
              if (cu.affineType == AffineModel::_6_PARAMS)
              {
                mvLB = affineAMVPInfo.mvCandLB[mvpIdx] + cu.mvdAffi[eRefList][2];
                mvLB += cu.mvdAffi[eRefList][0];
              }
              PU::setAllAffineMv(cu, mvLT, mvRT, mvLB, eRefList, true);
            }
          }
        }
        else
        {
          for (uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++)
          {
            RefPicList eRefList = RefPicList(uiRefListIdx);
            if ((cu.cs->slice->m_numRefIdx[eRefList] > 0 || (eRefList == RPL0 && CU::isIBC(cu))) &&
                (cu.interDir & (1 << uiRefListIdx)))
            {
              AMVPInfo amvpInfo;
              PU::fillMvpCand(cu, eRefList, cu.refIdx[eRefList], amvpInfo);
              cu.mvpNum[eRefList] = amvpInfo.numCand;
              if (!cu.cs->pcv->isEncoder)
              {
                cu.mvd[eRefList].changeTransPrecAmvr2Internal(cu.imv);
              }
              cu.mv[eRefList] = amvpInfo.mvCand[cu.mvpIdx[eRefList]] + cu.mvd[eRefList];
              cu.mv[eRefList].foldToStorageBitDepth();
            }
          }
        }
        PU::spanMotionInfo(cu);
      }
    }
    if (!cu.geoFlag)
    {
      if (g_mctsDecCheckEnabled && !MCTSHelper::checkMvBufferForMCTSConstraint(cu, true))
      {
        printf("DECODER: cu motion vector across tile boundaries (%d,%d,%d,%d)\n", cu.lx(), cu.ly(), cu.lwidth(),
               cu.lheight());
      }
    }
    if (CU::isIBC(cu))
    {
      const int          cuPelX    = cu.lx();
      const int          cuPelY    = cu.ly();
      int                roiWidth  = cu.lwidth();
      int                roiHeight = cu.lheight();
      const unsigned int lcuWidth  = cu.cs->slice->m_sps->m_maxCuWidth;
      int                xPred     = cu.mv[0].getHor() >> MV_FRACTIONAL_BITS_INTERNAL;
      int                yPred     = cu.mv[0].getVer() >> MV_FRACTIONAL_BITS_INTERNAL;
      CHECK(!m_pcInterPred->isLumaBvValid(lcuWidth, cuPelX, cuPelY, roiWidth, roiHeight, xPred, yPred),
            "invalid block vector for IBC detected.");
    }
  }
}
//! \}
