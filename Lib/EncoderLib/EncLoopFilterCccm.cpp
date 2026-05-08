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

#include "EncLoopFilterCccm.h"

void EncLoopFilterCccm::lfCccmRDO(CodingStructure &cs, const PelUnitBuf recSAO, PelUnitBuf recYuv, CtxPool *ctxCache,
                                  CABACEncoder *cabacEncoder, Slice *pcSlice)
{

  m_ctxCache       = ctxCache;
  m_CABACEstimator = cabacEncoder->getCABACEstimator(pcSlice->m_sps);
  m_CABACEstimator->initCtxModels(*pcSlice);
  m_CABACEstimator->resetBits();

  lfCccmAllocateArraysEncoder(cs.pcv->maxCUWidth, cs.pcv->maxCUHeight);
  lfCccmCreatePelStorage(cs);

  cs.slice->lfCccmClearControlInformation();

  PelStorage localPelStorage;
  // note was chroma-only 4:2:0 in ECM
  localPelStorage.create(ChromaFormat::_420, Area(0, 0, cs.picture->lwidth(), cs.picture->lheight()));

  const PreCalcValues pcv = *cs.pcv;

  PelStorage recYuvInheritStorage;

  const bool doFrameLevelInherit = !cs.slice->isIntra() && cs.slice->lfCccmGetReferencePicture();

  // collect distortions
  RdCost rdcost;
  struct lfCccmEncCandidate
  {
    int      ctuRsAddr;
    int      uselfCccm;
    int      windowSize;
    int      modelType;
    int      ctuMerge;
    int      frameLevelInherit;
    uint64_t distCb;
    uint64_t distCr;
    CPelBuf  bufCb;
    CPelBuf  bufCr;

    bool isEqualCand(const lfCccmEncCandidate &test)
    {
      if (test.uselfCccm != uselfCccm)
      {
        return false;
      }
      if (test.windowSize != windowSize)
      {
        return false;
      }
      if (test.modelType != modelType)
      {
        return false;
      }
      if (test.ctuRsAddr != ctuRsAddr)
      {
        return false;
      }
      return true;
    };
  };

  const TempCtx ctxStartlfCccm1(m_ctxCache, SubCtx(Ctx::LfCccmFlag, m_CABACEstimator->getCtx()));
  auto          resetCtx1 = [&]() { m_CABACEstimator->getCtx() = SubCtx(Ctx::LfCccmFlag, ctxStartlfCccm1); };

  cs.slice->m_lfCccmEnabledFlag = true;

  const double lambda = (cs.slice->getLambdas()[COMP_Cb] + cs.slice->getLambdas()[COMP_Cr]) / 4.0;

  double frameLevelInheritCost = MAX_DOUBLE;
  if (doFrameLevelInherit)
  {
    cs.slice->m_lfCccmFrameLevelInherit = 1;
    m_CABACEstimator->resetBits();
    m_CABACEstimator->lfCccm(cs, 0);
    frameLevelInheritCost               = lambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
    cs.slice->m_lfCccmFrameLevelInherit = 0;
    resetCtx1();

    // note was chroma-only 4:2:0 in ECM
    recYuvInheritStorage.create(ChromaFormat::_420, Area(0, 0, cs.picture->lwidth(), cs.picture->lheight()));
    recYuvInheritStorage.copyFrom(recYuv, false, true);
  }

  double totalCost    = 0;
  double totalCostOff = 0;
  for (int ctuRsAddr = 0; ctuRsAddr < cs.picture->m_ctuNums; ctuRsAddr++)
  {

    bool ctuProcessedEnc = false;

    const int      xc = ctuRsAddr % pcv.widthInCtus;
    const int      yc = ctuRsAddr / pcv.widthInCtus;
    const UnitArea ctuArea =
      clipArea(UnitArea(pcv.chrFormat,
                        Area(xc << pcv.maxCUWidthLog2, yc << pcv.maxCUHeightLog2, pcv.maxCUWidth, pcv.maxCUWidth)),
               *cs.slice->m_pic);
    UnitArea bufArea = ctuArea;
    bufArea.repositionTo(UnitArea(pcv.chrFormat, Area(0, 0, bufArea.lwidth(), bufArea.lheight())));

    CPelBuf origCb = cs.picture->getTrueOrigBuf(ctuArea.Cb());
    CPelBuf origCr = cs.picture->getTrueOrigBuf(ctuArea.Cr());

    CPelBuf saoCb = recSAO.subBuf(ctuArea).Cb();
    CPelBuf saoCr = recSAO.subBuf(ctuArea).Cr();

    std::vector<lfCccmEncCandidate> rdoCandidates;

    // initial distortion
    {
      lfCccmEncCandidate curCand;
      curCand.ctuRsAddr         = ctuRsAddr;
      curCand.uselfCccm         = 0;
      curCand.windowSize        = 0;
      curCand.modelType         = 0;
      curCand.ctuMerge          = 0;
      curCand.frameLevelInherit = 0;
      curCand.distCb = rdcost.getDistPart(origCb, saoCb, cs.sps->m_bitDepths[ChannelType::CHROMA], COMP_Y, DFunc::SSE);
      curCand.distCr = rdcost.getDistPart(origCr, saoCr, cs.sps->m_bitDepths[ChannelType::CHROMA], COMP_Y, DFunc::SSE);
      curCand.bufCb  = recSAO.subBuf(ctuArea).Cb();
      curCand.bufCr  = recSAO.subBuf(ctuArea).Cr();

      rdoCandidates.push_back(curCand);
    }
    // regular candidates (full set)
    {
      cs.slice->lfCccmClearControlInformation(ctuRsAddr);
      for (int windowSizeIndx = 0; windowSizeIndx < m_lfCccmMaxNumWindows; windowSizeIndx++)
      {
        for (int modelTypeIndx = 0; modelTypeIndx < m_lfCccmMaxNumModels; modelTypeIndx++)
        {
          cs.slice->m_lfCccmEnabled.at(ctuRsAddr)      = 1;
          cs.slice->m_lfCccmWindowSizeIndex[ctuRsAddr] = windowSizeIndx;
          cs.slice->m_lfCccmModelType[ctuRsAddr]       = modelTypeIndx;

          localPelStorage.subBuf(ctuArea).Cb().copyFrom(saoCb);
          localPelStorage.subBuf(ctuArea).Cr().copyFrom(saoCr);

          lfCccmCtuProcess(cs, recSAO, ctuRsAddr, localPelStorage.Cb(), localPelStorage.Cr(),
                           &ctuProcessedEnc); // very awkward use of localPelStorage

          m_lfCccmOutputsEncoder.at(windowSizeIndx)
            .at(modelTypeIndx)
            .subBuf(bufArea)
            .Cb()
            .copyFrom(localPelStorage.subBuf(ctuArea).Cb()); // awkward
          m_lfCccmOutputsEncoder.at(windowSizeIndx)
            .at(modelTypeIndx)
            .subBuf(bufArea)
            .Cr()
            .copyFrom(localPelStorage.subBuf(ctuArea).Cr());

          lfCccmEncCandidate curCand;
          curCand.ctuRsAddr         = ctuRsAddr;
          curCand.uselfCccm         = 1;
          curCand.windowSize        = cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr);
          curCand.modelType         = cs.slice->m_lfCccmModelType.at(ctuRsAddr);
          curCand.ctuMerge          = cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr);
          curCand.frameLevelInherit = 0;
          curCand.bufCb             = m_lfCccmOutputsEncoder.at(windowSizeIndx).at(modelTypeIndx).subBuf(bufArea).Cb();
          curCand.bufCr             = m_lfCccmOutputsEncoder.at(windowSizeIndx).at(modelTypeIndx).subBuf(bufArea).Cr();
          curCand.distCb =
            rdcost.getDistPart(origCb, curCand.bufCb, cs.sps->m_bitDepths[ChannelType::CHROMA], COMP_Y, DFunc::SSE);
          curCand.distCr =
            rdcost.getDistPart(origCr, curCand.bufCr, cs.sps->m_bitDepths[ChannelType::CHROMA], COMP_Y, DFunc::SSE);
          rdoCandidates.push_back(curCand);
        }
      }
    }
    const int numCandRegular = int(rdoCandidates.size());
    // merge candidates distortion
    {
      const int nCand = (int)cs.slice->lfCccmGetMergeCandidates(ctuRsAddr).size();
      for (int cand = 0; cand < nCand; cand++)
      {
        cs.slice->m_lfCccmCTUMerge[ctuRsAddr] = 1 + 2 * cand;
        cs.slice->lfCccmMerge(ctuRsAddr);

        lfCccmEncCandidate curCand;

        curCand.ctuRsAddr         = ctuRsAddr;
        curCand.uselfCccm         = 1;
        curCand.windowSize        = cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr);
        curCand.modelType         = cs.slice->m_lfCccmModelType.at(ctuRsAddr);
        curCand.ctuMerge          = cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr);
        curCand.frameLevelInherit = 0;

        for (int i = 0; i < numCandRegular; i++)
        {
          if (curCand.isEqualCand(rdoCandidates.at(i)))
          {
            curCand.distCb = rdoCandidates.at(i).distCb;
            curCand.distCr = rdoCandidates.at(i).distCr;
            curCand.bufCb  = rdoCandidates.at(i).bufCb;
            curCand.bufCr  = rdoCandidates.at(i).bufCr;
            rdoCandidates.push_back(curCand);
            break;
          }
        }
      }
    }
    // frame-level inherit
    {
      if (doFrameLevelInherit)
      {
        cs.slice->m_lfCccmFrameLevelInherit = 1;
        lfCccmSetFrameLevelInheritedParameters(cs, ctuRsAddr);
        cs.slice->m_lfCccmFrameLevelInherit = 0;

        lfCccmEncCandidate curCand;
        curCand.ctuRsAddr         = ctuRsAddr;
        curCand.uselfCccm         = cs.slice->m_lfCccmEnabled.at(ctuRsAddr);
        curCand.windowSize        = cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr);
        curCand.modelType         = cs.slice->m_lfCccmModelType.at(ctuRsAddr);
        curCand.ctuMerge          = cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr);
        curCand.frameLevelInherit = 1;

        for (int i = 0; i < numCandRegular; i++)
        {
          if (curCand.isEqualCand(rdoCandidates.at(i)))
          {
            curCand.distCb = rdoCandidates.at(i).distCb;
            curCand.distCr = rdoCandidates.at(i).distCr;
            curCand.bufCb  = rdoCandidates.at(i).bufCb;
            curCand.bufCr  = rdoCandidates.at(i).bufCr;
            rdoCandidates.push_back(curCand);
            break;
          }
        }
      }
    }
    const TempCtx ctxStartlfCccm2(m_ctxCache, SubCtx(Ctx::LfCccmFlag, m_CABACEstimator->getCtx()));
    auto          resetCtx2 = [&]() { m_CABACEstimator->getCtx() = SubCtx(Ctx::LfCccmFlag, ctxStartlfCccm2); };

    double             bestCost = MAX_DOUBLE;
    lfCccmEncCandidate bestCand;

    for (auto curCand: rdoCandidates)
    {
      cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr) = curCand.windowSize;
      cs.slice->m_lfCccmModelType.at(ctuRsAddr)       = curCand.modelType;
      cs.slice->m_lfCccmEnabled.at(ctuRsAddr)         = curCand.uselfCccm;
      cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr)        = curCand.ctuMerge;

      const double curDist = (curCand.distCb + curCand.distCr) / 2.0;

      if (curCand.frameLevelInherit == 1)
      {
        frameLevelInheritCost += curDist;
        recYuvInheritStorage.subBuf(ctuArea).Cb().copyFrom(curCand.bufCb);
        recYuvInheritStorage.subBuf(ctuArea).Cr().copyFrom(curCand.bufCr);
        continue;
      }

      resetCtx2();
      m_CABACEstimator->resetBits();
      m_CABACEstimator->lfCccm(cs, ctuRsAddr);
      const double curCost = lambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits() + curDist;

      if (!curCand.uselfCccm && !curCand.frameLevelInherit)
      {
        totalCostOff += curDist;
      }

      if (curCost < bestCost)
      {
        bestCost = curCost;
        bestCand = curCand;
      }
    }
    recYuv.subBuf(ctuArea).Cb().copyFrom(bestCand.bufCb);
    recYuv.subBuf(ctuArea).Cr().copyFrom(bestCand.bufCr);
    cs.slice->m_lfCccmWindowSizeIndex.at(ctuRsAddr) = bestCand.windowSize;
    cs.slice->m_lfCccmModelType.at(ctuRsAddr)       = bestCand.modelType;
    cs.slice->m_lfCccmEnabled.at(ctuRsAddr)         = bestCand.uselfCccm;
    cs.slice->m_lfCccmCTUMerge.at(ctuRsAddr)        = bestCand.ctuMerge;

    resetCtx2();
    m_CABACEstimator->lfCccm(cs, ctuRsAddr);
    totalCost += bestCost;
  }
  if (doFrameLevelInherit && (frameLevelInheritCost < totalCost))
  {
    cs.slice->m_lfCccmFrameLevelInherit = 1;
    lfCccmSetFrameLevelInheritedParameters(cs);
    recYuv.copyFrom(recYuvInheritStorage, false, true);
    totalCost = frameLevelInheritCost;
  }
  if (totalCostOff < totalCost)
  {
    cs.slice->lfCccmClearControlInformation();
    cs.slice->m_lfCccmEnabledFlag = false;
    recYuv.copyFrom(cs.getRecoBuf(), false, true);
    totalCost = totalCostOff;
  }
  resetCtx1();
  lfCccmDeallocateArraysEncoder();
}
