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

/** \file     EncAdaptiveLoopFilter.cpp
 \brief    estimation part of adaptive loop filter class
 */
#include "EncAdaptiveLoopFilterVtm.h"
#include "EncCfg.h"

#include "CommonLib/Picture.h"
#include "CommonLib/CodingStructure.h"

#define AlfCtx(c) SubCtx(Ctx::Alf, c)

#include <algorithm>

#include "EncAdaptiveLoopFilterBaseDefs.h"

EncAdaptiveLoopFilterVtm::EncAdaptiveLoopFilterVtm()
  : EncAdaptiveLoopFilterBase<AlfParametersVtm>()
  , AdaptiveLoopFilterVtm()
{
  for (auto &covCompBuf: m_alfCovariance)
  {
    covCompBuf.clear();
  }
  for (auto &covFrameChBuf: m_alfCovarianceFrame)
  {
    covFrameChBuf.clear();
  }

  m_alfCovarianceCcAlf.clear();
  m_alfCovarianceFrameCcAlf.clear();
}

void EncAdaptiveLoopFilterVtm::create(const EncCfg *encCfg, const int picWidth, const int picHeight,
                                      const ChromaFormat chromaFormatIdc, const int maxCUWidth, const int maxCUHeight,
                                      const int maxCUDepth, const BitDepths &inputBitDepth)
{
  AdaptiveLoopFilterVtm::create(picWidth, picHeight, chromaFormatIdc, maxCUWidth, maxCUHeight, maxCUDepth,
                                inputBitDepth);
  EncAdaptiveLoopFilter::createSharedEncMembers(encCfg);

  const int numBinsLuma   = m_encCfg->m_useNonLinearAlfLuma ? MAX_ALF_NUM_CLIP_VALS : 1;
  const int numBinsChroma = m_encCfg->m_useNonLinearAlfChroma ? MAX_ALF_NUM_CLIP_VALS : 1;

  for (const auto chType: { ChannelType::LUMA, ChannelType::CHROMA })
  {
    const int numClasses = isLuma(chType) ? MAX_NUM_ALF_CLASSES : 1;

    m_alfCovarianceFrame[chType].resize(m_filterShapes[chType].size());
    for (int i = 0; i != m_filterShapes[chType].size(); i++)
    {
      m_alfCovarianceFrame[chType][i].resize(numClasses);
      for (int k = 0; k < numClasses; k++)
      {
        m_alfCovarianceFrame[chType][i][k].create(m_filterShapes[chType][i].numCoeff,
                                                  isLuma(chType) ? numBinsLuma : numBinsChroma);
      }
    }
  }

  for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
  {
    ChannelType chType     = toChannelType(CompID(compIdx));
    int         numClasses = compIdx ? 1 : MAX_NUM_ALF_CLASSES;

    m_alfCovariance[compIdx].resize(m_filterShapes[chType].size());
    for (int i = 0; i != m_filterShapes[chType].size(); i++)
    {
      m_alfCovariance[compIdx][i].resize(m_numCTUsInPic);
      for (int j = 0; j < m_numCTUsInPic; j++)
      {
        m_alfCovariance[compIdx][i][j].resize(numClasses);
        for (int k = 0; k < numClasses; k++)
        {
          m_alfCovariance[compIdx][i][j][k].create(m_filterShapes[chType][i].numCoeff,
                                                   isLuma(chType) ? numBinsLuma : numBinsChroma);
        }
      }
    }
  }

  for (int i = 0; i != m_filterShapes[ChannelType::LUMA].size(); i++)
  {
    for (int j = 0; j <= MAX_NUM_ALF_CLASSES + 1; j++)
    {
      m_alfCovarianceMerged[i][j].create(m_filterShapes[ChannelType::LUMA][i].numCoeff, numBinsLuma);
    }
  }

  m_filterCoeffSet = new int *[std::max(MAX_NUM_ALF_CLASSES, ALF_MAX_NUM_ALTERNATIVES_CHROMA)];
  m_filterClippSet = new int *[std::max(MAX_NUM_ALF_CLASSES, ALF_MAX_NUM_ALTERNATIVES_CHROMA)];
  for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
  {
    m_filterCoeffSet[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
    m_filterClippSet[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
  }

  for (int i = 0; i < ALF_NUM_FIXED_FILTER_SETS; ++i)
  {
    m_ctbDistortionFixedFilter[i] = new float[m_numCTUsInPic];
  }

  for (int i = 0; i < ALF_CTB_MAX_NUM_APS; ++i)
  {
    m_distCtbApsLuma[i] = new float[m_numCTUsInPic];
  }

  m_distCtbLumaNewFilt = new float[m_numCTUsInPic];

  memset(m_clipDefaultEnc, 0, sizeof(m_clipDefaultEnc));

  m_alfCovarianceCcAlf.resize(m_filterShapesCcAlf.size());
  m_alfCovarianceFrameCcAlf.resize(m_filterShapesCcAlf.size());
  for (int i = 0; i < m_filterShapesCcAlf.size(); ++i)
  {
    m_alfCovarianceFrameCcAlf[i].create(m_filterShapesCcAlf[i].numCoeff, 1);
    m_alfCovarianceCcAlf[i].resize(m_numCTUsInPic);
    for (int j = 0; j < m_numCTUsInPic; ++j)
    {
      m_alfCovarianceCcAlf[i][j].create(m_filterShapesCcAlf[i].numCoeff, 1);
    }
  }

  for (int i = 0; i < MAX_NUM_CC_ALF_FILTERS; i++)
  {
    m_trainingDistortion[i] = new uint64_t[m_numCTUsInPic];
  }
}

void EncAdaptiveLoopFilterVtm::destroy()
{
  if (!m_created)
  {
    return;
  }

  for (auto &covFrameChBuf: m_alfCovarianceFrame)
  {
    covFrameChBuf.clear();
  }

  for (auto &covCompBuf: m_alfCovariance)
  {
    covCompBuf.clear();
  }

  if (m_filterCoeffSet)
  {
    for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
    {
      delete[] m_filterCoeffSet[i];
      m_filterCoeffSet[i] = nullptr;
    }
    delete[] m_filterCoeffSet;
    m_filterCoeffSet = nullptr;
  }

  if (m_filterClippSet)
  {
    for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
    {
      delete[] m_filterClippSet[i];
      m_filterClippSet[i] = nullptr;
    }
    delete[] m_filterClippSet;
    m_filterClippSet = nullptr;
  }

  for (int i = 0; i < ALF_NUM_FIXED_FILTER_SETS; ++i)
  {
    if (m_ctbDistortionFixedFilter[i])
    {
      delete[] m_ctbDistortionFixedFilter[i];
      m_ctbDistortionFixedFilter[i] = nullptr;
    }
  }

  for (int i = 0; i < ALF_CTB_MAX_NUM_APS; ++i)
  {
    if (m_distCtbApsLuma[i])
    {
      delete[] m_distCtbApsLuma[i];
      m_distCtbApsLuma[i] = nullptr;
    }
  }

  if (m_distCtbLumaNewFilt)
  {
    delete[] m_distCtbLumaNewFilt;
    m_distCtbLumaNewFilt = nullptr;
  }

  m_alfCovarianceFrameCcAlf.clear();
  m_alfCovarianceCcAlf.clear();

  for (int i = 0; i < MAX_NUM_CC_ALF_FILTERS; i++)
  {
    if (m_trainingDistortion[i])
    {
      delete[] m_trainingDistortion[i];
      m_trainingDistortion[i] = nullptr;
    }
  }

  EncAdaptiveLoopFilter::destroySharedEncMembers();
  AdaptiveLoopFilterVtm::destroy();
}

void EncAdaptiveLoopFilterVtm::xSetupCcAlfAPS(const CodingStructure &cs)
{
  if (m_ccAlfFilterParam.ccAlfFilterEnabled[COMP_Cb - 1])
  {
    int  ccAlfCbApsId = cs.slice->m_ccAlfCbApsId;
    APS *aps          = m_apsMap->getPS(cs.slice->m_ccAlfCbApsId);
    if (aps == nullptr)
    {
      aps               = m_apsMap->allocatePS(ccAlfCbApsId);
      aps->m_temporalId = cs.slice->m_uiTLayer;
    }

    CcAlfFilterParam &ccAlfAPSParam = aps->m_ccAlfAPSParam.getVtmParam();

    ccAlfAPSParam.ccAlfFilterEnabled[COMP_Cb - 1] = 1;
    ccAlfAPSParam.ccAlfFilterCount[COMP_Cb - 1]   = m_ccAlfFilterParam.ccAlfFilterCount[COMP_Cb - 1];
    for (int filterIdx = 0; filterIdx < MAX_NUM_CC_ALF_FILTERS; filterIdx++)
    {
      ccAlfAPSParam.ccAlfFilterIdxEnabled[COMP_Cb - 1][filterIdx] =
        m_ccAlfFilterParam.ccAlfFilterIdxEnabled[COMP_Cb - 1][filterIdx];
      std::copy_n(m_ccAlfFilterParam.ccAlfCoeff[COMP_Cb - 1][filterIdx], MAX_NUM_CC_ALF_CHROMA_COEFF,
                  ccAlfAPSParam.ccAlfCoeff[COMP_Cb - 1][filterIdx]);
    }
    aps->m_APSId   = ccAlfCbApsId;
    aps->m_APSType = ApsType::ALF;
    if (m_reuseApsId[COMP_Cb - 1] < 0)
    {
      ccAlfAPSParam.newCcAlfFilter[COMP_Cb - 1] = 1;
      m_apsMap->setChangedFlag(ccAlfCbApsId, true);
      aps->m_temporalId = cs.slice->m_uiTLayer;
    }
    cs.slice->m_ccAlfCbEnabledFlag = true;
  }
  else
  {
    cs.slice->m_ccAlfCbEnabledFlag = false;
  }

  if (m_ccAlfFilterParam.ccAlfFilterEnabled[COMP_Cr - 1])
  {
    int  ccAlfCrApsId = cs.slice->m_ccAlfCrApsId;
    APS *aps          = m_apsMap->getPS(cs.slice->m_ccAlfCrApsId);
    if (aps == nullptr)
    {
      aps               = m_apsMap->allocatePS(ccAlfCrApsId);
      aps->m_temporalId = cs.slice->m_uiTLayer;
    }

    CcAlfFilterParam &ccAlfAPSParam = aps->m_ccAlfAPSParam.getVtmParam();

    ccAlfAPSParam.ccAlfFilterEnabled[COMP_Cr - 1] = 1;
    ccAlfAPSParam.ccAlfFilterCount[COMP_Cr - 1]   = m_ccAlfFilterParam.ccAlfFilterCount[COMP_Cr - 1];
    for (int filterIdx = 0; filterIdx < MAX_NUM_CC_ALF_FILTERS; filterIdx++)
    {
      ccAlfAPSParam.ccAlfFilterIdxEnabled[COMP_Cr - 1][filterIdx] =
        m_ccAlfFilterParam.ccAlfFilterIdxEnabled[COMP_Cr - 1][filterIdx];
      std::copy_n(m_ccAlfFilterParam.ccAlfCoeff[COMP_Cr - 1][filterIdx], MAX_NUM_CC_ALF_CHROMA_COEFF,
                  ccAlfAPSParam.ccAlfCoeff[COMP_Cr - 1][filterIdx]);
    }
    aps->m_APSId = ccAlfCrApsId;
    if (m_reuseApsId[COMP_Cr - 1] < 0)
    {
      ccAlfAPSParam.newCcAlfFilter[COMP_Cr - 1] = 1;
      m_apsMap->setChangedFlag(ccAlfCrApsId, true);
      aps->m_temporalId = cs.slice->m_uiTLayer;
    }
    aps->m_APSType                 = ApsType::ALF;
    cs.slice->m_ccAlfCrEnabledFlag = true;
  }
  else
  {
    cs.slice->m_ccAlfCrEnabledFlag = false;
  }
}

void EncAdaptiveLoopFilterVtm::ALFProcess(CodingStructure &cs, const double *lambdas,
#if ENABLE_QPA
                                          const double lambdaChromaWeight,
#endif
                                          Picture *pic, uint32_t numSliceSegments)
{
  PROFILER_SCOPE(1, g_timeProfiler, P_ALF);
  // IRAP AU is assumed
  if ((cs.slice->m_pendingRasInit || cs.slice->isIDRorBLA() ||
       (cs.slice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA && m_encCfg->m_craAPSreset)))
  {
    memset(cs.slice->m_alfApss, 0, sizeof(*cs.slice->m_alfApss) * ALF_CTB_MAX_NUM_APS);
    m_apsIdStart = m_encCfg->m_alfapsIDShift + m_encCfg->m_maxNumAlfAps;

    m_apsMap->clearActive();
    for (int i = m_encCfg->m_alfapsIDShift; i < m_encCfg->m_alfapsIDShift + m_encCfg->m_maxNumAlfAps; i++)
    {
      APS *alfAPS = m_apsMap->getPS(i);
      m_apsMap->clearChangedFlag(i);
      if (alfAPS)
      {
        alfAPS->m_alfAPSParam.getVtmParam().reset();
        alfAPS->m_ccAlfAPSParam.getVtmParam().reset();
        alfAPS = nullptr;
      }
    }
  }

  AlfParam alfParam;
  alfParam.reset();
  const TempCtx ctxStart(m_ctxPool, AlfCtx(m_CABACEstimator->getCtx()));

  const TempCtx ctxStartCcAlf(m_ctxPool, SubCtx(Ctx::CcAlfFilterControlFlag, m_CABACEstimator->getCtx()));

  // set available filter shapes
  alfParam.filterShapes = &m_filterShapes;

  // set clipping range
  m_clpRngs = cs.slice->m_clpRngs;

  // set CTU ALF enable flags, it was already reset before ALF process
  CHECKD(cs.picture->getAlfModes(CompID::COMP_Y).size() != m_numCTUsInPic,
         "Unexpected number of CTB mode entries (Y).");
  CHECKD(cs.picture->getAlfModes(CompID::COMP_Cb).size() != m_numCTUsInPic,
         "Unexpected number of CTB mode entries (Cb).");
  CHECKD(cs.picture->getAlfModes(CompID::COMP_Cr).size() != m_numCTUsInPic,
         "Unexpected number of CTB mode entries (Cr).");
  m_modes = &cs.picture->getAlfModes();

  // reset ALF parameters
  alfParam.reset();
  int shiftLuma     = 2 * DISTORTION_PRECISION_ADJUSTMENT(m_inputBitDepth[ChannelType::LUMA]);
  int shiftChroma   = 2 * DISTORTION_PRECISION_ADJUSTMENT(m_inputBitDepth[ChannelType::CHROMA]);
  m_lambda[COMP_Y]  = lambdas[COMP_Y] * double(1 << shiftLuma);
  m_lambda[COMP_Cb] = lambdas[COMP_Cb] * double(1 << shiftChroma);
  m_lambda[COMP_Cr] = lambdas[COMP_Cr] * double(1 << shiftChroma);
  PelUnitBuf orgYuv = m_encCfg->m_alfTrueOrg ? cs.getTrueOrgBuf() : cs.getOrgBuf();

  m_tempBuf.copyFrom(cs.getRecoBuf());
  PelUnitBuf recYuv = m_tempBuf.getBuf(cs.area);
  recYuv.extendBorderPel(MAX_ALF_FILTER_LENGTH >> 1);

  // derive classification
  const CPelBuf       &recLuma = recYuv.get(COMP_Y);
  const PreCalcValues &pcv     = *cs.pcv;
  bool                 clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;

  for (int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
  {
    for (int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
    {
      const int width  = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
      const int height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
      int       rasterSliceAlfPad = 0;
      if (isCrossedBySubPicBoundaries(cs, xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight,
                                      rasterSliceAlfPad))
      {
        const bool clipT = clipTop || (yPos == 0);
        const bool clipB = clipBottom || (yPos + height == pcv.lumaHeight);
        const bool clipL = clipLeft || (xPos == 0);
        const bool clipR = clipRight || (xPos + width == pcv.lumaWidth);
        const int  wBuf  = width + (clipL ? 0 : MAX_ALF_PADDING_SIZE) + (clipR ? 0 : MAX_ALF_PADDING_SIZE);
        const int  hBuf  = height + (clipT ? 0 : MAX_ALF_PADDING_SIZE) + (clipB ? 0 : MAX_ALF_PADDING_SIZE);

        PelUnitBuf buf = m_tempBuf2.subBuf(UnitArea(cs.area.chromaFormat, Area(0, 0, wBuf, hBuf)));
        buf.copyFrom(recYuv.subBuf(UnitArea(
          cs.area.chromaFormat,
          Area(xPos - (clipL ? 0 : MAX_ALF_PADDING_SIZE), yPos - (clipT ? 0 : MAX_ALF_PADDING_SIZE), wBuf, hBuf))));
        // pad top-left unavailable samples for raster slice
        if (rasterSliceAlfPad & 1)
        {
          buf.padBorderPel(MAX_ALF_PADDING_SIZE, 1);
        }

        // pad bottom-right unavailable samples for raster slice
        if (rasterSliceAlfPad & 2)
        {
          buf.padBorderPel(MAX_ALF_PADDING_SIZE, 2);
        }
        buf.extendBorderPel(MAX_ALF_PADDING_SIZE);
        buf =
          buf.subBuf(UnitArea(cs.area.chromaFormat,
                              Area(clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, width, height)));

        const Area blkSrc(0, 0, width, height);
        const Area blkDst(xPos, yPos, width, height);
        deriveClassification(buf.get(COMP_Y), blkDst, blkSrc);
      }
      else
      {
        Area blk(xPos, yPos, width, height);
        deriveClassification(recLuma, blk, blk);
      }
    }
  }

  // get CTB stats for filtering
  if (m_alfWSSD)
  {
    deriveStatsForFiltering<true>(orgYuv, recYuv, cs);
  }
  else
  {
    deriveStatsForFiltering<false>(orgYuv, recYuv, cs);
  }

  CHECKD(cs.slice->m_pic->getAlfModes(COMP_Y).size() != m_numCTUsInPic, "Unexpected number of CTB modes.");
  std::for_each(cs.slice->m_pic->getAlfModes(COMP_Y).begin(), cs.slice->m_pic->getAlfModes(COMP_Y).end(),
                [](int &m) { LumaCtbModeHandler::setApsFilter(m, 0); });
  // consider using new filter (only)
  alfParam.newFilterFlag[ChannelType::LUMA]   = true;
  alfParam.newFilterFlag[ChannelType::CHROMA] = true;
  cs.slice->m_numAlfApsIdsLuma                = 1;   // Only new filter for RD cost optimization
  // derive filter (luma)
  firstPass(cs, alfParam, orgYuv, recYuv, cs.getRecoBuf(), ChannelType::LUMA
#if ENABLE_QPA
            ,
            lambdaChromaWeight
#endif
  );

  // derive filter (chroma)
  if (!(m_encCfg->m_maxNumAlfAps == 0) &&
      isChromaEnabled(cs.pcv->chrFormat))   // Find ALF parameters for chroma if ALF APS is enabled
  {
    firstPass(cs, alfParam, orgYuv, recYuv, cs.getRecoBuf(), ChannelType::CHROMA
#if ENABLE_QPA
              ,
              lambdaChromaWeight
#endif
    );
  }

  // let alfEncoderCtb decide now
  alfParam.newFilterFlag.fill(false);
  cs.slice->m_numAlfApsIdsLuma = 0;
  m_CABACEstimator->getCtx()   = AlfCtx(ctxStart);
  alfEncoderCtb(cs, alfParam
#if ENABLE_QPA
                ,
                lambdaChromaWeight
#endif
  );

  for (int s = 0; s < numSliceSegments; s++)
  {
    if (pic->m_slices[s]->m_isLossless)
    {
      for (uint32_t ctuIdx = 0; ctuIdx < pic->m_slices[s]->getNumCtuInSlice(); ctuIdx++)
      {
        uint32_t ctuRsAddr = pic->m_slices[s]->getCtuAddrInSlice(ctuIdx);
        CtbModeHandler::setDisabled((*m_modes)[COMP_Y][ctuRsAddr]);
        CtbModeHandler::setDisabled((*m_modes)[COMP_Cb][ctuRsAddr]);
        CtbModeHandler::setDisabled((*m_modes)[COMP_Cr][ctuRsAddr]);
      }
    }
  }

  alfReconstructor(cs, recYuv);

  // Do not transmit CC ALF if it is unchanged
  if (cs.slice->m_alfEnabledFlag[COMP_Y])
  {
    for (int32_t lumaAlfApsId: cs.slice->m_alfApsIdsLuma)
    {
      APS *aps = (lumaAlfApsId >= 0) ? m_apsMap->getPS(lumaAlfApsId) : nullptr;
      if (aps && m_apsMap->getChangedFlag(lumaAlfApsId))
      {
        CcAlfFilterParamBase &ccAlfAPSParam = aps->m_ccAlfAPSParam.getParam();

        ccAlfAPSParam.newCcAlfFilter[0] = false;
        ccAlfAPSParam.newCcAlfFilter[1] = false;
      }
    }
  }
  int chromaAlfApsId =
    (cs.slice->m_alfEnabledFlag[COMP_Cb] || cs.slice->m_alfEnabledFlag[COMP_Cr]) ? cs.slice->m_alfApsIdChroma : -1;
  APS *aps = (chromaAlfApsId >= 0) ? m_apsMap->getPS(chromaAlfApsId) : nullptr;
  if (aps && m_apsMap->getChangedFlag(chromaAlfApsId))
  {
    CcAlfFilterParamBase &ccAlfAPSParam = aps->m_ccAlfAPSParam.getParam();

    ccAlfAPSParam.newCcAlfFilter[0] = false;
    ccAlfAPSParam.newCcAlfFilter[1] = false;
  }

  if (!cs.slice->m_sps->m_ccalfEnabledFlag)
  {
    return;
  }

  m_tempBuf.get(COMP_Cb).copyFrom(cs.getRecoBuf().get(COMP_Cb));
  m_tempBuf.get(COMP_Cr).copyFrom(cs.getRecoBuf().get(COMP_Cr));
  recYuv = m_tempBuf.getBuf(cs.area);
  recYuv.extendBorderPel(MAX_ALF_FILTER_LENGTH >> 1);

  m_CABACEstimator->getCtx() = SubCtx(Ctx::CcAlfFilterControlFlag, ctxStartCcAlf);
  deriveCcAlfFilter(cs, COMP_Cb, orgYuv, recYuv, cs.getRecoBuf());
  m_CABACEstimator->getCtx() = SubCtx(Ctx::CcAlfFilterControlFlag, ctxStartCcAlf);
  deriveCcAlfFilter(cs, COMP_Cr, orgYuv, recYuv, cs.getRecoBuf());

  xSetupCcAlfAPS(cs);

  for (int compIdx = 1; compIdx < getNumberValidComponents(cs.pcv->chrFormat); compIdx++)
  {
    CompID compID = CompID(compIdx);
    if (m_ccAlfFilterParam.ccAlfFilterEnabled[compIdx - 1])
    {
      applyCcAlfFilter(cs, compID, cs.getRecoBuf().get(compID), recYuv, m_ccAlfFilterControl[compIdx - 1],
                       m_ccAlfFilterParam.ccAlfCoeff[compIdx - 1], -1);
    }
  }
}

float EncAdaptiveLoopFilterVtm::deriveCtbAlfEnableFlags(CodingStructure &cs, const int shapeIdx, ChannelType channel,
#if ENABLE_QPA
                                                        const double chromaWeight,
#endif
                                                        const int numClasses, const int numCoeff, float &distUnfilter)
{
  TempCtx      ctxTempStart(m_ctxPool);
  TempCtx      ctxTempBest(m_ctxPool);
  TempCtx      ctxTempAltStart(m_ctxPool);
  TempCtx      ctxTempAltBest(m_ctxPool);
  const CompID compIDFirst = isLuma(channel) ? COMP_Y : COMP_Cb;
  const CompID compIDLast  = isLuma(channel) ? COMP_Y : COMP_Cr;
  const int    numAlts     = isLuma(channel) ? 1 : m_alfParamTemp.numAlternativesChroma;

  float cost   = 0;
  distUnfilter = 0;

  setSliceEnabledFlag(m_alfParamTemp, channel, true);
#if ENABLE_QPA
  CHECK((chromaWeight > 0.0) && (cs.slice->getFirstCtuRsAddrInSlice() != 0),
        "incompatible start CTU address, must be 0");
#endif

  if (isLuma(channel))
  {
    reconstructLumaCoeff(m_alfParamTemp, true);
  }
  else
  {
    reconstructChromaCoeff(m_alfParamTemp, true);
  }

  for (int altIdx = 0; altIdx < numAlts; ++altIdx)
  {
    for (int classIdx = 0; classIdx < (isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1); classIdx++)
    {
      const coeff_t *const coeffSrc =
        isLuma(channel) ? &m_coeffFinal[classIdx * MAX_NUM_ALF_LUMA_COEFF] : m_chromaCoeffFinal[altIdx];
      const clip_t *const clippSrc =
        isLuma(channel) ? &m_clippFinal[classIdx * MAX_NUM_ALF_LUMA_COEFF] : m_chromaClippFinal[altIdx];
      int *const coeffDst = m_filterCoeffSet[isLuma(channel) ? classIdx : altIdx];
      int *const clippDst = m_filterClippSet[isLuma(channel) ? classIdx : altIdx];
      for (int i = 0; i < (isLuma(channel) ? MAX_NUM_ALF_LUMA_COEFF : MAX_NUM_ALF_CHROMA_COEFF); i++)
      {
        coeffDst[i] = coeffSrc[i];
        clippDst[i] = clippSrc[i];
      }
    }
  }

  for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++)
  {
    for (int compID = compIDFirst; compID <= compIDLast; compID++)
    {
#if ENABLE_QPA
      const double ctuLambda = chromaWeight > 0.0
        ? (isLuma(channel) ? cs.picture->m_uEnerHpCtu[ctuIdx] : cs.picture->m_uEnerHpCtu[ctuIdx] / chromaWeight)
        : m_lambda[compID];
#else
      const double ctuLambda = m_lambda[compID];
#endif

      const float distUnfilterCtu = m_ctbDistortionUnfilter[compID][ctuIdx];

      ctxTempStart = AlfCtx(m_CABACEstimator->getCtx());
      m_CABACEstimator->resetBits();
      (*m_modes)[compID][ctuIdx] = isLuma(channel) ? CTB_MODE_LUMA_APS0 : CTB_MODE_CHROMA_ALT0;
      m_CABACEstimator->codeAlfCtuEnableFlag(cs, ctuIdx, compID, &m_alfParamTemp);
      if (isLuma(channel))
      {
        // Evaluate cost of signaling filter set index for convergence of filters enabled flag / filter derivation
        assert(cs.slice->m_numAlfApsIdsLuma == 1);
        m_CABACEstimator->codeAlfCtuFilterIndex(cs, ctuIdx, m_alfParamTemp.enabledFlag[COMP_Y]);
      }
      float costOn = distUnfilterCtu + ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();

      ctxTempBest = AlfCtx(m_CABACEstimator->getCtx());
      if (isLuma(channel))
      {
        costOn += getFilteredDistortion(m_alfCovariance[compID][shapeIdx][ctuIdx], numClasses,
                                        m_alfParamTemp.numLumaFilters - 1, numCoeff);
      }
      else
      {
        float bestAltCost = MAX_FLOAT;
        int   bestAltIdx  = -1;
        ctxTempAltStart   = AlfCtx(ctxTempBest);
        for (int altIdx = 0; altIdx < numAlts; ++altIdx)
        {
          if (altIdx)
          {
            m_CABACEstimator->getCtx() = AlfCtx(ctxTempAltStart);
          }
          m_CABACEstimator->resetBits();
          ChromaCtbModeHandler::setAlternative((*m_modes)[compID][ctuIdx], altIdx);
          m_CABACEstimator->codeAlfCtuChromaAlternative(cs, ctuIdx, static_cast<CompID>(compID), m_alfParamTemp);
          float rAltCost = ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();

          float altDist = 0.;
          altDist += m_alfCovariance[compID][shapeIdx][ctuIdx][0].calcErrorForCoeffs(
            m_filterClippSet[altIdx], m_filterCoeffSet[altIdx], numCoeff, 1, COEFF_SCALE_BITS_ALF);

          float altCost = altDist + rAltCost;
          if (altCost < bestAltCost)
          {
            bestAltCost = altCost;
            bestAltIdx  = altIdx;
            ctxTempBest = AlfCtx(m_CABACEstimator->getCtx());
          }
        }
        ChromaCtbModeHandler::setAlternative((*m_modes)[compID][ctuIdx], bestAltIdx);
        costOn += bestAltCost;
      }

      int bestIdxOn              = (*m_modes)[compID][ctuIdx];
      m_CABACEstimator->getCtx() = AlfCtx(ctxTempStart);
      m_CABACEstimator->resetBits();
      CtbModeHandler::setDisabled((*m_modes)[compID][ctuIdx]);
      m_CABACEstimator->codeAlfCtuEnableFlag(cs, ctuIdx, compID, &m_alfParamTemp);
      float costOff = distUnfilterCtu + ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();

      if (costOn < costOff)
      {
        cost += costOn;
        m_CABACEstimator->getCtx() = AlfCtx(ctxTempBest);
        (*m_modes)[compID][ctuIdx] = bestIdxOn;
      }
      else
      {
        cost += costOff;
        distUnfilter += distUnfilterCtu;
      }
    }
  }

  if (isChroma(channel))
  {
    setSliceEnabledFlag(m_alfParamTemp, channel, m_modes);
  }

  return cost;
}

void EncAdaptiveLoopFilterVtm::firstPass(CodingStructure &cs, AlfParam &alfParam, const PelUnitBuf &orgUnitBuf,
                                         const PelUnitBuf &recExtBuf, const PelUnitBuf &recBuf,
                                         const ChannelType channel
#if ENABLE_QPA
                                         ,
                                         const double lambdaChromaWeight   // = 0.0
#endif
)
{
  const TempCtx ctxStart(m_ctxPool, AlfCtx(m_CABACEstimator->getCtx()));
  TempCtx       ctxBest(m_ctxPool);

  float costMin = MAX_FLOAT;

  std::vector<AlfFilterShape> &alfFilterShape = (*alfParam.filterShapes)[channel];
  m_bitsNewFilter[channel]                    = 0;
  const int numClasses                        = isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1;
  int       coeffBits                         = 0;

  for (int shapeIdx = 0; shapeIdx < alfFilterShape.size(); shapeIdx++)
  {
    m_alfParamTemp = alfParam;
    // 1. get unfiltered distortion
    if (isChroma(channel))
    {
      m_alfParamTemp.numAlternativesChroma = 1;
    }

    float cost =
      isLuma(channel) ? m_unFiltDistCompnent[COMP_Y] : m_unFiltDistCompnent[COMP_Cb] + m_unFiltDistCompnent[COMP_Cr];
    cost /= 1.001f;   // slight preference for unfiltered choice

    if (cost < costMin)
    {
      costMin = cost;
      setSliceEnabledFlag(alfParam, channel, false);
      // no CABAC signalling
      ctxBest = AlfCtx(ctxStart);
      setCtuEnableFlag(m_indexTmp, channel, CTB_MODE_OFF);
    }

    // For chroma, nonlinear flag is checked for each alternative filter
    const bool useNonlinearAlf = isLuma(channel) ? m_encCfg->m_useNonLinearAlfLuma : m_encCfg->m_useNonLinearAlfChroma;

    for (bool nonLinearFlag: { false, true })
    {
      if (nonLinearFlag && !useNonlinearAlf)
      {
        continue;
      }

      for (int numAlternatives = isLuma(channel) ? 1 : getMaxNumAlternativesChroma(); numAlternatives > 0;
           numAlternatives--)
      {
        if (isChroma(channel))
        {
          m_alfParamTemp.numAlternativesChroma = numAlternatives;
        }

        // 2. all CTUs are on
        setSliceEnabledFlag(m_alfParamTemp, channel, true);
        (isLuma(channel) ? m_alfParamTemp.lumaNonLinearFlag : m_alfParamTemp.chromaNonLinearFlag) = nonLinearFlag;
        m_CABACEstimator->getCtx()                                                                = AlfCtx(ctxStart);

        // all alternatives are on
        if (isChroma(channel))
        {
          initCtuAlternativeChroma(m_modes);
        }
        else
        {
          setCtuEnableFlag(m_modes, channel, CTB_MODE_LUMA_APS0);
        }

        cost = getFilterCoeffAndCost(cs, 0, channel, true, shapeIdx, coeffBits);

        if (cost < costMin)
        {
          m_bitsNewFilter[channel] = coeffBits;
          costMin                  = cost;
          copyAlfParam(alfParam, m_alfParamTemp, channel);
          ctxBest = AlfCtx(m_CABACEstimator->getCtx());
          copyIndices(m_indexTmp, m_modes, channel);
        }

        // 3. CTU decision
        float     distUnfilter = 0;
        float     prevItCost   = MAX_FLOAT;
        const int iterNum = isLuma(channel) ? (2 * 4 + 1) : (2 * (2 + m_alfParamTemp.numAlternativesChroma - 1) + 1);
        for (int iter = 0; iter < iterNum; iter++)
        {
          if (iter % 2 == 0)
          {
            m_CABACEstimator->getCtx() = AlfCtx(ctxStart);

            cost = m_lambda[isLuma(channel) ? COMP_Y : COMP_Cb] * coeffBits;

            cost += deriveCtbAlfEnableFlags(cs, shapeIdx, channel,
#if ENABLE_QPA
                                            lambdaChromaWeight,
#endif
                                            numClasses, alfFilterShape[shapeIdx].numCoeff, distUnfilter);

            if (cost < costMin)
            {
              m_bitsNewFilter[channel] = coeffBits;
              costMin                  = cost;
              ctxBest                  = AlfCtx(m_CABACEstimator->getCtx());
              copyIndices(m_indexTmp, m_modes, channel);
              copyAlfParam(alfParam, m_alfParamTemp, channel);
            }
            else if (cost >= prevItCost)
            {
              // High probability that we have converged or we are diverging
              break;
            }
            prevItCost = cost;
          }
          else
          {
            // unfiltered distortion is added due to some CTBs may not use filter
            // no need to reset CABAC here, since coeffBits is not affected
            /*cost = */ getFilterCoeffAndCost(cs, distUnfilter, channel, true, shapeIdx, coeffBits);
          }
        }   // for iter
        // Decrease number of alternatives and reset ctu params and filters
      }
    }   // for nonLineaFlag
  }   // for shapeIdx
  m_CABACEstimator->getCtx() = AlfCtx(ctxBest);

  copyIndices(m_modes, m_indexTmp, channel);
}

void EncAdaptiveLoopFilterVtm::copyAlfParam(AlfParam &alfParamDst, AlfParam &alfParamSrc, ChannelType channel)
{
  if (isLuma(channel))
  {
    alfParamDst = alfParamSrc;
  }
  else
  {
    alfParamDst.enabledFlag[COMP_Cb]  = alfParamSrc.enabledFlag[COMP_Cb];
    alfParamDst.enabledFlag[COMP_Cr]  = alfParamSrc.enabledFlag[COMP_Cr];
    alfParamDst.numAlternativesChroma = alfParamSrc.numAlternativesChroma;
    alfParamDst.chromaNonLinearFlag   = alfParamSrc.chromaNonLinearFlag;
    alfParamDst.chromaCoeff           = alfParamSrc.chromaCoeff;
    alfParamDst.chromaClipp           = alfParamSrc.chromaClipp;
  }
}

float EncAdaptiveLoopFilterVtm::getFilterCoeffAndCost(CodingStructure &cs, float distUnfilter, ChannelType channel,
                                                      bool bReCollectStat, int shapeIdx, int &coeffBits,
                                                      bool onlyFilterCost)
{
  float dist                     = distUnfilter;
  coeffBits                      = 0;
  AlfFilterShape &alfFilterShape = (*m_alfParamTemp.filterShapes)[channel][shapeIdx];
  // get filter coeff
  if (isLuma(channel))
  {
    // collect stat based on CTU decision
    if (bReCollectStat)
    {
      getFrameStats(channel, shapeIdx, 0);
    }
    std::fill_n(m_alfClipMerged[shapeIdx][0][0], MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_CLASSES,
                m_alfParamTemp.lumaNonLinearFlag ? ALF_NUM_CLIP_VALS[ChannelType::LUMA] / 2 : 0);

    // Reset Merge Tmp Cov
    m_alfCovarianceMerged[shapeIdx][MAX_NUM_ALF_CLASSES].reset();
    m_alfCovarianceMerged[shapeIdx][MAX_NUM_ALF_CLASSES + 1].reset();
    // distortion
    dist += mergeFiltersAndCost(m_alfParamTemp, alfFilterShape, m_alfCovarianceFrame[channel][shapeIdx],
                                m_alfCovarianceMerged[shapeIdx], m_alfClipMerged[shapeIdx], coeffBits);
  }
  else
  {
    // distortion
    CHECK(m_alfParamTemp.numAlternativesChroma < 1, "Wrong number of m_alfParamTemp.numAlternativesChroma");

    for (int altIdx = 0; altIdx < m_alfParamTemp.numAlternativesChroma; ++altIdx)
    {
      // collect stat based on CTU decision
      if (bReCollectStat)
      {
        getFrameStats(channel, shapeIdx, altIdx);
      }
      assert(alfFilterShape.numCoeff == m_alfCovarianceFrame[channel][shapeIdx][0].numCoeff);

      AlfParam  bestSliceParam;
      float     bestDist         = MAX_FLOAT;
      float     bestCost         = MAX_FLOAT;
      int       bestCoeffBits    = 0;
      const int nonLinearFlagMax = m_encCfg->m_useNonLinearAlfChroma ? 1 : 0;

      for (int nonLinearFlag = 0; nonLinearFlag <= nonLinearFlagMax; nonLinearFlag++)
      {
        const int currentNonLinearFlag = m_alfParamTemp.chromaNonLinearFlag ? 1 : 0;
        if (nonLinearFlag != currentNonLinearFlag)
        {
          continue;
        }

        std::fill_n(m_filterClippSet[altIdx], MAX_NUM_ALF_CHROMA_COEFF,
                    nonLinearFlag ? ALF_NUM_CLIP_VALS[ChannelType::CHROMA] / 2 : 0);
        float dist = m_alfCovarianceFrame[channel][shapeIdx][0].pixAcc +
          deriveCoeffQuant(m_filterClippSet[altIdx], m_filterCoeffSet[altIdx],
                           m_alfCovarianceFrame[channel][shapeIdx][0], alfFilterShape, COEFF_SCALE_BITS_ALF,
                           nonLinearFlag);
        for (int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++)
        {
          m_alfParamTemp.chromaCoeff[altIdx][i] = m_filterCoeffSet[altIdx][i];
          m_alfParamTemp.chromaClipp[altIdx][i] = m_filterClippSet[altIdx][i];
        }
        int   coeffBits = getChromaCoeffRate(m_alfParamTemp, altIdx);
        float cost      = dist + m_lambda[isLuma(channel) ? COMP_Y : COMP_Cb] * coeffBits;
        if (cost < bestCost)
        {
          bestCost       = cost;
          bestDist       = dist;
          bestCoeffBits  = coeffBits;
          bestSliceParam = m_alfParamTemp;
        }
      }
      coeffBits += bestCoeffBits;
      dist += bestDist;
      m_alfParamTemp = bestSliceParam;
    }
    coeffBits += lengthUvlc(m_alfParamTemp.numAlternativesChroma - 1);
    coeffBits++;
  }
  if (onlyFilterCost)
  {
    return dist + m_lambda[isLuma(channel) ? COMP_Y : COMP_Cb] * coeffBits;
  }
  float rate = coeffBits;
  m_CABACEstimator->resetBits();
  m_CABACEstimator->codeAlfCtuEnableFlags(cs, channel, &m_alfParamTemp);
  for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++)
  {
    if (isLuma(channel))
    {
      // Evaluate cost of signaling filter set index for convergence of filters enabled flag / filter derivation
      CHECKD(cs.slice->m_pic->getAlfModes(COMP_Y)[ctuIdx] != CTB_MODE_LUMA_APS0 &&
               cs.slice->m_pic->getAlfModes(COMP_Y)[ctuIdx] != CTB_MODE_OFF,
             "Unexpected CTU ALF mode.");
      CHECKD(cs.slice->m_numAlfApsIdsLuma != 1, "Unexpected number of luma APSs.");
      m_CABACEstimator->codeAlfCtuFilterIndex(cs, ctuIdx, m_alfParamTemp.enabledFlag[COMP_Y]);
    }
  }
  if (isChroma(channel))
  {
    m_CABACEstimator->codeAlfCtuChromaAlternatives(cs, m_alfParamTemp);
  }
  rate += FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
  return dist + m_lambda[isLuma(channel) ? COMP_Y : COMP_Cb] * rate;
}

int EncAdaptiveLoopFilterVtm::getChromaCoeffRate(AlfParam &alfParam, int altIdx) const
{
  int                  iBits = 0;
  const AlfFilterShape alfShape(ALF_FILTER_5);

  // Filter coefficients
  for (int i = 0; i < alfShape.numCoeff - 1; i++)
  {
    iBits += lengthUvlc(abs(alfParam.chromaCoeff[altIdx][i]));   // alf_coeff_chroma[altIdx][i]
    if ((alfParam.chromaCoeff[altIdx][i]) != 0)
    {
      iBits += 1;
    }
  }

  if (m_alfParamTemp.chromaNonLinearFlag)
  {
    for (int i = 0; i < alfShape.numCoeff - 1; i++)
    {
      if (!abs(alfParam.chromaCoeff[altIdx][i]))
      {
        alfParam.chromaClipp[altIdx][i] = 0;
      }
    }
    iBits += ((alfShape.numCoeff - 1) << 1);
  }
  return iBits;
}

float EncAdaptiveLoopFilterVtm::getFilteredDistortion(const CovarianceVec1D &cov, const int numClasses,
                                                      const int numFiltersMinus1, const int numCoeff) const
{
  CHECKD(cov.size() != numClasses, "Covariance vector size does not match number of classes.");

  float dist = 0;
  for (int classIdx = 0; classIdx < numClasses; classIdx++)
  {
    dist += cov[classIdx].calcErrorForCoeffs(m_filterClippSet[classIdx], m_filterCoeffSet[classIdx], numCoeff, 1,
                                             COEFF_SCALE_BITS_ALF);
  }
  return dist;
}

float EncAdaptiveLoopFilterVtm::mergeFiltersAndCost(
  AlfParam &alfParam, AlfFilterShape &alfShape, const CovarianceVec1D &covFrame, CovMergedArr1D &covMerged,
  int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], int &coeffBitsFinal)
{
  int   numFiltersBest = 0;
  int   numFilters     = MAX_NUM_ALF_CLASSES;
  bool  codedVarBins[MAX_NUM_ALF_CLASSES];
  float errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2];

  float cost, cost0, dist, distForce0, costMin = MAX_FLOAT;
  int   coeffBits, coeffBitsForce0;

  mergeClasses(alfShape, covFrame, covMerged, clipMerged, MAX_NUM_ALF_CLASSES, m_filterIndices);

  while (numFilters >= 1)
  {
    dist = deriveFilterCoeffs(covFrame, covMerged, clipMerged, alfShape, m_filterIndices[numFilters - 1], numFilters,
                              errorForce0CoeffTab);
    // filter coeffs are stored in m_filterCoeffSet
    distForce0      = getDistForce0(alfShape, numFilters, errorForce0CoeffTab, codedVarBins);
    coeffBits       = deriveFilterCoefficientsPredictionMode(alfShape, m_filterCoeffSet, numFilters);
    coeffBitsForce0 = getCostFilterCoeffForce0(alfShape, m_filterCoeffSet, numFilters, codedVarBins);

    cost  = dist + m_lambda[COMP_Y] * coeffBits;
    cost0 = distForce0 + m_lambda[COMP_Y] * coeffBitsForce0;

    if (cost0 < cost)
    {
      cost = cost0;
    }

    if (cost <= costMin)
    {
      costMin        = cost;
      numFiltersBest = numFilters;
    }
    numFilters--;
  }

  dist            = deriveFilterCoeffs(covFrame, covMerged, clipMerged, alfShape, m_filterIndices[numFiltersBest - 1],
                                       numFiltersBest, errorForce0CoeffTab);
  coeffBits       = deriveFilterCoefficientsPredictionMode(alfShape, m_filterCoeffSet, numFiltersBest);
  distForce0      = getDistForce0(alfShape, numFiltersBest, errorForce0CoeffTab, codedVarBins);
  coeffBitsForce0 = getCostFilterCoeffForce0(alfShape, m_filterCoeffSet, numFiltersBest, codedVarBins);

  cost  = dist + m_lambda[COMP_Y] * coeffBits;
  cost0 = distForce0 + m_lambda[COMP_Y] * coeffBitsForce0;

  alfParam.numLumaFilters = numFiltersBest;
  float distReturn;
  if (cost <= cost0)
  {
    distReturn     = dist;
    coeffBitsFinal = coeffBits;
  }
  else
  {
    distReturn     = distForce0;
    coeffBitsFinal = coeffBitsForce0;

    for (int varInd = 0; varInd < numFiltersBest; varInd++)
    {
      if (codedVarBins[varInd] == 0)
      {
        memset(m_filterCoeffSet[varInd], 0, sizeof(int) * MAX_NUM_ALF_LUMA_COEFF);
        memset(m_filterClippSet[varInd], 0, sizeof(int) * MAX_NUM_ALF_LUMA_COEFF);
      }
    }
  }

  for (int ind = 0; ind < alfParam.numLumaFilters; ++ind)
  {
    for (int i = 0; i < alfShape.numCoeff; i++)
    {
      alfParam.lumaCoeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_filterCoeffSet[ind][i];
      alfParam.lumaClipp[ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_filterClippSet[ind][i];
    }
  }

  auto &filterIdcsDst = alfParam.filterCoeffDeltaIdx;
  std::copy_n(std::begin(m_filterIndices[numFiltersBest - 1]), MAX_NUM_ALF_CLASSES, std::begin(filterIdcsDst));

  coeffBitsFinal += getNonFilterCoeffRate(alfParam);
  return distReturn;
}

int EncAdaptiveLoopFilterVtm::getNonFilterCoeffRate(AlfParam &alfParam)
{
  const int numLumaFilters = alfParam.numLumaFilters;
  const int maxNumClasses  = MAX_NUM_ALF_CLASSES;

  CHECK(numLumaFilters < 1, "Wrong number of alfParam.numLumaFilters[altIdx]");

  int len = 0   // alf_coefficients_delta_flag
    + 2   // slice_alf_chroma_idc                     u(2)
    + lengthUvlc(numLumaFilters - 1);   // alf_luma_num_filters_signalled_minus1   ue(v)

  if (numLumaFilters > 1)
  {
    const int coeffLength = ceilLog2(numLumaFilters);
    for (int i = 0; i < maxNumClasses; ++i)
    {
      len += coeffLength;   // alf_luma_coeff_delta_idx   u(v)
    }
  }

  return len;
}

int EncAdaptiveLoopFilterVtm::getCostFilterCoeffForce0(AlfFilterShape &alfShape, int **pDiffQFilterCoeffIntPP,
                                                       const int numFilters, bool *codedVarBins)
{
  int len = 0;
  // Filter coefficients
  for (int ind = 0; ind < numFilters; ++ind)
  {
    if (codedVarBins[ind])
    {
      for (int i = 0; i < alfShape.numCoeff - 1; i++)
      {
        len += lengthUvlc(abs(pDiffQFilterCoeffIntPP[ind][i]));   // alf_coeff_luma_delta[i][j]
        if ((abs(pDiffQFilterCoeffIntPP[ind][i]) != 0))
        {
          len += 1;
        }
      }
    }
    else
    {
      for (int i = 0; i < alfShape.numCoeff - 1; i++)
      {
        len += lengthUvlc(0);   // alf_coeff_luma_delta[i][j]
      }
    }
  }

  if (m_alfParamTemp.lumaNonLinearFlag)
  {
    for (int ind = 0; ind < numFilters; ++ind)
    {
      for (int i = 0; i < alfShape.numCoeff - 1; i++)
      {
        if (!abs(pDiffQFilterCoeffIntPP[ind][i]))
        {
          m_filterClippSet[ind][i] = 0;
        }
        len += 2;
      }
    }
  }

  return len;
}

int EncAdaptiveLoopFilterVtm::deriveFilterCoefficientsPredictionMode(const AlfFilterShape   &alfShape,
                                                                     const int *const *const filterSet,
                                                                     const int               numFilters)
{
  const int costCoeff = lengthFilterCoeffs(alfShape, numFilters, filterSet);
  if (m_alfParamTemp.lumaNonLinearFlag)
  {
    return costCoeff + getCostFilterClipp(alfShape.numCoeff, filterSet, numFilters);
  }
  else
  {
    return costCoeff;
  }
}

int EncAdaptiveLoopFilterVtm::lengthFilterCoeffs(const AlfFilterShape &alfShape, const int numFilters,
                                                 const int *const *const filterCoeff) const
{
  int bitCnt = 0;
  for (int ind = 0; ind < numFilters; ++ind)
  {
    for (int i = 0; i < alfShape.numCoeff - 1; i++)
    {
      bitCnt += lengthUvlc(abs(filterCoeff[ind][i]));
      if (abs(filterCoeff[ind][i]) != 0)
      {
        bitCnt += 1;
      }
    }
  }
  return bitCnt;
}

float EncAdaptiveLoopFilterVtm::getDistForce0(AlfFilterShape &alfShape, const int numFilters,
                                              float errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], bool *codedVarBins)
{
  int bitsVarBin[MAX_NUM_ALF_CLASSES];

  for (int ind = 0; ind < numFilters; ++ind)
  {
    bitsVarBin[ind] = 0;
    for (int i = 0; i < alfShape.numCoeff - 1; i++)
    {
      bitsVarBin[ind] += lengthUvlc(abs(m_filterCoeffSet[ind][i]));
      if (abs(m_filterCoeffSet[ind][i]) != 0)
      {
        bitsVarBin[ind] += 1;
      }
    }
  }

  int zeroBitsVarBin = 0;
  for (int i = 0; i < alfShape.numCoeff - 1; i++)
  {
    zeroBitsVarBin += lengthUvlc(0);
  }

  const bool nonLinearFlag = m_alfParamTemp.lumaNonLinearFlag;
  if (nonLinearFlag)
  {
    for (int ind = 0; ind < numFilters; ++ind)
    {
      for (int i = 0; i < alfShape.numCoeff - 1; i++)
      {
        if (!abs(m_filterCoeffSet[ind][i]))
        {
          m_filterClippSet[ind][i] = 0;
        }
      }
    }
  }

  float distForce0 = getDistCoeffForce0(codedVarBins, errorTabForce0Coeff, bitsVarBin, zeroBitsVarBin, numFilters);

  return distForce0;
}

float EncAdaptiveLoopFilterVtm::deriveFilterCoeffs(
  const CovarianceVec1D &cov, CovMergedArr1D &covMerged,
  int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], AlfFilterShape &alfShape,
  const short *const filterIndices, const int numFilters, float errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2])
{
  float          error  = 0.0;
  AlfCovariance &tmpCov = covMerged[MAX_NUM_ALF_CLASSES];

  for (int filtIdx = 0; filtIdx < numFilters; filtIdx++)
  {
    tmpCov.reset();
    bool foundClip = false;
    for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
    {
      if (filterIndices[classIdx] == filtIdx)
      {
        tmpCov += cov[classIdx];
        if (!foundClip)
        {
          foundClip = true;   // clip should be at the adress of shortest one
          memcpy(m_filterClippSet[filtIdx], clipMerged[numFilters - 1][classIdx], sizeof(int[MAX_NUM_ALF_LUMA_COEFF]));
        }
      }
    }

    // Find coeffcients
    assert(alfShape.numCoeff == tmpCov.numCoeff);
    errorTabForce0Coeff[filtIdx][1] = tmpCov.pixAcc +
      deriveCoeffQuant(m_filterClippSet[filtIdx], m_filterCoeffSet[filtIdx], tmpCov, alfShape, COEFF_SCALE_BITS_ALF,
                       false);

    errorTabForce0Coeff[filtIdx][0] = tmpCov.pixAcc;
    error += errorTabForce0Coeff[filtIdx][1];
  }
  return error;
}

float EncAdaptiveLoopFilterVtm::deriveCoeffQuant(int *const filterClipp, int *const filterCoeffQuant,
                                                 const AlfCovariance &cov, const AlfFilterShape &shape,
                                                 const int fractionalBits, const bool optimizeClip)
{
  const int factor   = 1 << fractionalBits;
  const int maxValue = factor - 1;
  const int minValue = -factor + 1;

  const int numCoeff = shape.numCoeff;
  float     filterCoeff[MAX_NUM_ALF_LUMA_COEFF];

  cov.optimizeFilter(numCoeff, filterClipp, filterCoeff, optimizeClip);
  roundFiltCoeff(filterCoeffQuant, filterCoeff, numCoeff, factor);

  for (int i = 0; i < numCoeff - 1; i++)
  {
    filterCoeffQuant[i] = std::min(maxValue, std::max(minValue, filterCoeffQuant[i]));
  }
  filterCoeffQuant[numCoeff - 1] = 0;

  static_assert(MAX_NUM_ALF_LUMA_COEFF > MAX_NUM_ALF_CHROMA_COEFF,
                "Expecting more coefficients for luma than for chroma.");
  const bool isLumaFilter = numCoeff > MAX_NUM_ALF_CHROMA_COEFF ? 1 : 0;

  int modified = 1;
  if ((isLumaFilter && m_encCfg->m_alfStrengthLuma != 1.0) || (!isLumaFilter && m_encCfg->m_alfStrengthChroma != 1.0))
  {
    modified = 0;
  }
  float errRef = cov.calcErrorForCoeffs(filterClipp, filterCoeffQuant, numCoeff, 1, fractionalBits);
  while (modified)
  {
    modified = 0;
    for (int sign: { 1, -1 })
    {
      float errMin = MAX_FLOAT;
      int   minInd = -1;

      for (int k = 0; k < numCoeff - 1; k++)
      {
        if (filterCoeffQuant[k] - sign > maxValue || filterCoeffQuant[k] - sign < minValue)
        {
          continue;
        }

        filterCoeffQuant[k] -= sign;

        float error = cov.calcErrorForCoeffs(filterClipp, filterCoeffQuant, numCoeff, 1, fractionalBits);
        if (error < errMin)
        {
          errMin = error;
          minInd = k;
        }
        filterCoeffQuant[k] += sign;
      }
      if (errMin < errRef)
      {
        filterCoeffQuant[minInd] -= sign;
        modified++;
        errRef = errMin;
      }
    }
  }

  return errRef;
}

void EncAdaptiveLoopFilterVtm::mergeClasses(
  const AlfFilterShape &alfShape, const CovarianceVec1D &cov, CovMergedArr1D &covMerged,
  int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], const int numClasses,
  short filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES]) const
{
  int     tmpClip[MAX_NUM_ALF_LUMA_COEFF];
  int     bestMergeClip[MAX_NUM_ALF_LUMA_COEFF];
  float   err[MAX_NUM_ALF_CLASSES];
  float   bestMergeErr = std::numeric_limits<float>::max();
  bool    availableClass[MAX_NUM_ALF_CLASSES];
  uint8_t indexList[MAX_NUM_ALF_CLASSES];
  uint8_t indexListTemp[MAX_NUM_ALF_CLASSES];
  int     numRemaining = numClasses;

  memset(filterIndices, 0, sizeof(short) * MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_CLASSES);

  const bool nonLinearFlag = m_alfParamTemp.lumaNonLinearFlag;

  for (int i = 0; i < numClasses; i++)
  {
    filterIndices[numRemaining - 1][i] = i;
    indexList[i]                       = i;
    availableClass[i]                  = true;
    covMerged[i]                       = cov[i];
    covMerged[i].numBins               = nonLinearFlag ? ALF_NUM_CLIP_VALS[ChannelType::LUMA] : 1;
  }

  // Try merging different covariance matrices

  // temporal AlfCovariance structure is allocated as the last element in covMerged array, the size of covMerged is
  // MAX_NUM_ALF_CLASSES + 1
  AlfCovariance &tmpCov = covMerged[MAX_NUM_ALF_CLASSES];
  tmpCov.numBins        = nonLinearFlag ? ALF_NUM_CLIP_VALS[ChannelType::LUMA] : 1;

  // init Clip
  for (int i = 0; i < numClasses; i++)
  {
    std::fill_n(clipMerged[numRemaining - 1][i], MAX_NUM_ALF_LUMA_COEFF,
                nonLinearFlag ? ALF_NUM_CLIP_VALS[ChannelType::LUMA] / 2 : 0);
    if (nonLinearFlag)
    {
      err[i] = covMerged[i].optimizeFilterClip(alfShape.numCoeff, clipMerged[numRemaining - 1][i]);
    }
    else
    {
      err[i] = covMerged[i].calculateError(clipMerged[numRemaining - 1][i]);
    }
  }

  while (numRemaining >= 2)
  {
    float errorMin        = std::numeric_limits<float>::max();
    int   bestToMergeIdx1 = 0, bestToMergeIdx2 = 1;

    for (int i = 0; i < numClasses - 1; i++)
    {
      if (availableClass[i])
      {
        for (int j = i + 1; j < numClasses; j++)
        {
          if (availableClass[j])
          {
            float error1 = err[i];
            float error2 = err[j];

            tmpCov.add(covMerged[i], covMerged[j]);
            for (int l = 0; l < MAX_NUM_ALF_LUMA_COEFF; ++l)
            {
              tmpClip[l] = (clipMerged[numRemaining - 1][i][l] + clipMerged[numRemaining - 1][j][l] + 1) >> 1;
            }

            float errorMerged = m_alfParamTemp.lumaNonLinearFlag ? tmpCov.optimizeFilterClip(alfShape.numCoeff, tmpClip)
                                                                 : tmpCov.calculateError(tmpClip);
            float error       = errorMerged - error1 - error2;

            if (error < errorMin)
            {
              bestMergeErr = errorMerged;
              memcpy(bestMergeClip, tmpClip, sizeof(bestMergeClip));
              errorMin        = error;
              bestToMergeIdx1 = i;
              bestToMergeIdx2 = j;
            }
          }
        }
      }
    }

    covMerged[bestToMergeIdx1] += covMerged[bestToMergeIdx2];
    memcpy(clipMerged[numRemaining - 2], clipMerged[numRemaining - 1],
           sizeof(int[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF]));
    memcpy(clipMerged[numRemaining - 2][bestToMergeIdx1], bestMergeClip, sizeof(bestMergeClip));
    err[bestToMergeIdx1]            = bestMergeErr;
    availableClass[bestToMergeIdx2] = false;

    for (int i = 0; i < numClasses; i++)
    {
      if (indexList[i] == bestToMergeIdx2)
      {
        indexList[i] = bestToMergeIdx1;
      }
    }

    numRemaining--;
    if (numRemaining <= numClasses)
    {
      std::memcpy(indexListTemp, indexList, sizeof(uint8_t) * numClasses);

      bool exist = false;
      int  ind   = 0;

      for (int j = 0; j < numClasses; j++)
      {
        exist = false;
        for (int i = 0; i < numClasses; i++)
        {
          if (indexListTemp[i] == j)
          {
            exist = true;
            break;
          }
        }

        if (exist)
        {
          for (int i = 0; i < numClasses; i++)
          {
            if (indexListTemp[i] == j)
            {
              filterIndices[numRemaining - 1][i] = ind;
              indexListTemp[i]                   = -1;
            }
          }
          ind++;
        }
      }
    }
  }
}

void EncAdaptiveLoopFilterVtm::getFrameStats(const ChannelType channel, const int shapeIdx, int altIdx)
{
  int numClasses = isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1;
  for (int i = 0; i < numClasses; i++)
  {
    m_alfCovarianceFrame[channel][shapeIdx][i].reset();
  }

  if (isLuma(channel))
  {
    getFrameStat(m_alfCovarianceFrame[ChannelType::LUMA][shapeIdx], m_alfCovariance[COMP_Y][shapeIdx],
                 (*m_modes)[COMP_Y], numClasses, altIdx);
  }
  else
  {
    getFrameStat(m_alfCovarianceFrame[ChannelType::CHROMA][shapeIdx], m_alfCovariance[COMP_Cb][shapeIdx],
                 (*m_modes)[COMP_Cb], numClasses, altIdx);
    getFrameStat(m_alfCovarianceFrame[ChannelType::CHROMA][shapeIdx], m_alfCovariance[COMP_Cr][shapeIdx],
                 (*m_modes)[COMP_Cr], numClasses, altIdx);
  }
}

void EncAdaptiveLoopFilterVtm::getFrameStat(CovarianceVec1D &frameCov, const CovarianceVec2D &ctbCov,
                                            const CtbModes &ctbModes, const int numClasses, int altIdx) const
{
  // XXX: Guessing the channel type from the number of classes.
  const ChannelType channel = numClasses > 1 ? ChannelType::LUMA : ChannelType::CHROMA;
  for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++)
  {
    if (CtbModeHandler::isEnabled(ctbModes[ctuIdx]))
    {
      for (int classIdx = 0; classIdx < numClasses; classIdx++)
      {
        if (isLuma(channel) || altIdx == ChromaCtbModeHandler::getAlternative(ctbModes[ctuIdx]))
        {
          frameCov[classIdx] += ctbCov[ctuIdx][classIdx];
        }
      }
    }
  }
}

template<bool alfWSSD>
void EncAdaptiveLoopFilterVtm::deriveStatsForFiltering(PelUnitBuf &orgYuv, PelUnitBuf &recYuv, CodingStructure &cs)
{
  int       ctuRsAddr          = 0;
  const int numberOfComponents = getNumberValidComponents(m_chromaFormat);

  // init CTU stats buffers
  for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    const CompID compID     = CompID(compIdx);
    const int    numClasses = isLuma(compID) ? MAX_NUM_ALF_CLASSES : 1;

    for (int shape = 0; shape != m_filterShapes[toChannelType(compID)].size(); shape++)
    {
      for (int classIdx = 0; classIdx < numClasses; classIdx++)
      {
        for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++)
        {
          m_alfCovariance[compIdx][shape][ctuIdx][classIdx].reset();
        }
      }
    }
  }

  // init Frame stats buffers
  for (auto chType = ChannelType::LUMA; chType <= ::getLastChannel(m_chromaFormat); chType++)
  {
    const int numClasses = isLuma(chType) ? MAX_NUM_ALF_CLASSES : 1;

    for (int shape = 0; shape != m_filterShapes[chType].size(); ++shape)
    {
      for (int classIdx = 0; classIdx < numClasses; classIdx++)
      {
        m_alfCovarianceFrame[chType][shape][classIdx].reset();
      }
    }
  }

  const PreCalcValues &pcv     = *cs.pcv;
  bool                 clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;

  for (int yPos = 0; yPos < m_picHeight; yPos += m_maxCUHeight)
  {
    for (int xPos = 0; xPos < m_picWidth; xPos += m_maxCUWidth)
    {
      const int width  = std::min(m_picWidth - xPos, m_maxCUWidth);
      const int height = std::min(m_picHeight - yPos, m_maxCUHeight);

      int rasterSliceAlfPad = 0;
      if (isCrossedBySubPicBoundaries(cs, xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight,
                                      rasterSliceAlfPad))
      {
        const bool clipT = clipTop || (yPos == 0);
        const bool clipB = clipBottom || (yPos + height == pcv.lumaHeight);
        const bool clipL = clipLeft || (xPos == 0);
        const bool clipR = clipRight || (xPos + width == pcv.lumaWidth);
        const int  wBuf  = width + (clipL ? 0 : MAX_ALF_PADDING_SIZE) + (clipR ? 0 : MAX_ALF_PADDING_SIZE);
        const int  hBuf  = height + (clipT ? 0 : MAX_ALF_PADDING_SIZE) + (clipB ? 0 : MAX_ALF_PADDING_SIZE);

        PelUnitBuf recBuf = m_tempBuf2.subBuf(UnitArea(cs.area.chromaFormat, Area(0, 0, wBuf, hBuf)));
        recBuf.copyFrom(recYuv.subBuf(UnitArea(
          cs.area.chromaFormat,
          Area(xPos - (clipL ? 0 : MAX_ALF_PADDING_SIZE), yPos - (clipT ? 0 : MAX_ALF_PADDING_SIZE), wBuf, hBuf))));
        // pad top-left unavailable samples for raster slice
        if (rasterSliceAlfPad & 1)
        {
          recBuf.padBorderPel(MAX_ALF_PADDING_SIZE, 1);
        }

        // pad bottom-right unavailable samples for raster slice
        if (rasterSliceAlfPad & 2)
        {
          recBuf.padBorderPel(MAX_ALF_PADDING_SIZE, 2);
        }
        recBuf.extendBorderPel(MAX_ALF_PADDING_SIZE);
        recBuf = recBuf.subBuf(
          UnitArea(cs.area.chromaFormat,
                   Area(clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, width, height)));

        const UnitArea area(m_chromaFormat, Area(0, 0, width, height));
        const UnitArea areaDst(m_chromaFormat, Area(xPos, yPos, width, height));
        for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
        {
          const CompID    compID   = CompID(compIdx);
          const CompArea &compArea = area.block(compID);

          ptrdiff_t recStride = recBuf.get(compID).stride;
          Pel      *rec       = recBuf.get(compID).bufAt(compArea);

          ptrdiff_t orgStride = orgYuv.get(compID).stride;
          Pel      *org       = orgYuv.get(compID).bufAt(xPos >> ::getComponentScaleX(compID, m_chromaFormat),
                                                         yPos >> ::getComponentScaleY(compID, m_chromaFormat));

          ptrdiff_t orgLumaStride = orgYuv.get(COMP_Y).stride;
          Pel      *orgLuma       = orgYuv.get(COMP_Y).bufAt(xPos, yPos);

          ChannelType chType = toChannelType(compID);

          for (int shape = 0; shape != m_filterShapes[chType].size(); shape++)
          {
            const CompArea &compAreaDst = areaDst.block(compID);

            getBlkStats(m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape],
                        compIdx ? nullptr : &m_classifier, org, orgStride, orgLuma, orgLumaStride, rec, recStride,
                        compAreaDst, compArea, chType, (compIdx == 0) ? m_alfVBLumaCTUHeight : m_alfVBChmaCTUHeight,
                        (compIdx == 0) ? m_alfVBLumaPos : m_alfVBChmaPos);
          }
        }

        for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
        {
          const CompID      compID     = CompID(compIdx);
          const ChannelType chType     = toChannelType(compID);
          const int         numClasses = isLuma(compID) ? MAX_NUM_ALF_CLASSES : 1;

          for (int shape = 0; shape != m_filterShapes[chType].size(); shape++)
          {
            for (int classIdx = 0; classIdx < numClasses; classIdx++)
            {
              m_alfCovarianceFrame[chType][shape][isLuma(compID) ? classIdx : 0] +=
                m_alfCovariance[compIdx][shape][ctuRsAddr][classIdx];
            }
          }
        }
      }
      else
      {
        const UnitArea area(m_chromaFormat, Area(xPos, yPos, width, height));

        for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
        {
          const CompID    compID   = CompID(compIdx);
          const CompArea &compArea = area.block(compID);

          ptrdiff_t recStride = recYuv.get(compID).stride;
          Pel      *rec       = recYuv.get(compID).bufAt(compArea);

          ptrdiff_t orgStride = orgYuv.get(compID).stride;
          Pel      *org       = orgYuv.get(compID).bufAt(compArea);

          ptrdiff_t orgLumaStride = orgYuv.get(COMP_Y).stride;
          Pel      *orgLuma       = orgYuv.get(COMP_Y).bufAt(area.block(COMP_Y));

          ChannelType chType = toChannelType(compID);

          for (int shape = 0; shape != m_filterShapes[chType].size(); shape++)
          {
            getBlkStats(m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape],
                        compIdx ? nullptr : &m_classifier, org, orgStride, orgLuma, orgLumaStride, rec, recStride,
                        compArea, compArea, chType, (compIdx == 0) ? m_alfVBLumaCTUHeight : m_alfVBChmaCTUHeight,
                        (compIdx == 0) ? m_alfVBLumaPos : m_alfVBChmaPos);

            const int numClasses = isLuma(compID) ? MAX_NUM_ALF_CLASSES : 1;
            for (int classIdx = 0; classIdx < numClasses; classIdx++)
            {
              m_alfCovarianceFrame[chType][shape][isLuma(compID) ? classIdx : 0] +=
                m_alfCovariance[compIdx][shape][ctuRsAddr][classIdx];
            }
          }
        }
      }
      ctuRsAddr++;
    }
  }

  initDistortion<alfWSSD>();
}

void EncAdaptiveLoopFilterVtm::getBlkStats(CovarianceVec1D &alfCovariance, const AlfFilterShape &shape,
                                           const ClassBuf *const classifier, const Pel *org, const ptrdiff_t orgStride,
                                           const Pel *orgLuma, const ptrdiff_t orgLumaStride, const Pel *rec,
                                           const ptrdiff_t recStride, const CompArea &areaDst, const CompArea &area,
                                           const ChannelType channel, int vbCTUHeight, int vbPos)
{
  Pel ELocal[MAX_NUM_ALF_LUMA_COEFF][MAX_ALF_NUM_CLIP_VALS];

  const int numBins = (isLuma(channel) ? m_encCfg->m_useNonLinearAlfLuma : m_encCfg->m_useNonLinearAlfChroma)
    ? ALF_NUM_CLIP_VALS[channel]
    : 1;

  const float strength    = isLuma(channel) ? m_encCfg->m_alfStrengthTargetLuma : m_encCfg->m_alfStrengthTargetChroma;
  const float invStrength = strength != 0.0 ? 1.0 / strength : 0.0;

  // TODO (AW): process in chunks of 4x4
  for (int i = 0; i < area.height; i++)
  {
    const int vbDistance = ((areaDst.y + i) % vbCTUHeight) - vbPos;
    for (int j = 0; j < area.width; j++)
    {
      const AlfClassifier *classifierSample =
        (classifier == nullptr) ? nullptr : &((*classifier)[areaDst.y + i][areaDst.x + j]);
      const int classIdx     = (classifier == nullptr) ? 0 : (*classifierSample).classIdx;
      const int transposeIdx = (classifier == nullptr) ? 0 : (*classifierSample).transposeIdx;

      std::fill_n(ELocal[0], MAX_NUM_ALF_LUMA_COEFF * MAX_ALF_NUM_CLIP_VALS, 0);

      calcCovariance(ELocal, rec + j, recStride, shape, transposeIdx, channel, vbDistance);

      const Pel *lumaPtr = orgLuma + (i << ::getChannelTypeScaleY(channel, m_chromaFormat)) * orgLumaStride +
        (j << ::getChannelTypeScaleX(channel, m_chromaFormat));
      const float weight = m_alfWSSD ? m_lumaLevelToWeightPLUT[*lumaPtr] : 1.0;
      const float yLocal = org[j] - rec[j];

      float e[MAX_ALF_NUM_CLIP_VALS][MAX_NUM_ALF_LUMA_COEFF];

      for (int b = 0; b < numBins; b++)
      {
        for (int k = 0; k < shape.numCoeff; k++)
        {
          e[b][k] = invStrength * ELocal[k][b];
        }
      }

      for (ptrdiff_t b0 = 0; b0 < numBins; ++b0)
      {
        for (ptrdiff_t k = 0; k < shape.numCoeff; ++k)
        {
          const float we = e[b0][k] * weight;

          for (ptrdiff_t b1 = 0; b1 <= b0; ++b1)
          {
            const ptrdiff_t maxl = b0 == b1 ? k + 1 : shape.numCoeff;
            for (ptrdiff_t l = 0; l < maxl; ++l)
            {
              const float curE = we * e[b1][l];
              alfCovariance[classIdx].E(b0, b1, k, l) += curE;
            }
          }

          const float curY = we * yLocal;
          alfCovariance[classIdx].y(b0, k) += curY;
        }
      }

      const float curP = weight * yLocal * yLocal;
      alfCovariance[classIdx].pixAcc += curP;
    }

    org += orgStride;
    rec += recStride;
  }
}

void EncAdaptiveLoopFilterVtm::calcCovariance(Pel ELocal[MAX_NUM_ALF_LUMA_COEFF][MAX_ALF_NUM_CLIP_VALS], const Pel *rec,
                                              const ptrdiff_t stride, const AlfFilterShape &shape,
                                              const int transposeIdx, const ChannelType channel, int vbDistance)
{
  int clipTopRow = -4;
  int clipBotRow = 4;
  if (vbDistance >= -3 && vbDistance < 0)
  {
    clipBotRow = -vbDistance - 1;
    clipTopRow = -clipBotRow;   // symmetric
  }
  else if (vbDistance >= 0 && vbDistance < 3)
  {
    clipTopRow = -vbDistance;
    clipBotRow = -clipTopRow;   // symmetric
  }

  const int *filterPattern    = shape.pattern.data();
  const int  halfFilterLength = shape.filterLength >> 1;
  const Pel *clip             = m_alfClippingValues[channel].data();
  const int  numBins          = (isLuma(channel) ? m_encCfg->m_useNonLinearAlfLuma : m_encCfg->m_useNonLinearAlfChroma)
              ? ALF_NUM_CLIP_VALS[channel]
              : 1;

  int k = 0;

  const Pel curr = rec[0];

  if (transposeIdx == 0)
  {
    for (int i = -halfFilterLength; i < 0; i++)
    {
      const Pel *rec0 = rec + std::max(i, clipTopRow) * stride;
      const Pel *rec1 = rec - std::max(i, -clipBotRow) * stride;
      for (int j = -halfFilterLength - i; j <= halfFilterLength + i; j++, k++)
      {
        for (int b = 0; b < numBins; b++)
        {
          ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec0[j], rec1[-j]);
        }
      }
    }
    for (int j = -halfFilterLength; j < 0; j++, k++)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec[j], rec[-j]);
      }
    }
  }
  else if (transposeIdx == 1)
  {
    for (int j = -halfFilterLength; j < 0; j++)
    {
      const Pel *rec0 = rec + j;
      const Pel *rec1 = rec - j;
      for (int i = -halfFilterLength - j; i <= halfFilterLength + j; i++, k++)
      {
        for (int b = 0; b < numBins; b++)
        {
          ELocal[filterPattern[k]][b] +=
            clipALF(clip[b], curr, rec0[std::max(i, clipTopRow) * stride], rec1[-std::max(i, -clipBotRow) * stride]);
        }
      }
    }
    for (int i = -halfFilterLength; i < 0; i++, k++)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[filterPattern[k]][b] +=
          clipALF(clip[b], curr, rec[std::max(i, clipTopRow) * stride], rec[-std::max(i, -clipBotRow) * stride]);
      }
    }
  }
  else if (transposeIdx == 2)
  {
    for (int i = -halfFilterLength; i < 0; i++)
    {
      const Pel *rec0 = rec + std::max(i, clipTopRow) * stride;
      const Pel *rec1 = rec - std::max(i, -clipBotRow) * stride;

      for (int j = halfFilterLength + i; j >= -halfFilterLength - i; j--, k++)
      {
        for (int b = 0; b < numBins; b++)
        {
          ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec0[j], rec1[-j]);
        }
      }
    }
    for (int j = -halfFilterLength; j < 0; j++, k++)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec[j], rec[-j]);
      }
    }
  }
  else
  {
    for (int j = -halfFilterLength; j < 0; j++)
    {
      const Pel *rec0 = rec + j;
      const Pel *rec1 = rec - j;
      for (int i = halfFilterLength + j; i >= -halfFilterLength - j; i--, k++)
      {
        for (int b = 0; b < numBins; b++)
        {
          ELocal[filterPattern[k]][b] +=
            clipALF(clip[b], curr, rec0[std::max(i, clipTopRow) * stride], rec1[-std::max(i, -clipBotRow) * stride]);
        }
      }
    }
    for (int i = -halfFilterLength; i < 0; i++, k++)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[filterPattern[k]][b] +=
          clipALF(clip[b], curr, rec[std::max(i, clipTopRow) * stride], rec[-std::max(i, -clipBotRow) * stride]);
      }
    }
  }

  for (int b = 0; b < numBins; b++)
  {
    ELocal[filterPattern[k]][b] += curr;
  }
}

template<bool alfWSSD> void EncAdaptiveLoopFilterVtm::initDistortion()
{
  for (int comp = 0; comp < MAX_NUM_COMP; comp++)
  {
    m_unFiltDistCompnent[comp] = 0.0;

    for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
    {
      m_ctbDistortionUnfilter[comp][ctbIdx] =
        getUnfilteredDistortion(m_alfCovariance[comp][0][ctbIdx], comp == 0 ? MAX_NUM_ALF_CLASSES : 1);

      m_unFiltDistCompnent[comp] += m_ctbDistortionUnfilter[comp][ctbIdx];
    }
  }

  for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ++ctbIdx)
  {
    for (int filterSetIdx = 0; filterSetIdx < ALF_NUM_FIXED_FILTER_SETS; ++filterSetIdx)
    {
      m_ctbDistortionFixedFilter[filterSetIdx][ctbIdx] = m_ctbDistortionUnfilter[CompID::COMP_Y][ctbIdx];
      for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; ++classIdx)
      {
        int filterIdx = m_classToFilterMapping[filterSetIdx][classIdx];
        m_ctbDistortionFixedFilter[filterSetIdx][ctbIdx] +=
          m_alfCovariance[CompID::COMP_Y][0][ctbIdx][classIdx].calcErrorForCoeffs(
            m_clipDefaultEnc, m_fixedFilterSetCoeff[filterIdx], MAX_NUM_ALF_LUMA_COEFF, 1, COEFF_SCALE_BITS_ALF);
      }
    }
  }
}

void EncAdaptiveLoopFilterVtm::initDistortionCcalf(const CompID compId)
{
  for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
  {
    m_ctbDistortionUnfilter[compId][ctbIdx] = m_alfCovarianceCcAlf[0][ctbIdx].pixAcc;
  }
}

void EncAdaptiveLoopFilterVtm::getDistNewFilter(AlfParam &alfParam)
{
  reconstructLumaCoeff(alfParam, true);
  for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; ++classIdx)
  {
    for (int coeff = 0; coeff < MAX_NUM_ALF_LUMA_COEFF; ++coeff)
    {
      m_filterTmp[coeff] = m_coeffFinal[classIdx * MAX_NUM_ALF_LUMA_COEFF + coeff];
      m_clipTmp[coeff]   = m_clippFinal[classIdx * MAX_NUM_ALF_LUMA_COEFF + coeff];
    }

    for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ++ctbIdx)
    {
      float               &dist = m_distCtbLumaNewFilt[ctbIdx];
      const AlfCovariance &cov  = m_alfCovariance[CompID::COMP_Y][0][ctbIdx][classIdx];

      if (classIdx == 0)
      {
        dist = m_ctbDistortionUnfilter[CompID::COMP_Y][ctbIdx];
      }
      dist += cov.calcErrorForCoeffs(m_clipTmp, m_filterTmp, MAX_NUM_ALF_LUMA_COEFF, 1, COEFF_SCALE_BITS_ALF);
    }
  }
}

void EncAdaptiveLoopFilterVtm::getDistApsFilter(const CodingStructure &cs, const AlfApsList &apsIds)
{
  for (int apsIdx = 0; apsIdx < apsIds.size(); apsIdx++)
  {
    const APS *curAPS      = cs.slice->m_alfApss[apsIds[apsIdx]];
    AlfParam   alfParamTmp = curAPS->m_alfAPSParam.getVtmParam();

    reconstructLumaCoeff(alfParamTmp, true);

    for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; ++classIdx)
    {
      for (int coeff = 0; coeff < MAX_NUM_ALF_LUMA_COEFF; ++coeff)
      {
        m_filterTmp[coeff] = m_coeffFinal[classIdx * MAX_NUM_ALF_LUMA_COEFF + coeff];
        m_clipTmp[coeff]   = m_clippFinal[classIdx * MAX_NUM_ALF_LUMA_COEFF + coeff];
      }

      for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ++ctbIdx)
      {
        float               &dist = m_distCtbApsLuma[apsIdx][ctbIdx];
        const AlfCovariance &cov  = m_alfCovariance[CompID::COMP_Y][0][ctbIdx][classIdx];

        if (classIdx == 0)
        {
          dist = m_ctbDistortionUnfilter[CompID::COMP_Y][ctbIdx];
        }
        dist += cov.calcErrorForCoeffs(m_clipTmp, m_filterTmp, MAX_NUM_ALF_LUMA_COEFF, 1, COEFF_SCALE_BITS_ALF);
      }
    }
  }
}

void EncAdaptiveLoopFilterVtm::alfEncoderCtb(CodingStructure &cs, AlfParam &alfParamNewFilters
#if ENABLE_QPA
                                             ,
                                             const double lambdaChromaWeight
#endif
)
{
  TempCtx  ctxStart(m_ctxPool, AlfCtx(m_CABACEstimator->getCtx()));
  TempCtx  ctxBest(m_ctxPool);
  TempCtx  ctxTempStart(m_ctxPool);
  TempCtx  ctxTempBest(m_ctxPool);
  TempCtx  ctxTempAltStart(m_ctxPool);
  TempCtx  ctxTempAltBest(m_ctxPool);
  AlfParam alfParamNewFiltersBest = alfParamNewFilters;

  APS **apss = cs.slice->m_alfApss;

  // luma
  m_alfParamTemp = alfParamNewFilters;
  setCtuEnableFlag(m_modes, ChannelType::LUMA, CTB_MODE_LUMA_APS0);
  float costOff = m_unFiltDistCompnent[CompID::COMP_Y];
  setCtuEnableFlag(m_modes, ChannelType::LUMA, CTB_MODE_OFF);
  const int         newApsId = getAvailableApsIdsLuma(cs);
  const AlfApsList &apsIds   = cs.slice->m_alfApsIdsLuma;
  AlfApsList        bestApsIds;
  float             costMin = MAX_FLOAT;

  getDistApsFilter(cs, apsIds);

  for (bool useNewFilter: { false, true })
  {
    if (useNewFilter && !alfParamNewFilters.enabledFlag[COMP_Y])
    {
      continue;
    }

    int bitsNewFilter = 0;
    if (useNewFilter)
    {
      bitsNewFilter = m_bitsNewFilter[ChannelType::LUMA];
      getDistNewFilter(alfParamNewFilters);
    }

    for (int numTemporalAps = 0; numTemporalAps <= apsIds.size(); numTemporalAps++)
    {
      const int numApss = numTemporalAps + (useNewFilter ? 1 : 0);

      if (m_encCfg->m_maxNumAlfAps == 0 && numApss > 0)
      {
        continue;
      }

      if (numApss > std::min(ALF_CTB_MAX_NUM_APS - 1, m_encCfg->m_maxNumAlfAps))
      {
        continue;
      }

      cs.slice->m_numAlfApsIdsLuma = numApss;

      const int numFilterSet = ALF_NUM_FIXED_FILTER_SETS + numApss;

      if (numTemporalAps == apsIds.size() && numTemporalAps > 0 && useNewFilter && newApsId == apsIds.back())
      {
        // last temporalAPS is occupied by new filter set and this temporal APS becomes unavailable
        continue;
      }

      const int numIter = useNewFilter ? 2 : 1;

      for (int iter = 0; iter < numIter; iter++)
      {
        m_alfParamTemp                     = alfParamNewFilters;
        m_alfParamTemp.enabledFlag[COMP_Y] = true;

        float curCost = 3 * m_lambda[COMP_Y];

        if (iter > 0)
        {
          // re-derive new filter-set
          float dDistOrgNewFilter    = 0;
          int   blocksUsingNewFilter = 0;
          for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
          {
            int &ctbMode = (*m_modes)[CompID::COMP_Y][ctbIdx];
            if (LumaCtbModeHandler::isApsFilter(ctbMode) && LumaCtbModeHandler::getApsIdx(ctbMode) == 0)
            {
              blocksUsingNewFilter++;
              dDistOrgNewFilter += m_ctbDistortionUnfilter[COMP_Y][ctbIdx];
              for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
              {
                short *pCoeff = m_coeffFinal;
                Pel   *pClipp = m_clippFinal;
                for (int i = 0; i < MAX_NUM_ALF_LUMA_COEFF; i++)
                {
                  m_filterTmp[i] = pCoeff[classIdx * MAX_NUM_ALF_LUMA_COEFF + i];
                  m_clipTmp[i]   = pClipp[classIdx * MAX_NUM_ALF_LUMA_COEFF + i];
                }

                const AlfCovariance &cov = m_alfCovariance[CompID::COMP_Y][0][ctbIdx][classIdx];
                dDistOrgNewFilter +=
                  cov.calcErrorForCoeffs(m_clipTmp, m_filterTmp, MAX_NUM_ALF_LUMA_COEFF, 1, COEFF_SCALE_BITS_ALF);
              }
            }
            else if (CtbModeHandler::isEnabled(ctbMode))
            {
              CtbModeHandler::setDisabled(ctbMode);
            }
          }

          if (blocksUsingNewFilter > 0 && blocksUsingNewFilter < m_numCTUsInPic)
          {
            int   bitNL[2] = { 0, 0 };
            float errNL[2] = { MAX_FLOAT, MAX_FLOAT };

            for (bool nonlinearFlag: { true, false })
            {
              if (nonlinearFlag && !m_encCfg->m_useNonLinearAlfLuma)
              {
                continue;
              }

              m_alfParamTemp.lumaNonLinearFlag = nonlinearFlag;

              const int idx = nonlinearFlag ? 1 : 0;
              errNL[idx]    = getFilterCoeffAndCost(cs, 0, ChannelType::LUMA, true, 0, bitNL[idx], true);

              if (nonlinearFlag)
              {
                m_alfParamTempNL = m_alfParamTemp;
              }
            }

            const bool  useNonlinear          = errNL[1] < errNL[0];
            const int   bitsNewFilterTempLuma = bitNL[useNonlinear ? 1 : 0];
            const float err                   = errNL[useNonlinear ? 1 : 0];

            if (useNonlinear)
            {
              m_alfParamTemp = m_alfParamTempNL;
            }

            if (dDistOrgNewFilter + m_lambda[COMP_Y] * m_bitsNewFilter[ChannelType::LUMA] < err)
            {
              // re-derived filter is not good, skip
              continue;
            }

            getDistNewFilter(m_alfParamTemp);
            bitsNewFilter = bitsNewFilterTempLuma;
          }
          else
          {
            // no block or all blocks using new filter, skip
            continue;
          }
        }

        m_CABACEstimator->getCtx() = ctxStart;
        for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
        {
          int  &currCtbMode     = (*m_modes)[COMP_Y][ctbIdx];
          float distUnfilterCtb = m_ctbDistortionUnfilter[COMP_Y][ctbIdx];
          // ctb on
          float costOn          = MAX_FLOAT;
          ctxTempStart          = AlfCtx(m_CABACEstimator->getCtx());
          int bestMode          = CTB_MODE_OFF;

          const int firstFilterSetIdx = m_encCfg->m_alfAllowPredefinedFilters ? 0 : ALF_NUM_FIXED_FILTER_SETS;

          for (int filterSetIdx = firstFilterSetIdx; filterSetIdx < numFilterSet; filterSetIdx++)
          {
            if (filterSetIdx < ALF_NUM_FIXED_FILTER_SETS)
            {
              LumaCtbModeHandler::setFixedFilter(currCtbMode, filterSetIdx);
            }
            else
            {
              LumaCtbModeHandler::setApsFilter(currCtbMode, filterSetIdx - ALF_NUM_FIXED_FILTER_SETS);
            }

            // rate
            m_CABACEstimator->getCtx() = AlfCtx(ctxTempStart);
            m_CABACEstimator->resetBits();
            m_CABACEstimator->codeAlfCtuEnableFlag(cs, ctbIdx, COMP_Y, &m_alfParamTemp);
            m_CABACEstimator->codeAlfCtuFilterIndex(cs, ctbIdx, m_alfParamTemp.enabledFlag[COMP_Y]);
            float rateOn = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();

            // distortion
            float dist;
            if (filterSetIdx < ALF_NUM_FIXED_FILTER_SETS)
            {
              dist = m_ctbDistortionFixedFilter[filterSetIdx][ctbIdx];
            }
            else if (useNewFilter && filterSetIdx == ALF_NUM_FIXED_FILTER_SETS)
            {
              dist = m_distCtbLumaNewFilt[ctbIdx];
            }
            else
            {
              dist = m_distCtbApsLuma[filterSetIdx - ALF_NUM_FIXED_FILTER_SETS - (useNewFilter ? 1 : 0)][ctbIdx];
            }

            // cost
            float costOnTmp = dist + m_lambda[COMP_Y] * rateOn;
            if (costOnTmp < costOn)
            {
              ctxTempBest = AlfCtx(m_CABACEstimator->getCtx());
              costOn      = costOnTmp;
              bestMode    = currCtbMode;
            }
          }
          // ctb off
          CtbModeHandler::setDisabled(currCtbMode);
          // rate
          m_CABACEstimator->getCtx() = AlfCtx(ctxTempStart);
          m_CABACEstimator->resetBits();
          m_CABACEstimator->codeAlfCtuEnableFlag(cs, ctbIdx, COMP_Y, &m_alfParamTemp);
          // cost
          float costOff = distUnfilterCtb + m_lambda[COMP_Y] * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
          if (costOn < costOff)
          {
            m_CABACEstimator->getCtx() = AlfCtx(ctxTempBest);
            currCtbMode                = bestMode;

            curCost += costOn;
          }
          else
          {
            curCost += costOff;
          }
        }   // for(ctbIdx)

        int tmpBits = bitsNewFilter + 3 * (numFilterSet - ALF_NUM_FIXED_FILTER_SETS);
        curCost += tmpBits * m_lambda[COMP_Y];
        if (curCost < costMin)
        {
          costMin = curCost;
          bestApsIds.resize(numFilterSet - ALF_NUM_FIXED_FILTER_SETS);
          for (int i = 0; i < bestApsIds.size(); i++)
          {
            if (i == 0 && useNewFilter)
            {
              bestApsIds[i] = newApsId;
            }
            else
            {
              bestApsIds[i] = apsIds[i - (useNewFilter ? 1 : 0)];
            }
          }
          alfParamNewFiltersBest = m_alfParamTemp;
          ctxBest                = AlfCtx(m_CABACEstimator->getCtx());
          copyIndices(m_indexTmp, m_modes, ChannelType::LUMA);
          alfParamNewFiltersBest.newFilterFlag[ChannelType::LUMA] = useNewFilter;
        }
      }   // for (int iter = 0; iter < numIter; iter++)
    }   // for (int numTemporalAps = 0; numTemporalAps < apsIds.size(); numTemporalAps++)
  }   // for (int useNewFilter = 0; useNewFilter <= 1; useNewFilter++)

  cs.slice->m_ccAlfCbApsId = newApsId;
  cs.slice->m_ccAlfCrApsId = newApsId;

  if (costOff <= costMin)
  {
    cs.slice->resetAlfEnabledFlag();
    cs.slice->m_numAlfApsIdsLuma = 0;
    setCtuEnableFlag(m_modes, ChannelType::LUMA, CTB_MODE_OFF);
    setCtuEnableFlag(m_modes, ChannelType::CHROMA, CTB_MODE_OFF);
    return;
  }
  else
  {
    cs.slice->m_alfEnabledFlag[COMP_Y] = true;
    cs.slice->m_numAlfApsIdsLuma       = (int)bestApsIds.size();
    cs.slice->m_alfApsIdsLuma          = bestApsIds;

    copyIndices(m_modes, m_indexTmp, ChannelType::LUMA);

    if (alfParamNewFiltersBest.newFilterFlag[ChannelType::LUMA])
    {
      APS *newAPS = m_apsMap->getPS(newApsId);
      if (newAPS == nullptr)
      {
        newAPS            = m_apsMap->allocatePS(newApsId);
        newAPS->m_APSId   = newApsId;
        newAPS->m_APSType = ApsType::ALF;
      }
      AlfParam &alfAPSParam = newAPS->m_alfAPSParam.getVtmParam();

      alfAPSParam                                    = alfParamNewFiltersBest;
      newAPS->m_temporalId                           = cs.slice->m_uiTLayer;
      alfAPSParam.newFilterFlag[ChannelType::CHROMA] = false;
      m_apsMap->setChangedFlag(newApsId);
      m_apsIdStart = newApsId;
    }

    const AlfApsList &apsIds = cs.slice->m_alfApsIdsLuma;
    for (int i = 0; i < (int)cs.slice->m_numAlfApsIdsLuma; i++)
    {
      apss[apsIds[i]] = m_apsMap->getPS(apsIds[i]);
    }
  }

  // chroma
  if (m_encCfg->m_maxNumAlfAps != 0 &&
      isChromaEnabled(cs.pcv->chrFormat))   //  Find ALF parameters for chroma if ALF APS is enabled.
  {
    m_alfParamTemp = alfParamNewFiltersBest;
    if (m_alfParamTemp.numAlternativesChroma < 1)
    {
      m_alfParamTemp.numAlternativesChroma = 1;
    }
    setCtuEnableFlag(m_modes, ChannelType::CHROMA, CTB_MODE_CHROMA_ALT0);
    costOff                    = m_unFiltDistCompnent[CompID::COMP_Cb] + m_unFiltDistCompnent[CompID::COMP_Cr];
    costMin                    = MAX_FLOAT;
    m_CABACEstimator->getCtx() = AlfCtx(ctxBest);
    ctxStart                   = AlfCtx(m_CABACEstimator->getCtx());
    int newApsIdChroma         = -1;
    if (alfParamNewFiltersBest.newFilterFlag[ChannelType::LUMA] &&
        (alfParamNewFiltersBest.enabledFlag[COMP_Cb] || alfParamNewFiltersBest.enabledFlag[COMP_Cr]))
    {
      newApsIdChroma = newApsId;
    }
    else if (alfParamNewFiltersBest.enabledFlag[COMP_Cb] || alfParamNewFiltersBest.enabledFlag[COMP_Cr])
    {
      int curId = m_apsIdStart;
      // Do not assign ALF APS for chroma if any new APS ID is not avaiable

      int counter = m_encCfg->m_maxNumAlfAps;
      while ((newApsIdChroma < 0) && ((counter--)))
      {
        curId--;
        if (curId < m_encCfg->m_alfapsIDShift)
        {
          curId = m_encCfg->m_alfapsIDShift + m_encCfg->m_maxNumAlfAps - 1;
        }
        if (std::find(bestApsIds.begin(), bestApsIds.end(), curId) == bestApsIds.end())
        {
          newApsIdChroma = curId;
        }
      }
    }

    for (int curApsId = m_encCfg->m_alfapsIDShift; curApsId < m_encCfg->m_alfapsIDShift + m_encCfg->m_maxNumAlfAps;
         curApsId++)
    {
      const bool reuseExistingAPS = curApsId != newApsIdChroma;

      if ((cs.slice->m_pendingRasInit || cs.slice->isIRAP()) && reuseExistingAPS)
      {
        continue;
      }

      APS  *curAPS  = m_apsMap->getPS(curApsId);
      float curCost = m_lambda[COMP_Cb] * 3;
      if (!reuseExistingAPS)
      {
        m_alfParamTemp = alfParamNewFilters;
        curCost += m_lambda[COMP_Cb] * m_bitsNewFilter[ChannelType::CHROMA];
      }
      else if (curAPS && curAPS->m_temporalId <= cs.slice->m_uiTLayer &&
               curAPS->m_layerId == cs.slice->m_pic->m_layerId)
      {
        const AlfParam &alfAPSParam = curAPS->m_alfAPSParam.getVtmParam();
        if (alfAPSParam.newFilterFlag[ChannelType::CHROMA])
        {
          m_alfParamTemp = alfAPSParam;
        }
        else
        {
          continue;
        }
      }
      else
      {
        continue;
      }
      reconstructChromaCoeff(m_alfParamTemp, true);
      m_CABACEstimator->getCtx() = AlfCtx(ctxStart);
      for (int compId = 1; compId < MAX_NUM_COMP; compId++)
      {
        m_alfParamTemp.enabledFlag[compId] = true;
        for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
        {
          // set to first mode for rate calculation of CTU enable flag
          ChromaCtbModeHandler::setAlternative((*m_modes)[compId][ctbIdx], 0);

          float distUnfilterCtu      = m_ctbDistortionUnfilter[compId][ctbIdx];
          // cost on
          ctxTempStart               = AlfCtx(m_CABACEstimator->getCtx());
          // rate
          m_CABACEstimator->getCtx() = AlfCtx(ctxTempStart);
          m_CABACEstimator->resetBits();
          // ctb flag
          m_CABACEstimator->codeAlfCtuEnableFlag(cs, ctbIdx, compId, &m_alfParamTemp);
          float rateOn = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
#if ENABLE_QPA
          const double ctuLambda =
            lambdaChromaWeight > 0.0 ? cs.picture->m_uEnerHpCtu[ctbIdx] / lambdaChromaWeight : m_lambda[compId];
#else
          const double ctuLambda = m_lambda[compId];
#endif
          float dist        = MAX_FLOAT;
          int   numAlts     = m_alfParamTemp.numAlternativesChroma;
          ctxTempBest       = AlfCtx(m_CABACEstimator->getCtx());
          float bestAltRate = 0;
          float bestAltCost = MAX_FLOAT;
          int   bestAltIdx  = -1;
          ctxTempAltStart   = AlfCtx(ctxTempBest);
          for (int altIdx = 0; altIdx < numAlts; ++altIdx)
          {
            ChromaCtbModeHandler::setAlternative((*m_modes)[compId][ctbIdx], altIdx);

            if (altIdx)
            {
              m_CABACEstimator->getCtx() = AlfCtx(ctxTempAltStart);
            }
            m_CABACEstimator->resetBits();
            m_CABACEstimator->codeAlfCtuChromaAlternative(cs, ctbIdx, static_cast<CompID>(compId), m_alfParamTemp);
            float altRate  = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
            float rAltCost = ctuLambda * altRate;

            // distortion
            for (int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++)
            {
              m_filterTmp[i] = m_chromaCoeffFinal[altIdx][i];
              m_clipTmp[i]   = m_chromaClippFinal[altIdx][i];
            }

            const AlfCovariance &cov = m_alfCovariance[compId][0][ctbIdx][0];

            float altDist =
              cov.calcErrorForCoeffs(m_clipTmp, m_filterTmp, MAX_NUM_ALF_CHROMA_COEFF, 1, COEFF_SCALE_BITS_ALF);
            float altCost = altDist + rAltCost;
            if (altCost < bestAltCost)
            {
              bestAltCost = altCost;
              bestAltIdx  = altIdx;
              bestAltRate = altRate;
              ctxTempBest = AlfCtx(m_CABACEstimator->getCtx());
              dist        = altDist;
            }
          }
          rateOn += bestAltRate;
          dist += distUnfilterCtu;
          // cost
          float costOn = dist + ctuLambda * rateOn;
          // cost off
          CtbModeHandler::setDisabled((*m_modes)[compId][ctbIdx]);
          // rate
          m_CABACEstimator->getCtx() = AlfCtx(ctxTempStart);
          m_CABACEstimator->resetBits();
          m_CABACEstimator->codeAlfCtuEnableFlag(cs, ctbIdx, compId, &m_alfParamTemp);
          // cost
          float costOff = distUnfilterCtu + m_lambda[compId] * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
          if (costOn < costOff)
          {
            m_CABACEstimator->getCtx() = AlfCtx(ctxTempBest);
            ChromaCtbModeHandler::setAlternative((*m_modes)[compId][ctbIdx], bestAltIdx);
            curCost += costOn;
          }
          else
          {
            curCost += costOff;
          }
        }
      }
      // chroma idc
      setSliceEnabledFlag(m_alfParamTemp, ChannelType::CHROMA, m_modes);

      if (curCost < costMin)
      {
        costMin                             = curCost;
        cs.slice->m_alfApsIdChroma          = curApsId;
        cs.slice->m_alfEnabledFlag[COMP_Cb] = m_alfParamTemp.enabledFlag[COMP_Cb];
        cs.slice->m_alfEnabledFlag[COMP_Cr] = m_alfParamTemp.enabledFlag[COMP_Cr];
        copyIndices(m_indexTmp, m_modes, ChannelType::CHROMA);
      }
    }

    if (newApsIdChroma >= 0)
    {
      cs.slice->m_ccAlfCbApsId = newApsIdChroma;
      cs.slice->m_ccAlfCrApsId = newApsIdChroma;
    }
    if (costOff < costMin)
    {
      cs.slice->m_alfEnabledFlag[COMP_Cb] = false;
      cs.slice->m_alfEnabledFlag[COMP_Cr] = false;
      setCtuEnableFlag(m_modes, ChannelType::CHROMA, CTB_MODE_OFF);
    }
    else
    {
      copyIndices(m_modes, m_indexTmp, ChannelType::CHROMA);
      if (cs.slice->m_alfApsIdChroma == newApsIdChroma)   // new filter
      {
        APS *newAPS = m_apsMap->getPS(newApsIdChroma);
        if (newAPS == nullptr)
        {
          newAPS            = m_apsMap->allocatePS(newApsIdChroma);
          newAPS->m_APSType = ApsType::ALF;
          newAPS->m_APSId   = newApsIdChroma;
          newAPS->m_alfAPSParam.getVtmParam().reset();
        }

        AlfParam &alfAPSParam = newAPS->m_alfAPSParam.getVtmParam();

        alfAPSParam.newFilterFlag[ChannelType::CHROMA] = true;
        if (!alfParamNewFiltersBest.newFilterFlag[ChannelType::LUMA])
        {
          alfAPSParam.newFilterFlag[ChannelType::LUMA] = false;
        }
        alfAPSParam.numAlternativesChroma = alfParamNewFilters.numAlternativesChroma;
        alfAPSParam.chromaNonLinearFlag   = alfParamNewFilters.chromaNonLinearFlag;
        newAPS->m_temporalId              = cs.slice->m_uiTLayer;
        for (int altIdx = 0; altIdx < ALF_MAX_NUM_ALTERNATIVES_CHROMA; ++altIdx)
        {
          for (int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++)
          {
            alfAPSParam.chromaCoeff[altIdx][i] = alfParamNewFilters.chromaCoeff[altIdx][i];
            alfAPSParam.chromaClipp[altIdx][i] = alfParamNewFilters.chromaClipp[altIdx][i];
          }
        }
        m_apsMap->setChangedFlag(newApsIdChroma);
        m_apsIdStart = newApsIdChroma;
      }
      apss[cs.slice->m_alfApsIdChroma] = m_apsMap->getPS(cs.slice->m_alfApsIdChroma);
    }
  }
}

void EncAdaptiveLoopFilterVtm::alfReconstructor(CodingStructure &cs, const PelUnitBuf &recExtBuf)
{
  if (!cs.slice->m_alfEnabledFlag[COMP_Y])
  {
    return;
  }
  reconstructCoeffAPSs(cs, true, cs.slice->m_alfEnabledFlag[COMP_Cb] || cs.slice->m_alfEnabledFlag[COMP_Cr], false);
  PelUnitBuf          &recBuf = cs.getRecoBufRef();
  const PreCalcValues &pcv    = *cs.pcv;

  int  ctuIdx  = 0;
  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
  for (int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
  {
    for (int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
    {
      const int width  = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
      const int height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;

      bool ctuEnableFlag = CtbModeHandler::isEnabled((*m_modes)[COMP_Y][ctuIdx]);
      for (int compIdx = 1; compIdx < MAX_NUM_COMP; compIdx++)
      {
        ctuEnableFlag |= CtbModeHandler::isEnabled((*m_modes)[compIdx][ctuIdx]);
      }
      int rasterSliceAlfPad = 0;
      if (ctuEnableFlag &&
          isCrossedBySubPicBoundaries(cs, xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight,
                                      rasterSliceAlfPad))
      {
        const bool clipT = clipTop || (yPos == 0);
        const bool clipB = clipBottom || (yPos + height == pcv.lumaHeight);
        const bool clipL = clipLeft || (xPos == 0);
        const bool clipR = clipRight || (xPos + width == pcv.lumaWidth);
        const int  wBuf  = width + (clipL ? 0 : MAX_ALF_PADDING_SIZE) + (clipR ? 0 : MAX_ALF_PADDING_SIZE);
        const int  hBuf  = height + (clipT ? 0 : MAX_ALF_PADDING_SIZE) + (clipB ? 0 : MAX_ALF_PADDING_SIZE);

        PelUnitBuf buf = m_tempBuf2.subBuf(UnitArea(cs.area.chromaFormat, Area(0, 0, wBuf, hBuf)));
        buf.copyFrom(recExtBuf.subBuf(UnitArea(
          cs.area.chromaFormat,
          Area(xPos - (clipL ? 0 : MAX_ALF_PADDING_SIZE), yPos - (clipT ? 0 : MAX_ALF_PADDING_SIZE), wBuf, hBuf))));
        // pad top-left unavailable samples for raster slice
        if (rasterSliceAlfPad & 1)
        {
          buf.padBorderPel(MAX_ALF_PADDING_SIZE, 1);
        }

        // pad bottom-right unavailable samples for raster slice
        if (rasterSliceAlfPad & 2)
        {
          buf.padBorderPel(MAX_ALF_PADDING_SIZE, 2);
        }
        buf.extendBorderPel(MAX_ALF_PADDING_SIZE);
        buf =
          buf.subBuf(UnitArea(cs.area.chromaFormat,
                              Area(clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, width, height)));

        if (CtbModeHandler::isEnabled((*m_modes)[COMP_Y][ctuIdx]))
        {
          const Area blkSrc(0, 0, width, height);
          const Area blkDst(xPos, yPos, width, height);
          const int  m = (*m_modes)[COMP_Y][ctuIdx];
          m_filter7x7Blk(m_classifier, recBuf, buf, blkDst, blkSrc, COMP_Y, getCoeffVals(m), getClipVals(m),
                         m_clpRngs.comp[COMP_Y], m_alfVBLumaCTUHeight, m_alfVBLumaPos);
        }

        const int chromaScaleX = getChannelTypeScaleX(ChannelType::CHROMA, recBuf.chromaFormat);
        const int chromaScaleY = getChannelTypeScaleY(ChannelType::CHROMA, recBuf.chromaFormat);

        for (int compIdx = 1; compIdx < MAX_NUM_COMP; compIdx++)
        {
          CompID compID = CompID(compIdx);
          if (CtbModeHandler::isEnabled((*m_modes)[compIdx][ctuIdx]))
          {
            const Area     blkSrc(0, 0, width >> chromaScaleX, height >> chromaScaleY);
            const Area     blkDst(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX,
                                  height >> chromaScaleY);
            const unsigned altNum = ChromaCtbModeHandler::getAlternative((*m_modes)[compID][ctuIdx]);
            m_filter5x5Blk(m_classifier, recBuf, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[altNum],
                           m_chromaClippFinal[altNum], m_clpRngs.comp[compIdx], m_alfVBChmaCTUHeight, m_alfVBChmaPos);
          }
        }
      }
      else
      {
        const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));
        if (CtbModeHandler::isEnabled((*m_modes)[COMP_Y][ctuIdx]))
        {
          Area      blk(xPos, yPos, width, height);
          const int m = (*m_modes)[COMP_Y][ctuIdx];
          m_filter7x7Blk(m_classifier, recBuf, recExtBuf, blk, blk, COMP_Y, getCoeffVals(m), getClipVals(m),
                         m_clpRngs.comp[COMP_Y], m_alfVBLumaCTUHeight, m_alfVBLumaPos);
        }

        const int chromaScaleX = getChannelTypeScaleX(ChannelType::CHROMA, recBuf.chromaFormat);
        const int chromaScaleY = getChannelTypeScaleY(ChannelType::CHROMA, recBuf.chromaFormat);

        for (int compIdx = 1; compIdx < MAX_NUM_COMP; compIdx++)
        {
          CompID compID = CompID(compIdx);
          if (CtbModeHandler::isEnabled((*m_modes)[compIdx][ctuIdx]))
          {
            Area blk(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY);
            const unsigned altIdx = ChromaCtbModeHandler::getAlternative((*m_modes)[compID][ctuIdx]);
            m_filter5x5Blk(m_classifier, recBuf, recExtBuf, blk, blk, compID, m_chromaCoeffFinal[altIdx],
                           m_chromaClippFinal[altIdx], m_clpRngs.comp[compIdx], m_alfVBChmaCTUHeight, m_alfVBChmaPos);
          }
        }
      }
      ctuIdx++;
    }
  }
}

void EncAdaptiveLoopFilterVtm::initCtuAlternativeChroma(CtuModes &ctuModes) const
{
  CtbModes &ctbModesCb = ctuModes[CompID::COMP_Cb];
  CtbModes &ctbModesCr = ctuModes[CompID::COMP_Cr];
  CHECKD(ctbModesCb.size() != m_numCTUsInPic, "Unexpected number of entries in CTB modes (Cb).");
  CHECKD(ctbModesCr.size() != m_numCTUsInPic, "Unexpected number of entries in CTB modes (Cr).");

  int altIdx = 0;
  for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ++ctuIdx)
  {
    ChromaCtbModeHandler::setAlternative(ctbModesCb[ctuIdx], altIdx);
    ChromaCtbModeHandler::setAlternative(ctbModesCr[ctuIdx], altIdx);
    if ((ctuIdx + 1) * m_alfParamTemp.numAlternativesChroma >= (altIdx + 1) * m_numCTUsInPic)
    {
      ++altIdx;
    }
  }
}

void EncAdaptiveLoopFilterVtm::deriveCcAlfFilterCoeff(
  CompID compID, const PelUnitBuf &recYuv, const PelUnitBuf &recYuvExt,
  short filterCoeff[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF], const uint8_t filterIdx)
{
  EncAdaptiveLoopFilterBase<AlfParametersVtm>::deriveCcAlfFilterCoeff(
    compID, recYuv, recYuvExt, filterCoeff, filterIdx, m_alfCovarianceFrameCcAlf[0], m_filterShapesCcAlf[0].numCoeff);
}

void EncAdaptiveLoopFilterVtm::getFrameStatsCcalf(CompID compIdx, int filterIdc)
{
  EncAdaptiveLoopFilterBase<AlfParametersVtm>::getFrameStatsCcalf(
    m_alfCovarianceFrameCcAlf, m_alfCovarianceCcAlf, compIdx, filterIdc, static_cast<int>(m_filterShapesCcAlf.size()));
}

void EncAdaptiveLoopFilterVtm::deriveCcAlfFilter(CodingStructure &cs, CompID compID, const PelUnitBuf &orgYuv,
                                                 const PelUnitBuf &tempDecYuvBuf, const PelUnitBuf &dstYuv)
{
  if (!cs.slice->m_alfEnabledFlag[COMP_Y])
  {
    m_ccAlfFilterParam.ccAlfFilterEnabled[compID - 1] = false;
    return;
  }

  m_limitCcAlf = m_encCfg->m_iQP >= m_encCfg->m_ccalfQpThreshold;
  if (m_limitCcAlf && cs.slice->m_iSliceQp <= m_encCfg->m_iQP + 1)
  {
    m_ccAlfFilterParam.ccAlfFilterEnabled[compID - 1] = false;
    return;
  }

  if (m_alfWSSD)
  {
    deriveStatsForCcAlfFiltering<true>(orgYuv, tempDecYuvBuf, compID, cs);
  }
  else
  {
    deriveStatsForCcAlfFiltering<false>(orgYuv, tempDecYuvBuf, compID, cs);
  }
  initDistortionCcalf(compID);

  uint8_t   bestMapFilterIdxToFilterIdc[MAX_NUM_CC_ALF_FILTERS + 1];
  const int scaleX               = getComponentScaleX(compID, cs.pcv->chrFormat);
  const int scaleY               = getComponentScaleY(compID, cs.pcv->chrFormat);
  const int ctuWidthC            = cs.pcv->maxCUWidth >> scaleX;
  const int ctuHeightC           = cs.pcv->maxCUHeight >> scaleY;
  const int picWidthC            = cs.pcv->lumaWidth >> scaleX;
  const int picHeightC           = cs.pcv->lumaHeight >> scaleY;
  const int maxTrainingIterCount = 15;

  if (m_limitCcAlf)
  {
    countLumaSwingGreaterThanThreshold(dstYuv.get(COMP_Y).bufAt(0, 0), dstYuv.get(COMP_Y).stride,
                                       dstYuv.get(COMP_Y).height, dstYuv.get(COMP_Y).width, cs.pcv->maxCUWidthLog2,
                                       cs.pcv->maxCUHeightLog2, m_lumaSwingGreaterThanThresholdCount, m_numCTUsInWidth);
  }
  if (m_limitCcAlf)
  {
    countChromaSampleValueNearMidPoint(dstYuv.get(compID).bufAt(0, 0), dstYuv.get(compID).stride,
                                       dstYuv.get(compID).height, dstYuv.get(compID).width,
                                       cs.pcv->maxCUWidthLog2 - scaleX, cs.pcv->maxCUHeightLog2 - scaleY,
                                       m_chromaSampleCountNearMidPoint, m_numCTUsInWidth);
  }

  for (int filterIdx = 0; filterIdx <= MAX_NUM_CC_ALF_FILTERS; filterIdx++)
  {
    if (filterIdx < MAX_NUM_CC_ALF_FILTERS)
    {
      memset(m_bestFilterCoeffSet[filterIdx], 0, sizeof(m_bestFilterCoeffSet[filterIdx]));
      bestMapFilterIdxToFilterIdc[filterIdx] = filterIdx + 1;
    }
    else
    {
      bestMapFilterIdxToFilterIdc[filterIdx] = 0;
    }
  }
  memset(m_bestFilterControl, 0, sizeof(uint8_t) * m_numCTUsInPic);
  int ccalfReuseApsId      = -1;
  m_reuseApsId[compID - 1] = -1;
  m_bestFilterCount        = 0;

  const TempCtx ctxStartCcAlfFilterControlFlag(m_ctxPool,
                                               SubCtx(Ctx::CcAlfFilterControlFlag, m_CABACEstimator->getCtx()));

  // compute cost of not filtering
  uint64_t unfilteredDistortion = 0;
  for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
  {
    unfilteredDistortion += (uint64_t)m_alfCovarianceCcAlf[0][ctbIdx].pixAcc;
  }

  float bestUnfilteredTotalCost = 1 * m_lambda[compID] + unfilteredDistortion;   // 1 bit is for gating flag

  bool             ccAlfFilterIdxEnabled[MAX_NUM_CC_ALF_FILTERS];
  short            ccAlfFilterCoeff[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF];
  uint8_t          ccAlfFilterCount             = MAX_NUM_CC_ALF_FILTERS;
  float            bestFilteredTotalCost        = MAX_FLOAT;
  bool             bestreuseTemporalFilterCoeff = false;
  std::vector<int> apsIds                       = getAvailableCcAlfApsIds(cs, compID);

  for (int testFilterIdx = 0; testFilterIdx < apsIds.size() + 1; ++testFilterIdx)
  {
    bool referencingExistingAps        = (testFilterIdx < apsIds.size()) ? true : false;
    int  maxNumberOfFiltersBeingTested = MAX_NUM_CC_ALF_FILTERS - (testFilterIdx - static_cast<int>(apsIds.size()));

    if (maxNumberOfFiltersBeingTested < 0)
    {
      maxNumberOfFiltersBeingTested = 1;
    }

    {
      // Instead of rewriting the control buffer for every training iteration just keep a mapping from filterIdx to
      // filterIdc
      uint8_t mapFilterIdxToFilterIdc[MAX_NUM_CC_ALF_FILTERS + 1];
      for (int filterIdx = 0; filterIdx <= MAX_NUM_CC_ALF_FILTERS; filterIdx++)
      {
        if (filterIdx == MAX_NUM_CC_ALF_FILTERS)
        {
          mapFilterIdxToFilterIdc[filterIdx] = 0;
        }
        else
        {
          mapFilterIdxToFilterIdc[filterIdx] = filterIdx + 1;
        }
      }

      // initialize filters
      for (int filterIdx = 0; filterIdx < MAX_NUM_CC_ALF_FILTERS; filterIdx++)
      {
        ccAlfFilterIdxEnabled[filterIdx] = false;
        memset(ccAlfFilterCoeff[filterIdx], 0, sizeof(ccAlfFilterCoeff[filterIdx]));
      }
      if (referencingExistingAps)
      {
        const CcAlfFilterParam &ccAlfAPSParam = m_apsMap->getPS(apsIds[testFilterIdx])->m_ccAlfAPSParam.getVtmParam();

        maxNumberOfFiltersBeingTested = ccAlfAPSParam.ccAlfFilterCount[compID - 1];
        ccAlfFilterCount              = maxNumberOfFiltersBeingTested;
        for (int filterIdx = 0; filterIdx < maxNumberOfFiltersBeingTested; filterIdx++)
        {
          ccAlfFilterIdxEnabled[filterIdx] = true;
          memcpy(ccAlfFilterCoeff[filterIdx], m_ccAlfFilterParam.ccAlfCoeff[compID - 1][filterIdx],
                 sizeof(ccAlfFilterCoeff[filterIdx]));
        }
        memcpy(ccAlfFilterCoeff, ccAlfAPSParam.ccAlfCoeff[compID - 1], sizeof(ccAlfFilterCoeff));
      }
      else
      {
        for (int i = 0; i < maxNumberOfFiltersBeingTested; i++)
        {
          ccAlfFilterIdxEnabled[i] = true;
        }
        ccAlfFilterCount = maxNumberOfFiltersBeingTested;
      }

      // initialize
      int       controlIdx = 0;
      const int columnSize = (m_buf->width / maxNumberOfFiltersBeingTested);
      for (int y = 0; y < m_buf->height; y += ctuHeightC)
      {
        for (int x = 0; x < m_buf->width; x += ctuWidthC)
        {
          m_trainingCovControl[controlIdx] = (x / columnSize) + 1;
          controlIdx++;
        }
      }

      // compute cost of filtering
      int   trainingIterCount = 0;
      bool  keepTraining      = true;
      bool  improvement       = false;
      float prevTotalCost     = MAX_FLOAT;
      while (keepTraining)
      {
        improvement = false;
        for (int filterIdx = 0; filterIdx < maxNumberOfFiltersBeingTested; filterIdx++)
        {
          if (ccAlfFilterIdxEnabled[filterIdx])
          {
            if (!referencingExistingAps)
            {
              getFrameStatsCcalf(compID, (filterIdx + 1));
              deriveCcAlfFilterCoeff(compID, dstYuv, tempDecYuvBuf, ccAlfFilterCoeff, filterIdx);
            }
            const int numCoeff        = m_filterShapesCcAlf[0].numCoeff - 1;
            int       log2BlockWidth  = cs.pcv->maxCUWidthLog2 - scaleX;
            int       log2BlockHeight = cs.pcv->maxCUHeightLog2 - scaleY;
            for (int y = 0; y < m_buf->height; y += (1 << log2BlockHeight))
            {
              for (int x = 0; x < m_buf->width; x += (1 << log2BlockWidth))
              {
                int ctuIdx = (y >> log2BlockHeight) * m_numCTUsInWidth + (x >> log2BlockWidth);
                m_trainingDistortion[filterIdx][ctuIdx] =
                  int(m_ctbDistortionUnfilter[compID][ctuIdx] +
                      m_alfCovarianceCcAlf[0][ctuIdx].calcErrorForCcAlfCoeffs(ccAlfFilterCoeff[filterIdx], numCoeff,
                                                                              COEFF_SCALE_BITS_CCALF + 1));
              }
            }
          }
        }

        m_CABACEstimator->getCtx() = ctxStartCcAlfFilterControlFlag;

        uint64_t curTotalDistortion = 0;
        float    curTotalRate       = 0;
        determineControlIdcValues(cs, compID, m_buf, ctuWidthC, ctuHeightC, picWidthC, picHeightC,
                                  m_ctbDistortionUnfilter, m_trainingDistortion, m_lumaSwingGreaterThanThresholdCount,
                                  m_chromaSampleCountNearMidPoint, (referencingExistingAps == true),
                                  m_trainingCovControl, m_filterControl, curTotalDistortion, curTotalRate,
                                  ccAlfFilterIdxEnabled, mapFilterIdxToFilterIdc, ccAlfFilterCount);

        // compute coefficient coding bit cost
        if (ccAlfFilterCount > 0)
        {
          if (referencingExistingAps)
          {
            curTotalRate += 1 + 3;   // +1 for enable flag, +3 APS ID in slice header
          }
          else
          {
            curTotalRate += getCoeffRateCcAlf(ccAlfFilterCoeff, ccAlfFilterIdxEnabled, ccAlfFilterCount, compID) + 1 +
              9;   // +1 for the enable flag, +9 3-bit for APS ID in slice header, 5-bit for APS ID in APS, a 1-bit
            // new filter flags (ignore shared cost such as other new-filter flags/NALU header/RBSP
            // terminating bit/byte alignment bits)
          }

          float curTotalCost = curTotalRate * m_lambda[compID] + curTotalDistortion;

          if (curTotalCost < prevTotalCost)
          {
            prevTotalCost = curTotalCost;
            improvement   = true;
          }

          if (curTotalCost < bestFilteredTotalCost)
          {
            bestFilteredTotalCost = curTotalCost;
            memcpy(m_bestFilterIdxEnabled, ccAlfFilterIdxEnabled, sizeof(ccAlfFilterIdxEnabled));
            memcpy(m_bestFilterCoeffSet, ccAlfFilterCoeff, sizeof(ccAlfFilterCoeff));
            memcpy(m_bestFilterControl, m_filterControl, sizeof(uint8_t) * m_numCTUsInPic);
            m_bestFilterCount = ccAlfFilterCount;
            ccalfReuseApsId   = referencingExistingAps ? apsIds[testFilterIdx] : -1;
            memcpy(bestMapFilterIdxToFilterIdc, mapFilterIdxToFilterIdc, sizeof(mapFilterIdxToFilterIdc));
          }
        }

        trainingIterCount++;
        if (!improvement || trainingIterCount > maxTrainingIterCount || referencingExistingAps)
        {
          keepTraining = false;
        }
      }
    }
  }

  if (bestUnfilteredTotalCost < bestFilteredTotalCost)
  {
    memset(m_bestFilterControl, 0, sizeof(uint8_t) * m_numCTUsInPic);
  }

  // save best coeff and control
  bool atleastOneBlockUndergoesFitlering = false;
  for (int controlIdx = 0; m_bestFilterCount > 0 && controlIdx < m_numCTUsInPic; controlIdx++)
  {
    if (m_bestFilterControl[controlIdx])
    {
      atleastOneBlockUndergoesFitlering = true;
      break;
    }
  }
  m_ccAlfFilterParam.numberValidComponents          = getNumberValidComponents(m_chromaFormat);
  m_ccAlfFilterParam.ccAlfFilterEnabled[compID - 1] = atleastOneBlockUndergoesFitlering;
  if (atleastOneBlockUndergoesFitlering)
  {
    // update the filter control indicators
    if (bestreuseTemporalFilterCoeff != 1)
    {
      short storedBestFilterCoeffSet[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF];
      for (int filterIdx = 0; filterIdx < MAX_NUM_CC_ALF_FILTERS; filterIdx++)
      {
        memcpy(storedBestFilterCoeffSet[filterIdx], m_bestFilterCoeffSet[filterIdx],
               sizeof(m_bestFilterCoeffSet[filterIdx]));
      }
      memcpy(m_filterControl, m_bestFilterControl, sizeof(uint8_t) * m_numCTUsInPic);

      int filterCount = 0;
      for (int filterIdx = 0; filterIdx < MAX_NUM_CC_ALF_FILTERS; filterIdx++)
      {
        uint8_t curFilterIdc = bestMapFilterIdxToFilterIdc[filterIdx];
        if (m_bestFilterIdxEnabled[filterIdx])
        {
          for (int controlIdx = 0; controlIdx < m_numCTUsInPic; controlIdx++)
          {
            if (m_filterControl[controlIdx] == (filterIdx + 1))
            {
              m_bestFilterControl[controlIdx] = curFilterIdc;
            }
          }
          memcpy(m_bestFilterCoeffSet[curFilterIdc - 1], storedBestFilterCoeffSet[filterIdx],
                 sizeof(storedBestFilterCoeffSet[filterIdx]));
          filterCount++;
        }
        m_bestFilterIdxEnabled[filterIdx] = (filterIdx < m_bestFilterCount) ? true : false;
      }
      CHECK(filterCount != m_bestFilterCount, "Number of filters enabled did not match the filter count");
    }

    m_ccAlfFilterParam.ccAlfFilterCount[compID - 1] = m_bestFilterCount;
    // cleanup before copying
    memset(m_ccAlfFilterControl[compID - 1], 0, sizeof(uint8_t) * m_numCTUsInPic);
    for (int filterIdx = 0; filterIdx < MAX_NUM_CC_ALF_FILTERS; filterIdx++)
    {
      memset(m_ccAlfFilterParam.ccAlfCoeff[compID - 1][filterIdx], 0,
             sizeof(m_ccAlfFilterParam.ccAlfCoeff[compID - 1][filterIdx]));
    }
    memset(m_ccAlfFilterParam.ccAlfFilterIdxEnabled[compID - 1], false,
           sizeof(m_ccAlfFilterParam.ccAlfFilterIdxEnabled[compID - 1]));
    for (int filterIdx = 0; filterIdx < m_bestFilterCount; filterIdx++)
    {
      m_ccAlfFilterParam.ccAlfFilterIdxEnabled[compID - 1][filterIdx] = m_bestFilterIdxEnabled[filterIdx];
      memcpy(m_ccAlfFilterParam.ccAlfCoeff[compID - 1][filterIdx], m_bestFilterCoeffSet[filterIdx],
             sizeof(m_bestFilterCoeffSet[filterIdx]));
    }
    memcpy(m_ccAlfFilterControl[compID - 1], m_bestFilterControl, sizeof(uint8_t) * m_numCTUsInPic);
    if (ccalfReuseApsId >= 0)
    {
      m_reuseApsId[compID - 1] = ccalfReuseApsId;
      if (compID == COMP_Cb)
      {
        cs.slice->m_ccAlfCbApsId = ccalfReuseApsId;
      }
      else
      {
        cs.slice->m_ccAlfCrApsId = ccalfReuseApsId;
      }
    }
  }
}

template<bool alfWSSD>
void EncAdaptiveLoopFilterVtm::deriveStatsForCcAlfFiltering(const PelUnitBuf &orgYuv, const PelUnitBuf &recYuv,
                                                            const int compIdx, CodingStructure &cs)
{
  // init CTU stats buffers
  for (int shape = 0; shape != m_filterShapesCcAlf.size(); ++shape)
  {
    for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++)
    {
      m_alfCovarianceCcAlf[shape][ctuIdx].reset();
    }
  }

  // init Frame stats buffers
  for (int shape = 0; shape != m_filterShapesCcAlf.size(); ++shape)
  {
    m_alfCovarianceFrameCcAlf[shape].reset();
  }

  const PreCalcValues &pcv     = *cs.pcv;
  bool                 clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;

  for (int yPos = 0, ctuRsAddr = 0; yPos < m_picHeight; yPos += m_maxCUHeight)
  {
    for (int xPos = 0; xPos < m_picWidth; xPos += m_maxCUWidth, ++ctuRsAddr)
    {
      const int width             = std::min(m_picWidth - xPos, m_maxCUWidth);
      const int height            = std::min(m_picHeight - yPos, m_maxCUHeight);
      int       rasterSliceAlfPad = 0;
      if (isCrossedBySubPicBoundaries(cs, xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight,
                                      rasterSliceAlfPad))
      {
        const bool clipT = clipTop || (yPos == 0);
        const bool clipB = clipBottom || (yPos + height == pcv.lumaHeight);
        const bool clipL = clipLeft || (xPos == 0);
        const bool clipR = clipRight || (xPos + width == pcv.lumaWidth);
        const int  wBuf  = width + (clipL ? 0 : MAX_ALF_PADDING_SIZE) + (clipR ? 0 : MAX_ALF_PADDING_SIZE);
        const int  hBuf  = height + (clipT ? 0 : MAX_ALF_PADDING_SIZE) + (clipB ? 0 : MAX_ALF_PADDING_SIZE);

        PelUnitBuf recBuf = m_tempBuf2.subBuf(UnitArea(cs.area.chromaFormat, Area(0, 0, wBuf, hBuf)));
        recBuf.copyFrom(recYuv.subBuf(UnitArea(
          cs.area.chromaFormat,
          Area(xPos - (clipL ? 0 : MAX_ALF_PADDING_SIZE), yPos - (clipT ? 0 : MAX_ALF_PADDING_SIZE), wBuf, hBuf))));
        // pad top-left unavailable samples for raster slice
        if (rasterSliceAlfPad & 1)
        {
          recBuf.padBorderPel(MAX_ALF_PADDING_SIZE, 1);
        }

        // pad bottom-right unavailable samples for raster slice
        if (rasterSliceAlfPad & 2)
        {
          recBuf.padBorderPel(MAX_ALF_PADDING_SIZE, 2);
        }
        recBuf.extendBorderPel(MAX_ALF_PADDING_SIZE);
        recBuf = recBuf.subBuf(
          UnitArea(cs.area.chromaFormat,
                   Area(clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, width, height)));

        const UnitArea area(m_chromaFormat, Area(0, 0, width, height));
        const UnitArea areaDst(m_chromaFormat, Area(xPos, yPos, width, height));

        const CompID compID = CompID(compIdx);

        for (int shape = 0; shape != m_filterShapesCcAlf.size(); ++shape)
        {
          getBlkStatsCcAlf<alfWSSD>(m_alfCovarianceCcAlf[0][ctuRsAddr], m_filterShapesCcAlf[shape], orgYuv, recBuf,
                                    areaDst, area, compID, yPos);
          m_alfCovarianceFrameCcAlf[shape] += m_alfCovarianceCcAlf[shape][ctuRsAddr];
        }
      }
      else
      {
        const UnitArea area(m_chromaFormat, Area(xPos, yPos, width, height));

        const CompID compID = CompID(compIdx);

        for (int shape = 0; shape != m_filterShapesCcAlf.size(); ++shape)
        {
          getBlkStatsCcAlf<alfWSSD>(m_alfCovarianceCcAlf[0][ctuRsAddr], m_filterShapesCcAlf[shape], orgYuv, recYuv,
                                    area, area, compID, yPos);
          m_alfCovarianceFrameCcAlf[shape] += m_alfCovarianceCcAlf[shape][ctuRsAddr];
        }
      }
    }
  }
}

template<bool alfWSSD>
void EncAdaptiveLoopFilterVtm::getBlkStatsCcAlf(AlfCovariance &alfCovariance, const AlfFilterShape &shape,
                                                const PelUnitBuf &orgYuv, const PelUnitBuf &recYuv,
                                                const UnitArea &areaDst, const UnitArea &area, const CompID compID,
                                                const int yPos)
{
  const int       numberOfComponents = getNumberValidComponents(m_chromaFormat);
  const CompArea &compArea           = areaDst.block(compID);
  ptrdiff_t       recStride[MAX_NUM_COMP];
  const Pel      *rec[MAX_NUM_COMP];
  for (int cIdx = 0; cIdx < numberOfComponents; cIdx++)
  {
    recStride[cIdx] = recYuv.get(CompID(cIdx)).stride;
    rec[cIdx]       = recYuv.get(CompID(cIdx)).bufAt(isLuma(CompID(cIdx)) ? area.lumaPos() : area.chromaPos());
  }

  ptrdiff_t  orgStride = orgYuv.get(compID).stride;
  const Pel *org       = orgYuv.get(compID).bufAt(compArea);

  const ptrdiff_t orgLumaStride = orgYuv.get(COMP_Y).stride;
  const Pel      *orgLuma       = orgYuv.get(COMP_Y).bufAt(areaDst.block(COMP_Y));

  int vbCTUHeight = m_alfVBLumaCTUHeight;
  int vbPos       = m_alfVBLumaPos;
  if ((yPos + m_maxCUHeight) >= m_picHeight)
  {
    vbPos = m_picHeight;
  }

  Pel ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][1];

  const float strength    = m_encCfg->m_ccalfStrengthTarget;
  const float invStrength = strength != 0.0f ? 1.0f / strength : 0.0f;

  for (int i = 0; i < compArea.height; i++)
  {
    const int iY = i << getComponentScaleY(compID, m_chromaFormat);

    const int  vbDistance  = (iY % vbCTUHeight) - vbPos;
    const bool skipThisRow = getComponentScaleY(compID, m_chromaFormat) == 0 && (vbDistance == 0 || vbDistance == 1);
    for (int j = 0; j < compArea.width && (!skipThisRow); j++)
    {
      const int jY = j << getComponentScaleX(compID, m_chromaFormat);

      std::memset(ELocal, 0, sizeof(ELocal));

      calcCovarianceCcAlf(ELocal, rec[COMP_Y] + jY, recStride[COMP_Y], shape, vbDistance);

      const Pel *lumaPtr = orgLuma + iY * orgLumaStride + jY;

      const float weight = alfWSSD ? m_lumaLevelToWeightPLUT[*lumaPtr] : 1.0f;
      const float yLocal = org[j] - rec[compID][j];

      float e[MAX_NUM_CC_ALF_CHROMA_COEFF];

      for (int k = 0; k < shape.numCoeff - 1; k++)
      {
        e[k] = invStrength * ELocal[k][0];
      }

      for (int k = 0; k < (shape.numCoeff - 1); k++)
      {
        const float we = weight * e[k];

        for (int l = k; l < (shape.numCoeff - 1); l++)
        {
          alfCovariance.E(0, 0, k, l) += we * e[l];
        }
        alfCovariance.y(0, k) += we * yLocal;
      }
      alfCovariance.pixAcc += weight * yLocal * yLocal;
    }
    org += orgStride;
    for (int srcCIdx = 0; srcCIdx < numberOfComponents; srcCIdx++)
    {
      CompID srcCompID = CompID(srcCIdx);
      if (toChannelType(srcCompID) == toChannelType(compID))
      {
        rec[srcCIdx] += recStride[srcCIdx];
      }
      else
      {
        if (isLuma(compID))
        {
          rec[srcCIdx] += (recStride[srcCIdx] >> getComponentScaleY(srcCompID, m_chromaFormat));
        }
        else
        {
          rec[srcCIdx] += (recStride[srcCIdx] << getComponentScaleY(compID, m_chromaFormat));
        }
      }
    }
  }
}

void EncAdaptiveLoopFilterVtm::calcCovarianceCcAlf(Pel ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][1], const Pel *rec,
                                                   const ptrdiff_t stride, const AlfFilterShape &shape, int vbDistance)
{
  CHECK(shape.filterType != CC_ALF, "Bad CC ALF shape");

  const Pel *recYM1 = rec - 1 * stride;
  const Pel *recY0  = rec;
  const Pel *recYP1 = rec + 1 * stride;
  const Pel *recYP2 = rec + 2 * stride;

  if (vbDistance == -2 || vbDistance == +1)
  {
    recYP2 = recYP1;
  }
  else if (vbDistance == -1 || vbDistance == 0)
  {
    recYM1 = recY0;
    recYP2 = recYP1 = recY0;
  }

  for (int b = 0; b < 1; b++)
  {
    const Pel centerValue = recY0[+0];

    ELocal[0][b] += recYM1[+0] - centerValue;
    ELocal[1][b] += recY0[-1] - centerValue;
    ELocal[2][b] += recY0[+1] - centerValue;
    ELocal[3][b] += recYP1[-1] - centerValue;
    ELocal[4][b] += recYP1[+0] - centerValue;
    ELocal[5][b] += recYP1[+1] - centerValue;
    ELocal[6][b] += recYP2[+0] - centerValue;
  }
}

void EncAdaptiveLoopFilterVtm::countLumaSwingGreaterThanThreshold(const Pel *luma, ptrdiff_t lumaStride, int height,
                                                                  int width, int log2BlockWidth, int log2BlockHeight,
                                                                  uint64_t *lumaSwingGreaterThanThresholdCount,
                                                                  int       lumaCountStride)
{
  const int lumaBitDepth = m_inputBitDepth[ChannelType::LUMA];
  const int threshold    = (1 << (m_inputBitDepth[ChannelType::LUMA] - 2)) - 1;

  // clang-format off
  // 3x4 Diamond
  static constexpr int xSupport[] = {  0, -1,  0,  1, -1,  0,  1,  0 };
  static constexpr int ySupport[] = { -1,  0,  0,  0,  1,  1,  1,  2 };
  // clang-format on

  static_assert(sizeof(xSupport) / sizeof(xSupport[0]) == MAX_NUM_CC_ALF_CHROMA_COEFF, "Invalid size of xSupport.");
  static_assert(sizeof(ySupport) / sizeof(ySupport[0]) == MAX_NUM_CC_ALF_CHROMA_COEFF, "Invalid size of ySupport.");

  const int numPosL = -*std::min_element(xSupport, xSupport + sizeof(xSupport) / sizeof(xSupport[0]));
  const int numPosR = *std::max_element(xSupport, xSupport + sizeof(xSupport) / sizeof(xSupport[0]));
  const int numPosT = -*std::min_element(ySupport, ySupport + sizeof(ySupport) / sizeof(ySupport[0]));
  const int numPosB = *std::max_element(ySupport, ySupport + sizeof(ySupport) / sizeof(ySupport[0]));
  CHECKD(numPosL < 0 || numPosR < 0 || numPosT < 0 || numPosB < 0, "Invalid number of neighbor positions.");

  for (int y = 0; y < height; y += (1 << log2BlockHeight))
  {
    for (int x = 0; x < width; x += (1 << log2BlockWidth))
    {
      lumaSwingGreaterThanThresholdCount[(y >> log2BlockHeight) * lumaCountStride + (x >> log2BlockWidth)] = 0;

      for (int yOff = 0; yOff < (1 << log2BlockHeight); yOff++)
      {
        for (int xOff = 0; xOff < (1 << log2BlockWidth); xOff++)
        {
          // only consider samples that are fully supported by picture
          if ((y + yOff) >= (height - numPosB) || (x + xOff) >= (width - numPosR) || (y + yOff) < numPosT ||
              (x + xOff) < numPosL)
          {
            continue;
          }

          int minVal = ((1 << lumaBitDepth) - 1);
          int maxVal = 0;
          for (int i = 0; i < MAX_NUM_CC_ALF_CHROMA_COEFF; ++i)
          {
            Pel p = luma[(yOff + ySupport[i]) * lumaStride + x + xOff + xSupport[i]];

            if (p < minVal)
            {
              minVal = p;
            }
            if (p > maxVal)
            {
              maxVal = p;
            }
          }

          if ((maxVal - minVal) > threshold)
          {
            lumaSwingGreaterThanThresholdCount[(y >> log2BlockHeight) * lumaCountStride + (x >> log2BlockWidth)]++;
          }
        }
      }
    }
    luma += (lumaStride << log2BlockHeight);
  }
}
