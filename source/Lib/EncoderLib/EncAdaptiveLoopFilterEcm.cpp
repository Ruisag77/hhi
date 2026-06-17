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

/** \file     EncAdaptiveLoopFilterEcm.cpp
 \brief    estimation part of the ECM adaptive loop filter class
 */
#include "EncAdaptiveLoopFilterEcm.h"
#include "EncCfg.h"

#include "CommonLib/Picture.h"
#include "CommonLib/CodingStructure.h"

#define AlfCtx(c) SubCtx(Ctx::Alf, c)

#include <algorithm>
#include <unordered_set>

#include "EncAdaptiveLoopFilterBaseDefs.h"

 /*
 * Presudo-random number generator based on XoShiRo.
 * Used in Simulated Annealing to randomize coefficient changes
 * and to provide stability for different launches and different platforms.
 */
class RandomGen
{
private:
  uint64_t state[4];

public:
  RandomGen()
  {
    {
      state[0] = 3780342784249958894ull;
      state[1] = 6804200696383934958ull;
      state[2] = 7225962928618213870ull;
      state[3] = 7542320113583526382ull;
    }
  }

  uint64_t cyclicShiftLeft(const uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }

  uint64_t nextRand()
  {
    uint64_t result    = cyclicShiftLeft(state[0] + state[3], 23) + state[0];
    uint64_t tempState = state[1] << 17;
    state[2] ^= state[0];
    state[3] ^= state[1];
    state[1] ^= state[2];
    state[0] ^= state[3];
    state[2] ^= tempState;
    state[3] = cyclicShiftLeft(state[3], 45);
    return result;
  }

  bool nextBool(double trueProbability)
  {
    uint64_t coin  = nextRand() >> 32;
    uint64_t bound = uint64_t(trueProbability * (1ull << 32));
    return coin < bound;
  }
};

EncAdaptiveLoopFilterEcm::EncAdaptiveLoopFilterEcm()
  : EncAdaptiveLoopFilterBase<AlfParametersEcm>()
  , AdaptiveLoopFilterEcm()
{
  m_filterScaleIdx = nullptr;
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

void EncAdaptiveLoopFilterEcm::create(const EncCfg *encCfg, const int picWidth, const int picHeight,
                                      const ChromaFormat chromaFormatIdc, const int maxCUWidth, const int maxCUHeight,
                                      const int maxCUDepth, const BitDepths &inputBitDepth)
{
  AdaptiveLoopFilterEcm::create(picWidth, picHeight, chromaFormatIdc, maxCUWidth, maxCUHeight, maxCUDepth,
                                inputBitDepth);
  EncAdaptiveLoopFilter::createSharedEncMembers(encCfg);

  if (m_encCfg->m_intraPeriod == 1)   // all intra
  {
    m_filterTypeTest[ChannelType::LUMA][ALF_FILTER_9_EXT_DB_RESI_DIRECT] = true;
    m_filterTypeTest[ChannelType::LUMA][ALF_FILTER_9_EXT_DB_RESI]        = false;
    m_enableLessClip                                                     = false;
  }
  else if (m_encCfg->m_intraPeriod > 1)   // random access
  {
    m_filterTypeTest[ChannelType::LUMA][ALF_FILTER_9_EXT_DB_RESI_DIRECT] = false;
    m_filterTypeTest[ChannelType::LUMA][ALF_FILTER_9_EXT_DB_RESI]        = true;
    m_enableLessClip                                                     = true;
  }
  else if (m_encCfg->m_intraPeriod == -1)   // low delay
  {
    m_filterTypeTest[ChannelType::LUMA][ALF_FILTER_9_EXT_DB_RESI_DIRECT] = false;
    m_filterTypeTest[ChannelType::LUMA][ALF_FILTER_9_EXT_DB_RESI]        = true;
    m_enableLessClip                                                     = true;
  }

  const int numBinsLuma   = m_encCfg->m_useNonLinearAlfLuma ? MAX_ALF_NUM_CLIP_VALS : 1;
  const int numBinsChroma = m_encCfg->m_useNonLinearAlfChroma ? MAX_ALF_NUM_CLIP_VALS : 1;

  for (const auto chType: { ChannelType::LUMA, ChannelType::CHROMA })
  {
    const int numClasses = isLuma(chType) ? MAX_NUM_ALF_CLASSES : 1;

    m_alfCovarianceFrame[chType].resize(m_filterShapes[chType].size());
    for (int i = 0; i != m_filterShapes[chType].size(); i++)
    {
      if (m_filterTypeTest[chType][m_filterShapes[chType][i].filterType] == false)
      {
        m_alfCovarianceFrame[chType][i].clear();
        continue;
      }
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
    const ChannelType chType         = toChannelType(CompID(compIdx));
    const int         numClassifiers = compIdx ? 1 : ALF_NUM_CLASSIFIER;

    m_alfCovariance[compIdx].resize(m_filterShapes[chType].size());
    for (int i = 0; i != m_filterShapes[chType].size(); i++)
    {
      if (m_filterTypeTest[chType][m_filterShapes[chType][i].filterType] == false)
      {
        m_alfCovariance[compIdx][i].clear();
        continue;
      }
      m_alfCovariance[compIdx][i].resize(m_numCTUsInPic);

      const unsigned numFixFiltSetCands = numFixedFilterSetCands(m_filterShapes[chType][i].filterType);
      for (int j = 0; j < m_numCTUsInPic; j++)
      {
        m_alfCovariance[compIdx][i][j].resize(numFixFiltSetCands);
        for (unsigned fixedFilterSetCandIdx = 0; fixedFilterSetCandIdx < numFixFiltSetCands; ++fixedFilterSetCandIdx)
        {
          m_alfCovariance[compIdx][i][j][fixedFilterSetCandIdx].resize(numClassifiers);
          for (int classifierIdx = 0; classifierIdx < numClassifiers; ++classifierIdx)
          {
            const int numClasses = compIdx ? 1 : ALF_NUM_CLASSES_CLASSIFIER[classifierIdx];

            m_alfCovariance[compIdx][i][j][fixedFilterSetCandIdx][classifierIdx].resize(numClasses);
            for (int k = 0; k < numClasses; ++k)
            {
              m_alfCovariance[compIdx][i][j][fixedFilterSetCandIdx][classifierIdx][k].create(
                m_filterShapes[chType][i].numCoeff, isLuma(chType) ? numBinsLuma : numBinsChroma);
            }
          }
        }
      }
    }
  }

  for (int i = 0; i != m_filterShapes[ChannelType::LUMA].size(); i++)
  {
    if (m_filterTypeTest[ChannelType::LUMA][m_filterShapes[ChannelType::LUMA][i].filterType] == false)
    {
      continue;
    }
    for (int j = 0; j <= MAX_NUM_ALF_CLASSES + 1; j++)
    {
      m_alfCovarianceMerged[i][j].create(m_filterShapes[ChannelType::LUMA][i].numCoeff, numBinsLuma);
    }
  }

  m_filterScaleIdx =
    new int8_t[std::max(ALF_MAX_NUM_ALTERNATIVES_LUMA * MAX_NUM_ALF_CLASSES, ALF_MAX_NUM_ALTERNATIVES_CHROMA)];
  m_filterCoeffSet =
    new int *[std::max(ALF_MAX_NUM_ALTERNATIVES_LUMA * MAX_NUM_ALF_CLASSES, ALF_MAX_NUM_ALTERNATIVES_CHROMA)];
  m_filterClippSet =
    new int *[std::max(ALF_MAX_NUM_ALTERNATIVES_LUMA * MAX_NUM_ALF_CLASSES, ALF_MAX_NUM_ALTERNATIVES_CHROMA)];
  for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
  {
    for (int j = 0; j < ALF_MAX_NUM_ALTERNATIVES_LUMA; j++)
    {
      m_filterCoeffSet[i * ALF_MAX_NUM_ALTERNATIVES_LUMA + j] = new int[MAX_NUM_ALF_LUMA_COEFF];
      m_filterClippSet[i * ALF_MAX_NUM_ALTERNATIVES_LUMA + j] = new int[MAX_NUM_ALF_LUMA_COEFF];
    }
  }

  for (int i = 0; i < NUM_FIXED_FILTERS; ++i)
  {
    for (int j = 0; j < NUM_FIXED_FILTER_SET_CANDS; ++j)
    {
      m_ctbDistortionFixedFilter[i][j] = new float[m_numCTUsInPic];
    }
  }

  for (int i = 0; i < ALF_CTB_MAX_NUM_APS; ++i)
  {
    for (int j = 0; j < ALF_MAX_NUM_ALTERNATIVES_LUMA; ++j)
    {
      for (int k = 0; k < NUM_FIXED_FILTER_SET_CANDS; ++k)
      {
        m_distCtbApsLuma[i][j][k] = new float[m_numCTUsInPic];
      }
    }
  }

  for (int i = 0; i < ALF_MAX_NUM_ALTERNATIVES_LUMA; ++i)
  {
    for (int j = 0; j < NUM_FIXED_FILTER_SET_CANDS; ++j)
    {
      m_distCtbLumaNewFilt[i][j] = new float[m_numCTUsInPic];
    }
  }

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

void EncAdaptiveLoopFilterEcm::destroy()
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

  if (m_filterScaleIdx)
  {
    delete[] m_filterScaleIdx;
    m_filterScaleIdx = nullptr;
  }

  if (m_filterCoeffSet)
  {
    for (int i = 0; i < MAX_NUM_ALF_CLASSES * ALF_MAX_NUM_ALTERNATIVES_LUMA; ++i)
    {
      delete[] m_filterCoeffSet[i];
      m_filterCoeffSet[i] = nullptr;
    }
    delete[] m_filterCoeffSet;
    m_filterCoeffSet = nullptr;
  }

  if (m_filterClippSet)
  {
    for (int i = 0; i < MAX_NUM_ALF_CLASSES * ALF_MAX_NUM_ALTERNATIVES_LUMA; ++i)
    {
      delete[] m_filterClippSet[i];
      m_filterClippSet[i] = nullptr;
    }
    delete[] m_filterClippSet;
    m_filterClippSet = nullptr;
  }

  for (int i = 0; i < NUM_FIXED_FILTERS; ++i)
  {
    for (int j = 0; j < NUM_FIXED_FILTER_SET_CANDS; ++j)
    {
      if (m_ctbDistortionFixedFilter[i][j])
      {
        delete[] m_ctbDistortionFixedFilter[i][j];
        m_ctbDistortionFixedFilter[i][j] = nullptr;
      }
    }
  }

  for (int i = 0; i < ALF_CTB_MAX_NUM_APS; ++i)
  {
    for (int j = 0; j < ALF_MAX_NUM_ALTERNATIVES_LUMA; ++j)
    {
      for (int k = 0; k < NUM_FIXED_FILTER_SET_CANDS; ++k)
      {
        if (m_distCtbApsLuma[i][j][k])
        {
          delete[] m_distCtbApsLuma[i][j][k];
          m_distCtbApsLuma[i][j][k] = nullptr;
        }
      }
    }
  }

  for (int i = 0; i < ALF_MAX_NUM_ALTERNATIVES_LUMA; ++i)
  {
    for (int k = 0; k < NUM_FIXED_FILTER_SET_CANDS; ++k)
    {
      if (m_distCtbLumaNewFilt[i][k])
      {
        delete[] m_distCtbLumaNewFilt[i][k];
        m_distCtbLumaNewFilt[i][k] = nullptr;
      }
    }
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
  AdaptiveLoopFilterEcm::destroy();
}

void EncAdaptiveLoopFilterEcm::xSetupCcAlfAPS(const CodingStructure &cs)
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

    CcAlfFilterParam &ccAlfAPSParam = aps->m_ccAlfAPSParam.getEcmParam();

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

    CcAlfFilterParam &ccAlfAPSParam = aps->m_ccAlfAPSParam.getEcmParam();

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

void EncAdaptiveLoopFilterEcm::ALFProcess(CodingStructure &cs, const double *lambdas,
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
        alfAPS->m_alfAPSParam.getEcmParam().reset();
        alfAPS->m_ccAlfAPSParam.getEcmParam().reset();
        alfAPS = nullptr;
      }
    }
  }

  std::fill_n(cs.slice->m_newAlfFixFiltSetCandIdx, MAX_NUM_COMP, -1);

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
  if (cs.slice->m_lfCccmEnabledFlag)
  {
    orgYuv = *m_newOrgBuf;
  }

  std::fill(m_ctuPadFlag.begin(), m_ctuPadFlag.end(), 0);

  m_tempBuf.copyFrom(cs.getRecoBuf());
  PelUnitBuf recYuv = m_tempBuf.getBuf(cs.area);
  recYuv.addMirrorExtension(MAX_FILTER_LENGTH_FIXED >> 1, false);

  // Setup the DBF input buffer.
  PelUnitBuf recYuvBeforeDb = m_tempBufBeforeDb.getBuf(cs.area);
  recYuvBeforeDb.addMirrorExtension(NUM_DB_PAD);

  // Setup the residual buffer.
  PelBuf resiLuma = m_tempBufResi.subBuf(cs.area.Y(), cs.area.Y());
  resiLuma.addMirrorExtension(MAX_FILTER_LENGTH_FIXED >> 1);

  // Setup the frame buffer for the SAO output. Needed only for CCALF
  // TODO: don't copy/mirror luma since it's not used
  m_tempBufSao.copyFrom(cs.getRecoBuf());
  PelUnitBuf tmpYuvSao = m_tempBufSao.getBuf(cs.area);
  tmpYuvSao.addMirrorExtension(MAX_FILTER_LENGTH_FIXED >> 1);

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
        buf.addMirrorExtension(MAX_ALF_PADDING_SIZE, false);
        buf =
          buf.subBuf(UnitArea(cs.area.chromaFormat,
                              Area(clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, width, height)));

        PelBuf bufResi = m_tempBufResiCtu.subBuf(0, 0, wBuf, hBuf);
        bufResi.copyFrom(resiLuma.subBuf(xPos - (clipL ? 0 : MAX_ALF_PADDING_SIZE),
                                         yPos - (clipT ? 0 : MAX_ALF_PADDING_SIZE), wBuf, hBuf));
        // pad top-left unavailable samples for raster slice
        if (rasterSliceAlfPad & 1)
        {
          bufResi.padBorderPel(MAX_ALF_PADDING_SIZE, MAX_ALF_PADDING_SIZE, 1);
        }

        // pad bottom-right unavailable samples for raster slice
        if (rasterSliceAlfPad & 2)
        {
          bufResi.padBorderPel(MAX_ALF_PADDING_SIZE, MAX_ALF_PADDING_SIZE, 2);
        }

        bufResi.addMirrorExtension(MAX_ALF_PADDING_SIZE);
        bufResi = bufResi.subBuf(clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, width, height);

        PelUnitBuf bufDb = m_tempBufBeforeDbCtu.subBuf(UnitArea(cs.area.chromaFormat, Area(0, 0, wBuf, hBuf)));
        bufDb.copyFrom(m_tempBufBeforeDb.subBuf(UnitArea(
          cs.area.chromaFormat, Area(xPos - (clipL ? 0 : NUM_DB_PAD), yPos - (clipT ? 0 : NUM_DB_PAD), wBuf, hBuf))));
        // pad top-left unavailable samples for raster slice
        if (rasterSliceAlfPad & 1)
        {
          bufDb.padBorderPel(NUM_DB_PAD, 1);
        }
        // pad bottom-right unavailable samples for raster slice
        if (rasterSliceAlfPad & 2)
        {
          bufDb.padBorderPel(NUM_DB_PAD, 2);
        }
        bufDb.addMirrorExtension(NUM_DB_PAD);
        bufDb = bufDb.subBuf(
          UnitArea(cs.area.chromaFormat, Area(clipL ? 0 : NUM_DB_PAD, clipT ? 0 : NUM_DB_PAD, width, height)));

        if (isChromaEnabled(m_chromaFormat))
        {
          for (int compIdx = 1; compIdx < MAX_NUM_COMP; compIdx++)
          {
            CompID     compID       = CompID(compIdx);
            const int  chromaScaleX = getComponentScaleX(compID, cs.area.chromaFormat);
            const int  chromaScaleY = getComponentScaleY(compID, cs.area.chromaFormat);
            const Area blkSrcChroma(0, 0, width >> chromaScaleX, height >> chromaScaleY);
            const Area blkDstChroma(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX,
                                    height >> chromaScaleY);
            deriveFixedFilterResultsChroma(buf.get(compID), bufDb.get(compID), blkSrcChroma, blkDstChroma, cs, -1,
                                           compID);
          }
        }

        const Area blkSrc(0, 0, width, height);
        const Area blkDst(xPos, yPos, width, height);
        deriveClassification(buf.get(COMP_Y), m_filterTypeTest[ChannelType::LUMA][ALF_FILTER_9_EXT_DB_RESI] == true,
                             bufResi, bufDb.get(COMP_Y), 0, blkDst, blkSrc, cs, ALF_NUM_CLASSIFIER);

        deriveGaussResults(bufDb.get(COMP_Y), blkDst, blkSrc);
      }
      else
      {
        if (isChromaEnabled(m_chromaFormat))
        {
          for (int compIdx = 1; compIdx < MAX_NUM_COMP; compIdx++)
          {
            CompID     compID       = CompID(compIdx);
            const int  chromaScaleX = getComponentScaleX(compID, cs.area.chromaFormat);
            const int  chromaScaleY = getComponentScaleY(compID, cs.area.chromaFormat);
            const Area blkChroma(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX,
                                 height >> chromaScaleY);
            deriveFixedFilterResultsChroma(recYuv.get(compID), recYuvBeforeDb.get(compID), blkChroma, blkChroma, cs, -1,
                                           compID);
          }
        }

        Area blk(xPos, yPos, width, height);
        deriveClassification(recLuma, m_filterTypeTest[ChannelType::LUMA][ALF_FILTER_9_EXT_DB_RESI] == true, resiLuma,
                             m_tempBufBeforeDb.getBuf(COMP_Y), 0, blk, blk, cs, ALF_NUM_CLASSIFIER);
        deriveGaussResults(recYuvBeforeDb.get(COMP_Y), blk, blk);
      }
    }
  }

  static_assert(FixFiltSetCand::SECOND == FixFiltSetCand::LAST,
                "Padding is missing for some fixed filter set candidates.");
  for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
  {
    extendFixedFilterResultsPic(CompID(compIdx), FixFiltSetCand::FIRST, FixFiltIdx::FIRST);
    extendFixedFilterResultsPic(CompID(compIdx), FixFiltSetCand::SECOND, FixFiltIdx::FIRST);
  }
  extendGaussResultsPic();

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

        PelBuf bufDb = m_tempBufBeforeDbCtu.getBuf(COMP_Y).subBuf(0, 0, wBuf, hBuf);
        bufDb.copyFrom(m_tempBufBeforeDb.getBuf(COMP_Y).subBuf(xPos - (clipL ? 0 : NUM_DB_PAD),
                                                               yPos - (clipT ? 0 : NUM_DB_PAD), wBuf, hBuf));
        // pad top-left unavailable samples for raster slice
        if (rasterSliceAlfPad & 1)
        {
          bufDb.padBorderPel(NUM_DB_PAD, NUM_DB_PAD, 1);
        }
        // pad bottom-right unavailable samples for raster slice
        if (rasterSliceAlfPad & 2)
        {
          bufDb.padBorderPel(NUM_DB_PAD, NUM_DB_PAD, 2);
        }
        bufDb.addMirrorExtension(NUM_DB_PAD);
        bufDb = bufDb.subBuf(clipL ? 0 : NUM_DB_PAD, clipT ? 0 : NUM_DB_PAD, width, height);

        const Area blkSrc(0, 0, width, height);
        const Area blkDst(xPos, yPos, width, height);
        copyAndExtendFixedFilterResultsCtu(FixFiltSetCand::FIRST, FixFiltIdx::FIRST, blkDst);
        copyAndExtendFixedFilterResultsCtu(FixFiltSetCand::SECOND, FixFiltIdx::FIRST, blkDst);
        deriveSecondFixedFilterResults(recLuma, bufDb, m_fixedFilterResultPerCtu, blkSrc, blkDst, cs, -1);
      }
      else
      {
        Area blk(xPos, yPos, width, height);
        deriveSecondFixedFilterResults(recLuma, m_tempBufBeforeDb.getBuf(COMP_Y), m_fixFilterResult[COMP_Y], blk, blk,
                                       cs, -1);
      }
    }
  }

  static_assert(FixFiltSetCand::SECOND == FixFiltSetCand::LAST,
                "Missing padding for some fixed filter set candidates.");
  extendFixedFilterResultsPic(COMP_Y, FixFiltSetCand::FIRST, FixFiltIdx::SECOND);
  extendFixedFilterResultsPic(COMP_Y, FixFiltSetCand::SECOND, FixFiltIdx::SECOND);

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
                [](int &m) { LumaCtbModeHandler::setApsFilter(m, 0, 0); });
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
  recYuv.addMirrorExtension(MAX_ALF_FILTER_LENGTH >> 1, false);

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

float EncAdaptiveLoopFilterEcm::deriveCtbAlfEnableFlags(CodingStructure &cs, const int shapeIdx, ChannelType channel,
#if ENABLE_QPA
                                                        const double chromaWeight,
#endif
                                                        const int numClasses, const int numCoeff, float &distUnfilter,
                                                        int fixedFilterSetIdx, bool *ctuModeChanged)
{
  if (ctuModeChanged)
  {
    *ctuModeChanged = false;
  }
  TempCtx      ctxTempStart(m_ctxPool);
  TempCtx      ctxTempBest(m_ctxPool);
  TempCtx      ctxTempAltStart(m_ctxPool);
  TempCtx      ctxTempAltBest(m_ctxPool);
  const CompID compIDFirst = isLuma(channel) ? COMP_Y : COMP_Cb;
  const CompID compIDLast  = isLuma(channel) ? COMP_Y : COMP_Cr;
  const int    numAlts = isLuma(channel) ? m_alfParamTemp.numAlternativesLuma : m_alfParamTemp.numAlternativesChroma;

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

  const int maxNumAlts = isLuma(channel) ? ALF_MAX_NUM_ALTERNATIVES_LUMA : ALF_MAX_NUM_ALTERNATIVES_CHROMA;
  for (int altIdx = 0; altIdx < maxNumAlts; ++altIdx)
  {
    for (int classIdx = 0; classIdx < (isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1); classIdx++)
    {
      m_filterScaleIdx[altIdx * numClasses + classIdx] =
        isLuma(channel) ? m_scaleIdxFinal[altIdx][classIdx] : m_chromaScaleIdxFinal[altIdx][0];
      const coeff_t *const coeffSrc =
        isLuma(channel) ? &m_coeffFinal[altIdx][classIdx * MAX_NUM_ALF_LUMA_COEFF] : m_chromaCoeffFinal[altIdx];
      const clip_t *const clippSrc =
        isLuma(channel) ? &m_clippFinal[altIdx][classIdx * MAX_NUM_ALF_LUMA_COEFF] : m_chromaClippFinal[altIdx];
      int *const coeffDst = m_filterCoeffSet[altIdx * numClasses + classIdx];
      int *const clippDst = m_filterClippSet[altIdx * numClasses + classIdx];
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
      int bestModeBefore         = (*m_modes)[compID][ctuIdx];
      (*m_modes)[compID][ctuIdx] = isLuma(channel) ? CTB_MODE_LUMA_APS0_ALT0 : CTB_MODE_CHROMA_ALT0;
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
          LumaCtbModeHandler::setAlternative((*m_modes)[compID][ctuIdx], altIdx);
          m_CABACEstimator->codeAlfCtuLumaAlternative(cs, ctuIdx, numAlts);
          float rAltCost      = ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
          int   classifierIdx = m_classifierFinal[altIdx];
          float altDist =
            getFilteredDistortion(m_alfCovariance[compID][shapeIdx][ctuIdx][fixedFilterSetIdx][classifierIdx],
                                  ALF_NUM_CLASSES_CLASSIFIER[classifierIdx], m_alfParamTemp.numLumaFilters[altIdx] - 1,
                                  numCoeff, altIdx, true);
          float altCost = altDist + rAltCost;

          if (altCost < bestAltCost)
          {
            bestAltCost = altCost;
            bestAltIdx  = altIdx;
            ctxTempBest = AlfCtx(m_CABACEstimator->getCtx());
          }
        }
        LumaCtbModeHandler::setAlternative((*m_modes)[compID][ctuIdx], bestAltIdx);
        costOn += bestAltCost;
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

          float altDist = 0.f;
          altDist += m_alfCovariance[compID][shapeIdx][ctuIdx][fixedFilterSetIdx][0][0].calcErrorForCoeffs(
            m_filterClippSet[altIdx], m_filterCoeffSet[altIdx], numCoeff, ALF_SCALE_FACTOR[m_filterScaleIdx[altIdx]],
            (isLuma(channel) ? COEFF_SCALE_BITS_LUMA : COEFF_SCALE_BITS_CHROMA) + ALF_SCALE_SHIFT);

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
      if ((*m_modes)[compID][ctuIdx] != bestModeBefore)
      {
        if (ctuModeChanged)
        {
          *ctuModeChanged = true;
        }
      }
    }
  }

  if (isChroma(channel))
  {
    setSliceEnabledFlag(m_alfParamTemp, channel, m_modes);
  }

  return cost;
}

void EncAdaptiveLoopFilterEcm::firstPass(CodingStructure &cs, AlfParam &alfParam, const PelUnitBuf &orgUnitBuf,
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

  int bestShapeIdx = -1, bestFixedFilterSetCandIdx = -1;
  for (int withSA = 0; withSA < 2; withSA++)
  {
    if (withSA && (bestShapeIdx < 0 || bestFixedFilterSetCandIdx < 0))
    {
      // ALF is disabled
      break;
    }
    int shapeIdxFrom = 0, shapeIdxTo = (int)alfFilterShape.size();
    if (withSA)
    {
      shapeIdxFrom = bestShapeIdx, shapeIdxTo = bestShapeIdx + 1;
    }
    for (int shapeIdx = shapeIdxFrom; shapeIdx < shapeIdxTo; shapeIdx++)
    {
      if (m_filterTypeTest[channel][alfFilterShape[shapeIdx].filterType] == false)
      {
        continue;
      }

      const unsigned numFixFiltSetCands = numFixedFilterSetCands(alfFilterShape[shapeIdx].filterType);

      m_alfParamTemp = alfParam;
      // 1. get unfiltered distortion
      if (isChroma(channel))
      {
        m_alfParamTemp.numAlternativesChroma = 1;
      }
      else
      {
        m_alfParamTemp.numAlternativesLuma = 1;
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

      int fixedFilterSetCandIdxFrom = 0, fixedFilterSetCandIdxTo = numFixFiltSetCands;
      if (withSA)
      {
        fixedFilterSetCandIdxFrom = bestFixedFilterSetCandIdx, fixedFilterSetCandIdxTo = bestFixedFilterSetCandIdx + 1;
      }
      for (unsigned fixedFilterSetCandIdx = fixedFilterSetCandIdxFrom; fixedFilterSetCandIdx < fixedFilterSetCandIdxTo;
           ++fixedFilterSetCandIdx)
      {
        int numAlternativesFrom = 1;
        int numAlternativesTo = (isLuma(channel) ? std::max(1, std::min(m_numCTUsInPic, ALF_MAX_NUM_ALTERNATIVES_LUMA))
                                                 : getMaxNumAlternativesChroma());
        if (withSA)
        {
          numAlternativesFrom = (isLuma(channel) ? alfParam.numAlternativesLuma : alfParam.numAlternativesChroma),
          numAlternativesTo   = numAlternativesFrom;
        }
        for (int numAlternatives = numAlternativesFrom; numAlternatives <= numAlternativesTo; ++numAlternatives)
        {
          if (isChroma(channel))
          {
            m_alfParamTemp.numAlternativesChroma = numAlternatives;
          }
          else
          {
            m_alfParamTemp.numAlternativesLuma = numAlternatives;
          }
          m_alfParamTemp.filterType[channel] = alfFilterShape[shapeIdx].filterType;

          if (withSA)
          {
            // 2. restore the best CTUs on/off flags, alternatives
            setSliceEnabledFlag(m_alfParamTemp, channel, m_indexTmp);
            copyIndices(m_modes, m_indexTmp, channel);
          }
          else
          {
            // 2. all CTUs are on
            setSliceEnabledFlag(m_alfParamTemp, channel, true);

            // all alternatives are on
            if (isChroma(channel))
            {
              initCtuAlternativeChroma(m_modes);
            }
            else
            {
              initCtuAlternativeLuma(m_modes);
            }
          }
          bool update = false;

          // 3. CTU decision
          float     distUnfilter       = 0;
          const int iterNum            = isLuma(channel) ? 5 : (2 + m_alfParamTemp.numAlternativesChroma);
          const int maxIterNumNoChange = withSA ? 2 : 1;
          bool      ctuModeChanged     = true;
          int       bestIter           = -1;
          float     bestIterCost       = MAX_FLOAT;
          for (int iter = 0; iter < iterNum && ctuModeChanged; iter++)
          {
            // TODO: check
            // unfiltered distortion is added due to some CTBs may not use filter
            // no need to reset CABAC here, since uiCoeffBits is not affected
            m_CABACEstimator->getCtx() = AlfCtx(ctxStart);
            getFilterCoeffAndCost(cs, distUnfilter, channel, true, shapeIdx, coeffBits, fixedFilterSetCandIdx, withSA);

            m_CABACEstimator->getCtx() = AlfCtx(ctxStart);
            cost                       = m_lambda[isLuma(channel) ? COMP_Y : COMP_Cb] * coeffBits;

            cost += deriveCtbAlfEnableFlags(cs, shapeIdx, channel,
#if ENABLE_QPA
                                            lambdaChromaWeight,
#endif
                                            numClasses, alfFilterShape[shapeIdx].numCoeff, distUnfilter,
                                            fixedFilterSetCandIdx, &ctuModeChanged);
            if (cost < costMin)
            {
              update                   = true;
              m_bitsNewFilter[channel] = coeffBits;
              costMin                  = cost;
              ctxBest                  = AlfCtx(m_CABACEstimator->getCtx());
              copyIndices(m_indexTmp, m_modes, channel);
              copyAlfParam(alfParam, m_alfParamTemp, channel);
              bestShapeIdx              = shapeIdx;
              bestFixedFilterSetCandIdx = fixedFilterSetCandIdx;
            }
            if (cost < bestIterCost)
            {
              bestIter     = iter;
              bestIterCost = cost;
            }
            else if (iter >= bestIter + maxIterNumNoChange)
            {
              // High probability that we have converged or we are diverging
              break;
            }
          }   // for iter

          if (update == false && numAlternatives > 1)
          {
            break;
          }
          // Decrease number of alternatives and reset ctu params and filters
        }
      }   // for fixedFilterSetIdx
    }   // for shapeIdx
  }   // for withSA
  m_CABACEstimator->getCtx() = AlfCtx(ctxBest);

  copyIndices(m_modes, m_indexTmp, channel);
}

void EncAdaptiveLoopFilterEcm::copyAlfParam(AlfParam &alfParamDst, AlfParam &alfParamSrc, ChannelType channel)
{
  if (isLuma(channel))
  {
    alfParamDst = alfParamSrc;
  }
  else
  {
    alfParamDst.enabledFlag[COMP_Cb]            = alfParamSrc.enabledFlag[COMP_Cb];
    alfParamDst.enabledFlag[COMP_Cr]            = alfParamSrc.enabledFlag[COMP_Cr];
    alfParamDst.numAlternativesChroma           = alfParamSrc.numAlternativesChroma;
    alfParamDst.filterType[ChannelType::CHROMA] = alfParamSrc.filterType[ChannelType::CHROMA];
    alfParamDst.chromaNonLinearFlag             = alfParamSrc.chromaNonLinearFlag;
    alfParamDst.chromaScaleIdx                  = alfParamSrc.chromaScaleIdx;
    alfParamDst.chromaCoeff                     = alfParamSrc.chromaCoeff;
    alfParamDst.chromaClipp                     = alfParamSrc.chromaClipp;
  }
}

float EncAdaptiveLoopFilterEcm::getFilterCoeffAndCost(CodingStructure &cs, float distUnfilter, ChannelType channel,
                                                      bool bReCollectStat, int shapeIdx, int &coeffBits,
                                                      int fixedFilterSetIdx, bool tryImproveBySA, bool onlyFilterCost)
{
  float dist                     = distUnfilter;
  coeffBits                      = 0;
  AlfFilterShape &alfFilterShape = (*m_alfParamTemp.filterShapes)[channel][shapeIdx];
  // get filter coeff
  if (isLuma(channel))
  {
    for (int altIdx = 0; altIdx < m_alfParamTemp.numAlternativesLuma; ++altIdx)
    {
      AlfParam bestSliceParam;
      float    bestCost      = MAX_FLOAT;
      float    bestDist      = MAX_FLOAT;
      int      bestCoeffBits = 0;
      for (int classifierIdx = 0; classifierIdx < ALF_NUM_CLASSIFIER; classifierIdx++)
      {
        if (classifierIdx == ALF_NUM_CLASSIFIER - 1 && cs.slice->isIntra())
        {
          continue;
        }
        // collect stat based on CTU decision
        if (bReCollectStat)
        {
          getFrameStats(channel, shapeIdx, altIdx, fixedFilterSetIdx, classifierIdx);
        }
        assert(alfFilterShape.numCoeff == m_alfCovarianceFrame[channel][shapeIdx][0].numCoeff);

        const int nonLinearFlagMax = m_encCfg->m_useNonLinearAlfLuma ? 2 : 1;
        for (int nonLinearFlag = 0; nonLinearFlag < nonLinearFlagMax; ++nonLinearFlag)
        {
          m_alfParamTemp.lumaClassifierIdx[altIdx] = classifierIdx;
          m_alfParamTemp.lumaNonLinearFlag[altIdx] = nonLinearFlag;
          std::fill_n(m_alfClipMerged[shapeIdx][0][0],
                      MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_CLASSES,
                      m_alfParamTemp.lumaNonLinearFlag[altIdx] ? ALF_NUM_CLIP_VALS[ChannelType::LUMA] / 2 : 0);

          // Reset Merge Tmp Cov
          m_alfCovarianceMerged[shapeIdx][MAX_NUM_ALF_CLASSES].reset();
          m_alfCovarianceMerged[shapeIdx][MAX_NUM_ALF_CLASSES + 1].reset();
          // distortion
          int   curCoeffBits;
          float curDist =
            mergeFiltersAndCost(m_alfParamTemp, alfFilterShape, m_alfCovarianceFrame[channel][shapeIdx],
                                m_alfCovarianceMerged[shapeIdx], m_alfClipMerged[shapeIdx], curCoeffBits, altIdx,
                                classifierIdx, m_alfParamTemp.numLumaFilters[altIdx], tryImproveBySA);
          float cost = curDist + m_lambda[COMP_Y] * curCoeffBits;
          if (cost < bestCost)
          {
            bestCost       = cost;
            bestDist       = curDist;
            bestCoeffBits  = curCoeffBits;
            bestSliceParam = m_alfParamTemp;
          }
        }   // for nonLinearFlag
      }   // for classifierIdx
      coeffBits += bestCoeffBits;
      dist += bestDist;
      m_alfParamTemp = bestSliceParam;
    }   // for altIdx
    coeffBits += lengthUvlc(m_alfParamTemp.numAlternativesLuma - 1);
    coeffBits += m_alfParamTemp.numAlternativesLuma;   // non-linear flags
    coeffBits += 1;   // alf_luma_13_ext_db_resi_direct : alf_luma_13_ext_db_resi
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
        getFrameStats(channel, shapeIdx, altIdx, fixedFilterSetIdx, 0);
      }
      assert(alfFilterShape.numCoeff == m_alfCovarianceFrame[channel][shapeIdx][0].numCoeff);

      AlfParam  bestSliceParam;
      float     bestCost         = MAX_FLOAT;
      float     bestDist         = MAX_FLOAT;
      int       bestCoeffBits    = 0;
      const int nonLinearFlagMax = m_encCfg->m_useNonLinearAlfChroma ? 1 : 0;

      for (int nonLinearFlag = 0; nonLinearFlag <= nonLinearFlagMax; nonLinearFlag++)
      {
        m_alfParamTemp.chromaNonLinearFlag[altIdx] = nonLinearFlag;

        std::fill_n(m_filterClippSet[altIdx], MAX_NUM_ALF_CHROMA_COEFF,
                    nonLinearFlag ? ALF_NUM_CLIP_VALS[ChannelType::CHROMA] / 2 : 0);
        float dist = m_alfCovarianceFrame[channel][shapeIdx][0].pixAcc +
          deriveCoeffQuant(m_filterClippSet[altIdx], m_filterCoeffSet[altIdx], m_filterScaleIdx[altIdx],
                           m_alfCovarianceFrame[channel][shapeIdx][0], alfFilterShape, false, COEFF_SCALE_BITS_CHROMA,
                           COEFF_MANTISSA_BITS_CHROMA, nonLinearFlag, tryImproveBySA, tryImproveBySA);
        m_alfParamTemp.chromaScaleIdx[altIdx][0] = m_filterScaleIdx[altIdx];
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
    coeffBits += m_alfParamTemp.numAlternativesChroma;   // non-linear flags
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
      CHECKD(LumaCtbModeHandler::isEnabled(cs.slice->m_pic->getAlfModes(CompID::COMP_Y)[ctuIdx]) &&
               (!LumaCtbModeHandler::isApsFilter(cs.slice->m_pic->getAlfModes(CompID::COMP_Y)[ctuIdx]) ||
                (LumaCtbModeHandler::getApsIdx(cs.slice->m_pic->getAlfModes(CompID::COMP_Y)[ctuIdx]) != 0)),
             "Unexpected CTU ALF mode.");
      CHECKD(cs.slice->m_numAlfApsIdsLuma != 1, "Unexpected number of luma APSs.");
      m_CABACEstimator->codeAlfCtuFilterIndex(cs, ctuIdx, m_alfParamTemp.enabledFlag[COMP_Y]);
      m_CABACEstimator->codeAlfCtuLumaAlternative(cs, ctuIdx, m_alfParamTemp.numAlternativesLuma);
    }
  }
  if (isChroma(channel))
  {
    m_CABACEstimator->codeAlfCtuChromaAlternatives(cs, m_alfParamTemp);
  }
  rate += FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
  return dist + m_lambda[isLuma(channel) ? COMP_Y : COMP_Cb] * rate;
}

int EncAdaptiveLoopFilterEcm::getChromaCoeffRate(AlfParam &alfParam, int altIdx)
{
  int                  iBits = 0;
  const AlfFilterShape alfShape =
    m_filterShapes[ChannelType::CHROMA]
                  [m_filterTypeToStatIndex[ChannelType::CHROMA][alfParam.filterType[ChannelType::CHROMA]]];

  // Filter coefficients
  for (int orderIdx = 0; orderIdx < alfShape.numOrder; orderIdx++)
  {
    int            startIdx = orderIdx == 0 ? 0 : alfShape.indexSecOrder;
    int            endIdx   = orderIdx == 0 ? alfShape.indexSecOrder : alfShape.numCoeff - 1;
    AlfHuffmanCode huffman(false, COEFF_SCALE_BITS_CHROMA, COEFF_MANTISSA_BITS_CHROMA, orderIdx);
    huffman.init();
    for (int coeffIdx = startIdx; coeffIdx < endIdx; coeffIdx++)
    {
      iBits += lengthHuffman(alfParam.chromaCoeff[altIdx][coeffIdx], huffman);
    }
  }

  if (m_alfParamTemp.chromaNonLinearFlag[altIdx])
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
  iBits += ALF_SCALE_BITS_NUM;
  return iBits;
}

float EncAdaptiveLoopFilterEcm::getFilteredDistortion(const CovarianceVec1D &cov, const int numClasses,
                                                      const int numFiltersMinus1, const int numCoeff, const int altIdx,
                                                      const bool isLuma) const
{
  CHECKD(cov.size() != numClasses, "Covariance vector size does not match number of classes.");

  float dist = 0;
  for (int classIdx = 0; classIdx < numClasses; classIdx++)
  {
    const int i = altIdx * numClasses + classIdx;
    dist += cov[classIdx].calcErrorForCoeffs(
      m_filterClippSet[i], m_filterCoeffSet[i], numCoeff, ALF_SCALE_FACTOR[m_filterScaleIdx[i]],
      (isLuma ? COEFF_SCALE_BITS_LUMA : COEFF_SCALE_BITS_CHROMA) + ALF_SCALE_SHIFT);
  }
  return dist;
}

float EncAdaptiveLoopFilterEcm::mergeFiltersAndCost(
  AlfParam &alfParam, AlfFilterShape &alfShape, const CovarianceVec1D &covFrame, CovMergedArr1D &covMerged,
  int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], int &coeffBitsFinal, int altIdx,
  int classifierIdx, int numFiltersLinear, const bool tryImproveBySA)
{
  int   numFiltersBest = 0;
  int   numFilters     = ALF_NUM_CLASSES_CLASSIFIER[classifierIdx];
  bool  codedVarBins[MAX_NUM_ALF_CLASSES];
  float errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2];

  float cost, cost0, dist, distForce0, costMin = MAX_FLOAT;
  int   nonFilterBits, coeffBits, coeffBitsForce0;

  const int maxNumFilters =
    m_alfParamTemp.lumaNonLinearFlag[altIdx] ? std::min(numFiltersLinear + 3, numFilters) : numFilters;
  int8_t     bestScaleIdx[MAX_NUM_ALF_CLASSES];
  int        bestCoeff[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF];
  int        bestClipp[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF];
  float      bestDist                                                 = 0;
  int        bestBits                                                 = 0;
  bool       bestCodedVarBins[MAX_NUM_ALF_CLASSES]                    = { false };
  static int mergedPair[MAX_NUM_ALF_CLASSES][2]                       = { { 0 } };
  int8_t     mergedScaleIdx[MAX_NUM_ALF_CLASSES]                      = { 0 };
  int        mergedCoeff[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF] = { { 0 } };
  float      mergedErr[MAX_NUM_ALF_CLASSES]                           = { 0 };
  if (m_alfParamTemp.lumaNonLinearFlag[altIdx] == false)
  {
    memset(mergedPair, 0, sizeof(mergedPair));
    mergeClasses(alfShape, covFrame, covMerged, clipMerged, numFilters, m_filterIndices, altIdx, mergedPair);
  }
  else
  {
    for (int i = 0; i <= MAX_NUM_ALF_CLASSES; ++i)
    {
      covMerged[i].numBins = ALF_NUM_CLIP_VALS[ChannelType::LUMA];
    }
    for (int i = 0; i < numFilters; ++i)
    {
      for (int j = 0; j < numFilters; ++j)
      {
        std::fill_n(clipMerged[i][j], MAX_NUM_ALF_LUMA_COEFF, 2);
      }
    }
  }
  numFilters = maxNumFilters;

  while (numFilters >= 1)
  {
    alfParam.numLumaFilters[altIdx] = numFilters;
    dist = deriveFilterCoeffs(covFrame, covMerged, clipMerged, alfShape, m_filterIndices[numFilters - 1], numFilters,
                              errorForce0CoeffTab, m_alfParamTemp.lumaNonLinearFlag[altIdx], classifierIdx,
                              numFilters == maxNumFilters ? true : false, mergedPair, mergedScaleIdx, mergedCoeff,
                              mergedErr, tryImproveBySA);
    // filter coeffs are stored in m_filterCoeffSet
    distForce0    = getDistForce0(alfShape, numFilters, errorForce0CoeffTab, codedVarBins, altIdx, true,
                                  COEFF_SCALE_BITS_LUMA, COEFF_MANTISSA_BITS_LUMA);
    nonFilterBits = getNonFilterCoeffRate(alfParam, altIdx, classifierIdx);
    coeffBits     = deriveFilterCoefficientsPredictionMode(alfShape, m_filterCoeffSet, numFilters,
                                                           m_alfParamTemp.lumaNonLinearFlag[altIdx], true,
                                                           COEFF_SCALE_BITS_LUMA, COEFF_MANTISSA_BITS_LUMA) +
      nonFilterBits;
    coeffBitsForce0 = getCostFilterCoeffForce0(alfShape, m_filterCoeffSet, numFilters, codedVarBins, altIdx, true,
                                               COEFF_SCALE_BITS_LUMA, COEFF_MANTISSA_BITS_LUMA) +
      nonFilterBits;

    cost  = dist + m_lambda[COMP_Y] * coeffBits;
    cost0 = distForce0 + m_lambda[COMP_Y] * coeffBitsForce0;

    bool cost0better = false;

    if (cost0 < cost)
    {
      cost        = cost0;
      cost0better = true;
    }

    if (cost <= costMin)
    {
      costMin        = cost;
      numFiltersBest = numFilters;
      memcpy(bestCodedVarBins, codedVarBins, sizeof(codedVarBins));
      for (int varInd = 0; varInd < numFilters; varInd++)
      {
        if (cost0better && (!bestCodedVarBins[varInd]))
        {
          bestScaleIdx[varInd] = 0;
          memset(bestCoeff[varInd], 0, sizeof(int) * MAX_NUM_ALF_LUMA_COEFF);
          memset(bestClipp[varInd], 0, sizeof(int) * MAX_NUM_ALF_LUMA_COEFF);
        }
        else
        {
          bestScaleIdx[varInd] = m_filterScaleIdx[varInd];
          memcpy(bestCoeff[varInd], m_filterCoeffSet[varInd], sizeof(int) * MAX_NUM_ALF_LUMA_COEFF);
          memcpy(bestClipp[varInd], m_filterClippSet[varInd], sizeof(int) * MAX_NUM_ALF_LUMA_COEFF);
        }
      }
      if (cost0better)
      {
        bestDist = distForce0;
        bestBits = coeffBitsForce0;
      }
      else
      {
        bestDist = dist;
        bestBits = coeffBits;
      }
    }
    numFilters--;
  }

  if (tryImproveBySA)
  {
    for (int filtIdx = 0; filtIdx < numFiltersBest; filtIdx++)
    {
      m_filterScaleIdx[filtIdx] = bestScaleIdx[filtIdx];
      memcpy(m_filterCoeffSet[filtIdx], bestCoeff[filtIdx], sizeof(int) * MAX_NUM_ALF_LUMA_COEFF);
      memcpy(m_filterClippSet[filtIdx], bestClipp[filtIdx], sizeof(int) * MAX_NUM_ALF_LUMA_COEFF);
    }
    dist = tryImproveFilterCoeffs(covFrame, covMerged, alfShape, m_filterIndices[numFiltersBest - 1], numFiltersBest,
                                  m_alfParamTemp.lumaNonLinearFlag[altIdx], classifierIdx);
    // filter coeffs are stored in m_filterCoeffSet
    nonFilterBits = getNonFilterCoeffRate(alfParam, altIdx, classifierIdx);
    coeffBits     = deriveFilterCoefficientsPredictionMode(alfShape, m_filterCoeffSet, numFiltersBest,
                                                           m_alfParamTemp.lumaNonLinearFlag[altIdx], true,
                                                           COEFF_SCALE_BITS_LUMA, COEFF_MANTISSA_BITS_LUMA) +
      nonFilterBits;
    cost = dist + m_lambda[COMP_Y] * coeffBits;

    if (cost < costMin)
    {
      costMin = cost;
      for (int varInd = 0; varInd < numFiltersBest; varInd++)
      {
        bestScaleIdx[varInd] = m_filterScaleIdx[varInd];
        memcpy(bestCoeff[varInd], m_filterCoeffSet[varInd], sizeof(int) * MAX_NUM_ALF_LUMA_COEFF);
        memcpy(bestClipp[varInd], m_filterClippSet[varInd], sizeof(int) * MAX_NUM_ALF_LUMA_COEFF);
      }
      bestDist = dist;
      bestBits = coeffBits;
    }
  }

  const float distReturn          = bestDist;
  coeffBitsFinal                  = bestBits;
  alfParam.numLumaFilters[altIdx] = numFiltersBest;

  for (int ind = 0; ind < alfParam.numLumaFilters[altIdx]; ++ind)
  {
    alfParam.lumaScaleIdx[altIdx][ind] = bestScaleIdx[ind];
    for (int i = 0; i < alfShape.numCoeff; i++)
    {
      alfParam.lumaCoeff[altIdx][ind * MAX_NUM_ALF_LUMA_COEFF + i] = bestCoeff[ind][i];
      alfParam.lumaClipp[altIdx][ind * MAX_NUM_ALF_LUMA_COEFF + i] = bestClipp[ind][i];
    }
  }

  auto &filterIdcsDst = alfParam.filterCoeffDeltaIdx[altIdx];
  std::copy_n(std::begin(m_filterIndices[numFiltersBest - 1]), MAX_NUM_ALF_CLASSES, std::begin(filterIdcsDst));

  return distReturn;
}

int EncAdaptiveLoopFilterEcm::getNonFilterCoeffRate(AlfParam &alfParam, int altIdx, int classifierIdx)
{
  const int numLumaFilters = alfParam.numLumaFilters[altIdx];
  const int maxNumClasses  = ALF_NUM_CLASSES_CLASSIFIER[classifierIdx];

  CHECK(numLumaFilters < 1, "Wrong number of alfParam.numLumaFilters[altIdx]");

  int len = 0   // alf_coefficients_delta_flag
    + lengthUvlc(numLumaFilters - 1);   // alf_luma_num_filters_signalled_minus1   ue(v)

  if (numLumaFilters > 1)
  {
    const int coeffLength = ceilLog2(numLumaFilters);
    for (int i = 0; i < maxNumClasses; ++i)
    {
      len += coeffLength;   // alf_luma_coeff_delta_idx   u(v)
    }
  }

  if (classifierIdx == 1)
  {
    len++;
  }
  else
  {
    len += 2;
  }

  return len;
}

int EncAdaptiveLoopFilterEcm::getCostFilterCoeffForce0(AlfFilterShape &alfShape, int **pDiffQFilterCoeffIntPP,
                                                       const int numFilters, bool *codedVarBins, int altIdx,
                                                       const bool isLuma, const int fractionalBits,
                                                       const int mantissaBits)
{
  int len = numFilters * ALF_SCALE_BITS_NUM;
  // Filter coefficients
  for (int orderIdx = 0; orderIdx < alfShape.numOrder; ++orderIdx)
  {
    int            startIdx = orderIdx == 0 ? 0 : alfShape.indexSecOrder;
    int            endIdx   = orderIdx == 0 ? alfShape.indexSecOrder : alfShape.numCoeff - 1;
    AlfHuffmanCode huffman(isLuma, fractionalBits, mantissaBits, orderIdx);
    huffman.init();
    for (int filtIdx = 0; filtIdx < numFilters; ++filtIdx)
    {
      for (int coeffIdx = startIdx; coeffIdx < endIdx; ++coeffIdx)
      {
        if (codedVarBins[filtIdx])
        {
          len += lengthHuffman(pDiffQFilterCoeffIntPP[filtIdx][coeffIdx], huffman);
        }
        else
        {
          len += lengthHuffman(0, huffman);
        }
      }
    }
  }

  if (m_alfParamTemp.lumaNonLinearFlag[altIdx])
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

int EncAdaptiveLoopFilterEcm::deriveFilterCoefficientsPredictionMode(const AlfFilterShape   &alfShape,
                                                                     const int *const *const filterSet,
                                                                     const int numFilters, const bool nonLinearFlag,
                                                                     const bool isLuma, const int fractionalBits,
                                                                     const int mantissaBits)
{
  const int costCoeff = lengthFilterCoeffs(alfShape, numFilters, filterSet, isLuma, fractionalBits, mantissaBits);
  if (nonLinearFlag)
  {
    return costCoeff + getCostFilterClipp(alfShape.numCoeff, filterSet, numFilters);
  }
  else
  {
    return costCoeff;
  }
}

int EncAdaptiveLoopFilterEcm::lengthFilterCoeffsOneFilter(const AlfFilterShape &alfShape, const int *filterCoeff,
                                                          const bool isLuma, const int fractionalBits,
                                                          const int mantissaBits) const
{
  int bitCnt = ALF_SCALE_BITS_NUM;
  for (int orderIdx = 0; orderIdx < alfShape.numOrder; ++orderIdx)
  {
    int            startIdx = orderIdx == 0 ? 0 : alfShape.indexSecOrder;
    int            endIdx   = orderIdx == 0 ? alfShape.indexSecOrder : alfShape.numCoeff - 1;
    AlfHuffmanCode huffman(isLuma, fractionalBits, mantissaBits, orderIdx);
    huffman.init();
    for (int coeffIdx = startIdx; coeffIdx < endIdx; ++coeffIdx)
    {
      bitCnt += lengthHuffman(filterCoeff[coeffIdx], huffman);
    }
  }
  return bitCnt;
}

int EncAdaptiveLoopFilterEcm::lengthFilterCoeffs(const AlfFilterShape &alfShape, const int numFilters,
                                                 const int *const *const filterCoeff, const bool isLuma,
                                                 const int fractionalBits, const int mantissaBits) const
{
  int bitCnt = numFilters * ALF_SCALE_BITS_NUM;
  for (int orderIdx = 0; orderIdx < alfShape.numOrder; ++orderIdx)
  {
    int            startIdx = orderIdx == 0 ? 0 : alfShape.indexSecOrder;
    int            endIdx   = orderIdx == 0 ? alfShape.indexSecOrder : alfShape.numCoeff - 1;
    AlfHuffmanCode huffman(isLuma, fractionalBits, mantissaBits, orderIdx);
    huffman.init();
    for (int filtIdx = 0; filtIdx < numFilters; ++filtIdx)
    {
      for (int coeffIdx = startIdx; coeffIdx < endIdx; ++coeffIdx)
      {
        bitCnt += lengthHuffman(filterCoeff[filtIdx][coeffIdx], huffman);
      }
    }
  }
  return bitCnt;
}

float EncAdaptiveLoopFilterEcm::getDistForce0(AlfFilterShape &alfShape, const int numFilters,
                                              float errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], bool *codedVarBins,
                                              int altIdx, const bool isLuma, const int fractionalBits,
                                              const int mantissaBits)
{
  int bitsVarBin[MAX_NUM_ALF_CLASSES];

  AlfHuffmanCode huffman(isLuma, fractionalBits, mantissaBits, 0);
  huffman.init();
  for (int ind = 0; ind < numFilters; ++ind)
  {
    bitsVarBin[ind] = 0;
    for (int i = 0; i < alfShape.numCoeff - 1; i++)
    {
      huffman.setGroup(i < alfShape.indexSecOrder ? 0 : 1);
      bitsVarBin[ind] += lengthHuffman(m_filterCoeffSet[ind][i], huffman);
    }
  }

  int zeroBitsVarBin = 0;
  for (int i = 0; i < alfShape.numCoeff - 1; i++)
  {
    huffman.setGroup(i < alfShape.indexSecOrder ? 0 : 1);
    zeroBitsVarBin += lengthHuffman(0, huffman);
  }

  const bool nonLinearFlag = m_alfParamTemp.lumaNonLinearFlag[altIdx];
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

int EncAdaptiveLoopFilterEcm::lengthHuffman(int coeffVal, AlfHuffmanCode &huffman)
{
  uint32_t symbol;
  int      length;
  huffman.encodeCoeff(coeffVal, symbol, length);
  return length;
}

float EncAdaptiveLoopFilterEcm::deriveFilterCoeffs(
  const CovarianceVec1D &cov, CovMergedArr1D &covMerged,
  int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], AlfFilterShape &alfShape,
  const short *const filterIndices, const int numFilters, float errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2],
  const bool nonLinear, const int classifierIdx, const bool isMaxNum, const int mergedPair[MAX_NUM_ALF_CLASSES][2],
  int8_t mergedScaleIdx[MAX_NUM_ALF_CLASSES], int mergedCoeff[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF],
  float mergedErr[MAX_NUM_ALF_CLASSES], const bool tryImproveScale)
{
  float          error  = 0.0;
  AlfCovariance &tmpCov = covMerged[MAX_NUM_ALF_CLASSES];

  int changedClass = -1;
  if (!isMaxNum)
  {
    memcpy(clipMerged[numFilters - 1], clipMerged[numFilters],
           sizeof(int[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF]));
  }

  const int maxNumClasses = ALF_NUM_CLASSES_CLASSIFIER[classifierIdx];
  for (int filtIdx = 0; filtIdx < numFilters; filtIdx++)
  {
    tmpCov.reset();
    bool foundClip     = false;
    bool changedFilter = isMaxNum;
    for (int classIdx = 0; classIdx < maxNumClasses; classIdx++)
    {
      if (filterIndices[classIdx] == filtIdx)
      {
        tmpCov += cov[classIdx];
        if (!foundClip)
        {
          foundClip = true;   // clip should be at the adress of shortest one
          if (changedFilter == false && mergedPair[numFilters][0] == classIdx)
          {
            changedFilter = true;
            if (nonLinear)
            {
              std::fill_n(clipMerged[numFilters - 1][classIdx], MAX_NUM_ALF_LUMA_COEFF, 2);
            }
          }
          changedClass              = classIdx;
          m_filterScaleIdx[filtIdx] = mergedScaleIdx[classIdx];
          memcpy(m_filterCoeffSet[filtIdx], mergedCoeff[classIdx], sizeof(int[MAX_NUM_ALF_LUMA_COEFF]));
          errorTabForce0Coeff[filtIdx][1] = mergedErr[classIdx];
          memcpy(m_filterClippSet[filtIdx], clipMerged[numFilters - 1][classIdx], sizeof(int[MAX_NUM_ALF_LUMA_COEFF]));
        }
      }
    }

    // Find coeffcients
    assert(alfShape.numCoeff == tmpCov.numCoeff);
    if (changedFilter)
    {
      errorTabForce0Coeff[filtIdx][1] = tmpCov.pixAcc +
        deriveCoeffQuant(m_filterClippSet[filtIdx], m_filterCoeffSet[filtIdx], m_filterScaleIdx[filtIdx], tmpCov,
                         alfShape, true, COEFF_SCALE_BITS_LUMA, COEFF_MANTISSA_BITS_LUMA, nonLinear, false,
                         tryImproveScale);

      mergedScaleIdx[changedClass] = m_filterScaleIdx[filtIdx];
      if (nonLinear)
      {
        memcpy(clipMerged[numFilters - 1][changedClass], m_filterClippSet[filtIdx],
               sizeof(int[MAX_NUM_ALF_LUMA_COEFF]));
      }
      memcpy(mergedCoeff[changedClass], m_filterCoeffSet[filtIdx], sizeof(int[MAX_NUM_ALF_LUMA_COEFF]));
      mergedErr[changedClass] = errorTabForce0Coeff[filtIdx][1];
    }
    errorTabForce0Coeff[filtIdx][0] = tmpCov.pixAcc;
    error += errorTabForce0Coeff[filtIdx][1];
  }
  return error;
}

float EncAdaptiveLoopFilterEcm::tryImproveFilterCoeffs(const CovarianceVec1D &cov, CovMergedArr1D &covMerged,
                                                       AlfFilterShape &alfShape, const short *const filterIndices,
                                                       const int numFilters, const bool nonLinear,
                                                       const int classifierIdx)
{
  float          error  = 0.0;
  AlfCovariance &tmpCov = covMerged[MAX_NUM_ALF_CLASSES];
  for (int filtIdx = 0; filtIdx < numFilters; filtIdx++)
  {
    tmpCov.reset();
    const int maxNumClasses = ALF_NUM_CLASSES_CLASSIFIER[classifierIdx];
    for (int classIdx = 0; classIdx < maxNumClasses; classIdx++)
    {
      if (filterIndices[classIdx] == filtIdx)
      {
        tmpCov += cov[classIdx];
      }
    }

    // Find coeffcients
    assert(alfShape.numCoeff == tmpCov.numCoeff);
    float curError = tmpCov.pixAcc +
      tryImproveCoeffQuant(m_filterClippSet[filtIdx], m_filterCoeffSet[filtIdx], m_filterScaleIdx[filtIdx], tmpCov,
                           alfShape, true, COEFF_SCALE_BITS_LUMA, COEFF_MANTISSA_BITS_LUMA, nonLinear);
    error += curError;
  }
  return error;
}

float EncAdaptiveLoopFilterEcm::deriveCoeffQuant(int *const filterClipp, int *const filterCoeffQuant, int8_t &scaleIdx,
                                                 const AlfCovariance &cov, const AlfFilterShape &shape,
                                                 const bool isLuma, const int fractionalBits, const int mantissaBits,
                                                 const bool optimizeClip, const bool tryImproveBySA,
                                                 const bool tryImproveScale)
{
  AlfCoeffRestriction coeffRestriction(fractionalBits, mantissaBits);
  coeffRestriction.init();
  const int factor   = 1 << fractionalBits;
  const int maxValue = coeffRestriction.getParam().maxValue;
  const int minValue = coeffRestriction.getParam().minValue;

  const int numCoeff = shape.numCoeff;
  float     filterCoeff[MAX_NUM_ALF_LUMA_COEFF];

  cov.optimizeFilter(shape.numCoeff, filterClipp, filterCoeff, optimizeClip, m_enableLessClip);

  int8_t scaleIdxFrom = 0, scaleIdxTo = 1;
  if (tryImproveScale)
  {
    scaleIdxFrom = 0, scaleIdxTo = (int8_t)(1 << ALF_SCALE_BITS_NUM);
  }
  int8_t bestScaleIdx = 0;
  int    bestFilterCoeffQuant[MAX_NUM_ALF_LUMA_COEFF];
  float  bestError = MAX_FLOAT, errRef = bestError;
  float  bestRdCost = MAX_FLOAT;
  for (scaleIdx = scaleIdxFrom; scaleIdx < scaleIdxTo; scaleIdx++)
  {
    float scaleFactor    = (float)ALF_SCALE_FACTOR[(int)scaleIdx] / (1 << ALF_SCALE_SHIFT);
    float scaleFactorInv = (float)(1 << ALF_SCALE_SHIFT) / ALF_SCALE_FACTOR[(int)scaleIdx];
    float filterCoeffScaled[MAX_NUM_ALF_LUMA_COEFF];
    for (int i = 0; i < numCoeff; i++)
    {
      filterCoeffScaled[i] = filterCoeff[i] * scaleFactorInv;
    }

    roundFiltCoeff(filterCoeffQuant, filterCoeffScaled, numCoeff, factor);

    for (int i = 0; i < numCoeff - 1; i++)
    {
      filterCoeffQuant[i] = std::min(maxValue, std::max(minValue, filterCoeffQuant[i]));
      filterCoeffQuant[i] =
        coeffRestriction.getParam().idxToCoeff[coeffRestriction.getParam().coeffToIdx[filterCoeffQuant[i] - minValue]];
    }
    filterCoeffQuant[numCoeff - 1] = 0;

    int modified = 1;
    if ((isLuma && m_encCfg->m_alfStrengthLuma != 1.0) || (!isLuma && m_encCfg->m_alfStrengthChroma != 1.0))
    {
      modified = 0;
    }
    float quadratic, dotProduct[MAX_NUM_ALF_LUMA_COEFF], constTermSum;
    float errRef =
      cov.initCachedCalcErrorForCoeffs(filterClipp, filterCoeffQuant, numCoeff, ALF_SCALE_FACTOR[scaleIdx],
                                       fractionalBits + ALF_SCALE_SHIFT, quadratic, dotProduct, constTermSum);
    while (modified)
    {
      modified = 0;
      for (int sign: { 1, -1 })
      {
        float errMin     = MAX_FLOAT;
        int   minInd     = -1;
        float coeffDelta = float(-sign) / factor * scaleFactor;

        for (int k = 0; k < numCoeff - 1; k++)
        {
          int oldVal = filterCoeffQuant[k];
          int newIdx = coeffRestriction.getParam().coeffToIdx[oldVal - minValue] - sign;
          if (newIdx < 0 || newIdx >= (int)coeffRestriction.getParam().idxToCoeff.size())
          {
            continue;
          }
          filterCoeffQuant[k] = coeffRestriction.getParam().idxToCoeff[newIdx];
          coeffDelta          = (double)(filterCoeffQuant[k] - oldVal) / (double)factor * scaleFactor;

          float error =
            cov.calcCachedCalcErrorForCoeffs(filterClipp, quadratic, dotProduct, constTermSum, k, coeffDelta);
          if (error < errMin)
          {
            errMin = error;
            minInd = k;
          }
          filterCoeffQuant[k] = oldVal;
        }
        if (errMin < errRef)
        {
          int oldVal = filterCoeffQuant[minInd];
          filterCoeffQuant[minInd] =
            coeffRestriction.getParam()
              .idxToCoeff[coeffRestriction.getParam().coeffToIdx[filterCoeffQuant[minInd] - minValue] - sign];
          coeffDelta = (double)(filterCoeffQuant[minInd] - oldVal) / (double)factor * scaleFactor;
          modified++;
          errRef = errMin;
          cov.updateCachedErrorForCoeffs(filterClipp, numCoeff, quadratic, dotProduct, constTermSum, minInd,
                                         coeffDelta);
        }
      }
    }
    // recalculate for computational stability
    errRef = cov.calcErrorForCoeffs(filterClipp, filterCoeffQuant, numCoeff, ALF_SCALE_FACTOR[scaleIdx],
                                    fractionalBits + ALF_SCALE_SHIFT);

    int    bits   = lengthFilterCoeffsOneFilter(shape, filterCoeffQuant, isLuma, fractionalBits, mantissaBits);
    double rdCost = errRef + m_lambda[isLuma ? COMP_Y : COMP_Cb] * bits;
    if (rdCost < bestRdCost)
    {
      bestRdCost   = rdCost;
      bestError    = errRef;
      bestScaleIdx = scaleIdx;
      std::copy_n(filterCoeffQuant, MAX_NUM_ALF_LUMA_COEFF, bestFilterCoeffQuant);
    }
  }
  errRef   = bestError;
  scaleIdx = bestScaleIdx;
  std::copy_n(bestFilterCoeffQuant, MAX_NUM_ALF_LUMA_COEFF, filterCoeffQuant);

  if (tryImproveBySA)
  {
    errRef = tryImproveCoeffQuant(filterClipp, filterCoeffQuant, scaleIdx, cov, shape, isLuma, fractionalBits,
                                  mantissaBits, optimizeClip);
  }
  return errRef;
}

float EncAdaptiveLoopFilterEcm::tryImproveCoeffQuant(int *const filterClipp, int *const filterCoeffQuant,
                                                     int8_t &scaleIdx, const AlfCovariance &cov,
                                                     const AlfFilterShape &shape, const bool isLuma,
                                                     const int fractionalBits, const int mantissaBits,
                                                     const bool optimizeClip)
{
  const int numCoeff = shape.numCoeff;
  float     errRef   = cov.calcErrorForCoeffs(filterClipp, filterCoeffQuant, numCoeff, ALF_SCALE_FACTOR[scaleIdx],
                                              fractionalBits + ALF_SCALE_SHIFT);
  int       bitsRef  = lengthFilterCoeffsOneFilter(shape, filterCoeffQuant, isLuma, fractionalBits, mantissaBits);
  float     costRef  = errRef + m_lambda[isLuma ? COMP_Y : COMP_Cb] * bitsRef;

  AlfCoeffRestriction coeffRestriction(fractionalBits, mantissaBits);
  coeffRestriction.init();
  const int factor   = 1 << fractionalBits;
  const int minValue = coeffRestriction.getParam().minValue;

  float coeffDelta, quadratic, dotProduct[MAX_NUM_ALF_LUMA_COEFF], constTermSum;

  int initFilterCoeff[MAX_NUM_ALF_LUMA_COEFF];
  std::copy_n(filterCoeffQuant, MAX_NUM_ALF_LUMA_COEFF, initFilterCoeff);

  struct StartPoint
  {
    std::vector<int> coeff;
    int8_t           scaleIdx;
    float            error;
    int              bits;
    float            rdCost;
  };
  std::vector<StartPoint> startQueue;
  std::vector<StartPoint> startPool;

  // Form the set of starting solutions. One set of coefficients for each scale factor
  int8_t scaleIdxFrom = 0, scaleIdxTo = (1 << ALF_SCALE_BITS_NUM);
  for (int8_t curScaleIdx = scaleIdxFrom; curScaleIdx < scaleIdxTo; curScaleIdx++)
  {
    startQueue.push_back(StartPoint());
    StartPoint &sp = startQueue.back();

    sp.coeff.assign(initFilterCoeff, initFilterCoeff + numCoeff);
    sp.scaleIdx = curScaleIdx;

    // Quick descent to optimize the set of coefficients for currect scale factor
    float scaleFactor = (float)ALF_SCALE_FACTOR[(int)curScaleIdx] / (1 << ALF_SCALE_SHIFT);
    sp.error = cov.initCachedCalcErrorForCoeffs(filterClipp, sp.coeff.data(), numCoeff, ALF_SCALE_FACTOR[curScaleIdx],
                                                fractionalBits + ALF_SCALE_SHIFT, quadratic, dotProduct, constTermSum);
    bool modified = true;
    while (modified)
    {
      modified = 0;
      for (int sign: { 1, -1 })
      {
        float errMin = MAX_FLOAT;
        int   minInd = -1;
        coeffDelta   = (float)-sign / (float)factor * scaleFactor;
        for (int k = 0; k < numCoeff - 1; k++)
        {
          int oldVal = sp.coeff[k];
          int newIdx = coeffRestriction.getParam().coeffToIdx[oldVal - minValue] - sign;
          if (newIdx < 0 || newIdx >= (int)coeffRestriction.getParam().idxToCoeff.size())
          {
            continue;
          }
          sp.coeff[k] = coeffRestriction.getParam().idxToCoeff[newIdx];
          coeffDelta  = (float)(sp.coeff[k] - oldVal) / (float)factor * scaleFactor;
          float error =
            cov.calcCachedCalcErrorForCoeffs(filterClipp, quadratic, dotProduct, constTermSum, k, coeffDelta);
          if (error < errMin)
          {
            errMin = error;
            minInd = k;
          }
          sp.coeff[k] = oldVal;
        }
        if (errMin < sp.error)
        {
          int oldVal       = sp.coeff[minInd];
          sp.coeff[minInd] = coeffRestriction.getParam()
                               .idxToCoeff[coeffRestriction.getParam().coeffToIdx[sp.coeff[minInd] - minValue] - sign];
          coeffDelta = (float)(sp.coeff[minInd] - oldVal) / (float)factor * scaleFactor;
          modified   = true;
          sp.error   = errMin;
          cov.updateCachedErrorForCoeffs(filterClipp, numCoeff, quadratic, dotProduct, constTermSum, minInd,
                                         coeffDelta);
        }
      }
    }
    sp.error  = cov.calcErrorForCoeffs(filterClipp, sp.coeff.data(), numCoeff, ALF_SCALE_FACTOR[sp.scaleIdx],
                                       fractionalBits + ALF_SCALE_SHIFT);
    sp.bits   = lengthFilterCoeffsOneFilter(shape, sp.coeff.data(), isLuma, fractionalBits, mantissaBits);
    sp.rdCost = sp.error + m_lambda[isLuma ? COMP_Y : COMP_Cb] * sp.bits;
  }

  // SA settings
  int startPoolSize = ALF_SA_SOLUTION_POOL_SIZE_MAX;
  int tmpFilterCoef[MAX_NUM_ALF_LUMA_COEFF], curFilterCoef[MAX_NUM_ALF_LUMA_COEFF];
  int posIdx[MAX_NUM_ALF_LUMA_COEFF];
  for (int i = 0; i < numCoeff - 1; i++)
  {
    posIdx[i] = i;
  }
  RandomGen gen;

  // Run SA multiple times taking starting solutions from startQueue/startPool
  for (int saRun = 0; saRun < ALF_SA_RUNS_COUNT; saRun++)
  {
    // Copy one of starting solutions
    if (startQueue.empty())
    {
      std::sort(startPool.begin(), startPool.end(),
                [](const StartPoint &p1, const StartPoint &p2) { return p1.rdCost < p2.rdCost; });
      startQueue.assign(startPool.rbegin(), startPool.rend());
      startPoolSize--;
      startPoolSize = std::max(startPoolSize, ALF_SA_SOLUTION_POOL_SIZE_MIN);
      if (startPoolSize < (int)startPool.size())
      {
        startPool.resize(startPoolSize);
      }
    }
    std::copy_n(startQueue.back().coeff.data(), numCoeff, curFilterCoef);
    std::copy_n(startQueue.back().coeff.data(), numCoeff, tmpFilterCoef);
    int8_t curScaleIdx = startQueue.back().scaleIdx;
    float  scaleFactor = (float)ALF_SCALE_FACTOR[(int)curScaleIdx] / (1 << ALF_SCALE_SHIFT);
    float  curError    = startQueue.back().error;
    int    curBits     = startQueue.back().bits;
    startQueue.pop_back();

    // Start SA iterations
    cov.initCachedCalcErrorForCoeffs(filterClipp, curFilterCoef, numCoeff, ALF_SCALE_FACTOR[curScaleIdx],
                                     fractionalBits + ALF_SCALE_SHIFT, quadratic, dotProduct, constTermSum);
    int iteration = 0, nonImprovingIterationNum = 0;
    while (nonImprovingIterationNum < numCoeff * ALF_SA_NON_IMPROVES_PER_PARAMETER_TO_STOP)
    {
      // Make a random move
      float error      = curError;
      int   posChanged = 0;
      std::copy_n(curFilterCoef, numCoeff, tmpFilterCoef);
      int curChangeCnt = ALF_SA_CHANGES_PER_ITERATION - (ALF_SA_CHANGES_PER_ITERATION > 1 ? (gen.nextRand() % 2) : 0);
      for (int j = 0; j < curChangeCnt; j++)
      {
        int idx = gen.nextRand() % (numCoeff - 1 - j);
        std::swap(posIdx[j], posIdx[j + idx]);
        int i            = posIdx[j];
        int move         = (gen.nextRand() % 2);
        move             = move * 2 - 1;
        int newIdx       = coeffRestriction.getParam().coeffToIdx[curFilterCoef[i] - minValue] - move;
        newIdx           = std::max(newIdx, 0);
        newIdx           = std::min(newIdx, (int)coeffRestriction.getParam().idxToCoeff.size() - 1);
        tmpFilterCoef[i] = coeffRestriction.getParam().idxToCoeff[newIdx];
        if (tmpFilterCoef[i] != curFilterCoef[i])
        {
          posChanged++;
          coeffDelta = (float)(tmpFilterCoef[i] - curFilterCoef[i]) / (float)factor * scaleFactor;
          error = cov.calcCachedCalcErrorForCoeffs(filterClipp, quadratic, dotProduct, constTermSum, i, coeffDelta);
          cov.updateCachedErrorForCoeffs(filterClipp, numCoeff, quadratic, dotProduct, constTermSum, i, coeffDelta);
        }
      }
      if (posChanged == 0)
      {
        continue;
      }

      // Decide about the move
      float prob      = 0;
      int   bits      = lengthFilterCoeffsOneFilter(shape, tmpFilterCoef, isLuma, fractionalBits, mantissaBits);
      float curLambda = 2.0 / (1.0 + pow(iteration + 1, -0.5)) - 1;
      curLambda *= m_lambda[isLuma ? COMP_Y : COMP_Cb];
      prob = (curError - error + curLambda * (curBits - bits));
      prob = prob * pow(iteration + 1, 2);
      prob = exp(-prob);
      prob = 1.0 / (1.0 + prob);

      if (gen.nextBool(prob))
      {
        // Accept the move
        if (prob > 0.9)
        {
          nonImprovingIterationNum = 0;
        }
        curError = error;
        curBits  = bits;
        std::copy_n(tmpFilterCoef, numCoeff, curFilterCoef);
      }
      else
      {
        // Decline the move
        for (int i = 0; i < numCoeff - 1; i++)
        {
          if (tmpFilterCoef[i] != curFilterCoef[i])
          {
            coeffDelta       = (float)(-tmpFilterCoef[i] + curFilterCoef[i]) / (float)factor * scaleFactor;
            tmpFilterCoef[i] = curFilterCoef[i];
            cov.updateCachedErrorForCoeffs(filterClipp, numCoeff, quadratic, dotProduct, constTermSum, i, coeffDelta);
          }
        }
      }
      iteration++;
      nonImprovingIterationNum++;
    }
    // Recalculate error for computational stability
    curError      = cov.calcErrorForCoeffs(filterClipp, tmpFilterCoef, numCoeff, ALF_SCALE_FACTOR[curScaleIdx],
                                           fractionalBits + ALF_SCALE_SHIFT);
    float curCost = curError + m_lambda[isLuma ? COMP_Y : COMP_Cb] * curBits;

    // Put the obtained solution into the pool of starting solutions
    bool isNew = true;
    for (int i = 0; i < (int)startPool.size(); i++)
    {
      if (curScaleIdx == startPool[i].scaleIdx &&
          std::equal(curFilterCoef, curFilterCoef + numCoeff, startPool[i].coeff.data()))
      {
        isNew = false;
        break;
      }
    }
    if (isNew)
    {
      int maxIdx = 0;
      if ((int)startPool.size() < startPoolSize)
      {
        startPool.emplace_back(StartPoint());
        startPool.back().rdCost = MAX_FLOAT;
        maxIdx                  = (int)startPool.size() - 1;
      }
      else
      {
        for (int i = 1; i < startPoolSize; i++)
        {
          if (startPool[i].rdCost > startPool[maxIdx].rdCost)
          {
            maxIdx = i;
          }
        }
      }
      if (curCost < startPool[maxIdx].rdCost)
      {
        startPool[maxIdx].coeff.assign(curFilterCoef, curFilterCoef + numCoeff);
        startPool[maxIdx].scaleIdx = curScaleIdx;
        startPool[maxIdx].error    = curError;
        startPool[maxIdx].bits     = curBits;
        startPool[maxIdx].rdCost   = curCost;
      }
    }

    // Save the solution into output if that is the best known
    if (curCost < costRef)
    {
      errRef  = curError;
      bitsRef = curBits;
      costRef = curCost;
      std::copy_n(curFilterCoef, numCoeff, filterCoeffQuant);
      scaleIdx = curScaleIdx;
    }
  }

  return errRef;
}

void EncAdaptiveLoopFilterEcm::mergeClasses(
  const AlfFilterShape &alfShape, const CovarianceVec1D &cov, CovMergedArr1D &covMerged,
  int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], const int numClasses,
  short filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES], const int altIdx,
  int mergedPair[MAX_NUM_ALF_CLASSES][2])
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

  const bool nonLinearFlag = m_alfParamTemp.lumaNonLinearFlag[altIdx];

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

  for (int i = 0; i < MAX_NUM_ALF_CLASSES; ++i)
  {
    for (int j = 0; j < MAX_NUM_ALF_CLASSES; ++j)
    {
      classChanged[i][j] = true;
      memset(clipHistory[i][j], 0, sizeof(int) * MAX_NUM_ALF_LUMA_COEFF);
    }
  }
  memset(errorHistory, 0, sizeof(float) * MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_CLASSES);

  // init Clip
  for (int i = 0; i < numClasses; i++)
  {
    std::fill_n(clipMerged[numRemaining - 1][i], MAX_NUM_ALF_LUMA_COEFF,
                nonLinearFlag ? ALF_NUM_CLIP_VALS[ChannelType::LUMA] / 2 : 0);
    if (nonLinearFlag)
    {
      err[i] = covMerged[i].optimizeFilterClip(alfShape.numCoeff, clipMerged[numRemaining - 1][i], m_enableLessClip);
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

            float errorMerged = 0;
            if (classChanged[i][j])
            {
              errorMerged        = tmpCov.calculateError(tmpClip);
              classChanged[i][j] = false;
              errorHistory[i][j] = errorMerged;
              memcpy(clipHistory[i][j], tmpClip, sizeof(tmpClip));
            }
            else
            {
              errorMerged = errorHistory[i][j];
              memcpy(tmpClip, clipHistory[i][j], sizeof(tmpClip));
            }
            float error = errorMerged - error1 - error2;

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
      classChanged[bestToMergeIdx1][i] = classChanged[i][bestToMergeIdx1] = true;
    }

    mergedPair[numRemaining - 1][0] = bestToMergeIdx1;
    mergedPair[numRemaining - 1][1] = bestToMergeIdx2;
    if (numRemaining == 2)
    {
      int ind = 0;
      for (int i = 0; i < numClasses; i++)
      {
        if (availableClass[i])
        {
          mergedPair[numRemaining - 2][ind] = i;
          ind++;
        }
      }
    }

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

void EncAdaptiveLoopFilterEcm::getFrameStats(const ChannelType channel, const int shapeIdx, int altIdx,
                                             int fixedFilterSetIdx, int classifierIdx)
{
  int numClasses = isLuma(channel) ? ALF_NUM_CLASSES_CLASSIFIER[classifierIdx] : 1;
  for (int i = 0; i < numClasses; i++)
  {
    m_alfCovarianceFrame[channel][shapeIdx][i].reset();
  }

  if (isLuma(channel))
  {
    getFrameStat(m_alfCovarianceFrame[ChannelType::LUMA][shapeIdx], m_alfCovariance[COMP_Y][shapeIdx],
                 (*m_modes)[COMP_Y], numClasses, altIdx, fixedFilterSetIdx, classifierIdx);
  }
  else
  {
    getFrameStat(m_alfCovarianceFrame[ChannelType::CHROMA][shapeIdx], m_alfCovariance[COMP_Cb][shapeIdx],
                 (*m_modes)[COMP_Cb], numClasses, altIdx, fixedFilterSetIdx, classifierIdx);
    getFrameStat(m_alfCovarianceFrame[ChannelType::CHROMA][shapeIdx], m_alfCovariance[COMP_Cr][shapeIdx],
                 (*m_modes)[COMP_Cr], numClasses, altIdx, fixedFilterSetIdx, classifierIdx);
  }
}

void EncAdaptiveLoopFilterEcm::getFrameStat(CovarianceVec1D &frameCov, const CovarianceVec4D &ctbCov,
                                            const CtbModes &ctbModes, const int numClasses, int altIdx,
                                            int fixedFilterSetIdx, int classifierIdx) const
{
  // XXX: Guessing the channel type from the number of classes.
  const ChannelType channel = numClasses > 1 ? ChannelType::LUMA : ChannelType::CHROMA;
  for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++)
  {
    if (CtbModeHandler::isEnabled(ctbModes[ctuIdx]))
    {
      for (int classIdx = 0; classIdx < numClasses; classIdx++)
      {
        if (altIdx ==
            (isLuma(channel) ? LumaCtbModeHandler::getAlternative(ctbModes[ctuIdx])
                             : ChromaCtbModeHandler::getAlternative(ctbModes[ctuIdx])))
        {
          frameCov[classIdx] += ctbCov[ctuIdx][fixedFilterSetIdx][classifierIdx][classIdx];
        }
      }
    }
  }
}

template<bool alfWSSD>
void EncAdaptiveLoopFilterEcm::deriveStatsForFiltering(PelUnitBuf &orgYuv, PelUnitBuf &recYuv, CodingStructure &cs)
{
  int       ctuRsAddr          = 0;
  const int numberOfComponents = getNumberValidComponents(m_chromaFormat);

  // init CTU stats buffers
  for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    const CompID compID = CompID(compIdx);
    for (int classifierIdx = 0; classifierIdx < (isLuma(compID) ? ALF_NUM_CLASSIFIER : 1); ++classifierIdx)
    {
      if (isLuma(compID) && classifierIdx == ALF_NUM_CLASSIFIER - 1 && cs.slice->isIntra())
      {
        continue;
      }

      const int numClasses = isLuma(compID) ? ALF_NUM_CLASSES_CLASSIFIER[classifierIdx] : 1;

      for (int shape = 0; shape != m_filterShapes[toChannelType(compID)].size(); shape++)
      {
        if (m_alfCovariance[compIdx].empty() || m_alfCovariance[compIdx][shape].empty())
        {
          continue;
        }

        if (m_filterTypeTest[toChannelType(compID)][m_filterShapes[toChannelType(compID)][shape].filterType] == false)
        {
          continue;
        }

        const int numFixFiltSetCands = numFixedFilterSetCands(m_filterShapes[toChannelType(compID)][shape].filterType);
        for (int classIdx = 0; classIdx < numClasses; classIdx++)
        {
          for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++)
          {
            for (int fixedFilterSetCandIdx = 0; fixedFilterSetCandIdx < numFixFiltSetCands; ++fixedFilterSetCandIdx)
            {
              m_alfCovariance[compIdx][shape][ctuIdx][fixedFilterSetCandIdx][classifierIdx][classIdx].reset();
            }
          }
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
      if (m_alfCovarianceFrame[chType][shape].empty())
      {
        continue;
      }

      if (m_filterTypeTest[chType][m_filterShapes[chType][shape].filterType] == false)
      {
        continue;
      }

      for (int classIdx = 0; classIdx < numClasses; classIdx++)
      {
        m_alfCovarianceFrame[chType][shape][classIdx].reset();
      }
    }
  }

  const PreCalcValues &pcv     = *cs.pcv;
  bool                 clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;

  PelBuf recYBeforeDb = m_tempBufBeforeDb.getBuf(COMP_Y).subBuf(cs.area.Y(), cs.area.Y());
  PelBuf resiY        = m_tempBufResi.subBuf(cs.area.Y(), cs.area.Y());

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
        recBuf.addMirrorExtension(MAX_ALF_PADDING_SIZE, false);
        recBuf = recBuf.subBuf(
          UnitArea(cs.area.chromaFormat,
                   Area(clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, width, height)));

        // Copy DBF input for the current block.
        PelBuf recBufDb = m_tempBufBeforeDbCtu.getBuf(COMP_Y).subBuf(0, 0, wBuf, hBuf);
        recBufDb.copyFrom(
          recYBeforeDb.subBuf(xPos - (clipL ? 0 : NUM_DB_PAD), yPos - (clipT ? 0 : NUM_DB_PAD), wBuf, hBuf));
        // pad top-left unavailable samples for raster slice
        if (rasterSliceAlfPad & 1)
        {
          recBufDb.padBorderPel(NUM_DB_PAD, NUM_DB_PAD, 1);
        }
        // pad bottom-right unavailable samples for raster slice
        if (rasterSliceAlfPad & 2)
        {
          recBufDb.padBorderPel(NUM_DB_PAD, NUM_DB_PAD, 2);
        }
        recBufDb.addMirrorExtension(NUM_DB_PAD);
        recBufDb = recBufDb.subBuf(clipL ? 0 : NUM_DB_PAD, clipT ? 0 : NUM_DB_PAD, width, height);

        // Copy residual for the current block.
        PelBuf resiBuf = m_tempBufResiCtu.subBuf(0, 0, wBuf, hBuf);
        resiBuf.copyFrom(
          resiY.subBuf(xPos - (clipL ? 0 : NUM_RESI_PAD), yPos - (clipT ? 0 : NUM_RESI_PAD), wBuf, hBuf));
        // pad top-left unavailable samples for raster slice
        if (rasterSliceAlfPad & 1)
        {
          resiBuf.padBorderPel(NUM_RESI_PAD, NUM_RESI_PAD, 1);
        }
        // pad bottom-right unavailable samples for raster slice
        if (rasterSliceAlfPad & 2)
        {
          resiBuf.padBorderPel(NUM_RESI_PAD, NUM_RESI_PAD, 2);
        }
        resiBuf.addMirrorExtension(NUM_RESI_PAD);
        resiBuf = resiBuf.subBuf(clipL ? 0 : NUM_RESI_PAD, clipT ? 0 : NUM_RESI_PAD, width, height);

        static_assert(NUM_FIXED_FILTER_SET_CANDS == 2, "Implementation expects 2 fixed filter set candidates.");
        copyAndExtendFixedFilterResultsCtu(FixFiltSetCand::FIRST, FixFiltIdx::SECOND, Area(xPos, yPos, width, height));
        copyAndExtendFixedFilterResultsCtu(FixFiltSetCand::SECOND, FixFiltIdx::SECOND, Area(xPos, yPos, width, height));
        copyAndExtendGaussResultsCtu(Area(xPos, yPos, width, height));

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

            if (m_filterTypeTest[chType][m_filterShapes[chType][shape].filterType] == false)
            {
              continue;
            }

            for (unsigned fixedFilterSetCandIdx = 0;
                 fixedFilterSetCandIdx < numFixedFilterSetCands(m_filterShapes[chType][shape].filterType);
                 fixedFilterSetCandIdx++)
            {
              const auto classifierType = ClassifierIndex::ADAPT_FILTER_GRAD_BASED;
              const auto classifierIdx  = static_cast<unsigned>(classifierType);
              getBlkStats<alfWSSD, false>(
                m_alfCovariance[compIdx][shape][ctuRsAddr][fixedFilterSetCandIdx][classifierIdx],
                m_filterShapes[chType][shape], compIdx ? nullptr : &m_classifier[classifierType], org, orgStride,
                orgLuma, orgLumaStride, rec, recStride, nullptr, nullptr,
                isLuma(compID) ? recBufDb.bufAt(compArea) : nullptr, isLuma(compID) ? recBufDb.stride : 0,
                isLuma(compID) ? resiBuf.bufAt(compAreaDst) : nullptr, isLuma(compID) ? resiBuf.stride : 0, compAreaDst,
                compArea, compID, fixedFilterSetCandIdx, classifierIdx, true);

              if (isLuma(compID))
              {
                const auto classifierType = ClassifierIndex::ADAPT_FILTER_BAND_BASED_RECO;
                const auto classifierIdx  = static_cast<unsigned>(classifierType);
                if (cs.slice->isIntra())
                {
                  getBlkStats<alfWSSD, false>(
                    m_alfCovariance[compIdx][shape][ctuRsAddr][fixedFilterSetCandIdx][classifierIdx],
                    m_filterShapes[chType][shape], &m_classifier[classifierType], org, orgStride, orgLuma,
                    orgLumaStride, rec, recStride, nullptr, nullptr, recBufDb.bufAt(compArea), recBufDb.stride,
                    resiBuf.bufAt(compAreaDst), resiBuf.stride, compAreaDst, compArea, compID, fixedFilterSetCandIdx,
                    classifierIdx, true);
                }
                else
                {
                  const auto classifierTypeNext = ClassifierIndex::ADAPT_FILTER_BAND_BASED_RESI;
                  const auto classifierIdxNext  = static_cast<unsigned>(classifierTypeNext);
                  getBlkStats<alfWSSD, true>(
                    m_alfCovariance[compIdx][shape][ctuRsAddr][fixedFilterSetCandIdx][classifierIdx],
                    m_filterShapes[chType][shape], &m_classifier[classifierType], org, orgStride, orgLuma,
                    orgLumaStride, rec, recStride, &m_classifier[classifierTypeNext],
                    &m_alfCovariance[compIdx][shape][ctuRsAddr][fixedFilterSetCandIdx][classifierIdxNext],
                    recBufDb.bufAt(compArea), recBufDb.stride, resiBuf.bufAt(compAreaDst), resiBuf.stride, compAreaDst,
                    compArea, compID, fixedFilterSetCandIdx, classifierIdx, true);
                }
              }
            }

            if (!isLuma(compID))
            {
              m_alfCovarianceFrame[chType][shape][0] += m_alfCovariance[compIdx][shape][ctuRsAddr][0][0][0];
            }
          }
        }
      }
      else
      {
        const bool isFixFiltPaddedPerCtu = isFixedFilterPaddedPerCtu(*cs.slice);
        if (isFixFiltPaddedPerCtu)
        {
          static_assert(NUM_FIXED_FILTER_SET_CANDS == 2, "Implementation expects 2 fixed filter set candidates.");
          copyAndExtendFixedFilterResultsCtu(FixFiltSetCand::FIRST, FixFiltIdx::SECOND,
                                             Area(xPos, yPos, width, height));
          copyAndExtendFixedFilterResultsCtu(FixFiltSetCand::SECOND, FixFiltIdx::SECOND,
                                             Area(xPos, yPos, width, height));
          copyAndExtendGaussResultsCtu(Area(xPos, yPos, width, height));
        }

        const UnitArea area(m_chromaFormat, Area(xPos, yPos, width, height));

        for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
        {
          const CompID    compID   = CompID(compIdx);
          const CompArea &compArea = area.block(compID);

          ptrdiff_t recStride = recYuv.get(compID).stride;
          Pel      *rec       = recYuv.get(compID).bufAt(compArea);

          const ptrdiff_t  recDbStride = isLuma(compID) ? recYBeforeDb.stride : 0;
          const Pel *const recDb       = isLuma(compID) ? recYBeforeDb.bufAt(compArea) : nullptr;

          const ptrdiff_t  resiStride = isLuma(compID) ? resiY.stride : 0;
          const Pel *const resi       = isLuma(compID) ? resiY.bufAt(compArea) : nullptr;

          ptrdiff_t orgStride = orgYuv.get(compID).stride;
          Pel      *org       = orgYuv.get(compID).bufAt(compArea);

          ptrdiff_t orgLumaStride = orgYuv.get(COMP_Y).stride;
          Pel      *orgLuma       = orgYuv.get(COMP_Y).bufAt(area.block(COMP_Y));

          ChannelType chType = toChannelType(compID);

          for (int shape = 0; shape != m_filterShapes[chType].size(); shape++)
          {
            if (m_filterTypeTest[chType][m_filterShapes[chType][shape].filterType] == false)
            {
              continue;
            }

            for (int fixedFilterSetCandIdx = 0;
                 fixedFilterSetCandIdx < numFixedFilterSetCands(m_filterShapes[chType][shape].filterType);
                 fixedFilterSetCandIdx++)
            {
              const auto classifierType = ClassifierIndex::ADAPT_FILTER_GRAD_BASED;
              const auto classifierIdx  = static_cast<unsigned>(classifierType);
              getBlkStats<alfWSSD, false>(
                m_alfCovariance[compIdx][shape][ctuRsAddr][fixedFilterSetCandIdx][classifierIdx],
                m_filterShapes[chType][shape], compIdx ? nullptr : &m_classifier[classifierType], org, orgStride,
                orgLuma, orgLumaStride, rec, recStride, nullptr, nullptr, recDb, recDbStride, resi, resiStride,
                compArea, compArea, compID, fixedFilterSetCandIdx, classifierIdx, isFixFiltPaddedPerCtu);

              if (isLuma(compID))
              {
                const auto classifierType = ClassifierIndex::ADAPT_FILTER_BAND_BASED_RECO;
                const auto classifierIdx  = static_cast<unsigned>(classifierType);
                if (cs.slice->isIntra())
                {
                  getBlkStats<alfWSSD, false>(
                    m_alfCovariance[compIdx][shape][ctuRsAddr][fixedFilterSetCandIdx][classifierIdx],
                    m_filterShapes[chType][shape], &m_classifier[classifierType], org, orgStride, orgLuma,
                    orgLumaStride, rec, recStride, nullptr, nullptr, recDb, recDbStride, resi, resiStride, compArea,
                    compArea, compID, fixedFilterSetCandIdx, classifierIdx, isFixFiltPaddedPerCtu);
                }
                else
                {
                  const auto classifierTypeNext = ClassifierIndex::ADAPT_FILTER_BAND_BASED_RESI;
                  const auto classifierIdxNext  = static_cast<unsigned>(classifierTypeNext);
                  getBlkStats<alfWSSD, true>(
                    m_alfCovariance[compIdx][shape][ctuRsAddr][fixedFilterSetCandIdx][classifierIdx],
                    m_filterShapes[chType][shape], &m_classifier[getClassifierIdxEnum(classifierIdx)], org, orgStride,
                    orgLuma, orgLumaStride, rec, recStride, &m_classifier[classifierTypeNext],
                    &m_alfCovariance[compIdx][shape][ctuRsAddr][fixedFilterSetCandIdx][classifierIdxNext], recDb,
                    recDbStride, resi, resiStride, compArea, compArea, compID, fixedFilterSetCandIdx, classifierIdx,
                    isFixFiltPaddedPerCtu);
                }
              }
            }

            if (!isLuma(compID))
            {
              m_alfCovarianceFrame[chType][shape][0] += m_alfCovariance[compIdx][shape][ctuRsAddr][0][0][0];
            }
          }
        }
      }
      ctuRsAddr++;
    }
  }

  initDistortion<alfWSSD>(cs);
}

template<bool alfWSSD, bool reuse> void EncAdaptiveLoopFilterEcm::getBlkStats(
  CovarianceVec1D &alfCovariance, const AlfFilterShape &shape, const ClassBuf *const classifier, const Pel *org,
  const ptrdiff_t orgStride, const Pel *orgLuma, const ptrdiff_t orgLumaStride, const Pel *rec,
  const ptrdiff_t recStride, const ClassBuf *const classifierNext, CovarianceVec1D *const alfCovarianceNext,
  const Pel *recBeforeDb, const ptrdiff_t recBeforeDbStride, const Pel *resi, const ptrdiff_t resiStride,
  const CompArea &areaDst, const CompArea &area, const CompID compID, const unsigned fixedFilterSetCandIdx,
  const int classifierIdx, const bool isFixFiltPaddedPerCtu)
{
  Pel ELocal[MAX_NUM_ALF_LUMA_COEFF][MAX_ALF_NUM_CLIP_VALS];

  const int numBins = (isLuma(compID) ? m_encCfg->m_useNonLinearAlfLuma : m_encCfg->m_useNonLinearAlfChroma)
    ? ALF_NUM_CLIP_VALS[toChannelType(compID)]
    : 1;

  const float strength    = isLuma(compID) ? m_encCfg->m_alfStrengthTargetLuma : m_encCfg->m_alfStrengthTargetChroma;
  const float invStrength = strength != 0.0 ? 1.0 / strength : 0.0;

  for (int i = 0; i < area.height; i++)
  {
    for (int j = 0; j < area.width; j++)
    {
      const AlfClassifier *classifierSample =
        (classifier == nullptr) ? nullptr : &((*classifier)[areaDst.y + i][areaDst.x + j]);
      const int classIdx     = (classifier == nullptr) ? 0 : (*classifierSample) >> 2;
      const int transposeIdx = (classifier == nullptr) ? 0 : (*classifierSample) & 0x3;

      const int classIdxNext = (classifierNext == nullptr) ? 0 : (*classifierNext)[areaDst.y + i][areaDst.x + j] >> 2;
      CHECK((classifierNext != nullptr) && !alfCovariance[classIdx].sameSizeAs((*alfCovarianceNext)[classIdxNext]),
            "Covariance size mismatch");

      std::fill_n(ELocal[0], MAX_NUM_ALF_LUMA_COEFF * MAX_ALF_NUM_CLIP_VALS, 0);

      calcCovariance(ELocal, rec + j, recStride, recBeforeDb == nullptr ? nullptr : recBeforeDb + j, recBeforeDbStride,
                     resi == nullptr ? nullptr : resi + j, resiStride, shape, transposeIdx, compID,
                     Position(areaDst.x + j, areaDst.y + i), Position(area.x + j, area.y + i), fixedFilterSetCandIdx,
                     Position(j, i), isFixFiltPaddedPerCtu);

      const Pel *lumaPtr = orgLuma + (i << ::getComponentScaleY(compID, m_chromaFormat)) * orgLumaStride +
        (j << ::getComponentScaleX(compID, m_chromaFormat));
      const float weight = alfWSSD ? m_lumaLevelToWeightPLUT[*lumaPtr] : 1.0;
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
              const float     curE = we * e[b1][l];
              const ptrdiff_t oe   = alfCovariance[classIdx].getOffsetEfast(b0, b1, k, l);
              alfCovariance[classIdx].data[oe] += curE;
              if (reuse)
              {
                (*alfCovarianceNext)[classIdxNext].data[oe] += curE;
              }
            }
          }

          const float     curY = we * yLocal;
          const ptrdiff_t oy   = alfCovariance[classIdx].getOffsetY(b0, k);
          alfCovariance[classIdx].data[oy] += curY;
          if (reuse)
          {
            (*alfCovarianceNext)[classIdxNext].data[oy] += curY;
          }
        }
      }

      const float curP = weight * yLocal * yLocal;
      alfCovariance[classIdx].pixAcc += curP;
      if (reuse)
      {
        (*alfCovarianceNext)[classIdxNext].pixAcc += curP;
      }
    }

    org += orgStride;
    rec += recStride;
    recBeforeDb += recBeforeDbStride;
    resi += resiStride;
  }
}

void EncAdaptiveLoopFilterEcm::calcCovariance(Pel ELocal[MAX_NUM_ALF_LUMA_COEFF][MAX_ALF_NUM_CLIP_VALS], const Pel *rec,
                                              const ptrdiff_t stride, const Pel *recBeforeDb,
                                              const ptrdiff_t recBeforeDbStride, const Pel *resi,
                                              const ptrdiff_t resiStride, const AlfFilterShape &shape,
                                              const int transposeIdx, const CompID compID, Position posDst,
                                              Position pos, const unsigned fixedFilterSetCandIdx, Position posInCtu,
                                              const bool isFixFiltPaddedPerCtu)
{
  const int *filterPattern    = shape.pattern.data();
  const int  halfFilterLength = shape.filterLength >> 1;
  const Pel *clip             = m_alfClippingValues[toChannelType(compID)].data();
  const int  numBins          = (isLuma(compID) ? m_encCfg->m_useNonLinearAlfLuma : m_encCfg->m_useNonLinearAlfChroma)
              ? ALF_NUM_CLIP_VALS[toChannelType(compID)]
              : 1;

  int k = 0;

  const Pel curr = rec[0];

  CHECKD(fixedFilterSetCandIdx >= NUM_FIXED_FILTER_SET_CANDS, "Fixed filter set candidate index is out of range.");
  const FixFiltSetCand fixedFilterSetCand = static_cast<FixFiltSetCand>(fixedFilterSetCandIdx);

  if (shape.filterType == ALF_FILTER_9_EXT_DB_RESI || shape.filterType == ALF_FILTER_9_EXT_DB_RESI_DIRECT)
  {
    const FilterRowPtrs<const Pel, 9> pImg(rec, stride);

    if (transposeIdx == 1)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[6][b] += clipALF(clip[b], curr, pImg[+4][+0], pImg[-4][-0]);
        ELocal[7][b] += clipALF(clip[b], curr, pImg[+3][+0], pImg[-3][-0]);
        ELocal[8][b] += clipALF(clip[b], curr, pImg[+2][+0], pImg[-2][-0]);
        ELocal[3][b] += clipALF(clip[b], curr, pImg[+1][+1], pImg[-1][-1]);
        ELocal[9][b] += clipALF(clip[b], curr, pImg[+1][+0], pImg[-1][-0]);
        ELocal[5][b] += clipALF(clip[b], curr, pImg[+1][-1], pImg[-1][+1]);
        ELocal[0][b] += clipALF(clip[b], curr, pImg[+0][+4], pImg[-0][-4]);
        ELocal[1][b] += clipALF(clip[b], curr, pImg[+0][+3], pImg[-0][-3]);
        ELocal[2][b] += clipALF(clip[b], curr, pImg[+0][+2], pImg[-0][-2]);
        ELocal[4][b] += clipALF(clip[b], curr, pImg[+0][+1], pImg[-0][-1]);
      }
    }
    else if (transposeIdx == 2)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[0][b] += clipALF(clip[b], curr, pImg[+4][+0], pImg[-4][-0]);
        ELocal[1][b] += clipALF(clip[b], curr, pImg[+3][+0], pImg[-3][-0]);
        ELocal[2][b] += clipALF(clip[b], curr, pImg[+2][+0], pImg[-2][-0]);
        ELocal[5][b] += clipALF(clip[b], curr, pImg[+1][+1], pImg[-1][-1]);
        ELocal[4][b] += clipALF(clip[b], curr, pImg[+1][+0], pImg[-1][-0]);
        ELocal[3][b] += clipALF(clip[b], curr, pImg[+1][-1], pImg[-1][+1]);
        ELocal[6][b] += clipALF(clip[b], curr, pImg[+0][+4], pImg[-0][-4]);
        ELocal[7][b] += clipALF(clip[b], curr, pImg[+0][+3], pImg[-0][-3]);
        ELocal[8][b] += clipALF(clip[b], curr, pImg[+0][+2], pImg[-0][-2]);
        ELocal[9][b] += clipALF(clip[b], curr, pImg[+0][+1], pImg[-0][-1]);
      }
    }
    else if (transposeIdx == 3)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[6][b] += clipALF(clip[b], curr, pImg[+4][+0], pImg[-4][-0]);
        ELocal[7][b] += clipALF(clip[b], curr, pImg[+3][+0], pImg[-3][-0]);
        ELocal[8][b] += clipALF(clip[b], curr, pImg[+2][+0], pImg[-2][-0]);
        ELocal[5][b] += clipALF(clip[b], curr, pImg[+1][+1], pImg[-1][-1]);
        ELocal[9][b] += clipALF(clip[b], curr, pImg[+1][+0], pImg[-1][-0]);
        ELocal[3][b] += clipALF(clip[b], curr, pImg[+1][-1], pImg[-1][+1]);
        ELocal[0][b] += clipALF(clip[b], curr, pImg[+0][+4], pImg[-0][-4]);
        ELocal[1][b] += clipALF(clip[b], curr, pImg[+0][+3], pImg[-0][-3]);
        ELocal[2][b] += clipALF(clip[b], curr, pImg[+0][+2], pImg[-0][-2]);
        ELocal[4][b] += clipALF(clip[b], curr, pImg[+0][+1], pImg[-0][-1]);
      }
    }
    else
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[0][b] += clipALF(clip[b], curr, pImg[+4][+0], pImg[-4][-0]);
        ELocal[1][b] += clipALF(clip[b], curr, pImg[+3][+0], pImg[-3][-0]);
        ELocal[2][b] += clipALF(clip[b], curr, pImg[+2][+0], pImg[-2][-0]);
        ELocal[3][b] += clipALF(clip[b], curr, pImg[+1][+1], pImg[-1][-1]);
        ELocal[4][b] += clipALF(clip[b], curr, pImg[+1][+0], pImg[-1][-0]);
        ELocal[5][b] += clipALF(clip[b], curr, pImg[+1][-1], pImg[-1][+1]);
        ELocal[6][b] += clipALF(clip[b], curr, pImg[+0][+4], pImg[-0][-4]);
        ELocal[7][b] += clipALF(clip[b], curr, pImg[+0][+3], pImg[-0][-3]);
        ELocal[8][b] += clipALF(clip[b], curr, pImg[+0][+2], pImg[-0][-2]);
        ELocal[9][b] += clipALF(clip[b], curr, pImg[+0][+1], pImg[-0][-1]);
      }
    }
    k = 10;
  }
  else
  {
    if (transposeIdx == 0)
    {
      for (int i = -halfFilterLength; i < 0; i++)
      {
        const Pel *rec0 = rec + i * stride;
        const Pel *rec1 = rec - i * stride;
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
            ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec0[i * stride], rec1[-i * stride]);
          }
        }
      }
      for (int i = -halfFilterLength; i < 0; i++, k++)
      {
        for (int b = 0; b < numBins; b++)
        {
          ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec[i * stride], rec[-i * stride]);
        }
      }
    }
    else if (transposeIdx == 2)
    {
      for (int i = -halfFilterLength; i < 0; i++)
      {
        const Pel *rec0 = rec + i * stride;
        const Pel *rec1 = rec - i * stride;
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
            ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec0[i * stride], rec1[-i * stride]);
          }
        }
      }
      for (int i = -halfFilterLength; i < 0; i++, k++)
      {
        for (int b = 0; b < numBins; b++)
        {
          ELocal[filterPattern[k]][b] += clipALF(clip[b], curr, rec[i * stride], rec[-i * stride]);
        }
      }
    }
  }   // data collection for new shape

  if (shape.filterType == ALF_FILTER_9_EXT_DB_RESI || shape.filterType == ALF_FILTER_9_EXT_DB_RESI_DIRECT)
  {
    const Pel *pImgFixedBasedCenter;
    const Pel *pImgGaussCenter;
    ptrdiff_t  fixedBasedStride;
    ptrdiff_t  gaussStride;
    if (isFixFiltPaddedPerCtu)
    {
      const auto &secFixedFilterResult = m_fixedFilterResultPerCtu[fixedFilterSetCand][FixFiltIdx::SECOND];
      pImgFixedBasedCenter             = &secFixedFilterResult[posInCtu.y][posInCtu.x];
      fixedBasedStride                 = secFixedFilterResult.stride;

      pImgGaussCenter = &m_gaussCtu[posInCtu.y][posInCtu.x];
      gaussStride     = m_gaussCtu.stride;
    }
    else
    {
      const auto &secFixedFilterResult = m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::SECOND];
      pImgFixedBasedCenter             = &secFixedFilterResult[pos.y][pos.x];
      fixedBasedStride                 = secFixedFilterResult.stride;

      pImgGaussCenter = &m_gaussPic[posDst.y][posDst.x];
      gaussStride     = m_gaussPic.stride;
    }
    const FilterRowPtrs<const Pel, 13> pImgFixedBased(pImgFixedBasedCenter, fixedBasedStride);
    // TODO: ALF: Need only size of 1 for ALF_FILTER_9_EXT_DB_RESI_DIRECT. BS 2023-10-04
    const FilterRowPtrs<const Pel, 5>  pImgGauss(pImgGaussCenter, gaussStride);

    if (transposeIdx == 1)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[22][b] += clipALF(clip[b], curr, pImgFixedBased[-6][+0], pImgFixedBased[+6][-0]);
        ELocal[23][b] += clipALF(clip[b], curr, pImgFixedBased[-5][+0], pImgFixedBased[+5][-0]);
        ELocal[24][b] += clipALF(clip[b], curr, pImgFixedBased[-4][+0], pImgFixedBased[+4][-0]);
        ELocal[25][b] += clipALF(clip[b], curr, pImgFixedBased[-3][+0], pImgFixedBased[+3][-0]);
        ELocal[17][b] += clipALF(clip[b], curr, pImgFixedBased[-2][-1], pImgFixedBased[+2][+1]);
        ELocal[26][b] += clipALF(clip[b], curr, pImgFixedBased[-2][-0], pImgFixedBased[+2][+0]);
        ELocal[21][b] += clipALF(clip[b], curr, pImgFixedBased[-2][+1], pImgFixedBased[+2][-1]);
        ELocal[14][b] += clipALF(clip[b], curr, pImgFixedBased[-1][-2], pImgFixedBased[+1][+2]);
        ELocal[18][b] += clipALF(clip[b], curr, pImgFixedBased[-1][-1], pImgFixedBased[+1][+1]);
        ELocal[27][b] += clipALF(clip[b], curr, pImgFixedBased[-1][-0], pImgFixedBased[+1][+0]);
        ELocal[20][b] += clipALF(clip[b], curr, pImgFixedBased[-1][+1], pImgFixedBased[+1][-1]);
        ELocal[16][b] += clipALF(clip[b], curr, pImgFixedBased[-1][+2], pImgFixedBased[+1][-2]);
        ELocal[10][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+6], pImgFixedBased[+0][-6]);
        ELocal[11][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+5], pImgFixedBased[+0][-5]);
        ELocal[12][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+4], pImgFixedBased[+0][-4]);
        ELocal[13][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+3], pImgFixedBased[+0][-3]);
        ELocal[15][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+2], pImgFixedBased[+0][-2]);
        ELocal[19][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+1], pImgFixedBased[+0][-1]);
      }
    }
    else if (transposeIdx == 2)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[10][b] += clipALF(clip[b], curr, pImgFixedBased[-6][+0], pImgFixedBased[+6][-0]);
        ELocal[11][b] += clipALF(clip[b], curr, pImgFixedBased[-5][+0], pImgFixedBased[+5][-0]);
        ELocal[12][b] += clipALF(clip[b], curr, pImgFixedBased[-4][+0], pImgFixedBased[+4][-0]);
        ELocal[13][b] += clipALF(clip[b], curr, pImgFixedBased[-3][+0], pImgFixedBased[+3][-0]);
        ELocal[16][b] += clipALF(clip[b], curr, pImgFixedBased[-2][-1], pImgFixedBased[+2][+1]);
        ELocal[15][b] += clipALF(clip[b], curr, pImgFixedBased[-2][-0], pImgFixedBased[+2][+0]);
        ELocal[14][b] += clipALF(clip[b], curr, pImgFixedBased[-2][+1], pImgFixedBased[+2][-1]);
        ELocal[21][b] += clipALF(clip[b], curr, pImgFixedBased[-1][-2], pImgFixedBased[+1][+2]);
        ELocal[20][b] += clipALF(clip[b], curr, pImgFixedBased[-1][-1], pImgFixedBased[+1][+1]);
        ELocal[19][b] += clipALF(clip[b], curr, pImgFixedBased[-1][-0], pImgFixedBased[+1][+0]);
        ELocal[18][b] += clipALF(clip[b], curr, pImgFixedBased[-1][+1], pImgFixedBased[+1][-1]);
        ELocal[17][b] += clipALF(clip[b], curr, pImgFixedBased[-1][+2], pImgFixedBased[+1][-2]);
        ELocal[22][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+6], pImgFixedBased[+0][-6]);
        ELocal[23][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+5], pImgFixedBased[+0][-5]);
        ELocal[24][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+4], pImgFixedBased[+0][-4]);
        ELocal[25][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+3], pImgFixedBased[+0][-3]);
        ELocal[26][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+2], pImgFixedBased[+0][-2]);
        ELocal[27][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+1], pImgFixedBased[+0][-1]);
      }
    }
    else if (transposeIdx == 3)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[22][b] += clipALF(clip[b], curr, pImgFixedBased[-6][+0], pImgFixedBased[+6][-0]);
        ELocal[23][b] += clipALF(clip[b], curr, pImgFixedBased[-5][+0], pImgFixedBased[+5][-0]);
        ELocal[24][b] += clipALF(clip[b], curr, pImgFixedBased[-4][+0], pImgFixedBased[+4][-0]);
        ELocal[25][b] += clipALF(clip[b], curr, pImgFixedBased[-3][+0], pImgFixedBased[+3][-0]);
        ELocal[21][b] += clipALF(clip[b], curr, pImgFixedBased[-2][-1], pImgFixedBased[+2][+1]);
        ELocal[26][b] += clipALF(clip[b], curr, pImgFixedBased[-2][-0], pImgFixedBased[+2][+0]);
        ELocal[17][b] += clipALF(clip[b], curr, pImgFixedBased[-2][+1], pImgFixedBased[+2][-1]);
        ELocal[16][b] += clipALF(clip[b], curr, pImgFixedBased[-1][-2], pImgFixedBased[+1][+2]);
        ELocal[20][b] += clipALF(clip[b], curr, pImgFixedBased[-1][-1], pImgFixedBased[+1][+1]);
        ELocal[27][b] += clipALF(clip[b], curr, pImgFixedBased[-1][-0], pImgFixedBased[+1][+0]);
        ELocal[18][b] += clipALF(clip[b], curr, pImgFixedBased[-1][+1], pImgFixedBased[+1][-1]);
        ELocal[14][b] += clipALF(clip[b], curr, pImgFixedBased[-1][+2], pImgFixedBased[+1][-2]);
        ELocal[10][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+6], pImgFixedBased[+0][-6]);
        ELocal[11][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+5], pImgFixedBased[+0][-5]);
        ELocal[12][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+4], pImgFixedBased[+0][-4]);
        ELocal[13][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+3], pImgFixedBased[+0][-3]);
        ELocal[15][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+2], pImgFixedBased[+0][-2]);
        ELocal[19][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+1], pImgFixedBased[+0][-1]);
      }
    }
    else
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[10][b] += clipALF(clip[b], curr, pImgFixedBased[-6][+0], pImgFixedBased[+6][-0]);
        ELocal[11][b] += clipALF(clip[b], curr, pImgFixedBased[-5][+0], pImgFixedBased[+5][-0]);
        ELocal[12][b] += clipALF(clip[b], curr, pImgFixedBased[-4][+0], pImgFixedBased[+4][-0]);
        ELocal[13][b] += clipALF(clip[b], curr, pImgFixedBased[-3][+0], pImgFixedBased[+3][-0]);
        ELocal[14][b] += clipALF(clip[b], curr, pImgFixedBased[-2][-1], pImgFixedBased[+2][+1]);
        ELocal[15][b] += clipALF(clip[b], curr, pImgFixedBased[-2][-0], pImgFixedBased[+2][+0]);
        ELocal[16][b] += clipALF(clip[b], curr, pImgFixedBased[-2][+1], pImgFixedBased[+2][-1]);
        ELocal[17][b] += clipALF(clip[b], curr, pImgFixedBased[-1][-2], pImgFixedBased[+1][+2]);
        ELocal[18][b] += clipALF(clip[b], curr, pImgFixedBased[-1][-1], pImgFixedBased[+1][+1]);
        ELocal[19][b] += clipALF(clip[b], curr, pImgFixedBased[-1][-0], pImgFixedBased[+1][+0]);
        ELocal[20][b] += clipALF(clip[b], curr, pImgFixedBased[-1][+1], pImgFixedBased[+1][-1]);
        ELocal[21][b] += clipALF(clip[b], curr, pImgFixedBased[-1][+2], pImgFixedBased[+1][-2]);
        ELocal[22][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+6], pImgFixedBased[+0][-6]);
        ELocal[23][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+5], pImgFixedBased[+0][-5]);
        ELocal[24][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+4], pImgFixedBased[+0][-4]);
        ELocal[25][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+3], pImgFixedBased[+0][-3]);
        ELocal[26][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+2], pImgFixedBased[+0][-2]);
        ELocal[27][b] += clipALF(clip[b], curr, pImgFixedBased[-0][+1], pImgFixedBased[+0][-1]);
      }
    }

    const FilterRowPtrs<const Pel, 3> pRecDbTmp(recBeforeDb, recBeforeDbStride);
    const Pel *const                  pResiTmp0 = resi;

    if (shape.filterType == ALF_FILTER_9_EXT_DB_RESI_DIRECT)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[28][b] += clipALF(clip[b], curr, pImgGauss[+2][+0], pImgGauss[-2][-0]);
        ELocal[29][b] += clipALF(clip[b], curr, pImgGauss[+1][+0], pImgGauss[-1][-0]);
        ELocal[30][b] += clipALF(clip[b], curr, pImgGauss[+0][+2], pImgGauss[-0][-2]);
        ELocal[31][b] += clipALF(clip[b], curr, pImgGauss[+0][+1], pImgGauss[-0][-1]);

        ELocal[32][b] += clipALF(clip[b], curr, pRecDbTmp[-1][+0], pRecDbTmp[+1][+0]);
        ELocal[33][b] += clipALF(clip[b], curr, pRecDbTmp[-0][-1], pRecDbTmp[+0][+1]);

        ELocal[34][b] +=
          clipALF(clip[b], curr, m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::FIRST][posDst.y][posDst.x]);
        ELocal[35][b] +=
          clipALF(clip[b], curr, m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::SECOND][posDst.y][posDst.x]);
        ELocal[36][b] += clipALF(clip[b], curr, pRecDbTmp[0][0]);
        ELocal[37][b] += clipALF(clip[b], 0, pResiTmp0[+0]);
        ELocal[38][b] += clipALF(clip[b], curr, pImgGauss[0][0]);
      }
      for (int b = 0; b < numBins; b++)
      {
        ELocal[shape.numCoeff - 1][b] += curr;
      }
    }
    else if (shape.filterType == ALF_FILTER_9_EXT_DB_RESI)
    {
      const FixFiltSetCand fixedFilterSetCandResi =
        fixedFilterSetCand == FixFiltSetCand::FIRST ? FixFiltSetCand::SECOND : FixFiltSetCand::FIRST;

      for (int b = 0; b < numBins; b++)
      {
        ELocal[28][b] += clipALF(clip[b], curr, pRecDbTmp[-1][+0], pRecDbTmp[+1][+0]);
        ELocal[29][b] += clipALF(clip[b], curr, pRecDbTmp[-0][-1], pRecDbTmp[+0][+1]);

        ELocal[30][b] +=
          clipALF(clip[b], curr, m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::FIRST][posDst.y][posDst.x]);
        ELocal[31][b] +=
          clipALF(clip[b], curr, m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::SECOND][posDst.y][posDst.x]);
        ELocal[32][b] += clipALF(clip[b], 0, m_fixFilterResiResult[fixedFilterSetCandResi][pos.y][pos.x]);
        ELocal[33][b] += clipALF(clip[b], curr, pRecDbTmp[0][+0]);
        ELocal[34][b] += clipALF(clip[b], 0, pResiTmp0[+0]);
        ELocal[35][b] += clipALF(clip[b], curr, pImgGauss[0][+0]);
      }
      for (int b = 0; b < numBins; b++)
      {
        ELocal[shape.numCoeff - 1][b] += curr;
      }
    }
  }
  else if (shape.filterType == ALF_FILTER_9)
  {
    if (transposeIdx == 0 || transposeIdx == 2)
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[20][b] += clipALF(
          clip[b], curr, m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::FIRST][posDst.y - 2][posDst.x],
          m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::FIRST][posDst.y + 2][posDst.x]);
        ELocal[21][b] += clipALF(
          clip[b], curr, m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::FIRST][posDst.y - 1][posDst.x],
          m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::FIRST][posDst.y + 1][posDst.x]);
        ELocal[22][b] += clipALF(
          clip[b], curr, m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::FIRST][posDst.y][posDst.x - 2],
          m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::FIRST][posDst.y][posDst.x + 2]);
        ELocal[23][b] += clipALF(
          clip[b], curr, m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::FIRST][posDst.y][posDst.x - 1],
          m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::FIRST][posDst.y][posDst.x + 1]);
        ELocal[24][b] +=
          clipALF(clip[b], curr, m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::FIRST][posDst.y][posDst.x]);
      }
    }
    else
    {
      for (int b = 0; b < numBins; b++)
      {
        ELocal[22][b] += clipALF(
          clip[b], curr, m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::FIRST][posDst.y - 2][posDst.x],
          m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::FIRST][posDst.y + 2][posDst.x]);
        ELocal[23][b] += clipALF(
          clip[b], curr, m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::FIRST][posDst.y - 1][posDst.x],
          m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::FIRST][posDst.y + 1][posDst.x]);
        ELocal[20][b] += clipALF(
          clip[b], curr, m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::FIRST][posDst.y][posDst.x - 2],
          m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::FIRST][posDst.y][posDst.x + 2]);
        ELocal[21][b] += clipALF(
          clip[b], curr, m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::FIRST][posDst.y][posDst.x - 1],
          m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::FIRST][posDst.y][posDst.x + 1]);
        ELocal[24][b] +=
          clipALF(clip[b], curr, m_fixFilterResult[compID][fixedFilterSetCand][FixFiltIdx::FIRST][posDst.y][posDst.x]);
      }
    }
    for (int b = 0; b < numBins; b++)
    {
      ELocal[shape.numCoeff - 1][b] += curr;
    }
  }
  else
  {
    for (int b = 0; b < numBins; b++)
    {
      ELocal[filterPattern[k]][b] += curr;
    }
  }
}

template<bool alfWSSD> void EncAdaptiveLoopFilterEcm::initDistortion(CodingStructure &cs)
{
  for (int comp = 0; comp < MAX_NUM_COMP; comp++)
  {
    m_unFiltDistCompnent[comp] = 0.0;

    const ChannelType chType = toChannelType(static_cast<CompID>(comp));
    for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
    {
      for (int shapeIdx = 0; shapeIdx < m_filterShapes[chType].size(); ++shapeIdx)
      {
        if (m_filterTypeTest[chType][m_filterShapes[chType][shapeIdx].filterType])
        {
          m_ctbDistortionUnfilter[comp][ctbIdx] =
            getUnfilteredDistortion(m_alfCovariance[comp][shapeIdx][ctbIdx][0][0], comp == 0 ? MAX_NUM_ALF_CLASSES : 1);

          m_unFiltDistCompnent[comp] += m_ctbDistortionUnfilter[comp][ctbIdx];
          break;
        }
      }
    }
  }

  const CPelBuf   orgBuf    = (m_encCfg->m_alfTrueOrg ? cs.getTrueOrgBuf() : cs.getOrgBuf()).Y();
  const ptrdiff_t orgStride = orgBuf.stride;
  for (int yPos = 0, ctbIdx = 0; yPos < m_picHeight; yPos += m_maxCUHeight)
  {
    for (int xPos = 0; xPos < m_picWidth; xPos += m_maxCUWidth, ++ctbIdx)
    {
      const int width  = std::min(m_picWidth - xPos, m_maxCUWidth);
      const int height = std::min(m_picHeight - yPos, m_maxCUHeight);
      for (int fixFiltIdxVal = 0; fixFiltIdxVal < NUM_FIXED_FILTERS; ++fixFiltIdxVal)
      {
        const auto fixedFilterIdx = static_cast<FixFiltIdx>(fixFiltIdxVal);

        for (int fixFiltSetCandIdxVal = 0; fixFiltSetCandIdxVal < NUM_FIXED_FILTER_SET_CANDS; ++fixFiltSetCandIdxVal)
        {
          const auto  fixFiltSetCandIdx    = static_cast<FixFiltSetCand>(fixFiltSetCandIdxVal);
          const auto &fixFiltResult        = m_fixFilterResult[COMP_Y][fixFiltSetCandIdx][fixedFilterIdx];
          auto       &ctbDistortionFixFilt = m_ctbDistortionFixedFilter[fixFiltIdxVal][fixFiltSetCandIdxVal][ctbIdx];

          ctbDistortionFixFilt = 0;
          const Pel *org       = orgBuf.bufAt(xPos, yPos);
          for (int y = 0; y < height; y++)
          {
            for (int x = 0; x < width; x++)
            {
              if (alfWSSD)
              {
                float weight = m_lumaLevelToWeightPLUT[org[x]];
                ctbDistortionFixFilt +=
                  weight * (org[x] - fixFiltResult[y + yPos][x + xPos]) * (org[x] - fixFiltResult[y + yPos][x + xPos]);
              }
              else
              {
                ctbDistortionFixFilt +=
                  (org[x] - fixFiltResult[y + yPos][x + xPos]) * (org[x] - fixFiltResult[y + yPos][x + xPos]);
              }
            }
            org += orgStride;
          }
        }
      }
    }
  }
}

void EncAdaptiveLoopFilterEcm::initDistortionCcalf(const CompID compId)
{
  for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
  {
    m_ctbDistortionUnfilter[compId][ctbIdx] = m_alfCovarianceCcAlf[0][ctbIdx].pixAcc;
  }
}

void EncAdaptiveLoopFilterEcm::getDistNewFilter(AlfParam &alfParam)
{
  reconstructLumaCoeff(alfParam, true);
  AlfFilterType  filterTypeCtb      = alfParam.filterType[ChannelType::LUMA];
  const unsigned numFixFiltSetCands = numFixedFilterSetCands(filterTypeCtb);

  for (int altIdx = 0; altIdx < alfParam.numAlternativesLuma; ++altIdx)
  {
    const int classifierIdx = m_classifierFinal[altIdx];
    for (int classIdx = 0; classIdx < ALF_NUM_CLASSES_CLASSIFIER[classifierIdx]; ++classIdx)
    {
      for (int coeff = 0; coeff < MAX_NUM_ALF_LUMA_COEFF; ++coeff)
      {
        m_filterTmp[coeff] = m_coeffFinal[altIdx][classIdx * MAX_NUM_ALF_LUMA_COEFF + coeff];
        m_clipTmp[coeff]   = m_clippFinal[altIdx][classIdx * MAX_NUM_ALF_LUMA_COEFF + coeff];
      }

      for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ++ctbIdx)
      {
        for (int fixedFilterSetCandIdx = 0; fixedFilterSetCandIdx < numFixFiltSetCands; ++fixedFilterSetCandIdx)
        {
          float               &dist = m_distCtbLumaNewFilt[altIdx][fixedFilterSetCandIdx][ctbIdx];
          const AlfCovariance &cov =
            m_alfCovariance[CompID::COMP_Y][m_filterTypeToStatIndex[ChannelType::LUMA][filterTypeCtb]][ctbIdx]
                           [fixedFilterSetCandIdx][classifierIdx][classIdx];

          if (classIdx == 0)
          {
            dist = m_ctbDistortionUnfilter[CompID::COMP_Y][ctbIdx];
          }
          dist += cov.calcErrorForCoeffs(
            m_clipTmp, m_filterTmp,
            m_filterShapes[ChannelType::LUMA][m_filterTypeToStatIndex[ChannelType::LUMA][filterTypeCtb]].numCoeff,
            ALF_SCALE_FACTOR[m_scaleIdxFinal[altIdx][classIdx]], COEFF_SCALE_BITS_LUMA + ALF_SCALE_SHIFT);
        }
      }
    }
  }
}

void EncAdaptiveLoopFilterEcm::getDistApsFilter(const CodingStructure &cs, const AlfApsList &apsIds)
{
  for (int apsIdx = 0; apsIdx < apsIds.size(); apsIdx++)
  {
    const APS *curAPS      = cs.slice->m_alfApss[apsIds[apsIdx]];
    AlfParam   alfParamTmp = curAPS->m_alfAPSParam.getEcmParam();

    reconstructLumaCoeff(alfParamTmp, true);

    const AlfFilterType filterTypeCtb      = alfParamTmp.filterType[ChannelType::LUMA];
    const unsigned      numFixFiltSetCands = numFixedFilterSetCands(filterTypeCtb);
    m_filterTypeApsLuma[apsIdx]            = filterTypeCtb;
    m_numLumaAltAps[apsIdx]                = alfParamTmp.numAlternativesLuma;
    for (int altIdx = 0; altIdx < alfParamTmp.numAlternativesLuma; ++altIdx)
    {
      const int classifierIdx                = m_classifierFinal[altIdx];
      m_classifierIdxApsLuma[apsIdx][altIdx] = classifierIdx;
      for (int classIdx = 0; classIdx < ALF_NUM_CLASSES_CLASSIFIER[classifierIdx]; ++classIdx)
      {
        for (int coeff = 0; coeff < MAX_NUM_ALF_LUMA_COEFF; ++coeff)
        {
          m_filterTmp[coeff] = m_coeffFinal[altIdx][classIdx * MAX_NUM_ALF_LUMA_COEFF + coeff];
          m_clipTmp[coeff]   = m_clippFinal[altIdx][classIdx * MAX_NUM_ALF_LUMA_COEFF + coeff];
        }

        for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ++ctbIdx)
        {
          for (int fixedFilterSetCandIdx = 0; fixedFilterSetCandIdx < numFixFiltSetCands; ++fixedFilterSetCandIdx)
          {
            float               &dist = m_distCtbApsLuma[apsIdx][altIdx][fixedFilterSetCandIdx][ctbIdx];
            const AlfCovariance &cov =
              m_alfCovariance[CompID::COMP_Y][m_filterTypeToStatIndex[ChannelType::LUMA][filterTypeCtb]][ctbIdx]
                             [fixedFilterSetCandIdx][classifierIdx][classIdx];

            if (classIdx == 0)
            {
              dist = m_ctbDistortionUnfilter[CompID::COMP_Y][ctbIdx];
            }
            dist += cov.calcErrorForCoeffs(
              m_clipTmp, m_filterTmp,
              m_filterShapes[ChannelType::LUMA][m_filterTypeToStatIndex[ChannelType::LUMA][filterTypeCtb]].numCoeff,
              ALF_SCALE_FACTOR[m_scaleIdxFinal[altIdx][classIdx]], COEFF_SCALE_BITS_LUMA + ALF_SCALE_SHIFT);
          }
        }
      }
    }
  }
}

void EncAdaptiveLoopFilterEcm::alfEncoderCtb(CodingStructure &cs, AlfParam &alfParamNewFilters
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
  setCtuEnableFlag(m_modes, ChannelType::LUMA, CTB_MODE_LUMA_APS0_ALT0);
  if (m_alfParamTemp.numAlternativesLuma < 1)
  {
    m_alfParamTemp.numAlternativesLuma = 1;
  }
  float costOff = m_unFiltDistCompnent[CompID::COMP_Y];
  setCtuEnableFlag(m_modes, ChannelType::LUMA, CTB_MODE_OFF);
  const int         newApsId = getAvailableApsIdsLuma(cs);
  const AlfApsList &apsIds   = cs.slice->m_alfApsIdsLuma;
  AlfApsList        bestApsIds;
  float             costMin = MAX_FLOAT;

  getDistApsFilter(cs, apsIds);

  AlfFilterType filterTypeNewFilter       = ALF_NUM_OF_FILTER_TYPES;
  int           numAltLumaNew             = alfParamNewFilters.numAlternativesLuma;
  int           bestFixedFilterSetCandIdx = -1;
  bool          filterWasRederived        = false;
  for (int withSA = 0; withSA < 2; ++withSA)
  {
    if (withSA && !(alfParamNewFiltersBest.newFilterFlag[ChannelType::LUMA] && filterWasRederived))
    {
      continue;
    }
    int fixedFilterSetCandIdxFrom = 0, fixedFilterSetCandIdxTo = NUM_FIXED_FILTER_SET_CANDS;
    if (withSA)
    {
      fixedFilterSetCandIdxFrom = bestFixedFilterSetCandIdx, fixedFilterSetCandIdxTo = bestFixedFilterSetCandIdx + 1;
    }
    for (int fixedFilterSetCandIdx = fixedFilterSetCandIdxFrom; fixedFilterSetCandIdx < fixedFilterSetCandIdxTo;
         ++fixedFilterSetCandIdx)
    {
      int useNewFilterFrom = 0, useNewFilterTo = 2;
      if (withSA)
      {
        useNewFilterFrom = 1, useNewFilterTo = 2;
      }
      for (int useNewFilter = useNewFilterFrom; useNewFilter < useNewFilterTo; ++useNewFilter)
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
          filterTypeNewFilter = alfParamNewFilters.filterType[ChannelType::LUMA];
          numAltLumaNew       = alfParamNewFilters.numAlternativesLuma;
        }

        std::unordered_set<int> triedBlocksUsingNewFilter;

        int numTemporalApsFrom = 0, numTemporalApsTo = (int)apsIds.size();
        if (withSA)
        {
          numTemporalApsFrom = (int)bestApsIds.size() - 1, numTemporalApsTo = (int)bestApsIds.size() - 1;
        }
        for (int numTemporalAps = numTemporalApsFrom; numTemporalAps <= numTemporalApsTo; numTemporalAps++)
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

          const int numFilterSet = NUM_FIXED_FILTERS + numApss;

          if (numTemporalAps == apsIds.size() && numTemporalAps > 0 && useNewFilter && newApsId == apsIds.back())
          {
            // last temporalAPS is occupied by new filter set and this temporal APS becomes unavailable
            continue;
          }

          int iterFrom = 0, iterTo = useNewFilter ? 2 : 1;
          if (withSA)
          {
            // here, useNewFilter = true
            iterFrom = 1, iterTo = 2;
          }
          for (int iter = iterFrom; iter < iterTo; iter++)
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
                  const unsigned altIdx        = LumaCtbModeHandler::getAlternative(ctbMode);
                  const int      classifierIdx = m_classifierFinal[altIdx];
                  for (int classIdx = 0; classIdx < ALF_NUM_CLASSES_CLASSIFIER[classifierIdx]; classIdx++)
                  {
                    short *pCoeff = m_coeffFinal[altIdx];
                    Pel   *pClipp = m_clippFinal[altIdx];
                    for (int i = 0; i < MAX_NUM_ALF_LUMA_COEFF; i++)
                    {
                      m_filterTmp[i] = pCoeff[classIdx * MAX_NUM_ALF_LUMA_COEFF + i];
                      m_clipTmp[i]   = pClipp[classIdx * MAX_NUM_ALF_LUMA_COEFF + i];
                    }

                    const AlfCovariance &cov =
                      m_alfCovariance[CompID::COMP_Y][m_filterTypeToStatIndex[ChannelType::LUMA][filterTypeNewFilter]]
                                     [ctbIdx][fixedFilterSetCandIdx][classifierIdx][classIdx];
                    dDistOrgNewFilter += cov.calcErrorForCoeffs(
                      m_clipTmp, m_filterTmp,
                      m_filterShapes[ChannelType::LUMA][m_filterTypeToStatIndex[ChannelType::LUMA][filterTypeNewFilter]]
                        .numCoeff,
                      ALF_SCALE_FACTOR[m_scaleIdxFinal[altIdx][classIdx]], COEFF_SCALE_BITS_LUMA + ALF_SCALE_SHIFT);
                  }
                }
                else if (CtbModeHandler::isEnabled(ctbMode))
                {
                  CtbModeHandler::setDisabled(ctbMode);
                }
              }

              if (blocksUsingNewFilter > 0 && blocksUsingNewFilter < m_numCTUsInPic &&
                  triedBlocksUsingNewFilter.count(blocksUsingNewFilter) ==
                    0 /* re-derived filter for this number of blocks was tried before */)
              {
                int   bitsNewFilterTempLuma = 0, bitsTemp = 0;
                float err           = 0.0;
                float costNewFilter = MAX_FLOAT;
                int   shapeIdxFrom = 0, shapeIdxTo = (int)m_filterShapes[ChannelType::LUMA].size();
                if (withSA)
                {
                  for (int shapeIdx = 0; shapeIdx < m_filterShapes[ChannelType::LUMA].size(); shapeIdx++)
                  {
                    if (m_filterShapes[ChannelType::LUMA][shapeIdx].filterType ==
                        alfParamNewFiltersBest.filterType[ChannelType::LUMA])
                    {
                      shapeIdxFrom = shapeIdx, shapeIdxTo = shapeIdx + 1;
                      break;
                    }
                  }
                }
                for (int shapeIdx = shapeIdxFrom; shapeIdx < shapeIdxTo; shapeIdx++)
                {
                  if (m_filterTypeTest[ChannelType::LUMA][m_filterShapes[ChannelType::LUMA][shapeIdx].filterType] ==
                      false)
                  {
                    continue;
                  }
                  m_alfParamTemp.filterType[ChannelType::LUMA] = m_filterShapes[ChannelType::LUMA][shapeIdx].filterType;
                  err = getFilterCoeffAndCost(cs, 0, ChannelType::LUMA, true, shapeIdx, bitsTemp, fixedFilterSetCandIdx,
                                              withSA, true);
                  if (err < costNewFilter)
                  {
                    costNewFilter         = err;
                    bitsNewFilterTempLuma = bitsTemp;
                    m_alfParamTempNL      = m_alfParamTemp;
                  }
                }
                err            = costNewFilter;
                m_alfParamTemp = m_alfParamTempNL;

                triedBlocksUsingNewFilter.insert(blocksUsingNewFilter);
                if (dDistOrgNewFilter + m_lambda[COMP_Y] * m_bitsNewFilter[ChannelType::LUMA] < err)
                {
                  // re-derived filter is not good, skip
                  continue;
                }
                filterWasRederived = true;

                getDistNewFilter(m_alfParamTemp);
                bitsNewFilter       = bitsNewFilterTempLuma;
                filterTypeNewFilter = m_alfParamTemp.filterType[ChannelType::LUMA];
                numAltLumaNew       = m_alfParamTemp.numAlternativesLuma;
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

              const int firstFilterSetIdx = m_encCfg->m_alfAllowPredefinedFilters ? 0 : NUM_FIXED_FILTERS;

              for (int filterSetIdx = firstFilterSetIdx; filterSetIdx < numFilterSet; filterSetIdx++)
              {
                // Select best filter set index inside one APS
                const unsigned iterAltLuma = (filterSetIdx < NUM_FIXED_FILTERS)
                  ? 1
                  : (useNewFilter
                       ? ((filterSetIdx == NUM_FIXED_FILTERS) ? numAltLumaNew
                                                              : m_numLumaAltAps[filterSetIdx - 1 - NUM_FIXED_FILTERS])
                       : m_numLumaAltAps[filterSetIdx - NUM_FIXED_FILTERS]);
                for (unsigned altIdx = 0; altIdx < iterAltLuma; ++altIdx)
                {
                  if (filterSetIdx < NUM_FIXED_FILTERS)
                  {
                    LumaCtbModeHandler::setFixedFilter(currCtbMode, filterSetIdx);
                  }
                  else
                  {
                    LumaCtbModeHandler::setApsFilter(currCtbMode, filterSetIdx - NUM_FIXED_FILTERS, altIdx);
                  }

                  // rate
                  m_CABACEstimator->getCtx() = AlfCtx(ctxTempStart);
                  m_CABACEstimator->resetBits();
                  m_CABACEstimator->codeAlfCtuEnableFlag(cs, ctbIdx, COMP_Y, &m_alfParamTemp);
                  m_CABACEstimator->codeAlfCtuFilterIndex(cs, ctbIdx, m_alfParamTemp.enabledFlag[COMP_Y]);
                  if (filterSetIdx >= NUM_FIXED_FILTERS)
                  {
                    m_CABACEstimator->codeAlfCtuLumaAlternative(cs, ctbIdx, iterAltLuma);
                  }
                  float rateOn = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();

                  // distortion
                  float dist;
                  if (filterSetIdx < NUM_FIXED_FILTERS)
                  {
                    dist = m_ctbDistortionFixedFilter[filterSetIdx][fixedFilterSetCandIdx][ctbIdx];
                  }
                  else if (useNewFilter && filterSetIdx == NUM_FIXED_FILTERS)
                  {
                    dist = m_distCtbLumaNewFilt[altIdx][fixedFilterSetCandIdx][ctbIdx];
                  }
                  else
                  {
                    dist = m_distCtbApsLuma[filterSetIdx - NUM_FIXED_FILTERS - (useNewFilter ? 1 : 0)][altIdx]
                                           [fixedFilterSetCandIdx][ctbIdx];
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

            int tmpBits = bitsNewFilter + 3 * (numFilterSet - NUM_FIXED_FILTERS) + 1;
            curCost += tmpBits * m_lambda[COMP_Y];
            if (curCost < costMin)
            {
              costMin = curCost;
              bestApsIds.resize(numFilterSet - NUM_FIXED_FILTERS);
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
              bestFixedFilterSetCandIdx = fixedFilterSetCandIdx;
              alfParamNewFiltersBest    = m_alfParamTemp;
              ctxBest                   = AlfCtx(m_CABACEstimator->getCtx());
              copyIndices(m_indexTmp, m_modes, ChannelType::LUMA);
              alfParamNewFiltersBest.newFilterFlag[ChannelType::LUMA] = useNewFilter;
            }
          }   // for (int iter = 0; iter < numIter; iter++)
        }   // for (int numTemporalAps = 0; numTemporalAps < apsIds.size(); numTemporalAps++)
      }   // for (int useNewFilter = 0; useNewFilter <= 1; useNewFilter++)
    }   // for (fixedFilterSetCandIdx)
  }   // for (withSA)

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
    cs.slice->m_alfEnabledFlag[COMP_Y]          = true;
    cs.slice->m_numAlfApsIdsLuma                = (int)bestApsIds.size();
    cs.slice->m_alfApsIdsLuma                   = bestApsIds;
    cs.slice->m_newAlfFixFiltSetCandIdx[COMP_Y] = bestFixedFilterSetCandIdx;

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
      AlfParam &alfAPSParam = newAPS->m_alfAPSParam.getEcmParam();

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

      APS *curAPS = m_apsMap->getPS(curApsId);

      for (int fixedFilterSetCandIdx = 0; fixedFilterSetCandIdx < NUM_FIXED_FILTER_SET_CANDS; ++fixedFilterSetCandIdx)
      {
        float curCost = m_lambda[COMP_Cb] * 3;
        if (!reuseExistingAPS)
        {
          m_alfParamTemp = alfParamNewFilters;
          curCost += m_lambda[COMP_Cb] * m_bitsNewFilter[ChannelType::CHROMA];
        }
        else if (curAPS && curAPS->m_temporalId <= cs.slice->m_uiTLayer &&
                 curAPS->m_layerId == cs.slice->m_pic->m_layerId)
        {
          const AlfParam &alfAPSParam = curAPS->m_alfAPSParam.getEcmParam();
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
        AlfFilterType filterTypeChroma = m_alfParamTemp.filterType[ChannelType::CHROMA];
        m_CABACEstimator->getCtx()     = AlfCtx(ctxStart);
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

              const AlfCovariance &cov =
                m_alfCovariance[compId][m_filterTypeToStatIndex[ChannelType::CHROMA][filterTypeChroma]][ctbIdx]
                               [fixedFilterSetCandIdx][0][0];

              float altDist = cov.calcErrorForCoeffs(
                m_clipTmp, m_filterTmp,
                m_filterShapes[ChannelType::CHROMA][m_filterTypeToStatIndex[ChannelType::CHROMA][filterTypeChroma]]
                  .numCoeff,
                ALF_SCALE_FACTOR[m_chromaScaleIdxFinal[altIdx][0]], COEFF_SCALE_BITS_CHROMA + ALF_SCALE_SHIFT);
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
          costMin                                      = curCost;
          cs.slice->m_alfApsIdChroma                   = curApsId;
          cs.slice->m_alfEnabledFlag[COMP_Cb]          = m_alfParamTemp.enabledFlag[COMP_Cb];
          cs.slice->m_alfEnabledFlag[COMP_Cr]          = m_alfParamTemp.enabledFlag[COMP_Cr];
          cs.slice->m_newAlfFixFiltSetCandIdx[COMP_Cb] = fixedFilterSetCandIdx;
          cs.slice->m_newAlfFixFiltSetCandIdx[COMP_Cr] = fixedFilterSetCandIdx;
          copyIndices(m_indexTmp, m_modes, ChannelType::CHROMA);
        }
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
          newAPS->m_alfAPSParam.getEcmParam().reset();
        }

        AlfParam &alfAPSParam = newAPS->m_alfAPSParam.getEcmParam();

        alfAPSParam.newFilterFlag[ChannelType::CHROMA] = true;
        if (!alfParamNewFiltersBest.newFilterFlag[ChannelType::LUMA])
        {
          alfAPSParam.newFilterFlag[ChannelType::LUMA] = false;
        }
        alfAPSParam.numAlternativesChroma = alfParamNewFilters.numAlternativesChroma;
        for (int altIdx = 0; altIdx < ALF_MAX_NUM_ALTERNATIVES_CHROMA; ++altIdx)
        {
          alfAPSParam.chromaNonLinearFlag[altIdx] = alfParamNewFilters.chromaNonLinearFlag[altIdx];
        }
        alfAPSParam.filterType[ChannelType::CHROMA] = alfParamNewFilters.filterType[ChannelType::CHROMA];
        newAPS->m_temporalId                        = cs.slice->m_uiTLayer;
        for (int altIdx = 0; altIdx < ALF_MAX_NUM_ALTERNATIVES_CHROMA; ++altIdx)
        {
          alfAPSParam.chromaScaleIdx[altIdx][0] = alfParamNewFilters.chromaScaleIdx[altIdx][0];
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

void EncAdaptiveLoopFilterEcm::alfReconstructor(CodingStructure &cs, const PelUnitBuf &recExtBuf)
{
  if (!cs.slice->m_alfEnabledFlag[COMP_Y])
  {
    return;
  }
  reconstructCoeffAPSs(cs, true, cs.slice->m_alfEnabledFlag[COMP_Cb] || cs.slice->m_alfEnabledFlag[COMP_Cr], false);
  PelUnitBuf          &recBuf = cs.getRecoBufRef();
  const PreCalcValues &pcv    = *cs.pcv;

  PelBuf tmpYBeforeDb = m_tempBufBeforeDb.getBuf(COMP_Y).subBuf(cs.area.Y(), cs.area.Y());
  PelBuf tmpYResi     = m_tempBufResi.subBuf(cs.area.Y(), cs.area.Y());

  const auto getFixedFilterSetCand = [](const Slice &slice, const CompID compID) -> std::tuple<int, FixFiltSetCand>
  {
    const int fixedFilterSetCandIdxVal = slice.m_newAlfFixFiltSetCandIdx[compID];
    CHECK(fixedFilterSetCandIdxVal < 0 || fixedFilterSetCandIdxVal >= NUM_FIXED_FILTER_SET_CANDS,
          "Fixed filter set candidate index is out of range.");
    return std::make_tuple(fixedFilterSetCandIdxVal, static_cast<FixFiltSetCand>(fixedFilterSetCandIdxVal));
  };

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
        buf.addMirrorExtension(MAX_ALF_PADDING_SIZE, false);
        buf =
          buf.subBuf(UnitArea(cs.area.chromaFormat,
                              Area(clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, width, height)));

        if (CtbModeHandler::isEnabled((*m_modes)[COMP_Y][ctuIdx]))
        {
          const Area blkSrc(0, 0, width, height);
          const Area blkDst(xPos, yPos, width, height);
          const int  m                                                 = (*m_modes)[COMP_Y][ctuIdx];
          const auto [fixedFilterSetCandIdxVal, fixedFilterSetCandIdx] = getFixedFilterSetCand(*cs.slice, COMP_Y);
          if (LumaCtbModeHandler::isFixedFilter(m))
          {
            copyFixedFilterResults(recBuf, blkDst, CompID::COMP_Y, fixedFilterSetCandIdx,
                                   LumaCtbModeHandler::getFixedFilterIdx(m) == 0 ? FixFiltIdx::FIRST
                                                                                 : FixFiltIdx::SECOND);
          }
          else
          {
            const AlfFilterType   filterTypeCtb = m_filterTypeApsLuma[LumaCtbModeHandler::getApsIdx(m)];
            const ClassifierIndex classifierIdx = getClassifierIdxEnum(
              m_classifierIdxApsLuma[LumaCtbModeHandler::getApsIdx(m)][LumaCtbModeHandler::getAlternative(m)]);

            copyAndExtendFixedFilterResultsCtu(fixedFilterSetCandIdx, FixFiltIdx::SECOND, blkDst);
            copyAndExtendGaussResultsCtu(blkDst);

            // Copy DBF input for the current block.
            PelBuf bufDb = m_tempBufBeforeDbCtu.getBuf(COMP_Y).subBuf(0, 0, wBuf, hBuf);
            bufDb.copyFrom(m_tempBufBeforeDb.getBuf(COMP_Y).subBuf(xPos - (clipL ? 0 : NUM_DB_PAD),
                                                                   yPos - (clipT ? 0 : NUM_DB_PAD), wBuf, hBuf));
            // pad top-left unavailable samples for raster slice
            if (rasterSliceAlfPad & 1)
            {
              bufDb.padBorderPel(NUM_DB_PAD, NUM_DB_PAD, 1);
            }
            // pad bottom-right unavailable samples for raster slice
            if (rasterSliceAlfPad & 2)
            {
              bufDb.padBorderPel(NUM_DB_PAD, NUM_DB_PAD, 2);
            }
            bufDb.addMirrorExtension(NUM_DB_PAD);
            bufDb = bufDb.subBuf(clipL ? 0 : NUM_DB_PAD, clipT ? 0 : NUM_DB_PAD, width, height);

            // Copy residual for the current block.
            PelBuf bufResi = m_tempBufResiCtu.subBuf(0, 0, wBuf, hBuf);
            bufResi.copyFrom(
              m_tempBufResi.subBuf(xPos - (clipL ? 0 : NUM_RESI_PAD), yPos - (clipT ? 0 : NUM_RESI_PAD), wBuf, hBuf));
            // pad top-left unavailable samples for raster slice
            if (rasterSliceAlfPad & 1)
            {
              bufResi.padBorderPel(NUM_RESI_PAD, NUM_RESI_PAD, 1);
            }
            // pad bottom-right unavailable samples for raster slice
            if (rasterSliceAlfPad & 2)
            {
              bufResi.padBorderPel(NUM_RESI_PAD, NUM_RESI_PAD, 2);
            }
            bufResi.addMirrorExtension(NUM_RESI_PAD);
            bufResi = bufResi.subBuf(clipL ? 0 : NUM_RESI_PAD, clipT ? 0 : NUM_RESI_PAD, width, height);

            alfFiltering(m_classifier[classifierIdx], recBuf, bufDb, bufResi, buf, blkDst, blkSrc, CompID::COMP_Y,
                         getScaleIdxVals(m), getCoeffVals(m), getClipVals(m), filterTypeCtb, true,
                         fixedFilterSetCandIdx);
          }
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
            const auto [fixedFilterSetCandIdxVal, fixedFilterSetCandIdx] = getFixedFilterSetCand(*cs.slice, compID);
            alfFiltering(m_classifier[ClassifierIndex::ADAPT_FILTER_GRAD_BASED], recBuf,
                         tmpYBeforeDb,   // XXX: Luma buffer. Must not be used by the filter.
                         tmpYResi,   // XXX: Luma buffer. Must not be used by the filter.
                         buf, blkDst, blkSrc, compID, m_chromaScaleIdxFinal[altNum], m_chromaCoeffFinal[altNum],
                         m_chromaClippFinal[altNum], m_filterTypeApsChroma, false, fixedFilterSetCandIdx);
          }
        }
      }
      else
      {
        const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));
        if (CtbModeHandler::isEnabled((*m_modes)[COMP_Y][ctuIdx]))
        {
          Area      blk(xPos, yPos, width, height);
          const int m                                                  = (*m_modes)[COMP_Y][ctuIdx];
          const auto [fixedFilterSetCandIdxVal, fixedFilterSetCandIdx] = getFixedFilterSetCand(*cs.slice, COMP_Y);
          if (LumaCtbModeHandler::isFixedFilter(m))
          {
            copyFixedFilterResults(recBuf, blk, CompID::COMP_Y, fixedFilterSetCandIdx,
                                   LumaCtbModeHandler::getFixedFilterIdx(m) == 0 ? FixFiltIdx::FIRST
                                                                                 : FixFiltIdx::SECOND);
          }
          else
          {
            const bool isFixFiltPaddedPerCtu = isFixedFilterPaddedPerCtu(*cs.slice);
            if (isFixFiltPaddedPerCtu)
            {
              copyAndExtendFixedFilterResultsCtu(fixedFilterSetCandIdx, FixFiltIdx::SECOND, blk);
              copyAndExtendGaussResultsCtu(blk);
            }

            const AlfFilterType   filterTypeCtb = m_filterTypeApsLuma[LumaCtbModeHandler::getApsIdx(m)];
            const ClassifierIndex classifierIdx = getClassifierIdxEnum(
              m_classifierIdxApsLuma[LumaCtbModeHandler::getApsIdx(m)][LumaCtbModeHandler::getAlternative(m)]);

            alfFiltering(m_classifier[classifierIdx], recBuf, tmpYBeforeDb, tmpYResi, recExtBuf, blk, blk,
                         CompID::COMP_Y, getScaleIdxVals(m), getCoeffVals(m), getClipVals(m), filterTypeCtb,
                         isFixFiltPaddedPerCtu, fixedFilterSetCandIdx);
          }
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
            const auto [fixedFilterSetCandIdxVal, fixedFilterSetCandIdx] = getFixedFilterSetCand(*cs.slice, compID);
            alfFiltering(m_classifier[ClassifierIndex::ADAPT_FILTER_GRAD_BASED], recBuf,
                         tmpYBeforeDb,   // XXX: Luma buffer. Must not be used by the filter.
                         tmpYResi,   // XXX: Luma buffer. Must not be used by the filter.
                         recExtBuf, blk, blk, compID, m_chromaScaleIdxFinal[altIdx], m_chromaCoeffFinal[altIdx],
                         m_chromaClippFinal[altIdx], m_filterTypeApsChroma, false, fixedFilterSetCandIdx);
          }
        }
      }
      ctuIdx++;
    }
  }
}

void EncAdaptiveLoopFilterEcm::initCtuAlternativeLuma(CtuModes &ctuModes) const
{
  CtbModes &ctbModes = ctuModes[CompID::COMP_Y];
  CHECKD(ctbModes.size() != m_numCTUsInPic, "Unexpected number of entries in CTB modes.");

  int altIdx = 0;
  for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ++ctuIdx)
  {
    int &ctbMode = ctbModes[ctuIdx];
    if (LumaCtbModeHandler::isApsFilter(ctbMode))
    {
      LumaCtbModeHandler::setAlternative(ctbMode, altIdx);
    }
    else
    {
      LumaCtbModeHandler::setApsFilter(ctbMode, 0, altIdx);
    }
    if ((ctuIdx + 1) * m_alfParamTemp.numAlternativesLuma >= (altIdx + 1) * m_numCTUsInPic)
    {
      ++altIdx;
    }
  }
}

void EncAdaptiveLoopFilterEcm::initCtuAlternativeChroma(CtuModes &ctuModes) const
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

void EncAdaptiveLoopFilterEcm::deriveCcAlfFilterCoeff(
  CompID compID, const PelUnitBuf &recYuv, const PelUnitBuf &recYuvExt,
  short filterCoeff[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF], const uint8_t filterIdx)
{
  EncAdaptiveLoopFilterBase<AlfParametersEcm>::deriveCcAlfFilterCoeff(
    compID, recYuv, recYuvExt, filterCoeff, filterIdx, m_alfCovarianceFrameCcAlf[0], m_filterShapesCcAlf[0].numCoeff);
}

void EncAdaptiveLoopFilterEcm::getFrameStatsCcalf(CompID compIdx, int filterIdc)
{
  EncAdaptiveLoopFilterBase<AlfParametersEcm>::getFrameStatsCcalf(
    m_alfCovarianceFrameCcAlf, m_alfCovarianceCcAlf, compIdx, filterIdc, static_cast<int>(m_filterShapesCcAlf.size()));
}

void EncAdaptiveLoopFilterEcm::deriveCcAlfFilter(CodingStructure &cs, CompID compID, const PelUnitBuf &orgYuv,
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

  for (int testFilterIdx = 0; testFilterIdx < apsIds.size() + MAX_NUM_CC_ALF_FILTERS; ++testFilterIdx)
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
        const CcAlfFilterParam &ccAlfAPSParam = m_apsMap->getPS(apsIds[testFilterIdx])->m_ccAlfAPSParam.getEcmParam();

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
void EncAdaptiveLoopFilterEcm::deriveStatsForCcAlfFiltering(const PelUnitBuf &orgYuv, const PelUnitBuf &recYuv,
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

  PelUnitBuf           recYuvSao = m_tempBufSao.getBuf(UnitArea(cs.area.chromaFormat, Area(cs.area.blocks[COMP_Y])));
  const PreCalcValues &pcv       = *cs.pcv;
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
        recBuf.addMirrorExtension(MAX_ALF_PADDING_SIZE, false);
        recBuf = recBuf.subBuf(
          UnitArea(cs.area.chromaFormat,
                   Area(clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, width, height)));

        // TODO: don't copy/mirror luma since it's not used
        PelUnitBuf recBufSao = m_tempBufSaoCtu.subBuf(UnitArea(cs.area.chromaFormat, Area(0, 0, wBuf, hBuf)));
        recBufSao.copyFrom(recYuvSao.subBuf(UnitArea(
          cs.area.chromaFormat,
          Area(xPos - (clipL ? 0 : MAX_ALF_PADDING_SIZE), yPos - (clipT ? 0 : MAX_ALF_PADDING_SIZE), wBuf, hBuf))));
        // pad top-left unavailable samples for raster slice
        if (rasterSliceAlfPad & 1)
        {
          recBufSao.padBorderPel(MAX_ALF_PADDING_SIZE, 1);
        }
        // pad bottom-right unavailable samples for raster slice
        if (rasterSliceAlfPad & 2)
        {
          recBufSao.padBorderPel(MAX_ALF_PADDING_SIZE, 2);
        }
        recBufSao.addMirrorExtension(MAX_ALF_PADDING_SIZE);
        recBufSao = recBufSao.subBuf(
          UnitArea(cs.area.chromaFormat,
                   Area(clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, width, height)));

        const UnitArea area(m_chromaFormat, Area(0, 0, width, height));
        const UnitArea areaDst(m_chromaFormat, Area(xPos, yPos, width, height));

        const CompID compID = CompID(compIdx);

        for (int shape = 0; shape != m_filterShapesCcAlf.size(); ++shape)
        {
          getBlkStatsCcAlf<alfWSSD>(m_alfCovarianceCcAlf[0][ctuRsAddr], m_filterShapesCcAlf[shape], orgYuv, recBuf,
                                    recBufSao, areaDst, area, compID, yPos);
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
                                    recYuvSao, area, area, compID, yPos);
          m_alfCovarianceFrameCcAlf[shape] += m_alfCovarianceCcAlf[shape][ctuRsAddr];
        }
      }
    }
  }
}

template<bool alfWSSD>
void EncAdaptiveLoopFilterEcm::getBlkStatsCcAlf(AlfCovariance &alfCovariance, const AlfFilterShape &shape,
                                                const PelUnitBuf &orgYuv, const PelUnitBuf &recYuv,
                                                const PelUnitBuf &recYuvSao, const UnitArea &areaDst,
                                                const UnitArea &area, const CompID compID, const int yPos)
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

  const ptrdiff_t orgStride = orgYuv.get(compID).stride;
  const Pel      *org       = orgYuv.get(compID).bufAt(compArea);

  const ptrdiff_t orgLumaStride = orgYuv.get(COMP_Y).stride;
  const Pel      *orgLuma       = orgYuv.get(COMP_Y).bufAt(areaDst.block(COMP_Y));

  const ptrdiff_t recSaoStride = recYuvSao.get(compID).stride;
  const Pel      *recSaoPtr    = recYuvSao.get(compID).bufAt(area.chromaPos());

  Pel ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][1];

  const float strength    = m_encCfg->m_ccalfStrengthTarget;
  const float invStrength = strength != 0.0 ? 1.0 / strength : 0.0;

  for (int i = 0; i < compArea.height; i++)
  {
    const int iY = i << getComponentScaleY(compID, m_chromaFormat);

    for (int j = 0; j < compArea.width; ++j)
    {
      const int jY = j << getComponentScaleX(compID, m_chromaFormat);

      std::memset(ELocal, 0, sizeof(ELocal));

      calcCovarianceCcAlf(ELocal, rec[COMP_Y] + jY, recStride[COMP_Y], recSaoPtr + j, recSaoStride, shape);

      const Pel *lumaPtr = orgLuma + iY * orgLumaStride + jY;

      const float weight = alfWSSD ? m_lumaLevelToWeightPLUT[*lumaPtr] : 1.0;
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
    recSaoPtr += recSaoStride;
  }
}

void EncAdaptiveLoopFilterEcm::calcCovarianceCcAlf(Pel ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][1], const Pel *rec,
                                                   const ptrdiff_t stride, const Pel *recSao, const ptrdiff_t saoStride,
                                                   const AlfFilterShape &shape)
{
  CHECK(shape.filterType != CC_ALF, "Bad CC ALF shape");

  const Pel *recYM1 = rec - 1 * stride;
  const Pel *recY0  = rec;
  const Pel *recYP1 = rec + 1 * stride;
  const Pel *recYP2 = rec + 2 * stride;
  const Pel *recYM2 = rec - 2 * stride;
  const Pel *recYM3 = rec - 3 * stride;
  const Pel *recYP3 = rec + 3 * stride;
  const Pel *recYM4 = rec - 4 * stride;
  const Pel *recYP4 = rec + 4 * stride;

  for (int b = 0; b < 1; b++)
  {
    const Pel centerValue    = recY0[+0];
    const Pel centerValueSao = recSao[+0];

    ELocal[0][b] += recYM4[+0] - centerValue;
    ELocal[1][b] += recYM3[+0] - centerValue;
    ELocal[2][b] += recYM2[+0] - centerValue;
    ELocal[3][b] += recYM1[+0] - centerValue;

    ELocal[4][b] += recY0[-4] - centerValue;
    ELocal[5][b] += recY0[-3] - centerValue;
    ELocal[6][b] += recY0[-2] - centerValue;
    ELocal[7][b] += recY0[-1] - centerValue;
    ELocal[8][b] += recY0[+1] - centerValue;
    ELocal[9][b] += recY0[+2] - centerValue;
    ELocal[10][b] += recY0[+3] - centerValue;
    ELocal[11][b] += recY0[+4] - centerValue;

    ELocal[12][b] += recYP1[-4] - centerValue;
    ELocal[13][b] += recYP1[-3] - centerValue;
    ELocal[14][b] += recYP1[-2] - centerValue;
    ELocal[15][b] += recYP1[-1] - centerValue;
    ELocal[16][b] += recYP1[+0] - centerValue;
    ELocal[17][b] += recYP1[+1] - centerValue;
    ELocal[18][b] += recYP1[+2] - centerValue;
    ELocal[19][b] += recYP1[+3] - centerValue;
    ELocal[20][b] += recYP1[+4] - centerValue;

    ELocal[21][b] += recYP2[+0] - centerValue;
    ELocal[22][b] += recYP3[+0] - centerValue;
    ELocal[23][b] += recYP4[+0] - centerValue;

    ELocal[24][b] += (recSao[-1 * saoStride + 0] - centerValueSao);
    ELocal[25][b] += (recSao[+0 * saoStride - 1] - centerValueSao);
    ELocal[26][b] += (recSao[+0 * saoStride + 1] - centerValueSao);
    ELocal[27][b] += (recSao[+1 * saoStride + 0] - centerValueSao);
  }
}

void EncAdaptiveLoopFilterEcm::countLumaSwingGreaterThanThreshold(const Pel *luma, ptrdiff_t lumaStride, int height,
                                                                  int width, int log2BlockWidth, int log2BlockHeight,
                                                                  uint64_t *lumaSwingGreaterThanThresholdCount,
                                                                  int       lumaCountStride)
{
  const int lumaBitDepth = m_inputBitDepth[ChannelType::LUMA];
  const int threshold    = (1 << (m_inputBitDepth[ChannelType::LUMA] - 2)) - 1;

  // clang-format off
  // 9x9 cross
  static constexpr int xSupport[] = {  0,  0,  0,  0, -4, -3, -2, -1,  0, +1, +2, +3, +4, -4, -3, -2, -1,  0, +1, +2, +3, +4,  0,  0,  0 };
  static constexpr int ySupport[] = { -4, -3, -2, -1,  0,  0,  0,  0,  0,  0,  0,  0,  0, +1, +1, +1, +1, +1, +1, +1, +1, +1, +2, +3, +4 };
  // clang-format on

  static_assert(sizeof(xSupport) / sizeof(xSupport[0]) == CC_ALF_NUM_COEFF_LUMA, "Invalid size of xSupport.");
  static_assert(sizeof(ySupport) / sizeof(ySupport[0]) == CC_ALF_NUM_COEFF_LUMA, "Invalid size of ySupport.");

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
          for (int i = 0; i < CC_ALF_NUM_COEFF_LUMA; ++i)
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
