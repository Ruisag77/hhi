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

/**
 \file     EncSampleAdaptiveOffset.cpp
 \brief       estimation part of sample adaptive offset class
 */
#include "EncSampleAdaptiveOffset.h"

#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/CodingStructure.h"
#include "CommonLib/BilateralFilter.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

//! \ingroup EncoderLib
//! \{

#define SAOCtx(c) SubCtx(Ctx::Sao, c)
#include <algorithm>

struct SetIdxCount
{
  uint8_t  setIdx;
  uint16_t count;
};

struct CtbCost
{
  int16_t pos;
  double  cost;
};

bool compareSetIdxCount(SetIdxCount a, SetIdxCount b) { return a.count > b.count; }

bool compareCtbCost(CtbCost a, CtbCost b) { return a.cost < b.cost; }

//! rounding with IBDI
inline double xRoundIbdi2(int bitDepth, double x) { return ((x) >= 0 ? ((int)((x) + 0.5)) : ((int)((x)-0.5))); }

inline double xRoundIbdi(int bitDepth, double x)
{
  return (bitDepth > 8 ? xRoundIbdi2(bitDepth, (x)) : ((x) >= 0 ? ((int)((x) + 0.5)) : ((int)((x)-0.5))));
}

void getCcSaomSampleStats_Core(int startx, int endx, const int bitDepth, int64_t *diff, uint32_t *count,
                               const int chromaScaleX, const uint16_t bandNumY, const uint16_t bandNumU,
                               const uint16_t bandNumV, const Pel *srcY, const Pel *srcU, const Pel *srcV,
                               const Pel *org, const Pel *dst)
{
  for (int x = startx; x < endx; x++)
  {
    const Pel *colY = srcY + x;
    const Pel *colU = srcU + (x >> chromaScaleX);
    const Pel *colV = srcV + (x >> chromaScaleX);

    const int bandY    = (*colY * bandNumY) >> bitDepth;
    const int bandU    = (*colU * bandNumU) >> bitDepth;
    const int bandV    = (*colV * bandNumV) >> bitDepth;
    const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
    const int classIdx = bandIdx;

    diff[classIdx] += org[x] - dst[x];
    count[classIdx]++;
  }
}

void getCcSaomChromaSampleStats_Core(int startx, int endx, const int bitDepth, int64_t *diff, uint32_t *count,
                                     const int chromaScaleX, const uint16_t bandNumY, const uint16_t bandNumU,
                                     const uint16_t bandNumV, const Pel *srcY, const Pel *srcU, const Pel *srcV,
                                     const Pel *org, const Pel *dst)
{
  for (int x = startx; x < endx; x++)
  {
    const Pel *colY = srcY + (x << chromaScaleX);
    const Pel *colU = srcU + x;
    const Pel *colV = srcV + x;

    const int bandY    = (*colY * bandNumY) >> bitDepth;
    const int bandU    = (*colU * bandNumU) >> bitDepth;
    const int bandV    = (*colV * bandNumV) >> bitDepth;
    const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
    const int classIdx = bandIdx;

    diff[classIdx] += org[x] - dst[x];
    count[classIdx]++;
  }
}

EncSampleAdaptiveOffset::EncSampleAdaptiveOffset()
{
  ::memset(m_saoDisabledRate, 0, sizeof(m_saoDisabledRate));
  m_getCcSaomSampleStats       = getCcSaomSampleStats_Core;
  m_getCcSaomChromaSampleStats = getCcSaomChromaSampleStats_Core;
#if ENABLE_SIMD_OPT_CCSAO
#ifdef TARGET_SIMD_X86
  initCCSAOX86();
#endif
#endif
}

EncSampleAdaptiveOffset::~EncSampleAdaptiveOffset() { destroyEncData(); }

void EncSampleAdaptiveOffset::createEncData(bool isPreDBFSamplesUsed, uint32_t numCTUsPic)
{
  // statistics
  const uint32_t sizeInCtus = numCTUsPic;
  m_statData.resize(sizeInCtus);

  for (uint32_t i = 0; i < sizeInCtus; i++)
  {
    m_statData[i] = new StatDataArray[MAX_NUM_COMP];
  }

  if (isPreDBFSamplesUsed)
  {
    m_preDBFstatData.resize(sizeInCtus);
    for (uint32_t i = 0; i < sizeInCtus; i++)
    {
      m_preDBFstatData[i] = new StatDataArray[MAX_NUM_COMP];
    }
  }

  for (const auto typeIdc: { SAOModeNewTypes::EO_0, SAOModeNewTypes::EO_90, SAOModeNewTypes::EO_135,
                             SAOModeNewTypes::EO_45, SAOModeNewTypes::BO })
  {
    m_skipLinesR[COMP_Y][typeIdc]  = 5;
    m_skipLinesR[COMP_Cb][typeIdc] = m_skipLinesR[COMP_Cr][typeIdc] = 3;

    m_skipLinesB[COMP_Y][typeIdc]  = 4;
    m_skipLinesB[COMP_Cb][typeIdc] = m_skipLinesB[COMP_Cr][typeIdc] = 2;

    if (isPreDBFSamplesUsed)
    {
      switch (typeIdc)
      {
      case SAOModeNewTypes::EO_0:
        m_skipLinesR[COMP_Y][typeIdc]  = 5;
        m_skipLinesR[COMP_Cb][typeIdc] = m_skipLinesR[COMP_Cr][typeIdc] = 3;

        m_skipLinesB[COMP_Y][typeIdc]  = 3;
        m_skipLinesB[COMP_Cb][typeIdc] = m_skipLinesB[COMP_Cr][typeIdc] = 1;
        break;

      case SAOModeNewTypes::EO_90:
        m_skipLinesR[COMP_Y][typeIdc]  = 4;
        m_skipLinesR[COMP_Cb][typeIdc] = m_skipLinesR[COMP_Cr][typeIdc] = 2;

        m_skipLinesB[COMP_Y][typeIdc]  = 4;
        m_skipLinesB[COMP_Cb][typeIdc] = m_skipLinesB[COMP_Cr][typeIdc] = 2;
        break;

      case SAOModeNewTypes::EO_135:
      case SAOModeNewTypes::EO_45:
        m_skipLinesR[COMP_Y][typeIdc]  = 5;
        m_skipLinesR[COMP_Cb][typeIdc] = m_skipLinesR[COMP_Cr][typeIdc] = 3;

        m_skipLinesB[COMP_Y][typeIdc]  = 4;
        m_skipLinesB[COMP_Cb][typeIdc] = m_skipLinesB[COMP_Cr][typeIdc] = 2;
        break;

      case SAOModeNewTypes::BO:
        m_skipLinesR[COMP_Y][typeIdc]  = 4;
        m_skipLinesR[COMP_Cb][typeIdc] = m_skipLinesR[COMP_Cr][typeIdc] = 2;

        m_skipLinesB[COMP_Y][typeIdc]  = 3;
        m_skipLinesB[COMP_Cb][typeIdc] = m_skipLinesB[COMP_Cr][typeIdc] = 1;
        break;

      default:
        THROW("Not a supported type");
        break;
      }
    }
  }

  if (m_createdEnc)
  {
    return;
  }
  m_createdEnc = true;

  for (int i = 0; i < MAX_CCSAO_SET_NUM; i++)
  {
    m_ccSaoStatData[i].resize(m_numCTUsInPic);
    m_ccSaoStatDataEdge[i].resize(m_numCTUsInPic);
  }
  int numStatsEdge = (m_CCSaoMode != 2)
    ? m_numCTUsInPic * MAX_CCSAO_EDGE_DIR * MAX_CCSAO_EDGE_THR * MAX_CCSAO_BAND_IDC * MAX_NUM_COMP * MAX_CCSAO_EDGE_IDC
    : m_numCTUsInPic * MAX_CCSAO_EDGE_DIR * 4 * MAX_CCSAO_BAND_IDC * MAX_NUM_COMP * MAX_CCSAO_EDGE_IDC;
  m_ccSaoStatDataEdgePre.resize(numStatsEdge);
  m_bestCcSaoControl = new uint8_t[m_numCTUsInPic];
  m_tempCcSaoControl = new uint8_t[m_numCTUsInPic];
  m_initCcSaoControl = new uint8_t[m_numCTUsInPic];

  for (int i = 0; i < MAX_CCSAO_SET_NUM; i++)
  {
    m_trainingDistortion[i] = new int64_t[m_numCTUsInPic];
  }
}

void EncSampleAdaptiveOffset::destroyEncData()
{
  for (uint32_t i = 0; i < m_statData.size(); i++)
  {
    delete[] m_statData[i];
  }
  m_statData.clear();

  for (int i = 0; i < m_preDBFstatData.size(); i++)
  {
    delete[] m_preDBFstatData[i];
  }
  m_preDBFstatData.clear();

  if (!m_createdEnc)
  {
    return;
  }
  m_createdEnc = false;

  if (m_bestCcSaoControl)
  {
    delete[] m_bestCcSaoControl;
    m_bestCcSaoControl = nullptr;
  }
  if (m_tempCcSaoControl)
  {
    delete[] m_tempCcSaoControl;
    m_tempCcSaoControl = nullptr;
  }
  if (m_initCcSaoControl)
  {
    delete[] m_initCcSaoControl;
    m_initCcSaoControl = nullptr;
  }

  for (int i = 0; i < MAX_CCSAO_SET_NUM; i++)
  {
    if (m_trainingDistortion[i])
    {
      delete[] m_trainingDistortion[i];
      m_trainingDistortion[i] = nullptr;
    }
  }
}

void EncSampleAdaptiveOffset::initCABACEstimator(CABACEncoder *cabacEncoder, CtxPool *ctxPool, Slice *pcSlice)
{
  m_CABACEstimator = cabacEncoder->getCABACEstimator(pcSlice->m_sps);
  m_ctxPool        = ctxPool;
  m_CABACEstimator->initCtxModels(*pcSlice);
  m_CABACEstimator->resetBits();
}

void EncSampleAdaptiveOffset::SAOProcess(CodingStructure &cs, bool *sliceEnabled, const double *lambdas,
#if ENABLE_QPA
                                         const double lambdaChromaWeight,
#endif
                                         const bool testSAODisableAtPictureLevel, const double saoEncodingRate,
                                         const double saoEncodingRateChroma, const bool isPreDBFSamplesUsed,
                                         bool isGreedyMergeEncoding, bool usingTrueOrg, BIFCabacEst *bifCABACEstimator)
{
  PROFILER_SCOPE(1, g_timeProfiler, P_SAO);
  PelUnitBuf org = usingTrueOrg ? cs.getTrueOrgBuf() : cs.getOrgBuf();
  PelUnitBuf res = cs.getRecoBuf();
  PelUnitBuf src = m_tempBuf;

  src.copyFrom(res);
  const PreCalcValues &pcv         = *cs.pcv;
  BifParams           &bifParams   = cs.picture->getBifParam(COMP_Y);
  int                  width       = cs.picture->lwidth();
  int                  height      = cs.picture->lheight();
  int                  blockWidth  = pcv.maxCUWidth;
  int                  blockHeight = pcv.maxCUHeight;

  int       widthInBlocks  = width / blockWidth + (width % blockWidth != 0);
  int       heightInBlocks = height / blockHeight + (height % blockHeight != 0);
  const int sizeinBlks     = widthInBlocks * heightInBlocks;
  // Currently no RDO to figure out if we should turn CTUs on or off

  bifParams.initCTUflag(sizeinBlks, 1, 1);

  if (cs.pps->m_chromaBIF)
  {
    BifParams &bifParamsCb = cs.picture->getBifParam(COMP_Cb);
    BifParams &bifParamsCr = cs.picture->getBifParam(COMP_Cr);

    bifParamsCb.initCTUflag(sizeinBlks, 0, 0);
    bifParamsCr.initCTUflag(sizeinBlks, 0, 0);
  }

  if (!cs.sps->m_saoEnabledFlag && (cs.pps->m_BIF || cs.pps->m_chromaBIF))
  {
    if (cs.pps->m_BIF)
    {
      m_bilateralFilter.bilateralFilterPicRDOperCTU(COMP_Y, cs, src, bifCABACEstimator); // Filters from src to res
    }
    if (cs.pps->m_chromaBIF)
    {
      // Cb
      m_bilateralFilter.bilateralFilterPicRDOperCTU(COMP_Cb, cs, src, bifCABACEstimator);
      // Cr
      m_bilateralFilter.bilateralFilterPicRDOperCTU(COMP_Cr, cs, src, bifCABACEstimator);
    }
    return;
  }
  memcpy(m_lambda, lambdas, sizeof(m_lambda));

  // collect statistics
  if (cs.pps->m_BIF || cs.pps->m_chromaBIF)
  {
    if (cs.pps->m_BIF)
    {
      m_bilateralFilter.bilateralFilterPicRDOperCTU(COMP_Y, cs, src, bifCABACEstimator); // Filters from src to res'
    }
    if (cs.pps->m_chromaBIF)
    {
      // Cb
      m_bilateralFilter.bilateralFilterPicRDOperCTU(COMP_Cb, cs, src, bifCABACEstimator);
      // Cr
      m_bilateralFilter.bilateralFilterPicRDOperCTU(COMP_Cr, cs, src, bifCABACEstimator);
    }
    getStatistics(m_statData, org, src, res, cs);
  }
  else
  {
    getStatistics(m_statData, org, src, src, cs);
  }
  if (isPreDBFSamplesUsed)
  {
    addPreDBFStatistics(m_statData);
  }
  if (cs.pps->m_BIF || cs.pps->m_chromaBIF)
  {
    res.copyFrom(src);
  }

  // slice on/off
  decidePicParams(*cs.slice, sliceEnabled, saoEncodingRate, saoEncodingRateChroma);

  // block on/off
  std::vector<SAOBlkParam> reconParams(cs.pcv->sizeInCtus);
  decideBlkParams(cs, sliceEnabled, m_statData, src, res, &reconParams[0], cs.picture->getSAO(),
                  testSAODisableAtPictureLevel,
#if ENABLE_QPA
                  lambdaChromaWeight,
#endif
                  saoEncodingRate, saoEncodingRateChroma, isGreedyMergeEncoding);

  DTRACE_UPDATE(g_trace_ctx, (std::make_pair("poc", cs.slice->m_poc)));
  DTRACE_PIC_COMP(D_REC_CB_LUMA_SAO, cs, cs.getRecoBuf(), COMP_Y);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_SAO, cs, cs.getRecoBuf(), COMP_Cb);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_SAO, cs, cs.getRecoBuf(), COMP_Cr);

  DTRACE(g_trace_ctx, D_CRC, "SAO");
  DTRACE_CRC(g_trace_ctx, D_CRC, cs, cs.getRecoBuf());
}

void EncSampleAdaptiveOffset::getPreDBFStatistics(CodingStructure &cs, bool usingTrueOrg)
{
  PelUnitBuf org = usingTrueOrg ? cs.getTrueOrgBuf() : cs.getOrgBuf();
  PelUnitBuf rec = cs.getRecoBuf();
  getStatistics(m_preDBFstatData, org, rec, rec, cs, true);
}

void EncSampleAdaptiveOffset::addPreDBFStatistics(std::vector<StatDataArray *> &blkStats)
{
  const uint32_t numCTUsPic = (uint32_t)blkStats.size();
  for (uint32_t n = 0; n < numCTUsPic; n++)
  {
    for (uint32_t compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
    {
      for (const auto typeIdc: { SAOModeNewTypes::EO_0, SAOModeNewTypes::EO_90, SAOModeNewTypes::EO_135,
                                 SAOModeNewTypes::EO_45, SAOModeNewTypes::BO })
      {
        blkStats[n][compIdx][typeIdc] += m_preDBFstatData[n][compIdx][typeIdc];
      }
    }
  }
}

void EncSampleAdaptiveOffset::getStatistics(std::vector<StatDataArray *> &blkStats, PelUnitBuf &orgYuv,
                                            PelUnitBuf &srcYuv, PelUnitBuf &bifYuv, CodingStructure &cs,
                                            bool isCalculatePreDeblockSamples)
{
  bool isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail;

  const PreCalcValues &pcv                = *cs.pcv;
  const int            numberOfComponents = getNumberValidComponents(pcv.chrFormat);

  size_t lineBufferSize = pcv.maxCUWidth + 1;
  if (m_signLineBuf1.size() != lineBufferSize)
  {
    m_signLineBuf1.resize(lineBufferSize);
    m_signLineBuf2.resize(lineBufferSize);
  }

  int ctuRsAddr = 0;
  for (uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
  {
    for (uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
    {
      const uint32_t width  = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
      const uint32_t height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
      const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));

      deriveLoopFilterBoundaryAvailability(cs, area.Y(), isLeftAvail, isAboveAvail, isAboveLeftAvail);

      // NOTE: The number of skipped lines during gathering CTU statistics depends on the slice boundary availabilities.
      // For simplicity, here only picture boundaries are considered.

      isRightAvail      = (xPos + pcv.maxCUWidth < pcv.lumaWidth);
      isBelowAvail      = (yPos + pcv.maxCUHeight < pcv.lumaHeight);
      isAboveRightAvail = ((yPos > 0) && (isRightAvail));

      int numHorVirBndry = 0, numVerVirBndry = 0;
      int horVirBndryPos[]     = { -1, -1, -1 };
      int verVirBndryPos[]     = { -1, -1, -1 };
      int horVirBndryPosComp[] = { -1, -1, -1 };
      int verVirBndryPosComp[] = { -1, -1, -1 };

      for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
      {
        const CompID    compID   = CompID(compIdx);
        const CompArea &compArea = area.block(compID);

        ptrdiff_t srcStride = srcYuv.get(compID).stride;
        Pel      *srcBlk    = srcYuv.get(compID).bufAt(compArea);

        ptrdiff_t orgStride = orgYuv.get(compID).stride;
        Pel      *orgBlk    = orgYuv.get(compID).bufAt(compArea);
        ptrdiff_t bifStride = bifYuv.get(compID).stride;
        Pel      *bifBlk    = bifYuv.get(compID).bufAt(compArea);
        for (int i = 0; i < numHorVirBndry; i++)
        {
          horVirBndryPosComp[i] = (horVirBndryPos[i] >> ::getComponentScaleY(compID, area.chromaFormat)) - compArea.y;
        }
        for (int i = 0; i < numVerVirBndry; i++)
        {
          verVirBndryPosComp[i] = (verVirBndryPos[i] >> ::getComponentScaleX(compID, area.chromaFormat)) - compArea.x;
        }

        getBlkStats(compID, cs.sps->m_bitDepths[toChannelType(compID)], blkStats[ctuRsAddr][compID], srcBlk, orgBlk,
                    bifBlk, bifStride, srcStride, orgStride, compArea.width, compArea.height, isLeftAvail, isRightAvail,
                    isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail, isCalculatePreDeblockSamples,
                    horVirBndryPosComp, verVirBndryPosComp, numHorVirBndry, numVerVirBndry);
      }
      ctuRsAddr++;
    }
  }
}

void EncSampleAdaptiveOffset::decidePicParams(const Slice &slice, bool *sliceEnabled, const double saoEncodingRate,
                                              const double saoEncodingRateChroma)
{
  if (slice.m_pendingRasInit)
  {   // reset
    for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
    {
      for (int tempLayer = 1; tempLayer < MAX_TLAYER; tempLayer++)
      {
        m_saoDisabledRate[compIdx][tempLayer] = 0.0;
      }
    }
  }

  const int hierPredLayerIdx = slice.m_hierPredLayerIdx;

  // decide sliceEnabled[compIdx]
  const int numberOfComponents = m_numberOfComponents;
  for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
  {
    sliceEnabled[compIdx] = false;
  }

  for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    // reset flags & counters
    sliceEnabled[compIdx] = true;

    if (saoEncodingRate > 0.0)
    {
      if (saoEncodingRateChroma > 0.0)
      {
        // decide slice-level on/off based on previous results
        if (hierPredLayerIdx > 0 &&
            (m_saoDisabledRate[compIdx][hierPredLayerIdx - 1] >
             ((compIdx == COMP_Y) ? saoEncodingRate : saoEncodingRateChroma)))
        {
          sliceEnabled[compIdx] = false;
        }
      }
      else
      {
        // decide slice-level on/off based on previous results
        if (hierPredLayerIdx > 0 && (m_saoDisabledRate[COMP_Y][0] > saoEncodingRate))
        {
          sliceEnabled[compIdx] = false;
        }
      }
    }
  }
}

int64_t EncSampleAdaptiveOffset::getDistortion(const int channelBitDepth, SAOModeNewTypes typeIdc, int typeAuxInfo,
                                               int *invQuantOffset, SAOStatData &statData)
{
  int64_t dist = 0;

  const int shift = 2 * DISTORTION_PRECISION_ADJUSTMENT(channelBitDepth);

  switch (typeIdc)
  {
  case SAOModeNewTypes::EO_0:
  case SAOModeNewTypes::EO_90:
  case SAOModeNewTypes::EO_135:
  case SAOModeNewTypes::EO_45:
    for (int offsetIdx = 0; offsetIdx < NUM_SAO_EO_CLASSES; offsetIdx++)
    {
      dist += estSaoDist(statData.count[offsetIdx], invQuantOffset[offsetIdx], statData.diff[offsetIdx], shift);
    }
    break;
  case SAOModeNewTypes::BO:
    for (int offsetIdx = typeAuxInfo; offsetIdx < typeAuxInfo + 4; offsetIdx++)
    {
      const int bandIdx = offsetIdx % NUM_SAO_BO_CLASSES;
      dist += estSaoDist(statData.count[bandIdx], invQuantOffset[bandIdx], statData.diff[bandIdx], shift);
    }
    break;
  default:
    THROW("Not a supported type");
    break;
  }

  return dist;
}

int EncSampleAdaptiveOffset::estIterOffset(SAOModeNewTypes typeIdx, double lambda, int offsetInput, int64_t count,
                                           int64_t diffSum, int shift, int bitIncrease, int64_t &bestDist,
                                           double &bestCost, int offsetTh)
{
  int     iterOffset, tempOffset;
  int64_t tempDist, tempRate;
  double  tempCost, tempMinCost;
  int     offsetOutput = 0;
  iterOffset           = offsetInput;
  // Assuming sending quantized value 0 results in zero offset and sending the value zero needs 1 bit. entropy coder can
  // be used to measure the exact rate here.
  tempMinCost          = lambda;
  while (iterOffset != 0)
  {
    // Calculate the bits required for signaling the offset
    tempRate = (typeIdx == SAOModeNewTypes::BO) ? (abs((int)iterOffset) + 2) : (abs((int)iterOffset) + 1);
    if (abs((int)iterOffset) == offsetTh)   // inclusive
    {
      tempRate--;
    }
    // Do the dequantization before distortion calculation
    tempOffset = iterOffset * (1 << bitIncrease);
    tempDist   = estSaoDist(count, tempOffset, diffSum, shift);
    tempCost   = ((double)tempDist + lambda * (double)tempRate);
    if (tempCost < tempMinCost)
    {
      tempMinCost  = tempCost;
      offsetOutput = iterOffset;
      bestDist     = tempDist;
      bestCost     = tempCost;
    }
    iterOffset = (iterOffset > 0) ? (iterOffset - 1) : (iterOffset + 1);
  }
  return offsetOutput;
}

void EncSampleAdaptiveOffset::deriveOffsets(CompID compIdx, const int channelBitDepth, SAOModeNewTypes typeIdc,
                                            SAOStatData &statData, int *quantOffsets, int &typeAuxInfo)
{
  int bitDepth = channelBitDepth;
  int shift    = 2 * DISTORTION_PRECISION_ADJUSTMENT(bitDepth);
  int offsetTh = SampleAdaptiveOffset::getMaxOffsetQVal(channelBitDepth);   // inclusive

  ::memset(quantOffsets, 0, sizeof(int) * MAX_NUM_SAO_CLASSES);

  // derive initial offsets
  int numClasses = (typeIdc == SAOModeNewTypes::BO) ? ((int)NUM_SAO_BO_CLASSES) : ((int)NUM_SAO_EO_CLASSES);
  for (int classIdx = 0; classIdx < numClasses; classIdx++)
  {
    if ((typeIdc != SAOModeNewTypes::BO) && (classIdx == SAO_CLASS_EO_PLAIN))
    {
      continue;   // offset will be zero
    }

    if (statData.count[classIdx] == 0)
    {
      continue;   // offset will be zero
    }

    quantOffsets[classIdx] =
      (int)xRoundIbdi(bitDepth,
                      (double)(statData.diff[classIdx] * (1 << DISTORTION_PRECISION_ADJUSTMENT(bitDepth))) /
                        (double)(statData.count[classIdx] << m_offsetStepLog2[compIdx]));
    quantOffsets[classIdx] = Clip3(-offsetTh, offsetTh, quantOffsets[classIdx]);
  }

  // adjust offsets
  switch (typeIdc)
  {
  case SAOModeNewTypes::EO_0:
  case SAOModeNewTypes::EO_90:
  case SAOModeNewTypes::EO_135:
  case SAOModeNewTypes::EO_45:
    {
      int64_t classDist;
      double  classCost;
      for (int classIdx = 0; classIdx < NUM_SAO_EO_CLASSES; classIdx++)
      {
        if (classIdx == SAO_CLASS_EO_FULL_VALLEY && quantOffsets[classIdx] < 0)
        {
          quantOffsets[classIdx] = 0;
        }
        if (classIdx == SAO_CLASS_EO_HALF_VALLEY && quantOffsets[classIdx] < 0)
        {
          quantOffsets[classIdx] = 0;
        }
        if (classIdx == SAO_CLASS_EO_HALF_PEAK && quantOffsets[classIdx] > 0)
        {
          quantOffsets[classIdx] = 0;
        }
        if (classIdx == SAO_CLASS_EO_FULL_PEAK && quantOffsets[classIdx] > 0)
        {
          quantOffsets[classIdx] = 0;
        }

        if (quantOffsets[classIdx] != 0)   // iterative adjustment only when derived offset is not zero
        {
          quantOffsets[classIdx] =
            estIterOffset(typeIdc, m_lambda[compIdx], quantOffsets[classIdx], statData.count[classIdx],
                          statData.diff[classIdx], shift, m_offsetStepLog2[compIdx], classDist, classCost, offsetTh);
        }
      }

      typeAuxInfo = 0;
    }
    break;
  case SAOModeNewTypes::BO:
    {
      int64_t distBOClasses[NUM_SAO_BO_CLASSES];
      double  costBOClasses[NUM_SAO_BO_CLASSES];
      ::memset(distBOClasses, 0, sizeof(int64_t) * NUM_SAO_BO_CLASSES);
      for (int classIdx = 0; classIdx < NUM_SAO_BO_CLASSES; classIdx++)
      {
        costBOClasses[classIdx] = m_lambda[compIdx];
        if (quantOffsets[classIdx] != 0)   // iterative adjustment only when derived offset is not zero
        {
          quantOffsets[classIdx] = estIterOffset(
            typeIdc, m_lambda[compIdx], quantOffsets[classIdx], statData.count[classIdx], statData.diff[classIdx],
            shift, m_offsetStepLog2[compIdx], distBOClasses[classIdx], costBOClasses[classIdx], offsetTh);
        }
      }

      // decide the starting band index
      double minCost = MAX_DOUBLE, cost;
      for (int band = 0; band < NUM_SAO_BO_CLASSES - 4 + 1; band++)
      {
        cost = costBOClasses[band];
        cost += costBOClasses[band + 1];
        cost += costBOClasses[band + 2];
        cost += costBOClasses[band + 3];

        if (cost < minCost)
        {
          minCost     = cost;
          typeAuxInfo = band;
        }
      }
      // clear those unused classes
      int clearQuantOffset[NUM_SAO_BO_CLASSES];
      ::memset(clearQuantOffset, 0, sizeof(int) * NUM_SAO_BO_CLASSES);
      for (int i = 0; i < 4; i++)
      {
        int band               = (typeAuxInfo + i) % NUM_SAO_BO_CLASSES;
        clearQuantOffset[band] = quantOffsets[band];
      }
      ::memcpy(quantOffsets, clearQuantOffset, sizeof(int) * NUM_SAO_BO_CLASSES);
    }
    break;
  default:
    {
      THROW("Not a supported type");
    }
  }
}

void EncSampleAdaptiveOffset::deriveModeNewRDO(const BitDepths &bitDepths, int ctuRsAddr, MergeBlkParams &mergeList,
                                               bool *sliceEnabled, std::vector<StatDataArray *> &blkStats,
                                               SAOBlkParam &modeParam, double &modeNormCost)
{
  double    minCost, cost;
  uint64_t  previousFracBits;
  const int numberOfComponents = m_numberOfComponents;

  int64_t   dist[MAX_NUM_COMP], modeDist[MAX_NUM_COMP];
  SAOOffset testOffset[MAX_NUM_COMP];
  int       invQuantOffset[MAX_NUM_SAO_CLASSES];
  for (int comp = 0; comp < MAX_NUM_COMP; comp++)
  {
    modeDist[comp] = 0;
  }

  // pre-encode merge flags
  modeParam[COMP_Y].modeIdc = SAOMode::OFF;
  const TempCtx ctxStartBlk(m_ctxPool, SAOCtx(m_CABACEstimator->getCtx()));
  m_CABACEstimator->sao_block_params(modeParam, bitDepths, sliceEnabled,
                                     (mergeList[SAOModeMergeTypes::LEFT] != nullptr),
                                     (mergeList[SAOModeMergeTypes::ABOVE] != nullptr), true);
  const TempCtx ctxStartLuma(m_ctxPool, SAOCtx(m_CABACEstimator->getCtx()));
  TempCtx       ctxBestLuma(m_ctxPool);

  //------ luma --------//
  {
    const CompID compIdx       = COMP_Y;
    //"off" case as initial cost
    modeParam[compIdx].modeIdc = SAOMode::OFF;
    m_CABACEstimator->resetBits();
    m_CABACEstimator->sao_offset_params(modeParam[compIdx], compIdx, sliceEnabled[compIdx],
                                        bitDepths[ChannelType::LUMA]);
    modeDist[compIdx] = 0;
    minCost           = m_lambda[compIdx] * (FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits());
    ctxBestLuma       = SAOCtx(m_CABACEstimator->getCtx());
    if (sliceEnabled[compIdx])
    {
      for (const auto typeIdc: { SAOModeNewTypes::EO_0, SAOModeNewTypes::EO_90, SAOModeNewTypes::EO_135,
                                 SAOModeNewTypes::EO_45, SAOModeNewTypes::BO })
      {
        testOffset[compIdx].modeIdc         = SAOMode::NEW;
        testOffset[compIdx].typeIdc.newType = typeIdc;

        // derive coded offset
        deriveOffsets(compIdx, bitDepths[ChannelType::LUMA], typeIdc, blkStats[ctuRsAddr][compIdx][typeIdc],
                      testOffset[compIdx].offset, testOffset[compIdx].typeAuxInfo);

        // inversed quantized offsets
        invertQuantOffsets(compIdx, typeIdc, testOffset[compIdx].typeAuxInfo, invQuantOffset,
                           testOffset[compIdx].offset);

        // get distortion
        dist[compIdx] =
          getDistortion(bitDepths[ChannelType::LUMA], testOffset[compIdx].typeIdc.newType,
                        testOffset[compIdx].typeAuxInfo, invQuantOffset, blkStats[ctuRsAddr][compIdx][typeIdc]);

        // get rate
        m_CABACEstimator->getCtx() = SAOCtx(ctxStartLuma);
        m_CABACEstimator->resetBits();
        m_CABACEstimator->sao_offset_params(testOffset[compIdx], compIdx, sliceEnabled[compIdx],
                                            bitDepths[ChannelType::LUMA]);
        double rate = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
        cost        = (double)dist[compIdx] + m_lambda[compIdx] * rate;
        if (cost < minCost)
        {
          minCost            = cost;
          modeDist[compIdx]  = dist[compIdx];
          modeParam[compIdx] = testOffset[compIdx];
          ctxBestLuma        = SAOCtx(m_CABACEstimator->getCtx());
        }
      }
    }
    m_CABACEstimator->getCtx() = SAOCtx(ctxBestLuma);
  }

  //------ chroma --------//
  //"off" case as initial cost
  cost             = 0;
  previousFracBits = 0;
  m_CABACEstimator->resetBits();
  for (uint32_t componentIndex = COMP_Cb; componentIndex < numberOfComponents; componentIndex++)
  {
    const CompID component = CompID(componentIndex);

    modeParam[component].modeIdc = SAOMode::OFF;
    modeDist[component]          = 0;
    m_CABACEstimator->sao_offset_params(modeParam[component], component, sliceEnabled[component],
                                        bitDepths[ChannelType::CHROMA]);
    const uint64_t currentFracBits = m_CABACEstimator->getEstFracBits();
    cost += m_lambda[component] * FRAC_BITS_SCALE * (currentFracBits - previousFracBits);
    previousFracBits = currentFracBits;
  }

  minCost = cost;

  // doesn't need to store cabac status here since the whole CTU parameters will be re-encoded at the end of this
  // function

  for (const auto typeIdc: { SAOModeNewTypes::EO_0, SAOModeNewTypes::EO_90, SAOModeNewTypes::EO_135,
                             SAOModeNewTypes::EO_45, SAOModeNewTypes::BO })
  {
    m_CABACEstimator->getCtx() = SAOCtx(ctxBestLuma);
    m_CABACEstimator->resetBits();
    previousFracBits = 0;
    cost             = 0;

    for (uint32_t componentIndex = COMP_Cb; componentIndex < numberOfComponents; componentIndex++)
    {
      const CompID component = CompID(componentIndex);
      if (!sliceEnabled[component])
      {
        testOffset[component].modeIdc = SAOMode::OFF;
        dist[component]               = 0;
        continue;
      }
      testOffset[component].modeIdc         = SAOMode::NEW;
      testOffset[component].typeIdc.newType = typeIdc;

      // derive offset & get distortion
      deriveOffsets(component, bitDepths[ChannelType::CHROMA], typeIdc, blkStats[ctuRsAddr][component][typeIdc],
                    testOffset[component].offset, testOffset[component].typeAuxInfo);
      invertQuantOffsets(component, typeIdc, testOffset[component].typeAuxInfo, invQuantOffset,
                         testOffset[component].offset);
      dist[component] = getDistortion(bitDepths[ChannelType::CHROMA], typeIdc, testOffset[component].typeAuxInfo,
                                      invQuantOffset, blkStats[ctuRsAddr][component][typeIdc]);
      m_CABACEstimator->sao_offset_params(testOffset[component], component, sliceEnabled[component],
                                          bitDepths[ChannelType::CHROMA]);
      const uint64_t currentFracBits = m_CABACEstimator->getEstFracBits();
      cost += dist[component] + (m_lambda[component] * FRAC_BITS_SCALE * (currentFracBits - previousFracBits));
      previousFracBits = currentFracBits;
    }

    if (cost < minCost)
    {
      minCost = cost;
      for (uint32_t componentIndex = COMP_Cb; componentIndex < numberOfComponents; componentIndex++)
      {
        modeDist[componentIndex]  = dist[componentIndex];
        modeParam[componentIndex] = testOffset[componentIndex];
      }
    }

  }   // SAO_TYPE loop

  //----- re-gen rate & normalized cost----//
  modeNormCost = 0;
  for (uint32_t componentIndex = COMP_Y; componentIndex < numberOfComponents; componentIndex++)
  {
    modeNormCost += (double)modeDist[componentIndex] / m_lambda[componentIndex];
  }

  m_CABACEstimator->getCtx() = SAOCtx(ctxStartBlk);
  m_CABACEstimator->resetBits();
  m_CABACEstimator->sao_block_params(modeParam, bitDepths, sliceEnabled,
                                     (mergeList[SAOModeMergeTypes::LEFT] != nullptr),
                                     (mergeList[SAOModeMergeTypes::ABOVE] != nullptr), false);
  modeNormCost += FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
}

void EncSampleAdaptiveOffset::deriveModeMergeRDO(const BitDepths &bitDepths, int ctuRsAddr, MergeBlkParams &mergeList,
                                                 bool *sliceEnabled, std::vector<StatDataArray *> &blkStats,
                                                 SAOBlkParam &modeParam, double &modeNormCost)
{
  modeNormCost = MAX_DOUBLE;

  double      cost;
  SAOBlkParam testBlkParam;
  const int   numberOfComponents = m_numberOfComponents;

  const TempCtx ctxStart(m_ctxPool, SAOCtx(m_CABACEstimator->getCtx()));
  TempCtx       ctxBest(m_ctxPool);

  for (const auto mergeType: { SAOModeMergeTypes::LEFT, SAOModeMergeTypes::ABOVE })
  {
    if (mergeList[mergeType] == nullptr)
    {
      continue;
    }

    testBlkParam    = *(mergeList[mergeType]);
    // normalized distortion
    double normDist = 0;
    for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
    {
      testBlkParam[compIdx].modeIdc           = SAOMode::MERGE;
      testBlkParam[compIdx].typeIdc.mergeType = mergeType;

      SAOOffset &mergedOffsetParam = (*(mergeList[mergeType]))[compIdx];

      if (mergedOffsetParam.modeIdc != SAOMode::OFF)
      {
        // offsets have been reconstructed. Don't call inversed quantization function.
        normDist +=
          (((double)getDistortion(bitDepths[toChannelType(CompID(compIdx))], mergedOffsetParam.typeIdc.newType,
                                  mergedOffsetParam.typeAuxInfo, mergedOffsetParam.offset,
                                  blkStats[ctuRsAddr][compIdx][mergedOffsetParam.typeIdc.newType])) /
           m_lambda[compIdx]);
      }
    }

    // rate
    m_CABACEstimator->getCtx() = SAOCtx(ctxStart);
    m_CABACEstimator->resetBits();
    m_CABACEstimator->sao_block_params(testBlkParam, bitDepths, sliceEnabled,
                                       (mergeList[SAOModeMergeTypes::LEFT] != nullptr),
                                       (mergeList[SAOModeMergeTypes::ABOVE] != nullptr), false);
    double rate = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
    cost        = normDist + rate;

    if (cost < modeNormCost)
    {
      modeNormCost = cost;
      modeParam    = testBlkParam;
      ctxBest      = SAOCtx(m_CABACEstimator->getCtx());
    }
  }
  if (modeNormCost < MAX_DOUBLE)
  {
    m_CABACEstimator->getCtx() = SAOCtx(ctxBest);
  }
}

void EncSampleAdaptiveOffset::decideBlkParams(CodingStructure &cs, bool *sliceEnabled,
                                              std::vector<StatDataArray *> &blkStats, PelUnitBuf &srcYuv,
                                              PelUnitBuf &resYuv, SAOBlkParam *reconParams, SAOBlkParam *codedParams,
                                              const bool testSAODisableAtPictureLevel,
#if ENABLE_QPA
                                              const double chromaWeight,
#endif
                                              const double saoEncodingRate, const double saoEncodingRateChroma,
                                              const bool isGreedymergeEncoding)

{
  const PreCalcValues &pcv             = *cs.pcv;
  bool                 allBlksDisabled = true;
  if (cs.sps->m_saoEnabledFlag)
  {
    // If SAO is enabled, we should investigate the components.
    // If SAO is disabled, we should stick with allBlksDisabled=true;
    const uint32_t numberOfComponents = m_numberOfComponents;
    for (uint32_t compId = COMP_Y; compId < numberOfComponents; compId++)
    {
      if (sliceEnabled[compId])
      {
        allBlksDisabled = false;
      }
    }
  }

  const TempCtx ctxPicStart(m_ctxPool, SAOCtx(m_CABACEstimator->getCtx()));

  SAOBlkParam modeParam;
  double      minCost, modeCost;

  double                       minCost2 = 0;
  std::vector<StatDataArray *> groupBlkStat;
  if (isGreedymergeEncoding)
  {
    groupBlkStat.resize(cs.pcv->sizeInCtus);
    for (uint32_t k = 0; k < cs.pcv->sizeInCtus; k++)
    {
      groupBlkStat[k] = new StatDataArray[MAX_NUM_COMP];
    }
  }
  SAOBlkParam testBlkParam;
  SAOBlkParam groupParam;

  MergeBlkParams tempMergeList;
  MergeBlkParams startingMergeList;
  tempMergeList.fill(nullptr);
  startingMergeList.fill(nullptr);

  int     mergeCtuAddr = 1;   // Ctu to be merged
  int     groupSize    = 1;
  double  cost[2]      = { 0, 0 };
  TempCtx ctxBeforeMerge(m_ctxPool);
  TempCtx ctxAfterMerge(m_ctxPool);

  double totalCost = 0;   // Used if testSAODisableAtPictureLevel==true

  int ctuRsAddr = 0;
#if ENABLE_QPA
  CHECK((chromaWeight > 0.0) && (cs.slice->getFirstCtuRsAddrInSlice() != 0),
        "incompatible start CTU address, must be 0");
#endif

  for (uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
  {
    for (uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth, ctuRsAddr++)
    {
      if (allBlksDisabled)
      {
        codedParams[ctuRsAddr].reset();
        if (!cs.pps->m_BIF && !cs.pps->m_chromaBIF)
        {
          continue;
        }
      }

      const uint32_t width  = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
      const uint32_t height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
      const UnitArea area(pcv.chrFormat, Area(xPos, yPos, width, height));

      const TempCtx ctxStart(m_ctxPool, SAOCtx(m_CABACEstimator->getCtx()));
      TempCtx       ctxBest(m_ctxPool);

      if (ctuRsAddr == mergeCtuAddr - 1)
      {
        ctxBeforeMerge = SAOCtx(m_CABACEstimator->getCtx());
      }

      // get merge list
      MergeBlkParams mergeList;
      getMergeList(cs, ctuRsAddr, reconParams, mergeList);

      minCost = MAX_DOUBLE;
#if ENABLE_QPA
      if (chromaWeight > 0.0)   // temporarily adopt local (CTU-wise) lambdas from QPA
      {
        for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
        {
          m_lambda[compIdx] = isLuma((CompID)compIdx) ? cs.picture->m_uEnerHpCtu[ctuRsAddr]
                                                      : cs.picture->m_uEnerHpCtu[ctuRsAddr] / chromaWeight;
        }
      }
#endif
      bool firstMode = true;
      for (const auto mode: { SAOMode::NEW, SAOMode::MERGE })
      {
        if (!firstMode)
        {
          m_CABACEstimator->getCtx() = SAOCtx(ctxStart);
        }
        firstMode = false;

        switch (mode)
        {
        case SAOMode::NEW:
          deriveModeNewRDO(cs.sps->m_bitDepths, ctuRsAddr, mergeList, sliceEnabled, blkStats, modeParam, modeCost);
          break;
        case SAOMode::MERGE:
          deriveModeMergeRDO(cs.sps->m_bitDepths, ctuRsAddr, mergeList, sliceEnabled, blkStats, modeParam, modeCost);
          break;
        default:
          THROW("Invalid SAO mode");
          break;
        }

        if (modeCost < minCost)
        {
          minCost                = modeCost;
          codedParams[ctuRsAddr] = modeParam;
          ctxBest                = SAOCtx(m_CABACEstimator->getCtx());
        }
      }

      if (!isGreedymergeEncoding)
      {
        totalCost += minCost;
      }

      m_CABACEstimator->getCtx() = SAOCtx(ctxBest);

      // apply reconstructed offsets
      reconParams[ctuRsAddr] = codedParams[ctuRsAddr];
      reconstructBlkSAOParam(reconParams[ctuRsAddr], mergeList);

      if (isGreedymergeEncoding)
      {
        if (ctuRsAddr == mergeCtuAddr - 1)
        {
          cost[0]   = minCost;   // previous
          groupSize = 1;
          getMergeList(cs, ctuRsAddr, reconParams, startingMergeList);
        }
        else if (ctuRsAddr == mergeCtuAddr)
        {
          cost[1]  = minCost;
          minCost2 = MAX_DOUBLE;
          for (int tmp = groupSize; tmp >= 0; tmp--)
          {
            for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
            {
              for (const auto i: { SAOModeNewTypes::EO_0, SAOModeNewTypes::EO_90, SAOModeNewTypes::EO_135,
                                   SAOModeNewTypes::EO_45, SAOModeNewTypes::BO })
              {
                for (int j = 0; j < MAX_NUM_SAO_CLASSES; j++)
                {
                  if (tmp == groupSize)
                  {
                    groupBlkStat[ctuRsAddr][compIdx][i].count[j] = blkStats[ctuRsAddr - tmp][compIdx][i].count[j];
                    groupBlkStat[ctuRsAddr][compIdx][i].diff[j]  = blkStats[ctuRsAddr - tmp][compIdx][i].diff[j];
                  }
                  else
                  {
                    groupBlkStat[ctuRsAddr][compIdx][i].count[j] += blkStats[ctuRsAddr - tmp][compIdx][i].count[j];
                    groupBlkStat[ctuRsAddr][compIdx][i].diff[j] += blkStats[ctuRsAddr - tmp][compIdx][i].diff[j];
                  }
                }
              }
            }
          }

          // Derive new offset for grouped CTUs
          m_CABACEstimator->getCtx() = SAOCtx(ctxBeforeMerge);
          deriveModeNewRDO(cs.sps->m_bitDepths, ctuRsAddr, startingMergeList, sliceEnabled, groupBlkStat, modeParam,
                           modeCost);

          // rate for mergeLeft CTB
          testBlkParam[COMP_Y].modeIdc           = SAOMode::MERGE;
          testBlkParam[COMP_Y].typeIdc.mergeType = SAOModeMergeTypes::LEFT;
          m_CABACEstimator->resetBits();
          m_CABACEstimator->sao_block_params(testBlkParam, cs.sps->m_bitDepths, sliceEnabled, true, false, true);
          double rate = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
          modeCost += rate * groupSize;
          if (modeCost < minCost2)
          {
            groupParam    = modeParam;
            minCost2      = modeCost;
            ctxAfterMerge = SAOCtx(m_CABACEstimator->getCtx());
          }

          // Test merge mode for grouped CTUs
          m_CABACEstimator->getCtx() = SAOCtx(ctxStart);
          deriveModeMergeRDO(cs.sps->m_bitDepths, ctuRsAddr, startingMergeList, sliceEnabled, groupBlkStat, modeParam,
                             modeCost);
          modeCost += rate * groupSize;
          if (modeCost < minCost2)
          {
            minCost2      = modeCost;
            groupParam    = modeParam;
            ctxAfterMerge = SAOCtx(m_CABACEstimator->getCtx());
          }

          totalCost += cost[0];
          totalCost += cost[1];

          if ((cost[0] + cost[1]) > minCost2)   // merge current CTU
          {
            // original merge all
            totalCost                          = totalCost - cost[0] - cost[1] + minCost2;
            codedParams[ctuRsAddr - groupSize] = groupParam;
            for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
            {
              codedParams[ctuRsAddr][compIdx].modeIdc           = SAOMode::MERGE;
              codedParams[ctuRsAddr][compIdx].typeIdc.mergeType = SAOModeMergeTypes::LEFT;
            }
            for (int i = groupSize; i >= 0; i--)   // change previous results
            {
              reconParams[ctuRsAddr - i] = codedParams[ctuRsAddr - i];
              getMergeList(cs, ctuRsAddr - i, reconParams, tempMergeList);
              reconstructBlkSAOParam(reconParams[ctuRsAddr - i], tempMergeList);
            }

            mergeCtuAddr += 1;
            if (mergeCtuAddr % pcv.widthInCtus == 0)   // reaching the end of a row
            {
              mergeCtuAddr += 1;
            }
            else   // next CTU can be merged with current group
            {
              cost[0] = minCost2;
              groupSize += 1;
            }
            m_CABACEstimator->getCtx() = SAOCtx(ctxAfterMerge);
          }
          else   // don't merge current CTU
          {
            mergeCtuAddr += 1;
            // Current block will be the starting block for successive operations
            cost[0] = cost[1];
            getMergeList(cs, ctuRsAddr, reconParams, startingMergeList);
            groupSize                  = 1;
            m_CABACEstimator->getCtx() = SAOCtx(ctxStart);
            ctxBeforeMerge             = SAOCtx(m_CABACEstimator->getCtx());
            m_CABACEstimator->getCtx() = SAOCtx(ctxBest);
            if (mergeCtuAddr % pcv.widthInCtus == 0)   // reaching the end of a row
            {
              mergeCtuAddr += 1;
            }
          }   // else, if(cost[0] + cost[1] > minCost2)
        }   // else if (ctuRsAddr == mergeCtuAddr)
      }
      else
      {
        if (cs.pps->m_BIF || cs.pps->m_chromaBIF)
        {
          offsetCTUnoClip(area, srcYuv, resYuv, reconParams[ctuRsAddr], cs);
          // Avoid old slow code
          // offsetCTUonlyBIF(area, srcYuv, resYuv, reconParams[ctuRsAddr], cs);
          // and instead do the code included below.

          // We don't need to clip if SAO was not performed on luma.
          SAOBlkParam mySAOblkParam = cs.picture->getSAO()[ctuRsAddr];
          SAOOffset  &myCtbOffset   = mySAOblkParam[0];
          BifParams  &bifParams     = cs.picture->getBifParam(COMP_Y);

          bool clipLumaIfNoBilat = false;
          if (myCtbOffset.modeIdc != SAOMode::OFF)
          {
            clipLumaIfNoBilat = true;
          }
          SAOOffset &myCtbOffsetCb = mySAOblkParam[1];
          SAOOffset &myCtbOffsetCr = mySAOblkParam[2];

          bool clipChromaIfNoBilat[MAX_NUM_COMP] = { false };

          if (myCtbOffsetCb.modeIdc != SAOMode::OFF)
          {
            clipChromaIfNoBilat[COMP_Cb] = true;
          }
          if (myCtbOffsetCr.modeIdc != SAOMode::OFF)
          {
            clipChromaIfNoBilat[COMP_Cr] = true;
          }
          if (cs.pps->m_BIF)
          {
            for (auto &currCU: cs.traverseCUs(CS::getArea(cs, area, ChannelType::LUMA), ChannelType::LUMA))
            {
              for (auto &currTU: CU::traverseTUs(currCU))
              {
                bool applyBIF = bifParams.ctuOn[ctuRsAddr] && m_bilateralFilter.getApplyBIF(currTU, COMP_Y);
                if (applyBIF)
                {
                  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
                  int  numHorVirBndry = 0, numVerVirBndry = 0;
                  int  horVirBndryPos[]               = { 0, 0, 0 };
                  int  verVirBndryPos[]               = { 0, 0, 0 };
                  bool isTUCrossedByVirtualBoundaries = m_bilateralFilter.isCrossedByVirtualBoundaries(
                    cs, currTU.lx(), currTU.ly(), currTU.lumaSize().width, currTU.lumaSize().height, clipTop,
                    clipBottom, clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);
                  m_bilateralFilter.bilateralFilterDiamond5x5(
                    COMP_Y, srcYuv, resYuv, currTU.cu->qp, cs.slice->clpRng(COMP_Y), currTU, false,
                    isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry, numVerVirBndry,
                    clipTop, clipBottom, clipLeft, clipRight);
                  // count_BIF++;
                }
                else
                {
                  // We don't need to clip if SAO was not performed on luma.
                  if (clipLumaIfNoBilat)
                  {
                    m_bilateralFilter.clipNotBilaterallyFilteredBlocks(COMP_Y, srcYuv, resYuv, cs.slice->clpRng(COMP_Y),
                                                                       currTU);
                    // count_clip_noBIF++;
                  }
                  // count_noBIF++;
                }
              }
            }
          }   // BIF LUMA is disabled
          else
          {
            for (auto &currCU: cs.traverseCUs(CS::getArea(cs, area, ChannelType::LUMA), ChannelType::LUMA))
            {
              for (auto &currTU: CU::traverseTUs(currCU))
              {
                if (clipLumaIfNoBilat)
                {
                  m_bilateralFilter.clipNotBilaterallyFilteredBlocks(COMP_Y, srcYuv, resYuv, cs.slice->clpRng(COMP_Y),
                                                                     currTU);
                }
              }
            }
          }
          if (cs.pps->m_chromaBIF)
          {
            bool        isDualTree = CS::isDualITree(cs);
            ChannelType chType     = isDualTree ? ChannelType::CHROMA : ChannelType::LUMA;

            for (auto &currCU: cs.traverseCUs(CS::getArea(cs, area, chType), chType))
            {
              bool chromaValid = currCU.Cb().valid() && currCU.Cr().valid();
              if (!chromaValid)
              {
                continue;
              }
              for (auto &currTU: CU::traverseTUs(currCU))
              {
                for (int compIdx = COMP_Cb; compIdx < MAX_NUM_COMP; compIdx++)
                {
                  CompID     compID          = CompID(compIdx);
                  BifParams &chromaBifParams = cs.picture->getBifParam(compID);
                  bool       applyChromaBIF =
                    chromaBifParams.ctuOn[ctuRsAddr] && m_bilateralFilter.getApplyBIF(currTU, compID);
                  if (applyChromaBIF)
                  {
                    bool      clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
                    int       numHorVirBndry = 0, numVerVirBndry = 0;
                    int       horVirBndryPos[] = { 0, 0, 0 };
                    int       verVirBndryPos[] = { 0, 0, 0 };
                    CompArea &myArea           = currTU.block(compID);
                    const int chromaScaleX     = getComponentScaleX(compID, currTU.cu->cs->pcv->chrFormat);
                    const int chromaScaleY     = getComponentScaleY(compID, currTU.cu->cs->pcv->chrFormat);
                    int       yPos             = myArea.y << chromaScaleY;
                    int       xPos             = myArea.x << chromaScaleX;
                    bool      isTUCrossedByVirtualBoundaries = m_bilateralFilter.isCrossedByVirtualBoundaries(
                      cs, xPos, yPos, myArea.width << chromaScaleX, myArea.height << chromaScaleY, clipTop, clipBottom,
                      clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);

                    m_bilateralFilter.bilateralFilterDiamond5x5(
                      compID, srcYuv, resYuv, currTU.cu->qp, cs.slice->clpRng(compID), currTU, false,
                      isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry, numVerVirBndry,
                      clipTop, clipBottom, clipLeft, clipRight);
                  }
                  else
                  {
                    bool useClip = clipChromaIfNoBilat[compID];
                    if (useClip && currTU.blocks[compID].valid())
                    {
                      m_bilateralFilter.clipNotBilaterallyFilteredBlocks(compID, srcYuv, resYuv,
                                                                         cs.slice->clpRng(compID), currTU);
                    }
                  }
                }
              }
            }
          }   // BIF chroma is disabled
          else if (isChromaEnabled(cs.sps->m_chromaFormatIdc))
          {
            bool        isDualTree = CS::isDualITree(cs);
            ChannelType chType     = isDualTree ? ChannelType::CHROMA : ChannelType::LUMA;

            for (auto &currCU: cs.traverseCUs(CS::getArea(cs, area, chType), chType))
            {
              bool chromaValid = currCU.Cb().valid() && currCU.Cr().valid();
              if (!chromaValid)
              {
                continue;
              }
              for (auto &currTU: CU::traverseTUs(currCU))
              {
                if (clipChromaIfNoBilat[COMP_Cb] && currTU.blocks[COMP_Cb].valid())
                {
                  m_bilateralFilter.clipNotBilaterallyFilteredBlocks(COMP_Cb, srcYuv, resYuv, cs.slice->clpRng(COMP_Cb),
                                                                     currTU);
                }
                if (clipChromaIfNoBilat[COMP_Cr] && currTU.blocks[COMP_Cr].valid())
                {
                  m_bilateralFilter.clipNotBilaterallyFilteredBlocks(COMP_Cr, srcYuv, resYuv, cs.slice->clpRng(COMP_Cr),
                                                                     currTU);
                }
              }
            }
          }
        }
        else
        {
          // We do not do BIF for this sequence, so we can use the regular SAO function
          offsetCTU(area, srcYuv, resYuv, reconParams[ctuRsAddr], cs);
        }
      }
    }   // ctuRsAddr
  }

#if ENABLE_QPA
  // restore global lambdas (might be unnecessary)
  if (chromaWeight > 0.0)
  {
    memcpy(m_lambda, cs.slice->getLambdas(), sizeof(m_lambda));
  }
#endif
  // reconstruct
  if (isGreedymergeEncoding || (cs.pps->m_BIF && allBlksDisabled) || (cs.pps->m_chromaBIF && allBlksDisabled))
  {
    ctuRsAddr = 0;
    for (uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
    {
      for (uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
      {
        const uint32_t width  = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
        const uint32_t height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;

        const UnitArea area(pcv.chrFormat, Area(xPos, yPos, width, height));
        if (cs.pps->m_BIF)
        {
          resYuv.subBuf(area).bufs[COMP_Y].copyFrom(srcYuv.subBuf(area).bufs[COMP_Y]);
        }
        if (cs.pps->m_chromaBIF)
        {
          resYuv.subBuf(area).bufs[COMP_Cb].copyFrom(srcYuv.subBuf(area).bufs[COMP_Cb]);
          resYuv.subBuf(area).bufs[COMP_Cr].copyFrom(srcYuv.subBuf(area).bufs[COMP_Cr]);
        }
        if (cs.pps->m_BIF || cs.pps->m_chromaBIF)
        {
          offsetCTUnoClip(area, srcYuv, resYuv, reconParams[ctuRsAddr], cs);
          SAOBlkParam mySAOblkParam = cs.picture->getSAO()[ctuRsAddr];
          SAOOffset  &myCtbOffset   = mySAOblkParam[0];
          BifParams  &bifParams     = cs.picture->getBifParam(COMP_Y);

          bool clipLumaIfNoBilat = false;
          if (myCtbOffset.modeIdc != SAOMode::OFF)
          {
            clipLumaIfNoBilat = true;
          }

          SAOOffset &myCtbOffsetCb                     = mySAOblkParam[1];
          SAOOffset &myCtbOffsetCr                     = mySAOblkParam[2];
          bool       clipChromaIfNoBilat[MAX_NUM_COMP] = { false };

          if (myCtbOffsetCb.modeIdc != SAOMode::OFF)
          {
            clipChromaIfNoBilat[COMP_Cb] = true;
          }
          if (myCtbOffsetCr.modeIdc != SAOMode::OFF)
          {
            clipChromaIfNoBilat[COMP_Cr] = true;
          }
          if (cs.pps->m_BIF)
          {
            for (auto &currCU: cs.traverseCUs(CS::getArea(cs, area, ChannelType::LUMA), ChannelType::LUMA))
            {
              for (auto &currTU: CU::traverseTUs(currCU))
              {
                bool applyBIF = bifParams.ctuOn[ctuRsAddr] && m_bilateralFilter.getApplyBIF(currTU, COMP_Y);
                if (applyBIF)
                {
                  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
                  int  numHorVirBndry = 0, numVerVirBndry = 0;
                  int  horVirBndryPos[]               = { 0, 0, 0 };
                  int  verVirBndryPos[]               = { 0, 0, 0 };
                  bool isTUCrossedByVirtualBoundaries = m_bilateralFilter.isCrossedByVirtualBoundaries(
                    cs, currTU.lx(), currTU.ly(), currTU.lumaSize().width, currTU.lumaSize().height, clipTop,
                    clipBottom, clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);
                  m_bilateralFilter.bilateralFilterDiamond5x5(
                    COMP_Y, srcYuv, resYuv, currTU.cu->qp, cs.slice->clpRng(COMP_Y), currTU, false,
                    isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry, numVerVirBndry,
                    clipTop, clipBottom, clipLeft, clipRight);
                }
                else
                {
                  // We don't need to clip if SAO was not performed on luma.
                  if (clipLumaIfNoBilat)
                  {
                    m_bilateralFilter.clipNotBilaterallyFilteredBlocks(COMP_Y, srcYuv, resYuv, cs.slice->clpRng(COMP_Y),
                                                                       currTU);
                  }
                }
              }
            }
          }
          else
          {
            for (auto &currCU: cs.traverseCUs(CS::getArea(cs, area, ChannelType::LUMA), ChannelType::LUMA))
            {
              for (auto &currTU: CU::traverseTUs(currCU))
              {
                if (clipLumaIfNoBilat)
                {
                  m_bilateralFilter.clipNotBilaterallyFilteredBlocks(COMP_Y, srcYuv, resYuv, cs.slice->clpRng(COMP_Y),
                                                                     currTU);
                }
              }
            }
          }

          if (cs.pps->m_chromaBIF)
          {
            bool        isDualTree = CS::isDualITree(cs);
            ChannelType chType     = isDualTree ? ChannelType::CHROMA : ChannelType::LUMA;

            for (auto &currCU: cs.traverseCUs(CS::getArea(cs, area, chType), chType))
            {
              bool chromaValid = currCU.Cb().valid() && currCU.Cr().valid();
              if (!chromaValid)
              {
                continue;
              }
              for (auto &currTU: CU::traverseTUs(currCU))
              {
                for (int compIdx = COMP_Cb; compIdx < MAX_NUM_COMP; compIdx++)
                {
                  CompID     compID          = CompID(compIdx);
                  BifParams &chromaBifParams = cs.picture->getBifParam(compID);
                  bool       applyChromaBIF =
                    chromaBifParams.ctuOn[ctuRsAddr] && m_bilateralFilter.getApplyBIF(currTU, compID);
                  if (applyChromaBIF)
                  {
                    bool      clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
                    int       numHorVirBndry = 0, numVerVirBndry = 0;
                    int       horVirBndryPos[] = { 0, 0, 0 };
                    int       verVirBndryPos[] = { 0, 0, 0 };
                    CompArea &myArea           = currTU.block(compID);
                    const int chromaScaleX     = getComponentScaleX(compID, currTU.cu->cs->pcv->chrFormat);
                    const int chromaScaleY     = getComponentScaleY(compID, currTU.cu->cs->pcv->chrFormat);
                    int       yPos             = myArea.y << chromaScaleY;
                    int       xPos             = myArea.x << chromaScaleX;
                    bool      isTUCrossedByVirtualBoundaries = m_bilateralFilter.isCrossedByVirtualBoundaries(
                      cs, xPos, yPos, myArea.width << chromaScaleX, myArea.height << chromaScaleY, clipTop, clipBottom,
                      clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos);

                    m_bilateralFilter.bilateralFilterDiamond5x5(
                      compID, srcYuv, resYuv, currTU.cu->qp, cs.slice->clpRng(compID), currTU, false,
                      isTUCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos, numHorVirBndry, numVerVirBndry,
                      clipTop, clipBottom, clipLeft, clipRight);
                  }
                  else
                  {
                    bool useClip = clipChromaIfNoBilat[compID];
                    if (useClip && currTU.blocks[compIdx].valid())
                    {
                      m_bilateralFilter.clipNotBilaterallyFilteredBlocks(compID, srcYuv, resYuv,
                                                                         cs.slice->clpRng(compID), currTU);
                    }
                  }
                }
              }
            }
          }
          else
          {
            bool        isDualTree = CS::isDualITree(cs);
            ChannelType chType     = isDualTree ? ChannelType::CHROMA : ChannelType::LUMA;

            for (auto &currCU: cs.traverseCUs(CS::getArea(cs, area, chType), chType))
            {
              bool chromaEnabled = isChromaEnabled(currCU.chromaFormat);
              if (!chromaEnabled)
              {
                // TODO: should we skip the entire loop in this case?
                continue;
              }
              bool chromaValid = currCU.Cb().valid() && currCU.Cr().valid();
              if (!chromaValid)
              {
                continue;
              }
              for (auto &currTU: CU::traverseTUs(currCU))
              {
                if (clipChromaIfNoBilat[COMP_Cb] && currTU.blocks[COMP_Cb].valid())
                {
                  m_bilateralFilter.clipNotBilaterallyFilteredBlocks(COMP_Cb, srcYuv, resYuv, cs.slice->clpRng(COMP_Cb),
                                                                     currTU);
                }
                if (clipChromaIfNoBilat[COMP_Cr] && currTU.blocks[COMP_Cr].valid())
                {
                  m_bilateralFilter.clipNotBilaterallyFilteredBlocks(COMP_Cr, srcYuv, resYuv, cs.slice->clpRng(COMP_Cr),
                                                                     currTU);
                }
              }
            }
          }
        }   // BIF = 1 OR CBIF = 1
        else
        {
          offsetCTU(area, srcYuv, resYuv, reconParams[ctuRsAddr], cs);
        }
        ctuRsAddr++;
      }
    }
    // delete memory
    if (!(cs.pps->m_BIF) || !(cs.pps->m_chromaBIF) || !allBlksDisabled)
    {
      for (uint32_t i = 0; i < groupBlkStat.size(); i++)
      {
        delete[] groupBlkStat[i];
      }
      groupBlkStat.clear();
    }
  }
  if (!allBlksDisabled && (totalCost >= 0) &&
      testSAODisableAtPictureLevel)   // SAO has not beneficial in this case - disable it
  {
    for (ctuRsAddr = 0; ctuRsAddr < pcv.sizeInCtus; ctuRsAddr++)
    {
      codedParams[ctuRsAddr].reset();
    }

    for (uint32_t componentIndex = 0; componentIndex < MAX_NUM_COMP; componentIndex++)
    {
      sliceEnabled[componentIndex] = false;
    }
    m_CABACEstimator->getCtx() = SAOCtx(ctxPicStart);

    resYuv.copyFrom(srcYuv);
  }
  disabledRate(cs, reconParams, saoEncodingRate, saoEncodingRateChroma);
}

void EncSampleAdaptiveOffset::disabledRate(CodingStructure &cs, SAOBlkParam *reconParams, const double saoEncodingRate,
                                           const double saoEncodingRateChroma)
{
  if (saoEncodingRate > 0.0)
  {
    const PreCalcValues &pcv = *cs.pcv;

    const uint32_t numberOfComponents = getNumberValidComponents(cs.picture->chromaFormat);

    const int hierPredLayerIdx = cs.slice->m_hierPredLayerIdx;

    int numCtusForSAOOff[MAX_NUM_COMP];

    for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
    {
      numCtusForSAOOff[compIdx] = 0;
      for (int ctuRsAddr = 0; ctuRsAddr < pcv.sizeInCtus; ctuRsAddr++)
      {
        if (reconParams[ctuRsAddr][compIdx].modeIdc == SAOMode::OFF)
        {
          numCtusForSAOOff[compIdx]++;
        }
      }
    }
    if (saoEncodingRateChroma > 0.0)
    {
      for (int compIdx = 0; compIdx < numberOfComponents; compIdx++)
      {
        m_saoDisabledRate[compIdx][hierPredLayerIdx] = (double)numCtusForSAOOff[compIdx] / (double)pcv.sizeInCtus;
      }
    }
    else if (hierPredLayerIdx == 0)
    {
      m_saoDisabledRate[COMP_Y][0] =
        (double)(numCtusForSAOOff[COMP_Y] + numCtusForSAOOff[COMP_Cb] + numCtusForSAOOff[COMP_Cr]) /
        (double)(pcv.sizeInCtus * 3);
    }
  }
}

void EncSampleAdaptiveOffset::getBlkStats(const CompID compIdx, const int channelBitDepth,
                                          StatDataArray &statsDataTypes, Pel *srcBlk, Pel *orgBlk, Pel *bifBlk,
                                          ptrdiff_t bifStride, ptrdiff_t srcStride, ptrdiff_t orgStride, int width,
                                          int height, bool isLeftAvail, bool isRightAvail, bool isAboveAvail,
                                          bool isBelowAvail, bool isAboveLeftAvail, bool isAboveRightAvail,
                                          bool isCalculatePreDeblockSamples, int horVirBndryPos[], int verVirBndryPos[],
                                          int numHorVirBndry, int numVerVirBndry)
{
  int                                    x, y, startX, startY, endX, endY, edgeType, firstLineStartX, firstLineEndX;
  int8_t                                 signLeft, signRight, signDown;
  int64_t                               *diff, *count;
  Pel                                   *srcLine, *orgLine;
  Pel                                   *bifLine;
  const EnumArray<int, SAOModeNewTypes> &skipLinesR = m_skipLinesR[compIdx];
  const EnumArray<int, SAOModeNewTypes> &skipLinesB = m_skipLinesB[compIdx];

  for (const auto typeIdx: { SAOModeNewTypes::EO_0, SAOModeNewTypes::EO_90, SAOModeNewTypes::EO_135,
                             SAOModeNewTypes::EO_45, SAOModeNewTypes::BO })
  {
    SAOStatData &statsData = statsDataTypes[typeIdx];
    statsData.reset();

    srcLine = srcBlk;
    orgLine = orgBlk;
    bifLine = bifBlk;
    diff    = statsData.diff;
    count   = statsData.count;
    switch (typeIdx)
    {
    case SAOModeNewTypes::EO_0:
      {
        diff += 2;
        count += 2;

        endY   = isBelowAvail ? (height - skipLinesB[typeIdx]) : height;
        startX = !isCalculatePreDeblockSamples ? (isLeftAvail ? 0 : 1)
                                               : (isRightAvail ? (width - skipLinesR[typeIdx]) : (width - 1));
        endX   = !isCalculatePreDeblockSamples ? (isRightAvail ? (width - skipLinesR[typeIdx]) : (width - 1))
                                               : (isRightAvail ? width : (width - 1));

        for (y = 0; y < endY; y++)
        {
          signLeft = (int8_t)sgn(srcLine[startX] - srcLine[startX - 1]);
          for (x = startX; x < endX; x++)
          {
            signRight = (int8_t)sgn(srcLine[x] - srcLine[x + 1]);
            edgeType  = signRight + signLeft;
            signLeft  = -signRight;
            diff[edgeType] += (orgLine[x] - bifLine[x]);
            count[edgeType]++;
          }
          srcLine += srcStride;
          orgLine += orgStride;
          bifLine += bifStride;
        }
        if (isCalculatePreDeblockSamples)
        {
          if (isBelowAvail)
          {
            startX = isLeftAvail ? 0 : 1;
            endX   = isRightAvail ? width : (width - 1);

            for (y = 0; y < skipLinesB[typeIdx]; y++)
            {
              signLeft = (int8_t)sgn(srcLine[startX] - srcLine[startX - 1]);
              for (x = startX; x < endX; x++)
              {
                signRight = (int8_t)sgn(srcLine[x] - srcLine[x + 1]);
                edgeType  = signRight + signLeft;
                signLeft  = -signRight;
                diff[edgeType] += (orgLine[x] - bifLine[x]);
                count[edgeType]++;
              }
              srcLine += srcStride;
              orgLine += orgStride;
              bifLine += bifStride;
            }
          }
        }
        break;
      }
    case SAOModeNewTypes::EO_90:
      {
        diff += 2;
        count += 2;
        int8_t *signUpLine = m_signLineBuf1.data();

        startX = (!isCalculatePreDeblockSamples) ? 0 : (isRightAvail ? (width - skipLinesR[typeIdx]) : width);
        startY = isAboveAvail ? 0 : 1;
        endX   = (!isCalculatePreDeblockSamples) ? (isRightAvail ? (width - skipLinesR[typeIdx]) : width) : width;
        endY   = isBelowAvail ? (height - skipLinesB[typeIdx]) : (height - 1);
        if (!isAboveAvail)
        {
          srcLine += srcStride;
          orgLine += orgStride;
          bifLine += bifStride;
        }

        Pel *srcLineAbove = srcLine - srcStride;
        for (x = startX; x < endX; x++)
        {
          signUpLine[x] = (int8_t)sgn(srcLine[x] - srcLineAbove[x]);
        }

        Pel *srcLineBelow;
        for (y = startY; y < endY; y++)
        {
          srcLineBelow = srcLine + srcStride;

          for (x = startX; x < endX; x++)
          {
            signDown      = (int8_t)sgn(srcLine[x] - srcLineBelow[x]);
            edgeType      = signDown + signUpLine[x];
            signUpLine[x] = -signDown;
            diff[edgeType] += (orgLine[x] - bifLine[x]);
            count[edgeType]++;
          }
          srcLine += srcStride;
          orgLine += orgStride;
          bifLine += bifStride;
        }
        if (isCalculatePreDeblockSamples)
        {
          if (isBelowAvail)
          {
            startX = 0;
            endX   = width;

            for (y = 0; y < skipLinesB[typeIdx]; y++)
            {
              srcLineBelow = srcLine + srcStride;
              srcLineAbove = srcLine - srcStride;

              for (x = startX; x < endX; x++)
              {
                edgeType = sgn(srcLine[x] - srcLineBelow[x]) + sgn(srcLine[x] - srcLineAbove[x]);
                diff[edgeType] += (orgLine[x] - bifLine[x]);
                count[edgeType]++;
              }
              srcLine += srcStride;
              orgLine += orgStride;
              bifLine += bifStride;
            }
          }
        }
        break;
      }
    case SAOModeNewTypes::EO_135:
      {
        diff += 2;
        count += 2;
        int8_t *signTmpLine;

        int8_t *signUpLine   = m_signLineBuf1.data();
        int8_t *signDownLine = m_signLineBuf2.data();

        startX = (!isCalculatePreDeblockSamples) ? (isLeftAvail ? 0 : 1)
                                                 : (isRightAvail ? (width - skipLinesR[typeIdx]) : (width - 1));

        endX = (!isCalculatePreDeblockSamples) ? (isRightAvail ? (width - skipLinesR[typeIdx]) : (width - 1))
                                               : (isRightAvail ? width : (width - 1));
        endY = isBelowAvail ? (height - skipLinesB[typeIdx]) : (height - 1);

        // prepare 2nd line's upper sign
        Pel *srcLineBelow = srcLine + srcStride;
        for (x = startX; x < endX + 1; x++)
        {
          signUpLine[x] = (int8_t)sgn(srcLineBelow[x] - srcLine[x - 1]);
        }

        // 1st line
        Pel *srcLineAbove = srcLine - srcStride;
        firstLineStartX   = (!isCalculatePreDeblockSamples) ? (isAboveLeftAvail ? 0 : 1) : startX;
        firstLineEndX     = (!isCalculatePreDeblockSamples) ? (isAboveAvail ? endX : 1) : endX;
        for (x = firstLineStartX; x < firstLineEndX; x++)
        {
          edgeType = sgn(srcLine[x] - srcLineAbove[x - 1]) - signUpLine[x + 1];
          diff[edgeType] += (orgLine[x] - bifLine[x]);
          count[edgeType]++;
        }
        srcLine += srcStride;
        orgLine += orgStride;
        bifLine += bifStride;
        // middle lines
        for (y = 1; y < endY; y++)
        {
          srcLineBelow = srcLine + srcStride;

          for (x = startX; x < endX; x++)
          {
            signDown = (int8_t)sgn(srcLine[x] - srcLineBelow[x + 1]);
            edgeType = signDown + signUpLine[x];
            diff[edgeType] += (orgLine[x] - bifLine[x]);
            count[edgeType]++;

            signDownLine[x + 1] = -signDown;
          }
          signDownLine[startX] = (int8_t)sgn(srcLineBelow[startX] - srcLine[startX - 1]);

          signTmpLine  = signUpLine;
          signUpLine   = signDownLine;
          signDownLine = signTmpLine;

          srcLine += srcStride;
          orgLine += orgStride;
          bifLine += bifStride;
        }
        if (isCalculatePreDeblockSamples)
        {
          if (isBelowAvail)
          {
            startX = isLeftAvail ? 0 : 1;
            endX   = isRightAvail ? width : (width - 1);

            for (y = 0; y < skipLinesB[typeIdx]; y++)
            {
              srcLineBelow = srcLine + srcStride;
              srcLineAbove = srcLine - srcStride;

              for (x = startX; x < endX; x++)
              {
                edgeType = sgn(srcLine[x] - srcLineBelow[x + 1]) + sgn(srcLine[x] - srcLineAbove[x - 1]);
                diff[edgeType] += (orgLine[x] - bifLine[x]);
                count[edgeType]++;
              }
              srcLine += srcStride;
              orgLine += orgStride;
              bifLine += bifStride;
            }
          }
        }
        break;
      }
    case SAOModeNewTypes::EO_45:
      {
        diff += 2;
        count += 2;
        int8_t *signUpLine = m_signLineBuf1.data();

        startX = (!isCalculatePreDeblockSamples) ? (isLeftAvail ? 0 : 1)
                                                 : (isRightAvail ? (width - skipLinesR[typeIdx]) : (width - 1));
        endX   = (!isCalculatePreDeblockSamples) ? (isRightAvail ? (width - skipLinesR[typeIdx]) : (width - 1))
                                                 : (isRightAvail ? width : (width - 1));
        endY   = isBelowAvail ? (height - skipLinesB[typeIdx]) : (height - 1);

        // prepare 2nd line upper sign
        Pel *srcLineBelow = srcLine + srcStride;
        for (x = startX - 1; x < endX; x++)
        {
          signUpLine[x + 1] = (int8_t)sgn(srcLineBelow[x] - srcLine[x + 1]);
        }

        // first line
        Pel *srcLineAbove = srcLine - srcStride;

        firstLineStartX = !isCalculatePreDeblockSamples ? (isAboveAvail ? startX : endX) : startX;
        firstLineEndX   = !isCalculatePreDeblockSamples ? (!isRightAvail && isAboveRightAvail ? width : endX) : endX;

        for (x = firstLineStartX; x < firstLineEndX; x++)
        {
          edgeType = sgn(srcLine[x] - srcLineAbove[x + 1]) - signUpLine[x];
          diff[edgeType] += (orgLine[x] - bifLine[x]);
          count[edgeType]++;
        }

        srcLine += srcStride;
        orgLine += orgStride;
        bifLine += bifStride;

        // middle lines
        for (y = 1; y < endY; y++)
        {
          srcLineBelow = srcLine + srcStride;

          for (x = startX; x < endX; x++)
          {
            signDown = (int8_t)sgn(srcLine[x] - srcLineBelow[x - 1]);
            edgeType = signDown + signUpLine[x + 1];
            diff[edgeType] += (orgLine[x] - bifLine[x]);
            count[edgeType]++;

            signUpLine[x] = -signDown;
          }
          signUpLine[endX] = (int8_t)sgn(srcLineBelow[endX - 1] - srcLine[endX]);
          srcLine += srcStride;
          orgLine += orgStride;
          bifLine += bifStride;
        }
        if (isCalculatePreDeblockSamples)
        {
          if (isBelowAvail)
          {
            startX = isLeftAvail ? 0 : 1;
            endX   = isRightAvail ? width : (width - 1);

            for (y = 0; y < skipLinesB[typeIdx]; y++)
            {
              srcLineBelow = srcLine + srcStride;
              srcLineAbove = srcLine - srcStride;

              for (x = startX; x < endX; x++)
              {
                edgeType = sgn(srcLine[x] - srcLineBelow[x - 1]) + sgn(srcLine[x] - srcLineAbove[x + 1]);
                diff[edgeType] += (orgLine[x] - bifLine[x]);
                count[edgeType]++;
              }
              srcLine += srcStride;
              orgLine += orgStride;
              bifLine += bifStride;
            }
          }
        }
        break;
      }
    case SAOModeNewTypes::BO:
      {
        startX = !isCalculatePreDeblockSamples ? 0 : (isRightAvail ? (width - skipLinesR[typeIdx]) : width);
        endX   = !isCalculatePreDeblockSamples ? (isRightAvail ? (width - skipLinesR[typeIdx]) : width) : width;
        endY   = isBelowAvail ? (height - skipLinesB[typeIdx]) : height;

        const int shiftBits = channelBitDepth - NUM_SAO_BO_CLASSES_LOG2;
        for (y = 0; y < endY; y++)
        {
          for (x = startX; x < endX; x++)
          {
            const int bandIdx = srcLine[x] >> shiftBits;
            diff[bandIdx] += (orgLine[x] - bifLine[x]);
            count[bandIdx]++;
          }
          srcLine += srcStride;
          orgLine += orgStride;
          bifLine += bifStride;
        }
        if (isCalculatePreDeblockSamples)
        {
          if (isBelowAvail)
          {
            startX = 0;
            endX   = width;

            for (y = 0; y < skipLinesB[typeIdx]; y++)
            {
              for (x = startX; x < endX; x++)
              {
                const int bandIdx = srcLine[x] >> shiftBits;
                diff[bandIdx] += (orgLine[x] - bifLine[x]);
                count[bandIdx]++;
              }
              srcLine += srcStride;
              orgLine += orgStride;
              bifLine += bifStride;
            }
          }
        }
        break;
      }
    default:
      THROW("Not a supported SAO type");
      break;
    }
  }
}

void EncSampleAdaptiveOffset::deriveLoopFilterBoundaryAvailability(CodingStructure &cs, const Position &pos,
                                                                   bool &isLeftAvail, bool &isAboveAvail,
                                                                   bool &isAboveLeftAvail) const
{
  bool isLoopFiltAcrossSlicePPS = cs.pps->m_loopFilterAcrossSlicesEnabledFlag;
  bool isLoopFiltAcrossTilePPS  = cs.pps->m_loopFilterAcrossTilesEnabledFlag;

  const int         width       = cs.pcv->maxCUWidth;
  const int         height      = cs.pcv->maxCUHeight;
  const CodingUnit *cuCurr      = cs.getCU(pos, ChannelType::LUMA);
  const CodingUnit *cuLeft      = cs.getCU(pos.offset(-width, 0), ChannelType::LUMA);
  const CodingUnit *cuAbove     = cs.getCU(pos.offset(0, -height), ChannelType::LUMA);
  const CodingUnit *cuAboveLeft = cs.getCU(pos.offset(-width, -height), ChannelType::LUMA);

  if (!isLoopFiltAcrossSlicePPS)
  {
    isLeftAvail      = (cuLeft == nullptr) ? false : CU::isSameSlice(*cuCurr, *cuLeft);
    isAboveAvail     = (cuAbove == nullptr) ? false : CU::isSameSlice(*cuCurr, *cuAbove);
    isAboveLeftAvail = (cuAboveLeft == nullptr) ? false : CU::isSameSlice(*cuCurr, *cuAboveLeft);
  }
  else
  {
    isLeftAvail      = (cuLeft != nullptr);
    isAboveAvail     = (cuAbove != nullptr);
    isAboveLeftAvail = (cuAboveLeft != nullptr);
  }

  if (!isLoopFiltAcrossTilePPS)
  {
    isLeftAvail      = (!isLeftAvail) ? false : CU::isSameTile(*cuCurr, *cuLeft);
    isAboveAvail     = (!isAboveAvail) ? false : CU::isSameTile(*cuCurr, *cuAbove);
    isAboveLeftAvail = (!isAboveLeftAvail) ? false : CU::isSameTile(*cuCurr, *cuAboveLeft);
  }

  const SubPic &curSubPic = cs.pps->getSubPicFromCU(*cuCurr);
  if (!curSubPic.m_loopFilterAcrossSubPicEnabledFlag)
  {
    isLeftAvail      = (!isLeftAvail) ? false : CU::isSameSubPic(*cuCurr, *cuLeft);
    isAboveAvail     = (!isAboveAvail) ? false : CU::isSameSubPic(*cuCurr, *cuAbove);
    isAboveLeftAvail = (!isAboveLeftAvail) ? false : CU::isSameSubPic(*cuCurr, *cuAboveLeft);
  }
}

void EncSampleAdaptiveOffset::CCSAOProcess(CodingStructure &cs, const double *lambdas, const int intraPeriod,
                                           const int ccsao_mode)
{
  PROFILER_SCOPE(1, g_timeProfiler, P_SAO);
  if (cs.slice->isIDRorBLA() || cs.slice->m_pendingRasInit)
  {
    m_ccSaoPrvParamEnc[COMP_Y].clear();
    m_ccSaoPrvParamEnc[COMP_Cb].clear();
    m_ccSaoPrvParamEnc[COMP_Cr].clear();
  }

  m_CCSaoMode       = ccsao_mode;
  PelUnitBuf orgYuv = cs.getOrgBuf();
  PelUnitBuf dstYuv = cs.getRecoBuf();
  PelUnitBuf srcYuv = m_ccSaoBuf.getBuf(cs.area);
  srcYuv.extendBorderPel(MAX_CCSAO_FILTER_LENGTH >> 1);
  m_intraPeriod = intraPeriod;

  setupCcSaoLambdas(cs, lambdas);
  setupCcSaoSH(cs, orgYuv);

  const TempCtx ctxStartCcSao(m_ctxPool, SubCtx(Ctx::CcSaoControlIdc, m_CABACEstimator->getCtx()));
  for (int compIdx = COMP_Y; compIdx < MAX_NUM_COMP; compIdx++)
  {
    CompID compID = (CompID)compIdx;
    resetCcSaoEdgeStats(m_ccSaoStatDataEdgePre);
    prepareCcSaoEdgeStats(cs, compID, orgYuv, srcYuv, dstYuv, m_ccSaoStatDataEdgePre, m_bestCcSaoParam);
    m_CABACEstimator->getCtx() = SubCtx(Ctx::CcSaoControlIdc, ctxStartCcSao);
    deriveCcSao(cs, compID, orgYuv, srcYuv, dstYuv);
  }
  setupCcSaoPrv(cs);
  applyCcSao(cs, *cs.pcv, srcYuv, dstYuv);

  DTRACE(g_trace_ctx, D_CRC, "CCSAO");
  DTRACE_CRC(g_trace_ctx, D_CRC, cs, cs.getRecoBuf());
}

void EncSampleAdaptiveOffset::setupCcSaoLambdas(CodingStructure &cs, const double *lambdas)
{
  m_lambda[COMP_Y]  = m_picWidth * m_picHeight <= 416 * 240 ? lambdas[COMP_Y] * 4.0 : lambdas[COMP_Y];
  m_lambda[COMP_Cb] = lambdas[COMP_Cb];
  m_lambda[COMP_Cr] = lambdas[COMP_Cr];
}

void EncSampleAdaptiveOffset::setupCcSaoSH(CodingStructure &cs, const CPelUnitBuf &orgYuv)
{
  Position topLeftLuma = Position(0, 0);
  Size     sizeLuma    = cs.area.lumaSize();

  if (isChromaEnabled(cs.picture->chromaFormat))
  {
    const CompArea yArea  = CompArea(COMP_Y, cs.picture->chromaFormat, Area(topLeftLuma, sizeLuma), true);
    const CompArea cbArea = CompArea(COMP_Cb, cs.picture->chromaFormat, Area(topLeftLuma, sizeLuma), true);
    const CompArea crArea = CompArea(COMP_Cr, cs.picture->chromaFormat, Area(topLeftLuma, sizeLuma), true);
    const CPelBuf  orgY   = cs.picture->getOrigBuf(yArea);
    const CPelBuf  orgCb  = cs.picture->getOrigBuf(cbArea);
    const CPelBuf  orgCr  = cs.picture->getOrigBuf(crArea);

    const int       m0  = (yArea.x > 0 ? 0 : 1);
    const int       n0  = (yArea.y > 0 ? 0 : 1);
    const int       m1  = (yArea.x + yArea.width < cs.picture->lwidth() ? yArea.width : yArea.width - 1);
    const int       n1  = (yArea.y + yArea.height < cs.picture->lheight() ? yArea.height : yArea.height - 1);
    const int       x0  = (cbArea.x > 0 ? 0 : 1);
    const int       y0  = (cbArea.y > 0 ? 0 : 1);
    const int       x1  = (cbArea.x + cbArea.width < cs.picture->Cb().width ? cbArea.width : cbArea.width - 1);
    const int       y1  = (cbArea.y + cbArea.height < cs.picture->Cb().height ? cbArea.height : cbArea.height - 1);
    const ptrdiff_t ys  = orgY.stride;
    const ptrdiff_t cbs = orgCb.stride;
    const ptrdiff_t crs = orgCr.stride;
    const Pel      *pY  = orgY.buf + n0 * ys;
    const Pel      *pCb = orgCb.buf + y0 * cbs;
    const Pel      *pCr = orgCr.buf + y0 * crs;

    int    absSumY  = 0;
    int    absSumCb = 0;
    int    absSumCr = 0;
    double avgY     = 0;
    double avgCb    = 0;
    double avgCr    = 0;

    for (int n = n0; n < n1; n++, pY += ys)
    {
      for (int m = m0; m < m1; m++)
      {
        int y = (12 * (int)pY[m] - 2 * ((int)pY[m - 1] + (int)pY[m + 1] + (int)pY[m - ys] + (int)pY[m + ys]) -
                 ((int)pY[m - 1 - ys] + (int)pY[m + 1 - ys] + (int)pY[m - 1 + ys] + (int)pY[m + 1 + ys]));
        absSumY += abs(y);
      }
    }

    for (int y = y0; y < y1; y++, pCb += cbs, pCr += crs)
    {
      for (int x = x0; x < x1; x++)
      {
        int cb = (12 * (int)pCb[x] - 2 * ((int)pCb[x - 1] + (int)pCb[x + 1] + (int)pCb[x - cbs] + (int)pCb[x + cbs]) -
                  ((int)pCb[x - 1 - cbs] + (int)pCb[x + 1 - cbs] + (int)pCb[x - 1 + cbs] + (int)pCb[x + 1 + cbs]));
        int cr = (12 * (int)pCr[x] - 2 * ((int)pCr[x - 1] + (int)pCr[x + 1] + (int)pCr[x - crs] + (int)pCr[x + crs]) -
                  ((int)pCr[x - 1 - crs] + (int)pCr[x + 1 - crs] + (int)pCr[x - 1 + crs] + (int)pCr[x + 1 + crs]));
        absSumCb += abs(cb);
        absSumCr += abs(cr);
      }
    }

    avgY        = (double)absSumY / (yArea.width * yArea.height);
    avgCb       = (double)absSumCb / (cbArea.width * cbArea.height);
    avgCr       = (double)absSumCr / (crArea.width * crArea.height);
    m_extChroma = 1.2 * avgCb > avgY || 1.2 * avgCr > avgY;
  }
}

void EncSampleAdaptiveOffset::deriveCcSao(CodingStructure &cs, const CompID compID, const CPelUnitBuf &orgYuv,
                                          const CPelUnitBuf &srcYuv, const CPelUnitBuf &dstYuv)
{
  double bestCost                         = 0;
  double tempCost                         = 0;
  double bestCostS[MAX_CCSAO_SET_NUM + 1] = { 0 };

  double bestCostG[17] = { 0 };
  int    classNumG[17] = { 0 };
  int    stageNum      = m_intraPeriod == 1 ? MAX_CCSAO_CLASS_NUM / 4 : MAX_CCSAO_CLASS_NUM / 16;
  for (int stage = 1; stage <= stageNum; stage++)
  {
    classNumG[stage] = stage * (MAX_CCSAO_CLASS_NUM / stageNum);
  }

  const int edgeCmpNum = m_extChroma ? MAX_NUM_COMP : MAX_NUM_LUMA_COMP;
  const int edgeIdcNum =
    (m_CCSaoMode != 2) && (m_intraPeriod != 1 || cs.sps->m_PLTMode || m_extChroma) ? MAX_CCSAO_EDGE_IDC : 1;

  m_bestCcSaoParam.reset();
  memset(m_bestCcSaoControl, 0, sizeof(uint8_t) * m_numCTUsInPic);

  for (int setNum = 1; setNum <= MAX_CCSAO_SET_NUM; setNum++)
  {
    if (setNum > 1)
    {
      getCcSaoStatistics(cs, compID, orgYuv, srcYuv, dstYuv, m_ccSaoStatData, m_bestCcSaoParam);
      getCcSaoStatisticsEdge(cs, compID, orgYuv, srcYuv, dstYuv, m_ccSaoStatDataEdge, m_ccSaoStatDataEdgePre,
                             m_bestCcSaoParam);
    }
    setupInitCcSaoParam(cs, compID, setNum, m_trainingDistortion, m_ccSaoStatData, m_ccSaoStatFrame,
                        m_ccSaoStatDataEdge, m_ccSaoStatFrameEdge, m_initCcSaoParam, m_bestCcSaoParam,
                        m_initCcSaoControl, m_bestCcSaoControl);

    for (int stage = 1; stage <= stageNum; stage++)
    {
      for (int bandNumY = 1; bandNumY <= MAX_CCSAO_BAND_NUM_Y; bandNumY++)
      {
        for (int bandNumU = 1; bandNumU <= MAX_CCSAO_BAND_NUM_U; bandNumU++)
        {
          for (int bandNumV = 1; bandNumV <= MAX_CCSAO_BAND_NUM_V; bandNumV++)
          {
            for (int candPosY = 0; candPosY < MAX_CCSAO_CAND_POS_Y && bandNumY > 1; candPosY++)
            {
              if (bandNumY < bandNumU || bandNumY < bandNumV)
              {
                continue;
              }

              int classNum = bandNumY * bandNumU * bandNumV;
              if (classNum > MAX_CCSAO_CLASS_NUM)
              {
                continue;
              }

              if (classNum <= classNumG[stage - 1] || classNum > classNumG[stage])
              {
                continue;
              }

              setupTempCcSaoParam(cs, compID, setNum, 0 /*dummy*/, candPosY, bandNumY, bandNumU, bandNumV,
                                  m_tempCcSaoParam, m_initCcSaoParam, m_tempCcSaoControl,
                                  m_initCcSaoControl /*, 0 (default Band type*/);
              getCcSaoStatistics(cs, compID, orgYuv, srcYuv, dstYuv, m_ccSaoStatData, m_tempCcSaoParam);
              deriveCcSaoRDO(cs, compID, m_trainingDistortion, m_ccSaoStatData, m_ccSaoStatFrame, m_ccSaoStatDataEdge,
                             m_ccSaoStatFrameEdge, m_bestCcSaoParam, m_tempCcSaoParam, m_bestCcSaoControl,
                             m_tempCcSaoControl, bestCost, tempCost);
            }
          }
        }
      }

      bestCostG[stage] = bestCost;
      if (bestCostG[stage] >= bestCostG[stage - 1])
      {
        break;
      }
    }

    int NUM_CCSAO_EDGE_THR = (m_CCSaoMode != 2) ? MAX_CCSAO_EDGE_THR : 4;

    for (int edgeCmp = COMP_Y; edgeCmp < edgeCmpNum; edgeCmp++)
    {
      for (int edgeDir = 0; edgeDir < MAX_CCSAO_EDGE_DIR; edgeDir++)
      {
        for (int edgeIdc = 0; edgeIdc < edgeIdcNum; edgeIdc++)
        {
          for (int bandIdc = 0; bandIdc < MAX_CCSAO_BAND_IDC; bandIdc++)
          {
            for (int edgeThr = 0; edgeThr < NUM_CCSAO_EDGE_THR; edgeThr++)
            {
              const int edgeNum = g_ccSaoEdgeNum[edgeIdc][0];
              const int bandNum = g_ccSaoBandTab[bandIdc][1];
              if (bandNum * edgeNum > MAX_CCSAO_CLASS_NUM)
              {
                continue;
              }

              setupTempCcSaoParam(cs, compID, setNum, edgeCmp, edgeDir, bandIdc, edgeThr, edgeIdc, m_tempCcSaoParam,
                                  m_initCcSaoParam, m_tempCcSaoControl, m_initCcSaoControl, CCSAO_SET_TYPE_EDGE);
              getCcSaoStatisticsEdge(cs, compID, orgYuv, srcYuv, dstYuv, m_ccSaoStatDataEdge, m_ccSaoStatDataEdgePre,
                                     m_tempCcSaoParam);
              deriveCcSaoRDO(cs, compID, m_trainingDistortion, m_ccSaoStatData, m_ccSaoStatFrame, m_ccSaoStatDataEdge,
                             m_ccSaoStatFrameEdge, m_bestCcSaoParam, m_tempCcSaoParam, m_bestCcSaoControl,
                             m_tempCcSaoControl, bestCost, tempCost);
            }
          }
        }
      }
    }

    bestCostS[setNum] = bestCost;
    if (bestCostS[setNum] >= bestCostS[setNum - 1])
    {
      break;
    }
  }

  if (!cs.slice->isIntra())
  {
    for (int prvId = 0; prvId < m_ccSaoPrvParamEnc[compID].size(); prvId++)
    {
      if (m_ccSaoPrvParamEnc[compID][prvId].temporalId > cs.slice->m_uiTLayer)
      {
        continue;
      }

      setupTempCcSaoParamFromPrv(cs, compID, prvId, m_tempCcSaoParam, m_ccSaoPrvParamEnc[compID][prvId],
                                 m_tempCcSaoControl);

      getCcSaoStatistics(cs, compID, orgYuv, srcYuv, dstYuv, m_ccSaoStatData, m_tempCcSaoParam);
      getCcSaoStatisticsEdge(cs, compID, orgYuv, srcYuv, dstYuv, m_ccSaoStatDataEdge, m_ccSaoStatDataEdgePre,
                             m_tempCcSaoParam);

      deriveCcSaoRDO(cs, compID, m_trainingDistortion, m_ccSaoStatData, m_ccSaoStatFrame, m_ccSaoStatDataEdge,
                     m_ccSaoStatFrameEdge, m_bestCcSaoParam, m_tempCcSaoParam, m_bestCcSaoControl, m_tempCcSaoControl,
                     bestCost, tempCost);
    }
  }

  bool oneBlockFiltered = false;
  for (int ctbIdx = 0; m_bestCcSaoParam.setNum > 0 && ctbIdx < m_numCTUsInPic; ctbIdx++)
  {
    if (m_bestCcSaoControl[ctbIdx])
    {
      oneBlockFiltered = true;
      break;
    }
  }

  m_ccSaoComParam.reset(compID);
  memset(m_ccSaoControl[compID], 0, sizeof(uint8_t) * m_numCTUsInPic);

  m_ccSaoComParam.enabled[compID]   = oneBlockFiltered;
  m_ccSaoComParam.extChroma[compID] = m_extChroma;
  if (oneBlockFiltered)
  {
    CcSaoEncParam storedBestCcSaoParam = m_bestCcSaoParam;
    memcpy(m_tempCcSaoControl, m_bestCcSaoControl, sizeof(uint8_t) * m_numCTUsInPic);

    int setNum = 0;
    for (int setIdx = 0; setIdx < MAX_CCSAO_SET_NUM; setIdx++)
    {
      uint8_t setIdc = m_bestCcSaoParam.mapIdxToIdc[setIdx];
      if (m_bestCcSaoParam.setEnabled[setIdx])
      {
        for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
        {
          if (m_tempCcSaoControl[ctbIdx] == (setIdx + 1))
          {
            m_bestCcSaoControl[ctbIdx] = setIdc;
          }
        }
        m_bestCcSaoParam.setType[setIdc - 1]          = storedBestCcSaoParam.setType[setIdx];
        m_bestCcSaoParam.candPos[setIdc - 1][COMP_Cb] = storedBestCcSaoParam.candPos[setIdx][COMP_Cb];
        m_bestCcSaoParam.candPos[setIdc - 1][COMP_Y]  = storedBestCcSaoParam.candPos[setIdx][COMP_Y];
        m_bestCcSaoParam.bandNum[setIdc - 1][COMP_Y]  = storedBestCcSaoParam.bandNum[setIdx][COMP_Y];
        m_bestCcSaoParam.bandNum[setIdc - 1][COMP_Cb] = storedBestCcSaoParam.bandNum[setIdx][COMP_Cb];
        m_bestCcSaoParam.bandNum[setIdc - 1][COMP_Cr] = storedBestCcSaoParam.bandNum[setIdx][COMP_Cr];
        memcpy(m_bestCcSaoParam.offset[setIdc - 1], storedBestCcSaoParam.offset[setIdx],
               sizeof(storedBestCcSaoParam.offset[setIdx]));
        setNum++;
      }
      m_bestCcSaoParam.setEnabled[setIdx] = setIdx < m_bestCcSaoParam.setNum ? true : false;
    }
    CHECK(setNum != m_bestCcSaoParam.setNum, "Number of sets enabled != setNum");

    m_ccSaoComParam.reusePrv[compID]   = m_bestCcSaoParam.reusePrv;
    m_ccSaoComParam.reusePrvId[compID] = m_bestCcSaoParam.reusePrvId;
    m_ccSaoComParam.setNum[compID]     = m_bestCcSaoParam.setNum;

    for (int setIdx = 0; setIdx < m_bestCcSaoParam.setNum; setIdx++)
    {
      m_ccSaoComParam.setEnabled[compID][setIdx]       = m_bestCcSaoParam.setEnabled[setIdx];
      m_ccSaoComParam.setType[compID][setIdx]          = m_bestCcSaoParam.setType[setIdx];
      m_ccSaoComParam.candPos[compID][setIdx][COMP_Cb] = m_bestCcSaoParam.candPos[setIdx][COMP_Cb];
      m_ccSaoComParam.candPos[compID][setIdx][COMP_Y]  = m_bestCcSaoParam.candPos[setIdx][COMP_Y];
      m_ccSaoComParam.bandNum[compID][setIdx][COMP_Y]  = m_bestCcSaoParam.bandNum[setIdx][COMP_Y];
      m_ccSaoComParam.bandNum[compID][setIdx][COMP_Cb] = m_bestCcSaoParam.bandNum[setIdx][COMP_Cb];

      m_ccSaoComParam.bandNum[compID][setIdx][COMP_Cr] = m_bestCcSaoParam.bandNum[setIdx][COMP_Cr];
      memcpy(m_ccSaoComParam.offset[compID][setIdx], m_bestCcSaoParam.offset[setIdx],
             sizeof(m_bestCcSaoParam.offset[setIdx]));
    }
    memcpy(m_ccSaoControl[compID], m_bestCcSaoControl, sizeof(uint8_t) * m_numCTUsInPic);
  }
}

void EncSampleAdaptiveOffset::setupInitCcSaoParam(
  CodingStructure &cs, const CompID compID, const int setNum, int64_t *trainingDistortion[MAX_CCSAO_SET_NUM],
  std::vector<CcSaoStatData> (&blkStats)[MAX_CCSAO_SET_NUM], CcSaoStatData     frameStats[MAX_CCSAO_SET_NUM],
  std::vector<CcSaoStatData> (&blkStatsEdge)[MAX_CCSAO_SET_NUM], CcSaoStatData frameStatsEdge[MAX_CCSAO_SET_NUM],
  CcSaoEncParam &initCcSaoParam, CcSaoEncParam &bestCcSaoParam, uint8_t *initCcSaoControl, uint8_t *bestCcSaoControl)
{
  initCcSaoParam.reset();
  memset(initCcSaoControl, 0, sizeof(uint8_t) * m_numCTUsInPic);

  if (setNum == 1)
  {
    std::fill_n(initCcSaoControl, m_numCTUsInPic, 1);
    return;
  }

  for (int setIdx = 0; setIdx < MAX_CCSAO_SET_NUM; setIdx++)
  {
    if (bestCcSaoParam.setEnabled[setIdx])
    {
      getCcSaoFrameStats(compID, setIdx, bestCcSaoControl, blkStats, frameStats, blkStatsEdge, frameStatsEdge,
                         bestCcSaoParam.setType[setIdx]);
      if (bestCcSaoParam.setType[setIdx] == CCSAO_SET_TYPE_BAND)
      {
        getCcSaoDistortion(compID, setIdx, blkStats, bestCcSaoParam.offset, trainingDistortion);
      }
      else
      {
        getCcSaoDistortion(compID, setIdx, blkStatsEdge, bestCcSaoParam.offset, trainingDistortion);
      }
    }
  }

  initCcSaoParam = bestCcSaoParam;

  int      ctbCntOn = 0;
  CtbCost *ctbCost  = new CtbCost[m_numCTUsInPic];

  for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
  {
    int64_t dist = 0;

    if (bestCcSaoControl[ctbIdx])
    {
      int setIdx = bestCcSaoControl[ctbIdx] - 1;
      dist       = trainingDistortion[setIdx][ctbIdx];
      ctbCntOn++;
    }

    ctbCost[ctbIdx].pos  = ctbIdx;
    ctbCost[ctbIdx].cost = (double)dist;
  }

  std::stable_sort(ctbCost, ctbCost + m_numCTUsInPic, compareCtbCost);

  for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
  {
    int ctbPos = ctbCost[ctbIdx].pos;
    if (ctbIdx < ctbCntOn)
    {
      if (ctbIdx * 2 > ctbCntOn)
      {
        initCcSaoControl[ctbPos] = setNum;
      }
      else
      {
        initCcSaoControl[ctbPos] = bestCcSaoControl[ctbPos];
      }
    }
    else
    {
      initCcSaoControl[ctbPos] = 0;
    }
  }

  delete[] ctbCost;
  ctbCost = nullptr;
}

void EncSampleAdaptiveOffset::setupTempCcSaoParam(CodingStructure &cs, const CompID compID, const int setNum,
                                                  const int edgeCmp, const int candPosY, const int bandNumY,
                                                  const int bandNumU, const int bandNumV, CcSaoEncParam &tempCcSaoParam,
                                                  CcSaoEncParam &initCcSaoParam, uint8_t *tempCcSaoControl,
                                                  uint8_t *initCcSaoControl, int setType)
{
  tempCcSaoParam.reset();
  memset(tempCcSaoControl, 0, sizeof(uint8_t) * m_numCTUsInPic);

  tempCcSaoParam = initCcSaoParam;
  memcpy(tempCcSaoControl, initCcSaoControl, sizeof(uint8_t) * m_numCTUsInPic);

  tempCcSaoParam.setNum                       = setNum;
  tempCcSaoParam.setEnabled[setNum - 1]       = true;
  tempCcSaoParam.setType[setNum - 1]          = setType;
  tempCcSaoParam.candPos[setNum - 1][COMP_Cb] = edgeCmp;
  tempCcSaoParam.candPos[setNum - 1][COMP_Y]  = candPosY;
  tempCcSaoParam.bandNum[setNum - 1][COMP_Y]  = bandNumY;
  tempCcSaoParam.bandNum[setNum - 1][COMP_Cb] = bandNumU;
  tempCcSaoParam.bandNum[setNum - 1][COMP_Cr] = bandNumV;

  CHECK(setNum > MAX_CCSAO_SET_NUM, "setNum exceeds the buffer size");

  for (int setIdx = 0; setIdx <= setNum; setIdx++)
  {
    tempCcSaoParam.mapIdxToIdc[setIdx] = setIdx < setNum ? setIdx + 1 : 0;
  }
}

void EncSampleAdaptiveOffset::setupTempCcSaoParamFromPrv(CodingStructure &cs, const CompID compID, const int prvId,
                                                         CcSaoEncParam &tempCcSaoParam, CcSaoPrvParam &prvCcSaoParam,
                                                         uint8_t *tempCcSaoControl)
{
  tempCcSaoParam.reset();
  memset(tempCcSaoControl, 0, sizeof(uint8_t) * m_numCTUsInPic);
  std::fill_n(tempCcSaoControl, m_numCTUsInPic, 1);

  tempCcSaoParam.reusePrv   = true;
  tempCcSaoParam.reusePrvId = prvId;
  tempCcSaoParam.setNum     = prvCcSaoParam.setNum;
  memcpy(tempCcSaoParam.setEnabled, prvCcSaoParam.setEnabled, sizeof(tempCcSaoParam.setEnabled));
  memcpy(tempCcSaoParam.setType, prvCcSaoParam.setType, sizeof(tempCcSaoParam.setType));
  memcpy(tempCcSaoParam.candPos, prvCcSaoParam.candPos, sizeof(tempCcSaoParam.candPos));
  memcpy(tempCcSaoParam.bandNum, prvCcSaoParam.bandNum, sizeof(tempCcSaoParam.bandNum));
  memcpy(tempCcSaoParam.offset, prvCcSaoParam.offset, sizeof(tempCcSaoParam.offset));

  CHECK(tempCcSaoParam.setNum > MAX_CCSAO_SET_NUM, "setNum exceeds the buffer size");

  for (int setIdx = 0; setIdx <= tempCcSaoParam.setNum; setIdx++)
  {
    tempCcSaoParam.mapIdxToIdc[setIdx] = setIdx < tempCcSaoParam.setNum ? setIdx + 1 : 0;
  }
}

void EncSampleAdaptiveOffset::getCcSaoStatistics(CodingStructure &cs, const CompID compID, const CPelUnitBuf &orgYuv,
                                                 const CPelUnitBuf &srcYuv, const CPelUnitBuf &dstYuv,
                                                 std::vector<CcSaoStatData> (&blkStats)[MAX_CCSAO_SET_NUM],
                                                 const CcSaoEncParam &ccSaoParam)
{
  bool isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail;

  const PreCalcValues &pcv = *cs.pcv;

  int ctuRsAddr = 0;
  for (uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
  {
    for (uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
    {
      const uint32_t width  = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
      const uint32_t height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
      const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));

      deriveLoopFilterBoundaryAvailability(cs, area.Y(), isLeftAvail, isAboveAvail, isAboveLeftAvail);

      // NOTE: The number of skipped lines during gathering CTU statistics depends on the slice boundary availabilities.
      // For simplicity, here only picture boundaries are considered.

      isRightAvail      = (xPos + pcv.maxCUWidth < pcv.lumaWidth);
      isBelowAvail      = (yPos + pcv.maxCUHeight < pcv.lumaHeight);
      isAboveRightAvail = ((yPos > 0) && (isRightAvail));

      for (int setIdx = 0; setIdx < MAX_CCSAO_SET_NUM; setIdx++)
      {
        blkStats[setIdx][ctuRsAddr].reset();
        if (!ccSaoParam.setEnabled[setIdx])
        {
          continue;
        }
        if (ccSaoParam.setType[setIdx] != CCSAO_SET_TYPE_BAND)
        {
          continue;
        }
        const CompArea &compArea   = area.block(compID);
        const ptrdiff_t srcStrideY = srcYuv.get(COMP_Y).stride;
        const ptrdiff_t srcStrideU = srcYuv.get(COMP_Cb).stride;
        const ptrdiff_t srcStrideV = srcYuv.get(COMP_Cr).stride;
        const Pel      *srcBlkY    = srcYuv.get(COMP_Y).bufAt(area.block(COMP_Y));
        const Pel      *srcBlkU    = srcYuv.get(COMP_Cb).bufAt(area.block(COMP_Cb));
        const Pel      *srcBlkV    = srcYuv.get(COMP_Cr).bufAt(area.block(COMP_Cr));
        const ptrdiff_t dstStride  = dstYuv.get(compID).stride;
        const ptrdiff_t orgStride  = orgYuv.get(compID).stride;
        const Pel      *dstBlk     = dstYuv.get(compID).bufAt(compArea);
        const Pel      *orgBlk     = orgYuv.get(compID).bufAt(compArea);

        const uint16_t candPosY = ccSaoParam.candPos[setIdx][COMP_Y];
        const uint16_t bandNumY = ccSaoParam.bandNum[setIdx][COMP_Y];
        const uint16_t bandNumU = ccSaoParam.bandNum[setIdx][COMP_Cb];
        const uint16_t bandNumV = ccSaoParam.bandNum[setIdx][COMP_Cr];

        getCcSaoBlkStats(compID, cs.area.chromaFormat, cs.sps->m_bitDepths[toChannelType(compID)], setIdx, blkStats,
                         ctuRsAddr, candPosY, bandNumY, bandNumU, bandNumV, srcBlkY, srcBlkU, srcBlkV, orgBlk, dstBlk,
                         srcStrideY, srcStrideU, srcStrideV, orgStride, dstStride, compArea.width, compArea.height,
                         isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail);
      }
      ctuRsAddr++;
    }
  }
}
void EncSampleAdaptiveOffset::resetCcSaoEdgeStats(std::vector<CcSaoStatData>(&blkStatsEdge))
{
  int numStatsEdge = (m_CCSaoMode != 2)
    ? m_numCTUsInPic * MAX_CCSAO_BAND_IDC * MAX_CCSAO_EDGE_DIR * MAX_CCSAO_EDGE_THR * MAX_NUM_COMP * MAX_CCSAO_EDGE_IDC
    : m_numCTUsInPic * MAX_CCSAO_BAND_IDC * MAX_CCSAO_EDGE_DIR * 4 * MAX_NUM_COMP * MAX_CCSAO_EDGE_IDC;
  for (int idx = 0; idx < numStatsEdge; idx++)
  {
    blkStatsEdge[idx].reset();
  }
}
void EncSampleAdaptiveOffset::prepareCcSaoEdgeStats(CodingStructure &cs, const CompID compID, const CPelUnitBuf &orgYuv,
                                                    const CPelUnitBuf &srcYuv, const CPelUnitBuf &dstYuv,
                                                    std::vector<CcSaoStatData>(&blkStatsEdge),
                                                    const CcSaoEncParam &ccSaoParam)
{
  bool isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail;

  const PreCalcValues &pcv = *cs.pcv;

  int ctuRsAddr = 0;
  for (uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
  {
    for (uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
    {
      const uint32_t width  = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
      const uint32_t height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
      const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));

      deriveLoopFilterBoundaryAvailability(cs, area.Y(), isLeftAvail, isAboveAvail, isAboveLeftAvail);

      // NOTE: The number of skipped lines during gathering CTU statistics depends on the slice boundary availabilities.
      // For simplicity, here only picture boundaries are considered.

      isRightAvail      = (xPos + pcv.maxCUWidth < pcv.lumaWidth);
      isBelowAvail      = (yPos + pcv.maxCUHeight < pcv.lumaHeight);
      isAboveRightAvail = ((yPos > 0) && (isRightAvail));

      const CompArea &compArea   = area.block(compID);
      const ptrdiff_t srcStrideY = srcYuv.get(COMP_Y).stride;
      const ptrdiff_t srcStrideU = srcYuv.get(COMP_Cb).stride;
      const ptrdiff_t srcStrideV = srcYuv.get(COMP_Cr).stride;
      const Pel      *srcBlkY    = srcYuv.get(COMP_Y).bufAt(area.block(COMP_Y));
      const Pel      *srcBlkU    = srcYuv.get(COMP_Cb).bufAt(area.block(COMP_Cb));
      const Pel      *srcBlkV    = srcYuv.get(COMP_Cr).bufAt(area.block(COMP_Cr));
      const ptrdiff_t dstStride  = dstYuv.get(compID).stride;
      const ptrdiff_t orgStride  = orgYuv.get(compID).stride;
      const Pel      *dstBlk     = dstYuv.get(compID).bufAt(compArea);
      const Pel      *orgBlk     = orgYuv.get(compID).bufAt(compArea);

      getCcSaoBlkStatsEdgePre(cs, compID, cs.area.chromaFormat, cs.sps->m_bitDepths[toChannelType(compID)],
                              m_ccSaoStatDataEdgePre, ctuRsAddr, srcBlkY, srcBlkU, srcBlkV, orgBlk, dstBlk, srcStrideY,
                              srcStrideU, srcStrideV, orgStride, dstStride, compArea.width, compArea.height,
                              isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail,
                              isAboveRightAvail);
      ctuRsAddr++;
    }
  }
}

void EncSampleAdaptiveOffset::getCcSaoStatisticsEdge(CodingStructure &cs, const CompID compID,
                                                     const CPelUnitBuf &orgYuv, const CPelUnitBuf &srcYuv,
                                                     const CPelUnitBuf &dstYuv,
                                                     std::vector<CcSaoStatData> (&blkStatsEdge)[MAX_CCSAO_SET_NUM],
                                                     std::vector<CcSaoStatData>(&blkStatsEdgePre),
                                                     const CcSaoEncParam &ccSaoParam)
{
  const PreCalcValues &pcv = *cs.pcv;

  int ctuRsAddr = 0;
  for (uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
  {
    for (uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
    {
      const uint32_t width  = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
      const uint32_t height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
      const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));

      for (int setIdx = 0; setIdx < MAX_CCSAO_SET_NUM; setIdx++)
      {
        blkStatsEdge[setIdx][ctuRsAddr].reset();
        if (!ccSaoParam.setEnabled[setIdx])
        {
          continue;
        }
        if (ccSaoParam.setType[setIdx] != CCSAO_SET_TYPE_EDGE)
        {
          continue;
        }

        const uint16_t edgeCmp = ccSaoParam.candPos[setIdx][COMP_Cb];
        const uint16_t edgeIdc = ccSaoParam.bandNum[setIdx][COMP_Cr];
        const uint16_t edgeDir = ccSaoParam.candPos[setIdx][COMP_Y];
        const uint16_t edgeThr = ccSaoParam.bandNum[setIdx][COMP_Cb];
        const uint16_t bandIdc = ccSaoParam.bandNum[setIdx][COMP_Y];

        int idx                         = getCcSaoEdgeStatIdx(ctuRsAddr, bandIdc, edgeCmp, edgeIdc, edgeDir, edgeThr);
        blkStatsEdge[setIdx][ctuRsAddr] = blkStatsEdgePre[idx];
      }
      ctuRsAddr++;
    }
  }
}

inline int EncSampleAdaptiveOffset::getCcSaoEdgeStatIdx(const int ctuRsAddr, const int bandIdc, const int edgeCmp,
                                                        const int edgeIdc, int edgeDir, const int edgeThr)
{
  int idx = (m_CCSaoMode != 2) ? ctuRsAddr * MAX_CCSAO_BAND_IDC * MAX_CCSAO_EDGE_DIR * MAX_CCSAO_EDGE_THR +
      bandIdc * MAX_CCSAO_EDGE_DIR * MAX_CCSAO_EDGE_THR + edgeDir * MAX_CCSAO_EDGE_THR + edgeThr
                               : ctuRsAddr * MAX_CCSAO_BAND_IDC * MAX_CCSAO_EDGE_DIR * 4 +
      bandIdc * MAX_CCSAO_EDGE_DIR * 4 + edgeDir * 4 + edgeThr;

  idx = idx * MAX_NUM_COMP + edgeCmp;
  idx = idx * MAX_CCSAO_EDGE_IDC + edgeIdc;

  return idx;
}

inline void EncSampleAdaptiveOffset::getCcSaomSampleStatsEdgePre(
  int startx, int endx, const ptrdiff_t srcStrideE, const int edgeIdcNum, const int bitDepth, const int edgePosXA,
  const int edgePosXB, const int edgePosYA, const int edgePosYB, std::vector<CcSaoStatData>(&blkStatsEdge),
  const int ctuRsAddr, const Pel *srcY, const Pel *srcU, const Pel *srcV, const int chromaScaleX, int edgeCmp,
  SAOModeNewTypes edgeDir, const Pel *org, const Pel *dst)
{
  int NUM_CCSAO_EDGE_THR = (m_CCSaoMode != 2) ? MAX_CCSAO_EDGE_THR : 4;

  for (int x = startx; x < endx; x++)
  {
    const Pel *colY              = srcY + x;
    const Pel *colU              = srcU + (x >> chromaScaleX);
    const Pel *colV              = srcV + (x >> chromaScaleX);
    const Pel *col[MAX_NUM_COMP] = { colY, colU, colV };
    const Pel *colE              = col[edgeCmp];
    const Pel *colA              = colE + srcStrideE * edgePosYA + edgePosXA;
    const Pel *colB              = colE + srcStrideE * edgePosYB + edgePosXB;

    for (int edgeIdc = 0; edgeIdc < edgeIdcNum; edgeIdc++)
    {
      const int edgeNum    = g_ccSaoEdgeNum[edgeIdc][0];
      const int edgeNumUni = g_ccSaoEdgeNum[edgeIdc][1];
      for (int edgeThr = 0; edgeThr < NUM_CCSAO_EDGE_THR; edgeThr++)
      {
        const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr];
        const int edgeIdxA   = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
        const int edgeIdxB   = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
        const int edgeIdx    = edgeIdxA * edgeNumUni + edgeIdxB;
        for (int bandIdc = 0; bandIdc < MAX_CCSAO_BAND_IDC; bandIdc++)
        {
          const int bandCmp = g_ccSaoBandTab[bandIdc][0];
          const int bandNum = g_ccSaoBandTab[bandIdc][1];
          if (bandNum * edgeNum > MAX_CCSAO_CLASS_NUM)
          {
            continue;
          }

          const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
          const int classIdx = bandIdx * edgeNum + edgeIdx;

          int idx = getCcSaoEdgeStatIdx(ctuRsAddr, bandIdc, edgeCmp, edgeIdc, (int)edgeDir, edgeThr);

          blkStatsEdge[idx].diff[classIdx] += org[x] - dst[x];
          blkStatsEdge[idx].count[classIdx]++;
        }
      }
    }
  }
}

inline void EncSampleAdaptiveOffset::getCcSaomChromaSampleStatsEdgePre(
  int startx, int endx, const ptrdiff_t srcStrideE, const int edgeIdcNum, const int bitDepth, const int edgePosXA,
  const int edgePosXB, const int edgePosYA, const int edgePosYB, std::vector<CcSaoStatData>(&blkStatsEdge),
  const int ctuRsAddr, const Pel *srcY, const Pel *srcU, const Pel *srcV, const int chromaScaleX, int edgeCmp,
  SAOModeNewTypes edgeDir, const Pel *org, const Pel *dst)
{
  int NUM_CCSAO_EDGE_THR = (m_CCSaoMode != 2) ? MAX_CCSAO_EDGE_THR : 4;

  for (int x = startx; x < endx; x++)
  {
    const Pel *colY              = srcY + (x << chromaScaleX);
    const Pel *colU              = srcU + x;
    const Pel *colV              = srcV + x;
    const Pel *col[MAX_NUM_COMP] = { colY, colU, colV };
    const Pel *colE              = col[edgeCmp];
    const Pel *colA              = colE + srcStrideE * edgePosYA + edgePosXA;
    const Pel *colB              = colE + srcStrideE * edgePosYB + edgePosXB;

    for (int edgeIdc = 0; edgeIdc < edgeIdcNum; edgeIdc++)
    {
      const int edgeNum    = g_ccSaoEdgeNum[edgeIdc][0];
      const int edgeNumUni = g_ccSaoEdgeNum[edgeIdc][1];
      for (int edgeThr = 0; edgeThr < NUM_CCSAO_EDGE_THR; edgeThr++)
      {
        const int edgeThrVal = g_ccSaoEdgeThr[edgeIdc][edgeThr];
        const int edgeIdxA   = getCcSaoEdgeIdx(*colE, *colA, edgeThrVal, edgeIdc);
        const int edgeIdxB   = getCcSaoEdgeIdx(*colE, *colB, edgeThrVal, edgeIdc);
        const int edgeIdx    = edgeIdxA * edgeNumUni + edgeIdxB;
        for (int bandIdc = 0; bandIdc < MAX_CCSAO_BAND_IDC; bandIdc++)
        {
          const int bandCmp = g_ccSaoBandTab[bandIdc][0];
          const int bandNum = g_ccSaoBandTab[bandIdc][1];
          if (bandNum * edgeNum > MAX_CCSAO_CLASS_NUM)
          {
            continue;
          }

          const int bandIdx  = (*col[bandCmp] * bandNum) >> bitDepth;
          const int classIdx = bandIdx * edgeNum + edgeIdx;

          int idx = getCcSaoEdgeStatIdx(ctuRsAddr, bandIdc, edgeCmp, edgeIdc, (int)edgeDir, edgeThr);

          blkStatsEdge[idx].diff[classIdx] += org[x] - dst[x];
          blkStatsEdge[idx].count[classIdx]++;
        }
      }
    }
  }
}

void EncSampleAdaptiveOffset::getCcSaoBlkStatsEdgePre(
  CodingStructure &cs, const CompID compID, const ChromaFormat chromaFormat, const int bitDepth,
  std::vector<CcSaoStatData>(&blkStatsEdge), const int ctuRsAddr, const Pel *srcY, const Pel *srcU, const Pel *srcV,
  const Pel *org, const Pel *dst, const ptrdiff_t srcStrideY, const ptrdiff_t srcStrideU, const ptrdiff_t srcStrideV,
  const ptrdiff_t orgStride, const ptrdiff_t dstStride, const int width, const int height, bool isLeftAvail,
  bool isRightAvail, bool isAboveAvail, bool isBelowAvail, bool isAboveLeftAvail, bool isAboveRightAvail)
{
  const int chromaScaleX   = getChannelTypeScaleX(ChannelType::CHROMA, chromaFormat);
  const int chromaScaleY   = getChannelTypeScaleY(ChannelType::CHROMA, chromaFormat);
  const int chromaScaleYM1 = 1 - chromaScaleY;

  const int edgeCmpNum = m_extChroma ? MAX_NUM_COMP : MAX_NUM_LUMA_COMP;
  const int edgeIdcNum =
    (m_CCSaoMode != 2) && (m_intraPeriod != 1 || cs.sps->m_PLTMode || m_extChroma) ? MAX_CCSAO_EDGE_IDC : 1;
  const ptrdiff_t srcStrideTab[MAX_NUM_COMP] = { srcStrideY, srcStrideU, srcStrideU };

  int        y, startX, startY, endX, endY;
  int        firstLineStartX, firstLineEndX;
  const Pel *srcYT = srcY;
  const Pel *srcUT = srcU;
  const Pel *srcVT = srcV;
  const Pel *orgT  = org;
  const Pel *dstT  = dst;

  switch (compID)
  {
  case COMP_Y:
    {
      for (int edgeCmp = COMP_Y; edgeCmp < edgeCmpNum; edgeCmp++)
      {
        const ptrdiff_t srcStrideE = srcStrideTab[edgeCmp];
        for (const auto edgeDir:
             { SAOModeNewTypes::EO_0, SAOModeNewTypes::EO_90, SAOModeNewTypes::EO_135, SAOModeNewTypes::EO_45 })
        {
          srcY          = srcYT;
          srcU          = srcUT;
          srcV          = srcVT;
          org           = orgT;
          dst           = dstT;
          int edgePosXA = g_ccSaoEdgePosX[(int)edgeDir][0], edgePosYA = g_ccSaoEdgePosY[(int)edgeDir][0];
          int edgePosXB = g_ccSaoEdgePosX[(int)edgeDir][1], edgePosYB = g_ccSaoEdgePosY[(int)edgeDir][1];

          switch (edgeDir)
          {
          case SAOModeNewTypes::NONE:
          case SAOModeNewTypes::BO:
          case SAOModeNewTypes::NUM:
          case SAOModeNewTypes::EO_0:
            {
              startX = isLeftAvail ? 0 : 1;
              endX   = isRightAvail ? width : (width - 1);
              for (y = 0; y < height; y++)
              {
                getCcSaomSampleStatsEdgePre(startX, endX, srcStrideE, edgeIdcNum, bitDepth, edgePosXA, edgePosXB,
                                            edgePosYA, edgePosYB, blkStatsEdge, ctuRsAddr, srcY, srcU, srcV,
                                            chromaScaleX, edgeCmp, edgeDir, org, dst);

                srcY += srcStrideY;
                srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
                srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
                org += orgStride;
                dst += dstStride;
              }
            }
            break;
          case SAOModeNewTypes::EO_90:
            {
              startY = isAboveAvail ? 0 : 1;
              endY   = isBelowAvail ? height : height - 1;
              if (!isAboveAvail)
              {
                srcY += srcStrideY;
                srcU += srcStrideU * chromaScaleYM1;
                srcV += srcStrideV * chromaScaleYM1;
                org += orgStride;
                dst += dstStride;
              }
              for (y = startY; y < endY; y++)
              {
                getCcSaomSampleStatsEdgePre(0, width, srcStrideE, edgeIdcNum, bitDepth, edgePosXA, edgePosXB, edgePosYA,
                                            edgePosYB, blkStatsEdge, ctuRsAddr, srcY, srcU, srcV, chromaScaleX, edgeCmp,
                                            edgeDir, org, dst);
                srcY += srcStrideY;
                srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
                srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
                org += orgStride;
                dst += dstStride;
              }
            }
            break;
          case SAOModeNewTypes::EO_135:
            {
              startX = isLeftAvail ? 0 : 1;
              endX   = isRightAvail ? width : (width - 1);

              // 1st line
              firstLineStartX = isAboveLeftAvail ? 0 : 1;
              firstLineEndX   = isAboveAvail ? endX : 1;
              getCcSaomSampleStatsEdgePre(firstLineStartX, firstLineEndX, srcStrideE, edgeIdcNum, bitDepth, edgePosXA,
                                          edgePosXB, edgePosYA, edgePosYB, blkStatsEdge, ctuRsAddr, srcY, srcU, srcV,
                                          chromaScaleX, edgeCmp, edgeDir, org, dst);
              srcY += srcStrideY;
              srcU += srcStrideU * chromaScaleYM1;
              srcV += srcStrideV * chromaScaleYM1;
              org += orgStride;
              dst += dstStride;

              // middle lines
              for (y = 1; y < height - 1; y++)
              {
                getCcSaomSampleStatsEdgePre(startX, endX, srcStrideE, edgeIdcNum, bitDepth, edgePosXA, edgePosXB,
                                            edgePosYA, edgePosYB, blkStatsEdge, ctuRsAddr, srcY, srcU, srcV,
                                            chromaScaleX, edgeCmp, edgeDir, org, dst);
                srcY += srcStrideY;
                srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
                srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
                org += orgStride;
                dst += dstStride;
              }
            }
            break;
          case SAOModeNewTypes::EO_45:
            {
              startX = isLeftAvail ? 0 : 1;
              endX   = isRightAvail ? width : (width - 1);

              // first line
              firstLineStartX = isAboveAvail ? startX : (width - 1);
              firstLineEndX   = isAboveRightAvail ? width : (width - 1);
              getCcSaomSampleStatsEdgePre(firstLineStartX, firstLineEndX, srcStrideE, edgeIdcNum, bitDepth, edgePosXA,
                                          edgePosXB, edgePosYA, edgePosYB, blkStatsEdge, ctuRsAddr, srcY, srcU, srcV,
                                          chromaScaleX, edgeCmp, edgeDir, org, dst);

              srcY += srcStrideY;
              srcU += srcStrideU * chromaScaleYM1;
              srcV += srcStrideV * chromaScaleYM1;
              org += orgStride;
              dst += dstStride;

              // middle lines
              for (y = 1; y < height - 1; y++)
              {
                getCcSaomSampleStatsEdgePre(startX, endX, srcStrideE, edgeIdcNum, bitDepth, edgePosXA, edgePosXB,
                                            edgePosYA, edgePosYB, blkStatsEdge, ctuRsAddr, srcY, srcU, srcV,
                                            chromaScaleX, edgeCmp, edgeDir, org, dst);

                srcY += srcStrideY;
                srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
                srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
                org += orgStride;
                dst += dstStride;
              }
            }
            break;
          }
        }
      }
      break;
    }   // case COMPONENT_Y
  case COMP_Cb:
  case COMP_Cr:
    {
      for (int edgeCmp = COMP_Y; edgeCmp < edgeCmpNum; edgeCmp++)
      {
        const ptrdiff_t srcStrideE = srcStrideTab[edgeCmp];
        for (const auto edgeDir:
             { SAOModeNewTypes::EO_0, SAOModeNewTypes::EO_90, SAOModeNewTypes::EO_135, SAOModeNewTypes::EO_45 })
        {
          srcY          = srcYT;
          srcU          = srcUT;
          srcV          = srcVT;
          org           = orgT;
          dst           = dstT;
          int edgePosXA = g_ccSaoEdgePosX[(int)edgeDir][0], edgePosYA = g_ccSaoEdgePosY[(int)edgeDir][0];
          int edgePosXB = g_ccSaoEdgePosX[(int)edgeDir][1], edgePosYB = g_ccSaoEdgePosY[(int)edgeDir][1];

          switch (edgeDir)
          {
          case SAOModeNewTypes::NONE:
          case SAOModeNewTypes::BO:
          case SAOModeNewTypes::NUM:
          case SAOModeNewTypes::EO_0:
            {
              startX = isLeftAvail ? 0 : 1;
              endX   = isRightAvail ? width : (width - 1);
              for (y = 0; y < height; y++)
              {
                getCcSaomChromaSampleStatsEdgePre(startX, endX, srcStrideE, edgeIdcNum, bitDepth, edgePosXA, edgePosXB,
                                                  edgePosYA, edgePosYB, blkStatsEdge, ctuRsAddr, srcY, srcU, srcV,
                                                  chromaScaleX, edgeCmp, edgeDir, org, dst);

                srcY += srcStrideY << chromaScaleY;
                srcU += srcStrideU;
                srcV += srcStrideV;
                org += orgStride;
                dst += dstStride;
              }
            }
            break;
          case SAOModeNewTypes::EO_90:
            {
              startY = isAboveAvail ? 0 : 1;
              endY   = isBelowAvail ? height : height - 1;
              if (!isAboveAvail)
              {
                srcY += srcStrideY << chromaScaleY;
                srcU += srcStrideU;
                srcV += srcStrideV;
                org += orgStride;
                dst += dstStride;
              }
              for (y = startY; y < endY; y++)
              {
                getCcSaomChromaSampleStatsEdgePre(0, width, srcStrideE, edgeIdcNum, bitDepth, edgePosXA, edgePosXB,
                                                  edgePosYA, edgePosYB, blkStatsEdge, ctuRsAddr, srcY, srcU, srcV,
                                                  chromaScaleX, edgeCmp, edgeDir, org, dst);

                srcY += srcStrideY << chromaScaleY;
                srcU += srcStrideU;
                srcV += srcStrideV;
                org += orgStride;
                dst += dstStride;
              }
            }
            break;
          case SAOModeNewTypes::EO_135:
            {
              startX = isLeftAvail ? 0 : 1;
              endX   = isRightAvail ? width : (width - 1);

              // 1st line
              firstLineStartX = isAboveLeftAvail ? 0 : 1;
              firstLineEndX   = isAboveAvail ? endX : 1;
              getCcSaomChromaSampleStatsEdgePre(firstLineStartX, firstLineEndX, srcStrideE, edgeIdcNum, bitDepth,
                                                edgePosXA, edgePosXB, edgePosYA, edgePosYB, blkStatsEdge, ctuRsAddr,
                                                srcY, srcU, srcV, chromaScaleX, edgeCmp, edgeDir, org, dst);

              srcY += srcStrideY << chromaScaleY;
              srcU += srcStrideU;
              srcV += srcStrideV;
              org += orgStride;
              dst += dstStride;

              // middle lines
              for (y = 1; y < height - 1; y++)
              {
                getCcSaomChromaSampleStatsEdgePre(startX, endX, srcStrideE, edgeIdcNum, bitDepth, edgePosXA, edgePosXB,
                                                  edgePosYA, edgePosYB, blkStatsEdge, ctuRsAddr, srcY, srcU, srcV,
                                                  chromaScaleX, edgeCmp, edgeDir, org, dst);

                srcY += srcStrideY << chromaScaleY;
                srcU += srcStrideU;
                srcV += srcStrideV;
                org += orgStride;
                dst += dstStride;
              }
            }
            break;
          case SAOModeNewTypes::EO_45:
            {
              startX = isLeftAvail ? 0 : 1;
              endX   = isRightAvail ? width : (width - 1);

              // first line
              firstLineStartX = isAboveAvail ? startX : (width - 1);
              firstLineEndX   = isAboveRightAvail ? width : (width - 1);
              getCcSaomChromaSampleStatsEdgePre(firstLineStartX, firstLineEndX, srcStrideE, edgeIdcNum, bitDepth,
                                                edgePosXA, edgePosXB, edgePosYA, edgePosYB, blkStatsEdge, ctuRsAddr,
                                                srcY, srcU, srcV, chromaScaleX, edgeCmp, edgeDir, org, dst);

              srcY += srcStrideY << chromaScaleY;
              srcU += srcStrideU;
              srcV += srcStrideV;
              org += orgStride;
              dst += dstStride;

              // middle lines
              for (y = 1; y < height - 1; y++)
              {
                getCcSaomChromaSampleStatsEdgePre(startX, endX, srcStrideE, edgeIdcNum, bitDepth, edgePosXA, edgePosXB,
                                                  edgePosYA, edgePosYB, blkStatsEdge, ctuRsAddr, srcY, srcU, srcV,
                                                  chromaScaleX, edgeCmp, edgeDir, org, dst);

                srcY += srcStrideY << chromaScaleY;
                srcU += srcStrideU;
                srcV += srcStrideV;
                org += orgStride;
                dst += dstStride;
              }
            }
            break;
          }
        }
      }
      break;
    }   // case COMP_Cb COMP_Cr
  default:
    {
      THROW("Not a supported CCSAO compID\n");
    }
  }
}

void EncSampleAdaptiveOffset::getCcSaoBlkStats(
  const CompID compID, const ChromaFormat chromaFormat, const int bitDepth, const int setIdx,
  std::vector<CcSaoStatData> (&blkStats)[MAX_CCSAO_SET_NUM], const int ctuRsAddr, const uint16_t candPosY,
  const uint16_t bandNumY, const uint16_t bandNumU, const uint16_t bandNumV, const Pel *srcY, const Pel *srcU,
  const Pel *srcV, const Pel *org, const Pel *dst, const ptrdiff_t srcStrideY, const ptrdiff_t srcStrideU,
  const ptrdiff_t srcStrideV, const ptrdiff_t orgStride, const ptrdiff_t dstStride, const int width, const int height,
  bool isLeftAvail, bool isRightAvail, bool isAboveAvail, bool isBelowAvail, bool isAboveLeftAvail,
  bool isAboveRightAvail)
{
  const int candPosYX = g_ccSaoCandPosX[COMP_Y][candPosY];
  const int candPosYY = g_ccSaoCandPosY[COMP_Y][candPosY];

  const int chromaScaleX   = getChannelTypeScaleX(ChannelType::CHROMA, chromaFormat);
  const int chromaScaleY   = getChannelTypeScaleY(ChannelType::CHROMA, chromaFormat);
  const int chromaScaleYM1 = 1 - chromaScaleY;

  int y, startX, startY, endX, endY;
  int firstLineStartX, firstLineEndX;

  int64_t  *diff  = blkStats[setIdx][ctuRsAddr].diff;
  uint32_t *count = blkStats[setIdx][ctuRsAddr].count;
  srcY            = srcY + srcStrideY * candPosYY + candPosYX;

  switch (compID)
  {
  case COMP_Y:
    {
      switch (candPosY)
      {
      case 0:   // top left (-1, -1), unlike SAO, CCSAO BO only uses one spatial neighbor sample to derive band
                // information
        /* total 9 cases will come up here
        for (-1,-1) just use the top and middle lines and check for vb */
        {
          startX = isLeftAvail ? 0 : 1;
          endX   = width;

          // 1st line

          firstLineStartX = isAboveLeftAvail ? 0 : 1;
          firstLineEndX   = isAboveAvail ? endX : 1;
          m_getCcSaomSampleStats(firstLineStartX, firstLineEndX, bitDepth, diff, count, chromaScaleX, bandNumY,
                                 bandNumU, bandNumV, srcY, srcU, srcV, org, dst);
          srcY += srcStrideY;
          srcU += srcStrideU * chromaScaleYM1;
          srcV += srcStrideV * chromaScaleYM1;
          org += orgStride;
          dst += dstStride;

          // middle lines
          for (y = 1; y < height; y++)
          {
            m_getCcSaomSampleStats(startX, endX, bitDepth, diff, count, chromaScaleX, bandNumY, bandNumU, bandNumV,
                                   srcY, srcU, srcV, org, dst);

            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            org += orgStride;
            dst += dstStride;
          }
          break;
        }
      case 1: /*(0, -1)  top sample */
        {
          startY = isAboveAvail ? 0 : 1;
          endY   = height;
          if (!isAboveAvail)
          {
            srcY += srcStrideY;
            srcU += srcStrideU * chromaScaleYM1;
            srcV += srcStrideV * chromaScaleYM1;
            org += orgStride;
            dst += dstStride;
          }
          for (y = startY; y < endY; y++)
          {
            m_getCcSaomSampleStats(0, width, bitDepth, diff, count, chromaScaleX, bandNumY, bandNumU, bandNumV, srcY,
                                   srcU, srcV, org, dst);

            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            org += orgStride;
            dst += dstStride;
          }
          break;
        }
      case 2: /*(0, -1)  top right sample */
        {
          startX          = isLeftAvail ? 0 : 1;
          endX            = width;
          // first line
          firstLineStartX = isAboveAvail ? startX : (width - 1);
          firstLineEndX   = isAboveRightAvail ? width : (width - 1);
          m_getCcSaomSampleStats(firstLineStartX, firstLineEndX, bitDepth, diff, count, chromaScaleX, bandNumY,
                                 bandNumU, bandNumV, srcY, srcU, srcV, org, dst);

          srcY += srcStrideY;
          srcU += srcStrideU * chromaScaleYM1;
          srcV += srcStrideV * chromaScaleYM1;
          org += orgStride;
          dst += dstStride;

          // middle lines
          for (y = 1; y < height; y++)
          {
            m_getCcSaomSampleStats(startX, endX, bitDepth, diff, count, chromaScaleX, bandNumY, bandNumU, bandNumV,
                                   srcY, srcU, srcV, org, dst);

            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            org += orgStride;
            dst += dstStride;
          }
          break;
        }
      case 3: /*(-1, 0)  left sample */
        {
          startX = isLeftAvail ? 0 : 1;
          endX   = width;

          for (y = 0; y < height; y++)
          {
            m_getCcSaomSampleStats(startX, endX, bitDepth, diff, count, chromaScaleX, bandNumY, bandNumU, bandNumV,
                                   srcY, srcU, srcV, org, dst);

            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            org += orgStride;
            dst += dstStride;
          }
          break;
        }
      case 4: /*(0, 0)  current sample */
        { /* when current sample is choosen there is no more dependency on neighbor samples*/

          for (y = 0; y < height; y++)
          {
            m_getCcSaomSampleStats(0, width, bitDepth, diff, count, chromaScaleX, bandNumY, bandNumU, bandNumV, srcY,
                                   srcU, srcV, org, dst);

            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            org += orgStride;
            dst += dstStride;
          }
          break;
        }
      case 5: /*(1, 0)  right sample */
        {
          startX = 0;
          endX   = isRightAvail ? width : (width - 1);

          for (y = 0; y < height; y++)
          {
            m_getCcSaomSampleStats(startX, endX, bitDepth, diff, count, chromaScaleX, bandNumY, bandNumU, bandNumV,
                                   srcY, srcU, srcV, org, dst);

            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            org += orgStride;
            dst += dstStride;
          }
          break;
        }
      case 6: /*(-1, 1)  below left sample */
        {
          startX = isLeftAvail ? 0 : 1;
          endX   = width;

          for (y = 0; y < height; y++)
          {
            m_getCcSaomSampleStats(startX, endX, bitDepth, diff, count, chromaScaleX, bandNumY, bandNumU, bandNumV,
                                   srcY, srcU, srcV, org, dst);

            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            org += orgStride;
            dst += dstStride;
          }
          break;
        }
      case 7: /*(0, 1)  below sample */
        {
          startY = 0;
          endY   = isBelowAvail ? height : height - 1;

          for (y = startY; y < endY; y++)
          {
            m_getCcSaomSampleStats(0, width, bitDepth, diff, count, chromaScaleX, bandNumY, bandNumU, bandNumV, srcY,
                                   srcU, srcV, org, dst);

            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            org += orgStride;
            dst += dstStride;
          }
          break;
        }
      case 8: /*(1, 1)  below right sample */
        {
          startX = 0;
          endX   = isRightAvail ? width : (width - 1);

          for (y = 0; y < height - 1; y++)
          {
            m_getCcSaomSampleStats(startX, endX, bitDepth, diff, count, chromaScaleX, bandNumY, bandNumU, bandNumV,
                                   srcY, srcU, srcV, org, dst);

            srcY += srcStrideY;
            srcU += srcStrideU * ((y & 0x1) | chromaScaleYM1);
            srcV += srcStrideV * ((y & 0x1) | chromaScaleYM1);
            org += orgStride;
            dst += dstStride;
          }
          break;
        }
        break;
      }
      break;
    }
  case COMP_Cb:
  case COMP_Cr:
    {
      switch (candPosY)
      {
      case 0:   // top left (-1, -1), unlike SAO, CCSAO BO only uses one spatial neighbor sample to derive band
                // information
        /* total 9 cases will come up here
        for (-1,-1) just use the top and middle lines and check for vb */
        {
          startX          = isLeftAvail ? 0 : 1;
          endX            = width;
          // 1st line
          firstLineStartX = isAboveLeftAvail ? 0 : 1;
          firstLineEndX   = isAboveAvail ? endX : 1;
          m_getCcSaomChromaSampleStats(firstLineStartX, firstLineEndX, bitDepth, diff, count, chromaScaleX, bandNumY,
                                       bandNumU, bandNumV, srcY, srcU, srcV, org, dst);
          srcY += srcStrideY << chromaScaleY;
          srcU += srcStrideU;
          srcV += srcStrideV;
          org += orgStride;
          dst += dstStride;

          // middle lines
          for (y = 1; y < height; y++)
          {
            m_getCcSaomChromaSampleStats(startX, endX, bitDepth, diff, count, chromaScaleX, bandNumY, bandNumU,
                                         bandNumV, srcY, srcU, srcV, org, dst);

            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            org += orgStride;
            dst += dstStride;
          }
          break;
        }
      case 1: /*(0, -1)  top sample */
        {
          startY = isAboveAvail ? 0 : 1;
          endY   = height;
          if (!isAboveAvail)
          {
            srcY += srcStrideY;
            srcU += srcStrideU * chromaScaleYM1;
            srcV += srcStrideV * chromaScaleYM1;
            org += orgStride;
            dst += dstStride;
          }
          for (y = startY; y < endY; y++)
          {
            m_getCcSaomChromaSampleStats(0, width, bitDepth, diff, count, chromaScaleX, bandNumY, bandNumU, bandNumV,
                                         srcY, srcU, srcV, org, dst);

            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            org += orgStride;
            dst += dstStride;
          }
          break;
        }
      case 2: /*(0, -1)  top right sample */
        {
          startX          = isLeftAvail ? 0 : 1;
          endX            = width;
          // first line
          firstLineStartX = isAboveAvail ? startX : (width - 1);
          firstLineEndX   = isAboveRightAvail ? width : (width - 1);
          m_getCcSaomChromaSampleStats(firstLineStartX, firstLineEndX, bitDepth, diff, count, chromaScaleX, bandNumY,
                                       bandNumU, bandNumV, srcY, srcU, srcV, org, dst);

          srcY += srcStrideY << chromaScaleY;
          srcU += srcStrideU;
          srcV += srcStrideV;
          org += orgStride;
          dst += dstStride;

          // middle lines
          for (y = 1; y < height; y++)
          {
            m_getCcSaomChromaSampleStats(startX, endX, bitDepth, diff, count, chromaScaleX, bandNumY, bandNumU,
                                         bandNumV, srcY, srcU, srcV, org, dst);

            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            org += orgStride;
            dst += dstStride;
          }
          break;
        }
      case 3: /*(-1, 0)  left sample */
        {
          startX = isLeftAvail ? 0 : 1;
          endX   = width;

          for (y = 0; y < height; y++)
          {
            m_getCcSaomChromaSampleStats(startX, endX, bitDepth, diff, count, chromaScaleX, bandNumY, bandNumU,
                                         bandNumV, srcY, srcU, srcV, org, dst);

            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            org += orgStride;
            dst += dstStride;
          }
          break;
        }
      case 4: /*(0, 0)  current sample */
        { /* when current sample is choosen there is no more dependency on neighbor samples*/

          for (y = 0; y < height; y++)
          {
            m_getCcSaomChromaSampleStats(0, width, bitDepth, diff, count, chromaScaleX, bandNumY, bandNumU, bandNumV,
                                         srcY, srcU, srcV, org, dst);

            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            org += orgStride;
            dst += dstStride;
          }
          break;
        }
      case 5: /*(1, 0)  right sample */
        {
          startX = 0;
          endX   = isRightAvail ? width : (width - 1);

          for (y = 0; y < height; y++)
          {
            m_getCcSaomChromaSampleStats(startX, endX, bitDepth, diff, count, chromaScaleX, bandNumY, bandNumU,
                                         bandNumV, srcY, srcU, srcV, org, dst);

            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            org += orgStride;
            dst += dstStride;
          }
          break;
        }
      case 6: /*(-1, 1)  below left sample */
        {
          startX = isLeftAvail ? 0 : 1;
          endX   = width;

          for (y = 0; y < height; y++)
          {
            m_getCcSaomChromaSampleStats(startX, endX, bitDepth, diff, count, chromaScaleX, bandNumY, bandNumU,
                                         bandNumV, srcY, srcU, srcV, org, dst);

            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            org += orgStride;
            dst += dstStride;
          }
          break;
        }
      case 7: /*(0, 1)  below sample */
        {
          startY = 0;
          endY   = isBelowAvail ? height : height - 1;

          for (y = startY; y < endY; y++)
          {
            m_getCcSaomChromaSampleStats(0, width, bitDepth, diff, count, chromaScaleX, bandNumY, bandNumU, bandNumV,
                                         srcY, srcU, srcV, org, dst);

            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            org += orgStride;
            dst += dstStride;
          }
          break;
        }
      case 8: /*(1, 1)  below right sample */
        {
          startX = 0;
          endX   = isRightAvail ? width : (width - 1);

          for (y = 0; y < height - 1; y++)
          {
            m_getCcSaomChromaSampleStats(startX, endX, bitDepth, diff, count, chromaScaleX, bandNumY, bandNumU,
                                         bandNumV, srcY, srcU, srcV, org, dst);

            srcY += srcStrideY << chromaScaleY;
            srcU += srcStrideU;
            srcV += srcStrideV;
            org += orgStride;
            dst += dstStride;
          }
          break;
        }
      }
      break;
    }
  default:
    {
      THROW("Not a supported CCSAO compID\n");
    }
  }
}

void EncSampleAdaptiveOffset::getCcSaoFrameStats(const CompID compID, const int setIdx, const uint8_t *ccSaoControl,
                                                 std::vector<CcSaoStatData> (&blkStats)[MAX_CCSAO_SET_NUM],
                                                 CcSaoStatData frameStats[MAX_CCSAO_SET_NUM],
                                                 std::vector<CcSaoStatData> (&blkStatsEdge)[MAX_CCSAO_SET_NUM],
                                                 CcSaoStatData frameStatsEdge[MAX_CCSAO_SET_NUM], const uint8_t setType)
{
  frameStats[setIdx].reset();
  frameStatsEdge[setIdx].reset();
  int setIdc = setIdx + 1;

  for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
  {
    if (ccSaoControl[ctbIdx] == setIdc)
    {
      if (setType == CCSAO_SET_TYPE_BAND)
      {
        frameStats[setIdx] += blkStats[setIdx][ctbIdx];
      }
      else /*CCSAO_SET_TYPE_EDGE*/
      {
        frameStatsEdge[setIdx] += blkStatsEdge[setIdx][ctbIdx];
      }
    }
  }
}

inline int EncSampleAdaptiveOffset::estCcSaoIterOffset(const double lambda, const int offsetInput, const int64_t count,
                                                       const int64_t diffSum, const int shift, const int bitIncrease,
                                                       int64_t &bestDist, double &bestCost, const int offsetTh)
{
  int     iterOffset, tempOffset;
  int64_t tempDist, tempRate;
  double  tempCost, tempMinCost;
  int     offsetOutput = 0;
  iterOffset           = offsetInput;
  // Assuming sending quantized value 0 results in zero offset and sending the value zero needs 1 bit. entropy coder can
  // be used to measure the exact rate here.
  tempMinCost          = lambda;
  while (iterOffset != 0)
  {
    // Calculate the bits required for signaling the offset
    tempRate = lengthUvlc(abs(iterOffset)) + (iterOffset == 0 ? 0 : 1);

    // Do the dequantization before distortion calculation
    tempOffset = iterOffset << bitIncrease;
    tempDist   = estSaoDist(count, tempOffset, diffSum, shift);
    tempCost   = ((double)tempDist + lambda * (double)tempRate);
    if (tempCost < tempMinCost)
    {
      tempMinCost  = tempCost;
      offsetOutput = iterOffset;
      bestDist     = tempDist;
      bestCost     = tempCost;
    }
    iterOffset = (iterOffset > 0) ? (iterOffset - 1) : (iterOffset + 1);
  }
  return offsetOutput;
}

void EncSampleAdaptiveOffset::deriveCcSaoOffsets(const CompID compID, const int bitDepth, const int setIdx,
                                                 CcSaoStatData frameStats[MAX_CCSAO_SET_NUM],
                                                 short         offset[MAX_CCSAO_SET_NUM][MAX_CCSAO_CLASS_NUM])
{
  int quantOffsets[MAX_CCSAO_CLASS_NUM] = { 0 };

  for (int k = 0; k < MAX_CCSAO_CLASS_NUM; k++)
  {
    if (frameStats[setIdx].count[k] == 0)
    {
      continue;
    }

    quantOffsets[k] =
      (int)xRoundIbdi(bitDepth,
                      (double)(frameStats[setIdx].diff[k] << DISTORTION_PRECISION_ADJUSTMENT(bitDepth)) /
                        (double)(frameStats[setIdx].count[k]));
    quantOffsets[k] = Clip3(-MAX_CCSAO_OFFSET_THR, MAX_CCSAO_OFFSET_THR, quantOffsets[k]);
  }

  int64_t dist[MAX_CCSAO_CLASS_NUM] = { 0 };
  double  cost[MAX_CCSAO_CLASS_NUM] = { 0 };
  for (int k = 0; k < MAX_CCSAO_CLASS_NUM; k++)
  {
    cost[k] = m_lambda[compID];
    if (quantOffsets[k] != 0)
    {
      quantOffsets[k] = estCcSaoIterOffset(m_lambda[compID], quantOffsets[k], frameStats[setIdx].count[k],
                                           frameStats[setIdx].diff[k], 0, 0, dist[k], cost[k], MAX_CCSAO_OFFSET_THR);
    }
  }

  for (int k = 0; k < MAX_CCSAO_CLASS_NUM; k++)
  {
    CHECK(quantOffsets[k] < -MAX_CCSAO_OFFSET_THR || quantOffsets[k] > MAX_CCSAO_OFFSET_THR,
          "Exceeded valid range for CCSAO offset");
    offset[setIdx][k] = quantOffsets[k];
  }
}

void EncSampleAdaptiveOffset::getCcSaoDistortion(const CompID compID, const int setIdx,
                                                 std::vector<CcSaoStatData> (&blkStats)[MAX_CCSAO_SET_NUM],
                                                 short    offset[MAX_CCSAO_SET_NUM][MAX_CCSAO_CLASS_NUM],
                                                 int64_t *trainingDistortion[MAX_CCSAO_SET_NUM])
{
  ::memset(trainingDistortion[setIdx], 0, sizeof(int64_t) * m_numCTUsInPic);

  for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
  {
    for (int k = 0; k < MAX_CCSAO_CLASS_NUM; k++)
    {
      trainingDistortion[setIdx][ctbIdx] +=
        estSaoDist(blkStats[setIdx][ctbIdx].count[k], offset[setIdx][k], blkStats[setIdx][ctbIdx].diff[k], 0);
    }
  }
}
void EncSampleAdaptiveOffset::determineCcSaoControlIdc(CodingStructure &cs, const CompID compID, const int ctuWidthC,
                                                       const int ctuHeightC, const int picWidthC, const int picHeightC,
                                                       CcSaoEncParam &ccSaoParam, uint8_t *ccSaoControl,
                                                       int64_t *trainingDistorsion[MAX_CCSAO_SET_NUM],
                                                       int64_t &curTotalDist, double &curTotalRate)
{
  bool setEnabled[MAX_CCSAO_SET_NUM];
  std::fill_n(setEnabled, MAX_CCSAO_SET_NUM, false);

  SetIdxCount setIdxCount[MAX_CCSAO_SET_NUM];
  for (int i = 0; i < MAX_CCSAO_SET_NUM; i++)
  {
    setIdxCount[i].setIdx = i;
    setIdxCount[i].count  = 0;
  }

  double prevRate = curTotalRate;

  TempCtx ctxInitial(m_ctxPool);
  TempCtx ctxBest(m_ctxPool);
  TempCtx ctxStart(m_ctxPool);
  ctxInitial = SubCtx(Ctx::CcSaoControlIdc, m_CABACEstimator->getCtx());
  ctxBest    = SubCtx(Ctx::CcSaoControlIdc, m_CABACEstimator->getCtx());

  int ctbIdx = 0;
  for (int yCtb = 0; yCtb < picHeightC; yCtb += ctuHeightC)
  {
    for (int xCtb = 0; xCtb < picWidthC; xCtb += ctuWidthC)
    {
      int64_t bestDist   = MAX_INT;
      double  bestRate   = MAX_DOUBLE;
      double  bestCost   = MAX_DOUBLE;
      uint8_t bestSetIdc = 0;
      uint8_t bestSetIdx = 0;

      m_CABACEstimator->getCtx() = ctxBest;
      ctxStart                   = SubCtx(Ctx::CcSaoControlIdc, m_CABACEstimator->getCtx());

      for (int setIdx = 0; setIdx <= MAX_CCSAO_SET_NUM; setIdx++)
      {
        if (setIdx < MAX_CCSAO_SET_NUM && !ccSaoParam.setEnabled[setIdx])
        {
          continue;
        }

        uint8_t setIdc             = ccSaoParam.mapIdxToIdc[setIdx];
        m_CABACEstimator->getCtx() = ctxStart;
        m_CABACEstimator->resetBits();
        const Position lumaPos = Position({ xCtb << getComponentScaleX(compID, cs.pcv->chrFormat),
                                            yCtb << getComponentScaleY(compID, cs.pcv->chrFormat) });
        m_CABACEstimator->codeCcSaoControlIdc(setIdc, cs, compID, ctbIdx, ccSaoControl, lumaPos, ccSaoParam.setNum);

        int64_t dist = setIdx == MAX_CCSAO_SET_NUM ? 0 : trainingDistorsion[setIdx][ctbIdx];
        double  rate = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
        double  cost = rate * m_lambda[compID] + dist;

        if (cost < bestCost)
        {
          bestCost             = cost;
          bestRate             = rate;
          bestDist             = dist;
          bestSetIdc           = setIdc;
          bestSetIdx           = setIdx;
          ctxBest              = SubCtx(Ctx::CcSaoControlIdc, m_CABACEstimator->getCtx());
          ccSaoControl[ctbIdx] = setIdx == MAX_CCSAO_SET_NUM ? 0 : setIdx + 1;
        }
      }
      if (bestSetIdc != 0)
      {
        setEnabled[bestSetIdx] = true;
        setIdxCount[bestSetIdx].count++;
      }
      curTotalRate += bestRate;
      curTotalDist += bestDist;
      ctbIdx++;
    }
  }

  if (!ccSaoParam.reusePrv)
  {
    std::copy_n(setEnabled, MAX_CCSAO_SET_NUM, ccSaoParam.setEnabled);

    std::stable_sort(setIdxCount, setIdxCount + MAX_CCSAO_SET_NUM, compareSetIdxCount);

    int setIdc        = 1;
    ccSaoParam.setNum = 0;
    for (SetIdxCount &s: setIdxCount)
    {
      int setIdx = s.setIdx;
      if (ccSaoParam.setEnabled[setIdx])
      {
        ccSaoParam.mapIdxToIdc[setIdx] = setIdc;
        ccSaoParam.setNum++;
        setIdc++;
      }
    }

    curTotalRate               = prevRate;
    m_CABACEstimator->getCtx() = ctxInitial;
    m_CABACEstimator->resetBits();
    ctbIdx = 0;
    for (int yCtb = 0; yCtb < picHeightC; yCtb += ctuHeightC)
    {
      for (int xCtb = 0; xCtb < picWidthC; xCtb += ctuWidthC)
      {
        const int      setIdxPlus1 = ccSaoControl[ctbIdx];
        const Position lumaPos     = Position({ xCtb << getComponentScaleX(compID, cs.pcv->chrFormat),
                                                yCtb << getComponentScaleY(compID, cs.pcv->chrFormat) });

        m_CABACEstimator->codeCcSaoControlIdc(setIdxPlus1 == 0 ? 0 : ccSaoParam.mapIdxToIdc[setIdxPlus1 - 1], cs,
                                              compID, ctbIdx, ccSaoControl, lumaPos, ccSaoParam.setNum);
        ctbIdx++;
      }
    }
    curTotalRate += FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
  }

  // restore for next iteration
  m_CABACEstimator->getCtx() = ctxInitial;
}

int EncSampleAdaptiveOffset::lengthUvlc(int uiCode)
{
  int uiLength = 1;
  int uiTemp   = ++uiCode;

  CHECK(!uiTemp, "Integer overflow");

  while (1 != uiTemp)
  {
    uiTemp >>= 1;
    uiLength += 2;
  }
  // Take care of cases where uiLength > 32
  return (uiLength >> 1) + ((uiLength + 1) >> 1);
}

int EncSampleAdaptiveOffset::getCcSaoParamRate(const CompID compID, const CcSaoEncParam &ccSaoParam)
{
  int bits = 0;

  if (ccSaoParam.setNum > 0)
  {
    bits += lengthUvlc(ccSaoParam.setNum - 1);

    int signaledSetNum = 0;
    for (int setIdx = 0; setIdx < MAX_CCSAO_SET_NUM; setIdx++)
    {
      if (ccSaoParam.setEnabled[setIdx])
      {
        bits += 1;
        if (ccSaoParam.setType[setIdx] == CCSAO_SET_TYPE_EDGE)
        {
          bits += m_extChroma ? MAX_CCSAO_EDGE_CMP_BITS : 0;
          bits += MAX_CCSAO_EDGE_IDC_BITS;
          bits += MAX_CCSAO_EDGE_DIR_BITS;
          bits += (m_CCSaoMode != 2) ? MAX_CCSAO_EDGE_THR_BITS : 2;
          bits += MAX_CCSAO_BAND_IDC_BITS;
        }
        else
        {
          bits += MAX_CCSAO_CAND_POS_Y_BITS;
          bits += MAX_CCSAO_BAND_NUM_Y_BITS;
          bits += MAX_CCSAO_BAND_NUM_U_BITS;
          bits += MAX_CCSAO_BAND_NUM_V_BITS;
        }

        int classNum = getCcSaoClassNumEnc(setIdx, ccSaoParam);
        for (int i = 0; i < classNum; i++)
        {
          bits += lengthUvlc(abs(ccSaoParam.offset[setIdx][i])) + (ccSaoParam.offset[setIdx][i] == 0 ? 0 : 1);
        }
        signaledSetNum++;
      }
    }
    CHECK(signaledSetNum != ccSaoParam.setNum, "Number of sets signaled not the same as indicated");
  }
  return bits;
}

int EncSampleAdaptiveOffset::getCcSaoClassNumEnc(const int setIdx, const CcSaoEncParam &ccSaoParam)
{
  int classNum = 0;

  if (ccSaoParam.setType[setIdx] == CCSAO_SET_TYPE_EDGE)
  {
    int bandIdc = ccSaoParam.bandNum[setIdx][COMP_Y], bandNum = g_ccSaoBandTab[bandIdc][1];
    int edgeIdc = ccSaoParam.bandNum[setIdx][COMP_Cr], edgeNum = g_ccSaoEdgeNum[edgeIdc][0];
    classNum = bandNum * edgeNum;
  }
  else
  {
    classNum =
      ccSaoParam.bandNum[setIdx][COMP_Y] * ccSaoParam.bandNum[setIdx][COMP_Cb] * ccSaoParam.bandNum[setIdx][COMP_Cr];
  }

  return classNum;
}

void EncSampleAdaptiveOffset::deriveCcSaoRDO(
  CodingStructure &cs, const CompID compID, int64_t *trainingDistortion[MAX_CCSAO_SET_NUM],
  std::vector<CcSaoStatData> (&blkStats)[MAX_CCSAO_SET_NUM], CcSaoStatData     frameStats[MAX_CCSAO_SET_NUM],
  std::vector<CcSaoStatData> (&blkStatsEdge)[MAX_CCSAO_SET_NUM], CcSaoStatData frameStatsEdge[MAX_CCSAO_SET_NUM],
  CcSaoEncParam &bestCcSaoParam, CcSaoEncParam &tempCcSaoParam, uint8_t *bestCcSaoControl, uint8_t *tempCcSaoControl,
  double &bestCost, double &tempCost)
{
  const int scaleX          = getComponentScaleX(compID, cs.pcv->chrFormat);
  const int scaleY          = getComponentScaleY(compID, cs.pcv->chrFormat);
  const int ctuWidthC       = cs.pcv->maxCUWidth >> scaleX;
  const int ctuHeightC      = cs.pcv->maxCUHeight >> scaleY;
  const int picWidthC       = cs.pcv->lumaWidth >> scaleX;
  const int picHeightC      = cs.pcv->lumaHeight >> scaleY;
  const int maxTrainingIter = 15;

  const TempCtx ctxStartCcSaoControlFlag(m_ctxPool, SubCtx(Ctx::CcSaoControlIdc, m_CABACEstimator->getCtx()));

  int    trainingIter = 0;
  bool   keepTraining = true;
  bool   improved     = false;
  double prevCost     = MAX_DOUBLE;
  while (keepTraining)
  {
    improved = false;

    for (int setIdx = 0; setIdx < MAX_CCSAO_SET_NUM; setIdx++)
    {
      if (tempCcSaoParam.setEnabled[setIdx])
      {
        if (tempCcSaoParam.reusePrv)
        {
          if (tempCcSaoParam.setType[setIdx] == CCSAO_SET_TYPE_BAND)
          {
            getCcSaoDistortion(compID, setIdx, blkStats, tempCcSaoParam.offset, trainingDistortion);
          }
          else
          {
            getCcSaoDistortion(compID, setIdx, blkStatsEdge, tempCcSaoParam.offset, trainingDistortion);
          }
        }
        else
        {
          getCcSaoFrameStats(compID, setIdx, tempCcSaoControl, blkStats, frameStats, blkStatsEdge, frameStatsEdge,
                             tempCcSaoParam.setType[setIdx]);
          if (tempCcSaoParam.setType[setIdx] == CCSAO_SET_TYPE_BAND)
          {
            deriveCcSaoOffsets(compID, cs.sps->m_bitDepths[toChannelType(compID)], setIdx, frameStats,
                               tempCcSaoParam.offset);
            getCcSaoDistortion(compID, setIdx, blkStats, tempCcSaoParam.offset, trainingDistortion);
          }
          else
          {
            deriveCcSaoOffsets(compID, cs.sps->m_bitDepths[toChannelType(compID)], setIdx, frameStatsEdge,
                               tempCcSaoParam.offset);
            getCcSaoDistortion(compID, setIdx, blkStatsEdge, tempCcSaoParam.offset, trainingDistortion);
          }
        }
      }
    }

    m_CABACEstimator->getCtx() = ctxStartCcSaoControlFlag;

    int64_t curTotalDist = 0;
    double  curTotalRate = 0;
    determineCcSaoControlIdc(cs, compID, ctuWidthC, ctuHeightC, picWidthC, picHeightC, tempCcSaoParam, tempCcSaoControl,
                             trainingDistortion, curTotalDist, curTotalRate);

    if (tempCcSaoParam.setNum > 0)
    {
      curTotalRate += !cs.slice->isIntra() ? 1 : 0;   // +1 reusePrv flag
      curTotalRate += tempCcSaoParam.reusePrv ? MAX_CCSAO_PRV_NUM_BITS : getCcSaoParamRate(compID, tempCcSaoParam);
      tempCost = curTotalRate * m_lambda[compID] + curTotalDist;

      if (tempCost < prevCost)
      {
        prevCost = tempCost;
        improved = true;
      }

      if (tempCost < bestCost)
      {
        bestCost       = tempCost;
        bestCcSaoParam = tempCcSaoParam;
        memcpy(bestCcSaoControl, tempCcSaoControl, sizeof(uint8_t) * m_numCTUsInPic);
      }
    }

    trainingIter++;
    if (!improved || trainingIter > maxTrainingIter || tempCcSaoParam.reusePrv)
    {
      keepTraining = false;
    }
  }
}

void EncSampleAdaptiveOffset::setupCcSaoPrv(CodingStructure &cs)
{
  if (cs.slice->isIDRorBLA() || cs.slice->m_pendingRasInit)
  {
    m_ccSaoPrvParamEnc[COMP_Y].clear();
    m_ccSaoPrvParamEnc[COMP_Cb].clear();
    m_ccSaoPrvParamEnc[COMP_Cr].clear();
  }

  for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
  {
    if (m_ccSaoComParam.enabled[compIdx] && !m_ccSaoComParam.reusePrv[compIdx])
    {
      if (m_ccSaoPrvParamEnc[compIdx].size() == MAX_CCSAO_PRV_NUM)
      {
        m_ccSaoPrvParamEnc[compIdx].pop_back();
      }

      CcSaoPrvParam prvParam;
      prvParam.temporalId = cs.slice->m_uiTLayer;
      prvParam.setNum     = m_ccSaoComParam.setNum[compIdx];
      std::memcpy(prvParam.setEnabled, m_ccSaoComParam.setEnabled[compIdx], sizeof(prvParam.setEnabled));
      std::memcpy(prvParam.setType, m_ccSaoComParam.setType[compIdx], sizeof(prvParam.setType));
      std::memcpy(prvParam.candPos, m_ccSaoComParam.candPos[compIdx], sizeof(prvParam.candPos));
      std::memcpy(prvParam.bandNum, m_ccSaoComParam.bandNum[compIdx], sizeof(prvParam.bandNum));
      std::memcpy(prvParam.offset, m_ccSaoComParam.offset[compIdx], sizeof(prvParam.offset));

      m_ccSaoPrvParamEnc[compIdx].insert(m_ccSaoPrvParamEnc[compIdx].begin(), prvParam);
    }
  }
}
//! \}
