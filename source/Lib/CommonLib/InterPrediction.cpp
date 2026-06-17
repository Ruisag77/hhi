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

/** \file     Prediction.cpp
    \brief    prediction class
*/

#include "InterPrediction.h"

#include "Buffer.h"
#include "UnitTools.h"
#include "MCTS.h"
#include "dtrace_codingstruct.h"
#include "dtrace_buffer.h"

#include <memory.h>
#include <algorithm>
#include "IntraPrediction.h"
#include "Reshape.h"

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

InterPrediction::InterPrediction()
  : m_currChromaFormat(ChromaFormat::UNDEFINED)
  , m_maxCompIDToPred(MAX_NUM_COMP)
  , m_pcRdCost(nullptr)
  , m_storedMv(nullptr)
  , m_gradX0(nullptr)
  , m_gradX1(nullptr)
  , m_gradY0(nullptr)
  , m_gradY1(nullptr)
  , m_absGx(nullptr)
  , m_absGy(nullptr)
  , m_dIx(nullptr)
  , m_dIy(nullptr)
  , m_dI(nullptr)
  , m_signGxGy(nullptr)
  , m_tmpxSample32bit(nullptr)
  , m_tmpySample32bit(nullptr)
  , m_sumAbsGxSample32bit(nullptr)
  , m_sumAbsGySample32bit(nullptr)
  , m_sumDIXSample32bit(nullptr)
  , m_sumDIYSample32bit(nullptr)
  , m_sumSignGyGxSample32bit(nullptr)
  , m_subPuMC(false)
  , m_ibcBufferWidth(0)
  , m_reshape(nullptr)
  , m_storeBeforeLIC(false)
  , m_pcLICRefLeftTemplate {}
  , m_pcLICRefAboveTemplate {}
  , m_pcLICRecLeftTemplate {}
  , m_pcLICRecAboveTemplate {}

  , m_curLICRefLeftTemplate {}
  , m_curLICRefAboveTemplate {}
  , m_curLICRecLeftTemplate {}
  , m_curLICRecAboveTemplate {}

  , m_templateAvailable {}
  , m_fillLicTpl {}

  , m_acPredBeforeLICBuffer {}
  , m_Gx(nullptr)
  , m_Gy(nullptr)
{
  for (uint32_t i = 0; i < 5; i++)
  {
    m_piDotProduct[i] = nullptr;
  }

  for (uint32_t ch = 0; ch < MAX_NUM_COMP; ch++)
  {
    for (uint32_t refList = 0; refList < NUM_RPL01; refList++)
    {
      m_acYuvPred[refList][ch] = nullptr;
    }
  }
  // one vector for each subblock
  for (uint32_t c = 0; c < (MAX_CU_SIZE / DMVR_SUBCU_HEIGHT * MAX_CU_SIZE / DMVR_SUBCU_WIDTH); c++)
  {
    m_dmvrRightBoundary[c]  = nullptr;
    m_dmvrBottomBoundary[c] = nullptr;
  }
  for (uint32_t c = 0; c < MAX_NUM_COMP; c++)
  {
    for (uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; i++)
    {
      for (uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; j++)
      {
        m_filteredBlock[i][j][c] = nullptr;
      }

      m_filteredBlockTmp[i][c] = nullptr;
    }
  }

  for (const auto l: { RPL0, RPL1 })
  {
    m_yuvPredTempDmvr[l] = nullptr;
    for (uint32_t ch = 0; ch < MAX_NUM_COMP; ch++)
    {
      m_refSamplesDmvr[l][ch] = nullptr;
    }
  }

  for (uint32_t i = 0; i < NUM_RPL01; i++)
  {
    m_mcMask[i]       = nullptr;
    m_mcMaskChroma[i] = nullptr;
  }

  int      mvSearchIdxBilMrg = 0;
  uint16_t currtPrio = 0, currIdx = 0;
  ::memset(m_pSearchEnlargeOffsetNum, 0, sizeof(m_pSearchEnlargeOffsetNum));

  for (int y = -BDMVR_INTME_RANGE; y <= BDMVR_INTME_RANGE; y++)
  {
    for (int x = -BDMVR_INTME_RANGE; x <= BDMVR_INTME_RANGE; x++)
    {
      if ((abs(x) + abs(y)) == 0)
      {
        currtPrio                                       = 0;
        currIdx                                         = m_pSearchEnlargeOffsetNum[currtPrio];
        m_pSearchEnlargeOffsetToIdx[currtPrio][currIdx] = mvSearchIdxBilMrg;
        m_costShift_1_bilMrg[mvSearchIdxBilMrg]         = 63;
        m_costShift_2_bilMrg[mvSearchIdxBilMrg++]       = 63;
      }
      else if ((abs(x) + abs(y)) < 4)
      {
        currtPrio                                       = 1;
        currIdx                                         = m_pSearchEnlargeOffsetNum[currtPrio];
        m_pSearchEnlargeOffsetToIdx[currtPrio][currIdx] = mvSearchIdxBilMrg;
        m_costShift_1_bilMrg[mvSearchIdxBilMrg]         = 63;
        m_costShift_2_bilMrg[mvSearchIdxBilMrg++]       = 63;
      }
      else if ((abs(x) + abs(y)) < 7)
      {
        currtPrio                                       = 2;
        currIdx                                         = m_pSearchEnlargeOffsetNum[currtPrio];
        m_pSearchEnlargeOffsetToIdx[currtPrio][currIdx] = mvSearchIdxBilMrg;
        m_costShift_1_bilMrg[mvSearchIdxBilMrg]         = 2;
        m_costShift_2_bilMrg[mvSearchIdxBilMrg++]       = 63;
      }
      else if ((abs(x) + abs(y)) < 11)
      {
        currtPrio                                       = 3;
        currIdx                                         = m_pSearchEnlargeOffsetNum[currtPrio];
        m_pSearchEnlargeOffsetToIdx[currtPrio][currIdx] = mvSearchIdxBilMrg;
        m_costShift_1_bilMrg[mvSearchIdxBilMrg]         = 1;
        m_costShift_2_bilMrg[mvSearchIdxBilMrg++]       = 63;
      }
      else
      {
        currtPrio                                       = 4;
        currIdx                                         = m_pSearchEnlargeOffsetNum[currtPrio];
        m_pSearchEnlargeOffsetToIdx[currtPrio][currIdx] = mvSearchIdxBilMrg;
        m_costShift_1_bilMrg[mvSearchIdxBilMrg]         = 1;
        m_costShift_2_bilMrg[mvSearchIdxBilMrg++]       = 2;
      }

      m_pSearchEnlargeOffset_bilMrg[currtPrio][currIdx] = Mv(x, y);
      m_pSearchEnlargeOffsetNum[currtPrio]++;
    }
  }
  for (auto i = 0; i < BDOF_SUBPU_MAX_NUM; i++)
  {
    m_bdofSubPuMvOffse2[i].setZero();
  }
  CHECK(mvSearchIdxBilMrg != (2 * BDMVR_INTME_RANGE + 1) * (2 * BDMVR_INTME_RANGE + 1),
        "this is wrong, mvSearchIdx_bilMrg != (2 * BDMVR_INTME_RANGE + 1) * (2 * BDMVR_INTME_RANGE + 1)");
}

InterPrediction::~InterPrediction() { destroy(); }

void InterPrediction::destroy()
{
  for (uint32_t i = 0; i < NUM_RPL01; i++)
  {
    for (uint32_t c = 0; c < MAX_NUM_COMP; c++)
    {
      xFree(m_acYuvPred[i][c]);
      m_acYuvPred[i][c] = nullptr;
    }
  }
  // one vector for each subblock
  for (uint32_t c = 0; c < (MAX_CU_SIZE / DMVR_SUBCU_HEIGHT * MAX_CU_SIZE / DMVR_SUBCU_WIDTH); c++)
  {
    xFree(m_dmvrRightBoundary[c]);
    m_dmvrRightBoundary[c] = nullptr;
    xFree(m_dmvrBottomBoundary[c]);
    m_dmvrBottomBoundary[c] = nullptr;
  }
  for (uint32_t c = 0; c < MAX_NUM_COMP; c++)
  {
    for (uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; i++)
    {
      for (uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; j++)
      {
        xFree(m_filteredBlock[i][j][c]);
        m_filteredBlock[i][j][c] = nullptr;
      }

      xFree(m_filteredBlockTmp[i][c]);
      m_filteredBlockTmp[i][c] = nullptr;
    }
  }

  m_geoPartBuf[0].destroy();
  m_geoPartBuf[1].destroy();

  m_colorTransResiBuf.destroy();

  if (m_storedMv != nullptr)
  {
    delete[] m_storedMv;
    m_storedMv = nullptr;
  }

  xFree(m_gradX0);
  m_gradX0 = nullptr;
  xFree(m_gradY0);
  m_gradY0 = nullptr;
  xFree(m_gradX1);
  m_gradX1 = nullptr;
  xFree(m_gradY1);
  m_gradY1 = nullptr;
  xFree(m_absGx);
  m_absGx = nullptr;
  xFree(m_absGy);
  m_absGy = nullptr;
  xFree(m_dIx);
  m_dIx = nullptr;
  xFree(m_dIy);
  m_dIy = nullptr;
  xFree(m_dI);
  m_dI = nullptr;
  xFree(m_signGxGy);
  m_signGxGy = nullptr;
  xFree(m_tmpxSample32bit);
  m_tmpxSample32bit = nullptr;
  xFree(m_tmpySample32bit);
  m_tmpySample32bit = nullptr;
  xFree(m_sumAbsGxSample32bit);
  m_sumAbsGxSample32bit = nullptr;
  xFree(m_sumAbsGySample32bit);
  m_sumAbsGySample32bit = nullptr;
  xFree(m_sumDIXSample32bit);
  m_sumDIXSample32bit = nullptr;
  xFree(m_sumDIYSample32bit);
  m_sumDIYSample32bit = nullptr;
  xFree(m_sumSignGyGxSample32bit);
  m_sumSignGyGxSample32bit = nullptr;
  xFree(m_filteredBlockTmpRPR);
  m_filteredBlockTmpRPR = nullptr;
  for (uint32_t i = 0; i < 5; i++)
  {
    xFree(m_piDotProduct[i]);
    m_piDotProduct[i] = nullptr;
  }

  for (const auto l: { RPL0, RPL1 })
  {
    xFree(m_yuvPredTempDmvr[l]);
    m_yuvPredTempDmvr[l] = nullptr;

    for (uint32_t ch = 0; ch < MAX_NUM_COMP; ch++)
    {
      xFree(m_refSamplesDmvr[l][ch]);
      m_refSamplesDmvr[l][ch] = nullptr;
    }
  }

  for (uint32_t i = 0; i < NUM_RPL01; i++)
  {
    if (m_mcMask[i])
    {
      xFree(m_mcMask[i]);
      m_mcMask[i] = nullptr;
    }

    if (m_mcMaskChroma[i])
    {
      xFree(m_mcMaskChroma[i]);
      m_mcMaskChroma[i] = nullptr;
    }
  }

  m_ibcBuffer.destroy();

  m_obmcTmp1.destroy();
  m_obmcTmp2.destroy();
  m_obmcTmp3.destroy();

  for (int i = 0; i < MAX_NUM_COMP; i++)
  {
    xFree(m_pcLICRefLeftTemplate[0][i]);
    m_pcLICRefLeftTemplate[0][i] = nullptr;
    xFree(m_pcLICRefLeftTemplate[1][i]);
    m_pcLICRefLeftTemplate[1][i] = nullptr;
    xFree(m_pcLICRefAboveTemplate[0][i]);
    m_pcLICRefAboveTemplate[0][i] = nullptr;
    xFree(m_pcLICRefAboveTemplate[1][i]);
    m_pcLICRefAboveTemplate[1][i] = nullptr;
    xFree(m_pcLICRecLeftTemplate[i]);
    m_pcLICRecLeftTemplate[i] = nullptr;
    xFree(m_pcLICRecAboveTemplate[i]);
    m_pcLICRecAboveTemplate[i] = nullptr;

    xFree(m_curLICRefLeftTemplate[0][i]);
    m_curLICRefLeftTemplate[0][i] = nullptr;
    xFree(m_curLICRefLeftTemplate[1][i]);
    m_curLICRefLeftTemplate[1][i] = nullptr;
    xFree(m_curLICRefAboveTemplate[0][i]);
    m_curLICRefAboveTemplate[0][i] = nullptr;
    xFree(m_curLICRefAboveTemplate[1][i]);
    m_curLICRefAboveTemplate[1][i] = nullptr;
    xFree(m_curLICRecLeftTemplate[i]);
    m_curLICRecLeftTemplate[i] = nullptr;
    xFree(m_curLICRecAboveTemplate[i]);
    m_curLICRecAboveTemplate[i] = nullptr;
  }

  xFree(m_Gx);
  m_Gx = nullptr;
  xFree(m_Gy);
  m_Gy = nullptr;
  m_acPredBeforeLICBuffer[0].destroy();
  m_acPredBeforeLICBuffer[1].destroy();
}

void InterPrediction::init(RdCost *pcRdCost, Reshape *pcReshape, ChromaFormat chromaFormatIdc, const int ctuSize,
                           const int picWidth, InterpolationFilter *pcInterpolationFilter)
{
  m_pcRdCost = pcRdCost;
  m_reshape  = pcReshape;
  m_if       = pcInterpolationFilter;
  // if it has been initialised before, but the chroma format has changed, release the memory and start again.
  if (m_acYuvPred[RPL0][COMP_Y] != nullptr && m_currChromaFormat != chromaFormatIdc)
  {
    destroy();
  }

  m_currChromaFormat = chromaFormatIdc;
  if (m_acYuvPred[RPL0][COMP_Y] == nullptr) // check if first is null (in which case, nothing initialised yet)
  {
    // one vector for each subblock
    for (uint32_t c = 0; c < (MAX_CU_SIZE / DMVR_SUBCU_HEIGHT * MAX_CU_SIZE / DMVR_SUBCU_WIDTH); c++)
    {
      m_dmvrRightBoundary[c]  = (Pel *)xMalloc(Pel, DMVR_SUBCU_HEIGHT);
      m_dmvrBottomBoundary[c] = (Pel *)xMalloc(Pel, DMVR_SUBCU_WIDTH);
    }
    for (uint32_t c = 0; c < MAX_NUM_COMP; c++)
    {
#if IF_12TAP
      int extendSize = std::max(2 * BDOF_EXTEND_SIZE + 2, 2 * BDMVR_INTME_RANGE);
      int extWidth   = MAX_CU_SIZE + extendSize + 32;
      int extHeight  = MAX_CU_SIZE + extendSize + 1;
#else
      int extWidth  = MAX_CU_SIZE + std::max(2 * BDOF_EXTEND_SIZE + 18, DMVR_SPAN + 15);
      int extHeight = MAX_CU_SIZE + std::max(2 * BDOF_EXTEND_SIZE + 3, DMVR_SPAN);
#endif

      for (uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; i++)
      {
#if IF_12TAP
        m_filteredBlockTmp[i][c] = (Pel *)xMalloc(Pel, (extWidth + 4) * (extHeight + 15 + 4));
#else
        m_filteredBlockTmp[i][c] = (Pel *)xMalloc(Pel, (extWidth + 4) * (extHeight + 7 + 4));
#endif

        for (uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; j++)
        {
          m_filteredBlock[i][j][c] = (Pel *)xMalloc(Pel, extWidth * extHeight);
        }
      }

      // new structure
      for (uint32_t i = 0; i < NUM_RPL01; i++)
      {
        m_acYuvPred[i][c] = (Pel *)xMalloc(Pel, MAX_CU_SIZE * MAX_CU_SIZE);
      }
    }

    m_geoPartBuf[0].create(UnitArea(chromaFormatIdc, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
    m_geoPartBuf[1].create(UnitArea(chromaFormatIdc, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));

    m_colorTransResiBuf.create(UnitArea(chromaFormatIdc, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));

    m_gradX0                 = (Pel *)xMalloc(Pel, BDOF_TEMP_BUFFER_SIZE);
    m_gradY0                 = (Pel *)xMalloc(Pel, BDOF_TEMP_BUFFER_SIZE);
    m_gradX1                 = (Pel *)xMalloc(Pel, BDOF_TEMP_BUFFER_SIZE);
    m_gradY1                 = (Pel *)xMalloc(Pel, BDOF_TEMP_BUFFER_SIZE);
    m_absGx                  = (Pel *)xMalloc(Pel, BDOF_TEMP_BUFFER_SIZE);
    m_absGy                  = (Pel *)xMalloc(Pel, BDOF_TEMP_BUFFER_SIZE);
    m_dIx                    = (Pel *)xMalloc(Pel, BDOF_TEMP_BUFFER_SIZE);
    m_dIy                    = (Pel *)xMalloc(Pel, BDOF_TEMP_BUFFER_SIZE);
    m_dI                     = (Pel *)xMalloc(Pel, BDOF_TEMP_BUFFER_SIZE);
    m_signGxGy               = (Pel *)xMalloc(Pel, BDOF_TEMP_BUFFER_SIZE);
    m_tmpxSample32bit        = (int *)xMalloc(int, BDOF_SUBPU_SIZE << 4);
    m_tmpySample32bit        = (int *)xMalloc(int, BDOF_SUBPU_SIZE << 4);
    m_sumAbsGxSample32bit    = (int *)xMalloc(int, BDOF_SUBPU_SIZE << 4);
    m_sumAbsGySample32bit    = (int *)xMalloc(int, BDOF_SUBPU_SIZE << 4);
    m_sumDIXSample32bit      = (int *)xMalloc(int, BDOF_SUBPU_SIZE << 4);
    m_sumDIYSample32bit      = (int *)xMalloc(int, BDOF_SUBPU_SIZE << 4);
    m_sumSignGyGxSample32bit = (int *)xMalloc(int, BDOF_SUBPU_SIZE << 4);
    for (uint32_t i = 0; i < 5; i++)
    {
      m_piDotProduct[i] = (int32_t *)xMalloc(int32_t, BDOF_TEMP_BUFFER_SIZE);
    }

    m_filteredBlockTmpRPR = (Pel *)xMalloc(Pel, TMP_RPR_WIDTH * TMP_RPR_HEIGHT);

    m_obmcTmp1.create(UnitArea(chromaFormatIdc, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
    m_obmcTmp2.create(UnitArea(chromaFormatIdc, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
    m_obmcTmp3.create(UnitArea(chromaFormatIdc, Area(0, 0, 4 * MIN_PU_SIZE, MIN_PU_SIZE)));
    m_obmcTmp3.fill(0);
    for (int i = 0; i < 4; i++)
    {
      m_obmcSubPred[i] =
        m_obmcTmp3.subBuf(UnitArea(chromaFormatIdc, Area(i * MIN_PU_SIZE, 0, MIN_PU_SIZE, MIN_PU_SIZE)));
    }
  }

  if (m_yuvPredTempDmvr[RPL0] == nullptr)
  {
    constexpr size_t w0 = DMVR_SUBCU_WIDTH + DMVR_SPAN - 1;
    constexpr size_t h0 = DMVR_SUBCU_HEIGHT + DMVR_SPAN - 1;
    constexpr size_t w1 = DMVR_SUBCU_WIDTH + DMVR_SPAN - 1 + NTAPS_LUMA;
    constexpr size_t h1 = DMVR_SUBCU_HEIGHT + DMVR_SPAN - 1 + NTAPS_LUMA;

    for (const auto l: { RPL0, RPL1 })
    {
      m_yuvPredTempDmvr[l] = (Pel *)xMalloc(Pel, w0 * h0);

      for (uint32_t ch = 0; ch < MAX_NUM_COMP; ch++)
      {
        m_refSamplesDmvr[l][ch] = (Pel *)xMalloc(Pel, w1 * h1);
      }
    }
  }

  if (m_storedMv == nullptr)
  {
    m_storedMv = new Mv[MVBUFFER_SIZE * MVBUFFER_SIZE];
  }
  m_ibcBuffer.destroy();
  if (m_ibcBuffer.bufs.empty())
  {
    m_ibcBufferWidth  = (picWidth + ctuSize - 1) / ctuSize * ctuSize;
    m_ibcBufferHeight = 3 * ctuSize;
    if (256 == ctuSize)
    {
      m_ibcBufferHeight = 2 * ctuSize;
    }
    m_ibcBuffer.create(UnitArea(chromaFormatIdc, Area(0, 0, m_ibcBufferWidth, m_ibcBufferHeight)));
  }

  if (m_mcMask[0] == nullptr)
  {
    for (unsigned i = 0; i < NUM_RPL01; i++)
    {
      m_mcMask[i]       = (bool *)xMalloc(bool, MAX_CU_SIZE *MAX_CU_SIZE);
      m_mcMaskChroma[i] = (bool *)xMalloc(bool, MAX_CU_SIZE *MAX_CU_SIZE);
    }
  }

  if (m_pcLICRefLeftTemplate[0][0] == nullptr)
  {
    for (int i = 0; i < MAX_NUM_COMP; i++)
    {
      m_pcLICRefLeftTemplate[0][i]  = (Pel *)xMalloc(Pel, MAX_CU_SIZE);
      m_pcLICRefLeftTemplate[1][i]  = (Pel *)xMalloc(Pel, MAX_CU_SIZE);
      m_pcLICRefAboveTemplate[0][i] = (Pel *)xMalloc(Pel, MAX_CU_SIZE);
      m_pcLICRefAboveTemplate[1][i] = (Pel *)xMalloc(Pel, MAX_CU_SIZE);
      m_pcLICRecLeftTemplate[i]     = (Pel *)xMalloc(Pel, MAX_CU_SIZE);
      m_pcLICRecAboveTemplate[i]    = (Pel *)xMalloc(Pel, MAX_CU_SIZE);

      m_curLICRefLeftTemplate[0][i]  = (Pel *)xMalloc(Pel, MAX_CU_SIZE);
      m_curLICRefLeftTemplate[1][i]  = (Pel *)xMalloc(Pel, MAX_CU_SIZE);
      m_curLICRefAboveTemplate[0][i] = (Pel *)xMalloc(Pel, MAX_CU_SIZE);
      m_curLICRefAboveTemplate[1][i] = (Pel *)xMalloc(Pel, MAX_CU_SIZE);
      m_curLICRecLeftTemplate[i]     = (Pel *)xMalloc(Pel, MAX_CU_SIZE);
      m_curLICRecAboveTemplate[i]    = (Pel *)xMalloc(Pel, MAX_CU_SIZE);
    }
  }
  m_Gx = (Pel *)xMalloc(Pel, BDOF_TEMP_BUFFER_SIZE);
  m_Gy = (Pel *)xMalloc(Pel, BDOF_TEMP_BUFFER_SIZE);

  if (m_acPredBeforeLICBuffer[0].bufs.empty())
  {
    m_acPredBeforeLICBuffer[0].create(chromaFormatIdc, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE));
    m_acPredBeforeLICBuffer[1].create(chromaFormatIdc, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE));
  }
}

bool InterPrediction::isMvOOB(const Mv &rcMv, const Position pos, const Size size, const SPS *sps, const PPS *pps,
                              bool *mcMask, bool *mcMaskChroma, bool lumaOnly)
{
  return g_pelBufOP.isMvOOB(rcMv, pos, size, sps, pps, mcMask, mcMaskChroma, lumaOnly, m_currChromaFormat);
}
bool InterPrediction::isMvOOBSubBlk(const Mv &rcMv, const Position pos, const Size size, const SPS *sps, const PPS *pps,
                                    bool *mcMask, int mcStride, bool *mcMaskChroma, int mcCStride, bool lumaOnly)
{
  return g_pelBufOP.isMvOOBSubBlk(rcMv, pos, size, sps, pps, mcMask, mcStride, mcMaskChroma, mcCStride, lumaOnly,
                                  m_currChromaFormat);
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

bool InterPrediction::xCheckIdenticalMotion(const CodingUnit &cu)
{
  const Slice &slice = *cu.cs->slice;

  if (slice.isInterB() && !cu.cs->pps->m_useBiWP)
  {
    if (cu.refIdx[0] >= 0 && cu.refIdx[1] >= 0)
    {
      const Picture *refPicL0 = slice.getRefPic(RPL0, cu.refIdx[0]);
      const Picture *refPicL1 = slice.getRefPic(RPL1, cu.refIdx[1]);

      if (refPicL0 == refPicL1)
      {
        if (!cu.affine)
        {
          if (cu.mv[0] == cu.mv[1])
          {
            return true;
          }
        }
        else
        {
          if (cu.mvAffi[0][0] == cu.mvAffi[1][0] && cu.mvAffi[0][1] == cu.mvAffi[1][1] &&
              (cu.affineType == AffineModel::_4_PARAMS || cu.mvAffi[0][2] == cu.mvAffi[1][2]))
          {
            return true;
          }
        }
      }
    }
  }

  return false;
}

void InterPrediction::xSubPuMC(CodingUnit &cu, PelUnitBuf &predBuf, const RefPicList &eRefPicList, const bool luma,
                               const bool chroma)
{
  // compute the location of the current PU
  Position puPos  = cu.lumaPos();
  Size     puSize = cu.lumaSize();

  int numPartLine, numPartCol, puHeight, puWidth;
  {
    numPartLine = std::max(puSize.width >> ATMVP_SUB_BLOCK_SIZE, 1u);
    numPartCol  = std::max(puSize.height >> ATMVP_SUB_BLOCK_SIZE, 1u);
    puHeight    = numPartCol == 1 ? puSize.height : 1 << ATMVP_SUB_BLOCK_SIZE;
    puWidth     = numPartLine == 1 ? puSize.width : 1 << ATMVP_SUB_BLOCK_SIZE;
  }

  CodingUnit subPu = cu;

  subPu.cs        = cu.cs;
  subPu.slice     = cu.slice;
  subPu.mergeType = MergeType::DEFAULT_N;

  bool isAffine = cu.affine;
  subPu.affine  = false;

  // join sub-pus containing the same motion
  bool verMC    = puSize.height > puSize.width;
  int  fstStart = (!verMC ? puPos.y : puPos.x);
  int  secStart = (!verMC ? puPos.x : puPos.y);
  int  fstEnd   = (!verMC ? puPos.y + puSize.height : puPos.x + puSize.width);
  int  secEnd   = (!verMC ? puPos.x + puSize.width : puPos.y + puSize.height);
  int  fstStep  = (!verMC ? puHeight : puWidth);
  int  secStep  = (!verMC ? puWidth : puHeight);

  bool scaled = cu.slice->getRefPic(RPL0, 0)->isRefScaled(cu.cs->pps) ||
    (cu.cs->slice->m_eSliceType == B_SLICE ? cu.slice->getRefPic(RPL1, 0)->isRefScaled(cu.cs->pps) : false);

  m_subPuMC = true;
  for (int fstDim = fstStart; fstDim < fstEnd; fstDim += fstStep)
  {
    for (int secDim = secStart; secDim < secEnd; secDim += secStep)
    {
      int               x     = !verMC ? secDim : fstDim;
      int               y     = !verMC ? fstDim : secDim;
      const MotionInfo &curMi = cu.getMotionInfo(Position { x, y });
      CHECK(curMi.usesLIC, "LIC flag is set for curMi");

      int length = secStep;
      int later  = secDim + secStep;

      while (later < secEnd)
      {
        const MotionInfo &laterMi =
          !verMC ? cu.getMotionInfo(Position { later, fstDim }) : cu.getMotionInfo(Position { fstDim, later });
        CHECK(laterMi.usesLIC, "LIC flag is set for laterMi");
        if (!scaled && laterMi == curMi)
        {
          length += secStep;
        }
        else
        {
          break;
        }
        later += secStep;
      }
      int dx = !verMC ? length : puWidth;
      int dy = !verMC ? puHeight : length;

      subPu.UnitArea::operator=(UnitArea(cu.chromaFormat, Area(x, y, dx, dy)));
      subPu                 = curMi;
      PelUnitBuf subPredBuf = predBuf.subBuf(UnitAreaRelative(cu, subPu));
      subPu.mmvdEncOptMode  = 0;
      motionCompensation(subPu, subPredBuf, eRefPicList, luma, chroma, nullptr, false);
      secDim = later - secStep;
    }
  }
  m_subPuMC = false;

  cu.affine = isAffine;
}

void InterPrediction::xPredInterUni(const CodingUnit &cu, const RefPicList &eRefPicList, PelUnitBuf &pcYuvPred,
                                    const bool bi, const bool bioApplied, const bool luma, const bool chroma,
                                    const bool isBdofMvRefine)
{
  const SPS &sps = *cu.cs->sps;

  int  refIdx = cu.refIdx[eRefPicList];
  Mv   mv[3];
  bool isIBC = false;
  if (CU::isIBC(cu))
  {
    isIBC = true;
  }
  if (cu.affine)
  {
    CHECK(refIdx < 0, "refIdx incorrect.");

    mv[0] = cu.mvAffi[eRefPicList][0];
    mv[1] = cu.mvAffi[eRefPicList][1];
    mv[2] = cu.mvAffi[eRefPicList][2];
  }
  else
  {
    mv[0] = cu.mv[eRefPicList];
  }

  if (!cu.affine)
  {
    if (!isIBC && cu.slice->getRefPic(eRefPicList, refIdx)->isRefScaled(cu.cs->pps) == false)
    {
      if (!cu.cs->pps->m_wrapAroundEnabledFlag)
      {
        if (bioApplied && cu.cs->sps->m_MCBP)
        {
          clipMv(mv[0], cu.lumaPos().offset(-(BDOF_EXTEND_SIZE + 1), -(BDOF_EXTEND_SIZE + 1)), cu.lumaSize(), sps,
                 *cu.cs->pps);
        }
        else
        {
          clipMv(mv[0], cu.lumaPos(), cu.lumaSize(), sps, *cu.cs->pps);
        }
      }
    }
  }

  for (uint32_t comp = COMP_Y; comp < pcYuvPred.bufs.size() && comp <= m_maxCompIDToPred; comp++)
  {
    const CompID compID = CompID(comp);
    if (isLuma(compID) && !luma)
    {
      continue;
    }
    if (isChroma(compID) && !chroma)
    {
      continue;
    }
    if (compID != COMP_Y && bioApplied && isBdofMvRefine)
    {
      continue;
    }
    if (cu.affine)
    {
      CHECK(bioApplied, "BIO is not allowed with affine");
      bool genChromaMv = (!luma && chroma && compID == COMP_Cb);
      xPredAffineBlk(compID, cu, cu.slice->getRefPic(eRefPicList, refIdx)->m_unscaledPic, mv, pcYuvPred, bi,
                     cu.slice->clpRng(compID), eRefPicList, genChromaMv,
                     cu.slice->getScalingRatio(eRefPicList, refIdx));
    }
    else
    {
      if (isIBC)
      {
        xPredInterBlk(compID, cu, cu.slice->m_pic, mv[0], pcYuvPred, bi, cu.slice->clpRng(compID), bioApplied, true,
                      eRefPicList);
      }
      else
      {
        xPredInterBlk(compID, cu, cu.slice->getRefPic(eRefPicList, refIdx)->m_unscaledPic, mv[0], pcYuvPred, bi,
                      cu.slice->clpRng(compID), bioApplied, false, eRefPicList,
                      cu.slice->getScalingRatio(eRefPicList, refIdx));
      }
    }
  }
}

void InterPrediction::xPredInterBiSubPuBDOF(const CodingUnit &cu, PelUnitBuf &pcYuvPred, const bool luma,
                                            const bool chroma)
{
  const Slice &slice      = *cu.cs->slice;
  bool         bioApplied = true;
  // common variable for all subPu
  const bool   lumaOnly = (luma && !chroma), chromaOnly = (!luma && chroma);
  const int    scaleBDOFThres = cu.cs->sps->m_dmvdBDOFExt ? BDOF_SUBPU_AREA_THRESHOLD1 : BDOF_SUBPU_AREA_THRESHOLD0;
  const int    scaleBDOF      = cu.lumaSize().width * cu.lumaSize().height < scaleBDOFThres ? 1 : 2;
  const int    bioDy          = std::min<int>(cu.lumaSize().height, BDOF_SUBPU_DIM * scaleBDOF);
  const int    bioDx          = std::min<int>(cu.lumaSize().width, BDOF_SUBPU_DIM * scaleBDOF);
  PelUnitBuf   srcPred0(cu, m_acYuvPred[0][0], m_acYuvPred[0][1], m_acYuvPred[0][2]);
  PelUnitBuf   srcPred1(cu, m_acYuvPred[1][0], m_acYuvPred[1][1], m_acYuvPred[1][2]);
  Position     cuPos = cu.lumaPos();
  CodingUnit   subCu = cu;

  CHECK(subCu.refIdx[0] < 0, "This is not possible for BDOF");
  CHECK(subCu.refIdx[1] < 0, "This is not possible for BDOF");

  int       bioSubPuIdx        = 0;
  const int bioSubPuStrideIncr = BDOF_SUBPU_STRIDE - std::max(1, (int)(cu.lumaSize().width >> BDOF_SUBPU_DIM_LOG2));

  int bioDx2 = bioDx;
  int bioDy2 = bioDy;
  for (int y = cuPos.y, yStart = 0; y < (cuPos.y + cu.lumaSize().height); y = y + bioDy2, yStart = yStart + bioDy2)
  {
    for (int x = cuPos.x, xStart = 0; x < (cuPos.x + cu.lumaSize().width); x = x + bioDx2, xStart = xStart + bioDx2)
    {
      Mv bioMv = m_bdofSubPuMvOffset[bioSubPuIdx];
      if (cu.bdmvrRefine)
      {
        bioDx2                = bioDx;
        bioDy2                = bioDy;
        int bdmvrSubPuIdxtemp = (yStart >> DMVR_SUBCU_HEIGHT_LOG2) * DMVR_SUBPU_STRIDE;
        while (
          ((x + bioDx2) < (cuPos.x + cu.lumaSize().width)) &&
          (m_bdofSubPuMvOffse2[bioSubPuIdx] == m_bdofSubPuMvOffse2[bioSubPuIdx + (bioDx2 >> BDOF_SUBPU_DIM_LOG2)]) &&
          (m_bdofSubPuMvOffset[bioSubPuIdx] == m_bdofSubPuMvOffset[bioSubPuIdx + (bioDx2 >> BDOF_SUBPU_DIM_LOG2)]) &&
          (!cu.bdmvrRefine || ((xStart >> DMVR_SUBCU_WIDTH_LOG2) == ((xStart + bioDx2) >> DMVR_SUBCU_WIDTH_LOG2)) ||
           ((m_bdmvrSubPuMvBuf[0][bdmvrSubPuIdxtemp + (xStart >> DMVR_SUBCU_WIDTH_LOG2)] ==
             m_bdmvrSubPuMvBuf[0][bdmvrSubPuIdxtemp + ((xStart + bioDx2) >> DMVR_SUBCU_WIDTH_LOG2)]) &&
            (m_bdmvrSubPuMvBuf[1][bdmvrSubPuIdxtemp + (xStart >> DMVR_SUBCU_WIDTH_LOG2)] ==
             m_bdmvrSubPuMvBuf[1][bdmvrSubPuIdxtemp + ((xStart + bioDx2) >> DMVR_SUBCU_WIDTH_LOG2)]))))
        {
          bioDx2 += bioDx;
        }
      }
      else
      {
        bioDx2 = cu.lumaSize().width;
        bioDy2 = cu.lumaSize().height;
      }
      subCu.UnitArea::operator=(UnitArea(cu.chromaFormat, Area(x, y, bioDx2, bioDy2)));

      if (cu.bdmvrRefine)
      {
        const int bdmvrSubPuIdx =
          (yStart >> DMVR_SUBCU_HEIGHT_LOG2) * DMVR_SUBPU_STRIDE + (xStart >> DMVR_SUBCU_WIDTH_LOG2);
        subCu.mv[0] = m_bdmvrSubPuMvBuf[0][bdmvrSubPuIdx] + bioMv;
        subCu.mv[1] = m_bdmvrSubPuMvBuf[1][bdmvrSubPuIdx] - bioMv;
      }
      else
      {
        subCu.mv[0] = cu.mv[0] + bioMv;
        subCu.mv[1] = cu.mv[1] - bioMv;
      }

      const bool lumaPredReady = cu.cs->sps->m_dmvdBDOFExt
        ? (!cu.bdmvrRefine || (m_bdofSubPuMvOffse2[bioSubPuIdx].hor == 0 && m_bdofSubPuMvOffse2[bioSubPuIdx].ver == 0))
        : (bioMv.hor == 0 && bioMv.ver == 0);

      // inter pred to generate buf data
      for (auto refList: { RPL0, RPL1 })
      {
        if (subCu.refIdx[refList] < 0)
        {
          continue;
        }
        RefPicList eRefPicList = refList;

        CHECK(CU::isIBC(subCu) && eRefPicList != RPL0, "Invalid interdir for ibc mode");
        CHECK(CU::isIBC(subCu) && subCu.refIdx[refList] != MAX_NUM_REF, "Invalid reference index for ibc mode");
        CHECK((CU::isInter(subCu) && subCu.refIdx[refList] >= slice.m_numRefIdx[eRefPicList]),
              "Invalid reference index");

        PelUnitBuf pcMbBuf = (refList == RPL0 ? srcPred0 : srcPred1).subBuf(UnitAreaRelative(cu, subCu));

        if (!lumaPredReady || !lumaOnly)
        {
          xPredInterUni(subCu, eRefPicList, pcMbBuf, true, bioApplied, luma && !lumaPredReady, chroma, false);
        }
      }

      // prepare dst sub buf
      PelUnitBuf  subYuvPredBuf = pcYuvPred.subBuf(UnitAreaRelative(cu, subCu));
      CPelUnitBuf srcSubPred0   = srcPred0.subBuf(UnitAreaRelative(cu, subCu));
      CPelUnitBuf srcSubPred1   = srcPred1.subBuf(UnitAreaRelative(cu, subCu));

      // generate the dst buf
      {
        const bool bioAppliedSubCu = lumaPredReady ? false : bioApplied;
        const bool chromaOnlySubCu = lumaPredReady ? true : chromaOnly;

        // do either when luma pred not ready or when luma is ready but chroma is also needed
        if (!lumaPredReady || !lumaOnly)
        {
          bool isOOB[2] = { false, false };
          if (cu.interDir == 3)
          {
            isOOB[0] = isMvOOB(subCu.mv[0], subCu.lumaPos(), subCu.lumaSize(), subCu.slice->m_sps, subCu.slice->m_pps,
                               m_mcMask[0], m_mcMaskChroma[0]);
            isOOB[1] = isMvOOB(subCu.mv[1], subCu.lumaPos(), subCu.lumaSize(), subCu.slice->m_sps, subCu.slice->m_pps,
                               m_mcMask[1], m_mcMaskChroma[1]);
          }
          const int chromaStride = lumaOnly ? 0 : subYuvPredBuf.Cb().width;
          xWeightedAverage(subCu, false /*isBdofMvRefine*/, 0 /*bdofBlockOffset*/, srcSubPred0, srcSubPred1,
                           subYuvPredBuf, bioAppliedSubCu, lumaOnly, chromaOnlySubCu, nullptr, m_mcMask,
                           subYuvPredBuf.Y().width, m_mcMaskChroma, chromaStride, isOOB);
        }
      }
      bioSubPuIdx += (bioDx2 >> BDOF_SUBPU_DIM_LOG2);
    }
    bioSubPuIdx += bioSubPuStrideIncr;
    bioSubPuIdx += ((bioDy >> BDOF_SUBPU_DIM_LOG2) - 1) * BDOF_SUBPU_STRIDE;
  }
}

void InterPrediction::xPredInterBiBDMVR(CodingUnit &cu, PelUnitBuf &pcYuvPred, const bool luma, const bool chroma,
                                        PelUnitBuf *yuvPredTmp /*= NULL*/)
{
  const PPS   &pps   = *cu.cs->pps;
  const Slice &slice = *cu.cs->slice;

  const bool bioApplied = PU::isBIOApplied(cu, slice, pps, m_subPuMC);

  if (!m_subPuMC && cu.cs->sps->m_dmvdBDOFExt)
  {
    Slice *slice = cu.slice;
    bool   alwCond =
      cu.ciipFlag && cu.licFlag && (((slice->m_poc - slice->getRefPOC(RPL0, 0)) == 1) && slice->m_checkLdc);
    bool dontGo = (cu.bcwIdx != BCW_DEFAULT && (yuvPredTmp || (!cu.ciipFlag || alwCond)));
    if (bioApplied && !dontGo && cu.refIdx[0] >= 0 && cu.refIdx[1] >= 0 &&
        ((cu.lwidth() * cu.lheight() < BDOF_SUBPU_AREA_THRESHOLD16) || cu.lwidth() == 4 || cu.lheight() == 4))
    {
      m_bdofMvRefined = true;
      if (luma)
      {
        for (int i = 0; i < cu.lheight() >> BDOF_SUBPU_DIM_LOG2; i++)
        {
          for (int j = 0; j < cu.lwidth() >> BDOF_SUBPU_DIM_LOG2; j++)
          {
            m_bdofSubPuMvOffset[i * BDOF_SUBPU_STRIDE + j].setZero();
            m_bdofSubPuMvOffse2[i * BDOF_SUBPU_STRIDE + j].setZero();
          }
        }
      }
      return;
    }
  }

  // common variable for all subPu
  const int  dy = std::min<int>(cu.lumaSize().height, DMVR_SUBCU_HEIGHT);
  const int  dx = std::min<int>(cu.lumaSize().width, DMVR_SUBCU_WIDTH);
  PelUnitBuf srcPred0(cu, m_acYuvPred[0][0], m_acYuvPred[0][1], m_acYuvPred[0][2]);
  PelUnitBuf srcPred1(cu, m_acYuvPred[1][0], m_acYuvPred[1][1], m_acYuvPred[1][2]);
  Position   puPos = cu.lumaPos();
  CodingUnit subPu = cu;

  int        subPuIdx            = 0;
  const int  dmvrSubPuStrideIncr = DMVR_SUBPU_STRIDE - std::max(1, (int)(cu.lumaSize().width >> DMVR_SUBCU_WIDTH_LOG2));
  int        length = 0, later = 0;
  int        width = cu.lwidth(), height = cu.lheight();
  int        subPuIdxColumn      = 0;
  const auto cuAreaSize          = cu.lumaSize().area();
  const bool reduceSubPuSizeBDOF = cuAreaSize < BDOF_SUBPU_AREA_THRESHOLD0;
  const bool scaleMvBDOF         = cuAreaSize >= BDOF_SUBPU_AREA_THRESHOLD0;

  if (height > width)
  {
    for (int x = puPos.x, xStart = 0; x < (puPos.x + cu.lumaSize().width); x = x + dx, xStart = xStart + dx)
    {
      subPuIdx = subPuIdxColumn;
      for (int y = puPos.y, yStart = 0; y < (puPos.y + cu.lumaSize().height); y = y + dy, yStart = yStart + dy)
      {
        subPu.mv[0] = m_bdmvrSubPuMvBuf[0][subPuIdx];
        subPu.mv[1] = m_bdmvrSubPuMvBuf[1][subPuIdx];
        length      = dy;
        later       = yStart + dy;
        subPuIdx += DMVR_SUBPU_STRIDE;
        while (later < height)
        {
          Mv nextMv[2] = { m_bdmvrSubPuMvBuf[0][subPuIdx], m_bdmvrSubPuMvBuf[1][subPuIdx] };
          if (nextMv[0] == subPu.mv[0] && nextMv[1] == subPu.mv[1])
          {
            length += dy;
          }
          else
          {
            break;
          }

          later += dy;
          subPuIdx += DMVR_SUBPU_STRIDE;
        }
        subPu.UnitArea::operator=(UnitArea(cu.chromaFormat, Area(x, y, dx, length)));
        const int bioSubPuIdx = (xStart >> BDOF_SUBPU_DIM_LOG2) + (yStart >> BDOF_SUBPU_DIM_LOG2) * BDOF_SUBPU_STRIDE;
        xDoPredBiBDMVR(cu, subPu, pcYuvPred, srcPred0, srcPred1, yuvPredTmp, bioApplied, luma, chroma, bioSubPuIdx,
                       scaleMvBDOF, reduceSubPuSizeBDOF);
        yStart = later - dy;
        y      = puPos.y + yStart;
      }
      subPuIdxColumn++;
    }
  }
  else
  {
    for (int y = puPos.y, yStart = 0; y < (puPos.y + cu.lumaSize().height); y = y + dy, yStart = yStart + dy)
    {
      for (int x = puPos.x, xStart = 0; x < (puPos.x + cu.lumaSize().width); x = x + dx, xStart = xStart + dx)
      {
        subPu.mv[0] = m_bdmvrSubPuMvBuf[0][subPuIdx];
        subPu.mv[1] = m_bdmvrSubPuMvBuf[1][subPuIdx];
        length      = dx;
        later       = xStart + dx;
        subPuIdx++;
        while (later < width)
        {
          Mv nextMv[2] = { m_bdmvrSubPuMvBuf[0][subPuIdx], m_bdmvrSubPuMvBuf[1][subPuIdx] };
          if (nextMv[0] == subPu.mv[0] && nextMv[1] == subPu.mv[1])
          {
            length += dx;
          }
          else
          {
            break;
          }

          later += dx;
          subPuIdx++;
        }
        subPu.UnitArea::operator=(UnitArea(cu.chromaFormat, Area(x, y, length, dy)));
        const int bioSubPuIdx = (xStart >> BDOF_SUBPU_DIM_LOG2) + (yStart >> BDOF_SUBPU_DIM_LOG2) * BDOF_SUBPU_STRIDE;
        xDoPredBiBDMVR(cu, subPu, pcYuvPred, srcPred0, srcPred1, yuvPredTmp, bioApplied, luma, chroma, bioSubPuIdx,
                       scaleMvBDOF, reduceSubPuSizeBDOF);
        xStart = later - dx;
        x      = puPos.x + xStart;
      }
      subPuIdx += dmvrSubPuStrideIncr;
    }
  }
}

void InterPrediction::xPredInterBiBDMVR2(CodingUnit &cu, PelUnitBuf &pcYuvPred, const bool luma, const bool chroma,
                                         PelUnitBuf *yuvPredTmp /*= NULL*/, int iter)
{
  const PPS   &pps   = *cu.cs->pps;
  const Slice &slice = *cu.cs->slice;

  const bool bioApplied = PU::isBIOApplied(cu, slice, pps, m_subPuMC);

  // common variable for all subPu
  const int cuAreaSize        = cu.lumaSize().area();
  int       scaleBDOF         = 2;
  int       bdofSubPuAreaThre = (m_subPuMC == true) ? BDOF_SUBPU_AREA_THRESHOLDAFFINE0 : BDOF_SUBPU_AREA_THRESHOLD0;
  if (cuAreaSize < bdofSubPuAreaThre)
  {
    scaleBDOF = 1;
  }
  if (iter == 1 && !m_subPuMC)
  {
    scaleBDOF = 4;
  }

  const int       dy = std::min<int>(cu.lumaSize().height, BDOF_SUBPU_DIM * scaleBDOF);
  const int       dx = std::min<int>(cu.lumaSize().width, BDOF_SUBPU_DIM * scaleBDOF);
  PelUnitBuf      srcPred0(cu, m_acYuvPred[0][0], m_acYuvPred[0][1], m_acYuvPred[0][2]);
  PelUnitBuf      srcPred1(cu, m_acYuvPred[1][0], m_acYuvPred[1][1], m_acYuvPred[1][2]);
  const Position &puPos = cu.lumaPos();
  CodingUnit      subPu = cu;

  int        bioSubPuIdx         = 0;
  const int  width               = cu.lwidth();
  const int  bioSubPuStrideIncr  = BDOF_SUBPU_STRIDE - std::max(1, width >> BDOF_SUBPU_DIM_LOG2);
  const bool reduceSubPuSizeBDOF = cuAreaSize < ((iter == 1) ? BDOF_SUBPU_AREA_THRESHOLD0 : BDOF_SUBPU_AREA_THRESHOLD1);
  const bool scaleMvBDOF = (cuAreaSize < BDOF_SUBPU_AREA_THRESHOLD0) || (cuAreaSize >= BDOF_SUBPU_AREA_THRESHOLD1);
  int        dx2         = dx;
  for (int y = puPos.y, yStart = 0; y < (puPos.y + cu.lumaSize().height); y = y + dy, yStart = yStart + dy)
  {
    for (int x = puPos.x, xStart = 0; x < (puPos.x + cu.lumaSize().width); x = x + dx2, xStart = xStart + dx2)
    {
      dx2                   = dx;
      int bdmvrSubPuIdxtemp = (yStart >> DMVR_SUBCU_HEIGHT_LOG2) * DMVR_SUBPU_STRIDE;
      while (((x + dx2) < (puPos.x + cu.lumaSize().width)) &&
             (m_bdofSubPuMvOffset[bioSubPuIdx] == m_bdofSubPuMvOffset[bioSubPuIdx + (dx2 >> BDOF_SUBPU_DIM_LOG2)]) &&
             (m_bdofSubPuMvOffse2[bioSubPuIdx] == m_bdofSubPuMvOffse2[bioSubPuIdx + (dx2 >> BDOF_SUBPU_DIM_LOG2)]) &&
             (((m_bdmvrSubPuMvBuf[0][bdmvrSubPuIdxtemp + (xStart >> DMVR_SUBCU_WIDTH_LOG2)] ==
                m_bdmvrSubPuMvBuf[0][bdmvrSubPuIdxtemp + ((xStart + dx2) >> DMVR_SUBCU_WIDTH_LOG2)]) &&
               (m_bdmvrSubPuMvBuf[1][bdmvrSubPuIdxtemp + (xStart >> DMVR_SUBCU_WIDTH_LOG2)] ==
                m_bdmvrSubPuMvBuf[1][bdmvrSubPuIdxtemp + ((xStart + dx2) >> DMVR_SUBCU_WIDTH_LOG2)]))))
      {
        dx2 += dx;
      }

      // subPu group size of 12 is not allowed?
      dx2      = (dx2 == 4) ? 4 : (dx2 >> 3) << 3;
      Mv bioMv = m_bdofSubPuMvOffset[bioSubPuIdx];

      if (((iter == BDOF_DMVR_MAX_ITER - 1) || m_subPuMC) && m_bdofSubPuMvOffse2[bioSubPuIdx].hor == 0 &&
          m_bdofSubPuMvOffse2[bioSubPuIdx].ver == 0)
      {
        bioSubPuIdx += std::max(1, (dx2 >> BDOF_SUBPU_DIM_LOG2));
        continue;
      }
      subPu.UnitArea::operator=(UnitArea(cu.chromaFormat, Area(x, y, dx2, dy)));
      const int       bdmvrSubPuIdx =
        (yStart >> DMVR_SUBCU_HEIGHT_LOG2) * DMVR_SUBPU_STRIDE + (xStart >> DMVR_SUBCU_WIDTH_LOG2);
      subPu.mv[0] = m_bdmvrSubPuMvBuf[0][bdmvrSubPuIdx] + bioMv;
      subPu.mv[1] = m_bdmvrSubPuMvBuf[1][bdmvrSubPuIdx] - bioMv;

      xDoPredBiBDMVR(cu, subPu, pcYuvPred, srcPred0, srcPred1, yuvPredTmp, bioApplied, luma, chroma, bioSubPuIdx,
                     scaleMvBDOF, reduceSubPuSizeBDOF, iter);
      bioSubPuIdx += (dx2 >> BDOF_SUBPU_DIM_LOG2);
    }
    bioSubPuIdx += bioSubPuStrideIncr;
    bioSubPuIdx += ((dy >> BDOF_SUBPU_DIM_LOG2) - 1) * BDOF_SUBPU_STRIDE;
  }
}

void InterPrediction::xDoPredBiBDMVR(const CodingUnit &cu, CodingUnit &subPu, PelUnitBuf &pcYuvPred,
                                     PelUnitBuf &srcPred0, PelUnitBuf &srcPred1, PelUnitBuf *yuvPredTmp,
                                     bool bioApplied, bool luma, bool chroma, int bioSubPuIdx, const bool scaleMvBDOF,
                                     const bool reduceSubPuSizeBDOF, int iter)

{
  const PPS   &pps      = *cu.cs->pps;
  const Slice &slice    = *cu.cs->slice;
  const bool   lumaOnly = (luma && !chroma), chromaOnly = (!luma && chroma);

  for (auto eRefPicList: { RPL0, RPL1 })
  {
    if (subPu.refIdx[eRefPicList] < 0)
    {
      continue;
    }

    CHECK(CU::isIBC(subPu) && eRefPicList != RPL0, "Invalid interdir for ibc mode");
    CHECK(CU::isIBC(subPu) && subPu.refIdx[eRefPicList] != MAX_NUM_REF, "Invalid reference index for ibc mode");
    CHECK((CU::isInter(subPu) && subPu.refIdx[eRefPicList] >= slice.m_numRefIdx[eRefPicList]),
          "Invalid reference index");

    PelUnitBuf pcMbBuf = (eRefPicList == RPL0 ? srcPred0 : srcPred1).subBuf(UnitAreaRelative(cu, subPu));

    bool bi = (subPu.refIdx[0] >= 0 && subPu.refIdx[1] >= 0) ||
      (((pps.m_useWP && slice.m_eSliceType == P_SLICE) || (pps.m_useBiWP && slice.m_eSliceType == B_SLICE)) &&
       !subPu.licFlag);
    bool isBdofMvRefineSkipChromaMC = (subPu.refIdx[0] >= 0 && subPu.refIdx[1] >= 0) && (yuvPredTmp == NULL);

    xPredInterUni(subPu, eRefPicList, pcMbBuf, bi, bioApplied, luma, chroma, isBdofMvRefineSkipChromaMC);
  }

  // prepare dst sub buf
  PelUnitBuf  subYuvPredBuf = pcYuvPred.subBuf(UnitAreaRelative(cu, subPu));
  CPelUnitBuf srcSubPred0   = srcPred0.subBuf(UnitAreaRelative(cu, subPu));
  CPelUnitBuf srcSubPred1   = srcPred1.subBuf(UnitAreaRelative(cu, subPu));

  // generate the dst buf
  bool isOOB[2] = { false, false };
  if (cu.interDir == 3)
  {
    isOOB[0] = isMvOOB(subPu.mv[0], subPu.lumaPos(), subPu.lumaSize(), subPu.slice->m_sps, subPu.slice->m_pps,
                       m_mcMask[0], m_mcMaskChroma[0]);
    isOOB[1] = isMvOOB(subPu.mv[1], subPu.lumaPos(), subPu.lumaSize(), subPu.slice->m_sps, subPu.slice->m_pps,
                       m_mcMask[1], m_mcMaskChroma[1]);
  }

  const int chromaStride = lumaOnly ? 0 : subYuvPredBuf.Cb().width;
  xWeightedAverage(subPu, true /*isBdofMvRefine*/, bioSubPuIdx /*bdofBlockOffset*/, srcSubPred0, srcSubPred1,
                   subYuvPredBuf, bioApplied, lumaOnly, chromaOnly, yuvPredTmp, m_mcMask, subYuvPredBuf.Y().width,
                   m_mcMaskChroma, chromaStride, isOOB, scaleMvBDOF, reduceSubPuSizeBDOF, iter);
}

void InterPrediction::xPredInterBi(CodingUnit &cu, PelUnitBuf &pcYuvPred, bool luma, bool chroma,
                                   PelUnitBuf *yuvPredTmp)
{
  const PPS   &pps   = *cu.cs->pps;
  const Slice &slice = *cu.cs->slice;

  if (cu.bdmvrRefine)
  {
    if (yuvPredTmp)   // pre-do MC for yuvPredTmp to avoid MC for yuvPredTmp within the subblock loop
    {
      for (auto eRefPicList: { RPL0, RPL1 })
      {
        CHECK(cu.refIdx[eRefPicList] == NOT_VALID, "pu.refIdx[refList] shouldn't be NOT_VALID.")

        PelUnitBuf pcMbBuf(cu, m_acYuvPred[eRefPicList][0], m_acYuvPred[eRefPicList][1], m_acYuvPred[eRefPicList][2]);
        xPredInterUni(cu, eRefPicList, pcMbBuf, true, false, luma, chroma);
      }

      CPelUnitBuf srcPred0(cu, m_acYuvPred[0][0], m_acYuvPred[0][1], m_acYuvPred[0][2]);
      CPelUnitBuf srcPred1(cu, m_acYuvPred[1][0], m_acYuvPred[1][1], m_acYuvPred[1][2]);

      const bool lumaOnly   = luma && !chroma;
      const bool chromaOnly = !luma && chroma;

      if (pps.m_useBiWP && slice.m_eSliceType == B_SLICE && cu.bcwIdx == BCW_DEFAULT)
      {
        xWeightedPredictionBi(cu, srcPred0, srcPred1, *yuvPredTmp, m_maxCompIDToPred, lumaOnly, chromaOnly);
      }
      else if (pps.m_useWP && slice.m_eSliceType == P_SLICE)
      {
        xWeightedPredictionUni(cu, srcPred0, RPL0, *yuvPredTmp, -1, m_maxCompIDToPred, lumaOnly, chromaOnly);
      }
      else
      {
        bool isOOB[2] = { false, false };
        if (cu.interDir == 3)
        {
          isOOB[0] = isMvOOB(cu.mv[0], cu.lumaPos(), cu.lumaSize(), cu.slice->m_sps, cu.slice->m_pps, m_mcMask[0],
                             m_mcMaskChroma[0]);
          isOOB[1] = isMvOOB(cu.mv[1], cu.lumaPos(), cu.lumaSize(), cu.slice->m_sps, cu.slice->m_pps, m_mcMask[1],
                             m_mcMaskChroma[1]);
        }

        const int chromaStride = lumaOnly ? 0 : yuvPredTmp->Cb().width;
        xWeightedAverage(cu, false /*isBdofMvRefine*/, 0 /*bioSubPuOffset*/, srcPred0, srcPred1, *yuvPredTmp, false,
                         lumaOnly, chromaOnly, nullptr, m_mcMask, yuvPredTmp->Y().width, m_mcMaskChroma, chromaStride,
                         isOOB);
      }

      yuvPredTmp = nullptr;
    }

    xPredInterBiBDMVR(cu, pcYuvPred, luma, chroma, yuvPredTmp);
    if (cu.cs->sps->m_dmvdBDOFExt && m_bdofMvRefined)
    {
      yuvPredTmp = nullptr;
      for (int i = 1; i < BDOF_DMVR_MAX_ITER; i++)
      {
        if (m_subPuMC && (i > 1))
        {
          continue;
        }
        xPredInterBiBDMVR2(cu, pcYuvPred, luma, chroma, yuvPredTmp, i);
      }
    }
    return;
  }

  const int refIdx0 = cu.refIdx[RPL0];
  const int refIdx1 = cu.refIdx[RPL1];

  bool bioApplied = false;

  if (cu.cs->sps->m_bdofEnabledFlag && !cu.cs->picHeader->m_bdofDisabledFlag)
  {
    if (cu.affine || m_subPuMC || cu.licFlag)
    {
      bioApplied = false;
    }
    else
    {
      bioApplied = PU::isSimpleSymmetricBiPred(cu) && !cu.ciipFlag && !cu.smvdMode;
    }
  }
  if (cu.mmvdEncOptMode == 2 && cu.mmvdMergeFlag)
  {
    bioApplied = false;
  }
  if (!luma)
  {
    bioApplied = false;
  }

  bool refIsScaled = (refIdx0 < 0 ? false : cu.slice->getRefPic(RPL0, refIdx0)->isRefScaled(cu.cs->pps)) ||
    (refIdx1 < 0 ? false : cu.slice->getRefPic(RPL1, refIdx1)->isRefScaled(cu.cs->pps));
  bioApplied = bioApplied && !refIsScaled;

  if (yuvPredTmp && bioApplied &&
      (cu.lwidth() > BDOF_SUBPU_DIM ||
       cu.lheight() > BDOF_SUBPU_DIM))   // pre-do MC for yuvPredTmp to avoid MC for yuvPredTmp within the subblock loop
  {
    for (auto eRefPicList: { RPL0, RPL1 })
    {
      CHECK(cu.refIdx[eRefPicList] == NOT_VALID, "pu.refIdx[refList] shouldn't be NOT_VALID.")

      PelUnitBuf pcMbBuf(cu, m_acYuvPred[eRefPicList][0], m_acYuvPred[eRefPicList][1], m_acYuvPred[eRefPicList][2]);
      xPredInterUni(cu, eRefPicList, pcMbBuf, true, false, luma, chroma);
    }

    CPelUnitBuf srcPred0(cu, m_acYuvPred[0][0], m_acYuvPred[0][1], m_acYuvPred[0][2]);
    CPelUnitBuf srcPred1(cu, m_acYuvPred[1][0], m_acYuvPred[1][1], m_acYuvPred[1][2]);

    const bool lumaOnly   = luma && !chroma;
    const bool chromaOnly = !luma && chroma;

    if (pps.m_useBiWP && slice.m_eSliceType == B_SLICE && cu.bcwIdx == BCW_DEFAULT)
    {
      xWeightedPredictionBi(cu, srcPred0, srcPred1, *yuvPredTmp, m_maxCompIDToPred, lumaOnly, chromaOnly);
    }
    else if (pps.m_useWP && slice.m_eSliceType == P_SLICE)
    {
      xWeightedPredictionUni(cu, srcPred0, RPL0, *yuvPredTmp, -1, m_maxCompIDToPred, lumaOnly, chromaOnly);
    }
    else
    {
      bool isOOB[NUM_RPL01] = { false, false };
      if (cu.interDir == 3)
      {
        isOOB[RPL0] = isMvOOB(cu.mv[RPL0], cu.Y().topLeft(), cu.lumaSize(), cu.slice->m_sps, cu.slice->m_pps,
                              m_mcMask[RPL0], m_mcMaskChroma[RPL0]);
        isOOB[RPL1] = isMvOOB(cu.mv[RPL1], cu.Y().topLeft(), cu.lumaSize(), cu.slice->m_sps, cu.slice->m_pps,
                              m_mcMask[RPL1], m_mcMaskChroma[RPL1]);
      }

      const int chromaStride = lumaOnly ? 0 : yuvPredTmp->Cb().width;
      xWeightedAverage(cu, false, 0 /*bioSubPuOffset*/, srcPred0, srcPred1, *yuvPredTmp, false, lumaOnly, chromaOnly,
                       nullptr, m_mcMask, yuvPredTmp->Y().width, m_mcMaskChroma, chromaStride, isOOB);
    }

    yuvPredTmp = nullptr;
  }

  if (cu.licFlag && cu.refIdx[0] >= 0 && cu.refIdx[1] >= 0)
  {
    std::fill_n(&m_templateAvailable[0][0], 2 * MAX_NUM_COMP, false);
  }
  for (auto eRefPicList: { RPL0, RPL1 })
  {
    if (cu.refIdx[eRefPicList] < 0)
    {
      continue;
    }

    CHECK(CU::isIBC(cu) && eRefPicList != RPL0, "Invalid interdir for ibc mode");
    CHECK(CU::isIBC(cu) && cu.refIdx[eRefPicList] != IBC_REF_IDX, "Invalid reference index for ibc mode");
    CHECK((CU::isInter(cu) && cu.refIdx[eRefPicList] >= slice.m_numRefIdx[eRefPicList]), "Invalid reference index");

    PelUnitBuf pcMbBuf(cu, m_acYuvPred[eRefPicList][0], m_acYuvPred[eRefPicList][1], m_acYuvPred[eRefPicList][2]);

    if (cu.refIdx[0] >= 0 && cu.refIdx[1] >= 0)
    {
      bool isBdofMvRefineSkipChromaMC = (yuvPredTmp == NULL);
      xPredInterUni(cu, eRefPicList, pcMbBuf, true, bioApplied, luma, chroma, isBdofMvRefineSkipChromaMC);
    }
    else
    {
      if (((pps.m_useWP && slice.m_eSliceType == P_SLICE) || (pps.m_useBiWP && slice.m_eSliceType == B_SLICE)) &&
          !cu.licFlag)
      {
        xPredInterUni(cu, eRefPicList, pcMbBuf, true, bioApplied, luma, chroma);
      }
      else
      {
        xPredInterUni(cu, eRefPicList, pcMbBuf, cu.geoFlag, bioApplied, luma, chroma);
      }
    }
  }

  CPelUnitBuf srcPred0(cu, m_acYuvPred[0][0], m_acYuvPred[0][1], m_acYuvPred[0][2]);
  CPelUnitBuf srcPred1(cu, m_acYuvPred[1][0], m_acYuvPred[1][1], m_acYuvPred[1][2]);

  const bool lumaOnly   = luma && !chroma;
  const bool chromaOnly = !luma && chroma;

  xLicCompAdj(cu, lumaOnly, chromaOnly);

  if (!cu.geoFlag && (!bioApplied) && pps.m_useBiWP && slice.m_eSliceType == B_SLICE && cu.bcwIdx == BCW_DEFAULT)
  {
    xWeightedPredictionBi(cu, srcPred0, srcPred1, pcYuvPred, m_maxCompIDToPred, lumaOnly, chromaOnly);
    if (yuvPredTmp)
    {
      yuvPredTmp->copyFrom(pcYuvPred);
    }
  }
  else if (!cu.geoFlag && pps.m_useWP && slice.m_eSliceType == P_SLICE)
  {
    xWeightedPredictionUni(cu, srcPred0, RPL0, pcYuvPred, -1, m_maxCompIDToPred, lumaOnly, chromaOnly);
    if (yuvPredTmp)
    {
      yuvPredTmp->copyFrom(pcYuvPred);
    }
  }
  else
  {
    bool isOOB[NUM_RPL01] = { false, false };

    if (cu.interDir == 3)
    {
      if (cu.affine && cu.mergeType != MergeType::SUBPU_ATMVP)   // affine
      {
        isOOB[RPL0] = m_isOOB[RPL0];
        isOOB[RPL1] = m_isOOB[RPL1];
      }
      else
      {
        isOOB[RPL0] = isMvOOB(cu.mv[RPL0], cu.Y().topLeft(), cu.lumaSize(), cu.slice->m_sps, cu.slice->m_pps,
                              m_mcMask[RPL0], m_mcMaskChroma[RPL0]);
        isOOB[RPL1] = isMvOOB(cu.mv[RPL1], cu.Y().topLeft(), cu.lumaSize(), cu.slice->m_sps, cu.slice->m_pps,
                              m_mcMask[RPL1], m_mcMaskChroma[RPL1]);
      }
    }

    const int chromaStride = lumaOnly ? 0 : pcYuvPred.Cb().width;

    xWeightedAverage(cu, true, 0, srcPred0, srcPred1, pcYuvPred, bioApplied, lumaOnly, chromaOnly, yuvPredTmp, m_mcMask,
                     pcYuvPred.Y().width, m_mcMaskChroma, chromaStride, isOOB,
                     (cu.lumaSize().area() >= BDOF_SUBPU_AREA_THRESHOLD0));
    xLicCopyPredBeforeLic(cu, pcYuvPred, luma, chroma, yuvPredTmp, isOOB);
  }
}

void InterPrediction::xPredInterBlk(const CompID compID, const CodingUnit &cu, const Picture *refPic, const Mv &_mv,
                                    PelUnitBuf &dstPic, const bool bi, const ClpRng &clpRng, const bool bioApplied,
                                    bool isIBC, RefPicList refPicList, const ScalingRatio scalingRatio)
{
  JVET_J0090_SET_REF_PICTURE(refPic, compID);
  const ChromaFormat chFmt  = cu.chromaFormat;
  const bool         rndRes = !bi;

  int shiftHor = MV_FRACTIONAL_BITS_INTERNAL + ::getComponentScaleX(compID, chFmt);
  int shiftVer = MV_FRACTIONAL_BITS_INTERNAL + ::getComponentScaleY(compID, chFmt);

  bool wrapRef = false;
  Mv   mv(_mv);
  if (!isIBC && refPic->isWrapAroundEnabled(cu.cs->pps))
  {
    wrapRef = wrapClipMv(mv, cu.blocks[0].pos(), cu.blocks[0].size(), cu.cs->sps, cu.cs->pps);
  }

  bool useAltHpelIf = cu.imv == IMV_HPEL;
  if (isIBC && cu.cs->sps->m_ibcFracFlag)
  {
    CHECK(useAltHpelIf, "IBC does not support IMV_HPEL");
  }

  if (!isIBC && scalingRatio != SCALE_1X &&
      xPredInterBlkRPR(scalingRatio, *cu.cs->pps, CompArea(compID, chFmt, cu.blocks[compID], dstPic.bufs[compID]),
                       refPic, mv, dstPic.bufs[compID].buf, dstPic.bufs[compID].stride, bi, wrapRef, clpRng,
                       InterpolationFilter::Filter::DEFAULT, useAltHpelIf))
  {
    CHECK(bioApplied, "BDOF should be disabled with RPR");
    xLicPredBlk(cu, compID, refPic, &_mv, bi, dstPic.bufs[compID], refPicList);
  }
  else
  {
    int xFrac, yFrac;
    if (isLuma(compID))
    {
      xFrac = mv.hor & 15;
      yFrac = mv.ver & 15;
    }
    else
    {
      xFrac = mv.hor * (1 << (1 - ::getComponentScaleX(compID, chFmt))) & 31;
      yFrac = mv.ver * (1 << (1 - ::getComponentScaleY(compID, chFmt))) & 31;
    }

    const auto filterIdx = isIBC && cu.cs->sps->m_ibcFracFlag && isLuma(compID) && !useAltHpelIf
      ? InterpolationFilter::Filter::IBC
      : (useAltHpelIf ? InterpolationFilter::Filter::HALFPEL_ALT : InterpolationFilter::Filter::DEFAULT);

    uint32_t bvValidType = IBC_BV_INVALID;
    if (isIBC)
    {
      if (cu.cs->sps->m_ibcFracFlag && (xFrac != 0 || yFrac != 0))
      {
        bvValidType =
          checkValidBv(cu, compID, dstPic.bufs[compID].width, dstPic.bufs[compID].height, mv, false, filterIdx, true);
      }
      else
      {
        xFrac = yFrac = 0;
      }
      JVET_J0090_SET_CACHE_ENABLE(false);
    }

    PelBuf  &dstBuf = dstPic.bufs[compID];
    unsigned width  = dstBuf.width;
    unsigned height = dstBuf.height;

    CHECK(dstBuf.Size::operator!=(cu.blocks[compID]) && bioApplied && isLuma(compID), "Incompatible size!");

    Position offset = cu.blocks[compID].pos().offset(mv.getHor() >> shiftHor, mv.getVer() >> shiftVer);

    int refBufExtendSize = 0;
    if (bioApplied && isLuma(compID))
    {
      refBufExtendSize = ((BDOF_EXTEND_SIZE + 1) << 1);   // trick to use SIMD filter
      offset.x -= (BDOF_EXTEND_SIZE + 1);
      offset.y -= (BDOF_EXTEND_SIZE + 1);
    }
    CPelBuf refBuf = refPic->getRecoBuf(
      CompArea(compID, chFmt, offset, Size(width + refBufExtendSize, height + refBufExtendSize)), wrapRef);

    if (isIBC && cu.cs->sps->m_ibcFracFlag && bvValidType == IBC_INT_BV_VALID)
    {
      xPredIBCBlkPadding(cu, compID, refPic, clpRng, refBuf, offset, xFrac, yFrac, (int)width, (int)height, filterIdx);
    }

    // backup data
    Pel      *backupDstBufPtr    = dstBuf.buf;
    ptrdiff_t backupDstBufStride = dstBuf.stride;

    if (bioApplied && isLuma(compID))
    {
      // change MC output
      width         = width + 2 * BDOF_EXTEND_SIZE + 2;
      height        = height + 2 * BDOF_EXTEND_SIZE + 2;
      dstBuf.stride = width;
      dstBuf.buf    = m_filteredBlockTmp[2 + refPicList][compID];
    }

    if (yFrac == 0)
    {
      m_if->filterHor(compID, refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, width, height, xFrac, rndRes,
                      clpRng, filterIdx);
    }
    else if (xFrac == 0)
    {
      m_if->filterVer(compID, refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, width, height, yFrac, true, rndRes,
                      clpRng, filterIdx);
    }
    else
    {
      PelBuf tmpBuf(m_filteredBlockTmp[0][compID], width, width, height);

      int filterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;
      if (isLuma(compID) && filterIdx == InterpolationFilter::Filter::IBC)
      {
        filterSize = NTAPS_LUMA_IBC;
      }
      const int margin = (filterSize >> 1) - 1;

      m_if->filterHor(compID, refBuf.bufAt(0, -margin), refBuf.stride, tmpBuf.buf, tmpBuf.stride, width,
                      height + filterSize - 1, xFrac, false, clpRng, filterIdx);
      JVET_J0090_SET_CACHE_ENABLE(false);
      m_if->filterVer(compID, tmpBuf.bufAt(0, margin), tmpBuf.stride, dstBuf.buf, dstBuf.stride, width, height, yFrac,
                      false, rndRes, clpRng, filterIdx);
    }
    // Enabled only in non-DMVR-non-BDOF process, In DMVR process, srcPadStride is always non-zero
    JVET_J0090_SET_CACHE_ENABLE(srcPadStride == 0 && !bioApplied);

    if (bioApplied && isLuma(compID))
    {
      // restore data
      dstBuf.buf    = backupDstBufPtr;
      dstBuf.stride = backupDstBufStride;
    }

    xLicPredBlk(cu, compID, refPic, &_mv, bi, dstBuf, refPicList);
  }
}

void InterPrediction::xPredIBCBlkPadding(const CodingUnit &pu, const CompID compID, const Picture *refPic,
                                         const ClpRng &clpRng, CPelBuf &refBufBeforePadding,
                                         const Position &refOffsetByIntBv, int xFrac, int yFrac, int width, int height,
                                         InterpolationFilter::Filter filterIdx)
{
  Position               offset = refOffsetByIntBv;
  CPelBuf               &refBuf = refBufBeforePadding;
  const CodingStructure &cs     = *pu.cs;

  const bool        wrapRef    = false;
  const ChannelType chType     = toChannelType(compID);
  const UnitScale  &unitSzLog2 = pu.cs->unitScale[compID];
  const UnitScale   unitSz(1 << unitSzLog2.posx, 1 << unitSzLog2.posy);

  // Get required reference sample area
  int xFilterTap = xFrac == 0 ? 1 : (isLuma(chType) ? NTAPS_LUMA_IBC : NTAPS_CHROMA);
  int yFilterTap = yFrac == 0 ? 1 : (isLuma(chType) ? NTAPS_LUMA_IBC : NTAPS_CHROMA);

  int ibcRefWidth  = width + (xFilterTap - 1);
  int ibcRefHeight = height + (yFilterTap - 1);
  int copyWidth    = ibcRefWidth;

  // Non-normative optimizations
  {
    copyWidth =
      ((ibcRefWidth + 3) >> 2) << 2;   // Note: make it be a multiple of 4, just for enabling SIMD non-normatively
  }

  // Load reference samples to local buffer
  Position addedOffsetTL(-((xFilterTap - 1) >> 1), -((yFilterTap - 1) >> 1));
  Position addedOffsetBR(xFilterTap >> 1, yFilterTap >> 1);
  Position ibcRefOffset = offset.offset(addedOffsetTL);
  CPelBuf  ibcRefBuf;
  ibcRefBuf =
    refPic->getRecoBuf(CompArea(compID, pu.chromaFormat, ibcRefOffset, Size(ibcRefWidth, ibcRefHeight)), wrapRef);

  // PelBuf localRefBuf(m_refSamplesDmvr[RPL0][compID], copyWidth, ibcRefWidth, ibcRefHeight);
  PelBuf localRefBuf(m_filteredBlockTmp[1][compID], copyWidth, ibcRefWidth, ibcRefHeight);
  m_if->filterHor(compID, (Pel *)ibcRefBuf.buf, ibcRefBuf.stride, localRefBuf.buf, localRefBuf.stride, copyWidth,
                  ibcRefHeight, 0, true, clpRng, InterpolationFilter::Filter::DEFAULT);

  refBuf.buf    = localRefBuf.bufAt(-addedOffsetTL.x, -addedOffsetTL.y);
  refBuf.stride = localRefBuf.stride;

  auto getNumSamplesInUnitFirst = [](int firstSamplePos, int unitSz)
  { return unitSz - (firstSamplePos & (unitSz - 1)); };
  auto getNumSamplesInUnitLast = [](int lastSamplePos, int unitSz) { return (lastSamplePos & (unitSz - 1)) + 1; };
  auto sameUnit    = [](int pos1, int pos2, int unitSzLog2) { return (pos1 >> unitSzLog2) == (pos2 >> unitSzLog2); };
  auto notLastUnit = [](int curSamplePos, int lastSamplePos, int unitSzLog2)
  { return ((curSamplePos >> unitSzLog2) << unitSzLog2) + (1 << unitSzLog2) <= lastSamplePos; };
  auto notLastUnitReverse = [](int curSamplePos, int lastSamplePos, int unitSzLog2)
  { return ((curSamplePos >> unitSzLog2) << unitSzLog2) - 1 >= lastSamplePos; };

  // Horizontal padding
  if (xFilterTap > 1)
  {
    bool only1UnitY        = sameUnit(offset.y, offset.y + height - 1, unitSzLog2.posy);
    int  numRowsIn1stUnit  = getNumSamplesInUnitFirst(offset.y, unitSz.posy);
    int  numRowsInLastUnit = getNumSamplesInUnitLast(offset.y + height - 1, unitSz.posy);

    // Padding toward left
    if (addedOffsetTL.x != 0)
    {
      int num1stUnitY  = only1UnitY ? height : numRowsIn1stUnit;
      int numLastUnitY = numRowsInLastUnit;
      int startUnitX   = ((offset.x >> unitSzLog2.posx) << unitSzLog2.posx) - offset.x - 1;
      int startUnitY   = numRowsIn1stUnit - unitSz.posy;

      Pel     *p = (Pel *)refBuf.buf;
      Position pPos(offset.x, offset.y);
      for (int unitY = startUnitY; unitY < height; unitY += unitSz.posy)
      {
        bool padded = false;
        int  numYs  = unitY == startUnitY ? num1stUnitY : ((unitY + unitSz.posy) < height ? unitSz.posy : numLastUnitY);
        for (int unitX = startUnitX; unitX >= addedOffsetTL.x; unitX -= unitSz.posx)
        {
          if (!cs.isDecomp(pPos.offset(unitX, unitY), chType))
          {
            for (int y = 0; y < numYs; ++y, p += refBuf.stride)
            {
              Pel val = p[unitX + 1];
              for (int x = unitX; x >= addedOffsetTL.x; --x)
              {
                p[x] = val;
              }
            }
            padded = true;
            break;
          }
        }
        p += (padded ? 0 : (numYs * refBuf.stride));
      }
    }

    // Padding toward right
    if (addedOffsetBR.x != 0)
    {
      int num1stUnitY  = only1UnitY ? height : numRowsIn1stUnit;
      int numLastUnitY = numRowsInLastUnit;
      int startUnitX =
        (((offset.x + width - 1) >> unitSzLog2.posx) << unitSzLog2.posx) - (offset.x + width - 1) + unitSz.posx;
      int startUnitY = numRowsIn1stUnit - unitSz.posy;

      Pel     *p = (Pel *)refBuf.buf + width - 1;
      Position pPos(offset.x + width - 1, offset.y);
      for (int unitY = startUnitY; unitY < height; unitY += unitSz.posy)
      {
        bool padded = false;
        int  numYs  = unitY == startUnitY ? num1stUnitY : ((unitY + unitSz.posy) < height ? unitSz.posy : numLastUnitY);
        for (int unitX = startUnitX; unitX <= addedOffsetBR.x; unitX += unitSz.posx)
        {
          if (!cs.isDecomp(pPos.offset(unitX, unitY), chType))
          {
            for (int y = 0; y < numYs; ++y, p += refBuf.stride)
            {
              Pel val = p[unitX - 1];
              for (int x = unitX; x <= addedOffsetBR.x; ++x)
              {
                p[x] = val;
              }
            }
            padded = true;
            break;
          }
        }
        p += (padded ? 0 : (numYs * refBuf.stride));
      }
    }
  }

  // Vertical padding
  if (yFilterTap > 1)
  {
    bool only1UnitX =
      sameUnit(offset.x + addedOffsetTL.x, offset.x + addedOffsetTL.x + ibcRefWidth - 1, unitSzLog2.posx);
    int numColsIn1stUnit  = getNumSamplesInUnitFirst(offset.x + addedOffsetTL.x, unitSz.posx);
    int numColsInLastUnit = getNumSamplesInUnitLast(offset.x + addedOffsetTL.x + ibcRefWidth - 1, unitSz.posx);

    // Padding toward above
    if (addedOffsetTL.y != 0)
    {
      int  num1stUnitX  = only1UnitX ? ibcRefWidth : numColsIn1stUnit;
      int  numLastUnitX = numColsInLastUnit;
      bool only1UnitY   = sameUnit(offset.y - 1, offset.y + addedOffsetTL.y, unitSzLog2.posy);
      int  num1stUnitY =
        only1UnitY ? -addedOffsetTL.y : getNumSamplesInUnitLast(offset.y - 1, unitSz.posy);   // Note: scan upward
      int numLastUnitY = only1UnitY ? num1stUnitY : getNumSamplesInUnitFirst(offset.y + addedOffsetTL.y, unitSz.posy);
      int startUnitX   = numColsIn1stUnit - unitSz.posx;
      int startUnitY   = only1UnitY ? -1 : (unitSz.posy - 1) - num1stUnitY;

      Pel     *p = (Pel *)refBuf.buf + addedOffsetTL.x;
      Position pPos(offset.x + addedOffsetTL.x, offset.y);
      for (int unitY = startUnitY; unitY >= addedOffsetTL.y; unitY -= unitSz.posy)
      {
        Pel *pPrev = p;
        int  numYs = unitY == startUnitY
           ? num1stUnitY
           : (notLastUnitReverse(pPos.y + unitY, pPos.y + addedOffsetTL.y, unitSzLog2.posy) ? unitSz.posy
                                                                                            : numLastUnitY);
        for (int unitX = startUnitX; unitX < ibcRefWidth; unitX += unitSz.posx)
        {
          int numXs = unitX == startUnitX
            ? num1stUnitX
            : (notLastUnit(pPos.x + unitX, pPos.x + ibcRefWidth - 1, unitSzLog2.posx) ? unitSz.posx : numLastUnitX);
          if (!cs.isDecomp(pPos.offset(unitX, unitY), chType))
          {
            unitX += unitSz.posx;
            for (; unitX < ibcRefWidth; unitX += unitSz.posx)
            {
              if (cs.isDecomp(pPos.offset(unitX, unitY), chType))
              {
                break;
              }
              numXs +=
                (notLastUnit(pPos.x + unitX, pPos.x + ibcRefWidth - 1, unitSzLog2.posx) ? unitSz.posx : numLastUnitX);
            }

            Pel      *pCur  = pPrev - refBuf.stride;
            const int memSz = numXs * sizeof(Pel);
            for (int y = 0; y < numYs; ++y, pCur -= refBuf.stride)
            {
              memcpy(pCur, pPrev, memSz);
            }

            numXs += unitSz.posx;   // Note: unitX already iterates to next unit, and thus numXs should be adjusted to
                                    // make pPrev align with unitX
          }
          pPrev += numXs;
        }
        p -= (numYs * refBuf.stride);
      }
    }

    // Padding toward bottom
    if (addedOffsetBR.y != 0)
    {
      int  num1stUnitX  = only1UnitX ? ibcRefWidth : numColsIn1stUnit;
      int  numLastUnitX = numColsInLastUnit;
      bool only1UnitY   = sameUnit(offset.y + height, offset.y + height - 1 + addedOffsetBR.y, unitSzLog2.posy);
      int  num1stUnitY  = only1UnitY ? addedOffsetBR.y : getNumSamplesInUnitFirst(offset.y + height, unitSz.posy);
      int  numLastUnitY =
        only1UnitY ? num1stUnitY : getNumSamplesInUnitLast(offset.y + height - 1 + addedOffsetBR.y, unitSz.posy);
      int startUnitX = numColsIn1stUnit - unitSz.posx;
      int startUnitY = only1UnitY ? 1 : num1stUnitY - (unitSz.posy - 1);

      Pel     *p = (Pel *)refBuf.buf + addedOffsetTL.x + (height - 1) * refBuf.stride;
      Position pPos(offset.x + addedOffsetTL.x, offset.y + height - 1);
      for (int unitY = startUnitY; unitY <= addedOffsetBR.y; unitY += unitSz.posy)
      {
        Pel *pPrev = p;
        int  numYs = unitY == startUnitY
           ? num1stUnitY
           : (notLastUnit(pPos.y + unitY, pPos.y + addedOffsetBR.y, unitSzLog2.posy) ? unitSz.posy : numLastUnitY);
        for (int unitX = startUnitX; unitX < ibcRefWidth; unitX += unitSz.posx)
        {
          int numXs = unitX == startUnitX
            ? num1stUnitX
            : (notLastUnit(pPos.x + unitX, pPos.x + ibcRefWidth - 1, unitSzLog2.posx) ? unitSz.posx : numLastUnitX);
          if (!cs.isDecomp(pPos.offset(unitX, unitY), chType))
          {
            unitX += unitSz.posx;
            for (; unitX < ibcRefWidth; unitX += unitSz.posx)
            {
              if (cs.isDecomp(pPos.offset(unitX, unitY), chType))
              {
                break;
              }
              numXs +=
                (notLastUnit(pPos.x + unitX, pPos.x + ibcRefWidth - 1, unitSzLog2.posx) ? unitSz.posx : numLastUnitX);
            }

            Pel      *pCur  = pPrev + refBuf.stride;
            const int memSz = numXs * sizeof(Pel);
            for (int y = 0; y < numYs; ++y, pCur += refBuf.stride)
            {
              memcpy(pCur, pPrev, memSz);
            }

            numXs += unitSz.posx;   // Note: unitX already iterates to next unit, and thus numXs should be adjusted to
                                    // make pPrev align with unitX
          }
          pPrev += numXs;
        }
        p += (numYs * refBuf.stride);
      }
    }
  }
}

void InterPrediction::xPredAffineBlk(const CompID &compID, const CodingUnit &cu, const Picture *refPic, const Mv *_mv,
                                     PelUnitBuf &dstPic, const bool bi, const ClpRng &clpRng, RefPicList eRefPicList,
                                     bool genChromaMv, const ScalingRatio scalingRatio, const bool calGradient)
{
  PROFILER_SCOPE(1, g_timeProfiler, P_MOTION_COMP_AFFINE);
  JVET_J0090_SET_REF_PICTURE(refPic, compID);

  const ChromaFormat    &chFmt = cu.chromaFormat;
  const CodingStructure &cs    = *cu.cs;
  const SPS             &sps   = *cs.sps;
  const PPS             &pps   = *cs.pps;
  const PicHeader       &ph    = *cs.picHeader;

  const int widthLuma  = cu.Y().width;
  const int heightLuma = cu.Y().height;

  int sbWidth  = AFFINE_SUBBLOCK_SIZE;
  int sbHeight = AFFINE_SUBBLOCK_SIZE;

  const bool isRefScaled = refPic->isRefScaled(&pps);

  const int dstExtW = (sbWidth + PROF_BORDER_EXT_W * 2 + 7) & ~7;
  const int dstExtH = sbHeight + PROF_BORDER_EXT_H * 2;
  PelBuf    dstExtBuf(m_filteredBlockTmp[1][compID], dstExtW, dstExtH);

  PelBuf &dstBuf = dstPic.bufs[compID];

  const bool wrapAroundEnabled = refPic->isWrapAroundEnabled(&pps);

  int dmvHorX = 0;
  int dmvHorY = 0;
  int dmvVerX = 0;
  int dmvVerY = 0;

  constexpr int PREC = MAX_CU_DEPTH;

  bool enableProfTmp = isLuma(compID) && !ph.m_profDisabledFlag && !isRefScaled && !m_skipProf;
  enableProfTmp &= ((cu.mmvdEncOptMode & 3) != 3);   // encoder-only

  if (isLuma(compID) || genChromaMv || chFmt == ChromaFormat::_444)
  {
    const Mv &mvLT = _mv[0];
    const Mv &mvRT = _mv[1];
    const Mv &mvLB = _mv[2];

    dmvHorX = (mvRT - mvLT).getHor() * (1 << (PREC - floorLog2(widthLuma)));
    dmvHorY = (mvRT - mvLT).getVer() * (1 << (PREC - floorLog2(widthLuma)));
    if (cu.affineType == AffineModel::_6_PARAMS)
    {
      dmvVerX = (mvLB - mvLT).getHor() * (1 << (PREC - floorLog2(heightLuma)));
      dmvVerY = (mvLB - mvLT).getVer() * (1 << (PREC - floorLog2(heightLuma)));
    }
    else
    {
      dmvVerX = -dmvHorY;
      dmvVerY = dmvHorX;
    }

    if (dmvHorX == 0 && dmvHorY == 0 && dmvVerX == 0 && dmvVerY == 0)
    {
      enableProfTmp = false;
    }

    const int baseHor = mvLT.getHor() * (1 << PREC);
    const int baseVer = mvLT.getVer() * (1 << PREC);

    for (int h = 0; h < heightLuma; h += sbHeight)
    {
      for (int w = 0; w < widthLuma; w += sbWidth)
      {
        const int weightHor = (sbWidth >> 1) + w;
        const int weightVer = (sbHeight >> 1) + h;

        Mv tmpMv;

        tmpMv.hor = baseHor + dmvHorX * weightHor + dmvVerX * weightVer;
        tmpMv.ver = baseVer + dmvHorY * weightHor + dmvVerY * weightVer;

        tmpMv >>= PREC - 4 + MV_FRACTIONAL_BITS_INTERNAL;
        tmpMv.clipToStorageBitDepth();

        m_storedMv[h / AFFINE_SUBBLOCK_SIZE * MVBUFFER_SIZE + w / AFFINE_SUBBLOCK_SIZE] = tmpMv;
      }
    }

    if (enableProfTmp && m_skipProfCond)
    {
      const int profThres = (m_biPredSearchAffine ? 2 : 1) << PREC;

      if (abs(dmvHorX) <= profThres && abs(dmvHorY) <= profThres && abs(dmvVerX) <= profThres &&
          abs(dmvVerY) <= profThres)
      {
        // Skip PROF during search when the adjustment is expected to be small
        enableProfTmp = false;
      }
    }
  }

  const bool enableProf = enableProfTmp;
  const bool isLast     = !(enableProf || calGradient) && !bi;

  int dMvScaleHor[AFFINE_SUBBLOCK_SIZE * AFFINE_SUBBLOCK_SIZE];
  int dMvScaleVer[AFFINE_SUBBLOCK_SIZE * AFFINE_SUBBLOCK_SIZE];

  if (enableProf)
  {
    for (int y = 0; y < sbHeight; y++)
    {
      for (int x = 0; x < sbWidth; x++)
      {
        const int wx = 2 * x - (sbWidth - 1);
        const int wy = 2 * y - (sbHeight - 1);

        dMvScaleHor[y * sbWidth + x] = wx * dmvHorX + wy * dmvVerX;
        dMvScaleVer[y * sbWidth + x] = wx * dmvHorY + wy * dmvVerY;
      }
    }

    // NOTE: the shift value is 7 and not 8 as in section 8.5.5.9 of the spec because
    // the values dMvScaleHor/dMvScaleVer are half of diffMvLX (which are always even
    // in equations 876 and 877)
    const int mvShift  = MAX_CU_DEPTH;
    const int dmvLimit = (1 << 5) - 1;

    const int sz = sbWidth * sbHeight;

    if (!g_pelBufOP.roundIntVector)
    {
      for (int idx = 0; idx < sz; idx++)
      {
        roundAffineMv(dMvScaleHor[idx], dMvScaleVer[idx], mvShift);
        dMvScaleHor[idx] = Clip3(-dmvLimit, dmvLimit, dMvScaleHor[idx]);
        dMvScaleVer[idx] = Clip3(-dmvLimit, dmvLimit, dMvScaleVer[idx]);
      }
    }
    else
    {
      g_pelBufOP.roundIntVector(dMvScaleHor, sz, mvShift, dmvLimit);
      g_pelBufOP.roundIntVector(dMvScaleVer, sz, mvShift, dmvLimit);
    }
  }
  else if (calGradient)
  {
    ::memset(dMvScaleHor, 0, sizeof(dMvScaleHor));
    ::memset(dMvScaleVer, 0, sizeof(dMvScaleVer));
  }

  // get prediction block by block
  const int scaleX = ::getComponentScaleX(compID, chFmt);
  const int scaleY = ::getComponentScaleY(compID, chFmt);

  const int width  = widthLuma >> scaleX;
  const int height = heightLuma >> scaleY;

  if (sbWidth > width)
  {
    sbWidth = width;
  }
  if (sbHeight > height)
  {
    sbHeight = height;
  }

  // OOB mask
  if ((isLuma(compID) || genChromaMv) && cu.interDir == 3)
  {
    bool *pMcMask = m_mcMask[eRefPicList];
    memset(pMcMask, false, widthLuma * heightLuma);
    bool *pMcMaskChroma  = m_mcMaskChroma[eRefPicList];
    int   chromaScale    = getComponentScaleX(COMP_Cb, m_currChromaFormat);
    int   cxWidthChroma  = widthLuma >> chromaScale;
    int   cxHeightChroma = heightLuma >> chromaScale;
    memset(pMcMaskChroma, false, cxWidthChroma * cxHeightChroma);
    m_isOOB[eRefPicList] = false;

    for (int h = 0; h < heightLuma; h += sbHeight)
    {
      for (int w = 0; w < widthLuma; w += sbWidth)
      {
        int   chromaScale = getComponentScaleX(COMP_Cb, m_currChromaFormat);
        bool *pMcMask     = m_mcMask[eRefPicList] + w + h * widthLuma;
        bool *pMcMaskChroma =
          m_mcMaskChroma[eRefPicList] + (w >> chromaScale) + (h >> chromaScale) * (widthLuma >> chromaScale);
        int             cxWidthChroma = widthLuma >> chromaScale;
        const ptrdiff_t idx           = h / AFFINE_SUBBLOCK_SIZE * MVBUFFER_SIZE + w / AFFINE_SUBBLOCK_SIZE;

        m_isOOB[eRefPicList] |=
          isMvOOBSubBlk(m_storedMv[idx], Position(cu.lx() + w, cu.ly() + h), Size(sbWidth, sbHeight), cu.slice->m_sps,
                        cu.slice->m_pps, pMcMask, widthLuma, pMcMaskChroma, cxWidthChroma);
      }
    }
  }

  Pel *const refLeftTemplate  = m_pcLICRefLeftTemplate[eRefPicList][compID];
  Pel *const refAboveTemplate = m_pcLICRefAboveTemplate[eRefPicList][compID];
  Pel *const recLeftTemplate  = m_pcLICRecLeftTemplate[compID];
  Pel *const recAboveTemplate = m_pcLICRecAboveTemplate[compID];
  bool const licCondition     = cu.licFlag && (m_encMotionEstimation || PU::checkRprLicCondition(cu));

  for (int h = 0; h < height; h += sbHeight)
  {
    for (int w = 0; w < width; w += sbWidth)
    {
      Mv curMv;

      const int hLuma = h << scaleY;
      const int wLuma = w << scaleX;

      const ptrdiff_t idx = hLuma / AFFINE_SUBBLOCK_SIZE * MVBUFFER_SIZE + wLuma / AFFINE_SUBBLOCK_SIZE;

      if (isLuma(compID) || chFmt == ChromaFormat::_444)
      {
        curMv = m_storedMv[idx];
      }
      else
      {
        if (width < 4)
        {
          curMv = m_storedMv[idx] + m_storedMv[idx + scaleY * MVBUFFER_SIZE];
        }
        else if (height < 4)
        {
          curMv = m_storedMv[idx] + m_storedMv[idx + scaleX];
        }
        else
        {
          curMv = m_storedMv[idx] + m_storedMv[idx + scaleY * MVBUFFER_SIZE + scaleX];
        }
        curMv >>= 1;
      }

      bool wrapRef = false;

      if (wrapAroundEnabled)
      {
        wrapRef = wrapClipMv(curMv, Position(cu.lx() + wLuma, cu.ly() + hLuma),
                             Size(sbWidth << scaleX, sbHeight << scaleY), &sps, &pps);
      }
      else if (!isRefScaled)
      {
        clipMv(curMv, cu.lumaPos(), cu.lumaSize(), sps, pps);
      }

      const auto filterIdx = InterpolationFilter::Filter::AFFINE;

      if (isRefScaled)
      {
        CHECK(enableProf, "PROF should be disabled with RPR");
        xPredInterBlkRPR(scalingRatio, pps,
                         CompArea(compID, chFmt, cu.blocks[compID].offset(w, h), Size(sbWidth, sbHeight)), refPic,
                         curMv, dstBuf.buf + w + h * dstBuf.stride, dstBuf.stride, bi, wrapRef, clpRng, filterIdx);
      }
      else
      {
        if (licCondition && (w == 0 || h == 0))
        {
          CHECK(bi && (eRefPicList == 1) && !m_fillLicTpl[compID], "unexpected");
          xGetSublkTemplate(cu, compID, *refPic, curMv, sbWidth, sbHeight, w, h, m_templateAvailable[compID],
                            refLeftTemplate, refAboveTemplate, recLeftTemplate, recAboveTemplate);
        }
        // get the MV in high precision
        int xFrac, yFrac, xInt, yInt;

        if (isLuma(compID))
        {
          xInt  = curMv.getHor() >> MV_FRAC_BITS_LUMA;
          xFrac = curMv.getHor() & MV_FRAC_MASK_LUMA;
          yInt  = curMv.getVer() >> MV_FRAC_BITS_LUMA;
          yFrac = curMv.getVer() & MV_FRAC_MASK_LUMA;
        }
        else
        {
          xInt  = curMv.getHor() * (1 << (1 - scaleX)) >> MV_FRAC_BITS_CHROMA;
          xFrac = curMv.getHor() * (1 << (1 - scaleX)) & MV_FRAC_MASK_CHROMA;
          yInt  = curMv.getVer() * (1 << (1 - scaleY)) >> MV_FRAC_BITS_CHROMA;
          yFrac = curMv.getVer() * (1 << (1 - scaleY)) & MV_FRAC_MASK_CHROMA;
        }

        const CPelBuf refBuf = refPic->getRecoBuf(
          CompArea(compID, chFmt, cu.blocks[compID].offset(xInt + w, yInt + h), cu.blocks[compID]), wrapRef);

        const Pel      *ref       = refBuf.buf;
        const ptrdiff_t refStride = refBuf.stride;

        Pel      *dst = nullptr;
        ptrdiff_t dstStride;
        if (enableProf || calGradient)
        {
          dst       = dstExtBuf.bufAt(PROF_BORDER_EXT_W, PROF_BORDER_EXT_H);
          dstStride = dstExtBuf.stride;
        }
        else
        {
          dst       = dstBuf.buf + w + h * dstBuf.stride;
          dstStride = dstBuf.stride;
        }

        if (xFrac && yFrac && sbWidth >= 4 && sbHeight >= 4)
        {
          m_if->filter4x4(compID, ref, refStride, dst, dstStride, 4, 4, xFrac, yFrac, isLast, chFmt, clpRng);
        }
        else if (xFrac && yFrac)
        {
          CHECK(isLuma(compID), "should not happen!");
          const int refExtH = dstExtH + MAX_FILTER_SIZE - 1;
          PelBuf    tmpBuf  = PelBuf(m_filteredBlockTmp[0][compID], dstExtW, refExtH);

          int vFilterSize = NTAPS_CHROMA;

          m_if->filterHor(compID, (Pel *)ref - ((vFilterSize >> 1) - 1) * refStride, refStride, tmpBuf.buf,
                          tmpBuf.stride, sbWidth, sbHeight + vFilterSize - 1, xFrac, false, cu.slice->clpRng(compID),
                          filterIdx);
          JVET_J0090_SET_CACHE_ENABLE(false);
          m_if->filterVer(compID, tmpBuf.buf + ((vFilterSize >> 1) - 1) * tmpBuf.stride, tmpBuf.stride, dst, dstStride,
                          sbWidth, sbHeight, yFrac, false, false /*rndRes=!bi*/, cu.slice->clpRng(compID), filterIdx);
        }
        else if (yFrac == 0)
        {
          m_if->filterHor(compID, ref, refStride, dst, dstStride, sbWidth, sbHeight, xFrac, isLast, clpRng, filterIdx);
        }
        else   // (xFrac == 0)
        {
          m_if->filterVer(compID, ref, refStride, dst, dstStride, sbWidth, sbHeight, yFrac, true, isLast, clpRng,
                          filterIdx);
        }

        if (enableProf || calGradient)
        {
          const int shift = IF_INTERNAL_FRAC_BITS(clpRng.bd);
          CHECKD(shift < 0, "shift must be positive");
          const int xOffset = xFrac >> (MV_FRAC_BITS_LUMA - 1);
          const int yOffset = yFrac >> (MV_FRAC_BITS_LUMA - 1);

          // NOTE: corners don't need to be padded
          const Pel *refPel = ref + yOffset * refStride + xOffset;
          Pel       *dstPel = dst;

          for (ptrdiff_t x = 0; x < sbWidth; x++)
          {
            const ptrdiff_t refOffset = sbHeight * refStride;
            const ptrdiff_t dstOffset = sbHeight * dstStride;

            dstPel[x - dstStride] = (refPel[x - refStride] << shift) - IF_INTERNAL_OFFS;
            dstPel[x + dstOffset] = (refPel[x + refOffset] << shift) - IF_INTERNAL_OFFS;
          }

          for (int y = 0; y < sbHeight; y++, refPel += refStride, dstPel += dstStride)
          {
            dstPel[-1]      = (refPel[-1] << shift) - IF_INTERNAL_OFFS;
            dstPel[sbWidth] = (refPel[sbWidth] << shift) - IF_INTERNAL_OFFS;
          }

          static_assert(
            PROF_BORDER_EXT_H <= BDOF_EXTEND_SIZE,
            "reuse BIO buffer for PROF, but PROF border height extension greater than BIO border extension");
          static_assert(PROF_BORDER_EXT_W <= BDOF_EXTEND_SIZE,
                        "reuse BIO buffer for PROF, but PROF border width extension greater than BIO border extension");

          const ptrdiff_t strideGrad = width;
          Pel            *gX         = m_gradX0 + h * width + w;
          Pel            *gY         = m_gradY0 + h * width + w;

          g_pelBufOP.profGradFilter(dstExtBuf.buf, dstExtBuf.stride, AFFINE_SUBBLOCK_WIDTH_EXT,
                                    AFFINE_SUBBLOCK_HEIGHT_EXT, strideGrad, gX, gY, clpRng.bd);

          const Pel offset = (1 << shift >> 1) + IF_INTERNAL_OFFS;

          Pel *src  = dst;
          Pel *dstY = dstBuf.bufAt(w, h);
          gX += PROF_BORDER_EXT_H * strideGrad + PROF_BORDER_EXT_W;
          gY += PROF_BORDER_EXT_H * strideGrad + PROF_BORDER_EXT_W;

          g_pelBufOP.applyPROF(dstY, dstBuf.stride, src, dstExtBuf.stride, sbWidth, sbHeight, gX, gY, strideGrad,
                               dMvScaleHor, dMvScaleVer, sbWidth, bi, shift, offset, clpRng);
        }
      }
    }
  }
  if (!isRefScaled && licCondition)
  {
    if (!bi || eRefPicList == RPL0)
    {
      m_fillLicTpl[compID] = true;
    }
  }
  xLicPredBlk(cu, compID, refPic, nullptr, bi, dstPic.bufs[compID], eRefPicList);
}

template<bool dmvdBDOFExt>
void InterPrediction::xStoreBDOFMvDataExt(Mv &bioMv, const int bdofBlockOffset, const int bioSubPuMvIndex,
                                          const int bioDx, const int bioDy, const bool scaleMvBDOF, const int iter)
{
  if (dmvdBDOFExt)
  {
    if (iter == 0)
    {
      if (!m_subPuMC || scaleMvBDOF)
      {
        bioMv >>= 1;
      }
      for (int i = 0; i < (bioDy >> BDOF_SUBPU_DIM_LOG2); i++)
      {
        for (int j = 0; j < (bioDx >> BDOF_SUBPU_DIM_LOG2); j++)
        {
          m_bdofSubPuMvOffset[bdofBlockOffset + bioSubPuMvIndex + i * BDOF_SUBPU_STRIDE + j] = bioMv;
          m_bdofSubPuMvOffse2[bdofBlockOffset + bioSubPuMvIndex + i * BDOF_SUBPU_STRIDE + j] = bioMv;
        }
      }
    }
    else
    {
      if (iter == 1 && !m_subPuMC && scaleMvBDOF)
      {
        bioMv >>= 1;
      }
      for (int i = 0; i < (bioDy >> BDOF_SUBPU_DIM_LOG2); i++)
      {
        for (int j = 0; j < (bioDx >> BDOF_SUBPU_DIM_LOG2); j++)
        {
          m_bdofSubPuMvOffset[bdofBlockOffset + bioSubPuMvIndex + i * BDOF_SUBPU_STRIDE + j] += bioMv;
          m_bdofSubPuMvOffse2[bdofBlockOffset + bioSubPuMvIndex + i * BDOF_SUBPU_STRIDE + j] = bioMv;
        }
      }
    }
  }
  else
  {
    for (int i = 0; i < (bioDy >> BDOF_SUBPU_DIM_LOG2); i++)
    {
      for (int j = 0; j < (bioDx >> BDOF_SUBPU_DIM_LOG2); j++)
      {
        m_bdofSubPuMvOffset[bdofBlockOffset + bioSubPuMvIndex + i * BDOF_SUBPU_STRIDE + j] = bioMv;
      }
    }
  }
}

template<bool dmvdBDOFExt> void InterPrediction::xResetBDOFMvData(const int bdofBlockOffset, const int bioSubPuMvIndex,
                                                                  const int bioDx, const int bioDy, int iter)
{
  if (dmvdBDOFExt)
  {
    if (iter == 0)
    {
      for (int i = 0; i < (bioDy >> BDOF_SUBPU_DIM_LOG2); i++)
      {
        for (int j = 0; j < (bioDx >> BDOF_SUBPU_DIM_LOG2); j++)
        {
          m_bdofSubPuMvOffset[bdofBlockOffset + bioSubPuMvIndex + i * BDOF_SUBPU_STRIDE + j].setZero();
          m_bdofSubPuMvOffse2[bdofBlockOffset + bioSubPuMvIndex + i * BDOF_SUBPU_STRIDE + j].setZero();
        }
      }
    }
    else
    {
      for (int i = 0; i < (bioDy >> BDOF_SUBPU_DIM_LOG2); i++)
      {
        for (int j = 0; j < (bioDx >> BDOF_SUBPU_DIM_LOG2); j++)
        {
          m_bdofSubPuMvOffse2[bdofBlockOffset + bioSubPuMvIndex + i * BDOF_SUBPU_STRIDE + j].setZero();
        }
      }
    }
  }
  else
  {
    for (int i = 0; i < (bioDy >> BDOF_SUBPU_DIM_LOG2); i++)
    {
      for (int j = 0; j < (bioDx >> BDOF_SUBPU_DIM_LOG2); j++)
      {
        m_bdofSubPuMvOffset[bdofBlockOffset + bioSubPuMvIndex + i * BDOF_SUBPU_STRIDE + j].setZero();
      }
    }
  }
}

inline void oobSubMcMask(const int bioDx, const int bioDy, const int width, bool *pSubMcMask[NUM_RPL01], bool *isOOB,
                         bool *isOOBTmp)
{
  if (isOOB[0] || isOOB[1])
  {
    for (int dir = 0; dir < 2; dir++)
    {
      bool *pMcMask = (dir == 0) ? pSubMcMask[0] : pSubMcMask[1];
      for (int y = 0; y < bioDy && !isOOBTmp[dir]; y++)
      {
        for (int x = 0; x < bioDx && !isOOBTmp[dir]; x++)
        {
          isOOBTmp[dir] |= pMcMask[x];
        }
        pMcMask += width;
      }
    }
  }
}

template<bool dmvdBDOFExt>
void InterPrediction::applyBiOptFlow(const CodingUnit &cu, const bool isBdofMvRefine, const int bdofBlockOffset,
                                     const CPelUnitBuf &yuvSrc0, const CPelUnitBuf &yuvSrc1, const int &refIdx0,
                                     const int &refIdx1, PelUnitBuf &yuvDst, const BitDepths &clipBitDepths,
                                     bool *mcMask[NUM_RPL01], bool *mcMaskChroma[NUM_RPL01], bool *isOOB,
                                     const bool scaleMvBDOF, const bool reduceSubPuSizeBDOF, int iter)

{
  PROFILER_SCOPE(1, g_timeProfiler, P_BDOF);
  const int height  = yuvDst.Y().height;
  const int width   = yuvDst.Y().width;
  int       heightG = height + 2 * BDOF_EXTEND_SIZE;
  int       widthG  = width + 2 * BDOF_EXTEND_SIZE;

  Pel *gradX0 = m_gradX0;
  Pel *gradX1 = m_gradX1;
  Pel *gradY0 = m_gradY0;
  Pel *gradY1 = m_gradY1;

  const int stridePredMC = widthG + 2;

  const Pel *srcY0 = m_filteredBlockTmp[2][COMP_Y] + stridePredMC + 1;
  const Pel *srcY1 = m_filteredBlockTmp[3][COMP_Y] + stridePredMC + 1;

  const int src0Stride = stridePredMC;
  const int src1Stride = stridePredMC;

  Pel       *dstY      = yuvDst.Y().buf;
  const int  dstStride = (int)yuvDst.Y().stride;
  const Pel *srcY0Temp = srcY0;
  const Pel *srcY1Temp = srcY1;

  for (int refList = 0; refList < NUM_RPL01; refList++)
  {
    Pel *dstTempPtr = m_filteredBlockTmp[2 + refList][COMP_Y] + stridePredMC + 1;
    Pel *gradY      = (refList == 0) ? m_gradY0 : m_gradY1;
    Pel *gradX      = (refList == 0) ? m_gradX0 : m_gradX1;

    xBioGradFilter(dstTempPtr, stridePredMC, widthG, heightG, widthG, gradX, gradY,
                   clipBitDepths[toChannelType(COMP_Y)]);
  }

  const ClpRng &clpRng   = cu.cs->slice->clpRng(COMP_Y);
  const int     bitDepth = clipBitDepths[toChannelType(COMP_Y)];
  const int     shiftNum = IF_INTERNAL_FRAC_BITS(bitDepth) + 1;
  const int     offset   = (1 << (shiftNum - 1)) + 2 * IF_INTERNAL_OFFS;
  const int     limit    = (1 << 5) - 1;

  int srcBlockOffset      = (stridePredMC + 1) * BDOF_EXTEND_SIZE;
  int bioBlockParamOffset = (widthG + 1);
  int dstBlockOffset      = 0;

  int scaleBDOF = 2;
  if ((isBdofMvRefine && cu.bdmvrRefine && reduceSubPuSizeBDOF) ||
      (!isBdofMvRefine && ((width % 8 == 4) && (width != 4))))
  {
    scaleBDOF = 1;
  }
  if (dmvdBDOFExt && iter == 0 && isBdofMvRefine && !m_subPuMC)
  {
    scaleBDOF = 4;
  }

  int       scaleBDOFLog2                 = (scaleBDOF == 4) ? 2 : scaleBDOF - 1;
  const int bioDx                         = (width < BDOF_SUBPU_DIM * scaleBDOF) ? width : BDOF_SUBPU_DIM * scaleBDOF;
  const int bioDy                         = (height < BDOF_SUBPU_DIM * scaleBDOF) ? height : BDOF_SUBPU_DIM * scaleBDOF;
  const int srcBlockOffsetIncrementY      = (stridePredMC << (BDOF_SUBPU_DIM_LOG2 + scaleBDOFLog2)) - width;
  const int dstBlockOffsetIncrementY      = (dstStride << (BDOF_SUBPU_DIM_LOG2 + scaleBDOFLog2)) - width;
  const int bioBlockParamOffsetIncrementY = (widthG << (BDOF_SUBPU_DIM_LOG2 + scaleBDOFLog2)) - width;

  if (isBdofMvRefine)
  {
    bool simBIOParameter = true;
    if (true)
    {
      g_pelBufOP.calcBIOParameterHighPrecision(srcY0, srcY1, gradX0, gradX1, gradY0, gradY1, widthG, heightG,
                                               src0Stride, src1Stride, widthG, bitDepth, m_piDotProduct[0],
                                               m_piDotProduct[1], m_piDotProduct[2], m_piDotProduct[3],
                                               m_piDotProduct[4], m_dI, m_Gx, m_Gy);
    }
    else
    {
      g_pelBufOP.calcBIOParameter(srcY0, srcY1, gradX0, gradX1, gradY0, gradY1, widthG, heightG, src0Stride, src1Stride,
                                  widthG, bitDepth, m_absGx, m_absGy, m_dIx, m_dIy, m_signGxGy, m_dI);
      simBIOParameter = true;
    }
    m_bdofMvRefined                     = true;
    int       bioSubPuMvIndex           = 0;
    const int bioSubPuMvIndexIncrementY = BDOF_SUBPU_STRIDE - std::max(1, (width >> BDOF_SUBPU_DIM_LOG2));
    const int bioBlockDistTh            = dmvdBDOFExt
                 ? ((m_subPuMC || (iter == 2)) ? ((bioDx * bioDy) << (6 - 4))
                                               : ((bioDx * bioDy) << (6 - 4 + ((cu.slice->m_pps->m_picInitQPMinus26 + 26) >> 5))))
                 : ((bioDx * bioDy) << (5 - 4));
    Pel      *dI                        = m_dI + 2 + 2 * widthG;
    for (int yBlock = 0; yBlock < height; yBlock += bioDy)
    {
      for (int xBlock = 0; xBlock < width; xBlock += bioDx)
      {
        srcY0Temp = srcY0 + srcBlockOffset;
        srcY1Temp = srcY1 + srcBlockOffset;

        int  costSubblockSAD = 0;
        Pel *tmp             = dI + bioBlockParamOffset;
        g_pelBufOP.calAbsSum(tmp, widthG, bioDx, bioDy, &costSubblockSAD);

        if (costSubblockSAD < bioBlockDistTh)
        {
          int   maskOffset    = yBlock * width + xBlock;
          bool *pSubMcMask[2] = { m_mcMask[0] + maskOffset, m_mcMask[1] + maskOffset };
          bool  isOOBTmp[2]   = { false, false };
          oobSubMcMask(bioDx, bioDy, width, pSubMcMask, isOOB, isOOBTmp);
          xResetBDOFMvData<dmvdBDOFExt>(bdofBlockOffset, bioSubPuMvIndex, bioDx, bioDy, iter);
          if (!dmvdBDOFExt || (iter != 0) || !cu.bdmvrRefine || m_subPuMC)
          {
            if (bioDx == 4)
            {
              g_pelBufOP.addAvg4(srcY0Temp, src0Stride, srcY1Temp, src1Stride, dstY + dstBlockOffset, dstStride, bioDx,
                                 bioDy, shiftNum, offset, clpRng, pSubMcMask, width, isOOBTmp);
            }
            else
            {
              g_pelBufOP.addAvg8(srcY0Temp, src0Stride, srcY1Temp, src1Stride, dstY + dstBlockOffset, dstStride, bioDx,
                                 bioDy, shiftNum, offset, clpRng, pSubMcMask, width, isOOBTmp);
            }
          }
          srcBlockOffset += bioDx;
          dstBlockOffset += bioDx;
          bioBlockParamOffset += bioDx;
          bioSubPuMvIndex += (bioDx >> BDOF_SUBPU_DIM_LOG2);
          continue;
        }
        if (!cu.bdmvrRefine)
        {
          xResetBDOFMvData<dmvdBDOFExt>(bdofBlockOffset, bioSubPuMvIndex, bioDx, bioDy, iter);
          int   maskOffset    = yBlock * width + xBlock;
          bool *pSubMcMask[2] = { m_mcMask[0] + maskOffset, m_mcMask[1] + maskOffset };
          bool  isOOBTmp[2]   = { false, false };
          oobSubMcMask(bioDx, bioDy, width, pSubMcMask, isOOB, isOOBTmp);
          subBlockBiOptFlow(dstY + dstBlockOffset, dstStride, srcY0Temp, src0Stride, srcY1Temp, src1Stride,
                            bioBlockParamOffset, widthG, bioDx, bioDy, clpRng, shiftNum, offset, limit, pSubMcMask,
                            width, isOOBTmp);
          srcBlockOffset += bioDx;
          dstBlockOffset += bioDx;
          bioBlockParamOffset += bioDx;
          bioSubPuMvIndex += (bioDx >> BDOF_SUBPU_DIM_LOG2);
          continue;
        }
        int32_t sumS[5];
        for (int i = 0; i < 5; i++)
        {
          sumS[i] = 0;
        }

        g_pelBufOP.calcBIOParamSum4HighPrecision(
          m_piDotProduct[0] + bioBlockParamOffset, m_piDotProduct[1] + bioBlockParamOffset,
          m_piDotProduct[2] + bioBlockParamOffset, m_piDotProduct[3] + bioBlockParamOffset,
          m_piDotProduct[4] + bioBlockParamOffset, bioDx + 4, bioDy + 4, widthG, &sumS[0], &sumS[1], &sumS[2], &sumS[3],
          &sumS[4], m_dI + bioBlockParamOffset, m_Gx + bioBlockParamOffset, m_Gy + bioBlockParamOffset, cu.geoFlag,
          m_subPuMC);

        int regVxVy = (1 << 10);
        regVxVy <<= 1;
        if (bioDx == 4)
        {
          regVxVy >>= 1;
        }
        if (bioDy == 4)
        {
          regVxVy >>= 1;
        }
        if ((bioDx == 16) || (bioDy == 16))
        {
          regVxVy <<= 1;
        }
        sumS[0] += regVxVy;
        sumS[3] += regVxVy;

        int64_t dD = (int64_t)sumS[0] * sumS[3] - (int64_t)sumS[1] * sumS[1];
        int64_t xD = (int64_t)sumS[2] * sumS[3] - (int64_t)sumS[4] * sumS[1];
        int64_t yD = (int64_t)sumS[0] * sumS[4] - (int64_t)sumS[2] * sumS[1];
        xD <<= 4;
        yD <<= 4;
        int tmpxBlock    = 0;
        int tmpyBlock    = 0;
        int divTable[16] = { 0, 7, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 1, 1, 0 };
        int signD        = dD > 0 ? 1 : -1;
        dD               = abs(dD);
        if (dD > 9)
        {
          int log2D = floorLog2Uint64(dD);
          int msbD  = int((dD << 4) >> log2D) & 15;
          int invD  = divTable[msbD] | 8;
          log2D += (msbD != 0);
          int shiftD  = log2D + 3;
          int offsetD = (1 << (shiftD - 1));
          tmpxBlock   = int((signD * xD * invD + offsetD) >> shiftD);
          tmpyBlock   = int((signD * yD * invD + offsetD) >> shiftD);
        }
        tmpxBlock = Clip3(-256, 256, tmpxBlock);
        tmpyBlock = Clip3(-256, 256, tmpyBlock);

        Mv bioMv;
        if (tmpxBlock >= 0)
        {
          bioMv.hor = ((tmpxBlock + 2) >> 2);
        }
        else
        {
          bioMv.hor = (-1) * ((((-1) * tmpxBlock) + 2) >> 2);
        }
        if (tmpyBlock >= 0)
        {
          bioMv.ver = ((tmpyBlock + 2) >> 2);
        }
        else
        {
          bioMv.ver = (-1) * ((((-1) * tmpyBlock) + 2) >> 2);
        }
        xStoreBDOFMvDataExt<dmvdBDOFExt>(bioMv, bdofBlockOffset, bioSubPuMvIndex, bioDx, bioDy, scaleMvBDOF, iter);
        if (bioMv.hor == 0 && bioMv.ver == 0)
        {
          if (!dmvdBDOFExt || iter != 0 || m_subPuMC)
          {

            // by doing this, we do not need to do second LUMA MC
            int   maskOffset    = yBlock * width + xBlock;
            bool *pSubMcMask[2] = { mcMask[0] + maskOffset, mcMask[1] + maskOffset };
            bool  isOOBTmp[2]   = { false, false };
            oobSubMcMask(bioDx, bioDy, width, pSubMcMask, isOOB, isOOBTmp);

            if (!simBIOParameter)
            {
              g_pelBufOP.calcBIOParameter(srcY0, srcY1, gradX0, gradX1, gradY0, gradY1, widthG, heightG, src0Stride,
                                          src1Stride, widthG, bitDepth, m_absGx, m_absGy, m_dIx, m_dIy, m_signGxGy,
                                          nullptr);
              simBIOParameter = true;
            }
            subBlockBiOptFlow(dstY + dstBlockOffset, dstStride, srcY0Temp, src0Stride, srcY1Temp, src1Stride,
                              bioBlockParamOffset, widthG, bioDx, bioDy, clpRng, shiftNum, offset, limit, pSubMcMask,
                              width, isOOBTmp);
          }
        }
        srcBlockOffset += bioDx;
        dstBlockOffset += bioDx;
        bioBlockParamOffset += bioDx;
        bioSubPuMvIndex += (bioDx >> BDOF_SUBPU_DIM_LOG2);
      }
      srcBlockOffset += srcBlockOffsetIncrementY;
      dstBlockOffset += dstBlockOffsetIncrementY;
      bioBlockParamOffset += bioBlockParamOffsetIncrementY;
      bioSubPuMvIndex += bioSubPuMvIndexIncrementY;
      bioSubPuMvIndex += ((bioDy >> BDOF_SUBPU_DIM_LOG2) - 1) * BDOF_SUBPU_STRIDE;
    }
    return;
  }

  g_pelBufOP.calcBIOParameterHighPrecision(srcY0, srcY1, gradX0, gradX1, gradY0, gradY1, widthG, heightG, src0Stride,
                                           src1Stride, widthG, bitDepth, m_piDotProduct[0], m_piDotProduct[1],
                                           m_piDotProduct[2], m_piDotProduct[3], m_piDotProduct[4], m_dI, m_Gx, m_Gy);

  for (int yBlock = 0; yBlock < height; yBlock += bioDy)
  {
    for (int xBlock = 0; xBlock < width; xBlock += bioDx)
    {
      srcY0Temp           = srcY0 + srcBlockOffset;
      srcY1Temp           = srcY1 + srcBlockOffset;
      int   maskOffset    = yBlock * width + xBlock;
      bool *pSubMcMask[2] = { mcMask[0] + maskOffset, mcMask[1] + maskOffset };
      bool  isOOBTmp[2]   = { false, false };
      oobSubMcMask(bioDx, bioDy, width, pSubMcMask, isOOB, isOOBTmp);
      subBlockBiOptFlow(dstY + dstBlockOffset, dstStride, srcY0Temp, src0Stride, srcY1Temp, src1Stride,
                        bioBlockParamOffset, widthG, bioDx, bioDy, clpRng, shiftNum, offset, limit, pSubMcMask, width,
                        isOOBTmp);
      srcBlockOffset += bioDx;
      dstBlockOffset += bioDx;
      bioBlockParamOffset += bioDx;
    }
    srcBlockOffset += srcBlockOffsetIncrementY;
    dstBlockOffset += dstBlockOffsetIncrementY;
    bioBlockParamOffset += bioBlockParamOffsetIncrementY;
  }
}

void InterPrediction::subBlockBiOptFlow(Pel *dstY, const int dstStride, const Pel *src0, const int src0Stride,
                                        const Pel *src1, const int src1Stride, int bioParamOffset,
                                        const int bioParamStride, int width, int height, const ClpRng &clpRng,
                                        const int shiftNum, const int offset, const int limit, bool *mcMask[2],
                                        int mcStride, bool *isOOB)
{
  if (width == 4)
  {
    g_pelBufOP.calcBIOParamSum5NOSIM4(
      m_piDotProduct[0] + bioParamOffset, m_piDotProduct[3] + bioParamOffset, m_piDotProduct[2] + bioParamOffset,
      m_piDotProduct[4] + bioParamOffset, m_piDotProduct[1] + bioParamOffset, bioParamStride, width, height,
      m_sumAbsGxSample32bit, m_sumAbsGySample32bit, m_sumDIXSample32bit, m_sumDIYSample32bit, m_sumSignGyGxSample32bit,
      m_dI + bioParamOffset, m_Gx + bioParamOffset, m_Gy + bioParamOffset);
  }
  else
  {
    g_pelBufOP.calcBIOParamSum5NOSIM8(
      m_piDotProduct[0] + bioParamOffset, m_piDotProduct[3] + bioParamOffset, m_piDotProduct[2] + bioParamOffset,
      m_piDotProduct[4] + bioParamOffset, m_piDotProduct[1] + bioParamOffset, bioParamStride, width, height,
      m_sumAbsGxSample32bit, m_sumAbsGySample32bit, m_sumDIXSample32bit, m_sumDIYSample32bit, m_sumSignGyGxSample32bit,
      m_dI + bioParamOffset, m_Gx + bioParamOffset, m_Gy + bioParamOffset);
  }
  const int bioSubblockSize = width * height;
  g_pelBufOP.calcBIOClippedVxVy(m_sumDIXSample32bit, m_sumAbsGxSample32bit, m_sumDIYSample32bit, m_sumAbsGySample32bit,
                                m_sumSignGyGxSample32bit, limit, bioSubblockSize, m_tmpxSample32bit, m_tmpySample32bit);
  bioParamOffset += ((bioParamStride + 1) << 1);
  g_pelBufOP.addBIOAvgN(src0, src0Stride, src1, src1Stride, dstY, dstStride, m_gradX0 + bioParamOffset,
                        m_gradX1 + bioParamOffset, m_gradY0 + bioParamOffset, m_gradY1 + bioParamOffset, bioParamStride,
                        width, height, m_tmpxSample32bit, m_tmpySample32bit, shiftNum, offset, clpRng, mcMask, mcStride,
                        isOOB);
}

void InterPrediction::xBioGradFilter(Pel *pSrc, ptrdiff_t srcStride, int width, int height, ptrdiff_t gradStride,
                                     Pel *gradX, Pel *gradY, int bitDepth)
{
  g_pelBufOP.bioGradFilter(pSrc, srcStride, width, height, gradStride, gradX, gradY, bitDepth);
}

void InterPrediction::xWeightedAverage(const CodingUnit &cu, const bool isBdofMvRefine, const int bdofBlockOffset,
                                       const CPelUnitBuf &pcYuvSrc0, const CPelUnitBuf &pcYuvSrc1, PelUnitBuf &pcYuvDst,
                                       const bool bioApplied, bool lumaOnly, bool chromaOnly, PelUnitBuf *yuvDstTmp,
                                       bool *mcMask[NUM_RPL01], int mcStride, bool *mcMaskChroma[NUM_RPL01],
                                       int mcCStride, bool *isOOB, const bool scaleMvBDOF,
                                       const bool reduceSubPuSizeBDOF, int iter)
{
  PROFILER_SCOPE(1, g_timeProfiler, P_MOTION_COMP_WEIGHT_AVG);
  CHECK((chromaOnly && lumaOnly), "should not happen");

  const int refIdx0 = cu.refIdx[0];
  const int refIdx1 = cu.refIdx[1];

  const BitDepths &clipBitDepths = cu.slice->m_sps->m_bitDepths;
  const ClpRngs   &clpRngs       = cu.slice->m_clpRngs;

  if (refIdx0 >= 0 && refIdx1 >= 0)
  {
    if (cu.bcwIdx != BCW_DEFAULT && (yuvDstTmp || !cu.ciipFlag))
    {
      CHECK(bioApplied, "Bcw is disallowed with BIO");
      pcYuvDst.addWeightedAvg(pcYuvSrc0, pcYuvSrc1, clpRngs, cu.bcwIdx, chromaOnly, lumaOnly, mcMask, mcStride,
                              mcMaskChroma, mcCStride, isOOB);
      if (yuvDstTmp)
      {
        yuvDstTmp->addAvg(pcYuvSrc0, pcYuvSrc1, clpRngs, chromaOnly, lumaOnly, mcMask, mcStride, mcMaskChroma,
                          mcCStride, isOOB);
      }
      return;
    }
    if (bioApplied)
    {
      if (cu.cs->sps->m_dmvdBDOFExt)
      {
        applyBiOptFlow<true>(cu, isBdofMvRefine, bdofBlockOffset, pcYuvSrc0, pcYuvSrc1, refIdx0, refIdx1, pcYuvDst,
                             clipBitDepths, mcMask, mcMaskChroma, isOOB, scaleMvBDOF, reduceSubPuSizeBDOF, iter);
      }
      else
      {
        applyBiOptFlow<false>(cu, isBdofMvRefine, bdofBlockOffset, pcYuvSrc0, pcYuvSrc1, refIdx0, refIdx1, pcYuvDst,
                              clipBitDepths, mcMask, mcMaskChroma, isOOB, scaleMvBDOF, reduceSubPuSizeBDOF, iter);
      }

      if (yuvDstTmp)
      {
        const int  src0Stride = cu.lwidth() + 2 * BDOF_EXTEND_SIZE + 2;
        const int  src1Stride = cu.lwidth() + 2 * BDOF_EXTEND_SIZE + 2;
        const Pel *pSrcY0     = m_filteredBlockTmp[2][COMP_Y] + (1 + BDOF_EXTEND_SIZE) * (src0Stride + 1);
        const Pel *pSrcY1     = m_filteredBlockTmp[3][COMP_Y] + (1 + BDOF_EXTEND_SIZE) * (src1Stride + 1);
        yuvDstTmp->bufs[0].addAvg(CPelBuf(pSrcY0, src0Stride, cu.lumaSize()),
                                  CPelBuf(pSrcY1, src1Stride, cu.lumaSize()), clpRngs.comp[0], mcMask, mcStride, isOOB);
      }
    }
    if (!bioApplied && (lumaOnly || chromaOnly))
    {
      pcYuvDst.addAvg(pcYuvSrc0, pcYuvSrc1, clpRngs, chromaOnly, lumaOnly, mcMask, mcStride, mcMaskChroma, mcCStride,
                      isOOB);
    }
    else if (!isBdofMvRefine || !bioApplied || yuvDstTmp != NULL)
    {
      if (bioApplied)
      {
        pcYuvDst.addAvg(pcYuvSrc0, pcYuvSrc1, clpRngs, true, false, mcMask, mcStride, mcMaskChroma, mcCStride, isOOB);
      }
      else
      {
        pcYuvDst.addAvg(pcYuvSrc0, pcYuvSrc1, clpRngs, chromaOnly, lumaOnly, mcMask, mcStride, mcMaskChroma, mcCStride,
                        isOOB);
      }
    }
    if (yuvDstTmp)
    {
      if (bioApplied)
      {
        if (isChromaEnabled(yuvDstTmp->chromaFormat))
        {
          yuvDstTmp->bufs[1].copyFrom(pcYuvDst.bufs[1]);
          yuvDstTmp->bufs[2].copyFrom(pcYuvDst.bufs[2]);
        }
      }
      else
      {
        yuvDstTmp->copyFrom(pcYuvDst, !chromaOnly, !lumaOnly);
      }
    }
  }
  else if (refIdx0 >= 0 && refIdx1 < 0)
  {
    if (cu.geoFlag)
    {
      pcYuvDst.copyFrom(pcYuvSrc0, !chromaOnly, !lumaOnly);
    }
    else
    {
      pcYuvDst.copyClip(pcYuvSrc0, clpRngs, !chromaOnly, !lumaOnly);
    }
    if (yuvDstTmp)
    {
      yuvDstTmp->copyFrom(pcYuvDst, !chromaOnly, !lumaOnly);
    }
  }
  else if (refIdx0 < 0 && refIdx1 >= 0)
  {
    if (cu.geoFlag)
    {
      pcYuvDst.copyFrom(pcYuvSrc1, !chromaOnly, !lumaOnly);
    }
    else
    {
      pcYuvDst.copyClip(pcYuvSrc1, clpRngs, !chromaOnly, !lumaOnly);
    }
    if (yuvDstTmp)
    {
      yuvDstTmp->copyFrom(pcYuvDst, !chromaOnly, !lumaOnly);
    }
  }
}

static void convert2HighPrec(CodingUnit &cu, PelUnitBuf &predBuf, bool lumaOnly, bool chromaOnly)
{
  const size_t istart = chromaOnly ? 1 : 0;
  const size_t iend   = lumaOnly ? 1 : predBuf.bufs.size();

  CHECK(lumaOnly && chromaOnly, "should not happen");

  PelUnitBuf &dstBuf = predBuf;

  for (size_t i = istart; i < iend; i++)
  {
    const CompID  compId = CompID(i);
    const ClpRng &clpRng = cu.slice->clpRng(compId);

    const int biShift  = IF_INTERNAL_PREC - cu.slice->clpRng(compId).bd;
    const Pel biOffset = -IF_INTERNAL_OFFS;
    dstBuf.bufs[compId].linearTransform(1, -biShift, biOffset, false, clpRng);
  }
}

void InterPrediction::motionCompensation(CodingUnit &cu, PelUnitBuf &predBuf, RefPicList eRefPicList, bool luma,
                                         bool chroma, PelUnitBuf *predBufWOBIO, bool obmc)
{
  PROFILER_SCOPE(1, g_timeProfiler, P_MOTION_COMPENSATION);
  // Note: there appears to be an interaction with weighted prediction that
  // makes the code follow different paths if chroma is on or off (in the encoder).
  // Therefore for 4:0:0, "chroma" is not changed to false.
  CHECK(predBufWOBIO && cu.ciipFlag, "the case should not happen!");
  m_bdofMvRefined = false;

  if (!cu.cs->pcv->isEncoder)
  {
    if (CU::isIBC(cu))
    {
      CHECK(!luma, "IBC only for Chroma is not allowed.");
      CHECK(cu.obmcFlag, "IBC with OBMC not allowed");
      xIntraBlockCopy(cu, predBuf, COMP_Y);
      if (chroma && isChromaEnabled(cu.chromaFormat))
      {
        xIntraBlockCopy(cu, predBuf, COMP_Cb);
        xIntraBlockCopy(cu, predBuf, COMP_Cr);
      }
      CHECK(cu.licFlag, "IBC with LIC not allowed");
      return;
    }
  }
  // dual tree handling for IBC as the only ref
  if ((!luma || !chroma) && eRefPicList == RPL0)
  {
    xPredInterUni(cu, eRefPicList, predBuf, false, false, luma, chroma);
    if (obmc)
    {
      obmcFilter(cu, predBuf, luma, chroma);
    }
    CHECK(cu.licFlag && isChromaEnabled(cu.chromaFormat), "unexpected");
    return;
  }
  // else, go with regular MC below
  CodingStructure &cs        = *cu.cs;
  const PPS       &pps       = *cs.pps;
  const SliceType  sliceType = cs.slice->m_eSliceType;

  if (luma || chroma)
  {
    if (eRefPicList != RPLX)
    {
      CHECK(predBufWOBIO != nullptr, "the case should not happen!");
      if (!CU::isIBC(cu) && ((sliceType == P_SLICE && pps.m_useWP) || (sliceType == B_SLICE && pps.m_useBiWP)) &&
          !cu.licFlag)
      {
        xPredInterUni(cu, eRefPicList, predBuf, true, false, luma, chroma);
        xWeightedPredictionUni(cu, predBuf, eRefPicList, predBuf, -1, m_maxCompIDToPred, (luma && !chroma),
                               (!luma && chroma));
      }
      else
      {
        xPredInterUni(cu, eRefPicList, predBuf, false, false, luma, chroma);
      }
    }
    else
    {
      if (cu.mergeType != MergeType::DEFAULT_N && cu.mergeType != MergeType::IBC)
      {
        CHECK(predBufWOBIO != nullptr, "the case should not happen!");
        xSubPuMC(cu, predBuf, eRefPicList, luma, chroma);
      }
      else if (xCheckIdenticalMotion(cu))
      {
        xPredInterUni(cu, RPL0, predBuf, false, false, luma, chroma);
        if (m_storeBeforeLIC)
        {
          CHECK(predBufWOBIO, "inlogic");
          {
            UnitArea   localUnitArea(cu.chromaFormat, Area(0, 0, cu.lumaSize().width, cu.lumaSize().height));
            PelUnitBuf predBeforeLICBuffer = m_acPredBeforeLICBuffer[RPL0].getBuf(localUnitArea);
            m_predictionBeforeLIC.copyFrom(predBeforeLICBuffer, luma, chroma);
          }
        }
        std::copy_n(&cu.licScaleAndOffset.scale[0][0], MAX_NUM_COMP, &cu.licScaleAndOffset.scale[1][0]);
        std::copy_n(&cu.licScaleAndOffset.offset[0][0], MAX_NUM_COMP, &cu.licScaleAndOffset.offset[1][0]);
        if (predBufWOBIO)
        {
          predBufWOBIO->copyFrom(predBuf, luma, chroma);
        }
      }
      else
      {
        xPredInterBi(cu, predBuf, luma, chroma, predBufWOBIO);
        if (m_bdofMvRefined)
        {
          xPredInterBiSubPuBDOF(cu, predBuf, luma, chroma);   // do not change the predBufWOBIO
        }
      }
    }
  }
  if (obmc)
  {
    obmcFilter(cu, predBuf, true, isChromaEnabled(cu.chromaFormat));
    CHECK(predBufWOBIO, "Not expected for OBMC");
  }

  if ((luma || chroma) && cu.geoFlag && cu.interDir == 3)
  {
    const bool lumaOnly   = luma && !chroma;
    const bool chromaOnly = !luma && chroma;
    convert2HighPrec(cu, predBuf, lumaOnly, chromaOnly);
  }

  return;
}

int InterPrediction::rightShiftMSB(int numer, int denom) { return numer >> floorLog2(denom); }

void InterPrediction::motionCompensationGeo(CodingUnit &cu, const GeoMergeCtx &geoMrgCtx, IntraPrediction *pcIntraPred,
                                            std::vector<Pel> *reshapeLUT)
{
  const bool     chroma = isChromaEnabled(cu.chromaFormat);
  const UnitArea localUnitArea(cu.cs->area.chromaFormat, Area(0, 0, cu.lwidth(), cu.lheight()));
  PelUnitBuf     predBuf = cu.cs->getPredBuf(cu);
  PelUnitBuf     tmpGeoBuf[2];
  uint8_t        intraMPM[NUM_MOST_PROBABLE_MODES];

  for (int i = 0; i < 2; i++)
  {
    tmpGeoBuf[i] = m_geoPartBuf[i].getBuf(localUnitArea);
    bool isIntra = cu.geoMergeIdx[i] >= GEO_MAX_NUM_UNI_CANDS;
    if (isIntra)
    {
      PU::getGeoIntraMPMs(cu, intraMPM, cu.geoSplitDir, g_geoTmShape[i][g_geoParams[cu.geoSplitDir].angleIdx]);
      cu.intraDir[ChannelType::LUMA] = cu.intraDir[ChannelType::CHROMA] =
        intraMPM[cu.geoMergeIdx[i] - GEO_MAX_NUM_UNI_CANDS];
      pcIntraPred->initIntraPatternChType(cu, cu.Y());
      pcIntraPred->predIntraAng(COMP_Y, tmpGeoBuf[i].Y(), cu, true, false);

      if (chroma)
      {
        pcIntraPred->initIntraPatternChType(cu, cu.Cb());
        pcIntraPred->predIntraAng(COMP_Cb, tmpGeoBuf[i].Cb(), cu, true, false);
        pcIntraPred->initIntraPatternChType(cu, cu.Cr());
        pcIntraPred->predIntraAng(COMP_Cr, tmpGeoBuf[i].Cr(), cu, true, false);
      }
    }
    else
    {
      if (cu.geoMMVDFlag[i])
      {
        geoMrgCtx[0].setGeoMmvdMergeInfo(cu, cu.geoMergeIdx[i], cu.geoMMVDIdx[i]);
      }
      else
      {
        geoMrgCtx[0].setMergeInfo(cu, cu.geoMergeIdx[i]);
      }
      PU::spanMotionInfo(cu);
      // TODO: check 4:0:0 interaction with weighted prediction.
      motionCompensation(cu, tmpGeoBuf[i], RPLX, true, chroma, nullptr, false);
    }
  }

  PU::spanGeoMMVDMotionInfo(cu, geoMrgCtx);

  for (int i = 0; i < 2; i++)
  {
    if (cu.gpmIntraFlag)
    {
      if (cu.geoMergeIdx[i] < GEO_MAX_NUM_UNI_CANDS)
      {
        tmpGeoBuf[i].roundToOutputBitdepth(tmpGeoBuf[i], cu.slice->m_clpRngs);
        obmcFilter(cu, tmpGeoBuf[i], true, chroma);
        if (reshapeLUT)
        {
          tmpGeoBuf[i].Y().rspSignal(*reshapeLUT);
        }
      }
    }
    if (g_mctsDecCheckEnabled && !MCTSHelper::checkMvBufferForMCTSConstraint(cu, true))
    {
      printf("DECODER_GEO_PU: cu motion vector across tile boundaries (%d,%d,%d,%d)\n", cu.lx(), cu.ly(), cu.lwidth(),
             cu.lheight());
    }
  }

  if (cu.gpmIntraFlag)
  {
    weightedGeoRounded(cu, chroma ? ChannelType::NUM : ChannelType::LUMA, predBuf, tmpGeoBuf[0], tmpGeoBuf[1]);
  }
  else
  {
    weightedGeo(cu, (chroma ? ChannelType::NUM : ChannelType::LUMA), predBuf, tmpGeoBuf[0], tmpGeoBuf[1]);
    obmcFilter(cu, predBuf, true, chroma);
  }
}

void InterPrediction::weightedGeo(CodingUnit &cu, const ChannelType channel, PelUnitBuf &predDst, PelUnitBuf &predSrc0,
                                  PelUnitBuf &predSrc1)
{
  if (channel != ChannelType::CHROMA)
  {
    m_if->weightedGeoBlk(cu, cu.lumaSize().width, cu.lumaSize().height, COMP_Y, cu.geoSplitDir, cu.geoBldIdx, predDst,
                         predSrc0, predSrc1);
  }

  if (channel != ChannelType::LUMA)
  {
    m_if->weightedGeoBlk(cu, cu.chromaSize().width, cu.chromaSize().height, COMP_Cb, cu.geoSplitDir, cu.geoBldIdx,
                         predDst, predSrc0, predSrc1);
    m_if->weightedGeoBlk(cu, cu.chromaSize().width, cu.chromaSize().height, COMP_Cr, cu.geoSplitDir, cu.geoBldIdx,
                         predDst, predSrc0, predSrc1);
  }
}

void InterPrediction::weightedGeoRounded(CodingUnit &cu, const ChannelType channel, PelUnitBuf &predDst,
                                         PelUnitBuf &predSrc0, PelUnitBuf &predSrc1)
{
  if (channel != ChannelType::CHROMA)
  {
    m_if->weightedGeoBlkRounded(cu, cu.lumaSize().width, cu.lumaSize().height, COMP_Y, cu.geoSplitDir, cu.geoBldIdx,
                                predDst, predSrc0, predSrc1);
  }

  if (channel != ChannelType::LUMA)
  {
    m_if->weightedGeoBlkRounded(cu, cu.chromaSize().width, cu.chromaSize().height, COMP_Cb, cu.geoSplitDir,
                                cu.geoBldIdx, predDst, predSrc0, predSrc1);
    m_if->weightedGeoBlkRounded(cu, cu.chromaSize().width, cu.chromaSize().height, COMP_Cr, cu.geoSplitDir,
                                cu.geoBldIdx, predDst, predSrc0, predSrc1);
  }
}

inline int32_t div_for_maxq7(int64_t N, int64_t D)
{
  int32_t sign, q;
  sign = 0;
  if (N < 0)
  {
    sign = 1;
    N    = -N;
  }

  q = 0;
  D = (D << 3);
  if (N >= D)
  {
    N -= D;
    q++;
  }
  q = (q << 1);

  D = (D >> 1);
  if (N >= D)
  {
    N -= D;
    q++;
  }
  q = (q << 1);

  if (N >= (D >> 1))
  {
    q++;
  }
  if (sign)
  {
    return (-q);
  }
  return (q);
}

void xSubPelErrorSrfc(uint64_t *sadBuffer, int32_t *deltaMv)
{
  int64_t numerator, denominator;
  int32_t mvDeltaSubPel;
  int32_t mvSubPelLvl = 4; /*1: half pel, 2: Qpel, 3:1/8, 4: 1/16*/
  /*horizontal*/
  numerator           = (int64_t)((sadBuffer[1] - sadBuffer[3]) << mvSubPelLvl);
  denominator         = (int64_t)((sadBuffer[1] + sadBuffer[3] - (sadBuffer[0] << 1)));

  if (denominator > 0)
  {
    if ((sadBuffer[1] != sadBuffer[0]) && (sadBuffer[3] != sadBuffer[0]))
    {
      mvDeltaSubPel = div_for_maxq7(numerator, denominator);
      deltaMv[0]    = (mvDeltaSubPel);
    }
    else
    {
      if (sadBuffer[1] == sadBuffer[0])
      {
        deltaMv[0] = -8;   // half pel
      }
      else
      {
        deltaMv[0] = 8;   // half pel
      }
    }
  }
  else
  {
    if (sadBuffer[1] < sadBuffer[3])
    {
      deltaMv[0] = -8;
    }
    else if (sadBuffer[1] == sadBuffer[3])
    {
      deltaMv[0] = 0;
    }
    else
    {
      deltaMv[0] = 8;
    }
  }

  /*vertical*/
  numerator   = (int64_t)((sadBuffer[2] - sadBuffer[4]) << mvSubPelLvl);
  denominator = (int64_t)((sadBuffer[2] + sadBuffer[4] - (sadBuffer[0] << 1)));
  if (denominator > 0)
  {
    if ((sadBuffer[2] != sadBuffer[0]) && (sadBuffer[4] != sadBuffer[0]))
    {
      mvDeltaSubPel = div_for_maxq7(numerator, denominator);
      deltaMv[1]    = (mvDeltaSubPel);
    }
    else
    {
      if (sadBuffer[2] == sadBuffer[0])
      {
        deltaMv[1] = -8;   // half pel
      }
      else
      {
        deltaMv[1] = 8;   // half pel
      }
    }
  }
  else
  {
    if (sadBuffer[2] < sadBuffer[4])
    {
      deltaMv[1] = -8;
    }
    else if (sadBuffer[2] == sadBuffer[4])
    {
      deltaMv[1] = 0;
    }
    else
    {
      deltaMv[1] = 8;
    }
  }
  return;
}

void InterPrediction::xBmAffineInit(const CodingUnit &pu)
{
  m_bmChFmt  = pu.chromaFormat;
  m_bmClpRng = pu.cs->slice->clpRng(COMP_Y);

  m_bmRefPic[RPL0] = pu.slice->getRefPic(RPL0, pu.refIdx[RPL0])->m_unscaledPic;
  m_bmRefPic[RPL1] = pu.slice->getRefPic(RPL1, pu.refIdx[RPL1])->m_unscaledPic;
  m_bmRefBuf[RPL0] = m_bmRefPic[RPL0]->getRecoBuf(COMP_Y, false);
  m_bmRefBuf[RPL1] = m_bmRefPic[RPL1]->getRecoBuf(COMP_Y, false);

  int      width  = pu.Y().width;
  int      height = pu.Y().height;
  Position puPos  = pu.lumaPos();
  int      shift  = MAX_CU_DEPTH;
  int      deltaMvHorX[2], deltaMvHorY[2], deltaMvVerX[2], deltaMvVerY[2];
  int      mvScaleHor[2];
  int      mvScaleVer[2];
  int      blockWidth[2] = { AFFINE_DMVR_MIN_SUBBLK_SIZE, AFFINE_DMVR_MIN_SUBBLK_SIZE },
      blockHeight[2]     = { AFFINE_DMVR_MIN_SUBBLK_SIZE, AFFINE_DMVR_MIN_SUBBLK_SIZE };
  int minSbW             = width > 16 ? 8 : 4;
  int minSbH             = height > 16 ? 8 : 4;
  blockWidth[0] = blockWidth[1] = minSbW;
  blockHeight[0] = blockHeight[1] = minSbH;

  for (int i = 0; i < 2; i++)
  {
    deltaMvHorX[i] = (pu.mvAffi[i][1] - pu.mvAffi[i][0]).getHor() << (shift - floorLog2(width));
    deltaMvHorY[i] = (pu.mvAffi[i][1] - pu.mvAffi[i][0]).getVer() << (shift - floorLog2(width));
    if (pu.affineType == AffineModel::_6_PARAMS)
    {
      deltaMvVerX[i] = (pu.mvAffi[i][2] - pu.mvAffi[i][0]).getHor() << (shift - floorLog2(height));
      deltaMvVerY[i] = (pu.mvAffi[i][2] - pu.mvAffi[i][0]).getVer() << (shift - floorLog2(height));
    }
    else
    {
      deltaMvVerX[i] = -deltaMvHorY[i];
      deltaMvVerY[i] = deltaMvHorX[i];
    }
    mvScaleHor[i] = pu.mvAffi[i][0].getHor() << shift;
    mvScaleVer[i] = pu.mvAffi[i][0].getVer() << shift;

    blockWidth[i]  = deriveAffineSubBlkSize(width, blockWidth[i], deltaMvHorX[i], deltaMvHorY[i], shift);
    blockHeight[i] = deriveAffineSubBlkSize(height, blockHeight[i], deltaMvVerX[i], deltaMvVerY[i], shift);
  }

  const int dx = std::min(blockWidth[0], blockWidth[1]);
  const int dy = std::min(blockHeight[0], blockHeight[1]);

  m_bmSubBlkW             = dx;
  m_bmSubBlkH             = dy;
  m_bmInterpolationTmpBuf = PelBuf(m_filteredBlockTmp[0][COMP_Y], m_bmSubBlkW, m_bmSubBlkH);
  m_bmFilterSize          = 2;
  m_bmInterpolationHOfst  = ((m_bmFilterSize >> 1) - 1) * (int)m_bmRefBuf[RPL0].stride;
  m_bmInterpolationVOfst  = ((m_bmFilterSize >> 1) - 1) * (int)m_bmInterpolationTmpBuf.stride;

  xBmInitAffineSubBlocks(puPos, width, height, dx, dy, mvScaleHor, mvScaleVer, deltaMvHorX, deltaMvHorY, deltaMvVerX,
                         deltaMvVerY);

  xInitBilateralMatching(pu.lwidth(), pu.lheight(), m_bmClpRng.bd, false, 2);
}

void InterPrediction::xBmInitAffineSubBlocks(const Position puPos, const int width, const int height, const int dx,
                                             const int dy, int mvScaleHor[2], int mvScaleVer[2], int deltaMvHorX[2],
                                             int deltaMvHorY[2], int deltaMvVerX[2], int deltaMvVerY[2])
{
  const int stepY  = dy;
  const int stepX  = dx;
  const int halfBW = dx >> 1;
  const int halfBH = dy >> 1;

  int mvScaleTmpHor[2];
  int mvScaleTmpVer[2];

  BMSubBlkInfo currSubBlk;
  m_bmSubBlkList.clear();
  for (int y = puPos.y, yStart = halfBH; y < (puPos.y + height); y = y + stepY, yStart = yStart + stepY)
  {
    for (int x = puPos.x, xStart = halfBW; x < (puPos.x + width); x = x + stepX, xStart = xStart + stepX)
    {
      currSubBlk.Area::operator=(Area(x, y, dx, dy));
      currSubBlk.m_cXInPU    = xStart;
      currSubBlk.m_cYInPU    = yStart;
      currSubBlk.m_predReady = false;
      // derive subblock MV
      for (int list = 0; list < 2; list++)
      {
        mvScaleTmpHor[list] = mvScaleHor[list] + deltaMvHorX[list] * xStart + deltaMvVerX[list] * yStart;
        mvScaleTmpVer[list] = mvScaleVer[list] + deltaMvHorY[list] * xStart + deltaMvVerY[list] * yStart;

        roundAffineMv(mvScaleTmpHor[list], mvScaleTmpVer[list], MAX_CU_DEPTH);
        currSubBlk.m_mv[list] = Mv(mvScaleTmpHor[list], mvScaleTmpVer[list]);
      }
      m_bmSubBlkList.push_back(currSubBlk);
    }
  }
}

void InterPrediction::xBmAffineIntSearch(
  const CodingUnit &pu, Mv (&mvOffset)[2], Distortion &minCost,
  Distortion totalCost[(AFFINE_DMVR_INT_SRCH_RANGE << 1) + 1][(AFFINE_DMVR_INT_SRCH_RANGE << 1) + 1])
{

  int iWidthExt          = m_bmSubBlkW + (AFFINE_DMVR_INT_SRCH_RANGE << 1);
  int iHeightExt         = m_bmSubBlkH + (AFFINE_DMVR_INT_SRCH_RANGE << 1);
  int dstStride          = iWidthExt;
  int searchCenterBufPos = AFFINE_DMVR_INT_SRCH_RANGE * dstStride + AFFINE_DMVR_INT_SRCH_RANGE;

  Mv mvInitial[2];
  Mv mvPreInterOffset = Mv((AFFINE_DMVR_INT_SRCH_RANGE << MV_FRACTIONAL_BITS_INTERNAL),
                           (AFFINE_DMVR_INT_SRCH_RANGE << MV_FRACTIONAL_BITS_INTERNAL));

  int       useHadmard = 2;
  DistParam cDistParam;
  cDistParam.applyWeight = false;
  cDistParam.useMR       = false;
  int bmCostShift        = 0;
  int bitDepth           = pu.slice->clpRng(COMP_Y).bd;
#if FULL_NBIT
  if (useHadmard)
  {
    bmCostShift = 1;   // magic shift, benefit for early terminate
  }
  else
  {
    bmCostShift = bitDepth > 8 ? bitDepth - 8 : 0;
  }
#else
  bmCostShift = 0;
#endif

  Pel   *pelBuffer[2] = { m_filteredBlock[3][RPL0][0] + searchCenterBufPos,
                          m_filteredBlock[3][RPL1][0] + searchCenterBufPos };
  PelBuf predBuf[2]   = { PelBuf(m_filteredBlock[3][RPL0][0], dstStride, iWidthExt, iHeightExt),
                          PelBuf(m_filteredBlock[3][RPL1][0], dstStride, iWidthExt, iHeightExt) };

  for (int i = -AFFINE_DMVR_INT_SRCH_RANGE; i <= AFFINE_DMVR_INT_SRCH_RANGE; i++)
  {
    for (int j = -AFFINE_DMVR_INT_SRCH_RANGE; j <= AFFINE_DMVR_INT_SRCH_RANGE; j++)
    {
      totalCost[AFFINE_DMVR_INT_SRCH_RANGE + i][AFFINE_DMVR_INT_SRCH_RANGE + j] = (abs(i) + abs(j)) << 4;
    }
  }

  CodingUnit subPu = pu;
  Distortion costArray[2 * AFFINE_DMVR_INT_SRCH_RANGE + 1][2 * AFFINE_DMVR_INT_SRCH_RANGE + 1];

  for (std::vector<BMSubBlkInfo>::iterator it = m_bmSubBlkList.begin(); it != m_bmSubBlkList.end(); ++it)
  {
    subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, *it));
    for (int list = 0; list < 2; list++)
    {
      mvInitial[list] = it->m_mv[list];

      xBDMVRFillBlkPredPelBuffer(subPu, *m_bmRefPic[list], mvInitial[list] - mvPreInterOffset, predBuf[list],
                                 pu.cs->slice->clpRng(COMP_Y));   // bi-linear interpolation
    }
    Distortion currSubBlkCost;
    Distortion bestSubBlkCost = std::numeric_limits<Distortion>::max();
    Mv         bestSubBlkDeltaMv;
    int        ibest = 0, jbest = 0;
    for (int i = -AFFINE_DMVR_INT_SRCH_RANGE; i <= AFFINE_DMVR_INT_SRCH_RANGE; i++)
    {
      for (int j = -AFFINE_DMVR_INT_SRCH_RANGE; j <= AFFINE_DMVR_INT_SRCH_RANGE; j++)
      {
        int    ofst       = i * dstStride + j;
        PelBuf currBuf[2] = { PelBuf(pelBuffer[RPL0] + ofst, dstStride, m_bmSubBlkW, m_bmSubBlkH),
                              PelBuf(pelBuffer[RPL1] - ofst, dstStride, m_bmSubBlkW, m_bmSubBlkH) };

        m_pcRdCost->setDistParam(cDistParam, currBuf[0], currBuf[1], bitDepth, COMP_Y, useHadmard);
        currSubBlkCost = cDistParam.distFunc(cDistParam) >> bmCostShift;

        totalCost[AFFINE_DMVR_INT_SRCH_RANGE + i][AFFINE_DMVR_INT_SRCH_RANGE + j] += currSubBlkCost;
        if (i == 0 && j == 0)
        {
          currSubBlkCost -= (currSubBlkCost >> 2);
        }
        costArray[AFFINE_DMVR_INT_SRCH_RANGE + i][AFFINE_DMVR_INT_SRCH_RANGE + j] = currSubBlkCost;
        if (currSubBlkCost < bestSubBlkCost)
        {
          bestSubBlkCost = currSubBlkCost;
          ibest          = i;
          jbest          = j;
        }
      }
    }
    bestSubBlkDeltaMv.set(jbest << MV_FRACTIONAL_BITS_INTERNAL, ibest << MV_FRACTIONAL_BITS_INTERNAL);
    if (abs(ibest) != AFFINE_DMVR_INT_SRCH_RANGE && abs(jbest) != AFFINE_DMVR_INT_SRCH_RANGE)
    {
      uint64_t sadbuffer[5];
      int32_t  tempDeltaMv[2] = { 0, 0 };
      ibest += AFFINE_DMVR_INT_SRCH_RANGE;
      jbest += AFFINE_DMVR_INT_SRCH_RANGE;
      sadbuffer[0] = costArray[ibest][jbest];
      sadbuffer[1] = costArray[ibest][jbest - 1];
      sadbuffer[2] = costArray[ibest - 1][jbest];
      sadbuffer[3] = costArray[ibest][jbest + 1];
      sadbuffer[4] = costArray[ibest + 1][jbest];
      xSubPelErrorSrfc(sadbuffer, tempDeltaMv);
      bestSubBlkDeltaMv += Mv(tempDeltaMv[0], tempDeltaMv[1]);
    }
    it->m_mvRefine[0] = mvInitial[0] + bestSubBlkDeltaMv;
    it->m_mvRefine[1] = mvInitial[1] - bestSubBlkDeltaMv;
  }

  int        ibest = 0, jbest = 0;
  Distortion tmpCost;
  for (int i = -AFFINE_DMVR_INT_SRCH_RANGE; i <= AFFINE_DMVR_INT_SRCH_RANGE; i++)
  {
    for (int j = -AFFINE_DMVR_INT_SRCH_RANGE; j <= AFFINE_DMVR_INT_SRCH_RANGE; j++)
    {
      tmpCost = totalCost[AFFINE_DMVR_INT_SRCH_RANGE + i][AFFINE_DMVR_INT_SRCH_RANGE + j];
      if (tmpCost < minCost)
      {
        minCost = tmpCost;
        ibest   = i;
        jbest   = j;
      }
    }
  }

  minCost -= ((abs(ibest) + abs(jbest)) << 4);
  mvOffset[0].set(jbest << MV_FRACTIONAL_BITS_INTERNAL, ibest << MV_FRACTIONAL_BITS_INTERNAL);
  mvOffset[1] = Mv(0, 0) - mvOffset[0];
}

void InterPrediction::xBmAffineHPelSearch(const CodingUnit &pu, Mv (&curBestMv)[2], Distortion &minCost,
                                          Distortion localCostArray[9])
{
  static const Mv cSearchOffset[8] = { Mv(-1, 1), Mv(0, 1),  Mv(1, 1),   Mv(1, 0),
                                       Mv(1, -1), Mv(0, -1), Mv(-1, -1), Mv(-1, 0) };
  int             nDirectStart     = 0;
  int             nDirectEnd       = 7;
  const int       nDirectRounding  = 8;
  const int       nDirectMask      = 0x07;
  if (minCost == std::numeric_limits<Distortion>::max())
  {
    Distortion tmCost =
      getDecoderSideDerivedMvCost(Mv(0, 0), curBestMv[0], AFFINE_DMVR_SEARCH_RANGE + 1, DECODER_SIDE_MV_WEIGHT);
    minCost = xGetBilateralMatchingErrorAffineForMvOffset(pu, curBestMv);
    if (minCost < tmCost)
    {
      return;
    }

    minCost += tmCost;
  }
  else
  {
    minCost +=
      getDecoderSideDerivedMvCost(Mv(0, 0), curBestMv[0], AFFINE_DMVR_SEARCH_RANGE + 1, DECODER_SIDE_MV_WEIGHT);
  }

  int maxSearchRounds = 2;
  int searchStepShift = MV_FRACTIONAL_BITS_INTERNAL - 1;

  for (uint32_t uiRound = 0; uiRound < maxSearchRounds; uiRound++)
  {
    int nBestDirect    = -1;
    Mv  mvCurCenter[2] = { curBestMv[0], curBestMv[1] };

    for (int nIdx = nDirectStart; nIdx <= nDirectEnd; nIdx++)
    {
      int nDirect = (nIdx + nDirectRounding) & nDirectMask;

      Mv mvOffset(cSearchOffset[nDirect].getHor() << searchStepShift,
                  cSearchOffset[nDirect].getVer() << searchStepShift);

      if (uiRound > 0)
      {
        if ((nDirect % 2) == 0)
        {
          continue;
        }
      }
      Mv         mvCand[2] = { mvCurCenter[0] + mvOffset, mvCurCenter[1] - mvOffset };
      Distortion tmCost =
        getDecoderSideDerivedMvCost(Mv(0, 0), mvCand[0], AFFINE_DMVR_SEARCH_RANGE + 1, DECODER_SIDE_MV_WEIGHT);
      if (tmCost > minCost)
      {
        localCostArray[nDirect] = 2 * tmCost;
        continue;
      }
      tmCost += xGetBilateralMatchingErrorAffineForMvOffset(pu, mvCand);
      localCostArray[nDirect] = tmCost;

      if (tmCost < minCost)
      {
        if (!uiRound)
        {
          nBestDirect = nDirect;
        }
        minCost      = tmCost;
        curBestMv[0] = mvCand[0];
        curBestMv[1] = mvCand[1];
      }
    }
    if (nBestDirect == -1)
    {
      break;
    }

    int nStep    = 2 - (nBestDirect & 0x01);
    nDirectStart = nBestDirect - nStep;
    nDirectEnd   = nBestDirect + nStep;

    if ((uiRound + 1) < maxSearchRounds)
    {
      xBDMVRUpdateSquareSearchCostLog(localCostArray, nBestDirect);
    }
  }
  minCost -= getDecoderSideDerivedMvCost(Mv(0, 0), curBestMv[0], AFFINE_DMVR_SEARCH_RANGE + 1, DECODER_SIDE_MV_WEIGHT);
}

void InterPrediction::xInitBilateralMatching(const int width, const int height, const int bitDepth, const bool useMR,
                                             const int useHadmard)
{
  Pel           *pelBuffer[2] = { m_filteredBlock[3][RPL0][0] + BDMVR_CENTER_POSITION,
                                  m_filteredBlock[3][RPL1][0] + BDMVR_CENTER_POSITION };
  const SizeType stride       = BDMVR_BUF_STRIDE;

  m_bmPredBuf[RPL0] = PelBuf(pelBuffer[RPL0], stride, width, height);
  m_bmPredBuf[RPL1] = PelBuf(pelBuffer[RPL1], stride, width, height);

  m_bmDistParam.applyWeight = false;
  m_bmDistParam.useMR       = useMR;

  m_pcRdCost->setDistParam(m_bmDistParam, m_bmPredBuf[RPL0], m_bmPredBuf[RPL1], bitDepth, COMP_Y, useHadmard);

#if FULL_NBIT
  if (useHadmard)
  {
    m_bmCostShift = 1;   // magic shift, benefit for early terminate
  }
  else
  {
    m_bmCostShift = bitDepth > 8 ? bitDepth - 8 : 0;
  }
#else
  m_bmCostShift = 0;
#endif
}

Distortion InterPrediction::xGetBilateralMatchingErrorAffineForMvOffset(const CodingUnit &pu, Mv (&mvOffset)[2])
{
  ptrdiff_t refStride = m_bmRefBuf[0].stride;
  CHECK(refStride != m_bmRefBuf[1].stride, "refStride != m_bmRefBuf[1].stride");
  ptrdiff_t dstStride = m_bmPredBuf[0].stride;
  CHECK(dstStride != m_bmPredBuf[1].stride, "dstStride != m_bmPredBuf[1].stride");

  Pel *dstTab[2] = { m_bmPredBuf[0].buf, m_bmPredBuf[1].buf };

  const int initPosX = m_bmSubBlkList.begin()->x;
  const int initPosY = m_bmSubBlkList.begin()->y;

  int iMvScaleTmpHor, iMvScaleTmpVer;

  const auto nFilterIdx = InterpolationFilter::Filter::DMVR;
  Distortion cost       = 0;
  for (std::vector<BMSubBlkInfo>::iterator it = m_bmSubBlkList.begin(); it != m_bmSubBlkList.end(); ++it)
  {
    const ptrdiff_t offset = it->x - initPosX + (it->y - initPosY) * dstStride;
    for (int i = 0; i < 2; i++)
    {

      Mv mv = it->m_mv[i] + mvOffset[i];
      clipMv(mv, pu.lumaPos(), pu.lumaSize(), *pu.cs->sps, *pu.cs->pps);
      iMvScaleTmpHor = mv.getHor();
      iMvScaleTmpVer = mv.getVer();

      int xFrac, yFrac, xInt, yInt;

      xInt  = iMvScaleTmpHor >> 4;
      xFrac = iMvScaleTmpHor & 15;
      yInt  = iMvScaleTmpVer >> 4;
      yFrac = iMvScaleTmpVer & 15;

      const Pel *ref = m_bmRefBuf[i].buf + xInt + it->x + (yInt + it->y) * refStride;
      Pel       *dst = &dstTab[i][offset];

      if (yFrac == 0)
      {
        m_if->filterHor(COMP_Y, ref, refStride, dst, dstStride, m_bmSubBlkW, m_bmSubBlkH, xFrac, false, m_bmClpRng,
                        nFilterIdx);
      }
      else if (xFrac == 0)
      {
        m_if->filterVer(COMP_Y, ref, refStride, dst, dstStride, m_bmSubBlkW, m_bmSubBlkH, yFrac, true, false,
                        m_bmClpRng, nFilterIdx);
      }
      else
      {
        m_if->filterHor(COMP_Y, ref - m_bmInterpolationHOfst, refStride, m_bmInterpolationTmpBuf.buf,
                        m_bmInterpolationTmpBuf.stride, m_bmSubBlkW, m_bmSubBlkH + m_bmFilterSize - 1, xFrac, false,
                        m_bmClpRng, nFilterIdx);
        JVET_J0090_SET_CACHE_ENABLE(false);
        m_if->filterVer(COMP_Y, m_bmInterpolationTmpBuf.buf + m_bmInterpolationVOfst, m_bmInterpolationTmpBuf.stride,
                        dst, dstStride, m_bmSubBlkW, m_bmSubBlkH, yFrac, false, false, m_bmClpRng, nFilterIdx);
        JVET_J0090_SET_CACHE_ENABLE(true);
      }
    }
  }
  cost = (m_bmDistParam.distFunc(m_bmDistParam) >> m_bmCostShift);
  return cost;
}

Distortion InterPrediction::xGetBilateralMatchingErrorAffine(const CodingUnit &pu, Mv (&mvAffi)[2][3])
{
  const int width  = pu.Y().width;
  const int height = pu.Y().height;
  int       deltaMvHorX[2], deltaMvHorY[2], deltaMvVerX[2], deltaMvVerY[2];

  for (int i = 0; i < 2; i++)
  {
    deltaMvHorX[i] = (mvAffi[i][1] - mvAffi[i][0]).getHor() << (MAX_CU_DEPTH - floorLog2(width));
    deltaMvHorY[i] = (mvAffi[i][1] - mvAffi[i][0]).getVer() << (MAX_CU_DEPTH - floorLog2(width));
    if (pu.affineType == AffineModel::_6_PARAMS)
    {
      deltaMvVerX[i] = (mvAffi[i][2] - mvAffi[i][0]).getHor() << (MAX_CU_DEPTH - floorLog2(height));
      deltaMvVerY[i] = (mvAffi[i][2] - mvAffi[i][0]).getVer() << (MAX_CU_DEPTH - floorLog2(height));
    }
    else
    {
      deltaMvVerX[i] = -deltaMvHorY[i];
      deltaMvVerY[i] = deltaMvHorX[i];
    }
  }

  int iMvScaleTmpHor, iMvScaleTmpVer;
  int iMvScaleTmpHor0[2] = { mvAffi[0][0].getHor() << MAX_CU_DEPTH, mvAffi[1][0].getHor() << MAX_CU_DEPTH };
  int iMvScaleTmpVer0[2] = { mvAffi[0][0].getVer() << MAX_CU_DEPTH, mvAffi[1][0].getVer() << MAX_CU_DEPTH };

  ptrdiff_t refStride = m_bmRefBuf[0].stride;
  CHECK(refStride != m_bmRefBuf[1].stride, "refStride != m_bmRefBuf[1].stride");
  ptrdiff_t dstStride = m_bmPredBuf[0].stride;
  CHECK(dstStride != m_bmPredBuf[1].stride, "dstStride != m_bmPredBuf[1].stride");

  Pel *dstTab[2] = { m_bmPredBuf[0].buf, m_bmPredBuf[1].buf };

  const int initPosX = m_bmSubBlkList.begin()->x;
  const int initPosY = m_bmSubBlkList.begin()->y;

  Mv         mv[2];
  Distortion cost       = 0;
  const auto nFilterIdx = InterpolationFilter::Filter::DMVR;
  for (std::vector<BMSubBlkInfo>::iterator it = m_bmSubBlkList.begin(); it != m_bmSubBlkList.end(); ++it)
  {
    for (int i = 0; i < 2; i++)
    {
      iMvScaleTmpHor = iMvScaleTmpHor0[i] + deltaMvHorX[i] * it->m_cXInPU + deltaMvVerX[i] * it->m_cYInPU;
      iMvScaleTmpVer = iMvScaleTmpVer0[i] + deltaMvHorY[i] * it->m_cXInPU + deltaMvVerY[i] * it->m_cYInPU;
      roundAffineMv(iMvScaleTmpHor, iMvScaleTmpVer, MAX_CU_DEPTH);
      mv[i] = Mv(iMvScaleTmpHor, iMvScaleTmpVer);
      clipMv(mv[i], pu.lumaPos(), pu.lumaSize(), *pu.cs->sps, *pu.cs->pps);
    }
    const ptrdiff_t offset = it->x - initPosX + (it->y - initPosY) * dstStride;
    for (int i = 0; i < 2; i++)
    {
      if (it->m_mv[i] == mv[i] && it->m_predReady)
      {
        continue;
      }
      it->m_mv[i]    = mv[i];
      iMvScaleTmpHor = mv[i].getHor();
      iMvScaleTmpVer = mv[i].getVer();

      int xFrac, yFrac, xInt, yInt;

      xInt  = iMvScaleTmpHor >> 4;
      xFrac = iMvScaleTmpHor & 15;
      yInt  = iMvScaleTmpVer >> 4;
      yFrac = iMvScaleTmpVer & 15;

      const Pel *ref = m_bmRefBuf[i].buf + xInt + it->x + (yInt + it->y) * refStride;

      Pel *dst = dstTab[i] + offset;
      if (yFrac == 0)
      {
        m_if->filterHor(COMP_Y, ref, refStride, dst, dstStride, m_bmSubBlkW, m_bmSubBlkH, xFrac, false, m_bmClpRng,
                        nFilterIdx);
      }
      else if (xFrac == 0)
      {
        m_if->filterVer(COMP_Y, ref, refStride, dst, dstStride, m_bmSubBlkW, m_bmSubBlkH, yFrac, true, false,
                        m_bmClpRng, nFilterIdx);
      }
      else
      {
        m_if->filterHor(COMP_Y, ref - m_bmInterpolationHOfst, refStride, m_bmInterpolationTmpBuf.buf,
                        m_bmInterpolationTmpBuf.stride, m_bmSubBlkW, m_bmSubBlkH + m_bmFilterSize - 1, xFrac, false,
                        m_bmClpRng, nFilterIdx);
        JVET_J0090_SET_CACHE_ENABLE(false);
        m_if->filterVer(COMP_Y, m_bmInterpolationTmpBuf.buf + m_bmInterpolationVOfst, m_bmInterpolationTmpBuf.stride,
                        dst, dstStride, m_bmSubBlkW, m_bmSubBlkH, yFrac, false, false, m_bmClpRng, nFilterIdx);
        JVET_J0090_SET_CACHE_ENABLE(true);
      }
    }
    it->m_predReady = true;
  }
  cost = (m_bmDistParam.distFunc(m_bmDistParam) >> m_bmCostShift);
  return cost;
}

bool InterPrediction::xBmAffineRegression(CodingUnit &pu, Distortion &minCost)
{
  std::vector<RMVFInfo> mvInfoVec[2];
  for (std::vector<BMSubBlkInfo>::iterator it = m_bmSubBlkList.begin(); it != m_bmSubBlkList.end(); ++it)
  {
    Position mvPos = Position(it->m_cXInPU, it->m_cYInPU);
    mvInfoVec[0].push_back(RMVFInfo(it->m_mvRefine[0], mvPos, -1));
    mvInfoVec[1].push_back(RMVFInfo(it->m_mvRefine[1], mvPos, -1));
    it->m_predReady = false;
  }

  Mv mvAffieRMVF[2][3];
  for (int list = 0; list < 2; list++)
  {
    PU::deriveAffineCandFromMvField(Position(0, 0), pu.lwidth(), pu.lheight(), mvInfoVec[list], mvAffieRMVF[list]);
  }

  auto savedAffineType = pu.affineType;
  pu.affineType        = AffineModel::_6_PARAMS;
  Distortion tmCost    = xGetBilateralMatchingErrorAffine(pu, mvAffieRMVF);
  if (tmCost < minCost)
  {
    for (int cpmvIdx = 0; cpmvIdx < 3; cpmvIdx++)
    {
      pu.mvAffi[0][cpmvIdx] = mvAffieRMVF[0][cpmvIdx];
      pu.mvAffi[1][cpmvIdx] = mvAffieRMVF[1][cpmvIdx];
    }
    minCost = tmCost;
    return true;
  }
  else
  {
    pu.affineType = savedAffineType;
    return false;
  }
}

template<bool checkMv>
Distortion InterPrediction::xGetBilateralMatchingErrorAffineCheckMv(const CodingUnit &pu, Mv (&mvAffi)[2][3])
{
  const int width  = pu.Y().width;
  const int height = pu.Y().height;
  int       deltaMvHorX[2], deltaMvHorY[2], deltaMvVerX[2], deltaMvVerY[2];

  for (int i = 0; i < 2; i++)
  {
    deltaMvHorX[i] = (mvAffi[i][1] - mvAffi[i][0]).getHor() << (MAX_CU_DEPTH - floorLog2(width));
    deltaMvHorY[i] = (mvAffi[i][1] - mvAffi[i][0]).getVer() << (MAX_CU_DEPTH - floorLog2(width));
    if (pu.affineType == AffineModel::_6_PARAMS)
    {
      deltaMvVerX[i] = (mvAffi[i][2] - mvAffi[i][0]).getHor() << (MAX_CU_DEPTH - floorLog2(height));
      deltaMvVerY[i] = (mvAffi[i][2] - mvAffi[i][0]).getVer() << (MAX_CU_DEPTH - floorLog2(height));
    }
    else
    {
      deltaMvVerX[i] = -deltaMvHorY[i];
      deltaMvVerY[i] = deltaMvHorX[i];
    }
  }

  int iMvScaleTmpHor, iMvScaleTmpVer;
  int iMvScaleTmpHor0[2] = { mvAffi[0][0].getHor() << MAX_CU_DEPTH, mvAffi[1][0].getHor() << MAX_CU_DEPTH };
  int iMvScaleTmpVer0[2] = { mvAffi[0][0].getVer() << MAX_CU_DEPTH, mvAffi[1][0].getVer() << MAX_CU_DEPTH };

  const ptrdiff_t refStride = m_bmRefBuf[0].stride;
  CHECK(refStride != m_bmRefBuf[1].stride, "refStride != m_bmRefBuf[1].stride");
  const ptrdiff_t dstStride = m_bmPredBuf[0].stride;
  CHECK(dstStride != m_bmPredBuf[1].stride, "dstStride != m_bmPredBuf[1].stride");

  const int initPosX = m_bmSubBlkList.begin()->x;
  const int initPosY = m_bmSubBlkList.begin()->y;

  Pel *dst[2] = { m_bmPredBuf[0].buf, m_bmPredBuf[1].buf };

  Mv         mv[2];
  Distortion cost       = 0;
  const auto nFilterIdx = InterpolationFilter::Filter::DMVR;
  for (std::vector<BMSubBlkInfo>::iterator it = m_bmSubBlkList.begin(); it != m_bmSubBlkList.end(); ++it)
  {
    for (int i = 0; i < 2; i++)
    {
      iMvScaleTmpHor = iMvScaleTmpHor0[i] + deltaMvHorX[i] * it->m_cXInPU + deltaMvVerX[i] * it->m_cYInPU;
      iMvScaleTmpVer = iMvScaleTmpVer0[i] + deltaMvHorY[i] * it->m_cXInPU + deltaMvVerY[i] * it->m_cYInPU;
      roundAffineMv(iMvScaleTmpHor, iMvScaleTmpVer, MAX_CU_DEPTH);
      mv[i] = Mv(iMvScaleTmpHor, iMvScaleTmpVer);
      clipMv(mv[i], pu.lumaPos(), pu.lumaSize(), *pu.cs->sps, *pu.cs->pps);
    }
    const ptrdiff_t offset = it->x - initPosX + (it->y - initPosY) * dstStride;

    for (int i = 0; i < 2; i++)
    {
      if (checkMv && it->m_mv[i] == mv[i])
      {
        continue;
      }
      it->m_mv[i]    = mv[i];
      iMvScaleTmpHor = mv[i].getHor();
      iMvScaleTmpVer = mv[i].getVer();

      int xFrac, yFrac, xInt, yInt;

      xInt  = iMvScaleTmpHor >> 4;
      xFrac = iMvScaleTmpHor & 15;
      yInt  = iMvScaleTmpVer >> 4;
      yFrac = iMvScaleTmpVer & 15;

      const Pel *ref   = m_bmRefBuf[i].buf + xInt + it->x + (yInt + it->y) * refStride;
      Pel       *ptDst = &dst[i][offset];

      if (yFrac == 0)
      {
        m_if->filterHor(COMP_Y, ref, refStride, ptDst, dstStride, m_bmSubBlkW, m_bmSubBlkH, xFrac, false, m_bmClpRng,
                        nFilterIdx);
      }
      else if (xFrac == 0)
      {
        m_if->filterVer(COMP_Y, ref, refStride, ptDst, dstStride, m_bmSubBlkW, m_bmSubBlkH, yFrac, true, false,
                        m_bmClpRng, nFilterIdx);
      }
      else
      {
        m_if->filterHor(COMP_Y, ref - m_bmInterpolationHOfst, refStride, m_bmInterpolationTmpBuf.buf,
                        m_bmInterpolationTmpBuf.stride, m_bmSubBlkW, m_bmSubBlkH + m_bmFilterSize - 1, xFrac, false,
                        m_bmClpRng, nFilterIdx);
        JVET_J0090_SET_CACHE_ENABLE(false);
        m_if->filterVer(COMP_Y, m_bmInterpolationTmpBuf.buf + m_bmInterpolationVOfst, m_bmInterpolationTmpBuf.stride,
                        ptDst, dstStride, m_bmSubBlkW, m_bmSubBlkH, yFrac, false, false, m_bmClpRng, nFilterIdx);
        JVET_J0090_SET_CACHE_ENABLE(true);
      }
    }
    it->m_predReady = 1;
  }
  cost = (m_bmDistParam.distFunc(m_bmDistParam) >> m_bmCostShift);
  return cost;
}

bool InterPrediction::processBDMVR4Affine(CodingUnit &pu)
{
  if (!pu.cs->slice->m_sps->m_useDMVD || !pu.cs->slice->isInterB())
  {
    return false;
  }
  CHECK(!pu.mergeFlag, "Merge mode must be used here");
  CHECK(pu.refIdx[0] < 0 || pu.refIdx[1] < 0, "Bilateral DMVR is performed for bi-prediction");

  Mv mvFinalPu[2];
  Mv mvInitialPu[2];
  mvFinalPu[0].setZero();
  mvFinalPu[1].setZero();
  mvInitialPu[0].setZero();
  mvInitialPu[1].setZero();

  xBmAffineInit(pu);

  Distortion minCost = std::numeric_limits<Distortion>::max();
  Distortion totalCost[(AFFINE_DMVR_INT_SRCH_RANGE << 1) + 1][(AFFINE_DMVR_INT_SRCH_RANGE << 1) + 1] = { {
    0,
  } };
  xBmAffineIntSearch(pu, mvFinalPu, minCost, totalCost);

  Distortion localCostArray[9] = { std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   minCost };

  xBmAffineHPelSearch(pu, mvFinalPu, minCost, localCostArray);
  // Model-based fractional MVD optimization
  if (localCostArray[8] > 0 && localCostArray[8] == minCost)
  {
    uint64_t sadbuffer[5];
    sadbuffer[0] = (uint64_t)localCostArray[8];   // center
    sadbuffer[1] = (uint64_t)localCostArray[7];   // left
    sadbuffer[2] = (uint64_t)localCostArray[5];   // above
    sadbuffer[3] = (uint64_t)localCostArray[3];   // right
    sadbuffer[4] = (uint64_t)localCostArray[1];   // bottom

    int32_t tempDeltaMv[2] = { 0, 0 };
    xSubPelErrorSrfc(sadbuffer, tempDeltaMv);
    if (tempDeltaMv[0] != 0 || tempDeltaMv[1] != 0)
    {
      mvFinalPu[0] += Mv(tempDeltaMv[0], tempDeltaMv[1]);
      mvFinalPu[1] -= Mv(tempDeltaMv[0], tempDeltaMv[1]);
    }
  }

  bool needRecalBMCost = true;

  pu.mvAffi[0][0] += mvFinalPu[0];
  pu.mvAffi[0][1] += mvFinalPu[0];
  pu.mvAffi[0][2] += mvFinalPu[0];
  pu.mvAffi[1][0] += mvFinalPu[1];
  pu.mvAffi[1][1] += mvFinalPu[1];
  pu.mvAffi[1][2] += mvFinalPu[1];

  if (m_bmSubBlkList.size() > 2)
  {
    if (needRecalBMCost)
    {
      minCost         = xGetBilateralMatchingErrorAffine(pu, pu.mvAffi);
      needRecalBMCost = false;
    }
    xBmAffineRegression(pu, minCost);
  }
  if (pu.cs->sps->m_affineParaRefinement)
  {
    if (PU::checkBDMVRCpmvRefinementPuUsage(pu) && pu.lwidth() > m_bmSubBlkW && pu.lheight() > m_bmSubBlkH)
    {
      if (needRecalBMCost)
      {
        minCost = xGetBilateralMatchingErrorAffine(pu, pu.mvAffi);
      }
      const int  lumaArea       = pu.lumaSize().area();
      const bool isTooSmallDist = minCost < lumaArea;
      if (!isTooSmallDist)
      {
        minCost = xBDMVRMv6ParameterSearchAffine(minCost, pu);
      }
    }
  }
  return true;
}

static const int ACTIVITY_TH[MAX_QP + 1] = { 0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  2,  2,  2,  3,  3,  3,
                                             4,  4,  5,  5,  6,  6,  7,  7,  8,  9,  9,  10, 10, 11, 12, 13,
                                             13, 14, 15, 16, 17, 18, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27,
                                             29, 30, 31, 32, 33, 34, 36, 36, 36, 36, 37, 37, 37, 37, 37, 37 };
Distortion InterPrediction::getBoundaryDistortion(CodingUnit &pu, int xx, int yy, const int widthInSubPu, int theWidth,
                                                  int theHeight, bool &lowSpatAct)
{
  // get L0'a and L1's prediction blocks
  Pel *pelBuffer[2] = { nullptr, nullptr };
  pelBuffer[0]      = m_filteredBlock[3][RPL0][0] + BDMVR_CENTER_POSITION;
  pelBuffer[1]      = m_filteredBlock[3][RPL1][0] + BDMVR_CENTER_POSITION;

  PelBuf predBuf[2] = { PelBuf(pelBuffer[RPL0], BDMVR_BUF_STRIDE, pu.lwidth(), pu.lheight()),
                        PelBuf(pelBuffer[RPL1], BDMVR_BUF_STRIDE, pu.lwidth(), pu.lheight()) };

  Pel *piCurrL0 = predBuf[0].buf;
  Pel *piCurrL1 = predBuf[1].buf;

  Distortion boundaryDiff = 0;

  if (yy > 0)
  {
    // bi-prediction of current samples
    int  biPredCurr[DMVR_SUBCU_WIDTH];
    Pel *piNbAbove = m_dmvrBottomBoundary[xx + (yy - 1) * widthInSubPu];
    // topmost boundary samples bipred
    for (int ii = 0; ii < theWidth; ii++)
    {
      biPredCurr[ii] = (piCurrL0[ii] + piCurrL1[ii] + 1) >> 1;
    }
    int limit1 = 8;
    int limit2 = 7;
    int limit3 = 5;
    int limit4 = 3;
    switch (theWidth)
    {
    case 8:
      limit1 = 4;
      limit2 = 3;
      limit3 = 2;
      limit4 = 1;
      break;
    default:
      break;
    }
    // difference in characteristics
    for (int ii = 1; ii < limit1; ii++)
    {
      boundaryDiff +=
        std::abs((biPredCurr[2 * ii] - biPredCurr[2 * ii - 1]) - (piNbAbove[2 * ii] - piNbAbove[2 * ii - 1]));
    }
    for (int ii = 1; ii < limit2; ii++)
    {
      boundaryDiff +=
        std::abs((biPredCurr[2 * ii + 1] - biPredCurr[2 * ii - 1]) - (piNbAbove[2 * ii + 1] - piNbAbove[2 * ii - 1]));
    }
    for (int ii = 1; ii < limit3; ii++)
    {
      boundaryDiff +=
        std::abs((biPredCurr[4 * ii - 1] - biPredCurr[4 * ii - 4]) - (piNbAbove[4 * ii - 1] - piNbAbove[4 * ii - 4]));
    }
    for (int ii = 1; ii < limit4; ii++)
    {
      boundaryDiff +=
        std::abs((biPredCurr[8 * ii - 1] - biPredCurr[8 * ii - 8]) - (piNbAbove[8 * ii - 1] - piNbAbove[8 * ii - 8]));
    }
  }

  if (xx > 0)
  {
    // bi-prediction of current samples
    int  biPredCurr[DMVR_SUBCU_HEIGHT];
    Pel *piNbLeft = m_dmvrRightBoundary[(xx - 1) + yy * widthInSubPu];
    int  nbStride = 1;
    // left most boundary sample bipred
    for (int ii = 0; ii < theHeight; ii++)
    {
      biPredCurr[ii] = (piCurrL0[ii * BDMVR_BUF_STRIDE] + piCurrL1[ii * BDMVR_BUF_STRIDE] + 1) >> 1;
    }
    int limit1 = 8;
    int limit2 = 7;
    int limit3 = 5;
    int limit4 = 3;
    switch (theHeight)
    {
    case 8:
      limit1 = 4;
      limit2 = 3;
      limit3 = 2;
      limit4 = 1;
      break;
    default:
      break;
    }
    // difference in characteristics
    for (int ii = 1; ii < limit1; ii++)
    {
      boundaryDiff += std::abs((biPredCurr[2 * ii] - biPredCurr[2 * ii - 1]) -
                               (piNbLeft[(2 * ii) * nbStride] - piNbLeft[(2 * ii - 1) * nbStride]));
    }
    for (int ii = 1; ii < limit2; ii++)
    {
      boundaryDiff += std::abs((biPredCurr[2 * ii + 1] - biPredCurr[2 * ii - 1]) -
                               (piNbLeft[(2 * ii + 1) * nbStride] - piNbLeft[(2 * ii - 1) * nbStride]));
    }
    for (int ii = 1; ii < limit3; ii++)
    {
      boundaryDiff += std::abs((biPredCurr[4 * ii - 1] - biPredCurr[4 * ii - 4]) -
                               (piNbLeft[(4 * ii - 1) * nbStride] - piNbLeft[(4 * ii - 4) * nbStride]));
    }
    for (int ii = 1; ii < limit4; ii++)
    {
      boundaryDiff += std::abs((biPredCurr[8 * ii - 1] - biPredCurr[8 * ii - 8]) -
                               (piNbLeft[(8 * ii - 1) * nbStride] - piNbLeft[(8 * ii - 8) * nbStride]));
    }
  }
  if (xx == 0 || yy == 0)
  {
    const int spatActivityThreshold = std::min(5, ACTIVITY_TH[pu.cs->slice->m_iSliceQp]);
    // measure spatial activity
    int       maxRefs               = 2;
    for (int theRef = 0; theRef < maxRefs; theRef++)
    {
      int             blkSumAct = 0;
      const ptrdiff_t blkStride = BDMVR_BUF_STRIDE;
      const Pel      *piOrg;
      if (theRef == 0)
      {
        piOrg = piCurrL0;
      }
      else
      {
        piOrg = piCurrL1;
      }
      piOrg += blkStride;   // start from the second row
      for (int row = 1; row < theHeight; row++)
      {
        for (int col = 1; col < theWidth; col++)
        {
          blkSumAct += std::abs(piOrg[col] - piOrg[col - 1]);
          blkSumAct += std::abs(piOrg[col] - piOrg[col - blkStride]);
        }
        piOrg += blkStride;
      }
      if (blkSumAct / ((DMVR_SUBCU_WIDTH - 1) * (DMVR_SUBCU_HEIGHT - 1)) < spatActivityThreshold)
      {
        lowSpatAct = true;
        break;
      }
    }
  }
  if (xx == 0 || yy == 0)
  {
    boundaryDiff = (boundaryDiff << 2);
  }
  else
  {
    boundaryDiff = (boundaryDiff << 4);
  }
  return boundaryDiff;
}
void InterPrediction::xBDMVRMvRefinement(int xx, int yy, int widthInSubPu, Mv mvOffset, Mv (&curBestMv)[2][2],
                                         CodingUnit &pu, const Mv (&initialMv)[2], bool useMR, int useHadmard)
{
  int        numberOfCandidates = 2;
  Distortion bestBoundaryDist   = INT64_MAX;
  for (int nIdx = 0; nIdx < numberOfCandidates; nIdx++)
  {
    Mv mvCand[2];
    mvCand[0] = curBestMv[nIdx][0];
    mvCand[1] = curBestMv[nIdx][1];

    Mv theMv[2];
    if (nIdx == 0)
    {
      theMv[0] = initialMv[0] + mvOffset;
      theMv[1] = initialMv[1] - mvOffset;
    }
    else
    {
      theMv[0] = initialMv[0];
      theMv[1] = initialMv[1];
    }
    Distortion curCandCost = xBDMVRGetMatchingError(pu, theMv, useMR, useHadmard);

    bool       lowSpatAct = false;
    Distortion boundaryDiff =
      getBoundaryDistortion(pu, xx, yy, widthInSubPu, DMVR_SUBCU_WIDTH, DMVR_SUBCU_HEIGHT, lowSpatAct);
    Distortion tmpCost =
      getDecoderSideDerivedMvCost(initialMv[0], theMv[0], BDMVR_INTME_RANGE + 1, DECODER_SIDE_MV_WEIGHT);
    curCandCost += tmpCost;
    if (lowSpatAct)
    {
      // punish reference samples with low spat activity
      boundaryDiff += (curCandCost << 3);
    }
    if ((nIdx == 0) && (xx == 0 || yy == 0) &&
        ((theMv[0].getAbsHor() > 112) || (theMv[0].getAbsVer() > 112) || (theMv[1].getAbsHor() > 112) ||
         (theMv[1].getAbsVer() > 112)))
    {
      boundaryDiff += (curCandCost << 1);
    }
    else
    {
      boundaryDiff += curCandCost;
    }

    if (boundaryDiff < bestBoundaryDist)
    {
      // curBestCost = curCandCost;
      curBestMv[0][0]  = mvCand[0];
      curBestMv[0][1]  = mvCand[1];
      bestBoundaryDist = boundaryDiff;
    }
  }
  // return curBestCost;
}

void InterPrediction::xSetBoundarySamples(const CodingUnit &pu, const Mv (&mv)[2], int dx, int dy, int xx, int yy,
                                          int widthInSubPu)
{
  // Fill L0'a and L1's prediction blocks
  Pel *pelBuffer[2] = { nullptr, nullptr };
  pelBuffer[0]      = m_filteredBlock[3][RPL0][0] + BDMVR_CENTER_POSITION;
  pelBuffer[1]      = m_filteredBlock[3][RPL1][0] + BDMVR_CENTER_POSITION;

  PelBuf predBuf[2] = { PelBuf(pelBuffer[RPL0], BDMVR_BUF_STRIDE, pu.lwidth(), pu.lheight()),
                        PelBuf(pelBuffer[RPL1], BDMVR_BUF_STRIDE, pu.lwidth(), pu.lheight()) };

  for (uint32_t refList = 0; refList < NUM_RPL01; refList++)
  {
    const Picture &refPic = *pu.slice->getRefPic((RefPicList)refList, pu.refIdx[refList])->m_unscaledPic;
    xBDMVRFillBlkPredPelBuffer(pu, refPic, mv[refList], predBuf[refList], pu.cs->slice->clpRng(COMP_Y));
  }

  Pel *theBuffer0 = predBuf[0].buf;
  Pel *theBuffer1 = predBuf[1].buf;
  Pel *piBottomNb = m_dmvrBottomBoundary[xx + yy * widthInSubPu];
  Pel *piRightNb  = m_dmvrRightBoundary[xx + yy * widthInSubPu];

  // store the bi-pred samples corresponding to the final motion
  for (int i = 0; i < dx; i++)
  {
    piBottomNb[i] =
      (theBuffer0[(dy - 1) * BDMVR_BUF_STRIDE + i] + theBuffer1[(dy - 1) * BDMVR_BUF_STRIDE + i] + 1) >> 1;
  }
  for (int j = 0; j < dy; j++)
  {
    piRightNb[j] = (theBuffer0[(dx - 1) + j * BDMVR_BUF_STRIDE] + theBuffer1[(dx - 1) + j * BDMVR_BUF_STRIDE] + 1) >> 1;
  }
}

bool InterPrediction::processBDMVR(CodingUnit &cu)
{
  if (!cu.cs->sps->m_useDMVD || !cu.cs->slice->isInterB())
  {
    return false;
  }
  PROFILER_SCOPE(1, g_timeProfiler, P_DMVR);

  CHECK(!cu.mergeFlag, "Merge mode must be used here");
  CHECK(cu.refIdx[0] < 0 || cu.refIdx[1] < 0, "Bilateral DMVR is performed for bi-prediction");

  const int poc0   = cu.slice->getRefPOC(RPL0, cu.refIdx[0]);
  const int poc1   = cu.slice->getRefPOC(RPL1, cu.refIdx[1]);
  const int poc    = cu.slice->m_poc;
  int       scale0 = 256;
  int       scale1 = 256;
  if (cu.bmDir == 0)
  {
    if (abs(poc1 - poc) > abs(poc0 - poc))
    {
      scale0 = (abs(poc0 - poc) << 8) / abs(poc1 - poc);
    }
    else if (abs(poc1 - poc) < abs(poc0 - poc))
    {
      scale1 = (abs(poc1 - poc) << 8) / abs(poc0 - poc);
    }
  }
  const int lumaArea    = cu.lumaSize().area();
  bool      subPURefine = true;
  Mv        puOrgMv[2]  = { cu.mv[0], cu.mv[1] };
  {
    Distortion minCost        = std::numeric_limits<Distortion>::max();
    bool       bUseMR         = lumaArea > 64;
    Mv         mvFinalPu[2]   = { cu.mv[0], cu.mv[1] };
    Mv         mvInitialPu[2] = { cu.mv[0], cu.mv[1] };
    if (cu.bmDir == 1)
    {
      minCost = xBDMVRGetMatchingError(cu, mvInitialPu, bUseMR, 0);
      if (minCost >= lumaArea)
      {
        minCost = xBDMVRMvOneTemplateHPelSquareSearch<1>(mvFinalPu, minCost, cu, mvInitialPu, 2,
                                                         MV_FRACTIONAL_BITS_INTERNAL - 1, bUseMR, 0);
      }
    }
    else if (cu.bmDir == 2)
    {
      minCost = xBDMVRGetMatchingError(cu, mvInitialPu, bUseMR, 0);
      if (minCost >= lumaArea)
      {
        minCost = xBDMVRMvOneTemplateHPelSquareSearch<2>(mvFinalPu, minCost, cu, mvInitialPu, 2,
                                                         MV_FRACTIONAL_BITS_INTERNAL - 1, bUseMR, 0);
      }
    }
    else
    {
      minCost = xBDMVRMvSquareSearch<false>(mvFinalPu, minCost, cu, mvInitialPu, BDMVR_INTME_MAX_NUM_SEARCH_ITERATION,
                                            MV_FRACTIONAL_BITS_INTERNAL, bUseMR, 0);
      if (minCost > 0)
      {
        minCost = xBDMVRMvSquareSearch<true>(mvFinalPu, minCost, cu, mvInitialPu, 2, MV_FRACTIONAL_BITS_INTERNAL - 1,
                                             bUseMR, 0);
      }
    }
    subPURefine = minCost >= lumaArea;
    cu.mv[RPL0] = (mvFinalPu[0] - puOrgMv[0]).getScaledMv(scale0) + puOrgMv[0];
    cu.mv[RPL1] = (mvFinalPu[1] - puOrgMv[1]).getScaledMv(scale1) + puOrgMv[1];
  }

  if (!subPURefine)
  {
    // span motion to subPU
    const int dy    = std::min<int>(cu.lumaSize().height, DMVR_SUBCU_HEIGHT);
    const int dx    = std::min<int>(cu.lumaSize().width, DMVR_SUBCU_WIDTH);
    Position  puPos = cu.lumaPos();

    int       subPuIdx = 0;
    const int dmvrSubPuStrideIncr =
      DMVR_SUBPU_STRIDE - std::max(1, (int)(cu.lumaSize().width >> DMVR_SUBCU_WIDTH_LOG2));
    for (int y = puPos.y, yStart = 0; y < (puPos.y + cu.lumaSize().height); y = y + dy, yStart = yStart + dy)
    {
      for (int x = puPos.x, xStart = 0; x < (puPos.x + cu.lumaSize().width); x = x + dx, xStart = xStart + dx)
      {
        m_bdmvrSubPuMvBuf[RPL0][subPuIdx] = cu.mv[0];
        m_bdmvrSubPuMvBuf[RPL1][subPuIdx] = cu.mv[1];
        subPuIdx++;
      }
      subPuIdx += dmvrSubPuStrideIncr;
    }
    cu.mv[0] = puOrgMv[0];
    cu.mv[1] = puOrgMv[1];
    return true;
  }

  const int  dy    = std::min<int>(cu.lumaSize().height, DMVR_SUBCU_HEIGHT);
  const int  dx    = std::min<int>(cu.lumaSize().width, DMVR_SUBCU_WIDTH);
  Position   puPos = cu.lumaPos();
  CodingUnit subPu = cu;

  int       subPuIdx            = 0;
  const int dmvrSubPuStrideIncr = DMVR_SUBPU_STRIDE - std::max(1, (int)(cu.lumaSize().width >> DMVR_SUBCU_WIDTH_LOG2));

  Distortion minCost      = std::numeric_limits<Distortion>::max();
  const Mv   mvInitial[2] = { cu.mv[0], cu.mv[1] };
  Mv         mvFinal[2]   = { cu.mv[0], cu.mv[1] };
  Mv         mvOffset;

  const Distortion earlyTerminateTh       = dx * dy;
  const int        adaptiveSearchRangeHor = (dx >> 1) < BDMVR_INTME_RANGE ? (dx >> 1) : BDMVR_INTME_RANGE;
  const int        adaptiveSearchRangeVer = (dy >> 1) < BDMVR_INTME_RANGE ? (dy >> 1) : BDMVR_INTME_RANGE;
  const bool adaptRange = (adaptiveSearchRangeHor != BDMVR_INTME_RANGE || adaptiveSearchRangeVer != BDMVR_INTME_RANGE);
  const int  maxSearchRound =
    std::min(cu.bmMergeFlag ? BM_MRG_SUB_PU_INT_MAX_SRCH_ROUND : BDMVR_INTME_MAX_NUM_SEARCH_ITERATION, 5);

  // prepare cDistParam for cost calculation
  DistParam cDistParam;
  cDistParam.applyWeight = false;
  cDistParam.useMR       = false;

  Pel *pelBuffer[2] = { nullptr, nullptr };
  pelBuffer[0]      = m_filteredBlock[3][RPL0][0] + BDMVR_CENTER_POSITION;
  pelBuffer[1]      = m_filteredBlock[3][RPL1][0] + BDMVR_CENTER_POSITION;

  PelBuf predBuf[2] = { PelBuf(pelBuffer[RPL0], BDMVR_BUF_STRIDE, dx, dy),
                        PelBuf(pelBuffer[RPL1], BDMVR_BUF_STRIDE, dx, dy) };

  int useHadamard = 2;   // subsampled STAD cost function
  m_pcRdCost->setDistParam(cDistParam, predBuf[0], predBuf[1], cu.slice->clpRng(COMP_Y).bd, COMP_Y, useHadamard);

  // prepare buffer for pre-interpolaction
  const Picture &refPic0 = *cu.slice->getRefPic(RPL0, cu.refIdx[RPL0])->m_unscaledPic;
  const Picture &refPic1 = *cu.slice->getRefPic(RPL1, cu.refIdx[RPL1])->m_unscaledPic;

  int iWidthExt    = dx + (BDMVR_INTME_RANGE << 1);
  int iHeightExt   = dy + (BDMVR_INTME_RANGE << 1);
  int iWidthOffset = BDMVR_SIMD_IF_FACTOR - (iWidthExt & (BDMVR_SIMD_IF_FACTOR - 1));
  iWidthOffset &= (BDMVR_SIMD_IF_FACTOR - 1);
  iWidthExt +=
    iWidthOffset;   // This ensures that iWidthExt is a factor-of-n number, assuming BDMVR_SIMD_IF_FACTOR is equal to n

  PelBuf predBufExt[2] = { PelBuf(m_filteredBlock[3][RPL0][0], BDMVR_BUF_STRIDE, iWidthExt, iHeightExt),
                           PelBuf(m_filteredBlock[3][RPL1][0], BDMVR_BUF_STRIDE, iWidthExt, iHeightExt) };

  Mv mvTlOff((BDMVR_INTME_RANGE << MV_FRACTIONAL_BITS_INTERNAL), (BDMVR_INTME_RANGE << MV_FRACTIONAL_BITS_INTERNAL));
  Mv mvTopLeft[2] = { mvInitial[0] - mvTlOff, mvInitial[1] - mvTlOff };

  const int widthInSubPu = cu.lumaSize().width / DMVR_SUBCU_WIDTH;
  int       xx           = -1;
  int       yy           = -1;
  for (int y = puPos.y, yStart = 0; y < (puPos.y + cu.lumaSize().height); y = y + dy, yStart = yStart + dy)
  {
    yy++;
    for (int x = puPos.x, xStart = 0; x < (puPos.x + cu.lumaSize().width); x = x + dx, xStart = xStart + dx)
    {
      xx++;
      subPu.UnitArea::operator=(UnitArea(cu.chromaFormat, Area(x, y, dx, dy)));

      minCost = std::numeric_limits<Distortion>::max();

      // Pre-interpolation
      xBDMVRFillBlkPredPelBuffer(subPu, refPic0, mvTopLeft[0], predBufExt[0], cu.cs->slice->clpRng(COMP_Y));
      xBDMVRFillBlkPredPelBuffer(subPu, refPic1, mvTopLeft[1], predBufExt[1], cu.cs->slice->clpRng(COMP_Y));

      if (adaptRange)
      {
        minCost = xBDMVRMvIntPelFullSearch<true, true>(mvOffset, minCost, mvInitial, maxSearchRound,
                                                       adaptiveSearchRangeHor, adaptiveSearchRangeVer, true,
                                                       earlyTerminateTh, cDistParam, pelBuffer, BDMVR_BUF_STRIDE);
      }
      else
      {
        minCost = xBDMVRMvIntPelFullSearch<false, true>(mvOffset, minCost, mvInitial, maxSearchRound,
                                                        adaptiveSearchRangeHor, adaptiveSearchRangeVer, true,
                                                        earlyTerminateTh, cDistParam, pelBuffer, BDMVR_BUF_STRIDE);
      }
      bool refinedApplied     = false;
      bool sizeCheckFulfilled = false;
      int  bestOffsetIdx =
        (mvOffset.getVer() + BDMVR_INTME_RANGE) * BDMVR_INTME_STRIDE + (mvOffset.getHor() + BDMVR_INTME_RANGE);
      mvOffset <<= MV_FRACTIONAL_BITS_INTERNAL;
      mvFinal[0] = mvInitial[0] + mvOffset;
      mvFinal[1] = mvInitial[1] - mvOffset;
      if ((cu.lwidth() >= 32 && cu.lheight() >= 32) || (cu.lwidth() > 16 && cu.lheight() == 16) ||
          (cu.lwidth() == 16 && cu.lheight() > 16))
      {
        sizeCheckFulfilled = true;
      }
      Mv mvCandidate[2][2];
      mvCandidate[0][0] = mvFinal[0];
      mvCandidate[0][1] = mvFinal[1];
      mvCandidate[1][0] = mvInitial[0];
      mvCandidate[1][1] = mvInitial[1];
      if (minCost >= earlyTerminateTh)
      {
        minCost = m_SADsEnlargeArray_bilMrg[bestOffsetIdx];
        Distortion tmpCost =
          getDecoderSideDerivedMvCost(mvInitial[0], mvFinal[0], BDMVR_INTME_RANGE + 1, DECODER_SIDE_MV_WEIGHT);

        if (minCost >= tmpCost)
        {
          minCost += tmpCost;
          if (sizeCheckFulfilled)
          {
            minCost = xBDMVRGetMatchingError(subPu, mvFinal, false, true);
            minCost += tmpCost;
          }

          minCost = xBDMVRMvSquareSearch<true>(mvFinal, minCost /*std::numeric_limits<Distortion>::max()*/, subPu,
                                               mvInitial, 2, MV_FRACTIONAL_BITS_INTERNAL - 1, false, useHadamard);
          mvCandidate[0][0] = mvFinal[0];
          mvCandidate[0][1] = mvFinal[1];

          if (sizeCheckFulfilled && !(mvOffset.getHor() == 0 && mvOffset.getVer() == 0))
          {
            refinedApplied = true;
            mvFinal[0]     = mvInitial[0];
            mvFinal[1]     = mvInitial[1];
            minCost        = xBDMVRGetMatchingError(subPu, mvFinal, false, useHadamard);
            tmpCost        = getDecoderSideDerivedMvCost(mvInitial[0], mvCandidate[1][0], BDMVR_INTME_RANGE + 1,
                                                         DECODER_SIDE_MV_WEIGHT);
            minCost += tmpCost;
            minCost = xBDMVRMvSquareSearch<true>(mvFinal, minCost /*std::numeric_limits<Distortion>::max()*/, subPu,
                                                 mvInitial, 2, MV_FRACTIONAL_BITS_INTERNAL - 1, false, useHadamard);
            mvCandidate[1][0] = mvFinal[0];
            mvCandidate[1][1] = mvFinal[1];
          }
        }
      }
      // now make selection based on boundary distortion from two candidates
      if (sizeCheckFulfilled)
      {
        if (refinedApplied)
        {
          xBDMVRMvRefinement(xx, yy, widthInSubPu, mvOffset, mvCandidate, subPu, mvInitial, false, useHadamard);
          mvFinal[0] = mvCandidate[0][0];
          mvFinal[1] = mvCandidate[0][1];
        }
        xSetBoundarySamples(subPu, mvFinal, dx, dy, xx, yy, widthInSubPu);
      }
      m_bdmvrSubPuMvBuf[RPL0][subPuIdx] = (mvFinal[0] - mvInitial[0]).getScaledMv(scale0) + mvInitial[0];
      m_bdmvrSubPuMvBuf[RPL1][subPuIdx] = (mvFinal[1] - mvInitial[1]).getScaledMv(scale1) + mvInitial[1];
      subPuIdx++;
    }
    xx = -1;
    subPuIdx += dmvrSubPuStrideIncr;
  }

  cu.mv[0] = puOrgMv[0];
  cu.mv[1] = puOrgMv[1];
  return true;
}

void InterPrediction::xBDMVRFillBlkPredPelBuffer(const CodingUnit &cu, const Picture &refPic, const Mv &_mv,
                                                 PelBuf &dstBuf, const ClpRng &clpRng)
{
  const CompID  compID = COMP_Y;
  const CPelBuf refBuf = refPic.getRecoBuf(refPic.blocks[compID]);

  const int lumaShift = 2 + MV_FRACTIONAL_BITS_DIFF;
  const int horShift  = (lumaShift + ::getComponentScaleX(compID, cu.chromaFormat));
  const int verShift  = (lumaShift + ::getComponentScaleY(compID, cu.chromaFormat));

  Mv mv(_mv);
  clipMv(mv, cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);
  const int xInt  = mv.getHor() >> horShift;
  const int yInt  = mv.getVer() >> verShift;
  const int xFrac = mv.getHor() & ((1 << horShift) - 1);
  const int yFrac = mv.getVer() & ((1 << verShift) - 1);

  const Pel *ref       = refBuf.bufAt(cu.blocks[compID].pos().offset(xInt, yInt));
  Pel       *dst       = dstBuf.buf;
  int        refStride = (int)refBuf.stride;
  int        dstStride = (int)dstBuf.stride;
  int        bw        = (int)dstBuf.width;
  int        bh        = (int)dstBuf.height;

  const auto filterIdx = InterpolationFilter::Filter::DMVR;

  if (yFrac == 0)
  {
    m_if->filterHor(compID, (Pel *)ref, refStride, dst, dstStride, bw, bh, xFrac, false /*rndRes=!bi*/,
                    cu.slice->clpRng(compID), filterIdx);
  }
  else if (xFrac == 0)
  {
    m_if->filterVer(compID, (Pel *)ref, refStride, dst, dstStride, bw, bh, yFrac, true, false /*rndRes=!bi*/,
                    cu.slice->clpRng(compID), filterIdx);
  }
  else
  {
    int vFilterSize = NTAPS_BILINEAR;

    PelBuf tmpBuf = PelBuf(m_filteredBlockTmp[0][compID], Size(bw + 2 * BDMVR_INTME_RANGE, bh + 2 * BDMVR_INTME_RANGE));

    m_if->filterHor(compID, (Pel *)ref - ((vFilterSize >> 1) - 1) * refStride, refStride, tmpBuf.buf, tmpBuf.stride, bw,
                    bh + vFilterSize - 1, xFrac, false, cu.slice->clpRng(compID), filterIdx);
    JVET_J0090_SET_CACHE_ENABLE(false);
    m_if->filterVer(compID, tmpBuf.buf + ((vFilterSize >> 1) - 1) * tmpBuf.stride, tmpBuf.stride, dst, dstStride, bw,
                    bh, yFrac, false, false /*rndRes=!bi*/, cu.slice->clpRng(compID), filterIdx);
    JVET_J0090_SET_CACHE_ENABLE(true);
  }
}

template<uint8_t dir> void InterPrediction::xBDMVRPreInterpolation(const CodingUnit &cu, const Mv (&mvCenter)[2],
                                                                   bool doPreInterpolationFP, bool doPreInterpolationHP)
{
  if (doPreInterpolationFP)
  {
    for (auto refList: { RPL0, RPL1 })
    {
      if (!(dir & (1 << refList)))
      {
        continue;
      }

      const Picture &refPic = *cu.slice->getRefPic(refList, cu.refIdx[refList]);

      int dstStride    = MAX_CU_SIZE + (BDMVR_INTME_RANGE << 1) + (BDMVR_SIMD_IF_FACTOR - 2);
      int iWidthExt    = (int)cu.lwidth() + (BDMVR_INTME_RANGE << 1);
      int iHeightExt   = (int)cu.lheight() + (BDMVR_INTME_RANGE << 1);
      int iWidthOffset = BDMVR_SIMD_IF_FACTOR - (iWidthExt & (BDMVR_SIMD_IF_FACTOR - 1));
      iWidthOffset &= (BDMVR_SIMD_IF_FACTOR - 1);
      iWidthExt += iWidthOffset;   // This ensures that iWidthExt is a factor-of-n number, assuming BDMVR_SIMD_IF_FACTOR
                                   // is equal to n

      Mv mv = mvCenter[refList] -
        Mv((BDMVR_INTME_RANGE << MV_FRACTIONAL_BITS_INTERNAL), (BDMVR_INTME_RANGE << MV_FRACTIONAL_BITS_INTERNAL));
      PelBuf predBuf(m_filteredBlock[3][refList][0], dstStride, iWidthExt, iHeightExt);

      xBDMVRFillBlkPredPelBuffer(cu, refPic, mv, predBuf, cu.cs->slice->clpRng(COMP_Y));
    }
  }

  if (doPreInterpolationHP)
  {
    const int offset          = 0 - (1 << (MV_FRACTIONAL_BITS_INTERNAL - 1));
    const Mv  cPhaseOffset[3] = { Mv(offset, 0), Mv(offset, offset), Mv(0, offset) };

    for (auto refList: { RPL0, RPL1 })
    {
      if (!(dir & (1 << refList)))
      {
        continue;
      }

      const Picture &refPic = *cu.slice->getRefPic(refList, cu.refIdx[refList]);

      for (int phaseIdx = 0; phaseIdx < 3; phaseIdx++)
      {
        int iRefStride   = MAX_CU_SIZE + (BDMVR_INTME_RANGE << 1) + (BDMVR_SIMD_IF_FACTOR - 2);
        int iWidthExt    = (int)cu.lwidth() + 1 - (phaseIdx >> 1);
        int iHeightExt   = (int)cu.lheight() + 1 - ((2 - phaseIdx) >> 1);
        int iWidthOffset = BDMVR_SIMD_IF_FACTOR - (iWidthExt & (BDMVR_SIMD_IF_FACTOR - 1));
        iWidthOffset &= (BDMVR_SIMD_IF_FACTOR - 1);
        iWidthExt += iWidthOffset;   // This ensures that iWidthExt is a factor-of-n number, assuming
                                     // BDMVR_SIMD_IF_FACTOR is equal to n

        Mv     mv = mvCenter[refList] + cPhaseOffset[phaseIdx];
        PelBuf predBuf(m_filteredBlock[phaseIdx][refList][0], iRefStride, iWidthExt, iHeightExt);

        xBDMVRFillBlkPredPelBuffer(cu, refPic, mv, predBuf, cu.cs->slice->clpRng(COMP_Y));
      }
    }
  }
}

template<bool adaptRange, bool useHadamard>
Distortion InterPrediction::xBDMVRMvIntPelFullSearch(Mv &mvOffset, Distortion curBestCost, const Mv (&initialMv)[2],
                                                     const int32_t maxSearchRounds, const int maxHorOffset,
                                                     const int maxVerOffset, const bool earlySkip,
                                                     const Distortion earlyTerminateTh, DistParam &cDistParam,
                                                     Pel *pelBuffer[2], const int stride)
{
  // check initial cost
  mvOffset.setZero();
  cDistParam.org.buf = pelBuffer[0];
  cDistParam.cur.buf = pelBuffer[1];

#if FULL_NBIT
  if (useHadamard)
  {
    curBestCost = cDistParam.distFunc(cDistParam) >> 1;   // magic shift, benefit for early terminate
  }
  else
  {
    int32_t precisionAdj = cDistParam.bitDepth > 8 ? cDistParam.bitDepth - 8 : 0;
    curBestCost          = cDistParam.distFunc(cDistParam) >> precisionAdj;
  }
#else
  curBestCost = cDistParam.distFunc(cDistParam);
#endif

  m_SADsEnlargeArray_bilMrg[BDMVR_INTME_CENTER] = curBestCost;
  curBestCost                                   = curBestCost - (curBestCost >> 2);   // cost tuning

  if (curBestCost < earlyTerminateTh)
  {
    return curBestCost;
  }

  Distortion tmCost      = MAX_UINT64;
  Distortion prevMinCost = MAX_UINT64;

  for (int searchPrio = 1; searchPrio < maxSearchRounds; searchPrio++)
  {
    prevMinCost = curBestCost;
    for (int currIdx = 0; currIdx < m_pSearchEnlargeOffsetNum[searchPrio]; currIdx++)
    {
      tmCost              = 0;
      int horOffset       = m_pSearchEnlargeOffset_bilMrg[searchPrio][currIdx].getHor();
      int verOffset       = m_pSearchEnlargeOffset_bilMrg[searchPrio][currIdx].getVer();
      int searchOffsetIdx = m_pSearchEnlargeOffsetToIdx[searchPrio][currIdx];

      if (adaptRange)
      {
        if (abs(horOffset) > maxHorOffset || abs(verOffset) > maxVerOffset)
        {
          continue;
        }
      }

      int bufOffset      = verOffset * stride + horOffset;
      cDistParam.org.buf = pelBuffer[0] + bufOffset;
      cDistParam.cur.buf = pelBuffer[1] - bufOffset;

#if FULL_NBIT
      if (useHadamard)
      {
        m_SADsEnlargeArray_bilMrg[searchOffsetIdx] =
          cDistParam.distFunc(cDistParam) >> 1;   // magic shift, benefit for early terminate
      }
      else
      {
        int32_t precisionAdj                       = cDistParam.bitDepth > 8 ? cDistParam.bitDepth - 8 : 0;
        m_SADsEnlargeArray_bilMrg[searchOffsetIdx] = cDistParam.distFunc(cDistParam) >> precisionAdj;
      }
#else
      m_SADsEnlargeArray_bilMrg[searchOffsetIdx] = cDistParam.distFunc(cDistParam);
#endif

      tmCost += m_SADsEnlargeArray_bilMrg[searchOffsetIdx];
      tmCost += (m_SADsEnlargeArray_bilMrg[searchOffsetIdx] >> m_costShift_1_bilMrg[searchOffsetIdx]);
      tmCost += (m_SADsEnlargeArray_bilMrg[searchOffsetIdx] >> m_costShift_2_bilMrg[searchOffsetIdx]);

      if (tmCost < curBestCost)
      {
        mvOffset    = Mv(horOffset, verOffset);
        curBestCost = tmCost;
      }
    }

    if (curBestCost < earlyTerminateTh)
    {
      break;
    }
    if (earlySkip && searchPrio > 1 && prevMinCost - curBestCost < earlyTerminateTh)
    {
      break;
    }
  }
  return curBestCost;
}

template<bool hPel> Distortion InterPrediction::xBDMVRMvSquareSearch(Mv (&curBestMv)[2], Distortion curBestCost,
                                                                     CodingUnit &cu, const Mv (&initialMv)[2],
                                                                     int32_t maxSearchRounds, int32_t searchStepShift,
                                                                     bool useMR, int useHadmard)
{
  static const Mv cSearchOffset[8]   = { Mv(-1, 1), Mv(0, 1),  Mv(1, 1),   Mv(1, 0),
                                         Mv(1, -1), Mv(0, -1), Mv(-1, -1), Mv(-1, 0) };
  int             nDirectStart       = 0;
  int             nDirectEnd         = 7;
  const int       nDirectRounding    = 8;
  const int       nDirectMask        = 0x07;
  bool            doPreInterpolation = searchStepShift == MV_FRACTIONAL_BITS_INTERNAL;

  // Calculate TM cost of initial MVs, if it is not set
  if (curBestCost == std::numeric_limits<Distortion>::max())
  {
    CHECK(searchStepShift < MV_FRACTIONAL_BITS_INTERNAL - 1, "this is not possible");
    if (hPel)
    {
      doPreInterpolation = true;
      Distortion tmCost  = getDecoderSideDerivedMvCost(
        initialMv[0], curBestMv[0], BDMVR_INTME_RANGE + (MV_FRACTIONAL_BITS_INTERNAL - searchStepShift),
        DECODER_SIDE_MV_WEIGHT);
      curBestCost = xBDMVRGetMatchingError(cu, curBestMv, useMR, useHadmard);

      if (curBestCost < tmCost)
      {
        return curBestCost;
      }

      curBestCost += tmCost;
    }
    else
    {
      curBestCost = xBDMVRGetMatchingError<3>(cu, curBestMv, 0 /*subPuOffset*/, useHadmard, useMR, doPreInterpolation,
                                              searchStepShift, curBestMv, initialMv, -1);
    }
  }

  Distortion localCostArray[9] = { std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   curBestCost };

  // Iterative search process
  for (uint32_t uiRound = 0; uiRound < maxSearchRounds; uiRound++)
  {
    int nBestDirect    = -1;
    Mv  mvCurCenter[2] = { curBestMv[0], curBestMv[1] };
    doPreInterpolation |= (searchStepShift == MV_FRACTIONAL_BITS_INTERNAL - 1);

    for (int nIdx = nDirectStart; nIdx <= nDirectEnd; nIdx++)
    {
      int nDirect = (nIdx + nDirectRounding) & nDirectMask;

      Mv mvOffset(cSearchOffset[nDirect].getHor() << searchStepShift,
                  cSearchOffset[nDirect].getVer() << searchStepShift);

      if (hPel && uiRound > 0)
      {
        if ((nDirect % 2) == 0)
        {
          continue;
        }
      }
      Mv mvCand[2] = { mvCurCenter[0] + mvOffset, mvCurCenter[1] - mvOffset };
      if (!hPel)
      {
        int currentIdx = BDMVR_INTME_CENTER + ((mvCand[0] - initialMv[0]).hor >> searchStepShift) +
          ((mvCand[0] - initialMv[0]).ver >> searchStepShift) * BDMVR_INTME_STRIDE;
        if (currentIdx < 0 || currentIdx >= BDMVR_INTME_AREA)
        {
          continue;
        }
      }

      Distortion tmCost = getDecoderSideDerivedMvCost(
        initialMv[0], mvCand[0], BDMVR_INTME_RANGE + (MV_FRACTIONAL_BITS_INTERNAL - searchStepShift),
        DECODER_SIDE_MV_WEIGHT);

      if (tmCost > curBestCost)
      {
        localCostArray[nDirect] = 2 * tmCost;
        continue;
      }

      tmCost += xBDMVRGetMatchingError<3>(cu, mvCand, 0 /*subPuOffset*/, useHadmard, useMR, doPreInterpolation,
                                          searchStepShift, mvCurCenter, initialMv, nDirect);
      localCostArray[nDirect] = tmCost;
      if (hPel && uiRound > 0)
      {
        continue;
      }

      if (tmCost < curBestCost)
      {
        nBestDirect  = nDirect;
        curBestCost  = tmCost;
        curBestMv[0] = mvCand[0];
        curBestMv[1] = mvCand[1];
      }
    }

    if (nBestDirect == -1)
    {
      break;
    }

    int nStep    = 2 - (nBestDirect & 0x01);
    nDirectStart = nBestDirect - nStep;
    nDirectEnd   = nBestDirect + nStep;

    if ((uiRound + 1) < maxSearchRounds)
    {
      xBDMVRUpdateSquareSearchCostLog(localCostArray, nBestDirect);
    }
  }

  if (!hPel)
  {
    return curBestCost;
  }

  // Model-based fractional MVD optimization
  Mv mvDiff = curBestMv[0] - initialMv[0];
  if (localCostArray[8] > 0 && localCostArray[8] == curBestCost &&
      mvDiff.getAbsHor() != (BDMVR_INTME_RANGE << MV_FRACTIONAL_BITS_INTERNAL) &&
      mvDiff.getAbsVer() != (BDMVR_INTME_RANGE << MV_FRACTIONAL_BITS_INTERNAL))
  {
    uint64_t sadbuffer[5];
    sadbuffer[0] = (uint64_t)localCostArray[8];   // center
    sadbuffer[1] = (uint64_t)localCostArray[7];   // left
    sadbuffer[2] = (uint64_t)localCostArray[5];   // above
    sadbuffer[3] = (uint64_t)localCostArray[3];   // right
    sadbuffer[4] = (uint64_t)localCostArray[1];   // bottom

    int32_t tempDeltaMv[2] = { 0, 0 };
    xSubPelErrorSrfc(sadbuffer, tempDeltaMv);

    curBestMv[0] += Mv(tempDeltaMv[0], tempDeltaMv[1]);
    curBestMv[1] -= Mv(tempDeltaMv[0], tempDeltaMv[1]);
  }

  return curBestCost;
}

void InterPrediction::xDeriveCPMV(CodingUnit &pu, const Mv (&curShiftMv)[2][3], const int deltaMvHorX,
                                  const int deltaMvHorY, const int deltaMvVerX, const int deltaMvVerY, const int baseCP,
                                  Mv (&cpMV)[2][3])
{
  int width  = pu.lwidth();
  int height = pu.lheight();
  int mvx = 0, mvy = 0;
  if (baseCP == 0)
  {
    // fix top-left
    cpMV[0][0] = pu.mvAffi[0][0] + curShiftMv[0][0];
    cpMV[1][0] = pu.mvAffi[1][0] + curShiftMv[1][0];

    mvx = deltaMvHorX * width;
    mvy = deltaMvHorY * width;
    roundAffineMv(mvx, mvy, MAX_CU_DEPTH);

    cpMV[0][1] = pu.mvAffi[0][1] + curShiftMv[0][1] + Mv(mvx, mvy);
    cpMV[1][1] = pu.mvAffi[1][1] + curShiftMv[1][1] - Mv(mvx, mvy);

    mvx = deltaMvVerX * height;
    mvy = deltaMvVerY * height;
    roundAffineMv(mvx, mvy, MAX_CU_DEPTH);

    cpMV[0][2] = pu.mvAffi[0][2] + curShiftMv[0][2] + Mv(mvx, mvy);
    cpMV[1][2] = pu.mvAffi[1][2] + curShiftMv[1][2] - Mv(mvx, mvy);
  }
  else if (baseCP == 1)
  {
    // fix top-right
    mvx = -deltaMvHorX * width;
    mvy = -deltaMvHorY * width;
    roundAffineMv(mvx, mvy, MAX_CU_DEPTH);

    cpMV[0][0] = pu.mvAffi[0][0] + curShiftMv[0][0] + Mv(mvx, mvy);
    cpMV[1][0] = pu.mvAffi[1][0] + curShiftMv[1][0] - Mv(mvx, mvy);

    cpMV[0][1] = pu.mvAffi[0][1] + curShiftMv[0][1];
    cpMV[1][1] = pu.mvAffi[1][1] + curShiftMv[1][1];

    mvx = -deltaMvHorX * width + deltaMvVerX * height;
    mvy = -deltaMvHorY * width + deltaMvVerY * height;
    roundAffineMv(mvx, mvy, MAX_CU_DEPTH);

    cpMV[0][2] = pu.mvAffi[0][2] + curShiftMv[0][2] + Mv(mvx, mvy);
    cpMV[1][2] = pu.mvAffi[1][2] + curShiftMv[1][2] - Mv(mvx, mvy);
  }
  else if (baseCP == 2)
  {
    // fix bottom-left
    mvx = -deltaMvVerX * height;
    mvy = -deltaMvVerY * height;
    roundAffineMv(mvx, mvy, MAX_CU_DEPTH);

    cpMV[0][0] = pu.mvAffi[0][0] + curShiftMv[0][0] + Mv(mvx, mvy);
    cpMV[1][0] = pu.mvAffi[1][0] + curShiftMv[1][0] - Mv(mvx, mvy);

    mvx = -deltaMvVerX * height + deltaMvHorX * width;
    mvy = -deltaMvVerY * height + deltaMvHorY * width;
    roundAffineMv(mvx, mvy, MAX_CU_DEPTH);

    cpMV[0][1] = pu.mvAffi[0][1] + curShiftMv[0][1] + Mv(mvx, mvy);
    cpMV[1][1] = pu.mvAffi[1][1] + curShiftMv[1][1] - Mv(mvx, mvy);

    cpMV[0][2] = pu.mvAffi[0][2] + curShiftMv[0][2];
    cpMV[1][2] = pu.mvAffi[1][2] + curShiftMv[1][2];
  }
  else
  {
    CHECK(1, "invalid base control point");
  }
}

Distortion InterPrediction::xBDMVRMv6ParameterSearchAffine(Distortion curBestCost, CodingUnit &pu)
{
  static const int cSearchOffset[8][4] = { { 0, 0, 0, -1 }, { 0, 0, -1, 0 }, { 0, -1, 0, 0 }, { -1, 0, 0, 0 },
                                           { 0, 0, 0, 1 },  { 0, 0, 1, 0 },  { 0, 1, 0, 0 },  { 1, 0, 0, 0 } };
  Mv               curShiftMv[2][3]    = { { Mv(0, 0), Mv(0, 0), Mv(0, 0) }, { Mv(0, 0), Mv(0, 0), Mv(0, 0) } };
  int              nDirectStart        = 0;
  int              nDirectEnd          = 7;
  const int        nDirectRounding     = 8;
  const int        nDirectMask         = 0x07;
  // Calculate TM cost of initial MVs, if it is not set
  if (curBestCost == std::numeric_limits<Distortion>::max())
  {
    Mv cpMV[2][3] = { { pu.mvAffi[0][0], pu.mvAffi[0][1], pu.mvAffi[0][2] },
                      { pu.mvAffi[1][0], pu.mvAffi[1][1], pu.mvAffi[1][2] } };
    {
      curBestCost = xGetBilateralMatchingErrorAffineCheckMv<false>(pu, cpMV);
    }
  }

  Distortion localCostArray[9] = { std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   curBestCost };
  int        costCmp[4]        = { 0 };

  int baseNum = DMVR_PARA_BASE_NUM;
  int maxSearchRounds;
  int width  = pu.lwidth();
  int height = pu.lheight();

  Distortion lastBestCost = std::numeric_limits<Distortion>::max();

  bool firstBmCost = true;
  for (int baseCP = 0; baseCP < baseNum; baseCP++)
  {
    int bestDeltaMvHorX = 0;
    int bestDeltaMvHorY = 0;
    int bestDeltaMvVerX = 0;
    int bestDeltaMvVerY = 0;
    nDirectStart        = 0;
    nDirectEnd          = 7;

    if (baseCP == 0)
    {
      maxSearchRounds = DMVR_PARA_ROUND_NUM_BASE0;
    }
    else if (baseCP == 1)
    {
      maxSearchRounds = DMVR_PARA_ROUND_NUM_BASE1;
    }
    else if (baseCP == 2)
    {
      maxSearchRounds = DMVR_PARA_ROUND_NUM_BASE2;
    }
    else
    {
      THROW("Wrong control point\n");
    }

    for (int uiRound = 0; uiRound < maxSearchRounds; uiRound++)
    {
      int nBestDirect = -1;

      int curDeltaMvHorX = bestDeltaMvHorX;
      int curDeltaMvHorY = bestDeltaMvHorY;
      int curDeltaMvVerX = bestDeltaMvVerX;
      int curDeltaMvVerY = bestDeltaMvVerY;

      lastBestCost = curBestCost;

      for (int nIdx = nDirectStart; nIdx <= nDirectEnd; nIdx++)
      {
        int nDirect = (nIdx + nDirectRounding) & nDirectMask;
        int deltaMvHorX, deltaMvHorY, deltaMvVerX, deltaMvVerY;
        if (baseCP)
        {
          if ((costCmp[0] == -1 && nDirect == 4) || (costCmp[0] == 1 && nDirect == 0) ||
              (costCmp[1] == -1 && nDirect == 5) || (costCmp[1] == 1 && nDirect == 1) ||
              (costCmp[2] == -1 && nDirect == 6) || (costCmp[2] == 1 && nDirect == 2) ||
              (costCmp[3] == -1 && nDirect == 7) || (costCmp[3] == 1 && nDirect == 3))
          {
            continue;
          }
        }

        int istep = 1;
        Mv  cpMV[2][3];
        deltaMvHorX = curDeltaMvHorX + ((istep * cSearchOffset[nDirect][0]) << (MAX_CU_DEPTH - PARA_PRECISION_BIT));
        deltaMvHorY = curDeltaMvHorY + ((istep * cSearchOffset[nDirect][1]) << (MAX_CU_DEPTH - PARA_PRECISION_BIT));
        deltaMvVerX = curDeltaMvVerX + ((istep * cSearchOffset[nDirect][2]) << (MAX_CU_DEPTH - PARA_PRECISION_BIT));
        deltaMvVerY = curDeltaMvVerY + ((istep * cSearchOffset[nDirect][3]) << (MAX_CU_DEPTH - PARA_PRECISION_BIT));

        xDeriveCPMV(pu, curShiftMv, deltaMvHorX, deltaMvHorY, deltaMvVerX, deltaMvVerY, baseCP, cpMV);
        Distortion tmCost = 0;
        if (firstBmCost)
        {
          tmCost += xGetBilateralMatchingErrorAffineCheckMv<false>(pu, cpMV);
          firstBmCost = false;
        }
        else
        {
          tmCost += xGetBilateralMatchingErrorAffineCheckMv<true>(pu, cpMV);
        }
        if (baseCP == 0)
        {
          localCostArray[nDirect] = tmCost;
        }
        if (tmCost < curBestCost)
        {
          nBestDirect     = nDirect;
          curBestCost     = tmCost;
          bestDeltaMvHorX = deltaMvHorX;
          bestDeltaMvHorY = deltaMvHorY;
          bestDeltaMvVerX = deltaMvVerX;
          bestDeltaMvVerY = deltaMvVerY;
        }
      }
      if (curBestCost > lastBestCost * TH_COST)
      {
        break;
      }
      if (baseCP == 0)
      {
        costCmp[0] = localCostArray[0] < localCostArray[4] ? -1 : (localCostArray[0] > localCostArray[4] ? 1 : 0);
        costCmp[1] = localCostArray[1] < localCostArray[5] ? -1 : (localCostArray[1] > localCostArray[5] ? 1 : 0);
        costCmp[2] = localCostArray[2] < localCostArray[6] ? -1 : (localCostArray[2] > localCostArray[6] ? 1 : 0);
        costCmp[3] = localCostArray[3] < localCostArray[7] ? -1 : (localCostArray[3] > localCostArray[7] ? 1 : 0);
      }
      if (nBestDirect == -1)
      {
        break;
      }
      int nStep    = 3;
      nDirectStart = nBestDirect - nStep;
      nDirectEnd   = nBestDirect + nStep;

      if (baseCP == 0 && (uiRound + 1) < maxSearchRounds)
      {
        int prevCenter             = (nBestDirect + 4) & 0x7;
        localCostArray[prevCenter] = localCostArray[8];
        localCostArray[8]          = localCostArray[nBestDirect];
      }
    }

    int mvx = 0, mvy = 0;
    if (baseCP == 0)
    {
      // fix top-left
      mvx = bestDeltaMvHorX * width;
      mvy = bestDeltaMvHorY * width;
      roundAffineMv(mvx, mvy, MAX_CU_DEPTH);

      curShiftMv[0][1] += Mv(mvx, mvy);
      curShiftMv[1][1] -= Mv(mvx, mvy);

      mvx = bestDeltaMvVerX * height;
      mvy = bestDeltaMvVerY * height;
      roundAffineMv(mvx, mvy, MAX_CU_DEPTH);

      curShiftMv[0][2] += Mv(mvx, mvy);
      curShiftMv[1][2] -= Mv(mvx, mvy);
    }
    else if (baseCP == 1)
    {
      // fix top-right

      mvx = -bestDeltaMvHorX * width;
      mvy = -bestDeltaMvHorY * width;
      roundAffineMv(mvx, mvy, MAX_CU_DEPTH);

      curShiftMv[0][0] += Mv(mvx, mvy);
      curShiftMv[1][0] -= Mv(mvx, mvy);

      mvx = -bestDeltaMvHorX * width + bestDeltaMvVerX * height;
      mvy = -bestDeltaMvHorY * width + bestDeltaMvVerY * height;
      roundAffineMv(mvx, mvy, MAX_CU_DEPTH);

      curShiftMv[0][2] += Mv(mvx, mvy);
      curShiftMv[1][2] -= Mv(mvx, mvy);
    }
    else if (baseCP == 2)
    {
      // fix bottom-left
      mvx = -bestDeltaMvVerX * height;
      mvy = -bestDeltaMvVerY * height;
      roundAffineMv(mvx, mvy, MAX_CU_DEPTH);

      curShiftMv[0][0] += Mv(mvx, mvy);
      curShiftMv[1][0] -= Mv(mvx, mvy);

      mvx = -bestDeltaMvVerX * height + bestDeltaMvHorX * width;
      mvy = -bestDeltaMvVerY * height + bestDeltaMvHorY * width;
      roundAffineMv(mvx, mvy, MAX_CU_DEPTH);

      curShiftMv[0][1] += Mv(mvx, mvy);
      curShiftMv[1][1] -= Mv(mvx, mvy);
    }
  }
  pu.mvAffi[0][0] += curShiftMv[0][0];
  pu.mvAffi[0][1] += curShiftMv[0][1];
  pu.mvAffi[0][2] += curShiftMv[0][2];
  pu.mvAffi[1][0] += curShiftMv[1][0];
  pu.mvAffi[1][1] += curShiftMv[1][1];
  pu.mvAffi[1][2] += curShiftMv[1][2];
  return curBestCost;
}

template<uint8_t dir>
Distortion InterPrediction::xBDMVRMvOneTemplateHPelSquareSearch(Mv (&curBestMv)[2], Distortion curBestCost,
                                                                CodingUnit &cu, const Mv (&initialMv)[2],
                                                                int32_t maxSearchRounds, int32_t searchStepShift,
                                                                bool useMR, int useHadmard)
{
  if (curBestCost == 0)
  {
    return 0;
  }

  static const Mv cSearchOffset[8]   = { Mv(-1, 1), Mv(0, 1),  Mv(1, 1),   Mv(1, 0),
                                         Mv(1, -1), Mv(0, -1), Mv(-1, -1), Mv(-1, 0) };
  int             nDirectStart       = 0;
  int             nDirectEnd         = 7;
  const int       nDirectRounding    = 8;
  const int       nDirectMask        = 0x07;
  bool            doPreInterpolation = searchStepShift == MV_FRACTIONAL_BITS_INTERNAL;
  const int       curRefList         = (dir >> 1);
  const int       templateRefList    = 1 - curRefList;
  // Calculate TM cost of initial MVs, if it is not set
  if (curBestCost == std::numeric_limits<Distortion>::max())
  {
    CHECK(searchStepShift < MV_FRACTIONAL_BITS_INTERNAL - 1, "this is not possible");
    Distortion tmCost = getDecoderSideDerivedMvCost(initialMv[curRefList], curBestMv[curRefList],
                                                    BDMVR_INTME_RANGE + (MV_FRACTIONAL_BITS_INTERNAL - searchStepShift),
                                                    DECODER_SIDE_MV_WEIGHT);
    curBestCost       = xBDMVRGetMatchingError(cu, curBestMv, useMR, useHadmard);
    if (curBestCost < tmCost)
    {
      return curBestCost;
    }

    curBestCost += tmCost;
  }

  Distortion localCostArray[9] = { std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   std::numeric_limits<Distortion>::max(),
                                   curBestCost };

  // Iterative search process
  for (uint32_t uiRound = 0; uiRound < maxSearchRounds; uiRound++)
  {
    int nBestDirect    = -1;
    Mv  mvCurCenter[2] = { curBestMv[0], curBestMv[1] };
    doPreInterpolation |= (searchStepShift == MV_FRACTIONAL_BITS_INTERNAL - 1);

    for (int nIdx = nDirectStart; nIdx <= nDirectEnd; nIdx++)
    {
      int nDirect = (nIdx + nDirectRounding) & nDirectMask;

      Mv mvOffset(cSearchOffset[nDirect].getHor() << searchStepShift,
                  cSearchOffset[nDirect].getVer() << searchStepShift);

      if (uiRound > 0)
      {
        if ((nDirect % 2) == 0)
        {
          continue;
        }
      }
      Mv mvCand[2]            = { mvCurCenter[0] + mvOffset, mvCurCenter[1] - mvOffset };
      mvCand[templateRefList] = initialMv[templateRefList];
      Distortion tmCost       = getDecoderSideDerivedMvCost(
        initialMv[curRefList], mvCand[curRefList], BDMVR_INTME_RANGE + (MV_FRACTIONAL_BITS_INTERNAL - searchStepShift),
        DECODER_SIDE_MV_WEIGHT);

      if (tmCost > curBestCost)
      {
        localCostArray[nDirect] = 2 * tmCost;
        continue;
      }

      tmCost += xBDMVRGetMatchingError<dir>(cu, mvCand, 0 /*subPuOffset*/, useHadmard, useMR, doPreInterpolation,
                                            searchStepShift, mvCurCenter, initialMv, nDirect);
      localCostArray[nDirect] = tmCost;
      if (uiRound > 0)
      {
        continue;
      }

      if (tmCost < curBestCost)
      {
        nBestDirect  = nDirect;
        curBestCost  = tmCost;
        curBestMv[0] = mvCand[0];
        curBestMv[1] = mvCand[1];
      }
    }

    if (nBestDirect == -1)
    {
      break;
    }

    int nStep    = 2 - (nBestDirect & 0x01);
    nDirectStart = nBestDirect - nStep;
    nDirectEnd   = nBestDirect + nStep;

    if ((uiRound + 1) < maxSearchRounds)
    {
      xBDMVRUpdateSquareSearchCostLog(localCostArray, nBestDirect);
    }
  }

  CHECK(curBestMv[templateRefList] != initialMv[templateRefList], "this is not possible");
  // Model-based fractional MVD optimization
  Mv mvDiff = curBestMv[curRefList] - initialMv[curRefList];
  if (localCostArray[8] > 0 && localCostArray[8] == curBestCost &&
      mvDiff.getAbsHor() != (BDMVR_INTME_RANGE << MV_FRACTIONAL_BITS_INTERNAL) &&
      mvDiff.getAbsVer() != (BDMVR_INTME_RANGE << MV_FRACTIONAL_BITS_INTERNAL))
  {
    uint64_t sadbuffer[5];
    sadbuffer[0] = (uint64_t)localCostArray[8];   // center
    sadbuffer[1] = (uint64_t)localCostArray[7];   // left
    sadbuffer[2] = (uint64_t)localCostArray[5];   // above
    sadbuffer[3] = (uint64_t)localCostArray[3];   // right
    sadbuffer[4] = (uint64_t)localCostArray[1];   // bottom

    int32_t tempDeltaMv[2] = { 0, 0 };
    xSubPelErrorSrfc(sadbuffer, tempDeltaMv);

    if (dir == 1)
    {
      curBestMv[0] += Mv(tempDeltaMv[0], tempDeltaMv[1]);
    }
    else
    {
      curBestMv[1] -= Mv(tempDeltaMv[0], tempDeltaMv[1]);
    }
  }

  return curBestCost;
}

Distortion InterPrediction::xBDMVRGetMatchingError(const CodingUnit &cu, const Mv (&mv)[2], bool useMR, int useHadmard)
{
  // Fill L0'a and L1's prediction blocks
  Pel            *pelBuffer[2] = { m_filteredBlock[3][RPL0][0] + BDMVR_CENTER_POSITION,
                                   m_filteredBlock[3][RPL1][0] + BDMVR_CENTER_POSITION };
  const ptrdiff_t stride       = BDMVR_BUF_STRIDE;
  PelBuf          predBuf[2]   = { PelBuf(pelBuffer[RPL0], stride, cu.lwidth(), cu.lheight()),
                                   PelBuf(pelBuffer[RPL1], stride, cu.lwidth(), cu.lheight()) };

  for (auto refList: { RPL0, RPL1 })
  {
    const Picture &refPic = *cu.slice->getRefPic((RefPicList)refList, cu.refIdx[refList]);
    xBDMVRFillBlkPredPelBuffer(cu, refPic, mv[refList], predBuf[refList], cu.cs->slice->clpRng(COMP_Y));
  }

  // Compute distortion between L0'a and L1's prediction blocks
  DistParam cDistParam;
  cDistParam.applyWeight = false;
  cDistParam.useMR       = useMR;

  m_pcRdCost->setDistParam(cDistParam, predBuf[0], predBuf[1], cu.slice->clpRng(COMP_Y).bd, COMP_Y, useHadmard);
#if FULL_NBIT
  if (useHadmard)
  {
    return cDistParam.distFunc(cDistParam) >> 1;   // magic shift, benefit for early terminate
  }
  else
  {
    int32_t precisionAdj = cDistParam.bitDepth > 8 ? cDistParam.bitDepth - 8 : 0;
    return cDistParam.distFunc(cDistParam) >> precisionAdj;
  }
#else
  return cDistParam.distFunc(cDistParam);
#endif
}

template<uint8_t dir>
Distortion InterPrediction::xBDMVRGetMatchingError(const CodingUnit &cu, const Mv (&mv)[2], const int subPuBufOffset,
                                                   int useHadmard, bool useMR, bool &doPreInterpolation,
                                                   int32_t searchStepShift, const Mv (&mvCenter)[2],
                                                   const Mv (&mvInitial)[2], int nDirect)
{
  // Pre-interpolation
  if (doPreInterpolation)
  {
    xBDMVRPreInterpolation<dir>(cu, mvCenter, searchStepShift == MV_FRACTIONAL_BITS_INTERNAL,
                                searchStepShift == MV_FRACTIONAL_BITS_INTERNAL - 1);
    doPreInterpolation = false;
  }

  // Locate L0'a and L1's prediction blocks in pre-interpolation buffer
  const int32_t stride       = BDMVR_BUF_STRIDE;
  Pel          *pelBuffer[2] = { nullptr, nullptr };

  if (searchStepShift == MV_FRACTIONAL_BITS_INTERNAL)
  {
    Mv mvDiff[2] = { mv[0] - mvInitial[0], mv[1] - mvInitial[1] };
    mvDiff[0] >>= MV_FRACTIONAL_BITS_INTERNAL;
    mvDiff[1] >>= MV_FRACTIONAL_BITS_INTERNAL;

    pelBuffer[0] = m_filteredBlock[3][RPL0][0] + BDMVR_CENTER_POSITION;
    pelBuffer[1] = m_filteredBlock[3][RPL1][0] + BDMVR_CENTER_POSITION;

    if (dir == 1)
    {
      // fix template at refList1
      CHECK(subPuBufOffset != 0, "this is not possible");
      pelBuffer[0] += subPuBufOffset + mvDiff[0].getVer() * stride + mvDiff[0].getHor();
      pelBuffer[1] += 0;
    }
    else if (dir == 2)
    {
      CHECK(subPuBufOffset != 0, "this is not possible");
      pelBuffer[0] += 0;
      pelBuffer[1] += subPuBufOffset + mvDiff[1].getVer() * stride + mvDiff[1].getHor();
    }
    else
    {
      pelBuffer[0] += subPuBufOffset + mvDiff[0].getVer() * stride + mvDiff[0].getHor();
      pelBuffer[1] += subPuBufOffset + mvDiff[1].getVer() * stride + mvDiff[1].getHor();
    }
  }
  else if (searchStepShift == MV_FRACTIONAL_BITS_INTERNAL - 1)
  {
    const int32_t         cFracBufOffset[8] = { stride, stride, stride + 1, 1, 1, 0, 0, 0 };
    static const uint32_t phaseIdxList[4]   = { 1, 2, 1, 0 };

    int phaseIdx = phaseIdxList[nDirect & 0x3];

    if (dir == 3)
    {
      pelBuffer[0] = m_filteredBlock[phaseIdx][RPL0][0] + cFracBufOffset[nDirect];
      pelBuffer[1] = m_filteredBlock[phaseIdx][RPL1][0] + cFracBufOffset[(nDirect + 4) & 0x7];
    }
    else if (dir == 1)
    {
      pelBuffer[0] = m_filteredBlock[phaseIdx][RPL0][0] + cFracBufOffset[nDirect];
      pelBuffer[1] = m_filteredBlock[3][RPL1][0] + BDMVR_CENTER_POSITION;
    }
    else
    {
      pelBuffer[0] = m_filteredBlock[3][RPL0][0] + BDMVR_CENTER_POSITION;
      pelBuffer[1] = m_filteredBlock[phaseIdx][RPL1][0] + cFracBufOffset[(nDirect + 4) & 0x7];
    }
  }
  else
  {
    return xBDMVRGetMatchingError(cu, mv, useMR);
  }

  PelBuf predBuf[2] = { PelBuf(pelBuffer[RPL0], stride, cu.lwidth(), cu.lheight()),
                        PelBuf(pelBuffer[RPL1], stride, cu.lwidth(), cu.lheight()) };

  // Compute distortion between L0'a and L1's prediction blocks
  DistParam cDistParam;
  cDistParam.applyWeight = false;
  cDistParam.useMR       = useMR;

  m_pcRdCost->setDistParam(cDistParam, predBuf[0], predBuf[1], cu.slice->clpRng(COMP_Y).bd, COMP_Y, useHadmard);

#if FULL_NBIT
  if (useHadmard)
  {
    return cDistParam.distFunc(cDistParam) >> 1;   // magic shift, benefit for early terminate
  }
  else
  {
    int32_t precisionAdj = cDistParam.bitDepth > 8 ? cDistParam.bitDepth - 8 : 0;
    return cDistParam.distFunc(cDistParam) >> precisionAdj;
  }
#else
  return cDistParam.distFunc(cDistParam);
#endif
}

bool InterPrediction::obmcFilter(const CodingUnit &cu, PelUnitBuf &predBuf, bool luma, bool chroma)
{
  if (!cu.obmcFlag || (!luma && !chroma))
  {
    return false;
  }
  PROFILER_SCOPE(1, g_timeProfiler, P_OBMC);

  const int blkWidth    = MIN_PU_SIZE;
  const int blkHeight   = MIN_PU_SIZE;
  const int numBlks[2]  = { (int)(cu.lwidth() / blkWidth), (int)(cu.lheight() / blkHeight) };
  bool      predChanged = false;

  // do obmc for top / left CU border
  for (int dir = 0; dir < 2; dir++)
  {
    int blksDone = 0;
    // loop over all sub-blocks of top / left border
    while (blksDone < numBlks[dir])
    {
      MotionInfo        neighMi;
      MotionInfo        curMi;
      bool              isValid = false;
      const CodingUnit *neighCu = nullptr;
      const int         blks =
        PU::getSameMotionNeighbor(cu, dir, blksDone, numBlks[dir] - blksDone, curMi, neighMi, neighCu, isValid);

      if (isValid && !(curMi.isSameMiObmc(neighMi) && cu.hasSameLicParameters(*neighCu)))
      {
        Area subArea;
        switch (dir)
        {
        case 0:
          subArea = Area(cu.lumaPos().offset(blksDone * blkWidth, 0), Size(blks * blkWidth, blkHeight));
          break;
        case 1:
          subArea = Area(cu.lumaPos().offset(0, blksDone * blkHeight), Size(blkWidth, blks * blkHeight));
          break;
        default:
          THROW("unknown direction");
        }

        CodingUnit      subCu = cu;
        subCu.UnitArea::operator=(UnitArea(cu.chromaFormat, subArea));
        subCu             = neighMi;
        subCu.bdmvrRefine = false;
        CHECK(neighMi.usesLIC != neighCu->licFlag, "neighboring LIC flag mismatch in OBMC");
        CHECK(neighCu->licFlag && neighCu->geoFlag, "neighboring CU has LIC and GEO flag set in OBMC");
        subCu.geoFlag              = false;
        subCu.ciipFlag             = neighCu->ciipFlag;
        subCu.licFlag              = neighMi.usesLIC;
        subCu.licScaleAndOffset    = neighCu->licScaleAndOffset;
        const UnitArea predArea    = UnitAreaRelative(cu, subCu);
        PelUnitBuf     dstBuf      = predBuf.subBuf(predArea);
        PelUnitBuf     tmpBuf      = m_obmcTmp1.subBuf(predArea);
        bool           skipObmc[3] = { !luma, !chroma, !chroma };

        xObmcIsSkip(subCu, dstBuf, skipObmc);
        xObmcFetchPred(subCu, tmpBuf, skipObmc);
        xObmcOverlap(subCu, dir, dstBuf, tmpBuf, skipObmc);
        predChanged |= (!skipObmc[0] || !skipObmc[1] || !skipObmc[2]);
      }

      blksDone += blks;
    }
  }

  // do obmc within CU
  if (cu.affine || cu.bdmvrRefine)
  {
    for (int by = 0; by < numBlks[1]; by++)
    {
      const bool doHor[2] = { by > 0, by < numBlks[1] - 1 };
      for (int bx = 0; bx < numBlks[0]; bx++)
      {
        const bool      doVer[2] = { bx > 0, bx < numBlks[0] - 1 };
        Area            subArea  = Area(cu.lumaPos().offset(bx * blkWidth, by * blkHeight), Size(blkWidth, blkHeight));
        CodingUnit      subCu    = cu;
        subCu.UnitArea::operator=(UnitArea(cu.chromaFormat, subArea));
        subCu.bdmvrRefine       = false;
        const UnitArea predArea = UnitAreaRelative(cu, subCu);
        PelUnitBuf     dstBuf   = predBuf.subBuf(predArea);
        bool           doBlend  = false;
        doBlend                 = xObmcSubBlock(cu, subCu, dstBuf, doVer, doHor, luma, chroma);
        predChanged |= doBlend;
      }
    }
  }

  return predChanged;
}

void InterPrediction::xObmcIsSkip(const CodingUnit &cu, const PelUnitBuf &predBuf, bool skipObmc[3])
{
  const UnitArea unitArea(cu.chromaFormat, Area(Position(0, 0), cu.lumaSize()));
  PelUnitBuf     tmpBuf = m_obmcTmp2.getBuf(unitArea);
  for (const auto comp: { COMP_Y, COMP_Cb, COMP_Cr })
  {
    if (skipObmc[comp])
    {
      continue;
    }
    skipObmc[comp] = xObmcCheckSkip(cu, comp, predBuf, tmpBuf);
  }
}

bool InterPrediction::xObmcCheckSkip(const CodingUnit &cu, CompID comp, const PelUnitBuf &predBuf, PelUnitBuf &tmpBuf)
{
  const int width    = cu.blocks[comp].width;
  const int height   = cu.blocks[comp].height;
  const int bitDepth = cu.cs->sps->m_bitDepths[toChannelType(comp)];
  const int thrs     = 96 << (bitDepth - 8);

  if (width == 0 || height == 0)
  {
    return true;
  }

  for (const auto l: { RPL0, RPL1 })
  {
    if (cu.interDir & (1 << l))
    {
      const Picture *refPic = cu.slice->getRefPic(l, cu.refIdx[l])->m_unscaledPic;
      Mv             mv     = cu.mv[l];

      clipMv(mv, cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);
      mv.roundToPrecision(MvPrecision::INTERNAL, MvPrecision::ONE);
      m_subPuMC = true;
      xPredInterBlk(comp, cu, refPic, mv, tmpBuf, false, cu.cs->slice->clpRng(comp), false, false, l,
                    cu.cs->slice->getScalingRatio(l, cu.refIdx[l]));
      m_subPuMC = false;

      const Pel      *pred       = predBuf.bufs[comp].buf;
      const ptrdiff_t predStride = predBuf.bufs[comp].stride;
      const Pel      *ref        = tmpBuf.bufs[comp].buf;
      const ptrdiff_t refStride  = tmpBuf.bufs[comp].stride;
      for (int h = 0; h < height; h++, pred += predStride, ref += refStride)
      {
        for (int w = 0; w < width; w++)
        {
          if (std::abs(pred[w] - ref[w]) >= thrs)
          {
            return true;
          }
        }
      }
    }
  }
  return false;
}

void InterPrediction::xObmcFetchPred(const CodingUnit &cu, PelUnitBuf &tmpBuf, const bool skipObmc[3])
{
  CodingUnit tmpCu    = cu;
  tmpCu.affine        = false;
  tmpCu.bcwIdx        = BCW_DEFAULT;
  tmpCu.geoFlag       = false;
  tmpCu.mmvdMergeFlag = false;

  const bool luma   = !skipObmc[COMP_Y];
  const bool chroma = !skipObmc[COMP_Cb] || !skipObmc[COMP_Cr];

  if (!luma && !chroma)
  {
    // nothing to predict, can happen if xObmcIsSkip modified the skipObmc array
    return;
  }

  m_subPuMC = true;
  if (xCheckIdenticalMotion(tmpCu))
  {
    xPredInterUni(tmpCu, RPL0, tmpBuf, false, false, luma, chroma);
  }
  else
  {
    xPredInterBi(tmpCu, tmpBuf, luma, chroma, nullptr);
  }
  m_subPuMC = false;
}

void InterPrediction::xObmcOverlap(const CodingUnit &cu, int dir, PelUnitBuf &predBuf, PelUnitBuf &tmpBuf,
                                   const bool skipObmc[3])
{
  for (const auto comp: { COMP_Y, COMP_Cb, COMP_Cr })
  {
    if (skipObmc[comp])
    {
      // checking this before retrieving the width/height prevent access to a non-existant buffer
      continue;
    }

    const int width  = cu.blocks[comp].width;
    const int height = cu.blocks[comp].height;

    if (width == 0 || height == 0)
    {
      continue;
    }

    Pel            *dstBuf    = predBuf.bufs[comp].buf;
    Pel            *srcBuf    = tmpBuf.bufs[comp].buf;
    const ptrdiff_t dstStride = predBuf.bufs[comp].stride;
    const ptrdiff_t srcStride = tmpBuf.bufs[comp].stride;

    switch (dir)
    {
    case 0:   // top border
      for (int x = 0; x < width; x++)
      {
        Pel *dst = dstBuf;
        Pel *src = srcBuf;
        dst[x]   = (26 * dst[x] + 6 * src[x] + 16) >> 5;
        if (comp == COMP_Y)
        {
          dst += dstStride;
          src += srcStride;
          dst[x] = (7 * dst[x] + src[x] + 4) >> 3;
          dst += dstStride;
          src += srcStride;
          dst[x] = (15 * dst[x] + src[x] + 8) >> 4;
          dst += dstStride;
          src += srcStride;
          dst[x] = (31 * dst[x] + src[x] + 16) >> 5;
        }
      }
      break;
    case 1:   // left border
      for (int y = 0; y < height; y++)
      {
        Pel *dst = &dstBuf[y * dstStride];
        Pel *src = &srcBuf[y * srcStride];
        dst[0]   = (26 * dst[0] + 6 * src[0] + 16) >> 5;
        if (comp == COMP_Y)
        {
          dst[1] = (7 * dst[1] + src[1] + 4) >> 3;
          dst[2] = (15 * dst[2] + src[2] + 8) >> 4;
          dst[3] = (31 * dst[3] + src[3] + 16) >> 5;
        }
      }
      break;
    default:
      THROW("unknown direction");
    }
  }
}

bool InterPrediction::xObmcSubBlock(const CodingUnit &cu, CodingUnit &subCu, PelUnitBuf &predBuf, const bool doVer[2],
                                    const bool doHor[2], bool luma, bool chroma)
{
  static constexpr int obmcWeights[2][4] = { { 27, 16, 6, 0 }, { 27, 0, 0, 0 } };

  CHECK(subCu.lumaSize().width > 4 || subCu.lumaSize().height > 4, "xObmcSubBlock only defined for blocks smaller 4x4");

  Position offsets[4]     = { Position(-1, 0), Position(MIN_PU_SIZE, 0), Position(0, -1), Position(0, MIN_PU_SIZE) };
  bool     fetchDir[4]    = { doVer[0], doVer[1], doHor[0], doHor[1] };
  bool     blendDir[4]    = { false, false, false, false };
  bool     doBlend        = false;
  const MotionInfo &curMi = cu.getMotionInfo(subCu.lumaPos());
  const bool        skipObmc[3] = { !luma, !chroma, !chroma };

  // fetch prediction from surrounding sub-blocks
  for (int dir = 0; dir < 4; dir++)
  {
    if (fetchDir[dir])
    {
      Position          neighPos = subCu.lumaPos().offset(offsets[dir]);
      const MotionInfo &neighMi  = cu.getMotionInfo(neighPos);
      if (curMi.isSameMiObmc(neighMi))
      {
        continue;
      }
      subCu = neighMi;
      xObmcFetchPred(subCu, m_obmcSubPred[dir], skipObmc);
      blendDir[dir] = true;
      doBlend       = true;
    }
  }

  // merge all predictions
  if (doBlend)
  {
    // prepare weight matrix
    int weights[4][2][4];
    for (int dir = 0; dir < 4; dir++)
    {
      if (blendDir[dir])
      {
        std::memcpy(weights[dir][0], obmcWeights, 8 * sizeof(int));
      }
      else
      {
        std::memset(weights[dir][0], 0, 8 * sizeof(int));
      }
    }
    // apply blending of sub block
    for (const auto comp: { COMP_Y, COMP_Cb, COMP_Cr })
    {
      if (skipObmc[comp])
      {
        continue;
      }
      xObmcSubBlockBlend(subCu, predBuf, comp, weights);
    }
  }

  return doBlend;
}

void InterPrediction::xObmcSubBlockBlend(const CodingUnit &subCu, PelUnitBuf &predBuf, CompID comp,
                                         const int weights[4][2][4])
{
  static constexpr int shift     = 7;
  static constexpr int add       = 1 << (shift - 1);
  static constexpr int maxWeight = 1 << shift;

  const int chan   = (int)toChannelType(comp);
  const int width  = subCu.blocks[comp].width;
  const int height = subCu.blocks[comp].height;
  Pel      *dst    = predBuf.bufs[comp].buf;
  Pel *src[4] = { m_obmcSubPred[0].bufs[comp].buf, m_obmcSubPred[1].bufs[comp].buf, m_obmcSubPred[2].bufs[comp].buf,
                  m_obmcSubPred[3].bufs[comp].buf };
  const ptrdiff_t dstStride = predBuf.bufs[comp].stride;
  const ptrdiff_t srcStride = m_obmcSubPred[0].bufs[comp].stride;
  for (int y = 0; y < height; y++)
  {
    const int invY = height - (y + 1);
    for (int x = 0; x < width; x++)
    {
      const int invX      = width - (x + 1);
      const int sumWeight = weights[0][chan][x] + weights[1][chan][invX] + weights[2][chan][y] + weights[3][chan][invY];
      if (sumWeight == 0)
      {
        continue;
      }
      const int remWeight = maxWeight - sumWeight;
      dst[x]              = (Pel)((remWeight * (int)dst[x] + weights[0][chan][x] * (int)src[0][x] +
                      weights[1][chan][invX] * (int)src[1][x] + weights[2][chan][y] * (int)src[2][x] +
                      weights[3][chan][invY] * (int)src[3][x] + add) >>
                     shift);
    }
    dst += dstStride;
    src[0] += srcStride;
    src[1] += srcStride;
    src[2] += srcStride;
    src[3] += srcStride;
  }
}

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
void InterPrediction::cacheAssign(CacheModel *cache)
{
  m_cacheModel = cache;
  m_if->cacheAssign(cache);
  m_if->initInterpolationFilter(!cache->isCacheEnable());
}
#endif

void InterPrediction::xFillIBCBuffer(CodingUnit &cu)
{
  CodingUnit &currCU = cu;

  for (const CompArea &area: currCU.blocks)
  {
    if (!area.valid())
    {
      continue;
    }

    const int      shiftSampleHor = ::getComponentScaleX(area.compID, cu.chromaFormat);
    const int      shiftSampleVer = ::getComponentScaleY(area.compID, cu.chromaFormat);
    const int      pux            = area.x % (m_ibcBufferWidth >> shiftSampleHor);
    const int      puy            = area.y % (m_ibcBufferHeight >> shiftSampleVer);
    const CompArea dstArea = CompArea(area.compID, cu.chromaFormat, Position(pux, puy), Size(area.width, area.height));
    CPelBuf        srcBuf  = cu.cs->getRecoBuf(area);
    PelBuf         dstBuf  = m_ibcBuffer.getBuf(dstArea);

    dstBuf.copyFrom(srcBuf);
  }
}

void InterPrediction::xIntraBlockCopy(CodingUnit &cu, PelUnitBuf &predBuf, const CompID compID)
{
  const int shiftSampleHor = ::getComponentScaleX(compID, cu.chromaFormat);
  const int shiftSampleVer = ::getComponentScaleY(compID, cu.chromaFormat);
  cu.bv                    = cu.mv[RPL0];
  cu.bv.changePrecision(MvPrecision::INTERNAL, MvPrecision::ONE);
  if (cu.cs->sps->m_ibcFracFlag)
  {
    const int bvShiftHor = MV_FRACTIONAL_BITS_INTERNAL + ::getComponentScaleX(compID, cu.chromaFormat);
    const int bvShiftVer = MV_FRACTIONAL_BITS_INTERNAL + ::getComponentScaleY(compID, cu.chromaFormat);
    int       xFrac      = cu.mv[RPL0].hor & ((1 << bvShiftHor) - 1);
    int       yFrac      = cu.mv[RPL0].ver & ((1 << bvShiftVer) - 1);

    if (xFrac != 0 || yFrac != 0)
    {
      xPredInterBlk(compID, cu, cu.slice->m_pic, cu.mv[0], predBuf, false, cu.slice->clpRng(compID), false, true, RPL0);
      return;
    }
  }
  int refx, refy;
  if (isLuma(compID))
  {
    refx = cu.lx() + cu.bv.hor;
    refy = cu.ly() + cu.bv.ver;
  }
  else
  {   // Cb or Cr
    refx = cu.Cb().x + (cu.bv.hor >> shiftSampleHor);
    refy = cu.Cb().y + (cu.bv.ver >> shiftSampleVer);
  }
  refx = refx % (m_ibcBufferWidth >> shiftSampleHor);
  refy = refy % (m_ibcBufferHeight >> shiftSampleVer);
  refx += (refx < 0) ? (m_ibcBufferWidth >> shiftSampleHor) : 0;
  refy += (refy < 0) ? (m_ibcBufferHeight >> shiftSampleVer) : 0;

  if (refy + predBuf.bufs[compID].height <= (m_ibcBufferHeight >> shiftSampleVer))
  {
    const CompArea srcArea = CompArea(compID, cu.chromaFormat, Position(refx, refy),
                                      Size(predBuf.bufs[compID].width, predBuf.bufs[compID].height));
    const CPelBuf  refBuf  = m_ibcBuffer.getBuf(srcArea);
    predBuf.bufs[compID].copyFrom(refBuf);
  }
  else
  {   // wrap around
    int      height = (m_ibcBufferHeight >> shiftSampleVer) - refy;
    CompArea srcArea =
      CompArea(compID, cu.chromaFormat, Position(refx, refy), Size(predBuf.bufs[compID].width, height));
    CPelBuf srcBuf = m_ibcBuffer.getBuf(srcArea);
    PelBuf  dstBuf = predBuf.bufs[compID].subBuf(0, 0, predBuf.bufs[compID].width, height);
    dstBuf.copyFrom(srcBuf);

    height  = refy + predBuf.bufs[compID].height - (m_ibcBufferHeight >> shiftSampleVer);
    srcArea = CompArea(compID, cu.chromaFormat, Position(refx, 0), Size(predBuf.bufs[compID].width, height));
    srcBuf  = m_ibcBuffer.getBuf(srcArea);
    dstBuf =
      predBuf.bufs[compID].subBuf(0, (m_ibcBufferHeight >> shiftSampleVer) - refy, predBuf.bufs[compID].width, height);
    dstBuf.copyFrom(srcBuf);
  }
}

void InterPrediction::resetIBCBuffer(const ChromaFormat chromaFormatIdc, const int ctuSize)
{
  const UnitArea area = UnitArea(chromaFormatIdc, Area(0, 0, m_ibcBufferWidth, m_ibcBufferHeight));
  m_ibcBuffer.getBuf(area).fill(-1);
}

void InterPrediction::resetVPDUforIBC(const ChromaFormat chromaFormatIdc, const int ctuSize, const int vSize,
                                      const int xPos, const int yPos)
{
  if (xPos == 0)
  {
    const UnitArea area = UnitArea(chromaFormatIdc, Area(0, yPos % m_ibcBufferHeight, m_ibcBufferWidth, ctuSize));
    m_ibcBuffer.getBuf(area).fill(-1);
  }

  if (256 == ctuSize)
  {
    if (xPos - 2 * ctuSize >= 0)
    {
      const UnitArea area =
        UnitArea(chromaFormatIdc,
                 Area((xPos - 2 * ctuSize) % m_ibcBufferWidth, (yPos + ctuSize) % m_ibcBufferHeight, ctuSize, ctuSize));
      m_ibcBuffer.getBuf(area).fill(-1);
    }
  }
  else if (xPos - 3 * ctuSize >= 0)
  {
    const UnitArea area =
      UnitArea(chromaFormatIdc,
               Area((xPos - 3 * ctuSize) % m_ibcBufferWidth, (yPos + ctuSize) % m_ibcBufferHeight, ctuSize, ctuSize));
    m_ibcBuffer.getBuf(area).fill(-1);
  }
}

bool InterPrediction::isLumaBvValid(const int ctuSize, const int xCb, const int yCb, const int width, const int height,
                                    const int xBv, const int yBv)
{
  int    refTLx = xCb + xBv;
  int    refTLy = yCb + yBv;
  PelBuf buf    = m_ibcBuffer.Y();
  for (int x = 0; x < width; x += 4)
  {
    for (int y = 0; y < height; y += 4)
    {
      if (buf.at((x + refTLx) % m_ibcBufferWidth, (y + refTLy) % m_ibcBufferHeight) == -1)
      {
        return false;
      }
      if (buf.at((x + 3 + refTLx) % m_ibcBufferWidth, (y + refTLy) % m_ibcBufferHeight) == -1)
      {
        return false;
      }
      if (buf.at((x + refTLx) % m_ibcBufferWidth, (y + 3 + refTLy) % m_ibcBufferHeight) == -1)
      {
        return false;
      }
      if (buf.at((x + 3 + refTLx) % m_ibcBufferWidth, (y + 3 + refTLy) % m_ibcBufferHeight) == -1)
      {
        return false;
      }
    }
  }
  return true;
}

bool InterPrediction::xPredInterBlkRPR(const ScalingRatio scalingRatio, const PPS &pps, const CompArea &blk,
                                       const Picture *refPic, const Mv &mv, Pel *dst, const ptrdiff_t dstStride,
                                       const bool bi, const bool wrapRef, const ClpRng &clpRng,
                                       const InterpolationFilter::Filter filterIndex, const bool useAltHpelIf)
{
  const ChromaFormat chFmt  = blk.chromaFormat;
  const CompID       compID = blk.compID;
  const bool         rndRes = !bi;

  int shiftHor = MV_FRACTIONAL_BITS_INTERNAL + (isLuma(compID) ? 0 : 1);
  int shiftVer = MV_FRACTIONAL_BITS_INTERNAL + (isLuma(compID) ? 0 : 1);

  int     width  = blk.width;
  int     height = blk.height;
  CPelBuf refBuf;

  const bool scaled = refPic->isRefScaled(&pps);

  if (scaled)
  {
    int row, col;
    int refPicWidth  = refPic->getPicWidthInLumaSamples();
    int refPicHeight = refPic->getPicHeightInLumaSamples();

    InterpolationFilter::Filter xFilter = filterIndex;
    InterpolationFilter::Filter yFilter = filterIndex;

    const int rprThreshold1 = (1 << ScalingRatio::BITS) * 5 / 4;
    const int rprThreshold2 = (1 << ScalingRatio::BITS) * 7 / 4;

    if (filterIndex == InterpolationFilter::Filter::DEFAULT || !isLuma(compID))
    {
      if (scalingRatio.x > rprThreshold2)
      {
        xFilter = InterpolationFilter::Filter::RPR2;
      }
      else if (scalingRatio.x > rprThreshold1)
      {
        xFilter = InterpolationFilter::Filter::RPR1;
      }

      if (scalingRatio.y > rprThreshold2)
      {
        yFilter = InterpolationFilter::Filter::RPR2;
      }
      else if (scalingRatio.y > rprThreshold1)
      {
        yFilter = InterpolationFilter::Filter::RPR1;
      }
    }
    else if (filterIndex == InterpolationFilter::Filter::AFFINE)
    {
      if (scalingRatio.x > rprThreshold2)
      {
        xFilter = InterpolationFilter::Filter::AFFINE_RPR2;
      }
      else if (scalingRatio.x > rprThreshold1)
      {
        xFilter = InterpolationFilter::Filter::AFFINE_RPR1;
      }

      if (scalingRatio.y > rprThreshold2)
      {
        yFilter = InterpolationFilter::Filter::AFFINE_RPR2;
      }
      else if (scalingRatio.y > rprThreshold1)
      {
        yFilter = InterpolationFilter::Filter::AFFINE_RPR1;
      }
    }

    if (useAltHpelIf)
    {
      if (xFilter == InterpolationFilter::Filter::DEFAULT && scalingRatio.x == SCALE_1X.x)
      {
        xFilter = InterpolationFilter::Filter::HALFPEL_ALT;
      }
      if (yFilter == InterpolationFilter::Filter::DEFAULT && scalingRatio.y == SCALE_1X.y)
      {
        yFilter = InterpolationFilter::Filter::HALFPEL_ALT;
      }
    }
    const int posShift = ScalingRatio::BITS - 4;

    const int stepX = (scalingRatio.x + 8) >> 4;
    const int stepY = (scalingRatio.y + 8) >> 4;

    const int offX = 1 << (posShift - shiftHor - 1);
    const int offY = 1 << (posShift - shiftVer - 1);

    const uint32_t scaleX = ::getComponentScaleX(compID, chFmt);
    const uint32_t scaleY = ::getComponentScaleY(compID, chFmt);

    const int64_t posX =
      ((blk.pos().x << scaleX) - (pps.m_scalingWindow.m_winLeftOffset * SPS::getWinUnitX(chFmt))) >> scaleX;
    const int64_t posY =
      ((blk.pos().y << scaleY) - (pps.m_scalingWindow.m_winTopOffset * SPS::getWinUnitY(chFmt))) >> scaleY;

    int addX =
      isLuma(compID) ? 0 : int(1 - refPic->m_cs->sps->m_horCollocatedChromaFlag) * 8 * (scalingRatio.x - SCALE_1X.x);
    int addY =
      isLuma(compID) ? 0 : int(1 - refPic->m_cs->sps->m_verCollocatedChromaFlag) * 8 * (scalingRatio.y - SCALE_1X.y);

    int boundLeft   = 0;
    int boundRight  = refPicWidth >> scaleX;
    int boundTop    = 0;
    int boundBottom = refPicHeight >> scaleY;
    if (refPic->m_subPictures.size() > 1)
    {
      const SubPic &curSubPic = pps.getSubPicFromPos(blk.lumaPos());
      if (curSubPic.m_treatedAsPicFlag)
      {
        boundLeft   = curSubPic.m_subPicLeft >> scaleX;
        boundRight  = curSubPic.m_subPicRight >> scaleX;
        boundTop    = curSubPic.m_subPicTop >> scaleY;
        boundBottom = curSubPic.m_subPicBottom >> scaleY;
      }
    }

    int64_t x0Int;
    int64_t y0Int;

    x0Int = ((posX << (4 + scaleX)) + mv.getHor()) * (int64_t)scalingRatio.x + addX;
    x0Int = sgn2(x0Int) * ((abs(x0Int) + (1ull << (7 + scaleX))) >> (8 + scaleX)) +
      ((refPic->m_scalingWindow.m_winLeftOffset * SPS::getWinUnitX(chFmt)) << ((posShift - scaleX)));

    y0Int = ((posY << (4 + scaleY)) + mv.getVer()) * (int64_t)scalingRatio.y + addY;
    y0Int = sgn2(y0Int) * ((abs(y0Int) + (1ull << (7 + scaleY))) >> (8 + scaleY)) +
      ((refPic->m_scalingWindow.m_winTopOffset * SPS::getWinUnitY(chFmt)) << ((posShift - scaleY)));

    const int extSize     = isLuma(compID) ? 1 : 2;
    int       vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;

    int yInt0 = ((int32_t)y0Int + offY) >> posShift;
    yInt0     = std::min(std::max(boundTop - (NTAPS_LUMA / 2), yInt0), boundBottom + (NTAPS_LUMA / 2));

    int xInt0 = ((int32_t)x0Int + offX) >> posShift;
    xInt0     = std::min(std::max(boundLeft - (NTAPS_LUMA / 2), xInt0), boundRight + (NTAPS_LUMA / 2));

    int refHeight = ((((int32_t)y0Int + (height - 1) * stepY) + offY) >> posShift) -
      ((((int32_t)y0Int + 0 * stepY) + offY) >> posShift) + 1;
    refHeight = std::max<int>(1, refHeight);

    CHECK(TMP_RPR_HEIGHT < refHeight + vFilterSize - 1 + extSize,
          "Buffer is not large enough, increase MAX_SCALING_RATIO");

    int tmpStride = width;
    int xInt = 0, yInt = 0;

    for (col = 0; col < width; col++)
    {
      int posX  = (int32_t)x0Int + col * stepX;
      xInt      = (posX + offX) >> posShift;
      xInt      = std::min(std::max(boundLeft - (NTAPS_LUMA / 2), xInt), boundRight + (NTAPS_LUMA / 2));
      int xFrac = ((posX + offX) >> (posShift - shiftHor)) & ((1 << shiftHor) - 1);

      CHECK(xInt0 > xInt, "Wrong horizontal starting point");

      Position offset = Position(xInt, yInt0);
      refBuf          = refPic->getRecoBuf(CompArea(compID, chFmt, offset, Size(1, refHeight)), wrapRef);

      Pel *const tempBuf = m_filteredBlockTmpRPR + col;

      m_if->filterHor(compID, (Pel *)refBuf.buf - ((vFilterSize >> 1) - 1) * refBuf.stride, refBuf.stride, tempBuf,
                      tmpStride, 1, refHeight + vFilterSize - 1 + extSize, xFrac, false, clpRng, xFilter);
    }

    for (row = 0; row < height; row++)
    {
      int posY  = (int32_t)y0Int + row * stepY;
      yInt      = (posY + offY) >> posShift;
      yInt      = std::min(std::max(boundTop - (NTAPS_LUMA / 2), yInt), boundBottom + (NTAPS_LUMA / 2));
      int yFrac = ((posY + offY) >> (posShift - shiftVer)) & ((1 << shiftVer) - 1);

      CHECK(yInt0 > yInt, "Wrong vertical starting point");

      const Pel *const tempBuf = m_filteredBlockTmpRPR + (yInt - yInt0) * tmpStride;

      JVET_J0090_SET_CACHE_ENABLE(false);
      m_if->filterVer(compID, tempBuf + ((vFilterSize >> 1) - 1) * tmpStride, tmpStride, dst + row * dstStride,
                      dstStride, width, 1, yFrac, false, rndRes, clpRng, yFilter);
      JVET_J0090_SET_CACHE_ENABLE(true);
    }
  }

  return scaled;
}

void MergeCtx::setMergeInfo(CodingUnit &cu, int candIdx) const
{
  cu.mergeIdx = candIdx;

  CHECK(candIdx >= numValidMergeCand, "Merge candidate does not exist");
  cu.regularMergeFlag = !(cu.ciipFlag || cu.geoFlag);
  cu.mergeFlag        = true;
  cu.mmvdMergeFlag    = false;
  cu.interDir         = interDirNeighbours[candIdx];
  cu.imv              = (!cu.geoFlag && useAltHpelIf[candIdx]) ? IMV_HPEL : 0;
  cu.mergeType        = CU::isIBC(cu) ? MergeType::IBC : MergeType::DEFAULT_N;
  cu.licFlag          = LICFlags[candIdx] ^ cu.oppositeLicFlag;
  CHECK(!cu.cs->slice->m_useLic && LICFlags[candIdx], "LIC flag set despite being disabled for slice");
  CHECK(!cu.cs->sps->m_biLicEnabledFlag && cu.interDir == 3 && cu.licFlag,
        "LIC is not used with bi-prediction in merge");
  cu.obmcFlag    = CU::isObmcAllowed(cu);
  cu.bdmvrRefine = false;

  for (const auto l: { RPL0, RPL1 })
  {
    cu.mv[l]     = mvFieldNeighbours[candIdx][l].mv;
    cu.mvd[l]    = Mv();
    cu.refIdx[l] = mvFieldNeighbours[candIdx][l].refIdx;
    cu.mvpIdx[l] = NOT_VALID;
    cu.mvpNum[l] = NOT_VALID;
  }

  if (CU::isIBC(cu))
  {
    cu.bv = cu.mv[RPL0];
    cu.bv.changePrecision(MvPrecision::INTERNAL, MvPrecision::ONE);   // used for only integer resolution
    if (!cu.cs->sps->m_ibcFracFlag)
    {
      cu.imv = cu.imv == IMV_HPEL ? 0 : cu.imv;
    }
  }
  cu.bcwIdx = (interDirNeighbours[candIdx] == 3) ? bcwIdx[candIdx] : BCW_DEFAULT;

  PU::restrictBiPredMergeCandsOne(cu);
  cu.mmvdEncOptMode = 0;
}

void MergeCtx::getMmvdDeltaMv(const Slice &slice, const MmvdIdx candIdx, Mv deltaMv[NUM_RPL01]) const
{
  const int mvdBaseIdx  = candIdx.pos.baseIdx;
  const int mvdStep     = candIdx.pos.step;
  const int mvdPosition = candIdx.pos.position;

  int offset = 1 << (mvdStep + MV_FRACTIONAL_BITS_DIFF);
  if (slice.m_picHeader->m_disFracMMVD)
  {
    offset <<= 2;
  }
  const int refList0 = mmvdBaseMv[mvdBaseIdx][RPL0].refIdx;
  const int refList1 = mmvdBaseMv[mvdBaseIdx][RPL1].refIdx;

  const Mv dMvTable[4] = { Mv(offset, 0), Mv(-offset, 0), Mv(0, offset), Mv(0, -offset) };
  if ((refList0 != -1) && (refList1 != -1))
  {
    const int poc0    = slice.getRefPOC(RPL0, refList0);
    const int poc1    = slice.getRefPOC(RPL1, refList1);
    const int currPoc = slice.m_poc;
    deltaMv[0]        = dMvTable[mvdPosition];

    if ((poc0 - currPoc) == (poc1 - currPoc))
    {
      deltaMv[1] = deltaMv[0];
    }
    else if (abs(poc1 - currPoc) > abs(poc0 - currPoc))
    {
      const int scale            = PU::getDistScaleFactor(currPoc, poc0, currPoc, poc1);
      deltaMv[1]                 = deltaMv[0];
      const bool isL0RefLongTerm = slice.getRefPic(RPL0, refList0)->m_longTerm;
      const bool isL1RefLongTerm = slice.getRefPic(RPL1, refList1)->m_longTerm;
      if (isL0RefLongTerm || isL1RefLongTerm)
      {
        if ((poc1 - currPoc) * (poc0 - currPoc) > 0)
        {
          deltaMv[0] = deltaMv[1];
        }
        else
        {
          deltaMv[0].set(-1 * deltaMv[1].getHor(), -1 * deltaMv[1].getVer());
        }
      }
      else
      {
        deltaMv[0] = deltaMv[1].getScaledMv(scale);
      }
    }
    else
    {
      const int  scale           = PU::getDistScaleFactor(currPoc, poc1, currPoc, poc0);
      const bool isL0RefLongTerm = slice.getRefPic(RPL0, refList0)->m_longTerm;
      const bool isL1RefLongTerm = slice.getRefPic(RPL1, refList1)->m_longTerm;
      if (isL0RefLongTerm || isL1RefLongTerm)
      {
        if ((poc1 - currPoc) * (poc0 - currPoc) > 0)
        {
          deltaMv[1] = deltaMv[0];
        }
        else
        {
          deltaMv[1].set(-1 * deltaMv[0].getHor(), -1 * deltaMv[0].getVer());
        }
      }
      else
      {
        deltaMv[1] = deltaMv[0].getScaledMv(scale);
      }
    }
  }
  else if (refList0 != -1)
  {
    deltaMv[0] = dMvTable[mvdPosition];
  }
  else if (refList1 != -1)
  {
    deltaMv[1] = dMvTable[mvdPosition];
  }
}

void MergeCtx::setMmvdMergeCandiInfo(CodingUnit &cu, const MmvdIdx candIdx)
{
  Mv tempMv[NUM_RPL01];

  getMmvdDeltaMv(*cu.cs->slice, candIdx, tempMv);
  const int mvdBaseIdx = candIdx.pos.baseIdx;

  const int refList0 = mmvdBaseMv[mvdBaseIdx][0].refIdx;
  const int refList1 = mmvdBaseMv[mvdBaseIdx][1].refIdx;

  if ((refList0 != -1) && (refList1 != -1))
  {
    cu.interDir     = 3;
    cu.mv[RPL0]     = mmvdBaseMv[mvdBaseIdx][0].mv + tempMv[0];
    cu.refIdx[RPL0] = refList0;
    cu.mv[RPL1]     = mmvdBaseMv[mvdBaseIdx][1].mv + tempMv[1];
    cu.refIdx[RPL1] = refList1;
  }
  else if (refList0 != -1)
  {
    cu.interDir     = 1;
    cu.mv[RPL0]     = mmvdBaseMv[mvdBaseIdx][0].mv + tempMv[0];
    cu.refIdx[RPL0] = refList0;
    cu.mv[RPL1]     = Mv(0, 0);
    cu.refIdx[RPL1] = -1;
  }
  else if (refList1 != -1)
  {
    cu.interDir     = 2;
    cu.mv[RPL0]     = Mv(0, 0);
    cu.refIdx[RPL0] = -1;
    cu.mv[RPL1]     = mmvdBaseMv[mvdBaseIdx][1].mv + tempMv[1];
    cu.refIdx[RPL1] = refList1;
  }

  cu.mmvdMergeFlag    = true;
  cu.mmvdMergeIdx     = candIdx;
  cu.mergeFlag        = true;
  cu.regularMergeFlag = true;
  cu.mergeIdx         = candIdx.val;
  cu.mergeType        = MergeType::DEFAULT_N;
  cu.licFlag          = LICFlags[mvdBaseIdx];
  cu.oppositeLicFlag  = false;
  cu.obmcFlag         = CU::isObmcAllowed(cu);

  cu.mvd[RPL0]    = Mv();
  cu.mvd[RPL1]    = Mv();
  cu.mvpIdx[RPL0] = NOT_VALID;
  cu.mvpIdx[RPL1] = NOT_VALID;
  cu.mvpNum[RPL0] = NOT_VALID;
  cu.mvpNum[RPL1] = NOT_VALID;
  cu.imv          = mmvdUseAltHpelIf[mvdBaseIdx] ? IMV_HPEL : 0;

  cu.bcwIdx = (interDirNeighbours[mvdBaseIdx] == 3) ? bcwIdx[mvdBaseIdx] : BCW_DEFAULT;

  for (int refList = 0; refList < 2; refList++)
  {
    if (cu.refIdx[refList] >= 0)
    {
      cu.mv[refList].clipToStorageBitDepth();
    }
  }

  PU::restrictBiPredMergeCandsOne(cu);
}

bool MergeCtx::checkSimilarMotion(int mergeCandIndex, uint32_t mvdSimilarityThresh) const
{
  if (mvFieldNeighbours[mergeCandIndex][0].refIdx < 0 && mvFieldNeighbours[mergeCandIndex][1].refIdx < 0)
  {
    return true;
  }

  int mvdTh = std::max<int>(mvdSimilarityThresh, 1);

  for (uint32_t ui = 0; ui < mergeCandIndex; ui++)
  {
    if (interDirNeighbours[ui] == interDirNeighbours[mergeCandIndex])
    {
      if (interDirNeighbours[ui] == 3)
      {
        if (mvFieldNeighbours[ui][0].refIdx == mvFieldNeighbours[mergeCandIndex][0].refIdx &&
            mvFieldNeighbours[ui][1].refIdx == mvFieldNeighbours[mergeCandIndex][1].refIdx)
        {
          Mv mvDiffL0 = mvFieldNeighbours[ui][0].mv - mvFieldNeighbours[mergeCandIndex][0].mv;
          Mv mvDiffL1 = mvFieldNeighbours[ui][1].mv - mvFieldNeighbours[mergeCandIndex][1].mv;

          if (mvDiffL0.getAbsHor() < mvdTh && mvDiffL0.getAbsVer() < mvdTh && mvDiffL1.getAbsHor() < mvdTh &&
              mvDiffL1.getAbsVer() < mvdTh)
          {
            return true;
          }
        }
      }
      else if (interDirNeighbours[ui] == 1)
      {
        if (mvFieldNeighbours[ui][0].refIdx == mvFieldNeighbours[mergeCandIndex][0].refIdx)
        {
          Mv mvDiff = mvFieldNeighbours[ui][0].mv - mvFieldNeighbours[mergeCandIndex][0].mv;

          if (mvDiff.getAbsHor() < mvdTh && mvDiff.getAbsVer() < mvdTh)
          {
            return true;
          }
        }
      }
      else if (interDirNeighbours[ui] == 2)
      {
        if (mvFieldNeighbours[ui][1].refIdx == mvFieldNeighbours[mergeCandIndex][1].refIdx)
        {
          Mv mvDiff = mvFieldNeighbours[ui][1].mv - mvFieldNeighbours[mergeCandIndex][1].mv;

          if (mvDiff.getAbsHor() < mvdTh && mvDiff.getAbsVer() < mvdTh)
          {
            return true;
          }
        }
      }
    }
  }

  return false;
}

void MergeCtx::initMrgCand(int mergeCandIndex)
{
  bcwIdx[mergeCandIndex]   = BCW_DEFAULT;
  LICFlags[mergeCandIndex] = false;
#if JVET_AE0159_FIBC   // TODO
  ibcFilterFlags[cnt] = false;
#endif
  interDirNeighbours[mergeCandIndex]             = 0;
  mvFieldNeighbours[mergeCandIndex][RPL0].refIdx = NOT_VALID;
  mvFieldNeighbours[mergeCandIndex][RPL1].refIdx = NOT_VALID;
  useAltHpelIf[mergeCandIndex]                   = false;
}

bool AffineMergeCtx::xCheckSimilarMotion(int mergeCandIndex, uint32_t mvdSimilarityThresh) const
{
  if (mvFieldNeighbours[mergeCandIndex][0][RPL0].refIdx < 0 && mvFieldNeighbours[mergeCandIndex][0][RPL1].refIdx < 0)
  {
    return true;
  }

  int mvdTh = std::max<int>(mvdSimilarityThresh, 1);

  for (uint32_t ui = 0; ui < mergeCandIndex; ui++)
  {
    if (interDirNeighbours[ui] == interDirNeighbours[mergeCandIndex])
    {
      if (interDirNeighbours[ui] == 3)
      {
        if (mvFieldNeighbours[ui][0][RPL0].refIdx == mvFieldNeighbours[mergeCandIndex][0][RPL0].refIdx &&
            mvFieldNeighbours[ui][0][RPL1].refIdx == mvFieldNeighbours[mergeCandIndex][0][RPL1].refIdx)
        {
          Mv mvDiff0L0 = mvFieldNeighbours[ui][0][RPL0].mv - mvFieldNeighbours[mergeCandIndex][0][RPL0].mv;
          Mv mvDiff0L1 = mvFieldNeighbours[ui][0][RPL1].mv - mvFieldNeighbours[mergeCandIndex][0][RPL1].mv;

          Mv mvDiff1L0 = mvFieldNeighbours[ui][1][RPL0].mv - mvFieldNeighbours[mergeCandIndex][1][RPL0].mv;
          Mv mvDiff1L1 = mvFieldNeighbours[ui][1][RPL1].mv - mvFieldNeighbours[mergeCandIndex][1][RPL1].mv;

          Mv mvDiff2L0 = mvFieldNeighbours[ui][2][RPL0].mv - mvFieldNeighbours[mergeCandIndex][2][RPL0].mv;
          Mv mvDiff2L1 = mvFieldNeighbours[ui][2][RPL1].mv - mvFieldNeighbours[mergeCandIndex][2][RPL1].mv;

          if (mvDiff0L0.getAbsHor() < mvdTh && mvDiff0L0.getAbsVer() < mvdTh && mvDiff0L1.getAbsHor() < mvdTh &&
              mvDiff0L1.getAbsVer() < mvdTh && mvDiff1L0.getAbsHor() < mvdTh && mvDiff1L0.getAbsVer() < mvdTh &&
              mvDiff1L1.getAbsHor() < mvdTh && mvDiff1L1.getAbsVer() < mvdTh && mvDiff2L0.getAbsHor() < mvdTh &&
              mvDiff2L0.getAbsVer() < mvdTh && mvDiff2L1.getAbsHor() < mvdTh && mvDiff2L1.getAbsVer() < mvdTh)
          {
            return true;
          }
        }
      }
      else if (interDirNeighbours[ui] == 1)
      {
        if (mvFieldNeighbours[ui][0][RPL0].refIdx == mvFieldNeighbours[mergeCandIndex][0][RPL0].refIdx)
        {
          Mv mvDiff0 = mvFieldNeighbours[ui][0][RPL0].mv - mvFieldNeighbours[mergeCandIndex][0][RPL0].mv;
          Mv mvDiff1 = mvFieldNeighbours[ui][1][RPL0].mv - mvFieldNeighbours[mergeCandIndex][1][RPL0].mv;
          Mv mvDiff2 = mvFieldNeighbours[ui][2][RPL0].mv - mvFieldNeighbours[mergeCandIndex][2][RPL0].mv;

          if (mvDiff0.getAbsHor() < mvdTh && mvDiff0.getAbsVer() < mvdTh && mvDiff1.getAbsHor() < mvdTh &&
              mvDiff1.getAbsVer() < mvdTh && mvDiff2.getAbsHor() < mvdTh && mvDiff2.getAbsVer() < mvdTh)
          {
            return true;
          }
        }
      }
      else if (interDirNeighbours[ui] == 2)
      {
        if (mvFieldNeighbours[ui][0][RPL1].refIdx == mvFieldNeighbours[mergeCandIndex][0][RPL1].refIdx)
        {
          Mv mvDiff0 = mvFieldNeighbours[ui][0][RPL1].mv - mvFieldNeighbours[mergeCandIndex][0][RPL1].mv;
          Mv mvDiff1 = mvFieldNeighbours[ui][1][RPL1].mv - mvFieldNeighbours[mergeCandIndex][1][RPL1].mv;
          Mv mvDiff2 = mvFieldNeighbours[ui][2][RPL1].mv - mvFieldNeighbours[mergeCandIndex][2][RPL1].mv;

          if (mvDiff0.getAbsHor() < mvdTh && mvDiff0.getAbsVer() < mvdTh && mvDiff1.getAbsHor() < mvdTh &&
              mvDiff1.getAbsVer() < mvdTh && mvDiff2.getAbsHor() < mvdTh && mvDiff2.getAbsVer() < mvdTh)
          {
            return true;
          }
        }
      }
    }
  }
  return false;
}

uint32_t InterPrediction::checkValidBvPU(const CodingUnit &pu, CompID compID, Mv mv, bool ignoreFracMv,
                                         InterpolationFilter::Filter filterIdx)
{
  return checkValidBv(pu, compID, (int)pu.blocks[compID].width, (int)pu.blocks[compID].height, mv, ignoreFracMv,
                      filterIdx);
}
uint32_t InterPrediction::checkValidBv(const CodingUnit &pu, CompID compID, int compWidth, int compHeight, Mv mv,
                                       bool ignoreFracMv, InterpolationFilter::Filter filterIdx, bool isFinalMC,
                                       bool checkAllRefValid)
{
  const int picWidth  = pu.cs->slice->m_pps->m_picWidthInLumaSamples;
  const int picHeight = pu.cs->slice->m_pps->m_picHeightInLumaSamples;
  const int lcuWidth  = pu.cs->slice->m_sps->m_maxCuWidth;

  const int szShiftHor = ::getComponentScaleX(compID, pu.chromaFormat);
  const int szShiftVer = ::getComponentScaleY(compID, pu.chromaFormat);
  const int lumaPosX   = pu.blocks[compID].x << szShiftHor;
  const int lumaPosY   = pu.blocks[compID].y << szShiftVer;
  const int lumaWidth  = compWidth << szShiftHor;
  const int lumaHeight = compHeight << szShiftVer;

  const int bvShiftHor = MV_FRACTIONAL_BITS_INTERNAL + szShiftHor;
  const int bvShiftVer = MV_FRACTIONAL_BITS_INTERNAL + szShiftVer;
  const int xFrac      = pu.cs->sps->m_ibcFracFlag ? (mv.hor & ((1 << bvShiftHor) - 1)) : 0;
  const int yFrac      = pu.cs->sps->m_ibcFracFlag ? (mv.ver & ((1 << bvShiftVer) - 1)) : 0;

  if ((!pu.cs->sps->m_ibcFracFlag || ignoreFracMv) || (xFrac == 0 && yFrac == 0))
  {
    int xLumaBv = (mv.hor >> bvShiftHor) << szShiftHor;
    int yLumaBv = (mv.ver >> bvShiftVer) << szShiftVer;

    return !searchBv(pu, lumaPosX, lumaPosY, lumaWidth, lumaHeight, picWidth, picHeight, xLumaBv, yLumaBv, lcuWidth, 1,
                     1, compID)
      ? IBC_BV_INVALID
      : (xFrac != 0 || yFrac != 0 ? IBC_INT_BV_VALID : IBC_BV_VALID);
  }

  int xLumaBv = (mv.hor >> bvShiftHor) << szShiftHor;
  int yLumaBv = (mv.ver >> bvShiftVer) << szShiftVer;

  int xFilterTap = xFrac == 0 ? 1 : (isLuma(compID) ? NTAPS_LUMA_IBC : NTAPS_CHROMA);
  int yFilterTap = yFrac == 0 ? 1 : (isLuma(compID) ? NTAPS_LUMA_IBC : NTAPS_CHROMA);

  if (isChroma(compID))
  {
    xFilterTap <<= (xFilterTap == 1 ? 0 : szShiftHor);
    yFilterTap <<= (yFilterTap == 1 ? 0 : szShiftVer);
  }

  if (searchBv(pu, lumaPosX, lumaPosY, lumaWidth, lumaHeight, picWidth, picHeight, xLumaBv, yLumaBv, lcuWidth,
               xFilterTap, yFilterTap, compID))
  {
    return IBC_BV_VALID;
  }
  else
  {
    if (isFinalMC)
    {
      return IBC_INT_BV_VALID;
    }
    if (checkAllRefValid)
    {
      return IBC_BV_INVALID;
    }
    return checkValidBv(pu, compID, compWidth, compHeight, mv, true);
  }
}

bool InterPrediction::searchBv(const CodingUnit &cu, int xPos, int yPos, int width, int height, int picWidth,
                               int picHeight, int xBv, int yBv, int ctuSize, int xFilterTap, int yFilterTap,
                               CompID compID)
{
  const int ctuSizeLog2 = floorLog2(ctuSize);

  const int szShiftHor = ::getComponentScaleX(compID, cu.chromaFormat);
  const int szShiftVer = ::getComponentScaleY(compID, cu.chromaFormat);
  int       refRightX  = xPos + xBv + width - 1 - szShiftHor;
  int       refLeftX   = xPos + xBv;
  int       refBottomY = yPos + yBv + height - 1 - szShiftVer;
  int       refTopY    = yPos + yBv;

  if (cu.cs->sps->m_ibcFracFlag)
  {
    if (xFilterTap > 1)
    {
      refLeftX -= (((xFilterTap - 1) >> 1) - szShiftHor);
      refRightX += (xFilterTap >> 1);
    }
    if (yFilterTap > 1)
    {
      refTopY -= (((yFilterTap - 1) >> 1) - szShiftVer);
      refBottomY += (yFilterTap >> 1);
    }
  }

  if (xPos < 0 || yPos < 0)
  {
    return false;
  }

  if (refLeftX < 0)
  {
    return false;
  }
  if (refRightX >= picWidth)
  {
    return false;
  }

  if (refTopY < 0)
  {
    return false;
  }
  if (refBottomY >= picHeight)
  {
    return false;
  }
  if (refRightX >= xPos && refBottomY >= yPos)
  {
    return false;
  }

  // Don't search the below CTU row
  if (refBottomY >> ctuSizeLog2 > yPos >> ctuSizeLog2)
  {
    return false;
  }

  if (cu.cs->pps->getNumTiles() > 1)
  {
    unsigned curTileIdx = cu.cs->pps->getTileIdx(Position(xPos, yPos));
    unsigned refTileIdx = cu.cs->pps->getTileIdx(Position(refLeftX, refTopY));
    if (curTileIdx != refTileIdx)
    {
      return false;
    }
    refTileIdx = cu.cs->pps->getTileIdx(Position(refLeftX, refBottomY));
    if (curTileIdx != refTileIdx)
    {
      return false;
    }
    refTileIdx = cu.cs->pps->getTileIdx(Position(refRightX, refTopY));
    if (curTileIdx != refTileIdx)
    {
      return false;
    }
    refTileIdx = cu.cs->pps->getTileIdx(Position(refRightX, refBottomY));
    if (curTileIdx != refTileIdx)
    {
      return false;
    }
  }

  if (256 == ctuSize)
  {
    if ((refTopY >> ctuSizeLog2) + 1 < (yPos >> ctuSizeLog2))
    {
      return false;
    }
    if (((refTopY >> ctuSizeLog2) == (yPos >> ctuSizeLog2)) && ((refRightX >> ctuSizeLog2) > (xPos >> ctuSizeLog2)))
    {
      return false;
    }
    if (((refTopY >> ctuSizeLog2) + 1 == (yPos >> ctuSizeLog2)) &&
        ((refLeftX >> ctuSizeLog2) + 1 < (xPos >> ctuSizeLog2)))
    {
      return false;
    }
  }
  else
  {
    if ((refTopY >> ctuSizeLog2) + 2 < (yPos >> ctuSizeLog2))
    {
      return false;
    }
    if (((refTopY >> ctuSizeLog2) == (yPos >> ctuSizeLog2)) && ((refRightX >> ctuSizeLog2) > (xPos >> ctuSizeLog2)))
    {
      return false;
    }
    if (((refTopY >> ctuSizeLog2) + 2 == (yPos >> ctuSizeLog2)) &&
        ((refLeftX >> ctuSizeLog2) + 2 < (xPos >> ctuSizeLog2)))
    {
      return false;
    }
  }

  // in the same CTU, or valid area from left CTU. Check if the reference block is already coded
  if (compID != COMP_Y)
  {
    refLeftX >>= szShiftHor;
    refRightX >>= szShiftHor;
    refTopY >>= szShiftVer;
    refBottomY >>= szShiftVer;
  }

  const Position refPosBR(refRightX, refBottomY);

  const ChannelType chType = toChannelType(compID);
  if (!cu.cs->isDecomp(refPosBR, chType))
  {
    return false;
  }
  // TODO (AW): if the bottom right is coded, top left has to be as well (?)
  // const Position refPosLT(refLeftX,  refTopY);
  // if (!cu.cs->isDecomp(refPosLT, chType))
  //{
  //  return false;
  //}
  return true;
}

Distortion InterPrediction::getDecoderSideDerivedMvCost(const Mv &mvStart, const Mv &mvCur, int searchRangeInFullPel,
                                                        int weight)
{
  int        searchRange = searchRangeInFullPel << MV_FRACTIONAL_BITS_INTERNAL;
  Mv         mvDist      = mvStart - mvCur;
  Distortion cost        = std::numeric_limits<Distortion>::max();
  if (mvDist.getAbsHor() <= searchRange && mvDist.getAbsVer() <= searchRange)
  {
    cost = (mvDist.getAbsHor() + mvDist.getAbsVer()) * weight;
    cost >>= MV_FRACTIONAL_BITS_DIFF;
  }

  return cost;
}

void InterPrediction::xBDMVRUpdateSquareSearchCostLog(Distortion *costLog, int bestDirect)
{
  CHECK(bestDirect < 0 || bestDirect > 7, "Error: Unknown bestDirect");

  int prevCenter      = (bestDirect + 4) & 0x7;
  costLog[prevCenter] = costLog[8];
  costLog[8]          = costLog[bestDirect];

  if (prevCenter & 0x1)
  {
    costLog[(prevCenter - 1 + 8) & 0x7] = costLog[(prevCenter - 2 + 8) & 0x7];
    costLog[(prevCenter + 1 + 8) & 0x7] = costLog[(prevCenter + 2 + 8) & 0x7];
    costLog[(prevCenter - 2 + 8) & 0x7] = costLog[(prevCenter - 3 + 8) & 0x7];
    costLog[(prevCenter + 2 + 8) & 0x7] = costLog[(prevCenter + 3 + 8) & 0x7];
    for (int offset = 3; offset < 6; ++offset)
    {
      costLog[(prevCenter + offset + 8) & 0x7] = std::numeric_limits<Distortion>::max();
    }
  }
  else
  {
    costLog[(prevCenter - 1 + 8) & 0x7] = costLog[(prevCenter - 3 + 8) & 0x7];
    costLog[(prevCenter + 1 + 8) & 0x7] = costLog[(prevCenter + 3 + 8) & 0x7];
    for (int offset = 2; offset < 7; ++offset)
    {
      costLog[(prevCenter + offset + 8) & 0x7] = std::numeric_limits<Distortion>::max();
    }
  }
}

void InterPrediction::xLocalIlluComp(const CodingUnit &cu, const CompID compID, const Picture *refPic, const Mv *mv,
                                     const bool biPred, PelBuf &dstBuf, const RefPicList refPicList)
{
  Pel  *refLeftTemplate   = m_pcLICRefLeftTemplate[refPicList][compID];
  Pel  *refAboveTemplate  = m_pcLICRefAboveTemplate[refPicList][compID];
  Pel  *recLeftTemplate   = m_pcLICRecLeftTemplate[compID];
  Pel  *recAboveTemplate  = m_pcLICRecAboveTemplate[compID];
  bool *templateAvailable = m_templateAvailable[compID];

  CHECK(m_subPuMC && m_storeBeforeLIC, "m_subPuMC && m_storeBeforeLIC");

  // fetch templates
  if (!m_subPuMC)
  {
    CHECK(biPred && (refPicList == RPL1) && !m_fillLicTpl[compID], "unexpected");
    if (!cu.affine)
    {
      CHECK((!biPred || refPicList == RPL0) && (templateAvailable[0] || templateAvailable[1]),
            "templateAvailable already set");
      xGetSublkTemplate(cu, compID, *refPic, *mv, cu.blocks[compID].width, cu.blocks[compID].height, 0, 0,
                        templateAvailable, refLeftTemplate, refAboveTemplate, recLeftTemplate, recAboveTemplate);
      if (!biPred || refPicList == RPL0)
      {
        m_fillLicTpl[compID] = true;
      }
    }
  }

  // bi-pred LIC is handled in xLicCompAdj()
  if (biPred)
  {
    return;
  }

  // derive LIC parameters (scale and offset)
  if (!m_subPuMC)
  {
    int &scale  = cu.licScaleAndOffset.scale[refPicList][compID];
    int &offset = cu.licScaleAndOffset.offset[refPicList][compID];

    xGetLICParamGeneral(cu, compID, templateAvailable, refLeftTemplate, refAboveTemplate, recLeftTemplate,
                        recAboveTemplate, scale, offset);
    templateAvailable[0] = templateAvailable[1] = false;
  }

  // apply LIC
  const int     scale  = cu.licScaleAndOffset.scale[refPicList][compID];
  const int     offset = cu.licScaleAndOffset.offset[refPicList][compID];
  const ClpRng &clpRng = cu.cs->slice->clpRng(compID);
  dstBuf.linearTransform(scale, m_LICShift, offset, true, clpRng);
}

void InterPrediction::xGetSublkTemplate(const CodingUnit &cu, const CompID compID, const Picture &refPic, const Mv &mv,
                                        const int sublkWidth, const int sublkHeight, const int posW, const int posH,
                                        bool *templateAvailable, Pel *refLeftTemplate, Pel *refAboveTemplate,
                                        Pel *recLeftTemplate, Pel *recAboveTemplate)
{
  const int bitDepth  = cu.cs->sps->m_bitDepths[toChannelType(COMP_Y)];
  const int precShift = std::max(0, bitDepth - 12);

  const Picture          &currPic = *cu.cs->picture;
  const CodingUnit *const cuAbove = cu.cs->getCU(cu.blocks[compID].pos().offset(0, -1), toChannelType(compID));
  const CodingUnit *const cuLeft  = cu.cs->getCU(cu.blocks[compID].pos().offset(-1, 0), toChannelType(compID));
  const CPelBuf           recBuf  = cuAbove || cuLeft ? currPic.getRecoBuf(cu.cs->picture->blocks[compID]) : CPelBuf();
  const CPelBuf           refBuf  = cuAbove || cuLeft ? refPic.getRecoBuf(refPic.blocks[compID]) : CPelBuf();

  std::vector<Pel> &invLUT  = m_reshape->m_invLUT;
  const bool        bInvLUT = isLuma(compID) && cu.cs->picHeader->m_lmcsEnabledFlag && m_reshape->m_ctuFlag;

  // above
  if (cuAbove && posH == 0)
  {
    xGetPredBlkTpl<true>(cu, compID, refBuf, mv, posW, posH, sublkWidth, refAboveTemplate);
    templateAvailable[0] = true;
    if (precShift)
    {
      for (int k = posW; k < posW + sublkWidth; k++)
      {
        int refVal = refAboveTemplate[k];
        refVal >>= precShift;
        refAboveTemplate[k] = refVal;
      }
    }
    if (!m_fillLicTpl[compID])
    {
      const Pel *rec = recBuf.bufAt(cu.blocks[compID].pos().offset(0, -1));
      if (bInvLUT)
      {
        for (int k = posW; k < posW + sublkWidth; k++)
        {
          int recVal = rec[k];
          recVal     = invLUT[recVal];
          recVal >>= precShift;
          recAboveTemplate[k] = recVal;
        }
      }
      else
      {
        for (int k = posW; k < posW + sublkWidth; k++)
        {
          int recVal = rec[k];
          recVal >>= precShift;
          recAboveTemplate[k] = recVal;
        }
      }
    }
  }

  // left
  if (cuLeft && posW == 0)
  {
    xGetPredBlkTpl<false>(cu, compID, refBuf, mv, posW, posH, sublkHeight, refLeftTemplate);
    templateAvailable[1] = true;
    if (precShift)
    {
      for (int k = posH; k < posH + sublkHeight; k++)
      {
        int refVal = refLeftTemplate[k];
        refVal >>= precShift;
        refLeftTemplate[k] = refVal;
      }
    }
    if (!m_fillLicTpl[compID])
    {
      const Pel *rec = recBuf.bufAt(cu.blocks[compID].pos().offset(-1, 0));
      if (bInvLUT)
      {
        for (int k = posH; k < posH + sublkHeight; k++)
        {
          int recVal = rec[recBuf.stride * k];
          recVal     = invLUT[recVal];
          recVal >>= precShift;
          recLeftTemplate[k] = recVal;
        }
      }
      else
      {
        for (int k = posH; k < posH + sublkHeight; k++)
        {
          int recVal = rec[recBuf.stride * k];
          recVal >>= precShift;
          recLeftTemplate[k] = recVal;
        }
      }
    }
  }
}

void InterPrediction::xGetLICParamGeneral(const CodingUnit &cu, const CompID compID, const bool *templateAvailable,
                                          Pel *refLeftTemplate, Pel *refAboveTemplate, Pel *recLeftTemplate,
                                          Pel *recAboveTemplate, int &scale, int &offset)
{
  const int cuWidth  = cu.blocks[compID].width;
  const int cuHeight = cu.blocks[compID].height;

  const int bitDepth     = cu.cs->sps->m_bitDepths[toChannelType(COMP_Y)];
  const int precShift    = std::max(0, bitDepth - 12);
  const int maxNumMinus1 = 30 - 2 * std::min(bitDepth, 12) - 1;
  const int minDimBit    = floorLog2(std::min(cuHeight, cuWidth));
  const int minDim       = 1 << minDimBit;
  int       minStepBit   = minDim > 8 ? 1 : 0;
  while (minDimBit > minStepBit + maxNumMinus1)
  {
    minStepBit++;
  }   // make sure log2(2*minDim/tmpStep) + 2*min(bitDepth,12) <= 30
  const int numSteps = minDim >> minStepBit;
  const int dimShift = minDimBit - minStepBit;

  // LICMultApprox[k] = ((1 << 15) + (k >> 1)) / k;
  static constexpr int LICMultApprox[] = {
    0,    32768, 16384, 10923, 8192, 6554, 5461, 4681, 4096, 3641, 3277, 2979, 2731, 2521, 2341, 2185,
    2048, 1928,  1820,  1725,  1638, 1560, 1489, 1425, 1365, 1311, 1260, 1214, 1170, 1130, 1092, 1057,
    1024, 993,   964,   936,   910,  886,  862,  840,  819,  799,  780,  762,  745,  728,  712,  697,
    683,  669,   655,   643,   630,  618,  607,  596,  585,  575,  565,  555,  546,  537,  529,  520,
  };

  //----- get correlation data -----
  int x = 0, y = 0, xx = 0, xy = 0, cntShift = 0;

  // above
  if (templateAvailable[0])
  {
    for (int k = 0; k < numSteps; k++)
    {
      CHECK(((k * cuWidth) >> dimShift) >= cuWidth, "Out of range");

      int refVal = refAboveTemplate[((k * cuWidth) >> dimShift)];
      int recVal = recAboveTemplate[((k * cuWidth) >> dimShift)];

      x += refVal;
      y += recVal;
      xx += refVal * refVal;
      xy += refVal * recVal;
    }

    cntShift = dimShift;
  }

  // left
  if (templateAvailable[1])
  {
    for (int k = 0; k < numSteps; k++)
    {
      CHECK(((k * cuHeight) >> dimShift) >= cuHeight, "Out of range");

      int refVal = refLeftTemplate[((k * cuHeight) >> dimShift)];
      int recVal = recLeftTemplate[((k * cuHeight) >> dimShift)];

      x += refVal;
      y += recVal;
      xx += refVal * refVal;
      xy += refVal * recVal;
    }

    cntShift += (cntShift ? 1 : dimShift);
  }

  //----- determine scale and offset -----
  if (cntShift == 0)
  {
    scale  = (1 << m_LICShift);
    offset = 0;
    return;
  }

  const int cropShift    = std::max(0, bitDepth - precShift + cntShift - 15);
  const int xzOffset     = (xx >> m_LICRegShift);
  const int sumX         = x << precShift;
  const int sumY         = y << precShift;
  const int sumXX        = ((xx + xzOffset) >> (cropShift << 1)) << cntShift;
  const int sumXY        = ((xy + xzOffset) >> (cropShift << 1)) << cntShift;
  const int sumXsumX     = (x >> cropShift) * (x >> cropShift);
  const int sumXsumY     = (x >> cropShift) * (y >> cropShift);
  int       a1           = sumXY - sumXsumY;
  int       a2           = sumXX - sumXsumX;
  int       scaleShiftA2 = getMSB(abs(a2)) - 6;
  int       scaleShiftA1 = scaleShiftA2 - m_LICShiftDiff;
  scaleShiftA2           = std::max(0, scaleShiftA2);
  scaleShiftA1           = std::max(0, scaleShiftA1);
  const int scaleShiftA  = scaleShiftA2 + 15 - m_LICShift - scaleShiftA1;
  a1                     = a1 >> scaleShiftA1;
  a2                     = Clip3(0, 63, a2 >> scaleShiftA2);
  scale                  = int((int64_t(a1) * int64_t(LICMultApprox[a2])) >> scaleShiftA);
  scale                  = Clip3(0, 1 << (m_LICShift + 2), scale);
  const int maxOffset    = (1 << (bitDepth - 1)) - 1;
  const int minOffset    = -1 - maxOffset;
  offset                 = (sumY - ((scale * sumX) >> m_LICShift) + ((1 << (cntShift)) >> 1)) >> cntShift;
  offset                 = Clip3(minOffset, maxOffset, offset);
}

template<bool TrueA_FalseL> void InterPrediction::xGetPredBlkTpl(const CodingUnit &cu, const CompID compID,
                                                                 const CPelBuf &refBuf, const Mv &mv, const int posW,
                                                                 const int posH, const int tplSize, Pel *predBlkTpl)
{
  const int lumaShift = 2 + MV_FRACTIONAL_BITS_DIFF;
  const int horShift  = (lumaShift + ::getComponentScaleX(compID, cu.chromaFormat));
  const int verShift  = (lumaShift + ::getComponentScaleY(compID, cu.chromaFormat));

  const int xInt  = mv.getHor() >> horShift;
  const int yInt  = mv.getVer() >> verShift;
  const int xFrac = mv.getHor() & ((1 << horShift) - 1);
  const int yFrac = mv.getVer() & ((1 << verShift) - 1);

  const Pel *ref;
  Pel       *dst;
  ptrdiff_t  refStride, dstStride;
  int        bw, bh;
  if (TrueA_FalseL)
  {
    ref       = refBuf.bufAt(cu.blocks[compID].pos().offset(xInt + posW, yInt + posH - 1));
    dst       = predBlkTpl + posW;
    refStride = refBuf.stride;
    dstStride = tplSize;
    bw        = tplSize;
    bh        = 1;
  }
  else
  {
    ref       = refBuf.bufAt(cu.blocks[compID].pos().offset(xInt + posW - 1, yInt + posH));
    dst       = predBlkTpl + posH;
    refStride = refBuf.stride;
    dstStride = 1;
    bw        = 1;
    bh        = tplSize;
  }

  if (yFrac == 0)
  {
    m_if->filterHor(compID, (Pel *)ref, refStride, dst, dstStride, bw, bh, xFrac, true, cu.slice->clpRng(compID),
                    InterpolationFilter::Filter::DEFAULT);
  }
  else if (xFrac == 0)
  {
    m_if->filterVer(compID, (Pel *)ref, refStride, dst, dstStride, bw, bh, yFrac, true, true, cu.slice->clpRng(compID),
                    InterpolationFilter::Filter::DEFAULT);
  }
  else
  {
    const int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;
    PelBuf    tmpBuf      = PelBuf(m_filteredBlockTmp[0][compID], Size(bw, bh + vFilterSize - 1));

    m_if->filterHor(compID, (Pel *)ref - ((vFilterSize >> 1) - 1) * refStride, refStride, tmpBuf.buf, tmpBuf.stride, bw,
                    bh + vFilterSize - 1, xFrac, false, cu.slice->clpRng(compID), InterpolationFilter::Filter::DEFAULT);
    JVET_J0090_SET_CACHE_ENABLE(false);
    m_if->filterVer(compID, tmpBuf.buf + ((vFilterSize >> 1) - 1) * tmpBuf.stride, tmpBuf.stride, dst, dstStride, bw,
                    bh, yFrac, false, true, cu.slice->clpRng(compID), InterpolationFilter::Filter::DEFAULT);
    JVET_J0090_SET_CACHE_ENABLE(true);
  }
}

void InterPrediction::xLicRemHighFreq(const CodingUnit &cu, const CompID compID, const int licIdx)
{
  const int     width  = cu.blocks[compID].width;
  const int     height = cu.blocks[compID].height;
  const ClpRng &clpRng = cu.cs->slice->clpRng(compID);

  const int    refRefList     = 1 - (licIdx % 2);
  const int8_t bcwWeight      = getBcwWeight(cu.bcwIdx, 1 - refRefList);
  const int8_t bcwWeightOther = g_BcwWeightBase - bcwWeight;

  const int normalizer = ((1 << 16) + (bcwWeight > 0 ? (bcwWeight >> 1) : -(bcwWeight >> 1))) / bcwWeight;
  const int weight0    = normalizer << g_BcwLog2WeightBase;
  const int weight1    = bcwWeightOther * normalizer;
  const int offset     = 1 << 15;

  if (m_templateAvailable[compID][0])   // above
  {
    if (licIdx > 0)
    {
      if ((width & 3) == 0)
      {
        g_pelBufOP.licRemoveWeightHighFreq4(
          &(m_pcLICRecAboveTemplate[compID][0]), &(m_curLICRefAboveTemplate[refRefList][compID][0]),
          &(m_curLICRecAboveTemplate[compID][0]), width, weight0, weight1, offset, clpRng);
      }
      else if ((width & 1) == 0)
      {
        g_pelBufOP.licRemoveWeightHighFreq2(
          &(m_pcLICRecAboveTemplate[compID][0]), &(m_curLICRefAboveTemplate[refRefList][compID][0]),
          &(m_curLICRecAboveTemplate[compID][0]), width, weight0, weight1, offset, clpRng);
      }
      else
      {
        THROW("Unsupported size!");
      }
    }
    else
    {
      memcpy(m_curLICRecAboveTemplate[compID], m_pcLICRecAboveTemplate[compID], width * sizeof(Pel));
    }
  }
  if (m_templateAvailable[compID][1])   // left
  {
    if (licIdx > 0)
    {
      if ((height & 3) == 0)
      {
        g_pelBufOP.licRemoveWeightHighFreq4(
          &(m_pcLICRecLeftTemplate[compID][0]), &(m_curLICRefLeftTemplate[refRefList][compID][0]),
          &(m_curLICRecLeftTemplate[compID][0]), height, weight0, weight1, offset, clpRng);
      }
      else if ((height & 1) == 0)
      {
        g_pelBufOP.licRemoveWeightHighFreq2(
          &(m_pcLICRecLeftTemplate[compID][0]), &(m_curLICRefLeftTemplate[refRefList][compID][0]),
          &(m_curLICRecLeftTemplate[compID][0]), height, weight0, weight1, offset, clpRng);
      }
      else
      {
        THROW("Unsupported size!");
      }
    }
    else
    {
      memcpy(m_curLICRecLeftTemplate[compID], m_pcLICRecLeftTemplate[compID], height * sizeof(Pel));
    }
  }
}

void InterPrediction::xLicCompAdj(CodingUnit &cu, const bool lumaOnly, const bool chromaOnly)
{
  const int refIdx0 = cu.refIdx[RPL0];
  const int refIdx1 = cu.refIdx[RPL1];
  if (cu.licFlag && !cu.ciipFlag && refIdx0 >= 0 && refIdx1 >= 0)
  {
    bool refIsScaled = (refIdx0 < 0 ? false : cu.slice->getRefPic(RPL0, refIdx0)->isRefScaled(cu.cs->pps)) ||
      (refIdx1 < 0 ? false : cu.slice->getRefPic(RPL1, refIdx1)->isRefScaled(cu.cs->pps));
    if (!refIsScaled)
    {
      CHECK(!cu.cs->sps->m_biLicEnabledFlag, "bi-LIC not allowed");
      if (!m_subPuMC)   // OBMC
      {
        xLicBiDerive(cu, lumaOnly, chromaOnly);
      }
      xLicBiApply(cu, lumaOnly, chromaOnly);
    }
  }
}

void InterPrediction::xLicBiApply(const CodingUnit &cu, const bool lumaOnly, const bool chromaOnly)
{
  const Pel biOffset = -IF_INTERNAL_OFFS;
  for (int refList = 0; refList < NUM_RPL01; refList++)
  {
    for (int compID = 0; compID < MAX_NUM_COMP; compID++)
    {
      if (isLuma(CompID(compID)) && chromaOnly)
      {
        continue;
      }
      if (isChroma(CompID(compID)) && lumaOnly)
      {
        continue;
      }

      const ClpRng &clpRng  = cu.slice->clpRng(CompID(compID));
      const int     biShift = IF_INTERNAL_PREC - clpRng.bd;

      PelBuf buf(m_acYuvPred[refList][compID], cu.block(CompID(compID)));
      buf.toLast(clpRng);
      buf.linearTransform(cu.licScaleAndOffset.scale[refList][compID], m_LICShift,
                          cu.licScaleAndOffset.offset[refList][compID], true, clpRng);
      buf.linearTransform(1, -biShift, biOffset, false, clpRng);
    }
  }
}

void InterPrediction::xLicBiDerive(CodingUnit &cu, const bool lumaOnly, const bool chromaOnly)
{
  for (uint32_t licIdx = 0; licIdx < NUM_LIC_ITERATION; licIdx++)
  {
    int licRefList = (licIdx % 2);

    for (int compID = 0; compID < MAX_NUM_COMP; compID++)
    {
      if (isLuma(CompID(compID)) && chromaOnly)
      {
        CHECK(m_templateAvailable[compID][0] || m_templateAvailable[compID][1],
              " there should be no luma template samples");
        continue;
      }
      if (isChroma(CompID(compID)) && lumaOnly)
      {
        CHECK(m_templateAvailable[compID][0] || m_templateAvailable[compID][1],
              " there should be no chroma template samples");
        continue;
      }

      xLicRemHighFreq(cu, CompID(compID), licIdx);
      xGetLICParamGeneral(cu, CompID(compID), m_templateAvailable[compID], m_pcLICRefLeftTemplate[licRefList][compID],
                          m_pcLICRefAboveTemplate[licRefList][compID], m_curLICRecLeftTemplate[compID],
                          m_curLICRecAboveTemplate[compID], cu.licScaleAndOffset.scale[licRefList][compID],
                          cu.licScaleAndOffset.offset[licRefList][compID]);

      const ClpRng &clpRng = cu.slice->clpRng(CompID(compID));
      if (licIdx < (NUM_LIC_ITERATION - 1))
      {
        if (m_templateAvailable[compID][0])
        {
          const int cWidth = cu.blocks[compID].width;
          PelBuf    aboveTemplate(m_pcLICRefAboveTemplate[licRefList][compID], Size(cWidth, 1));
          PelBuf    curAboveTemplate(m_curLICRefAboveTemplate[licRefList][compID], Size(cWidth, 1));
          curAboveTemplate.copyFrom(aboveTemplate);
          curAboveTemplate.linearTransform(cu.licScaleAndOffset.scale[licRefList][compID], m_LICShift,
                                           cu.licScaleAndOffset.offset[licRefList][compID], true, clpRng);
        }
        if (m_templateAvailable[compID][1])
        {
          const int cHeight = cu.blocks[compID].height;
          PelBuf    leftTemplate(m_pcLICRefLeftTemplate[licRefList][compID], Size(cHeight, 1));
          PelBuf    curLeftTemplate(m_curLICRefLeftTemplate[licRefList][compID], Size(cHeight, 1));
          curLeftTemplate.copyFrom(leftTemplate);
          curLeftTemplate.linearTransform(cu.licScaleAndOffset.scale[licRefList][compID], m_LICShift,
                                          cu.licScaleAndOffset.offset[licRefList][compID], true, clpRng);
        }
      }
    }
  }
  std::fill_n(&m_templateAvailable[0][0], 2 * MAX_NUM_COMP, false);
}

void InterPrediction::xLicCopyPredBeforeLic(CodingUnit &cu, PelUnitBuf &pcYuvPred, bool luma, bool chroma,
                                            PelUnitBuf *yuvPredTmp, bool *isOOB)
{
  if (!m_storeBeforeLIC)
  {
    return;
  }

  const Slice &slice      = *cu.cs->slice;
  const bool   lumaOnly   = luma && !chroma;
  const bool   chromaOnly = !luma && chroma;

  CHECK(yuvPredTmp, "inlogic");

  UnitArea localUnitArea(cu.chromaFormat, Area(0, 0, cu.lumaSize().width, cu.lumaSize().height));
  if (cu.interDir == 3)
  {
    CHECK(cu.refIdx[0] < 0 || cu.refIdx[1] < 0, "invaid refIdx values for bi-pred");
    PelUnitBuf predBeforeLICBuffer0 = m_acPredBeforeLICBuffer[RPL0].getBuf(localUnitArea);
    PelUnitBuf predBeforeLICBuffer1 = m_acPredBeforeLICBuffer[RPL1].getBuf(localUnitArea);

    const int chromaStride = lumaOnly ? 0 : pcYuvPred.Cb().width;
    if (!cu.mergeFlag && cu.bcwIdx != BCW_DEFAULT)
    {
      m_predictionBeforeLIC.addWeightedAvg(predBeforeLICBuffer0, predBeforeLICBuffer1, slice.m_clpRngs, cu.bcwIdx,
                                           chromaOnly, lumaOnly, m_mcMask, pcYuvPred.Y().width, m_mcMaskChroma,
                                           chromaStride, isOOB);
    }
    else
    {
      m_predictionBeforeLIC.addAvg(predBeforeLICBuffer0, predBeforeLICBuffer1, slice.m_clpRngs, chromaOnly, lumaOnly,
                                   m_mcMask, pcYuvPred.Y().width, m_mcMaskChroma, chromaStride, isOOB);
    }
  }
  else if (cu.interDir == 1)
  {
    CHECK(cu.refIdx[0] < 0, "invalid refIdx value for L0 uni-pred")
    PelUnitBuf predBeforeLICBuffer = m_acPredBeforeLICBuffer[RPL0].getBuf(localUnitArea);
    m_predictionBeforeLIC.copyFrom(predBeforeLICBuffer, luma, chroma);
  }
  else if (cu.interDir == 2)
  {
    CHECK(cu.refIdx[1] < 0, "invalid refIdx value for L1 uni-pred");
    PelUnitBuf predBeforeLICBuffer = m_acPredBeforeLICBuffer[RPL1].getBuf(localUnitArea);
    m_predictionBeforeLIC.copyFrom(predBeforeLICBuffer, luma, chroma);
  }
}

void InterPrediction::xLicPredBlk(const CodingUnit &cu, const CompID compID, const Picture *refPic, const Mv *mv,
                                  const bool bi, PelBuf dstBuf, const RefPicList refPicList)
{
  if (m_storeBeforeLIC)
  {
    UnitArea   localUnitArea(cu.chromaFormat, Area(0, 0, cu.lumaSize().width, cu.lumaSize().height));
    PelUnitBuf predBeforeLICBuffer = m_acPredBeforeLICBuffer[refPicList].getBuf(localUnitArea);
    predBeforeLICBuffer.bufs[compID].copyFrom(dstBuf);
  }

  if (!cu.licFlag)
  {
    return;
  }
  if (cu.ciipFlag)
  {
    return;
  }
  if (!m_encMotionEstimation && !PU::checkRprLicCondition(cu))
  {
    return;
  }
  if (m_encMotionEstimation && refPic->isRefScaled(cu.cs->pps))
  {
    return;
  }

  CHECK(cu.geoFlag, "Geometric mode is not used with LIC");
  CHECK(CU::isIBC(cu), "IBC mode is not used with LIC");
  CHECK(!cu.cs->sps->m_biLicEnabledFlag && (cu.interDir == 3 || bi), "Bi-prediction is not used with LIC");

  xLocalIlluComp(cu, compID, refPic, mv, bi, dstBuf, refPicList);
}

void InterPrediction::mcFramePad(Picture *pcCurPic, Slice &slice)
{
  const Size     blkSizeBuff = Size(slice.m_sps->m_ctuSize, slice.m_sps->m_ctuSize);
  const Area     blkAreaBuff = Area(Position(), blkSizeBuff);
  const UnitArea blkUnitAreaBuff(slice.m_sps->m_chromaFormatIdc, blkAreaBuff);

  const Size     blkSizeCurBuff = Size(4, 4);
  const Area     blkAreaCurBuff = Area(Position(), blkSizeCurBuff);
  const UnitArea blkUnitCurAreaBuff(slice.m_sps->m_chromaFormatIdc, blkAreaCurBuff);

  const Size     blkSizeConBuff = Size(MC_PAD_SIZE, MC_PAD_SIZE);
  const Area     blkAreaConBuff = Area(Position(), blkSizeConBuff);
  const UnitArea blkUnitConAreaBuff(slice.m_sps->m_chromaFormatIdc, blkAreaConBuff);

  PelStorage *pPadYUVContainerDyn = new PelStorage;
  pPadYUVContainerDyn->create(blkUnitConAreaBuff);
  PelStorage *pCurBuffYUV = new PelStorage;
  pCurBuffYUV->create(blkUnitCurAreaBuff);

  CodingUnit blkDataTmp(blkUnitAreaBuff);
  blkDataTmp.cs      = pcCurPic->m_cs;
  blkDataTmp.bcwIdx  = BCW_DEFAULT;
  blkDataTmp.licFlag = false;
  blkDataTmp.affine  = false;
  blkDataTmp.geoFlag = false;
  blkDataTmp.imv     = IMV_OFF;
  blkDataTmp.slice   = &slice;
  blkDataTmp.cs      = pcCurPic->m_cs;

  // four directions MC padding
  mcFramePadTop(pcCurPic, slice, &blkDataTmp, pPadYUVContainerDyn, blkUnitAreaBuff, pCurBuffYUV);
  mcFramePadBottom(pcCurPic, slice, &blkDataTmp, pPadYUVContainerDyn, blkUnitAreaBuff, pCurBuffYUV);
  mcFramePadLeft(pcCurPic, slice, &blkDataTmp, pPadYUVContainerDyn, blkUnitAreaBuff, pCurBuffYUV);
  mcFramePadRight(pcCurPic, slice, &blkDataTmp, pPadYUVContainerDyn, blkUnitAreaBuff, pCurBuffYUV);

  // repetitive padding for the extend padding area

  pcCurPic->extendMcPaddedBorder(slice.m_sps->m_ctuSize);

  pPadYUVContainerDyn->destroy();
  delete pPadYUVContainerDyn;
  pCurBuffYUV->destroy();
  delete pCurBuffYUV;
}

template<boundaryDirection T>
void InterPrediction::mcFramePadOneSide(Picture *pcCurPic, Slice &slice, CodingUnit *blkDataTmp,
                                        PelStorage *pPadYUVContainerDyn, const UnitArea blkUnitAreaBuff,
                                        PelStorage *pCurBuffYUV)
{
  static constexpr bool isBotTop       = (T == BD_BOTTOM) || (T == BD_TOP);
  const int             ctuSize        = slice.m_sps->m_ctuSize;
  const int             iWidthFrm      = slice.m_pps->m_picWidthInLumaSamples;
  const int             iHeightFrm     = slice.m_pps->m_picHeightInLumaSamples;
  const int             numCtuInWidth  = iWidthFrm / ctuSize + (iWidthFrm % ctuSize != 0);
  const int             numCtuInHeight = iHeightFrm / ctuSize + (iHeightFrm % ctuSize != 0);
  const int             xBlkBoundIdx = (iWidthFrm % ctuSize) == 0 ? (ctuSize / 4 - 1) : ((iWidthFrm % ctuSize) / 4) - 1;
  const int yBlkBoundIdx = (iHeightFrm % ctuSize) == 0 ? (ctuSize / 4 - 1) : ((iHeightFrm % ctuSize) / 4) - 1;
  const int maxCtuIdx    = isBotTop ? numCtuInWidth : numCtuInHeight;

  int maxCh = pcCurPic->chromaFormat == ChromaFormat::_400 ? 0 : 2;

  for (int ctuIdx = 0; ctuIdx < maxCtuIdx; ctuIdx++)
  {
    Position ctuPos;
    if constexpr (T == BD_TOP)
    {
      ctuPos = ctuPos.offset(ctuSize * ctuIdx, 0);
    }
    else if constexpr (T == BD_BOTTOM)
    {
      ctuPos = ctuPos.offset(ctuSize * ctuIdx, ctuSize * (numCtuInHeight - 1));
    }
    else if constexpr (T == BD_LEFT)
    {
      ctuPos = ctuPos.offset(0, ctuSize * ctuIdx);
    }
    else
    {
      ctuPos = ctuPos.offset(ctuSize * (numCtuInWidth - 1), ctuSize * ctuIdx);
    }

    UnitArea uarea;
    if constexpr (T == BD_TOP)
    {
      uarea = UnitArea(pcCurPic->chromaFormat, Area(ctuPos.offset(0, -ctuSize), Size(ctuSize, ctuSize)));
    }
    else if constexpr (T == BD_BOTTOM)
    {
      uarea = UnitArea(pcCurPic->chromaFormat, Area(ctuPos.offset(0, (yBlkBoundIdx + 1) * 4), Size(ctuSize, ctuSize)));
    }
    else if constexpr (T == BD_LEFT)
    {
      uarea = UnitArea(pcCurPic->chromaFormat, Area(ctuPos.offset(-ctuSize, 0), Size(ctuSize, ctuSize)));
    }
    else
    {
      uarea = UnitArea(pcCurPic->chromaFormat, Area(ctuPos.offset((xBlkBoundIdx + 1) * 4, 0), Size(ctuSize, ctuSize)));
    }
    PelUnitBuf pPadBuffYUV = pcCurPic->getRecoBuf().subBuf(uarea);

    int maxIdxSubBlkPlus1;
    if (isBotTop)
    {
      maxIdxSubBlkPlus1 = (ctuIdx == (numCtuInWidth - 1)) ? (xBlkBoundIdx + 1) : (ctuSize / 4);
    }
    else
    {
      maxIdxSubBlkPlus1 = (ctuIdx == (numCtuInHeight - 1)) ? (yBlkBoundIdx + 1) : (ctuSize / 4);
    }

    // MC
    for (int subBlkIdx = 0; subBlkIdx < maxIdxSubBlkPlus1; subBlkIdx++)
    {
      blkDataTmp->bcwIdx = BCW_DEFAULT;
      Position subBlkPos = ctuPos;
      if (isBotTop)
      {
        subBlkPos = subBlkPos.offset(subBlkIdx * 4, 0);
      }
      else
      {
        subBlkPos = subBlkPos.offset(0, subBlkIdx * 4);
      }
      Position subBlkMvPos = subBlkPos;
      if constexpr (T == BD_BOTTOM)
      {
        subBlkMvPos = subBlkMvPos.offset(0, yBlkBoundIdx * 4);
      }
      else if constexpr (T == BD_RIGHT)
      {
        subBlkMvPos = subBlkMvPos.offset(xBlkBoundIdx * 4, 0);
      }

      short reflistIdx[2] = { -1, -1 };
      Mv    subBlkMv[2];

      if (pcCurPic->m_cs->getMotionInfo(subBlkMvPos).isInter && !pcCurPic->m_cs->getMotionInfo(subBlkMvPos).isIBCmot)
      {
        reflistIdx[RPL0] = pcCurPic->m_cs->getMotionInfo(subBlkMvPos).refIdx[RPL0];
        reflistIdx[RPL1] = pcCurPic->m_cs->getMotionInfo(subBlkMvPos).refIdx[RPL1];
        subBlkMv[RPL0]   = pcCurPic->m_cs->getMotionInfo(subBlkMvPos).mv[RPL0];
        subBlkMv[RPL1]   = pcCurPic->m_cs->getMotionInfo(subBlkMvPos).mv[RPL1];
      }
      int useList = -1;
      if (reflistIdx[0] >= 0 && reflistIdx[1] >= 0)
      {
        if constexpr (T == BD_TOP)
        {
          useList = (subBlkMv[0].getVer() > subBlkMv[1].getVer()) ? 0 : 1;
        }
        else if constexpr (T == BD_BOTTOM)
        {
          useList = (subBlkMv[0].getVer() <= subBlkMv[1].getVer()) ? 0 : 1;
        }
        else if constexpr (T == BD_LEFT)
        {
          useList = (subBlkMv[0].getHor() > subBlkMv[1].getHor()) ? 0 : 1;
        }
        else
        {
          useList = (subBlkMv[0].getHor() <= subBlkMv[1].getHor()) ? 0 : 1;
        }
      }
      else
      {
        useList = (reflistIdx[0] >= 0) ? 0 : 1;
      }
      reflistIdx[1 - useList] = -1;
      int validPadSize        = 0;
      if (reflistIdx[useList] >= 0)
      {
        int      iMVBitShift = MV_FRACTIONAL_BITS_INTERNAL;
        MvField  tempBiMvFieldAddOffset[2];
        Mv       mvAddOffset;
        Position subBlkMCPos = subBlkPos, mcBlksize;

        if constexpr (T == BD_TOP)
        {
          mvAddOffset.set(0, -ctuSize << iMVBitShift);
          validPadSize = (((subBlkMv[useList].getVer() >> iMVBitShift) + 3) >> 2) << 2;

          if (subBlkMv[useList].getVer() > 0 &&
              !slice.getRefPic((useList == 1) ? RPL1 : RPL0, reflistIdx[useList])
                 ->m_unscaledPic->m_cs->slice->isIntra() &&
              slice.getRefPic((useList == 1) ? RPL1 : RPL0, reflistIdx[useList])->m_unscaledPic->m_temporalId >=
                PAD_MORE_TL)
          {
            validPadSize = std::max(validPadSize, 4);
          }

          validPadSize = std::max(validPadSize, 0);
          validPadSize = std::min(validPadSize, MC_PAD_SIZE);
          mcBlksize    = mcBlksize.offset(4, validPadSize);
          subBlkMCPos  = subBlkMCPos.offset(0, (ctuSize - validPadSize));
        }
        else if constexpr (T == BD_BOTTOM)
        {
          mvAddOffset.set(0, ((yBlkBoundIdx + 1) * 4) << iMVBitShift);
          validPadSize = (((-subBlkMv[useList].getVer() >> iMVBitShift) + 3) >> 2) << 2;

          if (subBlkMv[useList].getVer() < 0 &&
              !slice.getRefPic((useList == 1) ? RPL1 : RPL0, reflistIdx[useList])
                 ->m_unscaledPic->m_cs->slice->isIntra() &&
              slice.getRefPic((useList == 1) ? RPL1 : RPL0, reflistIdx[useList])->m_unscaledPic->m_temporalId >=
                PAD_MORE_TL)
          {
            validPadSize = std::max(validPadSize, 4);
          }

          validPadSize = std::max(validPadSize, 0);
          validPadSize = std::min(validPadSize, MC_PAD_SIZE);
          mcBlksize    = mcBlksize.offset(4, validPadSize);
        }
        else if constexpr (T == BD_LEFT)
        {
          mvAddOffset.set(-ctuSize << iMVBitShift, 0);
          validPadSize = (((subBlkMv[useList].getHor() >> iMVBitShift) + 3) >> 2) << 2;

          if (subBlkMv[useList].getHor() > 0 &&
              !slice.getRefPic((useList == 1) ? RPL1 : RPL0, reflistIdx[useList])
                 ->m_unscaledPic->m_cs->slice->isIntra() &&
              slice.getRefPic((useList == 1) ? RPL1 : RPL0, reflistIdx[useList])->m_unscaledPic->m_temporalId >=
                PAD_MORE_TL)
          {
            validPadSize = std::max(validPadSize, 4);
          }

          validPadSize = std::max(validPadSize, 0);
          validPadSize = std::min(validPadSize, MC_PAD_SIZE);
          mcBlksize    = mcBlksize.offset(validPadSize, 4);
          subBlkMCPos  = subBlkMCPos.offset((ctuSize - validPadSize), 0);
        }
        else
        {
          mvAddOffset.set(((xBlkBoundIdx + 1) * 4) << iMVBitShift, 0);
          validPadSize = (((-subBlkMv[useList].getHor() >> iMVBitShift) + 3) >> 2) << 2;

          if (subBlkMv[useList].getHor() < 0 &&
              !slice.getRefPic((useList == 1) ? RPL1 : RPL0, reflistIdx[useList])
                 ->m_unscaledPic->m_cs->slice->isIntra() &&
              slice.getRefPic((useList == 1) ? RPL1 : RPL0, reflistIdx[useList])->m_unscaledPic->m_temporalId >=
                PAD_MORE_TL)
          {
            validPadSize = std::max(validPadSize, 4);
          }

          validPadSize = std::max(validPadSize, 0);
          validPadSize = std::min(validPadSize, MC_PAD_SIZE);
          mcBlksize    = mcBlksize.offset(validPadSize, 4);
        }

        tempBiMvFieldAddOffset[useList].mv     = mvAddOffset + subBlkMv[useList];
        tempBiMvFieldAddOffset[useList].refIdx = (int8_t)(reflistIdx[useList]);
        if (reflistIdx[1 - useList] >= 0)
        {
          tempBiMvFieldAddOffset[1 - useList].mv     = mvAddOffset + subBlkMv[1 - useList];
          tempBiMvFieldAddOffset[1 - useList].refIdx = (int8_t)reflistIdx[1 - useList];
        }

        if (validPadSize > 0)
        {
          // start to predict the DC compensate area
          const Size     blkSizeCurBuff = Size(4, 4);
          const Area     blkAreaCurBuff = Area(Position(), blkSizeCurBuff);
          const UnitArea blkUnitCurAreaBuff(slice.m_sps->m_chromaFormatIdc, blkAreaCurBuff);
          int            compDiff[3] = { 0, 0, 0 };
          for (int chan = 0; chan <= maxCh; chan++)
          {
            Position curposition(subBlkMvPos.x >> getComponentScaleX(CompID(chan), ChromaFormat::_420),
                                 subBlkMvPos.y >> getComponentScaleY(CompID(chan), ChromaFormat::_420));
            blkDataTmp->blocks[chan].pos().repositionTo(curposition);
          }
          PelUnitBuf pcYuvPred   = pCurBuffYUV->getBuf(blkUnitCurAreaBuff);
          CodingUnit resizePu4X4 = *blkDataTmp;

          CHECK(pcYuvPred.Y().width != 4, "this is not possible");
          CHECK(pcYuvPred.Y().height != 4, "this is not possible");
          resizePu4X4.UnitArea::operator=(
            UnitArea(blkDataTmp->chromaFormat, Area(blkDataTmp->lumaPos().x, blkDataTmp->lumaPos().y, 4, 4)));
          blkDataTmp->refIdx[useList]     = (int8_t)reflistIdx[useList];
          blkDataTmp->mv[useList]         = subBlkMv[useList];
          blkDataTmp->refIdx[1 - useList] = (int8_t)reflistIdx[1 - useList];
          blkDataTmp->mv[1 - useList]     = subBlkMv[1 - useList];
          blkDataTmp->interDir            = useList + 1;
          resizePu4X4                     = *blkDataTmp;
          xPredInterUni(resizePu4X4, RefPicList(useList), pcYuvPred, false, false, true, true);

          for (int chan = 0; chan <= maxCh; chan++)
          {
            const CompID ch = CompID(chan);

            Pel *piTxtRec = pcCurPic->getBuf(ch, PIC_RECONSTRUCTION)
                              .bufAt(subBlkMvPos.x >> getComponentScaleX(CompID(chan), ChromaFormat::_420),
                                     subBlkMvPos.y >> getComponentScaleY(CompID(chan), ChromaFormat::_420));
            const int iStrideRec = int(pcCurPic->getBuf(ch, PIC_RECONSTRUCTION).stride);

            Pel      *piTxtBuff   = pCurBuffYUV->getBuf(blkUnitCurAreaBuff).bufs[ch].bufAt(0, 0);
            const int iStrideBuff = int(pCurBuffYUV->getBuf(blkUnitCurAreaBuff).bufs[ch].stride);

            for (int idy = 0; idy < (4 >> getComponentScaleY(CompID(chan), ChromaFormat::_420)); idy += 2)
            {
              for (int idx = 0; idx < (4 >> getComponentScaleX(CompID(chan), ChromaFormat::_420)); idx += 2)
              {
                compDiff[chan] += (piTxtRec[idx] - piTxtBuff[idx]);
                compDiff[chan] += (piTxtRec[idx + 1] - piTxtBuff[idx + 1]);
                compDiff[chan] += (piTxtRec[idx + iStrideRec] - piTxtBuff[idx + iStrideBuff]);
                compDiff[chan] += (piTxtRec[idx + 1 + iStrideRec] - piTxtBuff[idx + 1 + iStrideBuff]);
              }
              piTxtRec += iStrideRec << 1;
              piTxtBuff += iStrideBuff << 1;
            }
            compDiff[chan] /= 16 >> getComponentScaleX(CompID(chan), ChromaFormat::_420) >>
              getComponentScaleY(CompID(chan), ChromaFormat::_420);
          }
          // start to predict the padding area
          for (int chan = 0; chan <= maxCh; chan++)
          {
            Position curposition(subBlkMCPos.x >> getComponentScaleX(CompID(chan), ChromaFormat::_420),
                                 subBlkMCPos.y >> getComponentScaleY(CompID(chan), ChromaFormat::_420));
            blkDataTmp->blocks[chan].pos().repositionTo(curposition);
          }
          Size       blkSizeConBuff = Size(mcBlksize.x, mcBlksize.y);
          Area       blkAreaConBuff = Area(Position(), blkSizeConBuff);
          UnitArea   blkUnitConAreaBuff(slice.m_sps->m_chromaFormatIdc, blkAreaConBuff);
          PelUnitBuf pcYuvPad    = pPadYUVContainerDyn->getBuf(blkUnitConAreaBuff);
          CodingUnit resizePuPad = *blkDataTmp;
          CHECK(pcYuvPad.Y().width != mcBlksize.x, "this is not possible");
          CHECK(pcYuvPad.Y().height != mcBlksize.y, "this is not possible");
          resizePuPad.UnitArea::operator=(
            UnitArea(blkDataTmp->chromaFormat,
                     Area(blkDataTmp->lumaPos().x, blkDataTmp->lumaPos().y, mcBlksize.x, mcBlksize.y)));
          blkDataTmp->refIdx[useList]     = tempBiMvFieldAddOffset[useList].refIdx;
          blkDataTmp->mv[useList]         = tempBiMvFieldAddOffset[useList].mv;
          blkDataTmp->refIdx[1 - useList] = tempBiMvFieldAddOffset[1 - useList].refIdx;
          blkDataTmp->mv[1 - useList]     = tempBiMvFieldAddOffset[1 - useList].mv;
          blkDataTmp->interDir            = useList + 1;
          resizePuPad                     = *blkDataTmp;

          xPredInterUni(resizePuPad, RefPicList(useList), pcYuvPad, false, false, true, true);

          for (int chan = 0; chan <= maxCh; chan++)
          {
            const CompID ch = CompID(chan);

            Pel *piTxtBuff = pPadBuffYUV.subBuf(blkUnitAreaBuff)
                               .get(ch)
                               .bufAt((subBlkMCPos - ctuPos).x >> getComponentScaleX(CompID(chan), ChromaFormat::_420),
                                      (subBlkMCPos - ctuPos).y >> getComponentScaleY(CompID(chan), ChromaFormat::_420));
            const int iStrideBuff = int(pPadBuffYUV.get(ch).stride);
            Pel      *piTmpBuff   = pPadYUVContainerDyn->getBuf(blkUnitConAreaBuff).bufs[ch].bufAt(0, 0);
            const int iStrideTmp  = int(pPadYUVContainerDyn->getBuf(blkUnitConAreaBuff).bufs[ch].stride);

            if (isBotTop)
            {
              if (chan == 0)
              {
                for (int idy = 0; idy < mcBlksize.y >> getComponentScaleY(CompID(chan), ChromaFormat::_420); idy++)
                {
                  {
                    piTxtBuff[0] = piTmpBuff[0];
                    piTxtBuff[1] = piTmpBuff[1];
                    piTxtBuff[2] = piTmpBuff[2];
                    piTxtBuff[3] = piTmpBuff[3];

                    piTxtBuff[0] += compDiff[chan];
                    piTxtBuff[1] += compDiff[chan];
                    piTxtBuff[2] += compDiff[chan];
                    piTxtBuff[3] += compDiff[chan];

                    piTxtBuff[0] = (piTxtBuff[0] < 0) ? 0 : piTxtBuff[0];
                    piTxtBuff[1] = (piTxtBuff[1] < 0) ? 0 : piTxtBuff[1];
                    piTxtBuff[2] = (piTxtBuff[2] < 0) ? 0 : piTxtBuff[2];
                    piTxtBuff[3] = (piTxtBuff[3] < 0) ? 0 : piTxtBuff[3];

                    piTxtBuff[0] = (piTxtBuff[0] > 1023) ? 1023 : piTxtBuff[0];
                    piTxtBuff[1] = (piTxtBuff[1] > 1023) ? 1023 : piTxtBuff[1];
                    piTxtBuff[2] = (piTxtBuff[2] > 1023) ? 1023 : piTxtBuff[2];
                    piTxtBuff[3] = (piTxtBuff[3] > 1023) ? 1023 : piTxtBuff[3];
                  }
                  piTxtBuff += iStrideBuff;
                  piTmpBuff += iStrideTmp;
                }
              }
              else
              {
                for (int idy = 0; idy < mcBlksize.y >> getComponentScaleY(CompID(chan), ChromaFormat::_420); idy++)
                {
                  piTxtBuff[0] = piTmpBuff[0];
                  piTxtBuff[1] = piTmpBuff[1];

                  piTxtBuff[0] += compDiff[chan];
                  piTxtBuff[1] += compDiff[chan];

                  piTxtBuff[0] = (piTxtBuff[0] < 0) ? 0 : piTxtBuff[0];
                  piTxtBuff[1] = (piTxtBuff[1] < 0) ? 0 : piTxtBuff[1];

                  piTxtBuff[0] = (piTxtBuff[0] > 1023) ? 1023 : piTxtBuff[0];
                  piTxtBuff[1] = (piTxtBuff[1] > 1023) ? 1023 : piTxtBuff[1];

                  piTxtBuff += iStrideBuff;
                  piTmpBuff += iStrideTmp;
                }
              }
            }
            else
            {
              if (chan == 0)
              {
                for (int idx = 0; idx < mcBlksize.x >> getComponentScaleX(CompID(chan), ChromaFormat::_420); idx++)
                {
                  const int idx1Txt = idx + iStrideBuff;
                  const int idx1Tmp = idx + iStrideTmp;
                  const int idx2Txt = idx + (iStrideBuff << 1);
                  const int idx2Tmp = idx + (iStrideTmp << 1);
                  const int idx3Txt = idx + iStrideBuff + (iStrideBuff << 1);
                  const int idx3Tmp = idx + iStrideTmp + (iStrideTmp << 1);

                  piTxtBuff[idx]     = piTmpBuff[idx];
                  piTxtBuff[idx1Txt] = piTmpBuff[idx1Tmp];
                  piTxtBuff[idx2Txt] = piTmpBuff[idx2Tmp];
                  piTxtBuff[idx3Txt] = piTmpBuff[idx3Tmp];

                  piTxtBuff[idx] += compDiff[chan];
                  piTxtBuff[idx1Txt] += compDiff[chan];
                  piTxtBuff[idx2Txt] += compDiff[chan];
                  piTxtBuff[idx3Txt] += compDiff[chan];

                  piTxtBuff[idx]     = (piTxtBuff[idx] < 0) ? 0 : piTxtBuff[idx];
                  piTxtBuff[idx1Txt] = (piTxtBuff[idx1Txt] < 0) ? 0 : piTxtBuff[idx1Txt];
                  piTxtBuff[idx2Txt] = (piTxtBuff[idx2Txt] < 0) ? 0 : piTxtBuff[idx2Txt];
                  piTxtBuff[idx3Txt] = (piTxtBuff[idx3Txt] < 0) ? 0 : piTxtBuff[idx3Txt];

                  piTxtBuff[idx]     = (piTxtBuff[idx] > 1023) ? 1023 : piTxtBuff[idx];
                  piTxtBuff[idx1Txt] = (piTxtBuff[idx1Txt] > 1023) ? 1023 : piTxtBuff[idx1Txt];
                  piTxtBuff[idx2Txt] = (piTxtBuff[idx2Txt] > 1023) ? 1023 : piTxtBuff[idx2Txt];
                  piTxtBuff[idx3Txt] = (piTxtBuff[idx3Txt] > 1023) ? 1023 : piTxtBuff[idx3Txt];
                }
              }
              else
              {
                for (int idx = 0; idx < mcBlksize.x >> getComponentScaleX(CompID(chan), ChromaFormat::_420); idx++)
                {
                  const int idx1Txt = idx + iStrideBuff;
                  const int idx1Tmp = idx + iStrideTmp;

                  piTxtBuff[idx]     = piTmpBuff[idx];
                  piTxtBuff[idx1Txt] = piTmpBuff[idx1Tmp];

                  piTxtBuff[idx] += compDiff[chan];
                  piTxtBuff[idx1Txt] += compDiff[chan];

                  piTxtBuff[idx]     = (piTxtBuff[idx] < 0) ? 0 : piTxtBuff[idx];
                  piTxtBuff[idx1Txt] = (piTxtBuff[idx1Txt] < 0) ? 0 : piTxtBuff[idx1Txt];

                  piTxtBuff[idx]     = (piTxtBuff[idx] > 1023) ? 1023 : piTxtBuff[idx];
                  piTxtBuff[idx1Txt] = (piTxtBuff[idx1Txt] > 1023) ? 1023 : piTxtBuff[idx1Txt];
                }
              }
            }
          }
        }
      }

      for (int chan = 0; chan <= maxCh; chan++)
      {
        const CompID ch = CompID(chan);
        Position     subBlkRepSrcPos;
        Position     subBlkRepPos;
        Position     repBlkSize;

        const int iStrideBuff = int(pPadBuffYUV.get(ch).stride);

        if constexpr (T == BD_TOP)
        {
          repBlkSize     = repBlkSize.offset(4 >> getComponentScaleX(ch, ChromaFormat::_420),
                                             (ctuSize - validPadSize) >> getComponentScaleY(ch, ChromaFormat::_420));
          subBlkRepPos   = subBlkRepPos.offset(subBlkIdx * 4, 0);
          Pel *piTxtBuff = pPadBuffYUV.subBuf(blkUnitAreaBuff)
                             .get(ch)
                             .bufAt(subBlkRepPos.x >> getComponentScaleX(ch, ChromaFormat::_420),
                                    subBlkRepPos.y >> getComponentScaleY(ch, ChromaFormat::_420));
          Pel *piTxtSrc;
          if (validPadSize == 0)
          {
            subBlkRepSrcPos = subBlkRepSrcPos.offset(subBlkPos.x, subBlkPos.y);
            piTxtSrc        = pcCurPic->getBuf(ch, PIC_RECONSTRUCTION)
                         .bufAt(subBlkRepSrcPos.x >> getComponentScaleX(ch, ChromaFormat::_420),
                                subBlkRepSrcPos.y >> getComponentScaleY(ch, ChromaFormat::_420));
          }
          else
          {
            subBlkRepSrcPos = subBlkRepSrcPos.offset(subBlkIdx * 4, (ctuSize - validPadSize));
            piTxtSrc        = pPadBuffYUV.subBuf(blkUnitAreaBuff)
                         .get(ch)
                         .bufAt(subBlkRepSrcPos.x >> getComponentScaleX(ch, ChromaFormat::_420),
                                subBlkRepSrcPos.y >> getComponentScaleY(ch, ChromaFormat::_420));
          }
          for (int idy = 0; idy < repBlkSize.y; idy++)
          {
            memcpy(piTxtBuff, piTxtSrc, sizeof(Pel) * repBlkSize.x);
            piTxtBuff += iStrideBuff;
          }
        }
        else if constexpr (T == BD_BOTTOM)
        {
          repBlkSize     = repBlkSize.offset(4 >> getComponentScaleX(ch, ChromaFormat::_420),
                                             (ctuSize - validPadSize) >> getComponentScaleY(ch, ChromaFormat::_420));
          subBlkRepPos   = subBlkRepPos.offset(subBlkIdx * 4, validPadSize);
          Pel *piTxtBuff = pPadBuffYUV.subBuf(blkUnitAreaBuff)
                             .get(ch)
                             .bufAt(subBlkRepPos.x >> getComponentScaleX(ch, ChromaFormat::_420),
                                    subBlkRepPos.y >> getComponentScaleY(ch, ChromaFormat::_420));
          Pel *piTxtSrc;
          if (validPadSize == 0)
          {
            subBlkRepSrcPos = subBlkRepSrcPos.offset(subBlkPos.x, iHeightFrm);
            piTxtSrc        = pcCurPic->getBuf(ch, PIC_RECONSTRUCTION)
                         .bufAt(subBlkRepSrcPos.x >> getComponentScaleX(ch, ChromaFormat::_420),
                                (subBlkRepSrcPos.y >> getComponentScaleY(ch, ChromaFormat::_420)) - 1);
          }
          else
          {
            subBlkRepSrcPos = subBlkRepSrcPos.offset(subBlkIdx * 4, validPadSize);
            piTxtSrc        = pPadBuffYUV.subBuf(blkUnitAreaBuff)
                         .get(ch)
                         .bufAt(subBlkRepSrcPos.x >> getComponentScaleX(ch, ChromaFormat::_420),
                                (subBlkRepSrcPos.y >> getComponentScaleY(ch, ChromaFormat::_420)) - 1);
          }
          for (int idy = 0; idy < repBlkSize.y; idy++)
          {
            memcpy(piTxtBuff, piTxtSrc, sizeof(Pel) * repBlkSize.x);
            piTxtBuff += iStrideBuff;
          }
        }
        else if constexpr (T == BD_LEFT)
        {
          int iStrideSrc;
          repBlkSize     = repBlkSize.offset((ctuSize - validPadSize) >> getComponentScaleX(ch, ChromaFormat::_420),
                                             4 >> getComponentScaleY(ch, ChromaFormat::_420));
          subBlkRepPos   = subBlkRepPos.offset(0, subBlkIdx * 4);
          Pel *piTxtBuff = pPadBuffYUV.subBuf(blkUnitAreaBuff)
                             .get(ch)
                             .bufAt(subBlkRepPos.x >> getComponentScaleX(ch, ChromaFormat::_420),
                                    subBlkRepPos.y >> getComponentScaleY(ch, ChromaFormat::_420));
          Pel *piTxtSrc;
          if (validPadSize == 0)
          {
            subBlkRepSrcPos = subBlkRepSrcPos.offset(subBlkPos.x, subBlkPos.y);
            piTxtSrc        = pcCurPic->getBuf(ch, PIC_RECONSTRUCTION)
                         .bufAt(subBlkRepSrcPos.x >> getComponentScaleX(ch, ChromaFormat::_420),
                                subBlkRepSrcPos.y >> getComponentScaleY(ch, ChromaFormat::_420));
            iStrideSrc = int(pcCurPic->getBuf(ch, PIC_RECONSTRUCTION).stride);
          }
          else
          {
            subBlkRepSrcPos = subBlkRepSrcPos.offset((ctuSize - validPadSize), subBlkIdx * 4);
            piTxtSrc        = pPadBuffYUV.subBuf(blkUnitAreaBuff)
                         .get(ch)
                         .bufAt(subBlkRepSrcPos.x >> getComponentScaleX(ch, ChromaFormat::_420),
                                subBlkRepSrcPos.y >> getComponentScaleY(ch, ChromaFormat::_420));
            iStrideSrc = int(pPadBuffYUV.get(ch).stride);
          }
          for (int idy = 0; idy < repBlkSize.y; idy++)
          {
            for (int idx = 0; idx < repBlkSize.x; idx++)
            {
              piTxtBuff[idx] = piTxtSrc[0];
            }
            piTxtBuff += iStrideBuff;
            piTxtSrc += iStrideSrc;
          }
        }
        else
        {
          int iStrideSrc;
          subBlkRepPos   = subBlkRepPos.offset(validPadSize, subBlkIdx * 4);
          Pel *piTxtBuff = pPadBuffYUV.subBuf(blkUnitAreaBuff)
                             .get(ch)
                             .bufAt(subBlkRepPos.x >> getComponentScaleX(ch, ChromaFormat::_420),
                                    subBlkRepPos.y >> getComponentScaleY(ch, ChromaFormat::_420));
          repBlkSize = repBlkSize.offset((ctuSize - validPadSize) >> getComponentScaleX(ch, ChromaFormat::_420),
                                         4 >> getComponentScaleY(ch, ChromaFormat::_420));
          Pel *piTxtSrc;
          if (validPadSize == 0)
          {
            subBlkRepSrcPos = subBlkRepSrcPos.offset(iWidthFrm, subBlkPos.y);
            piTxtSrc        = pcCurPic->getBuf(ch, PIC_RECONSTRUCTION)
                         .bufAt((subBlkRepSrcPos.x >> getComponentScaleX(ch, ChromaFormat::_420)) - 1,
                                subBlkRepSrcPos.y >> getComponentScaleY(ch, ChromaFormat::_420));
            iStrideSrc = int(pcCurPic->getBuf(ch, PIC_RECONSTRUCTION).stride);
          }
          else
          {
            subBlkRepSrcPos = subBlkRepSrcPos.offset(validPadSize, subBlkIdx * 4);
            piTxtSrc        = pPadBuffYUV.subBuf(blkUnitAreaBuff)
                         .get(ch)
                         .bufAt((subBlkRepSrcPos.x >> getComponentScaleX(ch, ChromaFormat::_420)) - 1,
                                subBlkRepSrcPos.y >> getComponentScaleY(ch, ChromaFormat::_420));
            iStrideSrc = int(pPadBuffYUV.get(ch).stride);
          }
          for (int idy = 0; idy < repBlkSize.y; idy++)
          {
            for (int idx = 0; idx < repBlkSize.x; idx++)
            {
              piTxtBuff[idx] = piTxtSrc[0];
            }
            piTxtBuff += iStrideBuff;
            piTxtSrc += iStrideSrc;
          }
        }
      }
    }
  }
}
