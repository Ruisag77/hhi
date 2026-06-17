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

/** \file     dtrace_codingstruct.h
 *  \brief    Easy to use dtrace calls concerning coding structures
 */

#ifndef _DTRACE_CODINGSTRUCT_H_
#define _DTRACE_CODINGSTRUCT_H_

#include "dtrace.h"
#include "dtrace_next.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/CodingStructure.h"
#include "CommonLib/Slice.h"
#include "CommonLib/Mv.h"
#include "CommonLib/Unit.h"
#include "CommonLib/UnitTools.h"

#include <cmath>

#if ENABLE_TRACING

inline void dtracePicComp(DTRACE_CHANNEL channel, CodingStructure &cs, const CPelUnitBuf &pelUnitBuf, CompID compId)
{
  if (!g_trace_ctx)
  {
    return;
  }
  if (!isChromaEnabled(pelUnitBuf.chromaFormat) && compId != COMP_Y)
  {
    return;
  }

  const Pel *src = pelUnitBuf.bufs[compId].buf;

  ptrdiff_t uiStride       = pelUnitBuf.bufs[compId].stride;
  uint32_t  uiWidth        = pelUnitBuf.bufs[compId].width;
  uint32_t  uiHeight       = pelUnitBuf.bufs[compId].height;
  uint32_t  uiChromaScaleX = getComponentScaleX(compId, pelUnitBuf.chromaFormat);
  uint32_t  uiChromaScaleY = getComponentScaleY(compId, pelUnitBuf.chromaFormat);

  DTRACE(g_trace_ctx, channel, "\n%s: poc = %d, size=%dx%d\n\n", g_trace_ctx->getChannelName(channel), cs.slice->m_poc,
         uiWidth, uiHeight);
  DTRACE_FRAME_BLOCKWISE(g_trace_ctx, channel, src, uiStride, uiWidth, uiHeight, cs.sps->m_maxCuWidth >> uiChromaScaleX,
                         cs.sps->m_maxCuHeight >> uiChromaScaleY);
}

#define OLD_RDCOST 1

inline void dtraceModeCost(CodingStructure &cs, double lambda)
{
  CHECK(cs.cus.size() != 1, "Only the cost for a single CU can be show with dtraceModeCost!");

  Distortion tempDist = cs.dist;

#if OLD_RDCOST
  uint64_t tempBits = cs.fracBits >> SCALE_BITS;
  uint64_t tempCost = (uint64_t)(cs.dist + (double)tempBits * lambda);
#else
  uint64_t tempBits = cs.fracBits;
  uint64_t tempCost = (uint64_t)cs.cost;
#endif

  if (cs.cost == MAX_DOUBLE)
  {
    tempCost = 0;
    tempBits = 0;
    tempDist = 0;
  }

  bool isIntra    = CU::isIntra(*cs.cus.front());
  int  intraModeL = isIntra ? cs.cus.front()->intraDir[ChannelType::LUMA] : 0;
  int  intraModeC = isIntra ? cs.cus.front()->intraDir[ChannelType::CHROMA] : 0;
  if (isIntra && intraModeC == DM_CHROMA_IDX)
  {
    intraModeC = 68;
  }
  int imvVal = 0;
  imvVal     = cs.cus[0]->imv;
  DTRACE(g_trace_ctx, D_MODE_COST,
         "ModeCost: %6lld %3d @(%4d,%4d) [%2dx%2d] %d "
         "(qp%d,pm%d,skip%d,m%d,mr%d,ci%d,g%d,gi%d,gm%d,%d,aff%d,mm%d,mmdir%d,mmstep%d,bm%d,timd%d,imv%d,lic%d/"
         "%d,IL%d,IC%d) tempCS = %lld (%d,%d)\n",
         DTRACE_GET_COUNTER(g_trace_ctx, D_MODE_COST), cs.slice->m_poc, cs.area.lx(), cs.area.ly(), cs.area.lwidth(),
         cs.area.lheight(), cs.cus[0]->qtDepth, cs.cus[0]->qp, cs.cus[0]->predMode, cs.cus[0]->skip,
         cs.cus[0]->mergeFlag, cs.cus[0]->regularMergeFlag, cs.cus[0]->ciipFlag, cs.cus[0]->geoFlag,
         cs.cus[0]->gpmIntraFlag, cs.cus[0]->geoMMVDFlag[0], cs.cus[0]->geoMMVDFlag[1], cs.cus[0]->affine,
         cs.cus[0]->mmvdMergeFlag, cs.cus[0]->mmvdMergeIdx.pos.position, cs.cus[0]->mmvdMergeIdx.pos.step,
         cs.cus[0]->bmMergeFlag, cs.cus[0]->timdFlag, imvVal, cs.cus[0]->licFlag, cs.cus[0]->oppositeLicFlag,
         intraModeL, intraModeC, tempCost, tempBits, tempDist);
}

inline void dtraceBestMode(CodingStructure *&tempCS, CodingStructure *&bestCS, double lambda, bool edo = false)
{
  bool        bSplitCS = tempCS->cus.size() > 1 || bestCS->cus.size() > 1;
  ChannelType chType   = tempCS->cus.back()->chType;

  // if the last CU does not align with the CS, we probably are at the edge
  bSplitCS |= tempCS->cus.back()->block(chType).bottomRight() != tempCS->area.block(chType).bottomRight();

  Distortion tempDist = tempCS->dist;
  Distortion bestDist = bestCS->dist;

#if OLD_RDCOST
  uint64_t tempBits = tempCS->fracBits >> SCALE_BITS;
  uint64_t bestBits = bestCS->fracBits >> SCALE_BITS;
  uint64_t tempCost = (uint64_t)(tempCS->dist + (double)tempBits * lambda);
  uint64_t bestCost = (uint64_t)(bestCS->dist + (double)bestBits * lambda);
#else
  uint64_t tempBits = tempCS->fracBits >> SCALE_BITS;
  uint64_t bestBits = bestCS->fracBits >> SCALE_BITS;
  uint64_t tempCost = (uint64_t)tempCS->cost;
  uint64_t bestCost = (uint64_t)bestCS->cost;
#endif

  uint64_t dbTempCost    = edo ? tempCS->costDbOffset : 0;
  uint64_t dbBestCost    = edo ? bestCS->costDbOffset : 0;
  uint64_t tempCostFinal = tempCost + dbTempCost;
  uint64_t bestCostFinal = bestCost + dbBestCost;

  if (tempCS->cost == MAX_DOUBLE)
  {
    tempCostFinal = MAX_UINT64;
    tempCost      = 0;
    tempBits      = 0;
    tempDist      = 0;
    dbTempCost    = 0;
  }
  if (bestCS->cost == MAX_DOUBLE)
  {
    bestCostFinal = MAX_UINT64;
    bestCost      = 0;
    bestBits      = 0;
    bestDist      = 0;
    dbBestCost    = 0;
  }

  bool isIntra    = CU::isIntra(*tempCS->cus[0]);
  int  intraModeL = isIntra ? tempCS->cus[0]->intraDir[ChannelType::LUMA] : 0;
  int  intraModeC = isIntra ? tempCS->cus[0]->intraDir[ChannelType::CHROMA] : 0;

  if (!bSplitCS)
  {
    DTRACE(g_trace_ctx, D_BEST_MODE,
           "CheckModeCost: %6lld %3d @(%4d,%4d) [%2dx%2d] %d (%d,%d,%2d,%d,%d) tempCS = %llu (%d,%d,%d,%d), bestCS = "
           "%llu (%d,%d,%d,%d): --> choose %s\n",
           DTRACE_GET_COUNTER(g_trace_ctx, D_BEST_MODE), tempCS->slice->m_poc, tempCS->area.lx(), tempCS->area.ly(),
           tempCS->area.lwidth(), tempCS->area.lheight(), tempCS->cus[0]->qtDepth, tempCS->cus[0]->qp,
           tempCS->cus[0]->predMode, tempCS->cus[0]->mergeFlag, intraModeL, intraModeC, tempCostFinal, tempBits,
           tempDist, tempCost, dbTempCost, bestCostFinal, bestBits, bestDist, bestCost, dbBestCost,
           tempCostFinal < bestCostFinal ? "TEMP" : "BEST");
  }
  else
  {
    DTRACE(g_trace_ctx, D_BEST_MODE,
           "CheckModeSplitCost: %6lld %3d @(%4d,%4d) [%2dx%2d] -------------------------- tempCS = %llu (%d,%d,%d,%d), "
           "bestCS = %llu (%d,%d,%d,%d): --> choose %s\n",
           DTRACE_GET_COUNTER(g_trace_ctx, D_BEST_MODE), tempCS->slice->m_poc, tempCS->area.lx(), tempCS->area.ly(),
           tempCS->area.lwidth(), tempCS->area.lheight(), tempCostFinal, tempBits, tempDist, tempCost, dbTempCost,
           bestCostFinal, bestBits, bestDist, bestCost, dbBestCost,
           tempCostFinal < bestCostFinal ? "TEMP STRUCTURE" : "BEST STRUCTURE");
  }
}

#define DTRACE_PIC_COMP(...)  dtracePicComp(__VA_ARGS__)
#define DTRACE_BEST_MODE(...) dtraceBestMode(__VA_ARGS__)
#define DTRACE_MODE_COST(...) dtraceModeCost(__VA_ARGS__)
#define DTRACE_STAT(...)      dtraceComprPicStat(__VA_ARGS__)

#else

#define DTRACE_BEST_MODE(...)
#define DTRACE_MODE_COST(...)
#define DTRACE_STAT(...)
#define DTRACE_PIC_COMP(...)

#endif

#endif // _DTRACE_HEVC_H_
