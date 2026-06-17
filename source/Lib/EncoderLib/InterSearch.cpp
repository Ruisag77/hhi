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

/** \file     EncSearch.cpp
 *  \brief    encoder inter search class
 */

#include "InterSearch.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Rom.h"
#include "CommonLib/MotionInfo.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_next.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/MCTS.h"
#include "CommonLib/BilateralFilter.h"
#include "EncModeCtrl.h"
#include "EncLib.h"

#include <math.h>
#include <limits>

//! \ingroup EncoderLib
//! \{

void InterSearch::resetReusedUniMvs()
{
  GCC_WARNING_DISABLE_class_memaccess memset(m_reusedUniMvBuf[0], 0, m_numReusedUniMvBufs * sizeof(MvRefSetArray));
  memset(m_reusedUniMvBuf[1], 0, m_numReusedUniMvBufs * sizeof(MvRefSetArray));
  GCC_WARNING_RESET
}

void InterSearch::xDestroyReusedUniMvs()
{
  m_numReusedUniMvBufs = 0;
  delete[] m_reusedUniMvBuf[0];
  delete[] m_reusedUniMvBuf[1];
  m_reusedUniMvBuf[0] = m_reusedUniMvBuf[1] = nullptr;
  GCC_WARNING_DISABLE_class_memaccess memset(m_reusedUniMVs, 0, sizeof(m_reusedUniMVs));
  memset(m_reusedUniMVsLIC, 0, sizeof(m_reusedUniMVsLIC));
  GCC_WARNING_RESET
}

void InterSearch::xCreateReusedUniMvs()
{
  GCC_WARNING_DISABLE_class_memaccess memset(m_reusedUniMVs, 0, sizeof(m_reusedUniMVs));
  memset(m_reusedUniMVsLIC, 0, sizeof(m_reusedUniMVsLIC));
  GCC_WARNING_RESET
  m_numReusedUniMvBufs = 0;

  for (int x = 0; x < MAX_CU_SIZE_IN_PARTS; x++)
  {
    for (int wIdx = 0; wIdx < MAX_NUM_SIZES; wIdx++)
    {
      const SizeType w = gp_sizeIdxInfo->sizeFrom(wIdx);
      if (!gp_sizeIdxInfo->isCuSize(w) || !gp_sizeIdxInfo->hasSizeAtOffset(x << MIN_CU_LOG2, w))
      {
        continue;
      }

      for (int y = 0; y < MAX_CU_SIZE_IN_PARTS; y++)
      {
        for (int hIdx = 0; hIdx < MAX_NUM_SIZES; hIdx++)
        {
          const SizeType h = gp_sizeIdxInfo->sizeFrom(hIdx);
          if (!gp_sizeIdxInfo->isCuSize(h) || !gp_sizeIdxInfo->hasSizeAtOffset(y << MIN_CU_LOG2, h))
          {
            continue;
          }

          m_numReusedUniMvBufs++;
        }
      }
    }
  }

  m_reusedUniMvBuf[0] = new MvRefSetArray[m_numReusedUniMvBufs];
  m_reusedUniMvBuf[1] = new MvRefSetArray[m_numReusedUniMvBufs];

  int idx = 0;

  for (int x = 0; x < MAX_CU_SIZE_IN_PARTS; x++)
  {
    for (int wIdx = 0; wIdx < MAX_NUM_SIZES; wIdx++)
    {
      const SizeType w = gp_sizeIdxInfo->sizeFrom(wIdx);
      if (!gp_sizeIdxInfo->isCuSize(w) || !gp_sizeIdxInfo->hasSizeAtOffset(x << MIN_CU_LOG2, w))
      {
        continue;
      }

      for (int y = 0; y < MAX_CU_SIZE_IN_PARTS; y++)
      {
        for (int hIdx = 0; hIdx < MAX_NUM_SIZES; hIdx++)
        {
          const SizeType h = gp_sizeIdxInfo->sizeFrom(hIdx);
          if (!gp_sizeIdxInfo->isCuSize(h) || !gp_sizeIdxInfo->hasSizeAtOffset(y << MIN_CU_LOG2, h))
          {
            continue;
          }

          m_reusedUniMVs[x][y][wIdx][hIdx]    = &m_reusedUniMvBuf[0][idx];
          m_reusedUniMVsLIC[x][y][wIdx][hIdx] = &m_reusedUniMvBuf[1][idx];

          idx++;
        }
      }
    }
  }

  resetReusedUniMvs();
}

void InterSearch::insertUniMvCandsReuseMv(const Area &blkArea, const PreCalcValues &pcv)
{
  unsigned idX, idY, idW, idH;
  getAreaIdx(blkArea, pcv, idX, idY, idW, idH);
  if (m_reusedUniMVs[idX][idY][idW][idH]->valid)
  {
    insertUniMvCands(blkArea, m_reusedUniMVs[idX][idY][idW][idH]->entry);
  }
  if (m_reusedUniMVsLIC[idX][idY][idW][idH]->valid)
  {
    swapUniMvBuffer();
    insertUniMvCands(blkArea, m_reusedUniMVsLIC[idX][idY][idW][idH]->entry);
    swapUniMvBuffer();
  }
}

static const Mv s_acMvRefineH[9] = {
  Mv(0, 0), // 0
  Mv(0, -1), // 1
  Mv(0, 1), // 2
  Mv(-1, 0), // 3
  Mv(1, 0), // 4
  Mv(-1, -1), // 5
  Mv(1, -1), // 6
  Mv(-1, 1), // 7
  Mv(1, 1)  // 8
};

static const Mv s_acMvRefineQ[9] = {
  Mv(0, 0), // 0
  Mv(0, -1), // 1
  Mv(0, 1), // 2
  Mv(-1, -1), // 5
  Mv(1, -1), // 6
  Mv(-1, 0), // 3
  Mv(1, 0), // 4
  Mv(-1, 1), // 7
  Mv(1, 1)  // 8
};

InterSearch::InterSearch()
  : m_encCfg(nullptr)
  , m_bilateralFilter(nullptr)
  , m_modeCtrl(nullptr)
  , m_pcTrQuant(nullptr)
  , m_pcReshape(nullptr)
  , m_searchRange(0)
  , m_bipredSearchRange(0)
  , m_motionEstimationSearchMethod(MESearchMethod::FULL)
  , m_CABACEstimator(nullptr)
  , m_ctxPool(nullptr)
  , m_pTempPel(nullptr)
  , m_isInitialized(false)
{
  for (int i = 0; i < MAX_NUM_REF_LIST_ADAPT_SR; i++)
  {
    memset(m_adaptSR[i], 0, MAX_IDX_ADAPT_SR * sizeof(int));
  }
  for (int i = 0; i < AMVP_MAX_NUM_CANDS + 1; i++)
  {
    memset(m_auiMVPIdxCost[i], 0, (AMVP_MAX_NUM_CANDS + 1) * sizeof(uint32_t));
  }

  xSetWpScalingDistParam(-1, RPLX, nullptr);
  m_affMVList        = nullptr;
  m_affMVListSize    = 0;
  m_affMVListIdx     = 0;
  m_uniMvList        = nullptr;
  m_uniMvListSize    = 0;
  m_uniMvListIdx     = 0;
  m_uniMvListLIC     = nullptr;
  m_uniMvListSizeLIC = 0;
  m_uniMvListIdxLIC  = 0;
  m_histBestSbt      = MAX_UCHAR;
  m_histBestMtsIdx   = MtsType::NONE;

  xCreateReusedUniMvs();
}

void InterSearch::destroy()
{
  CHECK(!m_isInitialized, "Not initialized");
  if (m_pTempPel)
  {
    delete[] m_pTempPel;
    m_pTempPel = nullptr;
  }

  m_pSaveCS = nullptr;

  for (uint32_t i = 0; i < NUM_RPL01; i++)
  {
    m_tmpPredStorage[i].destroy();
  }
  m_tmpStorageCtu.destroy();
  m_tmpAffiStorage.destroy();

  if (m_tmpAffiError != nullptr)
  {
    delete[] m_tmpAffiError;
  }
  if (m_tmpAffiDeri[0] != nullptr)
  {
    delete[] m_tmpAffiDeri[0];
  }
  if (m_tmpAffiDeri[1] != nullptr)
  {
    delete[] m_tmpAffiDeri[1];
  }
  if (m_affMVList)
  {
    delete[] m_affMVList;
    m_affMVList = nullptr;
  }

  m_affMVListIdx  = 0;
  m_affMVListSize = 0;
  if (m_uniMvList)
  {
    delete[] m_uniMvList;
    m_uniMvList = nullptr;
  }
  m_uniMvListIdx  = 0;
  m_uniMvListSize = 0;
  if (m_uniMvListLIC)
  {
    delete[] m_uniMvListLIC;
    m_uniMvListLIC = nullptr;
  }
  m_uniMvListIdxLIC  = 0;
  m_uniMvListSizeLIC = 0;

  xDestroyReusedUniMvs();

  m_isInitialized = false;
}

void InterSearch::setTempBuffers(CodingStructure **pSaveCS) { m_pSaveCS = pSaveCS; }

InterSearch::~InterSearch()
{
  if (m_isInitialized)
  {
    destroy();
  }
}

void InterSearch::init(const EncCfg *encCfg, BilateralFilter *bilateralFilter, TrQuant *pcTrQuant,
                       EncModeCtrl *pcEncModeCtrl, int searchRange, int bipredSearchRange,
                       MESearchMethod motionEstimationSearchMethod, bool useCompositeRef, const uint32_t maxCUWidth,
                       const uint32_t maxCUHeight, const uint32_t maxTotalCUDepth, RdCost *pcRdCost,
                       CABACWriter *CABACEstimator, CtxPool *ctxPool, EncReshape *pcReshape,
                       const uint32_t curPicWidthY, InterpolationFilter *pcInterpolationFilter)
{
  CHECK(m_isInitialized, "Already initialized");
  m_defaultCachedBvs.clear();
  m_encCfg                       = encCfg;
  m_bilateralFilter              = bilateralFilter;
  m_pcTrQuant                    = pcTrQuant;
  m_searchRange                  = searchRange;
  m_bipredSearchRange            = bipredSearchRange;
  m_motionEstimationSearchMethod = motionEstimationSearchMethod;
  m_CABACEstimator               = CABACEstimator;
  m_ctxPool                      = ctxPool;
  m_useCompositeRef              = useCompositeRef;
  m_pcReshape                    = pcReshape;
  m_modeCtrl                     = pcEncModeCtrl;

  for (uint32_t dir = 0; dir < MAX_NUM_REF_LIST_ADAPT_SR; dir++)
  {
    for (uint32_t refIdx = 0; refIdx < MAX_IDX_ADAPT_SR; refIdx++)
    {
      m_adaptSR[dir][refIdx] = searchRange;
    }
  }

  // initialize motion cost
  for (int num = 0; num < AMVP_MAX_NUM_CANDS + 1; num++)
  {
    for (int idx = 0; idx < AMVP_MAX_NUM_CANDS; idx++)
    {
      if (idx < num)
      {
        m_auiMVPIdxCost[idx][num] = xGetMvpIdxBits(idx, num);
      }
      else
      {
        m_auiMVPIdxCost[idx][num] = MAX_UINT;
      }
    }
  }

  const ChromaFormat cform = encCfg->m_chromaFormatIdc;
  InterPrediction::init(pcRdCost, pcReshape, cform, maxCUHeight, curPicWidthY, pcInterpolationFilter);

  for (uint32_t i = 0; i < NUM_RPL01; i++)
  {
    m_tmpPredStorage[i].create(UnitArea(cform, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
  }
  m_tmpStorageCtu.create(UnitArea(cform, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
  m_tmpAffiStorage.create(UnitArea(cform, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
  m_tmpAffiError     = new Pel[MAX_CU_SIZE * MAX_CU_SIZE];
  m_tmpAffiDeri[0]   = new Pel[MAX_CU_SIZE * MAX_CU_SIZE];
  m_tmpAffiDeri[1]   = new Pel[MAX_CU_SIZE * MAX_CU_SIZE];
  m_pTempPel         = new Pel[maxCUWidth * maxCUHeight];
  m_affMVListMaxSize = encCfg->m_isLowDelay ? AFFINE_ME_LIST_SIZE_LD : AFFINE_ME_LIST_SIZE;
  if (!m_affMVList)
  {
    m_affMVList = new AffineMVInfo[m_affMVListMaxSize];
  }
  m_affMVListIdx     = 0;
  m_affMVListSize    = 0;
  m_uniMvListMaxSize = 15;
  if (!m_uniMvList)
  {
    m_uniMvList = new BlkUniMvInfo[m_uniMvListMaxSize];
  }
  m_uniMvListIdx  = 0;
  m_uniMvListSize = 0;
  if (!m_uniMvListLIC)
  {
    m_uniMvListLIC = new BlkUniMvInfo[m_uniMvListMaxSize];
  }
  m_uniMvListIdxLIC  = 0;
  m_uniMvListSizeLIC = 0;
  m_isInitialized    = true;
}

void InterSearch::resetSavedAffineMotion()
{
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      m_affineMotion.acMvAffine4Para[i][j] = Mv(0, 0);
      m_affineMotion.acMvAffine6Para[i][j] = Mv(0, 0);
    }
    m_affineMotion.acMvAffine6Para[i][2] = Mv(0, 0);

    m_affineMotion.affine4ParaRefIdx[i] = -1;
    m_affineMotion.affine6ParaRefIdx[i] = -1;
  }
  for (int i = 0; i < 3; i++)
  {
    m_affineMotion.hevcCost[i] = std::numeric_limits<Distortion>::max();
  }
  m_affineMotion.affine4ParaAvail = false;
  m_affineMotion.affine6ParaAvail = false;
}

void InterSearch::storeAffineMotion(Mv acAffineMv[2][3], int16_t affineRefIdx[2], AffineModel affineType, int bcwIdx)
{
  if ((bcwIdx == BCW_DEFAULT || !m_affineMotion.affine6ParaAvail) && affineType == AffineModel::_6_PARAMS)
  {
    for (int i = 0; i < 2; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        m_affineMotion.acMvAffine6Para[i][j] = acAffineMv[i][j];
      }
      m_affineMotion.affine6ParaRefIdx[i] = affineRefIdx[i];
    }
    m_affineMotion.affine6ParaAvail = true;
  }

  if ((bcwIdx == BCW_DEFAULT || !m_affineMotion.affine4ParaAvail) && affineType == AffineModel::_4_PARAMS)
  {
    for (int i = 0; i < 2; i++)
    {
      for (int j = 0; j < 2; j++)
      {
        m_affineMotion.acMvAffine4Para[i][j] = acAffineMv[i][j];
      }
      m_affineMotion.affine4ParaRefIdx[i] = affineRefIdx[i];
    }
    m_affineMotion.affine4ParaAvail = true;
  }
}

void InterSearch::savePrevAffMVInfo(int idx, AffineMVInfo &tmpMVInfo, bool &isSaved)
{
  if (m_affMVListSize > idx)
  {
    tmpMVInfo = m_affMVList[(m_affMVListIdx - 1 - idx + m_affMVListMaxSize) % m_affMVListMaxSize];
    isSaved   = true;
  }
  else
  {
    isSaved = false;
  }
}

void InterSearch::addAffMVInfo(AffineMVInfo &tmpMVInfo)
{
  int           j        = 0;
  AffineMVInfo *prevInfo = nullptr;
  for (; j < m_affMVListSize; j++)
  {
    prevInfo = m_affMVList + ((m_affMVListIdx - j - 1 + m_affMVListMaxSize) % (m_affMVListMaxSize));
    if ((tmpMVInfo.x == prevInfo->x) && (tmpMVInfo.y == prevInfo->y) && (tmpMVInfo.w == prevInfo->w) &&
        (tmpMVInfo.h == prevInfo->h))
    {
      break;
    }
  }
  if (j < m_affMVListSize)
  {
    *prevInfo = tmpMVInfo;
  }
  else
  {
    m_affMVList[m_affMVListIdx] = tmpMVInfo;
    m_affMVListIdx              = (m_affMVListIdx + 1) % m_affMVListMaxSize;
    m_affMVListSize             = std::min(m_affMVListSize + 1, m_affMVListMaxSize);
  }
}

void InterSearch::insertUniMvCands(const Area &blkArea, RefSetArray<Mv> &cMvTemp)
{
  BlkUniMvInfo *curMvInfo = m_uniMvList + m_uniMvListIdx;
  int           j         = 0;
  for (; j < m_uniMvListSize; j++)
  {
    const BlkUniMvInfo *prevMvInfo =
      m_uniMvList + ((m_uniMvListIdx - 1 - j + m_uniMvListMaxSize) % (m_uniMvListMaxSize));
    if ((blkArea.x == prevMvInfo->x) && (blkArea.y == prevMvInfo->y) && (blkArea.width == prevMvInfo->w) &&
        (blkArea.height == prevMvInfo->h))
    {
      break;
    }
  }

  if (j < m_uniMvListSize)
  {
    curMvInfo = m_uniMvList + ((m_uniMvListIdx - 1 - j + m_uniMvListMaxSize) % (m_uniMvListMaxSize));
  }

  ::memcpy(curMvInfo->uniMvs, cMvTemp, sizeof(cMvTemp));
  if (j == m_uniMvListSize)  // new element
  {
    curMvInfo->x    = blkArea.x;
    curMvInfo->y    = blkArea.y;
    curMvInfo->w    = blkArea.width;
    curMvInfo->h    = blkArea.height;
    m_uniMvListSize = std::min(m_uniMvListSize + 1, m_uniMvListMaxSize);
    m_uniMvListIdx  = (m_uniMvListIdx + 1) % (m_uniMvListMaxSize);
  }
}

void InterSearch::savePrevUniMvInfo(CompArea blkArea, BlkUniMvInfo &tmpUniMvInfo, bool &isUniMvInfoSaved)
{
  int           j            = 0;
  BlkUniMvInfo *curUniMvInfo = nullptr;
  for (; j < m_uniMvListSize; j++)
  {
    curUniMvInfo = m_uniMvList + ((m_uniMvListIdx - 1 - j + m_uniMvListMaxSize) % (m_uniMvListMaxSize));
    if ((blkArea.x == curUniMvInfo->x) && (blkArea.y == curUniMvInfo->y) && (blkArea.width == curUniMvInfo->w) &&
        (blkArea.height == curUniMvInfo->h))
    {
      break;
    }
  }

  if (j < m_uniMvListSize)
  {
    isUniMvInfoSaved = true;
    tmpUniMvInfo     = *curUniMvInfo;
  }
}

void InterSearch::addUniMvInfo(BlkUniMvInfo &tmpUniMVInfo)
{
  int           j             = 0;
  BlkUniMvInfo *prevUniMvInfo = nullptr;
  for (; j < m_uniMvListSize; j++)
  {
    prevUniMvInfo = m_uniMvList + ((m_uniMvListIdx - 1 - j + m_uniMvListMaxSize) % (m_uniMvListMaxSize));
    if ((tmpUniMVInfo.x == prevUniMvInfo->x) && (tmpUniMVInfo.y == prevUniMvInfo->y) &&
        (tmpUniMVInfo.w == prevUniMvInfo->w) && (tmpUniMVInfo.h == prevUniMvInfo->h))
    {
      break;
    }
  }
  if (j < m_uniMvListSize)
  {
    *prevUniMvInfo = tmpUniMVInfo;
  }
  else
  {
    m_uniMvList[m_uniMvListIdx] = tmpUniMVInfo;
    m_uniMvListIdx              = (m_uniMvListIdx + 1) % m_uniMvListMaxSize;
    m_uniMvListSize             = std::min(m_uniMvListSize + 1, m_uniMvListMaxSize);
  }
}

inline void InterSearch::xTZSearchHelp(IntTZSearchStruct &rcStruct, const int iSearchX, const int iSearchY,
                                       const uint8_t ucPointNr, const uint32_t uiDistance)
{
  Distortion uiSad = 0;

//  CHECK(!( !( rcStruct.searchRange.left > iSearchX || rcStruct.searchRange.right < iSearchX ||
//  rcStruct.searchRange.top > iSearchY || rcStruct.searchRange.bottom < iSearchY )), "Unspecified error");

  const Pel *const piRefSrch = rcStruct.piRefY + iSearchY * rcStruct.iRefStride + iSearchX;

  m_cDistParam.cur.buf = piRefSrch;

  if (1 == rcStruct.subShiftMode)
  {
    // motion cost
    Distortion uiBitCost = m_pcRdCost->getCostOfVectorWithPredictor(iSearchX, iSearchY, rcStruct.imvShift);

    // Skip search if bit cost is already larger than best SAD
    if (uiBitCost < rcStruct.uiBestSad)
    {
      Distortion uiTempSad = m_cDistParam.distFunc(m_cDistParam);

      if ((uiTempSad + uiBitCost) < rcStruct.uiBestSad)
      {
        // it's not supposed that any member of DistParams is manipulated beside cur.buf
        int        subShift = m_cDistParam.subShift;
        const Pel *pOrgCpy  = m_cDistParam.org.buf;
        uiSad += uiTempSad >> m_cDistParam.subShift;

        while (m_cDistParam.subShift > 0)
        {
          int isubShift        = m_cDistParam.subShift - 1;
          m_cDistParam.org.buf = rcStruct.pcPatternKey->buf + (rcStruct.pcPatternKey->stride << isubShift);
          m_cDistParam.cur.buf = piRefSrch + (rcStruct.iRefStride << isubShift);
          uiTempSad            = m_cDistParam.distFunc(m_cDistParam);
          uiSad += uiTempSad >> m_cDistParam.subShift;

          if (((uiSad << isubShift) + uiBitCost) > rcStruct.uiBestSad)
          {
            break;
          }

          m_cDistParam.subShift--;
        }

        if (m_cDistParam.subShift == 0)
        {
          uiSad += uiBitCost;

          if (uiSad < rcStruct.uiBestSad)
          {
            rcStruct.uiBestSad                         = uiSad;
            rcStruct.iBestX                            = iSearchX;
            rcStruct.iBestY                            = iSearchY;
            rcStruct.uiBestDistance                    = uiDistance;
            rcStruct.uiBestRound                       = 0;
            rcStruct.ucPointNr                         = ucPointNr;
            m_cDistParam.maximumDistortionForEarlyExit = uiSad;
          }
        }

        // restore org ptr
        m_cDistParam.org.buf  = pOrgCpy;
        m_cDistParam.subShift = subShift;
      }
    }
  }
  else
  {
    uiSad = m_cDistParam.distFunc(m_cDistParam);

    // only add motion cost if uiSad is smaller than best. Otherwise pointless
    // to add motion cost.
    if (uiSad < rcStruct.uiBestSad)
    {
      // motion cost
      uiSad += m_pcRdCost->getCostOfVectorWithPredictor(iSearchX, iSearchY, rcStruct.imvShift);

      if (uiSad < rcStruct.uiBestSad)
      {
        rcStruct.uiBestSad                         = uiSad;
        rcStruct.iBestX                            = iSearchX;
        rcStruct.iBestY                            = iSearchY;
        rcStruct.uiBestDistance                    = uiDistance;
        rcStruct.uiBestRound                       = 0;
        rcStruct.ucPointNr                         = ucPointNr;
        m_cDistParam.maximumDistortionForEarlyExit = uiSad;
      }
    }
  }
}

inline void InterSearch::xTZ2PointSearch(IntTZSearchStruct &rcStruct)
{
  const SearchRange &sr = rcStruct.searchRange;

  static const int xOffset[2][9] = { { 0, -1, -1, 0, -1, +1, -1, -1, +1 }, { 0, 0, +1, +1, -1, +1, 0, +1, 0 } };
  static const int yOffset[2][9] = { { 0, 0, -1, -1, +1, -1, 0, +1, 0 }, { 0, -1, -1, 0, -1, +1, +1, +1, +1 } };

  // 2 point search,                   //   1 2 3
  // check only the 2 untested points  //   4 0 5
  // around the start point            //   6 7 8
  const int iX1 = rcStruct.iBestX + xOffset[0][rcStruct.ucPointNr];
  const int iX2 = rcStruct.iBestX + xOffset[1][rcStruct.ucPointNr];

  const int iY1 = rcStruct.iBestY + yOffset[0][rcStruct.ucPointNr];
  const int iY2 = rcStruct.iBestY + yOffset[1][rcStruct.ucPointNr];

  if (iX1 >= sr.left && iX1 <= sr.right && iY1 >= sr.top && iY1 <= sr.bottom)
  {
    xTZSearchHelp(rcStruct, iX1, iY1, 0, 2);
  }

  if (iX2 >= sr.left && iX2 <= sr.right && iY2 >= sr.top && iY2 <= sr.bottom)
  {
    xTZSearchHelp(rcStruct, iX2, iY2, 0, 2);
  }
}

inline void InterSearch::xTZ8PointSquareSearch(IntTZSearchStruct &rcStruct, const int iStartX, const int iStartY,
                                               const int iDist)
{
  const SearchRange &sr = rcStruct.searchRange;
  // 8 point search,                   //   1 2 3
  // search around the start point     //   4 0 5
  // with the required  distance       //   6 7 8
  CHECK(iDist == 0, "Invalid distance");
  const int iTop    = iStartY - iDist;
  const int iBottom = iStartY + iDist;
  const int iLeft   = iStartX - iDist;
  const int iRight  = iStartX + iDist;
  rcStruct.uiBestRound += 1;

  if (iTop >= sr.top) // check top
  {
    if (iLeft >= sr.left) // check top left
    {
      xTZSearchHelp(rcStruct, iLeft, iTop, 1, iDist);
    }
    // top middle
    xTZSearchHelp(rcStruct, iStartX, iTop, 2, iDist);

    if (iRight <= sr.right) // check top right
    {
      xTZSearchHelp(rcStruct, iRight, iTop, 3, iDist);
    }
  } // check top
  if (iLeft >= sr.left) // check middle left
  {
    xTZSearchHelp(rcStruct, iLeft, iStartY, 4, iDist);
  }
  if (iRight <= sr.right) // check middle right
  {
    xTZSearchHelp(rcStruct, iRight, iStartY, 5, iDist);
  }
  if (iBottom <= sr.bottom) // check bottom
  {
    if (iLeft >= sr.left) // check bottom left
    {
      xTZSearchHelp(rcStruct, iLeft, iBottom, 6, iDist);
    }
    // check bottom middle
    xTZSearchHelp(rcStruct, iStartX, iBottom, 7, iDist);

    if (iRight <= sr.right) // check bottom right
    {
      xTZSearchHelp(rcStruct, iRight, iBottom, 8, iDist);
    }
  } // check bottom
}

inline void InterSearch::xTZ8PointDiamondSearch(IntTZSearchStruct &rcStruct, const int iStartX, const int iStartY,
                                                const int iDist, const bool bCheckCornersAtDist1)
{
  const SearchRange &sr = rcStruct.searchRange;
  // 8 point search,                   //   1 2 3
  // search around the start point     //   4 0 5
  // with the required  distance       //   6 7 8
  CHECK(iDist == 0, "Invalid distance");
  const int iTop    = iStartY - iDist;
  const int iBottom = iStartY + iDist;
  const int iLeft   = iStartX - iDist;
  const int iRight  = iStartX + iDist;
  rcStruct.uiBestRound += 1;

  if (iDist == 1)
  {
    if (iTop >= sr.top) // check top
    {
      if (bCheckCornersAtDist1)
      {
        if (iLeft >= sr.left) // check top-left
        {
          xTZSearchHelp(rcStruct, iLeft, iTop, 1, iDist);
        }
        xTZSearchHelp(rcStruct, iStartX, iTop, 2, iDist);
        if (iRight <= sr.right) // check middle right
        {
          xTZSearchHelp(rcStruct, iRight, iTop, 3, iDist);
        }
      }
      else
      {
        xTZSearchHelp(rcStruct, iStartX, iTop, 2, iDist);
      }
    }
    if (iLeft >= sr.left) // check middle left
    {
      xTZSearchHelp(rcStruct, iLeft, iStartY, 4, iDist);
    }
    if (iRight <= sr.right) // check middle right
    {
      xTZSearchHelp(rcStruct, iRight, iStartY, 5, iDist);
    }
    if (iBottom <= sr.bottom) // check bottom
    {
      if (bCheckCornersAtDist1)
      {
        if (iLeft >= sr.left) // check top-left
        {
          xTZSearchHelp(rcStruct, iLeft, iBottom, 6, iDist);
        }
        xTZSearchHelp(rcStruct, iStartX, iBottom, 7, iDist);
        if (iRight <= sr.right) // check middle right
        {
          xTZSearchHelp(rcStruct, iRight, iBottom, 8, iDist);
        }
      }
      else
      {
        xTZSearchHelp(rcStruct, iStartX, iBottom, 7, iDist);
      }
    }
  }
  else
  {
    if (iDist <= 8)
    {
      const int iTop2    = iStartY - (iDist >> 1);
      const int iBottom2 = iStartY + (iDist >> 1);
      const int iLeft2   = iStartX - (iDist >> 1);
      const int iRight2  = iStartX + (iDist >> 1);

      if (iTop >= sr.top && iLeft >= sr.left && iRight <= sr.right && iBottom <= sr.bottom) // check border
      {
        xTZSearchHelp(rcStruct, iStartX, iTop, 2, iDist);
        xTZSearchHelp(rcStruct, iLeft2, iTop2, 1, iDist >> 1);
        xTZSearchHelp(rcStruct, iRight2, iTop2, 3, iDist >> 1);
        xTZSearchHelp(rcStruct, iLeft, iStartY, 4, iDist);
        xTZSearchHelp(rcStruct, iRight, iStartY, 5, iDist);
        xTZSearchHelp(rcStruct, iLeft2, iBottom2, 6, iDist >> 1);
        xTZSearchHelp(rcStruct, iRight2, iBottom2, 8, iDist >> 1);
        xTZSearchHelp(rcStruct, iStartX, iBottom, 7, iDist);
      }
      else // check border
      {
        if (iTop >= sr.top) // check top
        {
          xTZSearchHelp(rcStruct, iStartX, iTop, 2, iDist);
        }
        if (iTop2 >= sr.top) // check half top
        {
          if (iLeft2 >= sr.left) // check half left
          {
            xTZSearchHelp(rcStruct, iLeft2, iTop2, 1, (iDist >> 1));
          }
          if (iRight2 <= sr.right) // check half right
          {
            xTZSearchHelp(rcStruct, iRight2, iTop2, 3, (iDist >> 1));
          }
        } // check half top
        if (iLeft >= sr.left) // check left
        {
          xTZSearchHelp(rcStruct, iLeft, iStartY, 4, iDist);
        }
        if (iRight <= sr.right) // check right
        {
          xTZSearchHelp(rcStruct, iRight, iStartY, 5, iDist);
        }
        if (iBottom2 <= sr.bottom) // check half bottom
        {
          if (iLeft2 >= sr.left) // check half left
          {
            xTZSearchHelp(rcStruct, iLeft2, iBottom2, 6, (iDist >> 1));
          }
          if (iRight2 <= sr.right) // check half right
          {
            xTZSearchHelp(rcStruct, iRight2, iBottom2, 8, (iDist >> 1));
          }
        } // check half bottom
        if (iBottom <= sr.bottom) // check bottom
        {
          xTZSearchHelp(rcStruct, iStartX, iBottom, 7, iDist);
        }
      } // check border
    }
    else // iDist > 8
    {
      if (iTop >= sr.top && iLeft >= sr.left && iRight <= sr.right && iBottom <= sr.bottom) // check border
      {
        xTZSearchHelp(rcStruct, iStartX, iTop, 0, iDist);
        xTZSearchHelp(rcStruct, iLeft, iStartY, 0, iDist);
        xTZSearchHelp(rcStruct, iRight, iStartY, 0, iDist);
        xTZSearchHelp(rcStruct, iStartX, iBottom, 0, iDist);
        for (int index = 1; index < 4; index++)
        {
          const int iPosYT = iTop + ((iDist >> 2) * index);
          const int iPosYB = iBottom - ((iDist >> 2) * index);
          const int iPosXL = iStartX - ((iDist >> 2) * index);
          const int iPosXR = iStartX + ((iDist >> 2) * index);
          xTZSearchHelp(rcStruct, iPosXL, iPosYT, 0, iDist);
          xTZSearchHelp(rcStruct, iPosXR, iPosYT, 0, iDist);
          xTZSearchHelp(rcStruct, iPosXL, iPosYB, 0, iDist);
          xTZSearchHelp(rcStruct, iPosXR, iPosYB, 0, iDist);
        }
      }
      else // check border
      {
        if (iTop >= sr.top) // check top
        {
          xTZSearchHelp(rcStruct, iStartX, iTop, 0, iDist);
        }
        if (iLeft >= sr.left) // check left
        {
          xTZSearchHelp(rcStruct, iLeft, iStartY, 0, iDist);
        }
        if (iRight <= sr.right) // check right
        {
          xTZSearchHelp(rcStruct, iRight, iStartY, 0, iDist);
        }
        if (iBottom <= sr.bottom) // check bottom
        {
          xTZSearchHelp(rcStruct, iStartX, iBottom, 0, iDist);
        }
        for (int index = 1; index < 4; index++)
        {
          const int iPosYT = iTop + ((iDist >> 2) * index);
          const int iPosYB = iBottom - ((iDist >> 2) * index);
          const int iPosXL = iStartX - ((iDist >> 2) * index);
          const int iPosXR = iStartX + ((iDist >> 2) * index);

          if (iPosYT >= sr.top) // check top
          {
            if (iPosXL >= sr.left) // check left
            {
              xTZSearchHelp(rcStruct, iPosXL, iPosYT, 0, iDist);
            }
            if (iPosXR <= sr.right) // check right
            {
              xTZSearchHelp(rcStruct, iPosXR, iPosYT, 0, iDist);
            }
          } // check top
          if (iPosYB <= sr.bottom) // check bottom
          {
            if (iPosXL >= sr.left) // check left
            {
              xTZSearchHelp(rcStruct, iPosXL, iPosYB, 0, iDist);
            }
            if (iPosXR <= sr.right) // check right
            {
              xTZSearchHelp(rcStruct, iPosXR, iPosYB, 0, iDist);
            }
          } // check bottom
        } // for ...
      } // check border
    } // iDist <= 8
  } // iDist == 1
}

Distortion InterSearch::xPatternRefinement(const CPelBuf *pcPatternKey, Mv baseRefMv, int iFrac, Mv &rcMvFrac,
                                           bool bAllowUseOfHadamard)
{
  Distortion dist;
  Distortion distBest   = std::numeric_limits<Distortion>::max();
  uint32_t   directBest = 0;

  Pel *piRefPos;
  int  iRefStride = pcPatternKey->width + 1;
  m_pcRdCost->setDistParam(m_cDistParam, *pcPatternKey, m_filteredBlock[0][0][0], iRefStride, m_lumaClpRng.bd, COMP_Y,
                           0, 1, (m_encCfg->m_bUseHADME && bAllowUseOfHadamard) ? 1 : 0);

  const Mv *pcMvRefine = (iFrac == 2 ? s_acMvRefineH : s_acMvRefineQ);
  for (uint32_t i = 0; i < 9; i++)
  {
    if (m_skipFracME && i > 0)
    {
      break;
    }
    Mv cMvTest = pcMvRefine[i];
    cMvTest += baseRefMv;

    int horVal = cMvTest.getHor() * iFrac;
    int verVal = cMvTest.getVer() * iFrac;
    piRefPos   = m_filteredBlock[verVal & 3][horVal & 3][0];

    if (horVal == 2 && (verVal & 1) == 0)
    {
      piRefPos += 1;
    }
    if ((horVal & 1) == 0 && verVal == 2)
    {
      piRefPos += iRefStride;
    }
    cMvTest = pcMvRefine[i];
    cMvTest += rcMvFrac;

    m_cDistParam.cur.buf = piRefPos;

    m_cDistParam.inputDepth = m_cDistParam.bitDepth + 4;
    dist                    = m_cDistParam.distFunc(m_cDistParam);
    dist += m_pcRdCost->getCostOfVectorWithPredictor(cMvTest.getHor(), cMvTest.getVer(), 0);

    if (dist < distBest)
    {
      distBest                                   = dist;
      directBest                                 = i;
      m_cDistParam.maximumDistortionForEarlyExit = dist;
    }
  }

  rcMvFrac = pcMvRefine[directBest];

  return distBest;
}

/// add ibc search functions here

void InterSearch::xIBCSearchMVCandUpdate(Distortion sad, int x, int y, Distortion *sadBestCand,
                                         static_vector<Mv, CHROMA_REFINEMENT_CANDIDATES> &cMVCand)
{
  int j = CHROMA_REFINEMENT_CANDIDATES - 1;

  if (sad < sadBestCand[CHROMA_REFINEMENT_CANDIDATES - 1])
  {
    for (int t = CHROMA_REFINEMENT_CANDIDATES - 1; t >= 0; t--)
    {
      if (sad < sadBestCand[t])
      {
        j = t;
      }
    }

    for (int k = CHROMA_REFINEMENT_CANDIDATES - 1; k > j; k--)
    {
      sadBestCand[k] = sadBestCand[k - 1];

      cMVCand[k].set(cMVCand[k - 1].getHor(), cMVCand[k - 1].getVer());
    }
    sadBestCand[j] = sad;
    cMVCand[j].set(x, y);
  }
}

int InterSearch::xIBCSearchMVChromaRefine(CodingUnit &cu, int roiWidth, int roiHeight, int cuPelX, int cuPelY,
                                          Distortion                                      *sadBestCand,
                                          static_vector<Mv, CHROMA_REFINEMENT_CANDIDATES> &cMVCand

)
{
  if ((!isChromaEnabled(cu.chromaFormat)) || (!cu.Cb().valid()))
  {
    return 0;
  }

  int        bestCandIdx = 0;
  Distortion sadBest     = std::numeric_limits<Distortion>::max();
  Distortion tempSad;

  Pel      *pRef;
  Pel      *pOrg;
  ptrdiff_t refStride, orgStride;
  int       width, height;

  int picWidth  = cu.cs->slice->m_pps->m_picWidthInLumaSamples;
  int picHeight = cu.cs->slice->m_pps->m_picHeightInLumaSamples;

  UnitArea allCompBlocks(cu.chromaFormat, (Area)cu.block(COMP_Y));
  for (int cand = 0; cand < CHROMA_REFINEMENT_CANDIDATES; cand++)
  {
    if (sadBestCand[cand] == std::numeric_limits<Distortion>::max())
    {
      continue;
    }

    if ((!cMVCand[cand].getHor()) && (!cMVCand[cand].getVer()))
    {
      continue;
    }

    if (((int)(cuPelY + cMVCand[cand].getVer() + roiHeight) >= picHeight) || ((cuPelY + cMVCand[cand].getVer()) < 0))
    {
      continue;
    }

    if (((int)(cuPelX + cMVCand[cand].getHor() + roiWidth) >= picWidth) || ((cuPelX + cMVCand[cand].getHor()) < 0))
    {
      continue;
    }

    tempSad = sadBestCand[cand];

    cu.mv[0] = cMVCand[cand];
    cu.mv[0].changePrecision(MvPrecision::ONE, MvPrecision::INTERNAL);
    cu.interDir  = 1;
    cu.refIdx[0] = cu.cs->slice->m_numRefIdx[RPL0]; // last idx in the list

    PelUnitBuf predBufTmp = m_tmpPredStorage[RPL0].getBuf(UnitAreaRelative(cu, cu));
    motionCompensation(cu, predBufTmp, RPL0);

    for (unsigned int ch = COMP_Cb; ch < ::getNumberValidComponents(cu.chromaFormat); ch++)
    {
      width  = roiWidth >> ::getComponentScaleX(CompID(ch), cu.chromaFormat);
      height = roiHeight >> ::getComponentScaleY(CompID(ch), cu.chromaFormat);

      PelUnitBuf  origBuf    = cu.cs->getOrgBuf(allCompBlocks);
      PelUnitBuf *pBuf       = &origBuf;
      CPelBuf     tmpPattern = pBuf->get(CompID(ch));
      pOrg                   = (Pel *)tmpPattern.buf;

      Picture      *refPic = cu.slice->m_pic;
      const CPelBuf refBuf = refPic->getRecoBuf(allCompBlocks.blocks[CompID(ch)]);
      pRef                 = (Pel *)refBuf.buf;

      refStride = refBuf.stride;
      orgStride = tmpPattern.stride;

      // CompID compID = (CompID)ch;
      PelUnitBuf *pBufRef       = &predBufTmp;
      CPelBuf     tmpPatternRef = pBufRef->get(CompID(ch));
      pRef                      = (Pel *)tmpPatternRef.buf;
      refStride                 = tmpPatternRef.stride;

      for (int row = 0; row < height; row++)
      {
        for (int col = 0; col < width; col++)
        {
          tempSad += ((abs(pRef[col] - pOrg[col])) >> (cu.cs->sps->m_bitDepths[ChannelType::CHROMA] - 8));
        }
        pRef += refStride;
        pOrg += orgStride;
      }
    }

    if (tempSad < sadBest)
    {
      sadBest     = tempSad;
      bestCandIdx = cand;
    }
  }

  return bestCandIdx;
}

template<size_t MAX_DST_NUM, size_t MAX_SRC_NUM>
static void xMergeCandLists(static_vector<Mv, MAX_DST_NUM> &dst, const static_vector<Mv, MAX_SRC_NUM> &src)
{
  if (dst.size() < MAX_DST_NUM)
  {
    for (const auto &candSrc: src)
    {
      if (candSrc != Mv() && std::find(dst.begin(), dst.end(), candSrc) == dst.end())
      {
        dst.push_back(candSrc);
        if (dst.size() >= MAX_DST_NUM)
        {
          return;
        }
      }
    }
  }
}

void InterSearch::xIntraPatternSearch(CodingUnit &cu, IntTZSearchStruct &cStruct, Mv &rcMv, Distortion &ruiCost,
                                      Mv *pcMvSrchRngLT, Mv *pcMvSrchRngRB, Mv *pcMvPred)
{
  const int srchRngHorLeft   = pcMvSrchRngLT->getHor();
  const int srchRngHorRight  = pcMvSrchRngRB->getHor();
  const int srchRngVerTop    = pcMvSrchRngLT->getVer();
  const int srchRngVerBottom = pcMvSrchRngRB->getVer();

  const unsigned int lcuWidth     = cu.cs->slice->m_sps->m_maxCuWidth;
  const int          puPelOffsetX = 0;
  const int          puPelOffsetY = 0;
  const int          cuPelX       = cu.lx();
  const int          cuPelY       = cu.ly();

  int roiWidth  = cu.lwidth();
  int roiHeight = cu.lheight();

  Distortion sad;
  Distortion sadBest = std::numeric_limits<Distortion>::max();
  int        bestX   = 0;
  int        bestY   = 0;

  const Pel *piRefSrch = cStruct.piRefY;

  int bestCandIdx = 0;

  Distortion                                      sadBestCand[CHROMA_REFINEMENT_CANDIDATES];
  static_vector<Mv, CHROMA_REFINEMENT_CANDIDATES> cMVCand;

  for (int cand = 0; cand < CHROMA_REFINEMENT_CANDIDATES; cand++)
  {
    sadBestCand[cand] = std::numeric_limits<Distortion>::max();
    cMVCand.push_back(Mv());
  }

  m_cDistParam.useMR = false;
  m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd,
                           COMP_Y, cStruct.subShiftMode);
  m_pcRdCost->setFullPelImvForZeroBvd(!cu.cs->sps->m_ibcFracFlag);

  const int picWidth  = cu.cs->slice->m_pps->m_picWidthInLumaSamples;
  const int picHeight = cu.cs->slice->m_pps->m_picHeightInLumaSamples;

  {
    m_cDistParam.subShift = 0;

    Distortion tempSadBest = 0;

    int srLeft = srchRngHorLeft, srRight = srchRngHorRight, srTop = srchRngVerTop, srBottom = srchRngVerBottom;
    m_acBVs.clear();
    xMergeCandLists(m_acBVs, m_defaultCachedBvs);

    static_vector<Mv, IBC_NUM_CANDIDATES> mvPredEncOnly;
    PU::getIbcMVPsEncOnly(cu, mvPredEncOnly);
    xMergeCandLists(m_acBVs, mvPredEncOnly);

    for (const auto &cand: m_acBVs)
    {
      int xPred = cand.getHor();
      int yPred = cand.getVer();

      if (!(xPred == 0 && yPred == 0) && !((yPred < srTop) || (yPred > srBottom)) &&
          !((xPred < srLeft) || (xPred > srRight)))
      {
        bool validCand = searchBv(cu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, xPred, yPred, lcuWidth);
        if (validCand)
        {
          sad                  = m_pcRdCost->getBvCostMultiplePreds(xPred, yPred, cu.cs->sps->m_AMVREnabledFlag);
          m_cDistParam.cur.buf = piRefSrch + cStruct.iRefStride * yPred + xPred;
          sad += m_cDistParam.distFunc(m_cDistParam);

          xIBCSearchMVCandUpdate(sad, xPred, yPred, sadBestCand, cMVCand);
        }
      }
    }

    bestX = cMVCand[0].getHor();
    bestY = cMVCand[0].getVer();
    rcMv.set(bestX, bestY);
    sadBest = sadBestCand[0];

    const int boundY = (0 - roiHeight - puPelOffsetY);
    for (int y = std::max(srchRngVerTop, 0 - cuPelY); y <= boundY; ++y)
    {
      if (!searchBv(cu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, 0, y, lcuWidth))
      {
        continue;
      }

      sad                  = m_pcRdCost->getBvCostMultiplePreds(0, y, cu.cs->sps->m_AMVREnabledFlag);
      m_cDistParam.cur.buf = piRefSrch + cStruct.iRefStride * y;
      sad += m_cDistParam.distFunc(m_cDistParam);

      xIBCSearchMVCandUpdate(sad, 0, y, sadBestCand, cMVCand);
      tempSadBest = sadBestCand[0];
      if (sadBestCand[0] <= 3)
      {
        bestX   = cMVCand[0].getHor();
        bestY   = cMVCand[0].getVer();
        sadBest = sadBestCand[0];
        rcMv.set(bestX, bestY);
        ruiCost = sadBest;
        goto end;
      }
    }

    const int boundX = std::max(srchRngHorLeft, -cuPelX);
    for (int x = 0 - roiWidth - puPelOffsetX; x >= boundX; --x)
    {
      if (!searchBv(cu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, 0, lcuWidth))
      {
        continue;
      }

      sad                  = m_pcRdCost->getBvCostMultiplePreds(x, 0, cu.cs->sps->m_AMVREnabledFlag);
      m_cDistParam.cur.buf = piRefSrch + x;
      sad += m_cDistParam.distFunc(m_cDistParam);

      xIBCSearchMVCandUpdate(sad, x, 0, sadBestCand, cMVCand);
      tempSadBest = sadBestCand[0];
      if (sadBestCand[0] <= 3)
      {
        bestX   = cMVCand[0].getHor();
        bestY   = cMVCand[0].getVer();
        sadBest = sadBestCand[0];
        rcMv.set(bestX, bestY);
        ruiCost = sadBest;
        goto end;
      }
    }

    bestX   = cMVCand[0].getHor();
    bestY   = cMVCand[0].getVer();
    sadBest = sadBestCand[0];
    if ((!bestX && !bestY) ||
        (sadBest - m_pcRdCost->getBvCostMultiplePreds(bestX, bestY, cu.cs->sps->m_AMVREnabledFlag) <= 32))
    {
      // chroma refine
      bestCandIdx = xIBCSearchMVChromaRefine(cu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand);
      bestX       = cMVCand[bestCandIdx].getHor();
      bestY       = cMVCand[bestCandIdx].getVer();
      sadBest     = sadBestCand[bestCandIdx];
      rcMv.set(bestX, bestY);
      ruiCost = sadBest;
      goto end;
    }

    int blkSize = (m_encCfg->m_ibcFastMethod & IBC_FAST_METHOD_NONSCC) ? 128 : 16;
    if (cu.lwidth() < blkSize && cu.lheight() < blkSize)
    {
      int verTop    = -(int)lcuWidth;
      int verBottom = std::min((int)(lcuWidth >> 2), (int)(lcuWidth - (cuPelY % lcuWidth) - roiHeight));
      int horLeft   = -(int)lcuWidth * 2;
      int horRight  = lcuWidth >> 2;
      if ((m_encCfg->m_ibcFastMethod & IBC_FAST_METHOD_NONSCC) && cu.predMode == MODE_IBC)
      {
        verTop    = bestY - IBC_SEARCH_RANGE;
        verBottom = bestY + IBC_SEARCH_RANGE;
        horLeft   = bestX - IBC_SEARCH_RANGE;
        horRight  = bestX + IBC_SEARCH_RANGE;
      }
      for (int y = std::max(verTop, -cuPelY); y <= verBottom; y += 2)
      {
        if ((y == 0) || ((int)(cuPelY + y + roiHeight) >= picHeight))
        {
          continue;
        }
        for (int x = std::max(horLeft, -cuPelX); x <= horRight; x++)
        {
          if ((x == 0) || ((int)(cuPelX + x + roiWidth) >= picWidth))
          {
            continue;
          }

          if (!searchBv(cu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, y, lcuWidth))
          {
            continue;
          }

          sad                  = m_pcRdCost->getBvCostMultiplePreds(x, y, cu.cs->sps->m_AMVREnabledFlag);
          m_cDistParam.cur.buf = piRefSrch + cStruct.iRefStride * y + x;
          sad += m_cDistParam.distFunc(m_cDistParam);

          xIBCSearchMVCandUpdate(sad, x, y, sadBestCand, cMVCand);
        }
      }

      bestX   = cMVCand[0].getHor();
      bestY   = cMVCand[0].getVer();
      sadBest = sadBestCand[0];
      if (sadBest - m_pcRdCost->getBvCostMultiplePreds(bestX, bestY, cu.cs->sps->m_AMVREnabledFlag) <= 16)
      {
        // chroma refine
        bestCandIdx = xIBCSearchMVChromaRefine(cu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand);

        bestX   = cMVCand[bestCandIdx].getHor();
        bestY   = cMVCand[bestCandIdx].getVer();
        sadBest = sadBestCand[bestCandIdx];
        rcMv.set(bestX, bestY);
        ruiCost = sadBest;
        goto end;
      }

      for (int y = (std::max(verTop, -cuPelY) + 1); y <= verBottom; y += 2)
      {
        if ((y == 0) || ((int)(cuPelY + y + roiHeight) >= picHeight))
        {
          continue;
        }

        for (int x = std::max(horLeft, -cuPelX); x <= horRight; x += 2)
        {
          if ((x == 0) || ((int)(cuPelX + x + roiWidth) >= picWidth))
          {
            continue;
          }

          if (!searchBv(cu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, y, lcuWidth))
          {
            continue;
          }

          sad                  = m_pcRdCost->getBvCostMultiplePreds(x, y, cu.cs->sps->m_AMVREnabledFlag);
          m_cDistParam.cur.buf = piRefSrch + cStruct.iRefStride * y + x;
          sad += m_cDistParam.distFunc(m_cDistParam);

          xIBCSearchMVCandUpdate(sad, x, y, sadBestCand, cMVCand);
          if (sadBestCand[0] <= 5)
          {
            // chroma refine & return
            bestCandIdx = xIBCSearchMVChromaRefine(cu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand);
            bestX       = cMVCand[bestCandIdx].getHor();
            bestY       = cMVCand[bestCandIdx].getVer();
            sadBest     = sadBestCand[bestCandIdx];
            rcMv.set(bestX, bestY);
            ruiCost = sadBest;
            goto end;
          }
        }
      }

      bestX   = cMVCand[0].getHor();
      bestY   = cMVCand[0].getVer();
      sadBest = sadBestCand[0];

      if ((sadBest >= tempSadBest) ||
          ((sadBest - m_pcRdCost->getBvCostMultiplePreds(bestX, bestY, cu.cs->sps->m_AMVREnabledFlag)) <= 32))
      {
        // chroma refine
        bestCandIdx = xIBCSearchMVChromaRefine(cu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand);
        bestX       = cMVCand[bestCandIdx].getHor();
        bestY       = cMVCand[bestCandIdx].getVer();
        sadBest     = sadBestCand[bestCandIdx];
        rcMv.set(bestX, bestY);
        ruiCost = sadBest;
        goto end;
      }

      tempSadBest = sadBestCand[0];

      for (int y = (std::max(verTop, -cuPelY) + 1); y <= verBottom; y += 2)
      {
        if ((y == 0) || ((int)(cuPelY + y + roiHeight) >= picHeight))
        {
          continue;
        }
        for (int x = (std::max(horLeft, -cuPelX) + 1); x <= horRight; x += 2)
        {

          if ((x == 0) || ((int)(cuPelX + x + roiWidth) >= picWidth))
          {
            continue;
          }

          if (!searchBv(cu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, x, y, lcuWidth))
          {
            continue;
          }

          sad                  = m_pcRdCost->getBvCostMultiplePreds(x, y, cu.cs->sps->m_AMVREnabledFlag);
          m_cDistParam.cur.buf = piRefSrch + cStruct.iRefStride * y + x;
          sad += m_cDistParam.distFunc(m_cDistParam);

          xIBCSearchMVCandUpdate(sad, x, y, sadBestCand, cMVCand);
          if (sadBestCand[0] <= 5)
          {
            // chroma refine & return
            bestCandIdx = xIBCSearchMVChromaRefine(cu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand);
            bestX       = cMVCand[bestCandIdx].getHor();
            bestY       = cMVCand[bestCandIdx].getVer();
            sadBest     = sadBestCand[bestCandIdx];
            rcMv.set(bestX, bestY);
            ruiCost = sadBest;
            goto end;
          }
        }
      }
    }
  }

  bestCandIdx = xIBCSearchMVChromaRefine(cu, roiWidth, roiHeight, cuPelX, cuPelY, sadBestCand, cMVCand);

  bestX   = cMVCand[bestCandIdx].getHor();
  bestY   = cMVCand[bestCandIdx].getVer();
  sadBest = sadBestCand[bestCandIdx];
  rcMv.set(bestX, bestY);
  ruiCost = sadBest;

end:
  if (cu.cs->sps->m_ibcFracFlag)
  {
    for (int cand = 0; cand < CHROMA_REFINEMENT_CANDIDATES; cand++)
    {
      if (sadBestCand[cand] == std::numeric_limits<Distortion>::max())
      {
        break;
      }
      m_bestSrchCostIntBv.insert(sadBestCand[cand], cMVCand[cand].getHor(), cMVCand[cand].getVer());
    }
  }
  m_acBVs.clear();
  xMergeCandLists(m_acBVs, m_defaultCachedBvs);

  m_defaultCachedBvs.clear();
  xMergeCandLists(m_defaultCachedBvs, cMVCand);
  xMergeCandLists(m_defaultCachedBvs, m_acBVs);

  for (unsigned int cand = 0; cand < CHROMA_REFINEMENT_CANDIDATES; cand++)
  {
    if (cMVCand[cand].getHor() == 0 && cMVCand[cand].getVer() == 0)
    {
      continue;
    }
    m_ctuRecord[cu.lumaPos()][cu.lumaSize()].bvRecord[cMVCand[cand]] = sadBestCand[cand];
  }

  return;
}

// based on xMotionEstimation
void InterSearch::xIBCEstimation(CodingUnit &cu, PelUnitBuf &origBuf, Mv *pcMvPred, Mv &rcMv, Distortion &ruiCost,
                                 const int localSearchRangeX, const int localSearchRangeY)
{
  const int          iPicWidth  = cu.cs->slice->m_pps->m_picWidthInLumaSamples;
  const int          iPicHeight = cu.cs->slice->m_pps->m_picHeightInLumaSamples;
  const unsigned int lcuWidth   = cu.cs->slice->m_sps->m_maxCuWidth;
  const int          cuPelX     = cu.lx();
  const int          cuPelY     = cu.ly();
  int                iRoiWidth  = cu.lwidth();
  int                iRoiHeight = cu.lheight();

  PelUnitBuf *pBuf = &origBuf;

  //  Search key pattern initialization
  CPelBuf  tmpPattern   = pBuf->Y();
  CPelBuf *pcPatternKey = &tmpPattern;
  PelBuf   tmpOrgLuma;

  if ((cu.cs->slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag))
  {
    const CompArea &area = cu.blocks[COMP_Y];
    CompArea        tmpArea(COMP_Y, area.chromaFormat, Position(0, 0), area.size());
    tmpOrgLuma = m_tmpStorageCtu.getBuf(tmpArea);
    tmpOrgLuma.copyFrom(tmpPattern);
    tmpOrgLuma.rspSignal(m_pcReshape->m_fwdLUT);
    pcPatternKey = (CPelBuf *)&tmpOrgLuma;
  }

  m_lumaClpRng         = cu.cs->slice->clpRng(COMP_Y);
  Picture      *refPic = cu.slice->m_pic;
  const CPelBuf refBuf = refPic->getRecoBuf(cu.blocks[COMP_Y]);

  IntTZSearchStruct cStruct;
  cStruct.pcPatternKey = pcPatternKey;
  cStruct.iRefStride   = refBuf.stride;
  cStruct.piRefY       = refBuf.buf;
  CHECK(cu.imv == IMV_HPEL, "IF_IBC");
  cStruct.imvShift     = cu.imv << 1;
  cStruct.subShiftMode = 0; // used by intra pattern search function

  // disable weighted prediction
  xSetWpScalingDistParam(-1, RPLX, cu.cs->slice);

  m_pcRdCost->getMotionCost(0);
  m_pcRdCost->setPredictors(pcMvPred);
  m_pcRdCost->setCostScale(0);
  m_pcRdCost->setFullPelImvForZeroBvd(!cu.cs->sps->m_ibcFracFlag);

  m_cDistParam.useMR = false;
  m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd,
                           COMP_Y, cStruct.subShiftMode);
  bool buffered = false;
  if (m_encCfg->m_ibcFastMethod & IBC_FAST_METHOD_BUFFERBV)
  {
    ruiCost                                     = MAX_UINT;
    std::unordered_map<Mv, Distortion> &history = m_ctuRecord[cu.lumaPos()][cu.lumaSize()].bvRecord;
    for (std::unordered_map<Mv, Distortion>::iterator p = history.begin(); p != history.end(); p++)
    {
      const Mv &bv = p->first;

      int xBv = bv.hor;
      int yBv = bv.ver;
      if (searchBv(cu, cuPelX, cuPelY, iRoiWidth, iRoiHeight, iPicWidth, iPicHeight, xBv, yBv, lcuWidth))
      {
        buffered             = true;
        Distortion sad       = m_pcRdCost->getBvCostMultiplePreds(xBv, yBv, cu.cs->sps->m_AMVREnabledFlag);
        m_cDistParam.cur.buf = cStruct.piRefY + cStruct.iRefStride * yBv + xBv;
        sad += m_cDistParam.distFunc(m_cDistParam);
        if (sad < ruiCost)
        {
          rcMv    = bv;
          ruiCost = sad;
        }
        else if (sad == ruiCost)
        {
          // stabilise the search through the unordered list
          if (bv.hor < rcMv.getHor() || (bv.hor == rcMv.getHor() && bv.ver < rcMv.getVer()))
          {
            // update the vector.
            rcMv = bv;
          }
        }
      }
    }

    if (buffered)
    {
      static_vector<Mv, IBC_NUM_CANDIDATES> mvPredEncOnly;
      PU::getIbcMVPsEncOnly(cu, mvPredEncOnly);

      for (const auto &cand: mvPredEncOnly)
      {
        int xPred = cand.getHor();
        int yPred = cand.getVer();

        if (searchBv(cu, cuPelX, cuPelY, iRoiWidth, iRoiHeight, iPicWidth, iPicHeight, xPred, yPred, lcuWidth))
        {
          Distortion sad       = m_pcRdCost->getBvCostMultiplePreds(xPred, yPred, cu.cs->sps->m_AMVREnabledFlag);
          m_cDistParam.cur.buf = cStruct.piRefY + cStruct.iRefStride * yPred + xPred;
          sad += m_cDistParam.distFunc(m_cDistParam);
          if (sad < ruiCost)
          {
            rcMv.set(xPred, yPred);
            ruiCost = sad;
          }
          else if (sad == ruiCost)
          {
            // stabilise the search through the unordered list
            if (xPred < rcMv.getHor() || (xPred == rcMv.getHor() && yPred < rcMv.getVer()))
            {
              // update the vector.
              rcMv.set(xPred, yPred);
            }
          }

          m_ctuRecord[cu.lumaPos()][cu.lumaSize()].bvRecord[Mv(xPred, yPred)] = sad;
        }
      }
    }
  }

  if (!buffered)
  {
    Mv cMvSrchRngLT;
    Mv cMvSrchRngRB;

    // assume that intra BV is integer-pel precision
    xSetIntraSearchRange(cu, cu.lwidth(), cu.lheight(), localSearchRangeX, localSearchRangeY, cMvSrchRngLT,
                         cMvSrchRngRB);

    //  Do integer search
    xIntraPatternSearch(cu, cStruct, rcMv, ruiCost, &cMvSrchRngLT, &cMvSrchRngRB, pcMvPred);
  }
}

// based on xSetSearchRange
void InterSearch::xSetIntraSearchRange(CodingUnit &cu, int iRoiWidth, int iRoiHeight, const int localSearchRangeX,
                                       const int localSearchRangeY, Mv &rcMvSrchRngLT, Mv &rcMvSrchRngRB)
{
  const SPS &sps = *cu.cs->sps;

  int srLeft, srRight, srTop, srBottom;

  const int cuPelX = cu.lx();
  const int cuPelY = cu.ly();

  const int lcuWidth = cu.cs->slice->m_sps->m_maxCuWidth;
  const int picWidth = cu.cs->slice->m_pps->m_picWidthInLumaSamples;

  srLeft = -cuPelX;
  srTop  = -2 * lcuWidth - (cuPelY % lcuWidth);
  if (256 == lcuWidth)
  {
    srTop = -lcuWidth - (cuPelY % lcuWidth);
  }
  srRight  = picWidth - cuPelX - iRoiWidth;
  srBottom = lcuWidth - (cuPelY % lcuWidth) - iRoiHeight;
  rcMvSrchRngLT.setHor(srLeft);
  rcMvSrchRngLT.setVer(srTop);
  rcMvSrchRngRB.setHor(srRight);
  rcMvSrchRngRB.setVer(srBottom);

  rcMvSrchRngLT <<= 2;
  rcMvSrchRngRB <<= 2;
  bool temp        = m_clipMvInSubPic;
  m_clipMvInSubPic = true;
  xClipMv(rcMvSrchRngLT, cu.lumaPos(), cu.lumaSize(), sps, *cu.cs->pps);
  xClipMv(rcMvSrchRngRB, cu.lumaPos(), cu.lumaSize(), sps, *cu.cs->pps);
  m_clipMvInSubPic = temp;
  rcMvSrchRngLT >>= 2;
  rcMvSrchRngRB >>= 2;
}

void InterSearch::xEstBvdBitCosts(EstBvdBitsStruct *p, unsigned useIBCFrac)
{
  const FracBitsAccess &fracBits = m_CABACEstimator->getCtx().getFracBitsAcess();

  p->bitsGt0FlagH[0] = fracBits.getFracBitsArray(Ctx::Bvd(HOR_BVD_CTX_OFFSET)).intBits[0];
  p->bitsGt0FlagH[1] = fracBits.getFracBitsArray(Ctx::Bvd(HOR_BVD_CTX_OFFSET)).intBits[1];
  ;

  p->bitsGt0FlagV[0] = fracBits.getFracBitsArray(Ctx::Bvd(VER_BVD_CTX_OFFSET)).intBits[0];
  p->bitsGt0FlagV[1] = fracBits.getFracBitsArray(Ctx::Bvd(VER_BVD_CTX_OFFSET)).intBits[1];

  const int epBitCost  = 1 << SCALE_BITS;
  const int horCtxThre = NUM_HOR_BVD_CTX;
  const int verCtxThre = NUM_VER_BVD_CTX;

  const int horCtxOs = HOR_BVD_CTX_OFFSET;
  const int verCtxOs = VER_BVD_CTX_OFFSET;

  uint32_t singleBitH[2];
  uint32_t singleBitV[2];
  int      bitsX = 0, bitsY = 0;

  for (int i = 0; i < BVD_IBC_MAX_PREFIX; i++)
  {
    if (i < horCtxThre)
    {
      const BinFracBits fracBitsPar = fracBits.getFracBitsArray(Ctx::Bvd(horCtxOs + i + 1));
      singleBitH[0]                 = fracBitsPar.intBits[0];
      singleBitH[1]                 = fracBitsPar.intBits[1];
    }
    else
    {
      singleBitH[0] = epBitCost;
      singleBitH[1] = epBitCost;
    }
    p->bitsH[i] = bitsX + singleBitH[0] + (i + BVD_CODING_GOLOMB_ORDER) * epBitCost;
    bitsX += singleBitH[1];
  }

  for (int i = 0; i < BVD_IBC_MAX_PREFIX; i++)
  {
    if (i < verCtxThre)
    {
      const BinFracBits fracBitsPar = fracBits.getFracBitsArray(Ctx::Bvd(verCtxOs + i + 1));
      singleBitV[0]                 = fracBitsPar.intBits[0];
      singleBitV[1]                 = fracBitsPar.intBits[1];
    }
    else
    {
      singleBitV[0] = epBitCost;
      singleBitV[1] = epBitCost;
    }
    p->bitsV[i] = bitsY + singleBitV[0] + (i + BVD_CODING_GOLOMB_ORDER) * epBitCost;
    bitsY += singleBitV[1];
  }
  const CtxSet &imvCtx = useIBCFrac ? Ctx::ImvFlagIBC : Ctx::ImvFlag;

  p->bitsIdx[0]            = fracBits.getFracBitsArray(Ctx::MVPIdx()).intBits[0];
  p->bitsIdx[1]            = fracBits.getFracBitsArray(Ctx::MVPIdx()).intBits[1];
  p->bitsImv[0]            = fracBits.getFracBitsArray(imvCtx(1)).intBits[0];
  p->bitsImv[1]            = fracBits.getFracBitsArray(imvCtx(1)).intBits[1];
  p->bitsFracImv[IMV_OFF]  = fracBits.getFracBitsArray(imvCtx(0)).intBits[0];
  p->bitsFracImv[IMV_FPEL] = fracBits.getFracBitsArray(imvCtx(0)).intBits[1] + p->bitsImv[0];
  p->bitsFracImv[IMV_4PEL] = fracBits.getFracBitsArray(imvCtx(0)).intBits[1] + p->bitsImv[1];
  p->bitsFracImv[IMV_HPEL] = std::numeric_limits<uint32_t>::max();
}

bool InterSearch::predIBCSearch(CodingUnit &cu, Partitioner &partitioner, const int localSearchRangeX,
                                const int localSearchRangeY, IbcHashMap &ibcHashMap)
{
  Mv cMvSrchRngLT;
  Mv cMvSrchRngRB;

  Mv cMv;
  Mv cMvPred;

  xEstBvdBitCosts(m_pcRdCost->getBvdBitCosts(), cu.cs->sps->m_ibcFracFlag);

  uint8_t imvForZeroMvd = (cu.cs->sps->m_ibcFracFlag ? IBC_SUBPEL_AMVR_MODE_FOR_ZERO_MVD : IMV_FPEL);

  {
    m_maxCompIDToPred = MAX_NUM_COMP;
    m_bestSrchCostIntBv.init(false, cu.cs->sps->m_ibcFracFlag);

    //////////////////////////////////////////////////////////
    /// ibc search
    AMVPInfo amvpInfo, amvpInfo4Pel, amvpInfoQPel, amvpInfoHPel;
    Mv       cMv, cMvPred[2];

    if (cu.cs->sps->m_ibcFracFlag)
    {
      cu.imv = IMV_OFF;
      PU::fillIBCMvpCand(cu, amvpInfoQPel);
    }

    cu.imv = 2;
    PU::fillIBCMvpCand(cu, amvpInfo4Pel);

    cu.imv = 1;
    PU::fillIBCMvpCand(cu, amvpInfo);
    // store in full pel accuracy, shift before use in search
    cMvPred[0] = amvpInfo.mvCand[0];
    cMvPred[0].changePrecision(MvPrecision::INTERNAL, MvPrecision::ONE);
    cMvPred[1] = amvpInfo.mvCand[1];
    cMvPred[1].changePrecision(MvPrecision::INTERNAL, MvPrecision::ONE);

    int iBvpNum    = 2;
    int bvpIdxBest = 0;
    cMv.setZero();
    Distortion cost = 0;
    if (cu.cs->sps->m_maxNumIBCMergeCand == 1)
    {
      iBvpNum    = 1;
      cMvPred[1] = cMvPred[0];
    }

    AMVPInfo *amvpInfoList[NUM_IMV_MODES] = { &amvpInfoQPel, &amvpInfo, &amvpInfo4Pel, &amvpInfoHPel };

    m_pcRdCost->setFullPelImvForZeroBvd(!cu.cs->sps->m_ibcFracFlag);

    if (m_encCfg->m_ibcHashSearch)
    {
      xIBCHashSearch(cu, cMvPred, iBvpNum, cMv, bvpIdxBest, ibcHashMap);
    }

    if (cMv.getHor() == 0 && cMv.getVer() == 0)
    {
      // if hash search does not work or is not enabled
      PelUnitBuf origBuf = cu.cs->getOrgBuf(cu);
      xIBCEstimation(cu, origBuf, cMvPred, cMv, cost, localSearchRangeX, localSearchRangeY);
    }

    if (cMv.getHor() == 0 && cMv.getVer() == 0)
    {
      return false;
    }
    /// ibc search
    /////////////////////////////////////////////////////////
    m_pcRdCost->setFullPelImvForZeroBvd(!cu.cs->sps->m_ibcFracFlag);
    m_pcRdCost->setPredictors(cMvPred);
    m_pcRdCost->setCostScale(0);
    m_pcRdCost->getBvCostMultiplePreds(cMv.getHor(), cMv.getVer(), cu.cs->sps->m_AMVREnabledFlag, &cu.imv, &bvpIdxBest);

    Mv curBestBv = cMv;

    cu.bv = cMv; // bv is always at integer accuracy
    cMv.changePrecision(MvPrecision::ONE, MvPrecision::INTERNAL);
    cu.mv[RPL0] = cMv; // store in fractional pel accuracy

    cu.mvpIdx[RPL0] = bvpIdxBest;

    if (cu.imv == 2 && cMv != amvpInfo4Pel.mvCand[bvpIdxBest])
    {
      cu.mvd[RPL0] = cMv - amvpInfo4Pel.mvCand[bvpIdxBest];
    }
    else
    {
      cu.mvd[RPL0] = cMv - amvpInfo.mvCand[bvpIdxBest];
    }

    if (cu.mvd[RPL0] == Mv(0, 0))
    {
      cu.imv   = imvForZeroMvd;
      cu.mv[0] = amvpInfoList[cu.imv][0].mvCand[bvpIdxBest];

      cu.bv = cu.mv[0];
      cu.bv.changePrecision(MvPrecision::INTERNAL,
                            MvPrecision::ONE /*MV_PRECISION_INT*/); // bv is always set at integer precision
    }
    if (cu.imv == 2)
    {
      assert((cMv.getHor() % 16 == 0) && (cMv.getVer() % 16 == 0));
    }

    cu.refIdx[RPL0] = IBC_REF_IDX;

    if (cu.cs->sps->m_ibcFracFlag)
    {
      if (NOT_VALID == m_bestSrchCostIntBv.find(curBestBv.getHor(), curBestBv.getVer()))
      {
        m_bestSrchCostIntBv.insert(0, curBestBv.getHor(), curBestBv.getVer());
      }

      xPredIBCFracPelSearch(cu, m_bestSrchCostIntBv, amvpInfoList[IMV_OFF], amvpInfoList[IMV_HPEL],
                            amvpInfoList[IMV_FPEL], amvpInfoList[IMV_4PEL]);
    }
  }

  return true;
}

template<int N> Distortion InterSearch::xPredIBCFracPelSearch(CodingUnit &pu, SrchCostBv<N> &intBvList,
                                                              AMVPInfo *amvpInfoQPel, AMVPInfo *amvpInfoHPel,
                                                              AMVPInfo *amvpInfoFPel, AMVPInfo *amvpInfo4Pel)
{
  if (intBvList.cnt == 0)
  {
    return std::numeric_limits<Distortion>::max();
  }
  intBvList.cnt = intBvList.enableMultiCandSrch ? intBvList.cnt : 1;

  AMVPInfo *amvpInfoList[NUM_IMV_MODES] = { amvpInfoQPel, amvpInfoFPel, amvpInfo4Pel, amvpInfoHPel };

  // Get original samples
  PelUnitBuf      origBuf = pu.cs->getOrgBuf(pu);
  const CompArea &area    = pu.blocks[COMP_Y];
  CompArea        tmpArea(COMP_Y, area.chromaFormat, Position(0, 0), area.size());

  PelBuf tmpOrgLuma[1];
  {
    if ((pu.cs->slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag))
    {
      tmpOrgLuma[0] = m_tmpStorageCtu.getBuf(tmpArea);
      tmpOrgLuma[0].rspSignal(origBuf.Y(), m_pcReshape->m_fwdLUT);
    }
    else
    {
      tmpOrgLuma[0] = origBuf.Y();
    }
  }

  // Compute cost for integer BV
  uint8_t    imvForZeroMvd = IBC_SUBPEL_AMVR_MODE_FOR_ZERO_MVD;
  PelUnitBuf predBuf       = pu.cs->getPredBuf(pu);
  DistParam  distParam;

  m_pcRdCost->setDistParam(distParam, tmpOrgLuma[0], predBuf.Y().buf, predBuf.Y().stride,
                           pu.cs->sps->m_bitDepths[ChannelType::LUMA], COMP_Y, 0, 1, 0);
  m_pcRdCost->getMotionCost(0);
  m_pcRdCost->setCostScale(0);

  for (int i = 0; i < intBvList.cnt; ++i)
  {
    pu.imv   = IMV_FPEL;
    pu.bv    = intBvList.mvList[i];
    pu.mv[0] = intBvList.mvList[i];
    pu.mv[0].changePrecision(MvPrecision::ONE /*MV_PRECISION_INT*/, MvPrecision::INTERNAL /*MV_PRECISION_INTERNAL*/);

    {
      Position offset = pu.Y().pos().offset(pu.bv.getHor(), pu.bv.getVer());
      CPelBuf  refBuf =
        pu.slice->m_pic->getRecoBuf(CompArea(COMP_Y, pu.chromaFormat, offset, Size(pu.lwidth(), pu.lheight())), false);
      distParam.cur.buf    = refBuf.buf;
      distParam.cur.stride = refBuf.stride;
    }

    {
      int        bvType        = 0;
      uint32_t   addExtraBits  = 1 + bvType;
      bool       has4PelIMV    = (pu.bv.getHor() & 3) == 0 && (pu.bv.getVer() & 3) == 0;
      uint8_t    tempMvpIdx[2] = { 0, 0 };
      Distortion tempBvCost[2] = {
        m_pcRdCost->getBvCostSingle(pu.mv[0], amvpInfoList[IMV_FPEL][bvType], IMV_FPEL, imvForZeroMvd, bvType == 2,
                                    bvType == 1, addExtraBits, tempMvpIdx[0]),
        has4PelIMV ? m_pcRdCost->getBvCostSingle(pu.mv[0], amvpInfoList[IMV_4PEL][bvType], IMV_4PEL, imvForZeroMvd,
                                                 bvType == 2, bvType == 1, addExtraBits, tempMvpIdx[1])
                   : std::numeric_limits<Distortion>::max()
      };

      uint8_t bestIdx         = tempBvCost[1] < tempBvCost[0] ? 1 : 0;
      intBvList.imvList[i]    = bestIdx == 0 ? IMV_FPEL : IMV_4PEL;
      intBvList.mvpIdxList[i] = tempMvpIdx[bestIdx];
      intBvList.costList[i]   = tempBvCost[bestIdx] + distParam.distFunc(distParam);
      intBvList.mvList[i]     = pu.mv[0];
    }
  }
  distParam.cur.buf    = predBuf.Y().buf;
  distParam.cur.stride = predBuf.Y().stride;

  // Frac-pel search
  const static Mv mvOffsetH[] = {
    Mv(-8, 0), Mv(8, 0), Mv(0, -8), Mv(0, 8), Mv(-8, -8), Mv(8, -8), Mv(-8, 8), Mv(8, 8),
  };
  const static Mv mvOffsetQ[] = {
    Mv(-4, 0), Mv(4, 0), Mv(0, -4), Mv(0, 4), Mv(-4, -4), Mv(4, -4), Mv(-4, 4), Mv(4, 4),
  };

  auto ibcFracPelSquareSearch = [&](uint8_t imv, int candIdx, const Mv *mvOffset, int numOffset)
  {
    CHECK(imv != IMV_HPEL && imv != IMV_OFF, "Error on IMV value in IBC fractional-pel search");
    Distortion &bestCost   = intBvList.costList[candIdx];
    uint8_t    &bestImv    = intBvList.imvList[candIdx];
    uint8_t    &bestMvpIdx = intBvList.mvpIdxList[candIdx];
    Mv         &bestMv     = intBvList.mvList[candIdx];

    pu.imv            = imv;
    const Mv centerMv = bestMv;
    for (int j = 0; j < numOffset; ++j)
    {
      pu.mv[0]           = centerMv + mvOffset[j];
      uint32_t validType = checkValidBvPU(pu, COMP_Y, pu.mv[0], true);
      if (validType == IBC_BV_INVALID)
      {
        continue;
      }
      getPredIBCBlk(pu, COMP_Y, pu.slice->m_pic, pu.mv[0], predBuf);

      // Check cost
      uint32_t addExtraBits = 1;
      distParam.org         = tmpOrgLuma[0];
      uint8_t    tempMvpIdx = 0;
      Distortion tempCost   = m_pcRdCost->getBvCostSingle(pu.mv[0], amvpInfoList[imv][0], imv, imvForZeroMvd, false,
                                                          false, addExtraBits, tempMvpIdx);

      tempCost += (tempCost < bestCost ? distParam.distFunc(distParam) : 0);

      if (tempCost < bestCost)
      {
        bestCost   = tempCost;
        bestImv    = imv;
        bestMvpIdx = tempMvpIdx;
        bestMv     = pu.mv[0];
      }
    }
  };

  if (!intBvList.enableMultiCandSrch && intBvList.mvList[intBvList.maxSize] != Mv())
  {
    if (NOT_VALID == intBvList.find(intBvList.mvList[intBvList.maxSize].hor, intBvList.mvList[intBvList.maxSize].ver))
    {
      intBvList.replaceAt(intBvList.maxSize, 1); // Add history best frac BV
      intBvList.cnt = 2;
    }
  }

  for (int i = 0; i < intBvList.cnt; ++i)
  {
    ibcFracPelSquareSearch(IMV_OFF, i, mvOffsetH, 8);
    ibcFracPelSquareSearch(IMV_OFF, i, mvOffsetQ, 8);
  }

  // Find best
  int bestCandIdx = 0;
  if (intBvList.cnt > 1)
  {
    for (int i = 1; i < intBvList.cnt; ++i)
    {
      if (intBvList.costList[i] < intBvList.costList[bestCandIdx])
      {
        bestCandIdx = i;
      }
    }
  }

  if (intBvList.enableMultiCandSrch)
  {
    // stored for next round
    intBvList.replaceAt(bestCandIdx, intBvList.maxSize);

    // Further refinement when best bv is fractional
    if ((intBvList.imvList[bestCandIdx] == IMV_OFF || intBvList.imvList[bestCandIdx] == IMV_HPEL))
    {
      intBvList.replaceAt(bestCandIdx, 0);
      intBvList.cnt = 1;
      bestCandIdx   = 0;

      ibcFracPelSquareSearch(IMV_OFF, 0, mvOffsetH, 8);
      ibcFracPelSquareSearch(IMV_OFF, 0, mvOffsetQ, 8);
    }
  }

  // Set best mode
  int bestBvType = 0;

  pu.imv       = intBvList.imvList[bestCandIdx];
  pu.mvpIdx[0] = intBvList.mvpIdxList[bestCandIdx];
  pu.mv[0]     = intBvList.mvList[bestCandIdx];
  pu.mvd[0]    = pu.mv[0] - amvpInfoList[pu.imv][bestBvType].mvCand[pu.mvpIdx[0]];

  pu.bv = pu.mv[0];
  pu.bv.changePrecision(MvPrecision::INTERNAL /*MV_PRECISION_INTERNAL*/,
                        MvPrecision::ONE /*INTERNALMV_PRECISION_INT*/); // bv is always stored at integer precision

  if (pu.mvd[RPL0] == Mv(0, 0))
  {
    pu.imv   = imvForZeroMvd;
    pu.mv[0] = amvpInfoList[pu.imv][bestBvType].mvCand[pu.mvpIdx[0]];
    pu.bv    = pu.mv[0];
    pu.bv.changePrecision(
      MvPrecision::INTERNAL /*MV_PRECISION_INTERNAL*/,
      MvPrecision::ONE /*INTERNALMV_PRECISION_INT*/); // pu.bv is always stored at integer precision
  }

  pu.refIdx[RPL0] = MAX_NUM_REF;
  return intBvList.costList[bestCandIdx];
}

void InterSearch::xIBCHashSearch(CodingUnit &cu, Mv *mvPred, int numMvPred, Mv &mv, int &idxMvPred,
                                 IbcHashMap &ibcHashMap)
{
  mv.setZero();
  m_pcRdCost->setCostScale(0);

  std::vector<Position> candPos;
  if (ibcHashMap.ibcHashMatch(cu.Y(), candPos, *cu.cs, m_encCfg->m_ibcHashSearchMaxCand,
                              m_encCfg->m_ibcHashSearchRange4SmallBlk))
  {
    unsigned int minCost = MAX_UINT;

    const unsigned int lcuWidth  = cu.cs->slice->m_sps->m_maxCuWidth;
    const int          cuPelX    = cu.lx();
    const int          cuPelY    = cu.ly();
    const int          picWidth  = cu.cs->slice->m_pps->m_picWidthInLumaSamples;
    const int          picHeight = cu.cs->slice->m_pps->m_picHeightInLumaSamples;
    int                roiWidth  = cu.lwidth();
    int                roiHeight = cu.lheight();

    for (std::vector<Position>::iterator pos = candPos.begin(); pos != candPos.end(); pos++)
    {
      Position bottomRight = pos->offset(cu.lwidth() - 1, cu.lheight() - 1);
      if (cu.cs->isDecomp(*pos, ChannelType::LUMA) && cu.cs->isDecomp(bottomRight, ChannelType::LUMA))
      {
        Position tmp = *pos - cu.Y().pos();
        Mv       candMv;
        candMv.set(tmp.x, tmp.y);

        if (!searchBv(cu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, candMv.getHor(), candMv.getVer(),
                      lcuWidth))
        {
          continue;
        }

        for (int n = 0; n < numMvPred; n++)
        {
          m_pcRdCost->setPredictor(mvPred[n]);

          unsigned int cost = m_pcRdCost->getBitsOfVectorWithPredictor(candMv.getHor(), candMv.getVer(), 0);

          if (cost < minCost)
          {
            mv        = candMv;
            idxMvPred = n;
            minCost   = cost;
          }

          int costQuadPel = MAX_UINT;
          if ((candMv.getHor() % 4 == 0) && (candMv.getVer() % 4 == 0) && (cu.cs->sps->m_AMVREnabledFlag))
          {
            Mv  mvPredQuadPel;
            int imvShift = 2;
            int offset   = 1 << (imvShift - 1);

            int x = (mvPred[n].hor + offset - (mvPred[n].hor >= 0)) >> 2;
            int y = (mvPred[n].ver + offset - (mvPred[n].ver >= 0)) >> 2;
            mvPredQuadPel.set(x, y);

            m_pcRdCost->setPredictor(mvPredQuadPel);

            costQuadPel = m_pcRdCost->getBitsOfVectorWithPredictor(candMv.getHor() >> 2, candMv.getVer() >> 2, 0);
          }

          if (costQuadPel < minCost)
          {
            mv        = candMv;
            idxMvPred = n;
            minCost   = costQuadPel;
          }
        }
      }
    }
  }
}

void InterSearch::addToSortList(std::list<BlockHash> &listBlockHash, std::list<int> &listCost, int cost,
                                const BlockHash &blockHash)
{
  std::list<BlockHash>::iterator itBlockHash = listBlockHash.begin();
  std::list<int>::iterator       itCost      = listCost.begin();

  while (itCost != listCost.end())
  {
    if (cost < (*itCost))
    {
      listCost.insert(itCost, cost);
      listBlockHash.insert(itBlockHash, blockHash);
      return;
    }

    ++itCost;
    ++itBlockHash;
  }

  listCost.push_back(cost);
  listBlockHash.push_back(blockHash);
}

void InterSearch::xSelectMatchesInter(const MapIterator &itBegin, int count, std::list<BlockHash> &listBlockHash,
                                      const BlockHash &currBlockHash)
{
  const int maxReturnNumber = Hash::NUM_LOG_BLK_SIZES;

  listBlockHash.clear();
  std::list<int> listCost;
  listCost.clear();

  MapIterator it = itBegin;
  for (int i = 0; i < count; i++, it++)
  {
    if ((*it).hashValue2 != currBlockHash.hashValue2)
    {
      continue;
    }

    int currCost = RdCost::xGetExpGolombNumberOfBits((*it).x - currBlockHash.x) +
      RdCost::xGetExpGolombNumberOfBits((*it).y - currBlockHash.y);

    if (listBlockHash.size() < maxReturnNumber)
    {
      addToSortList(listBlockHash, listCost, currCost, (*it));
    }
    else if (!listCost.empty() && currCost < listCost.back())
    {
      listCost.pop_back();
      listBlockHash.pop_back();
      addToSortList(listBlockHash, listCost, currCost, (*it));
    }
  }
}
void InterSearch::xSelectRectangleMatchesInter(const MapIterator &itBegin, int count,
                                               std::list<BlockHash> &listBlockHash, const BlockHash &currBlockHash,
                                               int width, int height, int idxNonSimple, unsigned int *&hashValues,
                                               int baseNum, int picWidth, int picHeight, bool isHorizontal,
                                               uint16_t *curHashPic)
{
  const int    maxReturnNumber = 5;
  int          baseSize        = std::min(width, height);
  unsigned int crcMask         = 1 << 16;
  crcMask -= 1;

  listBlockHash.clear();
  std::list<int> listCost;
  listCost.clear();

  MapIterator it = itBegin;

  for (int i = 0; i < count; i++, it++)
  {
    if ((*it).hashValue2 != currBlockHash.hashValue2)
    {
      continue;
    }
    int xRef = (*it).x;
    int yRef = (*it).y;
    if (isHorizontal)
    {
      xRef -= idxNonSimple * baseSize;
    }
    else
    {
      yRef -= idxNonSimple * baseSize;
    }
    if (xRef < 0 || yRef < 0 || xRef + width >= picWidth || yRef + height >= picHeight)
    {
      continue;
    }
    // check Other baseSize hash values
    uint16_t *refHashValue = curHashPic + yRef * picWidth + xRef;
    bool      isSame       = true;

    for (int k = 0; k < baseNum; k++)
    {
      if ((*refHashValue) != (uint16_t)(hashValues[k] & crcMask))
      {
        isSame = false;
        break;
      }
      refHashValue += (isHorizontal ? baseSize : (baseSize * picWidth));
    }
    if (!isSame)
    {
      continue;
    }

    int currCost = RdCost::xGetExpGolombNumberOfBits(xRef - currBlockHash.x) +
      RdCost::xGetExpGolombNumberOfBits(yRef - currBlockHash.y);

    BlockHash refBlockHash;
    refBlockHash.hashValue2 = (*it).hashValue2;
    refBlockHash.x          = xRef;
    refBlockHash.y          = yRef;

    if (listBlockHash.size() < maxReturnNumber)
    {
      addToSortList(listBlockHash, listCost, currCost, refBlockHash);
    }
    else if (!listCost.empty() && currCost < listCost.back())
    {
      listCost.pop_back();
      listBlockHash.pop_back();
      addToSortList(listBlockHash, listCost, currCost, refBlockHash);
    }
  }
}

bool InterSearch::xRectHashInterEstimation(CodingUnit &cu, RefPicList &bestRefPicList, int &bestRefIndex, Mv &bestMv,
                                           Mv &bestMvd, int &bestMVPIndex, bool &isPerfectMatch)
{
  int width  = cu.lumaSize().width;
  int height = cu.lumaSize().height;

  int  baseSize     = std::min(width, height);
  bool isHorizontal = true;
  ;
  int baseNum = 0;
  if (height < width)
  {
    isHorizontal = true;
    baseNum      = 1 << (floorLog2(width) - floorLog2(height));
  }
  else
  {
    isHorizontal = false;
    baseNum      = 1 << (floorLog2(height) - floorLog2(width));
  }

  int             xPos       = cu.lumaPos().x;
  int             yPos       = cu.lumaPos().y;
  const ptrdiff_t currStride = cu.cs->picture->getOrigBuf().get(COMP_Y).stride;
  const Pel      *curPel     = cu.cs->picture->getOrigBuf().get(COMP_Y).buf + yPos * currStride + xPos;
  int             picWidth   = cu.slice->m_pps->m_picWidthInLumaSamples;
  int             picHeight  = cu.slice->m_pps->m_picHeightInLumaSamples;

  int           xBase        = xPos;
  int           yBase        = yPos;
  const Pel    *basePel      = curPel;
  int           idxNonSimple = -1;
  unsigned int *hashValue1s  = new unsigned int[baseNum];
  unsigned int *hashValue2s  = new unsigned int[baseNum];

  for (int k = 0; k < baseNum; k++)
  {
    if (isHorizontal)
    {
      xBase   = xPos + k * baseSize;
      basePel = curPel + k * baseSize;
    }
    else
    {
      yBase   = yPos + k * baseSize;
      basePel = curPel + k * baseSize * currStride;
    }

    if (idxNonSimple == -1 && !Hash::isHorizontalPerfectLuma(basePel, currStride, baseSize, baseSize) &&
        !Hash::isVerticalPerfectLuma(basePel, currStride, baseSize, baseSize))
    {
      idxNonSimple = k;
    }
    Hash::getBlockHashValue((cu.cs->picture->getOrigBuf()), baseSize, baseSize, xBase, yBase,
                            cu.slice->m_sps->m_bitDepths, hashValue1s[k], hashValue2s[k]);
  }
  if (idxNonSimple == -1)
  {
    idxNonSimple = 0;
  }

  Distortion bestCost = UINT64_MAX;

  BlockHash currBlockHash;
  currBlockHash.x = xPos;// still use the first base block location
  currBlockHash.y = yPos;

  currBlockHash.hashValue2 = hashValue2s[idxNonSimple];

  m_pcRdCost->setDistParam(m_cDistParam, cu.cs->getOrgBuf(cu).Y(), 0, 0, m_lumaClpRng.bd, COMP_Y, 0, 1, 0);

  int imvBest    = 0;
  int numPredDir = cu.slice->isInterP() ? 1 : 2;
  for (int refList = 0; refList < numPredDir; refList++)
  {
    RefPicList eRefPicList  = (refList == 0) ? RPL0 : RPL1;
    int        refPicNumber = cu.slice->m_numRefIdx[eRefPicList];

    for (int refIdx = 0; refIdx < refPicNumber; refIdx++)
    {
      int bitsOnRefIdx = 1;
      if (refPicNumber > 1)
      {
        bitsOnRefIdx += refIdx + 1;
        if (refIdx == refPicNumber - 1)
        {
          bitsOnRefIdx--;
        }
      }
      m_numHashMVStoreds[eRefPicList][refIdx] = 0;

      const ScalingRatio &scaleRatio = cu.slice->getScalingRatio(eRefPicList, refIdx);
      if (scaleRatio != SCALE_1X)
      {
        continue;
      }

      if (refList == 0 || cu.slice->m_list1IdxToList0Idx[refIdx] < 0)
      {
        int count =
          static_cast<int>(cu.slice->getRefPic(eRefPicList, refIdx)->m_hashMap.count(hashValue1s[idxNonSimple]));
        if (count == 0)
        {
          continue;
        }

        std::list<BlockHash> listBlockHash;
        xSelectRectangleMatchesInter(
          cu.slice->getRefPic(eRefPicList, refIdx)->m_hashMap.getFirstIterator(hashValue1s[idxNonSimple]), count,
          listBlockHash, currBlockHash, width, height, idxNonSimple, hashValue2s, baseNum, picWidth, picHeight,
          isHorizontal, cu.slice->getRefPic(eRefPicList, refIdx)->m_hashMap.getHashPic(baseSize));

        m_numHashMVStoreds[eRefPicList][refIdx] = int(listBlockHash.size());
        if (listBlockHash.empty())
        {
          continue;
        }
        AMVPInfo currAMVPInfoPel;
        AMVPInfo currAMVPInfo4Pel;
        AMVPInfo currAMVPInfoQPel;

        cu.imv = 2;
        PU::fillMvpCand(cu, eRefPicList, refIdx, currAMVPInfo4Pel);
        cu.imv = 1;
        PU::fillMvpCand(cu, eRefPicList, refIdx, currAMVPInfoPel);
        cu.imv = 0;
        PU::fillMvpCand(cu, eRefPicList, refIdx, currAMVPInfoQPel);
        for (int mvpIdxTemp = 0; mvpIdxTemp < 2; mvpIdxTemp++)
        {
          currAMVPInfoQPel.mvCand[mvpIdxTemp].changePrecision(MvPrecision::INTERNAL, MvPrecision::QUARTER);
          currAMVPInfoPel.mvCand[mvpIdxTemp].changePrecision(MvPrecision::INTERNAL, MvPrecision::QUARTER);
          currAMVPInfo4Pel.mvCand[mvpIdxTemp].changePrecision(MvPrecision::INTERNAL, MvPrecision::QUARTER);
        }

        bool            wrap        = cu.slice->getRefPic(eRefPicList, refIdx)->isWrapAroundEnabled(cu.cs->pps);
        const Pel      *refBufStart = cu.slice->getRefPic(eRefPicList, refIdx)->getRecoBuf(wrap).get(COMP_Y).buf;
        const ptrdiff_t refStride   = cu.slice->getRefPic(eRefPicList, refIdx)->getRecoBuf(wrap).get(COMP_Y).stride;
        m_cDistParam.cur.stride     = refStride;

        m_pcRdCost->selectMotionLambda();
        m_pcRdCost->setCostScale(0);

        std::list<BlockHash>::iterator it;
        int                            countMV = 0;
        for (it = listBlockHash.begin(); it != listBlockHash.end(); ++it)
        {
          int          curMVPIdx  = 0;
          unsigned int curMVPbits = MAX_UINT;
          Mv           cMv((*it).x - currBlockHash.x, (*it).y - currBlockHash.y);
          m_hashMVStoreds[eRefPicList][refIdx][countMV++] = cMv;
          cMv.changePrecision(MvPrecision::ONE, MvPrecision::QUARTER);

          for (int mvpIdxTemp = 0; mvpIdxTemp < 2; mvpIdxTemp++)
          {
            Mv cMvPredPel = currAMVPInfoQPel.mvCand[mvpIdxTemp];
            m_pcRdCost->setPredictor(cMvPredPel);

            unsigned int tempMVPbits = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.getHor(), cMv.getVer(), 0);

            if (tempMVPbits < curMVPbits)
            {
              curMVPbits = tempMVPbits;
              curMVPIdx  = mvpIdxTemp;
              cu.imv     = 0;
            }

            if (cu.slice->m_sps->m_AMVREnabledFlag)
            {
              unsigned int bitsMVP1Pel = MAX_UINT;
              Mv           mvPred1Pel  = currAMVPInfoPel.mvCand[mvpIdxTemp];
              m_pcRdCost->setPredictor(mvPred1Pel);
              bitsMVP1Pel = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.getHor(), cMv.getVer(), 2);

              if (bitsMVP1Pel < curMVPbits)
              {
                curMVPbits = bitsMVP1Pel;
                curMVPIdx  = mvpIdxTemp;
                cu.imv     = 1;
              }

              if ((cMv.getHor() % 16 == 0) && (cMv.getVer() % 16 == 0))
              {
                unsigned int bitsMVP4Pel = MAX_UINT;
                Mv           mvPred4Pel  = currAMVPInfo4Pel.mvCand[mvpIdxTemp];
                m_pcRdCost->setPredictor(mvPred4Pel);
                bitsMVP4Pel = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.getHor(), cMv.getVer(), 4);

                if (bitsMVP4Pel < curMVPbits)
                {
                  curMVPbits = bitsMVP4Pel;
                  curMVPIdx  = mvpIdxTemp;
                  cu.imv     = 2;
                }
              }
            }
          }

          curMVPbits += bitsOnRefIdx;

          m_cDistParam.cur.buf = refBufStart + (*it).y * refStride + (*it).x;
          Distortion currSad   = m_cDistParam.distFunc(m_cDistParam);
          Distortion currCost  = currSad + m_pcRdCost->getCost(curMVPbits);

          if (!isPerfectMatch)
          {
            if (cu.slice->getRefPic(eRefPicList, refIdx)->m_slices[0]->m_iSliceQp <= cu.slice->m_iSliceQp)
            {
              isPerfectMatch = true;
            }
          }

          if (currCost < bestCost)
          {
            bestCost       = currCost;
            bestRefPicList = eRefPicList;
            bestRefIndex   = refIdx;
            bestMv         = cMv;
            bestMVPIndex   = curMVPIdx;
            imvBest        = cu.imv;
            if (cu.imv == 2)
            {
              bestMvd = cMv - currAMVPInfo4Pel.mvCand[curMVPIdx];
            }
            else if (cu.imv == 1)
            {
              bestMvd = cMv - currAMVPInfoPel.mvCand[curMVPIdx];
            }
            else
            {
              bestMvd = cMv - currAMVPInfoQPel.mvCand[curMVPIdx];
            }
          }
        }
      }
    }
  }
  delete[] hashValue1s;
  delete[] hashValue2s;
  cu.imv = imvBest;
  if (bestMvd == Mv(0, 0))
  {
    cu.imv = 0;
    return false;
  }
  return (bestCost < MAX_INT);
}

bool InterSearch::xHashInterEstimation(CodingUnit &cu, RefPicList &bestRefPicList, int &bestRefIndex, Mv &bestMv,
                                       Mv &bestMvd, int &bestMVPIndex, bool &isPerfectMatch)
{
  int width  = cu.lumaSize().width;
  int height = cu.lumaSize().height;
  if (width != height)
  {
    return xRectHashInterEstimation(cu, bestRefPicList, bestRefIndex, bestMv, bestMvd, bestMVPIndex, isPerfectMatch);
  }
  int xPos = cu.lumaPos().x;
  int yPos = cu.lumaPos().y;

  uint32_t   hashValue1;
  uint32_t   hashValue2;
  Distortion bestCost = UINT64_MAX;

  if (!Hash::getBlockHashValue((cu.cs->picture->getOrigBuf()), width, height, xPos, yPos, cu.slice->m_sps->m_bitDepths,
                               hashValue1, hashValue2))
  {
    return false;
  }

  BlockHash currBlockHash;
  currBlockHash.x          = xPos;
  currBlockHash.y          = yPos;
  currBlockHash.hashValue2 = hashValue2;

  m_pcRdCost->setDistParam(m_cDistParam, cu.cs->getOrgBuf(cu).Y(), 0, 0, m_lumaClpRng.bd, COMP_Y, 0, 1, 0);

  int imvBest = 0;

  int numPredDir = cu.slice->isInterP() ? 1 : 2;
  for (int refList = 0; refList < numPredDir; refList++)
  {
    RefPicList eRefPicList  = (refList == 0) ? RPL0 : RPL1;
    int        refPicNumber = cu.slice->m_numRefIdx[eRefPicList];

    for (int refIdx = 0; refIdx < refPicNumber; refIdx++)
    {
      int bitsOnRefIdx = 1;
      if (refPicNumber > 1)
      {
        bitsOnRefIdx += refIdx + 1;
        if (refIdx == refPicNumber - 1)
        {
          bitsOnRefIdx--;
        }
      }
      m_numHashMVStoreds[eRefPicList][refIdx] = 0;

      const ScalingRatio &scaleRatio = cu.slice->getScalingRatio(eRefPicList, refIdx);
      if (scaleRatio != SCALE_1X)
      {
        continue;
      }

      if (refList == 0 || cu.slice->m_list1IdxToList0Idx[refIdx] < 0)
      {
        int count = static_cast<int>(cu.slice->getRefPic(eRefPicList, refIdx)->m_hashMap.count(hashValue1));
        if (count == 0)
        {
          continue;
        }

        std::list<BlockHash> listBlockHash;
        xSelectMatchesInter(cu.slice->getRefPic(eRefPicList, refIdx)->m_hashMap.getFirstIterator(hashValue1), count,
                            listBlockHash, currBlockHash);
        m_numHashMVStoreds[eRefPicList][refIdx] = (int)listBlockHash.size();
        if (listBlockHash.empty())
        {
          continue;
        }
        AMVPInfo currAMVPInfoPel;
        AMVPInfo currAMVPInfo4Pel;
        cu.imv = 2;
        PU::fillMvpCand(cu, eRefPicList, refIdx, currAMVPInfo4Pel);

        cu.imv = 1;
        PU::fillMvpCand(cu, eRefPicList, refIdx, currAMVPInfoPel);
        AMVPInfo currAMVPInfoQPel;
        cu.imv = 0;
        PU::fillMvpCand(cu, eRefPicList, refIdx, currAMVPInfoQPel);
        CHECK(currAMVPInfoPel.numCand <= 1, "Wrong")
        for (int mvpIdxTemp = 0; mvpIdxTemp < 2; mvpIdxTemp++)
        {
          currAMVPInfoQPel.mvCand[mvpIdxTemp].changePrecision(MvPrecision::INTERNAL, MvPrecision::QUARTER);
          currAMVPInfoPel.mvCand[mvpIdxTemp].changePrecision(MvPrecision::INTERNAL, MvPrecision::QUARTER);
          currAMVPInfo4Pel.mvCand[mvpIdxTemp].changePrecision(MvPrecision::INTERNAL, MvPrecision::QUARTER);
        }

        bool            wrap        = cu.slice->getRefPic(eRefPicList, refIdx)->isWrapAroundEnabled(cu.cs->pps);
        const Pel      *refBufStart = cu.slice->getRefPic(eRefPicList, refIdx)->getRecoBuf(wrap).get(COMP_Y).buf;
        const ptrdiff_t refStride   = cu.slice->getRefPic(eRefPicList, refIdx)->getRecoBuf(wrap).get(COMP_Y).stride;

        m_cDistParam.cur.stride = refStride;

        m_pcRdCost->selectMotionLambda();
        m_pcRdCost->setCostScale(0);

        std::list<BlockHash>::iterator it;
        int                            countMV = 0;
        for (it = listBlockHash.begin(); it != listBlockHash.end(); ++it)
        {
          int          curMVPIdx  = 0;
          unsigned int curMVPbits = MAX_UINT;
          Mv           cMv((*it).x - currBlockHash.x, (*it).y - currBlockHash.y);
          m_hashMVStoreds[eRefPicList][refIdx][countMV++] = cMv;
          cMv.changePrecision(MvPrecision::ONE, MvPrecision::QUARTER);

          for (int mvpIdxTemp = 0; mvpIdxTemp < 2; mvpIdxTemp++)
          {
            Mv cMvPredPel = currAMVPInfoQPel.mvCand[mvpIdxTemp];
            m_pcRdCost->setPredictor(cMvPredPel);

            unsigned int tempMVPbits = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.getHor(), cMv.getVer(), 0);

            if (tempMVPbits < curMVPbits)
            {
              curMVPbits = tempMVPbits;
              curMVPIdx  = mvpIdxTemp;
              cu.imv     = 0;
            }

            if (cu.slice->m_sps->m_AMVREnabledFlag)
            {
              unsigned int bitsMVP1Pel = MAX_UINT;
              Mv           mvPred1Pel  = currAMVPInfoPel.mvCand[mvpIdxTemp];
              m_pcRdCost->setPredictor(mvPred1Pel);
              bitsMVP1Pel = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.getHor(), cMv.getVer(), 2);

              if (bitsMVP1Pel < curMVPbits)
              {
                curMVPbits = bitsMVP1Pel;
                curMVPIdx  = mvpIdxTemp;
                cu.imv     = 1;
              }

              if ((cMv.getHor() % 16 == 0) && (cMv.getVer() % 16 == 0))
              {
                unsigned int bitsMVP4Pel = MAX_UINT;
                Mv           mvPred4Pel  = currAMVPInfo4Pel.mvCand[mvpIdxTemp];
                m_pcRdCost->setPredictor(mvPred4Pel);
                bitsMVP4Pel = m_pcRdCost->getBitsOfVectorWithPredictor(cMv.getHor(), cMv.getVer(), 4);

                if (bitsMVP4Pel < curMVPbits)
                {
                  curMVPbits = bitsMVP4Pel;
                  curMVPIdx  = mvpIdxTemp;
                  cu.imv     = 2;
                }
              }
            }
          }

          curMVPbits += bitsOnRefIdx;

          m_cDistParam.cur.buf = refBufStart + (*it).y * refStride + (*it).x;
          Distortion currSad   = m_cDistParam.distFunc(m_cDistParam);
          Distortion currCost  = currSad + m_pcRdCost->getCost(curMVPbits);

          if (!isPerfectMatch)
          {
            if (cu.slice->getRefPic(eRefPicList, refIdx)->m_slices[0]->m_iSliceQp <= cu.slice->m_iSliceQp)
            {
              isPerfectMatch = true;
            }
          }

          if (currCost < bestCost)
          {
            bestCost       = currCost;
            bestRefPicList = eRefPicList;
            bestRefIndex   = refIdx;
            bestMv         = cMv;
            bestMVPIndex   = curMVPIdx;
            imvBest        = cu.imv;
            if (cu.imv == 2)
            {
              bestMvd = cMv - currAMVPInfo4Pel.mvCand[curMVPIdx];
            }
            else if (cu.imv == 1)
            {
              bestMvd = cMv - currAMVPInfoPel.mvCand[curMVPIdx];
            }
            else
            {
              bestMvd = cMv - currAMVPInfoQPel.mvCand[curMVPIdx];
            }
          }
        }
      }
    }
  }
  cu.imv = imvBest;
  if (bestMvd == Mv(0, 0))
  {
    cu.imv = 0;
    return false;
  }
  return (bestCost < MAX_INT);
}

bool InterSearch::predInterHashSearch(CodingUnit &cu, Partitioner &partitioner, bool &isPerfectMatch)
{
  Mv         bestMv, bestMvd;
  RefPicList bestRefPicList;
  int        bestRefIndex;
  int        bestMVPIndex;

  Mv cMvZero;
  cu.mv[RPL0]     = Mv();
  cu.mv[RPL1]     = Mv();
  cu.mvd[RPL0]    = cMvZero;
  cu.mvd[RPL1]    = cMvZero;
  cu.refIdx[RPL0] = NOT_VALID;
  cu.refIdx[RPL1] = NOT_VALID;
  cu.mvpIdx[RPL0] = NOT_VALID;
  cu.mvpIdx[RPL1] = NOT_VALID;
  cu.mvpNum[RPL0] = NOT_VALID;
  cu.mvpNum[RPL1] = NOT_VALID;

  if (xHashInterEstimation(cu, bestRefPicList, bestRefIndex, bestMv, bestMvd, bestMVPIndex, isPerfectMatch))
  {
    cu.interDir           = static_cast<int>(bestRefPicList) + 1;
    cu.mv[bestRefPicList] = bestMv;
    cu.mv[bestRefPicList].changePrecision(MvPrecision::QUARTER, MvPrecision::INTERNAL);

    cu.mvd[bestRefPicList] = bestMvd;
    cu.mvd[bestRefPicList].changePrecision(MvPrecision::QUARTER, MvPrecision::INTERNAL);
    cu.refIdx[bestRefPicList] = bestRefIndex;
    cu.mvpIdx[bestRefPicList] = bestMVPIndex;

    cu.mvpNum[bestRefPicList] = 2;

    PU::spanMotionInfo(cu);
    PelUnitBuf predBuf = cu.cs->getPredBuf(cu);
    motionCompensation(cu, predBuf, RPLX);
    return true;
  }

  return false;
}

//! search of the best candidate for inter prediction
void InterSearch::predInterSearch(CodingUnit &cu, Partitioner &partitioner)
{
  PROFILER_SCOPE(TP_ENABLE_INTER_MVD_SEARCH_STAGES, g_timeProfiler, P_INTER_MVD_SEARCH);
  CodingStructure &cs = *cu.cs;

  AMVPInfo amvp[NUM_RPL01];
  Mv       cMvSrchRngLT;
  Mv       cMvSrchRngRB;

  Mv cMvZero;

  Mv              cMv[NUM_RPL01];
  Mv              cMvBi[NUM_RPL01];
  RefSetArray<Mv> cMvTemp;
  RefSetArray<Mv> cMvHevcTemp;
  int             iNumPredDir = cs.slice->isInterP() ? 1 : 2;

  RefSetArray<Mv> cMvPred;

  RefSetArray<Mv>  cMvPredBi;
  RefSetArray<int> aaiMvpIdxBi;

  RefSetArray<int> aaiMvpIdx;
  RefSetArray<int> aaiMvpNum;

  RefSetArray<AMVPInfo> aacAMVPInfo;

  int refIdx[NUM_RPL01] = { 0, 0 };   // If un-initialized, may cause SEGV in bi-directional prediction iterative stage.
  int iRefIdxBi[NUM_RPL01] = { -1, -1 };

  uint32_t mbBits[3] = { 1, 1, 0 };

  uint32_t uiLastMode     = 0;
  uint32_t uiLastModeTemp = 0;
  int      iRefStart, iRefEnd;

  int symMode = 0;

  int        bestBiPRefIdxL1 = 0;
  int        bestBiPMvpL1    = 0;
  Distortion biPDistTemp     = std::numeric_limits<Distortion>::max();

  uint8_t bcwIdx         = cu.cs->slice->isInterB() ? cu.bcwIdx : BCW_DEFAULT;
  bool    enforceBcwPred = false;

  // Loop over Prediction Units
  uint32_t        puIdx = 0;
  WPScalingParam *wp0;
  WPScalingParam *wp1;
  int             tryBipred      = 0;
  bool            checkAffine    = (cu.imv == 0 || cu.slice->m_sps->m_affineAmvrEnabledFlag) && cu.imv != IMV_HPEL;
  bool            checkNonAffine = cu.imv == 0 || cu.imv == IMV_HPEL ||
    (cu.slice->m_sps->m_AMVREnabledFlag && cu.imv <= (cu.slice->m_sps->m_AMVREnabledFlag ? IMV_4PEL : 0));
  const CodingUnit *bestCU = m_modeCtrl->getBestCU();
  bool trySmvd = (bestCU != nullptr && cu.imv == 2 && checkAffine) ? (!bestCU->mergeFlag && !bestCU->affine) : true;

  if (cu.imv && bestCU != nullptr && checkAffine)
  {
    checkAffine = !(bestCU->mergeFlag || !bestCU->affine);
  }
  constexpr int affineMeTSize = 256;
  if (checkAffine && m_encCfg->m_adaptBypassAffineMe && cu.lumaSize().area() > affineMeTSize)
  {
    constexpr int affineMeTNeighbor = 4;
    int           neighborAvai = 0, neighborAffine = 0;
    PU::getNeighborAffineInfo(cu, neighborAvai, neighborAffine);
    if (neighborAffine == 0 && neighborAvai >= affineMeTNeighbor)
    {
      checkAffine = false;
      if (bestCU != nullptr && bestCU->affine)
      {
        if (!bestCU->mergeFlag || bestCU->mergeType != MergeType::SUBPU_ATMVP)
        {
          checkAffine = !cs.slice->m_meetBiPredT;
        }
      }
    }
  }

  if (cu.imv == 2 && checkNonAffine && cu.slice->m_sps->m_affineAmvrEnabledFlag)
  {
    checkNonAffine = m_affineMotion.hevcCost[1] < m_affineMotion.hevcCost[0];
  }

  amvp[RPL0].numCand = 0;
  amvp[RPL1].numCand = 0;
  memset(aacAMVPInfo, 0, sizeof(aacAMVPInfo));

  {
    m_encMotionEstimation          = true;
    const CodingUnit *parentBestCu = m_modeCtrl->getBestCU(1);
    if (parentBestCu && !parentBestCu->affine)
    {
      m_skipProf = true;
    }
    m_skipProfCond    = !cu.slice->m_checkLdc;
    // motion estimation only evaluates luma component
    m_maxCompIDToPred = MAX_NUM_COMP;
    //    m_maxCompIDToPred = COMP_Y;

    PU::spanMotionInfo(cu);
    Distortion uiHevcCost        = std::numeric_limits<Distortion>::max();
    Distortion uiAffineCost      = std::numeric_limits<Distortion>::max();
    Distortion uiCost[NUM_RPL01] = { std::numeric_limits<Distortion>::max(), std::numeric_limits<Distortion>::max() };
    Distortion costBi            = MAX_DISTORTION;
    Distortion costTemp;

    uint32_t   bits[3];
    uint32_t   bitsTemp;
    Distortion bestBiPDist = std::numeric_limits<Distortion>::max();

    Distortion uiCostTempL0[MAX_NUM_REF];
    for (int iNumRef = 0; iNumRef < MAX_NUM_REF; iNumRef++)
    {
      uiCostTempL0[iNumRef] = std::numeric_limits<Distortion>::max();
    }
    uint32_t uiBitsTempL0[MAX_NUM_REF];

    Mv         mvValidList1;
    int        refIdxValidList1 = 0;
    uint32_t   bitsValidList1   = MAX_UINT;
    Distortion costValidList1   = std::numeric_limits<Distortion>::max();

    PelUnitBuf origBuf = cu.cs->getOrgBuf(cu);

    xGetBlkBits(cs.slice->isInterP(), mbBits);

    m_pcRdCost->selectMotionLambda();

    unsigned imvShift = cu.imv == IMV_HPEL ? 1 : (cu.imv << 1);
    if (checkNonAffine)
    {
      //  Uni-directional prediction
      for (int refList = 0; refList < iNumPredDir; refList++)
      {
        RefPicList eRefPicList = (refList ? RPL1 : RPL0);
        for (int refIdxTemp = 0; refIdxTemp < cs.slice->m_numRefIdx[eRefPicList]; refIdxTemp++)
        {
          bitsTemp = mbBits[refList];
          if (cs.slice->m_numRefIdx[eRefPicList] > 1)
          {
            bitsTemp += refIdxTemp + 1;
            if (refIdxTemp == cs.slice->m_numRefIdx[eRefPicList] - 1)
            {
              bitsTemp--;
            }
          }
          xEstimateMvPredAMVP(cu, origBuf, eRefPicList, refIdxTemp, cMvPred[refList][refIdxTemp], amvp[eRefPicList],
                              false, &biPDistTemp);

          aaiMvpIdx[refList][refIdxTemp] = cu.mvpIdx[eRefPicList];
          aaiMvpNum[refList][refIdxTemp] = cu.mvpNum[eRefPicList];

          if (cs.picHeader->m_mvdL1ZeroFlag && refList == 1 && biPDistTemp < bestBiPDist)
          {
            bestBiPDist     = biPDistTemp;
            bestBiPMvpL1    = aaiMvpIdx[refList][refIdxTemp];
            bestBiPRefIdxL1 = refIdxTemp;
          }

          bitsTemp += m_auiMVPIdxCost[aaiMvpIdx[refList][refIdxTemp]][AMVP_MAX_NUM_CANDS];

          if (m_encCfg->m_bFastMEForGenBLowDelayEnabled && refList == 1)   // list 1
          {
            if (cs.slice->m_list1IdxToList0Idx[refIdxTemp] >= 0)
            {
              cMvTemp[RPL1][refIdxTemp] = cMvTemp[RPL0][cs.slice->m_list1IdxToList0Idx[refIdxTemp]];
              costTemp                  = uiCostTempL0[cs.slice->m_list1IdxToList0Idx[refIdxTemp]];
              /*first subtract the bit-rate part of the cost of the other list*/
              costTemp -= m_pcRdCost->getCost(uiBitsTempL0[cs.slice->m_list1IdxToList0Idx[refIdxTemp]]);
              /*correct the bit-rate part of the current ref*/
              m_pcRdCost->setPredictor(cMvPred[refList][refIdxTemp]);
              bitsTemp += m_pcRdCost->getBitsOfVectorWithPredictor(cMvTemp[RPL1][refIdxTemp].getHor(),
                                                                   cMvTemp[RPL1][refIdxTemp].getVer(),
                                                                   imvShift + MV_FRACTIONAL_BITS_DIFF);
              /*calculate the correct cost*/
              costTemp += m_pcRdCost->getCost(bitsTemp);
            }
            else
            {
              xMotionEstimation(cu, origBuf, eRefPicList, cMvPred[refList][refIdxTemp], refIdxTemp,
                                cMvTemp[refList][refIdxTemp], aaiMvpIdx[refList][refIdxTemp], bitsTemp, costTemp,
                                amvp[eRefPicList]);
            }
          }
          else
          {
            xMotionEstimation(cu, origBuf, eRefPicList, cMvPred[refList][refIdxTemp], refIdxTemp,
                              cMvTemp[refList][refIdxTemp], aaiMvpIdx[refList][refIdxTemp], bitsTemp, costTemp,
                              amvp[eRefPicList]);
          }
          if (cu.cs->sps->m_useBcw && cu.bcwIdx == BCW_DEFAULT && cu.cs->slice->isInterB())
          {
            const bool checkIdentical = true;
            m_uniMotions.setReadMode(checkIdentical, (uint32_t)refList, (uint32_t)refIdxTemp);
            m_uniMotions.copyFrom(cMvTemp[refList][refIdxTemp], costTemp - m_pcRdCost->getCost(bitsTemp),
                                  (uint32_t)refList, (uint32_t)refIdxTemp);
          }
          xCopyAMVPInfo(&amvp[eRefPicList],
                        &aacAMVPInfo[refList][refIdxTemp]);   // must always be done ( also when AMVP_MODE = AM_NONE )
          xCheckBestMVP(eRefPicList, cMvTemp[refList][refIdxTemp], cMvPred[refList][refIdxTemp],
                        aaiMvpIdx[refList][refIdxTemp], amvp[eRefPicList], bitsTemp, costTemp, cu.imv);
          if (refList == 0)
          {
            uiCostTempL0[refIdxTemp] = costTemp;
            uiBitsTempL0[refIdxTemp] = bitsTemp;
          }

          if (costTemp < uiCost[refList])
          {
            uiCost[refList] = costTemp;
            bits[refList]   = bitsTemp;   // storing for bi-prediction

            // set motion
            cMv[refList]    = cMvTemp[refList][refIdxTemp];
            refIdx[refList] = refIdxTemp;
          }

          if (refList == 1 && costTemp < costValidList1 && cs.slice->m_list1IdxToList0Idx[refIdxTemp] < 0)
          {
            costValidList1 = costTemp;
            bitsValidList1 = bitsTemp;

            // set motion
            mvValidList1     = cMvTemp[refList][refIdxTemp];
            refIdxValidList1 = refIdxTemp;
          }
        }
      }

      ::memcpy(cMvHevcTemp, cMvTemp, sizeof(cMvTemp));
      if (cu.imv == 0 && (!cu.slice->m_sps->m_useBcw || bcwIdx == BCW_DEFAULT))
      {
        insertUniMvCands(cu.Y(), cMvTemp);

        unsigned idX, idY, idW, idH;
        getAreaIdx(cu.Y(), *cu.slice->m_pps->pcv, idX, idY, idW, idH);

        {
          const auto reusedUniMVs = cu.licFlag ? m_reusedUniMVsLIC : m_reusedUniMVs;
          static_assert(sizeof(cMvTemp) == sizeof(MvRefSetArray::entry));

          ::memcpy(&(reusedUniMVs[idX][idY][idW][idH]->entry), cMvTemp, sizeof(cMvTemp));
          reusedUniMVs[idX][idY][idW][idH]->valid = true;
        }
      }
      //  Bi-predictive Motion estimation
      if ((cs.slice->isInterB()) && (PU::isBipredRestriction(cu) == false) &&
          (cu.slice->m_checkLdc || bcwIdx == BCW_DEFAULT || !m_affineModeSelected || !m_encCfg->m_BcwFast ||
           (cs.sps->m_biLicEnabledFlag && cu.licFlag && bcwIdx != BCW_DEFAULT)) &&
          (cs.sps->m_biLicEnabledFlag || !cu.licFlag))
      {
        PROFILER_SCOPE(TP_ENABLE_INTER_MVD_SEARCH_STAGES, g_timeProfiler, P_INTER_MVD_SEARCH_B);
        bool doBiPred   = true;
        tryBipred       = 1;
        cMvBi[RPL0]     = cMv[RPL0];
        cMvBi[RPL1]     = cMv[RPL1];
        iRefIdxBi[RPL0] = refIdx[RPL0];
        iRefIdxBi[RPL1] = refIdx[RPL1];

        ::memcpy(cMvPredBi, cMvPred, sizeof(cMvPred));
        ::memcpy(aaiMvpIdxBi, aaiMvpIdx, sizeof(aaiMvpIdx));

        uint32_t motBits[NUM_RPL01];

        if (cs.picHeader->m_mvdL1ZeroFlag)
        {
          xCopyAMVPInfo(&aacAMVPInfo[RPL1][bestBiPRefIdxL1], &amvp[RPL1]);
          aaiMvpIdxBi[RPL1][bestBiPRefIdxL1] = bestBiPMvpL1;
          cMvPredBi[RPL1][bestBiPRefIdxL1]   = amvp[RPL1].mvCand[bestBiPMvpL1];

          cMvBi[RPL1]     = cMvPredBi[RPL1][bestBiPRefIdxL1];
          iRefIdxBi[RPL1] = bestBiPRefIdxL1;
          cu.mv[RPL1]     = cMvBi[RPL1];
          cu.refIdx[RPL1] = iRefIdxBi[RPL1];
          cu.mvpIdx[RPL1] = bestBiPMvpL1;

          if (m_encCfg->m_seiCfg.m_MCTSEncConstraint)
          {
            Mv   restrictedMv = cu.mv[RPL1];
            Area curTileAreaRestricted;
            curTileAreaRestricted = cu.cs->picture->m_mctsInfo.getTileAreaSubPelRestricted(cu);
            MCTSHelper::clipMvToArea(restrictedMv, cu.Y(), curTileAreaRestricted, *cu.cs->sps);
            // If sub-pel filter samples are not inside of allowed area
            if (restrictedMv != cu.mv[RPL1])
            {
              costBi   = MAX_DISTORTION;
              doBiPred = false;
            }
          }
          PelUnitBuf predBufTmp = m_tmpPredStorage[RPL1].getBuf(UnitAreaRelative(cu, cu));
          motionCompensation(cu, predBufTmp, RPL1);

          motBits[RPL0] = bits[RPL0] - mbBits[RPL0];
          motBits[RPL1] = mbBits[RPL1];

          if (cs.slice->m_numRefIdx[RPL1] > 1)
          {
            motBits[1] += bestBiPRefIdxL1 + 1;
            if (bestBiPRefIdxL1 == cs.slice->m_numRefIdx[RPL1] - 1)
            {
              motBits[RPL1]--;
            }
          }

          motBits[RPL1] += m_auiMVPIdxCost[aaiMvpIdxBi[RPL1][bestBiPRefIdxL1]][AMVP_MAX_NUM_CANDS];

          bits[2] = mbBits[2] + motBits[RPL0] + motBits[RPL1];

          cMvTemp[RPL1][bestBiPRefIdxL1] = cMvBi[RPL1];
        }
        else
        {
          motBits[RPL0] = bits[RPL0] - mbBits[RPL0];
          motBits[RPL1] = bits[RPL1] - mbBits[RPL1];
          bits[2]       = mbBits[2] + motBits[RPL0] + motBits[RPL1];
        }

        if (doBiPred)
        {
          // 4-times iteration (default)
          int numIter = 4;

          // fast encoder setting: only one iteration
          if (m_encCfg->m_fastInterSearchMode == FASTINTERSEARCH_MODE1 ||
              m_encCfg->m_fastInterSearchMode == FASTINTERSEARCH_MODE2 || cs.picHeader->m_mvdL1ZeroFlag)
          {
            numIter = 1;
          }

          enforceBcwPred = (bcwIdx != BCW_DEFAULT);
          for (int iter = 0; iter < numIter; iter++)
          {
            int refList = iter % 2;

            if (m_encCfg->m_fastInterSearchMode == FASTINTERSEARCH_MODE1 ||
                m_encCfg->m_fastInterSearchMode == FASTINTERSEARCH_MODE2)
            {

              if (uiCost[RPL0] <= uiCost[RPL1])
              {
                refList = 1;
              }
              else
              {
                refList = 0;
              }
              if (bcwIdx != BCW_DEFAULT)
              {
                refList = (abs(getBcwWeight(bcwIdx, RPL0)) > abs(getBcwWeight(bcwIdx, RPL1)) ? 1 : 0);
              }
            }
            else if (iter == 0)
            {
              refList = 0;
            }
            if (iter == 0 && !cs.picHeader->m_mvdL1ZeroFlag)
            {
              cu.mv[1 - refList]     = cMv[1 - refList];
              cu.refIdx[1 - refList] = refIdx[1 - refList];
              PelUnitBuf predBufTmp  = m_tmpPredStorage[1 - refList].getBuf(UnitAreaRelative(cu, cu));
              motionCompensation(cu, predBufTmp, RefPicList(1 - refList));
            }

            RefPicList eRefPicList = (refList ? RPL1 : RPL0);

            if (cs.picHeader->m_mvdL1ZeroFlag)
            {
              refList     = 0;
              eRefPicList = RPL0;
            }

            bool changed = false;

            iRefStart = 0;
            iRefEnd   = cs.slice->m_numRefIdx[eRefPicList] - 1;
            for (int refIdxTemp = iRefStart; refIdxTemp <= iRefEnd; refIdxTemp++)
            {
              bitsTemp = mbBits[2] + motBits[1 - refList];
              bitsTemp += (cs.slice->m_sps->m_useBcw ? getWeightIdxBits(bcwIdx) : 0);
              if (cs.slice->m_numRefIdx[eRefPicList] > 1)
              {
                bitsTemp += refIdxTemp + 1;
                if (refIdxTemp == cs.slice->m_numRefIdx[eRefPicList] - 1)
                {
                  bitsTemp--;
                }
              }
              bitsTemp += m_auiMVPIdxCost[aaiMvpIdxBi[refList][refIdxTemp]][AMVP_MAX_NUM_CANDS];
              if (cs.slice->getBiDirPred())
              {
                bitsTemp += 1;   // add one bit for symmetrical MVD mode
              }
              // call ME
              xCopyAMVPInfo(&aacAMVPInfo[refList][refIdxTemp], &amvp[eRefPicList]);
              xMotionEstimation(cu, origBuf, eRefPicList, cMvPredBi[refList][refIdxTemp], refIdxTemp,
                                cMvTemp[refList][refIdxTemp], aaiMvpIdxBi[refList][refIdxTemp], bitsTemp, costTemp,
                                amvp[eRefPicList], true);

              xCheckBestMVP(eRefPicList, cMvTemp[refList][refIdxTemp], cMvPredBi[refList][refIdxTemp],
                            aaiMvpIdxBi[refList][refIdxTemp], amvp[eRefPicList], bitsTemp, costTemp, cu.imv);
              if (costTemp < costBi)
              {
                changed = true;

                cMvBi[refList]     = cMvTemp[refList][refIdxTemp];
                iRefIdxBi[refList] = refIdxTemp;

                costBi           = costTemp;
                motBits[refList] = bitsTemp - mbBits[2] - motBits[1 - refList];
                motBits[refList] -= (cs.slice->m_sps->m_useBcw ? getWeightIdxBits(bcwIdx) : 0);
                bits[2] = bitsTemp;

                if (numIter != 1)
                {
                  //  Set motion
                  cu.mv[eRefPicList]     = cMvBi[refList];
                  cu.refIdx[eRefPicList] = iRefIdxBi[refList];
                  PelUnitBuf predBufTmp  = m_tmpPredStorage[refList].getBuf(UnitAreaRelative(cu, cu));
                  motionCompensation(cu, predBufTmp, eRefPicList);
                }
              }
            }   // for loop-refIdxTemp

            if (!changed)
            {
              if ((costBi <= uiCost[RPL0] && costBi <= uiCost[RPL1]) || enforceBcwPred)
              {
                xCopyAMVPInfo(&aacAMVPInfo[RPL0][iRefIdxBi[RPL0]], &amvp[RPL0]);
                xCheckBestMVP(RPL0, cMvBi[RPL0], cMvPredBi[RPL0][iRefIdxBi[RPL0]], aaiMvpIdxBi[RPL0][iRefIdxBi[RPL0]],
                              amvp[RPL0], bits[2], costBi, cu.imv);
                if (!cs.picHeader->m_mvdL1ZeroFlag)
                {
                  xCopyAMVPInfo(&aacAMVPInfo[RPL1][iRefIdxBi[RPL1]], &amvp[RPL1]);
                  xCheckBestMVP(RPL1, cMvBi[RPL1], cMvPredBi[RPL1][iRefIdxBi[RPL1]], aaiMvpIdxBi[RPL1][iRefIdxBi[RPL1]],
                                amvp[RPL1], bits[2], costBi, cu.imv);
                }
              }
              break;
            }
          }   // for loop-iter
        }
        cu.refIdxBi[RPL0] = iRefIdxBi[RPL0];
        cu.refIdxBi[RPL1] = iRefIdxBi[RPL1];

        if (cs.slice->getBiDirPred() && trySmvd)
        {
          Distortion symCost;
          Mv         cMvPredSym[NUM_RPL01];
          int        mvpIdxSym[NUM_RPL01];
          int        numStartCand = m_encCfg->m_SMVD > 1 ? 1 : 5;
          bool       testSME      = true;
          double     th1          = 1.02;

          int        curRefList  = RPL0;
          int        tarRefList  = 1 - curRefList;
          RefPicList eCurRefList = (curRefList ? RPL1 : RPL0);
          int        refIdxCur   = cs.slice->getSymRefIdx(curRefList);
          int        refIdxTar   = cs.slice->getSymRefIdx(tarRefList);
          CHECK(refIdxCur == -1 || refIdxTar == -1, "Uninitialized reference index not allowed");

          if (aacAMVPInfo[curRefList][refIdxCur].mvCand[0] == aacAMVPInfo[curRefList][refIdxCur].mvCand[1])
          {
            aacAMVPInfo[curRefList][refIdxCur].numCand = 1;
          }
          if (aacAMVPInfo[tarRefList][refIdxTar].mvCand[0] == aacAMVPInfo[tarRefList][refIdxTar].mvCand[1])
          {
            aacAMVPInfo[tarRefList][refIdxTar].numCand = 1;
          }

          MvField    cCurMvField, cTarMvField;
          Distortion costStart = std::numeric_limits<Distortion>::max();

          for (int i = 0; i < aacAMVPInfo[curRefList][refIdxCur].numCand; i++)
          {
            for (int j = 0; j < aacAMVPInfo[tarRefList][refIdxTar].numCand; j++)
            {
              cCurMvField.setMvField(aacAMVPInfo[curRefList][refIdxCur].mvCand[i], refIdxCur);
              cTarMvField.setMvField(aacAMVPInfo[tarRefList][refIdxTar].mvCand[j], refIdxTar);
              Distortion cost = xGetSymmetricCost(cu, origBuf, eCurRefList, cCurMvField, cTarMvField, bcwIdx);

              if (cost < costStart)
              {
                costStart              = cost;
                cMvPredSym[curRefList] = aacAMVPInfo[curRefList][refIdxCur].mvCand[i];
                cMvPredSym[tarRefList] = aacAMVPInfo[tarRefList][refIdxTar].mvCand[j];
                mvpIdxSym[curRefList]  = i;
                mvpIdxSym[tarRefList]  = j;
              }
            }
          }
          cCurMvField.mv = cMvPredSym[curRefList];
          cTarMvField.mv = cMvPredSym[tarRefList];

          m_pcRdCost->setCostScale(0);
          Mv pred = cMvPredSym[curRefList];
          pred.changeTransPrecInternal2Amvr(cu.imv);
          m_pcRdCost->setPredictor(pred);
          Mv mv = cCurMvField.mv;
          mv.changeTransPrecInternal2Amvr(cu.imv);
          uint32_t bits = m_pcRdCost->getBitsOfVectorWithPredictor(mv.hor, mv.ver, 0);
          bits += m_auiMVPIdxCost[mvpIdxSym[curRefList]][AMVP_MAX_NUM_CANDS];
          bits += m_auiMVPIdxCost[mvpIdxSym[tarRefList]][AMVP_MAX_NUM_CANDS];
          costStart += m_pcRdCost->getCost(bits);

          constexpr int MAX_NUM_SYM_MVD_CANDS = 5;

          static_vector<Mv, MAX_NUM_SYM_MVD_CANDS> symmvdCands;

          auto smmvdCandsGen = [&](Mv mvCand, bool mvPrecAdj)
          {
            if (mvPrecAdj && cu.imv)
            {
              mvCand.roundTransPrecInternal2Amvr(cu.imv);
            }

            bool toAddMvCand = true;
            for (const auto &pos: symmvdCands)
            {
              if (pos == mvCand)
              {
                toAddMvCand = false;
                break;
              }
            }

            if (toAddMvCand)
            {
              symmvdCands.push_back(mvCand);
            }
          };

          smmvdCandsGen(cMvHevcTemp[curRefList][refIdxCur], false);
          smmvdCandsGen(cMvTemp[curRefList][refIdxCur], false);
          if (iRefIdxBi[curRefList] == refIdxCur)
          {
            smmvdCandsGen(cMvBi[curRefList], false);
          }
          for (int i = 0; i < m_uniMvListSize && symmvdCands.size() < symmvdCands.capacity(); i++)
          {
            if (symmvdCands.size() >= numStartCand)
            {
              break;
            }
            const BlkUniMvInfo *curMvInfo =
              m_uniMvList + ((m_uniMvListIdx - 1 - i + m_uniMvListMaxSize) % (m_uniMvListMaxSize));
            smmvdCandsGen(curMvInfo->uniMvs[curRefList][refIdxCur], true);
          }

          for (auto mvStart: symmvdCands)
          {
            bool checked = false;   // if it has been checkin in the mvPred.
            for (int i = 0; i < aacAMVPInfo[curRefList][refIdxCur].numCand && !checked; i++)
            {
              checked |= (mvStart == aacAMVPInfo[curRefList][refIdxCur].mvCand[i]);
            }
            if (checked)
            {
              continue;
            }

            Distortion bestCost = costStart;
            symmvdCheckBestMvp(cu, origBuf, mvStart, (RefPicList)curRefList, aacAMVPInfo, bcwIdx, cMvPredSym, mvpIdxSym,
                               costStart);

            if (costStart < bestCost)
            {
              cCurMvField.setMvField(mvStart, refIdxCur);
              cTarMvField.setMvField(mvStart.getSymmvdMv(cMvPredSym[curRefList], cMvPredSym[tarRefList]), refIdxTar);
            }
          }
          Mv startPtMv = cCurMvField.mv;

          Distortion mvpCost = m_pcRdCost->getCost(m_auiMVPIdxCost[mvpIdxSym[curRefList]][AMVP_MAX_NUM_CANDS] +
                                                   m_auiMVPIdxCost[mvpIdxSym[tarRefList]][AMVP_MAX_NUM_CANDS]);
          symCost            = costStart - mvpCost;

          // ME
          testSME = m_encCfg->m_SMVD <= 2 || (symCost < costBi * th1 && costBi < uiCost[RPL0] && costBi < uiCost[RPL1]);
          if (testSME)
          {
            xSymmetricMotionEstimation(cu, origBuf, cMvPredSym[curRefList], cMvPredSym[tarRefList], eCurRefList,
                                       cCurMvField, cTarMvField, symCost, bcwIdx);
          }
          symCost += mvpCost;

          if (startPtMv != cCurMvField.mv)
          {   // if ME change MV, run a final check for best MVP.
            symmvdCheckBestMvp(cu, origBuf, cCurMvField.mv, (RefPicList)curRefList, aacAMVPInfo, bcwIdx, cMvPredSym,
                               mvpIdxSym, symCost, true);
          }

          bits = mbBits[2];
          bits += 1;   // add one bit for #symmetrical MVD mode
          bits += (cs.slice->m_sps->m_useBcw ? getWeightIdxBits(bcwIdx) : 0);
          symCost += m_pcRdCost->getCost(bits);
          cTarMvField.setMvField(cCurMvField.mv.getSymmvdMv(cMvPredSym[curRefList], cMvPredSym[tarRefList]), refIdxTar);

          if (m_encCfg->m_seiCfg.m_MCTSEncConstraint)
          {
            if (!(MCTSHelper::checkMvForMCTSConstraint(cu, cCurMvField.mv) &&
                  MCTSHelper::checkMvForMCTSConstraint(cu, cTarMvField.mv)))
            {
              symCost = std::numeric_limits<Distortion>::max();
            }
          }
          // save results

          if (symCost < costBi)
          {
            costBi  = symCost;
            symMode = 1 + curRefList;

            cMvBi[curRefList]                            = cCurMvField.mv;
            iRefIdxBi[curRefList]                        = cCurMvField.refIdx;
            aaiMvpIdxBi[curRefList][cCurMvField.refIdx]  = mvpIdxSym[curRefList];
            cMvPredBi[curRefList][iRefIdxBi[curRefList]] = cMvPredSym[curRefList];

            cMvBi[tarRefList]                            = cTarMvField.mv;
            iRefIdxBi[tarRefList]                        = cTarMvField.refIdx;
            aaiMvpIdxBi[tarRefList][cTarMvField.refIdx]  = mvpIdxSym[tarRefList];
            cMvPredBi[tarRefList][iRefIdxBi[tarRefList]] = cMvPredSym[tarRefList];
          }
        }
      }   // if (B_SLICE)

      //  Clear Motion Field
      for (uint32_t rpl = 0; rpl < NUM_RPL01; rpl++)
      {
        cu.mv[rpl]     = Mv();
        cu.mvd[rpl]    = cMvZero;
        cu.refIdx[rpl] = NOT_VALID;
        cu.mvpIdx[rpl] = NOT_VALID;
        cu.mvpNum[rpl] = NOT_VALID;
      }

      // Set Motion Field

      cMv[RPL1]    = mvValidList1;
      refIdx[RPL1] = refIdxValidList1;
      bits[RPL1]   = bitsValidList1;
      uiCost[RPL1] = costValidList1;
      if (cu.cs->pps->m_useBiWP == true && tryBipred && (bcwIdx != BCW_DEFAULT))
      {
        CHECK(iRefIdxBi[RPL0] < 0, "Invalid picture reference index");
        CHECK(iRefIdxBi[RPL1] < 0, "Invalid picture reference index");
        wp0 = cu.cs->slice->getWpScaling(RPL0, iRefIdxBi[RPL0]);
        wp1 = cu.cs->slice->getWpScaling(RPL1, iRefIdxBi[RPL1]);
        if (WPScalingParam::isWeighted(wp0) || WPScalingParam::isWeighted(wp1))
        {
          costBi         = MAX_DISTORTION;
          enforceBcwPred = false;
        }
      }
      if (enforceBcwPred)
      {
        uiCost[RPL0] = uiCost[RPL1] = MAX_DISTORTION;
      }

      uiLastModeTemp = uiLastMode;

      if (costBi <= uiCost[RPL0] && costBi <= uiCost[RPL1])
      {
        uiLastMode = 2;

        for (uint32_t rpl = 0; rpl < NUM_RPL01; rpl++)
        {
          CHECK(iRefIdxBi[rpl] < 0, "Invalid picture reference index");
          cu.mv[rpl]     = cMvBi[rpl];
          cu.mvd[rpl]    = cMvBi[rpl] - cMvPredBi[rpl][iRefIdxBi[rpl]];
          cu.refIdx[rpl] = iRefIdxBi[rpl];
          cu.mvpIdx[rpl] = aaiMvpIdxBi[rpl][iRefIdxBi[rpl]];
          cu.mvpNum[rpl] = aaiMvpNum[rpl][iRefIdxBi[rpl]];
        }
        cu.interDir = 3;

        cu.smvdMode = symMode;
      }
      else if (uiCost[RPL0] <= uiCost[RPL1])
      {
        uiLastMode      = 0;
        cu.mv[RPL0]     = cMv[RPL0];
        cu.mvd[RPL0]    = cMv[RPL0] - cMvPred[RPL0][refIdx[RPL0]];
        cu.refIdx[RPL0] = refIdx[RPL0];
        cu.mvpIdx[RPL0] = aaiMvpIdx[RPL0][refIdx[RPL0]];
        cu.mvpNum[RPL0] = aaiMvpNum[RPL0][refIdx[RPL0]];
        cu.interDir     = 1;
      }
      else
      {
        uiLastMode      = 1;
        cu.mv[RPL1]     = cMv[RPL1];
        cu.mvd[RPL1]    = cMv[RPL1] - cMvPred[RPL1][refIdx[RPL1]];
        cu.refIdx[RPL1] = refIdx[RPL1];
        cu.mvpIdx[RPL1] = aaiMvpIdx[RPL1][refIdx[RPL1]];
        cu.mvpNum[RPL1] = aaiMvpNum[RPL1][refIdx[RPL1]];
        cu.interDir     = 2;
      }

      if (bcwIdx != BCW_DEFAULT)
      {
        cu.bcwIdx = BCW_DEFAULT;   // Reset to default for the Non-NormalMC modes.
      }

      uiHevcCost = (costBi <= uiCost[RPL0] && costBi <= uiCost[RPL1])
        ? costBi
        : ((uiCost[RPL0] <= uiCost[RPL1]) ? uiCost[RPL0] : uiCost[RPL1]);
    }

    if (m_encCfg->m_Affine > 2)
    {
      if (cu.slice->m_uiTLayer > 3)
      {
        checkAffine = false;
      }
      else
      {
        if ((m_encCfg->m_Affine == 4) && (cu.slice->m_uiTLayer >= 2))
        {
          checkAffine =
            m_modeCtrl->comprCUCtx->bestCU ? (checkAffine && m_modeCtrl->comprCUCtx->bestCU->affine) : checkAffine;
        }
      }
    }

    if (cu.licFlag && !m_doAffineLic)
    {
      checkAffine = false;
    }

    if (std::min(cu.lwidth(), cu.lheight()) >= (1 << (cu.slice->m_sps->m_log2MinAffineBlkSizeMinus3 + 3)) &&
        cu.slice->m_sps->m_useAffine && checkAffine && m_encCfg->m_AffineAmvp &&
        (bcwIdx == BCW_DEFAULT || m_affineModeSelected || !m_encCfg->m_BcwFast ||
         (cu.cs->sps->m_biLicEnabledFlag && cu.licFlag && bcwIdx != BCW_DEFAULT)))
    {
      PROFILER_SCOPE(TP_ENABLE_INTER_MVD_SEARCH_STAGES, g_timeProfiler, P_INTER_MVD_SEARCH_AFFINE);
      m_hevcCost          = uiHevcCost;
      // save normal hevc result
      uint32_t uiMRGIndex = cu.mergeIdx;
      bool     bMergeFlag = cu.mergeFlag;
      uint32_t uiInterDir = cu.interDir;
      int      iSymMode   = cu.smvdMode;

      Mv       cMvd[NUM_RPL01];
      uint32_t uiMvpIdx[NUM_RPL01], uiMvpNum[NUM_RPL01];

      MvField cHevcMvField[NUM_RPL01];

      for (uint32_t rpl = 0; rpl < NUM_RPL01; rpl++)
      {
        uiMvpIdx[rpl] = cu.mvpIdx[rpl];
        uiMvpNum[rpl] = cu.mvpNum[rpl];
        cMvd[rpl]     = cu.mvd[rpl];

        cHevcMvField[rpl].setMvField(cu.mv[rpl], cu.refIdx[rpl]);
      }

      // do affine ME & Merge
      cu.affineType = AffineModel::_4_PARAMS;
      RefSetArray<Mv[3]> acMvAffine4Para;
      int                refIdx4Para[NUM_RPL01] = { -1, -1 };

      xPredAffineInterSearch(cu, origBuf, puIdx, uiLastModeTemp, uiAffineCost, cMvHevcTemp, acMvAffine4Para,
                             refIdx4Para, bcwIdx, enforceBcwPred,
                             cu.slice->m_sps->m_useBcw ? getWeightIdxBits(bcwIdx) : 0);

      if (cu.imv == 0)
      {
        storeAffineMotion(cu.mvAffi, cu.refIdx, cu.affineType, bcwIdx);
      }

      if (cu.slice->m_sps->m_AffineType)
      {
        if (uiAffineCost < uiHevcCost * 1.05)   ///< condition for 6 parameter affine ME
        {
          // save 4 parameter results
          Mv      bestMv[NUM_RPL01][3], bestMvd[NUM_RPL01][3];
          int     bestMvpIdx[NUM_RPL01], bestMvpNum[NUM_RPL01], bestRefIdx[NUM_RPL01];
          uint8_t bestInterDir;

          bestInterDir = cu.interDir;
          for (uint32_t rpl = 0; rpl < NUM_RPL01; rpl++)
          {
            bestRefIdx[rpl] = cu.refIdx[rpl];
            bestMvpIdx[rpl] = cu.mvpIdx[rpl];
            bestMvpNum[rpl] = cu.mvpNum[rpl];

            for (int verIdx = 0; verIdx < 3; verIdx++)
            {
              bestMv[rpl][verIdx]  = cu.mvAffi[rpl][verIdx];
              bestMvd[rpl][verIdx] = cu.mvdAffi[rpl][verIdx];
            }

            refIdx4Para[rpl] = bestRefIdx[rpl];
          }

          Distortion uiAffine6Cost = std::numeric_limits<Distortion>::max();
          cu.affineType            = AffineModel::_6_PARAMS;
          xPredAffineInterSearch(cu, origBuf, puIdx, uiLastModeTemp, uiAffine6Cost, cMvHevcTemp, acMvAffine4Para,
                                 refIdx4Para, bcwIdx, enforceBcwPred,
                                 (cu.slice->m_sps->m_useBcw ? getWeightIdxBits(bcwIdx) : 0));

          if (cu.imv == 0)
          {
            storeAffineMotion(cu.mvAffi, cu.refIdx, cu.affineType, bcwIdx);
          }

          // reset to 4 parameter affine inter mode
          if (uiAffineCost <= uiAffine6Cost)
          {
            cu.affineType = AffineModel::_4_PARAMS;
            cu.interDir   = bestInterDir;

            for (uint32_t rpl = 0; rpl < NUM_RPL01; rpl++)
            {
              cu.refIdx[rpl] = bestRefIdx[rpl];
              cu.mvpIdx[rpl] = bestMvpIdx[rpl];
              cu.mvpNum[rpl] = bestMvpNum[rpl];

              for (int verIdx = 0; verIdx < 3; verIdx++)
              {
                cu.mvdAffi[rpl][verIdx] = bestMvd[rpl][verIdx];
              }

              PU::setAllAffineMv(cu, bestMv[rpl][0], bestMv[rpl][1], bestMv[rpl][2], RefPicList(rpl));
            }
          }
          else
          {
            uiAffineCost = uiAffine6Cost;
          }
        }

        uiAffineCost += m_pcRdCost->getCost(1);   // add one bit for affine_type
      }

      if (uiAffineCost < uiHevcCost)
      {
        if (m_encCfg->m_seiCfg.m_MCTSEncConstraint && !MCTSHelper::checkMvBufferForMCTSConstraint(cu))
        {
          uiAffineCost = std::numeric_limits<Distortion>::max();
        }
      }

      if (uiHevcCost <= uiAffineCost)
      {
        // set hevc me result
        cu.affine           = false;
        cu.mergeFlag        = bMergeFlag;
        cu.regularMergeFlag = false;
        cu.mergeIdx         = uiMRGIndex;
        cu.interDir         = uiInterDir;
        cu.smvdMode         = iSymMode;

        for (uint32_t rpl = 0; rpl < NUM_RPL01; rpl++)
        {
          cu.mv[rpl]     = cHevcMvField[rpl].mv;
          cu.refIdx[rpl] = cHevcMvField[rpl].refIdx;
          cu.mvpIdx[rpl] = uiMvpIdx[rpl];
          cu.mvpNum[rpl] = uiMvpNum[rpl];
          cu.mvd[rpl]    = cMvd[rpl];
        }
      }
      else
      {
        cu.smvdMode = 0;
        CHECK(!cu.affine, "Wrong.");
        uiLastMode = uiLastModeTemp;
      }
    }

    if (cu.interDir == 3 && !cu.mergeFlag)
    {
      if (bcwIdx != BCW_DEFAULT)
      {
        cu.bcwIdx = bcwIdx;
      }
    }
    m_maxCompIDToPred = MAX_NUM_COMP;

    PU::spanMotionInfo(cu);

    m_skipProf         = false;
    m_skipProfCond     = false;
    //  MC
    PelUnitBuf predBuf = cu.cs->getPredBuf(cu);
    if (bcwIdx == BCW_DEFAULT || !m_affineMotion.affine4ParaAvail || !m_affineMotion.affine6ParaAvail)
    {
      if (cu.imv < 3)
      {
        m_affineMotion.hevcCost[cu.imv] = uiHevcCost;
      }
    }
    if (cu.licFlag)
    {
      m_storeBeforeLIC =
        !cu.cs->sps->m_biLicEnabledFlag || cu.interDir == 1 || cu.interDir == 2 || cu.affine || cu.smvdMode;
      m_predictionBeforeLIC = m_tmpStorageCtu.getBuf(UnitAreaRelative(cu, cu));
    }
    m_encMotionEstimation = false;
    motionCompensation(cu, predBuf, RPLX);
    m_storeBeforeLIC = false;
    puIdx++;
  }

  if (cu.licFlag &&
      cu.interDir != 10)   // xCheckRDCostInterIMV initializes pu.interDir by using 10. When checkAffine and
                           // checkNonAffine are both false, pu.interDir remains 10 which should be avoided
  {
    CHECK(cu.interDir != 1 && cu.interDir != 2 && (!cu.cs->sps->m_biLicEnabledFlag || cu.interDir != 3),
          "Invalid InterDir for LIC");

    PelUnitBuf predBuf = cu.cs->getPredBuf(cu);
    DistParam  distParam;
    m_pcRdCost->setDistParam(distParam, cs.getOrgBuf().Y(), predBuf.Y(), cs.sps->m_bitDepths[ChannelType::LUMA], COMP_Y,
                             1);
    const Distortion distLicOn = distParam.distFunc(distParam);

    if (cu.cs->sps->m_biLicEnabledFlag && !(cu.interDir == 1 || cu.interDir == 2 || cu.affine || cu.smvdMode))
    {
      cu.licFlag = false;
      motionCompensation(cu, m_predictionBeforeLIC, RPLX);
      cu.licFlag = true;
    }
    m_pcRdCost->setDistParam(distParam, cs.getOrgBuf().Y(), m_predictionBeforeLIC.Y(),
                             cs.sps->m_bitDepths[ChannelType::LUMA], COMP_Y, 1);
    const Distortion distLicOff = distParam.distFunc(distParam);
    if (distLicOn >= distLicOff)
    {
      cu.licFlag = false;
      PU::spanLicFlags(cu, false);
      predBuf.copyFrom(m_predictionBeforeLIC);
    }
  }
  m_predictionBeforeLIC = PelUnitBuf();

  xSetWpScalingDistParam(-1, RPLX, cu.cs->slice);

  return;
}

uint32_t InterSearch::xCalcAffineMVBits(CodingUnit &cu, Mv acMvTemp[3], Mv acMvPred[3])
{
  const int mvNum = cu.getNumAffineMvs();
  m_pcRdCost->setCostScale(0);
  uint32_t bitsTemp = 0;

  for (int verIdx = 0; verIdx < mvNum; verIdx++)
  {
    Mv pred = verIdx == 0 ? acMvPred[verIdx] : acMvPred[verIdx] + acMvTemp[0] - acMvPred[0];
    pred.changeAffinePrecInternal2Amvr(cu.imv);
    m_pcRdCost->setPredictor(pred);
    Mv mv = acMvTemp[verIdx];
    mv.changeAffinePrecInternal2Amvr(cu.imv);

    bitsTemp += m_pcRdCost->getBitsOfVectorWithPredictor(mv.getHor(), mv.getVer(), 0);
  }

  return bitsTemp;
}

// AMVP
void InterSearch::xEstimateMvPredAMVP(CodingUnit &cu, PelUnitBuf &origBuf, RefPicList eRefPicList, int refIdx,
                                      Mv &rcMvPred, AMVPInfo &rAMVPInfo, bool bFilled, Distortion *puiDistBiP)
{
  Mv         cBestMv;
  int        iBestIdx   = 0;
  Distortion uiBestCost = std::numeric_limits<Distortion>::max();
  int        i;

  AMVPInfo *pcAMVPInfo = &rAMVPInfo;

  // Fill the MV Candidates
  if (!bFilled)
  {
    PU::fillMvpCand(cu, eRefPicList, refIdx, *pcAMVPInfo);
  }

  // initialize Mvp index & Mvp
  iBestIdx = 0;
  cBestMv  = pcAMVPInfo->mvCand[0];

  PelUnitBuf predBuf = m_tmpStorageCtu.getBuf(UnitAreaRelative(cu, cu));

  //-- Check Minimum Cost.
  for (i = 0; i < pcAMVPInfo->numCand; i++)
  {
    Distortion uiTmpCost =
      xGetTemplateCost(cu, origBuf, predBuf, pcAMVPInfo->mvCand[i], i, AMVP_MAX_NUM_CANDS, eRefPicList, refIdx);

    if (uiBestCost > uiTmpCost)
    {
      uiBestCost    = uiTmpCost;
      cBestMv       = pcAMVPInfo->mvCand[i];
      iBestIdx      = i;
      (*puiDistBiP) = uiTmpCost;
    }
  }

  // Setting Best MVP
  rcMvPred               = cBestMv;
  cu.mvpIdx[eRefPicList] = iBestIdx;
  cu.mvpNum[eRefPicList] = pcAMVPInfo->numCand;

  return;
}

uint32_t InterSearch::xGetMvpIdxBits(int idx, int num)
{
  CHECK(idx < 0 || num < 0 || idx >= num, "Invalid parameters");

  if (num == 1)
  {
    return 0;
  }

  uint32_t length = 1;
  int      temp   = idx;
  if (temp == 0)
  {
    return length;
  }

  bool bCodeLast = (num - 1 > temp);

  length += (temp - 1);

  if (bCodeLast)
  {
    length++;
  }

  return length;
}

void InterSearch::xGetBlkBits(bool isPSlice, uint32_t blkBit[3])
{
  blkBit[0] = (!isPSlice) ? 3 : 1;
  blkBit[1] = 3;
  blkBit[2] = 5;
}

void InterSearch::xCopyAMVPInfo(AMVPInfo *pSrc, AMVPInfo *pDst)
{
  pDst->numCand = pSrc->numCand;
  for (int i = 0; i < pSrc->numCand; i++)
  {
    pDst->mvCand[i] = pSrc->mvCand[i];
  }
}

void InterSearch::xCheckBestMVP(RefPicList eRefPicList, Mv cMv, Mv &rcMvPred, int &riMVPIdx, AMVPInfo &amvpInfo,
                                uint32_t &ruiBits, Distortion &ruiCost, const uint8_t imv)
{

  if (imv > 0 && imv < 3)
  {
    return;
  }

  AMVPInfo *pcAMVPInfo = &amvpInfo;

  CHECK(pcAMVPInfo->mvCand[riMVPIdx] != rcMvPred, "Invalid MV prediction candidate");

  if (pcAMVPInfo->numCand < 2)
  {
    return;
  }

  m_pcRdCost->setCostScale(0);

  int iBestMVPIdx = riMVPIdx;

  Mv pred = rcMvPred;
  pred.changeTransPrecInternal2Amvr(imv);
  m_pcRdCost->setPredictor(pred);
  Mv mv = cMv;
  mv.changeTransPrecInternal2Amvr(imv);
  int iOrgMvBits = m_pcRdCost->getBitsOfVectorWithPredictor(mv.getHor(), mv.getVer(), 0);
  iOrgMvBits += m_auiMVPIdxCost[riMVPIdx][AMVP_MAX_NUM_CANDS];
  int iBestMvBits = iOrgMvBits;

  for (int mvpIdx = 0; mvpIdx < pcAMVPInfo->numCand; mvpIdx++)
  {
    if (mvpIdx == riMVPIdx)
    {
      continue;
    }

    pred = pcAMVPInfo->mvCand[mvpIdx];
    pred.changeTransPrecInternal2Amvr(imv);
    m_pcRdCost->setPredictor(pred);
    int iMvBits = m_pcRdCost->getBitsOfVectorWithPredictor(mv.getHor(), mv.getVer(), 0);
    iMvBits += m_auiMVPIdxCost[mvpIdx][AMVP_MAX_NUM_CANDS];

    if (iMvBits < iBestMvBits)
    {
      iBestMvBits = iMvBits;
      iBestMVPIdx = mvpIdx;
    }
  }

  if (iBestMVPIdx != riMVPIdx)   // if changed
  {
    rcMvPred = pcAMVPInfo->mvCand[iBestMVPIdx];

    riMVPIdx           = iBestMVPIdx;
    uint32_t uiOrgBits = ruiBits;
    ruiBits            = uiOrgBits - iOrgMvBits + iBestMvBits;
    ruiCost            = (ruiCost - m_pcRdCost->getCost(uiOrgBits)) + m_pcRdCost->getCost(ruiBits);
  }
}

Distortion InterSearch::xGetTemplateCost(const CodingUnit &cu, PelUnitBuf &origBuf, PelUnitBuf &predBuf, Mv cMvCand,
                                         int mvpIdx, int mvpNum, RefPicList eRefPicList, int refIdx)
{
  Distortion uiCost = std::numeric_limits<Distortion>::max();

  const Picture *picRef = cu.slice->getRefPic(eRefPicList, refIdx);
  clipMv(cMvCand, cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);

  // prediction pattern
  const bool bi = cu.slice->m_testWeightPred && cu.slice->m_eSliceType == P_SLICE && !cu.licFlag;

  xPredInterBlk(COMP_Y, cu, picRef, cMvCand, predBuf, bi, cu.slice->clpRng(COMP_Y), false, false, eRefPicList);

  if (bi)
  {
    xWeightedPredictionUni(cu, predBuf, eRefPicList, predBuf, refIdx, m_maxCompIDToPred);
  }

  // calc distortion

  uiCost =
    m_pcRdCost->getDistPart(origBuf.Y(), predBuf.Y(), cu.cs->sps->m_bitDepths[ChannelType::LUMA], COMP_Y, DFunc::SAD);
  uiCost += m_pcRdCost->getCost(m_auiMVPIdxCost[mvpIdx][mvpNum]);

  return uiCost;
}

Distortion InterSearch::xGetAffineTemplateCost(CodingUnit &cu, PelUnitBuf &origBuf, PelUnitBuf &predBuf, Mv acMvCand[3],
                                               int mvpIdx, int mvpNum, RefPicList eRefPicList, int refIdx)
{
  Distortion uiCost = std::numeric_limits<Distortion>::max();

  const Picture *picRef = cu.slice->getRefPic(eRefPicList, refIdx);

  // prediction pattern
  const bool bi = cu.slice->m_testWeightPred && cu.slice->m_eSliceType == P_SLICE && !cu.licFlag;
  Mv         mv[3];
  memcpy(mv, acMvCand, sizeof(mv));

  xPredAffineBlk(COMP_Y, cu, picRef, mv, predBuf, bi, cu.slice->clpRng(COMP_Y), eRefPicList);
  if (bi)
  {
    xWeightedPredictionUni(cu, predBuf, eRefPicList, predBuf, refIdx, m_maxCompIDToPred);
  }

  // calc distortion
  const DFunc distFunc = (cu.cs->slice->m_disableSATDForRd) ? DFunc::SAD : DFunc::HAD;
  uiCost =
    m_pcRdCost->getDistPart(origBuf.Y(), predBuf.Y(), cu.cs->sps->m_bitDepths[ChannelType::LUMA], COMP_Y, distFunc);
  uiCost += m_pcRdCost->getCost(m_auiMVPIdxCost[mvpIdx][mvpNum]);
  DTRACE(g_trace_ctx, D_COMMON, " (%d) affineTemplateCost=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON), uiCost);
  return uiCost;
}

void InterSearch::xMotionEstimation(CodingUnit &cu, PelUnitBuf &origBuf, RefPicList eRefPicList, Mv &rcMvPred,
                                    int refIdxPred, Mv &rcMv, int &riMVPIdx, uint32_t &ruiBits, Distortion &ruiCost,
                                    const AMVPInfo &amvpInfo, bool bBi)
{
  if (cu.cs->sps->m_useBcw && cu.bcwIdx != BCW_DEFAULT && !bBi &&
      xReadBufferedUniMv(cu, eRefPicList, refIdxPred, rcMvPred, rcMv, ruiBits, ruiCost))
  {
    return;
  }

  Mv cMvHalf, cMvQter;

  CHECK(eRefPicList >= MAX_NUM_REF_LIST_ADAPT_SR || refIdxPred >= int(MAX_IDX_ADAPT_SR),
        "Invalid reference picture list");
  m_searchRange = m_adaptSR[eRefPicList][refIdxPred];

  int    iSrchRng = (bBi ? m_bipredSearchRange : m_searchRange);
  double fWeight  = 1.0;

  PelUnitBuf  origBufTmp = m_tmpStorageCtu.getBuf(UnitAreaRelative(cu, cu));
  PelUnitBuf *pBuf       = &origBuf;

  if (bBi)   // Bi-predictive ME
  {
    // NOTE: Other buf contains predicted signal from another direction
    PelUnitBuf otherBuf = m_tmpPredStorage[1 - (int)eRefPicList].getBuf(UnitAreaRelative(cu, cu));
    origBufTmp.copyFrom(origBuf);
    origBufTmp.removeHighFreq(otherBuf, m_encCfg->m_bClipForBiPredMeEnabled, cu.slice->m_clpRngs,
                              getBcwWeight(cu.bcwIdx, eRefPicList));
    pBuf = &origBufTmp;

    fWeight = xGetMEDistortionWeight(cu.bcwIdx, eRefPicList);
  }
  m_cDistParam.isBiPred = bBi;
  m_cDistParam.useMR    = cu.licFlag;

  //  Search key pattern initialization
  CPelBuf  tmpPattern   = pBuf->Y();
  CPelBuf *pcPatternKey = &tmpPattern;

  m_lumaClpRng = cu.cs->slice->clpRng(COMP_Y);

  bool    wrap = cu.slice->getRefPic(eRefPicList, refIdxPred)->isWrapAroundEnabled(cu.cs->pps);
  CPelBuf buf  = cu.slice->getRefPic(eRefPicList, refIdxPred)->getRecoBuf(cu.blocks[COMP_Y], wrap);

  IntTZSearchStruct cStruct;
  cStruct.pcPatternKey = pcPatternKey;
  cStruct.iRefStride   = buf.stride;
  cStruct.piRefY       = buf.buf;
  cStruct.imvShift     = cu.imv == IMV_HPEL ? 1 : (cu.imv << 1);
  cStruct.useAltHpelIf = cu.imv == IMV_HPEL;
  cStruct.inCtuSearch  = false;
  cStruct.zeroMV       = false;

  if (m_useCompositeRef && cu.cs->slice->getRefPic(eRefPicList, refIdxPred)->m_longTerm)
  {
    cStruct.inCtuSearch = true;
  }

  auto blkCache = dynamic_cast<CacheBlkInfoCtrl *>(m_modeCtrl);

  bool bQTBTMV  = false;
  bool bQTBTMV2 = false;
  Mv   cIntMv;
  if (!bBi)
  {
    bool bValid = blkCache && blkCache->getMv(cu, eRefPicList, refIdxPred, cIntMv);
    if (bValid)
    {
      bQTBTMV2 = true;
      cIntMv.changePrecision(MvPrecision::ONE, MvPrecision::INTERNAL);
    }
  }

  Mv predQuarter = rcMvPred;
  predQuarter.changePrecision(MvPrecision::INTERNAL, MvPrecision::QUARTER);
  m_pcRdCost->setPredictor(predQuarter);

  m_pcRdCost->setCostScale(2);

  if (cu.licFlag)
  {
    m_cDistParam.applyWeight = false;
  }
  else
  {
    xSetWpScalingDistParam(refIdxPred, eRefPicList, cu.slice);
  }
  m_currRefPicList  = eRefPicList;
  m_currRefPicIndex = refIdxPred;
  m_skipFracME      = false;
  //  Do integer search
  if (m_motionEstimationSearchMethod == MESearchMethod::FULL || bBi || bQTBTMV)
  {
    cStruct.subShiftMode = m_encCfg->m_fastInterSearchMode == FASTINTERSEARCH_MODE1 ||
        m_encCfg->m_fastInterSearchMode == FASTINTERSEARCH_MODE3
      ? 2
      : 0;
    m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd,
                             COMP_Y, cStruct.subShiftMode);

    Mv bestInitMv = (bBi ? rcMv : rcMvPred);
    Mv cTmpMv     = bestInitMv;

    clipMv(cTmpMv, cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);
    cTmpMv.changePrecision(MvPrecision::INTERNAL, MvPrecision::ONE);
    m_cDistParam.cur.buf = cStruct.piRefY + (cTmpMv.ver * cStruct.iRefStride) + cTmpMv.hor;
    Distortion uiBestSad = m_cDistParam.distFunc(m_cDistParam);
    uiBestSad += m_pcRdCost->getCostOfVectorWithPredictor(cTmpMv.hor, cTmpMv.ver, cStruct.imvShift);

    const MvPrecision tmpIntMvPrec = (cu.imv == IMV_4PEL ? MvPrecision::FOUR : MvPrecision::ONE);
    for (int i = 0; i < m_uniMvListSize; i++)
    {
      const BlkUniMvInfo *curMvInfo =
        m_uniMvList + ((m_uniMvListIdx - 1 - i + m_uniMvListMaxSize) % (m_uniMvListMaxSize));
      Mv tmpCurMv = curMvInfo->uniMvs[eRefPicList][refIdxPred];
      tmpCurMv.changePrecision(MvPrecision::INTERNAL, tmpIntMvPrec);

      int j = 0;
      for (; j < i; j++)
      {
        const BlkUniMvInfo *prevMvInfo =
          m_uniMvList + ((m_uniMvListIdx - 1 - j + m_uniMvListMaxSize) % (m_uniMvListMaxSize));
        Mv tmpPrevMv = prevMvInfo->uniMvs[eRefPicList][refIdxPred];
        tmpPrevMv.changePrecision(MvPrecision::INTERNAL, tmpIntMvPrec);
        if (tmpCurMv == tmpPrevMv)
        {
          break;
        }
      }
      if (j < i)
      {
        continue;
      }

      cTmpMv = curMvInfo->uniMvs[eRefPicList][refIdxPred];
      clipMv(cTmpMv, cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);
      cTmpMv.changePrecision(MvPrecision::INTERNAL, tmpIntMvPrec);
      if (tmpIntMvPrec != MvPrecision::ONE)
      {
        cTmpMv.changePrecision(tmpIntMvPrec, MvPrecision::ONE);
      }
      m_cDistParam.cur.buf = cStruct.piRefY + (cTmpMv.ver * cStruct.iRefStride) + cTmpMv.hor;

      Distortion uiSad = m_cDistParam.distFunc(m_cDistParam);
      uiSad += m_pcRdCost->getCostOfVectorWithPredictor(cTmpMv.hor, cTmpMv.ver, cStruct.imvShift);
      if (uiSad < uiBestSad)
      {
        uiBestSad                                  = uiSad;
        bestInitMv                                 = curMvInfo->uniMvs[eRefPicList][refIdxPred];
        m_cDistParam.maximumDistortionForEarlyExit = uiSad;
      }
    }

    if (!bQTBTMV)
    {
      xSetSearchRange(cu, bestInitMv, iSrchRng, cStruct.searchRange, cStruct);
    }
    xPatternSearch(cStruct, rcMv, ruiCost);
  }
  else if (bQTBTMV2)
  {
    rcMv = cIntMv;

    cStruct.subShiftMode =
      !m_encCfg->m_bRestrictMESampling && m_encCfg->m_motionEstimationSearchMethod == MESearchMethod::SELECTIVE ? 1
      : m_encCfg->m_fastInterSearchMode == FASTINTERSEARCH_MODE1 ||
        m_encCfg->m_fastInterSearchMode == FASTINTERSEARCH_MODE3
      ? 2
      : 0;
    xTZSearch(cu, eRefPicList, refIdxPred, cStruct, rcMv, ruiCost, nullptr, false, true);
  }
  else
  {
    cStruct.subShiftMode =
      !m_encCfg->m_bRestrictMESampling && m_encCfg->m_motionEstimationSearchMethod == MESearchMethod::SELECTIVE ? 1
      : m_encCfg->m_fastInterSearchMode == FASTINTERSEARCH_MODE1 ||
        m_encCfg->m_fastInterSearchMode == FASTINTERSEARCH_MODE3
      ? 2
      : 0;

    rcMv                          = rcMvPred;
    const Mv *pIntegerMv2Nx2NPred = 0;
    xPatternSearchFast(cu, eRefPicList, refIdxPred, cStruct, rcMv, ruiCost, pIntegerMv2Nx2NPred);
    if (blkCache)
    {
      blkCache->setMv(cu.cs->area, eRefPicList, refIdxPred, rcMv);
    }
    else
    {
      m_integerMv2Nx2N[eRefPicList][refIdxPred] = rcMv;
    }
  }

  DTRACE(g_trace_ctx, D_ME, "%d %d %d :MECostFPel<L%d,%d>: %d,%d,%dx%d, %d", DTRACE_GET_COUNTER(g_trace_ctx, D_ME),
         cu.slice->m_poc, 0, (int)eRefPicList, (int)bBi, cu.lx(), cu.ly(), cu.lwidth(), cu.lheight(), ruiCost);
  // sub-pel refinement for sub-pel resolution
  if (cu.imv == 0 || cu.imv == IMV_HPEL)
  {
    if (m_encCfg->m_seiCfg.m_MCTSEncConstraint)
    {
      Area curTileAreaSubPelRestricted = cu.cs->picture->m_mctsInfo.getTileAreaSubPelRestricted(cu);
      // Area adjustment, because subpel refinement is going to (x-1;y-1) direction
      curTileAreaSubPelRestricted.x += 1;
      curTileAreaSubPelRestricted.y += 1;
      curTileAreaSubPelRestricted.width -= 1;
      curTileAreaSubPelRestricted.height -= 1;
      if (!MCTSHelper::checkMvIsNotInRestrictedArea(cu, rcMv, curTileAreaSubPelRestricted, MvPrecision::ONE))
      {
        MCTSHelper::clipMvToArea(rcMv, cu.Y(), curTileAreaSubPelRestricted, *cu.cs->sps, 0);
      }
    }
    xPatternSearchFracDIF(cu, eRefPicList, refIdxPred, cStruct, rcMv, cMvHalf, cMvQter, ruiCost);
    m_pcRdCost->setCostScale(0);
    rcMv <<= 2;
    rcMv += (cMvHalf <<= 1);
    rcMv += cMvQter;
    uint32_t uiMvBits = m_pcRdCost->getBitsOfVectorWithPredictor(rcMv.getHor(), rcMv.getVer(), cStruct.imvShift);
    ruiBits += uiMvBits;
    ruiCost = (Distortion)(floor(fWeight * ((double)ruiCost - (double)m_pcRdCost->getCost(uiMvBits))) +
                           (double)m_pcRdCost->getCost(ruiBits));
    rcMv.changePrecision(MvPrecision::QUARTER, MvPrecision::INTERNAL);
  }
  else   // integer refinement for integer-pel and 4-pel resolution
  {
    rcMv.changePrecision(MvPrecision::ONE, MvPrecision::INTERNAL);
    xPatternSearchIntRefine(cu, cStruct, rcMv, rcMvPred, riMVPIdx, ruiBits, ruiCost, amvpInfo, fWeight);
  }

  if (cu.licFlag)
  {
    PelUnitBuf     predTempBuf = m_tmpAffiStorage.getBuf(UnitAreaRelative(cu, cu));
    const Picture *picRef      = cu.slice->getRefPic(eRefPicList, refIdxPred);
    Mv             rcMvClipped(rcMv);
    if (cu.cs->sps->m_MCBP)
    {
      clipMv(rcMvClipped, cu.lumaPos(), cu.lumaSize(), *(cu.cs->sps), *(cu.cs->pps));
    }
    xPredInterBlk(COMP_Y, cu, picRef, rcMvClipped, predTempBuf, false, cu.slice->clpRng(COMP_Y), false, false,
                  eRefPicList);

    DistParam distParam;
    m_pcRdCost->setDistParam(distParam, pBuf->Y(), predTempBuf.Y(), cu.cs->sps->m_bitDepths[ChannelType::LUMA], COMP_Y,
                             cu.cs->slice->m_disableSATDForRd ? 0 : 1);
    ruiCost = (Distortion)floor(fWeight * (double)distParam.distFunc(distParam)) + m_pcRdCost->getCost(ruiBits);
  }
  DTRACE(g_trace_ctx, D_ME, "   MECost<L%d,%d>: %6d (%d)  MV:%d,%d\n", (int)eRefPicList, (int)bBi, ruiCost, ruiBits,
         rcMv.getHor() << 2, rcMv.getVer() << 2);
}

void InterSearch::xSetSearchRange(const CodingUnit &cu, const Mv &cMvPred, const int iSrchRng, SearchRange &sr,
                                  IntTZSearchStruct &cStruct)
{
  const int iMvShift  = MV_FRACTIONAL_BITS_INTERNAL;
  Mv        cFPMvPred = cMvPred;
  clipMv(cFPMvPred, cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);

  Mv mvTL(cFPMvPred.getHor() - (iSrchRng << iMvShift), cFPMvPred.getVer() - (iSrchRng << iMvShift));
  Mv mvBR(cFPMvPred.getHor() + (iSrchRng << iMvShift), cFPMvPred.getVer() + (iSrchRng << iMvShift));

  if (m_encCfg->m_seiCfg.m_MCTSEncConstraint)
  {
    MCTSHelper::clipMvToArea(mvTL, cu.Y(), cu.cs->picture->m_mctsInfo.getTileArea(), *cu.cs->sps);
    MCTSHelper::clipMvToArea(mvBR, cu.Y(), cu.cs->picture->m_mctsInfo.getTileArea(), *cu.cs->sps);
  }
  else
  {
    xClipMv(mvTL, cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);
    xClipMv(mvBR, cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);
  }

  mvTL >>= iMvShift;
  mvBR >>= iMvShift;

  sr.left   = mvTL.hor;
  sr.top    = mvTL.ver;
  sr.right  = mvBR.hor;
  sr.bottom = mvBR.ver;

  if (m_useCompositeRef && cStruct.inCtuSearch)
  {
    Position             posRB = cu.Y().bottomRight();
    Position             posTL = cu.Y().topLeft();
    const PreCalcValues *pcv   = cu.cs->pcv;
    Position             posRBinCTU(posRB.x & pcv->maxCUWidthMask, posRB.y & pcv->maxCUHeightMask);
    Position posLTinCTU = Position(posTL.x & pcv->maxCUWidthMask, posTL.y & pcv->maxCUHeightMask).offset(-4, -4);
    if (sr.left < -posLTinCTU.x)
    {
      sr.left = -posLTinCTU.x;
    }
    if (sr.top < -posLTinCTU.y)
    {
      sr.top = -posLTinCTU.y;
    }
    if (sr.right > ((int)pcv->maxCUWidth - 4 - posRBinCTU.x))
    {
      sr.right = (int)pcv->maxCUWidth - 4 - posRBinCTU.x;
    }
    if (sr.bottom > ((int)pcv->maxCUHeight - 4 - posRBinCTU.y))
    {
      sr.bottom = (int)pcv->maxCUHeight - 4 - posRBinCTU.y;
    }
    if (posLTinCTU.x == -4 || posLTinCTU.y == -4)
    {
      sr.left = sr.right = sr.bottom = sr.top = 0;
      cStruct.zeroMV                          = 1;
    }
    if (posRBinCTU.x == pcv->maxCUWidthMask || posRBinCTU.y == pcv->maxCUHeightMask)
    {
      sr.left = sr.right = sr.bottom = sr.top = 0;
      cStruct.zeroMV                          = 1;
    }
  }
}

void InterSearch::xPatternSearch(IntTZSearchStruct &cStruct, Mv &rcMv, Distortion &ruiSAD)
{
  Distortion uiSad;
  Distortion uiSadBest = std::numeric_limits<Distortion>::max();
  int        iBestX    = 0;
  int        iBestY    = 0;

  //-- jclee for using the SAD function pointer
  m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd,
                           COMP_Y, cStruct.subShiftMode);

  const SearchRange &sr = cStruct.searchRange;

  const Pel *piRef = cStruct.piRefY + (sr.top * cStruct.iRefStride);
  for (int y = sr.top; y <= sr.bottom; y++)
  {
    for (int x = sr.left; x <= sr.right; x++)
    {
      //  find min. distortion position
      m_cDistParam.cur.buf = piRef + x;

      uiSad = m_cDistParam.distFunc(m_cDistParam);

      // motion cost
      uiSad += m_pcRdCost->getCostOfVectorWithPredictor(x, y, cStruct.imvShift);

      if (uiSad < uiSadBest)
      {
        uiSadBest                                  = uiSad;
        iBestX                                     = x;
        iBestY                                     = y;
        m_cDistParam.maximumDistortionForEarlyExit = uiSad;
      }
    }
    piRef += cStruct.iRefStride;
  }
  rcMv.set(iBestX, iBestY);

  cStruct.uiBestSad = uiSadBest;   // th for testing
  ruiSAD            = uiSadBest - m_pcRdCost->getCostOfVectorWithPredictor(iBestX, iBestY, cStruct.imvShift);
  return;
}

void InterSearch::xPatternSearchFast(const CodingUnit &cu, RefPicList eRefPicList, int refIdxPred,
                                     IntTZSearchStruct &cStruct, Mv &rcMv, Distortion &ruiSAD,
                                     const Mv *const pIntegerMv2Nx2NPred)
{
  switch (m_motionEstimationSearchMethod)
  {
  case MESearchMethod::DIAMOND:
    xTZSearch(cu, eRefPicList, refIdxPred, cStruct, rcMv, ruiSAD, pIntegerMv2Nx2NPred, false);
    break;

  case MESearchMethod::SELECTIVE:
    xTZSearchSelective(cu, eRefPicList, refIdxPred, cStruct, rcMv, ruiSAD, pIntegerMv2Nx2NPred);
    break;

  case MESearchMethod::DIAMOND_ENHANCED:
    xTZSearch(cu, eRefPicList, refIdxPred, cStruct, rcMv, ruiSAD, pIntegerMv2Nx2NPred, true);
    break;

  case MESearchMethod::FULL:   // shouldn't get here.
  default:
    break;
  }
}

void InterSearch::xTZSearch(const CodingUnit &cu, RefPicList eRefPicList, int refIdxPred, IntTZSearchStruct &cStruct,
                            Mv &rcMv, Distortion &ruiSAD, const Mv *const pIntegerMv2Nx2NPred,
                            const bool bExtendedSettings, const bool bFastSettings)
{
  const bool bUseRasterInFastMode = true;   // toggle this to further reduce runtime

  const bool     bUseAdaptiveRaster           = bExtendedSettings;
  const int      iRaster                      = (bFastSettings && bUseRasterInFastMode) ? 8 : 5;
  const bool     bTestZeroVector              = true && !bFastSettings;
  const bool     bTestZeroVectorStart         = bExtendedSettings;
  const bool     bTestZeroVectorStop          = false;
  const bool     bFirstSearchDiamond          = true;   // 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch
  const bool     bFirstCornersForDiamondDist1 = bExtendedSettings;
  const bool     bFirstSearchStop             = m_encCfg->m_bFastMEAssumingSmootherMVEnabled;
  const uint32_t uiFirstSearchRounds =
    bFastSettings ? (bUseRasterInFastMode ? 3 : 2) : 3;   // first search stop X rounds after best match (must be >=1)
  const bool     bEnableRasterSearch      = bFastSettings ? bUseRasterInFastMode : true;
  const bool     bAlwaysRasterSearch      = bExtendedSettings;   // true: BETTER but factor 2 slower
  const bool     bRasterRefinementEnable  = false;   // enable either raster refinement or star refinement
  const bool     bRasterRefinementDiamond = false;   // 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch
  const bool     bRasterRefinementCornersForDiamondDist1 = bExtendedSettings;
  const bool     bStarRefinementEnable                   = true;   // enable either star refinement or raster refinement
  const bool     bStarRefinementDiamond = true;   // 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch
  const bool     bStarRefinementCornersForDiamondDist1 = bExtendedSettings;
  const bool     bStarRefinementStop                   = false || bFastSettings;
  const uint32_t uiStarRefinementRounds    = 2;   // star refinement stop X rounds after best match (must be >=1)
  const bool     bNewZeroNeighbourhoodTest = bExtendedSettings;

  int searchRange = m_searchRange;
  if (m_encCfg->m_seiCfg.m_MCTSEncConstraint)
  {
    MCTSHelper::clipMvToArea(rcMv, cu.Y(), cu.cs->picture->m_mctsInfo.getTileArea(), *cu.cs->sps);
  }
  else
  {
    clipMv(rcMv, cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);
  }
  rcMv.changePrecision(MvPrecision::INTERNAL, MvPrecision::ONE);

  // init TZSearchStruct
  cStruct.uiBestSad = std::numeric_limits<Distortion>::max();

  //
  m_cDistParam.maximumDistortionForEarlyExit = cStruct.uiBestSad;
  m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd,
                           COMP_Y, cStruct.subShiftMode);

  // distortion

  // set rcMv (Median predictor) as start point and as best point
  xTZSearchHelp(cStruct, rcMv.getHor(), rcMv.getVer(), 0, 0);

  // test whether zero Mv is better start point than Median predictor
  if (bTestZeroVector)
  {
    if ((rcMv.getHor() != 0 || rcMv.getVer() != 0) && (0 != cStruct.iBestX || 0 != cStruct.iBestY))
    {
      // only test 0-vector if not obviously previously tested.
      xTZSearchHelp(cStruct, 0, 0, 0, 0);
    }
  }

  SearchRange &sr = cStruct.searchRange;

  if (pIntegerMv2Nx2NPred != 0)
  {
    Mv integerMv2Nx2NPred = *pIntegerMv2Nx2NPred;
    integerMv2Nx2NPred.changePrecision(MvPrecision::ONE, MvPrecision::INTERNAL);
    if (m_encCfg->m_seiCfg.m_MCTSEncConstraint)
    {
      MCTSHelper::clipMvToArea(integerMv2Nx2NPred, cu.Y(), cu.cs->picture->m_mctsInfo.getTileArea(), *cu.cs->sps);
    }
    else
    {
      clipMv(integerMv2Nx2NPred, cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);
    }
    integerMv2Nx2NPred.changePrecision(MvPrecision::INTERNAL, MvPrecision::ONE);

    if ((rcMv != integerMv2Nx2NPred) &&
        (integerMv2Nx2NPred.getHor() != cStruct.iBestX || integerMv2Nx2NPred.getVer() != cStruct.iBestY))
    {
      // only test integerMv2Nx2NPred if not obviously previously tested.
      xTZSearchHelp(cStruct, integerMv2Nx2NPred.getHor(), integerMv2Nx2NPred.getVer(), 0, 0);
    }
  }

  const MvPrecision tmpIntMvPrec = (cu.imv == IMV_4PEL ? MvPrecision::FOUR : MvPrecision::ONE);
  for (int i = 0; i < m_uniMvListSize; i++)
  {
    const BlkUniMvInfo *curMvInfo =
      m_uniMvList + ((m_uniMvListIdx - 1 - i + m_uniMvListMaxSize) % (m_uniMvListMaxSize));
    Mv tmpCurMv = curMvInfo->uniMvs[eRefPicList][refIdxPred];
    tmpCurMv.changePrecision(MvPrecision::INTERNAL, tmpIntMvPrec);

    int j = 0;
    for (; j < i; j++)
    {
      const BlkUniMvInfo *prevMvInfo =
        m_uniMvList + ((m_uniMvListIdx - 1 - j + m_uniMvListMaxSize) % (m_uniMvListMaxSize));
      Mv tmpPrevMv = prevMvInfo->uniMvs[eRefPicList][refIdxPred];
      tmpPrevMv.changePrecision(MvPrecision::INTERNAL, tmpIntMvPrec);
      if (tmpCurMv == tmpPrevMv)
      {
        break;
      }
    }
    if (j < i)
    {
      continue;
    }

    Mv cTmpMv = curMvInfo->uniMvs[eRefPicList][refIdxPred];
    clipMv(cTmpMv, cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);
    cTmpMv.changePrecision(MvPrecision::INTERNAL, tmpIntMvPrec);
    if (tmpIntMvPrec != MvPrecision::ONE)
    {
      cTmpMv.changePrecision(tmpIntMvPrec, MvPrecision::ONE);
    }
    m_cDistParam.cur.buf = cStruct.piRefY + (cTmpMv.ver * cStruct.iRefStride) + cTmpMv.hor;

    Distortion uiSad = m_cDistParam.distFunc(m_cDistParam);
    uiSad += m_pcRdCost->getCostOfVectorWithPredictor(cTmpMv.hor, cTmpMv.ver, cStruct.imvShift);
    if (uiSad < cStruct.uiBestSad)
    {
      cStruct.uiBestSad                          = uiSad;
      cStruct.iBestX                             = cTmpMv.hor;
      cStruct.iBestY                             = cTmpMv.ver;
      m_cDistParam.maximumDistortionForEarlyExit = uiSad;
    }
  }

  {
    // set search range
    Mv currBestMv(cStruct.iBestX, cStruct.iBestY);
    currBestMv <<= MV_FRACTIONAL_BITS_INTERNAL;
    xSetSearchRange(cu, currBestMv, m_searchRange >> (bFastSettings ? 1 : 0), sr, cStruct);
  }
  if (m_modeCtrl->useHashME && (m_currRefPicList == 0 || cu.slice->m_list1IdxToList0Idx[m_currRefPicIndex] < 0))
  {
    int minSize = std::min(cu.lumaSize().width, cu.lumaSize().height);
    if (minSize < 128 && minSize >= 4)
    {
      int numberOfOtherMvps = m_numHashMVStoreds[m_currRefPicList][m_currRefPicIndex];
      for (int i = 0; i < numberOfOtherMvps; i++)
      {
        xTZSearchHelp(cStruct, m_hashMVStoreds[m_currRefPicList][m_currRefPicIndex][i].getHor(),
                      m_hashMVStoreds[m_currRefPicList][m_currRefPicIndex][i].getVer(), 0, 0);
      }
      if (numberOfOtherMvps > 0)
      {
        // write out best match
        rcMv.set(cStruct.iBestX, cStruct.iBestY);
        ruiSAD = cStruct.uiBestSad -
          m_pcRdCost->getCostOfVectorWithPredictor(cStruct.iBestX, cStruct.iBestY, cStruct.imvShift);
        m_skipFracME = true;
        return;
      }
    }
  }

  // start search
  int iDist   = 0;
  int iStartX = cStruct.iBestX;
  int iStartY = cStruct.iBestY;

  const bool bBestCandidateZero = (cStruct.iBestX == 0) && (cStruct.iBestY == 0);

  // first search around best position up to now.
  // The following works as a "subsampled/log" window search around the best candidate
  for (iDist = 1; iDist <= searchRange; iDist *= 2)
  {
    if (bFirstSearchDiamond == 1)
    {
      xTZ8PointDiamondSearch(cStruct, iStartX, iStartY, iDist, bFirstCornersForDiamondDist1);
    }
    else
    {
      xTZ8PointSquareSearch(cStruct, iStartX, iStartY, iDist);
    }

    if (bFirstSearchStop && (cStruct.uiBestRound >= uiFirstSearchRounds))   // stop criterion
    {
      break;
    }
  }

  if (!bNewZeroNeighbourhoodTest)
  {
    // test whether zero Mv is a better start point than Median predictor
    if (bTestZeroVectorStart && ((cStruct.iBestX != 0) || (cStruct.iBestY != 0)))
    {
      xTZSearchHelp(cStruct, 0, 0, 0, 0);
      if ((cStruct.iBestX == 0) && (cStruct.iBestY == 0))
      {
        // test its neighborhood
        for (iDist = 1; iDist <= searchRange; iDist *= 2)
        {
          xTZ8PointDiamondSearch(cStruct, 0, 0, iDist, false);
          if (bTestZeroVectorStop && (cStruct.uiBestRound > 0))   // stop criterion
          {
            break;
          }
        }
      }
    }
  }
  else
  {
    // Test also zero neighbourhood but with half the range
    // It was reported that the original (above) search scheme using bTestZeroVectorStart did not
    // make sense since one would have already checked the zero candidate earlier
    // and thus the conditions for that test would have not been satisfied
    if (bTestZeroVectorStart == true && bBestCandidateZero != true)
    {
      for (iDist = 1; iDist <= (searchRange >> 1); iDist *= 2)
      {
        xTZ8PointDiamondSearch(cStruct, 0, 0, iDist, false);
        if (bTestZeroVectorStop && (cStruct.uiBestRound > 2))   // stop criterion
        {
          break;
        }
      }
    }
  }

  // calculate only 2 missing points instead 8 points if cStruct.uiBestDistance == 1
  if (cStruct.uiBestDistance == 1)
  {
    cStruct.uiBestDistance = 0;
    xTZ2PointSearch(cStruct);
  }

  // raster search if distance is too big
  if (bUseAdaptiveRaster)
  {
    int         iWindowSize = iRaster;
    SearchRange localsr     = sr;

    if (!(bEnableRasterSearch && (((int)(cStruct.uiBestDistance) >= iRaster))))
    {
      iWindowSize++;
      localsr.left /= 2;
      localsr.right /= 2;
      localsr.top /= 2;
      localsr.bottom /= 2;
    }
    cStruct.uiBestDistance = iWindowSize;
    for (iStartY = localsr.top; iStartY <= localsr.bottom; iStartY += iWindowSize)
    {
      for (iStartX = localsr.left; iStartX <= localsr.right; iStartX += iWindowSize)
      {
        xTZSearchHelp(cStruct, iStartX, iStartY, 0, iWindowSize);
      }
    }
  }
  else
  {
    if (bEnableRasterSearch && (((int)(cStruct.uiBestDistance) >= iRaster) || bAlwaysRasterSearch))
    {
      cStruct.uiBestDistance = iRaster;
      for (iStartY = sr.top; iStartY <= sr.bottom; iStartY += iRaster)
      {
        for (iStartX = sr.left; iStartX <= sr.right; iStartX += iRaster)
        {
          xTZSearchHelp(cStruct, iStartX, iStartY, 0, iRaster);
        }
      }
    }
  }

  // raster refinement

  if (bRasterRefinementEnable && cStruct.uiBestDistance > 0)
  {
    while (cStruct.uiBestDistance > 0)
    {
      iStartX = cStruct.iBestX;
      iStartY = cStruct.iBestY;
      if (cStruct.uiBestDistance > 1)
      {
        iDist = cStruct.uiBestDistance >>= 1;
        if (bRasterRefinementDiamond == 1)
        {
          xTZ8PointDiamondSearch(cStruct, iStartX, iStartY, iDist, bRasterRefinementCornersForDiamondDist1);
        }
        else
        {
          xTZ8PointSquareSearch(cStruct, iStartX, iStartY, iDist);
        }
      }

      // calculate only 2 missing points instead 8 points if cStruct.uiBestDistance == 1
      if (cStruct.uiBestDistance == 1)
      {
        cStruct.uiBestDistance = 0;
        if (cStruct.ucPointNr != 0)
        {
          xTZ2PointSearch(cStruct);
        }
      }
    }
  }

  // star refinement
  if (bStarRefinementEnable && cStruct.uiBestDistance > 0)
  {
    while (cStruct.uiBestDistance > 0)
    {
      iStartX                = cStruct.iBestX;
      iStartY                = cStruct.iBestY;
      cStruct.uiBestDistance = 0;
      cStruct.ucPointNr      = 0;
      for (iDist = 1; iDist < searchRange + 1; iDist *= 2)
      {
        if (bStarRefinementDiamond == 1)
        {
          xTZ8PointDiamondSearch(cStruct, iStartX, iStartY, iDist, bStarRefinementCornersForDiamondDist1);
        }
        else
        {
          xTZ8PointSquareSearch(cStruct, iStartX, iStartY, iDist);
        }
        if (bStarRefinementStop && (cStruct.uiBestRound >= uiStarRefinementRounds))   // stop criterion
        {
          break;
        }
      }

      // calculate only 2 missing points instead 8 points if cStrukt.uiBestDistance == 1
      if (cStruct.uiBestDistance == 1)
      {
        cStruct.uiBestDistance = 0;
        if (cStruct.ucPointNr != 0)
        {
          xTZ2PointSearch(cStruct);
        }
      }
    }
  }

  // write out best match
  rcMv.set(cStruct.iBestX, cStruct.iBestY);
  ruiSAD =
    cStruct.uiBestSad - m_pcRdCost->getCostOfVectorWithPredictor(cStruct.iBestX, cStruct.iBestY, cStruct.imvShift);
}

void InterSearch::xTZSearchSelective(const CodingUnit &cu, RefPicList eRefPicList, int refIdxPred,
                                     IntTZSearchStruct &cStruct, Mv &rcMv, Distortion &ruiSAD,
                                     const Mv *const pIntegerMv2Nx2NPred)
{
  const bool     bTestZeroVector        = true;
  const bool     bEnableRasterSearch    = true;
  const bool     bAlwaysRasterSearch    = false;   // 1: BETTER but factor 15x slower
  const bool     bStarRefinementEnable  = true;   // enable either star refinement or raster refinement
  const bool     bStarRefinementDiamond = true;   // 1 = xTZ8PointDiamondSearch   0 = xTZ8PointSquareSearch
  const bool     bStarRefinementStop    = false;
  const uint32_t uiStarRefinementRounds = 2;   // star refinement stop X rounds after best match (must be >=1)
  const int      searchRange            = m_searchRange;
  const int      iSearchRangeInitial    = m_searchRange >> 2;
  const int      uiSearchStep           = 4;
  const int      iMVDistThresh          = 8;

  int iStartX = 0;
  int iStartY = 0;
  int iDist   = 0;

  clipMv(rcMv, cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);
  rcMv.changePrecision(MvPrecision::INTERNAL, MvPrecision::ONE);

  // init TZSearchStruct
  cStruct.uiBestSad = std::numeric_limits<Distortion>::max();
  cStruct.iBestX    = 0;
  cStruct.iBestY    = 0;

  m_cDistParam.maximumDistortionForEarlyExit = cStruct.uiBestSad;
  m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd,
                           COMP_Y, cStruct.subShiftMode);

  // set rcMv (Median predictor) as start point and as best point
  xTZSearchHelp(cStruct, rcMv.getHor(), rcMv.getVer(), 0, 0);

  // test whether zero Mv is better start point than Median predictor
  if (bTestZeroVector)
  {
    xTZSearchHelp(cStruct, 0, 0, 0, 0);
  }

  SearchRange &sr = cStruct.searchRange;

  if (pIntegerMv2Nx2NPred != 0)
  {
    Mv integerMv2Nx2NPred = *pIntegerMv2Nx2NPred;
    integerMv2Nx2NPred.changePrecision(MvPrecision::ONE, MvPrecision::INTERNAL);
    clipMv(integerMv2Nx2NPred, cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);
    integerMv2Nx2NPred.changePrecision(MvPrecision::INTERNAL, MvPrecision::ONE);

    xTZSearchHelp(cStruct, integerMv2Nx2NPred.getHor(), integerMv2Nx2NPred.getVer(), 0, 0);
  }

  const MvPrecision tmpIntMvPrec = (cu.imv == IMV_4PEL ? MvPrecision::FOUR : MvPrecision::ONE);
  for (int i = 0; i < m_uniMvListSize; i++)
  {
    const BlkUniMvInfo *curMvInfo =
      m_uniMvList + ((m_uniMvListIdx - 1 - i + m_uniMvListMaxSize) % (m_uniMvListMaxSize));
    Mv tmpCurMv = curMvInfo->uniMvs[eRefPicList][refIdxPred];
    tmpCurMv.changePrecision(MvPrecision::INTERNAL, tmpIntMvPrec);

    int j = 0;
    for (; j < i; j++)
    {
      const BlkUniMvInfo *prevMvInfo =
        m_uniMvList + ((m_uniMvListIdx - 1 - j + m_uniMvListMaxSize) % (m_uniMvListMaxSize));
      Mv tmpPrevMv = prevMvInfo->uniMvs[eRefPicList][refIdxPred];
      tmpPrevMv.changePrecision(MvPrecision::INTERNAL, tmpIntMvPrec);
      if (tmpCurMv == tmpPrevMv)
      {
        break;
      }
    }
    if (j < i)
    {
      continue;
    }

    Mv cTmpMv = curMvInfo->uniMvs[eRefPicList][refIdxPred];
    clipMv(cTmpMv, cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);
    cTmpMv.changePrecision(MvPrecision::INTERNAL, tmpIntMvPrec);
    if (tmpIntMvPrec != MvPrecision::ONE)
    {
      cTmpMv.changePrecision(tmpIntMvPrec, MvPrecision::ONE);
    }
    m_cDistParam.cur.buf = cStruct.piRefY + (cTmpMv.ver * cStruct.iRefStride) + cTmpMv.hor;

    Distortion uiSad = m_cDistParam.distFunc(m_cDistParam);
    uiSad += m_pcRdCost->getCostOfVectorWithPredictor(cTmpMv.hor, cTmpMv.ver, cStruct.imvShift);
    if (uiSad < cStruct.uiBestSad)
    {
      cStruct.uiBestSad                          = uiSad;
      cStruct.iBestX                             = cTmpMv.hor;
      cStruct.iBestY                             = cTmpMv.ver;
      m_cDistParam.maximumDistortionForEarlyExit = uiSad;
    }
  }

  {
    // set search range
    Mv currBestMv(cStruct.iBestX, cStruct.iBestY);
    currBestMv <<= 2;
    xSetSearchRange(cu, currBestMv, m_searchRange, sr, cStruct);
  }
  if (m_modeCtrl->useHashME && (m_currRefPicList == 0 || cu.slice->m_list1IdxToList0Idx[m_currRefPicIndex] < 0))
  {
    int minSize = std::min(cu.lumaSize().width, cu.lumaSize().height);
    if (minSize < 128 && minSize >= 4)
    {
      int numberOfOtherMvps = m_numHashMVStoreds[m_currRefPicList][m_currRefPicIndex];
      for (int i = 0; i < numberOfOtherMvps; i++)
      {
        xTZSearchHelp(cStruct, m_hashMVStoreds[m_currRefPicList][m_currRefPicIndex][i].getHor(),
                      m_hashMVStoreds[m_currRefPicList][m_currRefPicIndex][i].getVer(), 0, 0);
      }
      if (numberOfOtherMvps > 0)
      {
        // write out best match
        rcMv.set(cStruct.iBestX, cStruct.iBestY);
        ruiSAD = cStruct.uiBestSad -
          m_pcRdCost->getCostOfVectorWithPredictor(cStruct.iBestX, cStruct.iBestY, cStruct.imvShift);
        m_skipFracME = true;
        return;
      }
    }
  }

  // Initial search
  int iBestX                = cStruct.iBestX;
  int iBestY                = cStruct.iBestY;
  int iFirstSrchRngHorLeft  = ((iBestX - iSearchRangeInitial) > sr.left) ? (iBestX - iSearchRangeInitial) : sr.left;
  int iFirstSrchRngVerTop   = ((iBestY - iSearchRangeInitial) > sr.top) ? (iBestY - iSearchRangeInitial) : sr.top;
  int iFirstSrchRngHorRight = ((iBestX + iSearchRangeInitial) < sr.right) ? (iBestX + iSearchRangeInitial) : sr.right;
  int iFirstSrchRngVerBottom =
    ((iBestY + iSearchRangeInitial) < sr.bottom) ? (iBestY + iSearchRangeInitial) : sr.bottom;

  for (iStartY = iFirstSrchRngVerTop; iStartY <= iFirstSrchRngVerBottom; iStartY += uiSearchStep)
  {
    for (iStartX = iFirstSrchRngHorLeft; iStartX <= iFirstSrchRngHorRight; iStartX += uiSearchStep)
    {
      xTZSearchHelp(cStruct, iStartX, iStartY, 0, 0);
      xTZ8PointDiamondSearch(cStruct, iStartX, iStartY, 1, false);
      xTZ8PointDiamondSearch(cStruct, iStartX, iStartY, 2, false);
    }
  }

  int iMaxMVDistToPred = (abs(cStruct.iBestX - iBestX) > iMVDistThresh || abs(cStruct.iBestY - iBestY) > iMVDistThresh);

  // full search with early exit if MV is distant from predictors
  if (bEnableRasterSearch && (iMaxMVDistToPred || bAlwaysRasterSearch))
  {
    for (iStartY = sr.top; iStartY <= sr.bottom; iStartY += 1)
    {
      for (iStartX = sr.left; iStartX <= sr.right; iStartX += 1)
      {
        xTZSearchHelp(cStruct, iStartX, iStartY, 0, 1);
      }
    }
  }
  // Smaller MV, refine around predictor
  else if (bStarRefinementEnable && cStruct.uiBestDistance > 0)
  {
    // start refinement
    while (cStruct.uiBestDistance > 0)
    {
      iStartX                = cStruct.iBestX;
      iStartY                = cStruct.iBestY;
      cStruct.uiBestDistance = 0;
      cStruct.ucPointNr      = 0;
      for (iDist = 1; iDist < searchRange + 1; iDist *= 2)
      {
        if (bStarRefinementDiamond == 1)
        {
          xTZ8PointDiamondSearch(cStruct, iStartX, iStartY, iDist, false);
        }
        else
        {
          xTZ8PointSquareSearch(cStruct, iStartX, iStartY, iDist);
        }
        if (bStarRefinementStop && (cStruct.uiBestRound >= uiStarRefinementRounds))   // stop criterion
        {
          break;
        }
      }

      // calculate only 2 missing points instead 8 points if cStrukt.uiBestDistance == 1
      if (cStruct.uiBestDistance == 1)
      {
        cStruct.uiBestDistance = 0;
        if (cStruct.ucPointNr != 0)
        {
          xTZ2PointSearch(cStruct);
        }
      }
    }
  }

  // write out best match
  rcMv.set(cStruct.iBestX, cStruct.iBestY);
  ruiSAD =
    cStruct.uiBestSad - m_pcRdCost->getCostOfVectorWithPredictor(cStruct.iBestX, cStruct.iBestY, cStruct.imvShift);
}

void InterSearch::xPatternSearchIntRefine(CodingUnit &cu, IntTZSearchStruct &cStruct, Mv &rcMv, Mv &rcMvPred,
                                          int &riMVPIdx, uint32_t &ruiBits, Distortion &ruiCost,
                                          const AMVPInfo &amvpInfo, double fWeight)
{

  CHECK(cu.imv == 0 || cu.imv == IMV_HPEL, "xPatternSearchIntRefine(): Sub-pel MV used.");
  CHECK(amvpInfo.mvCand[riMVPIdx] != rcMvPred, "xPatternSearchIntRefine(): MvPred issue.");

  const SPS &sps = *cu.cs->sps;
  m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY, cStruct.iRefStride, m_lumaClpRng.bd,
                           COMP_Y, 0, 1, (m_encCfg->m_bUseHADME && !cu.cs->slice->m_disableSATDForRd) ? 1 : 0);

  // -> set MV scale for cost calculation to QPEL (0)
  m_pcRdCost->setCostScale(0);

  Distortion dist, uiSATD = 0;
  Distortion bestDist = std::numeric_limits<Distortion>::max();
  // subtract old MVP costs because costs for all newly tested MVPs are added in here
  ruiBits -= m_auiMVPIdxCost[riMVPIdx][AMVP_MAX_NUM_CANDS];

  Mv  cBestMv = rcMv;
  Mv  cBaseMvd[2];
  int iBestBits   = 0;
  int iBestMVPIdx = riMVPIdx;
  Mv  testPos[9]  = { { 0, 0 }, { -1, -1 }, { -1, 0 }, { -1, 1 }, { 0, -1 }, { 0, 1 }, { 1, -1 }, { 1, 0 }, { 1, 1 } };

  cBaseMvd[0] = (rcMv - amvpInfo.mvCand[0]);
  cBaseMvd[1] = (rcMv - amvpInfo.mvCand[1]);
  CHECK((cBaseMvd[0].getHor() & 0x03) != 0 || (cBaseMvd[0].getVer() & 0x03) != 0,
        "xPatternSearchIntRefine(): AMVP cand 0 Mvd issue.");
  CHECK((cBaseMvd[1].getHor() & 0x03) != 0 || (cBaseMvd[1].getVer() & 0x03) != 0,
        "xPatternSearchIntRefine(): AMVP cand 1 Mvd issue.");

  cBaseMvd[0].roundTransPrecInternal2Amvr(cu.imv);
  cBaseMvd[1].roundTransPrecInternal2Amvr(cu.imv);

  // test best integer position and all 8 neighboring positions
  for (int pos = 0; pos < 9; pos++)
  {
    Mv cTestMv[2];
    // test both AMVP candidates for each position
    for (int mvpIdx = 0; mvpIdx < amvpInfo.numCand; mvpIdx++)
    {
      cTestMv[mvpIdx] = testPos[pos];
      cTestMv[mvpIdx].changeTransPrecAmvr2Internal(cu.imv);
      cTestMv[mvpIdx] += cBaseMvd[mvpIdx];
      cTestMv[mvpIdx] += amvpInfo.mvCand[mvpIdx];

      // MCTS and IMV
      if (m_encCfg->m_seiCfg.m_MCTSEncConstraint)
      {
        Mv cTestMVRestr = cTestMv[mvpIdx];
        MCTSHelper::clipMvToArea(cTestMVRestr, cu.Y(), cu.cs->picture->m_mctsInfo.getTileAreaIntPelRestricted(cu),
                                 *cu.cs->sps);

        if (cTestMVRestr != cTestMv[mvpIdx])
        {
          // Skip this IMV pos, cause clipping affects IMV scaling
          continue;
        }
      }
      if (mvpIdx == 0 || cTestMv[0] != cTestMv[1])
      {
        Mv cTempMV = cTestMv[mvpIdx];
        if (!m_encCfg->m_seiCfg.m_MCTSEncConstraint)
        {
          clipMv(cTempMV, cu.lumaPos(), cu.lumaSize(), sps, *cu.cs->pps);
        }
        m_cDistParam.cur.buf = cStruct.piRefY + cStruct.iRefStride * (cTempMV.getVer() >> MV_FRACTIONAL_BITS_INTERNAL) +
          (cTempMV.getHor() >> MV_FRACTIONAL_BITS_INTERNAL);
        dist = uiSATD = (Distortion)(m_cDistParam.distFunc(m_cDistParam) * fWeight);
      }
      else
      {
        dist = uiSATD;
      }

      int iMvBits = m_auiMVPIdxCost[mvpIdx][AMVP_MAX_NUM_CANDS];
      Mv  pred    = amvpInfo.mvCand[mvpIdx];
      pred.changeTransPrecInternal2Amvr(cu.imv);
      m_pcRdCost->setPredictor(pred);
      Mv mv = cTestMv[mvpIdx];
      mv.changeTransPrecInternal2Amvr(cu.imv);
      iMvBits += m_pcRdCost->getBitsOfVectorWithPredictor(mv.getHor(), mv.getVer(), 0);
      dist += m_pcRdCost->getCost(iMvBits);

      if (dist < bestDist)
      {
        bestDist    = dist;
        cBestMv     = cTestMv[mvpIdx];
        iBestMVPIdx = mvpIdx;
        iBestBits   = iMvBits;
      }
    }
  }
  if (bestDist == std::numeric_limits<Distortion>::max())
  {
    ruiCost = std::numeric_limits<Distortion>::max();
    return;
  }

  rcMv     = cBestMv;
  rcMvPred = amvpInfo.mvCand[iBestMVPIdx];
  riMVPIdx = iBestMVPIdx;
  m_pcRdCost->setPredictor(rcMvPred);

  ruiBits += iBestBits;
  // taken from JEM 5.0
  // verify since it makes no sence to subtract Lamda*(Rmvd+Rmvpidx) from D+Lamda(Rmvd)
  // this would take the rate for the MVP idx out of the cost calculation
  // however this rate is always 1 so impact is small
  ruiCost = bestDist - m_pcRdCost->getCost(iBestBits) + m_pcRdCost->getCost(ruiBits);
  // taken from JEM 5.0
  // verify since it makes no sense to add rate for MVDs twicce

  return;
}

void InterSearch::xPatternSearchFracDIF(const CodingUnit &cu, RefPicList eRefPicList, int refIdx,
                                        IntTZSearchStruct &cStruct, const Mv &rcMvInt, Mv &rcMvHalf, Mv &rcMvQter,
                                        Distortion &ruiCost)
{
  PROFILER_SCOPE(0, g_timeProfiler, P_FRAC_PEL_SEARCH);

  //  Reference pattern initialization (integer scale)
  ptrdiff_t offset = rcMvInt.getHor() + rcMvInt.getVer() * cStruct.iRefStride;
  CPelBuf   cPatternRoi(cStruct.piRefY + offset, cStruct.iRefStride, *cStruct.pcPatternKey);
  if (m_skipFracME)
  {
    Mv baseRefMv(0, 0);
    rcMvHalf.setZero();
    m_pcRdCost->setCostScale(0);
    xExtDIFUpSamplingH(&cPatternRoi, cStruct.useAltHpelIf);
    rcMvQter = rcMvInt;
    rcMvQter <<= 2;   // for mv-cost
    ruiCost = xPatternRefinement(cStruct.pcPatternKey, baseRefMv, 1, rcMvQter, !cu.cs->slice->m_disableSATDForRd);
    return;
  }

  if (cStruct.imvShift > IMV_FPEL || (m_useCompositeRef && cStruct.zeroMV))
  {
    m_pcRdCost->setDistParam(m_cDistParam, *cStruct.pcPatternKey, cStruct.piRefY + offset, cStruct.iRefStride,
                             m_lumaClpRng.bd, COMP_Y, 0, 1,
                             (m_encCfg->m_bUseHADME && !cu.cs->slice->m_disableSATDForRd) ? 1 : 0);
    ruiCost = m_cDistParam.distFunc(m_cDistParam);
    ruiCost += m_pcRdCost->getCostOfVectorWithPredictor(rcMvInt.getHor(), rcMvInt.getVer(), cStruct.imvShift);
    return;
  }

  //  Half-pel refinement
  m_pcRdCost->setCostScale(1);
  xExtDIFUpSamplingH(&cPatternRoi, cStruct.useAltHpelIf);

  rcMvHalf = rcMvInt;
  rcMvHalf <<= 1;   // for mv-cost
  Mv baseRefMv(0, 0);
  ruiCost = xPatternRefinement(cStruct.pcPatternKey, baseRefMv, 2, rcMvHalf, (!cu.cs->slice->m_disableSATDForRd));

  //  quarter-pel refinement
  if (cStruct.imvShift == IMV_OFF)
  {
    m_pcRdCost->setCostScale(0);
    xExtDIFUpSamplingQ(&cPatternRoi, rcMvHalf);
    baseRefMv = rcMvHalf;
    baseRefMv <<= 1;

    rcMvQter = rcMvInt;
    rcMvQter <<= 1;   // for mv-cost
    rcMvQter += rcMvHalf;
    rcMvQter <<= 1;
    ruiCost = xPatternRefinement(cStruct.pcPatternKey, baseRefMv, 1, rcMvQter, (!cu.cs->slice->m_disableSATDForRd));
  }
}

Distortion InterSearch::xGetSymmetricCost(CodingUnit &cu, PelUnitBuf &origBuf, RefPicList eCurRefPicList,
                                          const MvField &cCurMvField, MvField &cTarMvField, int bcwIdx)
{
  Distortion cost           = std::numeric_limits<Distortion>::max();
  RefPicList eTarRefPicList = (RefPicList)(1 - (int)eCurRefPicList);

  // get prediction of eCurRefPicList
  PelUnitBuf     predBufA = m_tmpPredStorage[eCurRefPicList].getBuf(UnitAreaRelative(cu, cu));
  const Picture *picRefA  = cu.slice->getRefPic(eCurRefPicList, cCurMvField.refIdx);
  Mv             mvA      = cCurMvField.mv;
  clipMv(mvA, cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);
  if ((mvA.hor & 15) == 0 && (mvA.ver & 15) == 0 && !cu.licFlag)
  {
    Position offset  = cu.blocks[COMP_Y].pos().offset(mvA.getHor() >> 4, mvA.getVer() >> 4);
    CPelBuf  pelBufA = picRefA->getRecoBuf(CompArea(COMP_Y, cu.chromaFormat, offset, cu.blocks[COMP_Y].size()), false);
    predBufA.bufs[0].buf    = const_cast<Pel *>(pelBufA.buf);
    predBufA.bufs[0].stride = pelBufA.stride;
    predBufA.bufs[0].width  = pelBufA.width;
    predBufA.bufs[0].height = pelBufA.height;
  }
  else
  {
    xPredInterBlk(COMP_Y, cu, picRefA, mvA, predBufA, false, cu.slice->clpRng(COMP_Y), false, false, eCurRefPicList);
  }

  // get prediction of eTarRefPicList
  PelUnitBuf     predBufB = m_tmpPredStorage[eTarRefPicList].getBuf(UnitAreaRelative(cu, cu));
  const Picture *picRefB  = cu.slice->getRefPic(eTarRefPicList, cTarMvField.refIdx);
  Mv             mvB      = cTarMvField.mv;
  clipMv(mvB, cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);
  if ((mvB.hor & 15) == 0 && (mvB.ver & 15) == 0 && !cu.licFlag)
  {
    Position offset  = cu.blocks[COMP_Y].pos().offset(mvB.getHor() >> 4, mvB.getVer() >> 4);
    CPelBuf  pelBufB = picRefB->getRecoBuf(CompArea(COMP_Y, cu.chromaFormat, offset, cu.blocks[COMP_Y].size()), false);
    predBufB.bufs[0].buf    = const_cast<Pel *>(pelBufB.buf);
    predBufB.bufs[0].stride = pelBufB.stride;
  }
  else
  {
    xPredInterBlk(COMP_Y, cu, picRefB, mvB, predBufB, false, cu.slice->clpRng(COMP_Y), false, false, eTarRefPicList);
  }

  PelUnitBuf bufTmp = m_tmpStorageCtu.getBuf(UnitAreaRelative(cu, cu));
  bufTmp.copyFrom(origBuf);
  bufTmp.removeHighFreq(predBufA, m_encCfg->m_bClipForBiPredMeEnabled, cu.slice->m_clpRngs,
                        getBcwWeight(cu.bcwIdx, eTarRefPicList));
  double fWeight = xGetMEDistortionWeight(cu.bcwIdx, eTarRefPicList);

  // calc distortion
  const DFunc distFunc = (!cu.slice->m_disableSATDForRd) ? DFunc::HAD : DFunc::SAD;
  cost                 = (Distortion)floor(fWeight *
                                           (double)m_pcRdCost->getDistPart(bufTmp.Y(), predBufB.Y(),
                                                                           cu.cs->sps->m_bitDepths[ChannelType::LUMA], COMP_Y, distFunc,
                                                                           nullptr, cu.cs->sps->m_bitDepths[ChannelType::LUMA] + 4));
  return (cost);
}

Distortion InterSearch::xSymmeticRefineMvSearch(CodingUnit &cu, PelUnitBuf &origBuf, Mv &rcMvCurPred, Mv &rcMvTarPred,
                                                RefPicList eRefPicList, MvField &rCurMvField, MvField &rTarMvField,
                                                Distortion uiMinCost, int SearchPattern, int nSearchStepShift,
                                                uint32_t uiMaxSearchRounds, int bcwIdx)
{
  const Mv mvSearchOffsetCross[4]   = { Mv(0, 1), Mv(1, 0), Mv(0, -1), Mv(-1, 0) };
  const Mv mvSearchOffsetSquare[8]  = { Mv(-1, 1), Mv(0, 1),  Mv(1, 1),   Mv(1, 0),
                                        Mv(1, -1), Mv(0, -1), Mv(-1, -1), Mv(-1, 0) };
  const Mv mvSearchOffsetDiamond[8] = { Mv(0, 2),  Mv(1, 1),   Mv(2, 0),  Mv(1, -1),
                                        Mv(0, -2), Mv(-1, -1), Mv(-2, 0), Mv(-1, 1) };
  const Mv mvSearchOffsetHexagon[6] = { Mv(2, 0), Mv(1, 2), Mv(-1, 2), Mv(-2, 0), Mv(-1, -2), Mv(1, -2) };

  int       nDirectStart = 0, nDirectEnd = 0, nDirectRounding = 0, nDirectMask = 0;
  const Mv *pSearchOffset;
  if (SearchPattern == 0)
  {
    nDirectEnd      = 3;
    nDirectRounding = 4;
    nDirectMask     = 0x03;
    pSearchOffset   = mvSearchOffsetCross;
  }
  else if (SearchPattern == 1)
  {
    nDirectEnd      = 7;
    nDirectRounding = 8;
    nDirectMask     = 0x07;
    pSearchOffset   = mvSearchOffsetSquare;
  }
  else if (SearchPattern == 2)
  {
    nDirectEnd      = 7;
    nDirectRounding = 8;
    nDirectMask     = 0x07;
    pSearchOffset   = mvSearchOffsetDiamond;
  }
  else if (SearchPattern == 3)
  {
    nDirectEnd    = 5;
    pSearchOffset = mvSearchOffsetHexagon;
  }
  else
  {
    THROW("Invalid search pattern");
  }

  int nBestDirect;
  for (uint32_t uiRound = 0; uiRound < uiMaxSearchRounds; uiRound++)
  {
    Distortion roundZeroBestCost = MAX_DISTORTION;
    const int  positionLut[8]    = { 0, 2, 4, 6, 1, 3, 5, 7 };
    nBestDirect                  = -1;
    MvField mvCurCenter          = rCurMvField;
    for (int nIdx = nDirectStart; nIdx <= nDirectEnd; nIdx++)
    {
      // terminate the search if none of the first four tested points hasn't provided improvement
      if (m_encCfg->m_SMVD > 1 && 2 == SearchPattern && 0 == uiRound && 4 == nIdx && roundZeroBestCost > uiMinCost)
      {
        break;
      }
      int nDirect;
      if (SearchPattern == 3)
      {
        nDirect = nIdx < 0 ? nIdx + 6 : nIdx >= 6 ? nIdx - 6 : nIdx;
      }
      else
      {
        if (m_encCfg->m_SMVD > 1 && 2 == SearchPattern && 0 == uiRound)
        {
          nDirect = positionLut[(nIdx + nDirectRounding) & nDirectMask];
        }
        else
        {
          nDirect = (nIdx + nDirectRounding) & nDirectMask;
        }
      }

      Mv mvOffset = pSearchOffset[nDirect];
      mvOffset <<= nSearchStepShift;
      MvField mvCand = mvCurCenter, mvPair;
      mvCand.mv += mvOffset;

      if (m_encCfg->m_seiCfg.m_MCTSEncConstraint)
      {
        if (!(MCTSHelper::checkMvForMCTSConstraint(cu, mvCand.mv)))
        {
          continue;   // Skip this this pos
        }
      }
      // get MVD cost
      Mv pred = rcMvCurPred;
      pred.changeTransPrecInternal2Amvr(cu.imv);
      m_pcRdCost->setPredictor(pred);
      m_pcRdCost->setCostScale(0);
      Mv mv = mvCand.mv;
      mv.changeTransPrecInternal2Amvr(cu.imv);
      uint32_t   uiMvBits = m_pcRdCost->getBitsOfVectorWithPredictor(mv.getHor(), mv.getVer(), 0);
      Distortion uiCost   = m_pcRdCost->getCost(uiMvBits);

      // get MVD pair and set target MV
      mvPair.refIdx = rTarMvField.refIdx;
      mvPair.mv.set(rcMvTarPred.hor - (mvCand.mv.hor - rcMvCurPred.hor),
                    rcMvTarPred.ver - (mvCand.mv.ver - rcMvCurPred.ver));
      if (m_encCfg->m_seiCfg.m_MCTSEncConstraint)
      {
        if (!(MCTSHelper::checkMvForMCTSConstraint(cu, mvPair.mv)))
        {
          continue;   // Skip this this pos
        }
      }
      uiCost += xGetSymmetricCost(cu, origBuf, eRefPicList, mvCand, mvPair, bcwIdx);

      if (uiCost < uiMinCost)
      {
        uiMinCost   = uiCost;
        rCurMvField = mvCand;
        rTarMvField = mvPair;
        nBestDirect = nDirect;
      }
      if (m_encCfg->m_SMVD > 1 && 2 == SearchPattern && 0 == uiRound && 4 > nIdx && uiCost < roundZeroBestCost)
      {
        roundZeroBestCost = uiCost;
      }
    }

    if (nBestDirect == -1)
    {
      break;
    }
    int nStep = 1;
    if ((SearchPattern == 1 || SearchPattern == 2) && m_encCfg->m_SMVD <= 1)
    {
      // test at most 3 points in fast presets
      nStep = 2 - (nBestDirect & 0x01);
    }
    nDirectStart = nBestDirect - nStep;
    nDirectEnd   = nBestDirect + nStep;
  }

  return (uiMinCost);
}

void InterSearch::xSymmetricMotionEstimation(CodingUnit &cu, PelUnitBuf &origBuf, Mv &rcMvCurPred, Mv &rcMvTarPred,
                                             RefPicList eRefPicList, MvField &rCurMvField, MvField &rTarMvField,
                                             Distortion &ruiCost, int bcwIdx)
{
  // Refine Search
  int nSearchStepShift = MV_FRACTIONAL_BITS_DIFF;
  int nDiamondRound    = 8;
  int nCrossRound      = 1;

  nSearchStepShift += cu.imv == IMV_HPEL ? 1 : (cu.imv << 1);
  nDiamondRound >>= cu.imv;

  ruiCost = xSymmeticRefineMvSearch(cu, origBuf, rcMvCurPred, rcMvTarPred, eRefPicList, rCurMvField, rTarMvField,
                                    ruiCost, 2, nSearchStepShift, nDiamondRound, bcwIdx);
  if (m_encCfg->m_SMVD < 3)
  {
    ruiCost = xSymmeticRefineMvSearch(cu, origBuf, rcMvCurPred, rcMvTarPred, eRefPicList, rCurMvField, rTarMvField,
                                      ruiCost, 0, nSearchStepShift, nCrossRound, bcwIdx);
  }
}

void InterSearch::xPredAffineInterSearch(CodingUnit &cu, PelUnitBuf &origBuf, int puIdx, uint32_t &lastMode,
                                         Distortion &affineCost, RefSetArray<Mv> &hevcMv,
                                         RefSetArray<Mv[3]> &mvAffine4Para, int refIdx4Para[NUM_RPL01], uint8_t bcwIdx,
                                         bool enforceBcwPred, uint32_t bcwIdxBits)
{
  PROFILER_SCOPE(1, g_timeProfiler, P_INTER_MVD_SEARCH_AFFINE);
  const Slice &slice = *cu.slice;

  affineCost = std::numeric_limits<Distortion>::max();

  Mv                 cMvZero;
  Mv                 aacMv[NUM_RPL01][3];
  Mv                 cMvBi[NUM_RPL01][3];
  RefSetArray<Mv[3]> cMvTemp;

  int iNumPredDir = slice.isInterP() ? 1 : 2;

  const int mvNum = cu.getNumAffineMvs();

  // Mvp
  RefSetArray<Mv[3]> cMvPred;
  RefSetArray<Mv[3]> cMvPredBi;
  RefSetArray<int>   aaiMvpIdxBi;
  RefSetArray<int>   aaiMvpIdx;
  RefSetArray<int>   aaiMvpNum;

  RefSetArray<AffineAMVPInfo> aacAffineAMVPInfo;
  AffineAMVPInfo              affiAMVPInfoTemp[NUM_RPL01];

  int refIdx[NUM_RPL01] = { 0, 0 };   // If un-initialized, may cause SEGV in bi-directional prediction iterative stage.
  int iRefIdxBi[NUM_RPL01];

  uint32_t mbBits[3] = { 1, 1, 0 };

  int iRefStart, iRefEnd;

  int        bestBiPRefIdxL1 = 0;
  int        bestBiPMvpL1    = 0;
  Distortion biPDistTemp     = std::numeric_limits<Distortion>::max();

  Distortion uiCost[2] = { std::numeric_limits<Distortion>::max(), std::numeric_limits<Distortion>::max() };
  Distortion costBi    = MAX_DISTORTION;
  Distortion costTemp;
  costTemp = std::numeric_limits<Distortion>::max();

  uint32_t   bits[3] = { 0 };
  uint32_t   bitsTemp;
  Distortion bestBiPDist = std::numeric_limits<Distortion>::max();

  Distortion uiCostTempL0[MAX_NUM_REF];
  for (int iNumRef = 0; iNumRef < MAX_NUM_REF; iNumRef++)
  {
    uiCostTempL0[iNumRef] = std::numeric_limits<Distortion>::max();
  }

  uint32_t uiBitsTempL0[MAX_NUM_REF];

  Mv              mvValidList1[4];
  int             refIdxValidList1 = 0;
  uint32_t        bitsValidList1   = MAX_UINT;
  Distortion      costValidList1   = std::numeric_limits<Distortion>::max();
  Mv              mvHevc[3];
  const bool      affineAmvrEnabled = cu.slice->m_sps->m_affineAmvrEnabledFlag;
  int             tryBipred         = 0;
  WPScalingParam *wp0;
  WPScalingParam *wp1;
  xGetBlkBits(slice.isInterP(), mbBits);

  cu.affine           = true;
  cu.mergeFlag        = false;
  cu.regularMergeFlag = false;
  if (bcwIdx != BCW_DEFAULT)
  {
    cu.bcwIdx = bcwIdx;
  }

  // Uni-directional prediction
  for (int refList = 0; refList < iNumPredDir; refList++)
  {
    RefPicList eRefPicList = (refList ? RPL1 : RPL0);
    cu.interDir            = (refList ? 2 : 1);
    for (int refIdxTemp = 0; refIdxTemp < slice.m_numRefIdx[eRefPicList]; refIdxTemp++)
    {
      // Get RefIdx bits
      bitsTemp = mbBits[refList];
      if (slice.m_numRefIdx[eRefPicList] > 1)
      {
        bitsTemp += refIdxTemp + 1;
        if (refIdxTemp == slice.m_numRefIdx[eRefPicList] - 1)
        {
          bitsTemp--;
        }
      }

      // Do Affine AMVP
      xEstimateAffineAMVP(cu, affiAMVPInfoTemp[eRefPicList], origBuf, eRefPicList, refIdxTemp,
                          cMvPred[refList][refIdxTemp], &biPDistTemp);
      if (affineAmvrEnabled)
      {
        biPDistTemp +=
          m_pcRdCost->getCost(xCalcAffineMVBits(cu, cMvPred[refList][refIdxTemp], cMvPred[refList][refIdxTemp]));
      }
      aaiMvpIdx[refList][refIdxTemp] = cu.mvpIdx[eRefPicList];
      aaiMvpNum[refList][refIdxTemp] = cu.mvpNum[eRefPicList];

      if (cu.affineType == AffineModel::_6_PARAMS && refIdx4Para[refList] != refIdxTemp)
      {
        xCopyAffineAMVPInfo(affiAMVPInfoTemp[eRefPicList], aacAffineAMVPInfo[refList][refIdxTemp]);
        continue;
      }

      // set hevc ME result as start search position when it is best than mvp
      for (int i = 0; i < 3; i++)
      {
        mvHevc[i] = hevcMv[refList][refIdxTemp];
        mvHevc[i].roundAffinePrecInternal2Amvr(cu.imv);
      }
      PelUnitBuf predBuf    = m_tmpStorageCtu.getBuf(UnitAreaRelative(cu, cu));
      Distortion uiCandCost = xGetAffineTemplateCost(cu, origBuf, predBuf, mvHevc, aaiMvpIdx[refList][refIdxTemp],
                                                     AMVP_MAX_NUM_CANDS, eRefPicList, refIdxTemp);

      if (affineAmvrEnabled)
      {
        uiCandCost += m_pcRdCost->getCost(xCalcAffineMVBits(cu, mvHevc, cMvPred[refList][refIdxTemp]));
      }

      // check stored affine motion
      bool affine4Para    = cu.affineType == AffineModel::_4_PARAMS;
      bool savedParaAvail = cu.imv &&
        ((m_affineMotion.affine4ParaRefIdx[refList] == refIdxTemp && affine4Para && m_affineMotion.affine4ParaAvail) ||
         (m_affineMotion.affine6ParaRefIdx[refList] == refIdxTemp && !affine4Para && m_affineMotion.affine6ParaAvail));

      if (savedParaAvail)
      {
        Mv mvFour[3];
        for (int i = 0; i < mvNum; i++)
        {
          mvFour[i] =
            affine4Para ? m_affineMotion.acMvAffine4Para[refList][i] : m_affineMotion.acMvAffine6Para[refList][i];
          mvFour[i].roundAffinePrecInternal2Amvr(cu.imv);
        }

        Distortion candCostInherit = xGetAffineTemplateCost(
          cu, origBuf, predBuf, mvFour, aaiMvpIdx[refList][refIdxTemp], AMVP_MAX_NUM_CANDS, eRefPicList, refIdxTemp);
        candCostInherit += m_pcRdCost->getCost(xCalcAffineMVBits(cu, mvFour, cMvPred[refList][refIdxTemp]));

        if (candCostInherit < uiCandCost)
        {
          uiCandCost = candCostInherit;
          memcpy(mvHevc, mvFour, 3 * sizeof(Mv));
        }
      }

      if (cu.affineType == AffineModel::_4_PARAMS && m_affMVListSize &&
          (!cu.cs->sps->m_useBcw || bcwIdx == BCW_DEFAULT))
      {
        int shift = MAX_CU_DEPTH;
        for (int i = 0; i < m_affMVListSize; i++)
        {
          AffineMVInfo *mvInfo = m_affMVList + ((m_affMVListIdx - i - 1 + m_affMVListMaxSize) % (m_affMVListMaxSize));

          // check;
          int j = 0;
          for (; j < i; j++)
          {
            AffineMVInfo *prevMvInfo =
              m_affMVList + ((m_affMVListIdx - j - 1 + m_affMVListMaxSize) % (m_affMVListMaxSize));
            if ((mvInfo->affMVs[refList][refIdxTemp][0] == prevMvInfo->affMVs[refList][refIdxTemp][0]) &&
                (mvInfo->affMVs[refList][refIdxTemp][1] == prevMvInfo->affMVs[refList][refIdxTemp][1]) &&
                (mvInfo->x == prevMvInfo->x) && (mvInfo->y == prevMvInfo->y) && (mvInfo->w == prevMvInfo->w))
            {
              break;
            }
          }
          if (j < i)
          {
            continue;
          }

          Mv  mvTmp[3], *nbMv = mvInfo->affMVs[refList][refIdxTemp];
          int vx, vy;
          int dMvHorX, dMvHorY, dMvVerX, dMvVerY;
          int mvScaleHor = nbMv[0].getHor() * (1 << shift);
          int mvScaleVer = nbMv[0].getVer() * (1 << shift);
          Mv  dMv        = nbMv[1] - nbMv[0];

          dMvHorX = dMv.getHor() * (1 << (shift - floorLog2(mvInfo->w)));
          dMvHorY = dMv.getVer() * (1 << (shift - floorLog2(mvInfo->w)));
          dMvVerX = -dMvHorY;
          dMvVerY = dMvHorX;

          vx = mvScaleHor + dMvHorX * (cu.lx() - mvInfo->x) + dMvVerX * (cu.ly() - mvInfo->y);
          vy = mvScaleVer + dMvHorY * (cu.lx() - mvInfo->x) + dMvVerY * (cu.ly() - mvInfo->y);

          mvTmp[0] = Mv(vx, vy);
          mvTmp[0] >>= shift;
          mvTmp[0].clipToStorageBitDepth();
          clipMv(mvTmp[0], cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);
          mvTmp[0].roundAffinePrecInternal2Amvr(cu.imv);

          vx = mvScaleHor + dMvHorX * (cu.lx() + cu.lwidth() - mvInfo->x) + dMvVerX * (cu.ly() - mvInfo->y);
          vy = mvScaleVer + dMvHorY * (cu.lx() + cu.lwidth() - mvInfo->x) + dMvVerY * (cu.ly() - mvInfo->y);

          mvTmp[1] = Mv(vx, vy);
          mvTmp[1] >>= shift;
          mvTmp[1].clipToStorageBitDepth();
          clipMv(mvTmp[1], cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);
          mvTmp[1].roundAffinePrecInternal2Amvr(cu.imv);

          Distortion tmpCost = xGetAffineTemplateCost(cu, origBuf, predBuf, mvTmp, aaiMvpIdx[refList][refIdxTemp],
                                                      AMVP_MAX_NUM_CANDS, eRefPicList, refIdxTemp);
          if (affineAmvrEnabled)
          {
            tmpCost += m_pcRdCost->getCost(xCalcAffineMVBits(cu, mvTmp, cMvPred[refList][refIdxTemp]));
          }

          if (tmpCost < uiCandCost)
          {
            uiCandCost = tmpCost;
            std::memcpy(mvHevc, mvTmp, 3 * sizeof(Mv));
          }
        }
      }
      if (cu.affineType == AffineModel::_6_PARAMS)
      {
        Mv mvFour[3];
        mvFour[0] = mvAffine4Para[refList][refIdxTemp][0];
        mvFour[1] = mvAffine4Para[refList][refIdxTemp][1];

        mvAffine4Para[refList][refIdxTemp][0].roundAffinePrecInternal2Amvr(cu.imv);
        mvAffine4Para[refList][refIdxTemp][1].roundAffinePrecInternal2Amvr(cu.imv);

        int shift = MAX_CU_DEPTH;
        int vx2   = (mvFour[0].getHor() * (1 << shift)) -
          ((mvFour[1].getVer() - mvFour[0].getVer()) *
           (1 << (shift + floorLog2(cu.lheight()) - floorLog2(cu.lwidth()))));
        int vy2 = (mvFour[0].getVer() * (1 << shift)) +
          ((mvFour[1].getHor() - mvFour[0].getHor()) *
           (1 << (shift + floorLog2(cu.lheight()) - floorLog2(cu.lwidth()))));
        int offset    = (1 << (shift - 1));
        vx2           = (vx2 + offset - (vx2 >= 0)) >> shift;
        vy2           = (vy2 + offset - (vy2 >= 0)) >> shift;
        mvFour[2].hor = vx2;
        mvFour[2].ver = vy2;
        mvFour[2].clipToStorageBitDepth();
        mvFour[0].roundAffinePrecInternal2Amvr(cu.imv);
        mvFour[1].roundAffinePrecInternal2Amvr(cu.imv);
        mvFour[2].roundAffinePrecInternal2Amvr(cu.imv);

        Distortion uiCandCostInherit = xGetAffineTemplateCost(
          cu, origBuf, predBuf, mvFour, aaiMvpIdx[refList][refIdxTemp], AMVP_MAX_NUM_CANDS, eRefPicList, refIdxTemp);

        if (affineAmvrEnabled)
        {
          uiCandCostInherit += m_pcRdCost->getCost(xCalcAffineMVBits(cu, mvFour, cMvPred[refList][refIdxTemp]));
        }

        if (uiCandCostInherit < uiCandCost)
        {
          uiCandCost = uiCandCostInherit;
          for (int i = 0; i < 3; i++)
          {
            mvHevc[i] = mvFour[i];
          }
        }
      }

      if (uiCandCost < biPDistTemp)
      {
        ::memcpy(cMvTemp[refList][refIdxTemp], mvHevc, sizeof(Mv) * 3);
      }
      else
      {
        ::memcpy(cMvTemp[refList][refIdxTemp], cMvPred[refList][refIdxTemp], sizeof(Mv) * 3);
      }

      // GPB list 1, save the best MvpIdx, RefIdx and Cost

      if (slice.m_picHeader->m_mvdL1ZeroFlag && refList == 1 && biPDistTemp < bestBiPDist)
      {
        bestBiPDist     = biPDistTemp;
        bestBiPMvpL1    = aaiMvpIdx[refList][refIdxTemp];
        bestBiPRefIdxL1 = refIdxTemp;
      }

      // Update bits
      bitsTemp += m_auiMVPIdxCost[aaiMvpIdx[refList][refIdxTemp]][AMVP_MAX_NUM_CANDS];

      if (m_encCfg->m_bFastMEForGenBLowDelayEnabled && refList == 1)   // list 1
      {
        if (slice.m_list1IdxToList0Idx[refIdxTemp] >= 0 &&
            (cu.affineType != AffineModel::_6_PARAMS || slice.m_list1IdxToList0Idx[refIdxTemp] == refIdx4Para[0]))
        {
          int iList1ToList0Idx = slice.m_list1IdxToList0Idx[refIdxTemp];
          ::memcpy(cMvTemp[1][refIdxTemp], cMvTemp[0][iList1ToList0Idx], sizeof(Mv) * 3);
          costTemp = uiCostTempL0[iList1ToList0Idx];

          costTemp -= m_pcRdCost->getCost(uiBitsTempL0[iList1ToList0Idx]);
          bitsTemp += xCalcAffineMVBits(cu, cMvTemp[refList][refIdxTemp], cMvPred[refList][refIdxTemp]);
          /*calculate the correct cost*/
          costTemp += m_pcRdCost->getCost(bitsTemp);
          DTRACE(g_trace_ctx, D_COMMON, " (%d) costTemp=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON), costTemp);
        }
        else
        {
          xAffineMotionEstimation(cu, origBuf, eRefPicList, cMvPred[refList][refIdxTemp], refIdxTemp,
                                  cMvTemp[refList][refIdxTemp], bitsTemp, costTemp, aaiMvpIdx[refList][refIdxTemp],
                                  affiAMVPInfoTemp[eRefPicList]);
        }
      }
      else
      {
        xAffineMotionEstimation(cu, origBuf, eRefPicList, cMvPred[refList][refIdxTemp], refIdxTemp,
                                cMvTemp[refList][refIdxTemp], bitsTemp, costTemp, aaiMvpIdx[refList][refIdxTemp],
                                affiAMVPInfoTemp[eRefPicList]);
      }
      if (cu.cs->sps->m_useBcw && cu.bcwIdx == BCW_DEFAULT && cu.slice->isInterB())
      {
        m_uniMotions.setReadModeAffine(true, (uint8_t)refList, (uint8_t)refIdxTemp, cu.affineType);
        m_uniMotions.copyAffineMvFrom(cMvTemp[refList][refIdxTemp], costTemp - m_pcRdCost->getCost(bitsTemp),
                                      (uint8_t)refList, (uint8_t)refIdxTemp, cu.affineType,
                                      aaiMvpIdx[refList][refIdxTemp]);
      }
      // Set best AMVP Index
      xCopyAffineAMVPInfo(affiAMVPInfoTemp[eRefPicList], aacAffineAMVPInfo[refList][refIdxTemp]);
      if (cu.imv != 2 || !m_encCfg->m_AffineAmvrEncOpt)
      {
        xCheckBestAffineMVP(cu, affiAMVPInfoTemp[eRefPicList], eRefPicList, cMvTemp[refList][refIdxTemp],
                            cMvPred[refList][refIdxTemp], aaiMvpIdx[refList][refIdxTemp], bitsTemp, costTemp);
      }

      if (refList == 0)
      {
        uiCostTempL0[refIdxTemp] = costTemp;
        uiBitsTempL0[refIdxTemp] = bitsTemp;
      }
      DTRACE(g_trace_ctx, D_COMMON, " (%d) costTemp=%d, uiCost[refList]=%d\n",
             DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON), costTemp, uiCost[refList]);

      if (costTemp < uiCost[refList])
      {
        uiCost[refList] = costTemp;
        bits[refList]   = bitsTemp;   // storing for bi-prediction

        // set best motion
        ::memcpy(aacMv[refList], cMvTemp[refList][refIdxTemp], sizeof(Mv) * 3);
        refIdx[refList] = refIdxTemp;
      }

      if (refList == 1 && costTemp < costValidList1 && slice.m_list1IdxToList0Idx[refIdxTemp] < 0)
      {
        costValidList1 = costTemp;
        bitsValidList1 = bitsTemp;

        // set motion
        memcpy(mvValidList1, cMvTemp[refList][refIdxTemp], sizeof(Mv) * 3);

        refIdxValidList1 = refIdxTemp;
      }
    }   // End refIdx loop
  }   // end Uni-prediction

  if (cu.affineType == AffineModel::_4_PARAMS)
  {
    ::memcpy(mvAffine4Para, cMvTemp, sizeof(cMvTemp));
    if (cu.imv == 0 && (!cu.cs->sps->m_useBcw || bcwIdx == BCW_DEFAULT))
    {
      AffineMVInfo *affMVInfo = m_affMVList + m_affMVListIdx;

      // check;
      int j = 0;
      for (; j < m_affMVListSize; j++)
      {
        AffineMVInfo *prevMvInfo = m_affMVList + ((m_affMVListIdx - j - 1 + m_affMVListMaxSize) % (m_affMVListMaxSize));
        if ((cu.lx() == prevMvInfo->x) && (cu.ly() == prevMvInfo->y) && (cu.lwidth() == prevMvInfo->w) &&
            (cu.lheight() == prevMvInfo->h))
        {
          break;
        }
      }
      if (j < m_affMVListSize)
      {
        affMVInfo = m_affMVList + ((m_affMVListIdx - j - 1 + m_affMVListMaxSize) % (m_affMVListMaxSize));
      }
      ::memcpy(affMVInfo->affMVs, cMvTemp, sizeof(cMvTemp));

      if (j == m_affMVListSize)
      {
        affMVInfo->x    = cu.lx();
        affMVInfo->y    = cu.ly();
        affMVInfo->w    = cu.lwidth();
        affMVInfo->h    = cu.lheight();
        m_affMVListSize = std::min(m_affMVListSize + 1, m_affMVListMaxSize);
        m_affMVListIdx  = (m_affMVListIdx + 1) % (m_affMVListMaxSize);
      }
    }
  }

  // Bi-directional prediction
  if (slice.isInterB() && !PU::isBipredRestriction(cu) && (cu.cs->sps->m_biLicEnabledFlag || !cu.licFlag))
  {
    tryBipred            = 1;
    cu.interDir          = 3;
    m_biPredSearchAffine = true;
    // Set as best list0 and list1
    iRefIdxBi[0]         = refIdx[0];
    iRefIdxBi[1]         = refIdx[1];

    ::memcpy(cMvBi, aacMv, sizeof(aacMv));
    ::memcpy(cMvPredBi, cMvPred, sizeof(cMvPred));
    ::memcpy(aaiMvpIdxBi, aaiMvpIdx, sizeof(aaiMvpIdx));

    uint32_t motBits[2];
    bool     doBiPred = true;

    if (slice.m_picHeader->m_mvdL1ZeroFlag)   // GPB, list 1 only use Mvp
    {
      xCopyAffineAMVPInfo(aacAffineAMVPInfo[1][bestBiPRefIdxL1], affiAMVPInfoTemp[RPL1]);
      cu.mvpIdx[RPL1]                 = bestBiPMvpL1;
      aaiMvpIdxBi[1][bestBiPRefIdxL1] = bestBiPMvpL1;

      // Set Mv for list1
      Mv pcMvTemp[3] = { affiAMVPInfoTemp[RPL1].mvCandLT[bestBiPMvpL1], affiAMVPInfoTemp[RPL1].mvCandRT[bestBiPMvpL1],
                         affiAMVPInfoTemp[RPL1].mvCandLB[bestBiPMvpL1] };
      ::memcpy(cMvPredBi[1][bestBiPRefIdxL1], pcMvTemp, sizeof(Mv) * 3);
      ::memcpy(cMvBi[1], pcMvTemp, sizeof(Mv) * 3);
      ::memcpy(cMvTemp[1][bestBiPRefIdxL1], pcMvTemp, sizeof(Mv) * 3);
      iRefIdxBi[1] = bestBiPRefIdxL1;

      if (m_encCfg->m_seiCfg.m_MCTSEncConstraint)
      {
        Area curTileAreaRestricted;
        curTileAreaRestricted = cu.cs->picture->m_mctsInfo.getTileAreaSubPelRestricted(cu);
        for (int i = 0; i < mvNum; i++)
        {
          Mv restrictedMv = pcMvTemp[i];
          MCTSHelper::clipMvToArea(restrictedMv, cu.Y(), curTileAreaRestricted, *cu.cs->sps);

          // If sub-pel filter samples are not inside of allowed area
          if (restrictedMv != pcMvTemp[i])
          {
            costBi   = MAX_DISTORTION;
            doBiPred = false;
          }
        }
      }
      // Get list1 prediction block
      PU::setAllAffineMv(cu, cMvBi[1][0], cMvBi[1][1], cMvBi[1][2], RPL1);
      cu.refIdx[RPL1] = iRefIdxBi[1];

      PelUnitBuf predBufTmp = m_tmpPredStorage[RPL1].getBuf(UnitAreaRelative(cu, cu));
      motionCompensation(cu, predBufTmp, RPL1);

      // Update bits
      motBits[0] = bits[0] - mbBits[0];
      motBits[1] = mbBits[1];

      if (slice.m_numRefIdx[RPL1] > 1)
      {
        motBits[1] += bestBiPRefIdxL1 + 1;
        if (bestBiPRefIdxL1 == slice.m_numRefIdx[RPL1] - 1)
        {
          motBits[1]--;
        }
      }
      motBits[1] += m_auiMVPIdxCost[aaiMvpIdxBi[1][bestBiPRefIdxL1]][AMVP_MAX_NUM_CANDS];
      bits[2] = mbBits[2] + motBits[0] + motBits[1];
    }
    else
    {
      motBits[0] = bits[0] - mbBits[0];
      motBits[1] = bits[1] - mbBits[1];
      bits[2]    = mbBits[2] + motBits[0] + motBits[1];
    }

    if (doBiPred)
    {
      // 4-times iteration (default)
      int numIter = 4;
      // fast encoder setting or GPB: only one iteration
      if (m_encCfg->m_fastInterSearchMode == FASTINTERSEARCH_MODE1 ||
          m_encCfg->m_fastInterSearchMode == FASTINTERSEARCH_MODE2 || slice.m_picHeader->m_mvdL1ZeroFlag)
      {
        numIter = 1;
      }

      for (int iter = 0; iter < numIter; iter++)
      {
        // Set RefList
        int refList = iter % 2;
        if (m_encCfg->m_fastInterSearchMode == FASTINTERSEARCH_MODE1 ||
            m_encCfg->m_fastInterSearchMode == FASTINTERSEARCH_MODE2)
        {

          if (uiCost[0] <= uiCost[1])
          {
            refList = 1;
          }
          else
          {
            refList = 0;
          }
          if (bcwIdx != BCW_DEFAULT)
          {
            refList = (abs(getBcwWeight(bcwIdx, RPL0)) > abs(getBcwWeight(bcwIdx, RPL1)) ? 1 : 0);
          }
        }
        else if (iter == 0)
        {
          refList = 0;
        }

        // First iterate, get prediction block of opposite direction
        if (iter == 0 && !slice.m_picHeader->m_mvdL1ZeroFlag)
        {
          PU::setAllAffineMv(cu, aacMv[1 - refList][0], aacMv[1 - refList][1], aacMv[1 - refList][2],
                             RefPicList(1 - refList));
          cu.refIdx[1 - refList] = refIdx[1 - refList];

          PelUnitBuf predBufTmp = m_tmpPredStorage[1 - refList].getBuf(UnitAreaRelative(cu, cu));
          motionCompensation(cu, predBufTmp, RefPicList(1 - refList));
        }

        RefPicList eRefPicList = (refList ? RPL1 : RPL0);

        if (slice.m_picHeader->m_mvdL1ZeroFlag)   // GPB, fix List 1, search List 0
        {
          refList     = 0;
          eRefPicList = RPL0;
        }

        bool changed = false;

        iRefStart = 0;
        iRefEnd   = slice.m_numRefIdx[eRefPicList] - 1;
        for (int refIdxTemp = iRefStart; refIdxTemp <= iRefEnd; refIdxTemp++)
        {
          if (cu.affineType == AffineModel::_6_PARAMS && refIdx4Para[refList] != refIdxTemp)
          {
            continue;
          }
          // update bits
          bitsTemp = mbBits[2] + motBits[1 - refList];
          bitsTemp += (cu.slice->m_sps->m_useBcw ? bcwIdxBits : 0);
          if (slice.m_numRefIdx[eRefPicList] > 1)
          {
            bitsTemp += refIdxTemp + 1;
            if (refIdxTemp == slice.m_numRefIdx[eRefPicList] - 1)
            {
              bitsTemp--;
            }
          }
          bitsTemp += m_auiMVPIdxCost[aaiMvpIdxBi[refList][refIdxTemp]][AMVP_MAX_NUM_CANDS];

          // call Affine ME
          xAffineMotionEstimation(cu, origBuf, eRefPicList, cMvPredBi[refList][refIdxTemp], refIdxTemp,
                                  cMvTemp[refList][refIdxTemp], bitsTemp, costTemp, aaiMvpIdxBi[refList][refIdxTemp],
                                  aacAffineAMVPInfo[refList][refIdxTemp], true);

          xCopyAffineAMVPInfo(aacAffineAMVPInfo[refList][refIdxTemp], affiAMVPInfoTemp[eRefPicList]);
          if (cu.imv != 2 || !m_encCfg->m_AffineAmvrEncOpt)
          {
            xCheckBestAffineMVP(cu, affiAMVPInfoTemp[eRefPicList], eRefPicList, cMvTemp[refList][refIdxTemp],
                                cMvPredBi[refList][refIdxTemp], aaiMvpIdxBi[refList][refIdxTemp], bitsTemp, costTemp);
          }

          if (costTemp < costBi)
          {
            changed = true;
            ::memcpy(cMvBi[refList], cMvTemp[refList][refIdxTemp], sizeof(Mv) * 3);
            iRefIdxBi[refList] = refIdxTemp;

            costBi           = costTemp;
            motBits[refList] = bitsTemp - mbBits[2] - motBits[1 - refList];
            motBits[refList] -= (cu.slice->m_sps->m_useBcw ? bcwIdxBits : 0);
            bits[2] = bitsTemp;

            if (numIter != 1)   // MC for next iter
            {
              //  Set motion
              PU::setAllAffineMv(cu, cMvBi[refList][0], cMvBi[refList][1], cMvBi[refList][2], eRefPicList);
              cu.refIdx[eRefPicList] = iRefIdxBi[eRefPicList];

              PelUnitBuf predBufTmp = m_tmpPredStorage[refList].getBuf(UnitAreaRelative(cu, cu));
              motionCompensation(cu, predBufTmp, eRefPicList);
            }
          }
        }   // for loop-refIdxTemp

        if (!changed)
        {

          if ((costBi <= uiCost[0] && costBi <= uiCost[1]) || enforceBcwPred)
          {
            xCopyAffineAMVPInfo(aacAffineAMVPInfo[0][iRefIdxBi[0]], affiAMVPInfoTemp[RPL0]);
            xCheckBestAffineMVP(cu, affiAMVPInfoTemp[RPL0], RPL0, cMvBi[0], cMvPredBi[0][iRefIdxBi[0]],
                                aaiMvpIdxBi[0][iRefIdxBi[0]], bits[2], costBi);

            if (!slice.m_picHeader->m_mvdL1ZeroFlag)
            {
              xCopyAffineAMVPInfo(aacAffineAMVPInfo[1][iRefIdxBi[1]], affiAMVPInfoTemp[RPL1]);
              xCheckBestAffineMVP(cu, affiAMVPInfoTemp[RPL1], RPL1, cMvBi[1], cMvPredBi[1][iRefIdxBi[1]],
                                  aaiMvpIdxBi[1][iRefIdxBi[1]], bits[2], costBi);
            }
          }
          break;
        }
      }   // for loop-iter
    }
    m_biPredSearchAffine = false;
  }   // if (B_SLICE)

  cu.mv[RPL0]     = Mv();
  cu.mv[RPL1]     = Mv();
  cu.mvd[RPL0]    = cMvZero;
  cu.mvd[RPL1]    = cMvZero;
  cu.refIdx[RPL0] = NOT_VALID;
  cu.refIdx[RPL1] = NOT_VALID;
  cu.mvpIdx[RPL0] = NOT_VALID;
  cu.mvpIdx[RPL1] = NOT_VALID;
  cu.mvpNum[RPL0] = NOT_VALID;
  cu.mvpNum[RPL1] = NOT_VALID;

  for (int verIdx = 0; verIdx < 3; verIdx++)
  {
    cu.mvdAffi[RPL0][verIdx] = cMvZero;
    cu.mvdAffi[RPL1][verIdx] = cMvZero;
  }

  // Set Motion Field
  memcpy(aacMv[1], mvValidList1, sizeof(Mv) * 3);
  refIdx[1] = refIdxValidList1;
  bits[1]   = bitsValidList1;
  uiCost[1] = costValidList1;

  if (cu.cs->pps->m_useBiWP == true && tryBipred && (bcwIdx != BCW_DEFAULT))
  {
    CHECK(iRefIdxBi[0] < 0, "Invalid picture reference index");
    CHECK(iRefIdxBi[1] < 0, "Invalid picture reference index");
    wp0 = cu.cs->slice->getWpScaling(RPL0, iRefIdxBi[0]);
    wp1 = cu.cs->slice->getWpScaling(RPL1, iRefIdxBi[1]);

    if (WPScalingParam::isWeighted(wp0) || WPScalingParam::isWeighted(wp1))
    {
      costBi         = MAX_DISTORTION;
      enforceBcwPred = false;
    }
  }
  if (enforceBcwPred)
  {
    uiCost[0] = uiCost[1] = MAX_DISTORTION;
  }

  // Affine ME result set

  if (costBi <= uiCost[0] && costBi <= uiCost[1])   // Bi
  {
    lastMode    = 2;
    affineCost  = costBi;
    cu.interDir = 3;
    PU::setAllAffineMv(cu, cMvBi[0][0], cMvBi[0][1], cMvBi[0][2], RPL0);
    PU::setAllAffineMv(cu, cMvBi[1][0], cMvBi[1][1], cMvBi[1][2], RPL1);
    cu.refIdx[RPL0] = iRefIdxBi[0];
    cu.refIdx[RPL1] = iRefIdxBi[1];

    for (int verIdx = 0; verIdx < mvNum; verIdx++)
    {
      cu.mvdAffi[RPL0][verIdx] = cMvBi[0][verIdx] - cMvPredBi[0][iRefIdxBi[0]][verIdx];
      cu.mvdAffi[RPL1][verIdx] = cMvBi[1][verIdx] - cMvPredBi[1][iRefIdxBi[1]][verIdx];
      if (verIdx != 0)
      {
        cu.mvdAffi[0][verIdx] = cu.mvdAffi[0][verIdx] - cu.mvdAffi[0][0];
        cu.mvdAffi[1][verIdx] = cu.mvdAffi[1][verIdx] - cu.mvdAffi[1][0];
      }
    }

    cu.mvpIdx[RPL0] = aaiMvpIdxBi[0][iRefIdxBi[0]];
    cu.mvpNum[RPL0] = aaiMvpNum[0][iRefIdxBi[0]];
    cu.mvpIdx[RPL1] = aaiMvpIdxBi[1][iRefIdxBi[1]];
    cu.mvpNum[RPL1] = aaiMvpNum[1][iRefIdxBi[1]];
  }
  else if (uiCost[0] <= uiCost[1])   // List 0
  {
    lastMode    = 0;
    affineCost  = uiCost[0];
    cu.interDir = 1;
    PU::setAllAffineMv(cu, aacMv[0][0], aacMv[0][1], aacMv[0][2], RPL0);
    cu.refIdx[RPL0] = refIdx[0];

    for (int verIdx = 0; verIdx < mvNum; verIdx++)
    {
      cu.mvdAffi[RPL0][verIdx] = aacMv[0][verIdx] - cMvPred[0][refIdx[0]][verIdx];
      if (verIdx != 0)
      {
        cu.mvdAffi[0][verIdx] = cu.mvdAffi[0][verIdx] - cu.mvdAffi[0][0];
      }
    }

    cu.mvpIdx[RPL0] = aaiMvpIdx[0][refIdx[0]];
    cu.mvpNum[RPL0] = aaiMvpNum[0][refIdx[0]];
  }
  else
  {
    lastMode    = 1;
    affineCost  = uiCost[1];
    cu.interDir = 2;
    PU::setAllAffineMv(cu, aacMv[1][0], aacMv[1][1], aacMv[1][2], RPL1);
    cu.refIdx[RPL1] = refIdx[1];

    for (int verIdx = 0; verIdx < mvNum; verIdx++)
    {
      cu.mvdAffi[RPL1][verIdx] = aacMv[1][verIdx] - cMvPred[1][refIdx[1]][verIdx];
      if (verIdx != 0)
      {
        cu.mvdAffi[1][verIdx] = cu.mvdAffi[1][verIdx] - cu.mvdAffi[1][0];
      }
    }

    cu.mvpIdx[RPL1] = aaiMvpIdx[1][refIdx[1]];
    cu.mvpNum[RPL1] = aaiMvpNum[1][refIdx[1]];
  }
  if (bcwIdx != BCW_DEFAULT)
  {
    cu.bcwIdx = BCW_DEFAULT;
  }
}

// Ax = b, m = {A, b}
void solveGaussElimination(double (*m)[7], double *x, int num)
{
#define NEARZERO(x) x == 0.

  const int numM1 = num - 1;

  for (int i = 0; i < numM1; i++)
  {
    // find non-zero diag
    int tempIdx = i;
    if (NEARZERO(m[i][i]))
    {
      for (int j = i + 1; j < num; j++)
      {
        if (!(NEARZERO(m[j][i])))
        {
          tempIdx = j;
          break;
        }
      }
    }

    // swap line
    if (tempIdx != i)
    {
      std::swap(m[i], m[tempIdx]);
    }

    double      *currRow   = m[i];
    const double diagCoeff = currRow[i];

    if (NEARZERO(diagCoeff))
    {
      std::memset(x, 0, sizeof(*x) * num);
      return;
    }

    // eliminate column
    for (int j = i + 1; j < num; j++)
    {
      double      *rowCoeff   = m[j];
      const double coeffRatio = rowCoeff[i] / diagCoeff;

      for (int k = i + 1; k <= num; k++)
      {
        rowCoeff[k] -= currRow[k] * coeffRatio;
      }
    }
  }

  if (NEARZERO(m[numM1][numM1]))
  {
    std::memset(x, 0, sizeof(*x) * num);
    return;
  }

  double *currRow = m[numM1];
  x[numM1]        = currRow[num] / currRow[numM1];

  for (int i = num - 2; i >= 0; i--)
  {
    currRow                = m[i];
    const double diagCoeff = currRow[i];

    if (NEARZERO(diagCoeff))
    {
      std::memset(x, 0, sizeof(*x) * num);
      return;
    }

    double temp = 0;
    for (int j = i + 1; j < num; j++)
    {
      temp += currRow[j] * x[j];
    }
    x[i] = (currRow[num] - temp) / diagCoeff;
  }
#undef NEARZERO
}

void InterSearch::xCheckBestAffineMVP(CodingUnit &cu, AffineAMVPInfo &affineAMVPInfo, RefPicList eRefPicList,
                                      Mv acMv[3], Mv acMvPred[3], int &riMVPIdx, uint32_t &ruiBits, Distortion &ruiCost)
{

  if (affineAMVPInfo.numCand < 2)
  {
    return;
  }

  const int mvNum = cu.getNumAffineMvs();

  m_pcRdCost->selectMotionLambda();
  m_pcRdCost->setCostScale(0);

  int iBestMVPIdx = riMVPIdx;

  // Get origin MV bits
  Mv  tmpPredMv[3];
  int iOrgMvBits = xCalcAffineMVBits(cu, acMv, acMvPred);
  iOrgMvBits += m_auiMVPIdxCost[riMVPIdx][AMVP_MAX_NUM_CANDS];

  int iBestMvBits = iOrgMvBits;
  for (int mvpIdx = 0; mvpIdx < affineAMVPInfo.numCand; mvpIdx++)
  {
    if (mvpIdx == riMVPIdx)
    {
      continue;
    }
    tmpPredMv[0] = affineAMVPInfo.mvCandLT[mvpIdx];
    tmpPredMv[1] = affineAMVPInfo.mvCandRT[mvpIdx];
    if (mvNum == 3)
    {
      tmpPredMv[2] = affineAMVPInfo.mvCandLB[mvpIdx];
    }
    int iMvBits = xCalcAffineMVBits(cu, acMv, tmpPredMv);
    iMvBits += m_auiMVPIdxCost[mvpIdx][AMVP_MAX_NUM_CANDS];

    if (iMvBits < iBestMvBits)
    {
      iBestMvBits = iMvBits;
      iBestMVPIdx = mvpIdx;
    }
  }

  if (iBestMVPIdx != riMVPIdx)   // if changed
  {
    acMvPred[0]        = affineAMVPInfo.mvCandLT[iBestMVPIdx];
    acMvPred[1]        = affineAMVPInfo.mvCandRT[iBestMVPIdx];
    acMvPred[2]        = affineAMVPInfo.mvCandLB[iBestMVPIdx];
    riMVPIdx           = iBestMVPIdx;
    uint32_t uiOrgBits = ruiBits;
    ruiBits            = uiOrgBits - iOrgMvBits + iBestMvBits;
    ruiCost            = (ruiCost - m_pcRdCost->getCost(uiOrgBits)) + m_pcRdCost->getCost(ruiBits);
  }
}

void InterSearch::xAffineMotionEstimation(CodingUnit &cu, PelUnitBuf &origBuf, RefPicList eRefPicList, Mv acMvPred[3],
                                          int refIdxPred, Mv acMv[3], uint32_t &ruiBits, Distortion &ruiCost,
                                          int &mvpIdx, const AffineAMVPInfo &aamvpi, bool bBi)
{
  if (cu.cs->sps->m_useBcw && cu.bcwIdx != BCW_DEFAULT && !bBi &&
      xReadBufferedAffineUniMv(cu, eRefPicList, refIdxPred, acMvPred, acMv, ruiBits, ruiCost, mvpIdx, aamvpi))
  {
    return;
  }

  uint32_t  dirBits    = ruiBits - m_auiMVPIdxCost[mvpIdx][aamvpi.numCand];
  int       bestMvpIdx = mvpIdx;
  const int width      = cu.lwidth();
  const int height     = cu.lheight();

  const Picture *refPic = cu.slice->getRefPic(eRefPicList, refIdxPred);

  // Set Origin YUV: pcYuv
  PelUnitBuf *pBuf    = &origBuf;
  double      fWeight = 1.0;

  PelUnitBuf  origBufTmp = m_tmpStorageCtu.getBuf(UnitAreaRelative(cu, cu));
  const DFunc distFunc   = (cu.cs->slice->m_disableSATDForRd) ? DFunc::SAD : DFunc::HAD;

  // if Bi, set to ( 2 * Org - ListX )
  if (bBi)
  {
    // NOTE: Other buf contains predicted signal from another direction
    PelUnitBuf otherBuf = m_tmpPredStorage[1 - (int)eRefPicList].getBuf(UnitAreaRelative(cu, cu));
    origBufTmp.copyFrom(origBuf);
    origBufTmp.removeHighFreq(otherBuf, m_encCfg->m_bClipForBiPredMeEnabled, cu.slice->m_clpRngs,
                              getBcwWeight(cu.bcwIdx, eRefPicList));
    pBuf = &origBufTmp;

    fWeight = xGetMEDistortionWeight(cu.bcwIdx, eRefPicList);
  }

  // pred YUV
  PelUnitBuf predBuf = m_tmpAffiStorage.getBuf(UnitAreaRelative(cu, cu));

  // Set start Mv position, use input mv as started search mv
  Mv acMvTemp[3];
  ::memcpy(acMvTemp, acMv, sizeof(Mv) * 3);

  // Set delta mv
  // malloc buffer
  const int  mvNum         = cu.getNumAffineMvs();
  const int  affineParaNum = 2 * mvNum;
  const int  iParaNum      = affineParaNum + 1;
  Pel       *piError       = m_tmpAffiError;
  Distortion uiCostBest    = std::numeric_limits<Distortion>::max();
  uint32_t   uiBitsBest    = 0;
  int64_t    i64EqualCoeff[7][7];

  Pel *pdDerivate[2];
  pdDerivate[0] = m_gradX0 + PROF_BORDER_EXT_H * width + PROF_BORDER_EXT_W;
  pdDerivate[1] = m_gradY0 + PROF_BORDER_EXT_H * width + PROF_BORDER_EXT_W;

  // do motion compensation with origin mv
  if (m_encCfg->m_seiCfg.m_MCTSEncConstraint)
  {
    Area curTileAreaRestricted = cu.cs->picture->m_mctsInfo.getTileAreaSubPelRestricted(cu);
    for (int i = 0; i < cu.getNumAffineMvs(); i++)
    {
      MCTSHelper::clipMvToArea(acMvTemp[i], cu.Y(), curTileAreaRestricted, *cu.cs->sps);
    }
  }
  else
  {
    for (int i = 0; i < cu.getNumAffineMvs(); i++)
    {
      clipMv(acMvTemp[i], cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);
    }
  }
  for (int i = 0; i < cu.getNumAffineMvs(); i++)
  {
    acMvTemp[i].roundAffinePrecInternal2Amvr(cu.imv);
  }

  xPredAffineBlk(COMP_Y, cu, refPic, acMvTemp, predBuf, false, cu.cs->slice->clpRng(COMP_Y), eRefPicList, false,
                 SCALE_1X, true);

  // get error
  uiCostBest = m_pcRdCost->getDistPart(predBuf.Y(), pBuf->Y(), cu.cs->sps->m_bitDepths[ChannelType::LUMA], COMP_Y,
                                       distFunc, nullptr, cu.cs->sps->m_bitDepths[ChannelType::LUMA] + 4);

  // get cost with mv
  m_pcRdCost->setCostScale(0);
  uiBitsBest = ruiBits;
  if (cu.imv == 2 && m_encCfg->m_AffineAmvrEncOpt)
  {
    uiBitsBest  = dirBits + xDetermineBestMvp(cu, acMvTemp, mvpIdx, aamvpi);
    acMvPred[0] = aamvpi.mvCandLT[mvpIdx];
    acMvPred[1] = aamvpi.mvCandRT[mvpIdx];
    acMvPred[2] = aamvpi.mvCandLB[mvpIdx];
  }
  else
  {
    DTRACE(g_trace_ctx, D_COMMON, " (%d) xx uiBitsBest=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON), uiBitsBest);
    uiBitsBest += xCalcAffineMVBits(cu, acMvTemp, acMvPred);
    DTRACE(g_trace_ctx, D_COMMON, " (%d) yy uiBitsBest=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON), uiBitsBest);
  }

  uiCostBest = (Distortion)(floor(fWeight * (double)uiCostBest) + (double)m_pcRdCost->getCost(uiBitsBest));

  DTRACE(g_trace_ctx, D_COMMON, " (%d) uiBitsBest=%d, uiCostBest=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON),
         uiBitsBest, uiCostBest);

  ::memcpy(acMv, acMvTemp, sizeof(Mv) * 3);

  const ptrdiff_t bufStride     = pBuf->Y().stride;
  const ptrdiff_t predBufStride = predBuf.Y().stride;
  Mv              prevIterMv[7][3];
  int             iIterTime;
  if (cu.affineType == AffineModel::_6_PARAMS)
  {
    iIterTime = bBi ? 3 : 4;
  }
  else
  {
    iIterTime = bBi ? 3 : 5;
  }

  if (!cu.cs->sps->m_AffineType)
  {
    iIterTime = bBi ? 5 : 7;
  }
  for (int iter = 0; iter < iIterTime; iter++)   // iterate loop
  {
    memcpy(prevIterMv[iter], acMvTemp, sizeof(Mv) * 3);
    /*********************************************************************************
     *                         use gradient to update mv
     *********************************************************************************/
    // get Error Matrix
    Pel *pOrg  = pBuf->Y().buf;
    Pel *pPred = predBuf.Y().buf;
    Pel *error = piError;
    for (int j = 0; j < height; j++)
    {
      for (int i = 0; i < width; i++)
      {
        error[i] = pOrg[i] - pPred[i];
      }
      pOrg += bufStride;
      pPred += predBufStride;
      error += width;
    }

    // solve delta x and y
    for (int row = 0; row < iParaNum; row++)
    {
      memset(&i64EqualCoeff[row][0], 0, iParaNum * sizeof(int64_t));
    }

    // the "6" is the shift number in gradient (calculated in IF_INTERNAL_PREC precision), "-1" is for gradient
    // normalization the input parameter "shift" in is to compensate dI with regard to the gradient
    const int bs = 6 - 1 - std::max<int>(2, (IF_INTERNAL_PREC - cu.cs->slice->clpRng(COMP_Y).bd));
    m_EqualCoeffComputer[cu.affineType == AffineModel::_6_PARAMS](piError, width, pdDerivate, width, i64EqualCoeff,
                                                                  width, height, bs);

    double dAffinePara[6];

    double pdEqualCoeff[6][7];
    for (int row = 0; row < affineParaNum; row++)
    {
      double  *dCoeff = pdEqualCoeff[row];
      int64_t *iCoeff = i64EqualCoeff[row + 1];
      for (int i = 0; i < iParaNum; i++)
      {
        dCoeff[i] = (double)iCoeff[i];
      }
    }

    solveGaussElimination(pdEqualCoeff, dAffinePara, affineParaNum);

    double dDeltaMv[6] = {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    };
    Mv acDeltaMv[3];

    // convert to delta mv
    dDeltaMv[0] = dAffinePara[0];
    dDeltaMv[2] = dAffinePara[2];
    if (cu.affineType == AffineModel::_6_PARAMS)
    {
      dDeltaMv[1] = dAffinePara[1] * width + dAffinePara[0];
      dDeltaMv[3] = dAffinePara[3] * width + dAffinePara[2];
      dDeltaMv[4] = dAffinePara[4] * height + dAffinePara[0];
      dDeltaMv[5] = dAffinePara[5] * height + dAffinePara[2];
    }
    else
    {
      dDeltaMv[1] = dAffinePara[1] * width + dAffinePara[0];
      dDeltaMv[3] = -dAffinePara[3] * width + dAffinePara[2];
    }

    for (int i = 0; i < 6; i++)
    {
      dDeltaMv[i] = Clip3(-8192.0, 8192.0, dDeltaMv[i]);
    }

    const double amvrScale = Mv::getAffineAmvrScale(cu.imv);

    acDeltaMv[0] = Mv((int)(dDeltaMv[0] * amvrScale + sgn2(dDeltaMv[0]) * 0.5),
                      (int)(dDeltaMv[2] * amvrScale + sgn2(dDeltaMv[2]) * 0.5));
    acDeltaMv[1] = Mv((int)(dDeltaMv[1] * amvrScale + sgn2(dDeltaMv[1]) * 0.5),
                      (int)(dDeltaMv[3] * amvrScale + sgn2(dDeltaMv[3]) * 0.5));

    acDeltaMv[0].changeAffinePrecAmvr2Internal(cu.imv);
    acDeltaMv[1].changeAffinePrecAmvr2Internal(cu.imv);

    if (cu.affineType == AffineModel::_6_PARAMS)
    {
      acDeltaMv[2] = Mv((int)(dDeltaMv[4] * amvrScale + sgn2(dDeltaMv[4]) * 0.5),
                        (int)(dDeltaMv[5] * amvrScale + sgn2(dDeltaMv[5]) * 0.5));
      acDeltaMv[2].changeAffinePrecAmvr2Internal(cu.imv);
    }
    if (!m_encCfg->m_AffineAmvrEncOpt)
    {
      bool allZero = true;
      for (int i = 0; i < mvNum; i++)
      {
        const Mv &deltaMv = acDeltaMv[i];
        if (deltaMv.getHor() != 0 || deltaMv.getVer() != 0)
        {
          allZero = false;
          break;
        }
      }

      if (allZero)
      {
        break;
      }
    }
    // do motion compensation with updated mv
    for (int i = 0; i < mvNum; i++)
    {
      acMvTemp[i] += acDeltaMv[i];
      acMvTemp[i].clipToStorageBitDepth();
      acMvTemp[i].roundAffinePrecInternal2Amvr(cu.imv);
      if (m_encCfg->m_seiCfg.m_MCTSEncConstraint)
      {
        MCTSHelper::clipMvToArea(acMvTemp[i], cu.Y(), cu.cs->picture->m_mctsInfo.getTileAreaSubPelRestricted(cu),
                                 *cu.cs->sps);
      }
      else
      {
        clipMv(acMvTemp[i], cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);
      }
    }

    if (m_encCfg->m_AffineAmvrEncOpt)
    {
      bool identical = false;
      for (int k = iter; k >= 0; k--)
      {
        if (acMvTemp[0] == prevIterMv[k][0] && acMvTemp[1] == prevIterMv[k][1])
        {
          identical = cu.affineType == AffineModel::_6_PARAMS ? acMvTemp[2] == prevIterMv[k][2] : true;
          if (identical)
          {
            break;
          }
        }
      }
      if (identical)
      {
        break;
      }
    }

    xPredAffineBlk(COMP_Y, cu, refPic, acMvTemp, predBuf, false, cu.slice->clpRng(COMP_Y), eRefPicList, false, SCALE_1X,
                   true);

    // get error
    Distortion costTemp =
      m_pcRdCost->getDistPart(predBuf.Y(), pBuf->Y(), cu.cs->sps->m_bitDepths[ChannelType::LUMA], COMP_Y, distFunc,
                              nullptr, cu.cs->sps->m_bitDepths[ChannelType::LUMA] + 4);
    DTRACE(g_trace_ctx, D_COMMON, " (%d) costTemp=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON), costTemp);

    // get cost with mv
    m_pcRdCost->setCostScale(0);
    uint32_t bitsTemp = ruiBits;
    if (cu.imv == 2 && m_encCfg->m_AffineAmvrEncOpt)
    {
      bitsTemp    = dirBits + xDetermineBestMvp(cu, acMvTemp, bestMvpIdx, aamvpi);
      acMvPred[0] = aamvpi.mvCandLT[bestMvpIdx];
      acMvPred[1] = aamvpi.mvCandRT[bestMvpIdx];
      acMvPred[2] = aamvpi.mvCandLB[bestMvpIdx];
    }
    else
    {
      bitsTemp += xCalcAffineMVBits(cu, acMvTemp, acMvPred);
    }

    costTemp = (Distortion)(floor(fWeight * (double)costTemp) + (double)m_pcRdCost->getCost(bitsTemp));

    // store best cost and mv
    if (costTemp < uiCostBest)
    {
      uiCostBest = costTemp;
      uiBitsBest = bitsTemp;
      memcpy(acMv, acMvTemp, sizeof(Mv) * 3);
      mvpIdx = bestMvpIdx;
    }
    else if (m_encCfg->m_Affine > 1)
    {
      break;
    }
  }

  auto checkCPMVRdCost = [&](Mv ctrlPtMv[3])
  {
    xPredAffineBlk(COMP_Y, cu, refPic, ctrlPtMv, predBuf, false, cu.slice->clpRng(COMP_Y), eRefPicList);

    // get error
    Distortion costTemp =
      m_pcRdCost->getDistPart(predBuf.Y(), pBuf->Y(), cu.cs->sps->m_bitDepths[ChannelType::LUMA], COMP_Y, distFunc,
                              nullptr, cu.cs->sps->m_bitDepths[ChannelType::LUMA] + 4);
    // get cost with mv
    m_pcRdCost->setCostScale(0);
    uint32_t bitsTemp = ruiBits;
    bitsTemp += xCalcAffineMVBits(cu, ctrlPtMv, acMvPred);
    costTemp = (Distortion)(floor(fWeight * (double)costTemp) + (double)m_pcRdCost->getCost(bitsTemp));
    // store best cost and mv
    if (costTemp < uiCostBest)
    {
      uiCostBest = costTemp;
      uiBitsBest = bitsTemp;
      ::memcpy(acMv, ctrlPtMv, sizeof(Mv) * 3);
    }
  };

  if (uiCostBest <= AFFINE_ME_LIST_MVP_TH * m_hevcCost)
  {

    Mv mvPredTmp[3] = { acMvPred[0], acMvPred[1], acMvPred[2] };
    Mv mvME[3];
    ::memcpy(mvME, acMv, sizeof(Mv) * 3);
    Mv dMv = mvME[0] - mvPredTmp[0];

    for (int j = 0; j < mvNum; j++)
    {
      if ((!j && mvME[j] != mvPredTmp[j]) || (j && mvME[j] != (mvPredTmp[j] + dMv)))
      {
        ::memcpy(acMvTemp, mvME, sizeof(Mv) * 3);
        acMvTemp[j] = mvPredTmp[j];

        if (j)
        {
          acMvTemp[j] += dMv;
        }

        checkCPMVRdCost(acMvTemp);
      }
    }

    // keep the rotation/zoom;
    if (mvME[0] != mvPredTmp[0])
    {
      ::memcpy(acMvTemp, mvME, sizeof(Mv) * 3);
      for (int i = 1; i < mvNum; i++)
      {
        acMvTemp[i] -= dMv;
      }
      acMvTemp[0] = mvPredTmp[0];

      checkCPMVRdCost(acMvTemp);
    }

    // keep the translation;
    if (cu.affineType == AffineModel::_6_PARAMS && mvME[1] != (mvPredTmp[1] + dMv) && mvME[2] != (mvPredTmp[2] + dMv))
    {
      ::memcpy(acMvTemp, mvME, sizeof(Mv) * 3);

      acMvTemp[1] = mvPredTmp[1] + dMv;
      acMvTemp[2] = mvPredTmp[2] + dMv;

      checkCPMVRdCost(acMvTemp);
    }

    // 8 nearest neighbor search
    const Mv testPos[8] = { { -1, 0 }, { 0, -1 }, { 0, 1 }, { 1, 0 }, { -1, -1 }, { -1, 1 }, { 1, 1 }, { 1, -1 } };

    const int maxSearchRound = (cu.imv) ? 3 : ((m_encCfg->m_AffineAmvrEncOpt && m_encCfg->m_isLowDelay) ? 2 : 3);

    for (int rnd = 0; rnd < maxSearchRound; rnd++)
    {
      bool modelChange = false;
      // search the model parameters with finear granularity;
      for (int j = 0; j < mvNum; j++)
      {
        bool loopChange = false;
        for (int iter = 0; iter < 2; iter++)
        {
          if (iter == 1 && !loopChange)
          {
            break;
          }
          Mv centerMv[3];
          memcpy(centerMv, acMv, sizeof(Mv) * 3);
          memcpy(acMvTemp, acMv, sizeof(Mv) * 3);

          for (int i = ((iter == 0) ? 0 : 4); i < ((iter == 0) ? 4 : 8); i++)
          {
            Mv delta = testPos[i];
            delta.changeAffinePrecAmvr2Internal(cu.imv);

            acMvTemp[j] = centerMv[j];
            acMvTemp[j] += delta;
            clipMv(acMvTemp[j], cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);
            xPredAffineBlk(COMP_Y, cu, refPic, acMvTemp, predBuf, false, cu.slice->clpRng(COMP_Y), eRefPicList);

            Distortion costTemp =
              m_pcRdCost->getDistPart(predBuf.Y(), pBuf->Y(), cu.cs->sps->m_bitDepths[ChannelType::LUMA], COMP_Y,
                                      distFunc, nullptr, cu.cs->sps->m_bitDepths[ChannelType::LUMA] + 4);
            uint32_t bitsTemp = ruiBits;
            bitsTemp += xCalcAffineMVBits(cu, acMvTemp, acMvPred);
            costTemp = (Distortion)(floor(fWeight * (double)costTemp) + (double)m_pcRdCost->getCost(bitsTemp));

            if (costTemp < uiCostBest)
            {
              uiCostBest = costTemp;
              uiBitsBest = bitsTemp;
              ::memcpy(acMv, acMvTemp, sizeof(Mv) * 3);
              modelChange = true;
              loopChange  = true;
            }
          }
        }
      }

      if (!modelChange)
      {
        break;
      }
    }
  }
  acMvPred[0] = aamvpi.mvCandLT[mvpIdx];
  acMvPred[1] = aamvpi.mvCandRT[mvpIdx];
  acMvPred[2] = aamvpi.mvCandLB[mvpIdx];

  ruiBits = uiBitsBest;
  ruiCost = uiCostBest;
  DTRACE(g_trace_ctx, D_COMMON, " (%d) uiBitsBest=%d, uiCostBest=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON),
         uiBitsBest, uiCostBest);
}

void InterSearch::xEstimateAffineAMVP(CodingUnit &cu, AffineAMVPInfo &affineAMVPInfo, PelUnitBuf &origBuf,
                                      RefPicList eRefPicList, int refIdx, Mv acMvPred[3], Distortion *puiDistBiP)
{
  Mv         bestMvLT, bestMvRT, bestMvLB;
  int        iBestIdx   = 0;
  Distortion uiBestCost = std::numeric_limits<Distortion>::max();

  // Fill the MV Candidates
  PU::fillAffineMvpCand(cu, eRefPicList, refIdx, affineAMVPInfo);
  CHECK(affineAMVPInfo.numCand == 0, "Assertion failed.");

  PelUnitBuf predBuf = m_tmpStorageCtu.getBuf(UnitAreaRelative(cu, cu));

  // initialize Mvp index & Mvp
  iBestIdx = 0;
  for (int i = 0; i < affineAMVPInfo.numCand; i++)
  {
    Mv mv[3] = { affineAMVPInfo.mvCandLT[i], affineAMVPInfo.mvCandRT[i], affineAMVPInfo.mvCandLB[i] };

    Distortion uiTmpCost = xGetAffineTemplateCost(cu, origBuf, predBuf, mv, i, AMVP_MAX_NUM_CANDS, eRefPicList, refIdx);

    if (uiBestCost > uiTmpCost)
    {
      uiBestCost  = uiTmpCost;
      bestMvLT    = affineAMVPInfo.mvCandLT[i];
      bestMvRT    = affineAMVPInfo.mvCandRT[i];
      bestMvLB    = affineAMVPInfo.mvCandLB[i];
      iBestIdx    = i;
      *puiDistBiP = uiTmpCost;
    }
  }

  // Setting Best MVP
  acMvPred[0] = bestMvLT;
  acMvPred[1] = bestMvRT;
  acMvPred[2] = bestMvLB;

  cu.mvpIdx[eRefPicList] = iBestIdx;
  cu.mvpNum[eRefPicList] = affineAMVPInfo.numCand;

  DTRACE(g_trace_ctx, D_COMMON, "#estAffi=%d \n", affineAMVPInfo.numCand);
}

void InterSearch::xCopyAffineAMVPInfo(AffineAMVPInfo &src, AffineAMVPInfo &dst)
{
  dst.numCand = src.numCand;
  DTRACE(g_trace_ctx, D_COMMON, " (%d) #copyAffi=%d \n", DTRACE_GET_COUNTER(g_trace_ctx, D_COMMON), src.numCand);
  ::memcpy(dst.mvCandLT, src.mvCandLT, sizeof(Mv) * src.numCand);
  ::memcpy(dst.mvCandRT, src.mvCandRT, sizeof(Mv) * src.numCand);
  ::memcpy(dst.mvCandLB, src.mvCandLB, sizeof(Mv) * src.numCand);
}

/**
 * \brief Generate half-sample interpolated block
 *
 * \param pattern Reference picture ROI
 * \param biPred    Flag indicating whether block is for biprediction
 */
void InterSearch::xExtDIFUpSamplingH(CPelBuf *pattern, bool useAltHpelIf)
{
  PROFILER_SCOPE(TP_ENABLE_INTER_FRAC_PEL_SEARCH_STAGES, g_timeProfiler, P_INTER_MVD_SEARCH_HPEL);
  const ClpRng &clpRng    = m_lumaClpRng;
  int           width     = pattern->width;
  int           height    = pattern->height;
  ptrdiff_t     srcStride = pattern->stride;

  ptrdiff_t  intStride = width + 1;
  ptrdiff_t  dstStride = width + 1;
  Pel       *intPtr;
  Pel       *dstPtr;
  int        filterSize     = NTAPS_LUMA;
  int        halfFilterSize = (filterSize >> 1);
  const Pel *srcPtr         = pattern->buf - halfFilterSize * srcStride - 1;

  const auto filterIdx = useAltHpelIf ? InterpolationFilter::Filter::HALFPEL_ALT : InterpolationFilter::Filter::DEFAULT;

  m_if->filterHor(COMP_Y, srcPtr, srcStride, m_filteredBlockTmp[0][0], intStride, width, height + filterSize,
                  0 << MV_FRACTIONAL_BITS_DIFF, false, clpRng, filterIdx);
  m_if->filterHor(COMP_Y, srcPtr + width, srcStride, m_filteredBlockTmp[0][0] + width, intStride, 1,
                  height + filterSize, 0 << MV_FRACTIONAL_BITS_DIFF, false, clpRng, filterIdx);
  if (!m_skipFracME)
  {
    m_if->filterHor(COMP_Y, srcPtr, srcStride, m_filteredBlockTmp[2][0], intStride, width, height + filterSize,
                    2 << MV_FRACTIONAL_BITS_DIFF, false, clpRng, filterIdx);
    m_if->filterHor(COMP_Y, srcPtr + width, srcStride, m_filteredBlockTmp[2][0] + width, intStride, 1,
                    height + filterSize, 2 << MV_FRACTIONAL_BITS_DIFF, false, clpRng, filterIdx);
  }

  intPtr = m_filteredBlockTmp[0][0] + halfFilterSize * intStride + 1;
  dstPtr = m_filteredBlock[0][0][0];
  m_if->filterVer(COMP_Y, intPtr, intStride, dstPtr, dstStride, width + 0, height + 0, 0 << MV_FRACTIONAL_BITS_DIFF,
                  false, true, clpRng, filterIdx);
  if (m_skipFracME)
  {
    return;
  }

  intPtr = m_filteredBlockTmp[0][0] + (halfFilterSize - 1) * intStride + 1;
  dstPtr = m_filteredBlock[2][0][0];
  m_if->filterVer(COMP_Y, intPtr, intStride, dstPtr, dstStride, width + 0, height + 1, 2 << MV_FRACTIONAL_BITS_DIFF,
                  false, true, clpRng, filterIdx);

  intPtr = m_filteredBlockTmp[2][0] + halfFilterSize * intStride;
  dstPtr = m_filteredBlock[0][2][0];
  m_if->filterVer(COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height + 0, 0 << MV_FRACTIONAL_BITS_DIFF, false,
                  true, clpRng, filterIdx);
  m_if->filterVer(COMP_Y, intPtr + width, intStride, dstPtr + width, dstStride, 1, height + 0,
                  0 << MV_FRACTIONAL_BITS_DIFF, false, true, clpRng, filterIdx);

  intPtr = m_filteredBlockTmp[2][0] + (halfFilterSize - 1) * intStride;
  dstPtr = m_filteredBlock[2][2][0];
  m_if->filterVer(COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height + 1, 2 << MV_FRACTIONAL_BITS_DIFF, false,
                  true, clpRng, filterIdx);
  m_if->filterVer(COMP_Y, intPtr + width, intStride, dstPtr + width, dstStride, 1, height + 1,
                  2 << MV_FRACTIONAL_BITS_DIFF, false, true, clpRng, filterIdx);
}

/**
 * \brief Generate quarter-sample interpolated blocks
 *
 * \param pattern    Reference picture ROI
 * \param halfPelRef Half-pel mv
 * \param biPred     Flag indicating whether block is for biprediction
 */
void InterSearch::xExtDIFUpSamplingQ(CPelBuf *pattern, Mv halfPelRef)
{
  PROFILER_SCOPE(TP_ENABLE_INTER_FRAC_PEL_SEARCH_STAGES, g_timeProfiler, P_INTER_MVD_SEARCH_QPEL);
  const ClpRng &clpRng    = m_lumaClpRng;
  int           width     = pattern->width;
  int           height    = pattern->height;
  ptrdiff_t     srcStride = pattern->stride;

  Pel const *srcPtr;
  ptrdiff_t  intStride = width + 1;
  ptrdiff_t  dstStride = width + 1;
  Pel       *intPtr;
  Pel       *dstPtr;
  int        filterSize = NTAPS_LUMA;

  int halfFilterSize = (filterSize >> 1);

  int extHeight = (halfPelRef.getVer() == 0) ? height + filterSize : height + filterSize - 1;

  // Horizontal filter 1/4
  srcPtr = pattern->buf - halfFilterSize * srcStride - 1;
  intPtr = m_filteredBlockTmp[1][0];
  if (halfPelRef.getVer() > 0)
  {
    srcPtr += srcStride;
  }
  if (halfPelRef.getHor() >= 0)
  {
    srcPtr += 1;
  }
  m_if->filterHor(COMP_Y, srcPtr, srcStride, intPtr, intStride, width, extHeight, 1 << MV_FRACTIONAL_BITS_DIFF, false,
                  clpRng, InterpolationFilter::Filter::DEFAULT);

  // Horizontal filter 3/4
  srcPtr = pattern->buf - halfFilterSize * srcStride - 1;
  intPtr = m_filteredBlockTmp[3][0];
  if (halfPelRef.getVer() > 0)
  {
    srcPtr += srcStride;
  }
  if (halfPelRef.getHor() > 0)
  {
    srcPtr += 1;
  }
  m_if->filterHor(COMP_Y, srcPtr, srcStride, intPtr, intStride, width, extHeight, 3 << MV_FRACTIONAL_BITS_DIFF, false,
                  clpRng, InterpolationFilter::Filter::DEFAULT);

  // Generate @ 1,1
  intPtr = m_filteredBlockTmp[1][0] + (halfFilterSize - 1) * intStride;
  dstPtr = m_filteredBlock[1][1][0];
  if (halfPelRef.getVer() == 0)
  {
    intPtr += intStride;
  }
  m_if->filterVer(COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1 << MV_FRACTIONAL_BITS_DIFF, false,
                  true, clpRng, InterpolationFilter::Filter::DEFAULT);

  // Generate @ 3,1
  intPtr = m_filteredBlockTmp[1][0] + (halfFilterSize - 1) * intStride;
  dstPtr = m_filteredBlock[3][1][0];
  m_if->filterVer(COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3 << MV_FRACTIONAL_BITS_DIFF, false,
                  true, clpRng, InterpolationFilter::Filter::DEFAULT);

  if (halfPelRef.getVer() != 0)
  {
    // Generate @ 2,1
    intPtr = m_filteredBlockTmp[1][0] + (halfFilterSize - 1) * intStride;
    dstPtr = m_filteredBlock[2][1][0];
    if (halfPelRef.getVer() == 0)
    {
      intPtr += intStride;
    }
    m_if->filterVer(COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 2 << MV_FRACTIONAL_BITS_DIFF, false,
                    true, clpRng, InterpolationFilter::Filter::DEFAULT);

    // Generate @ 2,3
    intPtr = m_filteredBlockTmp[3][0] + (halfFilterSize - 1) * intStride;
    dstPtr = m_filteredBlock[2][3][0];
    if (halfPelRef.getVer() == 0)
    {
      intPtr += intStride;
    }
    m_if->filterVer(COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 2 << MV_FRACTIONAL_BITS_DIFF, false,
                    true, clpRng, InterpolationFilter::Filter::DEFAULT);
  }
  else
  {
    // Generate @ 0,1
    intPtr = m_filteredBlockTmp[1][0] + halfFilterSize * intStride;
    dstPtr = m_filteredBlock[0][1][0];
    m_if->filterVer(COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 0 << MV_FRACTIONAL_BITS_DIFF, false,
                    true, clpRng, InterpolationFilter::Filter::DEFAULT);

    // Generate @ 0,3
    intPtr = m_filteredBlockTmp[3][0] + halfFilterSize * intStride;
    dstPtr = m_filteredBlock[0][3][0];
    m_if->filterVer(COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 0 << MV_FRACTIONAL_BITS_DIFF, false,
                    true, clpRng, InterpolationFilter::Filter::DEFAULT);
  }

  if (halfPelRef.getHor() != 0)
  {
    // Generate @ 1,2
    intPtr = m_filteredBlockTmp[2][0] + (halfFilterSize - 1) * intStride;
    dstPtr = m_filteredBlock[1][2][0];
    if (halfPelRef.getHor() > 0)
    {
      intPtr += 1;
    }
    if (halfPelRef.getVer() >= 0)
    {
      intPtr += intStride;
    }
    m_if->filterVer(COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1 << MV_FRACTIONAL_BITS_DIFF, false,
                    true, clpRng, InterpolationFilter::Filter::DEFAULT);

    // Generate @ 3,2
    intPtr = m_filteredBlockTmp[2][0] + (halfFilterSize - 1) * intStride;
    dstPtr = m_filteredBlock[3][2][0];
    if (halfPelRef.getHor() > 0)
    {
      intPtr += 1;
    }
    if (halfPelRef.getVer() > 0)
    {
      intPtr += intStride;
    }
    m_if->filterVer(COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3 << MV_FRACTIONAL_BITS_DIFF, false,
                    true, clpRng, InterpolationFilter::Filter::DEFAULT);
  }
  else
  {
    // Generate @ 1,0
    intPtr = m_filteredBlockTmp[0][0] + (halfFilterSize - 1) * intStride + 1;
    dstPtr = m_filteredBlock[1][0][0];
    if (halfPelRef.getVer() >= 0)
    {
      intPtr += intStride;
    }
    m_if->filterVer(COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1 << MV_FRACTIONAL_BITS_DIFF, false,
                    true, clpRng, InterpolationFilter::Filter::DEFAULT);

    // Generate @ 3,0
    intPtr = m_filteredBlockTmp[0][0] + (halfFilterSize - 1) * intStride + 1;
    dstPtr = m_filteredBlock[3][0][0];
    if (halfPelRef.getVer() > 0)
    {
      intPtr += intStride;
    }
    m_if->filterVer(COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3 << MV_FRACTIONAL_BITS_DIFF, false,
                    true, clpRng, InterpolationFilter::Filter::DEFAULT);
  }

  // Generate @ 1,3
  intPtr = m_filteredBlockTmp[3][0] + (halfFilterSize - 1) * intStride;
  dstPtr = m_filteredBlock[1][3][0];
  if (halfPelRef.getVer() == 0)
  {
    intPtr += intStride;
  }
  m_if->filterVer(COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 1 << MV_FRACTIONAL_BITS_DIFF, false,
                  true, clpRng, InterpolationFilter::Filter::DEFAULT);

  // Generate @ 3,3
  intPtr = m_filteredBlockTmp[3][0] + (halfFilterSize - 1) * intStride;
  dstPtr = m_filteredBlock[3][3][0];
  m_if->filterVer(COMP_Y, intPtr, intStride, dstPtr, dstStride, width, height, 3 << MV_FRACTIONAL_BITS_DIFF, false,
                  true, clpRng, InterpolationFilter::Filter::DEFAULT);
}

//! set wp tables
void InterSearch::xSetWpScalingDistParam(int refIdx, RefPicList eRefPicListCur, Slice *pcSlice)
{
  if (refIdx < 0)
  {
    m_cDistParam.applyWeight = false;
    return;
  }

  WPScalingParam *wp0, *wp1;

  m_cDistParam.applyWeight = (pcSlice->m_eSliceType == P_SLICE && pcSlice->m_testWeightPred) ||
    (pcSlice->m_eSliceType == B_SLICE && pcSlice->m_testWeightBiPred);

  if (!m_cDistParam.applyWeight)
  {
    return;
  }

  int refIdx0 = (eRefPicListCur == RPL0) ? refIdx : (-1);
  int refIdx1 = (eRefPicListCur == RPL1) ? refIdx : (-1);

  getWpScaling(pcSlice, refIdx0, refIdx1, wp0, wp1);

  if (refIdx0 < 0)
  {
    wp0 = nullptr;
  }
  if (refIdx1 < 0)
  {
    wp1 = nullptr;
  }

  m_cDistParam.wpCur = nullptr;

  if (eRefPicListCur == RPL0)
  {
    m_cDistParam.wpCur = wp0;
  }
  else
  {
    m_cDistParam.wpCur = wp1;
  }
}

void InterSearch::xEncodeInterResidualQT(CodingStructure &cs, Partitioner &partitioner, const CompID &compID)
{
  const UnitArea      &currArea = partitioner.currArea();
  const TransformUnit &currTU =
    *cs.getTU(isLuma(partitioner.chType) ? currArea.lumaPos() : currArea.chromaPos(), partitioner.chType);
  const CodingUnit &cu        = *currTU.cu;
  const unsigned    currDepth = partitioner.currTrDepth;

  const bool bSubdiv = currDepth != currTU.depth;

  if (compID == MAX_NUM_TBLOCKS)   // we are not processing a channel, instead we always recurse and code the CBFs
  {
    if (partitioner.canSplit(TU_MAX_TR_SPLIT, cs))
    {
      CHECK(!bSubdiv, "Not performing the implicit TU split");
    }
    else if (cu.sbtInfo && partitioner.canSplit(PartSplit(cu.getSbtTuSplit()), cs))
    {
      CHECK(!bSubdiv, "Not performing the implicit TU split - sbt");
    }
    else
    {
      CHECK(bSubdiv, "transformsplit not supported");
    }

    CHECK(CU::isIntra(cu), "Inter search provided with intra CU");

    if (isChromaEnabled(cu.chromaFormat) && (!CS::isDualITree(cs) || isChroma(partitioner.chType)))
    {
      {
        const bool chromaCbf = TU::getCbfAtDepth(currTU, COMP_Cb, currDepth);
        if (!(cu.sbtInfo && (currDepth == 0 || (currDepth == 1 && currTU.noResidual))))
        {
          m_CABACEstimator->cbf_comp(chromaCbf, currArea.blocks[COMP_Cb], currDepth, false, BdpcmMode::NONE);
        }
      }
      {
        const bool chromaCbf = TU::getCbfAtDepth(currTU, COMP_Cr, currDepth);
        if (!(cu.sbtInfo && (currDepth == 0 || (currDepth == 1 && currTU.noResidual))))
        {
          m_CABACEstimator->cbf_comp(chromaCbf, currArea.blocks[COMP_Cr], currDepth,
                                     TU::getCbfAtDepth(currTU, COMP_Cb, currDepth), BdpcmMode::NONE);
        }
      }
    }

    if (!bSubdiv && !(cu.sbtInfo && currTU.noResidual) && !isChroma(partitioner.chType))
    {
      m_CABACEstimator->cbf_comp(TU::getCbfAtDepth(currTU, COMP_Y, currDepth), currArea.Y(), currDepth, false,
                                 BdpcmMode::NONE);
    }
  }

  if (!bSubdiv)
  {
    if (compID != MAX_NUM_TBLOCKS)   // we have already coded the CBFs, so now we code coefficients
    {
      if (currArea.blocks[compID].valid())
      {
        if (compID == COMP_Cr)
        {
          const int cbfMask =
            (TU::getCbf(currTU, COMP_Cb) ? CBF_MASK_CB : 0) + (TU::getCbf(currTU, COMP_Cr) ? CBF_MASK_CR : 0);
          m_CABACEstimator->joint_cb_cr(currTU, cbfMask);
        }
        if (TU::getCbf(currTU, compID))
        {
          m_CABACEstimator->residual_coding_last(currTU, compID);
          m_CABACEstimator->residual_coding_coef(currTU, compID);
          m_CABACEstimator->residual_coding_sign(currTU, compID);
        }
      }
    }
  }
  else
  {
    if (compID == MAX_NUM_TBLOCKS || TU::getCbfAtDepth(currTU, compID, currDepth))
    {
      if (partitioner.canSplit(TU_MAX_TR_SPLIT, cs))
      {
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
        xEncodeInterResidualQT(cs, partitioner, compID);
      } while (partitioner.nextPart(cs));

      partitioner.exitCurrSplit();
    }
  }
}

void InterSearch::calcMinDistSbt(CodingStructure &cs, const CodingUnit &cu, const uint8_t sbtAllowed)
{
  if (!sbtAllowed)
  {
    m_estMinDistSbt[NUMBER_SBT_MODE] = 0;
    for (int comp = 0; comp < getNumberValidTBlocks(*cs.pcv); comp++)
    {
      const CompID compID = CompID(comp);
      CPelBuf      pred   = cs.getPredBuf(compID);
      CPelBuf      org    = cs.getOrgBuf(compID);
      m_estMinDistSbt[NUMBER_SBT_MODE] +=
        m_pcRdCost->getDistPart(org, pred, cs.sps->m_bitDepths[toChannelType(compID)], compID, DFunc::SSE);
    }
    return;
  }

  // SBT fast algorithm 2.1 : estimate a minimum RD cost of a SBT mode based on the luma distortion of uncoded part and
  // coded part (assuming distorted can be reduced to 1/16);
  //                          if this cost is larger than the best cost, no need to try a specific SBT mode
  int                                  cuWidth  = cu.lwidth();
  int                                  cuHeight = cu.lheight();
  const int                            numPartX = (cuWidth >= 8 ? 4 : 1);
  const int                            numPartY = (cuHeight >= 8 ? 4 : 1);
  std::vector<std::vector<Distortion>> dist(numPartY, std::vector<Distortion>(numPartX, 0));

  for (uint32_t c = 0; c < getNumberValidTBlocks(*cs.pcv); c++)
  {
    const CompID     compID     = CompID(c);
    const CompArea  &compArea   = cu.blocks[compID];
    const CPelBuf    orgPel     = cs.getOrgBuf(compArea);
    const CPelBuf    predPel    = cs.getPredBuf(compArea);
    int              lengthX    = compArea.width / numPartX;
    int              lengthY    = compArea.height / numPartY;
    ptrdiff_t        strideOrg  = orgPel.stride;
    ptrdiff_t        stridePred = predPel.stride;
    uint32_t         shift = DISTORTION_PRECISION_ADJUSTMENT((*cs.sps.m_bitDepths[toChannelType(compID)] - 8) << 1);
    Intermediate_Int temp;

    // calc distY of 16 sub parts
    for (int j = 0; j < numPartY; j++)
    {
      for (int i = 0; i < numPartX; i++)
      {
        int        posX    = i * lengthX;
        int        posY    = j * lengthY;
        const Pel *ptrOrg  = orgPel.bufAt(posX, posY);
        const Pel *ptrPred = predPel.bufAt(posX, posY);
        Distortion sum     = 0;
        for (int n = 0; n < lengthY; n++)
        {
          for (int m = 0; m < lengthX; m++)
          {
            temp = ptrOrg[m] - ptrPred[m];
            sum += Distortion((temp * temp) >> shift);
          }
          ptrOrg += strideOrg;
          ptrPred += stridePred;
        }
        if (isChroma(compID))
        {
          sum = (Distortion)(sum * m_pcRdCost->getChromaWeight());
        }
        dist[j][i] += sum;
      }
    }
  }

  // SSE of a CU
  m_estMinDistSbt[NUMBER_SBT_MODE] = 0;
  for (int j = 0; j < numPartY; j++)
  {
    for (int i = 0; i < numPartX; i++)
    {
      m_estMinDistSbt[NUMBER_SBT_MODE] += dist[j][i];
    }
  }
  // init per-mode dist
  for (int i = SBT_VER_H0; i < NUMBER_SBT_MODE; i++)
  {
    m_estMinDistSbt[i] = std::numeric_limits<uint64_t>::max();
  }

  // SBT fast algorithm 1: not try SBT if the residual is too small to compensate bits for encoding residual info
  uint64_t minNonZeroResiFracBits = 12 << SCALE_BITS;
  if (m_pcRdCost->calcRdCost(0, m_estMinDistSbt[NUMBER_SBT_MODE]) < m_pcRdCost->calcRdCost(minNonZeroResiFracBits, 0))
  {
    m_skipSbtAll = true;
    return;
  }

  // derive estimated minDist of SBT = zero-residual part distortion + non-zero residual part distortion / 16
  int        shift        = 5;
  Distortion distResiPart = 0, distNoResiPart = 0;

  if (CU::targetSbtAllowed(SBT_VER_HALF, sbtAllowed))
  {
    int offsetResiPart   = 0;
    int offsetNoResiPart = numPartX / 2;
    distResiPart = distNoResiPart = 0;
    assert(numPartX >= 2);
    for (int j = 0; j < numPartY; j++)
    {
      for (int i = 0; i < numPartX / 2; i++)
      {
        distResiPart += dist[j][i + offsetResiPart];
        distNoResiPart += dist[j][i + offsetNoResiPart];
      }
    }
    m_estMinDistSbt[SBT_VER_H0] = (distResiPart >> shift) + distNoResiPart;
    m_estMinDistSbt[SBT_VER_H1] = (distNoResiPart >> shift) + distResiPart;
  }

  if (CU::targetSbtAllowed(SBT_HOR_HALF, sbtAllowed))
  {
    int offsetResiPart   = 0;
    int offsetNoResiPart = numPartY / 2;
    assert(numPartY >= 2);
    distResiPart = distNoResiPart = 0;
    for (int j = 0; j < numPartY / 2; j++)
    {
      for (int i = 0; i < numPartX; i++)
      {
        distResiPart += dist[j + offsetResiPart][i];
        distNoResiPart += dist[j + offsetNoResiPart][i];
      }
    }
    m_estMinDistSbt[SBT_HOR_H0] = (distResiPart >> shift) + distNoResiPart;
    m_estMinDistSbt[SBT_HOR_H1] = (distNoResiPart >> shift) + distResiPart;
  }

  if (CU::targetSbtAllowed(SBT_VER_QUAD, sbtAllowed))
  {
    assert(numPartX == 4);
    m_estMinDistSbt[SBT_VER_Q0] = m_estMinDistSbt[SBT_VER_Q1] = 0;
    for (int j = 0; j < numPartY; j++)
    {
      m_estMinDistSbt[SBT_VER_Q0] += dist[j][0] + ((dist[j][1] + dist[j][2] + dist[j][3]) << shift);
      m_estMinDistSbt[SBT_VER_Q1] += dist[j][3] + ((dist[j][0] + dist[j][1] + dist[j][2]) << shift);
    }
    m_estMinDistSbt[SBT_VER_Q0] = m_estMinDistSbt[SBT_VER_Q0] >> shift;
    m_estMinDistSbt[SBT_VER_Q1] = m_estMinDistSbt[SBT_VER_Q1] >> shift;
  }

  if (CU::targetSbtAllowed(SBT_HOR_QUAD, sbtAllowed))
  {
    assert(numPartY == 4);
    m_estMinDistSbt[SBT_HOR_Q0] = m_estMinDistSbt[SBT_HOR_Q1] = 0;
    for (int i = 0; i < numPartX; i++)
    {
      m_estMinDistSbt[SBT_HOR_Q0] += dist[0][i] + ((dist[1][i] + dist[2][i] + dist[3][i]) << shift);
      m_estMinDistSbt[SBT_HOR_Q1] += dist[3][i] + ((dist[0][i] + dist[1][i] + dist[2][i]) << shift);
    }
    m_estMinDistSbt[SBT_HOR_Q0] = m_estMinDistSbt[SBT_HOR_Q0] >> shift;
    m_estMinDistSbt[SBT_HOR_Q1] = m_estMinDistSbt[SBT_HOR_Q1] >> shift;
  }

  if (CU::targetSbtAllowed(SBT_QUAD, sbtAllowed))
  {
    m_estMinDistSbt[SBT_Q0] = m_estMinDistSbt[SBT_Q1] = m_estMinDistSbt[SBT_Q2] = m_estMinDistSbt[SBT_Q3] = 0;

    for (int j = 0; j < numPartY / 2; j++)
    {
      for (int i = 0; i < numPartX / 2; i++)
      {
        m_estMinDistSbt[SBT_Q0] += dist[j][i];
        m_estMinDistSbt[SBT_Q1] += dist[j][i + (numPartX >> 1)];
        m_estMinDistSbt[SBT_Q2] += dist[j + (numPartY >> 1)][i];
        m_estMinDistSbt[SBT_Q3] += dist[j + (numPartY >> 1)][i + (numPartX >> 1)];
      }
    }
    m_estMinDistSbt[SBT_Q0] =
      (m_estMinDistSbt[NUMBER_SBT_MODE] - m_estMinDistSbt[SBT_Q0]) + (m_estMinDistSbt[SBT_Q0] >> shift);
    m_estMinDistSbt[SBT_Q1] =
      (m_estMinDistSbt[NUMBER_SBT_MODE] - m_estMinDistSbt[SBT_Q1]) + (m_estMinDistSbt[SBT_Q1] >> shift);
    m_estMinDistSbt[SBT_Q2] =
      (m_estMinDistSbt[NUMBER_SBT_MODE] - m_estMinDistSbt[SBT_Q2]) + (m_estMinDistSbt[SBT_Q2] >> shift);
    m_estMinDistSbt[SBT_Q3] =
      (m_estMinDistSbt[NUMBER_SBT_MODE] - m_estMinDistSbt[SBT_Q3]) + (m_estMinDistSbt[SBT_Q3] >> shift);
  }

  if (CU::targetSbtAllowed(SBT_QUARTER, sbtAllowed))
  {
    m_estMinDistSbt[SBT_QT0] = m_estMinDistSbt[SBT_QT1] = m_estMinDistSbt[SBT_QT2] = m_estMinDistSbt[SBT_QT3] = 0;

    for (int j = 0; j < numPartY / 4; j++)
    {
      for (int i = 0; i < numPartX / 4; i++)
      {
        m_estMinDistSbt[SBT_QT0] += dist[j][i];
        m_estMinDistSbt[SBT_QT1] += dist[j][i + (3 * numPartX >> 2)];
        m_estMinDistSbt[SBT_QT2] += dist[j + (3 * numPartY >> 2)][i];
        m_estMinDistSbt[SBT_QT3] += dist[j + (3 * numPartY >> 2)][i + (3 * numPartX >> 2)];
      }
    }
    m_estMinDistSbt[SBT_QT0] =
      (m_estMinDistSbt[NUMBER_SBT_MODE] - m_estMinDistSbt[SBT_QT0]) + (m_estMinDistSbt[SBT_QT0] >> shift);
    m_estMinDistSbt[SBT_QT1] =
      (m_estMinDistSbt[NUMBER_SBT_MODE] - m_estMinDistSbt[SBT_QT1]) + (m_estMinDistSbt[SBT_QT1] >> shift);
    m_estMinDistSbt[SBT_QT2] =
      (m_estMinDistSbt[NUMBER_SBT_MODE] - m_estMinDistSbt[SBT_QT2]) + (m_estMinDistSbt[SBT_QT2] >> shift);
    m_estMinDistSbt[SBT_QT3] =
      (m_estMinDistSbt[NUMBER_SBT_MODE] - m_estMinDistSbt[SBT_QT3]) + (m_estMinDistSbt[SBT_QT3] >> shift);
  }

  // SBT fast algorithm 5: try N SBT modes with the lowest distortion
  Distortion temp[NUMBER_SBT_MODE];
  memcpy(temp, m_estMinDistSbt, sizeof(Distortion) * NUMBER_SBT_MODE);
  memset(m_sbtRdoOrder, 255, NUMBER_SBT_MODE);
  int startIdx = 0, numRDO;
  numRDO       = CU::targetSbtAllowed(SBT_VER_HALF, sbtAllowed) + CU::targetSbtAllowed(SBT_HOR_HALF, sbtAllowed);
  numRDO       = std::min((numRDO << 1), SBT_NUM_RDO);
  for (int i = startIdx; i < startIdx + numRDO; i++)
  {
    Distortion minDist = std::numeric_limits<uint64_t>::max();
    for (int n = SBT_VER_H0; n <= SBT_HOR_H1; n++)
    {
      if (temp[n] < minDist)
      {
        minDist          = temp[n];
        m_sbtRdoOrder[i] = n;
      }
    }
    temp[m_sbtRdoOrder[i]] = std::numeric_limits<uint64_t>::max();
  }

  startIdx += numRDO;
  numRDO = ((CU::targetSbtAllowed(SBT_VER_QUAD, sbtAllowed) + CU::targetSbtAllowed(SBT_HOR_QUAD, sbtAllowed)) << 1) +
    (CU::targetSbtAllowed(SBT_QUAD, sbtAllowed) << 2);
  numRDO = std::min(numRDO, SBT_NUM_RDO);

  for (int i = startIdx; i < startIdx + numRDO; i++)
  {
    Distortion minDist = std::numeric_limits<uint64_t>::max();
    for (int n = SBT_VER_Q0; n < NUMBER_SBT_MODE; n++)
    {
      if (n >= SBT_QT0 && n <= SBT_QT3)
      {
        continue;
      }
      if (temp[n] < minDist)
      {
        minDist          = temp[n];
        m_sbtRdoOrder[i] = n;
      }
    }
    temp[m_sbtRdoOrder[i]] = std::numeric_limits<uint64_t>::max();
  }

  startIdx += numRDO;
  numRDO = CU::targetSbtAllowed(SBT_QUARTER, sbtAllowed);
  numRDO = std::min((numRDO << 2), SBT_NUM_RDO);

  for (int i = startIdx; i < startIdx + numRDO; i++)
  {
    Distortion minDist = std::numeric_limits<uint64_t>::max();
    for (int n = SBT_QT0; n <= SBT_QT3; n++)
    {
      if (temp[n] < minDist)
      {
        minDist          = temp[n];
        m_sbtRdoOrder[i] = n;
      }
    }
    temp[m_sbtRdoOrder[i]] = std::numeric_limits<uint64_t>::max();
  }
}

uint8_t InterSearch::skipSbtByRDCost(int width, int height, int mtDepth, uint8_t sbtIdx, uint8_t sbtPos,
                                     double bestCost, Distortion distSbtOff, double costSbtOff, bool rootCbfSbtOff)
{
  int sbtMode = CU::getSbtMode(sbtIdx, sbtPos);

  // SBT fast algorithm 2.2 : estimate a minimum RD cost of a SBT mode based on the luma distortion of uncoded part and
  // coded part (assuming distorted can be reduced to 1/16);
  //                          if this cost is larger than the best cost, no need to try a specific SBT mode
  if (m_pcRdCost->calcRdCost(11 << SCALE_BITS, m_estMinDistSbt[sbtMode]) > bestCost)
  {
    return 0;   // early skip type 0
  }

  if (costSbtOff != MAX_DOUBLE)
  {
    if (!rootCbfSbtOff)
    {
      // SBT fast algorithm 3: skip SBT when the residual is too small (estCost is more accurate than fast algorithm 1,
      // counting PU mode bits)
      uint64_t   minNonZeroResiFracBits = 10 << SCALE_BITS;
      Distortion distResiPart;
      if (sbtIdx == SBT_VER_HALF || sbtIdx == SBT_HOR_HALF)
      {
        distResiPart = (Distortion)(((m_estMinDistSbt[NUMBER_SBT_MODE] - m_estMinDistSbt[sbtMode]) * 9) >> 4);
      }
      else
      {
        distResiPart = (Distortion)(((m_estMinDistSbt[NUMBER_SBT_MODE] - m_estMinDistSbt[sbtMode]) * 3) >> 3);
      }

      double estCost = (costSbtOff - m_pcRdCost->calcRdCost(0 << SCALE_BITS, distSbtOff)) +
        m_pcRdCost->calcRdCost(minNonZeroResiFracBits, m_estMinDistSbt[sbtMode] + distResiPart);
      if (estCost > costSbtOff)
      {
        return 1;
      }
      if (estCost > bestCost)
      {
        return 2;
      }
    }
    else
    {
      // SBT fast algorithm 4: skip SBT when an estimated RD cost is larger than the bestCost
      double weight  = sbtMode > SBT_HOR_H1 ? 0.4 : 0.6;
      double estCost = ((costSbtOff - m_pcRdCost->calcRdCost(0 << SCALE_BITS, distSbtOff)) * weight) +
        m_pcRdCost->calcRdCost(0 << SCALE_BITS, m_estMinDistSbt[sbtMode]);
      if (estCost > bestCost)
      {
        return 3;
      }
    }
  }
  return MAX_UCHAR;
}

TrEst::Cost InterSearch::xPreCalcTrans(TransformUnit &tu, const CompID compID, PreTrList &tl)
{
  const CompArea &area          = tu.blocks[compID];
  const bool      lossless      = tu.cs->slice->m_isLossless && m_encCfg->m_costMode == COST_LOSSLESS_CODING;
  const bool      reshapeChroma = (isChroma(compID) && tu.cs->slice->m_lmcsEnabledFlag &&
                              tu.cs->picHeader->m_lmcsChromaResidualScaleFlag && area.area() > 4);

  //----- calculate residual -----
  PelBuf res(tl.res[compID], area);
  res.copyFrom(tu.cs->getOrgResiBuf(area));
  if (reshapeChroma)
  {
    res.scaleSignal(tu.getChromaAdj(), 1, tu.cu->cs->slice->clpRng(compID));
  }

  //----- create transform list -----
  const auto trTypes =
    TU::getTransCandInter(tu, compID, lossless, m_encCfg->m_useChromaTS, m_histBestMtsIdx, m_histBestSbt);
  tl.trTypes.clear();
  tl.trTypesAdd.clear();
  for (const auto &tr: trTypes)
  {
    (tr == MtsType::DCT2_DCT2 || tr == MtsType::SKIP ? tl.trTypes : tl.trTypesAdd).push_back(tr);
  }

  //----- do all transforms -----
  m_pcTrQuant->preCalcTrans(tu, compID, res, tl.trTypes, tl.trCoeffs);

  //----- select transforms -----
  return xSelectTrCand(tu, compID, tl);
}

void InterSearch::xPreCalcTransUpd(TransformUnit &tu, const CompID compID, PreTrList &tl, TrEst::Cost &costDCT)
{
  //----- modify transform list -----
  std::swap(tl.trTypes, tl.trTypesAdd);
  const bool dctAvailable = tl.trTypesAdd[0] == MtsType::DCT2_DCT2;

  //----- for NSPT/LFNST: derive DIMD intra mode -----
  if (includesNST(tl.trTypes))
  {
    tu.derivedIntraDirsLuma = IntraPrediction::deriveIpmForTransform(tu.cs->getPredBuf(tu).Y(), *tu.cu);
  }

  //----- do all transforms -----
  m_pcTrQuant->preCalcTrans(tu, compID, PelBuf(tl.res[compID], tu.blocks[compID]), tl.trTypes, tl.trCoeffs,
                            dctAvailable);

  //----- select transforms -----
  xSelectTrCandUpd(tu, compID, tl, costDCT);
}

CbfMaskList InterSearch::xPreCalcTransJCCR(TransformUnit &tu, PreTrList &tl)
{
  CbfMaskList cbfMasks;
  const bool  cbfCb = TU::getCbf(tu, COMP_Cb);
  const bool  cbfCr = TU::getCbf(tu, COMP_Cr);
  if (!cbfCb && !cbfCr)
  {
    return cbfMasks;
  }

  //--- calculate residuals and determine JCCR types to be tested ---
  const bool      lossless   = tu.cs->slice->m_isLossless && m_encCfg->m_costMode == COST_LOSSLESS_CODING;
  const CompArea &area       = tu.blocks[COMP_Cb];
  const PelBuf    resCb      = PelBuf(tl.res[COMP_Cb], area);
  const PelBuf    resCr      = PelBuf(tl.res[COMP_Cr], area);
  PelBuf          resJCCR    = PelBuf(tl.res[JOINT_CbCr], area);
  const PelBuf   *resCbCr[2] = { &resCb, &resCr };
  PelBuf         *resICT[3]  = { nullptr, nullptr, &resJCCR };
  cbfMasks                   = m_pcTrQuant->selectICTCandidates(tu, resCbCr, resICT);
  CHECK(cbfMasks.size() > 1, "Something wrong: multiple JCCR modes in Inter");
  if (cbfMasks.empty())
  {
    return cbfMasks;
  }

  //--- determine transforms to be tested (based on non-JCCR data) ---
  const bool      trCb   = cbfCb && tu.mtsIdx[COMP_Cb] != MtsType::SKIP;
  const bool      trCr   = cbfCr && tu.mtsIdx[COMP_Cr] != MtsType::SKIP;
  const bool      tsCb   = cbfCb && tu.mtsIdx[COMP_Cb] == MtsType::SKIP;
  const bool      tsCr   = cbfCr && tu.mtsIdx[COMP_Cr] == MtsType::SKIP;
  const bool      trOnly = (trCb && !cbfCr) || (trCr && !cbfCb) || (trCb && trCr);
  const bool      tsOnly = (tsCb && !cbfCr) || (tsCr && !cbfCb) || (tsCb && tsCr);
  const TransList trTypes =
    TU::getTransCandInter(tu, COMP_Cb, lossless, m_encCfg->m_useChromaTS, m_histBestMtsIdx, m_histBestSbt);
  tl.trTypes.clear();
  for (const auto &tr: trTypes)
  {
    if ((tr == MtsType::SKIP && trOnly) || (tr != MtsType::SKIP && tsOnly))
    {
      continue;
    }
    tl.trTypes.push_back(tr);
  }
  CHECK(tl.trTypes.empty(), "Empty transform list for JCCR");

  //--- do all transforms ---
  m_pcTrQuant->preCalcTrans(tu, COMP_Cb, resJCCR, tl.trTypes, tl.trCoeffs);

  //----- select transforms -----
  xSelectTrCand(tu, JOINT_CbCr, tl);

  return cbfMasks;
}

TrEst::Cost InterSearch::xSelectTrCand(TransformUnit &tu, const CompID compId, PreTrList &tl)
{
  TrEst::Cost costDCT(0, MtsType::NONE, std::numeric_limits<double>::max());
  CHECK(tl.trTypes.empty(), "empty candidate list");
  if (tl.trTypes.size() == 1)
  {
    return costDCT;
  }

  //----- calculate simple cost measures and empty transform lists -----
  PreCost                  preCost(tu, compId, m_CABACEstimator, m_pcRdCost);
  std::vector<TrEst::Cost> cl;
  {
    cl.reserve(to_underlying(MtsType::NUM));
    preCost.init(tl);
    for (const MtsType tr: tl.trTypes)
    {
      if (tr == MtsType::DCT2_DCT2)
      {
        costDCT = preCost(tr);
      }
      else
      {
        cl.push_back(preCost(tr));
      }
    }
    tl.trTypes.clear();
  }
  CHECK(costDCT.tr != MtsType::DCT2_DCT2, "Initial transform list must contain DCT, unless it only contains TSKIP");

  //----- derive cost threshold and get single sorted list -----
  const double costThreshold = costDCT.cost * 1.2;   // precost threshold : inter
  tl.trTypes.push_back(costDCT.tr);
  sortCL(cl);

  //---- remove transform candidates -----
  while (!cl.empty() && cl.back().cost > costThreshold)
  {
    cl.pop_back();
  }

  //----- add to transform lists -----
  for (const auto &cand: cl)
  {
    tl.trTypes.push_back(cand.tr);
  }
  return costDCT;
}

void InterSearch::xSelectTrCandUpd(TransformUnit &tu, const CompID compId, PreTrList &tl, TrEst::Cost &costDCT)
{
  if (tl.trTypes.empty())
  {
    return;
  }

  //----- calculate simple cost measures and empty transform lists -----
  PreCost                  preCost(tu, compId, m_CABACEstimator, m_pcRdCost);
  std::vector<TrEst::Cost> cl;
  cl.reserve(to_underlying(MtsType::NUM));
  preCost.init(tl);
  if (costDCT.tr != MtsType::DCT2_DCT2)
  {
    costDCT = preCost(MtsType::DCT2_DCT2);
  }
  for (const MtsType tr: tl.trTypes)
  {
    cl.push_back(preCost(tr));
  }
  tl.trTypes.clear();

  //----- derive cost threshold and get single sorted list -----
  const double costThreshold = costDCT.cost * 1.10;   // precost threshold : inter
  sortCL(cl);

  //---- remove transform candidates -----
  while (!cl.empty() && cl.back().cost > costThreshold)
  {
    cl.pop_back();
  }

  //----- add to transform lists -----
  for (const auto &cand: cl)
  {
    tl.trTypes.push_back(cand.tr);
  }
}

InterSearch::PreCost::PreCost(TransformUnit &tu, const CompID cId, CABACWriter *cabacEst, RdCost *rdCost)
  : TrEst::PreCostBase(cId, tu, rdCost->getMotionLambda(), 1.15, 1.0)
  , bitEst(cabacEst)
{}

void InterSearch::PreCost::init(const PreTrList &tl)
{
  trList = &tl;
  TrEst::PreCostBase::initResidual(trList->trTypes, trList->res[compId]);
}

TrEst::Cost InterSearch::PreCost::operator()(const MtsType tr)
{
  double cost = estTransBits(tr);
  cost += TrEst::PreCostBase::getEstTrCost(tr, trList->trCoeffs);
  return TrEst::Cost(0, tr, cost);
}

double InterSearch::PreCost::estTransBits(const MtsType tr)
{
  const CompID cid = (compId == CompID::JOINT_CbCr ? CompID::COMP_Cb : compId);
  const auto   cbf = tu.cbf[cid];
  tu.mtsIdx[cid]   = tr;
  tu.cbf[cid]      = 1;   // for mts_idx coding
  const bool upd   = bitEst->countWithUpdate(false);
  bitEst->resetBits();
  bitEst->ts_flag(tu, cid);
  bitEst->nst_idx(tu, cuCtx);
  bitEst->mts_idx(tu, cuCtx);
  bitEst->countWithUpdate(upd);
  tu.cbf[cid] = cbf;
  return FRAC_BITS_SCALE * double(bitEst->getEstFracBits());
}

void InterSearch::xEstimateInterResidualQT(CodingStructure &cs, Partitioner &partitioner,
                                           Distortion *puiZeroDist /*= nullptr*/
                                           ,
                                           const bool luma, const bool chroma, PelUnitBuf *orgResi)
{
  const UnitArea &currArea = partitioner.currArea();
  const SPS      &sps      = *cs.sps;
  m_pcRdCost->setChromaFormat(sps.m_chromaFormatIdc);

  const uint32_t    numValidComp = getNumberValidComponents(sps.m_chromaFormatIdc);
  const uint32_t    numTBlocks   = getNumberValidTBlocks(*cs.pcv);
  const CodingUnit &cu           = *cs.getCU(partitioner.chType);
  const unsigned    currDepth    = partitioner.currTrDepth;

  bool checkFull = !partitioner.canSplit(TU_MAX_TR_SPLIT, cs);
  if (cu.sbtInfo && partitioner.canSplit(PartSplit(cu.getSbtTuSplit()), cs))
  {
    checkFull = false;
  }
  bool checkSplit = !checkFull;

  // get temporary data
  CodingStructure *csSplit = nullptr;
  CodingStructure *csFull  = nullptr;
  if (checkSplit)
  {
    csSplit = &cs;
  }
  else if (checkFull)
  {
    csFull = &cs;
  }

  Distortion    uiSingleDist        = 0;
  Distortion    uiSingleDistComp[3] = { 0, 0, 0 };
  const TempCtx ctxStart(m_ctxPool, m_CABACEstimator->getCtx());

  if (checkFull)
  {
    TransformUnit &tu = csFull->addTU(CS::getArea(cs, currArea, partitioner.chType), partitioner.chType);
    tu.depth          = currDepth;

    tu.mtsIdx.fill(MtsType::DCT2_DCT2);

    tu.checkTuNoResidual(partitioner.currPartIdx());
    Position tuPos = tu.Y();
    tuPos.relativeTo(cu.Y());
    const UnitArea relativeUnitArea(tu.chromaFormat, Area(tuPos, tu.Y().size()));

    const Slice &slice = *cs.slice;
    if (slice.m_lmcsEnabledFlag && slice.m_picHeader->m_lmcsChromaResidualScaleFlag &&
        !(CS::isDualITree(cs) && slice.isIntra() && CU::isIBC(*tu.cu)))
    {
      const CompArea &areaY = tu.cu->blocks[COMP_Y];
      int             adj   = m_pcReshape->calculateChromaAdjVpduNei(tu, areaY);
      tu.setChromaAdj(adj);
    }

    memset(m_pTempPel, 0,
           sizeof(Pel) * tu.Y().area());   // not necessary needed for inside of recursion (only at the beginning)

    double minCost[MAX_NUM_TBLOCKS] = { MAX_DOUBLE, MAX_DOUBLE, MAX_DOUBLE };

    CodingStructure &saveCS = *m_pSaveCS[0];
    saveCS.pcv              = cs.pcv;
    saveCS.picture          = cs.picture;
    saveCS.compactResize(currArea);
    saveCS.clearTUs();
    TransformUnit &bestTU    = saveCS.addTU(CS::getArea(cs, currArea, partitioner.chType), partitioner.chType);
    const bool     checkJCCR = (chroma && isChromaEnabled(tu.chromaFormat) && tu.blocks[COMP_Cb].valid() &&
                            sps.m_jointCbCrEnabledFlag && !tu.noResidual);
    auto          &tl        = m_trCandList;

    for (uint32_t c = 0; c < numTBlocks; c++)
    {
      const CompID compID = CompID(c);
      if (compID == COMP_Y && !luma)
      {
        continue;
      }
      if (compID != COMP_Y && !chroma)
      {
        continue;
      }
      const CompArea &compArea        = tu.blocks[compID];
      const int       channelBitDepth = sps.m_bitDepths[toChannelType(compID)];
      const bool lumaReshaping = isLuma(compID) && slice.m_picHeader->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag &&
        !tu.cu->ciipFlag && !tu.cu->gpmIntraFlag && !CU::isIBC(*tu.cu);

      if (!tu.blocks[compID].valid())
      {
        continue;
      }

      bool trTested = false;
      auto dctCost  = xPreCalcTrans(tu, compID, tl);
      for (int trGroup = 0; trGroup < 2; trGroup++)
      {
        if (trGroup)
        {
          if (!TU::getCbfAtDepth(bestTU, compID, currDepth))
          {
            break;
          }
          if (m_encCfg->m_useTransformSkipFast && bestTU.mtsIdx[compID] == MtsType::SKIP)
          {
            break;
          }
          xPreCalcTransUpd(tu, compID, tl, dctCost);
        }
        for (const auto tr: tl.trTypes)
        {
          const bool isFirstTr = (!trGroup && tr == tl.trTypes.front());
          if (trTested && tr != MtsType::SKIP && !TU::getCbfAtDepth(bestTU, compID, currDepth))
          {
            continue;
          }
          tu.mtsIdx[compID] = tr;
          trTested          = trTested || tr != MtsType::SKIP;

          //----- init QP and lambda -----
          QpParam cQP(tu, compID);
#if RDOQ_CHROMA_LAMBDA
          m_pcTrQuant->selectLambda(compID);
#endif
          if (slice.m_lmcsEnabledFlag && isChroma(compID) && slice.m_picHeader->m_lmcsChromaResidualScaleFlag)
          {
            double cRescale = (double)(1 << CSCALE_FP_PREC) / (double)(tu.getChromaAdj());
            m_pcTrQuant->setLambda(m_pcTrQuant->getLambda() / (cRescale * cRescale));
          }
          if (sps.m_jointCbCrEnabledFlag && isChroma(compID) && (tu.cu->cs->slice->m_iSliceQp > 18))
          {
            m_pcTrQuant->setLambda(1.05 * m_pcTrQuant->getLambda());
          }

          //----- quantization -----
          m_CABACEstimator->getCtx() = ctxStart;
          TCoeff currAbsSum          = 0;
          m_pcTrQuant->quantNxN(tu, compID, cQP, currAbsSum, m_CABACEstimator->getCtx(), tl.trCoeffs);

          //----- calculate rd-costs -----
          m_CABACEstimator->resetBits();
          uint64_t   currCompFracBits = 0;
          Distortion currCompDist     = 0;
          double     currCompCost     = 0;
          uint64_t   nonCoeffFracBits = 0;
          Distortion nonCoeffDist     = 0;
          double     nonCoeffCost     = 0;

          if (isFirstTr)
          {
            // initialized with zero residual distortion
            const CPelBuf zeroBuf(m_pTempPel, compArea);
            nonCoeffDist =
              m_pcRdCost->getDistPart(zeroBuf, csFull->getOrgResiBuf(compArea), channelBitDepth, compID, DFunc::SSE);
            if (!tu.noResidual)
            {
              const bool prevCbf = (compID == COMP_Cr ? tu.cbf[COMP_Cb] : false);
              m_CABACEstimator->cbf_comp(false, compArea, currDepth, prevCbf, BdpcmMode::NONE);
            }
            nonCoeffFracBits = m_CABACEstimator->getEstFracBits();
#if WCG_EXT
            if (m_encCfg->m_lumaLevelToDeltaQPMapping.isEnabled())
            {
              nonCoeffCost = m_pcRdCost->calcRdCost(nonCoeffFracBits, nonCoeffDist, false);
            }
            else
#endif
            {
              nonCoeffCost = m_pcRdCost->calcRdCost(nonCoeffFracBits, nonCoeffDist);
            }

            if (puiZeroDist != nullptr)
            {
              *puiZeroDist += nonCoeffDist;   // initialized with zero residual distortion
            }
          }

          if (currAbsSum >
              0)   // if non-zero coefficients are present, a residual needs to be derived for further prediction
          {
            if (isFirstTr)
            {
              m_CABACEstimator->getCtx() = ctxStart;
              m_CABACEstimator->resetBits();
            }

            const bool prevCbf = (compID == COMP_Cr ? tu.cbf[COMP_Cb] : false);
            m_CABACEstimator->cbf_comp(true, compArea, currDepth, prevCbf, BdpcmMode::NONE);
            if (compID == COMP_Cr)
            {
              const int cbfMask = (tu.cbf[COMP_Cb] ? CBF_MASK_CB : 0) + CBF_MASK_CR;
              m_CABACEstimator->joint_cb_cr(tu, cbfMask);
            }

            bool invalidTrans = false;
            if (isMTS(tu.mtsIdx[compID]) || isNST(tu.mtsIdx[compID]))
            {
              auto   tclevels = tu.getCoeffs(COMP_Y);
              TCoeff sumAbsAC = currAbsSum - abs(tclevels.buf[0]);
              invalidTrans    = (sumAbsAC == 0);
            }

            bool hasSignPred = false;

            if (!invalidTrans)
            {
              hasSignPred = m_pcTrQuant->prdCoeffSigns(tu, compID, lumaReshaping ? &m_pcReshape->m_fwdLUT : nullptr);

              CUCtx cuCtx;
              cuCtx.isDQPCoded         = true;
              cuCtx.isChromaQpAdjCoded = true;
              m_CABACEstimator->residual_coding_last(tu, compID, &cuCtx);
              m_CABACEstimator->residual_coding_coef(tu, compID, &cuCtx);
              m_CABACEstimator->residual_coding_sign(tu, compID);
              m_CABACEstimator->nst_idx(tu, cuCtx);
              m_CABACEstimator->mts_idx(tu, cuCtx);
              if (isMTS(tu.mtsIdx[compID]))
              {
                invalidTrans = (!cuCtx.mtsLastScanPos || cuCtx.violatesMtsCoeffConstraint);
              }
              else if (isNST(tu.mtsIdx[compID]))
              {
                invalidTrans = (!cuCtx.lfnstLastScanPos || cuCtx.violatesLfnstConstrained[toChannelType(compID)]);
              }
            }

            if (invalidTrans)
            {
              currCompCost = MAX_DOUBLE;
            }
            else
            {
              currCompFracBits   = m_CABACEstimator->getEstFracBits();
              PelBuf  resiBuf    = csFull->getResiBuf(compArea);
              CPelBuf orgResiBuf = csFull->getOrgResiBuf(compArea);

              m_pcTrQuant->invTransformNxN(tu, compID, resiBuf, cQP, hasSignPred);
              if (slice.m_lmcsEnabledFlag && isChroma(compID) && slice.m_picHeader->m_lmcsChromaResidualScaleFlag &&
                  tu.blocks[compID].area() > 4)
              {
                resiBuf.scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(compID));
              }
              // getCbf() is going to be 1 since currAbsSum > 0 here, according to the if-statement a couple of lines
              // up.
              if (cs.pps->m_BIF && isLuma(compID) && m_bilateralFilter->getApplyBIF(tu, compID))
              {
                CompArea tmpArea1(compID, tu.chromaFormat, Position(0, 0), Size(resiBuf.width, resiBuf.height));

                PelBuf tmpRecLuma = m_tmpStorageCtu.getBuf(tmpArea1);
                tmpRecLuma.copyFrom(resiBuf);

                const CPelBuf predBuf     = csFull->getPredBuf(compArea);
                PelBuf        recIPredBuf = csFull->slice->m_pic->getRecoBuf(compArea);
                CPelBuf       reco        = csFull->getRecoBuf(compID);
                m_bilateralFilter->bilateralFilterRDOdiamond5x5(compID, tmpRecLuma, predBuf, tmpRecLuma, tu.cu->qp,
                                                                recIPredBuf, reco, cs.slice->clpRng(compID), tu, false);
                currCompDist = m_pcRdCost->getDistPart(orgResiBuf, tmpRecLuma, channelBitDepth, compID, DFunc::SSE);
              }
              else
              {
                if (isChroma(compID))
                {
                  if (cs.pps->m_chromaBIF && isChroma(compID) && m_bilateralFilter->getApplyBIF(tu, compID))
                  {
                    // chroma and bilateral
                    CompArea tmpArea1(compID, tu.chromaFormat, Position(0, 0), Size(resiBuf.width, resiBuf.height));
                    PelBuf   tmpRecChroma = m_tmpStorageCtu.getBuf(tmpArea1);
                    tmpRecChroma.copyFrom(resiBuf);

                    const CPelBuf predBuf     = csFull->getPredBuf(compArea);
                    PelBuf        recIPredBuf = csFull->slice->m_pic->getRecoBuf(compArea);
                    CPelBuf       reco        = csFull->getRecoBuf(compID);
                    m_bilateralFilter->bilateralFilterRDOdiamond5x5(compID, tmpRecChroma, predBuf, tmpRecChroma,
                                                                    tu.cu->qp, recIPredBuf, reco,
                                                                    cs.slice->clpRng(compID), tu, false);
                    currCompDist =
                      m_pcRdCost->getDistPart(orgResiBuf, tmpRecChroma, channelBitDepth, compID, DFunc::SSE);
                  }
                  else
                  {   // chroma but not bilateral
                    currCompDist = m_pcRdCost->getDistPart(orgResiBuf, resiBuf, channelBitDepth, compID, DFunc::SSE);
                  }
                }
                else
                {   // luma but not bilateral
                  currCompDist = m_pcRdCost->getDistPart(orgResiBuf, resiBuf, channelBitDepth, compID, DFunc::SSE);
                }
              }

#if WCG_EXT
              currCompCost = m_pcRdCost->calcRdCost(currCompFracBits, currCompDist, false);
#else
              currCompCost = m_pcRdCost->calcRdCost(currCompFracBits, currCompDist);
#endif
            }
          }
          else if (!isFirstTr)
          {
            currCompCost = MAX_DOUBLE;
          }
          else
          {
            csFull->getResiBuf(compArea).fill(0);
            tu.getCoeffs(compID).fill(0);
            TU::setCbfAtDepth(tu, compID, currDepth, false);
            currCompFracBits = nonCoeffFracBits;
            currCompDist     = nonCoeffDist;
            currCompCost     = nonCoeffCost;
          }

          //----- compare costs -----
          if (currCompCost < minCost[compID])
          {
            // copy component
            if (isFirstTr && nonCoeffCost < currCompCost)   // check for forced null
            {
              csFull->getResiBuf(compArea).fill(0);
              tu.getCoeffs(compID).fill(0);
              TU::setCbfAtDepth(tu, compID, currDepth, false);
              currAbsSum       = 0;
              currCompFracBits = nonCoeffFracBits;
              currCompDist     = nonCoeffDist;
              currCompCost     = nonCoeffCost;
            }

            uiSingleDistComp[compID] = currCompDist;
            minCost[compID]          = currCompCost;

            bestTU.copyComponentFrom(tu, compID);
            saveCS.getResiBuf(compArea).copyFrom(csFull->getResiBuf(compArea));
          }
          if (tu.noResidual)
          {
            CHECK(currCompFracBits > 0 || currAbsSum, "currCompFracBits > 0 when tu noResidual");
          }
        }
      }

      // copy component
      tu.copyComponentFrom(bestTU, compID);
      csFull->getResiBuf(compArea).copyFrom(saveCS.getResiBuf(compArea));

      PelBuf picRecoBuf = csFull->getRecoBuf(compArea);
      ClpRng clpRngTmp  = cs.slice->setNewClipRange(true, &m_pcReshape->m_fwdLUT, compID);
      if (lumaReshaping)
      {
        picRecoBuf.copyFrom(cs.getPredBuf(compArea));
        picRecoBuf.rspSignal(m_pcReshape->m_fwdLUT);
        picRecoBuf.reconstruct(picRecoBuf, csFull->getResiBuf(compArea), clpRngTmp);
      }
      else
      {
        picRecoBuf.reconstruct(cs.getPredBuf(compArea), csFull->getResiBuf(compArea), clpRngTmp);
      }
      tu.cs->picture->getRecoBuf(compArea).copyFrom(picRecoBuf);
    }   // component loop

    if (checkJCCR)
    {
      const CompArea &cbArea          = tu.blocks[COMP_Cb];
      const CompArea &crArea          = tu.blocks[COMP_Cr];
      const int       channelBitDepth = sps.m_bitDepths[toChannelType(COMP_Cb)];
      const bool      reshape =
        slice.m_lmcsEnabledFlag && slice.m_picHeader->m_lmcsChromaResidualScaleFlag && cbArea.area() > 4;
      const CbfMaskList cbfMaskList = xPreCalcTransJCCR(tu, tl);
      double            minCostCbCr = minCost[COMP_Cb] + minCost[COMP_Cr];
      bool              lastIsBest  = true;

      for (const auto cbfMask: cbfMaskList)
      {
        CompID codedComp = (cbfMask & CBF_MASK_CB) != 0 ? COMP_Cb : COMP_Cr;
        CompID otherComp = codedComp == COMP_Cr ? COMP_Cb : COMP_Cr;

        //----- set lambda -----
        m_pcTrQuant->selectLambda(codedComp);
        // Lambda is loosened for the joint mode with respect to single modes as the same residual is used for both
        // chroma blocks
        const int    absIct = abs(TU::getICTMode(tu));
        const double lfact  = (absIct == 1 || absIct == 3 ? 0.8 : 0.5);
        m_pcTrQuant->setLambda(lfact * m_pcTrQuant->getLambda());
        if (tu.cu->cs->slice->m_iSliceQp > 18)
        {
          m_pcTrQuant->setLambda(1.05 * m_pcTrQuant->getLambda());
        }
        if (reshape)
        {
          double cRescale = (double)(1 << CSCALE_FP_PREC) / (double)tu.getChromaAdj();
          m_pcTrQuant->setLambda(m_pcTrQuant->getLambda() / (cRescale * cRescale));
        }

        //----- check transforms -----
        bool JCCRInvalid = false;
        for (const auto trType: tl.trTypes)
        {
          if (JCCRInvalid)
          {
            break;
          }
          TCoeff     absSum         = 0;
          Distortion currCompDistCb = 0;
          Distortion currCompDistCr = 0;
          double     currCompCost   = 0;
          lastIsBest                = false;
          tu.jointCbCr              = (uint8_t)cbfMask;
          tu.mtsIdx[codedComp]      = trType;
          tu.mtsIdx[otherComp]      = trType;

          m_CABACEstimator->getCtx() = ctxStart;
          m_CABACEstimator->resetBits();

          QpParam qpCbCr(tu, codedComp);
          m_pcTrQuant->quantNxN(tu, codedComp, qpCbCr, absSum, m_CABACEstimator->getCtx(), tl.trCoeffs);

          if (absSum)
          {
            TU::setCbfAtDepth(tu, otherComp, tu.depth, tu.jointCbCr == 3);
            tu.getCoeffs(otherComp).fill(0);

            //----- reconstruct residuals -----
            PelBuf resCb = csFull->getResiBuf(cbArea);
            PelBuf resCr = csFull->getResiBuf(crArea);
            m_pcTrQuant->invTransformNxN(tu, codedComp, (codedComp == COMP_Cb ? resCb : resCr), qpCbCr);
            m_pcTrQuant->invTransformICT(tu, resCb, resCr);
            if (reshape)
            {
              resCb.scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(COMP_Cb));
              resCr.scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(COMP_Cr));
            }

            //----- calculate distortion -----
            currCompDistCb =
              m_pcRdCost->getDistPart(csFull->getOrgResiBuf(cbArea), resCb, channelBitDepth, COMP_Cb, DFunc::SSE);
            currCompDistCr =
              m_pcRdCost->getDistPart(csFull->getOrgResiBuf(crArea), resCr, channelBitDepth, COMP_Cr, DFunc::SSE);

            //----- estimate rate -----
            const bool cbfCb = (cbfMask & CBF_MASK_CB) != 0;
            const bool cbfCr = (cbfMask & CBF_MASK_CR) != 0;
            m_CABACEstimator->cbf_comp(cbfCb, cbArea, currDepth, false, BdpcmMode::NONE);
            m_CABACEstimator->cbf_comp(cbfCr, crArea, currDepth, cbfCb, BdpcmMode::NONE);
            m_CABACEstimator->joint_cb_cr(tu, cbfMask);
            if (cbfCb)
            {
              m_CABACEstimator->residual_coding_last(tu, COMP_Cb);
              m_CABACEstimator->residual_coding_coef(tu, COMP_Cb);
              m_CABACEstimator->residual_coding_sign(tu, COMP_Cb);
            }
            if (cbfCr)
            {
              m_CABACEstimator->residual_coding_last(tu, COMP_Cr);
              m_CABACEstimator->residual_coding_coef(tu, COMP_Cr);
              m_CABACEstimator->residual_coding_sign(tu, COMP_Cr);
            }
            uint64_t currCompFracBits = m_CABACEstimator->getEstFracBits();

            //----- set rd-cost -----
#if WCG_EXT
            currCompCost = m_pcRdCost->calcRdCost(currCompFracBits, currCompDistCr + currCompDistCb, false);
#else
            currCompCost = m_pcRdCost->calcRdCost(currCompFracBits, currCompDistCr + currCompDistCb);
#endif
          }
          else
          {
            currCompCost = MAX_DOUBLE;
            JCCRInvalid  = true;
          }

          //----- evaluate -----
          if (currCompCost < minCostCbCr)
          {
            lastIsBest                = true;
            uiSingleDistComp[COMP_Cb] = currCompDistCb;
            uiSingleDistComp[COMP_Cr] = currCompDistCr;
            minCostCbCr               = currCompCost;

            bestTU.copyComponentFrom(tu, COMP_Cb);
            bestTU.copyComponentFrom(tu, COMP_Cr);
            saveCS.getResiBuf(cbArea).copyFrom(csFull->getResiBuf(cbArea));
            saveCS.getResiBuf(crArea).copyFrom(csFull->getResiBuf(crArea));
          }
        }
      }
      //---- copy back ----
      if (!lastIsBest)
      {
        tu.copyComponentFrom(bestTU, COMP_Cb);
        tu.copyComponentFrom(bestTU, COMP_Cr);
        csFull->getResiBuf(cbArea).copyFrom(saveCS.getResiBuf(cbArea));
        csFull->getResiBuf(crArea).copyFrom(saveCS.getResiBuf(crArea));
      }
    }

    if (tu.jointCbCr)
    {
      for (int c = (int)COMP_Cb; c <= (int)COMP_Cr; c++)
      {
        CompID comp       = CompID(c);
        PelBuf picRecoBuf = csFull->getRecoBuf(tu.blocks[comp]);

        picRecoBuf.reconstruct(cs.getPredBuf(tu.blocks[comp]), csFull->getResiBuf(tu.blocks[comp]),
                               tu.cu->cs->slice->clpRng(comp));
        tu.cs->picture->getRecoBuf(tu.blocks[comp]).copyFrom(picRecoBuf);
      }

      m_pcTrQuant->prdCoeffSigns(tu, COMP_Cb);
    }

    m_CABACEstimator->getCtx() = ctxStart;
    m_CABACEstimator->resetBits();
    if (!tu.noResidual)
    {
      static const CompID cbfGetComp[MAX_NUM_COMP] = { COMP_Cb, COMP_Cr, COMP_Y };
      for (unsigned c = isChromaEnabled(tu.chromaFormat) ? 0 : 2; c < MAX_NUM_COMP; c++)
      {
        const CompID compID = cbfGetComp[c];
        if (compID == COMP_Y && !luma)
        {
          continue;
        }
        if (compID != COMP_Y && !chroma)
        {
          continue;
        }
        if (tu.blocks[compID].valid())
        {
          const bool prevCbf = (compID == COMP_Cr ? TU::getCbfAtDepth(tu, COMP_Cb, currDepth) : false);
          m_CABACEstimator->cbf_comp(TU::getCbfAtDepth(tu, compID, currDepth), tu.blocks[compID], currDepth, prevCbf,
                                     BdpcmMode::NONE);
        }
      }
    }

    for (uint32_t ch = 0; ch < numValidComp; ch++)
    {
      const CompID compID = CompID(ch);
      if (compID == COMP_Y && !luma)
      {
        continue;
      }
      if (compID != COMP_Y && !chroma)
      {
        continue;
      }
      if (tu.blocks[compID].valid())
      {
        if (compID == COMP_Cr)
        {
          const int cbfMask = (TU::getCbf(tu, COMP_Cb) ? CBF_MASK_CB : 0) + (TU::getCbf(tu, COMP_Cr) ? CBF_MASK_CR : 0);
          m_CABACEstimator->joint_cb_cr(tu, cbfMask);
        }
        if (TU::getCbf(tu, compID))
        {
          m_CABACEstimator->residual_coding_last(tu, compID);
          m_CABACEstimator->residual_coding_coef(tu, compID);
          m_CABACEstimator->residual_coding_sign(tu, compID);
        }
        uiSingleDist += uiSingleDistComp[compID];
      }
    }
    if (tu.noResidual)
    {
      CHECK(m_CABACEstimator->getEstFracBits() > 0, "no residual TU's bits shall be 0");
    }

    csFull->fracBits += m_CABACEstimator->getEstFracBits();
    csFull->dist += uiSingleDist;
#if WCG_EXT
    if (m_encCfg->m_lumaLevelToDeltaQPMapping.isEnabled())
    {
      csFull->cost = m_pcRdCost->calcRdCost(csFull->fracBits, csFull->dist, false);
    }
    else
#endif
    {
      csFull->cost = m_pcRdCost->calcRdCost(csFull->fracBits, csFull->dist);
    }
  }   // check full

  // code sub-blocks
  if (checkSplit)
  {
    if (checkFull)
    {
      m_CABACEstimator->getCtx() = ctxStart;
    }

    if (partitioner.canSplit(TU_MAX_TR_SPLIT, cs))
    {
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
      xEstimateInterResidualQT(*csSplit, partitioner, checkFull ? nullptr : puiZeroDist, luma, chroma, orgResi);

      csSplit->cost = m_pcRdCost->calcRdCost(csSplit->fracBits, csSplit->dist);
    } while (partitioner.nextPart(*csSplit));

    partitioner.exitCurrSplit();

    unsigned anyCbfSet  = 0;
    unsigned compCbf[3] = { 0, 0, 0 };

    if (!checkFull)
    {
      for (auto &currTU: csSplit->traverseTUs(currArea, partitioner.chType))
      {
        for (unsigned ch = 0; ch < numTBlocks; ch++)
        {
          compCbf[ch] |= (TU::getCbfAtDepth(currTU, CompID(ch), currDepth + 1) ? 1 : 0);
        }
      }

      for (auto &currTU: csSplit->traverseTUs(currArea, partitioner.chType))
      {
        TU::setCbfAtDepth(currTU, COMP_Y, currDepth, compCbf[COMP_Y]);
        if (isChromaEnabled(currArea.chromaFormat))
        {
          TU::setCbfAtDepth(currTU, COMP_Cb, currDepth, compCbf[COMP_Cb]);
          TU::setCbfAtDepth(currTU, COMP_Cr, currDepth, compCbf[COMP_Cr]);
        }
      }

      anyCbfSet = compCbf[COMP_Y];
      if (isChromaEnabled(currArea.chromaFormat))
      {
        anyCbfSet |= compCbf[COMP_Cb];
        anyCbfSet |= compCbf[COMP_Cr];
      }

      m_CABACEstimator->getCtx() = ctxStart;
      m_CABACEstimator->resetBits();

      // when compID isn't a channel, code Cbfs:
      xEncodeInterResidualQT(*csSplit, partitioner, MAX_NUM_TBLOCKS);
      for (uint32_t ch = 0; ch < numValidComp; ch++)
      {
        const CompID compID = CompID(ch);
        if (compID == COMP_Y && !luma)
        {
          continue;
        }
        if (compID != COMP_Y && !chroma)
        {
          continue;
        }
        xEncodeInterResidualQT(*csSplit, partitioner, CompID(ch));
      }

      csSplit->fracBits = m_CABACEstimator->getEstFracBits();
      csSplit->cost     = m_pcRdCost->calcRdCost(csSplit->fracBits, csSplit->dist);

      if (checkFull && anyCbfSet && csSplit->cost < csFull->cost)
      {
        cs.useSubStructure(*csSplit, partitioner.chType, currArea, false, false, false, true, true);
        cs.cost = csSplit->cost;
      }
    }

    if (csSplit && csFull)
    {
      csSplit->releaseIntermediateData();
      csFull->releaseIntermediateData();
    }
  }
}

void InterSearch::encodeResAndCalcRdInterCU(CodingStructure &cs, Partitioner &partitioner, const bool &skipResidual,
                                            const bool luma, const bool chroma)
{
  m_pcRdCost->setChromaFormat(cs.sps->m_chromaFormatIdc);

  CodingUnit &cu = *cs.getCU(partitioner.chType);

  const ChromaFormat format = cs.area.chromaFormat;
  ;
  const int  numValidComponents = getNumberValidComponents(format);
  const SPS &sps                = *cs.sps;

  if (skipResidual)   //  No residual coding : SKIP mode
  {
    cu.skip    = true;
    cu.rootCbf = false;
    CHECK(cu.sbtInfo != 0, "sbtInfo shall be 0 if CU has no residual");
    cs.getResiBuf().fill(0);
    {
      cs.getRecoBuf().copyFrom(cs.getPredBuf());
      if (m_encCfg->m_lmcsEnabled && (cs.slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag) && !cu.ciipFlag &&
          !CU::isIBC(cu) && !cu.gpmIntraFlag)
      {
        cs.getRecoBuf().Y().rspSignal(m_pcReshape->m_fwdLUT);
      }
    }

    if (luma)
    {
      ClpRng  clpRngLuma       = cu.slice->setNewClipRange(true, &m_pcReshape->m_fwdLUT, COMP_Y);
      ClpRngs clpRngs          = cu.slice->m_clpRngs;
      clpRngs.comp[COMP_Y].min = clpRngLuma.min;
      clpRngs.comp[COMP_Y].max = clpRngLuma.max;

      cs.getRecoBuf().copyClip(cs.getRecoBuf(), clpRngs, true, false);
    }

    // add empty TU(s)
    cs.addEmptyTUs(partitioner);
    Distortion distortion = 0;

    for (int comp = 0; comp < numValidComponents; comp++)
    {
      const CompID compID = CompID(comp);
      if (compID == COMP_Y && !luma)
      {
        continue;
      }
      if (compID != COMP_Y && !chroma)
      {
        continue;
      }
      CPelBuf reco = cs.getRecoBuf(compID);
      CPelBuf org  = cs.getOrgBuf(compID);
#if WCG_EXT
      if (m_encCfg->m_lumaLevelToDeltaQPMapping.isEnabled() ||
          (m_encCfg->m_lmcsEnabled && (cs.slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag)))
      {
        const CPelBuf orgLuma = cs.getOrgBuf(cs.area.blocks[COMP_Y]);
        if (compID == COMP_Y && !(m_encCfg->m_lumaLevelToDeltaQPMapping.isEnabled()))
        {
          const CompArea &areaY = cu.Y();

          CompArea tmpArea1(COMP_Y, areaY.chromaFormat, Position(0, 0), areaY.size());
          PelBuf   tmpRecLuma = m_tmpStorageCtu.getBuf(tmpArea1);
          tmpRecLuma.copyFrom(reco);
          tmpRecLuma.rspSignal(m_pcReshape->m_invLUT);
          distortion += m_pcRdCost->getDistPart(org, tmpRecLuma, sps.m_bitDepths[toChannelType(compID)], compID,
                                                DFunc::SSE_WTD, &orgLuma);
        }
        else
        {
          distortion += m_pcRdCost->getDistPart(org, reco, sps.m_bitDepths[toChannelType(compID)], compID,
                                                DFunc::SSE_WTD, &orgLuma);
        }
      }
      else
#endif
      {
        distortion += m_pcRdCost->getDistPart(org, reco, sps.m_bitDepths[toChannelType(compID)], compID, DFunc::SSE);
      }
    }

    m_CABACEstimator->resetBits();

    CodingUnit &cu = *cs.getCU(partitioner.chType);

    m_CABACEstimator->cu_skip_flag(cu);
    m_CABACEstimator->merge_data(cu);

    cs.dist     = distortion;
    cs.fracBits = m_CABACEstimator->getEstFracBits();
    cs.cost     = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);

    return;
  }

  //  Residual coding.
  if (luma)
  {
    cs.getResiBuf().bufs[0].copyFrom(cs.getOrgBuf().bufs[0]);
    if (cs.slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag)
    {
      const CompArea &areaY = cu.Y();
      CompArea        tmpArea(COMP_Y, areaY.chromaFormat, Position(0, 0), areaY.size());
      PelBuf          tmpPred = m_tmpStorageCtu.getBuf(tmpArea);
      tmpPred.copyFrom(cs.getPredBuf(COMP_Y));

      if (!cu.ciipFlag && !cu.gpmIntraFlag && !CU::isIBC(cu))
      {
        tmpPred.rspSignal(m_pcReshape->m_fwdLUT);
      }
      cs.getResiBuf(COMP_Y).rspSignal(m_pcReshape->m_fwdLUT);
      cs.getResiBuf(COMP_Y).subtract(tmpPred);
    }
    else
    {
      cs.getResiBuf().bufs[0].subtract(cs.getPredBuf().bufs[0]);
    }
  }
  if (chroma && isChromaEnabled(cs.pcv->chrFormat))
  {
    cs.getResiBuf().bufs[1].copyFrom(cs.getOrgBuf().bufs[1]);
    cs.getResiBuf().bufs[2].copyFrom(cs.getOrgBuf().bufs[2]);
    cs.getResiBuf().bufs[1].subtract(cs.getPredBuf().bufs[1]);
    cs.getResiBuf().bufs[2].subtract(cs.getPredBuf().bufs[2]);
  }
  const UnitArea   curUnitArea = partitioner.currArea();
  CodingStructure &saveCS      = *m_pSaveCS[1];
  saveCS.pcv                   = cs.pcv;
  saveCS.picture               = cs.picture;
  saveCS.compactResize(curUnitArea);
  saveCS.clearCUs();
  saveCS.clearTUs();
  for (const auto &ppcu: cs.cus)
  {
    CodingUnit &pcu = saveCS.addCU(*ppcu, ppcu->chType);
    pcu             = *ppcu;
  }

  PelUnitBuf     orgResidual;
  const UnitArea localUnitArea(cs.area.chromaFormat, Area(0, 0, cu.lwidth(), cu.lheight()));
  orgResidual = m_colorTransResiBuf.getBuf(localUnitArea);
  orgResidual.copyFrom(cs.getResiBuf());

  const TempCtx ctxStart(m_ctxPool, m_CABACEstimator->getCtx());
  int           numAllowedColorSpace = 1;
  Distortion    zeroDistortion       = 0;

  double  bestCost    = MAX_DOUBLE;
  bool    bestRootCbf = false;
  uint8_t bestsbtInfo = 0;
  uint8_t orgSbtInfo  = cu.sbtInfo;
  int     bestIter    = 0;

  for (int iter = 0; iter < numAllowedColorSpace; iter++)
  {
    cu.sbtInfo = orgSbtInfo;

    m_CABACEstimator->resetBits();
    m_CABACEstimator->getCtx() = ctxStart;
    cs.clearTUs();
    cs.fracBits = 0;
    cs.dist     = 0;
    cs.cost     = 0;

    zeroDistortion = 0;
    if (luma)
    {
      cs.getOrgResiBuf().bufs[0].copyFrom(orgResidual.bufs[0]);
    }
    if (chroma && isChromaEnabled(cs.pcv->chrFormat))
    {
      cs.getOrgResiBuf().bufs[1].copyFrom(orgResidual.bufs[1]);
      cs.getOrgResiBuf().bufs[2].copyFrom(orgResidual.bufs[2]);
    }
    xEstimateInterResidualQT(cs, partitioner, &zeroDistortion, luma, chroma);
    TransformUnit &firstTU = *cs.getTU(partitioner.chType);

    cu.rootCbf = false;
    m_CABACEstimator->resetBits();
    m_CABACEstimator->rqt_root_cbf(cu);
    const uint64_t zeroFracBits = m_CABACEstimator->getEstFracBits();
    double         zeroCost;
    {
#if WCG_EXT
      if (m_encCfg->m_lumaLevelToDeltaQPMapping.isEnabled())
      {
        zeroCost = m_pcRdCost->calcRdCost(zeroFracBits, zeroDistortion, false);
      }
      else
#endif
      {
        zeroCost = m_pcRdCost->calcRdCost(zeroFracBits, zeroDistortion);
      }
    }

    const int numValidTBlocks = ::getNumberValidTBlocks(*cs.pcv);
    for (uint32_t i = 0; i < numValidTBlocks; i++)
    {
      cu.rootCbf |= TU::getCbfAtDepth(firstTU, CompID(i), 0);
    }

    // -------------------------------------------------------
    // If a block full of 0's is efficient, then just use 0's.
    // The costs at this point do not include header bits.

    if (zeroCost < cs.cost || !cu.rootCbf)
    {
      cs.cost    = zeroCost;
      cu.sbtInfo = 0;
      cu.rootCbf = false;

      cs.clearTUs();

      // add new "empty" TU(s) spanning the whole CU
      cs.addEmptyTUs(partitioner);
    }
    if (cs.cost < bestCost)
    {
      bestIter = iter;

      if (iter != (numAllowedColorSpace - 1))
      {
        bestCost    = cs.cost;
        bestRootCbf = cu.rootCbf;
        bestsbtInfo = cu.sbtInfo;

        saveCS.clearTUs();
        for (const auto &ptu: cs.tus)
        {
          TransformUnit &tu = saveCS.addTU(*ptu, ptu->chType);
          tu                = *ptu;
        }
        saveCS.getResiBuf(curUnitArea).copyFrom(cs.getResiBuf(curUnitArea));
      }
    }
  }

  if (bestIter != (numAllowedColorSpace - 1))
  {
    cu.rootCbf = bestRootCbf;
    cu.sbtInfo = bestsbtInfo;

    cs.clearTUs();
    for (const auto &ptu: saveCS.tus)
    {
      TransformUnit &tu = cs.addTU(*ptu, ptu->chType);
      tu                = *ptu;
    }
    cs.getResiBuf(curUnitArea).copyFrom(saveCS.getResiBuf(curUnitArea));
  }

  // all decisions now made. Fully encode the CU, including the headers:
  m_CABACEstimator->getCtx() = ctxStart;

  uint64_t finalFracBits = xGetSymbolFracBitsInter(cs, partitioner);
  // we've now encoded the CU, and so have a valid bit cost
  if (!cu.rootCbf)
  {
    if (luma)
    {
      cs.getResiBuf().bufs[0].fill(0);   // Clear the residual image, if we didn't code it.
      bool   signalIsNotReshaped = m_pcReshape->m_ctuFlag && !cu.ciipFlag && !cu.gpmIntraFlag && !CU::isIBC(cu);
      ClpRng clpRngTmp           = cs.slice->setNewClipRange(!signalIsNotReshaped, &m_pcReshape->m_fwdLUT, COMP_Y);
      cs.getRecoBuf().bufs[0].copyClip(cs.getPredBuf().bufs[0], clpRngTmp);
      if (cs.slice->m_lmcsEnabledFlag && signalIsNotReshaped)
      {
        cs.getRecoBuf().bufs[0].rspSignal(m_pcReshape->m_fwdLUT);
      }
      clpRngTmp = cs.slice->setNewClipRange(true, &m_pcReshape->m_fwdLUT, COMP_Y);
      cs.getRecoBuf().bufs[0].copyClip(cs.getRecoBuf().bufs[0], clpRngTmp);
    }
    if (chroma && isChromaEnabled(cs.pcv->chrFormat))
    {
      cs.getResiBuf().bufs[1].fill(0);   // Clear the residual image, if we didn't code it.
      cs.getResiBuf().bufs[2].fill(0);   // Clear the residual image, if we didn't code it.
      cs.getRecoBuf().bufs[1].copyClip(cs.getPredBuf().bufs[1], cs.slice->m_clpRngs.comp[1]);
      cs.getRecoBuf().bufs[2].copyClip(cs.getPredBuf().bufs[2], cs.slice->m_clpRngs.comp[2]);
    }
  }

  // update with clipped distortion and cost (previously unclipped reconstruction values were used)
  Distortion finalDistortion = 0;
  for (int comp = 0; comp < numValidComponents; comp++)
  {
    const CompID compID = CompID(comp);
    if (compID == COMP_Y && !luma)
    {
      continue;
    }
    if (compID != COMP_Y && !chroma)
    {
      continue;
    }
    CPelBuf         reco  = cs.getRecoBuf(compID);
    CPelBuf         org   = cs.getOrgBuf(compID);
    const CompArea &areaY = cu.Y();
    CompArea        tmpArea1(COMP_Y, areaY.chromaFormat, Position(0, 0), areaY.size());
    PelBuf          tmpRecLuma;
    if (isLuma(compID))
    {
      tmpRecLuma = m_tmpStorageCtu.getBuf(tmpArea1);
      tmpRecLuma.copyFrom(reco);
      if (m_encCfg->m_lmcsEnabled && (cs.slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag) &&
          !(m_encCfg->m_lumaLevelToDeltaQPMapping.isEnabled()))
      {
        tmpRecLuma.rspSignal(m_pcReshape->m_invLUT);
      }
      if (cs.pps->m_BIF && isLuma(compID))
      {
        for (auto &currTU: CU::traverseTUs(cu))
        {
          Position tuPosInCu = currTU.lumaPos() - cu.lumaPos();
          PelBuf   tmpSubBuf = tmpRecLuma.subBuf(tuPosInCu, currTU.lumaSize());

          if (m_bilateralFilter->getApplyBIF(currTU, compID))
          {
            CompArea compArea    = currTU.blocks[compID];
            PelBuf   recIPredBuf = cs.slice->m_pic->getRecoBuf(compArea);
            // Only reshape surrounding samples if reshaping is on
            if (m_encCfg->m_lmcsEnabled && cs.slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag &&
                !m_encCfg->m_lumaLevelToDeltaQPMapping.isEnabled())
            {
              m_bilateralFilter->bilateralFilterRDOdiamond5x5(compID, tmpSubBuf, tmpSubBuf, tmpSubBuf, currTU.cu->qp,
                                                              recIPredBuf, reco, cs.slice->clpRng(compID), currTU, true,
                                                              true, &m_pcReshape->m_invLUT);
            }
            else
            {
              m_bilateralFilter->bilateralFilterRDOdiamond5x5(compID, tmpSubBuf, tmpSubBuf, tmpSubBuf, currTU.cu->qp,
                                                              recIPredBuf, reco, cs.slice->clpRng(compID), currTU,
                                                              true);
            }
          }
        }
      }
    }
    PelBuf tmpRecChroma;
    if (isChroma(compID))
    {
      const CompArea &area = cu.blocks[compID];
      CompArea        tmpArea2(compID, area.chromaFormat, Position(0, 0), area.size());
      tmpRecChroma = m_tmpStorageCtu.getBuf(tmpArea2);
      tmpRecChroma.copyFrom(reco);

      if (cs.pps->m_chromaBIF && isChroma(compID))
      {
        for (auto &currTU: CU::traverseTUs(cu))
        {
          Position tuPosInCu = currTU.chromaPos() - cu.chromaPos();
          PelBuf   tmpSubBuf = tmpRecChroma.subBuf(tuPosInCu, currTU.chromaSize());
          if (m_bilateralFilter->getApplyBIF(currTU, compID))
          {
            CompArea compArea    = currTU.blocks[compID];
            PelBuf   recIPredBuf = cs.slice->m_pic->getRecoBuf(compArea);
            m_bilateralFilter->bilateralFilterRDOdiamond5x5(compID, tmpSubBuf, tmpSubBuf, tmpSubBuf, currTU.cu->qp,
                                                            recIPredBuf, reco, cs.slice->clpRng(compID), currTU, true);
          }
        }
      }
    }
#if WCG_EXT
    if (m_encCfg->m_lumaLevelToDeltaQPMapping.isEnabled() ||
        (m_encCfg->m_lmcsEnabled && (cs.slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag)))
    {
      const CPelBuf orgLuma = cs.getOrgBuf(cs.area.blocks[COMP_Y]);
      if (compID == COMP_Y)
      {
        finalDistortion += m_pcRdCost->getDistPart(org, tmpRecLuma, sps.m_bitDepths[toChannelType(compID)], compID,
                                                   DFunc::SSE_WTD, &orgLuma);
      }
      else
      {
        finalDistortion += m_pcRdCost->getDistPart(org, tmpRecChroma, sps.m_bitDepths[toChannelType(compID)], compID,
                                                   DFunc::SSE_WTD, &orgLuma);
      }
    }
    else
#endif
    {
      if (compID == COMP_Y)
      {
        finalDistortion +=
          m_pcRdCost->getDistPart(org, tmpRecLuma, sps.m_bitDepths[toChannelType(compID)], compID, DFunc::SSE);
      }
      else
      {
        finalDistortion +=
          m_pcRdCost->getDistPart(org, tmpRecChroma, sps.m_bitDepths[toChannelType(compID)], compID, DFunc::SSE);
      }
    }
  }

  cs.dist     = finalDistortion;
  cs.fracBits = finalFracBits;
  cs.cost     = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);
  CHECK(cs.tus.size() == 0, "No TUs present");
}

uint64_t InterSearch::xGetSymbolFracBitsInter(CodingStructure &cs, Partitioner &partitioner)
{
  uint64_t    fracBits = 0;
  CodingUnit &cu       = *cs.getCU(partitioner.chType);

  m_CABACEstimator->resetBits();

  if (cu.mergeFlag && !cu.rootCbf)
  {
    cu.skip = true;
    m_CABACEstimator->cu_skip_flag(cu);
    if (cu.ciipFlag)
    {
      // CIIP shouldn't be skip, the upper level function will deal with it, i.e. setting the overall cost to MAX_DOUBLE
    }
    else
    {
      m_CABACEstimator->merge_data(cu);
    }
    fracBits += m_CABACEstimator->getEstFracBits();
  }
  else
  {
    CHECK(cu.skip, "Skip flag has to be off at this point!");

    if (cu.Y().valid())
    {
      m_CABACEstimator->cu_skip_flag(cu);
    }
    m_CABACEstimator->pred_mode(cu);
    m_CABACEstimator->cu_pred_data(cu);
    CUCtx cuCtx;
    cuCtx.isDQPCoded         = true;
    cuCtx.isChromaQpAdjCoded = true;
    m_CABACEstimator->cu_residual(cu, partitioner, cuCtx);
    fracBits += m_CABACEstimator->getEstFracBits();
  }

  return fracBits;
}

double InterSearch::xGetMEDistortionWeight(uint8_t bcwIdx, RefPicList eRefPicList)
{
  if (bcwIdx != BCW_DEFAULT)
  {
    return (double)abs(getBcwWeight(bcwIdx, eRefPicList)) / BCW_WEIGHT_BASE;
  }
  else
  {
    return 0.5;
  }
}

bool InterSearch::xReadBufferedUniMv(CodingUnit &cu, RefPicList eRefPicList, int32_t refIdx, Mv &pcMvPred, Mv &rcMv,
                                     uint32_t &ruiBits, Distortion &ruiCost)
{
  if (m_uniMotions.isReadMode((uint32_t)eRefPicList, (uint32_t)refIdx))
  {
    m_uniMotions.copyTo(rcMv, ruiCost, (uint32_t)eRefPicList, (uint32_t)refIdx);

    Mv pred = pcMvPred;
    pred.changeTransPrecInternal2Amvr(cu.imv);
    m_pcRdCost->setPredictor(pred);
    m_pcRdCost->setCostScale(0);

    Mv mv = rcMv;
    mv.changeTransPrecInternal2Amvr(cu.imv);
    uint32_t mvBits = m_pcRdCost->getBitsOfVectorWithPredictor(mv.getHor(), mv.getVer(), 0);

    ruiBits += mvBits;
    ruiCost += m_pcRdCost->getCost(ruiBits);
    return true;
  }
  return false;
}

bool InterSearch::xReadBufferedAffineUniMv(CodingUnit &cu, RefPicList eRefPicList, int32_t refIdx, Mv acMvPred[3],
                                           Mv acMv[3], uint32_t &ruiBits, Distortion &ruiCost, int &mvpIdx,
                                           const AffineAMVPInfo &aamvpi)
{
  if (m_uniMotions.isReadModeAffine((uint32_t)eRefPicList, (uint32_t)refIdx, cu.affineType))
  {
    m_uniMotions.copyAffineMvTo(acMv, ruiCost, (uint32_t)eRefPicList, (uint32_t)refIdx, cu.affineType, mvpIdx);
    m_pcRdCost->setCostScale(0);
    acMvPred[0] = aamvpi.mvCandLT[mvpIdx];
    acMvPred[1] = aamvpi.mvCandRT[mvpIdx];
    acMvPred[2] = aamvpi.mvCandLB[mvpIdx];

    uint32_t mvBits = 0;
    for (int verIdx = 0; verIdx < cu.getNumAffineMvs(); verIdx++)
    {
      Mv pred = verIdx ? acMvPred[verIdx] + acMv[0] - acMvPred[0] : acMvPred[verIdx];
      pred.changePrecision(MvPrecision::INTERNAL, MvPrecision::QUARTER);
      m_pcRdCost->setPredictor(pred);
      Mv mv = acMv[verIdx];
      mv.changePrecision(MvPrecision::INTERNAL, MvPrecision::QUARTER);
      mvBits += m_pcRdCost->getBitsOfVectorWithPredictor(mv.getHor(), mv.getVer(), 0);
    }
    ruiBits += mvBits;
    ruiCost += m_pcRdCost->getCost(ruiBits);
    return true;
  }
  return false;
}

void InterSearch::initWeightIdxBits()
{
  for (int n = 0; n < BCW_NUM; ++n)
  {
    m_estWeightIdxBits[n] = deriveWeightIdxBits(n);
  }
}

void InterSearch::xClipMv(Mv &rcMv, const Position &pos, const Size &size, const SPS &sps, const PPS &pps)
{
  int mvShift = MV_FRACTIONAL_BITS_INTERNAL;
  int offset  = 8;

  int horMax = (pps.m_picWidthInLumaSamples + offset - (int)pos.x - 1) << mvShift;
  int horMin = (-(int)sps.m_maxCuWidth - offset - (int)pos.x + 1) * (1 << mvShift);

  int verMax = (pps.m_picHeightInLumaSamples + offset - (int)pos.y - 1) << mvShift;
  int verMin = (-(int)sps.m_maxCuHeight - offset - (int)pos.y + 1) * (1 << mvShift);

  const SubPic &curSubPic = pps.getSubPicFromPos(pos);
  if (curSubPic.m_treatedAsPicFlag && m_clipMvInSubPic)
  {
    horMax = ((curSubPic.m_subPicRight + 1) + offset - (int)pos.x - 1) << mvShift;
    horMin = (-(int)sps.m_maxCuWidth - offset - ((int)pos.x - curSubPic.m_subPicLeft) + 1) * (1 << mvShift);

    verMax = ((curSubPic.m_subPicBottom + 1) + offset - (int)pos.y - 1) << mvShift;
    verMin = (-(int)sps.m_maxCuHeight - offset - ((int)pos.y - curSubPic.m_subPicTop) + 1) * (1 << mvShift);
  }
  if (pps.m_wrapAroundEnabledFlag)
  {
    int horMax = (pps.m_picWidthInLumaSamples + sps.m_maxCuWidth - size.width + offset - (int)pos.x - 1) << mvShift;
    int horMin = (-(int)sps.m_maxCuWidth - offset - (int)pos.x + 1) * (1 << mvShift);
    rcMv.setHor(std::min(horMax, std::max(horMin, rcMv.getHor())));
    rcMv.setVer(std::min(verMax, std::max(verMin, rcMv.getVer())));
    return;
  }

  rcMv.setHor(std::min(horMax, std::max(horMin, rcMv.getHor())));
  rcMv.setVer(std::min(verMax, std::max(verMin, rcMv.getVer())));
}

uint32_t InterSearch::xDetermineBestMvp(CodingUnit &cu, Mv acMvTemp[3], int &mvpIdx, const AffineAMVPInfo &aamvpi)
{
  bool     mvpUpdated = false;
  uint32_t minBits    = std::numeric_limits<uint32_t>::max();

  for (int i = 0; i < aamvpi.numCand; i++)
  {
    Mv       mvPred[3] = { aamvpi.mvCandLT[i], aamvpi.mvCandRT[i], aamvpi.mvCandLB[i] };
    uint32_t candBits  = m_auiMVPIdxCost[i][aamvpi.numCand];
    candBits += xCalcAffineMVBits(cu, acMvTemp, mvPred);

    if (candBits < minBits)
    {
      minBits    = candBits;
      mvpIdx     = i;
      mvpUpdated = true;
    }
  }

  CHECK(!mvpUpdated, "xDetermineBestMvp() error");

  return minBits;
}

void InterSearch::symmvdCheckBestMvp(CodingUnit &cu, PelUnitBuf &origBuf, Mv curMv, RefPicList curRefList,
                                     RefSetArray<AMVPInfo> &amvpInfo, int32_t bcwIdx, Mv cMvPredSym[NUM_RPL01],
                                     int32_t mvpIdxSym[NUM_RPL01], Distortion &bestCost, bool skip)
{

  RefPicList tarRefList = (RefPicList)(1 - curRefList);
  int32_t    refIdxCur  = cu.slice->getSymRefIdx(curRefList);
  int32_t    refIdxTar  = cu.slice->getSymRefIdx(tarRefList);

  MvField cCurMvField, cTarMvField;
  cCurMvField.setMvField(curMv, refIdxCur);
  AMVPInfo &amvpCur = amvpInfo[curRefList][refIdxCur];
  AMVPInfo &amvpTar = amvpInfo[tarRefList][refIdxTar];
  m_pcRdCost->setCostScale(0);

  // get prediction of eCurRefPicList
  PelUnitBuf     predBufA = m_tmpPredStorage[curRefList].getBuf(UnitAreaRelative(cu, cu));
  const Picture *picRefA  = cu.slice->getRefPic(curRefList, cCurMvField.refIdx);
  Mv             mvA      = cCurMvField.mv;
  clipMv(mvA, cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);
  if ((mvA.hor & 15) == 0 && (mvA.ver & 15) == 0 && !cu.licFlag)
  {
    Position offset  = cu.blocks[COMP_Y].pos().offset(mvA.getHor() >> 4, mvA.getVer() >> 4);
    CPelBuf  pelBufA = picRefA->getRecoBuf(CompArea(COMP_Y, cu.chromaFormat, offset, cu.blocks[COMP_Y].size()), false);
    predBufA.bufs[0].buf    = const_cast<Pel *>(pelBufA.buf);
    predBufA.bufs[0].stride = pelBufA.stride;
  }
  else
  {
    xPredInterBlk(COMP_Y, cu, picRefA, mvA, predBufA, false, cu.slice->clpRng(COMP_Y), false, false, curRefList);
  }
  PelUnitBuf bufTmp = m_tmpStorageCtu.getBuf(UnitAreaRelative(cu, cu));
  bufTmp.copyFrom(origBuf);
  bufTmp.removeHighFreq(predBufA, m_encCfg->m_bClipForBiPredMeEnabled, cu.slice->m_clpRngs,
                        getBcwWeight(cu.bcwIdx, tarRefList));

  double fWeight = xGetMEDistortionWeight(cu.bcwIdx, tarRefList);

  int32_t skipMvpIdx[2];
  skipMvpIdx[0] = skip ? mvpIdxSym[0] : -1;
  skipMvpIdx[1] = skip ? mvpIdxSym[1] : -1;

  for (int i = 0; i < amvpCur.numCand; i++)
  {
    for (int j = 0; j < amvpTar.numCand; j++)
    {
      if (skipMvpIdx[curRefList] == i && skipMvpIdx[tarRefList] == j)
      {
        continue;
      }

      cTarMvField.setMvField(curMv.getSymmvdMv(amvpCur.mvCand[i], amvpTar.mvCand[j]), refIdxTar);

      // get prediction of eTarRefPicList
      PelUnitBuf     predBufB = m_tmpPredStorage[tarRefList].getBuf(UnitAreaRelative(cu, cu));
      const Picture *picRefB  = cu.slice->getRefPic(tarRefList, cTarMvField.refIdx);
      Mv             mvB      = cTarMvField.mv;
      clipMv(mvB, cu.lumaPos(), cu.lumaSize(), *cu.cs->sps, *cu.cs->pps);
      if ((mvB.hor & 15) == 0 && (mvB.ver & 15) == 0 && !cu.licFlag)
      {
        Position offset = cu.blocks[COMP_Y].pos().offset(mvB.getHor() >> 4, mvB.getVer() >> 4);
        CPelBuf  pelBufB =
          picRefB->getRecoBuf(CompArea(COMP_Y, cu.chromaFormat, offset, cu.blocks[COMP_Y].size()), false);
        predBufB.bufs[0].buf    = const_cast<Pel *>(pelBufB.buf);
        predBufB.bufs[0].stride = pelBufB.stride;
      }
      else
      {
        xPredInterBlk(COMP_Y, cu, picRefB, mvB, predBufB, false, cu.slice->clpRng(COMP_Y), false, false, tarRefList);
      }
      // calc distortion
      const DFunc distFunc = (!cu.slice->m_disableSATDForRd) ? DFunc::HAD : DFunc::SAD;
      Distortion  cost     = (Distortion)floor(
        fWeight *
        (double)m_pcRdCost->getDistPart(bufTmp.Y(), predBufB.Y(), cu.cs->sps->m_bitDepths[ChannelType::LUMA], COMP_Y,
                                             distFunc, nullptr, cu.cs->sps->m_bitDepths[ChannelType::LUMA] + 4));

      Mv pred = amvpCur.mvCand[i];
      pred.changeTransPrecInternal2Amvr(cu.imv);
      m_pcRdCost->setPredictor(pred);
      Mv mv = curMv;
      mv.changeTransPrecInternal2Amvr(cu.imv);
      uint32_t bits = m_pcRdCost->getBitsOfVectorWithPredictor(mv.hor, mv.ver, 0);
      bits += m_auiMVPIdxCost[i][AMVP_MAX_NUM_CANDS];
      bits += m_auiMVPIdxCost[j][AMVP_MAX_NUM_CANDS];
      cost += m_pcRdCost->getCost(bits);

      if (cost < bestCost)
      {
        bestCost               = cost;
        cMvPredSym[curRefList] = amvpCur.mvCand[i];
        cMvPredSym[tarRefList] = amvpTar.mvCand[j];
        mvpIdxSym[curRefList]  = i;
        mvpIdxSym[tarRefList]  = j;
      }
    }
  }
}

uint64_t InterSearch::calcPuMeBits(CodingUnit &cu)
{
  assert(cu.mergeFlag);
  assert(!CU::isIBC(cu));
  m_CABACEstimator->resetBits();
  m_CABACEstimator->merge_flag(cu);
  if (cu.mergeFlag)
  {
    m_CABACEstimator->merge_data(cu);
  }
  return m_CABACEstimator->getEstFracBits();
}

//! \}
