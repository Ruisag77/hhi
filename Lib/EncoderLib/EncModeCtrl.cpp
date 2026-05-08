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

/** \file     EncModeCtrl.cpp
    \brief    Encoder controller for trying out specific modes
*/

#include "EncModeCtrl.h"

#include "AQp.h"
#include "RateCtrl.h"

#include "CommonLib/RdCost.h"
#include "CommonLib/CodingStructure.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"

#include "CommonLib/dtrace_next.h"

#include <cmath>

static constexpr double UNSET_IMV_COST = MAX_DOUBLE * 0.125;   // Some large, unique value

static bool isValidPosAndSize(int idPos, int idSize)
{
  const int size = gp_sizeIdxInfo->sizeFrom(idSize);
  if (!gp_sizeIdxInfo->isCuSize(size) || !gp_sizeIdxInfo->hasSizeAtOffset(idPos << MIN_CU_LOG2, size))
  {
    return false;
  }
  return true;
}

void EncModeCtrl::init(const EncCfg *encCfg, RateCtrl *pRateCtrl, RdCost *pRdCost, std::map<int, double *> *adaptQPmap)
{
  m_encCfg     = encCfg;
  m_pcRateCtrl = pRateCtrl;
  m_pcRdCost   = pRdCost;
  fastDeltaQp  = false;
  m_adaptQPmap = adaptQPmap;
#if SHARP_LUMA_DELTA_QP
  m_lumaQPOffset = 0;

  initLumaDeltaQpLUT();
#endif
}

void EncModeCtrl::setEarlySkipDetected() { comprCUCtx->earlySkip = true; }

void EncModeCtrl::getMinMaxQP(int &minQP, int &maxQP, double &deltaQPForLambda, const CodingStructure &cs,
                              const Partitioner &partitioner, const int baseQP, const PartSplit splitMode)
{
  if (m_encCfg->m_RCEnableRateControl)
  {
    minQP = m_pcRateCtrl->getRCQP();
    maxQP = m_pcRateCtrl->getRCQP();
    return;
  }

  const unsigned subdivIncr   = (splitMode == CU_QUAD_SPLIT) ? 2 : (splitMode == CU_BT_SPLIT) ? 1 : 0;
  const bool     qgEnable     = partitioner.currQgEnable(); // QG possible at current level
  const bool qgEnableChildren = qgEnable && ((partitioner.currSubdiv + subdivIncr) <= cs.slice->getCuQpDeltaSubdiv()) &&
    (subdivIncr > 0); // QG possible at next level
  const bool isLeafQG = (qgEnable && !qgEnableChildren);

  if (isLeafQG) // QG at deepest level
  {
    const SPS &sps     = *cs.sps;
    int        deltaQP = m_encCfg->m_iMaxDeltaQP;
    minQP              = Clip3(-sps.m_qpBDOffset[ChannelType::LUMA], MAX_QP, baseQP - deltaQP);
    maxQP              = Clip3(-sps.m_qpBDOffset[ChannelType::LUMA], MAX_QP, baseQP + deltaQP);
    Position  pos      = partitioner.currQgPos;
    const int ctuSize  = sps.m_ctuSize;

    // BIM offset derived on CTU basis in full resolution
    // to get correct CTU in full resolution the position in the low resolution need to be scaled
    double scaleWidth  = (double)sps.m_maxWidthInLumaSamples / (double)cs.picture->lwidth();
    double scaleHeight = (double)sps.m_maxHeightInLumaSamples / (double)cs.picture->lheight();
    int    blockId;
    int    thePosY = (int)((double)pos.y * scaleHeight + 0.5);
    int    thePosX = (int)((double)pos.x * scaleWidth + 0.5);
    blockId        = (thePosY / m_encCfg->m_bimUnitSize) *
        ((sps.m_maxWidthInLumaSamples + m_encCfg->m_bimUnitSize - 1) / m_encCfg->m_bimUnitSize) +
      (thePosX / m_encCfg->m_bimUnitSize);
    const double *bimPtr = m_adaptQPmap->count(cs.slice->m_poc) ? m_adaptQPmap->at(cs.slice->m_poc) : nullptr;
    if (m_encCfg->m_bimEnabled > 0)
    {
      double avgBimOffset = 0.0;
      double scaleFactor  = (scaleWidth + scaleHeight) / 2;
      // now we need to derive an average BIM offset in the full resolution that corresponds to a scaled size
      int    maxY         = ctuSize * scaleHeight / m_encCfg->m_bimUnitSize;
      int    maxX         = ctuSize * scaleWidth / m_encCfg->m_bimUnitSize;
      // make sure that the BIM block is inside the picture width and height
      if ((thePosY + maxY * m_encCfg->m_bimUnitSize) > sps.m_maxHeightInLumaSamples)
      {
        int reduceHeight = (thePosY + (maxY - 1) * m_encCfg->m_bimUnitSize) - sps.m_maxHeightInLumaSamples;
        int reduceY      = (reduceHeight + m_encCfg->m_bimUnitSize) / m_encCfg->m_bimUnitSize;
        maxY             = maxY - reduceY;
      }
      if ((thePosX + maxX * m_encCfg->m_bimUnitSize) > sps.m_maxWidthInLumaSamples)
      {
        int reduceWidth = (thePosX + (maxX - 1) * m_encCfg->m_bimUnitSize) - sps.m_maxWidthInLumaSamples;
        int reduceX     = (reduceWidth + m_encCfg->m_bimUnitSize) / m_encCfg->m_bimUnitSize;
        maxX            = maxX - reduceX;
      }
      for (int y = 0; y < maxY * m_encCfg->m_bimUnitSize; y = y + m_encCfg->m_bimUnitSize)
      {
        for (int x = 0; x < maxX * m_encCfg->m_bimUnitSize; x = x + m_encCfg->m_bimUnitSize)
        {
          int theBlockId = blockId +
            (y / m_encCfg->m_bimUnitSize) *
              ((sps.m_maxWidthInLumaSamples + m_encCfg->m_bimUnitSize - 1) / m_encCfg->m_bimUnitSize) +
            (x / m_encCfg->m_bimUnitSize);
          int theBimOffset = bimPtr ? bimPtr[theBlockId] : 0;
          avgBimOffset += theBimOffset;
        }
      }
      if (maxY * maxX > 0)
      {
        avgBimOffset = avgBimOffset / (double)(maxY * maxX);
      }
      else
      {
        avgBimOffset = 0;
      }
      avgBimOffset = avgBimOffset / scaleFactor;

      if (m_encCfg->m_bimEnabled == 1)
      {
        int intAvgBimOffset = (int)(0.5 + avgBimOffset);
        deltaQPForLambda    = avgBimOffset - (double)intAvgBimOffset;
        minQP += intAvgBimOffset;
        maxQP += intAvgBimOffset;
      }
      else if (m_encCfg->m_bimEnabled == 2)
      {
        deltaQPForLambda = avgBimOffset;
      }
    }
  }
  else if (qgEnableChildren) // more splits and not the deepest QG level
  {
    minQP = baseQP;
    maxQP = baseQP;
  }
  else // deeper than QG
  {
    minQP = cs.currQP[partitioner.chType];
    maxQP = minQP;
  }
}

int EncModeCtrl::xComputeDQP(const CodingStructure &cs, const Partitioner &partitioner)
{
  const Picture *picture = cs.picture;

  const unsigned  aqDepth = std::min(partitioner.currSubdiv / 2, (uint32_t)picture->m_aqlayer.size() - 1);
  const AQpLayer *aqLayer = picture->m_aqlayer[aqDepth];

  const double maxQpScale   = pow(2.0, m_encCfg->m_iQPAdaptationRange / 6.0);
  const double avgActivity  = aqLayer->getAvgActivity();
  const double cuActivity   = aqLayer->getActivity(cs.area.Y().topLeft());
  const double normActivity = (maxQpScale * cuActivity + avgActivity) / (cuActivity + maxQpScale * avgActivity);
  const double qpOffset     = std::log2(normActivity) * 6.0;

  return int(floor(qpOffset + 0.49999));
}

#if SHARP_LUMA_DELTA_QP
void EncModeCtrl::initLumaDeltaQpLUT()
{
  const LumaLevelToDeltaQPMapping &mapping = m_encCfg->m_lumaLevelToDeltaQPMapping;

  if (!mapping.isEnabled())
  {
    return;
  }

  // map the sparse LumaLevelToDeltaQPMapping.mapping to a fully populated linear table.

  int         lastDeltaQPValue = 0;
  std::size_t nextSparseIndex  = 0;
  for (int index = 0; index < LUMA_LEVEL_TO_DQP_LUT_MAXSIZE; index++)
  {
    while (nextSparseIndex < mapping.mapping.size() && index >= mapping.mapping[nextSparseIndex].first)
    {
      lastDeltaQPValue = mapping.mapping[nextSparseIndex].second;
      nextSparseIndex++;
    }
    m_lumaLevelToDeltaQPLUT[index] = lastDeltaQPValue;
  }
}

int EncModeCtrl::calculateLumaDQP(const CPelBuf &rcOrg)
{
  double avg = 0;

  // Get QP offset derived from Luma level
#if !WCG_EXT
  if (m_encCfg->m_lumaLevelToDeltaQPMapping.mode == LUMALVL_TO_DQP_AVG_METHOD)
#else
  CHECK(m_encCfg->m_lumaLevelToDeltaQPMapping.mode != LUMALVL_TO_DQP_AVG_METHOD, "invalid delta qp mode");
#endif
  {
    // Use average luma value
    avg = (double)rcOrg.computeAvg();
  }
#if !WCG_EXT
  else
  {
    // Use maximum luma value
    int maxVal = 0;
    for (uint32_t y = 0; y < rcOrg.height; y++)
    {
      for (uint32_t x = 0; x < rcOrg.width; x++)
      {
        const Pel &v = rcOrg.at(x, y);
        if (v > maxVal)
        {
          maxVal = v;
        }
      }
    }
    // use a percentage of the maxVal
    avg = (double)maxVal * m_encCfg->m_lumaLevelToDeltaQPMapping.maxMethodWeight;
  }
#endif
  int lumaBD     = m_encCfg->m_internalBitDepth[ChannelType::LUMA];
  int lumaIdxOrg = Clip3<int>(0, int(1 << lumaBD) - 1, int(avg + 0.5));
  int lumaIdx    = lumaBD < 10 ? lumaIdxOrg << (10 - lumaBD) : lumaBD > 10 ? lumaIdxOrg >> (lumaBD - 10) : lumaIdxOrg;
  int QP         = m_lumaLevelToDeltaQPLUT[lumaIdx];
  return QP;
}
#endif

int EncModeCtrl::calculateLumaDQPsmooth(const CPelBuf &rcOrg, int baseQP, double threshold, double scale, double offset,
                                        int limit)
{
  double avg  = 0;
  double diff = 0;
  double thr  = (double)threshold * rcOrg.height * rcOrg.width;
  int    qp   = 0;
  CHECK(rcOrg.height > 128 || rcOrg.width > 128, "Method not applicable to CTU256");
  if (rcOrg.height >= 64 && rcOrg.width >= 64)
  {
    const int numBasis = 6;

    // clang-format off
    double invb[numBasis][numBasis] = { {0.001*0.244140625000000,                         0,                         0,                        0,                        0,                        0},
                                      {                      0,   0.001*0.013204564833946,   0.001*0.002080251479290, -0.001*0.000066039729501, -0.001*0.000165220364313,        0.000000000000000},
                                      {                      0,   0.001*0.002080251479290,   0.001*0.013204564833946, -0.001*0.000066039729501,        0.000000000000000, -0.001*0.000165220364313},
                                      {                      0,  -0.001*0.000066039729501,  -0.001*0.000066039729501,  0.001*0.000002096499349,        0.000000000000000,        0.000000000000000},
                                      {                      0,  -0.001*0.000165220364313,         0.000000000000000,        0.000000000000000,  0.001*0.000002622545465,        0.000000000000000},
                                      {                      0,         0.000000000000000,  -0.001*0.000165220364313,        0.000000000000000,        0.000000000000000,  0.001*0.000002622545465} };
    double boffset[5] = { -31.5, -31.5, -992.25, -1333.5, -1333.5 };
    // clang-format on

    int listQuadrantsX[4] = { 0, 64, 0, 64 };
    int listQuadrantsY[4] = { 0, 0, 64, 64 };

    double b1sum;
    double b2sum;
    double b3sum;
    double b4sum;
    double b5sum;
    double b6sum;
    int    numQuadrantsX = (rcOrg.width == 128) ? 2 : 1;
    int    numQuadrantsY = (rcOrg.height == 128) ? 2 : 1;
    // loop over quadrants
    for (int posy = 0; posy < numQuadrantsY; posy++)
    {
      for (int posx = 0; posx < numQuadrantsX; posx++)
      {
        b2sum = 0.0;
        b3sum = 0.0;
        b4sum = 0.0;
        b5sum = 0.0;
        b6sum = 0.0;
        avg   = 0.0;
        for (uint32_t y = 0; y < 64; y++)
        {
          for (uint32_t x = 0; x < 64; x++)
          {
            const Pel &v = rcOrg.at(x + listQuadrantsX[posx + 2 * posy], y + listQuadrantsY[posx + 2 * posy]);
            b2sum += ((double)v) * ((double)x + boffset[0]);
            b3sum += ((double)v) * ((double)y + boffset[1]);
            b4sum += ((double)v) * ((double)x * (double)y + boffset[2]);
            b5sum += ((double)v) * ((double)x * (double)x + boffset[3]);
            b6sum += ((double)v) * ((double)y * (double)y + boffset[4]);
            avg += (double)v;
          }
        }
        b1sum = avg;
        double r[numBasis];
        for (uint32_t b = 0; b < numBasis; b++)
        {
          r[b] = invb[b][0] * b1sum + invb[b][1] * b2sum + invb[b][2] * b3sum + invb[b][3] * b4sum +
            invb[b][4] * b5sum + invb[b][5] * b6sum;
        }
        // compute SAD for model
        for (uint32_t y = 0; y < 64; y++)
        {
          for (uint32_t x = 0; x < 64; x++)
          {
            const Pel &v = rcOrg.at(x + listQuadrantsX[posx + 2 * posy], y + listQuadrantsY[posx + 2 * posy]);

            diff +=
              abs((int)v -
                  (int)(r[0] + r[1] * ((double)x + boffset[0]) + r[2] * ((double)y + boffset[1]) +
                        r[3] * ((double)x * (double)y + boffset[2]) + r[4] * ((double)x * (double)x + boffset[3]) +
                        r[5] * ((double)y * (double)y + boffset[4])));
          }
        }
      }
    }
    if (diff < thr)
    {
      qp = std::max(limit, std::min(0, (int)(scale * (double)baseQP + offset)));
    }
  }
  return qp;
}

void CacheBlkInfoCtrl::create()
{
  const unsigned numPos = MAX_CU_SIZE_IN_PARTS;

  for (unsigned x = 0; x < numPos; x++)
  {
    for (unsigned y = 0; y < numPos; y++)
    {
      m_codedCUInfo[x][y] = new CodedCUInfo **[gp_sizeIdxInfo->numWidths()];

      for (int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++)
      {
        if (!isValidPosAndSize(x, wIdx))
        {
          m_codedCUInfo[x][y][wIdx] = nullptr;
          continue;
        }

        m_codedCUInfo[x][y][wIdx] = new CodedCUInfo *[gp_sizeIdxInfo->numHeights()];

        for (int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++)
        {
          if (!isValidPosAndSize(y, hIdx))
          {
            m_codedCUInfo[x][y][wIdx][hIdx] = nullptr;
            continue;
          }

          m_codedCUInfo[x][y][wIdx][hIdx] = new CodedCUInfo;
        }
      }
    }
  }
}

void CacheBlkInfoCtrl::destroy()
{
  const unsigned numPos = MAX_CU_SIZE_IN_PARTS;

  for (unsigned x = 0; x < numPos; x++)
  {
    for (unsigned y = 0; y < numPos; y++)
    {
      for (int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++)
      {
        if (m_codedCUInfo[x][y][wIdx])
        {
          for (int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++)
          {
            if (m_codedCUInfo[x][y][wIdx][hIdx])
            {
              delete m_codedCUInfo[x][y][wIdx][hIdx];
            }
          }
          delete[] m_codedCUInfo[x][y][wIdx];
        }
      }
      delete[] m_codedCUInfo[x][y];
    }
  }
}

void CacheBlkInfoCtrl::init(const Slice &slice)
{
  const unsigned numPos = MAX_CU_SIZE_IN_PARTS;

  for (unsigned x = 0; x < numPos; x++)
  {
    for (unsigned y = 0; y < numPos; y++)
    {
      for (int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++)
      {
        if (m_codedCUInfo[x][y][wIdx])
        {
          for (int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++)
          {
            if (m_codedCUInfo[x][y][wIdx][hIdx])
            {
              std::fill_n(reinterpret_cast<char *>(m_codedCUInfo[x][y][wIdx][hIdx]), sizeof(CodedCUInfo), 0);
            }
          }
        }
      }
    }
  }
  m_pcv = slice.m_pps->pcv;
}

const CodedCUInfo &CacheBlkInfoCtrl::getBlkInfo(const UnitArea &area) const
{
  unsigned idX, idY, idW, idH;
  getAreaIdx(area.Y(), *m_pcv, idX, idY, idW, idH);
  return *m_codedCUInfo[idX][idY][idW][idH];
}

CodedCUInfo &CacheBlkInfoCtrl::getCodedCUInfo(const UnitArea &area)
{
  unsigned idX, idY, idW, idH;
  getAreaIdx(area.Y(), *m_pcv, idX, idY, idW, idH);
  return *m_codedCUInfo[idX][idY][idW][idH];
}

void CacheBlkInfoCtrl::setMv(const UnitArea &area, const RefPicList refPicList, const int refIdx, const Mv &rMv)
{
  if (refIdx >= MAX_STORED_CU_INFO_REFS)
  {
    return;
  }

  unsigned idX, idY, idW, idH;
  getAreaIdx(area.Y(), *m_pcv, idX, idY, idW, idH);

  m_codedCUInfo[idX][idY][idW][idH]->saveMv[refPicList][refIdx]  = rMv;
  m_codedCUInfo[idX][idY][idW][idH]->validMv[refPicList][refIdx] = true;
}

bool CacheBlkInfoCtrl::getMv(const UnitArea &area, const RefPicList refPicList, const int refIdx, Mv &rMv) const
{
  unsigned idX, idY, idW, idH;
  getAreaIdx(area.Y(), *m_pcv, idX, idY, idW, idH);

  if (refIdx >= MAX_STORED_CU_INFO_REFS)
  {
    rMv = m_codedCUInfo[idX][idY][idW][idH]->saveMv[refPicList][0];
    return false;
  }

  rMv = m_codedCUInfo[idX][idY][idW][idH]->saveMv[refPicList][refIdx];
  return m_codedCUInfo[idX][idY][idW][idH]->validMv[refPicList][refIdx];
}

void SaveLoadEncInfoSbt::init(const Slice &slice) { m_pcv = slice.m_pps->pcv; }

void SaveLoadEncInfoSbt::create()
{
  const int numSizeIdx = gp_sizeIdxInfo->idxFrom(SBT_MAX_SIZE) - MIN_CU_LOG2 + 1;
  const int numPosIdx  = MAX_CU_SIZE_IN_PARTS;

  for (int xIdx = 0; xIdx < numPosIdx; xIdx++)
  {
    for (int yIdx = 0; yIdx < numPosIdx; yIdx++)
    {
      m_saveLoadSbt[xIdx][yIdx] = new SaveLoadStructSbt *[numSizeIdx];
      for (int wIdx = 0; wIdx < numSizeIdx; wIdx++)
      {
        m_saveLoadSbt[xIdx][yIdx][wIdx] = new SaveLoadStructSbt[numSizeIdx];
      }
    }
  }
}

void SaveLoadEncInfoSbt::destroy()
{
  const int numSizeIdx = gp_sizeIdxInfo->idxFrom(SBT_MAX_SIZE) - MIN_CU_LOG2 + 1;
  const int numPosIdx  = MAX_CU_SIZE_IN_PARTS;

  for (int xIdx = 0; xIdx < numPosIdx; xIdx++)
  {
    for (int yIdx = 0; yIdx < numPosIdx; yIdx++)
    {
      for (int wIdx = 0; wIdx < numSizeIdx; wIdx++)
      {
        delete[] m_saveLoadSbt[xIdx][yIdx][wIdx];
      }
      delete[] m_saveLoadSbt[xIdx][yIdx];
    }
  }
}

SaveLoadEncInfoSbt::BestSbt SaveLoadEncInfoSbt::findBestSbt(const UnitArea &area, const uint32_t curPuSse) const
{
  unsigned idX, idY, idW, idH;
  getAreaIdx(area.Y(), *m_pcv, idX, idY, idW, idH);
  const SaveLoadStructSbt &sbtSave = m_saveLoadSbt[idX][idY][idW - MIN_CU_LOG2][idH - MIN_CU_LOG2];

  for (int i = 0; i < sbtSave.numPuInfoStored; i++)
  {
    if (curPuSse == sbtSave.puSse[i])
    {
      return { sbtSave.puSbt[i], sbtSave.puTrs[i] };
    }
  }

  return { MAX_UCHAR, MtsType::NONE };
}

bool SaveLoadEncInfoSbt::saveBestSbt(const UnitArea &area, const uint32_t curPuSse, const uint8_t curPuSbt,
                                     const MtsType curPuTrs)
{
  unsigned idX, idY, idW, idH;
  getAreaIdx(area.Y(), *m_pcv, idX, idY, idW, idH);
  SaveLoadStructSbt &sbtSave = m_saveLoadSbt[idX][idY][idW - MIN_CU_LOG2][idH - MIN_CU_LOG2];

  if (sbtSave.numPuInfoStored == SBT_NUM_SL)
  {
    return false;
  }

  sbtSave.puSse[sbtSave.numPuInfoStored] = curPuSse;
  sbtSave.puSbt[sbtSave.numPuInfoStored] = curPuSbt;
  sbtSave.puTrs[sbtSave.numPuInfoStored] = curPuTrs;
  sbtSave.numPuInfoStored++;
  return true;
}

void SaveLoadEncInfoSbt::resetSaveloadSbt(int maxSbtSize)
{
  int numSizeIdx = gp_sizeIdxInfo->idxFrom(maxSbtSize) - MIN_CU_LOG2 + 1;
  int numPosIdx  = MAX_CU_SIZE >> MIN_CU_LOG2;

  for (int xIdx = 0; xIdx < numPosIdx; xIdx++)
  {
    for (int yIdx = 0; yIdx < numPosIdx; yIdx++)
    {
      for (int wIdx = 0; wIdx < numSizeIdx; wIdx++)
      {
        memset(m_saveLoadSbt[xIdx][yIdx][wIdx], 0, numSizeIdx * sizeof(SaveLoadStructSbt));
      }
    }
  }
}

#if REUSE_CU_RESULTS
static bool isTheSameNbHood(const CodingUnit &cu, const CodingStructure &cs, const Partitioner &partitioner, int picW,
                            int picH)
{
  if (cu.chType != partitioner.chType)
  {
    return false;
  }

  const PartitioningStack &ps = partitioner.getPartStack();

  int i = 1;

  for (; i < ps.size(); i++)
  {
    if (ps[i].split != CU::getSplitAtDepth(cu, i - 1))
    {
      break;
    }
  }

  const UnitArea &cmnAnc = ps[i - 1].parts[ps[i - 1].idx];
  const UnitArea  cuArea = CS::getArea(cs, cu, partitioner.chType);

  for (int i = 0; i < cmnAnc.blocks.size(); i++)
  {
    if (i < cuArea.blocks.size() && cuArea.blocks[i].valid() && cuArea.blocks[i].pos() != cmnAnc.blocks[i].pos())
    {
      return false;
    }
  }

  return true;
}

void BestEncInfoCache::create(const ChromaFormat chFmt)
{
  const int numPos = MAX_CU_SIZE_IN_PARTS;

  for (unsigned x = 0; x < numPos; x++)
  {
    for (unsigned y = 0; y < numPos; y++)
    {
      m_bestEncInfo[x][y] = new BestEncodingInfo **[gp_sizeIdxInfo->numWidths()];

      for (int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++)
      {
        if (!isValidPosAndSize(x, wIdx))
        {
          m_bestEncInfo[x][y][wIdx] = nullptr;
          continue;
        }

        m_bestEncInfo[x][y][wIdx] = new BestEncodingInfo *[gp_sizeIdxInfo->numHeights()];

        for (int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++)
        {
          if (!isValidPosAndSize(y, hIdx))
          {
            m_bestEncInfo[x][y][wIdx][hIdx] = nullptr;
            continue;
          }

          m_bestEncInfo[x][y][wIdx][hIdx] = new BestEncodingInfo;

          int w = gp_sizeIdxInfo->sizeFrom(wIdx);
          int h = gp_sizeIdxInfo->sizeFrom(hIdx);

          const UnitArea area(chFmt, Area(0, 0, w, h));

          new (&m_bestEncInfo[x][y][wIdx][hIdx]->cu) CodingUnit(area);
#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS
          m_bestEncInfo[x][y][wIdx][hIdx]->numTus = 0;
          for (int i = 0; i < REUSE_CU_RESULTS_MAX_NUM_TUS; i++)
          {
            new (&m_bestEncInfo[x][y][wIdx][hIdx]->tus[i]) TransformUnit(area);
          }
#else
          new (&m_bestEncInfo[x][y][wIdx][hIdx]->tu) TransformUnit(area);
#endif

          m_bestEncInfo[x][y][wIdx][hIdx]->poc      = -1;
          m_bestEncInfo[x][y][wIdx][hIdx]->testMode = EncTestMode();
        }
      }
    }
  }
}

void BestEncInfoCache::destroy()
{
  const unsigned numPos = MAX_CU_SIZE_IN_PARTS;

  for (unsigned x = 0; x < numPos; x++)
  {
    for (unsigned y = 0; y < numPos; y++)
    {
      for (int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++)
      {
        if (m_bestEncInfo[x][y][wIdx])
        {
          for (int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++)
          {
            delete m_bestEncInfo[x][y][wIdx][hIdx];
          }

          delete[] m_bestEncInfo[x][y][wIdx];
        }
      }

      delete[] m_bestEncInfo[x][y];
    }
  }

  delete[] m_pCoeff;
  delete[] m_pSignsPredArea;

  if (m_pltIdx != nullptr)
  {
    delete[] m_pltIdx;
    m_pltIdx = nullptr;
  }

  if (m_runType != nullptr)
  {
    delete[] m_runType;
    m_runType = nullptr;
  }
}

void BestEncInfoCache::init(const Slice &slice)
{
  bool isInitialized = nullptr != m_pcv;

  m_pcv                 = slice.m_pps->pcv;
  const unsigned numPos = MAX_CU_SIZE_IN_PARTS;

  if (isInitialized)
  {
    if (slice.m_iSliceQp != m_sliceQp)
    {
      const unsigned numPos = MAX_CU_SIZE >> MIN_CU_LOG2;
      for (unsigned x = 0; x < numPos; x++)
      {
        for (unsigned y = 0; y < numPos; y++)
        {
          for (int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++)
          {
            if (m_bestEncInfo[x][y][wIdx])
            {
              for (int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++)
              {
                if (m_bestEncInfo[x][y][wIdx][hIdx])
                {
                  m_bestEncInfo[x][y][wIdx][hIdx]->cu.qp = -128;
                }
              }
            }
          }
        }
      }
      m_sliceQp = slice.m_iSliceQp;
    }
    return;
  }

  size_t numCoeff = 0;

  for (unsigned x = 0; x < numPos; x++)
  {
    for (unsigned y = 0; y < numPos; y++)
    {
      for (int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++)
      {
        if (m_bestEncInfo[x][y][wIdx])
        {
          for (int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++)
          {
            if (m_bestEncInfo[x][y][wIdx][hIdx])
            {
              for (const CompArea &blk: m_bestEncInfo[x][y][wIdx][hIdx]->cu.blocks)
              {
                numCoeff += blk.area();
              }
            }
          }
        }
      }
    }
  }

#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS
  m_pCoeff         = new TCoeff[numCoeff * REUSE_CU_RESULTS_MAX_NUM_TUS];
  m_pSignsPredArea = new uint8_t[numCoeff * REUSE_CU_RESULTS_MAX_NUM_TUS];
  if (slice.m_sps->m_PLTMode)
  {
    m_pltIdx  = new Pel[numCoeff * REUSE_CU_RESULTS_MAX_NUM_TUS];
    m_runType = new bool[numCoeff * REUSE_CU_RESULTS_MAX_NUM_TUS];
  }
#else
  m_pCoeff         = new TCoeff[numCoeff];
  m_pSignsPredArea = new uint8_t[numCoeff];
  if (slice.m_sps->m_PLTMode)
  {
    m_pltIdx  = new Pel[numCoeff];
    m_runType = new bool[numCoeff];
  }
#endif

  TCoeff  *coeffPtr         = m_pCoeff;
  uint8_t *signsPredAreaPtr = m_pSignsPredArea;
  Pel     *pltIdxPtr        = m_pltIdx;
  bool    *runTypePtr       = m_runType;
  m_dummyCS.pcv             = m_pcv;

  for (unsigned x = 0; x < numPos; x++)
  {
    for (unsigned y = 0; y < numPos; y++)
    {
      for (int wIdx = 0; wIdx < gp_sizeIdxInfo->numWidths(); wIdx++)
      {
        if (m_bestEncInfo[x][y][wIdx])
        {
          for (int hIdx = 0; hIdx < gp_sizeIdxInfo->numHeights(); hIdx++)
          {
            if (m_bestEncInfo[x][y][wIdx][hIdx])
            {
              TCoeff *coeff[MAX_NUM_TBLOCKS] = {
                0,
              };
              uint8_t *signsPredArea[MAX_NUM_TBLOCKS] = {
                0,
              };
              EnumArray<Pel *, ChannelType> pltIdx;
              pltIdx.fill(nullptr);
              EnumArray<bool *, ChannelType> runType;
              runType.fill(nullptr);

#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS
              for (int i = 0; i < REUSE_CU_RESULTS_MAX_NUM_TUS; i++)
              {
                TransformUnit  &tu   = m_bestEncInfo[x][y][wIdx][hIdx]->tus[i];
                const UnitArea &area = tu;

                for (int i = 0; i < area.blocks.size(); i++)
                {
                  coeff[i] = coeffPtr;
                  coeffPtr += area.blocks[i].area();
                  signsPredArea[i] = signsPredAreaPtr;
                  signsPredAreaPtr += area.blocks[i].area();
                  const auto        compId = CompID(i);
                  const ChannelType chType = toChannelType(compId);
                  if (compId == getFirstComponentOfChannel(chType) && pltIdxPtr != nullptr)
                  {
                    pltIdx[chType] = pltIdxPtr;
                    pltIdxPtr += area.blocks[i].area();
                  }
                  if (compId == getFirstComponentOfChannel(chType) && runTypePtr != nullptr)
                  {
                    runType[chType] = runTypePtr;
                    runTypePtr += area.blocks[i].area();
                  }
                }

                tu.cs = &m_dummyCS;
                tu.init(coeff, signsPredArea, pltIdx, runType);
              }
#else
              const UnitArea &area = m_bestEncInfo[x][y][wIdx][hIdx]->tu;

              for (int i = 0; i < area.blocks.size(); i++)
              {
                coeff[i] = coeffPtr;
                coeffPtr += area.blocks[i].area();
                signsPredArea[i] = signsPredAreaPtr;
                signsPredAreaPtr += area.blocks[i].area();
                pltIdx[i] = pltIdxPtr;
                pltIdxPtr += area.blocks[i].area();
                runType[i] = runTypePtr;
                runTypePtr += area.blocks[i].area();
              }

              m_bestEncInfo[x][y][wIdx][hIdx]->tu.cs = &m_dummyCS;
              m_bestEncInfo[x][y][wIdx][hIdx]->tu.init(coeff, signsPredArea, pltIdx, runType);
#endif
            }
          }
        }
      }
    }
  }
}

bool BestEncInfoCache::setFromCs(const CodingStructure &cs, const Partitioner &partitioner)
{
#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS
  if (cs.cus.size() != 1 || cs.tus.size() > REUSE_CU_RESULTS_MAX_NUM_TUS)
#else
  if (cs.cus.size() != 1 || cs.tus.size() != 1)
#endif
  {
    return false;
  }

  unsigned idX, idY, idW, idH;
  getAreaIdx(cs.area.Y(), *m_pcv, idX, idY, idW, idH);
  BestEncodingInfo &encInfo = *m_bestEncInfo[idX][idY][idW][idH];

  encInfo.poc = cs.picture->m_poc;
  encInfo.cu.repositionTo(*cs.cus.front());
#if !REUSE_CU_RESULTS_WITH_MULTIPLE_TUS
  encInfo.tu.repositionTo(*cs.tus.front());
#endif
  encInfo.cu = *cs.cus.front();
#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS
  int tuIdx = 0;
  for (auto tu: cs.tus)
  {
    encInfo.tus[tuIdx].repositionTo(*tu);
    encInfo.tus[tuIdx].resizeTo(*tu);
    for (auto &blk: tu->blocks)
    {
      if (blk.valid())
      {
        encInfo.tus[tuIdx].copyComponentFrom(*tu, blk.compID);
      }
    }
    tuIdx++;
  }
  CHECKD(cs.tus.size() > MAX_NUM_TUS, "Exceeding tus array boundaries");
  encInfo.numTus = cs.tus.size();
#else
  for (auto &blk: cs.tus.front()->blocks)
  {
    if (blk.valid())
    {
      encInfo.tu.copyComponentFrom(*cs.tus.front(), blk.compID);
    }
  }
#endif
  encInfo.testMode     = getCSEncMode(cs);
  encInfo.dist         = cs.dist;
  encInfo.encDbOptCost = cs.costDbOffset;

  return true;
}

bool BestEncInfoCache::isValid(const CodingStructure &cs, const Partitioner &partitioner, int qp) const
{
  unsigned idX, idY, idW, idH;
  getAreaIdx(cs.area.Y(), *m_pcv, idX, idY, idW, idH);
  BestEncodingInfo &encInfo = *m_bestEncInfo[idX][idY][idW][idH];

  if (encInfo.cu.qp != qp || cs.slice->m_chromaQpAdjEnabled)
  {
    return false;
  }
  if (cs.picture->m_poc != encInfo.poc ||
      CS::getArea(cs, cs.area, partitioner.chType) != CS::getArea(cs, encInfo.cu, partitioner.chType) ||
      !isTheSameNbHood(encInfo.cu, cs, partitioner, cs.picture->lwidth(), cs.picture->lheight()) ||
      CU::isIBC(encInfo.cu) || partitioner.currQgEnable() || cs.currQP[partitioner.chType] != encInfo.cu.qp)
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool BestEncInfoCache::setCsFrom(CodingStructure &cs, EncTestMode &testMode, const Partitioner &partitioner) const
{
  unsigned idX, idY, idW, idH;
  getAreaIdx(cs.area.Y(), *m_pcv, idX, idY, idW, idH);
  BestEncodingInfo &encInfo = *m_bestEncInfo[idX][idY][idW][idH];

  if (cs.picture->m_poc != encInfo.poc ||
      CS::getArea(cs, cs.area, partitioner.chType) != CS::getArea(cs, encInfo.cu, partitioner.chType) ||
      !isTheSameNbHood(encInfo.cu, cs, partitioner, cs.picture->lwidth(), cs.picture->lheight()) ||
      partitioner.currQgEnable() || cs.currQP[partitioner.chType] != encInfo.cu.qp)
  {
    return false;
  }

  CodingUnit &cu = cs.addCU(CS::getArea(cs, cs.area, partitioner.chType), partitioner.chType);
#if !REUSE_CU_RESULTS_WITH_MULTIPLE_TUS
  TransformUnit &tu = cs.addTU(CS::getArea(cs, cs.area, partitioner.chType), partitioner.chType);
#endif

  cu.repositionTo(encInfo.cu);
#if !REUSE_CU_RESULTS_WITH_MULTIPLE_TUS
  tu.repositionTo(encInfo.tu);
#endif

  cu = encInfo.cu;
#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS
  CHECKD(!(encInfo.numTus > 0), "Empty tus array");
  for (int i = 0; i < encInfo.numTus; i++)
  {
    TransformUnit &tu = cs.addTU(encInfo.tus[i], partitioner.chType);

    for (auto &blk: tu.blocks)
    {
      if (blk.valid())
      {
        tu.copyComponentFrom(encInfo.tus[i], blk.compID);
      }
    }
  }
#else
  for (auto &blk: tu.blocks)
  {
    if (blk.valid())
    {
      tu.copyComponentFrom(encInfo.tu, blk.compID);
    }
  }
#endif

  cs.dist         = encInfo.dist;
  cs.costDbOffset = encInfo.encDbOptCost;
  testMode        = encInfo.testMode;

  return true;
}

#endif

//////////////////////////////////////////////////////////////////////////
// EncModeCtrlQTBT
//////////////////////////////////////////////////////////////////////////

void EncModeCtrl::create(const EncCfg *encCfg)
{
  CacheBlkInfoCtrl::create();
#if REUSE_CU_RESULTS
  BestEncInfoCache::create(encCfg->m_chromaFormatIdc);
#endif
  SaveLoadEncInfoSbt::create();
}

void EncModeCtrl::destroy()
{
  CacheBlkInfoCtrl::destroy();
#if REUSE_CU_RESULTS
  BestEncInfoCache::destroy();
#endif
  SaveLoadEncInfoSbt::destroy();
}

void EncModeCtrl::initCTUEncoding(const Slice &slice)
{
  CacheBlkInfoCtrl::init(slice);
#if REUSE_CU_RESULTS
  BestEncInfoCache::init(slice);
#endif
  SaveLoadEncInfoSbt::init(slice);

  CHECK(!m_ComprCUCtxList.empty(), "Mode list is not empty at the beginning of a CTU");

  m_slice = &slice;

  if (m_encCfg->m_e0023FastEnc)
  {
    const int thres = m_encCfg->m_compositeRefEnabled ? 2 * PICTURE_DISTANCE_TH : PICTURE_DISTANCE_TH;
    m_skipSplitThreshold =
      ((slice.getMinPictureDistance(m_encCfg->m_ibcFastMethod) <= thres) ? FAST_SKIP_DEPTH : SKIP_DEPTH);
  }
  else
  {
    m_skipSplitThreshold = SKIP_DEPTH;
  }
}

void EncModeCtrl::initCULevel(Partitioner &partitioner, const CodingStructure &cs)
{
  // Min/max depth
  unsigned minDepth = 0;
  unsigned maxDepth =
    floorLog2(cs.sps->m_ctuSize) - floorLog2(cs.sps->getMinQTSize(m_slice->m_eSliceType, partitioner.chType));
  if (m_encCfg->m_useFastLCTU)
  {
    if (auto adPartitioner = dynamic_cast<AdaptiveDepthPartitioner *>(&partitioner))
    {
      // LARGE CTU
      adPartitioner->setMaxMinDepth(minDepth, maxDepth, cs);
    }
  }

  m_ComprCUCtxList.push_back(ComprCUCtx(cs, minDepth, maxDepth));

  const CodingUnit *cuLeft  = cs.getCU(cs.area.block(partitioner.chType).pos().offset(-1, 0), partitioner.chType);
  const CodingUnit *cuAbove = cs.getCU(cs.area.block(partitioner.chType).pos().offset(0, -1), partitioner.chType);
  unsigned          maxBtd  = cs.pcv->getMaxBtDepth(*cs.slice, partitioner.chType);

  bool qtBeforeBt =
    ((cuLeft && cuAbove && cuLeft->qtDepth > partitioner.currQtDepth && cuAbove->qtDepth > partitioner.currQtDepth) ||
     (cuLeft && !cuAbove && cuLeft->qtDepth > partitioner.currQtDepth) ||
     (!cuLeft && cuAbove && cuAbove->qtDepth > partitioner.currQtDepth) ||
     (!cuAbove && !cuLeft && cs.area.lwidth() >= (32 << cs.slice->m_hierPredLayerIdx)) ||
     (m_encCfg->m_qtbttSpeedUp && maxBtd < ((cs.slice->isIntra() && !cs.sps->m_ibcFlag) ? 3 : 2))) &&
    (cs.area.lwidth() > (cs.pcv->getMinQtSize(*cs.slice, partitioner.chType) << 1));

  bool canNo, canQt, canBh, canBv, canTh, canTv;

  partitioner.canSplit(cs, canNo, canQt, canBh, canBv, canTh, canTv);

  if (!canBh && !canBv)
  {
    qtBeforeBt = true;
  }

  comprCUCtx = &m_ComprCUCtxList.back();

  // set features
  comprCUCtx->bestNonSplitCost     = MAX_DOUBLE;
  comprCUCtx->bestCostVertSplit    = MAX_DOUBLE;
  comprCUCtx->bestCostHorzSplit    = MAX_DOUBLE;
  comprCUCtx->bestCostTriHorzSplit = MAX_DOUBLE;
  comprCUCtx->bestCostTriVertSplit = MAX_DOUBLE;
  comprCUCtx->doTriHorzSplit       = canTh;
  comprCUCtx->doTriVertSplit       = canTv;
  comprCUCtx->doMoreSplits         = 3;
  comprCUCtx->bestCostImv          = UNSET_IMV_COST;
  comprCUCtx->bestCostNoImv        = UNSET_IMV_COST;
  comprCUCtx->bestCostGPM          = UNSET_IMV_COST;
  comprCUCtx->bestCostLIC          = UNSET_IMV_COST;
  comprCUCtx->qtBeforeBt           = qtBeforeBt;
  comprCUCtx->didQuadSplit         = false;
  comprCUCtx->isBestNoSplitSkip    = false;
  comprCUCtx->maxQtSubDepth        = 0;
  comprCUCtx->baseQp               = -1;

  // QP
  int baseQP = cs.baseQP;
  if (!CS::isDualITree(cs) || isLuma(partitioner.chType))
  {
    if (m_encCfg->m_bUseAdaptiveQP)
    {
      baseQP = Clip3(-cs.sps->m_qpBDOffset[ChannelType::LUMA], MAX_QP, baseQP + xComputeDQP(cs, partitioner));
    }
#if ENABLE_QPA_SUB_CTU
    else if (m_encCfg->m_bUsePerceptQPA && !m_encCfg->m_RCEnableRateControl && cs.pps->m_useDQP &&
             cs.slice->getCuQpDeltaSubdiv() > 0)
    {
      const PreCalcValues &pcv = *cs.pcv;

      if ((partitioner.currArea().lwidth() < pcv.maxCUWidth) && (partitioner.currArea().lheight() < pcv.maxCUHeight) &&
          cs.picture)
      {
        const Position &pos     = partitioner.currQgPos;
        const unsigned  mtsLog2 = (unsigned)floorLog2(std::min(cs.sps->getMaxTbSize(), pcv.maxCUWidth));
        const unsigned  stride  = pcv.maxCUWidth >> mtsLog2;

        baseQP = cs.picture->m_subCtuQP[((pos.x & pcv.maxCUWidthMask) >> mtsLog2) +
                                        stride * ((pos.y & pcv.maxCUHeightMask) >> mtsLog2)];
      }
    }
#endif
#if SHARP_LUMA_DELTA_QP
    if (m_encCfg->m_lumaLevelToDeltaQPMapping.isEnabled())
    {
      if (partitioner.currQgEnable())
      {
        m_lumaQPOffset = calculateLumaDQP(cs.getOrgBuf(clipArea(cs.area.Y(), cs.picture->Y())));
      }
      baseQP = Clip3(-cs.sps->m_qpBDOffset[ChannelType::LUMA], MAX_QP, baseQP - m_lumaQPOffset);
    }
#endif
    if (m_encCfg->m_smoothQPReductionEnable)
    {
      int smoothQPoffset = 0;
      if (partitioner.currQgEnable())
      {
        // enable smooth QP reduction on selected frames
        bool checkSmoothQP = false;
        if (m_encCfg->m_smoothQPReductionPeriodicity != 0)
        {
          checkSmoothQP = ((m_encCfg->m_smoothQPReductionPeriodicity == 0) && cs.slice->isIntra()) ||
            (m_encCfg->m_smoothQPReductionPeriodicity == 1) ||
            ((cs.slice->m_poc % m_encCfg->m_smoothQPReductionPeriodicity) == 0);
        }
        else
        {
          checkSmoothQP = ((m_encCfg->m_smoothQPReductionPeriodicity == 0) && cs.slice->isIntra());
        }
        if (checkSmoothQP)
        {
          bool isIntraSlice = cs.slice->isIntra();
          if (isIntraSlice)
          {
            smoothQPoffset = calculateLumaDQPsmooth(
              cs.getOrgBuf(clipArea(cs.area.Y(), cs.picture->Y())), baseQP, m_encCfg->m_smoothQPReductionThresholdIntra,
              m_encCfg->m_smoothQPReductionModelScaleIntra, m_encCfg->m_smoothQPReductionModelOffsetIntra,
              m_encCfg->m_smoothQPReductionLimitIntra);
          }
          else
          {
            smoothQPoffset = calculateLumaDQPsmooth(
              cs.getOrgBuf(clipArea(cs.area.Y(), cs.picture->Y())), baseQP, m_encCfg->m_smoothQPReductionThresholdInter,
              m_encCfg->m_smoothQPReductionModelScaleInter, m_encCfg->m_smoothQPReductionModelOffsetInter,
              m_encCfg->m_smoothQPReductionLimitInter);
          }
        }
      }
      baseQP = Clip3(-cs.sps->m_qpBDOffset[ChannelType::LUMA], MAX_QP, baseQP + smoothQPoffset);
    }
  }
  comprCUCtx->baseQp = baseQP;

#if REUSE_CU_RESULTS
  const bool isReusingCu = BestEncInfoCache::isValid(cs, partitioner, baseQP);  // put this into try mode
#else
  const bool isReusingCu = false;
#endif
  comprCUCtx->isReusingCu = isReusingCu;

  comprCUCtx->lastTestMode = EncTestMode();
}

void EncModeCtrl::finishCULevel(Partitioner &partitioner)
{
  m_ComprCUCtxList.pop_back();
  comprCUCtx = m_ComprCUCtxList.size() ? &m_ComprCUCtxList.back() : nullptr;
}

const CodingUnit *EncModeCtrl::getBestCU(int depthOffset) const
{
  if (depthOffset + 1 < m_ComprCUCtxList.size())
  {
    return m_ComprCUCtxList[m_ComprCUCtxList.size() - 1 - depthOffset].bestCU;
  }
  else
  {
    return nullptr;
  }
}

bool EncModeCtrl::trySplit(const EncTestMode &encTestmode, const CodingStructure &cs, Partitioner &partitioner)
{
  if (comprCUCtx->currTestMode.type == ETM_INVALID && comprCUCtx->currTestMode.opts == ETO_INVALID)
  {
    comprCUCtx->currTestMode = encTestmode;
  }
  comprCUCtx->lastTestMode = comprCUCtx->currTestMode;

  bool valid = xTrySplitInternal(encTestmode, cs, partitioner);
  if (valid)
  {
    comprCUCtx->currTestMode = encTestmode;
  }
  return valid;
}

bool EncModeCtrl::xTrySplitInternal(const EncTestMode &encTestmode, const CodingStructure &cs, Partitioner &partitioner)
{
  ComprCUCtx &cuECtx = m_ComprCUCtxList.back();

  CHECK(!isModeSplit(encTestmode), " wrong split mode");

  // Fast checks, partitioning depended
  if (cuECtx.isHashPerfectMatch)
  {
    return false;
  }

  const PartSplit implicitSplit = partitioner.getImplicitSplit(cs);
  const bool      isBoundary    = implicitSplit != CU_DONT_SPLIT;

  if (isBoundary && encTestmode.type != ETM_SPLIT_QT)
  {
    return getPartSplit(encTestmode) == implicitSplit;
  }
  else if (isBoundary && encTestmode.type == ETM_SPLIT_QT)
  {
    return partitioner.canSplit(CU_QUAD_SPLIT, cs);
  }

  const Slice           &slice  = *m_slice;
  const uint32_t         width  = partitioner.currArea().lumaSize().width;
  const CodingStructure *bestCS = cuECtx.bestCS;
  const CodingUnit      *bestCU = cuECtx.bestCU;

  if (cuECtx.minDepth > partitioner.currQtDepth && partitioner.canSplit(CU_QUAD_SPLIT, cs))
  {
    // enforce QT
    return encTestmode.type == ETM_SPLIT_QT;
  }
  else if (encTestmode.type == ETM_SPLIT_QT && cuECtx.maxDepth <= partitioner.currQtDepth)
  {
    if (!(partitioner.chType == ChannelType::LUMA && partitioner.currBtDepth == 0 &&
          (partitioner.currArea().lwidth() == 128 || partitioner.currArea().lwidth() == 64 ||
           partitioner.currArea().lwidth() == 32)) ||
        !m_encCfg->m_tempPartPredEnabled)
    {
    // don't check this QT depth
      return false;
    }
  }

  //////////////////////////////////////////////////////////////////////////
  // skip-history rule - don't split further if at least for three past levels
  //                     in the split tree it was found that skip is the best mode
  //////////////////////////////////////////////////////////////////////////
  int skipScore = 0;

  if ((!slice.isIntra() || slice.m_ibcFlag) && cuECtx.isBestNoSplitSkip)
  {
    for (int i = 2; i < m_ComprCUCtxList.size(); i++)
    {
      if ((m_ComprCUCtxList.end() - i)->isBestNoSplitSkip)
      {
        skipScore += 1;
      }
      else
      {
        break;
      }
    }
  }

  const PartSplit split = getPartSplit(encTestmode);
  if (!partitioner.canSplit(split, cs) || skipScore >= 2)
  {
    if (split == CU_HORZ_SPLIT)
    {
      cuECtx.didHorzSplit = false;
    }
    if (split == CU_VERT_SPLIT)
    {
      cuECtx.didVertSplit = false;
    }
    if (split == CU_QUAD_SPLIT)
    {
      cuECtx.didQuadSplit = false;
    }
    return false;
  }

  if (m_encCfg->m_contentBasedFastQtbt)
  {
    const CompArea &currArea = partitioner.currArea().Y();
    int             cuHeight = currArea.height;
    int             cuWidth  = currArea.width;

    const bool condIntraInter =
      m_encCfg->m_intraPeriod == 1 ? (partitioner.currBtDepth == 0) : (cuHeight > 32 && cuWidth > 32);

    if (cuWidth == cuHeight && condIntraInter && split != CU_QUAD_SPLIT)
    {
      const CPelBuf bufCurrArea = cs.getOrgBuf(partitioner.currArea().block(COMP_Y));

      double horVal = 0;
      double verVal = 0;
      double dupVal = 0;
      double dowVal = 0;

      const double th = m_encCfg->m_intraPeriod == 1 ? 1.2 : 1.0;

      unsigned j, k;

      for (j = 0; j < cuWidth - 1; j++)
      {
        for (k = 0; k < cuHeight - 1; k++)
        {
          horVal += abs(bufCurrArea.at(j + 1, k) - bufCurrArea.at(j, k));
          verVal += abs(bufCurrArea.at(j, k + 1) - bufCurrArea.at(j, k));
          dowVal += abs(bufCurrArea.at(j + 1, k) - bufCurrArea.at(j, k + 1));
          dupVal += abs(bufCurrArea.at(j + 1, k + 1) - bufCurrArea.at(j, k));
        }
      }
      if (horVal > th * verVal && sqrt(2) * horVal > th * dowVal && sqrt(2) * horVal > th * dupVal &&
          (split == CU_HORZ_SPLIT || split == CU_TRIH_SPLIT))
      {
        return false;
      }
      if (th * dupVal < sqrt(2) * verVal && th * dowVal < sqrt(2) * verVal && th * horVal < verVal &&
          (split == CU_VERT_SPLIT || split == CU_TRIV_SPLIT))
      {
        return false;
      }
    }

    if (m_encCfg->m_intraPeriod == 1 && cuWidth <= 32 && cuHeight <= 32 && bestCS && bestCS->tus.size() == 1 &&
        bestCU && bestCU->depth == partitioner.currDepth && partitioner.currBtDepth > 1 && isLuma(partitioner.chType))
    {
      if (!bestCU->rootCbf)
      {
        return false;
      }
    }
  }

  if (bestCU && bestCU->skip && bestCU->mtDepth >= m_skipSplitThreshold && !isModeSplit(cuECtx.lastTestMode))
  {
    return false;
  }

//    int featureToSet = -1;

  switch (split)
  {
  case CU_QUAD_SPLIT:
    {
      if (!cuECtx.qtBeforeBt && bestCU)
      {
        unsigned          maxBTD = partitioner.currPartLevel().maxMttDepth;
        const CodingUnit *cuBR   = bestCS->cus.back();
        unsigned          height = partitioner.currArea().lumaSize().height;

        if (bestCU &&
            ((bestCU->btDepth == 0 && maxBTD >= ((slice.isIntra() && !slice.m_ibcFlag) ? 3 : 2)) ||
             (bestCU->btDepth == 1 && cuBR && cuBR->btDepth == 1 &&
              maxBTD >= ((slice.isIntra() && !slice.m_ibcFlag) ? 4 : 3))) &&
            (width <= MAX_TB_SIZEY && height <= MAX_TB_SIZEY) && cuECtx.didHorzSplit && cuECtx.didVertSplit)
        {
          return false;
        }
      }
      if (m_encCfg->m_bUseEarlyCU && bestCS && bestCS->cost != MAX_DOUBLE && bestCU && bestCU->skip &&
          cuECtx.nonSkipWasTested && bestCS->cus.size() == 1)
      {
        return false;
      }
      if (fastDeltaQp && width <= slice.m_pps->pcv->fastDeltaQPCuMaxSize)
      {
        return false;
      }
    }
    break;
  case CU_HORZ_SPLIT:
    break;
  case CU_VERT_SPLIT:
    break;
  case CU_TRIH_SPLIT:
    if (cuECtx.didHorzSplit && bestCU && bestCU->btDepth == partitioner.currBtDepth && !bestCU->rootCbf)
    {
      return false;
    }

    if (m_encCfg->m_qtbttSpeedUp)
    {
      if (cuECtx.didHorzSplit && cuECtx.didVertSplit && cuECtx.bestCostHorzSplit > cuECtx.bestCostVertSplit)
      {
        return false;
      }
    }

    if (!cuECtx.doTriHorzSplit)
    {
      return false;
    }

    if (m_encCfg->m_ttFastSkip && xSkipTreeCandidate(split, cuECtx, m_slice->m_eSliceType))
    {
      return false;
    }
    break;
  case CU_TRIV_SPLIT:
    if (cuECtx.didVertSplit && bestCU && bestCU->btDepth == partitioner.currBtDepth && !bestCU->rootCbf)
    {
      return false;
    }

    if (m_encCfg->m_qtbttSpeedUp)
    {
      if (cuECtx.didHorzSplit && cuECtx.didVertSplit && cuECtx.bestCostHorzSplit < cuECtx.bestCostVertSplit)
      {
        return false;
      }
    }

    if (!cuECtx.doTriVertSplit)
    {
      return false;
    }

    if (m_encCfg->m_ttFastSkip && xSkipTreeCandidate(split, cuECtx, m_slice->m_eSliceType))
    {
      return false;
    }
    break;
  default:
    THROW("Only CU split modes are governed by the EncModeCtrl");
    return false;
    break;
  }

  switch (split)
  {
  case CU_HORZ_SPLIT:
  case CU_TRIH_SPLIT:
    if (cuECtx.qtBeforeBt && cuECtx.didQuadSplit)
    {
      if (cuECtx.maxQtSubDepth > partitioner.currQtDepth + 1)
      {
        if (CU_HORZ_SPLIT == split)
        {
          cuECtx.didHorzSplit = false;
        }
        return false;
      }
    }
    break;
  case CU_VERT_SPLIT:
  case CU_TRIV_SPLIT:
    if (cuECtx.qtBeforeBt && cuECtx.didQuadSplit)
    {
      if (cuECtx.maxQtSubDepth > partitioner.currQtDepth + 1)
      {
        if (CU_VERT_SPLIT == split)
        {
          cuECtx.didVertSplit = false;
        }
        return false;
      }
    }
    break;
  default:
    break;
  }

  if (cs.sps->m_log2ParallelMergeLevelMinus2)
  {
    const CompArea &area = partitioner.currArea().Y();
    const SizeType  size = 1 << (cs.sps->m_log2ParallelMergeLevelMinus2 + 2);

    if (!cs.slice->isIntra() && (area.width > size || area.height > size))
    {
      if (area.height <= size && split == CU_HORZ_SPLIT)
      {
        return false;
      }
      if (area.width <= size && split == CU_VERT_SPLIT)
      {
        return false;
      }
      if (area.height <= 2 * size && split == CU_TRIH_SPLIT)
      {
        return false;
      }
      if (area.width <= 2 * size && split == CU_TRIV_SPLIT)
      {
        return false;
      }
    }
  }

  const bool doSplit = !m_encCfg->m_qtbttSpeedUp || !!cuECtx.doMoreSplits;

  if (split == CU_QUAD_SPLIT)
  {
    cuECtx.didQuadSplit = doSplit;
  }
  else if (split == CU_HORZ_SPLIT)
  {
    // dont reset didHorzSplit with fastSplitScoring
    cuECtx.didHorzSplit |= doSplit;
  }
  else if (split == CU_VERT_SPLIT)
  {
    // dont reset didVertSplit with fastSplitScoring
    cuECtx.didVertSplit |= doSplit;
  }
  return doSplit;
}

bool EncModeCtrl::tryMode(const EncTestMode &encTestmode, const CodingStructure &cs, Partitioner &partitioner)
{
  if (comprCUCtx->currTestMode.type == ETM_INVALID && comprCUCtx->currTestMode.opts == ETO_INVALID)
  {
    comprCUCtx->currTestMode = encTestmode;
  }

  comprCUCtx->lastTestMode = comprCUCtx->currTestMode;
  bool valid               = xTryModeInternal(encTestmode, cs, partitioner);
  if (valid)
  {
    comprCUCtx->currTestMode = encTestmode;
  }
  return valid;
}

bool EncModeCtrl::xTryModeInternal(const EncTestMode &encTestmode, const CodingStructure &cs, Partitioner &partitioner)
{
  ComprCUCtx &cuECtx = m_ComprCUCtxList.back();

  if (cuECtx.minDepth > partitioner.currQtDepth && partitioner.canSplit(CU_QUAD_SPLIT, cs))
  {
    // enforce QT
    return false;
  }

  // Fast checks, partitioning depended
  if (cuECtx.isHashPerfectMatch && encTestmode.type != ETM_MERGE_SKIP && encTestmode.type != ETM_INTER_ME)
  {
    return false;
  }

  // if early skip detected, skip all modes checking but the splits
  if (cuECtx.earlySkip && m_encCfg->m_useEarlySkipDetection && !isModeSplit(encTestmode) && !(isModeInter(encTestmode)))
  {
    return false;
  }

#if REUSE_CU_RESULTS
  if (cuECtx.isReusingCu)
  {
    // if reusing is enabled do not test other modes than reco cached
    return (encTestmode.type == ETM_RECO_CACHED);
  }
  else if (encTestmode.type == ETM_RECO_CACHED)
  {
    return false;
  }
#endif
  const Slice           &slice    = *m_slice;
  const uint32_t         numComp  = getNumberValidComponents(slice.m_sps->m_chromaFormatIdc);
  const CodingStructure *bestCS   = cuECtx.bestCS;
  const EncTestMode      bestMode = bestCS ? getCSEncMode(*bestCS) : EncTestMode();

  CodedCUInfo &relatedCU = CacheBlkInfoCtrl::getCodedCUInfo(partitioner.currArea());

  if (partitioner.currArea().Y().valid() && cuECtx.lastTestMode.type == ETM_INTRA && bestCS && bestCS->cus.size() == 1)
  {
    int cnt = 0;
    for (auto tu: bestCS->tus)
    {
      cnt += tu->countNonZero();
    }
    cuECtx.bestIntraNzCnt = cnt;
  }

  if (encTestmode.type == ETM_INTRA)
  {
    if (fastDeltaQp && (cs.area.lumaSize().width > cs.pcv->fastDeltaQPCuMaxSize))
    {
      return false; // only check necessary 2Nx2N Intra in fast delta-QP mode
    }

    const int maxSize = std::min<int>(MAX_TB_SIZEY, MAX_INTRA_SIZE);

    if (m_encCfg->m_useFastLCTU && partitioner.currArea().lumaSize().area() > maxSize * maxSize)
    {
      return (m_encCfg->m_dualITree == 0 && m_encCfg->m_uiMaxMTTHierarchyDepthI == 0 &&
              cs.sps->getMinQTSize(cs.slice->m_eSliceType, partitioner.chType) > maxSize);
    }

    if (partitioner.currArea().lumaSize().width > maxSize || partitioner.currArea().lumaSize().height > maxSize)
    {
      return false;
    }

    if (m_encCfg->m_usePbIntraFast && (!cs.slice->isIntra() || cs.slice->m_ibcFlag) && (0 == cuECtx.interHad) &&
        cuECtx.bestCU && !CU::isIntra(*cuECtx.bestCU))
    {
      return false;
    }

    // INTRA MODES
    if (cs.slice->m_ibcFlag && !cuECtx.bestTU)
    {
      return true;
    }
    if (partitioner.currArea().lumaSize().width == 4 && partitioner.currArea().lumaSize().height == 4 &&
        !slice.isIntra() && !cuECtx.bestTU)
    {
      return true;
    }
    if (!(slice.isIntra() || bestMode.type == ETM_INTRA || !cuECtx.bestTU ||
          ((!m_encCfg->m_bDisableIntraPUsInInterSlices) && (!relatedCU.isInter || !relatedCU.isIBC) &&
           ((cuECtx.bestTU->cbf[0] != 0) || ((numComp > COMP_Cb) && cuECtx.bestTU->cbf[1] != 0) ||
            ((numComp > COMP_Cr) && cuECtx.bestTU->cbf[2] != 0)  // avoid very complex intra if it is unlikely
            ))))
    {
      return false;
    }
    if ((m_encCfg->m_ibcFastMethod & IBC_FAST_METHOD_NOINTRA_IBCCBF0) &&
        (bestMode.type == ETM_IBC || bestMode.type == ETM_IBC_MERGE) &&
        (!cuECtx.bestCU->Y().valid() || cuECtx.bestTU->cbf[0] == 0) &&
        (!cuECtx.bestCU->Cb().valid() || cuECtx.bestTU->cbf[1] == 0) &&
        (!cuECtx.bestCU->Cr().valid() || cuECtx.bestTU->cbf[2] == 0))
    {
      return false;
    }
    if (cuECtx.lastTestMode.type != ETM_INTRA && cuECtx.bestCS && cuECtx.bestCU && cuECtx.interHad)
    {
      // th put this somewhere else
      // Get SATD threshold from best Inter-CU
      if (!cs.slice->isIntra() && m_encCfg->m_usePbIntraFast && !cs.slice->m_disableSATDForRd)
      {
        CodingUnit *bestCU = cuECtx.bestCU;
        if (bestCU && !CU::isIntra(*bestCU))
        {
          DistParam distParam;
          const int useHad = 1;
          m_pcRdCost->setDistParam(distParam, cs.getOrgBuf(COMP_Y), cuECtx.bestCS->getPredBuf(COMP_Y),
                                   cs.sps->m_bitDepths[ChannelType::LUMA], COMP_Y, useHad);
          cuECtx.interHad = distParam.distFunc(distParam);
        }
      }
    }
    return true;
  }
  else if (encTestmode.type == ETM_PALETTE)
  {
    if (partitioner.currArea().lumaSize().width > 64 || partitioner.currArea().lumaSize().height > 64 ||
        ((partitioner.currArea().lumaSize().width * partitioner.currArea().lumaSize().height <= 16) &&
         (isLuma(partitioner.chType))) ||
        ((isChromaEnabled(partitioner.currArea().chromaFormat) &&
          partitioner.currArea().chromaSize().width * partitioner.currArea().chromaSize().height <= 16) &&
         (!isLuma(partitioner.chType)) && CS::isDualITree(cs)))
    {
      return false;
    }
    const Area currCu =
      CS::getArea(cs, cs.area, partitioner.chType).blocks[getFirstComponentOfChannel(partitioner.chType)];
    try
    {
      double storedCost = slice.m_mapPltCost[isChroma(partitioner.chType)].at(currCu.pos()).at(currCu.size());
      if (bestMode.type != ETM_INVALID && storedCost > cuECtx.bestCS->cost)
      {
        return false;
      }
    }
    catch (const std::out_of_range &)
    {
      // do nothing if no stored cost value was found.
    }
    return doPlt;
  }
  else if (encTestmode.type == ETM_IBC || encTestmode.type == ETM_IBC_MERGE)
  {
    // IBC MODES
    if ((m_encCfg->m_ibcFastMethod & IBC_FAST_METHOD_NONSCC) && cuECtx.bestIntraNzCnt < IBC_NONSCC_ENC_RD_NZ_COUNT)
    {
      return false;
    }
    return slice.m_ibcFlag && partitioner.currArea().lumaSize().width <= IBC_MAX_CU_SIZE &&
      partitioner.currArea().lumaSize().height <= IBC_MAX_CU_SIZE;
  }
  else if (encTestmode.type == ETM_HASH_INTER)
  {
    const int minSize = std::min(cs.area.lwidth(), cs.area.lheight());
    return useHashME && (minSize < 128 && minSize >= 4);
  }
  else if (isModeInter(encTestmode))
  {
    // INTER MODES (ME + MERGE/SKIP)
    CHECK(slice.isIntra(), "Inter-mode should not be in the I-Slice mode list!");

    if (fastDeltaQp)
    {
      if (encTestmode.type == ETM_MERGE_SKIP)
      {
        return false;
      }
      if (cs.area.lumaSize().width > cs.pcv->fastDeltaQPCuMaxSize)
      {
        return false; // only check necessary 2Nx2N Inter in fast deltaqp mode
      }
    }

    // --- Check if we can quit current mode using SAVE/LOAD coding history

    if (encTestmode.type == ETM_INTER_ME)
    {
      if (encTestmode.opts == ETO_STANDARD)
      {
        // NOTE: ETO_STANDARD is always done when early SKIP mode detection is enabled
        if (!m_encCfg->m_useEarlySkipDetection && (relatedCU.isSkip || relatedCU.isIntra))
        {
          return false;
        }
      }
      else if (encTestmode.getAmvrSearchMode() != EncTestMode::AmvrSearchMode::NONE)
      {
        if (!m_encCfg->m_AffineAmvr &&
            (encTestmode.getAmvrSearchMode() == EncTestMode::AmvrSearchMode::FOUR_PEL_FAST &&
             cuECtx.bestCostNoImv * AMVR_FAST_4PEL_TH < cuECtx.bestCostImv))
        {
          return false;
        }
      }
    }

    if (encTestmode.type == ETM_INTER_ME && (encTestmode.opts & ETO_LIC))
    {
      if (relatedCU.skipLIC)
      {
        return false;
      }
      if (m_encCfg->m_fastLICMode && cuECtx.bestCU && cuECtx.bestCU->skip)
      {
        return false;
      }
      if (m_encCfg->m_fastLICMode == 2)
      {
        if ((partitioner.currArea().lumaSize().width > 64 || partitioner.currArea().lumaSize().height > 64) &&
            encTestmode.getAmvrSearchMode() == EncTestMode::AmvrSearchMode::NONE)
        {
          return false;
        }
        if (cuECtx.bestCU && encTestmode.getAmvrSearchMode() != EncTestMode::AmvrSearchMode::NONE &&
            !(cuECtx.bestCU->imv || cuECtx.bestCU->licFlag))
        {
          return false;
        }
      }
    }

    return true;
  }
  else
  {
    THROW("invalid encTestMode");
  }
}

bool EncModeCtrl::finishNonSplitModes(const CodingStructure &cs, Partitioner &pm)
{
  if (cs.cost == MAX_DOUBLE)
  {
    return false;
  }

  ComprCUCtx            &cuECtx    = *comprCUCtx;
  const Slice           &slice     = *cs.slice;
  const CodingStructure *bestCS    = cuECtx.bestCS;
  const CodingUnit      *bestCU    = cuECtx.bestCU;
  CodedCUInfo           &relatedCU = CacheBlkInfoCtrl::getCodedCUInfo(pm.currArea());

  // th this seems to be a bug
  if (cuECtx.isHashPerfectMatch)
  {
    return false;
  }

  bool insertUniMvFlag = false;
#if REUSE_CU_RESULTS
  if ((cuECtx.bestCostNoImv == UNSET_IMV_COST || cuECtx.isReusingCu) && !slice.isIntra())
#else
  if (cuECtx.bestCostNoImv == UNSET_IMV_COST && !slice.isIntra())
#endif
  {
    insertUniMvFlag = true;
  }

#if REUSE_CU_RESULTS
  if (nullptr == bestCS)
  {
    return false;
  }
  setFromCs(*bestCS, pm);
#endif

  if (bestCU->skip)
  {
    cuECtx.doMoreSplits--;
  }

  // assume the non-split modes are done and set the marks for the best found mode
  if (bestCU)
  {
    if (m_encCfg->m_fastLIC)
    {
      const double licCost = cuECtx.bestCostLIC;
      const int    c1      = (m_encCfg->m_fastLIC & 0x03);
      const int    c2      = (m_encCfg->m_fastLIC & 0x04);
      const double r       = ((c1 == 0x01) ? 1.0 : ((c1 == 0x02) ? 1.1 : 1.2));
      if (licCost != UNSET_IMV_COST)
      {
        if (licCost > (bestCS->cost * r) && bestCU->skip)
        {
          relatedCU.skipLIC = true;
        }
        else if (c2 && licCost > (bestCS->cost * 1.1))
        {
          relatedCU.skipLIC = true;
        }
      }
    }

    if (!slice.isIntra())
    {
      if (UNSET_IMV_COST != cuECtx.bestCostGPM)
      {
        relatedCU.isSkipGPM |= (cuECtx.bestCostGPM > (bestCS->cost * 1.2)) ||
          (cuECtx.bestCostGPM > bestCS->cost &&
           (bestCU->skip || (!cuECtx.bestTU->cbf[0] && !cuECtx.bestTU->cbf[1] && !cuECtx.bestTU->cbf[2])));
      }
    }

    if (CU::isInter(*bestCU))
    {
      relatedCU.isInter = true;
      relatedCU.isSkip |= bestCU->skip;
      relatedCU.isMMVDSkip |= bestCU->mmvdSkip;
      relatedCU.bcwIdx = bestCU->bcwIdx;
    }
    else if (CU::isIBC(*bestCU))
    {
      relatedCU.isIBC = true;
      relatedCU.isSkip |= bestCU->skip;
    }
    else if (CU::isIntra(*bestCU))
    {
      relatedCU.isIntra = true;
    }

    cuECtx.isBestNoSplitSkip = bestCU->skip;
  }

  // update the best non-split cost
  cuECtx.bestNonSplitCost = bestCS->cost;

  return insertUniMvFlag;
}

bool EncModeCtrl::xSkipTreeCandidate(const PartSplit split, const ComprCUCtx &cuECtx, const SliceType sliceType) const
{
  const double ttEncSpeedRate = m_encCfg->m_ttFastSkipThr;
  const double horXorVerRate  = m_encCfg->m_ttFastSkipThr;

  if (sliceType == I_SLICE && !(m_encCfg->m_ttFastSkip & FAST_METHOD_TT_ENC_SPEEDUP_ISLICE))
  {
    return false;
  }

  if (sliceType == B_SLICE && !(m_encCfg->m_ttFastSkip & FAST_METHOD_TT_ENC_SPEEDUP_BSLICE))
  {
    return false;
  }

  if (split == CU_TRIH_SPLIT)
  {
    if (m_encCfg->m_ttFastSkip & FAST_METHOD_ENC_SPEEDUP_BT_BASED)
    {
      if (cuECtx.bestNonSplitCost < MAX_DOUBLE && cuECtx.bestCostHorzSplit < MAX_DOUBLE)
      {
        if (cuECtx.bestCostHorzSplit > ttEncSpeedRate * cuECtx.bestNonSplitCost)
        {
          return true;
        }
      }
    }

    if (m_encCfg->m_ttFastSkip & FAST_METHOD_HOR_XOR_VER)
    {
      if (cuECtx.bestCostHorzSplit < MAX_DOUBLE && cuECtx.bestCostVertSplit < MAX_DOUBLE)
      {
        if (cuECtx.bestCostHorzSplit > horXorVerRate * cuECtx.bestCostVertSplit)
        {
          return true;
        }
      }
    }
  }
  if (split == CU_TRIV_SPLIT)
  {
    if (m_encCfg->m_ttFastSkip & FAST_METHOD_ENC_SPEEDUP_BT_BASED)
    {
      if (cuECtx.bestNonSplitCost < MAX_DOUBLE && cuECtx.bestCostVertSplit < MAX_DOUBLE)
      {
        if (cuECtx.bestCostVertSplit > ttEncSpeedRate * cuECtx.bestNonSplitCost)
        {
          return true;
        }
      }
    }

    if (m_encCfg->m_ttFastSkip & FAST_METHOD_HOR_XOR_VER)
    {
      if (cuECtx.bestCostHorzSplit < MAX_DOUBLE && cuECtx.bestCostVertSplit < MAX_DOUBLE)
      {
        if (cuECtx.bestCostVertSplit > horXorVerRate * cuECtx.bestCostHorzSplit)
        {
          return true;
        }
      }
    }
  }
  return false;
}

bool EncModeCtrl::useModeResult(const EncTestMode &encTestmode, CodingStructure *&tempCS, Partitioner &partitioner,
                                bool useEncDbOpt)
{
  tempCS->etmType = encTestmode.type;
  tempCS->etmOpts = encTestmode.opts;

  ComprCUCtx &cuECtx = m_ComprCUCtxList.back();

  if (encTestmode.type == ETM_SPLIT_BT_H)
  {
    cuECtx.bestCostHorzSplit = tempCS->cost;
  }
  else if (encTestmode.type == ETM_SPLIT_BT_V)
  {
    cuECtx.bestCostVertSplit = tempCS->cost;
  }
  else if (encTestmode.type == ETM_SPLIT_TT_H)
  {
    cuECtx.bestCostTriHorzSplit = tempCS->cost;
  }
  else if (encTestmode.type == ETM_SPLIT_TT_V)
  {
    cuECtx.bestCostTriVertSplit = tempCS->cost;
  }
  else if (isModeInter(encTestmode) && tempCS->cus.size() == 1)
  {
    cuECtx.nonSkipWasTested |= !tempCS->cus.front()->skip;
  }

  if (m_encCfg->m_Imv4PelFast && m_encCfg->m_ImvMode && encTestmode.type == ETM_INTER_ME)
  {
    const auto amvrSearchMode = encTestmode.getAmvrSearchMode();

    if (amvrSearchMode == EncTestMode::AmvrSearchMode::FULL_PEL)
    {
      if (tempCS->cost < cuECtx.bestCostImv)
      {
        cuECtx.bestCostImv = tempCS->cost;
      }
    }
    else if (amvrSearchMode == EncTestMode::AmvrSearchMode::NONE)
    {
      if (tempCS->cost < cuECtx.bestCostNoImv)
      {
        cuECtx.bestCostNoImv = tempCS->cost;
      }
    }
  }

  if (encTestmode.type == ETM_MERGE_SKIP && tempCS->cus.front()->geoFlag && tempCS->cost < cuECtx.bestCostGPM)
  {
    cuECtx.bestCostGPM = tempCS->cost;
  }

  if (encTestmode.type == ETM_INTER_ME && (encTestmode.opts & ETO_LIC))
  {
    if (tempCS->cost < cuECtx.bestCostLIC)
    {
      cuECtx.bestCostLIC = tempCS->cost;
    }
  }

  if (encTestmode.type == ETM_SPLIT_QT)
  {
    int maxQtD = 0;
    for (const auto &cu: tempCS->cus)
    {
      maxQtD = std::max<int>(maxQtD, cu->qtDepth);
    }
    cuECtx.maxQtSubDepth = maxQtD;
  }
  if (!m_encCfg->m_disableFastDecisionTT)
  {
    int maxMtD = partitioner.currPartLevel().maxMttDepth + partitioner.currImplicitBtDepth;

    if (encTestmode.type == ETM_SPLIT_BT_H)
    {
      if (tempCS->cus.size() > 2)
      {
        int hD2  = tempCS->area.block(partitioner.chType).height / 2;
        int cu1H = tempCS->cus.front()->block(partitioner.chType).height;
        int cu2H = tempCS->cus.back()->block(partitioner.chType).height;

        cuECtx.doTriHorzSplit = cu1H < hD2 || cu2H < hD2 || partitioner.currMtDepth + 1 == maxMtD;
      }
    }
    else if (encTestmode.type == ETM_SPLIT_BT_V)
    {
      if (tempCS->cus.size() > 2)
      {
        int wD2  = tempCS->area.block(partitioner.chType).width / 2;
        int cu1W = tempCS->cus.front()->block(partitioner.chType).width;
        int cu2W = tempCS->cus.back()->block(partitioner.chType).width;

        cuECtx.doTriVertSplit = cu1W < wD2 || cu2W < wD2 || partitioner.currMtDepth + 1 == maxMtD;
      }
    }
  }

  if (encTestmode.type == ETM_SPLIT_BT_V || encTestmode.type == ETM_SPLIT_BT_H || encTestmode.type == ETM_SPLIT_QT)
  {
    bool isAllSkip = true;

    for (const auto &cu: tempCS->cus)
    {
      isAllSkip &= cu->skip;
    }

    if (isAllSkip)
    {
      cuECtx.doMoreSplits -= encTestmode.type == ETM_SPLIT_QT ? 2 : 1;
      if (cuECtx.doMoreSplits < 0)
      {
        cuECtx.doMoreSplits = 0;
      }
    }
  }
  // for now just a simple decision based on RD-cost or choose tempCS if bestCS is not yet coded
  if (tempCS->cost != MAX_DOUBLE &&
      (!cuECtx.bestCS ||
       ((tempCS->cost + (useEncDbOpt ? tempCS->costDbOffset : 0)) <
        (cuECtx.bestCS->cost + (useEncDbOpt ? cuECtx.bestCS->costDbOffset : 0)))))
  {
    cuECtx.bestCS = tempCS;
    cuECtx.bestCU = tempCS->cus[0];
    cuECtx.bestTU = cuECtx.bestCU->firstTU;

    return true;
  }

  return false;
}
