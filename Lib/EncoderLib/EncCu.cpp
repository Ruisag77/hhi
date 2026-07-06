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

/** \file     EncCu.cpp
    \brief    Coding Unit (CU) encoder class
*/

#include "EncCu.h"

#include "EncLib.h"
#include "Analyze.h"
#include "AQp.h"
#include "EncCfg.h"

#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"
#include "MCTS.h"

#include "CommonLib/dtrace_buffer.h"

#include <stdio.h>
#include <cmath>
#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <map>
#include <mutex>
#include <string>
#include <utility>

#if defined(_WIN32)
#include <direct.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#endif

//! \ingroup EncoderLib
//! \{

namespace
{
static const char *FINAL_PLANAR_LOG_DIR = "div2k_planar";

struct FinalPlanarLogEntry
{
  FILE *fp { nullptr };
  bool  headerWritten { false };
};

struct FinalPlanarNeighborLog
{
  int avail { 0 };
  int predMode { -1 };
  int modeClass { 0 };
  int intraDirLuma { -1 };
  int width { -1 };
  int height { -1 };
  int qp { -1 };
  int depth { -1 };
  int qtDepth { -1 };
  int btDepth { -1 };
  int mtDepth { -1 };
  int sameSize { 0 };
  int isPlanar { 0 };
  int isDc { 0 };
  int isAngular { 0 };
  int isSpecial { 0 };
  int mip { 0 };
  int eip { 0 };
  int sgpm { 0 };
  int dimd { 0 };
  int timd { 0 };
  int timdSad { 0 };
  int obic { 0 };
  int bdpcm { 0 };
  int multiRefIdx { 0 };
  int plDir { -1 };
};

struct FinalPlanarNeighborStats
{
  int numAvail { 0 };
  int numIntra { 0 };
  int numPlanar { 0 };
  int numDc { 0 };
  int numAngular { 0 };
  int numSpecial { 0 };
  int numInter { 0 };
  int numIbc { 0 };
  int numPlt { 0 };
  int numSameSize { 0 };
  int numSameQp { 0 };
  int numSmaller { 0 };
  int numLarger { 0 };
};

std::mutex                                                g_finalPlanarLogMutex;
std::map<std::pair<std::string, int>, FinalPlanarLogEntry> g_finalPlanarLogs;
bool                                                      g_finalPlanarLogDirCreated = false;

bool createFinalPlanarLogDir()
{
  if (g_finalPlanarLogDirCreated)
  {
    return true;
  }

#if defined(_WIN32)
  const int ret = _mkdir(FINAL_PLANAR_LOG_DIR);
#else
  const int ret = mkdir(FINAL_PLANAR_LOG_DIR, 0777);
#endif
  if (ret == 0 || errno == EEXIST)
  {
    g_finalPlanarLogDirCreated = true;
    return true;
  }

  std::fprintf(stderr, "Failed to create %s directory for final Planar log\n", FINAL_PLANAR_LOG_DIR);
  return false;
}

std::string getFinalPlanarLogSequenceName(const std::string &inputFileName)
{
  const size_t slashPos = inputFileName.find_last_of("/\\");
  const size_t namePos  = slashPos == std::string::npos ? 0 : slashPos + 1;
  std::string  name     = inputFileName.substr(namePos);
  const size_t dotPos   = name.find_last_of('.');
  if (dotPos != std::string::npos)
  {
    name = name.substr(0, dotPos);
  }
  return name.empty() ? std::string("sequence") : name;
}

FinalPlanarLogEntry *getOrCreateFinalPlanarLogEntry(const std::string &sequenceName, const int qp)
{
  if (!createFinalPlanarLogDir())
  {
    return nullptr;
  }

  const auto key = std::make_pair(sequenceName, qp);
  auto       it  = g_finalPlanarLogs.find(key);
  if (it != g_finalPlanarLogs.end())
  {
    return &it->second;
  }

  const std::string path =
    std::string(FINAL_PLANAR_LOG_DIR) + "/" + sequenceName + "_qp" + std::to_string(qp) + ".log";
  FinalPlanarLogEntry entry;
  entry.fp = std::fopen(path.c_str(), "w");
  if (!entry.fp)
  {
    std::fprintf(stderr, "Failed to open final Planar log file: %s\n", path.c_str());
    return nullptr;
  }

  auto inserted = g_finalPlanarLogs.emplace(key, entry);
  return &inserted.first->second;
}

bool isSpecialIntraModeForPlanarLog(const CodingUnit &cu)
{
  return cu.mipFlag || cu.eipFlag || cu.sgpm || cu.dimdFlag || cu.timdFlag || cu.timdSadFlag || cu.obicFlag ||
         cu.bdpcmMode[0] != BdpcmMode::NONE || cu.multiRefIdx != 0;
}

int classifyFinalPlanarNeighborMode(const CodingUnit *neighborCu)
{
  if (!neighborCu || !CU::isIntra(*neighborCu))
  {
    return 0;
  }
  if (isSpecialIntraModeForPlanarLog(*neighborCu))
  {
    return 4;
  }

  const uint32_t intraDir = neighborCu->intraDir[ChannelType::LUMA];
  if (intraDir == PLANAR_IDX)
  {
    return 1;
  }
  if (intraDir == DC_IDX)
  {
    return 2;
  }
  return intraDir < NUM_LUMA_MODE ? 3 : 4;
}

FinalPlanarNeighborLog makeFinalPlanarNeighborLog(const CodingUnit *neighborCu, const CompArea &currArea)
{
  FinalPlanarNeighborLog log;
  if (!neighborCu || !neighborCu->Y().valid())
  {
    return log;
  }

  const CompArea &area = neighborCu->block(COMP_Y);
  log.avail            = 1;
  log.predMode         = static_cast<int>(neighborCu->predMode);
  log.modeClass        = classifyFinalPlanarNeighborMode(neighborCu);
  log.width            = static_cast<int>(area.width);
  log.height           = static_cast<int>(area.height);
  log.qp               = neighborCu->qp;
  log.depth            = neighborCu->depth;
  log.qtDepth          = neighborCu->qtDepth;
  log.btDepth          = neighborCu->btDepth;
  log.mtDepth          = neighborCu->mtDepth;
  log.sameSize         = area.width == currArea.width && area.height == currArea.height ? 1 : 0;

  if (CU::isIntra(*neighborCu))
  {
    log.intraDirLuma = static_cast<int>(neighborCu->intraDir[ChannelType::LUMA]);
    log.isSpecial    = isSpecialIntraModeForPlanarLog(*neighborCu) ? 1 : 0;
    log.isPlanar     = log.modeClass == 1 ? 1 : 0;
    log.isDc         = log.modeClass == 2 ? 1 : 0;
    log.isAngular    = log.modeClass == 3 ? 1 : 0;
    log.mip          = neighborCu->mipFlag ? 1 : 0;
    log.eip          = neighborCu->eipFlag ? 1 : 0;
    log.sgpm         = neighborCu->sgpm ? 1 : 0;
    log.dimd         = neighborCu->dimdFlag ? 1 : 0;
    log.timd         = neighborCu->timdFlag ? 1 : 0;
    log.timdSad      = neighborCu->timdSadFlag ? 1 : 0;
    log.obic         = neighborCu->obicFlag ? 1 : 0;
    log.bdpcm        = neighborCu->bdpcmMode[0] != BdpcmMode::NONE ? 1 : 0;
    log.multiRefIdx  = neighborCu->multiRefIdx;
    log.plDir        = static_cast<int>(neighborCu->plDir);
  }

  return log;
}

void addFinalPlanarNeighborStats(FinalPlanarNeighborStats &stats, const FinalPlanarNeighborLog &log,
                                 const CompArea &currArea, const int currQp)
{
  if (!log.avail)
  {
    return;
  }

  stats.numAvail++;
  stats.numPlanar += log.isPlanar;
  stats.numDc += log.isDc;
  stats.numAngular += log.isAngular;
  stats.numSpecial += log.isSpecial;
  stats.numSameSize += log.sameSize;
  stats.numSameQp += log.qp == currQp ? 1 : 0;

  const int currAreaSize  = static_cast<int>(currArea.width * currArea.height);
  const int neighAreaSize = log.width * log.height;
  stats.numSmaller += neighAreaSize < currAreaSize ? 1 : 0;
  stats.numLarger += neighAreaSize > currAreaSize ? 1 : 0;

  switch (log.predMode)
  {
  case MODE_INTRA:
    stats.numIntra++;
    break;
  case MODE_INTER:
    stats.numInter++;
    break;
  case MODE_IBC:
    stats.numIbc++;
    break;
  case MODE_PLT:
    stats.numPlt++;
    break;
  default:
    break;
  }
}

void writeFinalPlanarNeighborHeader(FILE *fp, const char *prefix)
{
  std::fprintf(fp,
               " %s_avail %s_pred_mode %s_mode_class %s_intra_dir_luma %s_width %s_height %s_qp %s_depth"
               " %s_qt_depth %s_bt_depth %s_mt_depth %s_same_size %s_is_planar %s_is_dc %s_is_angular"
               " %s_is_special %s_mip %s_eip %s_sgpm %s_dimd %s_timd %s_timd_sad %s_obic %s_bdpcm"
               " %s_multi_ref_idx %s_pl_dir",
               prefix, prefix, prefix, prefix, prefix, prefix, prefix, prefix, prefix, prefix, prefix, prefix,
               prefix, prefix, prefix, prefix, prefix, prefix, prefix, prefix, prefix, prefix, prefix, prefix,
               prefix, prefix);
}

void writeFinalPlanarNeighborValues(FILE *fp, const FinalPlanarNeighborLog &log)
{
  std::fprintf(fp,
               " %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
               log.avail, log.predMode, log.modeClass, log.intraDirLuma, log.width, log.height, log.qp, log.depth,
               log.qtDepth, log.btDepth, log.mtDepth, log.sameSize, log.isPlanar, log.isDc, log.isAngular,
               log.isSpecial, log.mip, log.eip, log.sgpm, log.dimd, log.timd, log.timdSad, log.obic, log.bdpcm,
               log.multiRefIdx, log.plDir);
}

bool isFinalBestPlanarBlock(const CodingUnit &cu)
{
  return CU::isIntra(cu) && !isSpecialIntraModeForPlanarLog(cu) && cu.intraDir[ChannelType::LUMA] == PLANAR_IDX;
}

void logFinalPlanarBlockSample(const CodingUnit &cu, const EncCfg *encCfg)
{
  if (!encCfg || !cu.cs || !cu.slice || !cu.Y().valid())
  {
    return;
  }

  const CompArea &area = cu.block(COMP_Y);
  const Position  a0   = area.bottomLeft().offset(-1, 1);
  const Position  a1   = area.bottomLeft().offset(-1, 0);
  const Position  b0   = area.topRight().offset(1, -1);
  const Position  b1   = area.topRight().offset(0, -1);
  const Position  b2   = area.topLeft().offset(-1, -1);
  const Position  b3   = area.topLeft().offset(0, -1);

  const CodingUnit *cuA0 = cu.cs->getCURestricted(a0, cu, ChannelType::LUMA);
  const CodingUnit *cuA1 = cu.cs->getCURestricted(a1, cu, ChannelType::LUMA);
  const CodingUnit *cuB0 = cu.cs->getCURestricted(b0, cu, ChannelType::LUMA);
  const CodingUnit *cuB1 = cu.cs->getCURestricted(b1, cu, ChannelType::LUMA);
  const CodingUnit *cuB2 = cu.cs->getCURestricted(b2, cu, ChannelType::LUMA);
  const CodingUnit *cuB3 = cu.cs->getCURestricted(b3, cu, ChannelType::LUMA);

  const FinalPlanarNeighborLog nA0 = makeFinalPlanarNeighborLog(cuA0, area);
  const FinalPlanarNeighborLog nA1 = makeFinalPlanarNeighborLog(cuA1, area);
  const FinalPlanarNeighborLog nB0 = makeFinalPlanarNeighborLog(cuB0, area);
  const FinalPlanarNeighborLog nB1 = makeFinalPlanarNeighborLog(cuB1, area);
  const FinalPlanarNeighborLog nB2 = makeFinalPlanarNeighborLog(cuB2, area);
  const FinalPlanarNeighborLog nB3 = makeFinalPlanarNeighborLog(cuB3, area);

  FinalPlanarNeighborStats neighborStats;
  addFinalPlanarNeighborStats(neighborStats, nA0, area, cu.qp);
  addFinalPlanarNeighborStats(neighborStats, nA1, area, cu.qp);
  addFinalPlanarNeighborStats(neighborStats, nB0, area, cu.qp);
  addFinalPlanarNeighborStats(neighborStats, nB1, area, cu.qp);
  addFinalPlanarNeighborStats(neighborStats, nB2, area, cu.qp);
  addFinalPlanarNeighborStats(neighborStats, nB3, area, cu.qp);

  const PreCalcValues *pcv              = cu.cs->pcv;
  const int            log2Width        = floorLog2(area.width);
  const int            log2Height       = floorLog2(area.height);
  const int            areaSize         = static_cast<int>(area.width * area.height);
  const int            aspectRatioX100  = area.height ? static_cast<int>((area.width * 100) / area.height) : 0;
  const int            ctuX             = pcv ? area.x >> pcv->maxCUWidthLog2 : 0;
  const int            ctuY             = pcv ? area.y >> pcv->maxCUHeightLog2 : 0;
  const int            xInCtu           = pcv ? area.x & pcv->maxCUWidthMask : area.x;
  const int            yInCtu           = pcv ? area.y & pcv->maxCUHeightMask : area.y;
  const int            isPicLeft        = area.x == 0 ? 1 : 0;
  const int            isPicTop         = area.y == 0 ? 1 : 0;
  const int            isPicRight       = pcv && area.x + static_cast<int>(area.width) >= static_cast<int>(pcv->lumaWidth);
  const int            isPicBottom      = pcv && area.y + static_cast<int>(area.height) >= static_cast<int>(pcv->lumaHeight);

  std::lock_guard<std::mutex> lock(g_finalPlanarLogMutex);

  const std::string    sequenceName = getFinalPlanarLogSequenceName(encCfg->m_inputFileName);
  FinalPlanarLogEntry *entry        = getOrCreateFinalPlanarLogEntry(sequenceName, encCfg->m_iQP);
  if (!entry || !entry->fp)
  {
    return;
  }

  if (!entry->headerWritten)
  {
    std::fprintf(entry->fp,
                 "# pred_mode: 0=inter, 1=intra, 2=ibc, 3=plt, -1=unavailable\n");
    std::fprintf(entry->fp,
                 "# mode_class: 0=unavailable_or_non_intra, 1=planar, 2=dc, 3=angular, 4=special_or_other\n");
    std::fprintf(entry->fp, "# pl_dir: -1=unavailable_or_non_intra, 0=no_dir, 1=horizontal, 2=vertical\n");
    std::fprintf(entry->fp,
                 "poc x y width height log2_width log2_height area aspect_ratio_x100 ctu_x ctu_y x_in_ctu"
                 " y_in_ctu is_pic_left is_pic_top is_pic_right is_pic_bottom depth qt_depth bt_depth mt_depth"
                 " split_series block_qp file_qp slice_type label_is_planar");
    writeFinalPlanarNeighborHeader(entry->fp, "A0");
    writeFinalPlanarNeighborHeader(entry->fp, "A1");
    writeFinalPlanarNeighborHeader(entry->fp, "B0");
    writeFinalPlanarNeighborHeader(entry->fp, "B1");
    writeFinalPlanarNeighborHeader(entry->fp, "B2");
    writeFinalPlanarNeighborHeader(entry->fp, "B3");
    std::fprintf(entry->fp,
                 " num_avail_neighbors num_intra_neighbors num_planar_neighbors num_dc_neighbors"
                 " num_angular_neighbors num_special_neighbors num_inter_neighbors num_ibc_neighbors"
                 " num_plt_neighbors num_same_size_neighbors num_same_qp_neighbors num_smaller_neighbors"
                 " num_larger_neighbors\n");
    entry->headerWritten = true;
  }

  std::fprintf(entry->fp,
               "%d %d %d %u %u %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %llu %d %d %d %d",
               cu.slice->m_poc, area.x, area.y, area.width, area.height, log2Width, log2Height, areaSize,
               aspectRatioX100, ctuX, ctuY, xInCtu, yInCtu, isPicLeft, isPicTop, isPicRight ? 1 : 0,
               isPicBottom ? 1 : 0, cu.depth, cu.qtDepth, cu.btDepth, cu.mtDepth,
               static_cast<unsigned long long>(cu.splitSeries), cu.qp, encCfg->m_iQP,
               static_cast<int>(cu.slice->m_eSliceType), isFinalBestPlanarBlock(cu) ? 1 : 0);
  writeFinalPlanarNeighborValues(entry->fp, nA0);
  writeFinalPlanarNeighborValues(entry->fp, nA1);
  writeFinalPlanarNeighborValues(entry->fp, nB0);
  writeFinalPlanarNeighborValues(entry->fp, nB1);
  writeFinalPlanarNeighborValues(entry->fp, nB2);
  writeFinalPlanarNeighborValues(entry->fp, nB3);
  std::fprintf(entry->fp, " %d %d %d %d %d %d %d %d %d %d %d %d %d\n", neighborStats.numAvail,
               neighborStats.numIntra, neighborStats.numPlanar, neighborStats.numDc, neighborStats.numAngular,
               neighborStats.numSpecial, neighborStats.numInter, neighborStats.numIbc, neighborStats.numPlt,
               neighborStats.numSameSize, neighborStats.numSameQp, neighborStats.numSmaller,
               neighborStats.numLarger);
  std::fflush(entry->fp);
}

void logFinalPlanarCodingStructure(const CodingStructure &finalCS, const CodingStructure &pictureCS, const EncCfg *encCfg)
{
  for (const CodingUnit *cu: finalCS.cus)
  {
    if (cu && cu->Y().valid())
    {
      const CodingUnit *pictureCu = pictureCS.getCU(cu->Y().pos(), ChannelType::LUMA);
      logFinalPlanarBlockSample(pictureCu ? *pictureCu : *cu, encCfg);
    }
  }
}
}   // namespace

// ====================================================================================================================

EncCu::EncCu() {}

void EncCu::create(const EncCfg *encCfg)
{
  unsigned     uiMaxWidth   = encCfg->m_CTUSize;
  unsigned     uiMaxHeight  = encCfg->m_CTUSize;
  ChromaFormat chromaFormat = encCfg->m_chromaFormatIdc;

  unsigned numWidths  = gp_sizeIdxInfo->numWidths();
  unsigned numHeights = gp_sizeIdxInfo->numHeights();

  m_pelUnitBufPool.initPelUnitBufPool(chromaFormat, uiMaxWidth, uiMaxHeight);
  m_mergeItemList.init(encCfg->m_maxMergeRdCandNumTotal * 2, chromaFormat, uiMaxWidth, uiMaxHeight);
  m_mergeItemListGeoMMVD.init(encCfg->m_maxMergeRdCandNumTotal, chromaFormat, uiMaxWidth, uiMaxHeight);

  const int ctuSize = encCfg->m_CTUSize;

  for (int i = 0; i < maxCuDepth; i++)
  {
    Area area = Area(0, 0, ctuSize >> (i >> 1), ctuSize >> ((i + 1) >> 1));

    if (area.width < (1 << MIN_CU_LOG2) || area.height < (1 << MIN_CU_LOG2))
    {
      m_pTempCS[i] = m_pBestCS[i] = m_obmcCS[i] = nullptr;
      continue;
    }

    m_pTempCS[i] = new CodingStructure(m_unitPool);
    m_pBestCS[i] = new CodingStructure(m_unitPool);
    m_obmcCS[i]  = new CodingStructure(m_unitPool);

    m_pTempCS[i]->create(chromaFormat, area, false, (bool)encCfg->m_PLTMode);
    m_pBestCS[i]->create(chromaFormat, area, false, (bool)encCfg->m_PLTMode);
    m_obmcCS[i]->create(chromaFormat, area, false, (bool)encCfg->m_PLTMode);
  }

  m_predWoObmcTmp = new PelStorage;
  m_predWoObmcTmp->create(UnitArea(chromaFormat, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
  m_predWoObmcBest = new PelStorage;
  m_predWoObmcBest->create(UnitArea(chromaFormat, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));

  for (unsigned ui = 0; ui < MRG_MAX_NUM_CANDS; ui++)
  {
    m_acMergeTmpBuffer[ui].create(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight));
  }

  m_cuChromaQpOffsetIdxPlus1 = 0;

  unsigned maxDepth = numWidths + numHeights;

  int sourceWidth     = encCfg->m_sourceWidth;
  int sourceHeight    = encCfg->m_sourceHeight;
  m_fastGpmMmvdSearch = (((encCfg->m_intraPeriod > 0) && ((sourceWidth * sourceHeight) <= (1920 * 1080))) ||
                         ((encCfg->m_intraPeriod < 0) && ((sourceWidth * sourceHeight) >= (1280 * 720)))) &&
    !encCfg->m_ibcMode;
  m_includeMoreMMVDCandFirstPass =
    ((encCfg->m_intraPeriod > 0) || ((encCfg->m_intraPeriod < 0) && m_fastGpmMmvdSearch));
  m_maxNumGPMDirFirstPass = ((encCfg->m_intraPeriod < 0) ? 50 : (m_fastGpmMmvdSearch ? 36 : 64));

  m_ctxBuffer.resize(maxDepth);
  m_CurrCtx = 0;
}

void EncCu::destroy()
{
  for (int i = 0; i < maxCuDepth; i++)
  {
    if (m_pTempCS[i])
    {
      m_pTempCS[i]->destroy();
      delete m_pTempCS[i];
      m_pTempCS[i] = nullptr;
    }

    if (m_pBestCS[i])
    {
      m_pBestCS[i]->destroy();
      delete m_pBestCS[i];
      m_pBestCS[i] = nullptr;
    }

    if (m_obmcCS[i])
    {
      m_obmcCS[i]->destroy();
      delete m_obmcCS[i];
      m_obmcCS[i] = nullptr;
    }
  }

#if REUSE_CU_RESULTS
  if (m_tmpStorageCtu)
  {
    m_tmpStorageCtu->destroy();
    delete m_tmpStorageCtu;
    m_tmpStorageCtu = nullptr;
  }
#endif

  for (unsigned ui = 0; ui < MRG_MAX_NUM_CANDS; ui++)
  {
    m_acMergeTmpBuffer[ui].destroy();
  }
  m_predWoObmcTmp->destroy();
  delete m_predWoObmcTmp;
  m_predWoObmcTmp = nullptr;
  m_predWoObmcBest->destroy();
  delete m_predWoObmcBest;
  m_predWoObmcBest = nullptr;

  m_modeCtrl = nullptr;
}

EncCu::~EncCu() {}

/** \param    pcEncLib      pointer of encoder class
 */
void EncCu::init(EncLib *pcEncLib, EncModeCtrl *pcEncModeCtrl, const SPS &sps)
{
  m_encCfg         = &pcEncLib->m_encCfg;
  m_modeCtrl       = pcEncModeCtrl;
  m_pcIntraSearch  = pcEncLib->getIntraSearch();
  m_pcInterSearch  = pcEncLib->getInterSearch();
  m_pcTrQuant      = pcEncLib->getTrQuant();
  m_pcRdCost       = pcEncLib->getRdCost();
  m_CABACEstimator = pcEncLib->getCABACEncoder()->getCABACEstimator(&sps);
  m_CABACEstimator->setEncCu(this);
  m_ctxPool          = pcEncLib->getCtxCache();
  m_pcRateCtrl       = pcEncLib->getRateCtrl();
  m_pcSliceEncoder   = pcEncLib->getSliceEncoder();
  m_deblockingFilter = pcEncLib->getDeblockingFilter();
  m_bilateralFilter  = pcEncLib->getBilateralFilter();
  m_geoCostList.init((m_encCfg->m_maxNumGeoCand + 1) * (GPM_EXT_MMVD_MAX_REFINE_NUM2));
  m_AFFBestSATDCost = MAX_DOUBLE;

  DecCu::init(m_pcTrQuant, m_pcIntraSearch, m_pcInterSearch);

  m_pcGOPEncoder = pcEncLib->getGOPEncoder();
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void EncCu::compressCtu(CodingStructure &cs, const UnitArea &area, const unsigned ctuRsAddr,
                        const EnumArray<int, ChannelType> &prevQP, const EnumArray<int, ChannelType> &currQP)
{
  m_modeCtrl->initCTUEncoding(*cs.slice);

  cs.slice->m_mapPltCost[0].clear();
  cs.slice->m_mapPltCost[1].clear();
  // init the partitioning manager
  QTBTPartitioner partitioner;
  partitioner.initCtu(area, ChannelType::LUMA, cs);
  if (m_encCfg->m_ibcMode)
  {
    if (area.lx() == 0 && area.ly() == 0)
    {
      m_pcInterSearch->resetIbcSearch();
    }
    m_pcInterSearch->resetCtuRecord();
    m_ctuIbcSearchRangeX = m_encCfg->m_ibcLocalSearchRangeX;
    m_ctuIbcSearchRangeY = m_encCfg->m_ibcLocalSearchRangeY;
  }
  if (m_encCfg->m_ibcMode && m_encCfg->m_ibcHashSearch &&
      (m_encCfg->m_ibcFastMethod & IBC_FAST_METHOD_ADAPTIVE_SEARCHRANGE))
  {
    const int hashHitRatio = m_ibcHashMap.getHashHitRatio(area.Y()); // in percent
    if (hashHitRatio < 5) // 5%
    {
      m_ctuIbcSearchRangeX >>= 1;
      m_ctuIbcSearchRangeY >>= 1;
    }
    if (cs.slice->m_numRefIdx[RPL0] > 0)
    {
      m_ctuIbcSearchRangeX >>= 1;
      m_ctuIbcSearchRangeY >>= 1;
    }
  }
  // init current context pointer
  m_CurrCtx = m_ctxBuffer.data();

  CodingStructure *tempCS = m_pTempCS[0];
  CodingStructure *bestCS = m_pBestCS[0];

  cs.initSubStructure(*tempCS, partitioner.chType, partitioner.currArea(), false);
  cs.initSubStructure(*bestCS, partitioner.chType, partitioner.currArea(), false);
  tempCS->currQP[ChannelType::LUMA] = bestCS->currQP[ChannelType::LUMA] = tempCS->baseQP = bestCS->baseQP =
    currQP[ChannelType::LUMA];
  tempCS->prevQP[ChannelType::LUMA] = bestCS->prevQP[ChannelType::LUMA] = prevQP[ChannelType::LUMA];

  if (m_encCfg->m_obmc)
  {
    CodingStructure *obmcCS = m_obmcCS[0];
    cs.initSubStructure(*obmcCS, partitioner.chType, partitioner.currArea(), false);
    obmcCS->currQP[ChannelType::LUMA] = obmcCS->baseQP = currQP[ChannelType::LUMA];
    obmcCS->prevQP[ChannelType::LUMA]                  = prevQP[ChannelType::LUMA];
  }

  xCompressCU(tempCS, bestCS, partitioner);
  cs.slice->m_mapPltCost[0].clear();
  cs.slice->m_mapPltCost[1].clear();
  // all signals were already copied during compression if the CTU was split - at this point only the structures are
  // copied to the top level CS
  const bool copyUnsplitCTUSignals = bestCS->cus.size() == 1;
  cs.useSubStructure(*bestCS, partitioner.chType, CS::getArea(*bestCS, area, partitioner.chType), copyUnsplitCTUSignals,
                     false, false, copyUnsplitCTUSignals, true);
  logFinalPlanarCodingStructure(*bestCS, cs, m_encCfg);

  if (CS::isDualITree(cs) && isChromaEnabled(cs.pcv->chrFormat))
  {
    m_CABACEstimator->getCtx() = m_CurrCtx->start;

    partitioner.initCtu(area, ChannelType::CHROMA, cs);

    cs.initSubStructure(*tempCS, partitioner.chType, partitioner.currArea(), false);
    cs.initSubStructure(*bestCS, partitioner.chType, partitioner.currArea(), false);
    tempCS->currQP[ChannelType::CHROMA] = bestCS->currQP[ChannelType::CHROMA] = tempCS->baseQP = bestCS->baseQP =
      currQP[ChannelType::CHROMA];
    tempCS->prevQP[ChannelType::CHROMA] = bestCS->prevQP[ChannelType::CHROMA] = prevQP[ChannelType::CHROMA];

    xCompressCU(tempCS, bestCS, partitioner);

    const bool copyUnsplitCTUSignals = bestCS->cus.size() == 1;
    cs.useSubStructure(*bestCS, partitioner.chType, CS::getArea(*bestCS, area, partitioner.chType),
                       copyUnsplitCTUSignals, false, false, copyUnsplitCTUSignals, true);
  }

  if (m_encCfg->m_RCEnableRateControl)
  {
    (m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr)).m_actualMSE =
      (double)bestCS->dist / (double)m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr).m_numberOfPixel;
  }
  // reset context states and uninit context pointer
  m_CABACEstimator->getCtx() = m_CurrCtx->start;
  m_CurrCtx                  = 0;

  // Ensure that a coding was found
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  CHECK(bestCS->cus.empty(), "No possible encoding found");
  CHECK(bestCS->cus[0]->predMode == NUMBER_OF_PREDICTION_MODES, "No possible encoding found");
  CHECK(bestCS->cost == MAX_DOUBLE, "No possible encoding found");
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

static int xCalcHADs8x8_ISlice(const Pel *piOrg, const ptrdiff_t strideOrg)
{
  int k, i, j, jj;
  int diff[64], m1[8][8], m2[8][8], m3[8][8], iSumHad = 0;

  for (k = 0; k < 64; k += 8)
  {
    diff[k + 0] = piOrg[0];
    diff[k + 1] = piOrg[1];
    diff[k + 2] = piOrg[2];
    diff[k + 3] = piOrg[3];
    diff[k + 4] = piOrg[4];
    diff[k + 5] = piOrg[5];
    diff[k + 6] = piOrg[6];
    diff[k + 7] = piOrg[7];

    piOrg += strideOrg;
  }

  // horizontal
  for (j = 0; j < 8; j++)
  {
    jj       = j << 3;
    m2[j][0] = diff[jj] + diff[jj + 4];
    m2[j][1] = diff[jj + 1] + diff[jj + 5];
    m2[j][2] = diff[jj + 2] + diff[jj + 6];
    m2[j][3] = diff[jj + 3] + diff[jj + 7];
    m2[j][4] = diff[jj] - diff[jj + 4];
    m2[j][5] = diff[jj + 1] - diff[jj + 5];
    m2[j][6] = diff[jj + 2] - diff[jj + 6];
    m2[j][7] = diff[jj + 3] - diff[jj + 7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  // vertical
  for (i = 0; i < 8; i++)
  {
    m3[0][i] = m2[0][i] + m2[4][i];
    m3[1][i] = m2[1][i] + m2[5][i];
    m3[2][i] = m2[2][i] + m2[6][i];
    m3[3][i] = m2[3][i] + m2[7][i];
    m3[4][i] = m2[0][i] - m2[4][i];
    m3[5][i] = m2[1][i] - m2[5][i];
    m3[6][i] = m2[2][i] - m2[6][i];
    m3[7][i] = m2[3][i] - m2[7][i];

    m1[0][i] = m3[0][i] + m3[2][i];
    m1[1][i] = m3[1][i] + m3[3][i];
    m1[2][i] = m3[0][i] - m3[2][i];
    m1[3][i] = m3[1][i] - m3[3][i];
    m1[4][i] = m3[4][i] + m3[6][i];
    m1[5][i] = m3[5][i] + m3[7][i];
    m1[6][i] = m3[4][i] - m3[6][i];
    m1[7][i] = m3[5][i] - m3[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j++)
    {
      iSumHad += abs(m2[i][j]);
    }
  }
  iSumHad -= abs(m2[0][0]);
  iSumHad = (iSumHad + 2) >> 2;
  return (iSumHad);
}

int EncCu::updateCtuDataISlice(const CPelBuf buf)
{
  int        xBl, yBl;
  const int  iBlkSize    = 8;
  const Pel *pOrgInit    = buf.buf;
  ptrdiff_t  iStrideOrig = buf.stride;

  int iSumHad = 0;
  for (yBl = 0; (yBl + iBlkSize) <= buf.height; yBl += iBlkSize)
  {
    for (xBl = 0; (xBl + iBlkSize) <= buf.width; xBl += iBlkSize)
    {
      const Pel *pOrg = pOrgInit + iStrideOrig * yBl + xBl;
      iSumHad += xCalcHADs8x8_ISlice(pOrg, iStrideOrig);
    }
  }
  return (iSumHad);
}

bool EncCu::xCheckBestMode(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner,
                           const EncTestMode &encTestMode, bool encDbOpt)
{

  bool bestChanged = false;

  if (!tempCS->cus.empty())
  {
    if (tempCS->cus.size() == 1)
    {
      const CodingUnit &cu = *tempCS->cus.front();
      CHECK(cu.skip && !cu.mergeFlag, "Skip flag without a merge flag is not allowed!");
#if WCG_EXT
      DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda(true));
#else
      DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());
#endif
    }

#if WCG_EXT
    DTRACE_BEST_MODE(tempCS, bestCS, m_pcRdCost->getLambda(true), encDbOpt);
#else
    DTRACE_BEST_MODE(tempCS, bestCS, m_pcRdCost->getLambda(), encDbOpt);
#endif

    if (m_modeCtrl->useModeResult(encTestMode, tempCS, partitioner, encDbOpt))
    {
      std::swap(tempCS, bestCS);
      // store temp best CI for next CU coding
      m_CurrCtx->best = m_CABACEstimator->getCtx();
      bestChanged     = true;
    }
  }

  // reset context states
  m_CABACEstimator->getCtx() = m_CurrCtx->start;

  return bestChanged;
}

void EncCu::prepare(CodingStructure *&tempCS, Partitioner &partitioner, EncTestMode &currTestMode)
{
  Slice     &slice = *tempCS->slice;
  const PPS &pps   = *tempCS->pps;

  if (pps.m_useDQP && CS::isDualITree(*tempCS) && isChroma(partitioner.chType))
  {
    const Position         chromaCentral(tempCS->area.Cb().chromaPos().offset(tempCS->area.Cb().chromaSize().width >> 1,
                                                                              tempCS->area.Cb().chromaSize().height >> 1));
    const Position         lumaRefPos(chromaCentral.x << getComponentScaleX(COMP_Cb, tempCS->area.chromaFormat),
                                      chromaCentral.y << getComponentScaleY(COMP_Cb, tempCS->area.chromaFormat));
    const CodingStructure *baseCS    = tempCS->picture->m_cs;
    const CodingUnit      *colLumaCu = baseCS->getCU(lumaRefPos, ChannelType::LUMA);

    if (colLumaCu)
    {
      currTestMode.qp = colLumaCu->qp;
    }
  }

#if SHARP_LUMA_DELTA_QP || ENABLE_QPA_SUB_CTU
  if (partitioner.currQgEnable() &&
      ((m_encCfg->m_bimEnabled) ||
#if SHARP_LUMA_DELTA_QP
       (m_encCfg->m_lumaLevelToDeltaQPMapping.isEnabled()) ||
#endif
       (m_encCfg->m_smoothQPReductionEnable) ||
#if ENABLE_QPA_SUB_CTU
       (m_encCfg->m_bUsePerceptQPA && !m_encCfg->m_RCEnableRateControl && pps.m_useDQP)
#else
       false
#endif
         ))
  {
    if (currTestMode.qp >= 0)
    {
      updateLambda(&slice, currTestMode.qp + currTestMode.deltaQPForLambda,
#if WCG_EXT && ER_CHROMA_QP_WCG_PPS
                   m_encCfg->m_wcgChromaQpControl.enabled,
#endif
                   CS::isDualITree(*tempCS) || (partitioner.currDepth == 0));
    }
  }
#endif
}

void EncCu::xCompressCU(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner,
                        double maxCostAllowed)
{
  PROFILER_SCOPE(1, g_timeProfiler, P_COMPRESS_CU);
  CHECK(maxCostAllowed < 0, "Wrong value of maxCostAllowed!");

  Slice         &slice   = *tempCS->slice;
  const PPS     &pps     = *tempCS->pps;
  const SPS     &sps     = *tempCS->sps;
  const uint32_t uiLPelX = tempCS->area.Y().lumaPos().x;
  const uint32_t uiTPelY = tempCS->area.Y().lumaPos().y;

  // palette operations
  uint32_t compBegin = COMP_Y;
  uint32_t numComp   = getNumberValidComponents(tempCS->area.chromaFormat);
  bool     jointPLT  = true;
  uint8_t  bestLastPLTSize[MAX_NUM_CHANNEL_TYPE];
  Pel      bestLastPLT[MAX_NUM_COMP][MAXPLTPREDSIZE]; // store LastPLT for
  uint8_t  curLastPLTSize[MAX_NUM_CHANNEL_TYPE];
  Pel      curLastPLT[MAX_NUM_COMP][MAXPLTPREDSIZE]; // store LastPLT if no partition

  m_CABACEstimator->DeriveCtx::setNeighbourCus(*tempCS, partitioner.currArea(), partitioner.chType);

  if (sps.m_PLTMode)
  {
    if (CS::isDualITree(*tempCS))
    {
      jointPLT = false;
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

    for (int i = compBegin; i < (compBegin + numComp); i++)
    {
      CompID comID           = jointPLT ? (CompID)compBegin : ((i > 0) ? COMP_Cb : COMP_Y);
      bestLastPLTSize[comID] = 0;
      curLastPLTSize[comID]  = tempCS->prevPLT.curPLTSize[comID];
      memcpy(curLastPLT[i], tempCS->prevPLT.curPLT[i], tempCS->prevPLT.curPLTSize[comID] * sizeof(Pel));
    }
  }
  const UnitArea currCsArea = clipArea(CS::getArea(*bestCS, bestCS->area, partitioner.chType), *tempCS->picture);

  m_modeCtrl->initCULevel(partitioner, *tempCS);
  m_pcInterSearch->resetFillLicTpl();

  if (partitioner.currQtDepth == 0 && partitioner.currMtDepth == 0 && !tempCS->slice->isIntra() &&
      (sps.m_useSBT || sps.m_explicitMtsInter))
  {
    auto slsSbt    = dynamic_cast<SaveLoadEncInfoSbt *>(m_modeCtrl);
    int  maxSLSize = sps.m_useSBT ? tempCS->slice->m_sps->getMaxTbSize() : MTS_INTER_MAX_CU_SIZE;
    slsSbt->resetSaveloadSbt(maxSLSize);
  }
  m_sbtCostSave[0] = m_sbtCostSave[1] = MAX_DOUBLE;

  m_CurrCtx->start = m_CABACEstimator->getCtx();

  if (slice.m_chromaQpAdjEnabled)
  {
    // TODO M0133 : double check encoder decisions with respect to chroma QG detection and actual encode
    int lgMinCuSize =
      sps.m_log2MinCodingBlockSize +
      std::max<int>(
        0, floorLog2(sps.m_ctuSize) - sps.m_log2MinCodingBlockSize - int((slice.getCuChromaQpOffsetSubdiv() + 1) / 2));
    if (partitioner.currQgChromaEnable())
    {
      m_cuChromaQpOffsetIdxPlus1 =
        ((uiLPelX >> lgMinCuSize) + (uiTPelY >> lgMinCuSize)) % (pps.m_chromaQpOffsetListLen + 1);
    }
  }
  else
  {
    m_cuChromaQpOffsetIdxPlus1 = 0;
  }

  DTRACE_UPDATE(g_trace_ctx, std::make_pair("cux", uiLPelX));
  DTRACE_UPDATE(g_trace_ctx, std::make_pair("cuy", uiTPelY));
  DTRACE_UPDATE(g_trace_ctx, std::make_pair("cuw", tempCS->area.lwidth()));
  DTRACE_UPDATE(g_trace_ctx, std::make_pair("cuh", tempCS->area.lheight()));
  DTRACE(g_trace_ctx, D_COMMON, "@(%4d,%4d) [%2dx%2d]\n", tempCS->area.lx(), tempCS->area.ly(), tempCS->area.lwidth(),
         tempCS->area.lheight());

  m_pcInterSearch->resetSavedAffineMotion();

  m_pcInterSearch->m_fastLicCtrl.init();

  if (tempCS->slice->m_checkLdc)
  {
    m_bestBcwCost.fill(std::numeric_limits<double>::max());
    m_bestBcwIdx.fill(BCW_NUM);
  }

  if (tempCS->picture->m_cs->area.contains(tempCS->area))
  {
    xCheckNonSplitModes(tempCS, bestCS, partitioner, maxCostAllowed);
  }

  SplitSeries splitSeries = -1;
  if (sps.m_PLTMode)
  {
    for (int i = compBegin; i < (compBegin + numComp); i++)
    {
      CompID comID                      = jointPLT ? (CompID)compBegin : ((i > 0) ? COMP_Cb : COMP_Y);
      tempCS->prevPLT.curPLTSize[comID] = curLastPLTSize[comID];
      memcpy(tempCS->prevPLT.curPLT[i], curLastPLT[i], curLastPLTSize[comID] * sizeof(Pel));
    }
    splitSeries = bestCS->cus.empty() ? -1 : bestCS->cus[0]->splitSeries;
  }

  xCheckSplitModes(tempCS, bestCS, partitioner, maxCostAllowed);

  if (sps.m_PLTMode)
  {
    if (bestCS->cus.size() > 0 && splitSeries != bestCS->cus[0]->splitSeries)
    {
      splitSeries          = bestCS->cus[0]->splitSeries;
      const CodingUnit &cu = *bestCS->cus.front();
      cu.cs->prevPLT       = bestCS->prevPLT;
      for (int i = compBegin; i < (compBegin + numComp); i++)
      {
        CompID comID           = jointPLT ? (CompID)compBegin : ((i > 0) ? COMP_Cb : COMP_Y);
        bestLastPLTSize[comID] = bestCS->cus[0]->cs->prevPLT.curPLTSize[comID];
        memcpy(bestLastPLT[i], bestCS->cus[0]->cs->prevPLT.curPLT[i],
               bestCS->cus[0]->cs->prevPLT.curPLTSize[comID] * sizeof(Pel));
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////
  // Finishing CU
  if (tempCS->cost == MAX_DOUBLE && bestCS->cost == MAX_DOUBLE)
  {
    // although some coding modes were planned to be tried in RDO, no coding mode actually finished encoding due to
    // early termination thus tempCS->cost and bestCS->cost are both MAX_DOUBLE; in this case, skip the following
    // process for normal case
    m_modeCtrl->finishCULevel(partitioner);
    return;
  }

  // set context states
  m_CABACEstimator->getCtx() = m_CurrCtx->best;

  // QP from last processed CU for further processing
  bestCS->prevQP[partitioner.chType] = bestCS->cus.back()->qp;

  if ((!slice.isIntra() || slice.m_ibcFlag) && isLuma(partitioner.chType) && bestCS->cus.size() == 1 &&
      (CU::isInter(*bestCS->cus.back()) || CU::isIBC(*bestCS->cus.back())) &&
      bestCS->area.Y() == (*bestCS->cus.back()).Y())
  {
    const CodingUnit &cu = *bestCS->cus.front();

    CU::saveMotionForHmvp(cu);
  }

  const CodingUnit &cu = *bestCS->cus.front();

  if (!(cu.chromaFormat == ChromaFormat::_400 || (CS::isDualITree(*cu.cs) && cu.chType == ChannelType::LUMA)) &&
      CU::isIntra(cu) && bestCS->cus.size() == 1 && bestCS->area.Cb() == (*bestCS->cus.back()).Cb())
  {
    CU::saveModelsInHCCP(cu);
  }

  if (slice.m_sps->m_useEIP && partitioner.chType == ChannelType::LUMA && CU::isIntra(cu) && bestCS->cus.size() == 1 &&
      bestCS->area.Y() == (*bestCS->cus.back()).Y())
  {
    CU::saveModelsInHEIP(cu);
  }

  bestCS->picture->getPredBuf(currCsArea).copyFrom(bestCS->getPredBuf(currCsArea));
  bestCS->picture->getRecoBuf(currCsArea).copyFrom(bestCS->getRecoBuf(currCsArea));
  m_modeCtrl->finishCULevel(partitioner);

  cu.cs->prevPLT = bestCS->prevPLT;
  // Assert if Best prediction mode is NONE
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  CHECK(bestCS->cus.empty(), "No possible encoding found");
  CHECK(bestCS->cus[0]->predMode == NUMBER_OF_PREDICTION_MODES, "No possible encoding found");
  CHECK(bestCS->cost == MAX_DOUBLE, "No possible encoding found");

  if (sps.m_PLTMode)
  {
    if (bestCS->cus.size() == 1) // no partition
    {
      CHECK(bestCS->cus[0]->tileIdx != bestCS->pps->getTileIdx(bestCS->area.lumaPos()), "Wrong tile index!");
      if (CU::isPLT(*bestCS->cus[0]))
      {
        for (int i = compBegin; i < (compBegin + numComp); i++)
        {
          CompID comID                      = jointPLT ? (CompID)compBegin : ((i > 0) ? COMP_Cb : COMP_Y);
          bestCS->prevPLT.curPLTSize[comID] = curLastPLTSize[comID];
          memcpy(bestCS->prevPLT.curPLT[i], curLastPLT[i], curLastPLTSize[comID] * sizeof(Pel));
        }
        bestCS->reorderPrevPLT(bestCS->prevPLT, bestCS->cus[0]->curPLTSize, bestCS->cus[0]->curPLT,
                               bestCS->cus[0]->reuseflag, compBegin, numComp, jointPLT);
      }
      else
      {
        for (int i = compBegin; i < (compBegin + numComp); i++)
        {
          CompID comID                      = jointPLT ? (CompID)compBegin : ((i > 0) ? COMP_Cb : COMP_Y);
          bestCS->prevPLT.curPLTSize[comID] = curLastPLTSize[comID];
          memcpy(bestCS->prevPLT.curPLT[i], curLastPLT[i], bestCS->prevPLT.curPLTSize[comID] * sizeof(Pel));
        }
      }
    }
    else
    {
      for (int i = compBegin; i < (compBegin + numComp); i++)
      {
        CompID comID                      = jointPLT ? (CompID)compBegin : ((i > 0) ? COMP_Cb : COMP_Y);
        bestCS->prevPLT.curPLTSize[comID] = bestLastPLTSize[comID];
        memcpy(bestCS->prevPLT.curPLT[i], bestLastPLT[i], bestCS->prevPLT.curPLTSize[comID] * sizeof(Pel));
      }
    }
  }
}

#if SHARP_LUMA_DELTA_QP || ENABLE_QPA_SUB_CTU
void EncCu::updateLambda(Slice *slice, const double dQP,
#if WCG_EXT && ER_CHROMA_QP_WCG_PPS
                         const bool useWCGChromaControl,
#endif
                         const bool updateRdCostLambda)
{
#if WCG_EXT && ER_CHROMA_QP_WCG_PPS
  if (useWCGChromaControl)
  {
    const double lambda =
      m_pcSliceEncoder->initializeLambda(slice, m_pcSliceEncoder->getGopId(), slice->m_iSliceQp, (double)dQP);
    const int clippedQP = Clip3(-slice->m_sps->m_qpBDOffset[ChannelType::LUMA], MAX_QP, (int)dQP);

    m_pcSliceEncoder->setUpLambda(slice, lambda, clippedQP);
    return;
  }
#endif
  int          qp    = dQP;
  const double oldQP = (double)slice->m_iSliceQpBase;
#if ENABLE_QPA_SUB_CTU
  const double oldLambda = (m_encCfg->m_bUsePerceptQPA && !m_encCfg->m_RCEnableRateControl && slice->m_pps->m_useDQP)
    ? slice->getLambdas()[0]
    : m_pcSliceEncoder->calculateLambda(slice, m_pcSliceEncoder->getGopId(), oldQP, oldQP, qp);
#else
  const double oldLambda = m_pcSliceEncoder->calculateLambda(slice, m_pcSliceEncoder->getGopId(), oldQP, oldQP, qp);
#endif
  const double newLambda = oldLambda * pow(2.0, ((double)dQP - oldQP) / 3.0);
#if RDOQ_CHROMA_LAMBDA
  const double lambdaArray[MAX_NUM_COMP] = { newLambda / m_pcRdCost->getDistortionWeight(COMP_Y),
                                             newLambda / m_pcRdCost->getDistortionWeight(COMP_Cb),
                                             newLambda / m_pcRdCost->getDistortionWeight(COMP_Cr) };
  m_pcTrQuant->setLambdas(lambdaArray);
#else
  m_pcTrQuant->setLambda(newLambda);
#endif
  if (updateRdCostLambda)
  {
    m_pcRdCost->setLambda(newLambda, slice->m_sps->m_bitDepths);
#if WCG_EXT
    if (!m_encCfg->m_lumaLevelToDeltaQPMapping.isEnabled())
    {
      m_pcRdCost->saveUnadjustedLambda();
    }
#endif
  }
}
#endif // SHARP_LUMA_DELTA_QP || ENABLE_QPA_SUB_CTU

void EncCu::xCheckModeSplit(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner,
                            const EncTestMode &encTestMode)
{
  // palette operations
  uint32_t compBegin = COMP_Y;
  uint32_t numComp   = getNumberValidComponents(tempCS->area.chromaFormat);
  bool     jointPLT  = true;
  uint8_t  curLastPLTSize[MAX_NUM_CHANNEL_TYPE];
  Pel      curLastPLT[MAX_NUM_COMP][MAXPLTPREDSIZE]; // store LastPLT if no partition

  if (tempCS->sps->m_PLTMode)
  {
    if (CS::isDualITree(*tempCS))
    {
      jointPLT = false;
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

    for (int i = compBegin; i < (compBegin + numComp); i++)
    {
      CompID comID          = jointPLT ? (CompID)compBegin : ((i > 0) ? COMP_Cb : COMP_Y);
      curLastPLTSize[comID] = tempCS->prevPLT.curPLTSize[comID];
      memcpy(curLastPLT[i], tempCS->prevPLT.curPLT[i], tempCS->prevPLT.curPLTSize[comID] * sizeof(Pel));
    }
  }

  xCheckModeSplitCore(tempCS, bestCS, partitioner, encTestMode);

  if (tempCS->sps->m_PLTMode)
  {
    for (int i = compBegin; i < (compBegin + numComp); i++)
    {
      CompID comID                      = jointPLT ? (CompID)compBegin : ((i > 0) ? COMP_Cb : COMP_Y);
      tempCS->prevPLT.curPLTSize[comID] = curLastPLTSize[comID];
      memcpy(tempCS->prevPLT.curPLT[i], curLastPLT[i], curLastPLTSize[comID] * sizeof(Pel));
    }
  }
}

void EncCu::xCheckModeSplitCore(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner,
                                const EncTestMode &encTestMode)
{
  const int    qp           = encTestMode.qp;
  const Slice &slice        = *tempCS->slice;
  const int    oldPrevQp    = tempCS->prevQP[partitioner.chType];
  const auto   oldMotionLut = tempCS->motionLut;
  const auto   oldCCPLut    = tempCS->ccpLut;
#if ENABLE_QPA_SUB_CTU
  const PPS     &pps       = *tempCS->pps;
  const uint32_t currDepth = partitioner.currDepth;
#endif
  const auto oldPLT    = tempCS->prevPLT;
  const auto oldEipLut = tempCS->eipLut;

  const PartSplit split = getPartSplit(encTestMode);

  CHECK(split == CU_DONT_SPLIT, "No proper split provided!");

  tempCS->initStructData(qp);

  m_CABACEstimator->DeriveCtx::setNeighbourCus(*tempCS, partitioner.currArea(), partitioner.chType);
  m_CABACEstimator->getCtx() = m_CurrCtx->start;
  m_CABACEstimator->resetBits();
  m_CABACEstimator->split_cu_mode(split, *tempCS, partitioner);
  uint64_t splitFracBits = m_CABACEstimator->getEstFracBits();

  ComprCUCtx &comprCUCtx = *m_modeCtrl->comprCUCtx;

  if (m_encCfg->m_useCostBasedMttSkipping && comprCUCtx.bestNonSplitCost < MAX_DOUBLE && split != CU_QUAD_SPLIT &&
      partitioner.currBtDepth == 0 && partitioner.chType == ChannelType::LUMA &&
      (partitioner.currArea().lwidth() == 64 || partitioner.currArea().lwidth() == 32))
  {
    const double splitSignalCostScaling = Clip3(0.0, 4.0, (4.0 * pow(0.5, (tempCS->baseQP - 22.0) / 5)));
    const int    thresholdMTT           = Clip3(0, MAX_INT,
                                                (140 - ((slice.m_iSliceQp - 22) * 3)) *
                                                  (1000000 - (int)(m_CABACEstimator->getEstFracBits() * splitSignalCostScaling)));

    if (comprCUCtx.bestNonSplitCost > thresholdMTT)
    {
      if (split == CU_HORZ_SPLIT)
      {
        comprCUCtx.didHorzSplit = false;
      }
      else if (split == CU_VERT_SPLIT)
      {
        comprCUCtx.didVertSplit = false;
      }
      else if (split == CU_TRIH_SPLIT)
      {
        comprCUCtx.doTriHorzSplit = false;
      }
      else if (split == CU_TRIV_SPLIT)
      {
        comprCUCtx.doTriVertSplit = false;
      }

      return;
    }
  }

  double costTemp = 0;
  if (m_encCfg->m_fastAdaptCostPredMode == 2)
  {
    int numChild = 3;
    if (split == CU_VERT_SPLIT || split == CU_HORZ_SPLIT)
    {
      numChild--;
    }
    else if (split == CU_QUAD_SPLIT)
    {
      numChild++;
    }

    int64_t approxBits = numChild << SCALE_BITS;

    const double factor =
      (tempCS->currQP[partitioner.chType] > 30 ? 1.11 : 1.085) + (isChroma(partitioner.chType) ? 0.2 : 0.0);
    costTemp = m_pcRdCost->calcRdCost(uint64_t(splitFracBits + approxBits + ((bestCS->fracBits) / factor)),
                                      Distortion(bestCS->dist / factor)) +
      bestCS->costDbOffset / factor;
  }
  else if (m_encCfg->m_fastAdaptCostPredMode == 1)
  {
    const double factor =
      (tempCS->currQP[partitioner.chType] > 30 ? 1.1 : 1.075) + (isChroma(partitioner.chType) ? 0.2 : 0.0);
    costTemp = m_pcRdCost->calcRdCost(uint64_t(splitFracBits + ((bestCS->fracBits) / factor)),
                                      Distortion(bestCS->dist / factor)) +
      bestCS->costDbOffset / factor;
  }
  else
  {
    const double factor = (tempCS->currQP[partitioner.chType] > 30 ? 1.1 : 1.075);
    costTemp            = m_pcRdCost->calcRdCost(uint64_t(splitFracBits + ((bestCS->fracBits) / factor)),
                                                 Distortion(bestCS->dist / factor)) +
      bestCS->costDbOffset / factor;
  }

  const double cost = costTemp;

  if (cost > bestCS->cost + bestCS->costDbOffset
#if ENABLE_QPA_SUB_CTU
      || (m_encCfg->m_bUsePerceptQPA && !m_encCfg->m_RCEnableRateControl && pps.m_useDQP &&
          (slice.getCuQpDeltaSubdiv() > 0) && (split == CU_HORZ_SPLIT || split == CU_VERT_SPLIT) &&
          (currDepth == 0)) // force quad-split or no split at CTU level
#endif
  )
  {
    xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);
    return;
  }

  partitioner.splitCurrArea(split, *tempCS);
  bool qgEnableChildren       = partitioner.currQgEnable(); // QG possible at children level
  bool qgChromaEnableChildren = partitioner.currQgChromaEnable(); // Chroma QG possible at children level

  m_CurrCtx++;

  tempCS->getRecoBuf().fill(0);

  tempCS->getPredBuf().fill(0);
  AffineMVInfo tmpMVInfo;
  bool         isAffMVInfoSaved;
  m_pcInterSearch->savePrevAffMVInfo(0, tmpMVInfo, isAffMVInfoSaved);
  BlkUniMvInfo tmpUniMvInfo;
  bool         isUniMvInfoSaved = false;
  if (!tempCS->slice->isIntra())
  {
    m_pcInterSearch->savePrevUniMvInfo(tempCS->area.Y(), tmpUniMvInfo, isUniMvInfoSaved);
  }
  BlkUniMvInfo tmpUniMvInfoLIC;
  bool         isUniMvInfoSavedLIC = false;
  if (tempCS->slice->m_useLic && !tempCS->slice->isIntra())
  {
    m_pcInterSearch->swapUniMvBuffer();
    m_pcInterSearch->savePrevUniMvInfo(tempCS->area.Y(), tmpUniMvInfoLIC, isUniMvInfoSavedLIC);
    m_pcInterSearch->swapUniMvBuffer();
  }

  do
  {
    const auto &subCUArea = partitioner.currArea();

    if (tempCS->picture->Y().contains(subCUArea.lumaPos()))
    {
      CodingStructure *tempSubCS = m_pTempCS[partitioner.currDepth];
      CodingStructure *bestSubCS = m_pBestCS[partitioner.currDepth];
      CodingStructure *obmcSubCS = m_obmcCS[partitioner.currDepth];

      tempCS->initSubStructure(*tempSubCS, partitioner.chType, subCUArea, false);
      tempCS->initSubStructure(*bestSubCS, partitioner.chType, subCUArea, false);
      tempCS->initSubStructure(*obmcSubCS, partitioner.chType, subCUArea, false);
      double newMaxCostAllowed = isLuma(partitioner.chType)
        ? std::min(encTestMode.maxCostAllowed, bestCS->cost - m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist))
        : MAX_DOUBLE;
      newMaxCostAllowed        = std::max(0.0, newMaxCostAllowed);

      xCompressCU(tempSubCS, bestSubCS, partitioner, newMaxCostAllowed);

      if (bestSubCS->cost == MAX_DOUBLE)
      {
        CHECK(split == CU_QUAD_SPLIT, "Split decision reusing cannot skip quad split");
        tempCS->cost         = MAX_DOUBLE;
        tempCS->costDbOffset = 0;
        m_CurrCtx--;
        partitioner.exitCurrSplit();
        xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);
        tempCS->motionLut = oldMotionLut;
        tempCS->ccpLut    = oldCCPLut;
        tempCS->prevPLT   = oldPLT;
        tempCS->releaseIntermediateData();
        tempCS->prevQP[partitioner.chType] = oldPrevQp;
        return;
      }

      bool keepResi = KEEP_PRED_AND_RESI_SIGNALS;
      tempCS->useSubStructure(*bestSubCS, partitioner.chType, CS::getArea(*tempCS, subCUArea, partitioner.chType),
                              KEEP_PRED_AND_RESI_SIGNALS, true, keepResi, keepResi, true);

      if (partitioner.currQgEnable())
      {
        tempCS->prevQP[partitioner.chType] = bestSubCS->prevQP[partitioner.chType];
      }

      tempSubCS->releaseIntermediateData();
      bestSubCS->releaseIntermediateData();
    }
  } while (partitioner.nextPart(*tempCS));

  partitioner.exitCurrSplit();

  m_CurrCtx--;

  tempCS->fracBits += splitFracBits;
  tempCS->cost = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);

  // Check Delta QP bits for splitted structure
  if (!qgEnableChildren) // check at deepest QG level only
  {
    xCheckDQP(*tempCS, partitioner, true);
  }
  if (!qgChromaEnableChildren)   // check at deepest cQG level only
  {
    xCheckChromaQPOffset(*tempCS, partitioner);
  }

  // If the configuration being tested exceeds the maximum number of bytes for a slice / slice-segment, then
  // a proper RD evaluation cannot be performed. Therefore, termination of the
  // slice/slice-segment must be made prior to this CTU.
  // This can be achieved by forcing the decision to be that of the rpcTempCU.
  // The exception is each slice / slice-segment must have at least one CTU.
  if (bestCS->cost != MAX_DOUBLE) {}
  else
  {
    bestCS->costDbOffset = 0;
  }

  // RD check for sub partitioned coding structure.
  xCheckBestMode(tempCS, bestCS, partitioner, encTestMode, m_encCfg->m_encDbOpt);

  if (isAffMVInfoSaved)
  {
    m_pcInterSearch->addAffMVInfo(tmpMVInfo);
  }

  if (!tempCS->slice->isIntra() && isUniMvInfoSaved)
  {
    m_pcInterSearch->addUniMvInfo(tmpUniMvInfo);
  }
  if (!tempCS->slice->isIntra() && isUniMvInfoSavedLIC)
  {
    m_pcInterSearch->swapUniMvBuffer();
    m_pcInterSearch->addUniMvInfo(tmpUniMvInfoLIC);
    m_pcInterSearch->swapUniMvBuffer();
  }

  tempCS->motionLut = oldMotionLut;
  tempCS->ccpLut    = oldCCPLut;
  tempCS->prevPLT   = oldPLT;
  tempCS->eipLut    = oldEipLut;

  tempCS->releaseIntermediateData();

  tempCS->prevQP[partitioner.chType] = oldPrevQp;
}

bool EncCu::xCheckRDCostIntra(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner,
                              const EncTestMode &encTestMode)
{
  PROFILER_SCOPE(1, g_timeProfiler, P_INTRA);

  ComprCUCtx &comprCUCtx       = *m_modeCtrl->comprCUCtx;
  Distortion  interHad         = comprCUCtx.interHad;
  bool        foundZeroRootCbf = false;

  tempCS->initStructData(encTestMode.qp);

  CodingUnit &cu   = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);
  cu.slice         = tempCS->slice;
  cu.tileIdx       = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
  cu.skip          = false;
  cu.mmvdSkip      = false;
  cu.predMode      = MODE_INTRA;
  cu.chromaQpAdj   = m_cuChromaQpOffsetIdxPlus1;
  cu.qp            = encTestMode.qp;
  tempCS->interHad = interHad;
  partitioner.setCUData(cu);

  CUCtxIntra cuCtxIntra;
  cuCtxIntra.mpmListSize = PU::getIntraMPMs(cu, cuCtxIntra.mpmList, cuCtxIntra.nonMPMList);

  if (isLuma(partitioner.chType))
  {
    m_pcIntraSearch->estIntraPredLumaQT(cu, partitioner, cuCtxIntra, encTestMode.maxCostAllowed, bestCS,
                                        &m_pelUnitBufPool);

    if (!CS::isDualITree(*tempCS))
    {
      tempCS->lumaCost = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);
    }

    if (m_encCfg->m_usePbIntraFast && tempCS->dist == std::numeric_limits<Distortion>::max() && tempCS->interHad == 0)
    {
      interHad            = 0; // JEM assumes only perfect reconstructions can from now on beat the inter mode
      comprCUCtx.interHad = 0;
      return foundZeroRootCbf;
    }

    if (!CS::isDualITree(*tempCS))
    {
      cu.cs->picture->getRecoBuf(cu.Y()).copyFrom(cu.cs->getRecoBuf(COMP_Y));
      cu.cs->picture->getPredBuf(cu.Y()).copyFrom(cu.cs->getPredBuf(COMP_Y));
    }
  }

  if (isChromaEnabled(tempCS->area.chromaFormat) &&
      (partitioner.chType == ChannelType::CHROMA || !CS::isDualITree(*tempCS)))
  {
    m_pcIntraSearch->estIntraPredChromaQT(cu, partitioner, MAX_DOUBLE);
  }

  cu.rootCbf = false;

  for (uint32_t t = 0; t < getNumberValidTBlocks(*cu.cs->pcv); t++)
  {
    cu.rootCbf |= cu.firstTU->cbf[t] != 0;
  }

  if (!cu.rootCbf)
  {
    foundZeroRootCbf = true;
  }

  // Get total bits for current mode: encode CU
  m_CABACEstimator->resetBits();

  if ((!cu.cs->slice->isIntra() || cu.cs->slice->m_ibcFlag) && cu.Y().valid())
  {
    m_CABACEstimator->cu_skip_flag(cu);
  }
  m_CABACEstimator->pred_mode(cu);
  m_CABACEstimator->cu_pred_data_intra(cu, cuCtxIntra);

  // Encode Coefficients
  CUCtx cuCtx;
  cuCtx.isDQPCoded         = true;
  cuCtx.isChromaQpAdjCoded = true;
  m_CABACEstimator->cu_residual(cu, partitioner, cuCtx);

  tempCS->fracBits = m_CABACEstimator->getEstFracBits();
  tempCS->cost     = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);

  xEncodeDontSplit(*tempCS, partitioner);
  xCheckDQP(*tempCS, partitioner);
  xCheckChromaQPOffset(*tempCS, partitioner);
  xCalDebCost(*tempCS, partitioner);
  xCheckBestMode(tempCS, bestCS, partitioner, encTestMode, m_encCfg->m_encDbOpt);

  return foundZeroRootCbf;
}

void EncCu::xCheckPLT(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner,
                      const EncTestMode &encTestMode)
{
  if (((partitioner.currArea().lumaSize().width * partitioner.currArea().lumaSize().height <= 16) &&
       isLuma(partitioner.chType)) ||
      (isChromaEnabled(partitioner.currArea().chromaFormat) &&
       (partitioner.currArea().chromaSize().width * partitioner.currArea().chromaSize().height <= 16) &&
       !isLuma(partitioner.chType) && CS::isDualITree(*tempCS)))
  {
    return;
  }
  tempCS->initStructData(encTestMode.qp);
  CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);
  partitioner.setCUData(cu);
  cu.slice    = tempCS->slice;
  cu.tileIdx  = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
  cu.skip     = false;
  cu.mmvdSkip = false;
  cu.predMode = MODE_PLT;

  cu.chromaQpAdj  = m_cuChromaQpOffsetIdxPlus1;
  cu.qp           = encTestMode.qp;
  cu.bdpcmMode[0] = BdpcmMode::NONE;

  CHECK(!CU::pltAllowed(cu), "PLT not allowed!");

  // tempCS->addPU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);
  tempCS->addTU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);
  // Search
  tempCS->dist = 0;

  if (CS::isDualITree(*tempCS))
  {
    if (isLuma(partitioner.chType))
    {
      m_pcIntraSearch->PLTSearch(*tempCS, partitioner, COMP_Y, 1);
    }
    if (isChromaEnabled(tempCS->area.chromaFormat) && partitioner.chType == ChannelType::CHROMA)
    {
      m_pcIntraSearch->PLTSearch(*tempCS, partitioner, COMP_Cb, 2);
    }
  }
  else
  {
    m_pcIntraSearch->PLTSearch(*tempCS, partitioner, COMP_Y, getNumberValidComponents(cu.chromaFormat));
  }

  m_CABACEstimator->getCtx() = m_CurrCtx->start;
  m_CABACEstimator->resetBits();
  if ((!cu.cs->slice->isIntra() || cu.cs->slice->m_ibcFlag) && cu.Y().valid())
  {
    m_CABACEstimator->cu_skip_flag(cu);
  }
  m_CABACEstimator->pred_mode(cu);

  // signaling
  CUCtx cuCtx;
  cuCtx.isDQPCoded         = true;
  cuCtx.isChromaQpAdjCoded = true;
  if (CS::isDualITree(*tempCS))
  {
    if (isLuma(partitioner.chType))
    {
      m_CABACEstimator->cu_palette_info(cu, COMP_Y, 1, cuCtx);
    }
    if (isChromaEnabled(tempCS->area.chromaFormat) && (partitioner.chType == ChannelType::CHROMA))
    {
      m_CABACEstimator->cu_palette_info(cu, COMP_Cb, 2, cuCtx);
    }
  }
  else
  {
    m_CABACEstimator->cu_palette_info(cu, COMP_Y, getNumberValidComponents(cu.chromaFormat), cuCtx);
  }
  tempCS->fracBits = m_CABACEstimator->getEstFracBits();
  tempCS->cost     = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);

  xEncodeDontSplit(*tempCS, partitioner);
  xCheckDQP(*tempCS, partitioner);
  xCheckChromaQPOffset(*tempCS, partitioner);
  xCalDebCost(*tempCS, partitioner);
  const Area currCuArea = cu.block(getFirstComponentOfChannel(partitioner.chType));
  cu.slice->m_mapPltCost[isChroma(partitioner.chType)][currCuArea.pos()][currCuArea.size()] = tempCS->cost;
  xCheckBestMode(tempCS, bestCS, partitioner, encTestMode, m_encCfg->m_encDbOpt);
}

void EncCu::xCheckDQP(CodingStructure &cs, Partitioner &partitioner, bool bKeepCtx)
{
  CHECK(bKeepCtx && cs.cus.size() <= 1 && partitioner.getImplicitSplit(cs) == CU_DONT_SPLIT,
        "bKeepCtx should only be set in split case");
  CHECK(!bKeepCtx && cs.cus.size() > 1, "bKeepCtx should never be set for non-split case");

  if (!cs.pps->m_useDQP)
  {
    return;
  }

  if (CS::isDualITree(cs) && isChroma(partitioner.chType))
  {
    return;
  }

  if (!partitioner.currQgEnable()) // do not consider split or leaf/not leaf QG condition (checked by caller)
  {
    return;
  }

  CodingUnit *cuFirst = cs.getCU(partitioner.chType);

  CHECK(!cuFirst, "No CU available");

  bool hasResidual = false;
  for (const auto &cu: cs.cus)
  {
    // not include the chroma CU because chroma CU is decided based on corresponding luma QP and deltaQP is not signaled
    // at chroma CU
    if (cu->rootCbf)
    {
      hasResidual = true;
      break;
    }
  }

  int predQP = CU::predictQP(*cuFirst, cs.prevQP[partitioner.chType]);

  if (hasResidual)
  {
    TempCtx ctxTemp(m_ctxPool);
    if (!bKeepCtx)
    {
      ctxTemp = SubCtx(Ctx::DeltaQP, m_CABACEstimator->getCtx());
    }

    m_CABACEstimator->resetBits();
    m_CABACEstimator->cu_qp_delta(*cuFirst, predQP, cuFirst->qp);

    cs.fracBits += m_CABACEstimator->getEstFracBits();   // dQP bits
    cs.cost = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);

    if (!bKeepCtx)
    {
      m_CABACEstimator->getCtx() = SubCtx(Ctx::DeltaQP, ctxTemp);
    }

    // NOTE: reset QPs for CUs without residuals up to first coded CU
    for (const auto &cu: cs.cus)
    {
      // not include the chroma CU because chroma CU is decided based on corresponding luma QP and deltaQP is not
      // signaled at chroma CU
      if (cu->predMode != MODE_PLT ? cu->rootCbf : cu->useEscape[COMP_Y])
      {
        break;
      }
      cu->qp = predQP;
    }
  }
  else
  {
    // No residuals: reset CU QP to predicted value
    for (const auto &cu: cs.cus)
    {
      cu->qp = predQP;
    }
  }
}

void EncCu::xCheckChromaQPOffset(CodingStructure &cs, Partitioner &partitioner)
{
  // doesn't apply if CU chroma QP offset is disabled
  if (!cs.slice->m_chromaQpAdjEnabled)
  {
    return;
  }

  // doesn't apply to luma CUs
  if (CS::isDualITree(cs) && isLuma(partitioner.chType))
  {
    return;
  }

  // check cost only at cQG top-level (everything below shall not be influenced by adj coding: it occurs only once)
  if (!partitioner.currQgChromaEnable())
  {
    return;
  }

  // check if chroma is coded or not
  bool isCoded = false;
  for (auto &cu: cs.cus)
  {
    for (const TransformUnit &tu: CU::traverseTUs(*cu))
    {
      if (tu.cbf[COMP_Cb] || tu.cbf[COMP_Cr])
      {
        isCoded = true;
        break;
      }
    }
    if (isCoded)
    {
      // estimate cost for coding cu_chroma_qp_offset
      TempCtx ctxTempAdjFlag(m_ctxPool);
      TempCtx ctxTempAdjIdc(m_ctxPool);
      ctxTempAdjFlag = SubCtx(Ctx::ChromaQpAdjFlag, m_CABACEstimator->getCtx());
      ctxTempAdjIdc  = SubCtx(Ctx::ChromaQpAdjIdc, m_CABACEstimator->getCtx());
      m_CABACEstimator->resetBits();
      m_CABACEstimator->cu_chroma_qp_offset(*cu);
      cs.fracBits += m_CABACEstimator->getEstFracBits();
      cs.cost                    = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);
      m_CABACEstimator->getCtx() = SubCtx(Ctx::ChromaQpAdjFlag, ctxTempAdjFlag);
      m_CABACEstimator->getCtx() = SubCtx(Ctx::ChromaQpAdjIdc, ctxTempAdjIdc);
      break;
    }
    else
    {
      // chroma QP adj is forced to 0 for leading uncoded CUs
      cu->chromaQpAdj = 0;
    }
  }
}

void EncCu::xCheckRDCostHashInter(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner,
                                  const EncTestMode &encTestMode)
{
  PROFILER_SCOPE(1, g_timeProfiler, P_INTER_HASH);
  bool isPerfectMatch = false;

  tempCS->initStructData(encTestMode.qp);
  m_pcInterSearch->resetBufferedUniMotions();
  m_pcInterSearch->setAffineModeSelected(false);
  m_pcInterSearch->setDoAffineLic(true);
  CodingUnit &cu = tempCS->addCU(tempCS->area, partitioner.chType);

  partitioner.setCUData(cu);
  cu.slice         = tempCS->slice;
  cu.tileIdx       = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
  cu.skip          = false;
  cu.predMode      = MODE_INTER;
  cu.chromaQpAdj   = m_cuChromaQpOffsetIdxPlus1;
  cu.qp            = encTestMode.qp;
  cu.mmvdSkip      = false;
  cu.mmvdMergeFlag = false;
  CHECK(cu.licFlag, "LIC flag is set");

  if (m_pcInterSearch->predInterHashSearch(cu, partitioner, isPerfectMatch))
  {
    double equBcwCost = MAX_DOUBLE;

    const bool obmcDone = xApplyObmc(*tempCS, cu);
    const bool newBest  = xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, 0, nullptr, &equBcwCost);

    if (obmcDone && newBest && bestCS->cost != MAX_DOUBLE)
    {
      xUpdateBestObmcCS(*bestCS, partitioner, encTestMode);
    }
  }
  tempCS->initStructData(encTestMode.qp);
  int minSize = std::min(cu.lwidth(), cu.lheight());
  if (minSize < 64)
  {
    isPerfectMatch = false;
  }
  m_modeCtrl->comprCUCtx->isHashPerfectMatch = isPerfectMatch;
}

uint16_t geoBufIdx(int mergeIdx, int mmvdIdx, int tmMode)
{
  CHECK(mergeIdx >= GEO_MAX_NUM_UNI_CANDS, "merge idx out of range");
  CHECK((mmvdIdx + 1) >= GPM_EXT_MMVD_MAX_REFINE_NUM2, "mmvd idx out of range");
  int16_t bufIdx = (mergeIdx + 1) * GPM_EXT_MMVD_MAX_REFINE_NUM2 + (mmvdIdx + 1);
  return bufIdx;
}

int mergeIdxFrom(const int geoBufIdx)
{
  CHECK((GEO_MAX_NUM_UNI_CANDS + 1) * GPM_EXT_MMVD_MAX_REFINE_NUM2 <= geoBufIdx, "geoBufIdx out of range");
  int mergeIdx = geoBufIdx / GPM_EXT_MMVD_MAX_REFINE_NUM2;
  return mergeIdx - 1;
}

int mmvdIdxFrom(const int geoBufIdx)
{
  CHECK((GEO_MAX_NUM_UNI_CANDS + 1) * GPM_EXT_MMVD_MAX_REFINE_NUM2 <= geoBufIdx, "geoBufIdx out of range");
  int mmvdIdx = geoBufIdx % GPM_EXT_MMVD_MAX_REFINE_NUM2;
  return mmvdIdx - 1;
}

int tmFlagFrom(const int geoBufIdx)
{
  CHECK((GEO_MAX_NUM_UNI_CANDS + 1) * GPM_EXT_MMVD_MAX_REFINE_NUM2 <= geoBufIdx, "geoBufIdx out of range");
  int mmvdIdx = geoBufIdx % GPM_EXT_MMVD_MAX_REFINE_NUM2;
  return mmvdIdx >= GPM_EXT_MMVD_MAX_REFINE_NUM;
}

bool mmvdFlagFrom(const int geoBufIdx) { return 0 <= mmvdIdxFrom(geoBufIdx); }

void EncCu::xCheckRDCostUnifiedMerge(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner,
                                     const EncTestMode &encTestMode)
{
  bool chromaEnabled = isChromaEnabled(tempCS->area.chromaFormat);

  PROFILER_SCOPE(1, g_timeProfiler, P_INTER_MRG);
  const Slice &slice = *tempCS->slice;

  CHECK(slice.m_eSliceType == I_SLICE, "Merge modes not available for I-slices");

  tempCS->initStructData(encTestMode.qp);

  MergeCtx          mergeCtx;
  MergeCtx          bmMergeCtx;
  GeoMergeCtx       geoMergeCtx;
  AffineMergeCtx    affineMergeCtx;
  AffineMergeCtx    affineMergeCtxOrig;
  GeoComboCostList &comboList = m_comboList;
  const SPS        &sps       = *tempCS->sps;

  if (sps.m_sbtmvpEnabledFlag)
  {
    Size bufSize                 = g_miScaling.scale(tempCS->area.lumaSize());
    affineMergeCtx.subPuMvpMiBuf = MotionBuf(m_SubPuMiBuf, bufSize);
  }

  m_mergeBestSATDCost = MAX_DOUBLE;

  PROFILER_TO_NEXT_SCOPE(1, g_timeProfiler, P_INTER_MRG_EST_CAND);
  CodingUnit *cu = getPuForInterPrediction(tempCS);

  partitioner.setCUData(*cu);

  PU::getInterMergeCandidates(*cu, mergeCtx, 0, -1, 1);

  PU::getInterMMVDMergeCandidates(*cu, mergeCtx);
  cu->regularMergeFlag = true;
  cu->mergeFlag        = true;

  bool hasOppositelicMrg = CU::hasOppositeLICFlag(*cu);
  cu->affine             = true;
  bool hasOppositelicAff = CU::hasOppositeLICFlag(*cu);
  cu->affine             = false;
  cu->oppositeLicFlag    = false;

  if (PU::isBMMergeFlagCoded(*cu))
  {
    cu->bmMergeFlag = true;
    PU::getInterBMCandidates(*cu, bmMergeCtx);
    cu->bmMergeFlag = false;
    cu->bdmvrRefine = false;
  }

  PelUnitBufVector<MRG_MAX_NUM_CANDS>    mrgPredBufNoCiip(m_pelUnitBufPool);
  PelUnitBufVector<MRG_MAX_NUM_CANDS>    mrgPredBufNoMvRefine(m_pelUnitBufPool);
  PelUnitBufVector<GPM_MODES_TOTAL>      geoBuffer(m_pelUnitBufPool);
  static_vector<bool, MRG_MAX_NUM_CANDS> isRegularTestedAsSkip;

  const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda() * FRAC_BITS_SCALE;

  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.lwidth(), tempCS->area.lheight()));
  for (int i = 0; i < mergeCtx.numValidMergeCand; i++)
  {
    mrgPredBufNoCiip.push_back(m_pelUnitBufPool.getPelUnitBuf(localUnitArea));
    mrgPredBufNoMvRefine.push_back(m_pelUnitBufPool.getPelUnitBuf(localUnitArea));
    isRegularTestedAsSkip.push_back(false);
  }

  int numMergeSatdCand = bestCS->area.lumaSize().area() >= 64 ? m_encCfg->m_mergeRdCandQuotaRegular
                                                              : m_encCfg->m_mergeRdCandQuotaRegularSmallBlk;

  bool isIntrainterEnabled = sps.m_useCiip;

  {
    const int  maxCiipSize   = std::min<int>(MAX_TB_SIZEY, MAX_INTRA_SIZE);
    const bool ciipAvailable = tempCS->area.lwidth() <= maxCiipSize && tempCS->area.lheight() <= maxCiipSize &&
      tempCS->area.lwidth() * tempCS->area.lheight() >= CIIP_MIN_AREA;

    isIntrainterEnabled &= ciipAvailable;
  }

  if (isIntrainterEnabled)
  {
    numMergeSatdCand += m_encCfg->m_mergeRdCandQuotaCiip;
  }

  bool affineMrgAvailSize = CU::isAffineAllowed(*cu);

  bool affineMmvdAvail = false;
  bool affineMrgAvail =
    !((m_encCfg->m_Affine > 2) && (bestCS->slice->m_uiTLayer > 3) && (!m_encCfg->m_sbTmvpEnableFlag)) &&
    (m_encCfg->m_Affine || sps.m_sbtmvpEnabledFlag) && m_encCfg->m_maxNumAffineMergeCand && affineMrgAvailSize;

  if (affineMrgAvail)
  {
    PU::getAffineMergeCand(*cu, affineMergeCtx);
    numMergeSatdCand += std::min(m_encCfg->m_mergeRdCandQuotaSubBlk, affineMergeCtx.numValidMergeCand);

    int affMmvdBaseIdxToMergeIdxOffset = (int)PU::getMergeIdxFromAffMmvdBaseIdx(affineMergeCtx, 0);
    int affMmvdBaseCount =
      std::min<int>((int)AF_MMVD_BASE_NUM, affineMergeCtx.numValidMergeCand - affMmvdBaseIdxToMergeIdxOffset);
    affineMmvdAvail = affineMrgAvail && affMmvdBaseCount >= 1 && sps.m_AffineMmvdMode;
    if (affineMmvdAvail || hasOppositelicAff)
    {
      affineMergeCtxOrig = affineMergeCtx;
    }
  }

  CodedCUInfo &relatedCU = m_modeCtrl->getCodedCUInfo(tempCS->area);

  bool      toAddGpmCand = false;
  GeoHelper gh;
  if (!relatedCU.isSkipGPM && CU::canUseGPM(*cu))
  {
    PROFILER_TO_NEXT_SCOPE(TP_ENABLE_INTER_MERGE_EST_STAGES, g_timeProfiler, P_INTER_MRG_EST_CAND_GPM);
    cu->mergeFlag        = true;
    cu->regularMergeFlag = false;
    cu->geoFlag          = true;
    PU::setGpmDirMode(*cu);
    PU::getGeoMergeCandidates(*cu, geoMergeCtx[0]);
    geoBuffer.resize(0, nullptr);
    geoBuffer.resize(geoBuffer.max_size(), nullptr);
    toAddGpmCand =
      prepareGpmComboList(geoMergeCtx, localUnitArea, sqrtLambdaForFirstPass, comboList, geoBuffer, gh, cu, false);
    numMergeSatdCand += toAddGpmCand ? std::min(m_encCfg->m_mergeRdCandQuotaGpm, (int)comboList.list.size()) : 0;
    PROFILER_TO_PREV_SCOPE(TP_ENABLE_INTER_MERGE_EST_STAGES, g_timeProfiler, P_INTER_MRG_EST_CAND);
  }

  numMergeSatdCand = std::min(numMergeSatdCand, m_encCfg->m_maxMergeRdCandNumTotal);
  // 1. Pass: get SATD-cost for selected candidates and reduce their count
  m_mergeItemList.resetList(toAddGpmCand ? numMergeSatdCand * 2 : numMergeSatdCand);
  //  m_mergeItemList.resetList(toAddGpmCand ? (numMergeSatdCand * 3 / 2) : numMergeSatdCand);
  const TempCtx ctxStart(m_ctxPool, m_CABACEstimator->getCtx());
  DistParam     distParam;
  const int     useHadamard = tempCS->slice->m_disableSATDForRd ? 0 : 1;
  // the third arguments to setDistParam is dummy and will be updated before being used
  m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), tempCS->getOrgBuf().Y(),
                           sps.m_bitDepths[ChannelType::LUMA], COMP_Y, useHadamard);

  addRegularCandsToPruningList(m_mergeItemList, mergeCtx, localUnitArea, sqrtLambdaForFirstPass, ctxStart,
                               &mrgPredBufNoCiip, &mrgPredBufNoMvRefine, distParam, cu);

  // CIIP needs to be checked right after regular merge as the checking is based on the best 4 regular merge cand in the
  // mergeItemList
  if (isIntrainterEnabled)
  {
    addCiipCandsToPruningList(m_mergeItemList, mergeCtx, localUnitArea, sqrtLambdaForFirstPass, ctxStart,
                              mrgPredBufNoCiip, mrgPredBufNoMvRefine, distParam, cu);
  }

  if (bmMergeCtx.numValidMergeCand > 0)
  {
    addBMCandsToPruningList(m_mergeItemList, bmMergeCtx, localUnitArea, sqrtLambdaForFirstPass, ctxStart, distParam,
                            cu);
  }

  if (cu->cs->sps->m_useMMVD)
  {
    addMmvdCandsToPruningList(m_mergeItemList, mergeCtx, localUnitArea, sqrtLambdaForFirstPass, ctxStart, distParam,
                              cu);
  }

  if (affineMergeCtx.numValidMergeCand > 0)
  {
    addAffineCandsToPruningList(m_mergeItemList, affineMergeCtx, localUnitArea, sqrtLambdaForFirstPass, ctxStart,
                                distParam, cu);
  }

  if (affineMmvdAvail)
  {
    addAffineMmvdCandsToPruningList(m_mergeItemList, affineMergeCtxOrig, localUnitArea, sqrtLambdaForFirstPass,
                                    ctxStart, distParam, cu);
  }

  int    numCand        = numMergeSatdCand;
  double thresholdLimit = m_mergeItemList.getMergeItemInList(0)->cost * MRG_FAST_RATIO;
  numMergeSatdCand      = updateRdCheckingNum(m_mergeItemList, thresholdLimit, numMergeSatdCand);

  // limit candidate list to orignal merge size list
  m_mergeItemList.shrink(numMergeSatdCand);

  bool testGPMMMVD = false;
  if (toAddGpmCand)
  {
    PROFILER_TO_NEXT_SCOPE(TP_ENABLE_INTER_MERGE_EST_STAGES, g_timeProfiler, P_INTER_MRG_EST_CAND_GPM);
    // shrink combo list to keep regualar merge cand
    comboList.list.resize(std::min((size_t)m_encCfg->m_maxMergeRdCandNumTotal * GEO_NUM_BLD, comboList.list.size()));
    double       bestCostSoFar  = m_mergeItemList.getMergeItemInList(0)->cost;
    double       worstCostSoFar = m_mergeItemList.getMergeItemInList(m_mergeItemList.size() - 1)->cost;
    const size_t numNonGeoCandsPreserved =
      std::min((size_t)m_encCfg->m_mergeRdCandQuotaNonGeoPreserved, m_mergeItemList.size());
    double bestGPMCost =
      addGpmCandsToPruningList(m_mergeItemList, geoMergeCtx, localUnitArea, sqrtLambdaForFirstPass, ctxStart, comboList,
                               gh, geoBuffer, distParam, cu, numNonGeoCandsPreserved);
    testGPMMMVD = bestGPMCost < worstCostSoFar && bestGPMCost < (bestCostSoFar * 1.1);   // 1.5
    relatedCU.isSkipGPM |= bestGPMCost > worstCostSoFar;
    numMergeSatdCand = std::min(numMergeSatdCand + numCand, (int)m_mergeItemList.size());

    if (bestGPMCost > bestCostSoFar)
    {
      double thresholdLimit = bestGPMCost * MRG_FAST_RATIO;
      numMergeSatdCand      = updateRdCheckingNum(m_mergeItemList, thresholdLimit, numMergeSatdCand);
    }
    PROFILER_TO_PREV_SCOPE(TP_ENABLE_INTER_MERGE_EST_STAGES, g_timeProfiler, P_INTER_MRG_EST_CAND);
  }

  cu->ciipFlag         = false;
  cu->geoFlag          = false;
  cu->bdmvrRefine      = false;
  cu->affine           = false;
  cu->mmvdSkip         = false;
  cu->bmMergeFlag      = false;
  cu->bdmvrRefine      = false;
  cu->regularMergeFlag = true;

  if (hasOppositelicMrg && mergeCtx.numValidMergeCand > 0)
  {
    cu->oppositeLicFlag       = true;
    mergeCtx.numCandToTestEnc = m_encCfg->m_maxNumOppositeLicMergeCand;
    addRegularCandsToPruningList(m_mergeItemList, mergeCtx, localUnitArea, sqrtLambdaForFirstPass, ctxStart, nullptr,
                                 nullptr, distParam, cu);
    cu->oppositeLicFlag       = false;
    mergeCtx.numCandToTestEnc = mergeCtx.numValidMergeCand;
  }
  if (hasOppositelicAff && affineMergeCtx.numValidMergeCand > 0)
  {
    cu->oppositeLicFlag                = true;
    affineMergeCtx                     = affineMergeCtxOrig;
    affineMergeCtx.numAffCandToTestEnc = m_encCfg->m_maxNumAffineOppositeLicMergeCand;
    addAffineCandsToPruningList(m_mergeItemList, affineMergeCtx, localUnitArea, sqrtLambdaForFirstPass, ctxStart,
                                distParam, cu);
    cu->oppositeLicFlag                = false;
    affineMergeCtx.numAffCandToTestEnc = affineMergeCtx.numValidMergeCand;
  }

  // 2. Pass: RD checking
  tempCS->initStructData(encTestMode.qp);
  m_CABACEstimator->getCtx() = ctxStart;
  bool bestIsSkip            = false;

  for (int gpmMMVD = 0; gpmMMVD < (testGPMMMVD ? 2 : 1); gpmMMVD++)
  {
    MergeItemList *mergeItemList = &m_mergeItemList;
    if (gpmMMVD)
    {
      CodingUnit *bestCU = bestCS->getCU(partitioner.chType);
      if (bestCU && bestCU->skip)
      {
        if ((!bestCU->geoFlag && !bestCU->affine && !bestCU->mmvdSkip && !bestCU->mmvdMergeFlag) ||
            (bestCU->affine && (bestCU->lwidth() >= 16 || bestCU->lheight() >= 16)))
        {
          continue;   // end the mode decision
        }
      }

      PROFILER_TO_NEXT_SCOPE(TP_ENABLE_INTER_MERGE_EST_STAGES, g_timeProfiler, P_INTER_MRG_EST_CAND_GPM_MMVD);
      CodingUnit *cuTmp = getPuForInterPrediction(tempCS);
      partitioner.setCUData(*cuTmp);
      cuTmp->mergeFlag        = true;
      cuTmp->regularMergeFlag = false;
      cuTmp->geoFlag          = true;
      cuTmp->oppositeLicFlag  = false;
      PU::setGpmDirMode(*cuTmp);

      // replace candidate list
      bool toAddGpmMMVDCand =
        prepareGpmComboList(geoMergeCtx, localUnitArea, sqrtLambdaForFirstPass, comboList, geoBuffer, gh, cuTmp, true);
      PROFILER_TO_PREV_SCOPE(TP_ENABLE_INTER_MERGE_EST_STAGES, g_timeProfiler, P_INTER_MRG_EST_CAND);
      if (!toAddGpmMMVDCand)
      {
        // nothing to process
        continue;
      }
      PROFILER_TO_NEXT_SCOPE(TP_ENABLE_INTER_MERGE_EST_STAGES, g_timeProfiler, P_INTER_MRG_EST_CAND_GPM_MMVD);
      numMergeSatdCand = std::min(m_encCfg->m_mergeRdCandQuotaGpm, (int)comboList.list.size());
      m_mergeItemListGeoMMVD.resetList(numMergeSatdCand);
      double bestCost =
        addGpmCandsToPruningList(m_mergeItemListGeoMMVD, geoMergeCtx, localUnitArea, sqrtLambdaForFirstPass, ctxStart,
                                 comboList, gh, geoBuffer, distParam, cuTmp, 0);
      mergeItemList = &m_mergeItemListGeoMMVD;
      PROFILER_TO_PREV_SCOPE(TP_ENABLE_INTER_MERGE_EST_STAGES, g_timeProfiler, P_INTER_MRG_EST_CAND);
      if (bestCost == MAX_DOUBLE)
      {
        // nothing to process
        continue;
      }

      // determine new limit
      thresholdLimit = std::min(mergeItemList->getMergeItemInList(0)->cost * MRG_FAST_RATIO,
                                m_mergeItemList.getMergeItemInList(0)->cost);
    }

    numMergeSatdCand = updateRdCheckingNum(*mergeItemList, thresholdLimit, numMergeSatdCand);

    PROFILER_TO_PREV_SCOPE(1, g_timeProfiler, P_INTER_MRG);
    PROFILER_TO_NEXT_SCOPE(1, g_timeProfiler, P_INTER_MRG_RD_CHECK);
    for (uint32_t noResidualPass = 0; noResidualPass < 2; noResidualPass++)
    {
      const bool forceNoResidual = noResidualPass == 1;
      for (uint32_t mrgHadIdx = 0; mrgHadIdx < numMergeSatdCand; mrgHadIdx++)
      {
        auto mergeItem = mergeItemList->getMergeItemInList(mrgHadIdx);
        CHECK(mergeItem == nullptr, "Wrong merge item");

        if (noResidualPass != 0 &&
            mergeItem->mergeItemType == MergeItem::MergeItemType::CIIP)   // intrainter does not support skip mode
        {
          if (isRegularTestedAsSkip[mergeItem->mergeIdx])
          {
            continue;
          }
        }
        if (((noResidualPass != 0) && mergeItem->noResidual) || ((noResidualPass == 0) && bestIsSkip))
        {
          continue;
        }

        cu = getPuForInterPrediction(tempCS);
        partitioner.setCUData(*cu);
        const bool resetCiip2Regular = mergeItem->exportMergeInfo(*cu, forceNoResidual);
        CHECK(!cu->cs->sps->m_biLicEnabledFlag && cu->licFlag && cu->interDir == 3, "bi-LIC not supported");

        if (m_encCfg->m_seiCfg.m_MCTSEncConstraint)
        {
          if (!MCTSHelper::checkMvBufferForMCTSConstraint(*cu))
          {
            // Do not use this mode
            tempCS->initStructData(encTestMode.qp);
            continue;
          }
        }

        if (mergeItem->mergeItemType == MergeItem::MergeItemType::MMVD && mergeItem->noBdofRefine)
        {
          // no BDOF refinement was made for the luma prediction, need to have luma prediction again
          mergeItem->lumaPredReady = false;
        }
        if (mergeItem->mergeItemType == MergeItem::MergeItemType::AFF_MMVD && mergeItem->noBdofRefine)
        {
          // no PROF refinement was made for the luma prediction in pre-check, need to have luma prediction again
          mergeItem->lumaPredReady = false;
        }

        PelUnitBuf *predBuf1 = nullptr, *predBuf2 = nullptr;
        if (mergeItem->mergeItemType == MergeItem::MergeItemType::CIIP)
        {
          predBuf1                   = mrgPredBufNoMvRefine[mergeItem->mergeIdx];
          // we check if OBMC was already performed for a previous CIIP mode that uses the same merge index
          auto currentMergeIndexCIIP = mergeItem->mergeIdx;
          for (uint32_t mrgHadIdxForCIIP = 0; mrgHadIdxForCIIP < mrgHadIdx; mrgHadIdxForCIIP++)
          {
            auto mergeItemForCIIP = mergeItemList->getMergeItemInList(mrgHadIdxForCIIP);
            if (mergeItemForCIIP->mergeItemType == MergeItem::MergeItemType::CIIP &&
                mergeItemForCIIP->mergeIdx == currentMergeIndexCIIP)
            {
              mergeItem->mergeIdxAlreadyCheckByCIIP = mergeItemForCIIP->obmcDone;
              break;
            }
          }
        }
        else if (mergeItem->mergeItemType == MergeItem::MergeItemType::GPM)
        {
          int tmShape0 = 0;
          int tmShape1 = 0;

          int gpmIdx0 = (cu->geoMergeIdx[0] < GEO_MAX_NUM_UNI_CANDS)
            ? geoBufIdx(cu->geoMergeIdx[0], (int8_t)cu->geoMMVDIdx[0], tmShape0)
            : gh.getGeoBufIntraIdx(cu->geoSplitDir, 0, cu->geoMergeIdx[0] - GEO_MAX_NUM_UNI_CANDS);
          int gpmIdx1 = (cu->geoMergeIdx[1] < GEO_MAX_NUM_UNI_CANDS)
            ? geoBufIdx(cu->geoMergeIdx[1], (int8_t)cu->geoMMVDIdx[1], tmShape1)
            : gh.getGeoBufIntraIdx(cu->geoSplitDir, 1, cu->geoMergeIdx[1] - GEO_MAX_NUM_UNI_CANDS);

          predBuf1 = geoBuffer[gpmIdx0];
          predBuf2 = geoBuffer[gpmIdx1];
          CHECK(nullptr == predBuf1, "something is wrong");
          CHECK(nullptr == predBuf2, "something is wrong");
        }

        // get prediction signal for RD check
        PelUnitBuf itemBuf = mergeItem->getPredBuf(localUnitArea);
        if (resetCiip2Regular)
        {
          itemBuf               = *mrgPredBufNoCiip[mergeItem->mergeIdx];
          cu->licScaleAndOffset = mergeItem->licScaleAndOffset;
          bool filterChroma     = isChromaEnabled(cu->chromaFormat);
          m_pcInterSearch->obmcFilter(*cu, itemBuf, true, filterChroma);
        }
        else if (!mergeItem->lumaPredReady || (!mergeItem->chromaPredReady && chromaEnabled) || !mergeItem->obmcDone)
        {
          if (mergeItem->mergeItemType == MergeItem::MergeItemType::CIIP && !mergeItem->obmcDone)
          {
            // regenerate prediction, now with OBMC
            mergeItem->lumaPredReady   = false;
            // the chroma prediction is assumed "done" if chroma is disabled
            mergeItem->chromaPredReady = !chromaEnabled;
          }
          bool chromaPredReady = (!mergeItem->chromaPredReady && chromaEnabled);

          CHECK((!mergeItem->lumaPredReady || !chromaPredReady) && mergeItem->obmcDone,
                "expected that OBMC is not done");
          generateMergePrediction(localUnitArea, mergeItem, *cu, !mergeItem->lumaPredReady, !mergeItem->chromaPredReady,
                                  itemBuf, true, forceNoResidual, predBuf1, predBuf2, &mergeCtx, true);
          CHECK(!mergeItem->obmcDone, "OBMC must be done at this point");
        }

        PelUnitBuf dstPredBuf = tempCS->getPredBuf(*cu);
        dstPredBuf.copyFrom(itemBuf);

        if (!cu->mmvdSkip && !cu->ciipFlag && !cu->affine && !cu->mmvdMergeFlag && !cu->geoFlag && !cu->bmMergeFlag &&
            noResidualPass != 0)
        {
          CHECK(mergeItem->mergeIdx >= mergeCtx.numValidMergeCand, "out of normal merge");
          isRegularTestedAsSkip[mergeItem->mergeIdx] = true;
        }

        xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, noResidualPass,
                             noResidualPass == 0 ? &mergeItem->noResidual : nullptr, nullptr);

        if (m_encCfg->m_useFastDecisionForMerge && !bestIsSkip && !cu->ciipFlag)
        {
          bestIsSkip = !bestCS->cus.empty() && bestCS->getCU(partitioner.chType)->rootCbf == 0;
        }
        tempCS->initStructData(encTestMode.qp);
      }   // end loop mrgHadIdx

      if (noResidualPass == 0 && m_encCfg->m_useEarlySkipDetection)
      {
        checkEarlySkip(bestCS, partitioner);
      }
    }
    PROFILER_TO_PREV_SCOPE(1, g_timeProfiler, P_INTER_MRG);
    PROFILER_TO_NEXT_SCOPE(1, g_timeProfiler, P_INTER_MRG_EST_CAND);
  }
  PROFILER_TO_PREV_SCOPE(1, g_timeProfiler, P_INTER_MRG);

  {
    CodingUnit *pcu = bestCS->getCU(partitioner.chType);
    if (pcu)
    {
      pcu->mmvdSkip = pcu->skip && pcu->mmvdSkip;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
// ibc merge/skip mode check
void EncCu::xCheckRDCostIBCModeMerge2Nx2N(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner,
                                          const EncTestMode &encTestMode)
{
  CHECK(partitioner.chType == ChannelType::CHROMA, "chroma IBC is derived");
  CHECK(!tempCS->sps->m_ibcMerge, "IBC merge should not be chekced at encoder with IBC merge disabled");

  // don't use IBC for large CUs
  if (tempCS->area.lwidth() > IBC_MAX_CU_SIZE || tempCS->area.lheight() > IBC_MAX_CU_SIZE)
  {
    return;
  }
  PROFILER_SCOPE(1, g_timeProfiler, P_IBC);

  const SPS &sps = *tempCS->sps;

  tempCS->initStructData(encTestMode.qp);
  MergeCtx mergeCtx;

  {
    // first get merge candidates
    CodingUnit cu(tempCS->area);
    cu.cs               = tempCS;
    cu.predMode         = MODE_IBC;
    cu.slice            = tempCS->slice;
    cu.tileIdx          = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
    cu.cs               = tempCS;
    cu.mmvdSkip         = false;
    cu.mmvdMergeFlag    = false;
    cu.regularMergeFlag = false;
    cu.geoFlag          = false;
    CHECK(cu.licFlag, "LIC flag is set");
    PU::getIBCMergeCandidates(cu, mergeCtx);
  }

  int candHasNoResidual[MRG_MAX_NUM_CANDS];
  for (unsigned int ui = 0; ui < mergeCtx.numValidMergeCand; ui++)
  {
    candHasNoResidual[ui] = 0;
  }

  bool                                       bestIsSkip     = false;
  unsigned                                   numMrgSATDCand = mergeCtx.numValidMergeCand;
  static_vector<unsigned, MRG_MAX_NUM_CANDS> rdModeList(MRG_MAX_NUM_CANDS);
  for (unsigned i = 0; i < MRG_MAX_NUM_CANDS; i++)
  {
    rdModeList[i] = i;
  }

  auto isFracBv = [&tempCS](Mv mv)
  {
    return tempCS->sps->m_ibcFracFlag &&
      ((mv.hor & ((1 << MV_FRACTIONAL_BITS_INTERNAL) - 1)) != 0 ||
       (mv.ver & ((1 << MV_FRACTIONAL_BITS_INTERNAL) - 1)) != 0);
  };

  static_vector<double, MRG_MAX_NUM_CANDS> candCostList(MRG_MAX_NUM_CANDS, MAX_DOUBLE);
  // 1. Pass: get SATD-cost for selected candidates and reduce their count
  {
    const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda();

    CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, (const ChannelType)partitioner.chType),
                                   (const ChannelType)partitioner.chType);

    partitioner.setCUData(cu);
    cu.slice       = tempCS->slice;
    cu.tileIdx     = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
    cu.skip        = false;
    cu.predMode    = MODE_IBC;
    cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
    cu.qp          = encTestMode.qp;
    cu.mmvdSkip    = false;
    cu.geoFlag     = false;
    CHECK(cu.licFlag, "LIC flag is set");
    DistParam distParam;
    const int useHadamard     = cu.slice->m_disableSATDForRd ? 0 : 1;
    cu.mmvdMergeFlag          = false;
    cu.regularMergeFlag       = false;
    Picture      *refPic      = cu.slice->m_pic;
    const CPelBuf refBuf      = refPic->getRecoBuf(cu.blocks[COMP_Y]);
    const Pel    *piRefSrch   = refBuf.buf;
    PelBuf        predBuf     = tempCS->getPredBuf(cu.Y());
    PelUnitBuf    predUnitBuf = cu.cs->getPredBuf(cu);
    if (tempCS->slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag)
    {
      const CompArea &area = cu.blocks[COMP_Y];
      CompArea        tmpArea(COMP_Y, area.chromaFormat, Position(0, 0), area.size());
      PelBuf          tmpLuma = m_tmpStorageCtu->getBuf(tmpArea);
      tmpLuma.copyFrom(tempCS->getOrgBuf().Y());
      tmpLuma.rspSignal(m_pcReshape->m_fwdLUT);
      m_pcRdCost->setDistParam(distParam, tmpLuma, refBuf, sps.m_bitDepths[ChannelType::LUMA], COMP_Y, useHadamard);
    }
    else
    {
      m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), refBuf, sps.m_bitDepths[ChannelType::LUMA], COMP_Y,
                               useHadamard);
    }
    ptrdiff_t      refStride = refBuf.stride;
    const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.lwidth(), tempCS->area.lheight()));
    int            numValidBv = mergeCtx.numValidMergeCand;
    for (unsigned int mergeCand = 0; mergeCand < mergeCtx.numValidMergeCand; mergeCand++)
    {
      mergeCtx.setMergeInfo(cu, mergeCand);   // set bv info in merge mode
      int roiWidth  = cu.lwidth();
      int roiHeight = cu.lheight();
      int xPred     = cu.bv.getHor();
      int yPred     = cu.bv.getVer();

      if (!m_pcInterSearch->checkValidBv(cu, COMP_Y, roiWidth, roiHeight, cu.mv[RPL0], true,
                                         InterpolationFilter::Filter::IBC))   // not valid bv derived
      {
        numValidBv--;
        continue;
      }
      PU::spanMotionInfo(cu);

      bool foundFracBV = isFracBv(cu.mv[0]);
      if (foundFracBV)
      {
        m_pcInterSearch->getPredIBCBlk(cu, COMP_Y, cu.slice->m_pic, cu.mv[0], predUnitBuf);
        distParam.cur = predBuf;
      }
      else
      {
        distParam.cur.buf    = piRefSrch + refStride * yPred + xPred;
        distParam.cur.stride = refStride;
      }

      Distortion   sad      = distParam.distFunc(distParam);
      unsigned int bitsCand = mergeCand + 1;
      if (mergeCand == tempCS->sps->m_maxNumMergeCand - 1)
      {
        bitsCand--;
      }
      double cost = (double)sad + (double)bitsCand * sqrtLambdaForFirstPass;
      updateCandList(mergeCand, cost, rdModeList, candCostList, numMrgSATDCand);
    }

    // Try to limit number of candidates using SATD-costs
    if (numValidBv)
    {
      numMrgSATDCand = numValidBv;
      for (unsigned int i = 1; i < numValidBv; i++)
      {
        if (candCostList[i] > MRG_FAST_RATIO * candCostList[0])
        {
          numMrgSATDCand = i;
          break;
        }
      }
    }
    else
    {
      tempCS->dist         = 0;
      tempCS->fracBits     = 0;
      tempCS->cost         = MAX_DOUBLE;
      tempCS->costDbOffset = 0;
      tempCS->initStructData(encTestMode.qp);
      return;
    }

    tempCS->initStructData(encTestMode.qp);
  }
  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.lwidth(), tempCS->area.lheight()));

  bool satdCandPredFilled[MRG_MAX_NUM_CANDS] = {
    false,
  };
  const unsigned int iteration = 2;
  // 2. Pass: check candidates using full RD test
  for (unsigned int numResidualPass = 0; numResidualPass < iteration; numResidualPass++)
  {
    for (unsigned int mrgHADIdx = 0; mrgHADIdx < numMrgSATDCand; mrgHADIdx++)
    {
      unsigned int mergeCand = rdModeList[mrgHADIdx];
      if (!(numResidualPass == 1 && candHasNoResidual[mergeCand] == 1))
      {
        if (!(bestIsSkip && (numResidualPass == 0)))
        {
          {
            // first get merge candidates
            CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, (const ChannelType)partitioner.chType),
                                           (const ChannelType)partitioner.chType);

            partitioner.setCUData(cu);
            cu.slice                         = tempCS->slice;
            cu.tileIdx                       = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
            cu.skip                          = false;
            cu.predMode                      = MODE_IBC;
            cu.chromaQpAdj                   = m_cuChromaQpOffsetIdxPlus1;
            cu.qp                            = encTestMode.qp;
            cu.sbtInfo                       = 0;
            cu.intraDir[ChannelType::LUMA]   = DC_IDX;   // set intra pred for ibc block
            cu.intraDir[ChannelType::CHROMA] = PLANAR_IDX;   // set intra pred for ibc block
            cu.mmvdSkip                      = false;
            cu.mmvdMergeFlag                 = false;
            cu.regularMergeFlag              = false;
            cu.geoFlag                       = false;
            CHECK(cu.licFlag, "LIC flag is set");
            mergeCtx.setMergeInfo(cu, mergeCand);
            PU::spanMotionInfo(cu);
            PelUnitBuf predBuf = tempCS->getPredBuf();

            const bool chroma = !CS::isDualITree(*tempCS);
            //  MC
            m_pcInterSearch->motionCompensation(cu, predBuf, RPL0, true, chroma, nullptr, true);
            bool hasBufferedCanPred = mrgHADIdx < MRG_MAX_NUM_CANDS && satdCandPredFilled[mrgHADIdx];
            if (hasBufferedCanPred)
            {
              predBuf.copyFrom(m_acMergeTmpBuffer[mrgHADIdx].getBuf(localUnitArea), true /*!chroma*/, chroma);
            }
            else
            {
              m_pcInterSearch->motionCompensation(cu, predBuf, RPL0, true, chroma, nullptr, true);
              if (mrgHADIdx < MRG_MAX_NUM_CANDS)
              {
                satdCandPredFilled[mrgHADIdx] = true;
                m_acMergeTmpBuffer[mrgHADIdx].getBuf(localUnitArea).copyFrom(predBuf, true /*!chroma*/, chroma);
              }
            }
            m_CABACEstimator->getCtx() = m_CurrCtx->start;

            m_pcInterSearch->setHistBestTrs(MAX_UCHAR, MtsType::NONE);
            m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, (numResidualPass != 0), true, chroma);
            xEncodeDontSplit(*tempCS, partitioner);
            xCheckDQP(*tempCS, partitioner);
            xCheckChromaQPOffset(*tempCS, partitioner);
            xCalDebCost(*tempCS, partitioner);
            xCheckBestMode(tempCS, bestCS, partitioner, encTestMode, m_encCfg->m_encDbOpt);

            tempCS->initStructData(encTestMode.qp);
          }

          if (m_encCfg->m_useFastDecisionForMerge && !bestIsSkip)
          {
            if (bestCS->getCU(partitioner.chType) == nullptr)
            {
              bestIsSkip = 0;
            }
            else
            {
              bestIsSkip = bestCS->getCU(partitioner.chType)->rootCbf == 0;
            }
          }
        }
      }
    }
  }
}

CodingUnit *EncCu::getPuForInterPrediction(CodingStructure *cs)
{
  CodingUnit *cu = cs->getCU(ChannelType::LUMA);
  if (cu == nullptr)
  {
    CHECK(cs->getCU(ChannelType::LUMA) != nullptr, "Wrong CU/PU setting in CS");
    CodingUnit &_cu = cs->addCU(cs->area, ChannelType::LUMA);
    cu              = &_cu;
  }
  cu->slice       = cs->slice;
  cu->tileIdx     = cs->pps->getTileIdx(cs->area.lumaPos());
  cu->skip        = false;
  cu->mmvdSkip    = false;
  cu->geoFlag     = false;
  cu->predMode    = MODE_INTER;
  cu->chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
  cu->qp          = cs->currQP[ChannelType::LUMA];
  CHECK(cu->licFlag, "LIC flag is set");

  return cu;
}

void EncCu::generateMergePrediction(const UnitArea &unitArea, MergeItem *mergeItem, CodingUnit &cu, bool luma,
                                    bool chroma, PelUnitBuf &dstBuf, bool finalRd, bool forceNoResidual,
                                    PelUnitBuf *predBuf1, PelUnitBuf *predBuf2, const MergeCtx *mrgCtx, bool obmc)
{
  // Disable chroma for when we are in 4:0:0
  chroma = chroma && (isChromaEnabled(cu.chromaFormat));

  CHECK((luma && mergeItem->lumaPredReady) || (chroma && mergeItem->chromaPredReady), "Prediction has been avaiable");
  // update Pu info
  mergeItem->exportMergeInfo(cu, forceNoResidual);

  switch (mergeItem->mergeItemType)
  {
  case MergeItem::MergeItemType::REGULAR:
    if (luma && PU::checkBDMVRCondition(cu))
    {
      m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR[0], m_mvBufBDMVR[1]);
      cu.bdmvrRefine = true;

      if (!cu.bmMergeFlag && mrgCtx->checkSimilarMotion(cu.mergeIdx, PU::getBDMVRMvdThreshold(cu)))
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
        cu.bdmvrRefine = m_pcInterSearch->processBDMVR(cu);
      }
    }

    // here predBuf1 is predBufNoMvRefine, predBuf2 is predBufNoCiip
    CHECK(((obmc || cu.bmMergeFlag) != (predBuf1 == nullptr)) && !cu.oppositeLicFlag, "unexpected condition");
    m_pcInterSearch->m_storeBeforeLIC = predBuf1 != nullptr && cu.licFlag;
    if (m_pcInterSearch->m_storeBeforeLIC)
    {
      CHECK(obmc, "not expected");
      CHECK(!m_pcInterSearch->m_predictionBeforeLIC.bufs.empty(),
            "m_pcInterSearch->m_predictionBeforeLIC is already set");
      m_pcInterSearch->m_predictionBeforeLIC = *predBuf1;
      predBuf1                               = nullptr;
    }
    m_pcInterSearch->motionCompensation(cu, dstBuf, RPLX, luma, chroma, predBuf1, obmc);
    m_pcInterSearch->m_storeBeforeLIC      = false;
    m_pcInterSearch->m_predictionBeforeLIC = PelUnitBuf();
    if (luma && cu.bdmvrRefine)
    {
      mergeItem->bdmvrRefine = cu.bdmvrRefine;
      // now getBdofSubPuMvOffsets is completed
      MotionBuf mb           = mergeItem->getMvBuf(unitArea);
      PU::spanBdmvrMotionInfo(cu, mb, m_mvBufBDMVR[0], m_mvBufBDMVR[1],
                              m_pcInterSearch->isBDOFMvRefined() ? m_pcInterSearch->getBdofSubPuMvOffset() : nullptr);
    }
    if (predBuf2 != nullptr)
    {
      predBuf2->copyFrom(dstBuf, luma, chroma);
    }
    if (mergeItem->interDir == 3)
    {
      mergeItem->mvField[0][RPL0].mv = cu.mv[RPL0];
      mergeItem->mvField[0][RPL1].mv = cu.mv[RPL1];
    }
    break;

  case MergeItem::MergeItemType::CIIP:
    // here predBuf1 is predBufNoMvRefine
    CHECK(predBuf1 == nullptr, "Invalid input buffer to CIIP");
    if (obmc && !mergeItem->mergeIdxAlreadyCheckByCIIP)
    {
      bool filterChroma = isChromaEnabled(cu.chromaFormat);
      m_pcInterSearch->obmcFilter(cu, *predBuf1, true, filterChroma);
    }
    if (luma)
    {
      dstBuf.Y().copyFrom(predBuf1->Y());
      if (cu.cs->slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag)
      {
        dstBuf.Y().rspSignal(m_pcReshape->m_fwdLUT);
      }
      CHECK(cu.ciipFlag == false, "error in CIIP!");
      m_pcIntraSearch->geneWeightedPred(dstBuf.Y(), cu,
                                        m_pcIntraSearch->getPredictorPtr2(COMP_Y, (uint32_t)cu.ciipMode), COMP_Y);
    }
    if (chroma)
    {
      dstBuf.copyFrom(*predBuf1, false, true);
      CHECK(cu.ciipFlag == false, "error in CIIP!");
      m_pcIntraSearch->geneWeightedPred(dstBuf.Cb(), cu,
                                        m_pcIntraSearch->getPredictorPtr2(COMP_Cb, (uint32_t)cu.ciipMode), COMP_Cb);
      m_pcIntraSearch->geneWeightedPred(dstBuf.Cr(), cu,
                                        m_pcIntraSearch->getPredictorPtr2(COMP_Cr, (uint32_t)cu.ciipMode), COMP_Cr);
    }
    break;

  case MergeItem::MergeItemType::MMVD:
    cu.mmvdEncOptMode       = finalRd ? 0 : (cu.mmvdMergeIdx.pos.step > 2 ? 2 : 1);
    mergeItem->noBdofRefine = cu.mmvdEncOptMode == 2 && cu.cs->sps->m_bdofEnabledFlag;
    m_pcInterSearch->motionCompensation(cu, dstBuf, RPLX, luma, chroma, nullptr, obmc);
    break;

  case MergeItem::MergeItemType::SBTMVP:
    m_pcInterSearch->motionCompensation(cu, dstBuf, RPLX, luma, chroma, nullptr, obmc);
    break;

  case MergeItem::MergeItemType::AFFINE:
    m_pcInterSearch->motionCompensation(cu, dstBuf, RPLX, luma, chroma, nullptr, obmc);
    break;

  case MergeItem::MergeItemType::AFF_MMVD:
    cu.mmvdEncOptMode       = finalRd ? 0 : (cu.mmvdMergeIdx.pos.step > 2 ? 3 : 0);
    mergeItem->noBdofRefine = cu.mmvdEncOptMode == 3 && !cu.cs->picHeader->m_profDisabledFlag;
    m_pcInterSearch->motionCompensation(cu, dstBuf, RPLX, luma, chroma, nullptr, obmc);
    cu.mmvdEncOptMode = 0;
    break;

  case MergeItem::MergeItemType::GPM:
    {
      const bool chromaGPM = isChromaEnabled(cu.chromaFormat);
      CHECK(predBuf1 == nullptr || predBuf2 == nullptr, "Invalid input buffer to GPM");
      const ChannelType chType = chromaGPM ? ChannelType::NUM : ChannelType::LUMA;
      if (cu.gpmIntraFlag)
      {
        bool reshape = cu.slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag;
        auto tmpBuf1 = m_pelUnitBufPool.getPelUnitBuf(unitArea);
        auto tmpBuf2 = m_pelUnitBufPool.getPelUnitBuf(unitArea);

        tmpBuf1->roundToOutputBitdepth(*predBuf1, cu.slice->m_clpRngs);
        tmpBuf2->roundToOutputBitdepth(*predBuf2, cu.slice->m_clpRngs);
        if (cu.geoMergeIdx[0] < GEO_MAX_NUM_UNI_CANDS)
        {
          if (obmc)
          {
            m_pcInterSearch->obmcFilter(cu, *tmpBuf1, true, chromaGPM);
          }
          if (reshape)
          {
            tmpBuf1->Y().rspSignal(m_pcReshape->m_fwdLUT);
          }
        }

        if (cu.geoMergeIdx[1] < GEO_MAX_NUM_UNI_CANDS)
        {
          if (obmc)
          {
            m_pcInterSearch->obmcFilter(cu, *tmpBuf2, true, chromaGPM);
          }
          if (reshape)
          {
            tmpBuf2->Y().rspSignal(m_pcReshape->m_fwdLUT);
          }
        }
        m_pcInterSearch->weightedGeoRounded(cu, chType, dstBuf, *tmpBuf1, *tmpBuf2);
        m_pelUnitBufPool.giveBack(tmpBuf2);
        m_pelUnitBufPool.giveBack(tmpBuf1);
      }
      else
      {
        if (luma || chroma)
        {
          m_pcInterSearch->weightedGeo(cu, chType, dstBuf, *predBuf1, *predBuf2);
        }
        if (obmc)
        {
          m_pcInterSearch->obmcFilter(cu, dstBuf, true, chromaGPM);
        }
      }
      mergeItem->lumaPredReady   = true;
      mergeItem->chromaPredReady = chromaGPM;
    }
    break;

  default:
    THROW("Wrong merge item type");
  }

  auto mergeItemPredBuf = mergeItem->getPredBuf(unitArea);
  if (dstBuf.Y().buf == mergeItemPredBuf.Y().buf)
  {
    // dst is the internal buffer
    mergeItem->lumaPredReady |= luma;
    mergeItem->chromaPredReady |= chroma;
  }
  else if (finalRd)
  {
    // at final RD stage, with and without residuals are both checked
    // it makes sense to buffer the prediction
    mergeItemPredBuf.copyFrom(dstBuf, luma, chroma);

    mergeItem->lumaPredReady |= luma;
    mergeItem->chromaPredReady |= chroma;
  }
  mergeItem->obmcDone = obmc;
  if (mergeItem->mergeItemType != MergeItem::MergeItemType::CIIP)
  {
    mergeItem->licScaleAndOffset.copyFrom(cu.licScaleAndOffset, luma, chroma);
  }
}

double EncCu::calcLumaCost4MergePrediction(const TempCtx &ctxStart, const PelUnitBuf &predBuf, double lambda,
                                           CodingUnit &cu, DistParam &distParam)
{
  distParam.cur              = predBuf.Y();
  auto dist                  = distParam.distFunc(distParam);
  m_CABACEstimator->getCtx() = ctxStart;
  auto   fracBits            = m_pcInterSearch->calcPuMeBits(cu);
  double cost                = (double)dist + (double)fracBits * lambda;
  return cost;
}

unsigned int EncCu::updateRdCheckingNum(MergeItemList &mergeItemList, double threshold, unsigned int numMergeSatdCand)
{
  for (uint32_t i = 0; i < numMergeSatdCand; i++)
  {
    const auto mergeItem = mergeItemList.getMergeItemInList(i);
    if (mergeItem == nullptr || mergeItem->cost > threshold)
    {
      numMergeSatdCand = i;
      break;
    }
  }
  return numMergeSatdCand;
}

void EncCu::checkEarlySkip(const CodingStructure *bestCS, const Partitioner &partitioner)
{
  const CodingUnit &bestCU = *bestCS->getCU(partitioner.chType);
  const CodingUnit &bestPU = bestCU;

  if (bestCU.rootCbf == 0)
  {
    if (bestPU.mergeFlag)
    {
      m_modeCtrl->setEarlySkipDetected();
    }
    else if (m_encCfg->m_motionEstimationSearchMethod != MESearchMethod::SELECTIVE)
    {
      int mvdAbsSum = 0;

      for (auto l: { RPL0, RPL1 })
      {
        if (bestCS->slice->m_numRefIdx[l] > 0)
        {
          mvdAbsSum += bestPU.mvd[l].getAbsHor() + bestPU.mvd[l].getAbsVer();
        }
      }

      if (mvdAbsSum == 0)
      {
        m_modeCtrl->setEarlySkipDetected();
      }
    }
  }
}

void EncCu::addRegularCandsToPruningList(MergeItemList &mergeItemList, const MergeCtx &mergeCtx,
                                         const UnitArea &localUnitArea, double sqrtLambdaForFirstPassIntra,
                                         const TempCtx &ctxStart, PelUnitBufVector<MRG_MAX_NUM_CANDS> *mrgPredBufNoCiip,
                                         PelUnitBufVector<MRG_MAX_NUM_CANDS> *mrgPredBufNoMvRefine,
                                         DistParam &distParam, CodingUnit *cu)
{
  PROFILER_SCOPE(TP_ENABLE_INTER_MERGE_EST_STAGES, g_timeProfiler, P_INTER_MRG_EST_CAND_REG);
  for (uint32_t uiMergeCand = 0; uiMergeCand < mergeCtx.numCandToTestEnc; uiMergeCand++)
  {
    if (cu->oppositeLicFlag && !mergeCtx.LICFlags[uiMergeCand] && !cu->cs->sps->m_biLicEnabledFlag &&
        mergeCtx.interDirNeighbours[uiMergeCand] == 3)
    {
      continue;
    }
    MergeItem *regularMerge = mergeItemList.allocateNewMergeItem();
    regularMerge->importMergeInfo(mergeCtx, uiMergeCand, MergeItem::MergeItemType::REGULAR, *cu);
    auto dstBuf = regularMerge->getPredBuf(localUnitArea);
    // For regular merge cands, both luma and chroma are predicted at the pruning stage.
    // CIIP may be reset back to regular merge in the pass of no residuals. In this case,
    // mrgPredBufNoCiip will be used directly without performing prediction
    generateMergePrediction(localUnitArea, regularMerge, *cu, true, true, dstBuf, false, false,
                            mrgPredBufNoMvRefine ? (*mrgPredBufNoMvRefine)[uiMergeCand] : nullptr,
                            mrgPredBufNoCiip ? (*mrgPredBufNoCiip)[uiMergeCand] : nullptr, &mergeCtx);
    regularMerge->cost = calcLumaCost4MergePrediction(ctxStart, dstBuf, sqrtLambdaForFirstPassIntra, *cu, distParam);
    mergeItemList.insertMergeItemToList(regularMerge);
  }
}

void EncCu::addCiipCandsToPruningList(MergeItemList &mergeItemList, const MergeCtx &mergeCtx,
                                      const UnitArea &localUnitArea, double sqrtLambdaForFirstPassIntra,
                                      const TempCtx &ctxStart, PelUnitBufVector<MRG_MAX_NUM_CANDS> &mrgPredBufNoCiip,
                                      PelUnitBufVector<MRG_MAX_NUM_CANDS> &mrgPredBufNoMvRefine, DistParam &distParam,
                                      CodingUnit *cu)
{
  PROFILER_SCOPE(TP_ENABLE_INTER_MERGE_EST_STAGES, g_timeProfiler, P_INTER_MRG_EST_CAND_CIIP);
  size_t numRegularCands = std::min(NUM_MRG_SATD_CAND, (int)mergeItemList.size());
  static_vector<const MergeItem *, NUM_MRG_SATD_CAND> regularMergeItems;
  for (int index = 0; index < std::min(NUM_MRG_SATD_CAND, (int)mergeItemList.size()); index++)
  {
    regularMergeItems.push_back(mergeItemList.getMergeItemInList(index));
  }

  size_t oldSize = m_mergeItemList.getListSize();
  // temporarily ensure old candidates are not removed
  m_mergeItemList.setListSize(std::max((1 + (int)CIIP_Type::NUM) * numRegularCands, oldSize));

  bool ciipIntraPredReady[(int)CIIP_Type::NUM] = { 0 };

  cu->ciipFlag        = true;
  cu->oppositeLicFlag = false;
  for (int ciipIdx = 0; ciipIdx < (int)CIIP_Type::NUM; ciipIdx++)
  {
    cu->ciipMode = (CIIP_Type)ciipIdx;
    for (auto regularMerge: regularMergeItems)
    {
      const int ciipMergeIdx = regularMerge->mergeIdx;

      if (!ciipIntraPredReady[ciipIdx])
      {
        // calculate intra prediction only once and save in m_pcIntraSearch->getPredictorPtr2
        cu->intraDir[ChannelType::LUMA] = PLANAR_IDX;
        m_pcIntraSearch->initIntraPatternChType(*cu, cu->Y());
        m_pcIntraSearch->predIntraAng(COMP_Y, cu->cs->getPredBuf(*cu).Y(), *cu, true, false);
        m_pcIntraSearch->switchBuffer(*cu, COMP_Y, cu->cs->getPredBuf(*cu).Y(),
                                      m_pcIntraSearch->getPredictorPtr2(COMP_Y, ciipIdx));
        if (isChromaEnabled(cu->chromaFormat))
        {
          cu->intraDir[ChannelType::CHROMA] = DM_CHROMA_IDX;
          m_pcIntraSearch->initIntraPatternChType(*cu, cu->Cb());
          m_pcIntraSearch->predIntraAng(COMP_Cb, cu->cs->getPredBuf(*cu).Cb(), *cu, true, false);
          m_pcIntraSearch->switchBuffer(*cu, COMP_Cb, cu->cs->getPredBuf(*cu).Cb(),
                                        m_pcIntraSearch->getPredictorPtr2(COMP_Cb, ciipIdx));
          m_pcIntraSearch->initIntraPatternChType(*cu, cu->Cr());
          m_pcIntraSearch->predIntraAng(COMP_Cr, cu->cs->getPredBuf(*cu).Cr(), *cu, true, false);
          m_pcIntraSearch->switchBuffer(*cu, COMP_Cr, cu->cs->getPredBuf(*cu).Cr(),
                                        m_pcIntraSearch->getPredictorPtr2(COMP_Cr, ciipIdx));
        }
        ciipIntraPredReady[ciipIdx] = true;
      }
      MergeItem *ciipMerge = mergeItemList.allocateNewMergeItem();
      ciipMerge->importMergeInfo(mergeCtx, ciipMergeIdx, MergeItem::MergeItemType::CIIP, *cu);
      auto dstBuf                  = ciipMerge->getPredBuf(localUnitArea);
      ciipMerge->licScaleAndOffset = regularMerge->licScaleAndOffset;
      generateMergePrediction(localUnitArea, ciipMerge, *cu, true, false, dstBuf, false, false,
                              mrgPredBufNoMvRefine[ciipMerge->mergeIdx], nullptr, &mergeCtx);
      ciipMerge->bdmvrRefine = regularMerge->bdmvrRefine;
      if (ciipMerge->bdmvrRefine)
      {
        ciipMerge->getMvBuf(localUnitArea).copyFrom(regularMerge->getMvBuf(localUnitArea));
      }
      if (cu->cs->slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag)
      {
        // distortion is calculated in the original domain
        PelUnitBuf *tmp = m_pelUnitBufPool.getPelUnitBuf(localUnitArea);
        tmp->copyFrom(dstBuf, true, false);
        tmp->Y().rspSignal(m_pcReshape->m_invLUT);
        ciipMerge->cost = calcLumaCost4MergePrediction(ctxStart, *tmp, sqrtLambdaForFirstPassIntra, *cu, distParam);
        m_pelUnitBufPool.giveBack(tmp);
      }
      else
      {
        ciipMerge->cost = calcLumaCost4MergePrediction(ctxStart, dstBuf, sqrtLambdaForFirstPassIntra, *cu, distParam);
      }
      mergeItemList.insertMergeItemToList(ciipMerge);
    }
  }
  cu->ciipFlag = false;

  m_mergeItemList.setListSize(oldSize);
}

void EncCu::addAffineCandsToPruningList(MergeItemList &mergeItemList, AffineMergeCtx &affineMergeCtx,
                                        const UnitArea &localUnitArea, double sqrtLambdaForFirstPass,
                                        const TempCtx &ctxStart, DistParam &distParam, CodingUnit *cu)
{
  PROFILER_SCOPE(TP_ENABLE_INTER_MERGE_EST_STAGES, g_timeProfiler, P_INTER_MRG_EST_CAND_AFFINE);
  bool sameMV[AFFINE_MRG_MAX_NUM_CANDS + 1] = {
    false,
  };
  if (m_encCfg->m_Affine > 1)
  {
    for (int m = 0; m < affineMergeCtx.numAffCandToTestEnc; m++)
    {
      if ((cu->slice->m_uiTLayer > 3) &&
          (affineMergeCtx.mergeType[m] != MergeType::SUBPU_ATMVP /*MRG_TYPE_SUBPU_ATMVP*/))
      {
        sameMV[m] = m != 0;
      }
      else if (!sameMV[m + 1])
      {
        for (int n = m + 1; n < affineMergeCtx.numValidMergeCand; n++)
        {
          // TODO: check interDir? check all affine candidates?
          if ((affineMergeCtx.mvFieldNeighbours[m][0][0].mv == affineMergeCtx.mvFieldNeighbours[n][0][0].mv) &&
              (affineMergeCtx.mvFieldNeighbours[m][0][1].mv == affineMergeCtx.mvFieldNeighbours[n][0][1].mv))
          {
            sameMV[n] = true;
          }
        }
      }
    }
  }

  if (cu->cs->sps->m_useDMVD && PU::checkBDMVR4Affine(*cu))
  {
    cu->regularMergeFlag = false;
    cu->mergeFlag        = true;
    cu->mmvdMergeFlag    = false;
    cu->affine           = true;
    cu->bdmvrRefine      = false;
    cu->imv              = 0;
    cu->mmvdEncOptMode   = 0;
    cu->mv[0].setZero();
    cu->mv[1].setZero();

    // back to front so that the refined motion doesn't change the result of xCheckSimilarMotion
    for (int candIdx = affineMergeCtx.numAffCandToTestEnc - 1; candIdx >= 0; candIdx--)
    {
      if ((m_encCfg->m_Affine > 1) && sameMV[candIdx])
      {
        continue;
      }
      if (affineMergeCtx.interDirNeighbours[candIdx] == 3 &&
          affineMergeCtx.mergeType[candIdx] != MergeType::SUBPU_ATMVP)
      {
        cu->mergeIdx   = candIdx;
        cu->interDir   = affineMergeCtx.interDirNeighbours[candIdx];
        cu->mergeType  = affineMergeCtx.mergeType[candIdx];
        cu->affineType = affineMergeCtx.affineType[candIdx];
        cu->bcwIdx     = affineMergeCtx.bcwIdx[candIdx];
        cu->licFlag    = affineMergeCtx.LICFlags[candIdx] ^ cu->oppositeLicFlag;
        cu->refIdx[0]  = affineMergeCtx.mvFieldNeighbours[candIdx][0][RPL0].refIdx;
        cu->refIdx[1]  = affineMergeCtx.mvFieldNeighbours[candIdx][0][RPL1].refIdx;
        if (PU::checkBDMVRCondition(*cu) &&
            !affineMergeCtx.xCheckSimilarMotion(cu->mergeIdx, PU::getBDMVRMvdThreshold(*cu)))
        {
          cu->mvAffi[RPL0][0] = affineMergeCtx.mvFieldNeighbours[candIdx][0][RPL0].mv;
          cu->mvAffi[RPL0][1] = affineMergeCtx.mvFieldNeighbours[candIdx][1][RPL0].mv;
          cu->mvAffi[RPL0][2] = affineMergeCtx.mvFieldNeighbours[candIdx][2][RPL0].mv;
          cu->mvAffi[RPL1][0] = affineMergeCtx.mvFieldNeighbours[candIdx][0][RPL1].mv;
          cu->mvAffi[RPL1][1] = affineMergeCtx.mvFieldNeighbours[candIdx][1][RPL1].mv;
          cu->mvAffi[RPL1][2] = affineMergeCtx.mvFieldNeighbours[candIdx][2][RPL1].mv;
          m_pcInterSearch->processBDMVR4Affine(*cu);
          affineMergeCtx.mvFieldNeighbours[candIdx][0][RPL0].mv = cu->mvAffi[RPL0][0];
          affineMergeCtx.mvFieldNeighbours[candIdx][1][RPL0].mv = cu->mvAffi[RPL0][1];
          affineMergeCtx.mvFieldNeighbours[candIdx][2][RPL0].mv = cu->mvAffi[RPL0][2];
          affineMergeCtx.mvFieldNeighbours[candIdx][0][RPL1].mv = cu->mvAffi[RPL1][0];
          affineMergeCtx.mvFieldNeighbours[candIdx][1][RPL1].mv = cu->mvAffi[RPL1][1];
          affineMergeCtx.mvFieldNeighbours[candIdx][2][RPL1].mv = cu->mvAffi[RPL1][2];
          affineMergeCtx.affineType[candIdx]                    = cu->affineType;
        }
      }
    }
  }

  for (uint32_t candIdx = 0; candIdx < affineMergeCtx.numAffCandToTestEnc; candIdx++)
  {
    if ((m_encCfg->m_Affine > 1) && sameMV[candIdx])
    {
      continue;
    }
    if (((!cu->cs->sps->m_biLicEnabledFlag && !affineMergeCtx.LICFlags[candIdx] &&
          affineMergeCtx.interDirNeighbours[candIdx] == 3) ||
         (affineMergeCtx.mergeType[candIdx] == MergeType::SUBPU_ATMVP)) &&
        cu->oppositeLicFlag)
    {
      continue;
    }

    MergeItem *mergeItem = mergeItemList.allocateNewMergeItem();
    mergeItem->importMergeInfo(affineMergeCtx, candIdx,
                               affineMergeCtx.mergeType[candIdx] == MergeType::SUBPU_ATMVP
                                 ? MergeItem::MergeItemType::SBTMVP
                                 : MergeItem::MergeItemType::AFFINE,
                               *cu);
    // mergeItem->licScaleAndOffset.reset();
    auto dstBuf = mergeItem->getPredBuf(localUnitArea);
    generateMergePrediction(localUnitArea, mergeItem, *cu, true, false, dstBuf, false, false, nullptr, nullptr,
                            nullptr);
    mergeItem->cost = calcLumaCost4MergePrediction(ctxStart, dstBuf, sqrtLambdaForFirstPass, *cu, distParam);

    mergeItemList.insertMergeItemToList(mergeItem);
  }
  cu->affine = false;
}

void EncCu::addMmvdCandsToPruningList(MergeItemList &mergeItemList, const MergeCtx &mergeCtx,
                                      const UnitArea &localUnitArea, double sqrtLambdaForFirstPassIntra,
                                      const TempCtx &ctxStart, DistParam &distParam, CodingUnit *cu)
{
  PROFILER_SCOPE(TP_ENABLE_INTER_MERGE_EST_STAGES, g_timeProfiler, P_INTER_MRG_EST_CAND_MMVD);
  cu->mmvdSkip         = true;
  cu->regularMergeFlag = true;
  cu->oppositeLicFlag  = false;
  const int tempNum    = (mergeCtx.numValidMergeCand > 1) ? MmvdIdx::ADD_NUM : MmvdIdx::ADD_NUM >> 1;
  for (int mmvdMergeCand = 0; mmvdMergeCand < tempNum; mmvdMergeCand++)
  {
    MmvdIdx mmvdIdx;
    mmvdIdx.val = mmvdMergeCand;
    if (mmvdIdx.pos.step >= m_encCfg->m_MmvdDisNum)
    {
      continue;
    }
    MergeItem *mmvdMerge = mergeItemList.allocateNewMergeItem();
    mmvdMerge->importMergeInfo(mergeCtx, mmvdIdx.val, MergeItem::MergeItemType::MMVD, *cu);
    auto dstBuf = mmvdMerge->getPredBuf(localUnitArea);
    generateMergePrediction(localUnitArea, mmvdMerge, *cu, true, false, dstBuf, false, false, nullptr, nullptr,
                            nullptr);
    mmvdMerge->cost = calcLumaCost4MergePrediction(ctxStart, dstBuf, sqrtLambdaForFirstPassIntra, *cu, distParam);
    mergeItemList.insertMergeItemToList(mmvdMerge);
  }
}

void EncCu::addAffineMmvdCandsToPruningList(MergeItemList &mergeItemList, AffineMergeCtx &affineMergeCtx,
                                            const UnitArea &localUnitArea, double sqrtLambdaForFirstPass,
                                            const TempCtx &ctxStart, DistParam &distParam, CodingUnit *cu)
{
  int baseIdxToMergeIdxOffset = (int)PU::getMergeIdxFromAffMmvdBaseIdx(affineMergeCtx, 0);
  if (affineMergeCtx.numValidMergeCand <= baseIdxToMergeIdxOffset)
  {
    return;
  }

  cu->affine          = true;
  cu->oppositeLicFlag = false;
  for (uint32_t affMmvdMergeCand = 0; affMmvdMergeCand < MmvdIdx::ADD_NUM; affMmvdMergeCand++)
  {
    MmvdIdx mmvdIdx;
    mmvdIdx.val = affMmvdMergeCand;

    if (mmvdIdx.pos.baseIdx >= AF_MMVD_BASE_NUM || mmvdIdx.pos.position >= AF_MMVD_OFFSET_DIR ||
        mmvdIdx.pos.step >= AF_MMVD_STEP_NUM)
    {
      continue;
    }

    if (affineMergeCtx.mergeType[mmvdIdx.pos.baseIdx + baseIdxToMergeIdxOffset] == MergeType::SUBPU_ATMVP)
    {
      continue;
    }

    MergeItem *mergeItem = mergeItemList.allocateNewMergeItem();
    mergeItem->importMergeInfo(affineMergeCtx, affMmvdMergeCand, MergeItem::MergeItemType::AFF_MMVD, *cu);
    auto dstBuf = mergeItem->getPredBuf(localUnitArea);
    generateMergePrediction(localUnitArea, mergeItem, *cu, true, false, dstBuf, false, false, nullptr, nullptr,
                            nullptr);
    mergeItem->cost = calcLumaCost4MergePrediction(ctxStart, dstBuf, sqrtLambdaForFirstPass, *cu, distParam);

    mergeItemList.insertMergeItemToList(mergeItem);
  }
  cu->affine        = false;
  cu->mmvdMergeFlag = false;
}

template<size_t N> double EncCu::addGpmCandsToPruningList(MergeItemList &mergeItemList, const GeoMergeCtx &geoMergeCtx,
                                                          const UnitArea &localUnitArea, double sqrtLambdaForFirstPass,
                                                          const TempCtx &ctxStart, const GeoComboCostList &comboList,
                                                          const GeoHelper &gh, PelUnitBufVector<N> &geoBuffer,
                                                          DistParam &distParamSAD2, CodingUnit *cu, size_t pos)
{
  double    minLumaCost      = MAX_DOUBLE;
  const int geoNumMrgSadCand = std::min(GEO_MAX_TRY_WEIGHTED_SAD, (int)comboList.list.size());
  cu->bdmvrRefine            = false;
  cu->oppositeLicFlag        = false;
  for (int candidateIdx = 0; candidateIdx < geoNumMrgSadCand; candidateIdx++)
  {
    const int          splitDir     = comboList.list[candidateIdx].splitDir;
    const MergeIdxPair mergeIdxPair = comboList.list[candidateIdx].mergeIdx;
    const uint16_t     gpmBufIdx0   = mergeIdxPair[0] >= GPM_EXT_MMVD_MAX_REFINE_NUM2
            ? mergeIdxPair[0]
            : gh.getGeoBufIntraIdx(splitDir, 0, mergeIdxPair[0]);
    const uint16_t     gpmBufIdx1   = mergeIdxPair[1] >= GPM_EXT_MMVD_MAX_REFINE_NUM2
            ? mergeIdxPair[1]
            : gh.getGeoBufIntraIdx(splitDir, 1, mergeIdxPair[1]);
    const int          bldIdx       = comboList.list[candidateIdx].bldIdx;
    const int          gpmIndex     = MergeItem::getGpmUnfiedIndex(splitDir, bldIdx, mergeIdxPair);

    MergeItem *mergeItem = mergeItemList.allocateNewMergeItem();
    mergeItem->importMergeInfo(geoMergeCtx, gpmIndex, MergeItem::MergeItemType::GPM, *cu);
    auto dstBuf = mergeItem->getPredBuf(localUnitArea);
    generateMergePrediction(localUnitArea, mergeItem, *cu, true, false, dstBuf, false, false, geoBuffer[gpmBufIdx0],
                            geoBuffer[gpmBufIdx1], nullptr);
    mergeItem->cost = calcLumaCost4MergePrediction(ctxStart, dstBuf, sqrtLambdaForFirstPass, *cu, distParamSAD2);
    minLumaCost     = std::min(minLumaCost, mergeItem->cost);

    mergeItemList.insertMergeItemToList(mergeItem, pos);
  }
  return minLumaCost;
}

void EncCu::addBMCandsToPruningList(MergeItemList &mergeItemList, const MergeCtx &mergeCtx,
                                    const UnitArea &localUnitArea, double sqrtLambdaForFirstPassIntra,
                                    const TempCtx &ctxStart, DistParam &distParam, CodingUnit *cu)
{
  PROFILER_SCOPE(TP_ENABLE_INTER_MERGE_EST_STAGES, g_timeProfiler, P_INTER_MRG_EST_CAND_BM);
  cu->bmMergeFlag     = true;
  cu->bdmvrRefine     = true;
  cu->bmDir           = 1;
  cu->oppositeLicFlag = false;
  for (uint32_t uiMergeCand = 0; uiMergeCand < mergeCtx.numCandToTestEnc; uiMergeCand++)
  {
    for (int bmDir = 1; bmDir <= 2; bmDir++)
    {
      MergeItem *regularMerge = mergeItemList.allocateNewMergeItem();
      cu->bmDir               = bmDir;
      regularMerge->importMergeInfo(mergeCtx, uiMergeCand, MergeItem::MergeItemType::REGULAR, *cu);

      auto dstBuf = regularMerge->getPredBuf(localUnitArea);
      generateMergePrediction(localUnitArea, regularMerge, *cu, true, true, dstBuf, false, false, nullptr, nullptr,
                              &mergeCtx);
      regularMerge->cost = calcLumaCost4MergePrediction(ctxStart, dstBuf, sqrtLambdaForFirstPassIntra, *cu, distParam);
      mergeItemList.insertMergeItemToList(regularMerge);
    }
  }
  cu->bmMergeFlag = false;
  cu->bdmvrRefine = false;
  cu->bmDir       = 0;
}

void addGeoCand(std::vector<GeoMergeCombo> &GMCLst, const GeoMergeCombo &gmc, size_t maxEntries)
{
  if (GMCLst.empty())
  {
    GMCLst.push_back(gmc);
    return;
  }

  auto riter = GMCLst.rbegin();
  for (; riter != GMCLst.rend(); riter++)
  {
    if (riter->cost < gmc.cost)
    {
      break;
    }
  }
  GMCLst.insert(riter.base(), gmc);

  if (GMCLst.size() >= maxEntries)
  {
    GMCLst.resize(maxEntries - 1);
  }
}

// new
template<size_t N> bool EncCu::prepareGpmComboList(GeoMergeCtx &geoMergeCtx, const UnitArea &localUnitArea,
                                                   double sqrtLambdaForFirstPass, GeoComboCostList &comboList,
                                                   PelUnitBufVector<N> &geoBuffer, GeoHelper &gh, CodingUnit *cu,
                                                   bool useMMVDS)
{
  const bool extMMVD           = cu->cs->picHeader->m_gpmMMVDTableFlag;
  const bool simpleGPMMMVDStep = (m_encCfg->m_intraPeriod == -1);
  int8_t     maxNumMMVDCand    = useMMVDS ? (extMMVD ? GPM_EXT_MMVD_MAX_REFINE_NUM : GPM_MMVD_MAX_REFINE_NUM) : 0;

  if (simpleGPMMMVDStep)
  {
    maxNumMMVDCand = std::min(maxNumMMVDCand, (int8_t)(extMMVD ? (5 << 3) : (5 << 2)));
  }

  const MergeCtx &mergeCtx = geoMergeCtx[0];

  PelUnitBufVector<GPM_MODES_TOTAL> geoTempBuf(m_pelUnitBufPool);
  uint8_t   maxNumMergeCandidates = std::min((int)cu->cs->sps->m_maxNumGeoCand, mergeCtx.numValidMergeCand);
  DistParam distParam;
  DistParam distParamWholeBlk;

  CodedCUInfo &relatedCU = m_modeCtrl->getCodedCUInfo(cu->cs->area);

  // the third arguments to setDistParam is dummy and will be updated before being used
  m_pcRdCost->setDistParam(distParamWholeBlk, cu->cs->getOrgBuf().Y(), cu->cs->getOrgBuf().Y(),
                           cu->cs->sps->m_bitDepths[ChannelType::LUMA], COMP_Y);
  Distortion bestWholeBlkSad       = MAX_UINT64;
  double     bestWholeIntraBlkCost = MAX_DOUBLE;
  double     bestWholeBlkCost      = MAX_DOUBLE;
  Distortion sadWholeBlk[GPM_MODES_TOTAL];
  int        pocMrg[MRG_MAX_NUM_CANDS];
  bool       isSkipThisCand[MRG_MAX_NUM_CANDS];
  Mv         mergeMv[MRG_MAX_NUM_CANDS];

  const bool useGpmMVD   = m_encCfg->m_Geo & 2;
  const bool useGpmIntra = m_encCfg->m_Geo & 4;
  const int  bldIdxD     = 2;
  const int  numGpmBlds  = m_encCfg->m_Geo & 8 ? GEO_NUM_BLD : 1;

  const size_t numCandPerPar = (m_fastGpmMmvdSearch ? 3 : 4) * numGpmBlds;

  if (useMMVDS)
  {
    if (!useGpmMVD)
    {
      comboList.list.clear();
      return false;
    }

    double costThresh = (comboList.list[0].cost * 1.2);
    for (int i = 0; i < maxNumMergeCandidates; i++)
    {
      isSkipThisCand[i] = true;
      for (size_t j = 0; j < comboList.list.size(); j++)
      {
        if (comboList.list[j].cost < costThresh &&
            ((i == mergeIdxFrom(comboList.list[j].mergeIdx[0])) || (i == mergeIdxFrom(comboList.list[j].mergeIdx[1]))))
        {
          isSkipThisCand[i] = false;
          break;
        }
      }
    }
  }
  else
  {
    for (int i = 0; i < maxNumMergeCandidates; i++)
    {
      isSkipThisCand[i] = false;
    }
  }
  GeoCost &geoCost = gh.geoCost;
  {
    const TempCtx ctxStart(m_ctxPool, m_CABACEstimator->getCtx());
    if (useMMVDS)
    {
      for (int idx = 0; idx < maxNumMMVDCand; idx++)
      {
        geoCost.MMVDIdx[idx] = sqrtLambdaForFirstPass * m_CABACEstimator->geo_mmvdIdx_est(ctxStart, idx, extMMVD);
      }
    }
    else
    {
      for (int idx = 0; idx < GEO_NUM_PARTITION_MODE; idx++)
      {
        geoCost.mode[idx] = sqrtLambdaForFirstPass * m_CABACEstimator->geo_mode_est(ctxStart, idx);
      }
      for (int idx = 0; idx < GEO_MAX_NUM_UNI_CANDS; idx++)
      {
        geoCost.mergeIdx[idx] =
          sqrtLambdaForFirstPass * m_CABACEstimator->geo_mergeIdx_est(ctxStart, idx, cu->cs->sps->m_maxNumGeoCand);
      }
      for (int idx = 0; idx < 2; idx++)
      {
        geoCost.MMVDFlag[idx] =
          useGpmMVD ? sqrtLambdaForFirstPass * m_CABACEstimator->geo_mmvdFlag_est(ctxStart, idx) : 0.0;
      }
      for (int idx = 0; idx < 2; idx++)
      {
        geoCost.TMFlag[idx] = 0.0;
      }
      for (int idx = 0; idx < 2; idx++)
      {
        geoCost.IntraFlag[idx] =
          useGpmIntra ? sqrtLambdaForFirstPass * m_CABACEstimator->geo_intraFlag_est(ctxStart, idx) : 0.0;
      }
      for (int idx = 0; idx < GEO_NUM_BLD; idx++)
      {
        geoCost.BldFlag[idx] =
          numGpmBlds > 1 ? sqrtLambdaForFirstPass * m_CABACEstimator->geo_bld_flag_est(ctxStart, idx) : 0.0;
      }
    }
    m_CABACEstimator->getCtx() = ctxStart;
  }

  // initialize the local bufer
  geoTempBuf.resize(geoTempBuf.max_size(), nullptr);

  if (!useMMVDS && useGpmIntra)   // do not process intra twice
  {
    ClpRng        clpRngDummy;
    const int     shiftDefault  = std::max<int>(2, (IF_INTERNAL_PREC - cu->slice->clpRng(COMP_Y).bd));
    const int32_t offsetDefault = (1 << (shiftDefault - 1)) + IF_INTERNAL_OFFS;
    const bool    chromaEnabled = isChromaEnabled(cu->chromaFormat);
    for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
    {
      for (int partIdx = 0; partIdx < 2; partIdx++)
      {
        PU::getGeoIntraMPMs(*cu, gh.m_geoIntraMPMList[splitDir][partIdx], splitDir,
                            g_geoTmShape[partIdx][g_geoParams[splitDir].angleIdx]);
        for (int intraIdx = 0; intraIdx < GEO_MAX_NUM_INTRA_CANDS; intraIdx++)
        {
          const int      intraPred = gh.geoIntraMPMList(splitDir, partIdx, intraIdx);
          const uint16_t gpmBufIdx = gh.setGeoBufIntraPred(intraPred);

          if (nullptr == geoBuffer[gpmBufIdx])
          {
            cu->intraDir[ChannelType::LUMA] = cu->intraDir[ChannelType::CHROMA] = intraPred;
            geoBuffer[gpmBufIdx] = m_pelUnitBufPool.getPelUnitBuf(localUnitArea);
            m_pcIntraSearch->initIntraPatternChType(*cu, cu->Y());
            m_pcIntraSearch->predIntraAng(COMP_Y, geoBuffer[gpmBufIdx]->Y(), *cu, true, false);
            if (chromaEnabled)
            {
              m_pcIntraSearch->initIntraPatternChType(*cu, cu->Cb());
              m_pcIntraSearch->predIntraAng(COMP_Cb, geoBuffer[gpmBufIdx]->Cb(), *cu, true, false);
              m_pcIntraSearch->initIntraPatternChType(*cu, cu->Cr());
              m_pcIntraSearch->predIntraAng(COMP_Cr, geoBuffer[gpmBufIdx]->Cr(), *cu, true, false);
            }
            geoTempBuf[gpmBufIdx] = m_pelUnitBufPool.getPelUnitBuf(localUnitArea);
            geoTempBuf[gpmBufIdx]->Y().copyFrom(geoBuffer[gpmBufIdx]->Y());

            geoBuffer[gpmBufIdx]->Y().linearTransform(1 << shiftDefault, 0, -offsetDefault, false, clpRngDummy);
            if (chromaEnabled)
            {
              geoBuffer[gpmBufIdx]->Cb().linearTransform(1 << shiftDefault, 0, -offsetDefault, false, clpRngDummy);
              geoBuffer[gpmBufIdx]->Cr().linearTransform(1 << shiftDefault, 0, -offsetDefault, false, clpRngDummy);
            }

            if (cu->slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag)
            {
              geoTempBuf[gpmBufIdx]->Y().rspSignal(m_pcReshape->m_invLUT);
            }

            distParamWholeBlk.cur  = geoTempBuf[gpmBufIdx]->Y();
            sadWholeBlk[gpmBufIdx] = distParamWholeBlk.distFunc(distParamWholeBlk);
            {
              double rateCand       = geoCost.mergeIdx[intraIdx] + geoCost.MMVDFlag[0] + geoCost.IntraFlag[1];
              double testCost       = (double)sadWholeBlk[gpmBufIdx] + rateCand;
              bestWholeIntraBlkCost = std::min(bestWholeIntraBlkCost, testCost);
            }
          }
        }
      }
    }
  }
  // lets try this
  //  bestWholeBlkCost = bestWholeIntraBlkCost;

  for (uint8_t mergeCand = 0; mergeCand < maxNumMergeCandidates; mergeCand++)
  {
    if (isSkipThisCand[mergeCand])   // this might collide with multihyp
    {
      continue;
    }

    mergeCtx.setMergeInfo(*cu, mergeCand);

    const auto refPicList = mergeCtx.mvFieldNeighbours[mergeCand][0].refIdx == -1 ? RPL1 : RPL0;
    const int  refIdx     = mergeCtx.mvFieldNeighbours[mergeCand][refPicList].refIdx;
    pocMrg[mergeCand]     = cu->cs->slice->getRefPOC(refPicList, refIdx);
    mergeMv[mergeCand]    = mergeCtx.mvFieldNeighbours[mergeCand][refPicList].mv;

    for (int i = 0; i < mergeCand; i++)
    {
      if (!isSkipThisCand[i] && (pocMrg[mergeCand] == pocMrg[i] && mergeMv[mergeCand] == mergeMv[i]))
      {
        isSkipThisCand[mergeCand] = true;
        break;
      }
    }
    if (isSkipThisCand[mergeCand])   // this might collide with multihyp
    {
      continue;
    }

    for (int8_t mmvdCand = -1; mmvdCand < maxNumMMVDCand; mmvdCand++)
    {
      const uint16_t gpmBufIdx = geoBufIdx(mergeCand, mmvdCand, 0);

      if (mmvdCand >= 0)
      {
        mergeCtx.setGeoMmvdMergeInfo(*cu, mergeCand, mmvdCand);
      }
      else
      {
        mergeCtx.setMergeInfo(*cu, mergeCand);
      }

      PU::spanMotionInfo(*cu);

      if (m_encCfg->m_seiCfg.m_MCTSEncConstraint && (!(MCTSHelper::checkMvBufferForMCTSConstraint(*cu))))
      {
        return false;
      }

      if (!geoBuffer[gpmBufIdx])
      {
        geoBuffer[gpmBufIdx] = m_pelUnitBufPool.getPelUnitBuf(localUnitArea);
        m_pcInterSearch->motionCompensation(*cu, *geoBuffer[gpmBufIdx], RPLX);
      }

      geoTempBuf[gpmBufIdx] = m_pelUnitBufPool.getPelUnitBuf(localUnitArea);
      geoTempBuf[gpmBufIdx]->Y().copyFrom(geoBuffer[gpmBufIdx]->Y());
      geoTempBuf[gpmBufIdx]->Y().roundToOutputBitdepth(geoTempBuf[gpmBufIdx]->Y(), cu->cs->slice->clpRng(COMP_Y));
      distParamWholeBlk.cur  = geoTempBuf[gpmBufIdx]->Y();
      sadWholeBlk[gpmBufIdx] = distParamWholeBlk.distFunc(distParamWholeBlk);
      {
        bestWholeBlkSad = sadWholeBlk[gpmBufIdx];
        double rateCand = geoCost.mergeIdx[mergeCand] +
          (mmvdCand >= 0 ? geoCost.MMVDFlag[1] + geoCost.MMVDIdx[mmvdCand]
                         : geoCost.MMVDFlag[0] + geoCost.IntraFlag[0] + geoCost.TMFlag[0]);
        double testCost  = (double)bestWholeBlkSad + rateCand;
        bestWholeBlkCost = std::min(bestWholeBlkCost, testCost);
      }
    }
  }

  const int wIdx = floorLog2(cu->lwidth()) - GEO_MIN_CU_LOG2;
  const int hIdx = floorLog2(cu->lheight()) - GEO_MIN_CU_LOG2;

  bool skipDir[GEO_NUM_PARTITION_MODE];
  bool useSkipDir = /*useMMVDS && */ relatedCU.numGeoDirCand > 0;
  if (useSkipDir)
  {
    memset(skipDir, 1, sizeof(skipDir));
    for (int idx = 0; idx < relatedCU.numGeoDirCand; idx++)
    {
      skipDir[relatedCU.geoDirCandList[idx]] = false;
    }
  }

  for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
  {
    if (useSkipDir && skipDir[splitDir])
    {
      continue;
    }
    int     maskStride = 0, maskStride2 = 0;
    int     stepX = 1;
    Pel    *sadMask;
    int16_t angle = g_geoParams[splitDir].angleIdx;
    if (g_angle2mirror[angle] == 2)
    {
      maskStride  = -GEO_WEIGHT_MASK_SIZE;
      maskStride2 = -(int)cu->lwidth();
      sadMask     = &g_globalGeoEncSADmask[g_angle2mask[g_geoParams[splitDir].angleIdx]]
                                      [(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) *
                                         GEO_WEIGHT_MASK_SIZE +
                                       g_weightOffset[splitDir][hIdx][wIdx][0]];
    }
    else if (g_angle2mirror[angle] == 1)
    {
      stepX       = -1;
      maskStride2 = cu->lwidth();
      maskStride  = GEO_WEIGHT_MASK_SIZE;
      sadMask     = &g_globalGeoEncSADmask[g_angle2mask[g_geoParams[splitDir].angleIdx]]
                                      [g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE +
                                       (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
    }
    else
    {
      maskStride  = GEO_WEIGHT_MASK_SIZE;
      maskStride2 = -(int)cu->lwidth();
      sadMask     = &g_globalGeoEncSADmask[g_angle2mask[g_geoParams[splitDir].angleIdx]]
                                      [g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE +
                                       g_weightOffset[splitDir][hIdx][wIdx][0]];
    }

    m_pcRdCost->setDistParam(distParam, cu->cs->getOrgBuf().Y(), nullptr, 0, sadMask, maskStride, stepX, maskStride2,
                             cu->cs->sps->m_bitDepths[ChannelType::LUMA], COMP_Y);
    for (uint8_t mergeCand = 0; mergeCand < maxNumMergeCandidates; mergeCand++)
    {
      if (!isSkipThisCand[mergeCand])
      {
        for (int8_t mmvdCand = useMMVDS ? 0 : -1; mmvdCand < maxNumMMVDCand; mmvdCand++)
        {
          const uint16_t gpmBufIdx  = geoBufIdx(mergeCand, mmvdCand, 0);
          distParam.cur.buf         = geoTempBuf[gpmBufIdx]->Y().buf;
          distParam.cur.stride      = geoTempBuf[gpmBufIdx]->Y().stride;
          const Distortion sadLarge = distParam.distFunc(distParam);
          const Distortion sadSmall = sadWholeBlk[gpmBufIdx] - sadLarge;
          double           rateCand = geoCost.mergeIdx[mergeCand] +
            (mmvdCand >= 0 ? geoCost.MMVDFlag[1] + geoCost.MMVDIdx[mmvdCand]
                           : geoCost.IntraFlag[0] + geoCost.MMVDFlag[0] + geoCost.TMFlag[0] / 2);
          const double cost0 = (double)sadLarge + rateCand;
          const double cost1 = (double)sadSmall + rateCand;

          m_geoCostList.insert(splitDir, 0, gpmBufIdx, cost0);
          m_geoCostList.insert(splitDir, 1, gpmBufIdx, cost1);
        }
      }
    }

    if (!useMMVDS && useGpmIntra)
    {
      for (int partIdx = 0; partIdx < 2; partIdx++)
      {
        for (int intraIdx = 0; intraIdx < GEO_MAX_NUM_INTRA_CANDS; intraIdx++)
        {
          const uint16_t gpmBufIdx  = gh.getGeoBufIntraIdx(splitDir, partIdx, intraIdx);
          distParam.cur.buf         = geoTempBuf[gpmBufIdx]->Y().buf;
          distParam.cur.stride      = geoTempBuf[gpmBufIdx]->Y().stride;
          const Distortion sadLarge = distParam.distFunc(distParam);
          const Distortion sadSmall = sadWholeBlk[gpmBufIdx] - sadLarge;
          double           rateCand = geoCost.mergeIdx[intraIdx] + geoCost.IntraFlag[1] + geoCost.MMVDFlag[0];
          const double     cost     = (double)((partIdx == 0) ? sadLarge : sadSmall) + rateCand;
          m_geoCostList.insert(splitDir, partIdx, intraIdx, cost);
        }
      }
    }
  }

  double costThr = 1.3 * bestWholeBlkCost;   // 3
  comboList.list.clear();
  for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
  {
    m_geoLists[splitDir].clear();
    if (useSkipDir && skipDir[splitDir])
    {
      continue;
    }
    const int mcs = useGpmIntra ? -1 : 0;
    for (int mergeCand0 = mcs; mergeCand0 < maxNumMergeCandidates; mergeCand0++)
    {
      const bool mergeCand0Intra = mergeCand0 == -1;
      if (mergeCand0Intra || !isSkipThisCand[mergeCand0])
      {
        int8_t minNumMMVDCand0 = mergeCand0Intra ? 0 : -1;
        int8_t maxNumMMVDCand0 = mergeCand0Intra ? GEO_MAX_NUM_INTRA_CANDS : maxNumMMVDCand;
        for (int mergeCand1 = (mergeCand0Intra ? 0 : mcs); mergeCand1 < maxNumMergeCandidates; mergeCand1++)
        {
          const bool mergeCand1Intra = mergeCand1 == -1;
          if (mergeCand1Intra || !isSkipThisCand[mergeCand1])
          {
            int8_t minNumMMVDCand1 = mergeCand1Intra ? 0 : -1;
            int8_t maxNumMMVDCand1 = mergeCand1Intra ? GEO_MAX_NUM_INTRA_CANDS : maxNumMMVDCand;
            for (int8_t mmvdCand0 = minNumMMVDCand0; mmvdCand0 < maxNumMMVDCand0; mmvdCand0++)
            {
              uint16_t gpmBufIdx0 = mergeCand0Intra ? mmvdCand0 : geoBufIdx(mergeCand0, mmvdCand0, 0);
              for (int8_t mmvdCand1 = minNumMMVDCand1; mmvdCand1 < maxNumMMVDCand1; mmvdCand1++)
              {
                uint16_t gpmBufIdx1 = mergeCand1Intra ? mmvdCand1 : geoBufIdx(mergeCand1, mmvdCand1, 0);
                if (gpmBufIdx0 == gpmBufIdx1 ||
                    (useMMVDS && !((!mergeCand0Intra && mmvdCand0 >= 0) || (!mergeCand1Intra && mmvdCand1 >= 0))))
                {
                  continue;
                }
                CHECK(gpmBufIdx0 >= GEO_MAX_NUM_INTRA_CANDS && gpmBufIdx0 < GPM_EXT_MMVD_MAX_REFINE_NUM2, "bad value");
                CHECK(gpmBufIdx1 >= GEO_MAX_NUM_INTRA_CANDS && gpmBufIdx1 < GPM_EXT_MMVD_MAX_REFINE_NUM2, "bad value");
                MergeIdxPair mergeIdxPair { gpmBufIdx0, gpmBufIdx1 };

                double tempCost = m_geoCostList.getCost(splitDir, mergeIdxPair);   // limit to 4 cand per split dir
                if (tempCost < costThr)
                {
                  if (numGpmBlds > 1)
                  {
                    for (int bldIdx = 0; bldIdx < numGpmBlds; bldIdx++)
                    {
                      addGeoCand(m_geoLists[splitDir],
                                 GeoMergeCombo(splitDir, bldIdx, mergeIdxPair,
                                               tempCost + geoCost.mode[splitDir] + geoCost.BldFlag[bldIdx]),
                                 numCandPerPar);
                    }
                  }
                  else
                  {
                    addGeoCand(m_geoLists[splitDir],
                               GeoMergeCombo(splitDir, bldIdxD, mergeIdxPair, tempCost + geoCost.mode[splitDir]),
                               numCandPerPar);
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
  {
    if (!m_geoLists[splitDir].empty())
    {
      comboList.list.insert(comboList.list.end(), m_geoLists[splitDir].begin(), m_geoLists[splitDir].end());
    }
  }

  if (comboList.list.empty())
  {
    return false;
  }

  comboList.sortByCost();

  if (relatedCU.numGeoDirCand == 0)
  {
    uint8_t i = 0;
    for (uint8_t j = 0; i < GEO_MAX_TRY_WEIGHTED_SATD && j < (int)comboList.list.size(); j++)
    {
      if (&relatedCU.geoDirCandList[i] ==
          std::find(&relatedCU.geoDirCandList[0], &relatedCU.geoDirCandList[i], comboList.list[j].splitDir))
      {
        relatedCU.geoDirCandList[i++] = comboList.list[j].splitDir;
      }
    }
    relatedCU.numGeoDirCand = i;
  }

  return true;
}

void EncCu::xCheckRDCostIBCMode(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner,
                                const EncTestMode &encTestMode)
{
  if (tempCS->area.lwidth() > IBC_MAX_CU_SIZE || tempCS->area.lheight() > IBC_MAX_CU_SIZE)
  {
    // disable IBC mode larger than 64x64
    return;
  }
  PROFILER_SCOPE(1, g_timeProfiler, P_IBC);

  tempCS->initStructData(encTestMode.qp);

  CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);

  partitioner.setCUData(cu);
  cu.slice       = tempCS->slice;
  cu.tileIdx     = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
  cu.skip        = false;
  cu.predMode    = MODE_IBC;
  cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
  cu.qp          = encTestMode.qp;
  cu.imv         = 0;
  cu.sbtInfo     = 0;

  cu.mmvdSkip         = false;
  cu.mmvdMergeFlag    = false;
  cu.regularMergeFlag = false;
  CHECK(cu.licFlag, "LIC flag is set");

  cu.intraDir[ChannelType::LUMA]   = DC_IDX;   // set intra pred for ibc block
  cu.intraDir[ChannelType::CHROMA] = PLANAR_IDX;   // set intra pred for ibc block

  cu.interDir     = 1;   // use list 0 for IBC mode
  cu.refIdx[RPL0] = IBC_REF_IDX;   // last idx in the list
  bool bValid =
    m_pcInterSearch->predIBCSearch(cu, partitioner, m_ctuIbcSearchRangeX, m_ctuIbcSearchRangeY, m_ibcHashMap);

  if (bValid)
  {
    PU::spanMotionInfo(cu);
    const bool chroma  = !CS::isDualITree(*tempCS);
    //  MC
    PelUnitBuf predBuf = cu.cs->getPredBuf(cu);
    m_pcInterSearch->motionCompensation(cu, predBuf, RPL0, true, chroma, nullptr, true);

    m_pcInterSearch->setHistBestTrs(MAX_UCHAR, MtsType::NONE);
    m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, false, true, chroma);
    xEncodeDontSplit(*tempCS, partitioner);
    xCheckDQP(*tempCS, partitioner);
    xCheckChromaQPOffset(*tempCS, partitioner);
    xCalDebCost(*tempCS, partitioner);
    xCheckBestMode(tempCS, bestCS, partitioner, encTestMode, m_encCfg->m_encDbOpt);
  }   // bValid
  else
  {
    tempCS->dist         = 0;
    tempCS->fracBits     = 0;
    tempCS->cost         = MAX_DOUBLE;
    tempCS->costDbOffset = 0;
  }
}
// check ibc mode in encoder RD
//////////////////////////////////////////////////////////////////////////////////////////////

void EncCu::xCheckRDCostInter(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner,
                              const EncTestMode &encTestMode)
{
  const bool lic = (encTestMode.opts & ETO_LIC) != 0;
  if (m_pcInterSearch->m_fastLicCtrl.skipRDCheckForLIC(lic, IMV_OFF, bestCS->cost, tempCS->area.Y().area()))
  {
    return;
  }
  PROFILER_SCOPE(1, g_timeProfiler, P_INTER_MVD);

  m_pcInterSearch->setAffineModeSelected(false);
  m_pcInterSearch->setDoAffineLic(true);

  m_pcInterSearch->resetBufferedUniMotions();

  const int bcwLoopNum = tempCS->slice->isInterB() && tempCS->sps->m_useBcw &&
      tempCS->area.lwidth() * tempCS->area.lheight() >= BCW_SIZE_CONSTRAINT && (tempCS->sps->m_biLicEnabledFlag || !lic)
    ? BCW_NUM
    : 1;

  double curBestCost = bestCS->cost;
  double equBcwCost  = MAX_DOUBLE;

  for (int bcwLoopIdx = 0; bcwLoopIdx < bcwLoopNum; bcwLoopIdx++)
  {
    if (m_encCfg->m_BcwFast)
    {
      const CodedCUInfo &codedCUInfo = m_modeCtrl->getBlkInfo(bestCS->area);
      if (codedCUInfo.isInter && g_BcwSearchOrder[bcwLoopIdx] != BCW_DEFAULT &&
          g_BcwSearchOrder[bcwLoopIdx] != codedCUInfo.bcwIdx)
      {
        continue;
      }
    }
    if (!tempCS->slice->m_checkLdc)
    {
      if (bcwLoopIdx != 0 && bcwLoopIdx != 3 && bcwLoopIdx != 4)
      {
        continue;
      }
    }

    tempCS->initStructData(encTestMode.qp);

    CodingUnit &cu = tempCS->addCU(tempCS->area, partitioner.chType);

    partitioner.setCUData(cu);
    cu.slice       = tempCS->slice;
    cu.tileIdx     = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
    cu.skip        = false;
    cu.mmvdSkip    = false;
    cu.predMode    = MODE_INTER;
    cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
    cu.qp          = encTestMode.qp;

    cu.bcwIdx       = g_BcwSearchOrder[bcwLoopIdx];
    uint8_t bcwIdx  = cu.bcwIdx;
    bool    testBcw = bcwIdx != BCW_DEFAULT;

    cu.licFlag = lic;
    if (cu.slice->m_useLic && lic)
    {
      m_pcInterSearch->swapUniMvBuffer();
    }
    m_pcInterSearch->predInterSearch(cu, partitioner);
    if (cu.slice->m_useLic && lic)
    {
      m_pcInterSearch->swapUniMvBuffer();
    }
    if (cu.licFlag)
    {
      if (!PU::checkRprLicCondition(cu))   // To check whether LIC actually performs in MC
      {
        cu.licFlag = false;
        PU::spanLicFlags(cu, false);
      }
    }
    bcwIdx = CU::getValidBcwIdx(cu);
    if (testBcw && bcwIdx == BCW_DEFAULT)   // Enabled BCW but the search results is uni
    {
      continue;
    }
    CHECK(!testBcw && bcwIdx != BCW_DEFAULT, "Bad BCW index");

    const bool obmcDone = xApplyObmc(*tempCS, cu);
    const bool newBest  = xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, 0, nullptr, &equBcwCost);

    if (obmcDone && newBest && bestCS->cost != MAX_DOUBLE)
    {
      xUpdateBestObmcCS(*bestCS, partitioner, encTestMode);
    }

    if (!testBcw)
    {
      m_pcInterSearch->setAffineModeSelected(bestCS->cus.front()->affine && !(bestCS->cus.front()->mergeFlag));
    }

    if (m_encCfg->m_BcwFast)
    {
      if (equBcwCost > curBestCost * BCW_COST_TH)
      {
        break;
      }
      if (!testBcw && cu.interDir != 3 && m_encCfg->m_isLowDelay)
      {
        break;
      }
      if (!testBcw && xIsBcwSkip(cu))
      {
        break;
      }
    }
  }
}

bool EncCu::xCheckRDCostInterAmvr(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner,
                                  const EncTestMode &encTestMode, double &bestIntPelCost)
{
  PROFILER_SCOPE(1, g_timeProfiler, P_INTER_MVD_AMVR);

  const auto amvrSearchMode = encTestMode.getAmvrSearchMode();
  m_pcInterSearch->setAffineModeSelected(false);
  m_pcInterSearch->setDoAffineLic(true);
  // Only Half-Pel, int-Pel, 4-Pel and fast 4-Pel allowed
  CHECK(amvrSearchMode < EncTestMode::AmvrSearchMode::FULL_PEL ||
          amvrSearchMode > EncTestMode::AmvrSearchMode::HALF_PEL,
        "Unsupported AMVR Mode");
  const bool testAltHpelFilter = amvrSearchMode == EncTestMode::AmvrSearchMode::HALF_PEL;
  // Fast 4-Pel Mode

  const bool lic = (encTestMode.opts & ETO_LIC) != 0;
  {
    const int iIMV = static_cast<int>(amvrSearchMode);
    if (m_pcInterSearch->m_fastLicCtrl.skipRDCheckForLIC(lic, (iIMV <= 2 ? iIMV : iIMV - 1), bestCS->cost,
                                                         tempCS->area.Y().area()))
    {
      return false;
    }
  }

  EncTestMode encTestModeBase = encTestMode;   // copy for clearing non-IMV options
  encTestModeBase.opts = EncTestModeOpts(encTestModeBase.opts & ETO_IMV);   // clear non-IMV options (is that intended?)

  m_pcInterSearch->resetBufferedUniMotions();

  const int bcwLoopNum = tempCS->slice->isInterB() && tempCS->sps->m_useBcw &&
      tempCS->area.lwidth() * tempCS->area.lheight() >= BCW_SIZE_CONSTRAINT && (tempCS->sps->m_biLicEnabledFlag || !lic)
    ? BCW_NUM
    : 1;

  bool   validMode   = false;
  double curBestCost = bestCS->cost;
  double equBcwCost  = MAX_DOUBLE;

  for (int bcwLoopIdx = 0; bcwLoopIdx < bcwLoopNum; bcwLoopIdx++)
  {
    if (m_encCfg->m_BcwFast)
    {
      const CodedCUInfo &codedCUInfo = m_modeCtrl->getBlkInfo(bestCS->area);
      if (codedCUInfo.isInter && g_BcwSearchOrder[bcwLoopIdx] != BCW_DEFAULT &&
          g_BcwSearchOrder[bcwLoopIdx] != codedCUInfo.bcwIdx)
      {
        continue;
      }
    }

    if (!tempCS->slice->m_checkLdc)
    {
      if (bcwLoopIdx != 0 && bcwLoopIdx != 3 && bcwLoopIdx != 4)
      {
        continue;
      }
    }

    if (m_encCfg->m_BcwFast && tempCS->slice->m_checkLdc && g_BcwSearchOrder[bcwLoopIdx] != BCW_DEFAULT &&
        (m_bestBcwIdx[0] != BCW_NUM && g_BcwSearchOrder[bcwLoopIdx] != m_bestBcwIdx[0]) &&
        (m_bestBcwIdx[1] != BCW_NUM && g_BcwSearchOrder[bcwLoopIdx] != m_bestBcwIdx[1]))
    {
      continue;
    }

    tempCS->initStructData(encTestMode.qp);

    CodingUnit &cu = tempCS->addCU(tempCS->area, partitioner.chType);

    partitioner.setCUData(cu);
    cu.slice       = tempCS->slice;
    cu.tileIdx     = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
    cu.skip        = false;
    cu.mmvdSkip    = false;
    cu.predMode    = MODE_INTER;
    cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
    cu.qp          = encTestMode.qp;

    if (testAltHpelFilter)
    {
      cu.imv = IMV_HPEL;
    }
    else
    {
      cu.imv = amvrSearchMode == EncTestMode::AmvrSearchMode::FULL_PEL ? IMV_FPEL : IMV_4PEL;
    }

    const bool affineAmvrEnabledFlag = !testAltHpelFilter && cu.slice->m_sps->m_affineAmvrEnabledFlag;

    cu.bcwIdx = g_BcwSearchOrder[bcwLoopIdx];

    uint8_t    bcwIdx  = cu.bcwIdx;
    const bool testBcw = bcwIdx != BCW_DEFAULT;

    cu.interDir = 10;

    cu.licFlag = lic;
    if (cu.slice->m_useLic && lic)
    {
      m_pcInterSearch->swapUniMvBuffer();
    }
    if (m_encCfg->m_fastLICAffine)
    {
      m_pcInterSearch->setDoAffineLic(bestCS->cus.front()->affine || bestCS->cus.front()->licFlag);
    }
    m_pcInterSearch->predInterSearch(cu, partitioner);
    if (cu.slice->m_useLic && lic)
    {
      m_pcInterSearch->swapUniMvBuffer();
    }

    if (cu.interDir <= 3)
    {
      bcwIdx = CU::getValidBcwIdx(cu);
      CHECK(!testBcw && bcwIdx != BCW_DEFAULT, "Bad BCW index");
    }
    else
    {
      return false;
    }
    if (cu.licFlag)
    {
      if (!PU::checkRprLicCondition(cu))   // To check whether LIC actually performs in MC
      {
        cu.licFlag = false;
        PU::spanLicFlags(cu, false);
      }
    }
    if (m_encCfg->m_seiCfg.m_MCTSEncConstraint &&
        ((cu.refIdx[RPL0] < 0 && cu.refIdx[RPL1] < 0) || !MCTSHelper::checkMvBufferForMCTSConstraint(cu)))
    {
      // Do not use this mode
      continue;
    }
    if (testBcw && bcwIdx == BCW_DEFAULT)   // Enabled Bcw but the search results is uni.
    {
      continue;
    }

    if (!CU::hasSubCUNonZeroMVd(cu) && !CU::hasSubCUNonZeroAffineMVd(cu))
    {
      if (m_modeCtrl->useModeResult(encTestModeBase, tempCS, partitioner, false))
      {
        std::swap(tempCS, bestCS);
        // store temp best CI for next CU coding
        m_CurrCtx->best = m_CABACEstimator->getCtx();
      }
      if (affineAmvrEnabledFlag)
      {
        continue;
      }
      else
      {
        return false;
      }
    }

    const bool obmcDone = xApplyObmc(*tempCS, cu);
    const bool newBest  = xEncodeInterResidual(tempCS, bestCS, partitioner, encTestModeBase, 0, nullptr, &equBcwCost);

    if (obmcDone && newBest && bestCS->cost != MAX_DOUBLE)
    {
      xUpdateBestObmcCS(*bestCS, partitioner, encTestMode);
    }

    if (cu.imv == IMV_FPEL && tempCS->cost < bestIntPelCost)
    {
      bestIntPelCost = tempCS->cost;
    }

    if (m_encCfg->m_BcwFast)
    {
      // Early termination conditions
      if (equBcwCost > curBestCost * BCW_COST_TH)
      {
        break;
      }
      if (!testBcw && cu.interDir != 3 && m_encCfg->m_isLowDelay)
      {
        break;
      }
      if (!testBcw && xIsBcwSkip(cu))
      {
        break;
      }
    }

    validMode = true;
  }

  return tempCS->slice->m_sps->m_affineAmvrEnabledFlag ? validMode : true;
}

bool EncCu::xApplyObmc(CodingStructure &cs, CodingUnit &cu)
{
  bool obmcDone = false;
  if (CU::isObmcAllowed(cu))
  {
    const UnitArea &area    = cs.area;
    PelUnitBuf      predBuf = cs.getPredBuf(area);
    PelUnitBuf tmpBuf = m_predWoObmcTmp->getBuf(UnitArea(area.chromaFormat, Area(0, 0, area.lwidth(), area.lheight())));
    cu.obmcFlag       = true;
    tmpBuf.copyFrom(predBuf);
    bool applyToChroma = isChromaEnabled(area.chromaFormat);
    obmcDone           = m_pcInterSearch->obmcFilter(cu, predBuf, true, applyToChroma);
  }
  return obmcDone;
}

void EncCu::xUpdateBestObmcCS(CodingStructure &cs, Partitioner &pm, const EncTestMode &etm)
{
  std::swap(m_predWoObmcTmp, m_predWoObmcBest);
  CodingStructure *obmcCS = m_obmcCS[pm.currDepth];
  obmcCS->initStructData(etm.qp);
  obmcCS->copyStructure(cs, pm.chType);
  m_etmObmcBest = etm;
}

void EncCu::xCheckRDCostInterWoObmc(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm)
{
  PROFILER_SCOPE(0, g_timeProfiler, P_INTER_MVD_WO_OBMC);

  // restore CU and prediction signal without OBMC
  const UnitArea  &area   = tempCS->area;
  CodingStructure *obmcCS = m_obmcCS[pm.currDepth];
  EncTestMode      etm    = m_etmObmcBest;
  tempCS->initStructData(etm.qp);
  tempCS->copyStructure(*obmcCS, pm.chType);
  CHECK(tempCS->cus.size() != 1, "only one CU per CS expected");
  CHECK(etm.type == ETM_INVALID || etm.opts == ETO_INVALID, "invalid CU test mode");

  PelUnitBuf predBuf = tempCS->getPredBuf(area);
  PelUnitBuf bestBuf = m_predWoObmcBest->getBuf(UnitArea(area.chromaFormat, Area(0, 0, area.lwidth(), area.lheight())));

  // check early abort
  if (bestCS->cus.size() && bestCS->cus[0]->obmcFlag)
  {
    const int        bd      = tempCS->sps->m_bitDepths[ChannelType::LUMA];
    const CPelBuf   &org     = tempCS->getOrgBuf(area.Y());
    const Distortion obmcOff = m_pcRdCost->getDistPart(org, bestBuf.Y(), bd, COMP_Y, DFunc::SAD_FULL_NBIT);
    const Distortion obmcOn =
      m_pcRdCost->getDistPart(org, bestCS->getPredBuf(area).Y(), bd, COMP_Y, DFunc::SAD_FULL_NBIT);
    const double thOff = 1.0;
    if (thOff * obmcOff >= obmcOn)
    {
      return;
    }
  }

  predBuf.copyFrom(bestBuf);

  // clear obmc flag
  CodingUnit &cu = *tempCS->cus.front();
  CHECK(!cu.obmcFlag, "expect to do inter without OBMC for an OBMC cu");
  cu.obmcFlag = false;

  // test CU without OBMC
  double equBcwCost = MAX_DOUBLE;

  xEncodeInterResidual(tempCS, bestCS, pm, etm, 0, nullptr, &equBcwCost);
}

void EncCu::xCalDebCost(CodingStructure &cs, Partitioner &partitioner)
{
  PROFILER_SCOPE(1, g_timeProfiler, P_DEBLOCK_FILTER);
  if (cs.cost == MAX_DOUBLE)
  {
    cs.costDbOffset = 0;
  }

  if (cs.slice->m_deblockingFilterDisable || !m_encCfg->m_encDbOpt)
  {
    return;
  }

  m_deblockingFilter->setEnc(true);
  const ChromaFormat format       = cs.area.chromaFormat;
  CodingUnit        *cu           = cs.getCU(partitioner.chType);
  const Position     lumaPos      = cu->Y().valid()
             ? cu->Y().pos()
             : recalcPosition(format, cu->chType, ChannelType::LUMA, cu->block(cu->chType).pos());
  bool               topEdgeAvai  = lumaPos.y > 0 && ((lumaPos.y % 4) == 0);
  bool               leftEdgeAvai = lumaPos.x > 0 && ((lumaPos.x % 4) == 0);
  bool               anyEdgeAvai  = topEdgeAvai || leftEdgeAvai;
  cs.costDbOffset                 = 0;

  if (anyEdgeAvai)
  {
    CompID compStr = (CS::isDualITree(cs) && !isLuma(partitioner.chType)) ? COMP_Cb : COMP_Y;
    CompID compEnd =
      (CS::isDualITree(cs) && isLuma(partitioner.chType)) || !isChromaEnabled(cs.area.chromaFormat) ? COMP_Y : COMP_Cr;

    const UnitArea currCsArea = clipArea(cs.area, *cs.picture);

    PelStorage &picDbBuf = m_deblockingFilter->getDbEncPicYuvBuffer();

    // deblock neighbour pixels
    const Size lumaSize = cu->Y().valid()
      ? cu->Y().size()
      : recalcSize(format, cu->chType, ChannelType::LUMA, cu->block(cu->chType).size());

    const int verOffset = lumaPos.y > 7 ? (lumaPos.y > 15 ? 16 : 8) : 4;
    const int horOffset = lumaPos.x > 7 ? (lumaPos.x > 15 ? 16 : 8) : 4;

    const UnitArea areaTop(format, Area(lumaPos.x, lumaPos.y - verOffset, lumaSize.width, verOffset));
    const UnitArea areaLeft(format, Area(lumaPos.x - horOffset, lumaPos.y, horOffset, lumaSize.height));

    for (int compIdx = compStr; compIdx <= compEnd; compIdx++)
    {
      CompID compId = (CompID)compIdx;

      // Copy current CU's reco to Deblock Pic Buffer
      const CompArea &curCompArea = currCsArea.block(compId);
      picDbBuf.getBuf(curCompArea).copyFrom(cs.getRecoBuf(curCompArea));
      if (cs.slice->m_lmcsEnabledFlag && m_pcReshape->m_sliceReshapeInfo.sliceReshaperEnableFlag && isLuma(compId))
      {
        picDbBuf.getBuf(curCompArea).rspSignal(m_pcReshape->m_invLUT);
      }

      // left neighbour
      if (leftEdgeAvai)
      {
        const CompArea &compArea = areaLeft.block(compId);
        picDbBuf.getBuf(compArea).copyFrom(cs.picture->getRecoBuf(compArea));
        if (cs.slice->m_lmcsEnabledFlag && m_pcReshape->m_sliceReshapeInfo.sliceReshaperEnableFlag && isLuma(compId))
        {
          picDbBuf.getBuf(compArea).rspSignal(m_pcReshape->m_invLUT);
        }
      }
      // top neighbour
      if (topEdgeAvai)
      {
        const CompArea &compArea = areaTop.block(compId);
        picDbBuf.getBuf(compArea).copyFrom(cs.picture->getRecoBuf(compArea));
        if (cs.slice->m_lmcsEnabledFlag && m_pcReshape->m_sliceReshapeInfo.sliceReshaperEnableFlag && isLuma(compId))
        {
          picDbBuf.getBuf(compArea).rspSignal(m_pcReshape->m_invLUT);
        }
      }
    }
    // Bilateral:
    // The CU itself, the above area and the area to the left have been copied into
    //     PelStorage&          picDbBuf = m_pcLoopFilter->getDbEncPicYuvBuffer();
    //  It is now possible to insert the code for bilateral filtering here.

    if (cs.pps->m_BIF && (!CS::isDualITree(cs) || isLuma(partitioner.chType)) && cu->Y().valid())
    {
      if (leftEdgeAvai && topEdgeAvai)
      {
        Pel *pDst = picDbBuf.bufs[COMP_Y].buf + (currCsArea.block(COMP_Y).y - 1) * picDbBuf.bufs[COMP_Y].stride +
          currCsArea.block(COMP_Y).x - 1;
        Pel *pSrc = cs.picture->getRecoBuf().bufs[COMP_Y].buf +
          (currCsArea.block(COMP_Y).y - 1) * cs.picture->getRecoBuf().bufs[COMP_Y].stride + currCsArea.block(COMP_Y).x -
          1;
        if (cs.slice->m_lmcsEnabledFlag && m_pcReshape->m_sliceReshapeInfo.sliceReshaperEnableFlag)
        {
          *pDst = m_pcReshape->m_invLUT[*pSrc];
        }
        else
        {
          *pDst = *pSrc;
        }
      }

      for (auto &currTU: CU::traverseTUs(*cu))
      {
        bool applyBIF = m_bilateralFilter->getApplyBIF(currTU, COMP_Y);
        if (applyBIF)
        {
          CompArea &compArea      = currTU.block(COMP_Y);
          PelBuf    recBuf        = picDbBuf.getBuf(compArea);
          PelBuf    recIPredBuf   = recBuf;
          PelBuf    recIPredBufCU = picDbBuf.getBuf(cu->block(COMP_Y));
          m_bilateralFilter->bilateralFilterRDOdiamond5x5(COMP_Y, recBuf, recBuf, recBuf, currTU.cu->qp, recIPredBuf,
                                                          recIPredBufCU, cs.slice->clpRng(COMP_Y), currTU, true);
        }
      }
    }
    if (cs.pps->m_chromaBIF)
    {
      if (leftEdgeAvai && topEdgeAvai)
      {
        Pel *pDst = picDbBuf.bufs[COMP_Cb].buf + (currCsArea.block(COMP_Cb).y - 1) * picDbBuf.bufs[COMP_Cb].stride +
          currCsArea.block(COMP_Cb).x - 1;
        Pel *pSrc = cs.picture->getRecoBuf().bufs[COMP_Cb].buf +
          (currCsArea.block(COMP_Cb).y - 1) * cs.picture->getRecoBuf().bufs[COMP_Cb].stride +
          currCsArea.block(COMP_Cb).x - 1;
        *pDst = *pSrc;

        pDst = picDbBuf.bufs[COMP_Cr].buf + (currCsArea.block(COMP_Cr).y - 1) * picDbBuf.bufs[COMP_Cr].stride +
          currCsArea.block(COMP_Cr).x - 1;
        pSrc = cs.picture->getRecoBuf().bufs[COMP_Cr].buf +
          (currCsArea.block(COMP_Cr).y - 1) * cs.picture->getRecoBuf().bufs[COMP_Cr].stride +
          currCsArea.block(COMP_Cr).x - 1;
        *pDst = *pSrc;
      }

      for (auto &currTU: CU::traverseTUs(*cu))
      {
        for (int compIdx = COMP_Cb; compIdx < MAX_NUM_COMP; compIdx++)
        {
          CompID compID         = CompID(compIdx);
          bool   applyChromaBIF = m_bilateralFilter->getApplyBIF(currTU, compID);
          if (applyChromaBIF)
          {
            CompArea &compArea      = currTU.block(compID);
            PelBuf    recBuf        = picDbBuf.getBuf(compArea);
            PelBuf    recIPredBuf   = recBuf;
            PelBuf    recIPredBufCU = picDbBuf.getBuf(cu->block(compID));
            m_bilateralFilter->bilateralFilterRDOdiamond5x5(compID, recBuf, recBuf, recBuf, currTU.cu->qp, recIPredBuf,
                                                            recIPredBufCU, cs.slice->clpRng(compID), currTU, true);
          }
        }
      }
    }

    // deblock
    if (leftEdgeAvai)
    {
      m_deblockingFilter->resetFilterLengths();
      m_deblockingFilter->deblockCu(*cu, DeblockingFilter::EdgeDir::VER);
    }

    if (topEdgeAvai)
    {
      m_deblockingFilter->resetFilterLengths();
      m_deblockingFilter->deblockCu(*cu, DeblockingFilter::EdgeDir::HOR);
    }

    // update current CU SSE
    Distortion distCur = 0;
    for (int compIdx = compStr; compIdx <= compEnd; compIdx++)
    {
      CompID  compId = (CompID)compIdx;
      CPelBuf reco   = picDbBuf.getBuf(currCsArea.block(compId));
      CPelBuf org    = cs.getOrgBuf(compId);
      distCur += getDistortionDb(cs, org, reco, compId, currCsArea.block(COMP_Y), true);
    }

    // calculate difference between DB_before_SSE and DB_after_SSE for neighbouring CUs
    Distortion distBeforeDb = 0, distAfterDb = 0;
    for (int compIdx = compStr; compIdx <= compEnd; compIdx++)
    {
      CompID compId = (CompID)compIdx;
      if (leftEdgeAvai)
      {
        const CompArea &compArea = areaLeft.block(compId);
        CPelBuf         org      = cs.picture->getOrigBuf(compArea);
        CPelBuf         reco     = cs.picture->getRecoBuf(compArea);
        CPelBuf         recoDb   = picDbBuf.getBuf(compArea);
        distBeforeDb += getDistortionDb(cs, org, reco, compId, areaLeft.block(COMP_Y), false);
        distAfterDb += getDistortionDb(cs, org, recoDb, compId, areaLeft.block(COMP_Y), true);
      }
      if (topEdgeAvai)
      {
        const CompArea &compArea = areaTop.block(compId);
        CPelBuf         org      = cs.picture->getOrigBuf(compArea);
        CPelBuf         reco     = cs.picture->getRecoBuf(compArea);
        CPelBuf         recoDb   = picDbBuf.getBuf(compArea);
        distBeforeDb += getDistortionDb(cs, org, reco, compId, areaTop.block(COMP_Y), false);
        distAfterDb += getDistortionDb(cs, org, recoDb, compId, areaTop.block(COMP_Y), true);
      }
    }

    // updated cost
    int64_t   distTmp = distCur - cs.dist + distAfterDb - distBeforeDb;
    const int sign    = sgn2(distTmp);
    distTmp           = distTmp < 0 ? -distTmp : distTmp;
    cs.costDbOffset   = sign * m_pcRdCost->calcRdCost(0, distTmp);
  }

  m_deblockingFilter->setEnc(false);
}

Distortion EncCu::getDistortionDb(CodingStructure &cs, CPelBuf org, CPelBuf reco, CompID compID,
                                  const CompArea &compArea, bool afterDb)
{
  Distortion dist = 0;
#if WCG_EXT
  m_pcRdCost->setChromaFormat(cs.sps->m_chromaFormatIdc);
  CPelBuf orgLuma = cs.picture->getOrigBuf(compArea);
  if (m_encCfg->m_lumaLevelToDeltaQPMapping.isEnabled() ||
      (m_encCfg->m_lmcsEnabled && (cs.slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag)))
  {
    if (compID == COMP_Y && !afterDb && !m_encCfg->m_lumaLevelToDeltaQPMapping.isEnabled())
    {
      CompArea tmpArea(COMP_Y, cs.area.chromaFormat, Position(0, 0), compArea.size());
      PelBuf   tmpRecLuma = m_tmpStorageCtu->getBuf(tmpArea);
      tmpRecLuma.copyFrom(reco);
      tmpRecLuma.rspSignal(m_pcReshape->m_invLUT);
      dist = m_pcRdCost->getDistPart(org, tmpRecLuma, cs.sps->m_bitDepths[toChannelType(compID)], compID,
                                     DFunc::SSE_WTD, &orgLuma);
    }
    else
    {
      dist = m_pcRdCost->getDistPart(org, reco, cs.sps->m_bitDepths[toChannelType(compID)], compID, DFunc::SSE_WTD,
                                     &orgLuma);
    }
  }
  else if (m_encCfg->m_lmcsEnabled && cs.slice->m_lmcsEnabledFlag && cs.slice->isIntra())   // intra slice
  {
    if (compID == COMP_Y && afterDb)
    {
      CompArea tmpArea(COMP_Y, cs.area.chromaFormat, Position(0, 0), compArea.size());
      PelBuf   tmpRecLuma = m_tmpStorageCtu->getBuf(tmpArea);
      tmpRecLuma.copyFrom(reco);
      tmpRecLuma.rspSignal(m_pcReshape->m_fwdLUT);
      dist = m_pcRdCost->getDistPart(org, tmpRecLuma, cs.sps->m_bitDepths[toChannelType(compID)], compID, DFunc::SSE);
    }
    else
    {
      if ((isChroma(compID) && m_encCfg->m_intraCMD))
      {
        dist = m_pcRdCost->getDistPart(org, reco, cs.sps->m_bitDepths[toChannelType(compID)], compID, DFunc::SSE_WTD,
                                       &orgLuma);
      }
      else
      {
        dist = m_pcRdCost->getDistPart(org, reco, cs.sps->m_bitDepths[toChannelType(compID)], compID, DFunc::SSE);
      }
    }
  }
  else
#endif
  {
    dist = m_pcRdCost->getDistPart(org, reco, cs.sps->m_bitDepths[toChannelType(compID)], compID, DFunc::SSE);
  }
  return dist;
}

bool EncCu::xEncodeInterResidual(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner,
                                 const EncTestMode &encTestMode, int residualPass, bool *bestHasNonResi,
                                 double *equBcwCost)
{
  CodingUnit *pcu              = tempCS->getCU(partitioner.chType);
  double      bestCostInternal = MAX_DOUBLE;
  double      bestCost         = bestCS->cost;
  double      bestCostBegin    = bestCS->cost;
  CodingUnit *prevBestCU       = bestCS->getCU(partitioner.chType);
  uint8_t     prevBestSbt      = (prevBestCU == nullptr) ? 0 : prevBestCU->sbtInfo;
  bool        swapped          = false;   // avoid unwanted data copy
  bool        bestChanged      = false;

  const CodingUnit &cu = *pcu;

  // Check whether MV and MVD are in valid range
  for (int refList = 0; refList < NUM_RPL01; refList++)
  {
    if (cu.refIdx[refList] >= 0)
    {
      if (!cu.affine)
      {
        if (!cu.mv[refList].isInRange())
        {
          return bestChanged;
        }

        Mv signaledMvd = cu.mvd[refList];
        signaledMvd.changeTransPrecInternal2Amvr(cu.imv);

        if (!signaledMvd.isInRangeDelta())
        {
          return bestChanged;
        }
      }
      else
      {
        for (int ctrlP = cu.getNumAffineMvs() - 1; ctrlP >= 0; ctrlP--)
        {
          if (!cu.mvAffi[refList][ctrlP].isInRange())
          {
            return bestChanged;
          }

          Mv signaledMvd = cu.mvdAffi[refList][ctrlP];
          signaledMvd.changeAffinePrecInternal2Amvr(cu.imv);

          if (!signaledMvd.isInRangeDelta())
          {
            return bestChanged;
          }
        }
      }
    }
  }

  const bool mtsAllowed = tempCS->sps->m_explicitMtsInter && CU::isInter(cu) &&
    partitioner.currArea().lwidth() <= tempCS->sps->m_interMTSMaxSize &&
    partitioner.currArea().lheight() <= tempCS->sps->m_interMTSMaxSize;
  uint8_t sbtAllowed = cu.checkAllowedSbt();
  // SBT resolution-dependent fast algorithm: not try size-64 SBT in RDO for low-resolution sequences (now resolution
  // below HD)
  if (tempCS->pps->m_picWidthInLumaSamples < (uint32_t)m_encCfg->m_SBTFast64WidthTh)
  {
    sbtAllowed = ((cu.lwidth() > 32 || cu.lheight() > 32)) ? 0 : sbtAllowed;
  }
  uint8_t    numRDOTried      = 0;
  Distortion sbtOffDist       = 0;
  bool       sbtOffRootCbf    = 0;
  double     sbtOffCost       = MAX_DOUBLE;
  double     currBestCost     = MAX_DOUBLE;
  bool       doPreAnalyzeResi = (sbtAllowed || mtsAllowed) && residualPass == 0;

  m_pcInterSearch->initTuAnalyzer();
  if (doPreAnalyzeResi)
  {
    m_pcInterSearch->calcMinDistSbt(*tempCS, cu, sbtAllowed);
  }

  auto slsSbt = dynamic_cast<SaveLoadEncInfoSbt *>(m_modeCtrl);
  int slShift = 4 + std::min((int)gp_sizeIdxInfo->idxFrom(cu.lwidth()) + (int)gp_sizeIdxInfo->idxFrom(cu.lheight()), 9);
  Distortion curPuSse    = m_pcInterSearch->getEstDistSbt(NUMBER_SBT_MODE);
  uint8_t    currBestSbt = 0;
  auto       currBestTrs = MtsType::NONE;
  uint8_t    histBestSbt = MAX_UCHAR;
  auto       histBestTrs = MtsType::NONE;
  m_pcInterSearch->setHistBestTrs(MAX_UCHAR, MtsType::NONE);
  if (doPreAnalyzeResi)
  {
    if (m_pcInterSearch->getSkipSbtAll() && !mtsAllowed)   // emt is off
    {
      histBestSbt = 0;   // try DCT2
      m_pcInterSearch->setHistBestTrs(histBestSbt, histBestTrs);
    }
    else
    {
      assert(curPuSse != std::numeric_limits<uint64_t>::max());
      SaveLoadEncInfoSbt::BestSbt compositeSbtTrs = slsSbt->findBestSbt(cu.cs->area, (uint32_t)(curPuSse >> slShift));

      histBestSbt = compositeSbtTrs.sbt;
      histBestTrs = compositeSbtTrs.trs;
      if (m_pcInterSearch->getSkipSbtAll() && CU::isSbtMode(histBestSbt))   // special case, skip SBT when loading SBT
      {
        histBestSbt = 0;   // try DCT2
      }
      m_pcInterSearch->setHistBestTrs(histBestSbt, histBestTrs);
    }
  }

  {
    pcu->skip    = false;
    pcu->sbtInfo = 0;

    const bool skipResidual = residualPass == 1;
    if (skipResidual || histBestSbt == MAX_UCHAR || !CU::isSbtMode(histBestSbt))
    {
      m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, skipResidual);
      numRDOTried += mtsAllowed ? 2 : 1;
      xEncodeDontSplit(*tempCS, partitioner);

      xCheckDQP(*tempCS, partitioner);
      xCheckChromaQPOffset(*tempCS, partitioner);

      if (nullptr != bestHasNonResi && (bestCostInternal > tempCS->cost))
      {
        bestCostInternal = tempCS->cost;
        if (!(tempCS->getCU(partitioner.chType)->ciipFlag))
        {
          *bestHasNonResi = !pcu->rootCbf;
        }
      }

      if (pcu->rootCbf == false)
      {
        if (tempCS->getCU(partitioner.chType)->ciipFlag)
        {
          tempCS->cost         = MAX_DOUBLE;
          tempCS->costDbOffset = 0;
          return bestChanged;
        }
      }
      currBestCost  = tempCS->cost;
      sbtOffCost    = tempCS->cost;
      sbtOffDist    = tempCS->dist;
      sbtOffRootCbf = pcu->rootCbf;
      currBestSbt   = CU::getSbtInfo(pcu->firstTU->mtsIdx[COMP_Y] > MtsType::SKIP ? SBT_OFF_MTS : SBT_OFF_DCT, 0);
      currBestTrs   = pcu->firstTU->mtsIdx[COMP_Y];
      bestChanged |= xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);
    }

    uint8_t numSbtRdo = CU::numSbtModeRdo(sbtAllowed);
    // early termination if all SBT modes are not allowed
    // normative
    if (!sbtAllowed || skipResidual)
    {
      numSbtRdo = 0;
    }
    // fast algorithm
    if ((histBestSbt != MAX_UCHAR && !CU::isSbtMode(histBestSbt)) || m_pcInterSearch->getSkipSbtAll())
    {
      numSbtRdo = 0;
    }
    if (bestCost != MAX_DOUBLE && sbtOffCost != MAX_DOUBLE)
    {
      double th = 1.07;
      if (!(prevBestSbt == 0 || m_sbtCostSave[0] == MAX_DOUBLE))
      {
        assert(m_sbtCostSave[1] <= m_sbtCostSave[0]);
        th *= (m_sbtCostSave[0] / m_sbtCostSave[1]);
      }
      if (sbtOffCost > bestCost * th)
      {
        numSbtRdo = 0;
      }
    }
    if (!sbtOffRootCbf && sbtOffCost != MAX_DOUBLE)
    {
      double th = Clip3(0.05, 0.55, (27 - pcu->qp) * 0.02 + 0.35);
      if (sbtOffCost < m_pcRdCost->calcRdCost((pcu->lwidth() * pcu->lheight()) << SCALE_BITS, 0) * th)
      {
        numSbtRdo = 0;
      }
    }

    if (histBestSbt != MAX_UCHAR && numSbtRdo != 0)
    {
      numSbtRdo = 1;
      m_pcInterSearch->initSbtRdoOrder(CU::getSbtMode(CU::getSbtIdx(histBestSbt), CU::getSbtPos(histBestSbt)));
    }

    for (int sbtModeIdx = 0; sbtModeIdx < numSbtRdo; sbtModeIdx++)
    {
      uint8_t sbtMode = m_pcInterSearch->getSbtRdoOrder(sbtModeIdx);
      uint8_t sbtIdx  = CU::getSbtIdxFromSbtMode(sbtMode);
      uint8_t sbtPos  = CU::getSbtPosFromSbtMode(sbtMode);

      // fast algorithm (early skip, save & load)
      if (histBestSbt == MAX_UCHAR)
      {
        uint8_t skipCode = m_pcInterSearch->skipSbtByRDCost(pcu->lwidth(), pcu->lheight(), pcu->mtDepth, sbtIdx, sbtPos,
                                                            bestCS->cost, sbtOffDist, sbtOffCost, sbtOffRootCbf);
        if (skipCode != MAX_UCHAR)
        {
          continue;
        }

        if (sbtModeIdx > 0)
        {
          uint8_t prevSbtMode = m_pcInterSearch->getSbtRdoOrder(sbtModeIdx - 1);
          // make sure the prevSbtMode is the same size as the current SBT mode (otherwise the estimated dist may not be
          // comparable)
          if (CU::isSameSbtSize(prevSbtMode, sbtMode))
          {
            Distortion currEstDist = m_pcInterSearch->getEstDistSbt(sbtMode);
            Distortion prevEstDist = m_pcInterSearch->getEstDistSbt(prevSbtMode);
            if (currEstDist > prevEstDist * 1.15)
            {
              continue;
            }
          }
        }
      }

      // init tempCS and TU
      if (bestCost == bestCS->cost)   // The first EMT pass didn't become the bestCS, so we clear the TUs generated
      {
        tempCS->clearTUs();
      }
      else if (false == swapped)
      {
        tempCS->initStructData(encTestMode.qp);
        tempCS->copyStructure(*bestCS, partitioner.chType);
        tempCS->getPredBuf().copyFrom(bestCS->getPredBuf());
        bestCost = bestCS->cost;
        pcu      = tempCS->getCU(partitioner.chType);
        swapped  = true;
      }
      else
      {
        tempCS->clearTUs();
        bestCost = bestCS->cost;
        pcu      = tempCS->getCU(partitioner.chType);
      }

      // we need to restart the distortion for the new tempCS, the bit count and the cost
      tempCS->dist     = 0;
      tempCS->fracBits = 0;
      tempCS->cost     = MAX_DOUBLE;
      pcu->skip        = false;

      // set SBT info
      pcu->setSbtIdx(sbtIdx);
      pcu->setSbtPos(sbtPos);

      // try residual coding
      m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, skipResidual);
      numRDOTried++;

      xEncodeDontSplit(*tempCS, partitioner);
      xCheckDQP(*tempCS, partitioner);
      xCheckChromaQPOffset(*tempCS, partitioner);

      if (nullptr != bestHasNonResi && (bestCostInternal > tempCS->cost))
      {
        bestCostInternal = tempCS->cost;
        if (!(tempCS->getCU(partitioner.chType)->ciipFlag))
        {
          *bestHasNonResi = !pcu->rootCbf;
        }
      }

      if (tempCS->cost < currBestCost)
      {
        currBestSbt = pcu->sbtInfo;
        currBestTrs = tempCS->tus[pcu->getSbtTuIdx()]->mtsIdx[COMP_Y];
        assert(!isMTS(currBestTrs));
        currBestCost = tempCS->cost;
      }

      bestChanged |= xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);
    }

    if (bestCostBegin != bestCS->cost)
    {
      m_sbtCostSave[0] = sbtOffCost;
      m_sbtCostSave[1] = currBestCost;
    }
  }   // end emt loop

  if (histBestSbt == MAX_UCHAR && doPreAnalyzeResi && numRDOTried > 1)
  {
    slsSbt->saveBestSbt(pcu->cs->area, (uint32_t)(curPuSse >> slShift), currBestSbt, currBestTrs);
  }
  tempCS->cost = currBestCost;
  if (ETM_INTER_ME == encTestMode.type)
  {
    if (equBcwCost != nullptr)
    {
      if (tempCS->cost < (*equBcwCost) && pcu->bcwIdx == BCW_DEFAULT)
      {
        (*equBcwCost) = tempCS->cost;
      }
    }
    else
    {
      CHECK(equBcwCost == nullptr, "equBcwCost == nullptr");
    }
    if (tempCS->slice->m_checkLdc && (!tempCS->sps->m_biLicEnabledFlag || !pcu->licFlag) && !pcu->imv &&
        pcu->bcwIdx != BCW_DEFAULT && tempCS->cost < m_bestBcwCost[1])
    {
      if (tempCS->cost < m_bestBcwCost[0])
      {
        m_bestBcwCost[1] = m_bestBcwCost[0];
        m_bestBcwCost[0] = tempCS->cost;
        m_bestBcwIdx[1]  = m_bestBcwIdx[0];
        m_bestBcwIdx[0]  = pcu->bcwIdx;
      }
      else
      {
        m_bestBcwCost[1] = tempCS->cost;
        m_bestBcwIdx[1]  = pcu->bcwIdx;
      }
    }
    m_pcInterSearch->m_fastLicCtrl.setBestAmvpRDBeforeLIC(*pcu, currBestCost);
  }

  return bestChanged;
}

void EncCu::xEncodeDontSplit(CodingStructure &cs, Partitioner &partitioner)
{
  m_CABACEstimator->resetBits();

  m_CABACEstimator->split_cu_mode(CU_DONT_SPLIT, cs, partitioner);

  cs.fracBits += m_CABACEstimator->getEstFracBits();   // split bits
  cs.cost = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);
}

#if REUSE_CU_RESULTS
void EncCu::xReuseCachedResult(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner)
{
  m_pcRdCost->setChromaFormat(tempCS->sps->m_chromaFormatIdc);
  BestEncInfoCache *bestEncCache = dynamic_cast<BestEncInfoCache *>(m_modeCtrl);
  CHECK(!bestEncCache, "If this mode is chosen, mode controller has to implement the mode caching capabilities");
  EncTestMode cachedMode;

  if (bestEncCache->setCsFrom(*tempCS, cachedMode, partitioner))
  {
    CodingUnit &cu = *tempCS->cus.front();
    partitioner.setCUData(cu);

    if (CU::isIntra(cu) || CU::isPLT(cu))
    {
      xReconIntraQT(cu);
    }
    else
    {
      PU::setGpmDirMode(cu);
      xDeriveCuMvs(cu);
      xReconInter(cu);
    }

    m_CABACEstimator->getCtx() = m_CurrCtx->start;
    m_CABACEstimator->resetBits();

    CUCtx cuCtx;
    cuCtx.isDQPCoded         = true;
    cuCtx.isChromaQpAdjCoded = true;
    m_CABACEstimator->coding_unit(cu, partitioner, cuCtx);

    tempCS->fracBits = m_CABACEstimator->getEstFracBits();
    tempCS->cost     = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);

    xEncodeDontSplit(*tempCS, partitioner);
    xCheckDQP(*tempCS, partitioner);
    xCheckChromaQPOffset(*tempCS, partitioner);
    xCheckBestMode(tempCS, bestCS, partitioner, cachedMode, m_encCfg->m_encDbOpt);
  }
  else
  {
    THROW("Should never happen!");
  }
}
#endif

MergeItem::MergeItem() {}
MergeItem::~MergeItem() {}

void MergeItem::create(ChromaFormat chromaFormat, const Area &area)
{
  if (m_pelStorage.bufs.empty())
  {
    m_pelStorage.create(chromaFormat, area);
    m_mvStorage.resize(g_miScaling.scaleArea(area.area()));
  }

  // reset data
  cost                       = MAX_DOUBLE;
  mergeIdx                   = 0;
  bcwIdx                     = 0;
  interDir                   = 0;
  useAltHpelIf               = false;
  affineType                 = AffineModel::_4_PARAMS;
  ciipMode                   = CIIP_Type::NORMAL;
  mergeIdxAlreadyCheckByCIIP = false;
  mergeItemType              = MergeItemType::NUM;

  noBdofRefine = false;
  noResidual   = false;

  lumaPredReady   = false;
  chromaPredReady = false;
  obmcDone        = false;
  bdmvrRefine     = false;

  bmMergeFlag = false;
  bmDir       = 0;

  licFlag = false;
  licScaleAndOffset.reset();
}
void MergeItem::importMergeInfo(const GeoMergeCtx &mergeCtx, int _mergeIdx, MergeItemType _mergeItemType,
                                CodingUnit &cu)
{
  mergeIdx        = _mergeIdx;
  mergeItemType   = _mergeItemType;
  oppositeLicFlag = false;

  switch (_mergeItemType)
  {
  case MergeItemType::GPM:
    mvField[0][RPL0].setMvField(Mv(0, 0), -1);
    mvField[0][RPL1].setMvField(Mv(0, 0), -1);
    bcwIdx       = BCW_DEFAULT;
    useAltHpelIf = false;
    MergeItem::updateGpmIdx(mergeIdx, cu.geoSplitDir, cu.geoBldIdx, cu.geoMergeIdx);
    cu.gpmIntraFlag = false;
    licFlag         = false;
    for (auto part: { 0, 1 })
    {
      int tmpMrgIdx = mergeIdxFrom(cu.geoMergeIdx[part]);
      if (tmpMrgIdx == -1)
      {
        cu.geoMergeIdx[part] = cu.geoMergeIdx[part] + GEO_MAX_NUM_UNI_CANDS;
        cu.geoMMVDIdx[part]  = -1;
        cu.geoMMVDFlag[part] = false;
        cu.gpmIntraFlag      = true;
      }
      else
      {
        cu.geoMMVDIdx[part]  = mmvdIdxFrom(cu.geoMergeIdx[part]);
        cu.geoMMVDFlag[part] = mmvdFlagFrom(cu.geoMergeIdx[part]);
        cu.geoMergeIdx[part] = tmpMrgIdx;
      }
    }
    PU::setGpmDirMode(cu);
    cu.mergeFlag = true;
    cu.affine    = false;
    cu.geoFlag   = true;
    cu.mergeType = MergeType::DEFAULT_N;

    PU::spanMotionInfo(cu);   // TODO: even needed?

    PU::spanGeoMMVDMotionInfo(cu, mergeCtx);
    getMvBuf(cu).copyFrom(cu.getMotionBuf());
    break;

  default:
    THROW("Wrong merge item type");
  }
}

void MergeItem::importMergeInfo(const MergeCtx &mergeCtx, int _mergeIdx, MergeItemType _mergeItemType, CodingUnit &cu)
{
  mergeIdx        = _mergeIdx;
  mergeItemType   = _mergeItemType;
  oppositeLicFlag = false;

  switch (_mergeItemType)
  {
  case MergeItemType::REGULAR:
  case MergeItemType::CIIP:
    CHECK(mergeIdx >= MRG_MAX_NUM_CANDS, "Too large merge index");
    mvField[0][RPL0] = mergeCtx.mvFieldNeighbours[mergeIdx][RPL0];
    mvField[0][RPL1] = mergeCtx.mvFieldNeighbours[mergeIdx][RPL1];
    interDir         = mergeCtx.interDirNeighbours[mergeIdx];
    bcwIdx           = mergeCtx.bcwIdx[mergeIdx];
    useAltHpelIf     = mergeCtx.useAltHpelIf[mergeIdx];
    bmMergeFlag      = cu.bmMergeFlag;
    bmDir            = cu.bmDir;
    ciipMode         = cu.ciipMode;
    licFlag          = mergeCtx.LICFlags[mergeIdx] ^ cu.oppositeLicFlag;
    oppositeLicFlag  = cu.oppositeLicFlag;
    break;
  case MergeItemType::MMVD:
    {
      MmvdIdx candIdx;
      candIdx.val           = mergeIdx;
      const int mmvdBaseIdx = candIdx.pos.baseIdx;
      mvField[0][RPL0]      = mergeCtx.mmvdBaseMv[mmvdBaseIdx][RPL0];
      mvField[0][RPL1]      = mergeCtx.mmvdBaseMv[mmvdBaseIdx][RPL1];
      Mv tempMv[NUM_RPL01];
      mergeCtx.getMmvdDeltaMv(*cu.cs->slice, candIdx, tempMv);
      mvField[0][RPL0].mv += tempMv[RPL0];
      mvField[0][RPL1].mv += tempMv[RPL1];
      interDir     = mergeCtx.interDirNeighbours[mmvdBaseIdx];
      bcwIdx       = mergeCtx.bcwIdx[mmvdBaseIdx];
      useAltHpelIf = mergeCtx.useAltHpelIf[mmvdBaseIdx];
      bmMergeFlag  = false;
      bmDir        = 0;
      licFlag      = mergeCtx.LICFlags[mmvdBaseIdx];
      break;
    }
  case MergeItemType::IBC:
  default:
    THROW("Wrong merge item type");
  }
}

void MergeItem::importMergeInfo(const AffineMergeCtx &mergeCtx, int _mergeIdx, MergeItemType _mergeItemType,
                                CodingUnit &cu)
{
  mergeIdx        = _mergeIdx;
  mergeItemType   = _mergeItemType;
  useAltHpelIf    = false;
  oppositeLicFlag = false;

  switch (_mergeItemType)
  {
  case MergeItemType::SBTMVP:
    affineType = mergeCtx.affineType[mergeIdx];
    interDir   = mergeCtx.interDirNeighbours[mergeIdx];
    bcwIdx     = mergeCtx.bcwIdx[mergeIdx];
    mvField    = mergeCtx.mvFieldNeighbours[mergeIdx];
    licFlag    = false;
    getMvBuf(cu).copyFrom(mergeCtx.subPuMvpMiBuf);
    break;

  case MergeItemType::AFFINE:
    affineType      = mergeCtx.affineType[mergeIdx];
    interDir        = mergeCtx.interDirNeighbours[mergeIdx];
    bcwIdx          = mergeCtx.bcwIdx[mergeIdx];
    mvField         = mergeCtx.mvFieldNeighbours[mergeIdx];
    licFlag         = mergeCtx.LICFlags[mergeIdx] ^ cu.oppositeLicFlag;
    oppositeLicFlag = cu.oppositeLicFlag;
    break;

  case MergeItemType::AFF_MMVD:
    {
      MmvdIdx mmvdIdx;
      mmvdIdx.val    = mergeIdx;
      int baseIdxOff = (int)PU::getMergeIdxFromAffMmvdBaseIdx(mergeCtx, mmvdIdx.pos.baseIdx);
      affineType     = mergeCtx.affineType[mmvdIdx.pos.baseIdx + baseIdxOff];
      interDir       = mergeCtx.interDirNeighbours[mmvdIdx.pos.baseIdx + baseIdxOff];
      bcwIdx         = mergeCtx.bcwIdx[mmvdIdx.pos.baseIdx + baseIdxOff];
      licFlag        = mergeCtx.LICFlags[mmvdIdx.pos.baseIdx + baseIdxOff];
      PU::getAffMmvdMvf(cu, mergeCtx, mvField, mmvdIdx.pos.baseIdx + baseIdxOff, mmvdIdx.pos.step,
                        mmvdIdx.pos.position);
      break;
    }

  default:
    THROW("Wrong merge item type");
  }
}

bool MergeItem::exportMergeInfo(CodingUnit &cu, bool forceNoResidual)
{
  const bool    resetCiip2Regular = mergeItemType == MergeItemType::CIIP && forceNoResidual;
  MergeItemType updatedType       = resetCiip2Regular ? MergeItemType::REGULAR : mergeItemType;

  cu.mergeFlag        = true;
  cu.regularMergeFlag = false;
  cu.mmvdMergeFlag    = false;
  cu.interDir         = interDir;
  cu.mergeIdx         = mergeIdx;
  cu.mergeType        = MergeType::DEFAULT_N;
  cu.mv[RPL0]         = mvField[0][RPL0].mv;
  cu.mv[RPL1]         = mvField[0][RPL1].mv;
  cu.refIdx[RPL0]     = mvField[0][RPL0].refIdx;
  cu.refIdx[RPL1]     = mvField[0][RPL1].refIdx;
  cu.mvd[RPL0]        = Mv();
  cu.mvd[RPL1]        = Mv();
  cu.mvpIdx[RPL0]     = NOT_VALID;
  cu.mvpIdx[RPL1]     = NOT_VALID;
  cu.mvpNum[RPL0]     = NOT_VALID;
  cu.mvpNum[RPL1]     = NOT_VALID;
  cu.bcwIdx           = (interDir == 3) ? bcwIdx : BCW_DEFAULT;
  cu.mmvdEncOptMode   = 0;
  cu.mmvdSkip         = false;
  cu.affine           = false;
  cu.affineType       = AffineModel::_4_PARAMS;
  cu.geoFlag          = false;
  cu.ciipFlag         = false;
  cu.ciipMode         = CIIP_Type::NORMAL;
  cu.imv              = (!cu.geoFlag && useAltHpelIf) ? IMV_HPEL : 0;
  cu.gpmIntraFlag     = false;
  cu.bdmvrRefine      = bdmvrRefine && updatedType == MergeItemType::REGULAR;
  cu.bmMergeFlag      = bmMergeFlag;
  cu.bmDir            = bmDir;

  cu.licFlag         = licFlag;
  cu.oppositeLicFlag = oppositeLicFlag;
  cu.licScaleAndOffset.copyFrom(licScaleAndOffset, true, true);

  switch (updatedType)
  {
  case MergeItemType::REGULAR:
    cu.regularMergeFlag = true;
    PU::restrictBiPredMergeCandsOne(cu);
    break;

  case MergeItemType::CIIP:
    CHECK(forceNoResidual, "Cannot force no residuals for CIIP");
    cu.ciipFlag                      = true;
    cu.ciipMode                      = ciipMode;
    cu.intraDir[ChannelType::LUMA]   = PLANAR_IDX;
    cu.intraDir[ChannelType::CHROMA] = DM_CHROMA_IDX;
    PU::restrictBiPredMergeCandsOne(cu);
    break;

  case MergeItemType::MMVD:
    cu.mmvdMergeFlag    = true;
    cu.mmvdMergeIdx.val = mergeIdx;
    cu.regularMergeFlag = true;
    if (forceNoResidual)
    {
      cu.mmvdSkip = true;
    }
    PU::restrictBiPredMergeCandsOne(cu);
    break;

  case MergeItemType::SBTMVP:
    CHECK(licFlag, "unexpectedly set LIC flag for SBTMVP");
    cu.affine    = true;
    cu.mergeType = MergeType::SUBPU_ATMVP;
    break;

  case MergeItemType::AFF_MMVD:
    cu.mmvdMergeFlag    = true;
    cu.mmvdMergeIdx.val = mergeIdx;
  case MergeItemType::AFFINE:
    cu.affine     = true;
    cu.affineType = affineType;
    PU::setAllAffineMvField(cu, mvField, RPL0);
    PU::setAllAffineMvField(cu, mvField, RPL1);
    break;

  case MergeItemType::GPM:
    CHECK(licFlag, "unexpectedly set LIC flag for GPM");
    cu.mergeIdx = -1;
    cu.geoFlag  = true;
    cu.bcwIdx   = BCW_DEFAULT;
    MergeItem::updateGpmIdx(mergeIdx, cu.geoSplitDir, cu.geoBldIdx, cu.geoMergeIdx);

    for (auto part: { 0, 1 })
    {
      int tmpMrgIdx = mergeIdxFrom(cu.geoMergeIdx[part]);
      if (tmpMrgIdx == -1)
      {
        cu.geoMergeIdx[part] = cu.geoMergeIdx[part] + GEO_MAX_NUM_UNI_CANDS;
        cu.geoMMVDIdx[part]  = -1;
        cu.geoMMVDFlag[part] = false;
        cu.gpmIntraFlag      = true;
      }
      else
      {
        cu.geoMMVDIdx[part]  = mmvdIdxFrom(cu.geoMergeIdx[part]);
        cu.geoMMVDFlag[part] = mmvdFlagFrom(cu.geoMergeIdx[part]);
        cu.geoMergeIdx[part] = tmpMrgIdx;
      }
    }
    PU::setGpmDirMode(cu);
    cu.imv = 0;
    break;

  case MergeItemType::IBC:
  default:
    THROW("Wrong merge item type");
  }

  if (mergeItemType == MergeItemType::GPM || mergeItemType == MergeItemType::SBTMVP ||
      (updatedType == MergeItemType::REGULAR && bdmvrRefine))
  {
    cu.getMotionBuf().copyFrom(getMvBuf(cu));
    // this is necessary to avoid incorrect merge candidates and mismatches
    if (mergeItemType == MergeItemType::GPM)
    {
      MotionBuf mb = cu.getMotionBuf();
      for (int y = 0; y < mb.height; y++)
      {
        for (int x = 0; x < mb.width; x++)
        {
          MotionInfo &mi  = mb.at(x, y);
          mi.useAltHpelIf = false;
        }
      }
    }
  }
  else
  {
    PU::spanMotionInfo(cu);
  }

  cu.obmcFlag = CU::isObmcAllowed(cu);

  return resetCiip2Regular;
}

MergeItemList::MergeItemList() {}

MergeItemList::~MergeItemList()
{
  for (auto p: m_list)
  {
    m_mergeItemPool.giveBack(p);
  }
  m_list.clear();
}

void MergeItemList::init(size_t maxSize, ChromaFormat chromaFormat, int ctuWidth, int ctuHeight)
{
  m_list.reserve(maxSize + 1);   // to avoid reallocation when inserting a new item
  m_chromaFormat   = chromaFormat;
  m_ctuArea.x      = 0;
  m_ctuArea.y      = 0;
  m_ctuArea.width  = ctuWidth;
  m_ctuArea.height = ctuHeight;
}

MergeItem *MergeItemList::allocateNewMergeItem()
{
  MergeItem *p = m_mergeItemPool.get();
  p->create(m_chromaFormat, m_ctuArea);

  return p;
}

void MergeItemList::insertMergeItemToList(MergeItem *p, const size_t startPos)
{
  if (m_list.empty())
  {
    m_list.push_back(p);
  }
  else if (m_list.size() == m_maxTrackingNum && p->cost >= m_list.back()->cost)
  {
    m_mergeItemPool.giveBack(p);
  }
  else
  {
    if (m_list.size() == m_maxTrackingNum)
    {
      m_mergeItemPool.giveBack(m_list.back());
      m_list.pop_back();
    }

    auto it = m_list.begin();
    for (size_t i = 0; it != m_list.end(); it++, i++)
    {
      if (p->cost < (*it)->cost && i >= startPos)
      {
        break;
      }
    }
    m_list.insert(it, p);
  }
}

MergeItem *MergeItemList::getMergeItemInList(size_t index)
{
  return index < m_maxTrackingNum && index < m_list.size() ? m_list[index] : nullptr;
}

void MergeItemList::resetList(size_t maxTrackingNum)
{
  for (auto p: m_list)
  {
    m_mergeItemPool.giveBack(p);
  }
  m_list.clear();
  m_list.reserve(maxTrackingNum);
  m_maxTrackingNum = maxTrackingNum;
}

void MergeItemList::shrink(size_t maxTrackingNum)
{
  while (m_list.size() > maxTrackingNum)
  {
    m_mergeItemPool.giveBack(m_list.back());
    m_list.pop_back();
  }
}

void EncCu::xCheckNonSplitModes(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm,
                                double &maxCostAllowed)
{
  ComprCUCtx     &comprCUCtx = *m_modeCtrl->comprCUCtx;
  const UnitArea &area       = tempCS->area;
  const SPS      &sps        = *tempCS->sps;
  const Slice    &slice      = *tempCS->slice;

  int    minQP            = comprCUCtx.baseQp;
  int    maxQP            = comprCUCtx.baseQp;
  double deltaQPForLambda = 0.0;
  m_modeCtrl->getMinMaxQP(minQP, maxQP, deltaQPForLambda, *tempCS, pm, comprCUCtx.baseQp, CU_DONT_SPLIT);
  const bool useLIC = slice.m_useLic;

  bool canNo = pm.canSplit(CU_DONT_SPLIT, *tempCS);

  if (!canNo)
  {
    EncTestMode encTestMode({ ETM_INVALID, comprCUCtx.baseQp, deltaQPForLambda });
    prepare(tempCS, pm, encTestMode);
    return;
  }

  // add first pass modes
  if (!slice.isIntra())
  {
    for (int qp = minQP; qp <= maxQP; qp++)
    {
      m_etmObmcBest = EncTestMode({ ETM_INVALID, ETO_INVALID, MAX_QP, 0, maxCostAllowed });
      {
        EncTestMode encTestMode({ ETM_HASH_INTER, ETO_STANDARD, qp, deltaQPForLambda, maxCostAllowed });
        if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm))
        {
          prepare(tempCS, pm, encTestMode);
          xCheckRDCostHashInter(tempCS, bestCS, pm, encTestMode);
        }
      }

      // add inter modes
      if (m_encCfg->m_useEarlySkipDetection)
      {
        {
          EncTestMode encTestMode({ ETM_INTER_ME, ETO_STANDARD, qp, deltaQPForLambda, maxCostAllowed });
          if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm))
          {
            prepare(tempCS, pm, encTestMode);
            xCheckRDCostInter(tempCS, bestCS, pm, encTestMode);
          }
        }

        {
          EncTestMode encTestMode({ ETM_MERGE_SKIP, ETO_STANDARD, qp, deltaQPForLambda, maxCostAllowed });
          if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm))
          {
            prepare(tempCS, pm, encTestMode);
            xCheckRDCostUnifiedMerge(tempCS, bestCS, pm, encTestMode);
          }
        }
      }
      else
      {
        {
          EncTestMode encTestMode({ ETM_MERGE_SKIP, ETO_STANDARD, qp, deltaQPForLambda, maxCostAllowed });
          if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm))
          {
            prepare(tempCS, pm, encTestMode);
            xCheckRDCostUnifiedMerge(tempCS, bestCS, pm, encTestMode);
          }
        }

        {
          EncTestMode encTestMode({ ETM_INTER_ME, ETO_STANDARD, qp, deltaQPForLambda, maxCostAllowed });
          if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm))
          {
            prepare(tempCS, pm, encTestMode);
            xCheckRDCostInter(tempCS, bestCS, pm, encTestMode);
          }
        }
      }

      // inter with illumination compensation
      if (useLIC)
      {
        EncTestMode encTestMode({ ETM_INTER_ME, ETO_LIC, qp, deltaQPForLambda, maxCostAllowed });
        if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm))
        {
          prepare(tempCS, pm, encTestMode);
          xCheckRDCostInter(tempCS, bestCS, pm, encTestMode);
        }
      }

      double amvrBestIntPelCost = MAX_DOUBLE;

      if (m_encCfg->m_ImvMode || m_encCfg->m_AffineAmvr)
      {
        {
          EncTestMode encTestMode({ ETM_INTER_ME,
                                    EncTestModeOpts(int(EncTestMode::AmvrSearchMode::FULL_PEL) << ETO_IMV_SHIFT), qp,
                                    deltaQPForLambda, maxCostAllowed });
          if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm))
          {
            prepare(tempCS, pm, encTestMode);
            xCheckRDCostInterAmvr(tempCS, bestCS, pm, encTestMode, amvrBestIntPelCost);
          }
        }

        // inter with imv and illumination compensation
        if (useLIC && m_encCfg->m_fastLICMode < 2)
        {
          EncTestMode encTestMode(
            { ETM_INTER_ME, EncTestModeOpts((int(EncTestMode::AmvrSearchMode::FULL_PEL) << ETO_IMV_SHIFT) | ETO_LIC),
              qp, deltaQPForLambda, maxCostAllowed });
          if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm))
          {
            prepare(tempCS, pm, encTestMode);
            xCheckRDCostInterAmvr(tempCS, bestCS, pm, encTestMode, amvrBestIntPelCost);
          }
        }

        {
          const auto  imv = m_encCfg->m_Imv4PelFast ? EncTestMode::AmvrSearchMode::FOUR_PEL_FAST
                                                    : EncTestMode::AmvrSearchMode::FOUR_PEL;
          EncTestMode encTestMode(
            { ETM_INTER_ME, EncTestModeOpts(int(imv) << ETO_IMV_SHIFT), qp, deltaQPForLambda, maxCostAllowed });
          if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm))
          {
            prepare(tempCS, pm, encTestMode);
            xCheckRDCostInterAmvr(tempCS, bestCS, pm, encTestMode, amvrBestIntPelCost);
          }

          // inter with imv and illumination compensation
          if (useLIC && m_encCfg->m_fastLICMode < 2)
          {
            EncTestMode encTestMode({ ETM_INTER_ME, EncTestModeOpts((int(imv) << ETO_IMV_SHIFT) | ETO_LIC), qp,
                                      deltaQPForLambda, maxCostAllowed });
            if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm))
            {
              prepare(tempCS, pm, encTestMode);
              xCheckRDCostInterAmvr(tempCS, bestCS, pm, encTestMode, amvrBestIntPelCost);
            }
          }
        }
      }
      if (m_encCfg->m_ImvMode)
      {
        EncTestMode encTestMode({ ETM_INTER_ME,
                                  EncTestModeOpts(int(EncTestMode::AmvrSearchMode::HALF_PEL) << ETO_IMV_SHIFT), qp,
                                  deltaQPForLambda, maxCostAllowed });
        const bool  skipAltHpelIF = (encTestMode.getAmvrSearchMode() == EncTestMode::AmvrSearchMode::HALF_PEL) &&
          (amvrBestIntPelCost > 1.25 * bestCS->cost);
        if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm) && !skipAltHpelIF)
        {
          prepare(tempCS, pm, encTestMode);
          xCheckRDCostInterAmvr(tempCS, bestCS, pm, encTestMode, amvrBestIntPelCost);
        }

        // inter with imv and illumination compensation
        if (useLIC && m_encCfg->m_fastLICMode < 2)
        {
          EncTestMode encTestMode(
            { ETM_INTER_ME, EncTestModeOpts((int(EncTestMode::AmvrSearchMode::HALF_PEL) << ETO_IMV_SHIFT) | ETO_LIC),
              qp, deltaQPForLambda, maxCostAllowed });
          const bool skipAltHpelIF = (encTestMode.getAmvrSearchMode() == EncTestMode::AmvrSearchMode::HALF_PEL) &&
            (amvrBestIntPelCost > 1.25 * bestCS->cost);
          if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm) && !skipAltHpelIF)
          {
            prepare(tempCS, pm, encTestMode);
            xCheckRDCostInterAmvr(tempCS, bestCS, pm, encTestMode, amvrBestIntPelCost);
          }
        }
      }
      if (useLIC && m_encCfg->m_fastLICMode == 2)
      {
        if (m_encCfg->m_ImvMode || m_encCfg->m_AffineAmvr)
        {
          EncTestMode encTestMode(
            { ETM_INTER_ME, EncTestModeOpts((int(EncTestMode::AmvrSearchMode::FULL_PEL) << ETO_IMV_SHIFT) | ETO_LIC),
              qp, deltaQPForLambda, maxCostAllowed });
          if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm))
          {
            prepare(tempCS, pm, encTestMode);
            xCheckRDCostInterAmvr(tempCS, bestCS, pm, encTestMode, amvrBestIntPelCost);
          }
          {
            const auto  imv = m_encCfg->m_Imv4PelFast ? EncTestMode::AmvrSearchMode::FOUR_PEL_FAST
                                                      : EncTestMode::AmvrSearchMode::FOUR_PEL;
            EncTestMode encTestMode({ ETM_INTER_ME, EncTestModeOpts((int(imv) << ETO_IMV_SHIFT) | ETO_LIC), qp,
                                      deltaQPForLambda, maxCostAllowed });
            if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm))
            {
              prepare(tempCS, pm, encTestMode);
              xCheckRDCostInterAmvr(tempCS, bestCS, pm, encTestMode, amvrBestIntPelCost);
            }
          }

          if (m_encCfg->m_ImvMode)
          {
            EncTestMode encTestMode(
              { ETM_INTER_ME, EncTestModeOpts((int(EncTestMode::AmvrSearchMode::HALF_PEL) << ETO_IMV_SHIFT) | ETO_LIC),
                qp, deltaQPForLambda, maxCostAllowed });
            const bool skipAltHpelIF = (encTestMode.getAmvrSearchMode() == EncTestMode::AmvrSearchMode::HALF_PEL) &&
              (amvrBestIntPelCost > 1.25 * bestCS->cost);
            if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm) && !skipAltHpelIF)
            {
              prepare(tempCS, pm, encTestMode);
              xCheckRDCostInterAmvr(tempCS, bestCS, pm, encTestMode, amvrBestIntPelCost);
            }
          }
        }
      }

      if (m_etmObmcBest.type != ETM_INVALID)
      {
        prepare(tempCS, pm, m_etmObmcBest);
        xCheckRDCostInterWoObmc(tempCS, bestCS, pm);
      }
    }
  }

  // until here without useDbCost!
  if (bestCS->cost != MAX_DOUBLE)
  {
    xCalDebCost(*bestCS, pm);
  }
  // from here on tests are done with encDbCost!

  for (int qp = minQP; qp <= maxQP; qp++)
  {
    // add ibc mode to intra path
    if (slice.m_ibcFlag && (pm.chType != ChannelType::CHROMA) && !(m_encCfg->m_ibcFastMethod & IBC_FAST_METHOD_NONSCC))
    {
      if (m_encCfg->m_ibcMerge)
      {
        EncTestMode encTestMode({ ETM_IBC_MERGE, ETO_STANDARD, qp, deltaQPForLambda, maxCostAllowed });
        if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm))
        {
          prepare(tempCS, pm, encTestMode);
          xCheckRDCostIBCModeMerge2Nx2N(tempCS, bestCS, pm, encTestMode);
        }
      }

      {
        EncTestMode encTestMode({ ETM_IBC, ETO_STANDARD, qp, deltaQPForLambda, maxCostAllowed });
        if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm))
        {
          prepare(tempCS, pm, encTestMode);
          xCheckRDCostIBCMode(tempCS, bestCS, pm, encTestMode);
        }
      }
    }

    // add intra modes
    const bool pltIsLast = slice.isIntra() || (area.lwidth() == 4 && area.lheight() == 4);
    if (sps.m_PLTMode && !pltIsLast)
    {
      EncTestMode encTestMode({ ETM_PALETTE, ETO_STANDARD, qp, deltaQPForLambda, maxCostAllowed });
      if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm))
      {
        prepare(tempCS, pm, encTestMode);
        xCheckPLT(tempCS, bestCS, pm, encTestMode);
      }
    }

    {
      EncTestMode encTestMode({ ETM_INTRA, ETO_STANDARD, qp, deltaQPForLambda, maxCostAllowed });
      if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm))
      {
        prepare(tempCS, pm, encTestMode);
        xCheckRDCostIntra(tempCS, bestCS, pm, encTestMode);
      }
    }

    if (sps.m_PLTMode && pltIsLast)
    {
      EncTestMode encTestMode({ ETM_PALETTE, ETO_STANDARD, qp, deltaQPForLambda, maxCostAllowed });
      if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm))
      {
        prepare(tempCS, pm, encTestMode);
        xCheckPLT(tempCS, bestCS, pm, encTestMode);
      }
    }

    if (slice.m_ibcFlag && (pm.chType != ChannelType::CHROMA) && (m_encCfg->m_ibcFastMethod & IBC_FAST_METHOD_NONSCC))
    {
      if (m_encCfg->m_ibcMerge)
      {
        EncTestMode encTestMode({ ETM_IBC_MERGE, ETO_STANDARD, qp, deltaQPForLambda, maxCostAllowed });
        if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm))
        {
          prepare(tempCS, pm, encTestMode);
          xCheckRDCostIBCModeMerge2Nx2N(tempCS, bestCS, pm, encTestMode);
        }
      }

      {
        EncTestMode encTestMode({ ETM_IBC, ETO_STANDARD, qp, deltaQPForLambda, maxCostAllowed });
        if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm))
        {
          prepare(tempCS, pm, encTestMode);
          xCheckRDCostIBCMode(tempCS, bestCS, pm, encTestMode);
        }
      }
    }

#if REUSE_CU_RESULTS
    {
      EncTestMode encTestMode({ ETM_RECO_CACHED, ETO_STANDARD, qp, deltaQPForLambda, maxCostAllowed });
      if (m_modeCtrl->tryMode(encTestMode, *tempCS, pm))
      {
        prepare(tempCS, pm, encTestMode);
        xReuseCachedResult(tempCS, bestCS, pm);
      }
    }
#endif
  }
  maxCostAllowed = std::min(maxCostAllowed, bestCS->cost);
  if (m_modeCtrl->finishNonSplitModes(*bestCS, pm))
  {
    m_pcInterSearch->insertUniMvCandsReuseMv(pm.currArea().Y(), *slice.m_pps->pcv);
  }
}

void EncCu::xCheckSplitModes(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm,
                             double &maxCostAllowed)
{
  ComprCUCtx &comprCUCtx = *m_modeCtrl->comprCUCtx;

  int    minQP            = comprCUCtx.baseQp;
  int    maxQP            = comprCUCtx.baseQp;
  double deltaQPForLambda = 0.0;
  m_modeCtrl->getMinMaxQP(minQP, maxQP, deltaQPForLambda, *tempCS, pm, comprCUCtx.baseQp, CU_QUAD_SPLIT);

  int    minQPq            = minQP;
  int    maxQPq            = maxQP;
  double deltaQPForLambdaq = 0.0;
  m_modeCtrl->getMinMaxQP(minQP, maxQP, deltaQPForLambdaq, *tempCS, pm, comprCUCtx.baseQp, CU_BT_SPLIT);

  bool canNo, canQt, canBh, canBv, canTh, canTv;

  pm.canSplit(*tempCS, canNo, canQt, canBh, canBv, canTh, canTv);

  if (comprCUCtx.qtBeforeBt && canQt)
  {
    EncTestMode encTestMode = { ETM_SPLIT_QT, ETO_STANDARD, -1, deltaQPForLambdaq, maxCostAllowed };
    if (m_modeCtrl->trySplit(encTestMode, *tempCS, pm))
    {
      for (encTestMode.qp = minQPq; encTestMode.qp <= maxQPq; encTestMode.qp++)
      {
        xCheckModeSplit(tempCS, bestCS, pm, encTestMode);
        maxCostAllowed = std::min(maxCostAllowed, bestCS->cost);
      }
    }
  }

  if (canBh)
  {
    EncTestMode encTestMode = { ETM_SPLIT_BT_H, ETO_STANDARD, -1, deltaQPForLambda, maxCostAllowed };
    if (m_modeCtrl->trySplit(encTestMode, *tempCS, pm))
    {
      for (encTestMode.qp = minQP; encTestMode.qp <= maxQP; encTestMode.qp++)
      {
        xCheckModeSplit(tempCS, bestCS, pm, encTestMode);
        maxCostAllowed = std::min(maxCostAllowed, bestCS->cost);
      }
    }
  }

  if (canBv)
  {
    EncTestMode encTestMode = { ETM_SPLIT_BT_V, ETO_STANDARD, -1, deltaQPForLambda, maxCostAllowed };
    if (m_modeCtrl->trySplit(encTestMode, *tempCS, pm))
    {
      for (encTestMode.qp = minQP; encTestMode.qp <= maxQP; encTestMode.qp++)
      {
        xCheckModeSplit(tempCS, bestCS, pm, encTestMode);
        maxCostAllowed = std::min(maxCostAllowed, bestCS->cost);
      }
    }
  }

  if (canTh)
  {
    EncTestMode encTestMode = { ETM_SPLIT_TT_H, ETO_STANDARD, -1, deltaQPForLambdaq, maxCostAllowed };
    if (m_modeCtrl->trySplit(encTestMode, *tempCS, pm))
    {
      for (encTestMode.qp = minQPq; encTestMode.qp <= maxQPq; encTestMode.qp++)
      {
        xCheckModeSplit(tempCS, bestCS, pm, encTestMode);
        maxCostAllowed = std::min(maxCostAllowed, bestCS->cost);
      }
    }
  }

  if (canTv)
  {
    EncTestMode encTestMode = { ETM_SPLIT_TT_V, ETO_STANDARD, -1, deltaQPForLambdaq, maxCostAllowed };
    if (m_modeCtrl->trySplit(encTestMode, *tempCS, pm))
    {
      for (encTestMode.qp = minQPq; encTestMode.qp <= maxQPq; encTestMode.qp++)
      {
        xCheckModeSplit(tempCS, bestCS, pm, encTestMode);
        maxCostAllowed = std::min(maxCostAllowed, bestCS->cost);
      }
    }
  }

  if (!comprCUCtx.qtBeforeBt && canQt)
  {
    EncTestMode encTestMode = { ETM_SPLIT_QT, ETO_STANDARD, -1, deltaQPForLambdaq, maxCostAllowed };
    if (m_modeCtrl->trySplit(encTestMode, *tempCS, pm))
    {
      for (encTestMode.qp = minQPq; encTestMode.qp <= maxQPq; encTestMode.qp++)
      {
        xCheckModeSplit(tempCS, bestCS, pm, encTestMode);
        maxCostAllowed = std::min(maxCostAllowed, bestCS->cost);
      }
    }
  }
}

//! \}
