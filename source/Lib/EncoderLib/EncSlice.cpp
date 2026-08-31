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

/** \file     EncSlice.cpp
    \brief    slice encoder class
*/

#include "EncSlice.h"

#include "EncLib.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/Picture.h"
#if K0149_BLOCK_STATISTICS
#include "CommonLib/dtrace_blockstatistics.h"
#endif

#include <cctype>
#include <filesystem>
#include <fstream>
#include <math.h>
#include <system_error>

namespace
{
std::string sanitizeTimdStatsPathComponent(const std::string &value)
{
  std::string sanitized;
  sanitized.reserve(value.size());
  for (const unsigned char ch: value)
  {
    sanitized.push_back(std::isalnum(ch) || ch == '.' || ch == '-' || ch == '_' ? char(ch) : '_');
  }
  return sanitized.empty() ? "unknown" : sanitized;
}

std::string escapeTimdStatsCsv(const std::string &value)
{
  if (value.find_first_of(",\"\r\n") == std::string::npos)
  {
    return value;
  }

  std::string escaped = "\"";
  for (const char ch: value)
  {
    escaped += ch;
    if (ch == '"')
    {
      escaped += '"';
    }
  }
  escaped += '"';
  return escaped;
}
}   // namespace

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

EncSlice::EncSlice()
  : m_encCABACTableIdx(I_SLICE)
#if ENABLE_QPA
  , m_adaptedLumaQP(-1)
#endif
{}

EncSlice::~EncSlice() { destroy(); }

void EncSlice::create(int width, int height, ChromaFormat chromaFormat, uint32_t iMaxCUWidth, uint32_t iMaxCUHeight,
                      uint8_t uhTotalDepth)
{
  int numBinBuffers = width / iMaxCUWidth + 1;

  for (int i = 0; i < numBinBuffers; i++)
  {
    m_binVectors.push_back(BinStoreVector());
    m_binVectors[i].reserve(CABAC_SPATIAL_MAX_BINS);
  }
}

void EncSlice::destroy()
{
  xWriteTimdUsageStats();

  // free lambda and QP arrays
  m_vdRdPicLambda.clear();
  m_vdRdPicQp.clear();
  m_viRdPicQp.clear();
  m_binVectors.clear();
}

void EncSlice::init(EncLib *pcEncLib, const SPS &sps)
{
  m_encCfg  = &pcEncLib->m_encCfg;
  m_pcLib   = pcEncLib;
  m_picList = pcEncLib->getListPic();

  m_pcGOPEncoder   = pcEncLib->getGOPEncoder();
  m_pcCuEncoder    = pcEncLib->getCuEncoder();
  m_pcInterSearch  = pcEncLib->getInterSearch();
  m_CABACWriter    = pcEncLib->getCABACEncoder()->getCABACWriter(&sps);
  m_CABACEstimator = pcEncLib->getCABACEncoder()->getCABACEstimator(&sps);
  m_pcTrQuant      = pcEncLib->getTrQuant();
  m_pcRdCost       = pcEncLib->getRdCost();

  // create lambda and QP arrays
  m_vdRdPicLambda.resize(m_encCfg->m_uiDeltaQpRD * 2 + 1);
  m_vdRdPicQp.resize(m_encCfg->m_uiDeltaQpRD * 2 + 1);
  m_viRdPicQp.resize(m_encCfg->m_uiDeltaQpRD * 2 + 1);
  m_pcRateCtrl = pcEncLib->getRateCtrl();

  xPrepareTimdUsageStats();
}

void EncSlice::xPrepareTimdUsageStats()
{
  namespace fs = std::filesystem;

  m_timdUsageByPoc.clear();
  m_timdUsageSequence.clear();
  m_timdUsageShardName.clear();
  m_timdUsageShardPath.clear();
  m_timdUsageStatsPrepared = false;
  m_timdUsageStatsWritten  = false;
  m_CABACWriter->setTimdMergeFlagFracBitsTracking(false);

  if (!m_encCfg || !m_encCfg->m_timdUsageStats)
  {
    return;
  }

  const fs::path bitstreamPath(m_encCfg->m_bitstreamFileName);
  const std::string parentName = bitstreamPath.parent_path().filename().string();
  const std::string rasMarker  = "_RAS_Result_qp";
  const size_t      markerPos  = parentName.find(rasMarker);
  if (markerPos != std::string::npos && markerPos > 0)
  {
    m_timdUsageSequence = parentName.substr(0, markerPos);
  }
  else
  {
    std::string inputStem = fs::path(m_encCfg->m_inputFileName).stem().string();
    const size_t separator = inputStem.find('_');
    m_timdUsageSequence = inputStem.substr(0, separator);
  }
  m_timdUsageSequence = sanitizeTimdStatsPathComponent(m_timdUsageSequence);

  m_timdUsageShardName = sanitizeTimdStatsPathComponent(bitstreamPath.stem().string());
  if (m_pcLib->getLayerId() > 0)
  {
    m_timdUsageShardName += "_layer" + std::to_string(m_pcLib->getLayerId());
  }

  const fs::path statsRoot = m_encCfg->m_timdUsageStatsRoot.empty() ? fs::path("timd_usage_stats")
                                                                    : fs::path(m_encCfg->m_timdUsageStatsRoot);
  const fs::path shardDir = statsRoot / ".shards" / m_timdUsageSequence /
    ("QP" + std::to_string(m_encCfg->m_iQP));

  std::error_code error;
  fs::create_directories(shardDir, error);
  if (error)
  {
    msg(WARNING, "Warning: cannot create TIMD usage statistics directory '%s': %s\n", shardDir.string().c_str(),
        error.message().c_str());
    return;
  }

  const fs::path shardPath = shardDir / (m_timdUsageShardName + ".csv");
  const fs::path tempPath  = fs::path(shardPath.string() + ".tmp");
  fs::remove(shardPath, error);
  error.clear();
  fs::remove(tempPath, error);
  m_timdUsageShardPath     = shardPath.string();
  m_timdUsageStatsPrepared = true;
  m_CABACWriter->setTimdMergeFlagFracBitsTracking(true);
}

void EncSlice::xAccumulateTimdUsageStats(const CodingStructure &cs, const UnitArea &ctuArea, const int poc)
{
  if (!m_timdUsageStatsPrepared)
  {
    return;
  }

  TimdUsageRecord &record = m_timdUsageByPoc[poc];
  record.ctus++;
  uint64_t countedMergeFlagCoded = 0;
  uint64_t countedMergeFlagZero  = 0;
  uint64_t countedMergeFlagOne   = 0;

  for (const CodingUnit &cu: cs.traverseCUs(ctuArea, ChannelType::LUMA))
  {
    if (!cu.Y().valid() || !isLuma(cu.chType))
    {
      continue;
    }

    const uint64_t area = cu.lumaSize().area();
    record.lumaCus++;
    record.lumaSamples += area;

    if (cu.predMode == MODE_INTRA)
    {
      record.intraPredLumaCus++;
      record.intraPredLumaSamples += area;
    }

    if (!cu.timdFlag)
    {
      continue;
    }

    record.timdTotal++;
    record.timdSamples += area;
    if (cu.timdSadFlag && cu.timdMergeFlag)
    {
      record.timdInvalid++;
      record.timdInvalidSamples += area;
    }
    else if (cu.timdMergeFlag)
    {
      record.timdMerge++;
      record.timdMergeSamples += area;
    }
    else if (cu.timdSadFlag)
    {
      record.timdSad++;
      record.timdSadSamples += area;
    }
    else
    {
      record.timdNormal++;
      record.timdNormalSamples += area;
    }

    const bool mergeAvailable = cu.cs->sps->m_useTIMDMerge && CU::allowTimdMerge(cu);
    if (mergeAvailable)
    {
      record.timdMergeAvailable++;
      record.timdMergeAvailableSamples += area;
      if (!cu.timdMergeFlag)
      {
        record.timdMergeNotSelected++;
        record.timdMergeNotSelectedSamples += area;
      }
    }

    // This condition deliberately mirrors CABACWriter::timd_merge_flag(). It counts only bins that are present in the
    // final bitstream, not merge candidates that lost to TIMDSAD or to a non-TIMD mode.
    const bool mergeFlagCoded = mergeAvailable && !cu.timdSadFlag;
    if (mergeFlagCoded)
    {
      record.timdMergeFlagCoded++;
      record.timdMergeFlagCodedSamples += area;
      countedMergeFlagCoded++;
      if (cu.timdMergeFlag)
      {
        record.timdMergeFlagOne++;
        record.timdMergeFlagOneSamples += area;
        countedMergeFlagOne++;
      }
      else
      {
        record.timdMergeFlagZero++;
        record.timdMergeFlagZeroSamples += area;
        countedMergeFlagZero++;
      }
    }
  }

  const TimdMergeFlagFracBitsStats &cabacStats = m_CABACWriter->getTimdMergeFlagFracBitsStats();
  CHECK(cabacStats.coded != countedMergeFlagCoded || cabacStats.zero != countedMergeFlagZero ||
          cabacStats.one != countedMergeFlagOne,
        "Final TIMD-Merge CABAC flag counts do not match final CU traversal");
  record.timdMergeFlagFracBits += cabacStats.fracBits;
  record.timdMergeFlagZeroFracBits += cabacStats.zeroFracBits;
  record.timdMergeFlagOneFracBits += cabacStats.oneFracBits;
  for (unsigned ctxId = 0; ctxId < 2; ctxId++)
  {
    record.timdMergeFlagCtxCoded[ctxId] += cabacStats.ctxCoded[ctxId];
    record.timdMergeFlagCtxZero[ctxId] += cabacStats.ctxZero[ctxId];
    record.timdMergeFlagCtxOne[ctxId] += cabacStats.ctxOne[ctxId];
    record.timdMergeFlagCtxFracBits[ctxId] += cabacStats.ctxFracBits[ctxId];
    record.timdMergeFlagCtxZeroFracBits[ctxId] += cabacStats.ctxZeroFracBits[ctxId];
    record.timdMergeFlagCtxOneFracBits[ctxId] += cabacStats.ctxOneFracBits[ctxId];
  }
}

void EncSlice::xWriteTimdUsageStats()
{
  namespace fs = std::filesystem;

  if (m_timdUsageStatsWritten)
  {
    return;
  }
  m_timdUsageStatsWritten = true;
  if (!m_timdUsageStatsPrepared || !m_encCfg)
  {
    return;
  }

  const fs::path shardPath(m_timdUsageShardPath);
  const fs::path tempPath(shardPath.string() + ".tmp");
  std::ofstream  output(tempPath, std::ios::out | std::ios::trunc);
  if (!output)
  {
    msg(WARNING, "Warning: cannot open TIMD usage statistics file '%s'\n", tempPath.string().c_str());
    return;
  }

  output << "schema_version,sequence,qp,layer_id,shard,bitstream,frame_skip,frames_to_encode,poc,source_frame,"
            "slices,ctus,luma_cus,intra_pred_luma_cus,timd_total,timd_normal,timdsad,timd_merge,timd_invalid,"
            "timd_merge_available,timd_merge_not_selected,timd_merge_flag_coded,timd_merge_flag_zero,"
            "timd_merge_flag_one,"
            "luma_samples,intra_pred_luma_samples,timd_samples,timd_normal_samples,timdsad_samples,"
            "timd_merge_samples,timd_invalid_samples,timd_merge_available_samples,"
            "timd_merge_not_selected_samples,timd_merge_flag_coded_samples,timd_merge_flag_zero_samples,"
            "timd_merge_flag_one_samples,"
            "timd_merge_flag_frac_bits,timd_merge_flag_zero_frac_bits,timd_merge_flag_one_frac_bits,"
            "timd_merge_flag_ctx0_coded,timd_merge_flag_ctx0_zero,timd_merge_flag_ctx0_one,"
            "timd_merge_flag_ctx0_frac_bits,timd_merge_flag_ctx0_zero_frac_bits,"
            "timd_merge_flag_ctx0_one_frac_bits,"
            "timd_merge_flag_ctx1_coded,timd_merge_flag_ctx1_zero,timd_merge_flag_ctx1_one,"
            "timd_merge_flag_ctx1_frac_bits,timd_merge_flag_ctx1_zero_frac_bits,"
            "timd_merge_flag_ctx1_one_frac_bits\n";

  for (const auto &[poc, record]: m_timdUsageByPoc)
  {
    const int64_t sourceFrame = m_encCfg->m_fieldSeqFlag
      ? int64_t(m_encCfg->m_frameSkip) * 2 + int64_t(poc) * m_encCfg->m_temporalSubsampleRatio
      : int64_t(m_encCfg->m_frameSkip) + int64_t(poc) * m_encCfg->m_temporalSubsampleRatio;

    output << 3 << ',' << escapeTimdStatsCsv(m_timdUsageSequence) << ',' << m_encCfg->m_iQP << ','
           << m_pcLib->getLayerId() << ',' << escapeTimdStatsCsv(m_timdUsageShardName) << ','
           << escapeTimdStatsCsv(m_encCfg->m_bitstreamFileName) << ',' << m_encCfg->m_frameSkip << ','
           << m_encCfg->m_framesToBeEncoded << ',' << poc << ',' << sourceFrame << ',' << record.slices << ','
           << record.ctus << ',' << record.lumaCus << ',' << record.intraPredLumaCus << ',' << record.timdTotal << ','
           << record.timdNormal << ',' << record.timdSad << ',' << record.timdMerge << ',' << record.timdInvalid << ','
           << record.timdMergeAvailable << ',' << record.timdMergeNotSelected << ',' << record.timdMergeFlagCoded << ','
           << record.timdMergeFlagZero << ',' << record.timdMergeFlagOne << ','
           << record.lumaSamples << ',' << record.intraPredLumaSamples << ',' << record.timdSamples << ','
           << record.timdNormalSamples << ',' << record.timdSadSamples << ',' << record.timdMergeSamples << ','
           << record.timdInvalidSamples << ',' << record.timdMergeAvailableSamples << ','
           << record.timdMergeNotSelectedSamples << ',' << record.timdMergeFlagCodedSamples << ','
           << record.timdMergeFlagZeroSamples << ',' << record.timdMergeFlagOneSamples << ','
           << record.timdMergeFlagFracBits << ',' << record.timdMergeFlagZeroFracBits << ','
           << record.timdMergeFlagOneFracBits << ',' << record.timdMergeFlagCtxCoded[0] << ','
           << record.timdMergeFlagCtxZero[0] << ',' << record.timdMergeFlagCtxOne[0] << ','
           << record.timdMergeFlagCtxFracBits[0] << ',' << record.timdMergeFlagCtxZeroFracBits[0] << ','
           << record.timdMergeFlagCtxOneFracBits[0] << ',' << record.timdMergeFlagCtxCoded[1] << ','
           << record.timdMergeFlagCtxZero[1] << ',' << record.timdMergeFlagCtxOne[1] << ','
           << record.timdMergeFlagCtxFracBits[1] << ',' << record.timdMergeFlagCtxZeroFracBits[1] << ','
           << record.timdMergeFlagCtxOneFracBits[1] << '\n';
  }
  output.close();
  if (!output)
  {
    msg(WARNING, "Warning: failed while writing TIMD usage statistics file '%s'\n", tempPath.string().c_str());
    return;
  }

  std::error_code error;
  fs::remove(shardPath, error);
  error.clear();
  fs::rename(tempPath, shardPath, error);
  if (error)
  {
    msg(WARNING, "Warning: cannot finalize TIMD usage statistics file '%s': %s\n", shardPath.string().c_str(),
        error.message().c_str());
  }
}

void EncSlice::setUpLambda(Slice *slice, const double dLambda, int qp)
{
  m_pcRdCost->resetStore();
  m_pcTrQuant->resetStore();
  // store lambda
  m_pcRdCost->setLambda(dLambda, slice->m_sps->m_bitDepths);

  // for RDO
  // in RdCost there is only one lambda because the luma and chroma bits are not separated, instead we weight the
  // distortion of chroma.
  double dLambdas[MAX_NUM_COMP] = { dLambda };
  for (uint32_t compIdx = 1; compIdx < MAX_NUM_COMP; compIdx++)
  {
    const CompID compID         = CompID(compIdx);
    int          chromaQPOffset = slice->m_pps->getQpOffset(compID) + slice->getSliceChromaQpDelta(compID);
    int          qpc            = slice->m_sps->getMappedChromaQpValue(compID, qp) + chromaQPOffset;
    double tmpWeight = pow(2.0, (qp - qpc) / 3.0);   // takes into account of the chroma qp mapping and chroma qp Offset
    if (slice->m_depQuantEnabledIdc)
    {
      tmpWeight *=
        (m_encCfg->m_gopSize >= 8 ? pow(2.0, 0.1 / 3.0)
                                  : pow(2.0, 0.2 / 3.0));   // increase chroma weight for dependent quantization (in
                                                            // order to reduce bit rate shift from chroma to luma)
    }
    m_pcRdCost->setDistortionWeight(compID, tmpWeight);
    dLambdas[compIdx] = dLambda / tmpWeight;
  }

#if RDOQ_CHROMA_LAMBDA
  // for RDOQ
  m_pcTrQuant->setLambdas(dLambdas);
#else
  m_pcTrQuant->setLambda(dLambda);
#endif

  // for SAO
  slice->setLambdas(dLambdas);
}

#if ENABLE_QPA

static inline int apprI3Log2(const double d)   // rounded 3*log2(d)
{
  return d < 1.5e-13 ? -128 : int(floor(3.0 * log(d) / log(2.0) + 0.5));
}

static inline int lumaDQPOffset(const uint32_t avgLumaValue, const int bitDepth)
{
  return (1 - int((3 * uint64_t(avgLumaValue * avgLumaValue)) >> uint64_t(2 * bitDepth - 1)));
}

static void filterAndCalculateAverageEnergies(const Pel *pSrc, const ptrdiff_t srcStride, double &hpEner,
                                              const int height, const int width,
                                              const uint32_t bitDepth /* luma bit-depth (4-16) */)
{
  uint64_t saAct = 0;

  // skip first row as there may be a black border frame
  pSrc += srcStride;
  // center rows
  for (int y = 1; y < height - 1; y++)
  {
    // skip column as there may be a black border frame

    for (int x = 1; x < width - 1; x++)   // and columns
    {
      const int f = 12 * pSrc[x] - 2 * (pSrc[x - 1] + pSrc[x + 1] + pSrc[x - srcStride] + pSrc[x + srcStride]) -
        pSrc[x - 1 - srcStride] - pSrc[x + 1 - srcStride] - pSrc[x - 1 + srcStride] - pSrc[x + 1 + srcStride];
      saAct += abs(f);
    }
    // skip column as there may be a black border frame
    pSrc += srcStride;
  }
  // skip last row as there may be a black border frame

  hpEner = double(saAct) / double((width - 2) * (height - 2));

  // lower limit, compensate for highpass amplification
  if (hpEner < double(1 << (bitDepth - 4)))
  {
    hpEner = double(1 << (bitDepth - 4));
  }
}

#ifndef GLOBAL_AVERAGING
#define GLOBAL_AVERAGING 1   // "global" averaging of a_k across a set instead of one picture
#endif

#if GLOBAL_AVERAGING
static double getAveragePictureEnergy(const CPelBuf picOrig, const uint32_t bitDepth)
{
  const double hpEnerPic =
    16.0 * sqrt((3840.0 * 2160.0) / double(picOrig.width * picOrig.height)) * double(1 << (2 * bitDepth - 10));

  return sqrt(hpEnerPic);   // square-root of a_pic value
}
#endif

static int getGlaringColorQPOffset(Picture *const pic, const int ctuAddr, Slice *const pcSlice, const int bitDepth,
                                   uint32_t &avgLumaValue)
{
  const PreCalcValues &pcv       = *pic->m_cs->pcv;
  const ChromaFormat   chrFmt    = pic->chromaFormat;
  const uint32_t       chrWidth  = pcv.maxCUWidth >> getChannelTypeScaleX(ChannelType::CHROMA, chrFmt);
  const uint32_t       chrHeight = pcv.maxCUHeight >> getChannelTypeScaleY(ChannelType::CHROMA, chrFmt);
  const int            midLevel  = 1 << (bitDepth - 1);
  int                  chrValue  = MAX_INT;
  avgLumaValue                   = (pcSlice != nullptr) ? 0 : (uint32_t)pic->getOrigBuf().Y().computeAvg();

  if (ctuAddr >= 0)   // luma
  {
    avgLumaValue = (uint32_t)pic->m_iOffsetCtu[ctuAddr];
  }
  else if (pcSlice != nullptr)
  {
    for (uint32_t ctuIdx = 0; ctuIdx < pcSlice->getNumCtuInSlice(); ctuIdx++)
    {
      uint32_t ctuRsAddr = pcSlice->getCtuAddrInSlice(ctuIdx);
      avgLumaValue += pic->m_iOffsetCtu[ctuRsAddr];
    }
    avgLumaValue = (avgLumaValue + (pcSlice->getNumCtuInSlice() >> 1)) / pcSlice->getNumCtuInSlice();
  }

  for (uint32_t comp = COMP_Cb; comp < MAX_NUM_COMP; comp++)
  {
    const CompID compID = (CompID)comp;
    int          avgCompValue;

    if (ctuAddr >= 0)   // chroma
    {
      const CompArea chrArea = clipArea(CompArea(compID, chrFmt,
                                                 Area((ctuAddr % pcv.widthInCtus) * chrWidth,
                                                      (ctuAddr / pcv.widthInCtus) * chrHeight, chrWidth, chrHeight)),
                                        pic->block(compID));

      avgCompValue = pic->getOrigBuf(chrArea).computeAvg();
    }
    else
    {
      avgCompValue = pic->getOrigBuf(pic->block(compID)).computeAvg();
    }

    if (chrValue > avgCompValue)
    {
      chrValue = avgCompValue;   // minimum of the DC offsets
    }
  }
  CHECK(chrValue < 0, "DC offset cannot be negative!");

  chrValue = (int)avgLumaValue - chrValue;

  if (chrValue > midLevel)
  {
    return apprI3Log2(double(chrValue * chrValue) / double(midLevel * midLevel));
  }

  return 0;
}

static int applyQPAdaptationChroma(Picture *const pic, Slice *const pcSlice, const EncCfg *encCfg, const int sliceQP)
{
  const int bitDepth                  = pcSlice->m_sps->m_bitDepths[ChannelType::LUMA];   // overall image bit-depth
  double    hpEner[MAX_NUM_COMP]      = { 0.0, 0.0, 0.0 };
  int       optSliceChromaQpOffset[2] = { 0, 0 };
  int       savedLumaQP               = -1;
  uint32_t  meanLuma                  = MAX_UINT;

  for (uint32_t comp = 0; comp < getNumberValidComponents(pic->chromaFormat); comp++)
  {
    const CompID  compID  = (CompID)comp;
    const CPelBuf picOrig = pic->getOrigBuf(pic->block(compID));

    filterAndCalculateAverageEnergies(picOrig.buf, picOrig.stride, hpEner[comp], picOrig.height, picOrig.width,
                                      bitDepth - (isChroma(compID) ? 1 : 0));
    if (isChroma(compID))
    {
      const int adaptChromaQPOffset = 2.0 * hpEner[comp] <= hpEner[0] ? 0 : apprI3Log2(2.0 * hpEner[comp] / hpEner[0]);

      if (savedLumaQP < 0)
      {
#if GLOBAL_AVERAGING
        int averageAdaptedLumaQP =
          Clip3(0, MAX_QP, sliceQP + apprI3Log2(hpEner[0] / getAveragePictureEnergy(pic->getOrigBuf().Y(), bitDepth)));
#else
        int averageAdaptedLumaQP = Clip3(0, MAX_QP, sliceQP); // mean slice QP
#endif

        averageAdaptedLumaQP += getGlaringColorQPOffset(pic, -1 /*ctuRsAddr*/, nullptr /*pcSlice*/, bitDepth, meanLuma);

        if (averageAdaptedLumaQP > MAX_QP
#if SHARP_LUMA_DELTA_QP
            && (encCfg->m_lumaLevelToDeltaQPMapping.mode != LUMALVL_TO_DQP_NUM_MODES)
#endif
        )
          averageAdaptedLumaQP = MAX_QP;
#if SHARP_LUMA_DELTA_QP

        // change mean picture QP index based on picture's average luma value (Sharp)
        if (encCfg->m_lumaLevelToDeltaQPMapping.mode == LUMALVL_TO_DQP_NUM_MODES)
        {
          if (meanLuma == MAX_UINT)
          {
            meanLuma = pic->getOrigBuf().Y().computeAvg();
          }

          averageAdaptedLumaQP = Clip3(0, MAX_QP, averageAdaptedLumaQP + lumaDQPOffset(meanLuma, bitDepth));
        }
#endif

        savedLumaQP = averageAdaptedLumaQP;
      }   // savedLumaQP < 0

      const int lumaChromaMappingDQP = savedLumaQP - pcSlice->m_sps->getMappedChromaQpValue(compID, savedLumaQP);

      optSliceChromaQpOffset[comp - 1] = std::min(3 + lumaChromaMappingDQP, adaptChromaQPOffset + lumaChromaMappingDQP);
    }
  }

  std::memcpy(encCfg->m_sliceChromaQpOffsetIntraOrPeriodic, optSliceChromaQpOffset,
              sizeof(encCfg->m_sliceChromaQpOffsetIntraOrPeriodic));

  return savedLumaQP;
}

#endif   // ENABLE_QPA

/**
 - non-referenced frame marking
 - QP computation based on temporal structure
 - lambda computation based on QP
 - set temporal layer ID and the parameter sets
 .
 \param pic         picture class
 \param pocLast       POC of last picture
 \param pocCurr       current POC
 \param numPicRcvd   number of received pictures
 \param gopId        POC offset for hierarchical structure
 \param rpcSlice      slice header class
 \param isField       true for field coding
 */
void EncSlice::initEncSlice(Picture *pic, const int pocLast, const int pocCurr, const int gopId, Slice *&rpcSlice,
                            const bool isField, bool isEncodeLtRef, int layerId, NalUnitType nalType)
{
  double     dQP;
  double     dLambda;
  PicHeader *picHeader = pic->m_cs->picHeader;
  pic->m_cs->resetPrevPLT(pic->m_cs->prevPLT);

  rpcSlice              = pic->m_slices[0];
  rpcSlice->m_pic       = pic;
  rpcSlice->m_picHeader = picHeader;
  rpcSlice->initSlice();
  rpcSlice->m_nuhLayerId = layerId;

  int multipleFactor = m_encCfg->m_compositeRefEnabled ? 2 : 1;
  if (m_encCfg->m_compositeRefEnabled && isEncodeLtRef)
  {
    picHeader->m_picOutputFlag = false;
  }
  else
  {
    picHeader->m_picOutputFlag = true;
  }
  rpcSlice->m_poc = pocCurr;

  if (m_encCfg->m_costMode != COST_LOSSLESS_CODING)
  {
    rpcSlice->m_depQuantEnabledIdc           = m_encCfg->m_DepQuantEnabledIdc;
    rpcSlice->m_signDataHidingEnabledFlag    = m_encCfg->m_SignDataHidingEnabledFlag;
    rpcSlice->m_tsResidualCodingDisabledFlag = false;

    CHECK((m_encCfg->m_DepQuantEnabledIdc || m_encCfg->m_SignDataHidingEnabledFlag) &&
            rpcSlice->m_tsResidualCodingDisabledFlag,
          "TSRC cannot be bypassed if either DQ or SDH are enabled at slice level.");
  }
  else
  {
    rpcSlice->m_depQuantEnabledIdc        = 0;   // should be disabled for lossless
    rpcSlice->m_signDataHidingEnabledFlag = false;   // should be disabled for lossless
    if (m_encCfg->m_TSRCdisableLL)
    {
      rpcSlice->m_tsResidualCodingDisabledFlag = true;
    }
  }

#if SHARP_LUMA_DELTA_QP
  pic->m_fieldPic = isField;
  m_gopID         = gopId;
#endif

  // depth computation based on GOP size
  int hierPredLayerIdx;
  {
    int poc = rpcSlice->m_poc;
    if (isField)
    {
      poc = (poc / 2) % (m_encCfg->m_gopSize / 2);
    }
    else
    {
      poc = poc % (m_encCfg->m_gopSize * multipleFactor);
    }

    if (poc == 0)
    {
      hierPredLayerIdx = 0;
    }
    else
    {
      int step         = m_encCfg->m_gopSize * multipleFactor;
      hierPredLayerIdx = 0;
      for (int i = step >> 1; i >= 1; i >>= 1)
      {
        for (int j = i; j < (m_encCfg->m_gopSize * multipleFactor); j += step)
        {
          if (j == poc)
          {
            i = 0;
            break;
          }
        }
        step >>= 1;
        hierPredLayerIdx++;
      }
    }

    if (m_encCfg->m_harmonizeGopFirstFieldCoupleEnabled && poc != 0)
    {
      if (isField && ((rpcSlice->m_poc % 2) == 1))
      {
        hierPredLayerIdx++;
      }
    }
  }

  // slice type
  SliceType eSliceType;

  eSliceType          = B_SLICE;
  const bool useIlRef = m_encCfg->m_avoidIntraInDepLayer && rpcSlice->m_pic->m_cs->vps &&
    m_encCfg->m_numRefLayers[rpcSlice->m_pic->m_cs->vps->m_generalLayerIdx[layerId]];
  if (m_encCfg->m_intraPeriod > 0)
  {
    if (!(isField && pocLast == 1) || !m_encCfg->m_efficientFieldIRAPEnabled)
    {
      if (m_encCfg->m_decodingRefreshType == 3)
      {
        eSliceType = (pocLast == 0 || pocCurr % (m_encCfg->m_intraPeriod * multipleFactor) == 0 ||
                      m_pcGOPEncoder->getGOPSize() == 0) &&
            (!useIlRef)
          ? I_SLICE
          : eSliceType;
      }
      else
      {
        eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % (m_encCfg->m_intraPeriod * multipleFactor) == 0 ||
                      m_pcGOPEncoder->getGOPSize() == 0) &&
            (!useIlRef)
          ? I_SLICE
          : eSliceType;
      }
    }
  }
  else
  {
    eSliceType = (pocLast == 0 || pocCurr == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
  }

  rpcSlice->m_hierPredLayerIdx = hierPredLayerIdx;
  rpcSlice->m_eSliceType       = eSliceType;
#if ENABLE_CABAC_DUMP
  rpcSlice->m_cabacInitSliceType = eSliceType;
#endif

  // ------------------------------------------------------------------------------------------------------------------
  // Non-referenced frame marking
  // ------------------------------------------------------------------------------------------------------------------

  pic->m_referenced = true;

  // ------------------------------------------------------------------------------------------------------------------
  // QP setting
  // ------------------------------------------------------------------------------------------------------------------

  rpcSlice->m_eNalUnitType = nalType;
  dQP = m_pcLib->getQPForPicture(gopId, rpcSlice, rpcSlice->m_eSliceType, rpcSlice->m_eNalUnitType, rpcSlice->m_poc);

  // ------------------------------------------------------------------------------------------------------------------
  // Lambda computation
  // ------------------------------------------------------------------------------------------------------------------

  const int temporalId = m_encCfg->m_GOPList[gopId].m_temporalId;
#if !SHARP_LUMA_DELTA_QP
  const std::vector<double> &intraLambdaModifiers = m_encCfg->m_adIntraLambdaModifier;
#endif
  int    qp;
  double dOrigQP = dQP;

  // pre-compute lambda and QP values for all possible QP candidates
  for (int iDQpIdx = 0; iDQpIdx < 2 * m_encCfg->m_uiDeltaQpRD + 1; iDQpIdx++)
  {
    // compute QP value
    dQP = dOrigQP + ((iDQpIdx + 1) >> 1) * (iDQpIdx % 2 ? -1 : 1);
    // compute lambda value
#if SHARP_LUMA_DELTA_QP
    dLambda = calculateLambda(rpcSlice, gopId, dQP, dQP, qp);
#else
    dLambda = initializeLambda(rpcSlice, gopId, int(dQP + 0.5), dQP);
    qp      = Clip3(-rpcSlice->m_sps->m_qpBDOffset[ChannelType::LUMA], MAX_QP, int(dQP + 0.5));
#endif

    m_vdRdPicLambda[iDQpIdx] = dLambda;
    m_vdRdPicQp[iDQpIdx]     = dQP;
    m_viRdPicQp[iDQpIdx]     = qp;
  }

  // obtain dQP = 0 case
  dLambda = m_vdRdPicLambda[0];
  dQP     = m_vdRdPicQp[0];
  qp      = m_viRdPicQp[0];

#if W0038_CQP_ADJ
#if ENABLE_QPA
  m_adaptedLumaQP = -1;

  if ((m_encCfg->m_bUsePerceptQPA || m_encCfg->m_sliceChromaQpOffsetPeriodicity > 0) &&
      !m_encCfg->m_RCEnableRateControl && rpcSlice->m_pps->m_sliceChromaQpFlag &&
      (rpcSlice->isIntra() ||
       (m_encCfg->m_sliceChromaQpOffsetPeriodicity > 0 &&
        (rpcSlice->m_poc % m_encCfg->m_sliceChromaQpOffsetPeriodicity) == 0)))
  {
    m_adaptedLumaQP = applyQPAdaptationChroma(pic, rpcSlice, m_encCfg, qp);
  }
#endif
  if (rpcSlice->m_pps->m_sliceChromaQpFlag)
  {
    const bool bUseIntraOrPeriodicOffset = (rpcSlice->isIntra() && !rpcSlice->m_ibcFlag) ||
      (m_encCfg->m_sliceChromaQpOffsetPeriodicity > 0 &&
       (rpcSlice->m_poc % m_encCfg->m_sliceChromaQpOffsetPeriodicity) == 0);
    int cbQP = bUseIntraOrPeriodicOffset ? m_encCfg->m_sliceChromaQpOffsetIntraOrPeriodic[0]
                                         : m_encCfg->m_GOPList[gopId].m_CbQPoffset;
    int crQP = bUseIntraOrPeriodicOffset ? m_encCfg->m_sliceChromaQpOffsetIntraOrPeriodic[1]
                                         : m_encCfg->m_GOPList[gopId].m_CrQPoffset;
    // adjust chroma QP such that it corresponds to the luma QP change when encoding in reduced resolution
    if (m_encCfg->m_gopBasedRPREnabledFlag || m_encCfg->m_rprFunctionalityTestingEnabledFlag)
    {
      auto mappedQpDelta = [&](CompID c, int qpOffset) -> int
      {
        int       qpBefore        = std::min(MAX_QP, qp - qpOffset);
        int       qpOffsetLimited = qp - qpBefore;
        const int mappedQpBefore  = rpcSlice->m_sps->getMappedChromaQpValue(c, qp - qpOffsetLimited);
        const int mappedQpAfter   = rpcSlice->m_sps->getMappedChromaQpValue(c, qp);
        return mappedQpBefore - mappedQpAfter + qpOffsetLimited;
      };
      if (m_encCfg->m_rprFunctionalityTestingEnabledFlag)
      {
        int currPoc    = rpcSlice->m_poc + m_encCfg->m_frameSkip;
        int rprSegment = currPoc / m_encCfg->m_rprSwitchingSegmentSize % m_encCfg->m_rprSwitchingListSize;
        cbQP += mappedQpDelta(COMP_Cb, m_encCfg->m_rprSwitchingQPOffsetOrderList[rprSegment]);
        crQP += mappedQpDelta(COMP_Cr, m_encCfg->m_rprSwitchingQPOffsetOrderList[rprSegment]);
      }
      else
      {
        if (rpcSlice->m_pps->m_ppsId == ENC_PPS_ID_RPR)   // ScalingRatioHor/ScalingRatioVer
        {
          cbQP += mappedQpDelta(COMP_Cb, m_encCfg->m_qpOffsetChromaRPR);
          crQP += mappedQpDelta(COMP_Cr, m_encCfg->m_qpOffsetChromaRPR);
        }
        else if (rpcSlice->m_pps->m_ppsId == ENC_PPS_ID_RPR2)   // ScalingRatioHor2/ScalingRatioVer2
        {
          cbQP += mappedQpDelta(COMP_Cb, m_encCfg->m_qpOffsetChromaRPR2);
          crQP += mappedQpDelta(COMP_Cr, m_encCfg->m_qpOffsetChromaRPR2);
        }
        else if (rpcSlice->m_pps->m_ppsId == ENC_PPS_ID_RPR3)   // ScalingRatioHor3/ScalingRatioVer3
        {
          cbQP += mappedQpDelta(COMP_Cb, m_encCfg->m_qpOffsetChromaRPR3);
          crQP += mappedQpDelta(COMP_Cr, m_encCfg->m_qpOffsetChromaRPR3);
        }
      }
    }
    int cbCrQP = (cbQP + crQP) >> 1;   // use floor of average chroma QP offset for joint-Cb/Cr coding

    cbQP = Clip3(-12, 12, cbQP + rpcSlice->m_pps->getQpOffset(COMP_Cb)) - rpcSlice->m_pps->getQpOffset(COMP_Cb);
    crQP = Clip3(-12, 12, crQP + rpcSlice->m_pps->getQpOffset(COMP_Cr)) - rpcSlice->m_pps->getQpOffset(COMP_Cr);
    rpcSlice->setSliceChromaQpDelta(COMP_Cb, Clip3(-12, 12, cbQP));
    CHECK(!(rpcSlice->getSliceChromaQpDelta(COMP_Cb) + rpcSlice->m_pps->getQpOffset(COMP_Cb) <= 12 &&
            rpcSlice->getSliceChromaQpDelta(COMP_Cb) + rpcSlice->m_pps->getQpOffset(COMP_Cb) >= -12),
          "Unspecified error");
    rpcSlice->setSliceChromaQpDelta(COMP_Cr, Clip3(-12, 12, crQP));
    CHECK(!(rpcSlice->getSliceChromaQpDelta(COMP_Cr) + rpcSlice->m_pps->getQpOffset(COMP_Cr) <= 12 &&
            rpcSlice->getSliceChromaQpDelta(COMP_Cr) + rpcSlice->m_pps->getQpOffset(COMP_Cr) >= -12),
          "Unspecified error");
    if (rpcSlice->m_sps->m_jointCbCrEnabledFlag)
    {
      cbCrQP =
        Clip3(-12, 12, cbCrQP + rpcSlice->m_pps->getQpOffset(JOINT_CbCr)) - rpcSlice->m_pps->getQpOffset(JOINT_CbCr);
      rpcSlice->setSliceChromaQpDelta(JOINT_CbCr, Clip3(-12, 12, cbCrQP));
    }
  }
  else
  {
    rpcSlice->setSliceChromaQpDelta(COMP_Cb, 0);
    rpcSlice->setSliceChromaQpDelta(COMP_Cr, 0);
    rpcSlice->setSliceChromaQpDelta(JOINT_CbCr, 0);
  }
#endif

#if RDOQ_CHROMA_LAMBDA
  m_pcRdCost->setDistortionWeight(COMP_Y, 1.0);   // no chroma weighting for luma
#endif
  setUpLambda(rpcSlice, dLambda, qp);

#if WCG_EXT
  // cost = Distortion + Lambda*R,
  // when QP is adjusted by luma, distortion is changed, so we have to adjust lambda to match the distortion, then the
  // cost function becomes costA = Distortion + AdjustedLambda * R          -- currently, costA is still used when
  // calculating intermediate cost of using SAD, HAD, resisual etc. an alternative way is to weight the distortion to
  // before the luma QP adjustment, then the cost function becomes costB = weightedDistortion + Lambda * R          --
  // currently, costB is used to calculat final cost, and when DF_FUNC is DF_DEFAULT
  m_pcRdCost->saveUnadjustedLambda();
#endif

  if (m_encCfg->m_bFastMEForGenBLowDelayEnabled)
  {
    // restore original slice type

    if (m_encCfg->m_intraPeriod > 0)
    {
      if (!(isField && pocLast == 1) || !m_encCfg->m_efficientFieldIRAPEnabled)
      {
        if (m_encCfg->m_decodingRefreshType == 3)
        {
          eSliceType = (pocLast == 0 || pocCurr % (m_encCfg->m_intraPeriod * multipleFactor) == 0 ||
                        m_pcGOPEncoder->getGOPSize() == 0) &&
              (!useIlRef)
            ? I_SLICE
            : eSliceType;
        }
        else
        {
          eSliceType =
            (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % (m_encCfg->m_intraPeriod * multipleFactor) == 0 ||
             m_pcGOPEncoder->getGOPSize() == 0) &&
              (!useIlRef)
            ? I_SLICE
            : eSliceType;
        }
      }
    }
    else
    {
      eSliceType = (pocLast == 0 || pocCurr == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
    }

    rpcSlice->m_eSliceType = eSliceType;
#if ENABLE_CABAC_DUMP
    rpcSlice->m_cabacInitSliceType = eSliceType;
#endif
  }

  if (m_encCfg->m_recalculateQPAccordingToLambda)
  {
    dQP = xGetQPValueAccordingToLambda(dLambda);
    qp  = Clip3(-rpcSlice->m_sps->m_qpBDOffset[ChannelType::LUMA], MAX_QP, (int)floor(dQP + 0.5));
  }

  rpcSlice->m_iSliceQp = qp;
  pic->m_lossyQP       = qp;
  if ((!rpcSlice->m_tsResidualCodingDisabledFlag) && (rpcSlice->m_sps->m_spsRangeExtension.m_tsrcRicePresentFlag))
  {
    rpcSlice->m_tsrcIndex = Clip3(MIN_TSRC_RICE, MAX_TSRC_RICE, (int)((19 - qp) / 6)) - 1;
  }
#if !W0038_CQP_ADJ
  rpcSlice->setSliceChromaQpDelta(COMP_Cb, 0);
  rpcSlice->setSliceChromaQpDelta(COMP_Cr, 0);
  rpcSlice->setSliceChromaQpDelta(JOINT_CbCr, 0);
#endif
  rpcSlice->m_chromaQpAdjEnabled =
    rpcSlice->m_pps->getCuChromaQpOffsetListEnabledFlag() && m_encCfg->m_cuChromaQpOffsetEnabled;
  rpcSlice->m_numRefIdx[RPL0] = m_encCfg->m_RPLList0[gopId].m_numRefPicsActive;
  rpcSlice->m_numRefIdx[RPL1] = m_encCfg->m_RPLList1[gopId].m_numRefPicsActive;

  if (m_encCfg->m_deblockingFilterMetric)
  {
    rpcSlice->m_deblockingFilterOverrideFlag     = true;
    rpcSlice->m_deblockingFilterDisable          = false;
    rpcSlice->m_deblockingFilterBetaOffsetDiv2   = 0;
    rpcSlice->m_deblockingFilterTcOffsetDiv2     = 0;
    rpcSlice->m_deblockingFilterCbBetaOffsetDiv2 = 0;
    rpcSlice->m_deblockingFilterCbTcOffsetDiv2   = 0;
    rpcSlice->m_deblockingFilterCrBetaOffsetDiv2 = 0;
    rpcSlice->m_deblockingFilterCrTcOffsetDiv2   = 0;
  }
  else if (rpcSlice->m_pps->m_deblockingFilterControlPresentFlag)
  {
    rpcSlice->m_deblockingFilterOverrideFlag =
      rpcSlice->m_pps->m_deblockingFilterOverrideEnabledFlag && !rpcSlice->m_pps->m_ppsDeblockingFilterDisabledFlag;
    rpcSlice->m_deblockingFilterDisable = rpcSlice->m_pps->m_ppsDeblockingFilterDisabledFlag;
    if (!rpcSlice->m_deblockingFilterDisable)
    {
      if (rpcSlice->m_deblockingFilterOverrideFlag && eSliceType != I_SLICE)
      {
        rpcSlice->m_deblockingFilterBetaOffsetDiv2 =
          m_encCfg->m_GOPList[gopId].m_betaOffsetDiv2 + m_encCfg->m_deblockingFilterBetaOffsetDiv2;
        rpcSlice->m_deblockingFilterTcOffsetDiv2 =
          m_encCfg->m_GOPList[gopId].m_tcOffsetDiv2 + m_encCfg->m_deblockingFilterTcOffsetDiv2;
        if (rpcSlice->m_pps->m_usePPSChromaTool)
        {
          rpcSlice->m_deblockingFilterCbBetaOffsetDiv2 =
            m_encCfg->m_GOPList[gopId].m_CbBetaOffsetDiv2 + m_encCfg->m_deblockingFilterCbBetaOffsetDiv2;
          rpcSlice->m_deblockingFilterCbTcOffsetDiv2 =
            m_encCfg->m_GOPList[gopId].m_CbTcOffsetDiv2 + m_encCfg->m_deblockingFilterCbTcOffsetDiv2;
          rpcSlice->m_deblockingFilterCrBetaOffsetDiv2 =
            m_encCfg->m_GOPList[gopId].m_CrBetaOffsetDiv2 + m_encCfg->m_deblockingFilterCrBetaOffsetDiv2;
          rpcSlice->m_deblockingFilterCrTcOffsetDiv2 =
            m_encCfg->m_GOPList[gopId].m_CrTcOffsetDiv2 + m_encCfg->m_deblockingFilterCrTcOffsetDiv2;
        }
        else
        {
          rpcSlice->m_deblockingFilterCbBetaOffsetDiv2 = rpcSlice->m_deblockingFilterBetaOffsetDiv2;
          rpcSlice->m_deblockingFilterCbTcOffsetDiv2   = rpcSlice->m_deblockingFilterTcOffsetDiv2;
          rpcSlice->m_deblockingFilterCrBetaOffsetDiv2 = rpcSlice->m_deblockingFilterBetaOffsetDiv2;
          rpcSlice->m_deblockingFilterCrTcOffsetDiv2   = rpcSlice->m_deblockingFilterTcOffsetDiv2;
        }
      }
      else
      {
        rpcSlice->m_deblockingFilterBetaOffsetDiv2   = m_encCfg->m_deblockingFilterBetaOffsetDiv2;
        rpcSlice->m_deblockingFilterTcOffsetDiv2     = m_encCfg->m_deblockingFilterTcOffsetDiv2;
        rpcSlice->m_deblockingFilterCbBetaOffsetDiv2 = m_encCfg->m_deblockingFilterCbBetaOffsetDiv2;
        rpcSlice->m_deblockingFilterCbTcOffsetDiv2   = m_encCfg->m_deblockingFilterCbTcOffsetDiv2;
        rpcSlice->m_deblockingFilterCrBetaOffsetDiv2 = m_encCfg->m_deblockingFilterCrBetaOffsetDiv2;
        rpcSlice->m_deblockingFilterCrTcOffsetDiv2   = m_encCfg->m_deblockingFilterCrTcOffsetDiv2;
      }
    }
  }
  else
  {
    rpcSlice->m_deblockingFilterOverrideFlag     = false;
    rpcSlice->m_deblockingFilterDisable          = false;
    rpcSlice->m_deblockingFilterBetaOffsetDiv2   = 0;
    rpcSlice->m_deblockingFilterTcOffsetDiv2     = 0;
    rpcSlice->m_deblockingFilterCbBetaOffsetDiv2 = 0;
    rpcSlice->m_deblockingFilterCbTcOffsetDiv2   = 0;
    rpcSlice->m_deblockingFilterCrBetaOffsetDiv2 = 0;
    rpcSlice->m_deblockingFilterCrTcOffsetDiv2   = 0;
  }

  pic->m_temporalId = temporalId;
  if (eSliceType == I_SLICE)
  {
    pic->m_temporalId = 0;
  }
  rpcSlice->m_uiTLayer = pic->m_temporalId;

  rpcSlice->m_disableSATDForRd = false;

  if ((m_encCfg->m_ibcHashSearch && m_encCfg->m_ibcMode) || m_encCfg->m_allowDisFracMMVD)
  {
    m_pcCuEncoder->getIbcHashMap().destroy();
    m_pcCuEncoder->getIbcHashMap().init(pic->m_cs->pps->m_picWidthInLumaSamples,
                                        pic->m_cs->pps->m_picHeightInLumaSamples);
  }

  if (rpcSlice->m_sps->m_spsRangeExtension.m_rrcRiceExtensionEnableFlag)
  {
    int bitDepth  = rpcSlice->m_sps->m_bitDepths[ChannelType::LUMA];
    int baseLevel = (bitDepth > 12) ? (rpcSlice->isIntra() ? 5 : 2 * 5) : (rpcSlice->isIntra() ? 2 * 5 : 3 * 5);
    rpcSlice->m_riceBaseLevelValue = baseLevel;
  }
  else
  {
    rpcSlice->m_riceBaseLevelValue = 4;
  }

  rpcSlice->lfCccmClearControlInformation();
}

double EncSlice::initializeLambda(const Slice *slice, const int gopId, const int refQP, const double dQP)
{
  const int       bitDepthLuma  = slice->m_sps->m_bitDepths[ChannelType::LUMA];
  const int       bitDepthShift = 6 * (bitDepthLuma - 8 - DISTORTION_PRECISION_ADJUSTMENT(bitDepthLuma)) - 12;
  const int       numberBFrames = m_encCfg->m_gopSize - 1;
  const SliceType sliceType     = slice->m_eSliceType;
  const int       temporalId    = m_encCfg->m_GOPList[gopId].m_temporalId;
  const std::vector<double> &intraLambdaModifiers = m_encCfg->m_adIntraLambdaModifier;
  // case #1: I or P slices (key-frame)
  double                     dQPFactor            = m_encCfg->m_GOPList[gopId].m_QPFactor;
  double                     dLambda, lambdaModifier;

  if (sliceType == I_SLICE)
  {
    if ((m_encCfg->m_dIntraQpFactor >= 0.0) && (m_encCfg->m_GOPList[gopId].m_sliceType != I_SLICE))
    {
      dQPFactor = m_encCfg->m_dIntraQpFactor;
    }
    else
    {
      if (m_encCfg->m_lambdaFromQPEnable)
      {
        dQPFactor = 0.57;
      }
      else
      {
        dQPFactor =
          0.57 * (1.0 - Clip3(0.0, 0.5, 0.05 * double(slice->m_pic->m_fieldPic ? numberBFrames >> 1 : numberBFrames)));
      }
    }
  }
  else if (m_encCfg->m_lambdaFromQPEnable)
  {
    dQPFactor = 0.57;
  }

  dLambda = dQPFactor * pow(2.0, (dQP + m_encCfg->m_lambdaScaleTowardsNextQP + bitDepthShift) / 3.0);

  if (slice->m_hierPredLayerIdx > 0 && !m_encCfg->m_lambdaFromQPEnable)
  {
    dLambda *= Clip3(2.0, 4.0, ((refQP + bitDepthShift) / 6.0));
  }
  // if Hadamard is used in motion estimation process
  if (!m_encCfg->m_bUseHADME && (sliceType != I_SLICE))
  {
    dLambda *= 0.95;
  }
  if ((sliceType != I_SLICE) || intraLambdaModifiers.empty())
  {
    lambdaModifier = m_encCfg->m_adLambdaModifier[temporalId];
  }
  else
  {
    lambdaModifier =
      intraLambdaModifiers[temporalId < intraLambdaModifiers.size() ? temporalId : intraLambdaModifiers.size() - 1];
  }
  dLambda *= lambdaModifier;

  return dLambda;
}

#if SHARP_LUMA_DELTA_QP || ENABLE_QPA_SUB_CTU
double EncSlice::calculateLambda(const Slice *slice,
                                 const int    gopId,   // entry in the GOP table
                                 const double refQP,   // initial slice-level QP
                                 const double dQP,   // initial double-precision QP
                                 int         &qp)   // returned integer QP.
{
  double dLambda = initializeLambda(slice, gopId, int(refQP + 0.5), dQP);
  qp             = Clip3(-slice->m_sps->m_qpBDOffset[ChannelType::LUMA], MAX_QP, int(dQP + 0.5));

  if (slice->m_depQuantEnabledIdc)
  {
    dLambda *= pow(
      2.0, 0.25 / 3.0);   // slight lambda adjustment for dependent quantization (due to different slope of quantizer)
  }

  // NOTE: the lambda modifiers that are sometimes applied later might be best always applied in here.
  return dLambda;
}
#endif

void EncSlice::resetQP(Picture *pic, int sliceQP, double lambda)
{
  Slice *slice = pic->m_slices[0];

  // store lambda
  slice->m_iSliceQp = sliceQP;
#if RDOQ_CHROMA_LAMBDA
  m_pcRdCost->setDistortionWeight(COMP_Y, 1.0);   // no chroma weighting for luma
#endif
  setUpLambda(slice, lambda, sliceQP);
#if WCG_EXT
  if (!m_encCfg->m_lumaLevelToDeltaQPMapping.isEnabled())
  {
    m_pcRdCost->saveUnadjustedLambda();
  }
#endif
}

#if ENABLE_QPA
static bool applyQPAdaptation(Picture *const pic, Slice *const pcSlice, const PreCalcValues &pcv,
                              const bool useSharpLumaDQP, const bool useFrameWiseQPA,
                              const int previouslyAdaptedLumaQP = -1)
{
  const int bitDepth        = pcSlice->m_sps->m_bitDepths[ChannelType::LUMA];
  const int iQPIndex        = pcSlice->m_iSliceQp;   // initial QP index for current slice, used in following loops
  bool      sliceQPModified = false;
  uint32_t  meanLuma        = MAX_UINT;
  double    hpEnerAvg       = 0.0;

#if GLOBAL_AVERAGING
  if (!useFrameWiseQPA || previouslyAdaptedLumaQP < 0)   // mean visual activity value and luma value in each CTU
#endif
  {
    for (uint32_t ctuIdx = 0; ctuIdx < pcSlice->getNumCtuInSlice(); ctuIdx++)
    {
      uint32_t       ctuRsAddr = pcSlice->getCtuAddrInSlice(ctuIdx);
      const Position pos((ctuRsAddr % pcv.widthInCtus) * pcv.maxCUWidth,
                         (ctuRsAddr / pcv.widthInCtus) * pcv.maxCUHeight);
      const CompArea ctuArea =
        clipArea(CompArea(COMP_Y, pic->chromaFormat, Area(pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight)), pic->Y());
      const CompArea fltArea =
        clipArea(CompArea(COMP_Y, pic->chromaFormat,
                          Area(pos.x > 0 ? pos.x - 1 : 0, pos.y > 0 ? pos.y - 1 : 0,
                               pcv.maxCUWidth + (pos.x > 0 ? 2 : 1), pcv.maxCUHeight + (pos.y > 0 ? 2 : 1))),
                 pic->Y());
      const CPelBuf picOrig = pic->getOrigBuf(fltArea);
      double        hpEner  = 0.0;

      filterAndCalculateAverageEnergies(picOrig.buf, picOrig.stride, hpEner, picOrig.height, picOrig.width, bitDepth);
      hpEnerAvg += hpEner;
      pic->m_uEnerHpCtu[ctuRsAddr] = hpEner;
      pic->m_iOffsetCtu[ctuRsAddr] = pic->getOrigBuf(ctuArea).computeAvg();
    }

    hpEnerAvg /= double(pcSlice->getNumCtuInSlice());
  }
#if GLOBAL_AVERAGING
  const double hpEnerPic = 1.0 / getAveragePictureEnergy(pic->getOrigBuf().Y(), bitDepth);   // inverse, speed
#else
  const double hpEnerPic = 1.0 / hpEnerAvg; // speedup: multiply instead of divide in loop below; 1.0 for tuning
#endif

  if (useFrameWiseQPA || (iQPIndex >= MAX_QP))
  {
    int iQPFixed = (previouslyAdaptedLumaQP < 0) ? Clip3(0, MAX_QP, iQPIndex + apprI3Log2(hpEnerAvg * hpEnerPic))
                                                 : previouslyAdaptedLumaQP;

    if (isChromaEnabled(pic->chromaFormat) && (iQPIndex < MAX_QP) && (previouslyAdaptedLumaQP < 0))
    {
      iQPFixed += getGlaringColorQPOffset(pic, -1 /*ctuRsAddr*/, pcSlice, bitDepth, meanLuma);

      if (iQPFixed > MAX_QP
#if SHARP_LUMA_DELTA_QP
          && !useSharpLumaDQP
#endif
      )
      {
        iQPFixed = MAX_QP;
      }
    }
#if SHARP_LUMA_DELTA_QP

    // change new fixed QP based on average CTU luma value (Sharp)
    if (useSharpLumaDQP && (iQPIndex < MAX_QP) && (previouslyAdaptedLumaQP < 0))
    {
      if (meanLuma == MAX_UINT)   // collect picture mean luma value
      {
        meanLuma = 0;

        for (uint32_t ctuIdx = 0; ctuIdx < pcSlice->getNumCtuInSlice(); ctuIdx++)
        {
          uint32_t ctuRsAddr = pcSlice->getCtuAddrInSlice(ctuIdx);

          meanLuma += pic->m_iOffsetCtu[ctuRsAddr];   // CTU mean
        }
        meanLuma = (meanLuma + (pcSlice->getNumCtuInSlice() >> 1)) / pcSlice->getNumCtuInSlice();
      }
      iQPFixed = Clip3(0, MAX_QP, iQPFixed + lumaDQPOffset(meanLuma, bitDepth));
    }
#endif

    if (iQPIndex >= MAX_QP)
    {
      iQPFixed = MAX_QP;
    }
    else if (iQPFixed != iQPIndex)
    {
      const double *oldLambdas               = pcSlice->getLambdas();
      const double  corrFactor               = pow(2.0, double(iQPFixed - iQPIndex) / 3.0);
      const double  newLambdas[MAX_NUM_COMP] = { oldLambdas[0] * corrFactor, oldLambdas[1] * corrFactor,
                                                 oldLambdas[2] * corrFactor };

      CHECK(iQPIndex != pcSlice->m_iSliceQpBase, "Invalid slice QP!");
      pcSlice->setLambdas(newLambdas);
      pcSlice->m_iSliceQp     = iQPFixed;   // update the slice/base QPs
      pcSlice->m_iSliceQpBase = iQPFixed;

      sliceQPModified = true;
    }

    for (uint32_t ctuIdx = 0; ctuIdx < pcSlice->getNumCtuInSlice(); ctuIdx++)
    {
      uint32_t ctuRsAddr = pcSlice->getCtuAddrInSlice(ctuIdx);

      pic->m_iOffsetCtu[ctuRsAddr] = (Pel)iQPFixed;   // fixed QPs
    }
  }
  else   // CTU-wise QPA
  {
    for (uint32_t ctuIdx = 0; ctuIdx < pcSlice->getNumCtuInSlice(); ctuIdx++)
    {
      uint32_t ctuRsAddr = pcSlice->getCtuAddrInSlice(ctuIdx);

      int iQPAdapt = Clip3(0, MAX_QP, iQPIndex + apprI3Log2(pic->m_uEnerHpCtu[ctuRsAddr] * hpEnerPic));

      if (pcv.widthInCtus > 1)   // try to enforce CTU SNR greater than zero dB
      {
        meanLuma = (uint32_t)pic->m_iOffsetCtu[ctuRsAddr];

        if (isChromaEnabled(pic->chromaFormat))
        {
          iQPAdapt += getGlaringColorQPOffset(pic, (int)ctuRsAddr, nullptr, bitDepth, meanLuma);

          if (iQPAdapt > MAX_QP
#if SHARP_LUMA_DELTA_QP
              && !useSharpLumaDQP
#endif
          )
          {
            iQPAdapt = MAX_QP;
          }
          CHECK(meanLuma != (uint32_t)pic->m_iOffsetCtu[ctuRsAddr], "luma DC offsets don't match");
        }
#if SHARP_LUMA_DELTA_QP

        // change adaptive QP based on mean CTU luma value (Sharp)
        if (useSharpLumaDQP)
        {
#if ENABLE_QPA_SUB_CTU
          pic->m_uEnerHpCtu[ctuRsAddr] = (double)meanLuma;   // for sub-CTU QPA
#endif
          iQPAdapt = Clip3(0, MAX_QP, iQPAdapt + lumaDQPOffset(meanLuma, bitDepth));
        }

#endif
        const uint32_t uRefScale = g_invQuantScales[0][iQPAdapt % 6] << ((iQPAdapt / 6) + bitDepth - 4);
        const CompArea subArea =
          clipArea(CompArea(COMP_Y, pic->chromaFormat,
                            Area((ctuRsAddr % pcv.widthInCtus) * pcv.maxCUWidth,
                                 (ctuRsAddr / pcv.widthInCtus) * pcv.maxCUHeight, pcv.maxCUWidth, pcv.maxCUHeight)),
                   pic->Y());
        const Pel      *pSrc       = pic->getOrigBuf(subArea).buf;
        const ptrdiff_t srcStride  = pic->getOrigBuf(subArea).stride;
        const SizeType  srcHeight  = pic->getOrigBuf(subArea).height;
        const SizeType  srcWidth   = pic->getOrigBuf(subArea).width;
        uint32_t        uAbsDCless = 0;

        // compute sum of absolute DC-less (high-pass) luma values
        for (SizeType h = 0; h < srcHeight; h++)
        {
          for (SizeType w = 0; w < srcWidth; w++)
          {
            uAbsDCless += (uint32_t)abs(pSrc[w] - (Pel)meanLuma);
          }
          pSrc += srcStride;
        }

        if (srcHeight >= 64 || srcWidth >= 64)   // normalization
        {
          const uint64_t blockSize = uint64_t(srcWidth * srcHeight);

          uAbsDCless = uint32_t((uint64_t(uAbsDCless) * 64 * 64 + (blockSize >> 1)) / blockSize);
        }

        if (uAbsDCless < 64 * 64)
        {
          uAbsDCless = 64 * 64;   // limit to 1
        }

        // reduce QP index if CTU would be fully quantized to zero
        if (uAbsDCless < uRefScale)
        {
          const int limit  = std::min(0, ((iQPIndex + 4) >> 3) - 6);
          const int redVal = std::max(limit, apprI3Log2((double)uAbsDCless / (double)uRefScale));

          iQPAdapt = std::max(0, iQPAdapt + redVal);
        }
      }

      pic->m_iOffsetCtu[ctuRsAddr] = (Pel)iQPAdapt;   // adapted QPs

#if ENABLE_QPA_SUB_CTU
      if (pcv.widthInCtus > 1 && pcSlice->getCuQpDeltaSubdiv() == 0)   // reduce local DQP rate peaks
#elif ENABLE_QPA_SUB_CTU
      if (pcv.widthInCtus > 1 && pcSlice->m_pps->getMaxCuDQPDepth() == 0)  // reduce local DQP rate peaks
#else
      if (pcv.widthInCtus > 1) // try to reduce local bitrate peaks via minimum smoothing of the adapted QPs
#endif
      {
        iQPAdapt = ctuRsAddr % pcv.widthInCtus;   // horizontal offset
        if (iQPAdapt == 0)
        {
          iQPAdapt = (ctuRsAddr > 1) ? pic->m_iOffsetCtu[ctuRsAddr - 2] : 0;
        }
        else   // iQPAdapt >= 1
        {
          iQPAdapt = (iQPAdapt > 1) ? std::min(pic->m_iOffsetCtu[ctuRsAddr - 2], pic->m_iOffsetCtu[ctuRsAddr])
                                    : pic->m_iOffsetCtu[ctuRsAddr];
        }
        if (ctuRsAddr > pcv.widthInCtus)
        {
          iQPAdapt = std::min(iQPAdapt, (int)pic->m_iOffsetCtu[ctuRsAddr - 1 - pcv.widthInCtus]);
        }
        if ((ctuRsAddr > 0) && (pic->m_iOffsetCtu[ctuRsAddr - 1] < (Pel)iQPAdapt))
        {
          pic->m_iOffsetCtu[ctuRsAddr - 1] = (Pel)iQPAdapt;
        }
        if ((ctuIdx == pcSlice->getNumCtuInSlice() - 1) &&
            (ctuRsAddr > pcv.widthInCtus))   // last CTU in the given slice
        {
          iQPAdapt = std::min(pic->m_iOffsetCtu[ctuRsAddr - 1], pic->m_iOffsetCtu[ctuRsAddr - pcv.widthInCtus]);
          if (pic->m_iOffsetCtu[ctuRsAddr] < (Pel)iQPAdapt)
          {
            pic->m_iOffsetCtu[ctuRsAddr] = (Pel)iQPAdapt;
          }
        }
      }
    }   // end iteration over all CTUs in current slice
  }

  return sliceQPModified;
}

#if ENABLE_QPA_SUB_CTU
static int applyQPAdaptationSubCtu(CodingStructure &cs, const UnitArea ctuArea, const uint32_t ctuAddr,
                                   const bool useSharpLumaDQP)
{
  const PreCalcValues &pcv          = *cs.pcv;
  const Picture       *pic          = cs.picture;
  const int            bitDepth     = cs.slice->m_sps->m_bitDepths[ChannelType::LUMA];   // overall image bit-depth
  const int            adaptedCtuQP = pic ? pic->m_iOffsetCtu[ctuAddr] : cs.slice->m_iSliceQpBase;

  if (!pic || cs.slice->getCuQpDeltaSubdiv() == 0)
  {
    return adaptedCtuQP;
  }

  for (unsigned addr = 0; addr < cs.picture->m_subCtuQP.size(); addr++)
  {
    cs.picture->m_subCtuQP[addr] = (int8_t)adaptedCtuQP;
  }
  if (cs.slice->m_iSliceQp < MAX_QP && pcv.widthInCtus > 1)
  {
#if SHARP_LUMA_DELTA_QP
    const int lumaCtuDQP = useSharpLumaDQP ? lumaDQPOffset((uint32_t)pic->m_uEnerHpCtu[ctuAddr], bitDepth) : 0;
#endif
    const unsigned mts     = std::min(cs.sps->getMaxTbSize(), pcv.maxCUWidth);
    const unsigned mtsLog2 = (unsigned)floorLog2(mts);
    const unsigned stride  = pcv.maxCUWidth >> mtsLog2;
    unsigned       numAct  = 0;   // number of block activities
    double         sumAct  = 0.0;   // sum of all block activities
    double         subAct[16];   // individual block activities
#if SHARP_LUMA_DELTA_QP
    uint32_t subMLV[16];   // individual mean luma values
#endif

    CHECK(mts * 4 < pcv.maxCUWidth || mts * 4 < pcv.maxCUHeight, "max. transform size is too small for given CTU size");

    for (unsigned h = 0; h < (pcv.maxCUHeight >> mtsLog2); h++)
    {
      for (unsigned w = 0; w < stride; w++)
      {
        const unsigned addr = w + h * stride;
        const PosType  x    = ctuArea.lx() + w * mts;
        const PosType  y    = ctuArea.ly() + h * mts;
        const CompArea fltArea =
          clipArea(CompArea(COMP_Y, pic->chromaFormat,
                            Area(x > 0 ? x - 1 : 0, y > 0 ? y - 1 : 0, mts + (x > 0 ? 2 : 1), mts + (y > 0 ? 2 : 1))),
                   pic->Y());
        const CPelBuf picOrig = pic->getOrigBuf(fltArea);

        if (x >= pic->lwidth() || y >= pic->lheight())
        {
          continue;
        }
        filterAndCalculateAverageEnergies(picOrig.buf, picOrig.stride, subAct[addr], picOrig.height, picOrig.width,
                                          bitDepth);
        numAct++;
        sumAct += subAct[addr];
#if SHARP_LUMA_DELTA_QP

        if (useSharpLumaDQP)
        {
          const CompArea subArea = clipArea(CompArea(COMP_Y, pic->chromaFormat, Area(x, y, mts, mts)), pic->Y());

          subMLV[addr] = pic->getOrigBuf(subArea).computeAvg();
        }
#endif
      }
    }
    if (sumAct <= 0.0)
    {
      return adaptedCtuQP;
    }

    sumAct = double(numAct) / sumAct;   // 1.0 / (average CTU activity)

    for (unsigned h = 0; h < (pcv.maxCUHeight >> mtsLog2); h++)
    {
      for (unsigned w = 0; w < stride; w++)
      {
        const unsigned addr = w + h * stride;

        if (ctuArea.lx() + w * mts >= pic->lwidth() || ctuArea.ly() + h * mts >= pic->lheight())
        {
          continue;
        }
        cs.picture->m_subCtuQP[addr] = (int8_t)Clip3(0, MAX_QP, adaptedCtuQP + apprI3Log2(subAct[addr] * sumAct));
#if SHARP_LUMA_DELTA_QP

        // change adapted QP based on mean sub-CTU luma value (Sharp)
        if (useSharpLumaDQP)
        {
          cs.picture->m_subCtuQP[addr] = (int8_t)Clip3(
            0, MAX_QP, (int)cs.picture->m_subCtuQP[addr] - lumaCtuDQP + lumaDQPOffset(subMLV[addr], bitDepth));
        }
#endif
      }
    }
  }

  return adaptedCtuQP;
}
#endif   // ENABLE_QPA_SUB_CTU
#endif   // ENABLE_QPA

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

//! set adaptive search range based on poc difference
void EncSlice::setSearchRange(Slice *pcSlice)
{
  int currPoc = pcSlice->m_poc;
  int iRefPOC;
  int iGOPSize    = m_encCfg->m_gopSize;
  int offset      = (iGOPSize >> 1);
  int iMaxSR      = m_encCfg->m_searchRange;
  int iNumPredDir = pcSlice->isInterP() ? 1 : 2;

  for (int dir = 0; dir < iNumPredDir; dir++)
  {
    RefPicList e = (dir ? RPL1 : RPL0);
    for (int refIdx = 0; refIdx < pcSlice->m_numRefIdx[e]; refIdx++)
    {
      iRefPOC            = pcSlice->getRefPic(e, refIdx)->m_poc;
      int newSearchRange = Clip3(m_encCfg->m_minSearchWindow, iMaxSR,
                                 (iMaxSR * ADAPT_SR_SCALE * abs(currPoc - iRefPOC) + offset) / iGOPSize);
      m_pcInterSearch->setAdaptiveSearchRange(dir, refIdx, newSearchRange);
    }
  }
}

void EncSlice::setLosslessSlice(Picture *pic, bool islossless)
{
  Slice *slice        = pic->m_slices[getSliceSegmentIdx()];
  slice->m_isLossless = islossless;

  if (m_encCfg->m_costMode == COST_LOSSLESS_CODING)
  {
    if (islossless)
    {
      int losslessQp =
        LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP - ((slice->m_sps->m_bitDepths[ChannelType::LUMA] - 8) * 6);
      slice->m_iSliceQp = losslessQp;   // update the slice/base QPs

      slice->m_tsResidualCodingDisabledFlag = m_encCfg->m_TSRCdisableLL ? true : false;
    }
    else
    {
      slice->m_iSliceQp                     = pic->m_lossyQP;
      slice->m_tsResidualCodingDisabledFlag = false;
    }
  }
}

/**
 Multi-loop slice encoding for different slice QP

 \param pic    picture class
 */
void EncSlice::precompressSlice(Picture *pic)
{
  // if deltaQP RD is not used, simply return
  if (m_encCfg->m_uiDeltaQpRD == 0)
  {
    return;
  }

  if (m_encCfg->m_RCEnableRateControl)
  {
    THROW("\nMultiple QP optimization is not allowed when rate control is enabled.");
  }

  Slice *pcSlice = pic->m_slices[getSliceSegmentIdx()];

  double   dPicRdCostBest = MAX_DOUBLE;
  uint32_t uiQpIdxBest    = 0;

  double dFrameLambda;
  int    SHIFT_QP = 12 +
    6 *
      (pcSlice->m_sps->m_bitDepths[ChannelType::LUMA] - 8 -
       DISTORTION_PRECISION_ADJUSTMENT(pcSlice->m_sps->m_bitDepths[ChannelType::LUMA]));

  // set frame lambda
  if (m_encCfg->m_gopSize > 1)
  {
    dFrameLambda = 0.68 * pow(2, (m_viRdPicQp[0] - SHIFT_QP) / 3.0) * (pcSlice->isInterB() ? 2 : 1);
  }
  else
  {
    dFrameLambda = 0.68 * pow(2, (m_viRdPicQp[0] - SHIFT_QP) / 3.0);
  }

  // for each QP candidate
  for (uint32_t uiQpIdx = 0; uiQpIdx < 2 * m_encCfg->m_uiDeltaQpRD + 1; uiQpIdx++)
  {
    pcSlice->m_iSliceQp = m_viRdPicQp[uiQpIdx];
    setUpLambda(pcSlice, m_vdRdPicLambda[uiQpIdx], m_viRdPicQp[uiQpIdx]);

    // try compress
    compressSlice(pic, true, m_encCfg->m_bFastDeltaQP);

    uint64_t uiPicDist  = m_uiPicDist;   // Distortion, as calculated by compressSlice.
    // NOTE: This distortion is the chroma-weighted SSE distortion for the slice.
    //       Previously a standard SSE distortion was calculated (for the entire frame).
    //       Which is correct?
    // TODO: Update loop filter, SAO and distortion calculation to work on one slice only.
    // uiPicDist = m_pcGOPEncoder->preLoopFilterPicAndCalcDist( pic );
    // compute RD cost and choose the best
    double   dPicRdCost = double(uiPicDist) + dFrameLambda * double(m_uiPicTotalBits);

    if (dPicRdCost < dPicRdCostBest)
    {
      uiQpIdxBest    = uiQpIdx;
      dPicRdCostBest = dPicRdCost;
    }
  }

  // set best values
  pcSlice->m_iSliceQp = m_viRdPicQp[uiQpIdxBest];
  setUpLambda(pcSlice, m_vdRdPicLambda[uiQpIdxBest], m_viRdPicQp[uiQpIdxBest]);
}

void EncSlice::calCostSliceI(Picture *pic)   // TODO: this only analyses the first slice segment. What about the others?
{
  double               iSumHadSlice = 0;
  Slice *const         pcSlice      = pic->m_slices[getSliceSegmentIdx()];
  const PreCalcValues &pcv          = *pic->m_cs->pcv;
  const SPS           &sps          = *(pcSlice->m_sps);
  const int            shift        = sps.m_bitDepths[ChannelType::LUMA] - 8;
  const int            offset       = (shift > 0) ? (1 << (shift - 1)) : 0;

  for (uint32_t ctuIdx = 0; ctuIdx < pcSlice->getNumCtuInSlice(); ctuIdx++)
  {
    uint32_t ctuRsAddr = pcSlice->getCtuAddrInSlice(ctuIdx);
    Position pos((ctuRsAddr % pcv.widthInCtus) * pcv.maxCUWidth, (ctuRsAddr / pcv.widthInCtus) * pcv.maxCUHeight);

    const int      height = std::min(pcv.maxCUHeight, pcv.lumaHeight - pos.y);
    const int      width  = std::min(pcv.maxCUWidth, pcv.lumaWidth - pos.x);
    const CompArea blk(COMP_Y, pcv.chrFormat, pos, Size(width, height));
    int            iSumHad = m_pcCuEncoder->updateCtuDataISlice(pic->getOrigBuf(blk));

    (m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr)).m_costIntra = (iSumHad + offset) >> shift;
    iSumHadSlice += (m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr)).m_costIntra;
  }
  m_pcRateCtrl->getRCPic()->setTotalIntraCost(iSumHadSlice);
}

void EncSlice::calCostPictureI(Picture *picture)
{
  double               sumHadPicture = 0;
  Slice *const         slice         = picture->m_slices[getSliceSegmentIdx()];
  const PreCalcValues &pcv           = *picture->m_cs->pcv;
  const SPS           &sps           = *(slice->m_sps);
  const int            shift         = sps.m_bitDepths[ChannelType::LUMA] - 8;
  const int            offset        = (shift > 0) ? (1 << (shift - 1)) : 0;

  for (uint32_t ctuIdx = 0; ctuIdx < picture->m_ctuNums; ctuIdx++)
  {
    Position pos((ctuIdx % pcv.widthInCtus) * pcv.maxCUWidth, (ctuIdx / pcv.widthInCtus) * pcv.maxCUHeight);

    const int      height = std::min(pcv.maxCUHeight, pcv.lumaHeight - pos.y);
    const int      width  = std::min(pcv.maxCUWidth, pcv.lumaWidth - pos.x);
    const CompArea blk(COMP_Y, pcv.chrFormat, pos, Size(width, height));
    int            sumHad = m_pcCuEncoder->updateCtuDataISlice(picture->getOrigBuf(blk));

    (m_pcRateCtrl->getRCPic()->getLCU(ctuIdx)).m_costIntra = (sumHad + offset) >> shift;
    sumHadPicture += (m_pcRateCtrl->getRCPic()->getLCU(ctuIdx)).m_costIntra;
  }
  m_pcRateCtrl->getRCPic()->setTotalIntraCost(sumHadPicture);
}

/** \param pic   picture class
 */
void EncSlice::compressSlice(Picture *pic, const bool bCompressEntireSlice, const bool bFastDeltaQP)
{
  PROFILER_SCOPE(1, g_timeProfiler, P_COMPRESS_SLICE);
  // if bCompressEntireSlice is true, then the entire slice (not slice segment) is compressed,
  //   effectively disabling the slice-segment-mode.

  Slice *const pcSlice = pic->m_slices[getSliceSegmentIdx()];

  if (pcSlice->m_sps->m_spsRangeExtension.m_rrcRiceExtensionEnableFlag)
  {
    int bitDepth  = pcSlice->m_sps->m_bitDepths[ChannelType::LUMA];
    int baseLevel = (bitDepth > 12) ? (pcSlice->isIntra() ? 5 : 2 * 5) : (pcSlice->isIntra() ? 2 * 5 : 3 * 5);
    pcSlice->m_riceBaseLevelValue = baseLevel;
  }
  else
  {
    pcSlice->m_riceBaseLevelValue = 4;
  }

  // initialize cost values - these are used by precompressSlice (they should be parameters).
  m_uiPicTotalBits = 0;
  m_uiPicDist      = 0;

  pcSlice->m_iSliceQpBase = pcSlice->m_iSliceQp;
  m_CABACEstimator->m_CABACDataStore->updateBufferState(pcSlice);

  m_CABACEstimator->initCtxModels(*pcSlice);

  m_pcCuEncoder->getModeCtrl()->fastDeltaQp = bFastDeltaQP;

  //------------------------------------------------------------------------------
  //  Weighted Prediction parameters estimation.
  //------------------------------------------------------------------------------
  // calculate AC/DC values for current picture
  if (pcSlice->m_pps->m_useWP || pcSlice->m_pps->m_useBiWP)
  {
    xCalcACDCParamSlice(pcSlice);
  }

  if (pcSlice->m_sps->m_ibcFlag)
  {
    auto sliceType = pcSlice->m_eSliceType;

    if (sliceType == I_SLICE)
    {
      pcSlice->m_ibcFlag = pcSlice->m_sps->m_ibcFlag;
    }
    else
    {
      pcSlice->m_ibcFlag = pcSlice->m_sps->m_ibcFlagInterSlice;
    }
  }

  const bool bWpExplicit = (pcSlice->m_eSliceType == P_SLICE && pcSlice->m_pps->m_useWP) ||
    (pcSlice->m_eSliceType == B_SLICE && pcSlice->m_pps->m_useBiWP);

  if (bWpExplicit)
  {

    xEstimateWPParamSlice(pcSlice, m_encCfg->m_weightedPredictionMethod);
    pcSlice->initWpScaling(pcSlice->m_sps);

    // check WP on/off
    xCheckWPEnable(pcSlice);
  }

  xSetUseLICOnPicLevel(pcSlice, m_encCfg->m_fastPicLevelLIC);

  pic->m_prevQP.fill(pcSlice->m_iSliceQp);

  CHECK(pic->m_prevQP[ChannelType::LUMA] == std::numeric_limits<int>::max(), "Invalid previous QP");

  CodingStructure &cs = *pic->m_cs;
  cs.slice            = pcSlice;
  cs.pcv              = pcSlice->m_pps->pcv;
  cs.fracBits         = 0;

  if (pcSlice->getFirstCtuRsAddrInSlice() == 0 &&
      (pcSlice->m_poc != m_encCfg->m_switchPOC || -1 == m_encCfg->m_debugCTU))
  {
    cs.initStructData(pcSlice->m_iSliceQp);
  }

#if ENABLE_QPA
  if (m_encCfg->m_bUsePerceptQPA && !m_encCfg->m_RCEnableRateControl)
  {
    if (applyQPAdaptation(pic, pcSlice, *cs.pcv, m_encCfg->m_lumaLevelToDeltaQPMapping.mode == LUMALVL_TO_DQP_NUM_MODES,
                          (m_encCfg->m_iQP >= 38) ||
                            (m_encCfg->m_sourceWidth <= 512 && m_encCfg->m_sourceHeight <= 320),
                          m_adaptedLumaQP))
    {
      m_CABACEstimator->m_CABACDataStore->updateBufferState(pcSlice);
      m_CABACEstimator->initCtxModels(*pcSlice);
      pic->m_prevQP.fill(pcSlice->m_iSliceQp);
      if (pcSlice->getFirstCtuRsAddrInSlice() == 0)
      {
        cs.currQP.fill(pcSlice->m_iSliceQp);
      }
    }
  }
#endif   // ENABLE_QPA

  bool checkPLTRatio = m_encCfg->m_intraPeriod != 1 && pcSlice->isIRAP();
  if (checkPLTRatio)
  {
    // th interesting plt can't be disabled
    m_pcCuEncoder->getModeCtrl()->doPlt = true;
  }
  else
  {
    bool doPlt                          = m_pcLib->getPltEnc();
    m_pcCuEncoder->getModeCtrl()->doPlt = doPlt;
  }

#if K0149_BLOCK_STATISTICS
  const SPS *sps = pcSlice->m_sps;
  CHECK(sps == 0, "No SPS present");
  writeBlockStatisticsHeader(sps);
#endif
  if (pcSlice->m_pendingRasInit)
  {
    // TODO: find a proper fix to enable affine search with RPR (missing gradient data when ref is scaled)
    m_pcInterSearch->resetGradBuffers();
  }
  m_pcInterSearch->resetAffineMVList();
  m_pcInterSearch->resetUniMvList();
  m_pcInterSearch->resetReusedUniMvs();
  encodeCtus(pic, bCompressEntireSlice, bFastDeltaQP, m_pcLib);
  if (checkPLTRatio)
  {
    m_pcLib->checkPltStats(pic);
  }
}

void EncSlice::checkDisFracMmvd(Picture *pic, uint32_t startCtuTsAddr, uint32_t boundingCtuTsAddr)
{
  CodingStructure     &cs            = *pic->m_cs;
  Slice               *pcSlice       = cs.slice;
  const PreCalcValues &pcv           = *cs.pcv;
  const uint32_t       widthInCtus   = pcv.widthInCtus;
  const uint32_t       hashThreshold = 20;
  uint32_t             totalCtu      = 0;
  uint32_t             hashRatio     = 0;

  if (!pcSlice->m_sps->m_fpelMmvdEnabledFlag)
  {
    return;
  }

  for (uint32_t ctuIdx = 0; ctuIdx < pcSlice->getNumCtuInSlice(); ctuIdx++)
  {
    const uint32_t ctuRsAddr     = pcSlice->getCtuAddrInSlice(ctuIdx);
    const uint32_t ctuXPosInCtus = ctuRsAddr % widthInCtus;
    const uint32_t ctuYPosInCtus = ctuRsAddr / widthInCtus;

    const Position pos(ctuXPosInCtus * pcv.maxCUWidth, ctuYPosInCtus * pcv.maxCUHeight);
    const UnitArea ctuArea(cs.area.chromaFormat, Area(pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight));

    hashRatio += m_pcCuEncoder->getIbcHashMap().getHashHitRatio(ctuArea.Y());
    totalCtu++;
  }

  if (hashRatio > totalCtu * hashThreshold)
  {
    pic->m_cs->picHeader->m_disFracMMVD = true;
  }
  if (!pic->m_cs->picHeader->m_disFracMMVD)
  {
    bool useIntegerMVD                  = (pic->lwidth() * pic->lheight() > 1920 * 1080);
    pic->m_cs->picHeader->m_disFracMMVD = useIntegerMVD;
  }
}

void EncSlice::setJointCbCrModes(CodingStructure &cs, const Position topLeftLuma, const Size sizeLuma)
{
  bool sgnFlag = true;

  if (isChromaEnabled(cs.picture->chromaFormat))
  {
    const CompArea  cbArea  = CompArea(COMP_Cb, cs.picture->chromaFormat, Area(topLeftLuma, sizeLuma), true);
    const CompArea  crArea  = CompArea(COMP_Cr, cs.picture->chromaFormat, Area(topLeftLuma, sizeLuma), true);
    const CPelBuf   orgCb   = cs.picture->getOrigBuf(cbArea);
    const CPelBuf   orgCr   = cs.picture->getOrigBuf(crArea);
    const int       x0      = (cbArea.x > 0 ? 0 : 1);
    const int       y0      = (cbArea.y > 0 ? 0 : 1);
    const int       x1      = (cbArea.x + cbArea.width < cs.picture->Cb().width ? cbArea.width : cbArea.width - 1);
    const int       y1      = (cbArea.y + cbArea.height < cs.picture->Cb().height ? cbArea.height : cbArea.height - 1);
    const ptrdiff_t cbs     = orgCb.stride;
    const ptrdiff_t crs     = orgCr.stride;
    const Pel      *pCb     = orgCb.buf + y0 * cbs;
    const Pel      *pCr     = orgCr.buf + y0 * crs;
    int64_t         sumCbCr = 0;

    // determine inter-chroma transform sign from correlation between high-pass filtered (i.e., zero-mean) Cb and Cr
    // planes
    for (int y = y0; y < y1; y++, pCb += cbs, pCr += crs)
    {
      for (int x = x0; x < x1; x++)
      {
        int cb = (12 * (int)pCb[x] - 2 * ((int)pCb[x - 1] + (int)pCb[x + 1] + (int)pCb[x - cbs] + (int)pCb[x + cbs]) -
                  ((int)pCb[x - 1 - cbs] + (int)pCb[x + 1 - cbs] + (int)pCb[x - 1 + cbs] + (int)pCb[x + 1 + cbs]));
        int cr = (12 * (int)pCr[x] - 2 * ((int)pCr[x - 1] + (int)pCr[x + 1] + (int)pCr[x - crs] + (int)pCr[x + crs]) -
                  ((int)pCr[x - 1 - crs] + (int)pCr[x + 1 - crs] + (int)pCr[x - 1 + crs] + (int)pCr[x + 1 + crs]));
        sumCbCr += cb * cr;
      }
    }

    sgnFlag = (sumCbCr < 0);
  }

  cs.picHeader->m_jointCbCrSignFlag = sgnFlag;
}

void EncSlice::encodeCtus(Picture *pic, const bool bCompressEntireSlice, const bool bFastDeltaQP, EncLib *pEncLib)
{
  CodingStructure     &cs          = *pic->m_cs;
  Slice               *pcSlice     = cs.slice;
  const PreCalcValues &pcv         = *cs.pcv;
  const uint32_t       widthInCtus = pcv.widthInCtus;
#if ENABLE_QPA
  const int iQPIndex = pcSlice->m_iSliceQpBase;
#endif
  pic->calcLumaClpParams();

  const EncCfg *encCfg       = &pEncLib->m_encCfg;
  CABACWriter  *pCABACWriter = pEncLib->getCABACEncoder()->getCABACEstimator(pcSlice->m_sps);
  TrQuant      *pTrQuant     = pEncLib->getTrQuant();
  RdCost       *pRdCost      = pEncLib->getRdCost();
  RateCtrl     *pRateCtrl    = pEncLib->getRateCtrl();
  pRdCost->setLosslessRDCost(pcSlice->m_isLossless);
#if RDOQ_CHROMA_LAMBDA
  pTrQuant->setLambdas(pcSlice->getLambdas());
#else
  pTrQuant->setLambda(pcSlice->getLambdas()[0]);
#endif
  pRdCost->setLambda(pcSlice->getLambdas()[0], pcSlice->m_sps->m_bitDepths);
#if WCG_EXT && ER_CHROMA_QP_WCG_PPS && ENABLE_QPA
  if (!encCfg->m_wcgChromaQpControl.enabled && encCfg->m_bUsePerceptQPA && !encCfg->m_RCEnableRateControl)
  {
    pRdCost->saveUnadjustedLambda();
  }
#endif

  EnumArray<int, ChannelType> prevQP;
  EnumArray<int, ChannelType> currQP;

  prevQP.fill(pcSlice->m_iSliceQp);
  currQP.fill(pcSlice->m_iSliceQp);

  int hashBlkHitPerc = -1;

  if (pcSlice->m_sps->m_fpelMmvdEnabledFlag || (pcSlice->m_ibcFlag && m_encCfg->m_ibcHashSearch))
  {
    m_pcCuEncoder->getIbcHashMap().rebuildPicHashMap(cs.picture->getTrueOrigBuf());
    if (!m_encCfg->m_isLowDelay)
    {
      hashBlkHitPerc               = m_pcCuEncoder->getIbcHashMap().calHashBlkMatchPerc(cs.area.Y());
      cs.slice->m_disableSATDForRd = hashBlkHitPerc > 59;
    }
    if ((pcSlice->m_sps->m_spsRangeExtension.m_tsrcRicePresentFlag) &&
        (m_pcGOPEncoder->getPreQP() != pcSlice->m_iSliceQp) && (pic->m_cs->pps->m_numSlicesInPic == 1) &&
        (pcSlice->m_tsrcIndex > 0) && (pcSlice->m_sps->m_bitDepths[ChannelType::LUMA] <= 12))
    {
      uint32_t totalCtu  = 0;
      uint32_t hashRatio = 0;
      for (uint32_t ctuIdx = 0; ctuIdx < pcSlice->getNumCtuInSlice(); ctuIdx++)
      {
        const uint32_t ctuRsAddr     = pcSlice->getCtuAddrInSlice(ctuIdx);
        const uint32_t ctuXPosInCtus = ctuRsAddr % widthInCtus;
        const uint32_t ctuYPosInCtus = ctuRsAddr / widthInCtus;
        const Position pos(ctuXPosInCtus * pcv.maxCUWidth, ctuYPosInCtus * pcv.maxCUHeight);
        const UnitArea ctuArea(cs.area.chromaFormat, Area(pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight));

        hashRatio += m_pcCuEncoder->getIbcHashMap().calHashBlkMatchPerc(cs.area.Y());
        totalCtu++;
      }
      if (totalCtu > 0)
      {
        if ((hashRatio < 4200) || (hashRatio < (41 * totalCtu)))
        {
          pcSlice->m_tsrcIndex = 0;
        }
      }
    }
  }

  if (encCfg->m_switchPOC != pic->m_poc || -1 == encCfg->m_debugCTU)
  {
    cs.ccpLut.lutCCP.resize(0);
    cs.eipLut.lutEip.resize(0);
  }

  // for every CTU in the slice
  for (uint32_t ctuIdx = 0; ctuIdx < pcSlice->getNumCtuInSlice(); ctuIdx++)
  {
    const int32_t ctuRsAddr = pcSlice->getCtuAddrInSlice(ctuIdx);

    // update CABAC state
    const uint32_t ctuXPosInCtus = ctuRsAddr % widthInCtus;
    const uint32_t ctuYPosInCtus = ctuRsAddr / widthInCtus;

    const Position pos(ctuXPosInCtus * pcv.maxCUWidth, ctuYPosInCtus * pcv.maxCUHeight);
    const UnitArea ctuArea(cs.area.chromaFormat, Area(pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight));
    DTRACE_UPDATE(g_trace_ctx, std::make_pair("ctu", ctuRsAddr));
    if (encCfg->m_switchPOC != pic->m_poc || -1 == encCfg->m_debugCTU || ctuRsAddr > encCfg->m_debugCTU)
    {
      if ((cs.slice->m_eSliceType != I_SLICE || cs.slice->m_ibcFlag) && cs.pps->ctuIsTileColBd(ctuXPosInCtus))
      {
        cs.motionLut.lut.resize(0);
        cs.motionLut.lutIbc.resize(0);
        for (int i = 0; i < MAX_NUM_AFFHMVP_ENTRIES; i++)
        {
          cs.motionLut.lutAff[i].resize(0);
        }
        cs.motionLut.lutAffInherit.resize(0);
      }
      if (cs.pps->ctuIsTileColBd(ctuXPosInCtus))
      {
        cs.ccpLut.lutCCP.resize(0);
        cs.eipLut.lutEip.resize(0);
      }
    }

    if (-1 != encCfg->m_debugCTU)
    {
      m_pcInterSearch->resetAffineMVList();
      m_pcInterSearch->resetUniMvList();
      m_pcInterSearch->resetReusedUniMvs();
    }

    const SubPic &curSubPic = pcSlice->m_pps->getSubPicFromPos(pos);
    // padding/restore at slice level
    if (pcSlice->m_pps->m_numSubPics >= 2 && curSubPic.m_treatedAsPicFlag && ctuIdx == 0)
    {
      int subPicX      = (int)curSubPic.m_subPicLeft;
      int subPicY      = (int)curSubPic.m_subPicTop;
      int subPicWidth  = (int)curSubPic.m_subPicWidthInLumaSample;
      int subPicHeight = (int)curSubPic.m_subPicHeightInLumaSample;

      for (int rlist = RPL0; rlist < NUM_RPL01; rlist++)
      {
        int n = pcSlice->m_numRefIdx[(RefPicList)rlist];
        for (int idx = 0; idx < n; idx++)
        {
          Picture *refPic = pcSlice->getRefPic((RefPicList)rlist, idx);

          if (!refPic->m_isSubPicBorderSaved && refPic->m_subPictures.size() > 1)
          {
            refPic->saveSubPicBorder(refPic->m_poc, subPicX, subPicY, subPicWidth, subPicHeight);
            refPic->extendSubPicBorder(refPic->m_poc, subPicX, subPicY, subPicWidth, subPicHeight);
            refPic->m_isSubPicBorderSaved = true;
          }
        }
      }
    }
    if (cs.pps->ctuIsTileColBd(ctuXPosInCtus) && cs.pps->ctuIsTileRowBd(ctuYPosInCtus))
    {
      pCABACWriter->initCtxModels(*pcSlice);
      cs.resetPrevPLT(cs.prevPLT);
      prevQP.fill(pcSlice->m_iSliceQp);
    }
    else if (cs.pps->ctuIsTileColBd(ctuXPosInCtus) && encCfg->m_entropyCodingSyncEnabledFlag)
    {
      // reset and then update contexts to the state at the end of the top CTU (if within current slice and tile).
      pCABACWriter->initCtxModels(*pcSlice);
      cs.resetPrevPLT(cs.prevPLT);
      if (cs.getCURestricted(pos.offset(0, -1), pos, pcSlice->m_independentSliceIdx, cs.pps->getTileIdx(pos),
                             ChannelType::LUMA))
      {
        // Top is available, we use it.
        pCABACWriter->getCtx() = pEncLib->m_entropyCodingSyncContextState;
        pCABACWriter->getCtx().riceStatReset(pcSlice->m_sps->m_bitDepths[ChannelType::LUMA],
                                             pcSlice->m_sps->m_spsRangeExtension.m_persistentRiceAdaptationEnabledFlag);
        cs.setPrevPLT(pEncLib->m_palettePredictorSyncState);
      }
      prevQP.fill(pcSlice->m_iSliceQp);
    }

#if RDOQ_CHROMA_LAMBDA && ENABLE_QPA && !ENABLE_QPA_SUB_CTU
    double oldLambdaArray[MAX_NUM_COMP] = { 0.0 };
#endif
    const double oldLambda = pRdCost->getLambda();
    if (encCfg->m_RCEnableRateControl)
    {
      int    estQP     = pcSlice->m_iSliceQp;
      double estLambda = -1.0;
      double bpp       = -1.0;

      if ((pic->m_slices[0]->isIRAP() && encCfg->m_RCForceIntraQP) || !encCfg->m_RCLCULevelRC)
      {
        estQP = pcSlice->m_iSliceQp;
      }
      else
      {
        bpp = pRateCtrl->getRCPic()->getLCUTargetBpp(pcSlice->isIRAP());
        if (pic->m_slices[0]->isIntra())
        {
          estLambda = pRateCtrl->getRCPic()->getLCUEstLambdaAndQP(bpp, pcSlice->m_iSliceQp, &estQP);
        }
        else
        {
          estLambda = pRateCtrl->getRCPic()->getLCUEstLambda(bpp);
          estQP     = pRateCtrl->getRCPic()->getLCUEstQP(estLambda, pcSlice->m_iSliceQp);
        }

        estQP = Clip3(-pcSlice->m_sps->m_qpBDOffset[ChannelType::LUMA], MAX_QP, estQP);

        pRdCost->setLambda(estLambda, pcSlice->m_sps->m_bitDepths);
#if WCG_EXT
        pRdCost->saveUnadjustedLambda();
#endif
        for (uint32_t compIdx = 1; compIdx < MAX_NUM_COMP; compIdx++)
        {
          const CompID compID         = CompID(compIdx);
          int          chromaQPOffset = pcSlice->m_pps->getQpOffset(compID) + pcSlice->getSliceChromaQpDelta(compID);
          int          qpc            = pcSlice->m_sps->getMappedChromaQpValue(compID, estQP) + chromaQPOffset;
          double       tmpWeight =
            pow(2.0, (estQP - qpc) / 3.0);   // takes into account of the chroma qp mapping and chroma qp Offset
          if (m_encCfg->m_DepQuantEnabledIdc)
          {
            tmpWeight *= (m_encCfg->m_gopSize >= 8
                            ? pow(2.0, 0.1 / 3.0)
                            : pow(2.0, 0.2 / 3.0));   // increase chroma weight for dependent quantization (in order to
                                                      // reduce bit rate shift from chroma to luma)
          }
          m_pcRdCost->setDistortionWeight(compID, tmpWeight);
        }
#if RDOQ_CHROMA_LAMBDA
        const double lambdaArray[MAX_NUM_COMP] = { estLambda / m_pcRdCost->getDistortionWeight(COMP_Y),
                                                   estLambda / m_pcRdCost->getDistortionWeight(COMP_Cb),
                                                   estLambda / m_pcRdCost->getDistortionWeight(COMP_Cr) };
        pTrQuant->setLambdas(lambdaArray);
#else
        pTrQuant->setLambda(estLambda);
#endif
      }

      pRateCtrl->setRCQP(estQP);
    }
#if ENABLE_QPA
    else if (encCfg->m_bUsePerceptQPA && pcSlice->m_pps->m_useDQP)
    {
#if ENABLE_QPA_SUB_CTU
      const int adaptedQP = applyQPAdaptationSubCtu(
        cs, ctuArea, ctuRsAddr, m_encCfg->m_lumaLevelToDeltaQPMapping.mode == LUMALVL_TO_DQP_NUM_MODES);
#else
      const int adaptedQP = pic->m_iOffsetCtu[ctuRsAddr];
#endif
      const double newLambda       = pcSlice->getLambdas()[0] * pow(2.0, double(adaptedQP - iQPIndex) / 3.0);
      pic->m_uEnerHpCtu[ctuRsAddr] = newLambda;   // for ALF and SAO
#if !ENABLE_QPA_SUB_CTU
#if RDOQ_CHROMA_LAMBDA
      pTrQuant->getLambdas(oldLambdaArray);   // save the old lambdas
      const double lambdaArray[MAX_NUM_COMP] = { newLambda / m_pcRdCost->getDistortionWeight(COMP_Y),
                                                 newLambda / m_pcRdCost->getDistortionWeight(COMP_Cb),
                                                 newLambda / m_pcRdCost->getDistortionWeight(COMP_Cr) };
      pTrQuant->setLambdas(lambdaArray);
#else
      pTrQuant->setLambda(newLambda);
#endif
      pRdCost->setLambda(newLambda, pcSlice->m_sps->m_bitDepths);
#endif
      currQP.fill(adaptedQP);
    }
#endif

    bool updateBcwCodingOrder = cs.slice->m_eSliceType == B_SLICE && ctuIdx == 0;
    if (updateBcwCodingOrder)
    {
      resetBcwCodingOrder(false, cs);
      m_pcInterSearch->initWeightIdxBits();
    }
    {
      m_pcCuEncoder->setDecCuReshaperInEncCU(m_pcLib->getReshaper(), pcSlice->m_sps->m_chromaFormatIdc);
    }
    if (!cs.slice->isIntra() && encCfg->m_seiCfg.m_MCTSEncConstraint)
    {
      pic->m_mctsInfo.init(&cs, ctuRsAddr);
    }

    if (ctuYPosInCtus)   // update the CABAC states based on CTU above
    {
      pCABACWriter->updateCtxs(getBinVector(ctuXPosInCtus));
    }

    pCABACWriter->setBinBuffer(nullptr);   // no data collection during the compress pass

    if (encCfg->m_switchPOC != pic->m_poc || ctuRsAddr >= encCfg->m_debugCTU)
    {
      m_pcCuEncoder->compressCtu(cs, ctuArea, ctuRsAddr, prevQP, currQP);
    }

    pCABACWriter->setBinBuffer(
      getBinVector(ctuXPosInCtus));   // clear the bin counters and prepare for collecting new data for this CTU
#if K0149_BLOCK_STATISTICS
    getAndStoreBlockStatistics(cs, ctuArea);
#endif

    pCABACWriter->resetBits();
    pCABACWriter->coding_tree_unit(cs, ctuArea, prevQP, ctuRsAddr, true, true, true
#if ENABLE_NNLF
                                   ,
                                   true
#endif
    );
    //    const int numberOfWrittenBits = int( pCABACWriter->getEstFracBits() >> SCALE_BITS );
    pCABACWriter->setBinBuffer(nullptr);   // done with the data collection for this CTU

    // Store probabilities of first CTU in line into buffer - used only if wavefront-parallel-processing is enabled.
    if (cs.pps->ctuIsTileColBd(ctuXPosInCtus) && encCfg->m_entropyCodingSyncEnabledFlag)
    {
      pEncLib->m_entropyCodingSyncContextState = pCABACWriter->getCtx();
      cs.storePrevPLT(pEncLib->m_palettePredictorSyncState);
    }

    int actualBits = int(cs.fracBits >> SCALE_BITS);
    actualBits -= (int)m_uiPicTotalBits;
    if (encCfg->m_RCEnableRateControl)
    {
      int    actualQP                = g_RCInvalidQPValue;
      double actualLambda            = pRdCost->getLambda();
      int    numberOfEffectivePixels = 0;

      int numberOfSkipPixel = 0;
      for (auto &cu: cs.traverseCUs(ctuArea, ChannelType::LUMA))
      {
        numberOfSkipPixel += cu.skip * cu.lumaSize().area();
      }

      for (auto &cu: cs.traverseCUs(ctuArea, ChannelType::LUMA))
      {
        if (!cu.skip || cu.rootCbf)
        {
          numberOfEffectivePixels += cu.lumaSize().area();
          break;
        }
      }
      double      skipRatio = (double)numberOfSkipPixel / ctuArea.lumaSize().area();
      CodingUnit *cu        = cs.getCU(ctuArea.lumaPos(), ChannelType::LUMA);

      if (numberOfEffectivePixels == 0)
      {
        actualQP = g_RCInvalidQPValue;
      }
      else
      {
        actualQP = cu->qp;
      }
      pRdCost->setLambda(oldLambda, pcSlice->m_sps->m_bitDepths);
      int estQP = pcSlice->m_iSliceQp;
      for (uint32_t compIdx = 1; compIdx < MAX_NUM_COMP; compIdx++)
      {
        const CompID compID         = CompID(compIdx);
        int          chromaQPOffset = pcSlice->m_pps->getQpOffset(compID) + pcSlice->getSliceChromaQpDelta(compID);
        int          qpc            = pcSlice->m_sps->getMappedChromaQpValue(compID, estQP) + chromaQPOffset;
        double       tmpWeight =
          pow(2.0, (estQP - qpc) / 3.0);   // takes into account of the chroma qp mapping and chroma qp Offset
        if (m_encCfg->m_DepQuantEnabledIdc)
        {
          tmpWeight *=
            (m_encCfg->m_gopSize >= 8 ? pow(2.0, 0.1 / 3.0)
                                      : pow(2.0, 0.2 / 3.0));   // increase chroma weight for dependent quantization (in
                                                                // order to reduce bit rate shift from chroma to luma)
        }
        m_pcRdCost->setDistortionWeight(compID, tmpWeight);
      }
      pRateCtrl->getRCPic()->updateAfterCTU(pRateCtrl->getRCPic()->getLCUCoded(), actualBits, actualQP, actualLambda,
                                            skipRatio, pcSlice->isIRAP() ? 0 : encCfg->m_RCLCULevelRC);
    }
#if ENABLE_QPA && !ENABLE_QPA_SUB_CTU
    else if (encCfg->m_bUsePerceptQPA && pcSlice->m_pps->m_useDQP)
    {
#if RDOQ_CHROMA_LAMBDA
      pTrQuant->setLambdas(oldLambdaArray);
#else
      pTrQuant->setLambda(oldLambda);
#endif
      pRdCost->setLambda(oldLambda, pcSlice->m_sps->m_bitDepths);
    }
#endif

    m_uiPicTotalBits += actualBits;
    m_uiPicDist = cs.dist;
    // for last Ctu in the slice
    if (pcSlice->m_pps->m_numSubPics >= 2 && curSubPic.m_treatedAsPicFlag &&
        ctuIdx == (pcSlice->getNumCtuInSlice() - 1))
    {
      int subPicX      = (int)curSubPic.m_subPicLeft;
      int subPicY      = (int)curSubPic.m_subPicTop;
      int subPicWidth  = (int)curSubPic.m_subPicWidthInLumaSample;
      int subPicHeight = (int)curSubPic.m_subPicHeightInLumaSample;

      for (int rlist = RPL0; rlist < NUM_RPL01; rlist++)
      {
        int n = pcSlice->m_numRefIdx[(RefPicList)rlist];
        for (int idx = 0; idx < n; idx++)
        {
          Picture *refPic = pcSlice->getRefPic((RefPicList)rlist, idx);
          if (refPic->m_isSubPicBorderSaved)
          {
            refPic->restoreSubPicBorder(refPic->m_poc, subPicX, subPicY, subPicWidth, subPicHeight);
            refPic->m_isSubPicBorderSaved = false;
          }
        }
      }
    }
  }
}

void EncSlice::encodeSlice(Picture *pic, OutputBitstream *pcSubstreams, uint32_t &numBinsCoded)
{

  Slice *const pcSlice                = pic->m_slices[getSliceSegmentIdx()];
  const bool   wavefrontsEnabled      = pcSlice->m_sps->m_entropyCodingSyncEnabledFlag;
  const bool   entryPointsPresentFlag = pcSlice->m_sps->m_entryPointPresentFlag;
  uint32_t     substreamSize          = 0;
  pcSlice->m_numSubstream             = 0;

  // setup coding structure
  CodingStructure &cs = *pic->m_cs;
  cs.slice            = pcSlice;
  // initialise entropy coder for the slice
  m_CABACWriter->initCtxModels(*pcSlice);

  DTRACE(g_trace_ctx, D_HEADER, "=========== POC: %d ===========\n", pcSlice->m_poc);

  pic->m_prevQP.fill(pcSlice->m_iSliceQp);

  const PreCalcValues &pcv         = *cs.pcv;
  const uint32_t       widthInCtus = pcv.widthInCtus;
  uint32_t             uiSubStrm   = 0;

  if (m_timdUsageStatsPrepared)
  {
    m_timdUsageByPoc[pic->m_poc].slices++;
  }

  static Ctx storedCtx;
  // for every CTU in the slice...
  for (uint32_t ctuIdx = 0; ctuIdx < pcSlice->getNumCtuInSlice(); ctuIdx++)
  {
    const uint32_t ctuRsAddr     = pcSlice->getCtuAddrInSlice(ctuIdx);
    const uint32_t ctuXPosInCtus = ctuRsAddr % widthInCtus;
    const uint32_t ctuYPosInCtus = ctuRsAddr / widthInCtus;

    DTRACE_UPDATE(g_trace_ctx, std::make_pair("ctu", ctuRsAddr));

    const Position pos(ctuXPosInCtus * pcv.maxCUWidth, ctuYPosInCtus * pcv.maxCUHeight);
    const UnitArea ctuArea(cs.area.chromaFormat, Area(pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight));
    m_CABACWriter->initBitstream(&pcSubstreams[uiSubStrm]);

    // set up CABAC contexts' state for this CTU
    if (cs.pps->ctuIsTileColBd(ctuXPosInCtus) && cs.pps->ctuIsTileRowBd(ctuYPosInCtus))
    {
      if (ctuIdx != 0)   // if it is the first CTU, then the entropy coder has already been reset
      {
        numBinsCoded += m_CABACWriter->getNumBins();
        m_CABACWriter->initCtxModels(*pcSlice);
        cs.resetPrevPLT(cs.prevPLT);
      }
    }
    else if (cs.pps->ctuIsTileColBd(ctuXPosInCtus) && wavefrontsEnabled)
    {
      // Synchronize cabac probabilities with upper CTU if it's available and at the start of a line.
      if (ctuIdx != 0)   // if it is the first CTU, then the entropy coder has already been reset
      {
        numBinsCoded += m_CABACWriter->getNumBins();
        m_CABACWriter->initCtxModels(*pcSlice);
        cs.resetPrevPLT(cs.prevPLT);
      }
      if (cs.getCURestricted(pos.offset(0, -1), pos, pcSlice->m_independentSliceIdx, cs.pps->getTileIdx(pos),
                             ChannelType::LUMA))
      {
        // Top is available, so use it.
        m_CABACWriter->getCtx() = m_entropyCodingSyncContextState;
        m_CABACWriter->getCtx().riceStatReset(
          pcSlice->m_sps->m_bitDepths[ChannelType::LUMA],
          pcSlice->m_sps->m_spsRangeExtension.m_persistentRiceAdaptationEnabledFlag);
        cs.setPrevPLT(m_palettePredictorSyncState);
      }
    }

    bool updateBcwCodingOrder = cs.slice->m_eSliceType == B_SLICE && ctuIdx == 0;
    if (updateBcwCodingOrder)
    {
      resetBcwCodingOrder(false, cs);
    }
    if (ctuRsAddr == 0)
    {
      if (cs.pps->m_BIF)
      {
        m_CABACWriter->bif(COMP_Y, *pcSlice, cs.picture->getBifParam(COMP_Y));
      }
      if (cs.pps->m_chromaBIF)
      {
        m_CABACWriter->bif(COMP_Cb, *pcSlice, cs.picture->getBifParam(COMP_Cb));
        m_CABACWriter->bif(COMP_Cr, *pcSlice, cs.picture->getBifParam(COMP_Cr));
      }
    }

    if (ctuYPosInCtus)   // final writing of the bitstream - update the CABAC states based on CTU above
    {
      m_CABACWriter->updateCtxs(getBinVector(ctuXPosInCtus));
    }

    m_CABACWriter->setBinBuffer(
      getBinVector(ctuXPosInCtus));   // clear the bin counters and prepare for collecting new data for this CTU
    if (m_timdUsageStatsPrepared)
    {
      m_CABACWriter->resetTimdMergeFlagFracBitsStats();
    }
    m_CABACWriter->coding_tree_unit(cs, ctuArea, pic->m_prevQP, ctuRsAddr);
    m_CABACWriter->setBinBuffer(nullptr);   // done with data collection for this CTU
    xAccumulateTimdUsageStats(cs, ctuArea, pic->m_poc);

    if (storeContexts(pcSlice, ctuXPosInCtus, ctuYPosInCtus))   // store CABAC context to be used in next frames
    {
      storedCtx = m_CABACWriter->getCtx();
    }

    // store probabilities of first CTU in line into buffer
    if (cs.pps->ctuIsTileColBd(ctuXPosInCtus) && wavefrontsEnabled)
    {
      m_entropyCodingSyncContextState = m_CABACWriter->getCtx();
      cs.storePrevPLT(m_palettePredictorSyncState);
    }

    // terminate the sub-stream, if required (end of slice-segment, end of tile, end of wavefront-CTU-row):
    bool isLastCTUsinSlice = ctuIdx == pcSlice->getNumCtuInSlice() - 1;
    bool isLastCTUinTile =
      !isLastCTUsinSlice && cs.pps->getTileIdx(ctuRsAddr) != cs.pps->getTileIdx(pcSlice->getCtuAddrInSlice(ctuIdx + 1));
    bool isLastCTUinWPP = !isLastCTUsinSlice && !isLastCTUinTile && wavefrontsEnabled &&
      cs.pps->ctuIsTileColBd(pcSlice->getCtuAddrInSlice(ctuIdx + 1) % cs.pps->m_picWidthInCtu);
    if (isLastCTUsinSlice || isLastCTUinTile || isLastCTUinWPP)   // this the the last CTU of the slice, tile, or WPP
    {
      m_CABACWriter->end_of_slice();   // end_of_slice_one_bit, end_of_tile_one_bit, or end_of_subset_one_bit

      // Byte-alignment in slice_data() when new tile
      pcSubstreams[uiSubStrm].writeByteAlignment();

      if (!isLastCTUsinSlice)   // Byte alignment only when it is not the last substream in the slice
      {
        // write sub-stream size
        substreamSize +=
          (pcSubstreams[uiSubStrm].getNumberOfWrittenBits() >> 3) + pcSubstreams[uiSubStrm].countStartCodeEmulations();
        pcSlice->m_numSubstream++;
        if (entryPointsPresentFlag)
        {
          pcSlice->m_substreamSizes.push_back(substreamSize);
          substreamSize = 0;
        }
      }
      uiSubStrm++;
    }
  }   // CTU-loop

  // decide cabac initidx
  if (pcSlice->m_pps->m_cabacInitPresentFlag)
  {
    m_encCABACTableIdx = m_CABACWriter->getCtxInitId(*pcSlice);
  }
  else
  {
    m_encCABACTableIdx = pcSlice->m_eSliceType;
  }
  numBinsCoded += m_CABACWriter->getNumBins();
  if (pcSlice->m_pps->pcv->sizeInCtus - 1 ==
      pcSlice->getCtuAddrInSlice(
        pcSlice->getNumCtuInSlice() -
        1))   // store CABAC context to be used in next frames when the last CTU in a picture is processed
  {
    m_CABACWriter->m_CABACDataStore->storeCtxStates(pcSlice, storedCtx);
  }
}

double EncSlice::xGetQPValueAccordingToLambda(double lambda) { return 4.2005 * log(lambda) + 13.7122; }

void EncSlice::xSetUseLICOnPicLevel(Slice *slice, bool fastMode)
{
  slice->m_useLic = slice->m_sps->m_licEnabledFlag && !slice->isIntra();

  if (!fastMode || !slice->m_useLic)
  {
    return;
  }
  slice->m_useLic = false;

  //----- get negated histogram of current picture -----

  int32_t              numValues = 1 << slice->m_sps->m_bitDepths[toChannelType(COMP_Y)];
  std::vector<int32_t> negCurrHist(numValues, 0);
  int32_t              numSamples = slice->m_pic->getOrigBuf().Y().width * slice->m_pic->getOrigBuf().Y().height;
  slice->m_pic->getOrigBuf().Y().subtractHistogram(negCurrHist);

  //----- get SAD threshold -----
  double  sampleThres  = 0.06;
  int32_t sadThreshold = int32_t(sampleThres * double(numSamples));

  //----- check delta histograms -----
  std::vector<int32_t> deltaHist;

  for (int dir = 0; dir < (slice->isInterB() ? 2 : 1) && !slice->m_useLic; dir++)
  {
    RefPicList eList  = RefPicList(dir);
    int        numRef = slice->m_numRefIdx[eList];

    for (int refIdx = 0; refIdx < numRef && !slice->m_useLic; refIdx++)
    {
      // get delta histogram
      deltaHist = negCurrHist;
      if (!slice->getRefPic(eList, refIdx)->isRefScaled(slice->m_pps))
      {
        slice->getRefPic(eList, refIdx)->getOrigBuf().Y().updateHistogram(deltaHist);
      }
      // get SAD of delta histogram
      int32_t sadHist = 0;
      for (std::size_t k = 0; k < numValues; k++)
      {
        sadHist += abs(deltaHist[k]);
      }

      // check
      slice->m_useLic = sadHist > sadThreshold;
    }
  }
}
//! \}
