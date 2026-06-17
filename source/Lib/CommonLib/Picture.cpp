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

/** \file     Picture.cpp
 *  \brief    Description of a coded picture
 */

#include "Picture.h"
#include "SEI.h"
#include "ChromaFormat.h"
#include "CommonLib/InterpolationFilter.h"
#include "UnitTools.h"
#include "CommonDef.h"

struct IdxCost
{
  int     i;
  int     k;
  int32_t cost;
};

// ---------------------------------------------------------------------------
// picture methods
// ---------------------------------------------------------------------------

Picture::Picture()
{
  m_cs                      = nullptr;
  m_isSubPicBorderSaved     = false;
  m_extendedBorder          = false;
  m_wrapAroundValid         = false;
  m_wrapAroundOffset        = 0;
  m_usedByCurr              = false;
  m_longTerm                = false;
  m_reconstructed           = false;
  m_neededForOutput         = false;
  m_referenced              = false;
  m_temporalId              = std::numeric_limits<uint32_t>::max();
  m_fieldPic                = false;
  m_topField                = false;
  m_precedingDRAP           = false;
  m_edrapRapId              = -1;
  m_colourTranfParams       = nullptr;
  m_nonReferencePictureFlag = false;
  m_maxTemporalBtDepth      = 0;

  m_prevQP.fill(-1);
  m_spliceIdx           = nullptr;
  m_ctuNums             = 0;
  m_layerId             = NOT_VALID;
  m_numSlices           = 1;
  m_unscaledPic         = nullptr;
  m_isMctfFiltered      = false;
  m_grainCharacteristic = nullptr;
  m_grainBuf            = nullptr;
}

void Picture::create(const ChromaFormat &_chromaFormat, const Size &size, const unsigned _maxCUSize,
                     const unsigned _margin, const bool _decoder, const int _layerId, const bool rprEnabled,
                     const bool gopBasedTemporalFilterEnabled, const bool fgcSEIAnalysisEnabled
#if JVET_Z0120_SII_SEI_PROCESSING
                     ,
                     const bool enablePostFilteringForHFR
#endif
#if ENABLE_NNLF
                     ,
                     bool nnlfStore
#endif
)
{
  m_layerId = _layerId;
  UnitArea::operator=(UnitArea(_chromaFormat, Area(Position { 0, 0 }, size)));
  margin       = rprEnabled ? (MAX_SCALING_RATIO * _margin) : _margin;
  const Area a = Area(Position(), size);
  m_bufs[PIC_RECONSTRUCTION].create(_chromaFormat, a, _maxCUSize, margin, MEMORY_ALIGN_DEF_SIZE);
  m_bufs[PIC_RECON_WRAP].create(_chromaFormat, a, _maxCUSize, margin, MEMORY_ALIGN_DEF_SIZE);

#if ENABLE_NNLF
  if (nnlfStore)
  {
    const int nnlfMargin = std::max(margin, NNLF_UNIFIED_INFER_SIZE_EXT);
    m_bufs[PIC_BS_MAP].create(_chromaFormat, a, _maxCUSize, nnlfMargin, MEMORY_ALIGN_DEF_SIZE);
    m_bufs[PIC_REC_BEFORE_DBF].create(_chromaFormat, a, _maxCUSize, nnlfMargin, MEMORY_ALIGN_DEF_SIZE);
    m_bufs[PIC_BLOCK_PRED_MODE].create(_chromaFormat, a, _maxCUSize, nnlfMargin, MEMORY_ALIGN_DEF_SIZE);
    m_bufs[PIC_PREDICTION_CUSTOM].create(_chromaFormat, a, _maxCUSize, nnlfMargin, MEMORY_ALIGN_DEF_SIZE);
    m_bufs[PIC_BLOCK_QP].create(_chromaFormat, a, _maxCUSize, nnlfMargin, MEMORY_ALIGN_DEF_SIZE);
  }
#endif

#if JVET_Z0120_SII_SEI_PROCESSING
  if (enablePostFilteringForHFR)
  {
    m_bufs[PIC_YUV_POST_REC].create(_chromaFormat, a, _maxCUSize, margin, MEMORY_ALIGN_DEF_SIZE);
  }
#endif

  if (!_decoder)
  {
    m_bufs[PIC_ORIGINAL].create(_chromaFormat, a);
    m_bufs[PIC_TRUE_ORIGINAL].create(_chromaFormat, a);
    if (gopBasedTemporalFilterEnabled)
    {
      m_bufs[PIC_FILTERED_ORIGINAL].create(_chromaFormat, a);
    }
    if (fgcSEIAnalysisEnabled)
    {
      m_bufs[PIC_FILTERED_ORIGINAL_FG].create(_chromaFormat, a);
    }
  }
#if !KEEP_PRED_AND_RESI_SIGNALS
  m_ctuArea = UnitArea(_chromaFormat, Area(Position { 0, 0 }, Size(_maxCUSize, _maxCUSize)));
#endif
  m_hashMap.clearAll();
}

void Picture::destroy()
{
  for (uint32_t t = 0; t < NUM_PIC_TYPES; t++)
  {
    m_bufs[t].destroy();
  }
  m_hashMap.clearAll();
  if (m_cs)
  {
    m_cs->destroy();
    delete m_cs;
    m_cs = nullptr;
  }

  for (auto &ps: m_slices)
  {
    delete ps;
  }
  m_slices.clear();

  for (auto &psei: m_SEIs)
  {
    delete psei;
  }
  m_SEIs.clear();

  if (m_spliceIdx)
  {
    delete[] m_spliceIdx;
    m_spliceIdx = nullptr;
  }
  m_invColourTransfBuf = nullptr;
  m_grainBuf           = nullptr;
}

void Picture::createTempBuffers(const unsigned _maxCUSize)
{
#if KEEP_PRED_AND_RESI_SIGNALS
  const Area a(Position { 0, 0 }, lumaSize());
#else
  const Area a = m_ctuArea.Y();
#endif

  m_bufs[PIC_PREDICTION].create(chromaFormat, a, _maxCUSize);
  m_bufs[PIC_RESIDUAL].create(chromaFormat, a, _maxCUSize);

  if (m_cs)
  {
    m_cs->rebindPicBufs();
  }
}

void Picture::destroyTempBuffers()
{
  for (uint32_t t = 0; t < NUM_PIC_TYPES; t++)
  {
    if (t == PIC_RESIDUAL || t == PIC_PREDICTION)
    {
      m_bufs[t].destroy();
    }
  }

  if (m_cs)
  {
    m_cs->rebindPicBufs();
  }
}

PelBuf            Picture::getOrigBuf(const CompArea &blk) { return getBuf(blk, PIC_ORIGINAL); }
const CPelBuf     Picture::getOrigBuf(const CompArea &blk) const { return getBuf(blk, PIC_ORIGINAL); }
PelUnitBuf        Picture::getOrigBuf(const UnitArea &unit) { return getBuf(unit, PIC_ORIGINAL); }
const CPelUnitBuf Picture::getOrigBuf(const UnitArea &unit) const { return getBuf(unit, PIC_ORIGINAL); }
PelUnitBuf        Picture::getOrigBuf() { return m_bufs[PIC_ORIGINAL]; }
const CPelUnitBuf Picture::getOrigBuf() const { return m_bufs[PIC_ORIGINAL]; }

PelBuf            Picture::getOrigBuf(const CompID compID) { return getBuf(compID, PIC_ORIGINAL); }
const CPelBuf     Picture::getOrigBuf(const CompID compID) const { return getBuf(compID, PIC_ORIGINAL); }
PelBuf            Picture::getTrueOrigBuf(const CompID compID) { return getBuf(compID, PIC_TRUE_ORIGINAL); }
const CPelBuf     Picture::getTrueOrigBuf(const CompID compID) const { return getBuf(compID, PIC_TRUE_ORIGINAL); }
PelUnitBuf        Picture::getTrueOrigBuf() { return m_bufs[PIC_TRUE_ORIGINAL]; }
const CPelUnitBuf Picture::getTrueOrigBuf() const { return m_bufs[PIC_TRUE_ORIGINAL]; }
PelBuf            Picture::getTrueOrigBuf(const CompArea &blk) { return getBuf(blk, PIC_TRUE_ORIGINAL); }
const CPelBuf     Picture::getTrueOrigBuf(const CompArea &blk) const { return getBuf(blk, PIC_TRUE_ORIGINAL); }

PelUnitBuf        Picture::getFilteredOrigBuf() { return m_bufs[PIC_FILTERED_ORIGINAL]; }
const CPelUnitBuf Picture::getFilteredOrigBuf() const { return m_bufs[PIC_FILTERED_ORIGINAL]; }
PelBuf            Picture::getFilteredOrigBuf(const CompArea &blk) { return getBuf(blk, PIC_FILTERED_ORIGINAL); }
const CPelBuf     Picture::getFilteredOrigBuf(const CompArea &blk) const { return getBuf(blk, PIC_FILTERED_ORIGINAL); }

PelBuf            Picture::getPredBuf(const CompArea &blk) { return getBuf(blk, PIC_PREDICTION); }
const CPelBuf     Picture::getPredBuf(const CompArea &blk) const { return getBuf(blk, PIC_PREDICTION); }
PelUnitBuf        Picture::getPredBuf(const UnitArea &unit) { return getBuf(unit, PIC_PREDICTION); }
const CPelUnitBuf Picture::getPredBuf(const UnitArea &unit) const { return getBuf(unit, PIC_PREDICTION); }

PelBuf            Picture::getResiBuf(const CompArea &blk) { return getBuf(blk, PIC_RESIDUAL); }
const CPelBuf     Picture::getResiBuf(const CompArea &blk) const { return getBuf(blk, PIC_RESIDUAL); }
PelUnitBuf        Picture::getResiBuf(const UnitArea &unit) { return getBuf(unit, PIC_RESIDUAL); }
const CPelUnitBuf Picture::getResiBuf(const UnitArea &unit) const { return getBuf(unit, PIC_RESIDUAL); }

PelBuf Picture::getRecoBuf(const CompID compID, bool wrap)
{
  return getBuf(compID, wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION);
}
const CPelBuf Picture::getRecoBuf(const CompID compID, bool wrap) const
{
  return getBuf(compID, wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION);
}
PelBuf Picture::getRecoBuf(const CompArea &blk, bool wrap)
{
  return getBuf(blk, wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION);
}
const CPelBuf Picture::getRecoBuf(const CompArea &blk, bool wrap) const
{
  return getBuf(blk, wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION);
}
PelUnitBuf Picture::getRecoBuf(const UnitArea &unit, bool wrap)
{
  return getBuf(unit, wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION);
}
const CPelUnitBuf Picture::getRecoBuf(const UnitArea &unit, bool wrap) const
{
  return getBuf(unit, wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION);
}
PelUnitBuf        Picture::getRecoBuf(bool wrap) { return m_bufs[wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION]; }
const CPelUnitBuf Picture::getRecoBuf(bool wrap) const { return m_bufs[wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION]; }

#if ENABLE_NNLF
PelBuf            Picture::getBsMapBuf(const CompID compID) { return getBuf(compID, PIC_BS_MAP); }
PelUnitBuf        Picture::getBsMapBuf() { return m_bufs[PIC_BS_MAP]; }
const CPelUnitBuf Picture::getBsMapBuf() const { return m_bufs[PIC_BS_MAP]; }
PelUnitBuf        Picture::getBsMapBuf(const UnitArea &unit) { return getBuf(unit, PIC_BS_MAP); }
const CPelUnitBuf Picture::getBsMapBuf(const UnitArea &unit) const { return getBuf(unit, PIC_BS_MAP); }
PelBuf            Picture::getBsMapBuf(const CompArea &blk) { return getBuf(blk, PIC_BS_MAP); }
const CPelBuf     Picture::getBsMapBuf(const CompArea &blk) const { return getBuf(blk, PIC_BS_MAP); }

PelBuf            Picture::getPredBufCustom(const CompID compID) { return getBuf(compID, PIC_PREDICTION_CUSTOM); }
PelUnitBuf        Picture::getPredBufCustom() { return m_bufs[PIC_PREDICTION_CUSTOM]; }
const CPelUnitBuf Picture::getPredBufCustom() const { return m_bufs[PIC_PREDICTION_CUSTOM]; }
PelBuf            Picture::getPredBufCustom(const CompArea &blk) { return getBuf(blk, PIC_PREDICTION_CUSTOM); }
const CPelBuf     Picture::getPredBufCustom(const CompArea &blk) const { return getBuf(blk, PIC_PREDICTION_CUSTOM); }
PelUnitBuf        Picture::getPredBufCustom(const UnitArea &unit) { return getBuf(unit, PIC_PREDICTION_CUSTOM); }
const CPelUnitBuf Picture::getPredBufCustom(const UnitArea &unit) const { return getBuf(unit, PIC_PREDICTION_CUSTOM); }

PelBuf            Picture::getRecBeforeDbfBuf(const CompID compID) { return getBuf(compID, PIC_REC_BEFORE_DBF); }
PelUnitBuf        Picture::getRecBeforeDbfBuf() { return m_bufs[PIC_REC_BEFORE_DBF]; }
const CPelUnitBuf Picture::getRecBeforeDbfBuf() const { return m_bufs[PIC_REC_BEFORE_DBF]; }
PelBuf            Picture::getRecBeforeDbfBuf(const CompArea &blk) { return getBuf(blk, PIC_REC_BEFORE_DBF); }
const CPelBuf     Picture::getRecBeforeDbfBuf(const CompArea &blk) const { return getBuf(blk, PIC_REC_BEFORE_DBF); }
PelUnitBuf        Picture::getRecBeforeDbfBuf(const UnitArea &unit) { return getBuf(unit, PIC_REC_BEFORE_DBF); }
const CPelUnitBuf Picture::getRecBeforeDbfBuf(const UnitArea &unit) const { return getBuf(unit, PIC_REC_BEFORE_DBF); }

PelBuf            Picture::getBlockPredModeBuf(const CompID compID) { return getBuf(compID, PIC_BLOCK_PRED_MODE); }
PelUnitBuf        Picture::getBlockPredModeBuf() { return m_bufs[PIC_BLOCK_PRED_MODE]; }
const CPelUnitBuf Picture::getBlockPredModeBuf() const { return m_bufs[PIC_BLOCK_PRED_MODE]; }
PelBuf            Picture::getBlockPredModeBuf(const CompArea &blk) { return getBuf(blk, PIC_BLOCK_PRED_MODE); }
const CPelBuf     Picture::getBlockPredModeBuf(const CompArea &blk) const { return getBuf(blk, PIC_BLOCK_PRED_MODE); }
PelUnitBuf        Picture::getBlockPredModeBuf(const UnitArea &unit) { return getBuf(unit, PIC_BLOCK_PRED_MODE); }
const CPelUnitBuf Picture::getBlockPredModeBuf(const UnitArea &unit) const { return getBuf(unit, PIC_BLOCK_PRED_MODE); }

PelBuf            Picture::getBlockQpBuf(const CompID compID) { return getBuf(compID, PIC_BLOCK_QP); }
PelUnitBuf        Picture::getBlockQpBuf() { return m_bufs[PIC_BLOCK_QP]; }
const CPelUnitBuf Picture::getBlockQpBuf() const { return m_bufs[PIC_BLOCK_QP]; }
PelBuf            Picture::getBlockQpBuf(const CompArea &blk) { return getBuf(blk, PIC_BLOCK_QP); }
const CPelBuf     Picture::getBlockQpBuf(const CompArea &blk) const { return getBuf(blk, PIC_BLOCK_QP); }
PelUnitBuf        Picture::getBlockQpBuf(const UnitArea &unit) { return getBuf(unit, PIC_BLOCK_QP); }
const CPelUnitBuf Picture::getBlockQpBuf(const UnitArea &unit) const { return getBuf(unit, PIC_BLOCK_QP); }
#endif

#if JVET_Z0120_SII_SEI_PROCESSING
PelUnitBuf        Picture::getPostRecBuf() { return m_bufs[PIC_YUV_POST_REC]; }
const CPelUnitBuf Picture::getPostRecBuf() const { return m_bufs[PIC_YUV_POST_REC]; }
#endif

void Picture::finalInit(const VPS *vps, const SPS &sps, const PPS &pps, PicHeader *picHeader, APS **alfApss,
                        APS *lmcsAps, APS *scalingListAps)
{
  for (auto &sei: m_SEIs)
  {
    delete sei;
  }
  m_SEIs.clear();
  clearSliceBuffer();

  const ChromaFormat chromaFormatIdc = sps.m_chromaFormatIdc;
  const int          width           = pps.m_picWidthInLumaSamples;
  const int          height          = pps.m_picHeightInLumaSamples;

  if (m_cs)
  {
    m_cs->initStructData();
  }
  else
  {
    m_cs      = new CodingStructure(g_xuPool);
    m_cs->sps = &sps;
    m_cs->create(chromaFormatIdc, Area(0, 0, width, height), true, (bool)sps.m_PLTMode, sps.m_tempPartPredEnabledFlag);
  }

  m_cs->vps     = vps;
  m_cs->picture = this;
  m_cs->slice =
    nullptr;   // the slices for this picture have not been set at this point. update cs->slice after swapSliceObject()
  m_cs->pps          = &pps;
  picHeader->m_spsId = sps.m_spsId;
  picHeader->m_ppsId = pps.m_ppsId;
  m_cs->picHeader    = picHeader;

  memcpy(m_cs->alfApss, alfApss, sizeof(m_cs->alfApss));
  m_cs->lmcsAps             = lmcsAps;
  m_cs->scalinglistAps      = scalingListAps;
  m_cs->pcv                 = pps.pcv;
  m_conformanceWindow       = pps.m_conformanceWindow;
  m_scalingWindow           = pps.m_scalingWindow;
  m_mixedNaluTypesInPicFlag = pps.m_mixedNaluTypesInPicFlag;
  m_nonReferencePictureFlag = picHeader->m_nonReferencePictureFlag;
  m_chromaFormatIdc         = sps.m_chromaFormatIdc;
  m_bitDepths               = sps.m_bitDepths;

  if (m_spliceIdx == nullptr)
  {
    m_ctuNums   = m_cs->pcv->sizeInCtus;
    m_spliceIdx = new int[m_ctuNums];
    std::fill_n(m_spliceIdx, m_ctuNums, 0);
  }
}

void Picture::allocateNewSlice()
{
  m_slices.push_back(new Slice);
  Slice &slice = *m_slices.back();
  memcpy(slice.m_alfApss, m_cs->alfApss, sizeof(m_cs->alfApss));

  slice.m_pps = m_cs->pps;
  slice.m_sps = m_cs->sps;
  slice.m_vps = m_cs->vps;
  if (m_slices.size() >= 2)
  {
    slice.copySliceInfo(m_slices[m_slices.size() - 2]);
    slice.initSlice();
  }
}

void Picture::fillSliceLossyLosslessArray(std::vector<uint16_t> sliceLosslessIndexArray, bool mixedLossyLossless)
{
  uint16_t numElementsinsliceLosslessIndexArray = (uint16_t)sliceLosslessIndexArray.size();
  uint32_t numSlices                            = m_cs->pps->m_numSlicesInPic;
  m_lossylosslessSliceArray.assign(numSlices, true);   // initialize to all slices are lossless
  if (mixedLossyLossless)
  {
    m_lossylosslessSliceArray.assign(numSlices, false);   // initialize to all slices are lossless
    CHECK(numElementsinsliceLosslessIndexArray == 0,
          "sliceLosslessArray is empty, must need to configure for mixed lossy/lossless");

    // mixed lossy/lossless slices, set only lossless slices;
    for (uint16_t i = 0; i < numElementsinsliceLosslessIndexArray; i++)
    {
      CHECK(sliceLosslessIndexArray[i] >= numSlices || sliceLosslessIndexArray[i] < 0,
            "index of lossless slice is out of slice index bound");
      m_lossylosslessSliceArray[sliceLosslessIndexArray[i]] = true;
    }
  }
  CHECK(m_lossylosslessSliceArray.size() < numSlices, "sliceLosslessArray size is less than number of slices");
}

Slice *Picture::swapSliceObject(Slice *p, uint32_t i)
{
  p->m_pps = m_cs->pps;
  p->m_sps = m_cs->sps;
  p->m_vps = m_cs->vps;
  p->setAlfAPSs(m_cs->alfApss);

  Slice *pTmp = m_slices[i];
  m_slices[i] = p;
  pTmp->m_pps = 0;
  pTmp->m_sps = 0;
  pTmp->m_vps = 0;
  memset(pTmp->m_alfApss, 0, sizeof(*pTmp->m_alfApss) * AlfParameters::ALF_CTB_MAX_NUM_APS);

  return pTmp;
}

void Picture::clearSliceBuffer()
{
  for (uint32_t i = 0; i < uint32_t(m_slices.size()); i++)
  {
    delete m_slices[i];
  }
  m_slices.clear();
}

void Picture::copyAdaptedLumaClip(bool copyRange)
{
  if (copyRange)
  {
    m_lumaClpRng.max = m_cs->slice->m_lumaPelMax;
    m_lumaClpRng.min = m_cs->slice->m_lumaPelMin;
  }

  ClpRngs adaptedClip          = m_cs->slice->m_clpRngs;
  adaptedClip.comp[COMP_Y].max = m_cs->slice->m_lumaPelMax;
  adaptedClip.comp[COMP_Y].min = m_cs->slice->m_lumaPelMin;
  getRecoBuf().copyClip(getRecoBuf(), adaptedClip, true, false);
}

// clang-format off
const TFilterCoeff DownsamplingFilterSRC[8][16][12] =
{
    { // D = 1
      {   0,   0,   0,   0,   0, 128,   0,   0,   0,   0,   0,   0 },
      {   0,   0,   0,   2,  -6, 127,   7,  -2,   0,   0,   0,   0 },
      {   0,   0,   0,   3, -12, 125,  16,  -5,   1,   0,   0,   0 },
      {   0,   0,   0,   4, -16, 120,  26,  -7,   1,   0,   0,   0 },
      {   0,   0,   0,   5, -18, 114,  36, -10,   1,   0,   0,   0 },
      {   0,   0,   0,   5, -20, 107,  46, -12,   2,   0,   0,   0 },
      {   0,   0,   0,   5, -21,  99,  57, -15,   3,   0,   0,   0 },
      {   0,   0,   0,   5, -20,  89,  68, -18,   4,   0,   0,   0 },
      {   0,   0,   0,   4, -19,  79,  79, -19,   4,   0,   0,   0 },
      {   0,   0,   0,   4, -18,  68,  89, -20,   5,   0,   0,   0 },
      {   0,   0,   0,   3, -15,  57,  99, -21,   5,   0,   0,   0 },
      {   0,   0,   0,   2, -12,  46, 107, -20,   5,   0,   0,   0 },
      {   0,   0,   0,   1, -10,  36, 114, -18,   5,   0,   0,   0 },
      {   0,   0,   0,   1,  -7,  26, 120, -16,   4,   0,   0,   0 },
      {   0,   0,   0,   1,  -5,  16, 125, -12,   3,   0,   0,   0 },
      {   0,   0,   0,   0,  -2,   7, 127,  -6,   2,   0,   0,   0 }
    },
    { // D = 1.5
      {   0,   2,   0, -14,  33,  86,  33, -14,   0,   2,   0,   0 },
      {   0,   1,   1, -14,  29,  85,  38, -13,  -1,   2,   0,   0 },
      {   0,   1,   2, -14,  24,  84,  43, -12,  -2,   2,   0,   0 },
      {   0,   1,   2, -13,  19,  83,  48, -11,  -3,   2,   0,   0 },
      {   0,   0,   3, -13,  15,  81,  53, -10,  -4,   3,   0,   0 },
      {   0,   0,   3, -12,  11,  79,  57,  -8,  -5,   3,   0,   0 },
      {   0,   0,   3, -11,   7,  76,  62,  -5,  -7,   3,   0,   0 },
      {   0,   0,   3, -10,   3,  73,  65,  -2,  -7,   3,   0,   0 },
      {   0,   0,   3,  -9,   0,  70,  70,   0,  -9,   3,   0,   0 },
      {   0,   0,   3,  -7,  -2,  65,  73,   3, -10,   3,   0,   0 },
      {   0,   0,   3,  -7,  -5,  62,  76,   7, -11,   3,   0,   0 },
      {   0,   0,   3,  -5,  -8,  57,  79,  11, -12,   3,   0,   0 },
      {   0,   0,   3,  -4, -10,  53,  81,  15, -13,   3,   0,   0 },
      {   0,   0,   2,  -3, -11,  48,  83,  19, -13,   2,   1,   0 },
      {   0,   0,   2,  -2, -12,  43,  84,  24, -14,   2,   1,   0 },
      {   0,   0,   2,  -1, -13,  38,  85,  29, -14,   1,   1,   0 }
    },
    { // D = 2
      {   0,   5,   -6,  -10,  37,  76,   37,  -10,  -6,    5,  0,   0}, //0
      {   0,   5,   -4,  -11,  33,  76,   40,  -9,    -7,    5,  0,   0}, //1
      //{   0,   5,   -3,  -12,  28,  75,   44,  -7,    -8,    5,  1,   0}, //2
      {  -1,   5,   -3,  -12,  29,  75,   45,  -7,    -8,   5,  0,   0}, //2 new coefficients in m24499
      {  -1,   4,   -2,  -13,  25,  75,   48,  -5,    -9,    5,  1,   0}, //3
      {  -1,   4,   -1,  -13,  22,  73,   52,  -3,    -10,  4,  1,   0}, //4
      {  -1,   4,   0,    -13,  18,  72,   55,  -1,    -11,  4,  2,  -1}, //5
      {  -1,   4,   1,    -13,  14,  70,   59,  2,    -12,  3,  2,  -1}, //6
      {  -1,   3,   1,    -13,  11,  68,   62,  5,    -12,  3,  2,  -1}, //7
      {  -1,   3,   2,    -13,  8,  65,   65,  8,    -13,  2,  3,  -1}, //8
      {  -1,   2,   3,    -12,  5,  62,   68,  11,    -13,  1,  3,  -1}, //9
      {  -1,   2,   3,    -12,  2,  59,   70,  14,    -13,  1,  4,  -1}, //10
      {  -1,   2,   4,    -11,  -1,  55,   72,  18,    -13,  0,  4,  -1}, //11
      {   0,   1,   4,    -10,  -3,  52,   73,  22,    -13,  -1,  4,  -1}, //12
      {   0,   1,   5,    -9,    -5,  48,   75,  25,    -13,  -2,  4,  -1}, //13
      //{   0,   1,   5,    -8,    -7,  44,   75,  28,    -12,  -3,  5,   0}, //14
      {    0,   0,   5,    -8,   -7,  45,   75,  29,    -12,  -3,  5,  -1}  , //14 new coefficients in m24499
      {   0,   0,   5,    -7,    -9,  40,   76,  33,    -11,  -4,  5,   0}, //15
    },
    { // D = 2.5
      {   2,  -3,   -9,  6,   39,  58,   39,  6,   -9,  -3,    2,    0}, // 0
      {   2,  -3,   -9,  4,   38,  58,   43,  7,   -9,  -4,    1,    0}, // 1
      {   2,  -2,   -9,  2,   35,  58,   44,  9,   -8,  -4,    1,    0}, // 2
      {   1,  -2,   -9,  1,   34,  58,   46,  11,   -8,  -5,    1,    0}, // 3
      //{   1,  -1,   -8,  -1,   31,  57,   48,  13,   -8,  -5,    1,    0}, // 4
      {   1,  -1,   -8,  -1,   31,  57,   47,  13,   -7,  -5,    1,    0},  // 4 new coefficients in m24499
      {   1,  -1,   -8,  -2,   29,  56,   49,  15,   -7,  -6,    1,    1}, // 5
      {   1,  0,   -8,  -3,   26,  55,   51,  17,   -7,  -6,    1,    1}, // 6
      {   1,  0,   -7,  -4,   24,  54,   52,  19,   -6,  -7,    1,    1}, // 7
      {   1,  0,   -7,  -5,   22,  53,   53,  22,   -5,  -7,    0,    1}, // 8
      {   1,  1,   -7,  -6,   19,  52,   54,  24,   -4,  -7,    0,    1}, // 9
      {   1,  1,   -6,  -7,   17,  51,   55,  26,   -3,  -8,    0,    1}, // 10
      {   1,  1,   -6,  -7,   15,  49,   56,  29,   -2,  -8,    -1,    1}, // 11
      //{   0,  1,   -5,  -8,   13,  48,   57,  31,   -1,  -8,    -1,    1}, // 12 new coefficients in m24499
      {   0,  1,   -5,  -7,   13,  47,  57,  31,  -1,    -8,   -1,    1}, // 12
      {   0,  1,   -5,  -8,   11,  46,   58,  34,   1,    -9,    -2,    1}, // 13
      {   0,  1,   -4,  -8,   9,    44,   58,  35,   2,    -9,    -2,    2}, // 14
      {   0,  1,   -4,  -9,   7,    43,   58,  38,   4,    -9,    -3,    2}, // 15
    },
    { // D = 3
      {  -2,  -7,   0,  17,  35,  43,  35,  17,   0,  -7,  -5,   2 },
      {  -2,  -7,  -1,  16,  34,  43,  36,  18,   1,  -7,  -5,   2 },
      {  -1,  -7,  -1,  14,  33,  43,  36,  19,   1,  -6,  -5,   2 },
      {  -1,  -7,  -2,  13,  32,  42,  37,  20,   3,  -6,  -5,   2 },
      {   0,  -7,  -3,  12,  31,  42,  38,  21,   3,  -6,  -5,   2 },
      {   0,  -7,  -3,  11,  30,  42,  39,  23,   4,  -6,  -6,   1 },
      {   0,  -7,  -4,  10,  29,  42,  40,  24,   5,  -6,  -6,   1 },
      {   1,  -7,  -4,   9,  27,  41,  40,  25,   6,  -5,  -6,   1 },
      {   1,  -6,  -5,   7,  26,  41,  41,  26,   7,  -5,  -6,   1 },
      {   1,  -6,  -5,   6,  25,  40,  41,  27,   9,  -4,  -7,   1 },
      {   1,  -6,  -6,   5,  24,  40,  42,  29,  10,  -4,  -7,   0 },
      {   1,  -6,  -6,   4,  23,  39,  42,  30,  11,  -3,  -7,   0 },
      {   2,  -5,  -6,   3,  21,  38,  42,  31,  12,  -3,  -7,   0 },
      {   2,  -5,  -6,   3,  20,  37,  42,  32,  13,  -2,  -7,  -1 },
      {   2,  -5,  -6,   1,  19,  36,  43,  33,  14,  -1,  -7,  -1 },
      {   2,  -5,  -7,   1,  18,  36,  43,  34,  16,  -1,  -7,  -2 }
    },
    { // D = 3.5
      {  -6,  -3,   5,  19,  31,  36,  31,  19,   5,  -3,  -6,   0 },
      {  -6,  -4,   4,  18,  31,  37,  32,  20,   6,  -3,  -6,  -1 },
      {  -6,  -4,   4,  17,  30,  36,  33,  21,   7,  -3,  -6,  -1 },
      {  -5,  -5,   3,  16,  30,  36,  33,  22,   8,  -2,  -6,  -2 },
      {  -5,  -5,   2,  15,  29,  36,  34,  23,   9,  -2,  -6,  -2 },
      {  -5,  -5,   2,  15,  28,  36,  34,  24,  10,  -2,  -6,  -3 },
      {  -4,  -5,   1,  14,  27,  36,  35,  24,  10,  -1,  -6,  -3 },
      {  -4,  -5,   0,  13,  26,  35,  35,  25,  11,   0,  -5,  -3 },
      {  -4,  -6,   0,  12,  26,  36,  36,  26,  12,   0,  -6,  -4 },
      {  -3,  -5,   0,  11,  25,  35,  35,  26,  13,   0,  -5,  -4 },
      {  -3,  -6,  -1,  10,  24,  35,  36,  27,  14,   1,  -5,  -4 },
      {  -3,  -6,  -2,  10,  24,  34,  36,  28,  15,   2,  -5,  -5 },
      {  -2,  -6,  -2,   9,  23,  34,  36,  29,  15,   2,  -5,  -5 },
      {  -2,  -6,  -2,   8,  22,  33,  36,  30,  16,   3,  -5,  -5 },
      {  -1,  -6,  -3,   7,  21,  33,  36,  30,  17,   4,  -4,  -6 },
      {  -1,  -6,  -3,   6,  20,  32,  37,  31,  18,   4,  -4,  -6 }
    },
    { // D = 4
      {  -9,   0,   9,  20,  28,  32,  28,  20,   9,   0,  -9,   0 },
      {  -9,   0,   8,  19,  28,  32,  29,  20,  10,   0,  -4,  -5 },
      {  -9,  -1,   8,  18,  28,  32,  29,  21,  10,   1,  -4,  -5 },
      {  -9,  -1,   7,  18,  27,  32,  30,  22,  11,   1,  -4,  -6 },
      {  -8,  -2,   6,  17,  27,  32,  30,  22,  12,   2,  -4,  -6 },
      {  -8,  -2,   6,  16,  26,  32,  31,  23,  12,   2,  -4,  -6 },
      {  -8,  -2,   5,  16,  26,  31,  31,  23,  13,   3,  -3,  -7 },
      {  -8,  -3,   5,  15,  25,  31,  31,  24,  14,   4,  -3,  -7 },
      {  -7,  -3,   4,  14,  25,  31,  31,  25,  14,   4,  -3,  -7 },
      {  -7,  -3,   4,  14,  24,  31,  31,  25,  15,   5,  -3,  -8 },
      {  -7,  -3,   3,  13,  23,  31,  31,  26,  16,   5,  -2,  -8 },
      {  -6,  -4,   2,  12,  23,  31,  32,  26,  16,   6,  -2,  -8 },
      {  -6,  -4,   2,  12,  22,  30,  32,  27,  17,   6,  -2,  -8 },
      {  -6,  -4,   1,  11,  22,  30,  32,  27,  18,   7,  -1,  -9 },
      {  -5,  -4,   1,  10,  21,  29,  32,  28,  18,   8,  -1,  -9 },
      {  -5,  -4,   0,  10,  20,  29,  32,  28,  19,   8,   0,  -9 }
    },
    { // D = 5.5
      {  -8,   7,  13,  18,  22,  24,  22,  18,  13,   7,   2, -10 },
      {  -8,   7,  13,  18,  22,  23,  22,  19,  13,   7,   2, -10 },
      {  -8,   6,  12,  18,  22,  23,  22,  19,  14,   8,   2, -10 },
      {  -9,   6,  12,  17,  22,  23,  23,  19,  14,   8,   3, -10 },
      {  -9,   6,  12,  17,  21,  23,  23,  19,  14,   9,   3, -10 },
      {  -9,   5,  11,  17,  21,  23,  23,  20,  15,   9,   3, -10 },
      {  -9,   5,  11,  16,  21,  23,  23,  20,  15,   9,   4, -10 },
      {  -9,   5,  10,  16,  21,  23,  23,  20,  15,  10,   4, -10 },
      { -10,   5,  10,  16,  20,  23,  23,  20,  16,  10,   5, -10 },
      { -10,   4,  10,  15,  20,  23,  23,  21,  16,  10,   5,  -9 },
      { -10,   4,   9,  15,  20,  23,  23,  21,  16,  11,   5,  -9 },
      { -10,   3,   9,  15,  20,  23,  23,  21,  17,  11,   5,  -9 },
      { -10,   3,   9,  14,  19,  23,  23,  21,  17,  12,   6,  -9 },
      { -10,   3,   8,  14,  19,  23,  23,  22,  17,  12,   6,  -9 },
      { -10,   2,   8,  14,  19,  22,  23,  22,  18,  12,   6,  -8 },
      { -10,   2,   7,  13,  19,  22,  23,  22,  18,  13,   7,  -8 }
    }
};

const TFilterCoeff m_lumaFilter12_alt[16][12] =
{
{ 0, 0, 0, 0, 0, 256, 0, 0, 0, 0, 0, 0, },
{ 1, -1, 0, 3, -12, 253, 16, -6, 2, 0, 0, 0, },
{ 0, 0, -3, 9, -24, 250, 32, -11, 4, -1, 0, 0, },
{ 0, 0, -4, 12, -32, 241, 52, -18, 8, -4, 2, -1, },
{ 0, 1, -6, 15, -38, 228, 75, -28, 14, -7, 3, -1, },
{ 0, 1, -7, 18, -43, 214, 96, -33, 16, -8, 3, -1, },
{ 1, 0, -6, 17, -44, 196, 119, -40, 20, -10, 4, -1, },
{ 0, 2, -9, 21, -47, 180, 139, -43, 20, -10, 4, -1, },
{ -1, 3, -9, 21, -46, 160, 160, -46, 21, -9, 3, -1, },
{ -1, 4, -10, 20, -43, 139, 180, -47, 21, -9, 2, 0, },
{ -1, 4, -10, 20, -40, 119, 196, -44, 17, -6, 0, 1, },
{ -1, 3, -8, 16, -33, 96, 214, -43, 18, -7, 1, 0, },
{ -1, 3, -7, 14, -28, 75, 228, -38, 15, -6, 1, 0, },
{ -1, 2, -4, 8, -18, 52, 241, -32, 12, -4, 0, 0, },
{ 0, 0, -1, 4, -11, 32, 250, -24, 9, -3, 0, 0, },
{ 0, 0, 0, 2, -6, 16, 253, -12, 3, 0, -1, 1, },
};
const TFilterCoeff m_chromaFilter6_alt[32][6] =
{
{0, 0, 256, 0, 0, 0, },
{ 1, -6, 256, 6, -1, 0, },
{ 2, -11, 254, 14, -4, 1, },
{ 4, -18, 252, 23, -6, 1, },
{ 6, -24, 249, 32, -9, 2, },
{ 6, -26, 244, 41, -12, 3, },
{ 7, -30, 239, 53, -18, 5, },
{ 8, -34, 235, 61, -19, 5, },
{ 10, -38, 228, 72, -22, 6, },
{ 10, -39, 220, 84, -26, 7, },
{ 10, -40, 213, 94, -29, 8, },
{ 11, -42, 205, 105, -32, 9, },
{ 11, -42, 196, 116, -35, 10, },
{ 11, -42, 186, 128, -37, 10, },
{ 11, -42, 177, 138, -38, 10, },
{ 11, -41, 167, 148, -40, 11, },
{ 11, -41, 158, 158, -41, 11, },
{ 11, -40, 148, 167, -41, 11, },
{ 10, -38, 138, 177, -42, 11, },
{ 10, -37, 128, 186, -42, 11, },
{ 10, -35, 116, 196, -42, 11, },
{ 9, -32, 105, 205, -42, 11, },
{ 8, -29, 94, 213, -40, 10, },
{ 7, -26, 84, 220, -39, 10, },
{ 6, -22, 72, 228, -38, 10, },
{ 5, -19, 61, 235, -34, 8, },
{ 5, -18, 53, 239, -30, 7, },
{ 3, -12, 41, 244, -26, 6, },
{ 2, -9, 32, 249, -24, 6, },
{ 1, -6, 23, 252, -18, 4, },
{ 1, -4, 14, 254, -11, 2, },
{ 0, -1, 6, 256, -6, 1, }
};

const TFilterCoeff m_lumaFilter_8[16][8] =
{
    { 0, 0, 0, 64, 0, 0, 0, 0 },
    {  0, 1, -3, 63, 4, -2, 1, 0 },
    { -1, 2, -5, 62, 8, -3, 1, 0 },
    { -1, 3, -8, 60, 13, -4, 1, 0 },
    { -1, 4, -10, 58, 17, -5, 1, 0 },
    { -1, 4, -11, 52, 26, -8, 3, -1 },
    { -1, 3, -9, 47, 31, -10, 4, -1 },
    { -1, 4, -11, 45, 34, -10, 4, -1 },
    { -1, 4, -11, 40, 40, -11, 4, -1 },
    { -1, 4, -10, 34, 45, -11, 4, -1 },
    { -1, 4, -10, 31, 47, -9, 3, -1 },
    { -1, 3, -8, 26, 52, -11, 4, -1 },
    {  0, 1, -5, 17, 58, -10, 4, -1 },
    {  0, 1, -4, 13, 60, -8, 3, -1 },
    {  0, 1, -3, 8, 62, -5, 2, -1 },
    {  0, 1, -2, 4, 63, -3, 1, 0 }
   };
const TFilterCoeff m_chromaFilter_4[32][4] =
{
    {0, 64, 0, 0},
    { -1, 63, 2, 0 },
    { -2, 62, 4, 0 },
    { -2, 60, 7, -1 },
    { -2, 58, 10, -2 },
    { -3, 57, 12, -2 },
    { -4, 56, 14, -2 },
    { -4, 55, 15, -2 },
    { -4, 54, 16, -2 },
    { -5, 53, 18, -2 },
    { -6, 52, 20, -2 },
    { -6, 49, 24, -3 },
    { -6, 46, 28, -4 },
    { -5, 44, 29, -4 },
    { -4, 42, 30, -4 },
    { -4, 39, 33, -4 },
    { -4, 36, 36, -4 },
    { -4, 33, 39, -4 },
    { -4, 30, 42, -4 },
    { -4, 29, 44, -5 },
    { -4, 28, 46, -6 },
    { -3, 24, 49, -6 },
    { -2, 20, 52, -6 },
    { -2, 18, 53, -5 },
    { -2, 16, 54, -4 },
    { -2, 15, 55, -4 },
    { -2, 14, 56, -4 },
    { -2, 12, 57, -3 },
    { -2, 10, 58, -2 },
    { -1, 7, 60, -2 },
    {  0, 4, 62, -2 },
    {  0, 2, 63, -1 },
    };
// clang-format on

void Picture::sampleRateConv(const ScalingRatio scalingRatio, const int scaleX, const int scaleY,
                             const CPelBuf &beforeScale, const int beforeScaleLeftOffset,
                             const int beforeScaleTopOffset, const PelBuf &afterScale, const int afterScaleLeftOffset,
                             const int afterScaleTopOffset, const int bitDepth, const bool useLumaFilter,
                             const bool downsampling, const bool horCollocatedPositionFlag,
                             const bool verCollocatedPositionFlag, const bool rescaleForDisplay,
                             const int upscaleFilterForDisplay)
{
  const Pel      *orgSrc    = beforeScale.buf;
  const int       orgWidth  = beforeScale.width;
  const int       orgHeight = beforeScale.height;
  const ptrdiff_t orgStride = beforeScale.stride;

  Pel            *scaledSrc    = afterScale.buf;
  const int       scaledWidth  = afterScale.width;
  const int       scaledHeight = afterScale.height;
  const ptrdiff_t scaledStride = afterScale.stride;

  if (orgWidth == scaledWidth && orgHeight == scaledHeight && scalingRatio == SCALE_1X && !beforeScaleLeftOffset &&
      !beforeScaleTopOffset && !afterScaleLeftOffset && !afterScaleTopOffset)
  {
    for (int j = 0; j < orgHeight; j++)
    {
      memcpy(scaledSrc + j * scaledStride, orgSrc + j * orgStride, sizeof(Pel) * orgWidth);
    }

    return;
  }
  const TFilterCoeff *filterHor =
    useLumaFilter ? &InterpolationFilter::m_lumaFilter12[0][0] : &InterpolationFilter::m_chromaFilter[0][0];
  const TFilterCoeff *filterVer =
    useLumaFilter ? &InterpolationFilter::m_lumaFilter12[0][0] : &InterpolationFilter::m_chromaFilter[0][0];
  if (rescaleForDisplay)
  {
    if (upscaleFilterForDisplay != 2)
    {
      filterHor = useLumaFilter ? (upscaleFilterForDisplay == 1 ? &m_lumaFilter12_alt[0][0] : &m_lumaFilter_8[0][0])
                                : (upscaleFilterForDisplay == 1 ? &m_chromaFilter6_alt[0][0] : &m_chromaFilter_4[0][0]);
      filterVer = useLumaFilter ? (upscaleFilterForDisplay == 1 ? &m_lumaFilter12_alt[0][0] : &m_lumaFilter_8[0][0])
                                : (upscaleFilterForDisplay == 1 ? &m_chromaFilter6_alt[0][0] : &m_chromaFilter_4[0][0]);
    }
  }
  const int numFracPositions = useLumaFilter ? 15 : 31;
  const int numFracShift     = useLumaFilter ? 4 : 5;

  const int posShiftX = ScalingRatio::BITS - numFracShift + scaleX;
  const int posShiftY = ScalingRatio::BITS - numFracShift + scaleY;

  const int addX = (1 << (posShiftX - 1)) + (beforeScaleLeftOffset << ScalingRatio::BITS) +
    ((int(1 - horCollocatedPositionFlag) * 8 * (scalingRatio.x - SCALE_1X.x) + (1 << (2 + scaleX))) >> (3 + scaleX));
  const int addY = (1 << (posShiftY - 1)) + (beforeScaleTopOffset << ScalingRatio::BITS) +
    ((int(1 - verCollocatedPositionFlag) * 8 * (scalingRatio.y - SCALE_1X.y) + (1 << (2 + scaleY))) >> (3 + scaleY));

  if (downsampling)
  {
    int verFilter = 0;
    int horFilter = 0;

    if (scalingRatio.x > (15 << ScalingRatio::BITS) / 4)
    {
      horFilter = 7;
    }
    else if (scalingRatio.x > (20 << ScalingRatio::BITS) / 7)
    {
      horFilter = 6;
    }
    else if (scalingRatio.x > (5 << ScalingRatio::BITS) / 2)
    {
      horFilter = 5;
    }
    else if (scalingRatio.x > (2 << ScalingRatio::BITS))
    {
      horFilter = 4;
    }
    else if (scalingRatio.x > (5 << ScalingRatio::BITS) / 3)
    {
      horFilter = 3;
    }
    else if (scalingRatio.x > (5 << ScalingRatio::BITS) / 4)
    {
      horFilter = 2;
    }
    else if (scalingRatio.x > (20 << ScalingRatio::BITS) / 19)
    {
      horFilter = 1;
    }

    if (scalingRatio.y > (15 << ScalingRatio::BITS) / 4)
    {
      verFilter = 7;
    }
    else if (scalingRatio.y > (20 << ScalingRatio::BITS) / 7)
    {
      verFilter = 6;
    }
    else if (scalingRatio.y > (5 << ScalingRatio::BITS) / 2)
    {
      verFilter = 5;
    }
    else if (scalingRatio.y > (2 << ScalingRatio::BITS))
    {
      verFilter = 4;
    }
    else if (scalingRatio.y > (5 << ScalingRatio::BITS) / 3)
    {
      verFilter = 3;
    }
    else if (scalingRatio.y > (5 << ScalingRatio::BITS) / 4)
    {
      verFilter = 2;
    }
    else if (scalingRatio.y > (20 << ScalingRatio::BITS) / 19)
    {
      verFilter = 1;
    }

    filterHor = &DownsamplingFilterSRC[horFilter][0][0];
    filterVer = &DownsamplingFilterSRC[verFilter][0][0];
  }

  int       filterLengthsLuma[3]   = { 8, 12, 12 };
  int       filterLengthsChroma[3] = { 4, 6, 6 };
  int       log2NormList[3]        = { 12, 16, 16 };
  const int filterLength           = downsampling
              ? 12
              : (rescaleForDisplay
                   ? (useLumaFilter ? filterLengthsLuma[upscaleFilterForDisplay] : filterLengthsChroma[upscaleFilterForDisplay])
                   : useLumaFilter ? NTAPS_LUMA
                                   : NTAPS_CHROMA);
  const int log2Norm = downsampling ? 14 : (rescaleForDisplay ? log2NormList[upscaleFilterForDisplay] : 16);
  int      *buf      = new int[orgHeight * scaledWidth];
  int       maxVal   = (1 << bitDepth) - 1;

  CHECK(bitDepth > 17, "Overflow may happen!");

  for (int i = 0; i < scaledWidth; i++)
  {
    const Pel *org = orgSrc;

    int  refPos  = (((i << scaleX) - afterScaleLeftOffset) * scalingRatio.x + addX) >> posShiftX;
    int  integer = refPos >> numFracShift;
    int  frac    = refPos & numFracPositions;
    int *tmp     = buf + i;

    for (int j = 0; j < orgHeight; j++)
    {
      int                 sum = 0;
      const TFilterCoeff *f   = filterHor + frac * filterLength;

      for (int k = 0; k < filterLength; k++)
      {
        int xInt = std::min<int>(std::max(0, integer + k - filterLength / 2 + 1), orgWidth - 1);
        sum += f[k] * org[xInt];   // postpone horizontal filtering gain removal after vertical filtering
      }

      *tmp = sum;

      tmp += scaledWidth;
      org += orgStride;
    }
  }

  Pel *dst = scaledSrc;

  for (int j = 0; j < scaledHeight; j++)
  {
    int refPos  = (((j << scaleY) - afterScaleTopOffset) * scalingRatio.y + addY) >> posShiftY;
    int integer = refPos >> numFracShift;
    int frac    = refPos & numFracPositions;

    for (int i = 0; i < scaledWidth; i++)
    {
      uint64_t            sum = 0;
      int                *tmp = buf + i;
      const TFilterCoeff *f   = filterVer + frac * filterLength;

      for (int k = 0; k < filterLength; k++)
      {
        int yInt = std::min<int>(std::max(0, integer + k - filterLength / 2 + 1), orgHeight - 1);
        sum += f[k] * tmp[yInt * scaledWidth];
      }
      const uint64_t one  = 1;
      int            sumS = (int)((sum + (one << (log2Norm - 1))) >> log2Norm);
      dst[i]              = std::min<int>(std::max(0, sumS), maxVal);
    }

    dst += scaledStride;
  }

  delete[] buf;
}

void Picture::rescalePicture(const ScalingRatio scalingRatio, const CPelUnitBuf &beforeScaling,
                             const Window &scalingWindowBefore, const PelUnitBuf &afterScaling,
                             const Window &scalingWindowAfter, const ChromaFormat chromaFormatIdc,
                             const BitDepths &bitDepths, const bool useLumaFilter, const bool downsampling,
                             const bool horCollocatedChromaFlag, const bool verCollocatedChromaFlag,
                             bool rescaleForDisplay, int upscaleFilterForDisplay)
{
  for (int comp = 0; comp < ::getNumberValidComponents(chromaFormatIdc); comp++)
  {
    CompID         compID      = CompID(comp);
    const CPelBuf &beforeScale = beforeScaling.get(compID);
    const PelBuf  &afterScale  = afterScaling.get(compID);

    sampleRateConv(
      scalingRatio, ::getComponentScaleX(compID, chromaFormatIdc), ::getComponentScaleY(compID, chromaFormatIdc),
      beforeScale, scalingWindowBefore.m_winLeftOffset * SPS::getWinUnitX(chromaFormatIdc),
      scalingWindowBefore.m_winTopOffset * SPS::getWinUnitY(chromaFormatIdc), afterScale,
      scalingWindowAfter.m_winLeftOffset * SPS::getWinUnitX(chromaFormatIdc),
      scalingWindowAfter.m_winTopOffset * SPS::getWinUnitY(chromaFormatIdc), bitDepths[toChannelType(compID)],
      downsampling || useLumaFilter ? true : isLuma(compID), downsampling, isLuma(compID) ? 1 : horCollocatedChromaFlag,
      isLuma(compID) ? 1 : verCollocatedChromaFlag, rescaleForDisplay, upscaleFilterForDisplay);
  }
}

void Picture::saveSubPicBorder(int POC, int subPicX0, int subPicY0, int subPicWidth, int subPicHeight)
{

  // 1.1 set up margin for back up memory allocation
  int xMargin = margin >> getComponentScaleX(COMP_Y, m_cs->area.chromaFormat);
  int yMargin = margin >> getComponentScaleY(COMP_Y, m_cs->area.chromaFormat);

  // 1.2 measure the size of back up memory
  Area     areaAboveBelow(0, 0, subPicWidth + 2 * xMargin, yMargin);
  Area     areaLeftRight(0, 0, xMargin, subPicHeight);
  UnitArea unitAreaAboveBelow(m_cs->area.chromaFormat, areaAboveBelow);
  UnitArea unitAreaLeftRight(m_cs->area.chromaFormat, areaLeftRight);

  // 1.3 create back up memory
  m_bufSubPicAbove.create(unitAreaAboveBelow);
  m_bufSubPicBelow.create(unitAreaAboveBelow);
  m_bufSubPicLeft.create(unitAreaLeftRight);
  m_bufSubPicRight.create(unitAreaLeftRight);
  m_bufWrapSubPicAbove.create(unitAreaAboveBelow);
  m_bufWrapSubPicBelow.create(unitAreaAboveBelow);

  for (int comp = 0; comp < getNumberValidComponents(m_cs->area.chromaFormat); comp++)
  {
    CompID compID = CompID(comp);

    // 2.1 measure the margin for each component
    int xmargin = margin >> getComponentScaleX(compID, m_cs->area.chromaFormat);
    int ymargin = margin >> getComponentScaleY(compID, m_cs->area.chromaFormat);

    // 2.2 calculate the origin of the subpicture
    int left = subPicX0 >> getComponentScaleX(compID, m_cs->area.chromaFormat);
    int top  = subPicY0 >> getComponentScaleY(compID, m_cs->area.chromaFormat);

    // 2.3 calculate the width/height of the subPic
    int width  = subPicWidth >> getComponentScaleX(compID, m_cs->area.chromaFormat);
    int height = subPicHeight >> getComponentScaleY(compID, m_cs->area.chromaFormat);

    // 3.1.1 set reconstructed picture
    PelBuf s   = m_bufs[PIC_RECONSTRUCTION].get(compID);
    Pel   *src = s.bufAt(left, top);

    // 3.2.1 set back up buffer for left
    PelBuf dBufLeft = m_bufSubPicLeft.getBuf(compID);
    Pel   *dstLeft  = dBufLeft.bufAt(0, 0);

    // 3.2.2 set back up buffer for right
    PelBuf dBufRight = m_bufSubPicRight.getBuf(compID);
    Pel   *dstRight  = dBufRight.bufAt(0, 0);

    // 3.2.3 copy to recon picture to back up buffer
    Pel *srcLeft  = src - xmargin;
    Pel *srcRight = src + width;
    for (int y = 0; y < height; y++)
    {
      ::memcpy(dstLeft + y * dBufLeft.stride, srcLeft + y * s.stride, sizeof(Pel) * xmargin);
      ::memcpy(dstRight + y * dBufRight.stride, srcRight + y * s.stride, sizeof(Pel) * xmargin);
    }

    // 3.3.1 set back up buffer for above
    PelBuf dBufTop = m_bufSubPicAbove.getBuf(compID);
    Pel   *dstTop  = dBufTop.bufAt(0, 0);

    // 3.3.2 set back up buffer for below
    PelBuf dBufBottom = m_bufSubPicBelow.getBuf(compID);
    Pel   *dstBottom  = dBufBottom.bufAt(0, 0);

    // 3.3.3 copy to recon picture to back up buffer
    Pel *srcTop    = src - xmargin - ymargin * s.stride;
    Pel *srcBottom = src - xmargin + height * s.stride;
    for (int y = 0; y < ymargin; y++)
    {
      ::memcpy(dstTop + y * dBufTop.stride, srcTop + y * s.stride, sizeof(Pel) * (2 * xmargin + width));
      ::memcpy(dstBottom + y * dBufBottom.stride, srcBottom + y * s.stride, sizeof(Pel) * (2 * xmargin + width));
    }

    // back up recon wrap buffer
    if (m_cs->sps->m_wrapAroundEnabledFlag)
    {
      PelBuf sWrap   = m_bufs[PIC_RECON_WRAP].get(compID);
      Pel   *srcWrap = sWrap.bufAt(left, top);

      // 3.4.1 set back up buffer for above
      PelBuf dBufTopWrap = m_bufWrapSubPicAbove.getBuf(compID);
      Pel   *dstTopWrap  = dBufTopWrap.bufAt(0, 0);

      // 3.4.2 set back up buffer for below
      PelBuf dBufBottomWrap = m_bufWrapSubPicBelow.getBuf(compID);
      Pel   *dstBottomWrap  = dBufBottomWrap.bufAt(0, 0);

      // 3.4.3 copy recon wrap picture to back up buffer
      Pel *srcTopWrap    = srcWrap - xmargin - ymargin * sWrap.stride;
      Pel *srcBottomWrap = srcWrap - xmargin + height * sWrap.stride;
      for (int y = 0; y < ymargin; y++)
      {
        ::memcpy(dstTopWrap + y * dBufTopWrap.stride, srcTopWrap + y * sWrap.stride,
                 sizeof(Pel) * (2 * xmargin + width));
        ::memcpy(dstBottomWrap + y * dBufBottomWrap.stride, srcBottomWrap + y * sWrap.stride,
                 sizeof(Pel) * (2 * xmargin + width));
      }
    }
  }
}

#if ENABLE_NNLF
void Picture::dumpPicBpmInfo()
{
  const PreCalcValues &pcv = *m_cs->pcv;
  // Defining a scanning area that covers the picture
  const UnitArea       picArea(m_cs->area.chromaFormat, Area(0, 0, pcv.lumaWidth, pcv.lumaHeight));
  // Traversing the CUs in the picture and filling a buffer with extracted block pred mode info
  for (int y = 0; y < pcv.heightInCtus; y++)
  {
    for (int x = 0; x < pcv.widthInCtus; x++)
    {
      const UnitArea ctuArea(pcv.chrFormat,
                             Area(x << pcv.maxCUWidthLog2, y << pcv.maxCUHeightLog2, pcv.maxCUWidth, pcv.maxCUWidth));
      for (auto &currCU: m_cs->traverseCUs(CS::getArea(*m_cs, ctuArea, ChannelType::LUMA), ChannelType::LUMA))
      {
        // Extracting if the current block is an inter coded block, an intra coded block or an IBC block
        // MODE_INTER   = 0,   ///< inter-prediction mode
        // MODE_INTRA   = 1,   ///< intra-prediction mode
        // MODE_IBC     = 2,   ///< ibc-prediction mode
        bool isInter = (currCU.predMode == MODE_INTER) ? true : false;
        bool isIntra = (currCU.predMode == MODE_INTRA) ? true : false;
        bool isIBC   = (currCU.predMode == MODE_IBC) ? true : false;

        // Extracting if the current block is a uni-prediction block
        // interDir     = 1    /// only L0 is used for the current block
        // interDir     = 2    /// only L1 is used for the current block
        // interDir     = 3    /// both L0 and L1 are used for the current block
        bool isUniPred = ((&currCU)->interDir == 1 || (&currCU)->interDir == 2) ? true : false;
        // For some reason the encoder has set currCU.interDir to -1 if the mode is GEO.
        // So we have to check for GEO as well. Every GEO-predicted block is per definition
        // uni-predicted (see JVET-L0124). In the decoder this is not a problem, but for the
        // encoder we need the line below.
        isUniPred      = (isUniPred || currCU.geoFlag);

        // Extracting if the current block is a bi-prediction block
        bool isBiPred = ((&currCU)->interDir == 3 && !currCU.geoFlag) ? true : false;

        Pel toFill = 0;

        if (isIntra == true)   // if an intra block
        {
          toFill = 0;
        }
        if (isIBC == true && currCU.skip == false)   // if a non-skipped IBC block
        {
          toFill = 2;
        }
        if (isIBC == true && currCU.skip == true)   // if a skipped IBC block
        {
          toFill = 3;
        }
        if ((isInter == true && currCU.skip == false) && (isUniPred == true))   // if a non-skipped uni-prediction block
        {
          toFill = 4;
        }
        if ((isInter == true && currCU.skip == true) && (isUniPred == true))   // if a skipped uni-prediction block
        {
          toFill = 5;
        }
        if ((isInter == true && currCU.skip == false) && (isBiPred == true))   // if a non-skipped bi-prediction block
        {
          toFill = 6;
        }
        if ((isInter == true && currCU.skip == true) && (isBiPred == true))   // if a skipped bi-prediction block
        {
          toFill = 7;
        }

        // Filling all three components of the buffer with extracted value
        if (currCU.Y().valid())
        {
          CompArea yArea(CompID::COMP_Y, m_cs->area.chromaFormat, currCU.lx(), currCU.ly(), currCU.lwidth(),
                         currCU.lheight(), true);
          auto     targetBuf = currCU.slice->m_pic->getBlockPredModeBuf(yArea);
          targetBuf.fill(toFill);
        }

        if (isChromaEnabled(pcv.chrFormat) && currCU.Cb().valid())
        {
          CompArea cbArea(CompID::COMP_Cb, m_cs->area.chromaFormat, currCU.chromaPos().x, currCU.chromaPos().y,
                          currCU.chromaSize().width, currCU.chromaSize().height, false);
          auto     targetBuf = currCU.slice->m_pic->getBlockPredModeBuf(cbArea);
          targetBuf.fill(toFill);

          CompArea crArea(CompID::COMP_Cr, m_cs->area.chromaFormat, currCU.chromaPos().x, currCU.chromaPos().y,
                          currCU.chromaSize().width, currCU.chromaSize().height, false);
          targetBuf = currCU.slice->m_pic->getBlockPredModeBuf(crArea);
          targetBuf.fill(toFill);
        }
      }
      if (!pcv.ISingleTree)
      {
        for (auto &currCU: m_cs->traverseCUs(CS::getArea(*m_cs, ctuArea, ChannelType::CHROMA), ChannelType::CHROMA))
        {
          if (isChromaEnabled(pcv.chrFormat) && currCU.Cb().valid())
          {
            // Extracting if the current block is an inter coded block, an intra coded block or an IBC block
            // MODE_INTER   = 0,   ///< inter-prediction mode
            // MODE_INTRA   = 1,   ///< intra-prediction mode
            // MODE_IBC     = 2,   ///< ibc-prediction mode
            bool isInter = (currCU.predMode == MODE_INTER) ? true : false;
            bool isIntra = (currCU.predMode == MODE_INTRA) ? true : false;
            bool isIBC   = (currCU.predMode == MODE_IBC) ? true : false;

            // Extracting if the current block is a uni-prediction block
            // interDir     = 1    /// only L0 is used for the current block
            // interDir     = 2    /// only L1 is used for the current block
            // interDir     = 3    /// both L0 and L1 are used for the current block
            bool isUniPred = ((&currCU)->interDir == 1 || (&currCU)->interDir == 2) ? true : false;
            // For some reason the encoder has set currCU.interDir to -1 if the mode is GEO.
            // So we have to check for GEO as well. Every GEO-predicted block is per definition
            // uni-predicted (see JVET-L0124). In the decoder this is not a problem, but for the
            // encoder we need the line below.
            isUniPred      = (isUniPred || currCU.geoFlag);

            // Extracting if the current block is a bi-prediction block
            bool isBiPred = ((&currCU)->interDir == 3 && !currCU.geoFlag) ? true : false;

            Pel toFill = 0;

            if (isIntra == true)   // if an intra block
            {
              toFill = 0;
            }
            if (isIBC == true && currCU.skip == false)   // if a non-skipped IBC block
            {
              toFill = 2;
            }
            if (isIBC == true && currCU.skip == true)   // if a skipped IBC block
            {
              toFill = 3;
            }
            if ((isInter == true && currCU.skip == false) &&
                (isUniPred == true))   // if a non-skipped uni-prediction block
            {
              toFill = 4;
            }
            if ((isInter == true && currCU.skip == true) && (isUniPred == true))   // if a skipped uni-prediction block
            {
              toFill = 5;
            }
            if ((isInter == true && currCU.skip == false) &&
                (isBiPred == true))   // if a non-skipped bi-prediction block
            {
              toFill = 6;
            }
            if ((isInter == true && currCU.skip == true) && (isBiPred == true))   // if a skipped bi-prediction block
            {
              toFill = 7;
            }

            CompArea cbArea(CompID::COMP_Cb, m_cs->area.chromaFormat, currCU.chromaPos().x, currCU.chromaPos().y,
                            currCU.chromaSize().width, currCU.chromaSize().height, false);
            auto     targetBuf = currCU.slice->m_pic->getBlockPredModeBuf(cbArea);
            targetBuf.fill(toFill);

            CompArea crArea(CompID::COMP_Cr, m_cs->area.chromaFormat, currCU.chromaPos().x, currCU.chromaPos().y,
                            currCU.chromaSize().width, currCU.chromaSize().height, false);
            targetBuf = currCU.slice->m_pic->getBlockPredModeBuf(crArea);
            targetBuf.fill(toFill);
          }
        }
      }
    }
  }
}

void Picture::dumpQpBlock()
{
  const PreCalcValues &pcv = *m_cs->pcv;
  // Defining a scanning area that covers the picture
  //  const UnitArea PICarea(cs.area.chromaFormat, Area(0, 0, pcv.lumaWidth, pcv.lumaHeight));
  // Traversing the CUs in the picture and filling a buffer with extracted info
  for (int y = 0; y < pcv.heightInCtus; y++)
  {
    for (int x = 0; x < pcv.widthInCtus; x++)
    {
      const UnitArea ctuArea(pcv.chrFormat,
                             Area(x << pcv.maxCUWidthLog2, y << pcv.maxCUHeightLog2, pcv.maxCUWidth, pcv.maxCUWidth));
      for (auto &currCU: m_cs->traverseCUs(CS::getArea(*m_cs, ctuArea, ChannelType::LUMA), ChannelType::LUMA))
      {
        if (currCU.Y().valid())
        {
          Pel      toFill = currCU.qp;   // sould also use chroma
          CompArea yArea(CompID::COMP_Y, m_cs->area.chromaFormat, currCU.lx(), currCU.ly(), currCU.lwidth(),
                         currCU.lheight(), true);
          auto     targetBuf = currCU.slice->m_pic->getBlockQpBuf(yArea);
          targetBuf.fill(toFill);
        }
        if (isChromaEnabled(pcv.chrFormat) && currCU.Cb().valid())
        {
          const bool useJQP = (abs(TU::getICTMode(*currCU.firstTU)) == 2);
          for (auto compID: { CompID::COMP_Cb, CompID::COMP_Cr })
          {
            int chromaQpOffset = m_cs->pps->getQpOffset(useJQP ? JOINT_CbCr : compID);
            chromaQpOffset += currCU.slice->getSliceChromaQpDelta(useJQP ? JOINT_CbCr : compID);
            chromaQpOffset +=
              m_cs->pps->getChromaQpOffsetListEntry(currCU.chromaQpAdj).u.offset[int(useJQP ? JOINT_CbCr : compID) - 1];
            int      qp     = currCU.qp;
            int      qpc    = currCU.slice->m_sps->getMappedChromaQpValue(compID, qp) + chromaQpOffset;
            Pel      toFill = qpc;
            CompArea area(compID, m_cs->area.chromaFormat, currCU.chromaPos().x, currCU.chromaPos().y,
                          currCU.chromaSize().width, currCU.chromaSize().height, false);
            auto     targetBuf = currCU.slice->m_pic->getBlockQpBuf(area);
            targetBuf.fill(toFill);
          }
        }
      }
      if (!pcv.ISingleTree)
      {
        for (auto &currCU: m_cs->traverseCUs(CS::getArea(*m_cs, ctuArea, ChannelType::CHROMA), ChannelType::CHROMA))
        {
          if (isChromaEnabled(pcv.chrFormat) && currCU.Cb().valid())
          {
            const bool useJQP = (abs(TU::getICTMode(*currCU.firstTU)) == 2);
            for (auto compID: { CompID::COMP_Cb, CompID::COMP_Cr })
            {
              int chromaQpOffset = m_cs->pps->getQpOffset(useJQP ? JOINT_CbCr : compID);
              chromaQpOffset += currCU.slice->getSliceChromaQpDelta(useJQP ? JOINT_CbCr : compID);
              chromaQpOffset += m_cs->pps->getChromaQpOffsetListEntry(currCU.chromaQpAdj)
                                  .u.offset[int(useJQP ? JOINT_CbCr : compID) - 1];
              int      qp     = currCU.qp;
              int      qpc    = currCU.slice->m_sps->getMappedChromaQpValue(compID, qp) + chromaQpOffset;
              Pel      toFill = qpc;
              CompArea area(compID, m_cs->area.chromaFormat, currCU.chromaPos().x, currCU.chromaPos().y,
                            currCU.chromaSize().width, currCU.chromaSize().height, false);
              auto     targetBuf = currCU.slice->m_pic->getBlockQpBuf(area);
              targetBuf.fill(toFill);
            }
          }
        }
      }
    }
  }
}

void Picture::paddingPicBufBorder(const PictureType picType, const int padSize, const int value)
{
  for (int comp = 0; comp < getNumberValidComponents(m_cs->area.chromaFormat); comp++)
  {
    CompID compID  = CompID(comp);
    PelBuf p       = m_bufs[picType].get(compID);
    Pel   *piTxt   = p.bufAt(0, 0);
    int    xmargin = padSize >> getComponentScaleX(compID, m_cs->area.chromaFormat);
    int    ymargin = padSize >> getComponentScaleY(compID, m_cs->area.chromaFormat);

    Pel *pi = piTxt;
    // do left and right margins
    for (int y = 0; y < p.height; y++)
    {
      for (int x = 0; x < xmargin; x++)
      {
        pi[-xmargin + x] = value;
        pi[p.width + x]  = value;
      }
      pi += p.stride;
    }

    // pi is now the (0,height) (bottom left of image within bigger picture
    pi -= xmargin;
    // pi is now the (-marginX, height), set value all
    for (int x = 0; x < (p.width + (xmargin << 1)); x++)
    {
      pi[x] = value;
    }
    for (int y = 0; y < ymargin - 1; y++)
    {
      ::memcpy(pi + (y + 1) * p.stride, pi, sizeof(Pel) * (p.width + (xmargin << 1)));
    }

    // pi is still (-marginX, height)
    pi -= ((p.height + 1) * p.stride);
    // pi is now (-marginX, -1), set value all
    for (int x = 0; x < (p.width + (xmargin << 1)); x++)
    {
      pi[x] = value;
    }
    for (int y = 0; y < ymargin - 1; y++)
    {
      ::memcpy(pi - (y + 1) * p.stride, pi, sizeof(Pel) * (p.width + (xmargin << 1)));
    }
  }
}

NNLFInferSize Picture::getInferSize(const Slice &slice)
{
  NNLFInferSize sizeId = NNLFInferSize::BASE;
  const SPS    &sps    = *slice.m_sps;
  const PPS    &pps    = *slice.m_pps;

  if (sps.m_nnlf == NNLFUnifiedID::HOP)
  {
    sizeId = NNLFInferSize::BASE;
  }
  else if (slice.isIntra())
  {
    sizeId = NNLFInferSize::LARGE;
  }
  else if (slice.m_iSliceQp >= 29 && pps.m_picWidthInLumaSamples > 832)
  {
    sizeId = NNLFInferSize::LARGE;
  }
  else
  {
    sizeId = NNLFInferSize::BASE;
  }

  return sizeId;
}

void Picture::initPicprms(const Slice &slice)
{
  const int sizeId    = getInferSize(slice);
  m_picprm.block_size = NNLF_UNIFIED_INFER_SIZE[sizeId];
  m_picprm.extension  = NNLF_UNIFIED_INFER_SIZE_EXT;
  m_picprm.prmNum     = NNLF_UNIFIED_MAX_NUM_PRMS;

  const PPS &pps            = *slice.m_pps;
  m_picprm.nb_blocks_height = (pps.m_picHeightInLumaSamples + m_picprm.block_size - 1) / m_picprm.block_size;
  m_picprm.nb_blocks_width  = (pps.m_picWidthInLumaSamples + m_picprm.block_size - 1) / m_picprm.block_size;

  m_picprm.prmId.resize(m_picprm.nb_blocks_height * m_picprm.nb_blocks_width);
  std::fill(m_picprm.prmId.begin(), m_picprm.prmId.end(), -1);

  m_picprm.temporal = false;
}
#endif

void Picture::extendSubPicBorder(int POC, int subPicX0, int subPicY0, int subPicWidth, int subPicHeight)
{

  for (int comp = 0; comp < getNumberValidComponents(m_cs->area.chromaFormat); comp++)
  {
    CompID compID = CompID(comp);

    // 2.1 measure the margin for each component
    int xmargin = margin >> getComponentScaleX(compID, m_cs->area.chromaFormat);
    int ymargin = margin >> getComponentScaleY(compID, m_cs->area.chromaFormat);

    // 2.2 calculate the origin of the Subpicture
    int left = subPicX0 >> getComponentScaleX(compID, m_cs->area.chromaFormat);
    int top  = subPicY0 >> getComponentScaleY(compID, m_cs->area.chromaFormat);

    // 2.3 calculate the width/height of the Subpicture
    int width  = subPicWidth >> getComponentScaleX(compID, m_cs->area.chromaFormat);
    int height = subPicHeight >> getComponentScaleY(compID, m_cs->area.chromaFormat);

    // 3.1 set reconstructed picture
    PelBuf s   = m_bufs[PIC_RECONSTRUCTION].get(compID);
    Pel   *src = s.bufAt(left, top);

    // 4.1 apply padding for left and right
    {
      Pel *dstLeft  = src - xmargin;
      Pel *dstRight = src + width;
      Pel *srcLeft  = src + 0;
      Pel *srcRight = src + width - 1;

      for (int y = 0; y < height; y++)
      {
        for (int x = 0; x < xmargin; x++)
        {
          dstLeft[x]  = *srcLeft;
          dstRight[x] = *srcRight;
        }
        dstLeft += s.stride;
        dstRight += s.stride;
        srcLeft += s.stride;
        srcRight += s.stride;
      }
    }

    // 4.2 apply padding on bottom
    Pel *srcBottom = src + s.stride * (height - 1) - xmargin;
    Pel *dstBottom = srcBottom + s.stride;
    for (int y = 0; y < ymargin; y++)
    {
      ::memcpy(dstBottom, srcBottom, sizeof(Pel) * (2 * xmargin + width));
      dstBottom += s.stride;
    }

    // 4.3 apply padding for top
    // si is still (-marginX, SubpictureHeight-1)
    Pel *srcTop = src - xmargin;
    Pel *dstTop = srcTop - s.stride;
    // si is now (-marginX, 0)
    for (int y = 0; y < ymargin; y++)
    {
      ::memcpy(dstTop, srcTop, sizeof(Pel) * (2 * xmargin + width));
      dstTop -= s.stride;
    }

    // Appy padding for recon wrap buffer
    if (m_cs->sps->m_wrapAroundEnabledFlag)
    {
      // set recon wrap picture
      PelBuf sWrap   = m_bufs[PIC_RECON_WRAP].get(compID);
      Pel   *srcWrap = sWrap.bufAt(left, top);

      // apply padding on bottom
      Pel *srcBottomWrap = srcWrap + sWrap.stride * (height - 1) - xmargin;
      Pel *dstBottomWrap = srcBottomWrap + sWrap.stride;
      for (int y = 0; y < ymargin; y++)
      {
        ::memcpy(dstBottomWrap, srcBottomWrap, sizeof(Pel) * (2 * xmargin + width));
        dstBottomWrap += sWrap.stride;
      }

      // apply padding for top
      // si is still (-marginX, SubpictureHeight-1)
      Pel *srcTopWrap = srcWrap - xmargin;
      Pel *dstTopWrap = srcTopWrap - sWrap.stride;
      // si is now (-marginX, 0)
      for (int y = 0; y < ymargin; y++)
      {
        ::memcpy(dstTopWrap, srcTopWrap, sizeof(Pel) * (2 * xmargin + width));
        dstTopWrap -= sWrap.stride;
      }
    }
  }   // end of for
}

void Picture::restoreSubPicBorder(int POC, int subPicX0, int subPicY0, int subPicWidth, int subPicHeight)
{
  for (int comp = 0; comp < getNumberValidComponents(m_cs->area.chromaFormat); comp++)
  {
    CompID compID = CompID(comp);

    // 2.1 measure the margin for each component
    int xmargin = margin >> getComponentScaleX(compID, m_cs->area.chromaFormat);
    int ymargin = margin >> getComponentScaleY(compID, m_cs->area.chromaFormat);

    // 2.2 calculate the origin of the subpicture
    int left = subPicX0 >> getComponentScaleX(compID, m_cs->area.chromaFormat);
    int top  = subPicY0 >> getComponentScaleY(compID, m_cs->area.chromaFormat);

    // 2.3 calculate the width/height of the subpicture
    int width  = subPicWidth >> getComponentScaleX(compID, m_cs->area.chromaFormat);
    int height = subPicHeight >> getComponentScaleY(compID, m_cs->area.chromaFormat);

    // 3.1 set reconstructed picture
    PelBuf s   = m_bufs[PIC_RECONSTRUCTION].get(compID);
    Pel   *src = s.bufAt(left, top);

    // 4.2.1 copy from back up buffer to recon picture
    PelBuf dBufLeft = m_bufSubPicLeft.getBuf(compID);
    Pel   *dstLeft  = dBufLeft.bufAt(0, 0);

    // 4.2.2 set back up buffer for right
    PelBuf dBufRight = m_bufSubPicRight.getBuf(compID);
    Pel   *dstRight  = dBufRight.bufAt(0, 0);

    // 4.2.3 copy to recon picture to back up buffer
    Pel *srcLeft  = src - xmargin;
    Pel *srcRight = src + width;

    for (int y = 0; y < height; y++)
    {
      // the destination and source position is reversed on purpose
      ::memcpy(srcLeft + y * s.stride, dstLeft + y * dBufLeft.stride, sizeof(Pel) * xmargin);
      ::memcpy(srcRight + y * s.stride, dstRight + y * dBufRight.stride, sizeof(Pel) * xmargin);
    }

    // 4.3.1 set back up buffer for above
    PelBuf dBufTop = m_bufSubPicAbove.getBuf(compID);
    Pel   *dstTop  = dBufTop.bufAt(0, 0);

    // 4.3.2 set back up buffer for below
    PelBuf dBufBottom = m_bufSubPicBelow.getBuf(compID);
    Pel   *dstBottom  = dBufBottom.bufAt(0, 0);

    // 4.3.3 copy to recon picture to back up buffer
    Pel *srcTop    = src - xmargin - ymargin * s.stride;
    Pel *srcBottom = src - xmargin + height * s.stride;

    for (int y = 0; y < ymargin; y++)
    {
      ::memcpy(srcTop + y * s.stride, dstTop + y * dBufTop.stride, sizeof(Pel) * (2 * xmargin + width));
      ::memcpy(srcBottom + y * s.stride, dstBottom + y * dBufBottom.stride, sizeof(Pel) * (2 * xmargin + width));
    }

    // restore recon wrap buffer
    if (m_cs->sps->m_wrapAroundEnabledFlag)
    {
      // set recon wrap picture
      PelBuf sWrap   = m_bufs[PIC_RECON_WRAP].get(compID);
      Pel   *srcWrap = sWrap.bufAt(left, top);

      // set back up buffer for above
      PelBuf dBufTopWrap = m_bufWrapSubPicAbove.getBuf(compID);
      Pel   *dstTopWrap  = dBufTopWrap.bufAt(0, 0);

      // set back up buffer for below
      PelBuf dBufBottomWrap = m_bufWrapSubPicBelow.getBuf(compID);
      Pel   *dstBottomWrap  = dBufBottomWrap.bufAt(0, 0);

      // copy to recon wrap picture from back up buffer
      Pel *srcTopWrap    = srcWrap - xmargin - ymargin * sWrap.stride;
      Pel *srcBottomWrap = srcWrap - xmargin + height * sWrap.stride;

      for (int y = 0; y < ymargin; y++)
      {
        ::memcpy(srcTopWrap + y * sWrap.stride, dstTopWrap + y * dBufTopWrap.stride,
                 sizeof(Pel) * (2 * xmargin + width));
        ::memcpy(srcBottomWrap + y * sWrap.stride, dstBottomWrap + y * dBufBottomWrap.stride,
                 sizeof(Pel) * (2 * xmargin + width));
      }
    }
  }

  // 5.0 destroy the back up memory
  m_bufSubPicAbove.destroy();
  m_bufSubPicBelow.destroy();
  m_bufSubPicLeft.destroy();
  m_bufSubPicRight.destroy();
  m_bufWrapSubPicAbove.destroy();
  m_bufWrapSubPicBelow.destroy();
}

#define TemplateMatchingPaddingTop    TemplateMatchingPadding<boundaryDirection::BD_TOP>
#define TemplateMatchingPaddingBottom TemplateMatchingPadding<boundaryDirection::BD_BOTTOM>
#define TemplateMatchingPaddingLeft   TemplateMatchingPadding<boundaryDirection::BD_LEFT>
#define TemplateMatchingPaddingRight  TemplateMatchingPadding<boundaryDirection::BD_RIGHT>

void Picture::extendPicBorder(const PPS *pps)
{
  if (m_extendedBorder)
  {
    if (isWrapAroundEnabled(pps) && (!m_wrapAroundValid || m_wrapAroundOffset != pps->m_wrapAroundOffset))
    {
      extendWrapBorder(pps);
    }
    return;
  }

  if (m_cs->sps->m_TMBP)
  {

    int picWidth  = m_cs->pps->m_picWidthInLumaSamples;
    int picHeight = m_cs->pps->m_picHeightInLumaSamples;

    Area picAreaTopBottom = Area(Position(0, 0), Size(picWidth, picHeight));
    TemplateMatchingPaddingTop(picAreaTopBottom);
    TemplateMatchingPaddingBottom(picAreaTopBottom);

    Area picAreaLeftRight = Area(Position(0, -TMP_PADSIZE), Size(picWidth, picHeight + 2 * TMP_PADSIZE));
    TemplateMatchingPaddingLeft(picAreaLeftRight);
    TemplateMatchingPaddingRight(picAreaLeftRight);
  }

  for (int comp = 0; comp < getNumberValidComponents(m_cs->area.chromaFormat); comp++)
  {
    CompID compID = CompID(comp);
    PelBuf p      = m_bufs[PIC_RECONSTRUCTION].get(compID);

    int  width;
    int  height;
    int  xmargin;
    int  ymargin;
    Pel *piTxt;
    if (m_cs->sps->m_TMBP)
    {
      width  = p.width + ((2 * TMP_PADSIZE) >> getComponentScaleX(compID, m_cs->area.chromaFormat));
      height = p.height + ((2 * TMP_PADSIZE) >> getComponentScaleY(compID, m_cs->area.chromaFormat));

      const int offsetX = TMP_PADSIZE >> getComponentScaleX(compID, m_cs->area.chromaFormat);
      const int offsetY = TMP_PADSIZE >> getComponentScaleY(compID, m_cs->area.chromaFormat);
      piTxt             = p.bufAt(-offsetX, -offsetY);

      xmargin = (margin - TMP_PADSIZE) >> getComponentScaleX(compID, m_cs->area.chromaFormat);
      ymargin = (margin - TMP_PADSIZE) >> getComponentScaleY(compID, m_cs->area.chromaFormat);
    }
    else
    {
      width  = p.width;
      height = p.height;

      piTxt   = p.bufAt(0, 0);
      xmargin = margin >> getComponentScaleX(compID, m_cs->area.chromaFormat);
      ymargin = margin >> getComponentScaleY(compID, m_cs->area.chromaFormat);
    }

    Pel *pi = piTxt;
    // do left and right margins
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < xmargin; x++)
      {
        pi[-xmargin + x] = pi[0];
        pi[width + x]    = pi[width - 1];
      }
      pi += p.stride;
    }

    // pi is now the (0,height) (bottom left of image within bigger picture
    pi -= (p.stride + xmargin);
    // pi is now the (-marginX, height-1)
    for (int y = 0; y < ymargin; y++)
    {
      ::memcpy(pi + (y + 1) * p.stride, pi, sizeof(Pel) * (width + (xmargin << 1)));
    }

    // pi is still (-marginX, height-1)
    pi -= ((height - 1) * p.stride);
    // pi is now (-marginX, 0)
    for (int y = 0; y < ymargin; y++)
    {
      ::memcpy(pi - (y + 1) * p.stride, pi, sizeof(Pel) * (width + (xmargin << 1)));
    }

    // reference picture with horizontal wrapped boundary
    if (isWrapAroundEnabled(pps))
    {
      extendWrapBorder(pps);
    }
    else
    {
      m_wrapAroundValid  = false;
      m_wrapAroundOffset = 0;
    }
  }

  m_extendedBorder = true;
}

void Picture::extendWrapBorder(const PPS *pps)
{
  for (int comp = 0; comp < getNumberValidComponents(m_cs->area.chromaFormat); comp++)
  {
    CompID compID = CompID(comp);
    PelBuf p      = m_bufs[PIC_RECON_WRAP].get(compID);
    p.copyFrom(m_bufs[PIC_RECONSTRUCTION].get(compID));
    Pel *piTxt   = p.bufAt(0, 0);
    int  xmargin = margin >> getComponentScaleX(compID, m_cs->area.chromaFormat);
    int  ymargin = margin >> getComponentScaleY(compID, m_cs->area.chromaFormat);
    Pel *pi      = piTxt;
    int  xoffset = pps->m_wrapAroundOffset >> getComponentScaleX(compID, m_cs->area.chromaFormat);
    for (int y = 0; y < p.height; y++)
    {
      for (int x = 0; x < xmargin; x++)
      {
        if (x < xoffset)
        {
          pi[-x - 1]      = pi[-x - 1 + xoffset];
          pi[p.width + x] = pi[p.width + x - xoffset];
        }
        else
        {
          pi[-x - 1]      = pi[0];
          pi[p.width + x] = pi[p.width - 1];
        }
      }
      pi += p.stride;
    }
    pi -= (p.stride + xmargin);
    for (int y = 0; y < ymargin; y++)
    {
      ::memcpy(pi + (y + 1) * p.stride, pi, sizeof(Pel) * (p.width + (xmargin << 1)));
    }
    pi -= ((p.height - 1) * p.stride);
    for (int y = 0; y < ymargin; y++)
    {
      ::memcpy(pi - (y + 1) * p.stride, pi, sizeof(Pel) * (p.width + (xmargin << 1)));
    }
  }
  m_wrapAroundValid  = true;
  m_wrapAroundOffset = pps->m_wrapAroundOffset;
}

PelBuf Picture::getBuf(const CompID compID, const PictureType &type) { return m_bufs[type].getBuf(compID); }

const CPelBuf Picture::getBuf(const CompID compID, const PictureType &type) const
{
  return m_bufs[type].getBuf(compID);
}

PelBuf Picture::getBuf(const CompArea &blk, const PictureType &type)
{
  if (!blk.valid())
  {
    return PelBuf();
  }

#if !KEEP_PRED_AND_RESI_SIGNALS
  if (type == PIC_RESIDUAL || type == PIC_PREDICTION)
  {
    CompArea localBlk = blk;
    localBlk.x &= (m_cs->pcv->maxCUWidthMask >> getComponentScaleX(blk.compID, blk.chromaFormat));
    localBlk.y &= (m_cs->pcv->maxCUHeightMask >> getComponentScaleY(blk.compID, blk.chromaFormat));

    return m_bufs[type].getBuf(localBlk);
  }
#endif

  return m_bufs[type].getBuf(blk);
}

const CPelBuf Picture::getBuf(const CompArea &blk, const PictureType &type) const
{
  if (!blk.valid())
  {
    return PelBuf();
  }

#if !KEEP_PRED_AND_RESI_SIGNALS
  if (type == PIC_RESIDUAL || type == PIC_PREDICTION)
  {
    CompArea localBlk = blk;
    localBlk.x &= (m_cs->pcv->maxCUWidthMask >> getComponentScaleX(blk.compID, blk.chromaFormat));
    localBlk.y &= (m_cs->pcv->maxCUHeightMask >> getComponentScaleY(blk.compID, blk.chromaFormat));

    return m_bufs[type].getBuf(localBlk);
  }
#endif

  return m_bufs[type].getBuf(blk);
}

PelUnitBuf Picture::getBuf(const UnitArea &unit, const PictureType &type)
{
  if (!isChromaEnabled(chromaFormat))
  {
    return PelUnitBuf(chromaFormat, getBuf(unit.Y(), type));
  }
  else
  {
    return PelUnitBuf(chromaFormat, getBuf(unit.Y(), type), getBuf(unit.Cb(), type), getBuf(unit.Cr(), type));
  }
}

const CPelUnitBuf Picture::getBuf(const UnitArea &unit, const PictureType &type) const
{
  if (!isChromaEnabled(chromaFormat))
  {
    return CPelUnitBuf(chromaFormat, getBuf(unit.Y(), type));
  }
  else
  {
    return CPelUnitBuf(chromaFormat, getBuf(unit.Y(), type), getBuf(unit.Cb(), type), getBuf(unit.Cr(), type));
  }
}

Pel *Picture::getOrigin(const PictureType &type, const CompID compID) const { return m_bufs[type].getOrigin(compID); }

void Picture::createSpliceIdx(int nums)
{
  m_ctuNums   = nums;
  m_spliceIdx = new int[m_ctuNums];
  memset(m_spliceIdx, 0, m_ctuNums * sizeof(int));
}

bool Picture::getSpliceFull()
{
  int count = 0;
  for (int i = 0; i < m_ctuNums; i++)
  {
    if (m_spliceIdx[i] != 0)
    {
      count++;
    }
  }
  if (count < m_ctuNums * 0.25)
  {
    return false;
  }
  return true;
}

void Picture::addPictureToHashMapForInter()
{
  int       picWidth  = m_slices[0]->m_pps->m_picWidthInLumaSamples;
  int       picHeight = m_slices[0]->m_pps->m_picHeightInLumaSamples;
  uint32_t *blockHashValues[2][2];
  bool     *isBlockSame[2][3];

  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      blockHashValues[i][j] = new uint32_t[picWidth * picHeight];
    }

    for (int j = 0; j < 3; j++)
    {
      isBlockSame[i][j] = new bool[picWidth * picHeight];
    }
  }
  m_hashMap.create(picWidth, picHeight);
  m_hashMap.generateBlock2x2HashValue(getOrigBuf(), picWidth, picHeight, m_slices[0]->m_sps->m_bitDepths,
                                      blockHashValues[0], isBlockSame[0]);   // 2x2
  m_hashMap.generateBlockHashValue(picWidth, picHeight, 4, 4, blockHashValues[0], blockHashValues[1], isBlockSame[0],
                                   isBlockSame[1]);   // 4x4
  m_hashMap.addToHashMapByRowWithPrecalData(blockHashValues[1], isBlockSame[1][2], picWidth, picHeight, 4, 4);

  m_hashMap.generateBlockHashValue(picWidth, picHeight, 8, 8, blockHashValues[1], blockHashValues[0], isBlockSame[1],
                                   isBlockSame[0]);   // 8x8
  m_hashMap.addToHashMapByRowWithPrecalData(blockHashValues[0], isBlockSame[0][2], picWidth, picHeight, 8, 8);

  m_hashMap.generateBlockHashValue(picWidth, picHeight, 16, 16, blockHashValues[0], blockHashValues[1], isBlockSame[0],
                                   isBlockSame[1]);   // 16x16
  m_hashMap.addToHashMapByRowWithPrecalData(blockHashValues[1], isBlockSame[1][2], picWidth, picHeight, 16, 16);

  m_hashMap.generateBlockHashValue(picWidth, picHeight, 32, 32, blockHashValues[1], blockHashValues[0], isBlockSame[1],
                                   isBlockSame[0]);   // 32x32
  m_hashMap.addToHashMapByRowWithPrecalData(blockHashValues[0], isBlockSame[0][2], picWidth, picHeight, 32, 32);

  m_hashMap.generateBlockHashValue(picWidth, picHeight, 64, 64, blockHashValues[0], blockHashValues[1], isBlockSame[0],
                                   isBlockSame[1]);   // 64x64
  m_hashMap.addToHashMapByRowWithPrecalData(blockHashValues[1], isBlockSame[1][2], picWidth, picHeight, 64, 64);

  m_hashMap.setInitial();

  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      delete[] blockHashValues[i][j];
    }

    for (int j = 0; j < 3; j++)
    {
      delete[] isBlockSame[i][j];
    }
  }
}

void Picture::calcLumaClpParams()
{
  int pelMax    = m_lumaClpRng.max;
  int pelMin    = m_lumaClpRng.min;
  int targetMin = 16 * (1 << (m_cs->sps->m_bitDepths[ChannelType::LUMA] - 8));
  int targetMax = 235 * (1 << (m_cs->sps->m_bitDepths[ChannelType::LUMA] - 8));
  if (!m_cs->slice->isIntra())
  {
    const Picture *const pColPic =
      m_cs->slice->getRefPic(RefPicList(1 - m_cs->slice->m_colFromL0Flag), m_cs->slice->m_colRefIdx)->m_unscaledPic;
    ClpRng colLumaClpRng = pColPic->m_lumaClpRng;
    targetMin            = colLumaClpRng.min;
    targetMax            = colLumaClpRng.max;
  }

  m_cs->slice->m_adaptiveClipQuant = !m_cs->slice->isIntra() && m_cs->slice->m_checkLdc;
  int clipDeltaShift               = m_cs->slice->getClipDeltaShift();

  int       pelMaxOF  = 0;
  int       pelMinOF  = (1 << m_cs->sps->m_bitDepths[ChannelType::LUMA]) - 1;
  const int orgPelMin = pelMin;
  {
    int deltaMinToSignal = (pelMin - targetMin);
    if (deltaMinToSignal < 0)
    {
      int absDelta = ((targetMin - pelMin) >> clipDeltaShift) << clipDeltaShift;
      pelMin       = targetMin - absDelta;
      while (pelMin > orgPelMin)
      {
        pelMin -= (1 << clipDeltaShift);
      }
      while (pelMin < 0)
      {
        pelMinOF = pelMin;
        pelMin   = 0;
      }
      CHECK(pelMin < 0, "this is not possible");
    }
    else if (deltaMinToSignal > 0)
    {
      int absDelta = (deltaMinToSignal >> clipDeltaShift) << clipDeltaShift;
      pelMin       = targetMin + absDelta;
      CHECK(pelMin > orgPelMin, "this is not possible");
      CHECK(pelMin < 0, "this is not possible");
    }
    else
    {
      CHECK(pelMin != targetMin, "this is not possible");
    }
  }

  const int orgPelMax = pelMax;
  {
    int deltaMaxToSignal = (pelMax - targetMax);
    if (deltaMaxToSignal < 0)
    {
      int absDelta = ((targetMax - pelMax) >> clipDeltaShift) << clipDeltaShift;
      pelMax       = targetMax - absDelta;
      CHECK(pelMax < orgPelMax, "this is not possible");
      CHECK(pelMax > (1 << m_cs->sps->m_bitDepths[ChannelType::LUMA]) - 1, "this is not possible");
    }
    else if (deltaMaxToSignal > 0)
    {
      int absDelta = (deltaMaxToSignal >> clipDeltaShift) << clipDeltaShift;
      pelMax       = targetMax + absDelta;
      while (pelMax < orgPelMax)
      {
        pelMax += (1 << clipDeltaShift);
      }
      while (pelMax >= (1 << m_cs->sps->m_bitDepths[ChannelType::LUMA]))
      {
        pelMaxOF = pelMax;
        pelMax   = (1 << m_cs->sps->m_bitDepths[ChannelType::LUMA]) - 1;
      }
      CHECK(pelMax > (1 << m_cs->sps->m_bitDepths[ChannelType::LUMA]) - 1, "this is not possible");
    }
    else
    {
      CHECK(pelMax != targetMax, "this is not possible");
    }
  }
  m_cs->slice->m_lumaPelMax = pelMax;
  m_cs->slice->m_lumaPelMin = pelMin;
  m_lumaClpRng.min          = pelMin;
  m_lumaClpRng.max          = pelMax;
  m_lumaClpRngforQuant.min  = std::min(pelMin, pelMinOF);
  m_lumaClpRngforQuant.max  = std::max(pelMax, pelMaxOF);
}

void Picture::createGrainSynthesizer(bool firstPictureInSequence, SEIFilmGrainSynthesizer *grainCharacteristics,
                                     PelStorage *grainBuf, int width, int height, ChromaFormat fmt, int bitDepth)
{
  m_grainCharacteristic = grainCharacteristics;
  m_grainBuf            = grainBuf;

  // Padding to make wd and ht multiple of max fgs window size(64)
  int paddedWdFGS = ((width - 1) | 0x3F) + 1 - width;
  int paddedHtFGS = ((height - 1) | 0x3F) + 1 - height;
  m_padValue      = (paddedWdFGS > paddedHtFGS) ? paddedWdFGS : paddedHtFGS;

  if (firstPictureInSequence)
  {
    // Create and initialize the Film Grain Synthesizer
    m_grainCharacteristic->create(width, height, fmt, bitDepth, 1);

    // Frame level PelStorage buffer created to blend Film Grain Noise into it
    m_grainBuf->create(chromaFormat, Area(0, 0, width, height), 0, m_padValue, 0, false);

    m_grainCharacteristic->fgsInit();
  }
}

PelUnitBuf Picture::getDisplayBufFG(bool wrap)
{
  SEI::PayloadType           payloadType;
  std::list<SEI *>::iterator message;

  for (message = m_SEIs.begin(); message != m_SEIs.end(); ++message)
  {
    payloadType = (*message)->payloadType();
    if (payloadType == SEI::PayloadType::FILM_GRAIN_CHARACTERISTICS)
    {
      m_grainCharacteristic->m_errorCode      = -1;
      *m_grainCharacteristic->m_fgcParameters = *static_cast<SEIFilmGrainCharacteristics *>(*message);
      /* Validation of Film grain characteristic parameters for the constrains of SMPTE-RDD5*/
      m_grainCharacteristic->m_errorCode      = m_grainCharacteristic->grainValidateParams();
      break;
    }
  }

  if (FGS_SUCCESS == m_grainCharacteristic->m_errorCode)
  {
    m_grainBuf->copyFrom(getRecoBuf());
    m_grainBuf->extendBorderPel(m_padValue);   // Padding to make wd and ht multiple of max fgs window size(64)

    m_grainCharacteristic->m_poc = m_poc;
    m_grainCharacteristic->grainSynthesizeAndBlend(m_grainBuf, m_slices[0]->getIdrPicFlag());

    return *m_grainBuf;
  }
  else
  {
    if (payloadType == SEI::PayloadType::FILM_GRAIN_CHARACTERISTICS)
    {
      msg(WARNING, "Film Grain synthesis is not performed. Error code: 0x%x \n", m_grainCharacteristic->m_errorCode);
    }
    return m_bufs[wrap ? PIC_RECON_WRAP : PIC_RECONSTRUCTION];
  }
}

void Picture::createColourTransfProcessor(bool firstPictureInSequence, SEIColourTransformApply *ctiCharacteristics,
                                          PelStorage *ctiBuf, int width, int height, ChromaFormat fmt, int bitDepth)
{
  m_colourTranfParams  = ctiCharacteristics;
  m_invColourTransfBuf = ctiBuf;
  if (firstPictureInSequence)
  {
    // Create and initialize the Colour Transform Processor
    m_colourTranfParams->create(width, height, fmt, bitDepth);

    // Frame level PelStorage buffer created to apply the Colour Transform
    m_invColourTransfBuf->create(UnitArea(chromaFormat, Area(0, 0, width, height)));
  }
}

PelUnitBuf Picture::getDisplayBuf()
{
  SEI::PayloadType           payloadType;
  std::list<SEI *>::iterator message;

  for (message = m_SEIs.begin(); message != m_SEIs.end(); ++message)
  {
    payloadType = (*message)->payloadType();
    if (payloadType == SEI::PayloadType::COLOUR_TRANSFORM_INFO)
    {
      // re-init parameters
      *m_colourTranfParams->m_pColourTransfParams = *static_cast<SEIColourTransformInfo *>(*message);
      // m_colourTranfParams->m_pColourTransfParams = static_cast<SEIColourTransformInfo*>(*message);
      break;
    }
  }

  m_invColourTransfBuf->copyFrom(getRecoBuf());

  if (m_colourTranfParams->m_pColourTransfParams != nullptr)
  {
    m_colourTranfParams->generateColourTransfLUTs();
    m_colourTranfParams->inverseColourTransform(m_invColourTransfBuf);
  }

  return *m_invColourTransfBuf;
}

#if JVET_Z0120_SII_SEI_PROCESSING
void Picture::copyToPic(const SPS *sps, PelStorage *picYuvSrc, PelStorage *picYuvDst)
{
  const ChromaFormat chromaFormatIdc    = sps->m_chromaFormatIdc;
  int                numValidComponents = getNumberValidComponents(chromaFormatIdc);

  Pel      *srcPxl, *dstPxl;
  ptrdiff_t srcStride;
  int       srcHeight, srcWidth;
  ptrdiff_t dstStride;

  for (int comp = 0; comp < numValidComponents; comp++)
  {
    if (comp == COMP_Y)
    {
      srcPxl    = picYuvSrc->Y().buf;
      dstPxl    = picYuvDst->Y().buf;
      srcStride = picYuvSrc->Y().stride;
      srcHeight = picYuvSrc->Y().height;
      srcWidth  = picYuvSrc->Y().width;
      dstStride = picYuvSrc->Y().stride;
    }
    else if (comp == COMP_Cb)
    {
      srcPxl    = picYuvSrc->Cb().buf;
      dstPxl    = picYuvDst->Cb().buf;
      srcStride = picYuvSrc->Cb().stride;
      srcHeight = picYuvSrc->Cb().height;
      srcWidth  = picYuvSrc->Cb().width;
      dstStride = picYuvSrc->Cb().stride;
    }
    else
    {
      srcPxl    = picYuvSrc->Cr().buf;
      dstPxl    = picYuvDst->Cr().buf;
      srcStride = picYuvSrc->Cr().stride;
      srcHeight = picYuvSrc->Cr().height;
      srcWidth  = picYuvSrc->Cr().width;
      dstStride = picYuvSrc->Cr().stride;
    }

    if (srcStride == dstStride)
    {
      ::memcpy(dstPxl, srcPxl, sizeof(Pel) * srcStride * srcHeight /*getTotalHeight(compId)*/);
    }
    else
    {
      for (int y = 0; y < srcHeight; y++, srcPxl += srcStride, dstPxl += dstStride)
      {
        ::memcpy(dstPxl, srcPxl, srcWidth * sizeof(Pel));
      }
    }
  }
}

Picture *Picture::findNextPicPOC(Picture *pic, PicList *picList)
{
  Picture          *nextPic     = nullptr;
  Picture          *listPic     = nullptr;
  PicList::iterator iterListPic = picList->begin();
  for (int i = 0; i < (int)(picList->size()); i++)
  {
    listPic = *(iterListPic);
    if (listPic->m_poc == pic->m_poc + 1)
    {
      nextPic = *(iterListPic);
    }
    iterListPic++;
  }
  return nextPic;
}

Picture *Picture::findPrevPicPOC(Picture *pic, PicList *picList)
{
  Picture          *prevPic     = nullptr;
  Picture          *listPic     = nullptr;
  PicList::iterator iterListPic = picList->begin();
  for (int i = 0; i < (int)(picList->size()); i++)
  {
    listPic = *(iterListPic);
    if (listPic->m_poc == pic->m_poc - 1)
    {
      prevPic = *(iterListPic);
    }
    iterListPic++;
  }
  return prevPic;
}

void Picture::xOutputPostFilteredPic(Picture *pic, PicList *picList, int blendingRatio)
{
  const SPS         *sps             = pic->m_cs->sps;
  const ChromaFormat chromaFormatIdc = sps->m_chromaFormatIdc;

  if ((pic->m_poc) % blendingRatio != 0 || pic->m_poc == 0)
  {
    pic->getPostRecBuf().copyFrom(pic->getRecoBuf());
  }

  if ((pic->m_poc + 1) % blendingRatio == 0)
  {
    Picture *nextPic = findNextPicPOC(pic, picList);
    if (nextPic)
    {
#if DISABLE_PRE_POST_FILTER_FOR_IDR_CRA
      if ((nextPic->m_pictureType == NAL_UNIT_CODED_SLICE_IDR_W_RADL) ||
          (nextPic->m_pictureType == NAL_UNIT_CODED_SLICE_IDR_N_LP) ||
          (nextPic->m_pictureType == NAL_UNIT_CODED_SLICE_CRA))
      {
        nextPic->getPostRecBuf().copyFrom(nextPic->getRecoBuf());
        return;
      }
#endif
      PelUnitBuf currTmp = pic->getRecoBuf();
      PelUnitBuf nextTmp = nextPic->getRecoBuf();
      PelUnitBuf postTmp = nextPic->getPostRecBuf();

      PelUnitBuf *currYuv = &currTmp;
      PelUnitBuf *nextYuv = &nextTmp;
      PelUnitBuf *postYuv = &postTmp;

      int numValidComponents = getNumberValidComponents(chromaFormatIdc);
      for (int chan = 0; chan < numValidComponents; chan++)
      {
        const CompID      ch             = CompID(chan);
        const ChannelType cType          = (ch == COMP_Y) ? ChannelType::LUMA : ChannelType::CHROMA;
        const int         bitDepth       = pic->m_cs->sps->m_bitDepths[cType];
        const int         maxOutputValue = (1 << bitDepth) - 1;

        Pel      *currPxl, *nextPxl, *postPxl;
        ptrdiff_t stride;
        int       height, width;
        if (ch == COMP_Y)
        {
          currPxl = currYuv->Y().buf;
          nextPxl = nextYuv->Y().buf;
          postPxl = postYuv->Y().buf;
          stride  = currYuv->Y().stride;
          height  = currYuv->Y().height;
          width   = currYuv->Y().width;
        }
        else if (ch == COMP_Cb)
        {
          nextPxl = nextYuv->Cb().buf;
          currPxl = currYuv->Cb().buf;
          postPxl = postYuv->Cb().buf;
          stride  = currYuv->Cb().stride;
          height  = currYuv->Cb().height;
          width   = currYuv->Cb().width;
        }
        else
        {
          nextPxl = nextYuv->Cr().buf;
          currPxl = currYuv->Cr().buf;
          postPxl = postYuv->Cr().buf;
          stride  = currYuv->Cr().stride;
          height  = currYuv->Cr().height;
          width   = currYuv->Cr().width;
        }
        for (int y = 0; y < height; y++)
        {
          for (int x = 0; x < width; x++)
          {
#if ENABLE_USER_DEFINED_WEIGHTS
            postPxl[x] = std::min(
              maxOutputValue, std::max(0, (int)(((nextPxl[x]) / SII_PF_W2) - ((currPxl[x] * SII_PF_W1) / SII_PF_W2))));
#else
            postPxl[x] = std::min(
              maxOutputValue,
              std::max(0, (((nextPxl[x] * (blendingRatio + 1)) / blendingRatio) - (currPxl[x] / blendingRatio))));
#endif
          }
          currPxl += stride;
          nextPxl += stride;
          postPxl += stride;
        }
      }
    }
  }
}

void Picture::xOutputPreFilteredPic(Picture *pic, PicList *picList, int blendingRatio, int intraPeriod)
{
  const SPS         *sps             = pic->m_cs->sps;
  const ChromaFormat chromaFormatIdc = sps->m_chromaFormatIdc;
#if DISABLE_PRE_POST_FILTER_FOR_IDR_CRA
  if (pic->m_poc == 0 || (pic->m_poc % intraPeriod == 0))
  {
    return;
  }
#endif
  if (pic->m_poc % blendingRatio == 0)
  {
    Picture *prevPic = findPrevPicPOC(pic, picList);
    if (prevPic)
    {
      PelStorage *currYuv            = &pic->m_bufs[PIC_ORIGINAL];
      PelStorage *prevYuv            = &prevPic->m_bufs[PIC_ORIGINAL];
      const int   numValidComponents = getNumberValidComponents(chromaFormatIdc);
      for (int chan = 0; chan < numValidComponents; chan++)
      {
        const CompID      ch             = CompID(chan);
        const ChannelType cType          = toChannelType(ch);
        const int         bitDepth       = pic->m_cs->sps->m_bitDepths[cType];
        const int         maxOutputValue = (1 << bitDepth) - 1;

        Pel      *currPxl, *prevPxl;
        ptrdiff_t stride;
        int       height, width;
        if (ch == COMP_Y)
        {
          currPxl = currYuv->Y().buf;
          prevPxl = prevYuv->Y().buf;
          stride  = currYuv->Y().stride;
          height  = currYuv->Y().height;
          width   = currYuv->Y().width;
        }
        else if (ch == COMP_Cb)
        {
          prevPxl = prevYuv->Cb().buf;
          currPxl = currYuv->Cb().buf;
          stride  = currYuv->Cb().stride;
          height  = currYuv->Cb().height;
          width   = currYuv->Cb().width;
        }
        else
        {
          prevPxl = prevYuv->Cr().buf;
          currPxl = currYuv->Cr().buf;
          stride  = currYuv->Cr().stride;
          height  = currYuv->Cr().height;
          width   = currYuv->Cr().width;
        }

        for (int y = 0; y < height; y++)
        {
          for (int x = 0; x < width; x++)
          {
#if ENABLE_USER_DEFINED_WEIGHTS
            currPxl[x] = std::min(maxOutputValue, std::max(0, (int)((currPxl[x] * SII_PF_W2) + (prevPxl[x] * SII_PF_W1));
#else
            currPxl[x] = std::min(
              maxOutputValue,
              std::max(0, (((currPxl[x] * blendingRatio) / (blendingRatio + 1)) + (prevPxl[x] / (blendingRatio + 1)))));
#endif
          }
          currPxl += stride;
          prevPxl += stride;
        }
      }
    }
  }
}
#endif

void Picture::copyAlfData(const Picture &p)
{
  for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
  {
    CHECK(p.m_alfModes[compIdx].size() != m_alfModes[compIdx].size(), "Size mismatch");

    std::copy(p.m_alfModes[compIdx].begin(), p.m_alfModes[compIdx].end(), m_alfModes[compIdx].begin());
  }
}

void Picture::resizeAlfData(const int numEntries)
{
  m_alfModes.resize(numEntries);
  std::for_each(m_alfModes.begin(), m_alfModes.end(),
                [](AlfParameters::CtbModes &compModes)
                {
                  std::for_each(compModes.begin(), compModes.end(),
                                [](int &ctbMode) { AlfParameters::CtbModeHandler::setDisabled(ctbMode); });
                });
}

void Picture::extendMcPaddedBorder(int end)
{
  for (int comp = 0; comp < getNumberValidComponents(m_cs->area.chromaFormat); comp++)
  {
    // pad corners
    CompID compID = CompID(comp);
    PelBuf p      = m_bufs[PIC_RECONSTRUCTION].get(compID);
    if (end < 0)
    {
      end = margin;
    }
    int xMarginCorner = end >> getComponentScaleX(compID, m_cs->area.chromaFormat);
    int yMarginCorner = end >> getComponentScaleY(compID, m_cs->area.chromaFormat);

    // do top left and top right corner
    {
      Pel *ptl  = p.bufAt(0, 0);
      Pel *ptr  = p.bufAt(p.width - 1, 0);
      Pel *ptli = ptl - p.stride;
      Pel *ptri = ptr - p.stride;
      for (int y = 1; y < yMarginCorner + 1; y++)
      {
        for (int x = 1; x < xMarginCorner + 1; x++)
        {
          ptli[-x] = ptl[-x];
          ptri[x]  = ptr[x];
        }
        ptli -= p.stride;
        ptri -= p.stride;
      }
    }

    // do bottom left and right corner
    {
      Pel *ptl  = p.bufAt(0, p.height - 1);
      Pel *ptr  = p.bufAt(p.width - 1, p.height - 1);
      Pel *ptli = ptl + p.stride;
      Pel *ptri = ptr + p.stride;
      for (int y = 1; y < yMarginCorner + 1; y++)
      {
        for (int x = 1; x < xMarginCorner + 1; x++)
        {
          ptli[-x] = ptl[-x];
          ptri[x]  = ptr[x];
        }
        ptli += p.stride;
        ptri += p.stride;
      }
    }

    // pad outside
    int width  = p.width + ((2 * end) >> getComponentScaleX(compID, m_cs->area.chromaFormat));
    int height = p.height + ((2 * end) >> getComponentScaleY(compID, m_cs->area.chromaFormat));

    int tmpOffsetX = end >> getComponentScaleX(compID, m_cs->area.chromaFormat);
    int tmpOffsetY = end >> getComponentScaleY(compID, m_cs->area.chromaFormat);

    Pel *piTxt = p.bufAt(-tmpOffsetX, -tmpOffsetY);

    int xmargin = (margin - end) >> getComponentScaleX(compID, m_cs->area.chromaFormat);
    int ymargin = (margin - end) >> getComponentScaleY(compID, m_cs->area.chromaFormat);

    Pel *pi = piTxt;
    // do left and right margins
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < xmargin; x++)
      {
        pi[-xmargin + x] = pi[0];
        pi[width + x]    = pi[width - 1];
      }
      pi += p.stride;
    }

    // pi is now the (0,height) (bottom left of image within bigger picture
    pi -= (p.stride + xmargin);
    // pi is now the (-marginX, height-1)
    for (int y = 0; y < ymargin; y++)
    {
      ::memcpy(pi + (y + 1) * p.stride, pi, sizeof(Pel) * (width + (xmargin << 1)));
    }

    // pi is still (-marginX, height-1)
    pi -= ((height - 1) * p.stride);
    // pi is now (-marginX, 0)
    for (int y = 0; y < ymargin; y++)
    {
      ::memcpy(pi - (y + 1) * p.stride, pi, sizeof(Pel) * (width + (xmargin << 1)));
    }
  }
}

void addCand(std::vector<IdxCost> &bestCands, int i, int k, int32_t cost, const int numAvgCands)
{
  IdxCost currIdx;
  currIdx.i    = i;
  currIdx.k    = k;
  currIdx.cost = cost;
  if (bestCands.size() == 0)
  {
    bestCands.push_back(currIdx);
  }
  else
  {
    uint64_t idx = bestCands.size();
    for (int n = 0; n < bestCands.size(); ++n)
    {
      if (bestCands[n].cost > cost)
      {
        idx = n;
        break;
      }
    }
    if (idx < bestCands.size())
    {
      bestCands.insert(bestCands.begin() + idx, currIdx);
    }
    else
    {
      bestCands.push_back(currIdx);
    }
  }
  while (bestCands.size() > numAvgCands)
  {
    bestCands.erase(bestCands.begin() + numAvgCands);
  }
}

template<boundaryDirection T> void Picture::TemplateMatchingPadding(Area subpicArea)
{
  PelUnitBuf   unitBuf                          = m_bufs[PIC_RECONSTRUCTION];
  ChromaFormat cf                               = unitBuf.chromaFormat;
  // candidate definition
  const int    NUM_STEPS                        = 16;
  const int    numCands[NUM_STEPS]              = { 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31 };
  const int    NUM_CANDS                        = 31;
  const int    candPos[NUM_STEPS][2][NUM_CANDS] = { { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
                                                      { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                                                        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
                                                    { { 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                        0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
                                                      { 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                                                        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
                                                    { { 2, -2, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                        0, 0,  0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
                                                      { 1, 1, 2, 2, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                                                        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
                                                    { { 3, -3, 2, -2, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                        0, 0,  0, 0,  0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0 },
                                                      { 1, 1, 2, 2, 3, 3, 4, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                                                        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
                                                    { { 4, -4, 3, -3, 2, -2, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0,
                                                        0, 0,  0, 0,  0, 0,  0, 0,  0, 0, 0, 0, 0, 0, 0 },
                                                      { 1, 1, 2, 2, 3, 3, 4, 4, 5, 1, 1, 1, 1, 1, 1, 1,
                                                        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
                                                    { { 5, -5, 4, -4, 3, -3, 2, -2, 1, -1, 0, 0, 0, 0, 0, 0,
                                                        0, 0,  0, 0,  0, 0,  0, 0,  0, 0,  0, 0, 0, 0, 0 },
                                                      { 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 1, 1, 1, 1, 1,
                                                        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
                                                    { { 6, -6, 5, -5, 4, -4, 3, -3, 2, -2, 1, -1, 0, 0, 0, 0,
                                                        0, 0,  0, 0,  0, 0,  0, 0,  0, 0,  0, 0,  0, 0, 0 },
                                                      { 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 1, 1, 1,
                                                        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
                                                    { { 7, -7, 6, -6, 5, -5, 4, -4, 3, -3, 2, -2, 1, -1, 0, 0,
                                                        0, 0,  0, 0,  0, 0,  0, 0,  0, 0,  0, 0,  0, 0,  0 },
                                                      { 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 1,
                                                        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
                                                    { { 8, -8, 7, -7, 6, -6, 5, -5, 4, -4, 3, -3, 2, -2, 1, -1,
                                                        0, 0,  0, 0,  0, 0,  0, 0,  0, 0,  0, 0,  0, 0,  0 },
                                                      { 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8,
                                                        9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
                                                    { { 9, -9, 8, -8, 7, -7, 6, -6, 5, -5, 4, -4, 3, -3, 2, -2,
                                                        1, -1, 0, 0,  0, 0,  0, 0,  0, 0,  0, 0,  0, 0,  0 },
                                                      { 1, 1, 2,  2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8,
                                                        9, 9, 10, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
                                                    { { 10, -10, 9, -9, 8, -8, 7, -7, 6, -6, 5, -5, 4, -4, 3, -3,
                                                        2,  -2,  1, -1, 0, 0,  0, 0,  0, 0,  0, 0,  0, 0,  0 },
                                                      { 1, 1, 2,  2,  3,  3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8,
                                                        9, 9, 10, 10, 11, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
                                                    { { 11, -11, 10, -10, 9, -9, 8, -8, 7, -7, 6, -6, 5, -5, 4, -4,
                                                        3,  -3,  2,  -2,  1, -1, 0, 0,  0, 0,  0, 0,  0, 0,  0 },
                                                      { 1, 1, 2,  2,  3,  3,  4,  4, 5, 5, 6, 6, 7, 7, 8, 8,
                                                        9, 9, 10, 10, 11, 11, 12, 1, 1, 1, 1, 1, 1, 1, 1 } },
                                                    { { 12, -12, 11, -11, 10, -10, 9, -9, 8, -8, 7, -7, 6, -6, 5, -5,
                                                        4,  -4,  3,  -3,  2,  -2,  1, -1, 0, 0,  0, 0,  0, 0,  0 },
                                                      { 1, 1, 2,  2,  3,  3,  4,  4,  5,  5, 6, 6, 7, 7, 8, 8,
                                                        9, 9, 10, 10, 11, 11, 12, 12, 13, 1, 1, 1, 1, 1, 1 } },
                                                    { { 13, -13, 12, -12, 11, -11, 10, -10, 9, -9, 8, -8, 7, -7, 6, -6,
                                                        5,  -5,  4,  -4,  3,  -3,  2,  -2,  1, -1, 0, 0,  0, 0,  0 },
                                                      { 1, 1, 2,  2,  3,  3,  4,  4,  5,  5,  6,  6, 7, 7, 8, 8,
                                                        9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 1, 1, 1, 1 } },
                                                    { { 14, -14, 13, -13, 12, -12, 11, -11, 10, -10, 9, -9, 8, -8, 7, -7,
                                                        6,  -6,  5,  -5,  4,  -4,  3,  -3,  2,  -2,  1, -1, 0, 0,  0 },
                                                      { 1, 1, 2,  2,  3,  3,  4,  4,  5,  5,  6,  6,  7,  7, 8, 8,
                                                        9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 1, 1 } },
                                                    { { 15, -15, 14, -14, 13, -13, 12, -12, 11, -11, 10, -10, 9, -9, 8, -8,
                                                        7,  -7,  6,  -6,  5,  -5,  4,  -4,  3,  -3,  2,  -2,  1, -1, 0 },
                                                      { 1, 1, 2,  2,  3,  3,  4,  4,  5,  5,  6,  6,  7,  7,  8, 8,
                                                        9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 16 } } };

  static constexpr bool isBotTop = (T == BD_BOTTOM) || (T == BD_TOP);

  const int indWidth  = isBotTop ? subpicArea.size().width : subpicArea.size().height;
  const int indStride = TMP_TEMPLATE_WIDTH - TMP_TW_MINUS_TBW_BY_TWO - TMP_TW_MINUS_TBW_BY_TWO;

  Size lumaKernelSize;
  if constexpr (isBotTop)
  {
    lumaKernelSize = Size(TMP_TEMPLATE_WIDTH, 1);
  }
  else
  {
    lumaKernelSize = Size(1, TMP_TEMPLATE_WIDTH);
  }

  CompStorage pixStorage = CompStorage();
  Area        area       = Area(Position(0, 0), lumaKernelSize);
  pixStorage.create(area);
  PelBuf pix = PelBuf(pixStorage);

  int currLine = 0;

  const int idxStop = (indWidth - TMP_TW_MINUS_TBW_BY_TWO + indStride - 1) / indStride;

  int ***candidateArray = new int **[idxStop];
  for (int i = 0; i < idxStop; ++i)
  {
    candidateArray[i] = new int *[TMP_NUM_AVG_CANDS];
    for (int j = 0; j < TMP_NUM_AVG_CANDS; ++j)
    {
      candidateArray[i][j] = new int[2];
    }
  }

  int ind = 0;
  for (int idx = 0; idx < idxStop; idx++)
  {
    if (ind > indWidth - TMP_TEMPLATE_WIDTH)
    {
      ind = indWidth - TMP_TEMPLATE_WIDTH;
    }

    int costs[TMP_MAX_NUM_STEPS][NUM_CANDS] = {};
    for (int s = 0; s < TMP_MAX_NUM_STEPS; s++)
    {
      for (int c = 0; c < NUM_CANDS; c++)
      {
        costs[s][c] = -1;
      }
    }

    int minCost = MAX_INT;
    int comp    = 0;

    PelBuf buf = unitBuf.get(CompID(comp));

    PelBuf kernel;
    if constexpr (T == BD_TOP)
    {
      kernel = buf.subBuf(Position(ind + subpicArea.pos().x, subpicArea.pos().y - currLine),
                          Size(TMP_TEMPLATE_WIDTH, TMP_TEMPLATE_HEIGHT));
    }
    else if constexpr (T == BD_BOTTOM)
    {
      kernel = buf.subBuf(Position(ind + subpicArea.pos().x,
                                   subpicArea.pos().y + subpicArea.size().height + currLine - TMP_TEMPLATE_HEIGHT),
                          Size(TMP_TEMPLATE_WIDTH, TMP_TEMPLATE_HEIGHT));
    }
    else if constexpr (T == BD_LEFT)
    {
      kernel = buf.subBuf(Position(subpicArea.pos().x - currLine, ind + subpicArea.pos().y),
                          Size(TMP_TEMPLATE_HEIGHT, TMP_TEMPLATE_WIDTH));
    }
    else if constexpr (T == BD_RIGHT)
    {
      kernel = buf.subBuf(Position(subpicArea.pos().x + subpicArea.size().width + currLine - TMP_TEMPLATE_HEIGHT,
                                   ind + subpicArea.pos().y),
                          Size(TMP_TEMPLATE_HEIGHT, TMP_TEMPLATE_WIDTH));
    }

    for (int k = 0; k < TMP_MAX_NUM_STEPS; k++)
    {
      for (int i = 0; i < numCands[k]; i++)
      {
        int tempCost = 0;
        if (ind + candPos[k][0][i] < 0 || ind + candPos[k][0][i] + TMP_TEMPLATE_WIDTH > indWidth)
        {
          continue;
        }

        // get cand
        PelBuf cand;
        if constexpr (T == BD_TOP)
        {
          cand = buf.subBuf(
            Position(ind + subpicArea.pos().x + candPos[k][0][i], subpicArea.pos().y - currLine + candPos[k][1][i]),
            Size(TMP_TEMPLATE_WIDTH, TMP_TEMPLATE_HEIGHT));
        }
        else if constexpr (T == BD_BOTTOM)
        {
          cand = buf.subBuf(
            Position(ind + subpicArea.pos().x + candPos[k][0][i],
                     subpicArea.pos().y + subpicArea.size().height + currLine - TMP_TEMPLATE_HEIGHT - candPos[k][1][i]),
            Size(TMP_TEMPLATE_WIDTH, TMP_TEMPLATE_HEIGHT));
        }
        else if constexpr (T == BD_LEFT)
        {
          cand = buf.subBuf(
            Position(subpicArea.pos().x - currLine + candPos[k][1][i], ind + subpicArea.pos().y + candPos[k][0][i]),
            Size(TMP_TEMPLATE_HEIGHT, TMP_TEMPLATE_WIDTH));
        }
        else if constexpr (T == BD_RIGHT)
        {
          cand = buf.subBuf(
            Position(subpicArea.pos().x + subpicArea.size().width + currLine - TMP_TEMPLATE_HEIGHT - candPos[k][1][i],
                     ind + subpicArea.pos().y + candPos[k][0][i]),
            Size(TMP_TEMPLATE_HEIGHT, TMP_TEMPLATE_WIDTH));
        }

        for (int n = 0; n < TMP_TEMPLATE_HEIGHT; n++)
        {
          for (int m = 0; m < TMP_TEMPLATE_WIDTH; m++)
          {
            Pel *a;
            Pel *b;

            if constexpr (isBotTop)
            {
              a = cand.bufAt(m, n);
              b = kernel.bufAt(m, n);
            }
            else
            {
              a = cand.bufAt(n, m);
              b = kernel.bufAt(n, m);
            }

            tempCost += std::abs(int32_t(*a) - int32_t(*b));
          }
        }
        costs[k][i] = tempCost;
        if (tempCost < minCost)
        {
          minCost = tempCost;
        }
      }
      if (k >= TMP_MIN_NUM_STEPS)
      {
        if (minCost <= TMP_EARLY_TERMINATION_THRESHOLD)
        {
          break;
        }
      }
    }
    // adaptive candidate selection
    std::vector<IdxCost> bestCands;
    int                  scaledThres = int(TMP_ACS_FACTOR * float(minCost));

    for (int k = 0; k < TMP_MAX_NUM_STEPS; k++)
    {
      for (int i = 0; i < NUM_CANDS; i++)
      {
        int cost = costs[k][i];
        if (cost >= 0 && cost <= scaledThres)
        {
          addCand(bestCands, i, k, cost, TMP_NUM_AVG_CANDS);
        }
      }
    }

    for (int n = 0; n < TMP_NUM_AVG_CANDS; n++)
    {
      if (n < bestCands.size())
      {
        candidateArray[idx][n][0] = bestCands.at(n).i;
        candidateArray[idx][n][1] = bestCands.at(n).k;
      }
      else
      {
        candidateArray[idx][n][0] = -1;
        candidateArray[idx][n][1] = -1;
      }
    }
    ind += indStride;
  }

  for (int pl = 0; pl < TMP_PADSIZE; pl++)
  {
    currLine = pl;
    ind      = 0;
    for (int idx = 0; idx < idxStop; idx++)
    {
      if (ind > indWidth - TMP_TEMPLATE_WIDTH)
      {
        ind = indWidth - TMP_TEMPLATE_WIDTH;
      }

      const int comp = 0;
      PelBuf    buf  = unitBuf.get(CompID(comp));

      pix.fill(0);

      int mStart = ind == 0 ? 0 : TMP_TW_MINUS_TBW_BY_TWO;
      int mEnd =
        ind == indWidth - TMP_TEMPLATE_WIDTH ? TMP_TEMPLATE_WIDTH : TMP_TEMPLATE_WIDTH - TMP_TW_MINUS_TBW_BY_TWO;

      int numSelCands = 0;
      for (int n = 0; n < TMP_NUM_AVG_CANDS; n++)
      {
        int i = candidateArray[idx][n][0];
        int k = candidateArray[idx][n][1];
        if (i < 0)
        {
          break;
        }
        numSelCands++;

        Area sourceArea;
        if constexpr (T == BD_TOP)
        {
          Position lumaPos =
            Position(ind + subpicArea.pos().x + candPos[k][0][i], subpicArea.pos().y - currLine + candPos[k][1][i] - 1);
          sourceArea = Area(lumaPos, lumaKernelSize);
        }
        else if constexpr (T == BD_BOTTOM)
        {
          Position lumaPos = Position(ind + subpicArea.pos().x + candPos[k][0][i],
                                      subpicArea.pos().y + subpicArea.size().height + currLine - candPos[k][1][i]);
          sourceArea       = Area(lumaPos, lumaKernelSize);
        }
        else if constexpr (T == BD_LEFT)
        {
          Position lumaPos =
            Position(subpicArea.pos().x - currLine + candPos[k][1][i] - 1, ind + subpicArea.pos().y + candPos[k][0][i]);
          sourceArea = Area(lumaPos, lumaKernelSize);
        }
        else if constexpr (T == BD_RIGHT)
        {
          Position lumaPos = Position(subpicArea.pos().x + subpicArea.size().width + currLine - candPos[k][1][i],
                                      ind + subpicArea.pos().y + candPos[k][0][i]);
          sourceArea       = Area(lumaPos, lumaKernelSize);
        }

        PelBuf source = buf.subBuf(sourceArea.pos(), sourceArea.size());
        for (int m = mStart; m < mEnd; m++)
        {
          if (isBotTop)
          {
            *pix.bufAt(m, 0) += *source.bufAt(m, 0);
          }
          else
          {
            *pix.bufAt(0, m) += *source.bufAt(0, m);
          }
        }
      }
      for (int m = mStart; m < mEnd; m++)
      {
        if (isBotTop)
        {
          *pix.bufAt(m, 0) /= numSelCands;
        }
        else
        {
          *pix.bufAt(0, m) /= numSelCands;
        }
      }

      Position lumaDstPos;
      if constexpr (T == BD_TOP)
      {
        lumaDstPos = Position(ind + subpicArea.pos().x, subpicArea.pos().y - currLine - 1);
      }
      else if constexpr (T == BD_BOTTOM)
      {
        lumaDstPos = Position(ind + subpicArea.pos().x, subpicArea.pos().y + subpicArea.size().height + currLine);
      }
      else if constexpr (T == BD_LEFT)
      {
        lumaDstPos = Position(subpicArea.pos().x - currLine - 1, ind + subpicArea.pos().y);
      }
      else if constexpr (T == BD_RIGHT)
      {
        lumaDstPos = Position(subpicArea.pos().x + subpicArea.size().width + currLine, ind + subpicArea.pos().y);
      }

      PelBuf subBuf = buf.subBuf(lumaDstPos, lumaKernelSize);

      for (int m = mStart; m < mEnd; m++)
      {
        if (isBotTop)
        {
          *subBuf.bufAt(m, 0) = *pix.bufAt(m, 0);
        }
        else
        {
          *subBuf.bufAt(0, m) = *pix.bufAt(0, m);
        }
      }
      ind += indStride;
    }
  }

  bool processChroma = true;
  int  ssX           = 0;
  int  ssY           = 0;
  if (cf == ChromaFormat::_400)
  {
    processChroma = false;
  }
  if (cf == ChromaFormat::_420)
  {
    ssX = 1;
    ssY = 1;
  }
  if (cf == ChromaFormat::_422)
  {
    ssX = 1;
    ssY = 0;
  }
  if (cf == ChromaFormat::_444)
  {
    ssX = 0;
    ssY = 0;
  }

  int plStride;
  int ssBoundary;
  if constexpr (isBotTop)
  {
    plStride   = ssY ? 2 : 1;
    ssBoundary = ssX;
  }
  else
  {
    plStride   = ssX ? 2 : 1;
    ssBoundary = ssY;
  }

  Size chromaKernelSize;
  if constexpr (isBotTop)
  {
    chromaKernelSize = Size(TMP_TEMPLATE_WIDTH >> ssX, 1);
  }
  else
  {
    chromaKernelSize = Size(1, TMP_TEMPLATE_WIDTH >> ssY);
  }

  CompStorage pixStorageCb = CompStorage();
  CompStorage pixStorageCr = CompStorage();
  pixStorageCb.create(Area(Position(0, 0), chromaKernelSize));
  pixStorageCr.create(Area(Position(0, 0), chromaKernelSize));
  PelBuf pixCb = PelBuf(pixStorageCb);
  PelBuf pixCr = PelBuf(pixStorageCr);

  if (processChroma)
  {
    for (int pl = 0; pl < TMP_PADSIZE; pl += plStride)
    {
      currLine = pl;
      ind      = 0;
      for (int idx = 0; idx < idxStop; idx++)
      {
        if (ind > indWidth - TMP_TEMPLATE_WIDTH)
        {
          ind = indWidth - TMP_TEMPLATE_WIDTH;
        }

        pixCb.fill(0);
        pixCr.fill(0);

        const int mStart = ind == 0 ? 0 : TMP_TW_MINUS_TBW_BY_TWO >> ssBoundary;
        const int mEnd   = ind == indWidth - TMP_TEMPLATE_WIDTH
            ? TMP_TEMPLATE_WIDTH >> ssBoundary
            : (TMP_TEMPLATE_WIDTH - TMP_TW_MINUS_TBW_BY_TWO) >> ssBoundary;

        int candCounter = 0;
        for (int indAdd = 0; indAdd < (1 + ssBoundary); indAdd++)
        {
          for (int n = 0; n < TMP_NUM_AVG_CANDS; n++)
          {
            int i = candidateArray[idx][n][0];
            int k = candidateArray[idx][n][1];

            if (i < 0 || k < 0)
            {
              break;
            }

            int ix = candPos[k][0][i];
            int iy = candPos[k][1][i];

            if (iy <= 1)
            {
              ix += candPos[k][0][i];
              iy += 1;
            }
            if (ind + indAdd + ix < 0 || ind + indAdd + ix + TMP_TEMPLATE_WIDTH > indWidth)
            {
              ix = candPos[k][0][i];
            }
            candCounter++;

            Area sourceArea;
            Area sourceAreaChroma;
            if constexpr (T == BD_TOP)
            {
              Position lumaPos =
                Position(ind + indAdd + subpicArea.pos().x + ix, subpicArea.pos().y - currLine + iy - 1);
              Position chromaPos = Position(lumaPos.x >> ssX, lumaPos.y >> ssY);
              sourceAreaChroma   = Area(chromaPos, chromaKernelSize);
            }
            else if constexpr (T == BD_BOTTOM)
            {
              Position lumaPos   = Position(ind + indAdd + subpicArea.pos().x + ix,
                                            subpicArea.pos().y + subpicArea.size().height + currLine - iy);
              Position chromaPos = Position(lumaPos.x >> ssX, lumaPos.y >> ssY);
              sourceAreaChroma   = Area(chromaPos, chromaKernelSize);
            }
            else if constexpr (T == BD_LEFT)
            {
              Position lumaPos =
                Position(subpicArea.pos().x - currLine + iy - 1, ind + indAdd + subpicArea.pos().y + ix);
              Position chromaPos = Position(lumaPos.x >> ssX, lumaPos.y >> ssY);
              sourceAreaChroma   = Area(chromaPos, chromaKernelSize);
            }
            else if constexpr (T == BD_RIGHT)
            {
              Position lumaPos   = Position(subpicArea.pos().x + subpicArea.size().width + currLine - iy,
                                            ind + indAdd + subpicArea.pos().y + ix);
              Position chromaPos = Position(lumaPos.x >> ssX, lumaPos.y >> ssY);
              sourceAreaChroma   = Area(chromaPos, chromaKernelSize);
            }

            PelBuf sourceCb = unitBuf.get(COMP_Cb).subBuf(sourceAreaChroma.pos(), sourceAreaChroma.size());
            PelBuf sourceCr = unitBuf.get(COMP_Cr).subBuf(sourceAreaChroma.pos(), sourceAreaChroma.size());
            for (int m = mStart; m < mEnd; m++)
            {
              if (isBotTop)
              {
                *pixCb.bufAt(m, 0) += *sourceCb.bufAt(m, 0);
                *pixCr.bufAt(m, 0) += *sourceCr.bufAt(m, 0);
              }
              else
              {
                *pixCb.bufAt(0, m) += *sourceCb.bufAt(0, m);
                *pixCr.bufAt(0, m) += *sourceCr.bufAt(0, m);
              }
            }
          }
        }
        for (int m = mStart; m < mEnd; m++)
        {
          if (isBotTop)
          {
            *pixCb.bufAt(m, 0) /= candCounter;
            *pixCr.bufAt(m, 0) /= candCounter;
          }
          else
          {
            *pixCb.bufAt(0, m) /= candCounter;
            *pixCr.bufAt(0, m) /= candCounter;
          }
        }

        Position lumaDstPos;
        Position chromaDstPos;
        if constexpr (T == BD_TOP)
        {
          lumaDstPos   = Position(ind + subpicArea.pos().x, subpicArea.pos().y - currLine - 2);
          chromaDstPos = Position(lumaDstPos.x >> ssX, lumaDstPos.y >> ssY);
        }
        else if constexpr (T == BD_BOTTOM)
        {
          lumaDstPos   = Position(ind + subpicArea.pos().x, subpicArea.pos().y + subpicArea.size().height + currLine);
          chromaDstPos = Position(lumaDstPos.x >> ssX, lumaDstPos.y >> ssY);
        }
        else if constexpr (T == BD_LEFT)
        {
          lumaDstPos   = Position(subpicArea.pos().x - currLine - 2, ind + subpicArea.pos().y);
          chromaDstPos = Position(lumaDstPos.x >> ssX, lumaDstPos.y >> ssY);
        }
        else if constexpr (T == BD_RIGHT)
        {
          lumaDstPos   = Position(subpicArea.pos().x + subpicArea.size().width + currLine, ind + subpicArea.pos().y);
          chromaDstPos = Position(lumaDstPos.x >> ssX, lumaDstPos.y >> ssY);
        }

        PelBuf dstCb = unitBuf.get(COMP_Cb).subBuf(chromaDstPos, chromaKernelSize);
        PelBuf dstCr = unitBuf.get(COMP_Cr).subBuf(chromaDstPos, chromaKernelSize);
        for (int m = mStart; m < mEnd; m++)
        {
          if (isBotTop)
          {
            *dstCb.bufAt(m, 0) = *pixCb.bufAt(m, 0);
            *dstCr.bufAt(m, 0) = *pixCr.bufAt(m, 0);
          }
          else
          {
            *dstCb.bufAt(0, m) = *pixCb.bufAt(0, m);
            *dstCr.bufAt(0, m) = *pixCr.bufAt(0, m);
          }
        }
        ind += indStride;
      }
    }
  }

  for (int i = 0; i < idxStop; ++i)
  {
    for (int j = 0; j < TMP_NUM_AVG_CANDS; ++j)
    {
      delete[] candidateArray[i][j];
    }
    delete[] candidateArray[i];
  }
  delete[] candidateArray;
}
