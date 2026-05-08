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

/** \file     AdaptiveLoopFilterEcm.cpp
    \brief    ECM adaptive loop filter class
*/

#include "AdaptiveLoopFilterEcm.h"
#include "AlfFixedFilters.h"
#include "CodingStructure.h"
#include "Picture.h"
#include <array>
#include <cmath>

AdaptiveLoopFilterEcm::AdaptiveLoopFilterEcm() : AdaptiveLoopFilter()
{
  initAdaptiveLoopFilter();
#if ENABLE_SIMD_OPT_ALF
#ifdef TARGET_SIMD_X86
  initAdaptiveLoopFilterX86();
#endif
#endif
}

void AdaptiveLoopFilterEcm::initAdaptiveLoopFilter()
{
  m_deriveClassificationLaplacian    = deriveClassificationLaplacian;
  m_deriveClassificationLaplacianBig = deriveClassificationLaplacianBig;
  m_deriveVariance                   = deriveVariance;
  m_calcClassGradBasedFixedFilt      = calcClassGradBasedFixedFilt;
  m_calcClassGradBasedAdaptFilt      = calcClassGradBasedAdaptFilt;
  m_calcClassBandBased               = calcClassBandBased;

  m_filterCcAlf = filterBlkCcAlf;

  m_filter9x9Blk                = filterBlk<ALF_FILTER_9>;
  m_filter9x9BlkExtDbResiDirect = filterBlk<ALF_FILTER_9_EXT_DB_RESI_DIRECT>;
  m_filter9x9BlkExtDbResi       = filterBlk<ALF_FILTER_9_EXT_DB_RESI>;

  m_fixFilter13x13Db9Blk = fixedFilter13x13Db9Blk;
  m_fixFilter9x9Db9Blk   = fixedFilter9x9Db9Blk;
  m_filterResi9x9Blk     = fixedFilteringResi;
  m_gaussFiltering       = gaussFiltering;
}

void AdaptiveLoopFilterEcm::alfFiltering(const ClassBuf &classifier, const PelUnitBuf &recDst,
                                         const CPelBuf &recBeforeDb, const CPelBuf &resiLuma, const CPelUnitBuf &recSrc,
                                         const Area &blkDst, const Area &blk, const CompID compId,
                                         const int8_t *scaleIdxSet, const short *filterSet, const Pel *fClipSet,
                                         AlfFilterType filterType, bool isFixFiltPaddedPerCtu,
                                         const FixFiltSetCand fixedFilterSetCandIdx)
{
  alfFilterFunc *filterFunc = nullptr;
  switch (filterType)
  {
    // clang-format off
  case ALF_FILTER_9:                    filterFunc = m_filter9x9Blk;                break;
  case ALF_FILTER_9_EXT_DB_RESI_DIRECT: filterFunc = m_filter9x9BlkExtDbResiDirect; break;
  case ALF_FILTER_9_EXT_DB_RESI:        filterFunc = m_filter9x9BlkExtDbResi;       break;
  default:                              THROW("Unsupported filter type.");          break;
    // clang-format on
  }

  static_assert(NUM_FIXED_FILTER_SET_CANDS == 2,
                "Derivation of residual filter candidate index is only implemented for two candidate sets.");
  const FixFiltSetCand fixedFilterSetCandIdxResi =
    (fixedFilterSetCandIdx == FixFiltSetCand::FIRST ? FixFiltSetCand::SECOND : FixFiltSetCand::FIRST);

  // TODO: ALF: ECM uses nullptrs here for non-fixed-filter settings. BS 2023-08-23
  filterFunc(classifier, recDst, recBeforeDb, resiLuma, recSrc, blkDst, blk, compId, scaleIdxSet, filterSet, fClipSet,
             m_clpRngs.comp[compId], m_fixFilterResult[compId][fixedFilterSetCandIdx],
             m_fixedFilterResultPerCtu[fixedFilterSetCandIdx], m_fixFilterResiResult[fixedFilterSetCandIdxResi],
             m_gaussPic, m_gaussCtu, isFixFiltPaddedPerCtu, fixedFilterSetCandIdx);
}

void AdaptiveLoopFilterEcm::applyCcAlfFilter(CodingStructure &cs, CompID compID, const PelBuf &dstBuf,
                                             const PelUnitBuf &recYuvExt, uint8_t *filterControl,
                                             const short filterSet[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF],
                                             const int   selectedFilterIdx)
{
  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;

  PelUnitBuf recYuvSao = m_tempBufSao.getBuf(cs.area);

  int ctuIdx = 0;
  for (int yPos = 0; yPos < m_picHeight; yPos += m_maxCUHeight)
  {
    for (int xPos = 0; xPos < m_picWidth; xPos += m_maxCUWidth)
    {
      int  filterIdx     = (filterControl == nullptr)
             ? selectedFilterIdx
             : filterControl[(yPos >> cs.pcv->maxCUHeightLog2) * cs.pcv->widthInCtus + (xPos >> cs.pcv->maxCUWidthLog2)];
      bool skipFiltering = (filterControl != nullptr && filterIdx == 0) ? true : false;
      if (!skipFiltering)
      {
        if (filterControl != nullptr)
        {
          filterIdx--;
        }

        const int16_t *filterCoeff = filterSet[filterIdx];

        const int width        = (xPos + m_maxCUWidth > m_picWidth) ? (m_picWidth - xPos) : m_maxCUWidth;
        const int height       = (yPos + m_maxCUHeight > m_picHeight) ? (m_picHeight - yPos) : m_maxCUHeight;
        const int chromaScaleX = getComponentScaleX(compID, m_chromaFormat);
        const int chromaScaleY = getComponentScaleY(compID, m_chromaFormat);

        int rasterSliceAlfPad = 0;
        if (isCrossedBySubPicBoundaries(cs, xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight,
                                        rasterSliceAlfPad))
        {
          const bool clipT = clipTop || (yPos == 0);
          const bool clipB = clipBottom || (yPos + height == m_picHeight);
          const bool clipL = clipLeft || (xPos == 0);
          const bool clipR = clipRight || (xPos + width == m_picWidth);
          const int  wBuf  = width + (clipL ? 0 : MAX_ALF_PADDING_SIZE) + (clipR ? 0 : MAX_ALF_PADDING_SIZE);
          const int  hBuf  = height + (clipT ? 0 : MAX_ALF_PADDING_SIZE) + (clipB ? 0 : MAX_ALF_PADDING_SIZE);
          PelUnitBuf buf   = m_tempBuf2.subBuf(UnitArea(cs.area.chromaFormat, Area(0, 0, wBuf, hBuf)));
          buf.copyFrom(recYuvExt.subBuf(UnitArea(
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
          // TODO: ALF: Possible overflow for chroma? BS 2023-07-17
          buf.addMirrorExtension(MAX_ALF_PADDING_SIZE, false);
          buf = buf.subBuf(
            UnitArea(cs.area.chromaFormat,
                     Area(clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, width, height)));

          // TODO: don't copy/mirror luma since it's not used
          PelUnitBuf bufSao = m_tempBufSaoCtu.subBuf(UnitArea(cs.area.chromaFormat, Area(0, 0, wBuf, hBuf)));
          bufSao.copyFrom(recYuvSao.subBuf(UnitArea(
            cs.area.chromaFormat,
            Area(xPos - (clipL ? 0 : MAX_ALF_PADDING_SIZE), yPos - (clipT ? 0 : MAX_ALF_PADDING_SIZE), wBuf, hBuf))));
          // pad top-left unavailable samples for raster slice
          if (rasterSliceAlfPad & 1)
          {
            bufSao.padBorderPel(MAX_ALF_PADDING_SIZE, 1);
          }
          // pad bottom-right unavailable samples for raster slice
          if (rasterSliceAlfPad & 2)
          {
            bufSao.padBorderPel(MAX_ALF_PADDING_SIZE, 2);
          }
          bufSao.addMirrorExtension(MAX_ALF_PADDING_SIZE);
          bufSao = bufSao.subBuf(
            UnitArea(cs.area.chromaFormat,
                     Area(clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, width, height)));

          const Area blkSrc(0, 0, width, height);

          const Area blkDst(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY);
          m_filterCcAlf(dstBuf, buf, bufSao, blkDst, blkSrc, compID, filterCoeff, m_clpRngs, cs);
        }
        else
        {
          const UnitArea area(m_chromaFormat, Area(xPos, yPos, width, height));

          Area blkDst(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY);
          Area blkSrc(xPos, yPos, width, height);

          m_filterCcAlf(dstBuf, recYuvExt, recYuvSao, blkDst, blkSrc, compID, filterCoeff, m_clpRngs, cs);
        }
      }
      ctuIdx++;
    }
  }
}

void AdaptiveLoopFilterEcm::ALFProcess(CodingStructure &cs)
{
  // set clipping range
  m_clpRngs = cs.slice->m_clpRngs;

  // set CTU enable flags
  CHECKD(cs.picture->getAlfModes(CompID::COMP_Y).size() != m_numCTUsInPic,
         "Unexpected number of CTB mode entries (Y).");
  CHECKD(cs.picture->getAlfModes(CompID::COMP_Cb).size() != m_numCTUsInPic,
         "Unexpected number of CTB mode entries (Cb).");
  CHECKD(cs.picture->getAlfModes(CompID::COMP_Cr).size() != m_numCTUsInPic,
         "Unexpected number of CTB mode entries (Cr).");
  m_modes = &cs.picture->getAlfModes();

  CtbModes *lumaModes    = nullptr;
  uint32_t  lastSliceIdx = 0xFFFFFFFF;

  PelUnitBuf recYuv = cs.getRecoBuf();

  // Setup the frame buffer for the reconstruction after SAO.
  m_tempBuf.copyFrom(recYuv);
  PelUnitBuf tmpYuv = m_tempBuf.getBuf(cs.area);
  tmpYuv.addMirrorExtension(MAX_FILTER_LENGTH_FIXED >> 1, false);

  // Setup the frame buffer for the reconstruction before DBF (DBF input).
  PelUnitBuf tmpYuvBeforeDb = m_tempBufBeforeDb.getBuf(cs.area);
  tmpYuvBeforeDb.addMirrorExtension(NUM_DB_PAD);

  // Setup the frame buffer for the residual.
  PelBuf tmpYResi = m_tempBufResi.subBuf(cs.area.lumaPos(), cs.area.lumaSize());
  tmpYResi.addMirrorExtension(MAX_FILTER_LENGTH_FIXED >> 1);

  // Setup the frame buffer for the SAO output. Needed only for CCALF
  // TODO: don't copy/mirror luma since it's not used
  m_tempBufSao.copyFrom(recYuv);
  PelUnitBuf tmpYuvSao = m_tempBufSao.getBuf(cs.area);
  tmpYuvSao.addMirrorExtension(MAX_FILTER_LENGTH_FIXED >> 1);

  for (int compIdx = 0; compIdx < MAX_NUM_COMP; ++compIdx)
  {
    std::fill(m_ctuAlreadyProcessedFlag[compIdx].begin(), m_ctuAlreadyProcessedFlag[compIdx].end(), false);
  }
  std::fill(m_ctuPadFlag.begin(), m_ctuPadFlag.end(), 0);

  const auto getFixedFilterSetCandIdx = [](const Slice &slice, CompID compID) -> std::tuple<int, FixFiltSetCand>
  {
    const int fixedFilterSetCandIdxVal = slice.m_newAlfFixFiltSetCandIdx[compID];
    CHECK(fixedFilterSetCandIdxVal < 0 || fixedFilterSetCandIdxVal >= NUM_FIXED_FILTER_SET_CANDS,
          "Fixed filter set candidate index is out of range.");
    return std::make_tuple(fixedFilterSetCandIdxVal, static_cast<FixFiltSetCand>(fixedFilterSetCandIdxVal));
  };

  const PreCalcValues &pcv = *cs.pcv;

  int  ctuIdx  = 0;
  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;

  for (int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
  {
    for (int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
    {
      // get first CU in CTU
      const CodingUnit *cu = cs.getCU(Position(xPos, yPos), ChannelType::LUMA);

      // skip this CTU if ALF is disabled
      if (!cu->slice->m_alfEnabledFlag[COMP_Y] && !cu->slice->m_alfEnabledFlag[COMP_Cb] &&
          !cu->slice->m_alfEnabledFlag[COMP_Cr])
      {
        ctuIdx++;
        continue;
      }

      // reload ALF APS each time the slice changes during raster scan filtering
      if (ctuIdx == 0 || lastSliceIdx != cu->slice->getSliceID() || lumaModes == nullptr)
      {
        cs.slice = cu->slice;
        reconstructCoeffAPSs(cs, true, cu->slice->m_alfEnabledFlag[COMP_Cb] || cu->slice->m_alfEnabledFlag[COMP_Cr],
                             false);
        CHECKD(cu->slice->m_pic->getAlfModes(COMP_Y).size() != m_numCTUsInPic,
               "Unexpected number of luma CTB mode entries.");
        lumaModes          = &cu->slice->m_pic->getAlfModes(COMP_Y);
        m_ccAlfFilterParam = cu->slice->m_ccAlfFilterParam.getEcmParam();
      }
      lastSliceIdx = cu->slice->getSliceID();

      const CcAlfFilterParam &sliceCcAlfFilterParam = cu->slice->m_ccAlfFilterParam.getEcmParamNoCheck();

      const int width         = std::min(pcv.maxCUWidth, pcv.lumaWidth - static_cast<unsigned>(xPos));
      const int height        = std::min(pcv.maxCUHeight, pcv.lumaHeight - static_cast<unsigned>(yPos));
      bool      ctuEnableFlag = CtbModeHandler::isEnabled((*m_modes)[COMP_Y][ctuIdx]);
      for (int compIdx = 1; compIdx < MAX_NUM_COMP; compIdx++)
      {
        ctuEnableFlag |= CtbModeHandler::isEnabled((*m_modes)[compIdx][ctuIdx]);
        if (sliceCcAlfFilterParam.ccAlfFilterEnabled[compIdx - 1])
        {
          ctuEnableFlag |= m_ccAlfFilterControl[compIdx - 1][ctuIdx] > 0;
        }
      }

      // CTU of size (width, height) at position (yPos, xPos).

      int rasterSliceAlfPad = 0;
      if (ctuEnableFlag &&
          isCrossedBySubPicBoundaries(cs, xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight,
                                      rasterSliceAlfPad))
      {
        // Sub-unit of size (width, height) at position (xPos, yPos).

        const bool clipT = clipTop || (yPos == 0);
        const bool clipB = clipBottom || (yPos + height == pcv.lumaHeight);
        const bool clipL = clipLeft || (xPos == 0);
        const bool clipR = clipRight || (xPos + width == pcv.lumaWidth);
        const int  wBuf  = width + (clipL ? 0 : MAX_ALF_PADDING_SIZE) + (clipR ? 0 : MAX_ALF_PADDING_SIZE);
        const int  hBuf  = height + (clipT ? 0 : MAX_ALF_PADDING_SIZE) + (clipB ? 0 : MAX_ALF_PADDING_SIZE);

        // Setup reco unit buffer for sub-unit (sub-unit area, extended by neighbour samples and mirroring).
        PelUnitBuf buf = m_tempBuf2.subBuf(UnitArea(cs.area.chromaFormat, Area(0, 0, wBuf, hBuf)));
        buf.copyFrom(tmpYuv.subBuf(UnitArea(
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
          const int  m = (*lumaModes)[ctuIdx];

          // Setup pre-DBF reco luma buffer for sub-block (sub-block area, ext. by neighbor samples and mirroring).
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

          // Setup resi luma buffer for sub-block (sub-block area, extended by neighbor samples and mirroring).
          PelBuf bufResi = m_tempBufResiCtu.subBuf(0, 0, wBuf, hBuf);
          bufResi.copyFrom(tmpYResi.subBuf(xPos - (clipL ? 0 : MAX_ALF_PADDING_SIZE),
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

          deriveClassification(
            buf.get(COMP_Y),
            (LumaCtbModeHandler::isApsFilter(m) &&
             m_filterTypeApsLuma[LumaCtbModeHandler::getApsIdx(m)] == ALF_FILTER_9_EXT_DB_RESI),
            bufResi, bufDb, 0, blkDst, blkSrc, cs,
            LumaCtbModeHandler::isApsFilter(m)
              ? m_classifierIdxApsLuma[LumaCtbModeHandler::getApsIdx(m)][LumaCtbModeHandler::getAlternative(m)]
              : -1);

          const auto [fixedFilterSetCandIdxVal, fixedFilterSetCandIdx] = getFixedFilterSetCandIdx(*cs.slice, COMP_Y);

          if (LumaCtbModeHandler::isApsFilter(m) ||
              (LumaCtbModeHandler::isFixedFilter(m) &&
               static_cast<FixFiltIdx>(LumaCtbModeHandler::getFixedFilterIdx(m)) != FixFiltIdx::FIRST))
          {
            // Copy the first fixed filter's results to the CTU buffer and add padding.
            copyAndExtendFixedFilterResultsCtu(fixedFilterSetCandIdx, FixFiltIdx::FIRST, blkDst);

            // Derive the fixed filter results for the second fixed filter.
            deriveSecondFixedFilterResults(buf.get(CompID::COMP_Y), bufDb, m_fixedFilterResultPerCtu, blkSrc, blkDst,
                                           cs, fixedFilterSetCandIdxVal);
          }

          if (LumaCtbModeHandler::isFixedFilter(m))
          {
            copyFixedFilterResults(recYuv, blkDst, CompID::COMP_Y, fixedFilterSetCandIdx,
                                   static_cast<FixFiltIdx>(LumaCtbModeHandler::getFixedFilterIdx(m)));
          }
          else
          {
            const unsigned        apsIdx        = LumaCtbModeHandler::getApsIdx(m);
            const unsigned        altIdx        = LumaCtbModeHandler::getAlternative(m);
            const ClassifierIndex classifierIdx = getClassifierIdxEnum(m_classifierIdxApsLuma[apsIdx][altIdx]);
            const AlfFilterType   filterTypeCtb = m_filterTypeApsLuma[apsIdx];
            const int8_t *const   scaleIdx      = m_scaleIdxApsLuma[apsIdx][altIdx];
            const short *const    coeff         = m_coeffApsLuma[apsIdx][altIdx];
            const Pel *const      clip          = m_clippApsLuma[apsIdx][altIdx];

            // Copy the second fixed filter's results to the CTU buffer and add padding.
            copyAndExtendFixedFilterResultsCtu(fixedFilterSetCandIdx, FixFiltIdx::SECOND, blkDst);

            // Derive the Gaussian filter results, copy them to the CTU buffer and add padding.
            deriveGaussResults(bufDb, blkDst, blkSrc);
            // TODO: ALF: Padding of gaussian filter results is only needed for ALF_FILTER_9_EXT_DB_RESI_DIRECT. BS
            // 2023-09-20
            copyAndExtendGaussResultsCtu(blkDst);

            alfFiltering(m_classifier[classifierIdx], recYuv, bufDb, bufResi, buf, blkDst, blkSrc, CompID::COMP_Y,
                         scaleIdx, coeff, clip, filterTypeCtb, true, fixedFilterSetCandIdx);
          }

          m_ctuAlreadyProcessedFlag[COMP_Y][ctuIdx] = true;
        }

        for (int compIdx = 1; compIdx < MAX_NUM_COMP; compIdx++)
        {
          CompID    compID       = CompID(compIdx);
          const int chromaScaleX = getComponentScaleX(compID, tmpYuv.chromaFormat);
          const int chromaScaleY = getComponentScaleY(compID, tmpYuv.chromaFormat);

          if (CtbModeHandler::isEnabled((*m_modes)[compIdx][ctuIdx]))
          {
            const Area     blkSrc(0, 0, width >> chromaScaleX, height >> chromaScaleY);
            const Area     blkDst(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX,
                                  height >> chromaScaleY);
            const unsigned altIdx = ChromaCtbModeHandler::getAlternative((*m_modes)[compIdx][ctuIdx]);

            PelBuf bufDb = m_tempBufBeforeDbCtu.getBuf(compID).subBuf(0, 0, wBuf, hBuf);
            bufDb.copyFrom(m_tempBufBeforeDb.getBuf(compID).subBuf(xPos - (clipL ? 0 : NUM_DB_PAD),
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

            const auto [fixedFilterSetCandIdxVal, fixedFilterSetCandIdx] = getFixedFilterSetCandIdx(*cs.slice, compID);
            deriveFixedFilterResultsChroma(buf.get(compID), bufDb, blkSrc, blkDst, cs, fixedFilterSetCandIdxVal,
                                           compID);

            alfFiltering(m_classifier[ClassifierIndex::ADAPT_FILTER_GRAD_BASED], recYuv,
                         bufDb,   // Must not be used by the filter.
                         tmpYResi,   // XXX: Luma buffer. Must not be used by the filter.
                         buf, blkDst, blkSrc, compID, m_chromaScaleIdxFinal[altIdx], m_chromaCoeffFinal[altIdx],
                         m_chromaClippFinal[altIdx], m_filterTypeApsChroma, false, fixedFilterSetCandIdx);

            m_ctuAlreadyProcessedFlag[compID][ctuIdx] = true;
          }
          if (sliceCcAlfFilterParam.ccAlfFilterEnabled[compIdx - 1])
          {
            const int filterIdx = m_ccAlfFilterControl[compIdx - 1][ctuIdx];

            if (filterIdx != 0)
            {
              // TODO: don't copy/mirror luma since it's not used
              PelUnitBuf bufSao = m_tempBufSaoCtu.subBuf(UnitArea(cs.area.chromaFormat, Area(0, 0, wBuf, hBuf)));
              bufSao.copyFrom(tmpYuvSao.subBuf(UnitArea(cs.area.chromaFormat,
                                                        Area(xPos - (clipL ? 0 : MAX_ALF_PADDING_SIZE),
                                                             yPos - (clipT ? 0 : MAX_ALF_PADDING_SIZE), wBuf, hBuf))));
              // pad top-left unavailable samples for raster slice
              if (rasterSliceAlfPad & 1)
              {
                bufSao.padBorderPel(MAX_ALF_PADDING_SIZE, 1);
              }
              // pad bottom-right unavailable samples for raster slice
              if (rasterSliceAlfPad & 2)
              {
                bufSao.padBorderPel(MAX_ALF_PADDING_SIZE, 2);
              }
              bufSao.addMirrorExtension(MAX_ALF_PADDING_SIZE);
              bufSao = bufSao.subBuf(
                UnitArea(cs.area.chromaFormat,
                         Area(clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, width, height)));

              const Area blkSrc(0, 0, width, height);
              Area blkDst(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY);

              const int16_t *filterCoeff = m_ccAlfFilterParam.ccAlfCoeff[compIdx - 1][filterIdx - 1];
              m_filterCcAlf(recYuv.get(compID), buf, bufSao, blkDst, blkSrc, compID, filterCoeff, m_clpRngs, cs);
            }
          }
        }
      }
      else
      {
        const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));
        if (CtbModeHandler::isEnabled((*m_modes)[COMP_Y][ctuIdx]))
        {
          Area      blk(xPos, yPos, width, height);
          const int m = (*lumaModes)[ctuIdx];

          deriveClassification(
            tmpYuv.get(COMP_Y),
            (LumaCtbModeHandler::isApsFilter(m) &&
             m_filterTypeApsLuma[LumaCtbModeHandler::getApsIdx(m)] == ALF_FILTER_9_EXT_DB_RESI),
            tmpYResi, tmpYuvBeforeDb.get(COMP_Y), m_ctuPadFlag[ctuIdx], blk, blk, cs,
            LumaCtbModeHandler::isApsFilter(m)
              ? m_classifierIdxApsLuma[LumaCtbModeHandler::getApsIdx(m)][LumaCtbModeHandler::getAlternative(m)]
              : -1);

          const auto [fixedFilterSetCandIdxVal, fixedFilterSetCandIdx] = getFixedFilterSetCandIdx(*cs.slice, COMP_Y);

          if (LumaCtbModeHandler::isApsFilter(m) ||
              (LumaCtbModeHandler::isFixedFilter(m) &&
               static_cast<FixFiltIdx>(LumaCtbModeHandler::getFixedFilterIdx(m)) != FixFiltIdx::FIRST))
          {
            // Setup the second fixed filter's input by extending the first fixed filter's output.
            deriveFixedFilterResultsCtuBoundary(tmpYuv.get(COMP_Y), tmpYuvBeforeDb.get(COMP_Y), blk,
                                                cs.slice->m_iSliceQp, ctuIdx, fixedFilterSetCandIdx, FixFiltIdx::FIRST);

            // Derive the fixed filter output for the second filter.
            deriveSecondFixedFilterResults(tmpYuv.get(CompID::COMP_Y), m_tempBufBeforeDb.get(COMP_Y),
                                           m_fixFilterResult[COMP_Y], blk, blk, cs, fixedFilterSetCandIdxVal);
          }

          if (LumaCtbModeHandler::isFixedFilter(m))
          {
            copyFixedFilterResults(recYuv, blk, CompID::COMP_Y, fixedFilterSetCandIdx,
                                   static_cast<FixFiltIdx>(LumaCtbModeHandler::getFixedFilterIdx(m)));
          }
          else
          {
            const unsigned        apsIdx        = LumaCtbModeHandler::getApsIdx(m);
            const unsigned        altIdx        = LumaCtbModeHandler::getAlternative(m);
            const ClassifierIndex classifierIdx = getClassifierIdxEnum(m_classifierIdxApsLuma[apsIdx][altIdx]);
            const AlfFilterType   filterTypeCtb = m_filterTypeApsLuma[apsIdx];
            const int8_t *const   scaleIdx      = m_scaleIdxApsLuma[apsIdx][altIdx];
            const short *const    coeff         = m_coeffApsLuma[apsIdx][altIdx];
            const Pel *const      clip          = m_clippApsLuma[apsIdx][altIdx];

            // Setup the adaptive filter's fixed-filter input by extending the second fixed filter's output.
            const bool isFixFiltPaddedPerCtu = isFixedFilterPaddedPerCtu(*cs.slice);
            if (isFixFiltPaddedPerCtu)
            {
              copyAndExtendFixedFilterResultsCtu(fixedFilterSetCandIdx, FixFiltIdx::SECOND, blk);
            }
            else
            {
              deriveFixedFilterResultsCtuBoundary(tmpYuv.get(CompID::COMP_Y), tmpYuvBeforeDb.get(COMP_Y), blk,
                                                  cs.slice->m_iSliceQp, ctuIdx, fixedFilterSetCandIdx,
                                                  FixFiltIdx::SECOND);
            }

            deriveGaussResults(tmpYuvBeforeDb.get(COMP_Y), blk, blk);
            // TODO: ALF: Extension of gaussian filter results is only needed for ALF_FILTER_9_EXT_DB_RESI_DIRECT. BS
            // 2023-09-20
            if (isFixFiltPaddedPerCtu)
            {
              copyAndExtendGaussResultsCtu(blk);
            }
            else
            {
              deriveGaussResultsCtuBoundary(tmpYuvBeforeDb.get(COMP_Y), blk, ctuIdx);
            }

            alfFiltering(m_classifier[classifierIdx], recYuv, tmpYuvBeforeDb.get(COMP_Y), tmpYResi, tmpYuv, blk, blk,
                         CompID::COMP_Y, scaleIdx, coeff, clip, filterTypeCtb, isFixFiltPaddedPerCtu,
                         fixedFilterSetCandIdx);
          }

          m_ctuAlreadyProcessedFlag[COMP_Y][ctuIdx] = true;
        }

        for (int compIdx = 1; compIdx < MAX_NUM_COMP; compIdx++)
        {
          CompID    compID       = CompID(compIdx);
          const int chromaScaleX = getComponentScaleX(compID, tmpYuv.chromaFormat);
          const int chromaScaleY = getComponentScaleY(compID, tmpYuv.chromaFormat);

          if (CtbModeHandler::isEnabled((*m_modes)[compIdx][ctuIdx]))
          {
            Area blk(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY);
            const unsigned altIdx = ChromaCtbModeHandler::getAlternative((*m_modes)[compIdx][ctuIdx]);

            const auto [fixedFilterSetCandIdxVal, fixedFilterSetCandIdx] = getFixedFilterSetCandIdx(*cs.slice, compID);
            deriveFixedFilterResultsChroma(tmpYuv.get(compID), tmpYuvBeforeDb.get(compID), blk, blk, cs,
                                           fixedFilterSetCandIdxVal, compID);
            deriveFixedFilterResultsCtuBoundaryChroma(tmpYuv.get(compID), tmpYuvBeforeDb.get(compID), blk,
                                                      cs.slice->m_iSliceQp + cs.slice->getSliceChromaQpDelta(compID),
                                                      ctuIdx, fixedFilterSetCandIdx, compID);

            alfFiltering(m_classifier[ClassifierIndex::ADAPT_FILTER_GRAD_BASED], recYuv,
                         tmpYuvBeforeDb.get(compID),   // Must not be used by the filter
                         tmpYResi,   // XXX: Luma buffer. Must not be used by the filter.
                         tmpYuv, blk, blk, compID, m_chromaScaleIdxFinal[altIdx], m_chromaCoeffFinal[altIdx],
                         m_chromaClippFinal[altIdx], m_filterTypeApsChroma, false, fixedFilterSetCandIdx);

            m_ctuAlreadyProcessedFlag[compID][ctuIdx] = true;
          }
          if (sliceCcAlfFilterParam.ccAlfFilterEnabled[compIdx - 1])
          {
            const int filterIdx = m_ccAlfFilterControl[compIdx - 1][ctuIdx];

            if (filterIdx != 0)
            {
              Area blkDst(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY);
              Area blkSrc(xPos, yPos, width, height);

              const int16_t *filterCoeff = m_ccAlfFilterParam.ccAlfCoeff[compIdx - 1][filterIdx - 1];
              m_filterCcAlf(recYuv.get(compID), tmpYuv, tmpYuvSao, blkDst, blkSrc, compID, filterCoeff, m_clpRngs, cs);
            }
          }
        }
      }
      ctuIdx++;
    }
  }
}

void AdaptiveLoopFilterEcm::copyAddInputsBeforeDBF(const CodingStructure &cs, TrQuant &trQuant)
{
  // Copy the pre-DBF reconstruction.
  m_tempBufBeforeDb.copyFrom(cs.getRecoBuf());

  // Copy the residual.
  for (TransformUnit *const tu: cs.tus)
  {
    PelBuf tuResiBuf = m_tempBufResi.subBuf(tu->lumaPos(), tu->lumaSize());
    if (TU::getCbf(*tu, CompID::COMP_Y))
    {
      const QpParam cQp(*tu, CompID::COMP_Y);
      trQuant.invTransformNxN(*tu, CompID::COMP_Y, tuResiBuf, cQp);
    }
    else
    {
      tuResiBuf.fill(0);
    }
  }
}

void AdaptiveLoopFilterEcm::reconstructCoeffAPSs(CodingStructure &cs, bool luma, bool chroma, bool isRdo)
{
  // luma
  APS    **aps = cs.slice->m_alfApss;
  AlfParam alfParamTmp;
  APS     *curAPS;
  if (luma)
  {
    for (int i = 0; i < cs.slice->m_numAlfApsIdsLuma; i++)
    {
      int apsIdx = cs.slice->m_alfApsIdsLuma[i];
      curAPS     = aps[apsIdx];
      CHECK(curAPS == nullptr, "invalid APS");
      alfParamTmp = curAPS->m_alfAPSParam.getEcmParam();
      reconstructLumaCoeff(alfParamTmp, isRdo);
      memcpy(m_scaleIdxApsLuma[i], m_scaleIdxFinal, sizeof(m_scaleIdxFinal));
      memcpy(m_coeffApsLuma[i], m_coeffFinal, sizeof(m_coeffFinal));
      memcpy(m_clippApsLuma[i], m_clippFinal, sizeof(m_clippFinal));
      memcpy(m_classifierIdxApsLuma[i], m_classifierFinal, sizeof(m_classifierFinal));
      m_filterTypeApsLuma[i] = alfParamTmp.filterType[ChannelType::LUMA];
      m_numLumaAltAps[i]     = alfParamTmp.numAlternativesLuma;
    }
  }

  // chroma
  if (chroma)
  {
    int apsIdxChroma = cs.slice->m_alfApsIdChroma;
    curAPS           = aps[apsIdxChroma];
    alfParamTmp      = curAPS->m_alfAPSParam.getEcmParam();
    reconstructChromaCoeff(alfParamTmp, isRdo);
    m_filterTypeApsChroma = alfParamTmp.filterType[ChannelType::CHROMA];
  }
}

void AdaptiveLoopFilterEcm::reconstructLumaCoeff(AlfParam &alfParam, const bool isRdo)
{
  const int factor         = 0;
  const int filterStatIdx  = m_filterTypeToStatIndex[ChannelType::LUMA][alfParam.filterType[ChannelType::LUMA]];
  const int numCoeff       = m_filterShapes[ChannelType::LUMA][filterStatIdx].numCoeff;
  const int numAlts        = alfParam.numAlternativesLuma;
  const int numCoeffMinus1 = numCoeff - 1;

  for (int altIdx = 0; altIdx < numAlts; ++altIdx)
  {
    const bool   nonLinearFlag = alfParam.lumaNonLinearFlag[altIdx];
    const auto  &coeffDeltaIdx = alfParam.filterCoeffDeltaIdx[altIdx];
    const int    numFilters    = alfParam.numLumaFilters[altIdx];
    auto        &coeffSrc      = alfParam.lumaCoeff[altIdx];
    const auto  &clippSrc      = alfParam.lumaClipp[altIdx];
    short *const coeffDst      = m_coeffFinal[altIdx];
    Pel *const   clippDst      = m_clippFinal[altIdx];

    // TODO: ALF: Why do we change the source coefficients? BS 2023-12-06
    for (int filterIdx = 0; filterIdx < numFilters; ++filterIdx)
    {
      short *coeff          = &coeffSrc[filterIdx * MAX_NUM_ALF_LUMA_COEFF];
      coeff[numCoeffMinus1] = factor;
      for (size_t coeffIdx = 0; coeffIdx < numCoeffMinus1; ++coeffIdx)
      {
        CHECK(coeff[coeffIdx] > (1 << COEFF_SCALE_BITS_LUMA) || coeff[coeffIdx] < -(1 << COEFF_SCALE_BITS_LUMA),
              "AlfCoeff shall be in the range of [minValue, maxValue]");
        CHECK(!isCoeffRestricted(coeff[coeffIdx], true), "AlfCoeff shall be restricted to certain values");
      }
    }

    for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; ++classIdx)
    {
      const int filterIdx = coeffDeltaIdx[classIdx];
      CHECK(!(filterIdx >= 0 && filterIdx < numFilters), "Bad coeff delta idx in ALF");

      m_scaleIdxFinal[altIdx][classIdx] = alfParam.lumaScaleIdx[altIdx][filterIdx];

      reconstructCoeffSingleFilter(
        &coeffSrc[filterIdx * MAX_NUM_ALF_LUMA_COEFF], &clippSrc[filterIdx * MAX_NUM_ALF_LUMA_COEFF],
        &coeffDst[classIdx * MAX_NUM_ALF_LUMA_COEFF], &clippDst[classIdx * MAX_NUM_ALF_LUMA_COEFF],
        m_alfClippingValues[ChannelType::LUMA], numCoeffMinus1, nonLinearFlag, isRdo, factor);
    }

    m_classifierFinal[altIdx] = alfParam.lumaClassifierIdx[altIdx];
  }
}

void AdaptiveLoopFilterEcm::reconstructChromaCoeff(AlfParam &alfParam, const bool isRdo)
{
  const int factor         = 0;
  const int filterStatIdx  = m_filterTypeToStatIndex[ChannelType::CHROMA][alfParam.filterType[ChannelType::CHROMA]];
  const int numCoeff       = m_filterShapes[ChannelType::CHROMA][filterStatIdx].numCoeff;
  const int numAlts        = alfParam.numAlternativesChroma;
  const int numCoeffMinus1 = numCoeff - 1;

  for (int altIdx = 0; altIdx < numAlts; ++altIdx)
  {
    const bool nonLinearFlag = alfParam.chromaNonLinearFlag[altIdx];

    // TODO: ALF: Why do we change the source coefficient? BS 2023-12-06
    alfParam.chromaCoeff[altIdx][numCoeffMinus1] = factor;
    for (size_t coeffIdx = 0; coeffIdx < numCoeffMinus1; ++coeffIdx)
    {
      CHECK(alfParam.chromaCoeff[altIdx][coeffIdx] > (1 << COEFF_SCALE_BITS_CHROMA) ||
              alfParam.chromaCoeff[altIdx][coeffIdx] < -(1 << COEFF_SCALE_BITS_CHROMA),
            "AlfCoeff shall be in the range of [minValue, maxValue]");
      CHECK(!isCoeffRestricted(alfParam.chromaCoeff[altIdx][coeffIdx], false),
            "AlfCoeff shall be restricted to certain values");
    }

    m_chromaScaleIdxFinal[altIdx][0] = alfParam.chromaScaleIdx[altIdx][0];

    reconstructCoeffSingleFilter(alfParam.chromaCoeff[altIdx].data(), alfParam.chromaClipp[altIdx].data(),
                                 m_chromaCoeffFinal[altIdx], m_chromaClippFinal[altIdx],
                                 m_alfClippingValues[ChannelType::CHROMA], numCoeffMinus1, nonLinearFlag, isRdo,
                                 factor);
  }
}

void AdaptiveLoopFilterEcm::create(const int picWidth, const int picHeight, const ChromaFormat format,
                                   const int maxCUWidth, const int maxCUHeight, const int maxCUDepth,
                                   const BitDepths &inputBitDepth)
{
  AdaptiveLoopFilter::create(picWidth, picHeight, format, maxCUWidth, maxCUHeight, maxCUDepth, inputBitDepth);

  m_filterShapesCcAlf.push_back(AlfFilterShape(CC_ALF));

  m_filterShapes[ChannelType::LUMA].push_back(AlfFilterShape(ALF_FILTER_9_EXT_DB_RESI_DIRECT));
  m_filterShapes[ChannelType::LUMA].push_back(AlfFilterShape(ALF_FILTER_9_EXT_DB_RESI));

  m_filterShapes[ChannelType::CHROMA].push_back(AlfFilterShape(ALF_FILTER_9));

  memset(m_filterTypeToStatIndex[ChannelType::LUMA], 0, sizeof(m_filterTypeToStatIndex[ChannelType::LUMA]));
  m_filterTypeToStatIndex[ChannelType::LUMA][ALF_FILTER_9_EXT_DB_RESI_DIRECT] = 0;
  m_filterTypeToStatIndex[ChannelType::LUMA][ALF_FILTER_9_EXT_DB_RESI]        = 1;

  memset(m_filterTypeToStatIndex[ChannelType::CHROMA], 0, sizeof(m_filterTypeToStatIndex[ChannelType::CHROMA]));
  m_filterTypeToStatIndex[ChannelType::CHROMA][ALF_FILTER_9] = 0;

  memset(m_filterTypeTest[ChannelType::LUMA], 0, sizeof(m_filterTypeTest[ChannelType::LUMA]));
  m_filterTypeTest[ChannelType::LUMA][ALF_FILTER_9_EXT_DB_RESI_DIRECT] = true;
  m_filterTypeTest[ChannelType::LUMA][ALF_FILTER_9_EXT_DB_RESI]        = true;

  memset(m_filterTypeTest[ChannelType::CHROMA], 0, sizeof(m_filterTypeTest[ChannelType::CHROMA]));
  m_filterTypeTest[ChannelType::CHROMA][ALF_FILTER_9] = true;

  m_tempBuf.destroy();
  // NOTE: make border 1 sample wider to avoid out-of-bounds memory access in SIMD code (simdDeriveClassificationBlk
  // function)
  m_tempBuf.create(format, Area(0, 0, picWidth, picHeight), maxCUWidth, MAX_FILTER_LENGTH_FIXED, 0, false);
  m_tempBuf2.destroy();
  // Double padding size is needed because of implementation of data copy and extension for boundary blocks.
  m_tempBuf2.create(format,
                    Area(0, 0, maxCUWidth + (MAX_ALF_PADDING_SIZE << 1), maxCUHeight + (MAX_ALF_PADDING_SIZE << 1)),
                    maxCUWidth, MAX_ALF_PADDING_SIZE, 0, false);

  // TODO: ALF: Only needed if filters with DBF are enabled. BS 2023-08-31
  m_tempBufBeforeDb.destroy();
  m_tempBufBeforeDb.create(format, Area(0, 0, picWidth, picHeight), maxCUWidth, NUM_DB_PAD, 0, false);
  m_tempBufBeforeDbCtu.destroy();
  // Double padding size is needed because of implementation of data copy and extension for boundary blocks.
  m_tempBufBeforeDbCtu.create(format, Area(0, 0, maxCUWidth + (NUM_DB_PAD << 1), maxCUHeight + (NUM_DB_PAD << 1)),
                              maxCUWidth, NUM_DB_PAD, 0, false);

  m_tempBufResi.destroy();
  m_tempBufResi.create(Size(picWidth, picHeight), maxCUWidth, MAX_FILTER_LENGTH_FIXED, 0);
  m_tempBufResiCtu.destroy();
  // Double padding size is needed because of implementation of data copy and extension for boundary blocks.
  m_tempBufResiCtu.create(Size(maxCUWidth + (MAX_ALF_PADDING_SIZE << 1), maxCUHeight + (MAX_ALF_PADDING_SIZE << 1)),
                          maxCUWidth, MAX_ALF_PADDING_SIZE, 0);

  // TODO: don't create luma buffer since it's not used
  m_tempBufSao.destroy();
  m_tempBufSao.create(format, Area(0, 0, picWidth, picHeight), maxCUWidth, MAX_FILTER_LENGTH_FIXED, 0, false);
  m_tempBufSaoCtu.destroy();
  m_tempBufSaoCtu.create(
    format, Area(0, 0, maxCUWidth + (MAX_ALF_PADDING_SIZE << 1), maxCUHeight + (MAX_ALF_PADDING_SIZE << 1)), maxCUWidth,
    MAX_ALF_PADDING_SIZE, 0, false);

  for (int compIdx = 0; compIdx < MAX_NUM_COMP; ++compIdx)
  {
    m_fixFilterResult[compIdx].destroy();
    if (isLuma(CompID(compIdx)))
    {
      m_fixFilterResult[compIdx].create(Size(m_picWidth, m_picHeight), 0, ALF_PADDING_SIZE_FIXED_RESULTS, 0);
    }
    else
    {
      // Only one fixed filter (indexed by `FixFiltIdx::FIRST`) is used for chroma
      for (auto &buf: m_fixFilterResult[compIdx])
      {
        buf[FixFiltIdx::FIRST].create(Size(m_picWidthChroma, m_picHeightChroma), 0, ALF_PADDING_SIZE_FIXED_RESULTS, 0);
      }
    }
  }

  m_fixedFilterResultPerCtu.destroy();
  m_fixedFilterResultPerCtu.create(Size(m_maxCUWidth, m_maxCUHeight), 0, ALF_PADDING_SIZE_FIXED_RESULTS, 0);

  m_fixFilterResiResult.destroy();
  m_fixFilterResiResult.create(Size(picWidth, picHeight));

  for (int compIdx = 0; compIdx < MAX_NUM_COMP; ++compIdx)
  {
    m_ctuAlreadyProcessedFlag[compIdx].resize(m_numCTUsInPic, false);
  }
  m_ctuPadFlag.resize(m_numCTUsInPic, 0);

  m_gaussPic.destroy();
  m_gaussCtu.destroy();
  m_gaussPic.create(Size(picWidth, picHeight), 0, ALF_PADDING_SIZE_GAUSS_RESULTS, 0);
  m_gaussCtu.create(Size(m_maxCUWidth, m_maxCUHeight), 0, ALF_PADDING_SIZE_GAUSS_RESULTS, 0);

  // Classification
  m_classifier.create(Size(picWidth, picHeight));
}

void AdaptiveLoopFilterEcm::destroy()
{
  if (!m_created)
  {
    return;
  }
  AdaptiveLoopFilter::destroy();

  m_classifier.destroy();

  // TODO: ALF: Only needed if filter has a DBF input. BS 2023-09-06
  m_tempBufBeforeDb.destroy();
  m_tempBufBeforeDbCtu.destroy();

  m_tempBufResi.destroy();
  m_tempBufResiCtu.destroy();

  m_tempBufSao.destroy();
  m_tempBufSaoCtu.destroy();

  for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
  {
    m_fixFilterResult[compIdx].destroy();
  }
  m_fixedFilterResultPerCtu.destroy();

  m_fixFilterResiResult.destroy();

  m_gaussPic.destroy();
  m_gaussCtu.destroy();

  for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
  {
    m_ctuAlreadyProcessedFlag[compIdx].clear();
  }
  m_ctuPadFlag.clear();

  m_filterShapes[ChannelType::LUMA].clear();
  m_filterShapes[ChannelType::CHROMA].clear();
  m_filterShapesCcAlf.clear();
}

void AdaptiveLoopFilterEcm::deriveClassification(const CPelBuf &srcLuma, const bool bResiFixed,
                                                 const CPelBuf &srcResiLuma, const CPelBuf &srcLumaBeforeDb,
                                                 const uint8_t ctuPadFlag, const Area &blkDst, const Area &blk,
                                                 CodingStructure &cs, const int multipleClassifierIdx)
{
  // TODO (AW): I think this is the wrong condition which misses CTU-based QP-adaptation!
  if (cs.slice->getCuQpDeltaSubdiv())
  {
    UnitArea curArea(cs.area.chromaFormat, blkDst);
    for (auto &currCU: cs.traverseCUs(curArea, ChannelType::LUMA))
    {
      deriveClassificationAndFixFilterResultsBlk(
        srcLuma, bResiFixed, srcResiLuma, srcLumaBeforeDb, 0,
        Area(currCU.lumaPos().x, currCU.lumaPos().y, currCU.lwidth(), currCU.lheight()),
        Area(currCU.lumaPos().x, currCU.lumaPos().y, currCU.lwidth(), currCU.lheight()), cs, currCU.qp,
        multipleClassifierIdx);
    }
  }
  else
  {
    int height = blk.pos().y + blk.height;
    int width  = blk.pos().x + blk.width;

    for (int i = blk.pos().y; i < height; i += CLASSIFICATION_BLK_SIZE)
    {
      int nHeight = std::min(i + CLASSIFICATION_BLK_SIZE, height) - i;

      for (int j = blk.pos().x; j < width; j += CLASSIFICATION_BLK_SIZE)
      {
        int nWidth = std::min(j + CLASSIFICATION_BLK_SIZE, width) - j;
        deriveClassificationAndFixFilterResultsBlk(
          srcLuma, bResiFixed, srcResiLuma, srcLumaBeforeDb, (j == blk.pos().x && i == blk.pos().y) ? ctuPadFlag : 0,
          Area(j - blk.pos().x + blkDst.pos().x, i - blk.pos().y + blkDst.pos().y, nWidth, nHeight),
          Area(j, i, nWidth, nHeight), cs, cs.slice->m_iSliceQp, multipleClassifierIdx);
      }
    }
  }
}

void AdaptiveLoopFilterEcm::deriveClassificationAndFixFilterResultsBlk(const CPelBuf &srcLuma, const bool bResiFixed,
                                                                       const CPelBuf &srcResiLuma,
                                                                       const CPelBuf &srcLumaBeforeDb,
                                                                       const uint8_t ctuPadFlag, const Area &blkDst,
                                                                       const Area &blk, CodingStructure &cs, int qp,
                                                                       const int multipleClassifierIdx)
{
  m_deriveVariance(srcLuma, blk, m_laplacian);
  m_deriveClassificationLaplacian(srcLuma, blk, m_laplacian, ALF_CLASSIFIER_FL);

  const FixedFilterSetCands fixedFilterSetCandIdcs(qp);

  // Calculate fixed filter classification for a gradient window size of 4x4.
  m_calcClassGradBasedFixedFilt(m_classifier[ClassifierIndex::FIXED_FILTER_4x4], blkDst, blk, ClsWndwSize::WNDW_4x4,
                                m_inputBitDepth[ChannelType::LUMA], m_laplacian);

  const unsigned xShift = ctuPadFlag & 0x01 ? std::min(blkDst.width, 16U) : 0;
  const unsigned yShift = ctuPadFlag & 0x02 ? std::min(blkDst.height, (unsigned)ALF_PADDING_SIZE_FIXED_RESULTS * 2) : 0;
  const Area     blkDstNew(blkDst.x + xShift, blkDst.y + yShift, blkDst.width - xShift, blkDst.height - yShift);
  const Area     blkNew(blk.x + xShift, blk.y + yShift, blk.width - xShift, blk.height - yShift);

  const int sliceFixedFilterSetCandIdx = cs.slice->m_newAlfFixFiltSetCandIdx[COMP_Y];
  for (int fixedFilterSetCandIdx = 0; fixedFilterSetCandIdx < NUM_FIXED_FILTER_SET_CANDS; ++fixedFilterSetCandIdx)
  {
    if (fixedFilterSetCandIdx == sliceFixedFilterSetCandIdx || sliceFixedFilterSetCandIdx == -1)
    {
      // Calculate the fixed filter results for the first fixed filter from the current candidate set.
      const FixFiltSetCand fixedFilterSetCand = static_cast<FixFiltSetCand>(fixedFilterSetCandIdx);
      const unsigned       fixedFilterSetIdx  = fixedFilterSetCandIdcs[fixedFilterSetCand];
      if (blkDst.width > xShift && blkDst.height > yShift)
      {
        alfFirstFixedFilterBlk(srcLuma, srcLumaBeforeDb, blkNew,
                               m_fixFilterResult[COMP_Y][fixedFilterSetCand][FixFiltIdx::FIRST], blkDstNew,
                               fixedFilterSetIdx);
      }

      if (bResiFixed)
      {
        // Calculate the fixed filter results for the residual-based fixed filter from the other candidate set.
        static_assert(NUM_FIXED_FILTER_SET_CANDS == 2, "Implementation only valid for two candidates.");
        const FixFiltSetCand fixedFilterSetCandResi =
          fixedFilterSetCand == FixFiltSetCand::FIRST ? FixFiltSetCand::SECOND : FixFiltSetCand::FIRST;
        const unsigned fixedFilterSetIdxResi = fixedFilterSetCandIdcs[fixedFilterSetCandResi];
        m_filterResi9x9Blk(m_classifier[ClassifierIndex::FIXED_FILTER_4x4], srcResiLuma, blk, blkDst,
                           m_fixFilterResiResult[fixedFilterSetCandResi], m_picWidth, fixedFilterSetIdxResi,
                           m_classIdnFixedFilter9Db9[fixedFilterSetIdxResi], m_clpRngs.comp[CompID::COMP_Y],
                           m_alfClippingValues[ChannelType::LUMA]);
      }
    }
  }

  // Calculate the laplacians and the fixed filter classification for a window size of 12x12.
  m_deriveClassificationLaplacianBig(blk, m_laplacian);
  m_calcClassGradBasedFixedFilt(m_classifier[ClassifierIndex::FIXED_FILTER_12x12], blkDst, blk, ClsWndwSize::WNDW_12x12,
                                m_inputBitDepth[ChannelType::LUMA], m_laplacian);

  if (multipleClassifierIdx == ALF_NUM_CLASSIFIER || multipleClassifierIdx == 0)
  {
    m_calcClassGradBasedAdaptFilt(m_classifier[ClassifierIndex::ADAPT_FILTER_GRAD_BASED], blkDst,
                                  Area(blk.pos().x, blk.pos().y, blk.width, blk.height), ClsWndwSize::WNDW_12x12,
                                  m_inputBitDepth[ChannelType::LUMA], m_laplacian);
  }

  for (int curClassifierIdx = 1; curClassifierIdx < ALF_NUM_CLASSIFIER; curClassifierIdx++)
  {
    static_assert(ALF_NUM_CLASSIFIER <= 3, "The number of classifiers is greater than expected.");
    const auto currClassifier = curClassifierIdx == 1 ? ClassifierIndex::ADAPT_FILTER_BAND_BASED_RECO
                                                      : ClassifierIndex::ADAPT_FILTER_BAND_BASED_RESI;
    if (multipleClassifierIdx == ALF_NUM_CLASSIFIER || curClassifierIdx == multipleClassifierIdx)
    {
      if (multipleClassifierIdx == ALF_NUM_CLASSIFIER && curClassifierIdx == 2 && cs.slice->isIntra())
      {
        continue;
      }

      m_calcClassBandBased(m_classifier[currClassifier], blk,
                           Area(blkDst.pos().x, blkDst.pos().y, blkDst.width, blkDst.height), srcLuma, curClassifierIdx,
                           m_inputBitDepth[ChannelType::LUMA], srcResiLuma, m_laplacian[0]);
    }
  }
}

void AdaptiveLoopFilterEcm::deriveClassificationLaplacian(const CPelBuf &srcLuma, const Area &blk,
                                                          LaplacianBuf &laplacian, const int side)
{
  const int       fl      = side;
  const int       fl2     = 2 * fl;
  const Pel      *src     = srcLuma.buf;
  const ptrdiff_t stride  = srcLuma.stride;
  const ptrdiff_t stride2 = 2 * stride;
  const ptrdiff_t yOffset = (blk.pos().y - fl) * stride;
  const Pel      *src0    = &src[yOffset - stride];
  const Pel      *src1    = &src[yOffset];
  const Pel      *src2    = &src[yOffset + stride];
  const Pel      *src3    = &src[yOffset + stride2];

  // Calculate the sums of the laplacian gradients for non-overlapping 2x2 blocks.
  for (int i = 0; i < blk.height + fl2; i += 2)
  {
    for (int j = 0; j < blk.width + fl2; j += 2)
    {
      const int  xOffset = blk.pos().x - fl + j;
      const Pel *pY      = src1 + xOffset;
      const Pel *pYup    = src0 + xOffset;
      const Pel *pYdn    = src2 + xOffset;
      const Pel *pYdn2   = src3 + xOffset;

      const Pel y0  = pY[0] << 1;
      const Pel y01 = pY[1] << 1;
      const Pel y1  = pYdn[0] << 1;
      const Pel y11 = pYdn[1] << 1;

      laplacian[VER][i >> 1][j >> 1] = abs(y0 - pYup[0] - pYdn[0]) + abs(y01 - pYup[1] - pYdn[1]) +
        abs(y1 - pY[0] - pYdn2[0]) + abs(y11 - pY[1] - pYdn2[1]);
      laplacian[HOR][i >> 1][j >> 1] = abs(y0 - pY[-1] - pY[1]) + abs(y01 - pY[0] - pY[2]) +
        abs(y1 - pYdn[-1] - pYdn[1]) + abs(y11 - pYdn[0] - pYdn[2]);
      laplacian[DIAG0][i >> 1][j >> 1] = abs(y0 - pYup[-1] - pYdn[1]) + abs(y01 - pYup[0] - pYdn[2]) +
        abs(y1 - pY[-1] - pYdn2[1]) + abs(y11 - pY[0] - pYdn2[2]);
      laplacian[DIAG1][i >> 1][j >> 1] = abs(y0 - pYup[1] - pYdn[-1]) + abs(y01 - pYup[2] - pYdn[0]) +
        abs(y1 - pY[1] - pYdn2[-1]) + abs(y11 - pY[2] - pYdn2[0]);
    }
    src0 = src0 + stride2;
    src1 = src1 + stride2;
    src2 = src2 + stride2;
    src3 = src3 + stride2;
  }

  // Calculate the sums of the laplacian gradients for overlapping 4x4 blocks.
  for (int i = 0; i < (blk.height + fl2) >> 1; i++)
  {
    for (int j = 1; j < (blk.width + fl2) >> 1; j++)
    {
      // Sum of the laplacian gradients for a 4x2 block.
      int jM1                  = j - 1;
      laplacian[VER][i][jM1]   = laplacian[VER][i][jM1] + laplacian[VER][i][j];
      laplacian[HOR][i][jM1]   = laplacian[HOR][i][jM1] + laplacian[HOR][i][j];
      laplacian[DIAG0][i][jM1] = laplacian[DIAG0][i][jM1] + laplacian[DIAG0][i][j];
      laplacian[DIAG1][i][jM1] = laplacian[DIAG1][i][jM1] + laplacian[DIAG1][i][j];
    }

    if (i > 0)
    {
      int iM1 = i - 1;
      for (int j = 0; j < ((blk.width + fl2) >> 1) - 1; j++)
      {
        // Sum of the laplacian gradients for a 4x4 block.
        laplacian[VER][iM1][j]   = laplacian[VER][iM1][j] + laplacian[VER][i][j];
        laplacian[HOR][iM1][j]   = laplacian[HOR][iM1][j] + laplacian[HOR][i][j];
        laplacian[DIAG0][iM1][j] = laplacian[DIAG0][iM1][j] + laplacian[DIAG0][i][j];
        laplacian[DIAG1][iM1][j] = laplacian[DIAG1][iM1][j] + laplacian[DIAG1][i][j];
      }
    }
  }
}

void AdaptiveLoopFilterEcm::deriveClassificationLaplacianBig(const Area &curBlk, LaplacianBuf &laplacian)
{
  int fl2 = ALF_CLASSIFIER_FL << 1;
  for (int i = 0; i < (curBlk.height + fl2) >> 1; i++)
  {
    for (int j = 4; j <= (curBlk.width + fl2 - 2) >> 1; j++)
    {
      int jM2 = j - 2;
      int jM4 = j - 4;
      laplacian[VER][i][jM4] += laplacian[VER][i][jM2] + laplacian[VER][i][j];
      laplacian[HOR][i][jM4] += laplacian[HOR][i][jM2] + laplacian[HOR][i][j];
      laplacian[DIAG0][i][jM4] += laplacian[DIAG0][i][jM2] + laplacian[DIAG0][i][j];
      laplacian[DIAG1][i][jM4] += laplacian[DIAG1][i][jM2] + laplacian[DIAG1][i][j];
    }
    if (i >= 4)
    {
      int iM4 = i - 4;
      int iM2 = i - 2;
      for (int j = 0; j <= (curBlk.width + fl2 - 11) >> 1; j++)
      {
        laplacian[VER][iM4][j] += laplacian[VER][iM2][j] + laplacian[VER][i][j];
        laplacian[HOR][iM4][j] += laplacian[HOR][iM2][j] + laplacian[HOR][i][j];
        laplacian[DIAG0][iM4][j] += laplacian[DIAG0][iM2][j] + laplacian[DIAG0][i][j];
        laplacian[DIAG1][iM4][j] += laplacian[DIAG1][iM2][j] + laplacian[DIAG1][i][j];
      }
    }
  }
}

void AdaptiveLoopFilterEcm::deriveVariance(const CPelBuf &srcLuma, const Area &blk, LaplacianBuf &laplacian)
{
  const int       fl         = DIST_CLASS;
  const ptrdiff_t stride     = srcLuma.stride;
  const ptrdiff_t stride2    = 2 * stride;
  const Pel      *src        = srcLuma.buf + (blk.pos().y - fl) * stride + blk.pos().x - fl;
  const int       numSample  = (fl * 2 + 2) * (fl * 2 + 2);
  const int       numSample2 = 128 * 128;
  const int       offset     = numSample2 >> 1;
  const int       fl2        = fl << 1;
  for (int i = 0; i < blk.height + fl2; i += 2)
  {
    const Pel *src1      = src + stride;
    const int  iOffset   = i >> 1;
    const int  iOffsetM4 = iOffset - 4;
    for (int j = 0; j < blk.width + fl2; j += 2)
    {
      int jOffset   = j >> 1;
      int jOffsetM4 = jOffset - 4;

      // Sum over 2x2 sliding window.
      laplacian[0][iOffset][jOffset] = src[j] + src[j + 1] + src1[j] + src1[j + 1];
      laplacian[1][iOffset][jOffset] =
        src[j] * src[j] + src[j + 1] * src[j + 1] + src1[j] * src1[j] + src1[j + 1] * src1[j + 1];

      if (jOffsetM4 >= 0)
      {
        // Sum over 10x2 sliding window.
        if (jOffsetM4 == 0)
        {
          laplacian[2][iOffset][jOffsetM4] = laplacian[0][iOffset][jOffset - 4] + laplacian[0][iOffset][jOffset - 3] +
            laplacian[0][iOffset][jOffset - 2] + laplacian[0][iOffset][jOffset - 1] + laplacian[0][iOffset][jOffset];
          laplacian[3][iOffset][jOffsetM4] = laplacian[1][iOffset][jOffset - 4] + laplacian[1][iOffset][jOffset - 3] +
            laplacian[1][iOffset][jOffset - 2] + laplacian[1][iOffset][jOffset - 1] + laplacian[1][iOffset][jOffset];
        }
        else
        {
          laplacian[2][iOffset][jOffsetM4] =
            laplacian[2][iOffset][jOffset - 5] - laplacian[0][iOffset][jOffset - 5] + laplacian[0][iOffset][jOffset];
          laplacian[3][iOffset][jOffsetM4] =
            laplacian[3][iOffset][jOffset - 5] - laplacian[1][iOffset][jOffset - 5] + laplacian[1][iOffset][jOffset];
        }

        if (iOffsetM4 >= 0)
        {
          // Sum over 10x10 sliding window.
          if (iOffsetM4 == 0)
          {
            laplacian[0][iOffsetM4][jOffsetM4] = laplacian[2][iOffsetM4][jOffsetM4] +
              laplacian[2][iOffset - 3][jOffsetM4] + laplacian[2][iOffset - 2][jOffsetM4] +
              laplacian[2][iOffset - 1][jOffsetM4] + laplacian[2][iOffset][jOffsetM4];
            laplacian[1][iOffsetM4][jOffsetM4] = laplacian[3][iOffsetM4][jOffsetM4] +
              laplacian[3][iOffset - 3][jOffsetM4] + laplacian[3][iOffset - 2][jOffsetM4] +
              laplacian[3][iOffset - 1][jOffsetM4] + laplacian[3][iOffset][jOffsetM4];
          }
          else
          {
            laplacian[0][iOffsetM4][jOffsetM4] = laplacian[0][iOffsetM4 - 1][jOffsetM4] -
              laplacian[2][iOffsetM4 - 1][jOffsetM4] + laplacian[2][iOffset][jOffsetM4];
            laplacian[1][iOffsetM4][jOffsetM4] = laplacian[1][iOffsetM4 - 1][jOffsetM4] -
              laplacian[3][iOffsetM4 - 1][jOffsetM4] + laplacian[3][iOffset][jOffsetM4];
          }

          // Variance for 10x10 sliding window.
          laplacian[VARIANCE][iOffsetM4][jOffsetM4] =
            (13 *
             ((numSample * laplacian[1][iOffsetM4][jOffsetM4] -
               laplacian[0][iOffsetM4][jOffsetM4] * laplacian[0][iOffsetM4][jOffsetM4] + offset) >>
              3)) >>
            14;
        }
      }
    }
    src += stride2;
  }
}

void AdaptiveLoopFilterEcm::calcClassBandBased(ClassBuf &classifier, const Area &blkDst, const Area &curBlk,
                                               const CPelBuf &srcLuma, const int classifierIdx, const int bitDepth,
                                               const CPelBuf &srcLumaResi, LaplacianBuf::DirBuf &buffer)
{
  constexpr int subBlkSize = 2;

  if (classifierIdx == 1)
  {
    const Pel      *src     = srcLuma.buf;
    const ptrdiff_t stride  = srcLuma.stride;
    const ptrdiff_t yOffset = blkDst.pos().y * stride;
    const Pel      *src0    = &src[yOffset];
    const Pel      *src1    = &src[yOffset + stride];
    const ptrdiff_t stride2 = 2 * stride;
    for (int i = 0; i < blkDst.height; i += subBlkSize)
    {
      for (int j = 0; j < blkDst.width; j += subBlkSize)
      {
        int        xOffset  = blkDst.pos().x + j;
        const Pel *pY0      = src0 + xOffset;
        const Pel *pY1      = src1 + xOffset;
        int        sum      = pY0[0] + pY0[1] + pY1[0] + pY1[1];
        int        classIdx = (sum * ALF_NUM_CLASSES_CLASSIFIER[classifierIdx]) >> (bitDepth + 2);
        for (int ii = curBlk.y + i; ii < curBlk.y + i + subBlkSize; ii++)
        {
          for (int jj = curBlk.x + j; jj < curBlk.x + j + subBlkSize; jj++)
          {
            classifier[ii][jj] = classIdx << 2;
          }
        }
      }
      src0 += stride2;
      src1 += stride2;
    }
  }
  else
  {
    const Pel      *src     = srcLumaResi.buf;
    const ptrdiff_t stride  = srcLumaResi.stride;
    const ptrdiff_t yOffset = blkDst.pos().y * stride;
    const Pel      *src0    = &src[yOffset];
    const ptrdiff_t stride2 = stride * 2;
    const Pel      *srcUp   = src0 - ALF_PADDING_SIZE_PRED * stride + blkDst.pos().x - ALF_PADDING_SIZE_PRED;
    const Pel      *srcDn   = srcUp + stride;
    // 2x2 sum
    for (int i = 0; i < blkDst.height + ALF_PADDING_SIZE_PRED * 2; i += 2)
    {
      for (int j = 0; j < blkDst.width + ALF_PADDING_SIZE_PRED * 2; j += 2)
      {
        buffer[i >> 1][j >> 1] = abs(srcUp[j]) + abs(srcUp[j + 1]) + abs(srcDn[j]) + abs(srcDn[j + 1]);
      }
      srcUp += stride2;
      srcDn += stride2;
    }
    // 2x4 sum
    for (int i = 0; i < (blkDst.height + ALF_PADDING_SIZE_PRED * 2) >> 1; i++)
    {
      for (int j = 0; j < ((blkDst.width + ALF_PADDING_SIZE_PRED * 2) >> 1) - 1; j++)
      {
        buffer[i][j] = buffer[i][j] + buffer[i][j + 1];
      }
    }
    // 4x4 sum
    for (int i = 0; i < ((blkDst.height + ALF_PADDING_SIZE_PRED * 2) >> 1) - 1; i++)
    {
      for (int j = 0; j < ((blkDst.width + ALF_PADDING_SIZE_PRED * 2) >> 1) - 1; j++)
      {
        buffer[i][j] = buffer[i][j] + buffer[i + 1][j];
      }
    }
    for (int i = 0; i < blkDst.height; i += subBlkSize)
    {
      for (int j = 0; j < blkDst.width; j += subBlkSize)
      {
        int i2          = i >> 1;
        int j2          = j >> 1;
        int sum         = buffer[i2][j2] + buffer[i2][j2 + 2] + buffer[i2 + 2][j2] + buffer[i2 + 2][j2 + 2];
        int shiftOffset = ALF_RESI_SHIFT_OFFSET;
        int classIdx    = sum >> (bitDepth - shiftOffset);
        if (classIdx > 24)
        {
          classIdx = 24;
        }
        for (int ii = curBlk.y + i; ii < curBlk.y + i + subBlkSize; ii++)
        {
          for (int jj = curBlk.x + j; jj < curBlk.x + j + subBlkSize; jj++)
          {
            classifier[ii][jj] = classIdx << 2;
          }
        }
      }
    }
  }
}

template<bool isFixedFilterClassifier>
void AdaptiveLoopFilterEcm::calcClassHelper(ClassBuf &classifier, const Area &blkDst, const Area &curBlk,
                                            const ClsWndwSize dirWindSize, unsigned bitDepth,
                                            const LaplacianBuf &laplacian)
{
  constexpr unsigned subBlkSize = 2;

  CHECK(dirWindSize != ClsWndwSize::WNDW_4x4 && dirWindSize != ClsWndwSize::WNDW_12x12 &&
          dirWindSize != ClsWndwSize::WNDW_CHROMA,
        "Invalid window size.");

  const unsigned shift = 9 + bitDepth;
  const unsigned mult  = dirWindSize == ClsWndwSize::WNDW_4x4 || dirWindSize == ClsWndwSize::WNDW_CHROMA ? 1407 : 156;

  const unsigned laplacianOffset = dirWindSize == ClsWndwSize::WNDW_4x4 ? 2 : 0;
  for (SizeType i = 0, yOffset = laplacianOffset; i < curBlk.height; i += subBlkSize, ++yOffset)
  {
    for (SizeType j = 0, xOffset = laplacianOffset; j < curBlk.width; j += subBlkSize, ++xOffset)
    {
      const uint32_t sumV = laplacian[VER][yOffset][xOffset];
      const uint32_t sumH = laplacian[HOR][yOffset][xOffset];
      uint32_t       hv0, hv1;
      unsigned       dirTempHV;
      if (sumV > sumH)
      {
        hv1       = sumV;
        hv0       = sumH;
        dirTempHV = 1;
      }
      else
      {
        hv1       = sumH;
        hv0       = sumV;
        dirTempHV = 3;
      }

      const uint32_t sumD0 = laplacian[DIAG0][yOffset][xOffset];
      const uint32_t sumD1 = laplacian[DIAG1][yOffset][xOffset];
      uint32_t       d0, d1;
      unsigned       dirTempD;
      if (sumD0 > sumD1)
      {
        d1       = sumD0;
        d0       = sumD1;
        dirTempD = 0;
      }
      else
      {
        d1       = sumD1;
        d0       = sumD0;
        dirTempD = 2;
      }

      unsigned mainDirection, secondaryDirection;
      uint32_t hvd0, hvd1;
      if ((uint64_t)d1 * hv0 > (uint64_t)d0 * hv1)
      {
        mainDirection      = dirTempD;
        secondaryDirection = dirTempHV;
        hvd1               = d1;
        hvd0               = d0;
      }
      else
      {
        mainDirection      = dirTempHV;
        secondaryDirection = dirTempD;
        hvd1               = hv1;
        hvd0               = hv0;
      }

      // activity
      const unsigned sum = sumV + sumH;
      unsigned       activity;
      if (!isFixedFilterClassifier)
      {
        static const unsigned th[] = { 0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4 };
        activity                   = th[std::min((mult * 3 * sum) >> shift, 15U)];
      }
      else
      {
        // clang-format off
        static const unsigned th[193] = {  0,  1,  2,  3,  4,  4,  5,  5,  6,  6,  6,  6,  7,  7,  7,  7,
                                           8,  8,  8,  8,  8,  8,  8,  8,  9,  9,  9,  9,  9,  9,  9,  9,
                                          10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
                                          11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11,
                                          12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
                                          12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
                                          13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
                                          13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
                                          14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
                                          14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
                                          14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
                                          14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
                                          15 };
        // clang-format on
        activity = th[std::min((mult * sum) >> (shift - 4), 192U)];
      }

      // direction
      unsigned direction;
      if (!isFixedFilterClassifier)
      {
        int directionStrength = 0;
        if (hvd1 * 2 > 9 * hvd0)
        {
          directionStrength = 2;
        }
        else if (hvd1 > 2 * hvd0)
        {
          directionStrength = 1;
        }

        if (directionStrength)
        {
          direction = ((mainDirection & 0x1) << 1) + directionStrength;
        }
        else
        {
          direction = 0;
        }
      }
      else
      {
        unsigned edgeStrengthHV;
        if (hv1 > 8 * hv0)
        {
          edgeStrengthHV = 6;
        }
        else if (hv1 * 2 > 9 * hv0)
        {
          edgeStrengthHV = 5;
        }
        else if (hv1 > 3 * hv0)
        {
          edgeStrengthHV = 4;
        }
        else if (hv1 > 2 * hv0)
        {
          edgeStrengthHV = 3;
        }
        else if (hv1 * 2 > 3 * hv0)
        {
          edgeStrengthHV = 2;
        }
        else if (hv1 * 4 > 5 * hv0)
        {
          edgeStrengthHV = 1;
        }
        else
        {
          edgeStrengthHV = 0;
        }

        unsigned edgeStrengthD;
        if (d1 > 8 * d0)
        {
          edgeStrengthD = 6;
        }
        else if (d1 * 2 > 9 * d0)
        {
          edgeStrengthD = 5;
        }
        else if (d1 > 3 * d0)
        {
          edgeStrengthD = 4;
        }
        else if (d1 > 2 * d0)
        {
          edgeStrengthD = 3;
        }
        else if (d1 * 2 > 3 * d0)
        {
          edgeStrengthD = 2;
        }
        else if (d1 * 4 > 5 * d0)
        {
          edgeStrengthD = 1;
        }
        else
        {
          edgeStrengthD = 0;
        }

        static_assert(NUM_DIR_FIX == 7,
                      "Missing mapping table implementation for the current number of fixed filter directionalities.");
        // clang-format off
        static const unsigned mappingDir[NUM_DIR_FIX][NUM_DIR_FIX] = {
          {  0,  0,  0,  0,  0,  0,  0 },
          {  1,  2,  0,  0,  0,  0,  0 },
          {  3,  4,  5,  0,  0,  0,  0 },
          {  6,  7,  8,  9,  0,  0,  0 },
          { 10, 11, 12, 13, 14,  0,  0 },
          { 15, 16, 17, 18, 19, 20,  0 },
          { 21, 22, 23, 24, 25, 26, 27 },
        };
        // clang-format on
        if ((uint64_t)hv1 * d0 > (uint64_t)hv0 * d1)
        {
          direction = mappingDir[edgeStrengthHV][edgeStrengthD];
        }
        else
        {
          direction = 28 + mappingDir[edgeStrengthD][edgeStrengthHV];
        }
      }

      int actDirInd = 0;
      if (!isFixedFilterClassifier)
      {
        actDirInd = 5 * activity + direction;
      }
      else
      {
        static const int divShift2[16] = { 2, 2, 4, 4, 6, 6, 6, 6, 8, 8, 8, 8, 10, 10, 10, 10 };
        static const int sqrtSum[50]   = { 0, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4,
                                           5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7 };
        const int        noDirAll      = NUM_DIR_FIX * (NUM_DIR_FIX + 1);
        const int        noDirActAll   = noDirAll * NUM_ACT_FIX;

        uint32_t sum2 = laplacian[VARIANCE][i >> 1][j >> 1] >> divShift2[activity];
        if (sum2 > 49)
        {
          sum2 = 49;
        }

        actDirInd = sqrtSum[sum2] * noDirActAll + noDirAll * activity + direction;
      }

      const int transposeTable[8] = { 0, 1, 0, 2, 2, 3, 1, 3 };
      int       transposeIdx      = transposeTable[mainDirection * 2 + (secondaryDirection >> 1)];

      int posYDst = blkDst.pos().y;
      int posXDst = blkDst.pos().x;

      for (int ii = posYDst + i; ii < posYDst + i + subBlkSize; ++ii)
      {
        for (int jj = posXDst + j; jj < posXDst + j + subBlkSize; ++jj)
        {
          classifier[ii][jj] = (actDirInd << 2) + transposeIdx;
        }
      }
    }
  }
}

template void AdaptiveLoopFilterEcm::calcClassHelper<false>(ClassBuf &classifier, const Area &blkDst,
                                                            const Area &curBlk, const ClsWndwSize dirWindSize,
                                                            unsigned bitDepth, const LaplacianBuf &laplacian);

template void AdaptiveLoopFilterEcm::calcClassHelper<true>(ClassBuf &classifier, const Area &blkDst, const Area &curBlk,
                                                           const ClsWndwSize dirWindSize, unsigned bitDepth,
                                                           const LaplacianBuf &laplacian);

template<AdaptiveLoopFilterEcm::AlfFilterType filtType> void AdaptiveLoopFilterEcm::filterBlk(
  const ClassBuf &classifier, const PelUnitBuf &recDst, const CPelBuf &recBeforeDbLuma, const CPelBuf &resiLuma,
  const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const CompID compId, const int8_t *scaleIdxSet,
  const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, const FixFiltBuf &fixedFilterResults,
  const FixFiltBuf &fixedFilterResultsPerCtu, const CompBuf &fixedFilterResiResults, const CompBuf &gaussPic,
  const CompBuf &gaussCtu, const bool isFixFiltPaddedPerCtu, const FixFiltSetCand fixedFilterSetCandIdx)
{
  CHECK(isLuma(compId) && (filtType != AlfFilterType::ALF_FILTER_9_EXT_DB_RESI) &&
          (filtType != AlfFilterType::ALF_FILTER_9_EXT_DB_RESI_DIRECT),
        "Invalid filter type for luma.");
  CHECK(isChroma(compId) && (filtType != AlfFilterType::ALF_FILTER_9), "Invalid filter type for chroma.");

  CHECK(static_cast<Size>(blkDst) != static_cast<Size>(blk), "Size mismatch between source and target area.");

  const CPelBuf &srcComp = recSrc.get(compId);
  PelBuf         dstComp = recDst.get(compId);   // TODO: ALF: No need for a copy but for non-const. BS 2023-09-27

  const ptrdiff_t srcStride = srcComp.stride;
  const ptrdiff_t dstStride = dstComp.stride;

  const int8_t *scaleIdx = scaleIdxSet;
  const short  *coef     = filterSet;
  const Pel    *clip     = fClipSet;

  const int shift  = (isLuma(compId) ? COEFF_SCALE_BITS_LUMA : COEFF_SCALE_BITS_CHROMA) + ALF_SCALE_SHIFT;
  const int offset = 1 << (shift - 1);

  int       transposeIdx = 0;
  const int clsSizeY     = 2;
  const int clsSizeX     = 2;

  CHECK(blk.y % clsSizeY, "Wrong y-coordinate in filtering");
  CHECK(blk.x % clsSizeX, "Wrong x-coordinate in filtering");
  CHECK(blk.height % clsSizeY, "Wrong height in filtering");
  CHECK(blk.width % clsSizeX, "Wrong width in filtering");

  const AlfClassifier *pClass = nullptr;

  const ptrdiff_t dstStride2 = dstStride * clsSizeY;
  const ptrdiff_t srcStride2 = srcStride * clsSizeY;

  // TODO: ALF: These arrays are much too large. BS 2023-09-26
  std::array<int, MAX_NUM_ALF_LUMA_COEFF> filterCoeff;
  std::array<int, MAX_NUM_ALF_LUMA_COEFF> filterClipp;

  constexpr unsigned filterSizeReco = 9;
  using RowPtrs                     = FilterRowPtrs<const Pel, filterSizeReco>;

  const Pel *pRecSrc0 = srcComp.bufAt(blk.pos());
  Pel       *pRecDst0 = dstComp.bufAt(blkDst.pos());

  for (int i = 0; i < blk.height; i += clsSizeY)
  {
    if (isLuma(compId))
    {
      pClass = classifier[blkDst.y + i] + blkDst.x;
    }

    for (int j = 0; j < blk.width; j += clsSizeX)
    {
      if (isLuma(compId))
      {
        const AlfClassifier &cl = pClass[j];
        transposeIdx            = cl & 0x3;
        int classIdx            = cl >> 2;
        scaleIdx                = scaleIdxSet + classIdx;
        coef                    = filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF;
        clip                    = fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF;
      }

      // Transpose the filters according to the current classification.
      if (filtType == ALF_FILTER_9_EXT_DB_RESI || filtType == ALF_FILTER_9_EXT_DB_RESI_DIRECT)
      {
        if (transposeIdx == 1)
        {
          filterCoeff = { // Filter coefficients for the reconstruction.
                          coef[6], coef[7], coef[8], coef[3], coef[9], coef[5], coef[0], coef[1], coef[2], coef[4],
                          // Filter coefficients for the second fixed filter output.
                          coef[22], coef[23], coef[24], coef[25], coef[17], coef[26], coef[21], coef[14], coef[18],
                          coef[27], coef[20], coef[16], coef[10], coef[11], coef[12], coef[13], coef[15], coef[19]
          };

          filterClipp = { // Clipping coefficients for the reconstruction.
                          clip[6], clip[7], clip[8], clip[3], clip[9], clip[5], clip[0], clip[1], clip[2], clip[4],
                          // Clipping coefficients for the second fixed filter output.
                          clip[22], clip[23], clip[24], clip[25], clip[17], clip[26], clip[21], clip[14], clip[18],
                          clip[27], clip[20], clip[16], clip[10], clip[11], clip[12], clip[13], clip[15], clip[19]
          };
        }
        else if (transposeIdx == 2)
        {
          filterCoeff = { // Filter coefficients for the reconstruction.
                          coef[0], coef[1], coef[2], coef[5], coef[4], coef[3], coef[6], coef[7], coef[8], coef[9],
                          // Filter coefficients for the second fixed filter output.
                          coef[10], coef[11], coef[12], coef[13], coef[16], coef[15], coef[14], coef[21], coef[20],
                          coef[19], coef[18], coef[17], coef[22], coef[23], coef[24], coef[25], coef[26], coef[27]
          };

          filterClipp = { // Clipping coefficients for the reconstruction.
                          clip[0], clip[1], clip[2], clip[5], clip[4], clip[3], clip[6], clip[7], clip[8], clip[9],
                          // Clipping coefficients for the second fixed filter output.
                          clip[10], clip[11], clip[12], clip[13], clip[16], clip[15], clip[14], clip[21], clip[20],
                          clip[19], clip[18], clip[17], clip[22], clip[23], clip[24], clip[25], clip[26], clip[27]
          };
        }
        else if (transposeIdx == 3)
        {
          filterCoeff = { // Filter coefficients for the reconstruction.
                          coef[6], coef[7], coef[8], coef[5], coef[9], coef[3], coef[0], coef[1], coef[2], coef[4],
                          // Filter coefficients for the second fixed filter output.
                          coef[22], coef[23], coef[24], coef[25], coef[21], coef[26], coef[17], coef[16], coef[20],
                          coef[27], coef[18], coef[14], coef[10], coef[11], coef[12], coef[13], coef[15], coef[19]
          };

          filterClipp = { // Clipping coefficients for the reconstruction.
                          clip[6], clip[7], clip[8], clip[5], clip[9], clip[3], clip[0], clip[1], clip[2], clip[4],
                          // Clipping coefficients for the second fixed filter output.
                          clip[22], clip[23], clip[24], clip[25], clip[21], clip[26], clip[17], clip[16], clip[20],
                          clip[27], clip[18], clip[14], clip[10], clip[11], clip[12], clip[13], clip[15], clip[19]
          };
        }
        else
        {
          filterCoeff = { // Filter coefficients for the reconstruction.
                          coef[0], coef[1], coef[2], coef[3], coef[4], coef[5], coef[6], coef[7], coef[8], coef[9],
                          // Filter coefficients for the second fixed filter output.
                          coef[10], coef[11], coef[12], coef[13], coef[14], coef[15], coef[16], coef[17], coef[18],
                          coef[19], coef[20], coef[21], coef[22], coef[23], coef[24], coef[25], coef[26], coef[27]
          };

          filterClipp = { // Clipping coefficients for the reconstruction.
                          clip[0], clip[1], clip[2], clip[3], clip[4], clip[5], clip[6], clip[7], clip[8], clip[9],
                          // Clipping coefficients for the second fixed filter output.
                          clip[10], clip[11], clip[12], clip[13], clip[14], clip[15], clip[16], clip[17], clip[18],
                          clip[19], clip[20], clip[21], clip[22], clip[23], clip[24], clip[25], clip[26], clip[27]
          };
        }
      }
      else   // (filtType == ALF_FILTER_9)
      {
        if (transposeIdx == 1)
        {
          filterCoeff = { coef[16], coef[9],  coef[17], coef[15], coef[4],  coef[10], coef[18], coef[14], coef[8],
                          coef[1],  coef[5],  coef[11], coef[19], coef[13], coef[7],  coef[3],  coef[0],  coef[2],
                          coef[6],  coef[12], coef[22], coef[23], coef[20], coef[21], coef[24], coef[25] };
          filterClipp = { clip[16], clip[9],  clip[17], clip[15], clip[4],  clip[10], clip[18], clip[14], clip[8],
                          clip[1],  clip[5],  clip[11], clip[19], clip[13], clip[7],  clip[3],  clip[0],  clip[2],
                          clip[6],  clip[12], clip[22], clip[23], clip[20], clip[21], clip[24], clip[25] };
        }
        else if (transposeIdx == 2)
        {
          filterCoeff = { coef[0],  coef[3],  coef[2],  coef[1],  coef[8],  coef[7],  coef[6],  coef[5],  coef[4],
                          coef[15], coef[14], coef[13], coef[12], coef[11], coef[10], coef[9],  coef[16], coef[17],
                          coef[18], coef[19], coef[20], coef[21], coef[22], coef[23], coef[24], coef[25] };
          filterClipp = { clip[0],  clip[3],  clip[2],  clip[1],  clip[8],  clip[7],  clip[6],  clip[5],  clip[4],
                          clip[15], clip[14], clip[13], clip[12], clip[11], clip[10], clip[9],  clip[16], clip[17],
                          clip[18], clip[19], clip[20], clip[21], clip[22], clip[23], clip[24], clip[25] };
        }
        else if (transposeIdx == 3)
        {
          filterCoeff = { coef[16], coef[15], coef[17], coef[9],  coef[8],  coef[14], coef[18], coef[10], coef[4],
                          coef[3],  coef[7],  coef[13], coef[19], coef[11], coef[5],  coef[1],  coef[0],  coef[2],
                          coef[6],  coef[12], coef[22], coef[23], coef[20], coef[21], coef[24], coef[25] };
          filterClipp = { clip[16], clip[15], clip[17], clip[9],  clip[8],  clip[14], clip[18], clip[10], clip[4],
                          clip[3],  clip[7],  clip[13], clip[19], clip[11], clip[5],  clip[1],  clip[0],  clip[2],
                          clip[6],  clip[12], clip[22], clip[23], clip[20], clip[21], clip[24], clip[25] };
        }
        else
        {
          filterCoeff = { coef[0],  coef[1],  coef[2],  coef[3],  coef[4],  coef[5],  coef[6],  coef[7],  coef[8],
                          coef[9],  coef[10], coef[11], coef[12], coef[13], coef[14], coef[15], coef[16], coef[17],
                          coef[18], coef[19], coef[20], coef[21], coef[22], coef[23], coef[24], coef[25] };
          filterClipp = { clip[0],  clip[1],  clip[2],  clip[3],  clip[4],  clip[5],  clip[6],  clip[7],  clip[8],
                          clip[9],  clip[10], clip[11], clip[12], clip[13], clip[14], clip[15], clip[16], clip[17],
                          clip[18], clip[19], clip[20], clip[21], clip[22], clip[23], clip[24], clip[25] };
        }
      }

      for (int ii = 0; ii < clsSizeY; ii++)
      {
        RowPtrs pRecSrc1Rows(pRecSrc0 + ii * srcStride + j, srcStride);
        Pel    *pRecDst1 = pRecDst0 + ii * dstStride + j;

        for (int jj = 0; jj < clsSizeX; jj++)
        {
          int32_t   sum  = 0;
          const Pel curr = pRecSrc1Rows[0][0];
          if (filtType == ALF_FILTER_9_EXT_DB_RESI || filtType == ALF_FILTER_9_EXT_DB_RESI_DIRECT)
          {
            // 9x9 cross-shaped filter for the ALF input reconstruction.
            sum += filterCoeff[0] * (clipALF(filterClipp[0], curr, pRecSrc1Rows[4][+0], pRecSrc1Rows[-4][-0]));
            sum += filterCoeff[1] * (clipALF(filterClipp[1], curr, pRecSrc1Rows[3][+0], pRecSrc1Rows[-3][-0]));
            sum += filterCoeff[2] * (clipALF(filterClipp[2], curr, pRecSrc1Rows[2][+0], pRecSrc1Rows[-2][-0]));

            sum += filterCoeff[3] * (clipALF(filterClipp[3], curr, pRecSrc1Rows[1][+1], pRecSrc1Rows[-1][-1]));
            sum += filterCoeff[4] * (clipALF(filterClipp[4], curr, pRecSrc1Rows[1][+0], pRecSrc1Rows[-1][-0]));
            sum += filterCoeff[5] * (clipALF(filterClipp[5], curr, pRecSrc1Rows[1][-1], pRecSrc1Rows[-1][+1]));

            sum += filterCoeff[6] * (clipALF(filterClipp[6], curr, pRecSrc1Rows[0][+4], pRecSrc1Rows[0][-4]));
            sum += filterCoeff[7] * (clipALF(filterClipp[7], curr, pRecSrc1Rows[0][+3], pRecSrc1Rows[0][-3]));
            sum += filterCoeff[8] * (clipALF(filterClipp[8], curr, pRecSrc1Rows[0][+2], pRecSrc1Rows[0][-2]));
            sum += filterCoeff[9] * (clipALF(filterClipp[9], curr, pRecSrc1Rows[0][+1], pRecSrc1Rows[0][-1]));
          }
          else   // (filtType == ALF_FILTER_9)
          {
            // 9x9 diamond-shaped filter for the ALF input reconstruction.
            sum += filterCoeff[0] * (clipALF(filterClipp[0], curr, pRecSrc1Rows[4][+0], pRecSrc1Rows[-4][+0]));

            sum += filterCoeff[1] * (clipALF(filterClipp[1], curr, pRecSrc1Rows[3][+1], pRecSrc1Rows[-3][-1]));
            sum += filterCoeff[2] * (clipALF(filterClipp[2], curr, pRecSrc1Rows[3][+0], pRecSrc1Rows[-3][+0]));
            sum += filterCoeff[3] * (clipALF(filterClipp[3], curr, pRecSrc1Rows[3][-1], pRecSrc1Rows[-3][+1]));

            sum += filterCoeff[4] * (clipALF(filterClipp[4], curr, pRecSrc1Rows[2][+2], pRecSrc1Rows[-2][-2]));
            sum += filterCoeff[5] * (clipALF(filterClipp[5], curr, pRecSrc1Rows[2][+1], pRecSrc1Rows[-2][-1]));
            sum += filterCoeff[6] * (clipALF(filterClipp[6], curr, pRecSrc1Rows[2][+0], pRecSrc1Rows[-2][+0]));
            sum += filterCoeff[7] * (clipALF(filterClipp[7], curr, pRecSrc1Rows[2][-1], pRecSrc1Rows[-2][+1]));
            sum += filterCoeff[8] * (clipALF(filterClipp[8], curr, pRecSrc1Rows[2][-2], pRecSrc1Rows[-2][+2]));

            sum += filterCoeff[9] * (clipALF(filterClipp[9], curr, pRecSrc1Rows[1][+3], pRecSrc1Rows[-1][-3]));
            sum += filterCoeff[10] * (clipALF(filterClipp[10], curr, pRecSrc1Rows[1][+2], pRecSrc1Rows[-1][-2]));
            sum += filterCoeff[11] * (clipALF(filterClipp[11], curr, pRecSrc1Rows[1][+1], pRecSrc1Rows[-1][-1]));
            sum += filterCoeff[12] * (clipALF(filterClipp[12], curr, pRecSrc1Rows[1][+0], pRecSrc1Rows[-1][+0]));
            sum += filterCoeff[13] * (clipALF(filterClipp[13], curr, pRecSrc1Rows[1][-1], pRecSrc1Rows[-1][+1]));
            sum += filterCoeff[14] * (clipALF(filterClipp[14], curr, pRecSrc1Rows[1][-2], pRecSrc1Rows[-1][+2]));
            sum += filterCoeff[15] * (clipALF(filterClipp[15], curr, pRecSrc1Rows[1][-3], pRecSrc1Rows[-1][+3]));

            sum += filterCoeff[16] * (clipALF(filterClipp[16], curr, pRecSrc1Rows[0][+4], pRecSrc1Rows[0][-4]));
            sum += filterCoeff[17] * (clipALF(filterClipp[17], curr, pRecSrc1Rows[0][+3], pRecSrc1Rows[0][-3]));
            sum += filterCoeff[18] * (clipALF(filterClipp[18], curr, pRecSrc1Rows[0][+2], pRecSrc1Rows[0][-2]));
            sum += filterCoeff[19] * (clipALF(filterClipp[19], curr, pRecSrc1Rows[0][+1], pRecSrc1Rows[0][-1]));

            sum += filterCoeff[20] *
              (clipALF(filterClipp[20], curr,
                       fixedFilterResults[FixFiltIdx::FIRST][blkDst.y + i + ii - 2][blkDst.x + j + jj],
                       fixedFilterResults[FixFiltIdx::FIRST][blkDst.y + i + ii + 2][blkDst.x + j + jj]));
            sum += filterCoeff[21] *
              (clipALF(filterClipp[21], curr,
                       fixedFilterResults[FixFiltIdx::FIRST][blkDst.y + i + ii - 1][blkDst.x + j + jj],
                       fixedFilterResults[FixFiltIdx::FIRST][blkDst.y + i + ii + 1][blkDst.x + j + jj]));
            sum += filterCoeff[22] *
              (clipALF(filterClipp[22], curr,
                       fixedFilterResults[FixFiltIdx::FIRST][blkDst.y + i + ii][blkDst.x + j + jj - 2],
                       fixedFilterResults[FixFiltIdx::FIRST][blkDst.y + i + ii][blkDst.x + j + jj + 2]));
            sum += filterCoeff[23] *
              (clipALF(filterClipp[23], curr,
                       fixedFilterResults[FixFiltIdx::FIRST][blkDst.y + i + ii][blkDst.x + j + jj - 1],
                       fixedFilterResults[FixFiltIdx::FIRST][blkDst.y + i + ii][blkDst.x + j + jj + 1]));
            sum += filterCoeff[24] *
              (clipALF(filterClipp[24], curr,
                       fixedFilterResults[FixFiltIdx::FIRST][blkDst.y + i + ii][blkDst.x + j + jj]));
          }

          if (filtType == ALF_FILTER_9_EXT_DB_RESI || filtType == ALF_FILTER_9_EXT_DB_RESI_DIRECT)
          {
            // Setup the input pointers for the additional filter components.
            const Pel *pImgFixedBasedCenter;
            const Pel *pImgGaussCenter;
            ptrdiff_t  pImgFixedBasedStride;
            ptrdiff_t  pImgGaussStride;
            if (isFixFiltPaddedPerCtu)
            {
              const auto &secFixedFilterResult = fixedFilterResultsPerCtu[FixFiltIdx::SECOND];
              pImgFixedBasedCenter             = &secFixedFilterResult[i + ii][j + jj];
              pImgFixedBasedStride             = secFixedFilterResult.stride;

              pImgGaussCenter = &gaussCtu[i + ii][j + jj];
              pImgGaussStride = gaussCtu.stride;
            }
            else
            {
              const auto &secFixedFilterResult = fixedFilterResults[FixFiltIdx::SECOND];
              pImgFixedBasedCenter             = &secFixedFilterResult[blkDst.y + i + ii][blkDst.x + j + jj];
              pImgFixedBasedStride             = secFixedFilterResult.stride;

              pImgGaussCenter = &gaussPic[blkDst.y + i + ii][blkDst.x + j + jj];
              pImgGaussStride = gaussPic.stride;
            }
            const Pel *const pRecDbCenter = recBeforeDbLuma.bufAt(blk.x + j + jj, blk.y + i + ii);
            const Pel *const pResiCenter  = resiLuma.bufAt(blk.x + j + jj, blk.y + i + ii);

            // 13x13 cross-shaped filter
            // for the output of the second fixed filter (fixed-filter-output-based) of the selected set.
            {
              FilterRowPtrs<const Pel, 13> pImgFixedBased(pImgFixedBasedCenter, pImgFixedBasedStride);

              sum += filterCoeff[10] * (clipALF(filterClipp[10], curr, pImgFixedBased[6][-0], pImgFixedBased[-6][+0]));
              sum += filterCoeff[11] * (clipALF(filterClipp[11], curr, pImgFixedBased[5][-0], pImgFixedBased[-5][+0]));
              sum += filterCoeff[12] * (clipALF(filterClipp[12], curr, pImgFixedBased[4][-0], pImgFixedBased[-4][+0]));
              sum += filterCoeff[13] * (clipALF(filterClipp[13], curr, pImgFixedBased[3][-0], pImgFixedBased[-3][+0]));

              sum += filterCoeff[14] * (clipALF(filterClipp[14], curr, pImgFixedBased[2][+1], pImgFixedBased[-2][-1]));
              sum += filterCoeff[15] * (clipALF(filterClipp[15], curr, pImgFixedBased[2][+0], pImgFixedBased[-2][-0]));
              sum += filterCoeff[16] * (clipALF(filterClipp[16], curr, pImgFixedBased[2][-1], pImgFixedBased[-2][+1]));

              sum += filterCoeff[17] * (clipALF(filterClipp[17], curr, pImgFixedBased[1][+2], pImgFixedBased[-1][-2]));
              sum += filterCoeff[18] * (clipALF(filterClipp[18], curr, pImgFixedBased[1][+1], pImgFixedBased[-1][-1]));
              sum += filterCoeff[19] * (clipALF(filterClipp[19], curr, pImgFixedBased[1][+0], pImgFixedBased[-1][-0]));
              sum += filterCoeff[20] * (clipALF(filterClipp[20], curr, pImgFixedBased[1][-1], pImgFixedBased[-1][+1]));
              sum += filterCoeff[21] * (clipALF(filterClipp[21], curr, pImgFixedBased[1][-2], pImgFixedBased[-1][+2]));

              sum += filterCoeff[22] * (clipALF(filterClipp[22], curr, pImgFixedBased[0][-6], pImgFixedBased[0][+6]));
              sum += filterCoeff[23] * (clipALF(filterClipp[23], curr, pImgFixedBased[0][-5], pImgFixedBased[0][+5]));
              sum += filterCoeff[24] * (clipALF(filterClipp[24], curr, pImgFixedBased[0][-4], pImgFixedBased[0][+4]));
              sum += filterCoeff[25] * (clipALF(filterClipp[25], curr, pImgFixedBased[0][-3], pImgFixedBased[0][+3]));
              sum += filterCoeff[26] * (clipALF(filterClipp[26], curr, pImgFixedBased[0][-2], pImgFixedBased[0][+2]));
              sum += filterCoeff[27] * (clipALF(filterClipp[27], curr, pImgFixedBased[0][-1], pImgFixedBased[0][+1]));
            }

            // 5x5 cross-shaped filter for the output of the gaussian fixed filter.
            constexpr unsigned offsetGauss = 28;
            if (filtType == ALF_FILTER_9_EXT_DB_RESI_DIRECT)
            {
              FilterRowPtrs<const Pel, 5> pImgGauss(pImgGaussCenter, pImgGaussStride);

              sum += coef[offsetGauss + 0] * clipALF(clip[offsetGauss + 0], curr, pImgGauss[-2][+0], pImgGauss[2][+0]);
              sum += coef[offsetGauss + 1] * clipALF(clip[offsetGauss + 1], curr, pImgGauss[-1][+0], pImgGauss[1][+0]);

              sum += coef[offsetGauss + 2] * clipALF(clip[offsetGauss + 2], curr, pImgGauss[0][-2], pImgGauss[0][+2]);
              sum += coef[offsetGauss + 3] * clipALF(clip[offsetGauss + 3], curr, pImgGauss[0][-1], pImgGauss[0][+1]);
            }

            // 3x3 diamond-shaped filter for the pre-DBF reconstruction.
            constexpr unsigned offsetDb = offsetGauss + (filtType == ALF_FILTER_9_EXT_DB_RESI_DIRECT ? 4 : 0);
            {
              FilterRowPtrs<const Pel, 3> pRecDbTmp(pRecDbCenter, recBeforeDbLuma.stride);

              sum += coef[offsetDb + 0] * clipALF(clip[offsetDb + 0], curr, pRecDbTmp[-1][+0], pRecDbTmp[1][-0]);
              sum += coef[offsetDb + 1] * clipALF(clip[offsetDb + 1], curr, pRecDbTmp[0][-1], pRecDbTmp[0][+1]);
            }

            // Centers of the filters for the outputs of the fixed filters of the selected set.
            constexpr unsigned offsetFixCenter = offsetDb + 2;
            sum += coef[offsetFixCenter + 0] *
              clipALF(clip[offsetFixCenter + 0], curr,
                      fixedFilterResults[FixFiltIdx::FIRST][blkDst.y + i + ii][blkDst.x + j + jj]);
            sum += coef[offsetFixCenter + 1] *
              clipALF(clip[offsetFixCenter + 1], curr,
                      fixedFilterResults[FixFiltIdx::SECOND][blkDst.y + i + ii][blkDst.x + j + jj]);

            // Center of the filter
            // for the output of the first fixed filter of the unselected set applied to the residual.
            constexpr unsigned offsetFixResi = offsetFixCenter + 2;
            if (filtType == ALF_FILTER_9_EXT_DB_RESI)
            {
              sum += coef[offsetFixResi + 0] *
                clipALF(clip[offsetFixResi + 0], 0, fixedFilterResiResults[blkDst.y + i + ii][blkDst.x + j + jj]);
            }

            // Center of the filter for the pre-DBF reconstruction.
            constexpr unsigned offsetDbCenter = offsetFixResi + (filtType == ALF_FILTER_9_EXT_DB_RESI ? 1 : 0);
            sum += coef[offsetDbCenter + 0] * clipALF(clip[offsetDbCenter + 0], curr, pRecDbCenter[+0]);

            // Center of the filter for the residual.
            constexpr unsigned offsetResi = offsetDbCenter + 1;
            sum += coef[offsetResi + 0] * clipALF(clip[offsetResi + 0], 0, pResiCenter[+0]);

            // Center of the filter for the output of the gaussian fixed filter.
            constexpr unsigned offsetGaussCenter = offsetResi + 1;
            sum += coef[offsetGaussCenter + 0] * clipALF(clip[offsetGaussCenter + 0], curr, pImgGaussCenter[0]);
          }

          sum *= ALF_SCALE_FACTOR[(int)scaleIdx[0]];
          sum = (sum + offset) >> shift;

          sum += curr;
          pRecDst1[jj] = ClipPel(sum, clpRng);

          ++pRecSrc1Rows;
        }
      }
    }

    pRecSrc0 += srcStride2;
    pRecDst0 += dstStride2;
  }
}

void AdaptiveLoopFilterEcm::filterBlkCcAlf(const PelBuf &dstBuf, const CPelUnitBuf &recSrc,
                                           const CPelUnitBuf &recSaoSrc, const Area &blkDst, const Area &blkSrc,
                                           const CompID compId, const int16_t *filterCoeff, const ClpRngs &clpRngs,
                                           CodingStructure &cs)
{
  CHECK(!isChroma(compId), "Must be chroma");

  const int clsSizeY = 4;
  const int clsSizeX = 4;

  const int scaleX = getComponentScaleX(compId, cs.slice->m_sps->m_chromaFormatIdc);
  const int scaleY = getComponentScaleY(compId, cs.slice->m_sps->m_chromaFormatIdc);

  CHECK(blkDst.y % clsSizeY, "Wrong y-coordinate in filtering");
  CHECK(blkDst.x % clsSizeX, "Wrong x-coordinate in filtering");
  CHECK(blkDst.height % clsSizeY, "Wrong height in filtering");
  CHECK(blkDst.width % clsSizeX, "Wrong width in filtering");

  CPelBuf srcBuf = recSrc.get(COMP_Y);

  const ptrdiff_t lumaStride   = srcBuf.stride;
  const ptrdiff_t chromaStride = dstBuf.stride;

  const Pel *lumaPtr   = srcBuf.buf + blkSrc.y * lumaStride + blkSrc.x;
  Pel       *chromaPtr = dstBuf.buf + blkDst.y * chromaStride + blkDst.x;

  CPelBuf         srcSaoBuf = recSaoSrc.get(compId);
  const ptrdiff_t saoStride = srcSaoBuf.stride;
  const Pel      *recSaoPtr = srcSaoBuf.buf + blkDst.y * saoStride + blkDst.x;

  for (int i = 0; i < blkDst.height; i += clsSizeY)
  {
    for (int j = 0; j < blkDst.width; j += clsSizeX)
    {
      for (int ii = 0; ii < clsSizeY; ii++)
      {
        int        row         = ii;
        int        col         = j;
        Pel       *srcSelf     = chromaPtr + col + row * chromaStride;
        const Pel *srcSaoCross = recSaoPtr + col + row * saoStride;

        // clang-format off
        const ptrdiff_t offsetM4 = -4 * lumaStride;
        const ptrdiff_t offsetM3 = -3 * lumaStride;
        const ptrdiff_t offsetM2 = -2 * lumaStride;
        const ptrdiff_t offsetM1 = -1 * lumaStride;
        const ptrdiff_t offset0  =  0;
        const ptrdiff_t offsetP1 =  1 * lumaStride;
        const ptrdiff_t offsetP2 =  2 * lumaStride;
        const ptrdiff_t offsetP3 =  3 * lumaStride;
        const ptrdiff_t offsetP4 =  4 * lumaStride;
        // clang-format on

        row <<= scaleY;
        col <<= scaleX;
        const Pel *srcCross = lumaPtr + col + row * lumaStride;

        for (int jj = 0; jj < clsSizeX; jj++)
        {
          const int jj2 = (jj << scaleX);

          int       sum             = 0;
          const Pel currSrcCross    = srcCross[offset0 + jj2];
          const Pel currSrcSAOCross = srcSaoCross[offset0 + jj];

          // clang-format off
          sum += filterCoeff[0]  * (srcCross[offsetM4 + jj2    ] - currSrcCross);
          sum += filterCoeff[1]  * (srcCross[offsetM3 + jj2    ] - currSrcCross);
          sum += filterCoeff[2]  * (srcCross[offsetM2 + jj2    ] - currSrcCross);
          sum += filterCoeff[3]  * (srcCross[offsetM1 + jj2    ] - currSrcCross);

          sum += filterCoeff[4]  * (srcCross[offset0  + jj2 - 4] - currSrcCross);
          sum += filterCoeff[5]  * (srcCross[offset0  + jj2 - 3] - currSrcCross);
          sum += filterCoeff[6]  * (srcCross[offset0  + jj2 - 2] - currSrcCross);
          sum += filterCoeff[7]  * (srcCross[offset0  + jj2 - 1] - currSrcCross);
          sum += filterCoeff[8]  * (srcCross[offset0  + jj2 + 1] - currSrcCross);
          sum += filterCoeff[9]  * (srcCross[offset0  + jj2 + 2] - currSrcCross);
          sum += filterCoeff[10] * (srcCross[offset0  + jj2 + 3] - currSrcCross);
          sum += filterCoeff[11] * (srcCross[offset0  + jj2 + 4] - currSrcCross);

          sum += filterCoeff[12] * (srcCross[offsetP1 + jj2 - 4] - currSrcCross);
          sum += filterCoeff[13] * (srcCross[offsetP1 + jj2 - 3] - currSrcCross);
          sum += filterCoeff[14] * (srcCross[offsetP1 + jj2 - 2] - currSrcCross);
          sum += filterCoeff[15] * (srcCross[offsetP1 + jj2 - 1] - currSrcCross);
          sum += filterCoeff[16] * (srcCross[offsetP1 + jj2 - 0] - currSrcCross);
          sum += filterCoeff[17] * (srcCross[offsetP1 + jj2 + 1] - currSrcCross);
          sum += filterCoeff[18] * (srcCross[offsetP1 + jj2 + 2] - currSrcCross);
          sum += filterCoeff[19] * (srcCross[offsetP1 + jj2 + 3] - currSrcCross);
          sum += filterCoeff[20] * (srcCross[offsetP1 + jj2 + 4] - currSrcCross);

          sum += filterCoeff[21] * (srcCross[offsetP2 + jj2    ] - currSrcCross);
          sum += filterCoeff[22] * (srcCross[offsetP3 + jj2    ] - currSrcCross);
          sum += filterCoeff[23] * (srcCross[offsetP4 + jj2    ] - currSrcCross);

          sum += filterCoeff[24] * (srcSaoCross[-1 * saoStride + jj + 0] - currSrcSAOCross);
          sum += filterCoeff[25] * (srcSaoCross[+0 * saoStride + jj - 1] - currSrcSAOCross);
          sum += filterCoeff[26] * (srcSaoCross[+0 * saoStride + jj + 1] - currSrcSAOCross);
          sum += filterCoeff[27] * (srcSaoCross[+1 * saoStride + jj + 0] - currSrcSAOCross);
          // clang-format on

          sum = (sum + (1 << COEFF_SCALE_BITS_CCALF >> 1)) >> COEFF_SCALE_BITS_CCALF;

          const int offset = 1 << clpRngs.comp[compId].bd >> 1;
          sum              = ClipPel(sum + offset, clpRngs.comp[compId]) - offset;
          sum += srcSelf[jj];
          srcSelf[jj] = ClipPel(sum, clpRngs.comp[compId]);
        }
      }
    }

    chromaPtr += chromaStride * clsSizeY;

    lumaPtr += lumaStride * clsSizeY << scaleY;
    recSaoPtr += saoStride * clsSizeY;
  }
}

void AdaptiveLoopFilterEcm::deriveSecondFixedFilterResults(const CPelBuf &srcLuma, const CPelBuf &srcLumaBeforeDb,
                                                           const FixFiltCandBuf &firstFixedFilterResults,
                                                           const Area &blkSrc, const Area &blkDst,
                                                           const CodingStructure &cs, const int fixedFilterSetCandIdx)
{
  if (cs.slice->getCuQpDeltaSubdiv())
  {
    UnitArea curArea(cs.area.chromaFormat, blkDst);
    for (auto &currCU: cs.traverseCUs(curArea, ChannelType::LUMA))
    {
      const Area currCuBlkSrc(Position(currCU.lx() - blkDst.x + blkSrc.x, currCU.ly() - blkDst.y + blkSrc.y),
                              currCU.lumaSize());
      deriveSecondFixedFilterResultsBlk(srcLuma, srcLumaBeforeDb, firstFixedFilterResults, currCuBlkSrc, currCU.Y(),
                                        currCU.qp, fixedFilterSetCandIdx);
    }
  }
  else
  {
    deriveSecondFixedFilterResultsBlk(srcLuma, srcLumaBeforeDb, firstFixedFilterResults, blkSrc, blkDst,
                                      cs.slice->m_iSliceQp, fixedFilterSetCandIdx);
  }
}

void AdaptiveLoopFilterEcm::deriveFixedFilterResultsChroma(const CPelBuf &src, const CPelBuf &srcBeforeDb,
                                                           const Area &blkDst, const Area &blk, CodingStructure &cs,
                                                           const int fixedFilterSetCandIdx, const CompID compId)
{
  if (cs.slice->getCuQpDeltaSubdiv())
  {
    UnitArea    curArea(cs.area.chromaFormat, blkDst);
    ChannelType ch = CS::isDualITree(cs) ? ChannelType::LUMA : ChannelType::CHROMA;
    for (auto &currCU: cs.traverseCUs(curArea, ch))
    {
      deriveFixedFilterResultsBlkChroma(src, srcBeforeDb, currCU.blocks[compId], currCU.blocks[compId], currCU.qp,
                                        fixedFilterSetCandIdx, compId);
    }
  }
  else
  {
    int height = blk.pos().y + blk.height;
    int width  = blk.pos().x + blk.width;
    int qp     = cs.slice->m_iSliceQp + cs.slice->getSliceChromaQpDelta(compId);

    for (int i = blk.pos().y; i < height; i += CLASSIFICATION_BLK_SIZE)
    {
      int nHeight = std::min(i + CLASSIFICATION_BLK_SIZE, height) - i;

      for (int j = blk.pos().x; j < width; j += CLASSIFICATION_BLK_SIZE)
      {
        int nWidth = std::min(j + CLASSIFICATION_BLK_SIZE, width) - j;
        deriveFixedFilterResultsBlkChroma(
          src, srcBeforeDb, Area(j - blk.pos().x + blkDst.pos().x, i - blk.pos().y + blkDst.pos().y, nWidth, nHeight),
          Area(j, i, nWidth, nHeight), qp, fixedFilterSetCandIdx, compId);
      }
    }
  }
}

void AdaptiveLoopFilterEcm::deriveSecondFixedFilterResultsBlk(const CPelBuf &srcLuma, const CPelBuf &srcLumaBeforeDb,
                                                              const FixFiltCandBuf &firstFixedFilterResults,
                                                              const Area &blkSrc, const Area &blkDst, const int qp,
                                                              const int fixedFilterSetCandIdx)
{
  const FixedFilterSetCands fixedFiltSetCands(qp);
  for (int currFixFiltSetCandIdx = 0; currFixFiltSetCandIdx < NUM_FIXED_FILTER_SET_CANDS; ++currFixFiltSetCandIdx)
  {
    if (currFixFiltSetCandIdx == fixedFilterSetCandIdx || fixedFilterSetCandIdx == -1)
    {
      const FixFiltSetCand currFixFiltSetCand = static_cast<FixFiltSetCand>(currFixFiltSetCandIdx);
      alfSecondFixedFilterBlk(srcLuma, srcLumaBeforeDb, firstFixedFilterResults[currFixFiltSetCand][FixFiltIdx::FIRST],
                              blkSrc, m_fixFilterResult[COMP_Y][currFixFiltSetCand][FixFiltIdx::SECOND], blkDst,
                              fixedFiltSetCands[currFixFiltSetCand]);
    }
  }
}

void AdaptiveLoopFilterEcm::deriveFixedFilterResultsBlkChroma(const CPelBuf &src, const CPelBuf &srcBeforeDb,
                                                              const Area &blkDst, const Area &blk, const int qp,
                                                              const int fixedFilterSetCandIdx, const CompID compId)
{
  m_deriveVariance(src, blk, m_laplacian);
  m_deriveClassificationLaplacian(src, blk, m_laplacian, ALF_CLASSIFIER_FL_CHROMA);
  m_calcClassGradBasedFixedFilt(m_classifier[ClassifierIndex::FIXED_FILTER_CHROMA], blkDst, blk,
                                ClsWndwSize::WNDW_CHROMA, m_inputBitDepth[ChannelType::CHROMA], m_laplacian);

  const FixedFilterSetCands fixedFiltSetCands(qp);
  for (int currFixFiltSetCandIdx = 0; currFixFiltSetCandIdx < NUM_FIXED_FILTER_SET_CANDS; ++currFixFiltSetCandIdx)
  {
    if (currFixFiltSetCandIdx == fixedFilterSetCandIdx || fixedFilterSetCandIdx == -1)
    {
      const FixFiltSetCand currFixFiltSetCand = FixFiltSetCand(currFixFiltSetCandIdx);
      alfChromaFixedFilterBlk(src, srcBeforeDb, blk, m_fixFilterResult[compId][currFixFiltSetCand][FixFiltIdx::FIRST],
                              blkDst, fixedFiltSetCands[currFixFiltSetCand], compId);
    }
  }
}

void AdaptiveLoopFilterEcm::deriveFixedFilterResultsCtuBoundary(const CPelBuf &srcLuma, const CPelBuf &srcLumaBeforeDb,
                                                                const Area &blkDst, const int qp, int ctuIdx,
                                                                const FixFiltSetCand fixedFilterSetCandIdx,
                                                                const FixFiltIdx     fixedFilterIdx)
{
  // Required boundary sizes for calculating the following filter(s).
  const unsigned requiredSize1st = 2 * ALF_PADDING_SIZE_FIXED_RESULTS;
  const unsigned requiredSize2nd = ALF_PADDING_SIZE_FIXED_RESULTS;

  // Desired boundary sizes enabling the use of SIMD (>= required sizes).
  const unsigned desiredWidth1st = 16;
  const unsigned desiredWidth2nd = 8;
  static_assert(desiredWidth1st >= requiredSize1st, "Desired size for 1st fixed filter is too small.");
  static_assert(desiredWidth2nd >= requiredSize2nd, "Desired size for 2nd fixed filter is too small.");

  // Get the neighboring areas for fixed filter calculation and padding.
  const NeighborCalcAndPadAreas calcAndPadAreas = calcCtuBoundaryCalcAndPadAreas(
    blkDst, fixedFilterIdx == FixFiltIdx::FIRST ? requiredSize1st : requiredSize2nd,
    fixedFilterIdx == FixFiltIdx::FIRST ? desiredWidth1st : desiredWidth2nd,
    fixedFilterIdx == FixFiltIdx::FIRST ? requiredSize1st : requiredSize2nd, ALF_PADDING_SIZE_FIXED_RESULTS, true);

  // Check whether the fixed filter is enabled for a neighboring CTU.
  const auto isFixedFilterEnabled = [&fixedFilterIdx](const int ctbMode)
  {
    return LumaCtbModeHandler::isApsFilter(ctbMode) ||
      (LumaCtbModeHandler::isFixedFilter(ctbMode) &&
       LumaCtbModeHandler::getFixedFilterIdx(ctbMode) >= (fixedFilterIdx == FixFiltIdx::FIRST ? 0 : 1));
  };

  // Check whether the fixed filter needs to be calculated for a neighboring area.
  const auto needsCalc = [&isFixedFilterEnabled, this](const int ctuIdx)
  { return !isFixedFilterEnabled((*m_modes)[CompID::COMP_Y][ctuIdx]) || !m_ctuAlreadyProcessedFlag[COMP_Y][ctuIdx]; };

  // Derive the fixed filter results for the required areas of the neighboring CTUs where they are not available.
  for (unsigned boundaryIdx = 0; boundaryIdx < static_cast<unsigned>(BndryLoc::NUM); ++boundaryIdx)
  {
    const CalculationArea &calcArea = calcAndPadAreas.m_calc[boundaryIdx];
    if (calcArea.m_valid && needsCalc(ctuIdx + calcArea.m_ctuIdxOffset))
    {
      if (boundaryIdx == 1)
      {
        m_ctuPadFlag[ctuIdx + calcArea.m_ctuIdxOffset] |= 0x01;
      }
      else if (boundaryIdx == 3)
      {
        m_ctuPadFlag[ctuIdx + calcArea.m_ctuIdxOffset] |= 0x02;
      }
      deriveFixedFilterResultsPerBlk(srcLuma, srcLumaBeforeDb, calcArea, qp, fixedFilterSetCandIdx, fixedFilterIdx);
    }
  }

  // Add padding at the picture boundary.
  extendBlockBoundaries(m_fixFilterResult[COMP_Y][fixedFilterSetCandIdx][fixedFilterIdx], calcAndPadAreas.m_pad);
}

void AdaptiveLoopFilterEcm::deriveFixedFilterResultsCtuBoundaryChroma(const CPelBuf &src, const CPelBuf &srcBeforeDb,
                                                                      const Area &blkDst, const int qp, int ctuIdx,
                                                                      const FixFiltSetCand fixedFilterSetCandIdx,
                                                                      const CompID         compId)
{
  // Required boundary sizes for calculating the following filter(s).
  const unsigned requiredSize = 2;

  // Desired boundary sizes enabling the use of SIMD (>= required sizes).
  const unsigned desiredWidth = 8;
  static_assert(desiredWidth >= requiredSize, "Desired size for chroma fixed filter is too small.");

  // Get the neighboring areas for fixed filter calculation and padding.
  const NeighborCalcAndPadAreas calcAndPadAreas = calcCtuBoundaryCalcAndPadAreas(
    blkDst, requiredSize, desiredWidth, requiredSize, ALF_PADDING_SIZE_FIXED_RESULTS, false);

  // Check whether the fixed filter needs to be calculated for a neighboring area.
  const auto needsCalc = [compId, this](const int ctuIdx) { return !m_ctuAlreadyProcessedFlag[compId][ctuIdx]; };

  // Derive the fixed filter results for the required areas of the neighboring CTUs where they are not available.
  for (unsigned boundaryIdx = 0; boundaryIdx < static_cast<unsigned>(BndryLocMain::NUM); ++boundaryIdx)
  {
    const CalculationArea &calcArea = calcAndPadAreas.m_calc[boundaryIdx];
    if (calcArea.m_valid && needsCalc(ctuIdx + calcArea.m_ctuIdxOffset))
    {
      deriveFixedFilterResultsPerBlkChroma(src, srcBeforeDb, calcArea, qp, fixedFilterSetCandIdx, compId);
    }
  }

  extendBlockBoundaries(m_fixFilterResult[compId][fixedFilterSetCandIdx][FixFiltIdx::FIRST], calcAndPadAreas.m_pad);
}

void AdaptiveLoopFilterEcm::deriveFixedFilterResultsPerBlk(const CPelBuf &srcLuma, const CPelBuf &srcLumaBeforeDb,
                                                           const Area &blkCur, const int qp,
                                                           const FixFiltSetCand fixedFilterSetCandIdx,
                                                           const FixFiltIdx     fixedFilterIdx)
{
  const unsigned fixedFilterSetIdx = FixedFilterSetCands::getFixedFilterSet(qp, fixedFilterSetCandIdx);
  if (fixedFilterIdx == FixFiltIdx::FIRST)
  {
    CompBuf &fixFilterResult = m_fixFilterResult[COMP_Y][fixedFilterSetCandIdx][FixFiltIdx::FIRST];

    m_deriveVariance(srcLuma, blkCur, m_laplacian);
    m_deriveClassificationLaplacian(srcLuma, blkCur, m_laplacian, ALF_CLASSIFIER_FL);
    m_calcClassGradBasedFixedFilt(m_classifier[ClassifierIndex::FIXED_FILTER_4x4], blkCur, blkCur,
                                  ClsWndwSize::WNDW_4x4, m_inputBitDepth[ChannelType::LUMA], m_laplacian);

    alfFirstFixedFilterBlk(srcLuma, srcLumaBeforeDb, blkCur, fixFilterResult, blkCur, fixedFilterSetIdx);

    m_deriveClassificationLaplacianBig(blkCur, m_laplacian);
    m_calcClassGradBasedFixedFilt(m_classifier[ClassifierIndex::FIXED_FILTER_12x12], blkCur, blkCur,
                                  ClsWndwSize::WNDW_12x12, m_inputBitDepth[ChannelType::LUMA], m_laplacian);
  }
  else
  {
    const CompBuf &firstFixFilterResult  = m_fixFilterResult[COMP_Y][fixedFilterSetCandIdx][FixFiltIdx::FIRST];
    CompBuf       &secondFixFilterResult = m_fixFilterResult[COMP_Y][fixedFilterSetCandIdx][FixFiltIdx::SECOND];

    alfSecondFixedFilterBlk(srcLuma, srcLumaBeforeDb, firstFixFilterResult, blkCur, secondFixFilterResult, blkCur,
                            fixedFilterSetIdx);
  }
}

void AdaptiveLoopFilterEcm::deriveFixedFilterResultsPerBlkChroma(const CPelBuf &src, const CPelBuf &srcBeforeDb,
                                                                 const Area &blkCur, const int qp,
                                                                 const FixFiltSetCand fixedFilterSetCandIdx,
                                                                 const CompID         compId)
{
  const unsigned fixedFilterSetIdx = FixedFilterSetCands::getFixedFilterSet(qp, fixedFilterSetCandIdx);

  m_deriveVariance(src, blkCur, m_laplacian);
  m_deriveClassificationLaplacian(src, blkCur, m_laplacian, ALF_CLASSIFIER_FL_CHROMA);
  m_calcClassGradBasedFixedFilt(m_classifier[ClassifierIndex::FIXED_FILTER_CHROMA], blkCur, blkCur,
                                ClsWndwSize::WNDW_CHROMA, m_inputBitDepth[ChannelType::CHROMA], m_laplacian);

  alfChromaFixedFilterBlk(src, srcBeforeDb, blkCur, m_fixFilterResult[compId][fixedFilterSetCandIdx][FixFiltIdx::FIRST],
                          blkCur, fixedFilterSetIdx, compId);
}

void AdaptiveLoopFilterEcm::deriveGaussResults(const CPelBuf &srcLumaBeforeDb, const Area &blkDst, const Area &blk)
{
  const int height = blk.y + blk.height;
  const int width  = blk.x + blk.width;

  for (int i = blk.y; i < height; i += CLASSIFICATION_BLK_SIZE)
  {
    const int nHeight = std::min(i + CLASSIFICATION_BLK_SIZE, height) - i;

    for (int j = blk.x; j < width; j += CLASSIFICATION_BLK_SIZE)
    {
      const int nWidth = std::min(j + CLASSIFICATION_BLK_SIZE, width) - j;
      deriveGaussResultsBlk(
        m_gaussPic, srcLumaBeforeDb, Area(j - blk.x + blkDst.x, i - blk.y + blkDst.y, nWidth, nHeight),
        Area(j, i, nWidth, nHeight), m_clpRngs.comp[CompID::COMP_Y], m_alfClippingValues[ChannelType::LUMA]);
    }
  }
}

void AdaptiveLoopFilterEcm::deriveGaussResultsBlk(
  CompBuf &gaussPic, const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, const ClpRng &clpRng,
  const std::array<Pel, AdaptiveLoopFilter::MAX_ALF_NUM_CLIP_VALS> &clippingValues)
{
  const bool useSimd = blkDst.width % 8 == 0;
  if (useSimd)
  {
    m_gaussFiltering(gaussPic, srcLuma, blkDst, blk, clpRng, clippingValues);
  }
  else
  {
    gaussFiltering(gaussPic, srcLuma, blkDst, blk, clpRng, clippingValues);
  }
}

void AdaptiveLoopFilterEcm::deriveGaussResultsCtuBoundary(const CPelBuf &srcLuma, const Area &blkDst, int ctuIdx)
{
  // Required boundary size for calculating the following filter.
  const unsigned requiredSize = ALF_PADDING_SIZE_GAUSS_RESULTS;

  // Desired boundary size enabling the use of SIMD (>= required size).
  const unsigned desiredWidth = 8;
  static_assert(desiredWidth >= requiredSize, "Desired size is too small.");

  // Get the neighboring areas for Gaussian filter calculation and padding.
  const NeighborCalcAndPadAreas calcAndPadAreas = calcCtuBoundaryCalcAndPadAreas(
    blkDst, requiredSize, desiredWidth, requiredSize, ALF_PADDING_SIZE_GAUSS_RESULTS, true);

  // Check if the Gaussian filter is enabled for a neighboring CTU.
  const auto isGaussFilterEnabled = [](const int ctbMode) { return LumaCtbModeHandler::isApsFilter(ctbMode); };

  // Check if the Gaussian filter needs to be calculated for a neighboring area.
  const auto needsCalc = [&isGaussFilterEnabled, this](const int ctuIdx)
  { return !isGaussFilterEnabled((*m_modes)[CompID::COMP_Y][ctuIdx]) || !m_ctuAlreadyProcessedFlag[COMP_Y][ctuIdx]; };

  // Derive the Gaussian filter results for the required areas of the neighboring CTUs where they are not available.
  for (unsigned boundaryIdx = 0; boundaryIdx < static_cast<unsigned>(BndryLoc::NUM); ++boundaryIdx)
  {
    const CalculationArea &calcArea = calcAndPadAreas.m_calc[boundaryIdx];
    if (calcArea.m_valid && needsCalc(ctuIdx + calcArea.m_ctuIdxOffset))
    {
      deriveGaussResultsBlk(m_gaussPic, srcLuma, calcArea, calcArea, m_clpRngs.comp[CompID::COMP_Y],
                            m_alfClippingValues[ChannelType::LUMA]);
    }
  }

  // Add padding at the picture boundary.
  extendBlockBoundaries(m_gaussPic, calcAndPadAreas.m_pad);
}

void AdaptiveLoopFilterEcm::fixedFilter9x9Db9Blk(
  const ClassBuf &classifier, const CPelBuf &src, const CPelBuf &srcBeforeDb, const Area &curBlk,
  CompBuf &fixedFilterResult, const Area &blkDst, int picWidth, const unsigned fixedFilterSetIdx, const ClpRng &clpRng,
  const std::array<Pel, AdaptiveLoopFilter::MAX_ALF_NUM_CLIP_VALS> &clippingValues)
{
  const int shift  = FIX_FILTER_COEFF_SCALE_BITS;
  const int offset = 1 << (shift - 1);

  const int posYDst = blkDst.y;
  const int posXDst = blkDst.x;

  const int posY = curBlk.y;
  const int posX = curBlk.x;

  const int clsSizeY = 2;
  const int clsSizeX = 2;

  const ptrdiff_t srcStride          = src.stride;
  const ptrdiff_t srcBeforeDbStride  = srcBeforeDb.stride;
  const ptrdiff_t srcStride2         = srcStride * clsSizeY;
  const ptrdiff_t srcBeforeDbStride2 = srcBeforeDbStride * clsSizeY;

  const Pel *pImgYPad0         = src.buf + posY * srcStride + posX;
  const Pel *pImgYBeforeDbPad0 = srcBeforeDb.buf + posY * srcBeforeDbStride + posX;

  constexpr unsigned numCoeff = FIX_FILTER_NUM_COEFF_9_DB_9;
  std::vector<short> filterCoeff(numCoeff);
  std::vector<short> filterClipp(numCoeff);

  for (int i = 0; i < curBlk.height; i += clsSizeY)
  {
    for (int j = 0; j < curBlk.width; j += clsSizeX)
    {
      const int classIdx     = classifier[posYDst + i][posXDst + j] >> 2;
      const int transposeIdx = classifier[posYDst + i][posXDst + j] & 0x3;

      const int   filterIdx = m_classIdnFixedFilter9Db9[fixedFilterSetIdx][classIdx];
      const auto &coeff     = m_filterCoeffFixed9Db9[fixedFilterSetIdx][filterIdx];
      const auto &clipp     = m_clippingFixed9Db9[fixedFilterSetIdx][filterIdx];
      if (transposeIdx == 0)
      {
        filterCoeff = { coeff[0],  coeff[1],  coeff[2],  coeff[3],  coeff[4],  coeff[5],  coeff[6],
                        coeff[7],  coeff[8],  coeff[9],  coeff[10], coeff[11], coeff[12], coeff[13],
                        coeff[14], coeff[15], coeff[16], coeff[17], coeff[18], coeff[19], coeff[20],
                        coeff[21], coeff[22], coeff[23], coeff[24], coeff[25], coeff[26], coeff[27],
                        coeff[28], coeff[29], coeff[30], coeff[31], coeff[32], coeff[33], coeff[34],
                        coeff[35], coeff[36], coeff[37], coeff[38], coeff[39], coeff[40] };
        filterClipp = { clipp[0],  clipp[1],  clipp[2],  clipp[3],  clipp[4],  clipp[5],  clipp[6],
                        clipp[7],  clipp[8],  clipp[9],  clipp[10], clipp[11], clipp[12], clipp[13],
                        clipp[14], clipp[15], clipp[16], clipp[17], clipp[18], clipp[19], clipp[20],
                        clipp[21], clipp[22], clipp[23], clipp[24], clipp[25], clipp[26], clipp[27],
                        clipp[28], clipp[29], clipp[30], clipp[31], clipp[32], clipp[33], clipp[34],
                        clipp[35], clipp[36], clipp[37], clipp[38], clipp[39], clipp[40] };
      }
      else if (transposeIdx == 1)
      {
        filterCoeff = { coeff[16], coeff[9],  coeff[17], coeff[15], coeff[4],  coeff[10], coeff[18],
                        coeff[14], coeff[8],  coeff[1],  coeff[5],  coeff[11], coeff[19], coeff[13],
                        coeff[7],  coeff[3],  coeff[0],  coeff[2],  coeff[6],  coeff[12], coeff[36],
                        coeff[29], coeff[37], coeff[35], coeff[24], coeff[30], coeff[38], coeff[34],
                        coeff[28], coeff[21], coeff[25], coeff[31], coeff[39], coeff[33], coeff[27],
                        coeff[23], coeff[20], coeff[22], coeff[26], coeff[32], coeff[40] };
        filterClipp = { clipp[16], clipp[9],  clipp[17], clipp[15], clipp[4],  clipp[10], clipp[18],
                        clipp[14], clipp[8],  clipp[1],  clipp[5],  clipp[11], clipp[19], clipp[13],
                        clipp[7],  clipp[3],  clipp[0],  clipp[2],  clipp[6],  clipp[12], clipp[36],
                        clipp[29], clipp[37], clipp[35], clipp[24], clipp[30], clipp[38], clipp[34],
                        clipp[28], clipp[21], clipp[25], clipp[31], clipp[39], clipp[33], clipp[27],
                        clipp[23], clipp[20], clipp[22], clipp[26], clipp[32], clipp[40] };
      }
      else if (transposeIdx == 2)
      {
        filterCoeff = { coeff[0],  coeff[3],  coeff[2],  coeff[1],  coeff[8],  coeff[7],  coeff[6],
                        coeff[5],  coeff[4],  coeff[15], coeff[14], coeff[13], coeff[12], coeff[11],
                        coeff[10], coeff[9],  coeff[16], coeff[17], coeff[18], coeff[19], coeff[20],
                        coeff[23], coeff[22], coeff[21], coeff[28], coeff[27], coeff[26], coeff[25],
                        coeff[24], coeff[35], coeff[34], coeff[33], coeff[32], coeff[31], coeff[30],
                        coeff[29], coeff[36], coeff[37], coeff[38], coeff[39], coeff[40] };
        filterClipp = { clipp[0],  clipp[3],  clipp[2],  clipp[1],  clipp[8],  clipp[7],  clipp[6],
                        clipp[5],  clipp[4],  clipp[15], clipp[14], clipp[13], clipp[12], clipp[11],
                        clipp[10], clipp[9],  clipp[16], clipp[17], clipp[18], clipp[19], clipp[20],
                        clipp[23], clipp[22], clipp[21], clipp[28], clipp[27], clipp[26], clipp[25],
                        clipp[24], clipp[35], clipp[34], clipp[33], clipp[32], clipp[31], clipp[30],
                        clipp[29], clipp[36], clipp[37], clipp[38], clipp[39], clipp[40] };
      }
      else
      {
        filterCoeff = { coeff[16], coeff[15], coeff[17], coeff[9],  coeff[8],  coeff[14], coeff[18],
                        coeff[10], coeff[4],  coeff[3],  coeff[7],  coeff[13], coeff[19], coeff[11],
                        coeff[5],  coeff[1],  coeff[0],  coeff[2],  coeff[6],  coeff[12], coeff[36],
                        coeff[35], coeff[37], coeff[29], coeff[28], coeff[34], coeff[38], coeff[30],
                        coeff[24], coeff[23], coeff[27], coeff[33], coeff[39], coeff[31], coeff[25],
                        coeff[21], coeff[20], coeff[22], coeff[26], coeff[32], coeff[40] };
        filterClipp = { clipp[16], clipp[15], clipp[17], clipp[9],  clipp[8],  clipp[14], clipp[18],
                        clipp[10], clipp[4],  clipp[3],  clipp[7],  clipp[13], clipp[19], clipp[11],
                        clipp[5],  clipp[1],  clipp[0],  clipp[2],  clipp[6],  clipp[12], clipp[36],
                        clipp[35], clipp[37], clipp[29], clipp[28], clipp[34], clipp[38], clipp[30],
                        clipp[24], clipp[23], clipp[27], clipp[33], clipp[39], clipp[31], clipp[25],
                        clipp[21], clipp[20], clipp[22], clipp[26], clipp[32], clipp[40] };
      }

      for (short &currClip: filterClipp)
      {
        currClip = clippingValues[currClip];
      }

      for (int ii = 0; ii < clsSizeY; ii++)
      {
        const Pel *pImg0Cur      = pImgYPad0 + ii * srcStride + j;
        const Pel *pImg0BeforeDb = pImgYBeforeDbPad0 + ii * srcBeforeDbStride + j;
        for (int jj = 0; jj < clsSizeX; jj++)
        {
          const Pel curr = pImg0Cur[0];

          const FilterRowPtrs<const Pel, 9> pImg(pImg0Cur, srcStride);
          const FilterRowPtrs<const Pel, 9> pImgBeforeDb(pImg0BeforeDb, srcBeforeDbStride);

          int sum = 0;

          static_assert(FIX_FILTER_NUM_COEFF_9_DB_9 == 41, "Unexpected number of coefficients.");

          // clang-format off
          sum += filterCoeff[0]  * (clipALF(filterClipp[0],  curr, pImg[-4][+0], pImg[+4][+0]));

          sum += filterCoeff[1]  * (clipALF(filterClipp[1],  curr, pImg[-3][-1], pImg[+3][+1]));
          sum += filterCoeff[2]  * (clipALF(filterClipp[2],  curr, pImg[-3][+0], pImg[+3][+0]));
          sum += filterCoeff[3]  * (clipALF(filterClipp[3],  curr, pImg[-3][+1], pImg[+3][-1]));

          sum += filterCoeff[4]  * (clipALF(filterClipp[4],  curr, pImg[-2][-2], pImg[+2][+2]));
          sum += filterCoeff[5]  * (clipALF(filterClipp[5],  curr, pImg[-2][-1], pImg[+2][+1]));
          sum += filterCoeff[6]  * (clipALF(filterClipp[6],  curr, pImg[-2][+0], pImg[+2][+0]));
          sum += filterCoeff[7]  * (clipALF(filterClipp[7],  curr, pImg[-2][+1], pImg[+2][-1]));
          sum += filterCoeff[8]  * (clipALF(filterClipp[8],  curr, pImg[-2][+2], pImg[+2][-2]));

          sum += filterCoeff[9]  * (clipALF(filterClipp[9],  curr, pImg[-1][-3], pImg[+1][+3]));
          sum += filterCoeff[10] * (clipALF(filterClipp[10], curr, pImg[-1][-2], pImg[+1][+2]));
          sum += filterCoeff[11] * (clipALF(filterClipp[11], curr, pImg[-1][-1], pImg[+1][+1]));
          sum += filterCoeff[12] * (clipALF(filterClipp[12], curr, pImg[-1][+0], pImg[+1][+0]));
          sum += filterCoeff[13] * (clipALF(filterClipp[13], curr, pImg[-1][+1], pImg[+1][-1]));
          sum += filterCoeff[14] * (clipALF(filterClipp[14], curr, pImg[-1][+2], pImg[+1][-2]));
          sum += filterCoeff[15] * (clipALF(filterClipp[15], curr, pImg[-1][+3], pImg[+1][-3]));

          sum += filterCoeff[16] * (clipALF(filterClipp[16], curr, pImg[-0][-4], pImg[+0][+4]));
          sum += filterCoeff[17] * (clipALF(filterClipp[17], curr, pImg[-0][-3], pImg[+0][+3]));
          sum += filterCoeff[18] * (clipALF(filterClipp[18], curr, pImg[-0][-2], pImg[+0][+2]));
          sum += filterCoeff[19] * (clipALF(filterClipp[19], curr, pImg[-0][-1], pImg[+0][+1]));

          sum += filterCoeff[20] * (clipALF(filterClipp[20], curr, pImgBeforeDb[-4][+0], pImgBeforeDb[+4][+0]));

          sum += filterCoeff[21] * (clipALF(filterClipp[21], curr, pImgBeforeDb[-3][-1], pImgBeforeDb[+3][+1]));
          sum += filterCoeff[22] * (clipALF(filterClipp[22], curr, pImgBeforeDb[-3][+0], pImgBeforeDb[+3][+0]));
          sum += filterCoeff[23] * (clipALF(filterClipp[23], curr, pImgBeforeDb[-3][+1], pImgBeforeDb[+3][-1]));

          sum += filterCoeff[24] * (clipALF(filterClipp[24], curr, pImgBeforeDb[-2][-2], pImgBeforeDb[+2][+2]));
          sum += filterCoeff[25] * (clipALF(filterClipp[25], curr, pImgBeforeDb[-2][-1], pImgBeforeDb[+2][+1]));
          sum += filterCoeff[26] * (clipALF(filterClipp[26], curr, pImgBeforeDb[-2][+0], pImgBeforeDb[+2][+0]));
          sum += filterCoeff[27] * (clipALF(filterClipp[27], curr, pImgBeforeDb[-2][+1], pImgBeforeDb[+2][-1]));
          sum += filterCoeff[28] * (clipALF(filterClipp[28], curr, pImgBeforeDb[-2][+2], pImgBeforeDb[+2][-2]));

          sum += filterCoeff[29] * (clipALF(filterClipp[29], curr, pImgBeforeDb[-1][-3], pImgBeforeDb[+1][+3]));
          sum += filterCoeff[30] * (clipALF(filterClipp[30], curr, pImgBeforeDb[-1][-2], pImgBeforeDb[+1][+2]));
          sum += filterCoeff[31] * (clipALF(filterClipp[31], curr, pImgBeforeDb[-1][-1], pImgBeforeDb[+1][+1]));
          sum += filterCoeff[32] * (clipALF(filterClipp[32], curr, pImgBeforeDb[-1][+0], pImgBeforeDb[+1][+0]));
          sum += filterCoeff[33] * (clipALF(filterClipp[33], curr, pImgBeforeDb[-1][+1], pImgBeforeDb[+1][-1]));
          sum += filterCoeff[34] * (clipALF(filterClipp[34], curr, pImgBeforeDb[-1][+2], pImgBeforeDb[+1][-2]));
          sum += filterCoeff[35] * (clipALF(filterClipp[35], curr, pImgBeforeDb[-1][+3], pImgBeforeDb[+1][-3]));

          sum += filterCoeff[36] * (clipALF(filterClipp[36], curr, pImgBeforeDb[-0][-4], pImgBeforeDb[+0][+4]));
          sum += filterCoeff[37] * (clipALF(filterClipp[37], curr, pImgBeforeDb[-0][-3], pImgBeforeDb[+0][+3]));
          sum += filterCoeff[38] * (clipALF(filterClipp[38], curr, pImgBeforeDb[-0][-2], pImgBeforeDb[+0][+2]));
          sum += filterCoeff[39] * (clipALF(filterClipp[39], curr, pImgBeforeDb[-0][-1], pImgBeforeDb[+0][+1]));

          sum += filterCoeff[40] * (clipALF(filterClipp[40], curr, pImgBeforeDb[0][0]));
          // clang-format on

          sum = (sum + offset) >> shift;
          sum += curr;

          fixedFilterResult[posYDst + ii + i][posXDst + jj + j] = ClipPel(sum, clpRng);

          ++pImg0Cur;
          ++pImg0BeforeDb;
        }   // jj
      }   // ii
    }   // j
    pImgYBeforeDbPad0 += srcBeforeDbStride2;
    pImgYPad0 += srcStride2;
  }
}

void AdaptiveLoopFilterEcm::fixedFilter13x13Db9Blk(
  const ClassBuf &classifier, const CPelBuf &srcLuma, const CPelBuf &srcLumaBeforeDb,
  const CompBuf &firstFixedFilterResult, const Area &curBlk, CompBuf &fixedFilterResult, const Area &blkDst,
  int picWidth, const unsigned fixedFilterSetIdx, const ClpRng &clpRng,
  const std::array<Pel, AdaptiveLoopFilter::MAX_ALF_NUM_CLIP_VALS> &clippingValues)
{
  const int shift  = FIX_FILTER_COEFF_SCALE_BITS;
  const int offset = 1 << (shift - 1);

  const int posYDst = blkDst.y;
  const int posXDst = blkDst.x;

  const int posY = curBlk.y;
  const int posX = curBlk.x;

  const int clsSizeY = 2;
  const int clsSizeX = 2;

  const ptrdiff_t srcStride              = srcLuma.stride;
  const ptrdiff_t srcBeforeDbStride      = srcLumaBeforeDb.stride;
  const ptrdiff_t firstFixFiltResStride  = firstFixedFilterResult.stride;
  const ptrdiff_t srcStride2             = srcStride * clsSizeY;
  const ptrdiff_t srcBeforeDbStride2     = srcBeforeDbStride * clsSizeY;
  const ptrdiff_t firstFixFiltResStride2 = firstFixFiltResStride * clsSizeY;

  const Pel *pImgYPad0            = srcLuma.buf + posY * srcStride + posX;
  const Pel *pImgYBeforeDbPad0    = srcLumaBeforeDb.buf + posY * srcBeforeDbStride + posX;
  const Pel *pFirstFixFiltResPad0 = firstFixedFilterResult.buf + posY * firstFixFiltResStride + posX;

  constexpr unsigned numCoeff = FIX_FILTER_NUM_COEFF_13_DB_9;
  std::vector<short> filterCoeff(numCoeff);
  std::vector<short> filterClipp(numCoeff);

  for (int i = 0; i < curBlk.height; i += clsSizeY)
  {
    for (int j = 0; j < curBlk.width; j += clsSizeX)
    {
      const int classIdx     = classifier[posYDst + i][posXDst + j] >> 2;
      const int transposeIdx = classifier[posYDst + i][posXDst + j] & 0x3;

      const int   filterIdx = m_classIdnFixedFilter13Db9[fixedFilterSetIdx][classIdx];
      const auto &coeff     = m_filterCoeffFixed13Db9[fixedFilterSetIdx][filterIdx];
      const auto &clipp     = m_clippingFixed13Db9[fixedFilterSetIdx][filterIdx];
      if (transposeIdx == 0)
      {
        filterCoeff = { coeff[0],  coeff[1],  coeff[2],  coeff[3],  coeff[4],  coeff[5],  coeff[6],  coeff[7],
                        coeff[8],  coeff[9],  coeff[10], coeff[11], coeff[12], coeff[13], coeff[14], coeff[15],
                        coeff[16], coeff[17], coeff[18], coeff[19], coeff[20], coeff[21], coeff[22], coeff[23],
                        coeff[24], coeff[25], coeff[26], coeff[27], coeff[28], coeff[29], coeff[30], coeff[31],
                        coeff[32], coeff[33], coeff[34], coeff[35], coeff[36], coeff[37], coeff[38], coeff[39],
                        coeff[40], coeff[41], coeff[42], coeff[43], coeff[44], coeff[45], coeff[46], coeff[47],
                        coeff[48], coeff[49], coeff[50], coeff[51], coeff[52], coeff[53], coeff[54], coeff[55],
                        coeff[56], coeff[57], coeff[58], coeff[59], coeff[60], coeff[61], coeff[62], coeff[63] };
        filterClipp = { clipp[0],  clipp[1],  clipp[2],  clipp[3],  clipp[4],  clipp[5],  clipp[6],  clipp[7],
                        clipp[8],  clipp[9],  clipp[10], clipp[11], clipp[12], clipp[13], clipp[14], clipp[15],
                        clipp[16], clipp[17], clipp[18], clipp[19], clipp[20], clipp[21], clipp[22], clipp[23],
                        clipp[24], clipp[25], clipp[26], clipp[27], clipp[28], clipp[29], clipp[30], clipp[31],
                        clipp[32], clipp[33], clipp[34], clipp[35], clipp[36], clipp[37], clipp[38], clipp[39],
                        clipp[40], clipp[41], clipp[42], clipp[43], clipp[44], clipp[45], clipp[46], clipp[47],
                        clipp[48], clipp[49], clipp[50], clipp[51], clipp[52], clipp[53], clipp[54], clipp[55],
                        clipp[56], clipp[57], clipp[58], clipp[59], clipp[60], clipp[61], clipp[62], clipp[63] };
      }
      else if (transposeIdx == 1)
      {
        filterCoeff = { coeff[36], coeff[25], coeff[37], coeff[35], coeff[16], coeff[26], coeff[38], coeff[34],
                        coeff[24], coeff[9],  coeff[17], coeff[27], coeff[39], coeff[33], coeff[23], coeff[15],
                        coeff[4],  coeff[10], coeff[18], coeff[28], coeff[40], coeff[32], coeff[22], coeff[14],
                        coeff[8],  coeff[1],  coeff[5],  coeff[11], coeff[19], coeff[29], coeff[41], coeff[31],
                        coeff[21], coeff[13], coeff[7],  coeff[3],  coeff[0],  coeff[2],  coeff[6],  coeff[12],
                        coeff[20], coeff[30], coeff[58], coeff[51], coeff[59], coeff[57], coeff[46], coeff[52],
                        coeff[60], coeff[56], coeff[50], coeff[43], coeff[47], coeff[53], coeff[61], coeff[55],
                        coeff[49], coeff[45], coeff[42], coeff[44], coeff[48], coeff[54], coeff[62], coeff[63] };
        filterClipp = { clipp[36], clipp[25], clipp[37], clipp[35], clipp[16], clipp[26], clipp[38], clipp[34],
                        clipp[24], clipp[9],  clipp[17], clipp[27], clipp[39], clipp[33], clipp[23], clipp[15],
                        clipp[4],  clipp[10], clipp[18], clipp[28], clipp[40], clipp[32], clipp[22], clipp[14],
                        clipp[8],  clipp[1],  clipp[5],  clipp[11], clipp[19], clipp[29], clipp[41], clipp[31],
                        clipp[21], clipp[13], clipp[7],  clipp[3],  clipp[0],  clipp[2],  clipp[6],  clipp[12],
                        clipp[20], clipp[30], clipp[58], clipp[51], clipp[59], clipp[57], clipp[46], clipp[52],
                        clipp[60], clipp[56], clipp[50], clipp[43], clipp[47], clipp[53], clipp[61], clipp[55],
                        clipp[49], clipp[45], clipp[42], clipp[44], clipp[48], clipp[54], clipp[62], clipp[63] };
      }
      else if (transposeIdx == 2)
      {
        filterCoeff = { coeff[0],  coeff[3],  coeff[2],  coeff[1],  coeff[8],  coeff[7],  coeff[6],  coeff[5],
                        coeff[4],  coeff[15], coeff[14], coeff[13], coeff[12], coeff[11], coeff[10], coeff[9],
                        coeff[24], coeff[23], coeff[22], coeff[21], coeff[20], coeff[19], coeff[18], coeff[17],
                        coeff[16], coeff[35], coeff[34], coeff[33], coeff[32], coeff[31], coeff[30], coeff[29],
                        coeff[28], coeff[27], coeff[26], coeff[25], coeff[36], coeff[37], coeff[38], coeff[39],
                        coeff[40], coeff[41], coeff[42], coeff[45], coeff[44], coeff[43], coeff[50], coeff[49],
                        coeff[48], coeff[47], coeff[46], coeff[57], coeff[56], coeff[55], coeff[54], coeff[53],
                        coeff[52], coeff[51], coeff[58], coeff[59], coeff[60], coeff[61], coeff[62], coeff[63] };
        filterClipp = { clipp[0],  clipp[3],  clipp[2],  clipp[1],  clipp[8],  clipp[7],  clipp[6],  clipp[5],
                        clipp[4],  clipp[15], clipp[14], clipp[13], clipp[12], clipp[11], clipp[10], clipp[9],
                        clipp[24], clipp[23], clipp[22], clipp[21], clipp[20], clipp[19], clipp[18], clipp[17],
                        clipp[16], clipp[35], clipp[34], clipp[33], clipp[32], clipp[31], clipp[30], clipp[29],
                        clipp[28], clipp[27], clipp[26], clipp[25], clipp[36], clipp[37], clipp[38], clipp[39],
                        clipp[40], clipp[41], clipp[42], clipp[45], clipp[44], clipp[43], clipp[50], clipp[49],
                        clipp[48], clipp[47], clipp[46], clipp[57], clipp[56], clipp[55], clipp[54], clipp[53],
                        clipp[52], clipp[51], clipp[58], clipp[59], clipp[60], clipp[61], clipp[62], clipp[63] };
      }
      else
      {
        filterCoeff = { coeff[36], coeff[35], coeff[37], coeff[25], coeff[24], coeff[34], coeff[38], coeff[26],
                        coeff[16], coeff[15], coeff[23], coeff[33], coeff[39], coeff[27], coeff[17], coeff[9],
                        coeff[8],  coeff[14], coeff[22], coeff[32], coeff[40], coeff[28], coeff[18], coeff[10],
                        coeff[4],  coeff[3],  coeff[7],  coeff[13], coeff[21], coeff[31], coeff[41], coeff[29],
                        coeff[19], coeff[11], coeff[5],  coeff[1],  coeff[0],  coeff[2],  coeff[6],  coeff[12],
                        coeff[20], coeff[30], coeff[58], coeff[57], coeff[59], coeff[51], coeff[50], coeff[56],
                        coeff[60], coeff[52], coeff[46], coeff[45], coeff[49], coeff[55], coeff[61], coeff[53],
                        coeff[47], coeff[43], coeff[42], coeff[44], coeff[48], coeff[54], coeff[62], coeff[63] };
        filterClipp = { clipp[36], clipp[35], clipp[37], clipp[25], clipp[24], clipp[34], clipp[38], clipp[26],
                        clipp[16], clipp[15], clipp[23], clipp[33], clipp[39], clipp[27], clipp[17], clipp[9],
                        clipp[8],  clipp[14], clipp[22], clipp[32], clipp[40], clipp[28], clipp[18], clipp[10],
                        clipp[4],  clipp[3],  clipp[7],  clipp[13], clipp[21], clipp[31], clipp[41], clipp[29],
                        clipp[19], clipp[11], clipp[5],  clipp[1],  clipp[0],  clipp[2],  clipp[6],  clipp[12],
                        clipp[20], clipp[30], clipp[58], clipp[57], clipp[59], clipp[51], clipp[50], clipp[56],
                        clipp[60], clipp[52], clipp[46], clipp[45], clipp[49], clipp[55], clipp[61], clipp[53],
                        clipp[47], clipp[43], clipp[42], clipp[44], clipp[48], clipp[54], clipp[62], clipp[63] };
      }

      for (short &currClip: filterClipp)
      {
        currClip = clippingValues[currClip];
      }

      for (int ii = 0; ii < clsSizeY; ii++)
      {
        const Pel *pImg0Cur         = pImgYPad0 + ii * srcStride + j;
        const Pel *pImg0BeforeDb    = pImgYBeforeDbPad0 + ii * srcBeforeDbStride + j;
        const Pel *pFirstFixFiltRes = pFirstFixFiltResPad0 + ii * firstFixFiltResStride + j;
        for (int jj = 0; jj < clsSizeX; jj++)
        {
          const Pel curr = pImg0Cur[0];

          const FilterRowPtrs<const Pel, 13> pImg(pFirstFixFiltRes, firstFixFiltResStride);
          const FilterRowPtrs<const Pel, 9>  pImgBeforeDb(pImg0BeforeDb, srcBeforeDbStride);

          int sum = 0;

          static_assert(FIX_FILTER_NUM_COEFF_13_DB_9 == 64, "Unexpected number of coefficients.");

          // clang-format off
          sum += filterCoeff[0]  * (clipALF(filterClipp[0],  curr, pImg[-6][+0], pImg[+6][+0]));

          sum += filterCoeff[1]  * (clipALF(filterClipp[1],  curr, pImg[-5][-1], pImg[+5][+1]));
          sum += filterCoeff[2]  * (clipALF(filterClipp[2],  curr, pImg[-5][+0], pImg[+5][+0]));
          sum += filterCoeff[3]  * (clipALF(filterClipp[3],  curr, pImg[-5][+1], pImg[+5][-1]));

          sum += filterCoeff[4]  * (clipALF(filterClipp[4],  curr, pImg[-4][-2], pImg[+4][+2]));
          sum += filterCoeff[5]  * (clipALF(filterClipp[5],  curr, pImg[-4][-1], pImg[+4][+1]));
          sum += filterCoeff[6]  * (clipALF(filterClipp[6],  curr, pImg[-4][+0], pImg[+4][+0]));
          sum += filterCoeff[7]  * (clipALF(filterClipp[7],  curr, pImg[-4][+1], pImg[+4][-1]));
          sum += filterCoeff[8]  * (clipALF(filterClipp[8],  curr, pImg[-4][+2], pImg[+4][-2]));

          sum += filterCoeff[9]  * (clipALF(filterClipp[9],  curr, pImg[-3][-3], pImg[+3][+3]));
          sum += filterCoeff[10] * (clipALF(filterClipp[10], curr, pImg[-3][-2], pImg[+3][+2]));
          sum += filterCoeff[11] * (clipALF(filterClipp[11], curr, pImg[-3][-1], pImg[+3][+1]));
          sum += filterCoeff[12] * (clipALF(filterClipp[12], curr, pImg[-3][+0], pImg[+3][+0]));
          sum += filterCoeff[13] * (clipALF(filterClipp[13], curr, pImg[-3][+1], pImg[+3][-1]));
          sum += filterCoeff[14] * (clipALF(filterClipp[14], curr, pImg[-3][+2], pImg[+3][-2]));
          sum += filterCoeff[15] * (clipALF(filterClipp[15], curr, pImg[-3][+3], pImg[+3][-3]));

          sum += filterCoeff[16] * (clipALF(filterClipp[16], curr, pImg[-2][-4], pImg[+2][+4]));
          sum += filterCoeff[17] * (clipALF(filterClipp[17], curr, pImg[-2][-3], pImg[+2][+3]));
          sum += filterCoeff[18] * (clipALF(filterClipp[18], curr, pImg[-2][-2], pImg[+2][+2]));
          sum += filterCoeff[19] * (clipALF(filterClipp[19], curr, pImg[-2][-1], pImg[+2][+1]));
          sum += filterCoeff[20] * (clipALF(filterClipp[20], curr, pImg[-2][+0], pImg[+2][+0]));
          sum += filterCoeff[21] * (clipALF(filterClipp[21], curr, pImg[-2][+1], pImg[+2][-1]));
          sum += filterCoeff[22] * (clipALF(filterClipp[22], curr, pImg[-2][+2], pImg[+2][-2]));
          sum += filterCoeff[23] * (clipALF(filterClipp[23], curr, pImg[-2][+3], pImg[+2][-3]));
          sum += filterCoeff[24] * (clipALF(filterClipp[24], curr, pImg[-2][+4], pImg[+2][-4]));

          sum += filterCoeff[25] * (clipALF(filterClipp[25], curr, pImg[-1][-5], pImg[+1][+5]));
          sum += filterCoeff[26] * (clipALF(filterClipp[26], curr, pImg[-1][-4], pImg[+1][+4]));
          sum += filterCoeff[27] * (clipALF(filterClipp[27], curr, pImg[-1][-3], pImg[+1][+3]));
          sum += filterCoeff[28] * (clipALF(filterClipp[28], curr, pImg[-1][-2], pImg[+1][+2]));
          sum += filterCoeff[29] * (clipALF(filterClipp[29], curr, pImg[-1][-1], pImg[+1][+1]));
          sum += filterCoeff[30] * (clipALF(filterClipp[30], curr, pImg[-1][+0], pImg[+1][+0]));
          sum += filterCoeff[31] * (clipALF(filterClipp[31], curr, pImg[-1][+1], pImg[+1][-1]));
          sum += filterCoeff[32] * (clipALF(filterClipp[32], curr, pImg[-1][+2], pImg[+1][-2]));
          sum += filterCoeff[33] * (clipALF(filterClipp[33], curr, pImg[-1][+3], pImg[+1][-3]));
          sum += filterCoeff[34] * (clipALF(filterClipp[34], curr, pImg[-1][+4], pImg[+1][-4]));
          sum += filterCoeff[35] * (clipALF(filterClipp[35], curr, pImg[-1][+5], pImg[+1][-5]));

          sum += filterCoeff[36] * (clipALF(filterClipp[36], curr, pImg[-0][-6], pImg[+0][+6]));
          sum += filterCoeff[37] * (clipALF(filterClipp[37], curr, pImg[-0][-5], pImg[+0][+5]));
          sum += filterCoeff[38] * (clipALF(filterClipp[38], curr, pImg[-0][-4], pImg[+0][+4]));
          sum += filterCoeff[39] * (clipALF(filterClipp[39], curr, pImg[-0][-3], pImg[+0][+3]));
          sum += filterCoeff[40] * (clipALF(filterClipp[40], curr, pImg[-0][-2], pImg[+0][+2]));
          sum += filterCoeff[41] * (clipALF(filterClipp[41], curr, pImg[-0][-1], pImg[+0][+1]));

          sum += filterCoeff[42] * (clipALF(filterClipp[42], curr, pImgBeforeDb[-4][+0], pImgBeforeDb[+4][+0]));

          sum += filterCoeff[43] * (clipALF(filterClipp[43], curr, pImgBeforeDb[-3][-1], pImgBeforeDb[+3][+1]));
          sum += filterCoeff[44] * (clipALF(filterClipp[44], curr, pImgBeforeDb[-3][+0], pImgBeforeDb[+3][+0]));
          sum += filterCoeff[45] * (clipALF(filterClipp[45], curr, pImgBeforeDb[-3][+1], pImgBeforeDb[+3][-1]));

          sum += filterCoeff[46] * (clipALF(filterClipp[46], curr, pImgBeforeDb[-2][-2], pImgBeforeDb[+2][+2]));
          sum += filterCoeff[47] * (clipALF(filterClipp[47], curr, pImgBeforeDb[-2][-1], pImgBeforeDb[+2][+1]));
          sum += filterCoeff[48] * (clipALF(filterClipp[48], curr, pImgBeforeDb[-2][+0], pImgBeforeDb[+2][+0]));
          sum += filterCoeff[49] * (clipALF(filterClipp[49], curr, pImgBeforeDb[-2][+1], pImgBeforeDb[+2][-1]));
          sum += filterCoeff[50] * (clipALF(filterClipp[50], curr, pImgBeforeDb[-2][+2], pImgBeforeDb[+2][-2]));

          sum += filterCoeff[51] * (clipALF(filterClipp[51], curr, pImgBeforeDb[-1][-3], pImgBeforeDb[+1][+3]));
          sum += filterCoeff[52] * (clipALF(filterClipp[52], curr, pImgBeforeDb[-1][-2], pImgBeforeDb[+1][+2]));
          sum += filterCoeff[53] * (clipALF(filterClipp[53], curr, pImgBeforeDb[-1][-1], pImgBeforeDb[+1][+1]));
          sum += filterCoeff[54] * (clipALF(filterClipp[54], curr, pImgBeforeDb[-1][+0], pImgBeforeDb[+1][+0]));
          sum += filterCoeff[55] * (clipALF(filterClipp[55], curr, pImgBeforeDb[-1][+1], pImgBeforeDb[+1][-1]));
          sum += filterCoeff[56] * (clipALF(filterClipp[56], curr, pImgBeforeDb[-1][+2], pImgBeforeDb[+1][-2]));
          sum += filterCoeff[57] * (clipALF(filterClipp[57], curr, pImgBeforeDb[-1][+3], pImgBeforeDb[+1][-3]));

          sum += filterCoeff[58] * (clipALF(filterClipp[58], curr, pImgBeforeDb[-0][-4], pImgBeforeDb[+0][+4]));
          sum += filterCoeff[59] * (clipALF(filterClipp[59], curr, pImgBeforeDb[-0][-3], pImgBeforeDb[+0][+3]));
          sum += filterCoeff[60] * (clipALF(filterClipp[60], curr, pImgBeforeDb[-0][-2], pImgBeforeDb[+0][+2]));
          sum += filterCoeff[61] * (clipALF(filterClipp[61], curr, pImgBeforeDb[-0][-1], pImgBeforeDb[+0][+1]));

          sum += filterCoeff[62] * (clipALF(filterClipp[62], curr, pImgBeforeDb[0][0]));

          sum += filterCoeff[63] * (clipALF(filterClipp[63], curr, pImg[0][0]));
          // clang-format on

          sum = (sum + offset) >> shift;
          sum += curr;

          fixedFilterResult[posYDst + ii + i][posXDst + jj + j] = ClipPel(sum, clpRng);

          ++pImg0Cur;
          ++pImg0BeforeDb;
          ++pFirstFixFiltRes;
        }   // jj
      }   // ii
    }   // j
    pImgYBeforeDbPad0 += srcBeforeDbStride2;
    pImgYPad0 += srcStride2;
    pFirstFixFiltResPad0 += firstFixFiltResStride2;
  }
}

void AdaptiveLoopFilterEcm::fixedFilteringResi(
  const ClassBuf &classifier, const CPelBuf &srcResiLuma, const Area &curBlk, const Area &blkDst,
  CompBuf &fixedFilterResiResult, int picWidth, const unsigned fixedFilterSetIdx,
  const short (&classIndFixed)[NUM_CLASSES_FIX], const ClpRng &clpRng,
  const std::array<Pel, AdaptiveLoopFilter::MAX_ALF_NUM_CLIP_VALS> &clippingValues)
{
  const int shift  = FIX_FILTER_COEFF_SCALE_BITS;
  const int offset = 1 << (shift - 1);

  const int posYDst = blkDst.y;
  const int posXDst = blkDst.x;

  int posY = curBlk.y;
  int posX = curBlk.x;

  const int clsSizeY = 2;
  const int clsSizeX = 2;

  const ptrdiff_t srcStride  = srcResiLuma.stride;
  const ptrdiff_t srcStride2 = srcStride * clsSizeY;

  const Pel *pImgYPad0 = srcResiLuma.buf + posY * srcStride + posX;

  for (int i = 0; i < curBlk.height; i += clsSizeY)
  {
    for (int j = 0; j < curBlk.width; j += clsSizeX)
    {
      int classIdx     = classifier[posYDst + i][posXDst + j] >> 2;
      int transposeIdx = classifier[posYDst + i][posXDst + j] & 0x3;
      int filterIdx    = classIndFixed[classIdx];

      const auto &coeff = m_filterCoeffFixed9Db9[fixedFilterSetIdx][filterIdx];
      const auto &clipp = m_clippingFixed9Db9[fixedFilterSetIdx][filterIdx];

      std::array<short, FIX_FILTER_NUM_COEFF_9_DB_9> filterCoeff;
      std::array<short, FIX_FILTER_NUM_COEFF_9_DB_9> filterClipp;

      if (transposeIdx == 0)
      {
        filterCoeff = { coeff[0],  coeff[1],  coeff[2],  coeff[3],  coeff[4],  coeff[5],  coeff[6],
                        coeff[7],  coeff[8],  coeff[9],  coeff[10], coeff[11], coeff[12], coeff[13],
                        coeff[14], coeff[15], coeff[16], coeff[17], coeff[18], coeff[19], coeff[20],
                        coeff[21], coeff[22], coeff[23], coeff[24], coeff[25], coeff[26], coeff[27],
                        coeff[28], coeff[29], coeff[30], coeff[31], coeff[32], coeff[33], coeff[34],
                        coeff[35], coeff[36], coeff[37], coeff[38], coeff[39], coeff[40] };
        filterClipp = { clipp[0],  clipp[1],  clipp[2],  clipp[3],  clipp[4],  clipp[5],  clipp[6],
                        clipp[7],  clipp[8],  clipp[9],  clipp[10], clipp[11], clipp[12], clipp[13],
                        clipp[14], clipp[15], clipp[16], clipp[17], clipp[18], clipp[19], clipp[20],
                        clipp[21], clipp[22], clipp[23], clipp[24], clipp[25], clipp[26], clipp[27],
                        clipp[28], clipp[29], clipp[30], clipp[31], clipp[32], clipp[33], clipp[34],
                        clipp[35], clipp[36], clipp[37], clipp[38], clipp[39], clipp[40] };
      }
      else if (transposeIdx == 1)
      {
        filterCoeff = { coeff[16], coeff[9],  coeff[17], coeff[15], coeff[4],  coeff[10], coeff[18],
                        coeff[14], coeff[8],  coeff[1],  coeff[5],  coeff[11], coeff[19], coeff[13],
                        coeff[7],  coeff[3],  coeff[0],  coeff[2],  coeff[6],  coeff[12], coeff[36],
                        coeff[29], coeff[37], coeff[35], coeff[24], coeff[30], coeff[38], coeff[34],
                        coeff[28], coeff[21], coeff[25], coeff[31], coeff[39], coeff[33], coeff[27],
                        coeff[23], coeff[20], coeff[22], coeff[26], coeff[32], coeff[40] };
        filterClipp = { clipp[16], clipp[9],  clipp[17], clipp[15], clipp[4],  clipp[10], clipp[18],
                        clipp[14], clipp[8],  clipp[1],  clipp[5],  clipp[11], clipp[19], clipp[13],
                        clipp[7],  clipp[3],  clipp[0],  clipp[2],  clipp[6],  clipp[12], clipp[36],
                        clipp[29], clipp[37], clipp[35], clipp[24], clipp[30], clipp[38], clipp[34],
                        clipp[28], clipp[21], clipp[25], clipp[31], clipp[39], clipp[33], clipp[27],
                        clipp[23], clipp[20], clipp[22], clipp[26], clipp[32], clipp[40] };
      }
      else if (transposeIdx == 2)
      {
        filterCoeff = { coeff[0],  coeff[3],  coeff[2],  coeff[1],  coeff[8],  coeff[7],  coeff[6],
                        coeff[5],  coeff[4],  coeff[15], coeff[14], coeff[13], coeff[12], coeff[11],
                        coeff[10], coeff[9],  coeff[16], coeff[17], coeff[18], coeff[19], coeff[20],
                        coeff[23], coeff[22], coeff[21], coeff[28], coeff[27], coeff[26], coeff[25],
                        coeff[24], coeff[35], coeff[34], coeff[33], coeff[32], coeff[31], coeff[30],
                        coeff[29], coeff[36], coeff[37], coeff[38], coeff[39], coeff[40] };
        filterClipp = { clipp[0],  clipp[3],  clipp[2],  clipp[1],  clipp[8],  clipp[7],  clipp[6],
                        clipp[5],  clipp[4],  clipp[15], clipp[14], clipp[13], clipp[12], clipp[11],
                        clipp[10], clipp[9],  clipp[16], clipp[17], clipp[18], clipp[19], clipp[20],
                        clipp[23], clipp[22], clipp[21], clipp[28], clipp[27], clipp[26], clipp[25],
                        clipp[24], clipp[35], clipp[34], clipp[33], clipp[32], clipp[31], clipp[30],
                        clipp[29], clipp[36], clipp[37], clipp[38], clipp[39], clipp[40] };
      }
      else
      {
        filterCoeff = { coeff[16], coeff[15], coeff[17], coeff[9],  coeff[8],  coeff[14], coeff[18],
                        coeff[10], coeff[4],  coeff[3],  coeff[7],  coeff[13], coeff[19], coeff[11],
                        coeff[5],  coeff[1],  coeff[0],  coeff[2],  coeff[6],  coeff[12], coeff[36],
                        coeff[35], coeff[37], coeff[29], coeff[28], coeff[34], coeff[38], coeff[30],
                        coeff[24], coeff[23], coeff[27], coeff[33], coeff[39], coeff[31], coeff[25],
                        coeff[21], coeff[20], coeff[22], coeff[26], coeff[32], coeff[40] };
        filterClipp = { clipp[16], clipp[15], clipp[17], clipp[9],  clipp[8],  clipp[14], clipp[18],
                        clipp[10], clipp[4],  clipp[3],  clipp[7],  clipp[13], clipp[19], clipp[11],
                        clipp[5],  clipp[1],  clipp[0],  clipp[2],  clipp[6],  clipp[12], clipp[36],
                        clipp[35], clipp[37], clipp[29], clipp[28], clipp[34], clipp[38], clipp[30],
                        clipp[24], clipp[23], clipp[27], clipp[33], clipp[39], clipp[31], clipp[25],
                        clipp[21], clipp[20], clipp[22], clipp[26], clipp[32], clipp[40] };
      }

      // Create the filter coefficients by adding the fixed filter coefficients for the pre-ALF and the pre-DBF inputs.
      constexpr unsigned numCoeff = FIX_FILTER_NUM_COEFF_9_DB_9 / 2;
      for (unsigned k = 0; k < numCoeff; ++k)
      {
        filterCoeff[k] = filterCoeff[k] + filterCoeff[numCoeff + k];
        filterClipp[k] = (filterClipp[k] + filterClipp[numCoeff + k] + 1) >> 1;
      }

      // Lookup the clipping coefficients for all positions including the center.
      for (unsigned k = 0; k < numCoeff; k++)
      {
        filterClipp[k] = clippingValues[filterClipp[k]];
      }
      filterClipp[FIX_FILTER_NUM_COEFF_9_DB_9 - 1] = clippingValues[filterClipp[FIX_FILTER_NUM_COEFF_9_DB_9 - 1]];

      static_assert(FIX_FILTER_NUM_COEFF_9_DB_9 == 41 && numCoeff == 20, "Unexpected number of filter coefficients.");

      for (int ii = 0; ii < clsSizeY; ii++)
      {
        const Pel *pImgCenter = pImgYPad0 + ii * srcStride + j;
        for (int jj = 0; jj < clsSizeX; jj++)
        {
          FilterRowPtrs<const Pel, 9> pImg(pImgCenter, srcStride);
          const Pel                   curr = pImgCenter[0];

          int sum = 0;
          // clang-format off
          sum += filterCoeff[0]  * (clipALF(filterClipp[0],  curr, pImg[-4][+0], pImg[+4][+0]));
                                                            
          sum += filterCoeff[1]  * (clipALF(filterClipp[1],  curr, pImg[-3][-1], pImg[+3][+1]));
          sum += filterCoeff[2]  * (clipALF(filterClipp[2],  curr, pImg[-3][+0], pImg[+3][+0]));
          sum += filterCoeff[3]  * (clipALF(filterClipp[3],  curr, pImg[-3][+1], pImg[+3][-1]));
                                                            
          sum += filterCoeff[4]  * (clipALF(filterClipp[4],  curr, pImg[-2][-2], pImg[+2][+2]));
          sum += filterCoeff[5]  * (clipALF(filterClipp[5],  curr, pImg[-2][-1], pImg[+2][+1]));
          sum += filterCoeff[6]  * (clipALF(filterClipp[6],  curr, pImg[-2][+0], pImg[+2][+0]));
          sum += filterCoeff[7]  * (clipALF(filterClipp[7],  curr, pImg[-2][+1], pImg[+2][-1]));
          sum += filterCoeff[8]  * (clipALF(filterClipp[8],  curr, pImg[-2][+2], pImg[+2][-2]));

          sum += filterCoeff[9]  * (clipALF(filterClipp[9] , curr, pImg[-1][-3], pImg[+1][+3]));
          sum += filterCoeff[10] * (clipALF(filterClipp[10], curr, pImg[-1][-2], pImg[+1][+2]));
          sum += filterCoeff[11] * (clipALF(filterClipp[11], curr, pImg[-1][-1], pImg[+1][+1]));
          sum += filterCoeff[12] * (clipALF(filterClipp[12], curr, pImg[-1][+0], pImg[+1][+0]));
          sum += filterCoeff[13] * (clipALF(filterClipp[13], curr, pImg[-1][+1], pImg[+1][-1]));
          sum += filterCoeff[14] * (clipALF(filterClipp[14], curr, pImg[-1][+2], pImg[+1][-2]));
          sum += filterCoeff[15] * (clipALF(filterClipp[15], curr, pImg[-1][+3], pImg[+1][-3]));

          sum += filterCoeff[16] * (clipALF(filterClipp[16], curr, pImg[-0][-4], pImg[+0][+4]));
          sum += filterCoeff[17] * (clipALF(filterClipp[17], curr, pImg[-0][-3], pImg[+0][+3]));
          sum += filterCoeff[18] * (clipALF(filterClipp[18], curr, pImg[-0][-2], pImg[+0][+2]));
          sum += filterCoeff[19] * (clipALF(filterClipp[19], curr, pImg[-0][-1], pImg[+0][+1]));

          sum += filterCoeff[40] * (clipALF(filterClipp[40], curr, 0));
          // clang-format on

          sum = (sum + offset) >> shift;
          sum += curr;

          fixedFilterResiResult[posYDst + ii + i][posXDst + jj + j] = Clip3<int>(-clpRng.max, clpRng.max, sum);

          ++pImgCenter;
        }   // jj
      }   // ii
    }   // j
    pImgYPad0 += srcStride2;
  }
}

void AdaptiveLoopFilterEcm::gaussFiltering(
  CompBuf &gaussPic, const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, const ClpRng &clpRng,
  const std::array<Pel, AdaptiveLoopFilter::MAX_ALF_NUM_CLIP_VALS> &clippingValues)
{
  const Pel *const   srcPtr   = srcLuma.bufAt(blk.pos());
  constexpr int      shift    = 10;
  constexpr int      diffTH   = 32;
  constexpr unsigned numCoeff = 12;

  // TODO: ALF: The coefficients are also defined in the SIMD code. BS 2023-09-08
  // clang-format off
  static constexpr short   gaussTable[numCoeff]     = {  8, 22, 30, 22, 22, 60, 85, 60, 22,  8, 30, 85 };
  static constexpr uint8_t gaussClipTable[numCoeff] = {  3,  2,  1,  2,  2,  1,  0,  1,  2,  3,  1,  0 };
  // clang-format on

  Pel clipValueTable[numCoeff];
  for (int cIdx = 0; cIdx < numCoeff; cIdx++)
  {
    clipValueTable[cIdx] = clippingValues[gaussClipTable[cIdx]];
  }

  for (int i = 0; i < blk.height; ++i)
  {
    for (int j = 0; j < blk.width; ++j)
    {
      FilterRowPtrs<const Pel, 7> pImg(srcPtr + i * srcLuma.stride + j, srcLuma.stride);
      const Pel                   curr = pImg[0][0];

      int sum = 0;
      // clang-format off
      sum += gaussTable[0]  * clipALF(clipValueTable[0],  curr, pImg[-3][+0], pImg[+3][-0]);

      sum += gaussTable[1]  * clipALF(clipValueTable[1],  curr, pImg[-2][-1], pImg[+2][+1]);
      sum += gaussTable[2]  * clipALF(clipValueTable[2],  curr, pImg[-2][-0], pImg[+2][+0]);
      sum += gaussTable[3]  * clipALF(clipValueTable[3],  curr, pImg[-2][+1], pImg[+2][-1]);

      sum += gaussTable[4]  * clipALF(clipValueTable[4],  curr, pImg[-1][-2], pImg[+1][+2]);
      sum += gaussTable[5]  * clipALF(clipValueTable[5],  curr, pImg[-1][-1], pImg[+1][+1]);
      sum += gaussTable[6]  * clipALF(clipValueTable[6],  curr, pImg[-1][-0], pImg[+1][+0]);
      sum += gaussTable[7]  * clipALF(clipValueTable[7],  curr, pImg[-1][+1], pImg[+1][-1]);
      sum += gaussTable[8]  * clipALF(clipValueTable[8],  curr, pImg[-1][+2], pImg[+1][-2]);

      sum += gaussTable[9]  * clipALF(clipValueTable[9],  curr, pImg[-0][-3], pImg[+0][+3]);
      sum += gaussTable[10] * clipALF(clipValueTable[10], curr, pImg[-0][-2], pImg[+0][+2]);
      sum += gaussTable[11] * clipALF(clipValueTable[11], curr, pImg[-0][-1], pImg[+0][+1]);
      // clang-format on

      sum = (sum + (1 << (shift - 1))) >> shift;
      sum = Clip3<int>(-diffTH, +diffTH, sum);
      sum += curr;

      gaussPic[blkDst.y + i][blkDst.x + j] = ClipPel(sum, clpRng);
    }   // width
  }   // height
}

void AdaptiveLoopFilterEcm::copyAndExtendFixedFilterResultsCtu(const FixFiltSetCand fixedFilterSetCandIdx,
                                                               const FixFiltIdx fixedFilterIdx, const Area &blk)
{
  const auto   &fixedFilterResultPic = m_fixFilterResult[COMP_Y][fixedFilterSetCandIdx][fixedFilterIdx];
  auto         &fixedFilterResultCtu = m_fixedFilterResultPerCtu[fixedFilterSetCandIdx][fixedFilterIdx];
  const CPelBuf srcBuf               = fixedFilterResultPic.subBuf(blk.pos(), blk.size());
  PelBuf        dstBuf               = fixedFilterResultCtu.subBuf(Position(0, 0), blk.size());
  dstBuf.copyFrom(srcBuf);
  dstBuf.addMirrorExtension(ALF_PADDING_SIZE_FIXED_RESULTS);
}

void AdaptiveLoopFilterEcm::copyAndExtendGaussResultsCtu(const Area &blkDst)
{
  const CPelBuf srcBuf = m_gaussPic.subBuf(blkDst.pos(), blkDst.size());
  PelBuf        dstBuf = m_gaussCtu.subBuf(Position(0, 0), blkDst.size());
  dstBuf.copyFrom(srcBuf);
  dstBuf.addMirrorExtension(ALF_PADDING_SIZE_GAUSS_RESULTS);
}

void AdaptiveLoopFilterEcm::copyFixedFilterResults(PelUnitBuf &recDst, const Area &blkDst, const CompID compId,
                                                   const FixFiltSetCand fixedFilterSetCandIdx,
                                                   const FixFiltIdx     fixedFilterIdx)
{
  const CPelBuf srcBuf =
    m_fixFilterResult[compId][fixedFilterSetCandIdx][fixedFilterIdx].subBuf(blkDst.pos(), blkDst.size());
  PelBuf dstBuf = recDst.get(compId).subBuf(blkDst.pos(), blkDst.size());
  dstBuf.copyFrom(srcBuf);
}

void AdaptiveLoopFilterEcm::extendFixedFilterResultsPic(const CompID compId, const FixFiltSetCand fixedFilterSetCandIdx,
                                                        const FixFiltIdx fixedFilterIdx)
{
  auto &fixedFilterResult = m_fixFilterResult[compId][fixedFilterSetCandIdx][fixedFilterIdx];
  CHECKD(fixedFilterResult !=
           (isLuma(compId) ? Size(m_picWidth, m_picHeight) : Size(m_picWidthChroma, m_picHeightChroma)),
         "Unexpected size of fixed filter results buffer.");
  fixedFilterResult.addMirrorExtension(ALF_PADDING_SIZE_FIXED_RESULTS);
}

void AdaptiveLoopFilterEcm::extendGaussResultsPic()
{
  CHECKD(m_gaussPic != Size(m_picWidth, m_picHeight), "Unexpected size of gaussian fixed filter results buffer.");
  m_gaussPic.addMirrorExtension(ALF_PADDING_SIZE_GAUSS_RESULTS);
}

void AdaptiveLoopFilterEcm::extendBlockBoundaries(CompBuf &buf, const AdaptiveLoopFilterEcm::PaddingAreas &padAreas)
{
  const NeighborArea &padAreaL = padAreas[static_cast<unsigned>(BndryLocMain::L)];
  const NeighborArea &padAreaR = padAreas[static_cast<unsigned>(BndryLocMain::R)];
  const NeighborArea &padAreaT = padAreas[static_cast<unsigned>(BndryLocMain::T)];
  const NeighborArea &padAreaB = padAreas[static_cast<unsigned>(BndryLocMain::B)];

  if (padAreaL.m_valid)
  {
    // Add mirror padding on the left.
    const int xDstEnd = padAreaL.x + padAreaL.width - 1;
    for (int y = 0; y < padAreaL.height; ++y)
    {
      for (int x = 0; x < padAreaL.width; ++x)
      {
        buf[padAreaL.y + y][xDstEnd - x] = buf[padAreaL.y + y][xDstEnd + x + 1];
      }
    }
  }

  if (padAreaR.m_valid)
  {
    // Add mirror padding on the right.
    for (int y = 0; y < padAreaR.height; ++y)
    {
      for (int x = 0; x < padAreaR.width; ++x)
      {
        buf[padAreaR.y + y][padAreaR.x + x] = buf[padAreaR.y + y][padAreaR.x - x - 1];
      }
    }
  }

  if (padAreaT.m_valid)
  {
    // Add mirror padding at the top.
    const int yDstEnd = padAreaT.y + padAreaT.height - 1;
    for (int y = 0; y < padAreaT.height; ++y)
    {
      std::copy_n(&buf[yDstEnd + y + 1][padAreaT.x], padAreaT.width, &buf[yDstEnd - y][padAreaT.x]);
    }
  }

  if (padAreaB.m_valid)
  {
    // Add mirror padding at the bottom.
    for (int y = 0; y < padAreaB.height; ++y)
    {
      std::copy_n(&buf[padAreaB.y - y - 1][padAreaB.x], padAreaB.width, &buf[padAreaB.y + y][padAreaB.x]);
    }
  }
}

AdaptiveLoopFilterEcm::NeighborCalcAndPadAreas
  AdaptiveLoopFilterEcm::calcCtuBoundaryCalcAndPadAreas(const Area &ctuArea, const unsigned requiredSize,
                                                        const unsigned desiredWidth, const unsigned desiredHeight,
                                                        const unsigned maxPadSize, const bool luma) const
{
  const auto getCalcAndPadSize = [requiredSize, maxPadSize](const unsigned desiredSize,
                                                            const unsigned marginSize) -> std::tuple<unsigned, unsigned>
  {
    if (marginSize >= desiredSize)
    {
      return std::tuple<unsigned, unsigned>(desiredSize, 0);
    }
    else if (marginSize >= requiredSize)
    {
      return std::tuple<unsigned, unsigned>(requiredSize, 0);
    }
    else
    {
      return std::tuple<unsigned, unsigned>(marginSize, std::min(requiredSize - marginSize, maxPadSize));
    }
  };

  NeighborCalcAndPadAreas areas;

  CalculationArea &calcAreaL  = areas.m_calc[static_cast<unsigned>(BndryLoc::L)];
  CalculationArea &calcAreaR  = areas.m_calc[static_cast<unsigned>(BndryLoc::R)];
  CalculationArea &calcAreaT  = areas.m_calc[static_cast<unsigned>(BndryLoc::T)];
  CalculationArea &calcAreaB  = areas.m_calc[static_cast<unsigned>(BndryLoc::B)];
  CalculationArea &calcAreaTL = areas.m_calc[static_cast<unsigned>(BndryLoc::TL)];
  CalculationArea &calcAreaTR = areas.m_calc[static_cast<unsigned>(BndryLoc::TR)];
  CalculationArea &calcAreaBL = areas.m_calc[static_cast<unsigned>(BndryLoc::BL)];
  CalculationArea &calcAreaBR = areas.m_calc[static_cast<unsigned>(BndryLoc::BR)];

  NeighborArea &padAreaL = areas.m_pad[static_cast<unsigned>(BndryLocMain::L)];
  NeighborArea &padAreaR = areas.m_pad[static_cast<unsigned>(BndryLocMain::R)];
  NeighborArea &padAreaT = areas.m_pad[static_cast<unsigned>(BndryLocMain::T)];
  NeighborArea &padAreaB = areas.m_pad[static_cast<unsigned>(BndryLocMain::B)];

  // Left
  const auto [leftCalcWidth, leftPadWidth] = getCalcAndPadSize(desiredWidth, ctuArea.x);
  if (leftCalcWidth > 0)
  {
    calcAreaL.m_valid        = true;
    calcAreaL.m_ctuIdxOffset = -1;
    calcAreaL                = Area(ctuArea.x - leftCalcWidth, ctuArea.y, leftCalcWidth, ctuArea.height);
  }
  else
  {
    calcAreaL.m_valid = false;
  }

  // Right
  const auto [rightCalcWidth, rightPadWidth] =
    getCalcAndPadSize(desiredWidth, (luma ? m_picWidth : m_picWidthChroma) - (ctuArea.x + ctuArea.width));
  if (rightCalcWidth > 0)
  {
    calcAreaR.m_valid        = true;
    calcAreaR.m_ctuIdxOffset = +1;
    calcAreaR                = Area(ctuArea.x + ctuArea.width, ctuArea.y, rightCalcWidth, ctuArea.height);
  }
  else
  {
    calcAreaR.m_valid = false;
  }

  // Top
  const auto [topCalcHeight, topPadHeight] = getCalcAndPadSize(desiredHeight, ctuArea.y);
  if (topCalcHeight > 0)
  {
    calcAreaT.m_valid        = true;
    calcAreaT.m_ctuIdxOffset = -m_numCTUsInWidth;
    calcAreaT                = Area(ctuArea.x, ctuArea.y - topCalcHeight, ctuArea.width, topCalcHeight);
  }
  else
  {
    calcAreaT.m_valid = false;
  }

  // Bottom
  const auto [bottomCalcHeight, bottomPadHeight] =
    getCalcAndPadSize(desiredHeight, (luma ? m_picHeight : m_picHeightChroma) - (ctuArea.y + ctuArea.height));
  if (bottomCalcHeight > 0)
  {
    calcAreaB.m_valid        = true;
    calcAreaB.m_ctuIdxOffset = +m_numCTUsInWidth;
    calcAreaB                = Area(ctuArea.x, ctuArea.y + ctuArea.height, ctuArea.width, bottomCalcHeight);
  }
  else
  {
    calcAreaB.m_valid = false;
  }

  // Top-left
  if (leftCalcWidth > 0 && topCalcHeight > 0)
  {
    calcAreaTL.m_valid        = true;
    calcAreaTL.m_ctuIdxOffset = -m_numCTUsInWidth - 1;
    calcAreaTL                = Area(calcAreaL.x, calcAreaT.y, calcAreaL.width, calcAreaT.height);
  }
  else
  {
    calcAreaTL.m_valid = false;
  }

  // Top-right
  if (rightCalcWidth > 0 && topCalcHeight > 0)
  {
    calcAreaTR.m_valid        = true;
    calcAreaTR.m_ctuIdxOffset = -m_numCTUsInWidth + 1;
    calcAreaTR                = Area(calcAreaR.x, calcAreaT.y, calcAreaR.width, calcAreaT.height);
  }
  else
  {
    calcAreaTR.m_valid = false;
  }

  // Bottom-left
  if (leftCalcWidth > 0 && bottomCalcHeight > 0)
  {
    calcAreaBL.m_valid        = true;
    calcAreaBL.m_ctuIdxOffset = +m_numCTUsInWidth - 1;
    calcAreaBL                = Area(calcAreaL.x, calcAreaB.y, calcAreaL.width, calcAreaB.height);
  }
  else
  {
    calcAreaBL.m_valid = false;
  }

  // Bottom-right
  if (rightCalcWidth > 0 && bottomCalcHeight > 0)
  {
    calcAreaBR.m_valid        = true;
    calcAreaBR.m_ctuIdxOffset = +m_numCTUsInWidth + 1;
    calcAreaBR                = Area(calcAreaR.x, calcAreaB.y, calcAreaR.width, calcAreaB.height);
  }
  else
  {
    calcAreaBR.m_valid = false;
  }

  // Left padding
  if (leftPadWidth > 0)
  {
    padAreaL.m_valid = true;
    padAreaL         = Area(ctuArea.x - leftCalcWidth - leftPadWidth, ctuArea.y - topCalcHeight, leftPadWidth,
                            ctuArea.height + topCalcHeight + bottomCalcHeight);
  }
  else
  {
    padAreaL.m_valid = false;
  }

  // Right padding
  if (rightPadWidth > 0)
  {
    padAreaR.m_valid = true;
    padAreaR         = Area(ctuArea.x + ctuArea.width + rightCalcWidth, ctuArea.y - topCalcHeight, rightPadWidth,
                            ctuArea.height + topCalcHeight + bottomCalcHeight);
  }
  else
  {
    padAreaR.m_valid = false;
  }

  // Top padding
  if (topPadHeight > 0)
  {
    padAreaT.m_valid = true;
    padAreaT         = Area(ctuArea.x - leftCalcWidth - leftPadWidth, ctuArea.y - topCalcHeight - topPadHeight,
                            ctuArea.width + leftCalcWidth + rightCalcWidth + leftPadWidth + rightPadWidth, topPadHeight);
  }
  else
  {
    padAreaT.m_valid = false;
  }

  // Bottom padding
  if (bottomPadHeight > 0)
  {
    padAreaB.m_valid = true;
    padAreaB         = Area(ctuArea.x - leftCalcWidth - leftPadWidth, ctuArea.y + ctuArea.height + bottomCalcHeight,
                            ctuArea.width + leftCalcWidth + rightCalcWidth + leftPadWidth + rightPadWidth, bottomPadHeight);
  }
  else
  {
    padAreaB.m_valid = false;
  }

  return areas;
}
