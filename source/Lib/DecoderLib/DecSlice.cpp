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

/** \file     DecSlice.cpp
    \brief    slice decoder class
*/

#include "DecSlice.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_next.h"

#include <vector>

//! \ingroup DecoderLib
//! \{

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

DecSlice::DecSlice() {}

DecSlice::~DecSlice() {}

void DecSlice::create(int width, int iMaxCUWidth)
{
  int numBinBuffers = width / iMaxCUWidth + 1;

  for (int i = 0; i < numBinBuffers; i++)
  {
    m_binVectors.push_back(BinStoreVector());
    m_binVectors[i].reserve(CABAC_SPATIAL_MAX_BINS);
  }
}

void DecSlice::destroy() { m_binVectors.clear(); }

void DecSlice::init(CABACDecoder *cabacDecoder, DecCu *pcCuDecoder)
{
  m_CABACDecoder = cabacDecoder;
  m_pcCuDecoder  = pcCuDecoder;
}

void DecSlice::decompressSlice(Slice *slice, InputBitstream *bitstream, int debugCTU)
{
  //-- For time output for each slice
  slice->startProcessingTimer();

  const SPS   *sps         = slice->m_sps;
  Picture     *pic         = slice->m_pic;
  CABACReader &cabacReader = *m_CABACDecoder->getCABACReader(BpmType::STD);
  cabacReader.m_CABACDataStore->updateBufferState(slice);

  // setup coding structure
  CodingStructure &cs = *pic->m_cs;
  cs.slice            = slice;
  cs.sps              = sps;
  cs.pps              = slice->m_pps;
  memcpy(cs.alfApss, slice->m_alfApss, sizeof(cs.alfApss));

  cs.lmcsAps        = slice->m_picHeader->m_lmcsAps;
  cs.scalinglistAps = slice->m_picHeader->m_scalingListAps;

  cs.pcv         = slice->m_pps->pcv;
  cs.chromaQpAdj = 0;

  cs.picture->resizeSAO(cs.pcv->sizeInCtus, 0);
  cs.picture->resizeBIF((CompID)0, cs.pcv->sizeInCtus);
  cs.picture->resizeBIF((CompID)1, cs.pcv->sizeInCtus);
  cs.picture->resizeBIF((CompID)2, cs.pcv->sizeInCtus);

  cs.resetPrevPLT(cs.prevPLT);

  if (slice->getFirstCtuRsAddrInSlice() == 0)
  {
    cs.picture->resizeAlfData(cs.pcv->sizeInCtus);
  }

  slice->lfCccmClearControlInformation();

  const unsigned numSubstreams = (unsigned)slice->m_substreamSizes.size() + 1;

  // init each couple {EntropyDecoder, Substream}
  // Table of extracted substreams.
  std::vector<InputBitstream *> ppcSubstreams(numSubstreams);
  for (unsigned idx = 0; idx < numSubstreams; idx++)
  {
    ppcSubstreams[idx] = bitstream->extractSubstream(idx + 1 < numSubstreams ? (slice->m_substreamSizes[idx] << 3)
                                                                             : bitstream->getNumBitsLeft());
  }

  const unsigned widthInCtus       = cs.pcv->widthInCtus;
  const bool     wavefrontsEnabled = cs.sps->m_entropyCodingSyncEnabledFlag;
  const bool     entryPointPresent = cs.sps->m_entryPointPresentFlag;

  cabacReader.initBitstream(ppcSubstreams[0]);
  cabacReader.initCtxModels(*slice);

  // Quantization parameter
  pic->m_prevQP.fill(slice->m_iSliceQp);
  CHECK(pic->m_prevQP[ChannelType::LUMA] == std::numeric_limits<int>::max(), "Invalid previous QP");

  DTRACE(g_trace_ctx, D_HEADER, "=========== POC: %d ===========\n", slice->m_poc);

  if (slice->m_eSliceType != I_SLICE && slice->getRefPic(RPL0, 0)->m_subPictures.size() > 1)
  {
    clipMv = clipMvInSubpic;
  }
  else
  {
    clipMv = clipMvInPic;
  }

  cs.ccpLut.lutCCP.resize(0);
  cs.eipLut.lutEip.resize(0);

  // for every CTU in the slice segment...
  static Ctx storedCtx;
  unsigned   subStrmId = 0;
  for (unsigned ctuIdx = 0; ctuIdx < slice->getNumCtuInSlice(); ctuIdx++)
  {
    const unsigned ctuRsAddr      = slice->getCtuAddrInSlice(ctuIdx);
    const unsigned ctuXPosInCtus  = ctuRsAddr % widthInCtus;
    const unsigned ctuYPosInCtus  = ctuRsAddr / widthInCtus;
    const unsigned tileColIdx     = slice->m_pps->ctuToTileCol(ctuXPosInCtus);
    const unsigned tileRowIdx     = slice->m_pps->ctuToTileRow(ctuYPosInCtus);
    const unsigned tileXPosInCtus = slice->m_pps->getTileColumnBd(tileColIdx);
    const unsigned tileYPosInCtus = slice->m_pps->getTileRowBd(tileRowIdx);
    const unsigned tileColWidth   = slice->m_pps->getTileColumnWidth(tileColIdx);
    const unsigned tileRowHeight  = slice->m_pps->getTileRowHeight(tileRowIdx);
    const unsigned tileIdx        = slice->m_pps->getTileIdx(ctuXPosInCtus, ctuYPosInCtus);
    const unsigned maxCUSize      = sps->m_maxCuWidth;
    Position       pos(ctuXPosInCtus * maxCUSize, ctuYPosInCtus * maxCUSize);
    UnitArea       ctuArea(cs.area.chromaFormat, Area(pos.x, pos.y, maxCUSize, maxCUSize));
    const SubPic  &curSubPic = slice->m_pps->getSubPicFromPos(pos);

    slice->lfCccmClearControlInformation(ctuRsAddr);

    // padding/restore at slice level
    if (slice->m_pps->m_numSubPics >= 2 && curSubPic.m_treatedAsPicFlag && ctuIdx == 0)
    {
      int subPicX      = (int)curSubPic.m_subPicLeft;
      int subPicY      = (int)curSubPic.m_subPicTop;
      int subPicWidth  = (int)curSubPic.m_subPicWidthInLumaSample;
      int subPicHeight = (int)curSubPic.m_subPicHeightInLumaSample;
      for (int rlist = RPL0; rlist < NUM_RPL01; rlist++)
      {
        int n = slice->m_numRefIdx[(RefPicList)rlist];
        for (int idx = 0; idx < n; idx++)
        {
          Picture *refPic = slice->getRefPic((RefPicList)rlist, idx);

          if (!refPic->m_isSubPicBorderSaved && refPic->m_subPictures.size() > 1)
          {
            refPic->saveSubPicBorder(refPic->m_poc, subPicX, subPicY, subPicWidth, subPicHeight);
            refPic->extendSubPicBorder(refPic->m_poc, subPicX, subPicY, subPicWidth, subPicHeight);
            refPic->m_isSubPicBorderSaved = true;
          }
        }
      }
    }

    DTRACE_UPDATE(g_trace_ctx, std::make_pair("ctu", ctuRsAddr));

    cabacReader.initBitstream(ppcSubstreams[subStrmId]);

    // set up CABAC contexts' state for this CTU
    if (ctuXPosInCtus == tileXPosInCtus && ctuYPosInCtus == tileYPosInCtus)
    {
      if (ctuIdx != 0) // if it is the first CTU, then the entropy coder has already been reset
      {
        cabacReader.initCtxModels(*slice);
        cs.resetPrevPLT(cs.prevPLT);
      }
      pic->m_prevQP.fill(slice->m_iSliceQp);
    }
    else if (ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled)
    {
      // Synchronize cabac probabilities with top CTU if it's available and at the start of a line.
      if (ctuIdx != 0) // if it is the first CTU, then the entropy coder has already been reset
      {
        cabacReader.initCtxModels(*slice);
        cs.resetPrevPLT(cs.prevPLT);
      }
      if (cs.getCURestricted(pos.offset(0, -1), pos, slice->m_independentSliceIdx, tileIdx, ChannelType::LUMA))
      {
        // Top is available, so use it.
        cabacReader.getCtx() = m_entropyCodingSyncContextState;
        cabacReader.getCtx().riceStatReset(slice->m_sps->m_bitDepths[ChannelType::LUMA],
                                           slice->m_sps->m_spsRangeExtension.m_persistentRiceAdaptationEnabledFlag);
        cs.setPrevPLT(m_palettePredictorSyncState);
      }
      pic->m_prevQP.fill(slice->m_iSliceQp);
    }

    bool updateBcwCodingOrder = cs.slice->m_eSliceType == B_SLICE && ctuIdx == 0;
    if (updateBcwCodingOrder)
    {
      resetBcwCodingOrder(true, cs);
    }

    if ((cs.slice->m_eSliceType != I_SLICE || cs.slice->m_ibcFlag) && ctuXPosInCtus == tileXPosInCtus)
    {
      cs.motionLut.lut.resize(0);
      cs.motionLut.lutIbc.resize(0);
      for (int i = 0; i < MAX_NUM_AFFHMVP_ENTRIES; i++)
      {
        cs.motionLut.lutAff[i].resize(0);
      }
      cs.motionLut.lutAffInherit.resize(0);
    }

    if (ctuXPosInCtus == tileXPosInCtus)
    {
      cs.ccpLut.lutCCP.resize(0);
      cs.eipLut.lutEip.resize(0);
    }

    if (!cs.slice->isIntra())
    {
      pic->m_mctsInfo.init(&cs, getCtuAddr(ctuArea.lumaPos(), *(cs.pcv)));
    }

    if (ctuRsAddr == debugCTU)
    {
      break;
    }
    if (ctuRsAddr == 0)
    {
      if (cs.pps->m_BIF)
      {
        cabacReader.bif(COMP_Y, cs);
      }
      if (cs.pps->m_chromaBIF)
      {
        cabacReader.bif(COMP_Cb, cs);
        cabacReader.bif(COMP_Cr, cs);
      }
    }

    if (ctuYPosInCtus) // update the CABAC states based on CTU above
    {
      cabacReader.updateCtxs(getBinVector(ctuXPosInCtus));
    }

    // Clear the bin counters and prepare for collecting new data for this CTU
    cabacReader.setBinBuffer(getBinVector(ctuXPosInCtus));

    cabacReader.coding_tree_unit(cs, ctuArea, pic->m_prevQP, ctuRsAddr);

    cabacReader.setBinBuffer(nullptr); // done with data collection for this CTU

    m_pcCuDecoder->decompressCtu(cs, ctuArea);

    if (storeContexts(slice, ctuXPosInCtus, ctuYPosInCtus)) // store CABAC context to be used in next frames
    {
      storedCtx = cabacReader.getCtx();
    }

    if (ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled)
    {
      m_entropyCodingSyncContextState = cabacReader.getCtx();
      cs.storePrevPLT(m_palettePredictorSyncState);
    }

    if (ctuIdx == slice->getNumCtuInSlice() - 1)
    {
      unsigned binVal = cabacReader.terminating_bit();
      CHECK(!binVal, "Expecting a terminating bit");
#if DECODER_CHECK_SUBSTREAM_AND_SLICE_TRAILING_BYTES
      cabacReader.remaining_bytes(false);
#endif
    }
    else if ((ctuXPosInCtus + 1 == tileXPosInCtus + tileColWidth) &&
             (ctuYPosInCtus + 1 == tileYPosInCtus + tileRowHeight || wavefrontsEnabled))
    {
      // The sub-stream/stream should be terminated after this CTU.
      // (end of slice-segment, end of tile, end of wavefront-CTU-row)
      unsigned binVal = cabacReader.terminating_bit();
      CHECK(!binVal, "Expecting a terminating bit");
      if (entryPointPresent)
      {
#if DECODER_CHECK_SUBSTREAM_AND_SLICE_TRAILING_BYTES
        cabacReader.remaining_bytes(true);
#endif
        subStrmId++;
      }
    }
    if (slice->m_pps->m_numSubPics >= 2 && curSubPic.m_treatedAsPicFlag && ctuIdx == (slice->getNumCtuInSlice() - 1))
    // for last Ctu in the slice
    {
      int subPicX      = (int)curSubPic.m_subPicLeft;
      int subPicY      = (int)curSubPic.m_subPicTop;
      int subPicWidth  = (int)curSubPic.m_subPicWidthInLumaSample;
      int subPicHeight = (int)curSubPic.m_subPicHeightInLumaSample;
      for (int rlist = RPL0; rlist < NUM_RPL01; rlist++)
      {
        int n = slice->m_numRefIdx[(RefPicList)rlist];
        for (int idx = 0; idx < n; idx++)
        {
          Picture *refPic = slice->getRefPic((RefPicList)rlist, idx);
          if (refPic->m_isSubPicBorderSaved)
          {
            refPic->restoreSubPicBorder(refPic->m_poc, subPicX, subPicY, subPicWidth, subPicHeight);
            refPic->m_isSubPicBorderSaved = false;
          }
        }
      }
    }
  }

  if (slice->m_pps->pcv->sizeInCtus - 1 ==
      slice->getCtuAddrInSlice(
        slice->getNumCtuInSlice() -
        1)) // store CABAC context to be used in next frames when the last CTU in a picture is processed
  {
    cabacReader.m_CABACDataStore->storeCtxStates(slice, storedCtx);
  }

  // deallocate all created substreams, including internal buffers.
  for (auto substr: ppcSubstreams)
  {
    delete substr;
  }
  slice->stopProcessingTimer();
}

//! \}
