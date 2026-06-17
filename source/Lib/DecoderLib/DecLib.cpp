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

/** \file     DecLib.cpp
    \brief    decoder class
*/

#include "NALread.h"
#include "DecLib.h"

#include "CommonLib/AdaptiveLoopFilterEcm.h"
#include "CommonLib/AdaptiveLoopFilterVtm.h"
#include "CommonLib/AlfParameters.h"
#include "CommonLib/dtrace_next.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/Buffer.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/ProfileTierLevel.h"
#if ENABLE_NNLF
#include "CommonLib/NNFilterUnified.h"
#endif

#include <fstream>
#include <set>
#include <stdio.h>
#include <fcntl.h>
#include "AnnexBread.h"
#include "NALread.h"
#if K0149_BLOCK_STATISTICS
#include "CommonLib/dtrace_blockstatistics.h"
#endif

#if RExt__DECODER_DEBUG_TOOL_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif

#if ENABLE_CABAC_DUMP
namespace CabacRetrain
{
extern void endFrame(int poc, int qp, bool switchBp, SliceType st);
}
#endif

bool tryDecodePicture(Picture *pcEncPic, const int expectedPoc, const std::string &bitstreamFileName,
#if ENABLE_NNLF
                      std::string nnlfModelName, int nnlfDebugOption,
#endif
                      EnumArray<ParameterSetMap<APS>, ApsType> *apsMap, bool bDecodeUntilPocFound, int debugCTU,
                      int debugPOC)
{
  int      poc;
  PicList *picList = nullptr;

  static bool bFirstCall                   = true;             /* TODO: MT */
  static bool loopFiltered[MAX_VPS_LAYERS] = { false };            /* TODO: MT */
  static int  iPOCLastDisplay              = -MAX_INT;         /* TODO: MT */

  static std::ifstream   *bitstreamFile = nullptr;  /* TODO: MT */
  static InputByteStream *bytestream    = nullptr;  /* TODO: MT */
  bool                    bRet          = false;

  // create & initialize internal classes
  static DecLib *pcDecLib = nullptr;              /* TODO: MT */

  if (pcEncPic)
  {
    if (bFirstCall)
    {
      bitstreamFile = new std::ifstream(bitstreamFileName.c_str(), std::ifstream::in | std::ifstream::binary);
      bytestream    = new InputByteStream(*bitstreamFile);

      CHECK(!*bitstreamFile, "failed to open bitstream file " << bitstreamFileName.c_str() << " for reading");
      // create decoder class
      pcDecLib = new DecLib;
      pcDecLib->create();

      // initialize decoder class
      pcDecLib->init(
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
        ""
#endif
      );

#if ENABLE_NNLF
      pcDecLib->m_decCfg.m_nnlfModelName   = nnlfModelName;
      pcDecLib->m_decCfg.m_nnlfDebugOption = nnlfDebugOption;
#endif

      pcDecLib->setDebugCTU(debugCTU);
      pcDecLib->setDebugPOC(debugPOC);
      pcDecLib->setDecodedPictureHashSEIEnabled(true);
      pcDecLib->setAPSMapEnc(apsMap);

      bFirstCall = false;
      msg(INFO, "start to decode %s \n", bitstreamFileName.c_str());
    }

    bool goOn = true;

    // main decoder loop
    while (!!*bitstreamFile && goOn)
    {
      InputNALUnit nalu;
      nalu.m_nalUnitType = NAL_UNIT_INVALID;

      // determine if next NAL unit will be the first one from a new picture
      bool bNewPicture    = pcDecLib->isNewPicture(bitstreamFile, bytestream);
      bool bNewAccessUnit = bNewPicture && pcDecLib->isNewAccessUnit(bNewPicture, bitstreamFile, bytestream);
      bNewPicture         = bNewPicture && bNewAccessUnit;

      if (!bNewPicture)
      {
        AnnexBStats stats = AnnexBStats();
        byteStreamNALUnit(*bytestream, nalu.getBitstream().getFifo(), stats);

        // call actual decoding function
        if (nalu.getBitstream().getFifo().empty())
        {
          /* this can happen if the following occur:
           *  - empty input file
           *  - two back-to-back start_code_prefixes
           *  - start_code_prefix immediately followed by EOF
           */
          msg(ERROR, "Warning: Attempt to decode an empty NAL unit\n");
        }
        else
        {
          read(nalu);
          int iSkipFrame = 0;

          pcDecLib->decode(nalu, iSkipFrame, iPOCLastDisplay, 0);
        }
      }

      if ((bNewPicture || !*bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS) &&
          !pcDecLib->getFirstSliceInSequence(nalu.m_nuhLayerId))
      {
        if (!loopFiltered[nalu.m_nuhLayerId] || *bitstreamFile)
        {
          pcDecLib->finishPictureLight(poc, picList);

          if (picList)
          {
            for (auto &pic: *picList)
            {
              if (pic->m_poc == poc && (!bDecodeUntilPocFound || expectedPoc == poc))
              {
                CHECK(pcEncPic->m_slices.size() == 0, "at least one slice should be available");

                CHECK(expectedPoc != poc, "mismatch in POC - check encoder configuration");

                if (debugCTU < 0 || poc != debugPOC)
                {
                  PicHeader *pcEncPicHeader = const_cast<PicHeader *>(pcEncPic->m_slices[0]->m_picHeader);
                  for (int i = 0; i < pic->m_slices.size(); i++)
                  {
                    if (pcEncPic->m_slices.size() <= i)
                    {
                      pcEncPic->m_slices.push_back(new Slice);
                      pcEncPic->m_slices.back()->initSlice();
                      pcEncPic->m_slices.back()->m_pps = pcEncPic->m_slices[0]->m_pps;
                      pcEncPic->m_slices.back()->m_sps = pcEncPic->m_slices[0]->m_sps;
                      pcEncPic->m_slices.back()->m_vps = pcEncPic->m_slices[0]->m_vps;
                      pcEncPic->m_slices.back()->m_pic = pcEncPic->m_slices[0]->m_pic;
                    }
                    pcEncPic->m_slices[i]->copySliceInfo(pic->m_slices[i], false);
                    pcEncPic->m_slices[i]->m_picHeader = pcEncPicHeader;
                  }
                  auto cpySplitSize = [](int which, PicHeader *dstPH, const PicHeader *srcPH)
                  {
                    dstPH->m_splitConsOverrideFlag       = srcPH->m_splitConsOverrideFlag;
                    dstPH->m_minQT[which]                = srcPH->m_minQT[which];
                    dstPH->m_maxMTTHierarchyDepth[which] = srcPH->m_maxMTTHierarchyDepth[which];
                    dstPH->m_maxBTSize[which]            = srcPH->m_maxBTSize[which];
                    dstPH->m_maxTTSize[which]            = srcPH->m_maxTTSize[which];
                  };
                  if (pic->m_slices[0]->m_picHeader->m_picIntraSliceAllowedFlag)
                  {
                    cpySplitSize(0, pcEncPicHeader, pic->m_slices[0]->m_picHeader);
                    if (pic->m_cs->sps->m_dualITree)
                    {
                      cpySplitSize(2, pcEncPicHeader, pic->m_slices[0]->m_picHeader);
                    }
                  }
                  if (pic->m_slices[0]->m_picHeader->m_picInterSliceAllowedFlag)
                  {
                    cpySplitSize(1, pcEncPicHeader, pic->m_slices[0]->m_picHeader);
                  }
                }

                pcEncPic->m_cs->slice = pcEncPic->m_slices.back();
                pcEncPic->calcLumaClpParams();

                if (debugCTU >= 0 && poc == debugPOC)
                {
                  pcEncPic->m_cs->initStructData();

                  pcEncPic->m_cs->copyStructure(*pic->m_cs, ChannelType::LUMA, true, true);

                  if (CS::isDualITree(*pcEncPic->m_cs))
                  {
                    pcEncPic->m_cs->copyStructure(*pic->m_cs, ChannelType::CHROMA, true, true);
                  }

                  for (auto &cu: pcEncPic->m_cs->cus)
                  {
                    cu->slice = pcEncPic->m_cs->slice;
                  }
                }
                else
                {
#if ENABLE_NNLF
                  if (pic->m_cs->sps->m_nnlf)
                  {
                    for (int i = 0; i < pic->m_slices.size(); ++i)
                    {
                      pcEncPic->m_slices[i]->m_nnlfUnifiedParam = pic->m_slices[i]->m_nnlfUnifiedParam;
                    }
                    pcEncPic->m_picprm      = pic->m_picprm;
                    pcEncPic->m_picprm.sprm = pcEncPic->m_slices[0]->m_nnlfUnifiedParam;
                  }
#endif
                  if (pic->m_cs->sps->m_saoEnabledFlag || pic->m_cs->pps->m_BIF || pic->m_cs->pps->m_chromaBIF)
                  {
                    pcEncPic->copySAO(*pic, 0);
                    pcEncPic->copyBIF(*pic);
                  }
                  if (pic->m_cs->sps->m_ccSaoEnabledFlag)
                  {
                    for (int i = 0; i < pic->m_slices.size(); i++)
                    {
                      pcEncPic->m_slices[i]->m_ccSaoEnabledFlag[COMP_Y] = pic->m_slices[i]->m_ccSaoEnabledFlag[COMP_Y];
                      pcEncPic->m_slices[i]->m_ccSaoEnabledFlag[COMP_Cb] =
                        pic->m_slices[i]->m_ccSaoEnabledFlag[COMP_Cb];
                      pcEncPic->m_slices[i]->m_ccSaoEnabledFlag[COMP_Cr] =
                        pic->m_slices[i]->m_ccSaoEnabledFlag[COMP_Cr];
                    }
                  }
                  if (pic->m_cs->sps->m_alfEnabledFlag)
                  {
                    pcEncPic->copyAlfData(*pic);

                    for (int i = 0; i < pic->m_slices.size(); i++)
                    {
                      pcEncPic->m_slices[i]->m_numAlfApsIdsLuma = pic->m_slices[i]->m_numAlfApsIdsLuma;
                      pcEncPic->m_slices[i]->m_alfApsIdsLuma    = pic->m_slices[i]->m_alfApsIdsLuma;
                      pcEncPic->m_slices[i]->setAlfAPSs(pic->m_slices[i]->m_alfApss);
                      pcEncPic->m_slices[i]->m_alfApsIdChroma = pic->m_slices[i]->m_alfApsIdChroma;
                      if (pic->m_cs->sps->m_alfImprovementsEnabledFlag)
                      {
                        memcpy(pcEncPic->m_slices[i]->m_newAlfFixFiltSetCandIdx,
                               pic->m_slices[i]->m_newAlfFixFiltSetCandIdx,
                               sizeof(pcEncPic->m_slices[i]->m_newAlfFixFiltSetCandIdx));
                      }
                      else
                      {
                        std::fill_n(pcEncPic->m_slices[i]->m_newAlfFixFiltSetCandIdx, MAX_NUM_COMP, -1);
                      }
                      pcEncPic->m_slices[i]->m_alfEnabledFlag[COMP_Y]  = pic->m_slices[i]->m_alfEnabledFlag[COMP_Y];
                      pcEncPic->m_slices[i]->m_alfEnabledFlag[COMP_Cb] = pic->m_slices[i]->m_alfEnabledFlag[COMP_Cb];
                      pcEncPic->m_slices[i]->m_alfEnabledFlag[COMP_Cr] = pic->m_slices[i]->m_alfEnabledFlag[COMP_Cr];
                      pcEncPic->m_slices[i]->m_ccAlfCbApsId            = pic->m_slices[i]->m_ccAlfCbApsId;
                      pcEncPic->m_slices[i]->m_ccAlfCbEnabledFlag      = pic->m_slices[i]->m_ccAlfCbEnabledFlag;
                      pcEncPic->m_slices[i]->m_ccAlfCrApsId            = pic->m_slices[i]->m_ccAlfCrApsId;
                      pcEncPic->m_slices[i]->m_ccAlfCrEnabledFlag      = pic->m_slices[i]->m_ccAlfCrEnabledFlag;
                    }
                  }

                  pcDecLib->executeLoopFilters();
                  pcDecLib->m_pic->copyAdaptedLumaClip();
                  if (pic->m_cs->sps->m_saoEnabledFlag || pic->m_cs->pps->m_BIF || pic->m_cs->pps->m_chromaBIF)
                  {
                    pcEncPic->copySAO(*pic, 1);
                  }

                  pcEncPic->m_cs->copyStructure(*pic->m_cs, ChannelType::LUMA, true, true);

                  if (CS::isDualITree(*pcEncPic->m_cs))
                  {
                    pcEncPic->m_cs->copyStructure(*pic->m_cs, ChannelType::CHROMA, true, true);
                  }
                }
                goOn = false; // exit the loop return
                bRet = true;
                break;
              }
            }
          }
          // postpone loop filters
          if (!bRet)
          {
            pcDecLib->executeLoopFilters();
            pcDecLib->m_pic->copyAdaptedLumaClip();
          }

          pcDecLib->finishPicture(poc, picList, DETAILS);

          // write output
          if (!picList->empty())
          {
            PicList::iterator iterPic                      = picList->begin();
            int               numPicsNotYetDisplayed       = 0;
            int               dpbFullness                  = 0;
            const SPS        *activeSPS                    = (picList->front()->m_cs->sps);
            uint32_t          maxNrSublayers               = activeSPS->m_maxSubLayers;
            uint32_t          maxNumReorderPicsHighestTid  = activeSPS->m_maxNumReorderPics[maxNrSublayers - 1];
            uint32_t          maxDecPicBufferingHighestTid = activeSPS->m_maxDecPicBuffering[maxNrSublayers - 1];
            const VPS        *referredVPS                  = picList->front()->m_cs->vps;

            if (referredVPS != nullptr && referredVPS->m_numLayersInOls[referredVPS->m_targetOlsIdx] > 1)
            {
              maxNumReorderPicsHighestTid  = referredVPS->getMaxNumReorderPics(maxNrSublayers - 1);
              maxDecPicBufferingHighestTid = referredVPS->getMaxDecPicBuffering(maxNrSublayers - 1);
            }

            while (iterPic != picList->end())
            {
              Picture *pcCurPic = *(iterPic);
              if (pcCurPic->m_neededForOutput && pcCurPic->m_poc > iPOCLastDisplay)
              {
                numPicsNotYetDisplayed++;
                dpbFullness++;
              }
              else if (pcCurPic->m_referenced)
              {
                dpbFullness++;
              }
              iterPic++;
            }

            iterPic = picList->begin();

            if (numPicsNotYetDisplayed > 2)
            {
              iterPic++;
            }

            Picture *pcCurPic = *(iterPic);
            if (numPicsNotYetDisplayed > 2 && pcCurPic->m_fieldPic) // Field Decoding
            {
              THROW("no field coding support ");
            }
            else if (!pcCurPic->m_fieldPic) // Frame Decoding
            {
              iterPic = picList->begin();

              while (iterPic != picList->end())
              {
                pcCurPic = *(iterPic);

                if (pcCurPic->m_neededForOutput && pcCurPic->m_poc > iPOCLastDisplay &&
                    (numPicsNotYetDisplayed > maxNumReorderPicsHighestTid ||
                     dpbFullness > maxDecPicBufferingHighestTid))
                {
                  numPicsNotYetDisplayed--;
                  if (!pcCurPic->m_referenced)
                  {
                    dpbFullness--;
                  }
                  // update POC of display order
                  iPOCLastDisplay = pcCurPic->m_poc;

                  // erase non-referenced picture in the reference picture list after display
                  if (!pcCurPic->m_referenced && pcCurPic->m_reconstructed)
                  {
                    pcCurPic->m_reconstructed = false;
                  }
                  pcCurPic->m_neededForOutput = false;
                }

                iterPic++;
              }
            }
          }

          pcDecLib->updateAssociatedIRAP();
          pcDecLib->updatePrevGDRInSameLayer();
          pcDecLib->updatePrevIRAPAndGDRSubpic();
          // LMCS APS will be assigned later in LMCS initialization step
          pcEncPic->m_cs->picHeader->m_lmcsAps   = nullptr;
          pcEncPic->m_cs->picHeader->m_lmcsApsId = -1;
          if (bitstreamFile)
          {
            pcDecLib->resetAccessUnitNals();
            pcDecLib->resetAccessUnitApsNals();
          }
        }
        loopFiltered[nalu.m_nuhLayerId] = (nalu.m_nalUnitType == NAL_UNIT_EOS);
        if (nalu.m_nalUnitType == NAL_UNIT_EOS)
        {
          pcDecLib->setFirstSliceInSequence(true, nalu.m_nuhLayerId);
        }
      }
      else if ((bNewPicture || !*bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS) &&
               pcDecLib->getFirstSliceInSequence(nalu.m_nuhLayerId))
      {
        pcDecLib->setFirstSliceInPicture(true);
      }
    }
  }

  if (!bRet)
  {
    CHECK(bDecodeUntilPocFound,
          " decoding failed - check decodeBitstream2 parameter File: " << bitstreamFileName.c_str());
    if (pcDecLib)
    {
      pcDecLib->destroy();
      pcDecLib->deletePicBuffer();
      delete pcDecLib;
      pcDecLib = nullptr;
    }
    bFirstCall = true;
    for (int i = 0; i < MAX_VPS_LAYERS; i++)
    {
      loopFiltered[i] = false;
    }
    iPOCLastDisplay = -MAX_INT;

    if (bytestream)
    {
      delete bytestream;
      bytestream = nullptr;
    }

    if (bitstreamFile)
    {
      delete bitstreamFile;
      bitstreamFile = nullptr;
    }
  }

  return bRet;
}

//! \ingroup DecoderLib
//! \{

DecLib::DecLib()
  : m_maxRefPicNum(0)
  , m_isFirstGeneralHrd(true)
  , m_prevGeneralHrdParams()
  , m_latestDRAPPOC(MAX_INT)
  , m_latestEDRAPPOC(MAX_INT)
  , m_latestEDRAPIndicationLeadingPicturesDecodableFlag(false)
  , m_associatedIRAPDecodingOrderNumber { 0 }
  , m_decodingOrderCounter(0)
  , m_puCounter(0)
  , m_seiInclusionFlag(false)
  , m_pocRandomAccess(MAX_INT)
  , m_lastRasPoc(MAX_INT)
  , m_cListPic()
  , m_parameterSetManager()
  , m_apcSlicePilot(nullptr)
  , m_SEIs()
  , m_sdiSEIInFirstAU(nullptr)
  , m_maiSEIInFirstAU(nullptr)
  , m_mvpSEIInFirstAU(nullptr)
  , m_cIntraPred()
  , m_cInterPred()
  , m_cTrQuant()
  , m_cSliceDecoder()
  , m_cTrQuantScalingList()
  , m_cCuDecoder()
  , m_HLSReader()
  , m_seiReader()
  , m_deblockingFilter()
  , m_cSAO()
  , m_alfEcm(nullptr)
  , m_alfVtm(nullptr)
  , m_cReshaper()
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  , m_cacheModel()
#endif
  , m_pic(nullptr)
  , m_prevLayerID(MAX_INT)
  , m_prevPOC(MAX_INT)
  , m_prevPicPOC(MAX_INT)
  , m_prevTid0POC(0)
  , m_isFirstSliceInPicture(true)
  , m_firstPictureInSequence(true)
  , m_grainCharacteristic()
  , m_grainBuf()
  , m_colourTranfParams()
  , m_firstSliceInBitstream(true)
  , m_isFirstAuInCvs(true)
  , m_prevSliceSkipped(false)
  , m_skippedPOC(MAX_INT)
  , m_skippedLayerID(MAX_INT)
  , m_lastPOCNoOutputPriorPics(-1)
  , m_isNoOutputPriorPics(false)
  , m_lastNoOutputBeforeRecoveryFlag { false }
  , m_sliceLmcsApsId(-1)
  , m_pDecodedSEIOutputStream(nullptr)
  , m_audIrapOrGdrAuFlag(false)
  , m_decodedPictureHashSEIEnabled(false)
  , m_numberOfChecksumErrorsDetected(0)
  , m_warningMessageSkipPicture(false)
  , m_prefixSEINALUs()
#if JVET_Z0120_SII_SEI_PROCESSING
  , m_ShutterFilterEnable(false)
#endif
  , m_debugPOC(-1)
  , m_debugCTU(-1)
  , m_opi(nullptr)
  , m_mTidExternalSet(false)
  , m_mTidOpiSet(false)
  , m_tOlsIdxTidExternalSet(false)
  , m_tOlsIdxTidOpiSet(false)
  , m_vps(nullptr)
  , m_maxDecSubPicIdx(0)
  , m_maxDecSliceAddrInSubPic(-1)
  , m_clsVPSid(0)
  , m_targetSubPicIdx(0)
  , m_dci(nullptr)
{
#if ENABLE_SIMD_OPT_BUFFER
  g_pelBufOP.initPelBufOpsX86();
#endif
#if ENABLE_SIMD_TRAFO
  g_tCoeffOps.initTCoeffOpsX86();
#endif
  memset(m_prevEOS, false, sizeof(m_prevEOS));
  memset(m_accessUnitEos, false, sizeof(m_accessUnitEos));
  std::fill_n(m_prevGDRInSameLayerPOC, MAX_VPS_LAYERS, -MAX_INT);
  std::fill_n(m_prevGDRInSameLayerRecoveryPOC, MAX_VPS_LAYERS, -MAX_INT);
  std::fill_n(m_firstSliceInSequence, MAX_VPS_LAYERS, true);
  std::fill_n(m_pocCRA, MAX_VPS_LAYERS, -MAX_INT);
  std::fill_n(m_accessUnitSpsNumSubpic, MAX_VPS_LAYERS, 1);
  for (int i = 0; i < MAX_VPS_LAYERS; i++)
  {
    m_associatedIRAPType[i] = NAL_UNIT_INVALID;
    std::fill_n(m_prevGDRSubpicPOC[i], MAX_NUM_SUB_PICS, -MAX_INT);
    std::fill_n(m_prevIRAPSubpicPOC[i], MAX_NUM_SUB_PICS, -MAX_INT);
    memset(m_prevIRAPSubpicDecOrderNo[i], 0, sizeof(int) * MAX_NUM_SUB_PICS);
    std::fill_n(m_prevIRAPSubpicType[i], MAX_NUM_SUB_PICS, NAL_UNIT_INVALID);
  }
  m_if.initInterpolationFilter(true);
}

DecLib::~DecLib()
{
  resetAccessUnitSeiNalus();
  resetPictureSeiNalus();
  resetPrefixSeiNalus();

  if (m_sdiSEIInFirstAU != nullptr)
  {
    delete m_sdiSEIInFirstAU;
  }
  m_sdiSEIInFirstAU = nullptr;
  if (m_maiSEIInFirstAU != nullptr)
  {
    delete m_maiSEIInFirstAU;
  }
  m_maiSEIInFirstAU = nullptr;
  if (m_mvpSEIInFirstAU != nullptr)
  {
    delete m_mvpSEIInFirstAU;
  }
  m_mvpSEIInFirstAU = nullptr;

  if (m_alfEcm != nullptr)
  {
    delete m_alfEcm;
    m_alfEcm = nullptr;
  }
  if (m_alfVtm != nullptr)
  {
    delete m_alfVtm;
    m_alfVtm = nullptr;
  }
}

void DecLib::create()
{
  m_apcSlicePilot     = new Slice;
  m_uiSliceSegmentIdx = 0;
#if ENABLE_TIME_PROFILING
  if (g_timeProfiler == nullptr)
  {
    g_timeProfiler = new TimeProfiler(P_DECODER);
  }
  msg(INFO, "\n Using runtime profiler\n");
#endif
}

void DecLib::destroy()
{
#if ENABLE_TIME_PROFILING
  if (g_timeProfiler && g_allTimeProfilers.empty())
  {
    g_timeProfiler->output(std::cout);
    delete g_timeProfiler;
  }
#endif
  delete m_apcSlicePilot;
  m_apcSlicePilot = nullptr;

  if (m_dci)
  {
    delete m_dci;
    m_dci = nullptr;
  }

  if (m_opi)
  {
    delete m_opi;
    m_opi = nullptr;
  }

#if ENABLE_NNLF
  if (m_unifiedNnlf)
  {
    m_unifiedNnlf->destroy();
    delete m_unifiedNnlf;
    m_unifiedNnlf = nullptr;
  }
#endif

  if (m_alfEcm != nullptr)
  {
    delete m_alfEcm;
    m_alfEcm = nullptr;
  }
  if (m_alfVtm != nullptr)
  {
    delete m_alfVtm;
    m_alfVtm = nullptr;
  }

  m_cSliceDecoder.destroy();
}

void DecLib::init(
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  const std::string &cacheCfgFileName
#endif
)
{
  m_cSliceDecoder.init(&m_CABACDecoder, &m_cCuDecoder);
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  m_cacheModel.create(cacheCfgFileName);
  m_cacheModel.clear();
  m_cInterPred.cacheAssign(&m_cacheModel);
#endif
  DTRACE_UPDATE(g_trace_ctx, std::make_pair("final", 1));
}

void DecLib::deletePicBuffer()
{
  PicList::iterator iterPic = m_cListPic.begin();
  int               size    = int(m_cListPic.size());

  for (int i = 0; i < size; i++)
  {
    Picture *pic = *(iterPic++);
    pic->destroy();

    delete pic;
    pic = nullptr;
  }
  if (m_alfEcm != nullptr)
  {
    m_alfEcm->destroy();
  }
  if (m_alfVtm != nullptr)
  {
    m_alfVtm->destroy();
  }
  m_cSAO.destroy();
  m_deblockingFilter.destroy();
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  m_cacheModel.reportSequence();
  m_cacheModel.destroy();
#endif
  m_cCuDecoder.destoryDecCuReshaprBuf();
  m_cReshaper.destroy();
}

Picture *DecLib::xGetNewPicBuffer(const SPS &sps, const PPS &pps, const uint32_t temporalLayer, const int layerId)
{
  Picture *pic   = nullptr;
  // getMaxDecPicBuffering() has space for the picture currently being decoded
  m_maxRefPicNum = (m_vps == nullptr || m_vps->m_numLayersInOls[m_vps->m_targetOlsIdx] == 1)
    ? sps.m_maxDecPicBuffering[temporalLayer]
    : m_vps->getMaxDecPicBuffering(temporalLayer);
  if (m_cListPic.size() < (uint32_t)m_maxRefPicNum)
  {
    pic = new Picture();

    pic->create(sps.m_chromaFormatIdc, Size(pps.m_picWidthInLumaSamples, pps.m_picHeightInLumaSamples),
                sps.m_maxCuWidth, sps.m_maxCuWidth + EXT_PICTURE_SIZE, true, layerId, sps.m_rprEnabledFlag, false, false
#if JVET_Z0120_SII_SEI_PROCESSING
                ,
                getShutterFilterFlag()
#endif
#if ENABLE_NNLF
                  ,
                sps.m_nnlfStore
#endif
    );

    m_cListPic.push_back(pic);

    return pic;
  }

  bool bBufferIsAvailable = false;
  for (auto *p: m_cListPic)
  {
    pic = p;  // workaround because range-based for-loops don't work with existing variables
    if (pic->m_reconstructed == false && !pic->m_neededForOutput)
    {
      pic->m_neededForOutput = false;
      bBufferIsAvailable     = true;
      break;
    }

    if (!pic->m_referenced && !pic->m_neededForOutput)
    {
      pic->m_neededForOutput = false;
      pic->m_reconstructed   = false;
      bBufferIsAvailable     = true;
      break;
    }
  }

  if (!bBufferIsAvailable)
  {
    // There is no room for this picture, either because of faulty encoder or dropped NAL. Extend the buffer.
    m_maxRefPicNum++;

    pic = new Picture();

    m_cListPic.push_back(pic);

    pic->create(sps.m_chromaFormatIdc, Size(pps.m_picWidthInLumaSamples, pps.m_picHeightInLumaSamples),
                sps.m_maxCuWidth, sps.m_maxCuWidth + EXT_PICTURE_SIZE, true, layerId, sps.m_rprEnabledFlag, false, false
#if JVET_Z0120_SII_SEI_PROCESSING
                ,
                getShutterFilterFlag()
#endif
#if ENABLE_NNLF
                  ,
                sps.m_nnlfStore
#endif
    );
  }
  else
  {
    if (!pic->Y().Size::operator==(Size(pps.m_picWidthInLumaSamples, pps.m_picHeightInLumaSamples)) ||
        pps.pcv->maxCUWidth != sps.m_maxCuWidth || pps.pcv->maxCUHeight != sps.m_maxCuHeight ||
        pic->m_layerId != layerId)
    {
      pic->destroy();

      pic->create(sps.m_chromaFormatIdc, Size(pps.m_picWidthInLumaSamples, pps.m_picHeightInLumaSamples),
                  sps.m_maxCuWidth, sps.m_maxCuWidth + EXT_PICTURE_SIZE, true, layerId, sps.m_rprEnabledFlag, false,
                  false
#if JVET_Z0120_SII_SEI_PROCESSING
                  ,
                  getShutterFilterFlag()
#endif
#if ENABLE_NNLF
                    ,
                  sps.m_nnlfStore
#endif
      );
    }
  }

  pic->m_extendedBorder  = false;
  pic->m_neededForOutput = false;
  pic->m_reconstructed   = false;

  return pic;
}

void DecLib::executeLoopFilters()
{
  PROFILER_SCOPE(1, g_timeProfiler, P_LOOPFILTERS);
  if (!m_pic)
  {
    return; // nothing to deblock
  }

  m_pic->m_cs->slice->startProcessingTimer();

  CodingStructure &cs = *m_pic->m_cs;

  if (cs.sps->m_lmcsEnabled && cs.picHeader->m_lmcsEnabledFlag)
  {
    PROFILER_TO_NEXT_SCOPE(1, g_timeProfiler, P_RESHAPER);
    const PreCalcValues &pcv = *cs.pcv;
    for (uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
    {
      for (uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
      {
        const CodingUnit *cu = cs.getCU(Position(xPos, yPos), ChannelType::LUMA);
        if (cu->slice->m_lmcsEnabledFlag)
        {
          const uint32_t width  = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
          const uint32_t height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
          const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));
          cs.getRecoBuf(area).get(COMP_Y).rspSignal(m_cReshaper.m_invLUT);
        }
      }
    }
#if ENABLE_NNLF
    if (cs.sps->m_nnlfStore)
    {
      uint64_t culength = cs.cus.size();
      for (uint64_t n = 0; n < culength; n++)
      {
        CodingUnit *cu = cs.cus.at(n);
        if (cu->slice->m_lmcsEnabledFlag)
        {
          if (((cu->predMode == MODE_INTRA || cu->predMode == MODE_IBC) && cu->chType != ChannelType::CHROMA) ||
              (cu->predMode == MODE_INTER && m_cReshaper.m_ctuFlag && cu->ciipFlag))
          {
            m_pic->getPredBufCustom(cu->block(CompID::COMP_Y)).rspSignal(m_cReshaper.m_invLUT);
          }
        }
      }
    }
#endif
    m_cReshaper.m_recReshaped = false;
    m_cSAO.setReshaper(&m_cReshaper);
    PROFILER_TO_PREV_SCOPE(1, g_timeProfiler, P_LOOPFILTERS);
  }

  if (cs.sps->m_alfEnabledFlag && cs.sps->m_alfImprovementsEnabledFlag)
  {
    m_alfEcm->copyAddInputsBeforeDBF(cs, m_cTrQuant);
  }

#if ENABLE_NNLF
  if (cs.sps->m_nnlfStore)
  {
    m_pic->getBsMapBuf().fill(0);
    m_pic->dumpPicBpmInfo();
    m_pic->dumpQpBlock();
    m_pic->getRecBeforeDbfBuf().copyFrom(m_pic->getRecoBuf());
  }
#endif

  // deblocking filter
  m_deblockingFilter.deblockingFilterPic(cs);
  PROFILER_TO_NEXT_SCOPE(1, g_timeProfiler, P_SAO);
  if (cs.sps->m_ccSaoEnabledFlag)
  {
    m_cSAO.getCcSaoBuf().copyFrom(cs.getRecoBuf());
  }

#if ENABLE_NNLF
  if (cs.sps->m_nnlf && !cs.picHeader->m_nnlfDisabled)
  {
    m_pic->paddingBsMapBufBorder(NNLF_UNIFIED_INFER_SIZE_EXT);
    m_pic->paddingRecBeforeDbfBufBorder(NNLF_UNIFIED_INFER_SIZE_EXT);
    m_pic->paddingPredBufBorder(NNLF_UNIFIED_INFER_SIZE_EXT);
    m_pic->paddingBPMBufBorder(NNLF_UNIFIED_INFER_SIZE_EXT);
    m_pic->paddingBlockQPBufBorder(NNLF_UNIFIED_INFER_SIZE_EXT, cs.slice->m_iSliceQp);

    const bool transInput = cs.sps->m_nnlf == NNLFUnifiedID::VLOP || cs.sps->m_nnlf == NNLFUnifiedID::LOP;
    m_unifiedNnlf->setNnlfParams(m_decCfg.m_nnlfDebugOption == 2, transInput);
    m_unifiedNnlf->setPicprms(&m_pic->m_picprm);
    m_unifiedNnlf->setSliceprms(cs.slice->m_nnlfUnifiedParam);
    m_unifiedNnlf->filter(*m_pic, false);
  }
#endif

  if (cs.sps->m_saoEnabledFlag || cs.pps->m_BIF || cs.pps->m_chromaBIF)
  {
    m_cSAO.SAOProcess(cs, cs.picture->getSAO());
  }
  if (cs.sps->m_ccSaoEnabledFlag)
  {
    m_cSAO.getCcSaoComParam() = cs.slice->m_ccSaoComParam;
    m_cSAO.CCSAOProcess(cs);
  }
  m_cSAO.jointClipSaoBifCcSao(cs);
  PROFILER_TO_PREV_SCOPE(1, g_timeProfiler, P_LOOPFILTERS);

  PelStorage cccmCorrection;
  m_cLoopFilterCccm.lfCccmSetFrameLevelInheritedParameters(cs);
  if (cs.slice->m_lfCccmEnabledFlag)
  {
    m_cLoopFilterCccm.lfCccmInitIntraPred(&m_cIntraPred);
    cccmCorrection.create(cs.getRecoBuf().chromaFormat,
                          Area(0, 0, cs.picture->lwidth(), cs.picture->lheight())); // in ECM was CHROMA_ONLY_420
    cccmCorrection.copyFrom(cs.getRecoBuf(), false, true);
    m_cLoopFilterCccm.lfCccmCreatePelStorage(cs);
    for (int ctuRsAddr = 0; ctuRsAddr < cs.picture->m_ctuNums; ctuRsAddr++)
    {
      m_cLoopFilterCccm.calculateCorrelationMatrix = m_cIntraPred.m_calculateCorrelationMatrix;
      m_cLoopFilterCccm.lfCccmCtuProcess(cs, cs.getRecoBuf(), ctuRsAddr, cccmCorrection.Cb(), cccmCorrection.Cr());
    }
    cccmCorrection.getBuf(COMP_Cb).subtract(cs.getRecoBuf(COMP_Cb));
    cccmCorrection.getBuf(COMP_Cr).subtract(cs.getRecoBuf(COMP_Cr));
  }

  if (cs.sps->m_alfEnabledFlag)
  {
    PROFILER_TO_NEXT_SCOPE(1, g_timeProfiler, P_ALF);
    // ALF decodes the differentially coded coefficients and stores them in the parameters structure.
    // Code could be restructured to do directly after parsing. So far we just pass a fresh non-const
    // copy in case the APS gets used more than once.
    if (cs.sps->m_alfImprovementsEnabledFlag)
    {
      m_alfEcm->getCcAlfFilterParam() = cs.slice->m_ccAlfFilterParam.getEcmParam();
      m_alfEcm->ALFProcess(cs);
    }
    else
    {
      m_alfVtm->getCcAlfFilterParam() = cs.slice->m_ccAlfFilterParam.getVtmParam();
      m_alfVtm->ALFProcess(cs);
    }
    DTRACE(g_trace_ctx, D_CRC, "ALF");
    DTRACE_CRC(g_trace_ctx, D_CRC, cs, cs.getRecoBuf());

    DTRACE_PIC_COMP(D_REC_CB_LUMA_ALF, cs, cs.getRecoBuf(), COMP_Y);
    DTRACE_PIC_COMP(D_REC_CB_CHROMA_ALF, cs, cs.getRecoBuf(), COMP_Cb);
    DTRACE_PIC_COMP(D_REC_CB_CHROMA_ALF, cs, cs.getRecoBuf(), COMP_Cr);

    PROFILER_TO_PREV_SCOPE(1, g_timeProfiler, P_LOOPFILTERS);
  }

  if (cs.slice->m_lfCccmEnabledFlag)
  {
    cs.getRecoBuf(COMP_Cb).reconstruct(cs.getRecoBuf(COMP_Cb), cccmCorrection.getBuf(COMP_Cb),
                                       cs.slice->clpRng(COMP_Cb));
    cs.getRecoBuf(COMP_Cr).reconstruct(cs.getRecoBuf(COMP_Cr), cccmCorrection.getBuf(COMP_Cr),
                                       cs.slice->clpRng(COMP_Cr));
    DTRACE(g_trace_ctx, D_CRC, "CCCM");
    DTRACE_CRC(g_trace_ctx, D_CRC, cs, cs.getRecoBuf());
  }

  for (int i = 0; i < cs.pps->m_numSubPics && m_targetSubPicIdx; i++)
  {
    // keep target subpic samples untouched, for other subpics mask their output sample value to 0
    int targetSubPicIdx = m_targetSubPicIdx - 1;
    if (i != targetSubPicIdx)
    {
      SubPic   subPicNoUse = cs.pps->m_subPics[i];
      uint32_t left        = subPicNoUse.m_subPicLeft;
      uint32_t right       = subPicNoUse.m_subPicRight;
      uint32_t top         = subPicNoUse.m_subPicTop;
      uint32_t bottom      = subPicNoUse.m_subPicBottom;
      for (uint32_t row = top; row <= bottom; row++)
      {
        for (uint32_t col = left; col <= right; col++)
        {
          cs.getRecoBuf().Y().at(col, row)            = 0;
          // for test only, hard coding using 4:2:0 chroma format
          cs.getRecoBuf().Cb().at(col >> 1, row >> 1) = 0;
          cs.getRecoBuf().Cr().at(col >> 1, row >> 1) = 0;
        }
      }
    }
  }

  m_pic->m_cs->slice->stopProcessingTimer();
}

void DecLib::applyNnPostFilter()
{
  if (m_cListPic.empty())
  {
    return;
  }
  m_nnPostFiltering.filterPictures(m_cListPic);
}

void DecLib::finishPictureLight(int &poc, PicList *&rPicList)
{
  Slice *pcSlice = m_pic->m_cs->slice;

  m_pic->m_neededForOutput = pcSlice->m_picHeader->m_picOutputFlag;

  const VPS *vps = pcSlice->m_vps;
  if (vps != nullptr)
  {
    if (!vps->m_vpsEachLayerIsAnOlsFlag)
    {
      const int layerId        = pcSlice->m_nuhLayerId;
      const int generalLayerId = vps->m_generalLayerIdx[layerId];
      bool      layerIsOutput  = true;

      if (vps->m_vpsOlsModeIdc == 0)
      {
        layerIsOutput = generalLayerId == vps->m_targetOlsIdx;
      }
      else if (vps->m_vpsOlsModeIdc == 1)
      {
        layerIsOutput = generalLayerId <= vps->m_targetOlsIdx;
      }
      else if (vps->m_vpsOlsModeIdc == 2)
      {
        layerIsOutput = vps->m_vpsOlsOutputLayerFlag[vps->m_targetOlsIdx][generalLayerId];
      }
      if (!layerIsOutput)
      {
        m_pic->m_neededForOutput = false;
      }
    }
  }
  m_pic->m_reconstructed = true;

  Slice::sortPicList(m_cListPic); // sorting for application output
  poc      = pcSlice->m_poc;
  rPicList = &m_cListPic;
  m_puCounter++;
}

void DecLib::finishPicture(int &poc, PicList *&rPicList, MsgLevel msgl, bool associatedWithNewClvs)
{
#if RExt__DECODER_DEBUG_TOOL_STATISTICS
  CodingStatistics::StatTool &s = CodingStatistics::GetStatisticTool(STATS__TOOL_TOTAL_FRAME);
  s.count++;
  s.pixels = s.count * m_pic->lwidth() * m_pic->lheight();
#endif
  CodingStructure &cs = *m_pic->m_cs;
  if ((cs.picture->m_temporalId == 0) || (cs.picture->m_temporalId < cs.slice->m_sps->m_maxSubLayers - 1))
  {
    CS::saveTemporalEipModel(cs);
  }

  Slice *pcSlice = m_pic->m_cs->slice;
  m_prevPicPOC   = pcSlice->m_poc;

  char c = (pcSlice->isIntra() ? 'I' : pcSlice->isInterP() ? 'P' : 'B');
  if (!m_pic->m_referenced)
  {
    c += 32;  // tolower
  }

#if ENABLE_CABAC_DUMP
  CabacRetrain::endFrame(pcSlice->m_poc, pcSlice->m_iSliceQp, pcSlice->m_cabacInitFlag,
                         pcSlice->isIntra()                             ? I_SLICE
                           : pcSlice->isInterP()                        ? P_SLICE
                           : pcSlice->isInterB() && pcSlice->m_checkLdc ? L_SLICE
                                                                        : B_SLICE);
#endif

  if (pcSlice->m_isDRAP)
  {
    c = 'D';
  }
  if (pcSlice->m_edrapRapId > 0)
  {
    c = 'E';
  }

  //-- For time output for each slice
  msg(msgl, "POC %4d LId: %2d TId: %1d ( %s, %c-SLICE, QP%3d ) ", pcSlice->m_poc, pcSlice->m_pic->m_layerId,
      pcSlice->m_uiTLayer, nalUnitTypeToString(pcSlice->m_eNalUnitType), c, pcSlice->m_iSliceQp);
  //if (m_decodedPictureHashSEIEnabled <= 1) //yongkai huo
  {
    // if we auto generate hashes we just skip decoding time output to get compareable log files
    msg(msgl, "[DT %6.3f] ", pcSlice->getProcessingTime());
  }
  for (int refList = 0; refList < 2; refList++)
  {
    msg(msgl, "[L%d", refList);
    for (int refIndex = 0; refIndex < pcSlice->m_numRefIdx[RefPicList(refList)]; refIndex++)
    {
      const ScalingRatio &scaleRatio = pcSlice->getScalingRatio(RefPicList(refList), refIndex);

      if (pcSlice->m_picHeader->m_enableTMVPFlag && pcSlice->m_colFromL0Flag == bool(1 - refList) &&
          pcSlice->m_colRefIdx == refIndex)
      {
        if (scaleRatio != SCALE_1X)
        {
          msg(msgl, " %dc(%1.2lfx, %1.2lfx)", pcSlice->getRefPOC(RefPicList(refList), refIndex),
              double(scaleRatio.x) / (1 << ScalingRatio::BITS), double(scaleRatio.y) / (1 << ScalingRatio::BITS));
        }
        else
        {
          msg(msgl, " %dc", pcSlice->getRefPOC(RefPicList(refList), refIndex));
        }
      }
      else
      {
        if (scaleRatio != SCALE_1X)
        {
          msg(msgl, " %d(%1.2lfx, %1.2lfx)", pcSlice->getRefPOC(RefPicList(refList), refIndex),
              double(scaleRatio.x) / (1 << ScalingRatio::BITS), double(scaleRatio.y) / (1 << ScalingRatio::BITS));
        }
        else
        {
          msg(msgl, " %d", pcSlice->getRefPOC(RefPicList(refList), refIndex));
        }
      }

      if (pcSlice->getRefPOC(RefPicList(refList), refIndex) == pcSlice->m_poc)
      {
        msg(msgl, ".%d", pcSlice->getRefPic(RefPicList(refList), refIndex)->m_layerId);
      }
    }
    msg(msgl, "] ");
  }
  if (m_decodedPictureHashSEIEnabled)
  {
    SEIMessages                  pictureHashes = getSeisByType(m_pic->m_SEIs, SEI::PayloadType::DECODED_PICTURE_HASH);
    const SEIDecodedPictureHash *hash =
      (pictureHashes.size() > 0) ? (SEIDecodedPictureHash *)*(pictureHashes.begin()) : nullptr;
    if (pictureHashes.size() > 1)
    {
      msg(WARNING, "Warning: Got multiple decoded picture hash SEI messages. Using first.");
    }

    if (!hash && m_decodedPictureHashSEIEnabled > 1)
    {
      calcAndPrintHashValue(((const Picture *)m_pic)->getRecoBuf(), HashType(m_decodedPictureHashSEIEnabled - 2),
                            pcSlice->m_sps->m_bitDepths, msgl);
    }
    else
    {
      m_numberOfChecksumErrorsDetected +=
        calcAndPrintHashStatus(((const Picture *)m_pic)->getRecoBuf(), hash, pcSlice->m_sps->m_bitDepths, msgl);
    }

    SEIMessages scalableNestingSeis = getSeisByType(m_pic->m_SEIs, SEI::PayloadType::SCALABLE_NESTING);
    for (auto seiIt: scalableNestingSeis)
    {
      SEIScalableNesting *nestingSei = dynamic_cast<SEIScalableNesting *>(seiIt);
      if (nestingSei->m_snSubpicFlag)
      {
        uint32_t    subpicId = nestingSei->m_snSubpicId.front();
        SEIMessages nestedPictureHashes =
          getSeisByType(nestingSei->m_nestedSEIs, SEI::PayloadType::DECODED_PICTURE_HASH);
        for (auto decPicHash: nestedPictureHashes)
        {
          const SubPic  &subpic  = pcSlice->m_pps->m_subPics[subpicId];
          const UnitArea area    = UnitArea(pcSlice->m_sps->m_chromaFormatIdc,
                                            Area(subpic.m_subPicLeft, subpic.m_subPicTop, subpic.m_subPicWidthInLumaSample,
                                                 subpic.m_subPicHeightInLumaSample));
          PelUnitBuf     recoBuf = m_pic->m_cs->getRecoBuf(area);
          m_numberOfChecksumErrorsDetected += calcAndPrintHashStatus(
            recoBuf, dynamic_cast<SEIDecodedPictureHash *>(decPicHash), pcSlice->m_sps->m_bitDepths, msgl);
        }
      }
    }
  }

  msg(msgl, "\n");

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  m_cacheModel.reportFrame();
  m_cacheModel.accumulateFrame();
  m_cacheModel.clear();
#endif

  m_pic->m_neededForOutput = pcSlice->m_picHeader->m_picOutputFlag;
  if (associatedWithNewClvs && m_pic->m_neededForOutput)
  {
    if (!pcSlice->m_pps->m_mixedNaluTypesInPicFlag && pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_RASL)
    {
      m_pic->m_neededForOutput = false;
    }
    else if (pcSlice->m_pps->m_mixedNaluTypesInPicFlag)
    {
      bool isRaslPic = true;
      for (int i = 0; isRaslPic && i < m_pic->m_numSlices; i++)
      {
        if (!(pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_RASL ||
              pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_RADL))
        {
          isRaslPic = false;
        }
      }
      if (isRaslPic)
      {
        m_pic->m_neededForOutput = false;
      }
    }
  }

  const VPS *vps = pcSlice->m_vps;
  if (vps != nullptr)
  {
    if (!vps->m_vpsEachLayerIsAnOlsFlag)
    {
      const int layerId        = pcSlice->m_nuhLayerId;
      const int generalLayerId = vps->m_generalLayerIdx[layerId];
      bool      layerIsOutput  = true;

      if (vps->m_vpsOlsModeIdc == 0)
      {
        layerIsOutput = generalLayerId == vps->m_targetOlsIdx;
      }
      else if (vps->m_vpsOlsModeIdc == 1)
      {
        layerIsOutput = generalLayerId <= vps->m_targetOlsIdx;
      }
      else if (vps->m_vpsOlsModeIdc == 2)
      {
        layerIsOutput = vps->m_vpsOlsOutputLayerFlag[vps->m_targetOlsIdx][generalLayerId];
      }
      if (!layerIsOutput)
      {
        m_pic->m_neededForOutput = false;
      }
    }
  }
  m_pic->m_reconstructed = true;

  // process buffered suffix APS NALUs
  processSuffixApsNalus();

  Slice::sortPicList(m_cListPic); // sorting for application output
  poc                       = pcSlice->m_poc;
  rPicList                  = &m_cListPic;
  m_isFirstSliceInPicture   = true; // TODO: immer true? hier ist irgendwas faul
  m_maxDecSubPicIdx         = 0;
  m_maxDecSliceAddrInSubPic = -1;

  bool bMCBP = m_pic->m_cs->sps->m_MCBP;
  bool bTMBP = m_pic->m_cs->sps->m_TMBP;
  if (!bMCBP && !bTMBP)
  {
    // use repetitive padding
    m_pic->extendPicBorder(pcSlice->m_pps);
  }
  else if (!bMCBP && bTMBP)
  {
    // use TM-padding (in extendPicBorder)
    m_pic->extendPicBorder(pcSlice->m_pps);
  }
  else if (bMCBP && !bTMBP)
  {
    // use MC-padding
    m_cInterPred.mcFramePad(m_pic, *(m_pic->m_cs->slice));
  }
  else
  {
    // use TM-padding for I-frames only
    if (pcSlice->isIntra())
    {
      m_pic->extendPicBorder(pcSlice->m_pps);
    }
    else
    {
      m_cInterPred.mcFramePad(m_pic, *(m_pic->m_cs->slice));
    }
  }

  m_pic->destroyTempBuffers();
  m_pic->m_cs->destroyCoeffs();
  if (m_pic->m_referenced && pcSlice->m_sps->m_tempPartPredEnabledFlag)
  {
    m_pic->m_cs->setSplitPred();
  }
  m_pic->m_cs->releaseIntermediateData();
  m_pic->m_cs->picHeader->initPicHeader();
  m_puCounter++;
}

void DecLib::checkNoOutputPriorPics(PicList *picList)
{
  if (!picList || !m_isNoOutputPriorPics)
  {
    return;
  }

  PicList::iterator iterPic = picList->begin();

  while (iterPic != picList->end())
  {
    Picture *picTmp = *(iterPic++);
    if (m_lastPOCNoOutputPriorPics != picTmp->m_poc)
    {
      picTmp->m_neededForOutput = false;
    }
  }
}

void DecLib::xUpdateRasInit(Slice *slice)
{
  slice->m_pendingRasInit = false;
  if (slice->m_poc > m_lastRasPoc)
  {
    m_lastRasPoc            = MAX_INT;
    slice->m_pendingRasInit = true;
  }
  if (slice->isIRAP())
  {
    m_lastRasPoc = slice->m_poc;
  }
}

void DecLib::xCreateLostPicture(int iLostPoc, const int layerId)
{
  msg(INFO, "\ninserting lost poc : %d\n", iLostPoc);
  Picture *cFillPic =
    xGetNewPicBuffer(*(m_parameterSetManager.getFirstSPS()), *(m_parameterSetManager.getFirstPPS()), 0, layerId);

  CHECK(!cFillPic->m_slices.size(), "No slices in picture");

  cFillPic->m_slices[0]->initSlice();

  PicList::iterator iterPic    = m_cListPic.begin();
  int               closestPoc = 1000000;
  while (iterPic != m_cListPic.end())
  {
    Picture *pic = *(iterPic++);
    if (abs(pic->m_poc - iLostPoc) < closestPoc && abs(pic->m_poc - iLostPoc) != 0 &&
        pic->m_poc != m_apcSlicePilot->m_poc)
    {
      closestPoc = abs(pic->m_poc - iLostPoc);
    }
  }
  iterPic = m_cListPic.begin();
  while (iterPic != m_cListPic.end())
  {
    Picture *pic = *(iterPic++);
    if (abs(pic->m_poc - iLostPoc) == closestPoc && pic->m_poc != m_apcSlicePilot->m_poc)
    {
      msg(INFO, "copying picture %d to %d (%d)\n", pic->m_poc, iLostPoc, m_apcSlicePilot->m_poc);
      cFillPic->getRecoBuf().copyFrom(pic->getRecoBuf());
      break;
    }
  }

//  for(int ctuRsAddr=0; ctuRsAddr<cFillPic->getNumberOfCtusInFrame(); ctuRsAddr++)  {
//  cFillPic->getCtu(ctuRsAddr)->initCtu(cFillPic, ctuRsAddr); }
  cFillPic->m_referenced       = true;
  cFillPic->m_slices[0]->m_poc = iLostPoc;
  xUpdatePreviousTid0POC(cFillPic->m_slices[0]);
  cFillPic->m_reconstructed   = true;
  cFillPic->m_neededForOutput = true;
  if (m_pocRandomAccess == MAX_INT)
  {
    m_pocRandomAccess = iLostPoc;
  }
}

void DecLib::xCreateUnavailablePicture(const PPS *pps, const int iUnavailablePoc, const bool longTermFlag,
                                       const int temporalId, const int layerId, const bool interLayerRefPicFlag)
{
  msg(INFO, "Note: Inserting unavailable POC : %d\n", iUnavailablePoc);
  auto const sps      = m_parameterSetManager.getSPS(pps->m_spsId);
  Picture   *cFillPic = xGetNewPicBuffer(*sps, *pps, 0, layerId);

  cFillPic->m_cs      = new CodingStructure(g_xuPool);
  cFillPic->m_cs->sps = sps;
  cFillPic->m_cs->pps = pps;
  cFillPic->m_cs->vps = m_parameterSetManager.getVPS(sps->m_vpsId);
  cFillPic->m_cs->create(
    cFillPic->m_cs->sps->m_chromaFormatIdc,
    Area(0, 0, cFillPic->m_cs->pps->m_picWidthInLumaSamples, cFillPic->m_cs->pps->m_picHeightInLumaSamples), true,
    (bool)(cFillPic->m_cs->sps->m_PLTMode), cFillPic->m_cs->sps->m_tempPartPredEnabledFlag);
  cFillPic->allocateNewSlice();
  cFillPic->m_chromaFormatIdc = sps->m_chromaFormatIdc;
  cFillPic->m_bitDepths       = sps->m_bitDepths;

  cFillPic->m_slices[0]->initSlice();

  cFillPic->m_decodingOrderNumber                  = 0;
  cFillPic->m_subLayerNonReferencePictureDueToSTSA = false;
  cFillPic->m_unscaledPic                          = cFillPic;

  uint32_t yFill = 1 << (sps->m_bitDepths[ChannelType::LUMA] - 1);
  uint32_t cFill = 1 << (sps->m_bitDepths[ChannelType::CHROMA] - 1);
  cFillPic->getRecoBuf().Y().fill(yFill);
  cFillPic->getRecoBuf().Cb().fill(cFill);
  cFillPic->getRecoBuf().Cr().fill(cFill);

  //  for(int ctuRsAddr=0; ctuRsAddr<cFillPic->getNumberOfCtusInFrame(); ctuRsAddr++)  {
  //  cFillPic->getCtu(ctuRsAddr)->initCtu(cFillPic, ctuRsAddr); }
  cFillPic->m_referenced           = true;
  cFillPic->m_interLayerRefPicFlag = interLayerRefPicFlag;
  cFillPic->m_longTerm             = longTermFlag;
  cFillPic->m_slices[0]->m_poc     = iUnavailablePoc;
  cFillPic->m_poc                  = iUnavailablePoc;
  if ((cFillPic->m_slices[0]->m_uiTLayer == 0) &&
      (cFillPic->m_slices[0]->m_eNalUnitType != NAL_UNIT_CODED_SLICE_RASL) &&
      (cFillPic->m_slices[0]->m_eNalUnitType != NAL_UNIT_CODED_SLICE_RADL))
  {
    m_prevTid0POC = cFillPic->m_slices[0]->m_poc;
  }

  cFillPic->m_reconstructed           = true;
  cFillPic->m_neededForOutput         = false;
  // picture header is not derived for generated reference picture
  cFillPic->m_slices[0]->m_picHeader  = nullptr;
  cFillPic->m_temporalId              = temporalId;
  cFillPic->m_nonReferencePictureFlag = false;
  cFillPic->m_slices[0]->m_pps        = pps;

  if (m_pocRandomAccess == MAX_INT)
  {
    m_pocRandomAccess = iUnavailablePoc;
  }
}

void DecLib::checkPicTypeAfterEos()
{
  int layerId = m_pic->m_slices[0]->m_nuhLayerId;
  if (m_prevEOS[layerId])
  {
    bool isIrapOrGdrPu = !m_pic->m_cs->pps->m_mixedNaluTypesInPicFlag &&
      (m_pic->m_slices[0]->isIRAP() || m_pic->m_slices[0]->m_eNalUnitType == NAL_UNIT_CODED_SLICE_GDR);
    CHECK(!isIrapOrGdrPu,
          "when present, the next PU of a particular layer after an EOS NAL unit that belongs to the same layer shall "
          "be an IRAP or GDR PU");

    m_prevEOS[layerId] = false;
  }
}

void DecLib::checkLayerIdIncludedInCvss()
{
  if (m_accessUnitPicInfo.empty())
  {
    // don't try to access, if there are no entries (e.g. bitstreams ends after skipping leading pictures)
    return;
  }
  if ((m_vps->m_maxLayers == 1 || m_audIrapOrGdrAuFlag) &&
      (m_isFirstAuInCvs || m_accessUnitPicInfo.begin()->m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
       m_accessUnitPicInfo.begin()->m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL))
  {
    // store layerIDs in the first AU
    m_firstAccessUnitPicInfo.assign(m_accessUnitPicInfo.begin(), m_accessUnitPicInfo.end());
  }
  else
  {
    // check whether the layerIDs in an AU are included in the layerIDs of the first AU
    for (auto pic = m_accessUnitPicInfo.begin(); pic != m_accessUnitPicInfo.end(); pic++)
    {
      bool layerIdFind;
      if (m_firstAccessUnitPicInfo.size() == 0)
      {
        msg(NOTICE, "Note: checkIncludedInFirstAu(), m_firstAccessUnitPicInfo.size() is 0.\n");
        continue;
      }
      for (auto picFirst = m_firstAccessUnitPicInfo.begin(); picFirst != m_firstAccessUnitPicInfo.end(); picFirst++)
      {
        layerIdFind = pic->m_nuhLayerId == picFirst->m_nuhLayerId ? true : false;
        if (layerIdFind)
        {
          break;
        }
      }
      CHECK(!layerIdFind,
            "each picture in an AU in a CVS shall have nuh_layer_id equal to the nuh_layer_id of one of the pictures "
            "present in the first AU of the CVS");
    }

    // check whether the layerID of EOS_NUT is included in the layerIDs of the first AU
    for (int i = 0; i < m_vps->m_maxLayers; i++)
    {
      int eosLayerId = m_vps->m_vpsLayerId[i];
      if (m_accessUnitEos[eosLayerId])
      {
        bool eosLayerIdFind;
        for (auto picFirst = m_firstAccessUnitPicInfo.begin(); picFirst != m_firstAccessUnitPicInfo.end(); picFirst++)
        {
          eosLayerIdFind = eosLayerId == picFirst->m_nuhLayerId ? true : false;
          if (eosLayerIdFind)
          {
            break;
          }
        }
        CHECK(!eosLayerIdFind,
              "When nal_unit_type is equal to EOS_NUT, nuh_layer_id shall be equal to one of the nuh_layer_id values "
              "of the layers present in the CVS");
      }
    }
  }
}

void DecLib::resetIsFirstAuInCvs()
{
  // update the value of m_isFirstAuInCvs for the next AU according to NAL_UNIT_EOS in each layer
  for (auto pic = m_accessUnitPicInfo.begin(); pic != m_accessUnitPicInfo.end(); pic++)
  {
    m_isFirstAuInCvs = m_accessUnitEos[pic->m_nuhLayerId] ? true : false;
    if (!m_isFirstAuInCvs)
    {
      break;
    }
  }
}

void DecLib::CheckNoOutputPriorPicFlagsInAccessUnit()
{
  if (m_accessUnitNoOutputPriorPicFlags.size() > 1)
  {
    bool anchor          = m_accessUnitNoOutputPriorPicFlags[0];
    bool isDiffFlagsInAu = std::find(m_accessUnitNoOutputPriorPicFlags.begin(), m_accessUnitNoOutputPriorPicFlags.end(),
                                     !anchor) != m_accessUnitNoOutputPriorPicFlags.end();
    CHECK(
      isDiffFlagsInAu,
      "The value of no_output_of_prior_pics_flag, when present, is required to be the same for all pictures in an AU");
  }
}

void DecLib::checkTidLayerIdInAccessUnit()
{
  int firstPicTid     = m_accessUnitPicInfo.begin()->m_temporalId;
  int firstPicLayerId = m_accessUnitPicInfo.begin()->m_nuhLayerId;

  bool isPicTidInAuSame                    = true;
  bool isSeiTidInAuSameAsAuTid             = true;
  bool isFdNaluLayerIdSameAsVclNaluLayerId = true;
  bool isFdTidInAuSameAsAuTid              = true;

  for (auto pic = m_accessUnitPicInfo.begin(); pic != m_accessUnitPicInfo.end(); pic++)
  {
    if (pic->m_temporalId != firstPicTid)
    {
      isPicTidInAuSame = false;
      break;
    }
  }
  CHECK(!isPicTidInAuSame, "All pictures in an AU shall have the same value of TemporalId");

  for (auto tid = m_accessUnitSeiTids.begin(); tid != m_accessUnitSeiTids.end(); tid++)
  {
    if ((*tid) != firstPicTid)
    {
      isSeiTidInAuSameAsAuTid = false;
      break;
    }
  }
  CHECK(!isSeiTidInAuSameAsAuTid,
        "The TemporalId of an SEI NAL unit shall be equal to the TemporalId of the AU containing the NAL unit");

  for (auto tempNalu = m_accessUnitNals.begin(); tempNalu != m_accessUnitNals.end(); tempNalu++)
  {
    if ((tempNalu->m_nalUnitType == NAL_UNIT_FD) && (tempNalu->m_nuhLayerId != firstPicLayerId))
    {
      isFdNaluLayerIdSameAsVclNaluLayerId = false;
      break;
    }
  }
  CHECK(!isFdNaluLayerIdSameAsVclNaluLayerId,
        "The nuh_layer_id of a filler data NAL unit shall be equal to the nuh_layer_id of associated VCL NAL unit");

  for (auto tempNalu = m_accessUnitNals.begin(); tempNalu != m_accessUnitNals.end(); tempNalu++)
  {
    if ((tempNalu->m_nalUnitType == NAL_UNIT_FD) && (tempNalu->m_temporalId != firstPicTid))
    {
      isFdTidInAuSameAsAuTid = false;
      break;
    }
  }
  CHECK(!isFdTidInAuSameAsAuTid,
        "The TemporalId of a filler data NAL unit shall be equal to the TemporalId of the AU containing the NAL unit");
}

void DecLib::checkSEIInAccessUnit()
{
  int  olsIdxIncludeAllLayes = -1;
  bool isNonNestedSliFound   = false;

  bool bSdiPresentInAu                = false;
  bool bAuxSEIsBeforeSdiSEIPresent[4] = { false, false, false, false };
  for (auto &sei: m_accessUnitSeiPayLoadTypes)
  {
    enum NalUnitType      naluType    = std::get<0>(sei);
    enum SEI::PayloadType payloadType = std::get<2>(sei);
    if (naluType == NAL_UNIT_PREFIX_SEI &&
        ((payloadType == SEI::PayloadType::BUFFERING_PERIOD || payloadType == SEI::PayloadType::PICTURE_TIMING ||
          payloadType == SEI::PayloadType::DECODING_UNIT_INFO ||
          payloadType == SEI::PayloadType::SUBPICTURE_LEVEL_INFO)))
    {
      bool olsIncludeAllLayersFind = false;
      for (int i = 0; i < m_vps->m_vpsNumOutputLayerSets; i++)
      {
        for (auto pic = m_firstAccessUnitPicInfo.begin(); pic != m_firstAccessUnitPicInfo.end(); pic++)
        {
          int targetLayerId = pic->m_nuhLayerId;
          for (int j = 0; j < m_vps->m_numLayersInOls[i]; j++)
          {
            olsIncludeAllLayersFind = m_vps->m_layerIdInOls[i][j] == targetLayerId ? true : false;
            if (olsIncludeAllLayersFind)
            {
              break;
            }
          }
          if (!olsIncludeAllLayersFind)
          {
            break;
          }
        }
        if (olsIncludeAllLayersFind)
        {
          olsIdxIncludeAllLayes = i;
          if (payloadType == SEI::PayloadType::SUBPICTURE_LEVEL_INFO)
          {
            isNonNestedSliFound = true;
          }
          break;
        }
      }
      CHECK(!olsIncludeAllLayersFind,
            "When there is no OLS that includes all layers in the current CVS in the entire bitstream, there shall be "
            "no non-scalable-nested SEI message with payloadType equal to 0 (BP), 1 (PT), 130 (DUI), or 203 (SLI)");
    }
    if (payloadType == SEI::PayloadType::SCALABILITY_DIMENSION_INFO)
    {
      bSdiPresentInAu = true;
    }
    else if (payloadType == SEI::PayloadType::MULTIVIEW_ACQUISITION_INFO && !bSdiPresentInAu)
    {
      bAuxSEIsBeforeSdiSEIPresent[0] = true;
    }
    else if (payloadType == SEI::PayloadType::ALPHA_CHANNEL_INFO && !bSdiPresentInAu)
    {
      bAuxSEIsBeforeSdiSEIPresent[1] = true;
    }
    else if (payloadType == SEI::PayloadType::DEPTH_REPRESENTATION_INFO && !bSdiPresentInAu)
    {
      bAuxSEIsBeforeSdiSEIPresent[2] = true;
    }
    else if (payloadType == SEI::PayloadType::MULTIVIEW_VIEW_POSITION && !bSdiPresentInAu)
    {
      bAuxSEIsBeforeSdiSEIPresent[3] = true;
    }
  }

  CHECK(bSdiPresentInAu && bAuxSEIsBeforeSdiSEIPresent[0],
        "When an AU contains both an SDI SEI message and an MAI SEI message, the SDI SEI message shall precede the MAI "
        "SEI message in decoding order.");
  CHECK(bSdiPresentInAu && bAuxSEIsBeforeSdiSEIPresent[1],
        "When an AU contains both an SDI SEI message with sdi_aux_id[i] equal to 1 for at least one value of i and an "
        "ACI SEI message, the SDI SEI message shall precede the ACI SEI message in decoding order.");
  CHECK(bSdiPresentInAu && bAuxSEIsBeforeSdiSEIPresent[2],
        "When an AU contains both an SDI SEI message with sdi_aux_id[i] equal to 2 for at least one value of i and a "
        "DRI SEI message, the SDI SEI message shall precede the DRI SEI message in decoding order.");

  if (m_isFirstAuInCvs)
  {
    // when a non-nested SLI SEI shows up, check sps_num_subpics_minus1 for the OLS contains all layers with multiple
    // subpictures per picture
    if (isNonNestedSliFound)
    {
      checkMultiSubpicNum(olsIdxIncludeAllLayes);
    }

    // when a nested SLI SEI shows up, loop over all applicable OLSs, and for layers in the each applicable OLS, check
    // sps_num_subpics_minus1 for these layers with multiple subpictures per picture
    for (auto sliInfo = m_accessUnitNestedSliSeiInfo.begin(); sliInfo != m_accessUnitNestedSliSeiInfo.end(); sliInfo++)
    {
      if (sliInfo->m_nestedSliPresent)
      {
        for (uint32_t olsIdxNestedSei = 0; olsIdxNestedSei < sliInfo->m_numOlssNestedSli; olsIdxNestedSei++)
        {
          int olsIdx = sliInfo->m_olsIdxNestedSLI[olsIdxNestedSei];
          checkMultiSubpicNum(olsIdx);
        }
      }
    }
  }
}

void DecLib::checkMultiSubpicNum(int olsIdx)
{
  int multiSubpicNum = 0;
  for (int layerIdx = 0; layerIdx < m_vps->m_numLayersInOls[olsIdx]; layerIdx++)
  {
    uint32_t layerId = m_vps->m_layerIdInOls[olsIdx][layerIdx];
    if (m_accessUnitSpsNumSubpic[layerId] > 1)
    {
      if (multiSubpicNum == 0)
      {
        multiSubpicNum = m_accessUnitSpsNumSubpic[layerId];
      }
      CHECK(multiSubpicNum != m_accessUnitSpsNumSubpic[layerId],
            "When an SLI SEI message is present for a CVS, the value of sps_num_subpics_minus1 shall be the same for "
            "all the SPSs referenced by the pictures in the layers with multiple subpictures per picture.")
    }
  }
}

#define SEI_REPETITION_CONSTRAINT_LIST_SIZE 21

/**
 - Count the number of identical SEI messages in the current picture
 */
void DecLib::checkSeiInPictureUnit()
{
  std::vector<std::tuple<int, uint32_t, uint8_t *>> seiList;

  // payload types subject to constrained SEI repetition
  int picUnitRepConSeiList[SEI_REPETITION_CONSTRAINT_LIST_SIZE] = { 0,   1,   19,  45,  129, 132, 133,
                                                                    137, 144, 145, 147, 148, 149, 150,
                                                                    153, 154, 155, 156, 168, 203, 204 };

  // extract SEI messages from NAL units
  for (auto &sei: m_pictureSeiNalus)
  {
    InputBitstream bs = sei->getBitstream();

    do
    {
      int      payloadType = 0;
      uint32_t val         = 0;

      do
      {
        bs.readByte(val);
        payloadType += val;
      } while (val == 0xFF);

      uint32_t payloadSize = 0;
      do
      {
        bs.readByte(val);
        payloadSize += val;
      } while (val == 0xFF);

      uint8_t *payload = new uint8_t[payloadSize];
      for (uint32_t i = 0; i < payloadSize; i++)
      {
        bs.readByte(val);
        payload[i] = (uint8_t)val;
      }
      seiList.push_back(std::tuple<int, uint32_t, uint8_t *>(payloadType, payloadSize, payload));
    } while (bs.getNumBitsLeft() > 8);
  }

  // count repeated messages in list
  for (uint32_t i = 0; i < seiList.size(); i++)
  {
    int      k, count = 1;
    int      payloadType1 = std::get<0>(seiList[i]);
    uint32_t payloadSize1 = std::get<1>(seiList[i]);
    uint8_t *payload1     = std::get<2>(seiList[i]);

    // only consider SEI payload types in the PicUnitRepConSeiList
    for (k = 0; k < SEI_REPETITION_CONSTRAINT_LIST_SIZE; k++)
    {
      if (payloadType1 == picUnitRepConSeiList[k])
      {
        break;
      }
    }
    if (k >= SEI_REPETITION_CONSTRAINT_LIST_SIZE)
    {
      continue;
    }

    // compare current SEI message with remaining messages in the list
    for (uint32_t j = i + 1; j < seiList.size(); j++)
    {
      int      payloadType2 = std::get<0>(seiList[j]);
      uint32_t payloadSize2 = std::get<1>(seiList[j]);
      uint8_t *payload2     = std::get<2>(seiList[j]);

      // check for identical SEI type, size, and payload
      if (payloadType1 == payloadType2 && payloadSize1 == payloadSize2)
      {
        if (memcmp(payload1, payload2, payloadSize1 * sizeof(uint8_t)) == 0)
        {
          count++;
        }
      }
    }
    CHECK(count > 4,
          "There shall be less than or equal to 4 identical sei_payload( ) syntax structures within a picture unit.");
  }

  // free SEI message list memory
  for (uint32_t i = 0; i < seiList.size(); i++)
  {
    uint8_t *payload = std::get<2>(seiList[i]);
    delete[] payload;
  }
  seiList.clear();
}

/**
 - Reset list of SEI NAL units from the current picture
 */
void DecLib::resetPictureSeiNalus()
{
  while (!m_pictureSeiNalus.empty())
  {
    delete m_pictureSeiNalus.front();
    m_pictureSeiNalus.pop_front();
  }
}

/**
 - Reset list of Prefix SEI NAL units from the current picture
 */
void DecLib::resetPrefixSeiNalus()
{
  while (!m_prefixSEINALUs.empty())
  {
    delete m_prefixSEINALUs.front();
    m_prefixSEINALUs.pop_front();
  }
}

void DecLib::checkSeiContentInAccessUnit()
{
  if (m_accessUnitSeiNalus.empty())
  {
    return;
  }
  std::vector<SeiPayload> seiList;   // payloadType, olsId, isNestedSEI, payloadSize, payload, duiIdx, subPicId

  // get the OLSs that cover all layers
  std::vector<uint32_t> olsIds;
  for (uint32_t i = 0; i < m_vps->m_vpsNumOutputLayerSets; i++)
  {
    bool olsIncludeAllLayersFind = false;
    for (auto pic = m_firstAccessUnitPicInfo.begin(); pic != m_firstAccessUnitPicInfo.end(); pic++)
    {
      int targetLayerId = pic->m_nuhLayerId;
      for (int j = 0; j < m_vps->m_numLayersInOls[i]; j++)
      {
        olsIncludeAllLayersFind = m_vps->m_layerIdInOls[i][j] == targetLayerId ? true : false;
        if (olsIncludeAllLayersFind)
        {
          break;
        }
      }
      if (!olsIncludeAllLayersFind)
      {
        break;
      }
    }
    if (olsIncludeAllLayersFind)
    {
      olsIds.push_back(i);
    }
  }

  // extract SEI messages from NAL units
  for (auto &sei: m_accessUnitSeiNalus)
  {
    InputBitstream bs = sei->getBitstream();

    do
    {
      int      payloadTypeVal = 0;
      uint32_t payloadLayerId = sei->m_nuhLayerId;
      uint32_t val            = 0;

      do
      {
        bs.readByte(val);
        payloadTypeVal += val;
      } while (val == 0xFF);

      auto payloadType = SEI::PayloadType(payloadTypeVal);

      if (payloadType == SEI::PayloadType::USER_DATA_REGISTERED_ITU_T_T35 ||
          payloadType == SEI::PayloadType::USER_DATA_UNREGISTERED)
      {
        break;
      }

      uint32_t payloadSize = 0;
      do
      {
        bs.readByte(val);
        payloadSize += val;
      } while (val == 0xFF);

      if (payloadType != SEI::PayloadType::SCALABLE_NESTING)
      {
        if (payloadType == SEI::PayloadType::BUFFERING_PERIOD || payloadType == SEI::PayloadType::PICTURE_TIMING ||
            payloadType == SEI::PayloadType::DECODING_UNIT_INFO ||
            payloadType == SEI::PayloadType::SUBPICTURE_LEVEL_INFO)
        {
          uint8_t *payload = new uint8_t[payloadSize];
          int      duiIdx  = 0;
          if (payloadType == SEI::PayloadType::DECODING_UNIT_INFO)
          {
            m_seiReader.getSEIDecodingUnitInfoDuiIdx(&bs, sei->m_nalUnitType, payloadLayerId, m_HRD, payloadSize,
                                                     duiIdx);
          }
          for (uint32_t i = 0; i < payloadSize; i++)
          {
            bs.readByte(val);
            payload[i] = (uint8_t)val;
          }
          for (uint32_t i = 0; i < olsIds.size(); i++)
          {
            if (i == 0)
            {
              seiList.push_back(SeiPayload { payloadType, olsIds.at(i), false, payloadSize, payload, duiIdx, 0 });
            }
            else
            {
              uint8_t *payloadTemp = new uint8_t[payloadSize];
              memcpy(payloadTemp, payload, payloadSize * sizeof(uint8_t));
              seiList.push_back(SeiPayload { payloadType, olsIds.at(i), false, payloadSize, payloadTemp, duiIdx, 0 });
            }
          }
        }
        else
        {
          uint8_t *payload = new uint8_t[payloadSize];
          for (uint32_t i = 0; i < payloadSize; i++)
          {
            bs.readByte(val);
            payload[i] = (uint8_t)val;
          }
          seiList.push_back(SeiPayload { payloadType, payloadLayerId, false, payloadSize, payload, 0, 0 });
        }
      }
      else
      {
        const SPS *sps = m_parameterSetManager.getActiveSPS();
        const VPS *vps = m_parameterSetManager.getVPS(sps->m_vpsId);
        m_seiReader.parseAndExtractSEIScalableNesting(&bs, sei->m_nalUnitType, payloadLayerId, vps, sps, m_HRD,
                                                      payloadSize, &seiList);
      }
    } while (bs.getNumBitsLeft() > 8);
  }

  // check contents of the repeated messages in list
  for (uint32_t i = 0; i < seiList.size(); i++)
  {
    SEI::PayloadType payloadType1    = seiList[i].payloadType;
    int              payLoadLayerId1 = seiList[i].payloadLayerId;
    bool             payLoadNested1  = seiList[i].payloadNested;
    uint32_t         payloadSize1    = seiList[i].payloadSize;
    uint8_t         *payload1        = seiList[i].payload;
    int              duiIdx1         = seiList[i].duiIdx;
    int              subPicId1       = seiList[i].subpicId;

    // compare current SEI message with remaining messages in the list
    for (uint32_t j = i + 1; j < seiList.size(); j++)
    {
      SEI::PayloadType payloadType2    = seiList[j].payloadType;
      int              payLoadLayerId2 = seiList[j].payloadLayerId;
      bool             payLoadNested2  = seiList[j].payloadNested;
      uint32_t         payloadSize2    = seiList[j].payloadSize;
      uint8_t         *payload2        = seiList[j].payload;
      int              duiIdx2         = seiList[j].duiIdx;
      int              subPicId2       = seiList[j].subpicId;

      // check for identical SEI type, olsId or layerId, size, payload, duiIdx, and subPicId
      if (payloadType1 == SEI::PayloadType::BUFFERING_PERIOD || payloadType1 == SEI::PayloadType::PICTURE_TIMING ||
          payloadType1 == SEI::PayloadType::DECODING_UNIT_INFO ||
          payloadType1 == SEI::PayloadType::SUBPICTURE_LEVEL_INFO)
      {
        CHECK(
          (payloadType1 == payloadType2) && (payLoadLayerId1 == payLoadLayerId2) && (duiIdx1 == duiIdx2) &&
            (subPicId1 == subPicId2) &&
            ((payloadSize1 != payloadSize2) || memcmp(payload1, payload2, payloadSize1 * sizeof(uint8_t))),
          "When there are multiple SEI messages with a particular value of payloadType not equal to 133 that are "
          "associated with a particular AU or DU and apply to a particular OLS or layer, regardless of whether some or "
          "all of these SEI messages are scalable-nested, the SEI messages shall have the same SEI payload content.");
      }
      else
      {
        bool sameLayer = false;
        if (!payLoadNested1 && !payLoadNested2)
        {
          sameLayer = (payLoadLayerId1 == payLoadLayerId2);
        }
        else if (payLoadNested1 && payLoadNested2)
        {
          sameLayer = true;
        }
        else
        {
          sameLayer = payLoadNested1 ? payLoadLayerId2 >= payLoadLayerId1 : payLoadLayerId1 >= payLoadLayerId2;
        }
        CHECK(
          payloadType1 == payloadType2 && sameLayer && (duiIdx1 == duiIdx2) && (subPicId1 == subPicId2) &&
            ((payloadSize1 != payloadSize2) || memcmp(payload1, payload2, payloadSize1 * sizeof(uint8_t))),
          "When there are multiple SEI messages with a particular value of payloadType not equal to 133 that are "
          "associated with a particular AU or DU and apply to a particular OLS or layer, regardless of whether some or "
          "all of these SEI messages are scalable-nested, the SEI messages shall have the same SEI payload content.");
      }
    }
  }

  // free SEI message list memory
  for (uint32_t i = 0; i < seiList.size(); i++)
  {
    uint8_t *payload = seiList[i].payload;
    delete[] payload;
  }
  seiList.clear();
}

/**
 - Reset list of SEI NAL units from the current access unit
 */
void DecLib::resetAccessUnitSeiNalus()
{
  while (!m_accessUnitSeiNalus.empty())
  {
    delete m_accessUnitSeiNalus.front();
    m_accessUnitSeiNalus.pop_front();
  }
}

/**
 - Process buffered list of suffix APS NALUs
 */
void DecLib::processSuffixApsNalus()
{
  while (!m_suffixApsNalus.empty())
  {
    xDecodeAPS(*m_suffixApsNalus.front());
    delete m_suffixApsNalus.front();
    m_suffixApsNalus.pop_front();
  }
}

/**
 - Determine if the first VCL NAL unit of a picture is also the first VCL NAL of an Access Unit
 */
bool DecLib::isSliceNaluFirstInAU(bool newPicture, InputNALUnit &nalu)
{
  // can only be the start of an AU if this is the start of a new picture
  if (newPicture == false)
  {
    return false;
  }

  // should only be called for slice NALU types
  if (nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_TRAIL && nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_STSA &&
      nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_RASL && nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_RADL &&
      nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_IDR_W_RADL && nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_IDR_N_LP &&
      nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_CRA && nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_GDR)
  {
    return false;
  }

  // check for layer ID less than or equal to previous picture's layer ID
  if (nalu.m_nuhLayerId <= m_prevLayerID)
  {
    return true;
  }

  // get slice POC
  m_apcSlicePilot->m_picHeader = &m_picHeader;
  m_apcSlicePilot->initSlice();
  InputBitstream bs(nalu.getBitstream());   // create copy
  m_HLSReader.setBitstream(&bs);
  m_HLSReader.getSlicePoc(m_apcSlicePilot, &m_picHeader, &m_parameterSetManager, m_prevTid0POC);

  // check for different POC
  return (m_apcSlicePilot->m_poc != m_prevPOC);
}

void DecLib::checkAPSInPictureUnit()
{
  bool firstVCLFound  = false;
  bool suffixAPSFound = false;
  for (auto &nalu: m_pictureUnitNals)
  {
    if (NALUnit::isVclNalUnitType(nalu))
    {
      firstVCLFound = true;
      CHECK(suffixAPSFound,
            "When any suffix APS NAL units are present in a PU, they shall follow the last VCL unit of the PU");
    }
    else if (nalu == NAL_UNIT_PREFIX_APS)
    {
      CHECK(firstVCLFound,
            "When any prefix APS NAL units are present in a PU, they shall precede the first VCL unit of the PU");
    }
    else if (nalu == NAL_UNIT_SUFFIX_APS)
    {
      suffixAPSFound = true;
    }
  }
}

template<class AlfParametersT> static const typename AlfParametersT::CcAlfFilterParam *getApsCcAlfParams(const APS &aps)
{
  THROW("Need specialization.");
  return nullptr;
}

template<> const AlfParametersEcm::CcAlfFilterParam *getApsCcAlfParams<AlfParametersEcm>(const APS &aps)
{
  return &aps.m_ccAlfAPSParam.getEcmParam();
}

template<> const AlfParametersVtm::CcAlfFilterParam *getApsCcAlfParams<AlfParametersVtm>(const APS &aps)
{
  return &aps.m_ccAlfAPSParam.getVtmParam();
}

template<class AlfParametersT>
static void activateAPSCcAlfComp(const Slice &slice, ParameterSetManager &parameterSetManager, APS **const apss,
                                 const int apsId, typename AlfParametersT::CcAlfFilterParam &filterParam,
                                 const CompID compId)
{
  APS *const aps = parameterSetManager.getAPS(apsId, ApsType::ALF);
  if (aps)
  {
    apss[apsId] = aps;
    if (false == parameterSetManager.activateAPS(apsId, ApsType::ALF))
    {
      THROW("APS activation failed!");
    }

    CHECK(aps->m_temporalId > slice.m_uiTLayer,
          "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit");
    // ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a coded
    // slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer containing
    // a coded slice NAL unit that referrs it.

    const auto    &ccAlfAPSParam = *getApsCcAlfParams<AlfParametersT>(*aps);
    const unsigned compIdx       = static_cast<unsigned>(compId) - 1;

    filterParam.ccAlfFilterCount[compIdx] = ccAlfAPSParam.ccAlfFilterCount[compIdx];
    for (int filterIdx = 0; filterIdx < filterParam.ccAlfFilterCount[compIdx]; filterIdx++)
    {
      filterParam.ccAlfFilterIdxEnabled[compIdx][filterIdx] = ccAlfAPSParam.ccAlfFilterIdxEnabled[compIdx][filterIdx];
      memcpy(filterParam.ccAlfCoeff[compIdx][filterIdx], ccAlfAPSParam.ccAlfCoeff[compIdx][filterIdx],
             sizeof(ccAlfAPSParam.ccAlfCoeff[compIdx][filterIdx]));
    }
  }
  else
  {
    THROW("CC ALF APS not available!");
  }
}

template<class AlfParametersT> static void activateAPSCcAlf(const Slice         &slice,
                                                            ParameterSetManager &parameterSetManager, APS **const apss,
                                                            typename AlfParametersT::CcAlfFilterParam &filterParam)
{
  // cleanup before copying
  for (int filterIdx = 0; filterIdx < AlfParametersT::MAX_NUM_CC_ALF_FILTERS; filterIdx++)
  {
    memset(filterParam.ccAlfCoeff[COMP_Cb - 1][filterIdx], 0, sizeof(filterParam.ccAlfCoeff[COMP_Cb - 1][filterIdx]));
    memset(filterParam.ccAlfCoeff[COMP_Cr - 1][filterIdx], 0, sizeof(filterParam.ccAlfCoeff[COMP_Cr - 1][filterIdx]));
  }
  memset(filterParam.ccAlfFilterIdxEnabled[COMP_Cb - 1], false, sizeof(filterParam.ccAlfFilterIdxEnabled[COMP_Cb - 1]));
  memset(filterParam.ccAlfFilterIdxEnabled[COMP_Cr - 1], false, sizeof(filterParam.ccAlfFilterIdxEnabled[COMP_Cr - 1]));

  if (slice.m_ccAlfCbEnabledFlag)
  {
    activateAPSCcAlfComp<AlfParametersT>(slice, parameterSetManager, apss, slice.m_ccAlfCbApsId, filterParam,
                                         CompID::COMP_Cb);
  }

  if (slice.m_ccAlfCrEnabledFlag)
  {
    activateAPSCcAlfComp<AlfParametersT>(slice, parameterSetManager, apss, slice.m_ccAlfCrApsId, filterParam,
                                         CompID::COMP_Cr);
  }
}

void activateAPS(PicHeader *picHeader, Slice *pSlice, ParameterSetManager &parameterSetManager, APS **apss,
                 APS *&lmcsAPS, APS *&scalingListAPS)
{
  const SPS *sps = parameterSetManager.getSPS(picHeader->m_spsId);
  // luma APSs
  if (pSlice->m_alfEnabledFlag[COMP_Y])
  {
    for (int i = 0; i < pSlice->m_alfApsIdsLuma.size(); i++)
    {
      int  apsId = pSlice->m_alfApsIdsLuma[i];
      APS *aps   = parameterSetManager.getAPS(apsId, ApsType::ALF);

      if (aps)
      {
        apss[apsId] = aps;
        if (false == parameterSetManager.activateAPS(apsId, ApsType::ALF))
        {
          THROW("APS activation failed!");
        }

        CHECK(aps->m_temporalId > pSlice->m_uiTLayer,
              "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit");
        // ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a
        // coded slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer
        // containing a coded slice NAL unit that referrs it.

        CHECK(!isChromaEnabled(sps->m_chromaFormatIdc) && aps->chromaPresentFlag,
              "When ChromaArrayType is equal to 0, the value of aps_chroma_present_flag of an ApsType::ALF shall be "
              "equal to 0");

        CHECK(
          ((sps->m_ccalfEnabledFlag == false) &&
           (aps->m_ccAlfAPSParam.getParam().newCcAlfFilter[0] || aps->m_ccAlfAPSParam.getParam().newCcAlfFilter[1])),
          "When sps_ccalf_enabled_flag is 0, the values of alf_cc_cb_filter_signal_flag and "
          "alf_cc_cr_filter_signal_flag shall be equal to 0");
      }
    }
  }

  if (pSlice->m_alfEnabledFlag[COMP_Cb] || pSlice->m_alfEnabledFlag[COMP_Cr])
  {
    // chroma APS
    int  apsId = pSlice->m_alfApsIdChroma;
    APS *aps   = parameterSetManager.getAPS(apsId, ApsType::ALF);
    if (aps)
    {
      apss[apsId] = aps;
      if (false == parameterSetManager.activateAPS(apsId, ApsType::ALF))
      {
        THROW("APS activation failed!");
      }

      CHECK(aps->m_temporalId > pSlice->m_uiTLayer,
            "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit");
      // ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a coded
      // slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer
      // containing a coded slice NAL unit that referrs it.

      CHECK(((sps->m_ccalfEnabledFlag == false) &&
             (aps->m_ccAlfAPSParam.getParam().newCcAlfFilter[0] || aps->m_ccAlfAPSParam.getParam().newCcAlfFilter[1])),
            "When sps_ccalf_enabled_flag is 0, the values of alf_cc_cb_filter_signal_flag and "
            "alf_cc_cr_filter_signal_flag shall be equal to 0");
    }
  }

  if (sps->m_alfImprovementsEnabledFlag)
  {
    activateAPSCcAlf<AlfParametersEcm>(*pSlice, parameterSetManager, apss, pSlice->m_ccAlfFilterParam.getEcmParam());
  }
  else
  {
    activateAPSCcAlf<AlfParametersVtm>(*pSlice, parameterSetManager, apss, pSlice->m_ccAlfFilterParam.getVtmParam());
  }

  if (picHeader->m_lmcsEnabledFlag && lmcsAPS == nullptr)
  {
    lmcsAPS = parameterSetManager.getAPS(picHeader->m_lmcsApsId, ApsType::LMCS);
    CHECK(lmcsAPS == nullptr, "No LMCS APS present");
    if (lmcsAPS)
    {
      parameterSetManager.clearAPSChangedFlag(picHeader->m_lmcsApsId, ApsType::LMCS);
      if (false == parameterSetManager.activateAPS(picHeader->m_lmcsApsId, ApsType::LMCS))
      {
        THROW("LMCS APS activation failed!");
      }

      CHECK(!isChromaEnabled(sps->m_chromaFormatIdc) && lmcsAPS->chromaPresentFlag,
            "When ChromaArrayType is equal to 0, the value of aps_chroma_present_flag of an ApsType::LMCS shall be "
            "equal to 0");

      CHECK(lmcsAPS->m_reshapeAPSInfo.maxNbitsNeededDeltaCW - 1 < 0 ||
              lmcsAPS->m_reshapeAPSInfo.maxNbitsNeededDeltaCW - 1 > sps->m_bitDepths[ChannelType::LUMA] - 2,
            "The value of lmcs_delta_cw_prec_minus1 of an ApsType::LMCS shall be in the range of 0 to BitDepth 2, "
            "inclusive");

      CHECK(lmcsAPS->m_temporalId > pSlice->m_uiTLayer,
            "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit");
      // ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a coded
      // slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer
      // containing a coded slice NAL unit that referrs it.
    }
  }
  picHeader->m_lmcsAps   = lmcsAPS;
  picHeader->m_lmcsApsId = lmcsAPS ? lmcsAPS->m_APSId : -1;

  if (picHeader->m_explicitScalingListEnabledFlag && scalingListAPS == nullptr)
  {
    scalingListAPS = parameterSetManager.getAPS(picHeader->m_scalingListApsId, ApsType::SCALING_LIST);
    CHECK(scalingListAPS == nullptr, "No SCALING LIST APS present");
    if (scalingListAPS)
    {
      parameterSetManager.clearAPSChangedFlag(picHeader->m_scalingListApsId, ApsType::SCALING_LIST);
      if (false == parameterSetManager.activateAPS(picHeader->m_scalingListApsId, ApsType::SCALING_LIST))
      {
        THROW("SCALING LIST APS activation failed!");
      }

      CHECK((!isChromaEnabled(sps->m_chromaFormatIdc) && scalingListAPS->chromaPresentFlag) ||
              (isChromaEnabled(sps->m_chromaFormatIdc) && !scalingListAPS->chromaPresentFlag),
            "The value of aps_chroma_present_flag of the APS NAL unit having aps_params_type equal to SCALING_APS and "
            "adaptation_parameter_set_id equal to ph_scaling_list_aps_id shall be equal to ChromaArrayType  = =  0 ? 0 "
            ": 1");

      CHECK(scalingListAPS->m_temporalId > pSlice->m_uiTLayer,
            "TemporalId shall be less than or equal to the TemporalId of the coded slice NAL unit");
      // ToDO: APS NAL unit containing the APS RBSP shall have nuh_layer_id either equal to the nuh_layer_id of a coded
      // slice NAL unit that referrs it, or equal to the nuh_layer_id of a direct dependent layer of the layer
      // containing a coded slice NAL unit that referrs it.
    }
  }
  picHeader->m_scalingListAps   = scalingListAPS;
  picHeader->m_scalingListApsId = scalingListAPS ? scalingListAPS->m_APSId : -1;
}

void DecLib::checkParameterSetsInclusionSEIconstraints(const InputNALUnit nalu)
{
  const PPS *pps            = m_pic->m_cs->pps;
  const APS *lmcsAPS        = m_pic->m_cs->lmcsAps;
  const APS *scalinglistAPS = m_pic->m_cs->scalinglistAps;
  APS      **apss           = m_parameterSetManager.getAPSs();

  CHECK(nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA && pps->m_temporalId == nalu.m_temporalId &&
          pps->m_puCounter > m_puCounter,
        "Violating Parameter Sets Inclusion Indication SEI constraint");

  for (int i = 0; i < AlfParameters::ALF_CTB_MAX_NUM_APS; i++)
  {
    if (apss[i] != nullptr)
    {
      CHECK(nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA && apss[i]->m_temporalId == nalu.m_temporalId &&
              apss[i]->m_puCounter > m_puCounter,
            "Violating Parameter Sets Inclusion Indication SEI constraint");
    }
  }
  if (lmcsAPS != nullptr)
  {
    CHECK(nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA && lmcsAPS->m_temporalId == nalu.m_temporalId &&
            lmcsAPS->m_puCounter > m_puCounter,
          "Violating Parameter Sets Inclusion Indication SEI constraint");
  }
  if (scalinglistAPS != nullptr)
  {
    CHECK(nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA && scalinglistAPS->m_temporalId == nalu.m_temporalId &&
            scalinglistAPS->m_puCounter > m_puCounter,
          "Violating Parameter Sets Inclusion Indication SEI constraint");
  }
}

void DecLib::xActivateParameterSets(const InputNALUnit nalu)
{
  const int layerId = nalu.m_nuhLayerId;
  if (m_isFirstSliceInPicture)
  {
    APS **apss = m_parameterSetManager.getAPSs();
    memset(apss, 0, sizeof(*apss) * AlfParameters::ALF_CTB_MAX_NUM_APS);
    const PPS *pps =
      m_parameterSetManager.getPPS(m_picHeader.m_ppsId);   // this is a temporary PPS object. Do not store this value
    CHECK(pps == 0, "Referred to PPS not present");

    const SPS *sps =
      m_parameterSetManager.getSPS(pps->m_spsId);   // this is a temporary SPS object. Do not store this value
    CHECK(sps == 0, "Referred to SPS not present");

    const VPS *vps = m_parameterSetManager.getVPS(sps->m_vpsId);
    CHECK(vps == 0, "Referred to VPS not present");

    if (nullptr != pps->pcv)
    {
      delete m_parameterSetManager.getPPS(m_picHeader.m_ppsId)->pcv;
    }
    m_parameterSetManager.getPPS(m_picHeader.m_ppsId)->pcv = new PreCalcValues(*sps, *pps, false);
    m_parameterSetManager.clearSPSChangedFlag(sps->m_spsId);
    m_parameterSetManager.clearPPSChangedFlag(pps->m_ppsId);

    if (false == m_parameterSetManager.activatePPS(m_picHeader.m_ppsId, m_apcSlicePilot->isIRAP()))
    {
      THROW("Parameter set activation failed!");
    }

    // update the stored VPS to the actually referred to VPS
    m_vps = m_parameterSetManager.getVPS(sps->m_vpsId);
    if (sps->m_vpsId == 0)
    {
      // No VPS in bitstream: set defaults values of variables in VPS to the ones signalled in SPS
      m_vps->m_vpsMaxSubLayers = sps->m_maxSubLayers;
      m_vps->m_vpsLayerId[0]   = sps->m_layerId;
      m_vps->deriveOutputLayerSets();
    }
    else
    {
      // VPS in the bitstream: check that SPS and VPS signalling are compatible
      CHECK(sps->m_maxSubLayers > m_vps->m_vpsMaxSubLayers,
            "The SPS signals more temporal sub-layers than allowed by the VPS");
    }

    m_parameterSetManager.getApsMap(ApsType::ALF)->clearActive();
    for (int i = 0; i < AlfParameters::ALF_CTB_MAX_NUM_APS; i++)
    {
      APS *aps = m_parameterSetManager.getAPS(i, ApsType::ALF);
      if (aps)
      {
        m_parameterSetManager.clearAPSChangedFlag(i, ApsType::ALF);
      }
    }
    APS *lmcsAPS        = nullptr;
    APS *scalinglistAPS = nullptr;
    activateAPS(&m_picHeader, m_apcSlicePilot, m_parameterSetManager, apss, lmcsAPS, scalinglistAPS);

    if (((vps != nullptr) && (vps->m_vpsGeneralHrdParamsPresentFlag)) || (sps->m_generalHrdParametersPresentFlag))
    {
      const GeneralHrdParams *generalHrdParams =
        (sps->m_generalHrdParametersPresentFlag ? &sps->m_generalHrdParams : &vps->m_generalHrdParams);
      m_HRD.m_generalHrdParams = *generalHrdParams;
    }

    xParsePrefixSEImessages();

    if (sps->m_spsRangeExtension.m_extendedPrecisionProcessingFlag || sps->m_bitDepths[ChannelType::LUMA] > 12 ||
        sps->m_bitDepths[ChannelType::CHROMA] > 12)
    {
      THROW("High bit depth support must be enabled at compile-time in order to decode this bitstream\n");
    }

    m_apcSlicePilot->applyReferencePictureListBasedMarking(m_cListPic, &m_apcSlicePilot->m_rpl[RPL0],
                                                           &m_apcSlicePilot->m_rpl[RPL1], layerId, *pps);

    //  Get a new picture buffer. This will also set up m_pic, and therefore give us a SPS and PPS pointer that we can
    //  use.
    m_pic = xGetNewPicBuffer(*sps, *pps, m_apcSlicePilot->m_uiTLayer, layerId);

    m_pic->finalInit(vps, *sps, *pps, &m_picHeader, apss, lmcsAPS, scalinglistAPS);

    m_pic->createGrainSynthesizer(m_firstPictureInSequence, &m_grainCharacteristic, &m_grainBuf,
                                  pps->m_picWidthInLumaSamples, pps->m_picHeightInLumaSamples, sps->m_chromaFormatIdc,
                                  sps->m_bitDepths[ChannelType::LUMA]);
    m_pic->createColourTransfProcessor(m_firstPictureInSequence, &m_colourTranfParams, &m_invColourTransfBuf,
                                       pps->m_picWidthInLumaSamples, pps->m_picHeightInLumaSamples,
                                       sps->m_chromaFormatIdc, sps->m_bitDepths[ChannelType::LUMA]);
    m_firstPictureInSequence = false;
    m_pic->createTempBuffers(m_pic->m_cs->pps->pcv->maxCUWidth);
    m_pic->m_cs->createCoeffs((bool)m_pic->m_cs->sps->m_PLTMode);

    m_pic->allocateNewSlice();
    // make the slice-pilot a real slice, and set up the slice-pilot for the next slice
    CHECK(m_pic->m_slices.size() != (m_uiSliceSegmentIdx + 1), "Invalid number of slices");
    m_apcSlicePilot = m_pic->swapSliceObject(m_apcSlicePilot, m_uiSliceSegmentIdx);

    // we now have a real slice:
    Slice *pSlice = m_pic->m_slices[m_uiSliceSegmentIdx];

    // Update the PPS and SPS pointers with the ones of the picture.
    pps = pSlice->m_pps;
    sps = pSlice->m_sps;

    // fix Parameter Sets, now that we have the real slice
    m_pic->m_cs->slice = pSlice;
    m_pic->m_cs->sps   = sps;
    m_pic->m_cs->pps   = pps;
    m_pic->m_cs->vps   = vps;

    memcpy(m_pic->m_cs->alfApss, apss, sizeof(m_pic->m_cs->alfApss));
    m_pic->m_cs->lmcsAps        = lmcsAPS;
    m_pic->m_cs->scalinglistAps = scalinglistAPS;

    m_pic->m_cs->pcv = pps->pcv;

    // Initialise the various objects for the new set of settings
    const int      maxDepth = floorLog2(sps->m_maxCuWidth) - pps->pcv->minCUWidthLog2;
    const uint32_t log2SaoOffsetScaleLuma =
      (uint32_t)std::max(0, sps->m_bitDepths[ChannelType::LUMA] - MAX_SAO_TRUNCATED_BITDEPTH);
    const uint32_t log2SaoOffsetScaleChroma =
      (uint32_t)std::max(0, sps->m_bitDepths[ChannelType::CHROMA] - MAX_SAO_TRUNCATED_BITDEPTH);
    m_cSAO.destroy();
    m_cSAO.create(pps->m_picWidthInLumaSamples, pps->m_picHeightInLumaSamples, sps->m_chromaFormatIdc,
                  sps->m_maxCuWidth, sps->m_maxCuHeight, maxDepth, log2SaoOffsetScaleLuma, log2SaoOffsetScaleChroma);
    pSlice->m_ccSaoControl[COMP_Y]  = m_cSAO.getCcSaoControlIdc(COMP_Y);
    pSlice->m_ccSaoControl[COMP_Cb] = m_cSAO.getCcSaoControlIdc(COMP_Cb);
    pSlice->m_ccSaoControl[COMP_Cr] = m_cSAO.getCcSaoControlIdc(COMP_Cr);
    m_deblockingFilter.create(maxDepth);
    m_cIntraPred.init(sps->m_chromaFormatIdc, sps->m_bitDepths[ChannelType::LUMA], &m_if);
    m_cInterPred.init(&m_cRdCost, &m_cReshaper, sps->m_chromaFormatIdc, sps->m_maxCuHeight,
                      pps->m_picWidthInLumaSamples, &m_if);
    if (sps->m_lmcsEnabled)
    {
      m_cReshaper.createDec(sps->m_bitDepths[ChannelType::LUMA]);
    }

    bool isField    = false;
    bool isTopField = false;

    if (!m_SEIs.empty())
    {
      // Check if any new Frame Field Info SEI has arrived
      SEIMessages frameFieldSEIs = getSeisByType(m_SEIs, SEI::PayloadType::FRAME_FIELD_INFO);
      if (frameFieldSEIs.size() > 0)
      {
        SEIFrameFieldInfo *ff = (SEIFrameFieldInfo *)*(frameFieldSEIs.begin());
        isField               = ff->m_fieldPicFlag;
        isTopField            = isField && (!ff->m_bottomFieldFlag);
      }
      SEIMessages inclusionSEIs = getSeisByType(m_SEIs, SEI::PayloadType::PARAMETER_SETS_INCLUSION_INDICATION);
      const SEIParameterSetsInclusionIndication *inclusion =
        (inclusionSEIs.size() > 0) ? (SEIParameterSetsInclusionIndication *)*(inclusionSEIs.begin()) : nullptr;
      if (inclusion != nullptr)
      {
        m_seiInclusionFlag = inclusion->m_selfContainedClvsFlag;
      }
    }
    if (m_seiInclusionFlag)
    {
      checkParameterSetsInclusionSEIconstraints(nalu);
    }

    // Set Field/Frame coding mode
    m_pic->m_fieldPic = isField;
    m_pic->m_topField = isTopField;

    // transfer any SEI messages that have been received to the picture
    m_pic->m_SEIs = m_SEIs;
    m_SEIs.clear();

    // Recursive structure
    m_cCuDecoder.init(&m_cTrQuant, &m_cIntraPred, &m_cInterPred);
    {
      m_cCuDecoder.initDecCuReshaper(&m_cReshaper, sps->m_chromaFormatIdc);
    }
    m_cTrQuant.init(m_cTrQuantScalingList.getQuant(), sps->getMaxTbSize(), false, false, false, false);

    // RdCost
    m_cRdCost.setCostMode(COST_STANDARD_LOSSY);   // not used in decoder side RdCost stuff -> set to default

    m_cSliceDecoder.create(pps->m_picWidthInLumaSamples, sps->m_maxCuWidth);

#if ENABLE_NNLF
    if (sps->m_nnlf)
    {
      if (m_unifiedNnlf)
      {
        m_unifiedNnlf->destroy();
        delete m_unifiedNnlf;
        m_unifiedNnlf = nullptr;
      }
      m_unifiedNnlf = new NNFilterUnified;
      CHECK(m_unifiedNnlf == nullptr, "out of memory");

      m_pic->initPicprms(*pSlice);

      const bool transInput = sps->m_nnlf == NNLFUnifiedID::VLOP || sps->m_nnlf == NNLFUnifiedID::LOP;
      m_unifiedNnlf->setNnlfParams(m_decCfg.m_nnlfDebugOption == 2, transInput);
      m_unifiedNnlf->setPicprms(&m_pic->m_picprm);
      m_unifiedNnlf->init(m_decCfg.m_nnlfModelName, pps->m_picWidthInLumaSamples, pps->m_picHeightInLumaSamples,
                          sps->m_chromaFormatIdc, NNLF_UNIFIED_MAX_NUM_PRMS);
    }
#endif

    if (sps->m_alfEnabledFlag)
    {
      const int maxDepth = floorLog2(sps->m_maxCuWidth) - sps->m_log2MinCodingBlockSize;
      if (sps->m_alfImprovementsEnabledFlag)
      {
        if (m_alfEcm == nullptr)
        {
          m_alfEcm = new AdaptiveLoopFilterEcm;
        }
        m_alfEcm->create(pps->m_picWidthInLumaSamples, pps->m_picHeightInLumaSamples, sps->m_chromaFormatIdc,
                         sps->m_maxCuWidth, sps->m_maxCuHeight, maxDepth, sps->m_bitDepths);

        pSlice->m_ccAlfFilterControl[0] = m_alfEcm->getCcAlfControlIdc(COMP_Cb);
        pSlice->m_ccAlfFilterControl[1] = m_alfEcm->getCcAlfControlIdc(COMP_Cr);
      }
      else
      {
        if (m_alfVtm == nullptr)
        {
          m_alfVtm = new AdaptiveLoopFilterVtm;
        }
        m_alfVtm->create(pps->m_picWidthInLumaSamples, pps->m_picHeightInLumaSamples, sps->m_chromaFormatIdc,
                         sps->m_maxCuWidth, sps->m_maxCuHeight, maxDepth, sps->m_bitDepths);

        pSlice->m_ccAlfFilterControl[0] = m_alfVtm->getCcAlfControlIdc(COMP_Cb);
        pSlice->m_ccAlfFilterControl[1] = m_alfVtm->getCcAlfControlIdc(COMP_Cr);
      }
    }
  }
  else
  {
    // make the slice-pilot a real slice, and set up the slice-pilot for the next slice
    m_pic->allocateNewSlice();
    CHECK(m_pic->m_slices.size() != (size_t)(m_uiSliceSegmentIdx + 1), "Invalid number of slices");
    m_apcSlicePilot = m_pic->swapSliceObject(m_apcSlicePilot, m_uiSliceSegmentIdx);

    Slice *pSlice = m_pic->m_slices[m_uiSliceSegmentIdx];   // we now have a real slice.

    const SPS *sps            = pSlice->m_sps;
    const PPS *pps            = pSlice->m_pps;
    APS      **apss           = pSlice->m_alfApss;
    APS       *lmcsAPS        = m_picHeader.m_lmcsAps;
    APS       *scalinglistAPS = m_picHeader.m_scalingListAps;

    // fix Parameter Sets, now that we have the real slice
    m_pic->m_cs->slice = pSlice;
    m_pic->m_cs->sps   = sps;
    m_pic->m_cs->pps   = pps;
    memcpy(m_pic->m_cs->alfApss, apss, sizeof(m_pic->m_cs->alfApss));
    m_pic->m_cs->lmcsAps        = lmcsAPS;
    m_pic->m_cs->scalinglistAps = scalinglistAPS;

    m_pic->m_cs->pcv = pps->pcv;

    // check that the current active PPS has not changed...
    if (m_parameterSetManager.getSPSChangedFlag(sps->m_spsId))
    {
      EXIT("Error - a new SPS has been decoded while processing a picture");
    }
    if (m_parameterSetManager.getPPSChangedFlag(pps->m_ppsId))
    {
      EXIT("Error - a new PPS has been decoded while processing a picture");
    }
    for (int i = 0; i < AlfParameters::ALF_CTB_MAX_NUM_APS; i++)
    {
      APS *aps = m_parameterSetManager.getAPS(i, ApsType::ALF);
      if (aps && m_parameterSetManager.getAPSChangedFlag(i, ApsType::ALF))
      {
        EXIT("Error - a new APS has been decoded while processing a picture");
      }
    }

    if (lmcsAPS && m_parameterSetManager.getAPSChangedFlag(lmcsAPS->m_APSId, ApsType::LMCS))
    {
      EXIT("Error - a new LMCS APS has been decoded while processing a picture");
    }
    if (scalinglistAPS && m_parameterSetManager.getAPSChangedFlag(scalinglistAPS->m_APSId, ApsType::SCALING_LIST))
    {
      EXIT("Error - a new SCALING LIST APS has been decoded while processing a picture");
    }

    activateAPS(&m_picHeader, pSlice, m_parameterSetManager, apss, lmcsAPS, scalinglistAPS);

    m_pic->m_cs->lmcsAps        = lmcsAPS;
    m_pic->m_cs->scalinglistAps = scalinglistAPS;

    xParsePrefixSEImessages();

    // Check if any new SEI has arrived
    if (!m_SEIs.empty())
    {
      // Currently only decoding Unit SEI message occurring between VCL NALUs copied
      SEIMessages &picSEI            = m_pic->m_SEIs;
      SEIMessages  decodingUnitInfos = extractSeisByType(picSEI, SEI::PayloadType::DECODING_UNIT_INFO);
      picSEI.insert(picSEI.end(), decodingUnitInfos.begin(), decodingUnitInfos.end());
      deleteSEIs(m_SEIs);
    }
    if (m_seiInclusionFlag)
    {
      checkParameterSetsInclusionSEIconstraints(nalu);
    }
  }
  xCheckParameterSetConstraints(layerId);
}

void DecLib::xCheckParameterSetConstraints(const int layerId)
{
  // Conformance checks
  Slice     *slice = m_pic->m_slices[m_uiSliceSegmentIdx];
  const SPS *sps   = slice->m_sps;
  const PPS *pps   = slice->m_pps;
  const VPS *vps   = slice->m_vps;

  if (sps->m_vpsId && (vps != nullptr))
  {
    bool setVpsId   = true;
    int  checkLayer = 0;
    while (checkLayer <= layerId)
    {
      if (!m_firstSliceInSequence[checkLayer++])
      {
        setVpsId = false;
      }
    }
    if (setVpsId)
    {
      m_clsVPSid = sps->m_vpsId;
    }
    CHECK(
      m_clsVPSid != sps->m_vpsId,
      "The value of sps_video_parameter_set_id shall be the same in all SPSs that are referred to by CLVSs in a CVS.");
  }

  if (((vps != nullptr) && (vps->m_vpsGeneralHrdParamsPresentFlag)) || (sps->m_generalHrdParametersPresentFlag))
  {
    if (((vps != nullptr) && (vps->m_vpsGeneralHrdParamsPresentFlag)) && (sps->m_generalHrdParametersPresentFlag))
    {
      CHECK(!(vps->m_generalHrdParams == sps->m_generalHrdParams),
            "It is a requirement of bitstream conformance that the content of the general_hrd_parameters( ) syntax "
            "structure present in any VPSs or SPSs in the bitstream shall be identical");
    }
    if (!m_isFirstGeneralHrd)
    {
      CHECK(!(m_prevGeneralHrdParams ==
              (sps->m_generalHrdParametersPresentFlag ? sps->m_generalHrdParams : vps->m_generalHrdParams)),
            "It is a requirement of bitstream conformance that the content of the general_hrd_parameters( ) syntax "
            "structure present in any VPSs or SPSs in the bitstream shall be identical");
    }
    m_prevGeneralHrdParams =
      (sps->m_generalHrdParametersPresentFlag ? sps->m_generalHrdParams : vps->m_generalHrdParams);
  }
  m_isFirstGeneralHrd = false;
  static std::unordered_map<int, int> m_clvssSPSid;

  if (slice->isClvssPu() && m_isFirstSliceInPicture)
  {
    m_clvssSPSid[layerId] = pps->m_spsId;
  }

  CHECK(m_clvssSPSid[layerId] != pps->m_spsId,
        "The value of pps_seq_parameter_set_id shall be the same in all PPSs that are referred to by coded pictures in "
        "a CLVS");

  CHECK(sps->m_GDREnabledFlag == false && m_picHeader.m_gdrPicFlag,
        "When sps_gdr_enabled_flag is equal to 0, the value of ph_gdr_pic_flag shall be equal to 0 ");
  if (!sps->m_useWP)
  {
    CHECK(pps->m_useWP,
          "When sps_weighted_pred_flag is equal to 0, the value of pps_weighted_pred_flag shall be equal to 0.");
  }

  if (!sps->m_useBiWP)
  {
    CHECK(pps->m_useBiWP,
          "When sps_weighted_bipred_flag is equal to 0, the value of pps_weighted_bipred_flag shall be equal to 0.");
  }

  const int minCuSize = 1 << sps->m_log2MinCodingBlockSize;
  CHECK((pps->m_picWidthInLumaSamples % (std::max(8, minCuSize))) != 0,
        "Coded frame width must be a multiple of Max(8, the minimum unit size)");
  CHECK((pps->m_picHeightInLumaSamples % (std::max(8, minCuSize))) != 0,
        "Coded frame height must be a multiple of Max(8, the minimum unit size)");
  if (!sps->m_resChangeInClvsEnabledFlag)
  {
    CHECK(pps->m_picWidthInLumaSamples != sps->m_maxWidthInLumaSamples,
          "When sps_res_change_in_clvs_allowed_flag equal to 0, the value of pps_pic_width_in_luma_samples shall be "
          "equal to sps_pic_width_max_in_luma_samples.");
    CHECK(pps->m_picHeightInLumaSamples != sps->m_maxHeightInLumaSamples,
          "When sps_res_change_in_clvs_allowed_flag equal to 0, the value of pps_pic_height_in_luma_samples shall be "
          "equal to sps_pic_height_max_in_luma_samples.");
  }
  if (sps->m_resChangeInClvsEnabledFlag)
  {
    CHECK(sps->m_subPicInfoPresentFlag != 0,
          "When sps_res_change_in_clvs_allowed_flag is equal to 1, the value of sps_subpic_info_present_flag shall be "
          "equal to 0.");
  }

  if (sps->m_ctuSize + 2 * (1 << sps->m_log2MinCodingBlockSize) > pps->m_picWidthInLumaSamples)
  {
    CHECK(pps->m_wrapAroundEnabledFlag,
          "Wraparound shall be disabled when the value of ( CtbSizeY / MinCbSizeY + 1) is greater than or equal to ( "
          "pps_pic_width_in_luma_samples / MinCbSizeY - 1 )");
  }

  if (vps != nullptr && vps->m_numOutputLayersInOls[vps->m_targetOlsIdx] > 1)
  {
    CHECK(sps->m_maxWidthInLumaSamples > vps->m_olsDpbPicSize[vps->m_targetOlsIdx].width,
          "sps_pic_width_max_in_luma_samples shall be less than or equal to the value of vps_ols_dpb_pic_width[ i ]");
    CHECK(sps->m_maxHeightInLumaSamples > vps->m_olsDpbPicSize[vps->m_targetOlsIdx].height,
          "sps_pic_height_max_in_luma_samples shall be less than or equal to the value of vps_ols_dpb_pic_height[ i ]");
    CHECK(sps->m_chromaFormatIdc > vps->m_olsDpbChromaFormatIdc[vps->m_targetOlsIdx],
          "sps_chroma_format_idc shall be less than or equal to the value of vps_ols_dpb_chroma_format[ i ]");
    CHECK((sps->m_bitDepths[ChannelType::LUMA] - 8) > vps->m_olsDpbBitDepthMinus8[vps->m_targetOlsIdx],
          "sps_bitdepth_minus8 shall be less than or equal to the value of vps_ols_dpb_bitdepth_minus8[ i ]");
  }

  static std::unordered_map<int, ChromaFormat> m_layerChromaFormat;
  static std::unordered_map<int, int>          m_layerBitDepth;

  if (vps != nullptr && vps->m_maxLayers > 1)
  {
    int          curLayerIdx          = vps->m_generalLayerIdx[layerId];
    ChromaFormat curLayerChromaFormat = sps->m_chromaFormatIdc;
    int          curLayerBitDepth     = sps->m_bitDepths[ChannelType::LUMA];

    if (slice->isClvssPu() && m_isFirstSliceInPicture)
    {
      m_layerChromaFormat[curLayerIdx] = curLayerChromaFormat;
      m_layerBitDepth[curLayerIdx]     = curLayerBitDepth;
    }
    else
    {
      CHECK(m_layerChromaFormat[curLayerIdx] != curLayerChromaFormat, "Different chroma format in the same layer.");
      CHECK(m_layerBitDepth[curLayerIdx] != curLayerBitDepth, "Different bit-depth in the same layer.");
    }

    for (int i = 0; i < curLayerIdx; i++)
    {
      if (vps->m_vpsDirectRefLayerFlag[curLayerIdx][i])
      {
        ChromaFormat refLayerChromaFormat = m_layerChromaFormat[i];
        CHECK(curLayerChromaFormat != refLayerChromaFormat,
              "The chroma formats of the current layer and the reference layer are different");
        int refLayerBitDepth = m_layerBitDepth[i];
        CHECK(curLayerBitDepth != refLayerBitDepth,
              "The bit-depth of the current layer and the reference layer are different");
        if (vps->getMaxTidIlRefPicsPlus1(curLayerIdx, i) == 0 && pps->m_mixedNaluTypesInPicFlag)
        {
          for (int j = 0; j < m_uiSliceSegmentIdx; j++)
          {
            Slice *preSlice = m_pic->m_slices[j];
            CHECK((preSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
                   preSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
                   preSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA),
                  "mixed IRAP and non-IRAP NAL units in the picture when sps_video_parameter_set_id is greater than 0 "
                  "and vps_max_tid_il_ref_pics_plus1[i][j] is equal to 0");
          }
        }
      }
    }
  }

  if (sps->m_profileTierLevel.m_constraintInfo.m_oneTilePerPicConstraintFlag)
  {
    CHECK(pps->getNumTiles() != 1,
          "When one_tile_per_pic_constraint_flag is equal to 1, each picture shall contain only one tile");
  }

  if (sps->m_profileTierLevel.m_constraintInfo.m_oneSlicePerPicConstraintFlag)
  {
    CHECK(pps->m_rectSliceFlag && pps->m_numSlicesInPic != 1,
          "When one_slice_per_pic_constraint_flag is equal to 1 and if pps_rect_slice_flag is equal to 1, the value of "
          "pps_num_slices_in_pic_minus1 shall be equal to 0");
  }

  if (sps->m_profileTierLevel.m_constraintInfo.m_noRprConstraintFlag)
  {
    CHECK(sps->m_rprEnabledFlag,
          "When gci_no_ref_pic_resampling_constraint_flag is equal to 1, the value of "
          "sps_ref_pic_resampling_enabled_flag shall be equal to 0");
  }
  if (sps->m_profileTierLevel.m_constraintInfo.m_noResChangeInClvsConstraintFlag)
  {
    CHECK(sps->m_resChangeInClvsEnabledFlag,
          "When gci_no_res_change_in_clvs_constraint_flag is equal to 1, the value of "
          "sps_res_change_in_clvs_allowed_flag shall be equal to 0");
  }

  if (sps->m_profileTierLevel.m_constraintInfo.m_noIdrRplConstraintFlag)
  {
    CHECK(sps->m_idrRefParamList,
          "When gci_no_idr_rpl_constraint_flag equal to 1 , the value of sps_idr_rpl_present_flag shall be equal to 0")
  }

  if (sps->m_profileTierLevel.m_constraintInfo.m_noMixedNaluTypesInPicConstraintFlag)
  {
    CHECK(pps->m_mixedNaluTypesInPicFlag,
          "When gci_no_mixed_nalu_types_in_pic_constraint_flag equal to 1, the value of "
          "pps_mixed_nalu_types_in_pic_flag shall be equal to 0")
  }

  if (sps->m_profileTierLevel.m_constraintInfo.m_noGdrConstraintFlag)
  {
    CHECK(sps->m_GDREnabledFlag,
          "gci_no_gdr_constraint_flag equal to 1 specifies that sps_gdr_enabled_flag for all pictures in OlsInScope "
          "shall be equal to 0");
  }

  if (sps->m_profileTierLevel.m_constraintInfo.m_noRectSliceConstraintFlag)
  {
    CHECK(
      pps->m_rectSliceFlag,
      "When gci_no_rectangular_slice_constraint_flag equal to 1, the value of pps_rect_slice_flag shall be equal to 0")
  }

  if (sps->m_profileTierLevel.m_constraintInfo.m_oneSlicePerSubpicConstraintFlag)
  {
    CHECK(!(pps->m_singleSlicePerSubPicFlag),
          "When gci_one_slice_per_subpic_constraint_flag equal to 1, the value of pps_single_slice_per_subpic_flag "
          "shall be equal to 1")
  }

  if (sps->m_profileTierLevel.m_constraintInfo.m_noSubpicInfoConstraintFlag)
  {
    CHECK(sps->m_subPicInfoPresentFlag,
          "When gci_no_subpic_info_constraint_flag is equal to 1, the value of sps_subpic_info_present_flag shall be "
          "equal to 0")
  }
  if (sps->m_profileTierLevel.m_constraintInfo.m_noMttConstraintFlag)
  {
    CHECK(
      (sps->getMaxMTTHierarchyDepth() || sps->getMaxMTTHierarchyDepthI() || sps->getMaxMTTHierarchyDepthIChroma()),
      "When gci_no_mtt_constraint_flag is equal to 1, the values of sps_max_mtt_hierarchy_depth_intra_slice_luma, "
      "sps_max_mtt_hierarchy_depth_inter_slice and sps_max_mtt_hierarchy_depth_intra_slice_chroma shall be equal to 0");
  }
  if (sps->m_profileTierLevel.m_constraintInfo.m_noWeightedPredictionConstraintFlag)
  {
    CHECK((sps->m_useWP || sps->m_useBiWP),
          "When gci_no_weighted_prediction_constraint_flag is equal to 1, the values of sps_weighted_pred_flag and "
          "sps_weighted_bipred_flag shall be equal to 0");
  }

  if (sps->m_profileTierLevel.m_constraintInfo.m_noChromaQpOffsetConstraintFlag)
  {
    CHECK((pps->getCuChromaQpOffsetListEnabledFlag()),
          "When gci_no_ChromaQpOffset_constraint_flag is equal to 1, the values of "
          "pps_cu_chroma_qp_offset_list_enabled_flag shall be equal to 0");
  }

  CHECK(sps->m_ctuSize > (1 << sps->m_profileTierLevel.m_constraintInfo.m_maxLog2CtuSizeConstraintIdc),
        "The CTU size specified by sps_log2_ctu_size_minus5 shall not exceed the constraint specified by "
        "gci_three_minus_max_log2_ctu_size_constraint_idc");

  if (sps->m_profileTierLevel.m_constraintInfo.m_noLumaTransformSize64ConstraintFlag)
  {
    CHECK(sps->m_log2MaxTbSize != 5,
          "When gci_no_luma_transform_size_64_constraint_flag is equal to 1, the value of "
          "sps_max_luma_transform_size_64_flag shall be equal to 0");
  }

  if (sps->m_maxWidthInLumaSamples == pps->m_picWidthInLumaSamples &&
      sps->m_maxHeightInLumaSamples == pps->m_picHeightInLumaSamples)
  {
    const Window &spsConfWin = sps->m_conformanceWindow;
    const Window &ppsConfWin = pps->m_conformanceWindow;
    CHECK(spsConfWin.m_winLeftOffset != ppsConfWin.m_winLeftOffset,
          "When picture size is equal to maximum picutre size, conformance window left offset in SPS and PPS shall be "
          "equal");
    CHECK(spsConfWin.m_winRightOffset != ppsConfWin.m_winRightOffset,
          "When picture size is equal to maximum picutre size, conformance window right offset in SPS and PPS shall be "
          "equal");
    CHECK(spsConfWin.m_winTopOffset != ppsConfWin.m_winTopOffset,
          "When picture size is equal to maximum picutre size, conformance window top offset in SPS and PPS shall be "
          "equal");
    CHECK(spsConfWin.m_winBottomOffset != ppsConfWin.m_winBottomOffset,
          "When picture size is equal to maximum picutre size, conformance window bottom offset in SPS and PPS shall "
          "be equal");
  }
  int levelIdcSps    = int(sps->m_profileTierLevel.m_levelIdc);
  int maxLevelIdxDci = 0;
  if (m_dci)
  {
    for (int i = 0; i < m_dci->getNumPTLs(); i++)
    {
      if (maxLevelIdxDci < int(m_dci->m_profileTierLevel[i].m_levelIdc))
      {
        maxLevelIdxDci = int(m_dci->m_profileTierLevel[i].m_levelIdc);
      }
    }
    CHECK(levelIdcSps > maxLevelIdxDci,
          "max level signaled in the DCI shall not be less than the level signaled in the SPS");
  }

  if (slice->m_picHeader->m_gdrOrIrapPicFlag && !slice->m_picHeader->m_gdrPicFlag &&
      (!vps || vps->m_vpsIndependentLayerFlag[vps->m_generalLayerIdx[layerId]]))
  {
    CHECK(
      slice->m_picHeader->m_picInterSliceAllowedFlag,
      "When ph_gdr_or_irap_pic_flag is equal to 1 and ph_gdr_pic_flag is equal to 0 and vps_independent_layer_flag[ "
      "GeneralLayerIdx[ nuh_layer_id ] ] is equal to 1, ph_inter_slice_allowed_flag shall be equal to 0");
  }

  if (sps->m_vpsId && vps->m_numLayersInOls[vps->m_targetOlsIdx] == 1)
  {
    CHECK(!sps->m_ptlDpbHrdParamsPresentFlag,
          "When sps_video_parameter_set_id is greater than 0 and there is an OLS that contains only one layer with "
          "nuh_layer_id equal to the nuh_layer_id of the SPS, the value of sps_ptl_dpb_hrd_params_present_flag shall "
          "be equal to 1");
  }

  const ProfileTierLevel &ptl = vps && vps->m_numLayersInOls[vps->m_targetOlsIdx] > 1
    ? vps->m_vpsProfileTierLevel[vps->m_olsPtlIdx[vps->m_targetOlsIdx]]
    : sps->m_profileTierLevel;

  ProfileTierLevelFeatures ptlFeatures;
  ptlFeatures.extractPTLInformation(ptl);
  const ProfileFeatures *profileFeatures = ptlFeatures.getProfileFeatures();
  if (profileFeatures != nullptr)
  {
    CHECK(sps->m_bitDepths[ChannelType::LUMA] > profileFeatures->maxBitDepth, "Bit depth exceeds profile limit");
    CHECK(sps->m_chromaFormatIdc > profileFeatures->maxChromaFormat, "Chroma format exceeds profile limit");
  }
  else
  {
    CHECK(sps->m_profileTierLevel.m_profileIdc != Profile::NONE, "Unknown profile");
    msg(WARNING, "Warning: Profile set to none or unknown value\n");
  }
  const TierLevelFeatures *tierLevelFeatures = ptlFeatures.getTierLevelFeatures();
  if (tierLevelFeatures != nullptr)
  {
    CHECK(pps->m_numTileCols > tierLevelFeatures->maxTileCols,
          "Number of tile columns signaled in PPS exceeds level limit");
    CHECK(pps->getNumTiles() > tierLevelFeatures->maxTilesPerAu, "Number of tiles signaled in PPS exceeds level limit");
  }
  else if (profileFeatures != nullptr)
  {
    CHECK(sps->m_profileTierLevel.m_levelIdc == Level::LEVEL15_5, "Cannot use level 15.5 with given profile");
    CHECK(sps->m_profileTierLevel.m_levelIdc != Level::NONE, "Unknown level");
    msg(WARNING, "Warning: Level set to none, invalid or unknown value\n");
  }
}

void DecLib::xParsePrefixSEIsForUnknownVCLNal()
{
  while (!m_prefixSEINALUs.empty())
  {
    // do nothing?
    msg(NOTICE, "Discarding Prefix SEI associated with unknown VCL NAL unit.\n");
    delete m_prefixSEINALUs.front();
    m_prefixSEINALUs.pop_front();
  }
  // TODO: discard following suffix SEIs as well?
}

void DecLib::xParsePrefixSEImessages()
{
  while (!m_prefixSEINALUs.empty())
  {
    InputNALUnit &nalu = *m_prefixSEINALUs.front();
    m_accessUnitSeiNalus.push_back(new InputNALUnit(nalu));
    m_accessUnitSeiTids.push_back(nalu.m_temporalId);
    const SPS *sps = m_parameterSetManager.getActiveSPS();
    const VPS *vps = m_parameterSetManager.getVPS(sps->m_vpsId);
    const bool seiMessageRead =
      m_seiReader.parseSEImessage(&(nalu.getBitstream()), m_SEIs, nalu.m_nalUnitType, nalu.m_nuhLayerId,
                                  nalu.m_temporalId, vps, sps, m_HRD, m_pDecodedSEIOutputStream);
#if JVET_S0257_DUMP_360SEI_MESSAGE
    m_seiCfgDump.write360SeiDump(m_decCfg.m_outputDecoded360SEIMessagesFilename, m_SEIs, sps);
#endif
    if (seiMessageRead)
    {
      m_accessUnitSeiPayLoadTypes.push_back(std::tuple<NalUnitType, int, SEI::PayloadType>(
        nalu.m_nalUnitType, nalu.m_nuhLayerId, m_SEIs.back()->payloadType()));
    }
    delete m_prefixSEINALUs.front();
    m_prefixSEINALUs.pop_front();
  }
  xCheckPrefixSEIMessages(m_SEIs);
  SEIMessages scalableNestingSEIs = getSeisByType(m_SEIs, SEI::PayloadType::SCALABLE_NESTING);
  if (scalableNestingSEIs.size())
  {
    SEIScalableNesting *nestedSei    = (SEIScalableNesting *)scalableNestingSEIs.front();
    SEIMessages         nestedSliSei = getSeisByType(nestedSei->m_nestedSEIs, SEI::PayloadType::SUBPICTURE_LEVEL_INFO);
    if (nestedSliSei.size() > 0)
    {
      AccessUnitNestedSliSeiInfo sliSeiInfo;
      sliSeiInfo.m_nestedSliPresent = true;
      sliSeiInfo.m_numOlssNestedSli = nestedSei->m_snNumOlssMinus1 + 1;
      for (uint32_t olsIdxNestedSei = 0; olsIdxNestedSei <= nestedSei->m_snNumOlssMinus1; olsIdxNestedSei++)
      {
        sliSeiInfo.m_olsIdxNestedSLI[olsIdxNestedSei] = nestedSei->m_snOlsIdx[olsIdxNestedSei];
      }
      m_accessUnitNestedSliSeiInfo.push_back(sliSeiInfo);
    }
  }
  xCheckDUISEIMessages(m_SEIs);
}

void DecLib::xCheckPrefixSEIMessages(SEIMessages &prefixSEIs)
{
  SEIMessages picTimingSEIs  = getSeisByType(prefixSEIs, SEI::PayloadType::PICTURE_TIMING);
  SEIMessages frameFieldSEIs = getSeisByType(prefixSEIs, SEI::PayloadType::FRAME_FIELD_INFO);

  if (!picTimingSEIs.empty() && !frameFieldSEIs.empty())
  {
    SEIPictureTiming  *pt = (SEIPictureTiming *)picTimingSEIs.front();
    SEIFrameFieldInfo *ff = (SEIFrameFieldInfo *)frameFieldSEIs.front();
    if (pt->m_ptDisplayElementalPeriodsMinus1 != ff->m_displayElementalPeriodsMinus1)
    {
      msg(WARNING,
          "Warning: ffi_display_elemental_periods_minus1 is different in picture timing and frame field information "
          "SEI messages!");
    }
  }
  if ((m_vps->m_maxLayers == 1 || m_audIrapOrGdrAuFlag) &&
      (m_isFirstAuInCvs || m_accessUnitPicInfo.begin()->m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
       m_accessUnitPicInfo.begin()->m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
       ((m_accessUnitPicInfo.begin()->m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA ||
         m_accessUnitPicInfo.begin()->m_nalUnitType == NAL_UNIT_CODED_SLICE_GDR) &&
        m_lastNoOutputBeforeRecoveryFlag[m_accessUnitPicInfo.begin()->m_nuhLayerId])) &&
      m_accessUnitPicInfo.size() == 1)
  {
    if (m_sdiSEIInFirstAU != nullptr)
    {
      delete m_sdiSEIInFirstAU;
    }
    m_sdiSEIInFirstAU = nullptr;
    if (m_maiSEIInFirstAU != nullptr)
    {
      delete m_maiSEIInFirstAU;
    }
    m_maiSEIInFirstAU = nullptr;
    if (m_mvpSEIInFirstAU != nullptr)
    {
      delete m_mvpSEIInFirstAU;
    }
    m_mvpSEIInFirstAU   = nullptr;
    SEIMessages sdiSEIs = getSeisByType(prefixSEIs, SEI::PayloadType::SCALABILITY_DIMENSION_INFO);
    if (!sdiSEIs.empty())
    {
      SEIScalabilityDimensionInfo *sdi = (SEIScalabilityDimensionInfo *)sdiSEIs.front();
      m_sdiSEIInFirstAU                = new SEIScalabilityDimensionInfo(*sdi);
      if (sdiSEIs.size() > 1)
      {
        for (SEIMessages::const_iterator it = sdiSEIs.begin(); it != sdiSEIs.end(); it++)
        {
          CHECK(!m_sdiSEIInFirstAU->isSDISameContent((SEIScalabilityDimensionInfo *)*it),
                "All SDI SEI messages in a CVS shall have the same content.")
        }
      }
    }
    SEIMessages maiSEIs = getSeisByType(prefixSEIs, SEI::PayloadType::MULTIVIEW_ACQUISITION_INFO);
    if (!maiSEIs.empty())
    {
      SEIMultiviewAcquisitionInfo *mai = (SEIMultiviewAcquisitionInfo *)maiSEIs.front();
      m_maiSEIInFirstAU                = new SEIMultiviewAcquisitionInfo(*mai);
      if (maiSEIs.size() > 1)
      {
        for (SEIMessages::const_iterator it = maiSEIs.begin(); it != maiSEIs.end(); it++)
        {
          CHECK(!m_maiSEIInFirstAU->isMAISameContent((SEIMultiviewAcquisitionInfo *)*it),
                "All MAI SEI messages in a CVS shall have the same content.")
        }
      }
    }
    SEIMessages mvpSEIs = getSeisByType(prefixSEIs, SEI::PayloadType::MULTIVIEW_VIEW_POSITION);
    if (!mvpSEIs.empty())
    {
      SEIMultiviewViewPosition *mvp = (SEIMultiviewViewPosition *)mvpSEIs.front();
      m_mvpSEIInFirstAU             = new SEIMultiviewViewPosition(*mvp);
      if (mvpSEIs.size() > 1)
      {
        for (SEIMessages::const_iterator it = mvpSEIs.begin(); it != mvpSEIs.end(); it++)
        {
          CHECK(!m_mvpSEIInFirstAU->isMVPSameContent((SEIMultiviewViewPosition *)*it),
                "All MVP SEI messages in a CVS shall have the same content.")
        }
      }
    }
  }
  else
  {
    SEIMessages sdiSEIs = getSeisByType(prefixSEIs, SEI::PayloadType::SCALABILITY_DIMENSION_INFO);
    CHECK(!m_sdiSEIInFirstAU && !sdiSEIs.empty(),
          "When an SDI SEI message is present in any AU of a CVS, an SDI SEI message shall be present for the first AU "
          "of the CVS.");
    if (!sdiSEIs.empty())
    {
      for (SEIMessages::const_iterator it = sdiSEIs.begin(); it != sdiSEIs.end(); it++)
      {
        CHECK(!m_sdiSEIInFirstAU->isSDISameContent((SEIScalabilityDimensionInfo *)*it),
              "All SDI SEI messages in a CVS shall have the same content.")
      }
    }
    SEIMessages maiSEIs = getSeisByType(prefixSEIs, SEI::PayloadType::MULTIVIEW_ACQUISITION_INFO);
    CHECK(!m_maiSEIInFirstAU && !maiSEIs.empty(),
          "When an MAI SEI message is present in any AU of a CVS, an MAI SEI message shall be present for the first AU "
          "of the CVS.");
    if (!maiSEIs.empty())
    {
      for (SEIMessages::const_iterator it = maiSEIs.begin(); it != maiSEIs.end(); it++)
      {
        CHECK(!m_maiSEIInFirstAU->isMAISameContent((SEIMultiviewAcquisitionInfo *)*it),
              "All MAI SEI messages in a CVS shall have the same content.")
      }
    }
    SEIMessages mvpSEIs = getSeisByType(prefixSEIs, SEI::PayloadType::MULTIVIEW_VIEW_POSITION);
    CHECK(!m_mvpSEIInFirstAU && !mvpSEIs.empty(),
          "When an MVP SEI message is present in any AU of a CVS, an MVP SEI message shall be present for the first AU "
          "of the CVS.");
    if (!mvpSEIs.empty())
    {
      for (SEIMessages::const_iterator it = mvpSEIs.begin(); it != mvpSEIs.end(); it++)
      {
        CHECK(!m_mvpSEIInFirstAU->isMVPSameContent((SEIMultiviewViewPosition *)*it),
              "All MVP SEI messages in a CVS shall have the same content.")
      }
    }
  }

  for (SEIMessages::const_iterator it = prefixSEIs.begin(); it != prefixSEIs.end(); it++)
  {
    if ((*it)->payloadType() == SEI::PayloadType::MULTIVIEW_ACQUISITION_INFO)
    {
      CHECK(!m_sdiSEIInFirstAU,
            "When a CVS does not contain an SDI SEI message, the CVS shall not contain an MAI SEI message.");
      SEIMultiviewAcquisitionInfo *maiSei = (SEIMultiviewAcquisitionInfo *)*it;
      CHECK(m_sdiSEIInFirstAU->m_sdiNumViews - 1 != maiSei->m_maiNumViewsMinus1,
            "The value of num_views_minus1 shall be equal to NumViews - 1");
    }
    else if ((*it)->payloadType() == SEI::PayloadType::ALPHA_CHANNEL_INFO)
    {
      CHECK(!m_sdiSEIInFirstAU,
            "When a CVS does not contain an SDI SEI message with sdi_aux_id[i] equal to 1 for at least one value of i, "
            "no picture in the CVS shall be associated with an ACI SEI message.");
    }
    else if ((*it)->payloadType() == SEI::PayloadType::DEPTH_REPRESENTATION_INFO)
    {
      CHECK(!m_sdiSEIInFirstAU,
            "When a CVS does not contain an SDI SEI message with sdi_aux_id[i] equal to 2 for at least one value of i, "
            "no picture in the CVS shall be associated with a DRI SEI message.");
    }
    else if ((*it)->payloadType() == SEI::PayloadType::MULTIVIEW_VIEW_POSITION)
    {
      CHECK(!m_sdiSEIInFirstAU,
            "When a CVS does not contain an SDI SEI message, the CVS shall not contain an MVP SEI message.");
      SEIMultiviewViewPosition *mvpSei = (SEIMultiviewViewPosition *)*it;
      CHECK(m_sdiSEIInFirstAU->m_sdiNumViews - 1 != mvpSei->m_mvpNumViewsMinus1,
            "The value of num_views_minus1 shall be equal to NumViews - 1");
    }
  }
}

void DecLib::xCheckDUISEIMessages(SEIMessages &prefixSEIs)
{
  SEIMessages BPSEIs  = getSeisByType(prefixSEIs, SEI::PayloadType::BUFFERING_PERIOD);
  SEIMessages DUISEIs = getSeisByType(prefixSEIs, SEI::PayloadType::DECODING_UNIT_INFO);
  if (BPSEIs.empty())
  {
    return;
  }
  else
  {
    bool duDelayFlag = false;

    SEIBufferingPeriod *bp = (SEIBufferingPeriod *)BPSEIs.front();
    if (bp->m_bpDecodingUnitHrdParamsPresentFlag)
    {
      if (!bp->m_decodingUnitDpbDuParamsInPicTimingSeiFlag)
      {
        if (DUISEIs.empty())
        {
          return;
        }
        for (auto it = DUISEIs.cbegin(); it != DUISEIs.cend(); ++it)
        {
          const SEIDecodingUnitInfo *dui = (const SEIDecodingUnitInfo *)*it;
          if (dui->m_picSptDpbOutputDuDelay != -1)
          {
            duDelayFlag = true;
            break;
          }
        }
        CHECK(duDelayFlag == false, "At least one DUI SEI should have dui->m_picSptDpbOutputDuDelay not equal to -1")
      }
    }
  }
}

void DecLib::xDecodePicHeader(InputNALUnit &nalu)
{
  m_HLSReader.setBitstream(&nalu.getBitstream());
  m_HLSReader.parsePictureHeader(&m_picHeader, &m_parameterSetManager, true);
  m_picHeader.m_valid = true;
}

bool DecLib::getMixedNaluTypesInPicFlag()
{
  if (!m_picHeader.m_valid)
  {
    return false;
  }

  PPS *pps = m_parameterSetManager.getPPS(m_picHeader.m_ppsId);
  CHECK(pps == 0, "No PPS present");

  return pps->m_mixedNaluTypesInPicFlag;
}

bool DecLib::xDecodeSlice(InputNALUnit &nalu, int &iSkipFrame, int iPOCLastDisplay)
{
  PROFILER_SCOPE(1, g_timeProfiler, P_DECODE_SLICE);
  m_apcSlicePilot->m_picHeader = &m_picHeader;
  m_apcSlicePilot->initSlice();   // the slice pilot is an object to prepare for a new slice
                                  // it is not associated with picture, sps or pps structures.

  Picture *scaledRefPic[MAX_NUM_REF] = {};

  if (m_isFirstSliceInPicture)
  {
    m_uiSliceSegmentIdx = 0;
  }
  else
  {
    CHECK(nalu.m_nalUnitType != m_pic->m_slices[m_uiSliceSegmentIdx - 1]->m_eNalUnitType &&
            !m_pic->m_cs->pps->m_mixedNaluTypesInPicFlag,
          "If pps_mixed_nalu_types_in_pic_flag is equal to 0, the value of NAL unit type shall be the same for all "
          "coded slice NAL units of a picture");
    m_apcSlicePilot->copySliceInfo(m_pic->m_slices[m_uiSliceSegmentIdx - 1]);
  }

  m_apcSlicePilot->m_eNalUnitType = nalu.m_nalUnitType;
  m_apcSlicePilot->m_nuhLayerId   = nalu.m_nuhLayerId;
  m_apcSlicePilot->m_uiTLayer     = nalu.m_temporalId;

  for (auto &naluTemporalId: m_accessUnitNals)
  {
    if (naluTemporalId.m_nalUnitType != NAL_UNIT_OPI && naluTemporalId.m_nalUnitType != NAL_UNIT_DCI &&
        naluTemporalId.m_nalUnitType != NAL_UNIT_VPS && naluTemporalId.m_nalUnitType != NAL_UNIT_SPS &&
        naluTemporalId.m_nalUnitType != NAL_UNIT_EOS && naluTemporalId.m_nalUnitType != NAL_UNIT_EOB)

    {
      CHECK(
        naluTemporalId.m_temporalId < nalu.m_temporalId,
        "TemporalId shall be greater than or equal to the TemporalId of the layer access unit containing the NAL unit");
    }
  }

  if (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_GDR)
  {
    CHECK(nalu.m_temporalId != 0, "Current GDR picture has TemporalId not equal to 0");
  }

  m_HLSReader.setBitstream(&nalu.getBitstream());
  m_apcSlicePilot->m_ccSaoComParam = m_cSAO.getCcSaoComParam();
  if (m_alfEcm != nullptr)
  {
    m_apcSlicePilot->m_ccAlfFilterParam.getEcmParam() = m_alfEcm->getCcAlfFilterParam();
  }
  else
  {
    m_apcSlicePilot->m_ccAlfFilterParam.getVtmParam() = m_alfVtm->getCcAlfFilterParam();
  }
  m_HLSReader.parseSliceHeader(m_apcSlicePilot, &m_picHeader, &m_parameterSetManager, m_prevTid0POC, m_prevPicPOC);

  if (m_picHeader.m_gdrOrIrapPicFlag && m_isFirstSliceInPicture)
  {
    m_accessUnitNoOutputPriorPicFlags.push_back(m_apcSlicePilot->m_noOutputOfPriorPicsFlag);
  }

  if (m_picHeader.m_gdrPicFlag &&
      m_prevGDRInSameLayerPOC[nalu.m_nuhLayerId] ==
        -MAX_INT)   // Only care about recovery POC if it is the first coded GDR picture in the layer
  {
    m_prevGDRInSameLayerRecoveryPOC[nalu.m_nuhLayerId] = m_apcSlicePilot->m_poc + m_picHeader.m_recoveryPocCnt;
  }

  PPS *pps = m_parameterSetManager.getPPS(m_picHeader.m_ppsId);
  CHECK(pps == 0, "No PPS present");
  SPS *sps = m_parameterSetManager.getSPS(pps->m_spsId);
  CHECK(sps == 0, "No SPS present");
  VPS *vps = m_parameterSetManager.getVPS(sps->m_vpsId);

  if (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA && vps != nullptr &&
      (vps->m_vpsIndependentLayerFlag[vps->m_generalLayerIdx[nalu.m_nuhLayerId]] == 1))
  {
    CHECK(nalu.m_temporalId == 0, "TemporalID of STSA picture shall not be zero in independent layers");
  }

  int currSubPicIdx = pps->getSubPicIdxFromSubPicId(m_apcSlicePilot->m_sliceSubPicId);
  int currSliceAddr = m_apcSlicePilot->getSliceID();
  for (int sp = 0; sp < currSubPicIdx; sp++)
  {
    currSliceAddr -= pps->m_subPics[sp].m_numSlicesInSubPic;
  }
  CHECK(currSubPicIdx < m_maxDecSubPicIdx, "Error in the order of coded slice NAL units of subpictures");
  CHECK(currSubPicIdx == m_maxDecSubPicIdx && currSliceAddr <= m_maxDecSliceAddrInSubPic,
        "Error in the order of coded slice NAL units within a subpicture");
  if (currSubPicIdx == m_maxDecSubPicIdx)
  {
    m_maxDecSliceAddrInSubPic = currSliceAddr;
  }
  if (currSubPicIdx > m_maxDecSubPicIdx)
  {
    m_maxDecSubPicIdx         = currSubPicIdx;
    m_maxDecSliceAddrInSubPic = currSliceAddr;
  }
  if ((sps->m_vpsId == 0) && (m_prevLayerID != MAX_INT))
  {
    CHECK(m_prevLayerID != nalu.m_nuhLayerId,
          "All VCL NAL unit in the CVS shall have the same value of nuh_layer_id "
          "when sps_video_parameter_set_id is equal to 0");
  }
  CHECK((sps->m_vpsId > 0) && (vps == 0), "Invalid VPS");

  const ProfileTierLevel &profileTierLevel = (vps == nullptr || vps->m_numLayersInOls[vps->m_targetOlsIdx] == 1)
    ? sps->m_profileTierLevel
    : vps->m_vpsProfileTierLevel[vps->m_olsPtlIdx[vps->m_targetOlsIdx]];

  if ((profileTierLevel.m_multiLayerEnabledFlag == 0) && (m_prevLayerID != MAX_INT))
  {
    CHECK(m_prevLayerID != nalu.m_nuhLayerId,
          "All slices in OlsInScope shall have the same value of nuh_layer_id when ptl_multilayer_enabled_flag is "
          "equal to 0");
  }

  if (vps != nullptr && !vps->m_vpsIndependentLayerFlag[vps->m_generalLayerIdx[nalu.m_nuhLayerId]])
  {
    bool pocIsSet = false;
    for (auto auNALit = m_accessUnitPicInfo.begin(); auNALit != m_accessUnitPicInfo.end(); auNALit++)
    {
      for (int refIdx = 0; refIdx < m_apcSlicePilot->m_numRefIdx[RPL0] && !pocIsSet; refIdx++)
      {
        if (m_apcSlicePilot->getRefPic(RPL0, refIdx) &&
            m_apcSlicePilot->getRefPic(RPL0, refIdx)->m_poc == (*auNALit).m_POC)
        {
          m_apcSlicePilot->m_poc = m_apcSlicePilot->getRefPic(RPL0, refIdx)->m_poc;
          pocIsSet               = true;
        }
      }
      for (int refIdx = 0; refIdx < m_apcSlicePilot->m_numRefIdx[RPL1] && !pocIsSet; refIdx++)
      {
        if (m_apcSlicePilot->getRefPic(RPL1, refIdx) &&
            m_apcSlicePilot->getRefPic(RPL1, refIdx)->m_poc == (*auNALit).m_POC)
        {
          m_apcSlicePilot->m_poc = m_apcSlicePilot->getRefPic(RPL1, refIdx)->m_poc;
          pocIsSet               = true;
        }
      }
    }
  }

  // update independent slice index
  uint32_t uiIndependentSliceIdx = 0;
  if (!m_isFirstSliceInPicture)
  {
    uiIndependentSliceIdx = m_pic->m_slices[m_uiSliceSegmentIdx - 1]->m_independentSliceIdx;
    uiIndependentSliceIdx++;
  }
  m_apcSlicePilot->m_independentSliceIdx = uiIndependentSliceIdx;

#if K0149_BLOCK_STATISTICS
  writeBlockStatisticsHeader(sps);
#endif

  DTRACE_UPDATE(g_trace_ctx, std::make_pair("poc", m_apcSlicePilot->m_poc));

  xUpdatePreviousTid0POC(m_apcSlicePilot);

  m_apcSlicePilot->m_prevGDRInSameLayerPOC = m_prevGDRInSameLayerPOC[nalu.m_nuhLayerId];
  m_apcSlicePilot->m_iAssociatedIRAPPOC    = m_pocCRA[nalu.m_nuhLayerId];
  m_apcSlicePilot->m_iAssociatedIRAPType   = m_associatedIRAPType[nalu.m_nuhLayerId];

  if (m_apcSlicePilot->getRapPicFlag() || m_apcSlicePilot->m_eNalUnitType == NAL_UNIT_CODED_SLICE_GDR)
  {
    // Derive NoOutputBeforeRecoveryFlag
    if (!pps->m_mixedNaluTypesInPicFlag)
    {
      if (m_firstSliceInSequence[nalu.m_nuhLayerId])
      {
        m_picHeader.m_noOutputBeforeRecoveryFlag = true;
      }
      else if (m_apcSlicePilot->getIdrPicFlag())
      {
        m_picHeader.m_noOutputBeforeRecoveryFlag = true;
      }
      else if (m_apcSlicePilot->m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA)
      {
        m_picHeader.m_noOutputBeforeRecoveryFlag = m_picHeader.m_handleCraAsCvsStartFlag;
      }
      else if (m_apcSlicePilot->m_eNalUnitType == NAL_UNIT_CODED_SLICE_GDR)
      {
        m_picHeader.m_noOutputBeforeRecoveryFlag = m_picHeader.m_handleGdrAsCvsStartFlag;
      }
    }
    else
    {
      m_picHeader.m_noOutputBeforeRecoveryFlag = false;
    }

    if (m_apcSlicePilot->m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA ||
        m_apcSlicePilot->m_eNalUnitType == NAL_UNIT_CODED_SLICE_GDR)
    {
      m_lastNoOutputBeforeRecoveryFlag[nalu.m_nuhLayerId] = m_picHeader.m_noOutputBeforeRecoveryFlag;
    }

    if (m_apcSlicePilot->m_noOutputOfPriorPicsFlag)
    {
      m_lastPOCNoOutputPriorPics = m_apcSlicePilot->m_poc;
      m_isNoOutputPriorPics      = true;
    }
    else
    {
      m_isNoOutputPriorPics = false;
    }
  }

  if (m_isFirstSliceInPicture && m_apcSlicePilot->m_poc != m_prevPOC &&
      (m_apcSlicePilot->getRapPicFlag() || m_apcSlicePilot->m_eNalUnitType == NAL_UNIT_CODED_SLICE_GDR) &&
      m_picHeader.m_noOutputBeforeRecoveryFlag && getNoOutputPriorPicsFlag())
  {
    checkNoOutputPriorPics(&m_cListPic);
    setNoOutputPriorPicsFlag(false);
  }

  // For inference of PicOutputFlag
  if (!pps->m_mixedNaluTypesInPicFlag && (m_apcSlicePilot->m_eNalUnitType == NAL_UNIT_CODED_SLICE_RASL))
  {
    if (m_lastNoOutputBeforeRecoveryFlag[nalu.m_nuhLayerId])
    {
      m_picHeader.m_picOutputFlag = false;
    }
  }

  {
    PPS *pps = m_parameterSetManager.getPPS(m_picHeader.m_ppsId);
    CHECK(pps == 0, "No PPS present");
    SPS *sps = m_parameterSetManager.getSPS(pps->m_spsId);
    CHECK(sps == 0, "No SPS present");
    if (sps->m_vpsId > 0)
    {
      VPS *vps = m_parameterSetManager.getVPS(sps->m_vpsId);
      CHECK(vps == 0, "No VPS present");
      bool isCurLayerNotOutput = true;
      for (int i = 0; i < vps->m_numLayersInOls[vps->m_targetOlsIdx]; i++)
      {
        if (vps->m_layerIdInOls[vps->m_targetOlsIdx][i] == nalu.m_nuhLayerId)
        {
          isCurLayerNotOutput = false;
          break;
        }
      }

      if (isCurLayerNotOutput)
      {
        m_picHeader.m_picOutputFlag = false;
      }
    }
  }

  // Reset POC MSB when CRA or GDR has NoOutputBeforeRecoveryFlag equal to 1
  if (!pps->m_mixedNaluTypesInPicFlag &&
      (m_apcSlicePilot->m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA ||
       m_apcSlicePilot->m_eNalUnitType == NAL_UNIT_CODED_SLICE_GDR) &&
      m_lastNoOutputBeforeRecoveryFlag[nalu.m_nuhLayerId])
  {
    int iMaxPOClsb             = 1 << sps->m_bitsForPoc;
    m_apcSlicePilot->m_poc     = m_apcSlicePilot->m_poc & (iMaxPOClsb - 1);
    m_lastPOCNoOutputPriorPics = m_apcSlicePilot->m_poc;
    xUpdatePreviousTid0POC(m_apcSlicePilot);
  }

  AccessUnitPicInfo picInfo;
  picInfo.m_nalUnitType = nalu.m_nalUnitType;
  picInfo.m_nuhLayerId  = nalu.m_nuhLayerId;
  picInfo.m_temporalId  = nalu.m_temporalId;
  picInfo.m_POC         = m_apcSlicePilot->m_poc;
  m_accessUnitPicInfo.push_back(picInfo);

  // Skip pictures due to random access

  if (isRandomAccessSkipPicture(iSkipFrame, iPOCLastDisplay, pps->m_mixedNaluTypesInPicFlag, nalu.m_nuhLayerId))
  {
    m_prevSliceSkipped = true;
    m_skippedPOC       = m_apcSlicePilot->m_poc;
    m_skippedLayerID   = nalu.m_nuhLayerId;

    // reset variables for bitstream conformance tests
    resetAccessUnitNals();
    resetAccessUnitApsNals();
    resetAccessUnitPicInfo();
    resetPictureUnitNals();
    resetPrefixSeiNalus();
    m_maxDecSubPicIdx         = 0;
    m_maxDecSliceAddrInSubPic = -1;
    return false;
  }
  // Skip TFD pictures associated with BLA/BLANT pictures

  // clear previous slice skipped flag
  m_prevSliceSkipped = false;

  // we should only get a different poc for a new picture (with CTU address==0)
  if (m_apcSlicePilot->m_poc != m_prevPOC && !m_firstSliceInSequence[nalu.m_nuhLayerId] &&
      (m_apcSlicePilot->getFirstCtuRsAddrInSlice() != 0))
  {
    msg(WARNING, "Warning, the first slice of a picture might have been lost!\n");
  }
  m_prevLayerID = nalu.m_nuhLayerId;

  // leave when a new picture is found
  if (m_apcSlicePilot->getFirstCtuRsAddrInSlice() == 0 && !m_isFirstSliceInPicture)
  {
    if (m_prevPOC >= m_pocRandomAccess)
    {
      DTRACE_UPDATE(g_trace_ctx, std::make_pair("final", 0));
      m_prevPOC = m_apcSlicePilot->m_poc;
      return true;
    }
    m_prevPOC = m_apcSlicePilot->m_poc;
  }
  else
  {
    DTRACE_UPDATE(g_trace_ctx, std::make_pair("final", 1));
  }

  // detect lost reference picture and insert copy of earlier frame.
  {
    int lostPoc;
    int refPicIndex;
    for (const auto l: { RPL0, RPL1 })
    {
      const ReferencePictureList *rpl = &m_apcSlicePilot->m_rpl[l];

      while ((lostPoc = m_apcSlicePilot->checkThatAllRefPicsAreAvailable(m_cListPic, rpl, 0, true, &refPicIndex,
                                                                         m_apcSlicePilot->m_numRefIdx[l])) > 0)
      {
        if (!pps->m_mixedNaluTypesInPicFlag &&
            ((m_apcSlicePilot->isIDRorBLA() && (sps->m_idrRefParamList || pps->m_rplInfoInPhFlag)) ||
             ((m_apcSlicePilot->m_eNalUnitType == NAL_UNIT_CODED_SLICE_GDR ||
               m_apcSlicePilot->m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA) &&
              m_picHeader.m_noOutputBeforeRecoveryFlag)))
        {
          if (!rpl->m_isInterLayerRefPic[refPicIndex])
          {
            xCreateUnavailablePicture(pps, lostPoc, rpl->m_isLongtermRefPic[refPicIndex], m_apcSlicePilot->m_uiTLayer,
                                      m_apcSlicePilot->m_nuhLayerId, rpl->m_isInterLayerRefPic[refPicIndex]);
          }
        }
        else
        {
          xCreateLostPicture(lostPoc - 1, m_apcSlicePilot->m_pic->m_layerId);
        }
      }
    }
  }

  m_prevPOC = m_apcSlicePilot->m_poc;

  if (m_isFirstSliceInPicture)
  {
    xUpdateRasInit(m_apcSlicePilot);
    m_cSAO.loadOrStoreCCSaoTemporalPredictor(m_apcSlicePilot, m_apcSlicePilot->m_ccSaoComParam);
  }

  // actual decoding starts here
  xActivateParameterSets(nalu);

  m_firstSliceInSequence[nalu.m_nuhLayerId] = false;
  m_firstSliceInBitstream                   = false;

  Slice *pcSlice                                = m_pic->m_slices[m_uiSliceSegmentIdx];
  m_pic->m_numSlices                            = m_uiSliceSegmentIdx + 1;
  pcSlice->m_pic                                = m_pic;
  m_pic->m_poc                                  = pcSlice->m_poc;
  m_pic->m_referenced                           = true;
  m_pic->m_temporalId                           = nalu.m_temporalId;
  m_pic->m_layerId                              = nalu.m_nuhLayerId;
  m_pic->m_subLayerNonReferencePictureDueToSTSA = false;

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

  if (pcSlice->m_sps->m_profileTierLevel.m_constraintInfo.m_noApsConstraintFlag)
  {
    bool flag = pcSlice->m_sps->m_ccalfEnabledFlag || pcSlice->m_picHeader->m_numAlfApsIdsLuma ||
      pcSlice->m_picHeader->m_alfEnabledFlag[COMP_Cb] || pcSlice->m_picHeader->m_alfEnabledFlag[COMP_Cr];
    CHECK(flag,
          "When no_aps_constraint_flag is equal to 1, the values of ph_num_alf_aps_ids_luma, sh_num_alf_aps_ids_luma, "
          "ph_alf_cb_flag, ph_alf_cr_flag, sh_alf_cb_flag, sh_alf_cr_flag, and sps_ccalf_enabled_flag shall all be "
          "equal to 0")
  }
  if (pcSlice->m_nuhLayerId != pcSlice->m_sps->m_layerId)
  {
    CHECK(pcSlice->m_sps->m_layerId > pcSlice->m_nuhLayerId,
          "Layer Id of SPS cannot be greater than layer Id of VCL NAL unit the refer to it");
    CHECK(pcSlice->m_sps->m_vpsId == 0,
          "VPSId of the referred SPS cannot be 0 when layer Id of SPS and layer Id of current slice are different");
    for (int i = 0; i < pcSlice->m_vps->m_vpsNumOutputLayerSets; i++)
    {
      bool isCurrLayerInOls = false;
      bool isRefLayerInOls  = false;
      int  j                = pcSlice->m_vps->m_numLayersInOls[i] - 1;
      for (; j >= 0; j--)
      {
        if (pcSlice->m_vps->m_layerIdInOls[i][j] == pcSlice->m_nuhLayerId)
        {
          isCurrLayerInOls = true;
        }
        if (pcSlice->m_vps->m_layerIdInOls[i][j] == pcSlice->m_sps->m_layerId)
        {
          isRefLayerInOls = true;
        }
      }
      CHECK(isCurrLayerInOls && !isRefLayerInOls,
            "When VCL NAl unit in layer A refers to SPS in layer B, all OLS that contains layer A shall also contains "
            "layer B");
    }
  }
  if (pcSlice->m_nuhLayerId != pcSlice->m_pps->m_layerId)
  {
    CHECK(pcSlice->m_pps->m_layerId > pcSlice->m_nuhLayerId,
          "Layer Id of PPS cannot be greater than layer Id of VCL NAL unit the refer to it");
    CHECK(pcSlice->m_sps->m_vpsId == 0,
          "VPSId of the referred SPS cannot be 0 when layer Id of PPS and layer Id of current slice are different");
    for (int i = 0; i < pcSlice->m_vps->m_vpsNumOutputLayerSets; i++)
    {
      bool isCurrLayerInOls = false;
      bool isRefLayerInOls  = false;
      int  j                = pcSlice->m_vps->m_numLayersInOls[i] - 1;
      for (; j >= 0; j--)
      {
        if (pcSlice->m_vps->m_layerIdInOls[i][j] == pcSlice->m_nuhLayerId)
        {
          isCurrLayerInOls = true;
        }
        if (pcSlice->m_vps->m_layerIdInOls[i][j] == pcSlice->m_pps->m_layerId)
        {
          isRefLayerInOls = true;
        }
      }
      CHECK(isCurrLayerInOls && !isRefLayerInOls,
            "When VCL NAl unit in layer A refers to PPS in layer B, all OLS that contains layer A shall also contains "
            "layer B");
    }
  }

  if (m_isFirstSliceInPicture)
  {
    m_pic->m_decodingOrderNumber = m_decodingOrderCounter;
    m_decodingOrderCounter++;
    m_pic->setPictureType(nalu.m_nalUnitType);
    checkPicTypeAfterEos();
    // store sub-picture numbers, sizes, and locations with a picture
    pcSlice->m_pic->m_subPictures.clear();

    for (int subPicIdx = 0; subPicIdx < sps->m_numSubPics; subPicIdx++)
    {
      pcSlice->m_pic->m_subPictures.push_back(pps->m_subPics[subPicIdx]);
    }
    pcSlice->m_pic->m_numSlices = pps->m_numSlicesInPic;
    pcSlice->m_pic->m_sliceSubpicIdx.clear();
  }
  pcSlice->m_pic->m_sliceSubpicIdx.push_back(pps->getSubPicIdxFromSubPicId(pcSlice->m_sliceSubPicId));
  pcSlice->checkCRA(&pcSlice->m_rpl[RPL0], &pcSlice->m_rpl[RPL1], m_pocCRA[nalu.m_nuhLayerId],
                    m_checkCRAFlags[nalu.m_nuhLayerId], m_cListPic);
  pcSlice->constructRefPicList(m_cListPic);
  pcSlice->m_prevGDRSubpicPOC   = m_prevGDRSubpicPOC[nalu.m_nuhLayerId][currSubPicIdx];
  pcSlice->m_prevIRAPSubpicPOC  = m_prevIRAPSubpicPOC[nalu.m_nuhLayerId][currSubPicIdx];
  pcSlice->m_prevIRAPSubpicType = m_prevIRAPSubpicType[nalu.m_nuhLayerId][currSubPicIdx];
  pcSlice->checkSubpicTypeConstraints(m_cListPic, &pcSlice->m_rpl[RPL0], &pcSlice->m_rpl[RPL1],
                                      m_prevIRAPSubpicDecOrderNo[nalu.m_nuhLayerId][currSubPicIdx]);
  pcSlice->checkRPL(&pcSlice->m_rpl[RPL0], &pcSlice->m_rpl[RPL1],
                    m_associatedIRAPDecodingOrderNumber[nalu.m_nuhLayerId], m_cListPic);
  pcSlice->checkSTSA(m_cListPic);
  if (m_pic->m_cs->vps &&
      !m_pic->m_cs->vps->m_vpsIndependentLayerFlag[m_pic->m_cs->vps->m_generalLayerIdx[nalu.m_nuhLayerId]] &&
      m_pic->m_cs->pps->m_numSubPics > 1)
  {
    CU::checkConformanceILRP(pcSlice);
  }

  bool bDisableTMVP = pcSlice->scaleRefPicList(scaledRefPic, m_pic->m_cs->picHeader, m_parameterSetManager.getAPSs(),
                                               m_picHeader.m_lmcsAps, m_picHeader.m_scalingListAps, true);
  if (m_pic->m_cs->picHeader->m_enableTMVPFlag && bDisableTMVP)
  {
    m_pic->m_cs->picHeader->m_enableTMVPFlag = false;
  }

  if (!pcSlice->isIntra())
  {
    bool lowDelay = true;
    int  currPoc  = pcSlice->m_poc;
    int  refIdx   = 0;

    for (refIdx = 0; refIdx < pcSlice->m_numRefIdx[RPL0] && lowDelay; refIdx++)
    {
      if (pcSlice->getRefPic(RPL0, refIdx)->m_poc > currPoc)
      {
        lowDelay = false;
      }
    }
    if (pcSlice->isInterB())
    {
      for (refIdx = 0; refIdx < pcSlice->m_numRefIdx[RPL1] && lowDelay; refIdx++)
      {
        if (pcSlice->getRefPic(RPL1, refIdx)->m_poc > currPoc)
        {
          lowDelay = false;
        }
      }
    }

    pcSlice->m_checkLdc = lowDelay;
  }

  reconstructClipRange(pcSlice);

  if (pcSlice->m_sps->m_useSMVD && pcSlice->m_checkLdc == false && pcSlice->m_picHeader->m_mvdL1ZeroFlag == false)
  {
    int currPOC = pcSlice->m_poc;

    int forwardPOC  = currPOC;
    int backwardPOC = currPOC;
    int ref         = 0;
    int refIdx0     = -1;
    int refIdx1     = -1;

    // search nearest forward POC in List 0
    for (ref = 0; ref < pcSlice->m_numRefIdx[RPL0]; ref++)
    {
      int        poc           = pcSlice->getRefPic(RPL0, ref)->m_poc;
      const bool isRefLongTerm = pcSlice->getRefPic(RPL0, ref)->m_longTerm;
      if (poc < currPOC && (poc > forwardPOC || refIdx0 == -1) && !isRefLongTerm)
      {
        forwardPOC = poc;
        refIdx0    = ref;
      }
    }

    // search nearest backward POC in List 1
    for (ref = 0; ref < pcSlice->m_numRefIdx[RPL1]; ref++)
    {
      int        poc           = pcSlice->getRefPic(RPL1, ref)->m_poc;
      const bool isRefLongTerm = pcSlice->getRefPic(RPL1, ref)->m_longTerm;
      if (poc > currPOC && (poc < backwardPOC || refIdx1 == -1) && !isRefLongTerm)
      {
        backwardPOC = poc;
        refIdx1     = ref;
      }
    }

    if (!(forwardPOC < currPOC && backwardPOC > currPOC))
    {
      forwardPOC  = currPOC;
      backwardPOC = currPOC;
      refIdx0     = -1;
      refIdx1     = -1;

      // search nearest backward POC in List 0
      for (ref = 0; ref < pcSlice->m_numRefIdx[RPL0]; ref++)
      {
        int        poc           = pcSlice->getRefPic(RPL0, ref)->m_poc;
        const bool isRefLongTerm = pcSlice->getRefPic(RPL0, ref)->m_longTerm;
        if (poc > currPOC && (poc < backwardPOC || refIdx0 == -1) && !isRefLongTerm)
        {
          backwardPOC = poc;
          refIdx0     = ref;
        }
      }

      // search nearest forward POC in List 1
      for (ref = 0; ref < pcSlice->m_numRefIdx[RPL1]; ref++)
      {
        int        poc           = pcSlice->getRefPic(RPL1, ref)->m_poc;
        const bool isRefLongTerm = pcSlice->getRefPic(RPL1, ref)->m_longTerm;
        if (poc < currPOC && (poc > forwardPOC || refIdx1 == -1) && !isRefLongTerm)
        {
          forwardPOC = poc;
          refIdx1    = ref;
        }
      }
    }

    if (forwardPOC < currPOC && backwardPOC > currPOC)
    {
      pcSlice->setBiDirPred(true, refIdx0, refIdx1);
    }
    else
    {
      pcSlice->setBiDirPred(false, -1, -1);
    }
  }
  else
  {
    pcSlice->setBiDirPred(false, -1, -1);
  }

  //---------------
  pcSlice->setRefPOCList();
  if (pcSlice->m_picHeader->m_enableTMVPFlag)
  {
    pcSlice->setRefRefIdxList();
  }

  NalUnitInfo naluInfo;
  naluInfo.m_nalUnitType     = nalu.m_nalUnitType;
  naluInfo.m_nuhLayerId      = nalu.m_nuhLayerId;
  naluInfo.m_firstCTUinSlice = pcSlice->getFirstCtuRsAddrInSlice();
  naluInfo.m_POC             = pcSlice->m_poc;
  xCheckMixedNalUnit(pcSlice, sps, nalu);
  m_nalUnitInfo[naluInfo.m_nuhLayerId].push_back(naluInfo);
  SEIMessages drapSEIs = getSeisByType(m_pic->m_SEIs, SEI::PayloadType::DEPENDENT_RAP_INDICATION);
  if (!drapSEIs.empty())
  {
    msg(NOTICE, "Dependent RAP indication SEI decoded\n");
    m_latestDRAPPOC   = pcSlice->m_poc;
    pcSlice->m_isDRAP = true;
  }
  pcSlice->m_latestDRAPPOC = m_latestDRAPPOC;
  pcSlice->checkConformanceForDRAP(nalu.m_temporalId);
  if (pcSlice->isIntra())
  {
    pcSlice->m_pic->m_edrapRapId = 0;
  }
  SEIMessages edrapSEIs = getSeisByType(m_pic->m_SEIs, SEI::PayloadType::EXTENDED_DRAP_INDICATION);
  if (!edrapSEIs.empty())
  {
    msg(NOTICE, "Extended DRAP indication SEI decoded\n");
    SEIExtendedDrapIndication *seiEdrap = (SEIExtendedDrapIndication *)edrapSEIs.front();
    pcSlice->m_edrapRapId               = seiEdrap->m_edrapIndicationRapIdMinus1 + 1;
    pcSlice->m_pic->m_edrapRapId        = (seiEdrap->m_edrapIndicationRapIdMinus1 + 1);
    pcSlice->m_edrapNumRefRapPics       = seiEdrap->m_edrapIndicationNumRefRapPicsMinus1 + 1;
    for (int i = 0; i < pcSlice->m_edrapNumRefRapPics; i++)
    {
      pcSlice->addEdrapRefRapIds(seiEdrap->m_edrapIndicationRefRapId[i]);
    }
    m_latestEDRAPIndicationLeadingPicturesDecodableFlag = seiEdrap->m_edrapIndicationLeadingPicturesDecodableFlag;
    m_latestEDRAPPOC                                    = pcSlice->m_poc;
  }
  pcSlice->m_latestEDRAPPOC                     = m_latestEDRAPPOC;
  pcSlice->m_latestEdrapLeadingPicDecodableFlag = m_latestEDRAPIndicationLeadingPicturesDecodableFlag;
  pcSlice->checkConformanceForEDRAP(nalu.m_temporalId);

  Quant *quant = m_cTrQuant.getQuant();

  if (pcSlice->m_explicitScalingListUsed)
  {
    APS *scalingListAPS = pcSlice->m_picHeader->m_scalingListAps;
    if (pcSlice->m_nuhLayerId != scalingListAPS->m_layerId)
    {
      CHECK(scalingListAPS->m_layerId > pcSlice->m_nuhLayerId,
            "Layer Id of APS cannot be greater than layer Id of VCL NAL unit the refer to it");
      CHECK(pcSlice->m_sps->m_vpsId == 0,
            "VPSId of the referred SPS cannot be 0 when layer Id of APS and layer Id of current slice are different");
      for (int i = 0; i < pcSlice->m_vps->m_vpsNumOutputLayerSets; i++)
      {
        bool isCurrLayerInOls = false;
        bool isRefLayerInOls  = false;
        for (int j = pcSlice->m_vps->m_numLayersInOls[i] - 1; j >= 0; j--)
        {
          if (pcSlice->m_vps->m_layerIdInOls[i][j] == pcSlice->m_nuhLayerId)
          {
            isCurrLayerInOls = true;
          }
          if (pcSlice->m_vps->m_layerIdInOls[i][j] == scalingListAPS->m_layerId)
          {
            isRefLayerInOls = true;
          }
        }
        CHECK(isCurrLayerInOls && !isRefLayerInOls,
              "When VCL NAl unit in layer A refers to APS in layer B, all OLS that contains layer A shall also "
              "contains layer B");
      }
    }
    ScalingList scalingList = scalingListAPS->m_scalingListApsInfo;
    quant->setScalingListDec(scalingList);
    quant->setUseScalingList(true);
  }
  else
  {
    quant->setUseScalingList(false);
  }

  if (pcSlice->m_sps->m_lmcsEnabled)
  {
    if (m_isFirstSliceInPicture)
    {
      m_sliceLmcsApsId = -1;
    }
    if (pcSlice->m_lmcsEnabledFlag)
    {
      APS *lmcsAPS = pcSlice->m_picHeader->m_lmcsAps;
      if (m_sliceLmcsApsId == -1)
      {
        m_sliceLmcsApsId = lmcsAPS->m_APSId;
      }
      else
      {
        CHECK(lmcsAPS->m_APSId != m_sliceLmcsApsId, "same APS ID shall be used for all slices in one picture");
      }
      if (pcSlice->m_nuhLayerId != lmcsAPS->m_layerId)
      {
        CHECK(lmcsAPS->m_layerId > pcSlice->m_nuhLayerId,
              "Layer Id of APS cannot be greater than layer Id of VCL NAL unit the refer to it");
        CHECK(pcSlice->m_sps->m_vpsId == 0,
              "VPSId of the referred SPS cannot be 0 when layer Id of APS and layer Id of current slice are different");
        for (int i = 0; i < pcSlice->m_vps->m_vpsNumOutputLayerSets; i++)
        {
          bool isCurrLayerInOls = false;
          bool isRefLayerInOls  = false;
          for (int j = pcSlice->m_vps->m_numLayersInOls[i] - 1; j >= 0; j--)
          {
            if (pcSlice->m_vps->m_layerIdInOls[i][j] == pcSlice->m_nuhLayerId)
            {
              isCurrLayerInOls = true;
            }
            if (pcSlice->m_vps->m_layerIdInOls[i][j] == lmcsAPS->m_layerId)
            {
              isRefLayerInOls = true;
            }
          }
          CHECK(isCurrLayerInOls && !isRefLayerInOls,
                "When VCL NAl unit in layer A refers to APS in layer B, all OLS that contains layer A shall also "
                "contains layer B");
        }
      }
      SliceReshapeInfo &sInfo      = lmcsAPS->m_reshapeAPSInfo;
      SliceReshapeInfo &tInfo      = m_cReshaper.m_sliceReshapeInfo;
      tInfo.reshaperModelMaxBinIdx = sInfo.reshaperModelMaxBinIdx;
      tInfo.reshaperModelMinBinIdx = sInfo.reshaperModelMinBinIdx;
      memcpy(tInfo.reshaperModelBinCWDelta, sInfo.reshaperModelBinCWDelta, sizeof(int) * (PIC_CODE_CW_BINS));
      tInfo.maxNbitsNeededDeltaCW         = sInfo.maxNbitsNeededDeltaCW;
      tInfo.chrResScalingOffset           = sInfo.chrResScalingOffset;
      tInfo.sliceReshaperEnableFlag       = pcSlice->m_lmcsEnabledFlag;
      tInfo.enableChromaAdj               = pcSlice->m_picHeader->m_lmcsChromaResidualScaleFlag;
      tInfo.sliceReshaperModelPresentFlag = true;
    }
    else
    {
      SliceReshapeInfo &tInfo             = m_cReshaper.m_sliceReshapeInfo;
      tInfo.sliceReshaperEnableFlag       = false;
      tInfo.enableChromaAdj               = false;
      tInfo.sliceReshaperModelPresentFlag = false;
    }
    if (pcSlice->m_lmcsEnabledFlag)
    {
      m_cReshaper.constructReshaper();
    }
    else
    {
      m_cReshaper.m_reshapeFlag = false;
    }
    if ((pcSlice->m_eSliceType == I_SLICE) && m_cReshaper.m_sliceReshapeInfo.sliceReshaperEnableFlag)
    {
      m_cReshaper.m_ctuFlag     = false;
      m_cReshaper.m_recReshaped = true;
    }
    else
    {
      if (m_cReshaper.m_sliceReshapeInfo.sliceReshaperEnableFlag)
      {
        m_cReshaper.m_ctuFlag     = true;
        m_cReshaper.m_recReshaped = true;
      }
      else
      {
        m_cReshaper.m_ctuFlag     = false;
        m_cReshaper.m_recReshaped = false;
      }
    }
  }
  else
  {
    m_cReshaper.m_ctuFlag     = false;
    m_cReshaper.m_recReshaped = false;
  }

  //  Decode a picture
  m_cSliceDecoder.decompressSlice(pcSlice, &(nalu.getBitstream()),
                                  (m_pic->m_poc == getDebugPOC() ? getDebugCTU() : -1));

  m_isFirstSliceInPicture = false;
  m_uiSliceSegmentIdx++;

  pcSlice->freeScaledRefPicList(scaledRefPic);

  return false;
}

void DecLib::updatePrevGDRInSameLayer()
{
  const NalUnitType pictureType = m_pic->getPictureType();

  if (pictureType == NAL_UNIT_CODED_SLICE_GDR && !m_pic->m_cs->pps->m_mixedNaluTypesInPicFlag)
  {
    m_prevGDRInSameLayerPOC[m_pic->m_layerId] = m_pic->m_poc;
  }
}

void DecLib::updateAssociatedIRAP()
{
  const NalUnitType pictureType = m_pic->getPictureType();

  if ((pictureType == NAL_UNIT_CODED_SLICE_IDR_W_RADL || pictureType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
       pictureType == NAL_UNIT_CODED_SLICE_CRA) &&
      !m_pic->m_cs->pps->m_mixedNaluTypesInPicFlag)
  {
    m_associatedIRAPDecodingOrderNumber[m_pic->m_layerId] = m_pic->m_decodingOrderNumber;
    m_pocCRA[m_pic->m_layerId]                            = m_pic->m_poc;
    m_checkCRAFlags[m_pic->m_layerId].clear();
    m_associatedIRAPType[m_pic->m_layerId] = pictureType;
  }
}

void DecLib::updatePrevIRAPAndGDRSubpic()
{
  for (int j = 0; j < m_uiSliceSegmentIdx; j++)
  {
    Slice    *pcSlice   = m_pic->m_slices[j];
    const int subpicIdx = pcSlice->m_pps->getSubPicIdxFromSubPicId(pcSlice->m_sliceSubPicId);
    if (pcSlice->getCtuAddrInSlice(0) == m_pic->m_cs->pps->m_subPics[subpicIdx].m_firstCtuInSubPic)
    {
      const NalUnitType subpicType = pcSlice->m_eNalUnitType;
      if (subpicType == NAL_UNIT_CODED_SLICE_IDR_W_RADL || subpicType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
          subpicType == NAL_UNIT_CODED_SLICE_CRA)
      {
        m_prevIRAPSubpicPOC[m_pic->m_layerId][subpicIdx]        = m_pic->m_poc;
        m_prevIRAPSubpicType[m_pic->m_layerId][subpicIdx]       = subpicType;
        m_prevIRAPSubpicDecOrderNo[m_pic->m_layerId][subpicIdx] = m_pic->m_decodingOrderNumber;
      }
      else if (subpicType == NAL_UNIT_CODED_SLICE_GDR)
      {
        m_prevGDRSubpicPOC[m_pic->m_layerId][subpicIdx] = m_pic->m_poc;
      }
    }
  }
}

void DecLib::reconstructClipRange(Slice *pcSlice)
{
  int clipDeltaShift = pcSlice->getClipDeltaShift();

  int deltaMax = pcSlice->m_lumaPelMax;
  if (deltaMax > 0)
  {
    deltaMax = (deltaMax << clipDeltaShift);
  }
  else if (deltaMax < 0)
  {
    deltaMax = -((-deltaMax) << clipDeltaShift);
  }
  int deltaMin = pcSlice->m_lumaPelMin;
  if (deltaMin > 0)
  {
    deltaMin = (deltaMin << clipDeltaShift);
  }
  else if (deltaMin < 0)
  {
    deltaMin = -((-deltaMin) << clipDeltaShift);
  }

  if (pcSlice->isIntra())
  {
    pcSlice->m_lumaPelMax = std::min(deltaMax + (235 * (1 << (pcSlice->m_sps->m_bitDepths[ChannelType::LUMA] - 8))),
                                     (1 << pcSlice->m_sps->m_bitDepths[ChannelType::LUMA]) - 1);
    pcSlice->m_lumaPelMin = std::max(0, deltaMin + (16 * (1 << (pcSlice->m_sps->m_bitDepths[ChannelType::LUMA] - 8))));
  }
  else   // not I-Slice
  {
    const Picture *const pColPic = pcSlice->getRefPic(RefPicList(1 - pcSlice->m_colFromL0Flag), pcSlice->m_colRefIdx);
    ClpRng               colLumaClpRng = pColPic->m_lumaClpRng;
    int lumaPelMax = std::min(deltaMax + colLumaClpRng.max, (1 << pcSlice->m_sps->m_bitDepths[ChannelType::LUMA]) - 1);
    int lumaPelMin = std::max(0, deltaMin + colLumaClpRng.min);
    CHECK(lumaPelMax > (1 << pcSlice->m_sps->m_bitDepths[ChannelType::LUMA]) - 1, "this is not possible");
    CHECK(lumaPelMin < 0, "this is not possible");
    CHECK(lumaPelMin > lumaPelMax, "this is not possible");
    pcSlice->m_lumaPelMax = lumaPelMax;
    pcSlice->m_lumaPelMin = lumaPelMin;
  }
}

void DecLib::xDecodeOPI(InputNALUnit &nalu)
{
  m_opi = new OPI();
  m_HLSReader.setBitstream(&nalu.getBitstream());

  CHECK(nalu.m_temporalId, "The value of TemporalId of OPI NAL units shall be equal to 0");

  m_HLSReader.parseOPI(m_opi);
}

void DecLib::xDecodeVPS(InputNALUnit &nalu)
{
  VPS *vps = new VPS();
  m_HLSReader.setBitstream(&nalu.getBitstream());

  CHECK(nalu.m_temporalId, "The value of TemporalId of VPS NAL units shall be equal to 0");

  m_HLSReader.parseVPS(vps);

  // storeVPS may directly delete the new VPS in case it is a repetition. Need to retrieve proper initialized memory
  // back
  int vpsID = vps->m_vpsId;
  m_parameterSetManager.storeVPS(vps, nalu.getBitstream().getFifo());

  if (m_vps == nullptr)
  {
    // m_vps is used for conformance checks. Unless a VPS is referred to, just set the first one we received
    // repeated parameter sets may be deleted, set a valid VPS pointer back from parameter set manager
    m_vps = m_parameterSetManager.getVPS(vpsID);
  }
}

void DecLib::xDecodeDCI(InputNALUnit &nalu)
{
  m_HLSReader.setBitstream(&nalu.getBitstream());

  CHECK(nalu.m_temporalId, "The value of TemporalId of DCI NAL units shall be equal to 0");
  if (!m_dci)
  {
    m_dci = new DCI;
    m_HLSReader.parseDCI(m_dci);
  }
  else
  {
    DCI dupDCI;
    m_HLSReader.parseDCI(&dupDCI);
    CHECK(!m_dci->IsIndenticalDCI(dupDCI), "Two signaled DCIs are different");
  }
}

void DecLib::xDecodeSPS(InputNALUnit &nalu)
{
  SPS *sps = new SPS();
  m_HLSReader.setBitstream(&nalu.getBitstream());

  CHECK(nalu.m_temporalId, "The value of TemporalId of SPS NAL units shall be equal to 0");

  m_HLSReader.parseSPS(sps);
  sps->m_layerId = nalu.m_nuhLayerId;

#if ENABLE_NNLF
  sps->m_nnlfStore = sps->m_nnlf || !m_decCfg.m_nnlfDumpBasename.empty();
#endif

  // XXX: Creating the ALF object here, assuming the SPS is parsed first.
  if ((m_alfEcm == nullptr) && (m_alfVtm == nullptr))
  {
    if (sps->m_alfImprovementsEnabledFlag)
    {
      m_alfEcm = new AdaptiveLoopFilterEcm;
    }
    else
    {
      m_alfVtm = new AdaptiveLoopFilterVtm;
    }
    ApsAlfParam::setAlfType(sps->m_alfImprovementsEnabledFlag);
    ApsCcAlfParam::setAlfType(sps->m_alfImprovementsEnabledFlag);
    m_apcSlicePilot->m_ccAlfFilterParam.create();
  }
  else
  {
    CHECK(!(m_alfEcm != nullptr) != !sps->m_alfImprovementsEnabledFlag,
          "The ALF module is already setup for a different version.");
  }

  DTRACE(g_trace_ctx, D_QP_PER_CTU, "CTU Size: %dx%d", sps->m_maxCuWidth, sps->m_maxCuHeight);
  m_accessUnitSpsNumSubpic[nalu.m_nuhLayerId] = sps->m_numSubPics;
  m_parameterSetManager.storeSPS(sps, nalu.getBitstream().getFifo());
}

void DecLib::xDecodePPS(InputNALUnit &nalu)
{
  PPS *pps = new PPS();
  m_HLSReader.setBitstream(&nalu.getBitstream());
  m_HLSReader.parsePPS(pps);
  pps->m_layerId    = nalu.m_nuhLayerId;
  pps->m_temporalId = nalu.m_temporalId;
  pps->m_puCounter  = m_puCounter;
  m_parameterSetManager.storePPS(pps, nalu.getBitstream().getFifo());
}

void DecLib::xDecodeAPS(InputNALUnit &nalu)
{
  APS *aps = new APS();

  m_HLSReader.setBitstream(&nalu.getBitstream());
  m_HLSReader.parseAPS(aps);
  aps->m_temporalId           = nalu.m_temporalId;
  aps->m_layerId              = nalu.m_nuhLayerId;
  aps->m_hasPrefixNalUnitType = nalu.m_nalUnitType == NAL_UNIT_PREFIX_APS;
  aps->m_puCounter            = m_puCounter;
  m_parameterSetManager.checkAuApsContent(aps, m_accessUnitApsNals[aps->m_APSType]);
  if (m_apsMapEnc)
  {
    APS *apsEnc = new APS();
    *apsEnc     = *aps;
    (*m_apsMapEnc)[aps->m_APSType].storePS(apsEnc->m_APSId, apsEnc);
  }

  if (nalu.m_nalUnitType == NAL_UNIT_SUFFIX_APS && m_prevSliceSkipped)
  {
    m_accessUnitApsNals[aps->m_APSType].pop_back();
  }

  // aps will be deleted if it was already stored (and did not changed),
  // thus, storing it must be last action.
  m_parameterSetManager.storeAPS(aps, nalu.getBitstream().getFifo());
}

bool DecLib::decode(InputNALUnit &nalu, int &iSkipFrame, int &iPOCLastDisplay, int iTargetOlsIdx)
{
  PROFILER_SCOPE(0, g_timeProfiler, P_TOP_LEVEL);
  bool ret;
  // ignore all NAL units of layers > 0
  if ((nalu.m_nalUnitType != NAL_UNIT_SUFFIX_APS && nalu.m_nalUnitType != NAL_UNIT_EOS &&
       nalu.m_nalUnitType != NAL_UNIT_EOB && nalu.m_nalUnitType != NAL_UNIT_SUFFIX_SEI &&
       nalu.m_nalUnitType != NAL_UNIT_FD && nalu.m_nalUnitType != NAL_UNIT_UNSPECIFIED_30 &&
       nalu.m_nalUnitType != NAL_UNIT_UNSPECIFIED_31) ||
      !m_prevSliceSkipped)
  {
    AccessUnitInfo auInfo;
    auInfo.m_nalUnitType = nalu.m_nalUnitType;
    auInfo.m_nuhLayerId  = nalu.m_nuhLayerId;
    auInfo.m_temporalId  = nalu.m_temporalId;
    m_accessUnitNals.push_back(auInfo);
    m_pictureUnitNals.push_back(nalu.m_nalUnitType);
  }
  switch (nalu.m_nalUnitType)
  {
  case NAL_UNIT_VPS:
    xDecodeVPS(nalu);
    if (getTOlsIdxExternalFlag())
    {
      m_vps->m_targetOlsIdx = iTargetOlsIdx;
    }
    else if (getTOlsIdxOpiFlag())
    {
      m_vps->m_targetOlsIdx = m_opi->m_opiolsidx;
    }
    else
    {
      m_vps->m_targetOlsIdx = m_vps->deriveTargetOLSIdx();
    }
    return false;
  case NAL_UNIT_OPI:
    xDecodeOPI(nalu);
    return false;
  case NAL_UNIT_DCI:
    xDecodeDCI(nalu);
    return false;
  case NAL_UNIT_SPS:
    xDecodeSPS(nalu);
    return false;

  case NAL_UNIT_PPS:
    xDecodePPS(nalu);
    return false;

  case NAL_UNIT_PH:
    xDecodePicHeader(nalu);
    return !m_isFirstSliceInPicture;

  case NAL_UNIT_PREFIX_APS:
    xDecodeAPS(nalu);
    return false;

  case NAL_UNIT_SUFFIX_APS:
    if (m_prevSliceSkipped)
    {
      xDecodeAPS(nalu);
    }
    else
    {
      m_suffixApsNalus.push_back(new InputNALUnit(nalu));
    }
    return false;

  case NAL_UNIT_PREFIX_SEI:
    // Buffer up prefix SEI messages until SPS of associated VCL is known.
    m_prefixSEINALUs.push_back(new InputNALUnit(nalu));
    m_pictureSeiNalus.push_back(new InputNALUnit(nalu));
    return false;

  case NAL_UNIT_SUFFIX_SEI:
    if (m_pic)
    {
      if (m_prevSliceSkipped)
      {
        msg(NOTICE, "Note: received suffix SEI but current picture is skipped.\n");
        return false;
      }
      m_pictureSeiNalus.push_back(new InputNALUnit(nalu));
      m_accessUnitSeiNalus.push_back(new InputNALUnit(nalu));
      m_accessUnitSeiTids.push_back(nalu.m_temporalId);
      const SPS *sps = m_parameterSetManager.getActiveSPS();
      const VPS *vps = m_parameterSetManager.getVPS(sps->m_vpsId);
      const bool seiMessageRead =
        m_seiReader.parseSEImessage(&(nalu.getBitstream()), m_pic->m_SEIs, nalu.m_nalUnitType, nalu.m_nuhLayerId,
                                    nalu.m_temporalId, vps, sps, m_HRD, m_pDecodedSEIOutputStream);
#if JVET_S0257_DUMP_360SEI_MESSAGE
      m_seiCfgDump.write360SeiDump(m_decCfg.m_outputDecoded360SEIMessagesFilename, m_pic->m_SEIs, sps);
#endif
      if (seiMessageRead)
      {
        m_accessUnitSeiPayLoadTypes.push_back(std::tuple<NalUnitType, int, SEI::PayloadType>(
          nalu.m_nalUnitType, nalu.m_nuhLayerId, m_pic->m_SEIs.back()->payloadType()));
      }
    }
    else
    {
      msg(NOTICE, "Note: received suffix SEI but no picture currently active.\n");
    }
    return false;

  case NAL_UNIT_CODED_SLICE_TRAIL:
  case NAL_UNIT_CODED_SLICE_STSA:
  case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
  case NAL_UNIT_CODED_SLICE_IDR_N_LP:
  case NAL_UNIT_CODED_SLICE_CRA:
  case NAL_UNIT_CODED_SLICE_GDR:
  case NAL_UNIT_CODED_SLICE_RADL:
  case NAL_UNIT_CODED_SLICE_RASL:
    ret = xDecodeSlice(nalu, iSkipFrame, iPOCLastDisplay);
    return ret;

  case NAL_UNIT_EOS:
    m_associatedIRAPType[nalu.m_nuhLayerId] = NAL_UNIT_INVALID;
    m_pocCRA[nalu.m_nuhLayerId]             = -MAX_INT;
    m_checkCRAFlags[nalu.m_nuhLayerId].clear();
    m_prevGDRInSameLayerPOC[nalu.m_nuhLayerId]         = -MAX_INT;
    m_prevGDRInSameLayerRecoveryPOC[nalu.m_nuhLayerId] = -MAX_INT;
    std::fill_n(m_prevGDRSubpicPOC[nalu.m_nuhLayerId], MAX_NUM_SUB_PICS, -MAX_INT);
    std::fill_n(m_prevIRAPSubpicPOC[nalu.m_nuhLayerId], MAX_NUM_SUB_PICS, -MAX_INT);
    memset(m_prevIRAPSubpicDecOrderNo[nalu.m_nuhLayerId], 0, sizeof(int) * MAX_NUM_SUB_PICS);
    std::fill_n(m_prevIRAPSubpicType[nalu.m_nuhLayerId], MAX_NUM_SUB_PICS, NAL_UNIT_INVALID);
    m_pocRandomAccess                  = MAX_INT;
    m_prevLayerID                      = MAX_INT;
    m_prevPOC                          = -MAX_INT;
    m_prevSliceSkipped                 = false;
    m_skippedPOC                       = 0;
    m_accessUnitEos[nalu.m_nuhLayerId] = true;
    m_prevEOS[nalu.m_nuhLayerId]       = true;
    return false;

  case NAL_UNIT_ACCESS_UNIT_DELIMITER:
    {
      AUDReader audReader;
      uint32_t  picType;
      audReader.parseAccessUnitDelimiter(&(nalu.getBitstream()), m_audIrapOrGdrAuFlag, picType);
      return !m_isFirstSliceInPicture;
    }

  case NAL_UNIT_EOB:
    return false;

  case NAL_UNIT_FD:
    {
      FDReader fdReader;
      uint32_t fdSize;
      fdReader.parseFillerData(&(nalu.getBitstream()), fdSize);
      msg(NOTICE, "Note: found NAL_UNIT_FD with %u bytes payload.\n", fdSize);
      return false;
    }

  case NAL_UNIT_RESERVED_IRAP_VCL_14:
  case NAL_UNIT_RESERVED_IRAP_VCL_15:
    msg(NOTICE, "Note: found reserved VCL NAL unit.\n");
    xParsePrefixSEIsForUnknownVCLNal();
    return false;
  case NAL_UNIT_RESERVED_VCL_12:
  case NAL_UNIT_RESERVED_VCL_13:
  case NAL_UNIT_RESERVED_NVCL_22:
  case NAL_UNIT_RESERVED_NVCL_23:
    msg(NOTICE, "Note: found reserved NAL unit.\n");
    return false;
  case NAL_UNIT_UNSPECIFIED_28:
  case NAL_UNIT_UNSPECIFIED_29:
  case NAL_UNIT_UNSPECIFIED_30:
  case NAL_UNIT_UNSPECIFIED_31:
    msg(NOTICE, "Note: found unspecified NAL unit.\n");
    return false;
  default:
    THROW("Invalid NAL unit type");
    break;
  }

  return false;
}

/** Function for checking if picture should be skipped because of random access. This function checks the skipping of
 * pictures in the case of -s option random access. All pictures prior to the random access point indicated by the
 * counter iSkipFrame are skipped. It also checks the type of Nal unit type at the random access point. If the random
 * access point is CRA/CRANT/BLA/BLANT, TFD pictures with POC less than the POC of the random access point are skipped.
 *  If the random access point is IDR all pictures after the random access point are decoded.
 *  If the random access point is none of the above, a warning is issues, and decoding of pictures with POC
 *  equal to or greater than the random access point POC is attempted. For non IDR/CRA/BLA random
 *  access point there is no guarantee that the decoder will not crash.
 */
bool DecLib::isRandomAccessSkipPicture(int &iSkipFrame, int &iPOCLastDisplay, bool mixedNaluInPicFlag, uint32_t layerId)
{
  if ((iSkipFrame > 0) && (m_apcSlicePilot->getFirstCtuRsAddrInSlice() == 0 && layerId == 0) &&
      (m_skippedPOC != MAX_INT) && (m_skippedLayerID != MAX_INT))
  {
    // When skipFrame count greater than 0, and current frame is not the first frame of sequence, decrement skipFrame
    // count. If skipFrame count is still greater than 0, the current frame will be skipped.
    iSkipFrame--;
  }

  if (iSkipFrame)
  {
    iSkipFrame--;   // decrement the counter
    m_maxDecSubPicIdx         = 0;
    m_maxDecSliceAddrInSubPic = -1;
    return true;
  }
  else if (m_apcSlicePilot->m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
           m_apcSlicePilot->m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP)
  {
    m_pocRandomAccess = -MAX_INT;   // no need to skip the reordered pictures in IDR, they are decodable.
  }
  else if (m_pocRandomAccess == MAX_INT)   // start of random access point, m_pocRandomAccess has not been set yet.
  {
    if (m_apcSlicePilot->m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA ||
        m_apcSlicePilot->m_eNalUnitType == NAL_UNIT_CODED_SLICE_GDR)
    {
      // set the POC random access since we need to skip the reordered pictures in the case of CRA/CRANT/BLA/BLANT.
      m_pocRandomAccess = m_apcSlicePilot->m_poc;
    }
    else
    {
      if (!m_warningMessageSkipPicture)
      {
        msg(WARNING,
            "Warning: This is not a valid random access point and the data is discarded until the first CRA or GDR "
            "picture\n");
        m_warningMessageSkipPicture = true;
      }
      iSkipFrame--;
      m_maxDecSubPicIdx         = 0;
      m_maxDecSliceAddrInSubPic = -1;
      return true;
    }
  }
  // skip the reordered pictures, if necessary
  else if (m_apcSlicePilot->m_poc < m_pocRandomAccess &&
           (m_apcSlicePilot->m_eNalUnitType == NAL_UNIT_CODED_SLICE_RASL || mixedNaluInPicFlag))
  {
    iPOCLastDisplay++;
    iSkipFrame--;
    m_maxDecSubPicIdx         = 0;
    m_maxDecSliceAddrInSubPic = -1;
    return true;
  }
  // if we reach here, then the picture is not skipped.
  return false;
}

void DecLib::checkNalUnitConstraints(uint32_t naluType)
{
  if (m_parameterSetManager.getActiveSPS() != nullptr)
  //      && &m_parameterSetManager.getActiveSPS()->m_profileTierLevel != nullptr)
  {
    const ConstraintInfo *cInfo = &m_parameterSetManager.getActiveSPS()->m_profileTierLevel.m_constraintInfo;
    xCheckNalUnitConstraintFlags(cInfo, naluType);
  }
}

void DecLib::xCheckNalUnitConstraintFlags(const ConstraintInfo *cInfo, uint32_t naluType)
{
  if (cInfo != nullptr)
  {
    CHECK(cInfo->m_noTrailConstraintFlag && naluType == NAL_UNIT_CODED_SLICE_TRAIL,
          "Non-conforming bitstream. no_trail_constraint_flag is equal to 1 but bitstream contains NAL unit of type "
          "TRAIL_NUT.");
    CHECK(cInfo->m_noStsaConstraintFlag && naluType == NAL_UNIT_CODED_SLICE_STSA,
          "Non-conforming bitstream. no_stsa_constraint_flag is equal to 1 but bitstream contains NAL unit of type "
          "STSA_NUT.");
    CHECK(cInfo->m_noRaslConstraintFlag && naluType == NAL_UNIT_CODED_SLICE_RASL,
          "Non-conforming bitstream. no_rasl_constraint_flag is equal to 1 but bitstream contains NAL unit of type "
          "RASL_NUT.");
    CHECK(cInfo->m_noRadlConstraintFlag && naluType == NAL_UNIT_CODED_SLICE_RADL,
          "Non-conforming bitstream. no_radl_constraint_flag is equal to 1 but bitstream contains NAL unit of type "
          "RADL_NUT.");
    CHECK(cInfo->m_noIdrConstraintFlag && (naluType == NAL_UNIT_CODED_SLICE_IDR_W_RADL),
          "Non-conforming bitstream. no_idr_constraint_flag is equal to 1 but bitstream contains NAL unit of type "
          "IDR_W_RADL.");
    CHECK(cInfo->m_noIdrConstraintFlag && (naluType == NAL_UNIT_CODED_SLICE_IDR_N_LP),
          "Non-conforming bitstream. no_idr_constraint_flag is equal to 1 but bitstream contains NAL unit of type "
          "IDR_N_LP.");
    CHECK(cInfo->m_noCraConstraintFlag && naluType == NAL_UNIT_CODED_SLICE_CRA,
          "Non-conforming bitstream. no_cra_constraint_flag is equal to 1 but bitstream contains NAL unit of type "
          "CRA_NUT.");
    CHECK(cInfo->m_noGdrConstraintFlag && naluType == NAL_UNIT_CODED_SLICE_GDR,
          "Non-conforming bitstream. no_gdr_constraint_flag is equal to 1 but bitstream contains NAL unit of type "
          "GDR_NUT.");
    CHECK(cInfo->m_noApsConstraintFlag && naluType == NAL_UNIT_PREFIX_APS,
          "Non-conforming bitstream. no_aps_constraint_flag is equal to 1 but bitstream contains NAL unit of type "
          "APS_PREFIX_NUT.");
    CHECK(cInfo->m_noApsConstraintFlag && naluType == NAL_UNIT_SUFFIX_APS,
          "Non-conforming bitstream. no_aps_constraint_flag is equal to 1 but bitstream contains NAL unit of type "
          "APS_SUFFIX_NUT.");
  }
}

void DecLib::xCheckMixedNalUnit(Slice *pcSlice, SPS *sps, InputNALUnit &nalu)
{
  if (pcSlice->m_pps->m_mixedNaluTypesInPicFlag)
  {
    CHECK(pcSlice->m_pps->m_numSlicesInPic < 2, "mixed nal unit type picture, but with less than 2 slices");

    CHECK(pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_GDR,
          "picture with mixed NAL unit type cannot have GDR slice");

    // Check that if current slice is IRAP type, the other type of NAL can only be TRAIL_NUT
    if (pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
        pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP || pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA)
    {
      for (int i = 0; i < m_uiSliceSegmentIdx; i++)
      {
        Slice *preSlice = m_pic->m_slices[i];
        CHECK((pcSlice->m_eNalUnitType != preSlice->m_eNalUnitType) &&
                (preSlice->m_eNalUnitType != NAL_UNIT_CODED_SLICE_TRAIL),
              "In a mixed NAL unt type picture, an IRAP slice can be mixed with Trail slice(s) only");
      }
    }

    // if this is the last slice of the picture, check whether that there are at least two different NAL unit types in
    // the picture
    if (pcSlice->m_pps->m_numSlicesInPic == (m_uiSliceSegmentIdx + 1))
    {
      bool hasDiffTypes = false;
      for (int i = 1; !hasDiffTypes && i <= m_uiSliceSegmentIdx; i++)
      {
        Slice *slice1 = m_pic->m_slices[i - 1];
        Slice *slice2 = m_pic->m_slices[i];
        if (slice1->m_eNalUnitType != slice2->m_eNalUnitType)
        {
          hasDiffTypes = true;
        }
      }
      CHECK(!hasDiffTypes, "VCL NAL units of the picture shall have two or more different nal_unit_type values");
    }
  }
  else   // all slices shall have the same nal unit type
  {
    bool sameNalUnitType = true;
    for (int i = 0; i < m_uiSliceSegmentIdx; i++)
    {
      Slice *preSlice = m_pic->m_slices[i];
      if (preSlice->m_eNalUnitType != pcSlice->m_eNalUnitType)
      {
        sameNalUnitType = false;
      }
    }
    CHECK(!sameNalUnitType, "pps_mixed_nalu_types_in_pic_flag is zero, but have different nal unit types");
  }
}
/**
- lookahead through next NAL units to determine if current NAL unit is the first NAL unit in a new picture
*/
bool DecLib::isNewPicture(std::ifstream *bitstreamFile, class InputByteStream *bytestream)
{
  bool ret      = false;
  bool finished = false;

  // cannot be a new picture if there haven't been any slices yet
  if (getFirstSliceInPicture())
  {
    return false;
  }

  // save stream position for backup
#if RExt__DECODER_DEBUG_STATISTICS
  CodingStatistics::CodingStatisticsData *backupStats =
    new CodingStatistics::CodingStatisticsData(CodingStatistics::GetStatistics());
  std::streampos location = bitstreamFile->tellg() - std::streampos(bytestream->getNumBufferedBytes());
#else
  std::streampos location = bitstreamFile->tellg();
#endif

  // look ahead until picture start location is determined
  while (!finished && !!(*bitstreamFile))
  {
    AnnexBStats  stats = AnnexBStats();
    InputNALUnit nalu;
    byteStreamNALUnit(*bytestream, nalu.getBitstream().getFifo(), stats);
    if (nalu.getBitstream().getFifo().empty())
    {
      msg(ERROR, "Warning: Attempt to decode an empty NAL unit\n");
    }
    else
    {
      // get next NAL unit type
      read(nalu);
      switch (nalu.m_nalUnitType)
      {
      // NUT that indicate the start of a new picture
      case NAL_UNIT_ACCESS_UNIT_DELIMITER:
      case NAL_UNIT_OPI:
      case NAL_UNIT_DCI:
      case NAL_UNIT_VPS:
      case NAL_UNIT_SPS:
      case NAL_UNIT_PPS:
      case NAL_UNIT_PH:
        ret      = true;
        finished = true;
        break;

      // NUT that may be the start of a new picture - check first bit in slice header
      case NAL_UNIT_CODED_SLICE_TRAIL:
      case NAL_UNIT_CODED_SLICE_STSA:
      case NAL_UNIT_CODED_SLICE_RASL:
      case NAL_UNIT_CODED_SLICE_RADL:
      case NAL_UNIT_RESERVED_VCL_12:
      case NAL_UNIT_RESERVED_VCL_13:
      case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
      case NAL_UNIT_CODED_SLICE_IDR_N_LP:
      case NAL_UNIT_CODED_SLICE_CRA:
      case NAL_UNIT_CODED_SLICE_GDR:
      case NAL_UNIT_RESERVED_IRAP_VCL_14:
      case NAL_UNIT_RESERVED_IRAP_VCL_15:
        ret      = checkPictureHeaderInSliceHeaderFlag(nalu);
        finished = true;
        break;

      // NUT that are not the start of a new picture
      case NAL_UNIT_EOS:
      case NAL_UNIT_EOB:
      case NAL_UNIT_SUFFIX_APS:
      case NAL_UNIT_SUFFIX_SEI:
      case NAL_UNIT_FD:
        ret      = false;
        finished = true;
        break;

      // NUT that might indicate the start of a new picture - keep looking
      case NAL_UNIT_PREFIX_APS:
      case NAL_UNIT_PREFIX_SEI:
      case NAL_UNIT_RESERVED_NVCL_22:
      case NAL_UNIT_RESERVED_NVCL_23:
      case NAL_UNIT_UNSPECIFIED_28:
      case NAL_UNIT_UNSPECIFIED_29:
      case NAL_UNIT_UNSPECIFIED_30:
      case NAL_UNIT_UNSPECIFIED_31:
      default:
        break;
      }
    }
  }

  // restore previous stream location - minus 3 due to the need for the annexB parser to read three extra bytes
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  bitstreamFile->clear();
  bitstreamFile->seekg(location);
  bytestream->reset();
  CodingStatistics::SetStatistics(*backupStats);
  delete backupStats;
#else
  bitstreamFile->clear();
  bitstreamFile->seekg(location - std::streamoff(3));
  bytestream->reset();
#endif

  // return TRUE if next NAL unit is the start of a new picture
  return ret;
}

/**
- lookahead through next NAL units to determine if current NAL unit is the first NAL unit in a new access unit
*/
bool DecLib::isNewAccessUnit(bool newPicture, std::ifstream *bitstreamFile, class InputByteStream *bytestream)
{
  bool ret      = false;
  bool finished = false;

  // can only be the start of an AU if this is the start of a new picture
  if (newPicture == false)
  {
    return false;
  }

  // save stream position for backup
#if RExt__DECODER_DEBUG_STATISTICS
  CodingStatistics::CodingStatisticsData *backupStats =
    new CodingStatistics::CodingStatisticsData(CodingStatistics::GetStatistics());
  std::streampos location = bitstreamFile->tellg() - std::streampos(bytestream->getNumBufferedBytes());
#else
  std::streampos location = bitstreamFile->tellg();
#endif

  // look ahead until access unit start location is determined
  while (!finished && !!(*bitstreamFile))
  {
    AnnexBStats  stats = AnnexBStats();
    InputNALUnit nalu;
    byteStreamNALUnit(*bytestream, nalu.getBitstream().getFifo(), stats);
    if (nalu.getBitstream().getFifo().empty())
    {
      msg(ERROR, "Warning: Attempt to decode an empty NAL unit\n");
    }
    else
    {
      // get next NAL unit type
      read(nalu);
      switch (nalu.m_nalUnitType)
      {
      // AUD always indicates the start of a new access unit
      case NAL_UNIT_ACCESS_UNIT_DELIMITER:
        ret      = true;
        finished = true;
        break;

      // slice types - check layer ID and POC
      case NAL_UNIT_CODED_SLICE_TRAIL:
      case NAL_UNIT_CODED_SLICE_STSA:
      case NAL_UNIT_CODED_SLICE_RASL:
      case NAL_UNIT_CODED_SLICE_RADL:
      case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
      case NAL_UNIT_CODED_SLICE_IDR_N_LP:
      case NAL_UNIT_CODED_SLICE_CRA:
      case NAL_UNIT_CODED_SLICE_GDR:
        ret      = isSliceNaluFirstInAU(newPicture, nalu);
        finished = true;
        break;

      // NUT that are not the start of a new access unit
      case NAL_UNIT_EOS:
      case NAL_UNIT_EOB:
      case NAL_UNIT_SUFFIX_APS:
      case NAL_UNIT_SUFFIX_SEI:
      case NAL_UNIT_FD:
        ret      = false;
        finished = true;
        break;

      // all other NUT - keep looking to find first VCL
      default:
        break;
      }
    }
  }

  // restore previous stream location
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  bitstreamFile->clear();
  bitstreamFile->seekg(location);
  bytestream->reset();
  CodingStatistics::SetStatistics(*backupStats);
  delete backupStats;
#else
  bitstreamFile->clear();
  bitstreamFile->seekg(location);
  bytestream->reset();
#endif

  // return TRUE if next NAL unit is the start of a new picture
  return ret;
}
//! \}
