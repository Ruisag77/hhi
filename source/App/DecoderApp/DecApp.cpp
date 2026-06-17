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

/** \file     DecApp.cpp
    \brief    Decoder application class
*/

#include <list>
#include <vector>
#include <stdio.h>
#include <fcntl.h>

#include "DecApp.h"
#include "DecAppCfg.h"
#include "DecoderLib/AnnexBread.h"
#include "DecoderLib/NALread.h"
#include "CommonLib/CodingStatistics.h"
#include "CommonLib/dtrace_codingstruct.h"

//! \ingroup DecoderApp
//! \{

static int calcGcd(int a, int b)
{
  // assume that a >= b
  return b == 0 ? a : calcGcd(b, a % b);
}

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

DecApp::DecApp() : m_iPOCLastDisplay(-MAX_INT)
{
  for (int i = 0; i < MAX_NUM_LAYER_IDS; i++)
  {
    m_newCLVS[i] = true;
  }
#if ENABLE_NNLF
  std::memset(m_nnlfDumpIoFiles, 0, sizeof(m_nnlfDumpIoFiles));
  m_nnlfDumpCount = 0;
#endif
}

DecApp::~DecApp()
{
#if ENABLE_TRACING
  tracing_uninit(g_trace_ctx);
  g_trace_ctx = nullptr;
#endif
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

bool DecApp::parseCfg(int argc, char *argv[])
{
  DecAppCfg decCfgParser;
  bool      ret;
  ret = decCfgParser.parseCfg(argc, argv, &m_cDecLib.m_decCfg);

  m_iMaxTemporalLayer      = m_cDecLib.m_decCfg.m_iMaxTemporalLayer;
  m_targetOutputLayerIdSet = m_cDecLib.m_decCfg.m_targetOutputLayerIdSet;

  return ret;
}

/**
 - create internal class
 - initialize internal class
 - until the end of the bitstream, call decoding function in DecApp class
 - delete allocated buffers
 - destroy internal class
 - returns the number of mismatching pictures
 */
uint32_t DecApp::decode()
{
  const DecCfg &decCfg = m_cDecLib.m_decCfg;
  int           poc;
  PicList      *picList   = nullptr;
  int           skipFrame = decCfg.m_iSkipFrame;

  std::ifstream bitstreamFile(decCfg.m_bitstreamFileName.c_str(), std::ifstream::in | std::ifstream::binary);
  if (!bitstreamFile)
  {
    EXIT("Failed to open bitstream file " << decCfg.m_bitstreamFileName.c_str() << " for reading");
  }

  InputByteStream bytestream(bitstreamFile);

  if (!decCfg.m_outputDecodedSEIMessagesFilename.empty() && decCfg.m_outputDecodedSEIMessagesFilename != "-")
  {
    m_seiMessageFileStream.open(decCfg.m_outputDecodedSEIMessagesFilename.c_str(), std::ios::out);
    if (!m_seiMessageFileStream.is_open() || !m_seiMessageFileStream.good())
    {
      EXIT("Unable to open file " << decCfg.m_outputDecodedSEIMessagesFilename.c_str()
                                  << " for writing decoded SEI messages");
    }
  }

  if (!decCfg.m_oplFilename.empty() && decCfg.m_oplFilename != "-")
  {
    m_oplFileStream.open(decCfg.m_oplFilename.c_str(), std::ios::out);
    if (!m_oplFileStream.is_open() || !m_oplFileStream.good())
    {
      EXIT("Unable to open file " << decCfg.m_oplFilename.c_str()
                                  << " to write an opl-file for conformance testing (see JVET-P2008 for details)");
    }
  }

  // create & initialize internal classes
  xCreateDecLib();

  m_iPOCLastDisplay += decCfg.m_iSkipFrame;      // set the last displayed POC correctly for skip forward.

  // clear contents of colour-remap-information-SEI output file
  if (!decCfg.m_colourRemapSEIFileName.empty())
  {
    std::ofstream ofile(decCfg.m_colourRemapSEIFileName.c_str());
    if (!ofile.good() || !ofile.is_open())
    {
      EXIT("Unable to open file " << decCfg.m_colourRemapSEIFileName.c_str()
                                  << " for writing colour-remap-information-SEI video");
    }
  }

  // clear contents of annotated-Regions-SEI output file
  if (!decCfg.m_annotatedRegionsSEIFileName.empty())
  {
    std::ofstream ofile(decCfg.m_annotatedRegionsSEIFileName.c_str());
    if (!ofile.good() || !ofile.is_open())
    {
      fprintf(stderr, "\nUnable to open file '%s' for writing annotated-Regions-SEI\n",
              decCfg.m_annotatedRegionsSEIFileName.c_str());
      exit(EXIT_FAILURE);
    }
  }

  // main decoder loop
  bool loopFiltered[MAX_VPS_LAYERS] = { false };

  bool bPicSkipped = false;

#if JVET_Z0120_SII_SEI_PROCESSING
  bool openedPostFile   = false;
  m_ShutterFilterEnable = !decCfg.m_shutterIntervalPostFileName
                             .empty();   // not apply shutter interval SEI processing if filename is not specified.
  m_cDecLib.setShutterFilterFlag(m_ShutterFilterEnable);
#endif

  bool isEosPresentInPu     = false;
  bool isEosPresentInLastPu = false;

  bool outputPicturePresentInBitstream = false;
  auto setOutputPicturePresentInStream = [&]()
  {
    if (!outputPicturePresentInBitstream)
    {
      PicList::iterator iterPic = picList->begin();
      while (!outputPicturePresentInBitstream && iterPic != picList->end())
      {
        Picture *pic = *(iterPic++);
        if (pic->m_neededForOutput)
        {
          outputPicturePresentInBitstream = true;
        }
      }
    }
  };

  m_cDecLib.setTOlsIdxExternalFlag(decCfg.m_tOlsIdxTidExternalSet);

  bool gdrRecoveryPeriod[MAX_NUM_LAYER_IDS] = { false };
  bool prevPicSkipped                       = true;
  int  lastNaluLayerId                      = -1;
  bool decodedSliceInAU                     = false;

  while (!!bitstreamFile)
  {
    InputNALUnit nalu;
    nalu.m_nalUnitType = NAL_UNIT_INVALID;

    // determine if next NAL unit will be the first one from a new picture
    bool bNewPicture = m_cDecLib.isNewPicture(&bitstreamFile, &bytestream);
    bool bNewAccessUnit =
      bNewPicture && decodedSliceInAU && m_cDecLib.isNewAccessUnit(bNewPicture, &bitstreamFile, &bytestream);
    if (!bNewPicture)
    {
      AnnexBStats stats = AnnexBStats();

      // find next NAL unit in stream
      byteStreamNALUnit(bytestream, nalu.getBitstream().getFifo(), stats);
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
        // read NAL unit header
        read(nalu);

        // flush output for first slice of an IDR picture
        if (m_cDecLib.getFirstSliceInPicture() &&
            (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
             nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP))
        {
          if (!m_cDecLib.getMixedNaluTypesInPicFlag())
          {
            m_newCLVS[nalu.m_nuhLayerId] = true;   // An IDR picture starts a new CLVS
            xFlushOutput(picList, nalu.m_nuhLayerId);
          }
          else
          {
            m_newCLVS[nalu.m_nuhLayerId] = false;
          }
        }
        else if (m_cDecLib.getFirstSliceInPicture() && nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA &&
                 isEosPresentInLastPu)
        {
          // A CRA that is immediately preceded by an EOS is a CLVSS
          m_newCLVS[nalu.m_nuhLayerId] = true;
          xFlushOutput(picList, nalu.m_nuhLayerId);
        }
        else if (m_cDecLib.getFirstSliceInPicture() && nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA &&
                 !isEosPresentInLastPu)
        {
          // A CRA that is not immediately precede by an EOS is not a CLVSS
          m_newCLVS[nalu.m_nuhLayerId] = false;
        }
        else if (m_cDecLib.getFirstSliceInPicture() && !isEosPresentInLastPu)
        {
          m_newCLVS[nalu.m_nuhLayerId] = false;
        }

        // parse NAL unit syntax if within target decoding layer
        if ((m_iMaxTemporalLayer < 0 || nalu.m_temporalId <= m_iMaxTemporalLayer) &&
            xIsNaluWithinTargetDecLayerIdSet(&nalu))
        {
          CHECK(nalu.m_temporalId > decCfg.m_iMaxTemporalLayer,
                "bitstream shall not include any NAL unit with TemporalId greater than HighestTid");
          if (decCfg.m_targetDecLayerIdSet.size())
          {
            CHECK(std::find(m_targetDecLayerIdSet.begin(), m_targetDecLayerIdSet.end(), nalu.m_nuhLayerId) ==
                    m_targetDecLayerIdSet.end(),
                  "bitstream shall not contain any other layers than included in the OLS with OlsIdx");
          }
          if (bPicSkipped)
          {
            if ((nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_TRAIL) ||
                (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA) ||
                (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_RASL) ||
                (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_RADL) ||
                (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL) ||
                (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP) ||
                (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_GDR))
            {
              if (decodedSliceInAU && m_cDecLib.isSliceNaluFirstInAU(true, nalu))
              {
                m_cDecLib.resetAccessUnitNals();
                m_cDecLib.resetAccessUnitApsNals();
                m_cDecLib.resetAccessUnitPicInfo();
              }
              bPicSkipped = false;
            }
          }

          int skipFrameCounter = skipFrame;
          m_cDecLib.decode(nalu, skipFrame, m_iPOCLastDisplay, decCfg.m_targetOlsIdx);

          if (prevPicSkipped && nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_GDR)
          {
            gdrRecoveryPeriod[nalu.m_nuhLayerId] = true;
          }

          if (skipFrameCounter == 1 &&
              (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_GDR || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA))
          {
            skipFrameCounter--;
          }

          if (skipFrame < skipFrameCounter &&
              ((nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_TRAIL) ||
               (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_RASL) ||
               (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_RADL) ||
               (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL) ||
               (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP) ||
               (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA) || (nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_GDR)))
          {
            if (decodedSliceInAU && m_cDecLib.isSliceNaluFirstInAU(true, nalu))
            {
              m_cDecLib.checkSeiInPictureUnit();
              m_cDecLib.resetPictureSeiNalus();
              m_cDecLib.checkAPSInPictureUnit();
              m_cDecLib.resetPictureUnitNals();
              m_cDecLib.resetAccessUnitSeiTids();
              m_cDecLib.checkSEIInAccessUnit();
              m_cDecLib.resetAccessUnitSeiPayLoadTypes();
              m_cDecLib.resetAccessUnitNals();
              m_cDecLib.resetAccessUnitApsNals();
              m_cDecLib.resetAccessUnitPicInfo();
            }
            bPicSkipped = true;
            skipFrame++;   // skipFrame count restore, the real decrement occur at the begin of next frame
          }

          if (nalu.m_nalUnitType == NAL_UNIT_OPI)
          {
            if (!m_cDecLib.m_mTidExternalSet && m_cDecLib.getOPI()->m_htidinfopresentflag)
            {
              m_iMaxTemporalLayer = m_cDecLib.getOPI()->m_opihtidplus1 - 1;
            }
            m_cDecLib.setHTidOpiSetFlag(m_cDecLib.getOPI()->m_htidinfopresentflag);
          }
          if (nalu.m_nalUnitType == NAL_UNIT_VPS)
          {
            m_cDecLib.deriveTargetOutputLayerSet(m_cDecLib.m_vps->m_targetOlsIdx);
            m_targetDecLayerIdSet    = m_cDecLib.m_vps->m_targetLayerIdSet;
            m_targetOutputLayerIdSet = m_cDecLib.m_vps->m_targetOutputLayerIdSet;
          }
          if (nalu.isSlice())
          {
            decodedSliceInAU = true;
          }
        }
        else
        {
          bPicSkipped = true;
          if (nalu.isSlice())
          {
            m_cDecLib.setFirstSliceInPicture(false);
          }
        }
      }

      if (nalu.isSlice() && nalu.m_nalUnitType != NAL_UNIT_CODED_SLICE_RASL)
      {
        prevPicSkipped = bPicSkipped;
      }

      // once an EOS NAL unit appears in the current PU, mark the variable isEosPresentInPu as true
      if (nalu.m_nalUnitType == NAL_UNIT_EOS)
      {
        isEosPresentInPu = true;
        m_newCLVS[nalu.m_nuhLayerId] =
          true;  // The presence of EOS means that the next picture is the beginning of new CLVS
      }
      // within the current PU, only EOS and EOB are allowed to be sent after an EOS nal unit
      if (isEosPresentInPu)
      {
        CHECK(nalu.m_nalUnitType != NAL_UNIT_EOS && nalu.m_nalUnitType != NAL_UNIT_EOB,
              "When an EOS NAL unit is present in a PU, it shall be the last NAL unit among all NAL units within the "
              "PU other than other EOS NAL units or an EOB NAL unit");
      }
      lastNaluLayerId = nalu.m_nuhLayerId;
    }
    else
    {
      nalu.m_nuhLayerId = lastNaluLayerId;
    }

    if (bNewPicture || !bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS)
    {
      if (!m_cDecLib.getFirstSliceInSequence(nalu.m_nuhLayerId) && !bPicSkipped)
      {
        if (!loopFiltered[nalu.m_nuhLayerId] || bitstreamFile)
        {
          m_cDecLib.executeLoopFilters();
          m_cDecLib.m_pic->copyAdaptedLumaClip();
          m_cDecLib.finishPicture(poc, picList, INFO, m_newCLVS[nalu.m_nuhLayerId]);
        }
        loopFiltered[nalu.m_nuhLayerId] = (nalu.m_nalUnitType == NAL_UNIT_EOS);
        if (nalu.m_nalUnitType == NAL_UNIT_EOS)
        {
          m_cDecLib.setFirstSliceInSequence(true, nalu.m_nuhLayerId);
        }

        m_cDecLib.updateAssociatedIRAP();
        m_cDecLib.updatePrevGDRInSameLayer();
        m_cDecLib.updatePrevIRAPAndGDRSubpic();

        if (gdrRecoveryPeriod[nalu.m_nuhLayerId])
        {
          if (m_cDecLib.getGDRRecoveryPocReached())
          {
            gdrRecoveryPeriod[nalu.m_nuhLayerId] = false;
          }
        }
      }
      else
      {
        m_cDecLib.setFirstSliceInPicture(true);
      }
    }

    if (picList)
    {
      if (gdrRecoveryPeriod[nalu.m_nuhLayerId]) // Suppress YUV and OPL output during GDR recovery
      {
        PicList::iterator iterPic = picList->begin();
        while (iterPic != picList->end())
        {
          Picture *pic = *(iterPic++);
          if (pic->m_layerId == nalu.m_nuhLayerId)
          {
            pic->m_neededForOutput = false;
          }
        }
      }

      BitDepths layerOutputBitDepth;

      PicList::iterator iterPicLayer = picList->begin();
      for (; iterPicLayer != picList->end(); ++iterPicLayer)
      {
        if ((*iterPicLayer)->m_layerId == nalu.m_nuhLayerId)
        {
          break;
        }
      }
      if (iterPicLayer != picList->end())
      {
        BitDepths &bitDepths = (*iterPicLayer)->m_bitDepths;

        for (auto channelType: { ChannelType::LUMA, ChannelType::CHROMA })
        {
          if (decCfg.m_outputBitDepth[channelType] == 0)
          {
            layerOutputBitDepth[channelType] = bitDepths[channelType];
          }
          else
          {
            layerOutputBitDepth[channelType] = decCfg.m_outputBitDepth[channelType];
          }
        }
        if (decCfg.m_packedYUVMode &&
            (layerOutputBitDepth[ChannelType::LUMA] != 10 && layerOutputBitDepth[ChannelType::LUMA] != 12))
        {
          EXIT("Invalid output bit-depth for packed YUV output, aborting\n");
        }

        if (!decCfg.m_reconFileName.empty() && !m_cVideoIOYuvReconFile[nalu.m_nuhLayerId].isOpen())
        {
          std::string reconFileName = decCfg.m_reconFileName;
          if (decCfg.m_reconFileName.compare("/dev/null") && m_cDecLib.m_vps != nullptr &&
              m_cDecLib.m_vps->m_maxLayers > 1 && xIsNaluWithinTargetOutputLayerIdSet(&nalu))
          {
            size_t      pos         = reconFileName.find_last_of('.');
            std::string layerString = std::string(".layer") + std::to_string(nalu.m_nuhLayerId);
            if (pos != std::string::npos)
            {
              reconFileName.insert(pos, layerString);
            }
            else
            {
              reconFileName.append(layerString);
            }
          }
          if ((m_cDecLib.m_vps != nullptr &&
               (m_cDecLib.m_vps->m_maxLayers == 1 || xIsNaluWithinTargetOutputLayerIdSet(&nalu))) ||
              m_cDecLib.m_vps == nullptr)
          {
            if (isY4mFileExt(reconFileName))
            {
              const auto sps        = picList->front()->m_cs->sps;
              int        frameRate  = 50;
              int        frameScale = 1;
              if (sps->m_generalHrdParametersPresentFlag)
              {
                const auto hrd                 = &sps->m_generalHrdParams;
                const auto olsHrdParam         = sps->m_olsHrdParams[sps->m_maxSubLayers - 1];
                int        elementDurationInTc = 1;
                if (olsHrdParam.m_fixedPicRateWithinCvsFlag)
                {
                  elementDurationInTc = olsHrdParam.m_elementDurationInTcMinus1 + 1;
                }
                else
                {
                  msg(WARNING,
                      "\nWarning: No fixed picture rate info is found in the bitstream, best guess is used.\n");
                }
                frameRate  = hrd->m_timeScale * elementDurationInTc;
                frameScale = hrd->m_numUnitsInTick;
                int gcd    = calcGcd(std::max(frameRate, frameScale), std::min(frameRate, frameScale));
                frameRate /= gcd;
                frameScale /= gcd;
              }
              else
              {
                msg(WARNING, "\nWarning: No frame rate info found in the bitstream, default 50 fps is used.\n");
              }
              const auto pps        = picList->front()->m_cs->pps;
              auto       confWindow = pps->m_conformanceWindow;
              const auto sx         = SPS::getWinUnitX(sps->m_chromaFormatIdc);
              const auto sy         = SPS::getWinUnitY(sps->m_chromaFormatIdc);
              const int  picWidth =
                pps->m_picWidthInLumaSamples - (confWindow.m_winLeftOffset + confWindow.m_winRightOffset) * sx;
              const int picHeight =
                pps->m_picHeightInLumaSamples - (confWindow.m_winTopOffset + confWindow.m_winBottomOffset) * sy;
              m_cVideoIOYuvReconFile[nalu.m_nuhLayerId].setOutputY4mInfo(picWidth, picHeight, frameRate, frameScale,
                                                                         layerOutputBitDepth[ChannelType::LUMA],
                                                                         sps->m_chromaFormatIdc);
            }
            m_cVideoIOYuvReconFile[nalu.m_nuhLayerId].open(reconFileName, true, layerOutputBitDepth,
                                                           layerOutputBitDepth, bitDepths);   // write mode
          }
        }
        // update file bitdepth shift if recon bitdepth changed between sequences
        for (auto channelType: { ChannelType::LUMA, ChannelType::CHROMA })
        {
          int reconBitdepth = (*iterPicLayer)->m_bitDepths[(ChannelType)channelType];
          int fileBitdepth  = m_cVideoIOYuvReconFile[nalu.m_nuhLayerId].getFileBitdepth(channelType);
          int bitdepthShift = m_cVideoIOYuvReconFile[nalu.m_nuhLayerId].getBitdepthShift(channelType);
          if (fileBitdepth + bitdepthShift != reconBitdepth)
          {
            m_cVideoIOYuvReconFile[nalu.m_nuhLayerId].setBitdepthShift(channelType, reconBitdepth - fileBitdepth);
          }
        }

        if (!decCfg.m_SEIFGSFileName.empty() && !m_videoIOYuvSEIFGSFile[nalu.m_nuhLayerId].isOpen())
        {
          std::string SEIFGSFileName = decCfg.m_SEIFGSFileName;
          if (decCfg.m_SEIFGSFileName.compare("/dev/null") && m_cDecLib.m_vps != nullptr &&
              m_cDecLib.m_vps->m_maxLayers > 1 && xIsNaluWithinTargetOutputLayerIdSet(&nalu))
          {
            size_t      pos         = SEIFGSFileName.find_last_of('.');
            std::string layerString = std::string(".layer") + std::to_string(nalu.m_nuhLayerId);
            if (pos != std::string::npos)
            {
              SEIFGSFileName.insert(pos, layerString);
            }
            else
            {
              SEIFGSFileName.append(layerString);
            }
          }
          if ((m_cDecLib.m_vps != nullptr &&
               (m_cDecLib.m_vps->m_maxLayers == 1 || xIsNaluWithinTargetOutputLayerIdSet(&nalu))) ||
              m_cDecLib.m_vps == nullptr)
          {
            m_videoIOYuvSEIFGSFile[nalu.m_nuhLayerId].open(SEIFGSFileName, true, layerOutputBitDepth,
                                                           layerOutputBitDepth, bitDepths);   // write mode
          }
        }
        // update file bitdepth shift if recon bitdepth changed between sequences
        if (!decCfg.m_SEIFGSFileName.empty())
        {
          for (const auto channelType: { ChannelType::LUMA, ChannelType::CHROMA })
          {
            int reconBitdepth = (*iterPicLayer)->m_bitDepths[(ChannelType)channelType];
            int fileBitdepth  = m_videoIOYuvSEIFGSFile[nalu.m_nuhLayerId].getFileBitdepth(channelType);
            int bitdepthShift = m_videoIOYuvSEIFGSFile[nalu.m_nuhLayerId].getBitdepthShift(channelType);
            if (fileBitdepth + bitdepthShift != reconBitdepth)
            {
              m_videoIOYuvSEIFGSFile[nalu.m_nuhLayerId].setBitdepthShift(channelType, reconBitdepth - fileBitdepth);
            }
          }
        }

        if (!decCfg.m_SEICTIFileName.empty() && !m_cVideoIOYuvSEICTIFile[nalu.m_nuhLayerId].isOpen())
        {
          std::string SEICTIFileName = decCfg.m_SEICTIFileName;
          if (decCfg.m_SEICTIFileName.compare("/dev/null") && m_cDecLib.m_vps != nullptr &&
              m_cDecLib.m_vps->m_maxLayers > 1 && xIsNaluWithinTargetOutputLayerIdSet(&nalu))
          {
            size_t pos = SEICTIFileName.find_last_of('.');
            if (pos != std::string::npos)
            {
              SEICTIFileName.insert(pos, std::to_string(nalu.m_nuhLayerId));
            }
            else
            {
              SEICTIFileName.append(std::to_string(nalu.m_nuhLayerId));
            }
          }
          if ((m_cDecLib.m_vps != nullptr &&
               (m_cDecLib.m_vps->m_maxLayers == 1 || xIsNaluWithinTargetOutputLayerIdSet(&nalu))) ||
              m_cDecLib.m_vps == nullptr)
          {
            m_cVideoIOYuvSEICTIFile[nalu.m_nuhLayerId].open(SEICTIFileName, true, layerOutputBitDepth,
                                                            layerOutputBitDepth, bitDepths);   // write mode
          }
        }

#if ENABLE_NNLF
        if (!decCfg.m_nnlfDumpBasename.empty() && !m_nnlfJsonFile.is_open())
        {
          const auto        sps = picList->front()->m_cs->sps;
          const auto        pps = picList->front()->m_cs->pps;
          const std::string basename =
            decCfg.m_nnlfDumpBasename.substr(decCfg.m_nnlfDumpBasename.find_last_of("/\\") + 1);
          const std::string bsname =
            decCfg.m_bitstreamFileName.substr(decCfg.m_bitstreamFileName.find_last_of("/\\") + 1);
          // open nnlf json file and write header information
          m_nnlfJsonFile.open(decCfg.m_nnlfDumpBasename + ".json");
          m_nnlfJsonFile << "{\n";
          for (auto &bufType: m_nnlfBufTypeMap)
          {
            m_nnlfJsonFile << " \"suffix" << bufType.second << "\": \"" << bufType.second << ".yuv\",\n";
          }
          m_nnlfJsonFile << " \"data\": [\n"
                         << "  {\n"
                         << "    \"bsname\": \"" << bsname << "\",\n"
                         << "    \"qp_base\": " << pps->m_picInitQPMinus26 + 26 << ",\n"
                         << "    \"basename\": \"" << basename << "\",\n"
                         << "    \"width\": " << sps->m_maxWidthInLumaSamples << ",\n"
                         << "    \"height\": " << sps->m_maxHeightInLumaSamples << ",\n";
          // open nnlf dump files
          for (auto &bufType: m_nnlfBufTypeMap)
          {
            m_nnlfDumpIoFiles[bufType.first] = new VideoIOYuv;
            m_nnlfDumpIoFiles[bufType.first]->open(decCfg.m_nnlfDumpBasename + bufType.second + ".yuv", true,
                                                   layerOutputBitDepth, layerOutputBitDepth, bitDepths);
          }
        }
#endif
      }
      if (!decCfg.m_annotatedRegionsSEIFileName.empty())
      {
        xOutputAnnotatedRegions(picList);
      }

#if JVET_Z0120_SII_SEI_PROCESSING
      PicList::iterator iterPic             = picList->begin();
      Picture          *pic                 = *(iterPic);
      SEIMessages       shutterIntervalInfo = getSeisByType(pic->m_SEIs, SEI::PayloadType::SHUTTER_INTERVAL_INFO);

      if (!decCfg.m_shutterIntervalPostFileName.empty())
      {
        bool                    hasValidSII = true;
        SEIShutterIntervalInfo *curSIIInfo  = nullptr;
        if ((pic->getPictureType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
             pic->getPictureType() == NAL_UNIT_CODED_SLICE_IDR_N_LP) &&
            m_newCLVS[nalu.m_nuhLayerId])
        {
          IdrSiiInfo curSII;
          curSII.m_picPoc = pic->m_poc;

          curSII.m_isValidSii                             = false;
          curSII.m_siiInfo.m_siiEnabled                   = false;
          curSII.m_siiInfo.m_siiNumUnitsInShutterInterval = 0;
          curSII.m_siiInfo.m_siiTimeScale                 = 0;
          curSII.m_siiInfo.m_siiMaxSubLayersMinus1        = 0;
          curSII.m_siiInfo.m_siiFixedSIwithinCLVS         = 0;

          if (shutterIntervalInfo.size() > 0)
          {
            SEIShutterIntervalInfo *seiShutterIntervalInfo = (SEIShutterIntervalInfo *)*(shutterIntervalInfo.begin());
            curSII.m_isValidSii                            = true;

            curSII.m_siiInfo.m_siiEnabled                   = seiShutterIntervalInfo->m_siiEnabled;
            curSII.m_siiInfo.m_siiNumUnitsInShutterInterval = seiShutterIntervalInfo->m_siiNumUnitsInShutterInterval;
            curSII.m_siiInfo.m_siiTimeScale                 = seiShutterIntervalInfo->m_siiTimeScale;
            curSII.m_siiInfo.m_siiMaxSubLayersMinus1        = seiShutterIntervalInfo->m_siiMaxSubLayersMinus1;
            curSII.m_siiInfo.m_siiFixedSIwithinCLVS         = seiShutterIntervalInfo->m_siiFixedSIwithinCLVS;
            curSII.m_siiInfo.m_siiSubLayerNumUnitsInSI.clear();
            for (int i = 0; i < seiShutterIntervalInfo->m_siiSubLayerNumUnitsInSI.size(); i++)
            {
              curSII.m_siiInfo.m_siiSubLayerNumUnitsInSI.push_back(
                seiShutterIntervalInfo->m_siiSubLayerNumUnitsInSI[i]);
            }

            uint32_t tmpInfo = (uint32_t)(m_activeSiiInfo.size() + 1);
            m_activeSiiInfo.insert(std::pair<uint32_t, IdrSiiInfo>(tmpInfo, curSII));
            curSIIInfo = seiShutterIntervalInfo;
          }
          else
          {
            curSII.m_isValidSii = false;
            hasValidSII         = false;
            uint32_t tmpInfo    = (uint32_t)(m_activeSiiInfo.size() + 1);
            m_activeSiiInfo.insert(std::pair<uint32_t, IdrSiiInfo>(tmpInfo, curSII));
          }
        }
        else
        {
          if (m_activeSiiInfo.size() == 1)
          {
            curSIIInfo = &(m_activeSiiInfo.begin()->second.m_siiInfo);
          }
          else
          {
            bool isLast = true;
            for (int i = 1; i < m_activeSiiInfo.size() + 1; i++)
            {
              if (pic->m_poc <= m_activeSiiInfo.at(i).m_picPoc)
              {
                if (m_activeSiiInfo[i - 1].m_isValidSii)
                {
                  curSIIInfo = &(m_activeSiiInfo.at(i - 1).m_siiInfo);
                }
                else
                {
                  hasValidSII = false;
                }
                isLast = false;
                break;
              }
            }
            if (isLast)
            {
              uint32_t tmpInfo = (uint32_t)(m_activeSiiInfo.size());
              curSIIInfo       = &(m_activeSiiInfo.at(tmpInfo).m_siiInfo);
            }
          }
        }

        if (hasValidSII)
        {
          if (!curSIIInfo->m_siiFixedSIwithinCLVS)
          {
            uint32_t siiMaxSubLayersMinus1 = curSIIInfo->m_siiMaxSubLayersMinus1;
            uint32_t numUnitsLFR           = curSIIInfo->m_siiSubLayerNumUnitsInSI[0];
            uint32_t numUnitsHFR           = curSIIInfo->m_siiSubLayerNumUnitsInSI[siiMaxSubLayersMinus1];

            int  blending_ratio        = (numUnitsLFR / numUnitsHFR);
            bool checkEqualValuesOfSFR = true;
            bool checkSubLayerSI       = false;
            int  i;

            // supports only the case of SFR = HFR / 2
            if (curSIIInfo->m_siiSubLayerNumUnitsInSI[siiMaxSubLayersMinus1] <
                curSIIInfo->m_siiSubLayerNumUnitsInSI[siiMaxSubLayersMinus1 - 1])
            {
              checkSubLayerSI = true;
            }
            else
            {
              fprintf(stderr,
                      "Warning: Shutter Interval SEI message processing is disabled due to SFR != (HFR / 2) \n");
            }
            // check shutter interval for all sublayer remains same for SFR pictures
            for (i = 1; i < siiMaxSubLayersMinus1; i++)
            {
              if (curSIIInfo->m_siiSubLayerNumUnitsInSI[0] != curSIIInfo->m_siiSubLayerNumUnitsInSI[i])
              {
                checkEqualValuesOfSFR = false;
              }
            }
            if (!checkEqualValuesOfSFR)
            {
              fprintf(stderr,
                      "Warning: Shutter Interval SEI message processing is disabled when shutter interval is not same "
                      "for SFR sublayers \n");
            }
            if (checkSubLayerSI && checkEqualValuesOfSFR)
            {
              m_ShutterFilterEnable = (numUnitsLFR == blending_ratio * numUnitsHFR);
              setBlendingRatio(blending_ratio);
            }
            else
            {
              m_ShutterFilterEnable = false;
            }

            const SPS *activeSPS = picList->front()->m_cs->sps;

            if (numUnitsLFR == blending_ratio * numUnitsHFR && activeSPS->m_maxSubLayers == 1 &&
                activeSPS->m_maxDecPicBuffering[0] == 1)
            {
              fprintf(stderr,
                      "Warning: Shutter Interval SEI message processing is disabled for single TempLayer and single "
                      "frame in DPB\n");
              m_ShutterFilterEnable = false;
            }
          }
          else
          {
            fprintf(stderr,
                    "Warning: Shutter Interval SEI message processing is disabled for fixed shutter interval case\n");
            m_ShutterFilterEnable = false;
          }
        }
        else
        {
          fprintf(stderr, "Warning: Shutter Interval information should be specified in SII-SEI message\n");
          m_ShutterFilterEnable = false;
        }
      }

      if (iterPicLayer != picList->end())
      {
        if ((!decCfg.m_shutterIntervalPostFileName.empty()) && (!openedPostFile) && m_ShutterFilterEnable)
        {
          BitDepths    &bitDepths = (*iterPicLayer)->m_bitDepths;
          std::ofstream ofile(decCfg.m_shutterIntervalPostFileName.c_str());
          if (!ofile.good() || !ofile.is_open())
          {
            fprintf(stderr, "\nUnable to open file '%s' for writing shutter-interval-SEI video\n",
                    decCfg.m_shutterIntervalPostFileName.c_str());
            exit(EXIT_FAILURE);
          }
          m_cTVideoIOYuvSIIPostFile.open(decCfg.m_shutterIntervalPostFileName, true, layerOutputBitDepth,
                                         layerOutputBitDepth,
                                         bitDepths);   // write mode
          openedPostFile = true;
        }
      }
#endif

      // write reconstruction to file
      if (bNewPicture)
      {
        setOutputPicturePresentInStream();
        xWriteOutput(picList, nalu.m_temporalId);
      }
      if (nalu.m_nalUnitType == NAL_UNIT_EOS)
      {
        if (!decCfg.m_annotatedRegionsSEIFileName.empty() && bNewPicture)
        {
          xOutputAnnotatedRegions(picList);
        }
        setOutputPicturePresentInStream();
        xWriteOutput(picList, nalu.m_temporalId);
        m_cDecLib.setFirstSliceInPicture(false);
      }
      // write reconstruction to file -- for additional bumping as defined in C.5.2.3
      if (!bNewPicture &&
          ((nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_TRAIL && nalu.m_nalUnitType <= NAL_UNIT_RESERVED_IRAP_VCL_15) ||
           (nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL && nalu.m_nalUnitType <= NAL_UNIT_CODED_SLICE_GDR)))
      {
        setOutputPicturePresentInStream();
        xWriteOutput(picList, nalu.m_temporalId);
      }
    }
    if (bNewPicture)
    {
      m_cDecLib.checkSeiInPictureUnit();
      m_cDecLib.resetPictureSeiNalus();
      // reset the EOS present status for the next PU check
      isEosPresentInLastPu = isEosPresentInPu;
      isEosPresentInPu     = false;
    }
    if (bNewPicture || !bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS)
    {
      m_cDecLib.checkAPSInPictureUnit();
      m_cDecLib.resetPictureUnitNals();
    }
    if (bNewAccessUnit || !bitstreamFile)
    {
      m_cDecLib.CheckNoOutputPriorPicFlagsInAccessUnit();
      m_cDecLib.resetAccessUnitNoOutputPriorPicFlags();
      m_cDecLib.checkLayerIdIncludedInCvss();
      m_cDecLib.checkSEIInAccessUnit();
      m_cDecLib.resetAccessUnitNestedSliSeiInfo();
      m_cDecLib.resetIsFirstAuInCvs();
      m_cDecLib.resetAccessUnitEos();
      m_cDecLib.resetAudIrapOrGdrAuFlag();
    }
    if (bNewAccessUnit)
    {
      decodedSliceInAU = false;
      m_cDecLib.checkTidLayerIdInAccessUnit();
      m_cDecLib.resetAccessUnitSeiTids();
      m_cDecLib.resetAccessUnitSeiPayLoadTypes();
      m_cDecLib.checkSeiContentInAccessUnit();
      m_cDecLib.resetAccessUnitSeiNalus();
      m_cDecLib.resetAccessUnitNals();
      m_cDecLib.resetAccessUnitApsNals();
      m_cDecLib.resetAccessUnitPicInfo();
    }
  }
  if (!decCfg.m_annotatedRegionsSEIFileName.empty())
  {
    xOutputAnnotatedRegions(picList);
  }
  // May need to check again one more time as in case one the bitstream has only one picture, the first check may miss
  // it
  setOutputPicturePresentInStream();
  CHECK(!outputPicturePresentInBitstream,
        "It is required that there shall be at least one picture with PictureOutputFlag equal to 1 in the bitstream")

  m_cDecLib.applyNnPostFilter();

  xFlushOutput(picList);

#if JVET_Z0120_SII_SEI_PROCESSING
  if (!decCfg.m_shutterIntervalPostFileName.empty() && m_ShutterFilterEnable)
  {
    m_cTVideoIOYuvSIIPostFile.close();
  }
#endif

  // get the number of checksum errors
  uint32_t nRet = m_cDecLib.getNumberOfChecksumErrorsDetected();

  // delete buffers
  m_cDecLib.deletePicBuffer();
  // destroy internal classes
  xDestroyDecLib();

#if RExt__DECODER_DEBUG_STATISTICS
  CodingStatistics::DestroyInstance();
#endif

  destroyROM();

  return nRet;
}

void DecApp::writeLineToOutputLog(Picture *pic)
{
  if (m_oplFileStream.is_open() && m_oplFileStream.good())
  {
    const SPS    *sps             = pic->m_cs->sps;
    ChromaFormat  chromaFormatIdc = sps->m_chromaFormatIdc;
    const Window &conf            = pic->m_conformanceWindow;
    const int     leftOffset      = conf.m_winLeftOffset * SPS::getWinUnitX(chromaFormatIdc);
    const int     rightOffset     = conf.m_winRightOffset * SPS::getWinUnitX(chromaFormatIdc);
    const int     topOffset       = conf.m_winTopOffset * SPS::getWinUnitY(chromaFormatIdc);
    const int     bottomOffset    = conf.m_winBottomOffset * SPS::getWinUnitY(chromaFormatIdc);
    PictureHash   recon_digest;
    auto numChar = calcMD5WithCropping(((const Picture *)pic)->getRecoBuf(), recon_digest, sps->m_bitDepths, leftOffset,
                                       rightOffset, topOffset, bottomOffset);

    const int croppedWidth  = pic->lwidth() - leftOffset - rightOffset;
    const int croppedHeight = pic->lheight() - topOffset - bottomOffset;

    m_oplFileStream << std::setw(3) << pic->m_layerId << ",";
    m_oplFileStream << std::setw(8) << pic->m_poc << "," << std::setw(5) << croppedWidth << "," << std::setw(5)
                    << croppedHeight << "," << hashToString(recon_digest, numChar) << "\n";
  }
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

void DecApp::xCreateDecLib()
{
  initROM();

  // create decoder class
  m_cDecLib.create();

  // initialize decoder class
  m_cDecLib.init(
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
    m_cacheCfgFile
#endif
  );

  const DecCfg &decCfg = m_cDecLib.m_decCfg;

  m_cDecLib.setDecodedPictureHashSEIEnabled(decCfg.m_decodedPictureHashSEIEnabled);

  if (!decCfg.m_outputDecodedSEIMessagesFilename.empty())
  {
    std::ostream &os = m_seiMessageFileStream.is_open() ? m_seiMessageFileStream : std::cout;
    m_cDecLib.setDecodedSEIMessageOutputStream(&os);
  }
  m_cDecLib.m_targetSubPicIdx = decCfg.m_targetSubPicIdx;
  m_cDecLib.initScalingList();
}

void DecApp::xDestroyDecLib()
{
  const DecCfg &decCfg = m_cDecLib.m_decCfg;

  if (!decCfg.m_reconFileName.empty())
  {
    for (auto &recFile: m_cVideoIOYuvReconFile)
    {
      recFile.second.close();
    }
  }
  if (!decCfg.m_SEIFGSFileName.empty())
  {
    for (auto &recFile: m_videoIOYuvSEIFGSFile)
    {
      recFile.second.close();
    }
  }
  if (!decCfg.m_SEICTIFileName.empty())
  {
    for (auto &recFile: m_cVideoIOYuvSEICTIFile)
    {
      recFile.second.close();
    }
  }

#if ENABLE_NNLF
  if (m_nnlfJsonFile.is_open())
  {
    m_nnlfJsonFile << "    \"data_count\": " << m_nnlfDumpCount << "\n"
                   << "  }\n"
                   << " ]\n"
                   << "}";
    m_nnlfJsonFile.close();
  }
  for (int i = 0; i < NUM_PIC_TYPES; i++)
  {
    if (m_nnlfDumpIoFiles[i])
    {
      m_nnlfDumpIoFiles[i]->close();
      delete m_nnlfDumpIoFiles[i];
      m_nnlfDumpIoFiles[i] = nullptr;
    }
  }
#endif

  // destroy decoder class
  m_cDecLib.destroy();
}

/** \param picList list of pictures to be written to file
    \param tId       temporal sub-layer ID
 */
void DecApp::xWriteOutput(PicList *picList, uint32_t tId)
{
  if (picList->empty())
  {
    return;
  }

  const DecCfg     &decCfg                 = m_cDecLib.m_decCfg;
  PicList::iterator iterPic                = picList->begin();
  int               numPicsNotYetDisplayed = 0;
  int               dpbFullness            = 0;
  uint32_t          maxNumReorderPicsHighestTid;
  uint32_t          maxDecPicBufferingHighestTid;
  const VPS        *referredVPS = picList->front()->m_cs->vps;

  if (referredVPS == nullptr || referredVPS->m_numLayersInOls[referredVPS->m_targetOlsIdx] == 1)
  {
    const SPS *activeSPS         = (picList->front()->m_cs->sps);
    const int  temporalId        = (m_iMaxTemporalLayer == -1 || m_iMaxTemporalLayer >= activeSPS->m_maxSubLayers)
              ? activeSPS->m_maxSubLayers - 1
              : m_iMaxTemporalLayer;
    maxNumReorderPicsHighestTid  = activeSPS->m_maxNumReorderPics[temporalId];
    maxDecPicBufferingHighestTid = activeSPS->m_maxDecPicBuffering[temporalId];
  }
  else
  {
    const int temporalId         = (m_iMaxTemporalLayer == -1 || m_iMaxTemporalLayer >= referredVPS->m_vpsMaxSubLayers)
              ? referredVPS->m_vpsMaxSubLayers - 1
              : m_iMaxTemporalLayer;
    maxNumReorderPicsHighestTid  = referredVPS->getMaxNumReorderPics(temporalId);
    maxDecPicBufferingHighestTid = referredVPS->getMaxDecPicBuffering(temporalId);
  }

  while (iterPic != picList->end())
  {
    Picture *pic = *(iterPic);
    if (pic->m_neededForOutput && pic->m_poc >= m_iPOCLastDisplay)
    {
      numPicsNotYetDisplayed++;
      dpbFullness++;
    }
    else if (pic->m_referenced)
    {
      dpbFullness++;
    }
    iterPic++;
  }

  iterPic = picList->begin();

  if (numPicsNotYetDisplayed >= 2)
  {
    iterPic++;
  }

  Picture *pic = *(iterPic);
  if (numPicsNotYetDisplayed >= 2 && pic->m_fieldPic)   // Field Decoding
  {
    PicList::iterator endPic = picList->end();
    endPic--;
    iterPic = picList->begin();
    while (iterPic != endPic)
    {
      Picture *picTop = *(iterPic);
      iterPic++;
      PicList::iterator iterPic2 = iterPic;
      while (iterPic2 != picList->end())
      {
        if ((*iterPic2)->m_layerId == picTop->m_layerId && (*iterPic2)->m_fieldPic &&
            (*iterPic2)->m_topField != picTop->m_topField)
        {
          break;
        }
        iterPic2++;
      }
      if (iterPic2 == picList->end())
      {
        continue;
      }

      Picture *picBottom = *(iterPic2);

      if (picTop->m_neededForOutput && picBottom->m_neededForOutput &&
          (numPicsNotYetDisplayed > maxNumReorderPicsHighestTid || dpbFullness > maxDecPicBufferingHighestTid) &&
          picBottom->m_poc >= m_iPOCLastDisplay)
      {
        // write to file
        numPicsNotYetDisplayed = numPicsNotYetDisplayed - 2;
        if (!decCfg.m_reconFileName.empty())
        {
          const Window &conf  = picTop->m_conformanceWindow;
          const bool    isTff = picTop->m_topField;

          bool display = true;

          if (display)
          {
            m_cVideoIOYuvReconFile[picTop->m_layerId].write(
              picTop->getRecoBuf(), picBottom->getRecoBuf(), decCfg.m_outputColourSpaceConvert,
              false,   // TODO: m_packedYUVMode,
              conf.m_winLeftOffset * SPS::getWinUnitX(picTop->m_cs->sps->m_chromaFormatIdc),
              conf.m_winRightOffset * SPS::getWinUnitX(picTop->m_cs->sps->m_chromaFormatIdc),
              conf.m_winTopOffset * SPS::getWinUnitY(picTop->m_cs->sps->m_chromaFormatIdc),
              conf.m_winBottomOffset * SPS::getWinUnitY(picTop->m_cs->sps->m_chromaFormatIdc), ChromaFormat::UNDEFINED,
              isTff);
          }
        }
        writeLineToOutputLog(picTop);
        writeLineToOutputLog(picBottom);

        // update POC of display order
        m_iPOCLastDisplay = picBottom->m_poc;

        // erase non-referenced picture in the reference picture list after display
        if (!picTop->m_referenced && picTop->m_reconstructed)
        {
          picTop->m_reconstructed = false;
        }
        if (!picBottom->m_referenced && picBottom->m_reconstructed)
        {
          picBottom->m_reconstructed = false;
        }
        picTop->m_neededForOutput    = false;
        picBottom->m_neededForOutput = false;
      }
    }
  }
  else if (!pic->m_fieldPic)   // Frame Decoding
  {
    iterPic = picList->begin();

    while (iterPic != picList->end())
    {
      pic = *(iterPic);

      if (pic->m_neededForOutput && pic->m_poc >= m_iPOCLastDisplay &&
          (numPicsNotYetDisplayed > maxNumReorderPicsHighestTid || dpbFullness > maxDecPicBufferingHighestTid))
      {
        // write to file
        numPicsNotYetDisplayed--;
        if (!pic->m_referenced)
        {
          dpbFullness--;
        }

        if (!decCfg.m_reconFileName.empty())
        {
          const Window &conf            = pic->m_conformanceWindow;
          ChromaFormat  chromaFormatIdc = pic->m_chromaFormatIdc;
          if (decCfg.m_upscaledOutput)
          {
            const SPS *sps = pic->m_cs->sps;
            m_cVideoIOYuvReconFile[pic->m_layerId].writeUpscaledPicture(
              *sps, *pic->m_cs->pps, pic->getRecoBuf(), decCfg.m_outputColourSpaceConvert, decCfg.m_packedYUVMode,
              decCfg.m_upscaledOutput, ChromaFormat::UNDEFINED, decCfg.m_clipOutputVideoToRec709Range,
              decCfg.m_upscaleFilterForDisplay);
          }
          else
          {
            m_cVideoIOYuvReconFile[pic->m_layerId].write(
              pic->getRecoBuf().get(COMP_Y).width, pic->getRecoBuf().get(COMP_Y).height, pic->getRecoBuf(),
              decCfg.m_outputColourSpaceConvert, decCfg.m_packedYUVMode,
              conf.m_winLeftOffset * SPS::getWinUnitX(chromaFormatIdc),
              conf.m_winRightOffset * SPS::getWinUnitX(chromaFormatIdc),
              conf.m_winTopOffset * SPS::getWinUnitY(chromaFormatIdc),
              conf.m_winBottomOffset * SPS::getWinUnitY(chromaFormatIdc), ChromaFormat::UNDEFINED,
              decCfg.m_clipOutputVideoToRec709Range);
          }
        }
        // Perform FGS on decoded frame and write to output FGS file
        if (!decCfg.m_SEIFGSFileName.empty())
        {
          const Window &conf            = pic->m_conformanceWindow;
          const SPS    *sps             = pic->m_cs->sps;
          ChromaFormat  chromaFormatIdc = sps->m_chromaFormatIdc;
          if (decCfg.m_upscaledOutput)
          {
            m_videoIOYuvSEIFGSFile[pic->m_layerId].writeUpscaledPicture(
              *sps, *pic->m_cs->pps, pic->getDisplayBufFG(), decCfg.m_outputColourSpaceConvert, decCfg.m_packedYUVMode,
              decCfg.m_upscaledOutput, ChromaFormat::UNDEFINED, decCfg.m_clipOutputVideoToRec709Range,
              decCfg.m_upscaleFilterForDisplay);
          }
          else
          {
            m_videoIOYuvSEIFGSFile[pic->m_layerId].write(
              pic->getRecoBuf().get(COMP_Y).width, pic->getRecoBuf().get(COMP_Y).height, pic->getDisplayBufFG(),
              decCfg.m_outputColourSpaceConvert, decCfg.m_packedYUVMode,
              conf.m_winLeftOffset * SPS::getWinUnitX(chromaFormatIdc),
              conf.m_winRightOffset * SPS::getWinUnitX(chromaFormatIdc),
              conf.m_winTopOffset * SPS::getWinUnitY(chromaFormatIdc),
              conf.m_winBottomOffset * SPS::getWinUnitY(chromaFormatIdc), ChromaFormat::UNDEFINED,
              decCfg.m_clipOutputVideoToRec709Range);
          }
        }

#if JVET_Z0120_SII_SEI_PROCESSING
        if (!decCfg.m_shutterIntervalPostFileName.empty() && m_ShutterFilterEnable)
        {
          int blendingRatio = getBlendingRatio();
          pic->xOutputPostFilteredPic(pic, picList, blendingRatio);

          const Window &conf            = pic->m_conformanceWindow;
          const SPS    *sps             = pic->m_cs->sps;
          ChromaFormat  chromaFormatIdc = sps->m_chromaFormatIdc;

          m_cTVideoIOYuvSIIPostFile.write(pic->getPostRecBuf().get(COMP_Y).width,
                                          pic->getPostRecBuf().get(COMP_Y).height, pic->getPostRecBuf(),
                                          decCfg.m_outputColourSpaceConvert, decCfg.m_packedYUVMode,
                                          conf.m_winLeftOffset * SPS::getWinUnitX(chromaFormatIdc),
                                          conf.m_winRightOffset * SPS::getWinUnitX(chromaFormatIdc),
                                          conf.m_winTopOffset * SPS::getWinUnitY(chromaFormatIdc),
                                          conf.m_winBottomOffset * SPS::getWinUnitY(chromaFormatIdc),
                                          ChromaFormat::UNDEFINED, decCfg.m_clipOutputVideoToRec709Range);
        }
#endif

        // Perform CTI on decoded frame and write to output CTI file
        if (!decCfg.m_SEICTIFileName.empty())
        {
          const Window &conf            = pic->m_conformanceWindow;
          const SPS    *sps             = pic->m_cs->sps;
          ChromaFormat  chromaFormatIdc = sps->m_chromaFormatIdc;
          if (decCfg.m_upscaledOutput)
          {
            m_cVideoIOYuvSEICTIFile[pic->m_layerId].writeUpscaledPicture(
              *sps, *pic->m_cs->pps, pic->getDisplayBuf(), decCfg.m_outputColourSpaceConvert, decCfg.m_packedYUVMode,
              decCfg.m_upscaledOutput, ChromaFormat::UNDEFINED, decCfg.m_clipOutputVideoToRec709Range,
              decCfg.m_upscaleFilterForDisplay);
          }
          else
          {
            m_cVideoIOYuvSEICTIFile[pic->m_layerId].write(
              pic->getRecoBuf().get(COMP_Y).width, pic->getRecoBuf().get(COMP_Y).height, pic->getDisplayBuf(),
              decCfg.m_outputColourSpaceConvert, decCfg.m_packedYUVMode,
              conf.m_winLeftOffset * SPS::getWinUnitX(chromaFormatIdc),
              conf.m_winRightOffset * SPS::getWinUnitX(chromaFormatIdc),
              conf.m_winTopOffset * SPS::getWinUnitY(chromaFormatIdc),
              conf.m_winBottomOffset * SPS::getWinUnitY(chromaFormatIdc), ChromaFormat::UNDEFINED,
              decCfg.m_clipOutputVideoToRec709Range);
          }
        }

#if ENABLE_NNLF
        xNnlfDumpPicData(*pic);
#endif
        writeLineToOutputLog(pic);

        // update POC of display order
        m_iPOCLastDisplay = pic->m_poc;

        // erase non-referenced picture in the reference picture list after display
        if (!pic->m_referenced && pic->m_reconstructed)
        {
          pic->m_reconstructed = false;
        }
        pic->m_neededForOutput = false;
      }

      iterPic++;
    }
  }
}

#if ENABLE_NNLF
void DecApp::xNnlfDumpPicData(Picture &pic)
{
  if (m_nnlfJsonFile.is_open())
  {
    const DecCfg      &decCfg          = m_cDecLib.m_decCfg;
    const ChromaFormat chromaFormatIdc = pic.m_chromaFormatIdc;
    const Window      &conf            = pic.m_conformanceWindow;
    // write picture binary data
    for (auto &bufType: m_nnlfBufTypeMap)
    {
      const int type = bufType.first;
      if (m_nnlfDumpIoFiles[type] && m_nnlfDumpIoFiles[type]->isOpen())
      {
        VideoIOYuv       &dumpFile = *m_nnlfDumpIoFiles[type];
        const CPelUnitBuf buf      = pic.m_bufs[type];
        dumpFile.write(buf.get(COMP_Y).width, buf.get(COMP_Y).height, buf, decCfg.m_outputColourSpaceConvert,
                       decCfg.m_packedYUVMode, conf.m_winLeftOffset * SPS::getWinUnitX(chromaFormatIdc),
                       conf.m_winRightOffset * SPS::getWinUnitX(chromaFormatIdc),
                       conf.m_winTopOffset * SPS::getWinUnitY(chromaFormatIdc),
                       conf.m_winBottomOffset * SPS::getWinUnitY(chromaFormatIdc), ChromaFormat::UNDEFINED,
                       decCfg.m_clipOutputVideoToRec709Range);
      }
    }
    m_nnlfDumpCount += 1;
  }
}
#endif

/** \param picList list of pictures to be written to file
 */
void DecApp::xFlushOutput(PicList *picList, const int layerId)
{
  if (!picList || picList->empty())
  {
    return;
  }

  const DecCfg     &decCfg  = m_cDecLib.m_decCfg;
  PicList::iterator iterPic = picList->begin();

  iterPic      = picList->begin();
  Picture *pic = *(iterPic);

  if (pic->m_fieldPic)   // Field Decoding
  {
    PicList::iterator endPic = picList->end();
    while (iterPic != endPic)
    {
      Picture *picTop = *iterPic;
      iterPic++;

      if (picTop == nullptr || (picTop->m_layerId != layerId && layerId != NOT_VALID))
      {
        continue;
      }

      PicList::iterator iterPic2 = iterPic;
      while (iterPic2 != endPic)
      {
        if ((*iterPic2) != nullptr && (*iterPic2)->m_layerId == picTop->m_layerId && (*iterPic2)->m_fieldPic &&
            (*iterPic2)->m_topField != picTop->m_topField)
        {
          break;
        }
        iterPic2++;
      }
      Picture *picBottom = iterPic2 == endPic ? nullptr : *iterPic2;

      if (picBottom != nullptr && picTop->m_neededForOutput && picBottom->m_neededForOutput)
      {
        // write to file
        if (!decCfg.m_reconFileName.empty())
        {
          const Window &conf  = picTop->m_conformanceWindow;
          const bool    isTff = picTop->m_topField;

          m_cVideoIOYuvReconFile[picTop->m_layerId].write(
            picTop->getRecoBuf(), picBottom->getRecoBuf(), decCfg.m_outputColourSpaceConvert,
            false,   // TODO: m_packedYUVMode,
            conf.m_winLeftOffset * SPS::getWinUnitX(picTop->m_cs->sps->m_chromaFormatIdc),
            conf.m_winRightOffset * SPS::getWinUnitX(picTop->m_cs->sps->m_chromaFormatIdc),
            conf.m_winTopOffset * SPS::getWinUnitY(picTop->m_cs->sps->m_chromaFormatIdc),
            conf.m_winBottomOffset * SPS::getWinUnitY(picTop->m_cs->sps->m_chromaFormatIdc), ChromaFormat::UNDEFINED,
            isTff);
        }
        writeLineToOutputLog(picTop);
        writeLineToOutputLog(picBottom);
        // update POC of display order
        m_iPOCLastDisplay = picBottom->m_poc;

        // erase non-referenced picture in the reference picture list after display
        if (!picTop->m_referenced && picTop->m_reconstructed)
        {
          picTop->m_reconstructed = false;
        }
        if (!picBottom->m_referenced && picBottom->m_reconstructed)
        {
          picBottom->m_reconstructed = false;
        }
        picTop->m_neededForOutput    = false;
        picBottom->m_neededForOutput = false;

        picTop->destroy();
        delete picTop;
        picBottom->destroy();
        delete picBottom;
        iterPic--;
        *iterPic = nullptr;
        iterPic++;
        *iterPic2 = nullptr;
      }
      else
      {
        picTop->destroy();
        delete picTop;
        iterPic--;
        *iterPic = nullptr;
        iterPic++;
      }
    }
  }
  else   // Frame decoding
  {
    while (iterPic != picList->end())
    {
      pic = *(iterPic);

      if (pic->m_layerId != layerId && layerId != NOT_VALID)
      {
        iterPic++;
        continue;
      }

      if (pic->m_neededForOutput)
      {
        // write to file
        if (!decCfg.m_reconFileName.empty())
        {
          const Window &conf            = pic->m_conformanceWindow;
          ChromaFormat  chromaFormatIdc = pic->m_chromaFormatIdc;
          if (decCfg.m_upscaledOutput)
          {
            const SPS *sps = pic->m_cs->sps;
            m_cVideoIOYuvReconFile[pic->m_layerId].writeUpscaledPicture(
              *sps, *pic->m_cs->pps, pic->getRecoBuf(), decCfg.m_outputColourSpaceConvert, decCfg.m_packedYUVMode,
              decCfg.m_upscaledOutput, ChromaFormat::UNDEFINED, decCfg.m_clipOutputVideoToRec709Range,
              decCfg.m_upscaleFilterForDisplay);
          }
          else
          {
            m_cVideoIOYuvReconFile[pic->m_layerId].write(
              pic->getRecoBuf().get(COMP_Y).width, pic->getRecoBuf().get(COMP_Y).height, pic->getRecoBuf(),
              decCfg.m_outputColourSpaceConvert, decCfg.m_packedYUVMode,
              conf.m_winLeftOffset * SPS::getWinUnitX(chromaFormatIdc),
              conf.m_winRightOffset * SPS::getWinUnitX(chromaFormatIdc),
              conf.m_winTopOffset * SPS::getWinUnitY(chromaFormatIdc),
              conf.m_winBottomOffset * SPS::getWinUnitY(chromaFormatIdc), ChromaFormat::UNDEFINED,
              decCfg.m_clipOutputVideoToRec709Range);
          }
        }
        // Perform FGS on decoded frame and write to output FGS file
        if (!decCfg.m_SEIFGSFileName.empty())
        {
          const Window &conf            = pic->m_conformanceWindow;
          const SPS    *sps             = pic->m_cs->sps;
          ChromaFormat  chromaFormatIdc = sps->m_chromaFormatIdc;
          if (decCfg.m_upscaledOutput)
          {
            m_videoIOYuvSEIFGSFile[pic->m_layerId].writeUpscaledPicture(
              *sps, *pic->m_cs->pps, pic->getDisplayBufFG(), decCfg.m_outputColourSpaceConvert, decCfg.m_packedYUVMode,
              decCfg.m_upscaledOutput, ChromaFormat::UNDEFINED, decCfg.m_clipOutputVideoToRec709Range,
              decCfg.m_upscaleFilterForDisplay);
          }
          else
          {
            m_videoIOYuvSEIFGSFile[pic->m_layerId].write(
              pic->getRecoBuf().get(COMP_Y).width, pic->getRecoBuf().get(COMP_Y).height, pic->getDisplayBufFG(),
              decCfg.m_outputColourSpaceConvert, decCfg.m_packedYUVMode,
              conf.m_winLeftOffset * SPS::getWinUnitX(chromaFormatIdc),
              conf.m_winRightOffset * SPS::getWinUnitX(chromaFormatIdc),
              conf.m_winTopOffset * SPS::getWinUnitY(chromaFormatIdc),
              conf.m_winBottomOffset * SPS::getWinUnitY(chromaFormatIdc), ChromaFormat::UNDEFINED,
              decCfg.m_clipOutputVideoToRec709Range);
          }
        }

#if JVET_Z0120_SII_SEI_PROCESSING
        if (!decCfg.m_shutterIntervalPostFileName.empty() && m_ShutterFilterEnable)
        {
          const int blendingRatio = getBlendingRatio();
          pic->xOutputPostFilteredPic(pic, picList, blendingRatio);

          const Window &conf            = pic->m_conformanceWindow;
          const SPS    *sps             = pic->m_cs->sps;
          ChromaFormat  chromaFormatIdc = sps->m_chromaFormatIdc;

          m_cTVideoIOYuvSIIPostFile.write(pic->getPostRecBuf().get(COMP_Y).width,
                                          pic->getPostRecBuf().get(COMP_Y).height, pic->getPostRecBuf(),
                                          decCfg.m_outputColourSpaceConvert, decCfg.m_packedYUVMode,
                                          conf.m_winLeftOffset * SPS::getWinUnitX(chromaFormatIdc),
                                          conf.m_winRightOffset * SPS::getWinUnitX(chromaFormatIdc),
                                          conf.m_winTopOffset * SPS::getWinUnitY(chromaFormatIdc),
                                          conf.m_winBottomOffset * SPS::getWinUnitY(chromaFormatIdc),
                                          ChromaFormat::UNDEFINED, decCfg.m_clipOutputVideoToRec709Range);
        }
#endif

        // Perform CTI on decoded frame and write to output CTI file
        if (!decCfg.m_SEICTIFileName.empty())
        {
          const Window &conf            = pic->m_conformanceWindow;
          const SPS    *sps             = pic->m_cs->sps;
          ChromaFormat  chromaFormatIdc = sps->m_chromaFormatIdc;
          if (decCfg.m_upscaledOutput)
          {
            m_cVideoIOYuvSEICTIFile[pic->m_layerId].writeUpscaledPicture(
              *sps, *pic->m_cs->pps, pic->getDisplayBuf(), decCfg.m_outputColourSpaceConvert, decCfg.m_packedYUVMode,
              decCfg.m_upscaledOutput, ChromaFormat::UNDEFINED, decCfg.m_clipOutputVideoToRec709Range,
              decCfg.m_upscaleFilterForDisplay);
          }
          else
          {
            m_cVideoIOYuvSEICTIFile[pic->m_layerId].write(
              pic->getRecoBuf().get(COMP_Y).width, pic->getRecoBuf().get(COMP_Y).height, pic->getDisplayBuf(),
              decCfg.m_outputColourSpaceConvert, decCfg.m_packedYUVMode,
              conf.m_winLeftOffset * SPS::getWinUnitX(chromaFormatIdc),
              conf.m_winRightOffset * SPS::getWinUnitX(chromaFormatIdc),
              conf.m_winTopOffset * SPS::getWinUnitY(chromaFormatIdc),
              conf.m_winBottomOffset * SPS::getWinUnitY(chromaFormatIdc), ChromaFormat::UNDEFINED,
              decCfg.m_clipOutputVideoToRec709Range);
          }
        }

#if ENABLE_NNLF
        xNnlfDumpPicData(*pic);
#endif

        writeLineToOutputLog(pic);
        // update POC of display order
        m_iPOCLastDisplay = pic->m_poc;

        // erase non-referenced picture in the reference picture list after display
        if (!pic->m_referenced && pic->m_reconstructed)
        {
          pic->m_reconstructed = false;
        }
        pic->m_neededForOutput = false;
      }
#if JVET_Z0120_SII_SEI_PROCESSING
      if (pic != nullptr && (m_cDecLib.m_decCfg.m_shutterIntervalPostFileName.empty() || !m_ShutterFilterEnable))
#else
      if (pic != nullptr)
#endif
      {
        pic->destroy();
        delete pic;
        pic      = nullptr;
        *iterPic = nullptr;
      }
      iterPic++;
    }
  }

  if (layerId != NOT_VALID)
  {
    picList->remove_if([](Picture *p) { return p == nullptr; });
  }
  else
  {
    picList->clear();
  }
  m_iPOCLastDisplay = -MAX_INT;
}

/** \param picList list of pictures to be written to file
 */
void DecApp::xOutputAnnotatedRegions(PicList *picList)
{
  if (!picList || picList->empty())
  {
    return;
  }
  PicList::iterator iterPic = picList->begin();

  while (iterPic != picList->end())
  {
    Picture *pic = *(iterPic);
    if (pic->m_neededForOutput)
    {
      // Check if any annotated region SEI has arrived
      SEIMessages annotatedRegionSEIs = getSeisByType(pic->m_SEIs, SEI::PayloadType::ANNOTATED_REGIONS);
      for (auto it = annotatedRegionSEIs.begin(); it != annotatedRegionSEIs.end(); it++)
      {
        const SEIAnnotatedRegions &seiAnnotatedRegions = *(SEIAnnotatedRegions *)(*it);

        if (seiAnnotatedRegions.m_hdr.m_cancelFlag)
        {
          m_arObjects.clear();
          m_arLabels.clear();
        }
        else
        {
          if (m_arHeader.m_receivedSettingsOnce)
          {
            // validate those settings that must stay constant are constant.
            assert(m_arHeader.m_occludedObjectFlag == seiAnnotatedRegions.m_hdr.m_occludedObjectFlag);
            assert(m_arHeader.m_partialObjectFlagPresentFlag ==
                   seiAnnotatedRegions.m_hdr.m_partialObjectFlagPresentFlag);
            assert(m_arHeader.m_objectConfidenceInfoPresentFlag ==
                   seiAnnotatedRegions.m_hdr.m_objectConfidenceInfoPresentFlag);
            assert((!m_arHeader.m_objectConfidenceInfoPresentFlag) ||
                   m_arHeader.m_objectConfidenceLength == seiAnnotatedRegions.m_hdr.m_objectConfidenceLength);
          }
          else
          {
            m_arHeader.m_receivedSettingsOnce = true;
            m_arHeader                        = seiAnnotatedRegions.m_hdr;   // copy the settings.
          }
          // Process label updates
          if (seiAnnotatedRegions.m_hdr.m_objectLabelPresentFlag)
          {
            for (auto srcIt = seiAnnotatedRegions.m_annotatedLabels.begin();
                 srcIt != seiAnnotatedRegions.m_annotatedLabels.end(); srcIt++)
            {
              const uint32_t labIdx = srcIt->first;
              if (srcIt->second.labelValid)
              {
                m_arLabels[labIdx] = srcIt->second.label;
              }
              else
              {
                m_arLabels.erase(labIdx);
              }
            }
          }

          // Process object updates
          for (auto srcIt = seiAnnotatedRegions.m_annotatedRegions.begin();
               srcIt != seiAnnotatedRegions.m_annotatedRegions.end(); srcIt++)
          {
            uint32_t                                          objIdx = srcIt->first;
            const SEIAnnotatedRegions::AnnotatedRegionObject &src    = srcIt->second;

            if (src.objectCancelFlag)
            {
              m_arObjects.erase(objIdx);
            }
            else
            {
              auto destIt = m_arObjects.find(objIdx);

              if (destIt == m_arObjects.end())
              {
                // New object arrived, needs to be appended to the map of tracked objects
                m_arObjects[objIdx] = src;
              }
              else   // Existing object, modifications to be done
              {
                SEIAnnotatedRegions::AnnotatedRegionObject &dst = destIt->second;

                if (seiAnnotatedRegions.m_hdr.m_objectLabelPresentFlag && src.objectLabelValid)
                {
                  dst.objectLabelValid = true;
                  dst.objLabelIdx      = src.objLabelIdx;
                }
                if (src.boundingBoxValid)
                {
                  dst.boundingBoxTop    = src.boundingBoxTop;
                  dst.boundingBoxLeft   = src.boundingBoxLeft;
                  dst.boundingBoxWidth  = src.boundingBoxWidth;
                  dst.boundingBoxHeight = src.boundingBoxHeight;
                  if (seiAnnotatedRegions.m_hdr.m_partialObjectFlagPresentFlag)
                  {
                    dst.partialObjectFlag = src.partialObjectFlag;
                  }
                  if (seiAnnotatedRegions.m_hdr.m_objectConfidenceInfoPresentFlag)
                  {
                    dst.objectConfidence = src.objectConfidence;
                  }
                }
              }
            }
          }
        }
      }

      if (!m_arObjects.empty())
      {
        FILE *fpPersist = fopen(m_cDecLib.m_decCfg.m_annotatedRegionsSEIFileName.c_str(), "ab");
        if (fpPersist == nullptr)
        {
          std::cout << "Not able to open file for writing persist SEI messages" << std::endl;
        }
        else
        {
          fprintf(fpPersist, "\n");
          fprintf(fpPersist, "Number of objects = %d\n", (int)m_arObjects.size());
          for (auto it = m_arObjects.begin(); it != m_arObjects.end(); ++it)
          {
            fprintf(fpPersist, "Object Idx = %d\n", it->first);
            fprintf(fpPersist, "Object Top = %d\n", it->second.boundingBoxTop);
            fprintf(fpPersist, "Object Left = %d\n", it->second.boundingBoxLeft);
            fprintf(fpPersist, "Object Width = %d\n", it->second.boundingBoxWidth);
            fprintf(fpPersist, "Object Height = %d\n", it->second.boundingBoxHeight);
            if (it->second.objectLabelValid)
            {
              auto labelIt = m_arLabels.find(it->second.objLabelIdx);
              fprintf(fpPersist, "Object Label = %s\n",
                      labelIt != m_arLabels.end() ? (labelIt->second.c_str()) : "<UNKNOWN>");
            }
            if (m_arHeader.m_partialObjectFlagPresentFlag)
            {
              fprintf(fpPersist, "Object Partial = %d\n", it->second.partialObjectFlag ? 1 : 0);
            }
            if (m_arHeader.m_objectConfidenceInfoPresentFlag)
            {
              fprintf(fpPersist, "Object Conf = %d\n", it->second.objectConfidence);
            }
          }
          fclose(fpPersist);
        }
      }
    }
    iterPic++;
  }
}

/** \param nalu Input nalu to check whether its LayerId is within targetDecLayerIdSet
 */
bool DecApp::xIsNaluWithinTargetDecLayerIdSet(const InputNALUnit *nalu) const
{
  if (!m_targetDecLayerIdSet.size())   // By default, the set is empty, meaning all LayerIds are allowed
  {
    return true;
  }

  return std::find(m_targetDecLayerIdSet.begin(), m_targetDecLayerIdSet.end(), nalu->m_nuhLayerId) !=
    m_targetDecLayerIdSet.end();
}

/** \param nalu Input nalu to check whether its LayerId is within targetOutputLayerIdSet
 */
bool DecApp::xIsNaluWithinTargetOutputLayerIdSet(const InputNALUnit *nalu) const
{
  if (!m_targetOutputLayerIdSet.size())   // By default, the set is empty, meaning all LayerIds are allowed
  {
    return true;
  }

  return std::find(m_targetOutputLayerIdSet.begin(), m_targetOutputLayerIdSet.end(), nalu->m_nuhLayerId) !=
    m_targetOutputLayerIdSet.end();
}

//! \}
