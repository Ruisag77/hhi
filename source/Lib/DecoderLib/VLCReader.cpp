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

/** \file     VLCWReader.cpp
 *  \brief    Reader for high level syntax
 */

//! \ingroup DecoderLib
//! \{

#include "VLCReader.h"

#include "CommonLib/CommonDef.h"
#include "AlfParameters.h"
#include "CommonLib/AlfParametersEcm.h"
#include "CommonLib/AlfParametersVtm.h"
#include "CommonLib/AdaptiveLoopFilter.h"
#if ENABLE_NNLF
#include "NNFilterUnified.h"
#endif
#include "CommonLib/dtrace_next.h"
#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif
#include "CommonLib/ProfileTierLevel.h"

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

#if ENABLE_TRACING || RExt__DECODER_DEBUG_BIT_STATISTICS
void VLCReader::xReadCode(const uint32_t length, uint32_t &value, const char *symbolName)
#else
void VLCReader::xReadCode(const uint32_t length, uint32_t &value, const char *)
#endif
{
  CHECK(length == 0, "Reading a code of length '0'");
  m_pcBitstream->read(length, value);

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(symbolName, length, value);
#endif

#if ENABLE_TRACING
  if (length < 10)
  {
    DTRACE(g_trace_ctx, D_HEADER, "%-50s u(%d)  : %u\n", symbolName, length, value);
  }
  else
  {
    DTRACE(g_trace_ctx, D_HEADER, "%-50s u(%d) : %u\n", symbolName, length, value);
  }
#endif
}

#if ENABLE_TRACING || RExt__DECODER_DEBUG_BIT_STATISTICS
void VLCReader::xReadUvlc(uint32_t &value, const char *symbolName)
#else
void VLCReader::xReadUvlc(uint32_t &value, const char *)
#endif
{
  uint32_t suffix    = 0;
  uint32_t prefixBit = 0;
  m_pcBitstream->read(1, prefixBit);

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  uint32_t totalLen = 1;
#endif

  if (0 == prefixBit)
  {
    uint32_t length = 0;

    while (prefixBit == 0)
    {
      m_pcBitstream->read(1, prefixBit);
      length++;
    }

    m_pcBitstream->read(length, suffix);
    suffix += (1 << length) - 1;

#if RExt__DECODER_DEBUG_BIT_STATISTICS
    totalLen += length + length;
#endif
  }

  value = suffix;

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(symbolName, int(totalLen), value);
#endif

#if ENABLE_TRACING
  DTRACE(g_trace_ctx, D_HEADER, "%-50s ue(v) : %u\n", symbolName, value);
#endif
}

#if ENABLE_TRACING || RExt__DECODER_DEBUG_BIT_STATISTICS
void VLCReader::xReadSvlc(int &value, const char *symbolName)
#else
void VLCReader::xReadSvlc(int &value, const char *)
#endif
{
  uint32_t prefixBit = 0;
  uint32_t suffix    = 0;

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  uint32_t totalLen = 1;
#endif

  m_pcBitstream->read(1, prefixBit);

  if (0 == prefixBit)
  {
    uint32_t length = 0;

    while (prefixBit == 0)
    {
      m_pcBitstream->read(1, prefixBit);
      length++;
    }

    m_pcBitstream->read(length, suffix);

    suffix += (1 << length);
    value = (suffix & 1) ? -(int)(suffix >> 1) : (int)(suffix >> 1);

#if RExt__DECODER_DEBUG_BIT_STATISTICS
    totalLen += length + length;
#endif
  }
  else
  {
    value = 0;
  }
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(symbolName, int(totalLen), suffix);
#endif

#if ENABLE_TRACING
  DTRACE(g_trace_ctx, D_HEADER, "%-50s se(v) : %d\n", symbolName, value);
#endif
}

#if ENABLE_TRACING || RExt__DECODER_DEBUG_BIT_STATISTICS
void VLCReader::xReadFlag(uint32_t &value, const char *symbolName)
#else
void VLCReader::xReadFlag(uint32_t &value, const char *)
#endif
{
  m_pcBitstream->read(1, value);

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(symbolName, 1, int(/*ruiCode*/ 0));
#endif

#if ENABLE_TRACING
  DTRACE(g_trace_ctx, D_HEADER, "%-50s u(1)  : %d\n", symbolName, value);
#endif
}

#if ENABLE_TRACING || RExt__DECODER_DEBUG_BIT_STATISTICS
void VLCReader::xReadString(std::string &value, const char *symbolName)
#else
void VLCReader::xReadString(std::string &value, const char *)
#endif
{
  uint32_t code;
  value = "";
  do
  {
    m_pcBitstream->read(8, code);
    if (code != 0)
    {
      value += (char)code;
    }
  } while (code != 0);

#if ENABLE_TRACING
  DTRACE(g_trace_ctx, D_HEADER, "%-50s u(1)  : %s\n", symbolName, value.c_str());
#endif
}

#if RExt__DECODER_DEBUG_BIT_STATISTICS || ENABLE_TRACING
void VLCReader::xReadSCode(const uint32_t length, int &value, const char *symbolName)
#else
void VLCReader::xReadSCode(const uint32_t length, int &value, const char *)
#endif
{
  uint32_t val;
  CHECK(length < 1 || length > 32, "Syntax element length must be in range 1..32");
  m_pcBitstream->read(length, val);
  value = length >= 32 ? int(val) : ((-int(val & (uint32_t(1) << (length - 1)))) | int(val));

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(symbolName, length, value);
#endif
#if ENABLE_TRACING
  if (length < 10)
  {
    DTRACE(g_trace_ctx, D_HEADER, "%-50s i(%d)  : %d\n", symbolName, length, value);
  }
  else
  {
    DTRACE(g_trace_ctx, D_HEADER, "%-50s i(%d) : %d\n", symbolName, length, value);
  }
#endif
}

void VLCReader::xReadRbspTrailingBits()
{
  uint32_t bit;
  xReadFlag(bit, "rbsp_stop_one_bit");
  CHECK(bit != 1, "Trailing bit not '1'");
  int cnt = 0;
  while (m_pcBitstream->getNumBitsUntilByteAligned())
  {
    xReadFlag(bit, "rbsp_alignment_zero_bit");
    CHECK(bit != 0, "Alignment bit is not '0'");
    cnt++;
  }
  CHECK(cnt >= 8, "Read more than '8' trailing bits");
}

void AUDReader::parseAccessUnitDelimiter(InputBitstream *bs, uint32_t &audIrapOrGdrAuFlag, uint32_t &picType)
{
  setBitstream(bs);

#if ENABLE_TRACING
  xTraceAccessUnitDelimiter();
#endif

  xReadFlag(audIrapOrGdrAuFlag, "aud_irap_or_gdr_au_flag");
  xReadCode(3, picType, "pic_type");
  xReadRbspTrailingBits();
}

void FDReader::parseFillerData(InputBitstream *bs, uint32_t &fdSize)
{
  setBitstream(bs);
#if ENABLE_TRACING
  xTraceFillerData();
#endif
  uint32_t ffByte;
  fdSize = 0;
  while (m_pcBitstream->getNumBitsLeft() > 8)
  {
    xReadCode(8, ffByte, "ff_byte");
    CHECK(ffByte != 0xff, "Invalid filler data : not '0xff'");
    fdSize++;
  }
  xReadRbspTrailingBits();
}

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

HLSyntaxReader::HLSyntaxReader() {}

HLSyntaxReader::~HLSyntaxReader() {}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void HLSyntaxReader::copyRefPicList(SPS *sps, ReferencePictureList *source_rpl, ReferencePictureList *dest_rp)
{
  dest_rp->m_numberOfShorttermPictures = source_rpl->m_numberOfShorttermPictures;

  dest_rp->m_numberOfInterLayerPictures = (sps->m_interLayerPresentFlag ? source_rpl->m_numberOfInterLayerPictures : 0);

  if (sps->m_longTermRefsPresent)
  {
    dest_rp->m_ltrpInSliceHeaderFlag    = source_rpl->m_ltrpInSliceHeaderFlag;
    dest_rp->m_numberOfLongtermPictures = source_rpl->m_numberOfLongtermPictures;
  }
  else
  {
    dest_rp->m_numberOfLongtermPictures = 0;
  }

  uint32_t numRefPic = dest_rp->getNumRefEntries();

  for (int ii = 0; ii < numRefPic; ii++)
  {
    dest_rp->setRefPicIdentifier(ii, source_rpl->m_refPicIdentifier[ii], source_rpl->m_isLongtermRefPic[ii],
                                 source_rpl->m_isInterLayerRefPic[ii], source_rpl->m_interLayerRefPicIdx[ii]);
  }
}

void HLSyntaxReader::parseRefPicListInterRPL(SPS *sps, const int numberOfRPL)
{
  // Precalculated numbers for hierarchical structure of pictures. The TemporalID
  // range in the NAL unit header is 0-6 so 64 is the largest that can be expressed
  static const int POC_values_for_gop_64[64] = { 64, 32, 16, 8,  4,  2,  1,  3,  6,  5,  7,  12, 10, 9,  11, 14,
                                                 13, 15, 24, 20, 18, 17, 19, 22, 21, 23, 28, 26, 25, 27, 30, 29,
                                                 31, 48, 40, 36, 34, 33, 35, 38, 37, 39, 44, 42, 41, 43, 46, 45,
                                                 47, 56, 52, 50, 49, 51, 54, 53, 55, 60, 58, 57, 59, 62, 61, 63 };

  // Decode number of hierarchical levels, for example GOP 32 would be 5 levels
  uint32_t hierarchicalLevels;
  xReadUvlc(hierarchicalLevels, "inter_rpl_hierarchical_levels");
  CHECK(hierarchicalLevels > 6, "inter_rpl_hierarchical_levels exceeds max value");

  const int *POC = POC_values_for_gop_64 + 6 - hierarchicalLevels;

  sps->m_QPoffsetRPL.resize(0);
  int QP_OFFSET[7] = { 0, 0, 0, 0, 0, 0, 0 };
  for (int i = 0; i < (hierarchicalLevels > 0 ? hierarchicalLevels + 1 : numberOfRPL); i++)
  {
    if (hierarchicalLevels > 0)
    {
      xReadSvlc(QP_OFFSET[i], "rpl_qp_delta");
    }
    else
    {
      int iCode;
      xReadSvlc(iCode, "rpl_qp_delta");
      sps->m_QPoffsetRPL.push_back(iCode);
    }
  }

  uint32_t         prevNumEntriesLX[2] = { 0, 0 };
  std::vector<int> rps;
  for (int rplIdx = 0; rplIdx < numberOfRPL; rplIdx++)
  {
    uint32_t              numEntriesLX[2];
    ReferencePictureList *rplLX[2];
    rplLX[0] = sps->m_rplList[RPL0].getReferencePictureList(rplIdx);
    rplLX[1] = sps->m_rplList[RPL1].getReferencePictureList(rplIdx);

    if (hierarchicalLevels > 0 && rplIdx < (1 << hierarchicalLevels))
    {
      int tID;
      int temp = POC[rplIdx];
      for (tID = hierarchicalLevels; tID > 0 && !(temp & 1); tID--)
      {
        temp >>= 1;
      }
      sps->m_QPoffsetRPL.push_back(QP_OFFSET[tID]);
    }
    if (rplIdx == 0)
    {
      // Decode L0 and L1 without Inter-RPL
      for (int lx = 0; lx < (sps->m_rpl1CopyFromRpl0Flag ? 1 : 2); lx++)
      {
        int deltaValue = 0;
        xReadUvlc(numEntriesLX[lx], "num_ref_entries_lx");
        for (int ii = 0; ii < numEntriesLX[lx]; ii++)
        {
          uint32_t uiCode;
          xReadUvlc(uiCode, "inter_rpl_entry_abs_delta_poc[Lx][rplIdx][i]");
          int Value = (ii == 0) ? uiCode + 1 : uiCode;
          if (Value > 0)
          {
            xReadFlag(uiCode, "inter_rpl_entry_sign_flag[Lx][rplIdx][i]");
            deltaValue = (uiCode) ? deltaValue - Value : deltaValue + Value;
          }
          rplLX[lx]->setRefPicIdentifier(ii, deltaValue, 0, false, 0);
        }
      }
    }
    else
    {
      // Calculate delta POC between the current and previous picture
      int DeltapocRPL = (hierarchicalLevels == 0) ? 1 : POC[rplIdx] - POC[rplIdx - 1];
      if (hierarchicalLevels > 0 && rplIdx >= (1 << hierarchicalLevels))
      {
        // Decode DeltaPOC
        uint32_t uiCode;
        xReadUvlc(uiCode, "inter_rpl_abs_delta_poc_minus1");
        DeltapocRPL = uiCode + 1;
        xReadFlag(uiCode, "inter_rpl_sign_flag");
        if (uiCode)
        {
          DeltapocRPL = -DeltapocRPL;
        }
      }

      // Update the RPS by recalculating POC offsets using the delta POC
      for (int i = 0; i < rps.size(); i++)
      {
        rps[i] -= DeltapocRPL;
      }

      // Add POC of the current picture and sort
      rps.push_back(-DeltapocRPL);
      sort(rps.begin(), rps.end());

      const int SizeRPS           = (int)rps.size();
      const int bitlength         = ceilLog2(SizeRPS);
      // Decode L0 and L1 entries
      uint32_t  useDefaultList[2] = { 0, 0 };
      for (int lx = 0; lx < (sps->m_rpl1CopyFromRpl0Flag ? 1 : 2); lx++)
      {
        int iCode;
        xReadSvlc(iCode, "delta_num_ref_entries_lx");
        numEntriesLX[lx] = numEntriesLX[1] = prevNumEntriesLX[lx] + iCode;
        xReadFlag(useDefaultList[lx], "inter_rpl_use_default_list_flag_lx");
        if (!useDefaultList[lx])
        {
          for (int ii = 0; ii < numEntriesLX[lx]; ii++)
          {
            uint32_t uiCode;
            xReadCode(bitlength, uiCode, "inter_rpl_idx_lx");
            rplLX[lx]->setRefPicIdentifier(ii, rps[uiCode], 0, false, 0);
          }
        }
      }

      // If signalled, set L0 and/or L1 to the default list
      // Todo: copy this code to encoder since default lists otherwise differ if lists are long enough
      int indexToFirstPositiveEntry;
      for (indexToFirstPositiveEntry = 0; indexToFirstPositiveEntry < SizeRPS && rps[indexToFirstPositiveEntry] < 0;
           indexToFirstPositiveEntry++)
        ;
      if (useDefaultList[0])
      {
        for (int ii = 0; ii < numEntriesLX[0]; ii++)
        {
          int index = indexToFirstPositiveEntry - ii - 1;
          if (index < 0)
          {
            index = ii % SizeRPS;
          }
          rplLX[0]->setRefPicIdentifier(ii, rps[index], 0, false, 0);
        }
      }
      if (useDefaultList[1])
      {
        for (int ii = 0; ii < numEntriesLX[1]; ii++)
        {
          int index = indexToFirstPositiveEntry + ii;
          if (index >= SizeRPS)
          {
            index = SizeRPS - (ii % SizeRPS) - 1;
          }
          rplLX[1]->setRefPicIdentifier(ii, rps[index], 0, false, 0);
        }
      }
    }

    if (sps->m_rpl1CopyFromRpl0Flag)
    {
      numEntriesLX[1] = numEntriesLX[0];
      for (int ii = 0; ii < numEntriesLX[1]; ii++)
      {
        rplLX[1]->setRefPicIdentifier(ii, rplLX[0]->m_refPicIdentifier[ii], 0, false, 0);
      }
    }

    for (int lx = 0; lx < 2; lx++)
    {
      prevNumEntriesLX[lx]                    = numEntriesLX[lx];
      rplLX[lx]->m_numberOfShorttermPictures  = numEntriesLX[lx];
      rplLX[lx]->m_numberOfLongtermPictures   = 0;
      rplLX[lx]->m_numberOfInterLayerPictures = 0;
      rplLX[lx]->m_interLayerPresentFlag      = sps->m_interLayerPresentFlag;
    }

    // Construct RPS for the next picture, add L0 and L1 entries without duplicates
    rps.resize(0);
    for (int lx = 0; lx < 2; lx++)
    {
      for (int ii = 0; ii < numEntriesLX[lx]; ii++)
      {
        int poc_rpl = rplLX[lx]->m_refPicIdentifier[ii];
        if (std::find(rps.begin(), rps.end(), poc_rpl) == rps.end())
        {
          rps.push_back(poc_rpl);
        }
      }
    }
  }
}

void HLSyntaxReader::parseRefPicList(SPS *sps, ReferencePictureList *rpl, int rplIdx)
{
  uint32_t code;
  xReadUvlc(code, "num_ref_entries[ listIdx ][ rplsIdx ]");
  uint32_t numRefPic = code;
  uint32_t numStrp   = 0;
  uint32_t numLtrp   = 0;
  uint32_t numIlrp   = 0;

  if (sps->m_longTermRefsPresent && numRefPic > 0 && rplIdx != -1)
  {
    xReadFlag(code, "ltrp_in_slice_header_flag[ listIdx ][ rplsIdx ]");
    rpl->m_ltrpInSliceHeaderFlag = code;
  }
  else if (sps->m_longTermRefsPresent)
  {
    rpl->m_ltrpInSliceHeaderFlag = 1;
  }

  bool isLongTerm;
  int  prevDelta  = MAX_INT;
  int  deltaValue = 0;
  bool firstSTRP  = true;

  rpl->m_interLayerPresentFlag = sps->m_interLayerPresentFlag;

  for (int ii = 0; ii < numRefPic; ii++)
  {
    uint32_t isInterLayerRefPic = 0;

    if (rpl->m_interLayerPresentFlag)
    {
      xReadFlag(isInterLayerRefPic, "inter_layer_ref_pic_flag[ listIdx ][ rplsIdx ][ i ]");

      if (isInterLayerRefPic)
      {
        xReadUvlc(code, "ilrp_idx[ listIdx ][ rplsIdx ][ i ]");
        rpl->setRefPicIdentifier(ii, 0, true, true, code);
        numIlrp++;
      }
    }

    if (!isInterLayerRefPic)
    {
      isLongTerm = false;
      if (sps->m_longTermRefsPresent)
      {
        xReadFlag(code, "st_ref_pic_flag[ listIdx ][ rplsIdx ][ i ]");
        isLongTerm = (code == 1) ? false : true;
      }

      if (!isLongTerm)
      {
        xReadUvlc(code, "abs_delta_poc_st[ listIdx ][ rplsIdx ][ i ]");
        if ((!sps->m_useWP && !sps->m_useBiWP) || (ii == 0))
        {
          code++;
        }
        int readValue = code;
        if (readValue > 0)
        {
          xReadFlag(code, "strp_entry_sign_flag[ listIdx ][ rplsIdx ][ i ]");
          if (code)
          {
            readValue = -readValue;
          }
        }
        if (firstSTRP)
        {
          firstSTRP = false;
          prevDelta = deltaValue = readValue;
        }
        else
        {
          deltaValue = prevDelta + readValue;
          prevDelta  = deltaValue;
        }

        rpl->setRefPicIdentifier(ii, deltaValue, isLongTerm, false, 0);
        numStrp++;
      }
      else
      {
        if (!rpl->m_ltrpInSliceHeaderFlag)
        {
          xReadCode(sps->m_bitsForPoc, code, "poc_lsb_lt[listIdx][rplsIdx][j]");
        }
        rpl->setRefPicIdentifier(ii, code, isLongTerm, false, 0);
        numLtrp++;
      }
    }
  }
  rpl->m_numberOfShorttermPictures  = numStrp;
  rpl->m_numberOfLongtermPictures   = numLtrp;
  rpl->m_numberOfInterLayerPictures = numIlrp;
}

void HLSyntaxReader::parsePPS(PPS *pcPPS)
{
#if ENABLE_TRACING
  xTracePPSHeader();
#endif
  uint32_t uiCode;

  int iCode;
  xReadCode(6, uiCode, "pps_pic_parameter_set_id");
  CHECK(uiCode > 63, "PPS id exceeds boundary (63)");
  pcPPS->m_ppsId = uiCode;

  xReadCode(4, uiCode, "pps_seq_parameter_set_id");
  pcPPS->m_spsId = uiCode;

  xReadFlag(uiCode, "pps_mixed_nalu_types_in_pic_flag");
  pcPPS->m_mixedNaluTypesInPicFlag = (uiCode == 1);

  xReadUvlc(uiCode, "pps_pic_width_in_luma_samples");
  pcPPS->m_picWidthInLumaSamples = uiCode;
  xReadUvlc(uiCode, "pps_pic_height_in_luma_samples");
  pcPPS->m_picHeightInLumaSamples = uiCode;
  xReadFlag(uiCode, "pps_conformance_window_flag");
  pcPPS->m_conformanceWindowFlag = uiCode;
  if (uiCode != 0)
  {
    Window &conf       = pcPPS->m_conformanceWindow;
    conf.m_enabledFlag = true;
    xReadUvlc(uiCode, "pps_conf_win_left_offset");
    conf.m_winLeftOffset = uiCode;
    xReadUvlc(uiCode, "pps_conf_win_right_offset");
    conf.m_winRightOffset = uiCode;
    xReadUvlc(uiCode, "pps_conf_win_top_offset");
    conf.m_winTopOffset = uiCode;
    xReadUvlc(uiCode, "pps_conf_win_bottom_offset");
    conf.m_winBottomOffset = uiCode;
  }
  xReadFlag(uiCode, "pps_scaling_window_explicit_signalling_flag");
  pcPPS->m_explicitScalingWindowFlag = uiCode;
  if (uiCode != 0)
  {
    Window &scalingWindow       = pcPPS->m_scalingWindow;
    scalingWindow.m_enabledFlag = true;
    xReadSvlc(iCode, "pps_scaling_win_left_offset");
    scalingWindow.m_winLeftOffset = iCode;
    xReadSvlc(iCode, "pps_scaling_win_right_offset");
    scalingWindow.m_winRightOffset = iCode;
    xReadSvlc(iCode, "pps_scaling_win_top_offset");
    scalingWindow.m_winTopOffset = iCode;
    xReadSvlc(iCode, "pps_scaling_win_bottom_offset");
    scalingWindow.m_winBottomOffset = iCode;
  }
  else
  {
    Window &scalingWindow = pcPPS->m_scalingWindow;
    Window &conf          = pcPPS->m_conformanceWindow;
    scalingWindow         = conf;
//    scalingWindow.setWindowLeftOffset( conf.m_winLeftOffset );
//    scalingWindow.setWindowRightOffset( conf.m_winRightOffset );
//    scalingWindow.setWindowTopOffset( conf.m_winTopOffset );
//    scalingWindow.setWindowBottomOffset( conf.m_winBottomOffset );
  }

  xReadFlag(uiCode, "pps_output_flag_present_flag");
  pcPPS->m_outputFlagPresentFlag = (uiCode == 1);

  xReadFlag(uiCode, "pps_no_pic_partition_flag");
  pcPPS->m_noPicPartitionFlag = uiCode == 1;
  xReadFlag(uiCode, "pps_subpic_id_mapping_present_flag");
  pcPPS->m_subPicIdMappingInPpsFlag = (uiCode != 0);
  if (pcPPS->m_subPicIdMappingInPpsFlag)
  {
    if (!pcPPS->m_noPicPartitionFlag)
    {
      xReadUvlc(uiCode, "pps_num_subpics_minus1");
      pcPPS->setNumSubPics(uiCode + 1);
    }
    else
    {
      pcPPS->setNumSubPics(1);
    }
    CHECK(uiCode > MAX_NUM_SUB_PICS - 1, "Number of sub-pictures exceeds limit");

    xReadUvlc(uiCode, "pps_subpic_id_len_minus1");
    pcPPS->m_subPicIdLen = (uiCode + 1);
    CHECK(uiCode > 15, "Invalid pps_subpic_id_len_minus1 signalled");

    CHECK((1 << pcPPS->m_subPicIdLen) < pcPPS->m_numSubPics, "pps_subpic_id_len exceeds valid range");
    for (int picIdx = 0; picIdx < pcPPS->m_numSubPics; picIdx++)
    {
      xReadCode(pcPPS->m_subPicIdLen, uiCode, "pps_subpic_id[i]");
      pcPPS->m_subPicId[picIdx] = uiCode;
    }
  }
  if (!pcPPS->m_noPicPartitionFlag)
  {
    int colIdx, rowIdx;
    pcPPS->resetTileSliceInfo();

    // CTU size - required to match size in SPS
    xReadCode(2, uiCode, "pps_log2_ctu_size_minus6");
    pcPPS->setLog2CtuSize(uiCode + 6);
    CHECK(uiCode > 2, "pps_log2_ctu_size_minus6 must be less than or equal to 2");

    // number of explicit tile columns/rows
    xReadUvlc(uiCode, "pps_num_exp_tile_columns_minus1");
    pcPPS->m_numExpTileCols = (uiCode + 1);
    xReadUvlc(uiCode, "pps_num_exp_tile_rows_minus1");
    pcPPS->m_numExpTileRows = (uiCode + 1);
    CHECK(pcPPS->m_numExpTileCols > MAX_TILE_COLS, "Number of explicit tile columns exceeds valid range");

    // tile sizes
    for (colIdx = 0; colIdx < pcPPS->m_numExpTileCols; colIdx++)
    {
      xReadUvlc(uiCode, "pps_tile_column_width_minus1[i]");
      pcPPS->addTileColumnWidth(uiCode + 1);
      CHECK(uiCode > (pcPPS->m_picWidthInCtu - 1),
            "The value of pps_tile_column_width_minus1[i] shall be in the range of 0 to PicWidthInCtbY-1, inclusive");
    }
    for (rowIdx = 0; rowIdx < pcPPS->m_numExpTileRows; rowIdx++)
    {
      xReadUvlc(uiCode, "pps_tile_row_height_minus1[i]");
      pcPPS->addTileRowHeight(uiCode + 1);
      CHECK(uiCode > (pcPPS->m_picHeightInCtu - 1),
            "The value of pps_tile_row_height_minus shall be in the range of 0 to PicHeightInCtbY-1, inclusive");
    }
    pcPPS->initTiles();
    // rectangular slice signalling
    if (pcPPS->getNumTiles() > 1)
    {
      xReadCode(1, uiCode, "pps_loop_filter_across_tiles_enabled_flag");
      pcPPS->m_loopFilterAcrossTilesEnabledFlag = uiCode == 1;
      xReadCode(1, uiCode, "pps_rect_slice_flag");
    }
    else
    {
      pcPPS->m_loopFilterAcrossTilesEnabledFlag = false;
      uiCode                                    = 1;
    }
    pcPPS->m_rectSliceFlag = uiCode == 1;
    if (pcPPS->m_rectSliceFlag)
    {
      xReadFlag(uiCode, "pps_single_slice_per_subpic_flag");
      pcPPS->m_singleSlicePerSubPicFlag = uiCode == 1;
    }
    else
    {
      pcPPS->m_singleSlicePerSubPicFlag = 0;
    }
    if (pcPPS->m_rectSliceFlag && !(pcPPS->m_singleSlicePerSubPicFlag))
    {
      int32_t tileIdx = 0;

      xReadUvlc(uiCode, "pps_num_slices_in_pic_minus1");
      pcPPS->m_numSlicesInPic = uiCode + 1;
      CHECK(pcPPS->m_numSlicesInPic > MAX_SLICES, "Number of slices in picture exceeds valid range");
      if ((pcPPS->m_numSlicesInPic - 1) > 1)
      {
        xReadCode(1, uiCode, "pps_tile_idx_delta_present_flag");
        pcPPS->m_tileIdxDeltaPresentFlag = uiCode == 1;
      }
      else
      {
        pcPPS->m_tileIdxDeltaPresentFlag = 0;
      }
      pcPPS->initRectSlices();

      // read rectangular slice parameters
      for (int i = 0; i < pcPPS->m_numSlicesInPic - 1; i++)
      {
        pcPPS->m_rectSlices[i].m_tileIdx = tileIdx;

        // complete tiles within a single slice
        if ((tileIdx % pcPPS->m_numTileCols) != pcPPS->m_numTileCols - 1)
        {
          xReadUvlc(uiCode, "pps_slice_width_in_tiles_minus1[i]");
          pcPPS->m_rectSlices[i].m_sliceWidthInTiles = uiCode + 1;
        }
        else
        {
          pcPPS->m_rectSlices[i].m_sliceWidthInTiles = 1;
        }

        if (tileIdx / pcPPS->m_numTileCols != pcPPS->m_numTileRows - 1 &&
            (pcPPS->m_tileIdxDeltaPresentFlag || tileIdx % pcPPS->m_numTileCols == 0))
        {
          xReadUvlc(uiCode, "pps_slice_height_in_tiles_minus1[i]");
          pcPPS->m_rectSlices[i].m_sliceHeightInTiles = uiCode + 1;
        }
        else
        {
          if ((tileIdx / pcPPS->m_numTileCols) == pcPPS->m_numTileRows - 1)
          {
            pcPPS->m_rectSlices[i].m_sliceHeightInTiles = 1;
          }
          else
          {
            pcPPS->m_rectSlices[i].m_sliceHeightInTiles = pcPPS->m_rectSlices[i - 1].m_sliceHeightInTiles;
          }
        }

        // multiple slices within a single tile special case
        if (pcPPS->m_rectSlices[i].m_sliceWidthInTiles == 1 && pcPPS->m_rectSlices[i].m_sliceHeightInTiles == 1)
        {
          if (pcPPS->getTileRowHeight(tileIdx / pcPPS->m_numTileCols) > 1)
          {
            xReadUvlc(uiCode, "pps_num_exp_slices_in_tile[i]");
            if (uiCode == 0)
            {
              pcPPS->m_rectSlices[i].m_numSlicesInTile  = 1;
              pcPPS->m_rectSlices[i].m_sliceHeightInCtu = pcPPS->getTileRowHeight(tileIdx / pcPPS->m_numTileCols);
            }
            else
            {
              uint32_t numExpSliceInTile = uiCode;
              uint32_t remTileRowHeight  = pcPPS->getTileRowHeight(tileIdx / pcPPS->m_numTileCols);
              int      j                 = 0;

              for (; j < numExpSliceInTile; j++)
              {
                xReadUvlc(uiCode, "pps_exp_slice_height_in_ctus_minus1[i]");
                pcPPS->m_rectSlices[i + j].m_sliceHeightInCtu = uiCode + 1;
                remTileRowHeight -= (uiCode + 1);
              }
              uint32_t uniformSliceHeight = uiCode + 1;

              while (remTileRowHeight >= uniformSliceHeight)
              {
                pcPPS->m_rectSlices[i + j].m_sliceHeightInCtu = uniformSliceHeight;
                remTileRowHeight -= uniformSliceHeight;
                j++;
              }
              if (remTileRowHeight > 0)
              {
                pcPPS->m_rectSlices[i + j].m_sliceHeightInCtu = remTileRowHeight;
                j++;
              }
              for (int k = 0; k < j; k++)
              {
                pcPPS->m_rectSlices[i + k].m_numSlicesInTile    = j;
                pcPPS->m_rectSlices[i + k].m_sliceWidthInTiles  = 1;
                pcPPS->m_rectSlices[i + k].m_sliceHeightInTiles = 1;
                pcPPS->m_rectSlices[i + k].m_tileIdx            = tileIdx;
              }
              i += (j - 1);
            }
          }
          else
          {
            pcPPS->m_rectSlices[i].m_numSlicesInTile  = 1;
            pcPPS->m_rectSlices[i].m_sliceHeightInCtu = pcPPS->getTileRowHeight(tileIdx / pcPPS->m_numTileCols);
          }
        }

        // tile index offset to start of next slice
        if (i < pcPPS->m_numSlicesInPic - 1)
        {
          if (pcPPS->m_tileIdxDeltaPresentFlag)
          {
            int32_t tileIdxDelta;
            xReadSvlc(tileIdxDelta, "pps_tile_idx_delta[i]");
            tileIdx += tileIdxDelta;
            CHECK(tileIdx < 0 || tileIdx >= pcPPS->getNumTiles(), "Invalid pps_tile_idx_delta.");
          }
          else
          {
            tileIdx += pcPPS->m_rectSlices[i].m_sliceWidthInTiles;
            if (tileIdx % pcPPS->m_numTileCols == 0)
            {
              tileIdx += (pcPPS->m_rectSlices[i].m_sliceHeightInTiles - 1) * pcPPS->m_numTileCols;
            }
          }
        }
      }
      pcPPS->m_rectSlices[pcPPS->m_numSlicesInPic - 1].m_tileIdx = tileIdx;
    }

    if (pcPPS->m_rectSliceFlag == 0 || pcPPS->m_singleSlicePerSubPicFlag || pcPPS->m_numSlicesInPic > 1)
    {
      xReadCode(1, uiCode, "pps_loop_filter_across_slices_enabled_flag");
      pcPPS->m_loopFilterAcrossSlicesEnabledFlag = uiCode == 1;
    }
    else
    {
      pcPPS->m_loopFilterAcrossSlicesEnabledFlag = false;
    }
  }
  else
  {
    pcPPS->m_singleSlicePerSubPicFlag = 1;
  }

  xReadFlag(uiCode, "pps_cabac_init_present_flag");
  pcPPS->m_cabacInitPresentFlag = uiCode ? true : false;

  xReadUvlc(uiCode, "pps_num_ref_idx_default_active_minus1[0]");
  CHECK(uiCode >= MAX_NUM_ACTIVE_REF,
        "The value of pps_num_ref_idx_default_active_minus1[0] shall be in the range of 0 to 14, inclusive");
  pcPPS->m_numRefIdxDefaultActive[RPL0] = uiCode + 1;

  xReadUvlc(uiCode, "pps_num_ref_idx_default_active_minus1[1]");
  CHECK(uiCode >= MAX_NUM_ACTIVE_REF,
        "The value of pps_num_ref_idx_default_active_minus1[1] shall be in the range of 0 to 14, inclusive");
  pcPPS->m_numRefIdxDefaultActive[RPL1] = uiCode + 1;

  xReadFlag(uiCode, "pps_rpl1_idx_present_flag");
  pcPPS->m_rpl1IdxPresentFlag = uiCode;
  xReadFlag(uiCode, "pps_weighted_pred_flag");          // Use of Weighting Prediction (P_SLICE)
  pcPPS->m_useWP = (uiCode == 1);
  xReadFlag(uiCode, "pps_weighted_bipred_flag");         // Use of Bi-Directional Weighting Prediction (B_SLICE)
  pcPPS->m_useBiWP = (uiCode == 1);
  xReadFlag(uiCode, "pps_ref_wraparound_enabled_flag");
  pcPPS->m_wrapAroundEnabledFlag = (uiCode ? true : false);
  if (pcPPS->m_wrapAroundEnabledFlag)
  {
    xReadUvlc(uiCode, "pps_ref_wraparound_offset");
    pcPPS->m_picWidthMinusWrapAroundOffset = uiCode;
  }
  else
  {
    pcPPS->m_picWidthMinusWrapAroundOffset = 0;
  }

  xReadSvlc(iCode, "pps_init_qp_minus26");
  pcPPS->m_picInitQPMinus26 = iCode;
  xReadFlag(uiCode, "pps_cu_qp_delta_enabled_flag");
  pcPPS->m_useDQP = uiCode ? true : false;
  xReadFlag(uiCode, "pps_chroma_tool_offsets_present_flag");
  pcPPS->m_usePPSChromaTool = uiCode ? true : false;
  if (pcPPS->m_usePPSChromaTool)
  {
    xReadSvlc(iCode, "pps_cb_qp_offset");
    pcPPS->setQpOffset(COMP_Cb, iCode);
    CHECK(pcPPS->getQpOffset(COMP_Cb) < -12, "Invalid Cb QP offset");
    CHECK(pcPPS->getQpOffset(COMP_Cb) > 12, "Invalid Cb QP offset");

    xReadSvlc(iCode, "pps_cr_qp_offset");
    pcPPS->setQpOffset(COMP_Cr, iCode);
    CHECK(pcPPS->getQpOffset(COMP_Cr) < -12, "Invalid Cr QP offset");
    CHECK(pcPPS->getQpOffset(COMP_Cr) > 12, "Invalid Cr QP offset");

    xReadFlag(uiCode, "pps_joint_cbcr_qp_offset_present_flag");
    pcPPS->m_chromaJointCbCrQpOffsetPresentFlag = uiCode ? true : false;

    if (pcPPS->m_chromaJointCbCrQpOffsetPresentFlag)
    {
      xReadSvlc(iCode, "pps_joint_cbcr_qp_offset_value");
    }
    else
    {
      iCode = 0;
    }
    pcPPS->setQpOffset(JOINT_CbCr, iCode);

    CHECK(pcPPS->getQpOffset(JOINT_CbCr) < -12, "Invalid CbCr QP offset");
    CHECK(pcPPS->getQpOffset(JOINT_CbCr) > 12, "Invalid CbCr QP offset");

    CHECK(MAX_NUM_COMP > 3, "Invalid maximal number of components");

    xReadFlag(uiCode, "pps_slice_chroma_qp_offsets_present_flag");
    pcPPS->m_sliceChromaQpFlag = (uiCode ? true : false);

    xReadFlag(uiCode, "pps_cu_chroma_qp_offset_list_enabled_flag");
    if (uiCode == 0)
    {
      pcPPS->m_chromaQpOffsetListLen = 0;
    }
    else
    {
      uint32_t tableSizeMinus1 = 0;
      xReadUvlc(tableSizeMinus1, "pps_chroma_qp_offset_list_len_minus1");
      CHECK(tableSizeMinus1 >= MAX_QP_OFFSET_LIST_SIZE, "Table size exceeds maximum");

      for (int cuChromaQpOffsetIdx = 0; cuChromaQpOffsetIdx <= (tableSizeMinus1); cuChromaQpOffsetIdx++)
      {
        int cbOffset;
        int crOffset;
        int jointCbCrOffset;
        xReadSvlc(cbOffset, "pps_cb_qp_offset_list[i]");
        CHECK(cbOffset < -12 || cbOffset > 12, "Invalid chroma QP offset");
        xReadSvlc(crOffset, "pps_cr_qp_offset_list[i]");
        CHECK(crOffset < -12 || crOffset > 12, "Invalid chroma QP offset");
        if (pcPPS->m_chromaJointCbCrQpOffsetPresentFlag)
        {
          xReadSvlc(jointCbCrOffset, "pps_joint_cbcr_qp_offset_list[i]");
        }
        else
        {
          jointCbCrOffset = 0;
        }
        CHECK(jointCbCrOffset < -12 || jointCbCrOffset > 12, "Invalid chroma QP offset");
        // table uses +1 for index (see comment inside the function)
        pcPPS->setChromaQpOffsetListEntry(cuChromaQpOffsetIdx + 1, cbOffset, crOffset, jointCbCrOffset);
      }
      CHECK(pcPPS->m_chromaQpOffsetListLen != tableSizeMinus1 + 1, "Invalid chroma QP offset list length");
    }
  }
  else
  {
    pcPPS->setQpOffset(COMP_Cb, 0);
    pcPPS->setQpOffset(COMP_Cr, 0);
    pcPPS->m_chromaJointCbCrQpOffsetPresentFlag = 0;
    pcPPS->m_sliceChromaQpFlag                  = 0;
    pcPPS->m_chromaQpOffsetListLen              = 0;
  }
  xReadFlag(uiCode, "sgpm_no_blend_flag");
  pcPPS->m_useSgpmNoBlend = uiCode ? true : false;

  xReadFlag(uiCode, "bilateral_filter_flag");
  pcPPS->m_BIF = uiCode ? true : false; // pcPPS->setUseBIF(uiCode != 0);
  if (pcPPS->m_BIF)
  {
    xReadCode(2, uiCode, "bilateral_filter_strength");
    pcPPS->m_BIFStrength = uiCode; // pcPPS->setBIFStrength(uiCode);
    xReadSvlc(iCode, "bilateral_filter_qp_offset");
    pcPPS->m_BIFQPOffset = iCode; // pcPPS->setBIFQPOffset(iCode);
  }
  xReadFlag(uiCode, "chroma bilateral_filter_flag");
  pcPPS->m_chromaBIF = uiCode ? true : false; // pcPPS->setUseChromaBIF(uiCode != 0);
  if (pcPPS->m_chromaBIF)
  {
    xReadCode(2, uiCode, "chroma bilateral_filter_strength");
    pcPPS->m_chromaBIFStrength = uiCode; // pcPPS->setChromaBIFStrength(uiCode);
    xReadSvlc(iCode, "chroma bilateral_filter_qp_offset");
    pcPPS->m_chromaBIFQPOffset = iCode; // pcPPS->setChromaBIFQPOffset(iCode);
  }

  xReadFlag(uiCode, "pps_deblocking_filter_control_present_flag");
  pcPPS->m_deblockingFilterControlPresentFlag = uiCode ? true : false;
  if (pcPPS->m_deblockingFilterControlPresentFlag)
  {
    xReadFlag(uiCode, "pps_deblocking_filter_override_enabled_flag");
    pcPPS->m_deblockingFilterOverrideEnabledFlag = uiCode ? true : false;
    xReadFlag(uiCode, "pps_deblocking_filter_disabled_flag");
    pcPPS->m_ppsDeblockingFilterDisabledFlag = uiCode ? true : false;
    if (!pcPPS->m_noPicPartitionFlag && pcPPS->m_deblockingFilterOverrideEnabledFlag)
    {
      xReadFlag(uiCode, "pps_dbf_info_in_ph_flag");
      pcPPS->m_dbfInfoInPhFlag = (uiCode ? true : false);
    }
    else
    {
      pcPPS->m_dbfInfoInPhFlag = false;
    }
    if (!pcPPS->m_ppsDeblockingFilterDisabledFlag)
    {
      xReadSvlc(iCode, "pps_beta_offset_div2");
      pcPPS->m_deblockingFilterBetaOffsetDiv2 = iCode;
      CHECK(pcPPS->m_deblockingFilterBetaOffsetDiv2 < -12 || pcPPS->m_deblockingFilterBetaOffsetDiv2 > 12,
            "Invalid deblocking filter configuration");

      xReadSvlc(iCode, "pps_tc_offset_div2");
      pcPPS->m_deblockingFilterTcOffsetDiv2 = iCode;
      CHECK(pcPPS->m_deblockingFilterTcOffsetDiv2 < -12 || pcPPS->m_deblockingFilterTcOffsetDiv2 > 12,
            "Invalid deblocking filter configuration");

      if (pcPPS->m_usePPSChromaTool)
      {
        xReadSvlc(iCode, "pps_cb_beta_offset_div2");
        pcPPS->m_deblockingFilterCbBetaOffsetDiv2 = iCode;
        CHECK(pcPPS->m_deblockingFilterCbBetaOffsetDiv2 < -12 || pcPPS->m_deblockingFilterCbBetaOffsetDiv2 > 12,
              "Invalid deblocking filter configuration");

        xReadSvlc(iCode, "pps_cb_tc_offset_div2");
        pcPPS->m_deblockingFilterCbTcOffsetDiv2 = iCode;
        CHECK(pcPPS->m_deblockingFilterCbTcOffsetDiv2 < -12 || pcPPS->m_deblockingFilterCbTcOffsetDiv2 > 12,
              "Invalid deblocking filter configuration");

        xReadSvlc(iCode, "pps_cr_beta_offset_div2");
        pcPPS->m_deblockingFilterCrBetaOffsetDiv2 = iCode;
        CHECK(pcPPS->m_deblockingFilterCrBetaOffsetDiv2 < -12 || pcPPS->m_deblockingFilterCrBetaOffsetDiv2 > 12,
              "Invalid deblocking filter configuration");

        xReadSvlc(iCode, "pps_cr_tc_offset_div2");
        pcPPS->m_deblockingFilterCrTcOffsetDiv2 = iCode;
        CHECK(pcPPS->m_deblockingFilterCrTcOffsetDiv2 < -12 || pcPPS->m_deblockingFilterCrTcOffsetDiv2 > 12,
              "Invalid deblocking filter configuration");
      }
      else
      {
        pcPPS->m_deblockingFilterCbBetaOffsetDiv2 = pcPPS->m_deblockingFilterBetaOffsetDiv2;
        pcPPS->m_deblockingFilterCbTcOffsetDiv2   = pcPPS->m_deblockingFilterTcOffsetDiv2;
        pcPPS->m_deblockingFilterCrBetaOffsetDiv2 = pcPPS->m_deblockingFilterBetaOffsetDiv2;
        pcPPS->m_deblockingFilterCrTcOffsetDiv2   = pcPPS->m_deblockingFilterTcOffsetDiv2;
      }
    }
  }
  else
  {
    pcPPS->m_deblockingFilterOverrideEnabledFlag = false;
    pcPPS->m_dbfInfoInPhFlag                     = false;
  }

  if (!pcPPS->m_noPicPartitionFlag)
  {
    xReadFlag(uiCode, "pps_rpl_info_in_ph_flag");
    pcPPS->m_rplInfoInPhFlag = (uiCode ? true : false);
    xReadFlag(uiCode, "pps_sao_info_in_ph_flag");
    pcPPS->m_saoInfoInPhFlag = (uiCode ? true : false);
    xReadFlag(uiCode, "pps_alf_info_in_ph_flag");
    pcPPS->m_alfInfoInPhFlag = (uiCode ? true : false);
    if ((pcPPS->m_useWP || pcPPS->m_useBiWP) && pcPPS->m_rplInfoInPhFlag)
    {
      xReadFlag(uiCode, "pps_wp_info_in_ph_flag");
      pcPPS->m_wpInfoInPhFlag = (uiCode ? true : false);
    }
    else
    {
      pcPPS->m_wpInfoInPhFlag = false;
    }
    xReadFlag(uiCode, "pps_qp_delta_info_in_ph_flag");
    pcPPS->m_qpDeltaInfoInPhFlag = (uiCode ? true : false);
  }
  else
  {
    pcPPS->m_rplInfoInPhFlag     = false;
    pcPPS->m_saoInfoInPhFlag     = false;
    pcPPS->m_alfInfoInPhFlag     = false;
    pcPPS->m_wpInfoInPhFlag      = false;
    pcPPS->m_qpDeltaInfoInPhFlag = false;
  }

  xReadFlag(uiCode, "pps_picture_header_extension_present_flag");
  pcPPS->m_pictureHeaderExtensionPresentFlag = uiCode;
  xReadFlag(uiCode, "pps_slice_header_extension_present_flag");
  pcPPS->m_sliceHeaderExtensionPresentFlag = uiCode;

  xReadFlag(uiCode, "pps_extension_flag");

  if (uiCode)
  {
    while (xMoreRbspData())
    {
      xReadFlag(uiCode, "pps_extension_data_flag");
    }
  }
  xReadRbspTrailingBits();
}

void HLSyntaxReader::parseAPS(APS *aps)
{
#if ENABLE_TRACING
  xTraceAPSHeader();
#endif

  uint32_t code;
  xReadCode(3, code, "aps_params_type");
  aps->m_APSType = ApsType(code);

  xReadCode(5, code, "adaptation_parameter_set_id");
  aps->m_APSId = code;
  uint32_t codeApsChromaPresentFlag;
  xReadFlag(codeApsChromaPresentFlag, "aps_chroma_present_flag");
  aps->chromaPresentFlag = codeApsChromaPresentFlag;

  const ApsType apsType = aps->m_APSType;

  if (apsType == ApsType::ALF)
  {
    parseAlfAps(aps);
  }
  else if (apsType == ApsType::LMCS)
  {
    parseLmcsAps(aps);
  }
  else if (apsType == ApsType::SCALING_LIST)
  {
    parseScalingListAps(aps);
  }
  xReadFlag(code, "aps_extension_flag");
  if (code)
  {
    while (xMoreRbspData())
    {
      xReadFlag(code, "aps_extension_data_flag");
    }
  }
  xReadRbspTrailingBits();
}

void HLSyntaxReader::parseAlfAps(APS *aps)
{
  uint32_t code;

  ApsAlfParam   paramContainer      = aps->m_alfAPSParam;
  ApsCcAlfParam ccAlfParamContainer = aps->m_ccAlfAPSParam;

  // Get the parameters and check validity once. No further checks are required below.
  AlfParameters::AlfParamBase         &param      = paramContainer.getParam();
  AlfParameters::CcAlfFilterParamBase &ccAlfParam = ccAlfParamContainer.getParam();

  CHECK(ApsCcAlfParam::isEcmAlf() != ApsAlfParam::isEcmAlf(), "Version mismatch between ALF and CCALF");

  if (ApsAlfParam::isEcmAlf())
  {
    paramContainer.getEcmParam().reset();
  }
  else
  {
    paramContainer.getVtmParam().reset();
  }

  param.enabledFlag[COMP_Y] = param.enabledFlag[COMP_Cb] = param.enabledFlag[COMP_Cr] = true;
  xReadFlag(code, "alf_luma_new_filter");
  param.newFilterFlag[ChannelType::LUMA] = code;

  if (aps->chromaPresentFlag)
  {
    xReadFlag(code, "alf_chroma_new_filter");
    param.newFilterFlag[ChannelType::CHROMA] = code;
  }
  else
  {
    param.newFilterFlag[ChannelType::CHROMA] = 0;
  }

  if (aps->chromaPresentFlag)
  {
    xReadFlag(code, "alf_cc_cb_filter_signal_flag");
    ccAlfParam.newCcAlfFilter[COMP_Cb - 1] = code;
  }
  else
  {
    ccAlfParam.newCcAlfFilter[COMP_Cb - 1] = 0;
  }
  if (aps->chromaPresentFlag)
  {
    xReadFlag(code, "alf_cc_cr_filter_signal_flag");
    ccAlfParam.newCcAlfFilter[COMP_Cr - 1] = code;
  }
  else
  {
    ccAlfParam.newCcAlfFilter[COMP_Cr - 1] = 0;
  }
  CHECK(param.newFilterFlag[ChannelType::LUMA] == 0 && param.newFilterFlag[ChannelType::CHROMA] == 0 &&
          ccAlfParam.newCcAlfFilter[COMP_Cb - 1] == 0 && ccAlfParam.newCcAlfFilter[COMP_Cr - 1] == 0,
        "bitstream conformance error: one of alf_luma_filter_signal_flag, alf_chroma_filter_signal_flag, "
        "alf_cross_COMP_Cb_filter_signal_flag, and alf_cross_COMP_Cr_filter_signal_flag shall be nonzero");

  if (param.newFilterFlag[ChannelType::LUMA])
  {
    if (ApsAlfParam::isEcmAlf())
    {
      AlfParametersEcm::AlfParam &param = paramContainer.getEcmParamNoCheck();

      if (AlfParametersEcm::ALF_MAX_NUM_ALTERNATIVES_LUMA > 1)
      {
        xReadUvlc(code, "alf_luma_num_alts_minus1");
      }
      else
      {
        code = 0;
      }
      param.numAlternativesLuma = code + 1;

      xReadFlag(code, "alf_luma_13_ext_db_resi_direct : alf_luma_13_ext_db_resi");
      param.filterType[ChannelType::LUMA] = code ? AlfParametersEcm::AlfFilterType::ALF_FILTER_9_EXT_DB_RESI_DIRECT
                                                 : AlfParametersEcm::AlfFilterType::ALF_FILTER_9_EXT_DB_RESI;

      for (int altIdx = 0; altIdx < param.numAlternativesLuma; ++altIdx)
      {
        xReadFlag(code, "alf_luma_classifier_band");
        param.lumaClassifierIdx[altIdx] = code;
        if (code == 0)
        {
          xReadFlag(code, "alf_luma_classifier_resi");
          if (code)
          {
            param.lumaClassifierIdx[altIdx] = 2;
          }
        }

        xReadFlag(code, "alf_luma_clip");
        param.lumaNonLinearFlag[altIdx] = code ? true : false;

        xReadUvlc(code, "alf_luma_num_filters_signalled_minus1");
        param.numLumaFilters[altIdx] = code + 1;

        if (param.numLumaFilters[altIdx] > 1)
        {
          const int length = ceilLog2(param.numLumaFilters[altIdx]);

          for (int i = 0; i < AlfParametersEcm::ALF_NUM_CLASSES_CLASSIFIER[(int)param.lumaClassifierIdx[altIdx]]; i++)
          {
            xReadCode(length, code, "alf_luma_coeff_delta_idx");
            param.filterCoeffDeltaIdx[altIdx][i] = code;
          }
        }
        else
        {
          std::fill_n(std::begin(param.filterCoeffDeltaIdx[altIdx]), AlfParameters::MAX_NUM_ALF_CLASSES, 0);
        }

        AlfParametersEcm::AlfFilterShape alfShape(param.filterType[ChannelType::LUMA]);

        alfFilter(param, false, altIdx);
      }
    }
    else
    {
      AlfParametersVtm::AlfParam &param = paramContainer.getVtmParamNoCheck();

      xReadFlag(code, "alf_luma_clip");
      param.lumaNonLinearFlag = code ? true : false;
      xReadUvlc(code, "alf_luma_num_filters_signalled_minus1");
      param.numLumaFilters = code + 1;
      if (param.numLumaFilters > 1)
      {
        const int length = ceilLog2(param.numLumaFilters);
        for (int i = 0; i < AlfParameters::MAX_NUM_ALF_CLASSES; i++)
        {
          xReadCode(length, code, "alf_luma_coeff_delta_idx");
          param.filterCoeffDeltaIdx[i] = code;
        }
      }
      else
      {
        std::fill(std::begin(param.filterCoeffDeltaIdx), std::end(param.filterCoeffDeltaIdx), 0);
      }
      alfFilter(param, false, 0);
    }
  }

  if (param.newFilterFlag[ChannelType::CHROMA])
  {
    if (ApsAlfParam::isVtmAlf())
    {
      AlfParametersVtm::AlfParam &param = paramContainer.getVtmParamNoCheck();

      xReadFlag(code, "alf_nonlinear_enable_flag_chroma");
      param.chromaNonLinearFlag = code ? true : false;
    }

    if (ApsAlfParam::isEcmAlf())
    {
      AlfParametersEcm::AlfParam &param = paramContainer.getEcmParamNoCheck();

      param.filterType[ChannelType::CHROMA] = AlfParametersEcm::AlfFilterType::ALF_FILTER_9;
    }

    if constexpr (AlfParameters::ALF_MAX_NUM_ALTERNATIVES_CHROMA > 1)
    {
      xReadUvlc(code, "alf_chroma_num_alts_minus1");
    }
    else
    {
      code = 0;
    }
    param.numAlternativesChroma = code + 1;

    for (int altIdx = 0; altIdx < param.numAlternativesChroma; ++altIdx)
    {
      if (ApsAlfParam::isEcmAlf())
      {
        AlfParametersEcm::AlfParam &param = paramContainer.getEcmParamNoCheck();

        xReadFlag(code, "alf_nonlinear_enable_flag_chroma");
        param.chromaNonLinearFlag[altIdx] = code ? true : false;

        AlfParametersEcm::AlfFilterShape alfShape(param.filterType[ChannelType::CHROMA]);

        alfFilter(param, true, altIdx);
      }
      else
      {
        AlfParametersVtm::AlfParam &param = paramContainer.getVtmParamNoCheck();

        alfFilter(param, true, altIdx);
      }
    }
  }

  if (ApsCcAlfParam::isEcmAlf())
  {
    xParseAlfApsCcAlf<AlfParametersEcm>(ccAlfParamContainer.getEcmParamNoCheck());
  }
  else
  {
    xParseAlfApsCcAlf<AlfParametersVtm>(ccAlfParamContainer.getVtmParamNoCheck());
  }

  aps->m_ccAlfAPSParam = std::move(ccAlfParamContainer);
  aps->m_alfAPSParam   = std::move(paramContainer);
}

template<class AlfParametersT>
void HLSyntaxReader::xParseAlfApsCcAlf(typename AlfParametersT::CcAlfFilterParam &ccAlfParam)
{
  uint32_t code;

  const int maxNumCcAlfFilters = AlfParametersT::MAX_NUM_CC_ALF_FILTERS;
  const int numCoeff = typename AlfParametersT::AlfFilterShape(AlfParametersT::AlfFilterType::CC_ALF).numCoeff;

  for (int ccIdx = 0; ccIdx < 2; ccIdx++)
  {
    if (ccAlfParam.newCcAlfFilter[ccIdx])
    {
      if (maxNumCcAlfFilters > 1)
      {
        xReadUvlc(code, ccIdx == 0 ? "alf_cc_cb_filters_signalled_minus1" : "alf_cc_cr_filters_signalled_minus1");
      }
      else
      {
        code = 0;
      }
      ccAlfParam.ccAlfFilterCount[ccIdx] = code + 1;

      for (int filterIdx = 0; filterIdx < ccAlfParam.ccAlfFilterCount[ccIdx]; filterIdx++)
      {
        ccAlfParam.ccAlfFilterIdxEnabled[ccIdx][filterIdx] = true;

        // Filter coefficients
        short *const coeff = ccAlfParam.ccAlfCoeff[ccIdx][filterIdx];
        for (int i = 0; i < numCoeff - 1; i++)
        {
          xReadCode(AlfParameters::CCALF_BITS_PER_COEFF_LEVEL, code,
                    ccIdx == 0 ? "alf_cc_cb_mapped_coeff_abs" : "alf_cc_cr_mapped_coeff_abs");
          if (code == 0)
          {
            coeff[i] = 0;
          }
          else
          {
            coeff[i] = 1 << (code - 1);
            xReadFlag(code, ccIdx == 0 ? "alf_cc_cb_coeff_sign" : "alf_cc_cr_coeff_sign");
            coeff[i] *= 1 - 2 * code;
          }
        }

        DTRACE(g_trace_ctx, D_SYNTAX, "%s coeff filterIdx %d: ", ccIdx == 0 ? "Cb" : "Cr", filterIdx);
        for (int i = 0; i < numCoeff; i++)
        {
          DTRACE(g_trace_ctx, D_SYNTAX, "%d ", coeff[i]);
        }
        DTRACE(g_trace_ctx, D_SYNTAX, "\n");
      }

      for (int filterIdx = ccAlfParam.ccAlfFilterCount[ccIdx]; filterIdx < maxNumCcAlfFilters; filterIdx++)
      {
        ccAlfParam.ccAlfFilterIdxEnabled[ccIdx][filterIdx] = false;
      }
    }
  }
}

void HLSyntaxReader::parseLmcsAps(APS *aps)
{
  uint32_t code;

  SliceReshapeInfo &info = aps->m_reshapeAPSInfo;
  memset(info.reshaperModelBinCWDelta, 0, PIC_CODE_CW_BINS * sizeof(int));
  xReadUvlc(code, "lmcs_min_bin_idx");
  info.reshaperModelMinBinIdx = code;
  xReadUvlc(code, "lmcs_delta_max_bin_idx");
  info.reshaperModelMaxBinIdx = PIC_CODE_CW_BINS - 1 - code;
  xReadUvlc(code, "lmcs_delta_cw_prec_minus1");
  info.maxNbitsNeededDeltaCW = code + 1;

  for (uint32_t i = info.reshaperModelMinBinIdx; i <= info.reshaperModelMaxBinIdx; i++)
  {
    xReadCode(info.maxNbitsNeededDeltaCW, code, "lmcs_delta_abs_cw[ i ]");
    int absCW = code;
    if (absCW > 0)
    {
      xReadCode(1, code, "lmcs_delta_sign_cw_flag[ i ]");
    }
    int signCW                      = code;
    info.reshaperModelBinCWDelta[i] = (1 - 2 * signCW) * absCW;
  }
  if (aps->chromaPresentFlag)
  {
    xReadCode(3, code, "lmcs_delta_abs_crs");
  }
  int absCW = aps->chromaPresentFlag ? code : 0;
  if (absCW > 0)
  {
    xReadCode(1, code, "lmcs_delta_sign_crs_flag");
  }
  int signCW               = code;
  info.chrResScalingOffset = (1 - 2 * signCW) * absCW;

  aps->m_reshapeAPSInfo = info;
}

void HLSyntaxReader::parseScalingListAps(APS *aps)
{
  ScalingList &info = aps->m_scalingListApsInfo;
  parseScalingList(&info, aps->chromaPresentFlag);
}

void HLSyntaxReader::parseVUI(VUI *pcVUI, SPS *pcSPS)
{
#if ENABLE_TRACING
  DTRACE(g_trace_ctx, D_HEADER, "----------- vui_parameters -----------\n");
#endif
  unsigned        vuiPayloadSize = pcSPS->m_vuiParametersPresentFlag;
  InputBitstream *bs             = getBitstream();
  setBitstream(bs->extractSubstream(vuiPayloadSize * 8));

  uint32_t symbol;

  xReadFlag(symbol, "vui_progressive_source_flag");
  pcVUI->m_progressiveSourceFlag = symbol;
  xReadFlag(symbol, "vui_interlaced_source_flag");
  pcVUI->m_interlacedSourceFlag = symbol;
  xReadFlag(symbol, "vui_non_packed_constraint_flag");
  pcVUI->m_nonPackedFlag = symbol;
  xReadFlag(symbol, "vui_non_projected_constraint_flag");
  pcVUI->m_nonProjectedFlag = symbol;
  xReadFlag(symbol, "vui_aspect_ratio_info_present_flag");
  pcVUI->m_aspectRatioInfoPresentFlag = symbol;
  if (pcVUI->m_aspectRatioInfoPresentFlag)
  {
    xReadFlag(symbol, "vui_aspect_ratio_constant_flag");
    pcVUI->m_aspectRatioConstantFlag = symbol;
    xReadCode(8, symbol, "vui_aspect_ratio_idc");
    pcVUI->m_aspectRatioIdc = symbol;
    if (pcVUI->m_aspectRatioIdc == 255)
    {
      xReadCode(16, symbol, "vui_sar_width");
      pcVUI->m_sarWidth = symbol;
      xReadCode(16, symbol, "vui_sar_height");
      pcVUI->m_sarHeight = symbol;
    }
  }

  xReadFlag(symbol, "vui_overscan_info_present_flag");
  pcVUI->m_overscanInfoPresentFlag = symbol;
  if (pcVUI->m_overscanInfoPresentFlag)
  {
    xReadFlag(symbol, "vui_overscan_appropriate_flag");
    pcVUI->m_overscanAppropriateFlag = symbol;
  }

  xReadFlag(symbol, "vui_colour_description_present_flag");
  pcVUI->m_colourDescriptionPresentFlag = symbol;
  if (pcVUI->m_colourDescriptionPresentFlag)
  {
    xReadCode(8, symbol, "vui_colour_primaries");
    pcVUI->m_colourPrimaries = symbol;
    xReadCode(8, symbol, "vui_transfer_characteristics");
    pcVUI->m_transferCharacteristics = symbol;
    xReadCode(8, symbol, "vui_matrix_coeffs");
    pcVUI->m_matrixCoefficients = symbol;
    xReadFlag(symbol, "vui_full_range_flag");
    pcVUI->m_videoFullRangeFlag = symbol;
  }

  xReadFlag(symbol, "vui_chroma_loc_info_present_flag");
  pcVUI->m_chromaLocInfoPresentFlag = symbol;
  if (pcVUI->m_chromaLocInfoPresentFlag)
  {
    if (pcVUI->m_progressiveSourceFlag && !pcVUI->m_interlacedSourceFlag)
    {
      xReadUvlc(symbol, "vui_chroma_sample_loc_type");
      pcVUI->m_chromaSampleLocType = symbol;
    }
    else
    {
      xReadUvlc(symbol, "vui_chroma_sample_loc_type_top_field");
      pcVUI->m_chromaSampleLocTypeTopField = symbol;
      xReadUvlc(symbol, "vui_chroma_sample_loc_type_bottom_field");
      pcVUI->m_chromaSampleLocTypeBottomField = symbol;
    }
  }

  int payloadBitsRem = getBitstream()->getNumBitsLeft();
  if (payloadBitsRem)      // Corresponds to more_data_in_payload()
  {
    while (payloadBitsRem > 9)    // payload_extension_present()
    {
      xReadCode(1, symbol, "vui_reserved_payload_extension_data");
      payloadBitsRem--;
    }
    int finalBits        = getBitstream()->peekBits(payloadBitsRem);
    int numFinalZeroBits = 0;
    int mask             = 0xff;
    while (finalBits & (mask >> numFinalZeroBits))
    {
      numFinalZeroBits++;
    }
    while (payloadBitsRem > 9 - numFinalZeroBits)     // payload_extension_present()
    {
      xReadCode(1, symbol, "vui_reserved_payload_extension_data");
      payloadBitsRem--;
    }
    xReadFlag(symbol, "vui_payload_bit_equal_to_one");
    CHECK(symbol != 1, "vui_payload_bit_equal_to_one not equal to 1");
    payloadBitsRem--;
    while (payloadBitsRem)
    {
      xReadFlag(symbol, "vui_payload_bit_equal_to_zero");
      CHECK(symbol != 0, "vui_payload_bit_equal_to_zero not equal to 0");
      payloadBitsRem--;
    }
  }
  delete getBitstream();
  setBitstream(bs);
}

void HLSyntaxReader::parseGeneralHrdParameters(GeneralHrdParams *hrd)
{
  uint32_t symbol;
  xReadCode(32, symbol, "num_units_in_tick");
  hrd->m_numUnitsInTick = symbol;
  xReadCode(32, symbol, "time_scale");
  hrd->m_timeScale = symbol;
  xReadFlag(symbol, "general_nal_hrd_parameters_present_flag");
  hrd->m_generalNalHrdParamsPresentFlag = symbol;
  xReadFlag(symbol, "general_vcl_hrd_parameters_present_flag");
  hrd->m_generalNalHrdParamsPresentFlag = symbol;
  if (hrd->m_generalNalHrdParamsPresentFlag || hrd->m_generalVclHrdParamsPresentFlag)
  {
    xReadFlag(symbol, "general_same_pic_timing_in_all_ols_flag");
    hrd->m_generalSamePicTimingInAllOlsFlag = symbol;
    xReadFlag(symbol, "general_decoding_unit_hrd_params_present_flag");
    hrd->m_generalDecodingUnitHrdParamsPresentFlag = symbol;
    if (hrd->m_generalDecodingUnitHrdParamsPresentFlag)
    {
      xReadCode(8, symbol, "tick_divisor_minus2");
      hrd->m_tickDivisorMinus2 = symbol;
    }
    xReadCode(4, symbol, "bit_rate_scale");
    hrd->m_bitRateScale = symbol;
    xReadCode(4, symbol, "cpb_size_scale");
    hrd->m_cpbSizeScale = symbol;
    if (hrd->m_generalDecodingUnitHrdParamsPresentFlag)
    {
      xReadCode(4, symbol, "cpb_size_du_scale");
      hrd->m_cpbSizeDuScale = symbol;
    }
    xReadUvlc(symbol, "hrd_cpb_cnt_minus1");
    hrd->m_hrdCpbCntMinus1 = symbol;
    CHECK(symbol > 31, "The value of hrd_cpb_cnt_minus1 shall be in the range of 0 to 31, inclusive");
  }
}

void HLSyntaxReader::parseOlsHrdParameters(GeneralHrdParams *generalHrd, OlsHrdParams *olsHrd, uint32_t firstSubLayer,
                                           uint32_t maxNumSubLayersMinus1)
{
  uint32_t symbol;

  for (int i = firstSubLayer; i <= maxNumSubLayersMinus1; i++)
  {
    OlsHrdParams *hrd = &(olsHrd[i]);
    xReadFlag(symbol, "fixed_pic_rate_general_flag");
    hrd->m_fixedPicRateGeneralFlag = (symbol == 1 ? true : false);
    if (!hrd->m_fixedPicRateGeneralFlag)
    {
      xReadFlag(symbol, "fixed_pic_rate_within_cvs_flag");
      hrd->m_fixedPicRateWithinCvsFlag = (symbol == 1 ? true : false);
    }
    else
    {
      hrd->m_fixedPicRateWithinCvsFlag = true;
    }

    hrd->m_lowDelayHrdFlag = false; // Inferred to be 0 when not present

    if (hrd->m_fixedPicRateWithinCvsFlag)
    {
      xReadUvlc(symbol, "elemental_duration_in_tc_minus1");
      hrd->m_elementDurationInTcMinus1 = symbol;
    }
    else if ((generalHrd->m_generalNalHrdParamsPresentFlag || generalHrd->m_generalVclHrdParamsPresentFlag) &&
             generalHrd->m_hrdCpbCntMinus1 == 0)
    {
      xReadFlag(symbol, "low_delay_hrd_flag");
      hrd->m_lowDelayHrdFlag = (symbol == 1 ? true : false);
    }

    for (int nalOrVcl = 0; nalOrVcl < 2; nalOrVcl++)
    {
      if (((nalOrVcl == 0) && (generalHrd->m_generalNalHrdParamsPresentFlag)) ||
          ((nalOrVcl == 1) && (generalHrd->m_generalVclHrdParamsPresentFlag)))
      {
        for (int j = 0; j <= (generalHrd->m_hrdCpbCntMinus1); j++)
        {
          xReadUvlc(symbol, "bit_rate_value_minus1");
          hrd->m_bitRateValueMinus1[j][nalOrVcl] = symbol;
          xReadUvlc(symbol, "cpb_size_value_minus1");
          hrd->m_cpbSizeValueMinus1[j][nalOrVcl] = symbol;
          if (generalHrd->m_generalDecodingUnitHrdParamsPresentFlag)
          {
            xReadUvlc(symbol, "cpb_size_du_value_minus1");
            hrd->m_ducpbSizeValueMinus1[j][nalOrVcl] = symbol;
            xReadUvlc(symbol, "bit_rate_du_value_minus1");
            hrd->m_duBitRateValueMinus1[j][nalOrVcl] = symbol;
          }
          xReadFlag(symbol, "cbr_flag");
          hrd->m_cbrFlag[j][nalOrVcl] = (symbol == 1 ? true : false);
        }
      }
    }
  }
  for (int i = 0; i < firstSubLayer; i++)
  {
    OlsHrdParams *hrdHighestTLayer         = &(olsHrd[maxNumSubLayersMinus1]);
    OlsHrdParams *hrdTemp                  = &(olsHrd[i]);
    bool          tempFlag                 = hrdHighestTLayer->m_fixedPicRateGeneralFlag;
    hrdTemp->m_fixedPicRateGeneralFlag     = tempFlag;
    tempFlag                               = hrdHighestTLayer->m_fixedPicRateWithinCvsFlag;
    hrdTemp->m_fixedPicRateWithinCvsFlag   = tempFlag;
    uint32_t tempElementDurationInTcMinus1 = hrdHighestTLayer->m_elementDurationInTcMinus1;
    hrdTemp->m_elementDurationInTcMinus1   = tempElementDurationInTcMinus1;
    for (int nalOrVcl = 0; nalOrVcl < 2; nalOrVcl++)
    {
      if (((nalOrVcl == 0) && (generalHrd->m_generalNalHrdParamsPresentFlag)) ||
          ((nalOrVcl == 1) && (generalHrd->m_generalVclHrdParamsPresentFlag)))
      {
        for (int j = 0; j <= (generalHrd->m_hrdCpbCntMinus1); j++)
        {
          uint32_t bitRate                           = hrdHighestTLayer->m_bitRateValueMinus1[j][nalOrVcl];
          hrdTemp->m_bitRateValueMinus1[j][nalOrVcl] = bitRate;
          uint32_t cpbSize                           = hrdHighestTLayer->m_cpbSizeValueMinus1[j][nalOrVcl];
          hrdTemp->m_cpbSizeValueMinus1[j][nalOrVcl] = cpbSize;
          if (generalHrd->m_generalDecodingUnitHrdParamsPresentFlag)
          {
            uint32_t bitRateDu                           = hrdHighestTLayer->m_duBitRateValueMinus1[j][nalOrVcl];
            hrdTemp->m_duBitRateValueMinus1[j][nalOrVcl] = bitRateDu;
            uint32_t cpbSizeDu                           = hrdHighestTLayer->m_ducpbSizeValueMinus1[j][nalOrVcl];
            hrdTemp->m_ducpbSizeValueMinus1[j][nalOrVcl] = cpbSizeDu;
          }
          bool flag                       = hrdHighestTLayer->m_cbrFlag[j][nalOrVcl];
          hrdTemp->m_cbrFlag[j][nalOrVcl] = flag;
        }
      }
    }
  }
}

void HLSyntaxReader::dpb_parameters(int maxSubLayersMinus1, bool subLayerInfoFlag, SPS *pcSPS)
{
  uint32_t code;
  for (int i = (subLayerInfoFlag ? 0 : maxSubLayersMinus1); i <= maxSubLayersMinus1; i++)
  {
    xReadUvlc(code, "dpb_max_dec_pic_buffering_minus1[i]");
    pcSPS->m_maxDecPicBuffering[i] = code + 1;
    xReadUvlc(code, "dpb_max_num_reorder_pics[i]");
    pcSPS->m_maxNumReorderPics[i] = code;
    CHECK(pcSPS->m_maxNumReorderPics[i] >= pcSPS->m_maxDecPicBuffering[i],
          "The value of dpb_max_num_reorder_pics[ i ] shall be in the range of 0 to dpb_max_dec_pic_buffering_minus1[ "
          "i ], inclusive");
    xReadUvlc(code, "dpb_max_latency_increase_plus1[i]");
    pcSPS->m_maxLatencyIncreasePlus1[i] = code;
  }

  if (!subLayerInfoFlag)
  {
    for (int i = 0; i < maxSubLayersMinus1; ++i)
    {
      pcSPS->m_maxDecPicBuffering[i]      = pcSPS->m_maxDecPicBuffering[maxSubLayersMinus1];
      pcSPS->m_maxNumReorderPics[i]       = pcSPS->m_maxNumReorderPics[maxSubLayersMinus1];
      pcSPS->m_maxLatencyIncreasePlus1[i] = pcSPS->m_maxLatencyIncreasePlus1[maxSubLayersMinus1];
    }
  }
}

void HLSyntaxReader::parseSPS(SPS *pcSPS)
{
  uint32_t uiCode;

#if ENABLE_TRACING
  xTraceSPSHeader();
#endif

  xReadCode(4, uiCode, "sps_seq_parameter_set_id");
  pcSPS->m_spsId = uiCode;
  xReadCode(4, uiCode, "sps_video_parameter_set_id");
  pcSPS->m_vpsId = uiCode;
  xReadCode(3, uiCode, "sps_max_sub_layers_minus1");
  pcSPS->m_maxSubLayers = (uiCode + 1);
  CHECK(uiCode > 6, "Invalid maximum number of T-layer signalled");
  xReadCode(2, uiCode, "sps_chroma_format_idc");
  pcSPS->m_chromaFormatIdc = ChromaFormat(uiCode);

  xReadCode(2, uiCode, "sps_log2_ctu_size_minus6");
  unsigned ctbLog2SizeY = uiCode + 6;
  pcSPS->m_ctuSize      = 1 << ctbLog2SizeY;
  CHECK(uiCode > 2, "sps_log2_ctu_size_minus6 must be less than or equal to 2");
  pcSPS->m_maxCuWidth  = pcSPS->m_ctuSize;
  pcSPS->m_maxCuHeight = pcSPS->m_ctuSize;
  xReadFlag(uiCode, "sps_ptl_dpb_hrd_params_present_flag");
  pcSPS->m_ptlDpbHrdParamsPresentFlag = uiCode;

  if (!pcSPS->m_vpsId)
  {
    CHECK(!pcSPS->m_ptlDpbHrdParamsPresentFlag,
          "When sps_video_parameter_set_id is equal to 0, the value of sps_ptl_dpb_hrd_params_present_flag shall be "
          "equal to 1");
  }

  if (pcSPS->m_ptlDpbHrdParamsPresentFlag)
  {
    parseProfileTierLevel(&pcSPS->m_profileTierLevel, true, pcSPS->m_maxSubLayers - 1);
  }

  xReadFlag(uiCode, "sps_gdr_enabled_flag");
  pcSPS->m_GDREnabledFlag = uiCode;

  if (pcSPS->m_profileTierLevel.m_constraintInfo.m_noGdrConstraintFlag)
  {
    CHECK(uiCode != 0,
          "When gci_no_gdr_constraint_flag equal to 1 , the value of sps_gdr_enabled_flag shall be equal to 0");
  }

  xReadFlag(uiCode, "sps_ref_pic_resampling_enabled_flag");
  pcSPS->m_rprEnabledFlag = uiCode;
  if (pcSPS->m_profileTierLevel.m_constraintInfo.m_noRprConstraintFlag)
  {
    CHECK(uiCode != 0,
          "When gci_no_ref_pic_resampling_constraint_flag is equal to 1, sps_ref_pic_resampling_enabled_flag shall be "
          "equal to 0");
  }
  if (uiCode)
  {
    xReadFlag(uiCode, "sps_res_change_in_clvs_allowed_flag");
    pcSPS->m_resChangeInClvsEnabledFlag = uiCode;
  }
  else
  {
    pcSPS->m_resChangeInClvsEnabledFlag = 0;
  }

  if (pcSPS->m_profileTierLevel.m_constraintInfo.m_noResChangeInClvsConstraintFlag)
  {
    CHECK(uiCode != 0,
          "When no_res_change_in_clvs_constraint_flag is equal to 1, sps_res_change_in_clvs_allowed_flag shall be "
          "equal to 0");
  }

  xReadUvlc(uiCode, "sps_pic_width_max_in_luma_samples");
  pcSPS->m_maxWidthInLumaSamples = uiCode;
  xReadUvlc(uiCode, "sps_pic_height_max_in_luma_samples");
  pcSPS->m_maxHeightInLumaSamples = uiCode;
  xReadFlag(uiCode, "sps_conformance_window_flag");
  if (uiCode != 0)
  {
    Window &conf       = pcSPS->m_conformanceWindow;
    conf.m_enabledFlag = true;
    xReadUvlc(uiCode, "sps_conf_win_left_offset");
    conf.m_winLeftOffset = uiCode;
    xReadUvlc(uiCode, "sps_conf_win_right_offset");
    conf.m_winRightOffset = uiCode;
    xReadUvlc(uiCode, "sps_conf_win_top_offset");
    conf.m_winTopOffset = uiCode;
    xReadUvlc(uiCode, "sps_conf_win_bottom_offset");
    conf.m_winBottomOffset = uiCode;
  }

  xReadFlag(uiCode, "sps_subpic_info_present_flag");
  pcSPS->m_subPicInfoPresentFlag = uiCode;
  if (pcSPS->m_profileTierLevel.m_constraintInfo.m_noSubpicInfoConstraintFlag)
  {
    CHECK(uiCode != 0,
          "When gci_no_subpic_info_constraint_flag is equal to 1, the value of sps_subpic_info_present_flag shall be "
          "equal to 0");
  }

  if (pcSPS->m_subPicInfoPresentFlag)
  {
    const int maxPicWidthInCtus  = ((pcSPS->m_maxWidthInLumaSamples - 1) / pcSPS->m_ctuSize) + 1;
    const int maxPicHeightInCtus = ((pcSPS->m_maxHeightInLumaSamples - 1) / pcSPS->m_ctuSize) + 1;

    xReadUvlc(uiCode, "sps_num_subpics_minus1");
    pcSPS->setNumSubPics(uiCode + 1);
    CHECK(uiCode > maxPicWidthInCtus * maxPicHeightInCtus - 1, "Invalid sps_num_subpics_minus1 value");
    if (pcSPS->m_numSubPics == 1)
    {
      pcSPS->m_subPicCtuTopLeftX[0] = 0;
      pcSPS->m_subPicCtuTopLeftY[0] = 0;
      pcSPS->m_subPicWidth[0]       = maxPicWidthInCtus;
      pcSPS->m_subPicHeight[0]      = maxPicHeightInCtus;

      pcSPS->m_independentSubPicsFlag = 1;
      pcSPS->m_subPicSameSizeFlag     = 0;

      pcSPS->m_subPicTreatedAsPicFlag[0]            = 1;
      pcSPS->m_loopFilterAcrossSubpicEnabledFlag[0] = 0;
    }
    else
    {
      xReadFlag(uiCode, "sps_independent_subpics_flag");
      pcSPS->m_independentSubPicsFlag = uiCode != 0;
      xReadFlag(uiCode, "sps_subpic_same_size_flag");
      pcSPS->m_subPicSameSizeFlag = uiCode;
      uint32_t tmpWidthVal        = maxPicWidthInCtus;
      uint32_t tmpHeightVal       = maxPicHeightInCtus;
      uint32_t numSubpicCols      = 1;
      for (int picIdx = 0; picIdx < pcSPS->m_numSubPics; picIdx++)
      {
        if (!pcSPS->m_subPicSameSizeFlag || picIdx == 0)
        {
          if ((picIdx > 0) && (pcSPS->m_maxWidthInLumaSamples > pcSPS->m_ctuSize))
          {
            xReadCode(ceilLog2(tmpWidthVal), uiCode, "sps_subpic_ctu_top_left_x[ i ]");
            pcSPS->m_subPicCtuTopLeftX[picIdx] = uiCode;
          }
          else
          {
            pcSPS->m_subPicCtuTopLeftX[picIdx] = 0;
          }
          if ((picIdx > 0) && (pcSPS->m_maxHeightInLumaSamples > pcSPS->m_ctuSize))
          {
            xReadCode(ceilLog2(tmpHeightVal), uiCode, "sps_subpic_ctu_top_left_y[ i ]");
            pcSPS->m_subPicCtuTopLeftY[picIdx] = uiCode;
          }
          else
          {
            pcSPS->m_subPicCtuTopLeftY[picIdx] = 0;
          }
          if (picIdx < pcSPS->m_numSubPics - 1 && pcSPS->m_maxWidthInLumaSamples > pcSPS->m_ctuSize)
          {
            xReadCode(ceilLog2(tmpWidthVal), uiCode, "sps_subpic_width_minus1[ i ]");
            pcSPS->m_subPicWidth[picIdx] = uiCode + 1;
          }
          else
          {
            pcSPS->m_subPicWidth[picIdx] = tmpWidthVal - pcSPS->m_subPicCtuTopLeftX[picIdx];
          }
          if (picIdx < pcSPS->m_numSubPics - 1 && pcSPS->m_maxHeightInLumaSamples > pcSPS->m_ctuSize)
          {
            xReadCode(ceilLog2(tmpHeightVal), uiCode, "sps_subpic_height_minus1[ i ]");
            pcSPS->m_subPicHeight[picIdx] = uiCode + 1;
          }
          else
          {
            pcSPS->m_subPicHeight[picIdx] = tmpHeightVal - pcSPS->m_subPicCtuTopLeftY[picIdx];
          }
          if (pcSPS->m_subPicSameSizeFlag)
          {
            numSubpicCols = tmpWidthVal / pcSPS->m_subPicWidth[0];
            CHECK(!(tmpWidthVal % pcSPS->m_subPicWidth[0] == 0), "sps_subpic_width_minus1[0] is invalid.");
            CHECK(!(tmpHeightVal % pcSPS->m_subPicHeight[0] == 0), "sps_subpic_height_minus1[0] is invalid.");
            CHECK(!(numSubpicCols * (tmpHeightVal / pcSPS->m_subPicHeight[0]) == pcSPS->m_numSubPics),
                  "when sps_subpic_same_size_flag is equal to, sps_num_subpics_minus1 is invalid");
          }
        }
        else
        {
          pcSPS->m_subPicCtuTopLeftX[picIdx] = (picIdx % numSubpicCols) * pcSPS->m_subPicWidth[0];
          pcSPS->m_subPicCtuTopLeftY[picIdx] = (picIdx / numSubpicCols) * pcSPS->m_subPicHeight[0];
          pcSPS->m_subPicWidth[picIdx]       = pcSPS->m_subPicWidth[0];
          pcSPS->m_subPicHeight[picIdx]      = pcSPS->m_subPicHeight[0];
        }
        if (!pcSPS->m_independentSubPicsFlag)
        {
          xReadFlag(uiCode, "sps_subpic_treated_as_pic_flag[ i ]");
          pcSPS->m_subPicTreatedAsPicFlag[picIdx] = uiCode;
          xReadFlag(uiCode, "sps_loop_filter_across_subpic_enabled_flag[ i ]");
          pcSPS->m_loopFilterAcrossSubpicEnabledFlag[picIdx] = uiCode;
        }
        else
        {
          pcSPS->m_subPicTreatedAsPicFlag[picIdx]            = 1;
          pcSPS->m_loopFilterAcrossSubpicEnabledFlag[picIdx] = 0;
        }
      }
    }

    xReadUvlc(uiCode, "sps_subpic_id_len_minus1");
    pcSPS->m_subPicIdLen = (uiCode + 1);
    CHECK(uiCode > 15, "Invalid sps_subpic_id_len_minus1 value");
    CHECK((1 << (uiCode + 1)) < pcSPS->m_numSubPics, "Invalid sps_subpic_id_len_minus1 value");
    xReadFlag(uiCode, "sps_subpic_id_mapping_explicitly_signalled_flag");
    pcSPS->m_subPicIdMappingExplicitlySignalledFlag = (uiCode != 0);
    if (pcSPS->m_subPicIdMappingExplicitlySignalledFlag)
    {
      xReadFlag(uiCode, "sps_subpic_id_mapping_present_flag");
      pcSPS->m_subPicIdMappingPresentFlag = (uiCode != 0);
      if (pcSPS->m_subPicIdMappingPresentFlag)
      {
        for (int picIdx = 0; picIdx < pcSPS->m_numSubPics; picIdx++)
        {
          xReadCode(pcSPS->m_subPicIdLen, uiCode, "sps_subpic_id[i]");
          pcSPS->m_subPicId[picIdx] = uiCode;
        }
      }
    }
  }
  else
  {
    pcSPS->m_subPicIdMappingExplicitlySignalledFlag = 0;
    pcSPS->setNumSubPics(1);
    pcSPS->m_subPicCtuTopLeftX[0] = 0;
    pcSPS->m_subPicCtuTopLeftY[0] = 0;
    pcSPS->m_subPicWidth[0]  = (pcSPS->m_maxWidthInLumaSamples + pcSPS->m_ctuSize - 1) >> floorLog2(pcSPS->m_ctuSize);
    pcSPS->m_subPicHeight[0] = (pcSPS->m_maxHeightInLumaSamples + pcSPS->m_ctuSize - 1) >> floorLog2(pcSPS->m_ctuSize);
  }

  if (!pcSPS->m_subPicIdMappingExplicitlySignalledFlag || !pcSPS->m_subPicIdMappingPresentFlag)
  {
    for (int picIdx = 0; picIdx < pcSPS->m_numSubPics; picIdx++)
    {
      pcSPS->m_subPicId[picIdx] = picIdx;
    }
  }

  xReadUvlc(uiCode, "sps_bitdepth_minus8");
  CHECK(uiCode > 8, "Invalid bit depth signalled");
  const Profile::Name profile = pcSPS->m_profileTierLevel.m_profileIdc;
  if (profile != Profile::NONE)
  {
    CHECK(uiCode + 8 > ProfileFeatures::getProfileFeatures(profile)->maxBitDepth,
          "sps_bitdepth_minus8 exceeds range supported by signalled profile");
  }
  pcSPS->m_bitDepths[ChannelType::LUMA]    = 8 + uiCode;
  pcSPS->m_bitDepths[ChannelType::CHROMA]  = 8 + uiCode;
  pcSPS->m_qpBDOffset[ChannelType::LUMA]   = (int)(6 * uiCode);
  pcSPS->m_qpBDOffset[ChannelType::CHROMA] = (int)(6 * uiCode);

  xReadFlag(uiCode, "sps_entropy_coding_sync_enabled_flag");
  pcSPS->m_entropyCodingSyncEnabledFlag = (uiCode == 1);
  xReadFlag(uiCode, "sps_entry_point_offsets_present_flag");
  pcSPS->m_entryPointPresentFlag = (uiCode == 1);
  xReadCode(4, uiCode, "sps_log2_max_pic_order_cnt_lsb_minus4");
  pcSPS->m_bitsForPoc = (4 + uiCode);
  CHECK(uiCode > 12, "sps_log2_max_pic_order_cnt_lsb_minus4 shall be in the range of 0 to 12");

  xReadFlag(uiCode, "sps_poc_msb_cycle_flag");
  pcSPS->m_pocMsbCycleFlag = (uiCode ? true : false);
  if (pcSPS->m_pocMsbCycleFlag)
  {
    xReadUvlc(uiCode, "sps_poc_msb_cycle_len_minus1");
    pcSPS->m_pocMsbCycleLen = (1 + uiCode);
    CHECK(uiCode > (32 - (pcSPS->m_bitsForPoc - 4) - 5),
          "The value of sps_poc_msb_cycle_len_minus1 shall be in the range of 0 to 32 - "
          "sps_log2_max_pic_order_cnt_lsb_minus4 - 5, inclusive");
  }

  // extra bits are for future extensions, we will read, but ignore them,
  // unless a meaning is specified in the spec
  xReadCode(2, uiCode, "sps_num_extra_ph_bytes");
  pcSPS->m_numExtraPHBytes          = uiCode;
  int               numExtraPhBytes = uiCode;
  std::vector<bool> extraPhBitPresentFlags;
  extraPhBitPresentFlags.resize(8 * numExtraPhBytes);
  for (int i = 0; i < 8 * numExtraPhBytes; i++)
  {
    xReadFlag(uiCode, "sps_extra_ph_bit_present_flag[ i ]");
    extraPhBitPresentFlags[i] = uiCode;
  }
  pcSPS->m_extraPHBitPresentFlag = extraPhBitPresentFlags;
  xReadCode(2, uiCode, "sps_num_extra_sh_bytes");
  pcSPS->m_numExtraSHBytes          = uiCode;
  int               numExtraShBytes = uiCode;
  std::vector<bool> extraShBitPresentFlags;
  extraShBitPresentFlags.resize(8 * numExtraShBytes);
  for (int i = 0; i < 8 * numExtraShBytes; i++)
  {
    xReadFlag(uiCode, "sps_extra_sh_bit_present_flag[ i ]");
    extraShBitPresentFlags[i] = uiCode;
  }
  pcSPS->m_extraSHBitPresentFlag = extraShBitPresentFlags;

  if (pcSPS->m_ptlDpbHrdParamsPresentFlag)
  {
    if (pcSPS->m_maxSubLayers - 1 > 0)
    {
      xReadFlag(uiCode, "sps_sublayer_dpb_params_flag");
      pcSPS->m_subLayerDpbParamsFlag = (uiCode ? true : false);
    }
    dpb_parameters(pcSPS->m_maxSubLayers - 1, pcSPS->m_subLayerDpbParamsFlag, pcSPS);
  }
  unsigned minQT[3]  = { 0, 0, 0 };
  unsigned maxBTD[3] = { 0, 0, 0 };

  unsigned maxBTSize[3] = { 0, 0, 0 };
  unsigned maxTTSize[3] = { 0, 0, 0 };
  xReadUvlc(uiCode, "sps_log2_min_luma_coding_block_size_minus2");
  int log2MinCUSize               = uiCode + 2;
  pcSPS->m_log2MinCodingBlockSize = log2MinCUSize;
  CHECK(uiCode > ctbLog2SizeY - 2, "Invalid sps_log2_min_luma_coding_block_size_minus2 signalled");

  CHECK(log2MinCUSize > std::min(6, (int)(ctbLog2SizeY)),
        "sps_log2_min_luma_coding_block_size_minus2 shall be in the range of 0 to min (4, log2_ctu_size - 2)");
  const int minCuSize = 1 << pcSPS->m_log2MinCodingBlockSize;
  CHECK((pcSPS->m_maxWidthInLumaSamples % (std::max(8, minCuSize))) != 0,
        "Coded frame width must be a multiple of Max(8, the minimum unit size)");
  CHECK((pcSPS->m_maxHeightInLumaSamples % (std::max(8, minCuSize))) != 0,
        "Coded frame height must be a multiple of Max(8, the minimum unit size)");

  xReadFlag(uiCode, "sps_partition_constraints_override_enabled_flag");
  pcSPS->m_partitionOverrideEnabled = uiCode;
  xReadUvlc(uiCode, "sps_log2_diff_min_qt_min_cb_intra_slice_luma");
  unsigned minQtLog2SizeIntraY = uiCode + pcSPS->m_log2MinCodingBlockSize;
  minQT[0]                     = 1 << minQtLog2SizeIntraY;
  CHECK(minQT[0] > MAX_CU_SIZE,
        "The value of sps_log2_diff_min_qt_min_cb_intra_slice_luma shall be in the range of 0 to min(6,CtbLog2SizeY) - "
        "MinCbLog2Size");
  CHECK(minQT[0] > (1 << ctbLog2SizeY),
        "The value of sps_log2_diff_min_qt_min_cb_intra_slice_luma shall be in the range of 0 to min(6,CtbLog2SizeY) - "
        "MinCbLog2Size");
  xReadUvlc(uiCode, "sps_max_mtt_hierarchy_depth_intra_slice_luma");
  maxBTD[0] = uiCode;
  CHECK(uiCode > 2 * (ctbLog2SizeY - log2MinCUSize),
        "sps_max_mtt_hierarchy_depth_intra_slice_luma shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)");

  maxTTSize[0] = maxBTSize[0] = minQT[0];
  if (maxBTD[0] != 0)
  {
    xReadUvlc(uiCode, "sps_log2_diff_max_bt_min_qt_intra_slice_luma");
    maxBTSize[0] <<= uiCode;
    CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeIntraY,
          "The value of sps_log2_diff_max_bt_min_qt_intra_slice_luma shall be in the range of 0 to CtbLog2SizeY - "
          "MinQtLog2SizeIntraY");
    xReadUvlc(uiCode, "sps_log2_diff_max_tt_min_qt_intra_slice_luma");
    maxTTSize[0] <<= uiCode;
    CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeIntraY,
          "The value of sps_log2_diff_max_tt_min_qt_intra_slice_luma shall be in the range of 0 to CtbLog2SizeY - "
          "MinQtLog2SizeIntraY");
    CHECK(maxTTSize[0] > MAX_CU_SIZE,
          "The value of sps_log2_diff_max_tt_min_qt_intra_slice_luma shall be in the range of 0 to min(6,CtbLog2SizeY) "
          "- MinQtLog2SizeIntraY");
  }
  if (isChromaEnabled(pcSPS->m_chromaFormatIdc))
  {
    xReadFlag(uiCode, "sps_qtbtt_dual_tree_intra_flag");
    pcSPS->m_dualITree = uiCode;
  }
  else
  {
    pcSPS->m_dualITree = 0;
  }
  if (pcSPS->m_dualITree)
  {
    xReadUvlc(uiCode, "sps_log2_diff_min_qt_min_cb_intra_slice_chroma");
    minQT[2] = 1 << (uiCode + pcSPS->m_log2MinCodingBlockSize);
    xReadUvlc(uiCode, "sps_max_mtt_hierarchy_depth_intra_slice_chroma");
    maxBTD[2] = uiCode;
    CHECK(uiCode > 2 * (ctbLog2SizeY - log2MinCUSize),
          "sps_max_mtt_hierarchy_depth_intra_slice_chroma shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)");
    maxTTSize[2] = maxBTSize[2] = minQT[2];
    if (maxBTD[2] != 0)
    {
      xReadUvlc(uiCode, "sps_log2_diff_max_bt_min_qt_intra_slice_chroma");
      maxBTSize[2] <<= uiCode;
      xReadUvlc(uiCode, "sps_log2_diff_max_tt_min_qt_intra_slice_chroma");
      maxTTSize[2] <<= uiCode;
      CHECK(maxTTSize[2] > MAX_CU_SIZE,
            "The value of sps_log2_diff_max_tt_min_qt_intra_slice_chroma shall be in the range of 0 to "
            "min(6,CtbLog2SizeY) - MinQtLog2SizeIntraChroma");
      CHECK(maxBTSize[2] > MAX_CU_SIZE,
            "The value of sps_log2_diff_max_bt_min_qt_intra_slice_chroma shall be in the range of 0 to "
            "min(6,CtbLog2SizeY) - MinQtLog2SizeIntraChroma");
    }
  }
  xReadUvlc(uiCode, "sps_log2_diff_min_qt_min_cb_inter_slice");
  unsigned minQtLog2SizeInterY = uiCode + pcSPS->m_log2MinCodingBlockSize;
  minQT[1]                     = 1 << minQtLog2SizeInterY;
  xReadUvlc(uiCode, "sps_max_mtt_hierarchy_depth_inter_slice");
  maxBTD[1] = uiCode;
  CHECK(uiCode > 2 * (ctbLog2SizeY - log2MinCUSize),
        "sps_max_mtt_hierarchy_depth_inter_slice shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)");
  maxTTSize[1] = maxBTSize[1] = minQT[1];
  if (maxBTD[1] != 0)
  {
    xReadUvlc(uiCode, "sps_log2_diff_max_bt_min_qt_inter_slice");
    maxBTSize[1] <<= uiCode;
    CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeInterY,
          "The value of sps_log2_diff_max_bt_min_qt_inter_slice shall be in the range of 0 to CtbLog2SizeY - "
          "MinQtLog2SizeInterY");
    xReadUvlc(uiCode, "sps_log2_diff_max_tt_min_qt_inter_slice");
    maxTTSize[1] <<= uiCode;
    CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeInterY,
          "The value of sps_log2_diff_max_tt_min_qt_inter_slice shall be in the range of 0 to CtbLog2SizeY - "
          "MinQtLog2SizeInterY");
    CHECK(maxTTSize[1] > MAX_CU_SIZE,
          "The value of sps_log2_diff_max_tt_min_qt_inter_slice shall be in the range of 0 to min(6,CtbLog2SizeY) - "
          "MinQtLog2SizeInterY");
  }

  pcSPS->setMinQTSizes(minQT);
  pcSPS->setMaxMTTHierarchyDepth(maxBTD[1], maxBTD[0], maxBTD[2]);
  pcSPS->setMaxBTSize(maxBTSize[1], maxBTSize[0], maxBTSize[2]);
  pcSPS->setMaxTTSize(maxTTSize[1], maxTTSize[0], maxTTSize[2]);

  if (pcSPS->m_ctuSize > 32)
  {
    xReadUvlc(uiCode, "sps_log2_max_luma_transform_size_minus5");
    pcSPS->m_log2MaxTbSize = uiCode + 5;
  }
  else
  {
    pcSPS->m_log2MaxTbSize = 5;
  }

  xReadFlag(uiCode, "sps_transform_skip_enabled_flag");
  pcSPS->m_transformSkipEnabledFlag = (uiCode ? true : false);
  if (pcSPS->m_transformSkipEnabledFlag)
  {
    xReadUvlc(uiCode, "sps_log2_transform_skip_max_size_minus2");
    pcSPS->m_log2MaxTransformSkipBlockSize = uiCode + 2;
    xReadFlag(uiCode, "sps_bdpcm_enabled_flag");
    pcSPS->m_bdpcmEnabledFlag = uiCode ? true : false;
  }
  xReadFlag(uiCode, "sps_mts_enabled_flag");
  pcSPS->m_mtsEnabled = uiCode != 0;
  if (pcSPS->m_mtsEnabled)
  {
    xReadFlag(uiCode, "sps_explicit_mts_intra_enabled_flag");
    pcSPS->m_explicitMtsIntra = uiCode != 0;
    xReadFlag(uiCode, "sps_explicit_mts_inter_enabled_flag");
    pcSPS->m_explicitMtsInter = uiCode != 0;
    if (pcSPS->m_explicitMtsInter)
    {
      xReadFlag(uiCode, "sps_inter_mts_max_size");
      pcSPS->m_interMTSMaxSize = (uiCode ? 16 : 32);
    }
  }
  xReadFlag(uiCode, "sps_intra_lfnst_intra_slice_enabled_flag");
  pcSPS->m_useIntraLFNSTinISlice = (uiCode != 0);
  xReadFlag(uiCode, "sps_intra_lfnst_inter_slice_enabled_flag");
  pcSPS->m_useIntraLFNSTinPBSlice = (uiCode != 0);
  xReadFlag(uiCode, "sps_inter_lfnst_enabled_flag");
  pcSPS->m_useInterLFNST = (uiCode != 0);
  if (pcSPS->m_useInterLFNST)
  {
    xReadFlag(uiCode, "sps_inter_lfnst_sbt_enabled_flag");
    pcSPS->m_useInterLFNSTSBT = (uiCode != 0);
  }

  if (isChromaEnabled(pcSPS->m_chromaFormatIdc))
  {
    xReadFlag(uiCode, "sps_joint_cbcr_enabled_flag");
    pcSPS->m_jointCbCrEnabledFlag = (uiCode ? true : false);
    ChromaQpMappingTableParams chromaQpMappingTableParams;
    xReadFlag(uiCode, "sps_same_qp_table_for_chroma_flag");
    chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag = uiCode;
    int numQpTables =
      chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag ? 1 : (pcSPS->m_jointCbCrEnabledFlag ? 3 : 2);
    chromaQpMappingTableParams.m_numQpTables = numQpTables;
    for (int i = 0; i < numQpTables; i++)
    {
      int32_t qpTableStart = 0;
      xReadSvlc(qpTableStart, "sps_qp_table_starts_minus26");
      chromaQpMappingTableParams.m_qpTableStartMinus26[i] = qpTableStart;
      CHECK(qpTableStart < -26 - pcSPS->m_qpBDOffset[ChannelType::LUMA] || qpTableStart > 36,
            "The value of sps_qp_table_start_minus26[ i ] shall be in the range of -26 - QpBdOffset to 36 inclusive");
      xReadUvlc(uiCode, "sps_num_points_in_qp_table_minus1");
      chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[i] = uiCode;
      CHECK(uiCode > 36 - qpTableStart,
            "The value of sps_num_points_in_qp_table_minus1[ i ] shall be in the range of "
            "0 to 36 - sps_qp_table_start_minus26[ i ], inclusive");
      std::vector<int> deltaQpInValMinus1(chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[i] + 1);
      std::vector<int> deltaQpOutVal(chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[i] + 1);
      for (int j = 0; j <= chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[i]; j++)
      {
        xReadUvlc(uiCode, "sps_delta_qp_in_val_minus1");
        deltaQpInValMinus1[j] = uiCode;
        xReadUvlc(uiCode, "sps_delta_qp_diff_val");
        deltaQpOutVal[j] = uiCode ^ deltaQpInValMinus1[j];
      }
      chromaQpMappingTableParams.m_deltaQpInValMinus1[i] = deltaQpInValMinus1;
      chromaQpMappingTableParams.m_deltaQpOutVal[i]      = deltaQpOutVal;
    }
    pcSPS->setChromaQpMappingTableFromParams(chromaQpMappingTableParams, pcSPS->m_qpBDOffset[ChannelType::CHROMA]);
    pcSPS->deriveChromaQPMappingTables();
  }

#if ENABLE_NNLF
  xReadUvlc(uiCode, "sps_nnlf_id");
  pcSPS->m_nnlf = uiCode;
  CHECK(pcSPS->m_nnlf < NNLFUnifiedID::OFF || pcSPS->m_nnlf >= NNLFUnifiedID::MAX, "NNLFUnifiedID out of range");
#endif

  xReadFlag(uiCode, "sps_sao_enabled_flag");
  pcSPS->m_saoEnabledFlag = (uiCode ? true : false);
  xReadFlag(uiCode, "sps_ccsao_enabled_flag");
  pcSPS->m_ccSaoEnabledFlag = (uiCode ? true : false);
  xReadFlag(uiCode, "sps_ccsao_fast_flag");
  pcSPS->m_ccSaoFastFlag = (uiCode ? true : false);
  xReadFlag(uiCode, "sps_lfcccm_enabled_flag");
  pcSPS->m_lfCccmEnabledFlag = (uiCode ? true : false);
  xReadFlag(uiCode, "sps_alf_enabled_flag");
  pcSPS->m_alfEnabledFlag = (uiCode ? true : false);
  if (pcSPS->m_alfEnabledFlag)
  {
    xReadFlag(uiCode, "sps_alf_improvements_enabled_flag");
    pcSPS->m_alfImprovementsEnabledFlag = (uiCode ? true : false);
  }
  else
  {
    pcSPS->m_alfImprovementsEnabledFlag = false;
  }
  if (pcSPS->m_alfEnabledFlag && isChromaEnabled(pcSPS->m_chromaFormatIdc))
  {
    xReadFlag(uiCode, "sps_ccalf_enabled_flag");
    pcSPS->m_ccalfEnabledFlag = (uiCode ? true : false);
  }
  else
  {
    pcSPS->m_ccalfEnabledFlag = false;
  }

  xReadCode(4, uiCode, "num_predicted_coef_signs");
  pcSPS->m_numPredSign = uiCode;
  if (pcSPS->m_numPredSign)
  {
    xReadCode(2, uiCode, "log2_sign_pred_area_minus2");
    pcSPS->m_log2SignPredArea = uiCode + 2;
  }
  xReadFlag(uiCode, "temp_cabac_init_mode");
  pcSPS->m_tempCabacInitMode = (uiCode ? true : false);
  xReadFlag(uiCode, "sps_lmcs_enable_flag");
  pcSPS->m_lmcsEnabled = uiCode == 1;

  xReadFlag(uiCode, "sps_weighted_pred_flag");
  pcSPS->m_useWP = (uiCode ? true : false);
  xReadFlag(uiCode, "sps_weighted_bipred_flag");
  pcSPS->m_useBiWP = (uiCode ? true : false);

  xReadFlag(uiCode, "sps_long_term_ref_pics_flag");
  pcSPS->m_longTermRefsPresent = uiCode;
  if (pcSPS->m_vpsId > 0)
  {
    xReadFlag(uiCode, "sps_inter_layer_prediction_enabled_flag");
    pcSPS->m_interLayerPresentFlag = uiCode;
  }
  else
  {
    pcSPS->m_interLayerPresentFlag = 0;
  }
  xReadFlag(uiCode, "sps_idr_rpl_present_flag");
  pcSPS->m_idrRefParamList = (bool)uiCode;
  if (pcSPS->m_profileTierLevel.m_constraintInfo.m_noIdrRplConstraintFlag)
  {
    CHECK(uiCode != 0,
          "When gci_no_idr_rpl_constraint_flag equal to 1 , the value of sps_idr_rpl_present_flag shall be equal to 0");
  }

  xReadFlag(uiCode, "sps_rpl1_same_as_rpl0_flag");
  pcSPS->m_rpl1CopyFromRpl0Flag = uiCode;

  xReadFlag(uiCode, "sps_inter_rpl_flag");
  pcSPS->m_useInterRPL = uiCode;
  if (uiCode)
  {
    xReadUvlc(uiCode, "sps_inter_rpl_num_ref_pic_lists_minus2");
    int numberOfRPL = uiCode + 2;
    pcSPS->createRplList(RPL0, numberOfRPL);
    pcSPS->createRplList(RPL1, numberOfRPL);
    parseRefPicListInterRPL(pcSPS, numberOfRPL);

    if (pcSPS->m_rpl1CopyFromRpl0Flag)
    {
      numberOfRPL = pcSPS->m_numRpl[RPL0];
      pcSPS->createRplList(RPL1, numberOfRPL);
      RPLList *rplListSource = &pcSPS->m_rplList[RPL0];
      RPLList *rplListDest   = &pcSPS->m_rplList[RPL1];

      for (uint32_t ii = 0; ii < numberOfRPL; ii++)
      {
        copyRefPicList(pcSPS, rplListSource->getReferencePictureList(ii), rplListDest->getReferencePictureList(ii));
      }
    }
  }
  else
  {
    // Read candidate for List0
    xReadUvlc(uiCode, "sps_num_ref_pic_lists[0]");
    uint32_t numberOfRPL = uiCode;
    pcSPS->createRplList(RPL0, numberOfRPL);
    RPLList              *rplList = &pcSPS->m_rplList[RPL0];
    ReferencePictureList *rpl;
    for (uint32_t ii = 0; ii < numberOfRPL; ii++)
    {
      rpl = rplList->getReferencePictureList(ii);
      parseRefPicList(pcSPS, rpl, ii);
    }

    // Read candidate for List1
    if (!pcSPS->m_rpl1CopyFromRpl0Flag)
    {
      xReadUvlc(uiCode, "sps_num_ref_pic_lists[1]");
      numberOfRPL = uiCode;
      pcSPS->createRplList(RPL1, numberOfRPL);
      rplList = &pcSPS->m_rplList[RPL1];
      for (uint32_t ii = 0; ii < numberOfRPL; ii++)
      {
        rpl = rplList->getReferencePictureList(ii);
        parseRefPicList(pcSPS, rpl, ii);
      }
    }
    else
    {
      numberOfRPL = pcSPS->m_numRpl[RPL0];
      pcSPS->createRplList(RPL1, numberOfRPL);
      RPLList *rplListSource = &pcSPS->m_rplList[RPL0];
      RPLList *rplListDest   = &pcSPS->m_rplList[RPL1];
      for (uint32_t ii = 0; ii < numberOfRPL; ii++)
      {
        copyRefPicList(pcSPS, rplListSource->getReferencePictureList(ii), rplListDest->getReferencePictureList(ii));
      }
    }
  }

  xReadFlag(uiCode, "sps_ref_wraparound_enabled_flag");
  pcSPS->m_wrapAroundEnabledFlag = (uiCode ? true : false);

  if (pcSPS->m_wrapAroundEnabledFlag)
  {
    for (int i = 0; i < pcSPS->m_numSubPics; i++)
    {
      CHECK(pcSPS->m_subPicTreatedAsPicFlag[i] &&
              (pcSPS->m_subPicWidth[i] != (pcSPS->m_maxWidthInLumaSamples + pcSPS->m_ctuSize - 1) / pcSPS->m_ctuSize),
            "sps_ref_wraparound_enabled_flag cannot be equal to 1 when there is at least one subpicture with "
            "SubPicTreatedAsPicFlag equal to 1 and the subpicture's width is not equal to picture's width");
    }
  }

  xReadFlag(uiCode, "sps_temporal_mvp_enabled_flag");
  pcSPS->m_temporalMvpEnabledFlag = uiCode;

  if (pcSPS->m_temporalMvpEnabledFlag)
  {
    xReadFlag(uiCode, "sps_sbtmvp_enabled_flag");
    pcSPS->m_sbtmvpEnabledFlag = (uiCode != 0);
  }
  else
  {
    pcSPS->m_sbtmvpEnabledFlag = false;
  }

  xReadFlag(uiCode, "sps_amvr_enabled_flag");
  pcSPS->m_AMVREnabledFlag = uiCode != 0;

  xReadFlag(uiCode, "sps_bdof_enabled_flag");
  pcSPS->m_bdofEnabledFlag = (uiCode != 0);
  if (pcSPS->m_bdofEnabledFlag)
  {
    xReadFlag(uiCode, "sps_bdof_control_present_in_ph_flag");
    pcSPS->m_bdofControlPresentInPhFlag = (uiCode != 0);
  }
  else
  {
    pcSPS->m_bdofControlPresentInPhFlag = false;
  }
  xReadFlag(uiCode, "sps_smvd_enabled_flag");
  pcSPS->m_useSMVD = (uiCode != 0);
  xReadFlag(uiCode, "sps_mmvd_enabled_flag");
  pcSPS->m_useMMVD = (uiCode != 0);
  if (pcSPS->m_useMMVD)
  {
    xReadFlag(uiCode, "sps_mmvd_fullpel_only_flag");
    pcSPS->m_fpelMmvdEnabledFlag = (uiCode != 0);
  }
  else
  {
    pcSPS->m_fpelMmvdEnabledFlag = false;
  }

  xReadUvlc(uiCode, "sps_six_minus_max_num_merge_cand");
  CHECK(MRG_MAX_NUM_CANDS <= uiCode, "Incorrrect max number of merge candidates!");
  pcSPS->m_maxNumMergeCand = MRG_MAX_NUM_CANDS - uiCode;
  xReadFlag(uiCode, "sps_sbt_enabled_flag");
  pcSPS->m_useSBT = (uiCode != 0);
  // MULTI_PASS_DMVR
  xReadFlag(uiCode, "sps_dmvd_enabled_flag");
  pcSPS->m_useDMVD = (uiCode != 0);
  if (pcSPS->m_useDMVD && pcSPS->m_bdofEnabledFlag)
  {
    xReadFlag(uiCode, "sps_dmvd_bdof_ext_enabled_flag");
    pcSPS->m_dmvdBDOFExt = (uiCode != 0);
  }
  else
  {
    pcSPS->m_dmvdBDOFExt = false;
  }

  xReadUvlc(uiCode, "sps_six_minus_max_num_bm_merge_cand");
  CHECK(BM_MRG_MAX_NUM_CANDS < uiCode, "Incorrrect max number of BM merge candidates!");
  pcSPS->m_maxNumBMMergeCand = BM_MRG_MAX_NUM_CANDS - uiCode;

  xReadFlag(uiCode, "sps_affine_enabled_flag");
  pcSPS->m_useAffine = uiCode != 0;
  if (pcSPS->m_useAffine)
  {
    xReadUvlc(uiCode, "sps_five_minus_max_num_subblock_merge_cand");
    CHECK(uiCode > AFFINE_MRG_MAX_NUM_CANDS - (pcSPS->m_sbtmvpEnabledFlag ? 1 : 0),
          "The value of sps_five_minus_max_num_subblock_merge_cand shall be in the range of 0 to N - "
          "sps_sbtmvp_enabled_flag");
    CHECK(AFFINE_MRG_MAX_NUM_CANDS < uiCode,
          "The value of sps_five_minus_max_num_subblock_merge_cand shall be in the range of 0 to 5 - "
          "sps_sbtmvp_enabled_flag");
    pcSPS->m_maxNumAffineMergeCand = AFFINE_MRG_MAX_NUM_CANDS - uiCode;
    xReadFlag(uiCode, "sps_affine_type_flag");
    pcSPS->m_AffineType = (uiCode != 0);
    xReadFlag(uiCode, "sps_affine_mmvd_enabled_flag");
    pcSPS->m_AffineMmvdMode = (uiCode != 0);
    if (pcSPS->m_AMVREnabledFlag)
    {
      xReadFlag(uiCode, "sps_affine_amvr_enabled_flag");
      pcSPS->m_affineAmvrEnabledFlag = (uiCode != 0);
    }
    xReadUvlc(uiCode, "sps_log2_min_affine_blocksize_minus_3");
    pcSPS->m_log2MinAffineBlkSizeMinus3 = uiCode;
    xReadFlag(uiCode, "sps_affine_prof_enabled_flag");
    pcSPS->m_usePROF = (uiCode != 0);
    if (pcSPS->m_usePROF)
    {
      xReadFlag(uiCode, "sps_prof_control_present_in_ph_flag");
      pcSPS->m_profControlPresentInPhFlag = (uiCode != 0);
    }
    else
    {
      pcSPS->m_profControlPresentInPhFlag = false;
    }
    xReadFlag(uiCode, "sps_affine_nontranslation_parameter_refinement");
    pcSPS->m_affineParaRefinement = (uiCode != 0);
    xReadFlag(uiCode, "sps_affine_sub_block_merge_mode_extension");
    pcSPS->m_affineSbMrgExt = (uiCode != 0);
  }
  xReadFlag(uiCode, "sps_additionalCMVP_flag");
  pcSPS->m_useAdditionalCMVP = uiCode != 0;
  xReadFlag(uiCode, "sps_bcw_enabled_flag");
  pcSPS->m_useBcw = uiCode != 0;

  xReadFlag(uiCode, "sps_ciip_enabled_flag");
  pcSPS->m_useCiip = uiCode != 0;
  if (pcSPS->m_maxNumMergeCand >= 2)
  {
    xReadFlag(uiCode, "sps_gpm_enabled_flag");
    pcSPS->m_useGeo = uiCode != 0;
    if (pcSPS->m_useGeo)
    {
      if (pcSPS->m_maxNumMergeCand >= 3)
      {
        xReadUvlc(uiCode, "sps_max_num_merge_cand_minus_max_num_gpm_cand");
        CHECK(pcSPS->m_maxNumMergeCand - 2 < uiCode,
              "sps_max_num_merge_cand_minus_max_num_gpm_cand must not be greater than the number of merge candidates "
              "minus 2");
        pcSPS->m_maxNumGeoCand = (uint32_t)(pcSPS->m_maxNumMergeCand - uiCode);
      }
      else
      {
        pcSPS->m_maxNumGeoCand = 2;
      }
    }
  }
  else
  {
    pcSPS->m_useGeo        = 0;
    pcSPS->m_maxNumGeoCand = 0;
  }
  pcSPS->m_mergeOppositeLic = false;
  xReadFlag(uiCode, "sps_oppositelic_merge_enabled_flag");
  pcSPS->m_mergeOppositeLic = uiCode != 0;
  if (pcSPS->m_mergeOppositeLic)
  {
    xReadUvlc(uiCode, "five_minus_max_num_oppositelic_merge_cand");
    pcSPS->m_maxNumOppositeLicMergeCand = (uint32_t)(REG_MRG_MAX_NUM_CANDS_OPPOSITELIC - uiCode);
    if (pcSPS->m_useAffine)
    {
      xReadUvlc(uiCode, "eight_minus_max_num_aff_oppolic_merge_cand");
      pcSPS->m_maxNumAffineOppositeLicMergeCand = (uint32_t)(AFF_MRG_MAX_NUM_CANDS_OPPOSITELIC - uiCode);
    }
    else
    {
      pcSPS->m_maxNumAffineOppositeLicMergeCand = 0;
    }
  }
  xReadUvlc(uiCode, "sps_log2_parallel_merge_level_minus2");
  CHECK(uiCode + 2 > ctbLog2SizeY,
        "The value of sps_log2_parallel_merge_level_minus2 shall be in the range of 0 to ctbLog2SizeY - 2");
  pcSPS->m_log2ParallelMergeLevelMinus2 = uiCode;

  xReadFlag(uiCode, "sps_mrl_enabled_flag");
  pcSPS->m_useMRL = uiCode != 0;
  xReadFlag(uiCode, "sps_mip_enabled_flag");
  pcSPS->m_useMIP = uiCode != 0;
  xReadFlag(uiCode, "sps_pdp_enabled_flag");
  pcSPS->m_pdpEnabledFlag = uiCode != 0;
  xReadFlag(uiCode, "sps_dirPlanar_enabled_flag");
  pcSPS->m_usedirPlanar = uiCode != 0;
  xReadFlag(uiCode, "sps_dimd_enabled_flag");
  pcSPS->m_useDIMD = uiCode != 0;
  if (pcSPS->m_useDIMD)
  {
    xReadFlag(uiCode, "sps_obic_enabled_flag");
    pcSPS->m_useOBIC = uiCode != 0;
  }
  xReadFlag(uiCode, "sps_timd_enabled_flag");
  pcSPS->m_useTIMD = uiCode != 0;
  if (pcSPS->m_useTIMD)
  {
    xReadFlag(uiCode, "sps_timd_merge_enabled_flag");
    pcSPS->m_useTIMDMerge = uiCode != 0;
    xReadFlag(uiCode, "sps_timd_sad_enabled_flag");
    pcSPS->m_useTIMDSAD = uiCode != 0;
  }
  else
  {
    pcSPS->m_useTIMDMerge = false;
    pcSPS->m_useTIMDSAD   = false;
  }
  xReadFlag(uiCode, "sps_eip_enabled_flag");
  pcSPS->m_useEIP = uiCode != 0;
  xReadFlag(uiCode, "sps_mmeip_enabled_flag");
  pcSPS->m_useMMEIP = uiCode != 0;
  xReadFlag(uiCode, "sps_sgpm_enabled_flag");
  pcSPS->m_useSgpm = uiCode != 0;
  xReadFlag(uiCode, "sps_tempPartPred_enabled_flag");
  pcSPS->m_tempPartPredEnabledFlag = uiCode != 0;
  xReadFlag(uiCode, "sps_mcbp_enabled_flag");
  pcSPS->m_MCBP = uiCode != 0;
  xReadFlag(uiCode, "sps_tmbp_enabled_flag");
  pcSPS->m_TMBP = uiCode != 0;

  if (isChromaEnabled(pcSPS->m_chromaFormatIdc))
  {
    xReadFlag(uiCode, "sps_dimd_chroma_enabled_flag");
    pcSPS->m_useDIMDChroma = uiCode != 0;
    xReadFlag(uiCode, "sps_cclm_enabled_flag");
    pcSPS->m_LMChroma = uiCode != 0;
    if (pcSPS->m_LMChroma == 1)
    {
      xReadFlag(uiCode, "sps_cccm_enabled_flag");
      pcSPS->m_CCCM = uiCode != 0;

      if (pcSPS->m_CCCM)
      {
        xReadFlag(uiCode, "sps_bvg_cccm");
        pcSPS->m_bvgCccm = uiCode != 0;
        xReadFlag(uiCode, "sps_cc_boost_tpl_ref_sel");
        pcSPS->m_ccBoostTplRefSel = uiCode != 0;
        xReadFlag(uiCode, "sps_cc_dec_der_mode");
        pcSPS->m_ccDecDerivedMode = uiCode != 0;
      }
      xReadFlag(uiCode, "sps_cc_boost_filter");
      pcSPS->m_ccBoostFilter = uiCode != 0;
      xReadFlag(uiCode, "sps_cc_merge");
      pcSPS->m_ccMerge = uiCode != 0;
      if (pcSPS->m_ccMerge)
      {
        xReadFlag(uiCode, "sps_cc_merge_fusion");
        pcSPS->m_ccMergeFusion = uiCode != 0;
      }
    }
  }
  else
  {
    pcSPS->m_LMChroma = 0;
  }
  if (pcSPS->m_chromaFormatIdc == ChromaFormat::_420)
  {
    xReadFlag(uiCode, "sps_chroma_horizontal_collocated_flag");
    pcSPS->m_horCollocatedChromaFlag = uiCode != 0;
    xReadFlag(uiCode, "sps_chroma_vertical_collocated_flag");
    pcSPS->m_verCollocatedChromaFlag = uiCode != 0;
  }
  else
  {
    pcSPS->m_horCollocatedChromaFlag = true;
    pcSPS->m_verCollocatedChromaFlag = true;
  }
  xReadFlag(uiCode, "sps_palette_enabled_flag");
  pcSPS->m_PLTMode = (uiCode != 0);
  CHECK(
    (profile == Profile::MAIN_12 || profile == Profile::MAIN_12_INTRA || profile == Profile::MAIN_12_STILL_PICTURE) &&
      uiCode != 0,
    "sps_palette_enabled_flag shall be equal to 0 for Main 12 (420) profiles");
  if (pcSPS->m_transformSkipEnabledFlag || pcSPS->m_PLTMode)
  {
    xReadUvlc(uiCode, "sps_internal_bit_depth_minus_input_bit_depth");
    pcSPS->m_internalMinusInputBitDepth[ChannelType::LUMA] = uiCode;
    CHECK(uiCode > 8, "Invalid sps_internal_bit_depth_minus_input_bit_depth signalled");
    pcSPS->m_internalMinusInputBitDepth[ChannelType::CHROMA] = uiCode;
  }
  xReadFlag(uiCode, "sps_ibc_enabled_flag");
  pcSPS->m_ibcFlag = uiCode != 0;
  if (pcSPS->m_ibcFlag)
  {
    pcSPS->m_ibcFracFlag = 0;
    if (pcSPS->m_AMVREnabledFlag)
    {
      xReadFlag(uiCode, "sps_ibc_frac_enabled_flag");
      pcSPS->m_ibcFracFlag = uiCode;
    }
    xReadFlag(uiCode, "sps_ibc_enabled_flag_inter_slice");
    pcSPS->m_ibcFlagInterSlice = uiCode;
    xReadFlag(uiCode, "sps_ibc_merge_enabled_flag");
    pcSPS->m_ibcMerge = uiCode;
    if (pcSPS->m_ibcMerge)
    {
      xReadUvlc(uiCode, "sps_six_minus_max_num_ibc_merge_cand");
      CHECK(IBC_MRG_MAX_NUM_CANDS <= uiCode, "Incorrect max number of IBC merge candidates!");
      pcSPS->m_maxNumIBCMergeCand = IBC_MRG_MAX_NUM_CANDS - uiCode;
    }
    else
    {
      // hack to enable AMVP for IBC
      pcSPS->m_maxNumIBCMergeCand = IBC_MRG_MAX_NUM_CANDS;
      // pcSPS->m_maxNumIBCMergeCand = 0;
    }
  }
  else
  {
    pcSPS->m_maxNumIBCMergeCand = 0;
  }

  xReadFlag(uiCode, "sps_ladf_enabled_flag");
  pcSPS->m_ladfEnabled = uiCode != 0;
  if (pcSPS->m_ladfEnabled)
  {
    int signedSymbol = 0;
    xReadCode(2, uiCode, "sps_num_ladf_intervals_minus2");
    pcSPS->m_ladfNumIntervals = uiCode + 2;
    xReadSvlc(signedSymbol, "sps_ladf_lowest_interval_qp_offset");
    pcSPS->m_ladfQpOffset[0] = signedSymbol;
    for (int k = 1; k < pcSPS->m_ladfNumIntervals; k++)
    {
      xReadSvlc(signedSymbol, "sps_ladf_qp_offset");
      pcSPS->m_ladfQpOffset[k] = signedSymbol;
      xReadUvlc(uiCode, "sps_ladf_delta_threshold_minus1");
      pcSPS->m_ladfIntervalLowerBound[k] = uiCode + pcSPS->m_ladfIntervalLowerBound[k - 1] + 1;
    }
  }
  xReadFlag(uiCode, "sps_explicit_scaling_list_enabled_flag");
  pcSPS->m_scalingListEnabledFlag = uiCode;
  if (pcSPS->m_profileTierLevel.m_constraintInfo.m_noExplicitScaleListConstraintFlag)
  {
    CHECK(uiCode != 0,
          "When gci_no_explicit_scaling_list_constraint_flag is equal to 1, sps_explicit_scaling_list_enabled_flag "
          "shall be equal to 0");
  }

  if ((pcSPS->m_useIntraLFNSTinISlice || pcSPS->m_useIntraLFNSTinPBSlice || pcSPS->m_useInterLFNST) &&
      pcSPS->m_scalingListEnabledFlag)
  {
    xReadFlag(uiCode, "sps_scaling_matrix_for_lfnst_disabled_flag");
    pcSPS->m_disableScalingMatrixForLfnstBlks = uiCode ? true : false;
  }

  xReadFlag(uiCode, "sps_dep_quant_enabled_flag");
  pcSPS->m_depQuantEnabledFlag = uiCode;
  xReadFlag(uiCode, "sps_sign_data_hiding_enabled_flag");
  pcSPS->m_signDataHidingEnabledFlag = uiCode;

  xReadFlag(uiCode, "sps_obmc_flag");
  pcSPS->m_useObmc = uiCode;

  xReadFlag(uiCode, "sps_lic_enabled_flag");
  pcSPS->m_licEnabledFlag = uiCode;
  if (pcSPS->m_licEnabledFlag)
  {
    xReadFlag(uiCode, "sps_bi_lic_enabled_flag");
    pcSPS->m_biLicEnabledFlag = uiCode;
  }

  if (pcSPS->m_ptlDpbHrdParamsPresentFlag)
  {
    xReadFlag(uiCode, "sps_timing_hrd_params_present_flag");
    pcSPS->m_generalHrdParametersPresentFlag = uiCode;
    if (pcSPS->m_generalHrdParametersPresentFlag)
    {
      parseGeneralHrdParameters(&pcSPS->m_generalHrdParams);
      if ((pcSPS->m_maxSubLayers - 1) > 0)
      {
        xReadFlag(uiCode, "sps_sublayer_cpb_params_present_flag");
        pcSPS->m_SubLayerCbpParametersPresentFlag = uiCode;
      }
      else if ((pcSPS->m_maxSubLayers - 1) == 0)
      {
        pcSPS->m_SubLayerCbpParametersPresentFlag = 0;
      }

      uint32_t firstSubLayer = pcSPS->m_SubLayerCbpParametersPresentFlag ? 0 : (pcSPS->m_maxSubLayers - 1);
      parseOlsHrdParameters(&pcSPS->m_generalHrdParams, pcSPS->m_olsHrdParams, firstSubLayer,
                            pcSPS->m_maxSubLayers - 1);
    }
  }

  xReadFlag(uiCode, "sps_field_seq_flag");
  pcSPS->m_fieldSeqFlag = uiCode;
  CHECK(pcSPS->m_profileTierLevel.m_frameOnlyConstraintFlag && uiCode,
        "When ptl_frame_only_constraint_flag equal to 1 , the value of sps_field_seq_flag shall be equal to 0");

  xReadFlag(uiCode, "sps_vui_parameters_present_flag");
  pcSPS->m_vuiParametersPresentFlag = uiCode;

  if (pcSPS->m_vuiParametersPresentFlag)
  {
    xReadUvlc(uiCode, "sps_vui_payload_size_minus1");
    pcSPS->m_vuiParametersPresentFlag = uiCode + 1;
    while (!isByteAligned())
    {
      xReadFlag(uiCode, "sps_vui_alignment_zero_bit");
      CHECK(uiCode != 0, "sps_vui_alignment_zero_bit not equal to 0");
    }
    parseVUI(&pcSPS->m_vuiParameters, pcSPS);
  }

  xReadFlag(uiCode, "sps_extension_present_flag");

  if (uiCode)
  {
    static const char *syntaxStrings[] = {
      "sps_range_extension_flag", "sps_extension_7bits[0]", "sps_extension_7bits[1]", "sps_extension_7bits[2]",
      "sps_extension_7bits[3]",   "sps_extension_7bits[4]", "sps_extension_7bits[5]", "sps_extension_7bits[6]",
    };

    bool sps_extension_flags[NUM_SPS_EXTENSION_FLAGS];

    for (int i = 0; i < NUM_SPS_EXTENSION_FLAGS; i++)
    {
      xReadFlag(uiCode, syntaxStrings[i]);
      sps_extension_flags[i] = uiCode != 0;
    }

    if (pcSPS->m_bitDepths[ChannelType::LUMA] <= 10)
    {
      CHECK(sps_extension_flags[SPS_EXT__REXT] == 1,
            "The value of sps_range_extension_flag shall be 0 when BitDepth is less than or equal to 10.");
    }

    bool bSkipTrailingExtensionBits = false;
    for (int i = 0; i < NUM_SPS_EXTENSION_FLAGS; i++) // loop used so that the order is determined by the enum.
    {
      if (sps_extension_flags[i])
      {
        switch (SPSExtensionFlagIndex(i))
        {
        case SPS_EXT__REXT:
          CHECK(bSkipTrailingExtensionBits, "Skipping trailing extension bits not supported");
          {
            SPSRExt &spsRangeExtension = pcSPS->m_spsRangeExtension;
            xReadFlag(uiCode, "extended_precision_processing_flag");
            spsRangeExtension.m_extendedPrecisionProcessingFlag = (uiCode != 0);
            if (pcSPS->m_transformSkipEnabledFlag)
            {
              xReadFlag(uiCode, "sps_ts_residual_coding_rice_present_in_sh_flag");
              spsRangeExtension.m_tsrcRicePresentFlag = (uiCode != 0);
            }
            xReadFlag(uiCode, "rrc_rice_extension_flag");
            spsRangeExtension.m_rrcRiceExtensionEnableFlag = (uiCode != 0);
            xReadFlag(uiCode, "persistent_rice_adaptation_enabled_flag");
            spsRangeExtension.m_persistentRiceAdaptationEnabledFlag = (uiCode != 0);
            xReadFlag(uiCode, "reverse_last_position_enabled_flag");
            spsRangeExtension.m_reverseLastSigCoeffEnabledFlag = (uiCode != 0);
          }
          break;
        default:
          bSkipTrailingExtensionBits = true;
          break;
        }
      }
    }
    if (bSkipTrailingExtensionBits)
    {
      while (xMoreRbspData())
      {
        xReadFlag(uiCode, "sps_extension_data_flag");
      }
    }
  }
  xReadRbspTrailingBits();
}

void HLSyntaxReader::parseOPI(OPI *opi)
{
#if ENABLE_TRACING
  xTraceOPIHeader();
#endif
  uint32_t symbol;

  xReadFlag(symbol, "opi_ols_info_present_flag");
  opi->m_olsinfopresentflag = symbol;
  xReadFlag(symbol, "opi_htid_info_present_flag");
  opi->m_htidinfopresentflag = symbol;

  if (opi->m_olsinfopresentflag)
  {
    xReadUvlc(symbol, "opi_ols_idx");
    opi->m_opiolsidx = symbol;
  }

  if (opi->m_htidinfopresentflag)
  {
    xReadCode(3, symbol, "opi_htid_plus1");
    opi->m_opihtidplus1 = symbol;
  }

  xReadFlag(symbol, "opi_extension_flag");
  if (symbol)
  {
    while (xMoreRbspData())
    {
      xReadFlag(symbol, "opi_extension_data_flag");
    }
  }
  xReadRbspTrailingBits();
}

void HLSyntaxReader::parseDCI(DCI *dci)
{
#if ENABLE_TRACING
  xTraceDCIHeader();
#endif
  uint32_t symbol;

  xReadCode(4, symbol, "dci_reserved_zero_4bits");

  uint32_t numPTLs;
  xReadCode(4, numPTLs, "dci_num_ptls_minus1");
  numPTLs += 1;

  std::vector<ProfileTierLevel> ptls;
  ptls.resize(numPTLs);
  for (int i = 0; i < numPTLs; i++)
  {
    parseProfileTierLevel(&ptls[i], true, 0);
  }
  dci->m_profileTierLevel = ptls;

  xReadFlag(symbol, "dci_extension_flag");
  if (symbol)
  {
    while (xMoreRbspData())
    {
      xReadFlag(symbol, "dci_extension_data_flag");
    }
  }
  xReadRbspTrailingBits();
}

void HLSyntaxReader::parseVPS(VPS *pcVPS)
{
#if ENABLE_TRACING
  xTraceVPSHeader();
#endif
  uint32_t uiCode;

  xReadCode(4, uiCode, "vps_video_parameter_set_id");
  CHECK(uiCode == 0, "vps_video_parameter_set_id equal to zero is reserved and shall not be used in a bitstream");
  pcVPS->m_vpsId = uiCode;

  xReadCode(6, uiCode, "vps_max_layers_minus1");
  pcVPS->m_maxLayers = uiCode + 1;
  CHECK(uiCode + 1 > MAX_VPS_LAYERS, "Signalled number of layers larger than MAX_VPS_LAYERS.");
  if (pcVPS->m_maxLayers - 1 == 0)
  {
    pcVPS->m_vpsEachLayerIsAnOlsFlag = 1;
  }
  xReadCode(3, uiCode, "vps_max_sublayers_minus1");
  pcVPS->m_vpsMaxSubLayers = uiCode + 1;
  CHECK(uiCode + 1 > MAX_VPS_SUBLAYERS, "Signalled number of sublayers larger than MAX_VPS_SUBLAYERS.");
  if (pcVPS->m_maxLayers > 1 && pcVPS->m_vpsMaxSubLayers > 1)
  {
    xReadFlag(uiCode, "vps_default_ptl_dpb_hrd_max_tid_flag");
    pcVPS->m_vpsDefaultPtlDpbHrdMaxTidFlag = uiCode;
  }
  else
  {
    pcVPS->m_vpsDefaultPtlDpbHrdMaxTidFlag = 1;
  }
  if (pcVPS->m_maxLayers > 1)
  {
    xReadFlag(uiCode, "vps_all_independent_layers_flag");
    pcVPS->m_vpsAllIndependentLayersFlag = uiCode;
    if (pcVPS->m_vpsAllIndependentLayersFlag == 0)
    {
      pcVPS->m_vpsEachLayerIsAnOlsFlag = 0;
    }
  }
  std::vector<std::vector<uint32_t>> maxTidilRefPicsPlus1;
  maxTidilRefPicsPlus1.resize(pcVPS->m_maxLayers, std::vector<uint32_t>(pcVPS->m_maxLayers, NOT_VALID));
  pcVPS->setMaxTidIlRefPicsPlus1(maxTidilRefPicsPlus1);
  for (uint32_t i = 0; i < pcVPS->m_maxLayers; i++)
  {
    xReadCode(6, uiCode, "vps_layer_id");
    pcVPS->m_vpsLayerId[i]           = uiCode;
    pcVPS->m_generalLayerIdx[uiCode] = i;

    if (i > 0 && !pcVPS->m_vpsAllIndependentLayersFlag)
    {
      xReadFlag(uiCode, "vps_independent_layer_flag");
      pcVPS->m_vpsIndependentLayerFlag[i] = uiCode;
      if (!pcVPS->m_vpsIndependentLayerFlag[i])
      {
        xReadFlag(uiCode, "max_tid_ref_present_flag[ i ]");
        bool     presentFlag = uiCode;
        uint16_t sumUiCode   = 0;
        for (int j = 0, k = 0; j < i; j++)
        {
          xReadFlag(uiCode, "vps_direct_ref_layer_flag");
          pcVPS->m_vpsDirectRefLayerFlag[i][j] = uiCode;
          if (uiCode)
          {
            pcVPS->m_interLayerRefIdx[i][j]    = k;
            pcVPS->m_directRefLayerIdx[i][k++] = j;
            sumUiCode++;
          }
          if (presentFlag && pcVPS->m_vpsDirectRefLayerFlag[i][j])
          {
            xReadCode(3, uiCode, "max_tid_il_ref_pics_plus1[ i ][ j ]");
            pcVPS->setMaxTidIlRefPicsPlus1(i, j, uiCode);
          }
          else
          {
            pcVPS->setMaxTidIlRefPicsPlus1(i, j, 7);
          }
        }
        CHECK(sumUiCode == 0,
              "There has to be at least one value of j such that the value of vps_direct_dependency_flag[ i ][ j ] is "
              "equal to 1,when vps_independent_layer_flag[ i ] is equal to 0 ");
      }
    }
  }

  if (pcVPS->m_maxLayers > 1)
  {
    if (pcVPS->m_vpsAllIndependentLayersFlag)
    {
      xReadFlag(uiCode, "vps_each_layer_is_an_ols_flag");
      pcVPS->m_vpsEachLayerIsAnOlsFlag = uiCode;
      if (pcVPS->m_vpsEachLayerIsAnOlsFlag == 0)
      {
        pcVPS->m_vpsOlsModeIdc = 2;
      }
    }
    if (!pcVPS->m_vpsEachLayerIsAnOlsFlag)
    {
      if (!pcVPS->m_vpsAllIndependentLayersFlag)
      {
        xReadCode(2, uiCode, "vps_ols_mode_idc");
        pcVPS->m_vpsOlsModeIdc = uiCode;
        CHECK(uiCode > MAX_VPS_OLS_MODE_IDC, "vps_ols_mode_idc shall be in the range of 0 to 2");
      }
      if (pcVPS->m_vpsOlsModeIdc == 2)
      {
        xReadCode(8, uiCode, "vps_num_output_layer_sets_minus2");
        pcVPS->m_vpsNumOutputLayerSets       = uiCode + 2;
        pcVPS->m_vpsOlsOutputLayerFlag[0][0] = true;
        for (uint32_t i = 1; i <= pcVPS->m_vpsNumOutputLayerSets - 1; i++)
        {
          for (uint32_t j = 0; j < pcVPS->m_maxLayers; j++)
          {
            xReadFlag(uiCode, "vps_ols_output_layer_flag");
            pcVPS->m_vpsOlsOutputLayerFlag[i][j] = uiCode;
          }
        }
      }
    }
    xReadCode(8, uiCode, "vps_num_ptls_minus1");
    pcVPS->setNumPtls(uiCode + 1);
  }
  else
  {
    pcVPS->setNumPtls(1);
  }
  pcVPS->deriveOutputLayerSets();
  CHECK(pcVPS->getNumPtls() > pcVPS->m_totalNumOLSs,
        "The value of vps_num_ptls_minus1 shall be less than TotalNumOlss");
  std::vector<bool> isPTLReferred(pcVPS->getNumPtls(), false);

  for (int i = 0; i < pcVPS->getNumPtls(); i++)
  {
    if (i > 0)
    {
      xReadFlag(uiCode, "vps_pt_present_flag");
      pcVPS->m_ptPresentFlag[i] = uiCode != 0;
    }
    else
    {
      pcVPS->m_ptPresentFlag[0] = true;
    }
    if (!pcVPS->m_vpsDefaultPtlDpbHrdMaxTidFlag)
    {
      xReadCode(3, uiCode, "vps_ptl_max_tid");
      pcVPS->m_ptlMaxTemporalId[i] = uiCode;
    }
    else
    {
      pcVPS->m_ptlMaxTemporalId[i] = pcVPS->m_vpsMaxSubLayers - 1;
    }
  }
  int cnt = 0;
  while (m_pcBitstream->getNumBitsUntilByteAligned())
  {
    xReadFlag(uiCode, "vps_ptl_reserved_zero_bit");
    CHECK(uiCode != 0, "Alignment bit is not '0'");
    cnt++;
  }
  CHECK(cnt >= 8, "Read more than '8' alignment bits");

  for (int i = 0; i < pcVPS->getNumPtls(); i++)
  {
    ProfileTierLevel ptl;
    parseProfileTierLevel(&ptl, pcVPS->m_ptPresentFlag[i], pcVPS->m_ptlMaxTemporalId[i]);

    if (!pcVPS->m_ptPresentFlag[i])
    {
      CHECK(i == 0, "Profile/Tier should always be present for first entry");

      ptl.copyProfileTierConstraintsFrom(pcVPS->m_vpsProfileTierLevel[i - 1]);
    }
    pcVPS->m_vpsProfileTierLevel[i] = ptl;
  }

  for (int i = 0; i < pcVPS->m_totalNumOLSs; i++)
  {
    if (pcVPS->getNumPtls() > 1 && pcVPS->getNumPtls() != pcVPS->m_totalNumOLSs)
    {
      xReadCode(8, uiCode, "vps_ols_ptl_idx");
      pcVPS->m_olsPtlIdx[i] = uiCode;
    }
    else if (pcVPS->getNumPtls() == pcVPS->m_totalNumOLSs)
    {
      pcVPS->m_olsPtlIdx[i] = i;
    }
    else
    {
      pcVPS->m_olsPtlIdx[i] = 0;
    }
    isPTLReferred[pcVPS->m_olsPtlIdx[i]] = true;
  }
  for (int i = 0; i < pcVPS->getNumPtls(); i++)
  {
    CHECK(!isPTLReferred[i],
          "Each profile_tier_level( ) syntax structure in the VPS shall be referred to by at least one value of "
          "vps_ols_ptl_idx[ i ] for i in the range of 0 to TotalNumOlss ? 1, inclusive");
  }

  if (!pcVPS->m_vpsEachLayerIsAnOlsFlag)
  {
    xReadUvlc(uiCode, "vps_num_dpb_params_minus1");
    pcVPS->m_numDpbParams = uiCode + 1;

    CHECK(pcVPS->m_numDpbParams > pcVPS->m_numMultiLayeredOlss,
          "The value of vps_num_dpb_params_minus1 shall be in the range of 0 to NumMultiLayerOlss - 1, inclusive");
    std::vector<bool> isDPBParamReferred(pcVPS->m_numDpbParams, false);

    if (pcVPS->m_numDpbParams > 0 && pcVPS->m_vpsMaxSubLayers > 1)
    {
      xReadFlag(uiCode, "vps_sublayer_dpb_params_present_flag");
      pcVPS->m_sublayerDpbParamsPresentFlag = uiCode;
    }

    pcVPS->m_dpbParameters.resize(pcVPS->m_numDpbParams);

    for (int i = 0; i < pcVPS->m_numDpbParams; i++)
    {
      if (!pcVPS->m_vpsDefaultPtlDpbHrdMaxTidFlag)
      {
        xReadCode(3, uiCode, "vps_dpb_max_tid[i]");
        pcVPS->m_dpbMaxTemporalId.push_back(uiCode);
        CHECK(uiCode > (pcVPS->m_vpsMaxSubLayers - 1),
              "The value of vps_dpb_max_tid[i] shall be in the range of 0 to vps_max_sublayers_minus1, inclusive.")
      }
      else
      {
        pcVPS->m_dpbMaxTemporalId.push_back(pcVPS->m_vpsMaxSubLayers - 1);
      }

      for (int j = (pcVPS->m_sublayerDpbParamsPresentFlag ? 0 : pcVPS->m_dpbMaxTemporalId[i]);
           j <= pcVPS->m_dpbMaxTemporalId[i]; j++)
      {
        xReadUvlc(uiCode, "dpb_max_dec_pic_buffering_minus1[i]");
        pcVPS->m_dpbParameters[i].maxDecPicBuffering[j] = uiCode + 1;
        xReadUvlc(uiCode, "dpb_max_num_reorder_pics[i]");
        pcVPS->m_dpbParameters[i].maxNumReorderPics[j] = uiCode;
        xReadUvlc(uiCode, "dpb_max_latency_increase_plus1[i]");
        pcVPS->m_dpbParameters[i].maxLatencyIncreasePlus1[j] = uiCode;
      }

      for (int j = (pcVPS->m_sublayerDpbParamsPresentFlag ? pcVPS->m_dpbMaxTemporalId[i] : 0);
           j < pcVPS->m_dpbMaxTemporalId[i]; j++)
      {
        // When dpb_max_dec_pic_buffering_minus1[ i ] is not present for i in the range of 0 to maxSubLayersMinus1 - 1,
        // inclusive, due to subLayerInfoFlag being equal to 0, it is inferred to be equal to
        // dpb_max_dec_pic_buffering_minus1[ maxSubLayersMinus1 ].
        pcVPS->m_dpbParameters[i].maxDecPicBuffering[j] =
          pcVPS->m_dpbParameters[i].maxDecPicBuffering[pcVPS->m_dpbMaxTemporalId[i]];

        // When dpb_max_num_reorder_pics[ i ] is not present for i in the range of 0 to maxSubLayersMinus1 - 1,
        // inclusive, due to subLayerInfoFlag being equal to 0, it is inferred to be equal to dpb_max_num_reorder_pics[
        // maxSubLayersMinus1 ].
        pcVPS->m_dpbParameters[i].maxNumReorderPics[j] =
          pcVPS->m_dpbParameters[i].maxNumReorderPics[pcVPS->m_dpbMaxTemporalId[i]];

        // When dpb_max_latency_increase_plus1[ i ] is not present for i in the range of 0 to maxSubLayersMinus1 - 1,
        // inclusive, due to subLayerInfoFlag being equal to 0, it is inferred to be equal to
        // dpb_max_latency_increase_plus1[ maxSubLayersMinus1 ].
        pcVPS->m_dpbParameters[i].maxLatencyIncreasePlus1[j] =
          pcVPS->m_dpbParameters[i].maxLatencyIncreasePlus1[pcVPS->m_dpbMaxTemporalId[i]];
      }
    }

    for (int i = 0, j = 0; i < pcVPS->m_totalNumOLSs; i++)
    {
      if (pcVPS->m_numLayersInOls[i] > 1)
      {
        xReadUvlc(uiCode, "vps_ols_dpb_pic_width[i]");
        pcVPS->m_olsDpbPicSize[i].width = uiCode;
        xReadUvlc(uiCode, "vps_ols_dpb_pic_height[i]");
        pcVPS->m_olsDpbPicSize[i].height = uiCode;
        xReadCode(2, uiCode, "vps_ols_dpb_chroma_format[i]");
        pcVPS->m_olsDpbChromaFormatIdc[i] = static_cast<ChromaFormat>(uiCode);
        xReadUvlc(uiCode, "vps_ols_dpb_bitdepth_minus8[i]");
        pcVPS->m_olsDpbBitDepthMinus8[i] = uiCode;
        const Profile::Name profile      = pcVPS->m_vpsProfileTierLevel[pcVPS->m_olsPtlIdx[i]].m_profileIdc;
        if (profile != Profile::NONE)
        {
          CHECK(uiCode + 8 > ProfileFeatures::getProfileFeatures(profile)->maxBitDepth,
                "vps_ols_dpb_bitdepth_minus8[ i ] exceeds range supported by signalled profile");
        }
        if ((pcVPS->m_numDpbParams > 1) && (pcVPS->m_numDpbParams != pcVPS->m_numMultiLayeredOlss))
        {
          xReadUvlc(uiCode, "vps_ols_dpb_params_idx[i]");
          pcVPS->m_olsDpbParamsIdx[i] = uiCode;
        }
        else if (pcVPS->m_numDpbParams == 1)
        {
          pcVPS->m_olsDpbParamsIdx[i] = 0;
        }
        else
        {
          pcVPS->m_olsDpbParamsIdx[i] = j;
        }
        j += 1;
        isDPBParamReferred[pcVPS->m_olsDpbParamsIdx[i]] = true;
      }
    }
    for (int i = 0; i < pcVPS->m_numDpbParams; i++)
    {
      CHECK(!isDPBParamReferred[i],
            "Each dpb_parameters( ) syntax structure in the VPS shall be referred to by at least one value of "
            "vps_ols_dpb_params_idx[i] for i in the range of 0 to NumMultiLayerOlss - 1, inclusive");
    }
  }

  if (!pcVPS->m_vpsEachLayerIsAnOlsFlag)
  {
    xReadFlag(uiCode, "vps_general_hrd_params_present_flag");
    pcVPS->m_vpsGeneralHrdParamsPresentFlag = uiCode;
  }
  if (pcVPS->m_vpsGeneralHrdParamsPresentFlag)
  {
    parseGeneralHrdParameters(&pcVPS->m_generalHrdParams);
    if ((pcVPS->m_vpsMaxSubLayers - 1) > 0)
    {
      xReadFlag(uiCode, "vps_sublayer_cpb_params_present_flag");
      pcVPS->m_vpsSublayerCpbParamsPresentFlag = uiCode;
    }
    else
    {
      pcVPS->m_vpsSublayerCpbParamsPresentFlag = 0;
    }
    xReadUvlc(uiCode, "vps_num_ols_timing_hrd_params_minus1");
    pcVPS->m_numOlsTimingHrdParamsMinus1 = uiCode;
    CHECK(uiCode >= pcVPS->m_numMultiLayeredOlss,
          "The value of vps_num_ols_timing_hrd_params_minus1 shall be in the range of 0 to NumMultiLayerOlss - 1, "
          "inclusive");
    std::vector<bool> isHRDParamReferred(uiCode + 1, false);
    pcVPS->m_olsHrdParams.clear();
    pcVPS->m_olsHrdParams.resize(pcVPS->m_numOlsTimingHrdParamsMinus1 + 1,
                                 std::vector<OlsHrdParams>(pcVPS->m_vpsMaxSubLayers));
    for (int i = 0; i <= pcVPS->m_numOlsTimingHrdParamsMinus1; i++)
    {
      if (!pcVPS->m_vpsDefaultPtlDpbHrdMaxTidFlag)
      {
        xReadCode(3, uiCode, "vps_hrd_max_tid[i]");
        pcVPS->m_hrdMaxTid[i] = uiCode;
        CHECK(uiCode > (pcVPS->m_vpsMaxSubLayers - 1),
              "The value of vps_hrd_max_tid[i] shall be in the range of 0 to vps_max_sublayers_minus1, inclusive.")
      }
      else
      {
        pcVPS->m_hrdMaxTid[i] = pcVPS->m_vpsMaxSubLayers - 1;
      }
      uint32_t firstSublayer = pcVPS->m_vpsSublayerCpbParamsPresentFlag ? 0 : pcVPS->m_hrdMaxTid[i];
      parseOlsHrdParameters(&pcVPS->m_generalHrdParams, &pcVPS->m_olsHrdParams[i][0], firstSublayer,
                            pcVPS->m_hrdMaxTid[i]);
    }
    for (int i = pcVPS->m_numOlsTimingHrdParamsMinus1 + 1; i < pcVPS->m_totalNumOLSs; i++)
    {
      pcVPS->m_hrdMaxTid[i] = pcVPS->m_vpsMaxSubLayers - 1;
    }
    for (int i = 0; i < pcVPS->m_numMultiLayeredOlss; i++)
    {
      if (((pcVPS->m_numOlsTimingHrdParamsMinus1 + 1) != pcVPS->m_numMultiLayeredOlss) &&
          (pcVPS->m_numOlsTimingHrdParamsMinus1 > 0))
      {
        xReadUvlc(uiCode, "vps_ols_timing_hrd_idx[i]");
        pcVPS->m_olsTimingHrdIdx[i] = uiCode;
        CHECK(uiCode > pcVPS->m_numOlsTimingHrdParamsMinus1,
              "The value of vps_ols_timing_hrd_idx[[ i ] shall be in the range of 0 to "
              "vps_num_ols_timing_hrd_params_minus1, inclusive.");
      }
      else if (pcVPS->m_numOlsTimingHrdParamsMinus1 == 0)
      {
        pcVPS->m_olsTimingHrdIdx[i] = 0;
      }
      else
      {
        pcVPS->m_olsTimingHrdIdx[i] = i;
      }
      isHRDParamReferred[pcVPS->m_olsTimingHrdIdx[i]] = true;
    }
    for (int i = 0; i <= pcVPS->m_numOlsTimingHrdParamsMinus1; i++)
    {
      CHECK(!isHRDParamReferred[i],
            "Each vps_ols_timing_hrd_parameters( ) syntax structure in the VPS shall be referred to by at least one "
            "value of vps_ols_timing_hrd_idx[ i ] for i in the range of 1 to NumMultiLayerOlss - 1, inclusive");
    }
  }
  else
  {
    for (int i = 0; i < pcVPS->m_totalNumOLSs; i++)
    {
      pcVPS->m_hrdMaxTid[i] = pcVPS->m_vpsMaxSubLayers - 1;
    }
  }

  xReadFlag(uiCode, "vps_extension_flag");
  if (uiCode)
  {
    while (xMoreRbspData())
    {
      xReadFlag(uiCode, "vps_extension_data_flag");
    }
  }
  pcVPS->checkVPS();
  xReadRbspTrailingBits();
}

void HLSyntaxReader::parsePictureHeader(PicHeader *picHeader, ParameterSetManager *parameterSetManager,
                                        bool readRbspTrailingBits)
{
  uint32_t uiCode;
  int      iCode;

  PPS *pps = nullptr;
  SPS *sps = nullptr;

#if ENABLE_TRACING
  xTracePictureHeader();
#endif

  xReadFlag(uiCode, "ph_gdr_or_irap_pic_flag");
  picHeader->m_gdrOrIrapPicFlag = (uiCode != 0);
  xReadFlag(uiCode, "ph_non_ref_pic_flag");
  picHeader->m_nonReferencePictureFlag = (uiCode != 0);
  if (picHeader->m_gdrOrIrapPicFlag)
  {
    xReadFlag(uiCode, "ph_gdr_pic_flag");
    picHeader->m_gdrPicFlag = (uiCode != 0);
  }
  else
  {
    picHeader->m_gdrPicFlag = false;
  }
  xReadFlag(uiCode, "ph_inter_slice_allowed_flag");
  picHeader->m_picInterSliceAllowedFlag = (uiCode != 0);
  if (picHeader->m_picInterSliceAllowedFlag)
  {
    xReadFlag(uiCode, "ph_intra_slice_allowed_flag");
    picHeader->m_picIntraSliceAllowedFlag = (uiCode != 0);
  }
  else
  {
    picHeader->m_picIntraSliceAllowedFlag = true;
  }
  CHECK(picHeader->m_picInterSliceAllowedFlag == 0 && picHeader->m_picIntraSliceAllowedFlag == 0,
        "Invalid picture without intra or inter slice");
  // parameter sets
  xReadUvlc(uiCode, "ph_pic_parameter_set_id");
  picHeader->m_ppsId = uiCode;
  pps                = parameterSetManager->getPPS(picHeader->m_ppsId);
  CHECK(pps == 0, "Invalid PPS");
  picHeader->m_spsId = pps->m_spsId;
  sps                = parameterSetManager->getSPS(picHeader->m_spsId);
  CHECK(sps == 0, "Invalid SPS");
  xReadCode(sps->m_bitsForPoc, uiCode, "ph_pic_order_cnt_lsb");
  picHeader->m_pocLsb = uiCode;
  if (picHeader->m_gdrPicFlag)
  {
    xReadUvlc(uiCode, "ph_recovery_poc_cnt");
    picHeader->m_recoveryPocCnt = uiCode;
  }
  else
  {
    picHeader->m_recoveryPocCnt = -1;
  }

  bool isIrapOrGdrWRecoveryPocCnt0 = (picHeader->m_gdrOrIrapPicFlag && !picHeader->m_gdrPicFlag) ||
    (picHeader->m_gdrPicFlag && picHeader->m_recoveryPocCnt == 0);

  if (!isIrapOrGdrWRecoveryPocCnt0)
  {
    const Profile::Name profile        = sps->m_profileTierLevel.m_profileIdc;
    bool                isIntraProfile = profile == Profile::MAIN_12_INTRA || profile == Profile::MAIN_12_444_INTRA ||
      profile == Profile::MAIN_16_444_INTRA;

    CHECK(isIntraProfile && !isIrapOrGdrWRecoveryPocCnt0,
          "Invalid non-irap pictures or gdr pictures with ph_recovery_poc_cnt!=0 for Intra profile");
    CHECK(sps->m_profileTierLevel.m_constraintInfo.m_allRapPicturesFlag == 1 && !isIrapOrGdrWRecoveryPocCnt0,
          "gci_all_rap_pictures_flag equal to 1 specifies that all pictures in OlsInScope are IRAP pictures or GDR "
          "pictures with ph_recovery_poc_cnt equal to 0");
  }

  std::vector<bool> phExtraBitsPresent = sps->m_extraPHBitPresentFlag;
  for (int i = 0; i < sps->m_numExtraPHBytes * 8; i++)
  {
    // extra bits are ignored (when present)
    if (phExtraBitsPresent[i])
    {
      xReadFlag(uiCode, "ph_extra_bit[ i ]");
    }
  }

  if (sps->m_pocMsbCycleFlag)
  {
    xReadFlag(uiCode, "ph_poc_msb_present_flag");
    picHeader->m_pocMsbPresentFlag = (uiCode != 0);
    if (picHeader->m_pocMsbPresentFlag)
    {
      xReadCode(sps->m_pocMsbCycleLen, uiCode, "ph_poc_msb_cycle_val");
      picHeader->m_pocMsbVal = uiCode;
    }
  }

  // alf enable flags and aps IDs
  picHeader->m_ccalfEnabledFlag[COMP_Cb] = false;
  picHeader->m_ccalfEnabledFlag[COMP_Cr] = false;
  if (sps->m_alfEnabledFlag)
  {
    if (pps->m_alfInfoInPhFlag)
    {
      xReadFlag(uiCode, "ph_alf_enabled_flag");
      const bool alfEnabledFlag           = uiCode != 0;
      picHeader->m_alfEnabledFlag[COMP_Y] = alfEnabledFlag;

      bool alfCbEnabledFlag = false;
      bool alfCrEnabledFlag = false;

      AlfParameters::AlfApsList apsIds;
      if (alfEnabledFlag)
      {
        if (sps->m_alfImprovementsEnabledFlag)
        {
          xReadFlag(uiCode, "ph_alf_fixed_filter_set_cand_idx_luma");
          picHeader->m_newAlfFixFiltSetCandIdx[COMP_Y] = uiCode;
        }

        xReadCode(3, uiCode, "ph_num_alf_aps_ids_luma");
        const int numAps = uiCode;

        for (int i = 0; i < numAps; i++)
        {
          xReadCode(3, uiCode, "ph_alf_aps_id_luma");
          const int apsId = uiCode;

          apsIds.push_back(apsId);

          APS *apsToCheckLuma = parameterSetManager->getAPS(apsId, ApsType::ALF);
          CHECK(apsToCheckLuma == nullptr, "referenced APS not found");
          CHECK(apsToCheckLuma->m_alfAPSParam.getParam().newFilterFlag[ChannelType::LUMA] != 1,
                "bitstream conformance error, alf_luma_filter_signal_flag shall be equal to 1");
        }

        if (isChromaEnabled(sps->m_chromaFormatIdc))
        {
          xReadCode(1, uiCode, "ph_alf_cb_enabled_flag");
          alfCbEnabledFlag = uiCode != 0;
          xReadCode(1, uiCode, "ph_alf_cr_enabled_flag");
          alfCrEnabledFlag = uiCode != 0;
          if (sps->m_alfImprovementsEnabledFlag)
          {
            if (alfCbEnabledFlag)
            {
              xReadFlag(uiCode, "ph_alf_fixed_filter_set_cand_idx_cb");
              picHeader->m_newAlfFixFiltSetCandIdx[COMP_Cb] = uiCode;
            }
            if (alfCrEnabledFlag)
            {
              xReadFlag(uiCode, "ph_alf_fixed_filter_set_cand_idx_cr");
              picHeader->m_newAlfFixFiltSetCandIdx[COMP_Cr] = uiCode;
            }
          }
        }

        if (alfCbEnabledFlag || alfCrEnabledFlag)
        {
          xReadCode(3, uiCode, "ph_alf_aps_id_chroma");
          picHeader->m_alfApsIdChroma = uiCode;
          APS *apsToCheckChroma       = parameterSetManager->getAPS(uiCode, ApsType::ALF);
          CHECK(apsToCheckChroma == nullptr, "referenced APS not found");
          CHECK(apsToCheckChroma->m_alfAPSParam.getParam().newFilterFlag[ChannelType::CHROMA] != 1,
                "bitstream conformance error, alf_chroma_filter_signal_flag shall be equal to 1");
        }
        if (sps->m_ccalfEnabledFlag)
        {
          xReadFlag(uiCode, "ph_cc_alf_cb_enabled_flag");
          picHeader->m_ccalfEnabledFlag[COMP_Cb] = (uiCode != 0);
          picHeader->m_ccAlfCbApsId              = -1;
          if (picHeader->m_ccalfEnabledFlag[COMP_Cb])
          {
            // parse APS ID
            xReadCode(3, uiCode, "ph_cc_alf_cb_aps_id");
            picHeader->m_ccAlfCbApsId = uiCode;
            APS *apsToCheckCcCb       = parameterSetManager->getAPS(uiCode, ApsType::ALF);
            CHECK(apsToCheckCcCb == nullptr, "referenced APS not found");
            CHECK(apsToCheckCcCb->m_ccAlfAPSParam.getParam().newCcAlfFilter[COMP_Cb - 1] != 1,
                  "bitstream conformance error, alf_cc_cb_filter_signal_flag shall be equal to 1");
          }
          // Cr
          xReadFlag(uiCode, "ph_cc_alf_cr_enabled_flag");
          picHeader->m_ccalfEnabledFlag[COMP_Cr] = (uiCode != 0);
          picHeader->m_ccAlfCbApsId              = -1;
          if (picHeader->m_ccalfEnabledFlag[COMP_Cr])
          {
            // parse APS ID
            xReadCode(3, uiCode, "ph_cc_alf_cr_aps_id");
            picHeader->m_ccAlfCrApsId = uiCode;
            APS *apsToCheckCcCr       = parameterSetManager->getAPS(uiCode, ApsType::ALF);
            CHECK(apsToCheckCcCr == nullptr, "referenced APS not found");
            CHECK(apsToCheckCcCr->m_ccAlfAPSParam.getParam().newCcAlfFilter[COMP_Cr - 1] != 1,
                  "bitstream conformance error, alf_cc_cr_filter_signal_flag shall be equal to 1");
          }
        }
      }

      picHeader->m_numAlfApsIdsLuma        = (int)apsIds.size();
      picHeader->m_alfApsIdsLuma           = apsIds;
      picHeader->m_alfEnabledFlag[COMP_Cb] = alfCbEnabledFlag;
      picHeader->m_alfEnabledFlag[COMP_Cr] = alfCrEnabledFlag;
    }
    else
    {
      picHeader->m_alfEnabledFlag[COMP_Y]  = true;
      picHeader->m_alfEnabledFlag[COMP_Cb] = true;
      picHeader->m_alfEnabledFlag[COMP_Cr] = true;
    }
  }
  else
  {
    picHeader->m_alfEnabledFlag[COMP_Y]  = false;
    picHeader->m_alfEnabledFlag[COMP_Cb] = false;
    picHeader->m_alfEnabledFlag[COMP_Cr] = false;
  }
  // luma mapping / chroma scaling controls
  if (sps->m_lmcsEnabled)
  {
    xReadFlag(uiCode, "ph_lmcs_enabled_flag");
    picHeader->m_lmcsEnabledFlag = (uiCode != 0);

    if (picHeader->m_lmcsEnabledFlag)
    {
      xReadCode(2, uiCode, "ph_lmcs_aps_id");
      picHeader->m_lmcsApsId = uiCode;

      if (isChromaEnabled(sps->m_chromaFormatIdc))
      {
        xReadFlag(uiCode, "ph_chroma_residual_scale_flag");
        picHeader->m_lmcsChromaResidualScaleFlag = (uiCode != 0);
      }
      else
      {
        picHeader->m_lmcsChromaResidualScaleFlag = false;
      }
    }
  }
  else
  {
    picHeader->m_lmcsEnabledFlag             = false;
    picHeader->m_lmcsChromaResidualScaleFlag = false;
  }
  // quantization scaling lists
  if (sps->m_scalingListEnabledFlag)
  {
    xReadFlag(uiCode, "ph_explicit_scaling_list_enabled_flag");
    picHeader->m_explicitScalingListEnabledFlag = uiCode;
    if (picHeader->m_explicitScalingListEnabledFlag)
    {
      xReadCode(3, uiCode, "ph_scaling_list_aps_id");
      picHeader->m_scalingListApsId = uiCode;
    }
  }
  else
  {
    picHeader->m_explicitScalingListEnabledFlag = false;
  }
  if (pps->m_picWidthInLumaSamples == sps->m_maxWidthInLumaSamples &&
      pps->m_picHeightInLumaSamples == sps->m_maxHeightInLumaSamples)
  {
    CHECK(pps->m_conformanceWindowFlag,
          "When pps_pic_width_in_luma_samples is equal to sps_pic_width_max_in_luma_samples and "
          "pps_pic_height_in_luma_samples is equal to sps_pic_height_max_in_luma_samples, the value of "
          "pps_conformance_window_flag shall be equal to 0");
    pps->m_conformanceWindow = sps->m_conformanceWindow;
    //    pps->getConformanceWindow().setWindowLeftOffset(sps->getConformanceWindow().m_winLeftOffset);
    //    pps->getConformanceWindow().setWindowRightOffset(sps->getConformanceWindow().m_winRightOffset);
    //    pps->getConformanceWindow().setWindowTopOffset(sps->getConformanceWindow().m_winTopOffset);
    //    pps->getConformanceWindow().setWindowBottomOffset(sps->getConformanceWindow().m_winBottomOffset);
    if (!pps->m_explicitScalingWindowFlag)
    {
      pps->m_scalingWindow = pps->m_conformanceWindow;
    }
  }
  CHECK(!sps->m_rprEnabledFlag && pps->m_explicitScalingWindowFlag,
        "When sps_ref_pic_resampling_enabled_flag is equal to 0, the value of "
        "pps_scaling_window_explicit_signalling_flag shall be equal to 0");

  // initialize tile/slice info for no partitioning case

  if (pps->m_noPicPartitionFlag)
  {
    pps->resetTileSliceInfo();
    pps->setLog2CtuSize(ceilLog2(sps->m_ctuSize));
    pps->m_numExpTileCols = 1;
    pps->m_numExpTileRows = 1;
    pps->addTileColumnWidth(pps->m_picWidthInCtu);
    pps->addTileRowHeight(pps->m_picHeightInCtu);
    pps->initTiles();
    pps->m_rectSliceFlag  = 1;
    pps->m_numSlicesInPic = 1;
    pps->initRectSlices();
    pps->m_tileIdxDeltaPresentFlag = 0;
    pps->m_rectSlices[0].m_tileIdx = 0;
    pps->initRectSliceMap(sps);
    // when no Pic partition, number of sub picture shall be less than 2
    CHECK(pps->m_numSubPics >= 2, "error, no picture partitions, but have equal to or more than 2 sub pictures");
  }
  else
  {
    CHECK(pps->m_ctuSize != sps->m_ctuSize, "PPS CTU size does not match CTU size in SPS");
    if (pps->m_rectSliceFlag)
    {
      pps->initRectSliceMap(sps);
    }
  }

  pps->initSubPic(*sps);

  // set wraparound offset from PPS and SPS info
  int minCbSizeY = (1 << sps->m_log2MinCodingBlockSize);
  CHECK(!sps->m_wrapAroundEnabledFlag && pps->m_wrapAroundEnabledFlag,
        "When sps_ref_wraparound_enabled_flag is equal to 0, the value of pps_ref_wraparound_enabled_flag shall be "
        "equal to 0.");
  CHECK((((sps->m_ctuSize / minCbSizeY) + 1) > ((pps->m_picWidthInLumaSamples / minCbSizeY) - 1)) &&
          pps->m_wrapAroundEnabledFlag,
        "When the value of CtbSizeY / MinCbSizeY + 1 is greater than pps_pic_width_in_luma_samples / MinCbSizeY - 1, "
        "the value of pps_ref_wraparound_enabled_flag shall be equal to 0.");
  if (pps->m_wrapAroundEnabledFlag)
  {
    CHECK((pps->m_picWidthMinusWrapAroundOffset >
           (pps->m_picWidthInLumaSamples / minCbSizeY - sps->m_ctuSize / minCbSizeY - 2)),
          "pps_pic_width_minus_wraparound_ofsfet shall be less than or equal to "
          "pps_pic_width_in_luma_samples/MinCbSizeY - CtbSizeY/MinCbSizeY-2");
    pps->m_wrapAroundOffset =
      minCbSizeY * (pps->m_picWidthInLumaSamples / minCbSizeY - pps->m_picWidthMinusWrapAroundOffset);
  }
  else
  {
    pps->m_wrapAroundOffset = 0;
  }

  // picture output flag
  if (pps->m_outputFlagPresentFlag && !picHeader->m_nonReferencePictureFlag)
  {
    xReadFlag(uiCode, "ph_pic_output_flag");
    picHeader->m_picOutputFlag = (uiCode != 0);
  }
  else
  {
    picHeader->m_picOutputFlag = true;
  }

  // reference picture lists
  if (pps->m_rplInfoInPhFlag)
  {
    bool rplSpsFlag = false;

    for (const auto l: { RPL0, RPL1 })
    {
      int numRplsInSps = sps->m_numRpl[l];
      if (numRplsInSps == 0)
      {
        rplSpsFlag = false;
      }
      else if (l == RPL0 || pps->m_rpl1IdxPresentFlag)
      {
        xReadFlag(uiCode, "rpl_sps_flag[i]");
        rplSpsFlag = uiCode != 0;
      }

      ReferencePictureList *rpl = &picHeader->m_rpl[l];
      if (!rplSpsFlag)
      {
        // explicit RPL in picture header
        *rpl = ReferencePictureList();
        parseRefPicList(sps, rpl, -1);
        picHeader->m_rplIdx[l] = -1;
      }
      else
      {
        // use list from SPS
        int rplIdx = 0;

        if (numRplsInSps > 1 && (l == RPL0 || pps->m_rpl1IdxPresentFlag))
        {
          int numBits = ceilLog2(numRplsInSps);
          xReadCode(numBits, uiCode, "rpl_idx[i]");
          rplIdx = uiCode;
        }
        else if (numRplsInSps != 1)
        {
          rplIdx = picHeader->m_rplIdx[RPL0];
          CHECK(rplIdx == -1, "There should be a list 0 RPL");
        }

        picHeader->m_rplIdx[l] = rplIdx;
        *rpl                   = *sps->m_rplList[l].getReferencePictureList(rplIdx);
      }
      if (picHeader->m_picInterSliceAllowedFlag && l == RPL0)
      {
        CHECK(picHeader->m_rpl[RPL0].getNumRefEntries() <= 0,
              "When pps_rpl_info_in_ph_flag is equal to 1 and ph_inter_slice_allowed_flag is equal to 1, the value of "
              "num_ref_entries[ 0 ][ RplsIdx[ 0 ] ] shall be greater than 0");
      }
      // POC MSB cycle signalling for LTRP
      for (int i = 0; i < rpl->getNumRefEntries(); i++)
      {
        rpl->m_deltaPocMSBPresentFlag[i] = false;
        rpl->m_deltaPOCMSBCycleLT[i]     = 0;
      }
      if (rpl->m_numberOfLongtermPictures)
      {
        for (int i = 0; i < rpl->getNumRefEntries(); i++)
        {
          if (rpl->m_isLongtermRefPic[i])
          {
            if (rpl->m_ltrpInSliceHeaderFlag)
            {
              xReadCode(sps->m_bitsForPoc, uiCode, "poc_lsb_lt[i][j]");
              rpl->setRefPicIdentifier(i, uiCode, true, false, 0);
            }
            xReadFlag(uiCode, "delta_poc_msb_present_flag[i][j]");
            rpl->m_deltaPocMSBPresentFlag[i] = uiCode ? true : false;
            if (uiCode)
            {
              xReadUvlc(uiCode, "delta_poc_msb_cycle_lt[i][j]");
              if (i != 0)
              {
                uiCode += rpl->m_deltaPOCMSBCycleLT[i - 1];
              }
              rpl->m_deltaPOCMSBCycleLT[i] = uiCode;
            }
            else if (i != 0)
            {
              rpl->m_deltaPOCMSBCycleLT[i] = rpl->m_deltaPOCMSBCycleLT[i - 1];
            }
            else
            {
              rpl->m_deltaPOCMSBCycleLT[i] = 0;
            }
          }
          else if (i != 0)
          {
            rpl->m_deltaPOCMSBCycleLT[i] = rpl->m_deltaPOCMSBCycleLT[i - 1];
          }
          else
          {
            rpl->m_deltaPOCMSBCycleLT[i] = 0;
          }
        }
      }
    }
  }

  // partitioning constraint overrides
  if (sps->m_partitionOverrideEnabled)
  {
    xReadFlag(uiCode, "ph_partition_constraints_override_flag");
    picHeader->m_splitConsOverrideFlag = (uiCode != 0);
  }
  else
  {
    picHeader->m_splitConsOverrideFlag = false;
  }
  // Q0781, two-flags
  unsigned minQT[3]     = { 0, 0, 0 };
  unsigned maxBTD[3]    = { 0, 0, 0 };
  unsigned maxBTSize[3] = { 0, 0, 0 };
  unsigned maxTTSize[3] = { 0, 0, 0 };
  unsigned ctbLog2SizeY = floorLog2(sps->m_ctuSize);

  if (picHeader->m_picIntraSliceAllowedFlag)
  {
    if (picHeader->m_splitConsOverrideFlag)
    {
      xReadUvlc(uiCode, "ph_log2_diff_min_qt_min_cb_intra_slice_luma");
      unsigned minQtLog2SizeIntraY = uiCode + sps->m_log2MinCodingBlockSize;
      minQT[0]                     = 1 << minQtLog2SizeIntraY;
      CHECK(minQT[0] > MAX_CU_SIZE,
            "The value of ph_log2_diff_min_qt_min_cb_intra_slice_luma shall be in the range of "
            "0 to min(6,CtbLog2SizeY) - MinCbLog2Size");
      xReadUvlc(uiCode, "ph_max_mtt_hierarchy_depth_intra_slice_luma");
      maxBTD[0] = uiCode;

      maxTTSize[0] = maxBTSize[0] = minQT[0];
      if (maxBTD[0] != 0)
      {
        xReadUvlc(uiCode, "ph_log2_diff_max_bt_min_qt_intra_slice_luma");
        maxBTSize[0] <<= uiCode;
        CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeIntraY,
              "The value of ph_log2_diff_max_bt_min_qt_intra_slice_luma shall be in the range of 0 to CtbLog2SizeY - "
              "MinQtLog2SizeIntraY");
        xReadUvlc(uiCode, "ph_log2_diff_max_tt_min_qt_intra_slice_luma");
        maxTTSize[0] <<= uiCode;
        CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeIntraY,
              "The value of ph_log2_diff_max_tt_min_qt_intra_slice_luma shall be in the range of 0 to CtbLog2SizeY - "
              "MinQtLog2SizeIntraY");
        CHECK(maxTTSize[0] > MAX_CU_SIZE,
              "The value of ph_log2_diff_max_tt_min_qt_intra_slice_luma shall be in the "
              "range of 0 to min(6,CtbLog2SizeY) - MinQtLog2SizeIntraY");
      }

      if (sps->m_dualITree)
      {
        xReadUvlc(uiCode, "ph_log2_diff_min_qt_min_cb_intra_slice_chroma");
        minQT[2] = 1 << (uiCode + sps->m_log2MinCodingBlockSize);
        CHECK(minQT[2] > MAX_CU_SIZE,
              "The value of ph_log2_diff_min_qt_min_cb_intra_slice_chroma shall be in the "
              "range of 0 to min(6,CtbLog2SizeY) - MinCbLog2Size");
        xReadUvlc(uiCode, "ph_max_mtt_hierarchy_depth_intra_slice_chroma");
        maxBTD[2]    = uiCode;
        maxTTSize[2] = maxBTSize[2] = minQT[2];
        if (maxBTD[2] != 0)
        {
          xReadUvlc(uiCode, "ph_log2_diff_max_bt_min_qt_intra_slice_chroma");
          maxBTSize[2] <<= uiCode;
          xReadUvlc(uiCode, "ph_log2_diff_max_tt_min_qt_intra_slice_chroma");
          maxTTSize[2] <<= uiCode;
          CHECK(maxBTSize[2] > MAX_CU_SIZE,
                "The value of ph_log2_diff_max_bt_min_qt_intra_slice_chroma shall be in the range of 0 to "
                "min(6,CtbLog2SizeY) - MinQtLog2SizeIntraChroma");
          CHECK(maxTTSize[2] > MAX_CU_SIZE,
                "The value of ph_log2_diff_max_tt_min_qt_intra_slice_chroma shall be in the range of 0 to "
                "min(6,CtbLog2SizeY) - MinQtLog2SizeIntraChroma");
        }
      }
    }
  }

  if (picHeader->m_picIntraSliceAllowedFlag)
  {
    // delta quantization and chrom and chroma offset
    if (pps->m_useDQP)
    {
      xReadUvlc(uiCode, "ph_cu_qp_delta_subdiv_intra_slice");
      picHeader->m_cuQpDeltaSubdivIntra = uiCode;
    }
    else
    {
      picHeader->m_cuQpDeltaSubdivIntra = 0;
    }
    if (pps->getCuChromaQpOffsetListEnabledFlag())
    {
      xReadUvlc(uiCode, "ph_cu_chroma_qp_offset_subdiv_intra_slice");
      picHeader->m_cuChromaQpOffsetSubdivIntra = uiCode;
    }
    else
    {
      picHeader->m_cuChromaQpOffsetSubdivIntra = 0;
    }
  }

  if (picHeader->m_picInterSliceAllowedFlag)
  {
    if (picHeader->m_splitConsOverrideFlag)
    {
      xReadUvlc(uiCode, "ph_log2_diff_min_qt_min_cb_inter_slice");
      unsigned minQtLog2SizeInterY = uiCode + sps->m_log2MinCodingBlockSize;
      minQT[1]                     = 1 << minQtLog2SizeInterY;
      CHECK(minQT[1] > 64,
            "The value of ph_log2_diff_min_qt_min_cb_inter_slice shall be in the range of 0 to min(6, CtbLog2SizeY) - "
            "MinCbLog2SizeY.");
      CHECK(minQT[1] > (1 << ctbLog2SizeY),
            "The value of ph_log2_diff_min_qt_min_cb_inter_slice shall be in the range of 0 to min(6, CtbLog2SizeY) - "
            "MinCbLog2SizeY");
      xReadUvlc(uiCode, "ph_max_mtt_hierarchy_depth_inter_slice");
      maxBTD[1] = uiCode;

      maxTTSize[1] = maxBTSize[1] = minQT[1];
      if (maxBTD[1] != 0)
      {
        xReadUvlc(uiCode, "ph_log2_diff_max_bt_min_qt_inter_slice");
        maxBTSize[1] <<= uiCode;
        CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeInterY,
              "The value of ph_log2_diff_max_bt_min_qt_inter_slice shall be in the range of 0 to CtbLog2SizeY - "
              "MinQtLog2SizeInterY");
        xReadUvlc(uiCode, "ph_log2_diff_max_tt_min_qt_inter_slice");
        maxTTSize[1] <<= uiCode;
        CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeInterY,
              "The value of ph_log2_diff_max_tt_min_qt_inter_slice shall be in the range of 0 to CtbLog2SizeY - "
              "MinQtLog2SizeInterY");
        CHECK(maxTTSize[1] > MAX_CU_SIZE,
              "The value of ph_log2_diff_max_tt_min_qt_inter_slice shall be in the range of 0 to min(6,CtbLog2SizeY) - "
              "MinQtLog2SizeInterY.");
      }
    }
    // delta quantization and chrom and chroma offset
    if (pps->m_useDQP)
    {
      xReadUvlc(uiCode, "ph_cu_qp_delta_subdiv_inter_slice");
      picHeader->m_cuQpDeltaSubdivInter = uiCode;
    }
    else
    {
      picHeader->m_cuQpDeltaSubdivInter = 0;
    }
    if (pps->getCuChromaQpOffsetListEnabledFlag())
    {
      xReadUvlc(uiCode, "ph_cu_chroma_qp_offset_subdiv_inter_slice");
      picHeader->m_cuChromaQpOffsetSubdivInter = uiCode;
    }
    else
    {
      picHeader->m_cuChromaQpOffsetSubdivInter = 0;
    }

    // temporal motion vector prediction
    if (sps->m_temporalMvpEnabledFlag)
    {
      xReadFlag(uiCode, "ph_temporal_mvp_enabled_flag");
      picHeader->m_enableTMVPFlag = uiCode != 0;
    }
    else
    {
      picHeader->m_enableTMVPFlag = false;
    }

    if (picHeader->m_enableTMVPFlag && pps->m_rplInfoInPhFlag)
    {
      if (picHeader->m_rpl[RPL1].getNumRefEntries() > 0)
      {
        xReadCode(1, uiCode, "ph_collocated_from_l0_flag");
        picHeader->m_picColFromL0Flag = uiCode;
      }
      else
      {
        picHeader->m_picColFromL0Flag = true;
      }
      if (picHeader->m_rpl[picHeader->m_picColFromL0Flag ? RPL0 : RPL1].getNumRefEntries() > 1)
      {
        xReadUvlc(uiCode, "ph_collocated_ref_idx");
        picHeader->m_colRefIdx = uiCode;
      }
      else
      {
        picHeader->m_colRefIdx = 0;
      }
    }
    else
    {
      picHeader->m_picColFromL0Flag = false;
    }

    // merge candidate list size
    // subblock merge candidate list size
    if (sps->m_useAffine)
    {
      picHeader->m_maxNumAffineMergeCand = sps->m_maxNumAffineMergeCand;
    }
    else
    {
      picHeader->m_maxNumAffineMergeCand = sps->m_sbtmvpEnabledFlag && picHeader->m_enableTMVPFlag;
    }

    // full-pel MMVD flag
    if (sps->m_fpelMmvdEnabledFlag)
    {
      xReadFlag(uiCode, "ph_fpel_mmvd_enabled_flag");
      picHeader->m_disFracMMVD = (uiCode != 0);
    }
    else
    {
      picHeader->m_disFracMMVD = false;
    }
    if (sps->m_useGeo)
    {
      xReadFlag(uiCode, "ph_gpm_ext_mmvd_flag");
      picHeader->m_gpmMMVDTableFlag = uiCode;
    }
    else
    {
      picHeader->m_gpmMMVDTableFlag = false;
    }

    // mvd L1 zero flag
    if (!pps->m_rplInfoInPhFlag || picHeader->m_rpl[RPL1].getNumRefEntries() > 0)
    {
      xReadFlag(uiCode, "ph_mvd_l1_zero_flag");
    }
    else
    {
      uiCode = 1;
    }
    picHeader->m_mvdL1ZeroFlag = (uiCode != 0);

    // picture level BDOF disable flags
    if (sps->m_bdofControlPresentInPhFlag && (!pps->m_rplInfoInPhFlag || picHeader->m_rpl[RPL1].getNumRefEntries() > 0))
    {
      xReadFlag(uiCode, "ph_bdof_disabled_flag");
      picHeader->m_bdofDisabledFlag = (uiCode != 0);
    }
    else
    {
      if (!sps->m_bdofControlPresentInPhFlag)
      {
        picHeader->m_bdofDisabledFlag = !sps->m_bdofEnabledFlag;
      }
      else
      {
        picHeader->m_bdofDisabledFlag = true;
      }
    }

    // picture level PROF disable flags
    if (sps->m_profControlPresentInPhFlag)
    {
      xReadFlag(uiCode, "ph_prof_disabled_flag");
      picHeader->m_profDisabledFlag = (uiCode != 0);
    }
    else
    {
      picHeader->m_profDisabledFlag = !sps->m_usePROF;
    }

    if ((pps->m_useWP || pps->m_useBiWP) && pps->m_wpInfoInPhFlag)
    {
      parsePredWeightTable(picHeader, pps, sps);
    }
  }
  // inherit constraint values from SPS
  if (!sps->m_partitionOverrideEnabled || !picHeader->m_splitConsOverrideFlag)
  {
    picHeader->setMinQTSizes(sps->m_minQT);
    picHeader->setMaxMTTHierarchyDepths(sps->m_maxMTTHierarchyDepth);
    picHeader->setMaxBTSizes(sps->m_maxBTSize);
    picHeader->setMaxTTSizes(sps->m_maxTTSize);
  }
  else
  {
    picHeader->setMinQTSizes(minQT);
    picHeader->setMaxMTTHierarchyDepths(maxBTD);
    picHeader->setMaxBTSizes(maxBTSize);
    picHeader->setMaxTTSizes(maxTTSize);
  }
  // ibc merge candidate list size
  if (pps->m_qpDeltaInfoInPhFlag)
  {
    int iCode = 0;
    xReadSvlc(iCode, "ph_qp_delta");
    picHeader->m_qpDelta = iCode;
  }

  // joint Cb/Cr sign flag
  if (sps->m_jointCbCrEnabledFlag)
  {
    xReadFlag(uiCode, "ph_joint_cbcr_sign_flag");
    picHeader->m_jointCbCrSignFlag = (uiCode != 0);
  }
  else
  {
    picHeader->m_jointCbCrSignFlag = false;
  }

#if ENABLE_NNLF
  // picture level nnlf disable flag
  if (sps->m_nnlf)
  {
    xReadFlag(uiCode, "ph_nnlf_disabled_flag");
    picHeader->m_nnlfDisabled = (uiCode != 0);
  }
  else
  {
    picHeader->m_nnlfDisabled = false;
  }
#endif

  // sao enable flags
  if (sps->m_saoEnabledFlag)
  {
    if (pps->m_saoInfoInPhFlag)
    {
      xReadFlag(uiCode, "ph_sao_luma_enabled_flag");
      picHeader->m_saoEnabledFlag[ChannelType::LUMA] = (uiCode != 0);

      if (isChromaEnabled(sps->m_chromaFormatIdc))
      {
        xReadFlag(uiCode, "ph_sao_chroma_enabled_flag");
        picHeader->m_saoEnabledFlag[ChannelType::CHROMA] = (uiCode != 0);
      }
    }
    else
    {
      picHeader->m_saoEnabledFlag[ChannelType::LUMA]   = true;
      picHeader->m_saoEnabledFlag[ChannelType::CHROMA] = isChromaEnabled(sps->m_chromaFormatIdc);
    }
  }
  else
  {
    picHeader->m_saoEnabledFlag[ChannelType::LUMA]   = false;
    picHeader->m_saoEnabledFlag[ChannelType::CHROMA] = false;
  }

  picHeader->m_ccSaoEnabledFlag[COMP_Y]  = sps->m_ccSaoEnabledFlag;
  picHeader->m_ccSaoEnabledFlag[COMP_Cb] = sps->m_ccSaoEnabledFlag;
  picHeader->m_ccSaoEnabledFlag[COMP_Cr] = sps->m_ccSaoEnabledFlag;

  if (sps->m_ccSaoEnabledFlag && pps->m_saoInfoInPhFlag)
  {
    xReadFlag(uiCode, "ph_cc_sao_y_enabled_flag");
    picHeader->m_ccSaoEnabledFlag[COMP_Y] = uiCode != 0;
    xReadFlag(uiCode, "ph_cc_sao_cb_enabled_flag");
    picHeader->m_ccSaoEnabledFlag[COMP_Cb] = uiCode != 0;
    xReadFlag(uiCode, "ph_cc_sao_cr_enabled_flag");
    picHeader->m_ccSaoEnabledFlag[COMP_Cr] = uiCode != 0;
  }

  // deblocking filter controls
  if (pps->m_deblockingFilterControlPresentFlag)
  {
    if (pps->m_dbfInfoInPhFlag)
    {
      xReadFlag(uiCode, "ph_deblocking_params_present_flag");
      picHeader->m_deblockingFilterOverrideFlag = (uiCode != 0);
    }
    else
    {
      picHeader->m_deblockingFilterOverrideFlag = false;
    }

    if (picHeader->m_deblockingFilterOverrideFlag)
    {
      if (!pps->m_ppsDeblockingFilterDisabledFlag)
      {
        xReadFlag(uiCode, "ph_deblocking_filter_disabled_flag");
        picHeader->m_deblockingFilterDisable = (uiCode != 0);
      }
      else
      {
        picHeader->m_deblockingFilterDisable = false;
      }
      if (!picHeader->m_deblockingFilterDisable)
      {
        xReadSvlc(iCode, "ph_beta_offset_div2");
        picHeader->m_deblockingFilterBetaOffsetDiv2 = iCode;
        CHECK(picHeader->m_deblockingFilterBetaOffsetDiv2 < -12 || picHeader->m_deblockingFilterBetaOffsetDiv2 > 12,
              "Invalid deblocking filter configuration");

        xReadSvlc(iCode, "ph_tc_offset_div2");
        picHeader->m_deblockingFilterTcOffsetDiv2 = iCode;
        CHECK(picHeader->m_deblockingFilterTcOffsetDiv2 < -12 || picHeader->m_deblockingFilterTcOffsetDiv2 > 12,
              "Invalid deblocking filter configuration");

        if (pps->m_usePPSChromaTool)
        {
          xReadSvlc(iCode, "ph_cb_beta_offset_div2");
          picHeader->m_deblockingFilterCbBetaOffsetDiv2 = iCode;
          CHECK(picHeader->m_deblockingFilterCbBetaOffsetDiv2 < -12 ||
                  picHeader->m_deblockingFilterCbBetaOffsetDiv2 > 12,
                "Invalid deblocking filter configuration");

          xReadSvlc(iCode, "ph_cb_tc_offset_div2");
          picHeader->m_deblockingFilterCbTcOffsetDiv2 = iCode;
          CHECK(picHeader->m_deblockingFilterCbTcOffsetDiv2 < -12 || picHeader->m_deblockingFilterCbTcOffsetDiv2 > 12,
                "Invalid deblocking filter configuration");

          xReadSvlc(iCode, "ph_cr_beta_offset_div2");
          picHeader->m_deblockingFilterCrBetaOffsetDiv2 = iCode;
          CHECK(picHeader->m_deblockingFilterCrBetaOffsetDiv2 < -12 ||
                  picHeader->m_deblockingFilterCrBetaOffsetDiv2 > 12,
                "Invalid deblocking filter configuration");

          xReadSvlc(iCode, "ph_cr_tc_offset_div2");
          picHeader->m_deblockingFilterCrTcOffsetDiv2 = iCode;
          CHECK(picHeader->m_deblockingFilterCrTcOffsetDiv2 < -12 || picHeader->m_deblockingFilterCrTcOffsetDiv2 > 12,
                "Invalid deblocking filter configuration");
        }
        else
        {
          picHeader->m_deblockingFilterCbBetaOffsetDiv2 = picHeader->m_deblockingFilterBetaOffsetDiv2;
          picHeader->m_deblockingFilterCbTcOffsetDiv2   = picHeader->m_deblockingFilterTcOffsetDiv2;
          picHeader->m_deblockingFilterCrBetaOffsetDiv2 = picHeader->m_deblockingFilterBetaOffsetDiv2;
          picHeader->m_deblockingFilterCrTcOffsetDiv2   = picHeader->m_deblockingFilterTcOffsetDiv2;
        }
      }
    }
    else
    {
      picHeader->m_deblockingFilterDisable          = pps->m_ppsDeblockingFilterDisabledFlag;
      picHeader->m_deblockingFilterBetaOffsetDiv2   = pps->m_deblockingFilterBetaOffsetDiv2;
      picHeader->m_deblockingFilterTcOffsetDiv2     = pps->m_deblockingFilterTcOffsetDiv2;
      picHeader->m_deblockingFilterCbBetaOffsetDiv2 = pps->m_deblockingFilterCbBetaOffsetDiv2;
      picHeader->m_deblockingFilterCbTcOffsetDiv2   = pps->m_deblockingFilterCbTcOffsetDiv2;
      picHeader->m_deblockingFilterCrBetaOffsetDiv2 = pps->m_deblockingFilterCrBetaOffsetDiv2;
      picHeader->m_deblockingFilterCrTcOffsetDiv2   = pps->m_deblockingFilterCrTcOffsetDiv2;
    }
  }
  else
  {
    picHeader->m_deblockingFilterDisable          = false;
    picHeader->m_deblockingFilterBetaOffsetDiv2   = 0;
    picHeader->m_deblockingFilterTcOffsetDiv2     = 0;
    picHeader->m_deblockingFilterCbBetaOffsetDiv2 = 0;
    picHeader->m_deblockingFilterCbTcOffsetDiv2   = 0;
    picHeader->m_deblockingFilterCrBetaOffsetDiv2 = 0;
    picHeader->m_deblockingFilterCrTcOffsetDiv2   = 0;
  }
  // picture header extension
  if (pps->m_pictureHeaderExtensionPresentFlag)
  {
    xReadUvlc(uiCode, "ph_extension_length");
    for (int i = 0; i < uiCode; i++)
    {
      uint32_t ignore_;
      xReadCode(8, ignore_, "ph_extension_data_byte");
    }
  }

  if (readRbspTrailingBits)
  {
    xReadRbspTrailingBits();
  }
}

void HLSyntaxReader::checkAlfNaluTidAndPicTid(Slice *pcSlice, PicHeader *picHeader,
                                              ParameterSetManager *parameterSetManager)
{
  SPS                             *sps       = parameterSetManager->getSPS(picHeader->m_spsId);
  PPS                             *pps       = parameterSetManager->getPPS(picHeader->m_ppsId);
  VPS                             *vps       = parameterSetManager->getVPS(sps->m_vpsId);
  int                              curPicTid = pcSlice->m_uiTLayer;
  APS                             *aps;
  const AlfParameters::AlfApsList &apsId = picHeader->m_alfApsIdsLuma;

  if (sps->m_alfEnabledFlag && pps->m_alfInfoInPhFlag && picHeader->m_alfEnabledFlag[COMP_Y])
  {
    // luma
    for (int i = 0; i < picHeader->m_numAlfApsIdsLuma; i++)
    {
      aps = parameterSetManager->getAPS(apsId[i], ApsType::ALF);
      CHECK(aps->m_temporalId > curPicTid,
            "The TemporalId of the APS NAL unit having aps_params_type equal to ApsType::ALF and "
            "adaptation_parameter_set_id equal to ph_alf_aps_id_luma[ i ] shall be less than or equal to the "
            "TemporalId of the picture associated with the PH.");
      if (pcSlice->m_nuhLayerId != aps->m_layerId)
      {
        CHECK(aps->m_layerId > pcSlice->m_nuhLayerId,
              "Layer Id of APS cannot be greater than layer Id of VCL NAL unit the refer to it");
        CHECK(sps->m_vpsId == 0,
              "VPSId of the referred SPS cannot be 0 when layer Id of APS and layer Id of current slice are different");
        for (int i = 0; i < vps->m_vpsNumOutputLayerSets; i++)
        {
          bool isCurrLayerInOls = false;
          bool isRefLayerInOls  = false;
          for (int j = vps->m_numLayersInOls[i] - 1; j >= 0; j--)
          {
            if (vps->m_layerIdInOls[i][j] == pcSlice->m_nuhLayerId)
            {
              isCurrLayerInOls = true;
            }
            if (vps->m_layerIdInOls[i][j] == aps->m_layerId)
            {
              isRefLayerInOls = true;
            }
          }
          CHECK(isCurrLayerInOls && !isRefLayerInOls,
                "When VCL NAl unit in layer A refers to APS in layer B, all OLS that contains layer A shall also "
                "contains layer B");
        }
      }
    }
    // chroma
    if (picHeader->m_alfEnabledFlag[COMP_Cb] || picHeader->m_alfEnabledFlag[COMP_Cr])
    {
      int chromaAlfApsId = picHeader->m_alfApsIdChroma;
      aps                = parameterSetManager->getAPS(chromaAlfApsId, ApsType::ALF);
      CHECK(aps->m_temporalId > curPicTid,
            "The TemporalId of the APS NAL unit having aps_params_type equal to ApsType::ALF and "
            "adaptation_parameter_set_id equal to ph_alf_aps_id_chroma shall be less than or equal to the TemporalId "
            "of the picture associated with the PH.");
      if (pcSlice->m_nuhLayerId != aps->m_layerId)
      {
        CHECK(aps->m_layerId > pcSlice->m_nuhLayerId,
              "Layer Id of APS cannot be greater than layer Id of VCL NAL unit the refer to it");
        CHECK(sps->m_vpsId == 0,
              "VPSId of the referred SPS cannot be 0 when layer Id of APS and layer Id of current slice are different");
        for (int i = 0; i < vps->m_vpsNumOutputLayerSets; i++)
        {
          bool isCurrLayerInOls = false;
          bool isRefLayerInOls  = false;
          for (int j = vps->m_numLayersInOls[i] - 1; j >= 0; j--)
          {
            if (vps->m_layerIdInOls[i][j] == pcSlice->m_nuhLayerId)
            {
              isCurrLayerInOls = true;
            }
            if (vps->m_layerIdInOls[i][j] == aps->m_layerId)
            {
              isRefLayerInOls = true;
            }
          }
          CHECK(isCurrLayerInOls && !isRefLayerInOls,
                "When VCL NAl unit in layer A refers to APS in layer B, all OLS that contains layer A shall also "
                "contains layer B");
        }
      }
    }
  }
}

void HLSyntaxReader::parseSliceHeader(Slice *pcSlice, PicHeader *picHeader, ParameterSetManager *parameterSetManager,
                                      const int prevTid0POC, const int prevPicPOC)
{
  uint32_t uiCode;
  int      iCode;

#if ENABLE_TRACING
  xTraceSliceHeader();
#endif
  PPS *pps = nullptr;
  SPS *sps = nullptr;
  xReadFlag(uiCode, "sh_picture_header_in_slice_header_flag");
  pcSlice->m_pictureHeaderInSliceHeader = uiCode;
  if (uiCode)
  {
    parsePictureHeader(picHeader, parameterSetManager, false);
    picHeader->m_valid = true;
  }
  CHECK(picHeader == 0, "Invalid Picture Header");
  CHECK(picHeader->m_valid == false, "Invalid Picture Header");
  checkAlfNaluTidAndPicTid(pcSlice, picHeader, parameterSetManager);
  pps = parameterSetManager->getPPS(picHeader->m_ppsId);
  //! KS: need to add error handling code here, if PPS is not available
  CHECK(pps == 0, "Invalid PPS");
  sps = parameterSetManager->getSPS(pps->m_spsId);
  //! KS: need to add error handling code here, if SPS is not available
  CHECK(sps == 0, "Invalid SPS");
  if (sps->m_profileTierLevel.m_constraintInfo.m_picHeaderInSliceHeaderConstraintFlag)
  {
    CHECK(pcSlice->m_pictureHeaderInSliceHeader == false,
          "PH shall be present in SH, when pic_header_in_slice_header_constraint_flag is equal to 1");
  }
  CHECK(pcSlice->m_pictureHeaderInSliceHeader && pps->m_rplInfoInPhFlag == 1,
        "When sh_picture_header_in_slice_header_flag is equal to 1, rpl_info_in_ph_flag shall be equal to 0");
  CHECK(pcSlice->m_pictureHeaderInSliceHeader && pps->m_dbfInfoInPhFlag == 1,
        "When sh_picture_header_in_slice_header_flag is equal to 1, dbf_info_in_ph_flag shall be equal to 0");
  CHECK(pcSlice->m_pictureHeaderInSliceHeader && pps->m_saoInfoInPhFlag == 1,
        "When sh_picture_header_in_slice_header_flag is equal to 1, sao_info_in_ph_flag shall be equal to 0");
  CHECK(pcSlice->m_pictureHeaderInSliceHeader && pps->m_alfInfoInPhFlag == 1,
        "When sh_picture_header_in_slice_header_flag is equal to 1, alf_info_in_ph_flag shall be equal to 0");
  CHECK(pcSlice->m_pictureHeaderInSliceHeader && pps->m_wpInfoInPhFlag == 1,
        "When sh_picture_header_in_slice_header_flag is equal to 1, wp_info_in_ph_flag shall be equal to 0");
  CHECK(pcSlice->m_pictureHeaderInSliceHeader && pps->m_qpDeltaInfoInPhFlag == 1,
        "When sh_picture_header_in_slice_header_flag is equal to 1, qp_delta_info_in_ph_flag shall be equal to 0");
  CHECK(pcSlice->m_pictureHeaderInSliceHeader && sps->m_subPicInfoPresentFlag == 1,
        "When sps_subpic_info_present_flag is equal to 1, the value of sh_picture_header_in_slice_header_flag shall be "
        "equal to 0");

  const ChromaFormat chFmt        = sps->m_chromaFormatIdc;
  const uint32_t     numValidComp = getNumberValidComponents(chFmt);
  const bool         hasChroma    = isChromaEnabled(chFmt);

  // picture order count
  uiCode         = picHeader->m_pocLsb;
  int iPOClsb    = uiCode;
  int iMaxPOClsb = 1 << sps->m_bitsForPoc;
  int iPOCmsb;
  if (pcSlice->getIdrPicFlag())
  {
    if (picHeader->m_pocMsbPresentFlag)
    {
      iPOCmsb = picHeader->m_pocMsbVal * iMaxPOClsb;
    }
    else
    {
      iPOCmsb = 0;
    }
    pcSlice->m_poc = iPOCmsb + iPOClsb;
  }
  else
  {
    int iPrevPOC    = prevTid0POC;
    int iPrevPOClsb = iPrevPOC & (iMaxPOClsb - 1);
    int iPrevPOCmsb = iPrevPOC - iPrevPOClsb;
    if (picHeader->m_pocMsbPresentFlag)
    {
      iPOCmsb = picHeader->m_pocMsbVal * iMaxPOClsb;
    }
    else
    {
      if ((iPOClsb < iPrevPOClsb) && ((iPrevPOClsb - iPOClsb) >= (iMaxPOClsb / 2)))
      {
        iPOCmsb = iPrevPOCmsb + iMaxPOClsb;
      }
      else if ((iPOClsb > iPrevPOClsb) && ((iPOClsb - iPrevPOClsb) > (iMaxPOClsb / 2)))
      {
        iPOCmsb = iPrevPOCmsb - iMaxPOClsb;
      }
      else
      {
        iPOCmsb = iPrevPOCmsb;
      }
    }
    pcSlice->m_poc = iPOCmsb + iPOClsb;
  }

  if (sps->m_subPicInfoPresentFlag)
  {
    uint32_t bitsSubPicId;
    bitsSubPicId = sps->m_subPicIdLen;
    xReadCode(bitsSubPicId, uiCode, "sh_subpic_id");
    pcSlice->m_sliceSubPicId = uiCode;
  }
  else
  {
    pcSlice->m_sliceSubPicId = 0;
  }

  // raster scan slices
  uint32_t sliceAddr = 0;
  if (pps->m_rectSliceFlag == 0)
  {
    // slice address is the raster scan tile index of first tile in slice
    if (pps->getNumTiles() > 1)
    {
      int bitsSliceAddress = ceilLog2(pps->getNumTiles());
      xReadCode(bitsSliceAddress, uiCode, "sh_slice_address");
      sliceAddr = uiCode;
    }
  }
  // rectangular slices
  else
  {
    // slice address is the index of the slice within the current sub-picture
    uint32_t currSubPicIdx = pps->getSubPicIdxFromSubPicId(pcSlice->m_sliceSubPicId);
    SubPic   currSubPic    = pps->m_subPics[currSubPicIdx];
    if (currSubPic.m_numSlicesInSubPic > 1)
    {
      int bitsSliceAddress = ceilLog2(currSubPic.m_numSlicesInSubPic);
      xReadCode(bitsSliceAddress, uiCode, "sh_slice_address");
      sliceAddr = uiCode;
      CHECK(sliceAddr >= currSubPic.m_numSlicesInSubPic, "Invalid slice address");
    }
    uint32_t picLevelSliceIdx = sliceAddr;
    for (int subpic = 0; subpic < currSubPicIdx; subpic++)
    {
      picLevelSliceIdx += pps->m_subPics[subpic].m_numSlicesInSubPic;
    }
    pcSlice->setSliceMap(pps->getSliceMap(picLevelSliceIdx));
    pcSlice->setSliceID(picLevelSliceIdx);
  }

  std::vector<bool> shExtraBitsPresent = sps->m_extraSHBitPresentFlag;
  for (int i = 0; i < sps->m_numExtraSHBytes * 8; i++)
  {
    // extra bits are ignored (when present)
    if (shExtraBitsPresent[i])
    {
      xReadFlag(uiCode, "sh_extra_bit[ i ]");
    }
  }

  if (pps->m_rectSliceFlag == 0)
  {
    uint32_t numTilesInSlice = 1;
    if (pps->getNumTiles() > 1)
    {
      if (((int)pps->getNumTiles() - (int)sliceAddr) > 1)
      {
        xReadUvlc(uiCode, "sh_num_tiles_in_slice_minus1");
        numTilesInSlice = uiCode + 1;
      }
      if (!pps->m_rectSliceFlag && sps->m_profileTierLevel.m_constraintInfo.m_oneSlicePerPicConstraintFlag)
      {
        CHECK(pps->getNumTiles() != uiCode + 1,
              "When pps_rect_slice_flag is equal to 0 and one_slice_per_pic_constraint_flag equal to 1, the value of "
              "sh_num_tiles_in_slice_minus1 present in each slice header shall be equal to NumTilesInPic - 1");
      }
    }
    CHECK(sliceAddr >= pps->getNumTiles(), "Invalid slice address");
    pcSlice->m_sliceMap.initSliceMap();
    pcSlice->setSliceID(sliceAddr);

    for (uint32_t tileIdx = sliceAddr; tileIdx < sliceAddr + numTilesInSlice; tileIdx++)
    {
      uint32_t tileX = tileIdx % pps->m_numTileCols;
      uint32_t tileY = tileIdx / pps->m_numTileCols;
      CHECK(tileY >= pps->m_numTileRows, "Number of tiles in slice exceeds the remaining number of tiles in picture");

      pcSlice->addCtusToSlice(pps->getTileColumnBd(tileX), pps->getTileColumnBd(tileX + 1), pps->getTileRowBd(tileY),
                              pps->getTileRowBd(tileY + 1), pps->m_picWidthInCtu);
    }
  }

  if (picHeader->m_picInterSliceAllowedFlag)
  {
    xReadUvlc(uiCode, "sh_slice_type");
    pcSlice->m_eSliceType = (SliceType)uiCode;
#if ENABLE_CABAC_DUMP
    pcSlice->m_cabacInitSliceType = (SliceType)uiCode;
#endif
    VPS *vps = parameterSetManager->getVPS(sps->m_vpsId);
    if (pcSlice->isIRAP() &&
        (sps->m_vpsId == 0 || pcSlice->m_poc != prevPicPOC ||
         vps->m_vpsIndependentLayerFlag[vps->m_generalLayerIdx[pcSlice->m_nuhLayerId]] == 1))
    {
      CHECK(uiCode != 2,
            "When nal_unit_type is in the range of IDR_W_RADL to CRA_NUT, inclusive, and vps_independent_layer_flag[ "
            "GeneralLayerIdx[ nuh_layer_id ] ] is equal to 1 or the current picture is the first picture in the "
            "current AU, sh_slice_type shall be equal to 2");
    }
  }
  else
  {
    pcSlice->m_eSliceType = I_SLICE;
#if ENABLE_CABAC_DUMP
    pcSlice->m_cabacInitSliceType = I_SLICE;
#endif
  }
  if (!picHeader->m_picIntraSliceAllowedFlag)
  {
    CHECK(pcSlice->m_eSliceType == I_SLICE, "when ph_intra_slice_allowed_flag = 0, no I_Slice is allowed");
  }
  if (pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA || pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
      pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL || pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_GDR)
  {
    xReadFlag(uiCode, "sh_no_output_of_prior_pics_flag");
    pcSlice->m_noOutputOfPriorPicsFlag = (uiCode != 0);
  }

  if (sps->m_lfCccmEnabledFlag && !pcSlice->isIntra())
  {
    xReadFlag(uiCode, "sh_lfcccm_enabled_flag");
    pcSlice->m_lfCccmEnabledFlag = uiCode;
  }

  // inherit values from picture header
  //   set default values in case slice overrides are disabled
  pcSlice->inheritFromPicHeader(picHeader, pps, sps);

  if (sps->m_alfEnabledFlag && !pps->m_alfInfoInPhFlag)
  {
    xReadFlag(uiCode, "sh_alf_enabled_flag");
    const bool alfEnabledFlag         = uiCode != 0;
    pcSlice->m_alfEnabledFlag[COMP_Y] = alfEnabledFlag;

    bool alfCbEnabledFlag = false;
    bool alfCrEnabledFlag = false;

    AlfParameters::AlfApsList apsIds;
    if (alfEnabledFlag)
    {
      if (sps->m_alfImprovementsEnabledFlag)
      {
        xReadFlag(uiCode, "slice_alf_fixed_filter_set_cand_idx_luma");
        pcSlice->m_newAlfFixFiltSetCandIdx[COMP_Y] = uiCode;
      }

      xReadCode(3, uiCode, "sh_num_alf_aps_ids_luma");
      const int numAps = uiCode;

      for (int i = 0; i < numAps; i++)
      {
        xReadCode(3, uiCode, "sh_alf_aps_id_luma[i]");
        const int apsId = uiCode;

        apsIds.push_back(apsId);

        APS *apsToCheckLuma = parameterSetManager->getAPS(apsId, ApsType::ALF);
        CHECK(apsToCheckLuma == nullptr, "referenced APS not found");
        CHECK(apsToCheckLuma->m_alfAPSParam.getParam().newFilterFlag[ChannelType::LUMA] != 1,
              "bitstream conformance error, alf_luma_filter_signal_flag shall be equal to 1");
      }

      if (hasChroma)
      {
        xReadCode(1, uiCode, "sh_alf_cb_enabled_flag");
        alfCbEnabledFlag = uiCode != 0;
        xReadCode(1, uiCode, "sh_alf_cr_enabled_flag");
        alfCrEnabledFlag = uiCode != 0;
        if (sps->m_alfImprovementsEnabledFlag)
        {
          if (alfCbEnabledFlag)
          {
            xReadFlag(uiCode, "slice_alf_fixed_filter_set_cand_idx_cb");
            pcSlice->m_newAlfFixFiltSetCandIdx[COMP_Cb] = uiCode;
          }
          if (alfCrEnabledFlag)
          {
            xReadFlag(uiCode, "slice_alf_fixed_filter_set_cand_idx_cr");
            pcSlice->m_newAlfFixFiltSetCandIdx[COMP_Cr] = uiCode;
          }
        }
      }

      if (alfCbEnabledFlag || alfCrEnabledFlag)
      {
        xReadCode(3, uiCode, "sh_alf_aps_id_chroma");
        pcSlice->m_alfApsIdChroma = uiCode;
        APS *apsToCheckChroma     = parameterSetManager->getAPS(uiCode, ApsType::ALF);
        CHECK(apsToCheckChroma == nullptr, "referenced APS not found");
        CHECK(apsToCheckChroma->m_alfAPSParam.getParam().newFilterFlag[ChannelType::CHROMA] != 1,
              "bitstream conformance error, alf_chroma_filter_signal_flag shall be equal to 1");
      }
    }

    pcSlice->m_numAlfApsIdsLuma        = (int)apsIds.size();
    pcSlice->m_alfApsIdsLuma           = apsIds;
    pcSlice->m_alfEnabledFlag[COMP_Cb] = alfCbEnabledFlag;
    pcSlice->m_alfEnabledFlag[COMP_Cr] = alfCrEnabledFlag;

    AlfParameters::CcAlfFilterParamBase &filterParam = pcSlice->m_ccAlfFilterParam.getParam();
    if (sps->m_ccalfEnabledFlag && pcSlice->m_alfEnabledFlag[COMP_Y])
    {
      xReadFlag(uiCode, "sh_alf_cc_cb_enabled_flag");
      pcSlice->m_ccAlfCbEnabledFlag               = uiCode;
      filterParam.ccAlfFilterEnabled[COMP_Cb - 1] = (uiCode == 1) ? true : false;
      pcSlice->m_ccAlfCbApsId                     = -1;
      if (filterParam.ccAlfFilterEnabled[COMP_Cb - 1])
      {
        // parse APS ID
        xReadCode(3, uiCode, "sh_alf_cc_cb_aps_id");
        pcSlice->m_ccAlfCbApsId = uiCode;
        APS *apsToCheckCcCb     = parameterSetManager->getAPS(uiCode, ApsType::ALF);
        CHECK(apsToCheckCcCb == nullptr, "referenced APS not found");
        CHECK(apsToCheckCcCb->m_ccAlfAPSParam.getParam().newCcAlfFilter[COMP_Cb - 1] != 1,
              "bitstream conformance error, alf_cc_cb_filter_signal_flag shall be equal to 1");
      }
      // Cr
      xReadFlag(uiCode, "sh_alf_cc_cr_enabled_flag");
      pcSlice->m_ccAlfCrEnabledFlag               = uiCode;
      filterParam.ccAlfFilterEnabled[COMP_Cr - 1] = (uiCode == 1) ? true : false;
      pcSlice->m_ccAlfCrApsId                     = -1;
      if (filterParam.ccAlfFilterEnabled[COMP_Cr - 1])
      {
        // parse APS ID
        xReadCode(3, uiCode, "sh_alf_cc_cr_aps_id");
        pcSlice->m_ccAlfCrApsId = uiCode;
        APS *apsToCheckCcCr     = parameterSetManager->getAPS(uiCode, ApsType::ALF);
        CHECK(apsToCheckCcCr == nullptr, "referenced APS not found");
        CHECK(apsToCheckCcCr->m_ccAlfAPSParam.getParam().newCcAlfFilter[COMP_Cr - 1] != 1,
              "bitstream conformance error, alf_cc_cr_filter_signal_flag shall be equal to 1");
      }
    }
    else
    {
      filterParam.ccAlfFilterEnabled[COMP_Cb - 1] = false;
      filterParam.ccAlfFilterEnabled[COMP_Cr - 1] = false;
      pcSlice->m_ccAlfCbApsId                     = -1;
      pcSlice->m_ccAlfCrApsId                     = -1;
    }
  }
  if (picHeader->m_lmcsEnabledFlag && !pcSlice->m_pictureHeaderInSliceHeader)
  {
    xReadFlag(uiCode, "sh_lmcs_used_flag");
    pcSlice->m_lmcsEnabledFlag = uiCode;
  }
  else
  {
    pcSlice->m_lmcsEnabledFlag = pcSlice->m_pictureHeaderInSliceHeader ? picHeader->m_lmcsEnabledFlag : false;
  }
  if (picHeader->m_explicitScalingListEnabledFlag && !pcSlice->m_pictureHeaderInSliceHeader)
  {
    xReadFlag(uiCode, "sh_explicit_scaling_list_used_flag");
    pcSlice->m_explicitScalingListUsed = uiCode;
  }
  else
  {
    pcSlice->m_explicitScalingListUsed =
      pcSlice->m_pictureHeaderInSliceHeader ? picHeader->m_explicitScalingListEnabledFlag : false;
  }

  if (pps->m_rplInfoInPhFlag)
  {
    pcSlice->m_rpl[RPL0] = picHeader->m_rpl[RPL0];
    pcSlice->m_rpl[RPL1] = picHeader->m_rpl[RPL1];
  }
  else if (pcSlice->getIdrPicFlag() && !(sps->m_idrRefParamList))
  {
    ReferencePictureList *rpl0 = &pcSlice->m_rpl[RPL0];
    (*rpl0)                    = ReferencePictureList();
    ReferencePictureList *rpl1 = &pcSlice->m_rpl[RPL1];
    (*rpl1)                    = ReferencePictureList();
  }
  else
  {
    // Read L0 related syntax elements
    bool rplSpsFlag0 = false;

    if (sps->m_numRpl[RPL0] > 0)
    {
      xReadFlag(uiCode, "ref_pic_list_sps_flag[0]");
      rplSpsFlag0 = uiCode != 0;
    }

    auto const rpl0 = &pcSlice->m_rpl[RPL0];
    if (!rplSpsFlag0)   // explicitly carried in this SH
    {
      *rpl0 = ReferencePictureList();
      parseRefPicList(sps, rpl0, -1);
      pcSlice->m_rplIdx[RPL0] = -1;
    }
    else   // Refer to list in SPS
    {
      int rpsIdx = 0;
      if (sps->m_numRpl[RPL0] > 1)
      {
        int numBits = ceilLog2(sps->m_numRpl[RPL0]);
        xReadCode(numBits, uiCode, "ref_pic_list_idx[0]");
        rpsIdx = uiCode;
      }

      pcSlice->m_rplIdx[RPL0] = rpsIdx;
      *rpl0                   = *sps->m_rplList[RPL0].getReferencePictureList(rpsIdx);
    }
    // Deal POC Msb cycle signalling for LTRP
    for (int i = 0; i < rpl0->getNumRefEntries(); i++)
    {
      rpl0->m_deltaPocMSBPresentFlag[i] = false;
      rpl0->m_deltaPOCMSBCycleLT[i]     = 0;
    }
    if (rpl0->m_numberOfLongtermPictures)
    {
      for (int i = 0; i < rpl0->getNumRefEntries(); i++)
      {
        if (rpl0->m_isLongtermRefPic[i])
        {
          if (rpl0->m_ltrpInSliceHeaderFlag)
          {
            xReadCode(sps->m_bitsForPoc, uiCode, "slice_poc_lsb_lt[i][j]");
            rpl0->setRefPicIdentifier(i, uiCode, true, false, 0);
          }
          xReadFlag(uiCode, "delta_poc_msb_present_flag[i][j]");
          rpl0->m_deltaPocMSBPresentFlag[i] = uiCode ? true : false;
          if (uiCode)
          {
            xReadUvlc(uiCode, "slice_delta_poc_msb_cycle_lt[i][j]");
            if (i != 0)
            {
              uiCode += rpl0->m_deltaPOCMSBCycleLT[i - 1];
            }
            rpl0->m_deltaPOCMSBCycleLT[i] = uiCode;
          }
          else if (i != 0)
          {
            rpl0->m_deltaPOCMSBCycleLT[i] = rpl0->m_deltaPOCMSBCycleLT[i - 1];
          }
          else
          {
            rpl0->m_deltaPOCMSBCycleLT[i] = 0;
          }
        }
        else if (i != 0)
        {
          rpl0->m_deltaPOCMSBCycleLT[i] = rpl0->m_deltaPOCMSBCycleLT[i - 1];
        }
        else
        {
          rpl0->m_deltaPOCMSBCycleLT[i] = 0;
        }
      }
    }

    // Read L1 related syntax elements
    bool rplSpsFlag1 = sps->m_numRpl[RPL1] == 0 ? false : rplSpsFlag0;
    if (sps->m_numRpl[RPL1] > 0 && pps->m_rpl1IdxPresentFlag)
    {
      xReadFlag(uiCode, "ref_pic_list_sps_flag[1]");
      rplSpsFlag1 = uiCode != 0;
    }

    auto const rpl1 = &pcSlice->m_rpl[RPL1];
    if (rplSpsFlag1)
    {
      if (sps->m_numRpl[RPL1] > 1 && pps->m_rpl1IdxPresentFlag)
      {
        int numBits = ceilLog2(sps->m_numRpl[RPL1]);
        xReadCode(numBits, uiCode, "ref_pic_list_idx[1]");
        pcSlice->m_rplIdx[RPL1] = uiCode;
        *rpl1                   = *sps->m_rplList[RPL1].getReferencePictureList(uiCode);
      }
      else if (sps->m_numRpl[RPL1] == 1)
      {
        pcSlice->m_rplIdx[RPL1] = 0;
        *rpl1                   = *sps->m_rplList[RPL1].getReferencePictureList(0);
      }
      else
      {
        assert(pcSlice->m_rplIdx[RPL0] != -1);
        pcSlice->m_rplIdx[RPL1] = pcSlice->m_rplIdx[RPL0];
        *rpl1                   = *sps->m_rplList[RPL1].getReferencePictureList(pcSlice->m_rplIdx[RPL0]);
      }
    }
    else
    {
      (*rpl1) = ReferencePictureList();
      parseRefPicList(sps, rpl1, -1);
      pcSlice->m_rplIdx[RPL1] = -1;
    }

    // Deal POC Msb cycle signalling for LTRP
    for (int i = 0; i < rpl1->getNumRefEntries(); i++)
    {
      rpl1->m_deltaPocMSBPresentFlag[i] = false;
      rpl1->m_deltaPOCMSBCycleLT[i]     = 0;
    }
    if (rpl1->m_numberOfLongtermPictures)
    {
      for (int i = 0; i < rpl1->getNumRefEntries(); i++)
      {
        if (rpl1->m_isLongtermRefPic[i])
        {
          if (rpl1->m_ltrpInSliceHeaderFlag)
          {
            xReadCode(sps->m_bitsForPoc, uiCode, "slice_poc_lsb_lt[i][j]");
            rpl1->setRefPicIdentifier(i, uiCode, true, false, 0);
          }
          xReadFlag(uiCode, "delta_poc_msb_present_flag[i][j]");
          rpl1->m_deltaPocMSBPresentFlag[i] = uiCode ? true : false;
          if (uiCode)
          {
            xReadUvlc(uiCode, "slice_delta_poc_msb_cycle_lt[i][j]");
            if (i != 0)
            {
              uiCode += rpl1->m_deltaPOCMSBCycleLT[i - 1];
            }
            rpl1->m_deltaPOCMSBCycleLT[i] = uiCode;
          }
          else if (i != 0)
          {
            rpl1->m_deltaPOCMSBCycleLT[i] = rpl1->m_deltaPOCMSBCycleLT[i - 1];
          }
          else
          {
            rpl1->m_deltaPOCMSBCycleLT[i] = 0;
          }
        }
        else if (i != 0)
        {
          rpl1->m_deltaPOCMSBCycleLT[i] = rpl1->m_deltaPOCMSBCycleLT[i - 1];
        }
        else
        {
          rpl1->m_deltaPOCMSBCycleLT[i] = 0;
        }
      }
    }
  }

  uint32_t numActiveRefs[NUM_RPL01] = { pcSlice->isIntra() ? 0u : 1u, pcSlice->isInterB() ? 1u : 0u };

  if ((!pcSlice->isIntra() && pcSlice->m_rpl[RPL0].getNumRefEntries() > 1) ||
      (pcSlice->isInterB() && pcSlice->m_rpl[RPL1].getNumRefEntries() > 1))
  {
    xReadFlag(uiCode, "sh_num_ref_idx_active_override_flag");
    if (uiCode)
    {
      if (pcSlice->m_rpl[RPL0].getNumRefEntries() > 1)
      {
        xReadUvlc(uiCode, "sh_num_ref_idx_active_minus1[0]");
        CHECK(uiCode >= MAX_NUM_ACTIVE_REF,
              "The value of sh_num_ref_idx_active_minus1[0] shall be in the range of 0 to 14, inclusive");
        numActiveRefs[RPL0] = uiCode + 1;
      }
      if (pcSlice->isInterB() && pcSlice->m_rpl[RPL1].getNumRefEntries() > 1)
      {
        xReadUvlc(uiCode, "sh_num_ref_idx_active_minus1[1]");
        CHECK(uiCode >= MAX_NUM_ACTIVE_REF,
              "The value of sh_num_ref_idx_active_minus1[1] shall be in the range of 0 to 14, inclusive");
        numActiveRefs[RPL1] = uiCode + 1;
      }
    }
    else
    {
      numActiveRefs[RPL0] = std::min<int>(pcSlice->m_rpl[RPL0].getNumRefEntries(), pps->m_numRefIdxDefaultActive[RPL0]);

      if (pcSlice->isInterB())
      {
        numActiveRefs[RPL1] =
          std::min<int>(pcSlice->m_rpl[RPL1].getNumRefEntries(), pps->m_numRefIdxDefaultActive[RPL1]);
      }
    }
  }

  pcSlice->m_numRefIdx[RPL0] = numActiveRefs[RPL0];
  pcSlice->m_numRefIdx[RPL1] = numActiveRefs[RPL1];

  if (pcSlice->isInterP() || pcSlice->isInterB())
  {
    CHECK(pcSlice->m_numRefIdx[RPL0] == 0,
          "Number of active entries in RPL0 of P or B picture shall be greater than 0");

    if (pcSlice->isInterB())
    {
      CHECK(pcSlice->m_numRefIdx[RPL1] == 0, "Number of active entries in RPL1 of B picture shall be greater than 0");
    }
  }

  pcSlice->m_cabacInitFlag = false;   // default
  if (pps->m_cabacInitPresentFlag && !pcSlice->isIntra())
  {
    xReadFlag(uiCode, "sh_cabac_init_flag");
    pcSlice->m_cabacInitFlag = (uiCode ? true : false);
    pcSlice->m_encCABACTableIdx =
      pcSlice->m_eSliceType == B_SLICE ? (uiCode ? P_SLICE : B_SLICE) : (uiCode ? B_SLICE : P_SLICE);
  }

  if (picHeader->m_enableTMVPFlag)
  {
    if (pcSlice->m_eSliceType == P_SLICE)
    {
      pcSlice->m_colFromL0Flag = true;
    }
    else if (!pps->m_rplInfoInPhFlag && pcSlice->m_eSliceType == B_SLICE)
    {
      xReadFlag(uiCode, "sh_collocated_from_l0_flag");
      pcSlice->m_colFromL0Flag = uiCode;
    }
    else
    {
      pcSlice->m_colFromL0Flag = picHeader->m_picColFromL0Flag;
    }

    if (!pps->m_rplInfoInPhFlag)
    {
      if (pcSlice->m_eSliceType != I_SLICE &&
          ((pcSlice->m_colFromL0Flag == 1 && pcSlice->m_numRefIdx[RPL0] > 1) ||
           (pcSlice->m_colFromL0Flag == 0 && pcSlice->m_numRefIdx[RPL1] > 1)))
      {
        xReadUvlc(uiCode, "sh_collocated_ref_idx");
        pcSlice->m_colRefIdx = uiCode;
      }
      else
      {
        pcSlice->m_colRefIdx = 0;
      }
    }
    else
    {
      pcSlice->m_colRefIdx = picHeader->m_colRefIdx;
    }
  }
  if ((pps->m_useWP && pcSlice->m_eSliceType == P_SLICE) || (pps->m_useBiWP && pcSlice->m_eSliceType == B_SLICE))
  {
    if (pps->m_wpInfoInPhFlag)
    {
      CHECK(pcSlice->m_numRefIdx[RPL0] > picHeader->m_numWeights[RPL0],
            "ERROR: Number of active reference picture L0 is greater than the number of weighted prediction signalled "
            "in Picture Header");
      CHECK(pcSlice->m_numRefIdx[RPL1] > picHeader->m_numWeights[RPL1],
            "ERROR: Number of active reference picture L1 is greater than the number of weighted prediction signalled "
            "in Picture Header");
      pcSlice->setWpScaling(picHeader->m_weightPredTable);
    }
    else
    {
      parsePredWeightTable(pcSlice, sps);
    }
    pcSlice->initWpScaling(sps);
  }
  else
  {
    for (int iNumRef = 0; iNumRef < ((pcSlice->m_eSliceType == B_SLICE) ? 2 : 1); iNumRef++)
    {
      RefPicList eRefPicList = (iNumRef ? RPL1 : RPL0);
      for (int refIdx = 0; refIdx < pcSlice->m_numRefIdx[eRefPicList]; refIdx++)
      {
        WPScalingParam *wp = pcSlice->getWpScaling(eRefPicList, refIdx);

        wp[0].presentFlag = false;
        wp[1].presentFlag = false;
        wp[2].presentFlag = false;
      }
    }
  }

  if (pcSlice->isIntra())
  {
    pcSlice->m_ibcFlag = sps->m_ibcFlag;
  }
  else
  {
    pcSlice->m_ibcFlag = sps->m_ibcFlagInterSlice;
  }

  int qpDelta = 0;
  if (pps->m_qpDeltaInfoInPhFlag)
  {
    qpDelta = picHeader->m_qpDelta;
  }
  else
  {
    xReadSvlc(iCode, "sh_qp_delta");
    qpDelta = iCode;
    if (sps->m_useInterRPL)
    {
      int idx = pcSlice->m_rplIdx[RPL0];
      if (!pcSlice->isIntra() && idx >= 0 && idx < sps->m_QPoffsetRPL.size())
      {
        qpDelta = qpDelta + sps->m_QPoffsetRPL[idx];
      }
    }
  }
  pcSlice->m_iSliceQp     = 26 + pps->m_picInitQPMinus26 + qpDelta;
  pcSlice->m_iSliceQpBase = pcSlice->m_iSliceQp;

  CHECK(pcSlice->m_iSliceQp < -sps->m_qpBDOffset[ChannelType::LUMA], "Invalid slice QP delta");
  CHECK(pcSlice->m_iSliceQp > MAX_QP, "Invalid slice QP");

  if (pps->m_sliceChromaQpFlag)
  {
    if (numValidComp > COMP_Cb)
    {
      xReadSvlc(iCode, "sh_cb_qp_offset");
      pcSlice->setSliceChromaQpDelta(COMP_Cb, iCode);
      CHECK(pcSlice->getSliceChromaQpDelta(COMP_Cb) < -12, "Invalid chroma QP offset");
      CHECK(pcSlice->getSliceChromaQpDelta(COMP_Cb) > 12, "Invalid chroma QP offset");
      CHECK((pps->getQpOffset(COMP_Cb) + pcSlice->getSliceChromaQpDelta(COMP_Cb)) < -12, "Invalid chroma QP offset");
      CHECK((pps->getQpOffset(COMP_Cb) + pcSlice->getSliceChromaQpDelta(COMP_Cb)) > 12, "Invalid chroma QP offset");
    }

    if (numValidComp > COMP_Cr)
    {
      xReadSvlc(iCode, "sh_cr_qp_offset");
      pcSlice->setSliceChromaQpDelta(COMP_Cr, iCode);
      CHECK(pcSlice->getSliceChromaQpDelta(COMP_Cr) < -12, "Invalid chroma QP offset");
      CHECK(pcSlice->getSliceChromaQpDelta(COMP_Cr) > 12, "Invalid chroma QP offset");
      CHECK((pps->getQpOffset(COMP_Cr) + pcSlice->getSliceChromaQpDelta(COMP_Cr)) < -12, "Invalid chroma QP offset");
      CHECK((pps->getQpOffset(COMP_Cr) + pcSlice->getSliceChromaQpDelta(COMP_Cr)) > 12, "Invalid chroma QP offset");
      if (sps->m_jointCbCrEnabledFlag)
      {
        xReadSvlc(iCode, "sh_joint_cbcr_qp_offset");
        pcSlice->setSliceChromaQpDelta(JOINT_CbCr, iCode);
        CHECK(pcSlice->getSliceChromaQpDelta(JOINT_CbCr) < -12, "Invalid chroma QP offset");
        CHECK(pcSlice->getSliceChromaQpDelta(JOINT_CbCr) > 12, "Invalid chroma QP offset");
        CHECK((pps->getQpOffset(JOINT_CbCr) + pcSlice->getSliceChromaQpDelta(JOINT_CbCr)) < -12,
              "Invalid chroma QP offset");
        CHECK((pps->getQpOffset(JOINT_CbCr) + pcSlice->getSliceChromaQpDelta(JOINT_CbCr)) > 12,
              "Invalid chroma QP offset");
      }
    }
  }

  if (pps->getCuChromaQpOffsetListEnabledFlag())
  {
    xReadFlag(uiCode, "sh_cu_chroma_qp_offset_enabled_flag");
    pcSlice->m_chromaQpAdjEnabled = (uiCode != 0);
  }
  else
  {
    pcSlice->m_chromaQpAdjEnabled = false;
  }

#if ENABLE_NNLF
  if (sps->m_nnlf && !picHeader->m_nnlfDisabled)
  {
    NnlfSliceParameters prm;
    xReadUvlc(uiCode, "slice_nnlf_unified_mode");
    prm.mode = uiCode - 1;
    if (prm.mode != -1)
    {
      const int numprms = NNLF_UNIFIED_MAX_NUM_PRMS;
      xReadUvlc(uiCode, "slice_nnlf_unified_scale_id");
      prm.scaleId = uiCode - 1;
      if (prm.scaleId == 0)
      {
        if (prm.mode < numprms)
        {
          xReadSCode(NNFilterUnified::log2ResidueScale + 1, iCode, "y nnScale");
          prm.scale[CompID::COMP_Y][prm.mode] = iCode + (1 << NNFilterUnified::log2ResidueScale);
          xReadSCode(NNFilterUnified::log2ResidueScale + 1, iCode, "cb nnScale");
          prm.scale[CompID::COMP_Cb][prm.mode] = iCode + (1 << NNFilterUnified::log2ResidueScale);
          xReadSCode(NNFilterUnified::log2ResidueScale + 1, iCode, "cr nnScale");
          prm.scale[CompID::COMP_Cr][prm.mode] = iCode + (1 << NNFilterUnified::log2ResidueScale);
        }
        else
        {
          for (int prmId = 0; prmId < numprms; prmId++)
          {
            xReadSCode(NNFilterUnified::log2ResidueScale + 1, iCode, "y nnScale");
            prm.scale[CompID::COMP_Y][prmId] = iCode + (1 << NNFilterUnified::log2ResidueScale);
            xReadSCode(NNFilterUnified::log2ResidueScale + 1, iCode, "cb nnScale");
            prm.scale[CompID::COMP_Cb][prmId] = iCode + (1 << NNFilterUnified::log2ResidueScale);
            xReadSCode(NNFilterUnified::log2ResidueScale + 1, iCode, "cr nnScale");
            prm.scale[CompID::COMP_Cr][prmId] = iCode + (1 << NNFilterUnified::log2ResidueScale);
          }
        }
      }

      int offsetI = (prm.scaleId == -1) ? 3 : prm.scaleId;
      if (prm.mode < numprms)
      {
        xReadFlag(uiCode, "y_roa_flag");
        if (uiCode == 0)
        {
          prm.offset[CompID::COMP_Y][prm.mode][offsetI] = 0;
        }
        else
        {
          xReadFlag(uiCode, "y_roa_offset");
          prm.offset[CompID::COMP_Y][prm.mode][offsetI] = uiCode + 1;
        }

        xReadFlag(uiCode, "u_roa_flag");
        if (uiCode == 0)
        {
          prm.offset[CompID::COMP_Cb][prm.mode][offsetI] = 0;
        }
        else
        {
          xReadFlag(uiCode, "u_roa_offset");
          prm.offset[CompID::COMP_Cb][prm.mode][offsetI] = uiCode + 1;
        }

        xReadFlag(uiCode, "v_roa_flag");
        if (uiCode == 0)
        {
          prm.offset[CompID::COMP_Cr][prm.mode][offsetI] = 0;
        }
        else
        {
          xReadFlag(uiCode, "v_roa_offset");
          prm.offset[CompID::COMP_Cr][prm.mode][offsetI] = uiCode + 1;
        }
      }
      else
      {
        for (int prmId = 0; prmId < numprms; prmId++)
        {
          xReadFlag(uiCode, "y_roa_flag");
          if (uiCode == 0)
          {
            prm.offset[CompID::COMP_Y][prmId][offsetI] = 0;
          }
          else
          {
            xReadFlag(uiCode, "y_roa_offset");
            prm.offset[CompID::COMP_Y][prmId][offsetI] = uiCode + 1;
          }

          xReadFlag(uiCode, "u_roa_flag");
          if (uiCode == 0)
          {
            prm.offset[CompID::COMP_Cb][prmId][offsetI] = 0;
          }
          else
          {
            xReadFlag(uiCode, "u_roa_offset");
            prm.offset[CompID::COMP_Cb][prmId][offsetI] = uiCode + 1;
          }

          xReadFlag(uiCode, "v_roa_flag");
          if (uiCode == 0)
          {
            prm.offset[CompID::COMP_Cr][prmId][offsetI] = 0;
          }
          else
          {
            xReadFlag(uiCode, "v_roa_offset");
            prm.offset[CompID::COMP_Cr][prmId][offsetI] = uiCode + 1;
          }
        }
      }
    }
    pcSlice->m_nnlfUnifiedParam = prm;
  }
#endif

  if (sps->m_saoEnabledFlag && !pps->m_saoInfoInPhFlag)
  {
    xReadFlag(uiCode, "sh_sao_luma_used_flag");
    pcSlice->m_saoEnabledFlag[ChannelType::LUMA] = (uiCode != 0);

    if (hasChroma)
    {
      xReadFlag(uiCode, "sh_sao_chroma_used_flag");
      pcSlice->m_saoEnabledFlag[ChannelType::CHROMA] = (uiCode != 0);
    }
  }
  parseCcSao(pcSlice, picHeader, sps, pcSlice->m_ccSaoComParam);
  if (pps->m_deblockingFilterControlPresentFlag)
  {
    if (pps->m_deblockingFilterOverrideEnabledFlag && !pps->m_dbfInfoInPhFlag)
    {
      xReadFlag(uiCode, "sh_deblocking_params_present_flag");
      pcSlice->m_deblockingFilterOverrideFlag = uiCode ? true : false;
    }
    else
    {
      pcSlice->m_deblockingFilterOverrideFlag = false;
    }
    if (pcSlice->m_deblockingFilterOverrideFlag)
    {
      if (!pps->m_ppsDeblockingFilterDisabledFlag)
      {
        xReadFlag(uiCode, "sh_deblocking_filter_disabled_flag");
        pcSlice->m_deblockingFilterDisable = uiCode != 0;
      }
      else
      {
        pcSlice->m_deblockingFilterDisable = false;
      }
      if (!pcSlice->m_deblockingFilterDisable)
      {
        xReadSvlc(iCode, "sh_luma_beta_offset_div2");
        pcSlice->m_deblockingFilterBetaOffsetDiv2 = iCode;
        CHECK(pcSlice->m_deblockingFilterBetaOffsetDiv2 < -12 || pcSlice->m_deblockingFilterBetaOffsetDiv2 > 12,
              "Invalid deblocking filter configuration");
        xReadSvlc(iCode, "sh_luma_tc_offset_div2");
        pcSlice->m_deblockingFilterTcOffsetDiv2 = iCode;
        CHECK(pcSlice->m_deblockingFilterTcOffsetDiv2 < -12 || pcSlice->m_deblockingFilterTcOffsetDiv2 > 12,
              "Invalid deblocking filter configuration");

        if (pps->m_usePPSChromaTool)
        {
          xReadSvlc(iCode, "sh_cb_beta_offset_div2");
          pcSlice->m_deblockingFilterCbBetaOffsetDiv2 = iCode;
          CHECK(pcSlice->m_deblockingFilterCbBetaOffsetDiv2 < -12 || pcSlice->m_deblockingFilterCbBetaOffsetDiv2 > 12,
                "Invalid deblocking filter configuration");
          xReadSvlc(iCode, "sh_cb_tc_offset_div2");
          pcSlice->m_deblockingFilterCbTcOffsetDiv2 = iCode;
          CHECK(pcSlice->m_deblockingFilterCbTcOffsetDiv2 < -12 || pcSlice->m_deblockingFilterCbTcOffsetDiv2 > 12,
                "Invalid deblocking filter configuration");

          xReadSvlc(iCode, "sh_cr_beta_offset_div2");
          pcSlice->m_deblockingFilterCrBetaOffsetDiv2 = iCode;
          CHECK(pcSlice->m_deblockingFilterCrBetaOffsetDiv2 < -12 || pcSlice->m_deblockingFilterCrBetaOffsetDiv2 > 12,
                "Invalid deblocking filter configuration");
          xReadSvlc(iCode, "sh_cr_tc_offset_div2");
          pcSlice->m_deblockingFilterCrTcOffsetDiv2 = iCode;
          CHECK(pcSlice->m_deblockingFilterCrTcOffsetDiv2 < -12 || pcSlice->m_deblockingFilterCrTcOffsetDiv2 > 12,
                "Invalid deblocking filter configuration");
        }
        else
        {
          pcSlice->m_deblockingFilterCbBetaOffsetDiv2 = pcSlice->m_deblockingFilterBetaOffsetDiv2;
          pcSlice->m_deblockingFilterCbTcOffsetDiv2   = pcSlice->m_deblockingFilterTcOffsetDiv2;
          pcSlice->m_deblockingFilterCrBetaOffsetDiv2 = pcSlice->m_deblockingFilterBetaOffsetDiv2;
          pcSlice->m_deblockingFilterCrTcOffsetDiv2   = pcSlice->m_deblockingFilterTcOffsetDiv2;
        }
      }
    }
    else
    {
      pcSlice->m_deblockingFilterDisable          = picHeader->m_deblockingFilterDisable;
      pcSlice->m_deblockingFilterBetaOffsetDiv2   = picHeader->m_deblockingFilterBetaOffsetDiv2;
      pcSlice->m_deblockingFilterTcOffsetDiv2     = picHeader->m_deblockingFilterTcOffsetDiv2;
      pcSlice->m_deblockingFilterCbBetaOffsetDiv2 = picHeader->m_deblockingFilterCbBetaOffsetDiv2;
      pcSlice->m_deblockingFilterCbTcOffsetDiv2   = picHeader->m_deblockingFilterCbTcOffsetDiv2;
      pcSlice->m_deblockingFilterCrBetaOffsetDiv2 = picHeader->m_deblockingFilterCrBetaOffsetDiv2;
      pcSlice->m_deblockingFilterCrTcOffsetDiv2   = picHeader->m_deblockingFilterCrTcOffsetDiv2;
    }
  }
  else
  {
    pcSlice->m_deblockingFilterDisable          = false;
    pcSlice->m_deblockingFilterBetaOffsetDiv2   = 0;
    pcSlice->m_deblockingFilterTcOffsetDiv2     = 0;
    pcSlice->m_deblockingFilterCbBetaOffsetDiv2 = 0;
    pcSlice->m_deblockingFilterCbTcOffsetDiv2   = 0;
    pcSlice->m_deblockingFilterCrBetaOffsetDiv2 = 0;
    pcSlice->m_deblockingFilterCrTcOffsetDiv2   = 0;
  }

  // dependent quantization
  if (sps->m_depQuantEnabledFlag)
  {
    xReadCode(2, uiCode, "sh_dep_quant_used_idc");
    CHECK(uiCode > 2, "Invalid value of sh_dep_quant_used_idc");
    pcSlice->m_depQuantEnabledIdc = uiCode;
  }
  else
  {
    pcSlice->m_depQuantEnabledIdc = 0;
  }

  // sign data hiding
  if (sps->m_signDataHidingEnabledFlag && !pcSlice->m_depQuantEnabledIdc)
  {
    xReadFlag(uiCode, "sh_sign_data_hiding_used_flag");
    pcSlice->m_signDataHidingEnabledFlag = (uiCode != 0);
  }
  else
  {
    pcSlice->m_signDataHidingEnabledFlag = false;
  }

  // signal TS residual coding disabled flag
  if (sps->m_transformSkipEnabledFlag && !pcSlice->m_depQuantEnabledIdc && !pcSlice->m_signDataHidingEnabledFlag)
  {
    xReadFlag(uiCode, "sh_ts_residual_coding_disabled_flag");
    pcSlice->m_tsResidualCodingDisabledFlag = (uiCode != 0);
  }
  else
  {
    pcSlice->m_tsResidualCodingDisabledFlag = false;
  }

  if ((!pcSlice->m_tsResidualCodingDisabledFlag) && sps->m_spsRangeExtension.m_tsrcRicePresentFlag)
  {
    xReadCode(3, uiCode, "sh_ts_residual_coding_rice_idx_minus1");
    pcSlice->m_tsrcIndex = uiCode;
  }
  if (sps->m_spsRangeExtension.m_reverseLastSigCoeffEnabledFlag)
  {
    xReadFlag(uiCode, "sh_reverse_last_sig_coeff_flag");
    pcSlice->m_reverseLastSigCoeffFlag = (uiCode != 0);
  }
  else
  {
    pcSlice->m_reverseLastSigCoeffFlag = false;
  }

  if (sps->m_licEnabledFlag && !pcSlice->isIntra())
  {
    xReadFlag(uiCode, "slice_lic_enable_flag");
    pcSlice->m_useLic = (uiCode != 0);
  }
  else
  {
    pcSlice->m_useLic = false;
  }

  if (pcSlice->getFirstCtuRsAddrInSlice() == 0)
  {
    pcSlice->setDefaultClpRng(*sps);
  }

  if (pps->m_sliceHeaderExtensionPresentFlag)
  {
    xReadUvlc(uiCode, "sh_slice_header_extension_length");
    for (int i = 0; i < uiCode; i++)
    {
      uint32_t ignore_;
      xReadCode(8, ignore_, "sh_slice_header_extension_data_byte");
    }
  }

  parseClippingValues(pcSlice);

  std::vector<uint32_t> entryPointOffset;

  pcSlice->m_numSubstream = 0;
  pcSlice->setNumSubstream(sps, pps);

  pcSlice->setNumEntryPoints(sps, pps);
  if (pcSlice->m_numEntryPoints > 0)
  {
    uint32_t offsetLenMinus1;
    xReadUvlc(offsetLenMinus1, "sh_entry_offset_len_minus1");
    entryPointOffset.resize(pcSlice->m_numEntryPoints);
    for (uint32_t idx = 0; idx < pcSlice->m_numEntryPoints; idx++)
    {
      xReadCode(offsetLenMinus1 + 1, uiCode, "sh_entry_point_offset_minus1");
      entryPointOffset[idx] = uiCode + 1;
    }
  }

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(STATS__BYTE_ALIGNMENT_BITS, m_pcBitstream->readByteAlignment(), 0);
#else
  m_pcBitstream->readByteAlignment();
#endif

  pcSlice->m_substreamSizes.clear();

  if (pcSlice->m_numEntryPoints > 0)
  {
    int endOfSliceHeaderLocation = m_pcBitstream->getByteLocation();

    // Adjust endOfSliceHeaderLocation to account for emulation prevention bytes in the slice segment header
    for (uint32_t curByteIdx = 0; curByteIdx < m_pcBitstream->numEmulationPreventionBytesRead(); curByteIdx++)
    {
      if (m_pcBitstream->getEmulationPreventionByteLocation(curByteIdx) < endOfSliceHeaderLocation)
      {
        endOfSliceHeaderLocation++;
      }
    }

    int curEntryPointOffset  = 0;
    int prevEntryPointOffset = 0;
    for (uint32_t idx = 0; idx < entryPointOffset.size(); idx++)
    {
      curEntryPointOffset += entryPointOffset[idx];

      int emulationPreventionByteCount = 0;
      for (uint32_t curByteIdx = 0; curByteIdx < m_pcBitstream->numEmulationPreventionBytesRead(); curByteIdx++)
      {
        if (m_pcBitstream->getEmulationPreventionByteLocation(curByteIdx) >=
              (prevEntryPointOffset + endOfSliceHeaderLocation) &&
            m_pcBitstream->getEmulationPreventionByteLocation(curByteIdx) <
              (curEntryPointOffset + endOfSliceHeaderLocation))
        {
          emulationPreventionByteCount++;
        }
      }

      entryPointOffset[idx] -= emulationPreventionByteCount;
      prevEntryPointOffset = curEntryPointOffset;
      pcSlice->m_substreamSizes.push_back(entryPointOffset[idx]);
    }
  }
  return;
}

void HLSyntaxReader::parseClippingValues(Slice *pcSlice)
{
  uint32_t value;
  xReadFlag(value, "adaptive_clip_quant");
  pcSlice->m_adaptiveClipQuant = (value ? true : false);
  xReadSvlc(pcSlice->m_lumaPelMax, "clip_luma_pel_max");
  xReadSvlc(pcSlice->m_lumaPelMin, "clip_luma_pel_min");
}

void HLSyntaxReader::getSlicePoc(Slice *pcSlice, PicHeader *picHeader, ParameterSetManager *parameterSetManager,
                                 const int prevTid0POC)
{
  uint32_t uiCode;
  uint32_t pocLsb;

  PPS *pps = nullptr;
  SPS *sps = nullptr;

  CHECK(picHeader == 0, "Invalid Picture Header");
  CHECK(picHeader->m_valid == false, "Invalid Picture Header");
  pps = parameterSetManager->getPPS(picHeader->m_ppsId);
  //! KS: need to add error handling code here, if PPS is not available
  CHECK(pps == 0, "Invalid PPS");
  sps = parameterSetManager->getSPS(pps->m_spsId);
  //! KS: need to add error handling code here, if SPS is not available
  CHECK(sps == 0, "Invalid SPS");

  DTRACE_UPDATE(g_trace_ctx, std::make_pair("final", 0));

  xReadFlag(uiCode, "sh_picture_header_in_slice_header_flag");
  if (uiCode == 0)
  {
    pocLsb = picHeader->m_pocLsb;
  }
  else
  {
    uint32_t phGdrOrIrapPicFlag;
    xReadFlag(phGdrOrIrapPicFlag, "ph_gdr_or_irap_pic_flag");
    xReadFlag(uiCode, "ph_non_ref_pic_flag");
    if (phGdrOrIrapPicFlag)
    {
      xReadFlag(uiCode, "ph_gdr_pic_flag");
    }
    xReadFlag(uiCode, "ph_inter_slice_allowed_flag");
    if (uiCode)
    {
      xReadFlag(uiCode, "ph_intra_slice_allowed_flag");
    }
    // parameter sets
    xReadUvlc(uiCode, "ph_pic_parameter_set_id");
    // picture order count
    xReadCode(sps->m_bitsForPoc, pocLsb, "ph_pic_order_cnt_lsb");
  }
  int maxPocLsb = 1 << sps->m_bitsForPoc;
  int pocMsb;
  if (pcSlice->getIdrPicFlag())
  {
    if (picHeader->m_pocMsbPresentFlag)
    {
      pocMsb = picHeader->m_pocMsbVal * maxPocLsb;
    }
    else
    {
      pocMsb = 0;
    }
    pcSlice->m_poc = pocMsb + pocLsb;
  }
  else
  {
    int prevPoc    = prevTid0POC;
    int prevPocLsb = prevPoc & (maxPocLsb - 1);
    int prevPocMsb = prevPoc - prevPocLsb;
    if (picHeader->m_pocMsbPresentFlag)
    {
      pocMsb = picHeader->m_pocMsbVal * maxPocLsb;
    }
    else
    {
      if ((pocLsb < prevPocLsb) && ((prevPocLsb - pocLsb) >= (maxPocLsb / 2)))
      {
        pocMsb = prevPocMsb + maxPocLsb;
      }
      else if ((pocLsb > prevPocLsb) && ((pocLsb - prevPocLsb) > (maxPocLsb / 2)))
      {
        pocMsb = prevPocMsb - maxPocLsb;
      }
      else
      {
        pocMsb = prevPocMsb;
      }
    }
    pcSlice->m_poc = pocMsb + pocLsb;
  }
  DTRACE_UPDATE(g_trace_ctx, std::make_pair("final", 1));
}

void HLSyntaxReader::parseConstraintInfo(ConstraintInfo *cinfo, const ProfileTierLevel *ptl)
{
  uint32_t symbol;
  xReadFlag(symbol, "gci_present_flag");
  cinfo->m_gciPresentFlag = (symbol ? true : false);
  if (cinfo->m_gciPresentFlag)
  {
    /* general */
    xReadFlag(symbol, "gci_intra_only_constraint_flag");
    cinfo->m_intraOnlyConstraintFlag = (symbol ? true : false);
    xReadFlag(symbol, "gci_all_layers_independent_constraint_flag");
    cinfo->m_allLayersIndependentConstraintFlag = (symbol ? true : false);
    xReadFlag(symbol, "gci_one_au_only_constraint_flag");
    cinfo->m_onePictureOnlyConstraintFlag = (symbol ? true : false);

    /* picture format */
    xReadCode(4, symbol, "gci_sixteen_minus_max_bitdepth_constraint_idc");
    cinfo->m_maxBitDepthConstraintIdc = (symbol > 8 ? 16 : (16 - symbol));
    CHECK(symbol > 8, "gci_sixteen_minus_max_bitdepth_constraint_idc shall be in the range 0 to 8, inclusive");
    xReadCode(2, symbol, "gci_three_minus_max_chroma_format_constraint_idc");
    cinfo->m_maxChromaFormatConstraintIdc = ((ChromaFormat)(3 - symbol));

    /* NAL unit type related */
    xReadFlag(symbol, "gci_no_mixed_nalu_types_in_pic_constraint_flag");
    cinfo->m_noMixedNaluTypesInPicConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_trail_constraint_flag");
    cinfo->m_noTrailConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_stsa_constraint_flag");
    cinfo->m_noStsaConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_rasl_constraint_flag");
    cinfo->m_noRaslConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_radl_constraint_flag");
    cinfo->m_noRadlConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_idr_constraint_flag");
    cinfo->m_noIdrConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_cra_constraint_flag");
    cinfo->m_noCraConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_gdr_constraint_flag");
    cinfo->m_noGdrConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_aps_constraint_flag");
    cinfo->m_noApsConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_idr_rpl_constraint_flag");
    cinfo->m_noIdrRplConstraintFlag = (symbol > 0 ? true : false);

    /* tile, slice, subpicture partitioning */
    xReadFlag(symbol, "gci_one_tile_per_pic_constraint_flag");
    cinfo->m_oneTilePerPicConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_pic_header_in_slice_header_constraint_flag");
    cinfo->m_picHeaderInSliceHeaderConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_one_slice_per_pic_constraint_flag");
    cinfo->m_oneSlicePerPicConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_rectangular_slice_constraint_flag");
    cinfo->m_noRectSliceConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_one_slice_per_subpic_constraint_flag");
    cinfo->m_oneSlicePerSubpicConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_subpic_info_constraint_flag");
    cinfo->m_noSubpicInfoConstraintFlag = (symbol > 0 ? true : false);

    /* CTU and block partitioning */
    xReadCode(2, symbol, "gci_three_minus_max_log2_ctu_size_constraint_idc");
    cinfo->m_maxLog2CtuSizeConstraintIdc = (((3 - symbol) + 5));
    xReadFlag(symbol, "gci_no_partition_constraints_override_constraint_flag");
    cinfo->m_noPartitionConstraintsOverrideConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_mtt_constraint_flag");
    cinfo->m_noMttConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_qtbtt_dual_tree_intra_constraint_flag");
    cinfo->m_noQtbttDualTreeIntraConstraintFlag = (symbol > 0 ? true : false);

    /* intra */
    xReadFlag(symbol, "gci_no_palette_constraint_flag");
    cinfo->m_noPaletteConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_ibc_constraint_flag");
    cinfo->m_noIbcConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_mrl_constraint_flag");
    cinfo->m_noMrlConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_mip_constraint_flag");
    cinfo->m_noMipConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_cclm_constraint_flag");
    cinfo->m_noCclmConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_sgpm_constraint_flag");
    cinfo->m_noSgpmConstraintFlag = (symbol > 0 ? true : false);

    /* inter */
    xReadFlag(symbol, "gci_no_ref_pic_resampling_constraint_flag");
    cinfo->m_noRprConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_res_change_in_clvs_constraint_flag");
    cinfo->m_noResChangeInClvsConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_weighted_prediction_constraint_flag");
    cinfo->m_noWeightedPredictionConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_ref_wraparound_constraint_flag");
    cinfo->m_noRefWraparoundConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_temporal_mvp_constraint_flag");
    cinfo->m_noTemporalMvpConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_sbtmvp_constraint_flag");
    cinfo->m_noSbtmvpConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_amvr_constraint_flag");
    cinfo->m_noAmvrConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_bdof_constraint_flag");
    cinfo->m_noBdofConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_smvd_constraint_flag");
    cinfo->m_noSmvdConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_dmvr_constraint_flag");
    cinfo->m_noDmvrConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_mmvd_constraint_flag");
    cinfo->m_noMmvdConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_affine_motion_constraint_flag");
    cinfo->m_noAffineMotionConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_prof_constraint_flag");
    cinfo->m_noProfConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_bcw_constraint_flag");
    cinfo->m_noBcwConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_ciip_constraint_flag");
    cinfo->m_noCiipConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_gpm_constraint_flag");
    cinfo->m_noGeoConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_obmc_constraint_flag");
    cinfo->m_noObmcConstraintFlag = (symbol > 0 ? true : false);

    /* transform, quantization, residual */
    xReadFlag(symbol, "gci_no_luma_transform_size_64_constraint_flag");
    cinfo->m_noLumaTransformSize64ConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_transform_skip_constraint_flag");
    cinfo->m_noTransformSkipConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_bdpcm_constraint_flag");
    cinfo->m_noBDPCMConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_mts_constraint_flag");
    cinfo->m_noMtsConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_lfnst_constraint_flag");
    cinfo->m_noLfnstConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_joint_cbcr_constraint_flag");
    cinfo->m_noJointCbCrConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_sbt_constraint_flag");
    cinfo->m_noSbtConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_act_constraint_flag");
    cinfo->m_noActConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_explicit_scaling_list_constraint_flag");
    cinfo->m_noExplicitScaleListConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_dep_quant_constraint_flag");
    cinfo->m_noDepQuantConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_sign_data_hiding_constraint_flag");
    cinfo->m_noSignDataHidingConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_cu_qp_delta_constraint_flag");
    cinfo->m_noCuQpDeltaConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_chroma_qp_offset_constraint_flag");
    cinfo->m_noChromaQpOffsetConstraintFlag = (symbol > 0 ? true : false);

    /* loop filter */
    xReadFlag(symbol, "gci_no_sao_constraint_flag");
    cinfo->m_noSaoConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_ccsao_constraint_flag");
    cinfo->m_noCCSaoConstraintFlag = symbol > 0 ? true : false;
    xReadFlag(symbol, "gci_no_alf_constraint_flag");
    cinfo->m_noAlfConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_ccalf_constraint_flag");
    cinfo->m_noCCAlfConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_lmcs_constraint_flag");
    cinfo->m_noLmcsConstraintFlag = (symbol > 0 ? true : false);
    xReadFlag(symbol, "gci_no_ladf_constraint_flag");
    cinfo->m_noLadfConstraintFlag = (symbol > 0 ? true : false);
    xReadCode(8, symbol, "gci_num_additional_bits");
    uint32_t const numAdditionalBits = symbol;
    int            numAdditionalBitsUsed;
    if (numAdditionalBits > 5)
    {
      xReadFlag(symbol, "gci_all_rap_pictures_flag");
      cinfo->m_allRapPicturesFlag = (symbol > 0 ? true : false);
      xReadFlag(symbol, "gci_no_extended_precision_processing_constraint_flag");
      cinfo->m_noExtendedPrecisionProcessingConstraintFlag = (symbol > 0 ? true : false);
      xReadFlag(symbol, "gci_no_ts_residual_coding_rice_constraint_flag");
      cinfo->m_noTsResidualCodingRiceConstraintFlag = (symbol > 0 ? true : false);
      xReadFlag(symbol, "gci_no_rrc_rice_extension_constraint_flag");
      cinfo->m_noRrcRiceExtensionConstraintFlag = (symbol > 0 ? true : false);
      xReadFlag(symbol, "gci_no_persistent_rice_adaptation_constraint_flag");
      cinfo->m_noPersistentRiceAdaptationConstraintFlag = (symbol > 0 ? true : false);
      xReadFlag(symbol, "gci_no_reverse_last_sig_coeff_constraint_flag");
      cinfo->m_noReverseLastSigCoeffConstraintFlag = (symbol > 0 ? true : false);
      numAdditionalBitsUsed                        = 6;
    }
    else if (numAdditionalBits > 0)
    {
      msg(ERROR, "Invalid bitstream: gci_num_additional_bits set to value %d (must be 0 or >= 6)\n", numAdditionalBits);
      numAdditionalBitsUsed = 0;
    }
    else
    {
      numAdditionalBitsUsed = 0;
    }
    for (int i = 0; i < numAdditionalBits - numAdditionalBitsUsed; i++)
    {
      xReadFlag(symbol, "gci_reserved_bit");
    }
  }
  while (!isByteAligned())
  {
    xReadFlag(symbol, "gci_alignment_zero_bit");
    CHECK(symbol != 0, "gci_alignment_zero_bit not equal to zero");
  }
}

void HLSyntaxReader::parseProfileTierLevel(ProfileTierLevel *ptl, bool profileTierPresentFlag,
                                           int maxNumSubLayersMinus1)
{
  uint32_t symbol;
  if (profileTierPresentFlag)
  {
    xReadCode(7, symbol, "general_profile_idc");
    ptl->m_profileIdc = Profile::Name(symbol);
    xReadFlag(symbol, "general_tier_flag");
    ptl->m_tierFlag = (symbol ? Level::HIGH : Level::MAIN);
  }

  xReadCode(8, symbol, "general_level_idc");
  ptl->m_levelIdc = Level::Name(symbol);
  CHECK(ptl->m_profileIdc != Profile::NONE && ptl->m_levelIdc < Level::LEVEL4 && ptl->m_tierFlag == Level::HIGH,
        "High tier not defined for levels below 4");

  xReadFlag(symbol, "ptl_frame_only_constraint_flag");
  ptl->m_frameOnlyConstraintFlag = symbol;
  xReadFlag(symbol, "ptl_multilayer_enabled_flag");
  ptl->m_multiLayerEnabledFlag = symbol;
  CHECK((ptl->m_profileIdc == Profile::MAIN_10 || ptl->m_profileIdc == Profile::MAIN_10_444 ||
         ptl->m_profileIdc == Profile::MAIN_10_STILL_PICTURE ||
         ptl->m_profileIdc == Profile::MAIN_10_444_STILL_PICTURE) &&
          symbol,
        "ptl_multilayer_enabled_flag shall be equal to 0 for non-multilayer profiles");

  if (profileTierPresentFlag)
  {
    parseConstraintInfo(&ptl->m_constraintInfo, ptl);
  }

  for (int i = maxNumSubLayersMinus1 - 1; i >= 0; i--)
  {
    xReadFlag(symbol, "sub_layer_level_present_flag[i]");
    ptl->m_subLayerLevelPresentFlag[i] = symbol;
  }

  while (!isByteAligned())
  {
    xReadFlag(symbol, "ptl_reserved_zero_bit");
    CHECK(symbol != 0, "ptl_reserved_zero_bit not equal to zero");
  }

  for (int i = maxNumSubLayersMinus1 - 1; i >= 0; i--)
  {
    if (ptl->m_subLayerLevelPresentFlag[i])
    {
      xReadCode(8, symbol, "sub_layer_level_idc");
      ptl->m_subLayerLevelIdc[i] = Level::Name(symbol);
    }
  }
  ptl->m_subLayerLevelIdc[maxNumSubLayersMinus1] = ptl->m_levelIdc;
  for (int i = maxNumSubLayersMinus1 - 1; i >= 0; i--)
  {
    if (!ptl->m_subLayerLevelPresentFlag[i])
    {
      ptl->m_subLayerLevelIdc[i] = ptl->m_subLayerLevelIdc[i + 1];
    }
  }

  if (profileTierPresentFlag)
  {
    xReadCode(8, symbol, "ptl_num_sub_profiles");
    uint8_t numSubProfiles = symbol;
    ptl->m_subProfileIdc.resize(numSubProfiles);
    for (int i = 0; i < numSubProfiles; i++)
    {
      xReadCode(32, symbol, "general_sub_profile_idc[i]");
      ptl->m_subProfileIdc[i] = symbol;
    }
  }
}

void HLSyntaxReader::parseTerminatingBit(uint32_t &ruiBit)
{
  ruiBit        = false;
  int iBitsLeft = m_pcBitstream->getNumBitsLeft();
  if (iBitsLeft <= 8)
  {
    uint32_t uiPeekValue = m_pcBitstream->peekBits(iBitsLeft);
    if (uiPeekValue == (1 << (iBitsLeft - 1)))
    {
      ruiBit = true;
    }
  }
}

void HLSyntaxReader::parseRemainingBytes(bool noTrailingBytesExpected)
{
  if (noTrailingBytesExpected)
  {
    CHECK(0 != m_pcBitstream->getNumBitsLeft(), "Bits left although no bits expected");
  }
  else
  {
    while (m_pcBitstream->getNumBitsLeft())
    {
      uint32_t trailingNullByte = m_pcBitstream->readByte();
      if (trailingNullByte != 0)
      {
        msg(ERROR, "Trailing byte should be 0, but has value %02x\n", trailingNullByte);
        THROW("Invalid trailing '0' byte");
      }
    }
  }
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

//! parse explicit wp tables
void HLSyntaxReader::parsePredWeightTable(Slice *pcSlice, const SPS *sps)
{
  const ChromaFormat chFmt        = sps->m_chromaFormatIdc;
  const int          numValidComp = int(getNumberValidComponents(chFmt));
  const bool         hasChroma    = isChromaEnabled(chFmt);

  uint32_t log2WeightDenomLuma       = 0;
  uint32_t log2WeightDenomChroma     = 0;
  uint32_t totalSignalledWeightFlags = 0;

  int deltaDenom;
  // decode delta_luma_log2_weight_denom :
  xReadUvlc(log2WeightDenomLuma, "luma_log2_weight_denom");
  CHECK(log2WeightDenomLuma > 7, "The value of luma_log2_weight_denom shall be in the range of 0 to 7");
  if (hasChroma)
  {
    xReadSvlc(deltaDenom, "delta_chroma_log2_weight_denom");
    CHECK((deltaDenom + (int)log2WeightDenomLuma) < 0,
          "luma_log2_weight_denom + delta_chroma_log2_weight_denom shall be in the range of 0 to 7");
    CHECK((deltaDenom + (int)log2WeightDenomLuma) > 7,
          "luma_log2_weight_denom + delta_chroma_log2_weight_denom shall be in the range of 0 to 7");
    log2WeightDenomChroma = (uint32_t)(deltaDenom + log2WeightDenomLuma);
  }

  for (const auto l: { RPL0, RPL1 })
  {
    const bool l0 = l == RPL0;

    if (!l0 && !pcSlice->isInterB())
    {
      continue;
    }

    for (int refIdx = 0; refIdx < pcSlice->m_numRefIdx[l]; refIdx++)
    {
      WPScalingParam *wp = pcSlice->getWpScaling(l, refIdx);

      wp[COMP_Y].log2WeightDenom = log2WeightDenomLuma;
      for (int j = 1; j < numValidComp; j++)
      {
        wp[j].log2WeightDenom = log2WeightDenomChroma;
      }

      uint32_t uiCode;
      xReadFlag(uiCode, (l0 ? "luma_weight_l0_flag[i]" : "luma_weight_l1_flag[i]"));
      wp[COMP_Y].presentFlag = uiCode != 0;
      totalSignalledWeightFlags += wp[COMP_Y].presentFlag ? 1 : 0;
    }
    if (hasChroma)
    {
      for (int refIdx = 0; refIdx < pcSlice->m_numRefIdx[l]; refIdx++)
      {
        WPScalingParam *wp = pcSlice->getWpScaling(l, refIdx);
        uint32_t        uiCode;
        xReadFlag(uiCode, (l0 ? "chroma_weight_l0_flag[i]" : "chroma_weight_l1_flag[i]"));
        for (int j = 1; j < numValidComp; j++)
        {
          wp[j].presentFlag = uiCode != 0;
          totalSignalledWeightFlags += wp[COMP_Cb].presentFlag ? 1 : 0;
        }
      }
    }
    else
    {
      for (int refIdx = 0; refIdx < MAX_NUM_REF; refIdx++)
      {
        WPScalingParam *wp = pcSlice->getWpScaling(l, refIdx);

        wp[COMP_Cb].presentFlag = false;
        wp[COMP_Cr].presentFlag = false;
      }
    }
    for (int refIdx = 0; refIdx < pcSlice->m_numRefIdx[l]; refIdx++)
    {
      WPScalingParam *wp = pcSlice->getWpScaling(l, refIdx);
      if (wp[COMP_Y].presentFlag)
      {
        int deltaWeight;
        xReadSvlc(deltaWeight, (l0 ? "delta_luma_weight_l0[i]" : "delta_luma_weight_l1[i]"));
        CHECK(deltaWeight < -128, "delta_luma_weight_lx shall be in the rage of -128 to 127");
        CHECK(deltaWeight > 127, "delta_luma_weight_lx shall be in the rage of -128 to 127");
        wp[COMP_Y].codedWeight = (deltaWeight + (1 << wp[COMP_Y].log2WeightDenom));
        xReadSvlc(wp[COMP_Y].codedOffset, (l0 ? "luma_offset_l0[i]" : "luma_offset_l1[i]"));
        const int range = sps->m_spsRangeExtension.m_highPrecisionOffsetsEnabledFlag
          ? (1 << sps->m_bitDepths[ChannelType::LUMA]) / 2
          : 128;
        CHECK(wp[COMP_Y].codedOffset < -range, "luma_offset_lx shall be in the rage of -128 to 127");
        CHECK(wp[COMP_Y].codedOffset >= range, "luma_offset_lx shall be in the rage of -128 to 127");
      }
      else
      {
        wp[COMP_Y].codedWeight = 1 << wp[COMP_Y].log2WeightDenom;
        wp[COMP_Y].codedOffset = 0;
      }
      if (hasChroma)
      {
        if (wp[COMP_Cb].presentFlag)
        {
          int range = sps->m_spsRangeExtension.m_highPrecisionOffsetsEnabledFlag
            ? (1 << sps->m_bitDepths[ChannelType::CHROMA]) / 2
            : 128;
          for (int j = 1; j < numValidComp; j++)
          {
            int deltaWeight;
            xReadSvlc(deltaWeight, (l0 ? "delta_chroma_weight_l0[i]" : "delta_chroma_weight_l1[i]"));
            CHECK(deltaWeight < -128, "delta_chroma_weight_lx shall be in the rage of -128 to 127");
            CHECK(deltaWeight > 127, "delta_chroma_weight_lx shall be in the rage of -128 to 127");
            wp[j].codedWeight = (deltaWeight + (1 << wp[j].log2WeightDenom));

            int deltaChroma;
            xReadSvlc(deltaChroma, (l0 ? "delta_chroma_offset_l0[i]" : "delta_chroma_offset_l1[i]"));
            CHECK(deltaChroma < -4 * range, "delta_chroma_offset_lx shall be in the range of -4 * 128 to 4 * 127");
            CHECK(deltaChroma > 4 * (range - 1), "delta_chroma_offset_lx shall be in the range of -4 * 128 to 4 * 127");
            int pred          = (range - ((range * wp[j].codedWeight) >> (wp[j].log2WeightDenom)));
            wp[j].codedOffset = Clip3(-range, range - 1, (deltaChroma + pred));
          }
        }
        else
        {
          for (int j = 1; j < numValidComp; j++)
          {
            wp[j].codedWeight = 1 << wp[j].log2WeightDenom;
            wp[j].codedOffset = 0;
          }
        }
      }
    }

    for (int refIdx = pcSlice->m_numRefIdx[l]; refIdx < MAX_NUM_REF; refIdx++)
    {
      WPScalingParam *wp = pcSlice->getWpScaling(l, refIdx);

      wp[COMP_Y].presentFlag  = false;
      wp[COMP_Cb].presentFlag = false;
      wp[COMP_Cr].presentFlag = false;
    }
  }
  CHECK(totalSignalledWeightFlags > 24, "Too many weight flag signalled");
}

void HLSyntaxReader::parsePredWeightTable(PicHeader *picHeader, const PPS *pps, const SPS *sps)
{
  const ChromaFormat chFmt        = sps->m_chromaFormatIdc;
  const int          numValidComp = getNumberValidComponents(chFmt);
  const bool         chroma       = isChromaEnabled(chFmt);

  uint32_t log2WeightDenomLuma       = 0;
  uint32_t log2WeightDenomChroma     = 0;
  uint32_t totalSignalledWeightFlags = 0;
  xReadUvlc(log2WeightDenomLuma, "luma_log2_weight_denom");
  CHECK(log2WeightDenomLuma > 7, "The value of luma_log2_weight_denom shall be in the range of 0 to 7");
  if (chroma)
  {
    int deltaDenom;
    xReadSvlc(deltaDenom, "delta_chroma_log2_weight_denom");
    log2WeightDenomChroma = deltaDenom + log2WeightDenomLuma;
    CHECK(log2WeightDenomChroma > 7,
          "luma_log2_weight_denom + delta_chroma_log2_weight_denom shall be in the range of 0 to 7");
  }

  for (const auto l: { RPL0, RPL1 })
  {
    const bool l0 = l == RPL0;

    WPScalingParam *wp;

    uint32_t numLxWeights = 0;
    if (l0 || (pps->m_useBiWP && picHeader->m_rpl[l].getNumRefEntries() > 0))
    {
      xReadUvlc(numLxWeights, (l0 ? "num_l0_weights" : "num_l1_weights"));
    }
    picHeader->m_numWeights[l] = numLxWeights;

    for (int refIdx = 0; refIdx < numLxWeights; refIdx++)
    {
      wp = picHeader->getWpScaling(l, refIdx);

      wp[COMP_Y].log2WeightDenom = log2WeightDenomLuma;
      for (int j = 1; j < numValidComp; j++)
      {
        wp[j].log2WeightDenom = log2WeightDenomChroma;
      }

      uint32_t uiCode;
      xReadFlag(uiCode, (l0 ? "luma_weight_l0_flag[i]" : "luma_weight_l1_flag[i]"));
      wp[COMP_Y].presentFlag = uiCode != 0;
      totalSignalledWeightFlags += wp[COMP_Y].presentFlag ? 1 : 0;
    }
    if (chroma)
    {
      uint32_t uiCode;
      for (int refIdx = 0; refIdx < numLxWeights; refIdx++)
      {
        wp = picHeader->getWpScaling(l, refIdx);
        xReadFlag(uiCode, (l0 ? "chroma_weight_l0_flag[i]" : "chroma_weight_l1_flag[i]"));
        for (int j = 1; j < numValidComp; j++)
        {
          wp[j].presentFlag = uiCode != 0;
          totalSignalledWeightFlags += wp[COMP_Cb].presentFlag ? 1 : 0;
        }
      }
    }
    else
    {
      for (int refIdx = 0; refIdx < MAX_NUM_REF; refIdx++)
      {
        wp = picHeader->getWpScaling(l, refIdx);

        wp[1].presentFlag = false;
        wp[2].presentFlag = false;
      }
    }
    for (int refIdx = 0; refIdx < numLxWeights; refIdx++)
    {
      wp = picHeader->getWpScaling(l, refIdx);
      if (wp[COMP_Y].presentFlag)
      {
        int deltaWeight;
        xReadSvlc(deltaWeight, (l0 ? "delta_luma_weight_l0[i]" : "delta_luma_weight_l1[i]"));
        CHECK(deltaWeight < -128, "delta_luma_weight_lx shall be in the rage of -128 to 127");
        CHECK(deltaWeight > 127, "delta_luma_weight_lx shall be in the rage of -128 to 127");
        wp[COMP_Y].codedWeight = (deltaWeight + (1 << wp[COMP_Y].log2WeightDenom));
        xReadSvlc(wp[COMP_Y].codedOffset, (l0 ? "luma_offset_l0[i]" : "luma_offset_l1[i]"));
        const int range = sps->m_spsRangeExtension.m_highPrecisionOffsetsEnabledFlag
          ? (1 << sps->m_bitDepths[ChannelType::LUMA]) / 2
          : 128;
        CHECK(wp[0].codedOffset < -range, "luma_offset_lx shall be in the rage of -128 to 127");
        CHECK(wp[0].codedOffset >= range, "luma_offset_lx shall be in the rage of -128 to 127");
      }
      else
      {
        wp[COMP_Y].codedWeight = (1 << wp[COMP_Y].log2WeightDenom);
        wp[COMP_Y].codedOffset = 0;
      }
      if (chroma)
      {
        if (wp[COMP_Cb].presentFlag)
        {
          int range = sps->m_spsRangeExtension.m_highPrecisionOffsetsEnabledFlag
            ? (1 << sps->m_bitDepths[ChannelType::CHROMA]) / 2
            : 128;
          for (int j = 1; j < numValidComp; j++)
          {
            int deltaWeight;
            xReadSvlc(deltaWeight, (l0 ? "delta_chroma_weight_l0[i]" : "delta_chroma_weight_l1[i]"));
            CHECK(deltaWeight < -128, "delta_chroma_weight_lx shall be in the rage of -128 to 127");
            CHECK(deltaWeight > 127, "delta_chroma_weight_lx shall be in the rage of -128 to 127");
            wp[j].codedWeight = (deltaWeight + (1 << wp[j].log2WeightDenom));

            int deltaChroma;
            xReadSvlc(deltaChroma, (l0 ? "delta_chroma_offset_l0[i]" : "delta_chroma_offset_l1[i]"));
            CHECK(deltaChroma < -4 * range, "delta_chroma_offset_lx shall be in the range of -4 * 128 to 4 * 127");
            CHECK(deltaChroma >= 4 * range, "delta_chroma_offset_lx shall be in the range of -4 * 128 to 4 * 127");
            int pred          = (range - ((range * wp[j].codedWeight) >> (wp[j].log2WeightDenom)));
            wp[j].codedOffset = Clip3(-range, range - 1, (deltaChroma + pred));
          }
        }
        else
        {
          for (int j = 1; j < numValidComp; j++)
          {
            wp[j].codedWeight = (1 << wp[j].log2WeightDenom);
            wp[j].codedOffset = 0;
          }
        }
      }
    }

    for (int refIdx = numLxWeights; refIdx < MAX_NUM_REF; refIdx++)
    {
      wp = picHeader->getWpScaling(l, refIdx);

      wp[COMP_Y].presentFlag  = false;
      wp[COMP_Cb].presentFlag = false;
      wp[COMP_Cr].presentFlag = false;
    }
  }
  CHECK(totalSignalledWeightFlags > 24, "Too many weight flag signalled");
}

/** decode quantization matrix
 * \param scalingList quantization matrix information
 */
void HLSyntaxReader::parseScalingList(ScalingList *scalingList, bool aps_chromaPrsentFlag)
{
  uint32_t code;
  bool     scalingListCopyModeFlag;
  scalingList->m_chromaScalingListPresentFlag = aps_chromaPrsentFlag;
  for (int scalingListId = 0; scalingListId < 28; scalingListId++)
  {
    if (aps_chromaPrsentFlag || scalingList->isLumaScalingList(scalingListId))
    {
      xReadFlag(code, "scaling_list_copy_mode_flag");
      scalingListCopyModeFlag                                     = (code) ? true : false;
      scalingList->m_scalingListPredModeFlagIsCopy[scalingListId] = scalingListCopyModeFlag;

      scalingList->m_scalingListPreditorModeFlag[scalingListId] = false;
      if (!scalingListCopyModeFlag)
      {
        xReadFlag(code, "scaling_list_predictor_mode_flag");
        scalingList->m_scalingListPreditorModeFlag[scalingListId] = code;
      }

      if ((scalingListCopyModeFlag || scalingList->m_scalingListPreditorModeFlag[scalingListId]) &&
          scalingListId != SCALING_LIST_1D_START_2x2 && scalingListId != SCALING_LIST_1D_START_4x4 &&
          scalingListId != SCALING_LIST_1D_START_8x8)   // Copy Mode
      {
        xReadUvlc(code, "scaling_list_pred_matrix_id_delta");
        scalingList->m_refMatrixId[scalingListId] = (uint32_t)((int)(scalingListId) - (code));
      }
      else if (scalingListCopyModeFlag || scalingList->m_scalingListPreditorModeFlag[scalingListId])
      {
        scalingList->m_refMatrixId[scalingListId] = (uint32_t)((int)(scalingListId));
      }
      if (scalingListCopyModeFlag)   // copy
      {
        if (scalingListId >= SCALING_LIST_1D_START_16x16)
        {
          scalingList->m_scalingListDC[scalingListId] =
            ((scalingListId == scalingList->m_refMatrixId[scalingListId]) ? 16
               : (scalingList->m_refMatrixId[scalingListId] < SCALING_LIST_1D_START_16x16)
               ? scalingList->getScalingListAddress(scalingList->m_refMatrixId[scalingListId])[0]
               : scalingList->m_scalingListDC[scalingList->m_refMatrixId[scalingListId]]);
        }
        scalingList->processRefMatrix(scalingListId, scalingList->m_refMatrixId[scalingListId]);
      }
      else
      {
        decodeScalingList(scalingList, scalingListId, scalingList->m_scalingListPreditorModeFlag[scalingListId]);
      }
    }
    else
    {
      scalingListCopyModeFlag                                     = true;
      scalingList->m_scalingListPredModeFlagIsCopy[scalingListId] = scalingListCopyModeFlag;
      scalingList->m_refMatrixId[scalingListId]                   = (uint32_t)((int)(scalingListId));
      if (scalingListId >= SCALING_LIST_1D_START_16x16)
      {
        scalingList->m_scalingListDC[scalingListId] = 16;
      }
      scalingList->processRefMatrix(scalingListId, scalingList->m_refMatrixId[scalingListId]);
    }
  }

  return;
}

/** decode DPCM
 * \param scalingList  quantization matrix information
 * \param sizeId size index
 * \param listId list index
 */
void HLSyntaxReader::decodeScalingList(ScalingList *scalingList, uint32_t scalingListId, bool isPredictor)
{
  int          matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2
             : (scalingListId < SCALING_LIST_1D_START_8x8)              ? 4
                                                                        : 8;
  int          i, coefNum = matrixSize * matrixSize;
  int          data;
  int          scalingListDcCoefMinus8 = 0;
  int          nextCoef                = (isPredictor) ? 0 : SCALING_LIST_START_VALUE;
  ScanElement *scan = g_scanOrder[SCAN_UNGROUPED][CoeffScanType::DIAG][gp_sizeIdxInfo->idxFrom(matrixSize)]
                                 [gp_sizeIdxInfo->idxFrom(matrixSize)];
  int *dst = scalingList->getScalingListAddress(scalingListId);

  int predListId = scalingList->m_refMatrixId[scalingListId];
  CHECK(isPredictor && predListId > scalingListId, "Scaling List error predictor!");
  const int *srcPred = (isPredictor)
    ? ((scalingListId == predListId) ? scalingList->getScalingListDefaultAddress(scalingListId)
                                     : scalingList->getScalingListAddress(predListId))
    : nullptr;
  if (isPredictor && scalingListId == predListId)
  {
    scalingList->m_scalingListDC[predListId] = SCALING_LIST_DC;
  }
  int predCoef = 0;

  if (scalingListId >= SCALING_LIST_1D_START_16x16)
  {
    xReadSvlc(scalingListDcCoefMinus8, "scaling_list_dc_coef_minus8");
    nextCoef += scalingListDcCoefMinus8;
    if (isPredictor)
    {
      predCoef = (predListId >= SCALING_LIST_1D_START_16x16) ? scalingList->m_scalingListDC[predListId] : srcPred[0];
    }
    scalingList->m_scalingListDC[scalingListId] = (nextCoef + predCoef + 256) & 255;
  }

  for (i = 0; i < coefNum; i++)
  {
    if (scalingListId >= SCALING_LIST_1D_START_64x64 && scan[i].x >= 4 && scan[i].y >= 4)
    {
      dst[scan[i].idx] = 0;
      continue;
    }
    xReadSvlc(data, "scaling_list_delta_coef");
    nextCoef += data;
    predCoef         = (isPredictor) ? srcPred[scan[i].idx] : 0;
    dst[scan[i].idx] = (nextCoef + predCoef + 256) & 255;
  }
}

bool HLSyntaxReader::xMoreRbspData()
{
  int bitsLeft = m_pcBitstream->getNumBitsLeft();

  // if there are more than 8 bits, it cannot be rbsp_trailing_bits
  if (bitsLeft > 8)
  {
    return true;
  }

  uint8_t lastByte = m_pcBitstream->peekBits(bitsLeft);
  int     cnt      = bitsLeft;

  // remove trailing bits equal to zero
  while ((cnt > 0) && ((lastByte & 1) == 0))
  {
    lastByte >>= 1;
    cnt--;
  }
  // remove bit equal to one
  cnt--;

  // we should not have a negative number of bits
  CHECK(cnt < 0, "Negative number of bits");

  // we have more data, if cnt is not zero
  return (cnt > 0);
}

void HLSyntaxReader::parseCcSao(Slice *pcSlice, PicHeader *picHeader, const SPS *sps, CcSaoComParam &ccSaoParam)
{
  ccSaoParam.reset();

  uint32_t uiCode;
  if (sps->m_ccSaoEnabledFlag)
  {
    xReadFlag(uiCode, "ccsao_y_enabled_flag");
    pcSlice->m_ccSaoEnabledFlag[COMP_Y] = uiCode;
    ccSaoParam.enabled[COMP_Y]          = uiCode;
    xReadFlag(uiCode, "ccsao_cb_enabled_flag");
    pcSlice->m_ccSaoEnabledFlag[COMP_Cb] = uiCode;
    ccSaoParam.enabled[COMP_Cb]          = uiCode;
    xReadFlag(uiCode, "ccsao_cr_enabled_flag");
    pcSlice->m_ccSaoEnabledFlag[COMP_Cr] = uiCode;
    ccSaoParam.enabled[COMP_Cr]          = uiCode;
  }
  else
  {
    ccSaoParam.enabled[COMP_Y]  = false;
    ccSaoParam.enabled[COMP_Cb] = false;
    ccSaoParam.enabled[COMP_Cr] = false;
  }

  if (ccSaoParam.enabled[COMP_Y] || ccSaoParam.enabled[COMP_Cb] || ccSaoParam.enabled[COMP_Cr])
  {
    xReadFlag(uiCode, "ccsao_ext_chroma_flag");
    ccSaoParam.extChroma[COMP_Y] = ccSaoParam.extChroma[COMP_Cb] = ccSaoParam.extChroma[COMP_Cr] = uiCode;
  }

  for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
  {
    if (ccSaoParam.enabled[compIdx])
    {
      if (!pcSlice->isIntra())
      {
        xReadFlag(uiCode, "ccsao_reuse_prv_flag");
        ccSaoParam.reusePrv[compIdx] = uiCode;
      }
      else
      {
        ccSaoParam.reusePrv[compIdx] = false;
      }

      if (ccSaoParam.reusePrv[compIdx])
      {
        xReadCode(MAX_CCSAO_PRV_NUM_BITS, uiCode, "ccsao_reuse_prv_id");
        ccSaoParam.reusePrvId[compIdx] = uiCode;
        continue;
      }

      xReadUvlc(uiCode, "ccsao_set_num");
      ccSaoParam.setNum[compIdx] = uiCode + 1;

      for (int setIdx = 0; setIdx < ccSaoParam.setNum[compIdx]; setIdx++)
      {
        ccSaoParam.setEnabled[compIdx][setIdx] = true;
        xReadFlag(uiCode, "ccsao_set_type");
        ccSaoParam.setType[compIdx][setIdx] = uiCode;
        if (ccSaoParam.setType[compIdx][setIdx] == CCSAO_SET_TYPE_EDGE)
        {
          if (ccSaoParam.extChroma[compIdx])
          {
            xReadCode(MAX_CCSAO_EDGE_CMP_BITS, uiCode, "ccsao_edge_cmp");
            ccSaoParam.candPos[compIdx][setIdx][COMP_Cb] = uiCode;
          }
          xReadCode(MAX_CCSAO_EDGE_IDC_BITS, uiCode, "ccsao_edge_idc");
          ccSaoParam.bandNum[compIdx][setIdx][COMP_Cr] = uiCode;
          xReadCode(MAX_CCSAO_EDGE_DIR_BITS, uiCode, "ccsao_edge_dir");
          ccSaoParam.candPos[compIdx][setIdx][COMP_Y] = uiCode;
          if (sps->m_ccSaoFastFlag)
          {
            xReadCode(2, uiCode, "ccsao_edge_thr");
          }
          else
          {
            xReadCode(MAX_CCSAO_EDGE_THR_BITS, uiCode, "ccsao_edge_thr");
          }
          ccSaoParam.bandNum[compIdx][setIdx][COMP_Cb] = uiCode;
          xReadCode(MAX_CCSAO_BAND_IDC_BITS, uiCode, "ccsao_band_idc");
          ccSaoParam.bandNum[compIdx][setIdx][COMP_Y] = uiCode;
        }
        else
        {
          xReadCode(MAX_CCSAO_CAND_POS_Y_BITS, uiCode, "ccsao_cand_pos_y");
          ccSaoParam.candPos[compIdx][setIdx][COMP_Y] = uiCode;
          xReadCode(MAX_CCSAO_BAND_NUM_Y_BITS, uiCode, "ccsao_band_num_y");
          ccSaoParam.bandNum[compIdx][setIdx][COMP_Y] = uiCode + 1;
          xReadCode(MAX_CCSAO_BAND_NUM_U_BITS, uiCode, "ccsao_band_num_u");
          ccSaoParam.bandNum[compIdx][setIdx][COMP_Cb] = uiCode + 1;
          xReadCode(MAX_CCSAO_BAND_NUM_V_BITS, uiCode, "ccsao_band_num_v");
          ccSaoParam.bandNum[compIdx][setIdx][COMP_Cr] = uiCode + 1;
        }
        short *offset   = ccSaoParam.offset[compIdx][setIdx];
        int    classNum = SampleAdaptiveOffset::getCcSaoClassNum(compIdx, setIdx, ccSaoParam);

        for (int i = 0; i < classNum; i++)
        {
          xReadUvlc(uiCode, "ccsao_offset_abs");
          offset[i] = uiCode;
          if (offset[i] != 0)
          {
            xReadFlag(uiCode, "ccsao_offset_sign");
            offset[i] = uiCode ? -offset[i] : offset[i];
          }
        }

        DTRACE(g_trace_ctx, D_HEADER, "offset setIdx %d: ", setIdx);
        for (int i = 0; i < classNum; i++)
        {
          DTRACE(g_trace_ctx, D_HEADER, "%d ", offset[i]);
        }
        DTRACE(g_trace_ctx, D_HEADER, "\n");
      }
    }
  }
}

int HLSyntaxReader::alfHuffmanDecode(AlfHuffmanCode &huffman)
{
  AlfHuffmanCode::Node *node = nullptr;
  uint32_t              b;
  uint32_t              buf = 0;
  do
  {
    xReadFlag(b, "alf_coeff_huffman");
    buf = (buf << 1) | b;
  } while (!huffman.decodeBit(b, node));
  return huffman.getCoeff(node);
}

void HLSyntaxReader::alfFilter(AlfParametersVtm::AlfParam &alfParam, const bool isChroma, const int altIdx)
{
  uint32_t code;

  AlfParametersVtm::AlfFilterShape alfShape(isChroma ? AlfParametersVtm::AlfFilterType::ALF_FILTER_5
                                                     : AlfParametersVtm::AlfFilterType::ALF_FILTER_7);

  const int numFilters = isChroma ? 1 : alfParam.numLumaFilters;
  short    *coeff      = isChroma ? alfParam.chromaCoeff[altIdx].data() : alfParam.lumaCoeff.data();
  Pel      *clipp      = isChroma ? alfParam.chromaClipp[altIdx].data() : alfParam.lumaClipp.data();

  const int offset = AlfParametersVtm::MAX_NUM_ALF_LUMA_COEFF;

  // Filter coefficients
  for (int ind = 0; ind < numFilters; ++ind)
  {
    for (int i = 0; i < alfShape.numCoeff - 1; i++)
    {
      xReadUvlc(code, isChroma ? "alf_chroma_coeff_abs" : "alf_luma_coeff_abs");
      coeff[ind * offset + i] = code;
      if (coeff[ind * offset + i] != 0)
      {
        xReadFlag(code, isChroma ? "alf_chroma_coeff_sign" : "alf_luma_coeff_sign");
        coeff[ind * offset + i] = (code) ? -coeff[ind * offset + i] : coeff[ind * offset + i];
      }
      CHECK(isChroma && (coeff[ind * offset + i] > 127 || coeff[ind * offset + i] < -128),
            "AlfCoeffC shall be in the range of -128 to 127, inclusive");
    }
  }

  // Clipping values coding
  if (isChroma ? alfParam.chromaNonLinearFlag : alfParam.lumaNonLinearFlag)
  {
    // Filter coefficients
    for (int ind = 0; ind < numFilters; ++ind)
    {
      for (int i = 0; i < alfShape.numCoeff - 1; i++)
      {
        xReadCode(2, code, isChroma ? "alf_chroma_clip_idx" : "alf_luma_clip_idx");
        clipp[ind * offset + i] = code;
      }
    }
  }
  else
  {
    for (int ind = 0; ind < numFilters; ++ind)
    {
      std::fill_n(clipp + ind * offset, alfShape.numCoeff, 0);
    }
  }
}

void HLSyntaxReader::alfFilter(AlfParametersEcm::AlfParam &alfParam, const bool isChroma, const int altIdx)
{
  uint32_t code;

  AlfParametersEcm::AlfFilterType alfFilterType =
    isChroma ? alfParam.filterType[ChannelType::CHROMA] : alfParam.filterType[ChannelType::LUMA];
  AlfParametersEcm::AlfFilterShape alfShape(alfFilterType);

  const int numFilters = isChroma ? 1 : alfParam.numLumaFilters[altIdx];
  int8_t   *scaleIdx   = isChroma ? alfParam.chromaScaleIdx[altIdx].data() : alfParam.lumaScaleIdx[altIdx].data();
  short    *coeff      = isChroma ? alfParam.chromaCoeff[altIdx].data() : alfParam.lumaCoeff[altIdx].data();
  Pel      *clipp      = isChroma ? alfParam.chromaClipp[altIdx].data() : alfParam.lumaClipp[altIdx].data();

  const int offset = isChroma ? AlfParametersEcm::MAX_NUM_ALF_CHROMA_COEFF : AlfParametersEcm::MAX_NUM_ALF_LUMA_COEFF;
  AlfHuffmanCode huffman(
    !isChroma, isChroma ? AlfParametersEcm::COEFF_SCALE_BITS_CHROMA : AlfParametersEcm::COEFF_SCALE_BITS_LUMA,
    isChroma ? AlfParametersEcm::COEFF_MANTISSA_BITS_CHROMA : AlfParametersEcm::COEFF_MANTISSA_BITS_LUMA, 0);
  huffman.init();

  // Filter coefficients
  for (int ind = 0; ind < numFilters; ++ind)
  {
    if (AlfParametersEcm::ALF_SCALE_BITS_NUM > 0)
    {
      xReadCode(AlfParametersEcm::ALF_SCALE_BITS_NUM, code,
                isChroma ? "alf_chroma_scale_factor" : "alf_luma_scale_factor");
    }
    else
    {
      code = 0;
    }
    scaleIdx[ind] = (int8_t)code;
    for (int i = 0; i < alfShape.numCoeff - 1; i++)
    {
      huffman.setGroup(i < alfShape.indexSecOrder ? 0 : 1);
      coeff[ind * offset + i] = alfHuffmanDecode(huffman);
    }
  }

  // Clipping values coding
  if (isChroma ? alfParam.chromaNonLinearFlag[altIdx] : alfParam.lumaNonLinearFlag[altIdx])
  {
    // Filter coefficients
    for (int ind = 0; ind < numFilters; ++ind)
    {
      for (int i = 0; i < alfShape.numCoeff - 1; i++)
      {
        xReadCode(2, code, isChroma ? "alf_chroma_clip_idx" : "alf_luma_clip_idx");
        clipp[ind * offset + i] = code;
      }
    }
  }
  else
  {
    for (int ind = 0; ind < numFilters; ++ind)
    {
      std::fill_n(clipp + ind * offset, alfShape.numCoeff, 0);
    }
  }
}

//! \}
