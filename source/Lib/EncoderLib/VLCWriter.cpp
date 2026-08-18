/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ITU/ISO/IEC
 * All rights reserved.
 *
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

/** \file     VLCWriter.cpp
 *  \brief    Writer for high level syntax
 */

#include "VLCWriter.h"
#include "SEIwrite.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Unit.h"
#include "CommonLib/Picture.h" // th remove this
#include "CommonLib/dtrace_next.h"
#include "CommonLib/AlfParameters.h"
#include "CommonLib/AlfParametersEcm.h"
#include "CommonLib/AlfParametersVtm.h"
#include "CommonLib/AdaptiveLoopFilter.h"
#include "CommonLib/ProfileTierLevel.h"
#include "CommonLib/SampleAdaptiveOffset.h"
#if ENABLE_NNLF
#include "NNFilterUnified.h"
#endif
//! \ingroup EncoderLib
//! \{

#if ENABLE_TRACING
bool g_HLSTraceEnable = true;
#endif

#if ENABLE_TRACING
void VLCWriter::xWriteSCode(const int value, const uint32_t length, const char *symbolName)
#else
void VLCWriter::xWriteSCode(const int value, const uint32_t length, const char *)
#endif
{
  CHECK(length < 1 || length > 32, "Syntax element length must be in range 1..32");
  CHECK(!(length == 32 || (value >= -(1 << (length - 1)) && value < (1 << (length - 1)))), "Invalid syntax element");
  m_pcBitIf->write(length == 32 ? uint32_t(value) : (uint32_t(value) & ((1 << length) - 1)), length);

#if ENABLE_TRACING
  if (g_HLSTraceEnable)
  {
    if (length < 10)
    {
      DTRACE(g_trace_ctx, D_HEADER, "%-50s u(%d)  : %d\n", symbolName, length, value);
    }
    else
    {
      DTRACE(g_trace_ctx, D_HEADER, "%-50s u(%d) : %d\n", symbolName, length, value);
    }
  }
#endif
}

#if ENABLE_TRACING
void VLCWriter::xWriteCode(const uint32_t value, const uint32_t length, const char *symbolName)
#else
void VLCWriter::xWriteCode(const uint32_t value, const uint32_t length, const char *)
#endif
{
  CHECK(length == 0, "Code of length '0' not supported");
  m_pcBitIf->write(value, length);

#if ENABLE_TRACING
  if (g_HLSTraceEnable)
  {
    if (length < 10)
    {
      DTRACE(g_trace_ctx, D_HEADER, "%-50s u(%d)  : %d\n", symbolName, length, value);
    }
    else
    {
      DTRACE(g_trace_ctx, D_HEADER, "%-50s u(%d) : %d\n", symbolName, length, value);
    }
  }
#endif
}

// write the VLC code without tracing
void VLCWriter::xWriteVlc(uint32_t value)
{
  uint32_t length = 1;
  uint32_t temp   = ++value;

  CHECK(!temp, "Integer overflow");
  while (1 != temp)
  {
    temp >>= 1;
    length += 2;
  }
  // Take care of cases where length > 32
  m_pcBitIf->write(0, length >> 1);
  m_pcBitIf->write(value, (length + 1) >> 1);
}

#if ENABLE_TRACING
void VLCWriter::xWriteUvlc(const uint32_t value, const char *symbolName)
#else
void VLCWriter::xWriteUvlc(const uint32_t value, const char *)
#endif
{
  xWriteVlc(value);

#if ENABLE_TRACING
  if (g_HLSTraceEnable)
  {
    DTRACE(g_trace_ctx, D_HEADER, "%-50s ue(v) : %d\n", symbolName, value);
  }
#endif
}

void VLCWriter::xWriteSvlc(const int value, const char *symbolName)
{
  uint32_t unsigendValue = uint32_t(value <= 0 ? (-value) << 1 : (value << 1) - 1);
  xWriteVlc(unsigendValue);

#if ENABLE_TRACING
  if (g_HLSTraceEnable)
  {
    DTRACE(g_trace_ctx, D_HEADER, "%-50s se(v) : %d\n", symbolName, value);
  }
#endif
}

#if ENABLE_TRACING
void VLCWriter::xWriteFlag(uint32_t value, const char *symbolName)
#else
void VLCWriter::xWriteFlag(uint32_t value, const char *)
#endif
{
  m_pcBitIf->write(value, 1);

#if ENABLE_TRACING
  if (g_HLSTraceEnable)
  {
    DTRACE(g_trace_ctx, D_HEADER, "%-50s u(1)  : %d\n", symbolName, value);
  }
#endif
}

#if ENABLE_TRACING
void VLCWriter::xWriteString(const std::string &value, const char *symbolName)
#else
void VLCWriter::xWriteString(const std::string &value, const char *)
#endif
{
  for (int i = 0; i < value.length(); ++i)
  {
    m_pcBitIf->write(value[i], 8);
  }
  m_pcBitIf->write('\0', 8);

#if ENABLE_TRACING
  if (g_HLSTraceEnable)
  {
    DTRACE(g_trace_ctx, D_HEADER, "%-50s st(v)  : %s\n", symbolName, value.c_str());
  }
#endif
}

void VLCWriter::xWriteRbspTrailingBits()
{
  xWriteFlag(1, "rbsp_stop_one_bit");
  int cnt = 0;
  while (m_pcBitIf->getNumBitsUntilByteAligned())
  {
    xWriteFlag(0, "rbsp_alignment_zero_bit");
    cnt++;
  }
  CHECK(cnt >= 8, "More than '8' alignment bytes read");
}

void AUDWriter::codeAUD(OutputBitstream &bs, const bool audIrapOrGdrAuFlag, const int pictureType)
{
#if ENABLE_TRACING
  xTraceAccessUnitDelimiter();
#endif

  CHECK(pictureType >= 3, "Invalid picture type");
  setBitstream(&bs);
  xWriteFlag(audIrapOrGdrAuFlag, "aud_irap_or_gdr_au_flag");
  xWriteCode(pictureType, 3, "pic_type");
  xWriteRbspTrailingBits();
}

void FDWriter::codeFD(OutputBitstream &bs, uint32_t &fdSize)
{
#if ENABLE_TRACING
  xTraceFillerData();
#endif
  setBitstream(&bs);
  uint32_t ffByte = 0xff;
  while (fdSize)
  {
    xWriteCode(ffByte, 8, "ff_byte");
    fdSize--;
  }
  xWriteRbspTrailingBits();
}

void HLSWriter::xCodeRefPicListInterRPL(const SPS *pcSPS, const int numberOfRPL)
{
  // Precalculated numbers for hierarchical structure of pictures. The TemporalID
  // range in the NAL unit header is 0-6 so 64 is the largest that can be expressed
  static const int POC_values_for_gop_64[64] = { 64, 32, 16, 8,  4,  2,  1,  3,  6,  5,  7,  12, 10, 9,  11, 14,
                                                 13, 15, 24, 20, 18, 17, 19, 22, 21, 23, 28, 26, 25, 27, 30, 29,
                                                 31, 48, 40, 36, 34, 33, 35, 38, 37, 39, 44, 42, 41, 43, 46, 45,
                                                 47, 56, 52, 50, 49, 51, 54, 53, 55, 60, 58, 57, 59, 62, 61, 63 };

  // Code hierarchicalLevels to use for generating the sequence of delta POC values instead of signalling them
  // ToDo: Size of GOP should not be derived from temporal layers but instead specified in the config file
  int hierarchicalLevels = pcSPS->m_maxSubLayers - 1;
  xWriteUvlc(hierarchicalLevels, "inter_rpl_hierarchical_levels");

  for (int i = 0; i < (hierarchicalLevels > 0 ? hierarchicalLevels + 1 : numberOfRPL); i++)
  {
    xWriteSvlc(pcSPS->m_QPoffsetRPL[i], "rpl_qp_delta");
  }

  uint32_t         prevNumEntriesLX[2] = { 0, 0 };
  std::vector<int> rps;
  for (int rplIdx = 0; rplIdx < numberOfRPL; rplIdx++)
  {
    uint32_t                    numEntriesLX[2];
    const ReferencePictureList *rplLX[2];
    rplLX[0] = pcSPS->m_rplList[RPL0].getReferencePictureList(rplIdx);
    rplLX[1] = pcSPS->m_rplList[RPL1].getReferencePictureList(rplIdx);

    numEntriesLX[0] = rplLX[0]->m_numberOfShorttermPictures;
    numEntriesLX[1] = rplLX[1]->m_numberOfShorttermPictures;

    if (rplIdx == 0)
    {
      for (int lx = 0; lx < (pcSPS->m_rpl1CopyFromRpl0Flag ? 1 : 2); lx++)
      {
        int prevDelta  = MAX_INT;
        int deltaValue = 0;
        xWriteUvlc(numEntriesLX[lx], "num_ref_entries_lX");
        for (int ii = 0; ii < numEntriesLX[lx]; ii++)
        {
          if (ii == 0)
          {
            deltaValue = prevDelta = rplLX[lx]->m_refPicIdentifier[ii];
          }
          else
          {
            deltaValue = rplLX[lx]->m_refPicIdentifier[ii] - prevDelta;
            prevDelta  = rplLX[lx]->m_refPicIdentifier[ii];
          }
          unsigned int absDeltaValue = (deltaValue < 0) ? 0 - deltaValue : deltaValue;
          if (ii == 0)
          {
            CHECK(!absDeltaValue, "Zero delta POC is not used without WP or is the 0-th entry");
            xWriteUvlc(absDeltaValue - 1, "inter_rpl_entry_abs_delta_poc[ LX ][ rplsIdx ][ i ]");
          }
          else
          {
            xWriteUvlc(absDeltaValue, "inter_rpl_entry_abs_delta_poc[ LX ][ rplsIdx ][ i ]");
          }
          if (absDeltaValue > 0)
          {
            xWriteFlag(deltaValue < 0 ? 1 : 0, "inter_rpl_entry_sign_flag[ LX ][ rplsIdx ][ i ]");
          }
        }
      }
    }
    else
    {
      int pocCurrent  = rplLX[0]->m_POCvalue;
      int pocPrevious = pcSPS->m_rplList[RPL0].getReferencePictureList(rplIdx - 1)->m_POCvalue;
      int deltapocRpl = (hierarchicalLevels == 0) ? 1 : pocCurrent - pocPrevious;

      // Check that InterRPL prediction is possible, otherwise throw an error
      int predictFromRpsPossibleFlag = 1;
      rps.push_back(0); // add current picture (temporarily)
      for (int lx = 0; lx < 2; lx++)
      {
        for (int ii = 0; ii < numEntriesLX[lx]; ii++)
        {
          int flag   = 0;
          int pocRpl = rplLX[lx]->m_refPicIdentifier[ii];
          for (int pocRps: rps)
          {
            if (pocRpl == (pocRps - deltapocRpl))
            {
              flag = 1;
            }
          }
          if (flag == 0)
          {
            predictFromRpsPossibleFlag = 0;
          }
        }
      }
      rps.pop_back();
      CHECK(predictFromRpsPossibleFlag == 0, "InterRPL enabled but prediction is not possible");

      // Calculate the delta POC between the current and previous picture.
      // If the picture is an additional picture, code it
      deltapocRpl = 1;
      if (hierarchicalLevels > 0)
      {
        deltapocRpl = pocCurrent - pocPrevious;
        if (rplIdx < (1 << hierarchicalLevels))
        {
          CHECK(POC_values_for_gop_64[6 - hierarchicalLevels + rplIdx] != pocCurrent,
                "InterRPL delta POC derivation failed due to mismatching config file RPL POC values");
          CHECK(POC_values_for_gop_64[6 - hierarchicalLevels + rplIdx - 1] != pocPrevious,
                "InterRPL delta POC derivation failed due to mismatching config file RPL POC values");
        }
      }
      if (hierarchicalLevels > 0 && rplIdx >= (1 << hierarchicalLevels))
      {
        unsigned int absValue = (deltapocRpl < 0) ? 0 - deltapocRpl : deltapocRpl;
        xWriteUvlc(absValue - 1, "inter_rpl_abs_delta_poc_minus1");
        xWriteFlag(deltapocRpl < 0 ? 1 : 0, "inter_rpl_sign_flag");
      }

      // Update the RPS by recalculating POC offsets using the delta POC
      for (int i = 0; i < rps.size(); i++)
      {
        rps[i] -= deltapocRpl;
      }

      // Add POC of the current picture and sort
      rps.push_back(-deltapocRpl);
      sort(rps.begin(), rps.end());

      const int SizeRPS = (int)rps.size();

      // Check whether L0 and L1 are identical to the default lists
      int indexToFirstPositiveEntry;
      for (indexToFirstPositiveEntry = 0; indexToFirstPositiveEntry < SizeRPS && rps[indexToFirstPositiveEntry] < 0;
           indexToFirstPositiveEntry++)
        ;

      uint32_t useDefaultList[2] = { 1, 1 };
      for (int ii = 0; ii < numEntriesLX[0]; ii++)
      {
        int pocRpl = rplLX[0]->m_refPicIdentifier[ii];
        int index  = indexToFirstPositiveEntry - ii - 1;
        if (index < 0)
        {
          index = ii % SizeRPS;
        }
        if (pocRpl != rps[index])
        {
          useDefaultList[0] = 0;
        }
      }
      for (int ii = 0; ii < numEntriesLX[1]; ii++)
      {
        int pocRpl = rplLX[1]->m_refPicIdentifier[ii];
        int index  = indexToFirstPositiveEntry + ii;
        if (index >= SizeRPS)
        {
          index = SizeRPS - (ii % SizeRPS) - 1;
        }
        if (pocRpl != rps[index])
        {
          useDefaultList[1] = 0;
        }
      }

      // Encode L0 and L1 entries
      for (int lx = 0; lx < (pcSPS->m_rpl1CopyFromRpl0Flag ? 1 : 2); lx++)
      {
        xWriteSvlc(numEntriesLX[lx] - prevNumEntriesLX[lx], "delta_num_ref_entries_lx");
        xWriteFlag(useDefaultList[lx], "inter_rpl_use_default_list_flag_lx");
        if (useDefaultList[lx] == 0)
        {
          const int bitlength = ceilLog2(SizeRPS);
          // Encode LX using RPS
          for (int ii = 0; ii < numEntriesLX[lx]; ii++)
          {
            int value  = 0;
            int pocRpl = rplLX[lx]->m_refPicIdentifier[ii];
            for (int i = 0; i < SizeRPS; i++)
            {
              if (pocRpl == rps[i])
              {
                value = i;
              }
            }
            xWriteCode(value, bitlength, "inter_rpl_idx_lx");
          }
        }
      }
    }

    prevNumEntriesLX[0] = numEntriesLX[0];
    prevNumEntriesLX[1] = numEntriesLX[1];

    // Construct RPS for the next picture, add L0 and L1 entries without duplicates
    rps.resize(0);
    for (int lx = 0; lx < 2; lx++)
    {
      for (int ii = 0; ii < numEntriesLX[lx]; ii++)
      {
        int pocRpl = rplLX[lx]->m_refPicIdentifier[ii];
        if (std::find(rps.begin(), rps.end(), pocRpl) == rps.end())
        {
          rps.push_back(pocRpl);
        }
      }
    }
  }
}

void HLSWriter::xCodeRefPicList(const ReferencePictureList *rpl, bool isLongTermPresent, uint32_t ltLsbBitsCount,
                                const bool isForbiddenZeroDeltaPoc, int rplIdx)
{
  uint32_t numRefPic = rpl->getNumRefEntries();
  xWriteUvlc(numRefPic, "num_ref_entries[ listIdx ][ rplsIdx ]");

  if (isLongTermPresent && numRefPic > 0 && rplIdx != -1)
  {
    xWriteFlag(rpl->m_ltrpInSliceHeaderFlag, "ltrp_in_slice_header_flag[ listIdx ][ rplsIdx ]");
  }
  int  prevDelta  = MAX_INT;
  int  deltaValue = 0;
  bool firstSTRP  = true;
  for (int ii = 0; ii < numRefPic; ii++)
  {
    if (rpl->m_interLayerPresentFlag)
    {
      xWriteFlag(rpl->m_isInterLayerRefPic[ii], "inter_layer_ref_pic_flag[ listIdx ][ rplsIdx ][ i ]");

      if (rpl->m_isInterLayerRefPic[ii])
      {
        CHECK(rpl->m_interLayerRefPicIdx[ii] < 0, "Wrong inter-layer reference index");
        xWriteUvlc(rpl->m_interLayerRefPicIdx[ii], "ilrp_idx[ listIdx ][ rplsIdx ][ i ]");
      }
    }

    if (!rpl->m_isInterLayerRefPic[ii])
    {
      if (isLongTermPresent)
      {
        xWriteFlag(!rpl->m_isLongtermRefPic[ii], "st_ref_pic_flag[ listIdx ][ rplsIdx ][ i ]");
      }

      if (!rpl->m_isLongtermRefPic[ii])
      {
        if (firstSTRP)
        {
          firstSTRP  = false;
          deltaValue = prevDelta = rpl->m_refPicIdentifier[ii];
        }
        else
        {
          deltaValue = rpl->m_refPicIdentifier[ii] - prevDelta;
          prevDelta  = rpl->m_refPicIdentifier[ii];
        }
        unsigned int absDeltaValue = (deltaValue < 0) ? 0 - deltaValue : deltaValue;
        if (isForbiddenZeroDeltaPoc || ii == 0)
        {
          CHECK(!absDeltaValue, "Zero delta POC is not used without WP or is the 0-th entry");
          xWriteUvlc(absDeltaValue - 1, "abs_delta_poc_st[ listIdx ][ rplsIdx ][ i ]");
        }
        else
        {
          xWriteUvlc(absDeltaValue, "abs_delta_poc_st[ listIdx ][ rplsIdx ][ i ]");
        }
        if (absDeltaValue > 0)
        {
          xWriteFlag(deltaValue < 0 ? 1 : 0, "strp_entry_sign_flag[ listIdx ][ rplsIdx ][ i ]");
        }
      }
      else if (!rpl->m_ltrpInSliceHeaderFlag)
      {
        xWriteCode(rpl->m_refPicIdentifier[ii], ltLsbBitsCount, "poc_lsb_lt[listIdx][rplsIdx][i]");
      }
    }
  }
}

void HLSWriter::codePPS(const PPS *pcPPS)
{
#if ENABLE_TRACING
  xTracePPSHeader();
#endif
  xWriteCode(pcPPS->m_ppsId, 6, "pps_pic_parameter_set_id");
  xWriteCode(pcPPS->m_spsId, 4, "pps_seq_parameter_set_id");

  xWriteFlag(pcPPS->m_mixedNaluTypesInPicFlag ? 1 : 0, "pps_mixed_nalu_types_in_pic_flag");

  xWriteUvlc(pcPPS->m_picWidthInLumaSamples, "pps_pic_width_in_luma_samples");
  xWriteUvlc(pcPPS->m_picHeightInLumaSamples, "pps_pic_height_in_luma_samples");

  Window conf = pcPPS->m_conformanceWindow;
  xWriteFlag(pcPPS->m_conformanceWindowFlag, "pps_conformance_window_flag");
  if (pcPPS->m_conformanceWindowFlag)
  {
    xWriteUvlc(conf.m_winLeftOffset, "pps_conf_win_left_offset");
    xWriteUvlc(conf.m_winRightOffset, "pps_conf_win_right_offset");
    xWriteUvlc(conf.m_winTopOffset, "pps_conf_win_top_offset");
    xWriteUvlc(conf.m_winBottomOffset, "pps_conf_win_bottom_offset");
  }
  Window scalingWindow = pcPPS->m_scalingWindow;
  xWriteFlag(pcPPS->m_explicitScalingWindowFlag, "pps_scaling_window_explicit_signalling_flag");
  if (pcPPS->m_explicitScalingWindowFlag)
  {
    xWriteSvlc(scalingWindow.m_winLeftOffset, "pps_scaling_win_left_offset");
    xWriteSvlc(scalingWindow.m_winRightOffset, "pps_scaling_win_right_offset");
    xWriteSvlc(scalingWindow.m_winTopOffset, "pps_scaling_win_top_offset");
    xWriteSvlc(scalingWindow.m_winBottomOffset, "pps_scaling_win_bottom_offset");
  }

  xWriteFlag(pcPPS->m_outputFlagPresentFlag ? 1 : 0, "pps_output_flag_present_flag");
  xWriteFlag(pcPPS->m_noPicPartitionFlag ? 1 : 0, "pps_no_pic_partition_flag");
  xWriteFlag(pcPPS->m_subPicIdMappingInPpsFlag ? 1 : 0, "pps_subpic_id_mapping_present_flag");
  if (pcPPS->m_subPicIdMappingInPpsFlag)
  {
    CHECK(pcPPS->m_numSubPics < 1, "PPS: NumSubPics cannot be less than 1");
    if (!pcPPS->m_noPicPartitionFlag)
    {
      xWriteUvlc(pcPPS->m_numSubPics - 1, "pps_num_subpics_minus1");
    }
    CHECK(pcPPS->m_subPicIdLen < 1, "PPS: SubPicIdLen cannot be less than 1");
    xWriteUvlc(pcPPS->m_subPicIdLen - 1, "pps_subpic_id_len_minus1");

    CHECK((1 << pcPPS->m_subPicIdLen) < pcPPS->m_numSubPics, "pps_subpic_id_len exceeds valid range");
    for (int picIdx = 0; picIdx < pcPPS->m_numSubPics; picIdx++)
    {
      xWriteCode(pcPPS->m_subPicId[picIdx], pcPPS->m_subPicIdLen, "pps_subpic_id[i]");
    }
  }
  if (!pcPPS->m_noPicPartitionFlag)
  {
    int colIdx, rowIdx;

    // CTU size - required to match size in SPS
    xWriteCode(pcPPS->m_log2CtuSize - 6, 2, "pps_log2_ctu_size_minus6");

    // number of explicit tile columns/rows
    xWriteUvlc(pcPPS->m_numExpTileCols - 1, "pps_num_exp_tile_columns_minus1");
    xWriteUvlc(pcPPS->m_numExpTileRows - 1, "pps_num_exp_tile_rows_minus1");

    // tile sizes
    for (colIdx = 0; colIdx < pcPPS->m_numExpTileCols; colIdx++)
    {
      xWriteUvlc(pcPPS->getTileColumnWidth(colIdx) - 1, "pps_tile_column_width_minus1[i]");
    }
    for (rowIdx = 0; rowIdx < pcPPS->m_numExpTileRows; rowIdx++)
    {
      xWriteUvlc(pcPPS->getTileRowHeight(rowIdx) - 1, "pps_tile_row_height_minus1[i]");
    }

    // rectangular slice signalling
    if (pcPPS->getNumTiles() > 1)
    {
      xWriteFlag(pcPPS->m_loopFilterAcrossTilesEnabledFlag, "pps_loop_filter_across_tiles_enabled_flag");
      xWriteFlag(pcPPS->m_rectSliceFlag ? 1 : 0, "pps_rect_slice_flag");
    }
    if (pcPPS->m_rectSliceFlag)
    {
      xWriteFlag(pcPPS->m_singleSlicePerSubPicFlag ? 1 : 0, "pps_single_slice_per_subpic_flag");
    }
    if (pcPPS->m_rectSliceFlag && !(pcPPS->m_singleSlicePerSubPicFlag))
    {
      xWriteUvlc(pcPPS->m_numSlicesInPic - 1, "pps_num_slices_in_pic_minus1");
      if ((pcPPS->m_numSlicesInPic - 1) > 1)
      {
        xWriteFlag(pcPPS->m_tileIdxDeltaPresentFlag ? 1 : 0, "pps_tile_idx_delta_present_flag");
      }

      // write rectangular slice parameters
      for (int i = 0; i < pcPPS->m_numSlicesInPic - 1; i++)
      {
        // complete tiles within a single slice
        if ((pcPPS->m_rectSlices[i].m_tileIdx % pcPPS->m_numTileCols) != pcPPS->m_numTileCols - 1)
        {
          xWriteUvlc(pcPPS->m_rectSlices[i].m_sliceWidthInTiles - 1, "pps_slice_width_in_tiles_minus1[i]");
        }

        if (pcPPS->m_rectSlices[i].m_tileIdx / pcPPS->m_numTileCols != pcPPS->m_numTileRows - 1 &&
            (pcPPS->m_tileIdxDeltaPresentFlag || pcPPS->m_rectSlices[i].m_tileIdx % pcPPS->m_numTileCols == 0))
        {
          xWriteUvlc(pcPPS->m_rectSlices[i].m_sliceHeightInTiles - 1, "pps_slice_height_in_tiles_minus1[i]");
        }

        // multiple slices within a single tile special case
        if (pcPPS->m_rectSlices[i].m_sliceWidthInTiles == 1 && pcPPS->m_rectSlices[i].m_sliceHeightInTiles == 1 &&
            pcPPS->getTileRowHeight(pcPPS->m_rectSlices[i].m_tileIdx / pcPPS->m_numTileCols) > 1)
        {
          uint32_t numExpSliceInTile =
            (pcPPS->m_rectSlices[i].m_numSlicesInTile == 1) ? 0 : pcPPS->m_rectSlices[i].m_numSlicesInTile;
          if (numExpSliceInTile > 1 &&
              pcPPS->m_rectSlices[i + numExpSliceInTile - 2].m_sliceHeightInCtu >=
                pcPPS->m_rectSlices[i + numExpSliceInTile - 1].m_sliceHeightInCtu)
          {
            numExpSliceInTile--;
            while (numExpSliceInTile > 1 &&
                   pcPPS->m_rectSlices[i + numExpSliceInTile - 2].m_sliceHeightInCtu ==
                     pcPPS->m_rectSlices[i + numExpSliceInTile - 1].m_sliceHeightInCtu)
            {
              numExpSliceInTile--;
            }
          }
          uint32_t expSliceHeightSum = 0;
          xWriteUvlc(numExpSliceInTile, "pps_num_exp_slices_in_tile[i]");
          for (int j = 0; j < numExpSliceInTile; j++)
          {
            xWriteUvlc(pcPPS->m_rectSlices[i + j].m_sliceHeightInCtu - 1, "pps_exp_slice_height_in_ctus_minus1[i]");
            expSliceHeightSum += pcPPS->m_rectSlices[i + j].m_sliceHeightInCtu;
          }

          CHECK(expSliceHeightSum > pcPPS->getTileRowHeight(pcPPS->m_rectSlices[i].m_tileIdx / pcPPS->m_numTileCols),
                "The sum of expressed slice heights is larger than the height of the tile containing the slices.");
          i += (pcPPS->m_rectSlices[i].m_numSlicesInTile - 1);
        }

        // tile index offset to start of next slice
        if (i < pcPPS->m_numSlicesInPic - 1)
        {
          if (pcPPS->m_tileIdxDeltaPresentFlag)
          {
            int32_t tileIdxDelta = pcPPS->m_rectSlices[i + 1].m_tileIdx - pcPPS->m_rectSlices[i].m_tileIdx;
            xWriteSvlc(tileIdxDelta, "pps_tile_idx_delta[i]");
          }
        }
      }
    }

    if (pcPPS->m_rectSliceFlag == 0 || pcPPS->m_singleSlicePerSubPicFlag || pcPPS->m_numSlicesInPic > 1)
    {
      xWriteFlag(pcPPS->m_loopFilterAcrossSlicesEnabledFlag, "pps_loop_filter_across_slices_enabled_flag");
    }
  }

  xWriteFlag(pcPPS->m_cabacInitPresentFlag ? 1 : 0, "pps_cabac_init_present_flag");
  xWriteUvlc(pcPPS->m_numRefIdxDefaultActive[RPL0] - 1, "pps_num_ref_idx_default_active_minus1[0]");
  xWriteUvlc(pcPPS->m_numRefIdxDefaultActive[RPL1] - 1, "pps_num_ref_idx_default_active_minus1[1]");
  xWriteFlag(pcPPS->m_rpl1IdxPresentFlag ? 1 : 0, "pps_rpl1_idx_present_flag");
  xWriteFlag(pcPPS->m_useWP ? 1 : 0, "pps_weighted_pred_flag");   // Use of Weighting Prediction (P_SLICE)
  xWriteFlag(pcPPS->m_useBiWP ? 1 : 0, "pps_weighted_bipred_flag");  // Use of Weighting Bi-Prediction (B_SLICE)
  xWriteFlag(pcPPS->m_wrapAroundEnabledFlag ? 1 : 0, "pps_ref_wraparound_enabled_flag");
  if (pcPPS->m_wrapAroundEnabledFlag)
  {
    xWriteUvlc(pcPPS->m_picWidthMinusWrapAroundOffset, "pps_pic_width_minus_wraparound_offset");
  }

  xWriteSvlc(pcPPS->m_picInitQPMinus26, "pps_init_qp_minus26");
  xWriteFlag(pcPPS->m_useDQP ? 1 : 0, "pps_cu_qp_delta_enabled_flag");
  xWriteFlag(pcPPS->m_usePPSChromaTool ? 1 : 0, "pps_chroma_tool_offsets_present_flag");
  if (pcPPS->m_usePPSChromaTool)
  {
    xWriteSvlc(pcPPS->getQpOffset(COMP_Cb), "pps_cb_qp_offset");
    xWriteSvlc(pcPPS->getQpOffset(COMP_Cr), "pps_cr_qp_offset");
    xWriteFlag(pcPPS->m_chromaJointCbCrQpOffsetPresentFlag ? 1 : 0, "pps_joint_cbcr_qp_offset_present_flag");
    if (pcPPS->m_chromaJointCbCrQpOffsetPresentFlag)
    {
      xWriteSvlc(pcPPS->getQpOffset(JOINT_CbCr), "pps_joint_cbcr_qp_offset_value");
    }

    xWriteFlag(pcPPS->m_sliceChromaQpFlag ? 1 : 0, "pps_slice_chroma_qp_offsets_present_flag");

    xWriteFlag(uint32_t(pcPPS->getCuChromaQpOffsetListEnabledFlag()), "pps_cu_chroma_qp_offset_list_enabled_flag");
    if (pcPPS->getCuChromaQpOffsetListEnabledFlag())
    {
      xWriteUvlc(pcPPS->m_chromaQpOffsetListLen - 1, "pps_chroma_qp_offset_list_len_minus1");
      /* skip zero index */
      for (int cuChromaQpOffsetIdx = 0; cuChromaQpOffsetIdx < pcPPS->m_chromaQpOffsetListLen; cuChromaQpOffsetIdx++)
      {
        xWriteSvlc(pcPPS->getChromaQpOffsetListEntry(cuChromaQpOffsetIdx + 1).u.comp.cbOffset,
                   "pps_cb_qp_offset_list[i]");
        xWriteSvlc(pcPPS->getChromaQpOffsetListEntry(cuChromaQpOffsetIdx + 1).u.comp.crOffset,
                   "pps_cr_qp_offset_list[i]");
        if (pcPPS->m_chromaJointCbCrQpOffsetPresentFlag)
        {
          xWriteSvlc(pcPPS->getChromaQpOffsetListEntry(cuChromaQpOffsetIdx + 1).u.comp.jointCbCrOffset,
                     "pps_joint_cbcr_qp_offset_list[i]");
        }
      }
    }
  }

  xWriteFlag(pcPPS->m_useSgpmNoBlend ? 1 : 0, "sgpm_no_blend_flag");
  ;
  xWriteFlag(pcPPS->m_BIF ? 1 : 0, "bilateral_filter_flag");
  if (pcPPS->m_BIF)
  {
    xWriteCode(pcPPS->m_BIFStrength, 2, "bilateral_filter_strength");
    xWriteSvlc(pcPPS->m_BIFQPOffset, "bilateral_filter_qp_offset");
  }
  xWriteFlag(pcPPS->m_chromaBIF ? 1 : 0, "chroma bilateral_filter_flag");
  if (pcPPS->m_chromaBIF)
  {
    xWriteCode(pcPPS->m_chromaBIFStrength, 2, "chroma bilateral_filter_strength");
    xWriteSvlc(pcPPS->m_chromaBIFQPOffset, "chroma bilateral_filter_qp_offset");
  }

  xWriteFlag(pcPPS->m_deblockingFilterControlPresentFlag ? 1 : 0, "pps_deblocking_filter_control_present_flag");
  if (pcPPS->m_deblockingFilterControlPresentFlag)
  {
    xWriteFlag(pcPPS->m_deblockingFilterOverrideEnabledFlag ? 1 : 0, "pps_deblocking_filter_override_enabled_flag");
    xWriteFlag(pcPPS->m_ppsDeblockingFilterDisabledFlag ? 1 : 0, "pps_deblocking_filter_disabled_flag");
    if (!pcPPS->m_noPicPartitionFlag && pcPPS->m_deblockingFilterOverrideEnabledFlag)
    {
      xWriteFlag(pcPPS->m_dbfInfoInPhFlag ? 1 : 0, "pps_dbf_info_in_ph_flag");
    }
    if (!pcPPS->m_ppsDeblockingFilterDisabledFlag)
    {
      xWriteSvlc(pcPPS->m_deblockingFilterBetaOffsetDiv2, "pps_beta_offset_div2");
      xWriteSvlc(pcPPS->m_deblockingFilterTcOffsetDiv2, "pps_tc_offset_div2");
      if (pcPPS->m_usePPSChromaTool)
      {
        xWriteSvlc(pcPPS->m_deblockingFilterCbBetaOffsetDiv2, "pps_cb_beta_offset_div2");
        xWriteSvlc(pcPPS->m_deblockingFilterCbTcOffsetDiv2, "pps_cb_tc_offset_div2");
        xWriteSvlc(pcPPS->m_deblockingFilterCrBetaOffsetDiv2, "pps_cr_beta_offset_div2");
        xWriteSvlc(pcPPS->m_deblockingFilterCrTcOffsetDiv2, "pps_cr_tc_offset_div2");
      }
    }
  }
  if (!pcPPS->m_noPicPartitionFlag)
  {
    xWriteFlag(pcPPS->m_rplInfoInPhFlag ? 1 : 0, "pps_rpl_info_in_ph_flag");
    xWriteFlag(pcPPS->m_saoInfoInPhFlag ? 1 : 0, "pps_sao_info_in_ph_flag");
    xWriteFlag(pcPPS->m_alfInfoInPhFlag ? 1 : 0, "pps_alf_info_in_ph_flag");
    if ((pcPPS->m_useWP || pcPPS->m_useBiWP) && pcPPS->m_rplInfoInPhFlag)
    {
      xWriteFlag(pcPPS->m_wpInfoInPhFlag ? 1 : 0, "pps_wp_info_in_ph_flag");
    }
    xWriteFlag(pcPPS->m_qpDeltaInfoInPhFlag ? 1 : 0, "pps_qp_delta_info_in_ph_flag");
  }

  xWriteFlag(pcPPS->m_pictureHeaderExtensionPresentFlag ? 1 : 0, "pps_picture_header_extension_present_flag");
  xWriteFlag(pcPPS->m_sliceHeaderExtensionPresentFlag ? 1 : 0, "pps_slice_header_extension_present_flag");

  xWriteFlag(0, "pps_extension_flag");
  xWriteRbspTrailingBits();
}

void HLSWriter::codeAPS(const APS *pcAPS)
{
#if ENABLE_TRACING
  xTraceAPSHeader();
#endif

  xWriteCode((int)pcAPS->m_APSType, 3, "aps_params_type");
  xWriteCode(pcAPS->m_APSId, 5, "adaptation_parameter_set_id");
  xWriteFlag(pcAPS->chromaPresentFlag, "aps_chroma_present_flag");

  if (pcAPS->m_APSType == ApsType::ALF)
  {
    codeAlfAps(pcAPS);
  }
  else if (pcAPS->m_APSType == ApsType::LMCS)
  {
    codeLmcsAps(pcAPS);
  }
  else if (pcAPS->m_APSType == ApsType::SCALING_LIST)
  {
    codeScalingListAps(pcAPS);
  }
  xWriteFlag(0,
             "aps_extension_flag");   // Implementation when this flag is equal to 1 should be added when it is needed.
                                      // Currently in the spec we don't have case when this flag is equal to 1
  xWriteRbspTrailingBits();
}

void HLSWriter::codeAlfAps(const APS *pcAPS)
{
  const ApsAlfParam   &paramContainer      = pcAPS->m_alfAPSParam;
  const ApsCcAlfParam &paramCcAlfContainer = pcAPS->m_ccAlfAPSParam;

  // Get the parameters and check validity once. No further checks are required below.
  const AlfParameters::AlfParamBase         &param      = paramContainer.getParam();
  const AlfParameters::CcAlfFilterParamBase &paramCcAlf = paramCcAlfContainer.getParam();

  xWriteFlag(param.newFilterFlag[ChannelType::LUMA], "alf_luma_new_filter");
  if (pcAPS->chromaPresentFlag)
  {
    xWriteFlag(param.newFilterFlag[ChannelType::CHROMA], "alf_chroma_new_filter");
  }

  if (pcAPS->chromaPresentFlag)
  {
    xWriteFlag(paramCcAlf.newCcAlfFilter[COMP_Cb - 1], "alf_cc_cb_filter_signal_flag");
    xWriteFlag(paramCcAlf.newCcAlfFilter[COMP_Cr - 1], "alf_cc_cr_filter_signal_flag");
  }

  if (param.newFilterFlag[ChannelType::LUMA])
  {
    if (ApsAlfParam::isEcmAlf())
    {
      const AlfParametersEcm::AlfParam &param = paramContainer.getEcmParamNoCheck();

      if (AlfParametersEcm::ALF_MAX_NUM_ALTERNATIVES_LUMA > 1)
      {
        xWriteUvlc(param.numAlternativesLuma - 1, "alf_luma_num_alts_minus1");
      }

      xWriteFlag(
        param.filterType[ChannelType::LUMA] == AlfParametersEcm::AlfFilterType::ALF_FILTER_9_EXT_DB_RESI_DIRECT ? 1 : 0,
        "alf_luma_13_ext_db_resi_direct : alf_luma_13_ext_db_resi");

      for (int altIdx = 0; altIdx < param.numAlternativesLuma; ++altIdx)
      {
        xWriteFlag(param.lumaClassifierIdx[altIdx] == 1 ? 1 : 0, "alf_luma_classifier_band");
        if (param.lumaClassifierIdx[altIdx] != 1)
        {
          xWriteFlag(param.lumaClassifierIdx[altIdx] == 2 ? 1 : 0, "alf_luma_classifier_resi");
        }

        xWriteFlag(param.lumaNonLinearFlag[altIdx], "alf_luma_clip");
        xWriteUvlc(param.numLumaFilters[altIdx] - 1, "alf_luma_num_filters_signalled_minus1");
        if (param.numLumaFilters[altIdx] > 1)
        {
          const int length = ceilLog2(param.numLumaFilters[altIdx]);

          for (int i = 0; i < AlfParametersEcm::ALF_NUM_CLASSES_CLASSIFIER[(int)param.lumaClassifierIdx[altIdx]]; i++)
          {
            xWriteCode(param.filterCoeffDeltaIdx[altIdx][i], length, "alf_luma_coeff_delta_idx");
          }
        }

        AlfParametersEcm::AlfFilterType  alfFilterType = param.filterType[ChannelType::LUMA];
        AlfParametersEcm::AlfFilterShape filterShape(alfFilterType);

        alfFilter(param, false, altIdx);
      }
    }
    else
    {
      const AlfParametersVtm::AlfParam &param = paramContainer.getVtmParamNoCheck();

      xWriteFlag(param.lumaNonLinearFlag, "alf_luma_clip");

      xWriteUvlc(param.numLumaFilters - 1, "alf_luma_num_filters_signalled_minus1");
      if (param.numLumaFilters > 1)
      {
        const int length = ceilLog2(param.numLumaFilters);
        for (int i = 0; i < AlfParameters::MAX_NUM_ALF_CLASSES; i++)
        {
          xWriteCode(param.filterCoeffDeltaIdx[i], length, "alf_luma_coeff_delta_idx");
        }
      }
      alfFilter(param, false, 0);
    }
  }

  if (param.newFilterFlag[ChannelType::CHROMA])
  {
    if (ApsAlfParam::isVtmAlf())
    {
      const AlfParametersVtm::AlfParam &param = paramContainer.getVtmParamNoCheck();

      xWriteFlag(param.chromaNonLinearFlag, "alf_nonlinear_enable_flag_chroma");
    }

    if constexpr (AlfParameters::ALF_MAX_NUM_ALTERNATIVES_CHROMA > 1)
    {
      xWriteUvlc(param.numAlternativesChroma - 1, "alf_chroma_num_alts_minus1");
    }

    for (int altIdx = 0; altIdx < param.numAlternativesChroma; ++altIdx)
    {
      if (ApsAlfParam::isEcmAlf())
      {
        const AlfParametersEcm::AlfParam &param = paramContainer.getEcmParamNoCheck();

        xWriteFlag(param.chromaNonLinearFlag[altIdx], "alf_nonlinear_enable_flag_chroma");

        AlfParametersEcm::AlfFilterType  alfFilterType = param.filterType[ChannelType::CHROMA];
        AlfParametersEcm::AlfFilterShape filterShape(alfFilterType);

        alfFilter(param, true, altIdx);
      }
      else
      {
        const AlfParametersVtm::AlfParam &param = paramContainer.getVtmParamNoCheck();

        alfFilter(param, true, altIdx);
      }
    }
  }

  if (ApsCcAlfParam::isEcmAlf())
  {
    xCodeAlfApsCcAlf<AlfParametersEcm>(paramCcAlfContainer.getEcmParamNoCheck());
  }
  else
  {
    xCodeAlfApsCcAlf<AlfParametersVtm>(paramCcAlfContainer.getVtmParamNoCheck());
  }
}

template<class AlfParametersT>
void HLSWriter::xCodeAlfApsCcAlf(const typename AlfParametersT::CcAlfFilterParam &paramCcAlf)
{
  const int maxNumCcAlfFilters = AlfParametersT::MAX_NUM_CC_ALF_FILTERS;
  const int numCoeff = typename AlfParametersT::AlfFilterShape(AlfParametersT::AlfFilterType::CC_ALF).numCoeff;

  for (int ccIdx = 0; ccIdx < 2; ccIdx++)
  {
    if (paramCcAlf.newCcAlfFilter[ccIdx])
    {
      const int filterCount = paramCcAlf.ccAlfFilterCount[ccIdx];
      CHECK(filterCount > maxNumCcAlfFilters, "CC ALF Filter count is too large");
      CHECK(filterCount == 0, "CC ALF Filter count is too small");

      if (maxNumCcAlfFilters > 1)
      {
        xWriteUvlc(filterCount - 1,
                   ccIdx == 0 ? "alf_cc_cb_filters_signalled_minus1" : "alf_cc_cr_filters_signalled_minus1");
      }

      for (int filterIdx = 0; filterIdx < filterCount; filterIdx++)
      {
        const short *const coeff = paramCcAlf.ccAlfCoeff[ccIdx][filterIdx];

        // Filter coefficients
        for (int i = 0; i < numCoeff - 1; i++)
        {
          if (coeff[i] == 0)
          {
            xWriteCode(0, AlfParameters::CCALF_BITS_PER_COEFF_LEVEL,
                       ccIdx == 0 ? "alf_cc_cb_mapped_coeff_abs" : "alf_cc_cr_mapped_coeff_abs");
          }
          else
          {
            xWriteCode(1 + floorLog2(abs(coeff[i])), AlfParameters::CCALF_BITS_PER_COEFF_LEVEL,
                       ccIdx == 0 ? "alf_cc_cb_mapped_coeff_abs" : "alf_cc_cr_mapped_coeff_abs");
            xWriteFlag(coeff[i] < 0 ? 1 : 0, ccIdx == 0 ? "alf_cc_cb_coeff_sign" : "alf_cc_cr_coeff_sign");
          }
        }

        DTRACE(g_trace_ctx, D_SYNTAX, "%s coeff filterIdx %d: ", ccIdx == 0 ? "Cb" : "Cr", filterIdx);
        for (int i = 0; i < numCoeff; i++)
        {
          DTRACE(g_trace_ctx, D_SYNTAX, "%d ", coeff[i]);
        }
        DTRACE(g_trace_ctx, D_SYNTAX, "\n");
      }
    }
  }
}

void HLSWriter::codeLmcsAps(const APS *pcAPS)
{
  SliceReshapeInfo param = pcAPS->m_reshapeAPSInfo;
  xWriteUvlc(param.reshaperModelMinBinIdx, "lmcs_min_bin_idx");
  xWriteUvlc(PIC_CODE_CW_BINS - 1 - param.reshaperModelMaxBinIdx, "lmcs_delta_max_bin_idx");
  CHECKD(param.maxNbitsNeededDeltaCW < 1, "maxNbitsNeededDeltaCW must be equal to or greater than 1");
  xWriteUvlc(param.maxNbitsNeededDeltaCW - 1, "lmcs_delta_cw_prec_minus1");

  for (int i = param.reshaperModelMinBinIdx; i <= param.reshaperModelMaxBinIdx; i++)
  {
    int deltaCW = param.reshaperModelBinCWDelta[i];
    int signCW  = (deltaCW < 0) ? 1 : 0;
    int absCW   = (deltaCW < 0) ? (-deltaCW) : deltaCW;
    xWriteCode(absCW, param.maxNbitsNeededDeltaCW, "lmcs_delta_abs_cw[ i ]");
    if (absCW > 0)
    {
      xWriteFlag(signCW, "lmcs_delta_sign_cw_flag[ i ]");
    }
  }
  int deltaCRS = pcAPS->chromaPresentFlag ? param.chrResScalingOffset : 0;
  int signCRS  = (deltaCRS < 0) ? 1 : 0;
  int absCRS   = (deltaCRS < 0) ? (-deltaCRS) : deltaCRS;
  if (pcAPS->chromaPresentFlag)
  {
    xWriteCode(absCRS, 3, "lmcs_delta_abs_crs");
  }
  if (absCRS > 0)
  {
    xWriteFlag(signCRS, "lmcs_delta_sign_crs_flag");
  }
}

void HLSWriter::codeScalingListAps(const APS *pcAPS)
{
  ScalingList param = pcAPS->m_scalingListApsInfo;
  codeScalingList(param, pcAPS->chromaPresentFlag);
}

void HLSWriter::codeVUI(const VUI *pcVUI, const SPS *pcSPS)
{
#if ENABLE_TRACING
  if (g_HLSTraceEnable)
  {
    DTRACE(g_trace_ctx, D_HEADER, "----------- vui_parameters -----------\n");
  }
#endif

  xWriteFlag(pcVUI->m_progressiveSourceFlag, "vui_progressive_source_flag");
  xWriteFlag(pcVUI->m_interlacedSourceFlag, "vui_interlaced_source_flag");
  xWriteFlag(pcVUI->m_nonPackedFlag, "vui_non_packed_constraint_flag");
  xWriteFlag(pcVUI->m_nonProjectedFlag, "vui_non_projected_constraint_flag");
  xWriteFlag(pcVUI->m_aspectRatioInfoPresentFlag, "vui_aspect_ratio_info_present_flag");
  if (pcVUI->m_aspectRatioInfoPresentFlag)
  {
    xWriteFlag(pcVUI->m_aspectRatioConstantFlag, "vui_aspect_ratio_constant_flag");
    xWriteCode(pcVUI->m_aspectRatioIdc, 8, "vui_aspect_ratio_idc");
    if (pcVUI->m_aspectRatioIdc == 255)
    {
      xWriteCode(pcVUI->m_sarWidth, 16, "vui_sar_width");
      xWriteCode(pcVUI->m_sarHeight, 16, "vui_sar_height");
    }
  }
  xWriteFlag(pcVUI->m_overscanInfoPresentFlag, "vui_overscan_info_present_flag");
  if (pcVUI->m_overscanInfoPresentFlag)
  {
    xWriteFlag(pcVUI->m_overscanAppropriateFlag, "vui_overscan_appropriate_flag");
  }
  xWriteFlag(pcVUI->m_colourDescriptionPresentFlag, "vui_colour_description_present_flag");
  if (pcVUI->m_colourDescriptionPresentFlag)
  {
    xWriteCode(pcVUI->m_colourPrimaries, 8, "vui_colour_primaries");
    xWriteCode(pcVUI->m_transferCharacteristics, 8, "vui_transfer_characteristics");
    xWriteCode(pcVUI->m_matrixCoefficients, 8, "vui_matrix_coeffs");
    xWriteFlag(pcVUI->m_videoFullRangeFlag, "vui_full_range_flag");
  }
  xWriteFlag(pcVUI->m_chromaLocInfoPresentFlag, "vui_chroma_loc_info_present_flag");
  if (pcVUI->m_chromaLocInfoPresentFlag)
  {
    if (pcVUI->m_progressiveSourceFlag && !pcVUI->m_interlacedSourceFlag)
    {
      xWriteUvlc(pcVUI->m_chromaSampleLocType, "vui_chroma_sample_loc_type");
    }
    else
    {
      xWriteUvlc(pcVUI->m_chromaSampleLocTypeTopField, "vui_chroma_sample_loc_type_top_field");
      xWriteUvlc(pcVUI->m_chromaSampleLocTypeBottomField, "vui_chroma_sample_loc_type_bottom_field");
    }
  }
  if (!isByteAligned())
  {
    xWriteFlag(1, "vui_payload_bit_equal_to_one");
    while (!isByteAligned())
    {
      xWriteFlag(0, "vui_payload_bit_equal_to_zero");
    }
  }
}

void HLSWriter::codeGeneralHrdparameters(const GeneralHrdParams *hrd)
{
  xWriteCode(hrd->m_numUnitsInTick, 32, "num_units_in_tick");
  xWriteCode(hrd->m_timeScale, 32, "time_scale");
  xWriteFlag(hrd->m_generalNalHrdParamsPresentFlag ? 1 : 0, "general_nal_hrd_parameters_present_flag");
  xWriteFlag(hrd->m_generalVclHrdParamsPresentFlag ? 1 : 0, "general_vcl_hrd_parameters_present_flag");
  if (hrd->m_generalNalHrdParamsPresentFlag || hrd->m_generalVclHrdParamsPresentFlag)
  {
    xWriteFlag(hrd->m_generalSamePicTimingInAllOlsFlag ? 1 : 0, "general_same_pic_timing_in_all_ols_flag");
    xWriteFlag(hrd->m_generalDecodingUnitHrdParamsPresentFlag ? 1 : 0, "general_decoding_unit_hrd_params_present_flag");
    if (hrd->m_generalDecodingUnitHrdParamsPresentFlag)
    {
      xWriteCode(hrd->m_tickDivisorMinus2, 8, "tick_divisor_minus2");
    }
    xWriteCode(hrd->m_bitRateScale, 4, "bit_rate_scale");
    xWriteCode(hrd->m_cpbSizeScale, 4, "cpb_size_scale");
    if (hrd->m_generalDecodingUnitHrdParamsPresentFlag)
    {
      xWriteCode(hrd->m_cpbSizeDuScale, 4, "cpb_size_du_scale");
    }
    xWriteUvlc(hrd->m_hrdCpbCntMinus1, "hrd_cpb_cnt_minus1");
  }
}
void HLSWriter::codeOlsHrdParameters(const GeneralHrdParams *generalHrd, const OlsHrdParams *olsHrd,
                                     const uint32_t firstSubLayer, const uint32_t maxNumSubLayersMinus1)
{

  for (int i = firstSubLayer; i <= maxNumSubLayersMinus1; i++)
  {
    const OlsHrdParams *hrd = &(olsHrd[i]);
    xWriteFlag(hrd->m_fixedPicRateGeneralFlag, "fixed_pic_rate_general_flag");

    if (!hrd->m_fixedPicRateGeneralFlag)
    {
      xWriteFlag(hrd->m_fixedPicRateWithinCvsFlag, "fixed_pic_rate_within_cvs_flag");
    }
    if (hrd->m_fixedPicRateWithinCvsFlag)
    {
      xWriteUvlc(hrd->m_elementDurationInTcMinus1, "elemental_duration_in_tc_minus1");
    }
    else if ((generalHrd->m_generalNalHrdParamsPresentFlag || generalHrd->m_generalVclHrdParamsPresentFlag) &&
             generalHrd->m_hrdCpbCntMinus1 == 0)
    {
      xWriteFlag(hrd->m_lowDelayHrdFlag, "low_delay_hrd_flag");
    }

    for (int nalOrVcl = 0; nalOrVcl < 2; nalOrVcl++)
    {
      if (((nalOrVcl == 0) && (generalHrd->m_generalNalHrdParamsPresentFlag)) ||
          ((nalOrVcl == 1) && (generalHrd->m_generalVclHrdParamsPresentFlag)))
      {
        for (int j = 0; j <= (generalHrd->m_hrdCpbCntMinus1); j++)
        {
          xWriteUvlc(hrd->m_bitRateValueMinus1[j][nalOrVcl], "bit_rate_value_minus1");
          xWriteUvlc(hrd->m_cpbSizeValueMinus1[j][nalOrVcl], "cpb_size_value_minus1");
          if (generalHrd->m_generalDecodingUnitHrdParamsPresentFlag)
          {
            xWriteUvlc(hrd->m_ducpbSizeValueMinus1[j][nalOrVcl], "cpb_size_du_value_minus1");
            xWriteUvlc(hrd->m_duBitRateValueMinus1[j][nalOrVcl], "bit_rate_du_value_minus1");
          }
          xWriteFlag(hrd->m_cbrFlag[j][nalOrVcl], "cbr_flag");
        }
      }
    }
  }
}

void HLSWriter::dpb_parameters(int maxSubLayersMinus1, bool subLayerInfoFlag, const SPS *pcSPS)
{
  for (uint32_t i = (subLayerInfoFlag ? 0 : maxSubLayersMinus1); i <= maxSubLayersMinus1; i++)
  {
    CHECK(pcSPS->m_maxDecPicBuffering[i] < 1, "MaxDecPicBuffering must be greater than 0");
    xWriteUvlc(pcSPS->m_maxDecPicBuffering[i] - 1, "dpb_max_dec_pic_buffering_minus1[i]");
    xWriteUvlc(pcSPS->m_maxNumReorderPics[i], "dpb_max_num_reorder_pics[i]");
    xWriteUvlc(pcSPS->m_maxLatencyIncreasePlus1[i], "dpb_max_latency_increase_plus1[i]");
  }
}

void HLSWriter::codeSPS(const SPS *pcSPS)
{
#if ENABLE_TRACING
  xTraceSPSHeader();
#endif
  xWriteCode(pcSPS->m_spsId, 4, "sps_seq_parameter_set_id");
  xWriteCode(pcSPS->m_vpsId, 4, "sps_video_parameter_set_id");
  CHECK(pcSPS->m_maxSubLayers == 0, "Maximum number of temporal sub-layers is '0'");

  xWriteCode(pcSPS->m_maxSubLayers - 1, 3, "sps_max_sub_layers_minus1");
  xWriteCode(int(pcSPS->m_chromaFormatIdc), 2, "sps_chroma_format_idc");
  xWriteCode(floorLog2(pcSPS->m_ctuSize) - 6, 2, "sps_log2_ctu_size_minus6");
  xWriteFlag(pcSPS->m_ptlDpbHrdParamsPresentFlag, "sps_ptl_dpb_hrd_params_present_flag");

  if (!pcSPS->m_vpsId)
  {
    CHECK(!pcSPS->m_ptlDpbHrdParamsPresentFlag,
          "When sps_video_parameter_set_id is equal to 0, the value of sps_ptl_dpb_hrd_params_present_flag shall be "
          "equal to 1");
  }

  if (pcSPS->m_ptlDpbHrdParamsPresentFlag)
  {
    codeProfileTierLevel(&pcSPS->m_profileTierLevel, true, pcSPS->m_maxSubLayers - 1);
  }

  xWriteFlag(pcSPS->m_GDREnabledFlag, "sps_gdr_enabled_flag");

  xWriteFlag(pcSPS->m_rprEnabledFlag, "sps_ref_pic_resampling_enabled_flag");
  if (pcSPS->m_rprEnabledFlag)
  {
    xWriteFlag(pcSPS->m_resChangeInClvsEnabledFlag, "sps_res_change_in_clvs_allowed_flag");
  }
  CHECK(
    !pcSPS->m_rprEnabledFlag && pcSPS->m_resChangeInClvsEnabledFlag,
    "When sps_ref_pic_resampling_enabled_flag is equal to 0, sps_res_change_in_clvs_allowed_flag shall be equal to 0");

  xWriteUvlc(pcSPS->m_maxWidthInLumaSamples, "sps_pic_width_max_in_luma_samples");
  xWriteUvlc(pcSPS->m_maxHeightInLumaSamples, "sps_pic_height_max_in_luma_samples");
  Window conf = pcSPS->m_conformanceWindow;
  xWriteFlag(conf.m_enabledFlag, "sps_conformance_window_flag");
  if (conf.m_enabledFlag)
  {
    xWriteUvlc(conf.m_winLeftOffset, "sps_conf_win_left_offset");
    xWriteUvlc(conf.m_winRightOffset, "sps_conf_win_right_offset");
    xWriteUvlc(conf.m_winTopOffset, "sps_conf_win_top_offset");
    xWriteUvlc(conf.m_winBottomOffset, "sps_conf_win_bottom_offset");
  }

  xWriteFlag(pcSPS->m_subPicInfoPresentFlag, "sps_subpic_info_present_flag");

  if (pcSPS->m_subPicInfoPresentFlag)
  {
    CHECK(pcSPS->m_numSubPics < 1, "SPS: NumSubPics cannot be less than 1");
    xWriteUvlc(pcSPS->m_numSubPics - 1, "sps_num_subpics_minus1");
    if (pcSPS->m_numSubPics > 1)
    {
      xWriteFlag(pcSPS->m_independentSubPicsFlag, "sps_independent_subpics_flag");
      xWriteFlag(pcSPS->m_subPicSameSizeFlag, "sps_subpic_same_size_flag");
      uint32_t tmpWidthVal  = (pcSPS->m_maxWidthInLumaSamples + pcSPS->m_ctuSize - 1) / pcSPS->m_ctuSize;
      uint32_t tmpHeightVal = (pcSPS->m_maxHeightInLumaSamples + pcSPS->m_ctuSize - 1) / pcSPS->m_ctuSize;
      for (int picIdx = 0; picIdx < pcSPS->m_numSubPics; picIdx++)
      {
        if (!pcSPS->m_subPicSameSizeFlag || picIdx == 0)
        {
          if ((picIdx > 0) && (pcSPS->m_maxWidthInLumaSamples > pcSPS->m_ctuSize))
          {
            xWriteCode(pcSPS->m_subPicCtuTopLeftX[picIdx], ceilLog2(tmpWidthVal), "sps_subpic_ctu_top_left_x[ i ]");
          }
          if ((picIdx > 0) && (pcSPS->m_maxHeightInLumaSamples > pcSPS->m_ctuSize))
          {
            xWriteCode(pcSPS->m_subPicCtuTopLeftY[picIdx], ceilLog2(tmpHeightVal), "sps_subpic_ctu_top_left_y[ i ]");
          }
          if (picIdx < pcSPS->m_numSubPics - 1 && pcSPS->m_maxWidthInLumaSamples > pcSPS->m_ctuSize)
          {
            xWriteCode(pcSPS->m_subPicWidth[picIdx] - 1, ceilLog2(tmpWidthVal), "sps_subpic_width_minus1[ i ]");
          }
          if (picIdx < pcSPS->m_numSubPics - 1 && pcSPS->m_maxHeightInLumaSamples > pcSPS->m_ctuSize)
          {
            xWriteCode(pcSPS->m_subPicHeight[picIdx] - 1, ceilLog2(tmpHeightVal), "sps_subpic_height_minus1[ i ]");
          }
        }
        if (!pcSPS->m_independentSubPicsFlag)
        {
          xWriteFlag(pcSPS->m_subPicTreatedAsPicFlag[picIdx], "sps_subpic_treated_as_pic_flag[ i ]");
          xWriteFlag(pcSPS->m_loopFilterAcrossSubpicEnabledFlag[picIdx],
                     "sps_loop_filter_across_subpic_enabled_flag[ i ]");
        }
      }
    }

    CHECK(pcSPS->m_subPicIdLen < 1, "SPS: SubPicIdLen cannot be less than 1");
    xWriteUvlc(pcSPS->m_subPicIdLen - 1, "sps_subpic_id_len_minus1");
    xWriteFlag(pcSPS->m_subPicIdMappingExplicitlySignalledFlag, "sps_subpic_id_mapping_explicitly_signalled_flag");
    if (pcSPS->m_subPicIdMappingExplicitlySignalledFlag)
    {
      xWriteFlag(pcSPS->m_subPicIdMappingPresentFlag, "sps_subpic_id_mapping_present_flag");
      if (pcSPS->m_subPicIdMappingPresentFlag)
      {
        for (int picIdx = 0; picIdx < pcSPS->m_numSubPics; picIdx++)
        {
          xWriteCode(pcSPS->m_subPicId[picIdx], pcSPS->m_subPicIdLen, "sps_subpic_id[i]");
        }
      }
    }
  }

  const Profile::Name profile = pcSPS->m_profileTierLevel.m_profileIdc;
  if (profile != Profile::NONE)
  {
    CHECK(pcSPS->m_bitDepths[ChannelType::LUMA] > ProfileFeatures::getProfileFeatures(profile)->maxBitDepth,
          "sps_bitdepth_minus8 exceeds range supported by signalled profile");
  }
  xWriteUvlc(pcSPS->m_bitDepths[ChannelType::LUMA] - 8, "sps_bitdepth_minus8");
  xWriteFlag(pcSPS->m_entropyCodingSyncEnabledFlag ? 1 : 0, "sps_entropy_coding_sync_enabled_flag");
  xWriteFlag(pcSPS->m_entryPointPresentFlag ? 1 : 0, "sps_entry_point_offsets_present_flag");
  xWriteCode(pcSPS->m_bitsForPoc - 4, 4, "sps_log2_max_pic_order_cnt_lsb_minus4");

  xWriteFlag(pcSPS->m_pocMsbCycleFlag ? 1 : 0, "sps_poc_msb_cycle_flag");
  if (pcSPS->m_pocMsbCycleFlag)
  {
    xWriteUvlc(pcSPS->m_pocMsbCycleLen - 1, "sps_poc_msb_cycle_len_minus1");
  }
  // extra bits are for future extensions, so these are currently hard coded to not being sent
  xWriteCode(0, 2, "sps_num_extra_ph_bytes");
  // for( i = 0; i < (sps_num_extra_ph_bytes * 8 ); i++ )
  //   sps_extra_ph_bit_present_flag[ i ]
  xWriteCode(0, 2, "sps_num_extra_sh_bytes");
  // for( i = 0; i < (sps_num_extra_sh_bytes * 8 ); i++ )
  //   sps_extra_sh_bit_present_flag[ i ]

  if (pcSPS->m_ptlDpbHrdParamsPresentFlag)
  {
    if (pcSPS->m_maxSubLayers - 1 > 0)
    {
      xWriteFlag(pcSPS->m_subLayerDpbParamsFlag, "sps_sublayer_dpb_params_flag");
    }
    dpb_parameters(pcSPS->m_maxSubLayers - 1, pcSPS->m_subLayerDpbParamsFlag, pcSPS);
  }
  CHECK(pcSPS->m_maxCuWidth != pcSPS->m_maxCuHeight, "Rectangular CTUs not supported");
  xWriteUvlc(pcSPS->m_log2MinCodingBlockSize - 2, "sps_log2_min_luma_coding_block_size_minus2");
  xWriteFlag(pcSPS->m_partitionOverrideEnabled, "sps_partition_constraints_override_enabled_flag");
  xWriteUvlc(floorLog2(pcSPS->getMinQTSize(I_SLICE)) - pcSPS->m_log2MinCodingBlockSize,
             "sps_log2_diff_min_qt_min_cb_intra_slice_luma");
  xWriteUvlc(pcSPS->getMaxMTTHierarchyDepthI(), "sps_max_mtt_hierarchy_depth_intra_slice_luma");
  if (pcSPS->getMaxMTTHierarchyDepthI() != 0)
  {
    xWriteUvlc(floorLog2(pcSPS->getMaxBTSizeI()) - floorLog2(pcSPS->getMinQTSize(I_SLICE)),
               "sps_log2_diff_max_bt_min_qt_intra_slice_luma");
    xWriteUvlc(floorLog2(pcSPS->getMaxTTSizeI()) - floorLog2(pcSPS->getMinQTSize(I_SLICE)),
               "sps_log2_diff_max_tt_min_qt_intra_slice_luma");
  }
  if (isChromaEnabled(pcSPS->m_chromaFormatIdc))
  {
    xWriteFlag(pcSPS->m_dualITree, "sps_qtbtt_dual_tree_intra_flag");
  }
  if (pcSPS->m_dualITree)
  {
    xWriteUvlc(floorLog2(pcSPS->getMinQTSize(I_SLICE, ChannelType::CHROMA)) - pcSPS->m_log2MinCodingBlockSize,
               "sps_log2_diff_min_qt_min_cb_intra_slice_chroma");
    xWriteUvlc(pcSPS->getMaxMTTHierarchyDepthIChroma(), "sps_max_mtt_hierarchy_depth_intra_slice_chroma");
    if (pcSPS->getMaxMTTHierarchyDepthIChroma() != 0)
    {
      xWriteUvlc(floorLog2(pcSPS->getMaxBTSizeIChroma()) - floorLog2(pcSPS->getMinQTSize(I_SLICE, ChannelType::CHROMA)),
                 "sps_log2_diff_max_bt_min_qt_intra_slice_chroma");
      xWriteUvlc(floorLog2(pcSPS->getMaxTTSizeIChroma()) - floorLog2(pcSPS->getMinQTSize(I_SLICE, ChannelType::CHROMA)),
                 "sps_log2_diff_max_tt_min_qt_intra_slice_chroma");
    }
  }
  xWriteUvlc(floorLog2(pcSPS->getMinQTSize(B_SLICE)) - pcSPS->m_log2MinCodingBlockSize,
             "sps_log2_diff_min_qt_min_cb_inter_slice");
  xWriteUvlc(pcSPS->getMaxMTTHierarchyDepth(), "sps_max_mtt_hierarchy_depth_inter_slice");
  if (pcSPS->getMaxMTTHierarchyDepth() != 0)
  {
    xWriteUvlc(floorLog2(pcSPS->getMaxBTSize()) - floorLog2(pcSPS->getMinQTSize(B_SLICE)),
               "sps_log2_diff_max_bt_min_qt_inter_slice");
    xWriteUvlc(floorLog2(pcSPS->getMaxTTSize()) - floorLog2(pcSPS->getMinQTSize(B_SLICE)),
               "sps_log2_diff_max_tt_min_qt_inter_slice");
  }
  if (pcSPS->m_ctuSize > 32)
  {
    xWriteUvlc(pcSPS->m_log2MaxTbSize - 5, "sps_log2_max_luma_transform_size_minus5");
  }

  xWriteFlag(pcSPS->m_transformSkipEnabledFlag ? 1 : 0, "sps_transform_skip_enabled_flag");
  if (pcSPS->m_transformSkipEnabledFlag)
  {
    xWriteUvlc(pcSPS->m_log2MaxTransformSkipBlockSize - 2, "sps_log2_transform_skip_max_size_minus2");
    xWriteFlag(pcSPS->m_bdpcmEnabledFlag ? 1 : 0, "sps_bdpcm_enabled_flag");
  }
  else
  {
    CHECK(pcSPS->m_bdpcmEnabledFlag, "BDPCM cannot be used when transform skip is disabled");
  }
  xWriteFlag(pcSPS->m_mtsEnabled ? 1 : 0, "sps_mts_enabled_flag");
  if (pcSPS->m_mtsEnabled)
  {
    xWriteFlag(pcSPS->m_explicitMtsIntra ? 1 : 0, "sps_explicit_mts_intra_enabled_flag");
    xWriteFlag(pcSPS->m_explicitMtsInter ? 1 : 0, "sps_explicit_mts_inter_enabled_flag");
    if (pcSPS->m_explicitMtsInter)
    {
      CHECK((pcSPS->m_interMTSMaxSize != 16 && pcSPS->m_interMTSMaxSize != 32), "interMTSMaxSize != 32 or 16");
      xWriteFlag(pcSPS->m_interMTSMaxSize == 16 ? 1 : 0, "sps_inter_mts_max_size");
    }
  }
  xWriteFlag(pcSPS->m_useIntraLFNSTinISlice ? 1 : 0, "sps_intra_lfnst_intra_slice_enabled_flag");
  xWriteFlag(pcSPS->m_useIntraLFNSTinPBSlice ? 1 : 0, "sps_intra_lfnst_inter_slice_enabled_flag");
  xWriteFlag(pcSPS->m_useInterLFNST ? 1 : 0, "sps_inter_lfnst_enabled_flag");
  if (pcSPS->m_useInterLFNST)
  {
    xWriteFlag(pcSPS->m_useInterLFNSTSBT ? 1 : 0, "sps_inter_lfnst_sbt_enabled_flag");
  }

  if (isChromaEnabled(pcSPS->m_chromaFormatIdc))
  {
    xWriteFlag(pcSPS->m_jointCbCrEnabledFlag, "sps_joint_cbcr_enabled_flag");
    const ChromaQpMappingTable &chromaQpMappingTable = pcSPS->m_chromaQpMappingTable;
    xWriteFlag(chromaQpMappingTable.m_sameCQPTableForAllChromaFlag, "sps_same_qp_table_for_chroma_flag");
    int numQpTables = chromaQpMappingTable.m_sameCQPTableForAllChromaFlag ? 1 : (pcSPS->m_jointCbCrEnabledFlag ? 3 : 2);
    CHECK(numQpTables != chromaQpMappingTable.m_numQpTables, " numQpTables does not match at encoder side ");
    for (int i = 0; i < numQpTables; i++)
    {
      xWriteSvlc(chromaQpMappingTable.m_qpTableStartMinus26[i], "sps_qp_table_starts_minus26");
      xWriteUvlc(chromaQpMappingTable.m_numPtsInCQPTableMinus1[i], "sps_num_points_in_qp_table_minus1");

      for (int j = 0; j <= chromaQpMappingTable.m_numPtsInCQPTableMinus1[i]; j++)
      {
        xWriteUvlc(chromaQpMappingTable.m_deltaQpInValMinus1[i][j], "sps_delta_qp_in_val_minus1");
        xWriteUvlc(chromaQpMappingTable.m_deltaQpOutVal[i][j] ^ chromaQpMappingTable.m_deltaQpInValMinus1[i][j],
                   "sps_delta_qp_diff_val");
      }
    }
  }

#if ENABLE_NNLF
  xWriteUvlc(pcSPS->m_nnlf, "sps_nnlf_id");
#endif

  xWriteFlag(pcSPS->m_saoEnabledFlag, "sps_sao_enabled_flag");
  xWriteFlag(pcSPS->m_ccSaoEnabledFlag, "sps_ccsao_enabled_flag");
  xWriteFlag(pcSPS->m_ccSaoFastFlag, "sps_ccsao_fast_flag");
  xWriteFlag(pcSPS->m_lfCccmEnabledFlag, "sps_lfcccm_enabled_flag");
  xWriteFlag(pcSPS->m_alfEnabledFlag, "sps_alf_enabled_flag");
  if (pcSPS->m_alfEnabledFlag)
  {
    xWriteFlag(pcSPS->m_alfImprovementsEnabledFlag, "sps_alf_improvements_enabled_flag");
  }
  if (pcSPS->m_alfEnabledFlag && isChromaEnabled(pcSPS->m_chromaFormatIdc))
  {
    xWriteFlag(pcSPS->m_ccalfEnabledFlag, "sps_ccalf_enabled_flag");
  }
  xWriteCode(pcSPS->m_numPredSign, 4, "num_predicted_coef_signs");
  if (pcSPS->m_numPredSign)
  {
    xWriteCode(pcSPS->m_log2SignPredArea - 2, 2, "log2_sign_pred_area_minus2");
  }
  xWriteFlag(pcSPS->m_tempCabacInitMode ? 1 : 0, "temp_cabac_init_mode");
  xWriteFlag(pcSPS->m_lmcsEnabled ? 1 : 0, "sps_lmcs_enable_flag");
  xWriteFlag(pcSPS->m_useWP ? 1 : 0, "sps_weighted_pred_flag");   // Use of Weighting Prediction (P_SLICE)
  xWriteFlag(pcSPS->m_useBiWP ? 1 : 0, "sps_weighted_bipred_flag");   // Use of Weighting Bi-Prediction (B_SLICE)

  xWriteFlag(pcSPS->m_longTermRefsPresent ? 1 : 0, "sps_long_term_ref_pics_flag");
  if (pcSPS->m_vpsId > 0)
  {
    xWriteFlag(pcSPS->m_interLayerPresentFlag ? 1 : 0, "sps_inter_layer_prediction_enabled_flag");
  }
  xWriteFlag(pcSPS->m_idrRefParamList ? 1 : 0, "sps_idr_rpl_present_flag");
  xWriteFlag(pcSPS->m_rpl1CopyFromRpl0Flag ? 1 : 0, "sps_rpl1_same_as_rpl0_flag");

  xWriteFlag(pcSPS->m_useInterRPL, "sps_inter_rpl_flag");
  if (pcSPS->m_useInterRPL)
  {
    const int numberOfRPL = pcSPS->m_numRpl[RPL0];
    xWriteUvlc(numberOfRPL - 2, "sps_inter_rpl_num_ref_pic_lists_minus2");
    xCodeRefPicListInterRPL(pcSPS, numberOfRPL);
  }
  else
  {
    for (const auto l: { RPL0, RPL1 })
    {
      if (l == RPL1 && pcSPS->m_rpl1CopyFromRpl0Flag)
      {
        continue;
      }
      const RPLList *rplList = &pcSPS->m_rplList[l];
      const int      numRpl  = pcSPS->m_numRpl[l];
      xWriteUvlc(numRpl, l == RPL0 ? "sps_num_ref_pic_lists[0]" : "sps_num_ref_pic_lists[1]");

      for (int rplIdx = 0; rplIdx < numRpl; rplIdx++)
      {
        const ReferencePictureList *rpl = rplList->getReferencePictureList(rplIdx);
        xCodeRefPicList(rpl, pcSPS->m_longTermRefsPresent, pcSPS->m_bitsForPoc, !pcSPS->m_useWP && !pcSPS->m_useBiWP,
                        rplIdx);
      }
    }
  }

  xWriteFlag(pcSPS->m_wrapAroundEnabledFlag ? 1 : 0, "sps_ref_wraparound_enabled_flag");

  xWriteFlag(pcSPS->m_temporalMvpEnabledFlag ? 1 : 0, "sps_temporal_mvp_enabled_flag");

  if (pcSPS->m_temporalMvpEnabledFlag)
  {
    xWriteFlag(pcSPS->m_sbtmvpEnabledFlag ? 1 : 0, "sps_sbtmvp_enabled_flag");
  }

  xWriteFlag(pcSPS->m_AMVREnabledFlag ? 1 : 0, "sps_amvr_enabled_flag");

  xWriteFlag(pcSPS->m_bdofEnabledFlag ? 1 : 0, "sps_bdof_enabled_flag");
  if (pcSPS->m_bdofEnabledFlag)
  {
    xWriteFlag(pcSPS->m_bdofControlPresentInPhFlag ? 1 : 0, "sps_bdof_control_present_in_ph_flag");
  }
  xWriteFlag(pcSPS->m_useSMVD ? 1 : 0, "sps_smvd_enabled_flag");
  xWriteFlag(pcSPS->m_useMMVD ? 1 : 0, "sps_mmvd_enabled_flag");
  if (pcSPS->m_useMMVD)
  {
    xWriteFlag(pcSPS->m_fpelMmvdEnabledFlag ? 1 : 0, "sps_mmvd_fullpel_only_flag");
  }
  xWriteUvlc(MRG_MAX_NUM_CANDS - pcSPS->m_maxNumMergeCand, "sps_six_minus_max_num_merge_cand");
  xWriteFlag(pcSPS->m_useSBT ? 1 : 0, "sps_sbt_enabled_flag");
  xWriteFlag(pcSPS->m_useDMVD ? 1 : 0, "sps_dmvd_enabled_flag");
  if (pcSPS->m_useDMVD && pcSPS->m_bdofEnabledFlag)
  {
    xWriteFlag(pcSPS->m_dmvdBDOFExt ? 1 : 0, "sps_bdof_ext_enabled_flag");
  }
  xWriteUvlc(BM_MRG_MAX_NUM_CANDS - pcSPS->m_maxNumBMMergeCand, "sps_six_minus_max_num_merge_cand");
  xWriteFlag(pcSPS->m_useAffine ? 1 : 0, "sps_affine_enabled_flag");
  if (pcSPS->m_useAffine)
  {
    xWriteUvlc(AFFINE_MRG_MAX_NUM_CANDS - pcSPS->m_maxNumAffineMergeCand, "sps_five_minus_max_num_subblock_merge_cand");
    xWriteFlag(pcSPS->m_AffineType ? 1 : 0, "sps_affine_type_flag");
    xWriteFlag(pcSPS->m_AffineMmvdMode ? 1 : 0, "sps_affine_mmvd_enabled_flag");
    if (pcSPS->m_AMVREnabledFlag)
    {
      xWriteFlag(pcSPS->m_affineAmvrEnabledFlag ? 1 : 0, "sps_affine_amvr_enabled_flag");
    }
    xWriteUvlc(pcSPS->m_log2MinAffineBlkSizeMinus3, "sps_log2_min_affine_blocksize_minus_3");
    xWriteFlag(pcSPS->m_usePROF ? 1 : 0, "sps_affine_prof_enabled_flag");
    if (pcSPS->m_usePROF)
    {
      xWriteFlag(pcSPS->m_profControlPresentInPhFlag ? 1 : 0, "sps_prof_control_present_in_ph_flag");
    }
    xWriteFlag(pcSPS->m_affineParaRefinement ? 1 : 0, "sps_affine_nontranslation_parameter_refinement");
    xWriteFlag(pcSPS->m_affineSbMrgExt ? 1 : 0, "sps_affine_sub_block_merge_mode_extension");
  }

  xWriteFlag(pcSPS->m_useAdditionalCMVP ? 1 : 0, "sps_additionalCMVP_flag");
  xWriteFlag(pcSPS->m_useBcw ? 1 : 0, "sps_bcw_enabled_flag");

  xWriteFlag(pcSPS->m_useCiip ? 1 : 0, "sps_ciip_enabled_flag");
  if (pcSPS->m_maxNumMergeCand >= 2)
  {
    xWriteFlag(pcSPS->m_useGeo ? 1 : 0, "sps_gpm_enabled_flag");
    if (pcSPS->m_useGeo)
    {
      CHECK(pcSPS->m_maxNumMergeCand < pcSPS->m_maxNumGeoCand,
            "The number of GPM candidates must not be greater than the number of merge candidates");
      CHECK(2 > pcSPS->m_maxNumGeoCand, "The number of GPM candidates must not be smaller than 2");
      if (pcSPS->m_maxNumMergeCand >= 3)
      {
        xWriteUvlc(pcSPS->m_maxNumMergeCand - pcSPS->m_maxNumGeoCand, "sps_max_num_merge_cand_minus_max_num_gpm_cand");
      }
    }
  }
  xWriteFlag(pcSPS->m_mergeOppositeLic ? 1 : 0, "sps_oppositelic_merge_enabled_flag");
  if (pcSPS->m_mergeOppositeLic)
  {
    xWriteUvlc(REG_MRG_MAX_NUM_CANDS_OPPOSITELIC - pcSPS->m_maxNumOppositeLicMergeCand,
               "five_minus_max_num_oppositelic_merge_cand");
    if (pcSPS->m_useAffine)
    {
      xWriteUvlc(AFF_MRG_MAX_NUM_CANDS_OPPOSITELIC - pcSPS->m_maxNumAffineOppositeLicMergeCand,
                 "eight_minus_max_num_aff_oppolic_merge_cand");
    }
  }

  xWriteUvlc(pcSPS->m_log2ParallelMergeLevelMinus2, "sps_log2_parallel_merge_level_minus2");

  xWriteFlag(pcSPS->m_useMRL ? 1 : 0, "sps_mrl_enabled_flag");
  xWriteFlag(pcSPS->m_useMIP ? 1 : 0, "sps_mip_enabled_flag");
  xWriteFlag(pcSPS->m_pdpEnabledFlag ? 1 : 0, "sps_pdp_enabled_flag");
  xWriteFlag(pcSPS->m_usedirPlanar ? 1 : 0, "sps_dirPlanar_enabled_flag");
  xWriteFlag(pcSPS->m_useDIMD ? 1 : 0, "sps_dimd_enabled_flag");
  if (pcSPS->m_useDIMD)
  {
    xWriteFlag(pcSPS->m_useOBIC ? 1 : 0, "sps_obic_enabled_flag");
  }
  xWriteFlag(pcSPS->m_useTIMD ? 1 : 0, "sps_timd_enabled_flag");
  if (pcSPS->m_useTIMD)
  {
    xWriteFlag(pcSPS->m_useTIMDSAD ? 1 : 0, "sps_timd_sad_enabled_flag");
    xWriteFlag(pcSPS->m_useOFTIMD ? 1 : 0, "sps_of_timd_enabled_flag");
  }
  xWriteFlag(pcSPS->m_useEIP ? 1 : 0, "sps_eip_enabled_flag");
  xWriteFlag(pcSPS->m_useMMEIP ? 1 : 0, "sps_mmeip_enabled_flag");
  xWriteFlag(pcSPS->m_useSgpm ? 1 : 0, "sps_sgpm_enabled_flag");
  xWriteFlag(pcSPS->m_tempPartPredEnabledFlag ? 1 : 0, "sps_tempPartPred_enabled_flag");
  xWriteFlag(pcSPS->m_MCBP ? 1 : 0, "sps_mcbp_enabled_flag");
  xWriteFlag(pcSPS->m_TMBP ? 1 : 0, "sps_tmbp_enabled_flag");
  if (isChromaEnabled(pcSPS->m_chromaFormatIdc))
  {
    xWriteFlag(pcSPS->m_useDIMDChroma ? 1 : 0, "sps_dimd_chroma_enabled_flag");
    xWriteFlag(pcSPS->m_LMChroma ? 1 : 0, "sps_cclm_enabled_flag");
    if (pcSPS->m_LMChroma == 1)
    {
      xWriteFlag(pcSPS->m_CCCM ? 1 : 0, "sps_cccm_enabled_flag");
      if (pcSPS->m_CCCM)
      {
        xWriteFlag(pcSPS->m_bvgCccm ? 1 : 0, "sps_bvg_cccm");
        xWriteFlag(pcSPS->m_ccBoostTplRefSel ? 1 : 0, "sps_cc_boost_tpl_ref_sel");
        xWriteFlag(pcSPS->m_ccDecDerivedMode ? 1 : 0, "sps_cc_dec_der_mode");
      }
      xWriteFlag(pcSPS->m_ccBoostFilter ? 1 : 0, "sps_cc_boost_filter");
      xWriteFlag(pcSPS->m_ccMerge ? 1 : 0, "sps_cc_merge");
      if (pcSPS->m_ccMerge)
      {
        xWriteFlag(pcSPS->m_ccMergeFusion ? 1 : 0, "sps_cc_merge_fusion");
      }
    }
  }
  if (pcSPS->m_chromaFormatIdc == ChromaFormat::_420)
  {
    xWriteFlag(pcSPS->m_horCollocatedChromaFlag ? 1 : 0, "sps_chroma_horizontal_collocated_flag");
    xWriteFlag(pcSPS->m_verCollocatedChromaFlag ? 1 : 0, "sps_chroma_vertical_collocated_flag");
  }
  else
  {
    CHECK(pcSPS->m_horCollocatedChromaFlag != 1, "Invalid value for horizontal collocated chroma flag");
    CHECK(pcSPS->m_verCollocatedChromaFlag != 1, "Invalid value for vertical collocated chroma flag");
  }
  CHECK(pcSPS->m_maxNumMergeCand > MRG_MAX_NUM_CANDS, "More merge candidates signalled than supported");
  xWriteFlag(pcSPS->m_PLTMode ? 1 : 0, "sps_palette_enabled_flag");
  if (pcSPS->m_transformSkipEnabledFlag || pcSPS->m_PLTMode)
  {
    xWriteUvlc(pcSPS->m_internalMinusInputBitDepth[ChannelType::LUMA], "sps_internal_bit_depth_minus_input_bit_depth");
  }
  xWriteFlag(pcSPS->m_ibcFlag ? 1 : 0, "sps_ibc_enabled_flag");
  if (pcSPS->m_ibcFlag)
  {
    if (pcSPS->m_AMVREnabledFlag)
    {
      xWriteFlag(pcSPS->m_ibcFracFlag ? 1 : 0, "sps_ibc_frac_enabled_flag");
    }
    xWriteFlag(pcSPS->m_ibcFlagInterSlice ? 1 : 0, "sps_ibc_enabled_flag_inter_slice");
    xWriteFlag(pcSPS->m_ibcMerge ? 1 : 0, "sps_ibc_merge_enabled_flag");
    if (pcSPS->m_ibcMerge)
    {
      CHECK(pcSPS->m_maxNumIBCMergeCand > IBC_MRG_MAX_NUM_CANDS, "More IBC merge candidates signalled than supported");
      xWriteUvlc(IBC_MRG_MAX_NUM_CANDS - pcSPS->m_maxNumIBCMergeCand, "sps_six_minus_max_num_ibc_merge_cand");
    }
  }
  xWriteFlag(pcSPS->m_ladfEnabled ? 1 : 0, "sps_ladf_enabled_flag");
  if (pcSPS->m_ladfEnabled)
  {
    xWriteCode(pcSPS->m_ladfNumIntervals - 2, 2, "sps_num_ladf_intervals_minus2");
    xWriteSvlc(pcSPS->m_ladfQpOffset[0], "sps_ladf_lowest_interval_qp_offset");
    for (int k = 1; k < pcSPS->m_ladfNumIntervals; k++)
    {
      xWriteSvlc(pcSPS->m_ladfQpOffset[k], "sps_ladf_qp_offset");
      xWriteUvlc(pcSPS->m_ladfIntervalLowerBound[k] - pcSPS->m_ladfIntervalLowerBound[k - 1] - 1,
                 "sps_ladf_delta_threshold_minus1");
    }
  }
  // KJS: reference picture sets to be replaced

  // KJS: remove scaling lists?
  xWriteFlag(pcSPS->m_scalingListEnabledFlag ? 1 : 0, "sps_explicit_scaling_list_enabled_flag");

  if ((pcSPS->m_useIntraLFNSTinISlice || pcSPS->m_useIntraLFNSTinPBSlice || pcSPS->m_useInterLFNST) &&
      pcSPS->m_scalingListEnabledFlag)
  {
    xWriteFlag(pcSPS->m_disableScalingMatrixForLfnstBlks, "sps_scaling_matrix_for_lfnst_disabled_flag");
  }

  xWriteFlag(pcSPS->m_depQuantEnabledFlag, "sps_dep_quant_enabled_flag");
  xWriteFlag(pcSPS->m_signDataHidingEnabledFlag, "sps_sign_data_hiding_enabled_flag");

  xWriteFlag(pcSPS->m_useObmc, "sps_obmc_flag");

  xWriteFlag(pcSPS->m_licEnabledFlag, "sps_lic_enabled_flag");
  if (pcSPS->m_licEnabledFlag)
  {
    xWriteFlag(pcSPS->m_biLicEnabledFlag, "sps_bi_lic_enabled_flag");
  }

  if (pcSPS->m_ptlDpbHrdParamsPresentFlag)
  {
    xWriteFlag(pcSPS->m_generalHrdParametersPresentFlag, "sps_timing_hrd_params_present_flag");
    if (pcSPS->m_generalHrdParametersPresentFlag)
    {
      codeGeneralHrdparameters(&pcSPS->m_generalHrdParams);
      if ((pcSPS->m_maxSubLayers - 1) > 0)
      {
        xWriteFlag(pcSPS->m_SubLayerCbpParametersPresentFlag, "sps_sublayer_cpb_params_present_flag");
      }
      uint32_t firstSubLayer = pcSPS->m_SubLayerCbpParametersPresentFlag ? 0 : (pcSPS->m_maxSubLayers - 1);
      codeOlsHrdParameters(&pcSPS->m_generalHrdParams, pcSPS->m_olsHrdParams, firstSubLayer, pcSPS->m_maxSubLayers - 1);
    }
  }

  xWriteFlag(pcSPS->m_fieldSeqFlag, "sps_field_seq_flag");
  xWriteFlag(pcSPS->m_vuiParametersPresentFlag, "sps_vui_parameters_present_flag");
  if (pcSPS->m_vuiParametersPresentFlag)
  {
    OutputBitstream *bs = getBitstream();
    OutputBitstream  bsCount;
    setBitstream(&bsCount);
#if ENABLE_TRACING
    bool traceEnable = g_HLSTraceEnable;
    g_HLSTraceEnable = false;
#endif
    codeVUI(&pcSPS->m_vuiParameters, pcSPS);
#if ENABLE_TRACING
    g_HLSTraceEnable = traceEnable;
#endif
    unsigned vui_payload_data_num_bits = bsCount.getNumberOfWrittenBits();
    CHECK(vui_payload_data_num_bits % 8 != 0, "Invalid number of VUI payload data bits");
    setBitstream(bs);
    xWriteUvlc((vui_payload_data_num_bits >> 3) - 1, "sps_vui_payload_size_minus1");
    while (!isByteAligned())
    {
      xWriteFlag(0, "sps_vui_alignment_zero_bit");
    }
    codeVUI(&pcSPS->m_vuiParameters, pcSPS);
  }

  bool sps_extension_present_flag                   = false;
  bool sps_extension_flags[NUM_SPS_EXTENSION_FLAGS] = { false };

  sps_extension_flags[SPS_EXT__REXT] = pcSPS->m_spsRangeExtension.settingsDifferFromDefaults();

  // Other SPS extension flags checked here.

  for (int i = 0; i < NUM_SPS_EXTENSION_FLAGS; i++)
  {
    sps_extension_present_flag |= sps_extension_flags[i];
  }

  xWriteFlag((sps_extension_present_flag ? 1 : 0), "sps_extension_present_flag");

  if (sps_extension_present_flag)
  {
    static const char *syntaxStrings[] = { "sps_range_extension_flag", "sps_extension_7bits[0]",
                                           "sps_extension_7bits[1]",   "sps_extension_7bits[2]",
                                           "sps_extension_7bits[3]",   "sps_extension_7bits[4]",
                                           "sps_extension_7bits[5]",   "sps_extension_7bits[6]" };

    if (pcSPS->m_bitDepths[ChannelType::LUMA] <= 10)
    {
      CHECK((sps_extension_flags[SPS_EXT__REXT] == 1),
            "The value of sps_range_extension_flag shall be 0 when BitDepth is less than or equal to 10.");
    }

    for (int i = 0; i < NUM_SPS_EXTENSION_FLAGS; i++)
    {
      xWriteFlag(sps_extension_flags[i] ? 1 : 0, syntaxStrings[i]);
    }

    for (int i = 0; i < NUM_SPS_EXTENSION_FLAGS; i++)   // loop used so that the order is determined by the enum.
    {
      if (sps_extension_flags[i])
      {
        switch (SPSExtensionFlagIndex(i))
        {
        case SPS_EXT__REXT:
          {
            const SPSRExt &spsRangeExtension = pcSPS->m_spsRangeExtension;

            xWriteFlag((spsRangeExtension.m_extendedPrecisionProcessingFlag ? 1 : 0),
                       "extended_precision_processing_flag");
            if (pcSPS->m_transformSkipEnabledFlag)
            {
              xWriteFlag((spsRangeExtension.m_tsrcRicePresentFlag ? 1 : 0),
                         "sps_ts_residual_coding_rice_present_in_sh_flag");
            }
            xWriteFlag((spsRangeExtension.m_rrcRiceExtensionEnableFlag ? 1 : 0), "rrc_rice_extension_flag");
            xWriteFlag((spsRangeExtension.m_persistentRiceAdaptationEnabledFlag ? 1 : 0),
                       "persistent_rice_adaptation_enabled_flag");
            xWriteFlag((spsRangeExtension.m_reverseLastSigCoeffEnabledFlag ? 1 : 0),
                       "reverse_last_sig_coeff_enabled_flag");
            break;
          }
        default:
          CHECK(sps_extension_flags[i] != false,
                "Unknown PPS extension signalled");   // Should never get here with an active SPS extension flag.
          break;
        }
      }
    }
  }
  xWriteRbspTrailingBits();
}

void HLSWriter::codeDCI(const DCI *dci)
{
#if ENABLE_TRACING
  xTraceDCIHeader();
#endif
  xWriteCode(0, 4, "dci_reserved_zero_4bits");
  uint32_t numPTLs = (uint32_t)dci->getNumPTLs();
  CHECK((numPTLs < 1) || (numPTLs > 15), "dci_num_plts_minus1 shall be in the range of 0 - 14");

  xWriteCode(numPTLs - 1, 4, "dci_num_ptls_minus1");

  for (int i = 0; i < numPTLs; i++)
  {
    ProfileTierLevel ptl = dci->m_profileTierLevel[i];
    codeProfileTierLevel(&ptl, true, 0);
  }

  xWriteFlag(0, "dci_extension_flag");
  xWriteRbspTrailingBits();
}

void HLSWriter::codeOPI(const OPI *opi)
{
#if ENABLE_TRACING
  xTraceOPIHeader();
#endif
  xWriteFlag(opi->m_olsinfopresentflag, "opi_ols_info_present_flag");
  xWriteFlag(opi->m_htidinfopresentflag, "opi_htid_info_present_flag");

  if (opi->m_olsinfopresentflag)
  {
    xWriteUvlc(opi->m_opiolsidx, "opi_ols_idx");
  }

  if (opi->m_htidinfopresentflag)
  {
    xWriteCode(opi->m_opihtidplus1, 3, "opi_htid_plus1");
  }
  xWriteFlag(0, "opi_extension_flag");
  xWriteRbspTrailingBits();
}

void HLSWriter::codeVPS(const VPS *pcVPS)
{
#if ENABLE_TRACING
  xTraceVPSHeader();
#endif
  xWriteCode(pcVPS->m_vpsId, 4, "vps_video_parameter_set_id");
  xWriteCode(pcVPS->m_maxLayers - 1, 6, "vps_max_layers_minus1");
  xWriteCode(pcVPS->m_vpsMaxSubLayers - 1, 3, "vps_max_sublayers_minus1");
  if (pcVPS->m_maxLayers > 1 && pcVPS->m_vpsMaxSubLayers > 1)
  {
    xWriteFlag(pcVPS->m_vpsDefaultPtlDpbHrdMaxTidFlag, "vps_default_ptl_dpb_hrd_max_tid_flag");
  }
  if (pcVPS->m_maxLayers > 1)
  {
    xWriteFlag(pcVPS->m_vpsAllIndependentLayersFlag, "vps_all_independent_layers_flag");
  }
  for (uint32_t i = 0; i < pcVPS->m_maxLayers; i++)
  {
    xWriteCode(pcVPS->m_vpsLayerId[i], 6, "vps_layer_id");
    if (i > 0 && !pcVPS->m_vpsAllIndependentLayersFlag)
    {
      xWriteFlag(pcVPS->m_vpsIndependentLayerFlag[i], "vps_independent_layer_flag");
      if (!pcVPS->m_vpsIndependentLayerFlag[i])
      {
        bool presentFlag = false;
        for (int j = 0; j < i; j++)
        {
          presentFlag |= ((pcVPS->getMaxTidIlRefPicsPlus1(i, j) != MAX_TLAYER) && pcVPS->m_vpsDirectRefLayerFlag[i][j]);
        }
        xWriteFlag(presentFlag, "max_tid_ref_present_flag[ i ]");
        for (int j = 0; j < i; j++)
        {
          xWriteFlag(pcVPS->m_vpsDirectRefLayerFlag[i][j], "vps_direct_ref_layer_flag");
          if (presentFlag && pcVPS->m_vpsDirectRefLayerFlag[i][j])
          {
            xWriteCode(pcVPS->getMaxTidIlRefPicsPlus1(i, j), 3, "max_tid_il_ref_pics_plus1[ i ][ j ]");
          }
        }
      }
    }
  }
  if (pcVPS->m_maxLayers > 1)
  {
    if (pcVPS->m_vpsAllIndependentLayersFlag)
    {
      xWriteFlag(pcVPS->m_vpsEachLayerIsAnOlsFlag, "vps_each_layer_is_an_ols_flag");
    }
    if (!pcVPS->m_vpsEachLayerIsAnOlsFlag)
    {
      if (!pcVPS->m_vpsAllIndependentLayersFlag)
      {
        xWriteCode(pcVPS->m_vpsOlsModeIdc, 2, "vps_ols_mode_idc");
      }
      if (pcVPS->m_vpsOlsModeIdc == 2)
      {
        xWriteCode(pcVPS->m_vpsNumOutputLayerSets - 2, 8, "vps_num_output_layer_sets_minus2");
        for (uint32_t i = 1; i < pcVPS->m_vpsNumOutputLayerSets; i++)
        {
          for (uint32_t j = 0; j < pcVPS->m_maxLayers; j++)
          {
            xWriteFlag(pcVPS->m_vpsOlsOutputLayerFlag[i][j], "vps_ols_output_layer_flag");
          }
        }
      }
    }
    CHECK(pcVPS->getNumPtls() - 1 >= pcVPS->m_totalNumOLSs, "vps_num_ptls_minus1 shall be less than TotalNumOlss");
    xWriteCode(pcVPS->getNumPtls() - 1, 8, "vps_num_ptls_minus1");
  }

  int totalNumOlss = pcVPS->m_totalNumOLSs;
  for (int i = 0; i < pcVPS->getNumPtls(); i++)
  {
    if (i > 0)
    {
      xWriteFlag(pcVPS->m_ptPresentFlag[i], "vps_pt_present_flag");
    }
    if (!pcVPS->m_vpsDefaultPtlDpbHrdMaxTidFlag)
    {
      xWriteCode(pcVPS->m_ptlMaxTemporalId[i], 3, "vps_ptl_max_tid");
    }
    else
    {
      CHECK(pcVPS->m_ptlMaxTemporalId[i] != pcVPS->m_vpsMaxSubLayers - 1,
            "When vps_default_ptl_dpb_hrd_max_tid_flag is equal to 1, the value of vps_ptl_max_tid[ i ] is inferred to "
            "be equal to vps_max_sublayers_minus1");
    }
  }
  int cnt = 0;
  while (m_pcBitIf->getNumBitsUntilByteAligned())
  {
    xWriteFlag(0, "vps_ptl_reserved_zero_bit");
    cnt++;
  }
  CHECK(cnt >= 8, "More than '8' alignment bytes written");
  for (int i = 0; i < pcVPS->getNumPtls(); i++)
  {
    codeProfileTierLevel(&pcVPS->m_vpsProfileTierLevel[i], pcVPS->m_ptPresentFlag[i], pcVPS->m_ptlMaxTemporalId[i]);
  }
  for (int i = 0; i < totalNumOlss; i++)
  {
    if (pcVPS->getNumPtls() > 1 && pcVPS->getNumPtls() != pcVPS->m_totalNumOLSs)
    {
      xWriteCode(pcVPS->m_olsPtlIdx[i], 8, "vps_ols_ptl_idx");
    }
  }

  if (!pcVPS->m_vpsEachLayerIsAnOlsFlag)
  {
    xWriteUvlc(pcVPS->m_numDpbParams - 1, "vps_num_dpb_params_minus1");

    if (pcVPS->m_vpsMaxSubLayers > 1)
    {
      xWriteFlag(pcVPS->m_sublayerDpbParamsPresentFlag, "vps_sublayer_dpb_params_present_flag");
    }

    for (int i = 0; i < pcVPS->m_numDpbParams; i++)
    {
      if (!pcVPS->m_vpsDefaultPtlDpbHrdMaxTidFlag)
      {
        xWriteCode(pcVPS->m_dpbMaxTemporalId[i], 3, "vps_dpb_max_tid[i]");
      }
      else
      {
        CHECK(pcVPS->m_dpbMaxTemporalId[i] != pcVPS->m_vpsMaxSubLayers - 1,
              "When vps_default_ptl_dpb_hrd_max_tid_flag is equal to 1, the value of vps_dpb_max_tid[ i ] is inferred "
              "to be equal to vps_max_sublayers_minus1");
      }

      for (int j = (pcVPS->m_sublayerDpbParamsPresentFlag ? 0 : pcVPS->m_dpbMaxTemporalId[i]);
           j <= pcVPS->m_dpbMaxTemporalId[i]; j++)
      {
        CHECK(pcVPS->m_dpbParameters[i].maxDecPicBuffering[j] < 1, "MaxDecPicBuffering must be greater than 0");
        xWriteUvlc(pcVPS->m_dpbParameters[i].maxDecPicBuffering[j] - 1, "dpb_max_dec_pic_buffering_minus1[i]");
        xWriteUvlc(pcVPS->m_dpbParameters[i].maxNumReorderPics[j], "dpb_max_num_reorder_pics[i]");
        xWriteUvlc(pcVPS->m_dpbParameters[i].maxLatencyIncreasePlus1[j], "dpb_max_latency_increase_plus1[i]");
      }
    }

    for (int i = 0; i < pcVPS->m_totalNumOLSs; i++)
    {
      if (pcVPS->m_numLayersInOls[i] > 1)
      {
        xWriteUvlc(pcVPS->m_olsDpbPicSize[i].width, "vps_ols_dpb_pic_width[i]");
        xWriteUvlc(pcVPS->m_olsDpbPicSize[i].height, "vps_ols_dpb_pic_height[i]");
        xWriteCode(to_underlying(pcVPS->m_olsDpbChromaFormatIdc[i]), 2, "vps_ols_dpb_chroma_format[i]");
        const Profile::Name profile = pcVPS->m_vpsProfileTierLevel[pcVPS->m_olsPtlIdx[i]].m_profileIdc;
        if (profile != Profile::NONE)
        {
          CHECK(pcVPS->m_olsDpbBitDepthMinus8[i] + 8 > ProfileFeatures::getProfileFeatures(profile)->maxBitDepth,
                "vps_ols_dpb_bitdepth_minus8[ i ] exceeds range supported by signalled profile");
        }
        xWriteUvlc(pcVPS->m_olsDpbBitDepthMinus8[i], "vps_ols_dpb_bitdepth_minus8[i]");
        if ((pcVPS->m_numDpbParams > 1) && (pcVPS->m_numDpbParams != pcVPS->m_numMultiLayeredOlss))
        {
          xWriteUvlc(pcVPS->m_olsDpbParamsIdx[i], "vps_ols_dpb_params_idx[i]");
        }
      }
    }
  }
  if (!pcVPS->m_vpsEachLayerIsAnOlsFlag)
  {
    xWriteFlag(pcVPS->m_vpsGeneralHrdParamsPresentFlag, "vps_general_hrd_params_present_flag");
  }
  if (pcVPS->m_vpsGeneralHrdParamsPresentFlag)
  {
    codeGeneralHrdparameters(&pcVPS->m_generalHrdParams);
    if ((pcVPS->m_vpsMaxSubLayers - 1) > 0)
    {
      xWriteFlag(pcVPS->m_vpsSublayerCpbParamsPresentFlag, "vps_sublayer_cpb_params_present_flag");
    }
    xWriteUvlc(pcVPS->m_numOlsTimingHrdParamsMinus1, "vps_num_ols_timing_hrd_params_minus1");
    for (int i = 0; i <= pcVPS->m_numOlsTimingHrdParamsMinus1; i++)
    {
      if (!pcVPS->m_vpsDefaultPtlDpbHrdMaxTidFlag)
      {
        xWriteCode(pcVPS->m_hrdMaxTid[i], 3, "vps_hrd_max_tid[i]");
      }
      else
      {
        CHECK(pcVPS->m_hrdMaxTid[i] != pcVPS->m_vpsMaxSubLayers - 1,
              "When vps_default_ptl_dpb_hrd_max_tid_flag is equal to 1, the value of vps_hrd_max_tid[ i ] is inferred "
              "to be equal to vps_max_sublayers_minus1");
      }
      uint32_t firstSublayer = pcVPS->m_vpsSublayerCpbParamsPresentFlag ? 0 : pcVPS->m_hrdMaxTid[i];
      codeOlsHrdParameters(&pcVPS->m_generalHrdParams, &pcVPS->m_olsHrdParams[i][0], firstSublayer,
                           pcVPS->m_hrdMaxTid[i]);
    }
    if ((pcVPS->m_numOlsTimingHrdParamsMinus1 > 0) &&
        ((pcVPS->m_numOlsTimingHrdParamsMinus1 + 1) != pcVPS->m_numMultiLayeredOlss))
    {
      for (int i = 0; i < pcVPS->m_numMultiLayeredOlss; i++)
      {
        xWriteUvlc(pcVPS->m_olsTimingHrdIdx[i], "vps_ols_timing_hrd_idx[i]");
      }
    }
  }

  xWriteFlag(0, "vps_extension_flag");

  // future extensions here..
  xWriteRbspTrailingBits();
}
void HLSWriter::codePictureHeader(PicHeader *picHeader, bool writeRbspTrailingBits, Slice *slice)
{
  const PPS *pps = nullptr;
  const SPS *sps = nullptr;

#if ENABLE_TRACING
  xTracePictureHeader();
#endif

  if (!slice)
  {
    slice = picHeader->m_pic->m_cs->slice;
  }
  xWriteFlag(picHeader->m_gdrOrIrapPicFlag, "ph_gdr_or_irap_pic_flag");
  xWriteFlag(picHeader->m_nonReferencePictureFlag, "ph_non_ref_pic_flag");
  if (picHeader->m_gdrOrIrapPicFlag)
  {
    xWriteFlag(picHeader->m_gdrPicFlag, "ph_gdr_pic_flag");
  }
  // Q0781, two-flags
  xWriteFlag(picHeader->m_picInterSliceAllowedFlag, "ph_inter_slice_allowed_flag");
  if (picHeader->m_picInterSliceAllowedFlag)
  {
    xWriteFlag(picHeader->m_picIntraSliceAllowedFlag, "ph_intra_slice_allowed_flag");
  }
  // parameter sets
  xWriteUvlc(picHeader->m_ppsId, "ph_pic_parameter_set_id");
  pps = slice->m_pps;
  CHECK(pps == 0, "Invalid PPS");
  sps = slice->m_sps;
  CHECK(sps == 0, "Invalid SPS");
  int pocBits = slice->m_sps->m_bitsForPoc;
  int pocMask = (1 << pocBits) - 1;
  xWriteCode(slice->m_poc & pocMask, pocBits, "ph_pic_order_cnt_lsb");
  if (picHeader->m_gdrPicFlag)
  {
    xWriteUvlc(picHeader->m_recoveryPocCnt, "ph_recovery_poc_cnt");
  }
  // PH extra bits are not written in the reference encoder
  // as these bits are reserved for future extensions
  // for( i = 0; i < NumExtraPhBits; i++ )
  //    ph_extra_bit[ i ]

  if (sps->m_pocMsbCycleFlag)
  {
    xWriteFlag(picHeader->m_pocMsbPresentFlag, "ph_poc_msb_present_flag");
    if (picHeader->m_pocMsbPresentFlag)
    {
      xWriteCode(picHeader->m_pocMsbVal, sps->m_pocMsbCycleLen, "ph_poc_msb_cycle_val");
    }
  }

  // alf enable flags and aps IDs
  if (sps->m_alfEnabledFlag)
  {
    if (pps->m_alfInfoInPhFlag)
    {
      xWriteFlag(picHeader->m_alfEnabledFlag[COMP_Y], "ph_alf_enabled_flag");
      if (picHeader->m_alfEnabledFlag[COMP_Y])
      {
        if (sps->m_alfImprovementsEnabledFlag)
        {
          xWriteFlag(picHeader->m_newAlfFixFiltSetCandIdx[COMP_Y], "ph_alf_fixed_filter_set_cand_idx_luma");
        }

        xWriteCode(picHeader->m_numAlfApsIdsLuma, 3, "ph_num_alf_aps_ids_luma");
        const AlfParameters::AlfApsList &apsId = picHeader->m_alfApsIdsLuma;
        for (int i = 0; i < picHeader->m_numAlfApsIdsLuma; i++)
        {
          xWriteCode(apsId[i], 3, "ph_alf_aps_id_luma");
        }

        const int alfChromaIdc = picHeader->m_alfEnabledFlag[COMP_Cb] + picHeader->m_alfEnabledFlag[COMP_Cr] * 2;
        if (isChromaEnabled(sps->m_chromaFormatIdc))
        {
          xWriteCode(picHeader->m_alfEnabledFlag[COMP_Cb], 1, "ph_alf_cb_enabled_flag");
          xWriteCode(picHeader->m_alfEnabledFlag[COMP_Cr], 1, "ph_alf_cr_enabled_flag");
          if (sps->m_alfImprovementsEnabledFlag)
          {
            if (picHeader->m_alfEnabledFlag[COMP_Cb])
            {
              xWriteFlag(picHeader->m_newAlfFixFiltSetCandIdx[COMP_Cb], "ph_alf_fixed_filter_set_cand_idx_cb");
            }
            if (picHeader->m_alfEnabledFlag[COMP_Cr])
            {
              xWriteFlag(picHeader->m_newAlfFixFiltSetCandIdx[COMP_Cr], "ph_alf_fixed_filter_set_cand_idx_cr");
            }
          }
        }
        if (alfChromaIdc)
        {
          xWriteCode(picHeader->m_alfApsIdChroma, 3, "ph_alf_aps_id_chroma");
        }
        if (sps->m_ccalfEnabledFlag)
        {
          xWriteFlag(picHeader->m_ccalfEnabledFlag[COMP_Cb], "ph_cc_alf_cb_enabled_flag");
          if (picHeader->m_ccalfEnabledFlag[COMP_Cb])
          {
            xWriteCode(picHeader->m_ccAlfCbApsId, 3, "ph_cc_alf_cb_aps_id");
          }
          xWriteFlag(picHeader->m_ccalfEnabledFlag[COMP_Cr], "ph_cc_alf_cr_enabled_flag");
          if (picHeader->m_ccalfEnabledFlag[COMP_Cr])
          {
            xWriteCode(picHeader->m_ccAlfCrApsId, 3, "ph_cc_alf_cr_aps_id");
          }
        }
      }
    }
    else
    {
      CHECK(!picHeader->m_alfEnabledFlag[COMP_Y] || !picHeader->m_alfEnabledFlag[COMP_Cb] ||
              !picHeader->m_alfEnabledFlag[COMP_Cr],
            "unexpect value");
      CHECK(picHeader->m_ccalfEnabledFlag[COMP_Cb] != sps->m_ccalfEnabledFlag, "unexpect value");
      CHECK(picHeader->m_ccalfEnabledFlag[COMP_Cr] != sps->m_ccalfEnabledFlag, "unexpect value");
    }
  }
  else
  {
    CHECK(picHeader->m_alfEnabledFlag[COMP_Y] || picHeader->m_alfEnabledFlag[COMP_Cb] ||
            picHeader->m_alfEnabledFlag[COMP_Cr],
          "unexpect value");
    CHECK(picHeader->m_ccalfEnabledFlag[COMP_Cb], "unexpect value");
    CHECK(picHeader->m_ccalfEnabledFlag[COMP_Cr], "unexpect value");
  }

  // luma mapping / chroma scaling controls
  if (sps->m_lmcsEnabled)
  {
    xWriteFlag(picHeader->m_lmcsEnabledFlag, "ph_lmcs_enabled_flag");
    if (picHeader->m_lmcsEnabledFlag)
    {
      xWriteCode(picHeader->m_lmcsApsId, 2, "ph_lmcs_aps_id");
      if (isChromaEnabled(sps->m_chromaFormatIdc))
      {
        xWriteFlag(picHeader->m_lmcsChromaResidualScaleFlag, "ph_chroma_residual_scale_flag");
      }
      else
      {
        CHECK(picHeader->m_lmcsChromaResidualScaleFlag, "unexpect value");
      }
    }
  }
  else
  {
    CHECK(picHeader->m_lmcsEnabledFlag, "unexpect value");
    CHECK(picHeader->m_lmcsChromaResidualScaleFlag, "unexpect value");
  }

  // quantization scaling lists
  if (sps->m_scalingListEnabledFlag)
  {
    xWriteFlag(picHeader->m_explicitScalingListEnabledFlag, "ph_scaling_list_present_flag");
    if (picHeader->m_explicitScalingListEnabledFlag)
    {
      xWriteCode(picHeader->m_scalingListApsId, 3, "ph_scaling_list_aps_id");
    }
  }
  else
  {
    CHECK(picHeader->m_explicitScalingListEnabledFlag, "unexpect value");
  }

  // picture output flag
  if (pps->m_outputFlagPresentFlag && !picHeader->m_nonReferencePictureFlag)
  {
    xWriteFlag(picHeader->m_picOutputFlag, "ph_pic_output_flag");
  }
  else
  {
    CHECK(!picHeader->m_picOutputFlag, "unexpect value");
  }

  // reference picture lists
  if (pps->m_rplInfoInPhFlag)
  {
    // List0 and List1
    for (const auto l: { RPL0, RPL1 })
    {
      const int  numRplsInSps = sps->m_numRpl[l];
      const int  rplIdx       = picHeader->m_rplIdx[l];
      const bool rplSpsFlag   = rplIdx != -1;

      if (numRplsInSps == 0)
      {
        CHECK(rplSpsFlag, "rpl_sps_flag[1] will be infer to 0 and this is not what was expected");
      }
      else if (l == RPL0 || pps->m_rpl1IdxPresentFlag)
      {
        xWriteFlag(rplSpsFlag ? 1 : 0, "rpl_sps_flag[i]");
      }
      else
      {
        bool rplSpsFlag0 = picHeader->m_rplIdx[RPL0] != -1;
        CHECK(rplSpsFlag != rplSpsFlag0, "rpl_sps_flag[1] will be infer to 0 and this is not what was expected");
      }

      if (rplSpsFlag)
      {
        CHECK(rplIdx >= numRplsInSps, "rpl_idx is too large");

        if (l == RPL0 || pps->m_rpl1IdxPresentFlag)
        {
          if (numRplsInSps > 1)
          {
            const int numBits = ceilLog2(numRplsInSps);
            xWriteCode(rplIdx, numBits, "rpl_idx[i]");
          }
        }
        else
        {
          CHECK(rplIdx != picHeader->m_rplIdx[RPL0], "RPL1Idx is not signalled but it is not the same as RPL0Idx");
        }
      }
      // explicit RPL in picture header
      else
      {
        xCodeRefPicList(&picHeader->m_rpl[l], sps->m_longTermRefsPresent, sps->m_bitsForPoc,
                        !sps->m_useWP && !sps->m_useBiWP, -1);
      }

      // POC MSB cycle signalling for LTRP
      const ReferencePictureList *rpl = &picHeader->m_rpl[l];

      if (rpl != nullptr && rpl->m_numberOfLongtermPictures > 0)
      {
        for (int i = 0; i < rpl->getNumRefEntries(); i++)
        {
          if (rpl->m_isLongtermRefPic[i])
          {
            if (rpl->m_ltrpInSliceHeaderFlag)
            {
              xWriteCode(rpl->m_refPicIdentifier[i], sps->m_bitsForPoc, "poc_lsb_lt[listIdx][rplsIdx][j]");
            }
            xWriteFlag(rpl->m_deltaPocMSBPresentFlag[i] ? 1 : 0, "delta_poc_msb_present_flag[i][j]");
            if (rpl->m_deltaPocMSBPresentFlag[i])
            {
              xWriteUvlc(rpl->m_deltaPOCMSBCycleLT[i], "delta_poc_msb_cycle_lt[i][j]");
            }
          }
        }
      }
    }
  }

  // partitioning constraint overrides
  if (sps->m_partitionOverrideEnabled)
  {
    xWriteFlag(picHeader->m_splitConsOverrideFlag, "ph_partition_constraints_override_flag");
  }
  else
  {
    CHECK(picHeader->m_splitConsOverrideFlag, "unexpect value");
  }
  // Q0781, two-flags
  if (picHeader->m_picIntraSliceAllowedFlag)
  {
    if (picHeader->m_splitConsOverrideFlag)
    {
      xWriteUvlc(floorLog2(picHeader->getMinQTSize(I_SLICE)) - sps->m_log2MinCodingBlockSize,
                 "ph_log2_diff_min_qt_min_cb_intra_slice_luma");
      xWriteUvlc(picHeader->getMaxMTTHierarchyDepth(I_SLICE), "ph_max_mtt_hierarchy_depth_intra_slice_luma");
      if (picHeader->getMaxMTTHierarchyDepth(I_SLICE) != 0)
      {
        xWriteUvlc(floorLog2(picHeader->getMaxBTSize(I_SLICE)) - floorLog2(picHeader->getMinQTSize(I_SLICE)),
                   "ph_log2_diff_max_bt_min_qt_intra_slice_luma");
        xWriteUvlc(floorLog2(picHeader->getMaxTTSize(I_SLICE)) - floorLog2(picHeader->getMinQTSize(I_SLICE)),
                   "ph_log2_diff_max_tt_min_qt_intra_slice_luma");
      }

      if (sps->m_dualITree)
      {
        xWriteUvlc(floorLog2(picHeader->getMinQTSize(I_SLICE, ChannelType::CHROMA)) - sps->m_log2MinCodingBlockSize,
                   "ph_log2_diff_min_qt_min_cb_intra_slice_chroma");
        xWriteUvlc(picHeader->getMaxMTTHierarchyDepth(I_SLICE, ChannelType::CHROMA),
                   "ph_max_mtt_hierarchy_depth_intra_slice_chroma");
        if (picHeader->getMaxMTTHierarchyDepth(I_SLICE, ChannelType::CHROMA) != 0)
        {
          xWriteUvlc(floorLog2(picHeader->getMaxBTSize(I_SLICE, ChannelType::CHROMA)) -
                       floorLog2(picHeader->getMinQTSize(I_SLICE, ChannelType::CHROMA)),
                     "ph_log2_diff_max_bt_min_qt_intra_slice_chroma");
          xWriteUvlc(floorLog2(picHeader->getMaxTTSize(I_SLICE, ChannelType::CHROMA)) -
                       floorLog2(picHeader->getMinQTSize(I_SLICE, ChannelType::CHROMA)),
                     "ph_log2_diff_max_tt_min_qt_intra_slice_chroma");
        }
      }
    }
  }
  if (picHeader->m_picIntraSliceAllowedFlag)
  {
    // delta quantization and chrom and chroma offset
    if (pps->m_useDQP)
    {
      xWriteUvlc(picHeader->m_cuQpDeltaSubdivIntra, "ph_cu_qp_delta_subdiv_intra_slice");
    }
    else
    {
      CHECK(picHeader->m_cuQpDeltaSubdivIntra, "unexpect value");
    }
    if (pps->getCuChromaQpOffsetListEnabledFlag())
    {
      xWriteUvlc(picHeader->m_cuChromaQpOffsetSubdivIntra, "ph_cu_chroma_qp_offset_subdiv_intra_slice");
    }
  }

  if (picHeader->m_picInterSliceAllowedFlag)
  {
    if (picHeader->m_splitConsOverrideFlag)
    {
      xWriteUvlc(floorLog2(picHeader->getMinQTSize(P_SLICE)) - sps->m_log2MinCodingBlockSize,
                 "ph_log2_diff_min_qt_min_cb_inter_slice");
      xWriteUvlc(picHeader->getMaxMTTHierarchyDepth(P_SLICE), "ph_max_mtt_hierarchy_depth_inter_slice");
      if (picHeader->getMaxMTTHierarchyDepth(P_SLICE) != 0)
      {
        xWriteUvlc(floorLog2(picHeader->getMaxBTSize(P_SLICE)) - floorLog2(picHeader->getMinQTSize(P_SLICE)),
                   "ph_log2_diff_max_bt_min_qt_inter_slice");
        xWriteUvlc(floorLog2(picHeader->getMaxTTSize(P_SLICE)) - floorLog2(picHeader->getMinQTSize(P_SLICE)),
                   "ph_log2_diff_max_tt_min_qt_inter_slice");
      }
    }

    // delta quantization and chrom and chroma offset
    if (pps->m_useDQP)
    {
      xWriteUvlc(picHeader->m_cuQpDeltaSubdivInter, "ph_cu_qp_delta_subdiv_inter_slice");
    }
    else
    {
      CHECK(picHeader->m_cuQpDeltaSubdivInter, "unexpect value");
    }
    if (pps->getCuChromaQpOffsetListEnabledFlag())
    {
      xWriteUvlc(picHeader->m_cuChromaQpOffsetSubdivInter, "ph_cu_chroma_qp_offset_subdiv_inter_slice");
    }

    // temporal motion vector prediction
    if (sps->m_temporalMvpEnabledFlag)
    {
      xWriteFlag(picHeader->m_enableTMVPFlag, "ph_temporal_mvp_enabled_flag");
      if (picHeader->m_enableTMVPFlag && pps->m_rplInfoInPhFlag)
      {
        if (picHeader->m_rpl[RPL1].getNumRefEntries() > 0)
        {
          xWriteCode(picHeader->m_picColFromL0Flag, 1, "ph_collocated_from_l0_flag");
        }
        if ((picHeader->m_picColFromL0Flag && picHeader->m_rpl[RPL0].getNumRefEntries() > 1) ||
            (!picHeader->m_picColFromL0Flag && picHeader->m_rpl[RPL1].getNumRefEntries() > 1))
        {
          xWriteUvlc(picHeader->m_colRefIdx, "ph_collocated_ref_idx");
        }
      }
    }
    else
    {
      CHECK(picHeader->m_enableTMVPFlag, "unexpect value");
    }

    // merge candidate list size
    // subblock merge candidate list size
    if (sps->m_useAffine)
    {
      CHECK(picHeader->m_maxNumAffineMergeCand != sps->m_maxNumAffineMergeCand, "unexpect value");
    }
    else
    {
      CHECK(picHeader->m_maxNumAffineMergeCand != (sps->m_sbtmvpEnabledFlag && picHeader->m_enableTMVPFlag),
            "unexpect value");
    }

    // full-pel MMVD flag
    if (sps->m_fpelMmvdEnabledFlag)
    {
      xWriteFlag(picHeader->m_disFracMMVD, "ph_fpel_mmvd_enabled_flag");
    }
    else
    {
      CHECK(picHeader->m_disFracMMVD, "unexpect value");
    }
    if (sps->m_useGeo)
    {
      xWriteFlag(picHeader->m_gpmMMVDTableFlag, "ph_gpm_ext_mmvd_flag");
    }

    // mvd L1 zero flag
    if (!pps->m_rplInfoInPhFlag || picHeader->m_rpl[RPL1].getNumRefEntries() > 0)
    {
      xWriteFlag(picHeader->m_mvdL1ZeroFlag, "ph_mvd_l1_zero_flag");
    }

    // picture level BDOF disable flags
    if (sps->m_bdofControlPresentInPhFlag && (!pps->m_rplInfoInPhFlag || picHeader->m_rpl[RPL1].getNumRefEntries() > 0))
    {
      xWriteFlag(picHeader->m_bdofDisabledFlag, "ph_bdof_disabled_flag");
    }
    else
    {
      CHECK(picHeader->m_bdofDisabledFlag, "unexpect value");
    }
    // picture level PROF disable flags
    if (sps->m_profControlPresentInPhFlag)
    {
      xWriteFlag(picHeader->m_profDisabledFlag, "ph_prof_disabled_flag");
    }

    if ((pps->m_useWP || pps->m_useBiWP) && pps->m_wpInfoInPhFlag)
    {
      xCodePredWeightTable(picHeader, pps, sps);
    }
  }
  // inherit constraint values from SPS
  if (!sps->m_partitionOverrideEnabled || !picHeader->m_splitConsOverrideFlag)
  {
    //    CHECK( picHeader->m_minQT != sps->m_minQT, "unexpect value");
    //    CHECK( picHeader->m_maxMTTHierarchyDepth != sps->m_maxMTTHierarchyDepth, "unexpect value");
    //    CHECK( picHeader->m_maxBTSize != sps->m_maxBTSize, "unexpect value");
    //    CHECK( picHeader->m_maxTTSize != sps->m_maxTTSize, "unexpect value");
  }
  // ibc merge candidate list size
  if (pps->m_qpDeltaInfoInPhFlag)
  {
    xWriteSvlc(picHeader->m_qpDelta, "ph_qp_delta");
  }

  // joint Cb/Cr sign flag
  if (sps->m_jointCbCrEnabledFlag)
  {
    xWriteFlag(picHeader->m_jointCbCrSignFlag, "ph_joint_cbcr_sign_flag");
  }
  else
  {
    CHECK(picHeader->m_jointCbCrSignFlag, "unexpect value");
  }

#if ENABLE_NNLF
  // picture level nnlf disable flag
  if (sps->m_nnlf)
  {
    xWriteFlag(picHeader->m_nnlfDisabled, "ph_nnlf_disabled_flag");
  }
  else
  {
    CHECK(picHeader->m_nnlfDisabled, "unexpect value");
  }
#endif

  // sao enable flags
  if (sps->m_saoEnabledFlag)
  {
    if (pps->m_saoInfoInPhFlag)
    {
      xWriteFlag(picHeader->m_saoEnabledFlag[ChannelType::LUMA], "ph_sao_luma_enabled_flag");
      if (isChromaEnabled(sps->m_chromaFormatIdc))
      {
        xWriteFlag(picHeader->m_saoEnabledFlag[ChannelType::CHROMA], "ph_sao_chroma_enabled_flag");
      }
    }
    else
    {
      CHECK(!picHeader->m_saoEnabledFlag[ChannelType::LUMA], "unexpect value");
      CHECK(!picHeader->m_saoEnabledFlag[ChannelType::CHROMA], "unexpect value");
    }
  }
  else
  {
    CHECK(picHeader->m_saoEnabledFlag[ChannelType::LUMA], "unexpect value");
    CHECK(picHeader->m_saoEnabledFlag[ChannelType::CHROMA], "unexpect value");
  }

  if (sps->m_ccSaoEnabledFlag)
  {
    if (pps->m_saoInfoInPhFlag)
    {
      xWriteFlag(picHeader->m_ccSaoEnabledFlag[COMP_Y], "ph_cc_sao_y_enabled_flag");
      xWriteFlag(picHeader->m_ccSaoEnabledFlag[COMP_Cb], "ph_cc_sao_cb_enabled_flag");
      xWriteFlag(picHeader->m_ccSaoEnabledFlag[COMP_Cr], "ph_cc_sao_cr_enabled_flag");
    }
    else
    {
      picHeader->m_ccSaoEnabledFlag[COMP_Y]  = true;
      picHeader->m_ccSaoEnabledFlag[COMP_Cb] = true;
      picHeader->m_ccSaoEnabledFlag[COMP_Cr] = true;
    }
  }
  else
  {
    picHeader->m_ccSaoEnabledFlag[COMP_Y]  = false;
    picHeader->m_ccSaoEnabledFlag[COMP_Cb] = false;
    picHeader->m_ccSaoEnabledFlag[COMP_Cr] = false;
  }

  // deblocking filter controls
  if (pps->m_deblockingFilterControlPresentFlag)
  {
    if (pps->m_dbfInfoInPhFlag)
    {
      xWriteFlag(picHeader->m_deblockingFilterOverrideFlag, "ph_deblocking_params_present_flag");
    }
    else
    {
      CHECK(picHeader->m_deblockingFilterOverrideFlag, "unexpect value");
    }

    if (picHeader->m_deblockingFilterOverrideFlag)
    {
      if (!pps->m_ppsDeblockingFilterDisabledFlag)
      {
        xWriteFlag(picHeader->m_deblockingFilterDisable, "ph_deblocking_filter_disabled_flag");
      }
      if (!picHeader->m_deblockingFilterDisable)
      {
        xWriteSvlc(picHeader->m_deblockingFilterBetaOffsetDiv2, "ph_beta_offset_div2");
        xWriteSvlc(picHeader->m_deblockingFilterTcOffsetDiv2, "ph_tc_offset_div2");
        if (pps->m_usePPSChromaTool)
        {
          xWriteSvlc(picHeader->m_deblockingFilterCbBetaOffsetDiv2, "ph_cb_beta_offset_div2");
          xWriteSvlc(picHeader->m_deblockingFilterCbTcOffsetDiv2, "ph_cb_tc_offset_div2");
          xWriteSvlc(picHeader->m_deblockingFilterCrBetaOffsetDiv2, "ph_cr_beta_offset_div2");
          xWriteSvlc(picHeader->m_deblockingFilterCrTcOffsetDiv2, "ph_cr_tc_offset_div2");
        }
      }
    }
    else
    {
      CHECK(picHeader->m_deblockingFilterDisable != pps->m_ppsDeblockingFilterDisabledFlag, "unexpect value");
      CHECK(picHeader->m_deblockingFilterBetaOffsetDiv2 != pps->m_deblockingFilterBetaOffsetDiv2, "unexpect value");
      CHECK(picHeader->m_deblockingFilterTcOffsetDiv2 != pps->m_deblockingFilterTcOffsetDiv2, "unexpect value");
      CHECK(picHeader->m_deblockingFilterCbBetaOffsetDiv2 != pps->m_deblockingFilterCbBetaOffsetDiv2, "unexpect value");
      CHECK(picHeader->m_deblockingFilterCbTcOffsetDiv2 != pps->m_deblockingFilterCbTcOffsetDiv2, "unexpect value");
      CHECK(picHeader->m_deblockingFilterCrBetaOffsetDiv2 != pps->m_deblockingFilterCrBetaOffsetDiv2, "unexpect value");
      CHECK(picHeader->m_deblockingFilterCrTcOffsetDiv2 != pps->m_deblockingFilterCrTcOffsetDiv2, "unexpect value");
    }
  }
  else
  {
    CHECK(picHeader->m_deblockingFilterDisable, "unexpect value");
    CHECK(picHeader->m_deblockingFilterBetaOffsetDiv2, "unexpect value");
    CHECK(picHeader->m_deblockingFilterTcOffsetDiv2, "unexpect value");
    CHECK(picHeader->m_deblockingFilterCbBetaOffsetDiv2, "unexpect value");
    CHECK(picHeader->m_deblockingFilterCbTcOffsetDiv2, "unexpect value");
    CHECK(picHeader->m_deblockingFilterCrBetaOffsetDiv2, "unexpect value");
    CHECK(picHeader->m_deblockingFilterCrTcOffsetDiv2, "unexpect value");
  }

  // picture header extension
  if (pps->m_pictureHeaderExtensionPresentFlag)
  {
    xWriteUvlc(0, "ph_extension_length");
  }

  if (writeRbspTrailingBits)
  {
    xWriteRbspTrailingBits();
  }
}
void HLSWriter::codeSliceHeader(Slice *pcSlice, PicHeader *picHeader)
{
#if ENABLE_TRACING
  xTraceSliceHeader();
#endif

  if (!picHeader)
  {
    CodingStructure &cs = *pcSlice->m_pic->m_cs;
    picHeader           = cs.picHeader;
  }
  const ChromaFormat format                = pcSlice->m_sps->m_chromaFormatIdc;
  const uint32_t     numberValidComponents = getNumberValidComponents(format);
  const bool         chromaEnabled         = isChromaEnabled(format);
  xWriteFlag(pcSlice->m_pictureHeaderInSliceHeader ? 1 : 0, "sh_picture_header_in_slice_header_flag");
  if (pcSlice->m_pictureHeaderInSliceHeader)
  {
    codePictureHeader(picHeader, false);
  }

  if (pcSlice->m_sps->m_subPicInfoPresentFlag)
  {
    uint32_t bitsSubPicId;
    bitsSubPicId = pcSlice->m_sps->m_subPicIdLen;
    xWriteCode(pcSlice->m_sliceSubPicId, bitsSubPicId, "sh_subpic_id");
  }

  // raster scan slices
  if (pcSlice->m_pps->m_rectSliceFlag == 0)
  {
    // slice address is the raster scan tile index of first tile in slice
    if (pcSlice->m_pps->getNumTiles() > 1)
    {
      int bitsSliceAddress = ceilLog2(pcSlice->m_pps->getNumTiles());
      xWriteCode(pcSlice->getSliceID(), bitsSliceAddress, "sh_slice_address");
      if ((int)pcSlice->m_pps->getNumTiles() - (int)pcSlice->getSliceID() > 1)
      {
        xWriteUvlc(pcSlice->getNumTilesInSlice() - 1, "sh_num_tiles_in_slice_minus1");
      }
    }
  }
  // rectangular slices
  else
  {
    // slice address is the index of the slice within the current sub-picture
    uint32_t currSubPicIdx = pcSlice->m_pps->getSubPicIdxFromSubPicId(pcSlice->m_sliceSubPicId);
    SubPic   currSubPic    = pcSlice->m_pps->m_subPics[currSubPicIdx];
    if (currSubPic.m_numSlicesInSubPic > 1)
    {
      int numSlicesInPreviousSubPics = 0;
      for (int sp = 0; sp < currSubPicIdx; sp++)
      {
        numSlicesInPreviousSubPics += pcSlice->m_pps->m_subPics[sp].m_numSlicesInSubPic;
      }
      int bitsSliceAddress = ceilLog2(currSubPic.m_numSlicesInSubPic);
      xWriteCode(pcSlice->getSliceID() - numSlicesInPreviousSubPics, bitsSliceAddress, "sh_slice_address");
    }
  }

  if (picHeader->m_picInterSliceAllowedFlag)
  {
    xWriteUvlc(pcSlice->m_eSliceType, "sh_slice_type");
  }
  if (pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA || pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
      pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL || pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_GDR)
  {
    xWriteFlag(pcSlice->m_noOutputOfPriorPicsFlag, "sh_no_output_of_prior_pics_flag");
  }
  if (!picHeader->m_picIntraSliceAllowedFlag)
  {
    CHECK(pcSlice->m_eSliceType == I_SLICE, "when ph_intra_slice_allowed_flag = 0, no I_Slice is allowed");
  }
  if (pcSlice->m_sps->m_lfCccmEnabledFlag && !pcSlice->isIntra())
  {
    xWriteFlag(pcSlice->m_lfCccmEnabledFlag, "sh_lfcccm_enabled_flag");
  }
  if (pcSlice->m_sps->m_alfEnabledFlag && !pcSlice->m_pps->m_alfInfoInPhFlag)
  {
    const int alfEnabled = pcSlice->m_alfEnabledFlag[COMP_Y];
    xWriteFlag(alfEnabled, "sh_alf_enabled_flag");

    if (alfEnabled)
    {
      if (pcSlice->m_sps->m_alfImprovementsEnabledFlag)
      {
        xWriteFlag(pcSlice->m_newAlfFixFiltSetCandIdx[COMP_Y], "slice_alf_fixed_filter_set_cand_idx_luma");
      }

      xWriteCode(pcSlice->m_numAlfApsIdsLuma, 3, "sh_num_alf_aps_ids_luma");
      const AlfParameters::AlfApsList &apsId = pcSlice->m_alfApsIdsLuma;
      for (int i = 0; i < pcSlice->m_numAlfApsIdsLuma; i++)
      {
        xWriteCode(apsId[i], 3, "sh_alf_aps_id_luma[i]");
      }

      const int alfChromaIdc = pcSlice->m_alfEnabledFlag[COMP_Cb] + pcSlice->m_alfEnabledFlag[COMP_Cr] * 2;
      if (chromaEnabled)
      {
        xWriteCode(pcSlice->m_alfEnabledFlag[COMP_Cb], 1, "sh_alf_cb_enabled_flag");
        xWriteCode(pcSlice->m_alfEnabledFlag[COMP_Cr], 1, "sh_alf_cr_enabled_flag");
        if (pcSlice->m_sps->m_alfImprovementsEnabledFlag)
        {
          if (pcSlice->m_alfEnabledFlag[COMP_Cb])
          {
            xWriteFlag(pcSlice->m_newAlfFixFiltSetCandIdx[COMP_Cb], "slice_alf_fixed_filter_set_cand_idx_cb");
          }
          if (pcSlice->m_alfEnabledFlag[COMP_Cr])
          {
            xWriteFlag(pcSlice->m_newAlfFixFiltSetCandIdx[COMP_Cr], "slice_alf_fixed_filter_set_cand_idx_cr");
          }
        }
      }
      if (alfChromaIdc)
      {
        xWriteCode(pcSlice->m_alfApsIdChroma, 3, "sh_alf_aps_id_chroma");
      }

      if (pcSlice->m_sps->m_ccalfEnabledFlag)
      {
        const AlfParameters::CcAlfFilterParamBase &filterParam = pcSlice->m_ccAlfFilterParam.getParam();
        xWriteFlag(filterParam.ccAlfFilterEnabled[COMP_Cb - 1] ? 1 : 0, "sh_alf_cc_cb_enabled_flag");
        if (filterParam.ccAlfFilterEnabled[COMP_Cb - 1])
        {
          // write CC ALF Cb APS ID
          xWriteCode(pcSlice->m_ccAlfCbApsId, 3, "sh_alf_cc_cb_aps_id");
        }
        // Cr
        xWriteFlag(filterParam.ccAlfFilterEnabled[COMP_Cr - 1] ? 1 : 0, "sh_alf_cc_cr_enabled_flag");
        if (filterParam.ccAlfFilterEnabled[COMP_Cr - 1])
        {
          // write CC ALF Cr APS ID
          xWriteCode(pcSlice->m_ccAlfCrApsId, 3, "sh_alf_cc_cr_aps_id");
        }
      }
    }
  }

  if (picHeader->m_lmcsEnabledFlag && !pcSlice->m_pictureHeaderInSliceHeader)
  {
    xWriteFlag(pcSlice->m_lmcsEnabledFlag, "sh_lmcs_used_flag");
  }
  if (picHeader->m_explicitScalingListEnabledFlag && !pcSlice->m_pictureHeaderInSliceHeader)
  {
    xWriteFlag(pcSlice->m_explicitScalingListUsed, "sh_explicit_scaling_list_used_flag");
  }

  if (!pcSlice->m_pps->m_rplInfoInPhFlag && (!pcSlice->getIdrPicFlag() || pcSlice->m_sps->m_idrRefParamList))
  {
    // Write L0 related syntax elements
    const int numRplsInSps = pcSlice->m_sps->m_numRpl[RPL0];
    const int rplIdx       = pcSlice->m_rplIdx[RPL0];

    if (numRplsInSps > 0)
    {
      xWriteFlag(rplIdx != -1 ? 1 : 0, "ref_pic_list_sps_flag[0]");
    }
    if (rplIdx != -1)
    {
      if (numRplsInSps > 1)
      {
        int numBits = ceilLog2(numRplsInSps);
        xWriteCode(rplIdx, numBits, "ref_pic_list_idx[0]");
      }
    }
    else
    {   // write local RPL0
      xCodeRefPicList(&pcSlice->m_rpl[RPL0], pcSlice->m_sps->m_longTermRefsPresent, pcSlice->m_sps->m_bitsForPoc,
                      !pcSlice->m_sps->m_useWP && !pcSlice->m_sps->m_useBiWP, -1);
    }
    // Deal POC Msb cycle signalling for LTRP
    if (pcSlice->m_rpl[RPL0].m_numberOfLongtermPictures)
    {
      for (int i = 0; i < pcSlice->m_rpl[RPL0].getNumRefEntries(); i++)
      {
        if (pcSlice->m_rpl[RPL0].m_isLongtermRefPic[i])
        {
          if (pcSlice->m_rpl[RPL0].m_ltrpInSliceHeaderFlag)
          {
            xWriteCode(pcSlice->m_rpl[RPL0].m_refPicIdentifier[i], pcSlice->m_sps->m_bitsForPoc,
                       "slice_poc_lsb_lt[listIdx][rplsIdx][j]");
          }
          xWriteFlag(pcSlice->m_rpl[RPL0].m_deltaPocMSBPresentFlag[i] ? 1 : 0, "delta_poc_msb_present_flag[i][j]");
          if (pcSlice->m_rpl[RPL0].m_deltaPocMSBPresentFlag[i])
          {
            xWriteUvlc(pcSlice->m_rpl[RPL0].m_deltaPOCMSBCycleLT[i], "delta_poc_msb_cycle_lt[i][j]");
          }
        }
      }
    }

    // Write L1 related syntax elements
    if (pcSlice->m_sps->m_numRpl[RPL1] > 0 && pcSlice->m_pps->m_rpl1IdxPresentFlag)
    {
      xWriteFlag(pcSlice->m_rplIdx[RPL1] != -1 ? 1 : 0, "ref_pic_list_sps_flag[1]");
    }
    else if (pcSlice->m_sps->m_numRpl[RPL1] == 0)
    {
      CHECK(pcSlice->m_rplIdx[RPL1] != -1, "rpl_sps_flag[1] will be infer to 0 and this is not what was expected");
    }
    else
    {
      auto rplsSpsFlag0 = pcSlice->m_rplIdx[RPL0] != -1 ? 1 : 0;
      auto rplsSpsFlag1 = pcSlice->m_rplIdx[RPL1] != -1 ? 1 : 0;
      CHECK(rplsSpsFlag1 != rplsSpsFlag0, "rpl_sps_flag[1] will be infer to 0 and this is not what was expected");
    }

    if (pcSlice->m_rplIdx[RPL1] != -1)
    {
      if (pcSlice->m_sps->m_numRpl[RPL1] > 1 && pcSlice->m_pps->m_rpl1IdxPresentFlag)
      {
        int numBits = ceilLog2(pcSlice->m_sps->m_numRpl[RPL1]);
        xWriteCode(pcSlice->m_rplIdx[RPL1], numBits, "ref_pic_list_idx[1]");
      }
      else if (pcSlice->m_sps->m_numRpl[RPL1] == 1)
      {
        CHECK(pcSlice->m_rplIdx[RPL1] != 0, "RPL1Idx is not signalled but it is not equal to 0");
      }
      else
      {
        CHECK(pcSlice->m_rplIdx[RPL1] != pcSlice->m_rplIdx[RPL0],
              "RPL1Idx is not signalled but it is not the same as RPL0Idx");
      }
    }
    else
    {   // write local RPL1
      xCodeRefPicList(&pcSlice->m_rpl[RPL1], pcSlice->m_sps->m_longTermRefsPresent, pcSlice->m_sps->m_bitsForPoc,
                      !pcSlice->m_sps->m_useWP && !pcSlice->m_sps->m_useBiWP, -1);
    }
    // Deal POC Msb cycle signalling for LTRP
    if (pcSlice->m_rpl[RPL1].m_numberOfLongtermPictures)
    {
      for (int i = 0; i < pcSlice->m_rpl[RPL1].getNumRefEntries(); i++)
      {
        if (pcSlice->m_rpl[RPL1].m_isLongtermRefPic[i])
        {
          if (pcSlice->m_rpl[RPL1].m_ltrpInSliceHeaderFlag)
          {
            xWriteCode(pcSlice->m_rpl[RPL1].m_refPicIdentifier[i], pcSlice->m_sps->m_bitsForPoc,
                       "slice_poc_lsb_lt[listIdx][rplsIdx][j]");
          }
          xWriteFlag(pcSlice->m_rpl[RPL1].m_deltaPocMSBPresentFlag[i] ? 1 : 0, "delta_poc_msb_present_flag[i][j]");
          if (pcSlice->m_rpl[RPL1].m_deltaPocMSBPresentFlag[i])
          {
            xWriteUvlc(pcSlice->m_rpl[RPL1].m_deltaPOCMSBCycleLT[i], "delta_poc_msb_cycle_lt[i][j]");
          }
        }
      }
    }
  }

  // check if numbers of active references match the defaults. If not, override

  CHECK(pcSlice->isIntra() && pcSlice->m_numRefIdx[RPL0] > 0, "Bad number of refs");
  CHECK(!pcSlice->isInterB() && pcSlice->m_numRefIdx[RPL1] > 0, "Bad number of refs");

  if ((!pcSlice->isIntra() && pcSlice->m_rpl[RPL0].getNumRefEntries() > 1) ||
      (pcSlice->isInterB() && pcSlice->m_rpl[RPL1].getNumRefEntries() > 1))
  {
    const int defaultL0 =
      std::min<int>(pcSlice->m_rpl[RPL0].getNumRefEntries(), pcSlice->m_pps->m_numRefIdxDefaultActive[RPL0]);

    bool overrideFlag = pcSlice->m_numRefIdx[RPL0] != defaultL0;

    if (!overrideFlag && pcSlice->isInterB())
    {
      const int defaultL1 =
        std::min<int>(pcSlice->m_rpl[RPL1].getNumRefEntries(), pcSlice->m_pps->m_numRefIdxDefaultActive[RPL1]);

      overrideFlag = pcSlice->m_numRefIdx[RPL1] != defaultL1;
    }

    xWriteFlag(overrideFlag ? 1 : 0, "sh_num_ref_idx_active_override_flag");
    if (overrideFlag)
    {
      if (pcSlice->m_rpl[RPL0].getNumRefEntries() > 1)
      {
        xWriteUvlc(pcSlice->m_numRefIdx[RPL0] - 1, "sh_num_ref_idx_active_minus1[0]");
      }

      if (pcSlice->isInterB() && pcSlice->m_rpl[RPL1].getNumRefEntries() > 1)
      {
        xWriteUvlc(pcSlice->m_numRefIdx[RPL1] - 1, "sh_num_ref_idx_active_minus1[1]");
      }
    }
  }

  if (!pcSlice->isIntra())
  {
    if (!pcSlice->isIntra() && pcSlice->m_pps->m_cabacInitPresentFlag)
    {
      SliceType sliceType        = pcSlice->m_eSliceType;
      SliceType encCABACTableIdx = pcSlice->m_encCABACTableIdx;
      bool      encCabacInitFlag = (sliceType != encCABACTableIdx && encCABACTableIdx != I_SLICE) ? true : false;
      xWriteFlag(encCabacInitFlag ? 1 : 0, "sh_cabac_init_flag");
    }
  }
  if (pcSlice->m_picHeader->m_enableTMVPFlag && !pcSlice->m_pps->m_rplInfoInPhFlag)
  {
    if (!pcSlice->m_pps->m_rplInfoInPhFlag)
    {
      if (pcSlice->m_eSliceType == B_SLICE)
      {
        xWriteFlag(pcSlice->m_colFromL0Flag, "sh_collocated_from_l0_flag");
      }
    }

    if (pcSlice->m_eSliceType != I_SLICE &&
        ((pcSlice->m_colFromL0Flag == 1 && pcSlice->m_numRefIdx[RPL0] > 1) ||
         (pcSlice->m_colFromL0Flag == 0 && pcSlice->m_numRefIdx[RPL1] > 1)))
    {
      xWriteUvlc(pcSlice->m_colRefIdx, "sh_collocated_ref_idx");
    }
  }

  if ((pcSlice->m_pps->m_useWP && pcSlice->m_eSliceType == P_SLICE) ||
      (pcSlice->m_pps->m_useBiWP && pcSlice->m_eSliceType == B_SLICE))
  {
    if (!pcSlice->m_pps->m_wpInfoInPhFlag)
    {
      xCodePredWeightTable(pcSlice);
    }
  }

  if (!pcSlice->m_pps->m_qpDeltaInfoInPhFlag)
  {
    int QP = 0;
    if (pcSlice->m_sps->m_useInterRPL)
    {
      int idx = pcSlice->m_rplIdx[RPL0];
      if (!pcSlice->isIntra() && idx >= 0 && idx < pcSlice->m_sps->m_QPoffsetRPL.size())
      {
        QP = pcSlice->m_sps->m_QPoffsetRPL[idx];
      }
    }
    xWriteSvlc(pcSlice->m_iSliceQp - (pcSlice->m_pps->m_picInitQPMinus26 + 26) - QP, "sh_qp_delta");
  }
  if (pcSlice->m_pps->m_sliceChromaQpFlag)
  {
    if (numberValidComponents > COMP_Cb)
    {
      xWriteSvlc(pcSlice->getSliceChromaQpDelta(COMP_Cb), "sh_cb_qp_offset");
    }
    if (numberValidComponents > COMP_Cr)
    {
      xWriteSvlc(pcSlice->getSliceChromaQpDelta(COMP_Cr), "sh_cr_qp_offset");
      if (pcSlice->m_sps->m_jointCbCrEnabledFlag)
      {
        xWriteSvlc(pcSlice->getSliceChromaQpDelta(JOINT_CbCr), "sh_joint_cbcr_qp_offset");
      }
    }
    CHECK(numberValidComponents < COMP_Cr + 1, "Not enough valid components");
  }

  if (pcSlice->m_pps->getCuChromaQpOffsetListEnabledFlag())
  {
    xWriteFlag(pcSlice->m_chromaQpAdjEnabled, "sh_cu_chroma_qp_offset_enabled_flag");
  }

#if ENABLE_NNLF
  if (pcSlice->m_sps->m_nnlf && !picHeader->m_nnlfDisabled)
  {
    const NnlfSliceParameters prm = pcSlice->m_nnlfUnifiedParam;
    xWriteUvlc(prm.mode + 1, "slice_nnlf_unified_mode");
    if (prm.mode != -1)
    {
      const int numprms = NNLF_UNIFIED_MAX_NUM_PRMS;
      xWriteUvlc(prm.scaleId + 1, "slice_nnlf_unified_scale_id");
      if (prm.scaleId == 0)
      {
        if (prm.mode < numprms)
        {
          xWriteSCode(prm.scale[CompID::COMP_Y][prm.mode] - (1 << NNFilterUnified::log2ResidueScale),
                      NNFilterUnified::log2ResidueScale + 1, "y nnScale");
          xWriteSCode(prm.scale[CompID::COMP_Cb][prm.mode] - (1 << NNFilterUnified::log2ResidueScale),
                      NNFilterUnified::log2ResidueScale + 1, "cb nnScale");
          xWriteSCode(prm.scale[CompID::COMP_Cr][prm.mode] - (1 << NNFilterUnified::log2ResidueScale),
                      NNFilterUnified::log2ResidueScale + 1, "cr nnScale");
        }
        else
        {
          for (int prmId = 0; prmId < numprms; prmId++)
          {
            xWriteSCode(prm.scale[CompID::COMP_Y][prmId] - (1 << NNFilterUnified::log2ResidueScale),
                        NNFilterUnified::log2ResidueScale + 1, "y nnScale");
            xWriteSCode(prm.scale[CompID::COMP_Cb][prmId] - (1 << NNFilterUnified::log2ResidueScale),
                        NNFilterUnified::log2ResidueScale + 1, "cb nnScale");
            xWriteSCode(prm.scale[CompID::COMP_Cr][prmId] - (1 << NNFilterUnified::log2ResidueScale),
                        NNFilterUnified::log2ResidueScale + 1, "cr nnScale");
          }
        }
      }

      int offsetI = (prm.scaleId == -1) ? 3 : prm.scaleId;
      if (prm.mode < numprms)
      {
        if (prm.offset[CompID::COMP_Y][prm.mode][offsetI] == 0)
        {
          xWriteFlag(0, "y_roa_flag");
        }
        else
        {
          xWriteFlag(1, "y_roa_flag");
          xWriteFlag(prm.offset[CompID::COMP_Y][prm.mode][offsetI] - 1, "y_roa_offset");
        }

        if (prm.offset[CompID::COMP_Cb][prm.mode][offsetI] == 0)
        {
          xWriteFlag(0, "u_roa_flag");
        }
        else
        {
          xWriteFlag(1, "u_roa_flag");
          xWriteFlag(prm.offset[CompID::COMP_Cb][prm.mode][offsetI] - 1, "u_roa_offset");
        }

        if (prm.offset[CompID::COMP_Cr][prm.mode][offsetI] == 0)
        {
          xWriteFlag(0, "v_roa_flag");
        }
        else
        {
          xWriteFlag(1, "v_roa_flag");
          xWriteFlag(prm.offset[CompID::COMP_Cr][prm.mode][offsetI] - 1, "v_roa_offset");
        }
      }
      else
      {
        for (int prmId = 0; prmId < numprms; prmId++)
        {
          if (prm.offset[CompID::COMP_Y][prmId][offsetI] == 0)
          {
            xWriteFlag(0, "y_roa_flag");
          }
          else
          {
            xWriteFlag(1, "y_roa_flag");
            xWriteFlag(prm.offset[CompID::COMP_Y][prmId][offsetI] - 1, "y_roa_offset");
          }

          if (prm.offset[CompID::COMP_Cb][prmId][offsetI] == 0)
          {
            xWriteFlag(0, "u_roa_flag");
          }
          else
          {
            xWriteFlag(1, "u_roa_flag");
            xWriteFlag(prm.offset[CompID::COMP_Cb][prmId][offsetI] - 1, "u_roa_offset");
          }

          if (prm.offset[CompID::COMP_Cr][prmId][offsetI] == 0)
          {
            xWriteFlag(0, "v_roa_flag");
          }
          else
          {
            xWriteFlag(1, "v_roa_flag");
            xWriteFlag(prm.offset[CompID::COMP_Cr][prmId][offsetI] - 1, "v_roa_offset");
          }
        }
      }
    }
  }
#endif

  if (pcSlice->m_sps->m_saoEnabledFlag && !pcSlice->m_pps->m_saoInfoInPhFlag)
  {
    xWriteFlag(pcSlice->m_saoEnabledFlag[ChannelType::LUMA], "sh_sao_luma_used_flag");
    if (chromaEnabled)
    {
      xWriteFlag(pcSlice->m_saoEnabledFlag[ChannelType::CHROMA], "sh_sao_chroma_used_flag");
    }
  }
  codeCcSao(pcSlice, picHeader, pcSlice->m_sps, pcSlice->m_ccSaoComParam);
  if (pcSlice->m_pps->m_deblockingFilterControlPresentFlag)
  {
    if (pcSlice->m_pps->m_deblockingFilterOverrideEnabledFlag && !pcSlice->m_pps->m_dbfInfoInPhFlag)
    {
      xWriteFlag(pcSlice->m_deblockingFilterOverrideFlag, "sh_deblocking_params_present_flag");
    }
    else
    {
      CHECK(pcSlice->m_deblockingFilterOverrideFlag, "unexpected value");
    }
    if (pcSlice->m_deblockingFilterOverrideFlag)
    {
      if (!pcSlice->m_pps->m_ppsDeblockingFilterDisabledFlag)
      {
        xWriteFlag(pcSlice->m_deblockingFilterDisable, "sh_deblocking_filter_disabled_flag");
      }
      if (!pcSlice->m_deblockingFilterDisable)
      {
        xWriteSvlc(pcSlice->m_deblockingFilterBetaOffsetDiv2, "sh_luma_beta_offset_div2");
        xWriteSvlc(pcSlice->m_deblockingFilterTcOffsetDiv2, "sh_luma_tc_offset_div2");
        if (pcSlice->m_pps->m_usePPSChromaTool)
        {
          xWriteSvlc(pcSlice->m_deblockingFilterCbBetaOffsetDiv2, "sh_cb_beta_offset_div2");
          xWriteSvlc(pcSlice->m_deblockingFilterCbTcOffsetDiv2, "sh_cb_tc_offset_div2");
          xWriteSvlc(pcSlice->m_deblockingFilterCrBetaOffsetDiv2, "sh_cr_beta_offset_div2");
          xWriteSvlc(pcSlice->m_deblockingFilterCrTcOffsetDiv2, "sh_cr_tc_offset_div2");
        }
      }
    }
    else
    {
      CHECK(pcSlice->m_deblockingFilterDisable != picHeader->m_deblockingFilterDisable, "unexpected value");
      CHECK(pcSlice->m_deblockingFilterBetaOffsetDiv2 != picHeader->m_deblockingFilterBetaOffsetDiv2,
            "unexpected value");
      CHECK(pcSlice->m_deblockingFilterTcOffsetDiv2 != picHeader->m_deblockingFilterTcOffsetDiv2, "unexpected value");
      CHECK(pcSlice->m_deblockingFilterCbBetaOffsetDiv2 != picHeader->m_deblockingFilterCbBetaOffsetDiv2,
            "unexpected value");
      CHECK(pcSlice->m_deblockingFilterCbTcOffsetDiv2 != picHeader->m_deblockingFilterCbTcOffsetDiv2,
            "unexpected value");
      CHECK(pcSlice->m_deblockingFilterCrBetaOffsetDiv2 != picHeader->m_deblockingFilterCrBetaOffsetDiv2,
            "unexpected value");
      CHECK(pcSlice->m_deblockingFilterCrTcOffsetDiv2 != picHeader->m_deblockingFilterCrTcOffsetDiv2,
            "unexpected value");
    }
  }
  else
  {
    CHECK(pcSlice->m_deblockingFilterDisable, "unexpected value");
    CHECK(pcSlice->m_deblockingFilterBetaOffsetDiv2, "unexpected value");
    CHECK(pcSlice->m_deblockingFilterTcOffsetDiv2, "unexpected value");
    CHECK(pcSlice->m_deblockingFilterCbBetaOffsetDiv2, "unexpected value");
    CHECK(pcSlice->m_deblockingFilterCbTcOffsetDiv2, "unexpected value");
    CHECK(pcSlice->m_deblockingFilterCrBetaOffsetDiv2, "unexpected value");
    CHECK(pcSlice->m_deblockingFilterCrTcOffsetDiv2, "unexpected value");
  }

  // dependent quantization
  if (pcSlice->m_sps->m_depQuantEnabledFlag)
  {
    CHECK(pcSlice->m_depQuantEnabledIdc > 2, "Invalid value of sh_dep_quant_used_idc");
    xWriteCode(pcSlice->m_depQuantEnabledIdc, 2, "sh_dep_quant_used_idc");
  }
  else
  {
    CHECK(pcSlice->m_depQuantEnabledIdc, "unexpected value");
  }

  // sign data hiding
  if (pcSlice->m_sps->m_signDataHidingEnabledFlag && !pcSlice->m_depQuantEnabledIdc)
  {
    xWriteFlag(pcSlice->m_signDataHidingEnabledFlag, "sh_sign_data_hiding_used_flag");
  }
  else
  {
    CHECK(pcSlice->m_signDataHidingEnabledFlag, "unexpected value");
  }

  // signal TS residual coding disabled flag
  if (pcSlice->m_sps->m_transformSkipEnabledFlag && !pcSlice->m_depQuantEnabledIdc &&
      !pcSlice->m_signDataHidingEnabledFlag)
  {
    xWriteFlag(pcSlice->m_tsResidualCodingDisabledFlag ? 1 : 0, "sh_ts_residual_coding_disabled_flag");
  }

  if ((!pcSlice->m_tsResidualCodingDisabledFlag) && (pcSlice->m_sps->m_spsRangeExtension.m_tsrcRicePresentFlag))
  {
    xWriteCode(pcSlice->m_tsrcIndex, 3, "sh_ts_residual_coding_rice_idx_minus1");
  }
  if (pcSlice->m_sps->m_spsRangeExtension.m_reverseLastSigCoeffEnabledFlag)
  {
    xWriteFlag(pcSlice->m_reverseLastSigCoeffFlag, "sh_reverse_last_sig_coeff_flag");
  }
  if (pcSlice->m_sps->m_licEnabledFlag && !pcSlice->isIntra())
  {
    xWriteFlag(pcSlice->m_useLic ? 1 : 0, "slice_lic_enable_flag");
  }
  if (pcSlice->m_pps->m_sliceHeaderExtensionPresentFlag)
  {
    xWriteUvlc(0, "sh_slice_header_extension_length");
  }

  codeClippingValues(pcSlice);
}

void HLSWriter::codeClippingValues(const Slice *pcSlice)
{
  xWriteFlag(pcSlice->m_adaptiveClipQuant ? 1 : 0, "adaptive_clip_quant");

  int deltaMax, deltaMin;
  if (pcSlice->isIntra())
  {
    deltaMax =
      pcSlice->m_pic->m_lumaClpRngforQuant.max - (235 * (1 << (pcSlice->m_sps->m_bitDepths[ChannelType::LUMA] - 8)));
    deltaMin =
      pcSlice->m_pic->m_lumaClpRngforQuant.min - 16 * (1 << (pcSlice->m_sps->m_bitDepths[ChannelType::LUMA] - 8));
  }
  else
  {
    const Picture *const pColPic =
      pcSlice->getRefPic(RefPicList(1 - pcSlice->m_colFromL0Flag), pcSlice->m_colRefIdx)->m_unscaledPic;
    ClpRng colLumaClpRng = pColPic->m_lumaClpRng;
    int    colLumaMin    = colLumaClpRng.min;
    int    colLumaMax    = colLumaClpRng.max;
    deltaMin             = pcSlice->m_pic->m_lumaClpRngforQuant.min - colLumaMin;
    deltaMax             = pcSlice->m_pic->m_lumaClpRngforQuant.max - colLumaMax;
  }

  int clipDeltaShift = pcSlice->getClipDeltaShift();

  if (deltaMax > 0)
  {
    deltaMax = (deltaMax >> clipDeltaShift);
  }
  else if (deltaMax < 0)
  {
    deltaMax = -((-deltaMax) >> clipDeltaShift);
  }
  if (deltaMin > 0)
  {
    deltaMin = (deltaMin >> clipDeltaShift);
  }
  else if (deltaMin < 0)
  {
    deltaMin = -((-deltaMin) >> clipDeltaShift);
  }

  xWriteSvlc(deltaMax, "clip_luma_pel_max");
  xWriteSvlc(deltaMin, "clip_luma_pel_min");
}

void HLSWriter::codeConstraintInfo(const ConstraintInfo *cinfo, const ProfileTierLevel *ptl)
{
  xWriteFlag(cinfo->m_gciPresentFlag, "gci_present_flag");
  if (cinfo->m_gciPresentFlag)
  {
    /* general */
    xWriteFlag(cinfo->m_intraOnlyConstraintFlag, "gci_intra_only_constraint_flag");
    xWriteFlag(cinfo->m_allLayersIndependentConstraintFlag, "gci_all_layers_independent_constraint_flag");
    xWriteFlag(cinfo->m_onePictureOnlyConstraintFlag, "gci_one_au_only_constraint_flag");

    /* picture format */
    xWriteCode(16 - cinfo->m_maxBitDepthConstraintIdc, 4, "gci_sixteen_minus_max_bitdepth_constraint_idc");
    xWriteCode(3 - to_underlying(cinfo->m_maxChromaFormatConstraintIdc), 2,
               "gci_three_minus_max_chroma_format_constraint_idc");

    /* NAL unit type related */
    xWriteFlag(cinfo->m_noMixedNaluTypesInPicConstraintFlag, "gci_no_mixed_nalu_types_in_pic_constraint_flag");
    xWriteFlag(cinfo->m_noTrailConstraintFlag, "gci_no_trail_constraint_flag");
    xWriteFlag(cinfo->m_noStsaConstraintFlag, "gci_no_stsa_constraint_flag");
    xWriteFlag(cinfo->m_noRaslConstraintFlag, "gci_no_rasl_constraint_flag");
    xWriteFlag(cinfo->m_noRadlConstraintFlag, "gci_no_radl_constraint_flag");
    xWriteFlag(cinfo->m_noIdrConstraintFlag, "gci_no_idr_constraint_flag");
    xWriteFlag(cinfo->m_noCraConstraintFlag, "gci_no_cra_constraint_flag");
    xWriteFlag(cinfo->m_noGdrConstraintFlag, "gci_no_gdr_constraint_flag");
    xWriteFlag(cinfo->m_noApsConstraintFlag, "gci_no_aps_constraint_flag");
    xWriteFlag(cinfo->m_noIdrRplConstraintFlag, "gci_no_idr_rpl_constraint_flag");

    /* tile, slice, subpicture partitioning */
    xWriteFlag(cinfo->m_oneTilePerPicConstraintFlag, "gci_one_tile_per_pic_constraint_flag");
    xWriteFlag(cinfo->m_picHeaderInSliceHeaderConstraintFlag, "gci_pic_header_in_slice_header_constraint_flag");
    xWriteFlag(cinfo->m_oneSlicePerPicConstraintFlag, "gci_one_slice_per_pic_constraint_flag");
    xWriteFlag(cinfo->m_noRectSliceConstraintFlag, "gci_no_rectangular_slice_constraint_flag");
    xWriteFlag(cinfo->m_oneSlicePerSubpicConstraintFlag, "gci_one_slice_per_subpic_constraint_flag");
    xWriteFlag(cinfo->m_noSubpicInfoConstraintFlag, "gci_no_subpic_info_constraint_flag");

    /* CTU and block partitioning */
    xWriteCode(3 - (cinfo->m_maxLog2CtuSizeConstraintIdc - 5), 2, "gci_three_minus_max_log2_ctu_size_constraint_idc");
    xWriteFlag(cinfo->m_noPartitionConstraintsOverrideConstraintFlag,
               "gci_no_partition_constraints_override_constraint_flag");
    xWriteFlag(cinfo->m_noMttConstraintFlag, "gci_no_mtt_constraint_flag");
    xWriteFlag(cinfo->m_noQtbttDualTreeIntraConstraintFlag, "gci_no_qtbtt_dual_tree_intra_constraint_flag");

    /* intra */
    xWriteFlag(cinfo->m_noPaletteConstraintFlag, "gci_no_palette_constraint_flag");
    xWriteFlag(cinfo->m_noIbcConstraintFlag, "gci_no_ibc_constraint_flag");
    xWriteFlag(cinfo->m_noMrlConstraintFlag, "gci_no_mrl_constraint_flag");
    xWriteFlag(cinfo->m_noMipConstraintFlag, "gci_no_mip_constraint_flag");
    xWriteFlag(cinfo->m_noCclmConstraintFlag, "gci_no_cclm_constraint_flag");
    xWriteFlag(cinfo->m_noSgpmConstraintFlag, "gci_no_sgpm_constraint_flag");

    /* inter */
    xWriteFlag(cinfo->m_noRprConstraintFlag, "gci_no_ref_pic_resampling_constraint_flag");
    xWriteFlag(cinfo->m_noResChangeInClvsConstraintFlag, "gci_no_res_change_in_clvs_constraint_flag");
    xWriteFlag(cinfo->m_noWeightedPredictionConstraintFlag, "gci_no_weighted_prediction_constraint_flag");
    xWriteFlag(cinfo->m_noRefWraparoundConstraintFlag, "gci_no_ref_wraparound_constraint_flag");
    xWriteFlag(cinfo->m_noTemporalMvpConstraintFlag, "gci_no_temporal_mvp_constraint_flag");
    xWriteFlag(cinfo->m_noSbtmvpConstraintFlag, "gci_no_sbtmvp_constraint_flag");
    xWriteFlag(cinfo->m_noAmvrConstraintFlag, "gci_no_amvr_constraint_flag");
    xWriteFlag(cinfo->m_noBdofConstraintFlag, "gci_no_bdof_constraint_flag");
    xWriteFlag(cinfo->m_noSmvdConstraintFlag, "gci_no_smvd_constraint_flag");
    xWriteFlag(cinfo->m_noDmvrConstraintFlag, "gci_no_dmvr_constraint_flag");
    xWriteFlag(cinfo->m_noMmvdConstraintFlag, "gci_no_mmvd_constraint_flag");
    xWriteFlag(cinfo->m_noAffineMotionConstraintFlag, "gci_no_affine_motion_constraint_flag");
    xWriteFlag(cinfo->m_noProfConstraintFlag, "gci_no_prof_constraint_flag");
    xWriteFlag(cinfo->m_noBcwConstraintFlag, "gci_no_bcw_constraint_flag");
    xWriteFlag(cinfo->m_noCiipConstraintFlag, "gci_no_ciip_constraint_flag");
    xWriteFlag(cinfo->m_noGeoConstraintFlag, "gci_no_gpm_constraint_flag");
    xWriteFlag(cinfo->m_noObmcConstraintFlag, "gci_no_obmc_constraint_flag");

    /* transform, quantization, residual */
    xWriteFlag(cinfo->m_noLumaTransformSize64ConstraintFlag, "gci_no_luma_transform_size_64_constraint_flag");
    xWriteFlag(cinfo->m_noTransformSkipConstraintFlag, "gci_no_transform_skip_constraint_flag");
    xWriteFlag(cinfo->m_noBDPCMConstraintFlag, "gci_no_bdpcm_constraint_flag");
    xWriteFlag(cinfo->m_noMtsConstraintFlag, "gci_no_mts_constraint_flag");
    xWriteFlag(cinfo->m_noLfnstConstraintFlag, "gci_no_lfnst_constraint_flag");
    xWriteFlag(cinfo->m_noJointCbCrConstraintFlag, "gci_no_joint_cbcr_constraint_flag");
    xWriteFlag(cinfo->m_noSbtConstraintFlag, "gci_no_sbt_constraint_flag");
    xWriteFlag(cinfo->m_noActConstraintFlag, "gci_no_act_constraint_flag");
    xWriteFlag(cinfo->m_noExplicitScaleListConstraintFlag, "gci_no_explicit_scaling_list_constraint_flag");
    xWriteFlag(cinfo->m_noDepQuantConstraintFlag, "gci_no_dep_quant_constraint_flag");
    xWriteFlag(cinfo->m_noSignDataHidingConstraintFlag, "gci_no_sign_data_hiding_constraint_flag");
    xWriteFlag(cinfo->m_noCuQpDeltaConstraintFlag, "gci_no_cu_qp_delta_constraint_flag");
    xWriteFlag(cinfo->m_noChromaQpOffsetConstraintFlag, "gci_no_chroma_qp_offset_constraint_flag");

    /* loop filter */
    xWriteFlag(cinfo->m_noSaoConstraintFlag, "gci_no_sao_constraint_flag");
    xWriteFlag(cinfo->m_noCCSaoConstraintFlag, "gci_no_ccsao_constraint_flag");
    xWriteFlag(cinfo->m_noAlfConstraintFlag, "gci_no_alf_constraint_flag");
    xWriteFlag(cinfo->m_noCCAlfConstraintFlag, "gci_no_ccalf_constraint_flag");
    xWriteFlag(cinfo->m_noLmcsConstraintFlag, "gci_no_lmcs_constraint_flag");
    xWriteFlag(cinfo->m_noLadfConstraintFlag, "gci_no_ladf_constraint_flag");
    Profile::Name profile = ptl->m_profileIdc;
    if (profile == Profile::MAIN_12 || profile == Profile::MAIN_12_INTRA || profile == Profile::MAIN_12_STILL_PICTURE ||
        profile == Profile::MAIN_12_444 || profile == Profile::MAIN_12_444_INTRA ||
        profile == Profile::MAIN_12_444_STILL_PICTURE || profile == Profile::MAIN_16_444 ||
        profile == Profile::MAIN_16_444_INTRA || profile == Profile::MAIN_16_444_STILL_PICTURE)
    {
      int numAdditionalBits = 6;
      xWriteCode(numAdditionalBits, 8, "gci_num_additional_bits");
      xWriteFlag(cinfo->m_allRapPicturesFlag, "gci_all_rap_pictures_flag");
      xWriteFlag(cinfo->m_noExtendedPrecisionProcessingConstraintFlag,
                 "gci_no_extended_precision_processing_constraint_flag");
      xWriteFlag(cinfo->m_noTsResidualCodingRiceConstraintFlag, "gci_no_ts_residual_coding_rice_constraint_flag");
      xWriteFlag(cinfo->m_noRrcRiceExtensionConstraintFlag, "gci_no_rrc_rice_extension_constraint_flag");
      xWriteFlag(cinfo->m_noPersistentRiceAdaptationConstraintFlag,
                 "gci_no_persistent_rice_adaptation_constraint_flag");
      xWriteFlag(cinfo->m_noReverseLastSigCoeffConstraintFlag, "gci_no_reverse_last_sig_coeff_constraint_flag");
    }
    else
    {
      xWriteCode(0, 8, "gci_num_additional_bits");
    }
  }

  while (!isByteAligned())
  {
    xWriteFlag(0, "gci_alignment_zero_bit");
  }
}

void HLSWriter::codeProfileTierLevel(const ProfileTierLevel *ptl, bool profileTierPresentFlag,
                                     int maxNumSubLayersMinus1)
{
  if (profileTierPresentFlag)
  {
    xWriteCode(int(ptl->m_profileIdc), 7, "general_profile_idc");
    xWriteFlag(ptl->m_tierFlag == Level::HIGH, "general_tier_flag");
  }

  xWriteCode(int(ptl->m_levelIdc), 8, "general_level_idc");

  xWriteFlag(ptl->m_frameOnlyConstraintFlag, "ptl_frame_only_constraint_flag");
  xWriteFlag(ptl->m_multiLayerEnabledFlag, "ptl_multilayer_enabled_flag");

  if (profileTierPresentFlag)
  {
    codeConstraintInfo(&ptl->m_constraintInfo, ptl);
  }

  for (int i = maxNumSubLayersMinus1 - 1; i >= 0; i--)
  {
    xWriteFlag(ptl->m_subLayerLevelPresentFlag[i], "sub_layer_level_present_flag[i]");
  }

  while (!isByteAligned())
  {
    xWriteFlag(0, "ptl_reserved_zero_bit");
  }

  for (int i = maxNumSubLayersMinus1 - 1; i >= 0; i--)
  {
    if (ptl->m_subLayerLevelPresentFlag[i])
    {
      xWriteCode(int(ptl->m_subLayerLevelIdc[i]), 8, "sub_layer_level_idc[i]");
    }
  }

  if (profileTierPresentFlag)
  {
    int numSubProfile = (int)ptl->m_subProfileIdc.size();
    xWriteCode(numSubProfile, 8, "ptl_num_sub_profiles");
    for (int i = 0; i < numSubProfile; i++)
    {
      xWriteCode(ptl->m_subProfileIdc[i], 32, "general_sub_profile_idc[i]");
    }
  }
}

/**
 * Write tiles and wavefront substreams sizes for the slice header (entry points).
 *
 * \param pSlice Slice structure that contains the substream size information.
 */
void HLSWriter::codeTilesWPPEntryPoint(const Slice *pSlice)
{
  if (pSlice->m_numEntryPoints == 0)
  {
    return;
  }
  uint32_t       maxOffset         = 0;
  const unsigned numSubstreamSizes = (unsigned)pSlice->m_substreamSizes.size();
  for (int idx = 0; idx < numSubstreamSizes; idx++)
  {
    uint32_t offset = pSlice->m_substreamSizes[idx];
    if (offset > maxOffset)
    {
      maxOffset = offset;
    }
  }

  // Determine number of bits "offsetLenMinus1+1" required for entry point information
  uint32_t offsetLenMinus1 = 0;
  while (maxOffset >= (1u << (offsetLenMinus1 + 1)))
  {
    offsetLenMinus1++;
    CHECK(offsetLenMinus1 + 1 >= 32, "Invalid offset length minus 1");
  }

  if (numSubstreamSizes > 0)
  {
    xWriteUvlc(offsetLenMinus1, "sh_entry_offset_len_minus1");
    for (uint32_t idx = 0; idx < numSubstreamSizes; idx++)
    {
      xWriteCode(pSlice->m_substreamSizes[idx] - 1, offsetLenMinus1 + 1, "sh_entry_point_offset_minus1");
    }
  }
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

//! Code weighted prediction tables
void HLSWriter::xCodePredWeightTable(const Slice *pcSlice)
{
  const WPScalingParam *wp;
  const ChromaFormat    format                    = pcSlice->m_sps->m_chromaFormatIdc;
  const uint32_t        numberValidComponents     = getNumberValidComponents(format);
  const bool            hasChroma                 = isChromaEnabled(format);
  uint32_t              totalSignalledWeightFlags = 0;

  wp = pcSlice->getWpScaling(RPL0, 0);

  xWriteUvlc(wp[COMP_Y].log2WeightDenom, "luma_log2_weight_denom");

  if (hasChroma)
  {
    CHECK(wp[COMP_Cb].log2WeightDenom != wp[COMP_Cr].log2WeightDenom, "Chroma blocks of different size not supported");
    const int deltaDenom = (wp[COMP_Cb].log2WeightDenom - wp[COMP_Y].log2WeightDenom);
    xWriteSvlc(deltaDenom, "delta_chroma_log2_weight_denom");
  }

  for (const auto l: { RPL0, RPL1 })
  {
    const bool l0 = l == RPL0;

    if (!l0 && !pcSlice->isInterB())
    {
      continue;
    }

    // NOTE: wp[].log2WeightDenom and wp[].presentFlag are actually per-channel-type settings.

    for (int refIdx = 0; refIdx < pcSlice->m_numRefIdx[l]; refIdx++)
    {
      wp = pcSlice->getWpScaling(l, refIdx);
      xWriteFlag(wp[COMP_Y].presentFlag, (l0 ? "luma_weight_l0_flag[i]" : "luma_weight_l1_flag[i]"));
      totalSignalledWeightFlags += wp[COMP_Y].presentFlag;
    }
    if (hasChroma)
    {
      for (int refIdx = 0; refIdx < pcSlice->m_numRefIdx[l]; refIdx++)
      {
        wp = pcSlice->getWpScaling(l, refIdx);
        CHECK(wp[COMP_Cb].presentFlag != wp[COMP_Cr].presentFlag, "Inconsistent settings for chroma channels");
        xWriteFlag(wp[COMP_Cb].presentFlag, (l0 ? "chroma_weight_l0_flag[i]" : "chroma_weight_l1_flag[i]"));
        totalSignalledWeightFlags += 2 * wp[COMP_Cb].presentFlag;
      }
    }

    for (int refIdx = 0; refIdx < pcSlice->m_numRefIdx[l]; refIdx++)
    {
      wp = pcSlice->getWpScaling(l, refIdx);
      if (wp[COMP_Y].presentFlag)
      {
        int deltaWeight = (wp[COMP_Y].codedWeight - (1 << wp[COMP_Y].log2WeightDenom));
        xWriteSvlc(deltaWeight, (l0 ? "delta_luma_weight_l0[i]" : "delta_luma_weight_l1[i]"));
        xWriteSvlc(wp[COMP_Y].codedOffset, (l0 ? "luma_offset_l0[i]" : "luma_offset_l1[i]"));
      }

      if (hasChroma)
      {
        if (wp[COMP_Cb].presentFlag)
        {
          for (int j = COMP_Cb; j < numberValidComponents; j++)
          {
            CHECK(wp[COMP_Cb].log2WeightDenom != wp[COMP_Cr].log2WeightDenom,
                  "Chroma blocks of different size not supported");
            int deltaWeight = (wp[j].codedWeight - (1 << wp[COMP_Cb].log2WeightDenom));
            xWriteSvlc(deltaWeight, (l0 ? "delta_chroma_weight_l0[i]" : "delta_chroma_weight_l1[i]"));

            int range       = pcSlice->m_sps->m_spsRangeExtension.m_highPrecisionOffsetsEnabledFlag
                    ? (1 << pcSlice->m_sps->m_bitDepths[ChannelType::CHROMA]) / 2
                    : 128;
            int pred        = (range - ((range * wp[j].codedWeight) >> (wp[j].log2WeightDenom)));
            int deltaChroma = (wp[j].codedOffset - pred);
            xWriteSvlc(deltaChroma, (l0 ? "delta_chroma_offset_l0[i]" : "delta_chroma_offset_l1[i]"));
          }
        }
      }
    }
  }
  CHECK(totalSignalledWeightFlags > 24, "Too many signalled weight flags");
}

void HLSWriter::xCodePredWeightTable(const PicHeader *picHeader, const PPS *pps, const SPS *sps)
{
  const WPScalingParam *wp;
  const ChromaFormat    format                    = sps->m_chromaFormatIdc;
  const uint32_t        numberValidComponents     = getNumberValidComponents(format);
  const bool            chroma                    = isChromaEnabled(format);
  uint32_t              totalSignalledWeightFlags = 0;

  wp = picHeader->getWpScaling(RPL0, 0);
  xWriteUvlc(wp[COMP_Y].log2WeightDenom, "luma_log2_weight_denom");

  if (chroma)
  {
    CHECK(wp[COMP_Cb].log2WeightDenom != wp[COMP_Cr].log2WeightDenom, "Chroma blocks of different size not supported");
    const int deltaDenom = (wp[COMP_Cb].log2WeightDenom - wp[COMP_Y].log2WeightDenom);
    xWriteSvlc(deltaDenom, "delta_chroma_log2_weight_denom");
  }

  for (const auto l: { RPL0, RPL1 })
  {
    const bool l0 = l == RPL0;

    int numLxWeights = 0;
    if (l0 || pps->m_useBiWP)
    {
      numLxWeights = picHeader->m_numWeights[l];
      xWriteUvlc(numLxWeights, (l0 ? "num_l0_weights" : "num_l1_weights"));
    }

    for (int refIdx = 0; refIdx < numLxWeights; refIdx++)
    {
      wp = picHeader->getWpScaling(l, refIdx);
      xWriteFlag(wp[COMP_Y].presentFlag, (l0 ? "luma_weight_l0_flag[i]" : "luma_weight_l1_flag[i]"));
      totalSignalledWeightFlags += wp[COMP_Y].presentFlag;
    }

    if (chroma)
    {
      for (int refIdx = 0; refIdx < numLxWeights; refIdx++)
      {
        wp = picHeader->getWpScaling(l, refIdx);
        CHECK(wp[COMP_Cb].presentFlag != wp[COMP_Cr].presentFlag, "Inconsistent settings for chroma channels");
        xWriteFlag(wp[COMP_Cb].presentFlag, (l0 ? "chroma_weight_l0_flag[i]" : "chroma_weight_l1_flag[i]"));
        totalSignalledWeightFlags += 2 * wp[COMP_Cb].presentFlag;
      }
    }

    for (int refIdx = 0; refIdx < numLxWeights; refIdx++)
    {
      wp = picHeader->getWpScaling(l, refIdx);
      if (wp[COMP_Y].presentFlag)
      {
        int deltaWeight = (wp[COMP_Y].codedWeight - (1 << wp[COMP_Y].log2WeightDenom));
        xWriteSvlc(deltaWeight, (l0 ? "delta_luma_weight_l0[i]" : "delta_luma_weight_l1[i]"));
        xWriteSvlc(wp[COMP_Y].codedOffset, (l0 ? "luma_offset_l0[i]" : "luma_offset_l1[i]"));
      }

      if (chroma)
      {
        if (wp[COMP_Cb].presentFlag)
        {
          for (int j = COMP_Cb; j < numberValidComponents; j++)
          {
            CHECK(wp[COMP_Cb].log2WeightDenom != wp[COMP_Cr].log2WeightDenom,
                  "Chroma blocks of different size not supported");
            int deltaWeight = (wp[j].codedWeight - (1 << wp[COMP_Cb].log2WeightDenom));
            xWriteSvlc(deltaWeight, (l0 ? "delta_chroma_weight_l0[i]" : "delta_chroma_weight_l1[i]"));

            int range       = sps->m_spsRangeExtension.m_highPrecisionOffsetsEnabledFlag
                    ? (1 << sps->m_bitDepths[ChannelType::CHROMA]) / 2
                    : 128;
            int pred        = (range - ((range * wp[j].codedWeight) >> (wp[j].log2WeightDenom)));
            int deltaChroma = (wp[j].codedOffset - pred);
            xWriteSvlc(deltaChroma, (l0 ? "delta_chroma_offset_l0[i]" : "delta_chroma_offset_l1[i]"));
          }
        }
      }
    }
  }

  CHECK(totalSignalledWeightFlags > 24, "Too many signalled weight flags");
}

/** code quantization matrix
 *  \param scalingList quantization matrix information
 */
void HLSWriter::codeScalingList(const ScalingList &scalingList, bool aps_chromaPresentFlag)
{
  // for each size
  for (uint32_t scalingListId = 0; scalingListId < 28; scalingListId++)
  {
    if (aps_chromaPresentFlag || scalingList.isLumaScalingList(scalingListId))
    {
      bool scalingListCopyModeFlag = scalingList.m_scalingListPredModeFlagIsCopy[scalingListId];
      xWriteFlag(scalingListCopyModeFlag, "scaling_list_copy_mode_flag");   // copy mode
      if (!scalingListCopyModeFlag)   // Copy Mode
      {
        xWriteFlag(scalingList.m_scalingListPreditorModeFlag[scalingListId], "scaling_list_predictor_mode_flag");
      }
      if ((scalingListCopyModeFlag || scalingList.m_scalingListPreditorModeFlag[scalingListId]) &&
          scalingListId != SCALING_LIST_1D_START_2x2 && scalingListId != SCALING_LIST_1D_START_4x4 &&
          scalingListId != SCALING_LIST_1D_START_8x8)
      {
        xWriteUvlc((int)scalingListId - (int)scalingList.m_refMatrixId[scalingListId],
                   "scaling_list_pred_matrix_id_delta");
      }
      if (!scalingListCopyModeFlag)
      {
        // DPCM
        xCodeScalingList(&scalingList, scalingListId, scalingList.m_scalingListPreditorModeFlag[scalingListId]);
      }
    }
  }
  return;
}
/** code DPCM
 * \param scalingList quantization matrix information
 * \param sizeId      size index
 * \param listId      list index
 */
void HLSWriter::xCodeScalingList(const ScalingList *scalingList, uint32_t scalingListId, bool isPredictor)
{
  int matrixSize =
    (scalingListId < SCALING_LIST_1D_START_4x4) ? 2 : ((scalingListId < SCALING_LIST_1D_START_8x8) ? 4 : 8);
  int          coefNum = matrixSize * matrixSize;
  ScanElement *scan    = g_scanOrder[SCAN_UNGROUPED][CoeffScanType::DIAG][gp_sizeIdxInfo->idxFrom(matrixSize)]
                                 [gp_sizeIdxInfo->idxFrom(matrixSize)];
  int nextCoef = (isPredictor) ? 0 : SCALING_LIST_START_VALUE;

  int        data;
  const int *src          = scalingList->getScalingListAddress(scalingListId);
  int        predListId   = scalingList->m_refMatrixId[scalingListId];
  const int *srcPred      = (isPredictor)
         ? ((scalingListId == predListId) ? scalingList->getScalingListDefaultAddress(scalingListId)
                                          : scalingList->getScalingListAddress(predListId))
         : nullptr;
  int        deltasrc[65] = { 0 };

  if (isPredictor)
  {
    if (scalingListId >= SCALING_LIST_1D_START_16x16)
    {
      deltasrc[64] = scalingList->m_scalingListDC[scalingListId] -
        ((predListId >= SCALING_LIST_1D_START_16x16)
           ? ((scalingListId == predListId) ? 16 : scalingList->m_scalingListDC[predListId])
           : srcPred[scan[0].idx]);
    }
    for (int i = 0; i < coefNum; i++)
    {
      deltasrc[i] = (src[scan[i].idx] - srcPred[scan[i].idx]);
    }
  }
  if (scalingListId >= SCALING_LIST_1D_START_16x16)
  {
    if (isPredictor)
    {
      data     = deltasrc[64];
      nextCoef = deltasrc[64];
    }
    else
    {
      data     = scalingList->m_scalingListDC[scalingListId] - nextCoef;
      nextCoef = scalingList->m_scalingListDC[scalingListId];
    }
    data = ((data + 128) & 255) - 128;
    xWriteSvlc((int8_t)data, "scaling_list_dc_coef");
  }
  for (int i = 0; i < coefNum; i++)
  {
    if (scalingListId >= SCALING_LIST_1D_START_64x64 && scan[i].x >= 4 && scan[i].y >= 4)
    {
      continue;
    }
    data     = (isPredictor) ? (deltasrc[i] - nextCoef) : (src[scan[i].idx] - nextCoef);
    nextCoef = (isPredictor) ? deltasrc[i] : src[scan[i].idx];
    data     = ((data + 128) & 255) - 128;
    xWriteSvlc((int8_t)data, "scaling_list_delta_coef");
  }
}

bool HLSWriter::xFindMatchingLTRP(const Slice *pcSlice, uint32_t *ltrpsIndex, int ltrpPOC, bool usedFlag)
{
  // bool state = true, state2 = false;
  int lsb = ltrpPOC & ((1 << pcSlice->m_sps->m_bitsForPoc) - 1);
  for (int k = 0; k < pcSlice->m_sps->m_numLongTermRefPicSPS; k++)
  {
    if ((lsb == pcSlice->m_sps->m_ltRefPicPocLsbSps[k]) && (usedFlag == pcSlice->m_sps->m_usedByCurrPicLtSPSFlag[k]))
    {
      *ltrpsIndex = k;
      return true;
    }
  }
  return false;
}

void HLSWriter::codeCcSao(const Slice *pcSlice, PicHeader *picHeader, const SPS *sps, const CcSaoComParam &ccSaoParam)
{
  if (pcSlice->m_sps->m_ccSaoEnabledFlag)
  {
    xWriteFlag(ccSaoParam.enabled[COMP_Y] ? 1 : 0, "ccsao_y_enabled_flag");
    xWriteFlag(ccSaoParam.enabled[COMP_Cb] ? 1 : 0, "ccsao_cb_enabled_flag");
    xWriteFlag(ccSaoParam.enabled[COMP_Cr] ? 1 : 0, "ccsao_cr_enabled_flag");

    if (ccSaoParam.enabled[COMP_Y] || ccSaoParam.enabled[COMP_Cb] || ccSaoParam.enabled[COMP_Cr])
    {
      xWriteFlag(ccSaoParam.extChroma[COMP_Y] ? 1 : 0, "ccsao_ext_chroma_flag");
    }

    for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
    {
      if (ccSaoParam.enabled[compIdx])
      {
        if (!pcSlice->isIntra())
        {
          xWriteFlag(ccSaoParam.reusePrv[compIdx] ? 1 : 0, "ccsao_reuse_prv_flag");
        }
        if (ccSaoParam.reusePrv[compIdx])
        {
          CHECK(ccSaoParam.reusePrvId[compIdx] < 0 || ccSaoParam.reusePrvId[compIdx] >= MAX_CCSAO_PRV_NUM,
                "CCSAO reusePrvId out of range");
          xWriteCode(ccSaoParam.reusePrvId[compIdx], MAX_CCSAO_PRV_NUM_BITS, "ccsao_reuse_prv_id");
          continue;
        }
        CHECK(ccSaoParam.setNum[compIdx] == 0 || ccSaoParam.setNum[compIdx] > MAX_CCSAO_SET_NUM,
              "CCSAO setNum out of range");
        xWriteUvlc(ccSaoParam.setNum[compIdx] - 1, "ccsao_set_num");

        for (int setIdx = 0; setIdx < ccSaoParam.setNum[compIdx]; setIdx++)
        {
          xWriteFlag(ccSaoParam.setType[compIdx][setIdx] ? 1 : 0, "ccsao_set_type");
          if (ccSaoParam.setType[compIdx][setIdx] == CCSAO_SET_TYPE_EDGE)
          {
            CHECK(ccSaoParam.candPos[compIdx][setIdx][COMP_Cb] >= MAX_NUM_COMP, "CCSAO edgeCmp out of range");
            CHECK(ccSaoParam.bandNum[compIdx][setIdx][COMP_Cr] >= MAX_CCSAO_EDGE_IDC, "CCSAO edgeIdc out of range");
            CHECK(ccSaoParam.candPos[compIdx][setIdx][COMP_Y] >= MAX_CCSAO_EDGE_DIR, "CCSAO edgeDir out of range");
            CHECK(ccSaoParam.bandNum[compIdx][setIdx][COMP_Cb] >= MAX_CCSAO_EDGE_THR, "CCSAO edgeThr out of range");
            CHECK(ccSaoParam.bandNum[compIdx][setIdx][COMP_Y] >= MAX_CCSAO_BAND_IDC, "CCSAO bandIdc out of range");

            if (ccSaoParam.extChroma[compIdx])
            {
              xWriteCode(ccSaoParam.candPos[compIdx][setIdx][COMP_Cb], MAX_CCSAO_EDGE_CMP_BITS, "ccsao_edge_cmp");
            }
            xWriteCode(ccSaoParam.bandNum[compIdx][setIdx][COMP_Cr], MAX_CCSAO_EDGE_IDC_BITS, "ccsao_edge_idc");
            xWriteCode(ccSaoParam.candPos[compIdx][setIdx][COMP_Y], MAX_CCSAO_EDGE_DIR_BITS, "ccsao_edge_dir");
            if (pcSlice->m_sps->m_ccSaoFastFlag)
            {
              xWriteCode(ccSaoParam.bandNum[compIdx][setIdx][COMP_Cb], 2, "ccsao_edge_thr");
            }
            else
            {
              xWriteCode(ccSaoParam.bandNum[compIdx][setIdx][COMP_Cb], MAX_CCSAO_EDGE_THR_BITS, "ccsao_edge_thr");
            }
            xWriteCode(ccSaoParam.bandNum[compIdx][setIdx][COMP_Y], MAX_CCSAO_BAND_IDC_BITS, "ccsao_band_idc");
          }
          else
          {
            CHECK(ccSaoParam.candPos[compIdx][setIdx][COMP_Y] >= MAX_CCSAO_CAND_POS_Y, "CCSAO candPosY out of range");
            CHECK(ccSaoParam.bandNum[compIdx][setIdx][COMP_Y] == 0 ||
                    ccSaoParam.bandNum[compIdx][setIdx][COMP_Y] > MAX_CCSAO_BAND_NUM_Y,
                  "CCSAO bandNumY out of range");
            CHECK(ccSaoParam.bandNum[compIdx][setIdx][COMP_Cb] == 0 ||
                    ccSaoParam.bandNum[compIdx][setIdx][COMP_Cb] > MAX_CCSAO_BAND_NUM_U,
                  "CCSAO bandNumU out of range");
            CHECK(ccSaoParam.bandNum[compIdx][setIdx][COMP_Cr] == 0 ||
                    ccSaoParam.bandNum[compIdx][setIdx][COMP_Cr] > MAX_CCSAO_BAND_NUM_V,
                  "CCSAO bandNumV out of range");
            xWriteCode(ccSaoParam.candPos[compIdx][setIdx][COMP_Y], MAX_CCSAO_CAND_POS_Y_BITS, "ccsao_cand_pos_y");
            xWriteCode(ccSaoParam.bandNum[compIdx][setIdx][COMP_Y] - 1, MAX_CCSAO_BAND_NUM_Y_BITS, "ccsao_band_num_y");
            xWriteCode(ccSaoParam.bandNum[compIdx][setIdx][COMP_Cb] - 1, MAX_CCSAO_BAND_NUM_U_BITS, "ccsao_band_num_u");
            xWriteCode(ccSaoParam.bandNum[compIdx][setIdx][COMP_Cr] - 1, MAX_CCSAO_BAND_NUM_V_BITS, "ccsao_band_num_v");
          }

          const short *offset   = ccSaoParam.offset[compIdx][setIdx];
          const int    classNum = SampleAdaptiveOffset::getCcSaoClassNum(compIdx, setIdx, ccSaoParam);
          for (int i = 0; i < classNum; i++)
          {
            CHECK((offset[i] > MAX_CCSAO_OFFSET_THR || offset[i] < -MAX_CCSAO_OFFSET_THR), "CCSAO offset out of range");
            xWriteUvlc(abs(offset[i]), "ccsao_offset_abs");
            if (abs(offset[i]) != 0)
            {
              xWriteFlag((offset[i] < 0) ? 1 : 0, "ccsao_offset_sign");
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
}

void HLSWriter::alfHuffmanEncode(const int coeff, AlfHuffmanCode &huffman)
{
  int      length;
  uint32_t symbol;
  huffman.encodeCoeff(coeff, symbol, length);
  xWriteCode(symbol, length, "alf_coeff_huffman");
}

void HLSWriter::alfFilter(const AlfParametersVtm::AlfParam &alfParam, const bool isChroma, const int altIdx)
{
  AlfParametersVtm::AlfFilterShape alfShape(isChroma ? AlfParametersVtm::AlfFilterType::ALF_FILTER_5
                                                     : AlfParametersVtm::AlfFilterType::ALF_FILTER_7);

  const short *coeff      = isChroma ? alfParam.chromaCoeff[altIdx].data() : alfParam.lumaCoeff.data();
  const Pel   *clipp      = isChroma ? alfParam.chromaClipp[altIdx].data() : alfParam.lumaClipp.data();
  const int    numFilters = isChroma ? 1 : alfParam.numLumaFilters;

  const int offset = AlfParametersVtm::MAX_NUM_ALF_LUMA_COEFF;

  // vlc for all

  // Filter coefficients
  for (int ind = 0; ind < numFilters; ++ind)
  {
    for (int i = 0; i < alfShape.numCoeff - 1; i++)
    {
      xWriteUvlc(abs(coeff[ind * offset + i]),
                 isChroma ? "alf_chroma_coeff_abs"
                          : "alf_luma_coeff_abs");   // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
      if (abs(coeff[ind * offset + i]) != 0)
      {
        xWriteFlag((coeff[ind * offset + i] < 0) ? 1 : 0, isChroma ? "alf_chroma_coeff_sign" : "alf_luma_coeff_sign");
      }
    }
  }

  // Clipping values coding
  if (isChroma ? alfParam.chromaNonLinearFlag : alfParam.lumaNonLinearFlag)
  {
    for (int ind = 0; ind < numFilters; ++ind)
    {
      for (int i = 0; i < alfShape.numCoeff - 1; i++)
      {
        xWriteCode(clipp[ind * offset + i], 2, isChroma ? "alf_chroma_clip_idx" : "alf_luma_clip_idx");
      }
    }
  }
}

void HLSWriter::alfFilter(const AlfParametersEcm::AlfParam &alfParam, const bool isChroma, const int altIdx)
{
  AlfParametersEcm::AlfFilterType alfFilterType =
    isChroma ? alfParam.filterType[ChannelType::CHROMA] : alfParam.filterType[ChannelType::LUMA];
  AlfParametersEcm::AlfFilterShape alfShape(alfFilterType);

  const int     numFilters = isChroma ? 1 : alfParam.numLumaFilters[altIdx];
  const int8_t *scaleIdx   = isChroma ? alfParam.chromaScaleIdx[altIdx].data() : alfParam.lumaScaleIdx[altIdx].data();
  const short  *coeff      = isChroma ? alfParam.chromaCoeff[altIdx].data() : alfParam.lumaCoeff[altIdx].data();
  const Pel    *clipp      = isChroma ? alfParam.chromaClipp[altIdx].data() : alfParam.lumaClipp[altIdx].data();

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
      xWriteCode((uint32_t)scaleIdx[ind], AlfParametersEcm::ALF_SCALE_BITS_NUM,
                 isChroma ? "alf_chroma_scale_factor" : "alf_luma_scale_factor");
    }
    for (int i = 0; i < alfShape.numCoeff - 1; i++)
    {
      huffman.setGroup(i < alfShape.indexSecOrder ? 0 : 1);
      alfHuffmanEncode(coeff[ind * offset + i], huffman);
    }
  }

  // Clipping values coding
  if (isChroma ? alfParam.chromaNonLinearFlag[altIdx] : alfParam.lumaNonLinearFlag[altIdx])
  {
    for (int ind = 0; ind < numFilters; ++ind)
    {
      for (int i = 0; i < alfShape.numCoeff - 1; i++)
      {
        xWriteCode(clipp[ind * offset + i], 2, isChroma ? "alf_chroma_clip_idx" : "alf_luma_clip_idx");
      }
    }
  }
}

//! \}
