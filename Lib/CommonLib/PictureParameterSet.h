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

#pragma once

#include "Common.h"
#include "SequenceParameterSet.h"

struct PreCalcValues;

struct SliceMap
{
  uint32_t m_sliceID {
    0
  };   // slice identifier (slice index for rectangular slices, slice address for raser-scan slices)
  uint32_t              m_numTilesInSlice { 0 };   // number of tiles in slice (raster-scan slices only)
  uint32_t              m_numCtuInSlice { 0 };   // number of CTUs in the slice
  std::vector<uint32_t> m_ctuAddrInSlice;          // raster-scan addresses of all the CTUs in the slice

  uint32_t getCtuAddrInSlice(int idx) const
  {
    CHECK(idx >= m_ctuAddrInSlice.size(), "CTU index exceeds number of CTUs in slice.");
    return m_ctuAddrInSlice[idx];
  }

  void initSliceMap()
  {
    m_sliceID         = 0;
    m_numTilesInSlice = 0;
    m_numCtuInSlice   = 0;
    m_ctuAddrInSlice.clear();
  }

  void addCtusToSlice(uint32_t startX, uint32_t stopX, uint32_t startY, uint32_t stopY, uint32_t picWidthInCtbsY)
  {
    CHECK(startX >= stopX || startY >= stopY, "Invalid slice definition");
    for (uint32_t ctbY = startY; ctbY < stopY; ctbY++)
    {
      for (uint32_t ctbX = startX; ctbX < stopX; ctbX++)
      {
        m_ctuAddrInSlice.push_back(ctbY * picWidthInCtbsY + ctbX);
        m_numCtuInSlice++;
      }
    }
  }
};

struct RectSlice
{
  uint32_t m_tileIdx { 0 };   // tile index corresponding to the first CTU in the slice
  uint32_t m_sliceWidthInTiles { 0 };   // slice width in units of tiles
  uint32_t m_sliceHeightInTiles { 0 };   // slice height in units of tiles
  uint32_t m_numSlicesInTile {
    0
  };   // number of slices in current tile for the special case of multiple slices inside a single tile
  uint32_t m_sliceHeightInCtu {
    0
  };   // slice height in units of CTUs for the special case of multiple slices inside a single tile
};

struct SubPic
{
  uint32_t m_subPicID { 0 };      // ID of subpicture
  uint32_t m_subPicIdx { 0 };      // Index of subpicture
  uint32_t m_numCTUsInSubPic { 0 };      // number of CTUs contained in this sub-picture
  uint32_t m_subPicCtuTopLeftX { 0 };      // horizontal position of top left CTU of the subpicture in unit of CTU
  uint32_t m_subPicCtuTopLeftY { 0 };      // vertical position of top left CTU of the subpicture in unit of CTU
  uint32_t m_subPicWidthInCTUs { 0 };      // the width of subpicture in units of CTU
  uint32_t m_subPicHeightInCTUs { 0 };      // the height of subpicture in units of CTU
  uint32_t m_subPicWidthInLumaSample { 0 };      // the width of subpicture in units of luma sample
  uint32_t m_subPicHeightInLumaSample { 0 };      // the height of subpicture in units of luma sample
  uint32_t m_firstCtuInSubPic { 0 };      // the raster scan index of the first CTU in a subpicture
  uint32_t m_lastCtuInSubPic { 0 };      // the raster scan index of the last CTU in a subpicture
  uint32_t m_subPicLeft { 0 };      // the position of left boundary
  uint32_t m_subPicRight { 0 };      // the position of right boundary
  uint32_t m_subPicTop { 0 };      // the position of top boundary
  uint32_t m_subPicBottom { 0 };      // the position of bottom boundary
  std::vector<uint32_t> m_ctuAddrInSubPic;   // raster scan addresses of all the CTUs in the slice

  bool m_treatedAsPicFlag {
    false
  };   // whether the subpicture is treated as a picture in the decoding process excluding in-loop filtering operations
  bool m_loopFilterAcrossSubPicEnabledFlag {
    false
  };   // whether in-loop filtering operations may be performed across the boundaries of the subpicture
  uint32_t m_numSlicesInSubPic { 0 };   // Number of slices contained in this subpicture

  void addAllCtusInPicToSubPic(uint32_t startX, uint32_t stopX, uint32_t startY, uint32_t stopY,
                               uint32_t picWidthInCtbsY)
  {
    CHECK(startX >= stopX || startY >= stopY, "Invalid slice definition");
    for (uint32_t ctbY = startY; ctbY < stopY; ctbY++)
    {
      for (uint32_t ctbX = startX; ctbX < stopX; ctbX++)
      {
        m_ctuAddrInSubPic.push_back(ctbY * picWidthInCtbsY + ctbX);
      }
    }
  }

  void addCTUsToSubPic(std::vector<uint32_t> ctuAddrInSlice)
  {
    for (auto ctu: ctuAddrInSlice)
    {
      m_ctuAddrInSubPic.push_back(ctu);
    }
  }

  bool isContainingPos(const Position &pos) const
  {
    return pos.x >= m_subPicLeft && pos.x <= m_subPicRight && pos.y >= m_subPicTop && pos.y <= m_subPicBottom;
  }

  bool containsCtu(const Position &pos) const
  {
    return pos.x >= m_subPicCtuTopLeftX && pos.x < m_subPicCtuTopLeftX + m_subPicWidthInCTUs &&
      pos.y >= m_subPicCtuTopLeftY && pos.y < m_subPicCtuTopLeftY + m_subPicHeightInCTUs;
  }

  bool containsCtu(int ctuAddr) const
  {
    for (auto &addr: m_ctuAddrInSubPic)
    {
      if (addr == ctuAddr)
      {
        return true;
      }
    }
    return false;
  }
};

struct ChromaQpAdj
{
  union
  {
    struct
    {
      int cbOffset;
      int crOffset;
      int jointCbCrOffset;
    } comp;
    int offset[3] = { 0, 0, 0 };
  } u;
};

// PPS class
struct PPS
{
  int  m_ppsId { 0 };   // pic_parameter_set_id
  int  m_spsId { 0 };   // seq_parameter_set_id
  int  m_picInitQPMinus26 { 0 };
  bool m_useDQP { false };
  bool m_usePPSChromaTool { false };
  bool m_sliceChromaQpFlag { false };   // slicelevel_chroma_qp_flag

  int m_layerId { 0 };
  int m_temporalId { 0 };
  int m_puCounter { 0 };

  // access channel
  int  m_chromaCbQpOffset { 0 };
  int  m_chromaCrQpOffset { 0 };
  bool m_chromaJointCbCrQpOffsetPresentFlag { 0 };
  int  m_chromaCbCrQpOffset { 0 };

  // Chroma QP Adjustments
  int m_chromaQpOffsetListLen { 0 };   // size (excludes the null entry used in the following array).

  // Array includes entry [0] for the null offset used when  cu_chroma_qp_offset_flag=0, and entries
  // [cu_chroma_qp_offset_idx+1...] otherwis
  ChromaQpAdj m_chromaQpAdjTableIncludingNullEntry[1 + MAX_QP_OFFSET_LIST_SIZE];

  uint32_t m_numRefIdxDefaultActive[NUM_RPL01] { 1, 1 };

  bool m_rpl1IdxPresentFlag { false };

  bool     m_useWP { false };   // Use of Weighting Prediction (P_SLICE)
  bool     m_useBiWP { false };   // Use of Weighting Bi-Prediction (B_SLICE)
  bool     m_outputFlagPresentFlag { false };   // Indicates the presence of output_flag in slice header
  uint32_t m_numSubPics { 1 };   // number of sub-pictures used - must match SPS
  bool     m_subPicIdMappingInPpsFlag { false };
  uint32_t m_subPicIdLen { 16 };   // sub-picture ID length in bits

  std::vector<uint16_t> m_subPicId;   // sub-picture ID for each sub-picture in the sequence
  bool                  m_noPicPartitionFlag { true };   // no picture partitioning flag - single slice, single tile
  uint8_t               m_log2CtuSize { 0 };   // log2 of the CTU size - required to match corresponding value in SPS
  uint16_t              m_ctuSize { 0 };   // CTU size
  uint32_t              m_picWidthInCtu { 0 };   // picture width in units of CTUs
  uint32_t              m_picHeightInCtu { 0 };   // picture height in units of CTUs
  uint32_t              m_numExpTileCols { 0 };   // number of explicitly specified tile columns
  uint32_t              m_numExpTileRows { 0 };   // number of explicitly specified tile rows
  uint32_t              m_numTileCols { 1 };   // number of tile columns
  uint32_t              m_numTileRows { 1 };   // number of tile rows
  std::vector<uint32_t> m_tileColWidth;   // tile column widths in units of CTUs
  std::vector<uint32_t> m_tileRowHeight;   // tile row heights in units of CTUs
  std::vector<uint32_t> m_tileColBd;   // tile column left-boundaries in units of CTUs
  std::vector<uint32_t> m_tileRowBd;   // tile row top-boundaries in units of CTUs
  std::vector<uint32_t> m_ctuToTileCol;   // mapping between CTU horizontal address and tile column index
  std::vector<uint32_t> m_ctuToTileRow;   // mapping between CTU vertical address and tile row index
  bool                  m_rectSliceFlag { true };   // rectangular slice flag
  bool                  m_singleSlicePerSubPicFlag { false };   // single slice per sub-picture flag
  std::vector<uint32_t> m_ctuToSubPicIdx;   // mapping between CTU and Sub-picture index
  uint32_t              m_numSlicesInPic {
    1
  };   // number of rectangular slices in the picture (raster-scan slice specified at slice level)
  bool                   m_tileIdxDeltaPresentFlag { false };   // tile index delta present flag
  std::vector<RectSlice> m_rectSlices;   // list of rectangular slice signalling parameters
  std::vector<SliceMap>  m_sliceMap;   // list of CTU maps for each slice in the picture
  std::vector<SubPic>    m_subPics;   // list of subpictures in the picture
  bool                   m_loopFilterAcrossTilesEnabledFlag { true };   // loop filtering applied across tiles flag
  bool                   m_loopFilterAcrossSlicesEnabledFlag { false };   // loop filtering applied across slices flag

  bool m_cabacInitPresentFlag { false };

  bool m_pictureHeaderExtensionPresentFlag {
    false
  };   // picture header extension flags present in picture headers or not
  bool m_sliceHeaderExtensionPresentFlag { false };
  bool m_deblockingFilterControlPresentFlag { false };
  bool m_deblockingFilterOverrideEnabledFlag { false };
  bool m_ppsDeblockingFilterDisabledFlag { false };
  int  m_deblockingFilterBetaOffsetDiv2 { 0 };   // beta offset for deblocking filter
  int  m_deblockingFilterTcOffsetDiv2 { 0 };   // tc offset for deblocking filter
  int  m_deblockingFilterCbBetaOffsetDiv2 { 0 };   // beta offset for Cb deblocking filter
  int  m_deblockingFilterCbTcOffsetDiv2 { 0 };   // tc offset for Cb deblocking filter
  int  m_deblockingFilterCrBetaOffsetDiv2 { 0 };   // beta offset for Cr deblocking filter
  int  m_deblockingFilterCrTcOffsetDiv2 { 0 };   // tc offset for Cr deblocking filter

  bool m_rplInfoInPhFlag { false };
  bool m_dbfInfoInPhFlag { false };
  bool m_saoInfoInPhFlag { false };
  bool m_alfInfoInPhFlag { false };
  bool m_wpInfoInPhFlag { false };
  bool m_qpDeltaInfoInPhFlag { false };
  bool m_mixedNaluTypesInPicFlag { false };

  bool     m_conformanceWindowFlag { false };
  uint32_t m_picWidthInLumaSamples { 0 };
  uint32_t m_picHeightInLumaSamples { 0 };
  Window   m_conformanceWindow;
  bool     m_explicitScalingWindowFlag { false };
  Window   m_scalingWindow;

  bool     m_wrapAroundEnabledFlag { false };   //< reference wrap around enabled or not
  unsigned m_picWidthMinusWrapAroundOffset { 0 };   //< pic_width_in_minCbSizeY - wraparound_offset_in_minCbSizeY
  unsigned m_wrapAroundOffset { 0 };   //< reference wrap around offset in luma samples
  bool     m_useSgpmNoBlend { false };
  bool     m_BIF { true };
  int      m_BIFStrength { 1u };
  int      m_BIFQPOffset { 0 };
  bool     m_chromaBIF { true };
  int      m_chromaBIFStrength { 1u };
  int      m_chromaBIFQPOffset { 0 };

  PreCalcValues *pcv { nullptr };

  PPS();
  virtual ~PPS();

  void setQpOffset(CompID compID, int i);
  int  getQpOffset(CompID compID) const
  {
    return (compID == COMP_Y) ? 0
                              : (compID == COMP_Cb     ? m_chromaCbQpOffset
                                   : compID == COMP_Cr ? m_chromaCrQpOffset
                                                       : m_chromaCbCrQpOffset);
  }

  bool               getCuChromaQpOffsetListEnabledFlag() const { return m_chromaQpOffsetListLen > 0; }
  const ChromaQpAdj &getChromaQpOffsetListEntry(int cuChromaQpOffsetIdxPlus1) const;
  void setChromaQpOffsetListEntry(int cuChromaQpOffsetIdxPlus1, int cbOffset, int crOffset, int jointCbCrOffset);
  void setNumSubPics(uint32_t u)
  {
    CHECK(u >= MAX_NUM_SUB_PICS, "Maximum number of subpictures exceeded");
    m_numSubPics = u;
    m_subPicId.resize(m_numSubPics);
  }
  uint32_t getSubPicIdxFromSubPicId(uint32_t subPicId) const;
  void     setLog2CtuSize(uint8_t u)
  {
    m_log2CtuSize    = u;
    m_ctuSize        = 1 << m_log2CtuSize;
    m_picWidthInCtu  = (m_picWidthInLumaSamples + m_ctuSize - 1) / m_ctuSize;
    m_picHeightInCtu = (m_picHeightInLumaSamples + m_ctuSize - 1) / m_ctuSize;
  }
  uint32_t getNumTiles() const { return m_numTileCols * m_numTileRows; }
  void     addTileColumnWidth(uint32_t u)
  {
    CHECK(m_tileColWidth.size() >= MAX_TILE_COLS, "Number of tile columns exceeds valid range");
    m_tileColWidth.push_back(u);
  }
  void     addTileRowHeight(uint32_t u) { m_tileRowHeight.push_back(u); }
  uint32_t getTileColumnWidth(int idx) const
  {
    CHECK(idx >= m_tileColWidth.size(), "Tile column index exceeds valid range");
    return m_tileColWidth[idx];
  }
  uint32_t getTileRowHeight(int idx) const
  {
    CHECK(idx >= m_tileRowHeight.size(), "Tile row index exceeds valid range");
    return m_tileRowHeight[idx];
  }
  uint32_t getTileColumnBd(int idx) const
  {
    CHECK(idx >= m_tileColBd.size(), "Tile column index exceeds valid range");
    return m_tileColBd[idx];
  }
  uint32_t getTileRowBd(int idx) const
  {
    CHECK(idx >= m_tileRowBd.size(), "Tile row index exceeds valid range");
    return m_tileRowBd[idx];
  }
  uint32_t ctuToTileCol(int ctuX) const
  {
    CHECK(ctuX >= m_ctuToTileCol.size(), "CTU address index exceeds valid range");
    return m_ctuToTileCol[ctuX];
  }
  uint32_t ctuToTileRow(int ctuY) const
  {
    CHECK(ctuY >= m_ctuToTileRow.size(), "CTU address index exceeds valid range");
    return m_ctuToTileRow[ctuY];
  }
  uint32_t ctuToTileColBd(int ctuX) const { return getTileColumnBd(ctuToTileCol(ctuX)); }
  uint32_t ctuToTileRowBd(int ctuY) const { return getTileRowBd(ctuToTileRow(ctuY)); }
  bool     ctuIsTileColBd(int ctuX) const { return ctuX == ctuToTileColBd(ctuX); }
  bool     ctuIsTileRowBd(int ctuY) const { return ctuY == ctuToTileRowBd(ctuY); }
  uint32_t getTileIdx(uint32_t ctuX, uint32_t ctuY) const
  {
    return (ctuToTileRow(ctuY) * m_numTileCols) + ctuToTileCol(ctuX);
  }
  uint32_t getTileIdx(uint32_t ctuRsAddr) const
  {
    return getTileIdx(ctuRsAddr % m_picWidthInCtu, ctuRsAddr / m_picWidthInCtu);
  }
  uint32_t getTileIdx(const Position &pos) const { return getTileIdx(pos.x / m_ctuSize, pos.y / m_ctuSize); }
  uint32_t getCtuToSubPicIdx(int idx) const
  {
    CHECK(idx >= m_ctuToSubPicIdx.size(), "CTU address index exceeds valid range");
    CHECK(m_numSubPics < 1, "Number of subpicture cannot be 0");
    return m_ctuToSubPicIdx[idx];
  }

  void          resetTileSliceInfo();
  void          initTiles();
  void          initRectSlices();
  void          initRectSliceMap(const SPS *sps);
  void          initSubPic(const SPS &sps);
  const SubPic &getSubPicFromPos(const Position &pos) const;
  const SubPic &getSubPicFromCU(const CodingUnit &cu) const;
  void          initRasterSliceMap(const std::vector<uint32_t> &sizes);
  void          checkSliceMap();
  SliceMap      getSliceMap(int idx) const
  {
    CHECK(idx >= m_numSlicesInPic, "Slice index exceeds valid range");
    return m_sliceMap[idx];
  }
};
