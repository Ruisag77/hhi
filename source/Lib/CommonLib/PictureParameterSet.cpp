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

#include "CommonDef.h"
#include "Unit.h"
#include "PictureParameterSet.h"
#include "Slice.h"

PPS::PPS()
{
  // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0. This is initialised here
  // and never subsequently changed.
  m_chromaQpAdjTableIncludingNullEntry[0].u.comp.cbOffset        = 0;
  m_chromaQpAdjTableIncludingNullEntry[0].u.comp.crOffset        = 0;
  m_chromaQpAdjTableIncludingNullEntry[0].u.comp.jointCbCrOffset = 0;
}

PPS::~PPS() { delete pcv; }

void PPS::setQpOffset(const CompID compID, const int i)
{
  if (compID == COMP_Cb)
  {
    m_chromaCbQpOffset = i;
  }
  else if (compID == COMP_Cr)
  {
    m_chromaCrQpOffset = i;
  }
  else if (compID == JOINT_CbCr)
  {
    m_chromaCbCrQpOffset = i;
  }
  else
  {
    THROW("Invalid chroma QP offset");
  }
}

const ChromaQpAdj &PPS::getChromaQpOffsetListEntry(const int cuChromaQpOffsetIdxPlus1) const
{
  CHECK(cuChromaQpOffsetIdxPlus1 >= m_chromaQpOffsetListLen + 1, "Invalid chroma QP offset");
  // Array includes entry [0] for the null offset used when
  // cu_chroma_qp_offset_flag=0, and entries [cu_chroma_qp_offset_idx+1...] otherwise
  return m_chromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1];
}

void PPS::setChromaQpOffsetListEntry(const int cuChromaQpOffsetIdxPlus1, const int cbOffset, const int crOffset,
                                     const int jointCbCrOffset)
{
  // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0, and entries
  // [cu_chroma_qp_offset_idx+1...] otherwise
  CHECK(cuChromaQpOffsetIdxPlus1 == 0 || cuChromaQpOffsetIdxPlus1 > MAX_QP_OFFSET_LIST_SIZE,
        "Invalid chroma QP offset");
  m_chromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].u.comp.cbOffset        = cbOffset;
  m_chromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].u.comp.crOffset        = crOffset;
  m_chromaQpAdjTableIncludingNullEntry[cuChromaQpOffsetIdxPlus1].u.comp.jointCbCrOffset = jointCbCrOffset;
  m_chromaQpOffsetListLen = std::max(m_chromaQpOffsetListLen, cuChromaQpOffsetIdxPlus1);
}

// reset tile and slice parameters and lists
void PPS::resetTileSliceInfo()
{
  m_numExpTileCols = 0;
  m_numExpTileRows = 0;
  m_numTileCols    = 0;
  m_numTileRows    = 0;
  m_numSlicesInPic = 0;
  m_tileColWidth.clear();
  m_tileRowHeight.clear();
  m_tileColBd.clear();
  m_tileRowBd.clear();
  m_ctuToTileCol.clear();
  m_ctuToTileRow.clear();
  m_ctuToSubPicIdx.clear();
  m_rectSlices.clear();
  m_sliceMap.clear();
}

// initialize tile row/column sizes and boundaries
void PPS::initTiles()
{
  int colIdx, rowIdx;
  int ctuX, ctuY;

  // check explicit tile column sizes
  uint32_t remainingWidthInCtu = m_picWidthInCtu;

  for (colIdx = 0; colIdx < m_numExpTileCols; colIdx++)
  {
    CHECK(m_tileColWidth[colIdx] > remainingWidthInCtu, "Tile column width exceeds picture width");
    remainingWidthInCtu -= m_tileColWidth[colIdx];
  }

  // divide remaining picture width into uniform tile columns
  uint32_t uniformTileColWidth = m_tileColWidth[colIdx - 1];
  while (remainingWidthInCtu > 0)
  {
    CHECK(colIdx >= MAX_TILE_COLS, "Number of tile columns exceeds valid range");
    uniformTileColWidth = std::min(remainingWidthInCtu, uniformTileColWidth);
    m_tileColWidth.push_back(uniformTileColWidth);
    remainingWidthInCtu -= uniformTileColWidth;
    colIdx++;
  }
  m_numTileCols = colIdx;

  // check explicit tile row sizes
  uint32_t remainingHeightInCtu = m_picHeightInCtu;

  for (rowIdx = 0; rowIdx < m_numExpTileRows; rowIdx++)
  {
    CHECK(m_tileRowHeight[rowIdx] > remainingHeightInCtu, "Tile row height exceeds picture height");
    remainingHeightInCtu -= m_tileRowHeight[rowIdx];
  }

  // divide remaining picture height into uniform tile rows
  uint32_t uniformTileRowHeight = m_tileRowHeight[rowIdx - 1];
  while (remainingHeightInCtu > 0)
  {
    uniformTileRowHeight = std::min(remainingHeightInCtu, uniformTileRowHeight);
    m_tileRowHeight.push_back(uniformTileRowHeight);
    remainingHeightInCtu -= uniformTileRowHeight;
    rowIdx++;
  }
  m_numTileRows = rowIdx;

  // set left column bounaries
  m_tileColBd.push_back(0);
  for (colIdx = 0; colIdx < m_numTileCols; colIdx++)
  {
    m_tileColBd.push_back(m_tileColBd[colIdx] + m_tileColWidth[colIdx]);
  }

  // set top row bounaries
  m_tileRowBd.push_back(0);
  for (rowIdx = 0; rowIdx < m_numTileRows; rowIdx++)
  {
    m_tileRowBd.push_back(m_tileRowBd[rowIdx] + m_tileRowHeight[rowIdx]);
  }

  // set mapping between horizontal CTU address and tile column index
  colIdx = 0;
  for (ctuX = 0; ctuX <= m_picWidthInCtu; ctuX++)
  {
    if (ctuX == m_tileColBd[colIdx + 1])
    {
      colIdx++;
    }
    m_ctuToTileCol.push_back(colIdx);
  }

  // set mapping between vertical CTU address and tile row index
  rowIdx = 0;
  for (ctuY = 0; ctuY <= m_picHeightInCtu; ctuY++)
  {
    if (ctuY == m_tileRowBd[rowIdx + 1])
    {
      rowIdx++;
    }
    m_ctuToTileRow.push_back(rowIdx);
  }
}

// - initialize memory for rectangular slice parameters
void PPS::initRectSlices()
{
  CHECK(m_numSlicesInPic > MAX_SLICES, "Number of slices in picture exceeds valid range");
  m_rectSlices.resize(m_numSlicesInPic);
}

// initialize mapping between rectangular slices and CTUs
void PPS::initRectSliceMap(const SPS *sps)
{
  uint32_t ctuY;
  uint32_t tileX, tileY;

  if (sps)
  {
    m_ctuToSubPicIdx.resize(m_picWidthInCtu * m_picHeightInCtu);
    if (sps->m_numSubPics > 1)
    {
      for (int i = 0; i <= sps->m_numSubPics - 1; i++)
      {
        for (int y = sps->m_subPicCtuTopLeftY[i]; y < sps->m_subPicCtuTopLeftY[i] + sps->m_subPicHeight[i]; y++)
        {
          for (int x = sps->m_subPicCtuTopLeftX[i]; x < sps->m_subPicCtuTopLeftX[i] + sps->m_subPicWidth[i]; x++)
          {
            m_ctuToSubPicIdx[x + y * m_picWidthInCtu] = i;
          }
        }
      }
    }
    else
    {
      for (int i = 0; i < m_picWidthInCtu * m_picHeightInCtu; i++)
      {
        m_ctuToSubPicIdx[i] = 0;
      }
    }
  }

  if (m_singleSlicePerSubPicFlag)
  {
    CHECK(sps == nullptr, "RectSliceMap can only be initialized for slice_per_sub_pic_flag with a valid SPS");
    m_numSlicesInPic = sps->m_numSubPics;

    // allocate new memory for slice list
    CHECK(m_numSlicesInPic > MAX_SLICES, "Number of slices in picture exceeds valid range");
    m_sliceMap.resize(m_numSlicesInPic);

    if (sps->m_numSubPics > 1)
    {
      // Q2001 v15 equation 29
      std::vector<uint32_t> subpicWidthInTiles;
      std::vector<uint32_t> subpicHeightInTiles;
      std::vector<uint32_t> subpicHeightLessThanOneTileFlag;
      subpicWidthInTiles.resize(sps->m_numSubPics);
      subpicHeightInTiles.resize(sps->m_numSubPics);
      subpicHeightLessThanOneTileFlag.resize(sps->m_numSubPics);
      for (uint32_t i = 0; i < sps->m_numSubPics; i++)
      {
        uint32_t leftX        = sps->m_subPicCtuTopLeftX[i];
        uint32_t rightX       = leftX + sps->m_subPicWidth[i] - 1;
        subpicWidthInTiles[i] = m_ctuToTileCol[rightX] + 1 - m_ctuToTileCol[leftX];

        uint32_t topY          = sps->m_subPicCtuTopLeftY[i];
        uint32_t bottomY       = topY + sps->m_subPicHeight[i] - 1;
        subpicHeightInTiles[i] = m_ctuToTileRow[bottomY] + 1 - m_ctuToTileRow[topY];

        if (subpicHeightInTiles[i] == 1 && sps->m_subPicHeight[i] < m_tileRowHeight[m_ctuToTileRow[topY]])
        {
          subpicHeightLessThanOneTileFlag[i] = 1;
        }
        else
        {
          subpicHeightLessThanOneTileFlag[i] = 0;
        }
      }

      for (int i = 0; i < m_numSlicesInPic; i++)
      {
        CHECK(m_numSlicesInPic != sps->m_numSubPics,
              "in single slice per subpic mode, number of slice and subpic shall be equal");
        m_sliceMap[i].initSliceMap();
        if (subpicHeightLessThanOneTileFlag[i])
        {
          m_sliceMap[i].addCtusToSlice(sps->m_subPicCtuTopLeftX[i], sps->m_subPicCtuTopLeftX[i] + sps->m_subPicWidth[i],
                                       sps->m_subPicCtuTopLeftY[i],
                                       sps->m_subPicCtuTopLeftY[i] + sps->m_subPicHeight[i], m_picWidthInCtu);
        }
        else
        {
          tileX = m_ctuToTileCol[sps->m_subPicCtuTopLeftX[i]];
          tileY = m_ctuToTileRow[sps->m_subPicCtuTopLeftY[i]];
          for (uint32_t j = 0; j < subpicHeightInTiles[i]; j++)
          {
            for (uint32_t k = 0; k < subpicWidthInTiles[i]; k++)
            {
              m_sliceMap[i].addCtusToSlice(getTileColumnBd(tileX + k), getTileColumnBd(tileX + k + 1),
                                           getTileRowBd(tileY + j), getTileRowBd(tileY + j + 1), m_picWidthInCtu);
            }
          }
        }
      }
      subpicWidthInTiles.clear();
      subpicHeightInTiles.clear();
      subpicHeightLessThanOneTileFlag.clear();
    }
    else
    {
      m_sliceMap[0].initSliceMap();
      for (int tileY = 0; tileY < m_numTileRows; tileY++)
      {
        for (int tileX = 0; tileX < m_numTileCols; tileX++)
        {
          m_sliceMap[0].addCtusToSlice(getTileColumnBd(tileX), getTileColumnBd(tileX + 1), getTileRowBd(tileY),
                                       getTileRowBd(tileY + 1), m_picWidthInCtu);
        }
      }
      m_sliceMap[0].m_sliceID = 0;
    }
  }
  else
  {
    // allocate new memory for slice list
    CHECK(m_numSlicesInPic > MAX_SLICES, "Number of slices in picture exceeds valid range");
    m_sliceMap.resize(m_numSlicesInPic);
    // generate CTU maps for all rectangular slices in picture
    for (uint32_t i = 0; i < m_numSlicesInPic; i++)
    {
      m_sliceMap[i].initSliceMap();

      // get position of first tile in slice
      tileX = m_rectSlices[i].m_tileIdx % m_numTileCols;
      tileY = m_rectSlices[i].m_tileIdx / m_numTileCols;

      // infer slice size for last slice in picture
      if (i == m_numSlicesInPic - 1)
      {
        m_rectSlices[i].m_sliceWidthInTiles  = (m_numTileCols - tileX);
        m_rectSlices[i].m_sliceHeightInTiles = (m_numTileRows - tileY);
        m_rectSlices[i].m_numSlicesInTile    = 1;
      }

      // set slice index
      m_sliceMap[i].m_sliceID = i;

      // complete tiles within a single slice case
      if (m_rectSlices[i].m_sliceWidthInTiles > 1 || m_rectSlices[i].m_sliceHeightInTiles > 1)
      {
        for (uint32_t j = 0; j < m_rectSlices[i].m_sliceHeightInTiles; j++)
        {
          for (uint32_t k = 0; k < m_rectSlices[i].m_sliceWidthInTiles; k++)
          {
            m_sliceMap[i].addCtusToSlice(getTileColumnBd(tileX + k), getTileColumnBd(tileX + k + 1),
                                         getTileRowBd(tileY + j), getTileRowBd(tileY + j + 1), m_picWidthInCtu);
          }
        }
      }
      // multiple slices within a single tile case
      else
      {
        uint32_t numSlicesInTile = m_rectSlices[i].m_numSlicesInTile;

        ctuY = getTileRowBd(tileY);
        for (uint32_t j = 0; j < numSlicesInTile - 1; j++)
        {
          m_sliceMap[i].addCtusToSlice(getTileColumnBd(tileX), getTileColumnBd(tileX + 1), ctuY,
                                       ctuY + m_rectSlices[i].m_sliceHeightInCtu, m_picWidthInCtu);
          ctuY += m_rectSlices[i].m_sliceHeightInCtu;
          i++;
          m_sliceMap[i].initSliceMap();
          m_sliceMap[i].m_sliceID = i;
        }

        // infer slice height for last slice in tile
        CHECK(ctuY >= getTileRowBd(tileY + 1), "Invalid rectangular slice signalling");
        m_rectSlices[i].m_sliceHeightInCtu = getTileRowBd(tileY + 1) - ctuY;
        m_sliceMap[i].addCtusToSlice(getTileColumnBd(tileX), getTileColumnBd(tileX + 1), ctuY, getTileRowBd(tileY + 1),
                                     m_picWidthInCtu);
      }
    }
  }
  // check for valid rectangular slice map
  checkSliceMap();
}

// initialize mapping between subpicture and CTUs
void PPS::initSubPic(const SPS &sps)
{
  if (m_subPicIdMappingInPpsFlag)
  {
    // When signalled, the number of subpictures has to match in PPS and SPS
    CHECK(m_numSubPics != sps.m_numSubPics, "pps_num_subpics_minus1 shall be equal to sps_num_subpics_minus1");
  }
  else
  {
    // When not signalled  set the numer equal for convenient access
    m_numSubPics = sps.m_numSubPics;
  }

  CHECK(m_numSubPics > MAX_NUM_SUB_PICS, "Number of sub-pictures in picture exceeds valid range");
  m_subPics.resize(m_numSubPics);

  // Check that no subpicture is specified outside of the conformance cropping window
  for (int i = 0; i < sps.m_numSubPics; i++)
  {
    CHECK((sps.m_subPicCtuTopLeftX[i] * sps.m_ctuSize) >=
            (sps.m_maxWidthInLumaSamples -
             sps.m_conformanceWindow.m_winRightOffset * SPS::getWinUnitX(sps.m_chromaFormatIdc)),
          "No subpicture can be located completely outside of the conformance cropping window");
    CHECK(((sps.m_subPicCtuTopLeftX[i] + sps.m_subPicWidth[i]) * sps.m_ctuSize) <=
            (sps.m_conformanceWindow.m_winLeftOffset * SPS::getWinUnitX(sps.m_chromaFormatIdc)),
          "No subpicture can be located completely outside of the conformance cropping window");
    CHECK((sps.m_subPicCtuTopLeftY[i] * sps.m_ctuSize) >=
            (sps.m_maxHeightInLumaSamples -
             sps.m_conformanceWindow.m_winBottomOffset * SPS::getWinUnitY(sps.m_chromaFormatIdc)),
          "No subpicture can be located completely outside of the conformance cropping window");
    CHECK(((sps.m_subPicCtuTopLeftY[i] + sps.m_subPicHeight[i]) * sps.m_ctuSize) <=
            (sps.m_conformanceWindow.m_winTopOffset * SPS::getWinUnitY(sps.m_chromaFormatIdc)),
          "No subpicture can be located completely outside of the conformance cropping window");
  }

  // m_ctuSize,  m_picWidthInCtu, and m_picHeightInCtu might not be initialized yet.
  if (m_ctuSize == 0 || m_picWidthInCtu == 0 || m_picHeightInCtu == 0)
  {
    m_ctuSize        = sps.m_ctuSize;
    m_picWidthInCtu  = (m_picWidthInLumaSamples + m_ctuSize - 1) / m_ctuSize;
    m_picHeightInCtu = (m_picHeightInLumaSamples + m_ctuSize - 1) / m_ctuSize;
  }
  for (int i = 0; i < m_numSubPics; i++)
  {
    m_subPics[i].m_subPicIdx = i;
    if (sps.m_subPicIdMappingExplicitlySignalledFlag)
    {
      if (m_subPicIdMappingInPpsFlag)
      {
        m_subPics[i].m_subPicID = m_subPicId[i];
      }
      else
      {
        m_subPics[i].m_subPicID = sps.m_subPicId[i];
      }
    }
    else
    {
      m_subPics[i].m_subPicID = i;
    }
    m_subPics[i].m_subPicCtuTopLeftX  = sps.m_subPicCtuTopLeftX[i];
    m_subPics[i].m_subPicCtuTopLeftY  = sps.m_subPicCtuTopLeftY[i];
    m_subPics[i].m_subPicWidthInCTUs  = sps.m_subPicWidth[i];
    m_subPics[i].m_subPicHeightInCTUs = sps.m_subPicHeight[i];

    uint32_t firstCTU               = sps.m_subPicCtuTopLeftY[i] * m_picWidthInCtu + sps.m_subPicCtuTopLeftX[i];
    m_subPics[i].m_firstCtuInSubPic = firstCTU;
    uint32_t lastCTU                = (sps.m_subPicCtuTopLeftY[i] + sps.m_subPicHeight[i] - 1) * m_picWidthInCtu +
      sps.m_subPicCtuTopLeftX[i] + sps.m_subPicWidth[i] - 1;
    m_subPics[i].m_lastCtuInSubPic = lastCTU;

    uint32_t left             = sps.m_subPicCtuTopLeftX[i] * m_ctuSize;
    m_subPics[i].m_subPicLeft = left;

    uint32_t right =
      std::min(m_picWidthInLumaSamples - 1, (sps.m_subPicCtuTopLeftX[i] + sps.m_subPicWidth[i]) * m_ctuSize - 1);
    m_subPics[i].m_subPicRight = right;

    m_subPics[i].m_subPicWidthInLumaSample = (right - left + 1);

    uint32_t top             = sps.m_subPicCtuTopLeftY[i] * m_ctuSize;
    m_subPics[i].m_subPicTop = top;

    uint32_t bottom =
      std::min(m_picHeightInLumaSamples - 1, (sps.m_subPicCtuTopLeftY[i] + sps.m_subPicHeight[i]) * m_ctuSize - 1);

    m_subPics[i].m_subPicHeightInLumaSample = (bottom - top + 1);

    m_subPics[i].m_subPicBottom = bottom;

    m_subPics[i].m_ctuAddrInSubPic.clear();

    if (m_numSlicesInPic == 1)
    {
      CHECK(m_numSubPics != 1, "only one slice in picture, but number of subpic is not one");
      m_subPics[i].addAllCtusInPicToSubPic(0, m_picWidthInCtu, 0, m_picHeightInCtu, m_picWidthInCtu);
      m_subPics[i].m_numSlicesInSubPic = 1;
    }
    else
    {
      int numSlicesInSubPic        = 0;
      int idxLastSliceInSubpic     = -1;
      int idxFirstSliceAfterSubpic = m_numSlicesInPic;
      for (int j = 0; j < m_numSlicesInPic; j++)
      {
        uint32_t ctu  = m_sliceMap[j].getCtuAddrInSlice(0);
        uint32_t ctuX = ctu % m_picWidthInCtu;
        uint32_t ctuY = ctu / m_picWidthInCtu;
        if (ctuX >= sps.m_subPicCtuTopLeftX[i] && ctuX < (sps.m_subPicCtuTopLeftX[i] + sps.m_subPicWidth[i]) &&
            ctuY >= sps.m_subPicCtuTopLeftY[i] && ctuY < (sps.m_subPicCtuTopLeftY[i] + sps.m_subPicHeight[i]))
        {
          // add ctus in a slice to the subpicture it belongs to
          m_subPics[i].addCTUsToSubPic(m_sliceMap[j].m_ctuAddrInSlice);
          numSlicesInSubPic++;
          idxLastSliceInSubpic = j;
        }
        else if (idxFirstSliceAfterSubpic == m_numSlicesInPic && idxLastSliceInSubpic != -1)
        {
          idxFirstSliceAfterSubpic = j;
        }
      }
      CHECK(idxFirstSliceAfterSubpic < idxLastSliceInSubpic,
            "The signalling order of slices shall follow the coding order");
      m_subPics[i].m_numSlicesInSubPic = numSlicesInSubPic;
    }
    m_subPics[i].m_treatedAsPicFlag                  = sps.m_subPicTreatedAsPicFlag[i];
    m_subPics[i].m_loopFilterAcrossSubPicEnabledFlag = sps.m_loopFilterAcrossSubpicEnabledFlag[i];
  }
}

const SubPic &PPS::getSubPicFromPos(const Position &pos) const
{
  for (int i = 0; i < m_numSubPics; i++)
  {
    if (m_subPics[i].isContainingPos(pos))
    {
      return m_subPics[i];
    }
  }
  return m_subPics[0];
}

const SubPic &PPS::getSubPicFromCU(const CodingUnit &cu) const
{
  const Position lumaPos = cu.Y().valid()
    ? cu.Y().pos()
    : recalcPosition(cu.chromaFormat, cu.chType, ChannelType::LUMA, cu.block(cu.chType).pos());
  return getSubPicFromPos(lumaPos);
}

uint32_t PPS::getSubPicIdxFromSubPicId(const uint32_t subPicId) const
{
  for (int i = 0; i < m_numSubPics; i++)
  {
    if (m_subPics[i].m_subPicID == subPicId)
    {
      return i;
    }
  }
  return 0;
}

void PPS::initRasterSliceMap(const std::vector<uint32_t> &numTilesInSlice)
{
  uint32_t tileIdx = 0;
  m_numSlicesInPic = (uint32_t)numTilesInSlice.size();

  // allocate new memory for slice list
  CHECK(m_numSlicesInPic > MAX_SLICES, "Number of slices in picture exceeds valid range");
  m_sliceMap.resize(m_numSlicesInPic);

  for (uint32_t sliceIdx = 0; sliceIdx < numTilesInSlice.size(); sliceIdx++)
  {
    m_sliceMap[sliceIdx].initSliceMap();
    m_sliceMap[sliceIdx].m_sliceID         = tileIdx;
    m_sliceMap[sliceIdx].m_numTilesInSlice = numTilesInSlice[sliceIdx];
    for (uint32_t idx = 0; idx < numTilesInSlice[sliceIdx]; idx++)
    {
      uint32_t tileX = tileIdx % m_numTileCols;
      uint32_t tileY = tileIdx / m_numTileCols;
      CHECK(tileY >= m_numTileRows, "Number of tiles in slice exceeds the remaining number of tiles in picture");

      m_sliceMap[sliceIdx].addCtusToSlice(getTileColumnBd(tileX), getTileColumnBd(tileX + 1), getTileRowBd(tileY),
                                          getTileRowBd(tileY + 1), m_picWidthInCtu);
      tileIdx++;
    }
  }

  // check for valid raster-scan slice map
  checkSliceMap();
}

// check if slice map covers the entire picture without skipping or duplicating any CTU positions
void PPS::checkSliceMap()
{
  uint32_t              i;
  std::vector<uint32_t> ctuList, sliceList;
  uint32_t              picSizeInCtu = m_picWidthInCtu * m_picHeightInCtu;
  for (i = 0; i < m_numSlicesInPic; i++)
  {
    sliceList = m_sliceMap[i].m_ctuAddrInSlice;
    ctuList.insert(ctuList.end(), sliceList.begin(), sliceList.end());
  }
  CHECK(ctuList.size() < picSizeInCtu, "Slice map contains too few CTUs");
  CHECK(ctuList.size() > picSizeInCtu, "Slice map contains too many CTUs");
  std::sort(ctuList.begin(), ctuList.end());
  for (i = 1; i < ctuList.size(); i++)
  {
    CHECK(ctuList[i] > ctuList[i - 1] + 1, "CTU missing in slice map");
    CHECK(ctuList[i] == ctuList[i - 1], "CTU duplicated in slice map");
  }
}
