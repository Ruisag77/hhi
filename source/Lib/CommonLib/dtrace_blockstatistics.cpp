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

/** \file     dtrace_blockstatistics.cpp
 *  \brief    DTrace block statistcis support for next software
 */

#include "dtrace_blockstatistics.h"
#include "dtrace.h"
#include "dtrace_next.h"
#include "CommonLib/Unit.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"
// #include "CommonLib/CodingStructure.h"
#include <queue>

#define BLOCK_STATS_POLYGON_MIN_POINTS 3
#define BLOCK_STATS_POLYGON_MAX_POINTS 5

#if K0149_BLOCK_STATISTICS
std::string GetBlockStatisticName(BlockStatistic statistic)
{
  auto statisticIterator = blockstatistic2description.find(statistic);
  // enforces that all delcared statistic enum items are also part of the map
  assert(statisticIterator != blockstatistic2description.end() &&
         "A block statistics declared in the enum is missing in the map for statistic description.");

  return std::get<0>(statisticIterator->second);
}

std::string GetBlockStatisticTypeString(BlockStatistic statistic)
{
  auto statisticIterator = blockstatistic2description.find(statistic);
  // enforces that all delcared statistic enum items are also part of the map
  assert(statisticIterator != blockstatistic2description.end() &&
         "A block statistics declared in the enum is missing in the map for statistic description.");

  BlockStatisticType statisticType = std::get<1>(statisticIterator->second);
  switch (statisticType)
  {
  case BlockStatisticType::Flag:
    return std::string("Flag");
    break;
  case BlockStatisticType::Vector:
    return std::string("Vector");
    break;
  case BlockStatisticType::Integer:
    return std::string("Integer");
    break;
  case BlockStatisticType::AffineTFVectors:
    return std::string("AffineTFVectors");
    break;
  case BlockStatisticType::Line:
    return std::string("Line");
    break;
  case BlockStatisticType::FlagPolygon:
    return std::string("FlagPolygon");
    break;
  case BlockStatisticType::VectorPolygon:
    return std::string("VectorPolygon");
    break;
  case BlockStatisticType::IntegerPolygon:
    return std::string("IntegerPolygon");
    break;
  default:
    assert(0);
    break;
  }
  return std::string();
}

std::string GetBlockStatisticTypeSpecificInfo(BlockStatistic statistic)
{
  auto statisticIterator = blockstatistic2description.find(statistic);
  // enforces that all delcared statistic enum items are also part of the map
  assert(statisticIterator != blockstatistic2description.end() &&
         "A block statistics declared in the enum is missing in the map for statistic description.");

  return std::get<2>(statisticIterator->second);
}

void CDTrace::dtrace_block_scalar(int k, const CodingStructure &cs, std::string stat_type, signed value)
{
#if BLOCK_STATS_AS_CSV
  dtrace<false>(k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%d\n", cs.picture->poc, cs.area.lx(), cs.area.ly(),
                cs.area.lwidth(), cs.area.lheight(), stat_type.c_str(), value);
#else
  dtrace<false>(k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s=%d\n", cs.picture->m_poc, cs.area.lx(), cs.area.ly(),
                cs.area.lwidth(), cs.area.lheight(), stat_type.c_str(), value);
#endif
}

void CDTrace::dtrace_block_scalar(int k, const CodingUnit &cu, std::string stat_type, signed value,
                                  bool isChroma /*= false*/)
{
  const CodingStructure &cs = *cu.cs;
#if BLOCK_STATS_AS_CSV
  if (isChroma)
  {
    dtrace<false>(k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%d\n", cs.picture->poc, cu.Cb().x * 2, cu.Cb().y * 2,
                  cu.Cb().width * 2, cu.Cb().height * 2, stat_type.c_str(), value);
  }
  else
  {
    dtrace<false>(k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%d\n", cs.picture->poc, cu.lx(), cu.ly(), cu.lwidth(),
                  cu.lheight(), stat_type.c_str(), value);
  }
#else
  if (isChroma)
  {
    dtrace<false>(k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s=%d\n", cs.picture->m_poc, cu.Cb().x * 2, cu.Cb().y * 2,
                  cu.Cb().width * 2, cu.Cb().height * 2, stat_type.c_str(), value);
  }
  else
  {
    dtrace<false>(k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s=%d\n", cs.picture->m_poc, cu.lx(), cu.ly(), cu.lwidth(),
                  cu.lheight(), stat_type.c_str(), value);
  }
#endif
}

void CDTrace::dtrace_block_vector(int k, const CodingUnit &cu, std::string stat_type, signed val_x, signed val_y,
                                  bool isChroma /*= false*/)
{
  const CodingStructure &cs = *cu.cs;
#if BLOCK_STATS_AS_CSV
  if (isChroma)
  {
    dtrace<false>(k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d\n", cs.picture->poc, cu.Cb().x * 2, cu.Cb().y * 2,
                  cu.Cb().width * 2, cu.Cb().height * 2, stat_type.c_str(), val_x * 2, val_y * 2);
  }
  else
  {
    dtrace<false>(k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d\n", cs.picture->poc, cu.lx(), cu.ly(), cu.lwidth(),
                  cu.lheight(), stat_type.c_str(), val_x, val_y);
  }
#else
  if (isChroma)
  {
    dtrace<false>(k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n", cs.picture->m_poc, cu.Cb().x * 2,
                  cu.Cb().y * 2, cu.Cb().width * 2, cu.Cb().height * 2, stat_type.c_str(), val_x * 2, val_y * 2);
  }
  else
  {
    dtrace<false>(k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n", cs.picture->m_poc, cu.lx(), cu.ly(),
                  cu.lwidth(), cu.lheight(), stat_type.c_str(), val_x, val_y);
  }
#endif
}

void CDTrace::dtrace_block_scalar(int k, const TransformUnit &tu, std::string stat_type, signed value,
                                  bool isChroma /*= false*/)
{
  const CodingStructure &cs = *tu.cs;
#if BLOCK_STATS_AS_CSV
  if (isChroma)
  {
    dtrace<false>(k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%d\n", cs.picture->poc, tu.Cb().x * 2, tu.Cb().y * 2,
                  tu.Cb().width * 2, tu.Cb().height * 2, stat_type.c_str(), value);
  }
  else
  {
    dtrace<false>(k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%d\n", cs.picture->poc, tu.lx(), tu.ly(), tu.lwidth(),
                  tu.lheight(), stat_type.c_str(), value);
  }
#else
  if (isChroma)
  {
    dtrace<false>(k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s=%d\n", cs.picture->m_poc, tu.Cb().x * 2, tu.Cb().y * 2,
                  tu.Cb().width * 2, tu.Cb().height * 2, stat_type.c_str(), value);
  }
  else
  {
    dtrace<false>(k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s=%d\n", cs.picture->m_poc, tu.lx(), tu.ly(), tu.lwidth(),
                  tu.lheight(), stat_type.c_str(), value);
  }
#endif
}

void CDTrace::dtrace_block_vector(int k, const TransformUnit &tu, std::string stat_type, signed val_x, signed val_y)
{
  const CodingStructure &cs = *tu.cs;
#if BLOCK_STATS_AS_CSV
  dtrace<false>(k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d\n", cs.picture->poc, tu.lx(), tu.ly(), tu.lwidth(),
                tu.lheight(), stat_type.c_str(), val_x, val_y);
#else
  dtrace<false>(k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n", cs.picture->m_poc, tu.lx(), tu.ly(),
                tu.lwidth(), tu.lheight(), stat_type.c_str(), val_x, val_y);
#endif
}

void CDTrace::dtrace_block_affinetf(int k, const CodingUnit &cu, std::string stat_type, signed val_x0, signed val_y0,
                                    signed val_x1, signed val_y1, signed val_x2, signed val_y2)
{
  const CodingStructure &cs = *cu.cs;
#if BLOCK_STATS_AS_CSV
  dtrace<false>(k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d;%4d;%4d;%4d;%4d\n", cs.picture->poc, cu.lx(), cu.ly(),
                cu.lwidth(), cu.lheight(), stat_type.c_str(), val_x0, val_y0, val_x1, val_y1, val_x2, val_y2);
#else
  dtrace<false>(k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d,%4d,%4d,%4d,%4d}\n", cs.picture->m_poc, cu.lx(),
                cu.ly(), cu.lwidth(), cu.lheight(), stat_type.c_str(), val_x0, val_y0, val_x1, val_y1, val_x2, val_y2);
#endif
}

void CDTrace::dtrace_block_line(int k, const CodingUnit &cu, std::string stat_type, int x0, int y0, int x1, int y1)
{
#if BLOCK_STATS_AS_CSV
  dtrace<false>(k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d;%4d;%4d;\n", cu.slice->m_poc, cu.lx(), cu.ly(), cu.lwidth(),
                cu.lheight(), stat_type.c_str(), x0, y0, x1, y1);
#else
  dtrace<false>(k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d,%4d,%4d}\n", cu.slice->m_poc, cu.lx(), cu.ly(),
                cu.lwidth(), cu.lheight(), stat_type.c_str(), x0, y0, x1, y1);
#endif
}

void CDTrace::dtrace_polygon_scalar(int k, int poc, const std::vector<Position> &polygon, std::string stat_type,
                                    signed value)
{
  assert(polygon.size() >= BLOCK_STATS_POLYGON_MIN_POINTS && "Not enough points to from polygon!");
  assert(polygon.size() <= BLOCK_STATS_POLYGON_MAX_POINTS && "Too many points. Unsupported polygon!");
  std::string polygonDescription;
#if BLOCK_STATS_AS_CSV
  for (auto position: polygon)
  {
    polygonDescription += std::to_string(position.x) + ";" + std::to_string(position.y) + ";";
  }

  dtrace<false>(k, "BlockStat;%d;%s%s;%d\n", poc, polygonDescription.c_str(), stat_type.c_str(), value);
#else
  for (auto position: polygon)
  {
    polygonDescription += "(" + std::to_string(position.x) + ", " + std::to_string(position.y) + ")--";
  }

  dtrace<false>(k, "BlockStat: POC %d @[%s] %s=%d\n", poc, polygonDescription.c_str(), stat_type.c_str(), value);
#endif
}

void CDTrace::dtrace_polygon_vector(int k, int poc, const std::vector<Position> &polygon, std::string stat_type,
                                    signed val_x, signed val_y)
{
  assert(polygon.size() >= BLOCK_STATS_POLYGON_MIN_POINTS && "Not enough points to from polygon!");
  assert(polygon.size() <= BLOCK_STATS_POLYGON_MAX_POINTS && "Too many points. Unsupported polygon!");
  std::string polygonDescription;
#if BLOCK_STATS_AS_CSV
  for (auto position: polygon)
  {
    polygonDescription += std::to_string(position.x) + ";" + std::to_string(position.y) + ";";
  }

  dtrace<false>(k, "BlockStat;%d;%s%s;%d;%d\n", poc, polygonDescription.c_str(), stat_type.c_str(), val_x, val_y);
#else
  for (auto position: polygon)
  {
    polygonDescription += "(" + std::to_string(position.x) + ", " + std::to_string(position.y) + ")--";
  }

  dtrace<false>(k, "BlockStat: POC %d @[%s] %s={%4d,%4d}\n", poc, polygonDescription.c_str(), stat_type.c_str(), val_x,
                val_y);
#endif
}

void retrieveGeoPolygons(const CodingUnit &cu, std::vector<Position> (&geoPartitions)[2], Position (&linePositions)[2])
{
  // adapted code from interpolation filter to find geo partition polygons like this:
  // use SAD mask, which should clearly partition the two polygons.
  // loop over boundary pixels and find positions where there is a change, these should be the polygon corners
  static bool                  isInitialized = false;
  static std::vector<Position> allGeoPartitionings[GEO_NUM_CU_SIZE][GEO_NUM_CU_SIZE][GEO_NUM_PARTITION_MODE][2];
  static Position              allGeoPartitioningLines[GEO_NUM_CU_SIZE][GEO_NUM_CU_SIZE][GEO_NUM_PARTITION_MODE][2];

  if (!isInitialized)
  {
    for (int hIdx = 0; hIdx < GEO_NUM_CU_SIZE; hIdx++)
    {
      int16_t height = 1 << (hIdx + GEO_MIN_CU_LOG2);
      for (int wIdx = 0; wIdx < GEO_NUM_CU_SIZE; wIdx++)
      {
        int16_t width = 1 << (wIdx + GEO_MIN_CU_LOG2);
        for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
        {
          const int angle = g_geoParams[splitDir].angleIdx;

          int  maskStride = 0;
          int  stepX      = 1;
          Pel *sadMask;

          const int16_t *wOffset = g_weightOffset[splitDir][hIdx][wIdx];

          if (g_angle2mirror[angle] == 2)
          {
            maskStride = -GEO_WEIGHT_MASK_SIZE;

            sadMask =
              &g_globalGeoEncSADmask[g_angle2mask[angle]]
                                    [(GEO_WEIGHT_MASK_SIZE - 1 - wOffset[1]) * GEO_WEIGHT_MASK_SIZE + wOffset[0]];
          }
          else if (g_angle2mirror[angle] == 1)
          {
            stepX      = -1;
            maskStride = GEO_WEIGHT_MASK_SIZE;

            sadMask = &g_globalGeoEncSADmask[g_angle2mask[angle]][wOffset[1] * GEO_WEIGHT_MASK_SIZE +
                                                                  (GEO_WEIGHT_MASK_SIZE - 1 - wOffset[0])];
          }
          else
          {
            maskStride = GEO_WEIGHT_MASK_SIZE;

            sadMask = &g_globalGeoEncSADmask[g_angle2mask[angle]][wOffset[1] * GEO_WEIGHT_MASK_SIZE + wOffset[0]];
          }

          int              currentPartition = 0;
          std::vector<Pel> boundaryOfMask; // for debugging

          Area     partitionArea = Area(0, 0, width, height);
          Position TL            = partitionArea.topLeft();
          Position TR            = partitionArea.topRight();
          TR                     = TR.offset(1, 0);
          Position BL            = partitionArea.bottomLeft();
          BL                     = BL.offset(0, 1);
          Position BR            = partitionArea.bottomRight();
          BR                     = BR.offset(1, 1);

          std::vector<Position> oneGeoPartitioning[2];
          Position              oneGeoPartitioningLine[2];
          // corner of block is a corner of the first partition
          oneGeoPartitioning[currentPartition].push_back(TL);

          // process top boundary
          for (int x = 0; x < width - 1; x++)
          {
            boundaryOfMask.push_back(*sadMask);
            if (*sadMask != *(sadMask + stepX))
            {
              // found a change of partitions, it is a corner of both partition polygons
              oneGeoPartitioning[currentPartition].push_back(Position(TL.x + x, TL.y));
              oneGeoPartitioningLine[currentPartition] = Position(TL.x + x, TL.y);
              currentPartition ^= 0x01;
              oneGeoPartitioning[currentPartition].push_back(Position(TL.x + x, TL.y));
            }
            sadMask += stepX;
          }

          // corner of block is a corner of the current partition
          oneGeoPartitioning[currentPartition].push_back(TR);

          // process right boundary
          for (int y = 0; y < height - 1; y++)
          {
            boundaryOfMask.push_back(*sadMask);
            if (*sadMask != *(sadMask + maskStride))
            {
              // found a change of partitions, it is a corner of both partition polygons
              oneGeoPartitioning[currentPartition].push_back(Position(TR.x, TR.y + y));
              oneGeoPartitioningLine[currentPartition] = Position(TR.x, TR.y + y);
              currentPartition ^= 0x01;
              oneGeoPartitioning[currentPartition].push_back(Position(TR.x, TR.y + y));
            }
            sadMask += maskStride;
          }

          // corner of block is a corner of the current partition
          oneGeoPartitioning[currentPartition].push_back(BR);

          // process bottom boundary
          for (int x = width - 1; x > 0; x--)
          {
            boundaryOfMask.push_back(*sadMask);
            if (*sadMask != *(sadMask - stepX))
            {
              // found a change of partitions, it is a corner of both partition polygons
              oneGeoPartitioning[currentPartition].push_back(Position(BL.x + x, BL.y));
              oneGeoPartitioningLine[currentPartition] = Position(BL.x + x, BL.y);
              currentPartition ^= 0x01;
              oneGeoPartitioning[currentPartition].push_back(Position(BL.x + x, BL.y));
            }
            sadMask -= stepX;
          }

          // corner of block is a corner of the current partition
          oneGeoPartitioning[currentPartition].push_back(BL);

          // process left boundary
          for (int y = height - 1; y > 0; y--)
          {
            boundaryOfMask.push_back(*sadMask);
            if (*sadMask != *(sadMask - maskStride))
            {
              // found a change of partitions, it is a corner of both partition polygons
              oneGeoPartitioning[currentPartition].push_back(Position(TL.x, TL.y + y));
              oneGeoPartitioningLine[currentPartition] = Position(TL.x, TL.y + y);
              currentPartition ^= 0x01;
              oneGeoPartitioning[currentPartition].push_back(Position(TL.x, TL.y + y));
            }
            sadMask -= maskStride;
          }

          // corner of block is a corner of the current partition
          oneGeoPartitioning[currentPartition].push_back(TL);

          // remove duplicate points
          for (auto geoPartIdx = 0; geoPartIdx < 2; geoPartIdx++)
          {
            // this will only remove consecutive duplicates
            auto last = std::unique(oneGeoPartitioning[geoPartIdx].begin(), oneGeoPartitioning[geoPartIdx].end());
            oneGeoPartitioning[geoPartIdx].erase(last, oneGeoPartitioning[geoPartIdx].end());
            // also check if first and last are the same
            if (oneGeoPartitioning[geoPartIdx].front() == oneGeoPartitioning[geoPartIdx].back())
            {
              oneGeoPartitioning[geoPartIdx].pop_back();
            }

            CHECK(!(oneGeoPartitioning[geoPartIdx].size() > 2 && oneGeoPartitioning[geoPartIdx].size() < 6),
                  "Invalid geo partition shape. Polygon should have between 3 and 5 corners.");
          }

          allGeoPartitionings[hIdx][wIdx][splitDir][0]     = oneGeoPartitioning[0];
          allGeoPartitionings[hIdx][wIdx][splitDir][1]     = oneGeoPartitioning[1];
          allGeoPartitioningLines[hIdx][wIdx][splitDir][0] = oneGeoPartitioningLine[0];
          allGeoPartitioningLines[hIdx][wIdx][splitDir][1] = oneGeoPartitioningLine[1];
        }
      }
    }
    isInitialized = true;
  }

  const uint8_t splitDir = cu.geoSplitDir;
  int16_t       wIdx     = floorLog2(cu.lwidth()) - GEO_MIN_CU_LOG2;
  int16_t       hIdx     = floorLog2(cu.lheight()) - GEO_MIN_CU_LOG2;

  Position TL = cu.Y().topLeft();

  geoPartitions[0] = allGeoPartitionings[hIdx][wIdx][splitDir][0];
  geoPartitions[1] = allGeoPartitionings[hIdx][wIdx][splitDir][1];
  linePositions[0] = allGeoPartitioningLines[hIdx][wIdx][splitDir][0];
  linePositions[1] = allGeoPartitioningLines[hIdx][wIdx][splitDir][1];

  // offset the partitioning to the current cu
  for (auto geoPartIdx = 0; geoPartIdx < 2; geoPartIdx++)
  {
    for (Position &polygonCorner: geoPartitions[geoPartIdx])
    {
      polygonCorner.repositionTo(polygonCorner.offset(TL));
    }
  }
}

std::queue<MergeCtx> geoMergeCtxtsOfCurrentCtu;
void                 storeGeoMergeCtx(MergeCtx geoMergeCtx) { geoMergeCtxtsOfCurrentCtu.push(geoMergeCtx); }

void writeBlockStatisticsHeader(const SPS *sps)
{
  static bool has_header_been_written = false;
  if (has_header_been_written)
  {
    return;
  }

  // only write header when block statistics are used
  bool write_blockstatistics =
    g_trace_ctx->isChannelActive(D_BLOCK_STATISTICS_ALL) || g_trace_ctx->isChannelActive(D_BLOCK_STATISTICS_CODED);
  if (!write_blockstatistics)
  {
    return;
  }

  DTRACE_HEADER(g_trace_ctx, "# VTMBMS Block Statistics\n");
  // sequence info
  DTRACE_HEADER(g_trace_ctx, "# Sequence size: [%dx %d]\n", sps->m_maxWidthInLumaSamples,
                sps->m_maxHeightInLumaSamples);
  // list statistics
  for (auto i = static_cast<int>(BlockStatistic::PredMode); i < static_cast<int>(BlockStatistic::NumBlockStatistics);
       i++)
  {
    BlockStatistic statistic                   = BlockStatistic(i);
    std::string    statitic_name               = GetBlockStatisticName(statistic);
    std::string    statitic_type               = GetBlockStatisticTypeString(statistic);
    std::string    statitic_type_specific_info = GetBlockStatisticTypeSpecificInfo(statistic);
    DTRACE_HEADER(g_trace_ctx, "# Block Statistic Type: %s; %s; %s\n", statitic_name.c_str(), statitic_type.c_str(),
                  statitic_type_specific_info.c_str());
  }

  has_header_been_written = true;
}

void getAndStoreBlockStatistics(const CodingStructure &cs, const UnitArea &ctuArea)
{
  // two differemt behaviors, depending on which information is needed
  bool writeAll   = g_trace_ctx->isChannelActive(D_BLOCK_STATISTICS_ALL);
  bool writeCoded = g_trace_ctx->isChannelActive(D_BLOCK_STATISTICS_CODED);

  CHECK(writeAll && writeCoded, "Either used D_BLOCK_STATISTICS_ALL or D_BLOCK_STATISTICS_CODED. Not both at once!")

  if (writeCoded)
  {
    writeAllCodedData(
      cs, ctuArea);    // this will write out important cu-based data, only if it is actually decoded and used
  }
  else if (writeAll)
  {
    writeAllData(cs, ctuArea);         // this will write out all inter- or intra-prediction related data
  }
}

void writeAllData(const CodingStructure &cs, const UnitArea &ctuArea)
{
  const int maxNumChannelType = isChromaEnabled(cs.pcv->chrFormat) && CS::isDualITree(cs) ? 2 : 1;
  const int nShift            = MV_FRACTIONAL_BITS_DIFF;
  const int nOffset           = 1 << (nShift - 1);
  for (int ch = 0; ch < maxNumChannelType; ch++)
  {
    const ChannelType chType = ChannelType(ch);

    for (const CodingUnit &cu: cs.traverseCUs(CS::getArea(cs, ctuArea, chType), chType))
    {
      if (isLuma(chType))
      {
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::PredMode),
                            cu.predMode);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::Depth),
                            cu.depth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::QT_Depth),
                            cu.qtDepth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::BT_Depth),
                            cu.btDepth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::MT_Depth),
                            cu.mtDepth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::ChromaQPAdj),
                            cu.chromaQpAdj);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::QP), cu.qp);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::SplitSeries),
                            (int)cu.splitSeries);

        // skip flag
        if (!cs.slice->isIntra())
        {
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::SkipFlag),
                              cu.skip);
        }

        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::BDPCM),
                            to_underlying(cu.getBdpcmMode(CompID::COMP_Y)));
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::BDPCMChroma),
                            to_underlying(cu.getBdpcmMode(CompID::COMP_Cb)));
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::TileIdx),
                            cu.tileIdx);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                            GetBlockStatisticName(BlockStatistic::IndependentSliceIdx),
                            cu.slice->m_independentSliceIdx);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                            GetBlockStatisticName(BlockStatistic::MMVDSkipFlag), cu.mmvdSkip);
      }
      else if (chType == ChannelType::CHROMA)
      {
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                   GetBlockStatisticName(BlockStatistic::Depth_Chroma), cu.depth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                   GetBlockStatisticName(BlockStatistic::QT_Depth_Chroma), cu.qtDepth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                   GetBlockStatisticName(BlockStatistic::BT_Depth_Chroma), cu.btDepth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                   GetBlockStatisticName(BlockStatistic::MT_Depth_Chroma), cu.mtDepth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                   GetBlockStatisticName(BlockStatistic::ChromaQPAdj_Chroma), cu.chromaQpAdj);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                   GetBlockStatisticName(BlockStatistic::QP_Chroma), cu.qp);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                   GetBlockStatisticName(BlockStatistic::SplitSeries_Chroma), (int)cu.splitSeries);

        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::BDPCMChroma),
                            to_underlying(cu.getBdpcmMode(CompID::COMP_Cb)));
      }

      switch (cu.predMode)
      {
      case MODE_INTER:
        {
          for (const CodingUnit &cu: cs.traverseCUs(CS::getArea(cs, ctuArea, chType), chType))
          {
            if (!cu.skip)
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                  GetBlockStatisticName(BlockStatistic::MergeFlag), cu.mergeFlag);
            }
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                GetBlockStatisticName(BlockStatistic::RegularMergeFlag), cu.regularMergeFlag);
            if (cu.mergeFlag)
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                  GetBlockStatisticName(BlockStatistic::MergeIdx), cu.mergeIdx);
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                  GetBlockStatisticName(BlockStatistic::MergeType), to_underlying(cu.mergeType));
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                  GetBlockStatisticName(BlockStatistic::MMVDMergeFlag), cu.mmvdMergeFlag);
              if (cu.mmvdSkip || cu.mmvdMergeFlag)
              {
                DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                    GetBlockStatisticName(BlockStatistic::MMVDMergeIdx), cu.mmvdMergeIdx.val);
              }
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                  GetBlockStatisticName(BlockStatistic::CiipFlag), cu.ciipFlag);
              if (cu.ciipFlag)
              {
                DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                    GetBlockStatisticName(BlockStatistic::Luma_IntraMode),
                                    cu.intraDir[ChannelType::LUMA]);
              }
            }
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                GetBlockStatisticName(BlockStatistic::AffineFlag), cu.affine);
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                GetBlockStatisticName(BlockStatistic::AffineType), to_underlying(cu.affineType));
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                GetBlockStatisticName(BlockStatistic::InterDir), cu.interDir);

            if (cu.interDir != 2 /* PRED_L1 */)
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                  GetBlockStatisticName(BlockStatistic::MVPIdxL0), cu.mvpIdx[RPL0]);
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                  GetBlockStatisticName(BlockStatistic::RefIdxL0), cu.refIdx[RPL0]);
            }
            if (cu.interDir != 1 /* PRED_L1 */)
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                  GetBlockStatisticName(BlockStatistic::MVPIdxL1), cu.mvpIdx[RPL1]);
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                  GetBlockStatisticName(BlockStatistic::RefIdxL1), cu.refIdx[RPL1]);
            }
            if (!cu.affine && !cu.geoFlag)
            {
              if (cu.interDir != 2 /* PRED_L1 */)
              {
                Mv mv   = cu.mv[RPL0];
                Mv mvd  = cu.mvd[RPL0];
                mv.hor  = mv.hor >= 0 ? (mv.hor + nOffset) >> nShift : -((-mv.hor + nOffset) >> nShift);
                mv.ver  = mv.ver >= 0 ? (mv.ver + nOffset) >> nShift : -((-mv.ver + nOffset) >> nShift);
                mvd.hor = mvd.hor >= 0 ? (mvd.hor + nOffset) >> nShift : -((-mvd.hor + nOffset) >> nShift);
                mvd.ver = mvd.ver >= 0 ? (mvd.ver + nOffset) >> nShift : -((-mvd.ver + nOffset) >> nShift);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                    GetBlockStatisticName(BlockStatistic::MVDL0), mvd.hor, mvd.ver);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                    GetBlockStatisticName(BlockStatistic::MVL0), mv.hor, mv.ver);
              }
              if (cu.interDir != 1 /* PRED_L1 */)
              {
                Mv mv   = cu.mv[RPL1];
                Mv mvd  = cu.mvd[RPL1];
                mv.hor  = mv.hor >= 0 ? (mv.hor + nOffset) >> nShift : -((-mv.hor + nOffset) >> nShift);
                mv.ver  = mv.ver >= 0 ? (mv.ver + nOffset) >> nShift : -((-mv.ver + nOffset) >> nShift);
                mvd.hor = mvd.hor >= 0 ? (mvd.hor + nOffset) >> nShift : -((-mvd.hor + nOffset) >> nShift);
                mvd.ver = mvd.ver >= 0 ? (mvd.ver + nOffset) >> nShift : -((-mvd.ver + nOffset) >> nShift);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                    GetBlockStatisticName(BlockStatistic::MVDL1), mvd.hor, mvd.ver);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                    GetBlockStatisticName(BlockStatistic::MVL1), mv.hor, mv.ver);
              }
            }
            else if (cu.affine)
            {
              if (cu.interDir != 2 /* PRED_L1 */)
              {
                Mv                mv[3];
                const CMotionBuf &mb = cu.getMotionBuf();
                mv[0]                = mb.at(0, 0).mv[RPL0];
                mv[1]                = mb.at(mb.width - 1, 0).mv[RPL0];
                mv[2]                = mb.at(0, mb.height - 1).mv[RPL0];
                // motion vectors should use low precision or they will appear to large
                mv[0].hor = mv[0].hor >= 0 ? (mv[0].hor + nOffset) >> nShift : -((-mv[0].hor + nOffset) >> nShift);
                mv[0].ver = mv[0].ver >= 0 ? (mv[0].ver + nOffset) >> nShift : -((-mv[0].ver + nOffset) >> nShift);
                mv[1].hor = mv[1].hor >= 0 ? (mv[1].hor + nOffset) >> nShift : -((-mv[1].hor + nOffset) >> nShift);
                mv[1].ver = mv[1].ver >= 0 ? (mv[1].ver + nOffset) >> nShift : -((-mv[1].ver + nOffset) >> nShift);
                mv[2].hor = mv[2].hor >= 0 ? (mv[2].hor + nOffset) >> nShift : -((-mv[2].hor + nOffset) >> nShift);
                mv[2].ver = mv[2].ver >= 0 ? (mv[2].ver + nOffset) >> nShift : -((-mv[2].ver + nOffset) >> nShift);
                DTRACE_BLOCK_AFFINETF(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                      GetBlockStatisticName(BlockStatistic::AffineMVL0), mv[0].hor, mv[0].ver,
                                      mv[1].hor, mv[1].ver, mv[2].hor, mv[2].ver);
              }
              if (cu.interDir != 1 /* PRED_L1 */)
              {
                Mv                mv[3];
                const CMotionBuf &mb = cu.getMotionBuf();
                mv[0]                = mb.at(0, 0).mv[RPL1];
                mv[1]                = mb.at(mb.width - 1, 0).mv[RPL1];
                mv[2]                = mb.at(0, mb.height - 1).mv[RPL1];
                // motion vectors should use low precision or they will appear to large
                mv[0].hor = mv[0].hor >= 0 ? (mv[0].hor + nOffset) >> nShift : -((-mv[0].hor + nOffset) >> nShift);
                mv[0].ver = mv[0].ver >= 0 ? (mv[0].ver + nOffset) >> nShift : -((-mv[0].ver + nOffset) >> nShift);
                mv[1].hor = mv[1].hor >= 0 ? (mv[1].hor + nOffset) >> nShift : -((-mv[1].hor + nOffset) >> nShift);
                mv[1].ver = mv[1].ver >= 0 ? (mv[1].ver + nOffset) >> nShift : -((-mv[1].ver + nOffset) >> nShift);
                mv[2].hor = mv[2].hor >= 0 ? (mv[2].hor + nOffset) >> nShift : -((-mv[2].hor + nOffset) >> nShift);
                mv[2].ver = mv[2].ver >= 0 ? (mv[2].ver + nOffset) >> nShift : -((-mv[2].ver + nOffset) >> nShift);
                DTRACE_BLOCK_AFFINETF(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                      GetBlockStatisticName(BlockStatistic::AffineMVL1), mv[0].hor, mv[0].ver,
                                      mv[1].hor, mv[1].ver, mv[2].hor, mv[2].ver);
              }
            }

            // tracing Motion buffers
            CMotionBuf mb = cu.getMotionBuf();
            // todo: assuming granulatiry == 4. can it be derived?
            for (int y = 0; y < mb.height; y++)
            {
              for (int x = 0; x < mb.width; x++)
              {
                const MotionInfo &pixMi = mb.at(x, y);

                if (pixMi.interDir == 1)
                {
                  const Mv mv = pixMi.mv[RPL0];
#if BLOCK_STATS_AS_CSV
                  g_trace_ctx->dtrace<false>(D_BLOCK_STATISTICS_ALL, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d\n",
                                             cs.picture->poc, cu.lx() + 4 * x, cu.ly() + 4 * y, 4, 4,
                                             GetBlockStatisticName(BlockStatistic::MotionBufL0).c_str(), mv.hor,
                                             mv.ver);
#else
                  g_trace_ctx->dtrace<false>(
                    D_BLOCK_STATISTICS_ALL, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n", cs.picture->m_poc,
                    cu.lx() + 4 * x, cu.ly() + 4 * y, 4, 4, GetBlockStatisticName(BlockStatistic::MotionBufL0).c_str(),
                    mv.hor, mv.ver);
#endif
                }
                else if (pixMi.interDir == 2)
                {
                  const Mv mv = pixMi.mv[RPL1];
#if BLOCK_STATS_AS_CSV
                  g_trace_ctx->dtrace<false>(D_BLOCK_STATISTICS_ALL, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d\n",
                                             cs.picture->poc, cu.lx() + 4 * x, cu.ly() + 4 * y, 4, 4,
                                             GetBlockStatisticName(BlockStatistic::MotionBufL1).c_str(), mv.hor,
                                             mv.ver);
#else
                  g_trace_ctx->dtrace<false>(
                    D_BLOCK_STATISTICS_ALL, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n", cs.picture->m_poc,
                    cu.lx() + 4 * x, cu.ly() + 4 * y, 4, 4, GetBlockStatisticName(BlockStatistic::MotionBufL1).c_str(),
                    mv.hor, mv.ver);
#endif
                }
                else if (pixMi.interDir == 3)
                {
                  {
                    const Mv mv = pixMi.mv[RPL0];
#if BLOCK_STATS_AS_CSV
                    g_trace_ctx->dtrace<false>(D_BLOCK_STATISTICS_ALL, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d\n",
                                               cs.picture->poc, cu.lx() + 4 * x, cu.ly() + 4 * y, 4, 4,
                                               GetBlockStatisticName(BlockStatistic::MotionBufL0).c_str(), mv.hor,
                                               mv.ver);
#else
                    g_trace_ctx->dtrace<false>(
                      D_BLOCK_STATISTICS_ALL, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n",
                      cs.picture->m_poc, cu.lx() + 4 * x, cu.ly() + 4 * y, 4, 4,
                      GetBlockStatisticName(BlockStatistic::MotionBufL0).c_str(), mv.hor, mv.ver);
#endif
                  }
                  {
                    const Mv mv = pixMi.mv[RPL1];
#if BLOCK_STATS_AS_CSV
                    g_trace_ctx->dtrace<false>(D_BLOCK_STATISTICS_ALL, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d\n",
                                               cs.picture->poc, cu.lx() + 4 * x, cu.ly() + 4 * y, 4, 4,
                                               GetBlockStatisticName(BlockStatistic::MotionBufL1).c_str(), mv.hor,
                                               mv.ver);
#else
                    g_trace_ctx->dtrace<false>(
                      D_BLOCK_STATISTICS_ALL, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n",
                      cs.picture->m_poc, cu.lx() + 4 * x, cu.ly() + 4 * y, 4, 4,
                      GetBlockStatisticName(BlockStatistic::MotionBufL1).c_str(), mv.hor, mv.ver);
#endif
                  }
                }
              }
            }
          }

          if (cu.geoFlag)
          {
            const uint8_t         candIdx0 = cu.geoMergeIdx[0];
            const uint8_t         candIdx1 = cu.geoMergeIdx[1];
            std::vector<Position> geoPartitions[2];
            Position              linePositions[2];
            retrieveGeoPolygons(cu, geoPartitions, linePositions);
            DTRACE_LINE(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::GeoPartitioning),
                        linePositions[0].x, linePositions[0].y, linePositions[1].x, linePositions[1].y);

            if (geoMergeCtxtsOfCurrentCtu.size() > 0)
            // Geo partition MVs can only be stored when using the statistics with the decoder. Encoder is not supported
            {
              MergeCtx geoMrgCtx = geoMergeCtxtsOfCurrentCtu.front();
              geoMergeCtxtsOfCurrentCtu.pop();

              // first partition
              {
                CodingUnit tmpPu = cu;
                if (candIdx0 >= GEO_MAX_NUM_UNI_CANDS)
                {
                  uint8_t intraMPM[NUM_MOST_PROBABLE_MODES];
                  PU::getGeoIntraMPMs(cu, intraMPM, cu.geoSplitDir,
                                      g_geoTmShape[0][g_geoParams[cu.geoSplitDir].angleIdx]);
                  const int intraDir   = intraMPM[candIdx0 - GEO_MAX_NUM_UNI_CANDS];
                  const int geoPartIdx = 0;
                  DTRACE_POLYGON_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu.slice->m_poc, geoPartitions[geoPartIdx],
                                        GetBlockStatisticName(BlockStatistic::GPMIntra), intraDir);
                }
                else
                {
                  geoMrgCtx.setMergeInfo(tmpPu, candIdx0);
                  const int geoPartIdx = 0;
                  for (int refIdx = 0; refIdx < 2; refIdx++)
                  {
                    if (tmpPu.refIdx[refIdx] != -1)
                    {
                      Mv tmpMv = tmpPu.mv[refIdx];
                      tmpMv.hor =
                        tmpMv.hor >= 0 ? (tmpMv.hor + nOffset) >> nShift : -((-tmpMv.hor + nOffset) >> nShift);
                      tmpMv.ver =
                        tmpMv.ver >= 0 ? (tmpMv.ver + nOffset) >> nShift : -((-tmpMv.ver + nOffset) >> nShift);
                      DTRACE_POLYGON_VECTOR(
                        g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu.slice->m_poc, geoPartitions[geoPartIdx],
                        GetBlockStatisticName(refIdx == 0 ? BlockStatistic::GeoMVL0 : BlockStatistic::GeoMVL1),
                        tmpMv.hor, tmpMv.ver);
                    }
                  }
                }
              }

              // second partition
              {
                CodingUnit tmpPu = cu;
                if (candIdx1 >= GEO_MAX_NUM_UNI_CANDS)
                {
                  uint8_t intraMPM[NUM_MOST_PROBABLE_MODES];
                  PU::getGeoIntraMPMs(cu, intraMPM, cu.geoSplitDir,
                                      g_geoTmShape[1][g_geoParams[cu.geoSplitDir].angleIdx]);
                  const int intraDir   = intraMPM[candIdx1 - GEO_MAX_NUM_UNI_CANDS + GEO_MAX_NUM_INTRA_CANDS];
                  const int geoPartIdx = 0;
                  DTRACE_POLYGON_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu.slice->m_poc, geoPartitions[geoPartIdx],
                                        GetBlockStatisticName(BlockStatistic::GPMIntra), intraDir);
                }
                else
                {
                  geoMrgCtx.setMergeInfo(tmpPu, candIdx1);
                  const int geoPartIdx = 1;
                  {
                    for (int refIdx = 0; refIdx < 2; refIdx++)
                    {
                      if (tmpPu.refIdx[refIdx] != -1)
                      {
                        Mv tmpMv = tmpPu.mv[refIdx];
                        tmpMv.hor =
                          tmpMv.hor >= 0 ? (tmpMv.hor + nOffset) >> nShift : -((-tmpMv.hor + nOffset) >> nShift);
                        tmpMv.ver =
                          tmpMv.ver >= 0 ? (tmpMv.ver + nOffset) >> nShift : -((-tmpMv.ver + nOffset) >> nShift);
                        DTRACE_POLYGON_VECTOR(
                          g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu.slice->m_poc, geoPartitions[geoPartIdx],
                          GetBlockStatisticName(refIdx == 0 ? BlockStatistic::GeoMVL0 : BlockStatistic::GeoMVL1),
                          tmpMv.hor, tmpMv.ver);
                      }
                    }
                  }
                }
              }
            }
          }

          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::SMVDFlag),
                              cu.smvdMode);
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::IMVMode),
                              cu.imv);
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::RootCbf),
                              cu.rootCbf);
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::BCWIndex),
                              cu.bcwIdx);
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::SbtIdx),
                              cu.getSbtIdx());
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::SbtPos),
                              cu.getSbtPos());
        }
        break;
      case MODE_INTRA:
        {
          if (isLuma(chType))
          {
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::MIPFlag),
                                cu.mipFlag);
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                GetBlockStatisticName(BlockStatistic::BvgMIPFlag), cu.bvgMipFlag);
          }

          for (auto chType = ChannelType::LUMA; chType <= ::getLastChannel(cu.chromaFormat); chType++)
          {
            if (cu.block(chType).valid())
            {
              for (const CodingUnit &cu: cs.traverseCUs(CS::getArea(cs, ctuArea, chType), chType))
              {
                if (isLuma(chType))
                {
                  const uint32_t chFinalMode = PU::getFinalIntraMode(cu, ChannelType(chType));
                  DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                      GetBlockStatisticName(BlockStatistic::Luma_IntraMode), chFinalMode);
                  DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                      GetBlockStatisticName(BlockStatistic::MultiRefIdx), cu.multiRefIdx);
                }
                else
                {
                  const uint32_t chFinalMode = PU::getFinalIntraMode(cu, ChannelType(chType));
                  DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                             GetBlockStatisticName(BlockStatistic::Chroma_IntraMode), chFinalMode);
                  // assert(0);
                }
              }
            }
          }
        }
        break;
      case MODE_IBC:
      case MODE_PLT:
        // TODO add relevant statistics
        break;
      default:
        THROW("Invalid prediction mode");
        break;
      }

      for (const TransformUnit &tu: CU::traverseTUs(cu))
      {
        if (tu.Y().valid())
        {
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu, GetBlockStatisticName(BlockStatistic::Cbf_Y),
                              tu.cbf[COMP_Y]);
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu, GetBlockStatisticName(BlockStatistic::MTSIdx_Y),
                              to_underlying(tu.mtsIdx[COMP_Y]));
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu, GetBlockStatisticName(BlockStatistic::LFNSTIdx),
                              TU::getNstIdx(tu, COMP_Y));
        }
        if (tu.Cb().valid())
        {
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu, GetBlockStatisticName(BlockStatistic::JointCbCr),
                              tu.jointCbCr);
        }

        if (isChromaEnabled(cu.chromaFormat))
        {
          DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu,
                                     GetBlockStatisticName(BlockStatistic::Cbf_Cb), tu.cbf[COMP_Cb]);
          DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu,
                                     GetBlockStatisticName(BlockStatistic::Cbf_Cr), tu.cbf[COMP_Cr]);
          DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu,
                                     GetBlockStatisticName(BlockStatistic::MTSIdx_Cb),
                                     to_underlying(tu.mtsIdx[COMP_Cb]));
          DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu,
                                     GetBlockStatisticName(BlockStatistic::MTSIdx_Cr),
                                     to_underlying(tu.mtsIdx[COMP_Cr]));
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu, GetBlockStatisticName(BlockStatistic::LFNSTIdx),
                              TU::getNstIdx(tu, COMP_Cb));
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu, GetBlockStatisticName(BlockStatistic::LFNSTIdx),
                              TU::getNstIdx(tu, COMP_Cr));
        }
      }
    }
  }

  CHECK(geoMergeCtxtsOfCurrentCtu.size() != 0,
        "Did not use all pushed back geo merge contexts. Should not be possible!");
}

void writeAllCodedData(const CodingStructure &cs, const UnitArea &ctuArea)
{
  const int nShift            = MV_FRACTIONAL_BITS_DIFF;
  const int nOffset           = 1 << (nShift - 1);
  const int maxNumChannelType = isChromaEnabled(cs.pcv->chrFormat) && CS::isDualITree(cs) ? 2 : 1;

  for (int ch = 0; ch < maxNumChannelType; ch++)
  {
    const ChannelType chType = ChannelType(ch);

    for (const CodingUnit &cu: cs.traverseCUs(CS::getArea(cs, ctuArea, chType), chType))
    {
      if (isLuma(chType))
      {
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::Depth),
                            cu.depth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::QT_Depth),
                            cu.qtDepth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::BT_Depth),
                            cu.btDepth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::MT_Depth),
                            cu.mtDepth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                            GetBlockStatisticName(BlockStatistic::ChromaQPAdj), cu.chromaQpAdj);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::QP),
                            cu.qp);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                            GetBlockStatisticName(BlockStatistic::SplitSeries), (int)cu.splitSeries);
        // skip flag
        if (!cs.slice->isIntra() && cu.Y().valid())
        {
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                              GetBlockStatisticName(BlockStatistic::SkipFlag), cu.skip);
          if (cu.skip)
          {
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                GetBlockStatisticName(BlockStatistic::MMVDSkipFlag), cu.mmvdSkip);
          }
        }

        // prediction mode and partitioning data
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::PredMode),
                            cu.predMode);
      }
      else if (chType == ChannelType::CHROMA)
      {
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                   GetBlockStatisticName(BlockStatistic::Depth_Chroma), cu.depth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                   GetBlockStatisticName(BlockStatistic::QT_Depth_Chroma), cu.qtDepth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                   GetBlockStatisticName(BlockStatistic::BT_Depth_Chroma), cu.btDepth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                   GetBlockStatisticName(BlockStatistic::MT_Depth_Chroma), cu.mtDepth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                   GetBlockStatisticName(BlockStatistic::ChromaQPAdj_Chroma), cu.chromaQpAdj);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                   GetBlockStatisticName(BlockStatistic::QP_Chroma), cu.qp);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                   GetBlockStatisticName(BlockStatistic::SplitSeries_Chroma), (int)cu.splitSeries);
      }

      for (const CodingUnit &cu: cs.traverseCUs(CS::getArea(cs, ctuArea, chType), chType))
      {
        switch (cu.predMode)
        {
        case MODE_INTRA:
          {
            if (cu.Y().valid())
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                  GetBlockStatisticName(BlockStatistic::Luma_IntraMode),
                                  PU::getFinalIntraMode(cu, ChannelType(chType)));
            }
            if (isChromaEnabled(cu.chromaFormat))
            {
              DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                         GetBlockStatisticName(BlockStatistic::Chroma_IntraMode),
                                         PU::getFinalIntraMode(cu, ChannelType::CHROMA));
            }
            if (cu.Y().valid() && isLuma(cu.chType))
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                  GetBlockStatisticName(BlockStatistic::MultiRefIdx), cu.multiRefIdx);
            }
            break;
          }
        case MODE_INTER:
          {
            if (!cu.skip)
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                  GetBlockStatisticName(BlockStatistic::MergeFlag), cu.mergeFlag);
            }
            if (cu.mergeFlag)
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                  GetBlockStatisticName(BlockStatistic::MergeIdx), cu.mergeIdx);
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                  GetBlockStatisticName(BlockStatistic::MergeType), to_underlying(cu.mergeType));
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                  GetBlockStatisticName(BlockStatistic::MMVDMergeFlag), cu.mmvdMergeFlag);
              if (cu.mmvdMergeFlag)
              {
                DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                    GetBlockStatisticName(BlockStatistic::MMVDMergeIdx), cu.mmvdMergeIdx.val);
              }
              if (!cu.cs->slice->isIntra() && cu.cs->sps->m_useAffine && cu.lumaSize().width >= 8 &&
                  cu.lumaSize().height >= 8 && !cu.mmvdMergeFlag && !cu.mmvdSkip)
              {
                DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                    GetBlockStatisticName(BlockStatistic::AffineFlag), cu.affine);
              }
              if (cu.cs->sps->m_useCiip && !cu.skip && !cu.affine && !(cu.lwidth() * cu.lheight() < 64) &&
                  !cu.mmvdMergeFlag)
              {
                DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                    GetBlockStatisticName(BlockStatistic::CiipFlag), cu.ciipFlag);
                if (cu.ciipFlag)
                {
                  if (cu.Y().valid())
                  {
                    DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                        GetBlockStatisticName(BlockStatistic::Luma_IntraMode),
                                        cu.intraDir[ChannelType::LUMA]);
                    DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                        GetBlockStatisticName(BlockStatistic::Chroma_IntraMode),
                                        cu.intraDir[ChannelType::CHROMA]);
                  }
                }
              }
            }
            else
            {
              if (!cu.cs->slice->isInterP())
              {
                DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                    GetBlockStatisticName(BlockStatistic::InterDir), cu.interDir);
              }
              if (!cu.cs->slice->isIntra() && cu.cs->sps->m_useAffine && cu.lumaSize().width > 8 &&
                  cu.lumaSize().height > 8)
              {
                DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                    GetBlockStatisticName(BlockStatistic::AffineFlag), cu.affine);
                if (cu.affine && !cu.mergeFlag && cu.cs->sps->m_AffineType)
                {
                  DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                      GetBlockStatisticName(BlockStatistic::AffineType), to_underlying(cu.affineType));
                }
              }
            }
            if (cu.interDir != 2 /* PRED_L1 */)
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                  GetBlockStatisticName(BlockStatistic::MVPIdxL0), cu.mvpIdx[RPL0]);
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                  GetBlockStatisticName(BlockStatistic::RefIdxL0), cu.refIdx[RPL0]);
            }
            if (cu.interDir != 1 /* PRED_L1 */)
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                  GetBlockStatisticName(BlockStatistic::MVPIdxL1), cu.mvpIdx[RPL1]);
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                  GetBlockStatisticName(BlockStatistic::RefIdxL1), cu.refIdx[RPL1]);
            }
            if (!cu.affine && !cu.geoFlag)
            {
              if (cu.interDir != 2 /* PRED_L1 */)
              {
                Mv mv   = cu.mv[RPL0];
                Mv mvd  = cu.mvd[RPL0];
                mv.hor  = mv.hor >= 0 ? (mv.hor + nOffset) >> nShift : -((-mv.hor + nOffset) >> nShift);
                mv.ver  = mv.ver >= 0 ? (mv.ver + nOffset) >> nShift : -((-mv.ver + nOffset) >> nShift);
                mvd.hor = mvd.hor >= 0 ? (mvd.hor + nOffset) >> nShift : -((-mvd.hor + nOffset) >> nShift);
                mvd.ver = mvd.ver >= 0 ? (mvd.ver + nOffset) >> nShift : -((-mvd.ver + nOffset) >> nShift);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                    GetBlockStatisticName(BlockStatistic::MVDL0), mvd.hor, mvd.ver);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                    GetBlockStatisticName(BlockStatistic::MVL0), mv.hor, mv.ver);
              }
              if (cu.interDir != 1 /* PRED_L1 */)
              {
                Mv mv   = cu.mv[RPL1];
                Mv mvd  = cu.mvd[RPL1];
                mv.hor  = mv.hor >= 0 ? (mv.hor + nOffset) >> nShift : -((-mv.hor + nOffset) >> nShift);
                mv.ver  = mv.ver >= 0 ? (mv.ver + nOffset) >> nShift : -((-mv.ver + nOffset) >> nShift);
                mvd.hor = mvd.hor >= 0 ? (mvd.hor + nOffset) >> nShift : -((-mvd.hor + nOffset) >> nShift);
                mvd.ver = mvd.ver >= 0 ? (mvd.ver + nOffset) >> nShift : -((-mvd.ver + nOffset) >> nShift);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                    GetBlockStatisticName(BlockStatistic::MVDL1), mvd.hor, mvd.ver);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                    GetBlockStatisticName(BlockStatistic::MVL1), mv.hor, mv.ver);
              }
            }
            else
            {
              if (cu.interDir != 2 /* PRED_L1 */)
              {
                Mv                mv[3];
                const CMotionBuf &mb = cu.getMotionBuf();
                mv[0]                = mb.at(0, 0).mv[RPL0];
                mv[1]                = mb.at(mb.width - 1, 0).mv[RPL0];
                mv[2]                = mb.at(0, mb.height - 1).mv[RPL0];
                // motion vectors should use low precision or they will appear to large
                mv[0].hor = mv[0].hor >= 0 ? (mv[0].hor + nOffset) >> nShift : -((-mv[0].hor + nOffset) >> nShift);
                mv[0].ver = mv[0].ver >= 0 ? (mv[0].ver + nOffset) >> nShift : -((-mv[0].ver + nOffset) >> nShift);
                mv[1].hor = mv[1].hor >= 0 ? (mv[1].hor + nOffset) >> nShift : -((-mv[1].hor + nOffset) >> nShift);
                mv[1].ver = mv[1].ver >= 0 ? (mv[1].ver + nOffset) >> nShift : -((-mv[1].ver + nOffset) >> nShift);
                mv[2].hor = mv[2].hor >= 0 ? (mv[2].hor + nOffset) >> nShift : -((-mv[2].hor + nOffset) >> nShift);
                mv[2].ver = mv[2].ver >= 0 ? (mv[2].ver + nOffset) >> nShift : -((-mv[2].ver + nOffset) >> nShift);
                DTRACE_BLOCK_AFFINETF(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                      GetBlockStatisticName(BlockStatistic::AffineMVL0), mv[0].hor, mv[0].ver,
                                      mv[1].hor, mv[1].ver, mv[2].hor, mv[2].ver);
              }
              if (cu.interDir != 1 /* PRED_L1 */)
              {
                Mv                mv[3];
                const CMotionBuf &mb = cu.getMotionBuf();
                mv[0]                = mb.at(0, 0).mv[RPL1];
                mv[1]                = mb.at(mb.width - 1, 0).mv[RPL1];
                mv[2]                = mb.at(0, mb.height - 1).mv[RPL1];
                // motion vectors should use low precision or they will appear to large
                mv[0].hor = mv[0].hor >= 0 ? (mv[0].hor + nOffset) >> nShift : -((-mv[0].hor + nOffset) >> nShift);
                mv[0].ver = mv[0].ver >= 0 ? (mv[0].ver + nOffset) >> nShift : -((-mv[0].ver + nOffset) >> nShift);
                mv[1].hor = mv[1].hor >= 0 ? (mv[1].hor + nOffset) >> nShift : -((-mv[1].hor + nOffset) >> nShift);
                mv[1].ver = mv[1].ver >= 0 ? (mv[1].ver + nOffset) >> nShift : -((-mv[1].ver + nOffset) >> nShift);
                mv[2].hor = mv[2].hor >= 0 ? (mv[2].hor + nOffset) >> nShift : -((-mv[2].hor + nOffset) >> nShift);
                mv[2].ver = mv[2].ver >= 0 ? (mv[2].ver + nOffset) >> nShift : -((-mv[2].ver + nOffset) >> nShift);
                DTRACE_BLOCK_AFFINETF(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                      GetBlockStatisticName(BlockStatistic::AffineMVL1), mv[0].hor, mv[0].ver,
                                      mv[1].hor, mv[1].ver, mv[2].hor, mv[2].ver);
              }
            }
            if (cu.cs->sps->m_AMVREnabledFlag && CU::hasSubCUNonZeroMVd(cu))
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu,
                                  GetBlockStatisticName(BlockStatistic::IMVMode), cu.imv);
            }
            if (CU::isBcwIdxCoded(cu))
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu,
                                  GetBlockStatisticName(BlockStatistic::BCWIndex), cu.bcwIdx);
            }
            break;
          }
        case MODE_IBC:
        case MODE_PLT:
          // TODO add relevant statistics
          break;
        default:
          {
            THROW("Invalid prediction mode");
            break;
          }
        }
      }   // end cu
      if (CU::isInter(cu))
      {
        if (!cu.mergeFlag)
        {
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::RootCbf),
                              cu.rootCbf);
        }
      }
      if (cu.rootCbf || CU::isIntra(cu))
      {
        for (const TransformUnit &tu: CU::traverseTUs(cu))
        {
          if (tu.Y().valid())
          {
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, tu, GetBlockStatisticName(BlockStatistic::Cbf_Y),
                                tu.cbf[COMP_Y]);
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, tu,
                                GetBlockStatisticName(BlockStatistic::MTSIdx_Y), to_underlying(tu.mtsIdx[COMP_Y]));
          }
          if (isChromaEnabled(cu.chromaFormat))
          {
            DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, tu,
                                       GetBlockStatisticName(BlockStatistic::Cbf_Cb), tu.cbf[COMP_Cb]);
            DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, tu,
                                       GetBlockStatisticName(BlockStatistic::Cbf_Cr), tu.cbf[COMP_Cr]);
            DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, tu,
                                       GetBlockStatisticName(BlockStatistic::MTSIdx_Cb),
                                       to_underlying(tu.mtsIdx[COMP_Cb]));
            DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, tu,
                                       GetBlockStatisticName(BlockStatistic::MTSIdx_Cr),
                                       to_underlying(tu.mtsIdx[COMP_Cr]));
          }
        }
      }
    }
  }
}
#endif
