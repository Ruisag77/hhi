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

#ifndef __CODINGSTATISTICS__
#define __CODINGSTATISTICS__

#include "CommonDef.h"
#include <stdio.h>
#include <string>
#include <map>
#include <math.h>
#include <cinttypes>
#include "ChromaFormat.h"

#ifndef RExt__DECODER_DEBUG_BIT_STATISTICS
#define RExt__DECODER_DEBUG_BIT_STATISTICS \
  0 ///< 0 (default) = decoder reports as normal, 1 = decoder produces bit usage statistics (will impact decoder run
                                                            ///< time by up to ~10%)
#endif

#ifndef RExt__DECODER_DEBUG_TOOL_MAX_FRAME_STATS
#define RExt__DECODER_DEBUG_TOOL_MAX_FRAME_STATS \
  (1 && RExt__DECODER_DEBUG_BIT_STATISTICS)   ///< 0 (default) = decoder reports as normal, 1 = decoder produces max
                                              ///< frame bit usage statistics
#endif

#define TR_ONLY_COEFF_STATS \
  (1 && RExt__DECODER_DEBUG_BIT_STATISTICS)   ///< 0 combine TS and non-TS decoder debug statistics. 1 = separate TS and
                                              ///< non-TS decoder debug statistics.
#define EPBINCOUNT_FIX \
  (1 && RExt__DECODER_DEBUG_BIT_STATISTICS)   ///< 0 use count to represent number of calls to decodeBins. 1 = count and
                                              ///< bins for EP bins are the same.

#ifndef RExt__DECODER_DEBUG_TOOL_STATISTICS
#define RExt__DECODER_DEBUG_TOOL_STATISTICS \
  0   ///< 0 (default) = decoder reports as normal, 1 = decoder produces tool usage statistics
#endif

#if RExt__DECODER_DEBUG_BIT_STATISTICS || RExt__DECODER_DEBUG_TOOL_STATISTICS
#define RExt__DECODER_DEBUG_STATISTICS 1
#endif

#if RExt__DECODER_DEBUG_TOOL_MAX_FRAME_STATS
static constexpr int EPBIN_WEIGHT_FACTOR = 4;
#endif

#if RExt__DECODER_DEBUG_BIT_STATISTICS
static const int64_t CODINGSTATISTICS_ENTROPYSCALE = 32768;

#define CODINGSTATISTICS_LIST                                                                                       \
  CODINGSTATISTICS_ITEM(NAL_UNIT_TOTAL_BODY), /* This is a special case and is not included in the total sums */    \
    CODINGSTATISTICS_ITEM(NAL_UNIT_PACKING), CODINGSTATISTICS_ITEM(EMULATION_PREVENTION_3_BYTES),                   \
    CODINGSTATISTICS_ITEM(NAL_UNIT_HEADER_BITS),                                                                    \
    CODINGSTATISTICS_ITEM(CABAC_INITIALISATION), /* Loosely syntaxically ordered */                                 \
    CODINGSTATISTICS_ITEM(CABAC_BITS__SKIP_FLAG), CODINGSTATISTICS_ITEM(CABAC_BITS__MERGE_FLAG),                    \
    CODINGSTATISTICS_ITEM(CABAC_BITS__MERGE_INDEX), CODINGSTATISTICS_ITEM(CABAC_BITS__MVP_IDX),                     \
    CODINGSTATISTICS_ITEM(CABAC_BITS__SPLIT_FLAG), CODINGSTATISTICS_ITEM(CABAC_BITS__PRED_MODE),                    \
    CODINGSTATISTICS_ITEM(CABAC_BITS__PLT_MODE), CODINGSTATISTICS_ITEM(CABAC_BITS__MIP),                            \
    CODINGSTATISTICS_ITEM(CABAC_BITS__DIMD_FLAG), CODINGSTATISTICS_ITEM(CABAC_BITS__OBIC_FLAG),                     \
    CODINGSTATISTICS_ITEM(CABAC_BITS__TIMD_FLAG), CODINGSTATISTICS_ITEM(CABAC_BITS__TIMDMERGE_FLAG),                \
    CODINGSTATISTICS_ITEM(CABAC_BITS__TIMDSAD_FLAG),                                                                \
    CODINGSTATISTICS_ITEM(CABAC_BITS__EIP_FLAG), CODINGSTATISTICS_ITEM(CABAC_BITS__SGPM),                           \
    CODINGSTATISTICS_ITEM(CABAC_BITS__MULTI_REF_LINE), CODINGSTATISTICS_ITEM(CABAC_BITS__INTRA_DIR_ANG),            \
    CODINGSTATISTICS_ITEM(CABAC_BITS__INTER_DIR), CODINGSTATISTICS_ITEM(CABAC_BITS__REF_FRM_IDX),                   \
    CODINGSTATISTICS_ITEM(CABAC_BITS__MVD), CODINGSTATISTICS_ITEM(CABAC_BITS__MVD_EP),                              \
    CODINGSTATISTICS_ITEM(CABAC_BITS__AFFINE_FLAG), CODINGSTATISTICS_ITEM(CABAC_BITS__AFFINE_DATA),                 \
    CODINGSTATISTICS_ITEM(CABAC_BITS__AFFINE_AMVR), CODINGSTATISTICS_ITEM(CABAC_BITS__QT_ROOT_CBF),                 \
    CODINGSTATISTICS_ITEM(CABAC_BITS__DELTA_QP_EP), CODINGSTATISTICS_ITEM(CABAC_BITS__CHROMA_QP_ADJUSTMENT),        \
    CODINGSTATISTICS_ITEM(CABAC_BITS__QT_CBF), CODINGSTATISTICS_ITEM(CABAC_BITS__JOINT_CB_CR),                      \
    CODINGSTATISTICS_ITEM(CABAC_BITS__MTS_FLAGS), CODINGSTATISTICS_ITEM(CABAC_BITS__LAST_SIG_X_Y),                  \
    CODINGSTATISTICS_ITEM(CABAC_BITS__SIG_COEFF_GROUP_FLAG), CODINGSTATISTICS_ITEM(CABAC_BITS__SIG_COEFF_MAP_FLAG), \
    CODINGSTATISTICS_ITEM(CABAC_BITS__PAR_FLAG), CODINGSTATISTICS_ITEM(CABAC_BITS__GT1_FLAG),                       \
    CODINGSTATISTICS_ITEM(CABAC_BITS__GT2_FLAG), CODINGSTATISTICS_ITEM(CABAC_BITS__GT3_FLAG),                       \
    CODINGSTATISTICS_ITEM(CABAC_BITS__GT4_FLAG), CODINGSTATISTICS_ITEM(CABAC_BITS__SIGN_BIT),                       \
    CODINGSTATISTICS_ITEM(CABAC_BITS__ESCAPE_BITS), CODINGSTATISTICS_ITEM(CABAC_BITS__SIG_COEFF_MAP_FLAG_TS),       \
    CODINGSTATISTICS_ITEM(CABAC_BITS__PAR_FLAG_TS), CODINGSTATISTICS_ITEM(CABAC_BITS__GT1_FLAG_TS),                 \
    CODINGSTATISTICS_ITEM(CABAC_BITS__GT2_FLAG_TS), CODINGSTATISTICS_ITEM(CABAC_BITS__SIGN_BIT_TS),                 \
    CODINGSTATISTICS_ITEM(CABAC_BITS__ESCAPE_BITS_TS), CODINGSTATISTICS_ITEM(CABAC_BITS__BIF),                      \
    CODINGSTATISTICS_ITEM(CABAC_BITS__SAO), CODINGSTATISTICS_ITEM(CABAC_BITS__LFNST),                               \
    CODINGSTATISTICS_ITEM(CABAC_BITS__ALF), CODINGSTATISTICS_ITEM(CABAC_BITS__BVD),                                 \
    CODINGSTATISTICS_ITEM(CABAC_BITS__OBMC), CODINGSTATISTICS_ITEM(CABAC_TRM_BITS),                                 \
    CODINGSTATISTICS_ITEM(BYTE_ALIGNMENT_BITS), CODINGSTATISTICS_ITEM(TRAILING_BITS),                               \
    CODINGSTATISTICS_ITEM(CABAC_EP_BIT_ALIGNMENT), CODINGSTATISTICS_ITEM(CABAC_BITS__ALIGNED_SIGN_BIT),             \
    CODINGSTATISTICS_ITEM(CABAC_BITS__ALIGNED_ESCAPE_BITS),                                                         \
    CODINGSTATISTICS_ITEM(CABAC_BITS__OTHER), /* unclassified bits */                                               \
    CODINGSTATISTICS_ITEM(CABAC_BITS__INVALID), CODINGSTATISTICS_ITEM(CABAC_BITS__IMV_FLAG),                        \
    CODINGSTATISTICS_ITEM(CABAC_BITS__BCW_IDX), CODINGSTATISTICS_ITEM(CABAC_BITS__SBT_MODE),                        \
    CODINGSTATISTICS_ITEM(CABAC_BITS__MH_INTRA_FLAG), CODINGSTATISTICS_ITEM(CABAC_BITS__GEO_INDEX),                 \
    CODINGSTATISTICS_ITEM(CABAC_BITS__SYMMVD_FLAG), CODINGSTATISTICS_ITEM(CABAC_BITS__BDPCM_MODE),                  \
    CODINGSTATISTICS_ITEM(CABAC_BITS__CROSS_COMP_ALF_BLOCK_LEVEL_IDC), CODINGSTATISTICS_ITEM(CABAC_BITS__LIC_FLAG), \
    CODINGSTATISTICS_ITEM(CABAC_BITS__NNLF), CODINGSTATISTICS_ITEM(TOOL_TOTAL_FRAME),                               \
    /* This is a special case and is not included in the report */ /* Alphabetically ordered */                     \
    CODINGSTATISTICS_ITEM(TOOL_AFF), CODINGSTATISTICS_ITEM(TOOL_DIMD), CODINGSTATISTICS_ITEM(TOOL_EIP),             \
    CODINGSTATISTICS_ITEM(TOOL_EMT), CODINGSTATISTICS_ITEM(TOOL_LFNST), CODINGSTATISTICS_ITEM(TOOL_MIP),            \
    CODINGSTATISTICS_ITEM(TOOL_OBIC), CODINGSTATISTICS_ITEM(TOOL_TIMD), CODINGSTATISTICS_ITEM(TOOL_TIMDMERGE),      \
    CODINGSTATISTICS_ITEM(TOOL_TIMDSAD),                                                                            \
    CODINGSTATISTICS_ITEM(TOOL_TOTAL)

#define CODINGSTATISTICS_ITEM(x) STATS__##x
enum CodingStatisticsType
{
  CODINGSTATISTICS_LIST,
  CODINGSTATISTICS_ITEM(NUM_STATS)
};
#undef CODINGSTATISTICS_ITEM

enum CodingStatisticsMode
{
  STATS__MODE_NONE  = 0,
  STATS__MODE_BITS  = 1,
  STATS__MODE_TOOLS = 2,

  STATS__MODE_ALL = STATS__MODE_BITS | STATS__MODE_TOOLS
};

static inline const char *getName(CodingStatisticsType name)
{
  CHECK(name >= STATS__NUM_STATS, "stats out of range");

#define CODINGSTATISTICS_ITEM(x) #x
  static const char *statNames[] = { CODINGSTATISTICS_LIST };
#undef CODINGSTATISTICS_ITEM
  static_assert(STATS__NUM_STATS == sizeof(statNames) / sizeof(char *),
                "statNames contains different number of entries than traced channels");
  return statNames[name];
}

static inline bool isAlignedBins(CodingStatisticsType statT)
{
  return statT == STATS__CABAC_BITS__ALIGNED_SIGN_BIT || statT == STATS__CABAC_BITS__ALIGNED_ESCAPE_BITS;
}

static const uint32_t CODING_STATS_NUM_WIDTHS     = 20;   // just define the number of widths and heigts as 15
static const uint32_t CODING_STATS_NUM_HEIGHTS    = 20;
static const uint32_t CODING_STATS_NUM_SIZES      = CODING_STATS_NUM_HEIGHTS * CODING_STATS_NUM_WIDTHS;
static const uint32_t CODING_STATS_NUM_SUBCLASSES = CODING_STATS_NUM_SIZES * (1 + MAX_NUM_COMP + MAX_NUM_CHANNEL_TYPE);

class CodingStatisticsClassType
{
public:
  CodingStatisticsClassType(const CodingStatisticsType t) : type(t), subClass(0) {}

  CodingStatisticsClassType(const CodingStatisticsType t, const uint32_t width, const uint32_t height)
    : type(t)
    , subClass(gp_sizeIdxInfo->idxFrom(height) * CODING_STATS_NUM_WIDTHS + gp_sizeIdxInfo->idxFrom(width))
  {}

  CodingStatisticsClassType(const CodingStatisticsType t, const int width, const int height)
    : type(t)
    , subClass(gp_sizeIdxInfo->idxFrom(height) * CODING_STATS_NUM_WIDTHS + gp_sizeIdxInfo->idxFrom(width))
  {}

  CodingStatisticsClassType(const CodingStatisticsType t, const CompID cid)
    : type(t)
    , subClass((cid + 1) * CODING_STATS_NUM_SIZES)
  {}

  CodingStatisticsClassType(const CodingStatisticsType t, const ChannelType chid)
    : type(t)
    , subClass((to_underlying(chid) + MAX_NUM_COMP + 1) * CODING_STATS_NUM_SIZES)
  {}

  CodingStatisticsClassType(const CodingStatisticsType t, const uint32_t width, const uint32_t height, const CompID cid)
    : type(t)
    , subClass((cid + 1) * CODING_STATS_NUM_SIZES + gp_sizeIdxInfo->idxFrom(height) * CODING_STATS_NUM_WIDTHS +
               gp_sizeIdxInfo->idxFrom(width))
  {}

  CodingStatisticsClassType(const CodingStatisticsType t, const uint32_t width, const uint32_t height,
                            const ChannelType chid)
    : type(t)
    , subClass((to_underlying(chid) + MAX_NUM_COMP + 1) * CODING_STATS_NUM_SIZES +
               gp_sizeIdxInfo->idxFrom(height) * CODING_STATS_NUM_WIDTHS + gp_sizeIdxInfo->idxFrom(width))
  {}

  ~CodingStatisticsClassType()
  {
    // should not be relevant, but it looks like some instances are used after they are destroyed
    type     = STATS__CABAC_BITS__INVALID;
    subClass = 0;
  }

  static uint32_t GetSubClassWidth(const uint32_t subClass) { return subClass % CODING_STATS_NUM_WIDTHS; }

  static uint32_t GetSubClassHeight(const uint32_t subClass)
  {
    return (subClass % CODING_STATS_NUM_SIZES) / CODING_STATS_NUM_WIDTHS;
  }

  static const char *GetSubClassString(const uint32_t subClass)
  {
    CHECK(subClass >= CODING_STATS_NUM_SUBCLASSES, "Subclass does not exist");
    static const char *strings[1 + MAX_NUM_COMP + MAX_NUM_CHANNEL_TYPE] = { "-", "Y", "Cb", "Cr", "Luma", "Chroma" };
    return strings[subClass / CODING_STATS_NUM_SIZES];
  }

  CodingStatisticsType type;
  uint32_t             subClass;
};

class CodingStatistics
{
public:
  struct StatLogValue
  {
    uint32_t values[512 + 1];
    StatLogValue()
    {
      const double es = double(CODINGSTATISTICS_ENTROPYSCALE);

      values[0] = 0;

      for (uint32_t i = 1; i < sizeof(values) / sizeof(uint32_t); i++)
      {
        values[i] = uint32_t(log(double(i)) * es / log(2.0));
      }
    }
  };

  struct SStat
  {
    SStat() : bits(0), count(0), sum(0), classCount(0) {}

    int64_t bits;
    int64_t count;
    int64_t sum;
    int64_t classCount;

    void clear()
    {
      bits       = 0;
      count      = 0;
      sum        = 0;
      classCount = 0;
    }

    SStat &operator+=(const SStat &src)
    {
      bits += src.bits;
      count += src.count;
      sum += src.sum;
      classCount += src.classCount;
      return *this;
    }

#if RExt__DECODER_DEBUG_TOOL_MAX_FRAME_STATS
    SStat &operator-=(const SStat &src)
    {
      bits -= src.bits;
      count -= src.count;
      sum -= src.sum;
      classCount -= src.classCount;
      return *this;
    }
#endif
  };

  struct StatTool
  {
    StatTool() : count(0), pixels(0), classCount(0) {}

    int64_t count;
    int64_t pixels;
    int64_t classCount;

    void clear()
    {
      count      = 0;
      pixels     = 0;
      classCount = 0;
    }

    StatTool &operator+=(const StatTool &src)
    {
      count += src.count;
      pixels += src.pixels;
      classCount += src.classCount;
      return *this;
    }
  };

#if RExt__DECODER_DEBUG_TOOL_MAX_FRAME_STATS
  struct SStat_max
  {
    SStat max_CABAC_state;
    SStat max_EP_state;
    SStat trf_CABAC_state;
    SStat trf_EP_state;
    SStat acc_trf_CABAC_state;
    SStat acc_trf_EP_state;
    SStat prev_CABAC_state;
    SStat prev_EP_state;
    SStat prev_trf_CABAC_state;
    SStat prev_trf_EP_state;

    void clear()
    {
      max_CABAC_state.clear();
      max_EP_state.clear();
      trf_CABAC_state.clear();
      trf_EP_state.clear();
      acc_trf_CABAC_state.clear();
      acc_trf_EP_state.clear();
      prev_CABAC_state.clear();
      prev_EP_state.clear();
      prev_trf_CABAC_state.clear();
      prev_trf_EP_state.clear();
    }
  };
#endif

  class CodingStatisticsData
  {
  private:
    SStat    statistics[STATS__NUM_STATS + 1][CODING_STATS_NUM_SUBCLASSES];
    SStat    statistics_ep[STATS__NUM_STATS + 1][CODING_STATS_NUM_SUBCLASSES];
    StatTool statistics_tool[STATS__NUM_STATS + 1][CODING_STATS_NUM_SUBCLASSES];
#if RExt__DECODER_DEBUG_TOOL_MAX_FRAME_STATS
    SStat_max statistics_max;
#endif
    std::map<std::string, SStat> mappings_ep;
    friend class CodingStatistics;
  };

  int m_mode;

private:
  CodingStatisticsData data;

  CodingStatistics() : m_mode(STATS__MODE_ALL), data() {}

  static void OutputLine(const char *pName, const char sep, uint32_t wIdx, uint32_t hIdx, const char *pSubClassStr,
                         const SStat &sCABAC, const SStat &sEP)
  {
    if (wIdx == 0 && hIdx == 0)
    {
      OutputLine(pName, sep, "-", "-", pSubClassStr, sCABAC, sEP);
    }
    else
    {
      printf("%c%-45s%c  %6d %6d %6s ", sep == '~' ? '[' : ' ', pName, sep, gp_sizeIdxInfo->sizeFrom(wIdx),
             gp_sizeIdxInfo->sizeFrom(hIdx), pSubClassStr);
      if (sCABAC.count > 0)
      {
        const double quote = 100.0 * sCABAC.count / (double)sCABAC.classCount;
        const double ratio = 100.0 * sCABAC.bits / (double)sCABAC.count;
        printf("%11.2f%% %12" PRId64 " %12" PRId64 " %12" PRId64 " %11.2f%%", quote, sCABAC.count, sCABAC.sum,
               sCABAC.bits, ratio);
      }
      else
      {
        printf("         -/- %12" PRId64 " %12" PRId64 " %12" PRId64 "          -/-", sCABAC.count, sCABAC.sum,
               sCABAC.bits);
      }
      printf(" %12" PRId64 " %12" PRId64 " %12" PRId64 " %12" PRId64 " (%12" PRId64 ")%c\n", sEP.count, sEP.sum,
             sEP.bits, sCABAC.bits + sEP.bits, (sCABAC.bits + sEP.bits) / 8, sep == '~' ? ']' : ' ');
    }
  }
  static void OutputLine(const char *pName, const char sep, const char *pWidthString, const char *pHeightString,
                         const char *pSubClassStr, const SStat &sCABAC, const SStat &sEP)
  {
    printf("%c%-45s%c  %6s %6s %6s ", sep == '~' ? '[' : ' ', pName, sep, pWidthString, pHeightString, pSubClassStr);
    if (sCABAC.count > 0)
    {
      const double quote = 100.0 * sCABAC.count / (double)sCABAC.classCount;
      const double ratio = 100.0 * sCABAC.bits / (double)sCABAC.count;
      printf("%11.2f%% %12" PRId64 " %12" PRId64 " %12" PRId64 " %11.2f%%", quote, sCABAC.count, sCABAC.sum,
             sCABAC.bits, ratio);
    }
    else
    {
      printf("         -/- %12" PRId64 " %12" PRId64 " %12" PRId64 "          -/-", sCABAC.count, sCABAC.sum,
             sCABAC.bits);
    }
    printf(" %12" PRId64 " %12" PRId64 " %12" PRId64 " %12" PRId64 " (%12" PRId64 ")%c\n", sEP.count, sEP.sum, sEP.bits,
           sCABAC.bits + sEP.bits, (sCABAC.bits + sEP.bits) / 8, sep == '~' ? ']' : ' ');
  }
  static void OutputLine(const char *pName, const char sep, const char *pWidthString, const char *pHeightString,
                         const char *pSubClassStr, const SStat &sEP)
  {
    printf("%c%-45s%c  %6s %6s %6s          -/- %12s %12s %12s %9s-/- %12" PRId64 " %12" PRId64 " %12" PRId64
           " %12" PRId64 " (%12" PRId64 ")%c\n",
           sep == '~' ? '[' : ' ', pName, sep, pWidthString, pHeightString, pSubClassStr, "", "", "", "", sEP.count,
           sEP.sum, sEP.bits, sEP.bits, (sEP.bits) / 8, sep == '~' ? ']' : ' ');
  }

  static void OutputLine(const char *pName, const char sep, const char *pWidthString, const char *pHeightString,
                         const char *pSubClassStr, const StatTool &sTool, uint64_t totalPixels)
  {
    const double ratio = 100.0 * sTool.pixels / (double)totalPixels;
    printf("%c%-45s%c  %6s %6s %6s %12" PRId64 "     %12" PRId64 "       %11.2f%%%c\n", sep == '~' ? '[' : ' ', pName,
           sep, pWidthString, pHeightString, pSubClassStr, sTool.count, sTool.pixels, ratio, sep == '~' ? ']' : ' ');
  }

  static void OutputLine(const char *pName, const char sep, uint32_t wIdx, uint32_t hIdx, const char *pSubClassStr,
                         const StatTool &sTool, uint64_t totalPixels)
  {
    const double ratio = 100.0 * sTool.pixels / (double)totalPixels;
    printf("%c%-45s%c  %6d %6d %6s %12" PRId64 "     %12" PRId64 "       %11.2f%%%c\n", sep == '~' ? '[' : ' ', pName,
           sep, gp_sizeIdxInfo->sizeFrom(wIdx), gp_sizeIdxInfo->sizeFrom(hIdx), pSubClassStr, sTool.count, sTool.pixels,
           ratio, sep == '~' ? ']' : ' ');
  }

  static void OutputDashedLine(const char *pText)
  {
    printf("--%s", pText);
    uint32_t tot = 0;
    for (; pText[tot] != 0; tot++)
      ;

    tot += 2;
    for (; tot < 202; tot++)
    {
      printf("-");
    }
    printf("\n");
  }

  void OutputBitStats()
  {
    const int64_t es = CODINGSTATISTICS_ENTROPYSCALE;

    int64_t countTotal = 0;
    int64_t classCounts[STATS__NUM_STATS];
    std::fill_n(classCounts, (size_t)STATS__NUM_STATS, 0);

#if RExt__DECODER_DEBUG_TOOL_MAX_FRAME_STATS
    SStat_max &max = GetStatisticMax();
#endif

    int64_t cr = 0;   // CABAC remainder, which is added to "STATS__CABAC_INITIALISATION"
    {
      int64_t totalCABACbits = 0, roundedCABACbits = 0;
      for (int i = STATS__NAL_UNIT_PACKING; i < STATS__NUM_STATS; i++)
      {
        int64_t classCount = 0;

        for (uint32_t c = 0; c < CODING_STATS_NUM_SUBCLASSES; c++)
        {
          totalCABACbits += data.statistics[i][c].bits;
          roundedCABACbits += data.statistics[i][c].bits / es;
          classCount += data.statistics[i][c].count;
        }

        for (uint32_t c = 0; c < CODING_STATS_NUM_SUBCLASSES; c++)
        {
          data.statistics[i][c].classCount = classCount;
        }

        classCounts[i] = classCount;
        countTotal += classCount;
      }
      int64_t remainder = totalCABACbits - roundedCABACbits * es;
      cr                = (remainder + es / 2) / es;
    }

    classCounts[0] = countTotal;

    printf("Note %s will be excluded from the total as it should be the sum of all the other entries (except for %s)\n",
           getName(STATS__NAL_UNIT_TOTAL_BODY), getName(STATS__NAL_UNIT_PACKING));
    printf(" %-45s-   Width Height   Type  CABAC quote  CABAC count    CABAC Sum   CABAC bits  CABAC ratio     EP "
           "Count       EP Sum      EP bits   Total bits ( Total bytes)\n",
           "Decoder statistics");

    OutputDashedLine("");
    SStat cabacTotalBits, epTotalBits;

    cabacTotalBits.classCount = countTotal;
    epTotalBits.classCount    = countTotal;

    SStat statTotals_cabac[CODING_STATS_NUM_SUBCLASSES];
    SStat statTotals_ep[CODING_STATS_NUM_SUBCLASSES];

    for (int i = 0; i < STATS__NUM_STATS; i++)
    {
      SStat cabacSubTotal, epSubTotal;
      cabacSubTotal.classCount = classCounts[i];
      epSubTotal.classCount    = classCounts[i];
      bool bHadClassifiedEntry = false;

      const char *pName = getName(CodingStatisticsType(i));

      for (uint32_t c = 0; c < CODING_STATS_NUM_SUBCLASSES; c++)
      {
        SStat &sCABACorig = data.statistics[i][c];
        SStat &sEP        = data.statistics_ep[i][c];

        if (sCABACorig.bits == 0 && sEP.bits == 0)
        {
          continue;
        }

        SStat sCABAC;
        {
          int64_t thisCABACbits = sCABACorig.bits / es;
          if (i == STATS__CABAC_INITIALISATION && sCABACorig.bits != 0)
          {
            thisCABACbits += cr;
#if EPBINCOUNT_FIX
            sCABACorig.count = 0;
#endif
            cr = 0;
          }
          sCABAC.bits       = thisCABACbits;
          sCABAC.count      = sCABACorig.count;
          sCABAC.sum        = sCABACorig.sum;
          sCABAC.classCount = classCounts[i];
        }
#if EPBINCOUNT_FIX
        if (i == STATS__BYTE_ALIGNMENT_BITS || i == STATS__TRAILING_BITS || i == STATS__NAL_UNIT_HEADER_BITS ||
            i == STATS__EMULATION_PREVENTION_3_BYTES)
        {
          sEP.count = 0;
        }
#endif
        uint32_t wIdx = CodingStatisticsClassType::GetSubClassWidth(c);
        uint32_t hIdx = CodingStatisticsClassType::GetSubClassHeight(c);
        OutputLine(pName, ':', wIdx, hIdx, CodingStatisticsClassType::GetSubClassString(c), sCABAC, sEP);
        cabacSubTotal += sCABAC;
        epSubTotal += sEP;

        if (i != STATS__NAL_UNIT_TOTAL_BODY)
        {
          cabacTotalBits += sCABAC;
          epTotalBits += sEP;
          statTotals_cabac[c] += sCABAC;
          statTotals_ep[c] += sEP;
        }
        bHadClassifiedEntry = bHadClassifiedEntry || (c != 0);
      }

      if (bHadClassifiedEntry)
      {
        cabacSubTotal.classCount = classCounts[i];
        OutputLine(pName, '~', "~~ST~~", "~~ST~~", "~~ST~~", cabacSubTotal, epSubTotal);

#if RExt__DECODER_DEBUG_TOOL_MAX_FRAME_STATS
        // For TRF
        if ((i == STATS__CABAC_BITS__SIG_COEFF_MAP_FLAG) || (i == STATS__CABAC_BITS__PAR_FLAG) ||
            (i == STATS__CABAC_BITS__GT1_FLAG) || (i == STATS__CABAC_BITS__GT2_FLAG) ||
            (i == STATS__CABAC_BITS__GT3_FLAG) || (i == STATS__CABAC_BITS__GT4_FLAG) ||
            (i == STATS__CABAC_BITS__ESCAPE_BITS))
        {
          max.acc_trf_CABAC_state += cabacSubTotal;
          max.acc_trf_EP_state += epSubTotal;
        }
#endif
      }
      if (i == STATS__NAL_UNIT_TOTAL_BODY)
      {
        OutputDashedLine("");
      }
    }
    OutputDashedLine("");
    OutputLine("CABAC Sub-total", '~', "~~ST~~", "~~ST~~", "~~ST~~", cabacTotalBits, epTotalBits);

    OutputDashedLine("CAVLC HEADER BITS");
    SStat cavlcTotalBits;
    for (std::map<std::string, SStat>::iterator it = data.mappings_ep.begin(); it != data.mappings_ep.end(); it++)
    {
      SStat s = it->second;
      cavlcTotalBits += s;
      OutputLine(it->first.c_str(), ':', "-", "-", "-", s);
    }

    OutputDashedLine("");
    OutputLine("CAVLC Header Sub-total", '~', "~~ST~~", "~~ST~~", "~~ST~~", cavlcTotalBits);

    // Now output the breakdowns
    OutputDashedLine("CABAC Break down by size");
    for (uint32_t w = 0; w < CODING_STATS_NUM_WIDTHS; w++)
    {
      for (uint32_t h = 0; h < CODING_STATS_NUM_HEIGHTS; h++)
      {
        SStat subTotalCabac, subTotalEP;
        for (uint32_t c = 0; c < CODING_STATS_NUM_SUBCLASSES; c += CODING_STATS_NUM_SIZES)
        {
          subTotalCabac += statTotals_cabac[c + h * CODING_STATS_NUM_WIDTHS + w];
          subTotalEP += statTotals_ep[c + h * CODING_STATS_NUM_WIDTHS + w];
        }
        if (subTotalCabac.bits != 0 || subTotalEP.bits != 0)
        {
          OutputLine("CABAC by size Sub-total", '=', w, h, "All", subTotalCabac, subTotalEP);
        }
      }
    }
    OutputDashedLine("Break down by component/Channel type");
    for (uint32_t c = 0; c < CODING_STATS_NUM_SUBCLASSES; c += CODING_STATS_NUM_SIZES)
    {
      SStat subTotalCabac, subTotalEP;
      for (uint32_t w = 0; w < CODING_STATS_NUM_WIDTHS; w++)
      {
        for (uint32_t h = 0; h < CODING_STATS_NUM_HEIGHTS; h++)
        {
          subTotalCabac += statTotals_cabac[c + h * CODING_STATS_NUM_WIDTHS + w];
          subTotalEP += statTotals_ep[c + h * CODING_STATS_NUM_WIDTHS + w];
        }
      }
      if (subTotalCabac.bits != 0 || subTotalEP.bits != 0)
      {
        OutputLine("CABAC by type Sub-total", '=', "-", "-", CodingStatisticsClassType::GetSubClassString(c),
                   subTotalCabac, subTotalEP);
      }
    }
    OutputDashedLine("Break down by size and component/Channel type");
    for (uint32_t c = 0; c < CODING_STATS_NUM_SUBCLASSES; c += CODING_STATS_NUM_SIZES)
    {
      for (uint32_t w = 0; w < CODING_STATS_NUM_WIDTHS; w++)
      {
        for (uint32_t h = 0; h < CODING_STATS_NUM_HEIGHTS; h++)
        {
          SStat subTotalCabac, subTotalEP;
          subTotalCabac += statTotals_cabac[c + h * CODING_STATS_NUM_WIDTHS + w];
          subTotalEP += statTotals_ep[c + h * CODING_STATS_NUM_WIDTHS + w];

          if (subTotalCabac.bits != 0 || subTotalEP.bits != 0)
          {
            OutputLine("CABAC by size and type Sub-total", '=', w, h, CodingStatisticsClassType::GetSubClassString(c),
                       subTotalCabac, subTotalEP);
          }
        }
      }
    }

    OutputDashedLine("");
    cabacTotalBits.classCount = countTotal;
    OutputLine("CABAC Sub-total", '~', "~~ST~~", "~~ST~~", "~~ST~~", cabacTotalBits, epTotalBits);
    OutputLine("CAVLC Header Sub-total", '~', "~~ST~~", "~~ST~~", "~~ST~~", cavlcTotalBits);
    OutputDashedLine("GRAND TOTAL");
    epTotalBits += cavlcTotalBits;
    OutputLine("TOTAL", '~', "~~GT~~", "~~GT~~", "~~GT~~", cabacTotalBits, epTotalBits);
#if RExt__DECODER_DEBUG_TOOL_MAX_FRAME_STATS
    OutputDashedLine("");
    OutputLine("CABAC MAX FRAME stat", '~', "~~ST~~", "~~ST~~", "~~ST~~", max.max_CABAC_state, max.max_EP_state);
    OutputLine("CABAC MAX FRAME TRF stat", '~', "~~ST~~", "~~ST~~", "~~ST~~", max.trf_CABAC_state, max.trf_EP_state);
    OutputLine("CABAC Accumulated TRF stat", '~', "~~ST~~", "~~ST~~", "~~ST~~", max.acc_trf_CABAC_state,
               max.acc_trf_EP_state);
#endif
  }

  void OutputToolStats()
  {
    printf("\n");
    printf(" %-45s-   Width Height   Type        Count  Impacted pixels  %% Impacted pixels\n", "Tools statistics");
    OutputDashedLine("");

    const uint64_t toolCount = STATS__TOOL_TOTAL - (STATS__TOOL_TOTAL_FRAME + 1);
    StatTool       subTotalTool[toolCount];
    StatTool       statTotalTool[toolCount][CODING_STATS_NUM_SUBCLASSES];
    uint64_t       totalPixels = GetStatisticTool(STATS__TOOL_TOTAL_FRAME).pixels;
    for (int i = 0; i < toolCount; i++)
    {
      const int   type  = i + (STATS__TOOL_TOTAL_FRAME + 1);
      const char *pName = getName(CodingStatisticsType(type));

      for (uint32_t c = 0; c < CODING_STATS_NUM_SUBCLASSES; c++)
      {
        StatTool &sTool = data.statistics_tool[type][c];
        if (sTool.count == 0)
        {
          continue;
        }

        uint32_t wIdx = CodingStatisticsClassType::GetSubClassWidth(c);
        uint32_t hIdx = CodingStatisticsClassType::GetSubClassHeight(c);
        OutputLine(pName, ':', wIdx, hIdx, CodingStatisticsClassType::GetSubClassString(c), sTool, totalPixels);

        statTotalTool[i][c] += sTool;
        subTotalTool[i] += sTool;
      }

      if (subTotalTool[i].count != 0)
      {
        OutputLine(pName, '~', "~~ST~~", "~~ST~~", "~~ST~~", subTotalTool[i], totalPixels);
      }
    }

    for (int i = 0; i < toolCount; i++)
    {
      const int   type  = i + (STATS__TOOL_TOTAL_FRAME + 1);
      const char *pName = getName(CodingStatisticsType(type));

      if (subTotalTool[i].count != 0)
      {
        OutputDashedLine("Break down by tool/Channel type");
      }

      for (uint32_t c = 0; c < CODING_STATS_NUM_SUBCLASSES; c += CODING_STATS_NUM_SIZES)
      {
        StatTool typeTotalTool;
        for (uint32_t w = 0; w < CODING_STATS_NUM_WIDTHS; w++)
        {
          for (uint32_t h = 0; h < CODING_STATS_NUM_HEIGHTS; h++)
          {
            typeTotalTool += statTotalTool[i][c + h * CODING_STATS_NUM_WIDTHS + w];
          }
        }

        if (typeTotalTool.count != 0)
        {
          OutputLine(pName, '=', "-", "-", CodingStatisticsClassType::GetSubClassString(c), typeTotalTool, totalPixels);
        }
      }
    }
  }

public:
  ~CodingStatistics()
  {
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    if (m_mode & STATS__MODE_BITS)
    {
      OutputBitStats();
    }
#endif   // RExt__DECODER_DEBUG_BIT_STATISTICS

#ifdef RExt__DECODER_DEBUG_TOOL_STATISTICS
    if (m_mode & STATS__MODE_TOOLS)
    {
      OutputToolStats();
    }
#endif   // RExt__DECODER_DEBUG_TOOL_STATISTICS
  }

  static CodingStatistics &GetSingletonInstance()
  {
    static CodingStatistics *inst = nullptr;
    if (!inst)
    {
      inst = new CodingStatistics;
    }

    return *inst;
  }

  static void DestroyInstance()
  {
    CodingStatistics *cs = &GetSingletonInstance();
    delete cs;
  }

  static const CodingStatisticsData &GetStatistics() { return GetSingletonInstance().data; }

  static void SetStatistics(const CodingStatisticsData &src) { GetSingletonInstance().data = src; }

  static SStat &GetStatisticEP(const CodingStatisticsClassType &stat)
  {
    return GetSingletonInstance().data.statistics_ep[stat.type][stat.subClass];
  }

  static SStat &GetStatisticEP(const std::string &str) { return GetSingletonInstance().data.mappings_ep[str]; }

  static SStat &GetStatisticEP(const char *pKey) { return GetStatisticEP(std::string(pKey)); }

  static StatTool &GetStatisticTool(const CodingStatisticsClassType &stat)
  {
    return GetSingletonInstance().data.statistics_tool[stat.type][stat.subClass];
  }

#if RExt__DECODER_DEBUG_TOOL_MAX_FRAME_STATS
  static SStat_max &GetStatisticMax() { return GetSingletonInstance().data.statistics_max; }
#endif

  static int getNumOnes(int bins)
  {
    CHECK(bins < 0, "Bins should not be nagative");

    int count = 0;
    while (bins)
    {
      count += bins & 1;
      bins >>= 1;
    }
    return count;
  }

  static void IncrementStatisticEP(const CodingStatisticsClassType &stat, const int numBits, const int value)
  {
    CHECK(stat.type == STATS__CABAC_BITS__INVALID, "Should never be used.");
    SStat &s = GetStatisticEP(stat);
    s.bits += numBits;
#if EPBINCOUNT_FIX
    s.count += numBits;
#else
    s.count++;
#endif
    s.sum += getNumOnes(value);
  }

  static void IncrementStatisticEP(const std::string &str, const int numBits, const int value)
  {
    SStat &s = GetStatisticEP(str);
    s.bits += numBits;
#if EPBINCOUNT_FIX
    s.count += numBits;
#else
    s.count++;
#endif
    s.sum += getNumOnes(value);
  }

  static void IncrementStatisticEP(const char *pKey, const int numBits, const int value)
  {
    SStat &s = GetStatisticEP(pKey);
    s.bits += numBits;
#if EPBINCOUNT_FIX
    s.count += numBits;
#else
    s.count++;
#endif
    s.sum += getNumOnes(value);
  }

  static void IncrementStatisticTool(const CodingStatisticsClassType &stat)
  {
    CHECK(stat.type < STATS__TOOL_TOTAL_FRAME || stat.type >= STATS__TOOL_TOTAL, "Should never be used.");
    StatTool &s = GetStatisticTool(stat);
    s.count++;

    uint32_t wIdx = CodingStatisticsClassType::GetSubClassWidth(stat.subClass);
    uint32_t hIdx = CodingStatisticsClassType::GetSubClassHeight(stat.subClass);

    s.pixels = s.count * gp_sizeIdxInfo->sizeFrom(wIdx) * gp_sizeIdxInfo->sizeFrom(hIdx);
  }

  StatLogValue values;

  static void UpdateCABACStat(const CodingStatisticsClassType &stat, uint32_t uiRangeBefore, uint32_t uiRangeAfter,
                              int val)
  {
    CHECK(stat.type == STATS__CABAC_BITS__INVALID, "Should never be used.");
    CodingStatistics &inst = GetSingletonInstance();
    // doing rangeBefore*p(x)=rangeAfter
    // p(x)=rangeAfter/rangeBefore
    // entropy = -log2(p(x))=-log(p(x))/log(2) = -(log rangeAfter - log rangeBefore) / log(2) = (log rangeBefore / log 2
    // - log rangeAfter / log 2)
    SStat            &s    = inst.data.statistics[stat.type][stat.subClass];
    s.bits += inst.values.values[uiRangeBefore] - inst.values.values[uiRangeAfter];
    s.count++;
    s.sum += val;
  }

#if RExt__DECODER_DEBUG_TOOL_MAX_FRAME_STATS
  static void UpdateMaxStat(CodingStatisticsData *data)
  {
    SStat_max    &max = GetStatisticMax();
    const int64_t es  = CODINGSTATISTICS_ENTROPYSCALE;

    int64_t countTotal = 0;
    int64_t classCounts[STATS__NUM_STATS];
    std::fill_n(classCounts, (size_t)STATS__NUM_STATS, 0);

    int64_t cr = 0;   // CABAC remainder, which is added to "STATS__CABAC_INITIALISATION"

    int64_t totalCABACbits = 0, roundedCABACbits = 0;
    for (int i = STATS__NAL_UNIT_PACKING; i < STATS__NUM_STATS; i++)
    {
      int64_t classCount = 0;

      for (uint32_t c = 0; c < CODING_STATS_NUM_SUBCLASSES; c++)
      {
        totalCABACbits += data->statistics[i][c].bits;
        roundedCABACbits += data->statistics[i][c].bits / es;
        classCount += data->statistics[i][c].count;
      }

      for (uint32_t c = 0; c < CODING_STATS_NUM_SUBCLASSES; c++)
      {
        data->statistics[i][c].classCount = classCount;
      }

      classCounts[i] = classCount;
      countTotal += classCount;
    }
    int64_t remainder = totalCABACbits - roundedCABACbits * es;
    cr                = (remainder + es / 2) / es;

    classCounts[0] = countTotal;

    SStat cabacTotalBits, epTotalBits, cabacTrfTotalBits, epTrfTotalBits;

    cabacTotalBits.classCount    = countTotal;
    epTotalBits.classCount       = countTotal;
    cabacTrfTotalBits.classCount = countTotal;
    epTrfTotalBits.classCount    = countTotal;

    // Calculate the actual bin and bit count
    for (int i = 0; i < STATS__NUM_STATS; i++)
    {
      for (uint32_t c = 0; c < CODING_STATS_NUM_SUBCLASSES; c++)
      {
        SStat &sCABACorig = data->statistics[i][c];
        SStat &sEP        = data->statistics_ep[i][c];

        if (sCABACorig.bits == 0 && sEP.bits == 0)
        {
          continue;
        }

        SStat sCABAC;
        {
          int64_t thisCABACbits = sCABACorig.bits / es;
          if (i == STATS__CABAC_INITIALISATION && sCABACorig.bits != 0)
          {
            thisCABACbits += cr;
#if EPBINCOUNT_FIX
            sCABACorig.count = 0;
#endif
            cr = 0;
          }
          sCABAC.bits       = thisCABACbits;
          sCABAC.count      = sCABACorig.count;
          sCABAC.sum        = sCABACorig.sum;
          sCABAC.classCount = classCounts[i];
        }
#if EPBINCOUNT_FIX
        if (i == STATS__BYTE_ALIGNMENT_BITS || i == STATS__TRAILING_BITS || i == STATS__NAL_UNIT_HEADER_BITS ||
            i == STATS__EMULATION_PREVENTION_3_BYTES)
        {
          sEP.count = 0;
        }
#endif

        if (i != STATS__NAL_UNIT_TOTAL_BODY)
        {
          cabacTotalBits += sCABAC;
          epTotalBits += sEP;

          // For TRF
          if ((i == STATS__CABAC_BITS__SIG_COEFF_MAP_FLAG) || (i == STATS__CABAC_BITS__PAR_FLAG) ||
              (i == STATS__CABAC_BITS__GT1_FLAG) || (i == STATS__CABAC_BITS__GT2_FLAG) ||
              (i == STATS__CABAC_BITS__GT3_FLAG) || (i == STATS__CABAC_BITS__GT4_FLAG) ||
              (i == STATS__CABAC_BITS__ESCAPE_BITS))
          {
            cabacTrfTotalBits += sCABAC;
            epTrfTotalBits += sEP;
          }
        }
      }
    }

    SStat delta_CABAC     = cabacTotalBits;
    SStat delta_EP        = epTotalBits;
    SStat delta_trf_CABAC = cabacTrfTotalBits;
    SStat delta_trf_EP    = epTrfTotalBits;

    delta_CABAC -= max.prev_CABAC_state;
    delta_EP -= max.prev_EP_state;

    delta_trf_CABAC -= max.prev_trf_CABAC_state;
    delta_trf_EP -= max.prev_trf_EP_state;
    int64_t max_frame_bins = EPBIN_WEIGHT_FACTOR * max.max_CABAC_state.count + max.max_EP_state.count;
    int64_t cur_frame_bins = EPBIN_WEIGHT_FACTOR * delta_CABAC.count + delta_EP.count;

    if (cur_frame_bins > max_frame_bins)
    {
      max.max_CABAC_state = delta_CABAC;
      max.max_EP_state    = delta_EP;
      max.trf_CABAC_state = delta_trf_CABAC;
      max.trf_EP_state    = delta_trf_EP;
    }

    max.prev_CABAC_state = cabacTotalBits;
    max.prev_EP_state    = epTotalBits;

    max.prev_trf_CABAC_state = cabacTrfTotalBits;
    max.prev_trf_EP_state    = epTrfTotalBits;
  }
#endif
};
#endif

#endif
