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

/** \file     EncAppCfg.cpp
    \brief    Handle encoder configuration parameters
*/

#include "EncAppCfg.h"

#include <map>
#include <istream>
template<class T1, class T2> static inline std::istream &operator>>(std::istream &in, std::map<T1, T2> &map);

#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <string>
#include <fstream>
#include <limits>

#include "Utilities/program_options_lite.h"
#include "Utilities/VideoIOYuv.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/AlfParameters.h"
#include "CommonLib/Rom.h"
#include "EncoderLib/RateCtrl.h"
#include "EncoderLib/EncCfg.h"

#include "CommonLib/dtrace_next.h"
#include "CommonLib/ProfileTierLevel.h"

#define MACRO_TO_STRING_HELPER(val) #val
#define MACRO_TO_STRING(val)        MACRO_TO_STRING_HELPER(val)

namespace po = df::program_options_lite;

enum ExtendedProfileName   // this is used for determining profile strings, where multiple profiles map to a single
                           // profile idc with various constraint flag combinations
{
  NONE,
  MAIN_10,
  MAIN_10_STILL_PICTURE,
  MAIN_10_444,
  MAIN_10_444_STILL_PICTURE,
  MULTILAYER_MAIN_10,
  MULTILAYER_MAIN_10_STILL_PICTURE,
  MULTILAYER_MAIN_10_444,
  MULTILAYER_MAIN_10_444_STILL_PICTURE,
  MAIN_12,
  MAIN_12_444,
  MAIN_16_444,
  MAIN_12_INTRA,
  MAIN_12_444_INTRA,
  MAIN_16_444_INTRA,
  MAIN_12_STILL_PICTURE,
  MAIN_12_444_STILL_PICTURE,
  MAIN_16_444_STILL_PICTURE,
  AUTO = -1
};

//! \ingroup EncoderApp
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

EncAppCfg::EncAppCfg()
#if EXTENSION_360_VIDEO
  : m_ext360(*this)
#endif
{}

EncAppCfg::~EncAppCfg() {}

std::istringstream &operator>>(std::istringstream &in, GOPEntry &entry)     // input
{
  in >> entry.m_sliceType;
  in >> entry.m_POC;
  in >> entry.m_QPOffset;
  in >> entry.m_QPOffsetModelOffset;
  in >> entry.m_QPOffsetModelScale;
#if W0038_CQP_ADJ
  in >> entry.m_CbQPoffset;
  in >> entry.m_CrQPoffset;
#endif
  in >> entry.m_QPFactor;
  in >> entry.m_tcOffsetDiv2;
  in >> entry.m_betaOffsetDiv2;
  in >> entry.m_CbTcOffsetDiv2;
  in >> entry.m_CbBetaOffsetDiv2;
  in >> entry.m_CrTcOffsetDiv2;
  in >> entry.m_CrBetaOffsetDiv2;
  in >> entry.m_temporalId;
  in >> entry.m_numRefPicsActive0;
  in >> entry.m_numRefPics0;
  for (int i = 0; i < entry.m_numRefPics0; i++)
  {
    in >> entry.m_deltaRefPics0[i];
  }
  in >> entry.m_numRefPicsActive1;
  in >> entry.m_numRefPics1;
  for (int i = 0; i < entry.m_numRefPics1; i++)
  {
    in >> entry.m_deltaRefPics1[i];
  }

  return in;
}

bool confirmPara(bool bflag, const char *message);

static inline ChromaFormat numberToChromaFormat(const int val)
{
  switch (val)
  {
  case 400:
    return ChromaFormat::_400;
    break;
  case 420:
    return ChromaFormat::_420;
    break;
  case 422:
    return ChromaFormat::_422;
    break;
  case 444:
    return ChromaFormat::_444;
    break;
  default:
    return ChromaFormat::UNDEFINED;
  }
}

static const struct MapStrToProfile
{
  const char   *str;
  Profile::Name value;
} strToProfile[] = {
  { "none", Profile::NONE },
  { "main_10", Profile::MAIN_10 },
  { "main_10_444", Profile::MAIN_10_444 },
  { "main_10_still_picture", Profile::MAIN_10_STILL_PICTURE },
  { "main_10_444_still_picture", Profile::MAIN_10_444_STILL_PICTURE },
  { "multilayer_main_10", Profile::MULTILAYER_MAIN_10 },
  { "multilayer_main_10_444", Profile::MULTILAYER_MAIN_10_444 },
  { "multilayer_main_10_still_picture", Profile::MULTILAYER_MAIN_10_STILL_PICTURE },
  { "multilayer_main_10_444_still_picture", Profile::MULTILAYER_MAIN_10_444_STILL_PICTURE },
  { "main_12", Profile::MAIN_12 },
  { "main_12_444", Profile::MAIN_12_444 },
  { "main_16_444", Profile::MAIN_16_444 },
  { "main_12_intra", Profile::MAIN_12_INTRA },
  { "main_12_444_intra", Profile::MAIN_12_444_INTRA },
  { "main_16_444_intra", Profile::MAIN_16_444_INTRA },
  { "main_12_still_picture", Profile::MAIN_12_STILL_PICTURE },
  { "main_12_444_still_picture", Profile::MAIN_12_444_STILL_PICTURE },
  { "main_16_444_still_picture", Profile::MAIN_16_444_STILL_PICTURE },
};

static const struct MapStrToExtendedProfile
{
  const char         *str;
  ExtendedProfileName value;
} strToExtendedProfile[] = {
  { "none", NONE },
  { "main_10", MAIN_10 },
  { "main_10_444", MAIN_10_444 },
  { "main_10_still_picture", MAIN_10_STILL_PICTURE },
  { "main_10_444_still_picture", MAIN_10_444_STILL_PICTURE },
  { "multilayer_main_10", MULTILAYER_MAIN_10 },
  { "multilayer_main_10_444", MULTILAYER_MAIN_10_444 },
  { "multilayer_main_10_still_picture", MULTILAYER_MAIN_10_STILL_PICTURE },
  { "multilayer_main_10_444_still_picture", MULTILAYER_MAIN_10_444_STILL_PICTURE },
  { "main_12", MAIN_12 },
  { "main_12_444", MAIN_12_444 },
  { "main_16_444", MAIN_16_444 },
  { "main_12_intra", MAIN_12_INTRA },
  { "main_12_444_intra", MAIN_12_444_INTRA },
  { "main_16_444_intra", MAIN_16_444_INTRA },
  { "main_12_still_picture", MAIN_12_STILL_PICTURE },
  { "main_12_444_still_picture", MAIN_12_444_STILL_PICTURE },
  { "main_16_444_still_picture", MAIN_16_444_STILL_PICTURE },
  { "auto", AUTO },
};

static const struct MapStrToTier
{
  const char *str;
  Level::Tier value;
} strToTier[] = {
  { "main", Level::MAIN },
  { "high", Level::HIGH },
};

static const struct MapStrToLevel
{
  const char *str;
  Level::Name value;
} strToLevel[] = {
  { "none", Level::NONE },    { "1", Level::LEVEL1 },     { "2", Level::LEVEL2 },     { "2.1", Level::LEVEL2_1 },
  { "3", Level::LEVEL3 },     { "3.1", Level::LEVEL3_1 }, { "4", Level::LEVEL4 },     { "4.1", Level::LEVEL4_1 },
  { "5", Level::LEVEL5 },     { "5.1", Level::LEVEL5_1 }, { "5.2", Level::LEVEL5_2 }, { "6", Level::LEVEL6 },
  { "6.1", Level::LEVEL6_1 }, { "6.2", Level::LEVEL6_2 }, { "6.3", Level::LEVEL6_3 }, { "15.5", Level::LEVEL15_5 },
};

// clang-format off
uint32_t g_uiMaxCpbSize[2][28] =
{
  //            LEVEL1,          LEVEL2,  LEVEL2_1,      LEVEL3,  LEVEL3_1,       LEVEL4,   LEVEL4_1,       LEVEL5,    LEVEL5_1,  LEVEL5_2,     LEVEL6,    LEVEL6_1,  LEVEL6_2   LEVEL6_3
  { 0, 0, 0, 0, 350000, 0, 0, 0, 1500000, 3000000, 0, 0, 6000000, 10000000, 0, 0, 12000000, 20000000, 0, 0,  25000000,  40000000,  60000000, 0,  80000000, 120000000, 240000000,  240000000 },
  { 0, 0, 0, 0,      0, 0, 0, 0,       0,       0, 0, 0,       0,        0, 0, 0, 30000000, 50000000, 0, 0, 100000000, 160000000, 240000000, 0, 240000000, 480000000, 800000000, 1600000000 }
};
// clang-format on

static const struct MapStrToCostMode
{
  const char *str;
  CostMode    value;
} strToCostMode[] = { { "lossy", COST_STANDARD_LOSSY },
                      { "sequence_level_lossless", COST_SEQUENCE_LEVEL_LOSSLESS },
                      { "lossless", COST_LOSSLESS_CODING },
                      { "mixed_lossless_lossy", COST_MIXED_LOSSLESS_LOSSY_CODING } };

static const struct MapStrToScalingListMode
{
  const char     *str;
  ScalingListMode value;
} strToScalingListMode[] = { { "0", SCALING_LIST_OFF },           { "1", SCALING_LIST_DEFAULT },
                             { "2", SCALING_LIST_FILE_READ },     { "off", SCALING_LIST_OFF },
                             { "default", SCALING_LIST_DEFAULT }, { "file", SCALING_LIST_FILE_READ } };

template<typename T, typename P> static std::string enumToString(P map[], uint32_t mapLen, const T val)
{
  for (uint32_t i = 0; i < mapLen; i++)
  {
    if (val == map[i].value)
    {
      return map[i].str;
    }
  }
  return std::string();
}

template<typename T, typename P> static std::istream &readStrToEnum(P map[], uint32_t mapLen, std::istream &in, T &val)
{
  std::string str;
  in >> str;

  for (uint32_t i = 0; i < mapLen; i++)
  {
    if (str == map[i].str)
    {
      val = map[i].value;
      goto found;
    }
  }
  /* not found */
  in.setstate(std::ios::failbit);
found:
  return in;
}

// inline to prevent compiler warnings for "unused static function"

static inline std::istream &operator>>(std::istream &in, ExtendedProfileName &profile)
{
  return readStrToEnum(strToExtendedProfile, sizeof(strToExtendedProfile) / sizeof(*strToExtendedProfile), in, profile);
}

namespace Level
{
static inline std::istream &operator>>(std::istream &in, Tier &tier)
{
  return readStrToEnum(strToTier, sizeof(strToTier) / sizeof(*strToTier), in, tier);
}

static inline std::istream &operator>>(std::istream &in, Name &level)
{
  return readStrToEnum(strToLevel, sizeof(strToLevel) / sizeof(*strToLevel), in, level);
}
}   // namespace Level

static inline std::istream &operator>>(std::istream &in, CostMode &mode)
{
  return readStrToEnum(strToCostMode, sizeof(strToCostMode) / sizeof(*strToCostMode), in, mode);
}

static inline std::istream &operator>>(std::istream &in, ScalingListMode &mode)
{
  return readStrToEnum(strToScalingListMode, sizeof(strToScalingListMode) / sizeof(*strToScalingListMode), in, mode);
}

template<class T> static inline std::istream &operator>>(std::istream &in, SMultiValueInput<T> &values)
{
  return values.readValues(in);
}

template<class T> T SMultiValueInput<T>::readValue(const char *&pStr, bool &bSuccess)
{
  T           val = T();
  std::string s(pStr);
  std::replace(s.begin(), s.end(), ',', ' '); // make comma separated into space separated
  std::istringstream iss(s);
  iss >> val;
  bSuccess = !iss.fail() // check nothing has gone wrong
    && !(val < minValIncl || val > maxValIncl) // check value is within range
    && (int)iss.tellg() != 0 // check we've actually read something
    && (iss.eof() || iss.peek() == ' '); // check next character is a space, or eof
  pStr += (iss.eof() ? s.size() : (std::size_t)iss.tellg());
  return val;
}

template<class T> std::istream &SMultiValueInput<T>::readValues(std::istream &in)
{
  values.clear();
  std::string str;
  while (!in.eof())
  {
    std::string tmp;
    in >> tmp;
    str += " " + tmp;
  }
  if (!str.empty())
  {
    const char *pStr = str.c_str();
    // soak up any whitespace
    for (; isspace(*pStr); pStr++)
      ;

    while (*pStr != 0)
    {
      bool bSuccess = true;
      T    val      = readValue(pStr, bSuccess);
      if (!bSuccess)
      {
        in.setstate(std::ios::failbit);
        break;
      }

      if (maxNumValuesIncl != 0 && values.size() >= maxNumValuesIncl)
      {
        in.setstate(std::ios::failbit);
        break;
      }
      values.push_back(val);
      // soak up any whitespace and up to 1 comma.
      for (; isspace(*pStr); pStr++)
        ;
      if (*pStr == ',')
      {
        pStr++;
      }
      for (; isspace(*pStr); pStr++)
        ;
    }
  }
  if (values.size() < minNumValuesIncl)
  {
    in.setstate(std::ios::failbit);
  }
  return in;
}

template<class T1, class T2> static inline std::istream &operator>>(std::istream &in, std::map<T1, T2> &map)
{
  T1 key;
  T2 value;
  try
  {
    in >> key;
    in >> value;
  }
  catch (...)
  {
    in.setstate(std::ios::failbit);
  }

  map[key] = value;
  return in;
}

static uint32_t getMaxTileColsByLevel(Level::Name level)
{
  switch (level)
  {
  case Level::LEVEL1:
  case Level::LEVEL2:
  case Level::LEVEL2_1:
    return 1;
  case Level::LEVEL3:
    return 2;
  case Level::LEVEL3_1:
    return 3;
  case Level::LEVEL4:
  case Level::LEVEL4_1:
    return 5;
  case Level::LEVEL5:
  case Level::LEVEL5_1:
  case Level::LEVEL5_2:
    return 10;
  case Level::LEVEL6:
  case Level::LEVEL6_1:
  case Level::LEVEL6_2:
    return 20;
  case Level::LEVEL6_3:
    return 30;
  default:
    return MAX_TILE_COLS;
  }
}

static uint32_t getMaxTileRowsByLevel(Level::Name level)
{
  switch (level)
  {
  case Level::LEVEL1:
  case Level::LEVEL2:
  case Level::LEVEL2_1:
    return 1;
  case Level::LEVEL3:
    return 2;
  case Level::LEVEL3_1:
    return 3;
  case Level::LEVEL4:
  case Level::LEVEL4_1:
    return 5;
  case Level::LEVEL5:
  case Level::LEVEL5_1:
  case Level::LEVEL5_2:
    return 11;
  case Level::LEVEL6:
  case Level::LEVEL6_1:
  case Level::LEVEL6_2:
    return 22;
  case Level::LEVEL6_3:
    return 33;
  default:
    return MAX_TILES / MAX_TILE_COLS;
  }
}

static uint32_t getMaxSlicesByLevel(Level::Name level)
{
  switch (level)
  {
  case Level::LEVEL1:
  case Level::LEVEL2:
    return 16;
  case Level::LEVEL2_1:
    return 20;
  case Level::LEVEL3:
    return 30;
  case Level::LEVEL3_1:
    return 40;
  case Level::LEVEL4:
  case Level::LEVEL4_1:
    return 75;
  case Level::LEVEL5:
  case Level::LEVEL5_1:
  case Level::LEVEL5_2:
    return 200;
  case Level::LEVEL6:
  case Level::LEVEL6_1:
  case Level::LEVEL6_2:
    return 600;
  case Level::LEVEL6_3:
    return 1000;
  default:
    return MAX_SLICES;
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/** \param  argc        number of arguments
    \param  argv        array of arguments
    \retval             true when success
 */
#ifdef _MSC_VER
// Disable optimizations to avoid long compile times
#pragma optimize("", off)
#endif

bool EncAppCfg::parseCfg(int argc, char *argv[], EncCfg *encCfg)
{
  int                 tmpChromaFormat                       = 0;
  int                 tmpInputChromaFormat                  = 420;
  int                 tmpConstraintChromaFormat             = 0;
  int                 tmpMaxChromaFormatConstraintIdc       = 3;
  int                 tmpWeightedPredictionMethod           = 0;
  int                 tmpFastInterSearchMode                = 0;
  int                 tmpMotionEstimationSearchMethod       = int(MESearchMethod::DIAMOND);
  int                 tmpDecodedPictureHashSEIMappedType    = 0;
  int                 tmpSubpicDecodedPictureHashMappedType = 0;
  int                 warnUnknowParameter                   = 0;
  uint32_t            lumaLevelToDeltaQPMode                = 0;
  ExtendedProfileName extendedProfile                       = ExtendedProfileName::NONE;
  bool                do_help                               = false;
  bool                sdr                                   = false;
  std::string         inputColourSpaceConvert               = "";
  std::string         inputPathPrefix                       = "";
#if ENABLE_TRACING
  bool        bTracingChannelsList = false;
  std::string sTracingRule         = "";
  std::string sTracingFile         = "";
#endif
#if ENABLE_SIMD_OPT
  std::string ignore = "";
#endif

  // Multi-value input fields:                                // minval, maxval (incl), min_entries, max_entries (incl)
  // [, default values, number of default values]
  SMultiValueInput<uint32_t> cfgTileColumnWidth(0, std::numeric_limits<uint32_t>::max(), 0,
                                                std::numeric_limits<uint32_t>::max());
  SMultiValueInput<uint32_t> cfgTileRowHeight(0, std::numeric_limits<uint32_t>::max(), 0,
                                              std::numeric_limits<uint32_t>::max());
  SMultiValueInput<uint32_t> cfgRectSlicePos(0, std::numeric_limits<uint32_t>::max(), 0,
                                             std::numeric_limits<uint32_t>::max());
  SMultiValueInput<uint32_t> cfgRasterSliceSize(0, std::numeric_limits<uint32_t>::max(), 0,
                                                std::numeric_limits<uint32_t>::max());

  SMultiValueInput<double> cfg_adIntraLambdaModifier(
    0, std::numeric_limits<double>::max(), 0,
    MAX_TLAYER);   ///< Lambda modifier for Intra pictures, one for each temporal layer. If size>temporalLayer, then use
                   ///< [temporalLayer], else if size>0, use [size()-1], else use m_adLambdaModifier.
  SMultiValueInput<uint16_t> cfgSliceLosslessArray(0, std::numeric_limits<uint16_t>::max(), 0, MAX_SLICES);
#if SHARP_LUMA_DELTA_QP
  const int             defaultLumaLevelTodQp_QpChangePoints[]   = { -3, -2, -1, 0, 1, 2, 3, 4, 5, 6 };
  const int             defaultLumaLevelTodQp_LumaChangePoints[] = { 0, 301, 367, 434, 501, 567, 634, 701, 767, 834 };
  SMultiValueInput<int> cfg_lumaLeveltoDQPMappingQP(-MAX_QP, MAX_QP, 0, LUMA_LEVEL_TO_DQP_LUT_MAXSIZE,
                                                    defaultLumaLevelTodQp_QpChangePoints,
                                                    sizeof(defaultLumaLevelTodQp_QpChangePoints) / sizeof(int));
  SMultiValueInput<int> cfg_lumaLeveltoDQPMappingLuma(
    0, std::numeric_limits<int>::max(), 0, LUMA_LEVEL_TO_DQP_LUT_MAXSIZE, defaultLumaLevelTodQp_LumaChangePoints,
    sizeof(defaultLumaLevelTodQp_LumaChangePoints) / sizeof(int));
#endif
  const int qpInVals[]  = { 25, 33, 43 };   // qpInVal values used to derive the chroma QP mapping table used in VTM-5.0
  const int qpOutVals[] = { 25, 32,
                            37 };   // qpOutVal values used to derive the chroma QP mapping table used in VTM-5.0
  SMultiValueInput<int> cfg_qpInValCb(MIN_QP_VALUE_FOR_16_BIT, MAX_QP, 0, MAX_NUM_QP_VALUES, qpInVals,
                                      sizeof(qpInVals) / sizeof(int));
  SMultiValueInput<int> cfg_qpOutValCb(MIN_QP_VALUE_FOR_16_BIT, MAX_QP, 0, MAX_NUM_QP_VALUES, qpOutVals,
                                       sizeof(qpOutVals) / sizeof(int));
  const int             zeroVector[] = { 0 };
  SMultiValueInput<int> cfg_qpInValCr(MIN_QP_VALUE_FOR_16_BIT, MAX_QP, 0, MAX_NUM_QP_VALUES, zeroVector, 1);
  SMultiValueInput<int> cfg_qpOutValCr(MIN_QP_VALUE_FOR_16_BIT, MAX_QP, 0, MAX_NUM_QP_VALUES, zeroVector, 1);
  SMultiValueInput<int> cfg_qpInValCbCr(MIN_QP_VALUE_FOR_16_BIT, MAX_QP, 0, MAX_NUM_QP_VALUES, zeroVector, 1);
  SMultiValueInput<int> cfg_qpOutValCbCr(MIN_QP_VALUE_FOR_16_BIT, MAX_QP, 0, MAX_NUM_QP_VALUES, zeroVector, 1);
  const int             cQpOffsets[] = { 6 };
  SMultiValueInput<int> cfg_cbQpOffsetList(-12, 12, 0, 6, cQpOffsets, 0);
  SMultiValueInput<int> cfg_crQpOffsetList(-12, 12, 0, 6, cQpOffsets, 0);
  SMultiValueInput<int> cfg_cbCrQpOffsetList(-12, 12, 0, 6, cQpOffsets, 0);

  const int             defaultPrimaryCodes[6]   = { 0, 50000, 0, 0, 50000, 0 };
  const int             defaultWhitePointCode[2] = { 16667, 16667 };
  SMultiValueInput<int> cfg_DisplayPrimariesCode(0, 50000, 6, 6, defaultPrimaryCodes,
                                                 sizeof(defaultPrimaryCodes) / sizeof(int));
  SMultiValueInput<int> cfg_DisplayWhitePointCode(0, 50000, 2, 2, defaultWhitePointCode,
                                                  sizeof(defaultWhitePointCode) / sizeof(int));

  SMultiValueInput<Pel>      cfg_SEICTILut0(0, ((1 << (2 + 12 - 1)) - 1), 0, MAX_CTI_LUT_SIZE + 1);
  SMultiValueInput<Pel>      cfg_SEICTILut1(0, ((1 << (2 + 12 - 1)) - 1), 0, MAX_CTI_LUT_SIZE + 1);
  SMultiValueInput<Pel>      cfg_SEICTILut2(0, ((1 << (2 + 12 - 1)) - 1), 0, MAX_CTI_LUT_SIZE + 1);
  SMultiValueInput<int>      cfg_omniViewportSEIAzimuthCentre(-11796480, 11796479, 0, 15);
  SMultiValueInput<int>      cfg_omniViewportSEIElevationCentre(-5898240, 5898240, 0, 15);
  SMultiValueInput<int>      cfg_omniViewportSEITiltCentre(-11796480, 11796479, 0, 15);
  SMultiValueInput<uint32_t> cfg_omniViewportSEIHorRange(1, 23592960, 0, 15);
  SMultiValueInput<uint32_t> cfg_omniViewportSEIVerRange(1, 11796480, 0, 15);
  SMultiValueInput<uint32_t> cfg_rwpSEIRwpTransformType(0, 7, 0, std::numeric_limits<uint8_t>::max());
  SMultiValueInput<bool>     cfg_rwpSEIRwpGuardBandFlag(0, 1, 0, std::numeric_limits<uint8_t>::max());
  SMultiValueInput<uint32_t> cfg_rwpSEIProjRegionWidth(0, std::numeric_limits<uint32_t>::max(), 0,
                                                       std::numeric_limits<uint8_t>::max());
  SMultiValueInput<uint32_t> cfg_rwpSEIProjRegionHeight(0, std::numeric_limits<uint32_t>::max(), 0,
                                                        std::numeric_limits<uint8_t>::max());
  SMultiValueInput<uint32_t> cfg_rwpSEIRwpSEIProjRegionTop(0, std::numeric_limits<uint32_t>::max(), 0,
                                                           std::numeric_limits<uint8_t>::max());
  SMultiValueInput<uint32_t> cfg_rwpSEIProjRegionLeft(0, std::numeric_limits<uint32_t>::max(), 0,
                                                      std::numeric_limits<uint8_t>::max());
  SMultiValueInput<uint32_t> cfg_rwpSEIPackedRegionWidth(0, std::numeric_limits<uint16_t>::max(), 0,
                                                         std::numeric_limits<uint8_t>::max());
  SMultiValueInput<uint32_t> cfg_rwpSEIPackedRegionHeight(0, std::numeric_limits<uint16_t>::max(), 0,
                                                          std::numeric_limits<uint8_t>::max());
  SMultiValueInput<uint32_t> cfg_rwpSEIPackedRegionTop(0, std::numeric_limits<uint16_t>::max(), 0,
                                                       std::numeric_limits<uint8_t>::max());
  SMultiValueInput<uint32_t> cfg_rwpSEIPackedRegionLeft(0, std::numeric_limits<uint16_t>::max(), 0,
                                                        std::numeric_limits<uint8_t>::max());
  SMultiValueInput<uint32_t> cfg_rwpSEIRwpLeftGuardBandWidth(0, std::numeric_limits<uint8_t>::max(), 0,
                                                             std::numeric_limits<uint8_t>::max());
  SMultiValueInput<uint32_t> cfg_rwpSEIRwpRightGuardBandWidth(0, std::numeric_limits<uint8_t>::max(), 0,
                                                              std::numeric_limits<uint8_t>::max());
  SMultiValueInput<uint32_t> cfg_rwpSEIRwpTopGuardBandHeight(0, std::numeric_limits<uint8_t>::max(), 0,
                                                             std::numeric_limits<uint8_t>::max());
  SMultiValueInput<uint32_t> cfg_rwpSEIRwpBottomGuardBandHeight(0, std::numeric_limits<uint8_t>::max(), 0,
                                                                std::numeric_limits<uint8_t>::max());
  SMultiValueInput<bool>     cfg_rwpSEIRwpGuardBandNotUsedForPredFlag(0, 1, 0, std::numeric_limits<uint8_t>::max());
  SMultiValueInput<uint32_t> cfg_rwpSEIRwpGuardBandType(0, 7, 0, 4 * std::numeric_limits<uint8_t>::max());
  SMultiValueInput<uint32_t> cfg_gcmpSEIFaceIndex(0, 5, 5, 6);
  SMultiValueInput<uint32_t> cfg_gcmpSEIFaceRotation(0, 3, 5, 6);
  SMultiValueInput<double>   cfg_gcmpSEIFunctionCoeffU(0.0, 1.0, 5, 6);
  SMultiValueInput<uint32_t> cfg_gcmpSEIFunctionUAffectedByVFlag(0, 1, 5, 6);
  SMultiValueInput<double>   cfg_gcmpSEIFunctionCoeffV(0.0, 1.0, 5, 6);
  SMultiValueInput<uint32_t> cfg_gcmpSEIFunctionVAffectedByUFlag(0, 1, 5, 6);
  SMultiValueInput<uint32_t> cfg_sdiSEILayerId(0, 63, 0, 63);
  SMultiValueInput<uint32_t> cfg_sdiSEIViewIdVal(0, 63, 0, std::numeric_limits<uint32_t>::max());
  SMultiValueInput<uint32_t> cfg_sdiSEIAuxId(0, 255, 0, 63);
  SMultiValueInput<uint32_t> cfg_sdiSEINumAssociatedPrimaryLayersMinus1(0, 63, 0, 63);
  SMultiValueInput<bool>     cfg_maiSEISignFocalLengthX(0, 1, 0, std::numeric_limits<uint32_t>::max());
  SMultiValueInput<uint32_t> cfg_maiSEIExponentFocalLengthX(0, 63, 0, std::numeric_limits<uint32_t>::max());
  SMultiValueInput<uint32_t> cfg_maiSEIMantissaFocalLengthX(0, std::numeric_limits<uint32_t>::max(), 0,
                                                            std::numeric_limits<uint32_t>::max());
  SMultiValueInput<bool>     cfg_maiSEISignFocalLengthY(0, 1, 0, std::numeric_limits<uint32_t>::max());
  SMultiValueInput<uint32_t> cfg_maiSEIExponentFocalLengthY(0, 63, 0, std::numeric_limits<uint32_t>::max());
  SMultiValueInput<uint32_t> cfg_maiSEIMantissaFocalLengthY(0, std::numeric_limits<uint32_t>::max(), 0,
                                                            std::numeric_limits<uint32_t>::max());
  SMultiValueInput<bool>     cfg_maiSEISignPrincipalPointX(0, 1, 0, std::numeric_limits<uint32_t>::max());
  SMultiValueInput<uint32_t> cfg_maiSEIExponentPrincipalPointX(0, 63, 0, std::numeric_limits<uint32_t>::max());
  SMultiValueInput<uint32_t> cfg_maiSEIMantissaPrincipalPointX(0, std::numeric_limits<uint32_t>::max(), 0,
                                                               std::numeric_limits<uint32_t>::max());
  SMultiValueInput<bool>     cfg_maiSEISignPrincipalPointY(0, 1, 0, std::numeric_limits<uint32_t>::max());
  SMultiValueInput<uint32_t> cfg_maiSEIExponentPrincipalPointY(0, 63, 0, std::numeric_limits<uint32_t>::max());
  SMultiValueInput<uint32_t> cfg_maiSEIMantissaPrincipalPointY(0, std::numeric_limits<uint32_t>::max(), 0,
                                                               std::numeric_limits<uint32_t>::max());
  SMultiValueInput<bool>     cfg_maiSEISignSkewFactor(0, 1, 0, std::numeric_limits<uint32_t>::max());
  SMultiValueInput<uint32_t> cfg_maiSEIExponentSkewFactor(0, 63, 0, std::numeric_limits<uint32_t>::max());
  SMultiValueInput<uint32_t> cfg_maiSEIMantissaSkewFactor(0, std::numeric_limits<uint32_t>::max(), 0,
                                                          std::numeric_limits<uint32_t>::max());
  SMultiValueInput<uint32_t> cfg_mvpSEIViewPosition(0, 63, 0, std::numeric_limits<uint32_t>::max());
  SMultiValueInput<uint32_t> cfg_driSEINonlinearModel(0, 31, 0, std::numeric_limits<uint32_t>::max());

  const int             defaultLadfQpOffset[3]           = { 1, 0, 1 };
  const int             defaultLadfIntervalLowerBound[2] = { 350, 833 };
  SMultiValueInput<int> cfg_ladfQpOffset(-MAX_QP, MAX_QP, 2, MAX_LADF_INTERVALS, defaultLadfQpOffset, 3);
  SMultiValueInput<int> cfg_ladfIntervalLowerBound(0, std::numeric_limits<int>::max(), 1, MAX_LADF_INTERVALS - 1,
                                                   defaultLadfIntervalLowerBound, 2);

  const int                  defaultRprSwitchingResolutionOrderList[12] = { 1, 0, 2, 0, 3, 0, 1, 0, 2, 0, 3, 0 };
  const int                  defaultRprSwitchingQPOffsetOrderList[12]   = { -2, 0, -4, 0, -6, 0, -2, 0, -4, 0, -6, 0 };
  SMultiValueInput<int>      cfg_rprSwitchingResolutionOrderList(0, 3, 0, MAX_RPR_SWITCHING_ORDER_LIST_SIZE,
                                                                 defaultRprSwitchingResolutionOrderList, 12);
  SMultiValueInput<int>      cfg_rprSwitchingQPOffsetOrderList(-MAX_QP, MAX_QP, 0, MAX_RPR_SWITCHING_ORDER_LIST_SIZE,
                                                               defaultRprSwitchingQPOffsetOrderList, 12);
  SMultiValueInput<uint32_t> cfg_SubProfile(0, std::numeric_limits<uint8_t>::max(), 0,
                                            std::numeric_limits<uint8_t>::max());
  SMultiValueInput<uint32_t> cfg_subPicCtuTopLeftX(0, std::numeric_limits<uint32_t>::max(), 0, MAX_NUM_SUB_PICS);
  SMultiValueInput<uint32_t> cfg_subPicCtuTopLeftY(0, std::numeric_limits<uint32_t>::max(), 0, MAX_NUM_SUB_PICS);
  SMultiValueInput<uint32_t> cfg_subPicWidth(1, std::numeric_limits<uint32_t>::max(), 0, MAX_NUM_SUB_PICS);
  SMultiValueInput<uint32_t> cfg_subPicHeight(1, std::numeric_limits<uint32_t>::max(), 0, MAX_NUM_SUB_PICS);
  SMultiValueInput<bool>     cfg_subPicTreatedAsPicFlag(0, 1, 0, MAX_NUM_SUB_PICS);
  SMultiValueInput<bool>     cfg_loopFilterAcrossSubpicEnabledFlag(0, 1, 0, MAX_NUM_SUB_PICS);
  SMultiValueInput<uint32_t> cfg_subPicId(0, std::numeric_limits<uint16_t>::max(), 0, MAX_NUM_SUB_PICS);

  SMultiValueInput<int> cfg_sliFractions(0, 255, 0, std::numeric_limits<int>::max());
  SMultiValueInput<int> cfg_sliNonSubpicLayersFractions(0, 255, 0, std::numeric_limits<int>::max());

  SMultiValueInput<Level::Name> cfg_sliRefLevels(Level::NONE, Level::LEVEL15_5, 0, 8 * MAX_VPS_SUBLAYERS);

  SMultiValueInput<uint16_t> cfg_poSEIPayloadType(0, 65535, 0, 256 * 2);
  SMultiValueInput<uint16_t> cfg_poSEIProcessingOrder(0, 65535, 0, 65536);

  SMultiValueInput<uint16_t> cfg_poSEINumofPrefixByte(0, 255, 0, 256);
  SMultiValueInput<uint16_t> cfg_poSEIPrefixByte(0, 255, 0, 256);

  SMultiValueInput<int32_t> cfg_postFilterHintSEIValues(INT32_MIN + 1, INT32_MAX, 1 * 1 * 1, 15 * 15 * 3);

  SMultiValueInput<uint32_t> cfg_FgcSEIIntensityIntervalLowerBoundComp0(0, 255, 0, 256);
  SMultiValueInput<uint32_t> cfg_FgcSEIIntensityIntervalLowerBoundComp1(0, 255, 0, 256);
  SMultiValueInput<uint32_t> cfg_FgcSEIIntensityIntervalLowerBoundComp2(0, 255, 0, 256);
  SMultiValueInput<uint32_t> cfg_FgcSEIIntensityIntervalUpperBoundComp0(0, 255, 0, 256);
  SMultiValueInput<uint32_t> cfg_FgcSEIIntensityIntervalUpperBoundComp1(0, 255, 0, 256);
  SMultiValueInput<uint32_t> cfg_FgcSEIIntensityIntervalUpperBoundComp2(0, 255, 0, 256);
  SMultiValueInput<uint32_t> cfg_FgcSEICompModelValueComp0(0, 65535, 0, 256 * 6);
  SMultiValueInput<uint32_t> cfg_FgcSEICompModelValueComp1(0, 65535, 0, 256 * 6);
  SMultiValueInput<uint32_t> cfg_FgcSEICompModelValueComp2(0, 65535, 0, 256 * 6);
  SMultiValueInput<unsigned> cfg_siiSEIInputNumUnitsInSI(0, std::numeric_limits<uint32_t>::max(), 0, 7);

  std::vector<SMultiValueInput<uint32_t>> cfg_nnPostFilterSEICharacteristicsInterpolatedPicturesList;
  std::vector<SMultiValueInput<bool>>     cfg_nnPostFilterSEICharacteristicsInputPicOutputFlagList;
  for (int i = 0; i < MAX_NUM_NN_POST_FILTERS; i++)
  {
    cfg_nnPostFilterSEICharacteristicsInterpolatedPicturesList.push_back(
      SMultiValueInput<uint32_t>(0, std::numeric_limits<uint32_t>::max(), 1, 0));
    cfg_nnPostFilterSEICharacteristicsInputPicOutputFlagList.push_back(SMultiValueInput<bool>(0, 1, 1, 0));
  }

  // clang-format off
  po::Options opts;

  opts.addOptions()

  ("help",                                            do_help,                                          "this help text")
  ("c",                                               po::parseConfigFile,                              "configuration file name")
  ("WarnUnknowParameter,w",                           warnUnknowParameter,                              "warn for unknown configuration parameters instead of failing")
  ("isSDR",                                           sdr,                                              "compatibility")
#if ENABLE_SIMD_OPT
  ("SIMD",                                            ignore,                                           "SIMD extension to use (SCALAR, SSE41, SSE42, AVX, AVX2, AVX512), default: the highest supported extension\n")
#endif
  // File, I/O and source parameters
  ("InputFile,i",                                     encCfg->m_inputFileName,                          "Original YUV input file name")
  ("InputPathPrefix,-ipp",                            inputPathPrefix,                                  "pathname to prepend to input filename")
  ("BitstreamFile,b",                                 encCfg->m_bitstreamFileName,                      "Bitstream output file name")
  ("ReconFile,o",                                     encCfg->m_reconFileName,                          "Reconstructed YUV output file name")
  ("TimdUsageStats",                                  encCfg->m_timdUsageStats,                         "Write final post-RDO TIMD/TIMDSAD/TIMD-Merge usage statistics")
  ("TimdUsageStatsRoot",                              encCfg->m_timdUsageStatsRoot,                     "Root directory for parallel-safe TIMD usage statistic shards")
#if JVET_Z0120_SII_SEI_PROCESSING
  ("SEIShutterIntervalPreFilename,-sii",              encCfg->m_seiCfg.m_shutterIntervalPreFileName,    "File name of Pre-Filtering video. If empty, not output video\n")
#endif
  ("SourceWidth,-wdt",                                encCfg->m_sourceWidth,                            "Source picture width")
  ("SourceHeight,-hgt",                               encCfg->m_sourceHeight,                           "Source picture height")
  ("SourceScalingRatioHor",                           encCfg->m_sourceScalingRatioHor,                  "Source picture  horizontal scaling ratio")
  ("SourceScalingRatioVer",                           encCfg->m_sourceScalingRatioVer,                  "Source picture vertical scaling ratio")
  ("InputBitDepth",                                   encCfg->m_inputBitDepth[ChannelType::LUMA],       "Bit-depth of input file")
  ("OutputBitDepth",                                  encCfg->m_outputBitDepth[ChannelType::LUMA],      "Bit-depth of output file (default:InternalBitDepth)")
  ("MSBExtendedBitDepth",                             encCfg->m_msbExtendedBitDepth[ChannelType::LUMA], "bit depth of luma component after addition of MSBs of value 0 (used for synthesising High Dynamic Range source material). (default:InputBitDepth)")
  ("InternalBitDepth",                                encCfg->m_internalBitDepth[ChannelType::LUMA],    "Bit-depth the codec operates at. (default: MSBExtendedBitDepth). If different to MSBExtendedBitDepth, source data will be converted")
  ("InputBitDepthC",                                  encCfg->m_inputBitDepth[ChannelType::CHROMA],     "As per InputBitDepth but for chroma component. (default:InputBitDepth)")
  ("OutputBitDepthC",                                 encCfg->m_outputBitDepth[ChannelType::CHROMA],    "As per OutputBitDepth but for chroma component. (default: use luma output bit-depth)")
  ("MSBExtendedBitDepthC",                            encCfg->m_msbExtendedBitDepth[ChannelType::CHROMA], "As per MSBExtendedBitDepth but for chroma component. (default:MSBExtendedBitDepth)")
  ("ExtendedPrecision",                               encCfg->m_extendedPrecisionProcessingFlag,        "Increased internal accuracies to support high bit depths (not valid in V1 profiles)")
  ("TSRCRicePresent",                                 encCfg->m_tsrcRicePresentFlag,                    "Indicate that TSRC Rice information is present in slice header (not valid in V1 profiles)")
  ("ReverseLastSigCoeff",                             encCfg->m_reverseLastSigCoeffEnabledFlag,         "enable reverse last significant coefficient postion in RRC (not valid in V1 profiles)")
  ("HighPrecisionPredictionWeighting",                encCfg->m_highPrecisionOffsetsEnabledFlag,        "Use high precision option for weighted prediction (not valid in V1 profiles)")
  ("InputColourSpaceConvert",                         inputColourSpaceConvert,                          "Colour space conversion to apply to input video. Permitted values are (empty string=UNCHANGED) " + getListOfColourSpaceConverts(true))
  ("SNRInternalColourSpace",                          encCfg->m_snrInternalColourSpace,                 "If true, then no colour space conversion is applied prior to SNR, otherwise inverse of input is applied.")
  ("OutputInternalColourSpace",                       encCfg->m_outputInternalColourSpace,              "If true, then no colour space conversion is applied for reconstructed video, otherwise inverse of input is applied.")
  ("InputChromaFormat",                               tmpInputChromaFormat,                             "InputChromaFormatIDC")
  ("MSEBasedSequencePSNR",                            encCfg->m_printMSEBasedSequencePSNR,              "0 (default) emit sequence PSNR only as a linear average of the frame PSNRs, 1 = also emit a sequence PSNR based on an average of the frame MSEs")
  ("PrintHexPSNR",                                    encCfg->m_printHexPsnr,                           "0 (default) don't emit hexadecimal PSNR for each frame, 1 = also emit hexadecimal PSNR values")
  ("PrintFrameMSE",                                   encCfg->m_printFrameMSE,                          "0 (default) emit only bit count and PSNRs for each frame, 1 = also emit MSE values")
  ("PrintSequenceMSE",                                encCfg->m_printSequenceMSE,                       "0 (default) emit only bit rate and PSNRs for the whole sequence, 1 = also emit MSE values")
  ("PrintMSSSIM",                                     encCfg->m_printMSSSIM,                            "0 (default) do not print MS-SSIM scores, 1 = print MS-SSIM scores for each frame and for the whole sequence")
  ("PrintWPSNR",                                      encCfg->m_printWPSNR,                             "0 (default) do not print HDR-PQ based wPSNR, 1 = print HDR-PQ based wPSNR")
  ("CabacZeroWordPaddingEnabled",                     encCfg->m_cabacZeroWordPaddingEnabled,            "0 do not add conforming cabac-zero-words to bit streams, 1 (default) = add cabac-zero-words as required")
  ("ChromaFormatIDC,-cf",                             tmpChromaFormat,                                  "ChromaFormatIDC (400|420|422|444 or set 0 (default) for same as InputChromaFormat)")
  ("ConformanceWindowMode",                           encCfg->m_conformanceWindowMode,                  "Window conformance mode (0: no window, 1:automatic padding (default), 2:padding parameters specified, 3:conformance window parameters specified")
  ("HorizontalPadding,-pdx",                          encCfg->m_sourcePadding[0],                       "Horizontal source padding for conformance window mode 2")
  ("VerticalPadding,-pdy",                            encCfg->m_sourcePadding[1],                       "Vertical source padding for conformance window mode 2")
  ("ConfWinLeft",                                     encCfg->m_confWinLeft,                            "Left offset for window conformance mode 3")
  ("ConfWinRight",                                    encCfg->m_confWinRight,                           "Right offset for window conformance mode 3")
  ("ConfWinTop",                                      encCfg->m_confWinTop,                             "Top offset for window conformance mode 3")
  ("ConfWinBottom",                                   encCfg->m_confWinBottom,                          "Bottom offset for window conformance mode 3")
  ("AccessUnitDelimiter",                             encCfg->m_AccessUnitDelimiter,                    "Enable Access Unit Delimiter NALUs")
  ("EnablePictureHeaderInSliceHeader",                encCfg->m_enablePictureHeaderInSliceHeader,       "Enable Picture Header in Slice Header")
  ("FrameRate,-fr",                                   encCfg->m_frameRate,                              "Frame rate")
  ("FrameSkip,-fs",                                   encCfg->m_frameSkip,                              "Number of frames to skip at start of input YUV")
  ("TemporalSubsampleRatio,-ts",                      encCfg->m_temporalSubsampleRatio,                 "Temporal sub-sample ratio when reading input YUV")
  ("FramesToBeEncoded,f",                             encCfg->m_framesToBeEncoded,                      "Number of frames to be encoded (default=all)")
  ("ClipInputVideoToRec709Range",                     encCfg->m_clipInputVideoToRec709Range,            "If true then clip input video to the Rec. 709 Range on loading when InternalBitDepth is less than MSBExtendedBitDepth")
  ("ClipOutputVideoToRec709Range",                    encCfg->m_clipOutputVideoToRec709Range,           "If true then clip output video to the Rec. 709 Range on saving when OutputBitDepth is less than InternalBitDepth")
  ("PYUV",                                            encCfg->m_packedYUVMode,                          "If true then output 10-bit and 12-bit YUV data as 5-byte and 3-byte (respectively) packed YUV data. Ignored for interlaced output.")
  ("SummaryOutFilename",                              encCfg->m_summaryOutFilename,                     "Filename to use for producing summary output file. If empty, do not produce a file.")
  ("SummaryPicFilenameBase",                          encCfg->m_summaryPicFilenameBase,                 "Base filename to use for producing summary picture output files. The actual filenames used will have I.txt, P.txt and B.txt appended. If empty, do not produce a file.")
  ("SummaryVerboseness",                              encCfg->m_summaryVerboseness,                     "Specifies the level of the verboseness of the text output")
  ("Verbosity,v",                                     encCfg->m_verbosity,                              "Specifies the level of the verboseness")
#if JVET_O0756_CONFIG_HDRMETRICS || JVET_O0756_CALCULATE_HDRMETRICS
  ("WhitePointDeltaE1",                               encCfg->m_whitePointDeltaE[0],                    "1st reference white point value")
  ("WhitePointDeltaE2",                               encCfg->m_whitePointDeltaE[1],                    "2nd reference white point value")
  ("WhitePointDeltaE3",                               encCfg->m_whitePointDeltaE[2],                    "3rd reference white point value")
  ("MaxSampleValue",                                  encCfg->m_maxSampleValue,                         "Maximum sample value for floats")
  ("InputSampleRange",                                encCfg->m_sampleRange,                            "Sample Range")
  ("InputColorPrimaries",                             encCfg->m_colorPrimaries,                         "Input Color Primaries")
  ("EnableTFunctionLUT",                              encCfg->m_enableTFunctionLUT,                     "Input Color Primaries")
  ("ChromaLocation",                                  encCfg->m_chromaLocation[0],                      "Location of Chroma Samples")
  ("ChromaUpsampleFilter",                            encCfg->m_chromaUPFilter,                         "420 to 444 conversion filters")
  ("CropOffsetLeft",                                  encCfg->m_cropOffsetLeft,                         "Crop Offset Left position")
  ("CropOffsetTop",                                   encCfg->m_cropOffsetTop,                          "Crop Offset Top position")
  ("CropOffsetRight",                                 encCfg->m_cropOffsetRight,                        "Crop Offset Right position")
  ("CropOffsetBottom",                                encCfg->m_cropOffsetBottom,                       "Crop Offset Bottom position")
  ("CalculateHdrMetrics",                             encCfg->m_calculateHdrMetrics,                    "Enable HDR metric calculation")
#endif

  ;opts.addOptions()

  //Field coding parameters
  ("FieldCoding",                                     encCfg->m_fieldSeqFlag,                           "Signals if it's a field based coding")
  ("TopFieldFirst, Tff",                              encCfg->m_isTopFieldFirst,                        "In case of field based coding, signals whether if it's a top field first or not")
  ("EfficientFieldIRAPEnabled",                       encCfg->m_efficientFieldIRAPEnabled,              "Enable to code fields in a specific, potentially more efficient, order.")
  ("HarmonizeGopFirstFieldCoupleEnabled",             encCfg->m_harmonizeGopFirstFieldCoupleEnabled,    "Enables harmonization of Gop first field couple")
  // Profile and level
  ("Profile",                                         extendedProfile,                                  "Profile name to use for encoding. Use [multilayer_]main_10[_444][_still_picture], auto, or none")
  ("Level",                                           encCfg->m_level,                                  "Level limit to be used, eg 5.1, or none")
  ("Tier",                                            encCfg->m_tier,                                   "Tier to use for interpretation of --Level (main or high only)")
  ("FrameOnlyConstraintFlag",                         encCfg->m_frameOnlyConstraintFlag,                "Bitstream contains only frames")
  ("MultiLayerEnabledFlag",                           encCfg->m_multiLayerEnabledFlag,                  "Bitstream might contain more than one layer")
  ("SubProfile",                                      cfg_SubProfile,                                   "Sub-profile idc")
  ("EnableDecodingCapabilityInformation",             encCfg->m_DCIEnabled,                             "Enables writing of Decoding Capability Information")
  ("MaxBitDepthConstraint",                           encCfg->m_bitDepthConstraint,                     "Bit depth to use for profile-constraint for RExt profiles. 0=automatically choose based upon other parameters")
  ("MaxChromaFormatConstraint",                       tmpConstraintChromaFormat,                        "Chroma-format to use for the profile-constraint for RExt profiles. 0=automatically choose based upon other parameters")
  ("GciPresentFlag",                                  encCfg->m_gciPresentFlag,                         "GCI field present")
  ("IntraOnlyConstraintFlag",                         encCfg->m_intraOnlyConstraintFlag,                "Value of intra_only_constraint_flag")
  ("AllLayersIndependentConstraintFlag",              encCfg->m_allLayersIndependentConstraintFlag,     "Indicate that all layers are independent")
  ("OnePictureOnlyConstraintFlag",                    encCfg->m_onePictureOnlyConstraintFlag,           "Value of general_intra_constraint_flag. Can only be used for single frame encodings. Will be set to true for still picture profiles")
  ("MaxBitDepthConstraintIdc",                        encCfg->m_maxBitDepthConstraintIdc,               "Indicate that sps_bitdepth_minus8 plus 8 shall be in the range of 0 to m_maxBitDepthConstraintIdc")
  ("MaxChromaFormatConstraintIdc",                    tmpMaxChromaFormatConstraintIdc,                  "Indicate that sps_chroma_format_idc shall be in the range of 0 to m_maxChromaFormatConstraintIdc")
  ("NoTrailConstraintFlag",                           encCfg->m_noTrailConstraintFlag,                  "Indicate that TRAIL is deactivated")
  ("NoStsaConstraintFlag",                            encCfg->m_noStsaConstraintFlag,                   "Indicate that STSA is deactivated")
  ("NoRaslConstraintFlag",                            encCfg->m_noRaslConstraintFlag,                   "Indicate that RSAL is deactivated")
  ("NoRadlConstraintFlag",                            encCfg->m_noRadlConstraintFlag,                   "Indicate that RADL is deactivated")
  ("NoIdrConstraintFlag",                             encCfg->m_noIdrConstraintFlag,                    "Indicate that IDR is deactivated")
  ("NoCraConstraintFlag",                             encCfg->m_noCraConstraintFlag,                    "Indicate that CRA is deactivated")
  ("NoGdrConstraintFlag",                             encCfg->m_noGdrConstraintFlag,                    "Indicate that GDR is deactivated")
  ("NoApsConstraintFlag",                             encCfg->m_noApsConstraintFlag,                    "Indicate that APS is deactivated")
  ("OneTilePerPicConstraintFlag",                     encCfg->m_oneTilePerPicConstraintFlag,            "Indicate that each picture shall contain only one tile")
  ("PicHeaderInSliceHeaderConstraintFlag",            encCfg->m_picHeaderInSliceHeaderConstraintFlag,   "Indicate that picture header is present in slice header")
  ("OneSlicePerPicConstraintFlag",                    encCfg->m_oneSlicePerPicConstraintFlag,           "Indicate that each picture shall contain only one slice")
  ("NoIdrRplConstraintFlag",                          encCfg->m_noIdrRplConstraintFlag,                 "Indicate that RPL is not present in SH of IDR slices")
  ("NoRectSliceConstraintFlag",                       encCfg->m_noRectSliceConstraintFlag,              "Indicate that rectagular slice is deactivated")
  ("OneSlicePerSubpicConstraintFlag",                 encCfg->m_oneSlicePerSubpicConstraintFlag,        "Indicate that each subpicture shall contain only one slice")
  ("NoSubpicInfoConstraintFlag",                      encCfg->m_noSubpicInfoConstraintFlag,             "Indicate that subpicture information is not present")
  ("MaxLog2CtuSizeConstraintIdc",                     encCfg->m_maxLog2CtuSizeConstraintIdc,            "Indicate that Log2CtuSize shall be in the range of 0 to m_maxLog2CtuSizeConstraintIdc")
  ("NoPartitionConstraintsOverrideConstraintFlag",    encCfg->m_noPartitionConstraintsOverrideConstraintFlag, "Indicate that Partition Override is deactivated")
  ("MttConstraintFlag",                               encCfg->m_noMttConstraintFlag,                    "Indicate that Mtt is deactivated")
  ("NoQtbttDualTreeIntraConstraintFlag",              encCfg->m_noQtbttDualTreeIntraConstraintFlag,     "Indicate that Qtbtt DualTree Intra is deactivated")
  ("NoPaletteConstraintFlag",                         encCfg->m_noPaletteConstraintFlag,                "Indicate that PLT is deactivated")
  ("NoIbcConstraintFlag",                             encCfg->m_noIbcConstraintFlag,                    "Indicate that IBC is deactivated")
  ("NoMrlConstraintFlag",                             encCfg->m_noMrlConstraintFlag,                    "Indicate that MRL is deactivated")
  ("NoMipConstraintFlag",                             encCfg->m_noMipConstraintFlag,                    "Indicate that MIP is deactivated")
  ("NoCclmConstraintFlag",                            encCfg->m_noCclmConstraintFlag,                   "Indicate that CCLM is deactivated")
  ("NoRprConstraintFlag",                             encCfg->m_noRprConstraintFlag,                    "Indicate that reference picture resampling is deactivated")
  ("NoResChangeInClvsConstraintFlag",                 encCfg->m_noResChangeInClvsConstraintFlag,        "Indicate that the picture spatial resolution does not change within any CLVS referring to the SPS")
  ("WeightedPredictionConstraintFlag",                encCfg->m_noWeightedPredictionConstraintFlag,     "Indicate that Weighted Prediction is deactivated")
  ("NoRefWraparoundConstraintFlag",                   encCfg->m_noRefWraparoundConstraintFlag,          "Indicate that Reference Wraparound is deactivated")
  ("NoTemporalMvpConstraintFlag",                     encCfg->m_noTemporalMvpConstraintFlag,            "Indicate that temporal MVP is deactivated")
  ("NoSbtmvpConstraintFlag",                          encCfg->m_noSbtmvpConstraintFlag,                 "Indicate that SbTMVP is deactivated")
  ("NoAmvrConstraintFlag",                            encCfg->m_noAmvrConstraintFlag,                   "Indicate that AMVR is deactivated")
  ("NoSmvdConstraintFlag",                            encCfg->m_noSmvdConstraintFlag,                   "Indicate that SMVD is deactivated")
  ("NoBdofConstraintFlag",                            encCfg->m_noBdofConstraintFlag,                   "Indicate that BIO is deactivated")
  ("NoMmvdConstraintFlag",                            encCfg->m_noMmvdConstraintFlag,                   "Indicate that MMVD is deactivated")
  ("NoAffineMotionConstraintFlag",                    encCfg->m_noAffineMotionConstraintFlag,           "Indicate that Affine is deactivated")
  ("NoProfConstraintFlag",                            encCfg->m_noProfConstraintFlag,                   "Indicate that PROF is deactivated")
  ("NoBcwConstraintFlag",                             encCfg->m_noBcwConstraintFlag,                    "Indicate that BCW is deactivated")
  ("NoCiipConstraintFlag",                            encCfg->m_noCiipConstraintFlag,                   "Indicate that CIIP is deactivated")
  ("NoGpmConstraintFlag",                             encCfg->m_noGeoConstraintFlag,                    "Indicate that GPM is deactivated")
  ("NoSgpmConstraintFlag",                            encCfg->m_noSgpmConstraintFlag,                   "Indicate that SGPM is deactivated")
  ("NoObmcConstraintFlag",                            encCfg->m_noObmcConstraintFlag,                   "Indicate that OBMC is deactivated")
  ("NoTransformSkipConstraintFlag",                   encCfg->m_noTransformSkipConstraintFlag,          "Indicate that Transform Skip is deactivated")
  ("NoLumaTransformSize64ConstraintFlag",             encCfg->m_noLumaTransformSize64ConstraintFlag,    "Indicate that Luma Transform Size 64 is deactivated")
  ("NoBDPCMConstraintFlag",                           encCfg->m_noBDPCMConstraintFlag,                  "Indicate that BDPCM is deactivated")
  ("NoMtsConstraintFlag",                             encCfg->m_noMtsConstraintFlag,                    "Indicate that MTS is deactivated")
  ("NoLfnstConstraintFlag",                           encCfg->m_noLfnstConstraintFlag,                  "Indicate that LFNST is deactivated")
  ("NoJointCbCrConstraintFlag",                       encCfg->m_noJointCbCrConstraintFlag,              "Indicate that JCCR is deactivated")
  ("NoSbtConstraintFlag",                             encCfg->m_noSbtConstraintFlag,                    "Indicate that SBT is deactivated")
  ("NoActConstraintFlag",                             encCfg->m_noActConstraintFlag,                    "Indicate that ACT is deactivated")
  ("NoExplicitScaleListConstraintFlag",               encCfg->m_noExplicitScaleListConstraintFlag,      "Indicate that explicit scaling list is deactivated")
  ("NoChromaQpOffsetConstraintFlag",                  encCfg->m_noChromaQpOffsetConstraintFlag,         "Indicate that chroma qp offset is zero")
  ("NoDepQuantConstraintFlag",                        encCfg->m_noDepQuantConstraintFlag,               "Indicate that DQ is deactivated")
  ("NoSignDataHidingConstraintFlag",                  encCfg->m_noSignDataHidingConstraintFlag,         "Indicate that SDH is deactivated")
  ("NoCuQpDeltaConstraintFlag",                       encCfg->m_noCuQpDeltaConstraintFlag,              "Indicate that CU QP delta is deactivated")
  ("NoSaoConstraintFlag",                             encCfg->m_noSaoConstraintFlag,                    "Indicate that SAO is deactivated")
  ("NoCCSaoConstraintFlag",                           encCfg->m_noCCSaoConstraintFlag,                  "Indicate that CCSAO is deactivated")
  ("NoAlfConstraintFlag",                             encCfg->m_noAlfConstraintFlag,                    "Indicate that ALF is deactivated")
  ("NoCCAlfConstraintFlag",                           encCfg->m_noCCAlfConstraintFlag,                  "Indicate that CCALF is deactivated")
  ("NoLmcsConstraintFlag",                            encCfg->m_noLmcsConstraintFlag,                   "Indicate that LMCS is deactivated")
  ("NoLadfConstraintFlag",                            encCfg->m_noLadfConstraintFlag,                   "Indicate that LADF is deactivated")
  ("AllRapPicturesFlag",                              encCfg->m_allRapPicturesFlag,                     "Indicate that all pictures in OlsInScope are IRAP pictures or GDR pictures with ph_recovery_poc_cnt equal to 0")
  ("NoExtendedPrecisionProcessingConstraintFlag",     encCfg->m_noExtendedPrecisionProcessingConstraintFlag, "Indicate that ExtendedPrecision is deactivated")
  ("NoTsResidualCodingRiceConstraintFlag",            encCfg->m_noTsResidualCodingRiceConstraintFlag,   "Indicate that TSRCRicePresent is deactivated")
  ("NoRrcRiceExtensionConstraintFlag",                encCfg->m_noRrcRiceExtensionConstraintFlag,       "Indicate that ExtendedRiceRRC is deactivated")
  ("NoPersistentRiceAdaptationConstraintFlag",        encCfg->m_noPersistentRiceAdaptationConstraintFlag, "Indicate that GolombRiceParameterAdaptation is deactivated")
  ("NoReverseLastSigCoeffConstraintFlag",             encCfg->m_noReverseLastSigCoeffConstraintFlag,    "Indicate that ReverseLastSigCoeff is deactivated")
  ("CTUSize",                                         encCfg->m_CTUSize,                                "CTUSize (specifies the CTU size if QTBT is on) [default: 128]")
  ("Log2MinCuSize",                                   encCfg->m_log2MinCUSize,                          "Log2 min CU size")
  ("SubPicInfoPresentFlag",                           encCfg->m_subPicInfoPresentFlag,                  "equal to 1 specifies that subpicture parameters are present in in the SPS RBSP syntax")
  ("NumSubPics",                                      encCfg->m_numSubPics,                             "specifies the number of subpictures")
  ("SubPicSameSizeFlag",                              encCfg->m_subPicSameSizeFlag,                     "equal to 1 specifies that all subpictures in the CLVS have the same width specified by sps_subpic_width_minus1[ 0 ] and the same height specified by sps_subpic_height_minus1[ 0 ].")
  ("SubPicCtuTopLeftX",                               cfg_subPicCtuTopLeftX,                            "specifies horizontal position of top left CTU of i-th subpicture in unit of CtbSizeY")
  ("SubPicCtuTopLeftY",                               cfg_subPicCtuTopLeftY,                            "specifies vertical position of top left CTU of i-th subpicture in unit of CtbSizeY")
  ("SubPicWidth",                                     cfg_subPicWidth,                                  "specifies the width of the i-th subpicture in units of CtbSizeY")
  ("SubPicHeight",                                    cfg_subPicHeight,                                 "specifies the height of the i-th subpicture in units of CtbSizeY")
  ("SubPicTreatedAsPicFlag",                          cfg_subPicTreatedAsPicFlag,                       "equal to 1 specifies that the i-th subpicture of each coded picture in the CLVS is treated as a picture in the decoding process excluding in-loop filtering operations")
  ("LoopFilterAcrossSubpicEnabledFlag",               cfg_loopFilterAcrossSubpicEnabledFlag,            "equal to 1 specifies that in-loop filtering operations may be performed across the boundaries of the i-th subpicture in each coded picture in the CLVS")
  ("SubPicIdMappingExplicitlySignalledFlag",          encCfg->m_subPicIdMappingExplicitlySignalledFlag, "equal to 1 specifies that the subpicture ID mapping is explicitly signalled, either in the SPS or in the PPSs")
  ("SubPicIdMappingInSpsFlag",                        encCfg->m_subPicIdMappingInSpsFlag,               "equal to 1 specifies that subpicture ID mapping is signalled in the SPS")
  ("SubPicIdLen",                                     encCfg->m_subPicIdLen,                            "specifies the number of bits used to represent the syntax element sps_subpic_id[ i ]. ")
  ("SubPicId",                                        cfg_subPicId,                                     "specifies that subpicture ID of the i-th subpicture")
  ("SingleSlicePerSubpic",                            encCfg->m_singleSlicePerSubPicFlag,               "Enables setting of a single slice per sub-picture (no explicit configuration required)")
  ("EnablePartitionConstraintsOverride",              encCfg->m_useSplitConsOverride,                   "Enable partition constraints override")
  ("MinQTISlice",                                     encCfg->m_minQt[0],                               "MinQTISlice")
  ("MinQTLumaISlice",                                 encCfg->m_minQt[0],                               "MinQTLumaISlice")
  ("MinQTChromaISliceInChromaSamples",                encCfg->m_minQt[2],                               "MinQTChromaISliceInChromaSamples")
  ("MinQTNonISlice",                                  encCfg->m_minQt[1],                               "MinQTNonISlice")
  ("MaxMTTHierarchyDepth",                            encCfg->m_uiMaxMTTHierarchyDepth,                 "MaxMTTHierarchyDepth (<10: apply for all temporal layers, >=10: each decimal digit specifies the depth for a temporal layer, last digit applying to the highest TL)")
  ("MaxMTTHierarchyDepthI",                           encCfg->m_uiMaxMTTHierarchyDepthI,                "MaxMTTHierarchyDepthI")
  ("MaxMTTHierarchyDepthISliceL",                     encCfg->m_uiMaxMTTHierarchyDepthI,                "MaxMTTHierarchyDepthISliceL")
  ("MaxMTTHierarchyDepthISliceC",                     encCfg->m_uiMaxMTTHierarchyDepthIChroma,          "MaxMTTHierarchyDepthISliceC")
  ("MaxBTLumaISlice",                                 encCfg->m_maxBt[0],                               "MaxBTLumaISlice")
  ("MaxBTChromaISlice",                               encCfg->m_maxBt[2],                               "MaxBTChromaISlice")
  ("MaxBTNonISlice",                                  encCfg->m_maxBt[1],                               "MaxBTNonISlice")
  ("MaxTTLumaISlice",                                 encCfg->m_maxTt[0],                               "MaxTTLumaISlice")
  ("MaxTTChromaISlice",                               encCfg->m_maxTt[2],                               "MaxTTChromaISlice")
  ("MaxTTNonISlice",                                  encCfg->m_maxTt[1],                               "MaxTTNonISlice")
  ("TTFastSkip",                                      encCfg->m_ttFastSkip,                             "fast skip method for TT split partition")
  ("TTFastSkipThr",                                   encCfg->m_ttFastSkipThr,                          "Threshold value of fast skip method for TT split partition")
  ("TemporalPartPred",                                encCfg->m_tempPartPredEnabled,                    "Enable the temporal partitioning prediction based on reference frames (0:off, 1:on)  [default: off]")
  ("DualITree",                                       encCfg->m_dualITree,                              "Use separate QTBT trees for intra slice luma and chroma channel types")
  ("IntraLFNSTISlice",                                encCfg->m_intraLFNSTISlice,                       "Enable LFNST/NSPT for intra blocks in intra slices (0:off, 1:on)  [default: off]" )
  ("IntraLFNSTPBSlice",                               encCfg->m_intraLFNSTPBSlice,                      "Enable LFNST/NSPT for intra blocks in inter slices (0:off, 1:on)  [default: off]" )
  ("InterLFNST",                                      encCfg->m_interLFNST,                             "Enable LFNST/NSPT for inter block (0:off, 1:on)  [default: off]" )
  ("InterLFNSTSBT",                                   encCfg->m_interLFNSTSBT,                          "Enable LFNST/NSPT for SBT block (0:off, 1:on)  [default: off]" )
  ("SbTMVP",                                          encCfg->m_sbTmvpEnableFlag,                       "Enable Subblock Temporal Motion Vector Prediction (0: off, 1: on) [default: off]")
  ("MMVD",                                            encCfg->m_MMVD,                                   "Enable Merge mode with Motion Vector Difference (0:off, 1:on)  [default: 1]")
  ("Affine",                                          encCfg->m_Affine,                                 "Enable affine prediction (0: disabled, 1: vtm, 2-4: fast modes)  [default: 0]")
  ("AffineType",                                      encCfg->m_AffineType,                             "Enable affine type prediction (0:off, 1:on)  [default: on]" )
  ("AffineParameterRefinement",                       encCfg->m_affineParaRefinement,                   "Affine non-translation parameter refinement")
  ("AdaptBypassAffineMe",                             encCfg->m_adaptBypassAffineMe,                    "Adaptively bypass affine ME (0: off, 1:on, defaul: off]")
  ("AffineMMVD",                                      encCfg->m_AffineMmvdMode,                         "Affine MMVD mode (0:off, 1:on)  [default: on]" )
  ("MinAffineBlkSize",                                encCfg->m_minAffineBlkSize,                       "Minimum blocksize for Affine Inter Search [default: 16]" )
  ("AffineSubBlockMrgExt",                            encCfg->m_affineSbMrgExt,                         "Sub-block merge mode extension for affine")
  ("PROF",                                            encCfg->m_PROF,                                   "Enable Prediction refinement with optical flow for affine mode (0:off, 1:on)  [default: off]")
  ("DMVD",                                            encCfg->m_useDMVD,                                "DMVD mode (0:off, 1:on)  [default: on]" )
  ("BIO",                                             encCfg->m_BIO,                                    "Enable bi-directional optical flow")
  ("DMVDBIOExt",                                      encCfg->m_DMVDBIOExt,                             "Enable bi-directional optical flow refinement extension for DMVD")
  ("IMV",                                             encCfg->m_ImvMode,                                "Adaptive MV precision Mode (IMV)\n"
                                                                                                        "\t0: disabled\n"
                                                                                                        "\t1: enabled (1/2-Pel, Full-Pel and 4-PEL)\n")
  ("IMV4PelFast",                                     encCfg->m_Imv4PelFast,                            "Fast 4-Pel Adaptive MV precision Mode 0:disabled, 1:enabled)  [default: 1]")
  ("LMChroma",                                        encCfg->m_LMChroma,                               " LMChroma prediction "
                                                                                                        "\t0:  Disable LMChroma\n"
                                                                                                        "\t1:  Enable LMChroma\n")

  ("CCCM",                                            encCfg->m_CCCM,                                    "Enable Convolutional Cross-Component Model intra prediction" )
  ("MCBP",                                            encCfg->m_MCBP,                                    "Enable Motion-Compensated Boundary Padding" )
  ("TMBP",                                            encCfg->m_TMBP,                                    "Enable Template Matching-based Boundary Padding" )
  ("HorCollocatedChroma",                             encCfg->m_horCollocatedChromaFlag,                "Specifies location of a chroma sample relatively to the luma sample in horizontal direction in the reference picture resampling\n"
                                                                                                        "\t0:  horizontally shifted by 0.5 units of luma samples\n"
                                                                                                        "\t1:  collocated (default)\n")
  ("VerCollocatedChroma",                             encCfg->m_verCollocatedChromaFlag,                "Specifies location of a chroma sample relatively to the luma sample in vertical direction in the cross-component linear model intra prediction and the reference picture resampling\n"
                                                                                                        "\t0:  horizontally co-sited, vertically shifted by 0.5 units of luma samples\n"
                                                                                                        "\t1:  collocated\n")
  ("MTS",                                             encCfg->m_mtsMode,                                "Multiple Transform Set (MTS)\n"
                                                                                                        "\t0:  Disable MTS\n"
                                                                                                        "\t1:  Enable explicit Intra MTS\n"
                                                                                                        "\t2:  Enable implicit Intra and explicit Inter MTS\n"
                                                                                                        "\t3:  Enable explicit Intra and explicit Inter MTS\n"
                                                                                                        "\t4:  Enable implicit Intra MTS\n")
  ("MTSImplicit",                                     encCfg->m_implicitMtsIntra,                       "Enable implicit Intra MTS (when MTS is 0)\n")
  ("SBT",                                             encCfg->m_SBT,                                    "Enable Sub-Block Transform for inter blocks\n" )
  ("SBTFast64WidthTh",                                encCfg->m_SBTFast64WidthTh,                       "Picture width threshold for testing size-64 SBT in RDO (now for HD and above sequences)\n")
  ("SMVD",                                            encCfg->m_SMVD,                                   "Enable Symmetric MVD(0:off 1:VTM 2:fast 3:faster)\n")
  ("CompositeLTReference",                            encCfg->m_compositeRefEnabled,                    "Enable Composite Long Term Reference Frame")
  ("BCW",                                             encCfg->m_bcw,                                    "Enable Generalized Bi-prediction(Bcw)")
  ("BcwFast",                                         encCfg->m_BcwFast,                                "Fast methods for Generalized Bi-prediction(Bcw)\n")
  ("LADF",                                            encCfg->m_ladfEnabled,                            "Luma adaptive deblocking filter QP Offset(L0414)")
  ("LadfNumIntervals",                                encCfg->m_ladfNumIntervals,                       "LADF number of intervals (2-5, inclusive)")
  ("LadfQpOffset",                                    cfg_ladfQpOffset,                                 "LADF QP offset")
  ("LadfIntervalLowerBound",                          cfg_ladfIntervalLowerBound,                       "LADF lower bound for 2nd lowest interval")
  ("CIIP",                                            encCfg->m_ciip,                                   "Enable CIIP mode")
  ("Geo",                                             encCfg->m_Geo,                                    "Enable geometric partitioning mode (0:off, 1:on)")
  ("SGPM",                                            encCfg->m_sgpm,                                   "Enable spatial geometric partitioning mode\n" )
  ("SgpmNoBlend",                                     encCfg->m_sgpmNoBlend,                            "Enable no blend for spatial geometric partitioning mode\n" )
  ("HashME",                                          encCfg->m_HashMECfgEnable,                        "Enable hash motion estimation (0:off, 1:on)")
  ("AllowDisFracMMVD",                                encCfg->m_allowDisFracMMVD,                       "Disable fractional MVD in MMVD mode adaptively")
  ("AffineAmvr",                                      encCfg->m_AffineAmvr,                             "Eanble AMVR for affine inter mode")
  ("AffineAmvrEncOpt",                                encCfg->m_AffineAmvrEncOpt,                       "Enable encoder optimization of affine AMVR")
  ("AffineAmvp",                                      encCfg->m_AffineAmvp,                             "Enable AMVP for affine inter mode")
  ("OBMC",                                            encCfg->m_obmc,                                   "enable (OBMC) overlapped block motion compensation")
  ("MmvdDisNum",                                      encCfg->m_MmvdDisNum,                             "Number of MMVD Distance Entries")
  ("PLT",                                             encCfg->m_PLTMode,                                "PLTMode (0x1:enabled, 0x0:disabled)  [default: disabled]")
  ("JointCbCr",                                       encCfg->m_jointCbCrMode,                          "Enable joint coding of chroma residuals (JointCbCr, 0:off, 1:on)")
  ("IBC",                                             encCfg->m_ibcMode,                                "IBCMode (0: disabled, 1: enabled for I slices, 2: enabled for P/B slices, 3: enabled for all slices)  [default: disabled]")
  ("IBCFrac",                                         encCfg->m_ibcFracMode,                            "IBCMode with fractional BV (0x1:enabled, 0x0:disabled)  [default: disabled]")
  ("IBCMerge",                                        encCfg->m_ibcMerge,                               "Enable IBC merge")
  ("IBCLocalSearchRangeX",                            encCfg->m_ibcLocalSearchRangeX,                   "Search range of IBC local search in x direction")
  ("IBCLocalSearchRangeY",                            encCfg->m_ibcLocalSearchRangeY,                   "Search range of IBC local search in y direction")
  ("IBCHashSearch",                                   encCfg->m_ibcHashSearch,                          "Hash based IBC search")
  ("IBCHashSearchMaxCand",                            encCfg->m_ibcHashSearchMaxCand,                   "Max candidates for hash based IBC search")
  ("IBCHashSearchRange4SmallBlk",                     encCfg->m_ibcHashSearchRange4SmallBlk,            "Small block search range in based IBC search")
  ("IBCFastMethod",                                   encCfg->m_ibcFastMethod,                          "Fast methods for IBC")
  ("WrapAround",                                      encCfg->m_wrapAround,                             "Enable horizontal wrap-around motion compensation for inter prediction (0:off, 1:on)  [default: off]")
  ("WrapAroundOffset",                                encCfg->m_wrapAroundOffset,                       "Offset in luma samples used for computing the horizontal wrap-around position")
  ("EncDbOpt",                                        encCfg->m_encDbOpt,                               "Encoder optimization with deblocking filter")
  ("LMCSEnable",                                      encCfg->m_lmcsEnabled,                            "Enable LMCS (luma mapping with chroma scaling")
  ("BIF",                                             encCfg->m_BIF,                                    "bilateral filter   (0: off, 1:on)  [default: on]")
  ("BIFStrength",                                     encCfg->m_BIFStrength,                            "bilateral filter strength  (0: half, 1: full, 2: double)  [default: full]")
  ("BIFQPOffset",                                     encCfg->m_BIFQPOffset,                            "bilateral filter QP offset (0: no offset)  [default: 0]")
  ("ChromaBIF",                                       encCfg->m_chromaBIF,                              "chroma bilateral filter   (0: off, 1:on)  [default: on]")
  ("ChromaBIFStrength",                               encCfg->m_chromaBIFStrength,                      "chroma bilateral filter strength  (0: half, 1: full, 2: double)  [default: full]")
  ("ChromaBIFQPOffset",                               encCfg->m_chromaBIFQPOffset,                      "chroma bilateral filter QP offset (0: no offset)  [default: 0]")
  ("LMCSSignalType",                                  encCfg->m_reshapeSignalType,                      "Input signal type: 0:SDR, 1:HDR-PQ, 2:HDR-HLG")
  ("LMCSUpdateCtrl",                                  encCfg->m_updateCtrl,                             "LMCS model update control: 0:RA, 1:AI, 2:LDB/LDP")
  ("LMCSAdpOption",                                   encCfg->m_adpOption,                              "LMCS adaptation options: 0:automatic(default),"
                                                                                                        "1: rsp both (CW66 for QP<=22), 2: rsp TID0 (for all QP),"
                                                                                                        "3: rsp inter(CW66 for QP<=22), 4: rsp inter(for all QP).")
  ("LMCSInitialCW",                                   encCfg->m_initialCW,                              "LMCS initial total codeword (0~1023) when LMCSAdpOption > 0")
  ("LMCSOffset",                                      encCfg->m_CSoffset,                               "LMCS chroma residual scaling offset")
  ("IntraCMD",                                        encCfg->m_intraCMD,                               "IntraChroma MD: 0: none, 1:fixed to default wPSNR weight")
  ("LCTUFast",                                        encCfg->m_useFastLCTU,                            "Fast methods for large CTU")
  ("FastMrg",                                         encCfg->m_useFastMrg,                             "Fast methods for inter merge")
  ("MaxMergeRdCandNumTotal",                          encCfg->m_maxMergeRdCandNumTotal,                 "Max total number of merge candidates in full RD checking")
  ("MergeRdCandQuotaRegular",                         encCfg->m_mergeRdCandQuotaRegular,                "Quota of regular merge candidates in full RD checking")
  ("MergeRdCandQuotaRegularSmallBlk",                 encCfg->m_mergeRdCandQuotaRegularSmallBlk,        "Quota of regular merge candidates in full RD checking for blocks < 64 luma samples")
  ("MergeRdCandQuotaSubBlk",                          encCfg->m_mergeRdCandQuotaSubBlk,                 "Quota of sub-block merge candidates in full RD checking")
  ("MergeRdCandQuotaCiip",                            encCfg->m_mergeRdCandQuotaCiip,                   "Quota of CIIP merge candidates in full RD checking")
  ("MergeRdCandQuotaGpm",                             encCfg->m_mergeRdCandQuotaGpm,                    "Quota of GPM merge candidates in full RD checking")
  ("MergeRdCandQuotaNonGeoPreserved",                 encCfg->m_mergeRdCandQuotaNonGeoPreserved,        "Preserved quota of non-geometrical merge candidates in full RD check")
  ("PBIntraFast",                                     encCfg->m_usePbIntraFast,                         "Fast assertion if the intra mode is probable")
  ("AMaxBT",                                          encCfg->m_useAMaxBT,                              "Adaptive maximal BT-size")
  ("CostBasedMTTSkipping",                            encCfg->m_useCostBasedMttSkipping,                "MTT split modes early termination based on cabac cost of split modes flags")
  ("E0023FastEnc",                                    encCfg->m_e0023FastEnc,                           "Fast encoding setting for QTBT (proposal E0023)")
  ("ContentBasedFastQtbt",                            encCfg->m_contentBasedFastQtbt,                   "Signal based QTBT speed-up")
  ("UseNonLinearAlfLuma",                             encCfg->m_useNonLinearAlfLuma,                    "Non-linear adaptive loop filters for Luma Channel")
  ("UseNonLinearAlfChroma",                           encCfg->m_useNonLinearAlfChroma,                  "Non-linear adaptive loop filters for Chroma Channels")
  ("MaxNumAlfAlternativesChroma",                     encCfg->m_maxNumAlfAlternativesChroma,            std::string("Maximum number of alternative Chroma filters (1-") + std::to_string(AlfParameters::ALF_MAX_NUM_ALTERNATIVES_CHROMA) + std::string (", inclusive)") )
  ("MRL",                                             encCfg->m_MRL,                                    "Enable MRL (multiple reference line intra prediction)")
  ("MIP",                                             encCfg->m_MIP,                                    "Enable MIP (matrix-based intra prediction)")
  ("PDP",                                             encCfg->m_pdp,                                    "PDP (0:off, 1:on)  [default: on]" )
  ("DirectionalPlanar",                               encCfg->m_dirPlanar,                              "Enable directional planar mode")
  ("DIMD",                                            encCfg->m_DIMD,                                   "Enable DIMD")
  ("DIMDChroma",                                      encCfg->m_DIMDChroma,                             "Enable DIMD chroma")
  ("TIMD",                                            encCfg->m_TIMD,                                   "Enable TIMD")
  ("TIMDSAD",                                         encCfg->m_TIMDSAD,                                "Enable TIMD-SAD")
  ("OBIC",                                            encCfg->m_OBIC,                                   "Enable OBIC")
  ("EIP",                                             encCfg->m_EIP,                                    "Enable EIP")
  ("MMEIP",                                           encCfg->m_MMEIP,                                  "Enable MMEIP")
  ("FastMIP",                                         encCfg->m_useFastMIP,                             "Fast encoder search for MIP (matrix-based intra prediction) 0:off, 1:vtm, 2,3,4 :faster")
  ("SplitPredictAdaptMode",                           encCfg->m_fastAdaptCostPredMode,                  "Mode for split cost prediction, 0..2 (Default: 0)" )
  ("DisableFastTTfromBT",                             encCfg->m_disableFastDecisionTT,                  "Disable fast decision for TT from BT")
  ("QtbttSpeedUp",                                    encCfg->m_qtbttSpeedUp,                           "Extra QTBTT speedup")
  ("InterMTSMaxSize",                                 encCfg->m_interMTSMaxSize,                        "InterMTSMaxSize")
  ("BvgCccm",                                         encCfg->m_bvgCccm,                                "Block Vector Guided CCCM (0: off, 1:on)  [default: on]")
  ("CcBoostFilter",                                   encCfg->m_ccBoostFilter,                          "Cross-component multi-model filter")
  ("CcBoostTplRefSel",                                encCfg->m_ccBoostTplRefSel,                       "Template based ref area selection for CCCM")
  ("CcMerge",                                         encCfg->m_ccMerge,                                "Cross-component merge")
  ("CcMergeFusion",                                   encCfg->m_ccMergeFusion,                          "Cross-component merge with fusion")
  ("CcDecDerivedMode",                                encCfg->m_ccDecDerivedMode,                       "Decoder derived cross-component mode")
  ;opts.addOptions()

  ("Log2MaxTbSize",                                   encCfg->m_log2MaxTbSize,                          "Maximum transform block size in logarithm base 2 (Default: 7)")
  // Coding structure paramters
  ("IntraPeriod,-ip",                                 encCfg->m_intraPeriod,                            "Intra period in frames, (-1: only first frame)")
  ("DecodingRefreshType,-dr",                         encCfg->m_decodingRefreshType,                    "Intra refresh type (0:none 1:CRA 2:IDR 3:RecPointSEI)")
  ("GOPSize,g",                                       encCfg->m_gopSize,                                "GOP size of temporal structure")
  ("DRAPPeriod",                                      encCfg->m_drapPeriod,                             "DRAP period in frames (0: disable Dependent RAP indication SEI messages)")
  ("EDRAPPeriod",                                     encCfg->m_edrapPeriod,                            "EDRAP period in frames (0: disable Extended Dependent RAP indication SEI messages)")
  ("ReWriteParamSets",                                encCfg->m_rewriteParamSets,                       "Enable rewriting of Parameter sets before every (intra) random access point")
  ("IDRRefParamList",                                 encCfg->m_idrRefParamList,                        "Enable indication of reference picture list syntax elements in slice headers of IDR pictures")
  // motion search options
  ("DisableIntraInInter",                             encCfg->m_bDisableIntraPUsInInterSlices,          "Flag to disable intra PUs in inter slices")
  ("FastSearch",                                      tmpMotionEstimationSearchMethod,                  "0:Full search 1:Diamond 2:Selective 3:Enhanced Diamond")
  ("SearchRange,-sr",                                 encCfg->m_searchRange,                            "Motion search range")
  ("BipredSearchRange",                               encCfg->m_bipredSearchRange,                      "Motion search range for bipred refinement")
  ("MinSearchWindow",                                 encCfg->m_minSearchWindow,                        "Minimum motion search window size for the adaptive window ME")
  ("RestrictMESampling",                              encCfg->m_bRestrictMESampling,                    "Restrict ME Sampling for selective inter motion search")
  ("ClipForBiPredMEEnabled",                          encCfg->m_bClipForBiPredMeEnabled,                "Enables clipping in the Bi-Pred ME. It is disabled to reduce encoder run-time")
  ("FastMEAssumingSmootherMVEnabled",                 encCfg->m_bFastMEAssumingSmootherMVEnabled,       "Enables fast ME assuming a smoother MV.")

  ("HadamardME",                                      encCfg->m_bUseHADME,                              "Hadamard ME for fractional-pel")
  ("ASR",                                             encCfg->m_bUseASR,                                "Adaptive motion search range")
  ("AdditionalCMVP",                                  encCfg->m_bUseAdditionalCMVP,                     "Additional CMVP candidates for merge")
  ;opts.addOptions()

  // Mode decision parameters
  ("LambdaModifier0,-LM0",                            encCfg->m_adLambdaModifier[ 0 ],                  "Lambda modifier for temporal layer 0. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifier1,-LM1",                            encCfg->m_adLambdaModifier[ 1 ],                  "Lambda modifier for temporal layer 1. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifier2,-LM2",                            encCfg->m_adLambdaModifier[ 2 ],                  "Lambda modifier for temporal layer 2. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifier3,-LM3",                            encCfg->m_adLambdaModifier[ 3 ],                  "Lambda modifier for temporal layer 3. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifier4,-LM4",                            encCfg->m_adLambdaModifier[ 4 ],                  "Lambda modifier for temporal layer 4. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifier5,-LM5",                            encCfg->m_adLambdaModifier[ 5 ],                  "Lambda modifier for temporal layer 5. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifier6,-LM6",                            encCfg->m_adLambdaModifier[ 6 ],                  "Lambda modifier for temporal layer 6. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifierI,-LMI",                            cfg_adIntraLambdaModifier,                        "Lambda modifiers for Intra pictures, comma separated, up to one the number of temporal layer. If entry for temporalLayer exists, then use it, else if some are specified, use the last, else use the standard LambdaModifiers.")
  ("IQPFactor,-IQF",                                  encCfg->m_dIntraQpFactor,                         "Intra QP Factor for Lambda Computation. If negative, the default will scale lambda based on GOP size (unless LambdaFromQpEnable then IntraQPOffset is used instead)")
  ("LambdaScaleTowardsNextQP",                        encCfg->m_lambdaScaleTowardsNextQP,               "Scale lambda towards lambda for next integer QP. A negative number increase bitrate and a positive number decrease bitrate.")
  /* Quantization parameters */
  ("QP,q",                                            encCfg->m_iQP,                                    "Qp value")
  ("QPIncrementFrame,-qpif",                          encCfg->m_qpIncrementAtSourceFrame,               "If a source file frame number is specified, the internal QP will be incremented for all POCs associated with source frames >= frame number. If empty, do not increment.")
  ("IntraQPOffset",                                   encCfg->m_intraQPOffset,                          "Qp offset value for intra slice, typically determined based on GOP size")
  ("LambdaFromQpEnable",                              encCfg->m_lambdaFromQPEnable,                     "Enable flag for derivation of lambda from QP")
  ("DeltaQpRD,-dqr",                                  encCfg->m_uiDeltaQpRD,                            "max dQp offset for slice")
  ("MaxDeltaQP,d",                                    encCfg->m_iMaxDeltaQP,                            "max dQp offset for block")
  ("MaxCuDQPSubdiv,-dqd",                             encCfg->m_cuQpDeltaSubdiv,                        "Maximum subdiv for CU luma Qp adjustment")
  ("MaxCuChromaQpOffsetSubdiv",                       encCfg->m_cuChromaQpOffsetSubdiv,                 "Maximum subdiv for CU chroma Qp adjustment")
  ("SliceCuChromaQpOffsetEnabled",                    encCfg->m_cuChromaQpOffsetEnabled,                "Enable local chroma QP offsets (slice level flag)")
  ("FastDeltaQP",                                     encCfg->m_bFastDeltaQP,                           "Fast Delta QP Algorithm")
#if SHARP_LUMA_DELTA_QP
  ("LumaLevelToDeltaQPMode",                          lumaLevelToDeltaQPMode,                           "Luma based Delta QP 0(default): not used. 1: Based on CTU average, 2: Based on Max luma in CTU")
#if !WCG_EXT
  ("LumaLevelToDeltaQPMaxValWeight",                  encCfg->m_lumaLevelToDeltaQPMapping.maxMethodWeight, "Weight of block max luma val when LumaLevelToDeltaQPMode = 2")
#endif
  ("LumaLevelToDeltaQPMappingLuma",                   cfg_lumaLeveltoDQPMappingLuma,                    "Luma to Delta QP Mapping - luma thresholds")
  ("LumaLevelToDeltaQPMappingDQP",                    cfg_lumaLeveltoDQPMappingQP,                      "Luma to Delta QP Mapping - DQP values")
#endif
  ("SmoothQPReductionEnable",                         encCfg->m_smoothQPReductionEnable,                "Enable QP reduction for smooth blocks according to: Clip3(SmoothQPReductionLimit, 0, SmoothQPReductionModelScale*baseQP+SmoothQPReductionModelOffset)")
  ("SmoothQPReductionPeriodicity",                    encCfg->m_smoothQPReductionPeriodicity,           "Periodicity parameter of the QP reduction model, 1: all frames, 0: only intra pictures, 2: every second frame, etc")
  ("SmoothQPReductionThresholdIntra",                 encCfg->m_smoothQPReductionThresholdIntra,        "Threshold parameter for smoothness for intra pictures (SmoothQPReductionThresholdIntra * number of samples in block)")
  ("SmoothQPReductionModelScaleIntra",                encCfg->m_smoothQPReductionModelScaleIntra,       "Scale parameter of the QP reduction model for intra pictures ")
  ("SmoothQPReductionModelOffsetIntra",               encCfg->m_smoothQPReductionModelOffsetIntra,      "Offset parameter of the QP reduction model for intra pictures ")
  ("SmoothQPReductionLimitIntra",                     encCfg->m_smoothQPReductionLimitIntra,            "Threshold parameter for controlling maximum amount of QP reduction by the QP reduction model for intra pictures ")
  ("SmoothQPReductionThresholdInter",                 encCfg->m_smoothQPReductionThresholdInter,        "Threshold parameter for smoothness for inter pictures (SmoothQPReductionThresholdInter * number of samples in block)")
  ("SmoothQPReductionModelScaleInter",                encCfg->m_smoothQPReductionModelScaleInter,       "Scale parameter of the QP reduction model for inter pictures")
  ("SmoothQPReductionModelOffsetInter",               encCfg->m_smoothQPReductionModelOffsetInter,      "Offset parameter of the QP reduction model for inter pictures")
  ("SmoothQPReductionLimitInter",                     encCfg->m_smoothQPReductionLimitInter,            "Threshold parameter for controlling maximum amount of QP reduction by the QP reduction model for inter pictures")
  ("BIM",                                             encCfg->m_bimEnabled,                             "Block Importance Mapping QP adaptation depending on estimated propagation of reference samples. 0 = Off, 1 = On with QP and lambda adaptation, 2 = On with lambda adaptation only")
  ("BIMunitSize",                                     encCfg->m_bimUnitSize,                            "Block size for derivation of BIM offsets (applied on CTU size)")
  ("UseIdentityTableForNon420Chroma",                 encCfg->m_useIdentityTableForNon420Chroma,        "True: Indicates that 422/444 chroma uses identity chroma QP mapping tables; False: explicit Qp table may be specified in config")
  ("SameCQPTablesForAllChroma",                       encCfg->m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag, "0: Different tables for Cb, Cr and joint Cb-Cr components, 1 (default): Same tables for all three chroma components")
  ("QpInValCb",                                       cfg_qpInValCb,                                    "Input coordinates for the QP table for Cb component")
  ("QpOutValCb",                                      cfg_qpOutValCb,                                   "Output coordinates for the QP table for Cb component")
  ("QpInValCr",                                       cfg_qpInValCr,                                    "Input coordinates for the QP table for Cr component")
  ("QpOutValCr",                                      cfg_qpOutValCr,                                   "Output coordinates for the QP table for Cr component")
  ("QpInValCbCr",                                     cfg_qpInValCbCr,                                  "Input coordinates for the QP table for joint Cb-Cr component")
  ("QpOutValCbCr",                                    cfg_qpOutValCbCr,                                 "Output coordinates for the QP table for joint Cb-Cr component")
  ("CbQpOffset,-cbqpofs",                             encCfg->m_chromaCbQpOffset,                       "Chroma Cb QP Offset")
  ("CrQpOffset,-crqpofs",                             encCfg->m_chromaCrQpOffset,                       "Chroma Cr QP Offset")
  ("CbQpOffsetDualTree",                              encCfg->m_chromaCbQpOffsetDualTree,               "Chroma Cb QP Offset for dual tree")
  ("CrQpOffsetDualTree",                              encCfg->m_chromaCrQpOffsetDualTree,               "Chroma Cr QP Offset for dual tree")
  ("CbCrQpOffset,-cbcrqpofs",                         encCfg->m_chromaCbCrQpOffset,                     "QP Offset for joint Cb-Cr mode")
  ("CbCrQpOffsetDualTree",                            encCfg->m_chromaCbCrQpOffsetDualTree,             "QP Offset for joint Cb-Cr mode in dual tree")
#if ER_CHROMA_QP_WCG_PPS
  ("WCGPPSEnable",                                    encCfg->m_wcgChromaQpControl.enabled,             "1: Enable the WCG PPS chroma modulation scheme. 0 (default) disabled")
  ("WCGPPSCbQpScale",                                 encCfg->m_wcgChromaQpControl.chromaCbQpScale,     "WCG PPS Chroma Cb QP Scale")
  ("WCGPPSCrQpScale",                                 encCfg->m_wcgChromaQpControl.chromaCrQpScale,     "WCG PPS Chroma Cr QP Scale")
  ("WCGPPSChromaQpScale",                             encCfg->m_wcgChromaQpControl.chromaQpScale,       "WCG PPS Chroma QP Scale")
  ("WCGPPSChromaQpOffset",                            encCfg->m_wcgChromaQpControl.chromaQpOffset,      "WCG PPS Chroma QP Offset")
#endif
#if W0038_CQP_ADJ
  ("SliceChromaQPOffsetPeriodicity",                  encCfg->m_sliceChromaQpOffsetPeriodicity,         "Used in conjunction with Slice Cb/Cr QpOffsetIntraOrPeriodic. Use 0 (default) to disable periodic nature.")
  ("SliceCbQpOffsetIntraOrPeriodic",                  encCfg->m_sliceChromaQpOffsetIntraOrPeriodic[0],  "Chroma Cb QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.")
  ("SliceCrQpOffsetIntraOrPeriodic",                  encCfg->m_sliceChromaQpOffsetIntraOrPeriodic[1],  "Chroma Cr QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.")
#endif
  ("CbQpOffsetList",                                  cfg_cbQpOffsetList,                               "Chroma Cb QP offset list for local adjustment")
  ("CrQpOffsetList",                                  cfg_crQpOffsetList,                               "Chroma Cb QP offset list for local adjustment")
  ("CbCrQpOffsetList",                                cfg_cbCrQpOffsetList,                             "Chroma joint Cb-Cr QP offset list for local adjustment")

  ("AdaptiveQP,-aq",                                  encCfg->m_bUseAdaptiveQP,                         "QP adaptation based on a psycho-visual model")
  ("MaxQPAdaptationRange,-aqr",                       encCfg->m_iQPAdaptationRange,                     "QP adaptation range")
#if ENABLE_QPA
  ("PerceptQPA,-qpa",                                 encCfg->m_bUsePerceptQPA,                         "perceptually motivated input-adaptive QP modification (default: 0 = off, ignored if -aq is set)")
  ("WPSNR,-wpsnr",                                    encCfg->m_bUseWPSNR,                              "output perceptually weighted peak SNR (WPSNR) instead of PSNR")
#endif
  ("dQPFile,m",                                       encCfg->m_dQPFileName,                            "dQP file name")
  ("RDOQ",                                            encCfg->m_useRDOQ,                                "")
  ("RDOQTS",                                          encCfg->m_useRDOQTS,                              "")
  ("SelectiveRDOQ",                                   encCfg->m_useSelectiveRDOQ,                       "Enable selective RDOQ")

  ;opts.addOptions()

  // Deblocking filter parameters
  ("DeblockingFilterDisable",                         encCfg->m_deblockingFilterDisable,                "")
  ("DeblockingFilterOffsetInPPS",                     encCfg->m_deblockingFilterOffsetInPPS,            "")
  ("DeblockingFilterBetaOffset_div2",                 encCfg->m_deblockingFilterBetaOffsetDiv2,         "")
  ("DeblockingFilterTcOffset_div2",                   encCfg->m_deblockingFilterTcOffsetDiv2,           "")
  ("DeblockingFilterCbBetaOffset_div2",               encCfg->m_deblockingFilterCbBetaOffsetDiv2,       "")
  ("DeblockingFilterCbTcOffset_div2",                 encCfg->m_deblockingFilterCbTcOffsetDiv2,         "")
  ("DeblockingFilterCrBetaOffset_div2",               encCfg->m_deblockingFilterCrBetaOffsetDiv2,       "")
  ("DeblockingFilterCrTcOffset_div2",                 encCfg->m_deblockingFilterCrTcOffsetDiv2,         "")
  ("DeblockingFilterMetric",                          encCfg->m_deblockingFilterMetric,                 "")
  // Coding tools
  ("ReconBasedCrossCPredictionEstimate",              encCfg->m_reconBasedCrossCPredictionEstimate,     "When determining the alpha value for cross-component prediction, use the decoded residual rather than the pre-transform encoder-side residual")
  ("TransformSkip",                                   encCfg->m_useTransformSkip,                       "Intra transform skipping")
  ("TransformSkipFast",                               encCfg->m_useTransformSkipFast,                   "Fast encoder search for transform skipping, winner takes it all mode.")
  ("TransformSkipLog2MaxSize",                        encCfg->m_log2MaxTransformSkipBlockSize,          "Specify transform-skip maximum size. Minimum 2, Maximum 5. (not valid in V1 profiles)")
  ("ChromaTS",                                        encCfg->m_useChromaTS,                            "Enable encoder search of chromaTS")
  ("BDPCM",                                           encCfg->m_useBDPCM,                               "BDPCM (0:off, 1:luma and chroma)")
  ("ResidualRotation",                                encCfg->m_transformSkipRotationEnabledFlag,       "Enable rotation of transform-skipped and transquant-bypassed TUs through 180 degrees prior to entropy coding (not valid in V1 profiles)")
  ("SingleSignificanceMapContext",                    encCfg->m_transformSkipContextEnabledFlag,        "Enable, for transform-skipped and transquant-bypassed TUs, the selection of a single significance map context variable for all coefficients (not valid in V1 profiles)")
  ("ExtendedRiceRRC",                                 encCfg->m_rrcRiceExtensionEnableFlag,             "Enable the extention of the Golomb-Rice parameter derivation for RRC")
  ("GolombRiceParameterAdaptation",                   encCfg->m_persistentRiceAdaptationEnabledFlag,    "Enable the adaptation of the Golomb-Rice parameter over the course of each slice")
  ("AlignCABACBeforeBypass",                          encCfg->m_cabacBypassAlignmentEnabledFlag,        "Align the CABAC engine to a defined fraction of a bit prior to coding bypass data. Must be 1 in high bit rate profile, 0 otherwise")
  ("SAO",                                             encCfg->m_useSao,                                 "Enable Sample Adaptive Offset")
  ("CCSAO",                                           encCfg->m_CCSAO,                                  "Cross-component Sample Adaptive Offset" )
  ("SaoTrueOrg",                                      encCfg->m_saoTrueOrg,                             "Using true original samples for SAO optimization when MCTF is enabled\n")
  ("TestSAODisableAtPictureLevel",                    encCfg->m_bTestSAODisableAtPictureLevel,          "Enables the testing of disabling SAO at the picture level after having analysed all blocks")
  ("SaoEncodingRate",                                 encCfg->m_saoEncodingRate,                        "When >0 SAO early picture termination is enabled for luma and chroma")
  ("SaoEncodingRateChroma",                           encCfg->m_saoEncodingRateChroma,                  "The SAO early picture termination rate to use for chroma (when m_SaoEncodingRate is >0). If <=0, use results for luma")
  ("MaxNumOffsetsPerPic",                             encCfg->m_maxNumOffsetsPerPic,                    "Max number of SAO offset per picture (Default: 2048)")
  ("SAOLcuBoundary",                                  encCfg->m_saoCtuBoundary,                         "0: right/bottom CTU boundary areas skipped from SAO parameter estimation, 1: non-deblocked pixels are used for those areas")
  ("SAOGreedyEnc",                                    encCfg->m_saoGreedyMergeEnc,                      "SAO greedy merge encoding algorithm")
  ("EnablePicPartitioning",                           encCfg->m_picPartitionFlag,                       "Enable picture partitioning (0: single tile, single slice, 1: multiple tiles/slices can be used)")
  ("MixedLossyLossless",                              encCfg->m_mixedLossyLossless,                     "Enable encoder to encode mixed lossy/lossless coding ")
  ("SliceLosslessArray",                              cfgSliceLosslessArray,                            " Lossless slice array Last lossless flag in the  list will be repeated uniformly to cover any remaining slice")
  ("TileColumnWidthArray",                            cfgTileColumnWidth,                               "Tile column widths in units of CTUs. Last column width in list will be repeated uniformly to cover any remaining picture width")
  ("TileRowHeightArray",                              cfgTileRowHeight,                                 "Tile row heights in units of CTUs. Last row height in list will be repeated uniformly to cover any remaining picture height")
  ("RasterScanSlices",                                encCfg->m_rasterSliceFlag,                        "Indicates if using raster-scan or rectangular slices (0: rectangular, 1: raster-scan)")
  ("RectSlicePositions",                              cfgRectSlicePos,                                  "Rectangular slice positions. List containing pairs of top-left CTU RS address followed by bottom-right CTU RS address")
  ("RectSliceFixedWidth",                             encCfg->m_rectSliceFixedWidth,                    "Fixed rectangular slice width in units of tiles (0: disable this feature and use RectSlicePositions instead)")
  ("RectSliceFixedHeight",                            encCfg->m_rectSliceFixedHeight,                   "Fixed rectangular slice height in units of tiles (0: disable this feature and use RectSlicePositions instead)")
  ("RasterSliceSizes",                                cfgRasterSliceSize,                               "Raster-scan slice sizes in units of tiles. Last size in list will be repeated uniformly to cover any remaining tiles in the picture")
  ("DisableLoopFilterAcrossTiles",                    encCfg->m_disableLFCrossTileBoundaryFlag,         "Loop filtering applied across tile boundaries or not (0: filter across tile boundaries  1: do not filter across tile boundaries)")
  ("DisableLoopFilterAcrossSlices",                   encCfg->m_disableLFCrossSliceBoundaryFlag,        "Loop filtering applied across slice boundaries or not (0: filter across slice boundaries 1: do not filter across slice boundaries)")
  ("FastUDIUseMPMEnabled",                            encCfg->m_bFastUDIUseMPMEnabled,                  "If enabled, adapt intra direction search, accounting for MPM")
  ("FastMEForGenBLowDelayEnabled",                    encCfg->m_bFastMEForGenBLowDelayEnabled,          "If enabled use a fast ME for generalised B Low Delay slices")
  ("WeightedPredP,-wpP",                              encCfg->m_useWeightedPred,                        "Use weighted prediction in P slices")
  ("WeightedPredB,-wpB",                              encCfg->m_useWeightedBiPred,                      "Use weighted (bidirectional) prediction in B slices")
  ("WeightedPredMethod,-wpM",                         tmpWeightedPredictionMethod,                      "Weighted prediction method")
  ("Log2ParallelMergeLevel",                          encCfg->m_log2ParallelMergeLevel,                 "Parallel merge estimation region")
  ("WaveFrontSynchro",                                encCfg->m_entropyCodingSyncEnabledFlag,           "0: entropy coding sync disabled; 1 entropy coding sync enabled")
  ("EntryPointsPresent",                              encCfg->m_entryPointPresentFlag,                  "0: entry points is not present; 1 entry points may be present in slice header")
  ("ScalingList",                                     encCfg->m_useScalingListId,                       "0/off: no scaling list, 1/default: default scaling lists, 2/file: scaling lists specified in ScalingListFile")
  ("ScalingListFile",                                 encCfg->m_scalingListFileName,                    "Scaling list file name. Use an empty string to produce help.")
  ("DisableScalingMatrixForLFNST",                    encCfg->m_disableScalingMatrixForLfnstBlks,       "Disable scaling matrices, when enabled, for LFNST-coded blocks")
  ("DisableScalingMatrixForAlternativeColourSpace",   encCfg->m_disableScalingMatrixForAlternativeColourSpace, "Disable scaling matrices when the colour space is not equal to the designated colour space of scaling matrix")
  ("ScalingMatrixDesignatedColourSpace",              encCfg->m_scalingMatrixDesignatedColourSpace,     "Indicates if the designated colour space of scaling matrices is equal to the original colour space")
  ("DepQuant",                                        encCfg->m_DepQuantEnabledIdc,                     "Dependent quantization [0: off, 1: 4 states, 2: 8 stages] (Default: 1)" )
  ("SignHideFlag,-SBH",                               encCfg->m_SignDataHidingEnabledFlag,              "Enable sign hiding" )
  ("NumSignPred",                                     encCfg->m_numPredSign,                            "Number of predicted transform coefficient signs")
  ("Log2SignPredArea",                                encCfg->m_log2SignPredArea,                       "log2 of width/height of area for sign prediction")
  ("TempCabacInit",                                   encCfg->m_tempCabacInitMode,                      "CABAC context initialization from previous picture (0:off, 1:on)")
  ("MaxNumMergeCand",                                 encCfg->m_maxNumMergeCand,                        "Maximum number of merge candidates")
  ("MaxNumAffineMergeCand",                           encCfg->m_maxNumAffineMergeCand,                  "Maximum number of affine merge candidates")
  ("MaxNumGeoCand",                                   encCfg->m_maxNumGeoCand,                          "Maximum number of geometric partitioning mode candidates")
  ("MaxNumIBCMergeCand",                              encCfg->m_maxNumIBCMergeCand,                     "Maximum number of IBC merge candidates")
  ("MaxNumBMMergeCand",                               encCfg->m_maxNumBMMergeCand,                      "Maximum number of BM merge candidates")
  ("MergeOppositeLic",                                encCfg->m_mergeOppositeLic,                       "Enable opposite LIC flag for merge")
  ("MaxNumOppositeLicMergeCand",                      encCfg->m_maxNumOppositeLicMergeCand,             "Maximum number of merge candidates with opposite LIC flag")
  ("MaxNumAffineOppositeLicMergeCand",                encCfg->m_maxNumAffineOppositeLicMergeCand,       "Maximum number of affine merge candidates with opposite LIC flag")
  ("SEIDecodedPictureHash,-dph",                      tmpDecodedPictureHashSEIMappedType,               "Control generation of decode picture hash SEI messages\n"
                                                                                                        "\t3: checksum\n"
                                                                                                        "\t2: CRC\n"
                                                                                                        "\t1: use MD5\n"
                                                                                                        "\t0: disable")
  ("SubpicDecodedPictureHash",                        tmpSubpicDecodedPictureHashMappedType,            "Control generation of decode picture hash SEI messages for each subpicture\n"
                                                                                                        "\t3: checksum\n"
                                                                                                        "\t2: CRC\n"
                                                                                                        "\t1: use MD5\n"
                                                                                                        "\t0: disable")
  ("TMVPMode",                                        encCfg->m_TMVPModeId,                             "TMVP mode 0: TMVP disable for all slices. 1: TMVP enable for all slices (default) 2: TMVP enable for certain slices only")
  ("SliceLevelRpl",                                   encCfg->m_sliceLevelRpl,                          "Code reference picture lists in slice headers rather than picture header.")
  ("SliceLevelDblk",                                  encCfg->m_sliceLevelDblk,                         "Code deblocking filter parameters in slice headers rather than picture header.")
  ("SliceLevelSao",                                   encCfg->m_sliceLevelSao,                          "Code SAO parameters in slice headers rather than picture header.")
  ("SliceLevelAlf",                                   encCfg->m_sliceLevelAlf,                          "Code ALF parameters in slice headers rather than picture header.")
  ("SliceLevelWeightedPrediction",                    encCfg->m_sliceLevelWp,                           "Code weighted prediction parameters in slice headers rather than picture header.")
  ("SliceLevelDeltaQp",                               encCfg->m_sliceLevelDeltaQp,                      "Code delta Qp in slice headers rather than picture header.")
  ("FEN",                                             tmpFastInterSearchMode,                           "fast encoder setting")
  ("ECU",                                             encCfg->m_bUseEarlyCU,                            "Early CU setting")
  ("FDM",                                             encCfg->m_useFastDecisionForMerge,                "Fast decision for Merge RD Cost")
  ("ESD",                                             encCfg->m_useEarlySkipDetection,                  "Early SKIP detection setting")
  ("RateControl",                                     encCfg->m_RCEnableRateControl,                    "Rate control: enable rate control" )
  ("TargetBitrate",                                   encCfg->m_RCTargetBitrate,                        "Rate control: target bit-rate" )
  ("KeepHierarchicalBit",                             encCfg->m_RCKeepHierarchicalBit,                  "Rate control: 0: equal bit allocation; 1: fixed ratio bit allocation; 2: adaptive ratio bit allocation" )
  ("LCULevelRateControl",                             encCfg->m_RCLCULevelRC,                           "Rate control: true: CTU level RC; false: picture level RC" )
  ("RCLCUSeparateModel",                              encCfg->m_RCUseLCUSeparateModel,                  "Rate control: use CTU level separate R-lambda model" )
  ("InitialQP",                                       encCfg->m_RCInitialQP,                            "Rate control: initial QP" )
  ("RCForceIntraQP",                                  encCfg->m_RCForceIntraQP,                         "Rate control: force intra QP to be equal to initial QP" )
  ("RCCpbSaturation",                                 encCfg->m_RCCpbSaturationEnabled,                 "Rate control: enable target bits saturation to avoid CPB overflow and underflow" )
  ("RCCpbSize",                                       encCfg->m_RCCpbSize,                              "Rate control: CPB size" )
  ("RCInitialCpbFullness",                            encCfg->m_RCInitialCpbFullness,                   "Rate control: initial CPB fullness" )
  ("CostMode",                                        encCfg->m_costMode,                               "Use alternative cost functions: choose between 'lossy', 'sequence_level_lossless', 'lossless' (which forces QP to " MACRO_TO_STRING(LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP) ") and 'mixed_lossless_lossy' (which used QP'=" MACRO_TO_STRING(LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME) " for pre-estimates of transquant-bypass blocks).")
  ("TSRCdisableLL",                                   encCfg->m_TSRCdisableLL,                          "Disable TSRC for lossless coding" )
  ("RecalculateQPAccordingToLambda",                  encCfg->m_recalculateQPAccordingToLambda,         "Recalculate QP values according to lambda values. Do not suggest to be enabled in all intra case")
  ("HrdParametersPresent,-hrd",                       encCfg->m_hrdParametersPresentFlag,               "Enable generation of hrd_parameters()")
  ("VuiParametersPresent,-vui",                       encCfg->m_vuiParametersPresentFlag,               "Enable generation of vui_parameters()")
  ("SamePicTimingInAllOLS",                           encCfg->m_samePicTimingInAllOLS,                  "Indicates that the same picture timing SEI message is used in all OLS")
  ("AspectRatioInfoPresent",                          encCfg->m_aspectRatioInfoPresentFlag,             "Signals whether aspect_ratio_idc is present")
  ("AspectRatioIdc",                                  encCfg->m_aspectRatioIdc,                         "aspect_ratio_idc")
  ("SarWidth",                                        encCfg->m_sarWidth,                               "horizontal size of the sample aspect ratio")
  ("SarHeight",                                       encCfg->m_sarHeight,                              "vertical size of the sample aspect ratio")
  ("ColourDescriptionPresent",                        encCfg->m_colourDescriptionPresentFlag,           "Signals whether colour_primaries, transfer_characteristics and matrix_coefficients are present")
  ("ColourPrimaries",                                 encCfg->m_colourPrimaries,                        "Indicates chromaticity coordinates of the source primaries")
  ("TransferCharacteristics",                         encCfg->m_transferCharacteristics,                "Indicates the opto-electronic transfer characteristics of the source")
  ("MatrixCoefficients",                              encCfg->m_matrixCoefficients,                     "Describes the matrix coefficients used in deriving luma and chroma from RGB primaries")
  ("ProgressiveSource",                               encCfg->m_progressiveSourceFlag,                  "Indicate that source is progressive")
  ("InterlacedSource",                                encCfg->m_interlacedSourceFlag,                   "Indicate that source is interlaced")
  ("NonPackedSourceConstraintFlag",                   encCfg->m_nonPackedConstraintFlag,                "Indicate that source does not contain frame packing")
  ("NonProjectedConstraintFlag",                      encCfg->m_nonProjectedConstraintFlag,             "Indicate that the bitstream contains projection SEI messages")
  ("ChromaLocInfoPresent",                            encCfg->m_chromaLocInfoPresentFlag,               "Signals whether chroma_sample_loc_type_top_field and chroma_sample_loc_type_bottom_field are present")
  ("ChromaSampleLocTypeTopField",                     encCfg->m_chromaSampleLocTypeTopField,            "Specifies the location of chroma samples for top field")
  ("ChromaSampleLocTypeBottomField",                  encCfg->m_chromaSampleLocTypeBottomField,         "Specifies the location of chroma samples for bottom field")
  ("ChromaSampleLocType",                             encCfg->m_chromaSampleLocType,                    "Specifies the location of chroma samples for progressive content")
  ("OverscanInfoPresent",                             encCfg->m_overscanInfoPresentFlag,                "Indicates whether conformant decoded pictures are suitable for display using overscan\n")
  ("OverscanAppropriate",                             encCfg->m_overscanAppropriateFlag,                "Indicates whether conformant decoded pictures are suitable for display using overscan\n")
  ("VideoFullRange",                                  encCfg->m_videoFullRangeFlag,                     "Indicates the black level and range of luma and chroma signals")

  ;opts.addOptions()

  ("SEIBufferingPeriod",                              encCfg->m_seiCfg.m_bufferingPeriodSEIEnabled,     "Control generation of buffering period SEI messages")
  ("SEIPictureTiming",                                encCfg->m_seiCfg.m_pictureTimingSEIEnabled,       "Control generation of picture timing SEI messages")
  ("SEIDecodingUnitInfo",                             encCfg->m_seiCfg.m_decodingUnitInfoSEIEnabled,    "Control generation of decoding unit information SEI message.")
  ("SEIScalableNesting",                              encCfg->m_seiCfg.m_scalableNestingSEIEnabled,     "Control generation of scalable nesting SEI messages")
  ("SEIFrameFieldInfo",                               encCfg->m_seiCfg.m_frameFieldInfoSEIEnabled,      "Control generation of frame field information SEI messages")
  ("SEIFramePacking",                                 encCfg->m_seiCfg.m_framePackingSEIEnabled,        "Control generation of frame packing SEI messages")
  ("SEIFramePackingType",                             encCfg->m_seiCfg.m_framePackingSEIType,           "Define frame packing arrangement\n"
                                                                                                        "\t3: side by side - frames are displayed horizontally\n"
                                                                                                        "\t4: top bottom - frames are displayed vertically\n"
                                                                                                        "\t5: frame alternation - one frame is alternated with the other")
  ("SEIFramePackingId",                               encCfg->m_seiCfg.m_framePackingSEIId,             "Id of frame packing SEI message for a given session")
  ("SEIFramePackingQuincunx",                         encCfg->m_seiCfg.m_framePackingSEIQuincunx,       "Indicate the presence of a Quincunx type video frame")
  ("SEIFramePackingInterpretation",                   encCfg->m_seiCfg.m_framePackingSEIInterpretation, "Indicate the interpretation of the frame pair\n"
                                                                                                        "\t0: unspecified\n"
                                                                                                        "\t1: stereo pair, frame0 represents left view\n"
                                                                                                        "\t2: stereo pair, frame0 represents right view")
  ("SEIDisplayOrientationEnabled",                    encCfg->m_seiCfg.m_doSEIEnabled,                  "Controls if display orientation packing SEI message enabled")
  ("SEIDisplayOrientationCancelFlag",                 encCfg->m_seiCfg.m_doSEICancelFlag,               "Specifies the persistence of any previous display orientation SEI message in output order.")
  ("SEIDisplayOrientationPersistenceFlag",            encCfg->m_seiCfg.m_doSEIPersistenceFlag,          "Specifies the persistence of the display orientation packing SEI message for the current layer.")
  ("SEIDisplayOrientationTransformType",              encCfg->m_seiCfg.m_doSEITransformType,            "specifies the rotation and mirroring to be applied to the picture.")
  ("SEIParameterSetsInclusionIndication",             encCfg->m_seiCfg.m_parameterSetsInclusionIndicationSEIEnabled, "Control generation of Parameter sets inclusion indication SEI messages")
  ("SEISelfContainedClvsFlag",                        encCfg->m_seiCfg.m_selfContainedClvsFlag,         "Self contained CLVS indication flag value")
  ("SEIMasteringDisplayColourVolume",                 encCfg->m_seiCfg.m_masteringDisplay.colourVolumeSEIEnabled, "Control generation of mastering display colour volume SEI messages")
  ("SEIMasteringDisplayMaxLuminance",                 encCfg->m_seiCfg.m_masteringDisplay.maxLuminance, "Specifies the mastering display maximum luminance value in units of 1/10000 candela per square metre (32-bit code value)")
  ("SEIMasteringDisplayMinLuminance",                 encCfg->m_seiCfg.m_masteringDisplay.minLuminance, "Specifies the mastering display minimum luminance value in units of 1/10000 candela per square metre (32-bit code value)")
  ("SEIMasteringDisplayPrimaries",                    cfg_DisplayPrimariesCode,                         "Mastering display primaries for all three colour planes in CIE xy coordinates in increments of 1/50000 (results in the ranges 0 to 50000 inclusive)")
  ("SEIMasteringDisplayWhitePoint",                   cfg_DisplayWhitePointCode,                        "Mastering display white point CIE xy coordinates in normalised increments of 1/50000 (e.g. 0.333 = 16667)")
  ("SEIPreferredTransferCharacteristics",             encCfg->m_seiCfg.m_preferredTransferCharacteristics, "Value for the preferred_transfer_characteristics field of the Alternative transfer characteristics SEI which will override the corresponding entry in the VUI. If negative, do not produce the respective SEI message")
  ("SEIErpEnabled",                                   encCfg->m_seiCfg.m_erpSEIEnabled,                 "Control generation of equirectangular projection SEI messages")
  ("SEIErpCancelFlag",                                encCfg->m_seiCfg.m_erpSEICancelFlag,              "Indicate that equirectangular projection SEI message cancels the persistence or follows")
  ("SEIErpPersistenceFlag",                           encCfg->m_seiCfg.m_erpSEIPersistenceFlag,         "Specifies the persistence of the equirectangular projection SEI messages")
  ("SEIErpGuardBandFlag",                             encCfg->m_seiCfg.m_erpSEIGuardBandFlag,           "Indicate the existence of guard band areas in the constituent picture")
  ("SEIErpGuardBandType",                             encCfg->m_seiCfg.m_erpSEIGuardBandType,           "Indicate the type of the guard band")
  ("SEIErpLeftGuardBandWidth",                        encCfg->m_seiCfg.m_erpSEILeftGuardBandWidth,      "Indicate the width of the guard band on the left side of the constituent picture")
  ("SEIErpRightGuardBandWidth",                       encCfg->m_seiCfg.m_erpSEIRightGuardBandWidth,     "Indicate the width of the guard band on the right side of the constituent picture")
  ("SEISphereRotationEnabled",                        encCfg->m_seiCfg.m_sphereRotationSEIEnabled,      "Control generation of sphere rotation SEI messages")
  ("SEISphereRotationCancelFlag",                     encCfg->m_seiCfg.m_sphereRotationSEICancelFlag,   "Indicate that sphere rotation SEI message cancels the persistence or follows")
  ("SEISphereRotationPersistenceFlag",                encCfg->m_seiCfg.m_sphereRotationSEIPersistenceFlag, "Specifies the persistence of the sphere rotation SEI messages")
  ("SEISphereRotationYaw",                            encCfg->m_seiCfg.m_sphereRotationSEIYaw,          "Specifies the value of the yaw rotation angle")
  ("SEISphereRotationPitch",                          encCfg->m_seiCfg.m_sphereRotationSEIPitch,        "Specifies the value of the pitch rotation angle")
  ("SEISphereRotationRoll",                           encCfg->m_seiCfg.m_sphereRotationSEIRoll,         "Specifies the value of the roll rotation angle")
  ("SEIOmniViewportEnabled",                          encCfg->m_seiCfg.m_omniViewportSEIEnabled,        "Control generation of omni viewport SEI messages")
  ("SEIOmniViewportId",                               encCfg->m_seiCfg.m_omniViewportSEIId,             "An identifying number that may be used to identify the purpose of the one or more recommended viewport regions")
  ("SEIOmniViewportCancelFlag",                       encCfg->m_seiCfg.m_omniViewportSEICancelFlag,     "Indicate that omni viewport SEI message cancels the persistence or follows")
  ("SEIOmniViewportPersistenceFlag",                  encCfg->m_seiCfg.m_omniViewportSEIPersistenceFlag, "Specifies the persistence of the omni viewport SEI messages")
  ("SEIOmniViewportCntMinus1",                        encCfg->m_seiCfg.m_omniViewportSEICntMinus1,      "specifies the number of recommended viewport regions minus 1")
  ("SEIOmniViewportAzimuthCentre",                    cfg_omniViewportSEIAzimuthCentre,                 "Indicate the centre of the i-th recommended viewport region")
  ("SEIOmniViewportElevationCentre",                  cfg_omniViewportSEIElevationCentre,               "Indicate the centre of the i-th recommended viewport region")
  ("SEIOmniViewportTiltCentre",                       cfg_omniViewportSEITiltCentre,                    "Indicates the tilt angle of the i-th recommended viewport region")
  ("SEIOmniViewportHorRange",                         cfg_omniViewportSEIHorRange,                      "Indicates the azimuth range of the i-th recommended viewport region")
  ("SEIOmniViewportVerRange",                         cfg_omniViewportSEIVerRange,                      "Indicates the elevation range of the i-th recommended viewport region")
  ("SEIRwpEnabled",                                   encCfg->m_seiCfg.m_rwpSEIEnabled,                 "Controls if region-wise packing SEI message enabled")
  ("SEIRwpCancelFlag",                                encCfg->m_seiCfg.m_rwpSEIRwpCancelFlag,           "Specifies the persistence of any previous region-wise packing SEI message in output order.")
  ("SEIRwpPersistenceFlag",                           encCfg->m_seiCfg.m_rwpSEIRwpPersistenceFlag,      "Specifies the persistence of the region-wise packing SEI message for the current layer.")
  ("SEIRwpConstituentPictureMatchingFlag",            encCfg->m_seiCfg.m_rwpSEIConstituentPictureMatchingFlag, "Specifies the information in the SEI message apply individually to each constituent picture or to the projected picture.")
  ("SEIRwpNumPackedRegions",                          encCfg->m_seiCfg.m_rwpSEINumPackedRegions,        "specifies the number of packed regions when constituent picture matching flag is equal to 0.")
  ("SEIRwpProjPictureWidth",                          encCfg->m_seiCfg.m_rwpSEIProjPictureWidth,        "Specifies the width of the projected picture.")
  ("SEIRwpProjPictureHeight",                         encCfg->m_seiCfg.m_rwpSEIProjPictureHeight,       "Specifies the height of the projected picture.")
  ("SEIRwpPackedPictureWidth",                        encCfg->m_seiCfg.m_rwpSEIPackedPictureWidth,      "specifies the width of the packed picture.")
  ("SEIRwpPackedPictureHeight",                       encCfg->m_seiCfg.m_rwpSEIPackedPictureHeight,     "Specifies the height of the packed picture.")
  ("SEIRwpTransformType",                             cfg_rwpSEIRwpTransformType,                       "specifies the rotation and mirroring to be applied to the i-th packed region.")
  ("SEIRwpGuardBandFlag",                             cfg_rwpSEIRwpGuardBandFlag,                       "specifies the existence of guard band in the i-th packed region.")
  ("SEIRwpProjRegionWidth",                           cfg_rwpSEIProjRegionWidth,                        "specifies the width of the i-th projected region.")
  ("SEIRwpProjRegionHeight",                          cfg_rwpSEIProjRegionHeight,                       "specifies the height of the i-th projected region.")
  ("SEIRwpProjRegionTop",                             cfg_rwpSEIRwpSEIProjRegionTop,                    "specifies the top sample row of the i-th projected region.")
  ("SEIRwpProjRegionLeft",                            cfg_rwpSEIProjRegionLeft,                         "specifies the left-most sample column of the i-th projected region.")
  ("SEIRwpPackedRegionWidth",                         cfg_rwpSEIPackedRegionWidth,                      "specifies the width of the i-th packed region.")
  ("SEIRwpPackedRegionHeight",                        cfg_rwpSEIPackedRegionHeight,                     "specifies the height of the i-th packed region.")
  ("SEIRwpPackedRegionTop",                           cfg_rwpSEIPackedRegionTop,                        "specifies the top luma sample row of the i-th packed region.")
  ("SEIRwpPackedRegionLeft",                          cfg_rwpSEIPackedRegionLeft,                       "specifies the left-most luma sample column of the i-th packed region.")
  ("SEIRwpLeftGuardBandWidth",                        cfg_rwpSEIRwpLeftGuardBandWidth,                  "specifies the width of the guard band on the left side of the i-th packed region.")
  ("SEIRwpRightGuardBandWidth",                       cfg_rwpSEIRwpRightGuardBandWidth,                 "specifies the width of the guard band on the right side of the i-th packed region.")
  ("SEIRwpTopGuardBandHeight",                        cfg_rwpSEIRwpTopGuardBandHeight,                  "specifies the height of the guard band above the i-th packed region.")
  ("SEIRwpBottomGuardBandHeight",                     cfg_rwpSEIRwpBottomGuardBandHeight,               "specifies the height of the guard band below the i-th packed region.")
  ("SEIRwpGuardBandNotUsedForPredFlag",               cfg_rwpSEIRwpGuardBandNotUsedForPredFlag,         "Specifies if the guard bands is used in the inter prediction process.")
  ("SEIRwpGuardBandType",                             cfg_rwpSEIRwpGuardBandType,                       "Specifies the type of the guard bands for the i-th packed region.")
  ("SEIGcmpEnabled",                                  encCfg->m_seiCfg.m_gcmpSEIEnabled,                "Control generation of generalized cubemap projection SEI messages")
  ("SEIGcmpCancelFlag",                               encCfg->m_seiCfg.m_gcmpSEICancelFlag,             "Indicate that generalized cubemap projection SEI message cancels the persistence or follows")
  ("SEIGcmpPersistenceFlag",                          encCfg->m_seiCfg.m_gcmpSEIPersistenceFlag,        "Specifies the persistence of the generalized cubemap projection SEI messages")
  ("SEIGcmpPackingType",                              encCfg->m_seiCfg.m_gcmpSEIPackingType,            "Specifies the packing type")
  ("SEIGcmpMappingFunctionType",                      encCfg->m_seiCfg.m_gcmpSEIMappingFunctionType,    "Specifies the mapping function used to adjust the sample locations of the cubemap projection")
  ("SEIGcmpFaceIndex",                                cfg_gcmpSEIFaceIndex,                             "Specifies the face index for the i-th face")
  ("SEIGcmpFaceRotation",                             cfg_gcmpSEIFaceRotation,                          "Specifies the rotation to be applied to the i-th face")
  ("SEIGcmpFunctionCoeffU",                           cfg_gcmpSEIFunctionCoeffU,                        "Specifies the coefficient used in the cubemap mapping function of the u-axis of the i-th face")
  ("SEIGcmpFunctionUAffectedByVFlag",                 cfg_gcmpSEIFunctionUAffectedByVFlag,              "Specifies whether the cubemap mapping function of the u-axis refers to the v position of the sample location")
  ("SEIGcmpFunctionCoeffV",                           cfg_gcmpSEIFunctionCoeffV,                        "Specifies the coefficient used in the cubemap mapping function of the v-axis of the i-th face")
  ("SEIGcmpFunctionVAffectedByUFlag",                 cfg_gcmpSEIFunctionVAffectedByUFlag,              "Specifies whether the cubemap mapping function of the v-axis refers to the u position of the sample location")
  ("SEIGcmpGuardBandFlag",                            encCfg->m_seiCfg.m_gcmpSEIGuardBandFlag,          "Indicate the existence of guard band areas in the picture")
  ("SEIGcmpGuardBandType",                            encCfg->m_seiCfg.m_gcmpSEIGuardBandType,          "Indicate the type of the guard bands")
  ("SEIGcmpGuardBandBoundaryExteriorFlag",            encCfg->m_seiCfg.m_gcmpSEIGuardBandBoundaryExteriorFlag, "Indicate whether face boundaries contain guard bands")
  ("SEIGcmpGuardBandSamplesMinus1",                   encCfg->m_seiCfg.m_gcmpSEIGuardBandSamplesMinus1, "Specifies the number of guard band samples minus1 used in the cubemap projected picture")
  ("SEISubpicLevelInfoEnabled",                       encCfg->m_seiCfg.m_cfgSubpictureLevelInfoSEI.m_enabled, "Control generation of Subpicture Level Information SEI messages")
  ("SEISubpicLevelInfoRefLevels",                     cfg_sliRefLevels,                                 "List of reference levels for Subpicture Level Information SEI messages")
  ("SEISubpicLevelInfoExplicitFraction",              encCfg->m_seiCfg.m_cfgSubpictureLevelInfoSEI.m_explicitFraction, "Enable sending of explicit fractions in Subpicture Level Information SEI messages")
  ("SEISubpicLevelInfoNumSubpics",                    encCfg->m_seiCfg.m_cfgSubpictureLevelInfoSEI.m_numSubpictures, "Number of subpictures for Subpicture Level Information SEI messages")
  ("SEIAnnotatedRegionsFileRoot,-ar",                 encCfg->m_seiCfg.m_arSEIFileRoot,                 "Annotated region SEI parameters root file name (wo num ext); only the file name base is to be added. Underscore and POC would be automatically addded to . E.g. \"-ar ar\" will search for files ar_0.txt, ar_1.txt, ...")
  ("SEISubpicLevelInfoMaxSublayers",                  encCfg->m_seiCfg.m_cfgSubpictureLevelInfoSEI.m_sliMaxSublayers, "Number of sublayers for Subpicture Level Information SEI messages")
  ("SEISubpicLevelInfoSublayerInfoPresentFlag",       encCfg->m_seiCfg.m_cfgSubpictureLevelInfoSEI.m_sliSublayerInfoPresentFlag, "Enable sending of level information for all sublayers in Subpicture Level Information SEI messages")
  ("SEISubpicLevelInfoRefLevelFractions",             cfg_sliFractions,                                 "List of subpicture level fractions for Subpicture Level Information SEI messages")
  ("SEISubpicLevelInfoNonSubpicLayersFractions",      cfg_sliNonSubpicLayersFractions,                  "List of level fractions for non-subpicture layers in Subpicture Level Information SEI messages")
  ("SEISampleAspectRatioInfo",                        encCfg->m_seiCfg.m_sampleAspectRatioInfoSEIEnabled, "Control generation of Sample Aspect Ratio Information SEI messages")
  ("SEISARICancelFlag",                               encCfg->m_seiCfg.m_sariCancelFlag,                "Indicates that Sample Aspect Ratio Information SEI message cancels the persistence or follows")
  ("SEISARIPersistenceFlag",                          encCfg->m_seiCfg.m_sariPersistenceFlag,           "Specifies the persistence of the Sample Aspect Ratio Information SEI message")
  ("SEISARIAspectRatioIdc",                           encCfg->m_seiCfg.m_sariAspectRatioIdc,            "Specifies the Sample Aspect Ratio IDC of Sample Aspect Ratio Information SEI messages")
  ("SEISARISarWidth",                                 encCfg->m_seiCfg.m_sariSarWidth,                  "Specifies the Sample Aspect Ratio Width of Sample Aspect Ratio Information SEI messages, if extended SAR is chosen.")
  ("SEISARISarHeight",                                encCfg->m_seiCfg.m_sariSarHeight,                 "Specifies the Sample Aspect Ratio Height of Sample Aspect Ratio Information SEI messages, if extended SAR is chosen.")
  ("SEIPhaseIndicationFullResolution",                encCfg->m_seiCfg.m_phaseIndicationSEIEnabledFullResolution, "Control generation of Phase Indication SEI messages for full resolution pictures.")
  ("SEIPIHorPhaseNumFullResolution",                  encCfg->m_seiCfg.m_horPhaseNumFullResolution,     "Specifies the Horizontal Phase Numerator of Phase Indication SEI messages for full resolution pictures.")
  ("SEIPIHorPhaseDenMinus1FullResolution",            encCfg->m_seiCfg.m_horPhaseDenMinus1FullResolution, "Specifies the Horizontal Phase Denominator minus 1 of Phase Indication SEI messages for full resolution pictures.")
  ("SEIPIVerPhaseNumFullResolution",                  encCfg->m_seiCfg.m_verPhaseNumFullResolution,     "Specifies the Vertical Phase Numerator of Phase Indication SEI messages for full resolution pictures.")
  ("SEIPIVerPhaseDenMinus1FullResolution",            encCfg->m_seiCfg.m_verPhaseDenMinus1FullResolution, "Specifies the Vertical Phase Denominator minus 1 of Phase Indication SEI messages for full resolution pictures.")
  ("SEIPhaseIndicationReducedResolution",             encCfg->m_seiCfg.m_phaseIndicationSEIEnabledReducedResolution, "Control generation of Phase Indication SEI messages for reduced resolution pictures.")
  ("SEIPIHorPhaseNumReducedResolution",               encCfg->m_seiCfg.m_horPhaseNumReducedResolution,  "Specifies the Horizontal Phase Numerator of Phase Indication SEI messages for reduced resolution pictures.")
  ("SEIPIHorPhaseDenMinus1ReducedResolution",         encCfg->m_seiCfg.m_horPhaseDenMinus1ReducedResolution, "Specifies the Horizontal Phase Denominator minus 1 of Phase Indication SEI messages for reduced resolution pictures.")
  ("SEIPIVerPhaseNumReducedResolution",               encCfg->m_seiCfg.m_verPhaseNumReducedResolution,  "Specifies the Vertical Phase Numerator of Phase Indication SEI messages for reduced resolution pictures.")
  ("SEIPIVerPhaseDenMinus1ReducedResolution",         encCfg->m_seiCfg.m_verPhaseDenMinus1ReducedResolution, "Specifies the Vertical Phase Denominator minus 1 of Phase Indication SEI messages for reduced resolution pictures.")
  ("MCTSEncConstraint",                               encCfg->m_seiCfg.m_MCTSEncConstraint,             "For MCTS, constrain motion vectors at tile boundaries")
  ("SEIShutterIntervalEnabled",                       encCfg->m_seiCfg.m_siiSEIEnabled,                 "Controls if shutter interval information SEI message is enabled")
  ("SEISiiTimeScale",                                 encCfg->m_seiCfg.m_siiSEITimeScale,               "Specifies sii_time_scale")
  ("SEISiiInputNumUnitsInShutterInterval",            cfg_siiSEIInputNumUnitsInSI,                      "Specifies sub_layer_num_units_in_shutter_interval")
#if ENABLE_TRACING
  ("TraceChannelsList",                               bTracingChannelsList,                             "List all available tracing channels")
  ("TraceRule",                                       sTracingRule,                                     "Tracing rule (ex: \"D_CABAC:poc==8\" or \"D_REC_CB_LUMA:poc==8\")")
  ("TraceFile",                                       sTracingFile,                                     "Tracing file")
#endif

  ;opts.addOptions()

  // film grain characteristics SEI
  ("SEIFGCEnabled",                                   encCfg->m_seiCfg.m_fgcSEIEnabled,                 "Control generation of the film grain characteristics SEI message")
  ("SEIFGCCancelFlag",                                encCfg->m_seiCfg.m_fgcSEICancelFlag,              "Specifies the persistence of any previous film grain characteristics SEI message in output order.")
  ("SEIFGCPersistenceFlag",                           encCfg->m_seiCfg.m_fgcSEIPersistenceFlag,         "Specifies the persistence of the film grain characteristics SEI message for the current layer.")
  ("SEIFGCModelID",                                   encCfg->m_seiCfg.m_fgcSEIModelID,                 "Specifies the film grain simulation model. 0: frequency filtering; 1: auto-regression.")
  ("SEIFGCSepColourDescPresentFlag",                  encCfg->m_seiCfg.m_fgcSEISepColourDescPresentFlag, "Specifies the presence of a distinct colour space description for the film grain characteristics specified in the SEI message.")
  ("SEIFGCBlendingModeID",                            encCfg->m_seiCfg.m_fgcSEIBlendingModeID,          "Specifies the blending mode used to blend the simulated film grain with the decoded images. 0: additive; 1: multiplicative.")
  ("SEIFGCLog2ScaleFactor",                           encCfg->m_seiCfg.m_fgcSEILog2ScaleFactor,         "Specifies a scale factor used in the film grain characterization equations.")
  ("SEIFGCCompModelPresentComp0",                     encCfg->m_seiCfg.m_fgcSEICompModelPresent[0],     "Specifies the presence of film grain modelling on colour component 0.")
  ("SEIFGCCompModelPresentComp1",                     encCfg->m_seiCfg.m_fgcSEICompModelPresent[1],     "Specifies the presence of film grain modelling on colour component 1.")
  ("SEIFGCCompModelPresentComp2",                     encCfg->m_seiCfg.m_fgcSEICompModelPresent[2],     "Specifies the presence of film grain modelling on colour component 2.")
  ("SEIFGCAnalysisEnabled",                           encCfg->m_seiCfg.m_fgcSEIAnalysisEnabled,         "Control adaptive film grain parameter estimation - film grain analysis")
  ("SEIFGCExternalMask",                              encCfg->m_seiCfg.m_fgcSEIExternalMask,            "Read external file with mask for film grain analysis. If empty string, use internally calculated mask.")
  ("SEIFGCExternalDenoised",                          encCfg->m_seiCfg.m_fgcSEIExternalDenoised,        "Read external file with denoised sequence for film grain analysis. If empty string, use MCTF for denoising.")
  ("SEIFGCTemporalFilterPastRefs",                    encCfg->m_seiCfg.m_fgcSEITemporalFilterPastRefs,  "Number of past references for temporal prefilter")
  ("SEIFGCTemporalFilterFutureRefs",                  encCfg->m_seiCfg.m_fgcSEITemporalFilterFutureRefs, "Number of future references for temporal prefilter")
  ("SEIFGCTemporalFilterStrengthFrame*",              encCfg->m_seiCfg.m_fgcSEITemporalFilterStrengths, "Strength for every * frame in FGC-specific temporal filter, where * is an integer.")
  ("SEIFGCPerPictureSEI",                             encCfg->m_seiCfg.m_fgcSEIPerPictureSEI,           "Film Grain SEI is added for each picture as speciffied in RDD5 to ensure bit accurate synthesis in tricky mode")
  ("SEIFGCNumIntensityIntervalMinus1Comp0",           encCfg->m_seiCfg.m_fgcSEINumIntensityIntervalMinus1[0], "Specifies the number of intensity intervals minus1 on colour component 0.")
  ("SEIFGCNumIntensityIntervalMinus1Comp1",           encCfg->m_seiCfg.m_fgcSEINumIntensityIntervalMinus1[1], "Specifies the number of intensity intervals minus1 on colour component 1.")
  ("SEIFGCNumIntensityIntervalMinus1Comp2",           encCfg->m_seiCfg.m_fgcSEINumIntensityIntervalMinus1[2], "Specifies the number of intensity intervals minus1 on colour component 2.")
  ("SEIFGCNumModelValuesMinus1Comp0",                 encCfg->m_seiCfg.m_fgcSEINumModelValuesMinus1[0], "Specifies the number of component model values minus1 on colour component 0.")
  ("SEIFGCNumModelValuesMinus1Comp1",                 encCfg->m_seiCfg.m_fgcSEINumModelValuesMinus1[1], "Specifies the number of component model values minus1 on colour component 1.")
  ("SEIFGCNumModelValuesMinus1Comp2",                 encCfg->m_seiCfg.m_fgcSEINumModelValuesMinus1[2], "Specifies the number of component model values minus1 on colour component 2.")
  ("SEIFGCIntensityIntervalLowerBoundComp0",          cfg_FgcSEIIntensityIntervalLowerBoundComp0,       "Specifies the lower bound for the intensity intervals on colour component 0.")
  ("SEIFGCIntensityIntervalLowerBoundComp1",          cfg_FgcSEIIntensityIntervalLowerBoundComp1,       "Specifies the lower bound for the intensity intervals on colour component 1.")
  ("SEIFGCIntensityIntervalLowerBoundComp2",          cfg_FgcSEIIntensityIntervalLowerBoundComp2,       "Specifies the lower bound for the intensity intervals on colour component 2.")
  ("SEIFGCIntensityIntervalUpperBoundComp0",          cfg_FgcSEIIntensityIntervalUpperBoundComp0,       "Specifies the upper bound for the intensity intervals on colour component 0.")
  ("SEIFGCIntensityIntervalUpperBoundComp1",          cfg_FgcSEIIntensityIntervalUpperBoundComp1,       "Specifies the upper bound for the intensity intervals on colour component 1.")
  ("SEIFGCIntensityIntervalUpperBoundComp2",          cfg_FgcSEIIntensityIntervalUpperBoundComp2,       "Specifies the upper bound for the intensity intervals on colour component 2.")
  ("SEIFGCCompModelValuesComp0",                      cfg_FgcSEICompModelValueComp0,                    "Specifies the component model values on colour component 0.")
  ("SEIFGCCompModelValuesComp1",                      cfg_FgcSEICompModelValueComp1,                    "Specifies the component model values on colour component 1.")
  ("SEIFGCCompModelValuesComp2",                      cfg_FgcSEICompModelValueComp2,                    "Specifies the component model values on colour component 2.")
  // content light level SEI                                                                                                    
  ("SEICLLEnabled",                                   encCfg->m_seiCfg.m_cllSEIEnabled,                 "Control generation of the content light level SEI message")
  ("SEICLLMaxContentLightLevel",                      encCfg->m_seiCfg.m_cllSEIMaxContentLevel,         "When not equal to 0, specifies an upper bound on the maximum light level among all individual samples in a 4:4:4 representation "
                                                                                                        "of red, green, and blue colour primary intensities in the linear light domain for the pictures of the CLVS, "
                                                                                                        "in units of candelas per square metre.When equal to 0, no such upper bound is indicated.")
  ("SEICLLMaxPicAvgLightLevel",                       encCfg->m_seiCfg.m_cllSEIMaxPicAvgLevel,          "When not equal to 0, specifies an upper bound on the maximum average light level among the samples in a 4:4:4 representation "
                                                                                                        "of red, green, and blue colour primary intensities in the linear light domain for any individual picture of the CLVS, "
                                                                                                        "in units of candelas per square metre.When equal to 0, no such upper bound is indicated.")
  // ambient viewing environment SEI
  ("SEIAVEEnabled",                                   encCfg->m_seiCfg.m_aveSEIEnabled,                 "Control generation of the ambient viewing environment SEI message")
  ("SEIAVEAmbientIlluminance",                        encCfg->m_seiCfg.m_aveSEIAmbientIlluminance,      "Specifies the environmental illluminance of the ambient viewing environment in units of 1/10000 lux for the ambient viewing environment SEI message")
  ("SEIAVEAmbientLightX",                             encCfg->m_seiCfg.m_aveSEIAmbientLightX,           "Specifies the normalized x chromaticity coordinate of the environmental ambient light in the nominal viewing enviornment according to the CIE 1931 definition in units of 1/50000 lux for the ambient viewing enviornment SEI message")
  ("SEIAVEAmbientLightY",                             encCfg->m_seiCfg.m_aveSEIAmbientLightY,           "Specifies the normalized y chromaticity coordinate of the environmental ambient light in the nominal viewing enviornment according to the CIE 1931 definition in units of 1/50000 lux for the ambient viewing enviornment SEI message")
  // colour tranform information SEI
  ("SEICTIEnabled",                                   encCfg->m_seiCfg.m_ctiSEIEnabled,                 "Control generation of the Colour transform information SEI message")
  ("SEICTIId",                                        encCfg->m_seiCfg.m_ctiSEIId,                      "Id of the Colour transform information SEI message")
  ("SEICTISignalInfoFlag",                            encCfg->m_seiCfg.m_ctiSEISignalInfoFlag,          "indicates if signal information are present in the Colour transform information SEI message")
  ("SEICTIFullRangeFlag",                             encCfg->m_seiCfg.m_ctiSEIFullRangeFlag,           "specifies signal range after applying the Colour transform information SEI message")
  ("SEICTIPrimaries",                                 encCfg->m_seiCfg.m_ctiSEIPrimaries,               "indicates the signal primaries after applying the Colour transform information SEI message")
  ("SEICTITransferFunction",                          encCfg->m_seiCfg.m_ctiSEITransferFunction,        "indicates the signal transfer function after applying the Colour transform information SEI message")
  ("SEICTIMatrixCoefs",                               encCfg->m_seiCfg.m_ctiSEIMatrixCoefs,             "indicates the signal matrix coefficients after applying the Colour transform information SEI message")
  ("SEICTICrossCompFlag",                             encCfg->m_seiCfg.m_ctiSEICrossComponentFlag,      "Specifies if cross-component transform mode is enabled in SEI CTI")
  ("SEICTICrossCompInferred",                         encCfg->m_seiCfg.m_ctiSEICrossComponentInferred,  "Specifies if cross-component transform LUT is inferred in SEI CTI")
  ("SEICTINbChromaLut",                               encCfg->m_seiCfg.m_ctiSEINumberChromaLut,         "Specifies the number of chroma LUTs in SEI CTI")
  ("SEICTIChromaOffset",                              encCfg->m_seiCfg.m_ctiSEIChromaOffset,            "Specifies the chroma offset of SEI CTI")
  ("SEICTILut0",                                      cfg_SEICTILut0,                                   "slope values for component 0 of SEI CTI")
  ("SEICTILut1",                                      cfg_SEICTILut1,                                   "slope values for component 1 of SEI CTI")
  ("SEICTILut2",                                      cfg_SEICTILut2,                                   "slope values for component 2 of SEI CTI")

  ;opts.addOptions()

  // content colour volume SEI
  ("SEICCVEnabled",                                   encCfg->m_seiCfg.m_ccvSEIEnabled,                 "Control generation of the Content Colour Volume SEI message")
  ("SEICCVCancelFlag",                                encCfg->m_seiCfg.m_ccvSEICancelFlag,              "Specifies the persistence of any previous content colour volume SEI message in output order.")
  ("SEICCVPersistenceFlag",                           encCfg->m_seiCfg.m_ccvSEIPersistenceFlag,         "Specifies the persistence of the content colour volume SEI message for the current layer.")
  ("SEICCVPrimariesPresent",                          encCfg->m_seiCfg.m_ccvSEIPrimariesPresentFlag,    "Specifies whether the CCV primaries are present in the content colour volume SEI message.")
  ("m_ccvSEIPrimariesX0",                             encCfg->m_seiCfg.m_ccvSEIPrimariesX[0],           "Specifies the x coordinate of the first (green) primary for the content colour volume SEI message")
  ("m_ccvSEIPrimariesY0",                             encCfg->m_seiCfg.m_ccvSEIPrimariesY[0],           "Specifies the y coordinate of the first (green) primary for the content colour volume SEI message")
  ("m_ccvSEIPrimariesX1",                             encCfg->m_seiCfg.m_ccvSEIPrimariesX[1],           "Specifies the x coordinate of the second (blue) primary for the content colour volume SEI message")
  ("m_ccvSEIPrimariesY1",                             encCfg->m_seiCfg.m_ccvSEIPrimariesY[1],           "Specifies the y coordinate of the second (blue) primary for the content colour volume SEI message")
  ("m_ccvSEIPrimariesX2",                             encCfg->m_seiCfg.m_ccvSEIPrimariesX[2],           "Specifies the x coordinate of the third (red) primary for the content colour volume SEI message")
  ("m_ccvSEIPrimariesY2",                             encCfg->m_seiCfg.m_ccvSEIPrimariesY[2],           "Specifies the y coordinate of the third (red) primary for the content colour volume SEI message")
  ("SEICCVMinLuminanceValuePresent",                  encCfg->m_seiCfg.m_ccvSEIMinLuminanceValuePresentFlag, "Specifies whether the CCV min luminance value is present in the content colour volume SEI message")
  ("SEICCVMinLuminanceValue",                         encCfg->m_seiCfg.m_ccvSEIMinLuminanceValue,       "specifies the CCV min luminance value  in the content colour volume SEI message")
  ("SEICCVMaxLuminanceValuePresent",                  encCfg->m_seiCfg.m_ccvSEIMaxLuminanceValuePresentFlag, "Specifies whether the CCV max luminance value is present in the content colour volume SEI message")
  ("SEICCVMaxLuminanceValue",                         encCfg->m_seiCfg.m_ccvSEIMaxLuminanceValue,       "specifies the CCV max luminance value  in the content colour volume SEI message")
  ("SEICCVAvgLuminanceValuePresent",                  encCfg->m_seiCfg.m_ccvSEIAvgLuminanceValuePresentFlag, "Specifies whether the CCV avg luminance value is present in the content colour volume SEI message")
  ("SEICCVAvgLuminanceValue",                         encCfg->m_seiCfg.m_ccvSEIAvgLuminanceValue,       "specifies the CCV avg luminance value  in the content colour volume SEI message")
  // scalability dimension information SEI
  ("SEISDIEnabled",                                   encCfg->m_seiCfg.m_sdiSEIEnabled,                 "Control generation of scalaibility dimension information SEI message")
  ("SEISDIMaxLayersMinus1",                           encCfg->m_seiCfg.m_sdiSEIMaxLayersMinus1,         "Specifies the maximum number of layers minus 1 in the current CVS")
  ("SEISDIMultiviewInfoFlag",                         encCfg->m_seiCfg.m_sdiSEIMultiviewInfoFlag,       "Specifies the current CVS may have multiple views and the sdi_view_id_val[ ] syntax elements are present in the scalaibility dimension information SEI message")
  ("SEISDIAuxiliaryInfoFlag",                         encCfg->m_seiCfg.m_sdiSEIAuxiliaryInfoFlag,       "Specifies that one or more layers in the current CVS may be auxiliary layers, which carry auxiliary information, and the sdi_aux_id[ ] syntax elements are present in the scalaibility dimension information SEI message")
  ("SEISDIViewIdLenMinus1",                           encCfg->m_seiCfg.m_sdiSEIViewIdLenMinus1,         "Specifies the length, in bits, of the sdi_view_id_val[ i ] syntax element minus 1 in the scalaibility dimension information SEI message")
  ("SEISDILayerId",                                   cfg_sdiSEILayerId,                                "List of the layer identifiers that may be present in the scalaibility dimension information SEI message in the current CVS")
  ("SEISDIViewIdVal",                                 cfg_sdiSEIViewIdVal,                              "List of the view identifiers in the scalaibility dimension information SEI message")
  ("SEISDIAuxId",                                     cfg_sdiSEIAuxId,                                  "List of the auxiliary identifiers in the scalaibility dimension information SEI message")
  ("SEISDINumAssociatedPrimaryLayersMinus1",          cfg_sdiSEINumAssociatedPrimaryLayersMinus1,       "List of the numbers of associated primary layers of i-th layer, which is an auxiliary layer.")
  // multiview acquisition information SEI
  ("SEIMAIEnabled",                                   encCfg->m_seiCfg.m_maiSEIEnabled,                 "Control generation of multiview acquisition information SEI message")
  ("SEIMAIIntrinsicParamFlag",                        encCfg->m_seiCfg.m_maiSEIIntrinsicParamFlag,      "Specifies the presence of intrinsic camera parameters in the multiview acquisition information SEI message")
  ("SEIMAIExtrinsicParamFlag",                        encCfg->m_seiCfg.m_maiSEIExtrinsicParamFlag,      "Specifies the presence of extrinsic camera parameters in the multiview acquisition information SEI message")
  ("SEIMAINumViewsMinus1",                            encCfg->m_seiCfg.m_maiSEINumViewsMinus1,          "Specifies the number of views minus 1 in the multiview acquisition information SEI message")
  ("SEIMAIIntrinsicParamsEqualFlag",                  encCfg->m_seiCfg.m_maiSEIIntrinsicParamsEqualFlag, "Specifies the intrinsic camera parameters are equal for all cameras in the multiview acquisition information SEI message")
  ("SEIMAIPrecFocalLength",                           encCfg->m_seiCfg.m_maiSEIPrecFocalLength,         "Specifies the exponent of the maximum allowable truncation error for focal_length_x[i] and focal_length_y[i] in the multiview acquisition information SEI message")
  ("SEIMAIPrecPrincipalPoint",                        encCfg->m_seiCfg.m_maiSEIPrecPrincipalPoint,      "Specifies the exponent of the maximum allowable truncation error for principal_point_x[i] and principal_point_y[i] in the multiview acquisition information SEI message")
  ("SEIMAIPrecSkewFactor",                            encCfg->m_seiCfg.m_maiSEIPrecSkewFactor,          "Specifies the exponent of the maximum allowable truncation error for skew factor in the multiview acquisition information SEI message")
  ("SEIMAISignFocalLengthX",                          cfg_maiSEISignFocalLengthX,                       "List of the signs of the focal length of the camera in the horizontal direction in the multiview acquisition information SEI message")
  ("SEIMAIExponentFocalLengthX",                      cfg_maiSEIExponentFocalLengthX,                   "List of the exponent parts of the focal length of the camera in the horizontal direction. in the multiview acquisition information SEI message")
  ("SEIMAIMantissaFocalLengthX",                      cfg_maiSEIMantissaFocalLengthX,                   "List of the mantissa parts of the focal length of the camera in the horizontal direction in the multiview acquisition information SEI message")
  ("SEIMAISignFocalLengthY",                          cfg_maiSEISignFocalLengthY,                       "List of the signs of the focal length of the camera in the vertical direction in the multiview acquisition information SEI message")
  ("SEIMAIExponentFocalLengthY",                      cfg_maiSEIExponentFocalLengthY,                   "List of the exponent parts of the focal length of the camera in the vertical direction in the multiview acquisition information SEI message")
  ("SEIMAIMantissaFocalLengthY",                      cfg_maiSEIMantissaFocalLengthY,                   "List of the mantissa parts of the focal length of the camera in the vertical direction in the multiview acquisition information SEI message")
  ("SEIMAISignPrincipalPointX",                       cfg_maiSEISignPrincipalPointX,                    "List of the signs of the principal point of the camera in the horizontal direction in the multiview acquisition information SEI message")
  ("SEIMAIExponentPrincipalPointX",                   cfg_maiSEIExponentPrincipalPointX,                "List of the exponent parts of the principal point of the camera in the horizontal direction in the multiview acquisition information SEI message")
  ("SEIMAIMantissaPrincipalPointX",                   cfg_maiSEIMantissaPrincipalPointX,                "List of the mantissa parts of the principal point of the camera in the horizontal direction in the multiview acquisition information SEI message")
  ("SEIMAISignPrincipalPointY",                       cfg_maiSEISignPrincipalPointY,                    "List of the signs of the principal point of the camera in the vertical direction in the multiview acquisition information SEI message")
  ("SEIMAIExponentPrincipalPointY",                   cfg_maiSEIExponentPrincipalPointY,                "List of the exponent parts of the principal point of the camera in the vertical direction in the multiview acquisition information SEI message")
  ("SEIMAIMantissaPrincipalPointY",                   cfg_maiSEIMantissaPrincipalPointY,                "List of the mantissa parts of the principal point of the camera in the vertical direction in the multiview acquisition information SEI message")
  ("SEIMAISignSkewFactor",                            cfg_maiSEISignSkewFactor,                         "List of the signs of the skew factor of the camera in the multiview acquisition information SEI message")
  ("SEIMAIExponentSkewFactor",                        cfg_maiSEIExponentSkewFactor,                     "List of the exponent parts of the skew factor of the camera in the multiview acquisition information SEI message")
  ("SEIMAIMantissaSkewFactor",                        cfg_maiSEIMantissaSkewFactor,                     "List of the mantissa parts of the skew factor of the camera in the multiview acquisition information SEI message")
  ("SEIMAIPrecRotationParam",                         encCfg->m_seiCfg.m_maiSEIPrecRotationParam,       "Specifies the exponent of the maximum allowable truncation error for rotation in the multiview acquisition information SEI message")
  ("SEIMAIPrecTranslationParam",                      encCfg->m_seiCfg.m_maiSEIPrecTranslationParam,    "Specifies the exponent of the maximum allowable truncation error for translation in the multiview acquisition information SEI message")
  // multiview view position SEI
  ("SEIMVPEnabled",                                   encCfg->m_seiCfg.m_mvpSEIEnabled,                 "Control generation of multiview view position SEI message")
  ("SEIMVPNumViewsMinus1",                            encCfg->m_seiCfg.m_mvpSEINumViewsMinus1,          "Specifies the number of views minus 1 in the multiview view postion SEI message")
  ("SEIMVPViewPosition",                              cfg_mvpSEIViewPosition,                           "List of View Positions in the multiview view postion SEI message")
  // alpha channel information SEI
  ("SEIACIEnabled",                                   encCfg->m_seiCfg.m_aciSEIEnabled,                 "Control generation of alpha channel information SEI message")
  ("SEIACICancelFlag",                                encCfg->m_seiCfg.m_aciSEICancelFlag,              "Specifies the persistence of any previous alpha channel information SEI message in output order")
  ("SEIACIUseIdc",                                    encCfg->m_seiCfg.m_aciSEIUseIdc,                  "Specifies the usage of the auxiliary picture in the alpha channel information SEI message")
  ("SEIACIBitDepthMinus8",                            encCfg->m_seiCfg.m_aciSEIBitDepthMinus8,          "Specifies the bit depth of the samples of the auxiliary picture in the alpha channel information SEI message")
  ("SEIACITransparentValue",                          encCfg->m_seiCfg.m_aciSEITransparentValue,        "Specifies the interpretation sample value of an auxiliary coded picture luma sample for which the associated luma and chroma samples of the primary coded picture are considered transparent for purposes of alpha blending in the alpha channel information SEI message")
  ("SEIACIOpaqueValue",                               encCfg->m_seiCfg.m_aciSEIOpaqueValue,             "Specifies the interpretation sample value of an auxiliary coded picture luma sample for which the associated luma and chroma samples of the primary coded picture are considered opaque for purposes of alpha blending in the alpha channel information SEI message")
  ("SEIACIIncrFlag",                                  encCfg->m_seiCfg.m_aciSEIIncrFlag,                "Specifies the interpretation sample value for each decoded auxiliary picture luma sample value is equal to the decoded auxiliary picture sample value for purposes of alpha blending in the alpha channel information SEI message")
  ("SEIACIClipFlag",                                  encCfg->m_seiCfg.m_aciSEIClipFlag,                "Specifies whether clipping operation is applied in the alpha channel information SEI message")
  ("SEIACIClipTypeFlag",                              encCfg->m_seiCfg.m_aciSEIClipTypeFlag,            "Specifies the type of clipping operation in the alpha channel information SEI message")

  ;opts.addOptions()

  // depth representation information SEI
  ("SEIDRIEnabled",                                   encCfg->m_seiCfg.m_driSEIEnabled,                 "Control generation of depth representation information SEI message")
  ("SEIDRIZNearFlag",                                 encCfg->m_seiCfg.m_driSEIZNearFlag,               "Specifies the presence of the nearest depth value in the depth representation information SEI message")
  ("SEIDRIZFarFlag",                                  encCfg->m_seiCfg.m_driSEIZFarFlag,                "Specifies the presence of the farthest depth value in the depth representation information SEI message")
  ("SEIDRIDMinFlag",                                  encCfg->m_seiCfg.m_driSEIDMinFlag,                "Specifies the presence of the minimum disparity value in the depth representation information SEI message")
  ("SEIDRIDMaxFlag",                                  encCfg->m_seiCfg.m_driSEIDMaxFlag,                "Specifies the presence of the maximum disparity value in the depth representation information SEI message")
  ("SEIDRIZNear",                                     encCfg->m_seiCfg.m_driSEIZNear,                   "Specifies the nearest depth value in the depth representation information SEI message")
  ("SEIDRIZFar",                                      encCfg->m_seiCfg.m_driSEIZFar,                    "Specifies the farest depth value in the depth representation information SEI message")
  ("SEIDRIDMin",                                      encCfg->m_seiCfg.m_driSEIDMin,                    "Specifies the minimum disparity value in the depth representation information SEI message")
  ("SEIDRIDMax",                                      encCfg->m_seiCfg.m_driSEIDMax,                    "Specifies the maximum disparity value in the depth representation information SEI message")
  ("SEIDRIDepthRepresentationType",                   encCfg->m_seiCfg.m_driSEIDepthRepresentationType, "Specifies the the representation definition of decoded luma samples of auxiliary pictures in the depth representation information SEI message")
  ("SEIDRIDisparityRefViewId",                        encCfg->m_seiCfg.m_driSEIDisparityRefViewId,      "Specifies the ViewId value against which the disparity values are derived in the depth representation information SEI message")
  ("SEIDRINonlinearNumMinus1",                        encCfg->m_seiCfg.m_driSEINonlinearNumMinus1,      "Specifies the number of piece-wise linear segments minus 2 for mapping of depth values to a scale that is uniformly quantized in terms of disparity  in the depth representation information SEI message")
  ("SEIDRINonlinearModel",                            cfg_driSEINonlinearModel,                         "List of the piece-wise linear segments for mapping of decoded luma sample values of an auxiliary picture to a scale that is uniformly quantized in terms of disparity in the depth representation information SEI message")
  ("SEIConstrainedRASL",                              encCfg->m_constrainedRaslEncoding,                "Control generation of constrained RASL encoding SEI message")
  //Processing order of SEI (pos)
  ("SEIPOEnabled",                                    encCfg->m_seiCfg.m_poSEIEnabled,                  "Specifies whether SEI processing order is applied or not")
  ("SEIPOPayLoadType",                                cfg_poSEIPayloadType,                             "List of payloadType for processing")
  ("SEIPOProcessingOrder",                            cfg_poSEIProcessingOrder,                         "List of payloadType processing order")
  ("SEIPONumofPrefixByte",                            cfg_poSEINumofPrefixByte,                         "List of number of prefix bytes")
  ("SEIPOPrefixByte",                                 cfg_poSEIPrefixByte,                              "List of prefix bytes")
  ("SEIPostFilterHintEnabled",                        encCfg->m_seiCfg.m_postFilterHintSEIEnabled,      "Control generation of post-filter Hint SEI message")
  ("SEIPostFilterHintCancelFlag",                     encCfg->m_seiCfg.m_postFilterHintSEICancelFlag,   "Specifies the persistence of any previous post-filter Hint SEI message in output order")
  ("SEIPostFilterHintPersistenceFlag",                encCfg->m_seiCfg.m_postFilterHintSEIPersistenceFlag, "Specifies the persistence of the post-filter Hint SEI message for the current layer")
  ("SEIPostFilterHintSizeY",                          encCfg->m_seiCfg.m_postFilterHintSEISizeY,        "Specifies the vertical size of the post-filter coefficient or correlation array")
  ("SEIPostFilterHintSizeX",                          encCfg->m_seiCfg.m_postFilterHintSEISizeX,        "Specifies the horizontal size of the post-filter coefficient or correlation array")
  ("SEIPostFilterHintType",                           encCfg->m_seiCfg.m_postFilterHintSEIType,         "Specifies the type of the post-filter: 2D-FIR filter (0, default), 1D-FIR filters (1) or Cross-correlation matrix (0)")
  ("SEIPostFilterHintChromaCoeffPresentFlag",         encCfg->m_seiCfg.m_postFilterHintSEIChromaCoeffPresentFlag, "Specifies the presence of post-filter coefficients for chroma")
  ("SEIPostFilterHintValue",                          cfg_postFilterHintSEIValues,                      "Specifies post-filter coefficients or elements of a cross-correlation matrix")
  //SEI manifest
  ("SEISEIManifestEnabled",                           encCfg->m_seiCfg.m_SEIManifestSEIEnabled,         "Controls if SEI Manifest SEI messages enabled")
  //SEI prefix indication
  ("SEISEIPrefixIndicationEnabled",                   encCfg->m_seiCfg.m_SEIPrefixIndicationSEIEnabled, "Controls if SEI Prefix Indications SEI messages enabled")

  ;opts.addOptions()

  ("DebugBitstream",                                  encCfg->m_decodeBitstreams[0],                    "Assume the frames up to POC DebugPOC will be the same as in this bitstream. Load those frames from the bitstream instead of encoding them." )
  ("DebugPOC",                                        encCfg->m_switchPOC,                              "If DebugBitstream is present, load frames up to this POC from this bitstream. Starting with DebugPOC, return to normal encoding." )
  ("DecodeBitstream1",                                encCfg->m_decodeBitstreams[0],                    "Assume the frames up to POC DebugPOC will be the same as in this bitstream. Load those frames from the bitstream instead of encoding them." )
  ("DecodeBitstream2",                                encCfg->m_decodeBitstreams[1],                    "Assume the frames up to POC DebugPOC will be the same as in this bitstream. Load those frames from the bitstream instead of encoding them." )
  ("SwitchPOC",                                       encCfg->m_switchPOC,                              "If DebugBitstream is present, load frames up to this POC from this bitstream. Starting with DebugPOC, return to normal encoding." )
  ("SwitchDQP",                                       encCfg->m_switchDQP,                              "delta QP applied to picture with switchPOC and subsequent pictures." )
  ("FastForwardToPOC",                                encCfg->m_fastForwardToPOC,                       "Get to encoding the specified POC as soon as possible by skipping temporal layers irrelevant for the specified POC." )
  ("StopAfterFFtoPOC",                                encCfg->m_stopAfterFFtoPOC,                       "If using fast forward to POC, after the POC of interest has been hit, stop further encoding.")
  ("ForceDecodeBitstream1",                           encCfg->m_forceDecodeBitstream1,                  "force decoding of bitstream 1 - use this only if you are realy sure about what you are doing ")
  ("DecodeBitstream2ModPOCAndType",                   encCfg->m_bs2ModPOCAndType,                       "Modify POC and NALU-type of second input bitstream, to use second BS as closing I-slice")

  ("DebugCTU",                                        encCfg->m_debugCTU,                               "If DebugBitstream is present, load frames up to this POC from this bitstream. Starting with DebugPOC-frame at CTUline containin debug CTU. Use DebugCTU -2 while encoding to enable reusing the bitstream for debugging with DebugCTU. DebugCTU -2 no support history based ME, -1 history based ME")
  ("AlfTrueOrg",                                      encCfg->m_alfTrueOrg,                             "Using true original samples for ALF optimization when MCTF is enabled\n")
  ("ALF",                                             encCfg->m_alf,                                    "Adaptive Loop Filter\n" )
  ("ALFCCCM",                                         encCfg->m_lfCccm,                                 "Adaptive Loop Filter: Convolutional Cross-Component Model\n")
  ("AlfImprovements",                                 encCfg->m_alfImprovements,                        "Enable improvements for ALF ported from ECM-10.0 (default: disabled)\n")
  ("MaxNumALFAPS",                                    encCfg->m_maxNumAlfAps,                           "Maximum number of ALF APSs" )
  ("AlfapsIDShift",                                   encCfg->m_alfapsIDShift,                          "shift for ALF APSs" )
  ("ConstantJointCbCrSignFlag",                       encCfg->m_constantJointCbCrSignFlag,              "Constant JointCbCr sign flag" )
  ("ALFStrengthLuma",                                 encCfg->m_alfStrengthLuma,                        "Adaptive Loop Filter strength for luma. The parameter scales the magnitudes of the ALF filter coefficients for luma. Valid range is 0.0 <= ALFStrengthLuma <= 1.0")
  ("ALFAllowPredefinedFilters",                       encCfg->m_alfAllowPredefinedFilters,              "Allow use of predefined filters for ALF")
  ("CCALFStrength",                                   encCfg->m_ccalfStrength,                          "Cross-component Adaptive Loop Filter strength. The parameter scales the magnitudes of the CCALF filter coefficients. Valid range is 0.0 <= CCALFStrength <= 1.0")
  ("ALFStrengthChroma",                               encCfg->m_alfStrengthChroma,                      "Adaptive Loop Filter strength for chroma. The parameter scales the magnitudes of the ALF filter coefficients for chroma. Valid range is 0.0 <= ALFStrengthChroma <= 1.0")
  ("ALFStrengthTargetLuma",                           encCfg->m_alfStrengthTargetLuma,                  "Adaptive Loop Filter strength target for ALF luma filter optimization. The parameter scales the auto-correlation matrix E and the cross-correlation vector y for luma. Valid range is 0.0 <= ALFStrengthTargetLuma <= 1.0")
  ("ALFStrengthTargetChroma",                         encCfg->m_alfStrengthTargetChroma,                "Adaptive Loop Filter strength target for ALF chroma filter optimization. The parameter scales the auto-correlation matrix E and the cross-correlation vector y for chroma. Valid range is 0.0 <= ALFStrengthTargetChroma <= 1.0")
  ("CCALFStrengthTarget",                             encCfg->m_ccalfStrengthTarget,                    "Cross-component Adaptive Loop Filter strength target for filter optimization. The parameter scales the auto-correlation matrix E and the cross-correlation vector y. Valid range is 0.0 <= CCALFStrengthTarget <= 1.0")
  ("CCALF",                                           encCfg->m_ccalf,                                  "Cross-component Adaptive Loop Filter" )
  ("CCALFQpTh",                                       encCfg->m_ccalfQpThreshold,                       "QP threshold above which encoder reduces CCALF usage")
  ("RPR",                                             encCfg->m_rprEnabledFlag,                         "Reference Sample Resolution" )
  ("LIC",                                             encCfg->m_licMode,                                "Local illumination compensation [LIC] (0:disabled, 1:enabled only for uni pred, 2: enabled for uni and bi pred)  [default: 0]")
  ("FastPicLevelLIC",                                 encCfg->m_fastPicLevelLIC,                        "Fast picture level LIC decision (0:disabled, 1:enabled)  [default: 0]")
  ("FastLICmode",                                     encCfg->m_fastLICMode,                            "Fast LIC Mode(0:disabled, 1:enabled speed 1, 2:enabled speed 2)  [default: 2]")
  ("ScalingRatioHor",                                 encCfg->m_scalingRatioHor,                        "Scaling ratio in hor direction")
  ("ScalingRatioVer",                                 encCfg->m_scalingRatioVer,                        "Scaling ratio in ver direction")
  ("GOPBasedRPR",                                     encCfg->m_gopBasedRPREnabledFlag,                 "Enables decision to encode pictures in GOP in full resolution or one of three downscaled resolutions(default is 1/2, 2/3 and 4/5 in both dimensions)")
  ("GOPBasedRPRQPTh",                                 encCfg->m_gopBasedRPRQPThreshold,                 "QP threshold parameter that determines which QP GOP-based RPR is invoked for given by QP >= GOPBasedRPRQPTh")
  ("ScalingRatioHor2",                                encCfg->m_scalingRatioHor2,                       "Scaling ratio in hor direction for GOP based RPR (2/3)")
  ("ScalingRatioVer2",                                encCfg->m_scalingRatioVer2,                       "Scaling ratio in ver direction for GOP based RPR (2/3)")
  ("ScalingRatioHor3",                                encCfg->m_scalingRatioHor3,                       "Scaling ratio in hor direction for GOP based RPR (4/5)")
  ("ScalingRatioVer3",                                encCfg->m_scalingRatioVer3,                       "Scaling ratio in ver direction for GOP based RPR (4/5)")
  ("PsnrThresholdRPR",                                encCfg->m_psnrThresholdRPR,                       "PSNR threshold for GOP based RPR (1/2)")
  ("PsnrThresholdRPR2",                               encCfg->m_psnrThresholdRPR2,                      "PSNR threshold for GOP based RPR (2/3)")
  ("PsnrThresholdRPR3",                               encCfg->m_psnrThresholdRPR3,                      "PSNR threshold for GOP based RPR (4/5)")
  ("QpOffsetRPR",                                     encCfg->m_qpOffsetRPR,                            "QP offset for RPR (-6 for 1/2)")
  ("QpOffsetRPR2",                                    encCfg->m_qpOffsetRPR2,                           "QP offset for RPR2 (-4 for 2/3)")
  ("QpOffsetRPR3",                                    encCfg->m_qpOffsetRPR3,                           "QP offset for RPR3 (-2 for 4/5)")
  ("QpOffsetChromaRPR",                               encCfg->m_qpOffsetChromaRPR,                      "QP offset for RPR (-6 for 0.5x)")
  ("QpOffsetChromaRPR2",                              encCfg->m_qpOffsetChromaRPR2,                     "QP offset for RPR2 (-4 for 2/3x)")
  ("QpOffsetChromaRPR3",                              encCfg->m_qpOffsetChromaRPR3,                     "QP offset for RPR3 (-2 for 4/5x)")
  ("PsnrChromaOffsetRPR",                             encCfg->m_psnrChromaOffsetRPR,                    "PSNR offset for including chroma rescaling performance in decision for GOP based RPR")
  ("RPRFunctionalityTesting",                         encCfg->m_rprFunctionalityTestingEnabledFlag,     "Enables RPR functionality testing")
  ("RPRSwitchingResolutionOrderList",                 cfg_rprSwitchingResolutionOrderList,              "Order of resolutions for each segment in RPR functionality testing where 0,1,2,3 corresponds to full resolution,4/5,2/3 and 1/2")
  ("RPRSwitchingQPOffsetOrderList",                   cfg_rprSwitchingQPOffsetOrderList,                "Order of QP offset for each segment in RPR functionality testing, where the QP is modified according to the given offset")
  ("RPRSwitchingSegmentSize",                         encCfg->m_rprSwitchingSegmentSize,                "Segment size with same resolution")
  ("RPRSwitchingTime",                                encCfg->m_rprSwitchingTime,                       "Segment switching time in seconds, when non-zero it defines the segment size according to frame rate (a multiple of 8)")
  ("RPRPopulatePPSatIntra",                           encCfg->m_rprPopulatePPSatIntraFlag,              "Populate all PPS which can be used in the sequence at the Intra, e.g. full-res, 4/5, 2/3 and 1/2")
  ("FractionNumFrames",                               encCfg->m_fractionOfFrames,                       "Encode a fraction of the specified in FramesToBeEncoded frames" )
  ("SwitchPocPeriod",                                 encCfg->m_switchPocPeriod,                        "Switch POC period for RPR" )
  ("UpscaledOutput",                                  encCfg->m_upscaledOutput,                         "Output upscaled (2), decoded but in full resolution buffer (1) or decoded cropped (0, default) picture for RPR" )
  ("UpscaleFilterForDisplay",                         encCfg->m_upscaleFilterForDisplay,                "Filters used for upscaling reconstruction to full resolution (2: ECM 12-tap luma and 6-tap chroma MC filters, 1: Alternative 12-tap luma and 6-tap chroma filters, 0: VVC 8-tap luma and 4-tap chroma MC filters)")
  ("MaxLayers",                                       encCfg->m_maxLayers,                              "Max number of layers" )
  ("EnableOperatingPointInformation",                 encCfg->m_OPIEnabled,                             "Enables writing of Operating Point Information (OPI)" )
  ("MaxTemporalLayer",                                encCfg->m_opiMaxTemporalLayer,                    "Maximum temporal layer to be signalled in OPI" )
  ("TargetOutputLayerSet",                            encCfg->m_targetOlsIdx,                           "Target output layer set index to be signalled in OPI" )
  ("PrintRefLayerMetrics",                            encCfg->m_refLayerMetricsEnabled,                 "0 (default) do not print ref layer metrics, 1 = print ref layer metrics based on current layer source")

  ;opts.addOptions()

  ("MaxSublayers",                                    encCfg->m_maxSublayers,                           "Max number of Sublayers")
  ("DefaultPtlDpbHrdMaxTidFlag",                      encCfg->m_defaultPtlDpbHrdMaxTidFlag,             "specifies that the syntax elements vps_ptl_max_tid[ i ], vps_dpb_max_tid[ i ], and vps_hrd_max_tid[ i ] are not present and are inferred to be equal to the default value vps_max_sublayers_minus1")
  ("AllIndependentLayersFlag",                        encCfg->m_allIndependentLayersFlag,               "All layers are independent layer")
  ("AllowablePredDirection",                          encCfg->m_predDirectionArray,                     "prediction directions allowed for i-th temporal layer")
  ("LayerId%d",                                       encCfg->m_layerId,                            MAX_VPS_LAYERS, "Layer ID")
  ("NumRefLayers%d",                                  encCfg->m_numRefLayers,                       MAX_VPS_LAYERS, "Number of direct reference layer index of i-th layer")
  ("RefLayerIdx%d",                                   encCfg->m_refLayerIdxStr,                     MAX_VPS_LAYERS, "Reference layer index(es)")
  ("EachLayerIsAnOlsFlag",                            encCfg->m_eachLayerIsAnOlsFlag,                   "Each layer is an OLS layer flag")
  ("OlsModeIdc",                                      encCfg->m_olsModeIdc,                             "Output layer set mode")
  ("NumOutputLayerSets",                              encCfg->m_numOutputLayerSets,                     "Number of output layer sets")
  ("OlsOutputLayer%d",                                encCfg->m_olsOutputLayerStr,                  MAX_VPS_LAYERS, "Output layer index of i-th OLS")
  ("NumPTLsInVPS",                                    encCfg->m_numPtlsInVps,                           "Number of profile_tier_level structures in VPS" )
  ("PtPresentInPTL%d",                                encCfg->m_ptPresentInPtl,                     MAX_NUM_OLSS, "Profile/Tier present in i-th PTL")
  ("AvoidIntraInDepLayers",                           encCfg->m_avoidIntraInDepLayer,                   "Replaces I pictures in dependent layers with B pictures" )
  ("MaxTidILRefPicsPlusOneLayerId%d",                 encCfg->m_maxTidILRefPicsPlus1Str,            MAX_VPS_LAYERS, "Maximum temporal ID for inter-layer reference pictures plus 1 of i-th layer, 0 for IRAP only")
  ("RPLofDepLayerInSH",                               encCfg->m_rplOfDepLayerInSh,                      "define Reference picture lists in slice header instead of SPS for dependant layers")

  ;opts.addOptions()

  ("TemporalFilter",                                  encCfg->m_gopBasedTemporalFilterEnabled,          "Enable GOP based temporal filter. Disabled per default")
  ("TemporalFilterPastRefs",                          encCfg->m_gopBasedTemporalFilterPastRefs,         "Number of past references for temporal prefilter")
  ("TemporalFilterFutureRefs",                        encCfg->m_gopBasedTemporalFilterFutureRefs,       "Number of future references for temporal prefilter")
  ("TemporalFilterUnitSize",                          encCfg->m_gopBasedTemporalFilterUnitSize,         "Block size for GOP based temporal filtering operation.")
  ("FirstValidFrame",                                 encCfg->m_firstValidFrame,                        "First valid frame")
  ("LastValidFrame",                                  encCfg->m_lastValidFrame,                         "Last valid frame")
  ("TemporalFilterStrengthFrame*",                    encCfg->m_gopBasedTemporalFilterStrengths,        "Strength for every * frame in GOP based temporal filter, where * is an integer."
                                                                                                        " E.g. --TemporalFilterStrengthFrame8 0.95 will enable GOP based temporal filter at every 8th frame with strength 0.95")
#if ENABLE_NNLF
  ("Nnlf",                                            encCfg->m_nnlf,                                   "NN-based in-loop filter (0: off, 1: VLOP, 2: LOP, 3: HOP)")
  ("NnlfModelName",                                   encCfg->m_nnlfModelName,                          "NNLF model file name (leave empty for default model)")
  ("NnlfPocDivisibleByN",                             encCfg->m_nnlfPocDivisibleByN,                    "Prohibits encoder to do NNLF on pictures where the POC is not divisible by this number: 1: off, 2: filter only even frames, 4: filter only frames with POC divisible by 4 etc")
  ("NnlfDebugOption",                                 encCfg->m_nnlfDebugOption,                        "NNLF debug option: 0: default, 1: apply only on I slice, 2: apply on all slices using I type as input")
  ("NnlfStartPOC",                                    encCfg->m_nnlfStartPoc,                           "POC, where NNLF filter will start working")
#endif
  ("InterRPL",                                        encCfg->m_interRPL,                               "InterRPL  (0: off, 1: on)  [default: off]")
  ;

  // clang-format on

#if EXTENSION_360_VIDEO
  TExt360AppEncCfg::TExt360AppEncCfgContext ext360CfgContext;
  m_ext360.addOptions(opts, ext360CfgContext);
#endif

  for (int i = 0; i < MAX_GOP; i++)
  {
    std::ostringstream cOSS;
    cOSS << "Frame" << (i + 1);
    opts.addOptions()(cOSS.str(), encCfg->m_GOPList[i], "");
  }

  for (int i = 0; i < MAX_NUM_OLSS; i++)
  {
    std::ostringstream cOSS1;
    cOSS1 << "LevelPTL" << i;
    opts.addOptions()(cOSS1.str(), encCfg->m_levelPtl[i], "");

    std::ostringstream cOSS2;
    cOSS2 << "OlsPTLIdx" << i;
    opts.addOptions()(cOSS2.str(), encCfg->m_olsPtlIdx[i], "");
  }

  unsigned tmpNnPostFilterSEICharacteristicsOutColourFormatIdc[MAX_NUM_NN_POST_FILTERS] = {
    (uint32_t)ChromaFormat::_420, (uint32_t)ChromaFormat::_420, (uint32_t)ChromaFormat::_420,
    (uint32_t)ChromaFormat::_420, (uint32_t)ChromaFormat::_420, (uint32_t)ChromaFormat::_420,
    (uint32_t)ChromaFormat::_420, (uint32_t)ChromaFormat::_420
  };

  opts.addOptions()("SEINNPostFilterCharacteristicsEnabled", encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsEnabled,
                    "Control generation of the Neural Network Post Filter Characteristics SEI messages");
  opts.addOptions()("SEINNPostFilterCharacteristicsNumFilters",
                    encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsNumFilters,
                    "Specifies the number of Neural Network Post Filter Characteristics SEI messages");
  for (int i = 0; i < MAX_NUM_NN_POST_FILTERS; i++)
  {
    std::ostringstream id;
    id << "SEINNPostFilterCharacteristicsId" << i;
    opts.addOptions()(id.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsId[i],
                      "Specifies the identifying number in the Neural Network Post Filter Characteristics SEI message");

    std::ostringstream modeIdc;
    modeIdc << "SEINNPostFilterCharacteristicsModeIdc" << i;
    opts.addOptions()(
      modeIdc.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsModeIdc[i],
      "Specifies the Neural Network Post Filter IDC in the Neural Network Post Filter Characteristics SEI message");

    std::ostringstream propertyPresentFlag;
    propertyPresentFlag << "SEINNPostFilterCharacteristicsPropertyPresentFlag" << i;
    opts.addOptions()(propertyPresentFlag.str(),
                      encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsPropertyPresentFlag[i],
                      "Specifies whether the filter purpose, input formatting, output formatting and complexity are "
                      "present in the Neural Network Post Filter Characteristics SEI message");

    std::ostringstream nnpfcBaseFlag;
    nnpfcBaseFlag << "SEINNPostFilterCharacteristicsBaseFlag" << i;
    opts.addOptions()(nnpfcBaseFlag.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsBaseFlag[i],
                      "Specifies whether the filter is a base filter or not");

    std::ostringstream purpose;
    purpose << "SEINNPostFilterCharacteristicsPurpose" << i;
    opts.addOptions()(purpose.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsPurpose[i],
                      "Specifies the purpose in the Neural Network Post Filter Characteristics SEI message");

    std::ostringstream outSubWidthCFlag;
    outSubWidthCFlag << "SEINNPostFilterCharacteristicsOutSubCFlag" << i;
    opts.addOptions()(outSubWidthCFlag.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsOutSubCFlag[i],
                      "Specifies output chroma format when upsampling");

    std::ostringstream outColourFormatIdc;
    outColourFormatIdc << "SEINNPostFilterCharacteristicsOutColourFormatIdc" << i;
    opts.addOptions()(outColourFormatIdc.str(), tmpNnPostFilterSEICharacteristicsOutColourFormatIdc[i],
                      "Specifies output chroma format for colourization purpose");

    std::ostringstream picWidthInLumaSamples;
    picWidthInLumaSamples << "SEINNPostFilterCharacteristicsPicWidthInLumaSamples" << i;
    opts.addOptions()(picWidthInLumaSamples.str(),
                      encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsPicWidthInLumaSamples[i],
                      "Specifies the horizontal luma sample counts of the output picture in the Neural Network Post "
                      "Filter Characteristics SEI message");

    std::ostringstream picHeightInLumaSamples;
    picHeightInLumaSamples << "SEINNPostFilterCharacteristicsPicHeightInLumaSamples" << i;
    opts.addOptions()(picHeightInLumaSamples.str(),
                      encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsPicHeightInLumaSamples[i],
                      "Specifies the vertical luma sample counts of the output picture in the Neural Network Post "
                      "Filter Characteristics SEI message");

    std::ostringstream inpTensorBitDepthLumaMinus8;
    inpTensorBitDepthLumaMinus8 << "SEINNPostFilterCharacteristicsInpTensorBitDepthLumaMinusEight" << i;
    opts.addOptions()(inpTensorBitDepthLumaMinus8.str(),
                      encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsInpTensorBitDepthLumaMinus8[i],
                      "Specifies the bit depth of the input tensor luma minus 8 in the Neural Network Post Filter "
                      "Characteristics SEI message");

    std::ostringstream inpTensorBitDepthChromaMinus8;
    inpTensorBitDepthChromaMinus8 << "SEINNPostFilterCharacteristicsInpTensorBitDepthChromaMinusEight" << i;
    opts.addOptions()(inpTensorBitDepthChromaMinus8.str(),
                      encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsInpTensorBitDepthChromaMinus8[i],
                      "Specifies the bit depth of the input tensor chroma minus 8 in the Neural Network Post Filter "
                      "Characteristics SEI message");

    std::ostringstream outTensorBitDepthLumaMinus8;
    outTensorBitDepthLumaMinus8 << "SEINNPostFilterCharacteristicsOutTensorBitDepthLumaMinusEight" << i;
    opts.addOptions()(outTensorBitDepthLumaMinus8.str(),
                      encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsOutTensorBitDepthLumaMinus8[i],
                      "Specifies the bit depth of the output tensor luma minus 8 in the Neural Network Post Filter "
                      "Characteristics SEI message");

    std::ostringstream outTensorBitDepthChromaMinus8;
    outTensorBitDepthChromaMinus8 << "SEINNPostFilterCharacteristicsOutTensorBitDepthChromaMinusEight" << i;
    opts.addOptions()(outTensorBitDepthChromaMinus8.str(),
                      encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsOutTensorBitDepthChromaMinus8[i],
                      "Specifies the bit depth of the output tensor chroma minus 8 in the Neural Network Post Filter "
                      "Characteristics SEI message");

    std::ostringstream componentLastFlag;
    componentLastFlag << "SEINNPostFilterCharacteristicsComponentLastFlag" << i;
    opts.addOptions()(componentLastFlag.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsComponentLastFlag[i],
                      "Specifies the channel component is located in the last dimension for the Neural Network Post "
                      "Filter Characteristics SEI message");

    std::ostringstream inpFormatIdc;
    inpFormatIdc << "SEINNPostFilterCharacteristicsInpFormatIdc" << i;
    opts.addOptions()(inpFormatIdc.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsInpFormatIdc[i],
                      "Specifies the method of converting an input sample in the the Neural Network Post Filter "
                      "Characteristics SEI message");

    std::ostringstream auxInpIdc;
    auxInpIdc << "SEINNPostFilterCharacteristicsAuxInpIdc" << i;
    opts.addOptions()(
      auxInpIdc.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsAuxInpIdc[i],
      "Specifies the auxillary input index in the Nueral Network Post Filter Characteristics SEI message");

    std::ostringstream sepColDescriptionFlag;
    sepColDescriptionFlag << "SEINNPostFilterCharacteristicsSepColDescriptionFlag" << i;
    opts.addOptions()(sepColDescriptionFlag.str(),
                      encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsSepColDescriptionFlag[i],
                      "Specifies the presence of seperate color descriptions in the Nueral Network Post Filter "
                      "Characteristics SEI message");

    std::ostringstream colPrimaries;
    colPrimaries << "SEINNPostFilterCharacteristicsColPrimaries" << i;
    opts.addOptions()(colPrimaries.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsColPrimaries[i],
                      "Specifies color primaries in the Nueral Network Post Filter Characteristics SEI message");

    std::ostringstream transCharacteristics;
    transCharacteristics << "SEINNPostFilterCharacteristicsTransCharacteristics" << i;
    opts.addOptions()(
      transCharacteristics.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsTransCharacteristics[i],
      "Specifies Transfer Characteristics in the Nueral Network Post Filter Characteristics SEI message");

    std::ostringstream matrixCoeffs;
    matrixCoeffs << "SEINNPostFilterCharacteristicsMatrixCoeffs" << i;
    opts.addOptions()(
      matrixCoeffs.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsMatrixCoeffs[i],
      "Specifies color matrix coefficients in the Nueral Network Post Filter Characteristics SEI message");
    std::ostringstream inpOrderIdc;
    inpOrderIdc << "SEINNPostFilterCharacteristicsInpOrderIdc" << i;
    opts.addOptions()(inpOrderIdc.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsInpOrderIdc[i],
                      "Specifies the method of ordering the input sample arrays in the Neural Network Post Filter "
                      "Characteristics SEI message");

    std::ostringstream outFormatIdc;
    outFormatIdc << "SEINNPostFilterCharacteristicsOutFormatIdc" << i;
    opts.addOptions()(outFormatIdc.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsOutFormatIdc[i],
                      "Specifies the method of converting an output sample in the the Neural Network Post Filter "
                      "Characteristics SEI message");

    std::ostringstream outOrderIdc;
    outOrderIdc << "SEINNPostFilterCharacteristicsOutOrderIdc" << i;
    opts.addOptions()(outOrderIdc.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsOutOrderIdc[i],
                      "Specifies the method of ordering the output sample arrays in the Neural Network Post Filter "
                      "Characteristics SEI message");

    std::ostringstream constantPatchSizeFlag;
    constantPatchSizeFlag << "SEINNPostFilterCharacteristicsConstantPatchSizeFlag" << i;
    opts.addOptions()(
      constantPatchSizeFlag.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsConstantPatchSizeFlag[i],
      "Specifies the patch size flag in the the Neural Network Post Filter Characteristics SEI message");

    std::ostringstream patchWidthMinus1;
    patchWidthMinus1 << "SEINNPostFilterCharacteristicsPatchWidthMinus1" << i;
    opts.addOptions()(patchWidthMinus1.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsPatchWidthMinus1[i],
                      "Specifies the horizontal sample counts of a patch in the Neural Network Post Filter "
                      "Characteristics SEI message");

    std::ostringstream patchHeightMinus1;
    patchHeightMinus1 << "SEINNPostFilterCharacteristicsPatchHeightMinus1" << i;
    opts.addOptions()(
      patchHeightMinus1.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsPatchHeightMinus1[i],
      "Specifies the vertical sample counts of a patch in the Neural Network Post Filter Characteristics SEI message");

    std::ostringstream extendedPatchWidthCdDeltaMinus1;
    extendedPatchWidthCdDeltaMinus1 << "SEINNPostFilterCharacteristicsExtendedPatchWidthCdDeltaMinus1" << i;
    opts.addOptions()(extendedPatchWidthCdDeltaMinus1.str(),
                      encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsExtendedPatchWidthCdDeltaMinus1[i],
                      "Specifies the extended horizontal sample counts of a patch in the Neural Network Post Filter "
                      "Characteristics SEI message");

    std::ostringstream extendedPatchHeightCdDeltaMinus1;
    extendedPatchHeightCdDeltaMinus1 << "SEINNPostFilterCharacteristicsExtendedPatchHeightCdDeltaMinus1" << i;
    opts.addOptions()(extendedPatchHeightCdDeltaMinus1.str(),
                      encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsExtendedPatchHeightCdDeltaMinus1[i],
                      "Specifies the extended vertical sample counts of a patch in the Neural Network Post Filter "
                      "Characteristics SEI message");

    std::ostringstream overlap;
    overlap << "SEINNPostFilterCharacteristicsOverlap" << i;
    opts.addOptions()(overlap.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsOverlap[i],
                      "Specifies the overlap in the Neural Network Post Filter Characteristics SEI message");

    std::ostringstream paddingType;
    paddingType << "SEINNPostFilterCharacteristicsPaddingType" << i;
    opts.addOptions()(paddingType.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsPaddingType[i],
                      "Specifies the process of padding when referencing sample locations outside the boundaries of "
                      "the cropped decoded output picture ");

    std::ostringstream lumaPadding;
    lumaPadding << "SEINNPostFilterCharacteristicsLumaPadding" << i;
    opts.addOptions()(lumaPadding.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsLumaPadding[i],
                      "Specifies the luma padding when when the padding type is fixed padding ");

    std::ostringstream crPadding;
    crPadding << "SEINNPostFilterCharacteristicsCrPadding" << i;
    opts.addOptions()(crPadding.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsCrPadding[i],
                      "Specifies the Cr padding when when the padding type is fixed padding ");

    std::ostringstream cbPadding;
    cbPadding << "SEINNPostFilterCharacteristicsCbPadding" << i;
    opts.addOptions()(cbPadding.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsCbPadding[i],
                      "Specifies the Cb padding when when the padding type is fixed padding ");

    std::ostringstream complexityInfoPresentFlag;
    complexityInfoPresentFlag << "SEINNPostFilterCharacteristicsComplexityInfoPresentFlag" << i;
    opts.addOptions()(complexityInfoPresentFlag.str(),
                      encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsComplexityInfoPresentFlag[i],
                      "Specifies the value of nnpfc_complexity_info_present_flag in the Neural Network Post Filter "
                      "Characteristics SEI message");

    std::ostringstream uriTag;
    uriTag << "SEINNPostFilterCharacteristicsUriTag" << i;
    opts.addOptions()(
      uriTag.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsUriTag[i],
      "Specifies the neural network uri tag in the Neural Network Post Filter Characteristics SEI message");

    std::ostringstream uri;
    uri << "SEINNPostFilterCharacteristicsUri" << i;
    opts.addOptions()(
      uri.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsUri[i],
      "Specifies the neural network information uri in the Neural Network Post Filter Characteristics SEI message");

    std::ostringstream parameterTypeIdc;
    parameterTypeIdc << "SEINNPostFilterCharacteristicsParameterTypeIdc" << i;
    opts.addOptions()(
      parameterTypeIdc.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsParameterTypeIdc[i],
      "Specifies the data type of parameters in the Neural Network Post Filter Characteristics SEI message");

    std::ostringstream log2ParameterBitLengthMinus3;
    log2ParameterBitLengthMinus3 << "SEINNPostFilterCharacteristicsLog2ParameterBitLengthMinus3" << i;
    opts.addOptions()(
      log2ParameterBitLengthMinus3.str(),
      encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsLog2ParameterBitLengthMinus3[i],
      "Indicates that the neural network does not use parameter of bit length greater than 2^(N+3) bits");

    std::ostringstream numParametersIdc;
    numParametersIdc << "SEINNPostFilterCharacteristicsNumParametersIdc" << i;
    opts.addOptions()(numParametersIdc.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsNumParametersIdc[i],
                      "Specifies the maximum number of parameters ((2048<<NumParametersIdc)-1) in the Neural Network "
                      "Post Filter Characteristics SEI message");

    std::ostringstream numKmacOperationsIdc;
    numKmacOperationsIdc << "SEINNPostFilterCharacteristicsNumKmacOperationsIdc" << i;
    opts.addOptions()(numKmacOperationsIdc.str(),
                      encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsNumKmacOperationsIdc[i],
                      "Specifies the maximum number of operations (KMAC) per pixel in the Neural Network Post Filter "
                      "Characteristics SEI message");

    std::ostringstream totalKilobyteSize;
    totalKilobyteSize << "SEINNPostFilterCharacteristicsTotalKilobyteSize" << i;
    opts.addOptions()(totalKilobyteSize.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsTotalKilobyteSize[i],
                      "Indicates the total size in kilobytes required to store the uncompressed NN parameters in the "
                      "Neural Network Post Filter Characteristics SEI message");

    std::ostringstream payloadFilename;
    payloadFilename << "SEINNPostFilterCharacteristicsPayloadFilename" << i;
    opts.addOptions()(payloadFilename.str(), encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsPayloadFilename[i],
                      "Specifies the NNR bitstream in the Neural Network Post Filter Characteristics SEI message");

    std::ostringstream numberDecodedInputPics;
    numberDecodedInputPics << "SEINNPostFilterCharacteristicsNumberInputDecodedPicsMinusOne" << i;
    opts.addOptions()(numberDecodedInputPics.str(),
                      encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsNumberInputDecodedPicturesMinus1[i],
                      "Specifies the number of decoded output pictures used as input for the post processing filter");

    std::ostringstream numberInterpolatedPics;
    numberInterpolatedPics << "SEINNPostFilterCharacteristicsNumberInterpolatedPics" << i;
    opts.addOptions()(numberInterpolatedPics.str(), cfg_nnPostFilterSEICharacteristicsInterpolatedPicturesList[i],
                      "Number of pictures to interpolate");

    std::ostringstream InputPicOutputFlag;
    InputPicOutputFlag << "SEINNPostFilterCharacteristicsInputPicOutputFlag" << i;
    opts.addOptions()(InputPicOutputFlag.str(), cfg_nnPostFilterSEICharacteristicsInputPicOutputFlagList[i],
                      "Indicates whether NNPF will generate a corresponding output picture for the input picture");

    opts.addOptions()("SEINNPostFilterActivationEnabled", encCfg->m_seiCfg.m_nnPostFilterSEIActivationEnabled,
                      "Control use of the Neural Network Post Filter SEI on current picture");
    opts.addOptions()("SEINNPostFilterActivationTargetId", encCfg->m_seiCfg.m_nnPostFilterSEIActivationTargetId,
                      "Target id of the Neural Network Post Filter on current picture");
    opts.addOptions()(
      "SEINNPostFilterActivationCancelFlag", encCfg->m_seiCfg.m_nnPostFilterSEIActivationCancelFlag,
      "Control use of the target neural network post filter established by any previous NNPFA SEI message");
    opts.addOptions()(
      "SEINNPostFilterActivationPersistenceFlag", encCfg->m_seiCfg.m_nnPostFilterSEIActivationPersistenceFlag,
      "Specifies the persistence of the target neural-network post-processing filter for the current layer");
  }

  po::ErrorReporter              err;
  const std::list<const char *> &argv_unhandled = po::scanArgv(opts, argc, (const char **)argv, err);

  if (encCfg->m_gopBasedRPREnabledFlag)
  {
    encCfg->m_upscaledOutput = 2;
    if (encCfg->m_scalingRatioHor == 1.0 && encCfg->m_scalingRatioVer == 1.0)
    {
      encCfg->m_scalingRatioHor = 2.0;
      encCfg->m_scalingRatioVer = 2.0;
    }
  }
  encCfg->m_resChangeInClvsEnabled = encCfg->m_scalingRatioHor != 1.0 || encCfg->m_scalingRatioVer != 1.0 ||
    encCfg->m_gopBasedRPREnabledFlag || encCfg->m_rprFunctionalityTestingEnabledFlag;
  encCfg->m_resChangeInClvsEnabled = encCfg->m_resChangeInClvsEnabled && encCfg->m_rprEnabledFlag;

  if (encCfg->m_constrainedRaslEncoding)
  {
    encCfg->m_craAPSreset       = true;
    encCfg->m_rprRASLtoolSwitch = true;
  }
  else
  {
    encCfg->m_craAPSreset       = false;
    encCfg->m_rprRASLtoolSwitch = false;
  }

  if (encCfg->m_fractionOfFrames != 1.0)
  {
    encCfg->m_framesToBeEncoded = int(encCfg->m_framesToBeEncoded * encCfg->m_fractionOfFrames);
  }

  if (encCfg->m_resChangeInClvsEnabled && !encCfg->m_switchPocPeriod)
  {
    encCfg->m_switchPocPeriod = encCfg->m_frameRate / 2 / encCfg->m_gopSize * encCfg->m_gopSize;
  }

  // Check the given value of intra period and decoding refresh type. If intra period is -1, set decoding refresh type
  // to be equal to 0. And vice versa
  if (encCfg->m_intraPeriod == -1)
  {
    encCfg->m_decodingRefreshType = 0;
  }
  if (!encCfg->m_decodingRefreshType)
  {
    encCfg->m_intraPeriod = -1;
  }

  encCfg->m_seiCfg.m_bpDeltasGOPStructure = false;
  if (encCfg->m_gopSize == 16)
  {
    if ((encCfg->m_GOPList[0].m_POC == 16 && encCfg->m_GOPList[0].m_temporalId == 0) &&
        (encCfg->m_GOPList[1].m_POC == 8 && encCfg->m_GOPList[1].m_temporalId == 1) &&
        (encCfg->m_GOPList[2].m_POC == 4 && encCfg->m_GOPList[2].m_temporalId == 2) &&
        (encCfg->m_GOPList[3].m_POC == 2 && encCfg->m_GOPList[3].m_temporalId == 3) &&
        (encCfg->m_GOPList[4].m_POC == 1 && encCfg->m_GOPList[4].m_temporalId == 4) &&
        (encCfg->m_GOPList[5].m_POC == 3 && encCfg->m_GOPList[5].m_temporalId == 4) &&
        (encCfg->m_GOPList[6].m_POC == 6 && encCfg->m_GOPList[6].m_temporalId == 3) &&
        (encCfg->m_GOPList[7].m_POC == 5 && encCfg->m_GOPList[7].m_temporalId == 4) &&
        (encCfg->m_GOPList[8].m_POC == 7 && encCfg->m_GOPList[8].m_temporalId == 4) &&
        (encCfg->m_GOPList[9].m_POC == 12 && encCfg->m_GOPList[9].m_temporalId == 2) &&
        (encCfg->m_GOPList[10].m_POC == 10 && encCfg->m_GOPList[10].m_temporalId == 3) &&
        (encCfg->m_GOPList[11].m_POC == 9 && encCfg->m_GOPList[11].m_temporalId == 4) &&
        (encCfg->m_GOPList[12].m_POC == 11 && encCfg->m_GOPList[12].m_temporalId == 4) &&
        (encCfg->m_GOPList[13].m_POC == 14 && encCfg->m_GOPList[13].m_temporalId == 3) &&
        (encCfg->m_GOPList[14].m_POC == 13 && encCfg->m_GOPList[14].m_temporalId == 4) &&
        (encCfg->m_GOPList[15].m_POC == 15 && encCfg->m_GOPList[15].m_temporalId == 4))
    {
      encCfg->m_seiCfg.m_bpDeltasGOPStructure = true;
    }
  }
  else if (encCfg->m_gopSize == 8)
  {
    if ((encCfg->m_GOPList[0].m_POC == 8 && encCfg->m_GOPList[0].m_temporalId == 0) &&
        (encCfg->m_GOPList[1].m_POC == 4 && encCfg->m_GOPList[1].m_temporalId == 1) &&
        (encCfg->m_GOPList[2].m_POC == 2 && encCfg->m_GOPList[2].m_temporalId == 2) &&
        (encCfg->m_GOPList[3].m_POC == 1 && encCfg->m_GOPList[3].m_temporalId == 3) &&
        (encCfg->m_GOPList[4].m_POC == 3 && encCfg->m_GOPList[4].m_temporalId == 3) &&
        (encCfg->m_GOPList[5].m_POC == 6 && encCfg->m_GOPList[5].m_temporalId == 2) &&
        (encCfg->m_GOPList[6].m_POC == 5 && encCfg->m_GOPList[6].m_temporalId == 3) &&
        (encCfg->m_GOPList[7].m_POC == 7 && encCfg->m_GOPList[7].m_temporalId == 3))
    {
      encCfg->m_seiCfg.m_bpDeltasGOPStructure = true;
    }
  }
  else
  {
    encCfg->m_seiCfg.m_bpDeltasGOPStructure = false;
  }
  for (int i = 0; encCfg->m_GOPList[i].m_POC != -1 && i < MAX_GOP + 1; i++)
  {
    encCfg->m_RPLList0[i].m_POC = encCfg->m_RPLList1[i].m_POC = encCfg->m_GOPList[i].m_POC;
    encCfg->m_RPLList0[i].m_temporalId = encCfg->m_RPLList1[i].m_temporalId = encCfg->m_GOPList[i].m_temporalId;
    encCfg->m_RPLList0[i].m_refPic = encCfg->m_RPLList1[i].m_refPic = encCfg->m_GOPList[i].m_refPic;
    encCfg->m_RPLList0[i].m_sliceType = encCfg->m_RPLList1[i].m_sliceType = encCfg->m_GOPList[i].m_sliceType;
    encCfg->m_RPLList0[i].m_isEncoded = encCfg->m_RPLList1[i].m_isEncoded = encCfg->m_GOPList[i].m_isEncoded;

    encCfg->m_RPLList0[i].m_numRefPicsActive      = encCfg->m_GOPList[i].m_numRefPicsActive0;
    encCfg->m_RPLList1[i].m_numRefPicsActive      = encCfg->m_GOPList[i].m_numRefPicsActive1;
    encCfg->m_RPLList0[i].m_numRefPics            = encCfg->m_GOPList[i].m_numRefPics0;
    encCfg->m_RPLList1[i].m_numRefPics            = encCfg->m_GOPList[i].m_numRefPics1;
    encCfg->m_RPLList0[i].m_ltrpInSliceHeaderFlag = encCfg->m_GOPList[i].m_ltrpInSliceHeaderFlag;
    encCfg->m_RPLList1[i].m_ltrpInSliceHeaderFlag = encCfg->m_GOPList[i].m_ltrpInSliceHeaderFlag;
    for (int j = 0; j < encCfg->m_GOPList[i].m_numRefPics0; j++)
    {
      encCfg->m_RPLList0[i].m_deltaRefPics[j] = encCfg->m_GOPList[i].m_deltaRefPics0[j];
    }
    for (int j = 0; j < encCfg->m_GOPList[i].m_numRefPics1; j++)
    {
      encCfg->m_RPLList1[i].m_deltaRefPics[j] = encCfg->m_GOPList[i].m_deltaRefPics1[j];
    }
  }

  if (encCfg->m_compositeRefEnabled)
  {
    for (int i = 0; i < encCfg->m_gopSize; i++)
    {
      encCfg->m_GOPList[i].m_POC *= 2;
      encCfg->m_RPLList0[i].m_POC *= 2;
      encCfg->m_RPLList1[i].m_POC *= 2;
      for (int j = 0; j < encCfg->m_RPLList0[i].m_numRefPics; j++)
      {
        encCfg->m_RPLList0[i].m_deltaRefPics[j] *= 2;
      }
      for (int j = 0; j < encCfg->m_RPLList1[i].m_numRefPics; j++)
      {
        encCfg->m_RPLList1[i].m_deltaRefPics[j] *= 2;
      }
    }
  }

  if (encCfg->m_interMTSMaxSize != 32 && encCfg->m_interMTSMaxSize != 16)
  {
    encCfg->m_interMTSMaxSize = (encCfg->m_sourceHeight > 1080 ? 32 : 16);
  }

  for (std::list<const char *>::const_iterator it = argv_unhandled.begin(); it != argv_unhandled.end(); it++)
  {
    msg(ERROR, "Unhandled argument ignored: `%s'\n", *it);
  }

  if (argc == 1 || do_help)
  {
    /* argc == 1: no options have been specified */
    po::doHelp(std::cout, opts);
    return false;
  }

  if (err.is_errored)
  {
    if (!warnUnknowParameter)
    {
      /* error report has already been printed on stderr */
      return false;
    }
  }

  g_verbosity = MsgLevel(encCfg->m_verbosity);

  /*
   * Set any derived parameters
   */

  if (encCfg->m_sourceScalingRatioHor != 1.0 || encCfg->m_sourceScalingRatioVer != 1.0)
  {
    encCfg->m_sourceWidthBeforeScale  = encCfg->m_sourceWidth;
    encCfg->m_sourceHeightBeforeScale = encCfg->m_sourceHeight;
    encCfg->m_sourceWidth             = int(round(encCfg->m_sourceWidth * encCfg->m_sourceScalingRatioHor));
    encCfg->m_sourceHeight            = int(round(encCfg->m_sourceHeight * encCfg->m_sourceScalingRatioVer));
  }
  else
  {
    encCfg->m_sourceWidthBeforeScale  = 0;
    encCfg->m_sourceHeightBeforeScale = 0;
  }
#if EXTENSION_360_VIDEO
  encCfg->m_inputFileWidth  = encCfg->m_sourceWidth;
  encCfg->m_inputFileHeight = encCfg->m_sourceHeight;
  encCfg->m_ext360.setMaxCUInfo(encCfg->m_CTUSize, 1 << MIN_CU_LOG2);
#endif

  if (!inputPathPrefix.empty() && inputPathPrefix.back() != '/' && inputPathPrefix.back() != '\\')
  {
    inputPathPrefix += "/";
  }
  encCfg->m_inputFileName = inputPathPrefix + encCfg->m_inputFileName;

  if (encCfg->m_firstValidFrame < 0)
  {
    encCfg->m_firstValidFrame = encCfg->m_frameSkip;
  }
  if (encCfg->m_lastValidFrame < 0)
  {
    encCfg->m_lastValidFrame = encCfg->m_firstValidFrame + encCfg->m_framesToBeEncoded - 1;
  }

  if (encCfg->m_temporalSubsampleRatio < 1)
  {
    EXIT("Error: TemporalSubsampleRatio must be greater than 0");
  }

  if (encCfg->m_log2SignPredArea < 0)   // set based on AI/RA/LD, resolution, and QP
  {
    if (encCfg->m_intraPeriod == 1)
    {
      if (encCfg->m_iQP > 22 && encCfg->m_iQP < 37)
      {
        encCfg->m_log2SignPredArea = 3;
      }
      else
      {
        encCfg->m_log2SignPredArea = 2;
      }
    }
    else if (encCfg->m_intraPeriod == -1)
    {
      if (encCfg->m_sourceHeight >= 1080)
      {
        encCfg->m_log2SignPredArea = (encCfg->m_iQP >= 32) ? 5 : 3;
      }
      else if (encCfg->m_sourceHeight >= 720)
      {
        encCfg->m_log2SignPredArea = (encCfg->m_iQP > 32) ? 5 : 4;
      }
      else if (encCfg->m_sourceHeight >= 480)
      {
        encCfg->m_log2SignPredArea = 4;
      }
      else
      {
        encCfg->m_log2SignPredArea = 3;
      }
      if (encCfg->m_ibcMode)
      {
        encCfg->m_log2SignPredArea = 3;
      }
    }
    else
    {
      encCfg->m_log2SignPredArea = 4;
    }
    encCfg->m_log2SignPredArea = std::min<int>(encCfg->m_log2SignPredArea, MAX_LOG2_SIGN_PRED_SIZE);
  }

  encCfg->m_framesToBeEncoded =
    (encCfg->m_framesToBeEncoded + encCfg->m_temporalSubsampleRatio - 1) / encCfg->m_temporalSubsampleRatio;
  encCfg->m_adIntraLambdaModifier = cfg_adIntraLambdaModifier.values;
  if (encCfg->m_fieldSeqFlag)
  {
    // Frame height
    encCfg->m_iSourceHeightOrg = encCfg->m_sourceHeight;
    // Field height
    encCfg->m_sourceHeight     = encCfg->m_sourceHeight >> 1;
    // number of fields to encode
    encCfg->m_framesToBeEncoded *= 2;
  }
  if (encCfg->m_subPicInfoPresentFlag)
  {
    CHECK(encCfg->m_numSubPics > MAX_NUM_SUB_PICS || encCfg->m_numSubPics < 1,
          "Number of subpicture must be within 1 to 2^16")
    if (!encCfg->m_subPicSameSizeFlag)
    {
      CHECK(cfg_subPicCtuTopLeftX.values.size() != encCfg->m_numSubPics,
            "Number of SubPicCtuTopLeftX values must be equal to NumSubPics");
      CHECK(cfg_subPicCtuTopLeftY.values.size() != encCfg->m_numSubPics,
            "Number of SubPicCtuTopLeftY values must be equal to NumSubPics");
      CHECK(cfg_subPicWidth.values.size() != encCfg->m_numSubPics,
            "Number of SubPicWidth values must be equal to NumSubPics");
      CHECK(cfg_subPicHeight.values.size() != encCfg->m_numSubPics,
            "Number of SubPicHeight values must be equal to NumSubPics");
    }
    else
    {
      CHECK(cfg_subPicCtuTopLeftX.values.size() != 0, "Number of SubPicCtuTopLeftX values must be equal to 0");
      CHECK(cfg_subPicCtuTopLeftY.values.size() != 0, "Number of SubPicCtuTopLeftY values must be equal to 0");
      CHECK(cfg_subPicWidth.values.size() != 1, "Number of SubPicWidth values must be equal to 1");
      CHECK(cfg_subPicHeight.values.size() != 1, "Number of SubPicHeight values must be equal to 1");
    }
    CHECK(cfg_subPicTreatedAsPicFlag.values.size() != encCfg->m_numSubPics,
          "Number of SubPicTreatedAsPicFlag values must be equal to NumSubPics");
    CHECK(cfg_loopFilterAcrossSubpicEnabledFlag.values.size() != encCfg->m_numSubPics,
          "Number of LoopFilterAcrossSubpicEnabledFlag values must be equal to NumSubPics");
    if (encCfg->m_subPicIdMappingExplicitlySignalledFlag)
    {
      CHECK(cfg_subPicId.values.size() != encCfg->m_numSubPics,
            "Number of SubPicId values must be equal to NumSubPics");
    }
    encCfg->m_subPicCtuTopLeftX                 = cfg_subPicCtuTopLeftX.values;
    encCfg->m_subPicCtuTopLeftY                 = cfg_subPicCtuTopLeftY.values;
    encCfg->m_subPicWidth                       = cfg_subPicWidth.values;
    encCfg->m_subPicHeight                      = cfg_subPicHeight.values;
    encCfg->m_subPicTreatedAsPicFlag            = cfg_subPicTreatedAsPicFlag.values;
    encCfg->m_loopFilterAcrossSubpicEnabledFlag = cfg_loopFilterAcrossSubpicEnabledFlag.values;
    if (encCfg->m_subPicIdMappingExplicitlySignalledFlag)
    {
      for (int i = 0; i < encCfg->m_numSubPics; i++)
      {
        encCfg->m_subPicId[i] = cfg_subPicId.values[i];
      }
    }
    uint32_t tmpWidthVal  = (encCfg->m_sourceWidth + encCfg->m_CTUSize - 1) / encCfg->m_CTUSize;
    uint32_t tmpHeightVal = (encCfg->m_sourceHeight + encCfg->m_CTUSize - 1) / encCfg->m_CTUSize;
    if (!encCfg->m_subPicSameSizeFlag)
    {
      for (int i = 0; i < encCfg->m_numSubPics; i++)
      {
        CHECK(encCfg->m_subPicCtuTopLeftX[i] + encCfg->m_subPicWidth[i] > tmpWidthVal,
              "Subpicture must not exceed picture boundary");
        CHECK(encCfg->m_subPicCtuTopLeftY[i] + encCfg->m_subPicHeight[i] > tmpHeightVal,
              "Subpicture must not exceed picture boundary");
      }
    }
    else
    {
      uint32_t numSubpicCols = tmpWidthVal / encCfg->m_subPicWidth[0];
      CHECK(tmpWidthVal % encCfg->m_subPicWidth[0] != 0, "sps_subpic_width_minus1[0] is invalid.");
      CHECK(tmpHeightVal % encCfg->m_subPicHeight[0] != 0, "sps_subpic_height_minus1[0] is invalid.");
      CHECK(numSubpicCols * (tmpHeightVal / encCfg->m_subPicHeight[0]) != encCfg->m_numSubPics,
            "when sps_subpic_same_size_flag is equal to, sps_num_subpics_minus1 is invalid");
    }
    // automatically determine subpicture ID lenght in case it is not specified
    if (encCfg->m_subPicIdLen == 0)
    {
      if (encCfg->m_subPicIdMappingExplicitlySignalledFlag)
      {
        // use the heighest specified ID
        auto maxIdVal         = std::max_element(encCfg->m_subPicId.begin(), encCfg->m_subPicId.end());
        encCfg->m_subPicIdLen = ceilLog2(*maxIdVal);
      }
      else
      {
        // use the number of subpictures
        encCfg->m_subPicIdLen = ceilLog2(encCfg->m_numSubPics);
      }
    }

    CHECK(encCfg->m_subPicIdLen > 16, "SubPicIdLen must not exceed 16 bits");
    CHECK(encCfg->m_resChangeInClvsEnabled, "resolution change in CLVS and subpictures cannot be enabled together");
  }

  if (encCfg->m_seiCfg.m_cfgSubpictureLevelInfoSEI.m_enabled)
  {
    CHECK(encCfg->m_numSubPics != encCfg->m_seiCfg.m_cfgSubpictureLevelInfoSEI.m_numSubpictures,
          "NumSubPics must be equal to SEISubpicLevelInfoNumSubpics");
    CHECK(encCfg->m_seiCfg.m_cfgSubpictureLevelInfoSEI.m_sliMaxSublayers != encCfg->m_maxSublayers,
          "SEISubpicLevelInfoMaxSublayers must be equal to vps_max_sublayers");
    if (encCfg->m_seiCfg.m_cfgSubpictureLevelInfoSEI.m_sliSublayerInfoPresentFlag)
    {
      CHECK(cfg_sliRefLevels.values.size() < encCfg->m_maxSublayers,
            "when sliSublayerInfoPresentFlag = 1, the number of reference levels must be greater than or equal to "
            "sublayers");
    }
    if (encCfg->m_seiCfg.m_cfgSubpictureLevelInfoSEI.m_explicitFraction)
    {
      encCfg->m_seiCfg.m_cfgSubpictureLevelInfoSEI.m_fractions = cfg_sliFractions.values;
      encCfg->m_seiCfg.m_cfgSubpictureLevelInfoSEI.m_refLevels = cfg_sliRefLevels.values;
      if (encCfg->m_seiCfg.m_cfgSubpictureLevelInfoSEI.m_sliSublayerInfoPresentFlag)
      {
        CHECK((int)cfg_sliRefLevels.values.size() / encCfg->m_maxSublayers *
                  encCfg->m_seiCfg.m_cfgSubpictureLevelInfoSEI.m_numSubpictures *
                  encCfg->m_seiCfg.m_cfgSubpictureLevelInfoSEI.m_sliMaxSublayers !=
                cfg_sliFractions.values.size(),
              "when sliSublayerInfoPresentFlag = 1, the number  of subpicture level fractions must be equal to the "
              "numer of subpictures times the number of reference levels times the number of sublayers");
      }
      else
      {
        CHECK((int)cfg_sliRefLevels.values.size() * encCfg->m_seiCfg.m_cfgSubpictureLevelInfoSEI.m_numSubpictures !=
                cfg_sliFractions.values.size(),
              "when sliSublayerInfoPresentFlag = 0, the number  of subpicture level fractions must be equal to the "
              "numer of subpictures times the number of reference levels");
      }
    }
    encCfg->m_seiCfg.m_cfgSubpictureLevelInfoSEI.m_nonSubpicLayersFraction = cfg_sliNonSubpicLayersFractions.values;
    if (encCfg->m_seiCfg.m_cfgSubpictureLevelInfoSEI.m_sliSublayerInfoPresentFlag)
    {
      CHECK((int)cfg_sliNonSubpicLayersFractions.values.size() !=
              (cfg_sliRefLevels.values.size() * encCfg->m_seiCfg.m_cfgSubpictureLevelInfoSEI.m_numSubpictures),
            "when sliSublayerInfoPresentFlag = 1, the number  of non-subpicture level fractions must be equal to the "
            "numer of reference levels times the number of sublayers");
    }
    else
    {
      CHECK((int)cfg_sliNonSubpicLayersFractions.values.size() != (cfg_sliRefLevels.values.size()),
            "when sliSublayerInfoPresentFlag = 0, the number  of non-subpicture level fractions must be equal to the "
            "numer of reference levels");
    }
  }

  if (encCfg->m_costMode != COST_LOSSLESS_CODING && encCfg->m_mixedLossyLossless)
  {
    encCfg->m_mixedLossyLossless = 0;
    msg(WARNING, "*************************************************************************\n");
    msg(WARNING, "* Mixed lossy lossles coding cannot enable in lossy costMode *\n");
    msg(WARNING, "* Forcely disabled  m_mixedLossyLossless *\n");
    msg(WARNING, "*************************************************************************\n");
  }
  if (!encCfg->m_mixedLossyLossless && cfgSliceLosslessArray.values.size() > 0)
  {
    msg(WARNING, "*************************************************************************\n");
    msg(WARNING, "* Mixed lossy lossles coding is not enabled *\n");
    msg(WARNING, "* ignoring the value of SliceLosslessArray *\n");
    msg(WARNING, "*************************************************************************\n");
  }

  if (encCfg->m_costMode == COST_LOSSLESS_CODING && encCfg->m_mixedLossyLossless)
  {
    encCfg->m_sliceLosslessArray.resize(cfgSliceLosslessArray.values.size());
    for (uint32_t i = 0; i < cfgSliceLosslessArray.values.size(); i++)
    {
      encCfg->m_sliceLosslessArray[i] = cfgSliceLosslessArray.values[i];
    }
  }

  if (encCfg->m_picPartitionFlag)
  {
    // store tile column widths
    encCfg->m_tileColumnWidth.resize(cfgTileColumnWidth.values.size());
    for (uint32_t i = 0; i < cfgTileColumnWidth.values.size(); i++)
    {
      encCfg->m_tileColumnWidth[i] = cfgTileColumnWidth.values[i];
    }

    // store tile row heights
    encCfg->m_tileRowHeight.resize(cfgTileRowHeight.values.size());
    for (uint32_t i = 0; i < cfgTileRowHeight.values.size(); i++)
    {
      encCfg->m_tileRowHeight[i] = cfgTileRowHeight.values[i];
    }

    // store rectangular slice positions
    if (!encCfg->m_rasterSliceFlag)
    {
      encCfg->m_rectSlicePos.resize(cfgRectSlicePos.values.size());
      for (uint32_t i = 0; i < cfgRectSlicePos.values.size(); i++)
      {
        encCfg->m_rectSlicePos[i] = cfgRectSlicePos.values[i];
      }
    }

    // store raster-scan slice sizes
    else
    {
      encCfg->m_rasterSliceSize.resize(cfgRasterSliceSize.values.size());
      for (uint32_t i = 0; i < cfgRasterSliceSize.values.size(); i++)
      {
        encCfg->m_rasterSliceSize[i] = cfgRasterSliceSize.values[i];
      }
    }
  }
  else
  {
    encCfg->m_tileColumnWidth.clear();
    encCfg->m_tileRowHeight.clear();
    encCfg->m_rectSlicePos.clear();
    encCfg->m_rasterSliceSize.clear();
    encCfg->m_rectSliceFixedWidth  = 0;
    encCfg->m_rectSliceFixedHeight = 0;
  }

  encCfg->m_numSubProfile = (uint8_t)cfg_SubProfile.values.size();
  encCfg->m_subProfile.resize(encCfg->m_numSubProfile);
  for (uint8_t i = 0; i < encCfg->m_numSubProfile; ++i)
  {
    encCfg->m_subProfile[i] = cfg_SubProfile.values[i];
  }
  /* rules for input, output and internal bitdepths as per help text */
  if (encCfg->m_msbExtendedBitDepth[ChannelType::LUMA] == 0)
  {
    encCfg->m_msbExtendedBitDepth[ChannelType::LUMA] = encCfg->m_inputBitDepth[ChannelType::LUMA];
  }
  if (encCfg->m_msbExtendedBitDepth[ChannelType::CHROMA] == 0)
  {
    encCfg->m_msbExtendedBitDepth[ChannelType::CHROMA] = encCfg->m_msbExtendedBitDepth[ChannelType::LUMA];
  }
  if (encCfg->m_internalBitDepth[ChannelType::LUMA] == 0)
  {
    encCfg->m_internalBitDepth[ChannelType::LUMA] = encCfg->m_msbExtendedBitDepth[ChannelType::LUMA];
  }
  encCfg->m_internalBitDepth[ChannelType::CHROMA] = encCfg->m_internalBitDepth[ChannelType::LUMA];
  if (encCfg->m_inputBitDepth[ChannelType::CHROMA] == 0)
  {
    encCfg->m_inputBitDepth[ChannelType::CHROMA] = encCfg->m_inputBitDepth[ChannelType::LUMA];
  }
  if (encCfg->m_outputBitDepth[ChannelType::LUMA] == 0)
  {
    encCfg->m_outputBitDepth[ChannelType::LUMA] = encCfg->m_internalBitDepth[ChannelType::LUMA];
  }
  if (encCfg->m_outputBitDepth[ChannelType::CHROMA] == 0)
  {
    encCfg->m_outputBitDepth[ChannelType::CHROMA] = encCfg->m_outputBitDepth[ChannelType::LUMA];
  }

  encCfg->m_inputChromaFormatIDC = numberToChromaFormat(tmpInputChromaFormat);
  encCfg->m_chromaFormatIdc =
    ((tmpChromaFormat == 0) ? (encCfg->m_inputChromaFormatIDC) : (numberToChromaFormat(tmpChromaFormat)));
#if EXTENSION_360_VIDEO
  encCfg->m_ext360.processOptions(ext360CfgContext);
#endif

  for (int i = 0; i < MAX_NUM_NN_POST_FILTERS; ++i)
  {
    encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsOutColourFormatIdc[i] =
      (ChromaFormat)tmpNnPostFilterSEICharacteristicsOutColourFormatIdc[i];
    encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsNumberInterpolatedPictures[i] =
      cfg_nnPostFilterSEICharacteristicsInterpolatedPicturesList[i].values;
    if (encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsNumberInterpolatedPictures[i].size() == 0)
    {
      encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsNumberInterpolatedPictures[i].push_back(0);
    }

    for (int j = 0; j < encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsNumberInterpolatedPictures[i].size(); ++j)
    {
      CHECK(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsNumberInterpolatedPictures[i][j] > 63,
            "The value of nnpfc_interpolated_pics[i] shall be in the range of 0 to 63, inclusive");
    }
    CHECK(int(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsNumberInterpolatedPictures[i].size()) <
            int(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsNumberInputDecodedPicturesMinus1[i]) - 1,
          "Number Interpolated Pictures List must be greater than number of decoder pictures list");

    encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsInputPicOutputFlag[i] =
      cfg_nnPostFilterSEICharacteristicsInputPicOutputFlagList[i].values;
    if (encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsInputPicOutputFlag[i].size() == 0)
    {
      encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsInputPicOutputFlag[i].push_back(0);
    }
  }

  if (isY4mFileExt(encCfg->m_inputFileName))
  {
    int          width = 0, height = 0, frameRate = 0, inputBitDepth = 0;
    ChromaFormat chromaFormat = ChromaFormat::_420;
    VideoIOYuv   inputFile;
    inputFile.parseY4mFileHeader(encCfg->m_inputFileName, width, height, frameRate, inputBitDepth, chromaFormat);
    if (width != encCfg->m_sourceWidth || height != encCfg->m_sourceHeight || frameRate != encCfg->m_frameRate ||
        inputBitDepth != encCfg->m_inputBitDepth[ChannelType::LUMA] || chromaFormat != encCfg->m_chromaFormatIdc)
    {
      msg(WARNING, "\nWarning: Y4M file info is different from input setting. Using the info from Y4M file\n");
      encCfg->m_sourceWidth  = width;
      encCfg->m_sourceHeight = height;
      encCfg->m_frameRate    = frameRate;
      encCfg->m_inputBitDepth.fill(inputBitDepth);
      encCfg->m_chromaFormatIdc     = chromaFormat;
      encCfg->m_msbExtendedBitDepth = encCfg->m_inputBitDepth;
    }
  }

  CHECK(!(tmpWeightedPredictionMethod >= 0 &&
          tmpWeightedPredictionMethod <= WP_PER_PICTURE_WITH_HISTOGRAM_AND_PER_COMP_AND_CLIPPING_AND_EXTENSION),
        "Error in cfg");
  encCfg->m_weightedPredictionMethod = WeightedPredictionMethod(tmpWeightedPredictionMethod);

  CHECK(tmpFastInterSearchMode < 0 || tmpFastInterSearchMode > FASTINTERSEARCH_MODE3, "Error in cfg");
  encCfg->m_fastInterSearchMode = FastInterSearchMode(tmpFastInterSearchMode);

  CHECK(tmpMotionEstimationSearchMethod < to_underlying(MESearchMethod::FULL) ||
          tmpMotionEstimationSearchMethod >= to_underlying(MESearchMethod::NUM),
        "Error in cfg");
  encCfg->m_motionEstimationSearchMethod = MESearchMethod(tmpMotionEstimationSearchMethod);

  if (extendedProfile == ExtendedProfileName::AUTO)
  {
    if (autoDetermineProfile(encCfg))
    {
      EXIT("Unable to determine profile from configured settings");
    }
  }
  else
  {
    switch (extendedProfile)
    {
    case ExtendedProfileName::NONE:
      encCfg->m_profile = Profile::NONE;
      break;
    case ExtendedProfileName::MAIN_10:
      encCfg->m_profile = Profile::MAIN_10;
      break;
    case ExtendedProfileName::MAIN_10_444:
      encCfg->m_profile = Profile::MAIN_10_444;
      break;
    case ExtendedProfileName::MAIN_10_STILL_PICTURE:
      encCfg->m_profile = Profile::MAIN_10_STILL_PICTURE;
      break;
    case ExtendedProfileName::MAIN_10_444_STILL_PICTURE:
      encCfg->m_profile = Profile::MAIN_10_444_STILL_PICTURE;
      break;
    case ExtendedProfileName::MULTILAYER_MAIN_10:
      encCfg->m_profile = Profile::MULTILAYER_MAIN_10;
      break;
    case ExtendedProfileName::MULTILAYER_MAIN_10_444:
      encCfg->m_profile = Profile::MULTILAYER_MAIN_10_444;
      break;
    case ExtendedProfileName::MULTILAYER_MAIN_10_STILL_PICTURE:
      encCfg->m_profile = Profile::MULTILAYER_MAIN_10_STILL_PICTURE;
      break;
    case ExtendedProfileName::MULTILAYER_MAIN_10_444_STILL_PICTURE:
      encCfg->m_profile = Profile::MULTILAYER_MAIN_10_444_STILL_PICTURE;
      break;
    case ExtendedProfileName::MAIN_12:
      encCfg->m_profile = Profile::MAIN_12;
      break;
    case ExtendedProfileName::MAIN_12_444:
      encCfg->m_profile = Profile::MAIN_12_444;
      break;
    case ExtendedProfileName::MAIN_16_444:
      encCfg->m_profile = Profile::MAIN_16_444;
      break;
    case ExtendedProfileName::MAIN_12_INTRA:
      encCfg->m_profile = Profile::MAIN_12_INTRA;
      break;
    case ExtendedProfileName::MAIN_12_444_INTRA:
      encCfg->m_profile = Profile::MAIN_12_444_INTRA;
      break;
    case ExtendedProfileName::MAIN_16_444_INTRA:
      encCfg->m_profile = Profile::MAIN_16_444_INTRA;
      break;
    case ExtendedProfileName::MAIN_12_STILL_PICTURE:
      encCfg->m_profile = Profile::MAIN_12_STILL_PICTURE;
      break;
    case ExtendedProfileName::MAIN_12_444_STILL_PICTURE:
      encCfg->m_profile = Profile::MAIN_12_444_STILL_PICTURE;
      break;
    case ExtendedProfileName::MAIN_16_444_STILL_PICTURE:
      encCfg->m_profile = Profile::MAIN_16_444_STILL_PICTURE;
      break;
    default:
      EXIT("Unable to determine profile from configured settings");
      break;
    }
  }

  {
    encCfg->m_chromaFormatConstraint =
      (tmpConstraintChromaFormat == 0) ? encCfg->m_chromaFormatIdc : numberToChromaFormat(tmpConstraintChromaFormat);
    encCfg->m_maxChromaFormatConstraintIdc = static_cast<ChromaFormat>(tmpMaxChromaFormatConstraintIdc);

    if (encCfg->m_bitDepthConstraint == 0)
    {
      if (encCfg->m_profile != Profile::NONE)
      {
        const ProfileFeatures *features = ProfileFeatures::getProfileFeatures(encCfg->m_profile);
        CHECK(features->profile != encCfg->m_profile, "Profile not found");
        encCfg->m_bitDepthConstraint = features->maxBitDepth;
      }
      else   // encCfg->m_profile == Profile::NONE
      {
        encCfg->m_bitDepthConstraint = 16;   // max value - unconstrained.
      }
    }
    CHECK(encCfg->m_bitDepthConstraint < encCfg->m_internalBitDepth[ChannelType::LUMA],
          "MaxBitDepthConstraint setting does not allow the specified luma bit depth to be coded.");
    CHECK(encCfg->m_bitDepthConstraint < encCfg->m_internalBitDepth[ChannelType::CHROMA],
          "MaxBitDepthConstraint setting does not allow the specified chroma bit depth to be coded.");
    CHECK(encCfg->m_chromaFormatConstraint < encCfg->m_chromaFormatIdc,
          "MaxChromaFormatConstraint setting does not allow the specified chroma format to be coded.");
    CHECK(encCfg->m_chromaFormatConstraint >= ChromaFormat::NUM,
          "Bad value given for MaxChromaFormatConstraint setting.")
    CHECK(encCfg->m_bitDepthConstraint < 8 || encCfg->m_bitDepthConstraint > 16,
          "MaxBitDepthConstraint setting must be in the range 8 to 16 (inclusive)");
  }

  encCfg->m_inputColourSpaceConvert = stringToInputColourSpaceConvert(inputColourSpaceConvert, true);
  encCfg->m_rgbFormat =
    encCfg->m_inputColourSpaceConvert == IPCOLOURSPACE_RGBtoGBR && encCfg->m_chromaFormatIdc == ChromaFormat::_444;
  if (encCfg->m_profile == Profile::MAIN_12 || encCfg->m_profile == Profile::MAIN_12_INTRA ||
      encCfg->m_profile == Profile::MAIN_12_STILL_PICTURE || encCfg->m_profile == Profile::MAIN_12_444 ||
      encCfg->m_profile == Profile::MAIN_12_444_INTRA || encCfg->m_profile == Profile::MAIN_12_444_STILL_PICTURE ||
      encCfg->m_profile == Profile::MAIN_16_444 || encCfg->m_profile == Profile::MAIN_16_444_INTRA ||
      encCfg->m_profile == Profile::MAIN_16_444_STILL_PICTURE)
  {
    encCfg->m_gciPresentFlag = true;
  }
  if (encCfg->m_profile == Profile::MAIN_12_INTRA || encCfg->m_profile == Profile::MAIN_12_444_INTRA ||
      encCfg->m_profile == Profile::MAIN_16_444_INTRA)
  {
    CHECK(encCfg->m_intraPeriod != 1, "IntraPeriod setting must be 1 for Intra profiles")
  }
  if (encCfg->m_profile == Profile::MULTILAYER_MAIN_10_STILL_PICTURE ||
      encCfg->m_profile == Profile::MAIN_10_STILL_PICTURE || encCfg->m_profile == Profile::MAIN_12_STILL_PICTURE ||
      encCfg->m_profile == Profile::MAIN_12_444_STILL_PICTURE ||
      encCfg->m_profile == Profile::MAIN_16_444_STILL_PICTURE)
  {
    CHECK(encCfg->m_framesToBeEncoded != 1, "FramesToBeEncoded setting must be 1 for Still Picture profiles")
  }

  // Picture width and height must be multiples of 8 and minCuSize
  const int minResolutionMultiple = std::max(8, 1 << encCfg->m_log2MinCUSize);

  switch (encCfg->m_conformanceWindowMode)
  {
  case 0:
    {
      // no conformance or padding
      encCfg->m_confWinLeft = encCfg->m_confWinRight = encCfg->m_confWinTop = encCfg->m_confWinBottom = 0;
      encCfg->m_sourcePadding[1] = encCfg->m_sourcePadding[0] = 0;
      break;
    }
  case 1:
    {
      // automatic padding to minimum CU size
      if (encCfg->m_sourceWidth % minResolutionMultiple)
      {
        encCfg->m_sourcePadding[0] = encCfg->m_confWinRight =
          ((encCfg->m_sourceWidth / minResolutionMultiple) + 1) * minResolutionMultiple - encCfg->m_sourceWidth;
        encCfg->m_sourceWidth += encCfg->m_confWinRight;
      }
      if (encCfg->m_sourceHeight % minResolutionMultiple)
      {
        encCfg->m_sourcePadding[1] = encCfg->m_confWinBottom =
          ((encCfg->m_sourceHeight / minResolutionMultiple) + 1) * minResolutionMultiple - encCfg->m_sourceHeight;
        encCfg->m_sourceHeight += encCfg->m_confWinBottom;
        if (encCfg->m_fieldSeqFlag)
        {
          encCfg->m_iSourceHeightOrg += encCfg->m_confWinBottom << 1;
          encCfg->m_sourcePadding[1] = encCfg->m_confWinBottom << 1;
        }
      }
      if (encCfg->m_sourcePadding[0] % SPS::getWinUnitX(encCfg->m_chromaFormatIdc) != 0)
      {
        EXIT("Error: picture width is not an integer multiple of the specified chroma subsampling");
      }
      if (encCfg->m_sourcePadding[1] % SPS::getWinUnitY(encCfg->m_chromaFormatIdc) != 0)
      {
        EXIT("Error: picture height is not an integer multiple of the specified chroma subsampling");
      }
      if (encCfg->m_sourcePadding[0])
      {
        msg(INFO, "Info: Conformance window automatically enabled. Adding %i lumal pel horizontally\n",
            encCfg->m_sourcePadding[0]);
      }
      if (encCfg->m_sourcePadding[1])
      {
        msg(INFO, "Info: Conformance window automatically enabled. Adding %i lumal pel vertically\n",
            encCfg->m_sourcePadding[1]);
      }
      break;
    }
  case 2:
    {
      // padding
      encCfg->m_sourceWidth += encCfg->m_sourcePadding[0];
      encCfg->m_sourceHeight += encCfg->m_sourcePadding[1];
      encCfg->m_confWinRight  = encCfg->m_sourcePadding[0];
      encCfg->m_confWinBottom = encCfg->m_sourcePadding[1];
      break;
    }
  case 3:
    {
      // conformance
      if ((encCfg->m_confWinLeft == 0) && (encCfg->m_confWinRight == 0) && (encCfg->m_confWinTop == 0) &&
          (encCfg->m_confWinBottom == 0))
      {
        msg(ERROR, "Warning: Conformance window enabled, but all conformance window parameters set to zero\n");
      }
      if ((encCfg->m_sourcePadding[1] != 0) || (encCfg->m_sourcePadding[0] != 0))
      {
        msg(ERROR, "Warning: Conformance window enabled, padding parameters will be ignored\n");
      }
      encCfg->m_sourcePadding[1] = encCfg->m_sourcePadding[0] = 0;
      break;
    }
  }
  CHECK(((encCfg->m_sourceWidth % minResolutionMultiple) || (encCfg->m_sourceHeight % minResolutionMultiple)),
        "Picture width or height (after padding) is not a multiple of 8 or minCuSize, please use "
        "ConformanceWindowMode=1 for automatic adjustment or ConformanceWindowMode=2 to specify padding manually!!");

  if (encCfg->m_conformanceWindowMode > 0 && encCfg->m_subPicInfoPresentFlag)
  {
    for (int i = 0; i < encCfg->m_numSubPics; i++)
    {
      CHECK((encCfg->m_subPicCtuTopLeftX[i] * encCfg->m_CTUSize) >=
              (encCfg->m_sourceWidth - encCfg->m_confWinRight * SPS::getWinUnitX(encCfg->m_chromaFormatIdc)),
            "No subpicture can be located completely outside of the conformance cropping window");
      CHECK(((encCfg->m_subPicCtuTopLeftX[i] + encCfg->m_subPicWidth[i]) * encCfg->m_CTUSize) <=
              (encCfg->m_confWinLeft * SPS::getWinUnitX(encCfg->m_chromaFormatIdc)),
            "No subpicture can be located completely outside of the conformance cropping window");
      CHECK((encCfg->m_subPicCtuTopLeftY[i] * encCfg->m_CTUSize) >=
              (encCfg->m_sourceHeight - encCfg->m_confWinBottom * SPS::getWinUnitY(encCfg->m_chromaFormatIdc)),
            "No subpicture can be located completely outside of the conformance cropping window");
      CHECK(((encCfg->m_subPicCtuTopLeftY[i] + encCfg->m_subPicHeight[i]) * encCfg->m_CTUSize) <=
              (encCfg->m_confWinTop * SPS::getWinUnitY(encCfg->m_chromaFormatIdc)),
            "No subpicture can be located completely outside of the conformance cropping window");
    }
  }

  if (tmpDecodedPictureHashSEIMappedType < 0 || tmpDecodedPictureHashSEIMappedType > to_underlying(HashType::NUM))
  {
    EXIT("Error: bad checksum mode");
  }
  // Need to map values to match those of the SEI message:
  if (tmpDecodedPictureHashSEIMappedType == 0)
  {
    encCfg->m_seiCfg.m_decodedPictureHashSEIType = HashType::NONE;
  }
  else
  {
    encCfg->m_seiCfg.m_decodedPictureHashSEIType = static_cast<HashType>(tmpDecodedPictureHashSEIMappedType - 1);
  }
  // Need to map values to match those of the SEI message:
  if (tmpSubpicDecodedPictureHashMappedType == 0)
  {
    encCfg->m_seiCfg.m_subpicDecodedPictureHashType = HashType::NONE;
  }
  else
  {
    encCfg->m_seiCfg.m_subpicDecodedPictureHashType = static_cast<HashType>(tmpSubpicDecodedPictureHashMappedType - 1);
  }
  // allocate slice-based dQP values
  encCfg->m_frameDeltaQps.resize(encCfg->m_framesToBeEncoded + encCfg->m_gopSize + 1);
  std::fill(encCfg->m_frameDeltaQps.begin(), encCfg->m_frameDeltaQps.end(), 0);

  if (encCfg->m_qpIncrementAtSourceFrame >= 0)
  {
    uint32_t switchingPOC = 0;
    if (encCfg->m_qpIncrementAtSourceFrame > encCfg->m_frameSkip)
    {
      // if switch source frame (ssf) = 10, and frame skip (fs)=2 and temporal subsample ratio (tsr) =1, then
      //    for this simulation switch at POC 8 (=10-2).
      // if ssf=10, fs=2, tsr=2, then for this simulation, switch at POC 4 (=(10-2)/2): POC0=Src2, POC1=Src4, POC2=Src6,
      // POC3=Src8, POC4=Src10
      switchingPOC = (encCfg->m_qpIncrementAtSourceFrame - encCfg->m_frameSkip) / encCfg->m_temporalSubsampleRatio;
    }
    for (uint32_t i = switchingPOC; i < encCfg->m_frameDeltaQps.size(); i++)
    {
      encCfg->m_frameDeltaQps[i] = 1;
    }
  }

#if SHARP_LUMA_DELTA_QP
  CHECK(lumaLevelToDeltaQPMode >= LUMALVL_TO_DQP_NUM_MODES, "Error in cfg");

  encCfg->m_lumaLevelToDeltaQPMapping.mode = LumaLevelToDQPMode(lumaLevelToDeltaQPMode);

  if (encCfg->m_lumaLevelToDeltaQPMapping.mode)
  {
    CHECK(cfg_lumaLeveltoDQPMappingLuma.values.size() != cfg_lumaLeveltoDQPMappingQP.values.size(), "Error in cfg");
    encCfg->m_lumaLevelToDeltaQPMapping.mapping.resize(cfg_lumaLeveltoDQPMappingLuma.values.size());
    for (uint32_t i = 0; i < cfg_lumaLeveltoDQPMappingLuma.values.size(); i++)
    {
      encCfg->m_lumaLevelToDeltaQPMapping.mapping[i] =
        std::pair<int, int>(cfg_lumaLeveltoDQPMappingLuma.values[i], cfg_lumaLeveltoDQPMappingQP.values[i]);
    }
  }
#endif

  CHECK(cfg_qpInValCb.values.size() != cfg_qpOutValCb.values.size(), "Chroma QP table for Cb is incomplete.");
  CHECK(cfg_qpInValCr.values.size() != cfg_qpOutValCr.values.size(), "Chroma QP table for Cr is incomplete.");
  CHECK(cfg_qpInValCbCr.values.size() != cfg_qpOutValCbCr.values.size(), "Chroma QP table for CbCr is incomplete.");
  if (encCfg->m_useIdentityTableForNon420Chroma && encCfg->m_chromaFormatIdc != ChromaFormat::_420)
  {
    encCfg->m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag = true;

    cfg_qpInValCb.values    = { 26 };
    cfg_qpInValCr.values    = { 26 };
    cfg_qpInValCbCr.values  = { 26 };
    cfg_qpOutValCb.values   = { 26 };
    cfg_qpOutValCr.values   = { 26 };
    cfg_qpOutValCbCr.values = { 26 };
  }

  // Need to have at least 2 points in the set. Add second one if only one given
  if (cfg_qpInValCb.values.size() == 1)
  {
    cfg_qpInValCb.values.push_back(cfg_qpInValCb.values[0] + 1);
    cfg_qpOutValCb.values.push_back(cfg_qpOutValCb.values[0] + 1);
  }
  if (cfg_qpInValCr.values.size() == 1)
  {
    cfg_qpInValCr.values.push_back(cfg_qpInValCr.values[0] + 1);
    cfg_qpOutValCr.values.push_back(cfg_qpOutValCr.values[0] + 1);
  }
  if (cfg_qpInValCbCr.values.size() == 1)
  {
    cfg_qpInValCbCr.values.push_back(cfg_qpInValCbCr.values[0] + 1);
    cfg_qpOutValCbCr.values.push_back(cfg_qpOutValCbCr.values[0] + 1);
  }

  int qpBdOffsetC = 6 * (encCfg->m_internalBitDepth[ChannelType::CHROMA] - 8);
  encCfg->m_chromaQpMappingTableParams.m_deltaQpInValMinus1[0].resize(cfg_qpInValCb.values.size());
  encCfg->m_chromaQpMappingTableParams.m_deltaQpOutVal[0].resize(cfg_qpOutValCb.values.size());
  encCfg->m_chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[0] = (int)cfg_qpOutValCb.values.size() - 2;
  encCfg->m_chromaQpMappingTableParams.m_qpTableStartMinus26[0]    = -26 + cfg_qpInValCb.values[0];
  CHECK(encCfg->m_chromaQpMappingTableParams.m_qpTableStartMinus26[0] < -26 - qpBdOffsetC ||
          encCfg->m_chromaQpMappingTableParams.m_qpTableStartMinus26[0] > 36,
        "qpTableStartMinus26[0] is out of valid range of -26 -qpBdOffsetC to 36, inclusive.")
  CHECK(cfg_qpInValCb.values[0] != cfg_qpOutValCb.values[0],
        "First qpInValCb value should be equal to first qpOutValCb value");
  for (int i = 0; i < cfg_qpInValCb.values.size() - 1; i++)
  {
    CHECK(cfg_qpInValCb.values[i] < -qpBdOffsetC || cfg_qpInValCb.values[i] > MAX_QP,
          "Some entries cfg_qpInValCb are out of valid range of -qpBdOffsetC to 63, inclusive.");
    CHECK(cfg_qpOutValCb.values[i] < -qpBdOffsetC || cfg_qpOutValCb.values[i] > MAX_QP,
          "Some entries cfg_qpOutValCb are out of valid range of -qpBdOffsetC to 63, inclusive.");
    encCfg->m_chromaQpMappingTableParams.m_deltaQpInValMinus1[0][i] =
      cfg_qpInValCb.values[i + 1] - cfg_qpInValCb.values[i] - 1;
    encCfg->m_chromaQpMappingTableParams.m_deltaQpOutVal[0][i] =
      cfg_qpOutValCb.values[i + 1] - cfg_qpOutValCb.values[i];
  }
  if (!encCfg->m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag)
  {
    encCfg->m_chromaQpMappingTableParams.m_deltaQpInValMinus1[1].resize(cfg_qpInValCr.values.size());
    encCfg->m_chromaQpMappingTableParams.m_deltaQpOutVal[1].resize(cfg_qpOutValCr.values.size());
    encCfg->m_chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[1] = (int)cfg_qpOutValCr.values.size() - 2;
    encCfg->m_chromaQpMappingTableParams.m_qpTableStartMinus26[1]    = -26 + cfg_qpInValCr.values[0];
    CHECK(encCfg->m_chromaQpMappingTableParams.m_qpTableStartMinus26[1] < -26 - qpBdOffsetC ||
            encCfg->m_chromaQpMappingTableParams.m_qpTableStartMinus26[1] > 36,
          "qpTableStartMinus26[1] is out of valid range of -26 -qpBdOffsetC to 36, inclusive.")
    CHECK(cfg_qpInValCr.values[0] != cfg_qpOutValCr.values[0],
          "First qpInValCr value should be equal to first qpOutValCr value");
    for (int i = 0; i < cfg_qpInValCr.values.size() - 1; i++)
    {
      CHECK(cfg_qpInValCr.values[i] < -qpBdOffsetC || cfg_qpInValCr.values[i] > MAX_QP,
            "Some entries cfg_qpInValCr are out of valid range of -qpBdOffsetC to 63, inclusive.");
      CHECK(cfg_qpOutValCr.values[i] < -qpBdOffsetC || cfg_qpOutValCr.values[i] > MAX_QP,
            "Some entries cfg_qpOutValCr are out of valid range of -qpBdOffsetC to 63, inclusive.");
      encCfg->m_chromaQpMappingTableParams.m_deltaQpInValMinus1[1][i] =
        cfg_qpInValCr.values[i + 1] - cfg_qpInValCr.values[i] - 1;
      encCfg->m_chromaQpMappingTableParams.m_deltaQpOutVal[1][i] =
        cfg_qpOutValCr.values[i + 1] - cfg_qpOutValCr.values[i];
    }
    encCfg->m_chromaQpMappingTableParams.m_deltaQpInValMinus1[2].resize(cfg_qpInValCbCr.values.size());
    encCfg->m_chromaQpMappingTableParams.m_deltaQpOutVal[2].resize(cfg_qpOutValCbCr.values.size());
    encCfg->m_chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[2] = (int)cfg_qpOutValCbCr.values.size() - 2;
    encCfg->m_chromaQpMappingTableParams.m_qpTableStartMinus26[2]    = -26 + cfg_qpInValCbCr.values[0];
    CHECK(encCfg->m_chromaQpMappingTableParams.m_qpTableStartMinus26[2] < -26 - qpBdOffsetC ||
            encCfg->m_chromaQpMappingTableParams.m_qpTableStartMinus26[2] > 36,
          "qpTableStartMinus26[2] is out of valid range of -26 -qpBdOffsetC to 36, inclusive.")
    CHECK(cfg_qpInValCbCr.values[0] != cfg_qpInValCbCr.values[0],
          "First qpInValCbCr value should be equal to first qpOutValCbCr value");
    for (int i = 0; i < cfg_qpInValCbCr.values.size() - 1; i++)
    {
      CHECK(cfg_qpInValCbCr.values[i] < -qpBdOffsetC || cfg_qpInValCbCr.values[i] > MAX_QP,
            "Some entries cfg_qpInValCbCr are out of valid range of -qpBdOffsetC to 63, inclusive.");
      CHECK(cfg_qpOutValCbCr.values[i] < -qpBdOffsetC || cfg_qpOutValCbCr.values[i] > MAX_QP,
            "Some entries cfg_qpOutValCbCr are out of valid range of -qpBdOffsetC to 63, inclusive.");
      encCfg->m_chromaQpMappingTableParams.m_deltaQpInValMinus1[2][i] =
        cfg_qpInValCbCr.values[i + 1] - cfg_qpInValCbCr.values[i] - 1;
      encCfg->m_chromaQpMappingTableParams.m_deltaQpOutVal[2][i] =
        cfg_qpInValCbCr.values[i + 1] - cfg_qpInValCbCr.values[i];
    }
  }

  /* Local chroma QP offsets configuration */
  CHECK(encCfg->m_cuChromaQpOffsetSubdiv < 0, "MaxCuChromaQpOffsetSubdiv shall be >= 0");
  CHECK(cfg_crQpOffsetList.values.size() != cfg_cbQpOffsetList.values.size(),
        "Chroma QP offset lists shall be the same size");
  CHECK(cfg_cbCrQpOffsetList.values.size() != cfg_cbQpOffsetList.values.size() &&
          cfg_cbCrQpOffsetList.values.size() > 0,
        "Chroma QP offset list for joint CbCr shall be either the same size as Cb and Cr or empty");
  if (encCfg->m_cuChromaQpOffsetSubdiv > 0 && !cfg_cbQpOffsetList.values.size())
  {
    msg(WARNING, "MaxCuChromaQpOffsetSubdiv has no effect when chroma QP offset lists are empty\n");
  }
  encCfg->m_cuChromaQpOffsetList.resize(cfg_cbQpOffsetList.values.size());
  for (int i = 0; i < cfg_cbQpOffsetList.values.size(); i++)
  {
    encCfg->m_cuChromaQpOffsetList[i].u.comp.cbOffset = cfg_cbQpOffsetList.values[i];
    encCfg->m_cuChromaQpOffsetList[i].u.comp.crOffset = cfg_crQpOffsetList.values[i];
    encCfg->m_cuChromaQpOffsetList[i].u.comp.jointCbCrOffset =
      cfg_cbCrQpOffsetList.values.size() ? cfg_cbCrQpOffsetList.values[i] : 0;
  }
  if (encCfg->m_rprFunctionalityTestingEnabledFlag)
  {
    encCfg->m_upscaledOutput = 2;
    if (encCfg->m_scalingRatioHor == 1.0 && encCfg->m_scalingRatioVer == 1.0)
    {
      encCfg->m_scalingRatioHor = 2.0;
      encCfg->m_scalingRatioVer = 2.0;
    }
    CHECK(cfg_rprSwitchingResolutionOrderList.values.size() > MAX_RPR_SWITCHING_ORDER_LIST_SIZE,
          "Length of RPRSwitchingResolutionOrderList exceeds maximum length");
    CHECK(cfg_rprSwitchingQPOffsetOrderList.values.size() > MAX_RPR_SWITCHING_ORDER_LIST_SIZE,
          "Length of RPRSwitchingQPOffsetOrderList exceeds maximum length");
    CHECK(cfg_rprSwitchingResolutionOrderList.values.size() != cfg_rprSwitchingQPOffsetOrderList.values.size(),
          "RPRSwitchingResolutionOrderList and RPRSwitchingQPOffsetOrderList shall be the same size");
    encCfg->m_rprSwitchingListSize = (int)cfg_rprSwitchingResolutionOrderList.values.size();
    for (int k = 0; k < encCfg->m_rprSwitchingListSize; k++)
    {
      encCfg->m_rprSwitchingResolutionOrderList[k] = cfg_rprSwitchingResolutionOrderList.values[k];
      encCfg->m_rprSwitchingQPOffsetOrderList[k]   = cfg_rprSwitchingQPOffsetOrderList.values[k];
    }
    if (encCfg->m_rprSwitchingTime != 0.0)
    {
      int segmentSize                   = 8 * int(((double)encCfg->m_frameRate * encCfg->m_rprSwitchingTime + 4) / 8);
      encCfg->m_rprSwitchingSegmentSize = segmentSize;
    }
  }
  if (encCfg->m_ladfEnabled)
  {
    CHECK(encCfg->m_ladfNumIntervals != cfg_ladfQpOffset.values.size(),
          "size of LadfQpOffset must be equal to LadfNumIntervals");
    CHECK(encCfg->m_ladfNumIntervals - 1 != cfg_ladfIntervalLowerBound.values.size(),
          "size of LadfIntervalLowerBound must be equal to LadfNumIntervals - 1");
    CHECK(encCfg->m_ladfNumIntervals > MAX_LADF_INTERVALS, "size of LadfQpOffset out of array bounds");
    for (int k = 0; k < encCfg->m_ladfNumIntervals; k++)
    {
      encCfg->m_ladfQpOffset[k] = cfg_ladfQpOffset.values[k];
    }
    encCfg->m_ladfIntervalLowerBound[0] = 0;
    for (int k = 1; k < encCfg->m_ladfNumIntervals; k++)
    {
      encCfg->m_ladfIntervalLowerBound[k] = cfg_ladfIntervalLowerBound.values[k - 1];
    }
  }

  if (encCfg->m_chromaFormatIdc != ChromaFormat::_420)
  {
    if (!encCfg->m_horCollocatedChromaFlag)
    {
      msg(WARNING, "\nWARNING: HorCollocatedChroma is forced to 1 for chroma formats other than 4:2:0\n");
      encCfg->m_horCollocatedChromaFlag = true;
    }
    if (!encCfg->m_verCollocatedChromaFlag)
    {
      msg(WARNING, "\nWARNING: VerCollocatedChroma is forced to 1 for chroma formats other than 4:2:0\n");
      encCfg->m_verCollocatedChromaFlag = true;
    }
  }
#if JVET_O0756_CONFIG_HDRMETRICS && !JVET_O0756_CALCULATE_HDRMETRICS
  if (encCfg->m_calculateHdrMetrics == true)
  {
    printf("Warning: Configuration enables HDR metric calculations.  However, HDR metric support was not linked when "
           "compiling the VTM.\n");
    encCfg->m_calculateHdrMetrics = false;
  }
#endif

  if (encCfg->m_alf)
  {
    CHECK(encCfg->m_maxNumAlfAlternativesChroma < 1 ||
            encCfg->m_maxNumAlfAlternativesChroma > AlfParameters::ALF_MAX_NUM_ALTERNATIVES_CHROMA,
          std::string("The maximum number of ALF Chroma filter alternatives must be in the range (1-") +
            std::to_string(AlfParameters::ALF_MAX_NUM_ALTERNATIVES_CHROMA) + std::string(", inclusive)"));

    // Configure the APS parameter classes depending on whether the ALF improvements are enabled.
    ApsAlfParam::setAlfType(encCfg->m_alfImprovements);
    ApsCcAlfParam::setAlfType(encCfg->m_alfImprovements);
  }

  // reading external dQP description from file
  if (!encCfg->m_dQPFileName.empty())
  {
    FILE *fpt = fopen(encCfg->m_dQPFileName.c_str(), "r");
    if (fpt)
    {
      int val;
      int poc = 0;
      encCfg->m_frameDeltaQps.clear();
      while (poc < encCfg->m_framesToBeEncoded)
      {
        if (fscanf(fpt, "%d", &val) == EOF)
        {
          break;
        }
        encCfg->m_frameDeltaQps.push_back(val);
        poc++;
      }
      fclose(fpt);
    }
  }

  if (encCfg->m_seiCfg.m_masteringDisplay.colourVolumeSEIEnabled)
  {
    for (uint32_t idx = 0; idx < 6; idx++)
    {
      encCfg->m_seiCfg.m_masteringDisplay.primaries[idx / 2][idx % 2] =
        uint16_t((cfg_DisplayPrimariesCode.values.size() > idx) ? cfg_DisplayPrimariesCode.values[idx] : 0);
    }
    for (uint32_t idx = 0; idx < 2; idx++)
    {
      encCfg->m_seiCfg.m_masteringDisplay.whitePoint[idx] =
        uint16_t((cfg_DisplayWhitePointCode.values.size() > idx) ? cfg_DisplayWhitePointCode.values[idx] : 0);
    }
  }
  // set sei film grain parameters.
  CHECK(!encCfg->m_seiCfg.m_fgcSEIEnabled && encCfg->m_seiCfg.m_fgcSEIAnalysisEnabled,
        "FGC SEI must be enabled in order to perform film grain analysis!");
  if (encCfg->m_seiCfg.m_fgcSEIEnabled)
  {
    if (encCfg->m_iQP < 17 && encCfg->m_seiCfg.m_fgcSEIAnalysisEnabled == true)
    {   // TODO: JVET_Z0047_FG_IMPROVEMENT: check this; the constraint may have gone
      msg(WARNING, "*************************************************************************\n");
      msg(WARNING,
          "* WARNING: Film Grain Estimation is disabled for Qp<17! FGC SEI will use default parameters for film grain! "
          "*\n");
      msg(WARNING, "*************************************************************************\n");
      encCfg->m_seiCfg.m_fgcSEIAnalysisEnabled = false;
    }
    if (encCfg->m_intraPeriod < 1)
    {   // low delay configuration
      msg(WARNING, "*************************************************************************\n");
      msg(WARNING, "* WARNING: For low delay configuration, FGC SEI is inserted for first frame only!*\n");
      msg(WARNING, "*************************************************************************\n");
      encCfg->m_seiCfg.m_fgcSEIPerPictureSEI   = false;
      encCfg->m_seiCfg.m_fgcSEIPersistenceFlag = true;
    }
    else if (encCfg->m_intraPeriod == 1)
    {   // all intra configuration
      msg(WARNING, "*************************************************************************\n");
      msg(WARNING, "* WARNING: For Intra Period = 1, FGC SEI is inserted per frame!*\n");
      msg(WARNING, "*************************************************************************\n");
      encCfg->m_seiCfg.m_fgcSEIPerPictureSEI   = true;
      encCfg->m_seiCfg.m_fgcSEIPersistenceFlag = false;
    }
    if (!encCfg->m_seiCfg.m_fgcSEIPerPictureSEI && !encCfg->m_seiCfg.m_fgcSEIPersistenceFlag)
    {
      msg(WARNING, "*************************************************************************\n");
      msg(WARNING, "* WARNING: SEIPerPictureSEI is set to 0, SEIPersistenceFlag needs to be set to 1! *\n");
      msg(WARNING, "*************************************************************************\n");
      encCfg->m_seiCfg.m_fgcSEIPersistenceFlag = true;
    }
    else if (encCfg->m_seiCfg.m_fgcSEIPerPictureSEI && encCfg->m_seiCfg.m_fgcSEIPersistenceFlag)
    {
      msg(WARNING, "*************************************************************************\n");
      msg(WARNING, "* WARNING: SEIPerPictureSEI is set to 1, SEIPersistenceFlag needs to be set to 0! *\n");
      msg(WARNING, "*************************************************************************\n");
      encCfg->m_seiCfg.m_fgcSEIPersistenceFlag = false;
    }
    if (encCfg->m_seiCfg.m_fgcSEIAnalysisEnabled && encCfg->m_seiCfg.m_fgcSEITemporalFilterStrengths.empty())
    {
      // By default: in random-acces = filter RAPs, in all-intra = filter every frame, otherwise = filter every 2s
      int filteredFrame = encCfg->m_intraPeriod < 1 ? 2 * encCfg->m_frameRate : encCfg->m_intraPeriod;
      encCfg->m_seiCfg.m_fgcSEITemporalFilterStrengths[filteredFrame] = 1.5;
    }
    uint32_t numModelCtr;
    if (encCfg->m_seiCfg.m_fgcSEICompModelPresent[0])
    {
      numModelCtr = 0;
      for (uint8_t i = 0; i <= encCfg->m_seiCfg.m_fgcSEINumIntensityIntervalMinus1[0]; i++)
      {
        encCfg->m_seiCfg.m_fgcSEIIntensityIntervalLowerBound[0][i] =
          uint32_t((cfg_FgcSEIIntensityIntervalLowerBoundComp0.values.size() > i)
                     ? cfg_FgcSEIIntensityIntervalLowerBoundComp0.values[i]
                     : 10);
        encCfg->m_seiCfg.m_fgcSEIIntensityIntervalUpperBound[0][i] =
          uint32_t((cfg_FgcSEIIntensityIntervalUpperBoundComp0.values.size() > i)
                     ? cfg_FgcSEIIntensityIntervalUpperBoundComp0.values[i]
                     : 250);
        for (uint8_t j = 0; j <= encCfg->m_seiCfg.m_fgcSEINumModelValuesMinus1[0]; j++)
        {
          encCfg->m_seiCfg.m_fgcSEICompModelValue[0][i][j] =
            uint32_t((cfg_FgcSEICompModelValueComp0.values.size() > numModelCtr)
                       ? cfg_FgcSEICompModelValueComp0.values[numModelCtr]
                       : 24);
          numModelCtr++;
        }
      }
    }
    if (encCfg->m_seiCfg.m_fgcSEICompModelPresent[1])
    {
      numModelCtr = 0;
      for (uint8_t i = 0; i <= encCfg->m_seiCfg.m_fgcSEINumIntensityIntervalMinus1[1]; i++)
      {
        encCfg->m_seiCfg.m_fgcSEIIntensityIntervalLowerBound[1][i] =
          uint32_t((cfg_FgcSEIIntensityIntervalLowerBoundComp1.values.size() > i)
                     ? cfg_FgcSEIIntensityIntervalLowerBoundComp1.values[i]
                     : 60);
        encCfg->m_seiCfg.m_fgcSEIIntensityIntervalUpperBound[1][i] =
          uint32_t((cfg_FgcSEIIntensityIntervalUpperBoundComp1.values.size() > i)
                     ? cfg_FgcSEIIntensityIntervalUpperBoundComp1.values[i]
                     : 200);

        for (uint8_t j = 0; j <= encCfg->m_seiCfg.m_fgcSEINumModelValuesMinus1[1]; j++)
        {
          encCfg->m_seiCfg.m_fgcSEICompModelValue[1][i][j] =
            uint32_t((cfg_FgcSEICompModelValueComp1.values.size() > numModelCtr)
                       ? cfg_FgcSEICompModelValueComp1.values[numModelCtr]
                       : 16);
          numModelCtr++;
        }
      }
    }
    if (encCfg->m_seiCfg.m_fgcSEICompModelPresent[2])
    {
      numModelCtr = 0;
      for (uint8_t i = 0; i <= encCfg->m_seiCfg.m_fgcSEINumIntensityIntervalMinus1[2]; i++)
      {
        encCfg->m_seiCfg.m_fgcSEIIntensityIntervalLowerBound[2][i] =
          uint32_t((cfg_FgcSEIIntensityIntervalLowerBoundComp2.values.size() > i)
                     ? cfg_FgcSEIIntensityIntervalLowerBoundComp2.values[i]
                     : 60);
        encCfg->m_seiCfg.m_fgcSEIIntensityIntervalUpperBound[2][i] =
          uint32_t((cfg_FgcSEIIntensityIntervalUpperBoundComp2.values.size() > i)
                     ? cfg_FgcSEIIntensityIntervalUpperBoundComp2.values[i]
                     : 250);

        for (uint8_t j = 0; j <= encCfg->m_seiCfg.m_fgcSEINumModelValuesMinus1[2]; j++)
        {
          encCfg->m_seiCfg.m_fgcSEICompModelValue[2][i][j] =
            uint32_t((cfg_FgcSEICompModelValueComp2.values.size() > numModelCtr)
                       ? cfg_FgcSEICompModelValueComp2.values[numModelCtr]
                       : 12);
          numModelCtr++;
        }
      }
    }
    encCfg->m_seiCfg.m_fgcSEILog2ScaleFactor =
      encCfg->m_seiCfg.m_fgcSEILog2ScaleFactor ? encCfg->m_seiCfg.m_fgcSEILog2ScaleFactor : 2;
  }
  if (encCfg->m_seiCfg.m_ctiSEIEnabled)
  {
    CHECK(!encCfg->m_seiCfg.m_ctiSEICrossComponentFlag && encCfg->m_seiCfg.m_ctiSEICrossComponentInferred,
          "CTI CrossComponentFlag is 0, but CTI CrossComponentInferred is 1 (must be 0 for CrossComponentFlag 0)");
    CHECK(!encCfg->m_seiCfg.m_ctiSEICrossComponentFlag && !encCfg->m_seiCfg.m_ctiSEICrossComponentInferred &&
            !encCfg->m_seiCfg.m_ctiSEINumberChromaLut,
          "For CTI CrossComponentFlag = 0, CTI NumberChromaLut needs to be specified (1 or 2) ");
    CHECK(encCfg->m_seiCfg.m_ctiSEICrossComponentFlag && !encCfg->m_seiCfg.m_ctiSEICrossComponentInferred &&
            !encCfg->m_seiCfg.m_ctiSEINumberChromaLut,
          "For CTI CrossComponentFlag = 1 and CrossComponentInferred = 0, CTI NumberChromaLut needs to be specified (1 "
          "or 2) ");

    CHECK(cfg_SEICTILut0.values.empty(), "SEI CTI (SEICTIEnabled) but no LUT0 specified");
    encCfg->m_seiCfg.m_ctiSEILut[0].presentFlag  = true;
    encCfg->m_seiCfg.m_ctiSEILut[0].numLutValues = (int)cfg_SEICTILut0.values.size();
    encCfg->m_seiCfg.m_ctiSEILut[0].lutValues    = cfg_SEICTILut0.values;

    if (!encCfg->m_seiCfg.m_ctiSEICrossComponentFlag ||
        (encCfg->m_seiCfg.m_ctiSEICrossComponentFlag && !encCfg->m_seiCfg.m_ctiSEICrossComponentInferred))
    {
      CHECK(cfg_SEICTILut1.values.empty(), "SEI CTI LUT1 not specified");
      encCfg->m_seiCfg.m_ctiSEILut[1].presentFlag  = true;
      encCfg->m_seiCfg.m_ctiSEILut[1].numLutValues = (int)cfg_SEICTILut1.values.size();
      encCfg->m_seiCfg.m_ctiSEILut[1].lutValues    = cfg_SEICTILut1.values;

      if (encCfg->m_seiCfg.m_ctiSEINumberChromaLut == 1)
      {   // Cb lut the same as Cr lut
        encCfg->m_seiCfg.m_ctiSEILut[2].presentFlag  = true;
        encCfg->m_seiCfg.m_ctiSEILut[2].numLutValues = encCfg->m_seiCfg.m_ctiSEILut[1].numLutValues;
        encCfg->m_seiCfg.m_ctiSEILut[2].lutValues    = encCfg->m_seiCfg.m_ctiSEILut[1].lutValues;
      }
      else if (encCfg->m_seiCfg.m_ctiSEINumberChromaLut == 2)
      {   // read from cfg
        CHECK(cfg_SEICTILut2.values.empty(), "SEI CTI LUT2 not specified");
        encCfg->m_seiCfg.m_ctiSEILut[2].presentFlag  = true;
        encCfg->m_seiCfg.m_ctiSEILut[2].numLutValues = (int)cfg_SEICTILut2.values.size();
        encCfg->m_seiCfg.m_ctiSEILut[2].lutValues    = cfg_SEICTILut2.values;
      }
      else
      {
        CHECK(encCfg->m_seiCfg.m_ctiSEINumberChromaLut < 1 && encCfg->m_seiCfg.m_ctiSEINumberChromaLut > 2,
              "Number of chroma LUTs is missing or out of range!");
      }
    }
    //  check if lut size is power of 2
    for (int idx = 0; idx < MAX_NUM_COMP; idx++)
    {
      int n = encCfg->m_seiCfg.m_ctiSEILut[idx].numLutValues - 1;
      CHECK(n > 0 && (n & (n - 1)) != 0, "Size of LUT minus 1 should be power of 2!");
      CHECK(n > MAX_CTI_LUT_SIZE, "LUT size minus 1 is larger than MAX_CTI_LUT_SIZE (64)!");
    }
  }
  if (encCfg->m_seiCfg.m_omniViewportSEIEnabled && !encCfg->m_seiCfg.m_omniViewportSEICancelFlag)
  {
    CHECK(!(encCfg->m_seiCfg.m_omniViewportSEICntMinus1 >= 0 && encCfg->m_seiCfg.m_omniViewportSEICntMinus1 < 16),
          "SEIOmniViewportCntMinus1 must be in the range of 0 to 16");
    encCfg->m_seiCfg.m_omniViewportSEIAzimuthCentre.resize(encCfg->m_seiCfg.m_omniViewportSEICntMinus1 + 1);
    encCfg->m_seiCfg.m_omniViewportSEIElevationCentre.resize(encCfg->m_seiCfg.m_omniViewportSEICntMinus1 + 1);
    encCfg->m_seiCfg.m_omniViewportSEITiltCentre.resize(encCfg->m_seiCfg.m_omniViewportSEICntMinus1 + 1);
    encCfg->m_seiCfg.m_omniViewportSEIHorRange.resize(encCfg->m_seiCfg.m_omniViewportSEICntMinus1 + 1);
    encCfg->m_seiCfg.m_omniViewportSEIVerRange.resize(encCfg->m_seiCfg.m_omniViewportSEICntMinus1 + 1);
    for (int i = 0; i < (encCfg->m_seiCfg.m_omniViewportSEICntMinus1 + 1); i++)
    {
      encCfg->m_seiCfg.m_omniViewportSEIAzimuthCentre[i] =
        cfg_omniViewportSEIAzimuthCentre.values.size() > i ? cfg_omniViewportSEIAzimuthCentre.values[i] : 0;
      encCfg->m_seiCfg.m_omniViewportSEIElevationCentre[i] =
        cfg_omniViewportSEIElevationCentre.values.size() > i ? cfg_omniViewportSEIElevationCentre.values[i] : 0;
      encCfg->m_seiCfg.m_omniViewportSEITiltCentre[i] =
        cfg_omniViewportSEITiltCentre.values.size() > i ? cfg_omniViewportSEITiltCentre.values[i] : 0;
      encCfg->m_seiCfg.m_omniViewportSEIHorRange[i] =
        cfg_omniViewportSEIHorRange.values.size() > i ? cfg_omniViewportSEIHorRange.values[i] : 0;
      encCfg->m_seiCfg.m_omniViewportSEIVerRange[i] =
        cfg_omniViewportSEIVerRange.values.size() > i ? cfg_omniViewportSEIVerRange.values[i] : 0;
    }
  }

  if (!encCfg->m_seiCfg.m_rwpSEIRwpCancelFlag && encCfg->m_seiCfg.m_rwpSEIEnabled)
  {
    CHECK(!(encCfg->m_seiCfg.m_rwpSEINumPackedRegions > 0 &&
            encCfg->m_seiCfg.m_rwpSEINumPackedRegions <= std::numeric_limits<uint8_t>::max()),
          "SEIRwpNumPackedRegions must be in the range of 1 to 255");
    CHECK(!(cfg_rwpSEIRwpTransformType.values.size() == encCfg->m_seiCfg.m_rwpSEINumPackedRegions),
          "Number of must SEIRwpTransformType values be equal to SEIRwpNumPackedRegions");
    CHECK(!(cfg_rwpSEIRwpGuardBandFlag.values.size() == encCfg->m_seiCfg.m_rwpSEINumPackedRegions),
          "Number of must SEIRwpGuardBandFlag values must be equal to SEIRwpNumPackedRegions");
    CHECK(!(cfg_rwpSEIProjRegionWidth.values.size() == encCfg->m_seiCfg.m_rwpSEINumPackedRegions),
          "Number of must SEIProjRegionWidth values must be equal to SEIRwpNumPackedRegions");
    CHECK(!(cfg_rwpSEIProjRegionHeight.values.size() == encCfg->m_seiCfg.m_rwpSEINumPackedRegions),
          "Number of must SEIProjRegionHeight values must be equal to SEIRwpNumPackedRegions");
    CHECK(!(cfg_rwpSEIRwpSEIProjRegionTop.values.size() == encCfg->m_seiCfg.m_rwpSEINumPackedRegions),
          "Number of must SEIRwpSEIProjRegionTop values must be equal to SEIRwpNumPackedRegions");
    CHECK(!(cfg_rwpSEIProjRegionLeft.values.size() == encCfg->m_seiCfg.m_rwpSEINumPackedRegions),
          "Number of must SEIProjRegionLeft values must be equal to SEIRwpNumPackedRegions");
    CHECK(!(cfg_rwpSEIPackedRegionWidth.values.size() == encCfg->m_seiCfg.m_rwpSEINumPackedRegions),
          "Number of must SEIPackedRegionWidth values must be equal to SEIRwpNumPackedRegions");
    CHECK(!(cfg_rwpSEIPackedRegionHeight.values.size() == encCfg->m_seiCfg.m_rwpSEINumPackedRegions),
          "Number of must SEIPackedRegionHeight values must be equal to SEIRwpNumPackedRegions");
    CHECK(!(cfg_rwpSEIPackedRegionTop.values.size() == encCfg->m_seiCfg.m_rwpSEINumPackedRegions),
          "Number of must SEIPackedRegionTop values must be equal to SEIRwpNumPackedRegions");
    CHECK(!(cfg_rwpSEIPackedRegionLeft.values.size() == encCfg->m_seiCfg.m_rwpSEINumPackedRegions),
          "Number of must SEIPackedRegionLeft values must be equal to SEIRwpNumPackedRegions");

    encCfg->m_seiCfg.m_rwpSEIRwpTransformType.resize(encCfg->m_seiCfg.m_rwpSEINumPackedRegions);
    encCfg->m_seiCfg.m_rwpSEIRwpGuardBandFlag.resize(encCfg->m_seiCfg.m_rwpSEINumPackedRegions);
    encCfg->m_seiCfg.m_rwpSEIProjRegionWidth.resize(encCfg->m_seiCfg.m_rwpSEINumPackedRegions);
    encCfg->m_seiCfg.m_rwpSEIProjRegionHeight.resize(encCfg->m_seiCfg.m_rwpSEINumPackedRegions);
    encCfg->m_seiCfg.m_rwpSEIRwpSEIProjRegionTop.resize(encCfg->m_seiCfg.m_rwpSEINumPackedRegions);
    encCfg->m_seiCfg.m_rwpSEIProjRegionLeft.resize(encCfg->m_seiCfg.m_rwpSEINumPackedRegions);
    encCfg->m_seiCfg.m_rwpSEIPackedRegionWidth.resize(encCfg->m_seiCfg.m_rwpSEINumPackedRegions);
    encCfg->m_seiCfg.m_rwpSEIPackedRegionHeight.resize(encCfg->m_seiCfg.m_rwpSEINumPackedRegions);
    encCfg->m_seiCfg.m_rwpSEIPackedRegionTop.resize(encCfg->m_seiCfg.m_rwpSEINumPackedRegions);
    encCfg->m_seiCfg.m_rwpSEIPackedRegionLeft.resize(encCfg->m_seiCfg.m_rwpSEINumPackedRegions);
    encCfg->m_seiCfg.m_rwpSEIRwpLeftGuardBandWidth.resize(encCfg->m_seiCfg.m_rwpSEINumPackedRegions);
    encCfg->m_seiCfg.m_rwpSEIRwpRightGuardBandWidth.resize(encCfg->m_seiCfg.m_rwpSEINumPackedRegions);
    encCfg->m_seiCfg.m_rwpSEIRwpTopGuardBandHeight.resize(encCfg->m_seiCfg.m_rwpSEINumPackedRegions);
    encCfg->m_seiCfg.m_rwpSEIRwpBottomGuardBandHeight.resize(encCfg->m_seiCfg.m_rwpSEINumPackedRegions);
    encCfg->m_seiCfg.m_rwpSEIRwpGuardBandNotUsedForPredFlag.resize(encCfg->m_seiCfg.m_rwpSEINumPackedRegions);
    encCfg->m_seiCfg.m_rwpSEIRwpGuardBandType.resize(4 * encCfg->m_seiCfg.m_rwpSEINumPackedRegions);
    for (int i = 0; i < encCfg->m_seiCfg.m_rwpSEINumPackedRegions; i++)
    {
      encCfg->m_seiCfg.m_rwpSEIRwpTransformType[i] = cfg_rwpSEIRwpTransformType.values[i];
      CHECK(!(encCfg->m_seiCfg.m_rwpSEIRwpTransformType[i] >= 0 && encCfg->m_seiCfg.m_rwpSEIRwpTransformType[i] <= 7),
            "SEIRwpTransformType must be in the range of 0 to 7");
      encCfg->m_seiCfg.m_rwpSEIRwpGuardBandFlag[i]    = cfg_rwpSEIRwpGuardBandFlag.values[i];
      encCfg->m_seiCfg.m_rwpSEIProjRegionWidth[i]     = cfg_rwpSEIProjRegionWidth.values[i];
      encCfg->m_seiCfg.m_rwpSEIProjRegionHeight[i]    = cfg_rwpSEIProjRegionHeight.values[i];
      encCfg->m_seiCfg.m_rwpSEIRwpSEIProjRegionTop[i] = cfg_rwpSEIRwpSEIProjRegionTop.values[i];
      encCfg->m_seiCfg.m_rwpSEIProjRegionLeft[i]      = cfg_rwpSEIProjRegionLeft.values[i];
      encCfg->m_seiCfg.m_rwpSEIPackedRegionWidth[i]   = cfg_rwpSEIPackedRegionWidth.values[i];
      encCfg->m_seiCfg.m_rwpSEIPackedRegionHeight[i]  = cfg_rwpSEIPackedRegionHeight.values[i];
      encCfg->m_seiCfg.m_rwpSEIPackedRegionTop[i]     = cfg_rwpSEIPackedRegionTop.values[i];
      encCfg->m_seiCfg.m_rwpSEIPackedRegionLeft[i]    = cfg_rwpSEIPackedRegionLeft.values[i];
      if (encCfg->m_seiCfg.m_rwpSEIRwpGuardBandFlag[i])
      {
        encCfg->m_seiCfg.m_rwpSEIRwpLeftGuardBandWidth[i]    = cfg_rwpSEIRwpLeftGuardBandWidth.values[i];
        encCfg->m_seiCfg.m_rwpSEIRwpRightGuardBandWidth[i]   = cfg_rwpSEIRwpRightGuardBandWidth.values[i];
        encCfg->m_seiCfg.m_rwpSEIRwpTopGuardBandHeight[i]    = cfg_rwpSEIRwpTopGuardBandHeight.values[i];
        encCfg->m_seiCfg.m_rwpSEIRwpBottomGuardBandHeight[i] = cfg_rwpSEIRwpBottomGuardBandHeight.values[i];
        CHECK(!(encCfg->m_seiCfg.m_rwpSEIRwpLeftGuardBandWidth[i] > 0 ||
                encCfg->m_seiCfg.m_rwpSEIRwpRightGuardBandWidth[i] > 0 ||
                encCfg->m_seiCfg.m_rwpSEIRwpTopGuardBandHeight[i] > 0 ||
                encCfg->m_seiCfg.m_rwpSEIRwpBottomGuardBandHeight[i] > 0),
              "At least one of the RWP guard band parameters mut be greater than zero");
        encCfg->m_seiCfg.m_rwpSEIRwpGuardBandNotUsedForPredFlag[i] = cfg_rwpSEIRwpGuardBandNotUsedForPredFlag.values[i];
        for (int j = 0; j < 4; j++)
        {
          encCfg->m_seiCfg.m_rwpSEIRwpGuardBandType[i * 4 + j] = cfg_rwpSEIRwpGuardBandType.values[i * 4 + j];
        }
      }
    }
  }
  if (encCfg->m_seiCfg.m_gcmpSEIEnabled && !encCfg->m_seiCfg.m_gcmpSEICancelFlag)
  {
    int numFace = encCfg->m_seiCfg.m_gcmpSEIPackingType == 4 || encCfg->m_seiCfg.m_gcmpSEIPackingType == 5 ? 5 : 6;
    CHECK(!(cfg_gcmpSEIFaceIndex.values.size() == numFace),
          "Number of SEIGcmpFaceIndex must be equal to 5 when SEIGcmpPackingType is equal to 4 or 5, otherwise, it "
          "must be equal to 6");
    CHECK(!(cfg_gcmpSEIFaceRotation.values.size() == numFace),
          "Number of SEIGcmpFaceRotation must be equal to 5 when SEIGcmpPackingType is equal to 4 or 5, otherwise, it "
          "must be equal to 6");
    encCfg->m_seiCfg.m_gcmpSEIFaceIndex.resize(numFace);
    encCfg->m_seiCfg.m_gcmpSEIFaceRotation.resize(numFace);
    if (encCfg->m_seiCfg.m_gcmpSEIMappingFunctionType == 2)
    {
      CHECK(!(cfg_gcmpSEIFunctionCoeffU.values.size() == numFace),
            "Number of SEIGcmpFunctionCoeffU must be equal to 5 when SEIGcmpPackingType is equal to 4 or 5, otherwise, "
            "it must be equal to 6");
      CHECK(!(cfg_gcmpSEIFunctionUAffectedByVFlag.values.size() == numFace),
            "Number of SEIGcmpFunctionUAffectedByVFlag must be equal to 5 when SEIGcmpPackingType is equal to 4 or 5, "
            "otherwise, it must be equal to 6");
      CHECK(!(cfg_gcmpSEIFunctionCoeffV.values.size() == numFace),
            "Number of SEIGcmpFunctionCoeffV must be equal to 5 when SEIGcmpPackingType is equal to 4 or 5, otherwise, "
            "it must be equal to 6");
      CHECK(!(cfg_gcmpSEIFunctionVAffectedByUFlag.values.size() == numFace),
            "Number of SEIGcmpFunctionVAffectedByUFlag must be equal to 5 when SEIGcmpPackingType is equal to 4 or 5, "
            "otherwise, it must be equal to 6");
      encCfg->m_seiCfg.m_gcmpSEIFunctionCoeffU.resize(numFace);
      encCfg->m_seiCfg.m_gcmpSEIFunctionUAffectedByVFlag.resize(numFace);
      encCfg->m_seiCfg.m_gcmpSEIFunctionCoeffV.resize(numFace);
      encCfg->m_seiCfg.m_gcmpSEIFunctionVAffectedByUFlag.resize(numFace);
    }
    for (int i = 0; i < numFace; i++)
    {
      encCfg->m_seiCfg.m_gcmpSEIFaceIndex[i]    = cfg_gcmpSEIFaceIndex.values[i];
      encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] = cfg_gcmpSEIFaceRotation.values[i];
      if (encCfg->m_seiCfg.m_gcmpSEIMappingFunctionType == 2)
      {
        encCfg->m_seiCfg.m_gcmpSEIFunctionCoeffU[i]           = cfg_gcmpSEIFunctionCoeffU.values[i];
        encCfg->m_seiCfg.m_gcmpSEIFunctionUAffectedByVFlag[i] = cfg_gcmpSEIFunctionUAffectedByVFlag.values[i];
        encCfg->m_seiCfg.m_gcmpSEIFunctionCoeffV[i]           = cfg_gcmpSEIFunctionCoeffV.values[i];
        encCfg->m_seiCfg.m_gcmpSEIFunctionVAffectedByUFlag[i] = cfg_gcmpSEIFunctionVAffectedByUFlag.values[i];
      }
    }
  }
  if (encCfg->m_seiCfg.m_sdiSEIEnabled)
  {
    if (encCfg->m_seiCfg.m_sdiSEIMultiviewInfoFlag || encCfg->m_seiCfg.m_sdiSEIAuxiliaryInfoFlag)
    {
      encCfg->m_seiCfg.m_sdiSEILayerId.resize(encCfg->m_seiCfg.m_sdiSEIMaxLayersMinus1 + 1);
      encCfg->m_seiCfg.m_sdiSEIViewIdVal.resize(encCfg->m_seiCfg.m_sdiSEIMaxLayersMinus1 + 1);
      encCfg->m_seiCfg.m_sdiSEIAuxId.resize(encCfg->m_seiCfg.m_sdiSEIMaxLayersMinus1 + 1);
      encCfg->m_seiCfg.m_sdiSEINumAssociatedPrimaryLayersMinus1.resize(encCfg->m_seiCfg.m_sdiSEIMaxLayersMinus1 + 1);
      for (int i = 0; i <= encCfg->m_seiCfg.m_sdiSEIMaxLayersMinus1; i++)
      {
        encCfg->m_seiCfg.m_sdiSEILayerId[i] = cfg_sdiSEILayerId.values[i];
        if (encCfg->m_seiCfg.m_sdiSEIMultiviewInfoFlag)
        {
          encCfg->m_seiCfg.m_sdiSEIViewIdVal[i] = cfg_sdiSEIViewIdVal.values[i];
        }
        if (encCfg->m_seiCfg.m_sdiSEIAuxiliaryInfoFlag)
        {
          encCfg->m_seiCfg.m_sdiSEIAuxId[i] = cfg_sdiSEIAuxId.values[i];
          if (encCfg->m_seiCfg.m_sdiSEIAuxId[i] > 0)
          {
            encCfg->m_seiCfg.m_sdiSEINumAssociatedPrimaryLayersMinus1[i] =
              cfg_sdiSEINumAssociatedPrimaryLayersMinus1.values[i];
          }
        }
      }
    }
  }
  if (encCfg->m_seiCfg.m_maiSEIEnabled)
  {
    if (encCfg->m_seiCfg.m_maiSEIIntrinsicParamFlag)
    {
      int numViews =
        encCfg->m_seiCfg.m_maiSEIIntrinsicParamsEqualFlag ? 1 : encCfg->m_seiCfg.m_maiSEINumViewsMinus1 + 1;
      encCfg->m_seiCfg.m_maiSEISignFocalLengthX.resize(numViews);
      encCfg->m_seiCfg.m_maiSEIExponentFocalLengthX.resize(numViews);
      encCfg->m_seiCfg.m_maiSEIMantissaFocalLengthX.resize(numViews);
      encCfg->m_seiCfg.m_maiSEISignFocalLengthY.resize(numViews);
      encCfg->m_seiCfg.m_maiSEIExponentFocalLengthY.resize(numViews);
      encCfg->m_seiCfg.m_maiSEIMantissaFocalLengthY.resize(numViews);
      encCfg->m_seiCfg.m_maiSEISignPrincipalPointX.resize(numViews);
      encCfg->m_seiCfg.m_maiSEIExponentPrincipalPointX.resize(numViews);
      encCfg->m_seiCfg.m_maiSEIMantissaPrincipalPointX.resize(numViews);
      encCfg->m_seiCfg.m_maiSEISignPrincipalPointY.resize(numViews);
      encCfg->m_seiCfg.m_maiSEIExponentPrincipalPointY.resize(numViews);
      encCfg->m_seiCfg.m_maiSEIMantissaPrincipalPointY.resize(numViews);
      encCfg->m_seiCfg.m_maiSEISignSkewFactor.resize(numViews);
      encCfg->m_seiCfg.m_maiSEIExponentSkewFactor.resize(numViews);
      encCfg->m_seiCfg.m_maiSEIMantissaSkewFactor.resize(numViews);
      for (int i = 0;
           i <= (encCfg->m_seiCfg.m_maiSEIIntrinsicParamsEqualFlag ? 0 : encCfg->m_seiCfg.m_maiSEINumViewsMinus1); i++)
      {
        encCfg->m_seiCfg.m_maiSEISignFocalLengthX[i]        = cfg_maiSEISignFocalLengthX.values[i];
        encCfg->m_seiCfg.m_maiSEIExponentFocalLengthX[i]    = cfg_maiSEIExponentFocalLengthX.values[i];
        encCfg->m_seiCfg.m_maiSEIMantissaFocalLengthX[i]    = cfg_maiSEIMantissaFocalLengthX.values[i];
        encCfg->m_seiCfg.m_maiSEISignFocalLengthY[i]        = cfg_maiSEISignFocalLengthY.values[i];
        encCfg->m_seiCfg.m_maiSEIExponentFocalLengthY[i]    = cfg_maiSEIExponentFocalLengthY.values[i];
        encCfg->m_seiCfg.m_maiSEIMantissaFocalLengthY[i]    = cfg_maiSEIMantissaFocalLengthY.values[i];
        encCfg->m_seiCfg.m_maiSEISignPrincipalPointX[i]     = cfg_maiSEISignPrincipalPointX.values[i];
        encCfg->m_seiCfg.m_maiSEIExponentPrincipalPointX[i] = cfg_maiSEIExponentPrincipalPointX.values[i];
        encCfg->m_seiCfg.m_maiSEIMantissaPrincipalPointX[i] = cfg_maiSEIMantissaPrincipalPointX.values[i];
        encCfg->m_seiCfg.m_maiSEISignPrincipalPointY[i]     = cfg_maiSEISignPrincipalPointY.values[i];
        encCfg->m_seiCfg.m_maiSEIExponentPrincipalPointY[i] = cfg_maiSEIExponentPrincipalPointY.values[i];
        encCfg->m_seiCfg.m_maiSEIMantissaPrincipalPointY[i] = cfg_maiSEIMantissaPrincipalPointY.values[i];
        encCfg->m_seiCfg.m_maiSEISignSkewFactor[i]          = cfg_maiSEISignSkewFactor.values[i];
        encCfg->m_seiCfg.m_maiSEIExponentSkewFactor[i]      = cfg_maiSEIExponentSkewFactor.values[i];
        encCfg->m_seiCfg.m_maiSEIMantissaSkewFactor[i]      = cfg_maiSEIMantissaSkewFactor.values[i];
      }
    }
  }
  if (encCfg->m_seiCfg.m_mvpSEIEnabled)
  {
    int numViews = encCfg->m_seiCfg.m_mvpSEINumViewsMinus1 + 1;
    encCfg->m_seiCfg.m_mvpSEIViewPosition.resize(numViews);
    for (int i = 0; i <= encCfg->m_seiCfg.m_mvpSEINumViewsMinus1; i++)
    {
      encCfg->m_seiCfg.m_mvpSEIViewPosition[i] = cfg_mvpSEIViewPosition.values[i];
    }
  }
  if (encCfg->m_seiCfg.m_driSEIEnabled)
  {
    encCfg->m_seiCfg.m_driSEINonlinearModel.resize(encCfg->m_seiCfg.m_driSEINonlinearNumMinus1 + 1);
    for (int i = 0; i < (encCfg->m_seiCfg.m_driSEINonlinearNumMinus1 + 1); i++)
    {
      encCfg->m_seiCfg.m_driSEINonlinearModel[i] =
        cfg_driSEINonlinearModel.values.size() > i ? cfg_driSEINonlinearModel.values[i] : 0;
    }
  }
  encCfg->m_reshapeCW.binCW.resize(3);
  encCfg->m_reshapeCW.rspFps     = encCfg->m_frameRate;
  encCfg->m_reshapeCW.rspPicSize = encCfg->m_sourceWidth * encCfg->m_sourceHeight;
  encCfg->m_reshapeCW.rspFpsToIp = std::max(16, 16 * (int)(round((double)encCfg->m_frameRate / 16.0)));
  encCfg->m_reshapeCW.rspBaseQP  = encCfg->m_iQP;
  encCfg->m_reshapeCW.updateCtrl = encCfg->m_updateCtrl;
  encCfg->m_reshapeCW.adpOption  = encCfg->m_adpOption;
  encCfg->m_reshapeCW.initialCW  = encCfg->m_initialCW;
#if ENABLE_TRACING
  g_trace_ctx = tracing_init(sTracingFile, sTracingRule);
  if (bTracingChannelsList && g_trace_ctx)
  {
    std::string sChannelsList;
    g_trace_ctx->getChannelsList(sChannelsList);
    msg(INFO, "\n Using tracing channels:\n\n%s\n", sChannelsList.c_str());
  }
#endif

#if ENABLE_QPA
  if (encCfg->m_bUsePerceptQPA && !encCfg->m_bUseAdaptiveQP && encCfg->m_dualITree &&
      (encCfg->m_chromaCbQpOffsetDualTree != 0 || encCfg->m_chromaCrQpOffsetDualTree != 0 ||
       encCfg->m_chromaCbCrQpOffsetDualTree != 0))
  {
    msg(WARNING, "*************************************************************************\n");
    msg(WARNING, "* WARNING: chroma QPA on, ignoring nonzero dual-tree chroma QP offsets! *\n");
    msg(WARNING, "*************************************************************************\n");
  }

#if ENABLE_QPA_SUB_CTU
  if ((encCfg->m_iQP < 38) && encCfg->m_bUsePerceptQPA && !encCfg->m_bUseAdaptiveQP &&
      (encCfg->m_sourceWidth <= 2048) && (encCfg->m_sourceHeight <= 1280)
#if WCG_EXT && ER_CHROMA_QP_WCG_PPS
      && (!encCfg->m_wcgChromaQpControl.enabled)
#endif
      && ((1 << (encCfg->m_log2MaxTbSize + 1)) == encCfg->m_CTUSize) &&
      (encCfg->m_sourceWidth > 512 || encCfg->m_sourceHeight > 320))
  {
    encCfg->m_cuQpDeltaSubdiv = 2;
  }
#else
  if ((encCfg->m_iQP < 38) && (encCfg->m_gopSize > 4) && encCfg->m_bUsePerceptQPA && !encCfg->m_bUseAdaptiveQP &&
      (encCfg->m_sourceHeight <= 1280) && (encCfg->m_sourceWidth <= 2048))
  {
    msg(WARNING, "*************************************************************************\n");
    msg(WARNING, "* WARNING: QPA on with large CTU for <=HD sequences, limiting CTU size! *\n");
    msg(WARNING, "*************************************************************************\n");

    encCfg->m_CTUSize = encCfg->m_maxCUWidth;
    if ((1u << encCfg->m_log2MaxTbSize) > encCfg->m_CTUSize)
    {
      encCfg->m_log2MaxTbSize--;
    }
  }
#endif
#endif   // ENABLE_QPA

#if JVET_Z0120_SII_SEI_PROCESSING
  encCfg->m_seiCfg.m_ShutterFilterEnable = false;
#endif
  if (encCfg->m_seiCfg.m_siiSEIEnabled)
  {
    assert(encCfg->m_seiCfg.m_siiSEITimeScale >= 0 && encCfg->m_seiCfg.m_siiSEITimeScale <= MAX_UINT);
    uint32_t sii_max_sub_layers = (uint32_t)cfg_siiSEIInputNumUnitsInSI.values.size();
    assert(sii_max_sub_layers > 0);
    if (sii_max_sub_layers > 1)
    {
      encCfg->m_seiCfg.m_siiSEISubLayerNumUnitsInSI.resize(sii_max_sub_layers);
      for (int32_t i = 0; i < sii_max_sub_layers; i++)
      {
        encCfg->m_seiCfg.m_siiSEISubLayerNumUnitsInSI[i] = cfg_siiSEIInputNumUnitsInSI.values[i];
        assert(encCfg->m_seiCfg.m_siiSEISubLayerNumUnitsInSI[i] >= 0 &&
               encCfg->m_seiCfg.m_siiSEISubLayerNumUnitsInSI[i] <= MAX_UINT);
      }
    }
    else
    {
      encCfg->m_seiCfg.m_siiSEINumUnitsInShutterInterval = cfg_siiSEIInputNumUnitsInSI.values[0];
      assert(encCfg->m_seiCfg.m_siiSEINumUnitsInShutterInterval >= 0 &&
             encCfg->m_seiCfg.m_siiSEINumUnitsInShutterInterval <= MAX_UINT);
    }
#if JVET_Z0120_SII_SEI_PROCESSING
    uint32_t siiMaxSubLayersMinus1 = sii_max_sub_layers - 1;
    int      blending_ratio        = (encCfg->m_seiCfg.m_siiSEISubLayerNumUnitsInSI[0] /
                          encCfg->m_seiCfg.m_siiSEISubLayerNumUnitsInSI[siiMaxSubLayersMinus1]);

    if (sii_max_sub_layers > 1 &&
        encCfg->m_seiCfg.m_siiSEISubLayerNumUnitsInSI[0] ==
          (blending_ratio * encCfg->m_seiCfg.m_siiSEISubLayerNumUnitsInSI[siiMaxSubLayersMinus1]))
    {
      encCfg->m_seiCfg.m_ShutterFilterEnable = true;
      double  fpsHFR                         = (double)encCfg->m_frameRate;
      int32_t i;
      bool    checkEqualValuesOfSFR = true;
      bool    checkSubLayerSI       = false;

      double shutterAngleFactor =
        (fpsHFR * ((double)(encCfg->m_seiCfg.m_siiSEISubLayerNumUnitsInSI[siiMaxSubLayersMinus1]))) /
        ((double)encCfg->m_seiCfg.m_siiSEITimeScale);

      // If shutterAngleFactor = 1 indicates that shutterAngle = 360
      // If shutterAngleFactor = 0.5 indicates that shutterAngle = 180
      // If shutterAngleFactor = 0.25 indicates that shutterAngle = 90

      if (shutterAngleFactor < 0.5)
      {
        for (int i = 0; i < siiMaxSubLayersMinus1; i++)
        {
          encCfg->m_seiCfg.m_siiSEISubLayerNumUnitsInSI[i] =
            encCfg->m_seiCfg.m_siiSEISubLayerNumUnitsInSI[siiMaxSubLayersMinus1];
        }
        encCfg->m_seiCfg.m_ShutterFilterEnable = false;
        printf("Warning: For the shutterAngle = %d, the blending can't be applied\n", (int)(shutterAngleFactor * 360));
      }
      // supports only the case of SFR = HFR / 2
      if (encCfg->m_seiCfg.m_siiSEISubLayerNumUnitsInSI[siiMaxSubLayersMinus1] <
          encCfg->m_seiCfg.m_siiSEISubLayerNumUnitsInSI[siiMaxSubLayersMinus1 - 1])
      {
        checkSubLayerSI = true;
      }
      // check shutter interval for all sublayer remains same for LFR pictures
      for (i = 1; i < siiMaxSubLayersMinus1; i++)
      {
        if (encCfg->m_seiCfg.m_siiSEISubLayerNumUnitsInSI[0] != encCfg->m_seiCfg.m_siiSEISubLayerNumUnitsInSI[i])
        {
          checkEqualValuesOfSFR = false;
        }
      }
      if (checkSubLayerSI && checkEqualValuesOfSFR)
      {
        encCfg->m_seiCfg.m_SII_BlendingRatio = blending_ratio;
      }
      else
      {
        encCfg->m_seiCfg.m_ShutterFilterEnable = false;
      }
    }
    else
    {
      printf("Warning: SII-processing is applied for multiple shutter intervals and number of LFR units should be 2 "
             "times of number of HFR units\n");
    }
#endif
  }

  if (encCfg->m_seiCfg.m_poSEIEnabled)
  {
    assert(cfg_poSEIPayloadType.values.size() > 1);
    assert(cfg_poSEIProcessingOrder.values.size() == cfg_poSEIPayloadType.values.size());
    encCfg->m_seiCfg.m_poSEIPayloadType.resize((uint32_t)cfg_poSEIPayloadType.values.size());
    encCfg->m_seiCfg.m_poSEIProcessingOrder.resize((uint32_t)cfg_poSEIPayloadType.values.size());
    encCfg->m_seiCfg.m_poSEIPrefixByte.resize((uint32_t)cfg_poSEIPayloadType.values.size());
    uint16_t prefixByteIdx = 0;
    for (uint32_t i = 0; i < (uint32_t)cfg_poSEIPayloadType.values.size(); i++)
    {
      encCfg->m_seiCfg.m_poSEIPayloadType[i]     = cfg_poSEIPayloadType.values[i];
      encCfg->m_seiCfg.m_poSEIProcessingOrder[i] = (uint16_t)cfg_poSEIProcessingOrder.values[i];
      if (encCfg->m_seiCfg.m_poSEIPayloadType[i] == (uint16_t)SEI::PayloadType::USER_DATA_REGISTERED_ITU_T_T35)
      {
        encCfg->m_seiCfg.m_poSEIPrefixByte[i].resize(cfg_poSEINumofPrefixByte.values[i]);
        for (uint32_t j = 0; j < cfg_poSEINumofPrefixByte.values[i]; j++)
        {
          encCfg->m_seiCfg.m_poSEIPrefixByte[i][j] = (uint8_t)cfg_poSEIPrefixByte.values[prefixByteIdx++];
        }
      }
      // Error check, to avoid same PayloadType and same prefix bytes when present with different PayloadOrder
      for (uint32_t j = 0; j < i; j++)
      {
        auto payloadType = SEI::PayloadType(cfg_poSEIPayloadType.values[i]);
        if (payloadType == SEI::PayloadType::USER_DATA_REGISTERED_ITU_T_T35)
        {
          for (uint32_t j = 0; j < i; j++)
          {
            if (encCfg->m_seiCfg.m_poSEIPayloadType[j] == encCfg->m_seiCfg.m_poSEIPayloadType[i])
            {
              auto numofPrefixBytes = std::min(cfg_poSEINumofPrefixByte.values[i], cfg_poSEINumofPrefixByte.values[j]);
              if (std::equal(encCfg->m_seiCfg.m_poSEIPrefixByte[i].begin() + 1,
                             encCfg->m_seiCfg.m_poSEIPrefixByte[i].begin() + numofPrefixBytes - 1,
                             encCfg->m_seiCfg.m_poSEIPrefixByte[j].begin()))
              {
                assert(encCfg->m_seiCfg.m_poSEIProcessingOrder[j] == encCfg->m_seiCfg.m_poSEIProcessingOrder[i]);
              }
            }
          }
        }
      }
    }
    // Error check, to avoid all SEI messages share the same PayloadOrder
    assert(!std::equal(cfg_poSEIProcessingOrder.values.begin() + 1, cfg_poSEIProcessingOrder.values.end(),
                       cfg_poSEIProcessingOrder.values.begin()));
    assert(encCfg->m_seiCfg.m_poSEIPayloadType.size() > 0);
    assert(encCfg->m_seiCfg.m_poSEIProcessingOrder.size() == encCfg->m_seiCfg.m_poSEIPayloadType.size());
  }

  if (encCfg->m_seiCfg.m_postFilterHintSEIEnabled)
  {
    CHECK(cfg_postFilterHintSEIValues.values.size() <= 0,
          "The number of filter coefficient shall be greater than zero");
    CHECK(!(cfg_postFilterHintSEIValues.values.size() ==
            ((encCfg->m_seiCfg.m_postFilterHintSEIChromaCoeffPresentFlag ? 3 : 1) *
             encCfg->m_seiCfg.m_postFilterHintSEISizeY * encCfg->m_seiCfg.m_postFilterHintSEISizeX)),
          "The number of filter coefficient shall match the matrix size and considering whether filters for chroma is "
          "present of not");
    encCfg->m_seiCfg.m_postFilterHintValues.resize(cfg_postFilterHintSEIValues.values.size());

    for (uint32_t i = 0; i < encCfg->m_seiCfg.m_postFilterHintValues.size(); i++)
    {
      encCfg->m_seiCfg.m_postFilterHintValues[i] = cfg_postFilterHintSEIValues.values[i];
    }
  }

  if (encCfg->m_costMode == COST_LOSSLESS_CODING)
  {
    bool firstSliceLossless = false;
    if (encCfg->m_mixedLossyLossless)
    {
      if (encCfg->m_sliceLosslessArray.size() > 0)
      {
        for (uint32_t i = 0; i < encCfg->m_sliceLosslessArray.size(); i++)
        {
          if (encCfg->m_sliceLosslessArray[i] == 0)
          {
            firstSliceLossless = true;
            break;
          }
        }
      }
    }
    else
    {
      firstSliceLossless = true;
    }
    if (firstSliceLossless)   // if first slice is lossless
    {
      encCfg->m_iQP =
        LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP - ((encCfg->m_internalBitDepth[ChannelType::LUMA] - 8) * 6);
    }
  }

  if (encCfg->m_ibcFracMode && (!encCfg->m_ImvMode || !encCfg->m_ibcMode))
  {
    encCfg->m_ibcFracMode = 0;
  }

  // check validity of input parameters
  if (xCheckParameter(encCfg))
  {
    // return check failed
    return false;
  }

  // print-out parameters
  xPrintParameter(encCfg);

  return true;
}
#ifdef _MSC_VER
// Restore optimizations
#pragma optimize("", on)
#endif

// ====================================================================================================================
// Private member functions
// ====================================================================================================================

bool EncAppCfg::xCheckParameter(EncCfg *encCfg)
{
  msg(NOTICE, "\n");
  if (encCfg->m_seiCfg.m_decodedPictureHashSEIType == HashType::NONE)
  {
    msg(DETAILS, "******************************************************************\n");
    msg(DETAILS, "** WARNING: --SEIDecodedPictureHash is now disabled by default. **\n");
    msg(DETAILS, "**          Automatic verification of decoded pictures by a     **\n");
    msg(DETAILS, "**          decoder requires this option to be enabled.         **\n");
    msg(DETAILS, "******************************************************************\n");
  }
  if (encCfg->m_profile == Profile::NONE)
  {
    msg(DETAILS, "***************************************************************************\n");
    msg(DETAILS, "** WARNING: For conforming bitstreams a valid Profile value must be set! **\n");
    msg(DETAILS, "***************************************************************************\n");
  }
  if (encCfg->m_level == Level::NONE)
  {
    msg(DETAILS, "***************************************************************************\n");
    msg(DETAILS, "** WARNING: For conforming bitstreams a valid Level value must be set!   **\n");
    msg(DETAILS, "***************************************************************************\n");
  }

  bool check_failed = false; /* abort if there is a fatal configuration problem */
#define xConfirmPara(a, b) check_failed |= confirmPara(a, b)

  xConfirmPara(encCfg->m_alfapsIDShift < 0, "ALF APSs shift should be positive");
  xConfirmPara(encCfg->m_alfapsIDShift + encCfg->m_maxNumAlfAps > AlfParameters::ALF_CTB_MAX_NUM_APS,
               "The number of ALF APSs should not be more than ALF_CTB_MAX_NUM_APS");

  xConfirmPara(encCfg->m_DepQuantEnabledIdc < 0 || encCfg->m_DepQuantEnabledIdc > 2,
               "DepQuant must be equal to 0, 1, or 2");

  if (encCfg->m_DepQuantEnabledIdc)
  {
    xConfirmPara(!encCfg->m_useRDOQ || !encCfg->m_useRDOQTS,
                 "RDOQ and RDOQTS must be equal to 1 if dependent quantization is enabled");
    xConfirmPara(encCfg->m_SignDataHidingEnabledFlag,
                 "SignHideFlag must be equal to 0 if dependent quantization is enabled");
  }

  xConfirmPara(encCfg->m_numPredSign < 0 || encCfg->m_numPredSign > SIGN_PRED_MAX_NUM,
               "Number of predicted coefficient signs must be positive value and no larger than SIGN_PRED_MAX_NUM");
  xConfirmPara(encCfg->m_numPredSign > 0 &&
                 (encCfg->m_log2SignPredArea < 2 || encCfg->m_log2SignPredArea > MAX_LOG2_SIGN_PRED_SIZE),
               "log2 of predicted sign area must be in between 2 and 5 inclusively");

  if (encCfg->m_wrapAround)
  {
    const int minCUSize = 1 << encCfg->m_log2MinCUSize;
    xConfirmPara(encCfg->m_wrapAroundOffset <= encCfg->m_CTUSize + minCUSize,
                 "Wrap-around offset must be greater than CtbSizeY + MinCbSize");
    xConfirmPara(encCfg->m_wrapAroundOffset > encCfg->m_sourceWidth,
                 "Wrap-around offset must not be greater than the source picture width");
    xConfirmPara(encCfg->m_wrapAroundOffset % minCUSize != 0,
                 "Wrap-around offset must be an integer multiple of the specified minimum CU size");
  }

#if SHARP_LUMA_DELTA_QP && ENABLE_QPA
  xConfirmPara(encCfg->m_bUsePerceptQPA && encCfg->m_lumaLevelToDeltaQPMapping.mode >= 2,
               "QPA and SharpDeltaQP mode 2 cannot be used together");
  if (encCfg->m_bUsePerceptQPA && encCfg->m_lumaLevelToDeltaQPMapping.mode == LUMALVL_TO_DQP_AVG_METHOD)
  {
    msg(WARNING, "*********************************************************************************\n");
    msg(WARNING, "** WARNING: Applying custom luma-based QPA with activity-based perceptual QPA! **\n");
    msg(WARNING, "*********************************************************************************\n");

    encCfg->m_lumaLevelToDeltaQPMapping.mode = LUMALVL_TO_DQP_NUM_MODES;   // special QPA mode
  }
#endif

  xConfirmPara(encCfg->m_useAMaxBT && !encCfg->m_useSplitConsOverride,
               "AMaxBt can only be used with PartitionConstriantsOverride enabled");
  xConfirmPara(encCfg->m_uiMaxMTTHierarchyDepth >= 10 && !encCfg->m_useSplitConsOverride,
               "MaxMTT depth can only be adapted per T-layer with PartitionConstriantsOverride enabled");
  xConfirmPara(encCfg->m_uiMaxMTTHierarchyDepth >= 10 && !encCfg->m_enablePictureHeaderInSliceHeader,
               "MaxMTT depth can only be adapted per T-layer with EnablePictureHeaderInSliceHeader enabled");

  xConfirmPara(encCfg->m_bitstreamFileName.empty(), "A bitstream file name must be specified (BitstreamFile)");
  xConfirmPara(encCfg->m_internalBitDepth[ChannelType::CHROMA] != encCfg->m_internalBitDepth[ChannelType::LUMA],
               "The internalBitDepth must be the same for luma and chroma");
  if (encCfg->m_profile != Profile::NONE)
  {
    xConfirmPara(encCfg->m_log2MaxTransformSkipBlockSize >= 6,
                 "Transform Skip Log2 Max Size must be less or equal to 5 for given profile.");
    xConfirmPara(encCfg->m_transformSkipRotationEnabledFlag == true,
                 "UseResidualRotation must not be enabled for given profile.");
    xConfirmPara(encCfg->m_transformSkipContextEnabledFlag == true,
                 "UseSingleSignificanceMapContext must not be enabled for given profile.");
    xConfirmPara(encCfg->m_highPrecisionOffsetsEnabledFlag == true,
                 "UseHighPrecisionPredictionWeighting must not be enabled for given profile.");
    xConfirmPara(encCfg->m_cabacBypassAlignmentEnabledFlag,
                 "AlignCABACBeforeBypass cannot be enabled for given profile.");
  }
  if (encCfg->m_profile != Profile::NONE && encCfg->m_profile != Profile::MAIN_12_444 &&
      encCfg->m_profile != Profile::MAIN_16_444 && encCfg->m_profile != Profile::MAIN_12_444_INTRA &&
      encCfg->m_profile != Profile::MAIN_16_444_INTRA && encCfg->m_profile != Profile::MAIN_12_444_STILL_PICTURE &&
      encCfg->m_profile != Profile::MAIN_12_444_STILL_PICTURE &&
      encCfg->m_profile != Profile::MAIN_16_444_STILL_PICTURE)
  {
    xConfirmPara(encCfg->m_rrcRiceExtensionEnableFlag == true,
                 "Extention of the Golomb-Rice parameter derivation for RRC must not be enabled for given profile.");
    xConfirmPara(encCfg->m_persistentRiceAdaptationEnabledFlag == true,
                 "GolombRiceParameterAdaption must not be enabled for given profile.");
    xConfirmPara(encCfg->m_extendedPrecisionProcessingFlag == true,
                 "UseExtendedPrecision must not be enabled for given profile.");
    xConfirmPara(encCfg->m_tsrcRicePresentFlag == true, "TSRCRicePresent must not be enabled for given profile.");
    xConfirmPara(encCfg->m_reverseLastSigCoeffEnabledFlag == true,
                 "ReverseLastSigCoeff must not be enabled for given profile.");
  }

  // check range of parameters
  xConfirmPara(encCfg->m_inputBitDepth[ChannelType::LUMA] < 8, "InputBitDepth must be at least 8");
  xConfirmPara(encCfg->m_inputBitDepth[ChannelType::CHROMA] < 8, "InputBitDepthC must be at least 8");

  if ((encCfg->m_internalBitDepth[ChannelType::LUMA] < encCfg->m_inputBitDepth[ChannelType::LUMA]) ||
      (encCfg->m_internalBitDepth[ChannelType::CHROMA] < encCfg->m_inputBitDepth[ChannelType::CHROMA]))
  {
    msg(WARNING, "*****************************************************************************\n");
    msg(WARNING, "** WARNING: InternalBitDepth is set to the lower value than InputBitDepth! **\n");
    msg(WARNING, "**          min_qp_prime_ts_minus4 will be clipped to 0 at the low end!    **\n");
    msg(WARNING, "*****************************************************************************\n");
  }

  if (encCfg->m_extendedPrecisionProcessingFlag)
  {
    for (const auto bd: encCfg->m_internalBitDepth)
    {
      xConfirmPara((bd > 8),
                   "Model is not configured to support high enough internal accuracies - enable "
                   "RExt__HIGH_BIT_DEPTH_SUPPORT to use increased precision internal data types etc...");
    }
  }
  else
  {
    for (const auto bd: encCfg->m_internalBitDepth)
    {
      xConfirmPara((bd > 12),
                   "Model is not configured to support high enough internal accuracies - enable "
                   "RExt__HIGH_BIT_DEPTH_SUPPORT to use increased precision internal data types etc...");
    }
  }

  xConfirmPara((encCfg->m_msbExtendedBitDepth[ChannelType::LUMA] < encCfg->m_inputBitDepth[ChannelType::LUMA]),
               "MSB-extended bit depth for luma channel (--MSBExtendedBitDepth) must be greater than or equal to input "
               "bit depth for luma channel (--InputBitDepth)");
  xConfirmPara((encCfg->m_msbExtendedBitDepth[ChannelType::CHROMA] < encCfg->m_inputBitDepth[ChannelType::CHROMA]),
               "MSB-extended bit depth for chroma channel (--MSBExtendedBitDepthC) must be greater than or equal to "
               "input bit depth for chroma channel (--InputBitDepthC)");

  bool check_sps_range_extension_flag = encCfg->m_extendedPrecisionProcessingFlag ||
    encCfg->m_rrcRiceExtensionEnableFlag || encCfg->m_persistentRiceAdaptationEnabledFlag ||
    encCfg->m_tsrcRicePresentFlag;
  if (encCfg->m_internalBitDepth[ChannelType::LUMA] <= 10)
  {
    xConfirmPara((check_sps_range_extension_flag == 1),
                 "RExt tools (Extended Precision Processing, RRC Rice Extension, Persistent Rice Adaptation and TSRC "
                 "Rice Extension) must be disabled for BitDepth is less than or equal to 10 (the value of "
                 "sps_range_extension_flag shall be 0 when BitDepth is less than or equal to 10.)");
  }
  xConfirmPara(encCfg->m_chromaFormatIdc >= ChromaFormat::NUM, "ChromaFormatIDC must be either 400, 420, 422 or 444");
  std::string sTempIPCSC = "InputColourSpaceConvert must be empty, " + getListOfColourSpaceConverts(true);
  xConfirmPara(encCfg->m_inputColourSpaceConvert >= NUMBER_INPUT_COLOUR_SPACE_CONVERSIONS, sTempIPCSC.c_str());
  xConfirmPara(encCfg->m_inputChromaFormatIDC >= ChromaFormat::NUM,
               "InputChromaFormatIDC must be either 400, 420, 422 or 444");
  xConfirmPara(encCfg->m_frameRate <= 0, "Frame rate must be more than 1");
  xConfirmPara(encCfg->m_framesToBeEncoded <= 0, "Total Number Of Frames encoded must be more than 0");
  xConfirmPara(encCfg->m_framesToBeEncoded < encCfg->m_switchPOC, "debug POC out of range");

  xConfirmPara(encCfg->m_gopSize < 1, "GOP Size must be greater or equal to 1");
  xConfirmPara(encCfg->m_gopSize > 1 && encCfg->m_gopSize % 2,
               "GOP Size must be a multiple of 2, if GOP Size is greater than 1");
  xConfirmPara((encCfg->m_intraPeriod > 0 && encCfg->m_intraPeriod < encCfg->m_gopSize) || encCfg->m_intraPeriod == 0,
               "Intra period must be more than GOP size, or -1 , not 0");
  xConfirmPara(encCfg->m_drapPeriod < 0, "DRAP period must be greater or equal to 0");
  xConfirmPara(encCfg->m_edrapPeriod < 0, "EDRAP period must be greater or equal to 0");
  xConfirmPara(encCfg->m_decodingRefreshType < 0 || encCfg->m_decodingRefreshType > 3,
               "Decoding Refresh Type must be comprised between 0 and 3 included");

  if (encCfg->m_fieldSeqFlag)
  {
    if (!encCfg->m_seiCfg.m_frameFieldInfoSEIEnabled)
    {
      msg(WARNING, "*************************************************************************************\n");
      msg(WARNING, "** WARNING: Frame field information SEI should be enabled for field coding!        **\n");
      msg(WARNING, "*************************************************************************************\n");
    }
  }
  if (encCfg->m_seiCfg.m_pictureTimingSEIEnabled && (!encCfg->m_seiCfg.m_bufferingPeriodSEIEnabled))
  {
    msg(WARNING, "****************************************************************************\n");
    msg(WARNING, "** WARNING: Picture Timing SEI requires Buffering Period SEI. Disabling.  **\n");
    msg(WARNING, "****************************************************************************\n");
    encCfg->m_seiCfg.m_pictureTimingSEIEnabled = false;
  }

  xConfirmPara(encCfg->m_seiCfg.m_bufferingPeriodSEIEnabled == true && encCfg->m_RCCpbSize == 0,
               "RCCpbSize must be greater than zero, when buffering period SEI is enabled");

  xConfirmPara(encCfg->m_log2MaxTransformSkipBlockSize < 2, "Transform Skip Log2 Max Size must be at least 2 (4x4)");

  xConfirmPara(encCfg->m_onePictureOnlyConstraintFlag && encCfg->m_framesToBeEncoded != 1,
               "When onePictureOnlyConstraintFlag is true, the number of frames to be encoded must be 1");
  if (encCfg->m_profile != Profile::NONE)
  {
    const ProfileFeatures *features = ProfileFeatures::getProfileFeatures(encCfg->m_profile);
    CHECK(features->profile != encCfg->m_profile, "Profile not found");
    xConfirmPara(encCfg->m_level == Level::LEVEL15_5 && !features->canUseLevel15p5,
                 "Profile does not support level 15.5");
    xConfirmPara(encCfg->m_level < Level::LEVEL4 && encCfg->m_tier == Level::HIGH,
                 "High tier not defined for levels below 4.");
  }

  xConfirmPara(encCfg->m_iQP < -6 * (encCfg->m_internalBitDepth[ChannelType::LUMA] - 8) || encCfg->m_iQP > MAX_QP,
               "QP exceeds supported range (-QpBDOffsety to 63)");
  xConfirmPara(encCfg->m_deblockingFilterMetric != 0 &&
                 (encCfg->m_deblockingFilterDisable || encCfg->m_deblockingFilterOffsetInPPS),
               "If DeblockingFilterMetric is non-zero then both LoopFilterDisable and LoopFilterOffsetInPPS must be 0");
  xConfirmPara(encCfg->m_deblockingFilterBetaOffsetDiv2 < -12 || encCfg->m_deblockingFilterBetaOffsetDiv2 > 12,
               "Loop Filter Beta Offset div. 2 exceeds supported range (-12 to 12");
  xConfirmPara(encCfg->m_deblockingFilterTcOffsetDiv2 < -12 || encCfg->m_deblockingFilterTcOffsetDiv2 > 12,
               "Loop Filter Tc Offset div. 2 exceeds supported range (-12 to 12)");
  xConfirmPara(encCfg->m_deblockingFilterCbBetaOffsetDiv2 < -12 || encCfg->m_deblockingFilterCbBetaOffsetDiv2 > 12,
               "Loop Filter Beta Offset div. 2 exceeds supported range (-12 to 12");
  xConfirmPara(encCfg->m_deblockingFilterCbTcOffsetDiv2 < -12 || encCfg->m_deblockingFilterCbTcOffsetDiv2 > 12,
               "Loop Filter Tc Offset div. 2 exceeds supported range (-12 to 12)");
  xConfirmPara(encCfg->m_deblockingFilterCrBetaOffsetDiv2 < -12 || encCfg->m_deblockingFilterCrBetaOffsetDiv2 > 12,
               "Loop Filter Beta Offset div. 2 exceeds supported range (-12 to 12");
  xConfirmPara(encCfg->m_deblockingFilterCrTcOffsetDiv2 < -12 || encCfg->m_deblockingFilterCrTcOffsetDiv2 > 12,
               "Loop Filter Tc Offset div. 2 exceeds supported range (-12 to 12)");
  xConfirmPara(encCfg->m_searchRange < 0, "Search Range must be more than 0");
  xConfirmPara(encCfg->m_bipredSearchRange < 0, "Bi-prediction refinement search range must be more than 0");
  xConfirmPara(encCfg->m_minSearchWindow < 0,
               "Minimum motion search window size for the adaptive window ME must be greater than or equal to 0");
  xConfirmPara(encCfg->m_iMaxDeltaQP > MAX_DELTA_QP, "Absolute Delta QP exceeds supported range (0 to 7)");
#if ENABLE_QPA
  xConfirmPara(encCfg->m_bUsePerceptQPA && encCfg->m_uiDeltaQpRD > 0,
               "Perceptual QPA cannot be used together with slice-level multiple-QP optimization");
#endif
#if SHARP_LUMA_DELTA_QP
  xConfirmPara(encCfg->m_lumaLevelToDeltaQPMapping.mode && encCfg->m_uiDeltaQpRD > 0,
               "Luma-level-based Delta QP cannot be used together with slice level multiple-QP optimization\n");
  xConfirmPara(encCfg->m_lumaLevelToDeltaQPMapping.mode && encCfg->m_RCEnableRateControl,
               "Luma-level-based Delta QP cannot be used together with rate control\n");
#endif
  if (encCfg->m_lumaLevelToDeltaQPMapping.mode && encCfg->m_lmcsEnabled)
  {
    msg(WARNING,
        "For HDR-PQ, LMCS should be used mutual-exclusively with Luma-level-based Delta QP. If use LMCS, turn lumaDQP "
        "off.\n");
    encCfg->m_lumaLevelToDeltaQPMapping.mode = LUMALVL_TO_DQP_DISABLED;
  }
  if (!encCfg->m_lmcsEnabled)
  {
    encCfg->m_reshapeSignalType = RESHAPE_SIGNAL_NULL;
    encCfg->m_intraCMD          = 0;
  }
  if (encCfg->m_lmcsEnabled && encCfg->m_reshapeSignalType == RESHAPE_SIGNAL_PQ)
  {
    encCfg->m_intraCMD = 1;
  }
  else if (encCfg->m_lmcsEnabled &&
           (encCfg->m_reshapeSignalType == RESHAPE_SIGNAL_SDR || encCfg->m_reshapeSignalType == RESHAPE_SIGNAL_HLG))
  {
    encCfg->m_intraCMD = 0;
  }
  else
  {
    encCfg->m_lmcsEnabled = false;
  }
  if (encCfg->m_lmcsEnabled)
  {
    xConfirmPara(encCfg->m_updateCtrl < 0, "Min. LMCS Update Control is 0");
    xConfirmPara(encCfg->m_updateCtrl > 2, "Max. LMCS Update Control is 2");
    xConfirmPara(encCfg->m_adpOption < 0, "Min. LMCS Adaptation Option is 0");
    xConfirmPara(encCfg->m_adpOption > 4, "Max. LMCS Adaptation Option is 4");
    xConfirmPara(encCfg->m_initialCW < 0, "Min. Initial Total Codeword is 0");
    xConfirmPara(encCfg->m_initialCW > 1023, "Max. Initial Total Codeword is 1023");
    xConfirmPara(encCfg->m_CSoffset < -7, "Min. LMCS Offset value is -7");
    xConfirmPara(encCfg->m_CSoffset > 7, "Max. LMCS Offset value is 7");
    if (encCfg->m_updateCtrl > 0 && encCfg->m_adpOption > 2)
    {
      encCfg->m_adpOption -= 2;
    }
  }

  if (!encCfg->m_LMChroma)
  {
    encCfg->m_ccMerge       = false;
    encCfg->m_ccMergeFusion = false;
    encCfg->m_ccBoostFilter = false;
  }
  if (!encCfg->m_CCCM)
  {
    encCfg->m_bvgCccm          = false;
    encCfg->m_ccBoostTplRefSel = false;
    encCfg->m_ccDecDerivedMode = false;
  }

  if (encCfg->m_seiCfg.m_ctiSEIEnabled)
  {
    xConfirmPara(encCfg->m_seiCfg.m_ctiSEINumberChromaLut < 0 || encCfg->m_seiCfg.m_ctiSEINumberChromaLut > 2,
                 "CTI number of chroma LUTs is out of range");
  }
  xConfirmPara(encCfg->m_chromaCbQpOffset < -12, "Min. Chroma Cb QP Offset is -12");
  xConfirmPara(encCfg->m_chromaCbQpOffset > 12, "Max. Chroma Cb QP Offset is  12");
  xConfirmPara(encCfg->m_chromaCrQpOffset < -12, "Min. Chroma Cr QP Offset is -12");
  xConfirmPara(encCfg->m_chromaCrQpOffset > 12, "Max. Chroma Cr QP Offset is  12");
  xConfirmPara(encCfg->m_chromaCbQpOffsetDualTree < -12, "Min. Chroma Cb QP Offset for dual tree is -12");
  xConfirmPara(encCfg->m_chromaCbQpOffsetDualTree > 12, "Max. Chroma Cb QP Offset for dual tree is  12");
  xConfirmPara(encCfg->m_chromaCrQpOffsetDualTree < -12, "Min. Chroma Cr QP Offset for dual tree is -12");
  xConfirmPara(encCfg->m_chromaCrQpOffsetDualTree > 12, "Max. Chroma Cr QP Offset for dual tree is  12");
  if (encCfg->m_dualITree && !isChromaEnabled(encCfg->m_chromaFormatIdc))
  {
    msg(WARNING, "****************************************************************************\n");
    msg(WARNING, "** WARNING: --DualITree has been disabled because the chromaFormat is 400 **\n");
    msg(WARNING, "****************************************************************************\n");
    encCfg->m_dualITree = false;
  }
  if (encCfg->m_alf)
  {
    xConfirmPara(encCfg->m_alfStrengthLuma < 0.0,
                 "ALFStrengthLuma is less than 0. Valid range is 0.0 <= ALFStrengthLuma <= 1.0");
    xConfirmPara(encCfg->m_alfStrengthLuma > 1.0,
                 "ALFStrengthLuma is greater than 1. Valid range is 0.0 <= ALFStrengthLuma <= 1.0");
  }
  if (encCfg->m_ccalf)
  {
    xConfirmPara(encCfg->m_ccalfStrength < 0.0,
                 "CCALFStrength is less than 0. Valid range is 0.0 <= CCALFStrength <= 1.0");
    xConfirmPara(encCfg->m_ccalfStrength > 1.0,
                 "CCALFStrength is greater than 1. Valid range is 0.0 <= CCALFStrength <= 1.0");
  }
  if (encCfg->m_alf)
  {
    xConfirmPara(encCfg->m_alfStrengthChroma < 0.0,
                 "ALFStrengthChroma is less than 0. Valid range is 0.0 <= ALFStrengthChroma <= 1.0");
    xConfirmPara(encCfg->m_alfStrengthChroma > 1.0,
                 "ALFStrengthChroma is greater than 1. Valid range is 0.0 <= ALFStrengthChroma <= 1.0");
    xConfirmPara(encCfg->m_alfStrengthTargetLuma < 0.0,
                 "ALFStrengthTargetLuma is less than 0. Valid range is 0.0 <= ALFStrengthTargetLuma <= 1.0");
    xConfirmPara(encCfg->m_alfStrengthTargetLuma > 1.0,
                 "ALFStrengthTargetLuma is greater than 1. Valid range is 0.0 <= ALFStrengthTargetLuma <= 1.0");
    xConfirmPara(encCfg->m_alfStrengthTargetChroma < 0.0,
                 "ALFStrengthTargetChroma is less than 0. Valid range is 0.0 <= ALFStrengthTargetChroma <= 1.0");
    xConfirmPara(encCfg->m_alfStrengthTargetChroma > 1.0,
                 "ALFStrengthTargetChroma is greater than 1. Valid range is 0.0 <= ALFStrengthTargetChroma <= 1.0");
  }
  if (encCfg->m_ccalf)
  {
    xConfirmPara(encCfg->m_ccalfStrengthTarget < 0.0,
                 "CCALFStrengthTarget is less than 0. Valid range is 0.0 <= CCALFStrengthTarget <= 1.0");
    xConfirmPara(encCfg->m_ccalfStrengthTarget > 1.0,
                 "CCALFStrengthTarget is greater than 1. Valid range is 0.0 <= CCALFStrengthTarget <= 1.0");
  }
  if (encCfg->m_ccalf && !isChromaEnabled(encCfg->m_chromaFormatIdc))
  {
    msg(WARNING, "****************************************************************************\n");
    msg(WARNING, "** WARNING: --CCALF has been disabled because the chromaFormat is 400     **\n");
    msg(WARNING, "****************************************************************************\n");
    encCfg->m_ccalf = false;
  }
  if (encCfg->m_alfImprovements)
  {
    xConfirmPara(encCfg->m_alf != true, "ALF improvements must not be enabled when ALF is disabled.");
  }
  if (encCfg->m_jointCbCrMode && !isChromaEnabled(encCfg->m_chromaFormatIdc))
  {
    msg(WARNING, "****************************************************************************\n");
    msg(WARNING, "** WARNING: --JointCbCr has been disabled because the chromaFormat is 400 **\n");
    msg(WARNING, "****************************************************************************\n");
    encCfg->m_jointCbCrMode = false;
  }

  if (encCfg->m_chromaBIF && !isChromaEnabled(encCfg->m_chromaFormatIdc))
  {
    msg(WARNING, "****************************************************************************\n");
    msg(WARNING, "** WARNING: --ChromaBIF has been disabled because the chromaFormat is 400 **\n");
    msg(WARNING, "****************************************************************************\n");
    encCfg->m_chromaBIF = false;
  }
  if (encCfg->m_CCSAO && !isChromaEnabled(encCfg->m_chromaFormatIdc))
  {
    msg(WARNING, "****************************************************************************\n");
    msg(WARNING, "** WARNING: --CCSAO has been disabled because the chromaFormat is 400 **\n");
    msg(WARNING, "****************************************************************************\n");
    encCfg->m_CCSAO = 0;
  }
  if (encCfg->m_jointCbCrMode)
  {
    xConfirmPara(encCfg->m_chromaCbCrQpOffset < -12, "Min. Joint Cb-Cr QP Offset is -12");
    xConfirmPara(encCfg->m_chromaCbCrQpOffset > 12, "Max. Joint Cb-Cr QP Offset is  12");
    xConfirmPara(encCfg->m_chromaCbCrQpOffsetDualTree < -12, "Min. Joint Cb-Cr QP Offset for dual tree is -12");
    xConfirmPara(encCfg->m_chromaCbCrQpOffsetDualTree > 12, "Max. Joint Cb-Cr QP Offset for dual tree is  12");
  }
  xConfirmPara(encCfg->m_iQPAdaptationRange <= 0, "QP Adaptation Range must be more than 0");
  if (encCfg->m_decodingRefreshType == 2)
  {
    xConfirmPara(encCfg->m_intraPeriod > 0 && encCfg->m_intraPeriod <= encCfg->m_gopSize,
                 "Intra period must be larger than GOP size for periodic IDR pictures");
  }

  const int minCuSize = 1 << encCfg->m_log2MinCUSize;
  xConfirmPara(encCfg->m_minQt[0] > MAX_CU_SIZE, "Min Luma QT size in I slices should be smaller than or equal to 64");
  xConfirmPara(encCfg->m_minQt[1] > MAX_CU_SIZE,
               "Min Luma QT size in non-I slices should be smaller than or equal to 64");
  xConfirmPara(encCfg->m_maxBt[2] > MAX_CU_SIZE,
               "Maximum BT size for chroma block in I slice should be smaller than or equal to 64");
  xConfirmPara(encCfg->m_maxTt[0] > MAX_CU_SIZE,
               "Maximum TT size for luma block in I slice should be smaller than or equal to 64");
  xConfirmPara(encCfg->m_maxTt[1] > MAX_CU_SIZE,
               "Maximum TT size for luma block in non-I slice should be smaller than or equal to 64");
  xConfirmPara(encCfg->m_maxTt[2] > MAX_CU_SIZE,
               "Maximum TT size for chroma block in I slice should be smaller than or equal to 64");
  xConfirmPara(encCfg->m_minQt[0] < minCuSize,
               "Min Luma QT size in I slices should be larger than or equal to minCuSize");
  xConfirmPara(encCfg->m_minQt[1] < minCuSize,
               "Min Luma QT size in non-I slices should be larger than or equal to minCuSize");
  xConfirmPara((encCfg->m_sourceWidth % minCuSize) || (encCfg->m_sourceHeight % minCuSize),
               "Picture width or height is not a multiple of minCuSize");
  const int minDiff = (int)floorLog2(encCfg->m_minQt[2]) -
    std::max(MIN_CU_LOG2,
             (int)encCfg->m_log2MinCUSize - (int)getChannelTypeScaleX(ChannelType::CHROMA, encCfg->m_chromaFormatIdc));
  xConfirmPara(minDiff < 0,
               "Min Chroma QT size in I slices is smaller than Min Luma CU size even considering color format");
  xConfirmPara((encCfg->m_minQt[2] << (int)getChannelTypeScaleX(ChannelType::CHROMA, encCfg->m_chromaFormatIdc)) >
                 std::min(64, (int)encCfg->m_CTUSize),
               "Min Chroma QT size in I slices should be smaller than or equal to CTB size or CB size after implicit "
               "split of CTB");
  xConfirmPara(encCfg->m_CTUSize < (MAX_CU_SIZE >> 2),
               "CTUSize must be greater than or equal to a quarter of max CU size");
  xConfirmPara(encCfg->m_CTUSize > MAX_CU_SIZE, "CTUSize must be less than or equal to 256");
  xConfirmPara((encCfg->m_CTUSize << 1) != (encCfg->m_CTUSize | (encCfg->m_CTUSize - 1)) + 1,
               "CTUSize must be a power of 2 (32, 64, or 128)");
  xConfirmPara(encCfg->m_maxBt[0] < encCfg->m_minQt[0],
               "Maximum BT size for luma block in I slice should be larger than minimum QT size");
  xConfirmPara(encCfg->m_maxBt[0] > encCfg->m_CTUSize,
               "Maximum BT size for luma block in I slice should be smaller than or equal to CTUSize");
  xConfirmPara(encCfg->m_maxBt[1] < encCfg->m_minQt[1],
               "Maximum BT size for luma block in non I slice should be larger than minimum QT size");
  xConfirmPara(encCfg->m_maxBt[1] > encCfg->m_CTUSize,
               "Maximum BT size for luma block in non I slice should be smaller than or equal to CTUSize");
  xConfirmPara(encCfg->m_maxBt[2] <
                 (encCfg->m_minQt[2] << (int)getChannelTypeScaleX(ChannelType::CHROMA, encCfg->m_chromaFormatIdc)),
               "Maximum BT size for chroma block in I slice should be larger than minimum QT size");
  xConfirmPara(encCfg->m_maxBt[2] > encCfg->m_CTUSize,
               "Maximum BT size for chroma block in I slice should be smaller than or equal to CTUSize");
  xConfirmPara(encCfg->m_maxTt[0] < encCfg->m_minQt[0],
               "Maximum TT size for luma block in I slice should be larger than minimum QT size");
  xConfirmPara(encCfg->m_maxTt[0] > encCfg->m_CTUSize,
               "Maximum TT size for luma block in I slice should be smaller than or equal to CTUSize");
  xConfirmPara(encCfg->m_maxTt[1] < encCfg->m_minQt[1],
               "Maximum TT size for luma block in non I slice should be larger than minimum QT size");
  xConfirmPara(encCfg->m_maxTt[1] > encCfg->m_CTUSize,
               "Maximum TT size for luma block in non I slice should be smaller than or equal to CTUSize");
  xConfirmPara(encCfg->m_maxTt[2] <
                 (encCfg->m_minQt[2] << (int)getChannelTypeScaleX(ChannelType::CHROMA, encCfg->m_chromaFormatIdc)),
               "Maximum TT size for chroma block in I slice should be larger than minimum QT size");
  xConfirmPara(encCfg->m_maxTt[2] > encCfg->m_CTUSize,
               "Maximum TT size for chroma block in I slice should be smaller than or equal to CTUSize");
  xConfirmPara((encCfg->m_sourceWidth % (std::max(8u, encCfg->m_log2MinCUSize))) != 0,
               "Resulting coded frame width must be a multiple of Max(8, the minimum CU size)");
  xConfirmPara((encCfg->m_sourceHeight % (std::max(8u, encCfg->m_log2MinCUSize))) != 0,
               "Resulting coded frame height must be a multiple of Max(8, the minimum CU size)");
  if (encCfg->m_uiMaxMTTHierarchyDepthI == 0)
  {
    encCfg->m_maxBt[0] = encCfg->m_minQt[0];
    encCfg->m_maxTt[0] = encCfg->m_minQt[0];
  }
  if (encCfg->m_uiMaxMTTHierarchyDepthIChroma == 0)
  {
    encCfg->m_maxBt[2] = encCfg->m_minQt[2]
      << (int)getChannelTypeScaleX(ChannelType::CHROMA, encCfg->m_chromaFormatIdc);
    encCfg->m_maxTt[2] = encCfg->m_minQt[2]
      << (int)getChannelTypeScaleX(ChannelType::CHROMA, encCfg->m_chromaFormatIdc);
  }
  if (encCfg->m_uiMaxMTTHierarchyDepth == 0)
  {
    encCfg->m_maxBt[1] = encCfg->m_minQt[1];
    encCfg->m_maxTt[1] = encCfg->m_minQt[1];
  }
  xConfirmPara(encCfg->m_log2MaxTbSize > 8, "Log2MaxTbSize must be 8 or smaller.");
  xConfirmPara(encCfg->m_log2MaxTbSize < 5, "Log2MaxTbSize must be 5 or greater.");
  xConfirmPara(encCfg->m_maxNumMergeCand < 1, "MaxNumMergeCand must be 1 or greater.");
  xConfirmPara(encCfg->m_maxNumMergeCand > MRG_MAX_NUM_CANDS,
               "MaxNumMergeCand must be no more than MRG_MAX_NUM_CANDS.");
  xConfirmPara(encCfg->m_maxNumGeoCand > GEO_MAX_NUM_UNI_CANDS,
               "MaxNumGeoCand must be no more than GEO_MAX_NUM_UNI_CANDS.");
  xConfirmPara(encCfg->m_maxNumGeoCand > encCfg->m_maxNumMergeCand,
               "MaxNumGeoCand must be no more than MaxNumMergeCand.");
  xConfirmPara(0 < encCfg->m_maxNumGeoCand && encCfg->m_maxNumGeoCand < 2,
               "MaxNumGeoCand must be no less than 2 unless MaxNumGeoCand is 0.");
  xConfirmPara(encCfg->m_maxNumIBCMergeCand < 1, "MaxNumIBCMergeCand must be 1 or greater.");
  xConfirmPara(encCfg->m_maxNumIBCMergeCand > IBC_MRG_MAX_NUM_CANDS,
               "MaxNumIBCMergeCand must be no more than IBC_MRG_MAX_NUM_CANDS.");
  xConfirmPara(encCfg->m_maxNumAffineMergeCand < (encCfg->m_sbTmvpEnableFlag ? 1 : 0),
               "MaxNumAffineMergeCand must be greater than 0 when SbTMVP is enabled");
  xConfirmPara(encCfg->m_maxNumAffineMergeCand > AFFINE_MRG_MAX_NUM_CANDS,
               "MaxNumAffineMergeCand must be no more than AFFINE_MRG_MAX_NUM_CANDS.");
  xConfirmPara(encCfg->m_maxNumBMMergeCand > BM_MRG_MAX_NUM_CANDS,
               "MaxNumBMMergeCand must be no more than BM_MRG_MAX_NUM_CANDS.");
  constexpr int maxCandNum = NUM_MRG_SATD_CAND + 1 + NUM_AFF_MRG_SATD_CAND + GEO_MAX_TRY_WEIGHTED_SATD;
  // Note: maxCandNum=15 is an empirical value for the number of candidate in RD checking
  // Limit maximum value of MaxMergeRdCandNumTotal to maxCandNum. Larger values are not expected to be beneficial
  xConfirmPara(encCfg->m_maxMergeRdCandNumTotal < 1 || encCfg->m_maxMergeRdCandNumTotal > maxCandNum,
               "MaxMergeRdCandNumTotal must be between 1 and 15, inclusive");
  xConfirmPara(encCfg->m_mergeRdCandQuotaRegular < 0 || encCfg->m_mergeRdCandQuotaRegular > maxCandNum ||
                 encCfg->m_mergeRdCandQuotaRegularSmallBlk < 0 ||
                 encCfg->m_mergeRdCandQuotaRegularSmallBlk > maxCandNum || encCfg->m_mergeRdCandQuotaSubBlk < 0 ||
                 encCfg->m_mergeRdCandQuotaSubBlk > maxCandNum || encCfg->m_mergeRdCandQuotaCiip < 0 ||
                 encCfg->m_mergeRdCandQuotaCiip > maxCandNum || encCfg->m_mergeRdCandQuotaGpm < 0 ||
                 encCfg->m_mergeRdCandQuotaGpm > maxCandNum,
               "MaxMergeRdCandNumReguar, MaxMergeRdCandNumReguarSmallBlk, MaxMergeRdCandNumSubBlk, "
               "MaxMergeRdCandNumCiip, and MaxMergeRdCandNumGpm must be between 0 and 15, inclusive");
  if (encCfg->m_Affine == 0)
  {
    xConfirmPara(encCfg->m_AffineMmvdMode, "Affine MMVD can't be enabled if Affine is disabled.");
    encCfg->m_maxNumAffineMergeCand            = encCfg->m_sbTmvpEnableFlag ? 1 : 0;
    encCfg->m_maxNumAffineOppositeLicMergeCand = 0;
    if (encCfg->m_PROF)
    {
      msg(WARNING, "PROF is forcefully disabled when Affine is off \n");
    }
    encCfg->m_PROF = false;
  }
  if (encCfg->m_mergeOppositeLic)
  {
    xConfirmPara(encCfg->m_maxNumOppositeLicMergeCand < 1, "MaxNumOppositeLicMergeCand must not be 0");
    xConfirmPara(encCfg->m_maxNumOppositeLicMergeCand > encCfg->m_maxNumMergeCand,
                 "MaxNumOppositeLicMergeCand must not be larger than MaxNumMergeCand");
    xConfirmPara(encCfg->m_maxNumAffineOppositeLicMergeCand > encCfg->m_maxNumAffineMergeCand,
                 "MaxNumAffineOppositeLicMergeCand must not be larger than MaxNumAffineMergeCand");
    xConfirmPara(encCfg->m_maxNumOppositeLicMergeCand > REG_MRG_MAX_NUM_CANDS_OPPOSITELIC,
                 "MaxNumOppositeLicMergeCand must not be larger than REG_MRG_MAX_NUM_CANDS_OPPOSITELIC");
    xConfirmPara(encCfg->m_maxNumAffineOppositeLicMergeCand > AFF_MRG_MAX_NUM_CANDS_OPPOSITELIC,
                 "MaxNumAffineOppositeLicMergeCand must not be larger than AFF_MRG_MAX_NUM_CANDS_OPPOSITELIC");
  }
  xConfirmPara((encCfg->m_minAffineBlkSize << 1) != (encCfg->m_minAffineBlkSize | (encCfg->m_minAffineBlkSize - 1)) + 1,
               "MinAffineBlkSize must be a power of 2 (8, 16, ...)");
  xConfirmPara(encCfg->m_minAffineBlkSize < 8, "MinAffineBlkSize must be greater than 4");

  xConfirmPara(encCfg->m_mtsMode < 0 || encCfg->m_mtsMode > 4, "MTS must in the range 0..4");
  xConfirmPara(encCfg->m_mtsMode != 0 && encCfg->m_implicitMtsIntra != 0,
               "MTSImplicit may be enabled only when MTS is 0");
  xConfirmPara(encCfg->m_interLFNSTSBT && !encCfg->m_interLFNST,
               "LFNST for SBT can only be enabled when inter LFNST is enabled");

  xConfirmPara(encCfg->m_useFastMIP < 0 || encCfg->m_useFastMIP > 4,
               "FastMip must be greater than 0 and smaller than 5");

  if (encCfg->m_tsrcRicePresentFlag)
  {
    xConfirmPara(!encCfg->m_useTransformSkip, "TSRCRicePresent cannot be enabled when transform skip is disabled.");
  }

  if (!encCfg->m_alf)
  {
    xConfirmPara(encCfg->m_ccalf, "CCALF cannot be enabled when ALF is disabled");
  }

  if (encCfg->m_maxNumAlfAps == 0)
  {
    xConfirmPara(encCfg->m_ccalf, "CCALF cannot be enabled when ALF APS is disabled");
  }

  xConfirmPara(encCfg->m_sourceWidth % SPS::getWinUnitX(encCfg->m_chromaFormatIdc) != 0,
               "Picture width must be an integer multiple of the specified chroma subsampling");
  xConfirmPara(encCfg->m_sourceHeight % SPS::getWinUnitY(encCfg->m_chromaFormatIdc) != 0,
               "Picture height must be an integer multiple of the specified chroma subsampling");

  xConfirmPara(encCfg->m_sourcePadding[0] % SPS::getWinUnitX(encCfg->m_chromaFormatIdc) != 0,
               "Horizontal padding must be an integer multiple of the specified chroma subsampling");
  xConfirmPara(encCfg->m_sourcePadding[1] % SPS::getWinUnitY(encCfg->m_chromaFormatIdc) != 0,
               "Vertical padding must be an integer multiple of the specified chroma subsampling");

  xConfirmPara(encCfg->m_confWinLeft % SPS::getWinUnitX(encCfg->m_chromaFormatIdc) != 0,
               "Left conformance window offset must be an integer multiple of the specified chroma subsampling");
  xConfirmPara(encCfg->m_confWinRight % SPS::getWinUnitX(encCfg->m_chromaFormatIdc) != 0,
               "Right conformance window offset must be an integer multiple of the specified chroma subsampling");
  xConfirmPara(encCfg->m_confWinTop % SPS::getWinUnitY(encCfg->m_chromaFormatIdc) != 0,
               "Top conformance window offset must be an integer multiple of the specified chroma subsampling");
  xConfirmPara(encCfg->m_confWinBottom % SPS::getWinUnitY(encCfg->m_chromaFormatIdc) != 0,
               "Bottom conformance window offset must be an integer multiple of the specified chroma subsampling");

  /* if this is an intra-only sequence, ie IntraPeriod=1, don't verify the GOP structure
   * This permits the ability to omit a GOP structure specification */
  if (encCfg->m_intraPeriod == 1 && encCfg->m_GOPList[0].m_POC == -1)
  {
    encCfg->m_GOPList[0]                    = GOPEntry();
    encCfg->m_GOPList[0].m_QPFactor         = 1;
    encCfg->m_GOPList[0].m_betaOffsetDiv2   = 0;
    encCfg->m_GOPList[0].m_tcOffsetDiv2     = 0;
    encCfg->m_GOPList[0].m_CbBetaOffsetDiv2 = 0;
    encCfg->m_GOPList[0].m_CbTcOffsetDiv2   = 0;
    encCfg->m_GOPList[0].m_CrBetaOffsetDiv2 = 0;
    encCfg->m_GOPList[0].m_CrTcOffsetDiv2   = 0;
    encCfg->m_GOPList[0].m_POC              = 1;
    encCfg->m_RPLList0[0]                   = RPLEntry();
    encCfg->m_RPLList1[0]                   = RPLEntry();
    encCfg->m_RPLList0[0].m_POC = encCfg->m_RPLList1[0].m_POC = 1;
    encCfg->m_RPLList0[0].m_numRefPicsActive                  = 4;
    encCfg->m_GOPList[0].m_numRefPicsActive0                  = 4;
  }
  else
  {
    xConfirmPara(encCfg->m_intraOnlyConstraintFlag, "IntraOnlyConstraintFlag cannot be 1 for inter sequences");
  }

  int                                      multipleFactor = encCfg->m_compositeRefEnabled ? 2 : 1;
  bool                                     verifiedGOP    = false;
  bool                                     errorGOP       = false;
  int                                      checkGOP       = 1;
  static_vector<int, MAX_NUM_REF_PICS + 1> refList;
  refList.push_back(0);
  if (encCfg->m_fieldSeqFlag)
  {
    refList.push_back(1);
  }
  bool isOK[MAX_GOP];
  for (int i = 0; i < MAX_GOP; i++)
  {
    isOK[i] = false;
  }
  int numOK = 0;
  xConfirmPara(encCfg->m_intraPeriod >= 0 && (encCfg->m_intraPeriod % encCfg->m_gopSize != 0),
               "Intra period must be a multiple of GOPSize, or -1");

  for (int i = 0; i < encCfg->m_gopSize; i++)
  {
    if (encCfg->m_GOPList[i].m_POC == encCfg->m_gopSize * multipleFactor)
    {
      xConfirmPara(encCfg->m_GOPList[i].m_temporalId != 0, "The last frame in each GOP must have temporal ID = 0 ");
    }
  }

  if ((encCfg->m_intraPeriod != 1) && !encCfg->m_deblockingFilterOffsetInPPS && (!encCfg->m_deblockingFilterDisable))
  {
    for (int i = 0; i < encCfg->m_gopSize; i++)
    {
      xConfirmPara((encCfg->m_GOPList[i].m_betaOffsetDiv2 + encCfg->m_deblockingFilterBetaOffsetDiv2) < -12 ||
                     (encCfg->m_GOPList[i].m_betaOffsetDiv2 + encCfg->m_deblockingFilterBetaOffsetDiv2) > 12,
                   "Loop Filter Beta Offset div. 2 for one of the GOP entries exceeds supported range (-12 to 12)");
      xConfirmPara((encCfg->m_GOPList[i].m_tcOffsetDiv2 + encCfg->m_deblockingFilterTcOffsetDiv2) < -12 ||
                     (encCfg->m_GOPList[i].m_tcOffsetDiv2 + encCfg->m_deblockingFilterTcOffsetDiv2) > 12,
                   "Loop Filter Tc Offset div. 2 for one of the GOP entries exceeds supported range (-12 to 12)");
      xConfirmPara((encCfg->m_GOPList[i].m_CbBetaOffsetDiv2 + encCfg->m_deblockingFilterCbBetaOffsetDiv2) < -12 ||
                     (encCfg->m_GOPList[i].m_CbBetaOffsetDiv2 + encCfg->m_deblockingFilterCbBetaOffsetDiv2) > 12,
                   "Loop Filter Beta Offset div. 2 for one of the GOP entries exceeds supported range (-12 to 12)");
      xConfirmPara((encCfg->m_GOPList[i].m_CbTcOffsetDiv2 + encCfg->m_deblockingFilterCbTcOffsetDiv2) < -12 ||
                     (encCfg->m_GOPList[i].m_CbTcOffsetDiv2 + encCfg->m_deblockingFilterCbTcOffsetDiv2) > 12,
                   "Loop Filter Tc Offset div. 2 for one of the GOP entries exceeds supported range (-12 to 12)");
      xConfirmPara((encCfg->m_GOPList[i].m_CrBetaOffsetDiv2 + encCfg->m_deblockingFilterCrBetaOffsetDiv2) < -12 ||
                     (encCfg->m_GOPList[i].m_CrBetaOffsetDiv2 + encCfg->m_deblockingFilterCrBetaOffsetDiv2) > 12,
                   "Loop Filter Beta Offset div. 2 for one of the GOP entries exceeds supported range (-12 to 12)");
      xConfirmPara((encCfg->m_GOPList[i].m_CrTcOffsetDiv2 + encCfg->m_deblockingFilterCrTcOffsetDiv2) < -12 ||
                     (encCfg->m_GOPList[i].m_CrTcOffsetDiv2 + encCfg->m_deblockingFilterCrTcOffsetDiv2) > 12,
                   "Loop Filter Tc Offset div. 2 for one of the GOP entries exceeds supported range (-12 to 12)");
    }
  }

#if W0038_CQP_ADJ
  for (int i = 0; i < encCfg->m_gopSize; i++)
  {
    xConfirmPara(abs(encCfg->m_GOPList[i].m_CbQPoffset) > 12,
                 "Cb QP Offset for one of the GOP entries exceeds supported range (-12 to 12)");
    xConfirmPara(abs(encCfg->m_GOPList[i].m_CbQPoffset + encCfg->m_chromaCbQpOffset) > 12,
                 "Cb QP Offset for one of the GOP entries, when combined with the PPS Cb offset, exceeds supported "
                 "range (-12 to 12)");
    xConfirmPara(abs(encCfg->m_GOPList[i].m_CrQPoffset) > 12,
                 "Cr QP Offset for one of the GOP entries exceeds supported range (-12 to 12)");
    xConfirmPara(abs(encCfg->m_GOPList[i].m_CrQPoffset + encCfg->m_chromaCrQpOffset) > 12,
                 "Cr QP Offset for one of the GOP entries, when combined with the PPS Cr offset, exceeds supported "
                 "range (-12 to 12)");
  }
  xConfirmPara(abs(encCfg->m_sliceChromaQpOffsetIntraOrPeriodic[0]) > 12,
               "Intra/periodic Cb QP Offset exceeds supported range (-12 to 12)");
  xConfirmPara(
    abs(encCfg->m_sliceChromaQpOffsetIntraOrPeriodic[0] + encCfg->m_chromaCbQpOffset) > 12,
    "Intra/periodic Cb QP Offset, when combined with the PPS Cb offset, exceeds supported range (-12 to 12)");
  xConfirmPara(abs(encCfg->m_sliceChromaQpOffsetIntraOrPeriodic[1]) > 12,
               "Intra/periodic Cr QP Offset exceeds supported range (-12 to 12)");
  xConfirmPara(
    abs(encCfg->m_sliceChromaQpOffsetIntraOrPeriodic[1] + encCfg->m_chromaCrQpOffset) > 12,
    "Intra/periodic Cr QP Offset, when combined with the PPS Cr offset, exceeds supported range (-12 to 12)");
#endif

  xConfirmPara(encCfg->m_maxSublayers < 1 || encCfg->m_maxSublayers > 7, "MaxSublayers must be in range [1..7]");

  xConfirmPara(encCfg->m_fastAdaptCostPredMode < 0 || encCfg->m_fastAdaptCostPredMode > 2,
               "FastAdaptCostPredMode must be in range [0..2]");

  int  extraRPLs    = 0;
  bool hasFutureRef = false;
  // start looping through frames in coding order until we can verify that the GOP structure is correct.
  while (!verifiedGOP && !errorGOP)
  {
    int       rplIdx = (checkGOP - 1) % encCfg->m_gopSize;
    const int curPOC =
      ((checkGOP - 1) / encCfg->m_gopSize) * encCfg->m_gopSize * multipleFactor + encCfg->m_RPLList0[rplIdx].m_POC;
    if (encCfg->m_RPLList0[rplIdx].m_POC < 0 || encCfg->m_RPLList1[rplIdx].m_POC < 0)
    {
      msg(WARNING, "\nError: found fewer Reference Picture Sets than GOPSize\n");
      errorGOP = true;
    }
    else
    {
      // check that all reference pictures are available, or have a POC < 0 meaning they might be available in the next
      // GOP.
      bool beforeI = false;
      for (int i = 0; i < encCfg->m_RPLList0[rplIdx].m_numRefPics; i++)
      {
        const int refPoc = curPOC - encCfg->m_RPLList0[rplIdx].m_deltaRefPics[i];
        if (refPoc < 0)
        {
          beforeI = true;
        }
        else
        {
          bool found = false;
          for (const int poc: refList)
          {
            if (poc == refPoc)
            {
              found = true;
              for (int k = 0; k < encCfg->m_gopSize; k++)
              {
                if (refPoc % (encCfg->m_gopSize * multipleFactor) ==
                    encCfg->m_RPLList0[k].m_POC % (encCfg->m_gopSize * multipleFactor))
                {
                  // TODO: check whether equal test is correct
                  if (encCfg->m_RPLList0[k].m_temporalId == encCfg->m_RPLList0[rplIdx].m_temporalId)
                  {
                    encCfg->m_RPLList0[k].m_refPic = true;
                  }
                }
              }
            }
          }
          if (!found)
          {
            msg(WARNING, "\nError: ref pic %d is not available for GOP frame %d\n",
                encCfg->m_RPLList0[rplIdx].m_deltaRefPics[i], rplIdx + 1);
            errorGOP = true;
          }
        }
      }
      if (!beforeI && !errorGOP)
      {
        // all ref frames were present
        if (!isOK[rplIdx])
        {
          numOK++;
          isOK[rplIdx] = true;
          if (numOK == encCfg->m_gopSize)
          {
            verifiedGOP = true;
          }
        }
      }
      else
      {
        const int newRplIdx = encCfg->m_gopSize + extraRPLs;
        CHECK(newRplIdx >= MAX_GOP, "Too many RPLs");

        // create a new RPLEntry for this frame containing all the reference pictures that were available (POC > 0)
        encCfg->m_RPLList0[newRplIdx] = encCfg->m_RPLList0[rplIdx];
        encCfg->m_RPLList1[newRplIdx] = encCfg->m_RPLList1[rplIdx];

        int newRefs0       = 0;
        int newActiveRefs0 = 0;
        for (int i = 0; i < encCfg->m_RPLList0[rplIdx].m_numRefPics; i++)
        {
          const int refPoc = curPOC - encCfg->m_RPLList0[rplIdx].m_deltaRefPics[i];
          if (refPoc >= 0)
          {
            encCfg->m_RPLList0[newRplIdx].m_deltaRefPics[newRefs0] = encCfg->m_RPLList0[rplIdx].m_deltaRefPics[i];
            newRefs0++;
            newActiveRefs0 += i < encCfg->m_RPLList0[rplIdx].m_numRefPicsActive ? 1 : 0;
          }
        }
        int numPrefActiveRefs0 = encCfg->m_RPLList0[rplIdx].m_numRefPicsActive;

        int newRefs1       = 0;
        int newActiveRefs1 = 0;
        for (int i = 0; i < encCfg->m_RPLList1[rplIdx].m_numRefPics; i++)
        {
          const int refPoc = curPOC - encCfg->m_RPLList1[rplIdx].m_deltaRefPics[i];
          if (refPoc >= 0)
          {
            encCfg->m_RPLList1[encCfg->m_gopSize + extraRPLs].m_deltaRefPics[newRefs1] =
              encCfg->m_RPLList1[rplIdx].m_deltaRefPics[i];
            newRefs1++;
            newActiveRefs1 += i < encCfg->m_RPLList1[rplIdx].m_numRefPicsActive ? 1 : 0;
          }
        }
        int numPrefActiveRefs1 = encCfg->m_RPLList1[rplIdx].m_numRefPicsActive;

        for (int offset = -1; offset > -checkGOP; offset--)
        {
          // step backwards in coding order and include any extra available pictures we might find useful to replace the
          // ones with POC < 0.
          int offGOP = (checkGOP - 1 + offset) % encCfg->m_gopSize;
          int offPOC = ((checkGOP - 1 + offset) / encCfg->m_gopSize) * (encCfg->m_gopSize * multipleFactor) +
            encCfg->m_RPLList0[offGOP].m_POC;
          if (offPOC >= 0 && encCfg->m_RPLList0[offGOP].m_temporalId <= encCfg->m_RPLList0[rplIdx].m_temporalId)
          {
            bool      newRef      = std::find(refList.begin(), refList.end(), offPOC) != refList.end();
            const int newDeltaPoc = curPOC - offPOC;
            for (int i = 0; i < newRefs0; i++)
            {
              if (encCfg->m_RPLList0[newRplIdx].m_deltaRefPics[i] == newDeltaPoc)
              {
                newRef = false;
              }
            }
            if (newRef)
            {
              int insertPoint = newActiveRefs0;
              // this picture can be added, find appropriate place in list and insert it.
              if (encCfg->m_RPLList0[offGOP].m_temporalId == encCfg->m_RPLList0[rplIdx].m_temporalId)
              {
                encCfg->m_RPLList0[offGOP].m_refPic = true;
              }
              for (int j = 0; j < newActiveRefs0; j++)
              {
                if (encCfg->m_RPLList0[newRplIdx].m_deltaRefPics[j] > newDeltaPoc && newDeltaPoc > 0)
                {
                  insertPoint = j;
                  break;
                }
              }
              int prev = newDeltaPoc;
              newRefs0++;
              newActiveRefs0++;
              for (int j = insertPoint; j < newRefs0; j++)
              {
                int newPrev = encCfg->m_RPLList0[encCfg->m_gopSize + extraRPLs].m_deltaRefPics[j];
                encCfg->m_RPLList0[encCfg->m_gopSize + extraRPLs].m_deltaRefPics[j] = prev;
                prev                                                                = newPrev;
              }
            }
          }
          if (newActiveRefs0 >= numPrefActiveRefs0)
          {
            break;
          }
        }

        for (int offset = -1; offset > -checkGOP; offset--)
        {
          // step backwards in coding order and include any extra available pictures we might find useful to replace the
          // ones with POC < 0.
          int offGOP = (checkGOP - 1 + offset) % encCfg->m_gopSize;
          int offPOC = ((checkGOP - 1 + offset) / encCfg->m_gopSize) * (encCfg->m_gopSize * multipleFactor) +
            encCfg->m_RPLList1[offGOP].m_POC;
          if (offPOC >= 0 && encCfg->m_RPLList1[offGOP].m_temporalId <= encCfg->m_RPLList1[rplIdx].m_temporalId)
          {
            bool      newRef      = std::find(refList.begin(), refList.end(), offPOC) != refList.end();
            const int newDeltaPoc = curPOC - offPOC;
            for (int i = 0; i < newRefs1; i++)
            {
              if (encCfg->m_RPLList1[newRplIdx].m_deltaRefPics[i] == newDeltaPoc)
              {
                newRef = false;
              }
            }
            if (newRef)
            {
              int insertPoint = newActiveRefs1;
              // this picture can be added, find appropriate place in list and insert it.
              if (encCfg->m_RPLList1[offGOP].m_temporalId == encCfg->m_RPLList1[rplIdx].m_temporalId)
              {
                encCfg->m_RPLList1[offGOP].m_refPic = true;
              }
              for (int j = 0; j < newActiveRefs1; j++)
              {
                if (encCfg->m_RPLList1[newRplIdx].m_deltaRefPics[j] > newDeltaPoc && newDeltaPoc > 0)
                {
                  insertPoint = j;
                  break;
                }
              }
              int prev = newDeltaPoc;
              newRefs1++;
              newActiveRefs1++;
              for (int j = insertPoint; j < newRefs1; j++)
              {
                std::swap(prev, encCfg->m_RPLList1[newRplIdx].m_deltaRefPics[j]);
              }
            }
          }
          if (newActiveRefs1 >= numPrefActiveRefs1)
          {
            break;
          }
        }

        encCfg->m_RPLList0[newRplIdx].m_numRefPics       = newRefs0;
        encCfg->m_RPLList0[newRplIdx].m_numRefPicsActive = newActiveRefs0;
        encCfg->m_RPLList1[newRplIdx].m_numRefPics       = newRefs1;
        encCfg->m_RPLList1[newRplIdx].m_numRefPicsActive = newActiveRefs1;

        rplIdx = newRplIdx;
        extraRPLs++;
      }
      refList.clear();
      for (int i = 0; i < encCfg->m_RPLList0[rplIdx].m_numRefPics; i++)
      {
        const int refPoc = curPOC - encCfg->m_RPLList0[rplIdx].m_deltaRefPics[i];
        hasFutureRef |= (encCfg->m_RPLList0[rplIdx].m_deltaRefPics[i] < 0);
        if (refPoc >= 0)
        {
          refList.push_back(refPoc);
        }
      }
      for (int i = 0; i < encCfg->m_RPLList1[rplIdx].m_numRefPics; i++)
      {
        const int refPoc = curPOC - encCfg->m_RPLList1[rplIdx].m_deltaRefPics[i];
        hasFutureRef |= (encCfg->m_RPLList1[rplIdx].m_deltaRefPics[i] < 0);
        if (refPoc >= 0)
        {
          if (std::find(refList.begin(), refList.end(), refPoc) == refList.end())
          {
            refList.push_back(refPoc);
          }
        }
      }
      refList.push_back(curPOC);
    }
    checkGOP++;
  }
  encCfg->m_isLowDelay = !hasFutureRef && encCfg->m_intraPeriod != 1;
  xConfirmPara(errorGOP, "Invalid GOP structure given");

  encCfg->m_maxTempLayer = 1;

  for (int i = 0; i < encCfg->m_gopSize; i++)
  {
    if (encCfg->m_GOPList[i].m_temporalId >= encCfg->m_maxTempLayer)
    {
      encCfg->m_maxTempLayer = encCfg->m_GOPList[i].m_temporalId + 1;
    }
    xConfirmPara(encCfg->m_GOPList[i].m_sliceType != 'B' && encCfg->m_GOPList[i].m_sliceType != 'P' &&
                   encCfg->m_GOPList[i].m_sliceType != 'I',
                 "Slice type must be equal to B or P or I");
  }
  for (int i = 0; i < MAX_TLAYER; i++)
  {
    encCfg->m_maxNumReorderPics[i]  = 0;
    encCfg->m_maxDecPicBuffering[i] = 1;
  }
  for (int i = 0; i < encCfg->m_gopSize; i++)
  {
    int numRefPic = encCfg->m_RPLList0[i].m_numRefPics;
    for (int tmp = 0; tmp < encCfg->m_RPLList1[i].m_numRefPics; tmp++)
    {
      bool notSame = true;
      for (int jj = 0; notSame && jj < encCfg->m_RPLList0[i].m_numRefPics; jj++)
      {
        if (encCfg->m_RPLList1[i].m_deltaRefPics[tmp] == encCfg->m_RPLList0[i].m_deltaRefPics[jj])
        {
          notSame = false;
        }
      }
      if (notSame)
      {
        numRefPic++;
      }
    }
    if (numRefPic + 1 > encCfg->m_maxDecPicBuffering[encCfg->m_GOPList[i].m_temporalId])
    {
      encCfg->m_maxDecPicBuffering[encCfg->m_GOPList[i].m_temporalId] = numRefPic + 1;
    }
    int highestDecodingNumberWithLowerPOC = 0;
    for (int j = 0; j < encCfg->m_gopSize; j++)
    {
      if (encCfg->m_GOPList[j].m_POC <= encCfg->m_GOPList[i].m_POC)
      {
        highestDecodingNumberWithLowerPOC = j;
      }
    }
    int numReorder = 0;
    for (int j = 0; j < highestDecodingNumberWithLowerPOC; j++)
    {
      if (encCfg->m_GOPList[j].m_temporalId <= encCfg->m_GOPList[i].m_temporalId &&
          encCfg->m_GOPList[j].m_POC > encCfg->m_GOPList[i].m_POC)
      {
        numReorder++;
      }
    }
    if (numReorder > encCfg->m_maxNumReorderPics[encCfg->m_GOPList[i].m_temporalId])
    {
      encCfg->m_maxNumReorderPics[encCfg->m_GOPList[i].m_temporalId] = numReorder;
    }
  }

  for (int i = 0; i < MAX_TLAYER - 1; i++)
  {
    // a lower layer can not have higher value of encCfg->m_maxNumReorderPics than a higher layer
    if (encCfg->m_maxNumReorderPics[i + 1] < encCfg->m_maxNumReorderPics[i])
    {
      encCfg->m_maxNumReorderPics[i + 1] = encCfg->m_maxNumReorderPics[i];
    }
    // the value of dpb_max_num_reorder_pics[ i ] shall be in the range of 0 to max_dec_pic_buffering[ i ] - 1,
    // inclusive
    if (encCfg->m_maxNumReorderPics[i] > encCfg->m_maxDecPicBuffering[i] - 1)
    {
      encCfg->m_maxDecPicBuffering[i] = encCfg->m_maxNumReorderPics[i] + 1;
    }
    // a lower layer can not have higher value of encCfg->m_maxDecPicBuffering than a higher layer
    if (encCfg->m_maxDecPicBuffering[i + 1] < encCfg->m_maxDecPicBuffering[i])
    {
      encCfg->m_maxDecPicBuffering[i + 1] = encCfg->m_maxDecPicBuffering[i];
    }
  }

  // the value of dpb_max_num_reorder_pics[ i ] shall be in the range of 0 to max_dec_pic_buffering[ i ] -  1, inclusive
  if (encCfg->m_maxNumReorderPics[MAX_TLAYER - 1] > encCfg->m_maxDecPicBuffering[MAX_TLAYER - 1] - 1)
  {
    encCfg->m_maxDecPicBuffering[MAX_TLAYER - 1] = encCfg->m_maxNumReorderPics[MAX_TLAYER - 1] + 1;
  }

  xConfirmPara(!encCfg->m_BIO && encCfg->m_DMVDBIOExt, "DMVDBIOExt cannot be enabled with BIO disabled");
  xConfirmPara(!encCfg->m_useDMVD && encCfg->m_DMVDBIOExt, "DMVDBIOExt cannot be enabled with DMVD disabled");

  if (encCfg->m_picPartitionFlag)
  {
    PPS      pps;
    uint32_t colIdx, rowIdx;
    uint32_t remSize;

    pps.m_picWidthInLumaSamples  = encCfg->m_sourceWidth;
    pps.m_picHeightInLumaSamples = encCfg->m_sourceHeight;
    pps.setLog2CtuSize(floorLog2(encCfg->m_CTUSize));

    // set default tile column if not provided
    if (encCfg->m_tileColumnWidth.size() == 0)
    {
      encCfg->m_tileColumnWidth.push_back(pps.m_picWidthInCtu);
    }
    // set default tile row if not provided
    if (encCfg->m_tileRowHeight.size() == 0)
    {
      encCfg->m_tileRowHeight.push_back(pps.m_picHeightInCtu);
    }

    // remove any tile columns that can be specified implicitly
    while (encCfg->m_tileColumnWidth.size() > 1 &&
           encCfg->m_tileColumnWidth.end()[-1] == encCfg->m_tileColumnWidth.end()[-2])
    {
      encCfg->m_tileColumnWidth.pop_back();
    }

    // remove any tile rows that can be specified implicitly
    while (encCfg->m_tileRowHeight.size() > 1 && encCfg->m_tileRowHeight.end()[-1] == encCfg->m_tileRowHeight.end()[-2])
    {
      encCfg->m_tileRowHeight.pop_back();
    }

    // setup tiles in temporary PPS structure
    remSize = pps.m_picWidthInCtu;
    for (colIdx = 0; remSize > 0 && colIdx < encCfg->m_tileColumnWidth.size(); colIdx++)
    {
      xConfirmPara(encCfg->m_tileColumnWidth[colIdx] == 0, "Tile column widths cannot be equal to 0");
      encCfg->m_tileColumnWidth[colIdx] = std::min(remSize, encCfg->m_tileColumnWidth[colIdx]);
      pps.addTileColumnWidth(encCfg->m_tileColumnWidth[colIdx]);
      remSize -= encCfg->m_tileColumnWidth[colIdx];
    }
    encCfg->m_tileColumnWidth.resize(colIdx);
    pps.m_numExpTileCols = ((uint32_t)encCfg->m_tileColumnWidth.size());
    remSize              = pps.m_picHeightInCtu;
    for (rowIdx = 0; remSize > 0 && rowIdx < encCfg->m_tileRowHeight.size(); rowIdx++)
    {
      xConfirmPara(encCfg->m_tileRowHeight[rowIdx] == 0, "Tile row heights cannot be equal to 0");
      encCfg->m_tileRowHeight[rowIdx] = std::min(remSize, encCfg->m_tileRowHeight[rowIdx]);
      pps.addTileRowHeight(encCfg->m_tileRowHeight[rowIdx]);
      remSize -= encCfg->m_tileRowHeight[rowIdx];
    }
    encCfg->m_tileRowHeight.resize(rowIdx);
    pps.m_numExpTileRows = ((uint32_t)encCfg->m_tileRowHeight.size());
    pps.initTiles();
    xConfirmPara(pps.m_numTileCols > getMaxTileColsByLevel(encCfg->m_level),
                 "Number of tile columns exceeds maximum number allowed according to specified level");
    xConfirmPara(pps.m_numTileRows > getMaxTileRowsByLevel(encCfg->m_level),
                 "Number of tile rows exceeds maximum number allowed according to specified level");
    encCfg->m_numTileCols = pps.m_numTileCols;
    encCfg->m_numTileRows = pps.m_numTileRows;

    // rectangular slices
    if (!encCfg->m_rasterSliceFlag)
    {
      if (!encCfg->m_singleSlicePerSubPicFlag)
      {
        uint32_t sliceIdx;
        bool     needTileIdxDelta = false;

        // generate slice list for the simplified fixed-rectangular-slice-size config option
        if (encCfg->m_rectSliceFixedWidth > 0 && encCfg->m_rectSliceFixedHeight > 0)
        {
          int tileIdx = 0;
          encCfg->m_rectSlicePos.clear();
          while (tileIdx < pps.getNumTiles())
          {
            uint32_t startTileX = tileIdx % pps.m_numTileCols;
            uint32_t startTileY = tileIdx / pps.m_numTileCols;
            uint32_t startCtuX  = pps.getTileColumnBd(startTileX);
            uint32_t startCtuY  = pps.getTileRowBd(startTileY);
            uint32_t stopCtuX   = (startTileX + encCfg->m_rectSliceFixedWidth) >= pps.m_numTileCols
                ? pps.m_picWidthInCtu - 1
                : pps.getTileColumnBd(startTileX + encCfg->m_rectSliceFixedWidth) - 1;
            uint32_t stopCtuY   = (startTileY + encCfg->m_rectSliceFixedHeight) >= pps.m_numTileRows
                ? pps.m_picHeightInCtu - 1
                : pps.getTileRowBd(startTileY + encCfg->m_rectSliceFixedHeight) - 1;
            uint32_t stopTileX  = pps.ctuToTileCol(stopCtuX);
            uint32_t stopTileY  = pps.ctuToTileRow(stopCtuY);

            // add rectangular slice to list
            encCfg->m_rectSlicePos.push_back(startCtuY * pps.m_picWidthInCtu + startCtuX);
            encCfg->m_rectSlicePos.push_back(stopCtuY * pps.m_picWidthInCtu + stopCtuX);

            // get slice size in tiles
            uint32_t sliceWidth  = stopTileX - startTileX + 1;
            uint32_t sliceHeight = stopTileY - startTileY + 1;

            // move to next tile in raster scan order
            tileIdx += sliceWidth;
            if (tileIdx % pps.m_numTileCols == 0)
            {
              tileIdx += (sliceHeight - 1) * pps.m_numTileCols;
            }
          }
        }

        xConfirmPara(encCfg->m_rectSlicePos.size() & 1,
                     "Odd number of rectangular slice positions provided. Rectangular slice positions must be "
                     "specified in pairs of (top-left / bottom-right) raster-scan CTU addresses.");

        // set default slice size if not provided
        if (encCfg->m_rectSlicePos.size() == 0)
        {
          encCfg->m_rectSlicePos.push_back(0);
          encCfg->m_rectSlicePos.push_back(pps.m_picWidthInCtu * pps.m_picHeightInCtu - 1);
        }

        pps.m_numSlicesInPic = (uint32_t)(encCfg->m_rectSlicePos.size() >> 1);
        xConfirmPara(pps.m_numSlicesInPic > MAX_SLICES, "Number of slices in picture exceeds valid range");
        xConfirmPara(pps.m_numSlicesInPic > getMaxSlicesByLevel(encCfg->m_level),
                     "Number of rectangular slices exceeds maximum number allowed according to specified level");
        pps.initRectSlices();

        // set slice parameters from CTU addresses
        for (sliceIdx = 0; sliceIdx < pps.m_numSlicesInPic; sliceIdx++)
        {
          xConfirmPara(encCfg->m_rectSlicePos[2 * sliceIdx] >= pps.m_picWidthInCtu * pps.m_picHeightInCtu,
                       "Rectangular slice position exceeds total number of CTU in picture.");
          xConfirmPara(encCfg->m_rectSlicePos[2 * sliceIdx + 1] >= pps.m_picWidthInCtu * pps.m_picHeightInCtu,
                       "Rectangular slice position exceeds total number of CTU in picture.");

          // map raster scan CTU address to X/Y position
          uint32_t startCtuX = encCfg->m_rectSlicePos[2 * sliceIdx] % pps.m_picWidthInCtu;
          uint32_t startCtuY = encCfg->m_rectSlicePos[2 * sliceIdx] / pps.m_picWidthInCtu;
          uint32_t stopCtuX  = encCfg->m_rectSlicePos[2 * sliceIdx + 1] % pps.m_picWidthInCtu;
          uint32_t stopCtuY  = encCfg->m_rectSlicePos[2 * sliceIdx + 1] / pps.m_picWidthInCtu;

          // get corresponding tile index
          uint32_t startTileX = pps.ctuToTileCol(startCtuX);
          uint32_t startTileY = pps.ctuToTileRow(startCtuY);
          uint32_t stopTileX  = pps.ctuToTileCol(stopCtuX);
          uint32_t stopTileY  = pps.ctuToTileRow(stopCtuY);
          uint32_t tileIdx    = startTileY * pps.m_numTileCols + startTileX;

          // get slice size in tiles
          uint32_t sliceWidth  = stopTileX - startTileX + 1;
          uint32_t sliceHeight = stopTileY - startTileY + 1;

          // check for slice / tile alignment
          xConfirmPara(startCtuX != pps.getTileColumnBd(startTileX),
                       "Rectangular slice position does not align with a left tile edge.");
          xConfirmPara(stopCtuX != (pps.getTileColumnBd(stopTileX + 1) - 1),
                       "Rectangular slice position does not align with a right tile edge.");
          if (sliceWidth > 1 || sliceHeight > 1)
          {
            xConfirmPara(startCtuY != pps.getTileRowBd(startTileY),
                         "Rectangular slice position does not align with a top tile edge.");
            xConfirmPara(stopCtuY != (pps.getTileRowBd(stopTileY + 1) - 1),
                         "Rectangular slice position does not align with a bottom tile edge.");
          }

          // set slice size and tile index
          pps.m_rectSlices[sliceIdx].m_sliceWidthInTiles  = sliceWidth;
          pps.m_rectSlices[sliceIdx].m_sliceHeightInTiles = sliceHeight;
          pps.m_rectSlices[sliceIdx].m_tileIdx            = tileIdx;
          if (sliceIdx > 0 && !needTileIdxDelta)
          {
            uint32_t lastTileIdx = pps.m_rectSlices[sliceIdx - 1].m_tileIdx;
            lastTileIdx += pps.m_rectSlices[sliceIdx - 1].m_sliceWidthInTiles;
            if (lastTileIdx % pps.m_numTileCols == 0)
            {
              lastTileIdx += (pps.m_rectSlices[sliceIdx - 1].m_sliceHeightInTiles - 1) * pps.m_numTileCols;
            }
            if (lastTileIdx != tileIdx)
            {
              needTileIdxDelta = true;
            }
          }

          // special case for multiple slices within a single tile
          if (sliceWidth == 1 && sliceHeight == 1)
          {
            uint32_t firstSliceIdx                        = sliceIdx;
            uint32_t numSlicesInTile                      = 1;
            pps.m_rectSlices[sliceIdx].m_sliceHeightInCtu = stopCtuY - startCtuY + 1;

            while (sliceIdx < pps.m_numSlicesInPic - 1)
            {
              uint32_t nextTileIdx;
              startCtuX   = encCfg->m_rectSlicePos[2 * (sliceIdx + 1)] % pps.m_picWidthInCtu;
              startCtuY   = encCfg->m_rectSlicePos[2 * (sliceIdx + 1)] / pps.m_picWidthInCtu;
              stopCtuX    = encCfg->m_rectSlicePos[2 * (sliceIdx + 1) + 1] % pps.m_picWidthInCtu;
              stopCtuY    = encCfg->m_rectSlicePos[2 * (sliceIdx + 1) + 1] / pps.m_picWidthInCtu;
              startTileX  = pps.ctuToTileCol(startCtuX);
              startTileY  = pps.ctuToTileRow(startCtuY);
              stopTileX   = pps.ctuToTileCol(stopCtuX);
              stopTileY   = pps.ctuToTileRow(stopCtuY);
              nextTileIdx = startTileY * pps.m_numTileCols + startTileX;
              sliceWidth  = stopTileX - startTileX + 1;
              sliceHeight = stopTileY - startTileY + 1;
              if (nextTileIdx != tileIdx || sliceWidth != 1 || sliceHeight != 1)
              {
                break;
              }
              numSlicesInTile++;
              sliceIdx++;
              pps.m_rectSlices[sliceIdx].m_sliceWidthInTiles  = 1;
              pps.m_rectSlices[sliceIdx].m_sliceHeightInTiles = 1;
              pps.m_rectSlices[sliceIdx].m_tileIdx            = tileIdx;
              pps.m_rectSlices[sliceIdx].m_sliceHeightInCtu   = stopCtuY - startCtuY + 1;
            }
            pps.m_rectSlices[firstSliceIdx].m_numSlicesInTile = numSlicesInTile;
          }
        }
        pps.m_tileIdxDeltaPresentFlag     = needTileIdxDelta;
        encCfg->m_tileIdxDeltaPresentFlag = needTileIdxDelta;

        // check rectangular slice mapping and full picture CTU coverage
        pps.initRectSliceMap(nullptr);

        // store rectangular slice parameters from temporary PPS structure
        encCfg->m_numSlicesInPic = pps.m_numSlicesInPic;
        encCfg->m_rectSlices.resize(pps.m_numSlicesInPic);
        for (sliceIdx = 0; sliceIdx < pps.m_numSlicesInPic; sliceIdx++)
        {
          encCfg->m_rectSlices[sliceIdx].m_sliceWidthInTiles  = pps.m_rectSlices[sliceIdx].m_sliceWidthInTiles;
          encCfg->m_rectSlices[sliceIdx].m_sliceHeightInTiles = pps.m_rectSlices[sliceIdx].m_sliceHeightInTiles;
          encCfg->m_rectSlices[sliceIdx].m_numSlicesInTile    = pps.m_rectSlices[sliceIdx].m_numSlicesInTile;
          encCfg->m_rectSlices[sliceIdx].m_sliceHeightInCtu   = pps.m_rectSlices[sliceIdx].m_sliceHeightInCtu;
          encCfg->m_rectSlices[sliceIdx].m_tileIdx            = pps.m_rectSlices[sliceIdx].m_tileIdx;
        }
      }
    }
    // raster-scan slices
    else
    {
      uint32_t listIdx  = 0;
      uint32_t remTiles = pps.getNumTiles();

      // set default slice size if not provided
      if (encCfg->m_rasterSliceSize.size() == 0)
      {
        encCfg->m_rasterSliceSize.push_back(remTiles);
      }

      // set raster slice sizes
      while (remTiles > 0)
      {
        // truncate if size exceeds number of remaining tiles
        if (listIdx < encCfg->m_rasterSliceSize.size())
        {
          encCfg->m_rasterSliceSize[listIdx] = std::min(remTiles, encCfg->m_rasterSliceSize[listIdx]);
          remTiles -= encCfg->m_rasterSliceSize[listIdx];
        }
        // replicate last size uniformly as needed to cover the remainder of the picture
        else
        {
          encCfg->m_rasterSliceSize.push_back(std::min(remTiles, encCfg->m_rasterSliceSize.back()));
          remTiles -= encCfg->m_rasterSliceSize.back();
        }
        listIdx++;
      }
      // shrink list if too many sizes were provided
      encCfg->m_rasterSliceSize.resize(listIdx);

      encCfg->m_numSlicesInPic = (uint32_t)encCfg->m_rasterSliceSize.size();
      xConfirmPara(encCfg->m_rasterSliceSize.size() > getMaxSlicesByLevel(encCfg->m_level),
                   "Number of raster-scan slices exceeds maximum number allowed according to specified level");
    }
  }
  else
  {
    encCfg->m_numTileCols    = 1;
    encCfg->m_numTileRows    = 1;
    encCfg->m_numSlicesInPic = 1;
  }

  if ((encCfg->m_seiCfg.m_MCTSEncConstraint) && (!encCfg->m_disableLFCrossTileBoundaryFlag))
  {
    printf("Warning: Constrained Encoding for Motion Constrained Tile Sets (MCTS) is enabled. Disabling filtering "
           "across tile boundaries!\n");
    encCfg->m_disableLFCrossTileBoundaryFlag = true;
  }
  if ((encCfg->m_seiCfg.m_MCTSEncConstraint) && (encCfg->m_TMVPModeId))
  {
    printf("Warning: Constrained Encoding for Motion Constrained Tile Sets (MCTS) is enabled. Disabling TMVP!\n");
    encCfg->m_TMVPModeId = 0;
  }

  if ((encCfg->m_seiCfg.m_MCTSEncConstraint) && (encCfg->m_alf))
  {
    printf("Warning: Constrained Encoding for Motion Constrained Tile Sets (MCTS) is enabled. Disabling ALF!\n");
    encCfg->m_alf = false;
  }
  if ((encCfg->m_seiCfg.m_MCTSEncConstraint) && (encCfg->m_BIO))
  {
    printf("Warning: Constrained Encoding for Motion Constrained Tile Sets (MCTS) is enabled. Disabling BIO!\n");
    encCfg->m_BIO        = false;
    encCfg->m_DMVDBIOExt = false;
  }

  xConfirmPara(encCfg->m_seiCfg.m_sariAspectRatioIdc < 0 || encCfg->m_seiCfg.m_sariAspectRatioIdc > 255,
               "SEISARISampleAspectRatioIdc must be in the range of 0 to 255");

  if (encCfg->m_RCEnableRateControl)
  {
    if (encCfg->m_RCForceIntraQP)
    {
      if (encCfg->m_RCInitialQP == 0)
      {
        msg(WARNING, "\nInitial QP for rate control is not specified. Reset not to use force intra QP!");
        encCfg->m_RCForceIntraQP = false;
      }
    }
    xConfirmPara(encCfg->m_uiDeltaQpRD > 0,
                 "Rate control cannot be used together with slice level multiple-QP optimization!\n");
    if ((encCfg->m_RCCpbSaturationEnabled) && (encCfg->m_level != Level::NONE) && (encCfg->m_profile != Profile::NONE))
    {
      uint32_t uiLevelIdx = (encCfg->m_level / 16) * 4 + (uint32_t)((encCfg->m_level % 16) / 3);
      xConfirmPara(encCfg->m_RCCpbSize > g_uiMaxCpbSize[encCfg->m_tier][uiLevelIdx],
                   "RCCpbSize should be smaller than or equal to Max CPB size according to tier and level");
      xConfirmPara(encCfg->m_RCInitialCpbFullness > 1, "RCInitialCpbFullness should be smaller than or equal to 1");
    }
  }
  else
  {
    xConfirmPara(encCfg->m_RCCpbSaturationEnabled != 0,
                 "Target bits saturation cannot be processed without Rate control");
  }

  if (encCfg->m_seiCfg.m_framePackingSEIEnabled)
  {
    xConfirmPara(encCfg->m_seiCfg.m_framePackingSEIType < 3 || encCfg->m_seiCfg.m_framePackingSEIType > 5,
                 "SEIFramePackingType must be in rage 3 to 5");
  }

  if (encCfg->m_seiCfg.m_doSEIEnabled)
  {
    xConfirmPara(encCfg->m_seiCfg.m_doSEITransformType < 0 || encCfg->m_seiCfg.m_doSEITransformType > 7,
                 "SEIDisplayOrientationTransformType must be in rage 0 to 7");
  }

  if (encCfg->m_seiCfg.m_erpSEIEnabled && !encCfg->m_seiCfg.m_erpSEICancelFlag)
  {
    xConfirmPara(encCfg->m_seiCfg.m_erpSEIGuardBandType < 0 || encCfg->m_seiCfg.m_erpSEIGuardBandType > 8,
                 "SEIEquirectangularprojectionGuardBandType must be in the range of 0 to 7");
    xConfirmPara(
      (encCfg->m_chromaFormatIdc == ChromaFormat::_420 || encCfg->m_chromaFormatIdc == ChromaFormat::_422) &&
        (encCfg->m_seiCfg.m_erpSEILeftGuardBandWidth % 2 == 1),
      "SEIEquirectangularprojectionLeftGuardBandWidth must be an even number for 4:2:0 or 4:2:2 chroma format");
    xConfirmPara(
      (encCfg->m_chromaFormatIdc == ChromaFormat::_420 || encCfg->m_chromaFormatIdc == ChromaFormat::_422) &&
        (encCfg->m_seiCfg.m_erpSEIRightGuardBandWidth % 2 == 1),
      "SEIEquirectangularprojectionRightGuardBandWidth must be an even number for 4:2:0 or 4:2:2 chroma format");
  }

  if (encCfg->m_seiCfg.m_sphereRotationSEIEnabled && !encCfg->m_seiCfg.m_sphereRotationSEICancelFlag)
  {
    xConfirmPara(encCfg->m_seiCfg.m_sphereRotationSEIYaw < -(180 << 16) ||
                   encCfg->m_seiCfg.m_sphereRotationSEIYaw > (180 << 16) - 1,
                 "SEISphereRotationYaw must be in the range of -11 796 480 to 11 796 479");
    xConfirmPara(encCfg->m_seiCfg.m_sphereRotationSEIPitch < -(90 << 16) ||
                   encCfg->m_seiCfg.m_sphereRotationSEIYaw > (90 << 16),
                 "SEISphereRotationPitch must be in the range of -5 898 240 to 5 898 240");
    xConfirmPara(encCfg->m_seiCfg.m_sphereRotationSEIRoll < -(180 << 16) ||
                   encCfg->m_seiCfg.m_sphereRotationSEIYaw > (180 << 16) - 1,
                 "SEISphereRotationRoll must be in the range of -11 796 480 to 11 796 479");
  }

  if (encCfg->m_seiCfg.m_omniViewportSEIEnabled && !encCfg->m_seiCfg.m_omniViewportSEICancelFlag)
  {
    xConfirmPara(encCfg->m_seiCfg.m_omniViewportSEIId < 0 || encCfg->m_seiCfg.m_omniViewportSEIId > 1023,
                 "SEIomniViewportId must be in the range of 0 to 1023");
    xConfirmPara(encCfg->m_seiCfg.m_omniViewportSEICntMinus1 < 0 || encCfg->m_seiCfg.m_omniViewportSEICntMinus1 > 15,
                 "SEIomniViewportCntMinus1 must be in the range of 0 to 15");
    for (uint32_t i = 0; i <= encCfg->m_seiCfg.m_omniViewportSEICntMinus1; i++)
    {
      xConfirmPara(encCfg->m_seiCfg.m_omniViewportSEIAzimuthCentre[i] < -(180 << 16) ||
                     encCfg->m_seiCfg.m_omniViewportSEIAzimuthCentre[i] > (180 << 16) - 1,
                   "SEIOmniViewportAzimuthCentre must be in the range of -11 796 480 to 11 796 479");
      xConfirmPara(encCfg->m_seiCfg.m_omniViewportSEIElevationCentre[i] < -(90 << 16) ||
                     encCfg->m_seiCfg.m_omniViewportSEIElevationCentre[i] > (90 << 16),
                   "SEIOmniViewportSEIElevationCentre must be in the range of -5 898 240 to 5 898 240");
      xConfirmPara(encCfg->m_seiCfg.m_omniViewportSEITiltCentre[i] < -(180 << 16) ||
                     encCfg->m_seiCfg.m_omniViewportSEITiltCentre[i] > (180 << 16) - 1,
                   "SEIOmniViewportTiltCentre must be in the range of -11 796 480 to 11 796 479");
      xConfirmPara(encCfg->m_seiCfg.m_omniViewportSEIHorRange[i] < 1 ||
                     encCfg->m_seiCfg.m_omniViewportSEIHorRange[i] > (360 << 16),
                   "SEIOmniViewportHorRange must be in the range of 1 to 360*2^16");
      xConfirmPara(encCfg->m_seiCfg.m_omniViewportSEIVerRange[i] < 1 ||
                     encCfg->m_seiCfg.m_omniViewportSEIVerRange[i] > (180 << 16),
                   "SEIOmniViewportVerRange must be in the range of 1 to 180*2^16");
    }
  }

  if (encCfg->m_seiCfg.m_gcmpSEIEnabled && !encCfg->m_seiCfg.m_gcmpSEICancelFlag)
  {
    xConfirmPara(encCfg->m_seiCfg.m_gcmpSEIMappingFunctionType < 0 || encCfg->m_seiCfg.m_gcmpSEIMappingFunctionType > 2,
                 "SEIGcmpMappingFunctionType must be in the range of 0 to 2");
    int numFace = encCfg->m_seiCfg.m_gcmpSEIPackingType == 4 || encCfg->m_seiCfg.m_gcmpSEIPackingType == 5 ? 5 : 6;
    for (int i = 0; i < numFace; i++)
    {
      xConfirmPara(encCfg->m_seiCfg.m_gcmpSEIFaceIndex[i] < 0 || encCfg->m_seiCfg.m_gcmpSEIFaceIndex[i] > 5,
                   "SEIGcmpFaceIndex must be in the range of 0 to 5");
      xConfirmPara(encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] < 0 || encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] > 3,
                   "SEIGcmpFaceRotation must be in the range of 0 to 3");
      if (encCfg->m_seiCfg.m_gcmpSEIMappingFunctionType == 2)
      {
        xConfirmPara(encCfg->m_seiCfg.m_gcmpSEIFunctionCoeffU[i] <= 0.0 ||
                       encCfg->m_seiCfg.m_gcmpSEIFunctionCoeffU[i] > 1.0,
                     "SEIGcmpFunctionCoeffU must be in the range (0, 1]");
        xConfirmPara(encCfg->m_seiCfg.m_gcmpSEIFunctionCoeffV[i] <= 0.0 ||
                       encCfg->m_seiCfg.m_gcmpSEIFunctionCoeffV[i] > 1.0,
                     "SEIGcmpFunctionCoeffV must be in the range (0, 1]");
      }
      if (i != 2 && (encCfg->m_seiCfg.m_gcmpSEIPackingType == 4 || encCfg->m_seiCfg.m_gcmpSEIPackingType == 5))
      {
        if (encCfg->m_seiCfg.m_gcmpSEIFaceIndex[2] == 0 || encCfg->m_seiCfg.m_gcmpSEIFaceIndex[2] == 1)
        {
          xConfirmPara(encCfg->m_seiCfg.m_gcmpSEIFaceIndex[i] == 0 || encCfg->m_seiCfg.m_gcmpSEIFaceIndex[i] == 1,
                       "SEIGcmpFaceIndex[i] must be in the range of 2 to 5 for i equal to 0, 1, 3, or 4 when "
                       "SEIGcmpFaceIndex[2] is equal to 0 or 1");
          if (encCfg->m_seiCfg.m_gcmpSEIPackingType == 4)
          {
            xConfirmPara(encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] != 0 &&
                           encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] != 2,
                         "SEIGcmpFaceRotation[i] must be 0 or 2 for i equal to 0, 1, 3, or 4 when SEIGcmpFaceIndex[2] "
                         "is equal to 0 or 1");
          }
          else
          {
            xConfirmPara(encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] != 1 &&
                           encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] != 3,
                         "SEIGcmpFaceRotation[i] must be 1 or 3 for i equal to 0, 1, 3, or 4 when SEIGcmpFaceIndex[2] "
                         "is equal to 0 or 1");
          }
        }
        else if (encCfg->m_seiCfg.m_gcmpSEIFaceIndex[2] == 2 || encCfg->m_seiCfg.m_gcmpSEIFaceIndex[2] == 3)
        {
          xConfirmPara(encCfg->m_seiCfg.m_gcmpSEIFaceIndex[i] == 2 || encCfg->m_seiCfg.m_gcmpSEIFaceIndex[i] == 3,
                       "SEIGcmpFaceIndex[i] must be 0, 1, 4 or 5 for i equal to 0, 1, 3, or 4 when SEIGcmpFaceIndex[2] "
                       "is equal to 2 or 3");
          if (encCfg->m_seiCfg.m_gcmpSEIPackingType == 4)
          {
            if (encCfg->m_seiCfg.m_gcmpSEIFaceIndex[i] == 1)
            {
              xConfirmPara(encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] != 0 &&
                             encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] != 2,
                           "SEIGcmpFaceRotation[i] must be 0 or 2 when SEIGcmpFaceIndex[2] is equal to 2 or 3 and "
                           "SEIGcmpFaceIndex[i] is equal to 1");
            }
            else
            {
              xConfirmPara(encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] != 1 &&
                             encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] != 3,
                           "SEIGcmpFaceRotation[i] must be 1 or 3 when SEIGcmpFaceIndex[2] is equal to 2 or 3 and "
                           "SEIGcmpFaceIndex[i] is equal to 0, 4 or 5");
            }
          }
          else
          {
            if (encCfg->m_seiCfg.m_gcmpSEIFaceIndex[i] == 1)
            {
              xConfirmPara(encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] != 1 &&
                             encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] != 3,
                           "SEIGcmpFaceRotation[i] must be 1 or 3 when SEIGcmpFaceIndex[2] is equal to 2 or 3 and "
                           "SEIGcmpFaceIndex[i] is equal to 1");
            }
            else
            {
              xConfirmPara(encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] != 0 &&
                             encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] != 2,
                           "SEIGcmpFaceRotation[i] must be 0 or 2 when SEIGcmpFaceIndex[2] is equal to 2 or 3 and "
                           "SEIGcmpFaceIndex[i] is equal to 0, 4 or 5");
            }
          }
        }
        else if (encCfg->m_seiCfg.m_gcmpSEIFaceIndex[2] == 4 || encCfg->m_seiCfg.m_gcmpSEIFaceIndex[2] == 5)
        {
          xConfirmPara(encCfg->m_seiCfg.m_gcmpSEIFaceIndex[i] == 4 || encCfg->m_seiCfg.m_gcmpSEIFaceIndex[i] == 5,
                       "SEIGcmpFaceIndex[i] must be in the range of 0 to 3 for i equal to 0, 1, 3, or 4 when "
                       "SEIGcmpFaceIndex[2] is equal to 4 or 5");
          if (encCfg->m_seiCfg.m_gcmpSEIPackingType == 4)
          {
            if (encCfg->m_seiCfg.m_gcmpSEIFaceIndex[i] == 0)
            {
              xConfirmPara(encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] != 0 &&
                             encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] != 2,
                           "SEIGcmpFaceRotation[i] must be 0 or 2 when SEIGcmpFaceIndex[2] is equal to 4 or 5 and "
                           "SEIGcmpFaceIndex[i] is equal to 0");
            }
            else
            {
              xConfirmPara(encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] != 1 &&
                             encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] != 3,
                           "SEIGcmpFaceRotation[i] must be 1 or 3 when SEIGcmpFaceIndex[2] is equal to 4 or 5 and "
                           "SEIGcmpFaceIndex[i] is equal to 1, 2 or 3");
            }
          }
          else
          {
            if (encCfg->m_seiCfg.m_gcmpSEIFaceIndex[i] == 0)
            {
              xConfirmPara(encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] != 1 &&
                             encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] != 3,
                           "SEIGcmpFaceRotation[i] must be 1 or 3 when SEIGcmpFaceIndex[2] is equal to 4 or 5 and "
                           "SEIGcmpFaceIndex[i] is equal to 0");
            }
            else
            {
              xConfirmPara(encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] != 0 &&
                             encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i] != 2,
                           "SEIGcmpFaceRotation[i] must be 0 or 2 when SEIGcmpFaceIndex[2] is equal to 4 or 5 and "
                           "SEIGcmpFaceIndex[i] is equal to 1, 2 or 3");
            }
          }
        }
      }
    }
    if (encCfg->m_seiCfg.m_gcmpSEIGuardBandFlag)
    {
      xConfirmPara(encCfg->m_seiCfg.m_gcmpSEIGuardBandSamplesMinus1 < 0 ||
                     encCfg->m_seiCfg.m_gcmpSEIGuardBandSamplesMinus1 > 15,
                   "SEIGcmpGuardBandSamplesMinus1 must be in the range of 0 to 15");
    }
  }

#if JVET_Z0120_SII_SEI_PROCESSING
  if (encCfg->m_seiCfg.m_siiSEIEnabled && encCfg->m_seiCfg.m_ShutterFilterEnable)
  {
    xConfirmPara(encCfg->m_maxTempLayer == 1 || encCfg->m_maxDecPicBuffering[0] == 1,
                 "Shutter Interval SEI message processing is disabled for single TempLayer and single frame in DPB\n");
  }
#endif

  if (encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsEnabled)
  {
    for (int i = 0; i < encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsNumFilters; i++)
    {
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsId[i] > MAX_NNPFC_ID,
                   "SEINNPostFilterCharacteristicsId must be in the range of 0 to 2^32-2");
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsModeIdc[i] > 255,
                   "SEINNPostFilterCharacteristicsModeIdc must be in the range of 0 to 255");
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsPurpose[i] > 1023,
                   "SEINNPostFilterCharacteristicsPurpose must be in the range of 0 to 1023");
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsNumberInputDecodedPicturesMinus1[i] > 63,
                   "SEINNPostFilterCharacteristicsNumberInputDecodedPicturesMinus1 must be in the range of 0 to 63");
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsInpTensorBitDepthLumaMinus8[i] > 24,
                   "SEINNPostFilterCharacteristicsInpTensorBitDepthLumaMinus8 must be in the range of 0 to 24");
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsInpTensorBitDepthChromaMinus8[i] > 24,
                   "SEINNPostFilterCharacteristicsInpTensorBitDepthChromaMinus8 must be in the range of 0 to 24");
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsOutTensorBitDepthLumaMinus8[i] > 24,
                   "SEINNPostFilterCharacteristicsOutTensorBitDepthLumaMinus8 must be in the range of 0 to 24");
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsOutTensorBitDepthChromaMinus8[i] > 24,
                   "SEINNPostFilterCharacteristicsOutTensorBitDepthChromaMinus8 must be in the range of 0 to 24");
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsInpFormatIdc[i] > 255,
                   "SEINNPostFilterCharacteristicsInpFormatIdc must be in the range of 0 to 255");
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsInpOrderIdc[i] > 255,
                   "SEINNPostFilterCharacteristicsInpOrderIdc must be in the range of  0 to 255");
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsColPrimaries[i] > 255,
                   "m_nnPostFilterSEICharacteristicsColPrimaries must in the range 0 to 255");
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsTransCharacteristics[i] > 255,
                   "m_nnPostFilterSEICharacteristicsTransCharacteristics must in the range 0 to 255");
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsMatrixCoeffs[i] > 255,
                   "m_nnPostFilterSEICharacteristicsMatrixCoeffs must in the range 0 to 255");
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsOutFormatIdc[i] > 255,
                   "SEINNPostFilterCharacteristicsOutFormatIdc must be in the range of 0 to 255");
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsOutOrderIdc[i] > 255,
                   "SEINNPostFilterCharacteristicsOutOrderIdc must be in the range of 0 to 255");
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsPatchWidthMinus1[i] > 32766,
                   "SEINNPostFilterCharacteristicsPatchWidthMinus1 must be in the range of 0 to 32766");
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsPatchHeightMinus1[i] > 32766,
                   "SEINNPostFilterCharacteristicsPatchHeightMinus1 must be in the range of 0 to 32766");
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsOverlap[i] > 16383,
                   "SEINNPostFilterCharacteristicsOverlap must be in the range of 0 to 16383");
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsPaddingType[i] > (1 << 4) - 1,
                   "SEINNPostFilterPaddingType must be in the range of 0 to 2^4-1");
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsLog2ParameterBitLengthMinus3[i] > 3,
                   "SEINNPostFilterCharacteristicsLog2ParameterBitLengthMinus3 must be in the range of 0 to 3");
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsNumParametersIdc[i] > 52,
                   "SEINNPostFilterCharacteristicsNumParametersIdc must be in the range of 0 to 52");
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsTotalKilobyteSize[i] >
                     (uint32_t)(((uint64_t)1 << 32) - 2),
                   "SEINNPostFilterCharacteristicsTotalKilobyteSize must be in the range of 0 to 2^32-2");
      xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsNumKmacOperationsIdc[i] >
                     (uint32_t)(((uint64_t)1 << 32) - 2),
                   "SEICharacteristicsNumKmacOperationsIdc must be in the range of 0 to 2^32-2");
    }
  }

  if (encCfg->m_seiCfg.m_nnPostFilterSEIActivationEnabled)
  {
    xConfirmPara(encCfg->m_seiCfg.m_nnPostFilterSEIActivationTargetId > MAX_NNPFA_ID,
                 "SEINNPostFilterActivationTargetId must be in the range of 0 to 2^32-2");
  }

  if (encCfg->m_seiCfg.m_phaseIndicationSEIEnabledFullResolution)
  {
    xConfirmPara(encCfg->m_seiCfg.m_horPhaseNumFullResolution < 0,
                 "m_horPhaseNumFullResolution must be in the range of 0 to 511, inclusive");
    xConfirmPara(encCfg->m_seiCfg.m_horPhaseDenMinus1FullResolution < 0,
                 "m_horPhaseDenMinus1FullResolution must be in the range of 0 to 511, inclusive");
    xConfirmPara(encCfg->m_seiCfg.m_verPhaseNumFullResolution < 0,
                 "m_verPhaseNumFullResolution must be in the range of 0 to 511, inclusive");
    xConfirmPara(encCfg->m_seiCfg.m_verPhaseDenMinus1FullResolution < 0,
                 "m_verPhaseDenMinus1FullResolution must be in the range of 0 to 511, inclusive");
    xConfirmPara(encCfg->m_seiCfg.m_horPhaseDenMinus1FullResolution > 511,
                 "m_horPhaseDenMinus1FullResolution must be in the range of 0 to 511, inclusive");
    xConfirmPara(
      encCfg->m_seiCfg.m_horPhaseNumFullResolution > encCfg->m_seiCfg.m_horPhaseDenMinus1FullResolution + 1,
      "m_horPhaseNumFullResolution must be in the range of 0 to m_horPhaseDenMinus1FullResolution + 1, inclusive");
    xConfirmPara(encCfg->m_seiCfg.m_verPhaseDenMinus1FullResolution > 511,
                 "m_verPhaseDenMinus1FullResolution must be in the range of 0 to 511, inclusive");
    xConfirmPara(
      encCfg->m_seiCfg.m_verPhaseNumFullResolution > encCfg->m_seiCfg.m_verPhaseDenMinus1FullResolution + 1,
      "m_verPhaseNumFullResolution must be in the range of 0 to m_verPhaseDenMinus1FullResolution + 1, inclusive");
  }
  if (encCfg->m_seiCfg.m_phaseIndicationSEIEnabledReducedResolution)
  {
    xConfirmPara(encCfg->m_seiCfg.m_horPhaseNumReducedResolution < 0,
                 "m_horPhaseNumReducedResolution must be in the range of 0 to 511, inclusive");
    xConfirmPara(encCfg->m_seiCfg.m_horPhaseDenMinus1ReducedResolution < 0,
                 "m_horPhaseDenMinus1ReducedResolution must be in the range of 0 to 511, inclusive");
    xConfirmPara(encCfg->m_seiCfg.m_verPhaseNumReducedResolution < 0,
                 "m_verPhaseNumReducedResolution must be in the range of 0 to 511, inclusive");
    xConfirmPara(encCfg->m_seiCfg.m_verPhaseDenMinus1ReducedResolution < 0,
                 "m_verPhaseDenMinus1ReducedResolution must be in the range of 0 to 511, inclusive");
    xConfirmPara(encCfg->m_seiCfg.m_horPhaseDenMinus1ReducedResolution > 511,
                 "m_horPhaseDenMinus1ReducedResolution must be in the range of 0 to 511, inclusive");
    xConfirmPara(encCfg->m_seiCfg.m_horPhaseNumReducedResolution >
                   encCfg->m_seiCfg.m_horPhaseDenMinus1ReducedResolution + 1,
                 "m_horPhaseNumReducedResolution must be in the range of 0 to m_horPhaseDenMinus1ReducedResolution + "
                 "1, inclusive");
    xConfirmPara(encCfg->m_seiCfg.m_verPhaseDenMinus1ReducedResolution > 511,
                 "m_verPhaseDenMinus1ReducedResolution must be in the range of 0 to 511, inclusive");
    xConfirmPara(encCfg->m_seiCfg.m_verPhaseNumReducedResolution >
                   encCfg->m_seiCfg.m_verPhaseDenMinus1ReducedResolution + 1,
                 "m_verPhaseNumReducedResolution must be in the range of 0 to m_verPhaseDenMinus1ReducedResolution + "
                 "1, inclusive");
  }

  xConfirmPara(encCfg->m_log2ParallelMergeLevel < 2, "Log2ParallelMergeLevel should be larger than or equal to 2");
  xConfirmPara(encCfg->m_log2ParallelMergeLevel > encCfg->m_CTUSize,
               "Log2ParallelMergeLevel should be less than or equal to CTU size");
  xConfirmPara(encCfg->m_seiCfg.m_preferredTransferCharacteristics > 255,
               "transfer_characteristics_idc should not be greater than 255.");
  xConfirmPara(unsigned(encCfg->m_ImvMode) > 1, "ImvMode exceeds range (0 to 1)");
  if (encCfg->m_AffineAmvr)
  {
    xConfirmPara(!encCfg->m_ImvMode, "AffineAmvr cannot be used when IMV is disabled.");
  }
  xConfirmPara(encCfg->m_decodeBitstreams[0] == encCfg->m_bitstreamFileName,
               "Debug bitstream and the output bitstream cannot be equal.\n");
  xConfirmPara(encCfg->m_decodeBitstreams[1] == encCfg->m_bitstreamFileName,
               "Decode2 bitstream and the output bitstream cannot be equal.\n");
  xConfirmPara(unsigned(encCfg->m_LMChroma) > 1, "LMMode exceeds range (0 to 1)");
  xConfirmPara((encCfg->m_LMChroma == 0) && encCfg->m_CCCM, "LMMode disabled but CCCM enabled");
  if (encCfg->m_gopBasedTemporalFilterEnabled)
  {
    xConfirmPara(encCfg->m_temporalSubsampleRatio != 1,
                 "GOP Based Temporal Filter only support Temporal sub-sample ratio 1");
    xConfirmPara(
      encCfg->m_gopBasedTemporalFilterPastRefs <= 0 && encCfg->m_gopBasedTemporalFilterFutureRefs <= 0,
      "Either TemporalFilterPastRefs or TemporalFilterFutureRefs must be larger than 0 when TemporalFilter is enabled");

    if ((encCfg->m_gopBasedTemporalFilterPastRefs != 0 &&
         encCfg->m_gopBasedTemporalFilterPastRefs != TF_DEFAULT_REFS) ||
        (encCfg->m_gopBasedTemporalFilterFutureRefs != 0 &&
         encCfg->m_gopBasedTemporalFilterFutureRefs != TF_DEFAULT_REFS))
    {
      msg(WARNING, "Number of frames used for temporal prefilter is different from default.\n");
    }
    xConfirmPara(encCfg->m_gopBasedTemporalFilterUnitSize < 8,
                 "GOP Based Temmporal Fitler unit size has to be bigger or equal than 8");
    xConfirmPara(encCfg->m_gopBasedTemporalFilterUnitSize > 32,
                 "GOP Based Temmporal Fitler unit size has to be smaller or equal than 32");
    xConfirmPara(encCfg->m_gopBasedTemporalFilterUnitSize & (encCfg->m_gopBasedTemporalFilterUnitSize - 1),
                 "GOP Based Temmporal Fitler unit size has to be a power of 2");
  }
  if (encCfg->m_bimEnabled)
  {
    xConfirmPara(encCfg->m_temporalSubsampleRatio != 1,
                 "Block Importance Mapping only support Temporal sub-sample ratio 1");
    xConfirmPara(encCfg->m_gopBasedTemporalFilterPastRefs <= 0 && encCfg->m_gopBasedTemporalFilterFutureRefs <= 0,
                 "Either TemporalFilterPastRefs or TemporalFilterFutureRefs must be larger than 0 when Block "
                 "Importance Mapping is enabled");
  }
#if EXTENSION_360_VIDEO
  check_failed |= encCfg->m_ext360.verifyParameters();
#endif

  xConfirmPara(encCfg->m_CTUSize <= 32 && (encCfg->m_log2MaxTbSize == 6),
               "Log2MaxTbSize must be less than 6 when CTU size is 32");

  xConfirmPara(encCfg->m_Affine < 0 || encCfg->m_Affine > 4, "Affine out of range [0..4]");
  xConfirmPara(encCfg->m_SMVD < 0 || encCfg->m_SMVD > 3, "SMVD out of range [0..3]");

  xConfirmPara(encCfg->m_TIMDSAD && !encCfg->m_TIMD, "TIMD must be enabled when TIMD-SAD is enabled");
  xConfirmPara(encCfg->m_OBIC && !encCfg->m_DIMD, "DIMD must be enabled when OBIC is enabled");

#undef xConfirmPara
  return check_failed;
}

const char *profileToString(const Profile::Name profile)
{
  static const uint32_t numberOfProfiles = sizeof(strToProfile) / sizeof(*strToProfile);

  for (uint32_t profileIndex = 0; profileIndex < numberOfProfiles; profileIndex++)
  {
    if (strToProfile[profileIndex].value == profile)
    {
      return strToProfile[profileIndex].str;
    }
  }

  // if we get here, we didn't find this profile in the list - so there is an error
  EXIT("ERROR: Unknown profile \"" << profile << "\" in profileToString");
  return "";
}

void EncAppCfg::xPrintParameter(EncCfg *encCfg)
{
  // msg( DETAILS, "\n" );
  msg(DETAILS, "Input          File                    : %s\n", encCfg->m_inputFileName.c_str());
  msg(DETAILS, "Bitstream      File                    : %s\n", encCfg->m_bitstreamFileName.c_str());
  msg(DETAILS, "Reconstruction File                    : %s\n", encCfg->m_reconFileName.c_str());
  msg(DETAILS, "TIMD usage statistics                  : %s\n",
      encCfg->m_timdUsageStats ? encCfg->m_timdUsageStatsRoot.c_str() : "Disabled");
#if JVET_Z0120_SII_SEI_PROCESSING
  if (encCfg->m_seiCfg.m_ShutterFilterEnable && !encCfg->m_seiCfg.m_shutterIntervalPreFileName.empty())
  {
    msg(DETAILS, "SII Pre-processed File                 : %s\n",
        encCfg->m_seiCfg.m_shutterIntervalPreFileName.c_str());
  }
#endif
  msg(DETAILS, "Real     Format                        : %dx%d %gHz\n",
      encCfg->m_sourceWidth - encCfg->m_confWinLeft - encCfg->m_confWinRight,
      encCfg->m_sourceHeight - encCfg->m_confWinTop - encCfg->m_confWinBottom,
      (double)encCfg->m_frameRate / encCfg->m_temporalSubsampleRatio);
  msg(DETAILS, "Internal Format                        : %dx%d %gHz\n", encCfg->m_sourceWidth, encCfg->m_sourceHeight,
      (double)encCfg->m_frameRate / encCfg->m_temporalSubsampleRatio);
  msg(DETAILS, "Sequence PSNR output                   : %s\n",
      (encCfg->m_printMSEBasedSequencePSNR ? "Linear average, MSE-based" : "Linear average only"));
  msg(DETAILS, "Hexadecimal PSNR output                : %s\n", (encCfg->m_printHexPsnr ? "Enabled" : "Disabled"));
  msg(DETAILS, "Sequence MSE output                    : %s\n", (encCfg->m_printSequenceMSE ? "Enabled" : "Disabled"));
  msg(DETAILS, "Frame MSE output                       : %s\n", (encCfg->m_printFrameMSE ? "Enabled" : "Disabled"));
  msg(DETAILS, "MS-SSIM output                         : %s\n", (encCfg->m_printMSSSIM ? "Enabled" : "Disabled"));
  msg(DETAILS, "Cabac-zero-word-padding                : %s\n",
      (encCfg->m_cabacZeroWordPaddingEnabled ? "Enabled" : "Disabled"));
  if (encCfg->m_fieldSeqFlag)
  {
    msg(DETAILS, "Frame/Field                            : Field based coding\n");
    msg(DETAILS, "Field index                            : %u - %d (%d fields)\n", encCfg->m_frameSkip,
        encCfg->m_frameSkip + encCfg->m_framesToBeEncoded - 1, encCfg->m_framesToBeEncoded);
    msg(DETAILS, "Field Order                            : %s field first\n",
        encCfg->m_isTopFieldFirst ? "Top" : "Bottom");
  }
  else
  {
    msg(DETAILS, "Frame/Field                            : Frame based coding\n");
    msg(DETAILS, "Frame index                            : %u - %d (%d frames)\n", encCfg->m_frameSkip,
        encCfg->m_frameSkip + encCfg->m_framesToBeEncoded - 1, encCfg->m_framesToBeEncoded);
  }
  {
    msg(DETAILS, "Profile                                : %s\n", profileToString(encCfg->m_profile));
  }
  msg(DETAILS, "AllRapPicturesFlag                     : %d\n", encCfg->m_allRapPicturesFlag);

  msg(DETAILS, "subpicture info present flag           : %s\n",
      encCfg->m_subPicInfoPresentFlag ? "Enabled" : "Disabled");
  if (encCfg->m_subPicInfoPresentFlag)
  {
    msg(DETAILS, "number of subpictures                  : %d\n", encCfg->m_numSubPics);
    msg(DETAILS, "subpicture size same flag              : %d\n", encCfg->m_subPicSameSizeFlag);
    if (encCfg->m_subPicSameSizeFlag)
    {
      msg(DETAILS, "[0]th subpicture size                  : [%d %d]\n", encCfg->m_subPicWidth[0],
          encCfg->m_subPicHeight[0]);
    }
    for (int i = 0; i < encCfg->m_numSubPics; i++)
    {
      if (!encCfg->m_subPicSameSizeFlag)
      {
        msg(DETAILS, "[%d]th subpicture location              : [%d %d]\n", i, encCfg->m_subPicCtuTopLeftX[i],
            encCfg->m_subPicCtuTopLeftY[i]);
        msg(DETAILS, "[%d]th subpicture size                  : [%d %d]\n", i, encCfg->m_subPicWidth[i],
            encCfg->m_subPicHeight[i]);
      }
      msg(DETAILS, "[%d]th subpicture treated as picture    : %d\n", i,
          encCfg->m_subPicTreatedAsPicFlag[i] ? "Enabled" : "Disabled");
      msg(DETAILS, "loop filter across [%d]th subpicture    : %d\n", i,
          encCfg->m_loopFilterAcrossSubpicEnabledFlag[i] ? "Enabled" : "Disabled");
    }
  }

  msg(DETAILS, "subpicture ID present flag             : %s\n",
      encCfg->m_subPicIdMappingExplicitlySignalledFlag ? "Enabled" : "Disabled");
  if (encCfg->m_subPicIdMappingExplicitlySignalledFlag)
  {
    msg(DETAILS, "subpicture ID signalling present flag  : %d\n", encCfg->m_subPicIdMappingInSpsFlag);
    for (int i = 0; i < encCfg->m_numSubPics; i++)
    {
      msg(DETAILS, "[%d]th subpictures ID length           : %d\n", i, encCfg->m_subPicIdLen);
      msg(DETAILS, "[%d]th subpictures ID                  : %d\n", i, encCfg->m_subPicId[i]);
    }
  }
  msg(DETAILS, "Max TB size                            : %d \n", 1 << encCfg->m_log2MaxTbSize);
  msg(DETAILS, "Motion search range                    : %d\n", encCfg->m_searchRange);
  msg(DETAILS, "Intra period                           : %d\n", encCfg->m_intraPeriod);
  msg(DETAILS, "Decoding refresh type                  : %d\n", encCfg->m_decodingRefreshType);
  msg(DETAILS, "DRAP period                            : %d\n", encCfg->m_drapPeriod);
  msg(DETAILS, "EDRAP period                           : %d\n", encCfg->m_edrapPeriod);
  if (encCfg->m_qpIncrementAtSourceFrame >= 0)
  {
    msg(DETAILS, "QP                                     : %d (incrementing internal QP at source frame %d)\n",
        encCfg->m_iQP, encCfg->m_qpIncrementAtSourceFrame);
  }
  else
  {
    msg(DETAILS, "QP                                     : %d\n", encCfg->m_iQP);
  }
  msg(DETAILS, "Max dQP signaling subdiv               : %d\n", encCfg->m_cuQpDeltaSubdiv);

  msg(DETAILS, "Cb QP Offset (dual tree)               : %d (%d)\n", encCfg->m_chromaCbQpOffset,
      encCfg->m_chromaCbQpOffsetDualTree);
  msg(DETAILS, "Cr QP Offset (dual tree)               : %d (%d)\n", encCfg->m_chromaCrQpOffset,
      encCfg->m_chromaCrQpOffsetDualTree);
  msg(DETAILS, "QP adaptation                          : %d (range=%d)\n", encCfg->m_bUseAdaptiveQP,
      (encCfg->m_bUseAdaptiveQP ? encCfg->m_iQPAdaptationRange : 0));
  msg(DETAILS, "GOP size                               : %d\n", encCfg->m_gopSize);
  msg(DETAILS, "Input bit depth                        : (Y:%d, C:%d)\n", encCfg->m_inputBitDepth[ChannelType::LUMA],
      encCfg->m_inputBitDepth[ChannelType::CHROMA]);
  msg(DETAILS, "MSB-extended bit depth                 : (Y:%d, C:%d)\n",
      encCfg->m_msbExtendedBitDepth[ChannelType::LUMA], encCfg->m_msbExtendedBitDepth[ChannelType::CHROMA]);
  msg(DETAILS, "Internal bit depth                     : (Y:%d, C:%d)\n", encCfg->m_internalBitDepth[ChannelType::LUMA],
      encCfg->m_internalBitDepth[ChannelType::CHROMA]);
  if (encCfg->m_cuChromaQpOffsetList.size() > 0)
  {
    msg(DETAILS, "Chroma QP offset list                  : (");
    for (int i = 0; i < encCfg->m_cuChromaQpOffsetList.size(); i++)
    {
      msg(DETAILS, "%d %d %d%s", encCfg->m_cuChromaQpOffsetList[i].u.comp.cbOffset,
          encCfg->m_cuChromaQpOffsetList[i].u.comp.crOffset, encCfg->m_cuChromaQpOffsetList[i].u.comp.jointCbCrOffset,
          (i + 1 < encCfg->m_cuChromaQpOffsetList.size() ? ", " : ")\n"));
    }
    msg(DETAILS, "cu_chroma_qp_offset_subdiv             : %d\n", encCfg->m_cuChromaQpOffsetSubdiv);
    msg(DETAILS, "cu_chroma_qp_offset_enabled_flag       : %s\n",
        (encCfg->m_cuChromaQpOffsetEnabled ? "Enabled" : "Disabled"));
  }
  else
  {
    msg(DETAILS, "Chroma QP offset list                  : Disabled\n");
  }
  msg(DETAILS, "extended_precision_processing_flag     : %s\n",
      (encCfg->m_extendedPrecisionProcessingFlag ? "Enabled" : "Disabled"));
  msg(DETAILS, "TSRC_Rice_present_flag                 : %s\n",
      (encCfg->m_tsrcRicePresentFlag ? "Enabled" : "Disabled"));
  msg(DETAILS, "reverse_last_sig_coeff_enabled_flag    : %s\n",
      (encCfg->m_reverseLastSigCoeffEnabledFlag ? "Enabled" : "Disabled"));
  msg(DETAILS, "transform_skip_rotation_enabled_flag   : %s\n",
      (encCfg->m_transformSkipRotationEnabledFlag ? "Enabled" : "Disabled"));
  msg(DETAILS, "transform_skip_context_enabled_flag    : %s\n",
      (encCfg->m_transformSkipContextEnabledFlag ? "Enabled" : "Disabled"));
  msg(DETAILS, "high_precision_offsets_enabled_flag    : %s\n",
      (encCfg->m_highPrecisionOffsetsEnabledFlag ? "Enabled" : "Disabled"));
  msg(DETAILS, "rrc_rice_extension_flag                : %s\n",
      (encCfg->m_rrcRiceExtensionEnableFlag ? "Enabled" : "Disabled"));
  msg(DETAILS, "persistent_rice_adaptation_enabled_flag: %s\n",
      (encCfg->m_persistentRiceAdaptationEnabledFlag ? "Enabled" : "Disabled"));
  msg(DETAILS, "cabac_bypass_alignment_enabled_flag    : %s\n",
      (encCfg->m_cabacBypassAlignmentEnabledFlag ? "Enabled" : "Disabled"));

  switch (encCfg->m_costMode)
  {
  case COST_STANDARD_LOSSY:
    msg(DETAILS, "Cost function:                         : Lossy coding (default)\n");
    break;
  case COST_SEQUENCE_LEVEL_LOSSLESS:
    msg(DETAILS, "Cost function:                         : Sequence_level_lossless coding\n");
    break;
  case COST_LOSSLESS_CODING:
    msg(DETAILS, "Cost function:                         : Lossless coding with fixed QP of %d\n",
        LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP);
    break;
  case COST_MIXED_LOSSLESS_LOSSY_CODING:
    msg(DETAILS,
        "Cost function:                         : Mixed_lossless_lossy coding with QP'=%d for lossless evaluation\n",
        LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME);
    break;
  default:
    msg(DETAILS, "Cost function:                         : Unknown\n");
    break;
  }

  msg(DETAILS, "RateControl                            : %d\n", encCfg->m_RCEnableRateControl);
  msg(DETAILS, "WeightedPredMethod                     : %d\n", int(encCfg->m_weightedPredictionMethod));

  if (encCfg->m_RCEnableRateControl)
  {
    msg(DETAILS, "TargetBitrate                          : %d\n", encCfg->m_RCTargetBitrate);
    msg(DETAILS, "KeepHierarchicalBit                    : %d\n", encCfg->m_RCKeepHierarchicalBit);
    msg(DETAILS, "LCULevelRC                             : %d\n", encCfg->m_RCLCULevelRC);
    msg(DETAILS, "UseLCUSeparateModel                    : %d\n", encCfg->m_RCUseLCUSeparateModel);
    msg(DETAILS, "InitialQP                              : %d\n", encCfg->m_RCInitialQP);
    msg(DETAILS, "ForceIntraQP                           : %d\n", encCfg->m_RCForceIntraQP);
    msg(DETAILS, "CpbSaturation                          : %d\n", encCfg->m_RCCpbSaturationEnabled);
    if (encCfg->m_RCCpbSaturationEnabled)
    {
      msg(DETAILS, "CpbSize                                : %d\n", encCfg->m_RCCpbSize);
      msg(DETAILS, "InitalCpbFullness                      : %.2f\n", encCfg->m_RCInitialCpbFullness);
    }
  }

  msg(VERBOSE, "TOOL CFG: ");
  msg(VERBOSE, "CTU:%d QT%d/%dBTT%d/%d ", encCfg->m_CTUSize, floorLog2(encCfg->m_CTUSize / encCfg->m_minQt[0]),
      floorLog2(encCfg->m_CTUSize / encCfg->m_minQt[1]), encCfg->m_uiMaxMTTHierarchyDepthI,
      encCfg->m_uiMaxMTTHierarchyDepth);
  msg(VERBOSE, "MaxTU:%d ", 1 << encCfg->m_log2MaxTbSize);
  msg(VERBOSE, "IBD:%d ",
      ((encCfg->m_internalBitDepth[ChannelType::LUMA] > encCfg->m_msbExtendedBitDepth[ChannelType::LUMA]) ||
       (encCfg->m_internalBitDepth[ChannelType::CHROMA] > encCfg->m_msbExtendedBitDepth[ChannelType::CHROMA])));
  msg(VERBOSE, "TransformSkip:%d ", encCfg->m_useTransformSkip);
  msg(VERBOSE, "TransformSkipFast:%d ", encCfg->m_useTransformSkipFast);
  msg(VERBOSE, "TransformSkipLog2MaxSize:%d ", encCfg->m_log2MaxTransformSkipBlockSize);
  msg(VERBOSE, "ChromaTS:%d ", encCfg->m_useChromaTS);
  msg(VERBOSE, "BDPCM:%d ", encCfg->m_useBDPCM);
  msg(VERBOSE, "Tiles: %dx%d ", encCfg->m_numTileCols, encCfg->m_numTileRows);
  msg(VERBOSE, "Slices: %d ", encCfg->m_numSlicesInPic);
  msg(VERBOSE, "MCTS:%d ", encCfg->m_seiCfg.m_MCTSEncConstraint);
#if ENABLE_NNLF
  msg(VERBOSE, "NNLF:%d ", encCfg->m_nnlf);
  msg(VERBOSE, "NnlfDebugOption:%d ", encCfg->m_nnlfDebugOption);
  if (encCfg->m_nnlf > 0)
  {
    msg(VERBOSE, "(StartPOC==%d) ", encCfg->m_nnlfStartPoc);
    msg(VERBOSE, "(PocDivisibleByN==%d) ", encCfg->m_nnlfPocDivisibleByN);
  }
#endif
  msg(VERBOSE, "SAO:%d ", (encCfg->m_useSao) ? (1) : (0));
  msg(VERBOSE, "CCSAO:%d ", encCfg->m_CCSAO ? 1 : 0);
  msg(VERBOSE, "ALF:%d ", encCfg->m_alf ? 1 : 0);
  if (encCfg->m_alf)
  {
    msg(VERBOSE, "UseNonLinearAlfLuma:%d ", encCfg->m_useNonLinearAlfLuma);
    msg(VERBOSE, "UseNonLinearAlfChroma:%d ", encCfg->m_useNonLinearAlfChroma);
    msg(VERBOSE, "MaxNumAlfAlternativesChroma:%d ", encCfg->m_maxNumAlfAlternativesChroma);
    msg(VERBOSE, "AlfImprovements:%d ", encCfg->m_alfImprovements ? 1 : 0);
    msg(VERBOSE, "CCALF:%d ", encCfg->m_ccalf ? 1 : 0);
    msg(VERBOSE, "ALFCCCM:%d ", encCfg->m_lfCccm ? 1 : 0);
    msg(VERBOSE, "MaxNumALFAPS %d ", encCfg->m_maxNumAlfAps);
    msg(VERBOSE, "AlfapsIDShift %d ", encCfg->m_alfapsIDShift);
  }
  msg(VERBOSE, "BIF:%d ", encCfg->m_BIF);
  if (encCfg->m_BIF)
  {
    msg(VERBOSE, "BIFStrength:%d ", encCfg->m_BIFStrength);
    msg(VERBOSE, "BIFQPOffset:%d ", encCfg->m_BIFQPOffset);
  }
  msg(VERBOSE, "ChromaBIF:%d ", encCfg->m_chromaBIF);
  if (encCfg->m_chromaBIF)
  {
    msg(VERBOSE, "ChromaBIFStrength:%d ", encCfg->m_chromaBIFStrength);
    msg(VERBOSE, "ChromaBIFQPOffset:%d ", encCfg->m_chromaBIFQPOffset);
  }
  msg(VERBOSE, "ConstantJointCbCrSignFlag:%d ", encCfg->m_constantJointCbCrSignFlag);
  msg(VERBOSE, "WPP:%d ", (int)encCfg->m_useWeightedPred);
  msg(VERBOSE, "WPB:%d ", (int)encCfg->m_useWeightedBiPred);
  msg(VERBOSE, "PME:%d ", encCfg->m_log2ParallelMergeLevel);
  const int wavefrontSubstreams =
    encCfg->m_entropyCodingSyncEnabledFlag ? (encCfg->m_sourceHeight + encCfg->m_CTUSize - 1) / encCfg->m_CTUSize : 1;
  msg(VERBOSE, "WaveFrontSynchro:%d WaveFrontSubstreams:%d ", encCfg->m_entropyCodingSyncEnabledFlag ? 1 : 0,
      wavefrontSubstreams);
  msg(VERBOSE, "ScalingList:%d ", encCfg->m_useScalingListId);
  msg(VERBOSE, "TMVPMode:%d ", encCfg->m_TMVPModeId);
  msg(VERBOSE, "DQ:%d ", encCfg->m_DepQuantEnabledIdc);
  msg(VERBOSE, "SignBitHidingFlag:%d ", encCfg->m_SignDataHidingEnabledFlag);
  msg(VERBOSE, "SignPred:%d ", encCfg->m_numPredSign);
  msg(VERBOSE, "TempCABAC:%d ", encCfg->m_tempCabacInitMode);
  msg(VERBOSE, "Log2SignPredArea:%d ", encCfg->m_log2SignPredArea);

  {
    msg(VERBOSE, "\nTOOL CFG: ");
    msg(VERBOSE, "IntraLFNSTISlice:%d ", encCfg->m_intraLFNSTISlice);
    msg(VERBOSE, "IntraLFNSTPBSlice:%d ", encCfg->m_intraLFNSTPBSlice);
    msg(VERBOSE, "InterLFNST:%d ", encCfg->m_interLFNST);
    msg(VERBOSE, "InterLFNSTSBT:%d ", encCfg->m_interLFNSTSBT);
    msg(VERBOSE, "MMVD:%d ", encCfg->m_MMVD);
    msg(VERBOSE, "Affine:%d ", encCfg->m_Affine);
    if (encCfg->m_Affine)
    {
      msg(VERBOSE, "AffineType:%d ", encCfg->m_AffineType);
      msg(VERBOSE, "AdaptBypassAffineMe:%d ", encCfg->m_adaptBypassAffineMe);
      msg(VERBOSE, "AffineMMVD:%d ", encCfg->m_AffineMmvdMode);
      msg(VERBOSE, "AffineParameterRefinement:%d ", encCfg->m_affineParaRefinement);
      msg(VERBOSE, "MinAffineBlkSize:%d ", encCfg->m_minAffineBlkSize);
      msg(VERBOSE, "PROF:%d ", encCfg->m_PROF);
      msg(VERBOSE, "AffineSubBlockMrgExt:%d ", encCfg->m_affineSbMrgExt);
    }
    msg(VERBOSE, "SbTMVP:%d ", encCfg->m_sbTmvpEnableFlag);
    msg(VERBOSE, "DualITree:%d ", encCfg->m_dualITree);
    msg(VERBOSE, "IMV:%d ", encCfg->m_ImvMode);
    msg(VERBOSE, "BIO:%d ", encCfg->m_BIO);
    msg(VERBOSE, "LMChroma:%d ", encCfg->m_LMChroma);
    msg(VERBOSE, "CCCM:%d ", encCfg->m_CCCM);
    msg(VERBOSE, "HorCollocatedChroma:%d ", encCfg->m_horCollocatedChromaFlag);
    msg(VERBOSE, "VerCollocatedChroma:%d ", encCfg->m_verCollocatedChromaFlag);

    {
      std::string s;
      const int   m = encCfg->m_mtsMode + (encCfg->m_implicitMtsIntra ? 4 : 0);
      if (m != 0)
      {
        s = "(";
        s += (m & 1) != 0 ? "explicit intra" : "implicit intra";
        if (m & 2)
        {
          s += ", explicit inter";
        }
        s += ")";
      }
      msg(VERBOSE, "MTS:%d%s ", m != 0, s.c_str());
      if (m & 2)
      {
        msg(VERBOSE, "InterMTSMaxSize: %d ", encCfg->m_interMTSMaxSize);
      }
    }
    msg(VERBOSE, "SBT:%d ", encCfg->m_SBT);
    msg(VERBOSE, "SMVD:%d ", encCfg->m_SMVD);
    msg(VERBOSE, "CompositeLTReference:%d ", encCfg->m_compositeRefEnabled);
    msg(VERBOSE, "Bcw:%d ", encCfg->m_bcw);
    if (encCfg->m_bcw)
    {
      msg(VERBOSE, "BcwFast:%d ", encCfg->m_BcwFast);
    }
    msg(VERBOSE, "LADF:%d ", encCfg->m_ladfEnabled);
    msg(VERBOSE, "CIIP:%d ", encCfg->m_ciip);
    msg(VERBOSE, "Geo:%d ", encCfg->m_Geo);
    msg(VERBOSE, "SGPM:%d ", encCfg->m_sgpm);
    if (encCfg->m_sgpm)
    {
      msg(VERBOSE, "SGPMNoBlend:%d ", encCfg->m_sgpmNoBlend);
    }
    encCfg->m_allowDisFracMMVD = encCfg->m_MMVD ? encCfg->m_allowDisFracMMVD : false;
    if (encCfg->m_MMVD)
    {
      msg(VERBOSE, "AllowDisFracMMVD:%d ", encCfg->m_allowDisFracMMVD);
    }
    if (encCfg->m_Affine)
    {
      msg(VERBOSE, "AffineAmvr:%d ", encCfg->m_AffineAmvr);
      msg(VERBOSE, "AffineAmvp:%d ", encCfg->m_AffineAmvp);
    }
    msg(VERBOSE, "DMVD:%d ", encCfg->m_useDMVD);
    msg(VERBOSE, "DMVDBIOExt:%d ", encCfg->m_DMVDBIOExt);
    msg(VERBOSE, "OBMC:%d ", encCfg->m_obmc);
    msg(VERBOSE, "MmvdDisNum:%d ", encCfg->m_MmvdDisNum);
    msg(VERBOSE, "JointCbCr:%d ", encCfg->m_jointCbCrMode);
  }
  msg(VERBOSE, "PLT:%d ", encCfg->m_PLTMode);
  msg(VERBOSE, "IBC:%d ", encCfg->m_ibcMode);
  if (encCfg->m_ibcMode)
  {
    msg(VERBOSE, "IBCFrac:%d ", encCfg->m_ibcFracMode);
    msg(VERBOSE, "IBCMerge:%d ", encCfg->m_ibcMerge);
  }
  msg(VERBOSE, "WrapAround:%d ", encCfg->m_wrapAround);
  if (encCfg->m_wrapAround)
  {
    msg(VERBOSE, "WrapAroundOffset:%d ", encCfg->m_wrapAroundOffset);
  }
  // ADD_NEW_TOOL (add some output indicating the usage of tools)
  msg(VERBOSE, "Reshape:%d ", encCfg->m_lmcsEnabled);
  if (encCfg->m_lmcsEnabled)
  {
    msg(VERBOSE, "(Signal:%s ",
        encCfg->m_reshapeSignalType == 0 ? "SDR" : (encCfg->m_reshapeSignalType == 2 ? "HDR-HLG" : "HDR-PQ"));
    msg(VERBOSE, "Opt:%d", encCfg->m_adpOption);
    if (encCfg->m_adpOption > 0)
    {
      msg(VERBOSE, " CW:%d", encCfg->m_initialCW);
    }
    msg(VERBOSE, " CSoffset:%d", encCfg->m_CSoffset);
    msg(VERBOSE, ") ");
  }
  msg(VERBOSE, "MRL:%d ", encCfg->m_MRL);
  msg(VERBOSE, "PDP:%d ", encCfg->m_pdp);
  msg(VERBOSE, "MIP:%d ", encCfg->m_MIP);
  msg(VERBOSE, "LIC:%d ", encCfg->m_licMode);
  msg(VERBOSE, "MergeOppositeLic:%d ", encCfg->m_mergeOppositeLic);
  if (encCfg->m_mergeOppositeLic)
  {
    msg(VERBOSE, "(Reg:%d/Aff:%d) ", encCfg->m_maxNumOppositeLicMergeCand, encCfg->m_maxNumAffineOppositeLicMergeCand);
  }
  msg(VERBOSE, "DirectionalPlanar:%d ", encCfg->m_dirPlanar);
  msg(VERBOSE, "DIMD:%d ", encCfg->m_DIMD);
  msg(VERBOSE, "DIMD chroma:%d ", encCfg->m_DIMDChroma);
  msg(VERBOSE, "TIMD:%d ", encCfg->m_TIMD);
  msg(VERBOSE, "TIMDSAD:%d ", encCfg->m_TIMDSAD);
  msg(VERBOSE, "TIMDMerge:%d ", encCfg->m_TIMD);
  msg(VERBOSE, "OBIC:%d ", encCfg->m_OBIC);
  msg(VERBOSE, "EIP:%d ", encCfg->m_EIP);
  msg(VERBOSE, "MMEIP:%d ", encCfg->m_MMEIP);
  msg(VERBOSE, "AdditionalCMVP:%d ", encCfg->m_bUseAdditionalCMVP);
  msg(VERBOSE, "MaxNumMergeCands(%d, Affine:%d, Geo:%d, IBC:%d, BM:%d)", encCfg->m_maxNumMergeCand,
      encCfg->m_maxNumAffineMergeCand, encCfg->m_maxNumGeoCand, encCfg->m_maxNumIBCMergeCand,
      encCfg->m_maxNumBMMergeCand);

  msg(VERBOSE, "\nFAST TOOL CFG: ");
  msg(VERBOSE, "LCTUFast:%d ", encCfg->m_useFastLCTU);
  msg(VERBOSE, "MaxMergeRdCandNumTotal:%d MergeRdCandQuota(Regular:%d RegularSmallBlk:%d ",
      encCfg->m_maxMergeRdCandNumTotal, encCfg->m_mergeRdCandQuotaRegular, encCfg->m_mergeRdCandQuotaRegularSmallBlk);
  msg(VERBOSE, "SubBlk:%d Ciip:%d Gpm:%d ", encCfg->m_mergeRdCandQuotaSubBlk, encCfg->m_mergeRdCandQuotaCiip,
      encCfg->m_mergeRdCandQuotaGpm);
  msg(VERBOSE, "NonGeoPreserved:%d) ", encCfg->m_mergeRdCandQuotaNonGeoPreserved);
  msg(VERBOSE, "FastMrg:%d ", encCfg->m_useFastMrg);
  msg(VERBOSE, "FEN:%d ", int(encCfg->m_fastInterSearchMode));
  msg(VERBOSE, "ECU:%d ", encCfg->m_bUseEarlyCU);
  msg(VERBOSE, "FDM:%d ", encCfg->m_useFastDecisionForMerge);
  msg(VERBOSE, "ESD:%d ", encCfg->m_useEarlySkipDetection);
  msg(VERBOSE, "PBIntraFast:%d ", encCfg->m_usePbIntraFast);
  if (encCfg->m_uiMaxMTTHierarchyDepthI || encCfg->m_uiMaxMTTHierarchyDepth || encCfg->m_uiMaxMTTHierarchyDepthIChroma)
  {
    msg(VERBOSE, "AMaxBT:%d ", encCfg->m_useAMaxBT);
    msg(VERBOSE, "CostBasedMTTSkipping:%d ", encCfg->m_useCostBasedMttSkipping);
    msg(VERBOSE, "ContentBasedFastQtbt:%d ", encCfg->m_contentBasedFastQtbt);
    msg(VERBOSE, "TTFastSkip:%d ", encCfg->m_ttFastSkip);
    msg(VERBOSE, "TTFastSkipThr:%.3f ", encCfg->m_ttFastSkipThr);
  }
  msg(VERBOSE, "E0023FastEnc:%d ", encCfg->m_e0023FastEnc);
  if (encCfg->m_ImvMode)
  {
    msg(VERBOSE, "IMV4PelFast:%d ", encCfg->m_Imv4PelFast);
  }
  if (encCfg->m_MIP)
  {
    msg(VERBOSE, "FastMIP:%d ", encCfg->m_useFastMIP);
  }
  if (encCfg->m_licMode)
  {
    msg(VERBOSE, "FastPicLevelLIC:%d ", encCfg->m_fastPicLevelLIC);
    msg(VERBOSE, "FastLICmode:%d ", encCfg->m_fastLICMode);
  }
  if (encCfg->m_ibcMode)
  {
    msg(VERBOSE, "IBCFastMethod:%d ", encCfg->m_ibcFastMethod);
    msg(VERBOSE, "HashME:%d ", encCfg->m_HashMECfgEnable);
  }
  if (encCfg->m_Affine && encCfg->m_AffineAmvr)
  {
    msg(VERBOSE, "AffineAmvrEncOpt:%d ", encCfg->m_AffineAmvrEncOpt);
  }
  msg(VERBOSE, "TemporalPartPred:%d ", encCfg->m_tempPartPredEnabled);

  msg(VERBOSE, "\nENC TOOL CFG: ");
  msg(VERBOSE, "RecalQP:%d ", encCfg->m_recalculateQPAccordingToLambda ? 1 : 0);
  msg(VERBOSE, "HAD:%d ", encCfg->m_bUseHADME);
  msg(VERBOSE, "RDQ:%d ", encCfg->m_useRDOQ);
  msg(VERBOSE, "RDQTS:%d ", encCfg->m_useRDOQTS);
#if SHARP_LUMA_DELTA_QP
  msg(VERBOSE, "LQP:%d ", encCfg->m_lumaLevelToDeltaQPMapping.mode);
#endif
  msg(VERBOSE, "SQP:%d ", encCfg->m_uiDeltaQpRD);
  msg(VERBOSE, "ASR:%d ", encCfg->m_bUseASR);
  msg(VERBOSE, "MinSearchWindow:%d ", encCfg->m_minSearchWindow);
  msg(VERBOSE, "RestrictMESampling:%d ", encCfg->m_bRestrictMESampling);
  msg(VERBOSE, "TemporalFilter(%dx%d):%d/%d ", encCfg->m_gopBasedTemporalFilterUnitSize,
      encCfg->m_gopBasedTemporalFilterUnitSize, encCfg->m_gopBasedTemporalFilterPastRefs,
      encCfg->m_gopBasedTemporalFilterFutureRefs);
  msg(VERBOSE, "BIM:%d ", encCfg->m_bimEnabled);
  msg(VERBOSE, "EncDbOpt:%d ", encCfg->m_encDbOpt);
  if (encCfg->m_resChangeInClvsEnabled)
  {
    if (encCfg->m_gopBasedRPREnabledFlag || encCfg->m_rprFunctionalityTestingEnabledFlag)
    {
      msg(VERBOSE, "RPR:(%1.2lfx, %1.2lfx)|%d ", encCfg->m_scalingRatioHor, encCfg->m_scalingRatioVer,
          encCfg->m_rprFunctionalityTestingEnabledFlag ? encCfg->m_rprSwitchingSegmentSize : encCfg->m_gopSize);
      msg(VERBOSE, "RPR2:(%1.2lfx, %1.2lfx)|%d ", encCfg->m_scalingRatioHor2, encCfg->m_scalingRatioVer2,
          encCfg->m_rprFunctionalityTestingEnabledFlag ? encCfg->m_rprSwitchingSegmentSize : encCfg->m_gopSize);
      msg(VERBOSE, "RPR3:(%1.2lfx, %1.2lfx)|%d ", encCfg->m_scalingRatioHor3, encCfg->m_scalingRatioVer3,
          encCfg->m_rprFunctionalityTestingEnabledFlag ? encCfg->m_rprSwitchingSegmentSize : encCfg->m_gopSize);
    }
    else
    {
      msg(VERBOSE, "RPR:(%1.2lfx, %1.2lfx)|%d ", encCfg->m_scalingRatioHor, encCfg->m_scalingRatioVer,
          encCfg->m_switchPocPeriod);
    }
  }
  else
  {
    msg(VERBOSE, "RPR:%d ", 0);
  }
  if (encCfg->m_rplOfDepLayerInSh)
  {
    msg(VERBOSE, "RPLofDepLayerInSH:%d ", encCfg->m_rplOfDepLayerInSh);
  }

  msg(VERBOSE, "\nSEI TOOL CFG: ");
  msg(VERBOSE, "SEI CTI:%d ", encCfg->m_seiCfg.m_ctiSEIEnabled);
  msg(VERBOSE, "SEI FGC:%d ", encCfg->m_seiCfg.m_fgcSEIEnabled);

  msg(VERBOSE, "SEI PO:%d ", encCfg->m_seiCfg.m_poSEIEnabled);

  msg(VERBOSE, "InterRPL:%d ", encCfg->m_interRPL);

#if EXTENSION_360_VIDEO
  encCfg->m_ext360.outputConfigurationSummary();
#endif

  if (encCfg->m_constrainedRaslEncoding)
  {
    msg(VERBOSE, "\n\nWarning: with SEIConstrainedRASL enabled, LMChroma estimation is skipped in RASL frames");
    if (encCfg->m_wrapAround)
    {
      msg(VERBOSE, "\n         and wrap-around motion compensation is disabled in RASL frames");
    }
  }

  msg(VERBOSE, "\n\n");

  msg(NOTICE, "\n");

  fflush(stdout);
}

bool confirmPara(bool bflag, const char *message)
{
  if (!bflag)
  {
    return false;
  }

  msg(ERROR, "Error: %s\n", message);
  return true;
}

//! \}
