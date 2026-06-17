
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

/** \file     TypeDef.h
    \brief    Define macros, basic types, new types and enumerations
*/

#ifndef __TYPEDEF__
#define __TYPEDEF__

#ifndef __COMMONDEF__
#error Include CommonDef.h not TypeDef.h
#endif

#include <array>
#include <vector>
#include <utility>
#include <sstream>
#include <cstddef>
#include <cstring>
#include <assert.h>
#include <cassert>
#include "CommonDef.h"

// clang-format off

#define ENABLE_NNLF                                       1
#define NNLF_USE_FLOAT                                    0

#define ENABLE_POST_CFE_CHANGES                           0

//########### place macros to be removed in next cycle below this line ###############
#define JVET_Z0117_CHROMA_IF                              1 // 6-tap Chroma IF

#define IF_12TAP                                          1 // 12-tap IF

#define JVET_Z0150_MEMORY_USAGE_PRINT                     1 // JVET-Z0150: Print memory usage

//########### place macros to be be kept below this line ###############
#define FIX_FOR_TEMPORARY_COMPILER_ISSUES_ENABLED         1 // Some compilers fail on particular code fragments, remove this when the compiler is fixed (or new version is used)

#define JVET_S0257_DUMP_360SEI_MESSAGE                    1 // Software support of 360 SEI messages

#define JVET_R0164_MEAN_SCALED_SATD                       1 // JVET-R0164: Use a mean scaled version of SATD in encoder decisions

#define JVET_M0497_MATRIX_MULT                            1 // 0: Fast method; 1: Matrix multiplication

#define APPLY_SBT_SL_ON_MTS                               1 // apply save & load fast algorithm on inter MTS when SBT is on

#define REUSE_CU_RESULTS                                  1
#if REUSE_CU_RESULTS
#define REUSE_CU_RESULTS_WITH_MULTIPLE_TUS                1
#define REUSE_CU_RESULTS_MAX_NUM_TUS                      4
#endif

#ifndef JVET_J0090_MEMORY_BANDWITH_MEASURE
#define JVET_J0090_MEMORY_BANDWITH_MEASURE                0
#endif

#ifndef EXTENSION_360_VIDEO
#define EXTENSION_360_VIDEO                               0   ///< extension for 360/spherical video coding support; this macro should be controlled by makefile, as it would be used to control whether the library is built and linked
#endif

#ifndef EXTENSION_HDRTOOLS
#define EXTENSION_HDRTOOLS                                0 //< extension for HDRTools/Metrics support; this macro should be controlled by makefile, as it would be used to control whether the library is built and linked
#endif

#define JVET_O0756_CONFIG_HDRMETRICS                      1
#if EXTENSION_HDRTOOLS
#define JVET_O0756_CALCULATE_HDRMETRICS                   1
#endif

#define JVET_Z0120_SII_SEI_PROCESSING                     1 // This is an example illustration of using SII SEI messages for backwards-compatible HFR video
#if JVET_Z0120_SII_SEI_PROCESSING
#define DISABLE_PRE_POST_FILTER_FOR_IDR_CRA               1
#define ENABLE_USER_DEFINED_WEIGHTS                       0 // User can specify weights for both current and previous picture, such that their sum = 1
#endif

// clang-format on

// ====================================================================================================================
// General settings
// ====================================================================================================================

#ifndef ENABLE_TRACING
#define ENABLE_TRACING \
  0 // DISABLE by default (enable only when debugging, requires 15% run-time in decoding) -- see documentation in
                                                            // 'doc/DTrace for NextSoftware.pdf'
#endif

#ifndef ENABLE_CABAC_DUMP
#define ENABLE_CABAC_DUMP 0   // JVET-AG0196 5.1 bin dumper
#endif

#if ENABLE_TRACING
#define K0149_BLOCK_STATISTICS \
  0 // enables block statistics, which can be analysed with YUView (https://github.com/IENT/YUView)
#if K0149_BLOCK_STATISTICS
#define BLOCK_STATS_AS_CSV \
  0 // statistics will be written in a comma separated value format. this is not supported by YUView
#endif
#endif

#ifndef ENABLE_TIME_PROFILING
#define ENABLE_TIME_PROFILING 0 // DISABLED by default
#endif

#define WCG_EXT   1
#define WCG_WPSNR WCG_EXT

#define KEEP_PRED_AND_RESI_SIGNALS 0

#ifndef ENABLE_VALGRIND_CODE
#define ENABLE_VALGRIND_CODE 0 // DISABLED by default (can be enabled by project configuration or make command)
#endif

#if ENABLE_VALGRIND_CODE
#define VALGRIND_MEMCLEAR(_ref, _size) memset(_ref, 0, (_size))
#else
#define VALGRIND_MEMCLEAR(_ref, _size)
#endif

// ====================================================================================================================
// Debugging
// ====================================================================================================================

// most debugging tools are now bundled within the ENABLE_TRACING macro -- see documentation to see how to use

#define PRINT_MACRO_VALUES \
  1 ///< When enabled, the encoder prints out a list of the non-environment-variable controlled macros and their
                                                            ///< values on startup

// ====================================================================================================================
// Tool Switches - transitory (these macros are likely to be removed in future revisions)
// ====================================================================================================================

#define DECODER_CHECK_SUBSTREAM_AND_SLICE_TRAILING_BYTES \
  1 ///< TODO: integrate this macro into a broader conformance checking system.
      // To use this capability enable config parameter LambdaFromQpEnable

// ====================================================================================================================
// Tool Switches
// ====================================================================================================================
//
// SIMD optimizations
#define SIMD_ENABLE 1   ///< Enable SIMD optimizations if available on compilation environment
#ifdef TARGET_SIMD_X86
#define ENABLE_SIMD_OPT SIMD_ENABLE   ///< SIMD optimizations, no impact on RD performance
#else
#define ENABLE_SIMD_OPT 0   ///< SIMD optimizations, no impact on RD performance
#endif
#define ENABLE_SIMD_OPT_MCIF \
  (1 && ENABLE_SIMD_OPT)   ///< SIMD optimization for the interpolation filter, no impact on RD performance
#define ENABLE_SIMD_OPT_BUFFER \
  (1 && ENABLE_SIMD_OPT)   ///< SIMD optimization for the buffer operations, no impact on RD performance
#define ENABLE_SIMD_OPT_DIST \
  (1 && ENABLE_SIMD_OPT)   ///< SIMD optimization for the distortion calculations(SAD,SSE,HADAMARD), no impact on RD
                           ///< performance
#define ENABLE_SIMD_OPT_AFFINE_ME \
  (1 && ENABLE_SIMD_OPT)   ///< SIMD optimization for affine ME, no impact on RD performance
#define ENABLE_SIMD_BILATERAL_FILTER (1 && ENABLE_SIMD_OPT)   ///< SIMD optimization for BIF
#define ENABLE_SIMD_OPT_INTRAPRED    (1 && ENABLE_SIMD_OPT)   ///< SIMD optimization for INTRAPRED
#define ENABLE_SIMD_OPT_ALF          (1 && ENABLE_SIMD_OPT)   ///< SIMD optimization for ALF
#define ENABLE_SIMD_TRAFO            (1 && ENABLE_SIMD_OPT)   ///< SIMD optimization for Transformation
#define ENABLE_SIMD_OPT_QUANT        (1 && ENABLE_SIMD_OPT)   ///< SIMD optimization for Quantization
#define ENABLE_SIMD_OPT_CCSAO        (1 && ENABLE_SIMD_OPT)   ///< SIMD optimization for CCSAO
#if ENABLE_SIMD_OPT_BUFFER
#define ENABLE_SIMD_OPT_BCW 1   ///< SIMD optimization for Bcw
#endif
#if IF_12TAP
#define IF_12TAP_SIMD 1   // Enable SIMD for 12-tap filter
#if IF_12TAP_SIMD
#define SIMD_4x4_12 1   // Enable 4x4-block combined passes for 12-tap filters
#endif
#endif

// End of SIMD optimizations

#if defined(TARGET_SIMD_X86)
#define SIMD_EVERYWHERE_EXTENSION_LEVEL SSE42
#endif

#define RDOQ_CHROMA_LAMBDA 1   ///< F386: weighting of chroma for RDOQ

#define W0038_CQP_ADJ 1   ///< chroma QP adjustment based on TL, CQPTLAdjustEnabled is set to 1;

#define SHARP_LUMA_DELTA_QP  1   ///< include non-normative LCU deltaQP and normative chromaQP change
#define ER_CHROMA_QP_WCG_PPS 1   ///< Chroma QP model for WCG used in Anchor 3.2
#define ENABLE_QPA \
  1   ///< Non-normative perceptual QP adaptation according to JVET-H0047 and JVET-K0206. Deactivated by default,
      ///< activated using encoder arguments --PerceptQPA=1 --SliceChromaQPOffsetPeriodicity=1
#define ENABLE_QPA_SUB_CTU (1 && ENABLE_QPA)   ///< when maximum delta-QP depth is greater than zero, use sub-CTU QPA

#define RDOQ_CHROMA 1   ///< use of RDOQ in chroma

#define FULL_NBIT 1

// ====================================================================================================================
// BILATERAL_FILTER
// ====================================================================================================================

struct BifParams
{
  static constexpr int BIF_QP    = 17;
  int                  frmOn     = 0;   // slice_bif_enabled_flag
  int                  allCtuOn  = 0;   // slice_bif_all_ctb_enabled_flag
  int                  numBlocks = 0;
  std::vector<int>     ctuOn;   // bif_ctb_flag[][]
  void                 initCTUflag(int num_Blocks, int frmOn, int allCtuOn)
  {
    numBlocks = num_Blocks;
    ctuOn.resize(numBlocks);
    if (!frmOn)
    {
      std::fill(ctuOn.begin(), ctuOn.end(), 0);
    }
    else if (allCtuOn)
    {
      std::fill(ctuOn.begin(), ctuOn.end(), 1);
    }
  }
};

// ====================================================================================================================
// Derived macros
// ====================================================================================================================

#define DISTORTION_PRECISION_ADJUSTMENT(x) 0

// ====================================================================================================================
// Error checks
// ====================================================================================================================

// ====================================================================================================================
// Named numerical types
// ====================================================================================================================

typedef int16_t  Pel;   ///< pixel type
typedef int      TCoeff;   ///< transform coefficient
typedef int16_t  TMatrixCoeff;   ///< transform matrix coefficient
typedef int16_t  TFilterCoeff;   ///< filter coefficient
typedef int      Intermediate_Int;   ///< used as intermediate value in calculations
typedef uint32_t Intermediate_UInt;   ///< used as intermediate value in calculations

typedef uint64_t SplitSeries;   ///< used to encoded the splits that caused a particular CU size

typedef uint64_t Distortion;   ///< distortion measurement

// ====================================================================================================================
// Enumeration
// ====================================================================================================================

// casts enum to underlying integer type
template<typename E> constexpr typename std::underlying_type<E>::type to_underlying(E e) noexcept
{
  return static_cast<typename std::underlying_type<E>::type>(e);
}

// array indexed by an enum type
template<class T, class E> struct EnumArray : public std::array<T, to_underlying(E::NUM)>
{
  using base            = std::array<T, to_underlying(E::NUM)>;
  using size_type       = E;
  using reference       = T &;
  using const_reference = const T &;

public:
  constexpr EnumArray() : base() {}
  constexpr EnumArray(std::initializer_list<T> l)
  {
    size_t j = 0;
    for (const auto &i: l)
    {
      base::at(j++) = i;
    }
  }

  reference                 operator[](size_type e) { return base::operator[](to_underlying(e)); }
  constexpr const_reference operator[](size_type e) const { return base::operator[](to_underlying(e)); }
};

enum NeighAreaType
{
  NEIGH_AREA_TOPLEFT = 0,
  NEIGH_AREA_LEFT,
  NEIGH_AREA_TOP,
};

enum ConvModelType
{
  CONV_MODEL_CCLM = 0,
  CONV_MODEL_CCCM,

  CONV_MODEL_CCCM_INTRA_FIRST = CONV_MODEL_CCCM,   // Used to loop over intra CCCM modes
  CONV_MODEL_CCCM_GRADLOC,
  CONV_MODEL_CCCM_MULTIF_1,
  CONV_MODEL_CCCM_MULTIF_2,
  CONV_MODEL_CCCM_MULTIF_3,
  CONV_MODEL_CCCM_BVG,
  CONV_MODEL_CCCM_NOSUBS,
  CONV_MODEL_CCCM_INTRA_LAST = CONV_MODEL_CCCM_NOSUBS,   // Used to loop over intra CCCM modes

  CONV_MODEL_EIP_FIRST,   // used to compute EIP model index
  CONV_MODEL_EIP_S = CONV_MODEL_EIP_FIRST,
  CONV_MODEL_EIP_V,
  CONV_MODEL_EIP_H,

  // Types to be added:
  /*
  CONV_MODEL_CCCM_INTER,
  */

  CONV_MODEL_UNDEFINED
};

enum IbcBvStatus
{
  IBC_BV_INVALID,
  IBC_BV_VALID,
  IBC_INT_BV_VALID
};

enum class ApsType : uint8_t
{
  ALF = 0,
  LMCS,
  SCALING_LIST,
  NUM
};

enum QuantFlags
{
  Q_INIT           = 0x0,
  Q_USE_RDOQ       = 0x1,
  Q_RDOQTS         = 0x2,
  Q_SELECTIVE_RDOQ = 0x4,
};

enum class TransType
{
  DCT2 = 0,
  DCT8,
  DST7,
  DCT5,
  DST4,
  DST1,
  IDTR,
  KLT0,
  KLT1,
  NUM,
};

enum class MtsType : int8_t
{
  NONE      = -1,
  DCT2_DCT2 = 0,
  DCT2_NST1,
  DCT2_NST2,
  DCT2_NST3,
  SKIP,
  MTS_1,
  MTS_2,
  MTS_3,
  MTS_4,
  MTS_5,
  MTS_6,
  END,
  NUM = END
};

static inline constexpr bool isNST(const MtsType &t) { return (t > MtsType::DCT2_DCT2 && t < MtsType::SKIP); }

static inline constexpr bool isMTS(const MtsType &t) { return (t > MtsType::SKIP); }

static inline constexpr int operator-(const MtsType &a, const MtsType &b)
{
  return to_underlying(a) - to_underlying(b);
}

static inline constexpr MtsType operator+(const MtsType &a, int b)
{
  return static_cast<MtsType>(to_underlying(a) + b);
}

static inline MtsType operator++(MtsType &a, int)
{
  MtsType b = a;

  a = static_cast<MtsType>(to_underlying(a) + 1);

  return b;
}

enum class PlanarDirType : int8_t
{
  NO_DIR = 0,
  HOR    = 1,
  VER    = 2
};
static inline bool isDirPlanar(const PlanarDirType &plDir) { return plDir != PlanarDirType::NO_DIR; }

enum TemplateType
{
  NO_NEIGHBOR         = 0,
  LEFT_NEIGHBOR       = 1,
  ABOVE_NEIGHBOR      = 2,
  LEFT_ABOVE_NEIGHBOR = 3
};

enum SbtIdx
{
  SBT_OFF_DCT  = 0,
  SBT_VER_HALF = 1,
  SBT_HOR_HALF = 2,
  SBT_VER_QUAD = 3,
  SBT_HOR_QUAD = 4,
  SBT_QUAD     = 5,
  SBT_QUARTER  = 6,
  NUMBER_SBT_IDX,
  SBT_OFF_MTS,   // note: must be after all SBT modes, only used in fast algorithm to discern the best mode is inter EMT
};

enum SbtPos
{
  SBT_POS0 = 0,
  SBT_POS1 = 1,
  NUMBER_SBT_POS
};

enum SbtMode
{
  SBT_VER_H0 = 0,
  SBT_VER_H1 = 1,
  SBT_HOR_H0 = 2,
  SBT_HOR_H1 = 3,
  SBT_VER_Q0 = 4,
  SBT_VER_Q1 = 5,
  SBT_HOR_Q0 = 6,
  SBT_HOR_Q1 = 7,
  SBT_Q0     = 8,
  SBT_Q1     = 9,
  SBT_Q2     = 10,
  SBT_Q3     = 11,
  SBT_QT0    = 12,
  SBT_QT1    = 13,
  SBT_QT2    = 14,
  SBT_QT3    = 15,
  NUMBER_SBT_MODE
};

enum class BdpcmMode : uint8_t
{
  NONE = 0,
  HOR,
  VER
};

/// supported slice type
enum SliceType
{
  B_SLICE               = 0,
  P_SLICE               = 1,
  I_SLICE               = 2,
  L_SLICE               = 3,
  NUMBER_OF_SLICE_TYPES = 4
};

// chroma formats (according to how the monochrome or the color planes are intended to be coded)
enum class ChromaFormat : uint8_t
{
  _400 = 0,
  _420,
  _422,
  _444,
  NUM,
  UNDEFINED = NUM
};

enum class ChannelType : uint8_t
{
  LUMA   = 0,
  CHROMA = 1,
  NUM
};

static inline ChannelType operator++(ChannelType &ch, int)
{
  ChannelType r = ch;

  ch = static_cast<ChannelType>(static_cast<int>(ch) + 1);
  return r;
}

static constexpr auto MAX_NUM_CHANNEL_TYPE = to_underlying(ChannelType::NUM);

enum CompID
{
  COMP_Y            = 0,
  MAX_NUM_LUMA_COMP = 1,
  COMP_Cb           = 1,
  COMP_Cr           = 2,
  MAX_NUM_COMP      = 3,
  JOINT_CbCr        = MAX_NUM_COMP,
  MAX_NUM_TBLOCKS   = MAX_NUM_COMP
};

#define MAP_CHROMA(c) (CompID(c))

struct CclmModelSingle
{
  int a       = 0;
  int b       = 0;
  int shift   = 0;
  int midLuma = 0;

  CclmModelSingle() {}
  CclmModelSingle(int _a, int _b, int _shift)
  {
    a     = _a;
    b     = _b;
    shift = _shift;
  }
};

struct CclmModel
{
  CclmModelSingle model[2];

  int multiModelLumaThr = 0;

  void setFirstModel(int a, int b, int shift)
  {
    model[0].a     = a;
    model[0].b     = b;
    model[0].shift = shift;
  }
  void setSecondModel(int a, int b, int shift, int thr)
  {
    model[1].a        = a;
    model[1].b        = b;
    model[1].shift    = shift;
    multiModelLumaThr = thr;
  }
};

struct CclmOffsets
{
  int8_t cb0 = 0;
  int8_t cr0 = 0;
  int8_t cb1 = 0;
  int8_t cr1 = 0;

  bool isActive() const { return cb0 || cr0 || cb1 || cr1; }
  void setAllZero()
  {
    cb0 = 0;
    cr0 = 0;
    cb1 = 0;
    cr1 = 0;
  }
  void setOffsets(int b0, int r0, int b1, int r1)
  {
    cb0 = b0;
    cr0 = r0;
    cb1 = b1;
    cr1 = r1;
  }
  void setOffset(CompID c, int model, int v)
  {
    if (c == COMP_Cb)
    {
      if (model == 0)
      {
        cb0 = v;
      }
      else
      {
        cb1 = v;
      }
    }
    else
    {
      if (model == 0)
      {
        cr0 = v;
      }
      else
      {
        cr1 = v;
      }
    }
  }
};

enum InputColourSpaceConversion   // defined in terms of conversion prior to input of encoder.
{
  IPCOLOURSPACE_UNCHANGED               = 0,
  IPCOLOURSPACE_YCbCrtoYCrCb            = 1,   // Mainly used for debug!
  IPCOLOURSPACE_YCbCrtoYYY              = 2,   // Mainly used for debug!
  IPCOLOURSPACE_RGBtoGBR                = 3,
  NUMBER_INPUT_COLOUR_SPACE_CONVERSIONS = 4
};

enum MATRIX_COEFFICIENTS   // Table E.5 (Matrix coefficients)
{
  MATRIX_COEFFICIENTS_RGB                           = 0,
  MATRIX_COEFFICIENTS_BT709                         = 1,
  MATRIX_COEFFICIENTS_UNSPECIFIED                   = 2,
  MATRIX_COEFFICIENTS_RESERVED_BY_ITUISOIEC         = 3,
  MATRIX_COEFFICIENTS_USFCCT47                      = 4,
  MATRIX_COEFFICIENTS_BT601_625                     = 5,
  MATRIX_COEFFICIENTS_BT601_525                     = 6,
  MATRIX_COEFFICIENTS_SMPTE240                      = 7,
  MATRIX_COEFFICIENTS_YCGCO                         = 8,
  MATRIX_COEFFICIENTS_BT2020_NON_CONSTANT_LUMINANCE = 9,
  MATRIX_COEFFICIENTS_BT2020_CONSTANT_LUMINANCE     = 10,
};

/// supported prediction type
enum PredMode
{
  MODE_INTER                 = 0,   ///< inter-prediction mode
  MODE_INTRA                 = 1,   ///< intra-prediction mode
  MODE_IBC                   = 2,   ///< ibc-prediction mode
  MODE_PLT                   = 3,   ///< plt-prediction mode
  NUMBER_OF_PREDICTION_MODES = 4,
};

/// reference list index
enum RefPicList
{
  RPL0      = 0,   ///< reference list 0
  RPL1      = 1,   ///< reference list 1
  NUM_RPL01 = 2,
  RPLX      = 100   ///< special mark
};

enum GeoTmMvCand
{
  GEO_TM_OFF = 0,
  GEO_TM_SHAPE_AL,
  GEO_TM_SHAPE_A,
  GEO_TM_SHAPE_L,
  GEO_NUM_TM_MV_CAND
};

/// distortion function index
enum class DFunc
{
  // SSE functions by size
  SSE = 0,
  SSE2,
  SSE4,
  SSE8,
  SSE16,
  SSE32,
  SSE64,
  SSE16N,

  // SAD functions by size
  SAD,
  SAD2,
  SAD4,
  SAD8,
  SAD16,
  SAD32,
  SAD64,
  SAD16N,

  // HAD functions by size
  HAD,
  HAD2,
  HAD4,
  HAD8,
  HAD16,
  HAD32,
  HAD64,
  HAD16N,

  HAD_fast,
  HAD2_fast,
  HAD4_fast,
  HAD8_fast,
  HAD16_fast,
  HAD32_fast,
  HAD64_fast,
  HAD16N_fast,

  // SAD functions by size (odd sizes)
  SAD12,
  SAD24,
  SAD48,

  // Mean-removed versions
  // NOTE: order must be the same as the regular versions above

  MRSAD,
  MRSAD2,
  MRSAD4,
  MRSAD8,
  MRSAD16,
  MRSAD32,
  MRSAD64,
  MRSAD16N,

  MRHAD,
  MRHAD2,
  MRHAD4,
  MRHAD8,
  MRHAD16,
  MRHAD32,
  MRHAD64,
  MRHAD16N,

  MRHAD_fast,
  MRHAD2_fast,
  MRHAD4_fast,
  MRHAD8_fast,
  MRHAD16_fast,
  MRHAD32_fast,
  MRHAD64_fast,
  MRHAD16N_fast,

  MRSAD12,
  MRSAD24,
  MRSAD48,

  // Full precision versions of SAD functions
  SAD_FULL_NBIT,
  SAD_FULL_NBIT2,
  SAD_FULL_NBIT4,
  SAD_FULL_NBIT8,
  SAD_FULL_NBIT16,
  SAD_FULL_NBIT32,
  SAD_FULL_NBIT64,
  SAD_FULL_NBIT16N,

  // Weighted SSE functions by size
  SSE_WTD,
  SSE2_WTD,
  SSE4_WTD,
  SSE8_WTD,
  SSE16_WTD,
  SSE32_WTD,
  SSE64_WTD,
  SSE16N_WTD,

  SAD_INTERMEDIATE_BITDEPTH,

  SAD_WITH_MASK,

  NUM
};

enum class DFuncDiff;

static inline DFuncDiff operator-(const DFunc &a, const DFunc &b)
{
  return static_cast<DFuncDiff>(to_underlying(a) - to_underlying(b));
}
static inline DFunc operator+(const DFunc &a, const DFuncDiff &b)
{
  return static_cast<DFunc>(to_underlying(a) + to_underlying(b));
}
static inline DFunc operator+(const DFunc &a, const DFunc &b)
{
  return static_cast<DFunc>(to_underlying(a) + to_underlying(b));
}

/// motion vector predictor direction used in AMVP
enum MvpDir
{
  MD_LEFT = 0,   ///< MVP of left block
  MD_ABOVE,   ///< MVP of above block
  MD_ABOVE_RIGHT,   ///< MVP of above right block
  MD_BELOW_LEFT,   ///< MVP of below left block
  MD_ABOVE_LEFT   ///< MVP of above left block
};

enum TransformDirection
{
  TRANSFORM_FORWARD              = 0,
  TRANSFORM_INVERSE              = 1,
  TRANSFORM_NUMBER_OF_DIRECTIONS = 2
};

/// supported ME search methods
enum class MESearchMethod : int
{
  FULL = 0,
  DIAMOND,
  SELECTIVE,
  DIAMOND_ENHANCED,
  NUM
};

/// coefficient scanning type used in ACS
enum class CoeffScanType
{
  DIAG     = 0,
  TRAV_HOR = 1,
  TRAV_VER = 2,
  NUM
};

static inline CoeffScanType operator++(CoeffScanType &a, int)
{
  CoeffScanType b = a;
  a               = static_cast<CoeffScanType>(to_underlying(a) + 1);
  return b;
}

enum CoeffScanGroupType
{
  SCAN_UNGROUPED             = 0,
  SCAN_GROUPED_4x4           = 1,
  SCAN_NUMBER_OF_GROUP_TYPES = 2
};

enum ScalingListMode
{
  SCALING_LIST_OFF,
  SCALING_LIST_DEFAULT,
  SCALING_LIST_FILE_READ
};

enum ScalingListSize
{
  SCALING_LIST_1x1 = 0,
  SCALING_LIST_2x2,
  SCALING_LIST_4x4,
  SCALING_LIST_8x8,
  SCALING_LIST_16x16,
  SCALING_LIST_32x32,
  SCALING_LIST_64x64,
  SCALING_LIST_128x128,
  SCALING_LIST_SIZE_NUM,
  // for user define matrix
  SCALING_LIST_FIRST_CODED = SCALING_LIST_2x2,
  SCALING_LIST_LAST_CODED  = SCALING_LIST_64x64
};

enum ScalingList1dStartIdx
{
  SCALING_LIST_1D_START_2x2   = 0,
  SCALING_LIST_1D_START_4x4   = 2,
  SCALING_LIST_1D_START_8x8   = 8,
  SCALING_LIST_1D_START_16x16 = 14,
  SCALING_LIST_1D_START_32x32 = 20,
  SCALING_LIST_1D_START_64x64 = 26,
};

// For use with decoded picture hash SEI messages, generated by encoder.
enum class HashType
{
  NONE     = -1,
  MD5      = 0,
  CRC      = 1,
  CHECKSUM = 2,
  NUM
};

enum class SAOMode : uint8_t
{
  OFF = 0,
  NEW,
  MERGE,
  NUM
};

enum class SAOModeMergeTypes : int8_t
{
  NONE = -1,
  LEFT = 0,
  ABOVE,
  NUM
};
enum class SAOModeNewTypes : int8_t
{
  NONE     = -1,
  START_EO = 0,
  EO_0     = START_EO,
  EO_90,
  EO_135,
  EO_45,

  START_BO,
  BO = START_BO,

  NUM
};

static constexpr int NUM_SAO_EO_TYPES_LOG2 = 2;

enum SAOEOClasses
{
  SAO_CLASS_EO_FULL_VALLEY = 0,
  SAO_CLASS_EO_HALF_VALLEY = 1,
  SAO_CLASS_EO_PLAIN       = 2,
  SAO_CLASS_EO_HALF_PEAK   = 3,
  SAO_CLASS_EO_FULL_PEAK   = 4,
  NUM_SAO_EO_CLASSES,
};
enum CCSAOSetTypes
{
  CCSAO_SET_TYPE_BAND = 0,
  CCSAO_SET_TYPE_EDGE = 1,
  NUM_CCSAO_SET_TYPES = 2
};
enum NNPC_PaddingType
{
  ZERO_PADDING        = 0,
  REPLICATION_PADDING = 1,
  REFLECTION_PADDING  = 2,
  WRAP_AROUND_PADDING = 3,
  FIXED_PADDING       = 4
};

enum NNPC_PurposeType
{
  UNKONWN                    = 0,
  VISUAL_QUALITY_IMPROVEMENT = 1,
  CHROMA_UPSAMPLING          = 2,
  RESOLUTION_UPSAMPLING      = 4,
  FRAME_RATE_UPSAMPLING      = 8,
  BIT_DEPTH_UPSAMPLING       = 16,
  COLOURIZATION              = 32
};

enum POST_FILTER_MODE
{
  ISO_IEC_15938_17 = 0,
  URI              = 1
};

#define NUM_SAO_BO_CLASSES_LOG2 5
#define NUM_SAO_BO_CLASSES      (1 << NUM_SAO_BO_CLASSES_LOG2)

namespace Profile
{
enum Name
{
  NONE                                 = 0,
  INTRA                                = 8,
  STILL_PICTURE                        = 64,
  MAIN_10                              = 1,
  MAIN_10_STILL_PICTURE                = MAIN_10 | STILL_PICTURE,
  MULTILAYER_MAIN_10                   = 17,
  MULTILAYER_MAIN_10_STILL_PICTURE     = MULTILAYER_MAIN_10 | STILL_PICTURE,
  MAIN_10_444                          = 33,
  MAIN_10_444_STILL_PICTURE            = MAIN_10_444 | STILL_PICTURE,
  MULTILAYER_MAIN_10_444               = 49,
  MULTILAYER_MAIN_10_444_STILL_PICTURE = MULTILAYER_MAIN_10_444 | STILL_PICTURE,
  MAIN_12                              = 2,
  MAIN_12_444                          = 34,
  MAIN_16_444                          = 35,
  MAIN_12_INTRA                        = MAIN_12 | INTRA,
  MAIN_12_444_INTRA                    = MAIN_12_444 | INTRA,
  MAIN_16_444_INTRA                    = MAIN_16_444 | INTRA,
  MAIN_12_STILL_PICTURE                = MAIN_12 | STILL_PICTURE,
  MAIN_12_444_STILL_PICTURE            = MAIN_12_444 | STILL_PICTURE,
  MAIN_16_444_STILL_PICTURE            = MAIN_16_444 | STILL_PICTURE,
};
}   // namespace Profile

namespace Level
{
enum Tier
{
  MAIN            = 0,
  HIGH            = 1,
  NUMBER_OF_TIERS = 2
};

enum Name
{
  // code = (major_level * 16 + minor_level * 3)
  NONE      = 0,
  LEVEL1    = 16,
  LEVEL2    = 32,
  LEVEL2_1  = 35,
  LEVEL3    = 48,
  LEVEL3_1  = 51,
  LEVEL4    = 64,
  LEVEL4_1  = 67,
  LEVEL5    = 80,
  LEVEL5_1  = 83,
  LEVEL5_2  = 86,
  LEVEL6    = 96,
  LEVEL6_1  = 99,
  LEVEL6_2  = 102,
  LEVEL6_3  = 105,
  LEVEL15_5 = 255,
};
}   // namespace Level

enum CostMode
{
  COST_STANDARD_LOSSY              = 0,
  COST_SEQUENCE_LEVEL_LOSSLESS     = 1,
  COST_LOSSLESS_CODING             = 2,
  COST_MIXED_LOSSLESS_LOSSY_CODING = 3
};

enum WeightedPredictionMethod
{
  WP_PER_PICTURE_WITH_SIMPLE_DC_COMBINED_COMPONENT                      = 0,
  WP_PER_PICTURE_WITH_SIMPLE_DC_PER_COMPONENT                           = 1,
  WP_PER_PICTURE_WITH_HISTOGRAM_AND_PER_COMPONENT                       = 2,
  WP_PER_PICTURE_WITH_HISTOGRAM_AND_PER_COMP_AND_CLIPPING               = 3,
  WP_PER_PICTURE_WITH_HISTOGRAM_AND_PER_COMP_AND_CLIPPING_AND_EXTENSION = 4
};

enum FastInterSearchMode
{
  FASTINTERSEARCH_DISABLED = 0,
  FASTINTERSEARCH_MODE1    = 1,   // TODO: assign better names to these.
  FASTINTERSEARCH_MODE2    = 2,
  FASTINTERSEARCH_MODE3    = 3
};

enum SPSExtensionFlagIndex
{
  SPS_EXT__REXT           = 0,
  NUM_SPS_EXTENSION_FLAGS = 8
};

// TODO: Existing names used for the different NAL unit types can be altered to better reflect the names in the spec.
//       However, the names in the spec are not yet stable at this point. Once the names are stable, a cleanup
//       effort can be done without use of macros to alter the names used to indicate the different NAL unit types.
enum NalUnitType
{
  // 1 byte non-VCL
  NAL_UNIT_PPS = 0,   // 0
  NAL_UNIT_PH,   // 1
  NAL_UNIT_PREFIX_APS,   // 2
  NAL_UNIT_SUFFIX_APS,   // 3

  // 2 byte non-VLC
  NAL_UNIT_ACCESS_UNIT_DELIMITER,   // 4
  NAL_UNIT_PREFIX_SEI,   // 5
  NAL_UNIT_SUFFIX_SEI,   // 6
  NAL_UNIT_FD,   // 7

  // 1 byte VCL
  NAL_UNIT_CODED_SLICE_TRAIL,   // 8
  NAL_UNIT_CODED_SLICE_STSA,   // 9
  NAL_UNIT_CODED_SLICE_RADL,   // 10
  NAL_UNIT_CODED_SLICE_RASL,   // 11

  // 2 byte VCL
  NAL_UNIT_RESERVED_VCL_12,
  NAL_UNIT_RESERVED_VCL_13,
  NAL_UNIT_RESERVED_IRAP_VCL_14,
  NAL_UNIT_RESERVED_IRAP_VCL_15,

  // 1 byte non-VCL with tID = 0
  NAL_UNIT_DCI,   // 16
  NAL_UNIT_SPS,   // 17
  NAL_UNIT_EOS,   // 18
  NAL_UNIT_EOB,   // 19

  // 2 byte non-VLC with tID = 0
  NAL_UNIT_VPS,   // 20
  NAL_UNIT_OPI,   // 21
  NAL_UNIT_RESERVED_NVCL_22,   // 22
  NAL_UNIT_RESERVED_NVCL_23,   // 23

  // 1 byte VCL with tID = 0
  NAL_UNIT_CODED_SLICE_IDR_W_RADL,   // 24
  NAL_UNIT_CODED_SLICE_IDR_N_LP,   // 25
  NAL_UNIT_CODED_SLICE_CRA,   // 26
  NAL_UNIT_CODED_SLICE_GDR,   // 27

  // 2 byte VCL with tID = 0
  NAL_UNIT_UNSPECIFIED_28,
  NAL_UNIT_UNSPECIFIED_29,
  NAL_UNIT_UNSPECIFIED_30,
  NAL_UNIT_UNSPECIFIED_31,
  NAL_UNIT_INVALID
};

#if SHARP_LUMA_DELTA_QP
enum LumaLevelToDQPMode
{
  LUMALVL_TO_DQP_DISABLED   = 0,
  LUMALVL_TO_DQP_AVG_METHOD = 1,   // use average of CTU to determine luma level
  LUMALVL_TO_DQP_NUM_MODES  = 2
};
#endif

enum class MergeType : uint8_t
{
  DEFAULT_N = 0,
  SUBPU_ATMVP,
  IBC,
  NUM
};

static constexpr int MAX_NUM_APS(ApsType t)
{
  switch (t)
  {
  case ApsType::ALF:
  case ApsType::SCALING_LIST:
    return 8;
  case ApsType::LMCS:
    return 4;
  default:
    return 32;
  }
}

//////////////////////////////////////////////////////////////////////////
// Encoder modes to try out
//////////////////////////////////////////////////////////////////////////

enum ImvMode
{
  IMV_OFF = 0,
  IMV_FPEL,
  IMV_4PEL,
  IMV_HPEL,
  NUM_IMV_MODES
};

// ====================================================================================================================
// Type definition
// ====================================================================================================================

/// parameters for adaptive loop filter

#define MAX_NUM_SAO_CLASSES 32   //(NUM_SAO_EO_GROUPS > NUM_SAO_BO_GROUPS)?NUM_SAO_EO_GROUPS:NUM_SAO_BO_GROUPS

struct SAOOffset
{
  SAOMode modeIdc;   // NEW, MERGE, OFF
  union
  {
    SAOModeMergeTypes mergeType;
    SAOModeNewTypes   newType;
  } typeIdc;
  int typeAuxInfo;   // BO: starting band index
  int offset[MAX_NUM_SAO_CLASSES];

  SAOOffset();
  ~SAOOffset();
  void reset();

  const SAOOffset &operator=(const SAOOffset &src);
};

struct SAOBlkParam
{

  SAOBlkParam();
  ~SAOBlkParam();
  void               reset();
  const SAOBlkParam &operator=(const SAOBlkParam &src);
  SAOOffset         &operator[](int compIdx) { return offsetParam[compIdx]; }
  const SAOOffset   &operator[](int compIdx) const { return offsetParam[compIdx]; }

private:
  SAOOffset offsetParam[MAX_NUM_COMP];
};

using BitDepths = EnumArray<int, ChannelType>;

struct ScalingRatio
{
  static constexpr int BITS = 14;

  int x;
  int y;

  bool operator==(const ScalingRatio &s) const { return x == s.x && y == s.y; }
  bool operator!=(const ScalingRatio &s) const { return x != s.x || y != s.y; }
};

enum PLTRunMode
{
  PLT_RUN_INDEX = 0,
  PLT_RUN_COPY  = 1,
  NUM_PLT_RUN   = 2
};

struct LutModel
{
  bool             presentFlag  = false;
  int              numLutValues = 0;
  std::vector<Pel> lutValues;
};

struct PictureHash
{
  std::vector<uint8_t> hash;

  bool operator==(const PictureHash &other) const
  {
    if (other.hash.size() != hash.size())
    {
      return false;
    }
    for (uint32_t i = 0; i < uint32_t(hash.size()); i++)
    {
      if (other.hash[i] != hash[i])
      {
        return false;
      }
    }
    return true;
  }

  bool operator!=(const PictureHash &other) const { return !(*this == other); }
};

struct SEITimeSet
{
  bool clockTimeStampFlag { false };
  bool numUnitFieldBasedFlag { false };
  int  countingType { 0 };
  bool fullTimeStampFlag { false };
  bool discontinuityFlag { false };
  bool cntDroppedFlag { false };
  int  numberOfFrames { 0 };
  int  secondsValue { 0 };
  int  minutesValue { 0 };
  int  hoursValue { 0 };
  bool secondsFlag { false };
  bool minutesFlag { false };
  bool hoursFlag { false };
  int  timeOffsetLength { 0 };
  int  timeOffsetValue { 0 };
};

struct SEIMasteringDisplay
{
  bool     colourVolumeSEIEnabled { false };
  uint32_t maxLuminance { 10000 };
  uint32_t minLuminance { 0 };
  uint16_t primaries[3][2] {};
  uint16_t whitePoint[2] {};
};

struct SEIQualityMetrics
{
  double psnr { 0 };
  double ssim { 0 };
  double wpsnr { 0 };
  double wspsnr { 0 };
};

struct SEIComplexityMetrics
{
  uint32_t portionNonZeroBlocksArea { 0 };
  uint32_t portionNonZero_4_8_16BlocksArea { 0 };
  uint32_t portionNonZero_32_64_128BlocksArea { 0 };
  uint32_t portionNonZero_256_512_1024BlocksArea { 0 };
  uint32_t portionNonZero_2048_4096BlocksArea { 0 };
  uint32_t portionNonZeroTransformCoefficientsArea { 0 };
  uint32_t portionIntraPredictedBlocksArea { 0 };
  uint32_t portionBdofBlocksArea { 0 };
  uint32_t portionBiAndGpmPredictedBlocksArea { 0 };
  uint32_t portionDeblockingInstances { 0 };
  uint32_t portionSaoInstances { 0 };
  uint32_t portionAlfInstances { 0 };
};

class ChromaCbfs
{
public:
  ChromaCbfs() : Cb(true), Cr(true) {}
  ChromaCbfs(bool _cbf) : Cb(_cbf), Cr(_cbf) {}

public:
  bool sigChroma(ChromaFormat chromaFormat) const
  {
    if (chromaFormat == ChromaFormat::_400)
    {
      return false;
    }
    return (Cb || Cr);
  }
  bool &cbf(CompID compID)
  {
    bool *cbfs[MAX_NUM_TBLOCKS] = { nullptr, &Cb, &Cr };

    return *cbfs[compID];
  }

public:
  bool Cb;
  bool Cr;
};

enum MsgLevel
{
  SILENT  = 0,
  ERROR   = 1,
  WARNING = 2,
  INFO    = 3,
  NOTICE  = 4,
  VERBOSE = 5,
  DETAILS = 6
};

enum ReshapeSignalType
{
  RESHAPE_SIGNAL_SDR  = 0,
  RESHAPE_SIGNAL_PQ   = 1,
  RESHAPE_SIGNAL_HLG  = 2,
  RESHAPE_SIGNAL_NULL = 100,
};

struct SgpmInfo
{
  int sgpmSplitDir;
  int sgpmMode0;
  int sgpmMode1;

  SgpmInfo() : sgpmSplitDir(0), sgpmMode0(0), sgpmMode1(0) {}
  SgpmInfo(const int sd, const int sm0, const int sm1) : sgpmSplitDir(sd), sgpmMode0(sm0), sgpmMode1(sm1) {}
  SgpmInfo &operator=(const SgpmInfo &other)
  {
    sgpmSplitDir = other.sgpmSplitDir;
    sgpmMode0    = other.sgpmMode0;
    sgpmMode1    = other.sgpmMode1;
    return *this;
  }
  bool operator==(const SgpmInfo &other) const
  {
    return (sgpmSplitDir == other.sgpmSplitDir) && (sgpmMode0 == other.sgpmMode0) && (sgpmMode1 == other.sgpmMode1);
  }
};

#if ENABLE_NNLF
enum NNLFUnifiedID
{
  OFF  = 0,
  VLOP = 1,
  LOP  = 2,
  HOP  = 3,
  MAX  = 4,
};

enum NNLFInferSize
{
  SMALL = 0,
  BASE  = 1,
  LARGE = 2,
};

enum NNInputType
{
  NN_INPUT_REC            = 0,
  NN_INPUT_PRED           = 1,
  NN_INPUT_PARTITION      = 2,
  NN_INPUT_BS             = 3,
  NN_INPUT_GLOBAL_QP      = 4,
  NN_INPUT_LOCAL_QP       = 5,
  NN_INPUT_LOCAL_QP_BLOCK = 6,
  NN_INPUT_SLICE_TYPE     = 7,
  NN_INPUT_IPB            = 8,
  NN_INPUT_REF_LIST_0     = 9,
  NN_INPUT_REF_LIST_1     = 10,
  NN_INPUT_ZERO           = 11,
  MAX_NUM_NN_INPUT
};

enum NnlfUnifiedInferGranularity
{
  NNLF_UNIFIED_INFER_GRANULARITY_SMALL   = 0,   // half size
  NNLF_UNIFIED_INFER_GRANULARITY_BASE    = 1,   // specified in SPS
  NNLF_UNIFIED_INFER_GRANULARITY_LARGE   = 2,   // double size
  MAX_NUM_NNLF_UNIFIED_INFER_GRANULARITY = 3
};
#endif

struct EipInfo
{
  enum class REFERENCE_TYPE : uint8_t
  {
    A      = 1 << 0,
    AL     = 1 << 1,
    L      = 1 << 2,
    AL_A   = AL | A,
    AL_L   = AL | L,
    A_L    = A | L,
    AL_A_L = AL | A | L,
    TPL_TYPE_UNDEFINED,
  };

  REFERENCE_TYPE recoType { REFERENCE_TYPE::TPL_TYPE_UNDEFINED };
  ConvModelType  filterShape { CONV_MODEL_UNDEFINED };

  EipInfo() = default;
  EipInfo(REFERENCE_TYPE _recoType, ConvModelType _filterShape) : recoType(_recoType), filterShape(_filterShape) {}
};

// ---------------------------------------------------------------------------
// exception class
// ---------------------------------------------------------------------------

class Exception : public std::exception
{
public:
  Exception(const std::string &_s) : m_str(_s) {}
  Exception(const Exception &_e) : std::exception(_e), m_str(_e.m_str) {}
  virtual ~Exception() noexcept {};
  virtual const char *what() const noexcept { return m_str.c_str(); }
  Exception          &operator=(const Exception &_e)
  {
    std::exception::operator=(_e);
    m_str = _e.m_str;
    return *this;
  }
  template<typename T> Exception &operator<<(T t)
  {
    std::ostringstream oss;
    oss << t;
    m_str += oss.str();
    return *this;
  }

private:
  std::string m_str;
};

// if a check fails with THROW or CHECK, please check if ported correctly from assert in revision 1196)
#define THROW(x) \
  throw(Exception("\nERROR: In function \"") << __FUNCTION__ << "\" in " << __FILE__ << ":" << __LINE__ << ": " << x)
#define CHECK(c, x) \
  if (c)            \
  {                 \
    THROW(x);       \
  }
#define EXIT(x)             throw(Exception("\n") << x << "\n")
#define CHECK_NULLPTR(_ptr) CHECK(!(_ptr), "Accessing an empty pointer pointer!")

#if !NDEBUG   // for non MSVC compiler, define _DEBUG if in debug mode to have same behavior between MSVC and others in
              // debug
#ifndef _DEBUG
#define _DEBUG 1
#endif
#endif

#if defined(_DEBUG)   // || 1   // TBD remove once the SW is ready
#define CHECKD(c, x) \
  if (c)             \
  {                  \
    THROW(x);        \
  }
#else
#define CHECKD(c, x) \
  {}
#endif   // _DEBUG

// ---------------------------------------------------------------------------
// static vector
// ---------------------------------------------------------------------------

template<typename T, size_t N> class static_vector
{
  T      _arr[N];
  size_t _size;

public:
  typedef T         value_type;
  typedef size_t    size_type;
  typedef ptrdiff_t difference_type;
  typedef T        &reference;
  typedef T const  &const_reference;
  typedef T        *pointer;
  typedef T const  *const_pointer;
  typedef T        *iterator;
  typedef T const  *const_iterator;

  static constexpr size_type max_num_elements = N;

  static_vector() : _size(0) {}
  static_vector(size_t N_) : _size(N_) {}
  static_vector(size_t N_, const T &_val) : _size(0) { resize(N_, _val); }
  template<typename It> static_vector(It _it1, It _it2) : _size(0)
  {
    while (_it1 < _it2)
    {
      _arr[_size++] = *_it1++;
    }
  }
  static_vector(std::initializer_list<T> _il) : _size(0)
  {
    typename std::initializer_list<T>::iterator src1 = _il.begin();
    typename std::initializer_list<T>::iterator src2 = _il.end();

    while (src1 < src2)
    {
      _arr[_size++] = *src1++;
    }

    CHECKD(_size > N, "capacity exceeded");
  }
  static_vector &operator=(std::initializer_list<T> _il)
  {
    _size = 0;

    typename std::initializer_list<T>::iterator src1 = _il.begin();
    typename std::initializer_list<T>::iterator src2 = _il.end();

    while (src1 < src2)
    {
      _arr[_size++] = *src1++;
    }

    CHECKD(_size > N, "capacity exceeded");
  }

  void resize(size_t N_)
  {
    CHECKD(N_ > N, "capacity exceeded");
    while (_size < N_)
    {
      _arr[_size++] = T();
    }
    _size = N_;
  }
  void resize(size_t N_, const T &_val)
  {
    CHECKD(N_ > N, "capacity exceeded");
    while (_size < N_)
    {
      _arr[_size++] = _val;
    }
    _size = N_;
  }
  void reserve(size_t N_) { CHECKD(N_ > N, "capacity exceeded"); }
  void push_back(const T &_val)
  {
    CHECKD(_size >= N, "capacity exceeded");
    _arr[_size++] = _val;
  }
  void push_back(T &&val)
  {
    CHECKD(_size >= N, "capacity exceeded");
    _arr[_size++] = std::forward<T>(val);
  }
  void pop_back()
  {
    CHECKD(_size == 0, "calling pop_back on an empty vector");
    _size--;
  }
  void pop_front()
  {
    CHECKD(_size == 0, "calling pop_front on an empty vector");
    _size--;
    for (int i = 0; i < _size; i++)
    {
      _arr[i] = _arr[i + 1];
    }
  }
  void      clear() { _size = 0; }
  reference at(size_t _i)
  {
    CHECKD(_i >= _size, "Trying to access an out-of-bound-element");
    return _arr[_i];
  }
  const_reference at(size_t _i) const
  {
    CHECKD(_i >= _size, "Trying to access an out-of-bound-element");
    return _arr[_i];
  }
  reference operator[](size_t _i)
  {
    CHECKD(_i >= _size, "Trying to access an out-of-bound-element");
    return _arr[_i];
  }
  const_reference operator[](size_t _i) const
  {
    CHECKD(_i >= _size, "Trying to access an out-of-bound-element");
    return _arr[_i];
  }
  reference front()
  {
    CHECKD(_size == 0, "Trying to access the first element of an empty vector");
    return _arr[0];
  }
  const_reference front() const
  {
    CHECKD(_size == 0, "Trying to access the first element of an empty vector");
    return _arr[0];
  }
  reference back()
  {
    CHECKD(_size == 0, "Trying to access the last element of an empty vector");
    return _arr[_size - 1];
  }
  const_reference back() const
  {
    CHECKD(_size == 0, "Trying to access the last element of an empty vector");
    return _arr[_size - 1];
  }
  pointer        data() { return _arr; }
  const_pointer  data() const { return _arr; }
  iterator       begin() { return _arr; }
  const_iterator begin() const { return _arr; }
  const_iterator cbegin() const { return _arr; }
  iterator       end() { return _arr + _size; }
  const_iterator end() const { return _arr + _size; };
  const_iterator cend() const { return _arr + _size; };
  size_type      size() const { return _size; };
  size_type      byte_size() const { return _size * sizeof(T); }
  bool           empty() const { return _size == 0; }

  size_type capacity() const { return N; }
  size_type max_size() const { return N; }
  size_type byte_capacity() const { return sizeof(_arr); }

  iterator insert(const_iterator _pos, const T &_val)
  {
    CHECKD(_size >= N, "capacity exceeded");
    for (difference_type i = _size - 1; i >= _pos - _arr; i--)
    {
      _arr[i + 1] = _arr[i];
    }
    *const_cast<iterator>(_pos) = _val;
    _size++;
    return const_cast<iterator>(_pos);
  }

  iterator insert(const_iterator _pos, T &&_val)
  {
    CHECKD(_size >= N, "capacity exceeded");
    for (difference_type i = _size - 1; i >= _pos - _arr; i--)
    {
      _arr[i + 1] = _arr[i];
    }
    *const_cast<iterator>(_pos) = std::forward<T>(_val);
    _size++;
    return const_cast<iterator>(_pos);
  }
  template<class InputIt> iterator insert(const_iterator _pos, InputIt first, InputIt last)
  {
    const difference_type numEl = last - first;
    CHECKD(_size + numEl > N, "capacity exceeded");
    for (difference_type i = _size - 1; i >= _pos - _arr; i--)
    {
      _arr[i + numEl] = _arr[i];
    }
    iterator it = const_cast<iterator>(_pos);
    _size += numEl;
    while (first != last)
    {
      *it++ = *first++;
    }
    return const_cast<iterator>(_pos);
  }

  iterator insert(const_iterator _pos, size_t numEl, const T &val)
  {   // const difference_type numEl = last - first;
    CHECKD(_size + numEl > N, "capacity exceeded");
    for (difference_type i = _size - 1; i >= _pos - _arr; i--)
    {
      _arr[i + numEl] = _arr[i];
    }
    iterator it = const_cast<iterator>(_pos);
    _size += numEl;
    for (int k = 0; k < numEl; k++)
    {
      *it++ = val;
    }
    return const_cast<iterator>(_pos);
  }

  void erase(const_iterator _pos)
  {
    iterator it   = const_cast<iterator>(_pos) - 1;
    iterator last = end() - 1;
    while (++it != last)
    {
      *it = *(it + 1);
    }
    _size--;
  }

protected:
  void resize_noinit(size_t N_)
  {
    CHECKD(N_ > N, "capacity exceeded");
    _size = N_;
  }
};

// ---------------------------------------------------------------------------
// This class contains a pool of objects that can be used and reused
// while minimizing the amount of required memory allocation and
// deallocation operations.
// ---------------------------------------------------------------------------

template<typename T> class Pool
{
  std::vector<T *> m_items;

public:
  ~Pool() { deleteEntries(); }

  void deleteEntries()
  {
    for (auto &p: m_items)
    {
      delete p;
      p = nullptr;
    }

    m_items.clear();
  }

  T *get()
  {
    T *ret;

    if (!m_items.empty())
    {
      ret = m_items.back();
      m_items.pop_back();
    }
    else
    {
      ret = new T;
    }

    return ret;
  }

  void giveBack(T *el) { m_items.push_back(el); }

  void giveBack(std::vector<T *> &vel)
  {
    m_items.insert(m_items.end(), vel.begin(), vel.end());
    vel.clear();
  }
};

typedef Pool<struct CodingUnit>    CuPool;
typedef Pool<struct TransformUnit> TuPool;

struct XuPool
{
  CuPool cuPool;
  TuPool tuPool;
};

//! \}

#define ALF_SAO_TRUE_ORG 1   // using true original samples for SAO and ALF optimization

#endif
