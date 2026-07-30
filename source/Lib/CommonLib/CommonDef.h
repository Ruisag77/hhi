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

/** \file     CommonDef.h
    \brief    Defines version information, constants and small in-line functions
*/

#ifndef __COMMONDEF__
#define __COMMONDEF__

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <limits>
#include <cstdlib>
#include <cstdint>

#ifdef _MSC_VER
#if _MSC_VER < 1910
#error "MS Visual Studio version not supported. Please upgrade to Visual Studio 2017 or higher (or use other compilers)"
#endif

// disable "signed and unsigned mismatch"
#pragma warning(disable : 4018)
// disable bool coercion "performance warning"
#pragma warning(disable : 4800)
#endif

#include "CommonSimdCfg.h"
#include "StandardizedMathFunctions.h"
#include "TypeDef.h"
#include "version.h"

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Platform information
// ====================================================================================================================

#ifdef __clang__
#define NVM_COMPILEDBY "[clang %d.%d.%d]", __clang_major__, __clang_minor__, __clang_patchlevel__
#ifdef __IA64__
#define NVM_ONARCH "[on 64-bit] "
#else
#define NVM_ONARCH "[on 32-bit] "
#endif
#elif __GNUC__
#define NVM_COMPILEDBY "[GCC %d.%d.%d]", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__
#ifdef __IA64__
#define NVM_ONARCH "[on 64-bit] "
#else
#define NVM_ONARCH "[on 32-bit] "
#endif
#endif

#ifdef __INTEL_COMPILER
#define NVM_COMPILEDBY "[ICC %d]", __INTEL_COMPILER
#elif defined _MSC_VER
#define NVM_COMPILEDBY "[VS %d]", _MSC_VER
#endif

#ifndef NVM_COMPILEDBY
#define NVM_COMPILEDBY "[Unk-CXX]"
#endif

#ifdef _WIN32
#define NVM_ONOS "[Windows]"
#elif __linux
#define NVM_ONOS "[Linux]"
#elif __CYGWIN__
#define NVM_ONOS "[Cygwin]"
#elif __APPLE__
#define NVM_ONOS "[Mac OS X]"
#else
#define NVM_ONOS "[Unk-OS]"
#endif

#define NVM_BITS "[%d bit] ", (sizeof(void *) == 8 ? 64 : 32) ///< used for checking 64-bit O/S

enum class AffineModel : uint8_t
{
  _4_PARAMS,
  _6_PARAMS,
  NUM
};

enum class CIIP_Type : uint8_t
{
  NORMAL,
  WITH_PDPC,
  NUM
};

enum boundaryDirection
{
  BD_TOP,
  BD_RIGHT,
  BD_BOTTOM,
  BD_LEFT,
  NUM_BD_DIRECTIONS
};

static constexpr int GTN          = 7;          // number of greater flags for regular coefficient coding
static constexpr int GTN_LEVEL    = (GTN + 1);  // largest level representable with greater flags
static constexpr int GTN_MAXSUM   = 80;         // maximum template sum
static constexpr int NSIGCTX      = 6;          // number of contexts for significance flag
static constexpr int NGTXCTX      = 7;          // number of contexts for greater flags
static constexpr int MAX_REG_BINS = GTN + 1;    // maximum number of regular bins per coefficient

static constexpr int    AFFINE_ME_LIST_SIZE    = 4;
static constexpr int    AFFINE_ME_LIST_SIZE_LD = 3;
static constexpr double AFFINE_ME_LIST_MVP_TH  = 1.0;
static constexpr int    AFFINE_MAX_NUM_CP      = 3;   // maximum number of control points for affine

static constexpr int32_t NUMBER_PADDED_SAMPLES = 2;
static constexpr int32_t BIF_ROUND_ADD         = 32;
static constexpr int32_t BIF_ROUND_SHIFT       = 6;
static constexpr int32_t BIF_MAD_SHIFT         = 5;

static constexpr int LOG2_SECONDARY_PREFIX_START_SIZE = 3;
static constexpr int LOG2_ID_SUFFIX_CTX_START_SIZE    = 4;

// ====================================================================================================================
// Common constants
// ====================================================================================================================

static constexpr uint64_t MAX_UINT64 = 0xFFFFFFFFFFFFFFFFU;
static constexpr uint32_t MAX_UINT   = 0xFFFFFFFFU; ///< max. value of unsigned 32-bit integer
static constexpr int      MAX_INT    = 2147483647; ///< max. value of signed 32-bit integer
static constexpr uint8_t  MAX_UCHAR  = 255;
static constexpr uint8_t  MAX_SCHAR  = 127;
static constexpr double   MAX_DOUBLE = std::numeric_limits<double>::max(); ///< max. value of double-type value
static constexpr float    MAX_FLOAT  = std::numeric_limits<float>::max();  ///< max. value of float-type value

static constexpr Distortion MAX_DISTORTION = std::numeric_limits<Distortion>::max();

// ====================================================================================================================
// Coding tool configuration
// ====================================================================================================================
// Most of these should not be changed - they resolve the meaning of otherwise magic numbers.
static constexpr int MAX_GOP            = 64;   // max. value of hierarchical GOP size
static constexpr int MAX_NUM_REF_PICS   = 29;   // max. number of pictures used for reference
static constexpr int MAX_NUM_REF        = 16;   // max. number of entries in picture reference list
static constexpr int MAX_NUM_ACTIVE_REF = 15;   // maximum number of active reference pictures
static constexpr int IBC_REF_IDX        = MAX_NUM_ACTIVE_REF;

// Array indexed by reference list index and reference picture index
template<class T> using RefSetArray = T[NUM_RPL01][MAX_NUM_REF];

static constexpr int MAX_QP    = 63;
static constexpr int NOT_VALID = -1;

static constexpr int QTBTTT_TEMPO_PRED_BUFFER_SIZE      = 16;
static constexpr int QTBTTT_TEMPO_PRED_BLOCK_RESOLUTION = 16;
static constexpr int LOG2_BLOCK_RESOLUTION              = 4;

static constexpr int AMVP_MAX_NUM_CANDS =
  2; ///< AMVP: advanced motion vector prediction - max number of final candidates
static constexpr int AMVP_MAX_NUM_CANDS_MEM =
  3; ///< AMVP: advanced motion vector prediction - max number of candidates
static constexpr int AMVP_DECIMATION_FACTOR   = 1;
static const int     NUM_CMVP_CANDS           = 4; ///< CMVP
static const int     MRG_MAX_NUM_CANDS        = 28 + NUM_CMVP_CANDS;   ///< for new maximum buffer of merging candidates
static constexpr int LAST_MERGE_IDX_CABAC     = 5;
static constexpr int AFFINE_MRG_MAX_NUM_CANDS = 15;   ///< AFFINE MERGE
static constexpr int AFF_NON_ADJACENT_DIST    = 4;
static constexpr int AFF_MAX_NON_ADJACENT_INHERITED_CANDS = 6;
static constexpr int RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE   = 30;
static constexpr int RMVF_MV_RANGE                        = (1 << 12);
static constexpr int RMVF_CUSIZE_THRED                    = 128;
static constexpr int RMVF_DISTANCE_THRED                  = 256;
static constexpr int RMVF_NUM_SUBBLK_THRED                = 255;
static constexpr int RMVF_PARAM_THRED                     = (1 << 20);
static constexpr int IBC_MRG_MAX_NUM_CANDS                = 6;   ///< IBC MERGE

static const int     ADAPTIVE_CLIP_SHIFT_DELTA_VALUE_1 = 5;
static const int     ADAPTIVE_CLIP_SHIFT_DELTA_VALUE_0 = 1;
static constexpr int MAX_TLAYER = 7;   ///< Explicit temporal layer QP offset - max number of temporal layer

static constexpr int ADAPT_SR_SCALE = 1;   ///< division factor for adaptive search range

static constexpr int MIN_TB_LOG2_SIZEY = 2;
static constexpr int MAX_TB_LOG2_SIZEY = 8;

static constexpr int MIN_TB_SIZEY = 1 << MIN_TB_LOG2_SIZEY;
static constexpr int MAX_TB_SIZEY = 1 << MAX_TB_LOG2_SIZEY;

static constexpr int MAX_NESTING_NUM_LAYER = 64;

static constexpr int MAX_VPS_LAYERS       = 64;
static constexpr int MAX_VPS_SUBLAYERS    = 7;
static constexpr int MAX_NUM_OLSS         = 256;
static constexpr int MAX_VPS_OLS_MODE_IDC = 2;

static constexpr int MAX_NUM_VPS      = 16;
static constexpr int MAX_NUM_SPS      = 16;
static constexpr int MAX_NUM_PPS      = 64;
static constexpr int NUM_APS_TYPE_LEN = 3;   // Currently APS Type has 3 bits
static constexpr int MAX_NUM_APS_TYPE = 8;   // Currently APS Type has 3 bits so the max type is 8

static constexpr int MAX_NUM_NN_POST_FILTERS = 8;

static constexpr int MAX_INTRA_SIZE = 128;
static constexpr int MIP_MAX_WIDTH  = MAX_INTRA_SIZE;
static constexpr int MIP_MAX_HEIGHT = MAX_INTRA_SIZE;
static constexpr int CIIP_MIN_AREA  = 32;

static constexpr int MAX_PDP_SIZE    = 32;
static constexpr int PDP_NUM_MODES   = 67;
static constexpr int PDP_NUM_SIZES   = 18;
static constexpr int PDP_NUM_GROUPS  = 67;
static constexpr int PDP_SHORT_TH[3] = { 0, 19, 49 };

static constexpr int MAX_NUM_AFFHMVP_ENTRIES_ONELIST = 5;
static constexpr int MAX_NUM_AFFHMVP_ENTRIES         = MAX_NUM_AFFHMVP_ENTRIES_ONELIST * 2;

static constexpr int MAX_CPB_CNT       = 32;   ///< Upper bound of (cpb_cnt_minus1 + 1)
static constexpr int MAX_NUM_LAYER_IDS = 64;
static constexpr int COEF_REMAIN_BIN_REDUCTION =
  5;   ///< indicates the level at which the VLC transitions from Golomb-Rice to TU+EG(k)
static constexpr int CU_DQP_TU_CMAX = 5;   ///< max number bins for truncated unary
static constexpr int CU_DQP_EG_k    = 0;   ///< expgolomb order

static constexpr int SBH_THRESHOLD = 4;   ///< value of the fixed SBH controlling threshold

static constexpr int MAX_TILE_COLS = 30;   // Maximum number of tile columns
static constexpr int MAX_TILES     = 990;   // Maximum number of tiles
static constexpr int MAX_SLICES    = 1000;   // Maximum number of slices per picture

static constexpr int MLS_GRP_NUM =
  MAX_TB_SIZEY * MAX_TB_SIZEY >> 4;   ///< Max number of coefficient groups, max(16, 256)

static constexpr int MLS_CG_SIZE = 4;   ///< Coefficient group size of 4x4; = MLS_CG_LOG2_WIDTH + MLS_CG_LOG2_HEIGHT

static constexpr int RVM_VCEGAM10_M = 4;

static constexpr int MAX_REF_LINE_IDX                          = 13;   // highest refLine offset in the list
static constexpr int MRL_NUM_REF_LINES                         = 6;   // number of candidates in the array
static constexpr int MULTI_REF_LINE_IDX[MRL_NUM_REF_LINES + 1] = { 0, 1, 3, 5, 7, MAX_REF_LINE_IDX - 1, 0 };
static constexpr int MULTI_REF_LINE_2_IDX[MAX_REF_LINE_IDX]    = {
  0, 1, -1, 2, -1, 3, -1, 4, -1, -1, -1, -1, MRL_NUM_REF_LINES - 1
};

static constexpr int NUM_DIR                 = 16;
static constexpr int NUM_INTRA_ANGULAR_MODES = 4 * NUM_DIR + 1;
static constexpr int ANGULAR_BASE            = 2;   // First two modes and planar and DC
static constexpr int NUM_LUMA_MODE           = ANGULAR_BASE + NUM_INTRA_ANGULAR_MODES;
static constexpr int NUM_LMC_MODE            = 1 + 2 + 3;   ///< LMC + MDLM_T + MDLM_L + MMLM + MMLM_L + MMLM_T
static constexpr int NUM_INTRA_MODE          = NUM_LUMA_MODE + NUM_LMC_MODE;

static constexpr int NUM_EXT_LUMA_MODE = 30;

static constexpr int PLANAR_IDX = 0;   ///< index for intra PLANAR mode
static constexpr int DC_IDX     = 1;   ///< index for intra DC     mode
static constexpr int HOR_IDX    = (1 * NUM_DIR + ANGULAR_BASE);   ///< index for intra HORIZONTAL mode
static constexpr int DIA_IDX    = (2 * NUM_DIR + ANGULAR_BASE);   ///< index for intra DIAGONAL   mode
static constexpr int VER_IDX    = (3 * NUM_DIR + ANGULAR_BASE);   ///< index for intra VERTICAL   mode
static constexpr int VDIA_IDX   = (4 * NUM_DIR + ANGULAR_BASE);   ///< index for intra VDIAGONAL  mode

static constexpr int BDPCM_IDX = 162;

static const int INTRA_FUSION_BITS = 16;
static const int LOC_DEP_BITS      = 5;
static const int DIMD_FUSION_NUM   = 6;   // max number of modes blended in DIMD (including planar)

static const int TIMD_IDX               = 199;   ///< index for intra TIMD mode
static const int TIMD_SAD_IDX           = 241;   ///< index for intra TIMD SAD mode
static const int TIMD_NUM_MODES_SORTED  = 15;
static const int TIMDDIFF_MAX_TEMP_SIZE = 8;
static const int TIMD_MAX_TEMP_SIZE     = 4;
static const int EXT_HOR_IDX            = 34;
static const int EXT_DIA_IDX            = 66;
static const int EXT_VER_IDX            = 98;
static const int EXT_VDIA_IDX           = 130;
static const int TIMD_FUSION_NUM        = 3;
static const int NUM_TIMD_MERGE_CUS         = 13 + 18;   // adjacent and non-adjacent positions
static const int TIMD_MERGE_MAX_NEIGHBOURS = 5;
#define MAP131TO67(mode) (mode < 2 ? mode : ((mode >> 1) + 1))
#define MAP67TO131(mode) (mode < 2 ? mode : ((mode << 1) - 2))

static const int OBIC_FUSION_NUM = 6;
static const int NUM_OBIC_CUS    = 13 + 18;   // (13: adjacent, 18: non-adjacent)

static const int MAX_IPM_FUSION_NUM = std::max({ DIMD_FUSION_NUM, TIMD_FUSION_NUM, OBIC_FUSION_NUM });

static constexpr int NUM_CHROMA_MODE = (5 + NUM_LMC_MODE);   ///< total number of chroma modes
static constexpr int LM_CHROMA_IDX   = NUM_LUMA_MODE;   ///< chroma mode index for derived from LM mode
static constexpr int MMLM_CHROMA_IDX = LM_CHROMA_IDX + 1;   ///< MDLM_L
static constexpr int MDLM_L_IDX      = LM_CHROMA_IDX + 2;   ///< MDLM_L
static constexpr int MMLM_L_IDX      = LM_CHROMA_IDX + 3;   ///< MDLM_T
static constexpr int MDLM_T_IDX      = LM_CHROMA_IDX + 4;   ///< MDLM_L
static constexpr int MMLM_T_IDX      = LM_CHROMA_IDX + 5;   ///< MDLM_T
static constexpr int DM_CHROMA_IDX   = NUM_INTRA_MODE;   ///< chroma mode index for derived from luma intra mode

static const int BVD_CODING_GOLOMB_ORDER = 1;
static const int NUM_HOR_BVD_CTX         = 5;
static const int NUM_VER_BVD_CTX         = 5;
static const int HOR_BVD_CTX_OFFSET      = 0;
static const int BVD_IBC_MAX_PREFIX      = 16;
static const int VER_BVD_CTX_OFFSET      = 6;

// JVET_AG0058_EIP
static const int EIP_IDX                                      = 201;
static const int MAX_EIP_SIZE                                 = 32;
static const int EIP_FILTER_TAP                               = 15;
static const int EIP_FILTER_SIZE                              = 7;
static const int MAX_EIP_REF_SIZE                             = EIP_FILTER_SIZE + MAX_EIP_SIZE;
static const int MAX_NUM_HEIP_CANDS                           = 6;
static const int MAX_MERGE_EIP                                = 12;
static const int NUM_EIP_MERGE_SIGNAL                         = 6;
static const int EIP_TPL_SIZE                                 = 1;
static const int NUM_EIP_FILTER_SHAPE                         = 3;
static const int NUM_DERIVED_EIP                              = 9;
static const int NUM_EIP_MODELS                               = 2;
static const int EIP_L2_REGULARIZATION_SAMPLE_THRESHOLD       = 2024;
static const int EIP_L2_REGULARIZATION_SMALL                  = 192;
static const int EIP_L2_REGULARIZATION_LARGE                  = 128;
static const int EIP_L2_MM_REGULARIZATION[6]                  = { 320, 288, 256, 224, 192, 128 };
static const int EIP_L2_MM_REGULARIZATION_SAMPLE_THRESHOLD[5] = { 128, 256, 512, 1024, 2048 };

static constexpr int      NUM_LFNST_INTRA_MODES = NUM_LUMA_MODE + NUM_EXT_LUMA_MODE;
static constexpr uint32_t L16W_ZO               = 96;
static constexpr uint32_t L16W                  = 96;
static constexpr uint32_t L16H                  = 32;
static constexpr uint32_t L16H_ZO               = 32;
static constexpr uint32_t L8W                   = 64;
static constexpr uint32_t L8W_ZO                = 64;
static constexpr uint32_t L8H                   = 32;
static constexpr uint32_t L8H_ZO                = 32;

static constexpr int64_t  MTS_TH_COEFF[2] = { 6, 32 };
static constexpr uint32_t MTS_NCANDS[3]   = { 1, 4, 6 };
static constexpr uint32_t MTS_INTRA_MAX_CU_SIZE =
  256;   ///< Max Intra CU size applying EMT, supported values: 8, 16, 32, 64, 128, 256
static constexpr uint32_t MTS_INTER_MAX_CU_SIZE =
  256;   ///< Max Inter CU size applying EMT, supported values: 8, 16, 32, 64, 128, 256
static constexpr int NUM_PRIMARY_MOST_PROBABLE_MODES   = 6;
static constexpr int NUM_SECONDARY_MOST_PROBABLE_MODES = 16;
static constexpr int NUM_MOST_PROBABLE_MODES = NUM_PRIMARY_MOST_PROBABLE_MODES + NUM_SECONDARY_MOST_PROBABLE_MODES;
static constexpr int NUM_NON_MPM_MODES       = NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES;
static constexpr int LM_SYMBOL_NUM           = (1 + NUM_LMC_MODE);

static constexpr int      MAX_NUM_MIP_MODE   = 32;   ///< maximum number of MIP pred. modes
static constexpr int      SGPM_NUM           = 16;
static constexpr int      SGPM_VIPM_TH       = 2;
static constexpr uint32_t MAX_SGPM_VIPM_SIZE = 256;
static constexpr int      FAST_UDI_MAX_RDMODE_NUM =
  (NUM_LUMA_MODE + MAX_NUM_MIP_MODE + SGPM_NUM +
   2 /*BDPCM*/);   ///< maximum number of RD comparison in fast-UDI estimation loop
static constexpr int LFNST_LAST_SIG_LUMA   = 1;
static constexpr int LFNST_LAST_SIG_CHROMA = 1;

static constexpr int NUM_LFNST_SETS        = 35;
static constexpr int NUM_LFNST_NUM_PER_SET = 4;

static constexpr int NUM_NSPT_BLOCK_TYPES   = 3;   ///< 0:Regular 1:Mixed 2:Inter/ITMP
static constexpr int NUM_NSPT_CLUSTERS_4x4  = 245;
static constexpr int NUM_NSPT_CLUSTERS_4x8  = 245;
static constexpr int NUM_NSPT_CLUSTERS_8x4  = 221;
static constexpr int NUM_NSPT_CLUSTERS_8x8  = 245;
static constexpr int NUM_NSPT_CLUSTERS_4x16 = 213;
static constexpr int NUM_NSPT_CLUSTERS_16x4 = 183;
static constexpr int NUM_NSPT_CLUSTERS_8x16 = 213;
static constexpr int NUM_NSPT_CLUSTERS_16x8 = 194;
static constexpr int NUM_NSPT_CLUSTERS_4x32 = 149;
static constexpr int NUM_NSPT_CLUSTERS_32x4 = 113;
static constexpr int NUM_NSPT_CLUSTERS_8x32 = 149;
static constexpr int NUM_NSPT_CLUSTERS_32x8 = 127;

static constexpr int CABAC_INIT_PRESENT_FLAG = 1;

static constexpr int MV_FRACTIONAL_BITS_INTERNAL = 4;
static constexpr int MV_FRACTIONAL_BITS_SIGNAL   = 2;
static constexpr int MV_FRACTIONAL_BITS_DIFF     = MV_FRACTIONAL_BITS_INTERNAL - MV_FRACTIONAL_BITS_SIGNAL;
static constexpr int LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL = 1 << MV_FRACTIONAL_BITS_SIGNAL;
static constexpr int MV_FRAC_BITS_LUMA                                     = MV_FRACTIONAL_BITS_INTERNAL;
static constexpr int MV_FRAC_BITS_CHROMA                                   = MV_FRACTIONAL_BITS_INTERNAL + 1;
static constexpr int MV_FRAC_MASK_LUMA                                     = (1 << MV_FRAC_BITS_LUMA) - 1;
static constexpr int MV_FRAC_MASK_CHROMA                                   = (1 << MV_FRAC_BITS_CHROMA) - 1;
static constexpr int LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS        = 1 << MV_FRAC_BITS_LUMA;
static constexpr int CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS      = 1 << MV_FRAC_BITS_CHROMA;

static constexpr int MAX_NUM_SUB_PICS           = (1 << 16);
static constexpr int MAX_NUM_LONG_TERM_REF_PICS = MAX_NUM_REF;
static constexpr int NUM_LONG_TERM_REF_PIC_SPS  = 0;

static constexpr int MAX_QP_OFFSET_LIST_SIZE = 6;   ///< Maximum size of QP offset list is 6 entries
static constexpr int MAX_NUM_CQP_MAPPING_TABLES =
  3;   ///< Maximum number of chroma QP mapping tables (Cb, Cr and joint Cb-Cr)
static constexpr int MIN_QP_VALUE_FOR_16_BIT = -48;   ////< Minimum value for QP (-6*(bitdepth - 8) ) for bit depth 16 ;
                                                      /// actual minimum QP value is bit depth dependent
static constexpr int MAX_NUM_QP_VALUES =
  MAX_QP + 1 - MIN_QP_VALUE_FOR_16_BIT;   ////< Maximum number of QP values possible - bit depth dependent

// Cost mode support
static constexpr int LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP = 0;   ///< QP to use for lossless coding.
static constexpr int LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME =
  4;   ///< QP' to use for mixed_lossy_lossless coding.
static constexpr int RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS = MAX_NUM_COMP;

static constexpr int RExt__PREDICTION_WEIGHTING_ANALYSIS_DC_PRECISION =
  0;   ///< Additional fixed bit precision used during encoder-side weighting prediction analysis. Currently only used
       ///< when high_precision_prediction_weighting_flag is set, for backwards compatibility reasons.

static constexpr int MAX_CU_DEPTH         = 8;   // log2(CTUSize)
static constexpr int MAX_CU_SIZE          = (1 << MAX_CU_DEPTH);
static constexpr int MIN_CU_LOG2          = 2;
static constexpr int MIN_CU_SIZE          = 1 << MIN_CU_LOG2;
static constexpr int MAX_CU_SIZE_IN_PARTS = MAX_CU_SIZE >> MIN_CU_LOG2;
static constexpr int MAX_NUM_PARTS_IN_CTU = MAX_CU_SIZE_IN_PARTS * MAX_CU_SIZE_IN_PARTS;
static constexpr int MIN_PU_SIZE          = 4;

// Maximum number of TUs within one CU. When max TB size is 32x32, up to 16 TUs within one CU (128x128) is supported
static constexpr int MAX_NUM_TUS              = 16;
static constexpr int MAX_LOG2_DIFF_CU_TR_SIZE = 3;
static constexpr int MAX_CU_TILING_PARTITIONS = 1 << (2 * MAX_LOG2_DIFF_CU_TR_SIZE);

static constexpr int MAX_NUM_SIZES = 9;

static constexpr int PIC_MARGIN = 16;

static constexpr int MAX_NONZERO_TU_SIZE = 256;

// returns the size of the part of a TU that is not zero'ed out
static inline constexpr int getNonzeroTuSize(int s) { return std::min(s, MAX_NONZERO_TU_SIZE); }

static constexpr int MAX_NUM_PART_IDXS_IN_CTU_WIDTH =
  MAX_CU_SIZE / MIN_PU_SIZE;   ///< maximum number of partition indices across the width of a CTU (or height of a CTU)
static constexpr int SCALING_LIST_REM_NUM = 6;

static constexpr int QUANT_SHIFT  = 14;   ///< Q(4) = 2^14
static constexpr int IQUANT_SHIFT = 6;

static constexpr int    SCALE_BITS      = 15;   // Precision for fractional bit estimates
static constexpr double FRAC_BITS_SCALE = 1.0 / (1 << SCALE_BITS);

static constexpr int SCALING_LIST_PRED_MODES = 2;
static constexpr int SCALING_LIST_NUM =
  MAX_NUM_COMP * SCALING_LIST_PRED_MODES;   ///< list number for quantization matrix

static constexpr int SCALING_LIST_START_VALUE = 8;   ///< start value for dpcm mode
static constexpr int MAX_MATRIX_COEF_NUM      = 64;   ///< max coefficient number for quantization matrix
static constexpr int MAX_MATRIX_SIZE_NUM      = 8;   ///< max size number for quantization matrix
static constexpr int SCALING_LIST_BITS        = 8;   ///< bit depth of scaling list entries
static constexpr int LOG2_SCALING_LIST_NEUTRAL_VALUE =
  4;   ///< log2 of the value that, when used in a scaling list, has no effect on quantisation
static constexpr int SCALING_LIST_DC = 16;   ///< default DC value

static constexpr int LAST_SIGNIFICANT_GROUPS = 16;

static constexpr int AFFINE_SUBBLOCK_SIZE = 4;   // Minimum affine MC block size

static constexpr int AF_MMVD_BASE_NUM   = 1;
static constexpr int AF_MMVD_STEP_NUM   = 5;   // number of distance offset
static constexpr int AF_MMVD_OFFSET_DIR = 4;   // 00: (+, 0); 01: (-, 0); 10: (0, +); 11 (0, -);

static const int AFF_MRG_MAX_NUM_CANDS_OPPOSITELIC = 8;
static const int REG_MRG_MAX_NUM_CANDS_OPPOSITELIC = 5;

union MmvdIdx
{
  using T = uint8_t;

  static constexpr int LOG_REFINE_STEP = 3;
  static constexpr int REFINE_STEP     = 1 << LOG_REFINE_STEP;
  static constexpr int LOG_BASE_MV_NUM = 1;
  static constexpr int BASE_MV_NUM     = 1 << LOG_BASE_MV_NUM;
  static constexpr int MAX_REFINE_NUM  = 4 * REFINE_STEP;
  static constexpr int ADD_NUM         = MAX_REFINE_NUM * BASE_MV_NUM;
  static constexpr int INVALID         = std::numeric_limits<T>::max();

  struct
  {
    T baseIdx : LOG_BASE_MV_NUM;
    T step : LOG_REFINE_STEP;
    T position : 2;
  } pos;
  T val;
};

static_assert(sizeof(MmvdIdx::val) == sizeof(MmvdIdx::pos), "MmvdIdx::val is not wide enough");

static constexpr int MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_LUMA   = 28;
static constexpr int MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_CHROMA = 28;

static const int     BDOF_SUBPU_DIM_LOG2              = 2;
static constexpr int BDOF_SUBPU_AREA_THRESHOLD16      = 512;
static constexpr int BDOF_SUBPU_AREA_THRESHOLD0       = 256;
static constexpr int BDOF_SUBPU_AREA_THRESHOLD1       = 2048;
static constexpr int BDOF_SUBPU_AREA_THRESHOLDAFFINE0 = 0;
static constexpr int BDOF_SUBPU_DIM                   = (1 << BDOF_SUBPU_DIM_LOG2);
static constexpr int BDOF_SUBPU_MAX_NUM               = ((MAX_CU_SIZE * MAX_CU_SIZE) >> (BDOF_SUBPU_DIM_LOG2 << 1));
static constexpr int BDOF_SUBPU_STRIDE                = (MAX_CU_SIZE >> BDOF_SUBPU_DIM_LOG2);
static constexpr int BDOF_SUBPU_SIZE                  = (1 << (BDOF_SUBPU_DIM_LOG2 << 1));
static constexpr int BDOF_EXTEND_SIZE                 = 3;

static constexpr int BDOF_TEMP_BUFFER_SIZE =
  (MAX_CU_SIZE + 2 * BDOF_EXTEND_SIZE) * (MAX_CU_SIZE + 2 * BDOF_EXTEND_SIZE);

static constexpr int BDOF_DMVR_MAX_ITER = 3;

static constexpr int PROF_BORDER_EXT_W = 1;
static constexpr int PROF_BORDER_EXT_H = 1;

static constexpr int    BCW_LOG2_WEIGHT_BASE = 3;
static constexpr int    BCW_WEIGHT_BASE      = 1 << BCW_LOG2_WEIGHT_BASE;
static constexpr int    BCW_NUM              = 5;   // the number of weight options
static constexpr int    BCW_DEFAULT          = BCW_NUM / 2;   // Default weighting index representing for w=0.5
static constexpr int    BCW_SIZE_CONSTRAINT  = 256;   // disabling Bcw if cu size is smaller than 256
static constexpr int    BCW_INV_BITS         = 16;
static constexpr double BCW_COST_TH          = 1.05;

static constexpr double AMVR_FAST_4PEL_TH = 1.06;

static constexpr int MAX_NUM_HMVP_CANDS =
  5;   ///< maximum number of HMVP candidates to be stored and used in merge list
static constexpr int MAX_NUM_HMVP_AVMPCANDS = 4;   ///< maximum number of HMVP candidates to be used in AMVP list
static constexpr int MAX_NUM_AFF_HMVP_CANDS = 7;
static constexpr int MAX_NUM_AFF_INHERIT_HMVP_CANDS = 9;

static constexpr int ALF_VB_POS_ABOVE_CTUROW_LUMA = 4;
static constexpr int ALF_VB_POS_ABOVE_CTUROW_CHMA = 2;

static constexpr int MAX_ENCODER_DEBLOCKING_QUALITY_LAYERS = 8;

#if SHARP_LUMA_DELTA_QP
static constexpr uint32_t LUMA_LEVEL_TO_DQP_LUT_MAXSIZE = 1024;   ///< max LUT size for QP offset based on luma

#endif
static constexpr int DMVR_SUBCU_WIDTH_LOG2  = 4;
static constexpr int DMVR_SUBCU_HEIGHT_LOG2 = 4;
static constexpr int DMVR_SUBCU_WIDTH       = 1 << DMVR_SUBCU_WIDTH_LOG2;
static constexpr int DMVR_SUBCU_HEIGHT      = 1 << DMVR_SUBCU_HEIGHT_LOG2;
static constexpr int DMVR_SUBPU_STRIDE      = (MAX_CU_SIZE >> DMVR_SUBCU_WIDTH_LOG2);
static constexpr int MAX_NUM_SUBCU_DMVR_LOG2 =
  MAX_CU_DEPTH + MAX_CU_DEPTH - DMVR_SUBCU_WIDTH_LOG2 - DMVR_SUBCU_HEIGHT_LOG2;
static constexpr int MAX_NUM_SUBCU_DMVR = 1 << MAX_NUM_SUBCU_DMVR_LOG2;

static constexpr int DMVR_RANGE = 2;
static constexpr int DMVR_SPAN  = 2 * DMVR_RANGE + 1;

// QTBT high level parameters
// for I slice luma CTB configuration para.
static constexpr int         MAX_BT_DEPTH              = 4;   ///<  <=7
                                         // for P/B slice CTU config. para.
static constexpr int         MAX_BT_DEPTH_INTER        = 4;   ///< <=7
                                               // for I slice chroma CTB configuration para. (in luma samples)
static constexpr int         MAX_BT_DEPTH_C            = 0;   ///< <=7
static constexpr int         MIN_DUALTREE_CHROMA_WIDTH = 4;
static constexpr int         MIN_DUALTREE_CHROMA_SIZE  = 16;
static constexpr SplitSeries SPLIT_BITS                = 5;
static constexpr SplitSeries SPLIT_DMULT               = 5;
static constexpr SplitSeries SPLIT_MASK                = (1 << SPLIT_BITS) - 1;

static constexpr int SKIP_DEPTH          = 3;
static constexpr int PICTURE_DISTANCE_TH = 1;
static constexpr int FAST_SKIP_DEPTH     = 2;

static constexpr double PBINTRA_RATIO         = 1.1;
static constexpr int    NUM_MRG_SATD_CAND     = 4;
static constexpr double MRG_FAST_RATIO        = 1.25;
static constexpr int    NUM_AFF_MRG_SATD_CAND = 2;

static constexpr int BDMVR_INTME_RANGE =
  8;   ///< Bilateral matching search range (n represents for -n pel to n pel, inclusive)
static constexpr int BDMVR_INTME_STRIDE = (BDMVR_INTME_RANGE << 1) + 1;   ///< Bilateral matching search stride
static constexpr int BDMVR_INTME_AREA   = BDMVR_INTME_STRIDE * BDMVR_INTME_STRIDE;   ///< Bilateral matching search area
static constexpr int BDMVR_INTME_CENTER =
  BDMVR_INTME_STRIDE * BDMVR_INTME_RANGE + BDMVR_INTME_RANGE;   ///< Bilateral matching search area center
static constexpr int BDMVR_SIMD_IF_FACTOR =
  8;   ///< Specify the pixel alignment factor for SIMD IF. (Usually this factor is 8)
static constexpr int BDMVR_INTME_MAX_NUM_SEARCH_ITERATION =
  26;   ///< for entire CU in bilateral mode, maximum number of refinement loops
static constexpr int BDMVR_BUF_STRIDE      = MAX_CU_SIZE + (BDMVR_INTME_RANGE << 1) + (BDMVR_SIMD_IF_FACTOR - 2);
static constexpr int BDMVR_CENTER_POSITION = BDMVR_INTME_RANGE * BDMVR_BUF_STRIDE + BDMVR_INTME_RANGE;
static constexpr int BM_MRG_MAX_NUM_CANDS =
  6;   ///< maximum number of BM merge candidates (note: should be at most equal to MRG_MAX_NUM_CANDS)
static constexpr int BM_MRG_SUB_PU_INT_MAX_SRCH_ROUND = 3;
static constexpr int DECODER_SIDE_MV_WEIGHT           = 4;   ///< lambda for decoder-side derived MVs

static constexpr int    AFFINE_DMVR_SEARCH_RANGE    = 3;
static constexpr int    AFFINE_DMVR_INT_SRCH_RANGE  = 2;
static constexpr int    AFFINE_DMVR_MIN_SUBBLK_SIZE = 4;
static constexpr int    DMVR_PARA_BASE_NUM          = 3;
static constexpr int    DMVR_PARA_ROUND_NUM_BASE0   = 7;
static constexpr int    DMVR_PARA_ROUND_NUM_BASE1   = 2;
static constexpr int    DMVR_PARA_ROUND_NUM_BASE2   = 1;
static constexpr double TH_COST                     = 0.90;
static constexpr int    PARA_PRECISION_BIT          = 2;

static constexpr double AMAXBT_TH32  = 15.0;
static constexpr double AMAXBT_TH64  = 30.0;
static constexpr double AMAXBT_TH128 = 60.0;

static constexpr int FAST_METHOD_TT_ENC_SPEEDUP =
  0x0001;   ///< Embedding flag, which, if false, de-activates all the following ABT_ENC_SPEEDUP_* modes
static constexpr int FAST_METHOD_HOR_XOR_VER           = 0x0002;
static constexpr int FAST_METHOD_ENC_SPEEDUP_BT_BASED  = 0x0004;
static constexpr int FAST_METHOD_TT_ENC_SPEEDUP_BSLICE = 0x0008;
static constexpr int FAST_METHOD_TT_ENC_SPEEDUP_ISLICE = 0x0010;

// need to know for static memory allocation
static constexpr int MAX_DELTA_QP = 7;   ///< maximum supported delta QP value

static constexpr int COM16_C806_TRANS_PREC = 0;

#if IF_12TAP
static constexpr int NTAPS_LUMA = 12;   // Number of taps for luma
#else
static constexpr int NTAPS_LUMA = 8;   // Number of taps for luma
#endif
static constexpr int NTAPS_LUMA_AFFINE = 6;   // Number of taps for luma affine
#if JVET_Z0117_CHROMA_IF
static constexpr int NTAPS_CHROMA = 6;   // Number of taps for chroma
#else
static constexpr int NTAPS_CHROMA = 4;   // Number of taps for chroma
#endif
static constexpr int NTAPS_BILINEAR  = 2;   // Number of taps for bilinear filter
static constexpr int MAX_FILTER_SIZE = NTAPS_LUMA > NTAPS_CHROMA ? NTAPS_LUMA : NTAPS_CHROMA;
static constexpr int NTAPS_LUMA_IBC  = 8;   // Number of taps for IBC luma filter

static constexpr int MAX_LADF_INTERVALS = 5;   /// max number of luma adaptive deblocking filter qp offset intervals

static constexpr int MAX_RPR_SWITCHING_ORDER_LIST_SIZE = 32;   /// max number of pre-defined RPR switching segments
static constexpr int ATMVP_SUB_BLOCK_SIZE              = 2;   ///< sub-block size for ATMVP
static constexpr int GEO_MAX_NUM_UNI_CANDS             = 15;
static constexpr int GEO_MAX_NUM_INTRA_CANDS           = 3;
static constexpr int GEO_MIN_CU_LOG2                   = 3;
static constexpr int GEO_MAX_CU_LOG2                   = 6;
static constexpr int GEO_MIN_CU_SIZE                   = 1 << GEO_MIN_CU_LOG2;
static constexpr int GEO_MAX_CU_SIZE                   = 1 << GEO_MAX_CU_LOG2;
static constexpr int GEO_NUM_CU_SIZE                   = (GEO_MAX_CU_LOG2 - GEO_MIN_CU_LOG2) + 1;
static constexpr int GEO_NUM_PARTITION_MODE            = 64;

static constexpr int GEO_LOG2_NUM_ANGLES    = 5;
static constexpr int GEO_NUM_ANGLES         = 1 << GEO_LOG2_NUM_ANGLES;
static constexpr int GEO_LOG2_NUM_DISTANCES = 2;
static constexpr int GEO_NUM_DISTANCES      = 1 << GEO_LOG2_NUM_DISTANCES;

static constexpr int GEO_NUM_PRESTORED_MASK        = 6;
static constexpr int GEO_WEIGHT_MASK_SIZE          = 3 * (GEO_MAX_CU_SIZE >> 3) * 2 + GEO_MAX_CU_SIZE;
static constexpr int GPM_MMVD_REFINE_STEP          = 8;
static constexpr int GPM_MMVD_REFINE_DIRECTION     = 4;
static constexpr int GPM_MMVD_MAX_REFINE_NUM       = (GPM_MMVD_REFINE_STEP * GPM_MMVD_REFINE_DIRECTION);
static constexpr int GPM_EXT_MMVD_REFINE_STEP      = 9;
static constexpr int GPM_EXT_MMVD_REFINE_DIRECTION = 8;
static constexpr int GPM_EXT_MMVD_MAX_REFINE_NUM2 =
  (GPM_EXT_MMVD_REFINE_STEP * GPM_EXT_MMVD_REFINE_DIRECTION) + GEO_NUM_TM_MV_CAND;
static constexpr int GPM_EXT_MMVD_MAX_REFINE_NUM = (GPM_EXT_MMVD_REFINE_STEP * GPM_EXT_MMVD_REFINE_DIRECTION);
static constexpr int GPM_MODES_TOTAL             = ((GPM_EXT_MMVD_MAX_REFINE_NUM2) * (GEO_MAX_NUM_UNI_CANDS + 1));
static constexpr int GEO_MAX_TRY_WEIGHTED_SAD    = 70;
static constexpr int GEO_MAX_TRY_WEIGHTED_SATD   = 8;
static constexpr int GEO_NUM_BLD                 = 5;
static constexpr int GEO_MIN_CU_LOG2_EX          = 2;
static constexpr int GEO_MAX_CU_LOG2_EX          = 6;
static constexpr int GEO_MIN_CU_SIZE_EX          = 1 << GEO_MIN_CU_LOG2_EX;
static constexpr int GEO_MAX_CU_SIZE_EX          = 1 << GEO_MAX_CU_LOG2_EX;
static constexpr int GEO_NUM_CU_SIZE_EX          = (GEO_MAX_CU_LOG2_EX - GEO_MIN_CU_LOG2_EX) + 1;

static constexpr int SGPM_MIN_PIX                  = 32;
static constexpr int SGPM_NUM_MPM                  = 3;
static constexpr int SGPM_TEMPLATE_SIZE            = 1;
static constexpr int AML_MERGE_TEMPLATE_SIZE       = 1;
static constexpr int GEO_MODE_SEL_TM_SIZE          = AML_MERGE_TEMPLATE_SIZE;
static constexpr int GEO_TM_ADDED_WEIGHT_MASK_SIZE = GEO_MODE_SEL_TM_SIZE;
static constexpr int GEO_WEIGHT_MASK_SIZE_EXT      = GEO_WEIGHT_MASK_SIZE + GEO_TM_ADDED_WEIGHT_MASK_SIZE * 2;
static constexpr int DIMD_MAX_TEMP_SIZE            = 4;
static constexpr int TOTAL_GEO_NUM_BLD             = 6;   // GPM 0~4, SGPM 1~5
#define GET_SGPM_BLD_IDX(a, b) \
  (std::min(a, b) <= 4 ? 1 : std::min(a, b) <= 8 ? 2 : std::min(a, b) <= 16 ? 3 : std::min(a, b) <= 32 ? 4 : 5)

static constexpr int SBT_MAX_SIZE               = MAX_TB_SIZEY;   ///< maximum CU size for using SBT
static constexpr int SBT_NUM_SL                 = 10;   ///< maximum number of historical PU decision saved for a CU
static constexpr int SBT_NUM_RDO                = 2;   ///< maximum number of SBT mode tried for a PU
static constexpr int SBT_QUAD_MIN_BLOCK_SIZE    = 8;
static constexpr int SBT_QUARTER_MIN_BLOCK_SIZE = 16;

static constexpr int IBC_MAX_CU_SIZE                      = 64;
static constexpr int IBC_NUM_CANDIDATES                   = 64;   ///< Maximum number of candidates to store/test
static constexpr int CHROMA_REFINEMENT_CANDIDATES         = 8;   /// 8 candidates BV to choose from
static constexpr int IBC_FAST_METHOD_NOINTRA_IBCCBF0      = 0x01;
static constexpr int IBC_FAST_METHOD_BUFFERBV             = 0X02;
static constexpr int IBC_FAST_METHOD_ADAPTIVE_SEARCHRANGE = 0X04;
static constexpr int IBC_FAST_METHOD_NONSCC               = 0X08;
static constexpr int IBC_NONSCC_ENC_RD_NZ_COUNT           = 3;
static constexpr int IBC_SEARCH_RANGE                     = 32;
static constexpr int IBC_SUBPEL_AMVR_MODE_FOR_ZERO_MVD    = IMV_OFF;
static constexpr int MV_EXPONENT_BITCOUNT                 = 4;
static constexpr int MV_MANTISSA_BITCOUNT                 = 6;
static constexpr int MV_MANTISSA_UPPER_LIMIT              = ((1 << (MV_MANTISSA_BITCOUNT - 1)) - 1);
static constexpr int MV_MANTISSA_LIMIT                    = (1 << (MV_MANTISSA_BITCOUNT - 1));
static constexpr int MV_EXPONENT_MASK                     = ((1 << MV_EXPONENT_BITCOUNT) - 1);

static constexpr int MV_BITS = 18;
static constexpr int MV_MAX  = (1 << (MV_BITS - 1)) - 1;
static constexpr int MV_MIN  = -(1 << (MV_BITS - 1));
static constexpr int MVD_MAX = MV_MAX;
static constexpr int MVD_MIN = MV_MIN;

static constexpr int     PIC_ANALYZE_CW_BINS = 32;
static constexpr int     PIC_CODE_CW_BINS    = 16;
static constexpr int     LMCS_SEG_NUM        = 32;
static constexpr int     FP_PREC             = 11;
static constexpr int     CSCALE_FP_PREC      = 11;
static constexpr uint8_t MIP_SHIFT_MATRIX    = 6;
static constexpr uint8_t MIP_OFFSET_MATRIX   = 32;

static constexpr int          LOG2_PALETTE_CG_SIZE    = 4;
static constexpr int          RUN_IDX_THRE            = 4;
static constexpr int          MAX_CU_BLKSIZE_PLT      = 64;
static constexpr int          NUM_TRELLIS_STATE       = 3;
static constexpr double       ENC_CHROMA_WEIGHTING    = 0.8;
static constexpr int          MAXPLTPREDSIZE          = 63;
static constexpr int          MAXPLTSIZE              = 31;
static constexpr int          MAXPLTPREDSIZE_DUALTREE = 31;
static constexpr int          MAXPLTSIZE_DUALTREE     = 15;
static constexpr double       PLT_CHROMA_WEIGHTING    = 0.8;
static constexpr int          PLT_ENCBITDEPTH         = 8;
static constexpr int          PLT_FAST_RATIO          = 100;
static constexpr int          ENC_PPS_ID_RPR          = 3;
static constexpr int          ENC_PPS_ID_RPR2         = 5;
static constexpr int          ENC_PPS_ID_RPR3         = 7;
static constexpr int          NUM_RPR_PPS             = 4;
static constexpr int          RPR_PPS_ID[NUM_RPR_PPS] = { 0, ENC_PPS_ID_RPR3, ENC_PPS_ID_RPR2, ENC_PPS_ID_RPR };
static constexpr int          MAX_SCALING_RATIO       = 2;   // max downsampling ratio for RPR
static constexpr ScalingRatio SCALE_1X = { 1 << ScalingRatio::BITS, 1 << ScalingRatio::BITS };   // scale ratio 1x

static constexpr int SIGN_PRED_MAX_NUM           = 8;
static constexpr int SIGN_PRED_MAX_NUM_NST       = std::min<int>(4, SIGN_PRED_MAX_NUM);
static constexpr int LOG2_SIGN_PRED_MAX_BS       = 7;
static constexpr int SIGN_PRED_MAX_BS            = 1 << LOG2_SIGN_PRED_MAX_BS;
static constexpr int SIGN_PRED_MAX_BUF_SIZE      = SIGN_PRED_MAX_BS >> 1;
static constexpr int SIGN_PRED_MAX_BS_INTRA      = std::min<int>(32, SIGN_PRED_MAX_BS);
static constexpr int SIGN_PRED_MAX_BS_INTER      = std::min<int>(128, SIGN_PRED_MAX_BS);
static constexpr int MAX_LOG2_SIGN_PRED_SIZE     = 5;
static constexpr int MAX_LOG2_SIGN_PRED_SIZE_NST = std::min<int>(2, MAX_LOG2_SIGN_PRED_SIZE);
static constexpr int SIGN_PRED_RESIDUAL_BITS     = 8;   ///< not configurable

static constexpr int MAX_TSRC_RICE            = 8;   ///< Maximum supported TSRC Rice parameter
static constexpr int MIN_TSRC_RICE            = 1;   ///< Minimum supported TSRC Rice parameter
static constexpr int MAX_CTI_LUT_SIZE         = 64;   ///< Maximum colour transform LUT size for CTI SEI
static constexpr int MAX_NUM_INTENSITIES      = 256;   ///< Maximum number of intensity intervals supported in FGC SEI
static constexpr int MAX_NUM_MODEL_VALUES     = 6;   ///< Maximum number of model values supported in FGC SEI
static constexpr int MAX_ALLOWED_MODEL_VALUES = 3;
static constexpr int MAX_ALLOWED_COMP_MODEL_PAIRS = 10;
static constexpr int MAX_STANDARD_DEVIATION =
  255;   // for 8-bit format; for higher bit depths, internal scaling is performed
static constexpr int DATA_BASE_SIZE = 64;
static constexpr int BLK_8          = 8;
static constexpr int BLK_16         = 16;
static constexpr int BLK_32         = 32;
static constexpr int BIT_DEPTH_8    = 8;

static constexpr int MSE_WEIGHT_FRAC_BITS = 16;
static constexpr int MSE_WEIGHT_ONE       = 1 << MSE_WEIGHT_FRAC_BITS;

static constexpr int CBF_MASK_CB   = 2;
static constexpr int CBF_MASK_CR   = 1;
static constexpr int CBF_MASK_CBCR = CBF_MASK_CB | CBF_MASK_CR;

static const int MC_PAD_SIZE      = 16;
static const int PAD_MORE_TL      = 1;
static const int EXT_PICTURE_SIZE = PIC_MARGIN + MC_PAD_SIZE;

static constexpr auto NADISTANCE_LEVEL    = 4;
static constexpr auto AFF_PARA_STORE_BITS = 16;
static constexpr auto AFF_PARA_SHIFT      = 0;

static constexpr int TEMP_CABAC_BUFFER_SIZE         = 5;
static constexpr int ADJUSTMENT_RANGE               = 7;
static constexpr int CABAC_SPATIAL_MAX_BINS         = 128;
static constexpr int CABAC_SPATIAL_MAX_BINS_PER_CTX = 4;

static constexpr int CCCM_NUM_PARAMS       = 7;
static constexpr int CCCM_NUM_PARAMS_MF1   = 10;
static constexpr int CCCM_NUM_PARAMS_MF2   = 11;
static constexpr int CCCM_NUM_PARAMS_MF3   = 11;
static constexpr int CCCM_NUM_PARAMS_NOSUB = 11;
static constexpr int CCCM_NUM_PARAMS_BVG   = 11;
static constexpr int CCCM_NUM_PARAMS_MAX   = CCCM_NUM_PARAMS_NOSUB;

static constexpr int CCCM_WINDOW_SIZE     = 6;
static constexpr int CCCM_FILTER_PADDING  = 1;   // E.g. 3x3 filter needs one padded sample
static constexpr int NUM_BVG_CCCM_CANDS   = 5;
static constexpr int BVG_CCCM_POS_OFFSET  = 4;
static constexpr int CCCM_MAX_REF_SAMPLES = 4 * (2 * CCCM_WINDOW_SIZE * (2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE));
static constexpr int CCCM_REF_SAMPLES_MAX = CCCM_MAX_REF_SAMPLES;

static constexpr int CCCM_LOC_SHIFT_MIN = 4;
static constexpr int CCCM_NUM_LUMA_BUFS = 5;
static constexpr int CCCM_MATRIX_BITS   = 22;
static constexpr int CCCM_DECIM_BITS    = 16;
static constexpr int CCCM_DECIM_ROUND   = (1 << (CCCM_DECIM_BITS - 1));

static constexpr int CCCM_MIN_PU_SIZE = 0;   // Set to 0 for no size restriction
static constexpr int CCCM_REF_LINES_ABOVE_CTU =
  0;   // Number of chroma lines allowed to be included in the reference area above the CTU (0: no restrictions)

static constexpr int ENCODER_INTRA_CHROMA_NUM_RDO = 8;
#if ENABLE_NNLF
static const uint32_t NNLF_UNIFIED_INFER_SIZE[3]  = { 64, 128, 256 };
static const uint32_t NNLF_UNIFIED_INFER_SIZE_EXT = 8;
static const uint32_t NNLF_UNIFIED_MAX_NUM_PRMS   = 2;
#endif

static constexpr int    LIC_MIN_CU_PIXELS = 32;   ///< smallest CU size (in terms of number of luma samples) of LIC
static constexpr double LIC_AMVP_SKIP_TH  = 1.2;   ///< Given a IMV mode, LIC is not tested if RD cost of non-LIC IMV
                                                  ///< AMVP mode is 1.2x worse than the current best RD cost
static constexpr int    NUM_LIC_ITERATION = 3;

static constexpr int SOLVER_NUM_PARAMS_MAX = std::max(CCCM_NUM_PARAMS_MAX, EIP_FILTER_TAP);

static constexpr uint32_t CCALF_CANDS_COEFF_NR                  = 8;
static constexpr int      CCALF_SMALL_TAB[CCALF_CANDS_COEFF_NR] = { 0, 1, 2, 4, 8, 16, 32, 64 };

#define MAX_CCSAO_SET_NUM 4
static const int MAX_CCSAO_CAND_POS_Y      = 9;
static const int MAX_CCSAO_CAND_POS_Y_BITS = 4;
static const int MAX_CCSAO_BAND_NUM_Y      = 16;
static const int MAX_CCSAO_BAND_NUM_Y_BITS = 4;
static const int MAX_CCSAO_BAND_NUM_U      = 4;
static const int MAX_CCSAO_BAND_NUM_U_BITS = 2;
static const int MAX_CCSAO_BAND_NUM_V      = 4;
static const int MAX_CCSAO_BAND_NUM_V_BITS = 2;
static const int MAX_CCSAO_CLASS_NUM       = 64;
static const int MAX_CCSAO_OFFSET_THR      = 15;
static const int MAX_CCSAO_FILTER_LENGTH   = 3;

static const int MAX_CCSAO_EDGE_CMP_BITS = 2;
static const int MAX_CCSAO_EDGE_IDC      = 2;
static const int MAX_CCSAO_EDGE_IDC_BITS = 1;
static const int MAX_CCSAO_EDGE_DIR      = 4;
static const int MAX_CCSAO_EDGE_DIR_BITS = 2;
static const int MAX_CCSAO_EDGE_THR      = 16;
static const int MAX_CCSAO_EDGE_THR_BITS = 4;
static const int MAX_CCSAO_EDGE_NUM      = 16;
static const int MAX_CCSAO_EDGE_NUM_BITS = 4;
static const int MAX_CCSAO_BAND_IDC      = 16;
static const int MAX_CCSAO_BAND_IDC_BITS = 4;
static const int MAX_CCSAO_PRV_NUM       = 16;
static const int MAX_CCSAO_PRV_NUM_BITS  = 4;

static const int MAX_CCP_CAND_LIST_SIZE = 12;
static const int MAX_NUM_HCCP_CANDS     = 6;

static const int MAX_CCP_FUSION_NUM = 12;

// define search area
static const int TMP_MIN_NUM_STEPS = 2;   // 0 equals stdpad, minimum size of triangle
static const int TMP_MAX_NUM_STEPS = 12;   // Limit the maximum number of steps executed, maximum size of triangle

static const int   TMP_NUM_AVG_CANDS = 4;   // Limit the number of averaged candidates
static const float TMP_ACS_FACTOR    = 1.5;   // adaptive candidate selection factor (needs to be <= 1)

// template size and target block size definition
static const int TMP_TEMPLATE_WIDTH  = 16;
static const int TMP_TEMPLATE_HEIGHT = 3;
static const int TMP_TW_MINUS_TBW_BY_TWO =
  2;   // defines how much smaller the target block is than the template on each side

static const int TMP_PADSIZE                     = 16;   // padding size for TM padding
static const int TMP_EARLY_TERMINATION_THRESHOLD = 128;   // threshold for early termination

struct lfCccmCand
{
  int8_t windowSize;
  int8_t modelType;
};
static constexpr int lfCccmMaxNumCands = 2;

// ====================================================================================================================
// SEI and related constants
// ====================================================================================================================

static constexpr uint32_t MAX_NNPFA_ID = 0xfffffffe;   // Maximum supported nnpfa_id
static constexpr uint32_t MAX_NNPFC_ID = 0xfffffffe;   // Maximum supported nnpfc_id
#if JVET_Z0120_SII_SEI_PROCESSING
static constexpr double SII_PF_W2 = 0.6;   // weight for current picture
static constexpr double SII_PF_W1 = 0.4;   // weight for previous picture , it must be equal to 1.0 - SII_PF_W2
#endif
// ====================================================================================================================
// Macro functions
// ====================================================================================================================

struct ClpRng
{
  int min { 0 };
  int max { 0 };
  int bd { 0 };
  int n { 0 };
};

struct ClpRngs
{
  ClpRng comp[MAX_NUM_COMP];   ///< the bit depth as indicated in the SPS
  bool   used;
  bool   chroma;
};

template<typename T> inline T Clip3(const T minVal, const T maxVal, const T a)
{
  return std::min<T>(std::max<T>(minVal, a), maxVal);
}   ///< general min/max clip
template<typename T> inline T ClipBD(const T x, const int bitDepth) { return Clip3(T(0), T((1 << bitDepth) - 1), x); }
template<typename T> inline T ClipPel(const T a, const ClpRng &clpRng)
{
  return std::min<T>(std::max<T>(clpRng.min, a), clpRng.max);
}   ///< clip reconstruction

extern MsgLevel g_verbosity;

#include <stdarg.h>
inline void msg(MsgLevel level, const char *fmt, ...)
{
  if (g_verbosity >= level)
  {
    va_list args;
    va_start(args, fmt);
    vfprintf(level == ERROR ? stderr : stdout, fmt, args);
    va_end(args);
  }
}

constexpr size_t MEMORY_ALIGN_DEF_SIZE = 32;   // for use with avx2 (256 bit)
constexpr size_t CACHE_MEM_ALIGN_SIZE  = 1024;

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
constexpr size_t MALLOC_ALIGN_SIZE = CACHE_MEM_ALIGN_SIZE;
#else
constexpr size_t MALLOC_ALIGN_SIZE = MEMORY_ALIGN_DEF_SIZE;
#endif

#if defined _MSC_VER || defined __MINGW64_VERSION_MAJOR
// Some compilers don't support std::aligned_alloc even though it is standardized
#define xMalloc(type, len) _aligned_malloc(sizeof(type) * (len), MEMORY_ALIGN_DEF_SIZE)
#define xFree(ptr)         _aligned_free(ptr)
#else
template<typename T> inline void *alignedAllocAdjustSize(size_t len)
{
  // std::aligned_alloc requires that the size parameter is an integral multiple of the alignment
  const size_t numBytes = (sizeof(T) * len + MALLOC_ALIGN_SIZE - 1) & ~(MALLOC_ALIGN_SIZE - 1);
  return std::aligned_alloc(MALLOC_ALIGN_SIZE, numBytes);
}
#define xMalloc(type, len) alignedAllocAdjustSize<type>(len)
#define xFree(ptr)         std::free(ptr)
#endif

#if defined _MSC_VER
#define ALIGN_DATA(nBytes, v) __declspec(align(nBytes)) v
#else
// #elif defined linux
#define ALIGN_DATA(nBytes, v) v __attribute__((aligned(nBytes)))
// #else
// #error unknown platform
#endif

#if defined(__GNUC__) && !defined(__clang__)
#define GCC_VERSION_AT_LEAST(x, y) (__GNUC__ > x || __GNUC__ == x && __GNUC_MINOR__ >= y)
#else
#define GCC_VERSION_AT_LEAST(x, y) 0
#endif

#ifdef __clang__
#define CLANG_VERSION_AT_LEAST(x, y) (__clang_major__ > x || __clang_major__ == x && __clang_minor__ >= y)
#else
#define CLANG_VERSION_AT_LEAST(x, y) 0
#endif

#ifdef __GNUC__
#define ALWAYS_INLINE __attribute__((always_inline)) inline
#elif defined _MSC_VER
#define ALWAYS_INLINE __forceinline
#else
#define ALWAYS_INLINE
#endif

// macros to selectively disable some usually useful warnings
#if defined __GNUC__ && !defined __clang__
#define GCC_WARNING_RESET _Pragma("GCC diagnostic pop");

#define GCC_EXTRA_WARNING_switch_enum \
  _Pragma("GCC diagnostic push");     \
  _Pragma("GCC diagnostic error \"-Wswitch-enum\"");
#else
#define GCC_WARNING_RESET

#define GCC_EXTRA_WARNING_switch_enum
#endif

#if __GNUC__ >= 8 && !defined __clang__
#define GCC_WARNING_DISABLE_maybe_uninitialized \
  _Pragma("GCC diagnostic push");               \
  _Pragma("GCC diagnostic ignored \"-Wmaybe-uninitialized\"");
#define GCC_WARNING_DISABLE_class_memaccess \
  _Pragma("GCC diagnostic push");           \
  _Pragma("GCC diagnostic ignored \"-Wclass-memaccess\"");
#else
#define GCC_WARNING_DISABLE_maybe_uninitialized
#define GCC_WARNING_DISABLE_class_memaccess
#endif

#if ENABLE_SIMD_OPT
#ifdef TARGET_SIMD_X86
typedef enum
{
  SCALAR = 0,
  SSE41,
  SSE42,
  AVX,
  AVX2,
  AVX512
} X86_VEXT;

X86_VEXT    read_x86_extension_flags(const std::string &extStrId = std::string());
const char *read_x86_extension(const std::string &extStrId);
#endif   // TARGET_SIMD_X86
#endif   // ENABLE_SIMD_OPT

// CASE-BREAK for breakpoints
#if defined(_MSC_VER) && defined(_DEBUG)
#define _CASE(_x) if (_x)
#define _BREAK \
  while (0)    \
    ;
#else
#define _CASE(...)
#define _BREAK
#endif
#define _AREA_AT(_a, _x, _y, _w, _h)      (_a.x == _x && _a.y == _y && _a.width == _w && _a.height == _h)
#define _AREA_CONTAINS(_a, _x, _y)        (_a.contains(Position { _x, _y }))
#define _UNIT_AREA_AT(_a, _x, _y, _w, _h) (_a.lx() == _x && _a.ly() == _y && _a.lwidth() == _w && _a.lheight() == _h)

// Make enum and strings macros, used for TimeProfiler and DTrace
#define MAKE_ENUM(VAR)    VAR,
#define MAKE_STRINGS(VAR) #VAR,
#define MAKE_ENUM_AND_STRINGS(source, enumName, enumStringName) \
  enum enumName                                                 \
  {                                                             \
    source(MAKE_ENUM)                                           \
  };                                                            \
  const char *const enumStringName[] = { source(MAKE_STRINGS) };

//! \}

#endif   // end of #ifndef  __COMMONDEF__
