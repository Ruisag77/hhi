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

/** \file     Rom.h
    \brief    global variables & functions (header)
*/

#ifndef __ROM__
#define __ROM__

#include "CommonDef.h"
#include "Common.h"

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <unordered_map>

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Initialize / destroy functions
// ====================================================================================================================

void initROM();
void destroyROM();

// ====================================================================================================================
// Data structure related table & variable
// ====================================================================================================================
extern int g_gradDivTable[16];

// flexible conversion from relative to absolute index
struct ScanElement
{
  uint32_t idx;
  uint16_t x;
  uint16_t y;
};

extern Size g_log2TxSubblockSize[MAX_CU_DEPTH + 1][MAX_CU_DEPTH + 1];

// Using directive needed because of a bug in clang-format that identifies this file as objective C without it.
using ScanElementPointer      = ScanElement *;
using ScanElementPointerArray = ScanElementPointer[MAX_CU_SIZE / 2 + 1][MAX_CU_SIZE / 2 + 1];
extern EnumArray<ScanElementPointerArray, CoeffScanType> g_scanOrder[SCAN_NUMBER_OF_GROUP_TYPES];

extern ScanElement g_coefTopLeftDiagScan8x8[MAX_CU_DEPTH + 1][64];
extern ScanElement g_coefTopLeftDiagScan16x16[MAX_CU_DEPTH + 1][256];

extern const int g_quantScales[2 /*0=4^n blocks, 1=2*4^n blocks*/][SCALING_LIST_REM_NUM];          // Q(QP%6)
extern const int g_invQuantScales[2 /*0=4^n blocks, 1=2*4^n blocks*/][SCALING_LIST_REM_NUM];          // IQ(QP%6)

static constexpr int NUM_TRANSFORM_MATRIX_SIZES = 8;

static const int g_transformMatrixShift[TRANSFORM_NUMBER_OF_DIRECTIONS] = { 6, 6 };

// ====================================================================================================================
// Scanning order & context mapping table
// ====================================================================================================================
extern const std::array<TCoeff, 4> g_riceThreshold;

extern const std::array<uint8_t, g_riceThreshold.size() + 1> g_riceShift;

extern const uint32_t g_groupIdx[MAX_TB_SIZEY];
extern const uint32_t g_minInGroup[LAST_SIGNIFICANT_GROUPS];
extern const uint64_t g_stateTransTab[3];
extern const uint32_t g_goRiceParsCoeff[32];
extern const uint32_t g_goRiceParsCoeffGTN[GTN_MAXSUM];
inline uint32_t       g_goRicePosCoeff0(int st, uint32_t ricePar) { return (1 + (st & 1)) << ricePar; }

// ====================================================================================================================
// Intra prediction table
// ====================================================================================================================

extern const int                    g_sizeData[PDP_NUM_SIZES][11];
extern std::unordered_map<int, int> g_size;

extern int16_t *g_pdpFilters[PDP_NUM_GROUPS][PDP_NUM_SIZES];
extern int16_t *g_pdpFiltersMip[PDP_NUM_GROUPS][PDP_NUM_SIZES];
extern int      g_validSizePdp[PDP_NUM_SIZES];
extern int      g_validSizeMip[PDP_NUM_SIZES];

extern const int16_t g_weights4x4[11][16][36];
extern const int16_t g_weightsShort4x4[8][16][20];
extern const int16_t g_weights4x8[11][32][52];
extern const int16_t g_weightsShort4x8[8][32][28];
extern const int16_t g_weights8x4[11][32][52];
extern const int16_t g_weightsShort8x4[8][32][28];
extern const int16_t g_weights8x8[11][64][68];
extern const int16_t g_weightsShort8x8[8][64][36];
extern const int16_t g_weights4x16[11][64][84];
extern const int16_t g_weightsShort4x16[8][64][44];
extern const int16_t g_weights16x4[11][64][84];
extern const int16_t g_weightsShort16x4[8][64][44];
extern const int16_t g_weights8x16[11][128][100];
extern const int16_t g_weightsShort8x16[8][128][52];
extern const int16_t g_weights16x8[11][128][100];
extern const int16_t g_weightsShort16x8[8][128][52];
extern const int16_t g_weights16x16[11][256][132];
extern const int16_t g_weightsShort16x16[8][256][68];
extern const int16_t g_weights16x32[7][256][97];
extern const int16_t g_weightsShort16x32[4][256][49];
extern const int16_t g_weights32x16[7][256][97];
extern const int16_t g_weightsShort32x16[4][256][49];
extern const int16_t g_weights32x32[7][256][129];
extern const int16_t g_weightsShort32x32[4][256][65];
extern const int16_t g_filterDataPdpMip[];

void createPdpFilters();
void destroyPdpFilters();
void createMipFilters();
void destroyMipFilters();

extern const uint8_t g_intraModeNumFastUseMPM2D[7 - MIN_CU_LOG2 + 1][7 - MIN_CU_LOG2 + 1];

extern const uint8_t g_chroma422IntraAngleMappingTable[NUM_INTRA_MODE];

// ====================================================================================================================
// Mode-Dependent DST Matrices
// ====================================================================================================================

extern const TMatrixCoeff g_trCoreDCT2P2[TRANSFORM_NUMBER_OF_DIRECTIONS][2][2];
extern const TMatrixCoeff g_trCoreDCT2P4[TRANSFORM_NUMBER_OF_DIRECTIONS][4][4];
extern const TMatrixCoeff g_trCoreDCT2P8[TRANSFORM_NUMBER_OF_DIRECTIONS][8][8];
extern const TMatrixCoeff g_trCoreDCT2P16[TRANSFORM_NUMBER_OF_DIRECTIONS][16][16];
extern const TMatrixCoeff g_trCoreDCT2P32[TRANSFORM_NUMBER_OF_DIRECTIONS][32][32];
extern const TMatrixCoeff g_trCoreDCT2P64[TRANSFORM_NUMBER_OF_DIRECTIONS][64][64];
extern const TMatrixCoeff g_trCoreDCT2P128[TRANSFORM_NUMBER_OF_DIRECTIONS][128][128];

extern const TMatrixCoeff g_trCoreDCT8P4[TRANSFORM_NUMBER_OF_DIRECTIONS][4][4];
extern const TMatrixCoeff g_trCoreDCT8P8[TRANSFORM_NUMBER_OF_DIRECTIONS][8][8];
extern const TMatrixCoeff g_trCoreDCT8P16[TRANSFORM_NUMBER_OF_DIRECTIONS][16][16];
extern const TMatrixCoeff g_trCoreDCT8P32[TRANSFORM_NUMBER_OF_DIRECTIONS][32][32];
extern const TMatrixCoeff g_trCoreDCT8P64[TRANSFORM_NUMBER_OF_DIRECTIONS][64][64];
extern const TMatrixCoeff g_trCoreDCT8P128[TRANSFORM_NUMBER_OF_DIRECTIONS][128][128];

extern const TMatrixCoeff g_trCoreDST7P4[TRANSFORM_NUMBER_OF_DIRECTIONS][4][4];
extern const TMatrixCoeff g_trCoreDST7P8[TRANSFORM_NUMBER_OF_DIRECTIONS][8][8];
extern const TMatrixCoeff g_trCoreDST7P16[TRANSFORM_NUMBER_OF_DIRECTIONS][16][16];
extern const TMatrixCoeff g_trCoreDST7P32[TRANSFORM_NUMBER_OF_DIRECTIONS][32][32];
extern const TMatrixCoeff g_trCoreDST7P64[TRANSFORM_NUMBER_OF_DIRECTIONS][64][64];
extern const TMatrixCoeff g_trCoreDST7P128[TRANSFORM_NUMBER_OF_DIRECTIONS][128][128];

extern TMatrixCoeff g_trCoreDCT2P256[256][256];
extern TMatrixCoeff g_trCoreDCT8P256[256][256];
extern TMatrixCoeff g_trCoreDST7P256[256][256];

extern TMatrixCoeff g_aiTr2[to_underlying(TransType::NUM)][2][2];
extern TMatrixCoeff g_aiTr4[to_underlying(TransType::NUM)][4][4];
extern TMatrixCoeff g_aiTr8[to_underlying(TransType::NUM)][8][8];
extern TMatrixCoeff g_aiTr16[to_underlying(TransType::NUM)][16][16];
extern TMatrixCoeff g_aiTr32[to_underlying(TransType::NUM)][32][32];
extern TMatrixCoeff g_aiTr64[to_underlying(TransType::NUM)][64][64];
extern TMatrixCoeff g_aiTr128[to_underlying(TransType::NUM)][128][128];
extern TMatrixCoeff g_aiTr256[to_underlying(TransType::NUM)][256][256];

extern const uint8_t   g_aucIpmToTrSetModDimd[3][12][67];
extern const uint8_t   g_aucIpmToTrSetModTimd[3][10][67];
extern const uint8_t   g_aucIpmToTrSetModMip[3][12][67];
extern const uint8_t   g_aucIpmToTrSetModSgpm[3][11][67];
extern const uint8_t   g_aucImplicitToTrSet[16][35];
extern const TransType g_aucImplicitTrIdxToTr[36][2];

extern const uint8_t   g_aucIpmToTrSet[16][36];
extern const uint8_t   g_aucTrSet[80][6];
extern const int8_t    g_aiIdLut[3][3];
extern const TransType g_aucTrIdxToTr[25][2];

extern const int8_t g_fwdLfnst16x16[35][3][L16H][L16W];
extern const int8_t g_fwdLfnst8x8[35][3][L8H][L8W];
extern const int8_t g_fwdLfnst4x4[35][3][16][16];

extern int8_t g_invLfnst16x16[35][3][L16W][L16H];
extern int8_t g_invLfnst8x8[35][3][L8W][L8H];
extern int8_t g_invLfnst4x4[35][3][16][16];

extern const uint8_t g_lfnstLut[NUM_LFNST_INTRA_MODES];
extern const uint8_t g_nsptLut[NUM_LFNST_INTRA_MODES];

extern const int8_t g_nspt4x4[NUM_NSPT_CLUSTERS_4x4][16][16];
extern const int8_t g_nspt4x8[NUM_NSPT_CLUSTERS_4x8][20][32];
extern const int8_t g_nspt8x4[NUM_NSPT_CLUSTERS_8x4][20][32];
extern const int8_t g_nspt8x8[NUM_NSPT_CLUSTERS_8x8][32][64];
extern const int8_t g_nspt4x16[NUM_NSPT_CLUSTERS_4x16][24][64];
extern const int8_t g_nspt16x4[NUM_NSPT_CLUSTERS_16x4][24][64];
extern const int8_t g_nspt8x16[NUM_NSPT_CLUSTERS_8x16][40][128];
extern const int8_t g_nspt16x8[NUM_NSPT_CLUSTERS_16x8][40][128];
extern const int8_t g_nspt4x32[NUM_NSPT_CLUSTERS_4x32][20][128];
extern const int8_t g_nspt32x4[NUM_NSPT_CLUSTERS_32x4][20][128];
extern const int8_t g_nspt8x32[NUM_NSPT_CLUSTERS_8x32][24][256];
extern const int8_t g_nspt32x8[NUM_NSPT_CLUSTERS_32x8][24][256];

extern const uint8_t g_nsptIdx4x4[35][NUM_NSPT_BLOCK_TYPES][3];
extern const uint8_t g_nsptIdx4x8[35][NUM_NSPT_BLOCK_TYPES][3];
extern const uint8_t g_nsptIdx8x4[35][NUM_NSPT_BLOCK_TYPES][3];
extern const uint8_t g_nsptIdx8x8[35][NUM_NSPT_BLOCK_TYPES][3];
extern const uint8_t g_nsptIdx4x16[35][NUM_NSPT_BLOCK_TYPES][3];
extern const uint8_t g_nsptIdx16x4[35][NUM_NSPT_BLOCK_TYPES][3];
extern const uint8_t g_nsptIdx8x16[35][NUM_NSPT_BLOCK_TYPES][3];
extern const uint8_t g_nsptIdx16x8[35][NUM_NSPT_BLOCK_TYPES][3];
extern const uint8_t g_nsptIdx4x32[35][NUM_NSPT_BLOCK_TYPES][3];
extern const uint8_t g_nsptIdx32x4[35][NUM_NSPT_BLOCK_TYPES][3];
extern const uint8_t g_nsptIdx8x32[35][NUM_NSPT_BLOCK_TYPES][3];
extern const uint8_t g_nsptIdx32x8[35][NUM_NSPT_BLOCK_TYPES][3];

// ====================================================================================================================
// Misc.
// ====================================================================================================================
extern int g_bdofWeight[1600];
extern int g_weight8x8[144];

extern SizeIndexInfo *gp_sizeIdxInfo;

extern const int g_ictModes[2][4];

extern UnitScale g_miScaling; // scaling object for motion scaling

/*! Sophisticated Trace-logging */
#if ENABLE_TRACING
#include "dtrace.h"
extern CDTrace *g_trace_ctx;
#endif
#include "TimeProfiler.h"
#if ENABLE_TIME_PROFILING
extern TimeProfiler               *g_timeProfiler;
extern std::vector<TimeProfiler *> g_allTimeProfilers;
#endif

const char *nalUnitTypeToString(NalUnitType type);

extern const char *matrixType[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM];
extern const char *matrixTypeDc[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM];

extern const int g_quantTSDefault4x4[4 * 4];
extern const int g_quantIntraDefault8x8[8 * 8];
extern const int g_quantInterDefault8x8[8 * 8];

extern const uint32_t g_scalingListSize[SCALING_LIST_SIZE_NUM];
extern const uint32_t g_scalingListSizeX[SCALING_LIST_SIZE_NUM];
extern const uint32_t g_scalingListId[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM];

extern MsgLevel g_verbosity;

extern const int8_t g_BcwWeightBase;
extern const int8_t g_BcwLog2WeightBase;
extern const int8_t g_BcwWeights[BCW_NUM];
extern const int8_t g_BcwSearchOrder[BCW_NUM];
extern int8_t       g_BcwCodingOrder[BCW_NUM];
extern int8_t       g_BcwParsingOrder[BCW_NUM];

class CodingStructure;
int8_t   getBcwWeight(uint8_t bcwIdx, uint8_t refFrameList);
void     resetBcwCodingOrder(bool runDecoding, const CodingStructure &cs);
uint32_t deriveWeightIdxBits(uint8_t bcwIdx);

extern const int8_t g_ccSaoCandPosX[MAX_NUM_LUMA_COMP][MAX_CCSAO_CAND_POS_Y];
extern const int8_t g_ccSaoCandPosY[MAX_NUM_LUMA_COMP][MAX_CCSAO_CAND_POS_Y];
extern const int8_t g_ccSaoEdgePosX[MAX_CCSAO_EDGE_DIR][2];
extern const int8_t g_ccSaoEdgePosY[MAX_CCSAO_EDGE_DIR][2];
extern const short  g_ccSaoEdgeThr[MAX_CCSAO_EDGE_IDC][MAX_CCSAO_EDGE_THR];
extern const int8_t g_ccSaoEdgeNum[MAX_CCSAO_EDGE_IDC][2];
extern const int8_t g_ccSaoBandTab[MAX_CCSAO_BAND_IDC][2];

//! \}

extern bool g_mctsDecCheckEnabled;

extern uint16_t g_paletteQuant[57];
extern uint8_t  g_paletteRunTopLut[5];
extern uint8_t  g_paletteRunLeftLut[5];

static constexpr int IBC_BUFFER_SIZE = 256 * 128 * 4;

void initGeoTemplate();

struct GeoParam
{
  uint8_t angleIdx : GEO_LOG2_NUM_ANGLES;
  uint8_t distanceIdx : GEO_LOG2_NUM_DISTANCES;
};

extern GeoParam g_geoParams[GEO_NUM_PARTITION_MODE];
extern int16_t *g_globalGeoWeights[TOTAL_GEO_NUM_BLD][GEO_NUM_PRESTORED_MASK];
extern int      g_bld2Width[TOTAL_GEO_NUM_BLD];
extern Pel     *g_globalGeoEncSADmask[GEO_NUM_PRESTORED_MASK];
extern int16_t  g_weightOffset[GEO_NUM_PARTITION_MODE][GEO_NUM_CU_SIZE][GEO_NUM_CU_SIZE][2];
extern int8_t   g_angle2mask[GEO_NUM_ANGLES];
extern int8_t   g_dis[GEO_NUM_ANGLES];
extern int8_t   g_angle2mirror[GEO_NUM_ANGLES];
extern int8_t   g_geoAngle2IntraAng[GEO_NUM_ANGLES];
extern uint8_t  g_geoTmShape[2][GEO_NUM_ANGLES];
extern Pel     *g_globalGeoWeightsTpl[GEO_NUM_PRESTORED_MASK];
extern int16_t  g_weightOffsetEx[GEO_NUM_PARTITION_MODE][GEO_NUM_CU_SIZE_EX][GEO_NUM_CU_SIZE_EX][2];
extern int8_t   g_sgpm_splitDir[GEO_NUM_PARTITION_MODE];

extern const Position g_eipFilter[NUM_EIP_FILTER_SHAPE][EIP_FILTER_TAP - 1];
extern const EipInfo  g_eipInfoLutMultiModelOff[4][4][NUM_DERIVED_EIP];
extern const EipInfo  g_eipInfoLutMultiModelOnTrue[4][4][NUM_DERIVED_EIP];
extern const EipInfo  g_eipInfoLutMultiModelOnFalse[4][4][NUM_DERIVED_EIP];

static constexpr int NUM_TRANSFORMS_SIGN_PRED = to_underlying(TransType::NUM) * to_underlying(TransType::NUM) +
  NUM_LFNST_SETS * (NUM_LFNST_NUM_PER_SET - 1) * NUM_NSPT_BLOCK_TYPES * 2;
extern int8_t *g_signPredTemplate[LOG2_SIGN_PRED_MAX_BS - 1][LOG2_SIGN_PRED_MAX_BS - 1][NUM_TRANSFORMS_SIGN_PRED];

extern int g_rmvfMultApproxTbl[3 << sizeof(int64_t)];
#endif  //__TCOMROM__
