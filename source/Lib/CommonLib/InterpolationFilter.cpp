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

/**
 * \file
 * \brief Implementation of InterpolationFilter class
 */

// ====================================================================================================================
// Includes
// ====================================================================================================================

#include "Rom.h"
#include "InterpolationFilter.h"
#include "ChromaFormat.h"
#include "dtrace_next.h"

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
CacheModel *InterpolationFilter::m_cacheModel;
#endif
//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Tables
// ====================================================================================================================
// clang-format off

#if IF_12TAP
#else
const TFilterCoeff InterpolationFilter::m_affineLumaFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA] =
{
  { 0,   0,  64,   0,   0, 0, 0, 0 },
  { 1,  -3,  63,   4,  -2, 1, 0, 0 },
  { 1,  -5,  62,   8,  -3, 1, 0, 0 },
  { 2,  -8,  60,  13,  -4, 1, 0, 0 },

  { 3, -10,  58,  17,  -5, 1, 0, 0 },
  { 3, -11,  52,  26,  -8, 2, 0, 0 },
  { 2,  -9,  47,  31, -10, 3, 0, 0 },
  { 3, -11,  45,  34, -10, 3, 0, 0 },

  { 3, -11,  40,  40, -11, 3, 0, 0 },
  { 3, -10,  34,  45, -11, 3, 0, 0 },
  { 3, -10,  31,  47,  -9, 2, 0, 0 },
  { 2,  -8,  26,  52, -11, 3, 0, 0 },

  { 1,  -5,  17,  58, -10, 3, 0, 0 },
  { 1,  -4,  13,  60,  -8, 2, 0, 0 },
  { 1,  -3,   8,  62,  -5, 1, 0, 0 },
  { 1,  -2,   4,  63,  -3, 1, 0, 0 }
};
#endif

#if IF_12TAP
// from higher cut-off freq to lower cut-off freq.
const TFilterCoeff InterpolationFilter::m_lumaFilter12[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][12] =
{
    { 0,     0,     0,     0,     0,   256,     0,     0,     0,     0,     0,     0, },
    {-1,     2,    -3,     6,   -14,   254,    16,    -7,     4,    -2,     1,     0, },
    {-1,     3,    -7,    12,   -26,   249,    35,   -15,     8,    -4,     2,     0, },
    {-2,     5,    -9,    17,   -36,   241,    54,   -22,    12,    -6,     3,    -1, },
    {-2,     5,   -11,    21,   -43,   230,    75,   -29,    15,    -8,     4,    -1, },
    {-2,     6,   -13,    24,   -48,   216,    97,   -36,    19,   -10,     4,    -1, },
    {-2,     7,   -14,    25,   -51,   200,   119,   -42,    22,   -12,     5,    -1, },
    {-2,     7,   -14,    26,   -51,   181,   140,   -46,    24,   -13,     6,    -2, },
    {-2,     6,   -13,    25,   -50,   162,   162,   -50,    25,   -13,     6,    -2, },
    {-2,     6,   -13,    24,   -46,   140,   181,   -51,    26,   -14,     7,    -2, },
    {-1,     5,   -12,    22,   -42,   119,   200,   -51,    25,   -14,     7,    -2, },
    {-1,     4,   -10,    19,   -36,    97,   216,   -48,    24,   -13,     6,    -2, },
    {-1,     4,    -8,    15,   -29,    75,   230,   -43,    21,   -11,     5,    -2, },
    {-1,     3,    -6,    12,   -22,    54,   241,   -36,    17,    -9,     5,    -2, },
    { 0,     2,    -4,     8,   -15,    35,   249,   -26,    12,    -7,     3,    -1, },
    { 0,     1,    -2,     4,    -7,    16,   254,   -14,     6,    -3,     2,    -1, },
#if SIMD_4x4_12
    {  0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0  }  // dummy line for SIMD reading of 2 chunks for 12-tap filter not to address wrong memory //kolya
#endif
};
#endif

// This is used for affine
#if IF_12TAP
const TFilterCoeff InterpolationFilter::m_lumaFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][8] =
#else
const TFilterCoeff InterpolationFilter::m_lumaFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA] =
#endif
{
#if IF_12TAP
  {      0,     0,       0, 64 * 4,      0,       0,      0,      0 },
  {      0, 1 * 4,  -3 * 4, 63 * 4,  4 * 4,  -2 * 4,  1 * 4,      0 },
  { -1 * 4, 2 * 4,  -5 * 4, 62 * 4,  8 * 4,  -3 * 4,  1 * 4,      0 },
  { -1 * 4, 3 * 4,  -8 * 4, 60 * 4, 13 * 4,  -4 * 4,  1 * 4,      0 },
  { -1 * 4, 4 * 4, -10 * 4, 58 * 4, 17 * 4,  -5 * 4,  1 * 4,      0 },
  { -1 * 4, 4 * 4, -11 * 4, 52 * 4, 26 * 4,  -8 * 4,  3 * 4, -1 * 4 },
  { -1 * 4, 3 * 4,  -9 * 4, 47 * 4, 31 * 4, -10 * 4,  4 * 4, -1 * 4 },
  { -1 * 4, 4 * 4, -11 * 4, 45 * 4, 34 * 4, -10 * 4,  4 * 4, -1 * 4 },
  { -1 * 4, 4 * 4, -11 * 4, 40 * 4, 40 * 4, -11 * 4,  4 * 4, -1 * 4 },
  { -1 * 4, 4 * 4, -10 * 4, 34 * 4, 45 * 4, -11 * 4,  4 * 4, -1 * 4 },
  { -1 * 4, 4 * 4, -10 * 4, 31 * 4, 47 * 4,  -9 * 4,  3 * 4, -1 * 4 },
  { -1 * 4, 3 * 4,  -8 * 4, 26 * 4, 52 * 4, -11 * 4,  4 * 4, -1 * 4 },
  {      0, 1 * 4,  -5 * 4, 17 * 4, 58 * 4, -10 * 4,  4 * 4, -1 * 4 },
  {      0, 1 * 4,  -4 * 4, 13 * 4, 60 * 4,  -8 * 4,  3 * 4, -1 * 4 },
  {      0, 1 * 4,  -3 * 4,  8 * 4, 62 * 4,  -5 * 4,  2 * 4, -1 * 4 },
  {      0, 1 * 4,  -2 * 4,  4 * 4, 63 * 4,  -3 * 4,  1 * 4,      0 }
#else
  {  0, 0,   0, 64,  0,   0,  0,  0 },
  {  0, 1,  -3, 63,  4,  -2,  1,  0 },
  { -1, 2,  -5, 62,  8,  -3,  1,  0 },
  { -1, 3,  -8, 60, 13,  -4,  1,  0 },
  { -1, 4, -10, 58, 17,  -5,  1,  0 },
  { -1, 4, -11, 52, 26,  -8,  3, -1 },
  { -1, 3,  -9, 47, 31, -10,  4, -1 },
  { -1, 4, -11, 45, 34, -10,  4, -1 },
  { -1, 4, -11, 40, 40, -11,  4, -1 },
  { -1, 4, -10, 34, 45, -11,  4, -1 },
  { -1, 4, -10, 31, 47,  -9,  3, -1 },
  { -1, 3,  -8, 26, 52, -11,  4, -1 },
  {  0, 1,  -5, 17, 58, -10,  4, -1 },
  {  0, 1,  -4, 13, 60,  -8,  3, -1 },
  {  0, 1,  -3,  8, 62,  -5,  2, -1 },
  {  0, 1,  -2,  4, 63,  -3,  1,  0 }
#endif
};

// 1.5x
#if IF_12TAP
const TFilterCoeff InterpolationFilter::m_lumaFilterRPR1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][12] =
#else
const TFilterCoeff InterpolationFilter::m_lumaFilterRPR1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA] =
#endif
{
#if IF_12TAP
  { 0, 10, -12, -19, 70, 158, 70, -19, -12, 10, 0, 0, },
  { 0, 10, -9, -22, 62, 156, 79, -17, -13, 9, 1, 0, },
  { -1, 10, -8, -22, 54, 155, 87, -13, -16, 9, 2, -1, },
  { -2, 10, -5, -23, 46, 152, 95, -9, -18, 9, 2, -1, },
  { -2, 9, -4, -23, 39, 150, 103, -6, -20, 8, 4, -2, },
  { -2, 8, -1, -24, 33, 144, 111, -2, -20, 6, 5, -2, },
  { -3, 9, -1, -24, 26, 142, 117, 4, -22, 5, 5, -2, },
  { -3, 8, 0, -23, 20, 137, 123, 10, -23, 4, 5, -2, },
  { -2, 6, 3, -25, 16, 130, 130, 16, -25, 3, 6, -2, },
  { -2, 5, 4, -23, 10, 123, 137, 20, -23, 0, 8, -3, },
  { -2, 5, 5, -22, 4, 117, 142, 26, -24, -1, 9, -3, },
  { -2, 5, 6, -20, -2, 111, 144, 33, -24, -1, 8, -2, },
  { -2, 4, 8, -20, -6, 103, 150, 39, -23, -4, 9, -2, },
  { -1, 2, 9, -18, -9, 95, 152, 46, -23, -5, 10, -2, },
  { -1, 2, 9, -16, -13, 87, 155, 54, -22, -8, 10, -1, },
  { 0, 1, 9, -13, -17, 79, 156, 62, -22, -9, 10, 0, },
#if SIMD_4x4_12
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }  // dummy line for SIMD reading of 2 chunks for 12-tap filter not to address wrong memory
#endif
#else
  { -1, -5, 17, 42, 17, -5, -1,  0 },
  {  0, -5, 15, 41, 19, -5, -1,  0 },
  {  0, -5, 13, 40, 21, -4, -1,  0 },
  {  0, -5, 11, 39, 24, -4, -2,  1 },
  {  0, -5,  9, 38, 26, -3, -2,  1 },
  {  0, -5,  7, 38, 28, -2, -3,  1 },
  {  1, -5,  5, 36, 30, -1, -3,  1 },
  {  1, -4,  3, 35, 32,  0, -4,  1 },
  {  1, -4,  2, 33, 33,  2, -4,  1 },
  {  1, -4,  0, 32, 35,  3, -4,  1 },
  {  1, -3, -1, 30, 36,  5, -5,  1 },
  {  1, -3, -2, 28, 38,  7, -5,  0 },
  {  1, -2, -3, 26, 38,  9, -5,  0 },
  {  1, -2, -4, 24, 39, 11, -5,  0 },
  {  0, -1, -4, 21, 40, 13, -5,  0 },
  {  0, -1, -5, 19, 41, 15, -5,  0 }
#endif
};

// 2x
#if IF_12TAP
const TFilterCoeff InterpolationFilter::m_lumaFilterRPR2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][12] =
#else
const TFilterCoeff InterpolationFilter::m_lumaFilterRPR2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA] =
#endif
{
#if IF_12TAP
  {   4,  -6, -18,  12,  78, 116,  78,  12, -18,  -6,   4,   0 },
  {   4,  -6, -18,   8,  76, 116,  86,  14, -18,  -8,   2,   0 },
  {   4,  -4, -18,   4,  70, 116,  88,  18, -16,  -8,   2,   0 },
  {   2,  -4, -18,   2,  68, 116,  92,  22, -16, -10,   2,   0 },
  {   2,  -2, -16,  -2,  62, 114,  94,  26, -14, -10,   2,   0 },
  {   2,  -2, -16,  -4,  58, 112,  98,  30, -14, -12,   2,   2 },
  {   2,   0, -16,  -6,  52, 110, 102,  34, -14, -12,   2,   2 },
  {   2,   0, -14,  -8,  48, 108, 104,  38, -12, -14,   2,   2 },
  {   2,   0, -14, -10,  44, 106, 106,  44, -10, -14,   0,   2 },
  {   2,   2, -14, -12,  38, 104, 108,  48,  -8, -14,   0,   2 },
  {   2,   2, -12, -14,  34, 102, 110,  52,  -6, -16,   0,   2 },
  {   2,   2, -12, -14,  30,  98, 112,  58,  -4, -16,  -2,   2 },
  {   0,   2, -10, -14,  26,  94, 114,  62,  -2, -16,  -2,   2 },
  {   0,   2, -10, -16,  22,  92, 116,  68,   2, -18,  -4,   2 },
  {   0,   2,  -8, -16,  18,  88, 116,  70,   4, -18,  -4,   4 },
  {   0,   2,  -8, -18,  14,  86, 116,  76,   8, -18,  -6,   4 },
#if SIMD_4x4_12
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }  // dummy line for SIMD reading of 2 chunks for 12-tap filter not to address wrong memory
#endif
#else
  { -4,  2, 20, 28, 20,  2, -4,  0 },
  { -4,  0, 19, 29, 21,  5, -4, -2 },
  { -4, -1, 18, 29, 22,  6, -4, -2 },
  { -4, -1, 16, 29, 23,  7, -4, -2 },
  { -4, -1, 16, 28, 24,  7, -4, -2 },
  { -4, -1, 14, 28, 25,  8, -4, -2 },
  { -3, -3, 14, 27, 26,  9, -3, -3 },
  { -3, -1, 12, 28, 25, 10, -4, -3 },
  { -3, -3, 11, 27, 27, 11, -3, -3 },
  { -3, -4, 10, 25, 28, 12, -1, -3 },
  { -3, -3,  9, 26, 27, 14, -3, -3 },
  { -2, -4,  8, 25, 28, 14, -1, -4 },
  { -2, -4,  7, 24, 28, 16, -1, -4 },
  { -2, -4,  7, 23, 29, 16, -1, -4 },
  { -2, -4,  6, 22, 29, 18, -1, -4 },
  { -2, -4,  5, 21, 29, 19,  0, -4 }
#endif
};

// clang-format off
// 1.5x
#if IF_12TAP
const TFilterCoeff InterpolationFilter::m_affineLumaFilterRPR1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][12] =
#else
const TFilterCoeff InterpolationFilter::m_affineLumaFilterRPR1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA] =
#endif
{
#if IF_12TAP
  {   0,   6,   -10,   -19,    74,   154,    74,   -19,   -10,     6,     0,    0},
  {   0,   8,    -8,   -22,    66,   153,    82,   -18,   -13,     8,     0,    0},
  {   0,   7,    -6,   -24,    58,   152,    90,   -14,   -15,     8,     0,    0},
  {   0,   7,    -5,   -25,    51,   150,    97,   -11,   -17,     8,     1,    0},
  {   0,   6,    -2,   -26,    43,   147,   105,    -7,   -19,     7,     2,    0},
  {   0,   6,    -1,   -26,    35,   144,   112,    -2,   -21,     7,     2,    0},
  {   0,   5,     1,   -26,    28,   140,   118,     3,   -22,     6,     3,    0},
  {   0,   5,     2,   -26,    22,   135,   125,     9,   -24,     5,     3,    0},
  {   0,   4,     4,   -25,    15,   130,   130,    15,   -25,     4,     4,    0},
  {   0,   3,     5,   -24,     9,   125,   135,    22,   -26,     2,     5,    0},
  {   0,   3,     6,   -22,     3,   118,   140,    28,   -26,     1,     5,    0},
  {   0,   2,     7,   -21,    -2,   112,   144,    35,   -26,    -1,     6,    0},
  {   0,   2,     7,   -19,    -7,   105,   147,    43,   -26,    -2,     6,    0},
  {   0,   1,     8,   -17,   -11,    97,   150,    51,   -25,    -5,     7,    0},
  {   0,   0,     8,   -15,   -14,    90,   152,    58,   -24,    -6,     7,    0},
  {   0,   0,     8,   -13,   -18,    82,   153,    66,   -22,    -8,     8,    0},
#if SIMD_4x4_12
  {  0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0  }
#endif
#else
  {  -6, 17, 42, 17, -5, -1,  0, 0 },
  {  -5, 15, 41, 19, -5, -1,  0, 0 },
  {  -5, 13, 40, 21, -4, -1,  0, 0 },
  {  -5, 11, 39, 24, -4, -1,  0, 0 },
  {  -5,  9, 38, 26, -3, -1,  0, 0 },
  {  -5,  7, 38, 28, -2, -2,  0, 0 },
  {  -4,  5, 36, 30, -1, -2,  0, 0 },
  {  -3,  3, 35, 32,  0, -3,  0, 0 },
  {  -3,  2, 33, 33,  2, -3,  0, 0 },
  {  -3,  0, 32, 35,  3, -3,  0, 0 },
  {  -2, -1, 30, 36,  5, -4,  0, 0 },
  {  -2, -2, 28, 38,  7, -5,  0, 0 },
  {  -1, -3, 26, 38,  9, -5,  0, 0 },
  {  -1, -4, 24, 39, 11, -5,  0, 0 },
  {  -1, -4, 21, 40, 13, -5,  0, 0 },
  {  -1, -5, 19, 41, 15, -5,  0, 0 }
#endif
};

// 2x
#if IF_12TAP
const TFilterCoeff InterpolationFilter::m_affineLumaFilterRPR2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][12] =
#else
const TFilterCoeff InterpolationFilter::m_affineLumaFilterRPR2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA] =
#endif
{
#if IF_12TAP
  {   0,   -3,   -14,    10,    77,   116,    77,    10,   -14,    -3,     0,    0},
  {   0,   -4,   -15,     8,    73,   116,    82,    14,   -15,    -5,     2,    0},
  {   0,   -3,   -15,     5,    69,   115,    86,    18,   -15,    -6,     2,    0},
  {   0,   -2,   -15,     2,    65,   114,    89,    22,   -14,    -7,     2,    0},
  {   0,   -1,   -14,    -1,    60,   113,    93,    26,   -14,    -8,     2,    0},
  {   0,   -1,   -14,    -3,    56,   112,    96,    30,   -13,    -9,     2,    0},
  {   0,    0,   -13,    -5,    51,   110,   100,    34,   -12,   -10,     1,    0},
  {   0,    0,   -13,    -7,    47,   108,   103,    38,   -10,   -11,     1,    0},
  {   0,    1,   -12,    -9,    43,   105,   105,    43,    -9,   -12,     1,    0},
  {   0,    1,   -11,   -10,    38,   103,   108,    47,    -7,   -13,     0,    0},
  {   0,    1,   -10,   -12,    34,   100,   110,    51,    -5,   -13,     0,    0},
  {   0,    2,    -9,   -13,    30,    96,   112,    56,    -3,   -14,    -1,    0},
  {   0,    2,    -8,   -14,    26,    93,   113,    60,    -1,   -14,    -1,    0},
  {   0,    2,    -7,   -14,    22,    89,   114,    65,     2,   -15,    -2,    0},
  {   0,    2,    -6,   -15,    18,    86,   115,    69,     5,   -15,    -3,    0},
  {   0,    2,    -5,   -15,    14,    82,   116,    73,     8,   -15,    -4,    0},
#if SIMD_4x4_12
  {  0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0  }
#endif
#else
  {  -2, 20, 28, 20,  2, -4,  0, 0 },
  {  -4, 19, 29, 21,  5, -6,  0, 0 },
  {  -5, 18, 29, 22,  6, -6,  0, 0 },
  {  -5, 16, 29, 23,  7, -6,  0, 0 },
  {  -5, 16, 28, 24,  7, -6,  0, 0 },
  {  -5, 14, 28, 25,  8, -6,  0, 0 },
  {  -6, 14, 27, 26,  9, -6,  0, 0 },
  {  -4, 12, 28, 25, 10, -7,  0, 0 },
  {  -6, 11, 27, 27, 11, -6,  0, 0 },
  {  -7, 10, 25, 28, 12, -4,  0, 0 },
  {  -6,  9, 26, 27, 14, -6,  0, 0 },
  {  -6,  8, 25, 28, 14, -5,  0, 0 },
  {  -6,  7, 24, 28, 16, -5,  0, 0 },
  {  -6,  7, 23, 29, 16, -5,  0, 0 },
  {  -6,  6, 22, 29, 18, -5,  0, 0 },
  {  -6,  5, 21, 29, 19, -4,  0, 0 }
#endif
};

#if IF_12TAP
const TFilterCoeff InterpolationFilter::m_lumaAltHpelIFilter[8] = { 0 * 4, 3 * 4, 9 * 4, 20 * 4, 20 * 4, 9 * 4, 3 * 4, 0 * 4 };
#else
const TFilterCoeff InterpolationFilter::m_lumaAltHpelIFilter[8] = { 0, 3, 9, 20, 20, 9, 3, 0 };
#endif

#if JVET_Z0117_CHROMA_IF
const TFilterCoeff InterpolationFilter::m_chromaFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][NTAPS_CHROMA] =
{
    {0, 0, 256, 0, 0, 0},
    {1, -6, 256, 7, -2, 0},
    {2, -11, 253, 15, -4, 1},
    {3, -16, 251, 23, -6, 1},
    {4, -21, 248, 33, -10, 2},
    {5, -25, 244, 42, -12, 2},
    {7, -30, 239, 53, -17, 4},
    {7, -32, 234, 62, -19, 4},
    {8, -35, 227, 73, -22, 5},
    {9, -38, 220, 84, -26, 7},
    {10, -40, 213, 95, -29, 7},
    {10, -41, 204, 106, -31, 8},
    {10, -42, 196, 117, -34, 9},
    {10, -41, 187, 127, -35, 8},
    {11, -42, 177, 138, -38, 10},
    {10, -41, 168, 148, -39, 10},
    {10, -40, 158, 158, -40, 10},
    {10, -39, 148, 168, -41, 10},
    {10, -38, 138, 177, -42, 11},
    {8, -35, 127, 187, -41, 10},
    {9, -34, 117, 196, -42, 10},
    {8, -31, 106, 204, -41, 10},
    {7, -29, 95, 213, -40, 10},
    {7, -26, 84, 220, -38, 9},
    {5, -22, 73, 227, -35, 8},
    {4, -19, 62, 234, -32, 7},
    {4, -17, 53, 239, -30, 7},
    {2, -12, 42, 244, -25, 5},
    {2, -10, 33, 248, -21, 4},
    {1, -6, 23, 251, -16, 3},
    {1, -4, 15, 253, -11, 2},
    {0, -2, 7, 256, -6, 1},
    { 0, 0, 0, 0, 0, 0 }  // dummy line for SIMD reading of last chunk using 8 x int16_t load for 6-tap filter not to address wrong memory
};
const TFilterCoeff InterpolationFilter::m_chromaFilter4[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][4] =
#else
const TFilterCoeff InterpolationFilter::m_chromaFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA] =
#endif
{
#if IF_12TAP
  {  0 * 4, 64 * 4,  0 * 4,  0 * 4 },
  { -1 * 4, 63 * 4,  2 * 4,  0 * 4 },
  { -2 * 4, 62 * 4,  4 * 4,  0 * 4 },
  { -2 * 4, 60 * 4,  7 * 4, -1 * 4 },
  { -2 * 4, 58 * 4, 10 * 4, -2 * 4 },
  { -3 * 4, 57 * 4, 12 * 4, -2 * 4 },
  { -4 * 4, 56 * 4, 14 * 4, -2 * 4 },
  { -4 * 4, 55 * 4, 15 * 4, -2 * 4 },
  { -4 * 4, 54 * 4, 16 * 4, -2 * 4 },
  { -5 * 4, 53 * 4, 18 * 4, -2 * 4 },
  { -6 * 4, 52 * 4, 20 * 4, -2 * 4 },
  { -6 * 4, 49 * 4, 24 * 4, -3 * 4 },
  { -6 * 4, 46 * 4, 28 * 4, -4 * 4 },
  { -5 * 4, 44 * 4, 29 * 4, -4 * 4 },
  { -4 * 4, 42 * 4, 30 * 4, -4 * 4 },
  { -4 * 4, 39 * 4, 33 * 4, -4 * 4 },
  { -4 * 4, 36 * 4, 36 * 4, -4 * 4 },
  { -4 * 4, 33 * 4, 39 * 4, -4 * 4 },
  { -4 * 4, 30 * 4, 42 * 4, -4 * 4 },
  { -4 * 4, 29 * 4, 44 * 4, -5 * 4 },
  { -4 * 4, 28 * 4, 46 * 4, -6 * 4 },
  { -3 * 4, 24 * 4, 49 * 4, -6 * 4 },
  { -2 * 4, 20 * 4, 52 * 4, -6 * 4 },
  { -2 * 4, 18 * 4, 53 * 4, -5 * 4 },
  { -2 * 4, 16 * 4, 54 * 4, -4 * 4 },
  { -2 * 4, 15 * 4, 55 * 4, -4 * 4 },
  { -2 * 4, 14 * 4, 56 * 4, -4 * 4 },
  { -2 * 4, 12 * 4, 57 * 4, -3 * 4 },
  { -2 * 4, 10 * 4, 58 * 4, -2 * 4 },
  { -1 * 4,  7 * 4, 60 * 4, -2 * 4 },
  {  0 * 4,  4 * 4, 62 * 4, -2 * 4 },
  {  0 * 4,  2 * 4, 63 * 4, -1 * 4 },
#else
  {  0, 64,  0,  0 },
  { -1, 63,  2,  0 },
  { -2, 62,  4,  0 },
  { -2, 60,  7, -1 },
  { -2, 58, 10, -2 },
  { -3, 57, 12, -2 },
  { -4, 56, 14, -2 },
  { -4, 55, 15, -2 },
  { -4, 54, 16, -2 },
  { -5, 53, 18, -2 },
  { -6, 52, 20, -2 },
  { -6, 49, 24, -3 },
  { -6, 46, 28, -4 },
  { -5, 44, 29, -4 },
  { -4, 42, 30, -4 },
  { -4, 39, 33, -4 },
  { -4, 36, 36, -4 },
  { -4, 33, 39, -4 },
  { -4, 30, 42, -4 },
  { -4, 29, 44, -5 },
  { -4, 28, 46, -6 },
  { -3, 24, 49, -6 },
  { -2, 20, 52, -6 },
  { -2, 18, 53, -5 },
  { -2, 16, 54, -4 },
  { -2, 15, 55, -4 },
  { -2, 14, 56, -4 },
  { -2, 12, 57, -3 },
  { -2, 10, 58, -2 },
  { -1,  7, 60, -2 },
  {  0,  4, 62, -2 },
  {  0,  2, 63, -1 },
#endif
};

//1.5x
const TFilterCoeff InterpolationFilter::m_chromaFilterRPR1[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA] =
{
#if IF_12TAP
  { -14, 64, 156, 64, -14, 0 },
  { -13, 59, 155, 68, -12, -1 },
  { -13, 56, 154, 71, -11, -1 },
  { -13, 52, 153, 76, -10, -2 },
  { -12, 47, 153, 79, -9, -2 },
  { -13, 45, 151, 84, -8, -3 },
  { -13, 39, 154, 88, -9, -3 },
  { -14, 34, 156, 94, -10, -4 },
  { -15, 33, 152, 99, -7, -6 },
  { -15, 30, 150, 103, -5, -7 },
  { -14, 26, 148, 107, -4, -7 },
  { -12, 23, 143, 110, -1, -7 },
  { -13, 18, 145, 116, -2, -8 },
  { -10, 15, 139, 118, 1, -7 },
  { -15, 19, 134, 122, 9, -13 },
  { -14, 18, 129, 123, 13, -13 },
  { -14, 18, 124, 124, 18, -14 },
  { -13, 13, 123, 129, 18, -14 },
  { -13, 9, 122, 134, 19, -15 },
  { -7, 1, 118, 139, 15, -10 },
  { -8, -2, 116, 145, 18, -13 },
  { -7, -1, 110, 143, 23, -12 },
  { -7, -4, 107, 148, 26, -14 },
  { -7, -5, 103, 150, 30, -15 },
  { -6, -7, 99, 152, 33, -15 },
  { -4, -10, 94, 156, 34, -14 },
  { -3, -9, 88, 154, 39, -13 },
  { -3, -8, 84, 151, 45, -13 },
  { -2, -9, 79, 153, 47, -12 },
  { -2, -10, 76, 153, 52, -13 },
  { -1, -11, 71, 154, 56, -13 },
  { -1, -12, 68, 155, 59, -13 },
#else
  { 12, 40, 12,  0 },
  { 11, 40, 13,  0 },
  { 10, 40, 15, -1 },
  {  9, 40, 16, -1 },
  {  8, 40, 17, -1 },
  {  8, 39, 18, -1 },
  {  7, 39, 19, -1 },
  {  6, 38, 21, -1 },
  {  5, 38, 22, -1 },
  {  4, 38, 23, -1 },
  {  4, 37, 24, -1 },
  {  3, 36, 25,  0 },
  {  3, 35, 26,  0 },
  {  2, 34, 28,  0 },
  {  2, 33, 29,  0 },
  {  1, 33, 30,  0 },
  {  1, 31, 31,  1 },
  {  0, 30, 33,  1 },
  {  0, 29, 33,  2 },
  {  0, 28, 34,  2 },
  {  0, 26, 35,  3 },
  {  0, 25, 36,  3 },
  { -1, 24, 37,  4 },
  { -1, 23, 38,  4 },
  { -1, 22, 38,  5 },
  { -1, 21, 38,  6 },
  { -1, 19, 39,  7 },
  { -1, 18, 39,  8 },
  { -1, 17, 40,  8 },
  { -1, 16, 40,  9 },
  { -1, 15, 40, 10 },
  {  0, 13, 40, 11 },
#endif
};

//2x
const TFilterCoeff InterpolationFilter::m_chromaFilterRPR2[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA] =
{
#if IF_12TAP
  { -5, 64, 138, 64, -5, 0},
  { -8, 66, 132, 73, -6, -1},
  { -9, 62, 134, 76, -5, -2},
  { -10, 59, 134, 80, -4, -3},
  { -10, 57, 132, 82, -2, -3},
  { -10, 53, 132, 85, 0, -4},
  { -11, 50, 132, 89, 1, -5},
  { -12, 47, 132, 93, 2, -6},
  { -13, 45, 131, 96, 4, -7},
  { -12, 40, 131, 99, 5, -7},
  { -12, 39, 127, 101, 9, -8},
  { -13, 36, 127, 105, 10, -9},
  { -12, 33, 124, 107, 14, -10},
  { -13, 30, 124, 111, 15, -11},
  { -13, 29, 121, 112, 18, -11},
  { -12, 25, 119, 115, 21, -12},
  { -13, 24, 117, 117, 24, -13},
  { -12, 21, 115, 119, 25, -12},
  { -11, 18, 112, 121, 29, -13},
  { -11, 15, 111, 124, 30, -13},
  { -10, 14, 107, 124, 33, -12},
  { -9, 10, 105, 127, 36, -13},
  { -8, 9, 101, 127, 39, -12},
  { -7, 5, 99, 131, 40, -12},
  { -7, 4, 96, 131, 45, -13},
  { -6, 2, 93, 132, 47, -12},
  { -5, 1, 89, 132, 50, -11},
  { -4, 0, 85, 132, 53, -10},
  { -3, -2, 82, 132, 57, -10},
  { -3, -4, 80, 134, 59, -10},
  { -2, -5, 76, 134, 62, -9},
  { -1, -6, 73, 132, 66, -8},
#else
  { 17, 30, 17,  0 },
  { 17, 30, 18, -1 },
  { 16, 30, 18,  0 },
  { 16, 30, 18,  0 },
  { 15, 30, 18,  1 },
  { 14, 30, 18,  2 },
  { 13, 29, 19,  3 },
  { 13, 29, 19,  3 },
  { 12, 29, 20,  3 },
  { 11, 28, 21,  4 },
  { 10, 28, 22,  4 },
  { 10, 27, 22,  5 },
  {  9, 27, 23,  5 },
  {  9, 26, 24,  5 },
  {  8, 26, 24,  6 },
  {  7, 26, 25,  6 },
  {  7, 25, 25,  7 },
  {  6, 25, 26,  7 },
  {  6, 24, 26,  8 },
  {  5, 24, 26,  9 },
  {  5, 23, 27,  9 },
  {  5, 22, 27, 10 },
  {  4, 22, 28, 10 },
  {  4, 21, 28, 11 },
  {  3, 20, 29, 12 },
  {  3, 19, 29, 13 },
  {  3, 19, 29, 13 },
  {  2, 18, 30, 14 },
  {  1, 18, 30, 15 },
  {  0, 18, 30, 16 },
  {  0, 18, 30, 16 },
  { -1, 18, 30, 17 }
#endif
};

const TFilterCoeff InterpolationFilter::m_bilinearFilterPrec4[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_BILINEAR] =
{
  { 16,  0, },
  { 15,  1, },
  { 14,  2, },
  { 13, 3, },
  { 12, 4, },
  { 11, 5, },
  { 10, 6, },
  { 9, 7, },
  { 8, 8, },
  { 7, 9, },
  { 6, 10, },
  { 5, 11, },
  { 4, 12, },
  { 3, 13, },
  { 2, 14, },
  { 1, 15, }
};


const TFilterCoeff InterpolationFilter::m_bilinearFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_BILINEAR] =
{
#if IF_12TAP
  { 64 * 4,  0 * 4, },
  { 60 * 4,  4 * 4, },
  { 56 * 4,  8 * 4, },
  { 52 * 4, 12 * 4, },
  { 48 * 4, 16 * 4, },
  { 44 * 4, 20 * 4, },
  { 40 * 4, 24 * 4, },
  { 36 * 4, 28 * 4, },
  { 32 * 4, 32 * 4, },
  { 28 * 4, 36 * 4, },
  { 24 * 4, 40 * 4, },
  { 20 * 4, 44 * 4, },
  { 16 * 4, 48 * 4, },
  { 12 * 4, 52 * 4, },
  {  8 * 4, 56 * 4, },
  {  4 * 4, 60 * 4, },
#else
  { 64,  0, },
  { 60,  4, },
  { 56,  8, },
  { 52, 12, },
  { 48, 16, },
  { 44, 20, },
  { 40, 24, },
  { 36, 28, },
  { 32, 32, },
  { 28, 36, },
  { 24, 40, },
  { 20, 44, },
  { 16, 48, },
  { 12, 52, },
  { 8, 56, },
  { 4, 60, },
#endif
};

#if IF_12TAP
#define _ADJIF_(x)  ((x) * 4)
#else
#define _ADJIF_(x)  (x)
#endif

const TFilterCoeff InterpolationFilter::m_bilinearFilterChroma[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_BILINEAR] =
{
  { _ADJIF_(64), _ADJIF_( 0), },
  { _ADJIF_(62), _ADJIF_( 2), },
  { _ADJIF_(60), _ADJIF_( 4), },
  { _ADJIF_(58), _ADJIF_( 6), },
  { _ADJIF_(56), _ADJIF_( 8), },
  { _ADJIF_(54), _ADJIF_(10), },
  { _ADJIF_(52), _ADJIF_(12), },
  { _ADJIF_(50), _ADJIF_(14), },
  { _ADJIF_(48), _ADJIF_(16), },
  { _ADJIF_(46), _ADJIF_(18), },
  { _ADJIF_(44), _ADJIF_(20), },
  { _ADJIF_(42), _ADJIF_(22), },
  { _ADJIF_(40), _ADJIF_(24), },
  { _ADJIF_(38), _ADJIF_(26), },
  { _ADJIF_(36), _ADJIF_(28), },
  { _ADJIF_(34), _ADJIF_(30), },
  { _ADJIF_(32), _ADJIF_(32), },
  { _ADJIF_(30), _ADJIF_(34), },
  { _ADJIF_(28), _ADJIF_(36), },
  { _ADJIF_(26), _ADJIF_(38), },
  { _ADJIF_(24), _ADJIF_(40), },
  { _ADJIF_(22), _ADJIF_(42), },
  { _ADJIF_(20), _ADJIF_(44), },
  { _ADJIF_(18), _ADJIF_(46), },
  { _ADJIF_(16), _ADJIF_(48), },
  { _ADJIF_(14), _ADJIF_(50), },
  { _ADJIF_(12), _ADJIF_(52), },
  { _ADJIF_(10), _ADJIF_(54), },
  { _ADJIF_( 8), _ADJIF_(56), },
  { _ADJIF_( 6), _ADJIF_(58), },
  { _ADJIF_( 4), _ADJIF_(60), },
  { _ADJIF_( 2), _ADJIF_(62), },
};

#undef _ADJIF_


// filter coefficients for INTRA_6TAP
const TFilterCoeff InterpolationFilter::m_weak4TapFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][4] = {
  { 0, 64, 0, 0 },    
  { -1, 64, 1, 0 },   
  { -3, 65, 3, -1 },  
  { -3, 63, 5, -1 },  
  { -4, 63, 6, -1 },
  { -5, 62, 9, -2 },  
  { -5, 60, 11, -2 }, 
  { -5, 58, 13, -2 }, 
  { -6, 57, 16, -3 }, 
  { -6, 55, 18, -3 },
  { -7, 54, 21, -4 }, 
  { -7, 52, 23, -4 }, 
  { -6, 48, 26, -4 }, 
  { -7, 47, 29, -5 }, 
  { -6, 43, 32, -5 },
  { -6, 41, 34, -5 }, 
  { -5, 37, 37, -5 }, 
  { -5, 34, 41, -6 }, 
  { -5, 32, 43, -6 }, 
  { -5, 29, 47, -7 },
  { -4, 26, 48, -6 }, 
  { -4, 23, 52, -7 }, 
  { -4, 21, 54, -7 }, 
  { -3, 18, 55, -6 }, 
  { -3, 16, 57, -6 },
  { -2, 13, 58, -5 }, 
  { -2, 11, 60, -5 }, 
  { -2, 9, 62, -5 },  
  { -1, 6, 63, -4 },  
  { -1, 5, 63, -3 },
  { -1, 3, 65, -3 },  
  { 0, 1, 64, -1 },
};
const TFilterCoeff InterpolationFilter::m_lumaIntraFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][6] = {
  { 0, 0, 256, 0, 0, 0 },         //  0/32 position
  { 0, -4, 253, 9, -2, 0 },       //  1/32 position
  { 1, -7, 249, 17, -4, 0 },      //  2/32 position
  { 1, -10, 245, 25, -6, 1 },     //  3/32 position
  { 1, -13, 241, 34, -8, 1 },     //  4/32 position
  { 2, -16, 235, 44, -10, 1 },    //  5/32 position
  { 2, -18, 229, 53, -12, 2 },    //  6/32 position
  { 2, -20, 223, 63, -14, 2 },    //  7/32 position
  { 2, -22, 217, 72, -15, 2 },    //  8/32 position
  { 3, -23, 209, 82, -17, 2 },    //  9/32 position
  { 3, -24, 202, 92, -19, 2 },    // 10/32 position
  { 3, -25, 194, 101, -20, 3 },   // 11/32 position
  { 3, -25, 185, 111, -21, 3 },   // 12/32 position
  { 3, -26, 178, 121, -23, 3 },   // 13/32 position
  { 3, -25, 168, 131, -24, 3 },   // 14/32 position
  { 3, -25, 159, 141, -25, 3 },   // 15/32 position
  { 3, -25, 150, 150, -25, 3 },   // half-pel position
  { 3, -25, 141, 159, -25, 3 },   //  17/32 position
  { 3, -24, 131, 168, -25, 3 },   //  18/32 position
  { 3, -23, 121, 178, -26, 3 },   //  19/32 position
  { 3, -21, 111, 185, -25, 3 },   //  20/32 position
  { 3, -20, 101, 194, -25, 3 },   //  21/32 position
  { 2, -19, 92, 202, -24, 3 },    //  22/32 position
  { 2, -17, 82, 209, -23, 3 },    //  23/32 position
  { 2, -15, 72, 217, -22, 2 },    //  24/32 position
  { 2, -14, 63, 223, -20, 2 },    //  25/32 position
  { 2, -12, 53, 229, -18, 2 },    //  26/32 position
  { 1, -10, 44, 235, -16, 2 },    // 27/32 position
  { 1, -8, 34, 241, -13, 1 },     // 28/32 position
  { 1, -6, 25, 245, -10, 1 },     // 29/32 position
  { 0, -4, 17, 249, -7, 1 },      // 30/32 position
  { 0, -2, 9, 253, -4, 0 },       // 31/32 position
};

const TFilterCoeff InterpolationFilter::m_lumaIntraFilterExt[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS<<1][6] =
{
  {   0,   0, 256,   0,   0,   0 },
  {   0,  -2, 255,   4,  -1,   0 },
  {   0,  -4, 253,   9,  -2,   0 },
  {   0,  -5, 251,  13,  -3,   0 },
  {   1,  -7, 249,  17,  -4,   0 },
  {   1,  -9, 247,  21,  -5,   1 },
  {   1, -10, 245,  25,  -6,   1 },
  {   1, -12, 243,  30,  -7,   1 },
  {   1, -13, 241,  34,  -8,   1 },
  {   2, -15, 238,  39,  -9,   1 },
  {   2, -16, 235,  44, -10,   1 },
  {   2, -17, 232,  49, -11,   1 },
  {   2, -18, 229,  53, -12,   2 },
  {   2, -19, 226,  58, -13,   2 },
  {   2, -20, 223,  63, -14,   2 },
  {   2, -21, 220,  68, -15,   2 },
  {   2, -22, 217,  72, -15,   2 },
  {   2, -23, 213,  78, -16,   2 },
  {   3, -23, 209,  82, -17,   2 },
  {   3, -24, 205,  88, -18,   2 },
  {   3, -24, 202,  92, -19,   2 },
  {   3, -24, 198,  97, -20,   2 },
  {   3, -25, 194, 101, -20,   3 },
  {   3, -25, 189, 106, -20,   3 },
  {   3, -25, 185, 111, -21,   3 },
  {   3, -25, 181, 116, -22,   3 },
  {   3, -26, 178, 121, -23,   3 },
  {   3, -26, 173, 126, -23,   3 },
  {   3, -25, 168, 131, -24,   3 },
  {   3, -25, 163, 137, -25,   3 },
  {   3, -25, 159, 141, -25,   3 },
  {   3, -25, 155, 145, -25,   3 },
  {   3, -25, 150, 150, -25,   3 },
  {   3, -25, 145, 155, -25,   3 },
  {   3, -25, 141, 159, -25,   3 },
  {   3, -25, 137, 163, -25,   3 },
  {   3, -24, 131, 168, -25,   3 },
  {   3, -24, 126, 173, -25,   3 },
  {   3, -23, 121, 178, -26,   3 },
  {   3, -22, 116, 181, -25,   3 },
  {   3, -21, 111, 185, -25,   3 },
  {   3, -20, 106, 189, -25,   3 },
  {   3, -20, 101, 194, -25,   3 },
  {   2, -20,  97, 198, -24,   3 },
  {   2, -19,  92, 202, -24,   3 },
  {   2, -18,  86, 206, -23,   3 },
  {   2, -17,  82, 209, -23,   3 },
  {   2, -16,  77, 213, -23,   3 },
  {   2, -15,  72, 217, -22,   2 },
  {   2, -15,  68, 220, -21,   2 },
  {   2, -14,  63, 223, -20,   2 },
  {   2, -13,  58, 226, -19,   2 },
  {   2, -12,  53, 229, -18,   2 },
  {   2, -11,  48, 232, -17,   2 },
  {   1, -10,  44, 235, -16,   2 },
  {   1,  -9,  39, 238, -15,   2 },
  {   1,  -8,  34, 241, -13,   1 },
  {   1,  -7,  29, 243, -11,   1 },
  {   1,  -6,  25, 245, -10,   1 },
  {   0,  -5,  21, 247,  -8,   1 },
  {   0,  -4,  17, 249,  -7,   1 },
  {   0,  -3,  13, 251,  -5,   0 },
  {   0,  -2,   9, 253,  -4,   0 },
  {   0,  -1,   5, 255,  -3,   0 },
};
#if JVET_Z0117_CHROMA_IF
const TFilterCoeff InterpolationFilter::g_aiExtIntraCubicFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << 1][4] = {
#else
const TFilterCoeff InterpolationFilter::g_aiExtIntraCubicFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS<<1][NTAPS_CHROMA] = {
#endif
  {   0, 256,   0,   0 },
  {  -1, 254,   4,  -1 },
  {  -3, 252,   8,  -1 },
  {  -4, 250,  12,  -2 },
  {  -5, 247,  17,  -3 },
  {  -6, 244,  21,  -3 },
  {  -7, 242,  25,  -4 },
  {  -8, 239,  29,  -4 },
  {  -9, 236,  34,  -5 },
  {  -9, 233,  38,  -6 },
  { -10, 230,  43,  -7 },
  { -11, 227,  47,  -7 },
  { -12, 224,  52,  -8 },
  { -12, 220,  56,  -8 },
  { -13, 217,  61,  -9 },
  { -14, 214,  65,  -9 },
  { -14, 210,  70, -10 },
  { -14, 206,  75, -11 },
  { -15, 203,  79, -11 },
  { -15, 199,  84, -12 },
  { -16, 195,  89, -12 },
  { -16, 191,  93, -12 },
  { -16, 187,  98, -13 },
  { -16, 183, 102, -13 },
  { -16, 179, 107, -14 },
  { -16, 174, 112, -14 },
  { -16, 170, 116, -14 },
  { -16, 166, 121, -15 },
  { -17, 162, 126, -15 },
  { -16, 157, 130, -15 },
  { -16, 153, 135, -16 },
  { -16, 148, 140, -16 },
  { -16, 144, 144, -16 },
  { -16, 140, 148, -16},
  { -16, 135, 153, -16},
  { -15, 130, 157, -16},
  { -15, 126, 162, -17},
  { -15, 121, 166, -16},
  { -14, 116, 170, -16},
  { -14, 112, 174, -16},
  { -14, 107, 179, -16},
  { -13, 102, 183, -16},
  { -13,  98, 187, -16},
  { -12,  93, 191, -16},
  { -12,  89, 195, -16},
  { -12,  84, 199, -15},
  { -11,  79, 203, -15},
  { -11,  75, 206, -14},
  { -10,  70, 210, -14},
  {  -9,  65, 214, -14},
  {  -9,  61, 217, -13},
  {  -8,  56, 220, -12},
  {  -8,  52, 224, -12},
  {  -7,  47, 227, -11},
  {  -7,  43, 230, -10},
  {  -6,  38, 233,  -9},
  {  -5,  34, 236,  -9},
  {  -4,  29, 239,  -8},
  {  -4,  25, 242,  -7},
  {  -3,  21, 244,  -6},
  {  -3,  17, 247,  -5},
  {  -2,  12, 250,  -4},
  {  -1,   8, 252,  -3},
  {  -1,   4, 254,  -1},
};
#if JVET_Z0117_CHROMA_IF
const TFilterCoeff InterpolationFilter::g_aiExtIntraGaussFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << 1][4] = {
#else
const TFilterCoeff InterpolationFilter::g_aiExtIntraGaussFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS<<1][NTAPS_CHROMA] = {
#endif
  {  47, 161,  47,   1 },
  {  45, 161,  49,   1 },
  {  43, 161,  51,   1 },
  {  42, 160,  52,   2 },
  {  40, 160,  54,   2 },
  {  38, 160,  56,   2 },
  {  37, 159,  58,   2 },
  {  35, 158,  61,   2 },
  {  34, 158,  62,   2 },
  {  32, 157,  65,   2 },
  {  31, 156,  67,   2 },
  {  29, 155,  69,   3 },
  {  28, 154,  71,   3 },
  {  27, 153,  73,   3 },
  {  26, 151,  76,   3 },
  {  25, 150,  78,   3 },
  {  23, 149,  80,   4 },
  {  22, 147,  83,   4 },
  {  21, 146,  85,   4 },
  {  20, 144,  87,   5 },
  {  19, 142,  90,   5 },
  {  18, 141,  92,   5 },
  {  17, 139,  94,   6 },
  {  16, 137,  97,   6 },
  {  16, 135,  99,   6 },
  {  15, 133, 101,   7 },
  {  14, 131, 104,   7 },
  {  13, 129, 106,   8 },
  {  13, 127, 108,   8 },
  {  12, 125, 111,   8 },
  {  11, 123, 113,   9 },
  {  11, 120, 116,   9 },
  {  10, 118, 118,  10 },
  {   9, 116, 120,  11},
  {   9, 113, 123,  11},
  {   8, 111, 125,  12},
  {   8, 108, 127,  13},
  {   8, 106, 129,  13},
  {   7, 104, 131,  14},
  {   7, 101, 133,  15},
  {   6,  99, 135,  16},
  {   6,  97, 137,  16},
  {   6,  94, 139,  17},
  {   5,  92, 141,  18},
  {   5,  90, 142,  19},
  {   5,  87, 144,  20},
  {   4,  85, 146,  21},
  {   4,  83, 147,  22},
  {   4,  80, 149,  23},
  {   3,  78, 150,  25},
  {   3,  76, 151,  26},
  {   3,  73, 153,  27},
  {   3,  71, 154,  28},
  {   3,  69, 155,  29},
  {   2,  67, 156,  31},
  {   2,  65, 157,  32},
  {   2,  62, 158,  34},
  {   2,  61, 158,  35},
  {   2,  58, 159,  37},
  {   2,  56, 160,  38},
  {   2,  54, 160,  40},
  {   2,  52, 160,  42},
  {   1,  51, 161,  43},
  {   1,  49, 161,  45},
};
// clang-format on

// ====================================================================================================================
// Private member functions
// ====================================================================================================================

InterpolationFilter::InterpolationFilter()
{
#if IF_12TAP
  m_filterHor[_12_TAPS][0][0] = filter<12, false, false, false, false>;
  m_filterHor[_12_TAPS][0][1] = filter<12, false, false, true, false>;
  m_filterHor[_12_TAPS][1][0] = filter<12, false, true, false, false>;
  m_filterHor[_12_TAPS][1][1] = filter<12, false, true, true, false>;
#endif
  m_filterHor[_8_TAPS][0][0] = filter<8, false, false, false, false>;
  m_filterHor[_8_TAPS][0][1] = filter<8, false, false, true, false>;
  m_filterHor[_8_TAPS][1][0] = filter<8, false, true, false, false>;
  m_filterHor[_8_TAPS][1][1] = filter<8, false, true, true, false>;

  m_filterHor[_4_TAPS][0][0] = filter<4, false, false, false, false>;
  m_filterHor[_4_TAPS][0][1] = filter<4, false, false, true, false>;
  m_filterHor[_4_TAPS][1][0] = filter<4, false, true, false, false>;
  m_filterHor[_4_TAPS][1][1] = filter<4, false, true, true, false>;

  m_filterHor[_2_TAPS_DMVR][0][0] = filter<2, false, false, false, true>;
  m_filterHor[_2_TAPS_DMVR][0][1] = filter<2, false, false, true, true>;
  m_filterHor[_2_TAPS_DMVR][1][0] = filter<2, false, true, false, true>;
  m_filterHor[_2_TAPS_DMVR][1][1] = filter<2, false, true, true, true>;

  m_filterHor[_6_TAPS][0][0] = filter<6, false, false, false, false>;
  m_filterHor[_6_TAPS][0][1] = filter<6, false, false, true, false>;
  m_filterHor[_6_TAPS][1][0] = filter<6, false, true, false, false>;
  m_filterHor[_6_TAPS][1][1] = filter<6, false, true, true, false>;

  m_filterHor[_2_TAPS][0][0] = filter<2, false, false, false, false>;
  m_filterHor[_2_TAPS][0][1] = filter<2, false, false, true, false>;
  m_filterHor[_2_TAPS][1][0] = filter<2, false, true, false, false>;
  m_filterHor[_2_TAPS][1][1] = filter<2, false, true, true, false>;

#if IF_12TAP
  m_filterVer[_12_TAPS][0][0] = filter<12, true, false, false, false>;
  m_filterVer[_12_TAPS][0][1] = filter<12, true, false, true, false>;
  m_filterVer[_12_TAPS][1][0] = filter<12, true, true, false, false>;
  m_filterVer[_12_TAPS][1][1] = filter<12, true, true, true, false>;
#endif
  m_filterVer[_8_TAPS][0][0] = filter<8, true, false, false, false>;
  m_filterVer[_8_TAPS][0][1] = filter<8, true, false, true, false>;
  m_filterVer[_8_TAPS][1][0] = filter<8, true, true, false, false>;
  m_filterVer[_8_TAPS][1][1] = filter<8, true, true, true, false>;

  m_filterVer[_4_TAPS][0][0] = filter<4, true, false, false, false>;
  m_filterVer[_4_TAPS][0][1] = filter<4, true, false, true, false>;
  m_filterVer[_4_TAPS][1][0] = filter<4, true, true, false, false>;
  m_filterVer[_4_TAPS][1][1] = filter<4, true, true, true, false>;

  m_filterVer[_2_TAPS_DMVR][0][0] = filter<2, true, false, false, true>;
  m_filterVer[_2_TAPS_DMVR][0][1] = filter<2, true, false, true, true>;
  m_filterVer[_2_TAPS_DMVR][1][0] = filter<2, true, true, false, true>;
  m_filterVer[_2_TAPS_DMVR][1][1] = filter<2, true, true, true, true>;

  m_filterVer[_6_TAPS][0][0] = filter<6, true, false, false, false>;
  m_filterVer[_6_TAPS][0][1] = filter<6, true, false, true, false>;
  m_filterVer[_6_TAPS][1][0] = filter<6, true, true, false, false>;
  m_filterVer[_6_TAPS][1][1] = filter<6, true, true, true, false>;

  m_filterVer[_2_TAPS][0][0] = filter<2, true, false, false, false>;
  m_filterVer[_2_TAPS][0][1] = filter<2, true, false, true, false>;
  m_filterVer[_2_TAPS][1][0] = filter<2, true, true, false, false>;
  m_filterVer[_2_TAPS][1][1] = filter<2, true, true, true, false>;

  m_filterCopy[0][0] = filterCopy<false, false>;
  m_filterCopy[0][1] = filterCopy<false, true>;
  m_filterCopy[1][0] = filterCopy<true, false>;
  m_filterCopy[1][1] = filterCopy<true, true>;

#if IF_12TAP
  m_filter4x4[0][0] = filterXxY_N12<false, 4>;
  m_filter4x4[0][1] = filterXxY_N12<true, 4>;
#else
  m_filter4x4[0][0] = filterXxY_N8<false, 4>;
  m_filter4x4[0][1] = filterXxY_N8<true, 4>;
#endif
#if JVET_Z0117_CHROMA_IF
  m_filter4x4[1][0] = filterXxY_N6<false, 4>;
  m_filter4x4[1][1] = filterXxY_N6<true, 4>;
#else
  m_filter4x4[1][0] = filterXxY_N4<false, 4>;
  m_filter4x4[1][1] = filterXxY_N4<true, 4>;
#endif

  m_weightedGeoBlk        = xWeightedGeoBlk;
  m_weightedGeoBlkRounded = xWeightedGeoBlkRounded;
  m_weightedSgpm          = xWeightedSgpm;
  m_sadTM                 = xSadTM;
  m_sgpmSadTM             = xSgpmSadTM;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// !!! NOTE !!!
//
//  This is the scalar version of the function.
//  If you change the functionality here, consider to switch off the SIMD implementation of this function.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<bool isFirst, bool isLast>
void InterpolationFilter::filterCopy(const ClpRng &clpRng, const Pel *src, const ptrdiff_t srcStride, Pel *dst,
                                     const ptrdiff_t dstStride, int width, int height, bool biMCForDMVR)
{
  int row, col;

  if (isFirst == isLast)
  {
    for (row = 0; row < height; row++)
    {
      for (col = 0; col < width; col++)
      {
        dst[col] = src[col];
        JVET_J0090_CACHE_ACCESS(&src[col], __FILE__, __LINE__);
      }

      src += srcStride;
      dst += dstStride;
    }
  }
  else if (isFirst)
  {
    const int shift = IF_INTERNAL_FRAC_BITS(clpRng.bd);

    if (biMCForDMVR)
    {
      int shift10BitOut, offset;
      if ((clpRng.bd - IF_INTERNAL_PREC_BILINEAR) > 0)
      {
        shift10BitOut = (clpRng.bd - IF_INTERNAL_PREC_BILINEAR);
        offset        = (1 << (shift10BitOut - 1));
        for (row = 0; row < height; row++)
        {
          for (col = 0; col < width; col++)
          {
            dst[col] = (src[col] + offset) >> shift10BitOut;
          }
          src += srcStride;
          dst += dstStride;
        }
      }
      else
      {
        shift10BitOut = (IF_INTERNAL_PREC_BILINEAR - clpRng.bd);
        for (row = 0; row < height; row++)
        {
          for (col = 0; col < width; col++)
          {
            dst[col] = src[col] << shift10BitOut;
          }
          src += srcStride;
          dst += dstStride;
        }
      }
    }
    else
    {
      if (shift >= 0)
      {
        for (row = 0; row < height; row++)
        {
          for (col = 0; col < width; col++)
          {
            Pel val  = src[col] << shift;
            dst[col] = val - (Pel)IF_INTERNAL_OFFS;
            JVET_J0090_CACHE_ACCESS(&src[col], __FILE__, __LINE__);
          }

          src += srcStride;
          dst += dstStride;
        }
      }
      else
      {
        for (row = 0; row < height; row++)
        {
          for (col = 0; col < width; col++)
          {
            Pel val  = (src[col] + (1 << (-shift) >> 1)) >> -shift;
            dst[col] = val - (Pel)IF_INTERNAL_OFFS;
            JVET_J0090_CACHE_ACCESS(&src[col], __FILE__, __LINE__);
          }

          src += srcStride;
          dst += dstStride;
        }
      }
    }
  }
  else
  {
    const int shift = IF_INTERNAL_FRAC_BITS(clpRng.bd);

    if (biMCForDMVR)
    {
      int shift10BitOut, offset;
      if ((clpRng.bd - IF_INTERNAL_PREC_BILINEAR) > 0)
      {
        shift10BitOut = (clpRng.bd - IF_INTERNAL_PREC_BILINEAR);
        offset        = (1 << (shift10BitOut - 1));
        for (row = 0; row < height; row++)
        {
          for (col = 0; col < width; col++)
          {
            dst[col] = (src[col] + offset) >> shift10BitOut;
          }
          src += srcStride;
          dst += dstStride;
        }
      }
      else
      {
        shift10BitOut = (IF_INTERNAL_PREC_BILINEAR - clpRng.bd);
        for (row = 0; row < height; row++)
        {
          for (col = 0; col < width; col++)
          {
            dst[col] = src[col] << shift10BitOut;
          }
          src += srcStride;
          dst += dstStride;
        }
      }
    }
    else
    {
      for (row = 0; row < height; row++)
      {
        for (col = 0; col < width; col++)
        {
          Pel val = src[col];
          val     = rightShift_round((val + IF_INTERNAL_OFFS), shift);

          dst[col] = ClipPel(val, clpRng);
          JVET_J0090_CACHE_ACCESS(&src[col], __FILE__, __LINE__);
        }

        src += srcStride;
        dst += dstStride;
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// !!! NOTE !!!
//
//  This is the scalar version of the function.
//  If you change the functionality here, consider to switch off the SIMD implementation of this function.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<int N, bool isVertical, bool isFirst, bool isLast, bool biMCForDMVR>
void InterpolationFilter::filter(const ClpRng &clpRng, Pel const *src, const ptrdiff_t srcStride, Pel *dst,
                                 const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *coeff)
{
  int row, col;

#if IF_12TAP
  Pel c[N];
  for (int i = 0; i < N; i++)
  {
    c[i] = coeff[i];
  }
#else
  Pel c[8];
  c[0] = coeff[0];
  c[1] = coeff[1];
  if (N >= 4)
  {
    c[2] = coeff[2];
    c[3] = coeff[3];
  }
  if (N >= 6)
  {
    c[4] = coeff[4];
    c[5] = coeff[5];
  }
  if (N == 8)
  {
    c[6] = coeff[6];
    c[7] = coeff[7];
  }
#endif

  const ptrdiff_t cStride = (isVertical) ? srcStride : 1;
  src -= (N / 2 - 1) * cStride;

  int offset;
  int headRoom = IF_INTERNAL_FRAC_BITS(clpRng.bd);
  int shift    = IF_FILTER_PREC;

  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20
  CHECK(shift < 0, "Negative shift");

  if (isLast)
  {
    shift += (isFirst) ? 0 : headRoom;
    offset = 1 << (shift - 1);
    offset += (isFirst) ? 0 : IF_INTERNAL_OFFS << IF_FILTER_PREC;
  }
  else
  {
    shift -= (isFirst) ? headRoom : 0;
    offset = (isFirst) ? -IF_INTERNAL_OFFS << shift : 0;
  }

  if (biMCForDMVR)
  {
    if (isFirst)
    {
      shift  = IF_FILTER_PREC_BILINEAR - (IF_INTERNAL_PREC_BILINEAR - clpRng.bd);
      offset = 1 << (shift - 1);
    }
    else
    {
      shift  = 4;
      offset = 1 << (shift - 1);
    }
  }
  for (row = 0; row < height; row++)
  {
    for (col = 0; col < width; col++)
    {
      int sum;

      sum = src[col + 0 * cStride] * c[0];
      sum += src[col + 1 * cStride] * c[1];
      JVET_J0090_CACHE_ACCESS(&src[col + 0 * cStride], __FILE__, __LINE__);
      JVET_J0090_CACHE_ACCESS(&src[col + 1 * cStride], __FILE__, __LINE__);
      if (N >= 4)
      {
        sum += src[col + 2 * cStride] * c[2];
        sum += src[col + 3 * cStride] * c[3];
        JVET_J0090_CACHE_ACCESS(&src[col + 2 * cStride], __FILE__, __LINE__);
        JVET_J0090_CACHE_ACCESS(&src[col + 3 * cStride], __FILE__, __LINE__);
      }
      if (N >= 6)
      {
        sum += src[col + 4 * cStride] * c[4];
        sum += src[col + 5 * cStride] * c[5];
        JVET_J0090_CACHE_ACCESS(&src[col + 4 * cStride], __FILE__, __LINE__);
        JVET_J0090_CACHE_ACCESS(&src[col + 5 * cStride], __FILE__, __LINE__);
      }
#if IF_12TAP
      if (N >= 8)
#else
      if (N == 8)
#endif
      {
        sum += src[col + 6 * cStride] * c[6];
        sum += src[col + 7 * cStride] * c[7];
        JVET_J0090_CACHE_ACCESS(&src[col + 6 * cStride], __FILE__, __LINE__);
        JVET_J0090_CACHE_ACCESS(&src[col + 7 * cStride], __FILE__, __LINE__);
      }
#if IF_12TAP
      if (N >= 10)
      {
        sum += src[col + 8 * cStride] * c[8];
        sum += src[col + 9 * cStride] * c[9];
        JVET_J0090_CACHE_ACCESS(&src[col + 8 * cStride], __FILE__, __LINE__);
        JVET_J0090_CACHE_ACCESS(&src[col + 9 * cStride], __FILE__, __LINE__);
      }
      if (N >= 12)
      {
        sum += src[col + 10 * cStride] * c[10];
        sum += src[col + 11 * cStride] * c[11];
        JVET_J0090_CACHE_ACCESS(&src[col + 10 * cStride], __FILE__, __LINE__);
        JVET_J0090_CACHE_ACCESS(&src[col + 11 * cStride], __FILE__, __LINE__);
      }

      if (N == 16)
      {
        sum += src[col + 12 * cStride] * c[12];
        sum += src[col + 13 * cStride] * c[13];
        sum += src[col + 14 * cStride] * c[14];
        sum += src[col + 15 * cStride] * c[15];
        JVET_J0090_CACHE_ACCESS(&src[col + 12 * cStride], __FILE__, __LINE__);
        JVET_J0090_CACHE_ACCESS(&src[col + 13 * cStride], __FILE__, __LINE__);
        JVET_J0090_CACHE_ACCESS(&src[col + 14 * cStride], __FILE__, __LINE__);
        JVET_J0090_CACHE_ACCESS(&src[col + 15 * cStride], __FILE__, __LINE__);
      }
#endif

      Pel val = (sum + offset) >> shift;
      if (isLast)
      {
        val = ClipPel(val, clpRng);
      }
      dst[col] = val;
    }

    src += srcStride;
    dst += dstStride;
  }
}

template<int N, bool biMCForDMVR> void InterpolationFilter::filterHor(const ClpRng &clpRng, Pel const *src,
                                                                      const ptrdiff_t srcStride, Pel *dst,
                                                                      const ptrdiff_t dstStride, int width, int height,
                                                                      bool isLast, TFilterCoeff const *coeff) const
{
  constexpr int IDX = tapToIdx(N, biMCForDMVR);
  static_assert(IDX < NUM_TAP_MODES, "Unsupported tap count");
  m_filterHor[IDX][1][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff);
}

template<int N, bool biMCForDMVR>
void InterpolationFilter::filterVer(const ClpRng &clpRng, Pel const *src, const ptrdiff_t srcStride, Pel *dst,
                                    const ptrdiff_t dstStride, int width, int height, bool isFirst, bool isLast,
                                    TFilterCoeff const *coeff) const
{
  constexpr int IDX = tapToIdx(N, biMCForDMVR);
  static_assert(IDX < NUM_TAP_MODES, "Unsupported tap count");
  m_filterVer[IDX][isFirst][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff);
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================
void InterpolationFilter::filterHor(const CompID compID, Pel const *src, const ptrdiff_t srcStride, Pel *dst,
                                    const ptrdiff_t dstStride, int width, int height, int frac, bool isLast,
                                    const ClpRng &clpRng, Filter nFilterIdx) const
{
  if (frac == 0 && nFilterIdx <= Filter::AFFINE)
  {
    m_filterCopy[true][isLast](clpRng, src, srcStride, dst, dstStride, width, height, nFilterIdx == Filter::DMVR);
  }
  else if (isLuma(compID))
  {
    CHECK(frac < 0 || frac >= LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS, "Invalid fraction");
    if (nFilterIdx == Filter::DMVR)
    {
      filterHor<NTAPS_BILINEAR, true>(clpRng, src, srcStride, dst, dstStride, width, height, isLast,
                                      m_bilinearFilterPrec4[frac]);
    }
    else if (nFilterIdx == Filter::AFFINE)
    {
#if IF_12TAP
      filterHor<NTAPS_LUMA, false>(clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilter12[frac]);
#else
      filterHor<NTAPS_LUMA_AFFINE, false>(clpRng, src, srcStride, dst, dstStride, width, height, isLast,
                                          m_affineLumaFilter[frac]);
#endif
    }
    else if (nFilterIdx == Filter::RPR1)
    {
      filterHor<NTAPS_LUMA, false>(clpRng, src, srcStride, dst, dstStride, width, height, isLast,
                                   m_lumaFilterRPR1[frac]);
    }
    else if (nFilterIdx == Filter::RPR2)
    {
      filterHor<NTAPS_LUMA, false>(clpRng, src, srcStride, dst, dstStride, width, height, isLast,
                                   m_lumaFilterRPR2[frac]);
    }
    else if (nFilterIdx == Filter::AFFINE_RPR1)
    {
      filterHor<NTAPS_LUMA, false>(clpRng, src, srcStride, dst, dstStride, width, height, isLast,
                                   m_affineLumaFilterRPR1[frac]);
    }
    else if (nFilterIdx == Filter::AFFINE_RPR2)
    {
      filterHor<NTAPS_LUMA, false>(clpRng, src, srcStride, dst, dstStride, width, height, isLast,
                                   m_affineLumaFilterRPR2[frac]);
    }
    else if (frac == LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS / 2 && nFilterIdx == Filter::HALFPEL_ALT)
    {
      filterHor<8, false>(clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaAltHpelIFilter);
    }
    else if (nFilterIdx == Filter::IBC)
    {
      filterHor<NTAPS_LUMA_IBC, false>(clpRng, src, srcStride, dst, dstStride, width, height, isLast,
                                       m_lumaFilter[frac]);
    }
    else
    {
#if IF_12TAP
      filterHor<NTAPS_LUMA, false>(clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilter12[frac]);
#else
      filterHor<NTAPS_LUMA, false>(clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilter[frac]);
#endif
    }
  }
  else
  {
    CHECK(frac < 0 || frac >= CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS, "Invalid fraction");
    if (nFilterIdx == Filter::DMVR)
    {
      filterHor<NTAPS_BILINEAR, false>(clpRng, src, srcStride, dst, dstStride, width, height, isLast,
                                       m_bilinearFilterChroma[frac]);
    }
    else if (nFilterIdx == Filter::RPR1)
    {
      filterHor<NTAPS_CHROMA, false>(clpRng, src, srcStride, dst, dstStride, width, height, isLast,
                                     m_chromaFilterRPR1[frac]);
    }
    else if (nFilterIdx == Filter::RPR2)
    {
      filterHor<NTAPS_CHROMA, false>(clpRng, src, srcStride, dst, dstStride, width, height, isLast,
                                     m_chromaFilterRPR2[frac]);
    }
    else
    {
      filterHor<NTAPS_CHROMA, false>(clpRng, src, srcStride, dst, dstStride, width, height, isLast,
                                     m_chromaFilter[frac]);
    }
  }
}

void InterpolationFilter::filterVer(const CompID compID, Pel const *src, const ptrdiff_t srcStride, Pel *dst,
                                    const ptrdiff_t dstStride, int width, int height, int frac, bool isFirst,
                                    bool isLast, const ClpRng &clpRng, Filter nFilterIdx) const
{
  if (frac == 0 && nFilterIdx <= Filter::AFFINE)
  {
    m_filterCopy[isFirst][isLast](clpRng, src, srcStride, dst, dstStride, width, height, nFilterIdx == Filter::DMVR);
  }
  else if (isLuma(compID))
  {
    CHECK(frac < 0 || frac >= LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS, "Invalid fraction");
    if (nFilterIdx == Filter::DMVR)
    {
      filterVer<NTAPS_BILINEAR, true>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast,
                                      m_bilinearFilterPrec4[frac]);
    }
    else if (nFilterIdx == Filter::AFFINE)
    {
#if IF_12TAP
      filterVer<NTAPS_LUMA, false>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast,
                                   m_lumaFilter12[frac]);
#else
      filterVer<NTAPS_LUMA_AFFINE, false>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast,
                                          m_affineLumaFilter[frac]);
#endif
    }
    else if (nFilterIdx == Filter::RPR1)
    {
      filterVer<NTAPS_LUMA, false>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast,
                                   m_lumaFilterRPR1[frac]);
    }
    else if (nFilterIdx == Filter::RPR2)
    {
      filterVer<NTAPS_LUMA, false>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast,
                                   m_lumaFilterRPR2[frac]);
    }
    else if (nFilterIdx == Filter::AFFINE_RPR1)
    {
      filterVer<NTAPS_LUMA, false>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast,
                                   m_affineLumaFilterRPR1[frac]);
    }
    else if (nFilterIdx == Filter::AFFINE_RPR2)
    {
      filterVer<NTAPS_LUMA, false>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast,
                                   m_affineLumaFilterRPR2[frac]);
    }
    else if (frac == 8 && nFilterIdx == Filter::HALFPEL_ALT)
    {
      filterVer<8, false>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaAltHpelIFilter);
    }
    else if (nFilterIdx == Filter::IBC)
    {
      filterVer<NTAPS_LUMA_IBC, false>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast,
                                       m_lumaFilter[frac]);
    }
    else
    {
#if IF_12TAP
      filterVer<NTAPS_LUMA, false>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast,
                                   m_lumaFilter12[frac]);
#else
      filterVer<NTAPS_LUMA, false>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast,
                                   m_lumaFilter[frac]);
#endif
    }
  }
  else
  {
    CHECK(frac < 0 || frac >= CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS, "Invalid fraction");
    if (nFilterIdx == Filter::DMVR)
    {
      filterVer<NTAPS_BILINEAR, false>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast,
                                       m_bilinearFilterChroma[frac]);
    }
    else if (nFilterIdx == Filter::RPR1)
    {
      filterVer<NTAPS_CHROMA, false>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast,
                                     m_chromaFilterRPR1[frac]);
    }
    else if (nFilterIdx == Filter::RPR2)
    {
      filterVer<NTAPS_CHROMA, false>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast,
                                     m_chromaFilterRPR2[frac]);
    }
    else
    {
      filterVer<NTAPS_CHROMA, false>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast,
                                     m_chromaFilter[frac]);
    }
  }
}

int InterpolationFilter::xSadTM(const CodingUnit &cu, const int width, const int height, const int templateWidth,
                                const int templateHeight, const CompID compIdx, PelBuf &predBuf, PelBuf &recBuf,
                                PelBuf &adBuf)
{
  int       sad         = 0;
  ptrdiff_t iPredStride = predBuf.stride;
  ptrdiff_t iRecStride  = recBuf.stride;
  ptrdiff_t iAdStride   = adBuf.stride;

  // top template
  Pel *piPred = predBuf.buf + templateWidth;
  // start point of predBuf is (-templateWidth, -templateHeight) of current block
  Pel *piAd   = adBuf.buf + templateWidth;
  Pel *piRec  = recBuf.buf - templateHeight * iRecStride;   // start point of recBuf is (0,0) of current block

  for (int y = 0; y < templateHeight; y++)
  {
    for (int x = 0; x < width; x++)
    {
      *piAd = abs(*piRec - *piPred);
      sad += *piAd;
      piRec++;
      piPred++;
      piAd++;
    }
    piPred += (iPredStride - width);
    piAd += (iAdStride - width);
    piRec += (iRecStride - width);
  }
  // left template
  piPred = predBuf.buf + templateHeight * iPredStride;
  // start point of predBuf is (-templateWidth, -templateHeight) of current block
  piAd   = adBuf.buf + templateHeight * iAdStride;
  piRec  = recBuf.buf - templateWidth;   // start point of recBuf is (0,0) of current block

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < templateWidth; x++)
    {
      *piAd = abs(*piRec - *piPred);
      sad += *piAd;
      piRec++;
      piPred++;
      piAd++;
    }
    piPred += (iPredStride - templateWidth);
    piAd += (iAdStride - templateWidth);
    piRec += (iRecStride - templateWidth);
  }
  return sad;
}

int InterpolationFilter::xSgpmSadTM(const CodingUnit &cu, const int width, const int height, const int templateWidth,
                                    const int templateHeight, const CompID compIdx, const uint8_t splitDir,
                                    PelBuf &adBuf)
{
  int16_t  angle      = g_geoParams[splitDir].angleIdx;
  int16_t  wIdx       = floorLog2(cu.lwidth()) - GEO_MIN_CU_LOG2_EX;
  int16_t  hIdx       = floorLog2(cu.lheight()) - GEO_MIN_CU_LOG2_EX;
  int16_t  stepX      = 1;
  int      maskStride = 0;
  int16_t *weight     = nullptr;

  if (g_angle2mirror[angle] == 2)
  {
    stepX      = 1;
    maskStride = -GEO_WEIGHT_MASK_SIZE_EXT;
    weight     = &g_globalGeoWeightsTpl[g_angle2mask[angle]]
                                   [(GEO_WEIGHT_MASK_SIZE_EXT - 1 - g_weightOffsetEx[splitDir][hIdx][wIdx][1] -
                                     GEO_TM_ADDED_WEIGHT_MASK_SIZE) *
                                      GEO_WEIGHT_MASK_SIZE_EXT +
                                    g_weightOffsetEx[splitDir][hIdx][wIdx][0] + GEO_TM_ADDED_WEIGHT_MASK_SIZE];
  }
  else if (g_angle2mirror[angle] == 1)
  {
    stepX      = -1;
    maskStride = GEO_WEIGHT_MASK_SIZE_EXT;
    weight     = &g_globalGeoWeightsTpl[g_angle2mask[angle]]
                                   [(g_weightOffsetEx[splitDir][hIdx][wIdx][1] + GEO_TM_ADDED_WEIGHT_MASK_SIZE) *
                                      GEO_WEIGHT_MASK_SIZE_EXT +
                                    (GEO_WEIGHT_MASK_SIZE_EXT - 1 - g_weightOffsetEx[splitDir][hIdx][wIdx][0] -
                                     GEO_TM_ADDED_WEIGHT_MASK_SIZE)];
  }
  else
  {
    stepX      = 1;
    maskStride = GEO_WEIGHT_MASK_SIZE_EXT;
    weight     = &g_globalGeoWeightsTpl[g_angle2mask[angle]]
                                   [(g_weightOffsetEx[splitDir][hIdx][wIdx][1] + GEO_TM_ADDED_WEIGHT_MASK_SIZE) *
                                      GEO_WEIGHT_MASK_SIZE_EXT +
                                    g_weightOffsetEx[splitDir][hIdx][wIdx][0] + GEO_TM_ADDED_WEIGHT_MASK_SIZE];
  }

  ptrdiff_t iAdStride = adBuf.stride;

  // top template
  Pel *piAd = adBuf.buf + templateWidth;   // start point of adBuf is (-templateWidth, -templateHeight) of current block
  Pel *weightTmp = weight - templateHeight * maskStride;

  int sum = 0;

  for (int y = 0; y < templateHeight; y++)
  {
    for (int x = 0; x < width; x++)
    {
      sum += *piAd * (*weightTmp);
      piAd++;
      weightTmp += stepX;
    }
    piAd += (iAdStride - width);
    weightTmp += (maskStride - width * stepX);
  }

  // left template
  piAd      = adBuf.buf + templateHeight * iAdStride;
  // start point of predBuf is (-templateWidth, -templateHeight) of current block
  weightTmp = weight - templateWidth * stepX;

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < templateWidth; x++)
    {
      sum += *piAd * (*weightTmp);
      piAd++;
      weightTmp += stepX;
    }
    piAd += (iAdStride - templateWidth);
    weightTmp += (maskStride - templateWidth * stepX);
  }
  return sum;
}

void InterpolationFilter::xWeightedSgpm(const CodingUnit &cu, const uint32_t width, const uint32_t height,
                                        const CompID compIdx, const uint8_t splitDir, PelBuf &predDst, PelBuf &predSrc0,
                                        PelBuf &predSrc1)
{
  Pel      *dst        = predDst.buf;
  Pel      *src0       = predSrc0.buf;
  Pel      *src1       = predSrc1.buf;
  ptrdiff_t strideDst  = predDst.stride - width;
  ptrdiff_t strideSrc0 = predSrc0.stride - width;
  ptrdiff_t strideSrc1 = predSrc1.stride - width;

  const ClpRng clipRng = cu.slice->m_clpRngs.comp[compIdx];

  const int32_t  shiftWeighted  = 5;
  const int32_t  offsetWeighted = 16;
  const uint32_t scaleX         = getComponentScaleX(compIdx, cu.chromaFormat);
  const uint32_t scaleY         = getComponentScaleY(compIdx, cu.chromaFormat);

  int16_t  angle  = g_geoParams[splitDir].angleIdx;
  int16_t  wIdx   = floorLog2(cu.lwidth()) - GEO_MIN_CU_LOG2_EX;
  int16_t  hIdx   = floorLog2(cu.lheight()) - GEO_MIN_CU_LOG2_EX;
  int16_t  stepX  = 1 << scaleX;
  int16_t  stepY  = 0;
  int16_t *weight = nullptr;

  int blendWIdx = 0;
  if (!cu.cs->pps->m_useSgpmNoBlend)
  {
    blendWIdx = GET_SGPM_BLD_IDX(cu.lwidth(), cu.lheight());
  }

  if (g_angle2mirror[angle] == 2)
  {
    stepY  = -(int)((GEO_WEIGHT_MASK_SIZE << scaleY) + cu.lwidth());
    weight = &g_globalGeoWeights[blendWIdx][g_angle2mask[angle]]
                                [(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffsetEx[splitDir][hIdx][wIdx][1]) *
                                   GEO_WEIGHT_MASK_SIZE +
                                 g_weightOffsetEx[splitDir][hIdx][wIdx][0]];
  }
  else if (g_angle2mirror[angle] == 1)
  {
    stepX  = -(1 << scaleX);
    stepY  = (GEO_WEIGHT_MASK_SIZE << scaleY) + cu.lwidth();
    weight = &g_globalGeoWeights[blendWIdx][g_angle2mask[angle]]
                                [g_weightOffsetEx[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE +
                                 (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffsetEx[splitDir][hIdx][wIdx][0])];
  }
  else
  {
    stepY  = (GEO_WEIGHT_MASK_SIZE << scaleY) - cu.lwidth();
    weight = &g_globalGeoWeights[blendWIdx][g_angle2mask[angle]]
                                [g_weightOffsetEx[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE +
                                 g_weightOffsetEx[splitDir][hIdx][wIdx][0]];
  }
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      *dst++ = ClipPel(rightShift((*weight * (*src0++) + ((32 - *weight) * (*src1++)) + offsetWeighted), shiftWeighted),
                       clipRng);
      weight += stepX;
    }
    dst += strideDst;
    src0 += strideSrc0;
    src1 += strideSrc1;
    weight += stepY;
  }
}

void InterpolationFilter::weightedGeoBlk(const CodingUnit &cu, const uint32_t width, const uint32_t height,
                                         const CompID compIdx, const uint8_t splitDir, const uint8_t bldIdx,
                                         PelUnitBuf &predDst, PelUnitBuf &predSrc0, PelUnitBuf &predSrc1)
{
  m_weightedGeoBlk(cu, width, height, compIdx, splitDir, bldIdx, predDst, predSrc0, predSrc1);
}

void InterpolationFilter::xWeightedGeoBlk(const CodingUnit &cu, const uint32_t width, const uint32_t height,
                                          const CompID compIdx, const uint8_t splitDir, const uint8_t bldIdx,
                                          PelUnitBuf &predDst, PelUnitBuf &predSrc0, PelUnitBuf &predSrc1)
{
  Pel      *dst        = predDst.get(compIdx).buf;
  Pel      *src0       = predSrc0.get(compIdx).buf;
  Pel      *src1       = predSrc1.get(compIdx).buf;
  ptrdiff_t strideDst  = predDst.get(compIdx).stride - width;
  ptrdiff_t strideSrc0 = predSrc0.get(compIdx).stride - width;
  ptrdiff_t strideSrc1 = predSrc1.get(compIdx).stride - width;

  const int8_t   log2WeightBase = 5;
  const ClpRng   clipRng        = cu.slice->m_clpRngs.comp[compIdx];
  const int32_t  clipbd         = clipRng.bd;
  const int32_t  shiftWeighted  = IF_INTERNAL_FRAC_BITS(clipbd) + log2WeightBase;
  const int32_t  offsetWeighted = (1 << (shiftWeighted - 1)) + (IF_INTERNAL_OFFS << log2WeightBase);
  const uint32_t scaleX         = getComponentScaleX(compIdx, cu.chromaFormat);
  const uint32_t scaleY         = getComponentScaleY(compIdx, cu.chromaFormat);

  const int angle = g_geoParams[splitDir].angleIdx;

  int16_t  wIdx   = floorLog2(cu.lwidth()) - GEO_MIN_CU_LOG2;
  int16_t  hIdx   = floorLog2(cu.lheight()) - GEO_MIN_CU_LOG2;
  int16_t  stepX  = 1 << scaleX;
  int16_t  stepY  = 0;
  int16_t *weight = nullptr;

  const int16_t *wOffset = g_weightOffset[splitDir][hIdx][wIdx];

  if (g_angle2mirror[angle] == 2)
  {
    stepY  = -(int)((GEO_WEIGHT_MASK_SIZE << scaleY) + cu.lwidth());
    weight = &g_globalGeoWeights[bldIdx][g_angle2mask[angle]]
                                [(GEO_WEIGHT_MASK_SIZE - 1 - wOffset[1]) * GEO_WEIGHT_MASK_SIZE + wOffset[0]];
  }
  else if (g_angle2mirror[angle] == 1)
  {
    stepX  = -1 * (1 << scaleX);
    stepY  = (GEO_WEIGHT_MASK_SIZE << scaleY) + cu.lwidth();
    weight = &g_globalGeoWeights[bldIdx][g_angle2mask[angle]]
                                [wOffset[1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - wOffset[0])];
  }
  else
  {
    stepY  = (GEO_WEIGHT_MASK_SIZE << scaleY) - cu.lwidth();
    weight = &g_globalGeoWeights[bldIdx][g_angle2mask[angle]][wOffset[1] * GEO_WEIGHT_MASK_SIZE + wOffset[0]];
  }
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      *dst++ = ClipPel(rightShift((*weight * (*src0++) + ((32 - *weight) * (*src1++)) + offsetWeighted), shiftWeighted),
                       clipRng);
      weight += stepX;
    }
    dst += strideDst;
    src0 += strideSrc0;
    src1 += strideSrc1;
    weight += stepY;
  }
}

void InterpolationFilter::filter4x4(const CompID compID, const Pel *src, ptrdiff_t srcStride, Pel *dst,
                                    ptrdiff_t dstStride, int width, int height, int fracX, int fracY, bool isLast,
                                    const ChromaFormat fmt, const ClpRng &clpRng)
{
  const int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;

#if IF_12TAP
  if (vFilterSize == 12)
#else
  if (vFilterSize == 8)
#endif
  {
#if IF_12TAP
    CHECKD(!isLuma(compID), "12-tap filter is only allowed for luma!");
    m_filter4x4[0][isLast](clpRng, src, srcStride, dst, dstStride, width, height, m_lumaFilter12[fracX],
                           m_lumaFilter12[fracY]);
#else
    CHECKD(!isLuma(compID), "8-tap filter is only allowed for luma!");
    m_filter4x4[0][isLast](clpRng, src, srcStride, dst, dstStride, width, height, m_affineLumaFilter[fracX],
                           m_affineLumaFilter[fracY]);
#endif
  }
#if JVET_Z0117_CHROMA_IF
  else if (vFilterSize == 6)
#else
  else if (vFilterSize == 4)
#endif
  {
#if JVET_Z0117_CHROMA_IF
    CHECKD(!isChroma(compID), "6-tap filter is only allowed for chroma!");
#else
    CHECKD(!isChroma(compID), "4-tap filter is only allowed for luma!");
#endif

    m_filter4x4[1][isLast](clpRng, src, srcStride, dst, dstStride, width, height, m_chromaFilter[fracX],
                           m_chromaFilter[fracY]);
  }
  else
  {
    THROW("4x4 interpolation filter does not support bilinear filtering!");
  }
}

template<bool isLast, int w> void InterpolationFilter::filterXxY_N4(const ClpRng &clpRng, const Pel *src,
                                                                    ptrdiff_t srcStride, Pel *_dst, ptrdiff_t dstStride,
                                                                    int width, int height, const TFilterCoeff *coeffH,
                                                                    const TFilterCoeff *coeffV)
{
  int row, col;

  Pel cH[4];
  cH[0] = coeffH[0];
  cH[1] = coeffH[1];
  cH[2] = coeffH[2];
  cH[3] = coeffH[3];
  Pel cV[4];
  cV[0] = coeffV[0];
  cV[1] = coeffV[1];
  cV[2] = coeffV[2];
  cV[3] = coeffV[3];

  int offset1st, offset2nd;
  int headRoom = std::max<int>(2, (IF_INTERNAL_PREC - clpRng.bd));
  int shift1st = IF_FILTER_PREC, shift2nd = IF_FILTER_PREC;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20

  if (isLast)
  {
    shift1st -= headRoom;
    shift2nd += headRoom;
    offset1st = -IF_INTERNAL_OFFS * (1 << shift1st);
    offset2nd = 1 << (shift2nd - 1);
    offset2nd += IF_INTERNAL_OFFS << IF_FILTER_PREC;
  }
  else
  {
    shift1st -= headRoom;
    offset1st = -IF_INTERNAL_OFFS * (1 << shift1st);
    offset2nd = 0;
  }

  src -= 1 + srcStride;

  int *tmp = (int *)alloca(w * height * sizeof(int));
  memset(tmp, 0, w * height * sizeof(int));

  int **dst = (int **)alloca(height * sizeof(int *));

  for (int i = 0; i < height; i++)
  {
    dst[i] = &tmp[i * w];
  }

  for (row = 0; row < (height + 3); row++)
  {
    for (col = 0; col < w; col++)
    {
      int sum;

      sum = src[col] * cH[0];
      sum += src[col + 1] * cH[1];
      sum += src[col + 2] * cH[2];
      sum += src[col + 3] * cH[3];

      sum = (sum + offset1st) >> shift1st;

      if (row >= 0 && row < (height + 0))
      {
        dst[row][col] += sum * cV[0];
      }
      if (row >= 1 && row < (height + 1))
      {
        dst[row - 1][col] += sum * cV[1];
      }
      if (row >= 2 && row < (height + 2))
      {
        dst[row - 2][col] += sum * cV[2];
      }

      if (row >= 3)
      {
        int val = (dst[row - 3][col] + sum * cV[3] + offset2nd) >> shift2nd;
        if (isLast)
        {
          val = ClipPel(val, clpRng);
        }
        _dst[col] = val;
      }
    }

    src += srcStride;
    if (row >= 3)
    {
      _dst += dstStride;
    }
  }
}

template<bool isLast, int w> void InterpolationFilter::filterXxY_N6(const ClpRng &clpRng, const Pel *src,
                                                                    ptrdiff_t srcStride, Pel *_dst, ptrdiff_t dstStride,
                                                                    int width, int h, const TFilterCoeff *coeffH,
                                                                    const TFilterCoeff *coeffV)
{
  int row, col;

  Pel cH[6];
  cH[0] = coeffH[0];
  cH[1] = coeffH[1];
  cH[2] = coeffH[2];
  cH[3] = coeffH[3];
  cH[4] = coeffH[4];
  cH[5] = coeffH[5];
  Pel cV[6];
  cV[0] = coeffV[0];
  cV[1] = coeffV[1];
  cV[2] = coeffV[2];
  cV[3] = coeffV[3];
  cV[4] = coeffV[4];
  cV[5] = coeffV[5];

  int offset1st, offset2nd;
  int headRoom = std::max<int>(2, (IF_INTERNAL_PREC - clpRng.bd));
  int shift1st = IF_FILTER_PREC, shift2nd = IF_FILTER_PREC;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20

  if (isLast)
  {
    shift1st -= headRoom;
    shift2nd += headRoom;
    offset1st = -IF_INTERNAL_OFFS * (1 << shift1st);
    offset2nd = 1 << (shift2nd - 1);
    offset2nd += IF_INTERNAL_OFFS << IF_FILTER_PREC;
  }
  else
  {
    shift1st -= headRoom;
    offset1st = -IF_INTERNAL_OFFS * (1 << shift1st);
    offset2nd = 0;
  }

  src -= 2 * (1 + srcStride);

  int *tmp = (int *)alloca(w * h * sizeof(int));
  memset(tmp, 0, w * h * sizeof(int));

  int **dst = (int **)alloca(h * sizeof(int *));

  for (int i = 0; i < h; i++)
  {
    dst[i] = &tmp[i * w];
  }

  for (row = 0; row < (h + 5); row++)
  {
    for (col = 0; col < w; col++)
    {
      int sum;

      sum = src[col] * cH[0];
      sum += src[col + 1] * cH[1];
      sum += src[col + 2] * cH[2];
      sum += src[col + 3] * cH[3];
      sum += src[col + 4] * cH[4];
      sum += src[col + 5] * cH[5];

      sum = (sum + offset1st) >> shift1st;

      if (row >= 0 && row < (h + 0))
      {
        dst[row][col] += sum * cV[0];
      }
      if (row >= 1 && row < (h + 1))
      {
        dst[row - 1][col] += sum * cV[1];
      }
      if (row >= 2 && row < (h + 2))
      {
        dst[row - 2][col] += sum * cV[2];
      }
      if (row >= 3 && row < (h + 3))
      {
        dst[row - 3][col] += sum * cV[3];
      }
      if (row >= 4 && row < (h + 4))
      {
        dst[row - 4][col] += sum * cV[4];
      }

      if (row >= 5)
      {
        int val = (dst[row - 5][col] + sum * cV[5] + offset2nd) >> shift2nd;
        if (isLast)
        {
          val = ClipPel(val, clpRng);
        }
        _dst[col] = val;
      }
    }

    src += srcStride;
    if (row >= 5)
    {
      _dst += dstStride;
    }
  }
}

template<bool isLast, int w> void InterpolationFilter::filterXxY_N8(const ClpRng &clpRng, const Pel *src,
                                                                    ptrdiff_t srcStride, Pel *_dst, ptrdiff_t dstStride,
                                                                    int width, int h, const TFilterCoeff *coeffH,
                                                                    const TFilterCoeff *coeffV)
{
  int row, col;

  Pel cH[8];
  cH[0] = coeffH[7];
  cH[1] = coeffH[0];
  cH[2] = coeffH[1];
  cH[3] = coeffH[2];
  cH[4] = coeffH[3];
  cH[5] = coeffH[4];
  cH[6] = coeffH[5];
  cH[7] = coeffH[6];
  Pel cV[8];
  cV[0] = coeffV[7];
  cV[1] = coeffV[0];
  cV[2] = coeffV[1];
  cV[3] = coeffV[2];
  cV[4] = coeffV[3];
  cV[5] = coeffV[4];
  cV[6] = coeffV[5];
  cV[7] = coeffV[6];

  int offset1st, offset2nd;
  int headRoom = std::max<int>(2, (IF_INTERNAL_PREC - clpRng.bd));
  int shift1st = IF_FILTER_PREC, shift2nd = IF_FILTER_PREC;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20

  if (isLast)
  {
    shift1st -= headRoom;
    shift2nd += headRoom;
    offset1st = -IF_INTERNAL_OFFS * (1 << shift1st);
    offset2nd = 1 << (shift2nd - 1);
    offset2nd += IF_INTERNAL_OFFS << IF_FILTER_PREC;
  }
  else
  {
    shift1st -= headRoom;
    offset1st = -IF_INTERNAL_OFFS * (1 << shift1st);
    offset2nd = 0;
  }

  src -= 3 * (1 + srcStride);

  int *tmp = (int *)alloca(w * h * sizeof(int));
  memset(tmp, 0, w * h * sizeof(int));

  int **dst = (int **)alloca(h * sizeof(int *));

  for (int i = 0; i < h; i++)
  {
    dst[i] = &tmp[i * w];
  }

  for (row = 0; row < (h + 7); row++)
  {
    for (col = 0; col < w; col++)
    {
      int sum;

      sum = src[col] * cH[0];
      sum += src[col + 1] * cH[1];
      sum += src[col + 2] * cH[2];
      sum += src[col + 3] * cH[3];
      sum += src[col + 4] * cH[4];
      sum += src[col + 5] * cH[5];
      sum += src[col + 6] * cH[6];
      sum += src[col + 7] * cH[7];

      sum = (sum + offset1st) >> shift1st;

      if (row >= 0 && row < (h + 0))
      {
        dst[row][col] += sum * cV[0];
      }
      if (row >= 1 && row < (h + 1))
      {
        dst[row - 1][col] += sum * cV[1];
      }
      if (row >= 2 && row < (h + 2))
      {
        dst[row - 2][col] += sum * cV[2];
      }
      if (row >= 3 && row < (h + 3))
      {
        dst[row - 3][col] += sum * cV[3];
      }
      if (row >= 4 && row < (h + 4))
      {
        dst[row - 4][col] += sum * cV[4];
      }
      if (row >= 5 && row < (h + 5))
      {
        dst[row - 5][col] += sum * cV[5];
      }
      if (row >= 6 && row < (h + 6))
      {
        dst[row - 6][col] += sum * cV[6];
      }

      if (row >= 7)
      {
        int val = (dst[row - 7][col] + sum * cV[7] + offset2nd) >> shift2nd;
        if (isLast)
        {
          val = ClipPel(val, clpRng);
        }
        _dst[col] = val;
      }
    }

    src += srcStride;
    if (row >= 7)
    {
      _dst += dstStride;
    }
  }
}

template<bool isLast, int w>
void InterpolationFilter::filterXxY_N12(const ClpRng &clpRng, const Pel *src, ptrdiff_t srcStride, Pel *_dst,
                                        ptrdiff_t dstStride, int width, int h, const TFilterCoeff *coeffH,
                                        const TFilterCoeff *coeffV)
{
  int row, col;

  Pel cH[12];
  cH[0]  = coeffH[0];
  cH[1]  = coeffH[1];
  cH[2]  = coeffH[2];
  cH[3]  = coeffH[3];
  cH[4]  = coeffH[4];
  cH[5]  = coeffH[5];
  cH[6]  = coeffH[6];
  cH[7]  = coeffH[7];
  cH[8]  = coeffH[8];
  cH[9]  = coeffH[9];
  cH[10] = coeffH[10];
  cH[11] = coeffH[11];
  Pel cV[12];
  cV[0]  = coeffV[0];
  cV[1]  = coeffV[1];
  cV[2]  = coeffV[2];
  cV[3]  = coeffV[3];
  cV[4]  = coeffV[4];
  cV[5]  = coeffV[5];
  cV[6]  = coeffV[6];
  cV[7]  = coeffV[7];
  cV[8]  = coeffV[8];
  cV[9]  = coeffV[9];
  cV[10] = coeffV[10];
  cV[11] = coeffV[11];

  int offset1st, offset2nd;
  int headRoom = std::max<int>(2, (IF_INTERNAL_PREC - clpRng.bd));
  int shift1st = IF_FILTER_PREC, shift2nd = IF_FILTER_PREC;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20

  if (isLast)
  {
    shift1st -= headRoom;
    shift2nd += headRoom;
    offset1st = -IF_INTERNAL_OFFS * (1 << shift1st);
    offset2nd = 1 << (shift2nd - 1);
    offset2nd += IF_INTERNAL_OFFS << IF_FILTER_PREC;
  }
  else
  {
    shift1st -= headRoom;
    offset1st = -IF_INTERNAL_OFFS * (1 << shift1st);
    offset2nd = 0;
  }

  src -= 5 * (1 + srcStride);

  int *tmp = (int *)alloca(w * h * sizeof(int));
  memset(tmp, 0, w * h * sizeof(int));

  int **dst = (int **)alloca(h * sizeof(int *));

  for (int i = 0; i < h; i++)
  {
    dst[i] = &tmp[i * w];
  }

  for (row = 0; row < (h + 11); row++)
  {
    for (col = 0; col < w; col++)
    {
      int sum;

      sum = src[col] * cH[0];
      sum += src[col + 1] * cH[1];
      sum += src[col + 2] * cH[2];
      sum += src[col + 3] * cH[3];
      sum += src[col + 4] * cH[4];
      sum += src[col + 5] * cH[5];
      sum += src[col + 6] * cH[6];
      sum += src[col + 7] * cH[7];
      sum += src[col + 8] * cH[8];
      sum += src[col + 9] * cH[9];
      sum += src[col + 10] * cH[10];
      sum += src[col + 11] * cH[11];

      sum = (sum + offset1st) >> shift1st;

      if (row >= 0 && row < (h + 0))
      {
        dst[row][col] += sum * cV[0];
      }
      if (row >= 1 && row < (h + 1))
      {
        dst[row - 1][col] += sum * cV[1];
      }
      if (row >= 2 && row < (h + 2))
      {
        dst[row - 2][col] += sum * cV[2];
      }
      if (row >= 3 && row < (h + 3))
      {
        dst[row - 3][col] += sum * cV[3];
      }
      if (row >= 4 && row < (h + 4))
      {
        dst[row - 4][col] += sum * cV[4];
      }
      if (row >= 5 && row < (h + 5))
      {
        dst[row - 5][col] += sum * cV[5];
      }
      if (row >= 6 && row < (h + 6))
      {
        dst[row - 6][col] += sum * cV[6];
      }
      if (row >= 7 && row < (h + 7))
      {
        dst[row - 7][col] += sum * cV[7];
      }
      if (row >= 8 && row < (h + 8))
      {
        dst[row - 8][col] += sum * cV[8];
      }
      if (row >= 9 && row < (h + 9))
      {
        dst[row - 9][col] += sum * cV[9];
      }
      if (row >= 10 && row < (h + 10))
      {
        dst[row - 10][col] += sum * cV[10];
      }

      if (row >= 11)
      {
        int val = (dst[row - 11][col] + sum * cV[11] + offset2nd) >> shift2nd;
        if (isLast)
        {
          val = ClipPel(val, clpRng);
        }
        _dst[col] = val;
      }
    }

    src += srcStride;
    if (row >= 11)
    {
      _dst += dstStride;
    }
  }
}

void InterpolationFilter::weightedGeoBlkRounded(const CodingUnit &cu, const uint32_t width, const uint32_t height,
                                                const CompID compIdx, const uint8_t splitDir, const uint8_t bldIdx,
                                                PelUnitBuf &predDst, PelUnitBuf &predSrc0, PelUnitBuf &predSrc1)
{
  m_weightedGeoBlkRounded(cu, width, height, compIdx, splitDir, bldIdx, predDst, predSrc0, predSrc1);
}

void InterpolationFilter::xWeightedGeoBlkRounded(const CodingUnit &cu, const uint32_t width, const uint32_t height,
                                                 const CompID compIdx, const uint8_t splitDir, const uint8_t bldIdx,
                                                 PelUnitBuf &predDst, PelUnitBuf &predSrc0, PelUnitBuf &predSrc1)
{
  Pel      *dst        = predDst.get(compIdx).buf;
  Pel      *src0       = predSrc0.get(compIdx).buf;
  Pel      *src1       = predSrc1.get(compIdx).buf;
  ptrdiff_t strideDst  = predDst.get(compIdx).stride - width;
  ptrdiff_t strideSrc0 = predSrc0.get(compIdx).stride - width;
  ptrdiff_t strideSrc1 = predSrc1.get(compIdx).stride - width;

  const uint32_t scaleX = getComponentScaleX(compIdx, cu.chromaFormat);
  const uint32_t scaleY = getComponentScaleY(compIdx, cu.chromaFormat);

  int16_t  angle  = g_geoParams[splitDir].angleIdx;
  int16_t  wIdx   = floorLog2(cu.lwidth()) - GEO_MIN_CU_LOG2;
  int16_t  hIdx   = floorLog2(cu.lheight()) - GEO_MIN_CU_LOG2;
  int16_t  stepX  = 1 << scaleX;
  int16_t  stepY  = 0;
  int16_t *weight = nullptr;

  if (g_angle2mirror[angle] == 2)
  {
    stepY = -(int)((GEO_WEIGHT_MASK_SIZE << scaleY) + cu.lwidth());
    weight =
      &g_globalGeoWeights[bldIdx][g_angle2mask[angle]]
                         [(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE +
                          g_weightOffset[splitDir][hIdx][wIdx][0]];
  }
  else if (g_angle2mirror[angle] == 1)
  {
    stepX  = -1 * (1 << scaleX);
    stepY  = (GEO_WEIGHT_MASK_SIZE << scaleY) + cu.lwidth();
    weight = &g_globalGeoWeights[bldIdx][g_angle2mask[angle]]
                                [g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE +
                                 (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
  }
  else
  {
    stepY = (GEO_WEIGHT_MASK_SIZE << scaleY) - cu.lwidth();
    weight =
      &g_globalGeoWeights[bldIdx][g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE +
                                                       g_weightOffset[splitDir][hIdx][wIdx][0]];
  }

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      *dst++ = (*weight * (*src0++) + ((32 - *weight) * (*src1++)) + 16) >> 5;
      weight += stepX;
    }
    dst += strideDst;
    src0 += strideSrc0;
    src1 += strideSrc1;
    weight += stepY;
  }
}

void InterpolationFilter::initInterpolationFilter(bool enable)
{
#if ENABLE_SIMD_OPT_MCIF
#ifdef TARGET_SIMD_X86
  if (enable)
  {
    initInterpolationFilterX86();
  }
#endif
#endif
}

//! \}
