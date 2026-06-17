/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ITU/ISO/IEC
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

#include "CommonDefX86.h"
#include "../BilateralFilter.h"

#ifdef TARGET_SIMD_X86

#if ENABLE_SIMD_BILATERAL_FILTER

#if USE_AVX2

inline void simdBifApplyLut(__m256i &val, __m256i &acc, int cutBitsNum, __m256i &bitsRound, __m256i &bitsRound2,
                            __m256i &lut)
{
  __m256i diffabs  = _mm256_abs_epi16(val); /* absolute value */
  __m256i diffabs2 = _mm256_add_epi16(diffabs, bitsRound2); /* + bitsRound2 */
  diffabs2         = _mm256_srl_epi16(diffabs2, _mm_set1_epi64x((int64_t)cutBitsNum)); /* >> cutBitsNum */
  diffabs          = _mm256_add_epi16(diffabs, bitsRound); /* + bitsRound2 */
  diffabs          = _mm256_srl_epi16(diffabs, _mm_set1_epi64x((int64_t)cutBitsNum)); /* >> cutBitsNum */
  diffabs          = _mm256_packus_epi16(diffabs2, diffabs); /* convert to 8 bits */
  diffabs          = _mm256_permute4x64_epi64(diffabs, 0xD8); /* permute bytes */
  diffabs          = _mm256_min_epu8(diffabs, _mm256_set1_epi8(15)); /* min(x,15) */
  diffabs          = _mm256_shuffle_epi8(lut, diffabs); /* lut */
  diffabs2         = _mm256_cvtepi8_epi16(*((__m128i *)&diffabs)); /* back to 16-bit */
  diffabs          = _mm256_cvtepi8_epi16(*((__m128i *)&diffabs + 1)); /* back to 16-bit */
  diffabs          = _mm256_srai_epi16(_mm256_add_epi16(_mm256_add_epi16(diffabs, diffabs2), _mm256_set1_epi16(1)),
                                       1); /* (v1 + v2 + 1) >> 1 */
  diffabs          = _mm256_sign_epi16(diffabs, val); /* add original sign */
  acc              = _mm256_add_epi16(diffabs, acc); /* add to acc */
}

template<X86_VEXT vext> void BilateralFilter::simdFilterDiamond5x5(uint32_t uiWidth, uint32_t uiHeight, int16_t block[],
                                                                   int16_t blkFilt[], const ClpRng &clpRng, Pel *recPtr,
                                                                   ptrdiff_t recStride, int iWidthExtSIMD, int bfac,
                                                                   int bifRoundAdd, int bifRoundShift, bool isRDO,
                                                                   const int8_t *lutRowPtr, bool noClip, int cutBitsNum)
{
  // if( uiWidth < 4 || ( uiWidth < 8 && isRDO ) )
  if (uiWidth < 4)
  {
    return blockBilateralFilterDiamond5x5(uiWidth, uiHeight, block, blkFilt, clpRng, recPtr, recStride, iWidthExtSIMD,
                                          bfac, bifRoundAdd, bifRoundShift, isRDO, lutRowPtr, noClip, cutBitsNum);
  }

  int pad      = 2;
  int padwidth = iWidthExtSIMD;

  __m256i center, left, right, up, down, lu, ld, ru, rd, acc, roundAdd, clipmin, clipmax, inputVals;
  __m256i ll, rr, uu, dd;
  __m128i lutTmp;

  clipmin = _mm256_set1_epi16(clpRng.min);
  clipmax = _mm256_set1_epi16(clpRng.max);

  acc                = _mm256_set1_epi32(0);
  lutTmp             = _mm_loadu_si128((__m128i *)(lutRowPtr));
  __m256i lut1       = _mm256_set_m128i(lutTmp, lutTmp);
  lutTmp             = _mm_loadu_si128((__m128i *)(lutRowPtr + 16));
  __m256i lut2       = _mm256_set_m128i(lutTmp, lutTmp);
  lutTmp             = _mm_loadu_si128((__m128i *)(lutRowPtr + 32));
  __m256i lut3       = _mm256_set_m128i(lutTmp, lutTmp);
  __m256i mmBfac     = _mm256_set1_epi16(bfac);
  roundAdd           = _mm256_set1_epi16(bifRoundAdd << 3);
  __m256i bitsRound  = _mm256_set1_epi16(1 << (cutBitsNum - 2));
  __m256i bitsRound2 = _mm256_set1_epi16((1 << (cutBitsNum - 2)) + (1 << (cutBitsNum - 1)));

  for (int row = 0; row < uiHeight; row++)
  {
    for (int col = 0; col < uiWidth; col += 16)
    {
      acc            = _mm256_set1_epi32(0);
      int16_t *point = &block[(row + pad) * padwidth + pad + col];

      center = _mm256_loadu_si256((__m256i *)(point));

      // load neighbours
      left  = _mm256_loadu_si256((__m256i *)(point - 1));
      right = _mm256_loadu_si256((__m256i *)(point + 1));
      up    = _mm256_loadu_si256((__m256i *)(point - padwidth));
      down  = _mm256_loadu_si256((__m256i *)(point + padwidth));

      lu = _mm256_loadu_si256((__m256i *)(point - 1 - padwidth));
      ld = _mm256_loadu_si256((__m256i *)(point - 1 + padwidth));
      ru = _mm256_loadu_si256((__m256i *)(point + 1 - padwidth));
      rd = _mm256_loadu_si256((__m256i *)(point + 1 + padwidth));

      ll = _mm256_loadu_si256((__m256i *)(point - 2));
      rr = _mm256_loadu_si256((__m256i *)(point + 2));
      uu = _mm256_loadu_si256((__m256i *)(point - 2 * padwidth));
      dd = _mm256_loadu_si256((__m256i *)(point + 2 * padwidth));

      // calculate diffs
      left  = _mm256_sub_epi16(left, center);
      right = _mm256_sub_epi16(right, center);
      up    = _mm256_sub_epi16(up, center);
      down  = _mm256_sub_epi16(down, center);

      lu = _mm256_sub_epi16(lu, center);
      ld = _mm256_sub_epi16(ld, center);
      ru = _mm256_sub_epi16(ru, center);
      rd = _mm256_sub_epi16(rd, center);

      ll = _mm256_sub_epi16(ll, center);
      rr = _mm256_sub_epi16(rr, center);
      uu = _mm256_sub_epi16(uu, center);
      dd = _mm256_sub_epi16(dd, center);

      // apply LUT
      simdBifApplyLut(left, acc, cutBitsNum, bitsRound, bitsRound2, lut1);
      simdBifApplyLut(right, acc, cutBitsNum, bitsRound, bitsRound2, lut1);
      simdBifApplyLut(up, acc, cutBitsNum, bitsRound, bitsRound2, lut1);
      simdBifApplyLut(down, acc, cutBitsNum, bitsRound, bitsRound2, lut1);

      simdBifApplyLut(lu, acc, cutBitsNum, bitsRound, bitsRound2, lut2);
      simdBifApplyLut(ld, acc, cutBitsNum, bitsRound, bitsRound2, lut2);
      simdBifApplyLut(ru, acc, cutBitsNum, bitsRound, bitsRound2, lut2);
      simdBifApplyLut(rd, acc, cutBitsNum, bitsRound, bitsRound2, lut2);

      simdBifApplyLut(ll, acc, cutBitsNum, bitsRound, bitsRound2, lut3);
      simdBifApplyLut(rr, acc, cutBitsNum, bitsRound, bitsRound2, lut3);
      simdBifApplyLut(uu, acc, cutBitsNum, bitsRound, bitsRound2, lut3);
      simdBifApplyLut(dd, acc, cutBitsNum, bitsRound, bitsRound2, lut3);

      // TU scaling
      acc = _mm256_mullo_epi16(acc, mmBfac);
      acc = _mm256_adds_epi16(acc, roundAdd);
      acc = _mm256_srai_epi16(acc, bifRoundShift + 3);

      // Instead we add our input values to the delta
      if (isRDO)
      {
        acc = _mm256_add_epi16(acc, center);
      }
      else
      {
        int16_t *recpoint = &recPtr[row * recStride + col];
        inputVals         = _mm256_loadu_si256((__m256i *)(recpoint));
        acc               = _mm256_add_epi16(acc, inputVals);
      }

      // Clip
      if (isRDO || !noClip)
      {
        acc = _mm256_max_epi16(acc, clipmin);
        acc = _mm256_min_epi16(acc, clipmax);
      }

      _mm256_storeu_si256((__m256i *)(blkFilt + (row + pad) * padwidth + col + pad), acc);
    }
  }

  // Copy back from tempbufFilter to recBuf
  int  onerow       = uiWidth * sizeof(Pel);
  // Copy back parameters
  Pel *tempBlockPtr = blkFilt + pad * padwidth + pad;
  for (uint32_t yy = 0; yy < uiHeight; yy++)
  {
    std::memcpy(recPtr, tempBlockPtr, onerow);
    recPtr += recStride;
    tempBlockPtr += padwidth;
  }
}

template<X86_VEXT vext>
int BilateralFilter::simdCalcMAD(int16_t *block, ptrdiff_t stride, int width, int height, int whlog2)
{
  if (width < 8)
  {
    return calcMAD(block, stride, width, height, whlog2);
  }
  int     sum32[8];
  __m256i acc32, val, average;
  acc32 = _mm256_setzero_si256();
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j += 8)
    {
      val   = _mm256_cvtepi16_epi32(_mm_loadu_si128((__m128i *)(block + j)));
      acc32 = _mm256_add_epi32(acc32, val);
    }
    block += stride;
  }
  block -= stride * height;
  acc32 = _mm256_hadd_epi32(acc32, acc32);
  acc32 = _mm256_hadd_epi32(acc32, acc32);
  _mm256_storeu_si256((__m256i *)sum32, acc32);
  average = _mm256_set1_epi32((sum32[0] + sum32[4] + (1 << (whlog2 - 1))) >> whlog2);
  acc32   = _mm256_setzero_si256();
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j += 8)
    {
      val   = _mm256_cvtepi16_epi32(_mm_loadu_si128((__m128i *)(block + j)));
      acc32 = _mm256_add_epi32(acc32, _mm256_abs_epi32(_mm256_sub_epi32(val, average)));
    }
    block += stride;
  }
  acc32 = _mm256_hadd_epi32(acc32, acc32);
  acc32 = _mm256_hadd_epi32(acc32, acc32);
  _mm256_storeu_si256((__m256i *)sum32, acc32);
  return (sum32[0] + sum32[4] + (1 << (whlog2 - 1))) >> whlog2;
}

#else // USE_AVX2

inline void simdBifApplyLut(__m128i &val, __m128i &acc, int cutBitsNum, __m128i &bitsRound, __m128i &bitsRound2,
                            __m128i &lut)
{
  __m128i diffabs  = _mm_abs_epi16(val); /* absolute value */
  __m128i diffabs2 = _mm_add_epi16(diffabs, bitsRound2); /* + bitsRound2 */
  diffabs2         = _mm_srl_epi16(diffabs2, _mm_set1_epi64x((int64_t)cutBitsNum)); /* >> cutBitsNum */
  diffabs          = _mm_add_epi16(diffabs, bitsRound); /* + bitsRound2 */
  diffabs          = _mm_srl_epi16(diffabs, _mm_set1_epi64x((int64_t)cutBitsNum)); /* >> cutBitsNum */
  diffabs          = _mm_packus_epi16(diffabs2, diffabs); /* convert to 8 bits */
  diffabs          = _mm_min_epu8(diffabs, _mm_set1_epi8(15)); /* min(x,15) */
  diffabs          = _mm_shuffle_epi8(lut, diffabs); /* lut */
  diffabs2         = _mm_cvtepi8_epi16(diffabs); /* back to 16-bit */
  diffabs          = _mm_cvtepi8_epi16(_mm_shuffle_epi32(diffabs, 0x4e)); /* back to 16-bit */
  diffabs =
    _mm_srai_epi16(_mm_add_epi16(_mm_add_epi16(diffabs, diffabs2), _mm_set1_epi16(1)), 1); /* (v1 + v2 + 1) >> 1 */
  diffabs = _mm_sign_epi16(diffabs, val); /* add original sign */
  acc     = _mm_add_epi16(diffabs, acc); /* add to acc */
}

template<X86_VEXT vext> void BilateralFilter::simdFilterDiamond5x5(uint32_t uiWidth, uint32_t uiHeight, int16_t block[],
                                                                   int16_t blkFilt[], const ClpRng &clpRng, Pel *recPtr,
                                                                   ptrdiff_t recStride, int iWidthExtSIMD, int bfac,
                                                                   int bifRoundAdd, int bifRoundShift, bool isRDO,
                                                                   const int8_t *lutRowPtr, bool noClip, int cutBitsNum)
{
  // if( uiWidth < 4 || ( uiWidth < 8 && isRDO ) )
  if (uiWidth < 4)
  {
    return blockBilateralFilterDiamond5x5(uiWidth, uiHeight, block, blkFilt, clpRng, recPtr, recStride, iWidthExtSIMD,
                                          bfac, bifRoundAdd, bifRoundShift, isRDO, lutRowPtr, noClip, cutBitsNum);
  }

  int pad      = 2;
  int padwidth = iWidthExtSIMD;

  __m128i center, left, right, up, down, lu, ld, ru, rd, acc, roundAdd, clipmin, clipmax, inputVals;
  __m128i ll, rr, uu, dd;

  clipmin = _mm_set1_epi16(clpRng.min);
  clipmax = _mm_set1_epi16(clpRng.max);

  acc                = _mm_set1_epi32(0);
  __m128i lut1       = _mm_loadu_si128((__m128i *)(lutRowPtr));
  __m128i lut2       = _mm_loadu_si128((__m128i *)(lutRowPtr + 16));
  __m128i lut3       = _mm_loadu_si128((__m128i *)(lutRowPtr + 32));
  __m128i mmBfac     = _mm_set1_epi16(bfac);
  roundAdd           = _mm_set1_epi16(bifRoundAdd << 3);
  __m128i bitsRound  = _mm_set1_epi16(1 << (cutBitsNum - 2));
  __m128i bitsRound2 = _mm_set1_epi16((1 << (cutBitsNum - 2)) + (1 << (cutBitsNum - 1)));

  for (int col = 0; col < uiWidth; col += 8)
  {
    for (int row = 0; row < uiHeight; row++)
    {
      acc            = _mm_set1_epi32(0);
      int16_t *point = &block[(row + pad) * padwidth + pad + col];

      center = _mm_loadu_si128((__m128i *)(point));

      // load neighbours
      left  = _mm_loadu_si128((__m128i *)(point - 1));
      right = _mm_loadu_si128((__m128i *)(point + 1));
      up    = _mm_loadu_si128((__m128i *)(point - padwidth));
      down  = _mm_loadu_si128((__m128i *)(point + padwidth));

      lu = _mm_loadu_si128((__m128i *)(point - 1 - padwidth));
      ld = _mm_loadu_si128((__m128i *)(point - 1 + padwidth));
      ru = _mm_loadu_si128((__m128i *)(point + 1 - padwidth));
      rd = _mm_loadu_si128((__m128i *)(point + 1 + padwidth));

      ll = _mm_loadu_si128((__m128i *)(point - 2));
      rr = _mm_loadu_si128((__m128i *)(point + 2));
      uu = _mm_loadu_si128((__m128i *)(point - 2 * padwidth));
      dd = _mm_loadu_si128((__m128i *)(point + 2 * padwidth));

      // calculate diffs
      left  = _mm_sub_epi16(left, center);
      right = _mm_sub_epi16(right, center);
      up    = _mm_sub_epi16(up, center);
      down  = _mm_sub_epi16(down, center);

      lu = _mm_sub_epi16(lu, center);
      ld = _mm_sub_epi16(ld, center);
      ru = _mm_sub_epi16(ru, center);
      rd = _mm_sub_epi16(rd, center);

      ll = _mm_sub_epi16(ll, center);
      rr = _mm_sub_epi16(rr, center);
      uu = _mm_sub_epi16(uu, center);
      dd = _mm_sub_epi16(dd, center);

      // apply LUT
      simdBifApplyLut(left, acc, cutBitsNum, bitsRound, bitsRound2, lut1);
      simdBifApplyLut(right, acc, cutBitsNum, bitsRound, bitsRound2, lut1);
      simdBifApplyLut(up, acc, cutBitsNum, bitsRound, bitsRound2, lut1);
      simdBifApplyLut(down, acc, cutBitsNum, bitsRound, bitsRound2, lut1);

      simdBifApplyLut(lu, acc, cutBitsNum, bitsRound, bitsRound2, lut2);
      simdBifApplyLut(ld, acc, cutBitsNum, bitsRound, bitsRound2, lut2);
      simdBifApplyLut(ru, acc, cutBitsNum, bitsRound, bitsRound2, lut2);
      simdBifApplyLut(rd, acc, cutBitsNum, bitsRound, bitsRound2, lut2);

      simdBifApplyLut(ll, acc, cutBitsNum, bitsRound, bitsRound2, lut3);
      simdBifApplyLut(rr, acc, cutBitsNum, bitsRound, bitsRound2, lut3);
      simdBifApplyLut(uu, acc, cutBitsNum, bitsRound, bitsRound2, lut3);
      simdBifApplyLut(dd, acc, cutBitsNum, bitsRound, bitsRound2, lut3);

      // TU scaling
      acc = _mm_mullo_epi16(acc, mmBfac);
      acc = _mm_adds_epi16(acc, roundAdd);
      acc = _mm_srai_epi16(acc, bifRoundShift + 3);

      // Instead we add our input values to the delta
      if (isRDO)
      {
        acc = _mm_add_epi16(acc, center);
      }
      else
      {
        int16_t *recpoint = &recPtr[row * recStride + col];
        inputVals         = _mm_loadu_si128((__m128i *)(recpoint));
        acc               = _mm_add_epi16(acc, inputVals);
      }

      // Clip
      if (isRDO || !noClip)
      {
        acc = _mm_max_epi16(acc, clipmin);
        acc = _mm_min_epi16(acc, clipmax);
      }

      _mm_storeu_si128((__m128i *)(blkFilt + (row + pad) * padwidth + col + pad), acc);
    }
  }

  // Copy back from tempbufFilter to recBuf
  int  onerow       = uiWidth * sizeof(Pel);
  // Copy back parameters
  Pel *tempBlockPtr = (short *)blkFilt + pad * padwidth + pad;
  for (uint32_t yy = 0; yy < uiHeight; yy++)
  {
    std::memcpy(recPtr, tempBlockPtr, onerow);
    recPtr += recStride;
    tempBlockPtr += padwidth;
  }
}

template<X86_VEXT vext>
int BilateralFilter::simdCalcMAD(int16_t *block, ptrdiff_t stride, int width, int height, int whlog2)
{
  if (width < 4)
  {
    return calcMAD(block, stride, width, height, whlog2);
  }
  int     sum32[4];
  __m128i acc32, val, average;
  acc32 = _mm_setzero_si128();
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j += 4)
    {
      val   = _mm_loadl_epi64((__m128i *)(block + j));
      val   = _mm_cvtepi16_epi32(val);
      acc32 = _mm_add_epi32(acc32, val);
    }
    block += stride;
  }
  block -= stride * height;
  acc32 = _mm_hadd_epi32(acc32, acc32);
  _mm_storeu_si128((__m128i *)sum32, acc32);
  average = _mm_set1_epi32((sum32[0] + sum32[1] + (1 << (whlog2 - 1))) >> whlog2);
  acc32   = _mm_setzero_si128();
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j += 4)
    {
      val   = _mm_loadl_epi64((__m128i *)(block + j));
      val   = _mm_cvtepi16_epi32(val);
      acc32 = _mm_add_epi32(acc32, _mm_abs_epi32(_mm_sub_epi32(val, average)));
    }
    block += stride;
  }
  acc32 = _mm_hadd_epi32(acc32, acc32);
  _mm_storeu_si128((__m128i *)sum32, acc32);
  return (sum32[0] + sum32[1] + (1 << (whlog2 - 1))) >> whlog2;
}

#endif // USE_AVX2

template<X86_VEXT vext> void BilateralFilter::_initBilateralFilterX86()
{
  m_bilateralFilterDiamond5x5 = simdFilterDiamond5x5<vext>;
  m_calcMAD                   = simdCalcMAD<vext>;
}

template void BilateralFilter::_initBilateralFilterX86<SIMDX86>();

#endif
#endif   // TARGET_SIMD_X86
