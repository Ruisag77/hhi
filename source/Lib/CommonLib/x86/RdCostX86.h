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

/** \file     RdCostX86.cpp
    \brief    RD cost computation class, SIMD version
*/

#include <math.h>
#include <limits>
#include "CommonDefX86.h"
#include "../Rom.h"
#include "../RdCost.h"

#ifdef TARGET_SIMD_X86

// #undef JVET_R0164_MEAN_SCALED_SATD
// #define JVET_R0164_MEAN_SCALED_SATD 0

typedef Pel Torg;
typedef Pel Tcur;

inline __m128i getSse1(const Pel *pSrc1, const ptrdiff_t strideSrc1, const Pel *pSrc2, const ptrdiff_t strideSrc2,
                       const int rows, const int shift)
{
  static_assert(sizeof(Pel) == 2, "Pel must be 16-bit wide");

  uint32_t sum = 0;

  for (int y = 0; y < rows; y++)
  {
    const uint16_t v1 = pSrc1[y * strideSrc1];
    const uint16_t v2 = pSrc2[y * strideSrc2];

    const int16_t  diff = v1 - v2;
    const uint32_t res  = diff * diff >> shift;

    sum += res;
  }

  return _mm_cvtsi32_si128(sum);
}

inline __m128i getSse2(const Pel *pSrc1, const ptrdiff_t strideSrc1, const Pel *pSrc2, const ptrdiff_t strideSrc2,
                       const int rows, const int shift)
{
  static_assert(sizeof(Pel) == 2, "Pel must be 16-bit wide");

  __m128i sum = _mm_setzero_si128();

  for (int y = 0; y < rows; y += 2)
  {
    const uint32_t v1a = *(uint32_t *)(pSrc1 + y * strideSrc1);
    const uint32_t v1b = *(uint32_t *)(pSrc1 + y * strideSrc1 + strideSrc1);
    const uint32_t v2a = *(uint32_t *)(pSrc2 + y * strideSrc2);
    const uint32_t v2b = *(uint32_t *)(pSrc2 + y * strideSrc2 + strideSrc2);

    const __m128i src1 = _mm_unpacklo_epi64(_mm_cvtsi32_si128(v1a), _mm_cvtsi32_si128(v1b));
    const __m128i src2 = _mm_unpacklo_epi64(_mm_cvtsi32_si128(v2a), _mm_cvtsi32_si128(v2b));

    const __m128i diff = _mm_sub_epi16(src1, src2);
    const __m128i res  = _mm_sra_epi32(_mm_madd_epi16(diff, diff), _mm_cvtsi32_si128(shift));
    sum                = _mm_add_epi32(sum, res);
  }

  return sum;
}

inline __m128i getSse4(const Pel *pSrc1, const ptrdiff_t strideSrc1, const Pel *pSrc2, const ptrdiff_t strideSrc2,
                       const int rows, const int shift)
{
  static_assert(sizeof(Pel) == 2, "Pel must be 16-bit wide");

  __m128i sum = _mm_setzero_si128();

  for (int y = 0; y < rows; y++)
  {
    const __m128i src1 = _mm_loadl_epi64((const __m128i *)(pSrc1 + y * strideSrc1));
    const __m128i src2 = _mm_loadl_epi64((const __m128i *)(pSrc2 + y * strideSrc2));

    const __m128i diff = _mm_sub_epi16(src1, src2);
    const __m128i res  = _mm_sra_epi32(_mm_madd_epi16(diff, diff), _mm_cvtsi32_si128(shift));
    sum                = _mm_add_epi32(sum, res);
  }

  return _mm_cvtepu32_epi64(sum);
}

inline __m128i getSse8(const Pel *pSrc1, const ptrdiff_t strideSrc1, const Pel *pSrc2, const ptrdiff_t strideSrc2,
                       const int rows, const int shift)
{
  static_assert(sizeof(Pel) == 2, "Pel must be 16-bit wide");

  __m128i sum = _mm_setzero_si128();

  for (int y = 0; y < rows; y++)
  {
    const __m128i src1 = _mm_loadu_si128((const __m128i *)(pSrc1 + y * strideSrc1));
    const __m128i src2 = _mm_loadu_si128((const __m128i *)(pSrc2 + y * strideSrc2));

    const __m128i diff = _mm_sub_epi16(src1, src2);
    const __m128i res  = _mm_sra_epi32(_mm_madd_epi16(diff, diff), _mm_cvtsi32_si128(shift));
    sum                = _mm_add_epi32(sum, res);
  }

  return _mm_add_epi64(_mm_cvtepu32_epi64(sum), _mm_unpackhi_epi32(sum, _mm_setzero_si128()));
}

#ifdef USE_AVX2
inline __m128i getSse16(const Pel *pSrc1, const ptrdiff_t strideSrc1, const Pel *pSrc2, const ptrdiff_t strideSrc2,
                        const int rows, const int shift)
{
  static_assert(sizeof(Pel) == 2, "Pel must be 16-bit wide");

  __m256i sum = _mm256_setzero_si256();

  for (int y = 0; y < rows; y++)
  {
    const __m256i src1 = _mm256_loadu_si256((const __m256i *)(pSrc1 + y * strideSrc1));
    const __m256i src2 = _mm256_loadu_si256((const __m256i *)(pSrc2 + y * strideSrc2));

    const __m256i diff = _mm256_sub_epi16(src1, src2);
    const __m256i res  = _mm256_sra_epi32(_mm256_madd_epi16(diff, diff), _mm_cvtsi32_si128(shift));
    sum                = _mm256_add_epi32(sum, res);
  }

  sum = _mm256_add_epi64(_mm256_unpacklo_epi32(sum, _mm256_setzero_si256()),
                         _mm256_unpackhi_epi32(sum, _mm256_setzero_si256()));
  return _mm_add_epi64(_mm256_castsi256_si128(sum), _mm256_extracti128_si256(sum, 1));
}
#endif

template<X86_VEXT vext> Distortion RdCost::xGetSSE_SIMD(const DistParam &rcDtParam)
{
  if (rcDtParam.applyWeight)
  {
    return RdCostWeightPrediction::xGetSSEw(rcDtParam);
  }

  const int       rows       = rcDtParam.org.height;
  const int       cols       = rcDtParam.org.width;
  const Pel      *pSrc1      = rcDtParam.org.buf;
  const Pel      *pSrc2      = rcDtParam.cur.buf;
  const ptrdiff_t strideSrc1 = rcDtParam.org.stride;
  const ptrdiff_t strideSrc2 = rcDtParam.cur.stride;

  const uint32_t shift = 2 * DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);

  __m128i sum = _mm_setzero_si128();

  if ((cols & 1) != 0)
  {
    for (int x = 0; x < cols; x += 1)
    {
      sum = _mm_add_epi64(sum, getSse1(pSrc1 + x, strideSrc1, pSrc2 + x, strideSrc2, rows, shift));
    }
  }
  else if ((cols & 2) != 0)
  {
    for (int x = 0; x < cols; x += 2)
    {
      sum = _mm_add_epi64(sum, getSse2(pSrc1 + x, strideSrc1, pSrc2 + x, strideSrc2, rows, shift));
    }
  }
  else if ((cols & 4) != 0)
  {
    for (int x = 0; x < cols; x += 4)
    {
      sum = _mm_add_epi64(sum, getSse4(pSrc1 + x, strideSrc1, pSrc2 + x, strideSrc2, rows, shift));
    }
  }
  else
  {
#ifdef USE_AVX2
    if (vext >= AVX2 && (cols & 15) == 0)
    {
      for (int x = 0; x < cols; x += 16)
      {
        sum = _mm_add_epi64(sum, getSse16(pSrc1 + x, strideSrc1, pSrc2 + x, strideSrc2, rows, shift));
      }
    }
    else
#endif
    {
      for (int x = 0; x < cols; x += 8)
      {
        sum = _mm_add_epi64(sum, getSse8(pSrc1 + x, strideSrc1, pSrc2 + x, strideSrc2, rows, shift));
      }
    }
  }

  sum = _mm_add_epi64(sum, _mm_shuffle_epi32(sum, _MM_SHUFFLE(1, 0, 3, 2)));

  return _mm_cvtsi128_si64(sum);
}

template<int WIDTH, X86_VEXT vext> Distortion RdCost::xGetSSE_NxN_SIMD(const DistParam &rcDtParam)
{
  if (rcDtParam.applyWeight)
  {
    return RdCostWeightPrediction::xGetSSEw(rcDtParam);
  }

  const Pel      *pSrc1      = rcDtParam.org.buf;
  const Pel      *pSrc2      = rcDtParam.cur.buf;
  const int       rows       = rcDtParam.org.height;
  const ptrdiff_t strideSrc1 = rcDtParam.org.stride;
  const ptrdiff_t strideSrc2 = rcDtParam.cur.stride;

  const uint32_t shift = 2 * DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);

  __m128i sum = _mm_setzero_si128();

  if (2 == WIDTH)
  {
    sum = getSse2(pSrc1, strideSrc1, pSrc2, strideSrc2, rows, shift);
  }
  else if (4 == WIDTH)
  {
    sum = getSse4(pSrc1, strideSrc1, pSrc2, strideSrc2, rows, shift);
  }
  else
  {
#ifdef USE_AVX2
    if (vext >= AVX2 && WIDTH >= 16)
    {
      for (int x = 0; x < WIDTH; x += 16)
      {
        sum = _mm_add_epi64(sum, getSse16(pSrc1 + x, strideSrc1, pSrc2 + x, strideSrc2, rows, shift));
      }
    }
    else
#endif
    {
      for (int x = 0; x < WIDTH; x += 8)
      {
        sum = _mm_add_epi64(sum, getSse8(pSrc1 + x, strideSrc1, pSrc2 + x, strideSrc2, rows, shift));
      }
    }
  }

  sum = _mm_add_epi64(sum, _mm_shuffle_epi32(sum, _MM_SHUFFLE(1, 0, 3, 2)));

  return _mm_cvtsi128_si64(sum);
}

template<X86_VEXT vext> Distortion RdCost::xGetSAD_SIMD(const DistParam &rcDtParam)
{
  if (rcDtParam.org.width < 4 || rcDtParam.bitDepth > 10 || rcDtParam.applyWeight)
  {
    return RdCost::xGetSAD(rcDtParam);
  }

  const short    *pSrc1      = (const short *)rcDtParam.org.buf;
  const short    *pSrc2      = (const short *)rcDtParam.cur.buf;
  int             rows       = rcDtParam.org.height;
  int             cols       = rcDtParam.org.width;
  int             subShift   = rcDtParam.subShift;
  int             subStep    = (1 << subShift);
  const ptrdiff_t strideSrc1 = rcDtParam.org.stride * subStep;
  const ptrdiff_t strideSrc2 = rcDtParam.cur.stride * subStep;

  uint32_t sum = 0;
  if (vext >= AVX2 && (cols & 15) == 0)
  {
#ifdef USE_AVX2
    // Do for width that multiple of 16
    __m256i vzero  = _mm256_setzero_si256();
    __m256i vsum32 = vzero;
    for (int y = 0; y < rows; y += subStep)
    {
      __m256i vsum16 = vzero;
      for (int x = 0; x < cols; x += 16)
      {
        __m256i vsrc1 = _mm256_lddqu_si256((__m256i *)(&pSrc1[x]));
        __m256i vsrc2 = _mm256_lddqu_si256((__m256i *)(&pSrc2[x]));
        vsum16        = _mm256_add_epi16(vsum16, _mm256_abs_epi16(_mm256_sub_epi16(vsrc1, vsrc2)));
      }
      __m256i vsumtemp = _mm256_add_epi32(_mm256_unpacklo_epi16(vsum16, vzero), _mm256_unpackhi_epi16(vsum16, vzero));
      vsum32           = _mm256_add_epi32(vsum32, vsumtemp);
      pSrc1 += strideSrc1;
      pSrc2 += strideSrc2;
    }
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    sum    = _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum32)) +
      _mm_cvtsi128_si32(_mm256_castsi256_si128(_mm256_permute2x128_si256(vsum32, vsum32, 0x11)));
#endif
  }
  else if ((cols & 7) == 0)
  {
    // Do with step of 8
    __m128i vzero  = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    for (int y = 0; y < rows; y += subStep)
    {
      __m128i vsum16 = vzero;
      for (int x = 0; x < cols; x += 8)
      {
        __m128i vsrc1 = _mm_loadu_si128((const __m128i *)(&pSrc1[x]));
        __m128i vsrc2 = _mm_lddqu_si128((const __m128i *)(&pSrc2[x]));
        vsum16        = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
      }
      __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
      vsum32           = _mm_add_epi32(vsum32, vsumtemp);
      pSrc1 += strideSrc1;
      pSrc2 += strideSrc2;
    }
    vsum32 = _mm_hadd_epi32(vsum32, vzero);
    vsum32 = _mm_hadd_epi32(vsum32, vzero);
    sum    = _mm_cvtsi128_si32(vsum32);
  }
  else
  {
    // Do with step of 4
    CHECK((cols & 3) != 0, "Not divisible by 4: " << cols);
    __m128i vzero  = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    for (int y = 0; y < rows; y += subStep)
    {
      __m128i vsum16 = vzero;
      for (int x = 0; x < cols; x += 4)
      {
        __m128i vsrc1 = _mm_loadl_epi64((const __m128i *)&pSrc1[x]);
        __m128i vsrc2 = _mm_loadl_epi64((const __m128i *)&pSrc2[x]);
        vsum16        = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
      }
      __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
      vsum32           = _mm_add_epi32(vsum32, vsumtemp);
      pSrc1 += strideSrc1;
      pSrc2 += strideSrc2;
    }
    vsum32 = _mm_hadd_epi32(vsum32, vzero);
    vsum32 = _mm_hadd_epi32(vsum32, vzero);
    sum    = _mm_cvtsi128_si32(vsum32);
  }

  sum <<= subShift;
  return sum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);
}

template<X86_VEXT vext> Distortion RdCost::xGetSAD_IBD_SIMD(const DistParam &rcDtParam)
{
  if (rcDtParam.org.width < 4 || rcDtParam.bitDepth > 10 || rcDtParam.applyWeight)
  {
    return RdCost::xGetSAD(rcDtParam);
  }

  const short    *src0       = (const short *)rcDtParam.org.buf;
  const short    *src1       = (const short *)rcDtParam.cur.buf;
  int             width      = rcDtParam.org.height;
  int             height     = rcDtParam.org.width;
  int             subShift   = rcDtParam.subShift;
  int             subStep    = (1 << subShift);
  const ptrdiff_t src0Stride = rcDtParam.org.stride * subStep;
  const ptrdiff_t src1Stride = rcDtParam.cur.stride * subStep;

  __m128i vtotalsum32 = _mm_setzero_si128();
  __m128i vzero       = _mm_setzero_si128();
  for (int y = 0; y < height; y += subStep)
  {
    for (int x = 0; x < width; x += 4)
    {
      __m128i vsrc1 = _mm_loadl_epi64((const __m128i *)(src0 + x));
      __m128i vsrc2 = _mm_loadl_epi64((const __m128i *)(src1 + x));
      vsrc1         = _mm_cvtepi16_epi32(vsrc1);
      vsrc2         = _mm_cvtepi16_epi32(vsrc2);
      vtotalsum32   = _mm_add_epi32(vtotalsum32, _mm_abs_epi32(_mm_sub_epi32(vsrc1, vsrc2)));
    }
    src0 += src0Stride;
    src1 += src1Stride;
  }
  vtotalsum32    = _mm_hadd_epi32(vtotalsum32, vzero);
  vtotalsum32    = _mm_hadd_epi32(vtotalsum32, vzero);
  Distortion sum = _mm_cvtsi128_si32(vtotalsum32);

  sum <<= subShift;
  return sum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);
}

template<int width, X86_VEXT vext> Distortion RdCost::xGetSAD_NxN_SIMD(const DistParam &rcDtParam)
{
  if (rcDtParam.bitDepth > 10 || rcDtParam.applyWeight)
  {
    return RdCost::xGetSAD(rcDtParam);
  }

  //  assert( rcDtParam.cols == width);
  const short    *pSrc1      = (const short *)rcDtParam.org.buf;
  const short    *pSrc2      = (const short *)rcDtParam.cur.buf;
  int             rows       = rcDtParam.org.height;
  int             subShift   = rcDtParam.subShift;
  int             subStep    = (1 << subShift);
  const ptrdiff_t strideSrc1 = rcDtParam.org.stride * subStep;
  const ptrdiff_t strideSrc2 = rcDtParam.cur.stride * subStep;

  uint32_t sum = 0;

  if (width == 4)
  {
    if (rows == 4 && subShift == 0)
    {
      __m128i vzero = _mm_setzero_si128();
      __m128i vsum  = vzero;
      __m128i vsrc1 = _mm_loadl_epi64((const __m128i *)pSrc1);
      vsrc1         = _mm_castps_si128(_mm_loadh_pi(_mm_castsi128_ps(vsrc1), (__m64 *)&pSrc1[strideSrc1]));
      __m128i vsrc2 = _mm_loadl_epi64((const __m128i *)pSrc2);
      vsrc2         = _mm_castps_si128(_mm_loadh_pi(_mm_castsi128_ps(vsrc2), (__m64 *)&pSrc2[strideSrc2]));
      vsum          = _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2));

      vsrc1 = _mm_loadl_epi64((const __m128i *)&pSrc1[2 * strideSrc1]);
      vsrc1 = _mm_castps_si128(_mm_loadh_pi(_mm_castsi128_ps(vsrc1), (__m64 *)&pSrc1[3 * strideSrc1]));
      vsrc2 = _mm_loadl_epi64((const __m128i *)&pSrc2[2 * strideSrc2]);
      vsrc2 = _mm_castps_si128(_mm_loadh_pi(_mm_castsi128_ps(vsrc2), (__m64 *)&pSrc2[3 * strideSrc2]));
      vsum  = _mm_hadd_epi16(vsum, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
      vsum  = _mm_hadd_epi16(vsum, vzero);
      vsum  = _mm_hadd_epi16(vsum, vzero);
      vsum  = _mm_hadd_epi16(vsum, vzero);
      sum   = _mm_cvtsi128_si32(vsum);
    }
    else
    {
      __m128i vzero  = _mm_setzero_si128();
      __m128i vsum32 = vzero;
      for (int y = 0; y < rows; y += subStep)
      {
        __m128i vsum16 = vzero;
        {
          __m128i vsrc1 = _mm_loadl_epi64((const __m128i *)pSrc1);
          __m128i vsrc2 = _mm_loadl_epi64((const __m128i *)pSrc2);
          vsum16        = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
        }
        __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
        vsum32           = _mm_add_epi32(vsum32, vsumtemp);
        pSrc1 += strideSrc1;
        pSrc2 += strideSrc2;
      }
      vsum32 = _mm_hadd_epi32(vsum32, vzero);
      vsum32 = _mm_hadd_epi32(vsum32, vzero);
      sum    = _mm_cvtsi128_si32(vsum32);
    }
  }
  else
  {
    if (vext >= AVX2 && width >= 16)
    {
#ifdef USE_AVX2
      // Do for width that multiple of 16
      __m256i vzero  = _mm256_setzero_si256();
      __m256i vsum32 = vzero;
      for (int y = 0; y < rows; y += subStep)
      {
        __m256i vsum16 = vzero;
        for (int x = 0; x < width; x += 16)
        {
          __m256i vsrc1 = _mm256_lddqu_si256((__m256i *)(&pSrc1[x]));
          __m256i vsrc2 = _mm256_lddqu_si256((__m256i *)(&pSrc2[x]));
          vsum16        = _mm256_add_epi16(vsum16, _mm256_abs_epi16(_mm256_sub_epi16(vsrc1, vsrc2)));
        }
        __m256i vsumtemp = _mm256_add_epi32(_mm256_unpacklo_epi16(vsum16, vzero), _mm256_unpackhi_epi16(vsum16, vzero));
        vsum32           = _mm256_add_epi32(vsum32, vsumtemp);
        pSrc1 += strideSrc1;
        pSrc2 += strideSrc2;
      }
      vsum32 = _mm256_hadd_epi32(vsum32, vzero);
      vsum32 = _mm256_hadd_epi32(vsum32, vzero);
      sum    = _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum32)) +
        _mm_cvtsi128_si32(_mm256_castsi256_si128(_mm256_permute2x128_si256(vsum32, vsum32, 0x11)));
#endif
    }
    else
    {
      // For width that multiple of 8
      __m128i vzero  = _mm_setzero_si128();
      __m128i vsum32 = vzero;
      for (int y = 0; y < rows; y += subStep)
      {
        __m128i vsum16 = vzero;
        for (int x = 0; x < width; x += 8)
        {
          __m128i vsrc1 = _mm_loadu_si128((const __m128i *)(&pSrc1[x]));
          __m128i vsrc2 = _mm_lddqu_si128((const __m128i *)(&pSrc2[x]));
          vsum16        = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
        }
        __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
        vsum32           = _mm_add_epi32(vsum32, vsumtemp);
        pSrc1 += strideSrc1;
        pSrc2 += strideSrc2;
      }
      vsum32 = _mm_hadd_epi32(vsum32, vzero);
      vsum32 = _mm_hadd_epi32(vsum32, vzero);
      sum    = _mm_cvtsi128_si32(vsum32);
    }
  }

  sum <<= subShift;
  return sum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);
}

static constexpr uint64_t INV_SQRT_2 = 0xb504f334U;   // 2^32 / sqrt(2.0)

static uint32_t xCalcHAD4x4_SSE(const Torg *piOrg, const Tcur *piCur, const ptrdiff_t strideOrg,
                                const ptrdiff_t strideCur)
{
  __m128i r0 = (sizeof(Torg) > 1)
    ? (_mm_loadl_epi64((const __m128i *)&piOrg[0]))
    : (_mm_unpacklo_epi8(_mm_cvtsi32_si128(*(const int *)&piOrg[0]), _mm_setzero_si128()));
  __m128i r1 = (sizeof(Torg) > 1)
    ? (_mm_loadl_epi64((const __m128i *)&piOrg[strideOrg]))
    : (_mm_unpacklo_epi8(_mm_cvtsi32_si128(*(const int *)&piOrg[strideOrg]), _mm_setzero_si128()));
  __m128i r2 = (sizeof(Torg) > 1)
    ? (_mm_loadl_epi64((const __m128i *)&piOrg[2 * strideOrg]))
    : (_mm_unpacklo_epi8(_mm_cvtsi32_si128(*(const int *)&piOrg[2 * strideOrg]), _mm_setzero_si128()));
  __m128i r3 = (sizeof(Torg) > 1)
    ? (_mm_loadl_epi64((const __m128i *)&piOrg[3 * strideOrg]))
    : (_mm_unpacklo_epi8(_mm_cvtsi32_si128(*(const int *)&piOrg[3 * strideOrg]), _mm_setzero_si128()));
  __m128i r4 = (sizeof(Tcur) > 1)
    ? (_mm_loadl_epi64((const __m128i *)&piCur[0]))
    : (_mm_unpacklo_epi8(_mm_cvtsi32_si128(*(const int *)&piCur[0]), _mm_setzero_si128()));
  __m128i r5 = (sizeof(Tcur) > 1)
    ? (_mm_loadl_epi64((const __m128i *)&piCur[strideCur]))
    : (_mm_unpacklo_epi8(_mm_cvtsi32_si128(*(const int *)&piCur[strideCur]), _mm_setzero_si128()));
  __m128i r6 = (sizeof(Tcur) > 1)
    ? (_mm_loadl_epi64((const __m128i *)&piCur[2 * strideCur]))
    : (_mm_unpacklo_epi8(_mm_cvtsi32_si128(*(const int *)&piCur[2 * strideCur]), _mm_setzero_si128()));
  __m128i r7 = (sizeof(Tcur) > 1)
    ? (_mm_loadl_epi64((const __m128i *)&piCur[3 * strideCur]))
    : (_mm_unpacklo_epi8(_mm_cvtsi32_si128(*(const int *)&piCur[3 * strideCur]), _mm_setzero_si128()));

  r0 = _mm_sub_epi16(r0, r4);
  r1 = _mm_sub_epi16(r1, r5);
  r2 = _mm_sub_epi16(r2, r6);
  r3 = _mm_sub_epi16(r3, r7);

  // first stage
  r4 = r0;
  r5 = r1;

  r0 = _mm_add_epi16(r0, r3);
  r1 = _mm_add_epi16(r1, r2);

  r4 = _mm_sub_epi16(r4, r3);
  r5 = _mm_sub_epi16(r5, r2);

  r2 = r0;
  r3 = r4;

  r0 = _mm_add_epi16(r0, r1);
  r2 = _mm_sub_epi16(r2, r1);
  r3 = _mm_sub_epi16(r3, r5);
  r5 = _mm_add_epi16(r5, r4);

  // shuffle - flip matrix for vertical transform
  r0 = _mm_unpacklo_epi16(r0, r5);
  r2 = _mm_unpacklo_epi16(r2, r3);

  r3 = r0;
  r0 = _mm_unpacklo_epi32(r0, r2);
  r3 = _mm_unpackhi_epi32(r3, r2);

  r1 = r0;
  r2 = r3;
  r1 = _mm_srli_si128(r1, 8);
  r3 = _mm_srli_si128(r3, 8);

  // second stage
  r4 = r0;
  r5 = r1;

  r0 = _mm_add_epi16(r0, r3);
  r1 = _mm_add_epi16(r1, r2);

  r4 = _mm_sub_epi16(r4, r3);
  r5 = _mm_sub_epi16(r5, r2);

  r2 = r0;
  r3 = r4;

  r0 = _mm_add_epi16(r0, r1);
  r2 = _mm_sub_epi16(r2, r1);
  r3 = _mm_sub_epi16(r3, r5);
  r5 = _mm_add_epi16(r5, r4);

  // abs
  __m128i Sum = _mm_abs_epi16(r0);
#if JVET_R0164_MEAN_SCALED_SATD
  uint32_t absDc = _mm_cvtsi128_si32(Sum) & 0x0000ffff;
#endif
  Sum = _mm_add_epi16(Sum, _mm_abs_epi16(r2));
  Sum = _mm_add_epi16(Sum, _mm_abs_epi16(r3));
  Sum = _mm_add_epi16(Sum, _mm_abs_epi16(r5));

  __m128i iZero = _mm_set1_epi16(0);
  Sum           = _mm_unpacklo_epi16(Sum, iZero);
  Sum           = _mm_hadd_epi32(Sum, Sum);
  Sum           = _mm_hadd_epi32(Sum, Sum);

  uint32_t sad = _mm_cvtsi128_si32(Sum);

#if JVET_R0164_MEAN_SCALED_SATD
  sad -= absDc;
  sad += absDc >> 2;
#endif
  sad = ((sad + 1) >> 1);

  return sad;
}

#define SUB_PRE_ROUND 0

// working up to 12-bit
template<bool sbsplh, bool sbsplv> static uint32_t xCalcHAD8x8_SSE(const Torg *piOrg, const Tcur *piCur,
                                                                   const ptrdiff_t strideOrg, const ptrdiff_t strideCur,
                                                                   const int bitDepth, const int inBitDepth)
{
  __m128i m2[8][2], m1[8][2];

  for (int k = 0; k < 8; k++)
  {
    __m128i r0, r1;
    if (!sbsplh && !sbsplv)
    {
      r0 = _mm_loadu_si128((const __m128i *)piOrg);
      r1 = _mm_loadu_si128((const __m128i *)piCur);

#if SUB_PRE_ROUND
      m2[k][0] = _mm_sub_epi16(r0, r1);
#endif
    }
    else if (!sbsplh)
    {
      r0         = _mm_loadu_si128((const __m128i *)piOrg);
      r1         = _mm_loadu_si128((const __m128i *)piCur);
      __m128i r2 = _mm_loadu_si128((const __m128i *)(piOrg + strideOrg));
      __m128i r3 = _mm_loadu_si128((const __m128i *)(piCur + strideCur));

      r0 = _mm_add_epi16(r0, r2);
      r1 = _mm_add_epi16(r1, r3);

#if !SUB_PRE_ROUND
      r0 = _mm_add_epi16(r0, _mm_set1_epi16(1));
      r1 = _mm_add_epi16(r1, _mm_set1_epi16(1));
      r0 = _mm_srai_epi16(r0, 1);
      r1 = _mm_srai_epi16(r1, 1);
#else
      r0       = _mm_sub_epi16(r0, r1);
      r0       = _mm_add_epi16(r0, _mm_sign_epi16(_mm_set1_epi16(1), r0));
      r0       = _mm_srai_epi16(r0, 1);
      m2[k][0] = r0;
#endif
    }
    else if (!sbsplv)
    {
      r0         = _mm_loadu_si128((const __m128i *)piOrg);
      r1         = _mm_loadu_si128((const __m128i *)piCur);
      __m128i r2 = _mm_loadu_si128((const __m128i *)(piOrg + 8));
      __m128i r3 = _mm_loadu_si128((const __m128i *)(piCur + 8));

      r0 = _mm_hadd_epi16(r0, r2);
      r1 = _mm_hadd_epi16(r1, r3);

#if !SUB_PRE_ROUND
      r0 = _mm_add_epi16(r0, _mm_set1_epi16(1));
      r1 = _mm_add_epi16(r1, _mm_set1_epi16(1));
      r0 = _mm_srai_epi16(r0, 1);
      r1 = _mm_srai_epi16(r1, 1);
#else
      r0       = _mm_sub_epi16(r0, r1);
      r0       = _mm_add_epi16(r0, _mm_sign_epi16(_mm_set1_epi16(1), r0));
      r0       = _mm_srai_epi16(r0, 1);
      m2[k][0] = r0;
#endif
    }
    else
    {
      r0         = _mm_loadu_si128((const __m128i *)piOrg);
      r1         = _mm_loadu_si128((const __m128i *)piCur);
      __m128i r2 = _mm_loadu_si128((const __m128i *)(piOrg + strideOrg));
      __m128i r3 = _mm_loadu_si128((const __m128i *)(piCur + strideCur));

      r0 = _mm_add_epi16(r0, r2);
      r1 = _mm_add_epi16(r1, r3);

      r2         = _mm_loadu_si128((const __m128i *)(piOrg + 8));
      r3         = _mm_loadu_si128((const __m128i *)(piCur + 8));
      __m128i r4 = _mm_loadu_si128((const __m128i *)(piOrg + strideOrg + 8));
      __m128i r5 = _mm_loadu_si128((const __m128i *)(piCur + strideCur + 8));

      r2 = _mm_add_epi16(r2, r4);
      r3 = _mm_add_epi16(r3, r5);

      r0 = _mm_hadd_epi16(r0, r2);
      r1 = _mm_hadd_epi16(r1, r3);

#if !SUB_PRE_ROUND
      r0 = _mm_add_epi16(r0, _mm_set1_epi16(2));
      r1 = _mm_add_epi16(r1, _mm_set1_epi16(2));
      r0 = _mm_srai_epi16(r0, 2);
      r1 = _mm_srai_epi16(r1, 2);
#else
      r0       = _mm_sub_epi16(r0, r1);
      r0       = _mm_add_epi16(r0, _mm_sign_epi16(_mm_set1_epi16(2), r0));
      r0       = _mm_srai_epi16(r0, 2);
      m2[k][0] = r0;
#endif
    }

#if !SUB_PRE_ROUND
    m2[k][0] = _mm_sub_epi16(r0, r1); // inBitDepth + 1 bit
#endif

    piCur += (strideCur << (sbsplv ? 1 : 0));
    piOrg += (strideOrg << (sbsplv ? 1 : 0));
  }

  __m128i n1[8][2];
  __m128i n2[8][2];

  if (inBitDepth && inBitDepth > 10)
  {
    for (int k = 0; k < 8; k++)
    {
      m2[k][1] = _mm_cvtepi16_epi32(_mm_srli_si128(m2[k][0], 8));
      m2[k][0] = _mm_cvtepi16_epi32(m2[k][0]);
    }

    for (int i = 0; i < 2; i++)
    {
      // horizontal
      m1[0][i] = _mm_add_epi32(m2[0][i], m2[4][i]);
      m1[1][i] = _mm_add_epi32(m2[1][i], m2[5][i]);
      m1[2][i] = _mm_add_epi32(m2[2][i], m2[6][i]);
      m1[3][i] = _mm_add_epi32(m2[3][i], m2[7][i]);
      m1[4][i] = _mm_sub_epi32(m2[0][i], m2[4][i]);
      m1[5][i] = _mm_sub_epi32(m2[1][i], m2[5][i]);
      m1[6][i] = _mm_sub_epi32(m2[2][i], m2[6][i]);
      m1[7][i] = _mm_sub_epi32(m2[3][i], m2[7][i]);

      m2[0][i] = _mm_add_epi32(m1[0][i], m1[2][i]);
      m2[1][i] = _mm_add_epi32(m1[1][i], m1[3][i]);
      m2[2][i] = _mm_sub_epi32(m1[0][i], m1[2][i]);
      m2[3][i] = _mm_sub_epi32(m1[1][i], m1[3][i]);
      m2[4][i] = _mm_add_epi32(m1[4][i], m1[6][i]);
      m2[5][i] = _mm_add_epi32(m1[5][i], m1[7][i]);
      m2[6][i] = _mm_sub_epi32(m1[4][i], m1[6][i]);
      m2[7][i] = _mm_sub_epi32(m1[5][i], m1[7][i]);

      m1[0][i] = _mm_add_epi32(m2[0][i], m2[1][i]);
      m1[1][i] = _mm_sub_epi32(m2[0][i], m2[1][i]);
      m1[2][i] = _mm_add_epi32(m2[2][i], m2[3][i]);
      m1[3][i] = _mm_sub_epi32(m2[2][i], m2[3][i]);
      m1[4][i] = _mm_add_epi32(m2[4][i], m2[5][i]);
      m1[5][i] = _mm_sub_epi32(m2[4][i], m2[5][i]);
      m1[6][i] = _mm_add_epi32(m2[6][i], m2[7][i]);
      m1[7][i] = _mm_sub_epi32(m2[6][i], m2[7][i]);

      m2[0][i] = _mm_unpacklo_epi32(m1[0][i], m1[1][i]);
      m2[1][i] = _mm_unpacklo_epi32(m1[2][i], m1[3][i]);
      m2[2][i] = _mm_unpackhi_epi32(m1[0][i], m1[1][i]);
      m2[3][i] = _mm_unpackhi_epi32(m1[2][i], m1[3][i]);
      m2[4][i] = _mm_unpacklo_epi32(m1[4][i], m1[5][i]);
      m2[5][i] = _mm_unpacklo_epi32(m1[6][i], m1[7][i]);
      m2[6][i] = _mm_unpackhi_epi32(m1[4][i], m1[5][i]);
      m2[7][i] = _mm_unpackhi_epi32(m1[6][i], m1[7][i]);

      m1[0][i] = _mm_unpacklo_epi64(m2[0][i], m2[1][i]);
      m1[1][i] = _mm_unpackhi_epi64(m2[0][i], m2[1][i]);
      m1[2][i] = _mm_unpacklo_epi64(m2[2][i], m2[3][i]);
      m1[3][i] = _mm_unpackhi_epi64(m2[2][i], m2[3][i]);
      m1[4][i] = _mm_unpacklo_epi64(m2[4][i], m2[5][i]);
      m1[5][i] = _mm_unpackhi_epi64(m2[4][i], m2[5][i]);
      m1[6][i] = _mm_unpacklo_epi64(m2[6][i], m2[7][i]);
      m1[7][i] = _mm_unpackhi_epi64(m2[6][i], m2[7][i]);
    }

    for (int i = 0; i < 8; i++)
    {
      int ii = i % 4;
      int ij = i >> 2;

      n2[i][0] = m1[ii][ij];
      n2[i][1] = m1[ii + 4][ij];
    }

    for (int i = 0; i < 2; i++)
    {
      n1[0][i] = _mm_add_epi32(n2[0][i], n2[4][i]);
      n1[1][i] = _mm_add_epi32(n2[1][i], n2[5][i]);
      n1[2][i] = _mm_add_epi32(n2[2][i], n2[6][i]);
      n1[3][i] = _mm_add_epi32(n2[3][i], n2[7][i]);
      n1[4][i] = _mm_sub_epi32(n2[0][i], n2[4][i]);
      n1[5][i] = _mm_sub_epi32(n2[1][i], n2[5][i]);
      n1[6][i] = _mm_sub_epi32(n2[2][i], n2[6][i]);
      n1[7][i] = _mm_sub_epi32(n2[3][i], n2[7][i]);

      n2[0][i] = _mm_add_epi32(n1[0][i], n1[2][i]);
      n2[1][i] = _mm_add_epi32(n1[1][i], n1[3][i]);
      n2[2][i] = _mm_sub_epi32(n1[0][i], n1[2][i]);
      n2[3][i] = _mm_sub_epi32(n1[1][i], n1[3][i]);
      n2[4][i] = _mm_add_epi32(n1[4][i], n1[6][i]);
      n2[5][i] = _mm_add_epi32(n1[5][i], n1[7][i]);
      n2[6][i] = _mm_sub_epi32(n1[4][i], n1[6][i]);
      n2[7][i] = _mm_sub_epi32(n1[5][i], n1[7][i]);
    }
  }
  else
  {
    // for (int i = 0; i < 2; i++)
    {
      // horizontal
      m1[0][0] = _mm_add_epi16(m2[0][0], m2[4][0]);
      m1[1][0] = _mm_add_epi16(m2[1][0], m2[5][0]);
      m1[2][0] = _mm_add_epi16(m2[2][0], m2[6][0]);
      m1[3][0] = _mm_add_epi16(m2[3][0], m2[7][0]);
      m1[4][0] = _mm_sub_epi16(m2[0][0], m2[4][0]);
      m1[5][0] = _mm_sub_epi16(m2[1][0], m2[5][0]);
      m1[6][0] = _mm_sub_epi16(m2[2][0], m2[6][0]);
      m1[7][0] = _mm_sub_epi16(m2[3][0], m2[7][0]); // 12 bit

      m2[0][0] = _mm_add_epi16(m1[0][0], m1[2][0]);
      m2[1][0] = _mm_add_epi16(m1[1][0], m1[3][0]);
      m2[2][0] = _mm_sub_epi16(m1[0][0], m1[2][0]);
      m2[3][0] = _mm_sub_epi16(m1[1][0], m1[3][0]);
      m2[4][0] = _mm_add_epi16(m1[4][0], m1[6][0]);
      m2[5][0] = _mm_add_epi16(m1[5][0], m1[7][0]);
      m2[6][0] = _mm_sub_epi16(m1[4][0], m1[6][0]);
      m2[7][0] = _mm_sub_epi16(m1[5][0], m1[7][0]); // 13 bit

      m1[0][0] = _mm_add_epi16(m2[0][0], m2[1][0]);
      m1[1][0] = _mm_sub_epi16(m2[0][0], m2[1][0]);
      m1[2][0] = _mm_add_epi16(m2[2][0], m2[3][0]);
      m1[3][0] = _mm_sub_epi16(m2[2][0], m2[3][0]);
      m1[4][0] = _mm_add_epi16(m2[4][0], m2[5][0]);
      m1[5][0] = _mm_sub_epi16(m2[4][0], m2[5][0]);
      m1[6][0] = _mm_add_epi16(m2[6][0], m2[7][0]);
      m1[7][0] = _mm_sub_epi16(m2[6][0], m2[7][0]); // 14 bit

      m2[0][0] = _mm_unpacklo_epi16(m1[0][0], m1[1][0]); // 0_0 1_0 0_1 1_1 0_2 1_2 0_3 1_3
      m2[1][0] = _mm_unpacklo_epi16(m1[2][0], m1[3][0]); // 2_0 3_0 2_1 3_1 2_2 3_2 2_3 3_3
      m2[2][0] = _mm_unpackhi_epi16(m1[0][0], m1[1][0]); // 0_4 1_4 0_5 1_5 0_6 1_6 0_7 1_7
      m2[3][0] = _mm_unpackhi_epi16(m1[2][0], m1[3][0]); // 2_4 3_4 2_5 3_5 2_6 3_6 2_7 3_7
      m2[4][0] = _mm_unpacklo_epi16(m1[4][0], m1[5][0]);
      m2[5][0] = _mm_unpacklo_epi16(m1[6][0], m1[7][0]);
      m2[6][0] = _mm_unpackhi_epi16(m1[4][0], m1[5][0]);
      m2[7][0] = _mm_unpackhi_epi16(m1[6][0], m1[7][0]);

      m1[0][0] = _mm_unpacklo_epi32(m2[0][0], m2[1][0]); // 0_0 1_0 2_0 3_0 0_1 1_1 2_1 3_1
      m1[1][0] = _mm_unpackhi_epi32(m2[0][0], m2[1][0]); // 0_2 1_2 2_2 3_2 0_3 1_3 2_3 3_3
      m1[2][0] = _mm_unpacklo_epi32(m2[2][0], m2[3][0]); // 0_4 1_4 2_4 3_4 0_5 1_5 2_5 3_5
      m1[3][0] = _mm_unpackhi_epi32(m2[2][0], m2[3][0]); // 0_6 1_6 2_6 3_6 0_7 1_7 2_7 3_7
      m1[4][0] = _mm_unpacklo_epi32(m2[4][0], m2[5][0]);
      m1[5][0] = _mm_unpackhi_epi32(m2[4][0], m2[5][0]);
      m1[6][0] = _mm_unpacklo_epi32(m2[6][0], m2[7][0]);
      m1[7][0] = _mm_unpackhi_epi32(m2[6][0], m2[7][0]);

      m2[0][0] = _mm_unpacklo_epi64(m1[0][0], m1[4][0]); // 0_0 1_0 2_0 3_0 4_0 5_0 6_0 7_0
      m2[1][0] = _mm_unpackhi_epi64(m1[0][0], m1[4][0]);
      m2[2][0] = _mm_unpacklo_epi64(m1[1][0], m1[5][0]);
      m2[3][0] = _mm_unpackhi_epi64(m1[1][0], m1[5][0]);
      m2[4][0] = _mm_unpacklo_epi64(m1[2][0], m1[6][0]);
      m2[5][0] = _mm_unpackhi_epi64(m1[2][0], m1[6][0]);
      m2[6][0] = _mm_unpacklo_epi64(m1[3][0], m1[7][0]);
      m2[7][0] = _mm_unpackhi_epi64(m1[3][0], m1[7][0]);
    }

    n1[0][0] = _mm_add_epi16(m2[0][0], m2[4][0]);
    n1[1][0] = _mm_add_epi16(m2[1][0], m2[5][0]);
    n1[2][0] = _mm_add_epi16(m2[2][0], m2[6][0]);
    n1[3][0] = _mm_add_epi16(m2[3][0], m2[7][0]);
    n1[4][0] = _mm_sub_epi16(m2[0][0], m2[4][0]);
    n1[5][0] = _mm_sub_epi16(m2[1][0], m2[5][0]);
    n1[6][0] = _mm_sub_epi16(m2[2][0], m2[6][0]);
    n1[7][0] = _mm_sub_epi16(m2[3][0], m2[7][0]); // 15 bit

    m1[0][0] = _mm_unpacklo_epi16(n1[0][0], n1[2][0]);
    m1[1][0] = _mm_unpackhi_epi16(n1[0][0], n1[2][0]);
    m1[2][0] = _mm_unpacklo_epi16(n1[1][0], n1[3][0]);
    m1[3][0] = _mm_unpackhi_epi16(n1[1][0], n1[3][0]);
    m1[4][0] = _mm_unpacklo_epi16(n1[4][0], n1[6][0]);
    m1[5][0] = _mm_unpackhi_epi16(n1[4][0], n1[6][0]);
    m1[6][0] = _mm_unpacklo_epi16(n1[5][0], n1[7][0]);
    m1[7][0] = _mm_unpackhi_epi16(n1[5][0], n1[7][0]);

    const __m128i xadd = _mm_setr_epi16(1, 1, 1, 1, 1, 1, 1, 1);
    const __m128i xsub = _mm_setr_epi16(1, -1, 1, -1, 1, -1, 1, -1);

    n2[0][0] = _mm_madd_epi16(m1[0][0], xadd);
    n2[0][1] = _mm_madd_epi16(m1[1][0], xadd);
    n2[1][0] = _mm_madd_epi16(m1[2][0], xadd);
    n2[1][1] = _mm_madd_epi16(m1[3][0], xadd);

    n2[2][0] = _mm_madd_epi16(m1[0][0], xsub);
    n2[2][1] = _mm_madd_epi16(m1[1][0], xsub);
    n2[3][0] = _mm_madd_epi16(m1[2][0], xsub);
    n2[3][1] = _mm_madd_epi16(m1[3][0], xsub);

    n2[4][0] = _mm_madd_epi16(m1[4][0], xadd);
    n2[4][1] = _mm_madd_epi16(m1[5][0], xadd);
    n2[5][0] = _mm_madd_epi16(m1[6][0], xadd);
    n2[5][1] = _mm_madd_epi16(m1[7][0], xadd);

    n2[6][0] = _mm_madd_epi16(m1[4][0], xsub);
    n2[6][1] = _mm_madd_epi16(m1[5][0], xsub);
    n2[7][0] = _mm_madd_epi16(m1[6][0], xsub);
    n2[7][1] = _mm_madd_epi16(m1[7][0], xsub);
  }

  for (int i = 0; i < 2; i++)
  {
    n1[0][i] = _mm_abs_epi32(_mm_add_epi32(n2[0][i], n2[1][i]));
    n1[1][i] = _mm_abs_epi32(_mm_sub_epi32(n2[0][i], n2[1][i]));
    n1[2][i] = _mm_abs_epi32(_mm_add_epi32(n2[2][i], n2[3][i]));
    n1[3][i] = _mm_abs_epi32(_mm_sub_epi32(n2[2][i], n2[3][i]));
    n1[4][i] = _mm_abs_epi32(_mm_add_epi32(n2[4][i], n2[5][i]));
    n1[5][i] = _mm_abs_epi32(_mm_sub_epi32(n2[4][i], n2[5][i]));
    n1[6][i] = _mm_abs_epi32(_mm_add_epi32(n2[6][i], n2[7][i]));
    n1[7][i] = _mm_abs_epi32(_mm_sub_epi32(n2[6][i], n2[7][i]));
  }
  for (int i = 0; i < 8; i++)
  {
    m1[i][0] = _mm_add_epi32(n1[i][0], n1[i][1]);
  }

  m1[0][0] = _mm_add_epi32(m1[0][0], m1[1][0]);
  m1[2][0] = _mm_add_epi32(m1[2][0], m1[3][0]);
  m1[4][0] = _mm_add_epi32(m1[4][0], m1[5][0]);
  m1[6][0] = _mm_add_epi32(m1[6][0], m1[7][0]);

  m1[0][0]    = _mm_add_epi32(m1[0][0], m1[2][0]);
  m1[4][0]    = _mm_add_epi32(m1[4][0], m1[6][0]);
  __m128i sum = _mm_add_epi32(m1[0][0], m1[4][0]);

  sum = _mm_hadd_epi32(sum, sum);
  sum = _mm_hadd_epi32(sum, sum);

  uint32_t sad = _mm_cvtsi128_si32(sum);
#if JVET_R0164_MEAN_SCALED_SATD
  uint32_t absDc = _mm_cvtsi128_si32(n1[0][0]);
  sad -= absDc;
  sad += absDc >> 2;
#endif
  sad = ((sad + 2) >> 2);

  return sad;
}

// working up to 12-bit
static uint32_t xCalcHAD16x8_SSE(const Torg *piOrg, const Tcur *piCur, const ptrdiff_t strideOrg,
                                 const ptrdiff_t strideCur, const int iBitDepth)
{
  __m128i m1[16][2][2], m2[16][2][2];
  __m128i sum = _mm_setzero_si128();

  for (int l = 0; l < 2; l++)
  {
    const Torg *piOrgPtr = piOrg + l * 8;
    const Tcur *piCurPtr = piCur + l * 8;
    for (int k = 0; k < 8; k++)
    {
      __m128i r0  = _mm_loadu_si128((__m128i *)piOrgPtr);
      __m128i r1  = _mm_lddqu_si128((__m128i *)piCurPtr);
      m2[k][l][0] = _mm_sub_epi16(r0, r1);
      m2[k][l][1] = _mm_cvtepi16_epi32(_mm_srli_si128(m2[k][l][0], 8));
      m2[k][l][0] = _mm_cvtepi16_epi32(m2[k][l][0]);
      piCurPtr += strideCur;
      piOrgPtr += strideOrg;
    }

    for (int i = 0; i < 2; i++)
    {
      // vertical
      m1[0][l][i] = _mm_add_epi32(m2[0][l][i], m2[4][l][i]);
      m1[1][l][i] = _mm_add_epi32(m2[1][l][i], m2[5][l][i]);
      m1[2][l][i] = _mm_add_epi32(m2[2][l][i], m2[6][l][i]);
      m1[3][l][i] = _mm_add_epi32(m2[3][l][i], m2[7][l][i]);
      m1[4][l][i] = _mm_sub_epi32(m2[0][l][i], m2[4][l][i]);
      m1[5][l][i] = _mm_sub_epi32(m2[1][l][i], m2[5][l][i]);
      m1[6][l][i] = _mm_sub_epi32(m2[2][l][i], m2[6][l][i]);
      m1[7][l][i] = _mm_sub_epi32(m2[3][l][i], m2[7][l][i]);

      m2[0][l][i] = _mm_add_epi32(m1[0][l][i], m1[2][l][i]);
      m2[1][l][i] = _mm_add_epi32(m1[1][l][i], m1[3][l][i]);
      m2[2][l][i] = _mm_sub_epi32(m1[0][l][i], m1[2][l][i]);
      m2[3][l][i] = _mm_sub_epi32(m1[1][l][i], m1[3][l][i]);
      m2[4][l][i] = _mm_add_epi32(m1[4][l][i], m1[6][l][i]);
      m2[5][l][i] = _mm_add_epi32(m1[5][l][i], m1[7][l][i]);
      m2[6][l][i] = _mm_sub_epi32(m1[4][l][i], m1[6][l][i]);
      m2[7][l][i] = _mm_sub_epi32(m1[5][l][i], m1[7][l][i]);

      m1[0][l][i] = _mm_add_epi32(m2[0][l][i], m2[1][l][i]);
      m1[1][l][i] = _mm_sub_epi32(m2[0][l][i], m2[1][l][i]);
      m1[2][l][i] = _mm_add_epi32(m2[2][l][i], m2[3][l][i]);
      m1[3][l][i] = _mm_sub_epi32(m2[2][l][i], m2[3][l][i]);
      m1[4][l][i] = _mm_add_epi32(m2[4][l][i], m2[5][l][i]);
      m1[5][l][i] = _mm_sub_epi32(m2[4][l][i], m2[5][l][i]);
      m1[6][l][i] = _mm_add_epi32(m2[6][l][i], m2[7][l][i]);
      m1[7][l][i] = _mm_sub_epi32(m2[6][l][i], m2[7][l][i]);
    }
  }

  // 4 x 8x4 blocks
  // 0 1
  // 2 3
#if JVET_R0164_MEAN_SCALED_SATD
  uint32_t absDc = 0;
#endif

  // transpose and do horizontal in two steps
  for (int l = 0; l < 2; l++)
  {
    int off = l * 4;

    __m128i n1[16];
    __m128i n2[16];

    m2[0][0][0] = _mm_unpacklo_epi32(m1[0 + off][0][0], m1[1 + off][0][0]);
    m2[1][0][0] = _mm_unpacklo_epi32(m1[2 + off][0][0], m1[3 + off][0][0]);
    m2[2][0][0] = _mm_unpackhi_epi32(m1[0 + off][0][0], m1[1 + off][0][0]);
    m2[3][0][0] = _mm_unpackhi_epi32(m1[2 + off][0][0], m1[3 + off][0][0]);

    m2[0][0][1] = _mm_unpacklo_epi32(m1[0 + off][0][1], m1[1 + off][0][1]);
    m2[1][0][1] = _mm_unpacklo_epi32(m1[2 + off][0][1], m1[3 + off][0][1]);
    m2[2][0][1] = _mm_unpackhi_epi32(m1[0 + off][0][1], m1[1 + off][0][1]);
    m2[3][0][1] = _mm_unpackhi_epi32(m1[2 + off][0][1], m1[3 + off][0][1]);

    n1[0] = _mm_unpacklo_epi64(m2[0][0][0], m2[1][0][0]);
    n1[1] = _mm_unpackhi_epi64(m2[0][0][0], m2[1][0][0]);
    n1[2] = _mm_unpacklo_epi64(m2[2][0][0], m2[3][0][0]);
    n1[3] = _mm_unpackhi_epi64(m2[2][0][0], m2[3][0][0]);
    n1[4] = _mm_unpacklo_epi64(m2[0][0][1], m2[1][0][1]);
    n1[5] = _mm_unpackhi_epi64(m2[0][0][1], m2[1][0][1]);
    n1[6] = _mm_unpacklo_epi64(m2[2][0][1], m2[3][0][1]);
    n1[7] = _mm_unpackhi_epi64(m2[2][0][1], m2[3][0][1]);

    // transpose 8x4 -> 4x8, block 1(3)
    m2[8 + 0][0][0] = _mm_unpacklo_epi32(m1[0 + off][1][0], m1[1 + off][1][0]);
    m2[8 + 1][0][0] = _mm_unpacklo_epi32(m1[2 + off][1][0], m1[3 + off][1][0]);
    m2[8 + 2][0][0] = _mm_unpackhi_epi32(m1[0 + off][1][0], m1[1 + off][1][0]);
    m2[8 + 3][0][0] = _mm_unpackhi_epi32(m1[2 + off][1][0], m1[3 + off][1][0]);

    m2[8 + 0][0][1] = _mm_unpacklo_epi32(m1[0 + off][1][1], m1[1 + off][1][1]);
    m2[8 + 1][0][1] = _mm_unpacklo_epi32(m1[2 + off][1][1], m1[3 + off][1][1]);
    m2[8 + 2][0][1] = _mm_unpackhi_epi32(m1[0 + off][1][1], m1[1 + off][1][1]);
    m2[8 + 3][0][1] = _mm_unpackhi_epi32(m1[2 + off][1][1], m1[3 + off][1][1]);

    n1[8 + 0] = _mm_unpacklo_epi64(m2[8 + 0][0][0], m2[8 + 1][0][0]);
    n1[8 + 1] = _mm_unpackhi_epi64(m2[8 + 0][0][0], m2[8 + 1][0][0]);
    n1[8 + 2] = _mm_unpacklo_epi64(m2[8 + 2][0][0], m2[8 + 3][0][0]);
    n1[8 + 3] = _mm_unpackhi_epi64(m2[8 + 2][0][0], m2[8 + 3][0][0]);
    n1[8 + 4] = _mm_unpacklo_epi64(m2[8 + 0][0][1], m2[8 + 1][0][1]);
    n1[8 + 5] = _mm_unpackhi_epi64(m2[8 + 0][0][1], m2[8 + 1][0][1]);
    n1[8 + 6] = _mm_unpacklo_epi64(m2[8 + 2][0][1], m2[8 + 3][0][1]);
    n1[8 + 7] = _mm_unpackhi_epi64(m2[8 + 2][0][1], m2[8 + 3][0][1]);

    n2[0]  = _mm_add_epi32(n1[0], n1[8]);
    n2[1]  = _mm_add_epi32(n1[1], n1[9]);
    n2[2]  = _mm_add_epi32(n1[2], n1[10]);
    n2[3]  = _mm_add_epi32(n1[3], n1[11]);
    n2[4]  = _mm_add_epi32(n1[4], n1[12]);
    n2[5]  = _mm_add_epi32(n1[5], n1[13]);
    n2[6]  = _mm_add_epi32(n1[6], n1[14]);
    n2[7]  = _mm_add_epi32(n1[7], n1[15]);
    n2[8]  = _mm_sub_epi32(n1[0], n1[8]);
    n2[9]  = _mm_sub_epi32(n1[1], n1[9]);
    n2[10] = _mm_sub_epi32(n1[2], n1[10]);
    n2[11] = _mm_sub_epi32(n1[3], n1[11]);
    n2[12] = _mm_sub_epi32(n1[4], n1[12]);
    n2[13] = _mm_sub_epi32(n1[5], n1[13]);
    n2[14] = _mm_sub_epi32(n1[6], n1[14]);
    n2[15] = _mm_sub_epi32(n1[7], n1[15]);

    n1[0]  = _mm_add_epi32(n2[0], n2[4]);
    n1[1]  = _mm_add_epi32(n2[1], n2[5]);
    n1[2]  = _mm_add_epi32(n2[2], n2[6]);
    n1[3]  = _mm_add_epi32(n2[3], n2[7]);
    n1[4]  = _mm_sub_epi32(n2[0], n2[4]);
    n1[5]  = _mm_sub_epi32(n2[1], n2[5]);
    n1[6]  = _mm_sub_epi32(n2[2], n2[6]);
    n1[7]  = _mm_sub_epi32(n2[3], n2[7]);
    n1[8]  = _mm_add_epi32(n2[8], n2[12]);
    n1[9]  = _mm_add_epi32(n2[9], n2[13]);
    n1[10] = _mm_add_epi32(n2[10], n2[14]);
    n1[11] = _mm_add_epi32(n2[11], n2[15]);
    n1[12] = _mm_sub_epi32(n2[8], n2[12]);
    n1[13] = _mm_sub_epi32(n2[9], n2[13]);
    n1[14] = _mm_sub_epi32(n2[10], n2[14]);
    n1[15] = _mm_sub_epi32(n2[11], n2[15]);

    n2[0]  = _mm_add_epi32(n1[0], n1[2]);
    n2[1]  = _mm_add_epi32(n1[1], n1[3]);
    n2[2]  = _mm_sub_epi32(n1[0], n1[2]);
    n2[3]  = _mm_sub_epi32(n1[1], n1[3]);
    n2[4]  = _mm_add_epi32(n1[4], n1[6]);
    n2[5]  = _mm_add_epi32(n1[5], n1[7]);
    n2[6]  = _mm_sub_epi32(n1[4], n1[6]);
    n2[7]  = _mm_sub_epi32(n1[5], n1[7]);
    n2[8]  = _mm_add_epi32(n1[8], n1[10]);
    n2[9]  = _mm_add_epi32(n1[9], n1[11]);
    n2[10] = _mm_sub_epi32(n1[8], n1[10]);
    n2[11] = _mm_sub_epi32(n1[9], n1[11]);
    n2[12] = _mm_add_epi32(n1[12], n1[14]);
    n2[13] = _mm_add_epi32(n1[13], n1[15]);
    n2[14] = _mm_sub_epi32(n1[12], n1[14]);
    n2[15] = _mm_sub_epi32(n1[13], n1[15]);

    n1[0]  = _mm_abs_epi32(_mm_add_epi32(n2[0], n2[1]));
    n1[1]  = _mm_abs_epi32(_mm_sub_epi32(n2[0], n2[1]));
    n1[2]  = _mm_abs_epi32(_mm_add_epi32(n2[2], n2[3]));
    n1[3]  = _mm_abs_epi32(_mm_sub_epi32(n2[2], n2[3]));
    n1[4]  = _mm_abs_epi32(_mm_add_epi32(n2[4], n2[5]));
    n1[5]  = _mm_abs_epi32(_mm_sub_epi32(n2[4], n2[5]));
    n1[6]  = _mm_abs_epi32(_mm_add_epi32(n2[6], n2[7]));
    n1[7]  = _mm_abs_epi32(_mm_sub_epi32(n2[6], n2[7]));
    n1[8]  = _mm_abs_epi32(_mm_add_epi32(n2[8], n2[9]));
    n1[9]  = _mm_abs_epi32(_mm_sub_epi32(n2[8], n2[9]));
    n1[10] = _mm_abs_epi32(_mm_add_epi32(n2[10], n2[11]));
    n1[11] = _mm_abs_epi32(_mm_sub_epi32(n2[10], n2[11]));
    n1[12] = _mm_abs_epi32(_mm_add_epi32(n2[12], n2[13]));
    n1[13] = _mm_abs_epi32(_mm_sub_epi32(n2[12], n2[13]));
    n1[14] = _mm_abs_epi32(_mm_add_epi32(n2[14], n2[15]));
    n1[15] = _mm_abs_epi32(_mm_sub_epi32(n2[14], n2[15]));

#if JVET_R0164_MEAN_SCALED_SATD
    if (l == 0)
    {
      absDc = _mm_cvtsi128_si32(n1[0]);
    }
#endif

    // sum up
    n1[0]  = _mm_add_epi32(n1[0], n1[1]);
    n1[2]  = _mm_add_epi32(n1[2], n1[3]);
    n1[4]  = _mm_add_epi32(n1[4], n1[5]);
    n1[6]  = _mm_add_epi32(n1[6], n1[7]);
    n1[8]  = _mm_add_epi32(n1[8], n1[9]);
    n1[10] = _mm_add_epi32(n1[10], n1[11]);
    n1[12] = _mm_add_epi32(n1[12], n1[13]);
    n1[14] = _mm_add_epi32(n1[14], n1[15]);

    n1[0]  = _mm_add_epi32(n1[0], n1[2]);
    n1[4]  = _mm_add_epi32(n1[4], n1[6]);
    n1[8]  = _mm_add_epi32(n1[8], n1[10]);
    n1[12] = _mm_add_epi32(n1[12], n1[14]);

    n1[0] = _mm_add_epi32(n1[0], n1[4]);
    n1[8] = _mm_add_epi32(n1[8], n1[12]);

    n1[0] = _mm_add_epi32(n1[0], n1[8]);
    sum   = _mm_add_epi32(sum, n1[0]);
  }

  sum = _mm_hadd_epi32(sum, sum);
  sum = _mm_hadd_epi32(sum, sum);

  uint32_t sad = _mm_cvtsi128_si32(sum);

#if JVET_R0164_MEAN_SCALED_SATD
  sad -= absDc;
  sad += absDc >> 2;
#endif
  sad = sad * INV_SQRT_2 >> 32;
  sad >>= 2;

  return sad;
}

// working up to 12-bit
static uint32_t xCalcHAD8x16_SSE(const Torg *piOrg, const Tcur *piCur, const ptrdiff_t strideOrg,
                                 const ptrdiff_t strideCur, const int iBitDepth)
{
  __m128i m1[2][16], m2[2][16];
  __m128i sum = _mm_setzero_si128();

  for (int k = 0; k < 16; k++)
  {
    __m128i r0 = _mm_loadu_si128((__m128i *)piOrg);
    __m128i r1 = _mm_lddqu_si128((__m128i *)piCur);
    m1[0][k]   = _mm_sub_epi16(r0, r1);
    m1[1][k]   = _mm_cvtepi16_epi32(_mm_srli_si128(m1[0][k], 8));
    m1[0][k]   = _mm_cvtepi16_epi32(m1[0][k]);
    piCur += strideCur;
    piOrg += strideOrg;
  }

  for (int i = 0; i < 2; i++)
  {
    // vertical
    m2[i][0]  = _mm_add_epi32(m1[i][0], m1[i][8]);
    m2[i][1]  = _mm_add_epi32(m1[i][1], m1[i][9]);
    m2[i][2]  = _mm_add_epi32(m1[i][2], m1[i][10]);
    m2[i][3]  = _mm_add_epi32(m1[i][3], m1[i][11]);
    m2[i][4]  = _mm_add_epi32(m1[i][4], m1[i][12]);
    m2[i][5]  = _mm_add_epi32(m1[i][5], m1[i][13]);
    m2[i][6]  = _mm_add_epi32(m1[i][6], m1[i][14]);
    m2[i][7]  = _mm_add_epi32(m1[i][7], m1[i][15]);
    m2[i][8]  = _mm_sub_epi32(m1[i][0], m1[i][8]);
    m2[i][9]  = _mm_sub_epi32(m1[i][1], m1[i][9]);
    m2[i][10] = _mm_sub_epi32(m1[i][2], m1[i][10]);
    m2[i][11] = _mm_sub_epi32(m1[i][3], m1[i][11]);
    m2[i][12] = _mm_sub_epi32(m1[i][4], m1[i][12]);
    m2[i][13] = _mm_sub_epi32(m1[i][5], m1[i][13]);
    m2[i][14] = _mm_sub_epi32(m1[i][6], m1[i][14]);
    m2[i][15] = _mm_sub_epi32(m1[i][7], m1[i][15]);

    m1[i][0]  = _mm_add_epi32(m2[i][0], m2[i][4]);
    m1[i][1]  = _mm_add_epi32(m2[i][1], m2[i][5]);
    m1[i][2]  = _mm_add_epi32(m2[i][2], m2[i][6]);
    m1[i][3]  = _mm_add_epi32(m2[i][3], m2[i][7]);
    m1[i][4]  = _mm_sub_epi32(m2[i][0], m2[i][4]);
    m1[i][5]  = _mm_sub_epi32(m2[i][1], m2[i][5]);
    m1[i][6]  = _mm_sub_epi32(m2[i][2], m2[i][6]);
    m1[i][7]  = _mm_sub_epi32(m2[i][3], m2[i][7]);
    m1[i][8]  = _mm_add_epi32(m2[i][8], m2[i][12]);
    m1[i][9]  = _mm_add_epi32(m2[i][9], m2[i][13]);
    m1[i][10] = _mm_add_epi32(m2[i][10], m2[i][14]);
    m1[i][11] = _mm_add_epi32(m2[i][11], m2[i][15]);
    m1[i][12] = _mm_sub_epi32(m2[i][8], m2[i][12]);
    m1[i][13] = _mm_sub_epi32(m2[i][9], m2[i][13]);
    m1[i][14] = _mm_sub_epi32(m2[i][10], m2[i][14]);
    m1[i][15] = _mm_sub_epi32(m2[i][11], m2[i][15]);

    m2[i][0]  = _mm_add_epi32(m1[i][0], m1[i][2]);
    m2[i][1]  = _mm_add_epi32(m1[i][1], m1[i][3]);
    m2[i][2]  = _mm_sub_epi32(m1[i][0], m1[i][2]);
    m2[i][3]  = _mm_sub_epi32(m1[i][1], m1[i][3]);
    m2[i][4]  = _mm_add_epi32(m1[i][4], m1[i][6]);
    m2[i][5]  = _mm_add_epi32(m1[i][5], m1[i][7]);
    m2[i][6]  = _mm_sub_epi32(m1[i][4], m1[i][6]);
    m2[i][7]  = _mm_sub_epi32(m1[i][5], m1[i][7]);
    m2[i][8]  = _mm_add_epi32(m1[i][8], m1[i][10]);
    m2[i][9]  = _mm_add_epi32(m1[i][9], m1[i][11]);
    m2[i][10] = _mm_sub_epi32(m1[i][8], m1[i][10]);
    m2[i][11] = _mm_sub_epi32(m1[i][9], m1[i][11]);
    m2[i][12] = _mm_add_epi32(m1[i][12], m1[i][14]);
    m2[i][13] = _mm_add_epi32(m1[i][13], m1[i][15]);
    m2[i][14] = _mm_sub_epi32(m1[i][12], m1[i][14]);
    m2[i][15] = _mm_sub_epi32(m1[i][13], m1[i][15]);

    m1[i][0]  = _mm_add_epi32(m2[i][0], m2[i][1]);
    m1[i][1]  = _mm_sub_epi32(m2[i][0], m2[i][1]);
    m1[i][2]  = _mm_add_epi32(m2[i][2], m2[i][3]);
    m1[i][3]  = _mm_sub_epi32(m2[i][2], m2[i][3]);
    m1[i][4]  = _mm_add_epi32(m2[i][4], m2[i][5]);
    m1[i][5]  = _mm_sub_epi32(m2[i][4], m2[i][5]);
    m1[i][6]  = _mm_add_epi32(m2[i][6], m2[i][7]);
    m1[i][7]  = _mm_sub_epi32(m2[i][6], m2[i][7]);
    m1[i][8]  = _mm_add_epi32(m2[i][8], m2[i][9]);
    m1[i][9]  = _mm_sub_epi32(m2[i][8], m2[i][9]);
    m1[i][10] = _mm_add_epi32(m2[i][10], m2[i][11]);
    m1[i][11] = _mm_sub_epi32(m2[i][10], m2[i][11]);
    m1[i][12] = _mm_add_epi32(m2[i][12], m2[i][13]);
    m1[i][13] = _mm_sub_epi32(m2[i][12], m2[i][13]);
    m1[i][14] = _mm_add_epi32(m2[i][14], m2[i][15]);
    m1[i][15] = _mm_sub_epi32(m2[i][14], m2[i][15]);
  }

  // process horizontal in two steps ( 2 x 8x8 blocks )

  for (int l = 0; l < 4; l++)
  {
    int off = l * 4;

    for (int i = 0; i < 2; i++)
    {
      // transpose 4x4
      m2[i][0 + off] = _mm_unpacklo_epi32(m1[i][0 + off], m1[i][1 + off]);
      m2[i][1 + off] = _mm_unpackhi_epi32(m1[i][0 + off], m1[i][1 + off]);
      m2[i][2 + off] = _mm_unpacklo_epi32(m1[i][2 + off], m1[i][3 + off]);
      m2[i][3 + off] = _mm_unpackhi_epi32(m1[i][2 + off], m1[i][3 + off]);

      m1[i][0 + off] = _mm_unpacklo_epi64(m2[i][0 + off], m2[i][2 + off]);
      m1[i][1 + off] = _mm_unpackhi_epi64(m2[i][0 + off], m2[i][2 + off]);
      m1[i][2 + off] = _mm_unpacklo_epi64(m2[i][1 + off], m2[i][3 + off]);
      m1[i][3 + off] = _mm_unpackhi_epi64(m2[i][1 + off], m2[i][3 + off]);
    }
  }

#if JVET_R0164_MEAN_SCALED_SATD
  uint32_t absDc = 0;
#endif

  for (int l = 0; l < 2; l++)
  {
    int off = l * 8;

    __m128i n1[2][8];
    __m128i n2[2][8];

    for (int i = 0; i < 8; i++)
    {
      int ii = i % 4;
      int ij = i >> 2;

      n2[0][i] = m1[ij][off + ii];
      n2[1][i] = m1[ij][off + ii + 4];
    }

    for (int i = 0; i < 2; i++)
    {
      n1[i][0] = _mm_add_epi32(n2[i][0], n2[i][4]);
      n1[i][1] = _mm_add_epi32(n2[i][1], n2[i][5]);
      n1[i][2] = _mm_add_epi32(n2[i][2], n2[i][6]);
      n1[i][3] = _mm_add_epi32(n2[i][3], n2[i][7]);
      n1[i][4] = _mm_sub_epi32(n2[i][0], n2[i][4]);
      n1[i][5] = _mm_sub_epi32(n2[i][1], n2[i][5]);
      n1[i][6] = _mm_sub_epi32(n2[i][2], n2[i][6]);
      n1[i][7] = _mm_sub_epi32(n2[i][3], n2[i][7]);

      n2[i][0] = _mm_add_epi32(n1[i][0], n1[i][2]);
      n2[i][1] = _mm_add_epi32(n1[i][1], n1[i][3]);
      n2[i][2] = _mm_sub_epi32(n1[i][0], n1[i][2]);
      n2[i][3] = _mm_sub_epi32(n1[i][1], n1[i][3]);
      n2[i][4] = _mm_add_epi32(n1[i][4], n1[i][6]);
      n2[i][5] = _mm_add_epi32(n1[i][5], n1[i][7]);
      n2[i][6] = _mm_sub_epi32(n1[i][4], n1[i][6]);
      n2[i][7] = _mm_sub_epi32(n1[i][5], n1[i][7]);

      n1[i][0] = _mm_abs_epi32(_mm_add_epi32(n2[i][0], n2[i][1]));
      n1[i][1] = _mm_abs_epi32(_mm_sub_epi32(n2[i][0], n2[i][1]));
      n1[i][2] = _mm_abs_epi32(_mm_add_epi32(n2[i][2], n2[i][3]));
      n1[i][3] = _mm_abs_epi32(_mm_sub_epi32(n2[i][2], n2[i][3]));
      n1[i][4] = _mm_abs_epi32(_mm_add_epi32(n2[i][4], n2[i][5]));
      n1[i][5] = _mm_abs_epi32(_mm_sub_epi32(n2[i][4], n2[i][5]));
      n1[i][6] = _mm_abs_epi32(_mm_add_epi32(n2[i][6], n2[i][7]));
      n1[i][7] = _mm_abs_epi32(_mm_sub_epi32(n2[i][6], n2[i][7]));

#if JVET_R0164_MEAN_SCALED_SATD
      if (l + i == 0)
      {
        absDc = _mm_cvtsi128_si32(n1[i][0]);
      }
#endif
    }

    for (int i = 0; i < 8; i++)
    {
      n2[0][i] = _mm_add_epi32(n1[0][i], n1[1][i]);
    }

    n2[0][0] = _mm_add_epi32(n2[0][0], n2[0][1]);
    n2[0][2] = _mm_add_epi32(n2[0][2], n2[0][3]);
    n2[0][4] = _mm_add_epi32(n2[0][4], n2[0][5]);
    n2[0][6] = _mm_add_epi32(n2[0][6], n2[0][7]);

    n2[0][0] = _mm_add_epi32(n2[0][0], n2[0][2]);
    n2[0][4] = _mm_add_epi32(n2[0][4], n2[0][6]);
    sum      = _mm_add_epi32(sum, _mm_add_epi32(n2[0][0], n2[0][4]));
  }

  sum = _mm_hadd_epi32(sum, sum);
  sum = _mm_hadd_epi32(sum, sum);

  uint32_t sad = _mm_cvtsi128_si32(sum);

#if JVET_R0164_MEAN_SCALED_SATD
  sad -= absDc;
  sad += absDc >> 2;
#endif
  sad = sad * INV_SQRT_2 >> 32;
  sad >>= 2;

  return sad;
}

template<bool sbsplh> static uint32_t xCalcHAD8x4_SSE(const Pel *piOrg, const Pel *piCur, const ptrdiff_t strideOrg,
                                                      const ptrdiff_t strideCur, const int iBitDepth)
{
  __m128i m1[8], m2[8];
  __m128i vzero = _mm_setzero_si128();

  for (int k = 0; k < 4; k++)
  {
    __m128i r0, r1;

    if (sbsplh)
    {
      r0         = _mm_loadu_si128((const __m128i *)piOrg);
      r1         = _mm_loadu_si128((const __m128i *)piCur);
      __m128i r2 = _mm_loadu_si128((const __m128i *)(piOrg + 8));
      __m128i r3 = _mm_loadu_si128((const __m128i *)(piCur + 8));

      r0 = _mm_hadd_epi16(r0, r2);
      r1 = _mm_hadd_epi16(r1, r3);

#if !SUB_PRE_ROUND
      r0 = _mm_add_epi16(r0, _mm_set1_epi16(1));
      r1 = _mm_add_epi16(r1, _mm_set1_epi16(1));
      r0 = _mm_srai_epi16(r0, 1);
      r1 = _mm_srai_epi16(r1, 1);
#else
      r0    = _mm_sub_epi16(r0, r1);
      r0    = _mm_add_epi16(r0, _mm_sign_epi16(_mm_set1_epi16(1), r0));
      r0    = _mm_srai_epi16(r0, 1);
      m1[k] = r0;
#endif
    }
    else
    {
      r0 = _mm_loadu_si128((const __m128i *)piOrg);
      r1 = _mm_loadu_si128((const __m128i *)piCur);
#if SUB_PRE_ROUND
      m1[k] = _mm_sub_epi16(r0, r1);
#endif
    }

#if !SUB_PRE_ROUND
    m1[k] = _mm_sub_epi16(r0, r1);
#endif
    piCur += strideCur;
    piOrg += strideOrg;
  }

  // vertical
  m2[0] = _mm_add_epi16(m1[0], m1[2]);
  m2[1] = _mm_add_epi16(m1[1], m1[3]);
  m2[2] = _mm_sub_epi16(m1[0], m1[2]);
  m2[3] = _mm_sub_epi16(m1[1], m1[3]);

  m1[0] = _mm_add_epi16(m2[0], m2[1]);
  m1[1] = _mm_sub_epi16(m2[0], m2[1]);
  m1[2] = _mm_add_epi16(m2[2], m2[3]);
  m1[3] = _mm_sub_epi16(m2[2], m2[3]);

  // transpose, partially
  {
    m2[0] = _mm_unpacklo_epi16(m1[0], m1[1]);
    m2[1] = _mm_unpacklo_epi16(m1[2], m1[3]);
    m2[2] = _mm_unpackhi_epi16(m1[0], m1[1]);
    m2[3] = _mm_unpackhi_epi16(m1[2], m1[3]);

    m1[0] = _mm_unpacklo_epi32(m2[0], m2[1]);
    m1[1] = _mm_unpackhi_epi32(m2[0], m2[1]);
    m1[2] = _mm_unpacklo_epi32(m2[2], m2[3]);
    m1[3] = _mm_unpackhi_epi32(m2[2], m2[3]);
  }

  // horizontal
  if (iBitDepth >= 10 /*sizeof( Torg ) > 1 || sizeof( Tcur ) > 1*/)
  {
    // finish transpose
    m2[0] = _mm_unpacklo_epi64(m1[0], vzero);
    m2[1] = _mm_unpackhi_epi64(m1[0], vzero);
    m2[2] = _mm_unpacklo_epi64(m1[1], vzero);
    m2[3] = _mm_unpackhi_epi64(m1[1], vzero);
    m2[4] = _mm_unpacklo_epi64(m1[2], vzero);
    m2[5] = _mm_unpackhi_epi64(m1[2], vzero);
    m2[6] = _mm_unpacklo_epi64(m1[3], vzero);
    m2[7] = _mm_unpackhi_epi64(m1[3], vzero);

    for (int i = 0; i < 8; i++)
    {
      m2[i] = _mm_cvtepi16_epi32(m2[i]);
    }

    m1[0] = _mm_add_epi32(m2[0], m2[4]);
    m1[1] = _mm_add_epi32(m2[1], m2[5]);
    m1[2] = _mm_add_epi32(m2[2], m2[6]);
    m1[3] = _mm_add_epi32(m2[3], m2[7]);
    m1[4] = _mm_sub_epi32(m2[0], m2[4]);
    m1[5] = _mm_sub_epi32(m2[1], m2[5]);
    m1[6] = _mm_sub_epi32(m2[2], m2[6]);
    m1[7] = _mm_sub_epi32(m2[3], m2[7]);

    m2[0] = _mm_add_epi32(m1[0], m1[2]);
    m2[1] = _mm_add_epi32(m1[1], m1[3]);
    m2[2] = _mm_sub_epi32(m1[0], m1[2]);
    m2[3] = _mm_sub_epi32(m1[1], m1[3]);
    m2[4] = _mm_add_epi32(m1[4], m1[6]);
    m2[5] = _mm_add_epi32(m1[5], m1[7]);
    m2[6] = _mm_sub_epi32(m1[4], m1[6]);
    m2[7] = _mm_sub_epi32(m1[5], m1[7]);

    m1[0] = _mm_abs_epi32(_mm_add_epi32(m2[0], m2[1]));
    m1[1] = _mm_abs_epi32(_mm_sub_epi32(m2[0], m2[1]));
    m1[2] = _mm_abs_epi32(_mm_add_epi32(m2[2], m2[3]));
    m1[3] = _mm_abs_epi32(_mm_sub_epi32(m2[2], m2[3]));
    m1[4] = _mm_abs_epi32(_mm_add_epi32(m2[4], m2[5]));
    m1[5] = _mm_abs_epi32(_mm_sub_epi32(m2[4], m2[5]));
    m1[6] = _mm_abs_epi32(_mm_add_epi32(m2[6], m2[7]));
    m1[7] = _mm_abs_epi32(_mm_sub_epi32(m2[6], m2[7]));
  }
  else
  {
    m2[0] = _mm_add_epi16(m1[0], m1[2]);
    m2[1] = _mm_add_epi16(m1[1], m1[3]);
    m2[2] = _mm_sub_epi16(m1[0], m1[2]);
    m2[3] = _mm_sub_epi16(m1[1], m1[3]);

    m1[0] = _mm_add_epi16(m2[0], m2[1]);
    m1[1] = _mm_sub_epi16(m2[0], m2[1]);
    m1[2] = _mm_add_epi16(m2[2], m2[3]);
    m1[3] = _mm_sub_epi16(m2[2], m2[3]);

    // finish transpose
    m2[0] = _mm_unpacklo_epi64(m1[0], vzero);
    m2[1] = _mm_unpackhi_epi64(m1[0], vzero);
    m2[2] = _mm_unpacklo_epi64(m1[1], vzero);
    m2[3] = _mm_unpackhi_epi64(m1[1], vzero);
    m2[4] = _mm_unpacklo_epi64(m1[2], vzero);
    m2[5] = _mm_unpackhi_epi64(m1[2], vzero);
    m2[6] = _mm_unpacklo_epi64(m1[3], vzero);
    m2[7] = _mm_unpackhi_epi64(m1[3], vzero);

    m1[0] = _mm_abs_epi16(_mm_add_epi16(m2[0], m2[1]));
    m1[1] = _mm_abs_epi16(_mm_sub_epi16(m2[0], m2[1]));
    m1[2] = _mm_abs_epi16(_mm_add_epi16(m2[2], m2[3]));
    m1[3] = _mm_abs_epi16(_mm_sub_epi16(m2[2], m2[3]));
    m1[4] = _mm_abs_epi16(_mm_add_epi16(m2[4], m2[5]));
    m1[5] = _mm_abs_epi16(_mm_sub_epi16(m2[4], m2[5]));
    m1[6] = _mm_abs_epi16(_mm_add_epi16(m2[6], m2[7]));
    m1[7] = _mm_abs_epi16(_mm_sub_epi16(m2[6], m2[7]));

    for (int i = 0; i < 8; i++)
    {
      m1[i] = _mm_unpacklo_epi16(m1[i], vzero);
    }
  }

#if JVET_R0164_MEAN_SCALED_SATD
  uint32_t absDc = _mm_cvtsi128_si32(m1[0]);
#endif

  m1[0] = _mm_add_epi32(m1[0], m1[1]);
  m1[1] = _mm_add_epi32(m1[2], m1[3]);
  m1[2] = _mm_add_epi32(m1[4], m1[5]);
  m1[3] = _mm_add_epi32(m1[6], m1[7]);

  m1[0] = _mm_add_epi32(m1[0], m1[1]);
  m1[1] = _mm_add_epi32(m1[2], m1[3]);

  __m128i sum = _mm_add_epi32(m1[0], m1[1]);

  sum = _mm_hadd_epi32(sum, sum);
  sum = _mm_hadd_epi32(sum, sum);

  uint32_t sad = _mm_cvtsi128_si32(sum);
  // sad = ((sad + 2) >> 2);
#if JVET_R0164_MEAN_SCALED_SATD
  sad -= absDc;
  sad += absDc >> 2;
#endif
  sad = sad * INV_SQRT_2 >> 32;
  sad >>= 1;

  return sad;
}

template<bool sbsplv> static uint32_t xCalcHAD4x8_SSE(const Pel *piOrg, const Pel *piCur, const ptrdiff_t strideOrg,
                                                      const ptrdiff_t strideCur, const int iBitDepth)
{
  __m128i m1[8], m2[8];

  for (int k = 0; k < 8; k++)
  {
    __m128i r0, r1;

    if (sbsplv)
    {
      r0         = _mm_loadl_epi64((const __m128i *)piOrg);
      r1         = _mm_loadl_epi64((const __m128i *)piCur);
      __m128i r2 = _mm_loadl_epi64((const __m128i *)(piOrg + strideOrg));
      __m128i r3 = _mm_loadl_epi64((const __m128i *)(piCur + strideCur));

      r0 = _mm_add_epi16(r0, r2);
      r1 = _mm_add_epi16(r1, r3);

#if !SUB_PRE_ROUND
      r0 = _mm_add_epi16(r0, _mm_set1_epi16(1));
      r1 = _mm_add_epi16(r1, _mm_set1_epi16(1));
      r0 = _mm_srai_epi16(r0, 1);
      r1 = _mm_srai_epi16(r1, 1);
#else
      r0    = _mm_sub_epi16(r0, r1);
      r0    = _mm_add_epi16(r0, _mm_sign_epi16(_mm_set1_epi16(1), r0));
      r0    = _mm_srai_epi16(r0, 1);
      m2[k] = r0;
#endif
    }
    else
    {
      r0 = _mm_loadl_epi64((const __m128i *)piOrg);
      r1 = _mm_loadl_epi64((const __m128i *)piCur);
#if SUB_PRE_ROUND
      m2[k] = _mm_sub_epi16(r0, r1);
#endif
    }

#if !SUB_PRE_ROUND
    m2[k] = _mm_sub_epi16(r0, r1);
#endif
    piCur += strideCur << (sbsplv ? 1 : 0);
    piOrg += strideOrg << (sbsplv ? 1 : 0);
  }

  // vertical

  m1[0] = _mm_add_epi16(m2[0], m2[4]);
  m1[1] = _mm_add_epi16(m2[1], m2[5]);
  m1[2] = _mm_add_epi16(m2[2], m2[6]);
  m1[3] = _mm_add_epi16(m2[3], m2[7]);
  m1[4] = _mm_sub_epi16(m2[0], m2[4]);
  m1[5] = _mm_sub_epi16(m2[1], m2[5]);
  m1[6] = _mm_sub_epi16(m2[2], m2[6]);
  m1[7] = _mm_sub_epi16(m2[3], m2[7]);

  m2[0] = _mm_add_epi16(m1[0], m1[2]);
  m2[1] = _mm_add_epi16(m1[1], m1[3]);
  m2[2] = _mm_sub_epi16(m1[0], m1[2]);
  m2[3] = _mm_sub_epi16(m1[1], m1[3]);
  m2[4] = _mm_add_epi16(m1[4], m1[6]);
  m2[5] = _mm_add_epi16(m1[5], m1[7]);
  m2[6] = _mm_sub_epi16(m1[4], m1[6]);
  m2[7] = _mm_sub_epi16(m1[5], m1[7]);

  m1[0] = _mm_add_epi16(m2[0], m2[1]);
  m1[1] = _mm_sub_epi16(m2[0], m2[1]);
  m1[2] = _mm_add_epi16(m2[2], m2[3]);
  m1[3] = _mm_sub_epi16(m2[2], m2[3]);
  m1[4] = _mm_add_epi16(m2[4], m2[5]);
  m1[5] = _mm_sub_epi16(m2[4], m2[5]);
  m1[6] = _mm_add_epi16(m2[6], m2[7]);
  m1[7] = _mm_sub_epi16(m2[6], m2[7]);

  // horizontal
  // transpose
  {
    m2[0] = _mm_unpacklo_epi16(m1[0], m1[1]);
    m2[1] = _mm_unpacklo_epi16(m1[2], m1[3]);
    m2[2] = _mm_unpacklo_epi16(m1[4], m1[5]);
    m2[3] = _mm_unpacklo_epi16(m1[6], m1[7]);

    m1[0] = _mm_unpacklo_epi32(m2[0], m2[1]);
    m1[1] = _mm_unpackhi_epi32(m2[0], m2[1]);
    m1[2] = _mm_unpacklo_epi32(m2[2], m2[3]);
    m1[3] = _mm_unpackhi_epi32(m2[2], m2[3]);

    m2[0] = _mm_unpacklo_epi64(m1[0], m1[2]);
    m2[1] = _mm_unpackhi_epi64(m1[0], m1[2]);
    m2[2] = _mm_unpacklo_epi64(m1[1], m1[3]);
    m2[3] = _mm_unpackhi_epi64(m1[1], m1[3]);
  }

#if JVET_R0164_MEAN_SCALED_SATD
  uint32_t absDc = 0;
#endif

  if (iBitDepth >= 10 /*sizeof( Torg ) > 1 || sizeof( Tcur ) > 1*/)
  {
    __m128i n1[4][2];
    __m128i n2[4][2];

    for (int i = 0; i < 4; i++)
    {
      n1[i][0] = _mm_cvtepi16_epi32(m2[i]);
      n1[i][1] = _mm_cvtepi16_epi32(_mm_shuffle_epi32(m2[i], 0xEE));
    }

    for (int i = 0; i < 2; i++)
    {
      n2[0][i] = _mm_add_epi32(n1[0][i], n1[2][i]);
      n2[1][i] = _mm_add_epi32(n1[1][i], n1[3][i]);
      n2[2][i] = _mm_sub_epi32(n1[0][i], n1[2][i]);
      n2[3][i] = _mm_sub_epi32(n1[1][i], n1[3][i]);

      n1[0][i] = _mm_abs_epi32(_mm_add_epi32(n2[0][i], n2[1][i]));
      n1[1][i] = _mm_abs_epi32(_mm_sub_epi32(n2[0][i], n2[1][i]));
      n1[2][i] = _mm_abs_epi32(_mm_add_epi32(n2[2][i], n2[3][i]));
      n1[3][i] = _mm_abs_epi32(_mm_sub_epi32(n2[2][i], n2[3][i]));
    }
    for (int i = 0; i < 4; i++)
    {
      m1[i] = _mm_add_epi32(n1[i][0], n1[i][1]);
    }

#if JVET_R0164_MEAN_SCALED_SATD
    absDc = _mm_cvtsi128_si32(n1[0][0]);
#endif
  }
  else
  {
    m1[0] = _mm_add_epi16(m2[0], m2[2]);
    m1[1] = _mm_add_epi16(m2[1], m2[3]);
    m1[2] = _mm_sub_epi16(m2[0], m2[2]);
    m1[3] = _mm_sub_epi16(m2[1], m2[3]);

    m2[0] = _mm_abs_epi16(_mm_add_epi16(m1[0], m1[1]));
    m2[1] = _mm_abs_epi16(_mm_sub_epi16(m1[0], m1[1]));
    m2[2] = _mm_abs_epi16(_mm_add_epi16(m1[2], m1[3]));
    m2[3] = _mm_abs_epi16(_mm_sub_epi16(m1[2], m1[3]));

    __m128i ma1, ma2;
    __m128i vzero = _mm_setzero_si128();

    for (int i = 0; i < 4; i++)
    {
      ma1   = _mm_unpacklo_epi16(m2[i], vzero);
      ma2   = _mm_unpackhi_epi16(m2[i], vzero);
      m1[i] = _mm_add_epi32(ma1, ma2);
    }

#if JVET_R0164_MEAN_SCALED_SATD
    absDc = _mm_cvtsi128_si32(m2[0]) & 0x0000ffff;
#endif
  }

  m1[0] = _mm_add_epi32(m1[0], m1[1]);
  m1[2] = _mm_add_epi32(m1[2], m1[3]);

  __m128i sum = _mm_add_epi32(m1[0], m1[2]);

  sum = _mm_hadd_epi32(sum, sum);
  sum = _mm_hadd_epi32(sum, sum);

  uint32_t sad = _mm_cvtsi128_si32(sum);

  // sad = ((sad + 2) >> 2);
#if JVET_R0164_MEAN_SCALED_SATD
  sad -= absDc;
  sad += absDc >> 2;
#endif
  sad = sad * INV_SQRT_2 >> 32;
  sad >>= 1;

  return sad;
}

#if USE_AVX2
namespace
{

typedef __m256i(Stack)[8];

constexpr auto lo = 0;   // the low-vertical-frequency half of the transform(s)
constexpr auto hi = 1;   // the high-vertical-frequency half of the transform(s)

template<int deeperThan10Bit, bool separate8x8, bool attenuateDcCoefficient, int row>
inline void processRow(Stack &e, __m256i (&verticalSum)[2], Stack (&data)[2], __m256i add, __m256i sub, __m256i shift)
{
  // vertical butterfly stride 1
  const __m256i p[2] = {
    _mm256_madd_epi16(e[row], add),
    _mm256_madd_epi16(e[row], sub),
  };

  if constexpr (!deeperThan10Bit && separate8x8)
  {
    __m256i absdiff[] = {
      _mm256_abs_epi32(p[0]),
      _mm256_abs_epi32(p[1]),
    };
    if constexpr (attenuateDcCoefficient && row == 0)
    {
      absdiff[0] = _mm256_srav_epi32(absdiff[0], shift);
    }
    verticalSum[0] = _mm256_add_epi32(verticalSum[0], absdiff[0]);
    verticalSum[1] = _mm256_add_epi32(verticalSum[1], absdiff[1]);
  }
  else
  {
    data[lo][row] = p[0];
    data[hi][row] = p[1];
  }
};

template<int maxBitDepth, bool separate8x8, bool attenuateDcCoefficient>
static uint32_t satd16x8(const int16_t *piOrg, const int16_t *piCur, ptrdiff_t strideOrg, ptrdiff_t strideCur)
{
  [[maybe_unused]] int32_t maxValue;

  // Note that we don't use the fastest code path requires inputs are guaranteed to be in the 10 bit depth range
  // 0..1023. This is because prediction sometimes passes us inputs that fall out of the expected 10-bit range A
  // pragmatic fix would be to use saturated add/sub for the final 16-bit stage but we must match scalar SATD behaviour
  // for all possible inputs so we use the slower code that supports 14 bits depth.
  constexpr auto deeperThan10Bit = maxBitDepth > 10;

  Stack data[2];

  [[maybe_unused]] constexpr auto haddsub_epi16 = [](__m256i &x, __m256i &y, int n = 1)
  {
    for (int i = 0; i < n; ++i)
    {
      const auto lo = _mm256_unpacklo_epi32(x, y);
      const auto hi = _mm256_unpackhi_epi32(x, y);
      x             = _mm256_add_epi16(lo, hi);
      y             = _mm256_sub_epi16(lo, hi);
    }
  };

  [[maybe_unused]] constexpr auto haddsub_epi32 = [](__m256i &x, __m256i &y, int n = 1)
  {
    for (int i = 0; i < n; ++i)
    {
      const auto lo = _mm256_unpacklo_epi32(x, y);
      const auto hi = _mm256_unpackhi_epi32(x, y);
      x             = _mm256_add_epi32(lo, hi);
      y             = _mm256_sub_epi32(lo, hi);
    }
  };

  __m256i verticalSum[2];
  if constexpr (!deeperThan10Bit && separate8x8)
  {
    verticalSum[0] = verticalSum[1] = _mm256_setzero_si256();
  }

  {
    [[maybe_unused]] Stack k;
    for (int row2 = 0; row2 < 8; row2 += 2)
    {
      __m256i input[2];
      for (int i = 0; i < 2; ++i)
      {
        const auto row = row2 + i;

        maxValue = (1 << maxBitDepth) - 1;

        if constexpr (separate8x8)
        {
          input[i] =
            _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *)piOrg), _mm256_loadu_si256((const __m256i *)piCur));
        }
        else
        {
          const __m128i diff[2] = {
            _mm_sub_epi16(_mm_loadu_si128((const __m128i *)&piOrg[0]), _mm_loadu_si128((const __m128i *)&piCur[0])),
            _mm_sub_epi16(_mm_loadu_si128((const __m128i *)&piOrg[8]), _mm_loadu_si128((const __m128i *)&piCur[8])),
          };

          auto sum   = _mm_add_epi16(diff[0], diff[1]);
          auto diff2 = _mm_sub_epi16(diff[0], diff[1]);
          input[i]   = _mm256_inserti128_si256(_mm256_castsi128_si256(sum), diff2, 1);
          maxValue <<= 1;
        }
        piCur += strideCur;
        piOrg += strideOrg;

        if constexpr (deeperThan10Bit)
        {
          // could potentially do more butterflies here with faster 16-bit values before going to 32-bit,
          // it depends on the maximum bitdepth we need to support

          CHECKD(maxValue > std::numeric_limits<int16_t>::max(), "we overflowed our int16_t");

          {
            // horizontal butterfly stride 1
            const auto add = _mm256_set_epi16(+1, +1, +1, +1, +1, +1, +1, +1, +1, +1, +1, +1, +1, +1, +1, +1);
            const auto sub = _mm256_set_epi16(-1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1, +1);
            data[lo][row]  = _mm256_madd_epi16(input[i], add);
            data[hi][row]  = _mm256_madd_epi16(input[i], sub);
          }
        }
      }

      if constexpr (!deeperThan10Bit)
      {
        auto a0 = input[0];
        auto a1 = input[1];

        // horizontal butterfly strides 1, 2 and 4
        haddsub_epi16(a0, a1, 3);
        maxValue <<= 3;

        k[row2 + 0] = a0;
        k[row2 + 1] = a1;
      }
    }

    if constexpr (!deeperThan10Bit)
    {
      Stack e;
      {
        // vertical butterfly stride 4
        e[0] = _mm256_add_epi16(k[0], k[4]);
        e[1] = _mm256_add_epi16(k[1], k[5]);
        e[2] = _mm256_add_epi16(k[2], k[6]);
        e[3] = _mm256_add_epi16(k[3], k[7]);
        e[4] = _mm256_sub_epi16(k[0], k[4]);
        e[5] = _mm256_sub_epi16(k[1], k[5]);
        e[6] = _mm256_sub_epi16(k[2], k[6]);
        e[7] = _mm256_sub_epi16(k[3], k[7]);
        maxValue <<= 1;
      }

      if constexpr (separate8x8)
      {
        // vertical butterfly stride 2
        Stack f;
        f[0] = _mm256_add_epi16(e[0], e[2]);
        f[1] = _mm256_add_epi16(e[1], e[3]);
        f[2] = _mm256_sub_epi16(e[0], e[2]);
        f[3] = _mm256_sub_epi16(e[1], e[3]);
        e[0] = f[0];
        e[1] = f[1];
        e[2] = f[2];
        e[3] = f[3];
        f[4] = _mm256_add_epi16(e[4], e[6]);
        f[5] = _mm256_add_epi16(e[5], e[7]);
        f[6] = _mm256_sub_epi16(e[4], e[6]);
        f[7] = _mm256_sub_epi16(e[5], e[7]);
        e[4] = f[4];
        e[5] = f[5];
        e[6] = f[6];
        e[7] = f[7];
        maxValue <<= 1;
      }

      CHECKD(maxValue > std::numeric_limits<int16_t>::max(), "we could overflow our int16_t");

      const auto add   = _mm256_set1_epi32(0x00010001);
      const auto sub   = _mm256_set1_epi32(0xffff0001);
      const auto shift = _mm256_setr_epi32(2, 0, 0, 0, 2, 0, 0, 0);
      processRow<deeperThan10Bit, separate8x8, attenuateDcCoefficient, 0>(e, verticalSum, data, add, sub, shift);
      processRow<deeperThan10Bit, separate8x8, attenuateDcCoefficient, 1>(e, verticalSum, data, add, sub, shift);
      processRow<deeperThan10Bit, separate8x8, attenuateDcCoefficient, 2>(e, verticalSum, data, add, sub, shift);
      processRow<deeperThan10Bit, separate8x8, attenuateDcCoefficient, 3>(e, verticalSum, data, add, sub, shift);
      processRow<deeperThan10Bit, separate8x8, attenuateDcCoefficient, 4>(e, verticalSum, data, add, sub, shift);
      processRow<deeperThan10Bit, separate8x8, attenuateDcCoefficient, 5>(e, verticalSum, data, add, sub, shift);
      processRow<deeperThan10Bit, separate8x8, attenuateDcCoefficient, 6>(e, verticalSum, data, add, sub, shift);
      processRow<deeperThan10Bit, separate8x8, attenuateDcCoefficient, 7>(e, verticalSum, data, add, sub, shift);
    }
  }

  if constexpr (deeperThan10Bit || !separate8x8)
  {
    for (int i = lo; i <= hi; ++i)
    {
      auto &g = data[i];

      if constexpr (deeperThan10Bit)
      {
        // vertical butterfly stride 4
        Stack d;
        d[0] = _mm256_add_epi32(g[0], g[4]);
        d[1] = _mm256_add_epi32(g[1], g[5]);
        haddsub_epi32(d[0], d[1], 3);   // horizontal strides 2 and 4; vertical stride 1
        d[2] = _mm256_add_epi32(g[2], g[6]);
        d[3] = _mm256_add_epi32(g[3], g[7]);
        haddsub_epi32(d[2], d[3], 3);   // horizontal strides 2 and 4; vertical stride 1
        d[4] = _mm256_sub_epi32(g[0], g[4]);
        d[5] = _mm256_sub_epi32(g[1], g[5]);
        haddsub_epi32(d[4], d[5], 3);   // horizontal strides 2 and 4; vertical stride 1
        d[6] = _mm256_sub_epi32(g[2], g[6]);
        d[7] = _mm256_sub_epi32(g[3], g[7]);
        haddsub_epi32(d[6], d[7], 3);   // horizontal strides 2 and 4; vertical stride 1

        for (int row = 0; row < 8; ++row)
        {
          g[row] = d[row];
        }
      }

      // vertical butterfly stride 2, and abs
      Stack absdiff;
      absdiff[0] = _mm256_abs_epi32(_mm256_add_epi32(g[0], g[2]));
      absdiff[1] = _mm256_abs_epi32(_mm256_add_epi32(g[1], g[3]));
      absdiff[2] = _mm256_abs_epi32(_mm256_sub_epi32(g[0], g[2]));
      absdiff[3] = _mm256_abs_epi32(_mm256_sub_epi32(g[1], g[3]));
      absdiff[4] = _mm256_abs_epi32(_mm256_add_epi32(g[4], g[6]));
      absdiff[5] = _mm256_abs_epi32(_mm256_add_epi32(g[5], g[7]));
      absdiff[6] = _mm256_abs_epi32(_mm256_sub_epi32(g[4], g[6]));
      absdiff[7] = _mm256_abs_epi32(_mm256_sub_epi32(g[5], g[7]));

      if constexpr (attenuateDcCoefficient)
      {
        if (i == lo)
        {
          const auto shift = _mm256_setr_epi32(2, 0, 0, 0, 2 * separate8x8, 0, 0, 0);
          absdiff[0]       = _mm256_srav_epi32(absdiff[0], shift);
        }
      }

      verticalSum[i] = _mm256_add_epi32(
        _mm256_add_epi32(_mm256_add_epi32(absdiff[0], absdiff[1]), _mm256_add_epi32(absdiff[2], absdiff[3])),
        _mm256_add_epi32(_mm256_add_epi32(absdiff[4], absdiff[5]), _mm256_add_epi32(absdiff[6], absdiff[7])));
    }
  }

  // three horizontal sums
  auto sum = _mm256_add_epi32(verticalSum[lo], verticalSum[hi]);

  {
    const auto h   = _mm256_unpackhi_epi64(sum, sum);
    const auto s   = _mm256_add_epi32(h, sum);
    const auto h32 = _mm256_shuffle_epi32(s, _MM_SHUFFLE(2, 3, 0, 1));
    sum            = _mm256_add_epi32(s, h32);
  }

  // final sum of verticalSum from low- and high-vertical frequency stacks
  auto s0 = _mm_cvtsi128_si32(_mm256_extractf128_si256(sum, 0));
  auto s1 = _mm_cvtsi128_si32(_mm256_extractf128_si256(sum, 1));
  if constexpr (separate8x8)
  {
    s0 = ((s0 + 2) >> 2);
    s1 = ((s1 + 2) >> 2);
  }
  uint32_t satd = s0 + s1;

  if constexpr (!separate8x8)
  {
    satd = satd * INV_SQRT_2 >> 32;
    satd >>= 2;
  }
  return satd;
}
}   // namespace
#endif

static uint32_t xCalcHAD16x16_AVX2(const Torg *piOrg, const Tcur *piCur, const ptrdiff_t strideOrg,
                                   const ptrdiff_t strideCur, int inputDepth)
{
#ifdef USE_AVX2
  constexpr auto attenuateDcCoefficient = JVET_R0164_MEAN_SCALED_SATD;
  const auto     f =
    inputDepth > 10 ? satd16x8<14, true, attenuateDcCoefficient> : satd16x8<10, true, attenuateDcCoefficient>;
  return f(piOrg, piCur, strideOrg, strideCur) + f(piOrg + 8 * strideOrg, piCur + 8 * strideCur, strideOrg, strideCur);
#else
  return 0;
#endif
}

static uint32_t xCalcHAD16x8_AVX2(const Torg *piOrg, const Tcur *piCur, const ptrdiff_t strideOrg,
                                  const ptrdiff_t strideCur, int inputDepth)
{
#ifdef USE_AVX2
  constexpr auto attenuateDcCoefficient = JVET_R0164_MEAN_SCALED_SATD;
  const auto     f =
    inputDepth > 10 ? satd16x8<14, false, attenuateDcCoefficient> : satd16x8<10, false, attenuateDcCoefficient>;
  return f(piOrg, piCur, strideOrg, strideCur);
#else
  return 0;
#endif
}

static uint32_t xCalcHAD8x16_AVX2(const Pel *piOrg, const Pel *piCur, const ptrdiff_t strideOrg,
                                  const ptrdiff_t strideCur, const int iBitDepth)
{
  uint32_t sad = 0;

#ifdef USE_AVX2
  __m256i m1[16], m2[16];

  {
    {
      for (int k = 0; k < 16; k++)
      {
        __m256i r0 = _mm256_cvtepi16_epi32(_mm_lddqu_si128((__m128i *)piOrg));
        __m256i r1 = _mm256_cvtepi16_epi32(_mm_lddqu_si128((__m128i *)piCur));
        m1[k]      = _mm256_sub_epi32(r0, r1);
        piCur += strideCur;
        piOrg += strideOrg;
      }
    }

    // vertical

    m2[0]  = _mm256_add_epi32(m1[0], m1[8]);
    m2[1]  = _mm256_add_epi32(m1[1], m1[9]);
    m2[2]  = _mm256_add_epi32(m1[2], m1[10]);
    m2[3]  = _mm256_add_epi32(m1[3], m1[11]);
    m2[4]  = _mm256_add_epi32(m1[4], m1[12]);
    m2[5]  = _mm256_add_epi32(m1[5], m1[13]);
    m2[6]  = _mm256_add_epi32(m1[6], m1[14]);
    m2[7]  = _mm256_add_epi32(m1[7], m1[15]);
    m2[8]  = _mm256_sub_epi32(m1[0], m1[8]);
    m2[9]  = _mm256_sub_epi32(m1[1], m1[9]);
    m2[10] = _mm256_sub_epi32(m1[2], m1[10]);
    m2[11] = _mm256_sub_epi32(m1[3], m1[11]);
    m2[12] = _mm256_sub_epi32(m1[4], m1[12]);
    m2[13] = _mm256_sub_epi32(m1[5], m1[13]);
    m2[14] = _mm256_sub_epi32(m1[6], m1[14]);
    m2[15] = _mm256_sub_epi32(m1[7], m1[15]);

    m1[0]  = _mm256_add_epi32(m2[0], m2[4]);
    m1[1]  = _mm256_add_epi32(m2[1], m2[5]);
    m1[2]  = _mm256_add_epi32(m2[2], m2[6]);
    m1[3]  = _mm256_add_epi32(m2[3], m2[7]);
    m1[4]  = _mm256_sub_epi32(m2[0], m2[4]);
    m1[5]  = _mm256_sub_epi32(m2[1], m2[5]);
    m1[6]  = _mm256_sub_epi32(m2[2], m2[6]);
    m1[7]  = _mm256_sub_epi32(m2[3], m2[7]);
    m1[8]  = _mm256_add_epi32(m2[8], m2[12]);
    m1[9]  = _mm256_add_epi32(m2[9], m2[13]);
    m1[10] = _mm256_add_epi32(m2[10], m2[14]);
    m1[11] = _mm256_add_epi32(m2[11], m2[15]);
    m1[12] = _mm256_sub_epi32(m2[8], m2[12]);
    m1[13] = _mm256_sub_epi32(m2[9], m2[13]);
    m1[14] = _mm256_sub_epi32(m2[10], m2[14]);
    m1[15] = _mm256_sub_epi32(m2[11], m2[15]);

    m2[0]  = _mm256_add_epi32(m1[0], m1[2]);
    m2[1]  = _mm256_add_epi32(m1[1], m1[3]);
    m2[2]  = _mm256_sub_epi32(m1[0], m1[2]);
    m2[3]  = _mm256_sub_epi32(m1[1], m1[3]);
    m2[4]  = _mm256_add_epi32(m1[4], m1[6]);
    m2[5]  = _mm256_add_epi32(m1[5], m1[7]);
    m2[6]  = _mm256_sub_epi32(m1[4], m1[6]);
    m2[7]  = _mm256_sub_epi32(m1[5], m1[7]);
    m2[8]  = _mm256_add_epi32(m1[8], m1[10]);
    m2[9]  = _mm256_add_epi32(m1[9], m1[11]);
    m2[10] = _mm256_sub_epi32(m1[8], m1[10]);
    m2[11] = _mm256_sub_epi32(m1[9], m1[11]);
    m2[12] = _mm256_add_epi32(m1[12], m1[14]);
    m2[13] = _mm256_add_epi32(m1[13], m1[15]);
    m2[14] = _mm256_sub_epi32(m1[12], m1[14]);
    m2[15] = _mm256_sub_epi32(m1[13], m1[15]);

    m1[0]  = _mm256_add_epi32(m2[0], m2[1]);
    m1[1]  = _mm256_sub_epi32(m2[0], m2[1]);
    m1[2]  = _mm256_add_epi32(m2[2], m2[3]);
    m1[3]  = _mm256_sub_epi32(m2[2], m2[3]);
    m1[4]  = _mm256_add_epi32(m2[4], m2[5]);
    m1[5]  = _mm256_sub_epi32(m2[4], m2[5]);
    m1[6]  = _mm256_add_epi32(m2[6], m2[7]);
    m1[7]  = _mm256_sub_epi32(m2[6], m2[7]);
    m1[8]  = _mm256_add_epi32(m2[8], m2[9]);
    m1[9]  = _mm256_sub_epi32(m2[8], m2[9]);
    m1[10] = _mm256_add_epi32(m2[10], m2[11]);
    m1[11] = _mm256_sub_epi32(m2[10], m2[11]);
    m1[12] = _mm256_add_epi32(m2[12], m2[13]);
    m1[13] = _mm256_sub_epi32(m2[12], m2[13]);
    m1[14] = _mm256_add_epi32(m2[14], m2[15]);
    m1[15] = _mm256_sub_epi32(m2[14], m2[15]);

    // transpose
    constexpr int perm_unpacklo_epi128 = (0 << 0) + (2 << 4);
    constexpr int perm_unpackhi_epi128 = (1 << 0) + (3 << 4);

    // 1. 8x8
    m2[0] = _mm256_unpacklo_epi32(m1[0], m1[1]);
    m2[1] = _mm256_unpacklo_epi32(m1[2], m1[3]);
    m2[2] = _mm256_unpacklo_epi32(m1[4], m1[5]);
    m2[3] = _mm256_unpacklo_epi32(m1[6], m1[7]);
    m2[4] = _mm256_unpackhi_epi32(m1[0], m1[1]);
    m2[5] = _mm256_unpackhi_epi32(m1[2], m1[3]);
    m2[6] = _mm256_unpackhi_epi32(m1[4], m1[5]);
    m2[7] = _mm256_unpackhi_epi32(m1[6], m1[7]);

    m1[0] = _mm256_unpacklo_epi64(m2[0], m2[1]);
    m1[1] = _mm256_unpackhi_epi64(m2[0], m2[1]);
    m1[2] = _mm256_unpacklo_epi64(m2[2], m2[3]);
    m1[3] = _mm256_unpackhi_epi64(m2[2], m2[3]);
    m1[4] = _mm256_unpacklo_epi64(m2[4], m2[5]);
    m1[5] = _mm256_unpackhi_epi64(m2[4], m2[5]);
    m1[6] = _mm256_unpacklo_epi64(m2[6], m2[7]);
    m1[7] = _mm256_unpackhi_epi64(m2[6], m2[7]);

    m2[0] = _mm256_permute2x128_si256(m1[0], m1[2], perm_unpacklo_epi128);
    m2[1] = _mm256_permute2x128_si256(m1[0], m1[2], perm_unpackhi_epi128);
    m2[2] = _mm256_permute2x128_si256(m1[1], m1[3], perm_unpacklo_epi128);
    m2[3] = _mm256_permute2x128_si256(m1[1], m1[3], perm_unpackhi_epi128);
    m2[4] = _mm256_permute2x128_si256(m1[4], m1[6], perm_unpacklo_epi128);
    m2[5] = _mm256_permute2x128_si256(m1[4], m1[6], perm_unpackhi_epi128);
    m2[6] = _mm256_permute2x128_si256(m1[5], m1[7], perm_unpacklo_epi128);
    m2[7] = _mm256_permute2x128_si256(m1[5], m1[7], perm_unpackhi_epi128);

    // 2. 8x8
    m2[0 + 8] = _mm256_unpacklo_epi32(m1[0 + 8], m1[1 + 8]);
    m2[1 + 8] = _mm256_unpacklo_epi32(m1[2 + 8], m1[3 + 8]);
    m2[2 + 8] = _mm256_unpacklo_epi32(m1[4 + 8], m1[5 + 8]);
    m2[3 + 8] = _mm256_unpacklo_epi32(m1[6 + 8], m1[7 + 8]);
    m2[4 + 8] = _mm256_unpackhi_epi32(m1[0 + 8], m1[1 + 8]);
    m2[5 + 8] = _mm256_unpackhi_epi32(m1[2 + 8], m1[3 + 8]);
    m2[6 + 8] = _mm256_unpackhi_epi32(m1[4 + 8], m1[5 + 8]);
    m2[7 + 8] = _mm256_unpackhi_epi32(m1[6 + 8], m1[7 + 8]);

    m1[0 + 8] = _mm256_unpacklo_epi64(m2[0 + 8], m2[1 + 8]);
    m1[1 + 8] = _mm256_unpackhi_epi64(m2[0 + 8], m2[1 + 8]);
    m1[2 + 8] = _mm256_unpacklo_epi64(m2[2 + 8], m2[3 + 8]);
    m1[3 + 8] = _mm256_unpackhi_epi64(m2[2 + 8], m2[3 + 8]);
    m1[4 + 8] = _mm256_unpacklo_epi64(m2[4 + 8], m2[5 + 8]);
    m1[5 + 8] = _mm256_unpackhi_epi64(m2[4 + 8], m2[5 + 8]);
    m1[6 + 8] = _mm256_unpacklo_epi64(m2[6 + 8], m2[7 + 8]);
    m1[7 + 8] = _mm256_unpackhi_epi64(m2[6 + 8], m2[7 + 8]);

    m2[0 + 8] = _mm256_permute2x128_si256(m1[0 + 8], m1[2 + 8], perm_unpacklo_epi128);
    m2[1 + 8] = _mm256_permute2x128_si256(m1[0 + 8], m1[2 + 8], perm_unpackhi_epi128);
    m2[2 + 8] = _mm256_permute2x128_si256(m1[1 + 8], m1[3 + 8], perm_unpacklo_epi128);
    m2[3 + 8] = _mm256_permute2x128_si256(m1[1 + 8], m1[3 + 8], perm_unpackhi_epi128);
    m2[4 + 8] = _mm256_permute2x128_si256(m1[4 + 8], m1[6 + 8], perm_unpacklo_epi128);
    m2[5 + 8] = _mm256_permute2x128_si256(m1[4 + 8], m1[6 + 8], perm_unpackhi_epi128);
    m2[6 + 8] = _mm256_permute2x128_si256(m1[5 + 8], m1[7 + 8], perm_unpacklo_epi128);
    m2[7 + 8] = _mm256_permute2x128_si256(m1[5 + 8], m1[7 + 8], perm_unpackhi_epi128);

    // horizontal
    m1[0] = _mm256_add_epi32(m2[0], m2[4]);
    m1[1] = _mm256_add_epi32(m2[1], m2[5]);
    m1[2] = _mm256_add_epi32(m2[2], m2[6]);
    m1[3] = _mm256_add_epi32(m2[3], m2[7]);
    m1[4] = _mm256_sub_epi32(m2[0], m2[4]);
    m1[5] = _mm256_sub_epi32(m2[1], m2[5]);
    m1[6] = _mm256_sub_epi32(m2[2], m2[6]);
    m1[7] = _mm256_sub_epi32(m2[3], m2[7]);

    m2[0] = _mm256_add_epi32(m1[0], m1[2]);
    m2[1] = _mm256_add_epi32(m1[1], m1[3]);
    m2[2] = _mm256_sub_epi32(m1[0], m1[2]);
    m2[3] = _mm256_sub_epi32(m1[1], m1[3]);
    m2[4] = _mm256_add_epi32(m1[4], m1[6]);
    m2[5] = _mm256_add_epi32(m1[5], m1[7]);
    m2[6] = _mm256_sub_epi32(m1[4], m1[6]);
    m2[7] = _mm256_sub_epi32(m1[5], m1[7]);

    m1[0] = _mm256_abs_epi32(_mm256_add_epi32(m2[0], m2[1]));
    m1[1] = _mm256_abs_epi32(_mm256_sub_epi32(m2[0], m2[1]));
    m1[2] = _mm256_abs_epi32(_mm256_add_epi32(m2[2], m2[3]));
    m1[3] = _mm256_abs_epi32(_mm256_sub_epi32(m2[2], m2[3]));
    m1[4] = _mm256_abs_epi32(_mm256_add_epi32(m2[4], m2[5]));
    m1[5] = _mm256_abs_epi32(_mm256_sub_epi32(m2[4], m2[5]));
    m1[6] = _mm256_abs_epi32(_mm256_add_epi32(m2[6], m2[7]));
    m1[7] = _mm256_abs_epi32(_mm256_sub_epi32(m2[6], m2[7]));

#if JVET_R0164_MEAN_SCALED_SATD
    int absDc = _mm_cvtsi128_si32(_mm256_castsi256_si128(m1[0]));
#endif

    m1[0 + 8] = _mm256_add_epi32(m2[0 + 8], m2[4 + 8]);
    m1[1 + 8] = _mm256_add_epi32(m2[1 + 8], m2[5 + 8]);
    m1[2 + 8] = _mm256_add_epi32(m2[2 + 8], m2[6 + 8]);
    m1[3 + 8] = _mm256_add_epi32(m2[3 + 8], m2[7 + 8]);
    m1[4 + 8] = _mm256_sub_epi32(m2[0 + 8], m2[4 + 8]);
    m1[5 + 8] = _mm256_sub_epi32(m2[1 + 8], m2[5 + 8]);
    m1[6 + 8] = _mm256_sub_epi32(m2[2 + 8], m2[6 + 8]);
    m1[7 + 8] = _mm256_sub_epi32(m2[3 + 8], m2[7 + 8]);

    m2[0 + 8] = _mm256_add_epi32(m1[0 + 8], m1[2 + 8]);
    m2[1 + 8] = _mm256_add_epi32(m1[1 + 8], m1[3 + 8]);
    m2[2 + 8] = _mm256_sub_epi32(m1[0 + 8], m1[2 + 8]);
    m2[3 + 8] = _mm256_sub_epi32(m1[1 + 8], m1[3 + 8]);
    m2[4 + 8] = _mm256_add_epi32(m1[4 + 8], m1[6 + 8]);
    m2[5 + 8] = _mm256_add_epi32(m1[5 + 8], m1[7 + 8]);
    m2[6 + 8] = _mm256_sub_epi32(m1[4 + 8], m1[6 + 8]);
    m2[7 + 8] = _mm256_sub_epi32(m1[5 + 8], m1[7 + 8]);

    m1[0 + 8] = _mm256_abs_epi32(_mm256_add_epi32(m2[0 + 8], m2[1 + 8]));
    m1[1 + 8] = _mm256_abs_epi32(_mm256_sub_epi32(m2[0 + 8], m2[1 + 8]));
    m1[2 + 8] = _mm256_abs_epi32(_mm256_add_epi32(m2[2 + 8], m2[3 + 8]));
    m1[3 + 8] = _mm256_abs_epi32(_mm256_sub_epi32(m2[2 + 8], m2[3 + 8]));
    m1[4 + 8] = _mm256_abs_epi32(_mm256_add_epi32(m2[4 + 8], m2[5 + 8]));
    m1[5 + 8] = _mm256_abs_epi32(_mm256_sub_epi32(m2[4 + 8], m2[5 + 8]));
    m1[6 + 8] = _mm256_abs_epi32(_mm256_add_epi32(m2[6 + 8], m2[7 + 8]));
    m1[7 + 8] = _mm256_abs_epi32(_mm256_sub_epi32(m2[6 + 8], m2[7 + 8]));

    // sum up
    m1[0] = _mm256_add_epi32(m1[0], m1[1]);
    m1[1] = _mm256_add_epi32(m1[2], m1[3]);
    m1[2] = _mm256_add_epi32(m1[4], m1[5]);
    m1[3] = _mm256_add_epi32(m1[6], m1[7]);
    m1[4] = _mm256_add_epi32(m1[8], m1[9]);
    m1[5] = _mm256_add_epi32(m1[10], m1[11]);
    m1[6] = _mm256_add_epi32(m1[12], m1[13]);
    m1[7] = _mm256_add_epi32(m1[14], m1[15]);

    // sum up
    m1[0] = _mm256_add_epi32(m1[0], m1[1]);
    m1[1] = _mm256_add_epi32(m1[2], m1[3]);
    m1[2] = _mm256_add_epi32(m1[4], m1[5]);
    m1[3] = _mm256_add_epi32(m1[6], m1[7]);

    m1[0] = _mm256_add_epi32(m1[0], m1[1]);
    m1[1] = _mm256_add_epi32(m1[2], m1[3]);

    __m256i sum = _mm256_add_epi32(m1[0], m1[1]);

    sum = _mm256_hadd_epi32(sum, sum);
    sum = _mm256_hadd_epi32(sum, sum);
    sum = _mm256_add_epi32(sum, _mm256_permute2x128_si256(sum, sum, 0x11));

    uint32_t sad2 = _mm_cvtsi128_si32(_mm256_castsi256_si128(sum));

#if JVET_R0164_MEAN_SCALED_SATD
    sad2 -= absDc;
    sad2 += absDc >> 2;
#endif
    sad = sad2 * INV_SQRT_2 >> 32;
    sad >>= 2;
  }

#endif   // USE_AVX2

  return (sad);
}
template<X86_VEXT vext> Distortion RdCost::xGetSADwMask_SIMD(const DistParam &rcDtParam)
{
  if (rcDtParam.org.width < 4 || rcDtParam.bitDepth > 10 || rcDtParam.applyWeight)
  {
    return RdCost::xGetSADwMask(rcDtParam);
  }

  const short    *src1       = (const short *)rcDtParam.org.buf;
  const short    *src2       = (const short *)rcDtParam.cur.buf;
  const short    *weightMask = (const short *)rcDtParam.mask;
  int             rows       = rcDtParam.org.height;
  int             cols       = rcDtParam.org.width;
  int             subShift   = rcDtParam.subShift;
  int             subStep    = (1 << subShift);
  const ptrdiff_t strideSrc1 = rcDtParam.org.stride * subStep;
  const ptrdiff_t strideSrc2 = rcDtParam.cur.stride * subStep;
  const ptrdiff_t strideMask = rcDtParam.maskStride * subStep;

  Distortion sum = 0;
  if (vext >= AVX2 && (cols & 15) == 0)
  {
#ifdef USE_AVX2
    // Do for width that multiple of 16
    __m256i vzero  = _mm256_setzero_si256();
    __m256i vsum32 = vzero;
    for (int y = 0; y < rows; y += subStep)
    {
      for (int x = 0; x < cols; x += 16)
      {
        __m256i vsrc1 = _mm256_lddqu_si256((__m256i *)(&src1[x]));
        __m256i vsrc2 = _mm256_lddqu_si256((__m256i *)(&src2[x]));
        __m256i vmask;
        if (rcDtParam.stepX == -1)
        {
          vmask                      = _mm256_lddqu_si256((__m256i *)((&weightMask[x]) - (x << 1) - (16 - 1)));
          const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3, 2,
                                                       5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
          vmask                      = _mm256_shuffle_epi8(vmask, shuffle_mask);
          vmask                      = _mm256_permute4x64_epi64(vmask, _MM_SHUFFLE(1, 0, 3, 2));
        }
        else
        {
          vmask = _mm256_lddqu_si256((__m256i *)(&weightMask[x]));
        }
        vsum32 = _mm256_add_epi32(vsum32, _mm256_madd_epi16(vmask, _mm256_abs_epi16(_mm256_sub_epi16(vsrc1, vsrc2))));
      }
      src1 += strideSrc1;
      src2 += strideSrc2;
      weightMask += strideMask;
    }
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    sum    = _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum32)) +
      _mm_cvtsi128_si32(_mm256_castsi256_si128(_mm256_permute2x128_si256(vsum32, vsum32, 0x11)));
#endif
  }
  else
  {
    // Do with step of 8
    __m128i vzero  = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    for (int y = 0; y < rows; y += subStep)
    {
      for (int x = 0; x < cols; x += 8)
      {
        __m128i vsrc1 = _mm_loadu_si128((const __m128i *)(&src1[x]));
        __m128i vsrc2 = _mm_lddqu_si128((const __m128i *)(&src2[x]));
        __m128i vmask;
        if (rcDtParam.stepX == -1)
        {
          vmask                      = _mm_lddqu_si128((__m128i *)((&weightMask[x]) - (x << 1) - (8 - 1)));
          const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
          vmask                      = _mm_shuffle_epi8(vmask, shuffle_mask);
        }
        else
        {
          vmask = _mm_lddqu_si128((const __m128i *)(&weightMask[x]));
        }
        vsum32 = _mm_add_epi32(vsum32, _mm_madd_epi16(vmask, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2))));
      }
      src1 += strideSrc1;
      src2 += strideSrc2;
      weightMask += strideMask;
    }
    vsum32 = _mm_hadd_epi32(vsum32, vzero);
    vsum32 = _mm_hadd_epi32(vsum32, vzero);
    sum    = _mm_cvtsi128_si32(vsum32);
  }
  sum <<= subShift;

  return sum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);
}

template<X86_VEXT vext> Distortion RdCost::xGetHADs_SIMD(const DistParam &rcDtParam)
{
  const Pel      *piOrg     = rcDtParam.org.buf;
  const Pel      *piCur     = rcDtParam.cur.buf;
  const int       rows      = rcDtParam.org.height;
  const int       cols      = rcDtParam.org.width;
  const ptrdiff_t strideCur = rcDtParam.cur.stride;
  const ptrdiff_t strideOrg = rcDtParam.org.stride;
  const int       iBitDepth = rcDtParam.bitDepth;

  const auto     inputDepth      = rcDtParam.inputDepth ? rcDtParam.inputDepth : rcDtParam.bitDepth;
  constexpr bool checkInputDepth = false;
  if constexpr (checkInputDepth)
  {
    int maxAbsDiff {};
    for (int y = 0; y < rows; ++y)
    {
      for (int x = 0; x < cols; ++x)
      {
        const int org     = piOrg[x + y * strideOrg];
        const int cur     = piCur[x + y * strideCur];
        const int absdiff = std::abs(org - cur);
        maxAbsDiff        = std::max(maxAbsDiff, absdiff);
      }
    }
    CHECK(maxAbsDiff >= (1 << inputDepth), "too big");
  }

  if (rcDtParam.bitDepth > 10 || rcDtParam.applyWeight)
  {
    return RdCost::xGetHADs<false>(rcDtParam);
  }

  int        x, y;
  Distortion sum = 0;

  if (cols > rows && (cols & 15) == 0 && (rows & 7) == 0)
  {
    for (y = 0; y < rows; y += 8)
    {
      for (x = 0; x < cols; x += 16)
      {
        if (vext >= AVX2)
        {
          sum += xCalcHAD16x8_AVX2(&piOrg[x], &piCur[x], strideOrg, strideCur, inputDepth);
        }
        else
        {
          sum += xCalcHAD16x8_SSE(&piOrg[x], &piCur[x], strideOrg, strideCur, iBitDepth);
        }
      }
      piOrg += strideOrg * 8;
      piCur += strideCur * 8;
    }
  }
  else if (cols < rows && (rows & 15) == 0 && (cols & 7) == 0)
  {
    for (y = 0; y < rows; y += 16)
    {
      for (x = 0; x < cols; x += 8)
      {
        if (vext >= AVX2)
        {
          sum += xCalcHAD8x16_AVX2(&piOrg[x], &piCur[x], strideOrg, strideCur, iBitDepth);
        }
        else
        {
          sum += xCalcHAD8x16_SSE(&piOrg[x], &piCur[x], strideOrg, strideCur, iBitDepth);
        }
      }
      piOrg += strideOrg * 16;
      piCur += strideCur * 16;
    }
  }
  else if (cols > rows && (cols & 7) == 0 && (rows & 3) == 0)
  {
    for (y = 0; y < rows; y += 4)
    {
      for (x = 0; x < cols; x += 8)
      {
        sum += xCalcHAD8x4_SSE<false>(&piOrg[x], &piCur[x], strideOrg, strideCur, iBitDepth);
      }
      piOrg += strideOrg * 4;
      piCur += strideCur * 4;
    }
  }
  else if (cols < rows && (rows & 7) == 0 && (cols & 3) == 0)
  {
    for (y = 0; y < rows; y += 8)
    {
      for (x = 0; x < cols; x += 4)
      {
        sum += xCalcHAD4x8_SSE<false>(&piOrg[x], &piCur[x], strideOrg, strideCur, iBitDepth);
      }
      piOrg += strideOrg * 8;
      piCur += strideCur * 8;
    }
  }
  else if (vext >= AVX2 && (((rows | cols) & 15) == 0) && (rows == cols))
  {
    ptrdiff_t offsetOrg = strideOrg << 4;
    ptrdiff_t offsetCur = strideCur << 4;
    for (y = 0; y < rows; y += 16)
    {
      for (x = 0; x < cols; x += 16)
      {
        sum += xCalcHAD16x16_AVX2(&piOrg[x], &piCur[x], strideOrg, strideCur, inputDepth);
      }
      piOrg += offsetOrg;
      piCur += offsetCur;
    }
  }
  else if ((((rows | cols) & 7) == 0) && (rows == cols))
  {
    ptrdiff_t offsetOrg = strideOrg << 3;
    ptrdiff_t offsetCur = strideCur << 3;
    for (y = 0; y < rows; y += 8)
    {
      for (x = 0; x < cols; x += 8)
      {
        sum += xCalcHAD8x8_SSE<false, false>(&piOrg[x], &piCur[x], strideOrg, strideCur, iBitDepth, inputDepth);
      }
      piOrg += offsetOrg;
      piCur += offsetCur;
    }
  }
  else if ((rows % 4 == 0) && (cols % 4 == 0))
  {
    ptrdiff_t offsetOrg = strideOrg << 2;
    ptrdiff_t offsetCur = strideCur << 2;

    for (y = 0; y < rows; y += 4)
    {
      for (x = 0; x < cols; x += 4)
      {
        sum += xCalcHAD4x4_SSE(&piOrg[x], &piCur[x], strideOrg, strideCur);
      }
      piOrg += offsetOrg;
      piCur += offsetCur;
    }
  }
  else if ((rows % 2 == 0) && (cols % 2 == 0))
  {
    ptrdiff_t offsetOrg = strideOrg << 1;
    ptrdiff_t offsetCur = strideCur << 1;
    for (y = 0; y < rows; y += 2)
    {
      for (x = 0; x < cols; x += 2)
      {
        sum += xCalcHADs2x2(&piOrg[x], &piCur[x * rcDtParam.step], strideOrg, strideCur, rcDtParam.step);
      }
      piOrg += offsetOrg;
      piCur += offsetCur;
    }
  }
  else
  {
    THROW("Unsupported size");
  }

  return sum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);
}

template<X86_VEXT vext> Distortion RdCost::xGetHADsFst_SIMD(const DistParam &rcDtParam)
{
  const Pel      *piOrg     = rcDtParam.org.buf;
  const Pel      *piCur     = rcDtParam.cur.buf;
  const int       rows      = rcDtParam.org.height;
  const int       cols      = rcDtParam.org.width;
  const ptrdiff_t strideCur = rcDtParam.cur.stride;
  const ptrdiff_t strideOrg = rcDtParam.org.stride;
  const int       iBitDepth = rcDtParam.bitDepth;

  const auto     inputDepth      = rcDtParam.inputDepth ? rcDtParam.inputDepth : rcDtParam.bitDepth;
  constexpr bool checkInputDepth = false;
  if constexpr (checkInputDepth)
  {
    int maxAbsDiff {};
    for (int y = 0; y < rows; ++y)
    {
      for (int x = 0; x < cols; ++x)
      {
        const int org     = piOrg[x + y * strideOrg];
        const int cur     = piCur[x + y * strideCur];
        const int absdiff = std::abs(org - cur);
        maxAbsDiff        = std::max(maxAbsDiff, absdiff);
      }
    }
    CHECK(maxAbsDiff >= (1 << inputDepth), "too big");
  }

  if (rcDtParam.bitDepth > 10 || rcDtParam.applyWeight)
  {
    return RdCost::xGetHADs<false>(rcDtParam);
  }

  int        x, y;
  Distortion sum = 0;

  if (rows >= 8 && cols >= 8)   // N(>=8)xM(>=8)
  {
#define FAST_HAD_LOOP(h, v)                                                                            \
  for (y = 0; y < rows; y += 16) /* if 8 or 4, only one iteration anyway */                            \
  {                                                                                                    \
    for (x = 0; x < cols; x += 16) /* if 8 or 4, only one iteration anyway */                          \
    {                                                                                                  \
      sum += xCalcHAD8x8_SSE<h, v>(&piOrg[x], &piCur[x], strideOrg, strideCur, iBitDepth, inputDepth); \
    }                                                                                                  \
    piOrg += offsetOrg;                                                                                \
    piCur += offsetCur;                                                                                \
  }

    int vscale = rows == 8 ? 0 : 1;
    int hscale = cols == 8 ? 0 : 1;

    ptrdiff_t offsetOrg = strideOrg << 4;
    ptrdiff_t offsetCur = strideCur << 4;

    if (vscale == 0)
    {
      if (hscale == 0)
      {
        FAST_HAD_LOOP(false, false);
      }
      else
      {
        FAST_HAD_LOOP(true, false);
      }
    }
    else
    {
      if (hscale == 0)
      {
        FAST_HAD_LOOP(false, true);
      }
      else
      {
        FAST_HAD_LOOP(true, true);
      }
    }

    sum <<= (hscale + vscale);
  }
  else if (cols >= rows && (cols & 7) == 0 && rows == 4)
  {
    CHECK(rows != 4, "If there are more than 4 rows, 8x8 variant should be used");

    int hscale = cols == 8 ? 0 : 1;

    // for (y = 0; y < rows; y += 4)
    {
      for (x = 0; x < cols; x += 16)
      {
        if (hscale)
        {
          sum += xCalcHAD8x4_SSE<true>(&piOrg[x], &piCur[x], strideOrg, strideCur, iBitDepth);
        }
        else
        {
          sum += xCalcHAD8x4_SSE<false>(&piOrg[x], &piCur[x], strideOrg, strideCur, iBitDepth);
        }
      }
      // piOrg += strideOrg * 4;
      // piCur += strideCur * 4;
    }

    sum <<= (hscale);
  }
  else if (cols <= rows && (rows & 7) == 0 && cols == 4)
  {
    CHECK(cols != 4, "If there are more than 4 cols, 8x8 variant should be used");

    int vscale = rows == 8 ? 0 : 1;

    for (y = 0; y < rows; y += 16)
    {
      // for (x = 0; x < cols; x += 4)
      {
        if (vscale)
        {
          sum += xCalcHAD4x8_SSE<true>(&piOrg[0], &piCur[0], strideOrg, strideCur, iBitDepth);
        }
        else
        {
          sum += xCalcHAD4x8_SSE<false>(&piOrg[0], &piCur[0], strideOrg, strideCur, iBitDepth);
        }
      }
      piOrg += strideOrg << 4;
      piCur += strideCur << 4;
    }

    sum <<= (vscale);
  }
  else if (rows == 4 && cols == 4)
  {
    sum += xCalcHAD4x4_SSE(piOrg, piCur, strideOrg, strideCur);
  }
  else
  {
    THROW("Should not happen!");
  }

#undef FAST_HAD_LOOP

  return sum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);
}

template<X86_VEXT vext> Distortion RdCost::xGetMRSAD_SIMD(const DistParam &rcDtParam)
{
  int width = rcDtParam.org.width;

  if (width < 4 || rcDtParam.bitDepth > 10 || rcDtParam.applyWeight)
  {
    return RdCost::xGetMRSAD(rcDtParam);
  }

  int             height    = rcDtParam.org.height;
  const short    *pOrg      = (const short *)rcDtParam.org.buf;
  const short    *pCur      = (const short *)rcDtParam.cur.buf;
  int             subShift  = rcDtParam.subShift;
  int             subStep   = 1 << subShift;
  const ptrdiff_t strideOrg = rcDtParam.org.stride * subStep;
  const ptrdiff_t strideCur = rcDtParam.cur.stride * subStep;
  int             deltaAvg = 0, rowCnt = 0;
  uint32_t        sum = 0;

  // internal bit-depth must be 12-bit or lower

  if (width & 7)   // multiple of 4
  {
    __m128i vzero  = _mm_setzero_si128();
    __m128i vsum32 = vzero;

    for (; height != 0; height -= subStep)
    {
      __m128i vsum16 = vzero;

      for (int n = 0; n < width; n += 4)
      {
        __m128i org = _mm_loadl_epi64((__m128i *)(pOrg + n));
        __m128i cur = _mm_loadl_epi64((__m128i *)(pCur + n));
        vsum16      = _mm_adds_epi16(vsum16, _mm_sub_epi16(org, cur));
      }

      __m128i vsign = _mm_cmpgt_epi16(vzero, vsum16);
      vsum32        = _mm_add_epi32(vsum32, _mm_unpacklo_epi16(vsum16, vsign));

      pOrg += strideOrg;
      pCur += strideCur;
      rowCnt++;
    }

    vsum32   = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
    vsum32   = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    deltaAvg = _mm_cvtsi128_si32(vsum32) / (width * rowCnt);

    pOrg   = (const short *)rcDtParam.org.buf;
    pCur   = (const short *)rcDtParam.cur.buf;
    height = rcDtParam.org.height;

    __m128i delta = _mm_set1_epi16(deltaAvg);
    vsum32        = vzero;

    for (; height != 0; height -= subStep)
    {
      __m128i vsum16 = vzero;

      for (int n = 0; n < width; n += 4)
      {
        __m128i org = _mm_loadl_epi64((__m128i *)(pOrg + n));
        __m128i cur = _mm_loadl_epi64((__m128i *)(pCur + n));
        __m128i abs = _mm_abs_epi16(_mm_sub_epi16(_mm_sub_epi16(org, cur), delta));
        vsum16      = _mm_adds_epu16(abs, vsum16);
      }

      vsum32 = _mm_add_epi32(vsum32, _mm_unpacklo_epi16(vsum16, vzero));

      pOrg += strideOrg;
      pCur += strideCur;
    }

    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    sum    = _mm_cvtsi128_si32(vsum32);
  }
#ifdef USE_AVX2
  else if (vext >= AVX2 && width >= 16)   // multiple of 16
  {
    __m256i vzero  = _mm256_setzero_si256();
    __m256i vsum32 = vzero;

    for (; height != 0; height -= subStep)
    {
      __m256i vsum16 = vzero;

      for (int n = 0; n < width; n += 16)
      {
        __m256i org = _mm256_lddqu_si256((__m256i *)(pOrg + n));
        __m256i cur = _mm256_lddqu_si256((__m256i *)(pCur + n));
        vsum16      = _mm256_adds_epi16(vsum16, _mm256_sub_epi16(org, cur));
      }

      __m256i vsign    = _mm256_cmpgt_epi16(vzero, vsum16);
      __m256i vsumtemp = _mm256_add_epi32(_mm256_unpacklo_epi16(vsum16, vsign), _mm256_unpackhi_epi16(vsum16, vsign));
      vsum32           = _mm256_add_epi32(vsum32, vsumtemp);

      pOrg += strideOrg;
      pCur += strideCur;
      rowCnt++;
    }

    vsum32   = _mm256_hadd_epi32(vsum32, vzero);
    vsum32   = _mm256_hadd_epi32(vsum32, vzero);
    deltaAvg = _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum32)) +
      _mm_cvtsi128_si32(_mm256_castsi256_si128(_mm256_permute2x128_si256(vsum32, vsum32, 0x11)));
    deltaAvg /= width * rowCnt;

    pOrg   = (const short *)rcDtParam.org.buf;
    pCur   = (const short *)rcDtParam.cur.buf;
    height = rcDtParam.org.height;

    __m256i delta = _mm256_set1_epi16(deltaAvg);
    vsum32        = vzero;

    for (; height != 0; height -= subStep)
    {
      __m256i vsum16 = vzero;

      for (int n = 0; n < width; n += 16)
      {
        __m256i org = _mm256_lddqu_si256((__m256i *)(pOrg + n));
        __m256i cur = _mm256_lddqu_si256((__m256i *)(pCur + n));
        __m256i abs = _mm256_abs_epi16(_mm256_sub_epi16(_mm256_sub_epi16(org, cur), delta));
        vsum16      = _mm256_adds_epi16(abs, vsum16);
      }

      __m256i vsumtemp = _mm256_add_epi32(_mm256_unpacklo_epi16(vsum16, vzero), _mm256_unpackhi_epi16(vsum16, vzero));
      vsum32           = _mm256_add_epi32(vsum32, vsumtemp);

      pOrg += strideOrg;
      pCur += strideCur;
    }

    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    sum    = _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum32)) +
      _mm_cvtsi128_si32(_mm256_castsi256_si128(_mm256_permute2x128_si256(vsum32, vsum32, 0x11)));
  }
#endif
  else   // multiple of 8
  {
    __m128i vzero  = _mm_setzero_si128();
    __m128i vsum32 = vzero;

    for (; height != 0; height -= subStep)
    {
      __m128i vsum16 = vzero;

      for (int n = 0; n < width; n += 8)
      {
        __m128i org = _mm_lddqu_si128((__m128i *)(pOrg + n));
        __m128i cur = _mm_lddqu_si128((__m128i *)(pCur + n));
        vsum16      = _mm_adds_epi16(vsum16, _mm_sub_epi16(org, cur));
      }

      __m128i vsign    = _mm_cmpgt_epi16(vzero, vsum16);
      __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vsign), _mm_unpackhi_epi16(vsum16, vsign));
      vsum32           = _mm_add_epi32(vsum32, vsumtemp);

      pOrg += strideOrg;
      pCur += strideCur;
      rowCnt++;
    }

    vsum32   = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
    vsum32   = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    deltaAvg = _mm_cvtsi128_si32(vsum32) / (width * rowCnt);

    pOrg   = (const short *)rcDtParam.org.buf;
    pCur   = (const short *)rcDtParam.cur.buf;
    height = rcDtParam.org.height;

    __m128i delta = _mm_set1_epi16(deltaAvg);
    vsum32        = vzero;

    for (; height != 0; height -= subStep)
    {
      __m128i vsum16 = vzero;

      for (int n = 0; n < width; n += 8)
      {
        __m128i org = _mm_lddqu_si128((__m128i *)(pOrg + n));
        __m128i cur = _mm_lddqu_si128((__m128i *)(pCur + n));
        __m128i abs = _mm_abs_epi16(_mm_sub_epi16(_mm_sub_epi16(org, cur), delta));
        vsum16      = _mm_adds_epu16(abs, vsum16);
      }

      __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
      vsum32           = _mm_add_epi32(vsum32, vsumtemp);

      pOrg += strideOrg;
      pCur += strideCur;
    }

    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    sum    = _mm_cvtsi128_si32(vsum32);
  }

  sum <<= subShift;
  return sum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);
}

template<X86_VEXT vext> void RdCost::_initRdCostX86()
{
  m_distortionFunc[DFunc::SSE]    = xGetSSE_SIMD<vext>;
  m_distortionFunc[DFunc::SSE2]   = xGetSSE_NxN_SIMD<2, vext>;
  m_distortionFunc[DFunc::SSE4]   = xGetSSE_NxN_SIMD<4, vext>;
  m_distortionFunc[DFunc::SSE8]   = xGetSSE_NxN_SIMD<8, vext>;
  m_distortionFunc[DFunc::SSE16]  = xGetSSE_NxN_SIMD<16, vext>;
  m_distortionFunc[DFunc::SSE32]  = xGetSSE_NxN_SIMD<32, vext>;
  m_distortionFunc[DFunc::SSE64]  = xGetSSE_NxN_SIMD<64, vext>;
  m_distortionFunc[DFunc::SSE16N] = xGetSSE_SIMD<vext>;

  m_distortionFunc[DFunc::SAD]    = xGetSAD_SIMD<vext>;
  m_distortionFunc[DFunc::SAD2]   = xGetSAD_SIMD<vext>;
  m_distortionFunc[DFunc::SAD4]   = xGetSAD_NxN_SIMD<4, vext>;
  m_distortionFunc[DFunc::SAD8]   = xGetSAD_NxN_SIMD<8, vext>;
  m_distortionFunc[DFunc::SAD16]  = xGetSAD_NxN_SIMD<16, vext>;
  m_distortionFunc[DFunc::SAD32]  = xGetSAD_NxN_SIMD<32, vext>;
  m_distortionFunc[DFunc::SAD64]  = xGetSAD_NxN_SIMD<64, vext>;
  m_distortionFunc[DFunc::SAD16N] = xGetSAD_SIMD<vext>;

  m_distortionFunc[DFunc::SAD12] = RdCost::xGetSAD_SIMD<vext>;
  m_distortionFunc[DFunc::SAD24] = RdCost::xGetSAD_SIMD<vext>;
  m_distortionFunc[DFunc::SAD48] = RdCost::xGetSAD_SIMD<vext>;

  m_distortionFunc[DFunc::HAD]    = RdCost::xGetHADs_SIMD<vext>;
  m_distortionFunc[DFunc::HAD2]   = RdCost::xGetHADs_SIMD<vext>;
  m_distortionFunc[DFunc::HAD4]   = RdCost::xGetHADs_SIMD<vext>;
  m_distortionFunc[DFunc::HAD8]   = RdCost::xGetHADs_SIMD<vext>;
  m_distortionFunc[DFunc::HAD16]  = RdCost::xGetHADs_SIMD<vext>;
  m_distortionFunc[DFunc::HAD32]  = RdCost::xGetHADs_SIMD<vext>;
  m_distortionFunc[DFunc::HAD64]  = RdCost::xGetHADs_SIMD<vext>;
  m_distortionFunc[DFunc::HAD16N] = RdCost::xGetHADs_SIMD<vext>;

  m_distortionFunc[DFunc::HAD_fast]    = RdCost::xGetHADsFst_SIMD<vext>;
  // m_distortionFunc[DFunc::HAD2_fast]   = RdCost::xGetHADsFst_SIMD<vext>;
  m_distortionFunc[DFunc::HAD4_fast]   = RdCost::xGetHADsFst_SIMD<vext>;
  m_distortionFunc[DFunc::HAD8_fast]   = RdCost::xGetHADsFst_SIMD<vext>;
  m_distortionFunc[DFunc::HAD16_fast]  = RdCost::xGetHADsFst_SIMD<vext>;
  m_distortionFunc[DFunc::HAD32_fast]  = RdCost::xGetHADsFst_SIMD<vext>;
  m_distortionFunc[DFunc::HAD64_fast]  = RdCost::xGetHADsFst_SIMD<vext>;
  m_distortionFunc[DFunc::HAD16N_fast] = RdCost::xGetHADsFst_SIMD<vext>;

  m_distortionFunc[DFunc::SAD_INTERMEDIATE_BITDEPTH] = RdCost::xGetSAD_IBD_SIMD<vext>;

  m_distortionFunc[DFunc::MRSAD]    = RdCost::xGetMRSAD_SIMD<vext>;
  m_distortionFunc[DFunc::MRSAD2]   = RdCost::xGetMRSAD_SIMD<vext>;
  m_distortionFunc[DFunc::MRSAD4]   = RdCost::xGetMRSAD_SIMD<vext>;
  m_distortionFunc[DFunc::MRSAD8]   = RdCost::xGetMRSAD_SIMD<vext>;
  m_distortionFunc[DFunc::MRSAD16]  = RdCost::xGetMRSAD_SIMD<vext>;
  m_distortionFunc[DFunc::MRSAD32]  = RdCost::xGetMRSAD_SIMD<vext>;
  m_distortionFunc[DFunc::MRSAD64]  = RdCost::xGetMRSAD_SIMD<vext>;
  m_distortionFunc[DFunc::MRSAD16N] = RdCost::xGetMRSAD_SIMD<vext>;
  m_distortionFunc[DFunc::MRSAD12]  = RdCost::xGetMRSAD_SIMD<vext>;
  m_distortionFunc[DFunc::MRSAD24]  = RdCost::xGetMRSAD_SIMD<vext>;
  m_distortionFunc[DFunc::MRSAD48]  = RdCost::xGetMRSAD_SIMD<vext>;

  m_distortionFunc[DFunc::SAD_WITH_MASK] = xGetSADwMask_SIMD<vext>;
}

template void RdCost::_initRdCostX86<SIMDX86>();

#endif   // #if TARGET_SIMD_X86
//! \}
