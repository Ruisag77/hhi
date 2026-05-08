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

/** \file     YuvX86.cpp
    \brief    SIMD averaging.
*/

//! \ingroup CommonLib
//! \{

#include "CommonLib/CommonDef.h"
#include "CommonDefX86.h"
#include "CommonLib/Unit.h"
#include "CommonLib/Buffer.h"
#include "CommonLib/InterpolationFilter.h"

#if ENABLE_SIMD_OPT_BUFFER
#ifdef TARGET_SIMD_X86

template<X86_VEXT vext> void roundBD_SSE(const Pel *srcp, const ptrdiff_t srcStride, Pel *dest,
                                         const ptrdiff_t destStride, int width, int height, const ClpRng &clpRng)
{
  const int32_t clipbd        = clpRng.bd;
  const int32_t shiftDefault  = std::max<int>(2, (IF_INTERNAL_PREC - clipbd));
  const int32_t offsetDefault = (1 << (shiftDefault - 1)) + IF_INTERNAL_OFFS;

#if USE_AVX2
  if (vext >= AVX2 && (width & 0x0f) == 0)
  {
    __m256i voffset = _mm256_set1_epi16((short)offsetDefault);
    __m256i vibdmin = _mm256_set1_epi16((short)clpRng.min);
    __m256i vibdmax = _mm256_set1_epi16((short)clpRng.max);
    __m256i vsrc;
    for (int row = 0; row < height; row++)
    {
      for (int col = 0; col < width; col += 16)
      {
        vsrc = _mm256_lddqu_si256((__m256i *)&srcp[col]);
        vsrc = _mm256_adds_epi16(vsrc, voffset);
        vsrc = _mm256_srai_epi16(vsrc, shiftDefault);
        vsrc = _mm256_min_epi16(vibdmax, _mm256_max_epi16(vibdmin, vsrc));
        _mm256_storeu_si256((__m256i *)&dest[col], vsrc);
      }
      srcp += srcStride;
      dest += destStride;
    }
  }
  else
  {
#endif
    __m128i voffset = _mm_set1_epi16((short)offsetDefault);
    __m128i vibdmin = _mm_set1_epi16((short)clpRng.min);
    __m128i vibdmax = _mm_set1_epi16((short)clpRng.max);
    __m128i vsrc;
    for (int row = 0; row < height; row++)
    {
      int col = 0;
      for (; col < ((width >> 3) << 3); col += 8)
      {
        vsrc = _mm_lddqu_si128((__m128i *)&srcp[col]);
        vsrc = _mm_adds_epi16(vsrc, voffset);
        vsrc = _mm_srai_epi16(vsrc, shiftDefault);
        vsrc = _mm_min_epi16(vibdmax, _mm_max_epi16(vibdmin, vsrc));
        _mm_storeu_si128((__m128i *)&dest[col], vsrc);
      }
      for (; col < ((width >> 2) << 2); col += 4)
      {
        vsrc = _mm_loadl_epi64((__m128i *)&srcp[col]);
        vsrc = _mm_adds_epi16(vsrc, voffset);
        vsrc = _mm_srai_epi16(vsrc, shiftDefault);
        vsrc = _mm_min_epi16(vibdmax, _mm_max_epi16(vibdmin, vsrc));
        _mm_storel_epi64((__m128i *)&dest[col], vsrc);
      }
      for (; col < width; col++)
      {
        dest[col] = ClipPel(rightShift(srcp[col] + offsetDefault, shiftDefault), clpRng);
      }
      srcp += srcStride;
      dest += destStride;
    }
#if USE_AVX2
  }
#endif
}

template<X86_VEXT vext> void weightedAvg_SSE(const Pel *src0, const ptrdiff_t src0Stride, const Pel *src1,
                                             const ptrdiff_t src1Stride, Pel *dest, const ptrdiff_t destStride,
                                             const int8_t w0, const int8_t w1, int shift, int offset, int width,
                                             int height, const ClpRng &clpRng)
{
#if USE_AVX2
  if ((vext >= AVX2) && (width & 0x7) == 0)
  {
    __m256i mw      = _mm256_unpacklo_epi16(_mm256_set1_epi16(w0), _mm256_set1_epi16(w1));
    __m256i voffset = _mm256_set1_epi32(offset);
    __m256i vibdmin = _mm256_set1_epi16((short)clpRng.min);
    __m256i vibdmax = _mm256_set1_epi16((short)clpRng.max);
    __m256i msrc0, msrc1, msum0, msum1;

    for (int row = 0; row < height; row++)
    {
      for (int col = 0; col < width; col += 8)
      {
        msrc0 = _mm256_castsi128_si256(_mm_lddqu_si128((__m128i *)(&src0[col])));
        msrc1 = _mm256_castsi128_si256(_mm_lddqu_si128((__m128i *)(&src1[col])));
        msum0 = _mm256_unpacklo_epi16(msrc0, msrc1);
        msum1 = _mm256_unpackhi_epi16(msrc0, msrc1);
        msum0 = _mm256_madd_epi16(msum0, mw);
        msum1 = _mm256_madd_epi16(msum1, mw);
        msum0 = _mm256_add_epi32(msum0, voffset);
        msum1 = _mm256_add_epi32(msum1, voffset);
        msum0 = _mm256_srai_epi32(msum0, shift);
        msum1 = _mm256_srai_epi32(msum1, shift);
        msum0 = _mm256_packs_epi32(msum0, msum1);
        msum0 = _mm256_min_epi16(vibdmax, _mm256_max_epi16(vibdmin, msum0));
        _mm_storeu_si128((__m128i *)&dest[col], _mm256_castsi256_si128(msum0));
      }
      src0 += src0Stride;
      src1 += src1Stride;
      dest += destStride;
    }
  }
  else
  {
#endif
    __m128i mw      = _mm_unpacklo_epi16(_mm_set1_epi16(w0), _mm_set1_epi16(w1));
    __m128i voffset = _mm_set1_epi32(offset);
    __m128i vibdmin = _mm_set1_epi16((short)clpRng.min);
    __m128i vibdmax = _mm_set1_epi16((short)clpRng.max);

    for (int row = 0; row < height; row++)
    {
      int col = 0;
      for (; col < ((width >> 2) << 2); col += 4)
      {
        __m128i msrc =
          _mm_unpacklo_epi16(_mm_loadl_epi64((__m128i *)&src0[col]), _mm_loadl_epi64((__m128i *)&src1[col]));
        msrc = _mm_madd_epi16(msrc, mw);
        msrc = _mm_add_epi32(msrc, voffset);
        msrc = _mm_srai_epi32(msrc, shift);
        msrc = _mm_packs_epi32(msrc, msrc);
        msrc = _mm_min_epi16(vibdmax, _mm_max_epi16(vibdmin, msrc));
        _mm_storel_epi64((__m128i *)&dest[col], msrc);
      }
      for (; col < width; col++)
      {
        dest[col] = ClipPel(rightShift(src0[col] * w0 + src1[col] * w1 + offset, shift), clpRng);
      }
      src0 += src0Stride;
      src1 += src1Stride;
      dest += destStride;
    }
#if USE_AVX2
  }
#endif
}

template<X86_VEXT vext> void weightedAvg3_SSE(const Pel *src0, const ptrdiff_t src0Stride, const Pel *src1,
                                              const ptrdiff_t src1Stride, const Pel *src2, const ptrdiff_t src2Stride,
                                              Pel *dest, const ptrdiff_t destStride, const int8_t w0, const int8_t w1,
                                              const int8_t w2, int shift, int offset, int width, int height,
                                              const ClpRng &clpRng)
{
#if USE_AVX2
  if ((vext >= AVX2) && (width & 0x7) == 0)
  {
    __m256i zero    = _mm256_setzero_si256();
    __m256i mw01    = _mm256_unpacklo_epi16(_mm256_set1_epi16(w0), _mm256_set1_epi16(w1));
    __m256i mw2x    = _mm256_unpacklo_epi16(_mm256_set1_epi16(w2), zero);
    __m256i voffset = _mm256_set1_epi32(offset);
    __m256i vibdmin = _mm256_set1_epi16((short)clpRng.min);
    __m256i vibdmax = _mm256_set1_epi16((short)clpRng.max);
    __m256i msrc0, msrc1, msrc2, msum0, msum1, msum2, msum3;

    for (int row = 0; row < height; row++)
    {
      for (int col = 0; col < width; col += 8)
      {
        msrc0 = _mm256_castsi128_si256(_mm_lddqu_si128((__m128i *)(&src0[col])));
        msrc1 = _mm256_castsi128_si256(_mm_lddqu_si128((__m128i *)(&src1[col])));
        msrc2 = _mm256_castsi128_si256(_mm_lddqu_si128((__m128i *)(&src2[col])));
        msum0 = _mm256_unpacklo_epi16(msrc0, msrc1);
        msum1 = _mm256_unpackhi_epi16(msrc0, msrc1);
        msum2 = _mm256_unpacklo_epi16(msrc2, zero);
        msum3 = _mm256_unpackhi_epi16(msrc2, zero);
        msum0 = _mm256_madd_epi16(msum0, mw01);
        msum1 = _mm256_madd_epi16(msum1, mw01);
        msum2 = _mm256_madd_epi16(msum2, mw2x);
        msum3 = _mm256_madd_epi16(msum3, mw2x);
        msum0 = _mm256_add_epi32(msum0, voffset);
        msum1 = _mm256_add_epi32(msum1, voffset);
        msum0 = _mm256_add_epi32(msum0, msum2);
        msum1 = _mm256_add_epi32(msum1, msum3);
        msum0 = _mm256_srai_epi32(msum0, shift);
        msum1 = _mm256_srai_epi32(msum1, shift);
        msum0 = _mm256_packs_epi32(msum0, msum1);
        msum0 = _mm256_min_epi16(vibdmax, _mm256_max_epi16(vibdmin, msum0));
        _mm_storeu_si128((__m128i *)&dest[col], _mm256_castsi256_si128(msum0));
      }
      src0 += src0Stride;
      src1 += src1Stride;
      src2 += src2Stride;
      dest += destStride;
    }
  }
  else
  {
#endif
    __m128i zero    = _mm_setzero_si128();
    __m128i mw01    = _mm_unpacklo_epi16(_mm_set1_epi16(w0), _mm_set1_epi16(w1));
    __m128i mw2x    = _mm_unpacklo_epi16(_mm_set1_epi16(w2), zero);
    __m128i voffset = _mm_set1_epi32(offset);
    __m128i vibdmin = _mm_set1_epi16((short)clpRng.min);
    __m128i vibdmax = _mm_set1_epi16((short)clpRng.max);

    for (int row = 0; row < height; row++)
    {
      int col = 0;
      for (; col < ((width >> 2) << 2); col += 4)
      {
        __m128i msrc =
          _mm_unpacklo_epi16(_mm_loadl_epi64((__m128i *)&src0[col]), _mm_loadl_epi64((__m128i *)&src1[col]));
        __m128i msrc2 = _mm_unpacklo_epi16(_mm_loadl_epi64((__m128i *)&src2[col]), zero);
        msrc          = _mm_madd_epi16(msrc, mw01);
        msrc2         = _mm_madd_epi16(msrc2, mw2x);
        msrc          = _mm_add_epi32(msrc, voffset);
        msrc          = _mm_add_epi32(msrc, msrc2);
        msrc          = _mm_srai_epi32(msrc, shift);
        msrc          = _mm_packs_epi32(msrc, msrc);
        msrc          = _mm_min_epi16(vibdmax, _mm_max_epi16(vibdmin, msrc));
        _mm_storel_epi64((__m128i *)&dest[col], msrc);
      }
      for (; col < width; col++)
      {
        dest[col] = ClipPel(rightShift(src0[col] * w0 + src1[col] * w1 + src2[col] * w2 + offset, shift), clpRng);
      }
      src0 += src0Stride;
      src1 += src1Stride;
      src2 += src2Stride;
      dest += destStride;
    }
#if USE_AVX2
  }
#endif
}

template<X86_VEXT vext> void copyClip_SSE(const Pel *srcp, const ptrdiff_t srcStride, Pel *dest,
                                          const ptrdiff_t destStride, int width, int height, const ClpRng &clpRng)
{
#if USE_AVX2
  if (vext >= AVX2 && (width & 0x0f) == 0)
  {
    __m256i vibdmin = _mm256_set1_epi16((short)clpRng.min);
    __m256i vibdmax = _mm256_set1_epi16((short)clpRng.max);
    __m256i vsrc;
    for (int row = 0; row < height; row++)
    {
      for (int col = 0; col < width; col += 16)
      {
        vsrc = _mm256_lddqu_si256((__m256i *)&srcp[col]);
        vsrc = _mm256_min_epi16(vibdmax, _mm256_max_epi16(vibdmin, vsrc));
        _mm256_storeu_si256((__m256i *)&dest[col], vsrc);
      }
      srcp += srcStride;
      dest += destStride;
    }
  }
  else
  {
#endif
    __m128i vibdmin = _mm_set1_epi16((short)clpRng.min);
    __m128i vibdmax = _mm_set1_epi16((short)clpRng.max);
    __m128i vsrc;
    for (int row = 0; row < height; row++)
    {
      int col = 0;
      for (; col < ((width >> 3) << 3); col += 8)
      {
        vsrc = _mm_lddqu_si128((__m128i *)&srcp[col]);
        vsrc = _mm_min_epi16(vibdmax, _mm_max_epi16(vibdmin, vsrc));
        _mm_storeu_si128((__m128i *)&dest[col], vsrc);
      }
      for (; col < ((width >> 2) << 2); col += 4)
      {
        vsrc = _mm_loadl_epi64((__m128i *)&srcp[col]);
        vsrc = _mm_min_epi16(vibdmax, _mm_max_epi16(vibdmin, vsrc));
        _mm_storel_epi64((__m128i *)&dest[col], vsrc);
      }
      for (; col < width; col++)
      {
        dest[col] = ClipPel(srcp[col], clpRng);
      }
      srcp += srcStride;
      dest += destStride;
    }
#if USE_AVX2
  }
#endif
}

template<X86_VEXT vext, unsigned inputSize, unsigned outputSize>
void mipMatrixMul_SSE(Pel *res, const Pel *input, const uint8_t *weight, const int maxVal, const int inputOffset,
                      bool transpose)
{
  int sum = 0;
  for (int i = 0; i < inputSize; i++)
  {
    sum += input[i];
  }
  const int offset = (1 << (MIP_SHIFT_MATRIX - 1)) - MIP_OFFSET_MATRIX * sum + (inputOffset << MIP_SHIFT_MATRIX);
  CHECK(inputSize != 4 * (inputSize >> 2), "Error, input size not divisible by four");

#if USE_AVX2
  const __m256i perm     = _mm256_setr_epi32(0, 4, 1, 5, 2, 6, 3, 7);
  __m256i       vibdimin = _mm256_set1_epi16(0);
  __m256i       vibdimax = _mm256_set1_epi16(maxVal);
  if (inputSize == 4 && outputSize == 4)
  {
    __m256i voffset = _mm256_set1_epi32(offset);
    __m256i vin     = _mm256_set1_epi64x(*((const int64_t *)input));
    __m256i vw      = _mm256_load_si256((const __m256i *)(weight));

    __m256i w0 = _mm256_cvtepi8_epi16(_mm256_castsi256_si128(vw));   // w0 - w16
    __m256i w1 = _mm256_cvtepi8_epi16(_mm256_extracti128_si256(vw, 1)); // w16 -w 32

    __m256i r0 = _mm256_madd_epi16(vin, w0);
    __m256i r1 = _mm256_madd_epi16(vin, w1);
    __m256i r2 = _mm256_hadd_epi32(r0, r1);

    r2         = _mm256_add_epi32(r2, voffset);
    __m256i r3 = _mm256_srai_epi32(r2, MIP_SHIFT_MATRIX);

    vw = _mm256_load_si256((const __m256i *)(weight + 32));
    w0 = _mm256_cvtepi8_epi16(_mm256_castsi256_si128(vw));   // w0 - w16
    w1 = _mm256_cvtepi8_epi16(_mm256_extracti128_si256(vw, 1)); // w16 -w 32

    r0 = _mm256_madd_epi16(vin, w0);
    r1 = _mm256_madd_epi16(vin, w1);
    r2 = _mm256_hadd_epi32(r0, r1);

    r2 = _mm256_add_epi32(r2, voffset);
    r2 = _mm256_srai_epi32(r2, MIP_SHIFT_MATRIX);
    r2 = _mm256_packs_epi32(r3, r2);
    r2 = _mm256_permutevar8x32_epi32(r2, perm);

    r2 = _mm256_min_epi16(vibdimax, _mm256_max_epi16(vibdimin, r2));

    if (transpose)
    {
      __m256i vshuf0 = _mm256_set_epi8(0xf, 0xe, 0xb, 0xa, 0x7, 0x6, 0x3, 0x2, 0xd, 0xc, 0x9, 0x8, 0x5, 0x4, 0x1, 0x0,
                                       0xf, 0xe, 0xb, 0xa, 0x7, 0x6, 0x3, 0x2, 0xd, 0xc, 0x9, 0x8, 0x5, 0x4, 0x1, 0x0);
      r2             = _mm256_permutevar8x32_epi32(r2, _mm256_set_epi32(7, 5, 3, 1, 6, 4, 2, 0));
      r2             = _mm256_shuffle_epi8(r2, vshuf0);
    }

    _mm256_store_si256((__m256i *)&res[0], r2);
  }
  else if (inputSize == 8)
  {
    __m256i voffset = _mm256_set1_epi32(offset);
    __m128i inv     = _mm_load_si128((__m128i *)input);
    __m256i vin     = _mm256_permute2f128_si256(_mm256_castsi128_si256(inv), _mm256_castsi128_si256(inv), 2);
    __m256i r2;
    for (int i = 0; i < outputSize * outputSize; i += 16)
    {
      __m256i vw = _mm256_load_si256((const __m256i *)(weight));

      __m256i w0 = _mm256_cvtepi8_epi16(_mm256_castsi256_si128(vw));   // w0 - w16
      __m256i w1 = _mm256_cvtepi8_epi16(_mm256_extracti128_si256(vw, 1)); // w16 -w 32

      __m256i r0 = _mm256_madd_epi16(vin, w0);
      __m256i r1 = _mm256_madd_epi16(vin, w1);
      r2         = _mm256_hadd_epi32(r0, r1);

      vw = _mm256_load_si256((const __m256i *)(weight + 32));
      w0 = _mm256_cvtepi8_epi16(_mm256_castsi256_si128(vw));   // w0 - w16
      w1 = _mm256_cvtepi8_epi16(_mm256_extracti128_si256(vw, 1)); // w16 -w 32

      r0         = _mm256_madd_epi16(vin, w0);
      r1         = _mm256_madd_epi16(vin, w1);
      __m256i r4 = _mm256_hadd_epi32(r0, r1);

      r2 = _mm256_hadd_epi32(r2, r4);

      r2         = _mm256_add_epi32(r2, voffset);
      __m256i r3 = _mm256_srai_epi32(r2, MIP_SHIFT_MATRIX);

      vw = _mm256_load_si256((const __m256i *)(weight + 64));

      w0 = _mm256_cvtepi8_epi16(_mm256_castsi256_si128(vw));   // w0 - w16
      w1 = _mm256_cvtepi8_epi16(_mm256_extracti128_si256(vw, 1)); // w16 -w 32

      r0 = _mm256_madd_epi16(vin, w0);
      r1 = _mm256_madd_epi16(vin, w1);
      r2 = _mm256_hadd_epi32(r0, r1);

      vw = _mm256_load_si256((const __m256i *)(weight + 96));
      w0 = _mm256_cvtepi8_epi16(_mm256_castsi256_si128(vw));   // w0 - w16
      w1 = _mm256_cvtepi8_epi16(_mm256_extracti128_si256(vw, 1)); // w16 -w 32

      r0 = _mm256_madd_epi16(vin, w0);
      r1 = _mm256_madd_epi16(vin, w1);

      r2 = _mm256_hadd_epi32(r2, _mm256_hadd_epi32(r0, r1));

      r2 = _mm256_add_epi32(r2, voffset);
      r2 = _mm256_srai_epi32(r2, MIP_SHIFT_MATRIX);

      r2 = _mm256_permutevar8x32_epi32(r2, perm);
      r3 = _mm256_permutevar8x32_epi32(r3, perm);

      r3 = _mm256_packs_epi32(r3, r2);
      r2 = _mm256_permute4x64_epi64(r3, 0xd8);

      r2 = _mm256_min_epi16(vibdimax, _mm256_max_epi16(vibdimin, r2));

      _mm256_store_si256((__m256i *)&res[0], r2);
      res += 16;
      weight += 128;
    }

    if (transpose)
    {
      if (outputSize == 4)
      {
        __m256i vshuf0 =
          _mm256_set_epi8(0xf, 0xe, 0xb, 0xa, 0x7, 0x6, 0x3, 0x2, 0xd, 0xc, 0x9, 0x8, 0x5, 0x4, 0x1, 0x0, 0xf, 0xe, 0xb,
                          0xa, 0x7, 0x6, 0x3, 0x2, 0xd, 0xc, 0x9, 0x8, 0x5, 0x4, 0x1, 0x0);
        r2 = _mm256_permutevar8x32_epi32(r2, _mm256_set_epi32(7, 5, 3, 1, 6, 4, 2, 0));
        r2 = _mm256_shuffle_epi8(r2, vshuf0);
        _mm256_store_si256((__m256i *)(res - 16), r2);
      }
      else
      {
        res -= 64;

        __m256i va, vb, vc, vd, wa, wb, wc, wd;

        va = _mm256_load_si256((const __m256i *)res);
        vb = _mm256_load_si256((const __m256i *)(res + 16));
        vc = _mm256_load_si256((const __m256i *)(res + 32));

        va = _mm256_permute4x64_epi64(va, 0xd8);
        vb = _mm256_permute4x64_epi64(vb, 0xd8);
        vc = _mm256_permute4x64_epi64(vc, 0xd8);
        vd = _mm256_permute4x64_epi64(r2, 0xd8);

        wa = _mm256_unpacklo_epi16(va, vb);
        wb = _mm256_unpackhi_epi16(va, vb);
        wc = _mm256_unpacklo_epi16(vc, vd);
        wd = _mm256_unpackhi_epi16(vc, vd);

        va = _mm256_unpacklo_epi16(wa, wb);
        vb = _mm256_unpackhi_epi16(wa, wb);
        vc = _mm256_unpacklo_epi16(wc, wd);
        vd = _mm256_unpackhi_epi16(wc, wd);

        va = _mm256_permute4x64_epi64(va, 0xd8);
        vb = _mm256_permute4x64_epi64(vb, 0xd8);
        vc = _mm256_permute4x64_epi64(vc, 0xd8);
        vd = _mm256_permute4x64_epi64(vd, 0xd8);

        wa = _mm256_unpacklo_epi64(va, vc);
        wb = _mm256_unpacklo_epi64(vb, vd);
        wc = _mm256_unpackhi_epi64(va, vc);
        wd = _mm256_unpackhi_epi64(vb, vd);

        _mm256_store_si256((__m256i *)res, wa);
        _mm256_store_si256((__m256i *)(res + 16), wb);
        _mm256_store_si256((__m256i *)(res + 32), wc);
        _mm256_store_si256((__m256i *)(res + 48), wd);
      }
    }
  }
#else
  __m128i zero     = _mm_set1_epi16(0);
  __m128i vibdimax = _mm_set1_epi16(maxVal);
  if (inputSize == 4 && outputSize == 4)
  {
    __m128i vin     = _mm_set1_epi64x(*((const int64_t *)input));
    __m128i voffset = _mm_set1_epi32(offset);
    __m128i r2      = vin;
    __m128i r;
    for (int i = 0; i < 2; i++)
    {
      r          = r2; // save the result from the first interation
      __m128i vw = _mm_load_si128((const __m128i *)weight);

      __m128i w0 = _mm_unpacklo_epi8(vw, zero);
      __m128i w1 = _mm_unpackhi_epi8(vw, zero);

      __m128i r0 = _mm_madd_epi16(vin, w0);
      __m128i r1 = _mm_madd_epi16(vin, w1);
      r2         = _mm_hadd_epi32(r0, r1);

      r2         = _mm_add_epi32(r2, voffset);
      __m128i r3 = _mm_srai_epi32(r2, MIP_SHIFT_MATRIX);

      vw = _mm_load_si128((const __m128i *)(weight + 16));
      w0 = _mm_unpacklo_epi8(vw, zero);
      w1 = _mm_unpackhi_epi8(vw, zero);

      r0 = _mm_madd_epi16(vin, w0);
      r1 = _mm_madd_epi16(vin, w1);
      r2 = _mm_hadd_epi32(r0, r1);

      r2 = _mm_add_epi32(r2, voffset);
      r2 = _mm_srai_epi32(r2, MIP_SHIFT_MATRIX);
      r2 = _mm_packs_epi32(r3, r2);

      r2 = _mm_min_epi16(vibdimax, _mm_max_epi16(zero, r2));

      _mm_store_si128((__m128i *)&res[0], r2);
      res += 8;
      weight += 32;
    }

    if (transpose)
    {
      __m128i vc, vd, va, vb;
      vc = _mm_unpacklo_epi16(r, r2);
      vd = _mm_unpackhi_epi16(r, r2);

      va = _mm_unpacklo_epi16(vc, vd);
      vb = _mm_unpackhi_epi16(vc, vd);

      _mm_store_si128((__m128i *)(res - 16), va);
      _mm_store_si128((__m128i *)(res - 8), vb);
    }
  }
  else
  {
    __m128i vin     = _mm_load_si128((const __m128i *)input);
    __m128i voffset = _mm_set1_epi32(offset);

    for (int i = 0; i < outputSize * outputSize; i += 4)
    {
      __m128i vw = _mm_load_si128((const __m128i *)(weight));

      __m128i w0 = _mm_unpacklo_epi8(vw, zero);
      __m128i w1 = _mm_unpackhi_epi8(vw, zero);

      __m128i r0 = _mm_madd_epi16(vin, w0);
      __m128i r1 = _mm_madd_epi16(vin, w1);
      __m128i r2 = _mm_hadd_epi32(r0, r1);

      vw = _mm_load_si128((const __m128i *)(weight + 16));
      w0 = _mm_unpacklo_epi8(vw, zero);
      w1 = _mm_unpackhi_epi8(vw, zero);

      r0 = _mm_madd_epi16(vin, w0);
      r1 = _mm_madd_epi16(vin, w1);

      r2 = _mm_hadd_epi32(r2, _mm_hadd_epi32(r0, r1));

      r2 = _mm_add_epi32(r2, voffset);
      r2 = _mm_srai_epi32(r2, MIP_SHIFT_MATRIX);

      r2 = _mm_packs_epi32(r2, r2);

      r2 = _mm_min_epi16(vibdimax, _mm_max_epi16(zero, r2));

      _mm_storel_epi64((__m128i *)&res[0], r2);
      res += 4;
      weight += 32;
    }

    if (transpose)
    {
      if (outputSize == 4)
      {
        res -= 16;
        __m128i vc, vd, va, vb;
        va = _mm_load_si128((const __m128i *)(res));
        vb = _mm_load_si128((const __m128i *)(res + 8));

        vc = _mm_unpacklo_epi16(va, vb);
        vd = _mm_unpackhi_epi16(va, vb);

        va = _mm_unpacklo_epi16(vc, vd);
        vb = _mm_unpackhi_epi16(vc, vd);

        _mm_store_si128((__m128i *)(res), va);
        _mm_store_si128((__m128i *)(res + 8), vb);
      }
      else
      {
        res -= 64;
        __m128i va, vb, vc, vd, ve, vf, vg, vh;

        va = _mm_load_si128((const __m128i *)(res));
        vb = _mm_load_si128((const __m128i *)(res + 8));
        vc = _mm_load_si128((const __m128i *)(res + 16));
        vd = _mm_load_si128((const __m128i *)(res + 24));
        ve = _mm_load_si128((const __m128i *)(res + 32));
        vf = _mm_load_si128((const __m128i *)(res + 40));
        vg = _mm_load_si128((const __m128i *)(res + 48));
        vh = _mm_load_si128((const __m128i *)(res + 56));

        __m128i va01b01 = _mm_unpacklo_epi16(va, vb);
        __m128i va23b23 = _mm_unpackhi_epi16(va, vb);
        __m128i vc01d01 = _mm_unpacklo_epi16(vc, vd);
        __m128i vc23d23 = _mm_unpackhi_epi16(vc, vd);
        __m128i ve01f01 = _mm_unpacklo_epi16(ve, vf);
        __m128i ve23f23 = _mm_unpackhi_epi16(ve, vf);
        __m128i vg01h01 = _mm_unpacklo_epi16(vg, vh);
        __m128i vg23h23 = _mm_unpackhi_epi16(vg, vh);

        va = _mm_unpacklo_epi32(va01b01, vc01d01);
        vb = _mm_unpackhi_epi32(va01b01, vc01d01);
        vc = _mm_unpacklo_epi32(va23b23, vc23d23);
        vd = _mm_unpackhi_epi32(va23b23, vc23d23);
        ve = _mm_unpacklo_epi32(ve01f01, vg01h01);
        vf = _mm_unpackhi_epi32(ve01f01, vg01h01);
        vg = _mm_unpacklo_epi32(ve23f23, vg23h23);
        vh = _mm_unpackhi_epi32(ve23f23, vg23h23);

        va01b01 = _mm_unpacklo_epi64(va, ve);
        va23b23 = _mm_unpackhi_epi64(va, ve);
        vc01d01 = _mm_unpacklo_epi64(vb, vf);
        vc23d23 = _mm_unpackhi_epi64(vb, vf);
        ve01f01 = _mm_unpacklo_epi64(vc, vg);
        ve23f23 = _mm_unpackhi_epi64(vc, vg);
        vg01h01 = _mm_unpacklo_epi64(vd, vh);
        vg23h23 = _mm_unpackhi_epi64(vd, vh);

        _mm_store_si128((__m128i *)(res), va01b01);
        _mm_store_si128((__m128i *)(res + 8), va23b23);
        _mm_store_si128((__m128i *)(res + 16), vc01d01);
        _mm_store_si128((__m128i *)(res + 24), vc23d23);
        _mm_store_si128((__m128i *)(res + 32), ve01f01);
        _mm_store_si128((__m128i *)(res + 40), ve23f23);
        _mm_store_si128((__m128i *)(res + 48), vg01h01);
        _mm_store_si128((__m128i *)(res + 56), vg23h23);
      }
    }
  }
#endif
}

template<X86_VEXT vext, int W> void copyClip_SSE(const int16_t *src, ptrdiff_t srcStride, int16_t *dst,
                                                 ptrdiff_t dstStride, int width, int height, const ClpRng &clpRng)
{
  if (vext >= AVX2 && (width & 15) == 0 && W == 8)
  {
#if USE_AVX2
    __m256i vbdmin = _mm256_set1_epi16(clpRng.min);
    __m256i vbdmax = _mm256_set1_epi16(clpRng.max);

    for (int row = 0; row < height; row++)
    {
      for (int col = 0; col < width; col += 16)
      {
        __m256i val = _mm256_loadu_si256((const __m256i *)&src[col]);
        val         = _mm256_min_epi16(vbdmax, _mm256_max_epi16(vbdmin, val));
        _mm256_storeu_si256((__m256i *)&dst[col], val);
      }

      src += srcStride;
      dst += dstStride;
    }
#endif
  }
  else if (W == 8)
  {
    __m128i vbdmin = _mm_set1_epi16(clpRng.min);
    __m128i vbdmax = _mm_set1_epi16(clpRng.max);

    for (int row = 0; row < height; row++)
    {
      for (int col = 0; col < width; col += 8)
      {
        __m128i val = _mm_loadu_si128((const __m128i *)&src[col]);
        val         = _mm_min_epi16(vbdmax, _mm_max_epi16(vbdmin, val));
        _mm_storeu_si128((__m128i *)&dst[col], val);
      }

      src += srcStride;
      dst += dstStride;
    }
  }
  else
  {
    __m128i vbdmin = _mm_set1_epi16(clpRng.min);
    __m128i vbdmax = _mm_set1_epi16(clpRng.max);

    for (int row = 0; row < height; row++)
    {
      for (int col = 0; col < width; col += 4)
      {
        __m128i val;
        val = _mm_loadl_epi64((const __m128i *)&src[col]);
        val = _mm_min_epi16(vbdmax, _mm_max_epi16(vbdmin, val));
        _mm_storel_epi64((__m128i *)&dst[col], val);
      }

      src += srcStride;
      dst += dstStride;
    }
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template<X86_VEXT vext, int W> void addAvg_SSE(const int16_t *src0, ptrdiff_t src0Stride, const int16_t *src1,
                                               ptrdiff_t src1Stride, int16_t *dst, ptrdiff_t dstStride, int width,
                                               int height, int shift, int offset, const ClpRng &clpRng,
                                               bool *mcMask[NUM_RPL01], int mcStride, bool *isOOB)
{
  if (mcMask == NULL || (!isOOB[RPL0] && !isOOB[RPL1]))
  {
    if (W == 8)
    {
      CHECK(offset & 1, "offset must be even");
      CHECK(offset < -32768 || offset > 32767, "offset must be a 16-bit value");

      __m128i vibdimin = _mm_set1_epi16(clpRng.min);
      __m128i vibdimax = _mm_set1_epi16(clpRng.max);

      for (int row = 0; row < height; row++)
      {
        for (int col = 0; col < width; col += 8)
        {
          __m128i vsrc0 = _mm_loadu_si128((const __m128i *)&src0[col]);
          __m128i vsrc1 = _mm_loadu_si128((const __m128i *)&src1[col]);

          vsrc0 = _mm_xor_si128(vsrc0, _mm_set1_epi16(0x7fff));
          vsrc1 = _mm_xor_si128(vsrc1, _mm_set1_epi16(0x7fff));
          vsrc0 = _mm_avg_epu16(vsrc0, vsrc1);
          vsrc0 = _mm_xor_si128(vsrc0, _mm_set1_epi16(0x7fff));
          vsrc0 = _mm_adds_epi16(vsrc0, _mm_set1_epi16(offset >> 1));
          vsrc0 = _mm_sra_epi16(vsrc0, _mm_cvtsi32_si128(shift - 1));
          vsrc0 = _mm_max_epi16(vsrc0, vibdimin);
          vsrc0 = _mm_min_epi16(vsrc0, vibdimax);
          _mm_storeu_si128((__m128i *)&dst[col], vsrc0);
        }

        src0 += src0Stride;
        src1 += src1Stride;
        dst += dstStride;
      }
    }
    else if (W == 4)
    {
      __m128i vzero    = _mm_setzero_si128();
      __m128i voffset  = _mm_set1_epi32(offset);
      __m128i vibdimin = _mm_set1_epi16(clpRng.min);
      __m128i vibdimax = _mm_set1_epi16(clpRng.max);

      for (int row = 0; row < height; row++)
      {
        for (int col = 0; col < width; col += 4)
        {
          __m128i vsum = _mm_loadl_epi64((const __m128i *)&src0[col]);
          __m128i vdst = _mm_loadl_epi64((const __m128i *)&src1[col]);
          vsum         = _mm_cvtepi16_epi32(vsum);
          vdst         = _mm_cvtepi16_epi32(vdst);
          vsum         = _mm_add_epi32(vsum, vdst);
          vsum         = _mm_add_epi32(vsum, voffset);
          vsum         = _mm_srai_epi32(vsum, shift);
          vsum         = _mm_packs_epi32(vsum, vzero);

          vsum = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsum));
          _mm_storel_epi64((__m128i *)&dst[col], vsum);
        }

        src0 += src0Stride;
        src1 += src1Stride;
        dst += dstStride;
      }
    }
    else
    {
      THROW("Unsupported size");
    }
  }
  else
  {
    const int clipbd    = clpRng.bd;
    const int shiftNum  = IF_INTERNAL_FRAC_BITS(clipbd) + 1;
    const int offset    = (1 << (shiftNum - 1)) + 2 * IF_INTERNAL_OFFS;
    int       shiftNum2 = IF_INTERNAL_FRAC_BITS(clipbd);
    const int offset2   = (1 << (shiftNum2 - 1)) + IF_INTERNAL_OFFS;
    bool     *pMcMask0  = mcMask[0];
    bool     *pMcMask1  = mcMask[1];
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        bool oob0 = pMcMask0[x];
        bool oob1 = pMcMask1[x];
        if (oob0 && !oob1)
        {
          dst[x] = ClipPel(rightShift(src1[x] + offset2, shiftNum2), clpRng);
        }
        else if (!oob0 && oob1)
        {
          dst[x] = ClipPel(rightShift(src0[x] + offset2, shiftNum2), clpRng);
        }
        else
        {
          dst[x] = ClipPel(rightShift((src0[x] + src1[x] + offset), shiftNum), clpRng);
        }
      }
      pMcMask0 += mcStride;
      pMcMask1 += mcStride;
      src0 += src0Stride;
      src1 += src1Stride;
      dst += dstStride;
    }
  }
}

template<X86_VEXT vext, int w>
void toLast_SSE(Pel *src, ptrdiff_t srcStride, int width, int height, int shiftNum, int offset, const ClpRng &clpRng)
{
  __m128i vzero    = _mm_setzero_si128();
  __m128i voffset  = _mm_set1_epi32(offset);
  __m128i vibdimin = _mm_set1_epi16(clpRng.min);
  __m128i vibdimax = _mm_set1_epi16(clpRng.max);

  if (w == 4)
  {
    for (int row = 0; row < height; row++)
    {
      for (int col = 0; col < width; col += 4)
      {
        __m128i vsrc = _mm_loadl_epi64((const __m128i *)&src[col]);
        vsrc         = _mm_cvtepi16_epi32(vsrc);
        vsrc         = _mm_add_epi32(vsrc, voffset);
        vsrc         = _mm_srai_epi32(vsrc, shiftNum);
        vsrc         = _mm_packs_epi32(vsrc, vzero);
        vsrc         = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsrc));
        _mm_storel_epi64((__m128i *)&src[col], vsrc);
      }
      src += srcStride;
    }
  }
  else if (w == 2)
  {
    for (int row = 0; row < height; row++)
    {
      for (int col = 0; col < width; col += 2)
      {
        __m128i vsrc        = _mm_cvtsi32_si128(*(int *)(&src[col]));
        vsrc                = _mm_cvtepi16_epi32(vsrc);
        vsrc                = _mm_add_epi32(vsrc, voffset);
        vsrc                = _mm_srai_epi32(vsrc, shiftNum);
        vsrc                = _mm_packs_epi32(vsrc, vzero);
        vsrc                = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsrc));
        *(int *)(&src[col]) = _mm_cvtsi128_si32(vsrc);
      }
      src += srcStride;
    }
  }
  else
  {
    THROW("Size unsupported!");
  }
}

template<X86_VEXT vext, int w> void licRemoveWeightHighFreq_SSE(const Pel *src0, const Pel *src1, Pel *dst, int length,
                                                                int w0, int w1, int offset, const ClpRng &clpRng)
{
  __m128i vzero    = _mm_setzero_si128();
  __m128i voffset  = _mm_set1_epi32(offset);
  __m128i vw0      = _mm_set1_epi32(w0);
  __m128i vw1      = _mm_set1_epi32(w1);
  __m128i vibdimin = _mm_set1_epi16(clpRng.min);
  __m128i vibdimax = _mm_set1_epi16(clpRng.max);

  if (w == 4)
  {
    for (int col = 0; col < length; col += 4)
    {
      __m128i vsrc0 = _mm_loadl_epi64((const __m128i *)&src0[col]);
      __m128i vsrc1 = _mm_loadl_epi64((const __m128i *)&src1[col]);
      vsrc0         = _mm_cvtepi16_epi32(vsrc0);
      vsrc1         = _mm_cvtepi16_epi32(vsrc1);
      vsrc0         = _mm_mullo_epi32(vsrc0, vw0);
      vsrc1         = _mm_mullo_epi32(vsrc1, vw1);
      vsrc0         = _mm_add_epi32(_mm_sub_epi32(vsrc0, vsrc1), voffset);
      vsrc0         = _mm_srai_epi32(vsrc0, 16);
      vsrc0         = _mm_packs_epi32(vsrc0, vzero);
      vsrc0         = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsrc0));
      _mm_storel_epi64((__m128i *)&dst[col], vsrc0);
    }
  }
  else if (w == 2)
  {
    for (int col = 0; col < length; col += 2)
    {
      __m128i vsrc0       = _mm_cvtsi32_si128(*(int *)(&src0[col]));
      __m128i vsrc1       = _mm_cvtsi32_si128(*(int *)(&src1[col]));
      vsrc0               = _mm_cvtepi16_epi32(vsrc0);
      vsrc1               = _mm_cvtepi16_epi32(vsrc1);
      vsrc0               = _mm_mullo_epi32(vsrc0, vw0);
      vsrc1               = _mm_mullo_epi32(vsrc1, vw1);
      vsrc0               = _mm_add_epi32(_mm_sub_epi32(vsrc0, vsrc1), voffset);
      vsrc0               = _mm_srai_epi32(vsrc0, 16);
      vsrc0               = _mm_packs_epi32(vsrc0, vzero);
      vsrc0               = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsrc0));
      *(int *)(&dst[col]) = _mm_cvtsi128_si32(vsrc0);
    }
  }
  else
  {
    THROW("Unsupported size!");
  }
}

template<X86_VEXT vext>
void copyBufferSimd(const Pel *src, ptrdiff_t srcStride, Pel *dst, ptrdiff_t dstStride, int width, int height)
{
  if (width < 8)
  {
    CHECK(width < 4, "width must be at least 4");

    for (size_t x = 0; x < width; x += 4)
    {
      if (x > width - 4)
      {
        x = width - 4;
      }
      for (size_t y = 0; y < height; y++)
      {
        __m128i val = _mm_loadl_epi64((const __m128i *)(src + y * srcStride + x));
        _mm_storel_epi64((__m128i *)(dst + y * dstStride + x), val);
      }
    }
  }
  else
  {
    for (size_t x = 0; x < width; x += 8)
    {
      if (x > width - 8)
      {
        x = width - 8;
      }
      for (size_t y = 0; y < height; y++)
      {
        __m128i val = _mm_loadu_si128((const __m128i *)(src + y * srcStride + x));
        _mm_storeu_si128((__m128i *)(dst + y * dstStride + x), val);
      }
    }
  }
}

template<X86_VEXT vext> void paddingSimd(Pel *dst, ptrdiff_t stride, int width, int height, int padSize)
{
  size_t extWidth = width + 2 * padSize;
  CHECK(extWidth < 8, "width plus 2 times padding size must be at least 8");

  if (padSize == 1)
  {
    for (ptrdiff_t i = 0; i < height; i++)
    {
      Pel left                = dst[i * stride];
      Pel right               = dst[i * stride + width - 1];
      dst[i * stride - 1]     = left;
      dst[i * stride + width] = right;
    }

    dst -= 1;

    for (size_t i = 0; i < extWidth - 8; i++)
    {
      __m128i top = _mm_loadu_si128((const __m128i *)(dst + i));
      _mm_storeu_si128((__m128i *)(dst - stride + i), top);
    }
    __m128i top = _mm_loadu_si128((const __m128i *)(dst + extWidth - 8));
    _mm_storeu_si128((__m128i *)(dst - stride + extWidth - 8), top);

    dst += height * stride;

    for (size_t i = 0; i < extWidth - 8; i++)
    {
      __m128i bottom = _mm_loadu_si128((const __m128i *)(dst - stride + i));
      _mm_storeu_si128((__m128i *)(dst + i), bottom);
    }
    __m128i bottom = _mm_loadu_si128((const __m128i *)(dst - stride + extWidth - 8));
    _mm_storeu_si128((__m128i *)(dst + extWidth - 8), bottom);
  }
  else if (padSize == 2)
  {
    for (ptrdiff_t i = 0; i < height; i++)
    {
      Pel left                    = dst[i * stride];
      Pel right                   = dst[i * stride + width - 1];
      dst[i * stride - 2]         = left;
      dst[i * stride - 1]         = left;
      dst[i * stride + width]     = right;
      dst[i * stride + width + 1] = right;
    }

    dst -= 2;

    for (size_t i = 0; i < extWidth - 8; i++)
    {
      __m128i top = _mm_loadu_si128((const __m128i *)(dst + i));
      _mm_storeu_si128((__m128i *)(dst - 2 * stride + i), top);
      _mm_storeu_si128((__m128i *)(dst - stride + i), top);
    }
    __m128i top = _mm_loadu_si128((const __m128i *)(dst + extWidth - 8));
    _mm_storeu_si128((__m128i *)(dst - 2 * stride + extWidth - 8), top);
    _mm_storeu_si128((__m128i *)(dst - stride + extWidth - 8), top);

    dst += height * stride;

    for (size_t i = 0; i < extWidth - 8; i++)
    {
      __m128i bottom = _mm_loadu_si128((const __m128i *)(dst - stride + i));
      _mm_storeu_si128((__m128i *)(dst + i), bottom);
      _mm_storeu_si128((__m128i *)(dst + stride + i), bottom);
    }
    __m128i bottom = _mm_loadu_si128((const __m128i *)(dst - stride + extWidth - 8));
    _mm_storeu_si128((__m128i *)(dst + extWidth - 8), bottom);
    _mm_storeu_si128((__m128i *)(dst + stride + extWidth - 8), bottom);
  }
  else
  {
    THROW("padding size must be 1 or 2");
  }
}
template<X86_VEXT vext> void calcBIOParameter_SSE(const Pel *srcY0Tmp, const Pel *srcY1Tmp, Pel *gradX0, Pel *gradX1,
                                                  Pel *gradY0, Pel *gradY1, int width, int height, const int src0Stride,
                                                  const int src1Stride, const int widthG, const int bitDepth,
                                                  Pel *absGX, Pel *absGY, Pel *dIX, Pel *dIY, Pel *signGY_GX, Pel *dI)
{
  width -= 2;
  height -= 2;
  const int bioParamOffset = widthG + 1;
  srcY0Tmp += src0Stride + 1;
  srcY1Tmp += src1Stride + 1;
  gradX0 += bioParamOffset;
  gradX1 += bioParamOffset;
  gradY0 += bioParamOffset;
  gradY1 += bioParamOffset;
  absGX += bioParamOffset;
  absGY += bioParamOffset;
  dIX += bioParamOffset;
  dIY += bioParamOffset;
  signGY_GX += bioParamOffset;
  if (dI)
  {
    dI += bioParamOffset;
  }
  int shift4 = 4;
  int shift5 = 1;
#ifdef USE_AVX2
  if (width == 8)
  {
    for (int y = 0; y < height; y++)
    {
      __m128i shiftSrcY0Tmp = _mm_srai_epi16(_mm_loadu_si128((const __m128i *)srcY0Tmp), shift4);
      __m128i shiftSrcY1Tmp = _mm_srai_epi16(_mm_loadu_si128((const __m128i *)srcY1Tmp), shift4);
      __m128i sumGradX =
        _mm_add_epi16(_mm_loadu_si128((const __m128i *)gradX0), _mm_loadu_si128((const __m128i *)gradX1));
      __m128i sumGradY =
        _mm_add_epi16(_mm_loadu_si128((const __m128i *)gradY0), _mm_loadu_si128((const __m128i *)gradY1));

      __m128i subTemp1  = _mm_sub_epi16(shiftSrcY1Tmp, shiftSrcY0Tmp);
      __m128i packTempX = _mm_srai_epi16(sumGradX, shift5);
      __m128i packTempY = _mm_srai_epi16(sumGradY, shift5);

      __m128i gX_tmp = _mm_abs_epi16(packTempX);
      __m128i gY_tmp = _mm_abs_epi16(packTempY);
      if (dI)
      {
        _mm_storeu_si128((__m128i *)dI, subTemp1);
      }
      __m128i dIX_tmp       = _mm_sign_epi16(subTemp1, packTempX);
      __m128i dIY_tmp       = _mm_sign_epi16(subTemp1, packTempY);
      __m128i signGY_GX_tmp = _mm_sign_epi16(packTempX, packTempY);

      _mm_storeu_si128((__m128i *)absGX, gX_tmp);
      _mm_storeu_si128((__m128i *)absGY, gY_tmp);
      _mm_storeu_si128((__m128i *)dIX, dIX_tmp);
      _mm_storeu_si128((__m128i *)dIY, dIY_tmp);
      _mm_storeu_si128((__m128i *)signGY_GX, signGY_GX_tmp);
      srcY0Tmp += src0Stride;
      srcY1Tmp += src1Stride;
      gradX0 += widthG;
      gradX1 += widthG;
      gradY0 += widthG;
      gradY1 += widthG;
      absGX += widthG;
      absGY += widthG;
      if (dI)
      {
        dI += widthG;
      }
      dIX += widthG;
      dIY += widthG;
      signGY_GX += widthG;
    }
  }
  else   // width = 12, 20, 36, 68, 132, 260
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 16)
      {
        __m256i shiftSrcY0Tmp = _mm256_srai_epi16(_mm256_loadu_si256((const __m256i *)(srcY0Tmp + x)), shift4);
        __m256i shiftSrcY1Tmp = _mm256_srai_epi16(_mm256_loadu_si256((const __m256i *)(srcY1Tmp + x)), shift4);
        __m256i sumGradX      = _mm256_add_epi16(_mm256_loadu_si256((const __m256i *)(gradX0 + x)),
                                                 _mm256_loadu_si256((const __m256i *)(gradX1 + x)));
        __m256i sumGradY      = _mm256_add_epi16(_mm256_loadu_si256((const __m256i *)(gradY0 + x)),
                                                 _mm256_loadu_si256((const __m256i *)(gradY1 + x)));

        __m256i subTemp1  = _mm256_sub_epi16(shiftSrcY1Tmp, shiftSrcY0Tmp);
        __m256i packTempX = _mm256_srai_epi16(sumGradX, shift5);
        __m256i packTempY = _mm256_srai_epi16(sumGradY, shift5);

        __m256i gX_tmp = _mm256_abs_epi16(packTempX);
        __m256i gY_tmp = _mm256_abs_epi16(packTempY);
        if (dI)
        {
          _mm256_storeu_si256((__m256i *)(dI + x), subTemp1);
        }
        __m256i dIX_tmp       = _mm256_sign_epi16(subTemp1, packTempX);
        __m256i dIY_tmp       = _mm256_sign_epi16(subTemp1, packTempY);
        __m256i signGY_GX_tmp = _mm256_sign_epi16(packTempX, packTempY);

        _mm256_storeu_si256((__m256i *)(absGX + x), gX_tmp);
        _mm256_storeu_si256((__m256i *)(absGY + x), gY_tmp);
        _mm256_storeu_si256((__m256i *)(dIX + x), dIX_tmp);
        _mm256_storeu_si256((__m256i *)(dIY + x), dIY_tmp);
        _mm256_storeu_si256((__m256i *)(signGY_GX + x), signGY_GX_tmp);
      }
      srcY0Tmp += src0Stride;
      srcY1Tmp += src1Stride;
      gradX0 += widthG;
      gradX1 += widthG;
      gradY0 += widthG;
      gradY1 += widthG;
      absGX += widthG;
      absGY += widthG;
      if (dI)
      {
        dI += widthG;
      }
      dIX += widthG;
      dIY += widthG;
      signGY_GX += widthG;
    }
  }
#else
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x += 8)
    {
      __m128i shiftSrcY0Tmp = _mm_srai_epi16(_mm_loadu_si128((const __m128i *)(srcY0Tmp + x)), shift4);
      __m128i shiftSrcY1Tmp = _mm_srai_epi16(_mm_loadu_si128((const __m128i *)(srcY1Tmp + x)), shift4);
      __m128i sumGradX =
        _mm_add_epi16(_mm_loadu_si128((const __m128i *)(gradX0 + x)), _mm_loadu_si128((const __m128i *)(gradX1 + x)));
      __m128i sumGradY =
        _mm_add_epi16(_mm_loadu_si128((const __m128i *)(gradY0 + x)), _mm_loadu_si128((const __m128i *)(gradY1 + x)));

      __m128i subTemp1  = _mm_sub_epi16(shiftSrcY1Tmp, shiftSrcY0Tmp);
      __m128i packTempX = _mm_srai_epi16(sumGradX, shift5);
      __m128i packTempY = _mm_srai_epi16(sumGradY, shift5);

      __m128i gX_tmp = _mm_abs_epi16(packTempX);
      __m128i gY_tmp = _mm_abs_epi16(packTempY);
      if (dI)
      {
        _mm_storeu_si128((__m128i *)(dI + x), subTemp1);
      }
      __m128i dIX_tmp       = _mm_sign_epi16(subTemp1, packTempX);
      __m128i dIY_tmp       = _mm_sign_epi16(subTemp1, packTempY);
      __m128i signGY_GX_tmp = _mm_sign_epi16(packTempX, packTempY);

      _mm_storeu_si128((__m128i *)(absGX + x), gX_tmp);
      _mm_storeu_si128((__m128i *)(absGY + x), gY_tmp);
      _mm_storeu_si128((__m128i *)(dIX + x), dIX_tmp);
      _mm_storeu_si128((__m128i *)(dIY + x), dIY_tmp);
      _mm_storeu_si128((__m128i *)(signGY_GX + x), signGY_GX_tmp);
    }
    srcY0Tmp += src0Stride;
    srcY1Tmp += src1Stride;
    gradX0 += widthG;
    gradX1 += widthG;
    gradY0 += widthG;
    gradY1 += widthG;
    absGX += widthG;
    absGY += widthG;
    dIX += widthG;
    dIY += widthG;
    if (dI)
    {
      dI += widthG;
    }
    signGY_GX += widthG;
  }
#endif
}

template<X86_VEXT vext, int ww>
void calcBIOParamSum5NOSIMCore_SSE(int32_t *absGX, int32_t *absGY, int32_t *dIX, int32_t *dIY, int32_t *signGyGx,
                                   const int widthG, const int width, const int height, int *sumAbsGX, int *sumAbsGY,
                                   int *sumDIX, int *sumDIY, int *sumSignGyGx, Pel *dI, Pel *gX, Pel *gY)
{
#ifdef USE_AVX2
  int       a = 1, b = 2, c = 4;
  const int shift1 = 1, shift2 = 2;
  __m256i   vmask[4]         = { _mm256_setr_epi32(a, b, c, b, a, 0, 0, 0), _mm256_setr_epi32(0, a, b, c, b, a, 0, 0),
                                 _mm256_setr_epi32(0, 0, a, b, c, b, a, 0), _mm256_setr_epi32(0, 0, 0, a, b, c, b, a) };
  __m256i   vzero            = _mm256_setzero_si256();
  __m256i   sumAbsGXTmp32    = vzero;
  __m256i   dummy            = vzero;
  __m256i   sumDIXTmp32      = vzero;
  __m256i   sumAbsGYTmp32    = vzero;
  __m256i   sumDIYTmp32      = vzero;
  __m256i   sumSignGyGxTmp32 = vzero;

  __m128i vmask2[4]  = { _mm_setr_epi16(a, b, c, b, a, 0, 0, 0), _mm_setr_epi16(0, a, b, c, b, a, 0, 0),
                         _mm_setr_epi16(0, 0, a, b, c, b, a, 0), _mm_setr_epi16(0, 0, 0, a, b, c, b, a) };
  __m128i vzero2     = _mm_setzero_si128();
  __m128i sumX0      = vzero2;
  __m128i sumX1      = vzero2;
  __m128i sumMean    = vzero2;
  __m128i sumAbsMean = vzero2;
  __m128i dummy2     = vzero2;
  __m128i fOne[4]    = { _mm_setr_epi16(1, 1, 1, 1, 1, 0, 0, 0), _mm_setr_epi16(0, 1, 1, 1, 1, 1, 0, 0),
                         _mm_setr_epi16(0, 0, 1, 1, 1, 1, 1, 0), _mm_setr_epi16(0, 0, 0, 1, 1, 1, 1, 1) };
  int     mean4[4];
  int     absMean4[4];

  const int widthG_2 = (widthG << 1);
  const int widthG_3 = widthG_2 + widthG;
  const int widthG_4 = widthG_3 + widthG;

  int sX0[4], sX1[4];
  int regVxVy =
    2528; // = ((1 << 11) * 100)/81. 100 is summation of the new weights, 81 was the summation of the old weights
  if (ww == 4)
  {
    for (int y = 0; y < height; y++)
    {
      const int sampleIdx = y * width;
      sumAbsGXTmp32 =
        _mm256_add_epi32(_mm256_loadu_si256((const __m256i *)absGX),
                         _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(absGX + widthG)), shift1));
      sumAbsGXTmp32 = _mm256_add_epi32(
        sumAbsGXTmp32, _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(absGX + widthG_2)), shift2));
      sumAbsGXTmp32 = _mm256_add_epi32(
        sumAbsGXTmp32, _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(absGX + widthG_3)), shift1));
      sumAbsGXTmp32 = _mm256_add_epi32(sumAbsGXTmp32, _mm256_loadu_si256((const __m256i *)(absGX + widthG_4)));

      dummy = _mm256_mullo_epi32(sumAbsGXTmp32, vmask[0]);
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
      sumAbsGX[sampleIdx] =
        _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
      dummy = _mm256_mullo_epi32(sumAbsGXTmp32, vmask[1]);
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
      sumAbsGX[sampleIdx + 1] =
        _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
      dummy = _mm256_mullo_epi32(sumAbsGXTmp32, vmask[2]);
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
      sumAbsGX[sampleIdx + 2] =
        _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
      dummy = _mm256_mullo_epi32(sumAbsGXTmp32, vmask[3]);
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
      sumAbsGX[sampleIdx + 3] =
        _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));

      sumAbsGYTmp32 =
        _mm256_add_epi32(_mm256_loadu_si256((const __m256i *)absGY),
                         _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(absGY + widthG)), shift1));
      sumAbsGYTmp32 = _mm256_add_epi32(
        sumAbsGYTmp32, _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(absGY + widthG_2)), shift2));
      sumAbsGYTmp32 = _mm256_add_epi32(
        sumAbsGYTmp32, _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(absGY + widthG_3)), shift1));
      sumAbsGYTmp32 = _mm256_add_epi32(sumAbsGYTmp32, _mm256_loadu_si256((const __m256i *)(absGY + widthG_4)));

      dummy = _mm256_mullo_epi32(sumAbsGYTmp32, vmask[0]);
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
      sumAbsGY[sampleIdx] =
        _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
      dummy = _mm256_mullo_epi32(sumAbsGYTmp32, vmask[1]);
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
      sumAbsGY[sampleIdx + 1] =
        _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
      dummy = _mm256_mullo_epi32(sumAbsGYTmp32, vmask[2]);
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
      sumAbsGY[sampleIdx + 2] =
        _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
      dummy = _mm256_mullo_epi32(sumAbsGYTmp32, vmask[3]);
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
      sumAbsGY[sampleIdx + 3] =
        _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));

      sumSignGyGxTmp32 =
        _mm256_add_epi32(_mm256_loadu_si256((const __m256i *)signGyGx),
                         _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(signGyGx + widthG)), shift1));
      sumSignGyGxTmp32 = _mm256_add_epi32(
        sumSignGyGxTmp32, _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(signGyGx + widthG_2)), shift2));
      sumSignGyGxTmp32 = _mm256_add_epi32(
        sumSignGyGxTmp32, _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(signGyGx + widthG_3)), shift1));
      sumSignGyGxTmp32 = _mm256_add_epi32(sumSignGyGxTmp32, _mm256_loadu_si256((const __m256i *)(signGyGx + widthG_4)));

      dummy = _mm256_mullo_epi32(sumSignGyGxTmp32, vmask[0]);
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
      sumSignGyGx[sampleIdx] =
        _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
      dummy = _mm256_mullo_epi32(sumSignGyGxTmp32, vmask[1]);
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
      sumSignGyGx[sampleIdx + 1] =
        _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
      dummy = _mm256_mullo_epi32(sumSignGyGxTmp32, vmask[2]);
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
      sumSignGyGx[sampleIdx + 2] =
        _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
      dummy = _mm256_mullo_epi32(sumSignGyGxTmp32, vmask[3]);
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
      sumSignGyGx[sampleIdx + 3] =
        _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));

      sumDIXTmp32 = _mm256_add_epi32(_mm256_loadu_si256((const __m256i *)dIX),
                                     _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(dIX + widthG)), shift1));
      sumDIXTmp32 =
        _mm256_add_epi32(sumDIXTmp32, _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(dIX + widthG_2)), shift2));
      sumDIXTmp32 =
        _mm256_add_epi32(sumDIXTmp32, _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(dIX + widthG_3)), shift1));
      sumDIXTmp32 = _mm256_add_epi32(sumDIXTmp32, _mm256_loadu_si256((const __m256i *)(dIX + widthG_4)));

      dummy = _mm256_mullo_epi32(sumDIXTmp32, vmask[0]);
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
      sumDIX[sampleIdx] =
        _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
      dummy = _mm256_mullo_epi32(sumDIXTmp32, vmask[1]);
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
      sumDIX[sampleIdx + 1] =
        _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
      dummy = _mm256_mullo_epi32(sumDIXTmp32, vmask[2]);
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
      sumDIX[sampleIdx + 2] =
        _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
      dummy = _mm256_mullo_epi32(sumDIXTmp32, vmask[3]);
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
      sumDIX[sampleIdx + 3] =
        _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));

      sumDIYTmp32 = _mm256_add_epi32(_mm256_loadu_si256((const __m256i *)dIY),
                                     _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(dIY + widthG)), shift1));
      sumDIYTmp32 =
        _mm256_add_epi32(sumDIYTmp32, _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(dIY + widthG_2)), shift2));
      sumDIYTmp32 =
        _mm256_add_epi32(sumDIYTmp32, _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(dIY + widthG_3)), shift1));
      sumDIYTmp32 = _mm256_add_epi32(sumDIYTmp32, _mm256_loadu_si256((const __m256i *)(dIY + widthG_4)));

      dummy = _mm256_mullo_epi32(sumDIYTmp32, vmask[0]);
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
      sumDIY[sampleIdx] =
        _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
      dummy = _mm256_mullo_epi32(sumDIYTmp32, vmask[1]);
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
      sumDIY[sampleIdx + 1] =
        _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
      dummy = _mm256_mullo_epi32(sumDIYTmp32, vmask[2]);
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
      sumDIY[sampleIdx + 2] =
        _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
      dummy = _mm256_mullo_epi32(sumDIYTmp32, vmask[3]);
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
      dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
      sumDIY[sampleIdx + 3] =
        _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));

      sumX0 = _mm_add_epi16(_mm_loadu_si128((const __m128i *)(gX)),
                            _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(gX + widthG)), shift1));
      sumX0 = _mm_add_epi16(sumX0, _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(gX + widthG_2)), shift2));
      sumX0 = _mm_add_epi16(sumX0, _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(gX + widthG_3)), shift1));
      sumX0 = _mm_add_epi16(sumX0, _mm_loadu_si128((const __m128i *)(gX + widthG_4)));

      dummy2 = _mm_madd_epi16(sumX0, vmask2[0]);
      dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
      dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
      sX0[0] = _mm_cvtsi128_si32(dummy2);
      dummy2 = _mm_madd_epi16(sumX0, vmask2[1]);
      dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
      dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
      sX0[1] = _mm_cvtsi128_si32(dummy2);
      dummy2 = _mm_madd_epi16(sumX0, vmask2[2]);
      dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
      dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
      sX0[2] = _mm_cvtsi128_si32(dummy2);
      dummy2 = _mm_madd_epi16(sumX0, vmask2[3]);
      dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
      dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
      sX0[3] = _mm_cvtsi128_si32(dummy2);

      sumX1 = _mm_add_epi16(_mm_loadu_si128((const __m128i *)(gY)),
                            _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(gY + widthG)), shift1));
      sumX1 = _mm_add_epi16(sumX1, _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(gY + widthG_2)), shift2));
      sumX1 = _mm_add_epi16(sumX1, _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(gY + widthG_3)), shift1));
      sumX1 = _mm_add_epi16(sumX1, _mm_loadu_si128((const __m128i *)(gY + widthG_4)));

      dummy2 = _mm_madd_epi16(sumX1, vmask2[0]);
      dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
      dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
      sX1[0] = _mm_cvtsi128_si32(dummy2);
      dummy2 = _mm_madd_epi16(sumX1, vmask2[1]);
      dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
      dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
      sX1[1] = _mm_cvtsi128_si32(dummy2);
      dummy2 = _mm_madd_epi16(sumX1, vmask2[2]);
      dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
      dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
      sX1[2] = _mm_cvtsi128_si32(dummy2);
      dummy2 = _mm_madd_epi16(sumX1, vmask2[3]);
      dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
      dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
      sX1[3] = _mm_cvtsi128_si32(dummy2);

      sumMean =
        _mm_add_epi16(_mm_loadu_si128((const __m128i *)(dI)), (_mm_loadu_si128((const __m128i *)(dI + widthG))));
      sumMean = _mm_add_epi16(sumMean, (_mm_loadu_si128((const __m128i *)(dI + widthG_2))));
      sumMean = _mm_add_epi16(sumMean, (_mm_loadu_si128((const __m128i *)(dI + widthG_3))));
      sumMean = _mm_add_epi16(sumMean, (_mm_loadu_si128((const __m128i *)(dI + widthG_4))));

      dummy2   = _mm_madd_epi16(sumMean, fOne[0]);
      dummy2   = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
      dummy2   = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
      mean4[0] = _mm_cvtsi128_si32(dummy2);
      dummy2   = _mm_madd_epi16(sumMean, fOne[1]);
      dummy2   = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
      dummy2   = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
      mean4[1] = _mm_cvtsi128_si32(dummy2);
      dummy2   = _mm_madd_epi16(sumMean, fOne[2]);
      dummy2   = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
      dummy2   = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
      mean4[2] = _mm_cvtsi128_si32(dummy2);
      dummy2   = _mm_madd_epi16(sumMean, fOne[3]);
      dummy2   = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
      dummy2   = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
      mean4[3] = _mm_cvtsi128_si32(dummy2);

      sumAbsMean = _mm_add_epi16(_mm_abs_epi16(_mm_loadu_si128((const __m128i *)(dI))),
                                 _mm_abs_epi16(_mm_loadu_si128((const __m128i *)(dI + widthG))));
      sumAbsMean = _mm_add_epi16(sumAbsMean, _mm_abs_epi16(_mm_loadu_si128((const __m128i *)(dI + widthG_2))));
      sumAbsMean = _mm_add_epi16(sumAbsMean, _mm_abs_epi16(_mm_loadu_si128((const __m128i *)(dI + widthG_3))));
      sumAbsMean = _mm_add_epi16(sumAbsMean, _mm_abs_epi16(_mm_loadu_si128((const __m128i *)(dI + widthG_4))));

      dummy2      = _mm_madd_epi16(sumAbsMean, fOne[0]);
      dummy2      = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
      dummy2      = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
      absMean4[0] = _mm_cvtsi128_si32(dummy2);
      dummy2      = _mm_madd_epi16(sumAbsMean, fOne[1]);
      dummy2      = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
      dummy2      = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
      absMean4[1] = _mm_cvtsi128_si32(dummy2);
      dummy2      = _mm_madd_epi16(sumAbsMean, fOne[2]);
      dummy2      = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
      dummy2      = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
      absMean4[2] = _mm_cvtsi128_si32(dummy2);
      dummy2      = _mm_madd_epi16(sumAbsMean, fOne[3]);
      dummy2      = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
      dummy2      = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
      absMean4[3] = _mm_cvtsi128_si32(dummy2);

      mean4[0] = (absMean4[0] > 2 * abs(mean4[0])) ? 0 : (mean4[0] + 32) >> 6;
      sumDIX[sampleIdx] -= sX0[0] * mean4[0];
      sumDIY[sampleIdx] -= sX1[0] * mean4[0];
      mean4[1] = (absMean4[1] > 2 * abs(mean4[1])) ? 0 : (mean4[1] + 32) >> 6;
      sumDIX[sampleIdx + 1] -= sX0[1] * mean4[1];
      sumDIY[sampleIdx + 1] -= sX1[1] * mean4[1];
      mean4[2] = (absMean4[2] > 2 * abs(mean4[2])) ? 0 : (mean4[2] + 32) >> 6;
      sumDIX[sampleIdx + 2] -= sX0[2] * mean4[2];
      sumDIY[sampleIdx + 2] -= sX1[2] * mean4[2];
      mean4[3] = (absMean4[3] > 2 * abs(mean4[3])) ? 0 : (mean4[3] + 32) >> 6;
      sumDIX[sampleIdx + 3] -= sX0[3] * mean4[3];
      sumDIY[sampleIdx + 3] -= sX1[3] * mean4[3];

      sumDIX[sampleIdx] += (sumDIX[sampleIdx] + 2) >> 2;
      sumDIY[sampleIdx] += (sumDIY[sampleIdx] + 2) >> 2;
      sumDIX[sampleIdx + 1] += (sumDIX[sampleIdx + 1] + 2) >> 2;
      sumDIY[sampleIdx + 1] += (sumDIY[sampleIdx + 1] + 2) >> 2;
      sumDIX[sampleIdx + 2] += (sumDIX[sampleIdx + 2] + 2) >> 2;
      sumDIY[sampleIdx + 2] += (sumDIY[sampleIdx + 2] + 2) >> 2;
      sumDIX[sampleIdx + 3] += (sumDIX[sampleIdx + 3] + 2) >> 2;
      sumDIY[sampleIdx + 3] += (sumDIY[sampleIdx + 3] + 2) >> 2;

      sumAbsGX[sampleIdx] += regVxVy;
      sumAbsGY[sampleIdx] += regVxVy;
      sumAbsGX[sampleIdx + 1] += regVxVy;
      sumAbsGY[sampleIdx + 1] += regVxVy;
      sumAbsGX[sampleIdx + 2] += regVxVy;
      sumAbsGY[sampleIdx + 2] += regVxVy;
      sumAbsGX[sampleIdx + 3] += regVxVy;
      sumAbsGY[sampleIdx + 3] += regVxVy;

      absGX += (widthG);
      absGY += (widthG);
      dIX += (widthG);
      dIY += (widthG);
      signGyGx += (widthG);
      gX += (widthG);
      gY += (widthG);
      dI += (widthG);
    }
  }
  else
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x = x + 4)
      {
        const int sampleIdx = y * width + x;
        sumAbsGXTmp32 =
          _mm256_add_epi32(_mm256_loadu_si256((const __m256i *)absGX),
                           _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(absGX + widthG)), shift1));
        sumAbsGXTmp32 = _mm256_add_epi32(
          sumAbsGXTmp32, _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(absGX + widthG_2)), shift2));
        sumAbsGXTmp32 = _mm256_add_epi32(
          sumAbsGXTmp32, _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(absGX + widthG_3)), shift1));
        sumAbsGXTmp32 = _mm256_add_epi32(sumAbsGXTmp32, _mm256_loadu_si256((const __m256i *)(absGX + widthG_4)));

        dummy = _mm256_mullo_epi32(sumAbsGXTmp32, vmask[0]);
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
        sumAbsGX[sampleIdx] =
          _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
        dummy = _mm256_mullo_epi32(sumAbsGXTmp32, vmask[1]);
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
        sumAbsGX[sampleIdx + 1] =
          _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
        dummy = _mm256_mullo_epi32(sumAbsGXTmp32, vmask[2]);
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
        sumAbsGX[sampleIdx + 2] =
          _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
        dummy = _mm256_mullo_epi32(sumAbsGXTmp32, vmask[3]);
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
        sumAbsGX[sampleIdx + 3] =
          _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));

        sumAbsGYTmp32 =
          _mm256_add_epi32(_mm256_loadu_si256((const __m256i *)absGY),
                           _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(absGY + widthG)), shift1));
        sumAbsGYTmp32 = _mm256_add_epi32(
          sumAbsGYTmp32, _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(absGY + widthG_2)), shift2));
        sumAbsGYTmp32 = _mm256_add_epi32(
          sumAbsGYTmp32, _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(absGY + widthG_3)), shift1));
        sumAbsGYTmp32 = _mm256_add_epi32(sumAbsGYTmp32, _mm256_loadu_si256((const __m256i *)(absGY + widthG_4)));

        dummy = _mm256_mullo_epi32(sumAbsGYTmp32, vmask[0]);
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
        sumAbsGY[sampleIdx] =
          _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
        dummy = _mm256_mullo_epi32(sumAbsGYTmp32, vmask[1]);
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
        sumAbsGY[sampleIdx + 1] =
          _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
        dummy = _mm256_mullo_epi32(sumAbsGYTmp32, vmask[2]);
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
        sumAbsGY[sampleIdx + 2] =
          _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
        dummy = _mm256_mullo_epi32(sumAbsGYTmp32, vmask[3]);
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
        sumAbsGY[sampleIdx + 3] =
          _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));

        sumSignGyGxTmp32 =
          _mm256_add_epi32(_mm256_loadu_si256((const __m256i *)signGyGx),
                           _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(signGyGx + widthG)), shift1));
        sumSignGyGxTmp32 = _mm256_add_epi32(
          sumSignGyGxTmp32, _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(signGyGx + widthG_2)), shift2));
        sumSignGyGxTmp32 = _mm256_add_epi32(
          sumSignGyGxTmp32, _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(signGyGx + widthG_3)), shift1));
        sumSignGyGxTmp32 =
          _mm256_add_epi32(sumSignGyGxTmp32, _mm256_loadu_si256((const __m256i *)(signGyGx + widthG_4)));

        dummy = _mm256_mullo_epi32(sumSignGyGxTmp32, vmask[0]);
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
        sumSignGyGx[sampleIdx] =
          _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
        dummy = _mm256_mullo_epi32(sumSignGyGxTmp32, vmask[1]);
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
        sumSignGyGx[sampleIdx + 1] =
          _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
        dummy = _mm256_mullo_epi32(sumSignGyGxTmp32, vmask[2]);
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
        sumSignGyGx[sampleIdx + 2] =
          _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
        dummy = _mm256_mullo_epi32(sumSignGyGxTmp32, vmask[3]);
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
        sumSignGyGx[sampleIdx + 3] =
          _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));

        sumDIXTmp32 = _mm256_add_epi32(_mm256_loadu_si256((const __m256i *)dIX),
                                       _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(dIX + widthG)), shift1));
        sumDIXTmp32 = _mm256_add_epi32(
          sumDIXTmp32, _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(dIX + widthG_2)), shift2));
        sumDIXTmp32 = _mm256_add_epi32(
          sumDIXTmp32, _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(dIX + widthG_3)), shift1));
        sumDIXTmp32 = _mm256_add_epi32(sumDIXTmp32, _mm256_loadu_si256((const __m256i *)(dIX + widthG_4)));

        dummy = _mm256_mullo_epi32(sumDIXTmp32, vmask[0]);
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
        sumDIX[sampleIdx] =
          _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
        dummy = _mm256_mullo_epi32(sumDIXTmp32, vmask[1]);
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
        sumDIX[sampleIdx + 1] =
          _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
        dummy = _mm256_mullo_epi32(sumDIXTmp32, vmask[2]);
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
        sumDIX[sampleIdx + 2] =
          _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
        dummy = _mm256_mullo_epi32(sumDIXTmp32, vmask[3]);
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
        sumDIX[sampleIdx + 3] =
          _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));

        sumDIYTmp32 = _mm256_add_epi32(_mm256_loadu_si256((const __m256i *)dIY),
                                       _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(dIY + widthG)), shift1));
        sumDIYTmp32 = _mm256_add_epi32(
          sumDIYTmp32, _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(dIY + widthG_2)), shift2));
        sumDIYTmp32 = _mm256_add_epi32(
          sumDIYTmp32, _mm256_slli_epi32(_mm256_loadu_si256((const __m256i *)(dIY + widthG_3)), shift1));
        sumDIYTmp32 = _mm256_add_epi32(sumDIYTmp32, _mm256_loadu_si256((const __m256i *)(dIY + widthG_4)));

        dummy = _mm256_mullo_epi32(sumDIYTmp32, vmask[0]);
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
        sumDIY[sampleIdx] =
          _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
        dummy = _mm256_mullo_epi32(sumDIYTmp32, vmask[1]);
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
        sumDIY[sampleIdx + 1] =
          _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
        dummy = _mm256_mullo_epi32(sumDIYTmp32, vmask[2]);
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
        sumDIY[sampleIdx + 2] =
          _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));
        dummy = _mm256_mullo_epi32(sumDIYTmp32, vmask[3]);
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b01001110));
        dummy = _mm256_add_epi32(dummy, _mm256_shuffle_epi32(dummy, 0b10110001));
        sumDIY[sampleIdx + 3] =
          _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 1)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(dummy, 0));

        sumX0 = _mm_add_epi16(_mm_loadu_si128((const __m128i *)(gX)),
                              _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(gX + widthG)), shift1));
        sumX0 = _mm_add_epi16(sumX0, _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(gX + widthG_2)), shift2));
        sumX0 = _mm_add_epi16(sumX0, _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(gX + widthG_3)), shift1));
        sumX0 = _mm_add_epi16(sumX0, _mm_loadu_si128((const __m128i *)(gX + widthG_4)));

        dummy2 = _mm_madd_epi16(sumX0, vmask2[0]);
        dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
        dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
        sX0[0] = _mm_cvtsi128_si32(dummy2);
        dummy2 = _mm_madd_epi16(sumX0, vmask2[1]);
        dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
        dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
        sX0[1] = _mm_cvtsi128_si32(dummy2);
        dummy2 = _mm_madd_epi16(sumX0, vmask2[2]);
        dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
        dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
        sX0[2] = _mm_cvtsi128_si32(dummy2);
        dummy2 = _mm_madd_epi16(sumX0, vmask2[3]);
        dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
        dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
        sX0[3] = _mm_cvtsi128_si32(dummy2);

        sumX1 = _mm_add_epi16(_mm_loadu_si128((const __m128i *)(gY)),
                              _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(gY + widthG)), shift1));
        sumX1 = _mm_add_epi16(sumX1, _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(gY + widthG_2)), shift2));
        sumX1 = _mm_add_epi16(sumX1, _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(gY + widthG_3)), shift1));
        sumX1 = _mm_add_epi16(sumX1, _mm_loadu_si128((const __m128i *)(gY + widthG_4)));

        dummy2 = _mm_madd_epi16(sumX1, vmask2[0]);
        dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
        dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
        sX1[0] = _mm_cvtsi128_si32(dummy2);
        dummy2 = _mm_madd_epi16(sumX1, vmask2[1]);
        dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
        dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
        sX1[1] = _mm_cvtsi128_si32(dummy2);
        dummy2 = _mm_madd_epi16(sumX1, vmask2[2]);
        dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
        dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
        sX1[2] = _mm_cvtsi128_si32(dummy2);
        dummy2 = _mm_madd_epi16(sumX1, vmask2[3]);
        dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
        dummy2 = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
        sX1[3] = _mm_cvtsi128_si32(dummy2);

        sumMean =
          _mm_add_epi16(_mm_loadu_si128((const __m128i *)(dI)), (_mm_loadu_si128((const __m128i *)(dI + widthG))));
        sumMean = _mm_add_epi16(sumMean, (_mm_loadu_si128((const __m128i *)(dI + widthG_2))));
        sumMean = _mm_add_epi16(sumMean, (_mm_loadu_si128((const __m128i *)(dI + widthG_3))));
        sumMean = _mm_add_epi16(sumMean, (_mm_loadu_si128((const __m128i *)(dI + widthG_4))));

        dummy2   = _mm_madd_epi16(sumMean, fOne[0]);
        dummy2   = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
        dummy2   = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
        mean4[0] = _mm_cvtsi128_si32(dummy2);
        dummy2   = _mm_madd_epi16(sumMean, fOne[1]);
        dummy2   = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
        dummy2   = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
        mean4[1] = _mm_cvtsi128_si32(dummy2);
        dummy2   = _mm_madd_epi16(sumMean, fOne[2]);
        dummy2   = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
        dummy2   = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
        mean4[2] = _mm_cvtsi128_si32(dummy2);
        dummy2   = _mm_madd_epi16(sumMean, fOne[3]);
        dummy2   = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
        dummy2   = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
        mean4[3] = _mm_cvtsi128_si32(dummy2);

        sumAbsMean = _mm_add_epi16(_mm_abs_epi16(_mm_loadu_si128((const __m128i *)(dI))),
                                   _mm_abs_epi16(_mm_loadu_si128((const __m128i *)(dI + widthG))));
        sumAbsMean = _mm_add_epi16(sumAbsMean, _mm_abs_epi16(_mm_loadu_si128((const __m128i *)(dI + widthG_2))));
        sumAbsMean = _mm_add_epi16(sumAbsMean, _mm_abs_epi16(_mm_loadu_si128((const __m128i *)(dI + widthG_3))));
        sumAbsMean = _mm_add_epi16(sumAbsMean, _mm_abs_epi16(_mm_loadu_si128((const __m128i *)(dI + widthG_4))));

        dummy2      = _mm_madd_epi16(sumAbsMean, fOne[0]);
        dummy2      = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
        dummy2      = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
        absMean4[0] = _mm_cvtsi128_si32(dummy2);
        dummy2      = _mm_madd_epi16(sumAbsMean, fOne[1]);
        dummy2      = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
        dummy2      = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
        absMean4[1] = _mm_cvtsi128_si32(dummy2);
        dummy2      = _mm_madd_epi16(sumAbsMean, fOne[2]);
        dummy2      = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
        dummy2      = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
        absMean4[2] = _mm_cvtsi128_si32(dummy2);
        dummy2      = _mm_madd_epi16(sumAbsMean, fOne[3]);
        dummy2      = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0x4e));   // 01001110
        dummy2      = _mm_add_epi32(dummy2, _mm_shuffle_epi32(dummy2, 0xb1));   // 10110001
        absMean4[3] = _mm_cvtsi128_si32(dummy2);

        mean4[0] = (absMean4[0] > 2 * abs(mean4[0])) ? 0 : (mean4[0] + 32) >> 6;
        sumDIX[sampleIdx] -= sX0[0] * mean4[0];
        sumDIY[sampleIdx] -= sX1[0] * mean4[0];
        mean4[1] = (absMean4[1] > 2 * abs(mean4[1])) ? 0 : (mean4[1] + 32) >> 6;
        sumDIX[sampleIdx + 1] -= sX0[1] * mean4[1];
        sumDIY[sampleIdx + 1] -= sX1[1] * mean4[1];
        mean4[2] = (absMean4[2] > 2 * abs(mean4[2])) ? 0 : (mean4[2] + 32) >> 6;
        sumDIX[sampleIdx + 2] -= sX0[2] * mean4[2];
        sumDIY[sampleIdx + 2] -= sX1[2] * mean4[2];
        mean4[3] = (absMean4[3] > 2 * abs(mean4[3])) ? 0 : (mean4[3] + 32) >> 6;
        sumDIX[sampleIdx + 3] -= sX0[3] * mean4[3];
        sumDIY[sampleIdx + 3] -= sX1[3] * mean4[3];

        sumDIX[sampleIdx] += (sumDIX[sampleIdx] + 2) >> 2;
        sumDIY[sampleIdx] += (sumDIY[sampleIdx] + 2) >> 2;
        sumDIX[sampleIdx + 1] += (sumDIX[sampleIdx + 1] + 2) >> 2;
        sumDIY[sampleIdx + 1] += (sumDIY[sampleIdx + 1] + 2) >> 2;
        sumDIX[sampleIdx + 2] += (sumDIX[sampleIdx + 2] + 2) >> 2;
        sumDIY[sampleIdx + 2] += (sumDIY[sampleIdx + 2] + 2) >> 2;
        sumDIX[sampleIdx + 3] += (sumDIX[sampleIdx + 3] + 2) >> 2;
        sumDIY[sampleIdx + 3] += (sumDIY[sampleIdx + 3] + 2) >> 2;

        sumAbsGX[sampleIdx] += regVxVy;
        sumAbsGY[sampleIdx] += regVxVy;
        sumAbsGX[sampleIdx + 1] += regVxVy;
        sumAbsGY[sampleIdx + 1] += regVxVy;
        sumAbsGX[sampleIdx + 2] += regVxVy;
        sumAbsGY[sampleIdx + 2] += regVxVy;
        sumAbsGX[sampleIdx + 3] += regVxVy;
        sumAbsGY[sampleIdx + 3] += regVxVy;
        absGX += 4;
        absGY += 4;
        dIX += 4;
        dIY += 4;
        signGyGx += 4;
        dI += 4;
        gX += 4;
        gY += 4;
      }
      absGX += (widthG - width);
      absGY += (widthG - width);
      dIX += (widthG - width);
      dIY += (widthG - width);
      signGyGx += (widthG - width);
      gX += (widthG - width);
      gY += (widthG - width);
      dI += (widthG - width);
    }
  }
  return;
#endif
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      const int sampleIdx    = y * width + x;
      sumAbsGX[sampleIdx]    = 0;
      sumAbsGY[sampleIdx]    = 0;
      sumDIX[sampleIdx]      = 0;
      sumDIY[sampleIdx]      = 0;
      sumSignGyGx[sampleIdx] = 0;
      int meanDiff           = 0;
      int absmeanDiff        = 0;
      int X0 = 0, X1 = 0;
      int w = 1, a = 1, b = 2, c = 4, d = 4, e = 8, f = 16;
      int weight[5][5] = { { a, b, c, b, a },
                           { b, d, e, d, b },
                           { c, e, f, e, c },
                           { b, d, e, d, b },
                           { a, b, c, b, a } };
      int regVxVy =
        2528;   // = ((1 << 11) * 100)/81. 100 is summation of the new weights, 81 was the summation of the old weights
      for (int yy = 0; yy < 5; yy++)
      {
        for (int xx = 0; xx < 5; xx++)
        {
          w = weight[yy][xx];
          sumAbsGX[sampleIdx] += w * absGX[xx];
          sumAbsGY[sampleIdx] += w * absGY[xx];
          sumDIX[sampleIdx] += w * dIX[xx];
          sumDIY[sampleIdx] += w * dIY[xx];
          meanDiff += dI[xx];
          absmeanDiff += abs(dI[xx]);
          X0 -= w * gX[xx];
          X1 -= w * gY[xx];
          sumSignGyGx[sampleIdx] += w * signGyGx[xx];
        }
        absGX += widthG;
        absGY += widthG;
        dIX += widthG;
        dIY += widthG;
        signGyGx += widthG;
        dI += widthG;
        gX += widthG;
        gY += widthG;
      }
      meanDiff = (absmeanDiff > 2 * abs(meanDiff)) ? 0 : (meanDiff + 32) >> 6;
      sumDIX[sampleIdx] += X0 * meanDiff;
      sumDIY[sampleIdx] += X1 * meanDiff;
      sumDIX[sampleIdx] += (sumDIX[sampleIdx] + 2) >> 2;
      sumDIY[sampleIdx] += (sumDIY[sampleIdx] + 2) >> 2;
      sumAbsGX[sampleIdx] += regVxVy;
      sumAbsGY[sampleIdx] += regVxVy;
      absGX += (1 - 5 * widthG);
      absGY += (1 - 5 * widthG);
      dIX += (1 - 5 * widthG);
      dIY += (1 - 5 * widthG);
      signGyGx += (1 - 5 * widthG);
      dI += (1 - 5 * widthG);
      gX += (1 - 5 * widthG);
      gY += (1 - 5 * widthG);
    }
    absGX += (widthG - width);
    absGY += (widthG - width);
    dIX += (widthG - width);
    dIY += (widthG - width);
    signGyGx += (widthG - width);
    gX += (widthG - width);
    gY += (widthG - width);
    dI += (widthG - width);
  }
}

template<X86_VEXT vext> void calcBIOParamSum5_SSE(Pel *absGX, Pel *absGY, Pel *dIX, Pel *dIY, Pel *signGyGx,
                                                  const int widthG, const int width, const int height, int *sumAbsGX,
                                                  int *sumAbsGY, int *sumDIX, int *sumDIY, int *sumSignGyGx)
{
  __m128i vzero    = _mm_setzero_si128();
  __m128i vmask[4] = { _mm_setr_epi16(1, 2, 3, 2, 1, 0, 0, 0), _mm_setr_epi16(0, 1, 2, 3, 2, 1, 0, 0),
                       _mm_setr_epi16(0, 0, 1, 2, 3, 2, 1, 0), _mm_setr_epi16(0, 0, 0, 1, 2, 3, 2, 1) };

  __m128i sumAbsGXTmp16    = vzero;
  __m128i sumDIXTmp16      = vzero;
  __m128i sumAbsGYTmp16    = vzero;
  __m128i sumDIYTmp16      = vzero;
  __m128i sumSignGyGxTmp16 = vzero;
  __m128i sumAbsGXTmp32    = vzero;
  __m128i sumDIXTmp32      = vzero;
  __m128i sumAbsGYTmp32    = vzero;
  __m128i sumDIYTmp32      = vzero;
  __m128i sumSignGyGxTmp32 = vzero;
  __m128i a12              = vzero;
  __m128i a3               = vzero;
  __m128i b12              = vzero;
  __m128i b3               = vzero;
  __m128i c1               = vzero;

  const int width_height  = (width * height);
  const int width_N       = (4 - width_height);
  const int widthG_height = (widthG * height);
  const int widthG_2      = (widthG << 1);
  const int widthG_3      = widthG_2 + widthG;
  const int widthG_4      = widthG_3 + widthG;
  const int widthG_N      = (4 - widthG_height);

  for (int x = 0; x < width; x += 4)
  {
    sumAbsGXTmp16 = _mm_add_epi16(_mm_loadu_si128((const __m128i *)absGX),
                                  _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(absGX + widthG)), 1));
    sumAbsGXTmp16 = _mm_add_epi16(sumAbsGXTmp16, _mm_loadu_si128((const __m128i *)(absGX + widthG_2)));
    sumAbsGXTmp16 =
      _mm_add_epi16(sumAbsGXTmp16, _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(absGX + widthG_2)), 1));   //
    sumAbsGXTmp16 =
      _mm_add_epi16(sumAbsGXTmp16, _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(absGX + widthG_3)), 1));

    sumDIXTmp16 = _mm_add_epi16(_mm_loadu_si128((const __m128i *)dIX),
                                _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(dIX + widthG)), 1));
    sumDIXTmp16 = _mm_add_epi16(sumDIXTmp16, _mm_loadu_si128((const __m128i *)(dIX + widthG_2)));
    sumDIXTmp16 =
      _mm_add_epi16(sumDIXTmp16, _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(dIX + widthG_2)), 1));   //
    sumDIXTmp16 = _mm_add_epi16(sumDIXTmp16, _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(dIX + widthG_3)), 1));

    sumAbsGYTmp16 = _mm_add_epi16(_mm_loadu_si128((const __m128i *)absGY),
                                  _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(absGY + widthG)), 1));
    sumAbsGYTmp16 = _mm_add_epi16(sumAbsGYTmp16, _mm_loadu_si128((const __m128i *)(absGY + widthG_2)));
    sumAbsGYTmp16 =
      _mm_add_epi16(sumAbsGYTmp16, _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(absGY + widthG_2)), 1));   //
    sumAbsGYTmp16 =
      _mm_add_epi16(sumAbsGYTmp16, _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(absGY + widthG_3)), 1));

    sumDIYTmp16 = _mm_add_epi16(_mm_loadu_si128((const __m128i *)dIY),
                                _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(dIY + widthG)), 1));
    sumDIYTmp16 = _mm_add_epi16(sumDIYTmp16, _mm_loadu_si128((const __m128i *)(dIY + widthG_2)));
    sumDIYTmp16 =
      _mm_add_epi16(sumDIYTmp16, _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(dIY + widthG_2)), 1));   //
    sumDIYTmp16 = _mm_add_epi16(sumDIYTmp16, _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(dIY + widthG_3)), 1));

    sumSignGyGxTmp16 = _mm_add_epi16(_mm_loadu_si128((const __m128i *)signGyGx),
                                     _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(signGyGx + widthG)), 1));
    sumSignGyGxTmp16 = _mm_add_epi16(sumSignGyGxTmp16, _mm_loadu_si128((const __m128i *)(signGyGx + widthG_2)));
    sumSignGyGxTmp16 =
      _mm_add_epi16(sumSignGyGxTmp16, _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(signGyGx + widthG_2)), 1));   //
    sumSignGyGxTmp16 =
      _mm_add_epi16(sumSignGyGxTmp16, _mm_slli_epi16(_mm_loadu_si128((const __m128i *)(signGyGx + widthG_3)), 1));

    __m128i absGXOneRow    = vzero;
    __m128i dIXOneRow      = vzero;
    __m128i absGYOneRow    = vzero;
    __m128i dIYOneRow      = vzero;
    __m128i signGyGxOneRow = vzero;
    for (int y = 0; y < height; y++)
    {
      sumAbsGXTmp16 = _mm_sub_epi16(sumAbsGXTmp16, absGXOneRow);

      absGXOneRow =
        _mm_add_epi16(_mm_loadu_si128((const __m128i *)absGX), _mm_loadu_si128((const __m128i *)(absGX + widthG)));   //
      absGXOneRow = _mm_add_epi16(absGXOneRow, _mm_loadu_si128((const __m128i *)(absGX + widthG_2)));   //
      absGXOneRow = _mm_sub_epi16(absGXOneRow, _mm_loadu_si128((const __m128i *)(absGX + widthG_3)));   //
      absGXOneRow = _mm_sub_epi16(absGXOneRow, _mm_loadu_si128((const __m128i *)(absGX + widthG_4)));   //

      sumAbsGXTmp16 = _mm_add_epi16(sumAbsGXTmp16, _mm_loadu_si128((const __m128i *)(absGX + widthG_4)));

      sumDIXTmp16 = _mm_sub_epi16(sumDIXTmp16, dIXOneRow);

      dIXOneRow =
        _mm_add_epi16(_mm_loadu_si128((const __m128i *)dIX), _mm_loadu_si128((const __m128i *)(dIX + widthG)));   //
      dIXOneRow = _mm_add_epi16(dIXOneRow, _mm_loadu_si128((const __m128i *)(dIX + widthG_2)));   //
      dIXOneRow = _mm_sub_epi16(dIXOneRow, _mm_loadu_si128((const __m128i *)(dIX + widthG_3)));   //
      dIXOneRow = _mm_sub_epi16(dIXOneRow, _mm_loadu_si128((const __m128i *)(dIX + widthG_4)));   //

      sumDIXTmp16 = _mm_add_epi16(sumDIXTmp16, _mm_loadu_si128((const __m128i *)(dIX + widthG_4)));

      sumAbsGYTmp16 = _mm_sub_epi16(sumAbsGYTmp16, absGYOneRow);

      absGYOneRow =
        _mm_add_epi16(_mm_loadu_si128((const __m128i *)absGY), _mm_loadu_si128((const __m128i *)(absGY + widthG)));   //
      absGYOneRow = _mm_add_epi16(absGYOneRow, _mm_loadu_si128((const __m128i *)(absGY + widthG_2)));   //
      absGYOneRow = _mm_sub_epi16(absGYOneRow, _mm_loadu_si128((const __m128i *)(absGY + widthG_3)));   //
      absGYOneRow = _mm_sub_epi16(absGYOneRow, _mm_loadu_si128((const __m128i *)(absGY + widthG_4)));   //

      sumAbsGYTmp16 = _mm_add_epi16(sumAbsGYTmp16, _mm_loadu_si128((const __m128i *)(absGY + widthG_4)));

      sumDIYTmp16 = _mm_sub_epi16(sumDIYTmp16, dIYOneRow);

      dIYOneRow =
        _mm_add_epi16(_mm_loadu_si128((const __m128i *)dIY), _mm_loadu_si128((const __m128i *)(dIY + widthG)));   //
      dIYOneRow = _mm_add_epi16(dIYOneRow, _mm_loadu_si128((const __m128i *)(dIY + widthG_2)));   //
      dIYOneRow = _mm_sub_epi16(dIYOneRow, _mm_loadu_si128((const __m128i *)(dIY + widthG_3)));   //
      dIYOneRow = _mm_sub_epi16(dIYOneRow, _mm_loadu_si128((const __m128i *)(dIY + widthG_4)));   //

      sumDIYTmp16 = _mm_add_epi16(sumDIYTmp16, _mm_loadu_si128((const __m128i *)(dIY + widthG_4)));

      sumSignGyGxTmp16 = _mm_sub_epi16(sumSignGyGxTmp16, signGyGxOneRow);

      signGyGxOneRow = _mm_add_epi16(_mm_loadu_si128((const __m128i *)signGyGx),
                                     _mm_loadu_si128((const __m128i *)(signGyGx + widthG)));   //
      signGyGxOneRow = _mm_add_epi16(signGyGxOneRow, _mm_loadu_si128((const __m128i *)(signGyGx + widthG_2)));   //
      signGyGxOneRow = _mm_sub_epi16(signGyGxOneRow, _mm_loadu_si128((const __m128i *)(signGyGx + widthG_3)));   //
      signGyGxOneRow = _mm_sub_epi16(signGyGxOneRow, _mm_loadu_si128((const __m128i *)(signGyGx + widthG_4)));   //

      sumSignGyGxTmp16 = _mm_add_epi16(sumSignGyGxTmp16, _mm_loadu_si128((const __m128i *)(signGyGx + widthG_4)));

      sumAbsGXTmp32 = _mm_madd_epi16(sumAbsGXTmp16, vmask[0]);
      sumAbsGYTmp32 = _mm_madd_epi16(sumAbsGYTmp16, vmask[0]);
      sumDIXTmp32   = _mm_madd_epi16(sumDIXTmp16, vmask[0]);
      sumDIYTmp32   = _mm_madd_epi16(sumDIYTmp16, vmask[0]);
      a12           = _mm_unpacklo_epi32(sumAbsGXTmp32, sumAbsGYTmp32);
      a3            = _mm_unpackhi_epi32(sumAbsGXTmp32, sumAbsGYTmp32);
      b12           = _mm_unpacklo_epi32(sumDIXTmp32, sumDIYTmp32);
      b3            = _mm_unpackhi_epi32(sumDIXTmp32, sumDIYTmp32);

      c1 = _mm_unpacklo_epi64(a12, b12);
      c1 = _mm_add_epi32(c1, _mm_unpackhi_epi64(a12, b12));
      c1 = _mm_add_epi32(c1, _mm_unpacklo_epi64(a3, b3));

      int regVxVy     = (1 << 8);
      *(sumAbsGX + 0) = _mm_cvtsi128_si32(c1);
      *(sumAbsGY + 0) = _mm_cvtsi128_si32(_mm_shuffle_epi32(c1, 0x55));
      *(sumDIX + 0)   = _mm_cvtsi128_si32(_mm_shuffle_epi32(c1, 0xaa));
      *(sumDIY + 0)   = _mm_cvtsi128_si32(_mm_shuffle_epi32(c1, 0xff));

      sumSignGyGxTmp32   = _mm_madd_epi16(sumSignGyGxTmp16, vmask[0]);
      sumSignGyGxTmp32   = _mm_add_epi32(sumSignGyGxTmp32, _mm_shuffle_epi32(sumSignGyGxTmp32, 0x4e));   // 01001110
      sumSignGyGxTmp32   = _mm_add_epi32(sumSignGyGxTmp32, _mm_shuffle_epi32(sumSignGyGxTmp32, 0xb1));   // 10110001
      *(sumSignGyGx + 0) = _mm_cvtsi128_si32(sumSignGyGxTmp32);

      sumAbsGXTmp32 = _mm_madd_epi16(sumAbsGXTmp16, vmask[1]);
      sumAbsGYTmp32 = _mm_madd_epi16(sumAbsGYTmp16, vmask[1]);
      sumDIXTmp32   = _mm_madd_epi16(sumDIXTmp16, vmask[1]);
      sumDIYTmp32   = _mm_madd_epi16(sumDIYTmp16, vmask[1]);
      a12           = _mm_unpacklo_epi32(sumAbsGXTmp32, sumAbsGYTmp32);
      a3            = _mm_unpackhi_epi32(sumAbsGXTmp32, sumAbsGYTmp32);
      b12           = _mm_unpacklo_epi32(sumDIXTmp32, sumDIYTmp32);
      b3            = _mm_unpackhi_epi32(sumDIXTmp32, sumDIYTmp32);

      c1 = _mm_unpacklo_epi64(a12, b12);
      c1 = _mm_add_epi32(c1, _mm_unpackhi_epi64(a12, b12));
      c1 = _mm_add_epi32(c1, _mm_unpacklo_epi64(a3, b3));

      *(sumAbsGX + 1) = _mm_cvtsi128_si32(c1);
      *(sumAbsGY + 1) = _mm_cvtsi128_si32(_mm_shuffle_epi32(c1, 0x55));
      *(sumDIX + 1)   = _mm_cvtsi128_si32(_mm_shuffle_epi32(c1, 0xaa));
      *(sumDIY + 1)   = _mm_cvtsi128_si32(_mm_shuffle_epi32(c1, 0xff));

      sumSignGyGxTmp32   = _mm_madd_epi16(sumSignGyGxTmp16, vmask[1]);
      sumSignGyGxTmp32   = _mm_add_epi32(sumSignGyGxTmp32, _mm_shuffle_epi32(sumSignGyGxTmp32, 0x4e));   // 01001110
      sumSignGyGxTmp32   = _mm_add_epi32(sumSignGyGxTmp32, _mm_shuffle_epi32(sumSignGyGxTmp32, 0xb1));   // 10110001
      *(sumSignGyGx + 1) = _mm_cvtsi128_si32(sumSignGyGxTmp32);

      sumAbsGXTmp32 = _mm_madd_epi16(sumAbsGXTmp16, vmask[2]);
      sumAbsGYTmp32 = _mm_madd_epi16(sumAbsGYTmp16, vmask[2]);
      sumDIXTmp32   = _mm_madd_epi16(sumDIXTmp16, vmask[2]);
      sumDIYTmp32   = _mm_madd_epi16(sumDIYTmp16, vmask[2]);
      a12           = _mm_unpacklo_epi32(sumAbsGXTmp32, sumAbsGYTmp32);
      a3            = _mm_unpackhi_epi32(sumAbsGXTmp32, sumAbsGYTmp32);
      b12           = _mm_unpacklo_epi32(sumDIXTmp32, sumDIYTmp32);
      b3            = _mm_unpackhi_epi32(sumDIXTmp32, sumDIYTmp32);

      c1 = _mm_unpackhi_epi64(a12, b12);
      c1 = _mm_add_epi32(c1, _mm_unpacklo_epi64(a3, b3));
      c1 = _mm_add_epi32(c1, _mm_unpackhi_epi64(a3, b3));

      *(sumAbsGX + 2) = _mm_cvtsi128_si32(c1);
      *(sumAbsGY + 2) = _mm_cvtsi128_si32(_mm_shuffle_epi32(c1, 0x55));
      *(sumDIX + 2)   = _mm_cvtsi128_si32(_mm_shuffle_epi32(c1, 0xaa));
      *(sumDIY + 2)   = _mm_cvtsi128_si32(_mm_shuffle_epi32(c1, 0xff));

      sumSignGyGxTmp32   = _mm_madd_epi16(sumSignGyGxTmp16, vmask[2]);
      sumSignGyGxTmp32   = _mm_add_epi32(sumSignGyGxTmp32, _mm_shuffle_epi32(sumSignGyGxTmp32, 0x4e));   // 01001110
      sumSignGyGxTmp32   = _mm_add_epi32(sumSignGyGxTmp32, _mm_shuffle_epi32(sumSignGyGxTmp32, 0xb1));   // 10110001
      *(sumSignGyGx + 2) = _mm_cvtsi128_si32(sumSignGyGxTmp32);

      sumAbsGXTmp32 = _mm_madd_epi16(sumAbsGXTmp16, vmask[3]);
      sumAbsGYTmp32 = _mm_madd_epi16(sumAbsGYTmp16, vmask[3]);
      sumDIXTmp32   = _mm_madd_epi16(sumDIXTmp16, vmask[3]);
      sumDIYTmp32   = _mm_madd_epi16(sumDIYTmp16, vmask[3]);
      a12           = _mm_unpacklo_epi32(sumAbsGXTmp32, sumAbsGYTmp32);
      a3            = _mm_unpackhi_epi32(sumAbsGXTmp32, sumAbsGYTmp32);
      b12           = _mm_unpacklo_epi32(sumDIXTmp32, sumDIYTmp32);
      b3            = _mm_unpackhi_epi32(sumDIXTmp32, sumDIYTmp32);

      c1 = _mm_unpackhi_epi64(a12, b12);
      c1 = _mm_add_epi32(c1, _mm_unpacklo_epi64(a3, b3));
      c1 = _mm_add_epi32(c1, _mm_unpackhi_epi64(a3, b3));

      *(sumAbsGX + 3) = _mm_cvtsi128_si32(c1);
      *(sumAbsGY + 3) = _mm_cvtsi128_si32(_mm_shuffle_epi32(c1, 0x55));
      *(sumDIX + 3)   = _mm_cvtsi128_si32(_mm_shuffle_epi32(c1, 0xaa));
      *(sumDIY + 3)   = _mm_cvtsi128_si32(_mm_shuffle_epi32(c1, 0xff));
      *(sumAbsGX + 0) += regVxVy;
      *(sumAbsGY + 0) += regVxVy;
      *(sumAbsGX + 1) += regVxVy;
      *(sumAbsGY + 1) += regVxVy;
      *(sumAbsGX + 2) += regVxVy;
      *(sumAbsGY + 2) += regVxVy;
      *(sumAbsGX + 3) += regVxVy;
      *(sumAbsGY + 3) += regVxVy;

      sumSignGyGxTmp32   = _mm_madd_epi16(sumSignGyGxTmp16, vmask[3]);
      sumSignGyGxTmp32   = _mm_add_epi32(sumSignGyGxTmp32, _mm_shuffle_epi32(sumSignGyGxTmp32, 0x4e));   // 01001110
      sumSignGyGxTmp32   = _mm_add_epi32(sumSignGyGxTmp32, _mm_shuffle_epi32(sumSignGyGxTmp32, 0xb1));   // 10110001
      *(sumSignGyGx + 3) = _mm_cvtsi128_si32(sumSignGyGxTmp32);

      // bio parameter increment
      absGX += widthG;
      absGY += widthG;
      dIX += widthG;
      dIY += widthG;
      signGyGx += widthG;
      // sum parameter increment
      sumAbsGX += width;
      sumAbsGY += width;
      sumDIX += width;
      sumDIY += width;
      sumSignGyGx += width;
    }
    // bio parameter back to first row
    absGX += widthG_N;
    absGY += widthG_N;
    dIX += widthG_N;
    dIY += widthG_N;
    signGyGx += widthG_N;
    // sum parameter back to first row
    sumAbsGX += width_N;
    sumAbsGY += width_N;
    sumDIX += width_N;
    sumDIY += width_N;
    sumSignGyGx += width_N;
  }
  sumDIX -= width;
  sumDIY -= width;
#ifdef USE_AVX2
  for (int idx = 0; idx < width_height; idx += 8)
  {
    __m256i sumDIXTmp256 = _mm256_loadu_si256((const __m256i *)sumDIX);
    __m256i sumDIYTmp256 = _mm256_loadu_si256((const __m256i *)sumDIY);
    _mm256_storeu_si256((__m256i *)sumDIX, _mm256_slli_epi32(sumDIXTmp256, 2));
    _mm256_storeu_si256((__m256i *)sumDIY, _mm256_slli_epi32(sumDIYTmp256, 2));
    sumDIX += 8;
    sumDIY += 8;
  }
#else
  for (int idx = 0; idx < width_height; idx += 4)
  {
    sumDIXTmp32 = _mm_loadu_si128((const __m128i *)sumDIX);
    sumDIYTmp32 = _mm_loadu_si128((const __m128i *)sumDIY);
    _mm_storeu_si128((__m128i *)sumDIX, _mm_slli_epi32(sumDIXTmp32, 2));
    _mm_storeu_si128((__m128i *)sumDIY, _mm_slli_epi32(sumDIYTmp32, 2));
    sumDIX += 4;
    sumDIY += 4;
  }
#endif
}

template<X86_VEXT vext>
void calcBIOParameterHighPrecision_SSE(const Pel *srcY0Tmp, const Pel *srcY1Tmp, Pel *gradX0, Pel *gradX1, Pel *gradY0,
                                       Pel *gradY1, int width, int height, const int src0Stride, const int src1Stride,
                                       const int widthG, const int bitDepth, int32_t *s1, int32_t *s2, int32_t *s3,
                                       int32_t *s5, int32_t *s6, Pel *dI, Pel *gX, Pel *gY)
{
  width -= 2;
  height -= 2;
  const int bioParamOffset = widthG + 1;
  srcY0Tmp += src0Stride + 1;
  srcY1Tmp += src1Stride + 1;
  gradX0 += bioParamOffset;
  gradX1 += bioParamOffset;
  gradY0 += bioParamOffset;
  gradY1 += bioParamOffset;
  s1 += bioParamOffset;
  s2 += bioParamOffset;
  s3 += bioParamOffset;
  s5 += bioParamOffset;
  s6 += bioParamOffset;
  gX += bioParamOffset;
  gY += bioParamOffset;

  int shift4 = 4;
  dI += bioParamOffset;
#ifdef USE_AVX2
  if (width == 8)
  {
    for (int y = 0; y < height; y++)
    {
      __m128i tempGX =
        _mm_add_epi16(_mm_loadu_si128((const __m128i *)(gradX0)), _mm_loadu_si128((const __m128i *)(gradX1)));
      __m128i tempGY =
        _mm_add_epi16(_mm_loadu_si128((const __m128i *)(gradY0)), _mm_loadu_si128((const __m128i *)(gradY1)));
      _mm_storeu_si128((__m128i *)(gX), tempGX);
      _mm_storeu_si128((__m128i *)(gY), tempGY);
      __m128i temp = _mm_sub_epi16(_mm_srai_epi16(_mm_loadu_si128((const __m128i *)(srcY1Tmp)), shift4),
                                   _mm_srai_epi16(_mm_loadu_si128((const __m128i *)(srcY0Tmp)), shift4));
      _mm_storeu_si128((__m128i *)(dI), temp);
      // s1[x] =  tempGX * tempGX;
      __m128i lowS1  = _mm_mullo_epi16(tempGX, tempGX);
      __m128i highS1 = _mm_mulhi_epi16(tempGX, tempGX);
      _mm_storeu_si128((__m128i *)(s1), _mm_unpacklo_epi16(lowS1, highS1));
      _mm_storeu_si128((__m128i *)(s1 + 4), _mm_unpackhi_epi16(lowS1, highS1));
      // s2[x] =  tempGX * tempGY;
      __m128i lowS2  = _mm_mullo_epi16(tempGX, tempGY);
      __m128i highS2 = _mm_mulhi_epi16(tempGX, tempGY);
      _mm_storeu_si128((__m128i *)(s2), _mm_unpacklo_epi16(lowS2, highS2));
      _mm_storeu_si128((__m128i *)(s2 + 4), _mm_unpackhi_epi16(lowS2, highS2));
      // s5[x] =  tempGY * tempGY;
      __m128i lowS5  = _mm_mullo_epi16(tempGY, tempGY);
      __m128i highS5 = _mm_mulhi_epi16(tempGY, tempGY);
      _mm_storeu_si128((__m128i *)(s5), _mm_unpacklo_epi16(lowS5, highS5));
      _mm_storeu_si128((__m128i *)(s5 + 4), _mm_unpackhi_epi16(lowS5, highS5));
      // s3[x] = -tempGX * temp;
      __m128i lowS3  = _mm_mullo_epi16(tempGX, temp);
      __m128i highS3 = _mm_mulhi_epi16(tempGX, temp);
      _mm_storeu_si128((__m128i *)(s3), _mm_unpacklo_epi16(lowS3, highS3));
      _mm_storeu_si128((__m128i *)(s3 + 4), _mm_unpackhi_epi16(lowS3, highS3));
      // s6[x] = -tempGY * temp;
      __m128i lowS6  = _mm_mullo_epi16(tempGY, temp);
      __m128i highS6 = _mm_mulhi_epi16(tempGY, temp);
      _mm_storeu_si128((__m128i *)(s6), _mm_unpacklo_epi16(lowS6, highS6));
      _mm_storeu_si128((__m128i *)(s6 + 4), _mm_unpackhi_epi16(lowS6, highS6));
      srcY0Tmp += src0Stride;
      srcY1Tmp += src1Stride;
      gradX0 += widthG;
      gradX1 += widthG;
      gradY0 += widthG;
      gradY1 += widthG;
      s1 += widthG;
      s2 += widthG;
      s3 += widthG;
      s5 += widthG;
      s6 += widthG;
      dI += widthG;
      gX += widthG;
      gY += widthG;
    }
  }
  else
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 16)
      {
        __m256i tempGX = _mm256_add_epi16(_mm256_loadu_si256((const __m256i *)(gradX0 + x)),
                                          _mm256_loadu_si256((const __m256i *)(gradX1 + x)));
        __m256i tempGY = _mm256_add_epi16(_mm256_loadu_si256((const __m256i *)(gradY0 + x)),
                                          _mm256_loadu_si256((const __m256i *)(gradY1 + x)));
        _mm256_storeu_si256((__m256i *)(gX + x), tempGX);
        _mm256_storeu_si256((__m256i *)(gY + x), tempGY);
        __m256i temp = _mm256_sub_epi16(_mm256_srai_epi16(_mm256_loadu_si256((const __m256i *)(srcY1Tmp + x)), shift4),
                                        _mm256_srai_epi16(_mm256_loadu_si256((const __m256i *)(srcY0Tmp + x)), shift4));
        _mm256_storeu_si256((__m256i *)(dI + x), temp);
        // s1[x] =  tempGX * tempGX;
        __m256i lowS1  = _mm256_mullo_epi16(tempGX, tempGX);
        __m256i highS1 = _mm256_mulhi_epi16(tempGX, tempGX);
        __m256i tem0S1 = _mm256_unpacklo_epi16(lowS1, highS1);
        __m256i tem1S1 = _mm256_unpackhi_epi16(lowS1, highS1);
        _mm256_storeu_si256((__m256i *)(s1 + x), _mm256_permute2x128_si256(tem0S1, tem1S1, 0x20));
        _mm256_storeu_si256((__m256i *)(s1 + x + 8), _mm256_permute2x128_si256(tem0S1, tem1S1, 0x31));
        // s2[x] =  tempGX * tempGY;
        __m256i lowS2  = _mm256_mullo_epi16(tempGX, tempGY);
        __m256i highS2 = _mm256_mulhi_epi16(tempGX, tempGY);
        __m256i tem0S2 = _mm256_unpacklo_epi16(lowS2, highS2);
        __m256i tem1S2 = _mm256_unpackhi_epi16(lowS2, highS2);
        _mm256_storeu_si256((__m256i *)(s2 + x), _mm256_permute2x128_si256(tem0S2, tem1S2, 0x20));
        _mm256_storeu_si256((__m256i *)(s2 + x + 8), _mm256_permute2x128_si256(tem0S2, tem1S2, 0x31));
        // s5[x] =  tempGY * tempGY;
        __m256i lowS5  = _mm256_mullo_epi16(tempGY, tempGY);
        __m256i highS5 = _mm256_mulhi_epi16(tempGY, tempGY);
        __m256i tem0S5 = _mm256_unpacklo_epi16(lowS5, highS5);
        __m256i tem1S5 = _mm256_unpackhi_epi16(lowS5, highS5);
        _mm256_storeu_si256((__m256i *)(s5 + x), _mm256_permute2x128_si256(tem0S5, tem1S5, 0x20));
        _mm256_storeu_si256((__m256i *)(s5 + x + 8), _mm256_permute2x128_si256(tem0S5, tem1S5, 0x31));
        // s3[x] = -tempGX * temp;
        __m256i lowS3  = _mm256_mullo_epi16(tempGX, temp);
        __m256i highS3 = _mm256_mulhi_epi16(tempGX, temp);
        __m256i tem0S3 = _mm256_unpacklo_epi16(lowS3, highS3);
        __m256i tem1S3 = _mm256_unpackhi_epi16(lowS3, highS3);
        _mm256_storeu_si256((__m256i *)(s3 + x), _mm256_permute2x128_si256(tem0S3, tem1S3, 0x20));
        _mm256_storeu_si256((__m256i *)(s3 + x + 8), _mm256_permute2x128_si256(tem0S3, tem1S3, 0x31));
        // s6[x] = -tempGY * temp;
        __m256i lowS6  = _mm256_mullo_epi16(tempGY, temp);
        __m256i highS6 = _mm256_mulhi_epi16(tempGY, temp);
        __m256i tem0S6 = _mm256_unpacklo_epi16(lowS6, highS6);
        __m256i tem1S6 = _mm256_unpackhi_epi16(lowS6, highS6);
        _mm256_storeu_si256((__m256i *)(s6 + x), _mm256_permute2x128_si256(tem0S6, tem1S6, 0x20));
        _mm256_storeu_si256((__m256i *)(s6 + x + 8), _mm256_permute2x128_si256(tem0S6, tem1S6, 0x31));
      }
      srcY0Tmp += src0Stride;
      srcY1Tmp += src1Stride;
      gradX0 += widthG;
      gradX1 += widthG;
      gradY0 += widthG;
      gradY1 += widthG;
      s1 += widthG;
      s2 += widthG;
      s3 += widthG;
      s5 += widthG;
      s6 += widthG;
      dI += widthG;
      gX += widthG;
      gY += widthG;
    }
  }
#else
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x += 8)
    {
      __m128i tempGX =
        _mm_add_epi16(_mm_loadu_si128((const __m128i *)(gradX0 + x)), _mm_loadu_si128((const __m128i *)(gradX1 + x)));
      __m128i tempGY =
        _mm_add_epi16(_mm_loadu_si128((const __m128i *)(gradY0 + x)), _mm_loadu_si128((const __m128i *)(gradY1 + x)));
      _mm_storeu_si128((__m128i *)(gX + x), tempGX);
      _mm_storeu_si128((__m128i *)(gY + x), tempGY);
      __m128i temp = _mm_sub_epi16(_mm_srai_epi16(_mm_loadu_si128((const __m128i *)(srcY1Tmp + x)), shift4),
                                   _mm_srai_epi16(_mm_loadu_si128((const __m128i *)(srcY0Tmp + x)), shift4));
      _mm_storeu_si128((__m128i *)(dI + x), temp);

      // s1[x] =  tempGX * tempGX;
      __m128i lowS1  = _mm_mullo_epi16(tempGX, tempGX);
      __m128i highS1 = _mm_mulhi_epi16(tempGX, tempGX);
      _mm_storeu_si128((__m128i *)(s1 + x), _mm_unpacklo_epi16(lowS1, highS1));
      _mm_storeu_si128((__m128i *)(s1 + x + 4), _mm_unpackhi_epi16(lowS1, highS1));
      // s2[x] =  tempGX * tempGY;
      __m128i lowS2  = _mm_mullo_epi16(tempGX, tempGY);
      __m128i highS2 = _mm_mulhi_epi16(tempGX, tempGY);
      _mm_storeu_si128((__m128i *)(s2 + x), _mm_unpacklo_epi16(lowS2, highS2));
      _mm_storeu_si128((__m128i *)(s2 + x + 4), _mm_unpackhi_epi16(lowS2, highS2));
      // s5[x] =  tempGY * tempGY;
      __m128i lowS5  = _mm_mullo_epi16(tempGY, tempGY);
      __m128i highS5 = _mm_mulhi_epi16(tempGY, tempGY);
      _mm_storeu_si128((__m128i *)(s5 + x), _mm_unpacklo_epi16(lowS5, highS5));
      _mm_storeu_si128((__m128i *)(s5 + x + 4), _mm_unpackhi_epi16(lowS5, highS5));
      // s3[x] = -tempGX * temp;
      __m128i lowS3  = _mm_mullo_epi16(tempGX, temp);
      __m128i highS3 = _mm_mulhi_epi16(tempGX, temp);
      _mm_storeu_si128((__m128i *)(s3 + x), _mm_unpacklo_epi16(lowS3, highS3));
      _mm_storeu_si128((__m128i *)(s3 + x + 4), _mm_unpackhi_epi16(lowS3, highS3));
      // s6[x] = -tempGY * temp;
      __m128i lowS6  = _mm_mullo_epi16(tempGY, temp);
      __m128i highS6 = _mm_mulhi_epi16(tempGY, temp);
      _mm_storeu_si128((__m128i *)(s6 + x), _mm_unpacklo_epi16(lowS6, highS6));
      _mm_storeu_si128((__m128i *)(s6 + x + 4), _mm_unpackhi_epi16(lowS6, highS6));
    }
    srcY0Tmp += src0Stride;
    srcY1Tmp += src1Stride;
    gradX0 += widthG;
    gradX1 += widthG;
    gradY0 += widthG;
    gradY1 += widthG;
    s1 += widthG;
    s2 += widthG;
    s3 += widthG;
    s5 += widthG;
    s6 += widthG;
    dI += widthG;
    gX += widthG;
    gY += widthG;
  }
#endif
}

template<X86_VEXT vext> void calcBIOParamSumHighPrecision_SSE(int32_t *s1, int32_t *s2, int32_t *s3, int32_t *s5,
                                                              int32_t *s6, int width, int height, const int widthG,
                                                              int32_t *sumS1, int32_t *sumS2, int32_t *sumS3,
                                                              int32_t *sumS5, int32_t *sumS6, Pel *dI, Pel *gX, Pel *gY,
                                                              bool isGPM = false, bool isSub = false)
{
  int meanDiff    = 0;
  int absmeanDiff = 0;
  if (!isGPM)
  {
    __m128i vzero     = _mm_setzero_si128();
    __m128i vsum32    = vzero;
    __m128i vdiff     = vzero;
    __m128i vsumabs32 = vzero;
    __m128i vdiffabs  = vzero;

    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x = x + 8)
      {
        vdiff         = _mm_loadu_si128((const __m128i *)(dI + x));
        __m128i vsign = _mm_cmpgt_epi16(vzero, vdiff);
        vdiffabs      = _mm_abs_epi16(vdiff);
        vsum32        = _mm_add_epi32(vsum32, _mm_unpacklo_epi16(vdiff, vsign));
        vsumabs32     = _mm_add_epi32(vsumabs32, _mm_unpacklo_epi16(vdiffabs, vzero));
        if (x + 8 < width || width == 8)
        {
          vsum32    = _mm_add_epi32(vsum32, _mm_unpackhi_epi16(vdiff, vsign));
          vsumabs32 = _mm_add_epi32(vsumabs32, _mm_unpackhi_epi16(vdiffabs, vzero));
        }
      }
      dI += widthG;
    }

    vsum32   = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
    vsum32   = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    meanDiff = _mm_cvtsi128_si32(vsum32);

    vsumabs32   = _mm_add_epi32(vsumabs32, _mm_shuffle_epi32(vsumabs32, 0x4e));   // 01001110
    vsumabs32   = _mm_add_epi32(vsumabs32, _mm_shuffle_epi32(vsumabs32, 0xb1));   // 10110001
    absmeanDiff = _mm_cvtsi128_si32(vsumabs32);

    if (isSub)
    {
      meanDiff = getMean(meanDiff + (height * width >> 1), height * width);
    }
    else
    {
      if ((absmeanDiff > 2 * abs(meanDiff)))
      {
        meanDiff = 0;
      }
      meanDiff = getMean(meanDiff + height * width, height * width << 1);
    }
  }
  __m128i vmask0 =
    _mm_setr_epi16(-meanDiff, -meanDiff, -meanDiff, -meanDiff, -meanDiff, -meanDiff, -meanDiff, -meanDiff);
  // int bdofWidth[]  = {8,  8,  8,   12,  12,  12,  20,  20,   20};
  // int bdofHeight[] = {8, 12, 20,    8,  12,  20,   8,  12,   20};
  int     weightOffset[] = { 0, 64, 160, 320, 416, 560, 800, 960, 1200 };
  int     idx            = 0;
  __m128i vzero          = _mm_setzero_si128();
  __m128i sumS1Temp      = vzero;
  __m128i sumS2Temp      = vzero;
  __m128i sumS5Temp      = vzero;
  __m128i sumS3Temp      = vzero;
  __m128i sumS6Temp      = vzero;

  if (width == 4 + 4)
  {
    idx         = (height == 8) ? 0 : ((height == 12) ? 1 : 2);
    int *weight = g_bdofWeight + weightOffset[idx];
    for (int y = 0; y < height; y++)
    {
      __m128i w0 = _mm_loadu_si128((const __m128i *)weight);
      __m128i w1 = _mm_loadu_si128((const __m128i *)(weight + 4));

      sumS1Temp = _mm_add_epi32(sumS1Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)s1), w0));
      sumS1Temp = _mm_add_epi32(sumS1Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s1 + 4)), w1));

      sumS2Temp = _mm_add_epi32(sumS2Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)s2), w0));
      sumS2Temp = _mm_add_epi32(sumS2Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s2 + 4)), w1));

      sumS5Temp = _mm_add_epi32(sumS5Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)s5), w0));
      sumS5Temp = _mm_add_epi32(sumS5Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s5 + 4)), w1));

      sumS3Temp      = _mm_add_epi32(sumS3Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)s3), w0));
      sumS3Temp      = _mm_add_epi32(sumS3Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s3 + 4)), w1));
      __m128i lowS3  = _mm_mullo_epi16(_mm_loadu_si128((const __m128i *)gX), vmask0);
      __m128i highS3 = _mm_mulhi_epi16(_mm_loadu_si128((const __m128i *)gX), vmask0);
      sumS3Temp      = _mm_add_epi32(sumS3Temp, _mm_mullo_epi32(_mm_unpacklo_epi16(lowS3, highS3), w0));
      sumS3Temp      = _mm_add_epi32(sumS3Temp, _mm_mullo_epi32(_mm_unpackhi_epi16(lowS3, highS3), w1));

      sumS6Temp      = _mm_add_epi32(sumS6Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)s6), w0));
      sumS6Temp      = _mm_add_epi32(sumS6Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s6 + 4)), w1));
      __m128i lowS6  = _mm_mullo_epi16(_mm_loadu_si128((const __m128i *)gY), vmask0);
      __m128i highS6 = _mm_mulhi_epi16(_mm_loadu_si128((const __m128i *)gY), vmask0);
      sumS6Temp      = _mm_add_epi32(sumS6Temp, _mm_mullo_epi32(_mm_unpacklo_epi16(lowS6, highS6), w0));
      sumS6Temp      = _mm_add_epi32(sumS6Temp, _mm_mullo_epi32(_mm_unpackhi_epi16(lowS6, highS6), w1));

      // bio parameter increment
      s1 += widthG;
      s2 += widthG;
      s5 += widthG;
      s3 += widthG;
      s6 += widthG;
      gX += widthG;
      gY += widthG;
      weight += width;
    }
  }
  else if (width == 8 + 4)   // width = 12
  {
    idx         = (height == 8) ? 3 : ((height == 12) ? 4 : 5);
    int *weight = g_bdofWeight + weightOffset[idx];
    for (int y = 0; y < height; y++)
    {
      __m128i w0 = _mm_loadu_si128((const __m128i *)weight);
      __m128i w1 = _mm_loadu_si128((const __m128i *)(weight + 4));
      __m128i w2 = _mm_loadu_si128((const __m128i *)(weight + 8));
      sumS1Temp  = _mm_add_epi32(sumS1Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)s1), w0));
      sumS1Temp  = _mm_add_epi32(sumS1Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s1 + 4)), w1));
      sumS1Temp  = _mm_add_epi32(sumS1Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s1 + 8)), w2));

      sumS2Temp = _mm_add_epi32(sumS2Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)s2), w0));
      sumS2Temp = _mm_add_epi32(sumS2Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s2 + 4)), w1));
      sumS2Temp = _mm_add_epi32(sumS2Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s2 + 8)), w2));

      sumS5Temp = _mm_add_epi32(sumS5Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)s5), w0));
      sumS5Temp = _mm_add_epi32(sumS5Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s5 + 4)), w1));
      sumS5Temp = _mm_add_epi32(sumS5Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s5 + 8)), w2));

      sumS3Temp      = _mm_add_epi32(sumS3Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)s3), w0));
      sumS3Temp      = _mm_add_epi32(sumS3Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s3 + 4)), w1));
      sumS3Temp      = _mm_add_epi32(sumS3Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s3 + 8)), w2));
      __m128i lowS3  = _mm_mullo_epi16(_mm_loadu_si128((const __m128i *)gX), vmask0);
      __m128i highS3 = _mm_mulhi_epi16(_mm_loadu_si128((const __m128i *)gX), vmask0);
      sumS3Temp      = _mm_add_epi32(sumS3Temp, _mm_mullo_epi32(_mm_unpacklo_epi16(lowS3, highS3), w0));
      sumS3Temp      = _mm_add_epi32(sumS3Temp, _mm_mullo_epi32(_mm_unpackhi_epi16(lowS3, highS3), w1));
      lowS3          = _mm_mullo_epi16(_mm_loadu_si128((const __m128i *)(gX + 8)), vmask0);
      highS3         = _mm_mulhi_epi16(_mm_loadu_si128((const __m128i *)(gX + 8)), vmask0);
      sumS3Temp      = _mm_add_epi32(sumS3Temp, _mm_mullo_epi32(_mm_unpacklo_epi16(lowS3, highS3), w2));

      sumS6Temp      = _mm_add_epi32(sumS6Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)s6), w0));
      sumS6Temp      = _mm_add_epi32(sumS6Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s6 + 4)), w1));
      sumS6Temp      = _mm_add_epi32(sumS6Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s6 + 8)), w2));
      __m128i lowS6  = _mm_mullo_epi16(_mm_loadu_si128((const __m128i *)gY), vmask0);
      __m128i highS6 = _mm_mulhi_epi16(_mm_loadu_si128((const __m128i *)gY), vmask0);
      sumS6Temp      = _mm_add_epi32(sumS6Temp, _mm_mullo_epi32(_mm_unpacklo_epi16(lowS6, highS6), w0));
      sumS6Temp      = _mm_add_epi32(sumS6Temp, _mm_mullo_epi32(_mm_unpackhi_epi16(lowS6, highS6), w1));
      lowS6          = _mm_mullo_epi16(_mm_loadu_si128((const __m128i *)(gY + 8)), vmask0);
      highS6         = _mm_mulhi_epi16(_mm_loadu_si128((const __m128i *)(gY + 8)), vmask0);
      sumS6Temp      = _mm_add_epi32(sumS6Temp, _mm_mullo_epi32(_mm_unpacklo_epi16(lowS6, highS6), w2));

      // bio parameter increment
      s1 += widthG;
      s2 += widthG;
      s5 += widthG;
      s3 += widthG;
      s6 += widthG;
      gX += widthG;
      gY += widthG;
      weight += width;
    }
  }
  else if (width == 16 + 4)   // width = 20
  {
    idx         = (height == 8) ? 6 : ((height == 12) ? 7 : 8);
    int *weight = g_bdofWeight + weightOffset[idx];
    for (int y = 0; y < height; y++)
    {
      __m128i w0 = _mm_loadu_si128((const __m128i *)weight);
      __m128i w1 = _mm_loadu_si128((const __m128i *)(weight + 4));
      __m128i w2 = _mm_loadu_si128((const __m128i *)(weight + 8));
      __m128i w3 = _mm_loadu_si128((const __m128i *)(weight + 12));
      __m128i w4 = _mm_loadu_si128((const __m128i *)(weight + 16));
      sumS1Temp  = _mm_add_epi32(sumS1Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)s1), w0));
      sumS1Temp  = _mm_add_epi32(sumS1Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s1 + 4)), w1));
      sumS1Temp  = _mm_add_epi32(sumS1Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s1 + 8)), w2));
      sumS1Temp  = _mm_add_epi32(sumS1Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s1 + 12)), w3));
      sumS1Temp  = _mm_add_epi32(sumS1Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s1 + 16)), w4));

      sumS2Temp = _mm_add_epi32(sumS2Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)s2), w0));
      sumS2Temp = _mm_add_epi32(sumS2Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s2 + 4)), w1));
      sumS2Temp = _mm_add_epi32(sumS2Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s2 + 8)), w2));
      sumS2Temp = _mm_add_epi32(sumS2Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s2 + 12)), w3));
      sumS2Temp = _mm_add_epi32(sumS2Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s2 + 16)), w4));

      sumS5Temp = _mm_add_epi32(sumS5Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)s5), w0));
      sumS5Temp = _mm_add_epi32(sumS5Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s5 + 4)), w1));
      sumS5Temp = _mm_add_epi32(sumS5Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s5 + 8)), w2));
      sumS5Temp = _mm_add_epi32(sumS5Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s5 + 12)), w3));
      sumS5Temp = _mm_add_epi32(sumS5Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s5 + 16)), w4));

      sumS3Temp      = _mm_add_epi32(sumS3Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)s3), w0));
      sumS3Temp      = _mm_add_epi32(sumS3Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s3 + 4)), w1));
      sumS3Temp      = _mm_add_epi32(sumS3Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s3 + 8)), w2));
      sumS3Temp      = _mm_add_epi32(sumS3Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s3 + 12)), w3));
      sumS3Temp      = _mm_add_epi32(sumS3Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s3 + 16)), w4));
      __m128i lowS3  = _mm_mullo_epi16(_mm_loadu_si128((const __m128i *)gX), vmask0);
      __m128i highS3 = _mm_mulhi_epi16(_mm_loadu_si128((const __m128i *)gX), vmask0);
      sumS3Temp      = _mm_add_epi32(sumS3Temp, _mm_mullo_epi32(_mm_unpacklo_epi16(lowS3, highS3), w0));
      sumS3Temp      = _mm_add_epi32(sumS3Temp, _mm_mullo_epi32(_mm_unpackhi_epi16(lowS3, highS3), w1));
      lowS3          = _mm_mullo_epi16(_mm_loadu_si128((const __m128i *)(gX + 8)), vmask0);
      highS3         = _mm_mulhi_epi16(_mm_loadu_si128((const __m128i *)(gX + 8)), vmask0);
      sumS3Temp      = _mm_add_epi32(sumS3Temp, _mm_mullo_epi32(_mm_unpacklo_epi16(lowS3, highS3), w2));
      sumS3Temp      = _mm_add_epi32(sumS3Temp, _mm_mullo_epi32(_mm_unpackhi_epi16(lowS3, highS3), w3));
      lowS3          = _mm_mullo_epi16(_mm_loadu_si128((const __m128i *)(gX + 16)), vmask0);
      highS3         = _mm_mulhi_epi16(_mm_loadu_si128((const __m128i *)(gX + 16)), vmask0);
      sumS3Temp      = _mm_add_epi32(sumS3Temp, _mm_mullo_epi32(_mm_unpacklo_epi16(lowS3, highS3), w4));

      sumS6Temp      = _mm_add_epi32(sumS6Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)s6), w0));
      sumS6Temp      = _mm_add_epi32(sumS6Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s6 + 4)), w1));
      sumS6Temp      = _mm_add_epi32(sumS6Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s6 + 8)), w2));
      sumS6Temp      = _mm_add_epi32(sumS6Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s6 + 12)), w3));
      sumS6Temp      = _mm_add_epi32(sumS6Temp, _mm_mullo_epi32(_mm_loadu_si128((const __m128i *)(s6 + 16)), w4));
      __m128i lowS6  = _mm_mullo_epi16(_mm_loadu_si128((const __m128i *)gY), vmask0);
      __m128i highS6 = _mm_mulhi_epi16(_mm_loadu_si128((const __m128i *)gY), vmask0);
      sumS6Temp      = _mm_add_epi32(sumS6Temp, _mm_mullo_epi32(_mm_unpacklo_epi16(lowS6, highS6), w0));
      sumS6Temp      = _mm_add_epi32(sumS6Temp, _mm_mullo_epi32(_mm_unpackhi_epi16(lowS6, highS6), w1));
      lowS6          = _mm_mullo_epi16(_mm_loadu_si128((const __m128i *)(gY + 8)), vmask0);
      highS6         = _mm_mulhi_epi16(_mm_loadu_si128((const __m128i *)(gY + 8)), vmask0);
      sumS6Temp      = _mm_add_epi32(sumS6Temp, _mm_mullo_epi32(_mm_unpacklo_epi16(lowS6, highS6), w2));
      sumS6Temp      = _mm_add_epi32(sumS6Temp, _mm_mullo_epi32(_mm_unpackhi_epi16(lowS6, highS6), w3));
      lowS6          = _mm_mullo_epi16(_mm_loadu_si128((const __m128i *)(gY + 16)), vmask0);
      highS6         = _mm_mulhi_epi16(_mm_loadu_si128((const __m128i *)(gY + 16)), vmask0);
      sumS6Temp      = _mm_add_epi32(sumS6Temp, _mm_mullo_epi32(_mm_unpacklo_epi16(lowS6, highS6), w4));

      // bio parameter increment
      s1 += widthG;
      s2 += widthG;
      s5 += widthG;
      s3 += widthG;
      s6 += widthG;
      gX += widthG;
      gY += widthG;
      weight += width;
    }
  }

  sumS1Temp = _mm_add_epi32(sumS1Temp, _mm_shuffle_epi32(sumS1Temp, 0x4e));   // 01001110
  sumS1Temp = _mm_add_epi32(sumS1Temp, _mm_shuffle_epi32(sumS1Temp, 0xb1));   // 10110001
  *sumS1    = _mm_cvtsi128_si32(sumS1Temp);

  sumS2Temp = _mm_add_epi32(sumS2Temp, _mm_shuffle_epi32(sumS2Temp, 0x4e));   // 01001110
  sumS2Temp = _mm_add_epi32(sumS2Temp, _mm_shuffle_epi32(sumS2Temp, 0xb1));   // 10110001
  *sumS2    = _mm_cvtsi128_si32(sumS2Temp);

  sumS5Temp = _mm_add_epi32(sumS5Temp, _mm_shuffle_epi32(sumS5Temp, 0x4e));   // 01001110
  sumS5Temp = _mm_add_epi32(sumS5Temp, _mm_shuffle_epi32(sumS5Temp, 0xb1));   // 10110001
  *sumS5    = _mm_cvtsi128_si32(sumS5Temp);

  sumS3Temp = _mm_add_epi32(sumS3Temp, _mm_shuffle_epi32(sumS3Temp, 0x4e));   // 01001110
  sumS3Temp = _mm_add_epi32(sumS3Temp, _mm_shuffle_epi32(sumS3Temp, 0xb1));   // 10110001
  *sumS3    = _mm_cvtsi128_si32(sumS3Temp);

  sumS6Temp = _mm_add_epi32(sumS6Temp, _mm_shuffle_epi32(sumS6Temp, 0x4e));   // 01001110
  sumS6Temp = _mm_add_epi32(sumS6Temp, _mm_shuffle_epi32(sumS6Temp, 0xb1));   // 10110001
  *sumS6    = _mm_cvtsi128_si32(sumS6Temp);
}

template<X86_VEXT vext> void calcBIOParamSum4_SSE(Pel *absGX, Pel *absGY, Pel *dIX, Pel *dIY, Pel *signGY_GX, int width,
                                                  int height, const int widthG, int *sumAbsGX, int *sumAbsGY,
                                                  int *sumDIX, int *sumDIY, int *sumSignGY_GX)
{
  __m128i vzero = _mm_setzero_si128();

  __m128i sumAbsGXTmp16    = vzero;
  __m128i sumDIXTmp16      = vzero;
  __m128i sumAbsGYTmp16    = vzero;
  __m128i sumDIYTmp16      = vzero;
  __m128i sumSignGyGxTmp16 = vzero;
  if (width == 8)
  {
    for (int y = 0; y < height; y++)
    {
      sumAbsGXTmp16    = _mm_add_epi16(sumAbsGXTmp16, _mm_loadu_si128((const __m128i *)absGX));
      sumDIXTmp16      = _mm_add_epi16(sumDIXTmp16, _mm_loadu_si128((const __m128i *)dIX));
      sumAbsGYTmp16    = _mm_add_epi16(sumAbsGYTmp16, _mm_loadu_si128((const __m128i *)absGY));
      sumDIYTmp16      = _mm_add_epi16(sumDIYTmp16, _mm_loadu_si128((const __m128i *)dIY));
      sumSignGyGxTmp16 = _mm_add_epi16(sumSignGyGxTmp16, _mm_loadu_si128((const __m128i *)signGY_GX));
      // bio parameter increment
      absGX += widthG;
      absGY += widthG;
      dIX += widthG;
      dIY += widthG;
      signGY_GX += widthG;
    }
  }
  else   // (width == 12)
  {
    for (int y = 0; y < height; y++)
    {
      sumAbsGXTmp16    = _mm_add_epi16(sumAbsGXTmp16, _mm_loadu_si128((const __m128i *)absGX));
      sumAbsGXTmp16    = _mm_add_epi16(sumAbsGXTmp16, _mm_loadl_epi64((const __m128i *)(absGX + 8)));
      sumDIXTmp16      = _mm_add_epi16(sumDIXTmp16, _mm_loadu_si128((const __m128i *)dIX));
      sumDIXTmp16      = _mm_add_epi16(sumDIXTmp16, _mm_loadl_epi64((const __m128i *)(dIX + 8)));
      sumAbsGYTmp16    = _mm_add_epi16(sumAbsGYTmp16, _mm_loadu_si128((const __m128i *)absGY));
      sumAbsGYTmp16    = _mm_add_epi16(sumAbsGYTmp16, _mm_loadl_epi64((const __m128i *)(absGY + 8)));
      sumDIYTmp16      = _mm_add_epi16(sumDIYTmp16, _mm_loadu_si128((const __m128i *)dIY));
      sumDIYTmp16      = _mm_add_epi16(sumDIYTmp16, _mm_loadl_epi64((const __m128i *)(dIY + 8)));
      sumSignGyGxTmp16 = _mm_add_epi16(sumSignGyGxTmp16, _mm_loadu_si128((const __m128i *)signGY_GX));
      sumSignGyGxTmp16 = _mm_add_epi16(sumSignGyGxTmp16, _mm_loadl_epi64((const __m128i *)(signGY_GX + 8)));
      // bio parameter increment
      absGX += widthG;
      absGY += widthG;
      dIX += widthG;
      dIY += widthG;
      signGY_GX += widthG;
    }
  }

  __m128i sumAbsGXTmp32 =
    _mm_add_epi32(_mm_unpacklo_epi16(sumAbsGXTmp16, vzero), _mm_unpackhi_epi16(sumAbsGXTmp16, vzero));
  __m128i sumAbsGYTmp32 =
    _mm_add_epi32(_mm_unpacklo_epi16(sumAbsGYTmp16, vzero), _mm_unpackhi_epi16(sumAbsGYTmp16, vzero));
  __m128i sumDIXTmp32 = _mm_madd_epi16(sumDIXTmp16, _mm_set1_epi16(1));
  __m128i sumDIYTmp32 = _mm_madd_epi16(sumDIYTmp16, _mm_set1_epi16(1));
  __m128i a12         = _mm_unpacklo_epi32(sumAbsGXTmp32, sumAbsGYTmp32);
  __m128i a3          = _mm_unpackhi_epi32(sumAbsGXTmp32, sumAbsGYTmp32);
  __m128i b12         = _mm_unpacklo_epi32(sumDIXTmp32, sumDIYTmp32);
  __m128i b3          = _mm_unpackhi_epi32(sumDIXTmp32, sumDIYTmp32);

  __m128i c1 = _mm_unpacklo_epi64(a12, b12);
  c1         = _mm_add_epi32(c1, _mm_unpackhi_epi64(a12, b12));
  c1         = _mm_add_epi32(c1, _mm_unpacklo_epi64(a3, b3));
  c1         = _mm_add_epi32(c1, _mm_unpackhi_epi64(a3, b3));

  *sumAbsGX = _mm_cvtsi128_si32(c1);
  *sumAbsGY = _mm_cvtsi128_si32(_mm_shuffle_epi32(c1, 0x55));
  *sumDIX   = _mm_cvtsi128_si32(_mm_shuffle_epi32(c1, 0xaa));
  *sumDIY   = _mm_cvtsi128_si32(_mm_shuffle_epi32(c1, 0xff));

  __m128i sumSignGyGxTmp32 = _mm_madd_epi16(sumSignGyGxTmp16, _mm_set1_epi16(1));
  sumSignGyGxTmp32         = _mm_add_epi32(sumSignGyGxTmp32, _mm_shuffle_epi32(sumSignGyGxTmp32, 0x4e));   // 01001110
  sumSignGyGxTmp32         = _mm_add_epi32(sumSignGyGxTmp32, _mm_shuffle_epi32(sumSignGyGxTmp32, 0xb1));   // 10110001
  *sumSignGY_GX            = _mm_cvtsi128_si32(sumSignGyGxTmp32);
}

inline __m128i mm_srav_epi32_sse2(__m128i a, __m128i count)
{
  // Shift each element separately and blend
  __m128i vZero = _mm_setzero_si128();
  __m128i cnt   = _mm_and_si128(count, _mm_set1_epi32(0x1FFFFFFF));

  __m128i shift0 = _mm_sra_epi32(a, _mm_blend_epi16(vZero, _mm_shuffle_epi32(cnt, _MM_SHUFFLE(0, 0, 0, 0)), 0x3));
  __m128i shift1 = _mm_sra_epi32(a, _mm_blend_epi16(vZero, _mm_shuffle_epi32(cnt, _MM_SHUFFLE(1, 1, 1, 1)), 0x3));
  __m128i shift2 = _mm_sra_epi32(a, _mm_blend_epi16(vZero, _mm_shuffle_epi32(cnt, _MM_SHUFFLE(2, 2, 2, 2)), 0x3));
  __m128i shift3 = _mm_sra_epi32(a, _mm_blend_epi16(vZero, _mm_shuffle_epi32(cnt, _MM_SHUFFLE(3, 3, 3, 3)), 0x3));

  __m128i result = _mm_blend_epi16(shift0, shift1, 0x0C);
  result         = _mm_blend_epi16(result, shift2, 0x30);
  result         = _mm_blend_epi16(result, shift3, 0xC0);
  return result;
}

inline __m128i mm_sllv_epi32_sse2(__m128i a, __m128i count)
{
  // Shift each element separately and blend
  __m128i vZero = _mm_setzero_si128();
  __m128i cnt   = _mm_and_si128(count, _mm_set1_epi32(0x1FFFFFFF));

  __m128i shift0 = _mm_sll_epi32(a, _mm_blend_epi16(vZero, _mm_shuffle_epi32(cnt, _MM_SHUFFLE(0, 0, 0, 0)), 0x3));
  __m128i shift1 = _mm_sll_epi32(a, _mm_blend_epi16(vZero, _mm_shuffle_epi32(cnt, _MM_SHUFFLE(1, 1, 1, 1)), 0x3));
  __m128i shift2 = _mm_sll_epi32(a, _mm_blend_epi16(vZero, _mm_shuffle_epi32(cnt, _MM_SHUFFLE(2, 2, 2, 2)), 0x3));
  __m128i shift3 = _mm_sll_epi32(a, _mm_blend_epi16(vZero, _mm_shuffle_epi32(cnt, _MM_SHUFFLE(3, 3, 3, 3)), 0x3));

  __m128i result = _mm_blend_epi16(shift0, shift1, 0x0C);
  result         = _mm_blend_epi16(result, shift2, 0x30);
  result         = _mm_blend_epi16(result, shift3, 0xC0);
  return result;
}

template<X86_VEXT vext>
void calcBIOClippedVxVy_SSE(int *sumDIXSample32bit, int *sumAbsGxSample32bit, int *sumDIYSample32bit,
                            int *sumAbsGySample32bit, int *sumSignGyGxSample32bit, const int limit,
                            const int bioSubblockSize, int *tmpxSample32bit, int *tmpySample32bit)
{
#ifdef USE_AVX2
  // Lookup table for division approximation
  alignas(32) int divTable[32] = { 32, 31, 30, 29, 28, 28, 27, 26, 26, 25, 24, 24, 23, 23, 22, 22,
                                   21, 21, 20, 20, 20, 19, 19, 19, 18, 18, 18, 17, 17, 17, 17, 16 };

  __m256i vLimit    = _mm256_set1_epi32(limit);
  __m256i vNegLimit = _mm256_set1_epi32(-limit);
  __m256i vConst5   = _mm256_set1_epi32(5);
  __m256i vConst31  = _mm256_set1_epi32(31);
  __m256i vConst1   = _mm256_set1_epi32(1);
  __m256i vZero     = _mm256_setzero_si256();

  for (int idx = 0; idx < bioSubblockSize; idx += 8)
  {
    // Load data
    __m256i vSumAbsGx    = _mm256_loadu_si256((__m256i *)&sumAbsGxSample32bit[idx]);
    __m256i vSumDIX      = _mm256_loadu_si256((__m256i *)&sumDIXSample32bit[idx]);
    __m256i vSumAbsGy    = _mm256_loadu_si256((__m256i *)&sumAbsGySample32bit[idx]);
    __m256i vSumDIY      = _mm256_loadu_si256((__m256i *)&sumDIYSample32bit[idx]);
    __m256i vSumSignGyGx = _mm256_loadu_si256((__m256i *)&sumSignGyGxSample32bit[idx]);

    // Compute log2D using leading zero count
    // #if USE_AVX512 // need to be tested
    //     __m256i vLog2D_Gx = _mm256_sub_epi32(_mm256_set1_epi32(31), _mm256_lzcnt_epi32(vSumAbsGx));
    //     __m256i vLog2D_Gy = _mm256_sub_epi32(_mm256_set1_epi32(31), _mm256_lzcnt_epi32(vSumAbsGy));
    // #else
    __m256i vConst32  = _mm256_set1_epi32(32);
    __m256i vConst158 = _mm256_set1_epi32(158);
    __m256i v;
    // prevent value from being rounded up to the next power of two
    v                   = _mm256_andnot_si256(_mm256_srli_epi32(vSumAbsGx, 8), vSumAbsGx);   // keep 8 MSB
    v                   = _mm256_castps_si256(_mm256_cvtepi32_ps(v));   // convert an integer to float
    v                   = _mm256_srli_epi32(v, 23);   // shift down the exponent
    v                   = _mm256_subs_epu16(vConst158, v);   // undo bias
    v                   = _mm256_min_epi16(v, vConst32);   // clamp at 32
    __m256i vLog2D_Gx   = _mm256_sub_epi32(vConst31, v);
    v                   = _mm256_andnot_si256(_mm256_srli_epi32(vSumAbsGy, 8), vSumAbsGy);   // keep 8 MSB
    v                   = _mm256_castps_si256(_mm256_cvtepi32_ps(v));   // convert an integer to float
    v                   = _mm256_srli_epi32(v, 23);   // shift down the exponent
    v                   = _mm256_subs_epu16(vConst158, v);   // undo bias
    v                   = _mm256_min_epi16(v, vConst32);   // clamp at 32
    __m256i vLog2D_Gy   = _mm256_sub_epi32(vConst31, v);
    // #endif
    //  Compute MSB ((x << 5) >> log2D) & 31
    __m256i vMsbD_Gx    = _mm256_and_si256(_mm256_srav_epi32(_mm256_slli_epi32(vSumAbsGx, 5), vLog2D_Gx), vConst31);
    __m256i vMsbD_Gy    = _mm256_and_si256(_mm256_srav_epi32(_mm256_slli_epi32(vSumAbsGy, 5), vLog2D_Gy), vConst31);
    __m256i vInvD_Gx    = _mm256_i32gather_epi32(divTable, vMsbD_Gx, 4);
    __m256i vShiftD_Gx  = _mm256_add_epi32(vLog2D_Gx, vConst5);
    __m256i vOffsetD_Gx = _mm256_sllv_epi32(vConst1, _mm256_sub_epi32(vShiftD_Gx, vConst1));
    __m256i vTmpX       = _mm256_srav_epi32(
      _mm256_add_epi32(_mm256_mullo_epi32(_mm256_slli_epi32(vSumDIX, 3), vInvD_Gx), vOffsetD_Gx), vShiftD_Gx);
    vTmpX = _mm256_min_epi32(_mm256_max_epi32(vTmpX, vNegLimit), vLimit);
    vTmpX =
      _mm256_blendv_epi8(vZero, vTmpX, _mm256_cmpgt_epi32(vSumAbsGx, vZero));   // Apply condition (if sumAbsGx > 0)

    // Y direction calculations
    __m256i vInvD_Gy    = _mm256_i32gather_epi32(divTable, vMsbD_Gy, 4);
    __m256i vShiftD_Gy  = _mm256_add_epi32(vLog2D_Gy, vConst5);
    __m256i vOffsetD_Gy = _mm256_sllv_epi32(vConst1, _mm256_sub_epi32(vShiftD_Gy, vConst1));

    __m256i vTmpY = _mm256_srav_epi32(
      _mm256_add_epi32(
        _mm256_mullo_epi32(_mm256_sub_epi32(_mm256_slli_epi32(vSumDIY, 3),
                                            _mm256_srai_epi32(_mm256_mullo_epi32(vTmpX, vSumSignGyGx), 1)),
                           vInvD_Gy),
        vOffsetD_Gy),
      vShiftD_Gy);
    vTmpY = _mm256_min_epi32(_mm256_max_epi32(vTmpY, vNegLimit), vLimit);
    vTmpY =
      _mm256_blendv_epi8(vZero, vTmpY, _mm256_cmpgt_epi32(vSumAbsGy, vZero));   // Apply condition (if sumAbsGy > 0)

    // Store results
    _mm256_storeu_si256((__m256i *)&tmpxSample32bit[idx], vTmpX);
    _mm256_storeu_si256((__m256i *)&tmpySample32bit[idx], vTmpY);
  }
#else
  alignas(16) int divTable[32] = { 32, 31, 30, 29, 28, 28, 27, 26, 26, 25, 24, 24, 23, 23, 22, 22,
                                   21, 21, 20, 20, 20, 19, 19, 19, 18, 18, 18, 17, 17, 17, 17, 16 };

  __m128i vLimit    = _mm_set1_epi32(limit);
  __m128i vNegLimit = _mm_set1_epi32(-limit);
  __m128i vConst5   = _mm_set1_epi32(5);
  __m128i vConst31  = _mm_set1_epi32(31);
  __m128i vConst1   = _mm_set1_epi32(1);
  __m128i vZero     = _mm_setzero_si128();

  for (int idx = 0; idx < bioSubblockSize; idx += 4)   // Process 4 elements per loop iteration
  {
    // Load data
    __m128i vSumAbsGx    = _mm_loadu_si128((__m128i *)&sumAbsGxSample32bit[idx]);
    __m128i vSumDIX      = _mm_loadu_si128((__m128i *)&sumDIXSample32bit[idx]);
    __m128i vSumAbsGy    = _mm_loadu_si128((__m128i *)&sumAbsGySample32bit[idx]);
    __m128i vSumDIY      = _mm_loadu_si128((__m128i *)&sumDIYSample32bit[idx]);
    __m128i vSumSignGyGx = _mm_loadu_si128((__m128i *)&sumSignGyGxSample32bit[idx]);

    // Compute log2D using leading zero count
    __m128i vConst32  = _mm_set1_epi32(32);
    __m128i vConst158 = _mm_set1_epi32(158);
    __m128i v;
    // prevent value from being rounded up to the next power of two
    v                 = _mm_andnot_si128(_mm_srli_epi32(vSumAbsGx, 8), vSumAbsGx); // keep 8 MSB
    v                 = _mm_castps_si128(_mm_cvtepi32_ps(v)); // convert an integer to float
    v                 = _mm_srli_epi32(v, 23); // shift down the exponent
    v                 = _mm_subs_epu16(vConst158, v); // undo bias
    v                 = _mm_min_epi16(v, vConst32); // clamp at 32
    __m128i vLog2D_Gx = _mm_sub_epi32(vConst31, v);
    v                 = _mm_andnot_si128(_mm_srli_epi32(vSumAbsGy, 8), vSumAbsGy); // keep 8 MSB
    v                 = _mm_castps_si128(_mm_cvtepi32_ps(v)); // convert an integer to float
    v                 = _mm_srli_epi32(v, 23); // shift down the exponent
    v                 = _mm_subs_epu16(vConst158, v); // undo bias
    v                 = _mm_min_epi16(v, vConst32); // clamp at 32
    __m128i vLog2D_Gy = _mm_sub_epi32(vConst31, v);

    // X direction calculations
    // Compute MSB ((x << 5) >> log2D) & 31
    __m128i vMsbD_Gx = _mm_and_si128(mm_srav_epi32_sse2(_mm_slli_epi32(vSumAbsGx, 5), vLog2D_Gx), vConst31);
    __m128i vInvD_Gx =
      _mm_set_epi32(divTable[_mm_extract_epi32(vMsbD_Gx, 3)], divTable[_mm_extract_epi32(vMsbD_Gx, 2)],
                    divTable[_mm_extract_epi32(vMsbD_Gx, 1)], divTable[_mm_extract_epi32(vMsbD_Gx, 0)]);
    __m128i vShiftD_Gx  = _mm_add_epi32(vLog2D_Gx, vConst5);
    __m128i vOffsetD_Gx = mm_sllv_epi32_sse2(vConst1, _mm_sub_epi32(vShiftD_Gx, vConst1));
    __m128i vTmpX =
      mm_srav_epi32_sse2(_mm_add_epi32(_mm_mullo_epi32(_mm_slli_epi32(vSumDIX, 3), vInvD_Gx), vOffsetD_Gx), vShiftD_Gx);
    vTmpX = _mm_min_epi32(_mm_max_epi32(vTmpX, vNegLimit), vLimit);
    vTmpX = _mm_blendv_epi8(vZero, vTmpX, _mm_cmpgt_epi32(vSumAbsGx, vZero));  // Apply condition (if sumAbsGx > 0)

    // Y direction calculations
    __m128i vMsbD_Gy = _mm_and_si128(mm_srav_epi32_sse2(_mm_slli_epi32(vSumAbsGy, 5), vLog2D_Gy), vConst31);
    __m128i vInvD_Gy =
      _mm_set_epi32(divTable[_mm_extract_epi32(vMsbD_Gy, 3)], divTable[_mm_extract_epi32(vMsbD_Gy, 2)],
                    divTable[_mm_extract_epi32(vMsbD_Gy, 1)], divTable[_mm_extract_epi32(vMsbD_Gy, 0)]);
    __m128i vShiftD_Gy  = _mm_add_epi32(vLog2D_Gy, vConst5);
    __m128i vOffsetD_Gy = mm_sllv_epi32_sse2(vConst1, _mm_sub_epi32(vShiftD_Gy, vConst1));

    __m128i vTmpY = mm_srav_epi32_sse2(
      _mm_add_epi32(
        _mm_mullo_epi32(
          _mm_sub_epi32(_mm_slli_epi32(vSumDIY, 3), _mm_srai_epi32(_mm_mullo_epi32(vTmpX, vSumSignGyGx), 1)), vInvD_Gy),
        vOffsetD_Gy),
      vShiftD_Gy);

    vTmpY = _mm_min_epi32(_mm_max_epi32(vTmpY, vNegLimit), vLimit);
    vTmpY = _mm_blendv_epi8(vZero, vTmpY, _mm_cmpgt_epi32(vSumAbsGy, vZero));  // Apply condition (if sumAbsGy > 0)

    // Store results
    _mm_storeu_si128((__m128i *)&tmpxSample32bit[idx], vTmpX);
    _mm_storeu_si128((__m128i *)&tmpySample32bit[idx], vTmpY);
  }
#endif
}

template<X86_VEXT vext> void addBIOAvgN_SSE(const Pel *src0, int src0Stride, const Pel *src1, int src1Stride, Pel *dst,
                                            int dstStride, const Pel *gradX0, const Pel *gradX1, const Pel *gradY0,
                                            const Pel *gradY1, int gradStride, int width, int height, int *tmpx,
                                            int *tmpy, int shift, int offset, const ClpRng &clpRng, bool *mcMask[2],
                                            int mcStride, bool *isOOB)
{
  if (isOOB[0] || isOOB[1])
  {
    int   b        = 0;
    int   offset2  = offset >> 1;
    int   shift2   = shift - 1;
    bool *pMcMask0 = mcMask[0];
    bool *pMcMask1 = mcMask[1];
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        b         = (int)tmpx[x] * (gradX0[x] - gradX1[x]) + (int)tmpy[x] * (gradY0[x] - gradY1[x]);
        bool oob0 = pMcMask0[x];
        bool oob1 = pMcMask1[x];
        if (oob0 && !oob1)
        {
          dst[x] = ClipPel(rightShift(src1[x] + offset2, shift2), clpRng);
        }
        else if (!oob0 && oob1)
        {
          dst[x] = ClipPel(rightShift(src0[x] + offset2, shift2), clpRng);
        }
        else
        {
          dst[x] = ClipPel(rightShift((src0[x] + src1[x] + b + offset), shift), clpRng);
        }
      }
      pMcMask0 += mcStride;
      pMcMask1 += mcStride;
      tmpx += width;
      tmpy += width;
      dst += dstStride;
      src0 += src0Stride;
      src1 += src1Stride;
      gradX0 += gradStride;
      gradX1 += gradStride;
      gradY0 += gradStride;
      gradY1 += gradStride;
    }
    return;
  }
  int       bb = 0;
  int       pX = 0, pY = 0;
  const int tt = 16;
#ifdef USE_AVX2
  __m128i vibdimin = _mm_set1_epi16(clpRng.min);
  __m128i vibdimax = _mm_set1_epi16(clpRng.max);
  __m128i var1     = _mm_setzero_si128();
  __m128i var2     = _mm_setzero_si128();

  if (width != 4)
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x = x + 8)
      {
        var1      = _mm_loadu_si128((const __m128i *)(gradX0 + x));
        var1      = _mm_sub_epi16(var1, _mm_loadu_si128((const __m128i *)(gradX1 + x)));
        var2      = _mm_loadu_si128((const __m128i *)(gradY0 + x));
        var2      = _mm_sub_epi16(var2, _mm_loadu_si128((const __m128i *)(gradY1 + x)));
        __m256i a = _mm256_inserti128_si256(_mm256_castsi128_si256(_mm_unpacklo_epi16(var1, var2)),
                                            _mm_unpackhi_epi16(var1, var2), 1);

        __m256i vClipTmpx256 = _mm256_loadu_si256((const __m256i *)(tmpx + x));
        __m256i vClipTmpy256 = _mm256_loadu_si256((const __m256i *)(tmpy + x));
        __m256i c            = _mm256_packs_epi32(_mm256_unpacklo_epi32(vClipTmpx256, vClipTmpy256),
                                                  _mm256_unpackhi_epi32(vClipTmpx256, vClipTmpy256));

        __m256i sum32 = _mm256_madd_epi16(a, c);   // 8 32bits

        var1 = _mm_loadu_si128((const __m128i *)(src0 + x));
        var2 = _mm_loadu_si128((const __m128i *)(src1 + x));
        a    = _mm256_inserti128_si256(_mm256_castsi128_si256(_mm_unpacklo_epi16(var1, var2)),
                                       _mm_unpackhi_epi16(var1, var2), 1);

        sum32         = _mm256_add_epi32(sum32, _mm256_madd_epi16(a, _mm256_set1_epi16(1)));
        sum32         = _mm256_add_epi32(sum32, _mm256_set1_epi32(offset));
        sum32         = _mm256_sra_epi32(sum32, _mm_cvtsi32_si128(shift));
        __m128i sum16 = _mm_packs_epi32(_mm256_castsi256_si128(sum32), _mm256_extractf128_si256(sum32, 1));
        sum16         = _mm_max_epi16(sum16, vibdimin);
        sum16         = _mm_min_epi16(sum16, vibdimax);
        _mm_storeu_si128((__m128i *)(dst + x), sum16);
      }
      for (int x = 0; x < width; x++)
      {
        pX = (tmpx[x] > tt) ? 1 : ((tmpx[x] < -tt) ? -1 : 0);
        pY = (tmpy[x] > tt) ? 1 : ((tmpy[x] < -tt) ? -1 : 0);
        if (pX == 0 && pY == 0)
        {
          continue;
        }
        int xX = tmpx[x] - 32 * pX;
        int yY = tmpy[x] - 32 * pY;
        bb     = (int)xX * (gradX0[x + pY * gradStride + pX] - gradX1[x - pY * gradStride - pX]) +
          (int)yY * (gradY0[x + pY * gradStride + pX] - gradY1[x - pY * gradStride - pX]);
        dst[x] = ClipPel(
          rightShift((src0[x + pY * src0Stride + pX] + src1[x - pY * src1Stride - pX] + bb + offset), shift), clpRng);
      }
      dst += dstStride;
      src0 += src0Stride;
      src1 += src1Stride;
      gradX0 += gradStride;
      gradX1 += gradStride;
      gradY0 += gradStride;
      gradY1 += gradStride;
      tmpx += width;
      tmpy += width;
    }
  }
  else   // (width == 4)
  {
    for (int y = 0; y < height; y++)
    {
      __m128i a =
        _mm_unpacklo_epi16(_mm_loadl_epi64((const __m128i *)gradX0), _mm_loadl_epi64((const __m128i *)gradY0));
      __m128i b =
        _mm_unpacklo_epi16(_mm_loadl_epi64((const __m128i *)gradX1), _mm_loadl_epi64((const __m128i *)gradY1));
      a = _mm_sub_epi16(a, b);

      __m128i vClipTmpx128 = _mm_loadu_si128((const __m128i *)tmpx);
      __m128i vClipTmpy128 = _mm_loadu_si128((const __m128i *)tmpy);
      __m128i c =
        _mm_packs_epi32(_mm_unpacklo_epi32(vClipTmpx128, vClipTmpy128), _mm_unpackhi_epi32(vClipTmpx128, vClipTmpy128));

      __m128i sum = _mm_madd_epi16(a, c);

      a   = _mm_unpacklo_epi16(_mm_loadl_epi64((const __m128i *)src0), _mm_loadl_epi64((const __m128i *)src1));
      sum = _mm_add_epi32(sum, _mm_madd_epi16(a, _mm_set1_epi16(1)));
      sum = _mm_add_epi32(sum, _mm_set1_epi32(offset));
      sum = _mm_sra_epi32(sum, _mm_cvtsi32_si128(shift));
      sum = _mm_packs_epi32(sum, sum);
      sum = _mm_max_epi16(sum, vibdimin);
      sum = _mm_min_epi16(sum, vibdimax);
      _mm_storel_epi64((__m128i *)dst, sum);
      for (int x = 0; x < width; x++)
      {
        pX = (tmpx[x] > tt) ? 1 : ((tmpx[x] < -tt) ? -1 : 0);
        pY = (tmpy[x] > tt) ? 1 : ((tmpy[x] < -tt) ? -1 : 0);
        if (pX == 0 && pY == 0)
        {
          continue;
        }
        int xX = tmpx[x] - 32 * pX;
        int yY = tmpy[x] - 32 * pY;
        bb     = (int)xX * (gradX0[x + pY * gradStride + pX] - gradX1[x - pY * gradStride - pX]) +
          (int)yY * (gradY0[x + pY * gradStride + pX] - gradY1[x - pY * gradStride - pX]);
        dst[x] = ClipPel(
          rightShift((src0[x + pY * src0Stride + pX] + src1[x - pY * src1Stride - pX] + bb + offset), shift), clpRng);
      }
      dst += dstStride;
      src0 += src0Stride;
      src1 += src1Stride;
      gradX0 += gradStride;
      gradX1 += gradStride;
      gradY0 += gradStride;
      gradY1 += gradStride;
      tmpx += width;
      tmpy += width;
    }
  }
#else
  __m128i vibdimin  = _mm_set1_epi16(clpRng.min);
  __m128i vibdimax  = _mm_set1_epi16(clpRng.max);
  __m128i vClipTmpx = _mm_setzero_si128();
  __m128i vClipTmpy = _mm_setzero_si128();

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x += 4)
    {
      __m128i a = _mm_unpacklo_epi16(_mm_loadl_epi64((const __m128i *)(gradX0 + x)),
                                     _mm_loadl_epi64((const __m128i *)(gradY0 + x)));
      __m128i b = _mm_unpacklo_epi16(_mm_loadl_epi64((const __m128i *)(gradX1 + x)),
                                     _mm_loadl_epi64((const __m128i *)(gradY1 + x)));
      a         = _mm_sub_epi16(a, b);

      vClipTmpx   = _mm_loadu_si128((const __m128i *)(tmpx + x));
      vClipTmpy   = _mm_loadu_si128((const __m128i *)(tmpy + x));
      __m128i c   = _mm_packs_epi32(_mm_unpacklo_epi32(vClipTmpx, vClipTmpy), _mm_unpackhi_epi32(vClipTmpx, vClipTmpy));
      __m128i sum = _mm_madd_epi16(a, c);

      a =
        _mm_unpacklo_epi16(_mm_loadl_epi64((const __m128i *)(src0 + x)), _mm_loadl_epi64((const __m128i *)(src1 + x)));
      sum = _mm_add_epi32(sum, _mm_madd_epi16(a, _mm_set1_epi16(1)));
      sum = _mm_add_epi32(sum, _mm_set1_epi32(offset));
      sum = _mm_sra_epi32(sum, _mm_cvtsi32_si128(shift));
      sum = _mm_packs_epi32(sum, sum);
      sum = _mm_max_epi16(sum, vibdimin);
      sum = _mm_min_epi16(sum, vibdimax);
      _mm_storel_epi64((__m128i *)(dst + x), sum);
    }
    for (int x = 0; x < width; x++)
    {
      pX = (tmpx[x] > tt) ? 1 : ((tmpx[x] < -tt) ? -1 : 0);
      pY = (tmpy[x] > tt) ? 1 : ((tmpy[x] < -tt) ? -1 : 0);
      if (pX == 0 && pY == 0)
      {
        continue;
      }
      int xX = tmpx[x] - 32 * pX;
      int yY = tmpy[x] - 32 * pY;
      bb     = (int)xX * (gradX0[x + pY * gradStride + pX] - gradX1[x - pY * gradStride - pX]) +
        (int)yY * (gradY0[x + pY * gradStride + pX] - gradY1[x - pY * gradStride - pX]);
      dst[x] = ClipPel(
        rightShift((src0[x + pY * src0Stride + pX] + src1[x - pY * src1Stride - pX] + bb + offset), shift), clpRng);
    }

    dst += dstStride;
    src0 += src0Stride;
    src1 += src1Stride;
    gradX0 += gradStride;
    gradX1 += gradStride;
    gradY0 += gradStride;
    gradY1 += gradStride;
    tmpx += width;
    tmpy += width;
  }
#endif
}

template<X86_VEXT vext> void calAbsSum_SSE(const Pel *diff, int stride, int width, int height, int *absSum)
{
  __m128i vzero  = _mm_setzero_si128();
  __m128i vsum32 = vzero;
  __m128i vdiff  = vzero;

  if (width != 4)
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x = x + 8)
      {
        vdiff  = _mm_abs_epi16(_mm_loadu_si128((const __m128i *)(diff + x)));
        vsum32 = _mm_add_epi32(vsum32, _mm_unpacklo_epi16(vdiff, vzero));
        vsum32 = _mm_add_epi32(vsum32, _mm_unpackhi_epi16(vdiff, vzero));
      }
      diff += stride;
    }
  }
  else   // (width == 4)
  {
    const int strideDouble = (stride << 1);
    for (int y = 0; y < height; y += 2)
    {
      vdiff = _mm_abs_epi16(
        _mm_unpacklo_epi16(_mm_loadl_epi64((const __m128i *)diff), _mm_loadl_epi64((const __m128i *)(diff + stride))));
      vsum32 = _mm_add_epi32(vsum32, _mm_unpacklo_epi16(vdiff, vzero));
      vsum32 = _mm_add_epi32(vsum32, _mm_unpackhi_epi16(vdiff, vzero));
      diff += strideDouble;
    }
  }

  vsum32  = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
  vsum32  = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
  *absSum = _mm_cvtsi128_si32(vsum32);
}
template<X86_VEXT vext> void applyPROF_SSE(Pel *dstPel, ptrdiff_t dstStride, const Pel *srcPel, ptrdiff_t srcStride,
                                           int width, int height, const Pel *gradX, const Pel *gradY,
                                           ptrdiff_t gradStride, const int *dMvX, const int *dMvY, ptrdiff_t dMvStride,
                                           const bool bi, int shiftNum, Pel offset, const ClpRng &clpRng)
{
  CHECKD((width & 3), "block width error!");

  const int dILimit = 1 << std::max<int>(clpRng.bd + 1, 13);

#ifdef USE_AVX2
  __m256i mm_dmvx, mm_dmvy, mm_gradx, mm_grady, mm_dI, mm_dI0, mm_src;
  __m256i mm_offset = _mm256_set1_epi16(offset);
  __m256i vibdimin  = _mm256_set1_epi16(clpRng.min);
  __m256i vibdimax  = _mm256_set1_epi16(clpRng.max);
  __m256i mm_dimin  = _mm256_set1_epi32(-dILimit);
  __m256i mm_dimax  = _mm256_set1_epi32(dILimit - 1);
#else
  __m128i mm_dmvx, mm_dmvy, mm_gradx, mm_grady, mm_dI, mm_dI0;
  __m128i mm_offset = _mm_set1_epi16(offset);
  __m128i vibdimin  = _mm_set1_epi16(clpRng.min);
  __m128i vibdimax  = _mm_set1_epi16(clpRng.max);
  __m128i mm_dimin  = _mm_set1_epi32(-dILimit);
  __m128i mm_dimax  = _mm_set1_epi32(dILimit - 1);
#endif

#if USE_AVX2
  for (int h = 0; h < height; h += 4)
#else
  for (int h = 0; h < height; h += 2)
#endif
  {
    const int *vX  = dMvX;
    const int *vY  = dMvY;
    const Pel *gX  = gradX;
    const Pel *gY  = gradY;
    const Pel *src = srcPel;
    Pel       *dst = dstPel;

    for (int w = 0; w < width; w += 4)
    {
#if USE_AVX2
      const int *vX0 = vX, *vY0 = vY;
      const Pel *gX0 = gX, *gY0 = gY;

      // first two rows
      mm_dmvx  = _mm256_inserti128_si256(_mm256_castsi128_si256(_mm_loadu_si128((const __m128i *)vX0)),
                                         _mm_loadu_si128((const __m128i *)(vX0 + dMvStride)), 1);
      mm_dmvy  = _mm256_inserti128_si256(_mm256_castsi128_si256(_mm_loadu_si128((const __m128i *)vY0)),
                                         _mm_loadu_si128((const __m128i *)(vY0 + dMvStride)), 1);
      mm_gradx = _mm256_inserti128_si256(_mm256_castsi128_si256(_mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i *)gX0))),
                                         _mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i *)(gX0 + gradStride))), 1);
      mm_grady = _mm256_inserti128_si256(_mm256_castsi128_si256(_mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i *)gY0))),
                                         _mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i *)(gY0 + gradStride))), 1);
      mm_dI0   = _mm256_add_epi32(_mm256_mullo_epi32(mm_dmvx, mm_gradx), _mm256_mullo_epi32(mm_dmvy, mm_grady));
      mm_dI0   = _mm256_min_epi32(mm_dimax, _mm256_max_epi32(mm_dimin, mm_dI0));

      // next two rows
      vX0 += (dMvStride << 1);
      vY0 += (dMvStride << 1);
      gX0 += (gradStride << 1);
      gY0 += (gradStride << 1);
      mm_dmvx  = _mm256_inserti128_si256(_mm256_castsi128_si256(_mm_loadu_si128((const __m128i *)vX0)),
                                         _mm_loadu_si128((const __m128i *)(vX0 + dMvStride)), 1);
      mm_dmvy  = _mm256_inserti128_si256(_mm256_castsi128_si256(_mm_loadu_si128((const __m128i *)vY0)),
                                         _mm_loadu_si128((const __m128i *)(vY0 + dMvStride)), 1);
      mm_gradx = _mm256_inserti128_si256(_mm256_castsi128_si256(_mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i *)gX0))),
                                         _mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i *)(gX0 + gradStride))), 1);
      mm_grady = _mm256_inserti128_si256(_mm256_castsi128_si256(_mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i *)gY0))),
                                         _mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i *)(gY0 + gradStride))), 1);
      mm_dI    = _mm256_add_epi32(_mm256_mullo_epi32(mm_dmvx, mm_gradx), _mm256_mullo_epi32(mm_dmvy, mm_grady));
      mm_dI    = _mm256_min_epi32(mm_dimax, _mm256_max_epi32(mm_dimin, mm_dI));

      // combine four rows
      mm_dI           = _mm256_packs_epi32(mm_dI0, mm_dI);
      const Pel *src0 = src + srcStride;
      mm_src          = _mm256_inserti128_si256(
        _mm256_castsi128_si256(_mm_unpacklo_epi64(_mm_loadl_epi64((const __m128i *)src),
                                                           _mm_loadl_epi64((const __m128i *)(src + (srcStride << 1))))),
        _mm_unpacklo_epi64(_mm_loadl_epi64((const __m128i *)src0),
                                    _mm_loadl_epi64((const __m128i *)(src0 + (srcStride << 1)))),
        1);
      mm_dI = _mm256_add_epi16(mm_dI, mm_src);
      if (!bi)
      {
        mm_dI = _mm256_srai_epi16(_mm256_adds_epi16(mm_dI, mm_offset), shiftNum);
        mm_dI = _mm256_min_epi16(vibdimax, _mm256_max_epi16(vibdimin, mm_dI));
      }

      // store final results
      __m128i dITmp = _mm256_extractf128_si256(mm_dI, 1);
      Pel    *dst0  = dst;
      _mm_storel_epi64((__m128i *)dst0, _mm256_castsi256_si128(mm_dI));
      dst0 += dstStride;
      _mm_storel_epi64((__m128i *)dst0, dITmp);
      dst0 += dstStride;
      _mm_storel_epi64((__m128i *)dst0,
                       _mm_unpackhi_epi64(_mm256_castsi256_si128(mm_dI), _mm256_castsi256_si128(mm_dI)));
      dst0 += dstStride;
      _mm_storel_epi64((__m128i *)dst0, _mm_unpackhi_epi64(dITmp, dITmp));
#else
      // first row
      mm_dmvx  = _mm_loadu_si128((const __m128i *)vX);
      mm_dmvy  = _mm_loadu_si128((const __m128i *)vY);
      mm_gradx = _mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i *)gX));
      mm_grady = _mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i *)gY));
      mm_dI0   = _mm_add_epi32(_mm_mullo_epi32(mm_dmvx, mm_gradx), _mm_mullo_epi32(mm_dmvy, mm_grady));
      mm_dI0   = _mm_min_epi32(mm_dimax, _mm_max_epi32(mm_dimin, mm_dI0));

      // second row
      mm_dmvx  = _mm_loadu_si128((const __m128i *)(vX + dMvStride));
      mm_dmvy  = _mm_loadu_si128((const __m128i *)(vY + dMvStride));
      mm_gradx = _mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i *)(gX + gradStride)));
      mm_grady = _mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i *)(gY + gradStride)));
      mm_dI    = _mm_add_epi32(_mm_mullo_epi32(mm_dmvx, mm_gradx), _mm_mullo_epi32(mm_dmvy, mm_grady));
      mm_dI    = _mm_min_epi32(mm_dimax, _mm_max_epi32(mm_dimin, mm_dI));

      // combine both rows
      mm_dI = _mm_packs_epi32(mm_dI0, mm_dI);
      mm_dI = _mm_add_epi16(
        _mm_unpacklo_epi64(_mm_loadl_epi64((const __m128i *)src), _mm_loadl_epi64((const __m128i *)(src + srcStride))),
        mm_dI);
      if (!bi)
      {
        mm_dI = _mm_srai_epi16(_mm_adds_epi16(mm_dI, mm_offset), shiftNum);
        mm_dI = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, mm_dI));
      }

      _mm_storel_epi64((__m128i *)dst, mm_dI);
      _mm_storel_epi64((__m128i *)(dst + dstStride), _mm_unpackhi_epi64(mm_dI, mm_dI));
#endif
      vX += 4;
      vY += 4;
      gX += 4;
      gY += 4;
      src += 4;
      dst += 4;
    }

#if USE_AVX2
    dMvX += (dMvStride << 2);
    dMvY += (dMvStride << 2);
    gradX += (gradStride << 2);
    gradY += (gradStride << 2);
    srcPel += (srcStride << 2);
    dstPel += (dstStride << 2);
#else
    dMvX += (dMvStride << 1);
    dMvY += (dMvStride << 1);
    gradX += (gradStride << 1);
    gradY += (gradStride << 1);
    srcPel += (srcStride << 1);
    dstPel += (dstStride << 1);
#endif
  }
}

template<X86_VEXT vext> void roundIntVector_SIMD(int *v, int size, unsigned int nShift, const int dmvLimit)
{
  CHECKD(size % 8 != 0, "Size must be multiple of 8");
#ifdef USE_AVX2
  if (vext >= AVX2 && size >= 8)
  {
    __m256i dMvMin  = _mm256_set1_epi32(-dmvLimit);
    __m256i dMvMax  = _mm256_set1_epi32(dmvLimit);
    __m256i nOffset = _mm256_set1_epi32(1 << (nShift - 1));
    __m256i vzero   = _mm256_setzero_si256();
    for (int i = 0; i < size; i += 8, v += 8)
    {
      __m256i src = _mm256_lddqu_si256((__m256i *)v);
      __m256i of  = _mm256_cmpgt_epi32(src, vzero);
      __m256i dst = _mm256_srai_epi32(_mm256_add_epi32(_mm256_add_epi32(src, nOffset), of), nShift);
      dst         = _mm256_min_epi32(dMvMax, _mm256_max_epi32(dMvMin, dst));
      _mm256_storeu_si256((__m256i *)v, dst);
    }
  }
  else
#endif
  {
    __m128i dMvMin  = _mm_set1_epi32(-dmvLimit);
    __m128i dMvMax  = _mm_set1_epi32(dmvLimit);
    __m128i nOffset = _mm_set1_epi32((1 << (nShift - 1)));
    __m128i vzero   = _mm_setzero_si128();
    for (int i = 0; i < size; i += 4, v += 4)
    {
      __m128i src = _mm_loadu_si128((__m128i *)v);
      __m128i of  = _mm_cmpgt_epi32(src, vzero);
      __m128i dst = _mm_srai_epi32(_mm_add_epi32(_mm_add_epi32(src, nOffset), of), nShift);
      dst         = _mm_min_epi32(dMvMax, _mm_max_epi32(dMvMin, dst));
      _mm_storeu_si128((__m128i *)v, dst);
    }
  }
}

template<X86_VEXT vext> void gradFilter_SSE(Pel *src, ptrdiff_t srcStride, int width, int height, ptrdiff_t gradStride,
                                            Pel *gradX, Pel *gradY, const int bitDepth)
{
  Pel *srcTmp   = src + srcStride + 1;
  Pel *gradXTmp = gradX + gradStride + 1;
  Pel *gradYTmp = gradY + gradStride + 1;

  int     widthInside  = width - 2;
  int     heightInside = height - 2;
  int     shift1       = 6;
  __m128i mmShift1     = _mm_cvtsi32_si128(shift1);
  assert((widthInside & 3) == 0);

  if (widthInside > 4)
  {
    for (int y = 0; y < heightInside; y++)
    {
      int x = 0;
      for (; x < widthInside; x += 8)
      {
        __m128i mmPixTop    = _mm_sra_epi16(_mm_loadu_si128((__m128i *)(srcTmp + x - srcStride)), mmShift1);
        __m128i mmPixBottom = _mm_sra_epi16(_mm_loadu_si128((__m128i *)(srcTmp + x + srcStride)), mmShift1);
        __m128i mmPixLeft   = _mm_sra_epi16(_mm_loadu_si128((__m128i *)(srcTmp + x - 1)), mmShift1);
        __m128i mmPixRight  = _mm_sra_epi16(_mm_loadu_si128((__m128i *)(srcTmp + x + 1)), mmShift1);

        __m128i mmGradVer = _mm_sub_epi16(mmPixBottom, mmPixTop);
        __m128i mmGradHor = _mm_sub_epi16(mmPixRight, mmPixLeft);

        _mm_storeu_si128((__m128i *)(gradYTmp + x), mmGradVer);
        _mm_storeu_si128((__m128i *)(gradXTmp + x), mmGradHor);
      }
      gradXTmp += gradStride;
      gradYTmp += gradStride;
      srcTmp += srcStride;
    }
  }
  else
  {
    __m128i mmPixTop = _mm_sra_epi16(
      _mm_unpacklo_epi64(_mm_loadl_epi64((__m128i *)(srcTmp - srcStride)), _mm_loadl_epi64((__m128i *)(srcTmp))),
      mmShift1);
    for (int y = 0; y < heightInside; y += 2)
    {
      __m128i mmPixBottom = _mm_sra_epi16(_mm_unpacklo_epi64(_mm_loadl_epi64((__m128i *)(srcTmp + srcStride)),
                                                             _mm_loadl_epi64((__m128i *)(srcTmp + (srcStride << 1)))),
                                          mmShift1);
      __m128i mmPixLeft   = _mm_sra_epi16(_mm_unpacklo_epi64(_mm_loadl_epi64((__m128i *)(srcTmp - 1)),
                                                             _mm_loadl_epi64((__m128i *)(srcTmp - 1 + srcStride))),
                                          mmShift1);
      __m128i mmPixRight  = _mm_sra_epi16(_mm_unpacklo_epi64(_mm_loadl_epi64((__m128i *)(srcTmp + 1)),
                                                             _mm_loadl_epi64((__m128i *)(srcTmp + 1 + srcStride))),
                                          mmShift1);

      __m128i mmGradVer = _mm_sub_epi16(mmPixBottom, mmPixTop);
      __m128i mmGradHor = _mm_sub_epi16(mmPixRight, mmPixLeft);

      _mm_storel_epi64((__m128i *)gradYTmp, mmGradVer);
      _mm_storel_epi64((__m128i *)(gradYTmp + gradStride), _mm_unpackhi_epi64(mmGradVer, mmGradHor));
      _mm_storel_epi64((__m128i *)gradXTmp, mmGradHor);
      _mm_storel_epi64((__m128i *)(gradXTmp + gradStride), _mm_unpackhi_epi64(mmGradHor, mmGradVer));

      mmPixTop = mmPixBottom;
      gradXTmp += gradStride << 1;
      gradYTmp += gradStride << 1;
      srcTmp += srcStride << 1;
    }
  }
}

template<X86_VEXT vext, int W> void reco_SSE(const int16_t *src0, ptrdiff_t src0Stride, const int16_t *src1,
                                             ptrdiff_t src1Stride, int16_t *dst, ptrdiff_t dstStride, int width,
                                             int height, const ClpRng &clpRng)
{
  if (W == 8)
  {
    if (vext >= AVX2 && (width & 15) == 0)
    {
#if USE_AVX2
      __m256i vbdmin = _mm256_set1_epi16(clpRng.min);
      __m256i vbdmax = _mm256_set1_epi16(clpRng.max);

      for (int row = 0; row < height; row++)
      {
        for (int col = 0; col < width; col += 16)
        {
          __m256i vdest = _mm256_lddqu_si256((const __m256i *)&src0[col]);
          __m256i vsrc1 = _mm256_lddqu_si256((const __m256i *)&src1[col]);

          vdest = _mm256_adds_epi16(vdest, vsrc1);
          vdest = _mm256_min_epi16(vbdmax, _mm256_max_epi16(vbdmin, vdest));

          _mm256_storeu_si256((__m256i *)&dst[col], vdest);
        }

        src0 += src0Stride;
        src1 += src1Stride;
        dst += dstStride;
      }
#endif
    }
    else
    {
      __m128i vbdmin = _mm_set1_epi16(clpRng.min);
      __m128i vbdmax = _mm_set1_epi16(clpRng.max);

      for (int row = 0; row < height; row++)
      {
        for (int col = 0; col < width; col += 8)
        {
          __m128i vdest = _mm_loadu_si128((const __m128i *)&src0[col]);
          __m128i vsrc1 = _mm_loadu_si128((const __m128i *)&src1[col]);

          vdest = _mm_adds_epi16(vdest, vsrc1);
          vdest = _mm_min_epi16(vbdmax, _mm_max_epi16(vbdmin, vdest));

          _mm_storeu_si128((__m128i *)&dst[col], vdest);
        }

        src0 += src0Stride;
        src1 += src1Stride;
        dst += dstStride;
      }
    }
  }
  else if (W == 4)
  {
    __m128i vbdmin = _mm_set1_epi16(clpRng.min);
    __m128i vbdmax = _mm_set1_epi16(clpRng.max);

    for (int row = 0; row < height; row++)
    {
      for (int col = 0; col < width; col += 4)
      {
        __m128i vsrc = _mm_loadl_epi64((const __m128i *)&src0[col]);
        __m128i vdst = _mm_loadl_epi64((const __m128i *)&src1[col]);

        vdst = _mm_adds_epi16(vdst, vsrc);
        vdst = _mm_min_epi16(vbdmax, _mm_max_epi16(vbdmin, vdst));

        _mm_storel_epi64((__m128i *)&dst[col], vdst);
      }

      src0 += src0Stride;
      src1 += src1Stride;
      dst += dstStride;
    }
  }
  else
  {
    THROW("Unsupported size");
  }
}

#if ENABLE_SIMD_OPT_BCW
template<X86_VEXT vext, int W> void removeWeightHighFreq_SSE(int16_t *src0, ptrdiff_t src0Stride, const int16_t *src1,
                                                             ptrdiff_t src1Stride, int width, int height, int bcwWeight,
                                                             const Pel minVal, const Pel maxVal)
{
  static_assert(W == 4 || W == 8, "W must be 4 or 8");

  const int32_t w =
    ((BCW_WEIGHT_BASE << BCW_INV_BITS) + (bcwWeight > 0 ? (bcwWeight >> 1) : -(bcwWeight >> 1))) / bcwWeight;

  if (W == 8)
  {
    for (int row = 0; row < height; row++)
    {
      for (int col = 0; col < width; col += 8)
      {
        const __m128i vsrc0 = _mm_loadu_si128((const __m128i *)&src0[col]);
        const __m128i vsrc1 = _mm_loadu_si128((const __m128i *)&src1[col]);

        const __m128i diff = _mm_sub_epi16(vsrc0, vsrc1);

        __m128i lo = _mm_cvtepi16_epi32(diff);
        lo         = _mm_mullo_epi32(lo, _mm_set1_epi32(w));
        lo         = _mm_add_epi32(lo, _mm_set1_epi32(1 << BCW_INV_BITS >> 1));
        lo         = _mm_srai_epi32(lo, BCW_INV_BITS);

        __m128i hi = _mm_cvtepi16_epi32(_mm_unpackhi_epi64(diff, diff));
        hi         = _mm_mullo_epi32(hi, _mm_set1_epi32(w));
        hi         = _mm_add_epi32(hi, _mm_set1_epi32(1 << BCW_INV_BITS >> 1));
        hi         = _mm_srai_epi32(hi, BCW_INV_BITS);

        __m128i res = _mm_packs_epi32(lo, hi);
        res         = _mm_add_epi16(res, vsrc1);
        res         = _mm_max_epi16(res, _mm_set1_epi16(minVal));
        res         = _mm_min_epi16(res, _mm_set1_epi16(maxVal));

        _mm_store_si128((__m128i *)&src0[col], res);
      }
      src0 += src0Stride;
      src1 += src1Stride;
    }
  }
  else if (W == 4)
  {
    for (int row = 0; row < height; row++)
    {
      const __m128i vsrc0 = _mm_loadl_epi64((const __m128i *)src0);
      const __m128i vsrc1 = _mm_loadl_epi64((const __m128i *)src1);

      const __m128i diff = _mm_sub_epi16(vsrc0, vsrc1);

      __m128i lo = _mm_cvtepi16_epi32(diff);
      lo         = _mm_mullo_epi32(lo, _mm_set1_epi32(w));
      lo         = _mm_add_epi32(lo, _mm_set1_epi32(1 << BCW_INV_BITS >> 1));
      lo         = _mm_srai_epi32(lo, BCW_INV_BITS);

      __m128i res = _mm_packs_epi32(lo, lo);
      res         = _mm_add_epi16(res, vsrc1);
      res         = _mm_max_epi16(res, _mm_set1_epi16(minVal));
      res         = _mm_min_epi16(res, _mm_set1_epi16(maxVal));

      _mm_storel_epi64((__m128i *)src0, res);

      src0 += src0Stride;
      src1 += src1Stride;
    }
  }
}

template<X86_VEXT vext, int W> void removeHighFreq_SSE(int16_t *src0, ptrdiff_t src0Stride, const int16_t *src1,
                                                       ptrdiff_t src1Stride, int width, int height)
{
  if (W == 8)
  {
    // TODO: AVX2 impl
    {
      for (int row = 0; row < height; row++)
      {
        for (int col = 0; col < width; col += 8)
        {
          __m128i vsrc0 = _mm_loadu_si128((const __m128i *)&src0[col]);
          __m128i vsrc1 = _mm_loadu_si128((const __m128i *)&src1[col]);

          vsrc0 = _mm_sub_epi16(_mm_slli_epi16(vsrc0, 1), vsrc1);
          _mm_store_si128((__m128i *)&src0[col], vsrc0);
        }

        src0 += src0Stride;
        src1 += src1Stride;
      }
    }
  }
  else if (W == 4)
  {
    CHECK(width != 4, "width must be 4");

    for (int row = 0; row < height; row += 2)
    {
      __m128i vsrc0   = _mm_loadl_epi64((const __m128i *)src0);
      __m128i vsrc1   = _mm_loadl_epi64((const __m128i *)src1);
      __m128i vsrc0_2 = _mm_loadl_epi64((const __m128i *)(src0 + src0Stride));
      __m128i vsrc1_2 = _mm_loadl_epi64((const __m128i *)(src1 + src1Stride));

      vsrc0 = _mm_unpacklo_epi64(vsrc0, vsrc0_2);
      vsrc1 = _mm_unpacklo_epi64(vsrc1, vsrc1_2);

      vsrc0 = _mm_sub_epi16(_mm_slli_epi16(vsrc0, 1), vsrc1);
      _mm_storel_epi64((__m128i *)src0, vsrc0);
      _mm_storel_epi64((__m128i *)(src0 + src0Stride), _mm_unpackhi_epi64(vsrc0, vsrc0));

      src0 += 2 * src0Stride;
      src1 += 2 * src1Stride;
    }
  }
  else
  {
    THROW("Unsupported size");
  }
}
#endif

template<bool doShift, bool shiftR, typename T> static inline void do_shift(T &vreg, int num);
#if USE_AVX2
template<> inline void do_shift<true, true, __m256i>(__m256i &vreg, int num)
{
  vreg = _mm256_sra_epi32(vreg, _mm_cvtsi32_si128(num));
}
template<> inline void do_shift<true, false, __m256i>(__m256i &vreg, int num)
{
  vreg = _mm256_sll_epi32(vreg, _mm_cvtsi32_si128(num));
}
template<> inline void do_shift<false, true, __m256i>(__m256i &vreg, int num) {}
template<> inline void do_shift<false, false, __m256i>(__m256i &vreg, int num) {}
#endif
template<> inline void do_shift<true, true, __m128i>(__m128i &vreg, int num)
{
  vreg = _mm_sra_epi32(vreg, _mm_cvtsi32_si128(num));
}
template<> inline void do_shift<true, false, __m128i>(__m128i &vreg, int num)
{
  vreg = _mm_sll_epi32(vreg, _mm_cvtsi32_si128(num));
}
template<> inline void do_shift<false, true, __m128i>(__m128i &vreg, int num) {}
template<> inline void do_shift<false, false, __m128i>(__m128i &vreg, int num) {}

template<bool mult, typename T> static inline void do_mult(T &vreg, T &vmult);
template<> inline void                             do_mult<false, __m128i>(__m128i &, __m128i &) {}
#if USE_AVX2
template<> inline void do_mult<false, __m256i>(__m256i &, __m256i &) {}
#endif
template<> inline void do_mult<true, __m128i>(__m128i &vreg, __m128i &vmult) { vreg = _mm_mullo_epi32(vreg, vmult); }
#if USE_AVX2
template<> inline void do_mult<true, __m256i>(__m256i &vreg, __m256i &vmult) { vreg = _mm256_mullo_epi32(vreg, vmult); }
#endif

template<bool add, typename T> static inline void do_add(T &vreg, T &vadd);
template<> inline void                            do_add<false, __m128i>(__m128i &, __m128i &) {}
#if USE_AVX2
template<> inline void do_add<false, __m256i>(__m256i &, __m256i &) {}
#endif
template<> inline void do_add<true, __m128i>(__m128i &vreg, __m128i &vadd) { vreg = _mm_add_epi32(vreg, vadd); }
#if USE_AVX2
template<> inline void do_add<true, __m256i>(__m256i &vreg, __m256i &vadd) { vreg = _mm256_add_epi32(vreg, vadd); }
#endif

template<bool clip, typename T> static inline void do_clip(T &vreg, T &vbdmin, T &vbdmax);
template<> inline void                             do_clip<false, __m128i>(__m128i &, __m128i &, __m128i &) {}
#if USE_AVX2
template<> inline void do_clip<false, __m256i>(__m256i &, __m256i &, __m256i &) {}
#endif
template<> inline void do_clip<true, __m128i>(__m128i &vreg, __m128i &vbdmin, __m128i &vbdmax)
{
  vreg = _mm_min_epi16(vbdmax, _mm_max_epi16(vbdmin, vreg));
}
#if USE_AVX2
template<> inline void do_clip<true, __m256i>(__m256i &vreg, __m256i &vbdmin, __m256i &vbdmax)
{
  vreg = _mm256_min_epi16(vbdmax, _mm256_max_epi16(vbdmin, vreg));
}
#endif

template<X86_VEXT vext, int W, bool doAdd, bool mult, bool doShift, bool shiftR, bool clip>
void linTf_SSE(const Pel *src, ptrdiff_t srcStride, Pel *dst, ptrdiff_t dstStride, int width, int height, int scale,
               int shift, int offset, const ClpRng &clpRng)
{
  if (vext >= AVX2 && (width & 7) == 0 && W == 8)
  {
#if USE_AVX2
    __m256i vzero   = _mm256_setzero_si256();
    __m256i vbdmin  = _mm256_set1_epi16(clpRng.min);
    __m256i vbdmax  = _mm256_set1_epi16(clpRng.max);
    __m256i voffset = _mm256_set1_epi32(offset);
    __m256i vscale  = _mm256_set1_epi32(scale);

    for (int row = 0; row < height; row++)
    {
      for (int col = 0; col < width; col += 8)
      {
        __m256i val;
        val = _mm256_cvtepi16_epi32(_mm_loadu_si128((const __m128i *)&src[col]));
        do_mult<mult, __m256i>(val, vscale);
        do_shift<doShift, shiftR, __m256i>(val, shift);
        do_add<doAdd, __m256i>(val, voffset);
        val = _mm256_packs_epi32(val, vzero);
        do_clip<clip, __m256i>(val, vbdmin, vbdmax);
        val = _mm256_permute4x64_epi64(val, (0 << 0) + (2 << 2) + (1 << 4) + (1 << 6));

        _mm_storeu_si128((__m128i *)&dst[col], _mm256_castsi256_si128(val));
      }

      src += srcStride;
      dst += dstStride;
    }
#endif
  }
  else
  {
    __m128i vzero   = _mm_setzero_si128();
    __m128i vbdmin  = _mm_set1_epi16(clpRng.min);
    __m128i vbdmax  = _mm_set1_epi16(clpRng.max);
    __m128i voffset = _mm_set1_epi32(offset);
    __m128i vscale  = _mm_set1_epi32(scale);

    for (int row = 0; row < height; row++)
    {
      for (int col = 0; col < width; col += 4)
      {
        __m128i val;
        val = _mm_loadl_epi64((const __m128i *)&src[col]);
        val = _mm_cvtepi16_epi32(val);
        do_mult<mult, __m128i>(val, vscale);
        do_shift<doShift, shiftR, __m128i>(val, shift);
        do_add<doAdd, __m128i>(val, voffset);
        val = _mm_packs_epi32(val, vzero);
        do_clip<clip, __m128i>(val, vbdmin, vbdmax);

        _mm_storel_epi64((__m128i *)&dst[col], val);
      }

      src += srcStride;
      dst += dstStride;
    }
  }
}

// clang-format off
template<X86_VEXT vext, int W>
void linTf_SSE_entry(const Pel *src, ptrdiff_t srcStride, Pel *dst, ptrdiff_t dstStride, int width, int height,
                     int scale, int shift, int offset, const ClpRng &clpRng, bool clip)
{
  int fn = ( offset == 0 ? 16 : 0 ) + ( scale == 1 ? 8 : 0 ) + ( shift == 0 ? 4 : 0 ) + ( shift < 0 ? 2 : 0 ) + ( !clip ? 1 : 0 );

  switch( fn )
  {
  case  0: linTf_SSE<vext, W, true,  true,  true,  true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case  1: linTf_SSE<vext, W, true,  true,  true,  true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case  2: linTf_SSE<vext, W, true,  true,  true,  false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case  3: linTf_SSE<vext, W, true,  true,  true,  false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case  4: linTf_SSE<vext, W, true,  true,  false, true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case  5: linTf_SSE<vext, W, true,  true,  false, true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case  6: linTf_SSE<vext, W, true,  true,  false, false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case  7: linTf_SSE<vext, W, true,  true,  false, false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case  8: linTf_SSE<vext, W, true,  false, true,  true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case  9: linTf_SSE<vext, W, true,  false, true,  true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 10: linTf_SSE<vext, W, true,  false, true,  false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 11: linTf_SSE<vext, W, true,  false, true,  false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 12: linTf_SSE<vext, W, true,  false, false, true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 13: linTf_SSE<vext, W, true,  false, false, true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 14: linTf_SSE<vext, W, true,  false, false, false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 15: linTf_SSE<vext, W, true,  false, false, false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 16: linTf_SSE<vext, W, false, true,  true,  true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 17: linTf_SSE<vext, W, false, true,  true,  true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 18: linTf_SSE<vext, W, false, true,  true,  false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 19: linTf_SSE<vext, W, false, true,  true,  false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 20: linTf_SSE<vext, W, false, true,  false, true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 21: linTf_SSE<vext, W, false, true,  false, true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 22: linTf_SSE<vext, W, false, true,  false, false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 23: linTf_SSE<vext, W, false, true,  false, false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 24: linTf_SSE<vext, W, false, false, true,  true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 25: linTf_SSE<vext, W, false, false, true,  true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 26: linTf_SSE<vext, W, false, false, true,  false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 27: linTf_SSE<vext, W, false, false, true,  false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 28: linTf_SSE<vext, W, false, false, false, true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 29: linTf_SSE<vext, W, false, false, false, true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 30: linTf_SSE<vext, W, false, false, false, false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 31: linTf_SSE<vext, W, false, false, false, false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  default:
    THROW( "Unknown parametrization of the linear transformation" );
    break;
  }
}
// clang-format on

template<X86_VEXT vext> bool isMvOOB_SSE(const Mv &rcMv, const Position pos, const Size size, const SPS *sps,
                                         const PPS *pps, bool *mcMask, bool *mcMaskChroma, bool lumaOnly,
                                         ChromaFormat componentID)
{
  int       chromaScale = getComponentScaleX(COMP_Cb, componentID);
  const int mvstep      = 1 << MV_FRACTIONAL_BITS_INTERNAL;
  const int mvstepHalf  = mvstep >> 1;

  int horMax = (((int)pps->m_picWidthInLumaSamples - 1) << MV_FRACTIONAL_BITS_INTERNAL) + mvstepHalf;
  int horMin = -mvstepHalf;
  int verMax = (((int)pps->m_picHeightInLumaSamples - 1) << MV_FRACTIONAL_BITS_INTERNAL) + mvstepHalf;
  int verMin = -mvstepHalf;

  int  offsetX = (pos.x << MV_FRACTIONAL_BITS_INTERNAL) + rcMv.getHor();
  int  offsetY = (pos.y << MV_FRACTIONAL_BITS_INTERNAL) + rcMv.getVer();
  bool isOOB   = false;
  if ((offsetX <= horMin) || ((offsetX + (size.width << MV_FRACTIONAL_BITS_INTERNAL) - 1) >= horMax) ||
      (offsetY <= verMin) || ((offsetY + (size.height << MV_FRACTIONAL_BITS_INTERNAL) - 1) >= verMax))
  {
    isOOB = true;
  }
  if (isOOB)
  {
    int     baseOffsetX = offsetX;
    bool   *pMcMask     = mcMask;
    __m128i mmMinusOne  = _mm_set1_epi32(-1);
    __m128i mmOne       = _mm_set1_epi32(1);
    __m128i mmMvStep    = _mm_set1_epi32(mvstep);
    __m128i mmMvStep1   = _mm_set1_epi32(mvstep << 2);

    __m128i mmOffsetX1 = _mm_set_epi32(baseOffsetX + (mvstep << 1) + mvstep, baseOffsetX + (mvstep << 1),
                                       baseOffsetX + mvstep, baseOffsetX);
    __m128i mmOffsetX, mmCheck0, mmCheck1, mmCheck2, mmCheck3, mmCheck;
    __m128i mmOffsetY = _mm_set1_epi32(offsetY);

    __m128i mmHorMin = _mm_set1_epi32(horMin);
    __m128i mmHorMax = _mm_set1_epi32(horMax);
    __m128i mmVerMin = _mm_set1_epi32(verMin);
    __m128i mmVerMax = _mm_set1_epi32(verMax);

    for (int y = 0; y < size.height; y++)
    {
      mmCheck2 = _mm_xor_si128(_mm_cmpgt_epi32(mmOffsetY, mmVerMin), mmMinusOne);
      mmCheck3 = _mm_xor_si128(_mm_cmplt_epi32(mmOffsetY, mmVerMax), mmMinusOne);
      mmCheck2 = _mm_or_si128(mmCheck2, mmCheck3);

      mmOffsetX = mmOffsetX1;
      for (int x = 0; x < size.width; x += 4)
      {
        mmCheck0 = _mm_xor_si128(_mm_cmpgt_epi32(mmOffsetX, mmHorMin), mmMinusOne);
        mmCheck1 = _mm_xor_si128(_mm_cmplt_epi32(mmOffsetX, mmHorMax), mmMinusOne);
        mmCheck  = _mm_or_si128(_mm_or_si128(mmCheck0, mmCheck1), mmCheck2);
        mmCheck  = _mm_add_epi32(_mm_xor_si128(mmCheck, mmMinusOne), mmOne);

        mmCheck               = _mm_packs_epi32(mmCheck, mmCheck);
        mmCheck               = _mm_packs_epi16(mmCheck, mmCheck);
        *(int *)(pMcMask + x) = _mm_cvtsi128_si32(mmCheck);

        mmOffsetX = _mm_add_epi32(mmOffsetX, mmMvStep1);
      }
      pMcMask += size.width;
      mmOffsetY = _mm_add_epi32(mmOffsetY, mmMvStep);
    }

    if (!lumaOnly)
    {
      bool *pMcMaskChroma  = mcMaskChroma;
      pMcMask              = mcMask;
      int     widthChroma  = (size.width) >> chromaScale;
      int     heightChroma = (size.height) >> chromaScale;
      int     widthLuma2   = size.width << chromaScale;
      __m128i mmShuffle = _mm_set_epi8(0xf, 0xd, 0xb, 0x9, 0x7, 0x5, 0x3, 0x1, 0xe, 0xc, 0xa, 0x8, 0x6, 0x4, 0x2, 0x0);
      __m128i mmMaskLuma;
      int     widthChromaMultiple8 = ((widthChroma >> 3) << 3);
      int     stepX = 8, stepX1 = (8 << chromaScale), stepX2 = (1 << chromaScale);
      for (int y = 0; y < heightChroma; y++)
      {
        int x = 0, x1 = 0;
        for (; x < widthChromaMultiple8; x += stepX, x1 += stepX1)
        {
          mmMaskLuma = _mm_lddqu_si128((__m128i const *)(pMcMask + x1));
          mmMaskLuma = _mm_shuffle_epi8(mmMaskLuma, mmShuffle);
          _mm_storel_epi64((__m128i *)(pMcMaskChroma + x), mmMaskLuma);
        }
        for (; x < widthChroma; x++, x1 += stepX2)
        {
          pMcMaskChroma[x] = pMcMask[x1];
        }
        pMcMaskChroma += widthChroma;
        pMcMask += widthLuma2;
      }
    }
  }
  else
  {
    bool *pMcMask = mcMask;
    memset(pMcMask, false, size.width * size.height);

    bool *pMcMaskChroma = mcMaskChroma;
    int   widthChroma   = (size.width) >> chromaScale;
    int   heightChroma  = (size.height) >> chromaScale;
    memset(pMcMaskChroma, false, widthChroma * heightChroma);
  }
  return isOOB;
}

template<X86_VEXT vext> bool isMvOOBSubBlk_SSE(const Mv &rcMv, const Position pos, const Size size, const SPS *sps,
                                               const PPS *pps, bool *mcMask, int mcStride, bool *mcMaskChroma,
                                               int mcCStride, bool lumaOnly, ChromaFormat componentID)
{
  int       chromaScale = getComponentScaleX(COMP_Cb, componentID);
  const int mvstep      = 1 << MV_FRACTIONAL_BITS_INTERNAL;
  const int mvstepHalf  = mvstep >> 1;

  int horMax = (((int)pps->m_picWidthInLumaSamples - 1) << MV_FRACTIONAL_BITS_INTERNAL) + mvstepHalf;
  int horMin = -mvstepHalf;
  int verMax = (((int)pps->m_picHeightInLumaSamples - 1) << MV_FRACTIONAL_BITS_INTERNAL) + mvstepHalf;
  int verMin = -mvstepHalf;

  int  offsetX = (pos.x << MV_FRACTIONAL_BITS_INTERNAL) + rcMv.getHor();
  int  offsetY = (pos.y << MV_FRACTIONAL_BITS_INTERNAL) + rcMv.getVer();
  bool isOOB   = false;
  if ((offsetX <= horMin) || ((offsetX + (size.width << MV_FRACTIONAL_BITS_INTERNAL) - 1) >= horMax) ||
      (offsetY <= verMin) || ((offsetY + (size.height << MV_FRACTIONAL_BITS_INTERNAL) - 1) >= verMax))
  {
    isOOB = true;
  }
  if (isOOB)
  {
    int     baseOffsetX = offsetX;
    bool   *pMcMask     = mcMask;
    __m128i mmMinusOne  = _mm_set1_epi32(-1);
    __m128i mmOne       = _mm_set1_epi32(1);
    __m128i mmMvStep    = _mm_set1_epi32(mvstep);
    __m128i mmMvStep1   = _mm_set1_epi32(mvstep << 2);

    __m128i mmOffsetX1 = _mm_set_epi32(baseOffsetX + (mvstep << 1) + mvstep, baseOffsetX + (mvstep << 1),
                                       baseOffsetX + mvstep, baseOffsetX);
    __m128i mmOffsetX, mmCheck0, mmCheck1, mmCheck2, mmCheck3, mmCheck;
    __m128i mmOffsetY = _mm_set1_epi32(offsetY);

    __m128i mmHorMin = _mm_set1_epi32(horMin);
    __m128i mmHorMax = _mm_set1_epi32(horMax);
    __m128i mmVerMin = _mm_set1_epi32(verMin);
    __m128i mmVerMax = _mm_set1_epi32(verMax);

    for (int y = 0; y < size.height; y++)
    {
      mmCheck2 = _mm_xor_si128(_mm_cmpgt_epi32(mmOffsetY, mmVerMin), mmMinusOne);
      mmCheck3 = _mm_xor_si128(_mm_cmplt_epi32(mmOffsetY, mmVerMax), mmMinusOne);
      mmCheck2 = _mm_or_si128(mmCheck2, mmCheck3);

      mmOffsetX = mmOffsetX1;
      for (int x = 0; x < size.width; x += 4)
      {
        mmCheck0 = _mm_xor_si128(_mm_cmpgt_epi32(mmOffsetX, mmHorMin), mmMinusOne);
        mmCheck1 = _mm_xor_si128(_mm_cmplt_epi32(mmOffsetX, mmHorMax), mmMinusOne);
        mmCheck  = _mm_or_si128(_mm_or_si128(mmCheck0, mmCheck1), mmCheck2);
        mmCheck  = _mm_add_epi32(_mm_xor_si128(mmCheck, mmMinusOne), mmOne);

        mmCheck               = _mm_packs_epi32(mmCheck, mmCheck);
        mmCheck               = _mm_packs_epi16(mmCheck, mmCheck);
        *(int *)(pMcMask + x) = _mm_cvtsi128_si32(mmCheck);

        mmOffsetX = _mm_add_epi32(mmOffsetX, mmMvStep1);
      }
      pMcMask += mcStride;
      mmOffsetY = _mm_add_epi32(mmOffsetY, mmMvStep);
    }

    if (!lumaOnly)
    {
      bool *pMcMaskChroma  = mcMaskChroma;
      pMcMask              = mcMask;
      int     widthChroma  = (size.width) >> chromaScale;
      int     heightChroma = (size.height) >> chromaScale;
      int     strideLuma2  = mcStride << chromaScale;
      __m128i mmShuffle = _mm_set_epi8(0xf, 0xd, 0xb, 0x9, 0x7, 0x5, 0x3, 0x1, 0xe, 0xc, 0xa, 0x8, 0x6, 0x4, 0x2, 0x0);
      __m128i mmMaskLuma;
      int     widthChromaMultiple8 = ((widthChroma >> 3) << 3);
      int     stepX = 8, stepX1 = (8 << chromaScale), stepX2 = (1 << chromaScale);
      for (int y = 0; y < heightChroma; y++)
      {
        int x = 0, x1 = 0;
        for (; x < widthChromaMultiple8; x += stepX, x1 += stepX1)
        {
          mmMaskLuma = _mm_lddqu_si128((__m128i const *)(pMcMask + x1));
          mmMaskLuma = _mm_shuffle_epi8(mmMaskLuma, mmShuffle);
          _mm_storel_epi64((__m128i *)(pMcMaskChroma + x), mmMaskLuma);
        }
        for (; x < widthChroma; x++, x1 += stepX2)
        {
          pMcMaskChroma[x] = pMcMask[x1];
        }
        pMcMaskChroma += mcCStride;
        pMcMask += strideLuma2;
      }
    }
  }
  else
  {
    bool *pMcMask = mcMask;
    for (int y = 0; y < size.height; y++)
    {
      memset(pMcMask, false, size.width);
      pMcMask += mcStride;
    }

    bool *pMcMaskChroma = mcMaskChroma;
    int   widthChroma   = (size.width) >> chromaScale;
    int   heightChroma  = (size.height) >> chromaScale;
    for (int y = 0; y < heightChroma; y++)
    {
      memset(pMcMaskChroma, false, widthChroma);
      pMcMaskChroma += mcCStride;
    }
  }
  return isOOB;
}

template<X86_VEXT vext>
void computeDeltaAndShiftCore_SSE(const Position posLT, Mv firstMv, std::vector<RMVFInfo> &mvpInfoVecOri)
{
#if USE_AVX2
  int quotient = int(mvpInfoVecOri.size()) / 8;
  int reminder = int(mvpInfoVecOri.size()) % 8;
  int tempHor[8];
  int tempVer[8];
  int tempposHor[8];
  int tempposVer[8];

  __m256i baseHor = _mm256_set1_epi32(firstMv.hor);
  __m256i baseVer = _mm256_set1_epi32(firstMv.ver);
  __m256i vibdmin = _mm256_set1_epi32(-RMVF_MV_RANGE);
  __m256i vibdmax = _mm256_set1_epi32(RMVF_MV_RANGE - 1);
  for (int i = 0; i < quotient; i++)
  {
    // mv hor
    __m256i mvHor  = _mm256_set_epi32(mvpInfoVecOri[(i << 3) + 7].mvp.hor, mvpInfoVecOri[(i << 3) + 6].mvp.hor,
                                      mvpInfoVecOri[(i << 3) + 5].mvp.hor, mvpInfoVecOri[(i << 3) + 4].mvp.hor,
                                      mvpInfoVecOri[(i << 3) + 3].mvp.hor, mvpInfoVecOri[(i << 3) + 2].mvp.hor,
                                      mvpInfoVecOri[(i << 3) + 1].mvp.hor, mvpInfoVecOri[(i << 3) + 0].mvp.hor);
    __m256i resHor = _mm256_sub_epi32(mvHor, baseHor);
    resHor         = _mm256_min_epi32(vibdmax, _mm256_max_epi32(vibdmin, resHor));
    _mm256_storeu_si256((__m256i *)&tempHor[0], resHor);

    // mv ver
    __m256i mvVer  = _mm256_set_epi32(mvpInfoVecOri[(i << 3) + 7].mvp.ver, mvpInfoVecOri[(i << 3) + 6].mvp.ver,
                                      mvpInfoVecOri[(i << 3) + 5].mvp.ver, mvpInfoVecOri[(i << 3) + 4].mvp.ver,
                                      mvpInfoVecOri[(i << 3) + 3].mvp.ver, mvpInfoVecOri[(i << 3) + 2].mvp.ver,
                                      mvpInfoVecOri[(i << 3) + 1].mvp.ver, mvpInfoVecOri[(i << 3) + 0].mvp.ver);
    __m256i resVer = _mm256_sub_epi32(mvVer, baseVer);
    resVer         = _mm256_min_epi32(vibdmax, _mm256_max_epi32(vibdmin, resVer));
    _mm256_storeu_si256((__m256i *)&tempVer[0], resVer);

    // pos x
    __m256i posHor    = _mm256_set_epi32(mvpInfoVecOri[(i << 3) + 7].pos.x, mvpInfoVecOri[(i << 3) + 6].pos.x,
                                         mvpInfoVecOri[(i << 3) + 5].pos.x, mvpInfoVecOri[(i << 3) + 4].pos.x,
                                         mvpInfoVecOri[(i << 3) + 3].pos.x, mvpInfoVecOri[(i << 3) + 2].pos.x,
                                         mvpInfoVecOri[(i << 3) + 1].pos.x, mvpInfoVecOri[(i << 3) + 0].pos.x);
    __m256i posHorLT  = _mm256_set1_epi32(posLT.x);
    __m256i resposHor = _mm256_sub_epi32(posHor, posHorLT);
    _mm256_storeu_si256((__m256i *)&tempposHor[0], resposHor);

    // pos y
    __m256i posVer    = _mm256_set_epi32(mvpInfoVecOri[(i << 3) + 7].pos.y, mvpInfoVecOri[(i << 3) + 6].pos.y,
                                         mvpInfoVecOri[(i << 3) + 5].pos.y, mvpInfoVecOri[(i << 3) + 4].pos.y,
                                         mvpInfoVecOri[(i << 3) + 3].pos.y, mvpInfoVecOri[(i << 3) + 2].pos.y,
                                         mvpInfoVecOri[(i << 3) + 1].pos.y, mvpInfoVecOri[(i << 3) + 0].pos.y);
    __m256i posVerLT  = _mm256_set1_epi32(posLT.y);
    __m256i resposVer = _mm256_sub_epi32(posVer, posVerLT);
    _mm256_storeu_si256((__m256i *)&tempposVer[0], resposVer);

    for (int j = 0; j < 8; j++)
    {
      mvpInfoVecOri[(i << 3) + j].mvp = Mv(tempHor[j], tempVer[j]);
      mvpInfoVecOri[(i << 3) + j].pos = Position(tempposHor[j], tempposVer[j]);
    }
  }
  for (int i = (quotient << 3); i < (quotient << 3) + reminder; i++)
  {
    mvpInfoVecOri[i].mvp.set(
      Clip3(-RMVF_MV_RANGE, RMVF_MV_RANGE - 1, mvpInfoVecOri[i].mvp.getHor() - firstMv.getHor()),
      Clip3(-RMVF_MV_RANGE, RMVF_MV_RANGE - 1, mvpInfoVecOri[i].mvp.getVer() - firstMv.getVer()));
    mvpInfoVecOri[i].pos = Position(mvpInfoVecOri[i].pos.x - posLT.x, mvpInfoVecOri[i].pos.y - posLT.y);
  }
#else
  int quotient = int(mvpInfoVecOri.size()) / 4;
  int reminder = int(mvpInfoVecOri.size()) % 4;
  int tempHor[4];
  int tempVer[4];
  int tempposHor[4];
  int tempposVer[4];

  __m128i baseHor = _mm_set1_epi32(firstMv.hor);
  __m128i baseVer = _mm_set1_epi32(firstMv.ver);
  __m128i vibdmin = _mm_set1_epi32(-RMVF_MV_RANGE);
  __m128i vibdmax = _mm_set1_epi32(RMVF_MV_RANGE - 1);
  for (int i = 0; i < quotient; i++)
  {
    // mv hor
    __m128i mvHor  = _mm_set_epi32(mvpInfoVecOri[(i << 2) + 3].mvp.hor, mvpInfoVecOri[(i << 2) + 2].mvp.hor,
                                   mvpInfoVecOri[(i << 2) + 1].mvp.hor, mvpInfoVecOri[(i << 2) + 0].mvp.hor);
    __m128i resHor = _mm_sub_epi32(mvHor, baseHor);
    resHor         = _mm_min_epi32(vibdmax, _mm_max_epi32(vibdmin, resHor));
    _mm_storeu_si128((__m128i *)&tempHor[0], resHor);

    // mv ver
    __m128i mvVer  = _mm_set_epi32(mvpInfoVecOri[(i << 2) + 3].mvp.ver, mvpInfoVecOri[(i << 2) + 2].mvp.ver,
                                   mvpInfoVecOri[(i << 2) + 1].mvp.ver, mvpInfoVecOri[(i << 2) + 0].mvp.ver);
    __m128i resVer = _mm_sub_epi32(mvVer, baseVer);
    resVer         = _mm_min_epi32(vibdmax, _mm_max_epi32(vibdmin, resVer));
    _mm_storeu_si128((__m128i *)&tempVer[0], resVer);

    // pos x
    __m128i posHor    = _mm_set_epi32(mvpInfoVecOri[(i << 2) + 3].pos.x, mvpInfoVecOri[(i << 2) + 2].pos.x,
                                      mvpInfoVecOri[(i << 2) + 1].pos.x, mvpInfoVecOri[(i << 2) + 0].pos.x);
    __m128i posHorLT  = _mm_set1_epi32(posLT.x);
    __m128i resposHor = _mm_sub_epi32(posHor, posHorLT);
    _mm_storeu_si128((__m128i *)&tempposHor[0], resposHor);

    // pos y
    __m128i posVer    = _mm_set_epi32(mvpInfoVecOri[(i << 2) + 3].pos.y, mvpInfoVecOri[(i << 2) + 2].pos.y,
                                      mvpInfoVecOri[(i << 2) + 1].pos.y, mvpInfoVecOri[(i << 2) + 0].pos.y);
    __m128i posVerLT  = _mm_set1_epi32(posLT.y);
    __m128i resposVer = _mm_sub_epi32(posVer, posVerLT);
    _mm_storeu_si128((__m128i *)&tempposVer[0], resposVer);

    for (int j = 0; j < 4; j++)
    {
      mvpInfoVecOri[(i << 2) + j].mvp = Mv(tempHor[j], tempVer[j]);
      mvpInfoVecOri[(i << 2) + j].pos = Position(tempposHor[j], tempposVer[j]);
    }
  }
  for (int i = (quotient << 2); i < (quotient << 2) + reminder; i++)
  {
    mvpInfoVecOri[i].mvp.set(
      Clip3(-RMVF_MV_RANGE, RMVF_MV_RANGE - 1, mvpInfoVecOri[i].mvp.getHor() - firstMv.getHor()),
      Clip3(-RMVF_MV_RANGE, RMVF_MV_RANGE - 1, mvpInfoVecOri[i].mvp.getVer() - firstMv.getVer()));
    mvpInfoVecOri[i].pos = Position(mvpInfoVecOri[i].pos.x - posLT.x, mvpInfoVecOri[i].pos.y - posLT.y);
  }
#endif
}
template<X86_VEXT vext> void computeDeltaAndShiftCoreAddi_SSE(const Position posLT, Mv firstMv,
                                                              std::vector<RMVFInfo> &mvpInfoVecOri,
                                                              std::vector<RMVFInfo> &mvpInfoVecRes)
{
#if USE_AVX2
  int quotient = int(mvpInfoVecOri.size()) / 8;
  int reminder = int(mvpInfoVecOri.size()) % 8;
  int tempHor[8];
  int tempVer[8];
  int tempposHor[8];
  int tempposVer[8];

  __m256i baseHor = _mm256_set1_epi32(firstMv.hor);
  __m256i baseVer = _mm256_set1_epi32(firstMv.ver);
  __m256i vibdmin = _mm256_set1_epi32(-RMVF_MV_RANGE);
  __m256i vibdmax = _mm256_set1_epi32(RMVF_MV_RANGE - 1);
  for (int i = 0; i < quotient; i++)
  {
    // mv hor
    __m256i mvHor  = _mm256_set_epi32(mvpInfoVecOri[(i << 3) + 7].mvp.hor, mvpInfoVecOri[(i << 3) + 6].mvp.hor,
                                      mvpInfoVecOri[(i << 3) + 5].mvp.hor, mvpInfoVecOri[(i << 3) + 4].mvp.hor,
                                      mvpInfoVecOri[(i << 3) + 3].mvp.hor, mvpInfoVecOri[(i << 3) + 2].mvp.hor,
                                      mvpInfoVecOri[(i << 3) + 1].mvp.hor, mvpInfoVecOri[(i << 3) + 0].mvp.hor);
    __m256i resHor = _mm256_sub_epi32(mvHor, baseHor);
    resHor         = _mm256_min_epi32(vibdmax, _mm256_max_epi32(vibdmin, resHor));
    _mm256_storeu_si256((__m256i *)&tempHor[0], resHor);

    // mv ver
    __m256i mvVer  = _mm256_set_epi32(mvpInfoVecOri[(i << 3) + 7].mvp.ver, mvpInfoVecOri[(i << 3) + 6].mvp.ver,
                                      mvpInfoVecOri[(i << 3) + 5].mvp.ver, mvpInfoVecOri[(i << 3) + 4].mvp.ver,
                                      mvpInfoVecOri[(i << 3) + 3].mvp.ver, mvpInfoVecOri[(i << 3) + 2].mvp.ver,
                                      mvpInfoVecOri[(i << 3) + 1].mvp.ver, mvpInfoVecOri[(i << 3) + 0].mvp.ver);
    __m256i resVer = _mm256_sub_epi32(mvVer, baseVer);
    resVer         = _mm256_min_epi32(vibdmax, _mm256_max_epi32(vibdmin, resVer));
    _mm256_storeu_si256((__m256i *)&tempVer[0], resVer);

    // pos x
    __m256i posHor    = _mm256_set_epi32(mvpInfoVecOri[(i << 3) + 7].pos.x, mvpInfoVecOri[(i << 3) + 6].pos.x,
                                         mvpInfoVecOri[(i << 3) + 5].pos.x, mvpInfoVecOri[(i << 3) + 4].pos.x,
                                         mvpInfoVecOri[(i << 3) + 3].pos.x, mvpInfoVecOri[(i << 3) + 2].pos.x,
                                         mvpInfoVecOri[(i << 3) + 1].pos.x, mvpInfoVecOri[(i << 3) + 0].pos.x);
    __m256i posHorLT  = _mm256_set1_epi32(posLT.x);
    __m256i resposHor = _mm256_sub_epi32(posHor, posHorLT);
    _mm256_storeu_si256((__m256i *)&tempposHor[0], resposHor);

    // pos y
    __m256i posVer    = _mm256_set_epi32(mvpInfoVecOri[(i << 3) + 7].pos.y, mvpInfoVecOri[(i << 3) + 6].pos.y,
                                         mvpInfoVecOri[(i << 3) + 5].pos.y, mvpInfoVecOri[(i << 3) + 4].pos.y,
                                         mvpInfoVecOri[(i << 3) + 3].pos.y, mvpInfoVecOri[(i << 3) + 2].pos.y,
                                         mvpInfoVecOri[(i << 3) + 1].pos.y, mvpInfoVecOri[(i << 3) + 0].pos.y);
    __m256i posVerLT  = _mm256_set1_epi32(posLT.y);
    __m256i resposVer = _mm256_sub_epi32(posVer, posVerLT);
    _mm256_storeu_si256((__m256i *)&tempposVer[0], resposVer);

    for (int j = 0; j < 8; j++)
    {
      mvpInfoVecRes.push_back(RMVFInfo(Mv(tempHor[j], tempVer[j]), Position(tempposHor[j], tempposVer[j]), -1));
    }
  }
  int oriSize = (int)mvpInfoVecRes.size();
  int offset  = (quotient << 3) - oriSize;
  for (int i = oriSize; i < oriSize + reminder; i++)
  {
    mvpInfoVecRes.push_back(
      RMVFInfo(Mv(Clip3(-RMVF_MV_RANGE, RMVF_MV_RANGE - 1, mvpInfoVecOri[i + offset].mvp.hor - firstMv.getHor()),
                  Clip3(-RMVF_MV_RANGE, RMVF_MV_RANGE - 1, mvpInfoVecOri[i + offset].mvp.ver - firstMv.getVer())),
               Position(mvpInfoVecOri[i + offset].pos.x - posLT.x, mvpInfoVecOri[i + offset].pos.y - posLT.y), -1));
  }
#else
  int quotient = int(mvpInfoVecOri.size()) / 4;
  int reminder = int(mvpInfoVecOri.size()) % 4;
  int tempHor[4];
  int tempVer[4];
  int tempposHor[4];
  int tempposVer[4];

  __m128i baseHor = _mm_set1_epi32(firstMv.hor);
  __m128i baseVer = _mm_set1_epi32(firstMv.ver);
  __m128i vibdmin = _mm_set1_epi32(-RMVF_MV_RANGE);
  __m128i vibdmax = _mm_set1_epi32(RMVF_MV_RANGE - 1);
  for (int i = 0; i < quotient; i++)
  {
    // mv hor
    __m128i mvHor  = _mm_set_epi32(mvpInfoVecOri[(i << 2) + 3].mvp.hor, mvpInfoVecOri[(i << 2) + 2].mvp.hor,
                                   mvpInfoVecOri[(i << 2) + 1].mvp.hor, mvpInfoVecOri[(i << 2) + 0].mvp.hor);
    __m128i resHor = _mm_sub_epi32(mvHor, baseHor);
    resHor         = _mm_min_epi32(vibdmax, _mm_max_epi32(vibdmin, resHor));
    _mm_storeu_si128((__m128i *)&tempHor[0], resHor);

    // mv ver
    __m128i mvVer  = _mm_set_epi32(mvpInfoVecOri[(i << 2) + 3].mvp.ver, mvpInfoVecOri[(i << 2) + 2].mvp.ver,
                                   mvpInfoVecOri[(i << 2) + 1].mvp.ver, mvpInfoVecOri[(i << 2) + 0].mvp.ver);
    __m128i resVer = _mm_sub_epi32(mvVer, baseVer);
    resVer         = _mm_min_epi32(vibdmax, _mm_max_epi32(vibdmin, resVer));
    _mm_storeu_si128((__m128i *)&tempVer[0], resVer);

    // pos x
    __m128i posHor    = _mm_set_epi32(mvpInfoVecOri[(i << 2) + 3].pos.x, mvpInfoVecOri[(i << 2) + 2].pos.x,
                                      mvpInfoVecOri[(i << 2) + 1].pos.x, mvpInfoVecOri[(i << 2) + 0].pos.x);
    __m128i posHorLT  = _mm_set1_epi32(posLT.x);
    __m128i resposHor = _mm_sub_epi32(posHor, posHorLT);
    _mm_storeu_si128((__m128i *)&tempposHor[0], resposHor);

    // pos y
    __m128i posVer    = _mm_set_epi32(mvpInfoVecOri[(i << 2) + 3].pos.y, mvpInfoVecOri[(i << 2) + 2].pos.y,
                                      mvpInfoVecOri[(i << 2) + 1].pos.y, mvpInfoVecOri[(i << 2) + 0].pos.y);
    __m128i posVerLT  = _mm_set1_epi32(posLT.y);
    __m128i resposVer = _mm_sub_epi32(posVer, posVerLT);
    _mm_storeu_si128((__m128i *)&tempposVer[0], resposVer);

    for (int j = 0; j < 4; j++)
    {
      mvpInfoVecRes.push_back(RMVFInfo(Mv(tempHor[j], tempVer[j]), Position(tempposHor[j], tempposVer[j]), -1));
    }
  }
  int oriSize = (int)mvpInfoVecRes.size();
  int offset  = (quotient << 2) - oriSize;
  for (int i = oriSize; i < oriSize + reminder; i++)
  {
    mvpInfoVecRes.push_back(
      RMVFInfo(Mv(Clip3(-RMVF_MV_RANGE, RMVF_MV_RANGE - 1, mvpInfoVecOri[i + offset].mvp.hor - firstMv.getHor()),
                  Clip3(-RMVF_MV_RANGE, RMVF_MV_RANGE - 1, mvpInfoVecOri[i + offset].mvp.ver - firstMv.getVer())),
               Position(mvpInfoVecOri[i + offset].pos.x - posLT.x, mvpInfoVecOri[i + offset].pos.y - posLT.y), -1));
  }
#endif
}
template<X86_VEXT vext> void buildRegressionMatrixCore_SSE(std::vector<RMVFInfo> &mvpInfoVecOriSrc,
                                                           int64_t sumbb[2][3][3], int64_t sumeb[2][3],
                                                           uint16_t addedSize)
{
#if USE_AVX2
  std::vector<RMVFInfo> mvpInfoVecOri;
  if (addedSize)
  {
    mvpInfoVecOri.assign(mvpInfoVecOriSrc.end() - addedSize, mvpInfoVecOriSrc.end());
  }
  else
  {
    mvpInfoVecOri.assign(mvpInfoVecOriSrc.begin(), mvpInfoVecOriSrc.end());
  }
  int iNum = (uint16_t)mvpInfoVecOri.size();

  int b[3];
  int e[2];
  int quotient = iNum / 8;
  int reminder = iNum % 8;
  for (int i = 0; i < quotient; i++)
  {
    // set e[]
    __m256i ezero =
      _mm256_set_epi32(mvpInfoVecOri[(i << 3) + 7].mvp.getHor(), mvpInfoVecOri[(i << 3) + 6].mvp.getHor(),
                       mvpInfoVecOri[(i << 3) + 5].mvp.getHor(), mvpInfoVecOri[(i << 3) + 4].mvp.getHor(),
                       mvpInfoVecOri[(i << 3) + 3].mvp.getHor(), mvpInfoVecOri[(i << 3) + 2].mvp.getHor(),
                       mvpInfoVecOri[(i << 3) + 1].mvp.getHor(), mvpInfoVecOri[(i << 3) + 0].mvp.getHor());
    __m256i eone = _mm256_set_epi32(mvpInfoVecOri[(i << 3) + 7].mvp.getVer(), mvpInfoVecOri[(i << 3) + 6].mvp.getVer(),
                                    mvpInfoVecOri[(i << 3) + 5].mvp.getVer(), mvpInfoVecOri[(i << 3) + 4].mvp.getVer(),
                                    mvpInfoVecOri[(i << 3) + 3].mvp.getVer(), mvpInfoVecOri[(i << 3) + 2].mvp.getVer(),
                                    mvpInfoVecOri[(i << 3) + 1].mvp.getVer(), mvpInfoVecOri[(i << 3) + 0].mvp.getVer());

    // set b[]
    __m256i bzero = _mm256_set_epi32(mvpInfoVecOri[(i << 3) + 7].pos.x, mvpInfoVecOri[(i << 3) + 6].pos.x,
                                     mvpInfoVecOri[(i << 3) + 5].pos.x, mvpInfoVecOri[(i << 3) + 4].pos.x,
                                     mvpInfoVecOri[(i << 3) + 3].pos.x, mvpInfoVecOri[(i << 3) + 2].pos.x,
                                     mvpInfoVecOri[(i << 3) + 1].pos.x, mvpInfoVecOri[(i << 3) + 0].pos.x);
    __m256i bone  = _mm256_set_epi32(mvpInfoVecOri[(i << 3) + 7].pos.y, mvpInfoVecOri[(i << 3) + 6].pos.y,
                                     mvpInfoVecOri[(i << 3) + 5].pos.y, mvpInfoVecOri[(i << 3) + 4].pos.y,
                                     mvpInfoVecOri[(i << 3) + 3].pos.y, mvpInfoVecOri[(i << 3) + 2].pos.y,
                                     mvpInfoVecOri[(i << 3) + 1].pos.y, mvpInfoVecOri[(i << 3) + 0].pos.y);

    // sumeb[][]
    __m256i tempRes00 = _mm256_mullo_epi32(ezero, bzero);
    __m256i tempRes01 = _mm256_mullo_epi32(ezero, bone);
    __m256i tempRes10 = _mm256_mullo_epi32(eone, bzero);
    __m256i tempRes11 = _mm256_mullo_epi32(eone, bone);

    __m256i vsum = _mm256_add_epi32(tempRes00, _mm256_srli_si256(tempRes00, 8));
    vsum         = _mm256_add_epi64(vsum, _mm256_srli_si256(vsum, 4));
    sumeb[0][0] +=
      _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(vsum, 1));

    vsum = _mm256_add_epi32(tempRes01, _mm256_srli_si256(tempRes01, 8));
    vsum = _mm256_add_epi64(vsum, _mm256_srli_si256(vsum, 4));
    sumeb[0][1] +=
      _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(vsum, 1));

    vsum = _mm256_add_epi32(tempRes10, _mm256_srli_si256(tempRes10, 8));
    vsum = _mm256_add_epi64(vsum, _mm256_srli_si256(vsum, 4));
    sumeb[1][0] +=
      _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(vsum, 1));

    vsum = _mm256_add_epi32(tempRes11, _mm256_srli_si256(tempRes11, 8));
    vsum = _mm256_add_epi64(vsum, _mm256_srli_si256(vsum, 4));
    sumeb[1][1] +=
      _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(vsum, 1));

    vsum = _mm256_add_epi32(ezero, _mm256_srli_si256(ezero, 8));
    vsum = _mm256_add_epi64(vsum, _mm256_srli_si256(vsum, 4));
    sumeb[0][2] +=
      _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(vsum, 1));

    vsum = _mm256_add_epi32(eone, _mm256_srli_si256(eone, 8));
    vsum = _mm256_add_epi64(vsum, _mm256_srli_si256(vsum, 4));
    sumeb[1][2] +=
      _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(vsum, 1));

    // sumbb[][][]
    tempRes00 = _mm256_mullo_epi32(bzero, bzero);
    tempRes01 = _mm256_mullo_epi32(bzero, bone);

    tempRes11 = _mm256_mullo_epi32(bone, bone);

    vsum = _mm256_add_epi32(tempRes00, _mm256_srli_si256(tempRes00, 8));
    vsum = _mm256_add_epi64(vsum, _mm256_srli_si256(vsum, 4));
    sumbb[0][0][0] +=
      _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(vsum, 1));

    vsum = _mm256_add_epi32(tempRes01, _mm256_srli_si256(tempRes01, 8));
    vsum = _mm256_add_epi64(vsum, _mm256_srli_si256(vsum, 4));
    sumbb[0][0][1] +=
      _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(vsum, 1));

    vsum = _mm256_add_epi32(tempRes11, _mm256_srli_si256(tempRes11, 8));
    vsum = _mm256_add_epi64(vsum, _mm256_srli_si256(vsum, 4));
    sumbb[0][1][1] +=
      _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(vsum, 1));

    vsum = _mm256_add_epi32(bzero, _mm256_srli_si256(bzero, 8));
    vsum = _mm256_add_epi64(vsum, _mm256_srli_si256(vsum, 4));
    sumbb[0][0][2] +=
      _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(vsum, 1));

    vsum = _mm256_add_epi32(bone, _mm256_srli_si256(bone, 8));
    vsum = _mm256_add_epi64(vsum, _mm256_srli_si256(vsum, 4));
    sumbb[0][1][2] +=
      _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum)) + _mm_cvtsi128_si32(_mm256_extracti128_si256(vsum, 1));
  }
  sumbb[1][0][0] = sumbb[0][0][0];
  sumbb[0][1][0] = sumbb[1][0][1] = sumbb[1][1][0] = sumbb[0][0][1];
  sumbb[1][1][1]                                   = sumbb[0][1][1];
  sumbb[1][0][2] = sumbb[1][2][0] = sumbb[0][2][0] = sumbb[0][0][2];
  sumbb[1][1][2] = sumbb[1][2][1] = sumbb[0][2][1] = sumbb[0][1][2];
  for (int ni = quotient << 3; ni < (quotient << 3) + reminder; ni++)   // for all neighbor PUs
  {
    // to avoid big values in matrix, it is better to use delta_x and delta_y value, ie.e. use the x,y with respect to
    // the top,left corner of current PU
    b[0] = mvpInfoVecOri[ni].pos.x;
    b[1] = mvpInfoVecOri[ni].pos.y;
    b[2] = 1;

    e[0] = mvpInfoVecOri[ni].mvp.getHor();
    e[1] = mvpInfoVecOri[ni].mvp.getVer();

    for (int c = 0; c < 2; c++)
    {
      for (int d = 0; d < 3; d++)
      {
        sumeb[c][d] += (e[c] * b[d]);
      }
      for (int d1 = 0; d1 < 3; d1++)
      {
        for (int d = 0; d < 3; d++)
        {
          sumbb[c][d1][d] += (b[d1] * b[d]);
        }
      }
    }
  }
  sumbb[1][2][2] = sumbb[0][2][2] = (int)mvpInfoVecOriSrc.size();
#else
  std::vector<RMVFInfo> mvpInfoVecOri;
  if (addedSize)
  {
    mvpInfoVecOri.assign(mvpInfoVecOriSrc.end() - addedSize, mvpInfoVecOriSrc.end());
  }
  else
  {
    mvpInfoVecOri.assign(mvpInfoVecOriSrc.begin(), mvpInfoVecOriSrc.end());
  }
  int iNum = (uint16_t)mvpInfoVecOri.size();
  int b[3];
  int e[2];
  int quotient = iNum / 4;
  int reminder = iNum % 4;
  for (int i = 0; i < quotient; i++)
  {
    // set e[]
    __m128i ezero = _mm_set_epi32(mvpInfoVecOri[(i << 2) + 3].mvp.getHor(), mvpInfoVecOri[(i << 2) + 2].mvp.getHor(),
                                  mvpInfoVecOri[(i << 2) + 1].mvp.getHor(), mvpInfoVecOri[(i << 2) + 0].mvp.getHor());
    __m128i eone  = _mm_set_epi32(mvpInfoVecOri[(i << 2) + 3].mvp.getVer(), mvpInfoVecOri[(i << 2) + 2].mvp.getVer(),
                                  mvpInfoVecOri[(i << 2) + 1].mvp.getVer(), mvpInfoVecOri[(i << 2) + 0].mvp.getVer());

    // set b[]
    __m128i bzero = _mm_set_epi32(mvpInfoVecOri[(i << 2) + 3].pos.x, mvpInfoVecOri[(i << 2) + 2].pos.x,
                                  mvpInfoVecOri[(i << 2) + 1].pos.x, mvpInfoVecOri[(i << 2) + 0].pos.x);
    __m128i bone  = _mm_set_epi32(mvpInfoVecOri[(i << 2) + 3].pos.y, mvpInfoVecOri[(i << 2) + 2].pos.y,
                                  mvpInfoVecOri[(i << 2) + 1].pos.y, mvpInfoVecOri[(i << 2) + 0].pos.y);

    // sumeb[][]
    __m128i tempRes00 = _mm_mullo_epi32(ezero, bzero);
    __m128i tempRes01 = _mm_mullo_epi32(ezero, bone);
    __m128i tempRes10 = _mm_mullo_epi32(eone, bzero);
    __m128i tempRes11 = _mm_mullo_epi32(eone, bone);

    __m128i vsum = _mm_add_epi32(tempRes00, _mm_srli_si128(tempRes00, 8));
    vsum         = _mm_add_epi32(vsum, _mm_srli_si128(vsum, 4));
    sumeb[0][0] += _mm_cvtsi128_si32(vsum);

    vsum = _mm_add_epi32(tempRes01, _mm_srli_si128(tempRes01, 8));
    vsum = _mm_add_epi32(vsum, _mm_srli_si128(vsum, 4));
    sumeb[0][1] += _mm_cvtsi128_si32(vsum);

    vsum = _mm_add_epi32(tempRes10, _mm_srli_si128(tempRes10, 8));
    vsum = _mm_add_epi32(vsum, _mm_srli_si128(vsum, 4));
    sumeb[1][0] += _mm_cvtsi128_si32(vsum);

    vsum = _mm_add_epi32(tempRes11, _mm_srli_si128(tempRes11, 8));
    vsum = _mm_add_epi32(vsum, _mm_srli_si128(vsum, 4));
    sumeb[1][1] += _mm_cvtsi128_si32(vsum);

    vsum = _mm_add_epi32(ezero, _mm_srli_si128(ezero, 8));
    vsum = _mm_add_epi32(vsum, _mm_srli_si128(vsum, 4));
    sumeb[0][2] += _mm_cvtsi128_si32(vsum);

    vsum = _mm_add_epi32(eone, _mm_srli_si128(eone, 8));
    vsum = _mm_add_epi32(vsum, _mm_srli_si128(vsum, 4));
    sumeb[1][2] += _mm_cvtsi128_si32(vsum);

    // sumbb[][][]
    tempRes00 = _mm_mullo_epi32(bzero, bzero);
    tempRes01 = _mm_mullo_epi32(bzero, bone);

    tempRes11 = _mm_mullo_epi32(bone, bone);

    vsum = _mm_add_epi32(tempRes00, _mm_srli_si128(tempRes00, 8));
    vsum = _mm_add_epi32(vsum, _mm_srli_si128(vsum, 4));
    sumbb[0][0][0] += _mm_cvtsi128_si32(vsum);

    vsum = _mm_add_epi32(tempRes01, _mm_srli_si128(tempRes01, 8));
    vsum = _mm_add_epi32(vsum, _mm_srli_si128(vsum, 4));
    sumbb[0][0][1] += _mm_cvtsi128_si32(vsum);

    vsum = _mm_add_epi32(tempRes11, _mm_srli_si128(tempRes11, 8));
    vsum = _mm_add_epi32(vsum, _mm_srli_si128(vsum, 4));
    sumbb[0][1][1] += _mm_cvtsi128_si32(vsum);

    vsum = _mm_add_epi32(bzero, _mm_srli_si128(bzero, 8));
    vsum = _mm_add_epi32(vsum, _mm_srli_si128(vsum, 4));
    sumbb[0][0][2] += _mm_cvtsi128_si32(vsum);

    vsum = _mm_add_epi32(bone, _mm_srli_si128(bone, 8));
    vsum = _mm_add_epi32(vsum, _mm_srli_si128(vsum, 4));
    sumbb[0][1][2] += _mm_cvtsi128_si32(vsum);
  }
  sumbb[1][0][0] = sumbb[0][0][0];
  sumbb[0][1][0] = sumbb[1][0][1] = sumbb[1][1][0] = sumbb[0][0][1];
  sumbb[1][1][1]                                   = sumbb[0][1][1];
  sumbb[1][0][2] = sumbb[1][2][0] = sumbb[0][2][0] = sumbb[0][0][2];
  sumbb[1][1][2] = sumbb[1][2][1] = sumbb[0][2][1] = sumbb[0][1][2];
  for (int ni = quotient << 2; ni < (quotient << 2) + reminder; ni++)// for all neighbor PUs
  {
    // to avoid big values in matrix, it is better to use delta_x and delta_y value, ie.e. use the x,y with respect to
    // the top,left corner of current PU
    b[0] = mvpInfoVecOri[ni].pos.x;
    b[1] = mvpInfoVecOri[ni].pos.y;
    b[2] = 1;

    e[0] = mvpInfoVecOri[ni].mvp.getHor();
    e[1] = mvpInfoVecOri[ni].mvp.getVer();

    for (int c = 0; c < 2; c++)
    {
      for (int d = 0; d < 3; d++)
      {
        sumeb[c][d] += (e[c] * b[d]);
      }
      for (int d1 = 0; d1 < 3; d1++)
      {
        for (int d = 0; d < 3; d++)
        {
          sumbb[c][d1][d] += (b[d1] * b[d]);
        }
      }
    }
  }
  sumbb[1][2][2] = sumbb[0][2][2] = (int)mvpInfoVecOriSrc.size();
#endif
}

template<X86_VEXT vext> void PelBufferOps::_initPelBufOpsX86()
{
  roundBD      = roundBD_SSE<vext>;
  weightedAvg  = weightedAvg_SSE<vext>;
  weightedAvg3 = weightedAvg3_SSE<vext>;

  addAvg8 = addAvg_SSE<vext, 8>;
  addAvg4 = addAvg_SSE<vext, 4>;

  toLast4                  = toLast_SSE<vext, 4>;
  toLast2                  = toLast_SSE<vext, 2>;
  licRemoveWeightHighFreq2 = licRemoveWeightHighFreq_SSE<vext, 2>;
  licRemoveWeightHighFreq4 = licRemoveWeightHighFreq_SSE<vext, 4>;

  copyClip  = copyClip_SSE<vext>;
  copyClip4 = copyClip_SSE<vext, 4>;
  copyClip8 = copyClip_SSE<vext, 8>;

  bioGradFilter                 = gradFilter_SSE<vext>;
  calcBIOParameter              = calcBIOParameter_SSE<vext>;
  calcBIOParameterHighPrecision = calcBIOParameterHighPrecision_SSE<vext>;
  calcBIOParamSum4HighPrecision = calcBIOParamSumHighPrecision_SSE<vext>;
  calcBIOParamSum5NOSIM4        = calcBIOParamSum5NOSIMCore_SSE<vext, 4>;
  calcBIOParamSum5NOSIM8        = calcBIOParamSum5NOSIMCore_SSE<vext, 8>;
  calcBIOParamSum5              = calcBIOParamSum5_SSE<vext>;
  calcBIOParamSum4              = calcBIOParamSum4_SSE<vext>;
  calcBIOClippedVxVy            = calcBIOClippedVxVy_SSE<vext>;
  addBIOAvgN                    = addBIOAvgN_SSE<vext>;
  calAbsSum                     = calAbsSum_SSE<vext>;

  copyBuffer = copyBufferSimd<vext>;
  reco8      = reco_SSE<vext, 8>;
  reco4      = reco_SSE<vext, 4>;

  linTf8 = linTf_SSE_entry<vext, 8>;
  linTf4 = linTf_SSE_entry<vext, 4>;

#if ENABLE_SIMD_OPT_BCW
  removeWeightHighFreq8 = removeWeightHighFreq_SSE<vext, 8>;
  removeWeightHighFreq4 = removeWeightHighFreq_SSE<vext, 4>;
  removeHighFreq8       = removeHighFreq_SSE<vext, 8>;
  removeHighFreq4       = removeHighFreq_SSE<vext, 4>;
#endif

  profGradFilter = gradFilter_SSE<vext>;
  applyPROF      = applyPROF_SSE<vext>;
  roundIntVector = roundIntVector_SIMD<vext>;

  mipMatrixMul_4_4 = mipMatrixMul_SSE<vext, 4, 4>;
  mipMatrixMul_8_4 = mipMatrixMul_SSE<vext, 8, 4>;
  mipMatrixMul_8_8 = mipMatrixMul_SSE<vext, 8, 8>;

  isMvOOB       = isMvOOB_SSE<vext>;
  isMvOOBSubBlk = isMvOOBSubBlk_SSE<vext>;
}

template void PelBufferOps::_initPelBufOpsX86<SIMDX86>();

#endif   // TARGET_SIMD_X86
#endif
//! \}
