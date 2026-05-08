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

#include "CommonDefX86.h"
#include "../Rom.h"
#include "../InterpolationFilter.h"

// #include "../ChromaFormat.h"

//! \ingroup CommonLib
//! \{

#ifdef TARGET_SIMD_X86

// ===========================
// Full-pel copy 8-bit/16-bit
// ===========================
template<bool isFirst, bool isLast> static void fullPelCopySSE(const ClpRng &clpRng, const Pel *src,
                                                               const ptrdiff_t srcStride, Pel *dst,
                                                               const ptrdiff_t dstStride, int width, int height)
{
  int     headroom         = IF_INTERNAL_FRAC_BITS(clpRng.bd);
  int     headroom_offset  = 1 << (headroom - 1);
  int     offset           = IF_INTERNAL_OFFS;
  __m128i voffset          = _mm_set1_epi16(offset);
  __m128i voffset_headroom = _mm_set1_epi16(headroom_offset);
  __m128i vheadroom        = _mm_cvtsi32_si128(headroom);

  __m128i vibdimin = _mm_set1_epi16(clpRng.min);
  __m128i vibdimax = _mm_set1_epi16(clpRng.max);
  __m128i vsrc, vsum;

  for (int row = 0; row < height; row++)
  {
    _mm_prefetch((const char *)src + 2 * srcStride, _MM_HINT_T0);
    _mm_prefetch((const char *)src + (width >> 1) + 2 * srcStride, _MM_HINT_T0);
    _mm_prefetch((const char *)src + width - 1 + 2 * srcStride, _MM_HINT_T0);

    for (int col = 0; col < width; col += 8)
    {
      vsrc = _mm_loadu_si128((__m128i const *)&src[col]);

      if (isFirst == isLast)
      {
        vsum = vsrc;
      }
      else if (isFirst)
      {
        vsrc = _mm_sll_epi16(vsrc, vheadroom);
        vsum = _mm_sub_epi16(vsrc, voffset);
      }
      else
      {
        vsrc = _mm_add_epi16(vsrc, voffset);
        vsrc = _mm_add_epi16(vsrc, voffset_headroom);
        vsrc = _mm_sra_epi16(vsrc, vheadroom);
        vsum = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsrc));
      }

      _mm_storeu_si128((__m128i *)&dst[col], vsum);
    }
    src += srcStride;
    dst += dstStride;
  }
}

template<bool isFirst, bool isLast> static void fullPelCopyAVX2(const ClpRng &clpRng, const Pel *src,
                                                                const ptrdiff_t srcStride, Pel *dst,
                                                                const ptrdiff_t dstStride, int width, int height)
{
#ifdef USE_AVX2
  int headroom        = IF_INTERNAL_FRAC_BITS(clpRng.bd);
  int offset          = 1 << (headroom - 1);
  int internal_offset = IF_INTERNAL_OFFS;

  __m256i vinternal_offset = _mm256_set1_epi16(internal_offset);
  __m256i vheadroom_offset = _mm256_set1_epi16(offset);
  __m128i vheadroom        = _mm_cvtsi32_si128(headroom);
  __m256i vibdimin         = _mm256_set1_epi16(clpRng.min);
  __m256i vibdimax         = _mm256_set1_epi16(clpRng.max);
  __m256i vsrc, vsum;

  for (int row = 0; row < height; row++)
  {
    _mm_prefetch((const char *)(src + 3 * srcStride), _MM_HINT_T0);
    _mm_prefetch((const char *)(src + (width >> 1) + 3 * srcStride), _MM_HINT_T0);
    _mm_prefetch((const char *)(src + width - 1 + 3 * srcStride), _MM_HINT_T0);

    for (int col = 0; col < width; col += 16)
    {
      vsrc = _mm256_loadu_si256((const __m256i *)&src[col]);

      if (isFirst == isLast)
      {
        vsum = vsrc;
      }
      else if (isFirst)
      {
        vsrc = _mm256_sll_epi16(vsrc, vheadroom);
        vsum = _mm256_sub_epi16(vsrc, vinternal_offset);
      }
      else
      {
        vsrc = _mm256_add_epi16(vsrc, vinternal_offset);
        vsrc = _mm256_add_epi16(vsrc, vheadroom_offset);
        vsrc = _mm256_sra_epi16(vsrc, vheadroom);
        vsum = _mm256_min_epi16(vibdimax, _mm256_max_epi16(vibdimin, vsrc));
      }

      _mm256_storeu_si256((__m256i *)&dst[col], vsum);
    }
    src += srcStride;
    dst += dstStride;
  }
#endif
}

template<X86_VEXT vext, bool isFirst, bool isLast>
static void simdFilterCopy(const ClpRng &clpRng, const Pel *src, const ptrdiff_t srcStride, int16_t *dst,
                           const ptrdiff_t dstStride, int width, int height, bool biMCForDMVR)
{
  if (!biMCForDMVR && IF_INTERNAL_FRAC_BITS(clpRng.bd) >= 0)
  {
#ifdef USE_AVX2
    if (width >= 16)
    {
      fullPelCopyAVX2<isFirst, isLast>(clpRng, src, srcStride, dst, dstStride, width & ~15, height);

      src += width & ~15;
      dst += width & ~15;
      width &= 15;
    }

#endif
    if (width >= 8)
    {
      fullPelCopySSE<isFirst, isLast>(clpRng, src, srcStride, dst, dstStride, width & ~7, height);

      src += width & ~7;
      dst += width & ~7;
      width &= 7;
    }
  }

  if (width > 0)
  {
    InterpolationFilter::filterCopy<isFirst, isLast>(clpRng, src, srcStride, dst, dstStride, width, height,
                                                     biMCForDMVR);
  }
}

#if IF_12TAP_SIMD
#if SIMD_4x4_12
template<bool isLast> static void simdInterpolate4x4_12tap(const ClpRng &clpRng, const Pel *src, ptrdiff_t srcStride,
                                                           Pel *dst, ptrdiff_t dstStride, int width, int height,
                                                           const TFilterCoeff *coeffH, const TFilterCoeff *coeffV)
{
  src = src - 5 - 5 * srcStride; // for 12-tap filter

  int offset1st, offset2nd;
  int headRoom = std::max<int>(2, (IF_INTERNAL_PREC - clpRng.bd));
  int shift1st = IF_FILTER_PREC_8, shift2nd = IF_FILTER_PREC_8;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20

  if (isLast)
  {
    shift1st -= headRoom;
    shift2nd += headRoom;
    offset1st = -IF_INTERNAL_OFFS * (1 << shift1st);
    offset2nd = 1 << (shift2nd - 1);
    offset2nd += IF_INTERNAL_OFFS << IF_FILTER_PREC_8;
  }
  else
  {
    shift1st -= headRoom;
    offset1st = -IF_INTERNAL_OFFS * (1 << shift1st);
    offset2nd = 0;
  }

  __m128i xmm0 = _mm_lddqu_si128((__m128i const *)(coeffH)); // vcoeffh
  __m128i xmm1 = _mm_lddqu_si128((__m128i const *)(coeffH + 8)); // vcoeffl
  xmm1         = _mm_unpacklo_epi64(xmm1, xmm1);

  __m128i xmm13 = _mm_set1_epi32(offset1st);
  __m128i xmm14 = _mm_set_epi8(9, 8, 7, 6, 5, 4, 3, 2, 7, 6, 5, 4, 3, 2, 1, 0);
  __m128i xmm15 = _mm_set_epi8(13, 12, 11, 10, 9, 8, 7, 6, 11, 10, 9, 8, 7, 6, 5, 4);

  __m128i b[8]; // intermediate storage, 2x4 in line

  for (int i = 0; i < 15; i++)
  {
    __m128i xmm2 = _mm_lddqu_si128((__m128i const *)src);
    __m128i xmm3 = _mm_lddqu_si128((__m128i const *)(src + 8));

    __m128i xmm4 = _mm_shuffle_epi8(xmm3, xmm14);
    __m128i xmm5 = _mm_shuffle_epi8(xmm3, xmm15);

    __m128i xmm6 = _mm_alignr_epi8(xmm3, xmm2, 2);
    __m128i xmm7 = _mm_alignr_epi8(xmm3, xmm2, 4);
    __m128i xmm8 = _mm_alignr_epi8(xmm3, xmm2, 6);

    xmm2 = _mm_madd_epi16(xmm2, xmm0);
    xmm6 = _mm_madd_epi16(xmm6, xmm0);
    xmm7 = _mm_madd_epi16(xmm7, xmm0);
    xmm8 = _mm_madd_epi16(xmm8, xmm0);

    xmm4 = _mm_madd_epi16(xmm4, xmm1);
    xmm5 = _mm_madd_epi16(xmm5, xmm1);

    __m128i xmm9  = _mm_unpacklo_epi64(xmm2, xmm6);
    xmm2          = _mm_unpackhi_epi64(xmm2, xmm6);
    __m128i xmm10 = _mm_unpacklo_epi64(xmm7, xmm8);
    xmm7          = _mm_unpackhi_epi64(xmm7, xmm8);

    xmm2 = _mm_add_epi32(xmm2, xmm9);
    xmm7 = _mm_add_epi32(xmm7, xmm10);
    xmm2 = _mm_add_epi32(xmm2, xmm4);
    xmm7 = _mm_add_epi32(xmm7, xmm5);

    xmm2 = _mm_hadd_epi32(xmm2, xmm7);

    xmm2 = _mm_add_epi32(xmm2, xmm13);
    xmm2 = _mm_srai_epi32(xmm2, shift1st); // 4 16-bit packed to 32-bit

    if (i & 1)
    {
#if DIST_SSE_ENABLE
      xmm2 = _mm_slli_si128(xmm2, 2);
#else
      xmm2 = _mm_bslli_si128(xmm2, 2);
#endif
      b[(i - 1) >> 1] = _mm_blend_epi16(b[(i - 1) >> 1], xmm2, 0xAA);
    }
    else
    {
      b[i >> 1] = xmm2;
    }

    src += srcStride;
  }

  // vertical filter
  xmm0         = _mm_unpacklo_epi16(_mm_set1_epi16(coeffV[0]), _mm_set1_epi16(coeffV[1]));
  xmm1         = _mm_unpacklo_epi16(_mm_set1_epi16(coeffV[2]), _mm_set1_epi16(coeffV[3]));
  __m128i xmm2 = _mm_unpacklo_epi16(_mm_set1_epi16(coeffV[4]), _mm_set1_epi16(coeffV[5]));
  __m128i xmm3 = _mm_unpacklo_epi16(_mm_set1_epi16(coeffV[6]), _mm_set1_epi16(coeffV[7]));
  __m128i xmm4 = _mm_unpacklo_epi16(_mm_set1_epi16(coeffV[8]), _mm_set1_epi16(coeffV[9]));
  __m128i xmm5 = _mm_unpacklo_epi16(_mm_set1_epi16(coeffV[10]), _mm_set1_epi16(coeffV[11]));

  __m128i d[4];

  // even
  for (int i = 0; i < 2; i++)
  {

    __m128i xmm6  = _mm_madd_epi16(xmm0, b[i + 0]);
    __m128i xmm7  = _mm_madd_epi16(xmm1, b[i + 1]);
    __m128i xmm8  = _mm_madd_epi16(xmm2, b[i + 2]);
    __m128i xmm9  = _mm_madd_epi16(xmm3, b[i + 3]);
    __m128i xmm10 = _mm_madd_epi16(xmm4, b[i + 4]);
    __m128i xmm11 = _mm_madd_epi16(xmm5, b[i + 5]);

    xmm6  = _mm_add_epi32(xmm6, xmm7);
    xmm8  = _mm_add_epi32(xmm8, xmm9);
    xmm10 = _mm_add_epi32(xmm10, xmm11);

    xmm6      = _mm_add_epi32(xmm6, xmm8);
    d[i << 1] = _mm_add_epi32(xmm6, xmm10);
  }

  // odd
  __m128i xmm12 = _mm_set_epi8(1, 0, 3, 2, 1, 0, 3, 2, 1, 0, 3, 2, 1, 0, 3, 2);

  xmm0 = _mm_shuffle_epi8(xmm0, xmm12);
  xmm1 = _mm_shuffle_epi8(xmm1, xmm12);
  xmm2 = _mm_shuffle_epi8(xmm2, xmm12);
  xmm3 = _mm_shuffle_epi8(xmm3, xmm12);
  xmm4 = _mm_shuffle_epi8(xmm4, xmm12);
  xmm5 = _mm_shuffle_epi8(xmm5, xmm12);

  for (int j = 0; j < 7; j++)
  {
    b[j] = _mm_blend_epi16(b[j], b[j + 1], 0x55);
  }

  for (int i = 0; i < 2; i++)
  {

    __m128i xmm6  = _mm_madd_epi16(xmm0, b[i + 0]);
    __m128i xmm7  = _mm_madd_epi16(xmm1, b[i + 1]);
    __m128i xmm8  = _mm_madd_epi16(xmm2, b[i + 2]);
    __m128i xmm9  = _mm_madd_epi16(xmm3, b[i + 3]);
    __m128i xmm10 = _mm_madd_epi16(xmm4, b[i + 4]);
    __m128i xmm11 = _mm_madd_epi16(xmm5, b[i + 5]);

    xmm6  = _mm_add_epi32(xmm6, xmm7);
    xmm8  = _mm_add_epi32(xmm8, xmm9);
    xmm10 = _mm_add_epi32(xmm10, xmm11);

    xmm6            = _mm_add_epi32(xmm6, xmm8);
    d[(i << 1) + 1] = _mm_add_epi32(xmm6, xmm10);
  }

  // pack and save to destination
  xmm13 = _mm_set1_epi32(offset2nd);
  xmm12 = _mm_setzero_si128();

  __m128i vibdimin, vibdimax;
  if (isLast)
  {
    vibdimin = _mm_set1_epi16(clpRng.min);
    vibdimax = _mm_set1_epi16(clpRng.max);
  }

  for (int i = 0; i < 4; i++)
  {

    d[i] = _mm_add_epi32(d[i], xmm13);
    d[i] = _mm_srai_epi32(d[i], shift2nd);
    d[i] = _mm_packs_epi32(d[i], xmm12);

    if (isLast)
    {
      d[i] = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, d[i]));
    }

    _mm_storel_epi64((__m128i *)dst, d[i]);

    dst += dstStride;
  }
}
#endif

// SIMD interpolation horizontal, block width modulo 4
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorM4_12tap(const Pel *src, const ptrdiff_t srcStride, Pel *dst, const ptrdiff_t dstStride,
                                       int width, int height, int shift, int offset, const ClpRng &clpRng,
                                       Pel const *coeff)
{
  static_assert(N == 12, "only filter size 12 is supported");
  _mm_prefetch((const char *)src + srcStride, _MM_HINT_T0);

  const __m128i voffset  = _mm_set1_epi32(offset);
  const __m128i vibdimin = _mm_set1_epi16(clpRng.min);
  const __m128i vibdimax = _mm_set1_epi16(clpRng.max);
  const __m128i vcoeffh  = _mm_lddqu_si128((__m128i const *)coeff);
  const __m128i vcoeffl  = _mm_lddqu_si128((__m128i const *)(coeff + 8));

  const __m128i vshuf0 = _mm_set_epi8(9, 8, 7, 6, 5, 4, 3, 2, 7, 6, 5, 4, 3, 2, 1, 0);
  const __m128i vshuf1 = _mm_set_epi8(13, 12, 11, 10, 9, 8, 7, 6, 11, 10, 9, 8, 7, 6, 5, 4);

  for (int row = 0; row < height; row++)
  {
    _mm_prefetch((const char *)src + 2 * srcStride, _MM_HINT_T0);

    for (int col = 0; col < width; col += 4)
    {
      __m128i vsum = _mm_setzero_si128();

      __m128i       vsrc0 = _mm_lddqu_si128((__m128i const *)&src[col]);
      const __m128i vsrc4 = _mm_lddqu_si128((__m128i const *)&src[col + 8]);

      __m128i vtmp0 = _mm_shuffle_epi8(vsrc4, vshuf0);
      __m128i vtmp1 = _mm_shuffle_epi8(vsrc4, vshuf1);

      __m128i vsrc1 = _mm_alignr_epi8(vsrc4, vsrc0, 2);
      __m128i vsrc2 = _mm_alignr_epi8(vsrc4, vsrc0, 4);
      __m128i vsrc3 = _mm_alignr_epi8(vsrc4, vsrc0, 6);

      vsrc0 = _mm_madd_epi16(vsrc0, vcoeffh);
      vsrc1 = _mm_madd_epi16(vsrc1, vcoeffh);
      vsrc2 = _mm_madd_epi16(vsrc2, vcoeffh);
      vsrc3 = _mm_madd_epi16(vsrc3, vcoeffh);

      vtmp0 = _mm_madd_epi16(vtmp0, vcoeffl);
      vtmp1 = _mm_madd_epi16(vtmp1, vcoeffl);

      __m128i vtmp2 = _mm_unpacklo_epi64(vsrc0, vsrc1);
      vsrc0         = _mm_unpackhi_epi64(vsrc0, vsrc1);
      __m128i vtmp3 = _mm_unpacklo_epi64(vsrc2, vsrc3);
      vsrc2         = _mm_unpackhi_epi64(vsrc2, vsrc3);

      vsrc0 = _mm_add_epi32(vsrc0, vtmp2);
      vsrc2 = _mm_add_epi32(vsrc2, vtmp3);
      vsrc0 = _mm_add_epi32(vsrc0, vtmp0);
      vsrc2 = _mm_add_epi32(vsrc2, vtmp1);

      vsum = _mm_hadd_epi32(vsrc0, vsrc2);
      {
        vsum = _mm_add_epi32(vsum, voffset);
        vsum = _mm_srai_epi32(vsum, shift);

        const __m128i vzero = _mm_setzero_si128();
        vsum                = _mm_packs_epi32(vsum, vzero);
      }

      if (shiftBack)
      { // clip
        vsum = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsum));
      }
      _mm_storel_epi64((__m128i *)&dst[col], vsum);
    }
    src += srcStride;
    dst += dstStride;
  }
}

#if USE_AVX2
template<X86_VEXT vext, int N, bool shiftBack, int m>
inline void simdInterpolateHorMx_12tap(const Pel *src, ptrdiff_t srcStride, Pel *dst, ptrdiff_t dstStride, int width,
                                       int height, int shift, int offset, const ClpRng &clpRng, Pel const *coeff)
{
  static_assert(m == 8 || m == 16);

  static_assert(N == 12, "only filter size 12 is supported");

  constexpr auto xStep = m;
  constexpr auto yStep = m == 8 ? 2 : 1;

  const __m256i                  voffset  = _mm256_set1_epi32(offset);
  [[maybe_unused]] const __m256i vibdimin = _mm256_set1_epi16(clpRng.min);
  [[maybe_unused]] const __m256i vibdimax = _mm256_set1_epi16(clpRng.max);

  for (int row = 0; row < height; row += yStep)
  {
    _mm_prefetch((const char *)&src[yStep * srcStride], _MM_HINT_T0);
    if constexpr (yStep == 2)
    {
      _mm_prefetch((const char *)&src[3 * srcStride], _MM_HINT_T0);
    }

    if constexpr (yStep == 2)
    {
      if (row == height - 1)
      {
        dstStride = srcStride = 0;
      }
    }

    for (int col = 0; col < width; col += xStep)
    {
      __m256i sum[2] = { voffset, voffset };

      for (int dx = 0; dx < 2; ++dx)
      {
        for (int i = 0; i < 6; ++i)
        {
          __m256i s;
          if constexpr (yStep == 2)
          {
            s =
              _mm256_inserti128_si256(_mm256_castsi128_si256(_mm_loadu_si128((__m128i const *)&src[col + 2 * i + dx])),
                                      _mm_loadu_si128((__m128i const *)&src[col + 2 * i + srcStride + dx]), 1);
          }
          else
          {
            s = _mm256_loadu_si256((__m256i const *)&src[col + 2 * i + dx]);
          }

          const auto c = _mm256_set1_epi32((coeff[2 * i] & 0xffff) | (coeff[2 * i + 1] << 16)); // bubble?
          const auto p = _mm256_madd_epi16(s, c);
          sum[dx]      = _mm256_add_epi32(sum[dx], p);
        }
      }

      sum[0]    = _mm256_srai_epi32(sum[0], shift);
      sum[1]    = _mm256_sllv_epi32(sum[1], _mm256_set1_epi32(16 - shift));
      auto vsum = _mm256_blend_epi16(sum[0], sum[1], 0xaa);

      if constexpr (shiftBack)
      {
        vsum = _mm256_min_epi16(vibdimax, _mm256_max_epi16(vibdimin, vsum));
      }

      if constexpr (yStep == 2)
      {
        _mm_storeu_si128((__m128i *)&dst[col + 0 * dstStride], _mm256_extractf128_si256(vsum, 0));
        _mm_storeu_si128((__m128i *)&dst[col + 1 * dstStride], _mm256_extractf128_si256(vsum, 1));
      }
      else
      {
        _mm256_storeu_si256((__m256i *)&dst[col], vsum);
      }
    }
    src += yStep * srcStride;
    dst += yStep * dstStride;
  }
}
#endif

// SIMD interpolation horizontal, block not modulo of 4
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorNonM4_12tap(const Pel *src, ptrdiff_t srcStride, Pel *dst, ptrdiff_t dstStride, int width,
                                          int height, int shift, int offset, const ClpRng &clpRng, Pel const *coeff)
{
  static_assert(N == 12, "only filter size 12 is supported");
  _mm_prefetch((const char *)src + srcStride, _MM_HINT_T0);

  const __m128i voffset  = _mm_set1_epi32(offset);
  __m128i       vibdimin = _mm_set1_epi16(clpRng.min);
  __m128i       vibdimax = _mm_set1_epi16(clpRng.max);
  const __m128i vcoeffh  = _mm_loadu_si128((__m128i const *)coeff);
  const __m128i vcoeffl  = _mm_loadu_si128((__m128i const *)(coeff + 8));

  const __m128i vshuf0 = _mm_set_epi8(9, 8, 7, 6, 5, 4, 3, 2, 7, 6, 5, 4, 3, 2, 1, 0);
  const __m128i vshuf1 = _mm_set_epi8(13, 12, 11, 10, 9, 8, 7, 6, 11, 10, 9, 8, 7, 6, 5, 4);

  int        multiple   = (width >> 2) << 2;
  const Pel *initialSrc = src;
  Pel       *initialDst = dst;

  if (multiple)
  {
    for (int row = 0; row < height; row++)
    {
      _mm_prefetch((const char *)src + 2 * srcStride, _MM_HINT_T0);

      for (int col = 0; col < multiple; col += 4)
      {
        __m128i vsum = _mm_setzero_si128();

        __m128i       vsrc0 = _mm_loadu_si128((__m128i const *)&src[col]);
        const __m128i vsrc4 = _mm_loadu_si128((__m128i const *)&src[col + 8]);

        __m128i vtmp0 = _mm_shuffle_epi8(vsrc4, vshuf0);
        __m128i vtmp1 = _mm_shuffle_epi8(vsrc4, vshuf1);

        __m128i vsrc1 = _mm_alignr_epi8(vsrc4, vsrc0, 2);
        __m128i vsrc2 = _mm_alignr_epi8(vsrc4, vsrc0, 4);
        __m128i vsrc3 = _mm_alignr_epi8(vsrc4, vsrc0, 6);

        vsrc0 = _mm_madd_epi16(vsrc0, vcoeffh);
        vsrc1 = _mm_madd_epi16(vsrc1, vcoeffh);
        vsrc2 = _mm_madd_epi16(vsrc2, vcoeffh);
        vsrc3 = _mm_madd_epi16(vsrc3, vcoeffh);

        vtmp0 = _mm_madd_epi16(vtmp0, vcoeffl);
        vtmp1 = _mm_madd_epi16(vtmp1, vcoeffl);

        __m128i vtmp2 = _mm_unpacklo_epi64(vsrc0, vsrc1);
        vsrc0         = _mm_unpackhi_epi64(vsrc0, vsrc1);
        __m128i vtmp3 = _mm_unpacklo_epi64(vsrc2, vsrc3);
        vsrc2         = _mm_unpackhi_epi64(vsrc2, vsrc3);

        vsrc0 = _mm_add_epi32(vsrc0, vtmp2);
        vsrc2 = _mm_add_epi32(vsrc2, vtmp3);
        vsrc0 = _mm_add_epi32(vsrc0, vtmp0);
        vsrc2 = _mm_add_epi32(vsrc2, vtmp1);

        vsum = _mm_hadd_epi32(vsrc0, vsrc2);
        {
          vsum = _mm_add_epi32(vsum, voffset);
          vsum = _mm_srai_epi32(vsum, shift);

          const __m128i vzero = _mm_setzero_si128();
          vsum                = _mm_packs_epi32(vsum, vzero);
        }

        if (shiftBack)
        { // clip
          vsum = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsum));
        }
        _mm_storel_epi64((__m128i *)&dst[col], vsum);
      }
      src += srcStride;
      dst += dstStride;
    }
  }

  if (multiple == width)
  {
    return;
  }

  vibdimin = _mm_set1_epi32(clpRng.min);
  vibdimax = _mm_set1_epi32(clpRng.max);

  src = initialSrc;
  dst = initialDst;

  for (int row = 0; row < height; row++)
  {
    for (int col = multiple; col < width; col++)
    {
      __m128i vsrc0 = _mm_loadu_si128((__m128i const *)&src[col]);
      __m128i vsrc1 = _mm_loadl_epi64((__m128i const *)&src[col + 8]);
      vsrc0         = _mm_madd_epi16(vsrc0, vcoeffh);
      vsrc1         = _mm_madd_epi16(vsrc1, vcoeffl);

      vsrc0 = _mm_hadd_epi32(vsrc0, vsrc1);
      vsrc0 = _mm_hadd_epi32(vsrc0, vsrc0);
      vsrc0 = _mm_hadd_epi32(vsrc0, vsrc0);

      vsrc0 = _mm_add_epi32(vsrc0, voffset);
      vsrc0 = _mm_srai_epi32(vsrc0, shift);

      if (shiftBack)
      {   // clip
        vsrc0 = _mm_min_epi32(vibdimax, _mm_max_epi32(vibdimin, vsrc0));
      }

      dst[col] = _mm_cvtsi128_si32(vsrc0);

      dst += dstStride;
      src += srcStride;
    }
  }
#if 0

  src = initialSrc;
  dst = initialDst;

  for (int row = 0; row < height; row++)
  {
    for (int col = multiple; col < width; col++)
    {
      int sum = 0;

      sum  = src[col] * coeff[0];
      sum += src[col + 1] * coeff[1];
      sum += src[col + 2] * coeff[2];
      sum += src[col + 3] * coeff[3];
      sum += src[col + 4] * coeff[4];
      sum += src[col + 5] * coeff[5];
      sum += src[col + 6] * coeff[6];
      sum += src[col + 7] * coeff[7];
      sum += src[col + 8] * coeff[8];
      sum += src[col + 9] * coeff[9];
      sum += src[col + 10] * coeff[10];
      sum += src[col + 11] * coeff[11];

      Pel val = (sum + offset) >> shift;
      if (shiftBack)
      {
        val = ClipPel(val, clpRng);
      }
      CHECK( dst[col] != val, "Problem!" );
    }

    src += srcStride;
    dst += dstStride;
  }
#endif
}
#endif

// SIMD interpolation horizontal, block width=2, N=6
template<X86_VEXT vext, int N, bool CLAMP>
static void simdInterpolateHorM2(const int16_t *src, const ptrdiff_t srcStride, int16_t *dst, const ptrdiff_t dstStride,
                                 int height, int shift, int offset, const ClpRng &clpRng, int16_t const *coeff)
{
  static_assert(N == 6, "only filter sizes 6 supported");
  _mm_prefetch((const char *)(src + srcStride), _MM_HINT_T0);

  const __m128i minVal = _mm_set1_epi16(clpRng.min);
  const __m128i maxVal = _mm_set1_epi16(clpRng.max);

  __m128i c = _mm_loadu_si128((__m128i const *)coeff);

  __m128i coeffs[3];
  coeffs[2] = _mm_shuffle_epi32(c, 0xaa);
  coeffs[1] = _mm_shuffle_epi32(c, 0x55);
  coeffs[0] = _mm_shuffle_epi32(c, 0x00);
  coeffs[0] = _mm_shuffle_epi32(c, 0x44);
  coeffs[1] = _mm_shuffle_epi32(c, 0xaa);

  const __m128i shuffle0 = _mm_setr_epi8(0, 1, 2, 3, 4, 5, 6, 7, 2, 3, 4, 5, 6, 7, 8, 9);
  const __m128i shuffle2 = _mm_setr_epi8(8, 9, 10, 11, 10, 11, 12, 13, 8, 9, 10, 11, 10, 11, 12, 13);

  for (ptrdiff_t row = 0; row < height; row++)
  {
    _mm_prefetch((const char *)(src + (row + 2) * srcStride), _MM_HINT_T0);
    __m128i       vsum = _mm_set1_epi32(offset);
    const __m128i val  = _mm_loadu_si128((const __m128i *)(src + srcStride * row));

    __m128i tmp  = _mm_shuffle_epi8(val, shuffle0);   // 0 1 2 3   1 2 3 4
    tmp          = _mm_madd_epi16(tmp, coeffs[0]);    // 0*C0+1*C1  2*C2+3*C3  1*C0+2*C1  3*C2+4*C2
    __m128i tmp2 = _mm_shuffle_epi8(val, shuffle2);   // 4 5 5 6 ...
    tmp2         = _mm_madd_epi16(tmp2, coeffs[1]);   // 4*C4+5*C5  5*C4+6*C5
    tmp          = _mm_hadd_epi32(tmp, tmp);
    vsum         = _mm_add_epi32(vsum, tmp);
    vsum         = _mm_add_epi32(vsum, tmp2);

    vsum = _mm_sra_epi32(vsum, _mm_cvtsi32_si128(shift));
    vsum = _mm_packs_epi32(vsum, vsum);
    if (CLAMP)
    {
      vsum = _mm_min_epi16(vsum, maxVal);
      vsum = _mm_max_epi16(vsum, minVal);
    }
    _mm_storeu_si32((int16_t *)(dst + dstStride * row), vsum);
  }
}

// SIMD interpolation horizontal, block width modulo 4
template<X86_VEXT vext, int N, bool CLAMP>
static void simdInterpolateHorM4(const int16_t *src, const ptrdiff_t srcStride, int16_t *dst, const ptrdiff_t dstStride,
                                 int width, int height, int shift, int offset, const ClpRng &clpRng,
                                 int16_t const *coeff)
{
#if IF_12TAP
  static_assert(N == 2 || N == 4 || N == 6 || N == 8 || N == 12, "only filter sizes 2, 4, 6, 8 and 12 are supported");
#else
  static_assert(N == 2 || N == 4 || N == 6 || N == 8, "only filter sizes 2, 4, 6, and 8 are supported");
#endif

  _mm_prefetch((const char *)(src + srcStride), _MM_HINT_T0);

  const __m128i minVal = _mm_set1_epi16(clpRng.min);
  const __m128i maxVal = _mm_set1_epi16(clpRng.max);

  __m128i c;
  switch (N)
  {
  case 2:
    c = _mm_cvtsi32_si128(*(int32_t *)coeff);
    break;
  case 4:
    c = _mm_loadl_epi64((__m128i const *)coeff);
    break;
  default:
    c = _mm_loadu_si128((__m128i const *)coeff);
    break;
  }

  __m128i coeffs[4];   // should be coeffs[N / 2] but MSVC doesn't like it
  switch (N)
  {
  case 8:
    coeffs[3] = _mm_shuffle_epi32(c, 0xff);
  case 6:
    coeffs[2] = _mm_shuffle_epi32(c, 0xaa);
  case 4:
    coeffs[1] = _mm_shuffle_epi32(c, 0x55);
  default:
    coeffs[0] = _mm_shuffle_epi32(c, 0x00);
    break;
  }

  const __m128i shuffle0 = _mm_setr_epi8(0, 1, 2, 3, 2, 3, 4, 5, 4, 5, 6, 7, 6, 7, 8, 9);
  const __m128i shuffle1 = _mm_setr_epi8(4, 5, 6, 7, 6, 7, 8, 9, 8, 9, 10, 11, 10, 11, 12, 13);

  for (ptrdiff_t row = 0; row < height; row++)
  {
    _mm_prefetch((const char *)(src + (row + 2) * srcStride), _MM_HINT_T0);

    for (ptrdiff_t col = 0; col < width; col += 4)
    {
      __m128i vsum = _mm_set1_epi32(offset);

      for (ptrdiff_t i = 0; i < N / 2; i += 2)
      {
        const __m128i val = _mm_loadu_si128((const __m128i *)(src + srcStride * row + col + 2 * i));

        vsum = _mm_add_epi32(vsum, _mm_madd_epi16(_mm_shuffle_epi8(val, shuffle0), coeffs[i]));

        if (i + 1 < N / 2)
        {
          vsum = _mm_add_epi32(vsum, _mm_madd_epi16(_mm_shuffle_epi8(val, shuffle1), coeffs[i + 1]));
        }
      }

      vsum = _mm_sra_epi32(vsum, _mm_cvtsi32_si128(shift));
      vsum = _mm_packs_epi32(vsum, vsum);

      if (CLAMP)
      {
        vsum = _mm_min_epi16(vsum, maxVal);
        vsum = _mm_max_epi16(vsum, minVal);
      }

      _mm_storel_epi64((__m128i *)(dst + dstStride * row + col), vsum);
    }
  }
}

// SIMD interpolation horizontal, block width modulo 8
template<X86_VEXT vext, int N, bool CLAMP>
static void simdInterpolateHorM8(const int16_t *src, const ptrdiff_t srcStride, int16_t *dst, const ptrdiff_t dstStride,
                                 int width, int height, int shift, int offset, const ClpRng &clpRng,
                                 int16_t const *coeff)
{
#if IF_12TAP
  static_assert(N == 2 || N == 4 || N == 6 || N == 8 || N == 12, "only filter sizes 2, 4, 6, 8 and 12 are supported");
#else
  static_assert(N == 2 || N == 4 || N == 6 || N == 8, "only filter sizes 2, 4, 6, and 8 are supported");
#endif

  std::array<ptrdiff_t, 3> memOffsets = { { 2 * srcStride, 2 * srcStride + (width >> 1),
                                            2 * srcStride + width - 8 + (N / 2 + 1) / 2 * 4 + 7 } };

  for (auto &off: memOffsets)
  {
    _mm_prefetch((const char *)(src - srcStride + off), _MM_HINT_T0);
  }

  const __m128i minVal = _mm_set1_epi16(clpRng.min);
  const __m128i maxVal = _mm_set1_epi16(clpRng.max);

  __m128i c;
  switch (N)
  {
  case 2:
    c = _mm_cvtsi32_si128(*(int32_t *)coeff);
    break;
  case 4:
    c = _mm_loadl_epi64((__m128i const *)coeff);
    break;
  default:
    c = _mm_loadu_si128((__m128i const *)coeff);
    break;
  }

  __m128i coeffs[4];   // should be coeffs[N / 2] but MSVC doesn't like it
  switch (N)
  {
  case 8:
    coeffs[3] = _mm_shuffle_epi32(c, 0xff);
  case 6:
    coeffs[2] = _mm_shuffle_epi32(c, 0xaa);
  case 4:
    coeffs[1] = _mm_shuffle_epi32(c, 0x55);
  default:
    coeffs[0] = _mm_shuffle_epi32(c, 0x00);
    break;
  }

  const __m128i shuffle0 = _mm_setr_epi8(0, 1, 2, 3, 2, 3, 4, 5, 4, 5, 6, 7, 6, 7, 8, 9);
  const __m128i shuffle1 = _mm_setr_epi8(4, 5, 6, 7, 6, 7, 8, 9, 8, 9, 10, 11, 10, 11, 12, 13);

  for (ptrdiff_t row = 0; row < height; row++)
  {
    for (auto &off: memOffsets)
    {
      _mm_prefetch((const char *)(src + row * srcStride + off), _MM_HINT_T0);
    }

    for (ptrdiff_t col = 0; col < width; col += 8)
    {
      __m128i vsum0 = _mm_set1_epi32(offset);
      __m128i vsum1 = _mm_set1_epi32(offset);

      __m128i val0 = _mm_loadu_si128((const __m128i *)(src + srcStride * row + col));

      for (ptrdiff_t i = 0; i < N / 2; i += 2)
      {
        const __m128i val1 = _mm_loadu_si128((const __m128i *)(src + srcStride * row + col + 2 * i + 4));

        vsum0 = _mm_add_epi32(vsum0, _mm_madd_epi16(_mm_shuffle_epi8(val0, shuffle0), coeffs[i]));
        vsum1 = _mm_add_epi32(vsum1, _mm_madd_epi16(_mm_shuffle_epi8(val1, shuffle0), coeffs[i]));

        if (i + 1 < N / 2)
        {
          vsum0 = _mm_add_epi32(vsum0, _mm_madd_epi16(_mm_shuffle_epi8(val0, shuffle1), coeffs[i + 1]));
          vsum1 = _mm_add_epi32(vsum1, _mm_madd_epi16(_mm_shuffle_epi8(val1, shuffle1), coeffs[i + 1]));
        }

        val0 = val1;
      }

      vsum0 = _mm_sra_epi32(vsum0, _mm_cvtsi32_si128(shift));
      vsum1 = _mm_sra_epi32(vsum1, _mm_cvtsi32_si128(shift));

      __m128i vsum = _mm_packs_epi32(vsum0, vsum1);

      if (CLAMP)
      {
        vsum = _mm_min_epi16(vsum, maxVal);
        vsum = _mm_max_epi16(vsum, minVal);
      }

      _mm_storeu_si128((__m128i *)(dst + dstStride * row + col), vsum);
    }
  }
}
template<X86_VEXT vext, int N, bool CLAMP>
static void simdInterpolateHorN6(const int16_t *src, const ptrdiff_t srcStride, int16_t *dst, const ptrdiff_t dstStride,
                                 int width, int height, int shift, int offset, const ClpRng &clpRng,
                                 int16_t const *coeff)
{
  static_assert(N == 6, "only filter sizes 6 supported");

  _mm_prefetch((const char *)(src + srcStride), _MM_HINT_T0);
  std::array<ptrdiff_t, 3> memOffsets = { { 2 * srcStride, 2 * srcStride + (width >> 1),
                                            2 * srcStride + width - 8 + (N / 2 + 1) / 2 * 4 + 7 } };

  const __m128i minVal = _mm_set1_epi16(clpRng.min);
  const __m128i maxVal = _mm_set1_epi16(clpRng.max);

  __m128i c = _mm_loadu_si128((__m128i const *)coeff);
  ;

  __m128i coeffs[3];
  coeffs[0] = _mm_shuffle_epi32(c, 0x00);
  coeffs[1] = _mm_shuffle_epi32(c, 0x55);
  coeffs[2] = _mm_shuffle_epi32(c, 0xaa);

  const __m128i shuffle0 = _mm_setr_epi8(0, 1, 2, 3, 2, 3, 4, 5, 4, 5, 6, 7, 6, 7, 8, 9);
  const __m128i shuffle1 = _mm_setr_epi8(4, 5, 6, 7, 6, 7, 8, 9, 8, 9, 10, 11, 10, 11, 12, 13);
  const __m128i shuffle2 = _mm_setr_epi8(8, 9, 10, 11, 10, 11, 12, 13, 8, 9, 10, 11, 10, 11, 12, 13);

  ptrdiff_t col;
  int       wdt;
  for (ptrdiff_t row = 0; row < height; row++)
  {
    for (auto &off: memOffsets)
    {
      _mm_prefetch((const char *)(src + row * srcStride + off), _MM_HINT_T0);
    }
    col = 0;
    wdt = width;
    while (wdt >= 8)  // mod8
    {
      __m128i vsum0 = _mm_set1_epi32(offset);
      __m128i vsum1 = _mm_set1_epi32(offset);
      __m128i val0  = _mm_loadu_si128((const __m128i *)(src + srcStride * row + col));
      for (ptrdiff_t i = 0; i < N / 2; i += 2)
      {
        const __m128i val1 = _mm_loadu_si128((const __m128i *)(src + srcStride * row + col + 2 * i + 4));
        vsum0              = _mm_add_epi32(vsum0, _mm_madd_epi16(_mm_shuffle_epi8(val0, shuffle0), coeffs[i]));
        vsum1              = _mm_add_epi32(vsum1, _mm_madd_epi16(_mm_shuffle_epi8(val1, shuffle0), coeffs[i]));
        if (i + 1 < N / 2)
        {
          vsum0 = _mm_add_epi32(vsum0, _mm_madd_epi16(_mm_shuffle_epi8(val0, shuffle1), coeffs[i + 1]));
          vsum1 = _mm_add_epi32(vsum1, _mm_madd_epi16(_mm_shuffle_epi8(val1, shuffle1), coeffs[i + 1]));
        }
        val0 = val1;
      }
      vsum0        = _mm_sra_epi32(vsum0, _mm_cvtsi32_si128(shift));
      vsum1        = _mm_sra_epi32(vsum1, _mm_cvtsi32_si128(shift));
      __m128i vsum = _mm_packs_epi32(vsum0, vsum1);
      if (CLAMP)
      {
        vsum = _mm_min_epi16(vsum, maxVal);
        vsum = _mm_max_epi16(vsum, minVal);
      }
      _mm_storeu_si128((__m128i *)(dst + dstStride * row + col), vsum);
      col += 8;
      wdt -= 8;
    }
    while (wdt >= 4)  // mod4
    {
      __m128i vsum = _mm_set1_epi32(offset);
      for (ptrdiff_t i = 0; i < N / 2; i += 2)
      {
        const __m128i val = _mm_loadu_si128((const __m128i *)(src + srcStride * row + col + 2 * i));
        vsum              = _mm_add_epi32(vsum, _mm_madd_epi16(_mm_shuffle_epi8(val, shuffle0), coeffs[i]));
        if (i + 1 < N / 2)
        {
          vsum = _mm_add_epi32(vsum, _mm_madd_epi16(_mm_shuffle_epi8(val, shuffle1), coeffs[i + 1]));
        }
      }
      vsum = _mm_sra_epi32(vsum, _mm_cvtsi32_si128(shift));
      vsum = _mm_packs_epi32(vsum, vsum);
      if (CLAMP)
      {
        vsum = _mm_min_epi16(vsum, maxVal);
        vsum = _mm_max_epi16(vsum, minVal);
      }
      _mm_storel_epi64((__m128i *)(dst + dstStride * row + col), vsum);
      col += 4;
      wdt -= 4;
    }
    if (wdt)  // 2
    {
      __m128i       vsum = _mm_set1_epi32(offset);
      const __m128i val  = _mm_loadu_si128((const __m128i *)(src + srcStride * row + col));
      vsum               = _mm_add_epi32(vsum, _mm_madd_epi16(_mm_shuffle_epi8(val, shuffle0), coeffs[0]));
      vsum               = _mm_add_epi32(vsum, _mm_madd_epi16(_mm_shuffle_epi8(val, shuffle1), coeffs[1]));
      vsum               = _mm_add_epi32(vsum, _mm_madd_epi16(_mm_shuffle_epi8(val, shuffle2), coeffs[2]));
      vsum               = _mm_sra_epi32(vsum, _mm_cvtsi32_si128(shift));
      vsum               = _mm_packs_epi32(vsum, vsum);
      if (CLAMP)
      {
        vsum = _mm_min_epi16(vsum, maxVal);
        vsum = _mm_max_epi16(vsum, minVal);
      }
      _mm_storeu_si32((int16_t *)(dst + dstStride * row + col), vsum);
    }
  }
}

#ifdef USE_AVX2
template<X86_VEXT vext, int N, bool CLAMP>
static void simdInterpolateHorM8_AVX2(const int16_t *src, const ptrdiff_t srcStride, int16_t *dst,
                                      const ptrdiff_t dstStride, int width, int height, int shift, int offset,
                                      const ClpRng &clpRng, int16_t const *coeff)
{
#if IF_12TAP
  static_assert(N == 2 || N == 4 || N == 6 || N == 8 || N == 12, "only filter sizes 2, 4, 6, 8 and 12 are supported");
#else
  static_assert(N == 2 || N == 4 || N == 6 || N == 8, "only filter sizes 2, 4, 6, and 8 are supported");
#endif

  std::array<ptrdiff_t, 3> memOffsets = { { 2 * srcStride, 2 * srcStride + (width >> 1),
                                            2 * srcStride + width - 8 + (N / 2 + 1) / 2 * 4 + 7 } };

  for (auto &off: memOffsets)
  {
    _mm_prefetch((const char *)(src - srcStride + off), _MM_HINT_T0);
  }

  const __m128i minVal = _mm_set1_epi16(clpRng.min);
  const __m128i maxVal = _mm_set1_epi16(clpRng.max);

  __m128i c0;
  switch (N)
  {
  case 2:
    c0 = _mm_cvtsi32_si128(*(int32_t *)coeff);
    break;
  case 4:
    c0 = _mm_loadl_epi64((__m128i const *)coeff);
    break;
  default:
    c0 = _mm_loadu_si128((__m128i const *)coeff);
    break;
  }
  __m256i c = _mm256_broadcastsi128_si256(c0);

  __m256i coeffs[4];   // should be coeffs[N / 2] but MSVC doesn't like it
  switch (N)
  {
  case 8:
    coeffs[3] = _mm256_shuffle_epi32(c, 0xff);
  case 6:
    coeffs[2] = _mm256_shuffle_epi32(c, 0xaa);
  case 4:
    coeffs[1] = _mm256_shuffle_epi32(c, 0x55);
  default:
    coeffs[0] = _mm256_shuffle_epi32(c, 0x00);
    break;
  }

  const __m256i shuffle0 = _mm256_broadcastsi128_si256(_mm_setr_epi8(0, 1, 2, 3, 2, 3, 4, 5, 4, 5, 6, 7, 6, 7, 8, 9));
  const __m256i shuffle1 =
    _mm256_broadcastsi128_si256(_mm_setr_epi8(4, 5, 6, 7, 6, 7, 8, 9, 8, 9, 10, 11, 10, 11, 12, 13));

  for (ptrdiff_t row = 0; row < height; row++)
  {
    for (auto &off: memOffsets)
    {
      _mm_prefetch((const char *)(src + row * srcStride + off), _MM_HINT_T0);
    }

    for (ptrdiff_t col = 0; col < width; col += 8)
    {
      __m256i vsum = _mm256_set1_epi32(offset);

      __m128i val0 = _mm_loadu_si128((const __m128i *)(src + srcStride * row + col));

      for (ptrdiff_t i = 0; i < N / 2; i += 2)
      {
        const __m128i val1 = _mm_loadu_si128((const __m128i *)(src + srcStride * row + col + 2 * i + 4));
        const __m256i val  = _mm256_inserti128_si256(_mm256_castsi128_si256(val0), val1, 1);

        vsum = _mm256_add_epi32(vsum, _mm256_madd_epi16(_mm256_shuffle_epi8(val, shuffle0), coeffs[i]));

        if (i + 1 < N / 2)
        {
          vsum = _mm256_add_epi32(vsum, _mm256_madd_epi16(_mm256_shuffle_epi8(val, shuffle1), coeffs[i + 1]));
        }

        val0 = val1;
      }

      vsum = _mm256_sra_epi32(vsum, _mm_cvtsi32_si128(shift));

      __m128i sum = _mm_packs_epi32(_mm256_castsi256_si128(vsum), _mm256_extracti128_si256(vsum, 1));

      if (CLAMP)
      {
        sum = _mm_min_epi16(sum, maxVal);
        sum = _mm_max_epi16(sum, minVal);
      }

      _mm_storeu_si128((__m128i *)(dst + dstStride * row + col), sum);
    }
  }
}
#endif

#if IF_12TAP_SIMD
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateVerM4_12tap(const Pel *src, ptrdiff_t srcStride, Pel *dst, ptrdiff_t dstStride, int width,
                                       int height, int shift, int offset, const ClpRng &clpRng, Pel const *coeff)
{
  static_assert(N == 12, "only filter size 12 is supported");
  const Pel *srcOrig = src;
  Pel       *dstOrig = dst;
  __m128i    vcoeff[N / 2], vsrc[N];
  __m128i    vzero    = _mm_setzero_si128();
  __m128i    voffset  = _mm_set1_epi32(offset);
  __m128i    vibdimin = _mm_set1_epi16(clpRng.min);
  __m128i    vibdimax = _mm_set1_epi16(clpRng.max);

  __m128i vsum;

  for (int i = 0; i < N; i += 2)
  {
    vcoeff[i / 2] = _mm_unpacklo_epi16(_mm_set1_epi16(coeff[i]), _mm_set1_epi16(coeff[i + 1]));
  }

  for (int col = 0; col < width; col += 4)
  {
    for (int i = 0; i < N - 1; i++)
    {
      vsrc[i] = _mm_loadl_epi64((__m128i const *)&src[col + i * srcStride]);
    }
    for (int row = 0; row < height; row++)
    {
      vsrc[N - 1] = _mm_loadl_epi64((__m128i const *)&src[col + (N - 1) * srcStride]);

      vsum = vzero;
      for (int i = 0; i < N; i += 2)
      {
        __m128i vsrc0 = _mm_unpacklo_epi16(vsrc[i], vsrc[i + 1]);
        vsum          = _mm_add_epi32(vsum, _mm_madd_epi16(vsrc0, vcoeff[i / 2]));
      }

      for (int i = 0; i < N - 1; i++)
      {
        vsrc[i] = vsrc[i + 1];
      }

      //       if( shiftBack )
      {
        vsum = _mm_add_epi32(vsum, voffset);
        vsum = _mm_srai_epi32(vsum, shift);
        vsum = _mm_packs_epi32(vsum, vzero);
      }

      if (shiftBack) // clip
      {
        vsum = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsum));
      }

      _mm_storel_epi64((__m128i *)&dst[col], vsum);

      src += srcStride;
      dst += dstStride;
    }
    src = srcOrig;
    dst = dstOrig;
  }
}

#if USE_AVX2
template<X86_VEXT, int N, bool clamp>
static void simdInterpolateVerM8_12tap(const Pel *src, ptrdiff_t srcStride, Pel *dst, ptrdiff_t dstStride, int width,
                                       int height, int shift, int offset, const ClpRng &clpRng, Pel const *coeff)
{
  static_assert(N == 12, "only filter size 12 is supported");

  CHECKD(height == 0, "there is a tiny optimization below that presumes non-zero height")

  Pel                        *dstOrig = dst;
  __m256i                     vsrc[11];
  __m256i                     vcoeff[6];
  [[maybe_unused]] const auto vibdimin = _mm_set1_epi16(clpRng.min);
  [[maybe_unused]] const auto vibdimax = _mm_set1_epi16(clpRng.max);

  for (int i = 0; i < 12; i += 2)
  {
    vcoeff[i / 2] = _mm256_unpacklo_epi16(_mm256_set1_epi16(coeff[i]), _mm256_set1_epi16(coeff[i + 1]));
  }

  for (int col = 0; col < width; col += 8)
  {
    for (int i = 0; i < 11; i++)
    {
      __m128i vtmp = _mm_loadu_si128((__m128i const *)&src[col + i * srcStride]);
      vsrc[i]      = _mm256_inserti128_si256(_mm256_castsi128_si256(vtmp), _mm_unpackhi_epi64(vtmp, vtmp), 1);
    }
    _mm_prefetch((const char *)&src[col + 11 * srcStride], _MM_HINT_T0);

    // hopefully the compiler keeps the following 11 variables in ymm registers during the main loop
    auto pair_0_1  = _mm256_unpacklo_epi16(vsrc[0], vsrc[1]);
    auto pair_1_2  = _mm256_unpacklo_epi16(vsrc[1], vsrc[2]);
    auto pair_2_3  = _mm256_unpacklo_epi16(vsrc[2], vsrc[3]);
    auto pair_3_4  = _mm256_unpacklo_epi16(vsrc[3], vsrc[4]);
    auto pair_4_5  = _mm256_unpacklo_epi16(vsrc[4], vsrc[5]);
    auto pair_5_6  = _mm256_unpacklo_epi16(vsrc[5], vsrc[6]);
    auto pair_6_7  = _mm256_unpacklo_epi16(vsrc[6], vsrc[7]);
    auto pair_7_8  = _mm256_unpacklo_epi16(vsrc[7], vsrc[8]);
    auto pair_8_9  = _mm256_unpacklo_epi16(vsrc[8], vsrc[9]);
    auto pair_9_10 = _mm256_unpacklo_epi16(vsrc[9], vsrc[10]);
    auto ping      = vsrc[10];

    auto p = &src[col + 11 * srcStride];

    for (int row = 0; row == 0 || row < height; ++row)
    {
      {
        auto vsum       = _mm256_set1_epi32(offset);
        vsum            = _mm256_add_epi32(vsum, _mm256_madd_epi16(vcoeff[0 / 2], pair_0_1));
        pair_0_1        = pair_2_3;
        vsum            = _mm256_add_epi32(vsum, _mm256_madd_epi16(vcoeff[2 / 2], pair_2_3));
        pair_2_3        = pair_4_5;
        vsum            = _mm256_add_epi32(vsum, _mm256_madd_epi16(vcoeff[4 / 2], pair_4_5));
        pair_4_5        = pair_6_7;
        vsum            = _mm256_add_epi32(vsum, _mm256_madd_epi16(vcoeff[6 / 2], pair_6_7));
        pair_6_7        = pair_8_9;
        vsum            = _mm256_add_epi32(vsum, _mm256_madd_epi16(vcoeff[8 / 2], pair_8_9));
        const auto vtmp = _mm_loadu_si128((__m128i const *)p);
        _mm_prefetch((const char *)&p[srcStride], _MM_HINT_T0);
        const auto pong = _mm256_inserti128_si256(_mm256_castsi128_si256(vtmp), _mm_unpackhi_epi64(vtmp, vtmp), 1);
        pair_8_9        = _mm256_unpacklo_epi16(ping, pong);
        ping            = pong;
        vsum            = _mm256_add_epi32(vsum, _mm256_madd_epi16(vcoeff[10 / 2], pair_8_9));

        vsum         = _mm256_srai_epi32(vsum, shift);
        vsum         = _mm256_packs_epi32(vsum, vsum);
        vsum         = _mm256_permute4x64_epi64(vsum, 0 + (2 << 2));
        auto vsum128 = _mm256_castsi256_si128(vsum);

        if constexpr (clamp)
        {
          vsum128 = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsum128));
        }

        _mm_storeu_si128((__m128i *)&dst[col], vsum128);

        p += srcStride;
        dst += dstStride;
      }

      if (++row < height)
      {
        auto vsum       = _mm256_set1_epi32(offset);
        vsum            = _mm256_add_epi32(vsum, _mm256_madd_epi16(vcoeff[0 / 2], pair_1_2));
        pair_1_2        = pair_3_4;
        vsum            = _mm256_add_epi32(vsum, _mm256_madd_epi16(vcoeff[2 / 2], pair_3_4));
        pair_3_4        = pair_5_6;
        vsum            = _mm256_add_epi32(vsum, _mm256_madd_epi16(vcoeff[4 / 2], pair_5_6));
        pair_5_6        = pair_7_8;
        vsum            = _mm256_add_epi32(vsum, _mm256_madd_epi16(vcoeff[6 / 2], pair_7_8));
        pair_7_8        = pair_9_10;
        vsum            = _mm256_add_epi32(vsum, _mm256_madd_epi16(vcoeff[8 / 2], pair_9_10));
        const auto vtmp = _mm_loadu_si128((__m128i const *)p);
        const auto pong = _mm256_inserti128_si256(_mm256_castsi128_si256(vtmp), _mm_unpackhi_epi64(vtmp, vtmp), 1);
        pair_9_10       = _mm256_unpacklo_epi16(ping, pong);
        ping            = pong;
        vsum            = _mm256_add_epi32(vsum, _mm256_madd_epi16(vcoeff[10 / 2], pair_9_10));

        vsum         = _mm256_srai_epi32(vsum, shift);
        vsum         = _mm256_packs_epi32(vsum, vsum);
        vsum         = _mm256_permute4x64_epi64(vsum, 0 + (2 << 2));
        auto vsum128 = _mm256_castsi256_si128(vsum);

        if constexpr (clamp)
        {
          vsum128 = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsum128));
        }

        _mm_storeu_si128((__m128i *)&dst[col], vsum128);

        p += srcStride;
        dst += dstStride;
      }
    }
    dst = dstOrig;
  }
}

template<X86_VEXT, int N, bool clamp>
static void simdInterpolateVerM16_12tap(const Pel *src, ptrdiff_t srcStride, Pel *dst, ptrdiff_t dstStride, int width,
                                        int height, int shift, int offset, const ClpRng &clpRng, Pel const *coeff)
{
  static_assert(N == 12, "only filter size 12 is supported");

  CHECKD(height == 0, "there is a tiny optimization below that presumes non-zero height")

  Pel *dstOrig = dst;

  int32_t coeffPairs[12];
  for (int i = 0; i < 12; ++i)
  {
    coeffPairs[i] = (coeff[i] & 0xffff) | (coeff[(i + 1) % 12] << 16);
  }

  for (int col = 0; col < width; col += 16)
  {
    //_mm_prefetch((const char*)&src[col + 11 * srcStride], _MM_HINT_T0);

    __m256i even, odd;

    auto p = &src[col];

    // hopefully the compiler keeps the pair_ variables in registers
    even             = _mm256_loadu_si256((const __m256i *)&p[0 * srcStride]);
    odd              = _mm256_loadu_si256((const __m256i *)&p[1 * srcStride]);
    auto pair_0_1_lo = _mm256_unpacklo_epi16(even, odd);
    auto pair_0_1_hi = _mm256_unpackhi_epi16(even, odd);

    even             = _mm256_loadu_si256((const __m256i *)&p[2 * srcStride]);
    odd              = _mm256_loadu_si256((const __m256i *)&p[3 * srcStride]);
    auto pair_2_3_lo = _mm256_unpacklo_epi16(even, odd);
    auto pair_2_3_hi = _mm256_unpackhi_epi16(even, odd);

    even             = _mm256_loadu_si256((const __m256i *)&p[4 * srcStride]);
    odd              = _mm256_loadu_si256((const __m256i *)&p[5 * srcStride]);
    auto pair_4_5_lo = _mm256_unpacklo_epi16(even, odd);
    auto pair_4_5_hi = _mm256_unpackhi_epi16(even, odd);

    even             = _mm256_loadu_si256((const __m256i *)&p[6 * srcStride]);
    odd              = _mm256_loadu_si256((const __m256i *)&p[7 * srcStride]);
    auto pair_6_7_lo = _mm256_unpacklo_epi16(even, odd);
    auto pair_6_7_hi = _mm256_unpackhi_epi16(even, odd);

    even             = _mm256_loadu_si256((const __m256i *)&p[8 * srcStride]);
    odd              = _mm256_loadu_si256((const __m256i *)&p[9 * srcStride]);
    auto pair_8_9_lo = _mm256_unpacklo_epi16(even, odd);
    auto pair_8_9_hi = _mm256_unpackhi_epi16(even, odd);

    even               = _mm256_loadu_si256((const __m256i *)&p[10 * srcStride]);
    auto pair_10_11_lo = _mm256_unpacklo_epi16(even, even);
    auto pair_10_11_hi = _mm256_unpackhi_epi16(even, even);

    for (int row = 0; row == 0 || row < height; ++row)
    {
      {
        // even
        const auto x  = _mm256_loadu_si256((const __m256i *)&p[11 * srcStride]);
        const auto lo = _mm256_unpacklo_epi16(x, x);
        const auto hi = _mm256_unpackhi_epi16(x, x);
        pair_10_11_lo = _mm256_blend_epi16(lo, pair_10_11_lo, 0x55);
        pair_10_11_hi = _mm256_blend_epi16(hi, pair_10_11_hi, 0x55);

        auto       vsum_lo     = _mm256_set1_epi32(offset);
        auto       vsum_hi     = vsum_lo;
        const auto coeff_0_1   = _mm256_set1_epi32(coeffPairs[0]);
        vsum_lo                = _mm256_add_epi32(vsum_lo, _mm256_madd_epi16(coeff_0_1, pair_0_1_lo));
        vsum_hi                = _mm256_add_epi32(vsum_hi, _mm256_madd_epi16(coeff_0_1, pair_0_1_hi));
        const auto coeff_2_3   = _mm256_set1_epi32(coeffPairs[2]);
        vsum_lo                = _mm256_add_epi32(vsum_lo, _mm256_madd_epi16(coeff_2_3, pair_2_3_lo));
        vsum_hi                = _mm256_add_epi32(vsum_hi, _mm256_madd_epi16(coeff_2_3, pair_2_3_hi));
        const auto coeff_4_5   = _mm256_set1_epi32(coeffPairs[4]);
        vsum_lo                = _mm256_add_epi32(vsum_lo, _mm256_madd_epi16(coeff_4_5, pair_4_5_lo));
        vsum_hi                = _mm256_add_epi32(vsum_hi, _mm256_madd_epi16(coeff_4_5, pair_4_5_hi));
        const auto coeff_6_7   = _mm256_set1_epi32(coeffPairs[6]);
        vsum_lo                = _mm256_add_epi32(vsum_lo, _mm256_madd_epi16(coeff_6_7, pair_6_7_lo));
        vsum_hi                = _mm256_add_epi32(vsum_hi, _mm256_madd_epi16(coeff_6_7, pair_6_7_hi));
        const auto coeff_8_9   = _mm256_set1_epi32(coeffPairs[8]);
        vsum_lo                = _mm256_add_epi32(vsum_lo, _mm256_madd_epi16(coeff_8_9, pair_8_9_lo));
        vsum_hi                = _mm256_add_epi32(vsum_hi, _mm256_madd_epi16(coeff_8_9, pair_8_9_hi));
        const auto coeff_10_11 = _mm256_set1_epi32(coeffPairs[10]);
        vsum_lo                = _mm256_add_epi32(vsum_lo, _mm256_madd_epi16(coeff_10_11, pair_10_11_lo));
        vsum_hi                = _mm256_add_epi32(vsum_hi, _mm256_madd_epi16(coeff_10_11, pair_10_11_hi));
        vsum_lo                = _mm256_srai_epi32(vsum_lo, shift);
        vsum_hi                = _mm256_srai_epi32(vsum_hi, shift);
        auto vsum              = _mm256_packs_epi32(vsum_lo, vsum_hi);

        if constexpr (clamp)
        {
          const auto vibdimin = _mm256_set1_epi16(clpRng.min);
          const auto vibdimax = _mm256_set1_epi16(clpRng.max);
          vsum                = _mm256_min_epi16(vibdimax, _mm256_max_epi16(vibdimin, vsum));
        }

        _mm256_storeu_si256((__m256i *)&dst[col], vsum);

        dst += dstStride;
      }

      if (++row < height)
      {
        // odd
        const auto x            = _mm256_loadu_si256((const __m256i *)&p[12 * srcStride]);
        const auto lo           = _mm256_unpacklo_epi16(x, x);
        const auto hi           = _mm256_unpackhi_epi16(x, x);
        const auto pair_12_1_lo = _mm256_blend_epi16(lo, pair_0_1_lo, 0xaa);
        const auto pair_12_1_hi = _mm256_blend_epi16(hi, pair_0_1_hi, 0xaa);

        auto       vsum_lo    = _mm256_set1_epi32(offset);
        auto       vsum_hi    = vsum_lo;
        const auto coeff_1_2  = _mm256_set1_epi32(coeffPairs[1]);
        vsum_lo               = _mm256_add_epi32(vsum_lo, _mm256_madd_epi16(coeff_1_2, pair_2_3_lo));
        vsum_hi               = _mm256_add_epi32(vsum_hi, _mm256_madd_epi16(coeff_1_2, pair_2_3_hi));
        const auto coeff_3_4  = _mm256_set1_epi32(coeffPairs[3]);
        vsum_lo               = _mm256_add_epi32(vsum_lo, _mm256_madd_epi16(coeff_3_4, pair_4_5_lo));
        vsum_hi               = _mm256_add_epi32(vsum_hi, _mm256_madd_epi16(coeff_3_4, pair_4_5_hi));
        const auto coeff_5_6  = _mm256_set1_epi32(coeffPairs[5]);
        vsum_lo               = _mm256_add_epi32(vsum_lo, _mm256_madd_epi16(coeff_5_6, pair_6_7_lo));
        vsum_hi               = _mm256_add_epi32(vsum_hi, _mm256_madd_epi16(coeff_5_6, pair_6_7_hi));
        const auto coeff_7_8  = _mm256_set1_epi32(coeffPairs[7]);
        vsum_lo               = _mm256_add_epi32(vsum_lo, _mm256_madd_epi16(coeff_7_8, pair_8_9_lo));
        vsum_hi               = _mm256_add_epi32(vsum_hi, _mm256_madd_epi16(coeff_7_8, pair_8_9_hi));
        const auto coeff_9_10 = _mm256_set1_epi32(coeffPairs[9]);
        vsum_lo               = _mm256_add_epi32(vsum_lo, _mm256_madd_epi16(coeff_9_10, pair_10_11_lo));
        vsum_hi               = _mm256_add_epi32(vsum_hi, _mm256_madd_epi16(coeff_9_10, pair_10_11_hi));
        const auto coeff_11_0 = _mm256_set1_epi32(coeffPairs[11]);
        vsum_lo               = _mm256_add_epi32(vsum_lo, _mm256_madd_epi16(coeff_11_0, pair_12_1_lo));
        vsum_hi               = _mm256_add_epi32(vsum_hi, _mm256_madd_epi16(coeff_11_0, pair_12_1_hi));
        vsum_lo               = _mm256_srai_epi32(vsum_lo, shift);
        vsum_hi               = _mm256_srai_epi32(vsum_hi, shift);
        auto vsum             = _mm256_packs_epi32(vsum_lo, vsum_hi);

        if constexpr (clamp)
        {
          const auto vibdimin = _mm256_set1_epi16(clpRng.min);
          const auto vibdimax = _mm256_set1_epi16(clpRng.max);
          vsum                = _mm256_min_epi16(vibdimax, _mm256_max_epi16(vibdimin, vsum));
        }

        _mm256_storeu_si256((__m256i *)&dst[col], vsum);

        pair_0_1_lo   = pair_2_3_lo;
        pair_0_1_hi   = pair_2_3_hi;
        pair_2_3_lo   = pair_4_5_lo;
        pair_2_3_hi   = pair_4_5_hi;
        pair_4_5_lo   = pair_6_7_lo;
        pair_4_5_hi   = pair_6_7_hi;
        pair_6_7_lo   = pair_8_9_lo;
        pair_6_7_hi   = pair_8_9_hi;
        pair_8_9_lo   = pair_10_11_lo;
        pair_8_9_hi   = pair_10_11_hi;
        pair_10_11_lo = pair_12_1_lo;
        pair_10_11_hi = pair_12_1_hi;

        p += 2 * srcStride;
        dst += dstStride;
      }
    }
    dst = dstOrig;
  }
}
#endif

template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateVerNonM4_12tap(const Pel *src, ptrdiff_t srcStride, Pel *dst, ptrdiff_t dstStride, int width,
                                          int height, int shift, int offset, const ClpRng &clpRng, Pel const *coeff)
{
  static_assert(N == 12, "only filter size 12 is supported");
  const Pel *srcOrig = src;
  Pel       *dstOrig = dst;
  __m128i    vcoeff[N / 2], vsrc[N];
  __m128i    vzero    = _mm_setzero_si128();
  __m128i    voffset  = _mm_set1_epi32(offset);
  __m128i    vibdimin = _mm_set1_epi16(clpRng.min);
  __m128i    vibdimax = _mm_set1_epi16(clpRng.max);

  __m128i vsum;

  int        multiple   = (width >> 2) << 2;
  const Pel *initialSrc = src;
  Pel       *initialDst = dst;

  for (int i = 0; i < N; i += 2)
  {
    vcoeff[i / 2] = _mm_unpacklo_epi16(_mm_set1_epi16(coeff[i]), _mm_set1_epi16(coeff[i + 1]));
  }

  for (int col = 0; col < multiple; col += 4)
  {
    for (int i = 0; i < N - 1; i++)
    {
      vsrc[i] = _mm_loadl_epi64((__m128i const *)&src[col + i * srcStride]);
    }
    for (int row = 0; row < height; row++)
    {
      vsrc[N - 1] = _mm_loadl_epi64((__m128i const *)&src[col + (N - 1) * srcStride]);

      vsum = vzero;
      for (int i = 0; i < N; i += 2)
      {
        __m128i vsrc0 = _mm_unpacklo_epi16(vsrc[i], vsrc[i + 1]);
        vsum          = _mm_add_epi32(vsum, _mm_madd_epi16(vsrc0, vcoeff[i / 2]));
      }

      for (int i = 0; i < N - 1; i++)
      {
        vsrc[i] = vsrc[i + 1];
      }

      //       if( shiftBack )
      {
        vsum = _mm_add_epi32(vsum, voffset);
        vsum = _mm_srai_epi32(vsum, shift);
        vsum = _mm_packs_epi32(vsum, vzero);
      }

      if (shiftBack) // clip
      {
        vsum = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsum));
      }

      _mm_storel_epi64((__m128i *)&dst[col], vsum);

      src += srcStride;
      dst += dstStride;
    }
    src = srcOrig;
    dst = dstOrig;
  }
  for (int row = 0; row < height; row++)
  {
    for (int col = multiple; col < width; col++)
    {
      int sum = 0;

      sum = initialSrc[col] * coeff[0];
      sum += initialSrc[col + 1 * srcStride] * coeff[1];
      sum += initialSrc[col + 2 * srcStride] * coeff[2];
      sum += initialSrc[col + 3 * srcStride] * coeff[3];
      sum += initialSrc[col + 4 * srcStride] * coeff[4];
      sum += initialSrc[col + 5 * srcStride] * coeff[5];
      sum += initialSrc[col + 6 * srcStride] * coeff[6];
      sum += initialSrc[col + 7 * srcStride] * coeff[7];
      sum += initialSrc[col + 8 * srcStride] * coeff[8];
      sum += initialSrc[col + 9 * srcStride] * coeff[9];
      sum += initialSrc[col + 10 * srcStride] * coeff[10];
      sum += initialSrc[col + 11 * srcStride] * coeff[11];

      Pel val = (sum + offset) >> shift;
      if (shiftBack)
      {
        val = ClipPel(val, clpRng);
      }
      initialDst[col] = val;
    }

    initialSrc += srcStride;
    initialDst += dstStride;
  }
}
#endif
// SIMD interpolate Ver Size 2
template<X86_VEXT vext, int N, bool CLAMP>
static void simdInterpolateVerM2(const int16_t *src, const ptrdiff_t srcStride, int16_t *dst, const ptrdiff_t dstStride,
                                 int height, int shift, int offset, const ClpRng &clpRng, int16_t const *coeff)
{
#if IF_12TAP
  static_assert(N == 2 || N == 4 || N == 6 || N == 8 || N == 12, "only filter sizes 2, 4, 6, 8 and 12 are supported");
#else
  static_assert(N == 2 || N == 4 || N == 6 || N == 8, "only filter sizes 2, 4, 6, and 8 are supported");
#endif

  const __m128i minVal = _mm_set1_epi16(clpRng.min);
  const __m128i maxVal = _mm_set1_epi16(clpRng.max);

  const __m128i vcoeff = N == 2
    ? _mm_cvtsi32_si128(*(uint32_t *)coeff)
    : (N == 4 ? _mm_loadl_epi64((__m128i const *)coeff) : _mm_loadu_si128((__m128i const *)coeff));

  for (ptrdiff_t row = 0; row < height; row++)
  {
    __m128i vsum = _mm_set1_epi32(offset);

    __m128i val[N / 2];

    for (ptrdiff_t i = 0; i < N / 2; i++)
    {
      const __m128i valA = _mm_loadu_si32((const int16_t *)(src + (row + 2 * i) * srcStride));
      const __m128i valB = _mm_loadu_si32((const int16_t *)(src + (row + 2 * i + 1) * srcStride));

      val[i] = _mm_unpacklo_epi16(valA, valB);
    }

    vsum = _mm_add_epi32(vsum, _mm_madd_epi16(val[0], _mm_shuffle_epi32(vcoeff, 0x00)));
    if (N >= 4)
    {
      vsum = _mm_add_epi32(vsum, _mm_madd_epi16(val[1], _mm_shuffle_epi32(vcoeff, 0x55)));
    }
    if (N >= 6)
    {
      vsum = _mm_add_epi32(vsum, _mm_madd_epi16(val[2], _mm_shuffle_epi32(vcoeff, 0xaa)));
    }
    if (N == 8)
    {
      vsum = _mm_add_epi32(vsum, _mm_madd_epi16(val[3], _mm_shuffle_epi32(vcoeff, 0xff)));
    }
    vsum = _mm_srai_epi32(vsum, shift);
    vsum = _mm_packs_epi32(vsum, vsum);

    if (CLAMP)
    {
      vsum = _mm_min_epi16(vsum, maxVal);
      vsum = _mm_max_epi16(vsum, minVal);
    }
    _mm_storeu_si32((int16_t *)(dst + row * dstStride), vsum);
  }
}

template<X86_VEXT vext, int N, bool CLAMP>
static void simdInterpolateVerM4(const int16_t *src, const ptrdiff_t srcStride, int16_t *dst, const ptrdiff_t dstStride,
                                 int width, int height, int shift, int offset, const ClpRng &clpRng,
                                 int16_t const *coeff)
{
#if IF_12TAP
  static_assert(N == 2 || N == 4 || N == 6 || N == 8 || N == 12, "only filter sizes 2, 4, 6, 8 and 12 are supported");
#else
  static_assert(N == 2 || N == 4 || N == 6 || N == 8, "only filter sizes 2, 4, 6, and 8 are supported");
#endif

  const __m128i minVal = _mm_set1_epi16(clpRng.min);
  const __m128i maxVal = _mm_set1_epi16(clpRng.max);

  const __m128i vcoeff = N == 2
    ? _mm_cvtsi32_si128(*(uint32_t *)coeff)
    : (N == 4 ? _mm_loadl_epi64((__m128i const *)coeff) : _mm_loadu_si128((__m128i const *)coeff));

  for (ptrdiff_t col = 0; col < width; col += 4)
  {
    for (ptrdiff_t row = 0; row < height; row++)
    {
      __m128i vsum = _mm_set1_epi32(offset);

      __m128i val[N / 2];

      for (ptrdiff_t i = 0; i < N / 2; i++)
      {
        const __m128i valA = _mm_loadl_epi64((__m128i const *)(src + col + (row + 2 * i) * srcStride));
        const __m128i valB = _mm_loadl_epi64((__m128i const *)(src + col + (row + 2 * i + 1) * srcStride));

        val[i] = _mm_unpacklo_epi16(valA, valB);
      }

      vsum = _mm_add_epi32(vsum, _mm_madd_epi16(val[0], _mm_shuffle_epi32(vcoeff, 0x00)));
      if (N >= 4)
      {
        vsum = _mm_add_epi32(vsum, _mm_madd_epi16(val[1], _mm_shuffle_epi32(vcoeff, 0x55)));
      }
      if (N >= 6)
      {
        vsum = _mm_add_epi32(vsum, _mm_madd_epi16(val[2], _mm_shuffle_epi32(vcoeff, 0xaa)));
      }
      if (N == 8)
      {
        vsum = _mm_add_epi32(vsum, _mm_madd_epi16(val[3], _mm_shuffle_epi32(vcoeff, 0xff)));
      }

      vsum = _mm_srai_epi32(vsum, shift);
      vsum = _mm_packs_epi32(vsum, vsum);

      if (CLAMP)
      {
        vsum = _mm_min_epi16(vsum, maxVal);
        vsum = _mm_max_epi16(vsum, minVal);
      }

      _mm_storel_epi64((__m128i *)(dst + row * dstStride + col), vsum);
    }
  }
}

template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateVerM8(const int16_t *src, const ptrdiff_t srcStride, int16_t *dst, const ptrdiff_t dstStride,
                                 int width, int height, int shift, int offset, const ClpRng &clpRng,
                                 int16_t const *coeff)
{
  const int16_t *srcOrig = src;
  int16_t       *dstOrig = dst;

  __m128i vcoeff[N / 2], vsrc[N];
  __m128i vzero    = _mm_setzero_si128();
  __m128i voffset  = _mm_set1_epi32(offset);
  __m128i vibdimin = _mm_set1_epi16(clpRng.min);
  __m128i vibdimax = _mm_set1_epi16(clpRng.max);

  __m128i vsum, vsuma, vsumb;

  for (int i = 0; i < N; i += 2)
  {
    vcoeff[i / 2] = _mm_unpacklo_epi16(_mm_set1_epi16(coeff[i]), _mm_set1_epi16(coeff[i + 1]));
  }

  for (int col = 0; col < width; col += 8)
  {
    for (int i = 0; i < N - 1; i++)
    {
      vsrc[i] = _mm_lddqu_si128((__m128i const *)&src[col + i * srcStride]);
    }

    for (int row = 0; row < height; row++)
    {
      vsrc[N - 1] = _mm_lddqu_si128((__m128i const *)&src[col + (N - 1) * srcStride]);
      vsuma = vsumb = vzero;
      for (int i = 0; i < N; i += 2)
      {
        __m128i vsrca = _mm_unpacklo_epi16(vsrc[i], vsrc[i + 1]);
        __m128i vsrcb = _mm_unpackhi_epi16(vsrc[i], vsrc[i + 1]);
        vsuma         = _mm_add_epi32(vsuma, _mm_madd_epi16(vsrca, vcoeff[i / 2]));
        vsumb         = _mm_add_epi32(vsumb, _mm_madd_epi16(vsrcb, vcoeff[i / 2]));
      }
      for (int i = 0; i < N - 1; i++)
      {
        vsrc[i] = vsrc[i + 1];
      }

      vsuma = _mm_add_epi32(vsuma, voffset);
      vsumb = _mm_add_epi32(vsumb, voffset);

      vsuma = _mm_srai_epi32(vsuma, shift);
      vsumb = _mm_srai_epi32(vsumb, shift);

      vsum = _mm_packs_epi32(vsuma, vsumb);

      if (shiftBack) // clip
      {
        vsum = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsum));
      }

      _mm_storeu_si128((__m128i *)&dst[col], vsum);

      src += srcStride;
      dst += dstStride;
    }
    src = srcOrig;
    dst = dstOrig;
  }
}

#ifdef USE_AVX2
template<X86_VEXT vext, int N, bool CLAMP>
static void simdInterpolateVerM8_AVX2(const int16_t *src, const ptrdiff_t srcStride, int16_t *dst,
                                      const ptrdiff_t dstStride, int width, int height, int shift, int offset,
                                      const ClpRng &clpRng, int16_t const *coeff)
{
  const __m128i minVal = _mm_set1_epi16(clpRng.min);
  const __m128i maxVal = _mm_set1_epi16(clpRng.max);

  __m256i coeffs[N / 2];

  for (int i = 0; i < N / 2; i++)
  {
    coeffs[i] = _mm256_broadcastd_epi32(_mm_cvtsi32_si128(*(int32_t *)&coeff[2 * i]));
  }

  for (ptrdiff_t col = 0; col < width; col += 8)
  {
    __m256i vsrc[N];

    for (int i = 0; i < N - 1; i++)
    {
      vsrc[i] = _mm256_castsi128_si256(_mm_loadu_si128((const __m128i *)(src + col + i * srcStride)));
      vsrc[i] = _mm256_permute4x64_epi64(vsrc[i], _MM_SHUFFLE(1, 1, 0, 0));
    }

    for (ptrdiff_t row = 0; row < height; row++)
    {
      vsrc[N - 1] = _mm256_castsi128_si256(_mm_loadu_si128((const __m128i *)(src + col + (row + N - 1) * srcStride)));
      vsrc[N - 1] = _mm256_permute4x64_epi64(vsrc[N - 1], _MM_SHUFFLE(1, 1, 0, 0));

      __m256i vsum = _mm256_set1_epi32(offset);

      for (int i = 0; i < N / 2; i++)
      {
        __m256i vsrc0 = _mm256_unpacklo_epi16(vsrc[2 * i], vsrc[2 * i + 1]);
        vsum          = _mm256_add_epi32(vsum, _mm256_madd_epi16(vsrc0, coeffs[i]));
      }

      vsum = _mm256_sra_epi32(vsum, _mm_cvtsi32_si128(shift));
      vsum = _mm256_packs_epi32(vsum, vsum);

      __m128i sum = _mm256_castsi256_si128(_mm256_permute4x64_epi64(vsum, _MM_SHUFFLE(3, 1, 2, 0)));

      if (CLAMP)
      {
        sum = _mm_min_epi16(sum, maxVal);
        sum = _mm_max_epi16(sum, minVal);
      }

      _mm_storeu_si128((__m128i *)(dst + row * dstStride + col), sum);

      for (int i = 0; i < N - 1; i++)
      {
        vsrc[i] = vsrc[i + 1];
      }
    }
  }
}
#endif
// SIMD interpolate Ver N6 generic
template<X86_VEXT vext, int N, bool CLAMP>
static void simdInterpolateVerN6(const int16_t *src, const ptrdiff_t srcStride, int16_t *dst, const ptrdiff_t dstStride,
                                 int width, int height, int shift, int offset, const ClpRng &clpRng,
                                 int16_t const *coeff)
{
  int wdt = width & 0xfffffff8;
  if (wdt >= 8)
  {
    simdInterpolateVerM8<vext, 6, CLAMP>(src, srcStride, dst, dstStride, wdt, height, shift, offset, clpRng, coeff);
    width -= wdt;
    src += wdt;
    dst += wdt;
  }
  wdt = width & 0x7;
  if (wdt >= 4)
  {
    simdInterpolateVerM4<vext, 6, CLAMP>(src, srcStride, dst, dstStride, 4, height, shift, offset, clpRng, coeff);
    width -= 4;
    src += 4;
    dst += 4;
  }
  if (wdt)
  {
    simdInterpolateVerM2<vext, 6, CLAMP>(src, srcStride, dst, dstStride, height, shift, offset, clpRng, coeff);
  }
}

template<int N, bool isLast> inline void interpolate(const int16_t *src, const ptrdiff_t cStride, int16_t *dst,
                                                     int width, int shift, int offset, int bitdepth, int maxVal,
                                                     int16_t const *c)
{
  for (int col = 0; col < width; col++)
  {
    int sum;

    sum = src[col + 0 * cStride] * c[0];
    sum += src[col + 1 * cStride] * c[1];
    if (N >= 4)
    {
      sum += src[col + 2 * cStride] * c[2];
      sum += src[col + 3 * cStride] * c[3];
    }
    if (N >= 6)
    {
      sum += src[col + 4 * cStride] * c[4];
      sum += src[col + 5 * cStride] * c[5];
    }
    if (N == 8)
    {
      sum += src[col + 6 * cStride] * c[6];
      sum += src[col + 7 * cStride] * c[7];
    }

    Pel val = (sum + offset) >> shift;
    if (isLast)
    {
      val = (val < 0) ? 0 : val;
      val = (val > maxVal) ? maxVal : val;
    }
    dst[col] = val;
  }
}

static inline __m128i simdInterpolateLuma2P8(int16_t const *src, const ptrdiff_t srcStride, __m128i *mmCoeff,
                                             const __m128i &mmOffset, int shift)
{
  __m128i sumHi = _mm_setzero_si128();
  __m128i sumLo = _mm_setzero_si128();
  for (int n = 0; n < 2; n++)
  {
    __m128i mmPix = _mm_loadu_si128((__m128i *)src);
    __m128i hi    = _mm_mulhi_epi16(mmPix, mmCoeff[n]);
    __m128i lo    = _mm_mullo_epi16(mmPix, mmCoeff[n]);
    sumHi         = _mm_add_epi32(sumHi, _mm_unpackhi_epi16(lo, hi));
    sumLo         = _mm_add_epi32(sumLo, _mm_unpacklo_epi16(lo, hi));
    src += srcStride;
  }
  sumHi = _mm_srai_epi32(_mm_add_epi32(sumHi, mmOffset), shift);
  sumLo = _mm_srai_epi32(_mm_add_epi32(sumLo, mmOffset), shift);
  return (_mm_packs_epi32(sumLo, sumHi));
}

static inline __m128i simdInterpolateLuma2P4(int16_t const *src, const ptrdiff_t srcStride, __m128i *mmCoeff,
                                             const __m128i &mmOffset, int shift)
{
  __m128i sumHi = _mm_setzero_si128();
  __m128i sumLo = _mm_setzero_si128();
  for (int n = 0; n < 2; n++)
  {
    __m128i mmPix = _mm_loadl_epi64((__m128i *)src);
    __m128i hi    = _mm_mulhi_epi16(mmPix, mmCoeff[n]);
    __m128i lo    = _mm_mullo_epi16(mmPix, mmCoeff[n]);
    sumHi         = _mm_add_epi32(sumHi, _mm_unpackhi_epi16(lo, hi));
    sumLo         = _mm_add_epi32(sumLo, _mm_unpacklo_epi16(lo, hi));
    src += srcStride;
  }
  sumHi = _mm_srai_epi32(_mm_add_epi32(sumHi, mmOffset), shift);
  sumLo = _mm_srai_epi32(_mm_add_epi32(sumLo, mmOffset), shift);
  return (_mm_packs_epi32(sumLo, sumHi));
}

static inline __m128i simdClip3(__m128i mmMin, __m128i mmMax, __m128i mmPix)
{
  __m128i mmMask = _mm_cmpgt_epi16(mmPix, mmMin);
  mmPix          = _mm_or_si128(_mm_and_si128(mmMask, mmPix), _mm_andnot_si128(mmMask, mmMin));
  mmMask         = _mm_cmplt_epi16(mmPix, mmMax);
  mmPix          = _mm_or_si128(_mm_and_si128(mmMask, mmPix), _mm_andnot_si128(mmMask, mmMax));
  return (mmPix);
}

template<X86_VEXT vext, bool isLast>
static void simdInterpolateN2_M8(const int16_t *src, const ptrdiff_t srcStride, int16_t *dst, const ptrdiff_t dstStride,
                                 const ptrdiff_t cStride, int width, int height, int shift, int offset,
                                 const ClpRng &clpRng, int16_t const *c)
{
  int     row, col;
  __m128i mmOffset = _mm_set1_epi32(offset);
  __m128i mmCoeff[2];
  __m128i mmMin = _mm_set1_epi16(clpRng.min);
  __m128i mmMax = _mm_set1_epi16(clpRng.max);
  for (int n = 0; n < 2; n++)
  {
    mmCoeff[n] = _mm_set1_epi16(c[n]);
  }
  for (row = 0; row < height; row++)
  {
    for (col = 0; col < width; col += 8)
    {
      __m128i mmFiltered = simdInterpolateLuma2P8(src + col, cStride, mmCoeff, mmOffset, shift);
      if (isLast)
      {
        mmFiltered = simdClip3(mmMin, mmMax, mmFiltered);
      }
      _mm_storeu_si128((__m128i *)(dst + col), mmFiltered);
    }
    src += srcStride;
    dst += dstStride;
  }
}

template<X86_VEXT vext, bool isLast>
static void simdInterpolateN2_M4(const int16_t *src, const ptrdiff_t srcStride, int16_t *dst, const ptrdiff_t dstStride,
                                 const ptrdiff_t cStride, int width, int height, int shift, int offset,
                                 const ClpRng &clpRng, int16_t const *c)
{
  int     row, col;
  __m128i mmOffset = _mm_set1_epi32(offset);
  __m128i mmCoeff[8];
  __m128i mmMin = _mm_set1_epi16(clpRng.min);
  __m128i mmMax = _mm_set1_epi16(clpRng.max);
  for (int n = 0; n < 2; n++)
  {
    mmCoeff[n] = _mm_set1_epi16(c[n]);
  }
  for (row = 0; row < height; row++)
  {
    for (col = 0; col < width; col += 4)
    {
      __m128i mmFiltered = simdInterpolateLuma2P4(src + col, cStride, mmCoeff, mmOffset, shift);
      if (isLast)
      {
        mmFiltered = simdClip3(mmMin, mmMax, mmFiltered);
      }
      _mm_storel_epi64((__m128i *)(dst + col), mmFiltered);
    }
    src += srcStride;
    dst += dstStride;
  }
}
#ifdef USE_AVX2
static inline __m256i simdInterpolateLuma10Bit2P16(int16_t const *src1, const ptrdiff_t srcStride, __m256i *mmCoeff,
                                                   const __m256i &mmOffset, __m128i &mmShift)
{
  __m256i sumLo;
  __m256i mmPix  = _mm256_loadu_si256((__m256i *)src1);
  __m256i mmPix1 = _mm256_loadu_si256((__m256i *)(src1 + srcStride));
  __m256i lo0    = _mm256_mullo_epi16(mmPix, mmCoeff[0]);
  __m256i lo1    = _mm256_mullo_epi16(mmPix1, mmCoeff[1]);
  sumLo          = _mm256_add_epi16(lo0, lo1);
  sumLo          = _mm256_sra_epi16(_mm256_add_epi16(sumLo, mmOffset), mmShift);
  return (sumLo);
}
#endif

static inline __m128i simdInterpolateLuma10Bit2P8(int16_t const *src1, const ptrdiff_t srcStride, __m128i *mmCoeff,
                                                  const __m128i &mmOffset, __m128i &mmShift)
{
  __m128i sumLo;
  __m128i mmPix  = _mm_loadu_si128((__m128i *)src1);
  __m128i mmPix1 = _mm_loadu_si128((__m128i *)(src1 + srcStride));
  __m128i lo0    = _mm_mullo_epi16(mmPix, mmCoeff[0]);
  __m128i lo1    = _mm_mullo_epi16(mmPix1, mmCoeff[1]);
  sumLo          = _mm_add_epi16(lo0, lo1);
  sumLo          = _mm_sra_epi16(_mm_add_epi16(sumLo, mmOffset), mmShift);
  return (sumLo);
}

static inline __m128i simdInterpolateLuma10Bit2P4(int16_t const *src, const ptrdiff_t srcStride, __m128i *mmCoeff,
                                                  const __m128i &mmOffset, __m128i &mmShift)
{
  __m128i sumLo;
  __m128i mmPix  = _mm_loadl_epi64((__m128i *)src);
  __m128i mmPix1 = _mm_loadl_epi64((__m128i *)(src + srcStride));
  __m128i lo0    = _mm_mullo_epi16(mmPix, mmCoeff[0]);
  __m128i lo1    = _mm_mullo_epi16(mmPix1, mmCoeff[1]);
  sumLo          = _mm_add_epi16(lo0, lo1);
  sumLo          = _mm_sra_epi16(_mm_add_epi16(sumLo, mmOffset), mmShift);
  return sumLo;
}

#ifdef USE_AVX2
static inline __m256i simdInterpolateLumaHighBit2P16(int16_t const *src1, const ptrdiff_t srcStride, __m256i *mmCoeff,
                                                     const __m256i &mmOffset, __m128i &mmShift)
{
  __m256i mm_mul_lo = _mm256_setzero_si256();
  __m256i mm_mul_hi = _mm256_setzero_si256();

  for (int coefIdx = 0; coefIdx < 2; coefIdx++)
  {
    __m256i mmPix = _mm256_lddqu_si256((__m256i *)(src1 + coefIdx * srcStride));
    __m256i mm_hi = _mm256_mulhi_epi16(mmPix, mmCoeff[coefIdx]);
    __m256i mm_lo = _mm256_mullo_epi16(mmPix, mmCoeff[coefIdx]);
    mm_mul_lo     = _mm256_add_epi32(mm_mul_lo, _mm256_unpacklo_epi16(mm_lo, mm_hi));
    mm_mul_hi     = _mm256_add_epi32(mm_mul_hi, _mm256_unpackhi_epi16(mm_lo, mm_hi));
  }
  mm_mul_lo      = _mm256_sra_epi32(_mm256_add_epi32(mm_mul_lo, mmOffset), mmShift);
  mm_mul_hi      = _mm256_sra_epi32(_mm256_add_epi32(mm_mul_hi, mmOffset), mmShift);
  __m256i mm_sum = _mm256_packs_epi32(mm_mul_lo, mm_mul_hi);
  return (mm_sum);
}
#endif

static inline __m128i simdInterpolateLumaHighBit2P8(int16_t const *src1, const ptrdiff_t srcStride, __m128i *mmCoeff,
                                                    const __m128i &mmOffset, __m128i &mmShift)
{
  __m128i mm_mul_lo = _mm_setzero_si128();
  __m128i mm_mul_hi = _mm_setzero_si128();

  for (int coefIdx = 0; coefIdx < 2; coefIdx++)
  {
    __m128i mmPix = _mm_loadu_si128((__m128i *)(src1 + coefIdx * srcStride));
    __m128i mm_hi = _mm_mulhi_epi16(mmPix, mmCoeff[coefIdx]);
    __m128i mm_lo = _mm_mullo_epi16(mmPix, mmCoeff[coefIdx]);
    mm_mul_lo     = _mm_add_epi32(mm_mul_lo, _mm_unpacklo_epi16(mm_lo, mm_hi));
    mm_mul_hi     = _mm_add_epi32(mm_mul_hi, _mm_unpackhi_epi16(mm_lo, mm_hi));
  }
  mm_mul_lo      = _mm_sra_epi32(_mm_add_epi32(mm_mul_lo, mmOffset), mmShift);
  mm_mul_hi      = _mm_sra_epi32(_mm_add_epi32(mm_mul_hi, mmOffset), mmShift);
  __m128i mm_sum = _mm_packs_epi32(mm_mul_lo, mm_mul_hi);
  return (mm_sum);
}

static inline __m128i simdInterpolateLumaHighBit2P4(int16_t const *src1, const ptrdiff_t srcStride, __m128i *mmCoeff,
                                                    const __m128i &mmOffset, __m128i &mmShift)
{
  __m128i mm_sum  = _mm_setzero_si128();
  __m128i mm_zero = _mm_setzero_si128();
  for (int coefIdx = 0; coefIdx < 2; coefIdx++)
  {
    __m128i mmPix  = _mm_loadl_epi64((__m128i *)(src1 + coefIdx * srcStride));
    __m128i mm_hi  = _mm_mulhi_epi16(mmPix, mmCoeff[coefIdx]);
    __m128i mm_lo  = _mm_mullo_epi16(mmPix, mmCoeff[coefIdx]);
    __m128i mm_mul = _mm_unpacklo_epi16(mm_lo, mm_hi);
    mm_sum         = _mm_add_epi32(mm_sum, mm_mul);
  }
  mm_sum = _mm_sra_epi32(_mm_add_epi32(mm_sum, mmOffset), mmShift);
  mm_sum = _mm_packs_epi32(mm_sum, mm_zero);
  return (mm_sum);
}

template<X86_VEXT vext, bool isLast>
static void simdInterpolateN2_HIGHBIT_M4(const int16_t *src, const ptrdiff_t srcStride, int16_t *dst,
                                         const ptrdiff_t dstStride, const ptrdiff_t cStride, int width, int height,
                                         int shift, int offset, const ClpRng &clpRng, int16_t const *c)
{
#if USE_AVX2
  __m256i mm256Offset = _mm256_set1_epi32(offset);
  __m256i mm256Coeff[2];
  for (int n = 0; n < 2; n++)
  {
    mm256Coeff[n] = _mm256_set1_epi16(c[n]);
  }
#endif
  __m128i mmOffset = _mm_set1_epi32(offset);
  __m128i mmCoeff[2];
  for (int n = 0; n < 2; n++)
  {
    mmCoeff[n] = _mm_set1_epi16(c[n]);
  }

  __m128i mmShift = _mm_cvtsi64_si128(shift);

  CHECK(width % 4 != 0, "Not Supported");

  for (int row = 0; row < height; row++)
  {
    int col = 0;
#if USE_AVX2
    for (; col < ((width >> 4) << 4); col += 16)
    {
      __m256i mmFiltered = simdInterpolateLumaHighBit2P16(src + col, cStride, mm256Coeff, mm256Offset, mmShift);
      _mm256_storeu_si256((__m256i *)(dst + col), mmFiltered);
    }
#endif
    for (; col < ((width >> 3) << 3); col += 8)
    {
      __m128i mmFiltered = simdInterpolateLumaHighBit2P8(src + col, cStride, mmCoeff, mmOffset, mmShift);
      _mm_storeu_si128((__m128i *)(dst + col), mmFiltered);
    }

    for (; col < ((width >> 2) << 2); col += 4)
    {
      __m128i mmFiltered = simdInterpolateLumaHighBit2P4(src + col, cStride, mmCoeff, mmOffset, mmShift);
      _mm_storel_epi64((__m128i *)(dst + col), mmFiltered);
    }
    src += srcStride;
    dst += dstStride;
  }
}

template<X86_VEXT vext, bool isLast>
static void simdInterpolateN2_10BIT_M4(const int16_t *src, const ptrdiff_t srcStride, int16_t *dst,
                                       const ptrdiff_t dstStride, const ptrdiff_t cStride, int width, int height,
                                       int shift, int offset, const ClpRng &clpRng, int16_t const *c)
{
  int     row, col;
  __m128i mmOffset = _mm_set1_epi16(offset);
  __m128i mmShift  = _mm_set_epi64x(0, shift);
  __m128i mmCoeff[2];
  for (int n = 0; n < 2; n++)
  {
    mmCoeff[n] = _mm_set1_epi16(c[n]);
  }

#if USE_AVX2
  __m256i mm256Offset = _mm256_set1_epi16(offset);
  __m256i mm256Coeff[2];
  for (int n = 0; n < 2; n++)
  {
    mm256Coeff[n] = _mm256_set1_epi16(c[n]);
  }
#endif
  for (row = 0; row < height; row++)
  {
    col = 0;
#if USE_AVX2
    // multiple of 16
    for (; col < ((width >> 4) << 4); col += 16)
    {
      __m256i mmFiltered = simdInterpolateLuma10Bit2P16(src + col, cStride, mm256Coeff, mm256Offset, mmShift);
      _mm256_storeu_si256((__m256i *)(dst + col), mmFiltered);
    }
#endif
    // multiple of 8
    for (; col < ((width >> 3) << 3); col += 8)
    {
      __m128i mmFiltered = simdInterpolateLuma10Bit2P8(src + col, cStride, mmCoeff, mmOffset, mmShift);
      _mm_storeu_si128((__m128i *)(dst + col), mmFiltered);
    }

    // last 4 samples
    __m128i mmFiltered = simdInterpolateLuma10Bit2P4(src + col, cStride, mmCoeff, mmOffset, mmShift);
    _mm_storel_epi64((__m128i *)(dst + col), mmFiltered);
    src += srcStride;
    dst += dstStride;
  }
}

template<X86_VEXT vext, int N, bool VERTICAL, bool FIRST, bool LAST, bool biMCForDMVR>
static void simdFilter(const ClpRng &clpRng, Pel const *src, const ptrdiff_t srcStride, Pel *dst,
                       const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *coeff)
{
  int row, col;

#if IF_12TAP_SIMD
  Pel c[16] = {
    0,
  };
#else
  Pel c[8];
#endif
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
#if IF_12TAP_SIMD
  if (N >= 8)
  {
    c[6] = coeff[6];
    c[7] = coeff[7];
  }
  if (N >= 10)
  {
    c[8] = coeff[8];
    c[9] = coeff[9];
  }
  if (N == 12)
  {
    c[10] = coeff[10];
    c[11] = coeff[11];
    c[12] = c[8];
    c[13] = c[9];
    c[14] = c[10];
    c[15] = c[11];
  }
#else
  if (N == 8)
  {
    c[6] = coeff[6];
    c[7] = coeff[7];
  }
#endif

  const ptrdiff_t cStride = (VERTICAL) ? srcStride : 1;
  src -= (N / 2 - 1) * cStride;

  int offset;
  int headRoom = IF_INTERNAL_FRAC_BITS(clpRng.bd);
  int shift    = IF_FILTER_PREC;

  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20

  if (biMCForDMVR)
  {
    if (FIRST)
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
  else
  {
    if (LAST)
    {
      shift += (FIRST) ? 0 : headRoom;
      offset = 1 << (shift - 1);
      offset += (FIRST) ? 0 : IF_INTERNAL_OFFS << IF_FILTER_PREC;
    }
    else
    {
      shift -= (FIRST) ? headRoom : 0;
      offset = (FIRST) ? -(IF_INTERNAL_OFFS << shift) : 0;
    }
  }

  const bool widthMult8 = (width & 7) == 0;
  const bool widthMult4 = (width & 3) == 0;

  {
#if IF_12TAP_SIMD
#if USE_AVX2
    if (N == 12 && width % 16 == 0)
    {
      if (!VERTICAL)
      {
        simdInterpolateHorMx_12tap<vext, 12, LAST, 16>(src, srcStride, dst, dstStride, width, height, shift, offset,
                                                       clpRng, c);
      }
      else
      {
        simdInterpolateVerM16_12tap<vext, 12, LAST>(src, srcStride, dst, dstStride, width, height, shift, offset,
                                                    clpRng, c);
      }
      return;
    }
    if (N == 12 && widthMult8)
    {
      if (!VERTICAL)
      {
        simdInterpolateHorMx_12tap<vext, 12, LAST, 8>(src, srcStride, dst, dstStride, width, height, shift, offset,
                                                      clpRng, c);
      }
      else
      {
        simdInterpolateVerM8_12tap<vext, 12, LAST>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng,
                                                   c);
      }
      return;
    }
    else
#endif
      if (N == 12 && widthMult4)
    {
      if (!VERTICAL)
      {
        simdInterpolateHorM4_12tap<vext, 12, LAST>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng,
                                                   c);
      }
      else
      {
        simdInterpolateVerM4_12tap<vext, 12, LAST>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng,
                                                   c);
      }
      return;
    }
    else if (N == 12 && width > 4)
    {
      if (!VERTICAL)
      {
        simdInterpolateHorNonM4_12tap<vext, 12, LAST>(src, srcStride, dst, dstStride, width, height, shift, offset,
                                                      clpRng, c);
      }
      else
      {
        simdInterpolateVerNonM4_12tap<vext, 12, LAST>(src, srcStride, dst, dstStride, width, height, shift, offset,
                                                      clpRng, c);
      }
      return;
    }
    else if (N == 12 && !VERTICAL)
    {
      simdInterpolateHorNonM4_12tap<vext, 12, LAST>(src, srcStride, dst, dstStride, width, height, shift, offset,
                                                    clpRng, c);
      return;
    }
    else
#endif
      if ((N == 8 || N == 6) && widthMult8)
    {
      if (!VERTICAL)
      {
#ifdef USE_AVX2
        if (vext >= AVX2)
        {
          simdInterpolateHorM8_AVX2<vext, N, LAST>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng,
                                                   c);
        }
        else
#endif
        {
          simdInterpolateHorM8<vext, N, LAST>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
        }
      }
      else
      {
#ifdef USE_AVX2
        if (vext >= AVX2)
        {
          simdInterpolateVerM8_AVX2<vext, N, LAST>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng,
                                                   c);
        }
        else
#endif
        {
          simdInterpolateVerM8<vext, N, LAST>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
        }
      }
      return;
    }
    else if ((N == 8 || N == 6) && widthMult4)
    {
      if (!VERTICAL)
      {
        simdInterpolateHorM4<vext, N, LAST>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
      }
      else
      {
        simdInterpolateVerM4<vext, N, LAST>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
      }
      return;
    }
    else if (N == 4 && widthMult4)
    {
      if (!VERTICAL)
      {
        if (widthMult8)
        {
#ifdef USE_AVX2
          if (vext >= AVX2)
          {
            simdInterpolateHorM8_AVX2<vext, 4, LAST>(src, srcStride, dst, dstStride, width, height, shift, offset,
                                                     clpRng, c);
          }
          else
#endif
          {
            simdInterpolateHorM8<vext, 4, LAST>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng,
                                                c);
          }
        }
        else
        {
          simdInterpolateHorM4<vext, 4, LAST>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
        }
      }
      else
      {
        simdInterpolateVerM4<vext, 4, LAST>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
      }
      return;
    }
    else if (biMCForDMVR)
    {
      if (N == 2 && widthMult4)
      {
        if (clpRng.bd <= 10)
        {
          simdInterpolateN2_10BIT_M4<vext, LAST>(src, srcStride, dst, dstStride, cStride, width, height, shift, offset,
                                                 clpRng, c);
        }
        else
        {
          simdInterpolateN2_HIGHBIT_M4<vext, LAST>(src, srcStride, dst, dstStride, cStride, width, height, shift,
                                                   offset, clpRng, c);
        }
        return;
      }
    }
    else if (N == 2 && widthMult8)
    {
      simdInterpolateN2_M8<vext, LAST>(src, srcStride, dst, dstStride, cStride, width, height, shift, offset, clpRng,
                                       c);
      return;
    }
    else if (N == 2 && widthMult4)
    {
      simdInterpolateN2_M4<vext, LAST>(src, srcStride, dst, dstStride, cStride, width, height, shift, offset, clpRng,
                                       c);
      return;
    }
    if (N == 6)
    {
      if (width == 2)
      {
        if (!VERTICAL)
        {
          simdInterpolateHorM2<vext, 6, LAST>(src, srcStride, dst, dstStride, height, shift, offset, clpRng, c);
        }
        else
        {
          simdInterpolateVerM2<vext, 6, LAST>(src, srcStride, dst, dstStride, height, shift, offset, clpRng, c);
        }
        return;
      }
      else if (width > 1)
      {
        if (!VERTICAL)
        {
          simdInterpolateHorN6<vext, 6, LAST>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
        }
        else
        {
          simdInterpolateVerN6<vext, 6, LAST>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
        }
        return;
      }
    }
  }
  for (row = 0; row < height; row++)
  {
    for (col = 0; col < width; col++)
    {
      int sum;

      sum = src[col + 0 * cStride] * c[0];
      sum += src[col + 1 * cStride] * c[1];
      if (N >= 4)
      {
        sum += src[col + 2 * cStride] * c[2];
        sum += src[col + 3 * cStride] * c[3];
      }
      if (N >= 6)
      {
        sum += src[col + 4 * cStride] * c[4];
        sum += src[col + 5 * cStride] * c[5];
      }
#if IF_12TAP_SIMD
      if (N >= 8)
      {
        sum += src[col + 6 * cStride] * c[6];
        sum += src[col + 7 * cStride] * c[7];
      }
      if (N >= 10)
      {
        sum += src[col + 8 * cStride] * c[8];
        sum += src[col + 9 * cStride] * c[9];
      }
      if (N == 12)
      {
        sum += src[col + 10 * cStride] * c[10];
        sum += src[col + 11 * cStride] * c[11];
      }
#else
      if (N == 8)
      {
        sum += src[col + 6 * cStride] * c[6];
        sum += src[col + 7 * cStride] * c[7];
      }
#endif

      Pel val = (sum + offset) >> shift;
      if (LAST)
      {
        val = ClipPel(val, clpRng);
      }
      dst[col] = val;
    }

    src += srcStride;
    dst += dstStride;
  }
}

template<X86_VEXT vext> int xSadTM_SSE(const CodingUnit &cu, const int width, const int height, const int templateWidth,
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

  if (width == 4)
  {
    __m128i vzero  = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    // for luma, to be confirmed
    for (int y = 0; y < templateHeight; y++)
    {
      __m128i vPred = _mm_loadl_epi64((__m128i *)(piPred));
      __m128i vRec  = _mm_loadl_epi64((__m128i *)(piRec));
      __m128i vAd   = _mm_abs_epi16(_mm_sub_epi16(vRec, vPred));
      _mm_storel_epi64((__m128i *)(piAd), vAd);
      __m128i vsumtemp = _mm_unpacklo_epi16(vAd, vzero);
      vsum32           = _mm_add_epi32(vsum32, vsumtemp);
      piPred += iPredStride;
      piAd += iAdStride;
      piRec += iRecStride;
    }
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    sad    = _mm_cvtsi128_si32(vsum32);
  }
#if USE_AVX2
  else if (0 == (width % 16))
  {
    __m256i vzero  = _mm256_setzero_si256();
    __m256i vsum32 = vzero;
    for (int y = 0; y < templateHeight; y++)
    {
      __m256i vsum16 = vzero;
      for (int x = 0; x < width; x += 16)
      {
        __m256i vPred = _mm256_lddqu_si256((__m256i *)(piPred + x));   // why not aligned with 128/256 bit boundaries
        __m256i vRec  = _mm256_lddqu_si256((__m256i *)(piRec + x));
        __m256i vAd   = _mm256_abs_epi16(_mm256_sub_epi16(vRec, vPred));
        _mm256_storeu_si256((__m256i *)(piAd + x), vAd);

        vsum16 = _mm256_add_epi16(vsum16, vAd);
      }
      __m256i vsumtemp = _mm256_add_epi32(_mm256_unpacklo_epi16(vsum16, vzero), _mm256_unpackhi_epi16(vsum16, vzero));
      vsum32           = _mm256_add_epi32(vsum32, vsumtemp);
      piPred += iPredStride;
      piAd += iAdStride;
      piRec += iRecStride;
    }
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    sad    = _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum32)) +
      _mm_cvtsi128_si32(_mm256_castsi256_si128(_mm256_permute2x128_si256(vsum32, vsum32, 0x11)));
  }
#endif
  else
  {
    __m128i vzero  = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    for (int y = 0; y < templateHeight; y++)
    {
      __m128i vsum16 = vzero;
      for (int x = 0; x < width; x += 8)
      {
        __m128i vPred = _mm_lddqu_si128((__m128i *)(piPred + x));
        __m128i vRec  = _mm_lddqu_si128((__m128i *)(piRec + x));
        __m128i vAd   = _mm_abs_epi16(_mm_sub_epi16(vRec, vPred));
        _mm_storeu_si128((__m128i *)(piAd + x), vAd);
        vsum16 = _mm_add_epi16(vsum16, vAd);
      }
      __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
      vsum32           = _mm_add_epi32(vsum32, vsumtemp);
      piPred += iPredStride;
      piAd += iAdStride;
      piRec += iRecStride;
    }
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    sad    = _mm_cvtsi128_si32(vsum32);
  }

  // left template
  piPred = predBuf.buf + templateHeight * iPredStride;
  // start point of predBuf is (-templateWidth, -templateHeight) of current block
  piAd   = adBuf.buf + templateHeight * iAdStride;
  piRec  = recBuf.buf - templateWidth;   // start point of recBuf is (0,0) of current block

  if (templateWidth == 4)
  {
    __m128i vzero  = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    // for luma, to be confirmed
    for (int y = 0; y < height; y++)
    {
      __m128i vPred = _mm_loadl_epi64((__m128i *)(piPred));
      __m128i vRec  = _mm_loadl_epi64((__m128i *)(piRec));
      __m128i vAd   = _mm_abs_epi16(_mm_sub_epi16(vRec, vPred));
      _mm_storel_epi64((__m128i *)(piAd), vAd);
      __m128i vsumtemp = _mm_unpacklo_epi16(vAd, vzero);
      vsum32           = _mm_add_epi32(vsum32, vsumtemp);
      piPred += iPredStride;
      piAd += iAdStride;
      piRec += iRecStride;
    }
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    sad += _mm_cvtsi128_si32(vsum32);
  }
#if USE_AVX2
  else if (0 == (templateWidth % 16))
  {
    __m256i vzero  = _mm256_setzero_si256();
    __m256i vsum32 = vzero;
    for (int y = 0; y < height; y++)
    {
      __m256i vsum16 = vzero;
      for (int x = 0; x < templateWidth; x += 16)
      {
        __m256i vPred = _mm256_lddqu_si256((__m256i *)(piPred + x));   // why not aligned with 128/256 bit boundaries
        __m256i vRec  = _mm256_lddqu_si256((__m256i *)(piRec + x));
        __m256i vAd   = _mm256_abs_epi16(_mm256_sub_epi16(vRec, vPred));
        _mm256_storeu_si256((__m256i *)(piAd + x), vAd);

        vsum16 = _mm256_add_epi16(vsum16, vAd);
      }
      __m256i vsumtemp = _mm256_add_epi32(_mm256_unpacklo_epi16(vsum16, vzero), _mm256_unpackhi_epi16(vsum16, vzero));
      vsum32           = _mm256_add_epi32(vsum32, vsumtemp);
      piPred += iPredStride;
      piAd += iAdStride;
      piRec += iRecStride;
    }
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    sad += _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum32)) +
      _mm_cvtsi128_si32(_mm256_castsi256_si128(_mm256_permute2x128_si256(vsum32, vsum32, 0x11)));
  }
#endif
  else if (0 == (templateWidth % 8))
  {
    __m128i vzero  = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    for (int y = 0; y < height; y++)
    {
      __m128i vsum16 = vzero;
      for (int x = 0; x < templateWidth; x += 8)
      {
        __m128i vPred = _mm_lddqu_si128((__m128i *)(piPred + x));
        __m128i vRec  = _mm_lddqu_si128((__m128i *)(piRec + x));
        __m128i vAd   = _mm_abs_epi16(_mm_sub_epi16(vRec, vPred));
        _mm_storeu_si128((__m128i *)(piAd + x), vAd);
        vsum16 = _mm_add_epi16(vsum16, vAd);
      }
      __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
      vsum32           = _mm_add_epi32(vsum32, vsumtemp);
      piPred += iPredStride;
      piAd += iAdStride;
      piRec += iRecStride;
    }
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    sad += _mm_cvtsi128_si32(vsum32);
  }
  else
  {
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
  }

  return sad;
}

template<X86_VEXT vext> int xSgpmSadTM_SSE(const CodingUnit &cu, const int width, const int height,
                                           const int templateWidth, const int templateHeight, const CompID compIdx,
                                           const uint8_t splitDir, PelBuf &adBuf)
{
  int      sum        = 0;
  int16_t  wIdx       = floorLog2(cu.lwidth()) - GEO_MIN_CU_LOG2_EX;
  int16_t  hIdx       = floorLog2(cu.lheight()) - GEO_MIN_CU_LOG2_EX;
  int16_t  angle      = g_geoParams[splitDir].angleIdx;
  int16_t  stepY      = 0;
  int16_t  stepX      = 1;
  int16_t *weightMask = nullptr;

  if (g_angle2mirror[angle] == 2)
  {
    stepY      = -GEO_WEIGHT_MASK_SIZE_EXT;
    weightMask = &g_globalGeoWeightsTpl[g_angle2mask[angle]]
                                       [(GEO_WEIGHT_MASK_SIZE_EXT - 1 - g_weightOffsetEx[splitDir][hIdx][wIdx][1] -
                                         GEO_TM_ADDED_WEIGHT_MASK_SIZE) *
                                          GEO_WEIGHT_MASK_SIZE_EXT +
                                        g_weightOffsetEx[splitDir][hIdx][wIdx][0] + GEO_TM_ADDED_WEIGHT_MASK_SIZE];
  }
  else if (g_angle2mirror[angle] == 1)
  {
    stepX      = -1;
    stepY      = GEO_WEIGHT_MASK_SIZE_EXT;
    weightMask = &g_globalGeoWeightsTpl[g_angle2mask[angle]]
                                       [(g_weightOffsetEx[splitDir][hIdx][wIdx][1] + GEO_TM_ADDED_WEIGHT_MASK_SIZE) *
                                          GEO_WEIGHT_MASK_SIZE_EXT +
                                        (GEO_WEIGHT_MASK_SIZE_EXT - 1 - g_weightOffsetEx[splitDir][hIdx][wIdx][0] -
                                         GEO_TM_ADDED_WEIGHT_MASK_SIZE)];
  }
  else
  {
    stepY      = GEO_WEIGHT_MASK_SIZE_EXT;
    weightMask = &g_globalGeoWeightsTpl[g_angle2mask[angle]]
                                       [(g_weightOffsetEx[splitDir][hIdx][wIdx][1] + GEO_TM_ADDED_WEIGHT_MASK_SIZE) *
                                          GEO_WEIGHT_MASK_SIZE_EXT +
                                        g_weightOffsetEx[splitDir][hIdx][wIdx][0] + GEO_TM_ADDED_WEIGHT_MASK_SIZE];
  }

  ptrdiff_t iAdStride = adBuf.stride;

  if (compIdx != COMP_Y && cu.chromaFormat == ChromaFormat::_420)
  {
    stepY <<= 1;
  }

  // top template
  Pel *piAd = adBuf.buf + templateWidth;   // start point of adBuf is (-templateWidth, -templateHeight) of current block
  int16_t *weightBackup = weightMask;
  weightMask            = weightMask - templateHeight * stepY;
  if (width == 4)
  {
    __m128i vzero  = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    for (int y = 0; y < templateHeight; y++)
    {
      __m128i vAd = _mm_loadl_epi64((__m128i *)(piAd));
      __m128i vMask;

      if (g_angle2mirror[angle] == 1)
      {
        vMask                      = _mm_loadl_epi64((__m128i *)(weightMask - (4 - 1)));
        const __m128i shuffle_mask = _mm_set_epi8(15, 14, 13, 12, 11, 10, 9, 8, 1, 0, 3, 2, 5, 4, 7, 6);
        vMask                      = _mm_shuffle_epi8(vMask, shuffle_mask);
      }
      else
      {
        vMask = _mm_loadl_epi64((__m128i *)weightMask);
      }
      vsum32 = _mm_add_epi32(vsum32, _mm_madd_epi16(vMask, vAd));

      piAd += iAdStride;
      weightMask += stepY;
    }
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    sum    = _mm_cvtsi128_si32(vsum32);
  }
#if USE_AVX2
  else if (0 == (width % 16))
  {
    __m256i vzero  = _mm256_setzero_si256();
    __m256i vsum32 = vzero;
    for (int y = 0; y < templateHeight; y++)
    {
      for (int x = 0; x < width; x += 16)
      {
        __m256i vAd = _mm256_lddqu_si256((__m256i *)(piAd + x));

        __m256i vMask;

        if (g_angle2mirror[angle] == 1)
        {
          vMask                      = _mm256_lddqu_si256((__m256i *)(weightMask - x - (16 - 1)));
          const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3, 2,
                                                       5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
          vMask                      = _mm256_shuffle_epi8(vMask, shuffle_mask);
          vMask                      = _mm256_permute4x64_epi64(vMask, _MM_SHUFFLE(1, 0, 3, 2));
        }
        else
        {
          vMask = _mm256_lddqu_si256((__m256i *)(weightMask + x));
        }
        vsum32 = _mm256_add_epi32(vsum32, _mm256_madd_epi16(vMask, vAd));
      }
      piAd += iAdStride;
      weightMask += stepY;
    }
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    sum    = _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum32)) +
      _mm_cvtsi128_si32(_mm256_castsi256_si128(_mm256_permute2x128_si256(vsum32, vsum32, 0x11)));
  }
#endif
  else
  {
    __m128i vzero  = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    for (int y = 0; y < templateHeight; y++)
    {
      for (int x = 0; x < width; x += 8)
      {
        __m128i vAd = _mm_lddqu_si128((__m128i *)(piAd + x));
        __m128i vMask;

        if (g_angle2mirror[angle] == 1)
        {
          vMask                      = _mm_lddqu_si128((__m128i *)(weightMask - x - (8 - 1)));
          const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
          vMask                      = _mm_shuffle_epi8(vMask, shuffle_mask);
        }
        else
        {
          vMask = _mm_lddqu_si128((__m128i *)(weightMask + x));
        }
        vsum32 = _mm_add_epi32(vsum32, _mm_madd_epi16(vMask, vAd));
      }
      piAd += iAdStride;
      weightMask += stepY;
    }
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    sum    = _mm_cvtsi128_si32(vsum32);
  }

  // left template
  piAd       = adBuf.buf + templateHeight * iAdStride;
  weightMask = weightBackup - templateWidth * stepX;
  if (templateWidth == 4)
  {
    __m128i vzero  = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    for (int y = 0; y < height; y++)
    {
      __m128i vAd = _mm_loadl_epi64((__m128i *)(piAd));
      __m128i vMask;

      if (g_angle2mirror[angle] == 1)
      {
        vMask                      = _mm_loadl_epi64((__m128i *)(weightMask - (4 - 1)));
        const __m128i shuffle_mask = _mm_set_epi8(15, 14, 13, 12, 11, 10, 9, 8, 1, 0, 3, 2, 5, 4, 7, 6);
        vMask                      = _mm_shuffle_epi8(vMask, shuffle_mask);
      }
      else
      {
        vMask = _mm_loadl_epi64((__m128i *)weightMask);
      }
      vsum32 = _mm_add_epi32(vsum32, _mm_madd_epi16(vMask, vAd));

      piAd += iAdStride;
      weightMask += stepY;
    }
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    sum += _mm_cvtsi128_si32(vsum32);
  }
#if USE_AVX2
  else if (0 == (templateWidth % 16))
  {
    __m256i vzero  = _mm256_setzero_si256();
    __m256i vsum32 = vzero;
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < templateWidth; x += 16)
      {
        __m256i vAd = _mm256_lddqu_si256((__m256i *)(piAd + x));

        __m256i vMask;

        if (g_angle2mirror[angle] == 1)
        {
          vMask                      = _mm256_lddqu_si256((__m256i *)(weightMask - x - (16 - 1)));
          const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3, 2,
                                                       5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
          vMask                      = _mm256_shuffle_epi8(vMask, shuffle_mask);
          vMask                      = _mm256_permute4x64_epi64(vMask, _MM_SHUFFLE(1, 0, 3, 2));
        }
        else
        {
          vMask = _mm256_lddqu_si256((__m256i *)(weightMask + x));
        }
        vsum32 = _mm256_add_epi32(vsum32, _mm256_madd_epi16(vMask, vAd));
      }
      piAd += iAdStride;
      weightMask += stepY;
    }
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    sum += _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum32)) +
      _mm_cvtsi128_si32(_mm256_castsi256_si128(_mm256_permute2x128_si256(vsum32, vsum32, 0x11)));
  }
#endif
  else if (0 == (templateWidth % 8))
  {
    __m128i vzero  = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < templateWidth; x += 8)
      {
        __m128i vAd = _mm_lddqu_si128((__m128i *)(piAd + x));
        __m128i vMask;

        if (g_angle2mirror[angle] == 1)
        {
          vMask                      = _mm_lddqu_si128((__m128i *)(weightMask - x - (8 - 1)));
          const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
          vMask                      = _mm_shuffle_epi8(vMask, shuffle_mask);
        }
        else
        {
          vMask = _mm_lddqu_si128((__m128i *)(weightMask + x));
        }
        vsum32 = _mm_add_epi32(vsum32, _mm_madd_epi16(vMask, vAd));
      }
      piAd += iAdStride;
      weightMask += stepY;
    }
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    sum += _mm_cvtsi128_si32(vsum32);
  }
  else
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < templateWidth; x++)
      {
        sum += *piAd * (*weightMask);
        piAd++;
        weightMask += stepX;
      }

      piAd += (iAdStride - templateWidth);
      weightMask += (stepY - templateWidth * stepX);
    }
  }

  return sum;
}

template<X86_VEXT vext> void xWeightedSgpm_SSE(const CodingUnit &cu, const uint32_t width, const uint32_t height,
                                               const CompID compIdx, const uint8_t splitDir, PelBuf &predDst,
                                               PelBuf &predSrc0, PelBuf &predSrc1)
{
  Pel      *dst        = predDst.buf;
  Pel      *src0       = predSrc0.buf;
  Pel      *src1       = predSrc1.buf;
  ptrdiff_t strideDst  = predDst.stride;
  ptrdiff_t strideSrc0 = predSrc0.stride;
  ptrdiff_t strideSrc1 = predSrc1.stride;

  // const char   log2WeightBase = 3;
  const ClpRng &clpRng = cu.slice->clpRng(compIdx);

  const int32_t shiftWeighted  = 5;
  const int32_t offsetWeighted = 16;
  int16_t       wIdx           = floorLog2(cu.lwidth()) - GEO_MIN_CU_LOG2_EX;
  int16_t       hIdx           = floorLog2(cu.lheight()) - GEO_MIN_CU_LOG2_EX;
  int16_t       angle          = g_geoParams[splitDir].angleIdx;
  int16_t       stepY          = 0;
  int16_t      *weight         = nullptr;

  int blendWIdx = 0;
  if (!cu.cs->pps->m_useSgpmNoBlend)
  {
    blendWIdx = GET_SGPM_BLD_IDX(cu.lwidth(), cu.lheight());
  }

  if (g_angle2mirror[angle] == 2)
  {
    stepY  = -GEO_WEIGHT_MASK_SIZE;
    weight = &g_globalGeoWeights[blendWIdx][g_angle2mask[angle]]
                                [(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffsetEx[splitDir][hIdx][wIdx][1]) *
                                   GEO_WEIGHT_MASK_SIZE +
                                 g_weightOffsetEx[splitDir][hIdx][wIdx][0]];
  }
  else if (g_angle2mirror[angle] == 1)
  {
    stepY  = GEO_WEIGHT_MASK_SIZE;
    weight = &g_globalGeoWeights[blendWIdx][g_angle2mask[angle]]
                                [g_weightOffsetEx[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE +
                                 (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffsetEx[splitDir][hIdx][wIdx][0])];
  }
  else
  {
    stepY  = GEO_WEIGHT_MASK_SIZE;
    weight = &g_globalGeoWeights[blendWIdx][g_angle2mask[angle]]
                                [g_weightOffsetEx[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE +
                                 g_weightOffsetEx[splitDir][hIdx][wIdx][0]];
  }
  const __m128i mmEight  = _mm_set1_epi16(32);
  const __m128i mmOffset = _mm_set1_epi32(offsetWeighted);
  const __m128i mmShift  = _mm_cvtsi32_si128(shiftWeighted);
  const __m128i mmMin    = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax    = _mm_set1_epi16(clpRng.max);

  if (compIdx != COMP_Y && cu.chromaFormat == ChromaFormat::_420)
  {
    stepY <<= 1;
  }
  if (width == 4)
  {
    // for luma, to be confirmed
    for (int y = 0; y < height; y++)
    {
      __m128i s0 = _mm_loadl_epi64((__m128i *)(src0));
      __m128i s1 = _mm_loadl_epi64((__m128i *)(src1));
      __m128i w0;
      if (compIdx != COMP_Y && cu.chromaFormat != ChromaFormat::_444)
      {
        if (g_angle2mirror[angle] == 1)
        {
          w0                         = _mm_loadu_si128((__m128i *)(weight - (8 - 1)));
          const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
          w0                         = _mm_shuffle_epi8(w0, shuffle_mask);
        }
        else
        {
          w0 = _mm_loadu_si128((__m128i *)(weight));
        }
        w0 = _mm_shuffle_epi8(w0, _mm_setr_epi8(0, 1, 4, 5, 8, 9, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0));
      }
      else
      {
        if (g_angle2mirror[angle] == 1)
        {
          w0                         = _mm_loadl_epi64((__m128i *)(weight - (4 - 1)));
          const __m128i shuffle_mask = _mm_set_epi8(15, 14, 13, 12, 11, 10, 9, 8, 1, 0, 3, 2, 5, 4, 7, 6);
          w0                         = _mm_shuffle_epi8(w0, shuffle_mask);
        }
        else
        {
          w0 = _mm_loadl_epi64((__m128i *)weight);
        }
      }

      __m128i w1 = _mm_sub_epi16(mmEight, w0);
      s0         = _mm_unpacklo_epi16(s0, s1);
      w0         = _mm_unpacklo_epi16(w0, w1);
      s0         = _mm_add_epi32(_mm_madd_epi16(s0, w0), mmOffset);
      s0         = _mm_sra_epi32(s0, mmShift);
      s0         = _mm_packs_epi32(s0, s0);
      s0         = _mm_min_epi16(mmMax, _mm_max_epi16(s0, mmMin));
      _mm_storel_epi64((__m128i *)(dst), s0);
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
#if USE_AVX2
  else if (0 == (width % 16))
  {
    const __m256i mmEightAVX2  = _mm256_set1_epi16(32);
    const __m256i mmOffsetAVX2 = _mm256_set1_epi32(offsetWeighted);
    const __m256i mmMinAVX2    = _mm256_set1_epi16(clpRng.min);
    const __m256i mmMaxAVX2    = _mm256_set1_epi16(clpRng.max);
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 16)
      {
        __m256i s0 = _mm256_lddqu_si256((__m256i *)(src0 + x));   // why not aligned with 128/256 bit boundaries
        __m256i s1 = _mm256_lddqu_si256((__m256i *)(src1 + x));

        __m256i w0 = _mm256_lddqu_si256((__m256i *)(weight + x));
        if (compIdx != COMP_Y && cu.chromaFormat != ChromaFormat::_444)
        {
          const __m256i mask = _mm256_set_epi16(0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1);
          __m256i       w0p0, w0p1;
          if (g_angle2mirror[angle] == 1)
          {
            w0p0 =
              _mm256_lddqu_si256((__m256i *)(weight - (x << 1) - (16 - 1)));   // first sub-sample the required weights.
            w0p1                       = _mm256_lddqu_si256((__m256i *)(weight - (x << 1) - 16 - (16 - 1)));
            const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3,
                                                         2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0p0                       = _mm256_shuffle_epi8(w0p0, shuffle_mask);
            w0p0                       = _mm256_permute4x64_epi64(w0p0, _MM_SHUFFLE(1, 0, 3, 2));
            w0p1                       = _mm256_shuffle_epi8(w0p1, shuffle_mask);
            w0p1                       = _mm256_permute4x64_epi64(w0p1, _MM_SHUFFLE(1, 0, 3, 2));
          }
          else
          {
            w0p0 = _mm256_lddqu_si256((__m256i *)(weight + (x << 1)));   // first sub-sample the required weights.
            w0p1 = _mm256_lddqu_si256((__m256i *)(weight + (x << 1) + 16));
          }
          w0p0 = _mm256_mullo_epi16(w0p0, mask);
          w0p1 = _mm256_mullo_epi16(w0p1, mask);
          w0   = _mm256_packs_epi16(w0p0, w0p1);
          w0   = _mm256_permute4x64_epi64(w0, _MM_SHUFFLE(3, 1, 2, 0));
        }
        else
        {
          if (g_angle2mirror[angle] == 1)
          {
            w0                         = _mm256_lddqu_si256((__m256i *)(weight - x - (16 - 1)));
            const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3,
                                                         2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0                         = _mm256_shuffle_epi8(w0, shuffle_mask);
            w0                         = _mm256_permute4x64_epi64(w0, _MM_SHUFFLE(1, 0, 3, 2));
          }
          else
          {
            w0 = _mm256_lddqu_si256((__m256i *)(weight + x));
          }
        }
        __m256i w1 = _mm256_sub_epi16(mmEightAVX2, w0);

        __m256i s0tmp = _mm256_unpacklo_epi16(s0, s1);
        __m256i w0tmp = _mm256_unpacklo_epi16(w0, w1);
        s0tmp         = _mm256_add_epi32(_mm256_madd_epi16(s0tmp, w0tmp), mmOffsetAVX2);
        s0tmp         = _mm256_sra_epi32(s0tmp, mmShift);

        s0 = _mm256_unpackhi_epi16(s0, s1);
        w0 = _mm256_unpackhi_epi16(w0, w1);
        s0 = _mm256_add_epi32(_mm256_madd_epi16(s0, w0), mmOffsetAVX2);
        s0 = _mm256_sra_epi32(s0, mmShift);

        s0 = _mm256_packs_epi32(s0tmp, s0);
        s0 = _mm256_min_epi16(mmMaxAVX2, _mm256_max_epi16(s0, mmMinAVX2));
        _mm256_storeu_si256((__m256i *)(dst + x), s0);
      }
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
#endif
  else
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 8)
      {
        __m128i s0 = _mm_lddqu_si128((__m128i *)(src0 + x));
        __m128i s1 = _mm_lddqu_si128((__m128i *)(src1 + x));
        __m128i w0;
        if (compIdx != COMP_Y && cu.chromaFormat != ChromaFormat::_444)
        {
          const __m128i mask = _mm_set_epi16(0, 1, 0, 1, 0, 1, 0, 1);
          __m128i       w0p0, w0p1;
          if (g_angle2mirror[angle] == 1)
          {
            w0p0 =
              _mm_lddqu_si128((__m128i *)(weight - (x << 1) - (8 - 1)));   // first sub-sample the required weights.
            w0p1                       = _mm_lddqu_si128((__m128i *)(weight - (x << 1) - 8 - (8 - 1)));
            const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0p0                       = _mm_shuffle_epi8(w0p0, shuffle_mask);
            w0p1                       = _mm_shuffle_epi8(w0p1, shuffle_mask);
          }
          else
          {
            w0p0 = _mm_lddqu_si128((__m128i *)(weight + (x << 1)));   // first sub-sample the required weights.
            w0p1 = _mm_lddqu_si128((__m128i *)(weight + (x << 1) + 8));
          }
          w0p0 = _mm_mullo_epi16(w0p0, mask);
          w0p1 = _mm_mullo_epi16(w0p1, mask);
          w0   = _mm_packs_epi32(w0p0, w0p1);
        }
        else
        {
          if (g_angle2mirror[angle] == 1)
          {
            w0                         = _mm_lddqu_si128((__m128i *)(weight - x - (8 - 1)));
            const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0                         = _mm_shuffle_epi8(w0, shuffle_mask);
          }
          else
          {
            w0 = _mm_lddqu_si128((__m128i *)(weight + x));
          }
        }
        __m128i w1 = _mm_sub_epi16(mmEight, w0);

        __m128i s0tmp = _mm_unpacklo_epi16(s0, s1);
        __m128i w0tmp = _mm_unpacklo_epi16(w0, w1);
        s0tmp         = _mm_add_epi32(_mm_madd_epi16(s0tmp, w0tmp), mmOffset);
        s0tmp         = _mm_sra_epi32(s0tmp, mmShift);

        s0 = _mm_unpackhi_epi16(s0, s1);
        w0 = _mm_unpackhi_epi16(w0, w1);
        s0 = _mm_add_epi32(_mm_madd_epi16(s0, w0), mmOffset);
        s0 = _mm_sra_epi32(s0, mmShift);

        s0 = _mm_packs_epi32(s0tmp, s0);
        s0 = _mm_min_epi16(mmMax, _mm_max_epi16(s0, mmMin));
        _mm_storeu_si128((__m128i *)(dst + x), s0);
      }
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
}

template<X86_VEXT vext, bool isLast> void simdFilter4x4_N6(const ClpRng &clpRng, Pel const *src, ptrdiff_t srcStride,
                                                           Pel *dst, ptrdiff_t dstStride, int width, int height,
                                                           TFilterCoeff const *coeffH, TFilterCoeff const *_coeffV)
{
  int row;

  src -= 2;
  src -= 2 * srcStride;

  _mm_prefetch((const char *)(src), _MM_HINT_T0);
  _mm_prefetch((const char *)(src + 1 * srcStride), _MM_HINT_T0);

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

  __m128i _dst0x;
  __m128i _dst1x;
  __m128i _dst2x;
  __m128i _dst3x;
  __m128i zerox = _mm_setzero_si128();

#if USE_AVX2
  if (vext >= AVX2)
  {
    __m256i off  = _mm256_set1_epi32(offset1st);
    __m256i zero = _mm256_setzero_si256();
    __m128i _src1x, _src2x, cVp1, cVp2;
    __m256i _dst0, _dst2, _src1, _src2, _src3;
    __m256i cV, cH;
    _src1x = _mm_loadu_si128((const __m128i *)(coeffH));
    _src1x = _mm_blend_epi16(_src1x, _mm_setzero_si128(), 0xc0);
    cH     = _mm256_set_m128i(_src1x, _src1x);
    cVp1   = _mm_setzero_si128();
    cVp2   = _mm_loadu_si128((const __m128i *)(_coeffV));
    cVp2   = _mm_shuffle_epi8(cVp2, _mm_setr_epi8(-1, -1, -1, -1, 10, 11, 8, 9, 6, 7, 4, 5, 2, 3, 0, 1));
    _dst0  = _mm256_set1_epi32(offset2nd);
    _dst2  = _mm256_set1_epi32(offset2nd);

    for (row = 1; row < 9; row += 2)
    {
      _mm_prefetch((const char *)(src + 2 * srcStride), _MM_HINT_T0);
      _mm_prefetch((const char *)(src + 3 * srcStride), _MM_HINT_T0);

      _src1x = _mm_alignr_epi8(cVp1, cVp2, 12);
      _src2x = _mm_alignr_epi8(cVp2, cVp1, 12);
      cVp1   = _src1x;
      cVp2   = _src2x;
      _src2x = _mm_bsrli_si128(_src1x, 2);
      _src2x = _mm_unpacklo_epi16(_src2x, _src1x);
      cV     = _mm256_set_m128i(_src2x, _src2x);

      // hor filter of row 0
      _src1x = _mm_loadu_si128((const __m128i *)&src[0]);
      _src2x = _mm_loadu_si128((const __m128i *)&src[1]);
      _src1  = _mm256_set_m128i(_src2x, _src1x);
      _src1  = _mm256_madd_epi16(_src1, cH);

      _src1x = _mm_loadu_si128((const __m128i *)&src[2]);
      _src2x = _mm_loadu_si128((const __m128i *)&src[3]);
      _src2  = _mm256_set_m128i(_src2x, _src1x);
      _src2  = _mm256_madd_epi16(_src2, cH);

      _src1 = _mm256_hadd_epi32(_src1, _src2);
      _src1 = _mm256_hadd_epi32(_src1, zero);
      _src1 = _mm256_add_epi32(_src1, off);
      _src3 = _mm256_srai_epi32(_src1, shift1st);

      src += srcStride;

      // hor filter of row 1
      _src1x = _mm_loadu_si128((const __m128i *)&src[0]);
      _src2x = _mm_loadu_si128((const __m128i *)&src[1]);
      _src1  = _mm256_set_m128i(_src2x, _src1x);
      _src1x = _mm_loadu_si128((const __m128i *)&src[2]);
      _src2x = _mm_loadu_si128((const __m128i *)&src[3]);
      _src2  = _mm256_set_m128i(_src2x, _src1x);

      _src1 = _mm256_madd_epi16(_src1, cH);
      _src2 = _mm256_madd_epi16(_src2, cH);
      _src1 = _mm256_hadd_epi32(_src1, _src2);
      _src1 = _mm256_hadd_epi32(_src1, zero);
      _src1 = _mm256_add_epi32(_src1, off);
      _src1 = _mm256_srai_epi32(_src1, shift1st);

      src += srcStride;

      // vertical filter
      _src2 = _mm256_unpacklo_epi16(_src3, _src1);
      _src1 = _mm256_shuffle_epi32(_src2, (0 << 0) + (0 << 2) + (0 << 4) + (0 << 6));
      _src3 = _mm256_shuffle_epi32(_src2, (2 << 0) + (2 << 2) + (2 << 4) + (2 << 6));

      _src1 = _mm256_madd_epi16(_src1, cV);
      _src3 = _mm256_madd_epi16(_src3, cV);

      _dst0 = _mm256_add_epi32(_src1, _dst0);
      _dst2 = _mm256_add_epi32(_src3, _dst2);
    }

    // process last row (9)
    _src1x = _mm_alignr_epi8(cVp1, cVp2, 14);
    _src1x = _mm_cvtepu16_epi32(_src1x);
    cV     = _mm256_set_m128i(_src1x, _src1x);

    _src1x = _mm_loadu_si128((const __m128i *)&src[0]);
    _src2x = _mm_loadu_si128((const __m128i *)&src[1]);
    _src1  = _mm256_set_m128i(_src2x, _src1x);
    _src1x = _mm_loadu_si128((const __m128i *)&src[2]);
    _src2x = _mm_loadu_si128((const __m128i *)&src[3]);
    _src2  = _mm256_set_m128i(_src2x, _src1x);
    _src1  = _mm256_madd_epi16(_src1, cH);
    _src2  = _mm256_madd_epi16(_src2, cH);
    _src1  = _mm256_hadd_epi32(_src1, _src2);
    _src1  = _mm256_hadd_epi32(_src1, zero);
    _src1  = _mm256_add_epi32(_src1, off);
    _src3  = _mm256_srai_epi32(_src1, shift1st);
    _src2  = _mm256_shuffle_epi32(_src3, (1 << 0) + (1 << 2) + (1 << 4) + (1 << 6));
    _src1  = _mm256_shuffle_epi32(_src3, (0 << 0) + (0 << 2) + (0 << 4) + (0 << 6));

    _src2 = _mm256_madd_epi16(_src2, cV);
    _src1 = _mm256_madd_epi16(_src1, cV);

    _dst0 = _mm256_add_epi32(_src1, _dst0);
    _dst2 = _mm256_add_epi32(_src2, _dst2);

    _dst0 = _mm256_srai_epi32(_dst0, shift2nd);
    _dst2 = _mm256_srai_epi32(_dst2, shift2nd);

    if (isLast)
    {
      __m256i vmin = _mm256_set1_epi32(clpRng.min);
      __m256i vmax = _mm256_set1_epi32(clpRng.max);

      _dst0 = _mm256_max_epi32(_mm256_min_epi32(_dst0, vmax), vmin);
      _dst2 = _mm256_max_epi32(_mm256_min_epi32(_dst2, vmax), vmin);
    }

    _dst0x = _mm256_castsi256_si128(_dst0);
    _dst1x = _mm256_extractf128_si256(_dst0, 1);
    _dst2x = _mm256_castsi256_si128(_dst2);
    _dst3x = _mm256_extractf128_si256(_dst2, 1);
  }
  else
#endif
  {
    ALIGN_DATA(64, TFilterCoeff coeffV[17]) = { 0,          0,          0,          _coeffV[5], _coeffV[4],
                                                _coeffV[3], _coeffV[2], _coeffV[1], _coeffV[0], 0,
                                                0,          0,          0,          0,          0 };

    __m128i cH  = _mm_loadu_si128((const __m128i *)(coeffH));
    cH          = _mm_blend_epi16(cH, _mm_setzero_si128(), 0xc0);
    __m128i off = _mm_set1_epi32(offset1st);
    __m128i _src1, _src2, _srcx, cV;

    _dst0x = _mm_set1_epi32(offset2nd);
    _dst1x = _mm_set1_epi32(offset2nd);
    _dst2x = _mm_set1_epi32(offset2nd);
    _dst3x = _mm_set1_epi32(offset2nd);

    for (row = 0; row < 9; row++)
    {
      _mm_prefetch((const char *)(src + 2 * srcStride), _MM_HINT_T0);
      _mm_prefetch((const char *)(src + 3 * srcStride), _MM_HINT_T0);

      cV = _mm_loadl_epi64((const __m128i *)(coeffV + 9 - row - 1));
      cV = _mm_cvtepu16_epi32(cV);

      _src1 = _mm_loadu_si128((const __m128i *)&src[0]);
      _src1 = _mm_madd_epi16(_src1, cH);

      _src2 = _mm_loadu_si128((const __m128i *)&src[1]);
      _src2 = _mm_madd_epi16(_src2, cH);
      _srcx = _mm_hadd_epi32(_src1, _src2);

      _src1 = _mm_loadu_si128((const __m128i *)&src[2]);
      _src1 = _mm_madd_epi16(_src1, cH);

      _src2 = _mm_loadu_si128((const __m128i *)&src[3]);
      _src2 = _mm_madd_epi16(_src2, cH);
      _src2 = _mm_hadd_epi32(_src1, _src2);

      _src2 = _mm_hadd_epi32(_srcx, _src2);
      _src2 = _mm_add_epi32(_src2, off);
      _src2 = _mm_srai_epi32(_src2, shift1st);

      _src1  = _mm_shuffle_epi32(_src2, 0);
      _src1  = _mm_madd_epi16(_src1, cV);
      _dst0x = _mm_add_epi32(_src1, _dst0x);

      _src1  = _mm_shuffle_epi32(_src2, 1 + 4 + 16 + 64);
      _src1  = _mm_madd_epi16(_src1, cV);
      _dst1x = _mm_add_epi32(_src1, _dst1x);

      _src1  = _mm_shuffle_epi32(_src2, 2 + 8 + 32 + 128);
      _src1  = _mm_madd_epi16(_src1, cV);
      _dst2x = _mm_add_epi32(_src1, _dst2x);

      _src1  = _mm_shuffle_epi32(_src2, 3 + 12 + 48 + 192);
      _src1  = _mm_madd_epi16(_src1, cV);
      _dst3x = _mm_add_epi32(_src1, _dst3x);

      src += srcStride;
    }

    _dst0x = _mm_srai_epi32(_dst0x, shift2nd);
    _dst1x = _mm_srai_epi32(_dst1x, shift2nd);
    _dst2x = _mm_srai_epi32(_dst2x, shift2nd);
    _dst3x = _mm_srai_epi32(_dst3x, shift2nd);

    if (isLast)
    {
      __m128i vmin = _mm_set1_epi32(clpRng.min);
      __m128i vmax = _mm_set1_epi32(clpRng.max);

      _dst0x = _mm_max_epi32(_mm_min_epi32(_dst0x, vmax), vmin);
      _dst1x = _mm_max_epi32(_mm_min_epi32(_dst1x, vmax), vmin);
      _dst2x = _mm_max_epi32(_mm_min_epi32(_dst2x, vmax), vmin);
      _dst3x = _mm_max_epi32(_mm_min_epi32(_dst3x, vmax), vmin);
    }
  }

  __m128i a01b01 = _mm_unpacklo_epi32(_dst0x, _dst1x);
  __m128i a23b23 = _mm_unpackhi_epi32(_dst0x, _dst1x);
  __m128i c01d01 = _mm_unpacklo_epi32(_dst2x, _dst3x);
  __m128i c23d23 = _mm_unpackhi_epi32(_dst2x, _dst3x);

  _dst0x = _mm_unpacklo_epi64(a01b01, c01d01);
  _dst1x = _mm_unpackhi_epi64(a01b01, c01d01);
  _dst2x = _mm_unpacklo_epi64(a23b23, c23d23);
  _dst3x = _mm_unpackhi_epi64(a23b23, c23d23);

  _dst0x = _mm_packs_epi32(_dst0x, zerox);
  _dst1x = _mm_packs_epi32(_dst1x, zerox);
  _dst2x = _mm_packs_epi32(_dst2x, zerox);
  _dst3x = _mm_packs_epi32(_dst3x, zerox);

  _mm_storel_epi64((__m128i *)(dst), _dst0x);
  _mm_storel_epi64((__m128i *)(dst + dstStride), _dst1x);
  _mm_storel_epi64((__m128i *)(dst + 2 * dstStride), _dst2x);
  _mm_storel_epi64((__m128i *)(dst + 3 * dstStride), _dst3x);
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template<X86_VEXT vext, bool isLast> void simdFilter4x4_N4(const ClpRng &clpRng, Pel const *src, ptrdiff_t srcStride,
                                                           Pel *dst, ptrdiff_t dstStride, int width, int height,
                                                           TFilterCoeff const *coeffH, TFilterCoeff const *_coeffV)
{
  int row;

  src -= 1;
  src -= srcStride;

  _mm_prefetch((const char *)(src), _MM_HINT_T0);
  _mm_prefetch((const char *)(src + srcStride), _MM_HINT_T0);

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

  __m128i _dst0x;
  __m128i _dst1x;
  __m128i _dst2x;
  __m128i _dst3x;
  __m128i zerox = _mm_setzero_si128();

#if USE_AVX2
  if (vext >= AVX2)
  {
    ALIGN_DATA(64, const TFilterCoeff coeffV[4]) = { _coeffV[3], _coeffV[2], _coeffV[1], _coeffV[0] };

    __m256i off  = _mm256_set1_epi32(offset1st);
    __m256i zero = _mm256_setzero_si256();
    __m128i _src1x, _src2x, cVp1, cVp2;
    __m256i _dst0, _dst2, _src1, _src2, _src3;
    __m256i cV, cH;
    cH   = _mm256_set1_epi64x(*((const long long int *)coeffH));
    cVp1 = _mm_setzero_si128();
    cVp2 = _mm_loadl_epi64((const __m128i *)coeffV);

    _dst0 = _mm256_set1_epi32(offset2nd);
    _dst2 = _mm256_set1_epi32(offset2nd);

    _src1x = _mm_alignr_epi8(cVp1, cVp2, 8);
    _src2x = _mm_alignr_epi8(cVp2, cVp1, 8);
    cVp1   = _src1x;
    cVp2   = _src2x;

    for (row = 0; row < 6; row += 2)
    {
      _mm_prefetch((const char *)(src + 2 * srcStride), _MM_HINT_T0);
      _mm_prefetch((const char *)(src + 3 * srcStride), _MM_HINT_T0);

      _src1x = _mm_alignr_epi8(cVp1, cVp2, 12);
      _src2x = _mm_alignr_epi8(cVp2, cVp1, 12);
      cVp1   = _src1x;
      cVp2   = _src2x;
      _src2x = _mm_bsrli_si128(_src1x, 2);
      _src2x = _mm_unpacklo_epi16(_src2x, _src1x);
      cV     = _mm256_set_m128i(_src2x, _src2x);

      // hor filter of row 0
      _src1x = _mm_loadl_epi64((const __m128i *)&src[0]);
      _src2x = _mm_loadl_epi64((const __m128i *)&src[1]);
      _src1  = _mm256_set_m128i(_src2x, _src1x);

      _src1x = _mm_loadl_epi64((const __m128i *)&src[2]);
      _src2x = _mm_loadl_epi64((const __m128i *)&src[3]);
      _src2  = _mm256_set_m128i(_src2x, _src1x);

      _src1 = _mm256_unpacklo_epi64(_src1, _src2);

      _src2 = _mm256_madd_epi16(_src1, cH);

      _src1 = _mm256_hadd_epi32(_src2, zero);
      _src1 = _mm256_add_epi32(_src1, off);
      _src3 = _mm256_srai_epi32(_src1, shift1st);

      src += srcStride;

      // hor filter of row 1
      _src1x = _mm_loadl_epi64((const __m128i *)&src[0]);
      _src2x = _mm_loadl_epi64((const __m128i *)&src[1]);
      _src1  = _mm256_set_m128i(_src2x, _src1x);

      _src1x = _mm_loadl_epi64((const __m128i *)&src[2]);
      _src2x = _mm_loadl_epi64((const __m128i *)&src[3]);
      _src2  = _mm256_set_m128i(_src2x, _src1x);

      _src1 = _mm256_unpacklo_epi64(_src1, _src2);

      _src2 = _mm256_madd_epi16(_src1, cH);

      _src1 = _mm256_hadd_epi32(_src2, zero);
      _src1 = _mm256_add_epi32(_src1, off);
      _src1 = _mm256_srai_epi32(_src1, shift1st);

      src += srcStride;

      // vertical filter
      _src2 = _mm256_unpacklo_epi16(_src3, _src1);
      _src1 = _mm256_shuffle_epi32(_src2, (0 << 0) + (0 << 2) + (0 << 4) + (0 << 6));
      _src3 = _mm256_shuffle_epi32(_src2, (2 << 0) + (2 << 2) + (2 << 4) + (2 << 6));

      _src1 = _mm256_madd_epi16(_src1, cV);
      _src3 = _mm256_madd_epi16(_src3, cV);

      _dst0 = _mm256_add_epi32(_src1, _dst0);
      _dst2 = _mm256_add_epi32(_src3, _dst2);
      // REF END
    }

    _src1x = _mm_alignr_epi8(cVp1, cVp2, 14);
    _src1x = _mm_cvtepu16_epi32(_src1x);
    cV     = _mm256_set_m128i(_src1x, _src1x);

    _src1x = _mm_loadl_epi64((const __m128i *)&src[0]);
    _src2x = _mm_loadl_epi64((const __m128i *)&src[1]);
    _src1  = _mm256_set_m128i(_src2x, _src1x);

    _src1x = _mm_loadl_epi64((const __m128i *)&src[2]);
    _src2x = _mm_loadl_epi64((const __m128i *)&src[3]);
    _src2  = _mm256_set_m128i(_src2x, _src1x);

    _src1 = _mm256_unpacklo_epi64(_src1, _src2);

    _src2 = _mm256_madd_epi16(_src1, cH);

    _src1 = _mm256_hadd_epi32(_src2, zero);
    _src1 = _mm256_add_epi32(_src1, off);
    _src1 = _mm256_srai_epi32(_src1, shift1st);

    _src2 = _mm256_unpacklo_epi16(_src1, zero);
    _src1 = _mm256_shuffle_epi32(_src2, (0 << 0) + (0 << 2) + (0 << 4) + (0 << 6));
    _src3 = _mm256_shuffle_epi32(_src2, (2 << 0) + (2 << 2) + (2 << 4) + (2 << 6));

    _src1 = _mm256_madd_epi16(_src1, cV);
    _src3 = _mm256_madd_epi16(_src3, cV);

    _dst0 = _mm256_add_epi32(_src1, _dst0);
    _dst2 = _mm256_add_epi32(_src3, _dst2);

    _dst0 = _mm256_srai_epi32(_dst0, shift2nd);
    _dst2 = _mm256_srai_epi32(_dst2, shift2nd);

    if (isLast)
    {
      __m256i vmin = _mm256_set1_epi32(clpRng.min);
      __m256i vmax = _mm256_set1_epi32(clpRng.max);

      _dst0 = _mm256_max_epi32(_mm256_min_epi32(_dst0, vmax), vmin);
      _dst2 = _mm256_max_epi32(_mm256_min_epi32(_dst2, vmax), vmin);
    }

    _dst0x = _mm256_castsi256_si128(_dst0);
    _dst1x = _mm256_extractf128_si256(_dst0, 1);
    _dst2x = _mm256_castsi256_si128(_dst2);
    _dst3x = _mm256_extractf128_si256(_dst2, 1);
  }
  else
#endif
  {
    ALIGN_DATA(64,
               const TFilterCoeff coeffV[10]) = { 0, 0, 0, _coeffV[3], _coeffV[2], _coeffV[1], _coeffV[0], 0, 0, 0 };

    __m128i cH  = _mm_loadl_epi64((const __m128i *)coeffH);
    cH          = _mm_unpacklo_epi64(cH, cH);
    __m128i off = _mm_set1_epi32(offset1st);
    __m128i _src1, _src2, _srcx, cV;

    _dst0x = _mm_set1_epi32(offset2nd);
    _dst1x = _mm_set1_epi32(offset2nd);
    _dst2x = _mm_set1_epi32(offset2nd);
    _dst3x = _mm_set1_epi32(offset2nd);

    for (row = 0; row < 7; row++)
    {
      _mm_prefetch((const char *)(src + 2 * srcStride), _MM_HINT_T0);
      _mm_prefetch((const char *)(src + 3 * srcStride), _MM_HINT_T0);

      cV = _mm_loadl_epi64((const __m128i *)(coeffV + 7 - row - 1));
      cV = _mm_cvtepu16_epi32(cV);

      _src1 = _mm_loadl_epi64((const __m128i *)&src[0]);
      _src2 = _mm_loadl_epi64((const __m128i *)&src[1]);
      _src1 = _mm_unpacklo_epi64(_src1, _src2);

      _srcx = _mm_madd_epi16(_src1, cH);

      _src1 = _mm_loadl_epi64((const __m128i *)&src[2]);
      _src2 = _mm_loadl_epi64((const __m128i *)&src[3]);
      _src1 = _mm_unpacklo_epi64(_src1, _src2);

      _src2 = _mm_madd_epi16(_src1, cH);

      _src2 = _mm_hadd_epi32(_srcx, _src2);
      _src2 = _mm_add_epi32(_src2, off);
      _src2 = _mm_srai_epi32(_src2, shift1st);

      _src1  = _mm_shuffle_epi32(_src2, 0);
      _src1  = _mm_madd_epi16(_src1, cV);
      _dst0x = _mm_add_epi32(_src1, _dst0x);

      _src1  = _mm_shuffle_epi32(_src2, 1 + 4 + 16 + 64);
      _src1  = _mm_madd_epi16(_src1, cV);
      _dst1x = _mm_add_epi32(_src1, _dst1x);

      _src1  = _mm_shuffle_epi32(_src2, 2 + 8 + 32 + 128);
      _src1  = _mm_madd_epi16(_src1, cV);
      _dst2x = _mm_add_epi32(_src1, _dst2x);

      _src1  = _mm_shuffle_epi32(_src2, 3 + 12 + 48 + 192);
      _src1  = _mm_madd_epi16(_src1, cV);
      _dst3x = _mm_add_epi32(_src1, _dst3x);

      src += srcStride;
    }

    _dst0x = _mm_srai_epi32(_dst0x, shift2nd);
    _dst1x = _mm_srai_epi32(_dst1x, shift2nd);
    _dst2x = _mm_srai_epi32(_dst2x, shift2nd);
    _dst3x = _mm_srai_epi32(_dst3x, shift2nd);

    if (isLast)
    {
      __m128i vmin = _mm_set1_epi32(clpRng.min);
      __m128i vmax = _mm_set1_epi32(clpRng.max);

      _dst0x = _mm_max_epi32(_mm_min_epi32(_dst0x, vmax), vmin);
      _dst1x = _mm_max_epi32(_mm_min_epi32(_dst1x, vmax), vmin);
      _dst2x = _mm_max_epi32(_mm_min_epi32(_dst2x, vmax), vmin);
      _dst3x = _mm_max_epi32(_mm_min_epi32(_dst3x, vmax), vmin);
    }
  }

  __m128i a01b01 = _mm_unpacklo_epi32(_dst0x, _dst1x);
  __m128i a23b23 = _mm_unpackhi_epi32(_dst0x, _dst1x);
  __m128i c01d01 = _mm_unpacklo_epi32(_dst2x, _dst3x);
  __m128i c23d23 = _mm_unpackhi_epi32(_dst2x, _dst3x);

  _dst0x = _mm_unpacklo_epi64(a01b01, c01d01);
  _dst1x = _mm_unpackhi_epi64(a01b01, c01d01);
  _dst2x = _mm_unpacklo_epi64(a23b23, c23d23);
  _dst3x = _mm_unpackhi_epi64(a23b23, c23d23);

  _dst0x = _mm_packs_epi32(_dst0x, zerox);
  _dst1x = _mm_packs_epi32(_dst1x, zerox);
  _dst2x = _mm_packs_epi32(_dst2x, zerox);
  _dst3x = _mm_packs_epi32(_dst3x, zerox);

  Pel *realDstLines[4] = { dst, dst + dstStride, dst + 2 * dstStride, dst + 3 * dstStride };

  _mm_storel_epi64((__m128i *)realDstLines[0], _dst0x);
  _mm_storel_epi64((__m128i *)realDstLines[1], _dst1x);
  _mm_storel_epi64((__m128i *)realDstLines[2], _dst2x);
  _mm_storel_epi64((__m128i *)realDstLines[3], _dst3x);
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template<X86_VEXT vext> void xWeightedGeoBlk_SSE(const CodingUnit &cu, const uint32_t width, const uint32_t height,
                                                 const CompID compIdx, const uint8_t splitDir, const uint8_t bldIdx,
                                                 PelUnitBuf &predDst, PelUnitBuf &predSrc0, PelUnitBuf &predSrc1)
{
  Pel      *dst        = predDst.get(compIdx).buf;
  Pel      *src0       = predSrc0.get(compIdx).buf;
  Pel      *src1       = predSrc1.get(compIdx).buf;
  ptrdiff_t strideDst  = predDst.get(compIdx).stride;
  ptrdiff_t strideSrc0 = predSrc0.get(compIdx).stride;
  ptrdiff_t strideSrc1 = predSrc1.get(compIdx).stride;

  const int8_t  log2WeightBase = 5;
  const ClpRng  clpRng         = cu.slice->m_clpRngs.comp[compIdx];
  const int32_t shiftWeighted  = IF_INTERNAL_FRAC_BITS(clpRng.bd) + log2WeightBase;
  const int32_t offsetWeighted = (1 << (shiftWeighted - 1)) + (IF_INTERNAL_OFFS << log2WeightBase);

  int16_t wIdx = floorLog2(cu.lwidth()) - GEO_MIN_CU_LOG2;
  int16_t hIdx = floorLog2(cu.lheight()) - GEO_MIN_CU_LOG2;

  const int angle = g_geoParams[splitDir].angleIdx;

  int16_t  stepY  = 0;
  int16_t *weight = nullptr;
  if (g_angle2mirror[angle] == 2)
  {
    stepY = -GEO_WEIGHT_MASK_SIZE;
    weight =
      &g_globalGeoWeights[bldIdx][g_angle2mask[angle]]
                         [(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE +
                          g_weightOffset[splitDir][hIdx][wIdx][0]];
  }
  else if (g_angle2mirror[angle] == 1)
  {
    stepY  = GEO_WEIGHT_MASK_SIZE;
    weight = &g_globalGeoWeights[bldIdx][g_angle2mask[angle]]
                                [g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE +
                                 (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
  }
  else
  {
    stepY = GEO_WEIGHT_MASK_SIZE;
    weight =
      &g_globalGeoWeights[bldIdx][g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE +
                                                       g_weightOffset[splitDir][hIdx][wIdx][0]];
  }
  const __m128i mmEight  = _mm_set1_epi16(32);
  const __m128i mmOffset = _mm_set1_epi32(offsetWeighted);
  const __m128i mmShift  = _mm_cvtsi32_si128(shiftWeighted);
  const __m128i mmMin    = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax    = _mm_set1_epi16(clpRng.max);

  if (compIdx != COMP_Y && cu.chromaFormat == ChromaFormat::_420)
  {
    stepY *= 2;
  }
  if (width == 4)
  {
    // it will occur to chroma only
    for (int y = 0; y < height; y++)
    {
      __m128i s0 = _mm_loadl_epi64((__m128i *)(src0));
      __m128i s1 = _mm_loadl_epi64((__m128i *)(src1));
      __m128i w0;
      if (g_angle2mirror[angle] == 1)
      {
        w0                         = _mm_loadu_si128((__m128i *)(weight - (8 - 1)));
        const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
        w0                         = _mm_shuffle_epi8(w0, shuffle_mask);
      }
      else
      {
        w0 = _mm_loadu_si128((__m128i *)(weight));
      }
      w0         = _mm_shuffle_epi8(w0, _mm_setr_epi8(0, 1, 4, 5, 8, 9, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0));
      __m128i w1 = _mm_sub_epi16(mmEight, w0);
      s0         = _mm_unpacklo_epi16(s0, s1);
      w0         = _mm_unpacklo_epi16(w0, w1);
      s0         = _mm_add_epi32(_mm_madd_epi16(s0, w0), mmOffset);
      s0         = _mm_sra_epi32(s0, mmShift);
      s0         = _mm_packs_epi32(s0, s0);
      s0         = _mm_min_epi16(mmMax, _mm_max_epi16(s0, mmMin));
      _mm_storel_epi64((__m128i *)(dst), s0);
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
#if USE_AVX2
  else if (width >= 16)
  {
    const __m256i mmEightAVX2  = _mm256_set1_epi16(32);
    const __m256i mmOffsetAVX2 = _mm256_set1_epi32(offsetWeighted);
    const __m256i mmMinAVX2    = _mm256_set1_epi16(clpRng.min);
    const __m256i mmMaxAVX2    = _mm256_set1_epi16(clpRng.max);
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 16)
      {
        __m256i s0 = _mm256_lddqu_si256((__m256i *)(src0 + x));   // why not aligned with 128/256 bit boundaries
        __m256i s1 = _mm256_lddqu_si256((__m256i *)(src1 + x));

        __m256i w0;
        if (compIdx != COMP_Y && cu.chromaFormat != ChromaFormat::_444)
        {
          const __m256i mask = _mm256_set_epi16(0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1);
          __m256i       w0p0, w0p1;
          if (g_angle2mirror[angle] == 1)
          {
            w0p0 =
              _mm256_lddqu_si256((__m256i *)(weight - (x << 1) - (16 - 1)));   // first sub-sample the required weights.
            w0p1                       = _mm256_lddqu_si256((__m256i *)(weight - (x << 1) - 16 - (16 - 1)));
            const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3,
                                                         2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0p0                       = _mm256_shuffle_epi8(w0p0, shuffle_mask);
            w0p0                       = _mm256_permute4x64_epi64(w0p0, _MM_SHUFFLE(1, 0, 3, 2));
            w0p1                       = _mm256_shuffle_epi8(w0p1, shuffle_mask);
            w0p1                       = _mm256_permute4x64_epi64(w0p1, _MM_SHUFFLE(1, 0, 3, 2));
          }
          else
          {
            w0p0 = _mm256_lddqu_si256((__m256i *)(weight + (x << 1)));   // first sub-sample the required weights.
            w0p1 = _mm256_lddqu_si256((__m256i *)(weight + (x << 1) + 16));
          }
          w0p0 = _mm256_mullo_epi16(w0p0, mask);
          w0p1 = _mm256_mullo_epi16(w0p1, mask);
          w0   = _mm256_packs_epi16(w0p0, w0p1);
          w0   = _mm256_permute4x64_epi64(w0, _MM_SHUFFLE(3, 1, 2, 0));
        }
        else
        {
          if (g_angle2mirror[angle] == 1)
          {
            w0                         = _mm256_lddqu_si256((__m256i *)(weight - x - (16 - 1)));
            const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3,
                                                         2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0                         = _mm256_shuffle_epi8(w0, shuffle_mask);
            w0                         = _mm256_permute4x64_epi64(w0, _MM_SHUFFLE(1, 0, 3, 2));
          }
          else
          {
            w0 = _mm256_lddqu_si256((__m256i *)(weight + x));
          }
        }
        __m256i w1 = _mm256_sub_epi16(mmEightAVX2, w0);

        __m256i s0tmp = _mm256_unpacklo_epi16(s0, s1);
        __m256i w0tmp = _mm256_unpacklo_epi16(w0, w1);
        s0tmp         = _mm256_add_epi32(_mm256_madd_epi16(s0tmp, w0tmp), mmOffsetAVX2);
        s0tmp         = _mm256_sra_epi32(s0tmp, mmShift);

        s0 = _mm256_unpackhi_epi16(s0, s1);
        w0 = _mm256_unpackhi_epi16(w0, w1);
        s0 = _mm256_add_epi32(_mm256_madd_epi16(s0, w0), mmOffsetAVX2);
        s0 = _mm256_sra_epi32(s0, mmShift);

        s0 = _mm256_packs_epi32(s0tmp, s0);
        s0 = _mm256_min_epi16(mmMaxAVX2, _mm256_max_epi16(s0, mmMinAVX2));
        _mm256_storeu_si256((__m256i *)(dst + x), s0);
      }
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
#endif
  else
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 8)
      {
        __m128i s0 = _mm_lddqu_si128((__m128i *)(src0 + x));
        __m128i s1 = _mm_lddqu_si128((__m128i *)(src1 + x));
        __m128i w0;
        if (compIdx != COMP_Y && cu.chromaFormat != ChromaFormat::_444)
        {
          const __m128i mask = _mm_set_epi16(0, 1, 0, 1, 0, 1, 0, 1);
          __m128i       w0p0, w0p1;
          if (g_angle2mirror[angle] == 1)
          {
            w0p0 =
              _mm_lddqu_si128((__m128i *)(weight - (x << 1) - (8 - 1)));   // first sub-sample the required weights.
            w0p1                       = _mm_lddqu_si128((__m128i *)(weight - (x << 1) - 8 - (8 - 1)));
            const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0p0                       = _mm_shuffle_epi8(w0p0, shuffle_mask);
            w0p1                       = _mm_shuffle_epi8(w0p1, shuffle_mask);
          }
          else
          {
            w0p0 = _mm_lddqu_si128((__m128i *)(weight + (x << 1)));   // first sub-sample the required weights.
            w0p1 = _mm_lddqu_si128((__m128i *)(weight + (x << 1) + 8));
          }
          w0p0 = _mm_mullo_epi16(w0p0, mask);
          w0p1 = _mm_mullo_epi16(w0p1, mask);
          w0   = _mm_packs_epi32(w0p0, w0p1);
        }
        else
        {
          if (g_angle2mirror[angle] == 1)
          {
            w0                         = _mm_lddqu_si128((__m128i *)(weight - x - (8 - 1)));
            const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0                         = _mm_shuffle_epi8(w0, shuffle_mask);
          }
          else
          {
            w0 = _mm_lddqu_si128((__m128i *)(weight + x));
          }
        }
        __m128i w1 = _mm_sub_epi16(mmEight, w0);

        __m128i s0tmp = _mm_unpacklo_epi16(s0, s1);
        __m128i w0tmp = _mm_unpacklo_epi16(w0, w1);
        s0tmp         = _mm_add_epi32(_mm_madd_epi16(s0tmp, w0tmp), mmOffset);
        s0tmp         = _mm_sra_epi32(s0tmp, mmShift);

        s0 = _mm_unpackhi_epi16(s0, s1);
        w0 = _mm_unpackhi_epi16(w0, w1);
        s0 = _mm_add_epi32(_mm_madd_epi16(s0, w0), mmOffset);
        s0 = _mm_sra_epi32(s0, mmShift);

        s0 = _mm_packs_epi32(s0tmp, s0);
        s0 = _mm_min_epi16(mmMax, _mm_max_epi16(s0, mmMin));
        _mm_storeu_si128((__m128i *)(dst + x), s0);
      }
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
}

template<X86_VEXT vext> void xWeightedGeoBlkRounded_SSE(const CodingUnit &pu, const uint32_t width,
                                                        const uint32_t height, const CompID compIdx,
                                                        const uint8_t splitDir, const uint8_t bldIdx,
                                                        PelUnitBuf &predDst, PelUnitBuf &predSrc0, PelUnitBuf &predSrc1)
{
  Pel      *dst        = predDst.get(compIdx).buf;
  Pel      *src0       = predSrc0.get(compIdx).buf;
  Pel      *src1       = predSrc1.get(compIdx).buf;
  ptrdiff_t strideDst  = predDst.get(compIdx).stride;
  ptrdiff_t strideSrc0 = predSrc0.get(compIdx).stride;
  ptrdiff_t strideSrc1 = predSrc1.get(compIdx).stride;

  int16_t  wIdx   = floorLog2(pu.lwidth()) - GEO_MIN_CU_LOG2;
  int16_t  hIdx   = floorLog2(pu.lheight()) - GEO_MIN_CU_LOG2;
  int16_t  angle  = g_geoParams[splitDir].angleIdx;
  int16_t  stepY  = 0;
  int16_t *weight = nullptr;

  if (g_angle2mirror[angle] == 2)
  {
    stepY = -GEO_WEIGHT_MASK_SIZE;
    weight =
      &g_globalGeoWeights[bldIdx][g_angle2mask[angle]]
                         [(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE +
                          g_weightOffset[splitDir][hIdx][wIdx][0]];
  }
  else if (g_angle2mirror[angle] == 1)
  {
    stepY  = GEO_WEIGHT_MASK_SIZE;
    weight = &g_globalGeoWeights[bldIdx][g_angle2mask[angle]]
                                [g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE +
                                 (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
  }
  else
  {
    stepY = GEO_WEIGHT_MASK_SIZE;
    weight =
      &g_globalGeoWeights[bldIdx][g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE +
                                                       g_weightOffset[splitDir][hIdx][wIdx][0]];
  }

  const __m128i mmEight  = _mm_set1_epi16(32);
  const __m128i mmOffset = _mm_set1_epi32(16);
  const __m128i mmShift  = _mm_cvtsi32_si128(5);

  if (compIdx != COMP_Y && pu.chromaFormat == ChromaFormat::_420)
  {
    stepY <<= 1;
  }
  if (width == 4)
  {
    // it will occur to chroma only
    for (int y = 0; y < height; y++)
    {
      __m128i s0 = _mm_loadl_epi64((__m128i *)(src0));
      __m128i s1 = _mm_loadl_epi64((__m128i *)(src1));
      __m128i w0;
      if (g_angle2mirror[angle] == 1)
      {
        w0                         = _mm_loadu_si128((__m128i *)(weight - (8 - 1)));
        const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
        w0                         = _mm_shuffle_epi8(w0, shuffle_mask);
      }
      else
      {
        w0 = _mm_loadu_si128((__m128i *)(weight));
      }
      w0         = _mm_shuffle_epi8(w0, _mm_setr_epi8(0, 1, 4, 5, 8, 9, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0));
      __m128i w1 = _mm_sub_epi16(mmEight, w0);
      s0         = _mm_unpacklo_epi16(s0, s1);
      w0         = _mm_unpacklo_epi16(w0, w1);
      s0         = _mm_add_epi32(_mm_madd_epi16(s0, w0), mmOffset);
      s0         = _mm_sra_epi32(s0, mmShift);
      s0         = _mm_packs_epi32(s0, s0);
      _mm_storel_epi64((__m128i *)(dst), s0);
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
#if USE_AVX2
  else if (width >= 16)
  {
    const __m256i mmEightAVX2  = _mm256_set1_epi16(32);
    const __m256i mmOffsetAVX2 = _mm256_set1_epi32(16);
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 16)
      {
        __m256i s0 = _mm256_lddqu_si256((__m256i *)(src0 + x));   // why not aligned with 128/256 bit boundaries
        __m256i s1 = _mm256_lddqu_si256((__m256i *)(src1 + x));

        __m256i w0;
        if (compIdx != COMP_Y && pu.chromaFormat != ChromaFormat::_444)
        {
          const __m256i mask = _mm256_set_epi16(0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1);
          __m256i       w0p0, w0p1;
          if (g_angle2mirror[angle] == 1)
          {
            w0p0 =
              _mm256_lddqu_si256((__m256i *)(weight - (x << 1) - (16 - 1)));   // first sub-sample the required weights.
            w0p1                       = _mm256_lddqu_si256((__m256i *)(weight - (x << 1) - 16 - (16 - 1)));
            const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3,
                                                         2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0p0                       = _mm256_shuffle_epi8(w0p0, shuffle_mask);
            w0p0                       = _mm256_permute4x64_epi64(w0p0, _MM_SHUFFLE(1, 0, 3, 2));
            w0p1                       = _mm256_shuffle_epi8(w0p1, shuffle_mask);
            w0p1                       = _mm256_permute4x64_epi64(w0p1, _MM_SHUFFLE(1, 0, 3, 2));
          }
          else
          {
            w0p0 = _mm256_lddqu_si256((__m256i *)(weight + (x << 1)));   // first sub-sample the required weights.
            w0p1 = _mm256_lddqu_si256((__m256i *)(weight + (x << 1) + 16));
          }
          w0p0 = _mm256_mullo_epi16(w0p0, mask);
          w0p1 = _mm256_mullo_epi16(w0p1, mask);
          w0   = _mm256_packs_epi16(w0p0, w0p1);
          w0   = _mm256_permute4x64_epi64(w0, _MM_SHUFFLE(3, 1, 2, 0));
        }
        else
        {
          if (g_angle2mirror[angle] == 1)
          {
            w0                         = _mm256_lddqu_si256((__m256i *)(weight - x - (16 - 1)));
            const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3,
                                                         2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0                         = _mm256_shuffle_epi8(w0, shuffle_mask);
            w0                         = _mm256_permute4x64_epi64(w0, _MM_SHUFFLE(1, 0, 3, 2));
          }
          else
          {
            w0 = _mm256_lddqu_si256((__m256i *)(weight + x));
          }
        }
        __m256i w1 = _mm256_sub_epi16(mmEightAVX2, w0);

        __m256i s0tmp = _mm256_unpacklo_epi16(s0, s1);
        __m256i w0tmp = _mm256_unpacklo_epi16(w0, w1);
        s0tmp         = _mm256_add_epi32(_mm256_madd_epi16(s0tmp, w0tmp), mmOffsetAVX2);
        s0tmp         = _mm256_sra_epi32(s0tmp, mmShift);

        s0 = _mm256_unpackhi_epi16(s0, s1);
        w0 = _mm256_unpackhi_epi16(w0, w1);
        s0 = _mm256_add_epi32(_mm256_madd_epi16(s0, w0), mmOffsetAVX2);
        s0 = _mm256_sra_epi32(s0, mmShift);

        s0 = _mm256_packs_epi32(s0tmp, s0);
        _mm256_storeu_si256((__m256i *)(dst + x), s0);
      }
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
#endif
  else
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 8)
      {
        __m128i s0 = _mm_lddqu_si128((__m128i *)(src0 + x));
        __m128i s1 = _mm_lddqu_si128((__m128i *)(src1 + x));
        __m128i w0;
        if (compIdx != COMP_Y && pu.chromaFormat != ChromaFormat::_444)
        {
          const __m128i mask = _mm_set_epi16(0, 1, 0, 1, 0, 1, 0, 1);
          __m128i       w0p0, w0p1;
          if (g_angle2mirror[angle] == 1)
          {
            w0p0 =
              _mm_lddqu_si128((__m128i *)(weight - (x << 1) - (8 - 1)));   // first sub-sample the required weights.
            w0p1                       = _mm_lddqu_si128((__m128i *)(weight - (x << 1) - 8 - (8 - 1)));
            const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0p0                       = _mm_shuffle_epi8(w0p0, shuffle_mask);
            w0p1                       = _mm_shuffle_epi8(w0p1, shuffle_mask);
          }
          else
          {
            w0p0 = _mm_lddqu_si128((__m128i *)(weight + (x << 1)));   // first sub-sample the required weights.
            w0p1 = _mm_lddqu_si128((__m128i *)(weight + (x << 1) + 8));
          }
          w0p0 = _mm_mullo_epi16(w0p0, mask);
          w0p1 = _mm_mullo_epi16(w0p1, mask);
          w0   = _mm_packs_epi32(w0p0, w0p1);
        }
        else
        {
          if (g_angle2mirror[angle] == 1)
          {
            w0                         = _mm_lddqu_si128((__m128i *)(weight - x - (8 - 1)));
            const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0                         = _mm_shuffle_epi8(w0, shuffle_mask);
          }
          else
          {
            w0 = _mm_lddqu_si128((__m128i *)(weight + x));
          }
        }
        __m128i w1 = _mm_sub_epi16(mmEight, w0);

        __m128i s0tmp = _mm_unpacklo_epi16(s0, s1);
        __m128i w0tmp = _mm_unpacklo_epi16(w0, w1);
        s0tmp         = _mm_add_epi32(_mm_madd_epi16(s0tmp, w0tmp), mmOffset);
        s0tmp         = _mm_sra_epi32(s0tmp, mmShift);

        s0 = _mm_unpackhi_epi16(s0, s1);
        w0 = _mm_unpackhi_epi16(w0, w1);
        s0 = _mm_add_epi32(_mm_madd_epi16(s0, w0), mmOffset);
        s0 = _mm_sra_epi32(s0, mmShift);

        s0 = _mm_packs_epi32(s0tmp, s0);
        _mm_storeu_si128((__m128i *)(dst + x), s0);
      }
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
}

template<X86_VEXT vext> void InterpolationFilter::_initInterpolationFilterX86()
{
#if IF_12TAP_SIMD
  m_filterHor[_12_TAPS][0][0] = simdFilter<vext, 12, false, false, false, false>;
  m_filterHor[_12_TAPS][0][1] = simdFilter<vext, 12, false, false, true, false>;
  m_filterHor[_12_TAPS][1][0] = simdFilter<vext, 12, false, true, false, false>;
  m_filterHor[_12_TAPS][1][1] = simdFilter<vext, 12, false, true, true, false>;
#endif
  m_filterHor[_8_TAPS][0][0] = simdFilter<vext, 8, false, false, false, false>;
  m_filterHor[_8_TAPS][0][1] = simdFilter<vext, 8, false, false, true, false>;
  m_filterHor[_8_TAPS][1][0] = simdFilter<vext, 8, false, true, false, false>;
  m_filterHor[_8_TAPS][1][1] = simdFilter<vext, 8, false, true, true, false>;

  m_filterHor[_4_TAPS][0][0] = simdFilter<vext, 4, false, false, false, false>;
  m_filterHor[_4_TAPS][0][1] = simdFilter<vext, 4, false, false, true, false>;
  m_filterHor[_4_TAPS][1][0] = simdFilter<vext, 4, false, true, false, false>;
  m_filterHor[_4_TAPS][1][1] = simdFilter<vext, 4, false, true, true, false>;

  m_filterHor[_2_TAPS_DMVR][0][0] = simdFilter<vext, 2, false, false, false, true>;
  m_filterHor[_2_TAPS_DMVR][0][1] = simdFilter<vext, 2, false, false, true, true>;
  m_filterHor[_2_TAPS_DMVR][1][0] = simdFilter<vext, 2, false, true, false, true>;
  m_filterHor[_2_TAPS_DMVR][1][1] = simdFilter<vext, 2, false, true, true, true>;

  m_filterHor[_6_TAPS][0][0] = simdFilter<vext, 6, false, false, false, false>;
  m_filterHor[_6_TAPS][0][1] = simdFilter<vext, 6, false, false, true, false>;
  m_filterHor[_6_TAPS][1][0] = simdFilter<vext, 6, false, true, false, false>;
  m_filterHor[_6_TAPS][1][1] = simdFilter<vext, 6, false, true, true, false>;

  m_filterHor[_2_TAPS][0][0] = simdFilter<vext, 2, false, false, false, false>;
  m_filterHor[_2_TAPS][0][1] = simdFilter<vext, 2, false, false, true, false>;
  m_filterHor[_2_TAPS][1][0] = simdFilter<vext, 2, false, true, false, false>;
  m_filterHor[_2_TAPS][1][1] = simdFilter<vext, 2, false, true, true, false>;

#if IF_12TAP_SIMD
  m_filterVer[_12_TAPS][0][0] = simdFilter<vext, 12, true, false, false, false>;
  m_filterVer[_12_TAPS][0][1] = simdFilter<vext, 12, true, false, true, false>;
  m_filterVer[_12_TAPS][1][0] = simdFilter<vext, 12, true, true, false, false>;
  m_filterVer[_12_TAPS][1][1] = simdFilter<vext, 12, true, true, true, false>;
#endif

  m_filterVer[_8_TAPS][0][0] = simdFilter<vext, 8, true, false, false, false>;
  m_filterVer[_8_TAPS][0][1] = simdFilter<vext, 8, true, false, true, false>;
  m_filterVer[_8_TAPS][1][0] = simdFilter<vext, 8, true, true, false, false>;
  m_filterVer[_8_TAPS][1][1] = simdFilter<vext, 8, true, true, true, false>;

  m_filterVer[_4_TAPS][0][0] = simdFilter<vext, 4, true, false, false, false>;
  m_filterVer[_4_TAPS][0][1] = simdFilter<vext, 4, true, false, true, false>;
  m_filterVer[_4_TAPS][1][0] = simdFilter<vext, 4, true, true, false, false>;
  m_filterVer[_4_TAPS][1][1] = simdFilter<vext, 4, true, true, true, false>;

  m_filterVer[_2_TAPS_DMVR][0][0] = simdFilter<vext, 2, true, false, false, true>;
  m_filterVer[_2_TAPS_DMVR][0][1] = simdFilter<vext, 2, true, false, true, true>;
  m_filterVer[_2_TAPS_DMVR][1][0] = simdFilter<vext, 2, true, true, false, true>;
  m_filterVer[_2_TAPS_DMVR][1][1] = simdFilter<vext, 2, true, true, true, true>;

  m_filterVer[_6_TAPS][0][0] = simdFilter<vext, 6, true, false, false, false>;
  m_filterVer[_6_TAPS][0][1] = simdFilter<vext, 6, true, false, true, false>;
  m_filterVer[_6_TAPS][1][0] = simdFilter<vext, 6, true, true, false, false>;
  m_filterVer[_6_TAPS][1][1] = simdFilter<vext, 6, true, true, true, false>;

  m_filterVer[_2_TAPS][0][0] = simdFilter<vext, 2, true, false, false, false>;
  m_filterVer[_2_TAPS][0][1] = simdFilter<vext, 2, true, false, true, false>;
  m_filterVer[_2_TAPS][1][0] = simdFilter<vext, 2, true, true, false, false>;
  m_filterVer[_2_TAPS][1][1] = simdFilter<vext, 2, true, true, true, false>;

#if IF_12TAP && SIMD_4x4_12
  m_filter4x4[0][0] = simdInterpolate4x4_12tap<false>;
  m_filter4x4[0][1] = simdInterpolate4x4_12tap<true>;
#else
  m_filter4x4[0][0] = simdFilter4x4_N6<vext, false>;
  m_filter4x4[0][1] = simdFilter4x4_N6<vext, true>;
#endif
#if JVET_Z0117_CHROMA_IF
  m_filter4x4[1][0] = simdFilter4x4_N6<vext, false>;
  m_filter4x4[1][1] = simdFilter4x4_N6<vext, true>;
#else
  m_filter4x4[1][0] = simdFilter4x4_N4<vext, false>;
  m_filter4x4[1][1] = simdFilter4x4_N4<vext, true>;
#endif

  m_filterCopy[0][0] = simdFilterCopy<vext, false, false>;
  m_filterCopy[0][1] = simdFilterCopy<vext, false, true>;
  m_filterCopy[1][0] = simdFilterCopy<vext, true, false>;
  m_filterCopy[1][1] = simdFilterCopy<vext, true, true>;

  m_weightedGeoBlk        = xWeightedGeoBlk_SSE<vext>;
  m_weightedGeoBlkRounded = xWeightedGeoBlkRounded_SSE<vext>;
  m_weightedSgpm          = xWeightedSgpm_SSE<vext>;
  m_sadTM                 = xSadTM_SSE<vext>;
  m_sgpmSadTM             = xSgpmSadTM_SSE<vext>;
}

template void InterpolationFilter::_initInterpolationFilterX86<SIMDX86>();

#endif   // #ifdef TARGET_SIMD_X86
//! \}
