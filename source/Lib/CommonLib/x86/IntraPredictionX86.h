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
 * \brief Implementation of IntraPrediction class
 */
// ====================================================================================================================
// Includes
// ====================================================================================================================

#include "CommonDefX86.h"
#include "../IntraPrediction.h"
#include "CommonLib/InterpolationFilter.h"

//! \ingroup CommonLib
//! \{

#ifdef TARGET_SIMD_X86

#if ENABLE_SIMD_OPT_INTRAPRED

template<X86_VEXT vext> void simdIntraPredAngleCpy(const Pel *refMain, Pel *pDsty, const int width, const int height,
                                                   const ptrdiff_t dstStride, const int intraPredAngle,
                                                   const int multiRefIdx, bool isExt)
{
  int x, y, deltaPos, deltaInt;
  int refMainIndex;
  deltaPos       = intraPredAngle * (1 + multiRefIdx);
  int deltaShift = isExt ? 6 : 5;

#ifdef USE_AVX2
  if (width >= 16)
  {
    for (y = 0; y < height; y++)
    {
      deltaInt = deltaPos >> deltaShift;
      for (x = 0; x < width; x += 16)
      {
        refMainIndex  = x + deltaInt + 1;
        __m256i vpred = _mm256_lddqu_si256((__m256i *)&refMain[refMainIndex]);
        _mm256_storeu_si256((__m256i *)&pDsty[x], vpred);
      }
      pDsty += dstStride;
      deltaPos += intraPredAngle;
    }
  }
  else
#endif
    if (width >= 8)
  {
    for (y = 0; y < height; y++)
    {
      deltaInt = deltaPos >> deltaShift;
      for (x = 0; x < width; x += 8)
      {
        refMainIndex  = x + deltaInt + 1;
        __m128i vpred = _mm_lddqu_si128((__m128i *)&refMain[refMainIndex]);
        _mm_storeu_si128((__m128i *)&pDsty[x], vpred);
      }
      pDsty += dstStride;
      deltaPos += intraPredAngle;
    }
  }
  else if (width == 4)
  {
    for (y = 0; y < height; y++)
    {
      deltaInt = deltaPos >> deltaShift;
      {
        refMainIndex  = deltaInt + 1;
        __m128i vpred = _mm_loadl_epi64((__m128i *)&refMain[refMainIndex]);
        _mm_storel_epi64((__m128i *)&pDsty[0], vpred);
      }
      pDsty += dstStride;
      deltaPos += intraPredAngle;
    }
  }
  else
  {
    for (y = 0, deltaPos = intraPredAngle * (1 + multiRefIdx); y < height;
         y++, deltaPos += intraPredAngle, pDsty += dstStride)
    {
      const int deltaInt = deltaPos >> deltaShift;
      // Just copy the integer samples
      for (x = 0; x < width; x++)
      {
        pDsty[x] = refMain[x + deltaInt + 1];
      }
    }
  }
}

template<X86_VEXT vext, int W> void IntraAnglePDPC_SIMD(Pel *pDsty, const ptrdiff_t dstStride, const Pel *refSide,
                                                        const int width, const int height, int scale, int invAngle)
{
  if (W >= 16)
  {
#ifdef USE_AVX2
    if (scale <= 2)
    {
      ALIGN_DATA(MEMORY_ALIGN_DEF_SIZE, short ref[16]);
      VALGRIND_MEMCLEAR(ref, sizeof(ref));
      // memclear only needed for valgrind to avoid error "not initialized".
      // Not initialized data from "ref" are multiplicated with 0 later anyhow
      __m256i wl16;
      if (scale == 0)
      {
        wl16  = _mm256_set_epi16(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 8, 32);
        scale = 3;
      }
      else if (scale == 1)
      {
        wl16  = _mm256_set_epi16(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 4, 8, 16, 32);
        scale = 6;
      }
      else
      {
        wl16  = _mm256_set_epi16(0, 0, 0, 0, 1, 1, 2, 2, 4, 4, 8, 8, 16, 16, 32, 32);
        scale = 12;
      }
      __m256i v32 = _mm256_set1_epi32(32);
      for (int y = 0; y < height; y++, pDsty += dstStride)
      {
        int invAngleSum = 256;
        for (int x = 0; x < scale; x++)
        {
          invAngleSum += invAngle;
          ref[x] = refSide[y + (invAngleSum >> 9) + 1];
        }
        __m256i xleft  = _mm256_load_si256((__m256i *)&ref[0]);
        __m256i xdst   = _mm256_loadu_si256((__m256i *)pDsty);
        __m256i xdstlo = _mm256_sub_epi16(xleft, xdst);
        __m256i tmplo  = _mm256_mullo_epi16(xdstlo, wl16);
        __m256i tmphi  = _mm256_mulhi_epi16(xdstlo, wl16);
        xdstlo         = _mm256_unpacklo_epi16(tmplo, tmphi);  // low
        tmphi          = _mm256_unpackhi_epi16(tmplo, tmphi);  // high

        tmplo = _mm256_add_epi32(xdstlo, v32);
        tmphi = _mm256_add_epi32(tmphi, v32);
        tmplo = _mm256_srai_epi32(tmplo, 6);
        tmphi = _mm256_srai_epi32(tmphi, 6);

        tmplo = _mm256_packs_epi32(tmplo, tmphi);
        xdst  = _mm256_add_epi16(tmplo, xdst);
        _mm256_storeu_si256((__m256i *)(pDsty), xdst);
      }
    }
    else
#endif
      for (int y = 0; y < height; y++, pDsty += dstStride)
      {
        int invAngleSum = 256;
        for (int x = 0; x < std::min(3 << scale, width); x++)
        {
          invAngleSum += invAngle;
          int wL   = 32 >> (2 * x >> scale);
          Pel left = refSide[y + (invAngleSum >> 9) + 1];
          pDsty[x] = pDsty[x] + ((wL * (left - pDsty[x]) + 32) >> 6);
        }
      }
  }
  else
  {
    ALIGN_DATA(MEMORY_ALIGN_DEF_SIZE, short ref[8]);
    VALGRIND_MEMCLEAR(ref, sizeof(ref));
    // memclear only needed for valgrind to avoid error "not initialized".
    // Not initialized data from "ref" are multiplicated with 0 later anyhow
    __m128i wl16;
    if (scale == 0)
    {
      wl16  = _mm_set_epi16(0, 0, 0, 0, 0, 2, 8, 32);
      scale = 3;
    }
    else if (scale == 1)
    {
      wl16  = _mm_set_epi16(0, 0, 1, 2, 4, 8, 16, 32);
      scale = 6;
    }
    else
    {
      wl16  = _mm_set_epi16(4, 4, 8, 8, 16, 16, 32, 32);
      scale = 8;
    }

    int xlim = std::min(scale, width);

    __m128i v32 = _mm_set1_epi32(32);
    for (int y = 0; y < height; y++, pDsty += dstStride)
    {
      int invAngleSum = 256;
      for (int x = 0; x < xlim; x++)
      {
        invAngleSum += invAngle;
        ref[x] = refSide[y + (invAngleSum >> 9) + 1];
      }

      __m128i xleft;
      __m128i xdst;
      if (W == 8)
      {
        xleft = _mm_load_si128((__m128i *)&ref[0]);
        xdst  = _mm_loadu_si128((__m128i *)pDsty);
      }
      else
      {
        xleft = _mm_load_si128((__m128i *)&ref[0]);
        xdst  = _mm_loadu_si64((__m128i *)pDsty);
      }
      __m128i xdstlo = _mm_sub_epi16(xleft, xdst);
      __m128i tmplo  = _mm_mullo_epi16(xdstlo, wl16);
      __m128i tmphi  = _mm_mulhi_epi16(xdstlo, wl16);
      xdstlo         = _mm_unpacklo_epi16(tmplo, tmphi);  // low
      tmphi          = _mm_unpackhi_epi16(tmplo, tmphi);  // high

      tmplo = _mm_add_epi32(xdstlo, v32);
      tmphi = _mm_add_epi32(tmphi, v32);
      tmplo = _mm_srai_epi32(tmplo, 6);
      tmphi = _mm_srai_epi32(tmphi, 6);

      tmplo = _mm_packs_epi32(tmplo, tmphi);
      xdst  = _mm_add_epi16(tmplo, xdst);
      if (W == 8)
      {
        _mm_storeu_si128((__m128i *)(pDsty), xdst);
      }
      else if (W == 4)
      {
        _mm_storel_epi64((__m128i *)(pDsty), xdst);
      }
      else
      {
        THROW("wrong blocksize");
      }
    }
  }
}

template<X86_VEXT vext> void simdIntraAnglePDPC(Pel *pDsty, const ptrdiff_t dstStride, const Pel *refSide,
                                                const int width, const int height, const int scale,
                                                const int absInvAngle)
{
  if (width >= 16)
  {
    IntraAnglePDPC_SIMD<vext, 16>(pDsty, dstStride, refSide, width, height, scale, absInvAngle);
  }
  else if (width == 8)
  {
    IntraAnglePDPC_SIMD<vext, 8>(pDsty, dstStride, refSide, width, height, scale, absInvAngle);
  }
  else if (width == 4)
  {
    IntraAnglePDPC_SIMD<vext, 4>(pDsty, dstStride, refSide, width, height, scale, absInvAngle);
  }
  else
  {
    for (int y = 0; y < height; y++, pDsty += dstStride)
    {
      int invAngleSum = 256;
      for (int x = 0; x < 2; x++)
      {
        invAngleSum += absInvAngle;
        int wL   = 32 >> (2 * x >> scale);
        Pel left = refSide[y + (invAngleSum >> 9) + 1];
        pDsty[x] = pDsty[x] + ((wL * (left - pDsty[x]) + 32) >> 6);
      }
    }
  }
#if USE_AVX2
  _mm256_zeroupper();
#endif
}
template<X86_VEXT vext> void simdIntraAngleGradPDPC(Pel *pDsty, const ptrdiff_t dstStride, const Pel *refSide,
                                                    const Pel *refMain, const int width, const int height, int scale,
                                                    const int intraPredAngle, const int multiRefIdx,
                                                    const ClpRng &clpRng, bool useExt)
{
  int gradOffset = useExt ? 32 : 16;
  int gradShift  = useExt ? 6 : 5;
  int gradMask   = useExt ? 63 : 31;
  if (width >= 16)
  {
#ifdef USE_AVX2
    if (scale <= 2)
    {
      int     deltaPos = intraPredAngle * (1 + multiRefIdx);
      __m256i wl16;
      if (scale == 0)
      {
        wl16 = _mm256_set_epi16(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 8, 32);
      }
      else if (scale == 1)
      {
        wl16 = _mm256_set_epi16(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 4, 8, 16, 32);
      }
      else
      {
        wl16 = _mm256_set_epi16(0, 0, 0, 0, 1, 1, 2, 2, 4, 4, 8, 8, 16, 16, 32, 32);
      }
      __m256i v32 = _mm256_set1_epi32(32);
      __m256i xdst;
      __m256i xdiff;
      __m256i tmplo;
      __m256i tmphi;
      __m256i vbdmin = _mm256_set1_epi16(clpRng.min);
      __m256i vbdmax = _mm256_set1_epi16(clpRng.max);

      for (int y = 0; y < height; y++)
      {
        const int deltaInt   = deltaPos >> gradShift;
        const int deltaFract = deltaPos & gradMask;
        const Pel left       = refSide[1 + y];
        const Pel topLeft =
          refMain[deltaInt] + ((deltaFract * (refMain[deltaInt + 1] - refMain[deltaInt]) + gradOffset) >> gradShift);

        xdiff = _mm256_set1_epi16(left - topLeft);
        tmplo = _mm256_mullo_epi16(xdiff, wl16);
        tmphi = _mm256_mulhi_epi16(xdiff, wl16);

        xdiff = _mm256_unpacklo_epi16(tmplo, tmphi);  // low
        tmphi = _mm256_unpackhi_epi16(tmplo, tmphi);  // high

        tmplo = _mm256_add_epi32(xdiff, v32);
        tmphi = _mm256_add_epi32(tmphi, v32);
        tmplo = _mm256_srai_epi32(tmplo, 6);
        tmphi = _mm256_srai_epi32(tmphi, 6);

        tmplo = _mm256_packs_epi32(tmplo, tmphi);  //  ((wL * (left - topLeft) + 32) >> 6)

        xdst = _mm256_loadu_si256((__m256i *)pDsty);
        xdst = _mm256_add_epi16(tmplo, xdst);
        xdst = _mm256_min_epi16(vbdmax, _mm256_max_epi16(vbdmin, xdst));
        _mm256_storeu_si256((__m256i *)(pDsty), xdst);

        deltaPos += intraPredAngle;
        pDsty += dstStride;
      }
    }
    else
#endif
      for (int y = 0, deltaPos = intraPredAngle * (1 + multiRefIdx); y < height;
           y++, deltaPos += intraPredAngle, pDsty += dstStride)
      {
        const int deltaInt   = deltaPos >> gradShift;
        const int deltaFract = deltaPos & gradMask;

        const Pel left = refSide[1 + y];
        const Pel topLeft =
          refMain[deltaInt] + ((deltaFract * (refMain[deltaInt + 1] - refMain[deltaInt]) + gradOffset) >> gradShift);

        for (int x = 0; x < std::min(3 << scale, width); x++)
        {
          int       wL  = 32 >> (2 * x >> scale);
          const Pel val = pDsty[x];
          pDsty[x]      = ClipPel(val + ((wL * (left - topLeft) + 32) >> 6), clpRng);
        }
      }
  }
  else
  {
    // width 4/8
    int     deltaPos = intraPredAngle * (1 + multiRefIdx);
    __m128i wl16;
    if (scale == 0)
    {
      wl16 = _mm_set_epi16(0, 0, 0, 0, 0, 2, 8, 32);
    }
    else if (scale == 1)
    {
      wl16 = _mm_set_epi16(0, 0, 1, 2, 4, 8, 16, 32);
    }
    else
    {
      wl16 = _mm_set_epi16(4, 4, 8, 8, 16, 16, 32, 32);
    }
    __m128i v32 = _mm_set1_epi32(32);
    __m128i xdst;
    __m128i xdiff;
    __m128i tmplo;
    __m128i tmphi;
    __m128i vbdmin = _mm_set1_epi16(clpRng.min);
    __m128i vbdmax = _mm_set1_epi16(clpRng.max);

    for (int y = 0; y < height; y++)
    {
      const int deltaInt   = deltaPos >> gradShift;
      const int deltaFract = deltaPos & gradMask;
      const Pel left       = refSide[1 + y];
      const Pel topLeft =
        refMain[deltaInt] + ((deltaFract * (refMain[deltaInt + 1] - refMain[deltaInt]) + gradOffset) >> gradShift);

      xdiff = _mm_set1_epi16(left - topLeft);
      tmplo = _mm_mullo_epi16(xdiff, wl16);
      tmphi = _mm_mulhi_epi16(xdiff, wl16);

      xdiff = _mm_unpacklo_epi16(tmplo, tmphi);  // low
      tmphi = _mm_unpackhi_epi16(tmplo, tmphi);  // high

      tmplo = _mm_add_epi32(xdiff, v32);
      tmphi = _mm_add_epi32(tmphi, v32);
      tmplo = _mm_srai_epi32(tmplo, 6);
      tmphi = _mm_srai_epi32(tmphi, 6);

      tmplo = _mm_packs_epi32(tmplo, tmphi);  //  ((wL * (left - topLeft) + 32) >> 6)
      if (width == 8)
      {
        xdst = _mm_loadu_si128((__m128i *)pDsty);
        xdst = _mm_add_epi16(tmplo, xdst);
        xdst = _mm_min_epi16(vbdmax, _mm_max_epi16(vbdmin, xdst));
        _mm_storeu_si128((__m128i *)(pDsty), xdst);
      }
      else
      {
        xdst = _mm_loadu_si64((__m128i *)pDsty);
        xdst = _mm_add_epi16(tmplo, xdst);
        xdst = _mm_min_epi16(vbdmax, _mm_max_epi16(vbdmin, xdst));
        _mm_storel_epi64((__m128i *)(pDsty), xdst);
      }
      deltaPos += intraPredAngle;
      pDsty += dstStride;
    }
  }
}
template<X86_VEXT vext> void simdIntraPredAngleLuma(const Pel *refMain, Pel *pDsty, const int width, const int height,
                                                    const ptrdiff_t dstStride, const int intraPredAngle,
                                                    const int multiRefIdx, const ClpRng &clpRng,
                                                    const bool interpolationFlag, bool useExt)
{
  int gradShift = useExt ? 6 : 5;
  int gradMask  = useExt ? 63 : 31;

  int        deltaPos       = intraPredAngle * (1 + multiRefIdx);
  const bool useCubicFilter = !interpolationFlag;
  int16_t   *pDst;
  if (width >= 8)
  {
    if (vext >= AVX2)
    {
#ifdef USE_AVX2
      __m256i shflmask1 =
        _mm256_set_epi8(0xd, 0xc, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0x5, 0x4, 0x9, 0x8, 0x7,
                        0x6, 0x5, 0x4, 0x3, 0x2, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0);
      __m256i shflmask2 =
        _mm256_set_epi8(0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0, 0x9, 0x8, 0x7,
                        0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0);
      __m256i offset = _mm256_set1_epi32(128);
      if ((width & 15) == 0)
      {
        __m256i vbdmin = _mm256_set1_epi16(clpRng.min);
        __m256i vbdmax = _mm256_set1_epi16(clpRng.max);
        for (int y = 0; y < height; y++)
        {
          const int deltaInt   = deltaPos >> gradShift;
          const int deltaFract = deltaPos & gradMask;

        // 4-tap Gaussian.
          const TFilterCoeff intraSmoothingFilter[6]  = { TFilterCoeff(0),
                                                          TFilterCoeff(64 - (deltaFract << 1)),
                                                          TFilterCoeff(128 - (deltaFract << 1)),
                                                          TFilterCoeff(64 + (deltaFract << 1)),
                                                          TFilterCoeff(deltaFract << 1),
                                                          TFilterCoeff(0) };
        // 6-tap Gaussian (for larger blocks).
          const TFilterCoeff intraSmoothingFilter2[6] = {
            TFilterCoeff(16 - (deltaFract >> 1)),     TFilterCoeff(64 - 3 * (deltaFract >> 1)),
            TFilterCoeff(96 - (deltaFract)),          TFilterCoeff(64 + (deltaFract)),
            TFilterCoeff(16 + 3 * (deltaFract >> 1)), TFilterCoeff((deltaFract >> 1))
          };

          const TFilterCoeff intraSmoothingFilterExt[6]  = { TFilterCoeff(0),
                                                             TFilterCoeff(64 - (deltaFract)),
                                                             TFilterCoeff(128 - (deltaFract)),
                                                             TFilterCoeff(64 + (deltaFract)),
                                                             TFilterCoeff(deltaFract),
                                                             TFilterCoeff(0) };
          const TFilterCoeff intraSmoothingFilter2Ext[6] = {
            TFilterCoeff(16 - (deltaFract >> 2)),     TFilterCoeff(64 - 3 * (deltaFract >> 2)),
            TFilterCoeff(96 - (deltaFract >> 1)),     TFilterCoeff(64 + (deltaFract >> 1)),
            TFilterCoeff(16 + 3 * (deltaFract >> 2)), TFilterCoeff((deltaFract >> 2))
          };
          const TFilterCoeff *const f = (useCubicFilter)
            ? (useExt ? InterpolationFilter::getIntraLumaFilterTableExt(deltaFract)
                      : InterpolationFilter::getIntraLumaFilterTable(deltaFract))
            : ((width >= 32 && height >= 32) ? (useExt ? intraSmoothingFilter2Ext : intraSmoothingFilter2)
                                             : (useExt ? intraSmoothingFilterExt : intraSmoothingFilter));

    // TODO: Check if correct if IF_12TAP==0

          int refMainIndex = deltaInt;
          pDst             = &pDsty[y * dstStride];
          __m256i coeff    = _mm256_broadcastsi128_si256(
            _mm_set_epi16(f[3], f[2], f[1], f[0], f[3], f[2], f[1], f[0]));    // load 4 16 bit filter coeffs
          __m256i coeff2 = _mm256_broadcastsi128_si256(_mm_set_epi16(f[5], f[4], f[5], f[4], f[5], f[4], f[5], f[4]));

          for (int x = 0; x < width; x += 16)
          {
            __m256i src1 = _mm256_broadcastsi128_si256(_mm_loadu_si128((const __m128i *)&refMain[refMainIndex - 1]));
            __m256i src2 =
              _mm256_broadcastsi128_si256(_mm_loadu_si128((const __m128i *)&refMain[refMainIndex + 4 - 1]));
            __m256i src3 = _mm256_loadu_si256((const __m256i *)&refMain[refMainIndex + 4 - 1]);
            src3         = _mm256_permute4x64_epi64(src3, 0x94);    // 3 4 5 6 7 8 9 10  7 8 9 10 11 12 13 14
            src3         = _mm256_shuffle_epi8(src3, shflmask2);
            ;  // 3 4  4 5  5 6  6 7    7 8   8 9  9 10  10 11
            src1        = _mm256_shuffle_epi8(src1, shflmask1);   // -1 0 1 2  0 1 2 3  1 2 3 4  2 3 4 5
            src2        = _mm256_shuffle_epi8(src2, shflmask1);   // 3 4 5 6  4 5 6 7  5 6 7 8  6 7 8 9
            src1        = _mm256_madd_epi16(src1, coeff);
            src2        = _mm256_madd_epi16(src2, coeff);
            __m256i sum = _mm256_hadd_epi32(src1, src2);
            sum         = _mm256_permute4x64_epi64(sum, 0xD8);

            src3 = _mm256_madd_epi16(coeff2, src3);
            sum  = _mm256_add_epi32(sum, src3);
            sum  = _mm256_add_epi32(sum, offset);
            sum  = _mm256_srai_epi32(sum, 8);

            refMainIndex += 8;
            src1 = _mm256_broadcastsi128_si256(_mm_loadu_si128((__m128i const *)&refMain[refMainIndex - 1]));
            src2 = _mm256_broadcastsi128_si256(_mm_loadu_si128((__m128i const *)&refMain[refMainIndex + 4 - 1]));
            src3 = _mm256_loadu_si256((const __m256i *)&refMain[refMainIndex + 4 - 1]);

            src3 = _mm256_permute4x64_epi64(src3, 0x94);   // 3 4 5 6 7 8 9 10  7 8 9 10 11 12 13 14
            src3 = _mm256_shuffle_epi8(src3, shflmask2);
            ;   // 3 4  4 5  5 6  6 7    7 8   8 9  9 10  10 11

            src1 = _mm256_shuffle_epi8(src1, shflmask1);   // -1 0 1 2  0 1 2 3 1 2 3 4  2 3 4 5
            src2 = _mm256_shuffle_epi8(src2, shflmask1);   // 3 4 5 6  4 5 6 7  5 6 7 8 6 7 8 9
            src1 = _mm256_madd_epi16(src1, coeff);
            src2 = _mm256_madd_epi16(src2, coeff);

            __m256i sum1 = _mm256_hadd_epi32(src1, src2);
            sum1         = _mm256_permute4x64_epi64(sum1, 0xD8);

            src3 = _mm256_madd_epi16(coeff2, src3);
            sum1 = _mm256_add_epi32(sum1, src3);

            sum1         = _mm256_add_epi32(sum1, offset);
            sum1         = _mm256_srai_epi32(sum1, 8);
            __m256i src0 = _mm256_packs_epi32(sum, sum1);
            src0         = _mm256_permute4x64_epi64(src0, 0xD8);
            refMainIndex += 8;
            src0 = _mm256_min_epi16(vbdmax, _mm256_max_epi16(vbdmin, src0));
            _mm256_storeu_si256((__m256i *)(pDst + x), src0);
          }
          deltaPos += intraPredAngle;
        }
      }
      else   // width =8
      {
        __m128i vbdmin = _mm_set1_epi16(clpRng.min);
        __m128i vbdmax = _mm_set1_epi16(clpRng.max);
        for (int y = 0; y < height; y++)
        {
          const int deltaInt   = deltaPos >> gradShift;
          const int deltaFract = deltaPos & gradMask;

          // 4-tap Gaussian.
          const TFilterCoeff intraSmoothingFilter[6]  = { TFilterCoeff(0),
                                                          TFilterCoeff(64 - (deltaFract << 1)),
                                                          TFilterCoeff(128 - (deltaFract << 1)),
                                                          TFilterCoeff(64 + (deltaFract << 1)),
                                                          TFilterCoeff(deltaFract << 1),
                                                          TFilterCoeff(0) };
          // 6-tap Gaussian (for larger blocks).
          const TFilterCoeff intraSmoothingFilter2[6] = {
            TFilterCoeff(16 - (deltaFract >> 1)),     TFilterCoeff(64 - 3 * (deltaFract >> 1)),
            TFilterCoeff(96 - (deltaFract)),          TFilterCoeff(64 + (deltaFract)),
            TFilterCoeff(16 + 3 * (deltaFract >> 1)), TFilterCoeff((deltaFract >> 1))
          };

          const TFilterCoeff intraSmoothingFilterExt[6]  = { TFilterCoeff(0),
                                                             TFilterCoeff(64 - (deltaFract)),
                                                             TFilterCoeff(128 - (deltaFract)),
                                                             TFilterCoeff(64 + (deltaFract)),
                                                             TFilterCoeff(deltaFract),
                                                             TFilterCoeff(0) };
          const TFilterCoeff intraSmoothingFilter2Ext[6] = {
            TFilterCoeff(16 - (deltaFract >> 2)),     TFilterCoeff(64 - 3 * (deltaFract >> 2)),
            TFilterCoeff(96 - (deltaFract >> 1)),     TFilterCoeff(64 + (deltaFract >> 1)),
            TFilterCoeff(16 + 3 * (deltaFract >> 2)), TFilterCoeff((deltaFract >> 2))
          };

          const TFilterCoeff *const f = (useCubicFilter)
            ? (useExt ? InterpolationFilter::getIntraLumaFilterTableExt(deltaFract)
                      : InterpolationFilter::getIntraLumaFilterTable(deltaFract))
            : ((width >= 32 && height >= 32) ? (useExt ? intraSmoothingFilter2Ext : intraSmoothingFilter2)
                                             : (useExt ? intraSmoothingFilterExt : intraSmoothingFilter));

          // TODO: Check if correct if IF_12TAP==0

          int refMainIndex = deltaInt;
          pDst             = &pDsty[y * dstStride];
          __m256i coeff    = _mm256_broadcastsi128_si256(
            _mm_set_epi16(f[3], f[2], f[1], f[0], f[3], f[2], f[1], f[0]));   // load 4 16 bit filter coeffs
          __m256i coeff2 = _mm256_broadcastsi128_si256(_mm_set_epi16(f[5], f[4], f[5], f[4], f[5], f[4], f[5], f[4]));

          __m256i src1 = _mm256_broadcastsi128_si256(_mm_loadu_si128((__m128i const *)&refMain[refMainIndex - 1]));
          __m256i src2 = _mm256_broadcastsi128_si256(_mm_loadu_si128((__m128i const *)&refMain[refMainIndex + 4 - 1]));
          __m256i src3 = _mm256_loadu_si256((const __m256i *)&refMain[refMainIndex + 4 - 1]);
          src3         = _mm256_permute4x64_epi64(src3, 0x94);   // 3 4 5 6 7 8 9 10  7 8 9 10 11 12 13 14
          src3         = _mm256_shuffle_epi8(src3, shflmask2);
          ;   // 3 4  4 5  5 6  6 7    7 8   8 9  9 10  10 11

          src1        = _mm256_shuffle_epi8(src1, shflmask1);   // -1 0 1 2  0 1 2 3 1 2 3 4  2 3 4 5
          src2        = _mm256_shuffle_epi8(src2, shflmask1);   // 3 4 5 6  4 5 6 7  5 6 7 8 6 7 8 9
          src1        = _mm256_madd_epi16(src1, coeff);
          src2        = _mm256_madd_epi16(src2, coeff);
          __m256i sum = _mm256_hadd_epi32(src1, src2);
          sum         = _mm256_permute4x64_epi64(sum, 0xD8);

          src3 = _mm256_madd_epi16(coeff2, src3);
          sum  = _mm256_add_epi32(sum, src3);

          sum             = _mm256_add_epi32(sum, offset);
          sum             = _mm256_srai_epi32(sum, 8);
          __m256i src0    = _mm256_permute4x64_epi64(_mm256_packs_epi32(sum, sum), 0x88);
          __m128i dest128 = _mm256_castsi256_si128(src0);
          dest128         = _mm_min_epi16(vbdmax, _mm_max_epi16(vbdmin, dest128));
          _mm_storeu_si128((__m128i *)(pDst), dest128);
          deltaPos += intraPredAngle;
        }
      }
#endif
    }
    else   // SSE widt >=8
    {
      __m128i shflmask1 = _mm_set_epi8(0x9, 0x8, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0);
      __m128i shflmask2 = _mm_set_epi8(0xd, 0xc, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0x5, 0x4);
      __m128i shflmask3 = _mm_set_epi8(0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0);
      __m128i offset    = _mm_set1_epi32(128);
      __m128i vbdmin    = _mm_set1_epi16(clpRng.min);
      __m128i vbdmax    = _mm_set1_epi16(clpRng.max);
      for (int y = 0; y < height; y++)
      {
        const int deltaInt   = deltaPos >> gradShift;
        const int deltaFract = deltaPos & gradMask;

        // 4-tap Gaussian.
        const TFilterCoeff intraSmoothingFilter[6]  = { TFilterCoeff(0),
                                                        TFilterCoeff(64 - (deltaFract << 1)),
                                                        TFilterCoeff(128 - (deltaFract << 1)),
                                                        TFilterCoeff(64 + (deltaFract << 1)),
                                                        TFilterCoeff(deltaFract << 1),
                                                        TFilterCoeff(0) };
        // 6-tap Gaussian (for larger blocks).
        const TFilterCoeff intraSmoothingFilter2[6] = {
          TFilterCoeff(16 - (deltaFract >> 1)),     TFilterCoeff(64 - 3 * (deltaFract >> 1)),
          TFilterCoeff(96 - (deltaFract)),          TFilterCoeff(64 + (deltaFract)),
          TFilterCoeff(16 + 3 * (deltaFract >> 1)), TFilterCoeff((deltaFract >> 1))
        };

        const TFilterCoeff intraSmoothingFilterExt[6]  = { TFilterCoeff(0),
                                                           TFilterCoeff(64 - (deltaFract)),
                                                           TFilterCoeff(128 - (deltaFract)),
                                                           TFilterCoeff(64 + (deltaFract)),
                                                           TFilterCoeff(deltaFract),
                                                           TFilterCoeff(0) };
        const TFilterCoeff intraSmoothingFilter2Ext[6] = {
          TFilterCoeff(16 - (deltaFract >> 2)),     TFilterCoeff(64 - 3 * (deltaFract >> 2)),
          TFilterCoeff(96 - (deltaFract >> 1)),     TFilterCoeff(64 + (deltaFract >> 1)),
          TFilterCoeff(16 + 3 * (deltaFract >> 2)), TFilterCoeff((deltaFract >> 2))
        };

        const TFilterCoeff *const f = (useCubicFilter)
          ? (useExt ? InterpolationFilter::getIntraLumaFilterTableExt(deltaFract)
                    : InterpolationFilter::getIntraLumaFilterTable(deltaFract))
          : ((width >= 32 && height >= 32) ? (useExt ? intraSmoothingFilter2Ext : intraSmoothingFilter2)
                                           : (useExt ? intraSmoothingFilterExt : intraSmoothingFilter));

        // TODO: Check if correct if IF_12TAP==0
        int refMainIndex = deltaInt;
        pDst             = &pDsty[y * dstStride];
        __m128i coeff  = _mm_set_epi16(f[3], f[2], f[1], f[0], f[3], f[2], f[1], f[0]);   // load 4 16 bit filter coeffs
        __m128i coeff2 = _mm_set_epi16(f[5], f[4], f[5], f[4], f[5], f[4], f[5], f[4]);
        for (int x = 0; x < width; x += 8)
        {
          __m128i src0 = _mm_loadu_si128(
            (__m128i const *)&refMain[refMainIndex - 1]);   // load 8 16 bit reference Pels   -1 0 1 2 3 4 5 6
          __m128i src1 = _mm_shuffle_epi8(src0, shflmask1);   // -1 0 1 2  0 1 2 3
          __m128i src2 = _mm_shuffle_epi8(src0, shflmask2);   // 1 2 3 4  2 3 4 5

          __m128i src3 = _mm_loadu_si128(
            (__m128i const *)&refMain[refMainIndex + 3]);   // load 8 16 bit reference Pels   3 4 5 6 7 8 9 10
          src3        = _mm_shuffle_epi8(src3, shflmask3);   // 3 4  4 5  5 6   6 7
          src0        = _mm_madd_epi16(coeff, src1);
          src1        = _mm_madd_epi16(coeff, src2);
          __m128i sum = _mm_hadd_epi32(src0, src1);
          src3        = _mm_madd_epi16(coeff2, src3);
          sum         = _mm_add_epi32(sum, src3);
          sum         = _mm_add_epi32(sum, offset);
          sum         = _mm_srai_epi32(sum, 8);

          refMainIndex += 4;
          src0 = _mm_loadu_si128(
            (__m128i const *)&refMain[refMainIndex - 1]);   // load 8 16 bit reference Pels   -1 0 1 2 3 4 5 6
          src1 = _mm_shuffle_epi8(src0, shflmask1);   // -1 0 1 2  0 1 2 3
          src2 = _mm_shuffle_epi8(src0, shflmask2);

          src3 = _mm_loadu_si128(
            (__m128i const *)&refMain[refMainIndex + 3]);   // load 8 16 bit reference Pels   3 4 5 6 7 8 9 10
          src3 = _mm_shuffle_epi8(src3, shflmask3);   // 3 4  4 5  5 6   6 7

          // 1 2 3 4  2 3 4 5
          src0         = _mm_madd_epi16(coeff, src1);
          src1         = _mm_madd_epi16(coeff, src2);
          __m128i sum1 = _mm_hadd_epi32(src0, src1);

          src3 = _mm_madd_epi16(coeff2, src3);
          sum1 = _mm_add_epi32(sum1, src3);

          sum1 = _mm_add_epi32(sum1, offset);
          sum1 = _mm_srai_epi32(sum1, 8);
          src0 = _mm_packs_epi32(sum, sum1);
          refMainIndex += 4;
          src0 = _mm_min_epi16(vbdmax, _mm_max_epi16(vbdmin, src0));
          _mm_storeu_si128((__m128i *)(pDst + x), src0);
        }
        deltaPos += intraPredAngle;
      }
    }
  }
  else if (width == 4)
  {
    __m128i shflmask1 = _mm_set_epi8(0x9, 0x8, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0);
    __m128i shflmask2 = _mm_set_epi8(0xd, 0xc, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0x5, 0x4);
    __m128i shflmask3 = _mm_set_epi8(0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0);

    __m128i offset = _mm_set1_epi32(128);
    __m128i vbdmin = _mm_set1_epi16(clpRng.min);
    __m128i vbdmax = _mm_set1_epi16(clpRng.max);
    for (int y = 0; y < height; y++)
    {
      const int deltaInt   = deltaPos >> gradShift;
      const int deltaFract = deltaPos & gradMask;

      // 4-tap Gaussian.
      const TFilterCoeff intraSmoothingFilter[6]  = { TFilterCoeff(0),
                                                      TFilterCoeff(64 - (deltaFract << 1)),
                                                      TFilterCoeff(128 - (deltaFract << 1)),
                                                      TFilterCoeff(64 + (deltaFract << 1)),
                                                      TFilterCoeff(deltaFract << 1),
                                                      TFilterCoeff(0) };
      // 6-tap Gaussian (for larger blocks).
      const TFilterCoeff intraSmoothingFilter2[6] = {
        TFilterCoeff(16 - (deltaFract >> 1)), TFilterCoeff(64 - 3 * (deltaFract >> 1)), TFilterCoeff(96 - (deltaFract)),
        TFilterCoeff(64 + (deltaFract)),      TFilterCoeff(16 + 3 * (deltaFract >> 1)), TFilterCoeff((deltaFract >> 1))
      };

      const TFilterCoeff intraSmoothingFilterExt[6]  = { TFilterCoeff(0),
                                                         TFilterCoeff(64 - (deltaFract)),
                                                         TFilterCoeff(128 - (deltaFract)),
                                                         TFilterCoeff(64 + (deltaFract)),
                                                         TFilterCoeff(deltaFract),
                                                         TFilterCoeff(0) };
      const TFilterCoeff intraSmoothingFilter2Ext[6] = {
        TFilterCoeff(16 - (deltaFract >> 2)),     TFilterCoeff(64 - 3 * (deltaFract >> 2)),
        TFilterCoeff(96 - (deltaFract >> 1)),     TFilterCoeff(64 + (deltaFract >> 1)),
        TFilterCoeff(16 + 3 * (deltaFract >> 2)), TFilterCoeff((deltaFract >> 2))
      };
      const TFilterCoeff *const f = (useCubicFilter)
        ? (useExt ? InterpolationFilter::getIntraLumaFilterTableExt(deltaFract)
                  : InterpolationFilter::getIntraLumaFilterTable(deltaFract))
        : ((width >= 32 && height >= 32) ? (useExt ? intraSmoothingFilter2Ext : intraSmoothingFilter2)
                                         : (useExt ? intraSmoothingFilterExt : intraSmoothingFilter));

      int refMainIndex = deltaInt;
      pDst             = &pDsty[y * dstStride];
      __m128i coeff    = _mm_set_epi16(f[3], f[2], f[1], f[0], f[3], f[2], f[1], f[0]);   // load 4 16 bit filter coeffs
      __m128i coeff2   = _mm_set_epi16(f[5], f[4], f[5], f[4], f[5], f[4], f[5], f[4]);
      {
        __m128i src0 = _mm_loadu_si128(
          (__m128i const *)&refMain[refMainIndex - 1]);   // load 8 16 bit reference Pels   -1 0 1 2 3 4 5 6
        __m128i src1 = _mm_shuffle_epi8(src0, shflmask1);   // -1 0 1 2  0 1 2 3
        __m128i src2 = _mm_shuffle_epi8(src0, shflmask2);   // 1 2 3 4  2 3 4 5
        __m128i src3 = _mm_loadu_si128(
          (__m128i const *)&refMain[refMainIndex + 3]);   // load 8 16 bit reference Pels   3 4 5 6 7 8 9 10
        src3        = _mm_shuffle_epi8(src3, shflmask3);   // 3 4  4 5  5 6   6 7
        src0        = _mm_madd_epi16(coeff, src1);
        src1        = _mm_madd_epi16(coeff, src2);
        __m128i sum = _mm_hadd_epi32(src0, src1);
        src3        = _mm_madd_epi16(coeff2, src3);
        sum         = _mm_add_epi32(sum, src3);
        sum         = _mm_add_epi32(sum, offset);
        sum         = _mm_srai_epi32(sum, 8);
        src0        = _mm_packs_epi32(sum, sum);
        refMainIndex += 4;
        src0 = _mm_min_epi16(vbdmax, _mm_max_epi16(vbdmin, src0));
        _mm_storel_epi64((__m128i *)(pDst), src0);
      }
      deltaPos += intraPredAngle;
    }
  }
  else
  {
    THROW("Unsupported size in IntraPredAngleCore_SIMD");
  }
#if USE_AVX2
  _mm256_zeroupper();
#endif
}

template<X86_VEXT vext> void simdIntraPredAngleChroma(const Pel *refMain, Pel *pDsty, const int width, const int height,
                                                      const ptrdiff_t dstStride, const int intraPredAngle,
                                                      const int multiRefIdx)
{
  int x, y, deltaPos, deltaInt, deltaFract;
  int refMainIndex;
  deltaPos = intraPredAngle * (1 + multiRefIdx);
#ifdef USE_AVX2
  if (width >= 16)
  {
    for (y = 0; y < height; y++)
    {
      deltaInt       = deltaPos >> 5;
      deltaFract     = deltaPos & 31;
      __m256i vfract = _mm256_set1_epi16(deltaFract);
      __m256i v16    = _mm256_set1_epi32(16);
      for (x = 0; x < width; x += 16)
      {
        refMainIndex   = x + deltaInt + 1;
        __m256i vpred0 = _mm256_lddqu_si256((__m256i *)&refMain[refMainIndex]);
        __m256i vpred1 = _mm256_lddqu_si256((__m256i *)&refMain[refMainIndex + 1]);
        __m256i vdiff  = _mm256_sub_epi16(vpred1, vpred0);
        __m256i vmul0  = _mm256_mullo_epi16(vfract, vdiff);
        __m256i vmul1  = _mm256_mulhi_epi16(vfract, vdiff);
        __m256i vtmp0  = _mm256_unpacklo_epi16(vmul0, vmul1);
        __m256i vtmp1  = _mm256_unpackhi_epi16(vmul0, vmul1);
        vmul0          = _mm256_add_epi32(vtmp0, v16);
        vmul1          = _mm256_add_epi32(vtmp1, v16);
        vmul0          = _mm256_srai_epi16(vmul0, 5);
        vmul1          = _mm256_srai_epi16(vmul1, 5);
        vmul0          = _mm256_packs_epi32(vmul0, vmul1);
        vpred0         = _mm256_add_epi16(vpred0, vmul0);
        _mm256_storeu_si256((__m256i *)&pDsty[x], vpred0);
      }
      pDsty += dstStride;
      deltaPos += intraPredAngle;
    }
  }
  else
#endif
    if (width >= 8)
  {
    for (y = 0; y < height; y++)
    {
      deltaInt       = deltaPos >> 5;
      deltaFract     = deltaPos & 31;
      __m128i vfract = _mm_set1_epi16(deltaFract);
      __m128i v16    = _mm_set1_epi32(16);
      for (x = 0; x < width; x += 8)
      {
        refMainIndex   = x + deltaInt + 1;
        __m128i vpred0 = _mm_lddqu_si128((__m128i *)&refMain[refMainIndex]);
        __m128i vpred1 = _mm_lddqu_si128((__m128i *)&refMain[refMainIndex + 1]);
        __m128i vdiff  = _mm_sub_epi16(vpred1, vpred0);
        __m128i vmul0  = _mm_mullo_epi16(vfract, vdiff);
        __m128i vmul1  = _mm_mulhi_epi16(vfract, vdiff);
        __m128i vtmp0  = _mm_unpacklo_epi16(vmul0, vmul1);
        __m128i vtmp1  = _mm_unpackhi_epi16(vmul0, vmul1);
        vmul0          = _mm_add_epi32(vtmp0, v16);
        vmul1          = _mm_add_epi32(vtmp1, v16);
        vmul0          = _mm_srai_epi16(vmul0, 5);
        vmul1          = _mm_srai_epi16(vmul1, 5);
        vmul0          = _mm_packs_epi32(vmul0, vmul1);
        vpred0         = _mm_add_epi16(vpred0, vmul0);
        _mm_storeu_si128((__m128i *)&pDsty[x], vpred0);
      }
      pDsty += dstStride;
      deltaPos += intraPredAngle;
    }
  }
  else
  {
    for (int y = 0, deltaPos = intraPredAngle * (1 + multiRefIdx); y < height;
         y++, deltaPos += intraPredAngle, pDsty += dstStride)
    {
      const int deltaInt   = deltaPos >> 5;
      const int deltaFract = deltaPos & 31;
      // Do linear filtering
      for (int x = 0; x < width; x++)
      {
        Pel p[2];
        p[0]     = refMain[deltaInt + x + 1];
        p[1]     = refMain[deltaInt + x + 2];
        pDsty[x] = p[0] + ((deltaFract * (p[1] - p[0]) + 16) >> 5);
      }
    }
  }
#if USE_AVX2
  _mm256_zeroupper();
#endif
}

template<X86_VEXT vext> void simdIntraHorVerPDPC(Pel *pDsty, const ptrdiff_t dstStride, Pel *refSide, const int width,
                                                 const int height, int scale, const Pel *refMain, const ClpRng &clpRng)
{
  const Pel topLeft = refMain[0];
  if (width >= 16)
  {
#ifdef USE_AVX2
    if (scale <= 2)
    {
      __m256i v32    = _mm256_set1_epi32(32);
      __m256i vbdmin = _mm256_set1_epi16(clpRng.min);
      __m256i vbdmax = _mm256_set1_epi16(clpRng.max);
      __m256i wl16;
      if (scale == 0)
      {
        wl16 = _mm256_set_epi16(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 8, 32);
      }
      else if (scale == 1)
      {
        wl16 = _mm256_set_epi16(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 4, 8, 16, 32);
      }
      else if (scale == 2)
      {
        wl16 = _mm256_set_epi16(0, 0, 0, 0, 1, 1, 2, 2, 4, 4, 8, 8, 16, 16, 32, 32);
      }
      else
      {
        wl16 = _mm256_set_epi16(0, 0, 0, 0, 1, 1, 2, 2, 4, 4, 8, 8, 16, 16, 32, 32);
      }
      __m256i xtopLeft = _mm256_set_epi16(topLeft, topLeft, topLeft, topLeft, topLeft, topLeft, topLeft, topLeft,
                                          topLeft, topLeft, topLeft, topLeft, topLeft, topLeft, topLeft, topLeft);

      for (int y = 0; y < height; y++, pDsty += dstStride)
      {
        // first column
        const Pel left  = refSide[1 + y];
        __m256i   xleft = _mm256_set_epi16(left, left, left, left, left, left, left, left, left, left, left, left, left,
                                           left, left, left);

        __m256i xdst = _mm256_loadu_si256((__m256i *)&refMain[1]);
        xleft        = _mm256_sub_epi16(xleft, xtopLeft);

        __m256i tmplo = _mm256_mullo_epi16(xleft, wl16);
        __m256i tmphi = _mm256_mulhi_epi16(xleft, wl16);
        xleft         = _mm256_unpacklo_epi16(tmplo, tmphi);   // low
        tmphi         = _mm256_unpackhi_epi16(tmplo, tmphi);   // high

        tmplo = _mm256_add_epi32(xleft, v32);
        tmphi = _mm256_add_epi32(tmphi, v32);
        tmplo = _mm256_srai_epi32(tmplo, 6);
        tmphi = _mm256_srai_epi32(tmphi, 6);

        tmplo = _mm256_packs_epi32(tmplo, tmphi);

        xdst = _mm256_adds_epi16(tmplo, xdst);
        xdst = _mm256_min_epi16(vbdmax, _mm256_max_epi16(vbdmin, xdst));
        _mm256_storeu_si256((__m256i *)(pDsty), xdst);

        // rest memcpy
        for (int x = 16; x < width; x += 16)
        {
          __m256i xdst = _mm256_loadu_si256((__m256i *)&refMain[x + 1]);
          _mm256_storeu_si256((__m256i *)(&pDsty[x]), xdst);
        }
      }
    }
    else
#endif
      for (int y = 0; y < height; y++)
      {
        memcpy(pDsty, &refMain[1], width * sizeof(Pel));
        const Pel left = refSide[1 + y];
        for (int x = 0; x < std::min(3 << scale, width); x++)
        {
          const int wL  = 32 >> (2 * x >> scale);
          const Pel val = pDsty[x];
          pDsty[x]      = ClipPel(val + ((wL * (left - topLeft) + 32) >> 6), clpRng);
        }
        pDsty += dstStride;
      }
  }
  else   // width <= 8
  {
    __m128i vbdmin = _mm_set1_epi16(clpRng.min);
    __m128i vbdmax = _mm_set1_epi16(clpRng.max);
    __m128i wl16;

    if (scale == 0)
    {
      wl16 = _mm_set_epi16(0, 0, 0, 0, 0, 2, 8, 32);
    }
    else if (scale == 1)
    {
      wl16 = _mm_set_epi16(0, 0, 1, 2, 4, 8, 16, 32);
    }
    else
    {
      wl16 = _mm_set_epi16(4, 4, 8, 8, 16, 16, 32, 32);
    }

    __m128i v32      = _mm_set1_epi32(32);
    __m128i xtopLeft = _mm_set_epi16(topLeft, topLeft, topLeft, topLeft, topLeft, topLeft, topLeft, topLeft);

    for (int y = 0; y < height; y++, pDsty += dstStride)
    {
      // first column
      const Pel left  = refSide[1 + y];
      __m128i   xleft = _mm_set_epi16(left, left, left, left, left, left, left, left);

      __m128i xdst = _mm_loadu_si128((__m128i *)&refMain[1]);
      xleft        = _mm_sub_epi16(xleft, xtopLeft);

      __m128i tmplo = _mm_mullo_epi16(xleft, wl16);
      __m128i tmphi = _mm_mulhi_epi16(xleft, wl16);
      xleft         = _mm_unpacklo_epi16(tmplo, tmphi);   // low
      tmphi         = _mm_unpackhi_epi16(tmplo, tmphi);   // high

      tmplo = _mm_add_epi32(xleft, v32);
      tmphi = _mm_add_epi32(tmphi, v32);
      tmplo = _mm_srai_epi32(tmplo, 6);
      tmphi = _mm_srai_epi32(tmphi, 6);

      tmplo = _mm_packs_epi32(tmplo, tmphi);

      xdst = _mm_adds_epi16(tmplo, xdst);
      xdst = _mm_min_epi16(vbdmax, _mm_max_epi16(vbdmin, xdst));

      if (width == 8)
      {
        _mm_storeu_si128((__m128i *)(pDsty), xdst);
      }
      else if (width == 4)
      {
        _mm_storel_epi64((__m128i *)(pDsty), xdst);
      }
      else
      {
        *(int32_t *)(pDsty) = _mm_cvtsi128_si32(xdst);
      }
    }
  }
#if USE_AVX2
  _mm256_zeroupper();
#endif
}

// NOTE: Bit-Limit - 24-bit source
template<X86_VEXT vext> void simdPredIntraPlanar(const CPelBuf &pSrc, PelBuf &pDst, const PlanarDirType &plDir)
{
  const uint32_t width  = pDst.width;
  const uint32_t height = pDst.height;
  const uint32_t log2W  = floorLog2(width);
  const uint32_t log2H  = floorLog2(height);

  const __m128i vLog2W = _mm_cvtsi32_si128(log2W);
  const __m128i vLog2H = _mm_cvtsi32_si128(log2H);

  if (plDir == PlanarDirType::NO_DIR)   // original planar
  {
    const uint32_t  offset = 1 << (log2W + log2H);
    const ptrdiff_t stride = pDst.stride;
    Pel            *pred   = pDst.buf;

    const Pel *ptrSrc = pSrc.buf;

    int leftColumn, rightColumn;
    Pel tmp;
    int topRight = pSrc.at(width + 1, 0);

    tmp                  = pSrc.at(height + 1, 1);
    __m128i bottomLeft16 = _mm_set_epi16(tmp, tmp, tmp, tmp, tmp, tmp, tmp, tmp);
    __m128i zero         = _mm_xor_si128(bottomLeft16, bottomLeft16);
    __m128i eight        = _mm_set_epi16(8, 8, 8, 8, 8, 8, 8, 8);
    __m128i offset32     = _mm_set_epi32(offset, offset, offset, offset);

    const uint32_t finalShift  = 1 + log2W + log2H;
    const __m128i  vFinalShift = _mm_cvtsi32_si128(finalShift);

    for (int y = 0; y < height; y++)
    {
      leftColumn            = pSrc.at(y + 1, 1);
      rightColumn           = topRight - leftColumn;
      leftColumn            = leftColumn << log2W;
      __m128i leftColumn32  = _mm_set_epi32(leftColumn, leftColumn, leftColumn, leftColumn);
      __m128i rightcolumn16 = _mm_set_epi16(rightColumn, rightColumn, rightColumn, rightColumn, rightColumn,
                                            rightColumn, rightColumn, rightColumn);
      __m128i y16           = _mm_set_epi16(y + 1, y + 1, y + 1, y + 1, y + 1, y + 1, y + 1, y + 1);
      __m128i x16           = _mm_set_epi16(8, 7, 6, 5, 4, 3, 2, 1);

      for (int x = 0; x < width; x += 8)
      {
        // topRow[x] = pSrc.at( x + 1, 0 );
        __m128i topRow16     = _mm_loadu_si128((__m128i const *)(ptrSrc + (x + 1)));
        // bottomRow[x] = bottomLeft - topRow[x];
        __m128i bottomRow16L = _mm_sub_epi16(bottomLeft16, topRow16);
        // (y+1)*bottomRow[x]
        __m128i tmpH         = _mm_mulhi_epi16(bottomRow16L, y16);
        __m128i tmpL         = _mm_mullo_epi16(bottomRow16L, y16);
        bottomRow16L         = _mm_unpacklo_epi16(tmpL, tmpH);
        __m128i bottomRow16H = _mm_unpackhi_epi16(tmpL, tmpH);
        // (topRow[x] topRow16H<< log2H)
        __m128i topRow32L    = _mm_unpacklo_epi16(topRow16, zero);
        __m128i topRow32H    = _mm_unpackhi_epi16(topRow16, zero);
        topRow32L            = _mm_sll_epi32(topRow32L, vLog2H);
        topRow32H            = _mm_sll_epi32(topRow32H, vLog2H);
        // vertPred    = (topRow[x] << log2H) + (y+1)*bottomRow[x];
        topRow32L            = _mm_add_epi32(topRow32L, bottomRow16L);
        topRow32H            = _mm_add_epi32(topRow32H, bottomRow16H);
        // horPred = leftColumn + (x+1)*rightColumn;
        tmpL                 = _mm_mullo_epi16(rightcolumn16, x16);
        tmpH                 = _mm_mulhi_epi16(rightcolumn16, x16);
        __m128i horpred32L   = _mm_unpacklo_epi16(tmpL, tmpH);
        __m128i horpred32H   = _mm_unpackhi_epi16(tmpL, tmpH);
        horpred32L           = _mm_add_epi32(leftColumn32, horpred32L);
        horpred32H           = _mm_add_epi32(leftColumn32, horpred32H);
        // pred[x]      = ( ( horPred << log2H ) + ( vertPred << log2W ) + offset ) >> finalShift;
        horpred32L           = _mm_sll_epi32(horpred32L, vLog2H);
        horpred32H           = _mm_sll_epi32(horpred32H, vLog2H);
        topRow32L            = _mm_sll_epi32(topRow32L, vLog2W);
        topRow32H            = _mm_sll_epi32(topRow32H, vLog2W);
        horpred32L           = _mm_add_epi32(horpred32L, topRow32L);
        horpred32H           = _mm_add_epi32(horpred32H, topRow32H);
        horpred32L           = _mm_add_epi32(horpred32L, offset32);
        horpred32H           = _mm_add_epi32(horpred32H, offset32);
        horpred32L           = _mm_srl_epi32(horpred32L, vFinalShift);
        horpred32H           = _mm_srl_epi32(horpred32H, vFinalShift);

        tmpL = _mm_packs_epi32(horpred32L, horpred32H);
        if (width >= 8)
        {
          _mm_storeu_si128((__m128i *)(pred + y * stride + x), (tmpL));
        }
        else if (width == 4)
        {
          _mm_storel_epi64((__m128i *)(pred + y * stride + x), (tmpL));
        }
        else if (width == 2)
        {
          *(int32_t *)(pred + y * stride + x) = _mm_cvtsi128_si32(tmpL);
        }
        else
        {
          pred[y * stride + x] = (Pel)_mm_extract_epi16(tmpL, 0);
        }
        x16 = _mm_add_epi16(x16, eight);
      }
    }
  }
  else if (plDir == PlanarDirType::HOR)   // planar hor
  {
    const ptrdiff_t stride = pDst.stride;
    Pel            *pred   = pDst.buf;
    const Pel      *ptrSrc = pSrc.buf;
    int             leftColumn, rightColumn;
    Pel             tmp;
    int             topRight = pSrc.at(width + 1, 0);
    tmp                      = pSrc.at(height + 1, 1);
    __m128i bottomLeft16     = _mm_set_epi16(tmp, tmp, tmp, tmp, tmp, tmp, tmp, tmp);
    __m128i zero             = _mm_xor_si128(bottomLeft16, bottomLeft16);
    __m128i eight            = _mm_set_epi16(8, 8, 8, 8, 8, 8, 8, 8);
    __m128i summand          = _mm_set1_epi32(1 << (log2W - 1));

    for (int y = 0; y < height; y++)
    {
      leftColumn            = pSrc.at(y + 1, 1);
      rightColumn           = topRight - leftColumn;
      leftColumn            = leftColumn << log2W;
      __m128i leftColumn32  = _mm_set_epi32(leftColumn, leftColumn, leftColumn, leftColumn);
      __m128i rightcolumn16 = _mm_set_epi16(rightColumn, rightColumn, rightColumn, rightColumn, rightColumn,
                                            rightColumn, rightColumn, rightColumn);
      __m128i y16           = _mm_set_epi16(y + 1, y + 1, y + 1, y + 1, y + 1, y + 1, y + 1, y + 1);
      __m128i x16           = _mm_set_epi16(8, 7, 6, 5, 4, 3, 2, 1);

      for (int x = 0; x < width; x += 8)
      {
        __m128i topRow16     = _mm_loadu_si128((__m128i const *)(ptrSrc + (x + 1)));
        __m128i bottomRow16L = _mm_sub_epi16(bottomLeft16, topRow16);
        __m128i tmpH         = _mm_mulhi_epi16(bottomRow16L, y16);
        __m128i tmpL         = _mm_mullo_epi16(bottomRow16L, y16);
        bottomRow16L         = _mm_unpacklo_epi16(tmpL, tmpH);
        __m128i bottomRow16H = _mm_unpackhi_epi16(tmpL, tmpH);

        __m128i topRow32L  = _mm_unpacklo_epi16(topRow16, zero);
        __m128i topRow32H  = _mm_unpackhi_epi16(topRow16, zero);
        topRow32L          = _mm_sll_epi32(topRow32L, vLog2H);
        topRow32H          = _mm_sll_epi32(topRow32H, vLog2H);
        topRow32L          = _mm_add_epi32(topRow32L, bottomRow16L);
        topRow32H          = _mm_add_epi32(topRow32H, bottomRow16H);
        tmpL               = _mm_mullo_epi16(rightcolumn16, x16);
        tmpH               = _mm_mulhi_epi16(rightcolumn16, x16);
        __m128i horpred32L = _mm_unpacklo_epi16(tmpL, tmpH);
        __m128i horpred32H = _mm_unpackhi_epi16(tmpL, tmpH);
        horpred32L         = _mm_add_epi32(leftColumn32, horpred32L);
        horpred32H         = _mm_add_epi32(leftColumn32, horpred32H);

        horpred32L = _mm_add_epi32(horpred32L, summand);
        horpred32H = _mm_add_epi32(horpred32H, summand);
        horpred32L = _mm_srl_epi32(horpred32L, vLog2W);
        horpred32H = _mm_srl_epi32(horpred32H, vLog2W);

        tmpL = _mm_packs_epi32(horpred32L, horpred32H);
        if (width >= 8)
        {
          _mm_storeu_si128((__m128i *)(pred + y * stride + x), (tmpL));
        }
        else if (width == 4)
        {
          _mm_storel_epi64((__m128i *)(pred + y * stride + x), (tmpL));
        }
        else if (width == 2)
        {
          *(int32_t *)(pred + y * stride + x) = _mm_cvtsi128_si32(tmpL);
        }
        else
        {
          pred[y * stride + x] = (Pel)_mm_extract_epi16(tmpL, 0);
        }
        x16 = _mm_add_epi16(x16, eight);
      }
    }
  }
  else   // planar ver
  {
    const ptrdiff_t stride = pDst.stride;
    Pel            *pred   = pDst.buf;
    const Pel      *ptrSrc = pSrc.buf;
    Pel             tmp;

    tmp                  = pSrc.at(height + 1, 1);
    __m128i bottomLeft16 = _mm_set_epi16(tmp, tmp, tmp, tmp, tmp, tmp, tmp, tmp);
    __m128i zero         = _mm_xor_si128(bottomLeft16, bottomLeft16);
    __m128i eight        = _mm_set_epi16(8, 8, 8, 8, 8, 8, 8, 8);
    __m128i summand      = _mm_set1_epi32(1 << (log2H - 1));

    for (int y = 0; y < height; y++)
    {
      __m128i y16 = _mm_set_epi16(y + 1, y + 1, y + 1, y + 1, y + 1, y + 1, y + 1, y + 1);
      __m128i x16 = _mm_set_epi16(8, 7, 6, 5, 4, 3, 2, 1);

      for (int x = 0; x < width; x += 8)
      {
        __m128i topRow16     = _mm_loadu_si128((__m128i const *)(ptrSrc + (x + 1)));
        __m128i bottomRow16L = _mm_sub_epi16(bottomLeft16, topRow16);
        __m128i tmpH         = _mm_mulhi_epi16(bottomRow16L, y16);
        __m128i tmpL         = _mm_mullo_epi16(bottomRow16L, y16);
        bottomRow16L         = _mm_unpacklo_epi16(tmpL, tmpH);
        __m128i bottomRow16H = _mm_unpackhi_epi16(tmpL, tmpH);

        __m128i topRow32L = _mm_unpacklo_epi16(topRow16, zero);
        __m128i topRow32H = _mm_unpackhi_epi16(topRow16, zero);
        topRow32L         = _mm_sll_epi32(topRow32L, vLog2H);
        topRow32H         = _mm_sll_epi32(topRow32H, vLog2H);
        topRow32L         = _mm_add_epi32(topRow32L, bottomRow16L);
        topRow32H         = _mm_add_epi32(topRow32H, bottomRow16H);

        topRow32L = _mm_add_epi32(topRow32L, summand);
        topRow32H = _mm_add_epi32(topRow32H, summand);
        topRow32L = _mm_srl_epi32(topRow32L, vLog2H);
        topRow32H = _mm_srl_epi32(topRow32H, vLog2H);

        tmpL = _mm_packs_epi32(topRow32L, topRow32H);
        if (width >= 8)
        {
          _mm_storeu_si128((__m128i *)(pred + y * stride + x), (tmpL));
        }
        else if (width == 4)
        {
          _mm_storel_epi64((__m128i *)(pred + y * stride + x), (tmpL));
        }
        else if (width == 2)
        {
          *(int32_t *)(pred + y * stride + x) = _mm_cvtsi128_si32(tmpL);
        }
        else
        {
          pred[y * stride + x] = (Pel)_mm_extract_epi16(tmpL, 0);
        }
        x16 = _mm_add_epi16(x16, eight);
      }
    }
  }
}

template<X86_VEXT vext> void simdIntraPredSampleFilter(const CPelBuf &Src, PelBuf &dstBuf)
{
  const int       iWidth    = dstBuf.width;
  const int       iHeight   = dstBuf.height;
  Pel            *pDst      = dstBuf.buf;
  const ptrdiff_t dstStride = dstBuf.stride;

  const Pel      *ptrSrc    = Src.buf;
  const ptrdiff_t srcStride = Src.stride;

  const int scale = ((floorLog2(iWidth * iHeight) - 2) >> 2);
  CHECK(scale < 0 || scale > 31, "PDPC: scale < 0 || scale > 2");

  if (scale < 3)
  {
#if USE_AVX2
    if (iWidth > 8)
    {
      __m256i tmplo, tmphi;
      __m256i w32 = _mm256_set_epi32(32, 32, 32, 32, 32, 32, 32, 32);
      __m256i wl16, wl16start;

      wl16start = _mm256_set_epi16(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 8, 32);

      if (scale == 1)
      {
        wl16start = _mm256_set_epi16(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 4, 8, 16, 32);
      }
      else if (scale == 2)
      {
        wl16start = _mm256_set_epi16(0, 0, 0, 0, 1, 1, 2, 2, 4, 4, 8, 8, 16, 16, 32, 32);
      }

      for (int y = 0; y < iHeight; y++)
      {
        int wT = 32 >> std::min(31, ((y << 1) >> scale));

        __m256i wt16    = _mm256_set_epi16(wT, wT, wT, wT, wT, wT, wT, wT, wT, wT, wT, wT, wT, wT, wT, wT);
        __m256i x16left = _mm256_broadcastw_epi16(_mm_loadu_si128((__m128i const *)(ptrSrc + ((y + 1) + srcStride))));
        if (wT)
        {
          for (int x = 0; x < iWidth; x += 16)
          {
            if (x == 0)
            {
              wl16 = wl16start;

              __m256i x16top = _mm256_loadu_si256((__m256i *)(ptrSrc + x + 1));   // load top
              __m256i x16dst = _mm256_loadu_si256((const __m256i *)(pDst + y * dstStride + x));   // load dst

              tmphi          = _mm256_sub_epi16(x16left, x16dst);
              tmplo          = _mm256_mullo_epi16(tmphi, wl16);   // wL * left-val
              tmphi          = _mm256_mulhi_epi16(tmphi, wl16);   // wL * left-val
              __m256i leftlo = _mm256_unpacklo_epi16(tmplo, tmphi);
              __m256i lefthi = _mm256_unpackhi_epi16(tmplo, tmphi);

              x16top        = _mm256_sub_epi16(x16top, x16dst);
              tmplo         = _mm256_mullo_epi16(x16top, wt16);   // wT*top-val
              tmphi         = _mm256_mulhi_epi16(x16top, wt16);   // wT*top-val
              __m256i toplo = _mm256_unpacklo_epi16(tmplo, tmphi);
              __m256i tophi = _mm256_unpackhi_epi16(tmplo, tmphi);

              __m256i dstlo = _mm256_add_epi32(leftlo, toplo);
              __m256i dsthi = _mm256_add_epi32(lefthi, tophi);
              dstlo         = _mm256_add_epi32(dstlo, w32);
              dsthi         = _mm256_add_epi32(dsthi, w32);

              dstlo = _mm256_srai_epi32(dstlo, 6);
              dsthi = _mm256_srai_epi32(dsthi, 6);

              dstlo = _mm256_packs_epi32(dstlo, dsthi);

              dstlo = _mm256_adds_epi16(dstlo, x16dst);
              _mm256_storeu_si256((__m256i *)(pDst + y * dstStride + x), dstlo);
            }
            else
            {
              __m256i x16top = _mm256_loadu_si256((__m256i *)(ptrSrc + x + 1));   // load top
              __m256i x16dst = _mm256_loadu_si256((const __m256i *)(pDst + y * dstStride + x));   // load dst

              x16top        = _mm256_sub_epi16(x16top, x16dst);
              tmplo         = _mm256_mullo_epi16(x16top, wt16);   // wT*top-val
              tmphi         = _mm256_mulhi_epi16(x16top, wt16);   // wT*top-val
              __m256i toplo = _mm256_unpacklo_epi16(tmplo, tmphi);
              __m256i tophi = _mm256_unpackhi_epi16(tmplo, tmphi);

              __m256i dstlo = _mm256_add_epi32(toplo, w32);
              __m256i dsthi = _mm256_add_epi32(tophi, w32);

              dstlo = _mm256_srai_epi32(dstlo, 6);
              dsthi = _mm256_srai_epi32(dsthi, 6);

              dstlo = _mm256_packs_epi32(dstlo, dsthi);

              dstlo = _mm256_adds_epi16(dstlo, x16dst);
              _mm256_storeu_si256((__m256i *)(pDst + y * dstStride + x), dstlo);
            }
          }   // for x
        }
        else
        {   // wT =0
          wl16           = wl16start;
          __m256i x16dst = _mm256_loadu_si256((const __m256i *)(pDst + y * dstStride));   // load dst
          tmphi          = _mm256_sub_epi16(x16left, x16dst);
          tmplo          = _mm256_mullo_epi16(tmphi, wl16);   // wL * left-val
          tmphi          = _mm256_mulhi_epi16(tmphi, wl16);   // wL * left-val
          __m256i leftlo = _mm256_unpacklo_epi16(tmplo, tmphi);
          __m256i lefthi = _mm256_unpackhi_epi16(tmplo, tmphi);

          __m256i dstlo = _mm256_add_epi32(leftlo, w32);
          __m256i dsthi = _mm256_add_epi32(lefthi, w32);

          dstlo = _mm256_srai_epi32(dstlo, 6);
          dsthi = _mm256_srai_epi32(dsthi, 6);

          dstlo = _mm256_packs_epi32(dstlo, dsthi);

          dstlo = _mm256_adds_epi16(dstlo, x16dst);
          _mm256_storeu_si256((__m256i *)(pDst + y * dstStride), dstlo);
        }
      }
    }
    else
#endif
    {
      __m128i       tmplo8, tmphi8;
      const __m128i const32x4 = _mm_set_epi32(32, 32, 32, 32);
      __m128i       wl8start, wl8start2;
      CHECK(scale < 0 || scale > 2, "PDPC: scale < 0 || scale > 2");

      wl8start  = _mm_set_epi16(0, 0, 0, 0, 0, 2, 8, 32);
      wl8start2 = _mm_set_epi16(0, 0, 0, 0, 0, 0, 0, 0);

      if (scale == 1)
      {
        wl8start  = _mm_set_epi16(0, 0, 1, 2, 4, 8, 16, 32);
        wl8start2 = _mm_set_epi16(0, 0, 0, 0, 0, 0, 0, 0);
      }
      else if (scale == 2)
      {
        wl8start  = _mm_set_epi16(4, 4, 8, 8, 16, 16, 32, 32);
        wl8start2 = _mm_set_epi16(0, 0, 0, 0, 1, 1, 2, 2);
      }

      __m128i wl8 = wl8start;
      for (int y = 0; y < iHeight; y++)
      {
        int     wT  = 32 >> std::min(31, ((y << 1) >> scale));
        __m128i wt8 = _mm_set_epi16(wT, wT, wT, wT, wT, wT, wT, wT);
        __m128i x8left;
        if (iWidth == 4)
        {
          x8left = _mm_loadu_si64((__m128i const *)(ptrSrc + ((y + 1) + srcStride)));
        }
        else if (iWidth == 2)
        {
          x8left = _mm_cvtsi32_si128(*(int32_t *)(ptrSrc + ((y + 1) + srcStride)));
        }
        else
        {
          x8left = _mm_loadu_si128((__m128i const *)(ptrSrc + ((y + 1) + srcStride)));
        }
        x8left = _mm_shufflelo_epi16(x8left, 0);
        x8left = _mm_shuffle_epi32(x8left, 0);
        if (wT)
        {
          for (int x = 0; x < iWidth; x += 8)
          {
            if (x > 8)
            {
              __m128i x8top;
              __m128i x8dst;

              if (iWidth == 4)
              {
                x8top = _mm_loadu_si64((__m128i *)(ptrSrc + x + 1));   // load top
                x8dst = _mm_loadu_si64((const __m128i *)(pDst + y * dstStride + x));   // load dst
              }
              else if (iWidth == 2)
              {
                x8top = _mm_cvtsi32_si128(*(int32_t *)(ptrSrc + x + 1));   // load top
                x8dst = _mm_cvtsi32_si128(*(int32_t *)(pDst + y * dstStride + x));   // load dst
              }
              else
              {
                x8top = _mm_loadu_si128((__m128i *)(ptrSrc + x + 1));   // load top
                x8dst = _mm_loadu_si128((const __m128i *)(pDst + y * dstStride + x));   // load dst
              }

              tmphi8         = _mm_sub_epi16(x8top, x8dst);
              tmplo8         = _mm_mullo_epi16(tmphi8, wt8);   // wT*top-val
              tmphi8         = _mm_mulhi_epi16(tmphi8, wt8);   // wT*top-val
              __m128i toplo8 = _mm_unpacklo_epi16(tmplo8, tmphi8);
              __m128i tophi8 = _mm_unpackhi_epi16(tmplo8, tmphi8);

              __m128i dstlo8 = _mm_add_epi32(toplo8, const32x4);
              __m128i dsthi8 = _mm_add_epi32(tophi8, const32x4);

              dstlo8 = _mm_srai_epi32(dstlo8, 6);
              dsthi8 = _mm_srai_epi32(dsthi8, 6);

              dstlo8 = _mm_packs_epi32(dstlo8, dsthi8);
              dstlo8 = _mm_adds_epi16(dstlo8, x8dst);

              _mm_storeu_si128((__m128i *)(pDst + y * dstStride + x), (dstlo8));
            }
            else   // x<=8
            {
              if (x == 0)
              {
                wl8 = wl8start;
              }
              else if (x == 8)
              {
                wl8 = wl8start2;
              }

              __m128i x8top;
              __m128i x8dst;

              if (iWidth == 4)
              {
                x8top = _mm_loadu_si64((__m128i *)(ptrSrc + x + 1));   // load top
                x8dst = _mm_loadu_si64((const __m128i *)(pDst + y * dstStride + x));   // load dst
              }
              else if (iWidth == 2)
              {
                x8top = _mm_cvtsi32_si128(*(int32_t *)(ptrSrc + x + 1));   // load top
                x8dst = _mm_cvtsi32_si128(*(int32_t *)(pDst + y * dstStride + x));   // load dst
              }
              else
              {
                x8top = _mm_loadu_si128((__m128i *)(ptrSrc + x + 1));   // load top
                x8dst = _mm_loadu_si128((const __m128i *)(pDst + y * dstStride + x));   // load dst
              }
              tmphi8          = _mm_sub_epi16(x8left, x8dst);
              tmplo8          = _mm_mullo_epi16(tmphi8, wl8);   // wL * left-val
              tmphi8          = _mm_mulhi_epi16(tmphi8, wl8);   // wL * left-val
              __m128i leftlo8 = _mm_unpacklo_epi16(tmplo8, tmphi8);
              __m128i lefthi8 = _mm_unpackhi_epi16(tmplo8, tmphi8);

              tmphi8         = _mm_sub_epi16(x8top, x8dst);
              tmplo8         = _mm_mullo_epi16(tmphi8, wt8);   // wT*top-val
              tmphi8         = _mm_mulhi_epi16(tmphi8, wt8);   // wT*top-val
              __m128i toplo8 = _mm_unpacklo_epi16(tmplo8, tmphi8);
              __m128i tophi8 = _mm_unpackhi_epi16(tmplo8, tmphi8);

              __m128i dstlo8 = _mm_add_epi32(leftlo8, toplo8);
              __m128i dsthi8 = _mm_add_epi32(lefthi8, tophi8);
              dstlo8         = _mm_add_epi32(dstlo8, const32x4);
              dsthi8         = _mm_add_epi32(dsthi8, const32x4);

              dstlo8 = _mm_srai_epi32(dstlo8, 6);
              dsthi8 = _mm_srai_epi32(dsthi8, 6);

              dstlo8 = _mm_packs_epi32(dstlo8, dsthi8);
              dstlo8 = _mm_adds_epi16(dstlo8, x8dst);

              if (iWidth >= 8)
              {
                _mm_storeu_si128((__m128i *)(pDst + y * dstStride + x), (dstlo8));
              }
              else if (iWidth == 4)
              {
                _mm_storel_epi64((__m128i *)(pDst + y * dstStride + x), (dstlo8));
              }
              else if (iWidth == 2)
              {
                *(int32_t *)(pDst + y * dstStride + x) = _mm_cvtsi128_si32(dstlo8);
              }
            }
          }
        }
        else   // wT =0
        {
          for (int x = 0; x < std::min(iWidth, 16); x += 8)
          {
            if (x == 0)
            {
              wl8 = wl8start;
            }
            else
            {
              wl8 = wl8start2;
            }

            __m128i x8dst;

            if (iWidth == 4)
            {
              x8dst = _mm_loadu_si64((const __m128i *)(pDst + y * dstStride + x));   // load dst
            }
            else if (iWidth == 2)
            {
              x8dst = _mm_cvtsi32_si128(*(int32_t *)(pDst + y * dstStride + x));   // load dst
            }
            else
            {
              x8dst = _mm_loadu_si128((const __m128i *)(pDst + y * dstStride + x));   // load dst
            }
            tmphi8          = _mm_sub_epi16(x8left, x8dst);
            tmplo8          = _mm_mullo_epi16(tmphi8, wl8);   // wL * left-val
            tmphi8          = _mm_mulhi_epi16(tmphi8, wl8);   // wL * left-val
            __m128i leftlo8 = _mm_unpacklo_epi16(tmplo8, tmphi8);
            __m128i lefthi8 = _mm_unpackhi_epi16(tmplo8, tmphi8);

            __m128i dstlo8 = _mm_add_epi32(leftlo8, const32x4);
            __m128i dsthi8 = _mm_add_epi32(lefthi8, const32x4);

            dstlo8 = _mm_srai_epi32(dstlo8, 6);
            dsthi8 = _mm_srai_epi32(dsthi8, 6);

            dstlo8 = _mm_packs_epi32(dstlo8, dsthi8);
            dstlo8 = _mm_adds_epi16(dstlo8, x8dst);

            if (iWidth >= 8)
            {
              _mm_storeu_si128((__m128i *)(pDst + y * dstStride + x), (dstlo8));
            }
            else if (iWidth == 4)
            {
              _mm_storel_epi64((__m128i *)(pDst + y * dstStride + x), (dstlo8));
            }
            else if (iWidth == 2)
            {
              *(int32_t *)(pDst + y * dstStride + x) = _mm_cvtsi128_si32(dstlo8);
            }
          }
        }
      }
    }
  }
  else   // scale ==3
  {
    for (int y = 0; y < iHeight; y++)
    {
      const int wT   = 32 >> std::min(31, ((y << 1) >> scale));
      const Pel left = Src.at(y + 1, 1);
      for (int x = 0; x < iWidth; x++)
      {
        const int wL    = 32 >> std::min(31, ((x << 1) >> scale));
        const Pel top   = Src.at(x + 1, 0);
        const Pel val   = dstBuf.at(x, y);
        dstBuf.at(x, y) = val + ((wL * (left - val) + wT * (top - val) + 32) >> 6);
      }
    }
  }
}

namespace
{

#ifdef USE_AVX2

struct DotProducts
{
  int             nSamples;
  const int16_t **input;
  int64_t        *output;
  int             outputStride;

  template<char mode>
  static inline void dotProduct(__m256i &accumulator, __m256i ax, __m256i ay, int i, __m256i extension)
  {
    if constexpr (mode != 0)
    {
      const auto product = _mm256_madd_epi16(ax, mode == 1 ? ax : ay);
      if constexpr (mode == 2)
      {
        extension = _mm256_srai_epi32(product, 32);
      }
      auto productHi = _mm256_unpackhi_epi32(product, extension);
      auto productLo = _mm256_unpacklo_epi32(product, extension);
      accumulator    = _mm256_add_epi64(accumulator, productHi);
      accumulator    = _mm256_add_epi64(accumulator, productLo);
    }
  }

  template<int mode> static inline void horizontalSum(__m256i accumulator, int64_t &sum)
  {
    if constexpr (mode != 0)
    {
      const auto sum128 =
        _mm_add_epi64(_mm256_extracti128_si256(accumulator, 0), _mm256_extracti128_si256(accumulator, 1));
      sum = _mm_extract_epi64(sum128, 0) + _mm_extract_epi64(sum128, 1);
    }
  }

  template<int mode> static inline void finishSum(int16_t ax, int16_t ay, int64_t &sum)
  {
    if constexpr (mode != 0)
    {
      if constexpr (mode == 1)
      {
        sum += ax * ax;
      }
      else
      {
        sum += ax * ay;
      }
    }
  }

  template<int mode> static inline void writeSum(int64_t &output, int64_t sum)
  {
    if constexpr (mode != 0)
    {
      output = sum;
    }
  }

  template<int modeTL, int modeTR, int modeBL, int modeBR> inline void compute_2x2(int x, int y)
  {
    auto out = &output[x + y * outputStride];
    {
      int        i    = 0;
      const int  step = sizeof(__m256i) / sizeof(int16_t);
      int64_t    sum[2][2] {};
      const auto zero              = _mm256_setzero_si256();
      __m256i    accumulator[2][2] = { { zero, zero }, { zero, zero } };
      for (; i < nSamples - (step - 1); i += step)
      {
        __m256i inputX[2] {}, inputY[2] {};

        if constexpr (modeTL || modeBL)
        {
          inputX[0] = _mm256_loadu_si256((__m256i *)&input[x + 0][i]);
        }
        if constexpr (modeTR || modeBR)
        {
          inputX[1] = _mm256_loadu_si256((__m256i *)&input[x + 1][i]);
        }
        if constexpr (modeTL || modeTR)
        {
          inputY[0] = _mm256_loadu_si256((__m256i *)&input[y + 0][i]);
        }
        if constexpr (modeBL || modeBR)
        {
          inputY[1] = _mm256_loadu_si256((__m256i *)&input[y + 1][i]);
        }

        dotProduct<modeTL>(accumulator[0][0], inputX[0], inputY[0], i, zero);
        dotProduct<modeTR>(accumulator[0][1], inputX[1], inputY[0], i, zero);
        dotProduct<modeBL>(accumulator[1][0], inputX[0], inputY[1], i, zero);
        dotProduct<modeBR>(accumulator[1][1], inputX[1], inputY[1], i, zero);
      }

      horizontalSum<modeTL>(accumulator[0][0], sum[0][0]);
      horizontalSum<modeTR>(accumulator[0][1], sum[0][1]);
      horizontalSum<modeBL>(accumulator[1][0], sum[1][0]);
      horizontalSum<modeBR>(accumulator[1][1], sum[1][1]);

      for (; i < nSamples; ++i)
      {
        int16_t inputX[2] {}, inputY[2] {};
        if constexpr (modeTL || modeBL)
        {
          inputX[0] = input[x + 0][i];
        }
        if constexpr (modeTR || modeBR)
        {
          inputX[1] = input[x + 1][i];
        }
        if constexpr (modeTL || modeTR)
        {
          inputY[0] = input[y + 0][i];
        }
        if constexpr (modeBL || modeBR)
        {
          inputY[1] = input[y + 1][i];
        }
        finishSum<modeTL>(inputX[0], inputY[0], sum[0][0]);
        finishSum<modeTR>(inputX[1], inputY[0], sum[0][1]);
        finishSum<modeBL>(inputX[0], inputY[1], sum[1][0]);
        finishSum<modeBR>(inputX[1], inputY[1], sum[1][1]);
      }

      writeSum<modeTL>(out[0 + 0 * outputStride], sum[0][0]);
      writeSum<modeTR>(out[1 + 0 * outputStride], sum[0][1]);
      writeSum<modeBL>(out[0 + 1 * outputStride], sum[1][0]);
      writeSum<modeBR>(out[1 + 1 * outputStride], sum[1][1]);
    }
  }
};
#endif

template<X86_VEXT vext> void calculateCorrelationMatrix(int nRows, int nCols, int nSamples, const int16_t *input[],
                                                        int64_t *output, int outputStride)
{
#ifdef USE_AVX2
  if (vext >= AVX2)
  {
    // We generate the top-right of the output matrix (including the diagonal)
    // Matrix will not be square: there will be extra columns.
    // example: here the number is the "mode" template parameter:
    // 0 means no computation, 2 means dot product, 1 means dot product with self (norm)
    // 1 2 2 2 2 2 2
    // 0 1 2 2 2 2 2
    // 0 0 1 2 2 2 2
    // 0 0 0 1 2 2 2
    // 0 0 0 0 1 2 2
    // To reduce memory reads, matrix is computed in 2x2 blocks. We could consider 3x3, but that might get too complex.

    DotProducts d { nSamples, input, output, outputStride };
    if (nCols % 2)
    {
      // odd number of columns, process rows two at a time
      for (int y = 0; y < (nRows & ~1); y += 2)
      {
        d.compute_2x2<0, 1, 0, 0>(y - 1, y);
        d.compute_2x2<2, 2, 1, 2>(y + 1, y);
        for (int x = y + 3; x < nCols; x += 2)
        {
          d.compute_2x2<2, 2, 2, 2>(x, y);
        }
      }
    }
    else
    {
      // even number of columns, process rows two at a time
      for (int y = 0; y < (nRows & ~1); y += 2)
      {
        d.compute_2x2<1, 2, 0, 1>(y, y);
        for (int x = y + 2; x < nCols; x += 2)
        {
          d.compute_2x2<2, 2, 2, 2>(x, y);
        }
      }
    }
    if (nRows & 1)
    {
      if (nCols % 2)
      {
        // odd number of columns, process last odd row
        d.compute_2x2<0, 1, 0, 0>(nRows - 2, nRows - 1);
        for (int x = nRows; x < nCols; x += 2)
        {
          d.compute_2x2<2, 2, 0, 0>(x, nRows - 1);
        }
      }
      else
      {
        // even number of columns, process last odd row
        d.compute_2x2<1, 2, 0, 0>(nRows - 1, nRows - 1);
        for (int x = nRows + 1; x < nCols; x += 2)
        {
          d.compute_2x2<2, 2, 0, 0>(x, nRows - 1);
        }
      }
    }
  }
  else
#endif
  {
    for (int y = 0; y < nRows; ++y)
    {
      for (int x = y; x < nCols; ++x)
      {
        int64_t sum = 0;
        for (int i = 0; i < nSamples; ++i)
        {
          sum += input[x][i] * input[y][i];
        }
        output[x + y * outputStride] = sum;
      }
    }
  }
}
}   // namespace

template<X86_VEXT vext> bool simdPredIntraOpt(PelBuf &pDst, const CodingUnit &cu, const uint32_t modeIdx,
                                              const ClpRng &clpRng, Pel *refF, Pel *refS)
{
  const uint32_t width   = pDst.width;
  const uint32_t height  = pDst.height;
  const int      sizeKey = (width << 8) + height;
  const int      sizeIdx = g_size.find(sizeKey) != g_size.end() ? g_size[sizeKey] : -1;

  if (sizeIdx < 0)
  {
    return false;
  }
  if (!cu.mipFlag && sizeIdx > 12 && modeIdx > 1 && (modeIdx % 4 != 2))
  {
    return false;
  }

  const int stride   = (int)pDst.stride;
  Pel      *pred     = pDst.buf;
  const int xShift   = g_sizeData[sizeIdx][8];
  const int yShift   = g_sizeData[sizeIdx][9];
  bool      shortLen = modeIdx < PDP_SHORT_TH[0] || (modeIdx >= PDP_SHORT_TH[1] && modeIdx <= PDP_SHORT_TH[2]);
  const int refLen   = shortLen ? g_sizeData[sizeIdx][10] : g_sizeData[sizeIdx][7];
  auto      ref      = shortLen ? refS : refF;
  auto      len4     = (((refLen + 7) / 8) * 8) * 4;
  int16_t  *filter =
    (cu.mipFlag ? (cu.mipTransposedFlag ? g_pdpFiltersMip[modeIdx + 16][sizeIdx] : g_pdpFiltersMip[modeIdx][sizeIdx])
                : g_pdpFilters[modeIdx][sizeIdx]);
  const int     addShift = 1 << 13;
  const __m128i offset   = _mm_set1_epi32(addShift);
  const __m128i max      = _mm_set1_epi32(1023);
  const __m128i zeros    = _mm_setzero_si128();
  __m128i       vmat[4];
  __m128i       vcoef[4];
  __m128i       vsrc;
  bool          isHeightLarge = (height == 32) && !cu.mipFlag;
  int           startY        = isHeightLarge ? 1 : 0;
  int           strideOffset  = isHeightLarge ? (stride << 1) : stride;
  int           offsetY       = isHeightLarge ? 2 : 1;
  if (isHeightLarge)
  {
    pred += stride;
  }

  for (int y = startY; y < height; y += offsetY, pred += strideOffset)
  {
    for (int x = 0; x < width; x += 4)
    {
      const int16_t *f0 = &filter[(y >> yShift) * width / 4 * len4 + ((x >> xShift) >> 2) * len4];

      vcoef[0] = _mm_setzero_si128();
      vcoef[1] = _mm_setzero_si128();
      vcoef[2] = _mm_setzero_si128();
      vcoef[3] = _mm_setzero_si128();
      int i;

      for (i = 0; i < refLen; i += 8)
      {
        vsrc    = _mm_loadu_si128((const __m128i *)&ref[i]);
        vmat[0] = _mm_loadu_si128((const __m128i *)f0);
        vmat[1] = _mm_loadu_si128((const __m128i *)(f0 + 8));
        vmat[2] = _mm_loadu_si128((const __m128i *)(f0 + 16));
        vmat[3] = _mm_loadu_si128((const __m128i *)(f0 + 24));

        vcoef[0] = _mm_add_epi32(_mm_madd_epi16(vsrc, vmat[0]), vcoef[0]);
        vcoef[1] = _mm_add_epi32(_mm_madd_epi16(vsrc, vmat[1]), vcoef[1]);
        vcoef[2] = _mm_add_epi32(_mm_madd_epi16(vsrc, vmat[2]), vcoef[2]);
        vcoef[3] = _mm_add_epi32(_mm_madd_epi16(vsrc, vmat[3]), vcoef[3]);

        f0 += 32;
      }
      vcoef[0] = _mm_hadd_epi32(vcoef[0], vcoef[1]);
      vcoef[2] = _mm_hadd_epi32(vcoef[2], vcoef[3]);

      vcoef[0] = _mm_hadd_epi32(vcoef[0], vcoef[2]);
      vcoef[0] = _mm_srai_epi32(_mm_add_epi32(vcoef[0], offset), 14);
      vcoef[0] = _mm_min_epi32(vcoef[0], max);
      vcoef[0] = _mm_max_epi32(vcoef[0], zeros);

      *((int64_t *)&pred[x]) = _mm_cvtsi128_si64(_mm_packs_epi32(vcoef[0], vcoef[0]));
    }
  }

  if (sizeIdx > 12 && !cu.mipFlag)
  {
    int  numMRLLeft = g_sizeData[sizeIdx][5];
    int  numMRLTop  = g_sizeData[sizeIdx][6];
    int  sampFacHor = pDst.width / 16;
    int  sampFacVer = pDst.height / 16;
    int  strideDst  = (pDst.width << 1) + numMRLLeft;   // fetching from g_ref always.
    Pel *ptrSrc     = refF + (strideDst * numMRLTop) + numMRLLeft - 1;
    Pel *ref2       = refF + (strideDst * (numMRLTop - 1)) + numMRLLeft;
    pred            = pDst.buf;
    for (int y = 0; y < pDst.height; y++)
    {
      if (0 != (sampFacHor - 1) && ((y & (sampFacVer - 1)) == (sampFacVer - 1)))
      {
        pred[0] = (ptrSrc[0] + pred[1] + 1) >> 1;
      }
      for (int x = 1; x < pDst.width; x++)
      {
        if (((x & (sampFacHor - 1)) != (sampFacHor - 1)) && ((y & (sampFacVer - 1)) == (sampFacVer - 1)))
        {
          pred[x] = (pred[x - 1] + pred[x + 1] + 1) >> 1;
        }
      }
      pred += stride;
      ptrSrc += numMRLLeft;
    }

    pred       = pDst.buf;
    Pel *pred1 = pred + stride;
    if (0 != (sampFacVer - 1))
    {
      for (int x = 0; x < pDst.width; x++)
      {
        pred[x] = (ref2[x] + pred1[x] + 1) >> 1;
      }
    }

    pred += stride;
    Pel *pred0 = pred - stride;
    pred1      = pred + stride;

    for (int y = 1; y < pDst.height; y++)
    {
      if (((y & (sampFacVer - 1)) != (sampFacVer - 1)))
      {
        for (int x = 0; x < pDst.width; x++)
        {
          pred[x] = (pred0[x] + pred1[x] + 1) >> 1;
        }
      }

      pred += stride;
      pred0 += stride;
      pred1 += stride;
    }
  }

  return true;
}

template<X86_VEXT vext> void IntraPrediction::_initIntraPredictionX86()
{
  m_IntraPredAngleCpy          = simdIntraPredAngleCpy<vext>;
  m_IntraPredAngleChroma       = simdIntraPredAngleChroma<vext>;
  m_IntraPredAngleLuma         = simdIntraPredAngleLuma<vext>;
  m_IntraAnglePDPC             = simdIntraAnglePDPC<vext>;
  m_IntraAngleGradPDPC         = simdIntraAngleGradPDPC<vext>;
  m_IntraHorVerPDPC            = simdIntraHorVerPDPC<vext>;
  m_PredIntraPlanar            = simdPredIntraPlanar<vext>;
  m_IntraPredSampleFilter      = simdIntraPredSampleFilter<vext>;
  m_calculateCorrelationMatrix = calculateCorrelationMatrix<vext>;
  m_xPredIntraOpt              = simdPredIntraOpt<vext>;
}

template void IntraPrediction::_initIntraPredictionX86<SIMDX86>();
#endif
#endif   // #ifdef TARGET_SIMD_X86
//! \}
