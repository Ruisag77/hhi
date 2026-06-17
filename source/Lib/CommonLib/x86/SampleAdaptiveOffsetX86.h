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
 * \brief Implementation of SampleAdaptiveOffset class
 */
// ====================================================================================================================
// Includes
// ====================================================================================================================

#include "CommonDefX86.h"
#include "CommonLib/SampleAdaptiveOffset.h"
#include "CommonLib/InterpolationFilter.h"

//! \ingroup CommonLib
//! \{

#ifdef TARGET_SIMD_X86

#if ENABLE_SIMD_OPT_CCSAO
template<X86_VEXT vext> void getCcSaomSampleStats_SIMD(int startx, int endx, const int bitDepth, int64_t *diff,
                                                       uint32_t *count, const int chromaScaleX, const uint16_t bandNumY,
                                                       const uint16_t bandNumU, const uint16_t bandNumV,
                                                       const Pel *srcY, const Pel *srcU, const Pel *srcV,
                                                       const Pel *org, const Pel *dst)
{
  if ((endx - startx) == 0)
  {
    return;
  }
#ifdef USE_AVX2
  if (((endx - startx) % 16) == 0)
  {
    if (bandNumY > 16)
    {
      THROW("bandNumY to large \n");
    }
    if (chromaScaleX != 1)
    {
      THROW("Not a supported chromaScaleX\n");
    }
    if (chromaScaleX != 1)
    {
      THROW("Not a supported chromaScaleY\n");
    }

    const Pel *pY   = srcY;
    const Pel *pU   = srcU;
    const Pel *pV   = srcV;
    const Pel *pOrg = org;
    const Pel *pDst = dst;

    __m256i shflmask   = _mm256_set_epi8(0xf, 0xe, 0xf, 0xe, 0xd, 0xc, 0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8,
                                         0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0, 0x1, 0x0);
    __m256i VbandNumY  = _mm256_set1_epi16(bandNumY);
    __m256i VbandNumU  = _mm256_set1_epi16(bandNumU);
    __m256i VbandNumV  = _mm256_set1_epi16(bandNumV);
    __m256i VbandNumUV = _mm256_set1_epi16(bandNumU * bandNumV);
    __m128i VBdph      = _mm_set1_epi64x((uint64_t)bitDepth);

    for (int x = startx; x < endx; x += 16)
    {
      __m256i vOrg  = _mm256_lddqu_si256((__m256i *)&pOrg[x]);
      __m256i vdst  = _mm256_lddqu_si256((__m256i *)&pDst[x]);
      __m256i vcolY = _mm256_lddqu_si256((__m256i *)&pY[x]);
      __m256i vcolU = _mm256_castsi128_si256(_mm_lddqu_si128((__m128i *)&pU[x >> chromaScaleX]));
      __m256i vcolV = _mm256_castsi128_si256(_mm_lddqu_si128((__m128i *)&pV[x >> chromaScaleX]));

      vdst = _mm256_subs_epi16(vOrg, vdst);
      __m256i tmphi;
      __m256i tmplo;
      __m256i vBandYlo;
      __m256i vBandUlo;
      __m256i vBandUhi;
      __m256i vBandVlo;

      // Y
      if (bandNumY == 16)
      {
        vBandYlo = _mm256_sra_epi16(vcolY, _mm_set1_epi64x((uint64_t)bitDepth - 4));
      }
      else
      {
        tmphi            = _mm256_mulhi_epi16(vcolY, VbandNumY);
        tmplo            = _mm256_mullo_epi16(vcolY, VbandNumY);
        vBandYlo         = _mm256_unpacklo_epi16(tmplo, tmphi);  // low
        __m256i vBandYhi = _mm256_unpackhi_epi16(tmplo, tmphi);  // high
        vBandYlo         = _mm256_sra_epi32(vBandYlo, VBdph);
        vBandYhi         = _mm256_sra_epi32(vBandYhi, VBdph);
        vBandYlo         = _mm256_packs_epi32(vBandYlo, vBandYhi);
      }
      // U
      vcolU = _mm256_permute4x64_epi64(vcolU, 0x50);
      vcolU = _mm256_shuffle_epi8(vcolU, shflmask);

      if (bandNumU == 2)
      {
        vBandUlo = _mm256_sra_epi16(vcolU, _mm_set1_epi64x((uint64_t)bitDepth - 1));
      }
      else if (bandNumU == 1)
      {
        vBandUlo = _mm256_sra_epi16(vcolU, _mm_set1_epi64x((uint64_t)bitDepth));
      }
      else
      {
        tmphi    = _mm256_mulhi_epi16(vcolU, VbandNumU);
        tmplo    = _mm256_mullo_epi16(vcolU, VbandNumU);
        vBandUlo = _mm256_unpacklo_epi16(tmplo, tmphi);  // low
        vBandUhi = _mm256_unpackhi_epi16(tmplo, tmphi);  // high
        vBandUlo = _mm256_sra_epi32(vBandUlo, VBdph);
        vBandUhi = _mm256_sra_epi32(vBandUhi, VBdph);
        vBandUlo = _mm256_packs_epi32(vBandUlo, vBandUhi);
      }
      // V
      vcolV = _mm256_permute4x64_epi64(vcolV, 0x50);
      vcolV = _mm256_shuffle_epi8(vcolV, shflmask);

      if (bandNumV == 2)
      {
        vBandVlo = _mm256_sra_epi16(vcolV, _mm_set1_epi64x((uint64_t)bitDepth - 1));
        vBandUlo = _mm256_slli_epi16(vBandUlo, 1);
      }
      else if (bandNumV == 1)
      {
        vBandVlo = _mm256_sra_epi16(vcolV, _mm_set1_epi64x((uint64_t)bitDepth));
      }
      else
      {
        tmphi            = _mm256_mulhi_epi16(vcolV, VbandNumV);
        tmplo            = _mm256_mullo_epi16(vcolV, VbandNumV);
        vBandVlo         = _mm256_unpacklo_epi16(tmplo, tmphi);  // low
        __m256i vBandVhi = _mm256_unpackhi_epi16(tmplo, tmphi);  // high
        vBandVlo         = _mm256_sra_epi32(vBandVlo, VBdph);
        vBandVhi         = _mm256_sra_epi32(vBandVhi, VBdph);
        vBandVlo         = _mm256_packs_epi32(vBandVlo, vBandVhi);

        //  bandU * bandNumV
        tmphi    = _mm256_mulhi_epi16(vBandUlo, VbandNumV);
        tmplo    = _mm256_mullo_epi16(vBandUlo, VbandNumV);
        vBandUlo = _mm256_unpacklo_epi16(tmplo, tmphi);  // low
        vBandUhi = _mm256_unpackhi_epi16(tmplo, tmphi);  // high
        vBandUlo = _mm256_packs_epi32(vBandUlo, vBandUhi);
      }
      // bandY * bandNumUV
      if ((bandNumU * bandNumV) == 2)
      {
        vBandYlo = _mm256_slli_epi16(vBandYlo, 1);
      }
      else if ((bandNumU * bandNumV) > 1)
      {
        tmphi            = _mm256_mulhi_epi16(vBandYlo, VbandNumUV);
        tmplo            = _mm256_mullo_epi16(vBandYlo, VbandNumUV);
        vBandYlo         = _mm256_unpacklo_epi16(tmplo, tmphi);  // low
        __m256i vBandYhi = _mm256_unpackhi_epi16(tmplo, tmphi);  // high
        vBandYlo         = _mm256_packs_epi32(vBandYlo, vBandYhi);
      }
      // sum up
      vBandYlo = _mm256_adds_epi16(vBandYlo, vBandUlo);
      vBandYlo = _mm256_adds_epi16(vBandYlo, vBandVlo);

      __m256i test      = _mm256_shuffle_epi8(vBandYlo, _mm256_set1_epi16(0x0100));
      test              = _mm256_cmpeq_epi16(vBandYlo, test);
      int     equal_low = _mm_test_all_ones(_mm256_extracti128_si256(test, 0));
      __m128i vidx      = _mm256_extracti128_si256(vBandYlo, 0);
      __m128i vdst_low  = _mm256_extracti128_si256(vdst, 0);
      if (equal_low)
      {
        int16_t idx = _mm_extract_epi16(vidx, 0);
        vdst_low    = _mm_hadds_epi16(vdst_low, vdst_low);
        vdst_low    = _mm_hadds_epi16(vdst_low, vdst_low);
        vdst_low    = _mm_hadds_epi16(vdst_low, vdst_low);
        int16_t sum = _mm_extract_epi16(vdst_low, 0);
        diff[idx] += sum;
        count[idx] += 8;
      }
      else
      {
        int16_t idx[8];
        int16_t sum[8];
        _mm_storeu_si128((__m128i *)&idx, vidx);
        _mm_storeu_si128((__m128i *)&sum, vdst_low);
        for (int n = 0; n < 8; n++)
        {
          diff[idx[n]] += sum[n];
          count[idx[n]]++;
        }
      }

      vidx               = _mm256_extracti128_si256(vBandYlo, 1);
      __m128i vdst_high  = _mm256_extracti128_si256(vdst, 1);
      int     equal_high = _mm_test_all_ones(_mm256_extracti128_si256(test, 1));
      if (equal_high)
      {
        int16_t idx = _mm_extract_epi16(vidx, 0);
        vdst_high   = _mm_hadds_epi16(vdst_high, vdst_high);
        vdst_high   = _mm_hadds_epi16(vdst_high, vdst_high);
        vdst_high   = _mm_hadds_epi16(vdst_high, vdst_high);
        int16_t sum = _mm_extract_epi16(vdst_high, 0);
        diff[idx] += sum;
        count[idx] += 8;
      }
      else
      {
        int16_t idx[8];
        int16_t sum[8];
        _mm_storeu_si128((__m128i *)&idx, vidx);
        _mm_storeu_si128((__m128i *)&sum, vdst_high);
        for (int n = 0; n < 8; n++)
        {
          diff[idx[n]] += sum[n];
          count[idx[n]]++;
        }
      }
    }
  }
  else
#endif
  {
    if (bandNumY > 16)
    {
      THROW("bandNumY to large \n");
    }
    if (chromaScaleX != 1)
    {
      THROW("Not a supported chromaScaleX\n");
    }
    if (chromaScaleX != 1)
    {
      THROW("Not a supported chromaScaleY\n");
    }

    const Pel *pY   = srcY;
    const Pel *pU   = srcU;
    const Pel *pV   = srcV;
    const Pel *pOrg = org;
    const Pel *pDst = dst;

    __m128i shflmask;
    if (startx % 2) // unequal
    {
      shflmask = _mm_set_epi8(0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0);
    }
    else
    {
      shflmask = _mm_set_epi8(0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0, 0x1, 0x0);
    }
    __m128i VbandNumY  = _mm_set1_epi16(bandNumY);
    __m128i VbandNumU  = _mm_set1_epi16(bandNumU);
    __m128i VbandNumV  = _mm_set1_epi16(bandNumV);
    __m128i VbandNumUV = _mm_set1_epi16(bandNumU * bandNumV);
    __m128i VBdph      = _mm_set1_epi64x((uint64_t)bitDepth);

    int x = startx;
    while ((endx - x) >= 8)
    {
      __m128i vOrg  = _mm_lddqu_si128((__m128i *)&pOrg[x]);
      __m128i vdst  = _mm_lddqu_si128((__m128i *)&pDst[x]);
      __m128i vcolY = _mm_lddqu_si128((__m128i *)&pY[x]);
      __m128i vcolU = _mm_lddqu_si128((__m128i *)&pU[x >> chromaScaleX]);
      __m128i vcolV = _mm_lddqu_si128((__m128i *)&pV[x >> chromaScaleX]);

      vdst = _mm_subs_epi16(vOrg, vdst);
      __m128i tmphi;
      __m128i tmplo;
      __m128i vBandYlo;
      __m128i vBandUlo;
      __m128i vBandUhi;
      __m128i vBandVlo;

      // Y
      if (bandNumY == 16)
      {
        vBandYlo = _mm_sra_epi16(vcolY, _mm_set1_epi64x((uint64_t)bitDepth - 4));
      }
      else
      {
        tmphi            = _mm_mulhi_epi16(vcolY, VbandNumY);
        tmplo            = _mm_mullo_epi16(vcolY, VbandNumY);
        vBandYlo         = _mm_unpacklo_epi16(tmplo, tmphi);  // low
        __m128i vBandYhi = _mm_unpackhi_epi16(tmplo, tmphi);  // high
        vBandYlo         = _mm_sra_epi32(vBandYlo, VBdph);
        vBandYhi         = _mm_sra_epi32(vBandYhi, VBdph);
        vBandYlo         = _mm_packs_epi32(vBandYlo, vBandYhi);
      }
    // U
      vcolU = _mm_shuffle_epi8(vcolU, shflmask);
      if (bandNumU == 2)
      {
        vBandUlo = _mm_sra_epi16(vcolU, _mm_set1_epi64x((uint64_t)bitDepth - 1));
      }
      else if (bandNumU == 1)
      {
        vBandUlo = _mm_sra_epi16(vcolU, _mm_set1_epi64x((uint64_t)bitDepth));
      }
      else
      {
        tmphi    = _mm_mulhi_epi16(vcolU, VbandNumU);
        tmplo    = _mm_mullo_epi16(vcolU, VbandNumU);
        vBandUlo = _mm_unpacklo_epi16(tmplo, tmphi);  // low
        vBandUhi = _mm_unpackhi_epi16(tmplo, tmphi);  // high
        vBandUlo = _mm_sra_epi32(vBandUlo, VBdph);
        vBandUhi = _mm_sra_epi32(vBandUhi, VBdph);
        vBandUlo = _mm_packs_epi32(vBandUlo, vBandUhi);
      }
   // V
      vcolV = _mm_shuffle_epi8(vcolV, shflmask);
      if (bandNumV == 2)
      {
        vBandVlo = _mm_sra_epi16(vcolV, _mm_set1_epi64x((uint64_t)bitDepth - 1));
        vBandUlo = _mm_slli_epi16(vBandUlo, 1);
      }
      else if (bandNumV == 1)
      {
        vBandVlo = _mm_sra_epi16(vcolV, _mm_set1_epi64x((uint64_t)bitDepth));
      }
      else
      {
        tmphi            = _mm_mulhi_epi16(vcolV, VbandNumV);
        tmplo            = _mm_mullo_epi16(vcolV, VbandNumV);
        vBandVlo         = _mm_unpacklo_epi16(tmplo, tmphi);  // low
        __m128i vBandVhi = _mm_unpackhi_epi16(tmplo, tmphi);  // high
        vBandVlo         = _mm_sra_epi32(vBandVlo, VBdph);
        vBandVhi         = _mm_sra_epi32(vBandVhi, VBdph);
        vBandVlo         = _mm_packs_epi32(vBandVlo, vBandVhi);

        //  bandU * bandNumV
        tmphi    = _mm_mulhi_epi16(vBandUlo, VbandNumV);
        tmplo    = _mm_mullo_epi16(vBandUlo, VbandNumV);
        vBandUlo = _mm_unpacklo_epi16(tmplo, tmphi);  // low
        vBandUhi = _mm_unpackhi_epi16(tmplo, tmphi);  // high
        vBandUlo = _mm_packs_epi32(vBandUlo, vBandUhi);
      }
      // bandY * bandNumUV
      if ((bandNumU * bandNumV) == 2)
      {
        vBandYlo = _mm_slli_epi16(vBandYlo, 1);
      }
      else if ((bandNumU * bandNumV) > 1)
      {
        tmphi            = _mm_mulhi_epi16(vBandYlo, VbandNumUV);
        tmplo            = _mm_mullo_epi16(vBandYlo, VbandNumUV);
        vBandYlo         = _mm_unpacklo_epi16(tmplo, tmphi);  // low
        __m128i vBandYhi = _mm_unpackhi_epi16(tmplo, tmphi);  // high
        vBandYlo         = _mm_packs_epi32(vBandYlo, vBandYhi);
      }
      // sum up
      vBandYlo = _mm_adds_epi16(vBandYlo, vBandUlo);
      vBandYlo = _mm_adds_epi16(vBandYlo, vBandVlo);

      __m128i test = _mm_shuffle_epi8(vBandYlo, _mm_set_epi8(1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0));
      test         = _mm_cmpeq_epi16(vBandYlo, test);
      int equal    = _mm_test_all_ones(test);
      if (equal)
      {
        int16_t idx = _mm_extract_epi16(vBandYlo, 0);
        vdst        = _mm_hadds_epi16(vdst, vdst);
        vdst        = _mm_hadds_epi16(vdst, vdst);
        vdst        = _mm_hadds_epi16(vdst, vdst);
        int16_t sum = _mm_extract_epi16(vdst, 0);
        diff[idx] += sum;
        count[idx] += 8;
      }
      else
      {
        int16_t idx[8];
        int16_t sum[8];
        _mm_storeu_si128((__m128i *)&idx, vBandYlo);
        _mm_storeu_si128((__m128i *)&sum, vdst);
        for (int n = 0; n < 8; n++)
        {
          diff[idx[n]] += sum[n];
          count[idx[n]]++;
        }
      }
      x += 8;
    }
    for (; x < endx; x++)
    {
      const Pel *colY = srcY + x;
      const Pel *colU = srcU + (x >> chromaScaleX);
      const Pel *colV = srcV + (x >> chromaScaleX);

      const int bandY    = (*colY * bandNumY) >> bitDepth;
      const int bandU    = (*colU * bandNumU) >> bitDepth;
      const int bandV    = (*colV * bandNumV) >> bitDepth;
      const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
      const int classIdx = bandIdx;

      diff[classIdx] += org[x] - dst[x];
      count[classIdx]++;
    }
  }
}

// get strange warning if compiled for ARM
// error: loop not vectorized: the optimizer was unable to perform the requested transformation; the transformation
// might be disabled or specified as part of an unsupported transformation ordering
// [-Werror,-Wpass-failed=transform-warning]
//  to avoid this, disable AVX2 in this case
#ifdef REAL_TARGET_ARM
#undef USE_AVX2
#endif
template<X86_VEXT vext> void getCcSaomChromaSampleStats_SIMD(int startx, int endx, const int bitDepth, int64_t *diff,
                                                             uint32_t *count, const int chromaScaleX,
                                                             const uint16_t bandNumY, const uint16_t bandNumU,
                                                             const uint16_t bandNumV, const Pel *srcY, const Pel *srcU,
                                                             const Pel *srcV, const Pel *org, const Pel *dst)
{
  if ((endx - startx) == 0)
  {
    return;
  }
#ifdef USE_AVX2
  if (((endx - startx) % 16) == 0)
  {
    if (bandNumY > 16)
    {
      THROW("bandNumY to large \n");
    }
    if (bandNumU > 4)
    {
      THROW("bandNumU to large \n");
    }
    if (bandNumV > 4)
    {
      THROW("bandNumV to large \n");
    }
    if (chromaScaleX != 1)
    {
      THROW("Not a supported chromaScaleX\n");
    }

    const Pel *pY   = srcY;
    const Pel *pU   = srcU;
    const Pel *pV   = srcV;
    const Pel *pOrg = org;
    const Pel *pDst = dst;

    __m256i VbandNumY  = _mm256_set1_epi16(bandNumY);
    __m256i VbandNumU  = _mm256_set1_epi16(bandNumU);
    __m256i VbandNumV  = _mm256_set1_epi16(bandNumV);
    __m256i VbandNumUV = _mm256_set1_epi16(bandNumU * bandNumV);
    __m128i VBdph      = _mm_set1_epi64x((uint64_t)bitDepth);
    for (int x = startx; x < endx; x += 16)
    {
      __m256i vOrg  = _mm256_lddqu_si256((__m256i *)&pOrg[x]);
      __m256i vdst  = _mm256_lddqu_si256((__m256i *)&pDst[x]);
      // jeden 2. y pixel
      __m256i vcolY = _mm256_lddqu_si256((__m256i *)&pY[x << 1]);
      __m256i Ytmp  = _mm256_shufflelo_epi16(vcolY, 0x8);
      __m256i Ytmp2 = _mm256_bsrli_epi128(vcolY, 8);
      Ytmp2         = _mm256_shufflelo_epi16(Ytmp2, 0x88);
      __m256i Ytmp3 = _mm256_blend_epi32(Ytmp, Ytmp2, 0xaa);
      vcolY         = _mm256_permute4x64_epi64(Ytmp3, 8);

      __m256i vcolY2 = _mm256_lddqu_si256((__m256i *)&pY[(x << 1) + 16]);
      Ytmp           = _mm256_shufflelo_epi16(vcolY2, 0x8);
      Ytmp2          = _mm256_bsrli_epi128(vcolY2, 8);
      Ytmp2          = _mm256_shufflelo_epi16(Ytmp2, 0x88);
      Ytmp3          = _mm256_blend_epi32(Ytmp, Ytmp2, 0xaa);
      vcolY2         = _mm256_permute4x64_epi64(Ytmp3, 8);

      vcolY = _mm256_permute2x128_si256(vcolY, vcolY2, 0x20);

      __m256i vcolU = _mm256_lddqu_si256((__m256i *)&pU[x]);
      __m256i vcolV = _mm256_lddqu_si256((__m256i *)&pV[x]);

      vdst = _mm256_subs_epi16(vOrg, vdst);
      __m256i tmphi;
      __m256i tmplo;
      __m256i vBandYlo;
      __m256i vBandUlo;
      __m256i vBandUhi;
      __m256i vBandVlo;
      // Y
      if (bandNumY == 16)
      {
        vBandYlo = _mm256_sra_epi16(vcolY, _mm_set1_epi64x((uint64_t)bitDepth - 4));
      }
      else
      {
        tmphi            = _mm256_mulhi_epi16(vcolY, VbandNumY);
        tmplo            = _mm256_mullo_epi16(vcolY, VbandNumY);
        vBandYlo         = _mm256_unpacklo_epi16(tmplo, tmphi);  // low
        __m256i vBandYhi = _mm256_unpackhi_epi16(tmplo, tmphi);  // high
        vBandYlo         = _mm256_sra_epi32(vBandYlo, VBdph);
        vBandYhi         = _mm256_sra_epi32(vBandYhi, VBdph);
        vBandYlo         = _mm256_packs_epi32(vBandYlo, vBandYhi);
      }
      // U
      if (bandNumU == 2)
      {
        vBandUlo = _mm256_sra_epi16(vcolU, _mm_set1_epi64x((uint64_t)bitDepth - 1));
      }
      else if (bandNumU == 1)
      {
        vBandUlo = _mm256_sra_epi16(vcolU, _mm_set1_epi64x((uint64_t)bitDepth));
      }
      else
      {
        tmphi    = _mm256_mulhi_epi16(vcolU, VbandNumU);
        tmplo    = _mm256_mullo_epi16(vcolU, VbandNumU);
        vBandUlo = _mm256_unpacklo_epi16(tmplo, tmphi);  // low
        vBandUhi = _mm256_unpackhi_epi16(tmplo, tmphi);  // high
        vBandUlo = _mm256_sra_epi32(vBandUlo, VBdph);
        vBandUhi = _mm256_sra_epi32(vBandUhi, VBdph);
        vBandUlo = _mm256_packs_epi32(vBandUlo, vBandUhi);
      }
      // V
      if (bandNumV == 2)
      {
        vBandVlo = _mm256_sra_epi16(vcolV, _mm_set1_epi64x((uint64_t)bitDepth - 1));
        vBandUlo = _mm256_slli_epi16(vBandUlo, 1);
      }
      else if (bandNumV == 1)
      {
        vBandVlo = _mm256_sra_epi16(vcolV, _mm_set1_epi64x((uint64_t)bitDepth));
      }
      else
      {
        tmphi            = _mm256_mulhi_epi16(vcolV, VbandNumV);
        tmplo            = _mm256_mullo_epi16(vcolV, VbandNumV);
        vBandVlo         = _mm256_unpacklo_epi16(tmplo, tmphi);  // low
        __m256i vBandVhi = _mm256_unpackhi_epi16(tmplo, tmphi);  // high
        vBandVlo         = _mm256_sra_epi32(vBandVlo, VBdph);
        vBandVhi         = _mm256_sra_epi32(vBandVhi, VBdph);
        vBandVlo         = _mm256_packs_epi32(vBandVlo, vBandVhi);

        //  bandU * bandNumV
        tmphi    = _mm256_mulhi_epi16(vBandUlo, VbandNumV);
        tmplo    = _mm256_mullo_epi16(vBandUlo, VbandNumV);
        vBandUlo = _mm256_unpacklo_epi16(tmplo, tmphi);  // low
        vBandUhi = _mm256_unpackhi_epi16(tmplo, tmphi);  // high
        vBandUlo = _mm256_packs_epi32(vBandUlo, vBandUhi);
      }
      // bandY * bandNumUV
      if ((bandNumU * bandNumV) == 2)
      {
        vBandYlo = _mm256_slli_epi16(vBandYlo, 1);
      }
      else if ((bandNumU * bandNumV) > 1)
      {
        tmphi            = _mm256_mulhi_epi16(vBandYlo, VbandNumUV);
        tmplo            = _mm256_mullo_epi16(vBandYlo, VbandNumUV);
        vBandYlo         = _mm256_unpacklo_epi16(tmplo, tmphi);  // low
        __m256i vBandYhi = _mm256_unpackhi_epi16(tmplo, tmphi);  // high
        vBandYlo         = _mm256_packs_epi32(vBandYlo, vBandYhi);
      }
      // sum up
      vBandYlo = _mm256_adds_epi16(vBandYlo, vBandUlo);
      vBandYlo = _mm256_adds_epi16(vBandYlo, vBandVlo);

      __m256i test  = _mm256_shuffle_epi8(vBandYlo, _mm256_set1_epi16(0x0100));
      test          = _mm256_cmpeq_epi16(vBandYlo, test);
      int equal_low = _mm_test_all_ones(_mm256_extracti128_si256(test, 0));

      __m128i vidx     = _mm256_extracti128_si256(vBandYlo, 0);
      __m128i vdst_low = _mm256_extracti128_si256(vdst, 0);
      if (equal_low)
      {
        int16_t idx = _mm_extract_epi16(vidx, 0);
        vdst_low    = _mm_hadds_epi16(vdst_low, vdst_low);
        vdst_low    = _mm_hadds_epi16(vdst_low, vdst_low);
        vdst_low    = _mm_hadds_epi16(vdst_low, vdst_low);
        int16_t sum = _mm_extract_epi16(vdst_low, 0);
        diff[idx] += sum;
        count[idx] += 8;
      }
      else
      {
        int16_t idx[8];
        int16_t sum[8];
        _mm_storeu_si128((__m128i *)&idx, vidx);
        _mm_storeu_si128((__m128i *)&sum, vdst_low);
        for (int n = 0; n < 8; n++)
        {
          diff[idx[n]] += sum[n];
          count[idx[n]]++;
        }
      }

      vidx              = _mm256_extracti128_si256(vBandYlo, 1);
      __m128i vdst_high = _mm256_extracti128_si256(vdst, 1);

      int equal_high = _mm_test_all_ones(_mm256_extracti128_si256(test, 1));
      if (equal_high)
      {
        int16_t idx = _mm_extract_epi16(vidx, 0);
        vdst_high   = _mm_hadds_epi16(vdst_high, vdst_high);
        vdst_high   = _mm_hadds_epi16(vdst_high, vdst_high);
        vdst_high   = _mm_hadds_epi16(vdst_high, vdst_high);
        int16_t sum = _mm_extract_epi16(vdst_high, 0);
        diff[idx] += sum;
        count[idx] += 8;
      }
      else
      {
        int16_t idx[8];
        int16_t sum[8];
        _mm_storeu_si128((__m128i *)&idx, vidx);
        _mm_storeu_si128((__m128i *)&sum, vdst_high);
        for (int n = 0; n < 8; n++)
        {
          diff[idx[n]] += sum[n];
          count[idx[n]]++;
        }
      }
    }
  }
  else
#endif
  {
    if (bandNumY > 16)
    {
      THROW("bandNumY to large \n");
    }
    if (bandNumU > 4)
    {
      THROW("bandNumU to large \n");
    }
    if (bandNumV > 4)
    {
      THROW("bandNumV to large \n");
    }
    if (chromaScaleX != 1)
    {
      THROW("Not a supported chromaScaleX\n");
    }
    const Pel *pY   = srcY;
    const Pel *pU   = srcU;
    const Pel *pV   = srcV;
    const Pel *pOrg = org;
    const Pel *pDst = dst;

    __m128i VbandNumY  = _mm_set1_epi16(bandNumY);
    __m128i VbandNumU  = _mm_set1_epi16(bandNumU);
    __m128i VbandNumV  = _mm_set1_epi16(bandNumV);
    __m128i VbandNumUV = _mm_set1_epi16(bandNumU * bandNumV);
    __m128i VBdph      = _mm_set1_epi64x((uint64_t)bitDepth);

    int x = startx;
    while ((endx - x) >= 8)
    {
      __m128i vOrg   = _mm_lddqu_si128((__m128i *)&pOrg[x]);
      __m128i vdst   = _mm_lddqu_si128((__m128i *)&pDst[x]);
      // jeden 2. y pixel
      __m128i vcolY  = _mm_lddqu_si128((__m128i *)&pY[x << 1]);
      __m128i Ytmp   = _mm_shufflelo_epi16(vcolY, 0x8);
      __m128i Ytmp2  = _mm_bsrli_si128(vcolY, 8);
      Ytmp2          = _mm_shufflelo_epi16(Ytmp2, 0x88);
      vcolY          = _mm_blend_epi16(Ytmp, Ytmp2, 0x0c);
      __m128i vcolY2 = _mm_lddqu_si128((__m128i *)&pY[(x << 1) + 8]);
      Ytmp           = _mm_shufflelo_epi16(vcolY2, 0x8);
      Ytmp2          = _mm_bsrli_si128(vcolY2, 8);
      Ytmp2          = _mm_shufflelo_epi16(Ytmp2, 0x88);
      __m128i Ytmp3  = _mm_blend_epi16(Ytmp, Ytmp2, 0x0c);
      Ytmp3          = _mm_bslli_si128(Ytmp3, 8);
      vcolY          = _mm_blend_epi16(vcolY, Ytmp3, 0xf0);

      __m128i vcolU = _mm_lddqu_si128((__m128i *)&pU[x]);
      __m128i vcolV = _mm_lddqu_si128((__m128i *)&pV[x]);

      vdst = _mm_subs_epi16(vOrg, vdst);
      __m128i tmphi;
      __m128i tmplo;
      __m128i vBandYlo;
      __m128i vBandUlo;
      __m128i vBandUhi;
      __m128i vBandVlo;
      // Y
      if (bandNumY == 16)
      {
        vBandYlo = _mm_sra_epi16(vcolY, _mm_set1_epi64x((uint64_t)bitDepth - 4));
      }
      else
      {
        tmphi            = _mm_mulhi_epi16(vcolY, VbandNumY);
        tmplo            = _mm_mullo_epi16(vcolY, VbandNumY);
        vBandYlo         = _mm_unpacklo_epi16(tmplo, tmphi);  // low
        __m128i vBandYhi = _mm_unpackhi_epi16(tmplo, tmphi);  // high
        vBandYlo         = _mm_sra_epi32(vBandYlo, VBdph);
        vBandYhi         = _mm_sra_epi32(vBandYhi, VBdph);
        vBandYlo         = _mm_packs_epi32(vBandYlo, vBandYhi);
      }
      // U
      if (bandNumU == 2)
      {
        vBandUlo = _mm_sra_epi16(vcolU, _mm_set1_epi64x((uint64_t)bitDepth - 1));
      }
      else if (bandNumU == 1)
      {
        vBandUlo = _mm_sra_epi16(vcolU, _mm_set1_epi64x((uint64_t)bitDepth));
      }
      else
      {
        tmphi    = _mm_mulhi_epi16(vcolU, VbandNumU);
        tmplo    = _mm_mullo_epi16(vcolU, VbandNumU);
        vBandUlo = _mm_unpacklo_epi16(tmplo, tmphi);  // low
        vBandUhi = _mm_unpackhi_epi16(tmplo, tmphi);  // high
        vBandUlo = _mm_sra_epi32(vBandUlo, VBdph);
        vBandUhi = _mm_sra_epi32(vBandUhi, VBdph);
        vBandUlo = _mm_packs_epi32(vBandUlo, vBandUhi);
      }
      // V
      if (bandNumV == 2)
      {
        vBandVlo = _mm_sra_epi16(vcolV, _mm_set1_epi64x((uint64_t)bitDepth - 1));
        vBandUlo = _mm_slli_epi16(vBandUlo, 1);
      }
      else if (bandNumV == 1)
      {
        vBandVlo = _mm_sra_epi16(vcolV, _mm_set1_epi64x((uint64_t)bitDepth));
      }
      else
      {
        tmphi            = _mm_mulhi_epi16(vcolV, VbandNumV);
        tmplo            = _mm_mullo_epi16(vcolV, VbandNumV);
        vBandVlo         = _mm_unpacklo_epi16(tmplo, tmphi);  // low
        __m128i vBandVhi = _mm_unpackhi_epi16(tmplo, tmphi);  // high
        vBandVlo         = _mm_sra_epi32(vBandVlo, VBdph);
        vBandVhi         = _mm_sra_epi32(vBandVhi, VBdph);
        vBandVlo         = _mm_packs_epi32(vBandVlo, vBandVhi);

        //  bandU * bandNumV
        tmphi    = _mm_mulhi_epi16(vBandUlo, VbandNumV);
        tmplo    = _mm_mullo_epi16(vBandUlo, VbandNumV);
        vBandUlo = _mm_unpacklo_epi16(tmplo, tmphi);  // low
        vBandUhi = _mm_unpackhi_epi16(tmplo, tmphi);  // high
        vBandUlo = _mm_packs_epi32(vBandUlo, vBandUhi);
      }
      // bandY * bandNumUV
      if ((bandNumU * bandNumV) == 2)
      {
        vBandYlo = _mm_slli_epi16(vBandYlo, 1);
      }
      else if ((bandNumU * bandNumV) > 1)
      {
        tmphi            = _mm_mulhi_epi16(vBandYlo, VbandNumUV);
        tmplo            = _mm_mullo_epi16(vBandYlo, VbandNumUV);
        vBandYlo         = _mm_unpacklo_epi16(tmplo, tmphi);  // low
        __m128i vBandYhi = _mm_unpackhi_epi16(tmplo, tmphi);  // high
        vBandYlo         = _mm_packs_epi32(vBandYlo, vBandYhi);
      }
      // sum up
      vBandYlo = _mm_adds_epi16(vBandYlo, vBandUlo);
      vBandYlo = _mm_adds_epi16(vBandYlo, vBandVlo);

      __m128i test = _mm_shuffle_epi8(vBandYlo, _mm_set_epi8(1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0));
      test         = _mm_cmpeq_epi16(vBandYlo, test);
      int equal    = _mm_test_all_ones(test);

      if (equal)
      {
        int16_t idx = _mm_extract_epi16(vBandYlo, 0);
        vdst        = _mm_hadds_epi16(vdst, vdst);
        vdst        = _mm_hadds_epi16(vdst, vdst);
        vdst        = _mm_hadds_epi16(vdst, vdst);
        int16_t sum = _mm_extract_epi16(vdst, 0);
        diff[idx] += sum;
        count[idx] += 8;
      }
      else
      {
        int16_t idx[8];
        int16_t sum[8];

        _mm_storeu_si128((__m128i *)&idx, vBandYlo);
        _mm_storeu_si128((__m128i *)&sum, vdst);

        for (int n = 0; n < 8; n++)
        {
          diff[idx[n]] += sum[n];
          count[idx[n]]++;
        }
      }
      x += 8;
    }
    for (; x < endx; x++)
    {
      const Pel *colY = srcY + (x << chromaScaleX);
      const Pel *colU = srcU + x;
      const Pel *colV = srcV + x;

      const int bandY    = (*colY * bandNumY) >> bitDepth;
      const int bandU    = (*colU * bandNumU) >> bitDepth;
      const int bandV    = (*colV * bandNumV) >> bitDepth;
      const int bandIdx  = bandY * bandNumU * bandNumV + bandU * bandNumV + bandV;
      const int classIdx = bandIdx;

      diff[classIdx] += org[x] - dst[x];
      count[classIdx]++;
    }
  }
}

template<X86_VEXT vext> void SampleAdaptiveOffset::_initCCSAOX86()
{
  m_getCcSaomSampleStats       = getCcSaomSampleStats_SIMD<vext>;
  m_getCcSaomChromaSampleStats = getCcSaomChromaSampleStats_SIMD<vext>;
}

template void SampleAdaptiveOffset::_initCCSAOX86<SIMDX86>();
#endif
#endif // #ifdef TARGET_SIMD_X86
//! \}
