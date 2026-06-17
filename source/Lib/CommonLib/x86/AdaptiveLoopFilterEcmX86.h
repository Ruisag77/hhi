/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
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

#include "CommonDefX86.h"
#include "../AdaptiveLoopFilterEcm.h"
#include "AlfFixedFilters.h"

#ifdef TARGET_SIMD_X86

static void simdDeriveClassificationLaplacian(const CPelBuf &srcLuma, const Area &blk,
                                              AdaptiveLoopFilterEcm::LaplacianBuf &laplacian, const int side)
{
  if (!(blk.width % 8 == 0) || !(blk.height % 2 == 0))
  {
    AdaptiveLoopFilterEcm::deriveClassificationLaplacian(srcLuma, blk, laplacian, side);
    return;
  }

  using Direction               = AdaptiveLoopFilterEcm::Direction;
  const ptrdiff_t  imgStride    = srcLuma.stride;
  const Pel *const srcExt       = srcLuma.buf;
  const int        flP1         = side + 1;
  const int        fl2          = side << 1;
  const int        imgHExtended = blk.height + fl2;
  const int        imgWExtended = blk.width + fl2;

  const int posX = blk.pos().x;
  const int posY = blk.pos().y;

  const ptrdiff_t stride2 = imgStride * 2;
  const ptrdiff_t stride3 = imgStride * 3;

  uint16_t colSums2x2[Direction::NUM_DIRECTIONS][(AdaptiveLoopFilterEcm::CLASSIFICATION_BLK_SIZE + 10) >> 1]
                     [((AdaptiveLoopFilterEcm::CLASSIFICATION_BLK_SIZE + 16) >> 1) + 4];

  for (int i = 0; i < imgHExtended; i += 2)
  {
    const ptrdiff_t offset = (i + posY - flP1) * imgStride + posX - flP1;

    const Pel *const imgY0 = &srcExt[offset];
    const Pel *const imgY1 = &srcExt[offset + imgStride];
    const Pel *const imgY2 = &srcExt[offset + stride2];
    const Pel *const imgY3 = &srcExt[offset + stride3];

    for (int j = 0; j < imgWExtended; j += 8)
    {
      const __m128i x0     = _mm_loadu_si128((const __m128i *)(imgY0 + j));
      const __m128i x1     = _mm_loadu_si128((const __m128i *)(imgY1 + j));
      const __m128i x2     = _mm_loadu_si128((const __m128i *)(imgY2 + j));
      const __m128i x3     = _mm_loadu_si128((const __m128i *)(imgY3 + j));
      const __m128i x0next = _mm_loadu_si128((const __m128i *)(imgY0 + j + 8));
      const __m128i x1next = _mm_loadu_si128((const __m128i *)(imgY1 + j + 8));
      const __m128i x2next = _mm_loadu_si128((const __m128i *)(imgY2 + j + 8));
      const __m128i x3next = _mm_loadu_si128((const __m128i *)(imgY3 + j + 8));

      const __m128i pixel1 = _mm_slli_epi16(_mm_alignr_epi8(x1next, x1, 2), 1);
      const __m128i pixel2 = _mm_slli_epi16(_mm_alignr_epi8(x2next, x2, 2), 1);

      // ver
      __m128i ver1 = _mm_add_epi16(_mm_alignr_epi8(x0next, x0, 2), _mm_alignr_epi8(x2next, x2, 2));
      ver1         = _mm_sub_epi16(pixel1, ver1);
      ver1         = _mm_abs_epi16(ver1);
      __m128i ver2 = _mm_add_epi16(_mm_alignr_epi8(x1next, x1, 2), _mm_alignr_epi8(x3next, x3, 2));
      ver2         = _mm_sub_epi16(pixel2, ver2);
      ver2         = _mm_abs_epi16(ver2);
      ver1         = _mm_add_epi16(ver1, ver2);   // 8 ver (each is 2 values in a col)

      // hor
      __m128i hor1 = _mm_add_epi16(_mm_alignr_epi8(x1next, x1, 4), x1);
      hor1         = _mm_sub_epi16(pixel1, hor1);
      hor1         = _mm_abs_epi16(hor1);
      __m128i hor2 = _mm_add_epi16(_mm_alignr_epi8(x2next, x2, 4), x2);
      hor2         = _mm_sub_epi16(pixel2, hor2);
      hor2         = _mm_abs_epi16(hor2);
      hor1         = _mm_add_epi16(hor1, hor2);

      // dig0
      __m128i di01 = _mm_add_epi16(_mm_alignr_epi8(x2next, x2, 4), x0);
      di01         = _mm_sub_epi16(pixel1, di01);
      di01         = _mm_abs_epi16(di01);
      __m128i di02 = _mm_add_epi16(_mm_alignr_epi8(x3next, x3, 4), x1);
      di02         = _mm_sub_epi16(pixel2, di02);
      di02         = _mm_abs_epi16(di02);
      di01         = _mm_add_epi16(di01, di02);

      // dig1
      __m128i di11 = _mm_add_epi16(_mm_alignr_epi8(x0next, x0, 4), x2);
      di11         = _mm_sub_epi16(pixel1, di11);
      di11         = _mm_abs_epi16(di11);
      __m128i di12 = _mm_add_epi16(_mm_alignr_epi8(x1next, x1, 4), x3);
      di12         = _mm_sub_epi16(pixel2, di12);
      di12         = _mm_abs_epi16(di12);
      di11         = _mm_add_epi16(di11, di12);

      __m128i vh = _mm_hadd_epi16(ver1, hor1);   // ver: 2x2, 2x2, 2x2, 2x2; hor: 2x2, 2x2, 2x2, 2x2
      __m128i di = _mm_hadd_epi16(di01, di11);

      _mm_storel_epi64((__m128i *)&colSums2x2[Direction::VER][i >> 1][j >> 1], vh);
      _mm_storel_epi64((__m128i *)&colSums2x2[Direction::HOR][i >> 1][j >> 1], _mm_srli_si128(vh, 8));
      _mm_storel_epi64((__m128i *)&colSums2x2[Direction::DIAG0][i >> 1][j >> 1], di);
      _mm_storel_epi64((__m128i *)&colSums2x2[Direction::DIAG1][i >> 1][j >> 1], _mm_srli_si128(di, 8));
    }   //(int j = 0; j < imgWExtended; j += 8)
  }     // for (int i = 0; i < imgHExtended; i += 2)

  // get 4x4 sums
  for (int i = 0; i < (imgHExtended >> 1); ++i)
  {
    for (int j = 0; j < (imgWExtended >> 1); j += 4)
    {
      __m128i x0v  = _mm_cvtepu16_epi32(_mm_loadl_epi64((__m128i *)&colSums2x2[Direction::VER][i][j]));
      __m128i x1v  = _mm_cvtepu16_epi32(_mm_loadl_epi64((__m128i *)&colSums2x2[Direction::VER][i][j + 1]));
      __m128i x0h  = _mm_cvtepu16_epi32(_mm_loadl_epi64((__m128i *)&colSums2x2[Direction::HOR][i][j]));
      __m128i x1h  = _mm_cvtepu16_epi32(_mm_loadl_epi64((__m128i *)&colSums2x2[Direction::HOR][i][j + 1]));
      __m128i x0d0 = _mm_cvtepu16_epi32(_mm_loadl_epi64((__m128i *)&colSums2x2[Direction::DIAG0][i][j]));
      __m128i x1d0 = _mm_cvtepu16_epi32(_mm_loadl_epi64((__m128i *)&colSums2x2[Direction::DIAG0][i][j + 1]));
      __m128i x0d1 = _mm_cvtepu16_epi32(_mm_loadl_epi64((__m128i *)&colSums2x2[Direction::DIAG1][i][j]));
      __m128i x1d1 = _mm_cvtepu16_epi32(_mm_loadl_epi64((__m128i *)&colSums2x2[Direction::DIAG1][i][j + 1]));

      x0v  = _mm_add_epi32(x0v, x1v);
      x0h  = _mm_add_epi32(x0h, x1h);
      x0d0 = _mm_add_epi32(x0d0, x1d0);
      x0d1 = _mm_add_epi32(x0d1, x1d1);
      _mm_storeu_si128((__m128i *)&laplacian[Direction::VER][i][j], x0v);
      _mm_storeu_si128((__m128i *)&laplacian[Direction::HOR][i][j], x0h);
      _mm_storeu_si128((__m128i *)&laplacian[Direction::DIAG0][i][j], x0d0);
      _mm_storeu_si128((__m128i *)&laplacian[Direction::DIAG1][i][j], x0d1);   // 2x4

      if (i > 0)   // 4x4
      {
        x1v  = _mm_loadu_si128((const __m128i *)&laplacian[Direction::VER][i - 1][j]);
        x1h  = _mm_loadu_si128((const __m128i *)&laplacian[Direction::HOR][i - 1][j]);
        x1d0 = _mm_loadu_si128((const __m128i *)&laplacian[Direction::DIAG0][i - 1][j]);
        x1d1 = _mm_loadu_si128((const __m128i *)&laplacian[Direction::DIAG1][i - 1][j]);
        x0v  = _mm_add_epi32(x0v, x1v);
        x0h  = _mm_add_epi32(x0h, x1h);
        x0d0 = _mm_add_epi32(x0d0, x1d0);
        x0d1 = _mm_add_epi32(x0d1, x1d1);
        _mm_storeu_si128((__m128i *)&laplacian[Direction::VER][i - 1][j], x0v);
        _mm_storeu_si128((__m128i *)&laplacian[Direction::HOR][i - 1][j], x0h);
        _mm_storeu_si128((__m128i *)&laplacian[Direction::DIAG0][i - 1][j], x0d0);
        _mm_storeu_si128((__m128i *)&laplacian[Direction::DIAG1][i - 1][j], x0d1);
      }
    }   // for (int j = 0; j < (imgWExtended >> 1); j+=4)
  }     // for (int i = 0; i < (imgHExtended >> 1); i++)
}

static void simdDeriveClassificationLaplacianBig(const Area &curBlk, AdaptiveLoopFilterEcm::LaplacianBuf &laplacian)
{
  if (!(curBlk.size().width % 8 == 0) || !(curBlk.size().height % 2 == 0))
  {
    AdaptiveLoopFilterEcm::deriveClassificationLaplacianBig(curBlk, laplacian);
    return;
  }

  // get 12x12 sums, laplacian stores 4x4 sums

  using Direction        = AdaptiveLoopFilterEcm::Direction;
  const int fl2          = AdaptiveLoopFilterEcm::ALF_CLASSIFIER_FL << 1;
  const int imgHExtended = curBlk.height + fl2;
  const int imgWExtended = curBlk.width + fl2;

  // 4x12 sums
  for (int i = 0; i < (imgHExtended >> 1); ++i)
  {
    __m128i x0v  = _mm_loadu_si128((const __m128i *)&laplacian[Direction::VER][i][0]);
    __m128i x0h  = _mm_loadu_si128((const __m128i *)&laplacian[Direction::HOR][i][0]);
    __m128i x0d0 = _mm_loadu_si128((const __m128i *)&laplacian[Direction::DIAG0][i][0]);
    __m128i x0d1 = _mm_loadu_si128((const __m128i *)&laplacian[Direction::DIAG1][i][0]);
    for (int j = 4; j < (imgWExtended >> 1); j += 4)
    {
      __m128i x2v = _mm_loadu_si128((const __m128i *)&laplacian[Direction::VER][i][j]);
      __m128i x1v = _mm_shuffle_epi32(_mm_blend_epi16(x0v, x2v, 0x0f), 0x4e);
      x0v         = _mm_add_epi32(x0v, x1v);
      x0v         = _mm_add_epi32(x0v, x2v);

      __m128i x2h = _mm_loadu_si128((const __m128i *)&laplacian[Direction::HOR][i][j]);
      __m128i x1h = _mm_shuffle_epi32(_mm_blend_epi16(x0h, x2h, 0x0f), 0x4e);
      x0h         = _mm_add_epi32(x0h, x1h);
      x0h         = _mm_add_epi32(x0h, x2h);

      __m128i x2d0 = _mm_loadu_si128((const __m128i *)&laplacian[Direction::DIAG0][i][j]);
      __m128i x1d0 = _mm_shuffle_epi32(_mm_blend_epi16(x0d0, x2d0, 0x0f), 0x4e);
      x0d0         = _mm_add_epi32(x0d0, x1d0);
      x0d0         = _mm_add_epi32(x0d0, x2d0);

      __m128i x2d1 = _mm_loadu_si128((const __m128i *)&laplacian[Direction::DIAG1][i][j]);
      __m128i x1d1 = _mm_shuffle_epi32(_mm_blend_epi16(x0d1, x2d1, 0x0f), 0x4e);
      x0d1         = _mm_add_epi32(x0d1, x1d1);
      x0d1         = _mm_add_epi32(x0d1, x2d1);

      _mm_storeu_si128((__m128i *)&laplacian[Direction::VER][i][j - 4], x0v);
      _mm_storeu_si128((__m128i *)&laplacian[Direction::HOR][i][j - 4], x0h);
      _mm_storeu_si128((__m128i *)&laplacian[Direction::DIAG0][i][j - 4], x0d0);
      _mm_storeu_si128((__m128i *)&laplacian[Direction::DIAG1][i][j - 4], x0d1);
      x0v  = x2v;
      x0h  = x2h;
      x0d0 = x2d0;
      x0d1 = x2d1;
    }
  }

  for (int i = 0; i < (imgHExtended >> 1) - 5; ++i)
  {
    for (int j = 0; j < (imgWExtended >> 1) - 5; j += 4)
    {
      __m128i x0v = _mm_loadu_si128((const __m128i *)&laplacian[Direction::VER][i][j]);
      __m128i x1v = _mm_loadu_si128((const __m128i *)&laplacian[Direction::VER][i + 2][j]);
      __m128i x2v = _mm_loadu_si128((const __m128i *)&laplacian[Direction::VER][i + 4][j]);
      x0v         = _mm_add_epi32(x0v, x1v);
      x0v         = _mm_add_epi32(x0v, x2v);

      __m128i x0h = _mm_loadu_si128((const __m128i *)&laplacian[Direction::HOR][i][j]);
      __m128i x1h = _mm_loadu_si128((const __m128i *)&laplacian[Direction::HOR][i + 2][j]);
      __m128i x2h = _mm_loadu_si128((const __m128i *)&laplacian[Direction::HOR][i + 4][j]);
      x0h         = _mm_add_epi32(x0h, x1h);
      x0h         = _mm_add_epi32(x0h, x2h);

      __m128i x0d0 = _mm_loadu_si128((const __m128i *)&laplacian[Direction::DIAG0][i][j]);
      __m128i x1d0 = _mm_loadu_si128((const __m128i *)&laplacian[Direction::DIAG0][i + 2][j]);
      __m128i x2d0 = _mm_loadu_si128((const __m128i *)&laplacian[Direction::DIAG0][i + 4][j]);
      x0d0         = _mm_add_epi32(x0d0, x1d0);
      x0d0         = _mm_add_epi32(x0d0, x2d0);

      __m128i x0d1 = _mm_loadu_si128((const __m128i *)&laplacian[Direction::DIAG1][i][j]);
      __m128i x1d1 = _mm_loadu_si128((const __m128i *)&laplacian[Direction::DIAG1][i + 2][j]);
      __m128i x2d1 = _mm_loadu_si128((const __m128i *)&laplacian[Direction::DIAG1][i + 4][j]);
      x0d1         = _mm_add_epi32(x0d1, x1d1);
      x0d1         = _mm_add_epi32(x0d1, x2d1);

      _mm_storeu_si128((__m128i *)&laplacian[Direction::VER][i][j], x0v);
      _mm_storeu_si128((__m128i *)&laplacian[Direction::HOR][i][j], x0h);
      _mm_storeu_si128((__m128i *)&laplacian[Direction::DIAG0][i][j], x0d0);
      _mm_storeu_si128((__m128i *)&laplacian[Direction::DIAG1][i][j], x0d1);
    }
  }
}

template<X86_VEXT vext>
static void simdDeriveVariance(const CPelBuf &srcLuma, const Area &blk, AdaptiveLoopFilterEcm::LaplacianBuf &variance)
{
  if (!(blk.width % 8 == 0) || !(blk.height % 2 == 0))
  {
    AdaptiveLoopFilterEcm::deriveVariance(srcLuma, blk, variance);
    return;
  }

  using Direction               = AdaptiveLoopFilterEcm::Direction;
  const ptrdiff_t  imgStride    = srcLuma.stride;
  const Pel *const srcExt       = srcLuma.buf;
  const int        fl           = AdaptiveLoopFilterEcm::DIST_CLASS;
  const size_t     fl2          = fl << 1;
  const size_t     imgHExtended = blk.height + fl2;

  const int numSample  = (fl * 2 + 2) * (fl * 2 + 2);
  const int numSample2 = 128 * 128;
  const int offset     = numSample2 >> 1;

  const int num[8] { numSample, numSample, numSample, numSample, numSample, numSample, numSample, numSample };
  const int mul[8] { 13, 13, 13, 13, 13, 13, 13, 13 };
  const int off[8] { offset, offset, offset, offset, offset, offset, offset, offset };

  const int posX = blk.pos().x;
  const int posY = blk.pos().y;

  int        iOffset    = 0;
  size_t     offsetPos  = (posY - fl) * imgStride + posX - fl;
  size_t     imgStride2 = imgStride << 1;
  const Pel *imgY0      = &srcExt[offsetPos];
  const Pel *imgY1      = &srcExt[offsetPos + imgStride];

#if USE_AVX2
  if (vext >= AVX2 && (blk.width % 32) == 0)
  {
    const __m256i n    = _mm256_loadu_si256((__m256i *)num);
    const __m256i m13  = _mm256_loadu_si256((__m256i *)mul);
    const __m256i o    = _mm256_loadu_si256((__m256i *)off);
    const __m256i ones = _mm256_set1_epi16(1);

    for (int i = 0; i < imgHExtended; i += 2, iOffset += 1, imgY0 += imgStride2, imgY1 += imgStride2)
    {
      for (int j = 0; j < blk.width; j += 32)
      {
        const int jOffset = j >> 1;

        __m256i x0 = _mm256_loadu_si256((__m256i *)(imgY0 + j));
        __m256i y0 = _mm256_loadu_si256((__m256i *)(imgY1 + j));

        __m256i x8 = _mm256_loadu_si256((__m256i *)(imgY0 + j + 8));
        __m256i y8 = _mm256_loadu_si256((__m256i *)(imgY1 + j + 8));

        __m256i xx0 = _mm256_madd_epi16(x0, x0);
        __m256i xx8 = _mm256_madd_epi16(x8, x8);
        __m256i yy0 = _mm256_madd_epi16(y0, y0);
        __m256i yy8 = _mm256_madd_epi16(y8, y8);

        x0         = _mm256_add_epi16(x0, y0);
        __m256i s8 = _mm256_add_epi16(x8, y8);

        x0 = _mm256_madd_epi16(x0, ones);
        s8 = _mm256_madd_epi16(s8, ones);

        xx0 = _mm256_add_epi32(xx0, yy0);
        xx8 = _mm256_add_epi32(xx8, yy8);

        __m256i xx2 = _mm256_alignr_epi8(xx8, xx0, 4);
        __m256i xx4 = _mm256_alignr_epi8(xx8, xx0, 8);
        __m256i xx6 = _mm256_alignr_epi8(xx8, xx0, 12);

        __m256i x2 = _mm256_alignr_epi8(s8, x0, 4);
        __m256i x4 = _mm256_alignr_epi8(s8, x0, 8);
        __m256i x6 = _mm256_alignr_epi8(s8, x0, 12);

        yy0 = _mm256_add_epi32(xx0, xx2);
        xx0 = _mm256_add_epi32(xx4, xx6);
        yy0 = _mm256_add_epi32(yy0, xx8);

        y0 = _mm256_add_epi32(x0, x2);
        x4 = _mm256_add_epi32(x4, x6);
        y0 = _mm256_add_epi32(y0, s8);

        __m256i sum2 = _mm256_add_epi32(yy0, xx0);
        __m256i sum  = _mm256_add_epi32(y0, x4);

        x0 = _mm256_loadu_si256((__m256i *)(imgY0 + j + 16));
        y0 = _mm256_loadu_si256((__m256i *)(imgY1 + j + 16));

        _mm256_storeu_si256((__m256i *)&variance[2][iOffset][jOffset], sum);
        _mm256_storeu_si256((__m256i *)&variance[3][iOffset][jOffset], sum2);

        x8 = _mm256_loadu_si256((__m256i *)(imgY0 + j + 24));
        y8 = _mm256_loadu_si256((__m256i *)(imgY1 + j + 24));

        xx0 = _mm256_madd_epi16(x0, x0);
        xx8 = _mm256_madd_epi16(x8, x8);
        yy0 = _mm256_madd_epi16(y0, y0);
        yy8 = _mm256_madd_epi16(y8, y8);

        x0 = _mm256_add_epi16(x0, y0);
        s8 = _mm256_add_epi16(x8, y8);

        x0 = _mm256_madd_epi16(x0, ones);
        s8 = _mm256_madd_epi16(s8, ones);

        xx0 = _mm256_add_epi32(xx0, yy0);
        xx8 = _mm256_add_epi32(xx8, yy8);

        xx2 = _mm256_alignr_epi8(xx8, xx0, 4);
        xx4 = _mm256_alignr_epi8(xx8, xx0, 8);
        xx6 = _mm256_alignr_epi8(xx8, xx0, 12);

        x2 = _mm256_alignr_epi8(s8, x0, 4);
        x4 = _mm256_alignr_epi8(s8, x0, 8);
        x6 = _mm256_alignr_epi8(s8, x0, 12);

        yy0 = _mm256_add_epi32(xx0, xx2);
        xx0 = _mm256_add_epi32(xx4, xx6);
        yy0 = _mm256_add_epi32(yy0, xx8);

        y0 = _mm256_add_epi32(x0, x2);
        x4 = _mm256_add_epi32(x4, x6);
        y0 = _mm256_add_epi32(y0, s8);

        __m256i summ2 = _mm256_add_epi32(yy0, xx0);
        __m256i summ  = _mm256_add_epi32(y0, x4);

        _mm256_storeu_si256((__m256i *)&variance[2][iOffset][jOffset + 8], summ);
        _mm256_storeu_si256((__m256i *)&variance[3][iOffset][jOffset + 8], summ2);

        if (i == 8)
        {
          x8         = _mm256_loadu_si256((__m256i *)&variance[2][iOffset - 4][jOffset]);
          y8         = _mm256_loadu_si256((__m256i *)&variance[3][iOffset - 4][jOffset]);
          x6         = _mm256_loadu_si256((__m256i *)&variance[2][iOffset - 3][jOffset]);
          __m256i y6 = _mm256_loadu_si256((__m256i *)&variance[3][iOffset - 3][jOffset]);
          x4         = _mm256_loadu_si256((__m256i *)&variance[2][iOffset - 2][jOffset]);
          __m256i y4 = _mm256_loadu_si256((__m256i *)&variance[3][iOffset - 2][jOffset]);
          x2         = _mm256_loadu_si256((__m256i *)&variance[2][iOffset - 1][jOffset]);
          __m256i y2 = _mm256_loadu_si256((__m256i *)&variance[3][iOffset - 1][jOffset]);

          xx8         = _mm256_loadu_si256((__m256i *)&variance[2][iOffset - 4][jOffset + 8]);
          yy8         = _mm256_loadu_si256((__m256i *)&variance[3][iOffset - 4][jOffset + 8]);
          xx6         = _mm256_loadu_si256((__m256i *)&variance[2][iOffset - 3][jOffset + 8]);
          __m256i yy6 = _mm256_loadu_si256((__m256i *)&variance[3][iOffset - 3][jOffset + 8]);
          xx4         = _mm256_loadu_si256((__m256i *)&variance[2][iOffset - 2][jOffset + 8]);
          __m256i yy4 = _mm256_loadu_si256((__m256i *)&variance[3][iOffset - 2][jOffset + 8]);
          xx2         = _mm256_loadu_si256((__m256i *)&variance[2][iOffset - 1][jOffset + 8]);
          __m256i yy2 = _mm256_loadu_si256((__m256i *)&variance[3][iOffset - 1][jOffset + 8]);

          x8  = _mm256_add_epi32(sum, x8);
          y8  = _mm256_add_epi32(sum2, y8);
          xx8 = _mm256_add_epi32(summ, xx8);
          yy8 = _mm256_add_epi32(summ2, yy8);

          x4  = _mm256_add_epi32(x6, x4);
          y4  = _mm256_add_epi32(y6, y4);
          xx4 = _mm256_add_epi32(xx6, xx4);
          yy4 = _mm256_add_epi32(yy6, yy4);

          x2  = _mm256_add_epi32(x8, x2);
          y2  = _mm256_add_epi32(y8, y2);
          xx2 = _mm256_add_epi32(xx8, xx2);
          yy2 = _mm256_add_epi32(yy8, yy2);

          sum   = _mm256_add_epi32(x4, x2);
          sum2  = _mm256_add_epi32(y4, y2);
          summ  = _mm256_add_epi32(xx4, xx2);
          summ2 = _mm256_add_epi32(yy4, yy2);
          _mm256_storeu_si256((__m256i *)&variance[0][iOffset - 4][jOffset], sum);
          _mm256_storeu_si256((__m256i *)&variance[1][iOffset - 4][jOffset], sum2);
          _mm256_storeu_si256((__m256i *)&variance[0][iOffset - 4][jOffset + 8], summ);
          _mm256_storeu_si256((__m256i *)&variance[1][iOffset - 4][jOffset + 8], summ2);

          sum2  = _mm256_mullo_epi32(sum2, n);
          summ2 = _mm256_mullo_epi32(summ2, n);
          sum   = _mm256_mullo_epi32(sum, sum);
          summ  = _mm256_mullo_epi32(summ, summ);
          sum2  = _mm256_add_epi32(sum2, o);
          summ2 = _mm256_add_epi32(summ2, o);
          sum2  = _mm256_sub_epi32(sum2, sum);
          summ2 = _mm256_sub_epi32(summ2, summ);
          sum2  = _mm256_srli_epi32(sum2, 3);
          sum2  = _mm256_mullo_epi32(sum2, m13);
          sum2  = _mm256_srli_epi32(sum2, 14);
          summ2 = _mm256_srli_epi32(summ2, 3);
          summ2 = _mm256_mullo_epi32(summ2, m13);
          summ2 = _mm256_srli_epi32(summ2, 14);
          _mm256_storeu_si256((__m256i *)&variance[Direction::VARIANCE][iOffset - 4][jOffset], sum2);
          _mm256_storeu_si256((__m256i *)&variance[Direction::VARIANCE][iOffset - 4][jOffset + 8], summ2);
        }
        else if (i > 8)
        {
          x8          = _mm256_loadu_si256((__m256i *)&variance[2][iOffset - 5][jOffset]);
          xx8         = _mm256_loadu_si256((__m256i *)&variance[2][iOffset - 5][jOffset + 8]);
          y8          = _mm256_loadu_si256((__m256i *)&variance[3][iOffset - 5][jOffset]);
          yy8         = _mm256_loadu_si256((__m256i *)&variance[3][iOffset - 5][jOffset + 8]);
          x6          = _mm256_loadu_si256((__m256i *)&variance[0][iOffset - 5][jOffset]);
          xx6         = _mm256_loadu_si256((__m256i *)&variance[0][iOffset - 5][jOffset + 8]);
          __m256i y6  = _mm256_loadu_si256((__m256i *)&variance[1][iOffset - 5][jOffset]);
          __m256i yy6 = _mm256_loadu_si256((__m256i *)&variance[1][iOffset - 5][jOffset + 8]);

          x6  = _mm256_sub_epi32(x6, x8);
          xx6 = _mm256_sub_epi32(xx6, xx8);
          y6  = _mm256_sub_epi32(y6, y8);
          yy6 = _mm256_sub_epi32(yy6, yy8);

          sum   = _mm256_add_epi32(x6, sum);
          sum2  = _mm256_add_epi32(y6, sum2);
          summ  = _mm256_add_epi32(xx6, summ);
          summ2 = _mm256_add_epi32(yy6, summ2);
          _mm256_storeu_si256((__m256i *)&variance[0][iOffset - 4][jOffset], sum);
          _mm256_storeu_si256((__m256i *)&variance[1][iOffset - 4][jOffset], sum2);
          _mm256_storeu_si256((__m256i *)&variance[0][iOffset - 4][jOffset + 8], summ);
          _mm256_storeu_si256((__m256i *)&variance[1][iOffset - 4][jOffset + 8], summ2);

          sum2  = _mm256_mullo_epi32(sum2, n);
          summ2 = _mm256_mullo_epi32(summ2, n);
          sum   = _mm256_mullo_epi32(sum, sum);
          summ  = _mm256_mullo_epi32(summ, summ);
          sum2  = _mm256_add_epi32(sum2, o);
          summ2 = _mm256_add_epi32(summ2, o);
          sum2  = _mm256_sub_epi32(sum2, sum);
          summ2 = _mm256_sub_epi32(summ2, summ);
          sum2  = _mm256_srli_epi32(sum2, 3);
          summ2 = _mm256_srli_epi32(summ2, 3);
          sum2  = _mm256_mullo_epi32(sum2, m13);
          summ2 = _mm256_mullo_epi32(summ2, m13);
          sum2  = _mm256_srli_epi32(sum2, 14);
          summ2 = _mm256_srli_epi32(summ2, 14);
          _mm256_storeu_si256((__m256i *)&variance[Direction::VARIANCE][iOffset - 4][jOffset], sum2);
          _mm256_storeu_si256((__m256i *)&variance[Direction::VARIANCE][iOffset - 4][jOffset + 8], summ2);
        }
      }
    }
  }
  else
  {
#endif
    const __m128i n     = _mm_loadu_si128((__m128i *)num);
    const __m128i m13   = _mm_loadu_si128((__m128i *)mul);
    const __m128i o     = _mm_loadu_si128((__m128i *)off);
    const __m128i zeros = _mm_setzero_si128();

    for (int i = 0; i < imgHExtended; i += 2, iOffset += 1, imgY0 += imgStride2, imgY1 += imgStride2)
    {
      __m128i x0 = _mm_loadu_si128((__m128i *)(imgY0));
      __m128i y0 = _mm_loadu_si128((__m128i *)(imgY1));

      for (int j = 0; j < blk.width; j += 8)
      {
        const int jOffset = j >> 1;

        __m128i x8 = _mm_loadu_si128((__m128i *)(imgY0 + j + 8));
        __m128i y8 = _mm_loadu_si128((__m128i *)(imgY1 + j + 8));

        __m128i s0 = _mm_hadd_epi16(x0, y0);
        __m128i s8 = _mm_hadd_epi16(x8, y8);

        __m128i xx0 = _mm_madd_epi16(x0, x0);
        __m128i xx8 = _mm_madd_epi16(x8, x8);
        __m128i yy0 = _mm_madd_epi16(y0, y0);
        __m128i yy8 = _mm_madd_epi16(y8, y8);

        x0         = _mm_unpacklo_epi16(s0, zeros);
        y0         = _mm_unpackhi_epi16(s0, zeros);
        s0         = _mm_unpacklo_epi16(s8, zeros);
        __m128i s2 = _mm_unpackhi_epi16(s8, zeros);

        xx0 = _mm_add_epi32(xx0, yy0);
        xx8 = _mm_add_epi32(xx8, yy8);

        __m128i xx2 = _mm_alignr_epi8(xx8, xx0, 4);
        __m128i xx4 = _mm_alignr_epi8(xx8, xx0, 8);
        __m128i xx6 = _mm_alignr_epi8(xx8, xx0, 12);

        x0 = _mm_add_epi32(x0, y0);
        s0 = _mm_add_epi32(s0, s2);

        __m128i x2 = _mm_alignr_epi8(s0, x0, 4);
        __m128i x4 = _mm_alignr_epi8(s0, x0, 8);
        __m128i x6 = _mm_alignr_epi8(s0, x0, 12);

        yy0 = _mm_add_epi32(xx0, xx2);
        xx0 = _mm_add_epi32(xx4, xx6);
        yy0 = _mm_add_epi32(yy0, xx8);

        y0 = _mm_add_epi32(x0, x2);
        x4 = _mm_add_epi32(x4, x6);
        y0 = _mm_add_epi32(y0, s0);

        __m128i sum2 = _mm_add_epi32(yy0, xx0);
        __m128i sum  = _mm_add_epi32(y0, x4);

        x0 = x8;
        y0 = y8;

        _mm_storeu_si128((__m128i *)&variance[2][iOffset][jOffset], sum);
        _mm_storeu_si128((__m128i *)&variance[3][iOffset][jOffset], sum2);

        if (i == 8)
        {
          x8         = _mm_loadu_si128((__m128i *)&variance[2][iOffset - 4][jOffset]);
          y8         = _mm_loadu_si128((__m128i *)&variance[3][iOffset - 4][jOffset]);
          x6         = _mm_loadu_si128((__m128i *)&variance[2][iOffset - 3][jOffset]);
          __m128i y6 = _mm_loadu_si128((__m128i *)&variance[3][iOffset - 3][jOffset]);
          x4         = _mm_loadu_si128((__m128i *)&variance[2][iOffset - 2][jOffset]);
          __m128i y4 = _mm_loadu_si128((__m128i *)&variance[3][iOffset - 2][jOffset]);
          x2         = _mm_loadu_si128((__m128i *)&variance[2][iOffset - 1][jOffset]);
          __m128i y2 = _mm_loadu_si128((__m128i *)&variance[3][iOffset - 1][jOffset]);

          x8 = _mm_add_epi32(sum, x8);
          y8 = _mm_add_epi32(sum2, y8);

          x4 = _mm_add_epi32(x6, x4);
          y4 = _mm_add_epi32(y6, y4);

          x2 = _mm_add_epi32(x8, x2);
          y2 = _mm_add_epi32(y8, y2);

          sum  = _mm_add_epi32(x4, x2);
          sum2 = _mm_add_epi32(y4, y2);
          _mm_storeu_si128((__m128i *)&variance[0][iOffset - 4][jOffset], sum);
          _mm_storeu_si128((__m128i *)&variance[1][iOffset - 4][jOffset], sum2);

          sum2 = _mm_mullo_epi32(sum2, n);
          sum  = _mm_mullo_epi32(sum, sum);
          sum2 = _mm_add_epi32(sum2, o);
          sum2 = _mm_sub_epi32(sum2, sum);
          sum2 = _mm_srli_epi32(sum2, 3);
          sum2 = _mm_mullo_epi32(sum2, m13);
          sum2 = _mm_srli_epi32(sum2, 14);
          _mm_storeu_si128((__m128i *)&variance[Direction::VARIANCE][iOffset - 4][jOffset], sum2);
        }
        else if (i > 8)
        {
          x8         = _mm_loadu_si128((__m128i *)&variance[2][iOffset - 5][jOffset]);
          y8         = _mm_loadu_si128((__m128i *)&variance[3][iOffset - 5][jOffset]);
          x6         = _mm_loadu_si128((__m128i *)&variance[0][iOffset - 5][jOffset]);
          __m128i y6 = _mm_loadu_si128((__m128i *)&variance[1][iOffset - 5][jOffset]);

          x6 = _mm_sub_epi32(x6, x8);
          y6 = _mm_sub_epi32(y6, y8);

          sum  = _mm_add_epi32(x6, sum);
          sum2 = _mm_add_epi32(y6, sum2);
          _mm_storeu_si128((__m128i *)&variance[0][iOffset - 4][jOffset], sum);
          _mm_storeu_si128((__m128i *)&variance[1][iOffset - 4][jOffset], sum2);

          sum2 = _mm_mullo_epi32(sum2, n);
          sum  = _mm_mullo_epi32(sum, sum);
          sum2 = _mm_add_epi32(sum2, o);
          sum2 = _mm_sub_epi32(sum2, sum);
          sum2 = _mm_srli_epi32(sum2, 3);
          sum2 = _mm_mullo_epi32(sum2, m13);
          sum2 = _mm_srli_epi32(sum2, 14);
          _mm_storeu_si128((__m128i *)&variance[Direction::VARIANCE][iOffset - 4][jOffset], sum2);
        }
      }
    }
#if USE_AVX2
  }
#endif
}

template<X86_VEXT vext>
static void simdCalcClassGradBasedFixedFilt(AdaptiveLoopFilterEcm::ClassBuf &classifier, const Area &blkDst,
                                            const Area &curBlk, const AdaptiveLoopFilterEcm::ClsWndwSize dirWindSize,
                                            int bitDepth, const AdaptiveLoopFilterEcm::LaplacianBuf &laplacian)
{
  if (!(curBlk.width % 8 == 0) || !(curBlk.height % 2 == 0))
  {
    AdaptiveLoopFilterEcm::calcClassGradBasedFixedFilt(classifier, blkDst, curBlk, dirWindSize, bitDepth, laplacian);
    return;
  }

  using Direction             = AdaptiveLoopFilterEcm::Direction;
  const uint8_t divShift2[16] = { 0, 0, 2, 2, 4, 4, 4, 4, 6, 6, 6, 6, 8, 8, 8, 8 };
  const uint8_t sqrtSum[32]   = { 0, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3,
                                  0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1 };

  const __m128i shift  = _mm_cvtsi32_si128(9 + bitDepth - 4);
  const __m128i mult   = _mm_set1_epi32(dirWindSize == AdaptiveLoopFilterEcm::ClsWndwSize::WNDW_4x4 ||
                                            dirWindSize == AdaptiveLoopFilterEcm::ClsWndwSize::WNDW_CHROMA
                                          ? 1407
                                          : 156);
  const __m128i dirOff = _mm_set1_epi32(AdaptiveLoopFilterEcm::NUM_DIR_FIX * (AdaptiveLoopFilterEcm::NUM_DIR_FIX + 1));
  const __m128i ones   = _mm_set1_epi32(1);
  const __m128i zeros  = _mm_setzero_si128();
  const __m128i scale  = _mm_set1_epi32(192);

  const int lapOffset = (dirWindSize == AdaptiveLoopFilterEcm::ClsWndwSize::WNDW_4x4) ? 2 : 0;
  for (int i = 0; i < curBlk.height; i += 2)
  {
    const int iOffset = (i >> 1) + lapOffset;
    for (int j = 0; j < curBlk.width; j += 8)
    {
      const int jOffset  = (j >> 1) + lapOffset;
      const int iOffsetV = i >> 1;
      const int jOffsetV = j >> 1;

      __m128i sumV =
        _mm_loadu_si128((const __m128i *)&laplacian[Direction::VER][iOffset][jOffset]);   // 4 32-bit values
      __m128i sumH  = _mm_loadu_si128((const __m128i *)&laplacian[Direction::HOR][iOffset][jOffset]);
      __m128i sumD0 = _mm_loadu_si128((const __m128i *)&laplacian[Direction::DIAG0][iOffset][jOffset]);
      __m128i sumD1 = _mm_loadu_si128((const __m128i *)&laplacian[Direction::DIAG1][iOffset][jOffset]);

      // sum += sumV + sumH;
      __m128i tempAct  = _mm_add_epi32(sumV, sumH);
      __m128i activity = _mm_mullo_epi32(tempAct, mult);
      activity         = _mm_srl_epi32(activity, shift);
      activity         = _mm_min_epi32(activity, scale);

      __m128i xmm2  = activity;
      __m128i xmm0  = _mm_setzero_si128();
      __m128i xmm15 = _mm_cmpeq_epi32(xmm0, xmm0);
      __m128i xmm1  = _mm_srli_epi32(xmm15, 31);
      __m128i xmm7  = _mm_srli_epi32(xmm15, 29);
      __m128i xmm8  = _mm_srli_epi32(xmm15, 28);
      __m128i xmm9  = _mm_add_epi32(_mm_slli_epi32(xmm7, 2), xmm1);

      const __m128i LUT192 = _mm_set_epi32(0x0C020A00, 0x0E040608, 0x0E040608, 0x0C020A00);

      xmm2 = _mm_or_si128(xmm2, _mm_srli_epi32(xmm2, 1));
      xmm2 = _mm_or_si128(xmm2, _mm_srli_epi32(xmm2, 2));
      xmm2 = _mm_or_si128(xmm2, _mm_srli_epi32(xmm2, 4));
      xmm2 = _mm_mullo_epi16(xmm2, xmm9);
      xmm2 = _mm_and_si128(_mm_srli_epi32(xmm2, 5), xmm7);
      xmm2 = _mm_shuffle_epi8(LUT192, xmm2);

      __m128i xmm4 = _mm_xor_si128(activity, _mm_srli_epi32(activity, 1));
      xmm4         = _mm_cmplt_epi32(xmm4, activity);
      xmm4         = _mm_or_si128(_mm_cmpeq_epi32(activity, xmm1), xmm4);
      xmm4         = _mm_and_si128(xmm4, xmm1);

      activity = _mm_or_si128(xmm2, xmm4);

      __m128i hv1 = _mm_max_epi32(sumV, sumH);
      __m128i hv0 = _mm_min_epi32(sumV, sumH);

      __m128i d1 = _mm_max_epi32(sumD0, sumD1);
      __m128i d0 = _mm_min_epi32(sumD0, sumD1);

      // edgeStrengthHV, to optimize
      __m128i hv0Two   = _mm_slli_epi32(hv0, 1);
      __m128i hv0Eight = _mm_slli_epi32(hv0, 3);
      __m128i hv1Two   = _mm_slli_epi32(hv1, 1);
      __m128i strength = _mm_cmpgt_epi32(_mm_slli_epi32(hv1, 2), _mm_add_epi32(hv0, _mm_slli_epi32(hv0, 2)));   // 4, 5
      __m128i edgeStrengthHV = _mm_and_si128(strength, ones);

      strength       = _mm_cmpgt_epi32(hv1Two, _mm_add_epi32(hv0, hv0Two));   // 2, 3
      edgeStrengthHV = _mm_add_epi32(edgeStrengthHV, _mm_and_si128(strength, ones));

      strength       = _mm_cmpgt_epi32(hv1, hv0Two);   // 1, 2
      edgeStrengthHV = _mm_add_epi32(edgeStrengthHV, _mm_and_si128(strength, ones));

      strength       = _mm_cmpgt_epi32(hv1, _mm_add_epi32(hv0, hv0Two));   // 1, 3
      edgeStrengthHV = _mm_add_epi32(edgeStrengthHV, _mm_and_si128(strength, ones));

      strength       = _mm_cmpgt_epi32(hv1Two, _mm_add_epi32(hv0, hv0Eight));   // 2, 9
      edgeStrengthHV = _mm_add_epi32(edgeStrengthHV, _mm_and_si128(strength, ones));

      strength       = _mm_cmpgt_epi32(hv1, hv0Eight);   // 1, 8
      edgeStrengthHV = _mm_add_epi32(edgeStrengthHV, _mm_and_si128(strength, ones));

      // edgeStrengthD, to optimize
      __m128i d0Two   = _mm_slli_epi32(d0, 1);
      __m128i d0Eight = _mm_slli_epi32(d0, 3);
      __m128i d1Two   = _mm_slli_epi32(d1, 1);
      strength        = _mm_cmpgt_epi32(_mm_slli_epi32(d1, 2), _mm_add_epi32(d0, _mm_slli_epi32(d0, 2)));   // 4, 5
      __m128i edgeStrengthD = _mm_and_si128(strength, ones);

      strength      = _mm_cmpgt_epi32(d1Two, _mm_add_epi32(d0, d0Two));   // 2, 3
      edgeStrengthD = _mm_add_epi32(edgeStrengthD, _mm_and_si128(strength, ones));

      strength      = _mm_cmpgt_epi32(d1, d0Two);   // 1, 2
      edgeStrengthD = _mm_add_epi32(edgeStrengthD, _mm_and_si128(strength, ones));

      strength      = _mm_cmpgt_epi32(d1, _mm_add_epi32(d0, d0Two));   // 1, 3
      edgeStrengthD = _mm_add_epi32(edgeStrengthD, _mm_and_si128(strength, ones));

      strength      = _mm_cmpgt_epi32(d1Two, _mm_add_epi32(d0, d0Eight));   // 2, 9
      edgeStrengthD = _mm_add_epi32(edgeStrengthD, _mm_and_si128(strength, ones));

      strength      = _mm_cmpgt_epi32(d1, d0Eight);   // 1, 8
      edgeStrengthD = _mm_add_epi32(edgeStrengthD, _mm_and_si128(strength, ones));

      const __m128i hv1Xd0e = _mm_mul_epi32(hv1, d0);
      const __m128i hv0Xd1e = _mm_mul_epi32(hv0, d1);
      const __m128i hv1Xd0o = _mm_mul_epi32(_mm_srli_si128(hv1, 4), _mm_srli_si128(d0, 4));
      const __m128i hv0Xd1o = _mm_mul_epi32(_mm_srli_si128(hv0, 4), _mm_srli_si128(d1, 4));

      const __m128i xmme = _mm_sub_epi64(hv0Xd1e, hv1Xd0e);
      const __m128i xmmo = _mm_sub_epi64(hv0Xd1o, hv1Xd0o);

      __m128i dirCondition = _mm_srai_epi32(_mm_blend_epi16(_mm_srli_si128(xmme, 4), xmmo, 0xCC), 31);

      __m128i cx        = _mm_blendv_epi8(edgeStrengthHV, edgeStrengthD, dirCondition);   // x
      __m128i cy        = _mm_blendv_epi8(edgeStrengthD, edgeStrengthHV, dirCondition);   // y
      __m128i dirOffset = _mm_blendv_epi8(_mm_set1_epi32(28), zeros, dirCondition);
      // direction = (y*(y+1))/2 + x
      __m128i direction = _mm_mullo_epi32(cy, cy);
      direction         = _mm_add_epi32(direction, cy);
      direction         = _mm_srli_epi32(direction, 1);
      direction         = _mm_add_epi32(direction, cx);
      direction         = _mm_andnot_si128(_mm_cmpgt_epi32(cx, cy), direction);
      direction         = _mm_add_epi32(direction, dirOffset);

      __m128i sum2     = _mm_loadu_si128((const __m128i *)&laplacian[Direction::VARIANCE][iOffsetV][jOffsetV]);
      __m128i shiftLut = _mm_loadu_si128((const __m128i *)divShift2);
      __m128i shiftVal = _mm_shuffle_epi8(shiftLut, activity);
      shiftVal         = _mm_add_epi32(shiftVal, xmm1);
      shiftVal         = _mm_add_epi32(shiftVal, xmm1);
#if USE_AVX2
      if (vext >= AVX2)
      {
        sum2 = _mm_srlv_epi32(sum2, shiftVal);
      }
      else
#endif
      {
        uint64_t tmpVal[4];
        int32_t *pVal = (int32_t *)tmpVal;
        _mm_storeu_si128((__m128i *)tmpVal, sum2);
        _mm_storeu_si128((__m128i *)(tmpVal + 2), shiftVal);
        pVal[0] >>= pVal[4];
        pVal[1] >>= pVal[5];
        pVal[2] >>= pVal[6];
        pVal[3] >>= pVal[7];
        sum2 = _mm_loadu_si128((const __m128i *)pVal);
      }

      __m128i LUT0  = _mm_loadu_si128((const __m128i *)sqrtSum);
      __m128i LUT1  = _mm_loadu_si128((const __m128i *)&sqrtSum[16]);
      __m128i xmm16 = _mm_set_epi32(16, 16, 16, 16);
      __m128i xmm35 = _mm_set_epi32(35, 35, 35, 35);
      __m128i xmm48 = _mm_set_epi32(48, 48, 48, 48);

      __m128i use1 = _mm_cmpgt_epi32(sum2, xmm8);

      __m128i idx0 = _mm_and_si128(sum2, xmm8);
      __m128i idx1 = _mm_sub_epi32(sum2, xmm16);
      idx1         = _mm_min_epi32(idx1, xmm8);

      idx0 = _mm_shuffle_epi8(LUT0, idx0);
      idx1 = _mm_shuffle_epi8(LUT1, idx1);

      idx1 = _mm_add_epi32(idx1, _mm_slli_epi32(xmm1, 2));

      idx0 = _mm_andnot_si128(use1, idx0);
      idx1 = _mm_and_si128(use1, idx1);
      idx0 = _mm_add_epi32(idx0, idx1);

      xmm35 = _mm_cmpgt_epi32(sum2, xmm35);
      xmm48 = _mm_cmpgt_epi32(sum2, xmm48);

      xmm35 = _mm_and_si128(xmm35, xmm1);
      xmm48 = _mm_and_si128(xmm48, xmm1);

      xmm35 = _mm_add_epi32(xmm35, xmm48);

      xmm2     = _mm_add_epi32(idx0, xmm35);
      xmm2     = _mm_slli_epi32(xmm2, 4);
      activity = _mm_add_epi32(activity, xmm2);

      __m128i classIdx = _mm_mullo_epi32(dirOff, activity);
      classIdx         = _mm_add_epi32(classIdx, direction);

      // transpose
      __m128i dirTempHVMinus1 = _mm_cmpgt_epi32(sumV, sumH);
      __m128i dirTempDMinus1  = _mm_cmpgt_epi32(sumD0, sumD1);
      __m128i transposeIdx    = _mm_set1_epi32(3);
      transposeIdx            = _mm_add_epi32(transposeIdx, dirTempHVMinus1);
      transposeIdx            = _mm_add_epi32(transposeIdx, dirTempDMinus1);
      transposeIdx            = _mm_add_epi32(transposeIdx, dirTempDMinus1);

      classIdx = _mm_slli_epi16(classIdx, 2);
      classIdx = _mm_add_epi16(classIdx, transposeIdx);
      classIdx = _mm_shuffle_epi8(classIdx, _mm_setr_epi8(0, 1, 0, 1, 4, 5, 4, 5, 8, 9, 8, 9, 12, 13, 12, 13));

      _mm_storeu_si128((__m128i *)&classifier[blkDst.pos().y + i][blkDst.pos().x + j], classIdx);
      _mm_storeu_si128((__m128i *)&classifier[blkDst.pos().y + i + 1][blkDst.pos().x + j], classIdx);
    }   // for (int j = 0; j < curBlk.width; j += 8)
  }   // for (int i = 0; i < curBlk.height; i += 2)
}

static void simdCalcClassGradBasedAdaptFilt(AdaptiveLoopFilterEcm::ClassBuf &classifier, const Area &blkDst,
                                            const Area &curBlk, const AdaptiveLoopFilterEcm::ClsWndwSize dirWindSize,
                                            int bitDepth, const AdaptiveLoopFilterEcm::LaplacianBuf &laplacian)
{
  if (!(curBlk.width % 8 == 0) || !(curBlk.height % 2 == 0))
  {
    AdaptiveLoopFilterEcm::calcClassGradBasedAdaptFilt(classifier, blkDst, curBlk, dirWindSize, bitDepth, laplacian);
    return;
  }

  using Direction     = AdaptiveLoopFilterEcm::Direction;
  const __m128i shift = _mm_cvtsi32_si128(9 + bitDepth);
  const __m128i mult  = _mm_set1_epi32(dirWindSize == AdaptiveLoopFilterEcm::ClsWndwSize::WNDW_4x4 ? 4221 : 468);
  const __m128i scale = _mm_set1_epi32(15);

  for (int i = 0; i < curBlk.height; i += 2)
  {
    const int iOffset = i >> 1;
    for (int j = 0; j < curBlk.width; j += 8)
    {
      const int jOffset = j >> 1;

      const __m128i sumV =
        _mm_loadu_si128((const __m128i *)&laplacian[Direction::VER][iOffset][jOffset]);   // 4 32-bit values
      const __m128i sumH  = _mm_loadu_si128((const __m128i *)&laplacian[Direction::HOR][iOffset][jOffset]);
      const __m128i sumD0 = _mm_loadu_si128((const __m128i *)&laplacian[Direction::DIAG0][iOffset][jOffset]);
      const __m128i sumD1 = _mm_loadu_si128((const __m128i *)&laplacian[Direction::DIAG1][iOffset][jOffset]);

      // sum += sumV + sumH;
      const __m128i tempAct  = _mm_add_epi32(sumV, sumH);
      __m128i       activity = _mm_mullo_epi32(tempAct, mult);
      activity               = _mm_srl_epi32(activity, shift);
      activity               = _mm_min_epi32(activity, scale);
      __m128i classIdx = _mm_shuffle_epi8(_mm_setr_epi8(0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4), activity);
      classIdx         = _mm_add_epi32(classIdx, _mm_slli_epi32(classIdx, 2));   // activity * 5

      const __m128i hv1 = _mm_max_epi32(sumV, sumH);
      const __m128i hv0 = _mm_min_epi32(sumV, sumH);

      const __m128i d1 = _mm_max_epi32(sumD0, sumD1);
      const __m128i d0 = _mm_min_epi32(sumD0, sumD1);

      const __m128i hv1Xd0e = _mm_mul_epi32(hv1, d0);
      const __m128i hv0Xd1e = _mm_mul_epi32(hv0, d1);
      const __m128i hv1Xd0o = _mm_mul_epi32(_mm_srli_si128(hv1, 4), _mm_srli_si128(d0, 4));
      const __m128i hv0Xd1o = _mm_mul_epi32(_mm_srli_si128(hv0, 4), _mm_srli_si128(d1, 4));

      const __m128i xmme = _mm_sub_epi64(hv1Xd0e, hv0Xd1e);
      const __m128i xmmo = _mm_sub_epi64(hv1Xd0o, hv0Xd1o);

      const __m128i dirCondition = _mm_srai_epi32(_mm_blend_epi16(_mm_srli_si128(xmme, 4), xmmo, 0xCC), 31);

      const __m128i hvd1      = _mm_blendv_epi8(hv1, d1, dirCondition);
      const __m128i hvd0      = _mm_blendv_epi8(hv0, d0, dirCondition);
      const __m128i strength1 = _mm_cmpgt_epi32(hvd1, _mm_add_epi32(hvd0, hvd0));
      const __m128i strength2 =
        _mm_cmpgt_epi32(_mm_add_epi32(hvd1, hvd1), _mm_add_epi32(hvd0, _mm_slli_epi32(hvd0, 3)));
      const __m128i offset    = _mm_and_si128(strength1, _mm_set1_epi32(1));
      __m128i       direction = _mm_add_epi32(offset, _mm_and_si128(strength2, _mm_set1_epi32(1)));
      direction               = _mm_add_epi32(direction, _mm_andnot_si128(dirCondition, _mm_set1_epi32(2)));
      direction               = _mm_and_si128(direction, strength1);
      classIdx                = _mm_add_epi32(direction, classIdx);

      // transpose
      const __m128i dirTempHVMinus1 = _mm_cmpgt_epi32(sumV, sumH);
      const __m128i dirTempDMinus1  = _mm_cmpgt_epi32(sumD0, sumD1);
      __m128i       transposeIdx    = _mm_set1_epi32(3);
      transposeIdx                  = _mm_add_epi32(transposeIdx, dirTempHVMinus1);
      transposeIdx                  = _mm_add_epi32(transposeIdx, dirTempDMinus1);
      transposeIdx                  = _mm_add_epi32(transposeIdx, dirTempDMinus1);
      classIdx                      = _mm_slli_epi16(classIdx, 2);
      classIdx                      = _mm_add_epi16(classIdx, transposeIdx);
      classIdx = _mm_shuffle_epi8(classIdx, _mm_setr_epi8(0, 1, 0, 1, 4, 5, 4, 5, 8, 9, 8, 9, 12, 13, 12, 13));

      _mm_storeu_si128((__m128i *)&classifier[blkDst.pos().y + i][blkDst.pos().x + j], classIdx);
      _mm_storeu_si128((__m128i *)&classifier[blkDst.pos().y + i + 1][blkDst.pos().x + j], classIdx);
    }   // for (int j = 0; j < curBlk.width; j += 8)
  }   // for (int i = 0; i < curBlk.height; i += 2)
}

constexpr uint16_t sh(int x) { return 0x0202 * (x & 7) + 0x0100 + 0x1010 * (x & 8); }

static const uint16_t shuffleTime9[4]     = { 0, 3, 1, 3 };
static const uint16_t shuffleOp9[4][3][2] = {
  {
    { 0, 1 },
    { 0, 1 },
    { 0, 1 },
  },   // 0
  {
    { 0, 1 },
    { 0, 2 },
    { 1, 2 },
  },   // 1
  {
    { 0, 1 },
    { 0, 1 },
    { 0, 1 },
  },   // 2
  {
    { 0, 1 },
    { 0, 2 },
    { 1, 2 },
  },   // 3
};

/// @brief Shuffle table for 9x9 diamond-shaped filter
static const uint16_t shuffleTab9[4][3][2][8] = {
  // clang-format off
  {
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //0
  {
    {
      { sh(0) , sh(9) , sh(2) , sh(15) , sh(4) , sh(10) , sh(6) , sh(14) },
      { sh(8) , sh(1) , sh(5), sh(11), sh(12), sh(13), sh(7), sh(3) },
    },
    {
      { sh(8) , sh(1) , sh(9), sh(3) , sh(4) , sh(5) , sh(10) , sh(7) },
      { sh(0) , sh(2) , sh(6), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3), sh(11), sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(4), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //1
  {
    {
      { sh(0) , sh(3) , sh(2) , sh(1) , sh(8) , sh(7) , sh(6) , sh(5) },
      { sh(4) , sh(15), sh(14), sh(13), sh(12), sh(11), sh(10), sh(9) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //2
  {
    {
      { sh(0) , sh(15), sh(2), sh(9), sh(8) , sh(14), sh(6), sh(10) },
      { sh(4) , sh(3) , sh(7), sh(13), sh(1), sh(11), sh(5), sh(12) },
    },
    {
      { sh(8) , sh(1) , sh(9), sh(3) , sh(4) , sh(5) , sh(10), sh(7) },
      { sh(0) , sh(2) , sh(6), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3), sh(11), sh(5) , sh(6) , sh(4) },
      { sh(8) , sh(9) , sh(10), sh(7), sh(12), sh(13), sh(14), sh(15) },
    },
  }, //3
  // clang-format on
};

static constexpr auto MAX_NUM_ALF_LUMA_COEFF = AdaptiveLoopFilterEcm::MAX_NUM_ALF_LUMA_COEFF;

template<X86_VEXT vext>
static void simdFilter9x9Blk(const AdaptiveLoopFilterEcm::ClassBuf &classifier, const PelUnitBuf &recDst,
                             const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const CompID compId,
                             const int8_t *scaleIdxSet, const short *filterSet, const Pel *fClipSet,
                             const ClpRng &clpRng, const AdaptiveLoopFilterEcm::FixFiltBuf &fixedFilterResults)
{

  const CPelBuf srcBuffer = recSrc.get(compId);
  PelBuf        dstBuffer = recDst.get(compId);

  const size_t srcStride = srcBuffer.stride;
  const size_t dstStride = dstBuffer.stride;

  const int shift =
    (isLuma(compId) ? AlfParametersEcm::COEFF_SCALE_BITS_LUMA : AlfParametersEcm::COEFF_SCALE_BITS_CHROMA) +
    AlfParametersEcm::ALF_SCALE_SHIFT;
  const int round = 1 << (shift - 1);

  const size_t width  = blk.width;
  const size_t height = blk.height;

  constexpr size_t STEP_X = 8;
  const size_t     stepY  = isChroma(compId) ? 4 : 2;

  static_assert(sizeof(*filterSet) == 2, "ALF coeffs must be 16-bit wide");
  static_assert(sizeof(*fClipSet) == 2, "ALF clip values must be 16-bit wide");

  CHECK(blk.y % stepY, "Wrong startHeight in filtering");
  CHECK(blk.x % STEP_X, "Wrong startWidth in filtering");
  CHECK(height % stepY, "Wrong endHeight in filtering");
  CHECK(width % 4, "Wrong endWidth in filtering");

  const Pel *src = srcBuffer.buf + blk.y * srcStride + blk.x;
  Pel       *dst = dstBuffer.buf + blkDst.y * dstStride + blkDst.x;

  const __m128i mmOffset = _mm_set1_epi32(round);
  const __m128i mmMin    = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax    = _mm_set1_epi16(clpRng.max);

  for (size_t i = 0; i < height; i += stepY)
  {
    const AdaptiveLoopFilterEcm::AlfClassifier *pClass =
      isChroma(compId) ? nullptr : classifier[blkDst.y + i] + blkDst.x;
    for (size_t j = 0; j < width; j += STEP_X)
    {
      __m128i params[2][2][13];
      int32_t scaleFactor[8];
      for (int k = 0; k < 2; k++)
      {
        __m128i rawCoeff[2][4], rawClip[2][4], s0, s1, s2, s3, rawTmp0, rawTmp1;

        for (int l = 0; l < 2; l++)
        {
          const int transposeIdx = pClass ? (pClass[j + 4 * k + 2 * l] & 0x3) : 0;
          const int classIdx     = pClass ? (pClass[j + 4 * k + 2 * l] >> 2) : 0;

          scaleFactor[k * 4 + l * 2] = scaleFactor[k * 4 + l * 2 + 1] =
            AdaptiveLoopFilterEcm::ALF_SCALE_FACTOR[(int)scaleIdxSet[classIdx]];

          rawCoeff[l][0] = _mm_loadu_si128((const __m128i *)(filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawCoeff[l][1] = _mm_loadu_si128((const __m128i *)(filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
          rawCoeff[l][2] = _mm_loadu_si128((const __m128i *)(filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));
          rawCoeff[l][3] = _mm_loadl_epi64((const __m128i *)(filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 24));
          rawClip[l][0]  = _mm_loadu_si128((const __m128i *)(fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawClip[l][1]  = _mm_loadu_si128((const __m128i *)(fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
          rawClip[l][2]  = _mm_loadu_si128((const __m128i *)(fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));
          rawClip[l][3]  = _mm_loadl_epi64((const __m128i *)(fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 24));

          for (int m = 0; m < shuffleTime9[transposeIdx]; m++)
          {
            int op0 = shuffleOp9[transposeIdx][m][0];
            int op1 = shuffleOp9[transposeIdx][m][1];

            s0 = _mm_loadu_si128((const __m128i *)shuffleTab9[transposeIdx][m][0]);
            s1 = _mm_xor_si128(s0, _mm_set1_epi8((int8_t)0x80));
            s2 = _mm_loadu_si128((const __m128i *)shuffleTab9[transposeIdx][m][1]);
            s3 = _mm_xor_si128(s2, _mm_set1_epi8((int8_t)0x80));

            rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawCoeff[l][op0], s0), _mm_shuffle_epi8(rawCoeff[l][op1], s1));
            rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawCoeff[l][op0], s2), _mm_shuffle_epi8(rawCoeff[l][op1], s3));
            rawCoeff[l][op0] = rawTmp0;
            rawCoeff[l][op1] = rawTmp1;

            rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s0), _mm_shuffle_epi8(rawClip[l][op1], s1));
            rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s2), _mm_shuffle_epi8(rawClip[l][op1], s3));
            rawClip[l][op0] = rawTmp0;
            rawClip[l][op1] = rawTmp1;
          }
        }

        params[k][0][0] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][0], 0x00), _mm_shuffle_epi32(rawCoeff[1][0], 0x00));
        params[k][0][1] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][0], 0x55), _mm_shuffle_epi32(rawCoeff[1][0], 0x55));
        params[k][0][2] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][0], 0xaa), _mm_shuffle_epi32(rawCoeff[1][0], 0xaa));
        params[k][0][3] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][0], 0xff), _mm_shuffle_epi32(rawCoeff[1][0], 0xff));
        params[k][0][4] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][1], 0x00), _mm_shuffle_epi32(rawCoeff[1][1], 0x00));
        params[k][0][5] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][1], 0x55), _mm_shuffle_epi32(rawCoeff[1][1], 0x55));
        params[k][0][6] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][1], 0xaa), _mm_shuffle_epi32(rawCoeff[1][1], 0xaa));
        params[k][0][7] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][1], 0xff), _mm_shuffle_epi32(rawCoeff[1][1], 0xff));
        params[k][0][8] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][2], 0x00), _mm_shuffle_epi32(rawCoeff[1][2], 0x00));
        params[k][0][9] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][2], 0x55), _mm_shuffle_epi32(rawCoeff[1][2], 0x55));
        params[k][0][10] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][2], 0xaa), _mm_shuffle_epi32(rawCoeff[1][2], 0xaa));
        params[k][0][11] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][2], 0xff), _mm_shuffle_epi32(rawCoeff[1][2], 0xff));
        params[k][0][12] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoeff[0][3], 0x00), _mm_shuffle_epi32(rawCoeff[1][3], 0x00));

        params[k][1][0] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][0], 0x00), _mm_shuffle_epi32(rawClip[1][0], 0x00));
        params[k][1][1] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][0], 0x55), _mm_shuffle_epi32(rawClip[1][0], 0x55));
        params[k][1][2] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][0], 0xaa), _mm_shuffle_epi32(rawClip[1][0], 0xaa));
        params[k][1][3] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][0], 0xff), _mm_shuffle_epi32(rawClip[1][0], 0xff));
        params[k][1][4] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][1], 0x00), _mm_shuffle_epi32(rawClip[1][1], 0x00));
        params[k][1][5] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][1], 0x55), _mm_shuffle_epi32(rawClip[1][1], 0x55));
        params[k][1][6] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][1], 0xaa), _mm_shuffle_epi32(rawClip[1][1], 0xaa));
        params[k][1][7] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][1], 0xff), _mm_shuffle_epi32(rawClip[1][1], 0xff));
        params[k][1][8] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][2], 0x00), _mm_shuffle_epi32(rawClip[1][2], 0x00));
        params[k][1][9] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][2], 0x55), _mm_shuffle_epi32(rawClip[1][2], 0x55));
        params[k][1][10] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][2], 0xaa), _mm_shuffle_epi32(rawClip[1][2], 0xaa));
        params[k][1][11] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][2], 0xff), _mm_shuffle_epi32(rawClip[1][2], 0xff));
        params[k][1][12] =
          _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][3], 0x00), _mm_shuffle_epi32(rawClip[1][3], 0x00));
      }

      for (size_t ii = 0; ii < stepY; ii++)
      {
        const AdaptiveLoopFilterEcm::FilterRowPtrs<const Pel, 9> pImg(src + j + ii * srcStride,
                                                                      static_cast<unsigned>(srcStride));
        const AdaptiveLoopFilterEcm::FilterRowPtrs<const Pel, 5> pImgFixedBased(
          fixedFilterResults[AdaptiveLoopFilterEcm::FixFiltIdx::FIRST][blkDst.y + i + ii] + blkDst.x + j,
          fixedFilterResults[AdaptiveLoopFilterEcm::FixFiltIdx::FIRST].stride);

        __m128i cur    = _mm_loadu_si128((const __m128i *)pImg[0]);
        __m128i accumA = _mm_setzero_si128();
        __m128i accumB = _mm_setzero_si128();

        auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
        {
          const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr0), cur);
          const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr2), cur);
          const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr1), cur);
          const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr3), cur);

          __m128i val01A = _mm_unpacklo_epi16(val00, val10);
          __m128i val01B = _mm_unpackhi_epi16(val00, val10);
          __m128i val01C = _mm_unpacklo_epi16(val01, val11);
          __m128i val01D = _mm_unpackhi_epi16(val01, val11);

          __m128i limit01A = params[0][1][i];
          __m128i limit01B = params[1][1][i];

          val01A = _mm_min_epi16(val01A, limit01A);
          val01B = _mm_min_epi16(val01B, limit01B);
          val01C = _mm_min_epi16(val01C, limit01A);
          val01D = _mm_min_epi16(val01D, limit01B);

          limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
          limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);

          val01A = _mm_max_epi16(val01A, limit01A);
          val01B = _mm_max_epi16(val01B, limit01B);
          val01C = _mm_max_epi16(val01C, limit01A);
          val01D = _mm_max_epi16(val01D, limit01B);

          val01A = _mm_add_epi16(val01A, val01C);
          val01B = _mm_add_epi16(val01B, val01D);

          const __m128i coeff01A = params[0][0][i];
          const __m128i coeff01B = params[1][0][i];

          accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
          accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
        };

        process2coeffs(0, pImg[+4] + 0, pImg[-4] + 0, pImg[+3] + 1, pImg[-3] - 1);
        process2coeffs(1, pImg[+3] + 0, pImg[-3] + 0, pImg[+3] - 1, pImg[-3] + 1);
        process2coeffs(2, pImg[+2] + 2, pImg[-2] - 2, pImg[+2] + 1, pImg[-2] - 1);
        process2coeffs(3, pImg[+2] + 0, pImg[-2] + 0, pImg[+2] - 1, pImg[-2] + 1);
        process2coeffs(4, pImg[+2] - 2, pImg[-2] + 2, pImg[+1] + 3, pImg[-1] - 3);
        process2coeffs(5, pImg[+1] + 2, pImg[-1] - 2, pImg[+1] + 1, pImg[-1] - 1);
        process2coeffs(6, pImg[+1] + 0, pImg[-1] + 0, pImg[+1] - 1, pImg[-1] + 1);
        process2coeffs(7, pImg[+1] - 2, pImg[-1] + 2, pImg[+1] - 3, pImg[-1] + 3);
        process2coeffs(8, pImg[+0] + 4, pImg[-0] - 4, pImg[+0] + 3, pImg[-0] - 3);
        process2coeffs(9, pImg[+0] + 2, pImg[-0] - 2, pImg[+0] + 1, pImg[-0] - 1);

        process2coeffs(10, pImgFixedBased[+2], pImgFixedBased[-2], pImgFixedBased[+1], pImgFixedBased[-1]);
        process2coeffs(11, pImgFixedBased[+0] - 2, pImgFixedBased[+0] + 2, pImgFixedBased[+0] - 1,
                       pImgFixedBased[+0] + 1);

        __m128i val00    = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)(pImgFixedBased[+0])), cur);
        __m128i val10    = _mm_setzero_si128();
        __m128i val01A   = _mm_unpacklo_epi16(val00, val10);
        __m128i val01B   = _mm_unpackhi_epi16(val00, val10);
        __m128i limit01A = params[0][1][12];
        __m128i limit01B = params[1][1][12];

        val01A   = _mm_min_epi16(val01A, limit01A);
        val01B   = _mm_min_epi16(val01B, limit01B);
        limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
        limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
        val01A   = _mm_max_epi16(val01A, limit01A);
        val01B   = _mm_max_epi16(val01B, limit01B);

        __m128i coeff01A = params[0][0][12];
        __m128i coeff01B = params[1][0][12];
        accumA           = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
        accumB           = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));

        accumA = _mm_mullo_epi32(accumA, _mm_loadu_si128((const __m128i *)scaleFactor));
        accumB = _mm_mullo_epi32(accumB, _mm_loadu_si128((const __m128i *)(scaleFactor + 4)));

        accumA = _mm_add_epi32(accumA, mmOffset);
        accumB = _mm_add_epi32(accumB, mmOffset);

        accumA = _mm_srai_epi32(accumA, shift);
        accumB = _mm_srai_epi32(accumB, shift);

        accumA = _mm_packs_epi32(accumA, accumB);
        accumA = _mm_add_epi16(accumA, cur);
        accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

        if (j + STEP_X <= width)
        {
          _mm_storeu_si128((__m128i *)(dst + ii * dstStride + j), accumA);
        }
        else
        {
          _mm_storel_epi64((__m128i *)(dst + ii * dstStride + j), accumA);
        }
      }   // for (size_t ii = 0; ii < stepY; ii++)
    }   // for (size_t j = 0; j < width; j += STEP_X)
    src += srcStride * stepY;
    dst += dstStride * stepY;
  }
}

template<X86_VEXT vext>
static void simdFilter9x9BlkWrapper(const AdaptiveLoopFilterEcm::ClassBuf &classifier, const PelUnitBuf &recDst,
                                    const CPelBuf &recBeforeDbLuma, const CPelBuf &resiLuma, const CPelUnitBuf &recSrc,
                                    const Area &blkDst, const Area &blk, const CompID compId, const int8_t *scaleIdxSet,
                                    const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng,
                                    const AdaptiveLoopFilterEcm::FixFiltBuf &fixedFilterResults,
                                    const AdaptiveLoopFilterEcm::FixFiltBuf &fixedFilterResultsPerCtu,
                                    const AdaptiveLoopFilterEcm::CompBuf    &fixedFilterResiResults,
                                    const AdaptiveLoopFilterEcm::CompBuf    &gaussPic,
                                    const AdaptiveLoopFilterEcm::CompBuf &gaussCtu, bool isFixedFilterPaddedPerCtu,
                                    const AdaptiveLoopFilterEcm::FixFiltSetCand fixedFilterSetCandIdx)
{
  constexpr size_t STEP_X = 8;
  const size_t     stepY  = isChroma(compId) ? 4 : 2;

  // can deal with widths of four - see write back operations
  if (!(blk.width % STEP_X == 0) || !(blk.height % stepY == 0))
  {
    AdaptiveLoopFilterEcm::filterBlk<AlfParametersEcm::ALF_FILTER_9>(
      classifier, recDst, recBeforeDbLuma, resiLuma, recSrc, blkDst, blk, compId, scaleIdxSet, filterSet, fClipSet,
      clpRng, fixedFilterResults, fixedFilterResultsPerCtu, fixedFilterResiResults, gaussPic, gaussCtu,
      isFixedFilterPaddedPerCtu, fixedFilterSetCandIdx);
    return;
  }
  simdFilter9x9Blk<vext>(classifier, recDst, recSrc, blkDst, blk, compId, scaleIdxSet, filterSet, fClipSet, clpRng,
                         fixedFilterResults);
}

static const uint16_t shuffleTime13FixedBasedLongLength[4]     = { 0, 4, 2, 4 };
static const uint16_t shuffleOp13FixedBasedLongLength[4][4][2] = {
  {
    { 0, 1 },
    { 0, 1 },
    { 0, 1 },
    { 0, 1 },
  },   // 0
  {
    { 0, 1 },
    { 1, 2 },
    { 1, 3 },
    { 2, 3 },
  },   // 1
  {
    { 0, 1 },
    { 1, 2 },
    { 1, 3 },
    { 2, 3 },
  },   // 2
  {
    { 0, 1 },
    { 1, 2 },
    { 1, 3 },
    { 2, 3 },
  },   // 3
};
static const uint16_t shuffleTab13FixedBasedLongLength[4][4][2][8] = {
  // clang-format off
  {
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1) , sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8) , sh(9) , sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
  },   // IDX = 0
  {
    {
      { sh(6), sh(7), sh(8) , sh(3) , sh(9) , sh(5) , sh(0) , sh(1) },
      { sh(2), sh(4), sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0) , sh(1), sh(14), sh(15), sh(4) , sh(5), sh(9), sh(7) },
      { sh(13), sh(6), sh(10), sh(11), sh(12), sh(8), sh(2), sh(3) },
    },
    {
      { sh(0), sh(1), sh(2), sh(3) , sh(8) , sh(9) , sh(6) , sh(10) },
      { sh(4), sh(5), sh(7), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0), sh(1), sh(2) , sh(11), sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8), sh(9), sh(10), sh(3) , sh(12), sh(13), sh(14), sh(15) },
    },
  },   // IDX = 1
  {
    {
      { sh(0), sh(1), sh(2) , sh(5) , sh(4) , sh(3) , sh(6) , sh(7) },
      { sh(8), sh(9), sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0), sh(1) , sh(2) , sh(3) , sh(4) , sh(5), sh(8) , sh(7) },
      { sh(6), sh(13), sh(12), sh(11), sh(10), sh(9), sh(14), sh(15) },
    },
    {
      { sh(0), sh(1), sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8), sh(9), sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0), sh(1), sh(2) , sh(3) , sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8), sh(9), sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
  },   // IDX = 2
  {
    {
      { sh(6), sh(7), sh(8) , sh(5) , sh(9) , sh(3) , sh(0) , sh(1) },
      { sh(2), sh(4), sh(10), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0), sh(1), sh(14), sh(15), sh(4) , sh(5), sh(13), sh(7) },
      { sh(9), sh(8), sh(12), sh(11), sh(10), sh(6), sh(2) , sh(3) },
    },
    {
      { sh(0), sh(1), sh(2), sh(3) , sh(8) , sh(9) , sh(6) , sh(10) },
      { sh(4), sh(5), sh(7), sh(11), sh(12), sh(13), sh(14), sh(15) },
    },
    {
      { sh(0), sh(1), sh(2) , sh(11), sh(4) , sh(5) , sh(6) , sh(7) },
      { sh(8), sh(9), sh(10), sh(3) , sh(12), sh(13), sh(14), sh(15) },
    },
  },   // IDX = 3
  // clang-format on
};

template<X86_VEXT vext> static void simdFilter9x9BlkExtDbResiDirect(
  const AdaptiveLoopFilterEcm::ClassBuf &classifier, const PelUnitBuf &recDst, const CPelBuf &recBeforeDbLuma,
  const CPelBuf &resiLuma, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const CompID compId,
  const int8_t *scaleIdxSet, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng,
  const AdaptiveLoopFilterEcm::FixFiltBuf &fixedFilterResults,
  const AdaptiveLoopFilterEcm::FixFiltBuf &fixedFilterResultsPerCtu, const AdaptiveLoopFilterEcm::CompBuf &gaussPic,
  const AdaptiveLoopFilterEcm::CompBuf &gaussCtu, bool isFixedFilterPaddedPerCtu,
  const AdaptiveLoopFilterEcm::FixFiltSetCand fixedFilterSetCandIdx)
{
  CHECK(compId != CompID::COMP_Y, "Pre-DBF and residual data is only available for luma.");

  const CPelBuf srcBuffer = recSrc.get(compId);
  PelBuf        dstBuffer = recDst.get(compId);

  const size_t srcStride         = srcBuffer.stride;
  const size_t dstStride         = dstBuffer.stride;
  const size_t srcBeforeDbStride = recBeforeDbLuma.stride;
  const size_t srcResiStride     = resiLuma.stride;

  const int shift =
    (isLuma(compId) ? AlfParametersEcm::COEFF_SCALE_BITS_LUMA : AlfParametersEcm::COEFF_SCALE_BITS_CHROMA) +
    AlfParametersEcm::ALF_SCALE_SHIFT;
  const int round = 1 << (shift - 1);

  const size_t width  = blk.width;
  const size_t height = blk.height;

  constexpr size_t STEP_X = 8;
  constexpr size_t STEP_Y = 1;

  const __m128i mmOffset = _mm_set1_epi32(round);
  const __m128i mmMin    = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax    = _mm_set1_epi16(clpRng.max);

  static_assert(sizeof(*filterSet) == 2, "ALF coeffs must be 16-bit wide");
  static_assert(sizeof(*fClipSet) == 2, "ALF clip values must be 16-bit wide");

  const Pel *src         = srcBuffer.buf + blk.y * srcStride + blk.x;
  Pel       *dst         = dstBuffer.buf + blkDst.y * dstStride + blkDst.x;
  const Pel *srcBeforeDb = recBeforeDbLuma.buf + blk.y * srcBeforeDbStride + blk.x;
  const Pel *srcResi     = resiLuma.buf + blk.y * srcResiStride + blk.x;

  for (size_t i = 0; i < height; i += STEP_Y)
  {
    const AdaptiveLoopFilterEcm::AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;
    for (size_t j = 0; j < width; j += STEP_X)
    {
      __m128i params[2][2][20];
      int32_t scaleFactor[8];

      for (int k = 0; k < 2; k++)
      {
        __m128i rawCoef[4][5], rawClip[4][5], s0, s1, s2, s3, rawTmp0, rawTmp1;
        for (int l = 0; l < 4; l++)
        {
          const int transposeIdx = pClass[j + 4 * k + l] & 0x3;
          const int classIdx     = pClass[j + 4 * k + l] >> 2;

          scaleFactor[k * 4 + l] = AdaptiveLoopFilterEcm::ALF_SCALE_FACTOR[(int)scaleIdxSet[classIdx]];

          rawCoef[l][0] = _mm_loadu_si128((const __m128i *)(filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawCoef[l][1] = _mm_loadu_si128((const __m128i *)(filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
          rawCoef[l][2] = _mm_loadu_si128((const __m128i *)(filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));
          rawCoef[l][3] = _mm_loadu_si128((const __m128i *)(filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 24));
          rawCoef[l][4] = _mm_loadu_si128((const __m128i *)(filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 32));

          rawClip[l][0] = _mm_loadu_si128((const __m128i *)(fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawClip[l][1] = _mm_loadu_si128((const __m128i *)(fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
          rawClip[l][2] = _mm_loadu_si128((const __m128i *)(fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));
          rawClip[l][3] = _mm_loadu_si128((const __m128i *)(fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 24));
          rawClip[l][4] = _mm_loadu_si128((const __m128i *)(fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 32));

          for (int m = 0; m < shuffleTime13FixedBasedLongLength[transposeIdx]; m++)
          {
            int op0 = shuffleOp13FixedBasedLongLength[transposeIdx][m][0];
            int op1 = shuffleOp13FixedBasedLongLength[transposeIdx][m][1];

            s0 = _mm_loadu_si128((const __m128i *)shuffleTab13FixedBasedLongLength[transposeIdx][m][0]);
            s1 = _mm_xor_si128(s0, _mm_set1_epi8((int8_t)0x80));
            s2 = _mm_loadu_si128((const __m128i *)shuffleTab13FixedBasedLongLength[transposeIdx][m][1]);
            s3 = _mm_xor_si128(s2, _mm_set1_epi8((int8_t)0x80));

            rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawCoef[l][op0], s0), _mm_shuffle_epi8(rawCoef[l][op1], s1));
            rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawCoef[l][op0], s2), _mm_shuffle_epi8(rawCoef[l][op1], s3));
            rawCoef[l][op0] = rawTmp0;
            rawCoef[l][op1] = rawTmp1;

            rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s0), _mm_shuffle_epi8(rawClip[l][op1], s1));
            rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s2), _mm_shuffle_epi8(rawClip[l][op1], s3));
            rawClip[l][op0] = rawTmp0;
            rawClip[l][op1] = rawTmp1;
          }
        }   // for l

        int limR, lim0, lim1, lim2, lim3;
        limR = 5, lim0 = 5, lim1 = 5, lim2 = 5, lim3 = 5;

        for (uint8_t l = 0; l < limR; l++)
        {
          int m = l << 2;
          if (l < lim0)
          {
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x00), _mm_shuffle_epi32(rawCoef[1][l], 0x00));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x00), _mm_shuffle_epi32(rawCoef[3][l], 0x00));
            params[k][0][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x00), _mm_shuffle_epi32(rawClip[1][l], 0x00));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x00), _mm_shuffle_epi32(rawClip[3][l], 0x00));
            params[k][1][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          }
          if (l < lim1)
          {
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x55), _mm_shuffle_epi32(rawCoef[1][l], 0x55));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x55), _mm_shuffle_epi32(rawCoef[3][l], 0x55));
            params[k][0][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x55), _mm_shuffle_epi32(rawClip[1][l], 0x55));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x55), _mm_shuffle_epi32(rawClip[3][l], 0x55));
            params[k][1][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          }
          if (l < lim2)
          {
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xaa), _mm_shuffle_epi32(rawCoef[1][l], 0xaa));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xaa), _mm_shuffle_epi32(rawCoef[3][l], 0xaa));
            params[k][0][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xaa), _mm_shuffle_epi32(rawClip[1][l], 0xaa));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xaa), _mm_shuffle_epi32(rawClip[3][l], 0xaa));
            params[k][1][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          }
          if (l < lim3)
          {
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xff), _mm_shuffle_epi32(rawCoef[1][l], 0xff));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xff), _mm_shuffle_epi32(rawCoef[3][l], 0xff));
            params[k][0][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xff), _mm_shuffle_epi32(rawClip[1][l], 0xff));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xff), _mm_shuffle_epi32(rawClip[3][l], 0xff));
            params[k][1][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          }
        }   // for l
      }   // for k

      const AdaptiveLoopFilterEcm::FilterRowPtrs<const Pel, 9> pImg(src + j, static_cast<unsigned>(srcStride));

      const Pel *center;
      ptrdiff_t  stride;
      if (isFixedFilterPaddedPerCtu)
      {
        const auto &secFixedFilterResultPerCtu = fixedFilterResultsPerCtu[AdaptiveLoopFilterEcm::FixFiltIdx::SECOND];
        center                                 = secFixedFilterResultPerCtu[i] + j;
        stride                                 = secFixedFilterResultPerCtu.stride;
      }
      else
      {
        const auto &secFixedFilterResult = fixedFilterResults[AdaptiveLoopFilterEcm::FixFiltIdx::SECOND];
        center                           = secFixedFilterResult[blkDst.y + i] + blkDst.x + j;
        stride                           = secFixedFilterResult.stride;
      }
      const AdaptiveLoopFilterEcm::FilterRowPtrs<const Pel, 13> pImgFixedBased(center, stride);

      if (isFixedFilterPaddedPerCtu)
      {
        center = gaussCtu[i] + j;
        stride = gaussCtu.stride;
      }
      else
      {
        center = gaussPic[blkDst.y + i] + blkDst.x + j;
        stride = gaussPic.stride;
      }
      const AdaptiveLoopFilterEcm::FilterRowPtrs<const Pel, 5> pImgGauss(center, stride);

      const AdaptiveLoopFilterEcm::FilterRowPtrs<const Pel, 3> pImgBeforeDb(srcBeforeDb + j,
                                                                            static_cast<unsigned>(srcBeforeDbStride));

      const Pel *pImgP0 = srcResi + j;

      __m128i cur    = _mm_loadu_si128((const __m128i *)pImg[0]);
      __m128i accumA = _mm_setzero_si128();
      __m128i accumB = _mm_setzero_si128();

      auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
      {
        const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr0), cur);
        const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr2), cur);
        const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr1), cur);
        const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr3), cur);

        __m128i val01A = _mm_unpacklo_epi16(val00, val10);
        __m128i val01B = _mm_unpackhi_epi16(val00, val10);
        __m128i val01C = _mm_unpacklo_epi16(val01, val11);
        __m128i val01D = _mm_unpackhi_epi16(val01, val11);

        __m128i limit01A = params[0][1][i];
        __m128i limit01B = params[1][1][i];

        val01A = _mm_min_epi16(val01A, limit01A);
        val01B = _mm_min_epi16(val01B, limit01B);
        val01C = _mm_min_epi16(val01C, limit01A);
        val01D = _mm_min_epi16(val01D, limit01B);

        limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
        limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);

        val01A = _mm_max_epi16(val01A, limit01A);
        val01B = _mm_max_epi16(val01B, limit01B);
        val01C = _mm_max_epi16(val01C, limit01A);
        val01D = _mm_max_epi16(val01D, limit01B);

        val01A = _mm_add_epi16(val01A, val01C);
        val01B = _mm_add_epi16(val01B, val01D);

        const __m128i coeff01A = params[0][0][i];
        const __m128i coeff01B = params[1][0][i];

        accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
        accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
      };

      process2coeffs(0, pImg[+4] + 0, pImg[-4] - 0, pImg[+3] + 0, pImg[-3] - 0);
      process2coeffs(1, pImg[+2] + 0, pImg[-2] - 0, pImg[+1] + 1, pImg[-1] - 1);
      process2coeffs(2, pImg[+1] + 0, pImg[-1] - 0, pImg[+1] - 1, pImg[-1] + 1);
      process2coeffs(3, pImg[+0] + 4, pImg[-0] - 4, pImg[+0] + 3, pImg[-0] - 3);
      process2coeffs(4, pImg[+0] + 2, pImg[-0] - 2, pImg[+0] + 1, pImg[-0] - 1);

      process2coeffs(5, pImgFixedBased[-6] - 0, pImgFixedBased[+6] + 0, pImgFixedBased[-5] - 0, pImgFixedBased[+5] + 0);
      process2coeffs(6, pImgFixedBased[-4] - 0, pImgFixedBased[+4] + 0, pImgFixedBased[-3] - 0, pImgFixedBased[+3] + 0);
      process2coeffs(7, pImgFixedBased[-2] - 1, pImgFixedBased[+2] + 1, pImgFixedBased[-2] - 0, pImgFixedBased[+2] + 0);
      process2coeffs(8, pImgFixedBased[-2] + 1, pImgFixedBased[+2] - 1, pImgFixedBased[-1] - 2, pImgFixedBased[+1] + 2);
      process2coeffs(9, pImgFixedBased[-1] - 1, pImgFixedBased[+1] + 1, pImgFixedBased[-1] - 0, pImgFixedBased[+1] + 0);
      process2coeffs(10, pImgFixedBased[-1] + 1, pImgFixedBased[+1] - 1, pImgFixedBased[-1] + 2,
                     pImgFixedBased[+1] - 2);
      process2coeffs(11, pImgFixedBased[-0] - 6, pImgFixedBased[+0] + 6, pImgFixedBased[-0] - 5,
                     pImgFixedBased[+0] + 5);
      process2coeffs(12, pImgFixedBased[-0] - 4, pImgFixedBased[+0] + 4, pImgFixedBased[-0] - 3,
                     pImgFixedBased[+0] + 3);
      process2coeffs(13, pImgFixedBased[-0] - 2, pImgFixedBased[+0] + 2, pImgFixedBased[-0] - 1,
                     pImgFixedBased[+0] + 1);

      process2coeffs(14, pImgGauss[+2] - 0, pImgGauss[-2] + 0, pImgGauss[+1] - 0, pImgGauss[-1] + 0);
      process2coeffs(15, pImgGauss[+0] - 2, pImgGauss[-0] + 2, pImgGauss[+0] - 1, pImgGauss[-0] + 1);

      process2coeffs(16, pImgBeforeDb[1] + 0, pImgBeforeDb[-1] + 0, pImgBeforeDb[0] + 1, pImgBeforeDb[0] - 1);

      __m128i val00 = _mm_sub_epi16(
        _mm_loadu_si128(
          (const __m128i *)(fixedFilterResults[AdaptiveLoopFilterEcm::FixFiltIdx::FIRST][blkDst.y + i] + blkDst.x + j)),
        cur);
      __m128i val10 = _mm_sub_epi16(
        _mm_loadu_si128((const __m128i *)(fixedFilterResults[AdaptiveLoopFilterEcm::FixFiltIdx::SECOND][blkDst.y + i] +
                                          blkDst.x + j)),
        cur);

      __m128i val01A = _mm_unpacklo_epi16(val00, val10);
      __m128i val01B = _mm_unpackhi_epi16(val00, val10);

      __m128i limit01A = params[0][1][17];
      __m128i limit01B = params[1][1][17];

      val01A   = _mm_min_epi16(val01A, limit01A);
      val01B   = _mm_min_epi16(val01B, limit01B);
      limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
      limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
      val01A   = _mm_max_epi16(val01A, limit01A);
      val01B   = _mm_max_epi16(val01B, limit01B);

      __m128i coeff01A = params[0][0][17];
      __m128i coeff01B = params[1][0][17];

      accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
      accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));

      // start prediction fixed filter
      __m128i zero = _mm_setzero_si128();
      val00        = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)(pImgBeforeDb[0])), cur);
      val10        = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)pImgP0), zero);
      val01A       = _mm_unpacklo_epi16(val00, val10);
      val01B       = _mm_unpackhi_epi16(val00, val10);

      limit01A = params[0][1][18];
      limit01B = params[1][1][18];

      val01A   = _mm_min_epi16(val01A, limit01A);
      val01B   = _mm_min_epi16(val01B, limit01B);
      limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
      limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
      val01A   = _mm_max_epi16(val01A, limit01A);
      val01B   = _mm_max_epi16(val01B, limit01B);

      coeff01A = params[0][0][18];
      coeff01B = params[1][0][18];

      accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
      accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
      // end prediction fixed filter

      val00  = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)(pImgGauss[0])), cur);
      val10  = _mm_setzero_si128();
      val01A = _mm_unpacklo_epi16(val00, val10);
      val01B = _mm_unpackhi_epi16(val00, val10);

      limit01A = params[0][1][19];
      limit01B = params[1][1][19];

      val01A   = _mm_min_epi16(val01A, limit01A);
      val01B   = _mm_min_epi16(val01B, limit01B);
      limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
      limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
      val01A   = _mm_max_epi16(val01A, limit01A);
      val01B   = _mm_max_epi16(val01B, limit01B);

      coeff01A = params[0][0][19];
      coeff01B = params[1][0][19];

      accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
      accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));

      accumA = _mm_mullo_epi32(accumA, _mm_loadu_si128((const __m128i *)scaleFactor));
      accumB = _mm_mullo_epi32(accumB, _mm_loadu_si128((const __m128i *)(scaleFactor + 4)));

      accumA = _mm_add_epi32(accumA, mmOffset);
      accumB = _mm_add_epi32(accumB, mmOffset);

      accumA = _mm_srai_epi32(accumA, shift);
      accumB = _mm_srai_epi32(accumB, shift);

      accumA = _mm_packs_epi32(accumA, accumB);
      accumA = _mm_add_epi16(accumA, cur);
      accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

      _mm_storeu_si128((__m128i *)(dst + j), accumA);
    }   // for j

    src += srcStride * STEP_Y;
    dst += dstStride * STEP_Y;
    srcBeforeDb += srcBeforeDbStride * STEP_Y;
    srcResi += srcResiStride * STEP_Y;
  }   // for i
}

template<X86_VEXT vext> static void simdFilter9x9BlkExtDbResiDirectWrapper(
  const AdaptiveLoopFilterEcm::ClassBuf &classifier, const PelUnitBuf &recDst, const CPelBuf &recBeforeDbLuma,
  const CPelBuf &resiLuma, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const CompID compId,
  const int8_t *scaleIdxSet, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng,
  const AdaptiveLoopFilterEcm::FixFiltBuf &fixedFilterResults,
  const AdaptiveLoopFilterEcm::FixFiltBuf &fixedFilterResultsPerCtu,
  const AdaptiveLoopFilterEcm::CompBuf &fixedFilterResiResults, const AdaptiveLoopFilterEcm::CompBuf &gaussPic,
  const AdaptiveLoopFilterEcm::CompBuf &gaussCtu, bool isFixedFilterPaddedPerCtu,
  const AdaptiveLoopFilterEcm::FixFiltSetCand fixedFilterSetCandIdx)
{
  constexpr size_t STEP_X = 8;
  constexpr size_t STEP_Y = 1;

  if (!(blk.width % STEP_X == 0) || !(blk.height % STEP_Y == 0))
  {
    AdaptiveLoopFilterEcm::filterBlk<AlfParametersEcm::ALF_FILTER_9_EXT_DB_RESI_DIRECT>(
      classifier, recDst, recBeforeDbLuma, resiLuma, recSrc, blkDst, blk, compId, scaleIdxSet, filterSet, fClipSet,
      clpRng, fixedFilterResults, fixedFilterResultsPerCtu, fixedFilterResiResults, gaussPic, gaussCtu,
      isFixedFilterPaddedPerCtu, fixedFilterSetCandIdx);
    return;
  }

  simdFilter9x9BlkExtDbResiDirect<vext>(classifier, recDst, recBeforeDbLuma, resiLuma, recSrc, blkDst, blk, compId,
                                        scaleIdxSet, filterSet, fClipSet, clpRng, fixedFilterResults,
                                        fixedFilterResultsPerCtu, gaussPic, gaussCtu, isFixedFilterPaddedPerCtu,
                                        fixedFilterSetCandIdx);
}

template<X86_VEXT vext> static void simdFilter9x9BlkExtDbResi(
  const AdaptiveLoopFilterEcm::ClassBuf &classifier, const PelUnitBuf &recDst, const CPelBuf &recBeforeDbLuma,
  const CPelBuf &resiLuma, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const CompID compId,
  const int8_t *scaleIdxSet, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng,
  const AdaptiveLoopFilterEcm::FixFiltBuf &fixedFilterResults,
  const AdaptiveLoopFilterEcm::FixFiltBuf &fixedFilterResultsPerCtu,
  const AdaptiveLoopFilterEcm::CompBuf &fixedFilterResiResults, const AdaptiveLoopFilterEcm::CompBuf &gaussPic,
  const AdaptiveLoopFilterEcm::CompBuf &gaussCtu, bool isFixedFilterPaddedPerCtu,
  const AdaptiveLoopFilterEcm::FixFiltSetCand fixedFilterSetCandIdx)
{
  CHECKD(compId != CompID::COMP_Y, "Pre-DBF and residual data is only available for luma.");

  constexpr size_t STEP_X = 8;
  constexpr size_t STEP_Y = 1;

  if (!(blk.width % STEP_X == 0) || !(blk.height % STEP_Y == 0))
  {
    AdaptiveLoopFilterEcm::filterBlk<AlfParametersEcm::ALF_FILTER_9_EXT_DB_RESI>(
      classifier, recDst, recBeforeDbLuma, resiLuma, recSrc, blkDst, blk, compId, scaleIdxSet, filterSet, fClipSet,
      clpRng, fixedFilterResults, fixedFilterResultsPerCtu, fixedFilterResiResults, gaussPic, gaussCtu,
      isFixedFilterPaddedPerCtu, fixedFilterSetCandIdx);
    return;
  }

  const CPelBuf srcBuffer = recSrc.get(compId);
  PelBuf        dstBuffer = recDst.get(compId);

  const size_t srcStride         = srcBuffer.stride;
  const size_t dstStride         = dstBuffer.stride;
  const size_t srcBeforeDbStride = recBeforeDbLuma.stride;
  const size_t srcResiStride     = resiLuma.stride;

  const int shift =
    (isLuma(compId) ? AlfParametersEcm::COEFF_SCALE_BITS_LUMA : AlfParametersEcm::COEFF_SCALE_BITS_CHROMA) +
    AlfParametersEcm::ALF_SCALE_SHIFT;
  const int round = 1 << (shift - 1);

  const size_t width  = blk.width;
  const size_t height = blk.height;

  const __m128i mmOffset = _mm_set1_epi32(round);
  const __m128i mmMin    = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax    = _mm_set1_epi16(clpRng.max);

  static_assert(sizeof(*filterSet) == 2, "ALF coeffs must be 16-bit wide");
  static_assert(sizeof(*fClipSet) == 2, "ALF clip values must be 16-bit wide");

  const Pel *src         = srcBuffer.buf + blk.y * srcStride + blk.x;
  Pel       *dst         = dstBuffer.buf + blkDst.y * dstStride + blkDst.x;
  const Pel *srcBeforeDb = recBeforeDbLuma.buf + blk.y * srcBeforeDbStride + blk.x;
  const Pel *srcResi     = resiLuma.buf + blk.y * srcResiStride + blk.x;

  for (size_t i = 0; i < height; i += STEP_Y)
  {
    const AdaptiveLoopFilterEcm::AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;
    for (size_t j = 0; j < width; j += STEP_X)
    {
      __m128i params[2][2][18];
      int32_t scaleFactor[8];
      for (int k = 0; k < 2; k++)
      {
        __m128i rawCoef[4][5], rawClip[4][5], s0, s1, s2, s3, rawTmp0, rawTmp1;
        for (int l = 0; l < 4; l++)
        {
          const int transposeIdx = pClass[j + 4 * k + l] & 0x3;
          const int classIdx     = pClass[j + 4 * k + l] >> 2;

          scaleFactor[k * 4 + l] = AdaptiveLoopFilterEcm::ALF_SCALE_FACTOR[(int)scaleIdxSet[classIdx]];

          rawCoef[l][0] = _mm_loadu_si128((const __m128i *)(filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawCoef[l][1] = _mm_loadu_si128((const __m128i *)(filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
          rawCoef[l][2] = _mm_loadu_si128((const __m128i *)(filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));
          rawCoef[l][3] = _mm_loadu_si128((const __m128i *)(filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 24));
          rawCoef[l][4] = _mm_loadu_si128((const __m128i *)(filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 32));

          rawClip[l][0] = _mm_loadu_si128((const __m128i *)(fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF));
          rawClip[l][1] = _mm_loadu_si128((const __m128i *)(fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 8));
          rawClip[l][2] = _mm_loadu_si128((const __m128i *)(fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 16));
          rawClip[l][3] = _mm_loadu_si128((const __m128i *)(fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 24));
          rawClip[l][4] = _mm_loadu_si128((const __m128i *)(fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF + 32));

          for (int m = 0; m < shuffleTime13FixedBasedLongLength[transposeIdx]; m++)
          {
            int op0 = shuffleOp13FixedBasedLongLength[transposeIdx][m][0];
            int op1 = shuffleOp13FixedBasedLongLength[transposeIdx][m][1];

            s0 = _mm_loadu_si128((const __m128i *)shuffleTab13FixedBasedLongLength[transposeIdx][m][0]);
            s1 = _mm_xor_si128(s0, _mm_set1_epi8((int8_t)0x80));
            s2 = _mm_loadu_si128((const __m128i *)shuffleTab13FixedBasedLongLength[transposeIdx][m][1]);
            s3 = _mm_xor_si128(s2, _mm_set1_epi8((int8_t)0x80));

            rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawCoef[l][op0], s0), _mm_shuffle_epi8(rawCoef[l][op1], s1));
            rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawCoef[l][op0], s2), _mm_shuffle_epi8(rawCoef[l][op1], s3));
            rawCoef[l][op0] = rawTmp0;
            rawCoef[l][op1] = rawTmp1;

            rawTmp0 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s0), _mm_shuffle_epi8(rawClip[l][op1], s1));
            rawTmp1 = _mm_or_si128(_mm_shuffle_epi8(rawClip[l][op0], s2), _mm_shuffle_epi8(rawClip[l][op1], s3));
            rawClip[l][op0] = rawTmp0;
            rawClip[l][op1] = rawTmp1;
          }
        }   // for l

        int limR, lim0, lim1, lim2, lim3;
        limR = 5, lim0 = 5, lim1 = 5, lim2 = 4, lim3 = 4;

        for (uint8_t l = 0; l < limR; l++)
        {
          int m = l << 2;
          if (l < lim0)
          {
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x00), _mm_shuffle_epi32(rawCoef[1][l], 0x00));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x00), _mm_shuffle_epi32(rawCoef[3][l], 0x00));
            params[k][0][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x00), _mm_shuffle_epi32(rawClip[1][l], 0x00));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x00), _mm_shuffle_epi32(rawClip[3][l], 0x00));
            params[k][1][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          }
          if (l < lim1)
          {
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x55), _mm_shuffle_epi32(rawCoef[1][l], 0x55));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x55), _mm_shuffle_epi32(rawCoef[3][l], 0x55));
            params[k][0][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x55), _mm_shuffle_epi32(rawClip[1][l], 0x55));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x55), _mm_shuffle_epi32(rawClip[3][l], 0x55));
            params[k][1][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          }
          if (l < lim2)
          {
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xaa), _mm_shuffle_epi32(rawCoef[1][l], 0xaa));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xaa), _mm_shuffle_epi32(rawCoef[3][l], 0xaa));
            params[k][0][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xaa), _mm_shuffle_epi32(rawClip[1][l], 0xaa));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xaa), _mm_shuffle_epi32(rawClip[3][l], 0xaa));
            params[k][1][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          }
          if (l < lim3)
          {
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xff), _mm_shuffle_epi32(rawCoef[1][l], 0xff));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xff), _mm_shuffle_epi32(rawCoef[3][l], 0xff));
            params[k][0][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xff), _mm_shuffle_epi32(rawClip[1][l], 0xff));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xff), _mm_shuffle_epi32(rawClip[3][l], 0xff));
            params[k][1][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          }
        }   // for l
      }   // for k

      const AdaptiveLoopFilterEcm::FilterRowPtrs<const Pel, 9> pImg(src + j, static_cast<unsigned>(srcStride));

      const Pel *center;
      ptrdiff_t  stride;
      if (isFixedFilterPaddedPerCtu)
      {
        const auto &secFixedFilterResultPerCtu = fixedFilterResultsPerCtu[AdaptiveLoopFilterEcm::FixFiltIdx::SECOND];
        center                                 = secFixedFilterResultPerCtu[i] + j;
        stride                                 = secFixedFilterResultPerCtu.stride;
      }
      else
      {
        const auto &secFixedFilterResult = fixedFilterResults[AdaptiveLoopFilterEcm::FixFiltIdx::SECOND];
        center                           = secFixedFilterResult[blkDst.y + i] + blkDst.x + j;
        stride                           = secFixedFilterResult.stride;
      }
      const AdaptiveLoopFilterEcm::FilterRowPtrs<const Pel, 13> pImgFixedBased(center, stride);

      const Pel *pImg0Gauss;
      if (isFixedFilterPaddedPerCtu)
      {
        pImg0Gauss = gaussCtu[i + 0] + j;
      }
      else
      {
        pImg0Gauss = gaussPic[blkDst.y + i] + blkDst.x + j;
      }

      const AdaptiveLoopFilterEcm::FilterRowPtrs<const Pel, 3> pImgBeforeDb(srcBeforeDb + j,
                                                                            static_cast<unsigned>(srcBeforeDbStride));

      const Pel *pImgP0 = srcResi + j;

      __m128i cur    = _mm_loadu_si128((const __m128i *)pImg[0]);
      __m128i accumA = _mm_setzero_si128();
      __m128i accumB = _mm_setzero_si128();

      auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
      {
        const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr0), cur);
        const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr2), cur);
        const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr1), cur);
        const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr3), cur);

        __m128i val01A = _mm_unpacklo_epi16(val00, val10);
        __m128i val01B = _mm_unpackhi_epi16(val00, val10);
        __m128i val01C = _mm_unpacklo_epi16(val01, val11);
        __m128i val01D = _mm_unpackhi_epi16(val01, val11);

        __m128i limit01A = params[0][1][i];
        __m128i limit01B = params[1][1][i];

        val01A = _mm_min_epi16(val01A, limit01A);
        val01B = _mm_min_epi16(val01B, limit01B);
        val01C = _mm_min_epi16(val01C, limit01A);
        val01D = _mm_min_epi16(val01D, limit01B);

        limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
        limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);

        val01A = _mm_max_epi16(val01A, limit01A);
        val01B = _mm_max_epi16(val01B, limit01B);
        val01C = _mm_max_epi16(val01C, limit01A);
        val01D = _mm_max_epi16(val01D, limit01B);

        val01A = _mm_add_epi16(val01A, val01C);
        val01B = _mm_add_epi16(val01B, val01D);

        const __m128i coeff01A = params[0][0][i];
        const __m128i coeff01B = params[1][0][i];

        accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
        accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
      };

      process2coeffs(0, pImg[+4] + 0, pImg[-4] - 0, pImg[+3] + 0, pImg[-3] - 0);
      process2coeffs(1, pImg[+2] + 0, pImg[-2] - 0, pImg[+1] + 1, pImg[-1] - 1);
      process2coeffs(2, pImg[+1] + 0, pImg[-1] - 0, pImg[+1] - 1, pImg[-1] + 1);
      process2coeffs(3, pImg[+0] + 4, pImg[-0] - 4, pImg[+0] + 3, pImg[-0] - 3);
      process2coeffs(4, pImg[+0] + 2, pImg[-0] - 2, pImg[+0] + 1, pImg[-0] - 1);

      process2coeffs(5, pImgFixedBased[-6] - 0, pImgFixedBased[+6] + 0, pImgFixedBased[-5] - 0, pImgFixedBased[+5] + 0);
      process2coeffs(6, pImgFixedBased[-4] - 0, pImgFixedBased[+4] + 0, pImgFixedBased[-3] - 0, pImgFixedBased[+3] + 0);
      process2coeffs(7, pImgFixedBased[-2] - 1, pImgFixedBased[+2] + 1, pImgFixedBased[-2] - 0, pImgFixedBased[+2] + 0);
      process2coeffs(8, pImgFixedBased[-2] + 1, pImgFixedBased[+2] - 1, pImgFixedBased[-1] - 2, pImgFixedBased[+1] + 2);
      process2coeffs(9, pImgFixedBased[-1] - 1, pImgFixedBased[+1] + 1, pImgFixedBased[-1] - 0, pImgFixedBased[+1] + 0);
      process2coeffs(10, pImgFixedBased[-1] + 1, pImgFixedBased[+1] - 1, pImgFixedBased[-1] + 2,
                     pImgFixedBased[+1] - 2);
      process2coeffs(11, pImgFixedBased[-0] - 6, pImgFixedBased[+0] + 6, pImgFixedBased[-0] - 5,
                     pImgFixedBased[+0] + 5);
      process2coeffs(12, pImgFixedBased[-0] - 4, pImgFixedBased[+0] + 4, pImgFixedBased[-0] - 3,
                     pImgFixedBased[+0] + 3);
      process2coeffs(13, pImgFixedBased[-0] - 2, pImgFixedBased[+0] + 2, pImgFixedBased[-0] - 1,
                     pImgFixedBased[+0] + 1);

      process2coeffs(14, pImgBeforeDb[1] + 0, pImgBeforeDb[-1] + 0, pImgBeforeDb[0] + 1, pImgBeforeDb[0] - 1);

      __m128i val00 = _mm_sub_epi16(
        _mm_loadu_si128(
          (const __m128i *)(fixedFilterResults[AdaptiveLoopFilterEcm::FixFiltIdx::FIRST][blkDst.y + i] + blkDst.x + j)),
        cur);
      __m128i val10 = _mm_sub_epi16(
        _mm_loadu_si128((const __m128i *)(fixedFilterResults[AdaptiveLoopFilterEcm::FixFiltIdx::SECOND][blkDst.y + i] +
                                          blkDst.x + j)),
        cur);

      __m128i val01A = _mm_unpacklo_epi16(val00, val10);
      __m128i val01B = _mm_unpackhi_epi16(val00, val10);

      __m128i limit01A = params[0][1][15];
      __m128i limit01B = params[1][1][15];

      val01A   = _mm_min_epi16(val01A, limit01A);
      val01B   = _mm_min_epi16(val01B, limit01B);
      limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
      limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
      val01A   = _mm_max_epi16(val01A, limit01A);
      val01B   = _mm_max_epi16(val01B, limit01B);

      __m128i coeff01A = params[0][0][15];
      __m128i coeff01B = params[1][0][15];

      accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
      accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));

      // start residual fixed filter
      __m128i zero = _mm_setzero_si128();
      val00 =
        _mm_sub_epi16(_mm_loadu_si128((const __m128i *)(fixedFilterResiResults[blkDst.y + i] + blkDst.x + j)), zero);

      val10  = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)(pImgBeforeDb[0])), cur);
      val01A = _mm_unpacklo_epi16(val00, val10);
      val01B = _mm_unpackhi_epi16(val00, val10);

      limit01A = params[0][1][16];
      limit01B = params[1][1][16];

      val01A   = _mm_min_epi16(val01A, limit01A);
      val01B   = _mm_min_epi16(val01B, limit01B);
      limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
      limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
      val01A   = _mm_max_epi16(val01A, limit01A);
      val01B   = _mm_max_epi16(val01B, limit01B);

      coeff01A = params[0][0][16];
      coeff01B = params[1][0][16];

      accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
      accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
      // end residual fixed filter

      val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)pImgP0), zero);
      val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)(pImg0Gauss)), cur);

      val01A = _mm_unpacklo_epi16(val00, val10);
      val01B = _mm_unpackhi_epi16(val00, val10);

      limit01A = params[0][1][17];
      limit01B = params[1][1][17];

      val01A   = _mm_min_epi16(val01A, limit01A);
      val01B   = _mm_min_epi16(val01B, limit01B);
      limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
      limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);
      val01A   = _mm_max_epi16(val01A, limit01A);
      val01B   = _mm_max_epi16(val01B, limit01B);

      coeff01A = params[0][0][17];
      coeff01B = params[1][0][17];

      accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
      accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));

      accumA = _mm_mullo_epi32(accumA, _mm_loadu_si128((const __m128i *)scaleFactor));
      accumB = _mm_mullo_epi32(accumB, _mm_loadu_si128((const __m128i *)(scaleFactor + 4)));

      accumA = _mm_add_epi32(accumA, mmOffset);
      accumB = _mm_add_epi32(accumB, mmOffset);

      accumA = _mm_srai_epi32(accumA, shift);
      accumB = _mm_srai_epi32(accumB, shift);

      accumA = _mm_packs_epi32(accumA, accumB);
      accumA = _mm_add_epi16(accumA, cur);
      accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

      _mm_storeu_si128((__m128i *)(dst + j), accumA);
    }   // for j

    src += srcStride * STEP_Y;
    dst += dstStride * STEP_Y;
    srcBeforeDb += srcBeforeDbStride * STEP_Y;
    srcResi += srcResiStride * STEP_Y;
  }   // for i
}

// Gauss Filter
template<X86_VEXT vext>
static void simdGaussFiltering(AdaptiveLoopFilterEcm::CompBuf &gaussPic, const CPelBuf &srcLuma, const Area &blkDst,
                               const Area &blk, const ClpRng &clpRng,
                               const std::array<Pel, AdaptiveLoopFilterEcm::MAX_ALF_NUM_CLIP_VALS> &clippingValues)
{
  // TODO: ALF: The coefficients are also defined in the non-SIMD code. BS 2023-09-08
  static constexpr int16_t gaussCoefTable[25] = {
    8, 22, 30, 22, 22, 60, 85, 60, 22, 8, 30, 85, 119, 85, 30, 8, 22, 60, 85, 60, 22, 22, 30, 22, 8,
  };

  static constexpr int16_t gaussClipIdxTable[25] = {
    3, 2, 1, 2, 2, 1, 0, 1, 2, 3, 1, 0, 0, 0, 1, 3, 2, 1, 0, 1, 2, 2, 1, 2, 3,
  };

  constexpr int STEP_X = 8;
  constexpr int STEP_Y = 1;

  if (!(blk.width % STEP_X == 0) || !(blk.height % STEP_Y == 0))
  {
    AdaptiveLoopFilterEcm::gaussFiltering(gaussPic, srcLuma, blkDst, blk, clpRng, clippingValues);
    return;
  }

  constexpr int16_t NUM_COEFF = 12;
  constexpr int16_t DIFF_TH   = 32;

  int16_t gaussClipTable[25] = { 0 };
  for (int i = 0; i < NUM_COEFF; ++i)
  {
    gaussClipTable[i] = clippingValues[gaussClipIdxTable[i]];
  }

  const __m128i offsetMax = _mm_set1_epi16(DIFF_TH);
  const __m128i offsetMin = _mm_sub_epi16(_mm_setzero_si128(), offsetMax);

  const CPelBuf  &srcBuffer = srcLuma;
  const ptrdiff_t srcStride = srcBuffer.stride;

  constexpr int SHIFT = 10;
  constexpr int ROUND = 1 << (SHIFT - 1);

  const int width  = blk.width;
  const int height = blk.height;

  const __m128i mmOffset = _mm_set1_epi32(ROUND);
  const __m128i mmMin    = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax    = _mm_set1_epi16(clpRng.max);

  static_assert(sizeof(*gaussCoefTable) == 2, "ALF coeffs must be 16-bit wide");
  static_assert(sizeof(*gaussClipTable) == 2, "ALF clip values must be 16-bit wide");

  const Pel *src = srcBuffer.buf + blk.y * srcStride + blk.x;

  for (int i = 0; i < height; i += STEP_Y)
  {
    for (int j = 0; j < width; j += STEP_X)
    {
      __m128i params[2][2][6];

      for (int k = 0; k < 2; k++)
      {
        __m128i rawCoef[4][2], rawClip[4][2], s0, s1;

        for (int l = 0; l < 4; l++)
        {
          rawCoef[l][0] = _mm_loadu_si128((const __m128i *)(gaussCoefTable + 0));
          rawCoef[l][1] = _mm_loadu_si128((const __m128i *)(gaussCoefTable + 8));

          rawClip[l][0] = _mm_loadu_si128((const __m128i *)(gaussClipTable + 0));
          rawClip[l][1] = _mm_loadu_si128((const __m128i *)(gaussClipTable + 8));
        }   // for l

        for (uint8_t l = 0; l < 2; l++)
        {
          int m = l << 2;

          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x00), _mm_shuffle_epi32(rawCoef[1][l], 0x00));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x00), _mm_shuffle_epi32(rawCoef[3][l], 0x00));
          params[k][0][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x00), _mm_shuffle_epi32(rawClip[1][l], 0x00));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x00), _mm_shuffle_epi32(rawClip[3][l], 0x00));
          params[k][1][0 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);

          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0x55), _mm_shuffle_epi32(rawCoef[1][l], 0x55));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0x55), _mm_shuffle_epi32(rawCoef[3][l], 0x55));
          params[k][0][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0x55), _mm_shuffle_epi32(rawClip[1][l], 0x55));
          s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0x55), _mm_shuffle_epi32(rawClip[3][l], 0x55));
          params[k][1][1 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);

          if (l < 1)
          {
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xaa), _mm_shuffle_epi32(rawCoef[1][l], 0xaa));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xaa), _mm_shuffle_epi32(rawCoef[3][l], 0xaa));
            params[k][0][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xaa), _mm_shuffle_epi32(rawClip[1][l], 0xaa));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xaa), _mm_shuffle_epi32(rawClip[3][l], 0xaa));
            params[k][1][2 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);

            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[0][l], 0xff), _mm_shuffle_epi32(rawCoef[1][l], 0xff));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawCoef[2][l], 0xff), _mm_shuffle_epi32(rawCoef[3][l], 0xff));
            params[k][0][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
            s0 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[0][l], 0xff), _mm_shuffle_epi32(rawClip[1][l], 0xff));
            s1 = _mm_unpacklo_epi64(_mm_shuffle_epi32(rawClip[2][l], 0xff), _mm_shuffle_epi32(rawClip[3][l], 0xff));
            params[k][1][3 + m] = _mm_blend_epi16(_mm_shuffle_epi32(s0, 0x88), _mm_shuffle_epi32(s1, 0x88), 0xf0);
          }
        }   // for l
      }   // for k

      const AdaptiveLoopFilterEcm::FilterRowPtrs<const Pel, 7> pImg(src + j, srcStride);

      __m128i cur    = _mm_loadu_si128((const __m128i *)pImg[0]);
      __m128i accumA = mmOffset;
      __m128i accumB = mmOffset;

      auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
      {
        const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr0), cur);
        const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr2), cur);
        const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr1), cur);
        const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr3), cur);

        __m128i val01A = _mm_unpacklo_epi16(val00, val10);
        __m128i val01B = _mm_unpackhi_epi16(val00, val10);
        __m128i val01C = _mm_unpacklo_epi16(val01, val11);
        __m128i val01D = _mm_unpackhi_epi16(val01, val11);

        __m128i limit01A = params[0][1][i];
        __m128i limit01B = params[1][1][i];

        val01A = _mm_min_epi16(val01A, limit01A);
        val01B = _mm_min_epi16(val01B, limit01B);
        val01C = _mm_min_epi16(val01C, limit01A);
        val01D = _mm_min_epi16(val01D, limit01B);

        limit01A = _mm_sub_epi16(_mm_setzero_si128(), limit01A);
        limit01B = _mm_sub_epi16(_mm_setzero_si128(), limit01B);

        val01A = _mm_max_epi16(val01A, limit01A);
        val01B = _mm_max_epi16(val01B, limit01B);
        val01C = _mm_max_epi16(val01C, limit01A);
        val01D = _mm_max_epi16(val01D, limit01B);

        val01A = _mm_add_epi16(val01A, val01C);
        val01B = _mm_add_epi16(val01B, val01D);

        const __m128i coeff01A = params[0][0][i];
        const __m128i coeff01B = params[1][0][i];

        accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff01A));
        accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff01B));
      };

      process2coeffs(0, pImg[-3] - 0, pImg[+3] + 0, pImg[-2] - 1, pImg[+2] + 1);
      process2coeffs(1, pImg[-2] - 0, pImg[+2] + 0, pImg[-2] + 1, pImg[+2] - 1);
      process2coeffs(2, pImg[-1] - 2, pImg[+1] + 2, pImg[-1] - 1, pImg[+1] + 1);
      process2coeffs(3, pImg[-1] - 0, pImg[+1] + 0, pImg[-1] + 1, pImg[+1] - 1);
      process2coeffs(4, pImg[-1] + 2, pImg[+1] - 2, pImg[-0] - 3, pImg[+0] + 3);
      process2coeffs(5, pImg[-0] - 2, pImg[+0] + 2, pImg[-0] - 1, pImg[+0] + 1);

      accumA = _mm_srai_epi32(accumA, SHIFT);
      accumB = _mm_srai_epi32(accumB, SHIFT);

      accumA = _mm_packs_epi32(accumA, accumB);

      // Clip Offset
      accumA = _mm_min_epi16(accumA, offsetMax);
      accumA = _mm_max_epi16(accumA, offsetMin);

      accumA = _mm_add_epi16(accumA, cur);
      accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

      _mm_storeu_si128((__m128i *)(gaussPic[blkDst.y + i] + blkDst.x + j), accumA);
    }   // for j
    src += srcStride * STEP_Y;
  }   // for i
}

template<size_t NUM_COEFF> using FixFilterPackedData =
  std::array<std::array<std::array<short, NUM_COEFF>, AdaptiveLoopFilterEcm::NUM_FIXED_FILTERS_PER_SET>,
             AdaptiveLoopFilterEcm::NUM_FIXED_FILTER_SETS>;

class FixFilterPackedData13Db9 : public FixFilterPackedData<AdaptiveLoopFilterEcm::FIX_FILTER_NUM_COEFF_13_DB_9>
{
public:
  FixFilterPackedData13Db9(const FixFilterPackedData13Db9 &) = delete;
  FixFilterPackedData13Db9(FixFilterPackedData13Db9 &&)      = delete;

  FixFilterPackedData13Db9 &operator=(const FixFilterPackedData13Db9 &) = delete;
  FixFilterPackedData13Db9 &operator=(FixFilterPackedData13Db9 &&)      = delete;

  // TODO: Could be constexpr if constructor was constexpr.
  static inline const FixFilterPackedData13Db9 &get();

private:
  // TODO: Could be constexpr but maximum number of steps is exceeded.
  inline FixFilterPackedData13Db9();
};

FixFilterPackedData13Db9::FixFilterPackedData13Db9()
  : FixFilterPackedData<AdaptiveLoopFilterEcm::FIX_FILTER_NUM_COEFF_13_DB_9>()
{
  constexpr std::array<int, AdaptiveLoopFilterEcm::FIX_FILTER_NUM_COEFF_13_DB_9> p0 {
    // clang-format off
     0,  2,  6, 12, 20, 30,
    36, 37, 38, 39, 40, 41,
     1,  4,  9, 16, 25, 11, 18, 27,
     3,  8, 15, 24, 35, 13, 22, 33,
     5, 10, 17, 26, 19, 28, 29, 31,
     7, 14, 23, 34, 21, 32,
    42, 44, 48, 54, 58, 59, 60, 61,
    43, 46, 51, 45, 50, 57,
    47, 52, 53, 49, 56, 55, 62, 63
    // clang-format on
  };

  auto &packedDataFixedFilters13Db9 = *this;
  for (int i = 0; i < AdaptiveLoopFilterEcm::NUM_FIXED_FILTER_SETS; ++i)
  {
    for (int j = 0; j < AdaptiveLoopFilterEcm::NUM_FIXED_FILTERS_PER_SET; ++j)
    {
      for (int k = 0; k < AdaptiveLoopFilterEcm::FIX_FILTER_NUM_COEFF_13_DB_9; ++k)
      {
        packedDataFixedFilters13Db9[i][j][k] =
          (m_filterCoeffFixed13Db9[i][j][p0[k]] << 2) | m_clippingFixed13Db9[i][j][p0[k]];
      }
    }
  }
}

const FixFilterPackedData13Db9 &FixFilterPackedData13Db9::get()
{
  static const FixFilterPackedData13Db9 obj;
  return obj;
}

// TODO: Could theoretically be constexpr. See FixFilterPackedData13Db9.
static const FixFilterPackedData13Db9 &packedDataFixedFilters13Db9 = FixFilterPackedData13Db9::get();

static const int8_t shTab[4][9][16] = {
  // clang-format off
  {
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
    {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 }
  },
  {
    { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
    {  8,  9,  6,  7,  4,  5,  2,  3,  0,  1, 14, 15, 12, 13, 10, 11 },
    {  8,  9,  6,  7,  4,  5,  2,  3,  0,  1, 14, 15, 12, 13, 10, 11 },
    {  6,  7,  4,  5,  2,  3,  0,  1, 10, 11,  8,  9, 12, 13, 14, 15 },
    {  0,  1,  2,  3, 10, 11,  8,  9,  6,  7,  4,  5, 14, 15, 12, 13 },
    {  8,  9, 10, 11, 12, 13, 14, 15,  0,  1,  2,  3,  4,  5,  6,  7 },
    {  4,  5,  2,  3,  0,  1, 10, 11,  8,  9,  6,  7, 12, 13, 14, 15 },
    {  2,  3,  0,  1,  4,  5,  8,  9,  6,  7, 10, 11, 12, 13, 14, 15 },
  },
  {
    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
    { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    { 14, 15, 12, 13,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11 },
    {  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,  2,  3,  0,  1 },
    {  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    {  6,  7,  8,  9, 10, 11,  0,  1,  2,  3,  4,  5, 12, 13, 14, 15 },
    {  6,  7,  8,  9, 10, 11,  0,  1,  2,  3,  4,  5, 12, 13, 14, 15 }
  },
  {
    { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    {  8,  9,  6,  7,  4,  5,  2,  3,  0,  1, 14, 15, 12, 13, 10, 11 },
    {  8,  9,  6,  7,  4,  5,  2,  3,  0,  1, 14, 15, 12, 13, 10, 11 },
    { 14, 15, 12, 13,  6,  7,  4,  5,  2,  3,  0,  1, 10, 11,  8,  9 },
    { 10, 11,  8,  9,  6,  7,  4,  5, 14, 15, 12, 13,  2,  3,  0,  1 },
    {  8,  9, 10, 11, 12, 13, 14, 15,  0,  1,  2,  3,  4,  5,  6,  7 },
    { 10, 11,  8,  9,  6,  7,  4,  5,  2,  3,  0,  1, 12, 13, 14, 15 },
    {  8,  9,  6,  7, 10, 11,  2,  3,  0,  1,  4,  5, 12, 13, 14, 15 }
  }
  // clang-format on
};

template<X86_VEXT vext> static void
  simdFixFilter13x13Db9Blk(const AdaptiveLoopFilterEcm::ClassBuf &classifier, const CPelBuf &srcLuma,
                           const CPelBuf &srcLumaBeforeDb, const AdaptiveLoopFilterEcm::CompBuf &firstFixedFilterResult,
                           const Area &curBlk, AdaptiveLoopFilterEcm::CompBuf &fixedFilterResult, const Area &blkDst,
                           int picWidth, const unsigned fixedFilterSetIdx, const ClpRng &clpRng,
                           const std::array<Pel, AdaptiveLoopFilterEcm::MAX_ALF_NUM_CLIP_VALS> &clippingValues)
{
  constexpr int STEP_X = 8;
  constexpr int STEP_Y = 2;

  if (!(curBlk.width % STEP_X == 0) || !(curBlk.height % STEP_Y == 0))
  {
    AdaptiveLoopFilterEcm::fixedFilter13x13Db9Blk(classifier, srcLuma, srcLumaBeforeDb, firstFixedFilterResult, curBlk,
                                                  fixedFilterResult, blkDst, picWidth, fixedFilterSetIdx, clpRng,
                                                  clippingValues);
    return;
  }

  constexpr int SHIFT = AdaptiveLoopFilterEcm::FIX_FILTER_COEFF_SCALE_BITS;
  constexpr int ROUND = 1 << (SHIFT - 1);

  const int width  = curBlk.width;
  const int height = curBlk.height;

  const ptrdiff_t srcStride              = srcLuma.stride;
  const ptrdiff_t srcBeforeDbStride      = srcLumaBeforeDb.stride;
  const ptrdiff_t firstFixFiltResStride  = firstFixedFilterResult.stride;
  const ptrdiff_t srcStride2             = srcStride * STEP_Y;
  const ptrdiff_t srcBeforeDbStride2     = srcBeforeDbStride * STEP_Y;
  const ptrdiff_t firstFixFiltResStride2 = firstFixFiltResStride * STEP_Y;

  const Pel *src             = srcLuma.buf + curBlk.y * srcStride + curBlk.x;
  const Pel *srcBeforeDb     = srcLumaBeforeDb.buf + curBlk.y * srcBeforeDbStride + curBlk.x;
  const Pel *firstFixFiltRes = firstFixedFilterResult.buf + curBlk.y * firstFixFiltResStride + curBlk.x;

  const auto &filterCoeffFixed = packedDataFixedFilters13Db9[fixedFilterSetIdx];
  const auto &classIndFixed    = m_classIdnFixedFilter13Db9[fixedFilterSetIdx];

#if USE_AVX2
  if (vext >= AVX2 && (width % 16) == 0)
  {
    const __m256i mmOffset            = _mm256_set1_epi32(ROUND);
    const __m256i mmMin               = _mm256_set1_epi16(clpRng.min);
    const __m256i mmMax               = _mm256_set1_epi16(clpRng.max);
    const __m128i mmClippingValues    = _mm_loadl_epi64((const __m128i *)clippingValues.data());
    __m256i       mmClippingValues256 = _mm256_castsi128_si256(mmClippingValues);
    mmClippingValues256               = _mm256_insertf128_si256(mmClippingValues256, mmClippingValues, 1);
    const __m256i mm11                = _mm256_set1_epi8(1);
    const __m256i mm3                 = _mm256_set1_epi16(3);
    for (int i = 0; i < height; i += STEP_Y)
    {
      const AdaptiveLoopFilterEcm::AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;

      for (int j = 0; j < width; j += STEP_X * 2)
      {
        __m256i params[32];
        __m256i rawCoef[4][9];

        for (int m = 0; m < 4; m++)
        {
          int       transposeIdx0 = pClass[j + 2 * m] & 0x3;
          const int filterIdx0    = classIndFixed[pClass[j + 2 * m] >> 2];
          int       transposeIdx1 = pClass[j + 2 * m + 8] & 0x3;
          const int filterIdx1    = classIndFixed[pClass[j + 2 * m + 8] >> 2];

          __m128i rawCoef00 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx0].data()));
          __m128i rawCoef01 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx0].data() + 6));
          __m128i rawCoef02 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx0].data() + 12));
          __m128i rawCoef03 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx0].data() + 20));
          __m128i rawCoef04 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx0].data() + 28));
          __m128i rawCoef05 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx0].data() + 34));
          __m128i rawCoef06 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx0].data() + 42));
          __m128i rawCoef07 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx0].data() + 50));
          __m128i rawCoef08 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx0].data() + 56));

          // transpose
          if (transposeIdx0 != 0)
          {
            const __m128i s00 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx0][0]);
            const __m128i s01 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx0][1]);
            const __m128i s02 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx0][2]);
            const __m128i s03 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx0][3]);
            const __m128i s04 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx0][4]);
            const __m128i s05 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx0][5]);
            const __m128i s06 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx0][6]);
            const __m128i s07 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx0][7]);
            const __m128i s08 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx0][8]);

            __m128i rawTmp[6];
            rawTmp[0] = rawCoef00;
            rawTmp[1] = rawCoef01;
            rawTmp[2] = _mm_shuffle_epi8(rawCoef02, s02);
            rawTmp[3] = _mm_shuffle_epi8(rawCoef03, s03);
            rawTmp[4] = _mm_shuffle_epi8(rawCoef04, s04);
            rawTmp[5] = _mm_shuffle_epi8(rawCoef05, s05);
            rawCoef00 = _mm_add_epi16(rawTmp[0], _mm_and_si128(s00, _mm_sub_epi16(rawTmp[1], rawTmp[0])));
            rawCoef01 = _mm_sub_epi16(rawTmp[1], _mm_and_si128(s00, _mm_sub_epi16(rawTmp[1], rawTmp[0])));
            rawCoef02 = _mm_add_epi16(rawTmp[2], _mm_and_si128(s01, _mm_sub_epi16(rawTmp[3], rawTmp[2])));
            rawCoef03 = _mm_sub_epi16(rawTmp[3], _mm_and_si128(s01, _mm_sub_epi16(rawTmp[3], rawTmp[2])));
            rawCoef04 = _mm_add_epi16(rawTmp[4], _mm_and_si128(s01, _mm_sub_epi16(rawTmp[5], rawTmp[4])));
            rawCoef05 = _mm_sub_epi16(rawTmp[5], _mm_and_si128(s01, _mm_sub_epi16(rawTmp[5], rawTmp[4])));
            rawCoef06 = _mm_shuffle_epi8(rawCoef06, s06);
            rawCoef07 = _mm_shuffle_epi8(rawCoef07, s07);
            rawCoef08 = _mm_shuffle_epi8(rawCoef08, s08);
          }

          __m128i rawCoef10 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx1].data()));
          __m128i rawCoef11 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx1].data() + 6));
          __m128i rawCoef12 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx1].data() + 12));
          __m128i rawCoef13 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx1].data() + 20));
          __m128i rawCoef14 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx1].data() + 28));
          __m128i rawCoef15 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx1].data() + 34));
          __m128i rawCoef16 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx1].data() + 42));
          __m128i rawCoef17 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx1].data() + 50));
          __m128i rawCoef18 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx1].data() + 56));

          // transpose
          if (transposeIdx1 != 0)
          {
            const __m128i s10 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx1][0]);
            const __m128i s11 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx1][1]);
            const __m128i s12 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx1][2]);
            const __m128i s13 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx1][3]);
            const __m128i s14 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx1][4]);
            const __m128i s15 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx1][5]);
            const __m128i s16 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx1][6]);
            const __m128i s17 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx1][7]);
            const __m128i s18 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx1][8]);

            __m128i rawTmp[6];
            rawTmp[0] = rawCoef10;
            rawTmp[1] = rawCoef11;
            rawTmp[2] = _mm_shuffle_epi8(rawCoef12, s12);
            rawTmp[3] = _mm_shuffle_epi8(rawCoef13, s13);
            rawTmp[4] = _mm_shuffle_epi8(rawCoef14, s14);
            rawTmp[5] = _mm_shuffle_epi8(rawCoef15, s15);
            rawCoef10 = _mm_add_epi16(rawTmp[0], _mm_and_si128(s10, _mm_sub_epi16(rawTmp[1], rawTmp[0])));
            rawCoef11 = _mm_sub_epi16(rawTmp[1], _mm_and_si128(s10, _mm_sub_epi16(rawTmp[1], rawTmp[0])));
            rawCoef12 = _mm_add_epi16(rawTmp[2], _mm_and_si128(s11, _mm_sub_epi16(rawTmp[3], rawTmp[2])));
            rawCoef13 = _mm_sub_epi16(rawTmp[3], _mm_and_si128(s11, _mm_sub_epi16(rawTmp[3], rawTmp[2])));
            rawCoef14 = _mm_add_epi16(rawTmp[4], _mm_and_si128(s11, _mm_sub_epi16(rawTmp[5], rawTmp[4])));
            rawCoef15 = _mm_sub_epi16(rawTmp[5], _mm_and_si128(s11, _mm_sub_epi16(rawTmp[5], rawTmp[4])));
            rawCoef16 = _mm_shuffle_epi8(rawCoef16, s16);
            rawCoef17 = _mm_shuffle_epi8(rawCoef17, s17);
            rawCoef18 = _mm_shuffle_epi8(rawCoef18, s18);
          }
          rawCoef[m][0] = _mm256_castsi128_si256(rawCoef00);
          rawCoef[m][0] = _mm256_insertf128_si256(rawCoef[m][0], rawCoef10, 1);
          rawCoef[m][1] = _mm256_castsi128_si256(rawCoef01);
          rawCoef[m][1] = _mm256_insertf128_si256(rawCoef[m][1], rawCoef11, 1);
          rawCoef[m][2] = _mm256_castsi128_si256(rawCoef02);
          rawCoef[m][2] = _mm256_insertf128_si256(rawCoef[m][2], rawCoef12, 1);
          rawCoef[m][3] = _mm256_castsi128_si256(rawCoef03);
          rawCoef[m][3] = _mm256_insertf128_si256(rawCoef[m][3], rawCoef13, 1);
          rawCoef[m][4] = _mm256_castsi128_si256(rawCoef04);
          rawCoef[m][4] = _mm256_insertf128_si256(rawCoef[m][4], rawCoef14, 1);
          rawCoef[m][5] = _mm256_castsi128_si256(rawCoef05);
          rawCoef[m][5] = _mm256_insertf128_si256(rawCoef[m][5], rawCoef15, 1);
          rawCoef[m][6] = _mm256_castsi128_si256(rawCoef06);
          rawCoef[m][6] = _mm256_insertf128_si256(rawCoef[m][6], rawCoef16, 1);
          rawCoef[m][7] = _mm256_castsi128_si256(rawCoef07);
          rawCoef[m][7] = _mm256_insertf128_si256(rawCoef[m][7], rawCoef17, 1);
          rawCoef[m][8] = _mm256_castsi128_si256(rawCoef08);
          rawCoef[m][8] = _mm256_insertf128_si256(rawCoef[m][8], rawCoef18, 1);
        }   // for(m)

        params[0] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]),
                                          _mm256_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
        params[1] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]),
                                          _mm256_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
        params[2] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][0], rawCoef[1][0]),
                                          _mm256_unpackhi_epi32(rawCoef[2][0], rawCoef[3][0]));

        params[3] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]),
                                          _mm256_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
        params[4] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]),
                                          _mm256_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
        params[5] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][1], rawCoef[1][1]),
                                          _mm256_unpackhi_epi32(rawCoef[2][1], rawCoef[3][1]));

        params[6] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]),
                                          _mm256_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
        params[7] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]),
                                          _mm256_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
        params[8] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][2], rawCoef[1][2]),
                                          _mm256_unpackhi_epi32(rawCoef[2][2], rawCoef[3][2]));
        params[9] = _mm256_unpackhi_epi64(_mm256_unpackhi_epi32(rawCoef[0][2], rawCoef[1][2]),
                                          _mm256_unpackhi_epi32(rawCoef[2][2], rawCoef[3][2]));

        params[10] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][3], rawCoef[1][3]),
                                           _mm256_unpacklo_epi32(rawCoef[2][3], rawCoef[3][3]));
        params[11] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][3], rawCoef[1][3]),
                                           _mm256_unpacklo_epi32(rawCoef[2][3], rawCoef[3][3]));
        params[12] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][3], rawCoef[1][3]),
                                           _mm256_unpackhi_epi32(rawCoef[2][3], rawCoef[3][3]));
        params[13] = _mm256_unpackhi_epi64(_mm256_unpackhi_epi32(rawCoef[0][3], rawCoef[1][3]),
                                           _mm256_unpackhi_epi32(rawCoef[2][3], rawCoef[3][3]));

        params[14] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][4], rawCoef[1][4]),
                                           _mm256_unpacklo_epi32(rawCoef[2][4], rawCoef[3][4]));
        params[15] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][4], rawCoef[1][4]),
                                           _mm256_unpacklo_epi32(rawCoef[2][4], rawCoef[3][4]));
        params[16] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][4], rawCoef[1][4]),
                                           _mm256_unpackhi_epi32(rawCoef[2][4], rawCoef[3][4]));

        params[17] = _mm256_unpackhi_epi64(_mm256_unpackhi_epi32(rawCoef[0][4], rawCoef[1][4]),
                                           _mm256_unpackhi_epi32(rawCoef[2][4], rawCoef[3][4]));

        params[18] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][5], rawCoef[1][5]),
                                           _mm256_unpacklo_epi32(rawCoef[2][5], rawCoef[3][5]));
        params[19] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][5], rawCoef[1][5]),
                                           _mm256_unpackhi_epi32(rawCoef[2][5], rawCoef[3][5]));
        params[20] = _mm256_unpackhi_epi64(_mm256_unpackhi_epi32(rawCoef[0][5], rawCoef[1][5]),
                                           _mm256_unpackhi_epi32(rawCoef[2][5], rawCoef[3][5]));

        params[21] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][6], rawCoef[1][6]),
                                           _mm256_unpacklo_epi32(rawCoef[2][6], rawCoef[3][6]));
        params[22] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][6], rawCoef[1][6]),
                                           _mm256_unpacklo_epi32(rawCoef[2][6], rawCoef[3][6]));
        params[23] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][6], rawCoef[1][6]),
                                           _mm256_unpackhi_epi32(rawCoef[2][6], rawCoef[3][6]));
        params[24] = _mm256_unpackhi_epi64(_mm256_unpackhi_epi32(rawCoef[0][6], rawCoef[1][6]),
                                           _mm256_unpackhi_epi32(rawCoef[2][6], rawCoef[3][6]));

        params[25] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][7], rawCoef[1][7]),
                                           _mm256_unpacklo_epi32(rawCoef[2][7], rawCoef[3][7]));
        params[26] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][7], rawCoef[1][7]),
                                           _mm256_unpacklo_epi32(rawCoef[2][7], rawCoef[3][7]));
        params[27] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][7], rawCoef[1][7]),
                                           _mm256_unpackhi_epi32(rawCoef[2][7], rawCoef[3][7]));

        params[28] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][8], rawCoef[1][8]),
                                           _mm256_unpacklo_epi32(rawCoef[2][8], rawCoef[3][8]));
        params[29] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][8], rawCoef[1][8]),
                                           _mm256_unpacklo_epi32(rawCoef[2][8], rawCoef[3][8]));
        params[30] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][8], rawCoef[1][8]),
                                           _mm256_unpackhi_epi32(rawCoef[2][8], rawCoef[3][8]));
        params[31] = _mm256_unpackhi_epi64(_mm256_unpackhi_epi32(rawCoef[0][8], rawCoef[1][8]),
                                           _mm256_unpackhi_epi32(rawCoef[2][8], rawCoef[3][8]));

        for (int ii = 0; ii < STEP_Y; ii++)
        {
          const Pel *pImg0Cur = src + j + ii * srcStride;

          const AdaptiveLoopFilterEcm::FilterRowPtrs<const Pel, 13> pImg(
            firstFixFiltRes + ii * firstFixFiltResStride + j, firstFixFiltResStride);

          const AdaptiveLoopFilterEcm::FilterRowPtrs<const Pel, 9> pImgBeforeDb(
            srcBeforeDb + ii * srcBeforeDbStride + j, srcBeforeDbStride);

          __m256i cur = _mm256_loadu_si256((const __m256i *)pImg0Cur);

          __m256i accumA = mmOffset;
          __m256i accumB = mmOffset;

          auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
          {
            const __m256i val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *)ptr0), cur);
            const __m256i val10 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *)ptr2), cur);
            const __m256i val01 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *)ptr1), cur);
            const __m256i val11 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *)ptr3), cur);

            __m256i val01A = _mm256_blend_epi16(val00, _mm256_slli_si256(val10, 2), 0xAA);
            __m256i val01B = _mm256_blend_epi16(_mm256_srli_si256(val00, 2), val10, 0xAA);
            __m256i val01C = _mm256_blend_epi16(val01, _mm256_slli_si256(val11, 2), 0xAA);
            __m256i val01D = _mm256_blend_epi16(_mm256_srli_si256(val01, 2), val11, 0xAA);

            __m256i mmClippingFixed = _mm256_and_si256(params[i], mm3);

            __m256i mmClippingFixed2 = _mm256_packs_epi16(mmClippingFixed, mmClippingFixed);
            mmClippingFixed2         = _mm256_add_epi8(mmClippingFixed2, mmClippingFixed2);
            __m256i xmm2             = _mm256_add_epi8(mmClippingFixed2, mm11);
            __m256i xmmA             = _mm256_unpacklo_epi8(mmClippingFixed2, xmm2);
            __m256i limit            = _mm256_shuffle_epi8(mmClippingValues256, xmmA);

            val01A = _mm256_min_epi16(val01A, limit);
            val01B = _mm256_min_epi16(val01B, limit);
            val01C = _mm256_min_epi16(val01C, limit);
            val01D = _mm256_min_epi16(val01D, limit);

            limit = _mm256_sub_epi16(_mm256_setzero_si256(), limit);

            val01A = _mm256_max_epi16(val01A, limit);
            val01B = _mm256_max_epi16(val01B, limit);
            val01C = _mm256_max_epi16(val01C, limit);
            val01D = _mm256_max_epi16(val01D, limit);

            val01A = _mm256_add_epi16(val01A, val01C);
            val01B = _mm256_add_epi16(val01B, val01D);

            const __m256i coeff = _mm256_srai_epi16(params[i], 2);

            accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff));
            accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff));
          };

          process2coeffs(0, pImg[+6] + 0, pImg[-6] + 0, pImg[+5] + 0, pImg[-5] - 0);
          process2coeffs(1, pImg[+4] + 0, pImg[-4] + 0, pImg[+3] - 0, pImg[-3] + 0);
          process2coeffs(2, pImg[+2] + 0, pImg[-2] - 0, pImg[+1] + 0, pImg[-1] - 0);

          process2coeffs(3, pImg[+0] + 6, pImg[-0] - 6, pImg[+0] + 5, pImg[-0] - 5);
          process2coeffs(4, pImg[+0] + 4, pImg[-0] - 4, pImg[+0] + 3, pImg[-0] - 3);
          process2coeffs(5, pImg[+0] + 2, pImg[-0] - 2, pImg[+0] + 1, pImg[-0] - 1);

          process2coeffs(6, pImg[+5] + 1, pImg[-5] - 1, pImg[+4] + 2, pImg[-4] - 2);
          process2coeffs(7, pImg[+3] + 3, pImg[-3] - 3, pImg[+2] + 4, pImg[-2] - 4);
          process2coeffs(8, pImg[+1] + 5, pImg[-1] - 5, pImg[+3] + 1, pImg[-3] - 1);
          process2coeffs(9, pImg[+2] + 2, pImg[-2] - 2, pImg[+1] + 3, pImg[-1] - 3);

          process2coeffs(10, pImg[+5] - 1, pImg[-5] + 1, pImg[+4] - 2, pImg[-4] + 2);
          process2coeffs(11, pImg[+3] - 3, pImg[-3] + 3, pImg[+2] - 4, pImg[-2] + 4);
          process2coeffs(12, pImg[+1] - 5, pImg[-1] + 5, pImg[+3] - 1, pImg[-3] + 1);
          process2coeffs(13, pImg[+2] - 2, pImg[-2] + 2, pImg[+1] - 3, pImg[-1] + 3);

          process2coeffs(14, pImg[+4] + 1, pImg[-4] - 1, pImg[+3] + 2, pImg[-3] - 2);
          process2coeffs(15, pImg[+2] + 3, pImg[-2] - 3, pImg[+1] + 4, pImg[-1] - 4);
          process2coeffs(16, pImg[+2] + 1, pImg[-2] - 1, pImg[+1] + 2, pImg[-1] - 2);

          process2coeffs(17, pImg[+1] + 1, pImg[-1] - 1, pImg[+1] - 1, pImg[-1] + 1);

          process2coeffs(18, pImg[+4] - 1, pImg[-4] + 1, pImg[+3] - 2, pImg[-3] + 2);
          process2coeffs(19, pImg[+2] - 3, pImg[-2] + 3, pImg[+1] - 4, pImg[-1] + 4);
          process2coeffs(20, pImg[+2] - 1, pImg[-2] + 1, pImg[+1] - 2, pImg[-1] + 2);

          process2coeffs(21, pImgBeforeDb[+4] + 0, pImgBeforeDb[-4] + 0, pImgBeforeDb[+3] + 0, pImgBeforeDb[-3] - 0);
          process2coeffs(22, pImgBeforeDb[+2] + 0, pImgBeforeDb[-2] + 0, pImgBeforeDb[+1] - 0, pImgBeforeDb[-1] + 0);
          process2coeffs(23, pImgBeforeDb[+0] + 4, pImgBeforeDb[+0] - 4, pImgBeforeDb[+0] + 3, pImgBeforeDb[+0] - 3);
          process2coeffs(24, pImgBeforeDb[+0] - 2, pImgBeforeDb[+0] + 2, pImgBeforeDb[+0] - 1, pImgBeforeDb[+0] + 1);
          process2coeffs(25, pImgBeforeDb[-3] - 1, pImgBeforeDb[+3] + 1, pImgBeforeDb[-2] - 2, pImgBeforeDb[+2] + 2);
          process2coeffs(26, pImgBeforeDb[-1] - 3, pImgBeforeDb[+1] + 3, pImgBeforeDb[-3] + 1, pImgBeforeDb[+3] - 1);
          process2coeffs(27, pImgBeforeDb[-2] + 2, pImgBeforeDb[+2] - 2, pImgBeforeDb[-1] + 3, pImgBeforeDb[+1] - 3);
          process2coeffs(28, pImgBeforeDb[-2] - 1, pImgBeforeDb[+2] + 1, pImgBeforeDb[-1] - 2, pImgBeforeDb[+1] + 2);
          process2coeffs(29, pImgBeforeDb[-1] - 1, pImgBeforeDb[+1] + 1, pImgBeforeDb[-2] + 1, pImgBeforeDb[+2] - 1);
          process2coeffs(30, pImgBeforeDb[-1] + 2, pImgBeforeDb[+1] - 2, pImgBeforeDb[-1] + 1, pImgBeforeDb[+1] - 1);

          process2coeffs(31, pImgBeforeDb[0], pImg0Cur, pImg[0], pImg0Cur);

          accumA = _mm256_srai_epi32(accumA, SHIFT);
          accumB = _mm256_srai_epi32(accumB, SHIFT);

          accumA = _mm256_blend_epi16(accumA, _mm256_slli_si256(accumB, 2), 0xAA);
          accumA = _mm256_add_epi16(accumA, cur);
          accumA = _mm256_min_epi16(mmMax, _mm256_max_epi16(accumA, mmMin));

          _mm256_storeu_si256((__m256i *)(&(fixedFilterResult[blkDst.y + i + ii][blkDst.x + j])), accumA);
        }   // for (size_t ii = 0; ii < STEP_Y; ii++)
      }   // for (size_t j = 0; j < width; j += STEP_X*2)

      src += srcStride2;
      srcBeforeDb += srcBeforeDbStride2;
      firstFixFiltRes += firstFixFiltResStride2;
    }
  }
  else
  {
#endif
    const __m128i mmOffset         = _mm_set1_epi32(ROUND);
    const __m128i mmMin            = _mm_set1_epi16(clpRng.min);
    const __m128i mmMax            = _mm_set1_epi16(clpRng.max);
    const __m128i mmClippingValues = _mm_loadl_epi64((const __m128i *)clippingValues.data());
    const __m128i mm11             = _mm_set1_epi8(1);
    const __m128i mm3              = _mm_set1_epi16(3);

    for (int i = 0; i < height; i += STEP_Y)
    {
      const AdaptiveLoopFilterEcm::AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;

      for (int j = 0; j < width; j += STEP_X)
      {
        __m128i params[32];
        __m128i rawCoef[4][9];

        for (int m = 0; m < 4; m++)
        {
          int       transposeIdx = pClass[j + 2 * m] & 0x3;
          const int filterIdx    = classIndFixed[pClass[j + 2 * m] >> 2];

          rawCoef[m][0] = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx].data()));
          rawCoef[m][1] = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx].data() + 6));
          rawCoef[m][2] = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx].data() + 12));
          rawCoef[m][3] = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx].data() + 20));
          rawCoef[m][4] = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx].data() + 28));
          rawCoef[m][5] = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx].data() + 34));
          rawCoef[m][6] = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx].data() + 42));
          rawCoef[m][7] = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx].data() + 50));
          rawCoef[m][8] = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx].data() + 56));

          if (transposeIdx != 0)
          // transpose
          {
            const __m128i s0 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx][0]);
            const __m128i s1 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx][1]);
            const __m128i s2 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx][2]);
            const __m128i s3 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx][3]);
            const __m128i s4 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx][4]);
            const __m128i s5 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx][5]);
            const __m128i s6 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx][6]);
            const __m128i s7 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx][7]);
            const __m128i s8 = _mm_loadu_si128((const __m128i *)shTab[transposeIdx][8]);

            __m128i rawTmp[6];
            rawTmp[0] = rawCoef[m][0];
            rawTmp[1] = rawCoef[m][1];
            rawTmp[2] = _mm_shuffle_epi8(rawCoef[m][2], s2);
            rawTmp[3] = _mm_shuffle_epi8(rawCoef[m][3], s3);
            rawTmp[4] = _mm_shuffle_epi8(rawCoef[m][4], s4);
            rawTmp[5] = _mm_shuffle_epi8(rawCoef[m][5], s5);

            rawCoef[m][6] = _mm_shuffle_epi8(rawCoef[m][6], s6);
            rawCoef[m][7] = _mm_shuffle_epi8(rawCoef[m][7], s7);
            rawCoef[m][8] = _mm_shuffle_epi8(rawCoef[m][8], s8);

            rawCoef[m][0] = _mm_add_epi16(rawTmp[0], _mm_and_si128(s0, _mm_sub_epi16(rawTmp[1], rawTmp[0])));
            rawCoef[m][1] = _mm_sub_epi16(rawTmp[1], _mm_and_si128(s0, _mm_sub_epi16(rawTmp[1], rawTmp[0])));
            rawCoef[m][2] = _mm_add_epi16(rawTmp[2], _mm_and_si128(s1, _mm_sub_epi16(rawTmp[3], rawTmp[2])));
            rawCoef[m][3] = _mm_sub_epi16(rawTmp[3], _mm_and_si128(s1, _mm_sub_epi16(rawTmp[3], rawTmp[2])));
            rawCoef[m][4] = _mm_add_epi16(rawTmp[4], _mm_and_si128(s1, _mm_sub_epi16(rawTmp[5], rawTmp[4])));
            rawCoef[m][5] = _mm_sub_epi16(rawTmp[5], _mm_and_si128(s1, _mm_sub_epi16(rawTmp[5], rawTmp[4])));
          }
        }   // for(m)

        params[0] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]),
                                       _mm_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
        params[1] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]),
                                       _mm_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
        params[2] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][0], rawCoef[1][0]),
                                       _mm_unpackhi_epi32(rawCoef[2][0], rawCoef[3][0]));

        params[3] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]),
                                       _mm_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
        params[4] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]),
                                       _mm_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
        params[5] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][1], rawCoef[1][1]),
                                       _mm_unpackhi_epi32(rawCoef[2][1], rawCoef[3][1]));

        params[6] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]),
                                       _mm_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
        params[7] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]),
                                       _mm_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
        params[8] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][2], rawCoef[1][2]),
                                       _mm_unpackhi_epi32(rawCoef[2][2], rawCoef[3][2]));
        params[9] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][2], rawCoef[1][2]),
                                       _mm_unpackhi_epi32(rawCoef[2][2], rawCoef[3][2]));

        params[10] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][3], rawCoef[1][3]),
                                        _mm_unpacklo_epi32(rawCoef[2][3], rawCoef[3][3]));
        params[11] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][3], rawCoef[1][3]),
                                        _mm_unpacklo_epi32(rawCoef[2][3], rawCoef[3][3]));
        params[12] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][3], rawCoef[1][3]),
                                        _mm_unpackhi_epi32(rawCoef[2][3], rawCoef[3][3]));
        params[13] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][3], rawCoef[1][3]),
                                        _mm_unpackhi_epi32(rawCoef[2][3], rawCoef[3][3]));

        params[14] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][4], rawCoef[1][4]),
                                        _mm_unpacklo_epi32(rawCoef[2][4], rawCoef[3][4]));
        params[15] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][4], rawCoef[1][4]),
                                        _mm_unpacklo_epi32(rawCoef[2][4], rawCoef[3][4]));
        params[16] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][4], rawCoef[1][4]),
                                        _mm_unpackhi_epi32(rawCoef[2][4], rawCoef[3][4]));

        params[17] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][4], rawCoef[1][4]),
                                        _mm_unpackhi_epi32(rawCoef[2][4], rawCoef[3][4]));

        params[18] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][5], rawCoef[1][5]),
                                        _mm_unpacklo_epi32(rawCoef[2][5], rawCoef[3][5]));
        params[19] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][5], rawCoef[1][5]),
                                        _mm_unpackhi_epi32(rawCoef[2][5], rawCoef[3][5]));
        params[20] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][5], rawCoef[1][5]),
                                        _mm_unpackhi_epi32(rawCoef[2][5], rawCoef[3][5]));

        params[21] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][6], rawCoef[1][6]),
                                        _mm_unpacklo_epi32(rawCoef[2][6], rawCoef[3][6]));
        params[22] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][6], rawCoef[1][6]),
                                        _mm_unpacklo_epi32(rawCoef[2][6], rawCoef[3][6]));
        params[23] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][6], rawCoef[1][6]),
                                        _mm_unpackhi_epi32(rawCoef[2][6], rawCoef[3][6]));
        params[24] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][6], rawCoef[1][6]),
                                        _mm_unpackhi_epi32(rawCoef[2][6], rawCoef[3][6]));

        params[25] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][7], rawCoef[1][7]),
                                        _mm_unpacklo_epi32(rawCoef[2][7], rawCoef[3][7]));
        params[26] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][7], rawCoef[1][7]),
                                        _mm_unpacklo_epi32(rawCoef[2][7], rawCoef[3][7]));
        params[27] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][7], rawCoef[1][7]),
                                        _mm_unpackhi_epi32(rawCoef[2][7], rawCoef[3][7]));

        params[28] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][8], rawCoef[1][8]),
                                        _mm_unpacklo_epi32(rawCoef[2][8], rawCoef[3][8]));
        params[29] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][8], rawCoef[1][8]),
                                        _mm_unpacklo_epi32(rawCoef[2][8], rawCoef[3][8]));
        params[30] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][8], rawCoef[1][8]),
                                        _mm_unpackhi_epi32(rawCoef[2][8], rawCoef[3][8]));
        params[31] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][8], rawCoef[1][8]),
                                        _mm_unpackhi_epi32(rawCoef[2][8], rawCoef[3][8]));

        for (int ii = 0; ii < STEP_Y; ii++)
        {
          const Pel *pImg0Cur = src + j + ii * srcStride;

          const AdaptiveLoopFilterEcm::FilterRowPtrs<const Pel, 13> pImg(
            firstFixFiltRes + ii * firstFixFiltResStride + j, firstFixFiltResStride);

          const AdaptiveLoopFilterEcm::FilterRowPtrs<const Pel, 9> pImgBeforeDb(
            srcBeforeDb + ii * srcBeforeDbStride + j, srcBeforeDbStride);

          __m128i cur = _mm_loadu_si128((const __m128i *)pImg0Cur);

          __m128i accumA = mmOffset;
          __m128i accumB = mmOffset;

          auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
          {
            const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr0), cur);
            const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr2), cur);
            const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr1), cur);
            const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr3), cur);

            __m128i val01A = _mm_blend_epi16(val00, _mm_slli_si128(val10, 2), 0xAA);
            __m128i val01B = _mm_blend_epi16(_mm_srli_si128(val00, 2), val10, 0xAA);
            __m128i val01C = _mm_blend_epi16(val01, _mm_slli_si128(val11, 2), 0xAA);
            __m128i val01D = _mm_blend_epi16(_mm_srli_si128(val01, 2), val11, 0xAA);

            __m128i mmClippingFixed = _mm_and_si128(params[i], mm3);

            __m128i mmClippingFixed2 = _mm_packs_epi16(mmClippingFixed, mmClippingFixed);
            mmClippingFixed2         = _mm_add_epi8(mmClippingFixed2, mmClippingFixed2);
            __m128i xmm2             = _mm_add_epi8(mmClippingFixed2, mm11);
            __m128i xmmA             = _mm_unpacklo_epi8(mmClippingFixed2, xmm2);
            __m128i limit            = _mm_shuffle_epi8(mmClippingValues, xmmA);

            val01A = _mm_min_epi16(val01A, limit);
            val01B = _mm_min_epi16(val01B, limit);
            val01C = _mm_min_epi16(val01C, limit);
            val01D = _mm_min_epi16(val01D, limit);

            limit = _mm_sub_epi16(_mm_setzero_si128(), limit);

            val01A = _mm_max_epi16(val01A, limit);
            val01B = _mm_max_epi16(val01B, limit);
            val01C = _mm_max_epi16(val01C, limit);
            val01D = _mm_max_epi16(val01D, limit);

            val01A = _mm_add_epi16(val01A, val01C);
            val01B = _mm_add_epi16(val01B, val01D);

            const __m128i coeff = _mm_srai_epi16(params[i], 2);

            accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff));
            accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff));
          };

          process2coeffs(0, pImg[+6] + 0, pImg[-6] + 0, pImg[+5] + 0, pImg[-5] - 0);
          process2coeffs(1, pImg[+4] + 0, pImg[-4] + 0, pImg[+3] - 0, pImg[-3] + 0);
          process2coeffs(2, pImg[+2] + 0, pImg[-2] - 0, pImg[+1] + 0, pImg[-1] - 0);

          process2coeffs(3, pImg[+0] + 6, pImg[-0] - 6, pImg[+0] + 5, pImg[-0] - 5);
          process2coeffs(4, pImg[+0] + 4, pImg[-0] - 4, pImg[+0] + 3, pImg[-0] - 3);
          process2coeffs(5, pImg[+0] + 2, pImg[-0] - 2, pImg[+0] + 1, pImg[-0] - 1);

          process2coeffs(6, pImg[+5] + 1, pImg[-5] - 1, pImg[+4] + 2, pImg[-4] - 2);
          process2coeffs(7, pImg[+3] + 3, pImg[-3] - 3, pImg[+2] + 4, pImg[-2] - 4);
          process2coeffs(8, pImg[+1] + 5, pImg[-1] - 5, pImg[+3] + 1, pImg[-3] - 1);
          process2coeffs(9, pImg[+2] + 2, pImg[-2] - 2, pImg[+1] + 3, pImg[-1] - 3);

          process2coeffs(10, pImg[+5] - 1, pImg[-5] + 1, pImg[+4] - 2, pImg[-4] + 2);
          process2coeffs(11, pImg[+3] - 3, pImg[-3] + 3, pImg[+2] - 4, pImg[-2] + 4);
          process2coeffs(12, pImg[+1] - 5, pImg[-1] + 5, pImg[+3] - 1, pImg[-3] + 1);
          process2coeffs(13, pImg[+2] - 2, pImg[-2] + 2, pImg[+1] - 3, pImg[-1] + 3);

          process2coeffs(14, pImg[+4] + 1, pImg[-4] - 1, pImg[+3] + 2, pImg[-3] - 2);
          process2coeffs(15, pImg[+2] + 3, pImg[-2] - 3, pImg[+1] + 4, pImg[-1] - 4);
          process2coeffs(16, pImg[+2] + 1, pImg[-2] - 1, pImg[+1] + 2, pImg[-1] - 2);

          process2coeffs(17, pImg[+1] + 1, pImg[-1] - 1, pImg[+1] - 1, pImg[-1] + 1);

          process2coeffs(18, pImg[+4] - 1, pImg[-4] + 1, pImg[+3] - 2, pImg[-3] + 2);
          process2coeffs(19, pImg[+2] - 3, pImg[-2] + 3, pImg[+1] - 4, pImg[-1] + 4);
          process2coeffs(20, pImg[+2] - 1, pImg[-2] + 1, pImg[+1] - 2, pImg[-1] + 2);

          process2coeffs(21, pImgBeforeDb[+4] + 0, pImgBeforeDb[-4] + 0, pImgBeforeDb[+3] + 0, pImgBeforeDb[-3] - 0);
          process2coeffs(22, pImgBeforeDb[+2] + 0, pImgBeforeDb[-2] + 0, pImgBeforeDb[+1] - 0, pImgBeforeDb[-1] + 0);
          process2coeffs(23, pImgBeforeDb[+0] + 4, pImgBeforeDb[+0] - 4, pImgBeforeDb[+0] + 3, pImgBeforeDb[+0] - 3);
          process2coeffs(24, pImgBeforeDb[+0] - 2, pImgBeforeDb[+0] + 2, pImgBeforeDb[+0] - 1, pImgBeforeDb[+0] + 1);
          process2coeffs(25, pImgBeforeDb[-3] - 1, pImgBeforeDb[+3] + 1, pImgBeforeDb[-2] - 2, pImgBeforeDb[+2] + 2);
          process2coeffs(26, pImgBeforeDb[-1] - 3, pImgBeforeDb[+1] + 3, pImgBeforeDb[-3] + 1, pImgBeforeDb[+3] - 1);
          process2coeffs(27, pImgBeforeDb[-2] + 2, pImgBeforeDb[+2] - 2, pImgBeforeDb[-1] + 3, pImgBeforeDb[+1] - 3);
          process2coeffs(28, pImgBeforeDb[-2] - 1, pImgBeforeDb[+2] + 1, pImgBeforeDb[-1] - 2, pImgBeforeDb[+1] + 2);
          process2coeffs(29, pImgBeforeDb[-1] - 1, pImgBeforeDb[+1] + 1, pImgBeforeDb[-2] + 1, pImgBeforeDb[+2] - 1);
          process2coeffs(30, pImgBeforeDb[-1] + 2, pImgBeforeDb[+1] - 2, pImgBeforeDb[-1] + 1, pImgBeforeDb[+1] - 1);

          process2coeffs(31, pImgBeforeDb[0], pImg0Cur, pImg[0], pImg0Cur);

          accumA = _mm_srai_epi32(accumA, SHIFT);
          accumB = _mm_srai_epi32(accumB, SHIFT);

          accumA = _mm_blend_epi16(accumA, _mm_slli_si128(accumB, 2), 0xAA);
          accumA = _mm_add_epi16(accumA, cur);
          accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

          _mm_storeu_si128((__m128i *)(&(fixedFilterResult[blkDst.y + i + ii][blkDst.x + j])), accumA);
        }   // for (size_t ii = 0; ii < STEP_Y; ii++)
      }   // for (size_t j = 0; j < width; j += STEP_X)

      src += srcStride2;
      srcBeforeDb += srcBeforeDbStride2;
      firstFixFiltRes += firstFixFiltResStride2;
    }
#if USE_AVX2
  }
#endif
}

class FixFilterPackedData9Db9 : public FixFilterPackedData<AdaptiveLoopFilterEcm::FIX_FILTER_NUM_COEFF_9_DB_9>
{
public:
  class P0 : public std::array<int, AdaptiveLoopFilterEcm::FIX_FILTER_NUM_COEFF_9_DB_9>
  {
  public:
    constexpr P0();
  };

  FixFilterPackedData9Db9(const FixFilterPackedData9Db9 &) = delete;
  FixFilterPackedData9Db9(FixFilterPackedData9Db9 &&)      = delete;

  FixFilterPackedData9Db9 &operator=(const FixFilterPackedData9Db9 &) = delete;
  FixFilterPackedData9Db9 &operator=(FixFilterPackedData9Db9 &&)      = delete;

  // TODO: Could be constexpr if constructor was constexpr.
  static inline const FixFilterPackedData9Db9 &get();

private:
  // TODO: Could be constexpr but maximum number of steps is exceeded.
  inline FixFilterPackedData9Db9();
};

constexpr FixFilterPackedData9Db9::P0::P0()
  : std::array<int, AdaptiveLoopFilterEcm::FIX_FILTER_NUM_COEFF_9_DB_9> {
    // clang-format off
     0,  1,  4,  9, 16,  3,  8, 15,
     2,  5, 10, 17,  7, 14,
     6, 11, 18, 13, 12, 19,
    20, 21, 24, 29, 36, 23, 28, 35,
    22, 25, 30, 37, 27, 34,
    26, 31, 38, 33, 32, 39, 40
    // clang-format on
  }
{}

FixFilterPackedData9Db9::FixFilterPackedData9Db9()
  : FixFilterPackedData<AdaptiveLoopFilterEcm::FIX_FILTER_NUM_COEFF_9_DB_9>()
{
  constexpr P0 p0Filter9Db9;
  auto        &packedDataFixedFilters9Db9 = *this;
  for (int i = 0; i < AdaptiveLoopFilterEcm::NUM_FIXED_FILTER_SETS; ++i)
  {
    for (int j = 0; j < AdaptiveLoopFilterEcm::NUM_FIXED_FILTERS_PER_SET; ++j)
    {
      for (int k = 0; k < AdaptiveLoopFilterEcm::FIX_FILTER_NUM_COEFF_9_DB_9; ++k)
      {
        packedDataFixedFilters9Db9[i][j][k] =
          (m_filterCoeffFixed9Db9[i][j][p0Filter9Db9[k]] << 2) | m_clippingFixed9Db9[i][j][p0Filter9Db9[k]];
      }
    }
  }
}

const FixFilterPackedData9Db9 &FixFilterPackedData9Db9::get()
{
  static const FixFilterPackedData9Db9 obj;
  return obj;
}

// TODO: Could theoretically be constexpr. See FixFilterPackedData9Db9.
static const FixFilterPackedData9Db9 &packedDataFixedFilters9Db9 = FixFilterPackedData9Db9::get();

static const int8_t shTab9Db9[4][3][16] = {
  // clang-format off
  {
    { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 },
    { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15 }
  },
  {
    { 8,  9,  6,  7,  4,  5,  2,  3,  0,  1, 14, 15, 12, 13, 10, 11 },
    { 6,  7,  4,  5,  2,  3,  0,  1, 10, 11,  8,  9, 12, 13, 14, 15 },
    { 4,  5,  2,  3,  0,  1,  6,  7, 10, 11,  8,  9, 12, 13, 14, 15 }
  },
  {
    { 0,  1, 10, 11, 12, 13, 14, 15,  8,  9,  2,  3,  4,  5,  6,  7 },
    { 0,  1,  8,  9, 10, 11,  6,  7,  2,  3,  4,  5, 12, 13, 14, 15 },
    { 0,  1,  6,  7,  4,  5,  2,  3,  8,  9, 10, 11, 12, 13, 14, 15 }
  },
  {
    { 8,  9, 14, 15, 12, 13, 10, 11,  0,  1,  6,  7,  4,  5,  2,  3 },
    { 6,  7, 10, 11,  8,  9,  0,  1,  4,  5,  2,  3, 12, 13, 14, 15 },
    { 4,  5,  6,  7,  0,  1,  2,  3, 10, 11,  8,  9, 12, 13, 14, 15 },
  }
  // clang-format on
};

template<X86_VEXT vext>
static void simdFixFilter9x9Db9Blk(const AdaptiveLoopFilterEcm::ClassBuf &classifier, const CPelBuf &srcLuma,
                                   const CPelBuf &srcLumaBeforeDb, const Area &curBlk,
                                   AdaptiveLoopFilterEcm::CompBuf &fixedFilterResult, const Area &blkDst, int picWidth,
                                   const unsigned fixedFilterSetIdx, const ClpRng &clpRng,
                                   const std::array<Pel, AdaptiveLoopFilterEcm::MAX_ALF_NUM_CLIP_VALS> &clippingValues)
{
  constexpr int SHIFT = AdaptiveLoopFilterEcm::FIX_FILTER_COEFF_SCALE_BITS;
  constexpr int ROUND = 1 << (SHIFT - 1);

  constexpr int STEP_X = 8;
  constexpr int STEP_Y = 2;

  if (!(curBlk.width % STEP_X == 0) || !(curBlk.height % STEP_Y == 0))
  {
    AdaptiveLoopFilterEcm::fixedFilter9x9Db9Blk(classifier, srcLuma, srcLumaBeforeDb, curBlk, fixedFilterResult, blkDst,
                                                picWidth, fixedFilterSetIdx, clpRng, clippingValues);
    return;
  }

  const int width  = curBlk.width;
  const int height = curBlk.height;

  const ptrdiff_t srcStride          = srcLuma.stride;
  const ptrdiff_t srcBeforeDbStride  = srcLumaBeforeDb.stride;
  const ptrdiff_t srcStride2         = srcStride * STEP_Y;
  const ptrdiff_t srcBeforeDbStride2 = srcBeforeDbStride * STEP_Y;

  const Pel *src         = srcLuma.buf + curBlk.y * srcStride + curBlk.x;
  const Pel *srcBeforeDb = srcLumaBeforeDb.buf + curBlk.y * srcBeforeDbStride + curBlk.x;

  const auto &filterCoeffFixed = packedDataFixedFilters9Db9[fixedFilterSetIdx];
  const auto &classIndFixed    = m_classIdnFixedFilter9Db9[fixedFilterSetIdx];

#if USE_AVX2
  if (vext >= AVX2 && (width % 16) == 0)
  {
    const __m256i mmOffset            = _mm256_set1_epi32(ROUND);
    const __m256i mmMin               = _mm256_set1_epi16(clpRng.min);
    const __m256i mmMax               = _mm256_set1_epi16(clpRng.max);
    const __m128i mmClippingValues    = _mm_loadl_epi64((const __m128i *)clippingValues.data());
    __m256i       mmClippingValues256 = _mm256_castsi128_si256(mmClippingValues);
    mmClippingValues256               = _mm256_insertf128_si256(mmClippingValues256, mmClippingValues, 1);
    const __m256i mm11                = _mm256_set1_epi8(1);
    const __m256i mm3                 = _mm256_set1_epi16(3);
    for (int i = 0; i < height; i += STEP_Y)
    {
      const AdaptiveLoopFilterEcm::AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;

      for (int j = 0; j < width; j += STEP_X * 2)
      {
        __m256i params[21];
        __m256i rawCoef[4][6];

        for (int m = 0; m < 4; m++)
        {
          int       transposeIdx0 = pClass[j + 2 * m] & 0x3;
          const int filterIdx0    = classIndFixed[pClass[j + 2 * m] >> 2];
          int       transposeIdx1 = pClass[j + 2 * m + 8] & 0x3;
          const int filterIdx1    = classIndFixed[pClass[j + 2 * m + 8] >> 2];

          __m128i rawCoef00 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx0].data()));
          __m128i rawCoef01 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx0].data() + 8));
          __m128i rawCoef02 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx0].data() + 14));
          __m128i rawCoef03 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx0].data() + 20));
          __m128i rawCoef04 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx0].data() + 28));
          __m128i rawCoef05 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx0].data() + 34));

          // transpose
          if (transposeIdx0 != 0)
          {
            const __m128i s00 = _mm_loadu_si128((const __m128i *)shTab9Db9[transposeIdx0][0]);
            const __m128i s01 = _mm_loadu_si128((const __m128i *)shTab9Db9[transposeIdx0][1]);
            const __m128i s02 = _mm_loadu_si128((const __m128i *)shTab9Db9[transposeIdx0][2]);

            rawCoef00 = _mm_shuffle_epi8(rawCoef00, s00);
            rawCoef01 = _mm_shuffle_epi8(rawCoef01, s01);
            rawCoef02 = _mm_shuffle_epi8(rawCoef02, s02);
            rawCoef03 = _mm_shuffle_epi8(rawCoef03, s00);
            rawCoef04 = _mm_shuffle_epi8(rawCoef04, s01);
            rawCoef05 = _mm_shuffle_epi8(rawCoef05, s02);
          }

          __m128i rawCoef10 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx1].data()));
          __m128i rawCoef11 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx1].data() + 8));
          __m128i rawCoef12 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx1].data() + 14));
          __m128i rawCoef13 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx1].data() + 20));
          __m128i rawCoef14 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx1].data() + 28));
          __m128i rawCoef15 = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx1].data() + 34));

          // transpose
          if (transposeIdx1 != 0)
          {
            const __m128i s10 = _mm_loadu_si128((const __m128i *)shTab9Db9[transposeIdx1][0]);
            const __m128i s11 = _mm_loadu_si128((const __m128i *)shTab9Db9[transposeIdx1][1]);
            const __m128i s12 = _mm_loadu_si128((const __m128i *)shTab9Db9[transposeIdx1][2]);

            rawCoef10 = _mm_shuffle_epi8(rawCoef10, s10);
            rawCoef11 = _mm_shuffle_epi8(rawCoef11, s11);
            rawCoef12 = _mm_shuffle_epi8(rawCoef12, s12);
            rawCoef13 = _mm_shuffle_epi8(rawCoef13, s10);
            rawCoef14 = _mm_shuffle_epi8(rawCoef14, s11);
            rawCoef15 = _mm_shuffle_epi8(rawCoef15, s12);
          }
          rawCoef[m][0] = _mm256_castsi128_si256(rawCoef00);
          rawCoef[m][0] = _mm256_insertf128_si256(rawCoef[m][0], rawCoef10, 1);
          rawCoef[m][1] = _mm256_castsi128_si256(rawCoef01);
          rawCoef[m][1] = _mm256_insertf128_si256(rawCoef[m][1], rawCoef11, 1);
          rawCoef[m][2] = _mm256_castsi128_si256(rawCoef02);
          rawCoef[m][2] = _mm256_insertf128_si256(rawCoef[m][2], rawCoef12, 1);
          rawCoef[m][3] = _mm256_castsi128_si256(rawCoef03);
          rawCoef[m][3] = _mm256_insertf128_si256(rawCoef[m][3], rawCoef13, 1);
          rawCoef[m][4] = _mm256_castsi128_si256(rawCoef04);
          rawCoef[m][4] = _mm256_insertf128_si256(rawCoef[m][4], rawCoef14, 1);
          rawCoef[m][5] = _mm256_castsi128_si256(rawCoef05);
          rawCoef[m][5] = _mm256_insertf128_si256(rawCoef[m][5], rawCoef15, 1);
        }   // for(m)

        params[0] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]),
                                          _mm256_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
        params[1] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]),
                                          _mm256_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
        params[2] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][0], rawCoef[1][0]),
                                          _mm256_unpackhi_epi32(rawCoef[2][0], rawCoef[3][0]));
        params[3] = _mm256_unpackhi_epi64(_mm256_unpackhi_epi32(rawCoef[0][0], rawCoef[1][0]),
                                          _mm256_unpackhi_epi32(rawCoef[2][0], rawCoef[3][0]));

        params[4] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]),
                                          _mm256_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
        params[5] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]),
                                          _mm256_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
        params[6] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][1], rawCoef[1][1]),
                                          _mm256_unpackhi_epi32(rawCoef[2][1], rawCoef[3][1]));

        params[7] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]),
                                          _mm256_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
        params[8] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]),
                                          _mm256_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
        params[9] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][2], rawCoef[1][2]),
                                          _mm256_unpackhi_epi32(rawCoef[2][2], rawCoef[3][2]));

        params[10] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][3], rawCoef[1][3]),
                                           _mm256_unpacklo_epi32(rawCoef[2][3], rawCoef[3][3]));
        params[11] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][3], rawCoef[1][3]),
                                           _mm256_unpacklo_epi32(rawCoef[2][3], rawCoef[3][3]));
        params[12] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][3], rawCoef[1][3]),
                                           _mm256_unpackhi_epi32(rawCoef[2][3], rawCoef[3][3]));
        params[13] = _mm256_unpackhi_epi64(_mm256_unpackhi_epi32(rawCoef[0][3], rawCoef[1][3]),
                                           _mm256_unpackhi_epi32(rawCoef[2][3], rawCoef[3][3]));

        params[14] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][4], rawCoef[1][4]),
                                           _mm256_unpacklo_epi32(rawCoef[2][4], rawCoef[3][4]));
        params[15] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][4], rawCoef[1][4]),
                                           _mm256_unpacklo_epi32(rawCoef[2][4], rawCoef[3][4]));
        params[16] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][4], rawCoef[1][4]),
                                           _mm256_unpackhi_epi32(rawCoef[2][4], rawCoef[3][4]));

        params[17] = _mm256_unpacklo_epi64(_mm256_unpacklo_epi32(rawCoef[0][5], rawCoef[1][5]),
                                           _mm256_unpacklo_epi32(rawCoef[2][5], rawCoef[3][5]));
        params[18] = _mm256_unpackhi_epi64(_mm256_unpacklo_epi32(rawCoef[0][5], rawCoef[1][5]),
                                           _mm256_unpacklo_epi32(rawCoef[2][5], rawCoef[3][5]));
        params[19] = _mm256_unpacklo_epi64(_mm256_unpackhi_epi32(rawCoef[0][5], rawCoef[1][5]),
                                           _mm256_unpackhi_epi32(rawCoef[2][5], rawCoef[3][5]));
        params[20] = _mm256_unpackhi_epi64(_mm256_unpackhi_epi32(rawCoef[0][5], rawCoef[1][5]),
                                           _mm256_unpackhi_epi32(rawCoef[2][5], rawCoef[3][5]));

        for (int ii = 0; ii < STEP_Y; ii++)
        {
          const AdaptiveLoopFilterEcm::FilterRowPtrs<const Pel, 9> pImg(src + ii * srcStride + j, srcStride);
          const AdaptiveLoopFilterEcm::FilterRowPtrs<const Pel, 9> pImgBeforeDb(
            srcBeforeDb + ii * srcBeforeDbStride + j, srcBeforeDbStride);

          __m256i cur = _mm256_loadu_si256((const __m256i *)pImg[0]);

          __m256i accumA = mmOffset;
          __m256i accumB = mmOffset;

          auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
          {
            const __m256i val00 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *)ptr0), cur);
            const __m256i val10 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *)ptr2), cur);
            const __m256i val01 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *)ptr1), cur);
            const __m256i val11 = _mm256_sub_epi16(_mm256_loadu_si256((const __m256i *)ptr3), cur);

            __m256i val01A = _mm256_blend_epi16(val00, _mm256_slli_si256(val10, 2), 0xAA);
            __m256i val01B = _mm256_blend_epi16(_mm256_srli_si256(val00, 2), val10, 0xAA);
            __m256i val01C = _mm256_blend_epi16(val01, _mm256_slli_si256(val11, 2), 0xAA);
            __m256i val01D = _mm256_blend_epi16(_mm256_srli_si256(val01, 2), val11, 0xAA);

            __m256i mmClippingFixed = _mm256_and_si256(params[i], mm3);

            __m256i mmClippingFixed2 = _mm256_packs_epi16(mmClippingFixed, mmClippingFixed);
            mmClippingFixed2         = _mm256_add_epi8(mmClippingFixed2, mmClippingFixed2);
            __m256i xmm2             = _mm256_add_epi8(mmClippingFixed2, mm11);
            __m256i xmmA             = _mm256_unpacklo_epi8(mmClippingFixed2, xmm2);
            __m256i limit            = _mm256_shuffle_epi8(mmClippingValues256, xmmA);

            val01A = _mm256_min_epi16(val01A, limit);
            val01B = _mm256_min_epi16(val01B, limit);
            val01C = _mm256_min_epi16(val01C, limit);
            val01D = _mm256_min_epi16(val01D, limit);

            limit = _mm256_sub_epi16(_mm256_setzero_si256(), limit);

            val01A = _mm256_max_epi16(val01A, limit);
            val01B = _mm256_max_epi16(val01B, limit);
            val01C = _mm256_max_epi16(val01C, limit);
            val01D = _mm256_max_epi16(val01D, limit);

            val01A = _mm256_add_epi16(val01A, val01C);
            val01B = _mm256_add_epi16(val01B, val01D);

            const __m256i coeff = _mm256_srai_epi16(params[i], 2);

            accumA = _mm256_add_epi32(accumA, _mm256_madd_epi16(val01A, coeff));
            accumB = _mm256_add_epi32(accumB, _mm256_madd_epi16(val01B, coeff));
          };

          process2coeffs(0, pImg[-4] + 0, pImg[+4] + 0, pImg[-3] - 1, pImg[+3] + 1);
          process2coeffs(1, pImg[-2] - 2, pImg[+2] + 2, pImg[-1] - 3, pImg[+1] + 3);
          process2coeffs(2, pImg[-0] - 4, pImg[+0] + 4, pImg[-3] + 1, pImg[+3] - 1);
          process2coeffs(3, pImg[-2] + 2, pImg[+2] - 2, pImg[-1] + 3, pImg[+1] - 3);

          process2coeffs(4, pImg[-3] + 0, pImg[+3] - 0, pImg[-2] - 1, pImg[+2] + 1);
          process2coeffs(5, pImg[-1] - 2, pImg[+1] + 2, pImg[-0] - 3, pImg[+0] + 3);
          process2coeffs(6, pImg[-2] + 1, pImg[+2] - 1, pImg[-1] + 2, pImg[+1] - 2);

          process2coeffs(7, pImg[-2] + 0, pImg[+2] - 0, pImg[-1] - 1, pImg[+1] + 1);
          process2coeffs(8, pImg[-0] - 2, pImg[+0] + 2, pImg[-1] + 1, pImg[+1] - 1);
          process2coeffs(9, pImg[-1] + 0, pImg[+1] - 0, pImg[-0] - 1, pImg[+0] + 1);

          process2coeffs(10, pImgBeforeDb[-4] + 0, pImgBeforeDb[+4] + 0, pImgBeforeDb[-3] - 1, pImgBeforeDb[+3] + 1);
          process2coeffs(11, pImgBeforeDb[-2] - 2, pImgBeforeDb[+2] + 2, pImgBeforeDb[-1] - 3, pImgBeforeDb[+1] + 3);
          process2coeffs(12, pImgBeforeDb[-0] - 4, pImgBeforeDb[+0] + 4, pImgBeforeDb[-3] + 1, pImgBeforeDb[+3] - 1);
          process2coeffs(13, pImgBeforeDb[-2] + 2, pImgBeforeDb[+2] - 2, pImgBeforeDb[-1] + 3, pImgBeforeDb[+1] - 3);

          process2coeffs(14, pImgBeforeDb[-3] + 0, pImgBeforeDb[+3] - 0, pImgBeforeDb[-2] - 1, pImgBeforeDb[+2] + 1);
          process2coeffs(15, pImgBeforeDb[-1] - 2, pImgBeforeDb[+1] + 2, pImgBeforeDb[-0] - 3, pImgBeforeDb[+0] + 3);
          process2coeffs(16, pImgBeforeDb[-2] + 1, pImgBeforeDb[+2] - 1, pImgBeforeDb[-1] + 2, pImgBeforeDb[+1] - 2);

          process2coeffs(17, pImgBeforeDb[-2] + 0, pImgBeforeDb[+2] - 0, pImgBeforeDb[-1] - 1, pImgBeforeDb[+1] + 1);
          process2coeffs(18, pImgBeforeDb[-0] - 2, pImgBeforeDb[+0] + 2, pImgBeforeDb[-1] + 1, pImgBeforeDb[+1] - 1);
          process2coeffs(19, pImgBeforeDb[-1] + 0, pImgBeforeDb[+1] - 0, pImgBeforeDb[-0] - 1, pImgBeforeDb[+0] + 1);

          process2coeffs(20, pImgBeforeDb[0], pImg[0], pImg[0], pImg[0]);

          accumA = _mm256_srai_epi32(accumA, SHIFT);
          accumB = _mm256_srai_epi32(accumB, SHIFT);

          accumA = _mm256_blend_epi16(accumA, _mm256_slli_si256(accumB, 2), 0xAA);
          accumA = _mm256_add_epi16(accumA, cur);
          accumA = _mm256_min_epi16(mmMax, _mm256_max_epi16(accumA, mmMin));

          _mm256_storeu_si256((__m256i *)(&(fixedFilterResult[blkDst.y + i + ii][blkDst.x + j])), accumA);
        }   // for (size_t ii = 0; ii < STEP_Y; ii++)
      }   // for (size_t j = 0; j < width; j += STEP_X)
      src += srcStride2;
      srcBeforeDb += srcBeforeDbStride2;
    }
  }
  else
  {
#endif
    const __m128i mmOffset         = _mm_set1_epi32(ROUND);
    const __m128i mmMin            = _mm_set1_epi16(clpRng.min);
    const __m128i mmMax            = _mm_set1_epi16(clpRng.max);
    const __m128i mmClippingValues = _mm_loadl_epi64((const __m128i *)clippingValues.data());
    const __m128i mm11             = _mm_set1_epi8(1);
    const __m128i mm3              = _mm_set1_epi16(3);

    for (int i = 0; i < height; i += STEP_Y)
    {
      const AdaptiveLoopFilterEcm::AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;

      for (int j = 0; j < width; j += STEP_X)
      {
        __m128i params[21];
        __m128i rawCoef[4][6];

        for (int m = 0; m < 4; m++)
        {
          int       transposeIdx = pClass[j + 2 * m] & 0x3;
          const int filterIdx    = classIndFixed[pClass[j + 2 * m] >> 2];

          rawCoef[m][0] = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx].data()));
          rawCoef[m][1] = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx].data() + 8));
          rawCoef[m][2] = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx].data() + 14));
          rawCoef[m][3] = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx].data() + 20));
          rawCoef[m][4] = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx].data() + 28));
          rawCoef[m][5] = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx].data() + 34));

          // transpose
          if (transposeIdx != 0)
          {
            const __m128i s0 = _mm_loadu_si128((const __m128i *)shTab9Db9[transposeIdx][0]);
            const __m128i s1 = _mm_loadu_si128((const __m128i *)shTab9Db9[transposeIdx][1]);
            const __m128i s2 = _mm_loadu_si128((const __m128i *)shTab9Db9[transposeIdx][2]);

            rawCoef[m][0] = _mm_shuffle_epi8(rawCoef[m][0], s0);
            rawCoef[m][1] = _mm_shuffle_epi8(rawCoef[m][1], s1);
            rawCoef[m][2] = _mm_shuffle_epi8(rawCoef[m][2], s2);
            rawCoef[m][3] = _mm_shuffle_epi8(rawCoef[m][3], s0);
            rawCoef[m][4] = _mm_shuffle_epi8(rawCoef[m][4], s1);
            rawCoef[m][5] = _mm_shuffle_epi8(rawCoef[m][5], s2);
          }
        }   // for(m)

        params[0] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]),
                                       _mm_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
        params[1] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]),
                                       _mm_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
        params[2] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][0], rawCoef[1][0]),
                                       _mm_unpackhi_epi32(rawCoef[2][0], rawCoef[3][0]));
        params[3] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][0], rawCoef[1][0]),
                                       _mm_unpackhi_epi32(rawCoef[2][0], rawCoef[3][0]));

        params[4] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]),
                                       _mm_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
        params[5] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]),
                                       _mm_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
        params[6] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][1], rawCoef[1][1]),
                                       _mm_unpackhi_epi32(rawCoef[2][1], rawCoef[3][1]));

        params[7] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]),
                                       _mm_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
        params[8] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]),
                                       _mm_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
        params[9] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][2], rawCoef[1][2]),
                                       _mm_unpackhi_epi32(rawCoef[2][2], rawCoef[3][2]));

        params[10] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][3], rawCoef[1][3]),
                                        _mm_unpacklo_epi32(rawCoef[2][3], rawCoef[3][3]));
        params[11] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][3], rawCoef[1][3]),
                                        _mm_unpacklo_epi32(rawCoef[2][3], rawCoef[3][3]));
        params[12] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][3], rawCoef[1][3]),
                                        _mm_unpackhi_epi32(rawCoef[2][3], rawCoef[3][3]));
        params[13] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][3], rawCoef[1][3]),
                                        _mm_unpackhi_epi32(rawCoef[2][3], rawCoef[3][3]));

        params[14] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][4], rawCoef[1][4]),
                                        _mm_unpacklo_epi32(rawCoef[2][4], rawCoef[3][4]));
        params[15] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][4], rawCoef[1][4]),
                                        _mm_unpacklo_epi32(rawCoef[2][4], rawCoef[3][4]));
        params[16] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][4], rawCoef[1][4]),
                                        _mm_unpackhi_epi32(rawCoef[2][4], rawCoef[3][4]));

        params[17] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][5], rawCoef[1][5]),
                                        _mm_unpacklo_epi32(rawCoef[2][5], rawCoef[3][5]));
        params[18] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][5], rawCoef[1][5]),
                                        _mm_unpacklo_epi32(rawCoef[2][5], rawCoef[3][5]));
        params[19] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][5], rawCoef[1][5]),
                                        _mm_unpackhi_epi32(rawCoef[2][5], rawCoef[3][5]));
        params[20] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][5], rawCoef[1][5]),
                                        _mm_unpackhi_epi32(rawCoef[2][5], rawCoef[3][5]));

        for (int ii = 0; ii < STEP_Y; ii++)
        {
          const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8;
          pImg0 = src + j + ii * srcStride;
          pImg1 = pImg0 + srcStride;
          pImg2 = pImg0 - srcStride;
          pImg3 = pImg1 + srcStride;
          pImg4 = pImg2 - srcStride;
          pImg5 = pImg3 + srcStride;
          pImg6 = pImg4 - srcStride;
          pImg7 = pImg5 + srcStride;
          pImg8 = pImg6 - srcStride;

          const Pel *pImg0BeforeDb, *pImg1BeforeDb, *pImg2BeforeDb, *pImg3BeforeDb, *pImg4BeforeDb, *pImg5BeforeDb,
            *pImg6BeforeDb, *pImg7BeforeDb, *pImg8BeforeDb;
          pImg0BeforeDb = srcBeforeDb + j + ii * srcBeforeDbStride;
          pImg1BeforeDb = pImg0BeforeDb + srcBeforeDbStride;
          pImg2BeforeDb = pImg0BeforeDb - srcBeforeDbStride;
          pImg3BeforeDb = pImg1BeforeDb + srcBeforeDbStride;
          pImg4BeforeDb = pImg2BeforeDb - srcBeforeDbStride;
          pImg5BeforeDb = pImg3BeforeDb + srcBeforeDbStride;
          pImg6BeforeDb = pImg4BeforeDb - srcBeforeDbStride;
          pImg7BeforeDb = pImg5BeforeDb + srcBeforeDbStride;
          pImg8BeforeDb = pImg6BeforeDb - srcBeforeDbStride;

          __m128i cur = _mm_loadu_si128((const __m128i *)pImg0);

          __m128i accumA = mmOffset;
          __m128i accumB = mmOffset;

          auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
          {
            const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr0), cur);
            const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr2), cur);
            const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr1), cur);
            const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr3), cur);

            __m128i val01A = _mm_blend_epi16(val00, _mm_slli_si128(val10, 2), 0xAA);
            __m128i val01B = _mm_blend_epi16(_mm_srli_si128(val00, 2), val10, 0xAA);
            __m128i val01C = _mm_blend_epi16(val01, _mm_slli_si128(val11, 2), 0xAA);
            __m128i val01D = _mm_blend_epi16(_mm_srli_si128(val01, 2), val11, 0xAA);

            __m128i mmClippingFixed = _mm_and_si128(params[i], mm3);

            __m128i mmClippingFixed2 = _mm_packs_epi16(mmClippingFixed, mmClippingFixed);
            mmClippingFixed2         = _mm_add_epi8(mmClippingFixed2, mmClippingFixed2);
            __m128i xmm2             = _mm_add_epi8(mmClippingFixed2, mm11);
            __m128i xmmA             = _mm_unpacklo_epi8(mmClippingFixed2, xmm2);
            __m128i limit            = _mm_shuffle_epi8(mmClippingValues, xmmA);

            val01A = _mm_min_epi16(val01A, limit);
            val01B = _mm_min_epi16(val01B, limit);
            val01C = _mm_min_epi16(val01C, limit);
            val01D = _mm_min_epi16(val01D, limit);

            limit = _mm_sub_epi16(_mm_setzero_si128(), limit);

            val01A = _mm_max_epi16(val01A, limit);
            val01B = _mm_max_epi16(val01B, limit);
            val01C = _mm_max_epi16(val01C, limit);
            val01D = _mm_max_epi16(val01D, limit);

            val01A = _mm_add_epi16(val01A, val01C);
            val01B = _mm_add_epi16(val01B, val01D);

            const __m128i coeff = _mm_srai_epi16(params[i], 2);

            accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff));
            accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff));
          };

          process2coeffs(0, pImg8 + 0, pImg7 + 0, pImg6 - 1, pImg5 + 1);
          process2coeffs(1, pImg4 - 2, pImg3 + 2, pImg2 - 3, pImg1 + 3);
          process2coeffs(2, pImg0 - 4, pImg0 + 4, pImg6 + 1, pImg5 - 1);
          process2coeffs(3, pImg4 + 2, pImg3 - 2, pImg2 + 3, pImg1 - 3);

          process2coeffs(4, pImg6 + 0, pImg5 - 0, pImg4 - 1, pImg3 + 1);
          process2coeffs(5, pImg2 - 2, pImg1 + 2, pImg0 - 3, pImg0 + 3);
          process2coeffs(6, pImg4 + 1, pImg3 - 1, pImg2 + 2, pImg1 - 2);

          process2coeffs(7, pImg4 + 0, pImg3 - 0, pImg2 - 1, pImg1 + 1);
          process2coeffs(8, pImg0 - 2, pImg0 + 2, pImg2 + 1, pImg1 - 1);
          process2coeffs(9, pImg2 + 0, pImg1 - 0, pImg0 - 1, pImg0 + 1);

          process2coeffs(10, pImg8BeforeDb + 0, pImg7BeforeDb + 0, pImg6BeforeDb - 1, pImg5BeforeDb + 1);
          process2coeffs(11, pImg4BeforeDb - 2, pImg3BeforeDb + 2, pImg2BeforeDb - 3, pImg1BeforeDb + 3);
          process2coeffs(12, pImg0BeforeDb - 4, pImg0BeforeDb + 4, pImg6BeforeDb + 1, pImg5BeforeDb - 1);
          process2coeffs(13, pImg4BeforeDb + 2, pImg3BeforeDb - 2, pImg2BeforeDb + 3, pImg1BeforeDb - 3);

          process2coeffs(14, pImg6BeforeDb + 0, pImg5BeforeDb - 0, pImg4BeforeDb - 1, pImg3BeforeDb + 1);
          process2coeffs(15, pImg2BeforeDb - 2, pImg1BeforeDb + 2, pImg0BeforeDb - 3, pImg0BeforeDb + 3);
          process2coeffs(16, pImg4BeforeDb + 1, pImg3BeforeDb - 1, pImg2BeforeDb + 2, pImg1BeforeDb - 2);

          process2coeffs(17, pImg4BeforeDb + 0, pImg3BeforeDb - 0, pImg2BeforeDb - 1, pImg1BeforeDb + 1);
          process2coeffs(18, pImg0BeforeDb - 2, pImg0BeforeDb + 2, pImg2BeforeDb + 1, pImg1BeforeDb - 1);
          process2coeffs(19, pImg2BeforeDb + 0, pImg1BeforeDb - 0, pImg0BeforeDb - 1, pImg0BeforeDb + 1);
          process2coeffs(20, pImg0BeforeDb, pImg0, pImg0, pImg0);

          accumA = _mm_srai_epi32(accumA, SHIFT);
          accumB = _mm_srai_epi32(accumB, SHIFT);

          accumA = _mm_blend_epi16(accumA, _mm_slli_si128(accumB, 2), 0xAA);
          accumA = _mm_add_epi16(accumA, cur);
          accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

          _mm_storeu_si128((__m128i *)(&(fixedFilterResult[blkDst.y + i + ii][blkDst.x + j])), accumA);
        }   // for (size_t ii = 0; ii < STEP_Y; ii++)
      }   // for (size_t j = 0; j < width; j += STEP_X)
      src += srcStride2;
      srcBeforeDb += srcBeforeDbStride2;
    }
#if USE_AVX2
  }
#endif
}

class FixFilterPackedData9Db9Combine
  : public FixFilterPackedData<AdaptiveLoopFilterEcm::FIX_FILTER_NUM_COEFF_DB_COMBINE_9_DB_9>
{
public:
  FixFilterPackedData9Db9Combine(const FixFilterPackedData9Db9Combine &) = delete;
  FixFilterPackedData9Db9Combine(FixFilterPackedData9Db9Combine &&)      = delete;

  FixFilterPackedData9Db9Combine &operator=(const FixFilterPackedData9Db9Combine &) = delete;
  FixFilterPackedData9Db9Combine &operator=(FixFilterPackedData9Db9Combine &&)      = delete;

  // TODO: Could be constexpr if constructor was constexpr.
  static inline const FixFilterPackedData9Db9Combine &get();

private:
  // TODO: Could be constexpr but maximum number of steps is exceeded.
  inline FixFilterPackedData9Db9Combine();
};

FixFilterPackedData9Db9Combine::FixFilterPackedData9Db9Combine()
  : FixFilterPackedData<AdaptiveLoopFilterEcm::FIX_FILTER_NUM_COEFF_DB_COMBINE_9_DB_9>()
{
  constexpr size_t NUM_COEFF_SRC       = AdaptiveLoopFilterEcm::FIX_FILTER_NUM_COEFF_9_DB_9;
  constexpr size_t NUM_COEFF_SRC_FIRST = NUM_COEFF_SRC / 2;

  constexpr FixFilterPackedData9Db9::P0                                                    p0Filter9Db9;
  constexpr std::array<int, AdaptiveLoopFilterEcm::FIX_FILTER_NUM_COEFF_DB_COMBINE_9_DB_9> p0Filter9Db9Combine {
    // clang-format off
    20, 21, 24, 29, 36, 23, 28, 35,
    22, 25, 30, 37, 27, 34,
    26, 31, 38, 33, 32, 39, 40
    // clang-format on
  };

  const auto &packedDataFixedFilters9Db9        = FixFilterPackedData9Db9::get();
  auto       &packedDataFixedFilters9Db9Combine = *this;
  for (int i = 0; i < AdaptiveLoopFilterEcm::NUM_FIXED_FILTER_SETS; ++i)
  {
    for (int j = 0; j < AdaptiveLoopFilterEcm::NUM_FIXED_FILTERS_PER_SET; ++j)
    {
      // For all positions of the source filter's first (reco) filter.
      for (int k = 0; k < NUM_COEFF_SRC_FIRST; ++k)
      {
        const int combineIdx = p0Filter9Db9Combine[k];
        if (combineIdx == -1)
        {
          // Copy the coefficients of the source filter's first filter.
          packedDataFixedFilters9Db9Combine[i][j][k] = packedDataFixedFilters9Db9[i][j][k];
        }
        else
        {
          // Combine the coefficients of the source filter's first and second filters.
          packedDataFixedFilters9Db9Combine[i][j][k] =
            ((m_filterCoeffFixed9Db9[i][j][p0Filter9Db9[k]] + m_filterCoeffFixed9Db9[i][j][combineIdx]) << 2) |
            ((m_clippingFixed9Db9[i][j][p0Filter9Db9[k]] + m_clippingFixed9Db9[i][j][combineIdx] + 1) >> 1);
        }
      }

      // For all positions of the source filters second filter.
      for (int k = NUM_COEFF_SRC_FIRST; k < AdaptiveLoopFilterEcm::FIX_FILTER_NUM_COEFF_DB_COMBINE_9_DB_9; ++k)
      {
        // Copy the coefficients of the source filter's second filter.
        const int combineIdx = p0Filter9Db9Combine[k];
        packedDataFixedFilters9Db9Combine[i][j][k] =
          (m_filterCoeffFixed9Db9[i][j][combineIdx] << 2) | m_clippingFixed9Db9[i][j][combineIdx];
      }
    }
  }
}

const FixFilterPackedData9Db9Combine &FixFilterPackedData9Db9Combine::get()
{
  static const FixFilterPackedData9Db9Combine obj;
  return obj;
}

// Could theoretically be constexpr. See FixFilterPackedData9Db9Combine.
static const FixFilterPackedData9Db9Combine &packedDataFixedFilters9Db9Combine = FixFilterPackedData9Db9Combine::get();

template<X86_VEXT vext> static void
  simdFilterResi9x9Db9Blk(const AdaptiveLoopFilterEcm::ClassBuf &classifier, const CPelBuf &srcResiLuma,
                          const Area &curBlk, const Area &blkDst, AdaptiveLoopFilterEcm::CompBuf &fixedFilterResiResult,
                          int picWidth, const unsigned fixedFilterSetIdx,
                          const short (&classIndFixed)[AdaptiveLoopFilterEcm::NUM_CLASSES_FIX], const ClpRng &clpRng,
                          const std::array<Pel, AdaptiveLoopFilterEcm::MAX_ALF_NUM_CLIP_VALS> &clippingValues)
{
  constexpr int STEP_X = 8;
  constexpr int STEP_Y = 2;

  if (!(curBlk.width % STEP_X == 0) || !(curBlk.height % STEP_Y == 0))
  {
    AdaptiveLoopFilterEcm::fixedFilteringResi(classifier, srcResiLuma, curBlk, blkDst, fixedFilterResiResult, picWidth,
                                              fixedFilterSetIdx, classIndFixed, clpRng, clippingValues);
    return;
  }

  const ptrdiff_t srcStride = srcResiLuma.stride;
  constexpr int   SHIFT     = AdaptiveLoopFilterEcm::FIX_FILTER_COEFF_SCALE_BITS;
  constexpr int   ROUND     = 1 << (SHIFT - 1);

  const int width  = curBlk.width;
  const int height = curBlk.height;

  const Pel *src = srcResiLuma.buf + curBlk.y * srcStride + curBlk.x;

  const __m128i mmOffset = _mm_set1_epi32(ROUND);

  const int     clpRngmin = -clpRng.max;
  const int     clpRngmax = clpRng.max;
  const __m128i mmMin     = _mm_set1_epi16(clpRngmin);
  const __m128i mmMax     = _mm_set1_epi16(clpRngmax);

  const __m128i mmClippingValues = _mm_loadl_epi64((const __m128i *)clippingValues.data());
  const __m128i mm11             = _mm_set1_epi8(1);
  const __m128i mm3              = _mm_set1_epi16(3);
  const auto   &filterCoeffFixed = packedDataFixedFilters9Db9Combine[fixedFilterSetIdx];
  const Pel     zeros[8]         = { 0 };
  for (int i = 0; i < height; i += STEP_Y)
  {
    const AdaptiveLoopFilterEcm::AlfClassifier *pClass = classifier[blkDst.y + i] + blkDst.x;

    for (int j = 0; j < width; j += STEP_X)
    {
      __m128i params[11];
      __m128i rawCoef[4][3];
      for (int m = 0; m < 4; m++)
      {
        int       transposeIdx = pClass[j + 2 * m] & 0x3;
        const int filterIdx    = classIndFixed[pClass[j + 2 * m] >> 2];

        rawCoef[m][0] = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx].data()));
        rawCoef[m][1] = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx].data() + 8));
        rawCoef[m][2] = _mm_loadu_si128((const __m128i *)(filterCoeffFixed[filterIdx].data() + 14));
        // transpose
        if (transposeIdx != 0)
        {
          const __m128i s0 = _mm_loadu_si128((const __m128i *)shTab9Db9[transposeIdx][0]);
          const __m128i s1 = _mm_loadu_si128((const __m128i *)shTab9Db9[transposeIdx][1]);
          const __m128i s2 = _mm_loadu_si128((const __m128i *)shTab9Db9[transposeIdx][2]);

          rawCoef[m][0] = _mm_shuffle_epi8(rawCoef[m][0], s0);
          rawCoef[m][1] = _mm_shuffle_epi8(rawCoef[m][1], s1);
          rawCoef[m][2] = _mm_shuffle_epi8(rawCoef[m][2], s2);
        }
      }   // for(m)

      params[0] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]),
                                     _mm_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
      params[1] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][0], rawCoef[1][0]),
                                     _mm_unpacklo_epi32(rawCoef[2][0], rawCoef[3][0]));
      params[2] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][0], rawCoef[1][0]),
                                     _mm_unpackhi_epi32(rawCoef[2][0], rawCoef[3][0]));
      params[3] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][0], rawCoef[1][0]),
                                     _mm_unpackhi_epi32(rawCoef[2][0], rawCoef[3][0]));

      params[4] = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]),
                                     _mm_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
      params[5] = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][1], rawCoef[1][1]),
                                     _mm_unpacklo_epi32(rawCoef[2][1], rawCoef[3][1]));
      params[6] = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][1], rawCoef[1][1]),
                                     _mm_unpackhi_epi32(rawCoef[2][1], rawCoef[3][1]));

      params[7]  = _mm_unpacklo_epi64(_mm_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]),
                                      _mm_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
      params[8]  = _mm_unpackhi_epi64(_mm_unpacklo_epi32(rawCoef[0][2], rawCoef[1][2]),
                                      _mm_unpacklo_epi32(rawCoef[2][2], rawCoef[3][2]));
      params[9]  = _mm_unpacklo_epi64(_mm_unpackhi_epi32(rawCoef[0][2], rawCoef[1][2]),
                                      _mm_unpackhi_epi32(rawCoef[2][2], rawCoef[3][2]));
      params[10] = _mm_unpackhi_epi64(_mm_unpackhi_epi32(rawCoef[0][2], rawCoef[1][2]),
                                      _mm_unpackhi_epi32(rawCoef[2][2], rawCoef[3][2]));
      for (int ii = 0; ii < STEP_Y; ii++)
      {
        const AdaptiveLoopFilterEcm::FilterRowPtrs<const Pel, 9> pImg(src + j + ii * srcStride, srcStride);

        __m128i cur    = _mm_loadu_si128((const __m128i *)pImg[0]);
        __m128i accumA = mmOffset;
        __m128i accumB = mmOffset;

        auto process2coeffs = [&](const int i, const Pel *ptr0, const Pel *ptr1, const Pel *ptr2, const Pel *ptr3)
        {
          const __m128i val00 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr0), cur);
          const __m128i val10 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr2), cur);
          const __m128i val01 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr1), cur);
          const __m128i val11 = _mm_sub_epi16(_mm_loadu_si128((const __m128i *)ptr3), cur);

          __m128i val01A = _mm_blend_epi16(val00, _mm_slli_si128(val10, 2), 0xAA);
          __m128i val01B = _mm_blend_epi16(_mm_srli_si128(val00, 2), val10, 0xAA);
          __m128i val01C = _mm_blend_epi16(val01, _mm_slli_si128(val11, 2), 0xAA);
          __m128i val01D = _mm_blend_epi16(_mm_srli_si128(val01, 2), val11, 0xAA);

          __m128i mmClippingFixed = _mm_and_si128(params[i], mm3);

          __m128i mmClippingFixed2 = _mm_packs_epi16(mmClippingFixed, mmClippingFixed);
          mmClippingFixed2         = _mm_add_epi8(mmClippingFixed2, mmClippingFixed2);
          __m128i xmm2             = _mm_add_epi8(mmClippingFixed2, mm11);
          __m128i xmmA             = _mm_unpacklo_epi8(mmClippingFixed2, xmm2);
          __m128i limit            = _mm_shuffle_epi8(mmClippingValues, xmmA);

          val01A = _mm_min_epi16(val01A, limit);
          val01B = _mm_min_epi16(val01B, limit);
          val01C = _mm_min_epi16(val01C, limit);
          val01D = _mm_min_epi16(val01D, limit);

          limit = _mm_sub_epi16(_mm_setzero_si128(), limit);

          val01A = _mm_max_epi16(val01A, limit);
          val01B = _mm_max_epi16(val01B, limit);
          val01C = _mm_max_epi16(val01C, limit);
          val01D = _mm_max_epi16(val01D, limit);

          val01A = _mm_add_epi16(val01A, val01C);
          val01B = _mm_add_epi16(val01B, val01D);

          const __m128i coeff = _mm_srai_epi16(params[i], 2);

          accumA = _mm_add_epi32(accumA, _mm_madd_epi16(val01A, coeff));
          accumB = _mm_add_epi32(accumB, _mm_madd_epi16(val01B, coeff));
        };

        process2coeffs(0, pImg[-4] + 0, pImg[+4] + 0, pImg[-3] - 1, pImg[+3] + 1);
        process2coeffs(1, pImg[-2] - 2, pImg[+2] + 2, pImg[-1] - 3, pImg[+1] + 3);
        process2coeffs(2, pImg[-0] - 4, pImg[+0] + 4, pImg[-3] + 1, pImg[+3] - 1);
        process2coeffs(3, pImg[-2] + 2, pImg[+2] - 2, pImg[-1] + 3, pImg[+1] - 3);

        process2coeffs(4, pImg[-3] + 0, pImg[+3] - 0, pImg[-2] - 1, pImg[+2] + 1);
        process2coeffs(5, pImg[-1] - 2, pImg[+1] + 2, pImg[-0] - 3, pImg[+0] + 3);
        process2coeffs(6, pImg[-2] + 1, pImg[+2] - 1, pImg[-1] + 2, pImg[+1] - 2);

        process2coeffs(7, pImg[-2] + 0, pImg[+2] - 0, pImg[-1] - 1, pImg[+1] + 1);
        process2coeffs(8, pImg[-0] - 2, pImg[+0] + 2, pImg[-1] + 1, pImg[+1] - 1);
        process2coeffs(9, pImg[-1] + 0, pImg[+1] - 0, pImg[-0] - 1, pImg[+0] + 1);

        process2coeffs(10, zeros, pImg[0], pImg[0], pImg[0]);

        accumA = _mm_srai_epi32(accumA, SHIFT);
        accumB = _mm_srai_epi32(accumB, SHIFT);

        accumA = _mm_blend_epi16(accumA, _mm_slli_si128(accumB, 2), 0xAA);
        accumA = _mm_add_epi16(accumA, cur);
        accumA = _mm_min_epi16(mmMax, _mm_max_epi16(accumA, mmMin));

        _mm_storeu_si128((__m128i *)(&(fixedFilterResiResult[blkDst.y + i + ii][blkDst.x + j])), accumA);
      }   // for (size_t ii = 0; ii < STEP_Y; ii++)
    }   // for (size_t j = 0; j < width; j += STEP_X)
    src += srcStride * STEP_Y;
  }
}

template<X86_VEXT vext> void AdaptiveLoopFilterEcm::_initAdaptiveLoopFilterX86()
{
  m_filter9x9Blk                = simdFilter9x9BlkWrapper<vext>;
  m_filter9x9BlkExtDbResiDirect = simdFilter9x9BlkExtDbResiDirectWrapper<vext>;
  m_filter9x9BlkExtDbResi       = simdFilter9x9BlkExtDbResi<vext>;
  m_fixFilter9x9Db9Blk          = simdFixFilter9x9Db9Blk<vext>;
  m_fixFilter13x13Db9Blk        = simdFixFilter13x13Db9Blk<vext>;
  m_filterResi9x9Blk            = simdFilterResi9x9Db9Blk<vext>;
  m_gaussFiltering              = simdGaussFiltering<vext>;

  m_deriveClassificationLaplacian    = simdDeriveClassificationLaplacian;
  m_deriveClassificationLaplacianBig = simdDeriveClassificationLaplacianBig;
  m_deriveVariance                   = simdDeriveVariance<vext>;
  m_calcClassGradBasedFixedFilt      = simdCalcClassGradBasedFixedFilt<vext>;
  m_calcClassGradBasedAdaptFilt      = simdCalcClassGradBasedAdaptFilt;
}

template void AdaptiveLoopFilterEcm::_initAdaptiveLoopFilterX86<SIMDX86>();
#endif   // TARGET_SIMD_X86
