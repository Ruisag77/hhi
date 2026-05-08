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

/** \file     Buffer.cpp
 *  \brief    Low-overhead class describing 2D memory layout
 */

#define DONT_UNDEF_SIZE_AWARE_PER_EL_OP

// unit needs to come first due to a forward declaration
#include "Unit.h"
#include "Buffer.h"
#include "InterpolationFilter.h"

int getMean(int sum, int div)
{
  int sign = 1;
  if (sum < 0)
  {
    sum  = -sum;
    sign = -1;
  }
  int divTable[16] = { 0, 7, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 1, 1, 0 };
  int x            = floorLog2(div);
  int normNum1     = (div << 4 >> x) & 15;
  int v            = divTable[normNum1] | 8;
  x += (normNum1 != 0);
  int shift  = 13 - x;
  int retVal = 0;
  if (shift < 0)
  {
    shift   = -shift;
    int add = (1 << (shift - 1));
    retVal  = (sum * v + add) >> shift;
  }
  else
  {
    retVal = (sum * v) << shift;
  }
  return sign * (retVal >> 16);
}

void applyPROFCore(Pel *dst, ptrdiff_t dstStride, const Pel *src, ptrdiff_t srcStride, int width, int height,
                   const Pel *gradX, const Pel *gradY, ptrdiff_t gradStride, const int *dMvX, const int *dMvY,
                   ptrdiff_t dMvStride, const bool bi, int shiftNum, Pel offset, const ClpRng &clpRng)
{
  int idx = 0;

  const int dILimit = 1 << std::max<int>(clpRng.bd + 1, 13);
  for (int h = 0; h < height; h++)
  {
    for (int w = 0; w < width; w++)
    {
      int32_t dI = dMvX[idx] * gradX[w] + dMvY[idx] * gradY[w];
      dI         = Clip3(-dILimit, dILimit - 1, dI);
      dst[w]     = src[w] + dI;
      if (!bi)
      {
        dst[w] = (dst[w] + offset) >> shiftNum;
        dst[w] = ClipPel(dst[w], clpRng);
      }

      idx++;
    }
    gradX += gradStride;
    gradY += gradStride;
    dst += dstStride;
    src += srcStride;
  }
}

template<unsigned inputSize, unsigned outputSize> void mipMatrixMulCore(Pel *res, const Pel *input,
                                                                        const uint8_t *weight, const int maxVal,
                                                                        const int inputOffset, bool transpose)
{
  Pel buffer[outputSize * outputSize];

  int sum = 0;
  for (int i = 0; i < inputSize; i++)
  {
    sum += input[i];
  }
  const int offset = (1 << (MIP_SHIFT_MATRIX - 1)) - MIP_OFFSET_MATRIX * sum + (inputOffset << MIP_SHIFT_MATRIX);
  CHECK(inputSize != 4 * (inputSize >> 2), "Error, input size not divisible by four");

  Pel     *mat    = transpose ? buffer : res;
  unsigned posRes = 0;
  for (unsigned n = 0; n < outputSize * outputSize; n++)
  {
    int tmp0 = input[0] * weight[0];
    int tmp1 = input[1] * weight[1];
    int tmp2 = input[2] * weight[2];
    int tmp3 = input[3] * weight[3];
    if (8 == inputSize)
    {
      tmp0 += input[4] * weight[4];
      tmp1 += input[5] * weight[5];
      tmp2 += input[6] * weight[6];
      tmp3 += input[7] * weight[7];
    }
    mat[posRes++] = Clip3<int>(0, maxVal, ((tmp0 + tmp1 + tmp2 + tmp3 + offset) >> MIP_SHIFT_MATRIX));

    weight += inputSize;
  }

  if (transpose)
  {
    for (int j = 0; j < outputSize; j++)
    {
      for (int i = 0; i < outputSize; i++)
      {
        res[j * outputSize + i] = buffer[i * outputSize + j];
      }
    }
  }
}

void roundBDCore(const Pel *srcp, const ptrdiff_t srcStride, Pel *dest, const ptrdiff_t destStride, int width,
                 int height, const ClpRng &clpRng)
{
  const int32_t clipbd        = clpRng.bd;
  const int32_t shiftDefault  = std::max<int>(2, (IF_INTERNAL_PREC - clipbd));
  const int32_t offsetDefault = (1 << (shiftDefault - 1)) + IF_INTERNAL_OFFS;

  if (width == 1)
  {
    THROW("Blocks of width = 1 not supported");
  }
  else
  {
#define RND_OP(ADDR) dest[ADDR] = ClipPel(rightShift(srcp[ADDR] + offsetDefault, shiftDefault), clpRng)
#define RND_INC      \
  srcp += srcStride; \
  dest += destStride;

    SIZE_AWARE_PER_EL_OP(RND_OP, RND_INC);

#undef RND_OP
#undef RND_INC
  }
}

void weightedAvgCore(const Pel *src0, const ptrdiff_t src0Stride, const Pel *src1, const ptrdiff_t src1Stride,
                     Pel *dest, const ptrdiff_t destStride, const int8_t w0, const int8_t w1, int shift, int offset,
                     int width, int height, const ClpRng &clpRng)
{

#define ADD_AVG_OP(ADDR) dest[ADDR] = ClipPel(rightShift((src0[ADDR] * w0 + src1[ADDR] * w1 + offset), shift), clpRng)
#define ADD_AVG_INC   \
  src0 += src0Stride; \
  src1 += src1Stride; \
  dest += destStride;

  SIZE_AWARE_PER_EL_OP(ADD_AVG_OP, ADD_AVG_INC);

#undef ADD_AVG_OP
#undef ADD_AVG_INC
}

void weightedAvgCore3(const Pel *src0, const ptrdiff_t src0Stride, const Pel *src1, const ptrdiff_t src1Stride,
                      const Pel *src2, const ptrdiff_t src2Stride, Pel *dest, const ptrdiff_t destStride,
                      const int8_t w0, const int8_t w1, const int8_t w2, int shift, int offset, int width, int height,
                      const ClpRng &clpRng)
{

#define ADD_AVG_OP(ADDR) \
  dest[ADDR] = ClipPel(rightShift((src0[ADDR] * w0 + src1[ADDR] * w1 + src2[ADDR] * w2 + offset), shift), clpRng)
#define ADD_AVG_INC   \
  src0 += src0Stride; \
  src1 += src1Stride; \
  src2 += src2Stride; \
  dest += destStride;

  SIZE_AWARE_PER_EL_OP(ADD_AVG_OP, ADD_AVG_INC);

#undef ADD_AVG_OP
#undef ADD_AVG_INC
}

void copyClipCore(const Pel *srcp, const ptrdiff_t srcStride, Pel *dest, const ptrdiff_t destStride, int width,
                  int height, const ClpRng &clpRng)
{
#define RECO_OP(ADDR) dest[ADDR] = ClipPel(srcp[ADDR], clpRng)
#define RECO_INC     \
  srcp += srcStride; \
  dest += destStride;

  SIZE_AWARE_PER_EL_OP(RECO_OP, RECO_INC);

#undef RECO_OP
#undef RECO_INC
}

template<typename T> void addAvgCore(const T *src1, ptrdiff_t src1Stride, const T *src2, ptrdiff_t src2Stride, T *dest,
                                     ptrdiff_t dstStride, int width, int height, int rshift, int offset,
                                     const ClpRng &clpRng, bool *mcMask[NUM_RPL01], int mcStride, bool *isOOB)
{
  if (mcMask == NULL || (!isOOB[RPL0] && !isOOB[RPL1]))
  {
#define ADD_AVG_CORE_OP(ADDR) dest[ADDR] = ClipPel(rightShift((src1[ADDR] + src2[ADDR] + offset), rshift), clpRng)
#define ADD_AVG_CORE_INC \
  src1 += src1Stride;    \
  src2 += src2Stride;    \
  dest += dstStride;

    SIZE_AWARE_PER_EL_OP(ADD_AVG_CORE_OP, ADD_AVG_CORE_INC);

#undef ADD_AVG_CORE_OP
#undef ADD_AVG_CORE_INC
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
          dest[x] = ClipPel(rightShift(src2[x] + offset2, shiftNum2), clpRng);
        }
        else if (!oob0 && oob1)
        {
          dest[x] = ClipPel(rightShift(src1[x] + offset2, shiftNum2), clpRng);
        }
        else
        {
          dest[x] = ClipPel(rightShift((src1[x] + src2[x] + offset), shiftNum), clpRng);
        }
      }
      pMcMask0 += mcStride;
      pMcMask1 += mcStride;
      src1 += src1Stride;
      src2 += src2Stride;
      dest += dstStride;
    }
  }
}

template<typename T>
void toLastCore(T *src, ptrdiff_t srcStride, int width, int height, int shiftNum, int offset, const ClpRng &clpRng)
{
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      src[x] = ClipPel(rightShift((src[x] + offset), shiftNum), clpRng);
    }
    src += srcStride;
  }
}

template<typename T> void licRemoveWeightHighFreqCore(const T *src0, const T *src1, T *dst, int length, int w0, int w1,
                                                      int offset, const ClpRng &clpRng)
{
  for (int w = 0; w < length; w++)
  {
    T iTemp = ClipPel(T((int(src0[w]) * w0 - int(src1[w]) * w1 + offset) >> 16), clpRng);
    dst[w]  = iTemp;
  }
}

void addBIOAvgCore(const Pel *src0, ptrdiff_t src0Stride, const Pel *src1, ptrdiff_t src1Stride, Pel *dst,
                   ptrdiff_t dstStride, const Pel *gradX0, const Pel *gradX1, const Pel *gradY0, const Pel *gradY1,
                   ptrdiff_t gradStride, int width, int height, int tmpx, int tmpy, int shift, int offset,
                   const ClpRng &clpRng, bool *mcMask[NUM_RPL01], int mcStride, bool *isOOB)
{
  int b = 0;

  if (isOOB[0] || isOOB[1])
  {
    int   offset2  = offset >> 1;
    int   shift2   = shift - 1;
    bool *pMcMask0 = mcMask[0];
    bool *pMcMask1 = mcMask[1];
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        b         = tmpx * (gradX0[x] - gradX1[x]) + tmpy * (gradY0[x] - gradY1[x]);
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
      dst += dstStride;
      src0 += src0Stride;
      src1 += src1Stride;
      gradX0 += gradStride;
      gradX1 += gradStride;
      gradY0 += gradStride;
      gradY1 += gradStride;
      pMcMask0 += mcStride;
      pMcMask1 += mcStride;
    }
  }
  else
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 4)
      {
        b      = tmpx * (gradX0[x] - gradX1[x]) + tmpy * (gradY0[x] - gradY1[x]);
        dst[x] = ClipPel(rightShift((src0[x] + src1[x] + b + offset), shift), clpRng);

        b          = tmpx * (gradX0[x + 1] - gradX1[x + 1]) + tmpy * (gradY0[x + 1] - gradY1[x + 1]);
        dst[x + 1] = ClipPel(rightShift((src0[x + 1] + src1[x + 1] + b + offset), shift), clpRng);

        b          = tmpx * (gradX0[x + 2] - gradX1[x + 2]) + tmpy * (gradY0[x + 2] - gradY1[x + 2]);
        dst[x + 2] = ClipPel(rightShift((src0[x + 2] + src1[x + 2] + b + offset), shift), clpRng);

        b          = tmpx * (gradX0[x + 3] - gradX1[x + 3]) + tmpy * (gradY0[x + 3] - gradY1[x + 3]);
        dst[x + 3] = ClipPel(rightShift((src0[x + 3] + src1[x + 3] + b + offset), shift), clpRng);
      }
      dst += dstStride;
      src0 += src0Stride;
      src1 += src1Stride;
      gradX0 += gradStride;
      gradX1 += gradStride;
      gradY0 += gradStride;
      gradY1 += gradStride;
    }
  }
}

void gradFilterCore(Pel *pSrc, ptrdiff_t srcStride, int width, int height, ptrdiff_t gradStride, Pel *gradX, Pel *gradY,
                    const int bitDepth)
{
  Pel *srcTmp   = pSrc + srcStride + 1;
  Pel *gradXTmp = gradX + gradStride + 1;
  Pel *gradYTmp = gradY + gradStride + 1;
  int  shift1   = 6;

  for (int y = 0; y < (height - 2); y++)
  {
    for (int x = 0; x < (width - 2); x++)
    {
      gradYTmp[x] = (srcTmp[x + srcStride] >> shift1) - (srcTmp[x - srcStride] >> shift1);
      gradXTmp[x] = (srcTmp[x + 1] >> shift1) - (srcTmp[x - 1] >> shift1);
    }
    gradXTmp += gradStride;
    gradYTmp += gradStride;
    srcTmp += srcStride;
  }
}

void calcBIOParameterHighPrecisionCore(const Pel *srcY0Tmp, const Pel *srcY1Tmp, Pel *gradX0, Pel *gradX1, Pel *gradY0,
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
  int32_t temp = 0, tempGX = 0, tempGY = 0;
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      temp   = (int32_t)((srcY1Tmp[x] >> shift4) - (srcY0Tmp[x] >> shift4));
      tempGX = (int32_t)(gradX0[x] + gradX1[x]);
      tempGY = (int32_t)(gradY0[x] + gradY1[x]);
      dI[x]  = (Pel)temp;
      s1[x]  = tempGX * tempGX;
      s2[x]  = tempGX * tempGY;
      s5[x]  = tempGY * tempGY;
      s3[x]  = tempGX * temp;
      s6[x]  = tempGY * temp;
      gX[x]  = tempGX;
      gY[x]  = tempGY;
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

  return;
}

void calcBIOParamSum4HighPrecisionCore(int32_t *s1, int32_t *s2, int32_t *s3, int32_t *s5, int32_t *s6, int width,
                                       int height, const int widthG, int32_t *sumS1, int32_t *sumS2, int32_t *sumS3,
                                       int32_t *sumS5, int32_t *sumS6, Pel *dI, Pel *gX, Pel *gY, bool isGPM = false,
                                       bool isSub = false)
{
  int meanDiff    = 0;
  int absmeanDiff = 0;
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        meanDiff += dI[x];
        absmeanDiff += abs(dI[x]);
      }
      dI += widthG;
    }

    if (isSub)
    {
      meanDiff = getMean(meanDiff + (height * width >> 1), height * width);
    }
    else
    {
      if (isGPM || (absmeanDiff > 2 * abs(meanDiff)))
      {
        meanDiff = 0;
      }
      meanDiff = getMean(meanDiff + height * width, height * width << 1);
    }
  }
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      int w = 1;
      if (width == 12 && height == 12)
      {
        w = g_weight8x8[y * 12 + x];
      }
      else
      {
        w = (x >= (width / 2) ? width - x : x + 1) * (y >= (height / 2) ? height - y : y + 1);
      }
      *sumS1 += w * s1[x];
      *sumS2 += w * s2[x];
      *sumS3 += w * s3[x];
      *sumS5 += w * s5[x];
      *sumS6 += w * s6[x];
      *sumS3 -= w * gX[x] * meanDiff;
      *sumS6 -= w * gY[x] * meanDiff;
    }
    s1 += widthG;
    s2 += widthG;
    s3 += widthG;
    s5 += widthG;
    s6 += widthG;
    gX += widthG;
    gY += widthG;
  }
}

void calcBIOParameterCore(const Pel *srcY0Tmp, const Pel *srcY1Tmp, Pel *gradX0, Pel *gradX1, Pel *gradY0, Pel *gradY1,
                          int width, int height, const int src0Stride, const int src1Stride, const int widthG,
                          const int bitDepth, Pel *absGX, Pel *absGY, Pel *dIX, Pel *dIY, Pel *signGY_GX, Pel *dI)
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
  int shift4 = 4;
  int shift5 = 1;
  if (dI)
  {
    dI += bioParamOffset;
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        int tmpGX    = (gradX0[x] + gradX1[x]) >> shift5;
        int tmpGY    = (gradY0[x] + gradY1[x]) >> shift5;
        int tmpDI    = (int)((srcY1Tmp[x] >> shift4) - (srcY0Tmp[x] >> shift4));
        dI[x]        = tmpDI;
        absGX[x]     = (tmpGX < 0 ? -tmpGX : tmpGX);
        absGY[x]     = (tmpGY < 0 ? -tmpGY : tmpGY);
        dIX[x]       = (tmpGX < 0 ? -tmpDI : (tmpGX == 0 ? 0 : tmpDI));
        dIY[x]       = (tmpGY < 0 ? -tmpDI : (tmpGY == 0 ? 0 : tmpDI));
        signGY_GX[x] = (tmpGY < 0 ? -tmpGX : (tmpGY == 0 ? 0 : tmpGX));
      }
      srcY0Tmp += src0Stride;
      srcY1Tmp += src1Stride;
      gradX0 += widthG;
      gradX1 += widthG;
      gradY0 += widthG;
      gradY1 += widthG;
      absGX += widthG;
      absGY += widthG;
      dI += widthG;
      dIX += widthG;
      dIY += widthG;
      signGY_GX += widthG;
    }

    return;
  }

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      int tmpGX    = (gradX0[x] + gradX1[x]) >> shift5;
      int tmpGY    = (gradY0[x] + gradY1[x]) >> shift5;
      int tmpDI    = (int)((srcY1Tmp[x] >> shift4) - (srcY0Tmp[x] >> shift4));
      absGX[x]     = (tmpGX < 0 ? -tmpGX : tmpGX);
      absGY[x]     = (tmpGY < 0 ? -tmpGY : tmpGY);
      dIX[x]       = (tmpGX < 0 ? -tmpDI : (tmpGX == 0 ? 0 : tmpDI));
      dIY[x]       = (tmpGY < 0 ? -tmpDI : (tmpGY == 0 ? 0 : tmpDI));
      signGY_GX[x] = (tmpGY < 0 ? -tmpGX : (tmpGY == 0 ? 0 : tmpGX));
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
    signGY_GX += widthG;
  }
}

void calcBIOParamSum5Core(Pel *absGX, Pel *absGY, Pel *dIX, Pel *dIY, Pel *signGY_GX, const int widthG, const int width,
                          const int height, int *sumAbsGX, int *sumAbsGY, int *sumDIX, int *sumDIY, int *sumSignGY_GX)
{
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      const int pixelIdx     = y * width + x;
      sumAbsGX[pixelIdx]     = 0;
      sumAbsGY[pixelIdx]     = 0;
      sumDIX[pixelIdx]       = 0;
      sumDIY[pixelIdx]       = 0;
      sumSignGY_GX[pixelIdx] = 0;
      for (int yy = 0; yy < 5; yy++)
      {
        for (int xx = 0; xx < 5; xx++)
        {
          int w = 1;
          w     = (xx >= 2 ? 5 - xx : xx + 1) * (yy >= 2 ? 5 - yy : yy + 1);
          sumAbsGX[pixelIdx] += w * absGX[xx];
          sumAbsGY[pixelIdx] += w * absGY[xx];
          sumDIX[pixelIdx] += w * dIX[xx];
          sumDIY[pixelIdx] += w * dIY[xx];
          sumSignGY_GX[pixelIdx] += w * signGY_GX[xx];
        }
        absGX += widthG;
        absGY += widthG;
        dIX += widthG;
        dIY += widthG;
        signGY_GX += widthG;
      }
      sumDIX[pixelIdx] <<= 2;
      sumDIY[pixelIdx] <<= 2;

      int regVxVy = (1 << 8);
      sumAbsGX[pixelIdx] += regVxVy;
      sumAbsGY[pixelIdx] += regVxVy;

      absGX += (1 - 5 * widthG);
      absGY += (1 - 5 * widthG);
      dIX += (1 - 5 * widthG);
      dIY += (1 - 5 * widthG);
      signGY_GX += (1 - 5 * widthG);
    }
    absGX += (widthG - width);
    absGY += (widthG - width);
    dIX += (widthG - width);
    dIY += (widthG - width);
    signGY_GX += (widthG - width);
  }
}

void calcBIOParamSum5NOSIMCore(int32_t *absGX, int32_t *absGY, int32_t *dIX, int32_t *dIY, int32_t *signGyGx,
                               const int widthG, const int width, const int height, int *sumAbsGX, int *sumAbsGY,
                               int *sumDIX, int *sumDIY, int *sumSignGyGx, Pel *dI, Pel *gX, Pel *gY)
{
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
      int sX0 = 0, sX1 = 0;
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
          sX0 -= w * gX[xx];
          sX1 -= w * gY[xx];
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
      sumDIX[sampleIdx] += sX0 * meanDiff;
      sumDIY[sampleIdx] += sX1 * meanDiff;
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

void calcBIOParamSum4Core(Pel *absGX, Pel *absGY, Pel *dIX, Pel *dIY, Pel *signGY_GX, int width, int height,
                          const int widthG, int *sumAbsGX, int *sumAbsGY, int *sumDIX, int *sumDIY, int *sumSignGY_GX)
{
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      *sumAbsGX += absGX[x];
      *sumAbsGY += absGY[x];
      *sumDIX += dIX[x];
      *sumDIY += dIY[x];
      *sumSignGY_GX += signGY_GX[x];
    }
    absGX += widthG;
    absGY += widthG;
    dIX += widthG;
    dIY += widthG;
    signGY_GX += widthG;
  }
}

void calcBIOClippedVxVyCore(int *sumDIXSample32bit, int *sumAbsGxSample32bit, int *sumDIYSample32bit,
                            int *sumAbsGySample32bit, int *sumSignGyGxSample32bit, const int limit,
                            const int bioSubblockSize, int *tmpxSample32bit, int *tmpySample32bit)
{
  int divTable[32] = { 32, 31, 30, 29, 28, 28, 27, 26, 26, 25, 24, 24, 23, 23, 22, 22,
                       21, 21, 20, 20, 20, 19, 19, 19, 18, 18, 18, 17, 17, 17, 17, 16 };
  int log2D = 0, msbD = 0, invD = 0, shiftD = 0, offsetD = 0;
  for (int idx = 0; idx < bioSubblockSize; idx++)
  {
    *tmpxSample32bit = 0;
    *tmpySample32bit = 0;
    if (*sumAbsGxSample32bit > 0)
    {
      log2D            = floorLog2(*sumAbsGxSample32bit);
      msbD             = int((*sumAbsGxSample32bit << 5) >> log2D) & 31;
      invD             = divTable[msbD];
      shiftD           = log2D + 5;
      offsetD          = (1 << (shiftD - 1));
      *tmpxSample32bit = Clip3(-limit, limit, int(((*sumDIXSample32bit << 3) * invD + offsetD) >> shiftD));
    }
    if (*sumAbsGySample32bit > 0)
    {
      log2D            = floorLog2(*sumAbsGySample32bit);
      msbD             = int((*sumAbsGySample32bit << 5) >> log2D) & 31;
      invD             = divTable[msbD];
      shiftD           = log2D + 5;
      offsetD          = (1 << (shiftD - 1));
      *tmpySample32bit = Clip3(
        -limit, limit,
        int((((*sumDIYSample32bit << 3) - ((*tmpxSample32bit) * (*sumSignGyGxSample32bit) >> 1)) * invD + offsetD) >>
            shiftD));
    }
    sumDIXSample32bit++;
    sumAbsGxSample32bit++;
    sumDIYSample32bit++;
    sumAbsGySample32bit++;
    sumSignGyGxSample32bit++;
    tmpxSample32bit++;
    tmpySample32bit++;
  }
}

void addBIOAvgNCore(const Pel *src0, int src0Stride, const Pel *src1, int src1Stride, Pel *dst, int dstStride,
                    const Pel *gradX0, const Pel *gradX1, const Pel *gradY0, const Pel *gradY1, int gradStride,
                    int width, int height, int *tmpx, int *tmpy, int shift, int offset, const ClpRng &clpRng,
                    bool *mcMask[2], int mcStride, bool *isOOB)
{
  int       b        = 0;
  int       offset2  = offset >> 1;
  int       shift2   = shift - 1;
  bool     *pMcMask0 = mcMask[0];
  bool     *pMcMask1 = mcMask[1];
  int       pX = 0, pY = 0;
  const int tt = 16;
  if (isOOB[0] || isOOB[1])
  {
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
  }
  else
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        pX     = (tmpx[x] > tt) ? 1 : ((tmpx[x] < -tt) ? -1 : 0);
        pY     = (tmpy[x] > tt) ? 1 : ((tmpy[x] < -tt) ? -1 : 0);
        int xX = tmpx[x] - 32 * pX;
        int yY = tmpy[x] - 32 * pY;
        b      = (int)xX * (gradX0[x + pY * gradStride + pX] - gradX1[x - pY * gradStride - pX]) +
          (int)yY * (gradY0[x + pY * gradStride + pX] - gradY1[x - pY * gradStride - pX]);
        dst[x] = ClipPel(
          rightShift((src0[x + pY * src0Stride + pX] + src1[x - pY * src1Stride - pX] + b + offset), shift), clpRng);
      }
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
  }
  return;
}

void calAbsSumCore(const Pel *diff, int stride, int width, int height, int *absSum)
{
  *absSum = 0;
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      *absSum += ::abs(diff[x]);
    }
    diff += stride;
  }
}

template<typename T> void reconstructCore(const T *src1, ptrdiff_t src1Stride, const T *src2, ptrdiff_t src2Stride,
                                          T *dest, ptrdiff_t dstStride, int width, int height, const ClpRng &clpRng)
{
#define RECO_CORE_OP(ADDR) dest[ADDR] = ClipPel(src1[ADDR] + src2[ADDR], clpRng)
#define RECO_CORE_INC \
  src1 += src1Stride; \
  src2 += src2Stride; \
  dest += dstStride;

  SIZE_AWARE_PER_EL_OP(RECO_CORE_OP, RECO_CORE_INC);

#undef RECO_CORE_OP
#undef RECO_CORE_INC
}

template<typename T> void linTfCore(const T *src, ptrdiff_t srcStride, Pel *dst, ptrdiff_t dstStride, int width,
                                    int height, int scale, int shift, int offset, const ClpRng &clpRng, bool bClip)
{
#define LINTF_CORE_OP(ADDR)                                                               \
  dst[ADDR] = (Pel)bClip ? ClipPel(rightShift(scale * src[ADDR], shift) + offset, clpRng) \
                         : (rightShift(scale * src[ADDR], shift) + offset)
#define LINTF_CORE_INC \
  src += srcStride;    \
  dst += dstStride;

  SIZE_AWARE_PER_EL_OP(LINTF_CORE_OP, LINTF_CORE_INC);

#undef LINTF_CORE_OP
#undef LINTF_CORE_INC
}

template<typename T> void copyClipCore(const T *src, ptrdiff_t srcStride, Pel *dst, ptrdiff_t dstStride, int width,
                                       int height, const ClpRng &clpRng)
{
#define RECO_OP(ADDR) dst[ADDR] = ClipPel(src[ADDR], clpRng)
#define RECO_INC    \
  src += srcStride; \
  dst += dstStride;

  SIZE_AWARE_PER_EL_OP(RECO_OP, RECO_INC);

#undef RECO_OP
#undef RECO_INC
}

bool isMvOOBCore(const Mv &rcMv, const Position pos, const Size size, const SPS *sps, const PPS *pps, bool *mcMask,
                 bool *mcMaskChroma, bool lumaOnly, ChromaFormat componentID)
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
    int   baseOffsetX = offsetX;
    bool *pMcMask     = mcMask;

    for (int y = 0; y < size.height; y++, offsetY += mvstep)
    {
      offsetX     = baseOffsetX;
      bool checkY = (offsetY <= verMin) || (offsetY >= verMax);
      for (int x = 0; x < size.width; x++, offsetX += mvstep)
      {
        pMcMask[x] = (offsetX <= horMin) || (offsetX >= horMax) || checkY;
      }
      pMcMask += size.width;
    }

    if (!lumaOnly)
    {
      bool *pMcMaskChroma = mcMaskChroma;
      pMcMask             = mcMask;
      int widthChroma     = (size.width) >> chromaScale;
      int heightChroma    = (size.height) >> chromaScale;
      int widthLuma2      = size.width << chromaScale;
      for (int y = 0; y < heightChroma; y++)
      {
        for (int x = 0; x < widthChroma; x++)
        {
          pMcMaskChroma[x] = pMcMask[x << chromaScale];
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

bool isMvOOBSubBlkCore(const Mv &rcMv, const Position pos, const Size size, const SPS *sps, const PPS *pps,
                       bool *mcMask, int mcStride, bool *mcMaskChroma, int mcCStride, bool lumaOnly,
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
    int   baseOffsetX = offsetX;
    bool *pMcMask     = mcMask;
    for (int y = 0; y < size.height; y++, offsetY += mvstep)
    {
      offsetX     = baseOffsetX;
      bool checkY = (offsetY <= verMin) || (offsetY >= verMax);
      ;
      for (int x = 0; x < size.width; x++, offsetX += mvstep)
      {
        pMcMask[x] = (offsetX <= horMin) || (offsetX >= horMax) || checkY;
      }
      pMcMask += mcStride;
    }

    if (!lumaOnly)
    {
      bool *pMcMaskChroma = mcMaskChroma;
      pMcMask             = mcMask;
      int widthChroma     = (size.width) >> chromaScale;
      int heightChroma    = (size.height) >> chromaScale;
      int strideLuma2     = mcStride << chromaScale;
      for (int y = 0; y < heightChroma; y++)
      {
        for (int x = 0; x < widthChroma; x++)
        {
          pMcMaskChroma[x] = pMcMask[x << chromaScale];
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

void computeDeltaAndShiftCore(const Position posLT, Mv firstMv, std::vector<RMVFInfo> &mvpInfoVecOri)
{
  for (int i = 0; i < int(mvpInfoVecOri.size()); i++)
  {
    mvpInfoVecOri[i].pos.x = mvpInfoVecOri[i].pos.x - posLT.x;
    mvpInfoVecOri[i].pos.y = mvpInfoVecOri[i].pos.y - posLT.y;
    mvpInfoVecOri[i].mvp.set(mvpInfoVecOri[i].mvp.getHor() - firstMv.getHor(),
                             mvpInfoVecOri[i].mvp.getVer() - firstMv.getVer());
    mvpInfoVecOri[i].mvp.set(Clip3(-RMVF_MV_RANGE, RMVF_MV_RANGE - 1, mvpInfoVecOri[i].mvp.hor),
                             Clip3(-RMVF_MV_RANGE, RMVF_MV_RANGE - 1, mvpInfoVecOri[i].mvp.ver));
  }
}
void computeDeltaAndShiftCoreAddi(const Position posLT, Mv firstMv, std::vector<RMVFInfo> &mvpInfoVecOri,
                                  std::vector<RMVFInfo> &mvpInfoVecRes)
{
  int offset = (int)mvpInfoVecRes.size();
  for (int i = 0; i < int(mvpInfoVecOri.size()); i++)
  {
    mvpInfoVecRes.push_back(RMVFInfo());
    mvpInfoVecRes[offset + i].pos.x = mvpInfoVecOri[i].pos.x - posLT.x;
    mvpInfoVecRes[offset + i].pos.y = mvpInfoVecOri[i].pos.y - posLT.y;
    mvpInfoVecRes[offset + i].mvp.set(mvpInfoVecOri[i].mvp.getHor() - firstMv.getHor(),
                                      mvpInfoVecOri[i].mvp.getVer() - firstMv.getVer());
    mvpInfoVecRes[offset + i].mvp.set(Clip3(-RMVF_MV_RANGE, RMVF_MV_RANGE - 1, mvpInfoVecRes[offset + i].mvp.hor),
                                      Clip3(-RMVF_MV_RANGE, RMVF_MV_RANGE - 1, mvpInfoVecRes[offset + i].mvp.ver));
  }
}
void buildRegressionMatrixCore(std::vector<RMVFInfo> &mvpInfoVecOri, int64_t sumbb[2][3][3], int64_t sumeb[2][3],
                               uint16_t addedSize)
{
  int iNum = (int)mvpInfoVecOri.size();
  int b[3];
  int e[2];
  for (int ni = addedSize ? iNum - addedSize : 0; ni < iNum; ni++)   // for all neighbor PUs
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
}

PelBufferOps::PelBufferOps()
{
  roundBD      = roundBDCore;
  weightedAvg  = weightedAvgCore;
  weightedAvg3 = weightedAvgCore3;
  copyClip     = copyClipCore;
  addAvg4      = addAvgCore<Pel>;
  addAvg8      = addAvgCore<Pel>;

  toLast2                  = toLastCore<Pel>;
  toLast4                  = toLastCore<Pel>;
  licRemoveWeightHighFreq2 = licRemoveWeightHighFreqCore<Pel>;
  licRemoveWeightHighFreq4 = licRemoveWeightHighFreqCore<Pel>;

  reco4 = reconstructCore<Pel>;
  reco8 = reconstructCore<Pel>;

  linTf4 = linTfCore<Pel>;
  linTf8 = linTfCore<Pel>;

  copyClip4 = copyClipCore;
  copyClip8 = copyClipCore;

  bioGradFilter                 = gradFilterCore;
  calcBIOParameter              = calcBIOParameterCore;
  calcBIOParamSum5              = calcBIOParamSum5Core;
  calcBIOParamSum5NOSIM4        = calcBIOParamSum5NOSIMCore;
  calcBIOParamSum5NOSIM8        = calcBIOParamSum5NOSIMCore;
  calcBIOParamSum4              = calcBIOParamSum4Core;
  calcBIOClippedVxVy            = calcBIOClippedVxVyCore;
  addBIOAvgN                    = addBIOAvgNCore;
  calAbsSum                     = calAbsSumCore;
  calcBIOParameterHighPrecision = calcBIOParameterHighPrecisionCore;
  calcBIOParamSum4HighPrecision = calcBIOParamSum4HighPrecisionCore;

  copyBuffer = copyBufferCore;
#if ENABLE_SIMD_OPT_BCW
  removeWeightHighFreq8 = nullptr;
  removeWeightHighFreq4 = nullptr;
  removeHighFreq8       = nullptr;
  removeHighFreq4       = nullptr;
#endif
  mipMatrixMul_4_4 = mipMatrixMulCore<4, 4>;
  mipMatrixMul_8_4 = mipMatrixMulCore<8, 4>;
  mipMatrixMul_8_8 = mipMatrixMulCore<8, 8>;

  profGradFilter           = gradFilterCore;
  applyPROF                = applyPROFCore;
  roundIntVector           = nullptr;
  isMvOOB                  = isMvOOBCore;
  isMvOOBSubBlk            = isMvOOBSubBlkCore;
  computeDeltaAndShift     = computeDeltaAndShiftCore;
  computeDeltaAndShiftAddi = computeDeltaAndShiftCoreAddi;
  buildRegressionMatrix    = buildRegressionMatrixCore;
}

PelBufferOps g_pelBufOP = PelBufferOps();

void copyBufferCore(const Pel *src, ptrdiff_t srcStride, Pel *dst, ptrdiff_t dstStride, int width, int height)
{
  int numBytes = width * sizeof(Pel);
  for (int i = 0; i < height; i++)
  {
    memcpy(dst + i * dstStride, src + i * srcStride, numBytes);
  }
}

template<> void AreaBuf<Pel>::addWeightedAvg(const AreaBuf<const Pel> &other1, const AreaBuf<const Pel> &other2,
                                             const ClpRng &clpRng, const int8_t bcwIdx, bool *mcMask[NUM_RPL01],
                                             int mcStride, bool *isOOB)
{
  int8_t w0 = getBcwWeight(bcwIdx, RPL0);
  int8_t w1 = getBcwWeight(bcwIdx, RPL1);

  const int8_t log2WeightBase = g_BcwLog2WeightBase;
  const Pel   *src1           = other1.buf;
  const Pel   *src2           = other2.buf;
  Pel         *dest           = buf;

  const ptrdiff_t src1Stride = other1.stride;
  const ptrdiff_t src2Stride = other2.stride;
  const ptrdiff_t destStride = stride;
  const int       clipbd     = clpRng.bd;
  if (!isOOB[RPL0] && !isOOB[RPL1])
  {
    const int shiftNum = std::max<int>(2, (IF_INTERNAL_PREC - clipbd)) + log2WeightBase;
    const int offset   = (1 << (shiftNum - 1)) + (IF_INTERNAL_OFFS << log2WeightBase);
    g_pelBufOP.weightedAvg(src1, src1Stride, src2, src2Stride, dest, destStride, w0, w1, shiftNum, offset, width,
                           height, clpRng);
  }
  else
  {
    int       shiftNum2 = IF_INTERNAL_FRAC_BITS(clipbd);
    const int offset2   = (1 << (shiftNum2 - 1)) + IF_INTERNAL_OFFS;
    bool     *pMcMask0  = mcMask[RPL0];
    bool     *pMcMask1  = mcMask[RPL1];

    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        bool oob0 = pMcMask0[x];
        bool oob1 = pMcMask1[x];
        if (oob0 && !oob1)
        {
          dest[x] = ClipPel(rightShift(src2[x] + offset2, shiftNum2), clpRng);
        }
        else if (!oob0 && oob1)
        {
          dest[x] = ClipPel(rightShift(src1[x] + offset2, shiftNum2), clpRng);
        }
        else
        {
          const int shiftNum = IF_INTERNAL_FRAC_BITS(clipbd) + log2WeightBase;
          const int offset   = (1 << (shiftNum - 1)) + (IF_INTERNAL_OFFS << log2WeightBase);
          dest[x + 0]        = ClipPel(rightShift((src1[x] * w0 + src2[x + 0] * w1 + offset), shiftNum), clpRng);
        }
      }
      pMcMask0 += mcStride;
      pMcMask1 += mcStride;
      src1 += src1Stride;
      src2 += src2Stride;
      dest += destStride;
    }
  }
}

template<> void AreaBuf<Pel>::rspSignal(const std::vector<Pel> &pLUT)
{
  Pel *dst = buf;
  Pel *src = buf;
  for (unsigned y = 0; y < height; y++)
  {
    for (unsigned x = 0; x < width; x++)
    {
      dst[x] = pLUT[src[x]];
    }
    dst += stride;
    src += stride;
  }
}

template<> void AreaBuf<Pel>::rspSignal(const AreaBuf<Pel> &toReshape, std::vector<Pel> &pLUT)
{
  CHECK(width != toReshape.width, "Incompatible size");
  CHECK(height != toReshape.height, "Incompatible size");

  Pel *dst = buf;
  Pel *src = toReshape.buf;
  for (unsigned y = 0; y < height; y++)
  {
    for (unsigned x = 0; x < width; x++)
    {
      dst[x] = pLUT[src[x]];
    }
    dst += stride;
    src += toReshape.stride;
  }
}

template<> void AreaBuf<Pel>::scaleSignal(const int scale, const bool dir, const ClpRng &clpRng)
{
  Pel *dst = buf;
  Pel *src = buf;
  int  absval;
  int  maxAbsclipBD = (1 << clpRng.bd) - 1;

  if (dir)   // forward
  {
    if (width == 1)
    {
      THROW("Blocks of width = 1 not supported");
    }
    else
    {
      for (unsigned y = 0; y < height; y++)
      {
        for (unsigned x = 0; x < width; x++)
        {
          const int sign = sgn2(src[x]);
          absval         = sign * src[x];
          dst[x] =
            (Pel)Clip3(-maxAbsclipBD, maxAbsclipBD, sign * (((absval << CSCALE_FP_PREC) + (scale >> 1)) / scale));
        }
        dst += stride;
        src += stride;
      }
    }
  }
  else   // inverse
  {
    for (unsigned y = 0; y < height; y++)
    {
      for (unsigned x = 0; x < width; x++)
      {
        src[x]         = (Pel)Clip3((Pel)(-maxAbsclipBD - 1), (Pel)maxAbsclipBD, src[x]);
        const int sign = sgn2(src[x]);
        absval         = sign * src[x];
        int val        = sign * ((absval * scale + (1 << (CSCALE_FP_PREC - 1))) >> CSCALE_FP_PREC);
        if (sizeof(Pel) == 2)   // avoid overflow when storing data
        {
          val = Clip3<int>(-32768, 32767, val);
        }
        dst[x] = (Pel)val;
      }
      dst += stride;
      src += stride;
    }
  }
}

template<> void AreaBuf<Pel>::applyLumaCTI(std::vector<Pel> &pLUTY)
{
  Pel *dst = buf;
  Pel *src = buf;
  for (unsigned y = 0; y < height; y++)
  {
    for (unsigned x = 0; x < width; x++)
    {
      dst[x] = pLUTY[src[x]];
    }
    dst += stride;
    src += stride;
  }
}

template<> void AreaBuf<Pel>::applyChromaCTI(Pel *bufY, ptrdiff_t strideY, std::vector<Pel> &pLUTC, int bitDepth,
                                             ChromaFormat chrFormat, bool fwdMap)
{
  int range  = 1 << bitDepth;
  int offset = range / 2;
  int sx     = 1 << getComponentScaleX(COMP_Cb, chrFormat);
  int sy     = 1 << getComponentScaleY(COMP_Cb, chrFormat);

  Pel *dst = buf;
  Pel *src = buf;
  if (fwdMap)
  {
    for (unsigned y = 0; y < height; y++)
    {
      for (unsigned x = 0; x < width; x++)
      {
        int    pelY  = bufY[sy * y * strideY + sx * x];
        double scale = (double)pLUTC[pelY] / (double)(1 << CSCALE_FP_PREC);
        dst[x]       = Clip3((Pel)0, (Pel)(range - 1), (Pel)(offset + (double)(src[x] - offset) / scale + .5));
      }
      dst += stride;
      src += stride;
    }
  }
  else
  {
    for (unsigned y = 0; y < height; y++)
    {
      for (unsigned x = 0; x < width; x++)
      {
        int pelY = bufY[sy * y * strideY + sx * x];
        int scal = pLUTC[pelY];
        dst[x]   = Clip3(0, range - 1,
                         ((offset << CSCALE_FP_PREC) + (src[x] - offset) * scal + (1 << (CSCALE_FP_PREC - 1))) >>
                           CSCALE_FP_PREC);
      }
      dst += stride;
      src += stride;
    }
  }
}

template<> void AreaBuf<Pel>::addAvg(const AreaBuf<const Pel> &other1, const AreaBuf<const Pel> &other2,
                                     const ClpRng &clpRng, bool *mcMask[NUM_RPL01], int mcStride, bool *isOOB)
{
  const Pel *src0 = other1.buf;
  const Pel *src2 = other2.buf;
  Pel       *dest = buf;

  const ptrdiff_t src1Stride = other1.stride;
  const ptrdiff_t src2Stride = other2.stride;
  const ptrdiff_t destStride = stride;
  const int       clipbd     = clpRng.bd;
  const int       shiftNum   = IF_INTERNAL_FRAC_BITS(clipbd) + 1;
  const int       offset     = (1 << (shiftNum - 1)) + 2 * IF_INTERNAL_OFFS;

  if (mcMask == NULL || (!isOOB[RPL0] && !isOOB[RPL1]))
  {
#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
    if ((width & 7) == 0)
    {
      g_pelBufOP.addAvg8(src0, src1Stride, src2, src2Stride, dest, destStride, width, height, shiftNum, offset, clpRng,
                         mcMask, mcStride, isOOB);
    }
    else if ((width & 3) == 0)
    {
      g_pelBufOP.addAvg4(src0, src1Stride, src2, src2Stride, dest, destStride, width, height, shiftNum, offset, clpRng,
                         mcMask, mcStride, isOOB);
    }
    else
#endif
    {
#define ADD_AVG_OP(ADDR) dest[ADDR] = ClipPel(rightShift((src0[ADDR] + src2[ADDR] + offset), shiftNum), clpRng)
#define ADD_AVG_INC   \
  src0 += src1Stride; \
  src2 += src2Stride; \
  dest += destStride;

      SIZE_AWARE_PER_EL_OP(ADD_AVG_OP, ADD_AVG_INC);

#undef ADD_AVG_OP
#undef ADD_AVG_INC
    }
  }
  else
  {
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
          dest[x] = ClipPel(rightShift(src2[x] + offset2, shiftNum2), clpRng);
        }
        else if (!oob0 && oob1)
        {
          dest[x] = ClipPel(rightShift(src0[x] + offset2, shiftNum2), clpRng);
        }
        else
        {
          dest[x] = ClipPel(rightShift((src0[x] + src2[x] + offset), shiftNum), clpRng);
        }
      }
      pMcMask0 += mcStride;
      pMcMask1 += mcStride;
      src0 += src1Stride;
      src2 += src2Stride;
      dest += destStride;
    }
  }
}

template<> void AreaBuf<Pel>::toLast(const ClpRng &clpRng)
{
  Pel            *src       = buf;
  const ptrdiff_t srcStride = stride;

  const int clipbd   = clpRng.bd;
  const int shiftNum = IF_INTERNAL_FRAC_BITS(clipbd);
  const int offset   = (1 << (shiftNum - 1)) + IF_INTERNAL_OFFS;

  if (width == 1)
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        src[x] = ClipPel(rightShift((src[x] + offset), shiftNum), clpRng);
      }
      src += srcStride;
    }
  }
  else if ((width & 3) == 0)
  {
    g_pelBufOP.toLast4(src, srcStride, width, height, shiftNum, offset, clpRng);
  }
  else if ((width & 1) == 0)
  {
    g_pelBufOP.toLast2(src, srcStride, width, height, shiftNum, offset, clpRng);
  }
  else
  {
    THROW("Unsupported size!");
  }
}

template<> void AreaBuf<Pel>::copyClip(const AreaBuf<const Pel> &src, const ClpRng &clpRng)
{
  const Pel *srcp = src.buf;
  Pel       *dest = buf;

  const ptrdiff_t srcStride  = src.stride;
  const ptrdiff_t destStride = stride;

  if ((width & 7) == 0)
  {
    g_pelBufOP.copyClip8(srcp, srcStride, dest, destStride, width, height, clpRng);
  }
  else if ((width & 3) == 0)
  {
    g_pelBufOP.copyClip4(srcp, srcStride, dest, destStride, width, height, clpRng);
  }
  else
  {
#define RECO_OP(ADDR) dest[ADDR] = ClipPel(srcp[ADDR], clpRng)
#define RECO_INC     \
  srcp += srcStride; \
  dest += destStride;

    SIZE_AWARE_PER_EL_OP(RECO_OP, RECO_INC);

#undef RECO_OP
#undef RECO_INC
  }
}

template<> void AreaBuf<Pel>::roundToOutputBitdepth(const AreaBuf<const Pel> &src, const ClpRng &clpRng)
{
  const Pel      *srcp       = src.buf;
  Pel            *dest       = buf;
  const ptrdiff_t srcStride  = src.stride;
  const ptrdiff_t destStride = stride;
  if (width == 1)
  {
    THROW("Blocks of width = 1 not supported");
  }
  else
  {
    g_pelBufOP.roundBD(srcp, srcStride, dest, destStride, width, height, clpRng);
  }
}

template<>
void AreaBuf<Pel>::reconstruct(const AreaBuf<const Pel> &pred, const AreaBuf<const Pel> &resi, const ClpRng &clpRng)
{
  const Pel *src1 = pred.buf;
  const Pel *src2 = resi.buf;
  Pel       *dest = buf;

  const ptrdiff_t src1Stride = pred.stride;
  const ptrdiff_t src2Stride = resi.stride;
  const ptrdiff_t destStride = stride;

#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
  if ((width & 7) == 0)
  {
    g_pelBufOP.reco8(src1, src1Stride, src2, src2Stride, dest, destStride, width, height, clpRng);
  }
  else if ((width & 3) == 0)
  {
    g_pelBufOP.reco4(src1, src1Stride, src2, src2Stride, dest, destStride, width, height, clpRng);
  }
  else
#endif
  {
#define RECO_OP(ADDR) dest[ADDR] = ClipPel(src1[ADDR] + src2[ADDR], clpRng)
#define RECO_INC      \
  src1 += src1Stride; \
  src2 += src2Stride; \
  dest += destStride;

    SIZE_AWARE_PER_EL_OP(RECO_OP, RECO_INC);

#undef RECO_OP
#undef RECO_INC
  }
}

template<>
void AreaBuf<Pel>::linearTransform(const int scale, const int shift, const int offset, bool bClip, const ClpRng &clpRng)
{
  const Pel *src = buf;
  Pel       *dst = buf;

  if (width == 1)
  {
    THROW("Blocks of width = 1 not supported");
  }
#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
  else if ((width & 7) == 0)
  {
    g_pelBufOP.linTf8(src, stride, dst, stride, width, height, scale, shift, offset, clpRng, bClip);
  }
  else if ((width & 3) == 0)
  {
    g_pelBufOP.linTf4(src, stride, dst, stride, width, height, scale, shift, offset, clpRng, bClip);
  }
#endif
  else
  {
#define LINTF_OP(ADDR)                                                                    \
  dst[ADDR] = (Pel)bClip ? ClipPel(rightShift(scale * src[ADDR], shift) + offset, clpRng) \
                         : (rightShift(scale * src[ADDR], shift) + offset)
#define LINTF_INC \
  src += stride;  \
  dst += stride;

    SIZE_AWARE_PER_EL_OP(LINTF_OP, LINTF_INC);

#undef LINTF_OP
#undef LINTF_INC
  }
}

#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
template<> void AreaBuf<Pel>::subtract(const Pel val)
{
  ClpRng clpRngDummy;
  linearTransform(1, 0, -val, false, clpRngDummy);
}
#endif

PelStorage::PelStorage()
{
  for (uint32_t i = 0; i < MAX_NUM_COMP; i++)
  {
    m_origin[i] = nullptr;
  }
}

PelStorage::~PelStorage() { destroy(); }

void PelStorage::create(const UnitArea &_unitArea)
{
  create(_unitArea.chromaFormat, _unitArea.blocks[0]);
  m_maxArea = _unitArea;
}

void PelStorage::create(const ChromaFormat &_chromaFormat, const Area &_area, const unsigned _maxCUSize,
                        const unsigned _margin, const unsigned _alignment, const bool _scaleChromaMargin)
{
  CHECK(!bufs.empty(), "Trying to re-create an already initialized buffer");

  chromaFormat = _chromaFormat;

  const uint32_t numCh = getNumberValidComponents(_chromaFormat);

  unsigned extHeight = _area.height;
  unsigned extWidth  = _area.width;

  if (_maxCUSize)
  {
    extHeight = ((_area.height + _maxCUSize - 1) / _maxCUSize) * _maxCUSize;
    extWidth  = ((_area.width + _maxCUSize - 1) / _maxCUSize) * _maxCUSize;
  }

  for (uint32_t i = 0; i < numCh; i++)
  {
    const CompID   compID = CompID(i);
    const unsigned scaleX = ::getComponentScaleX(compID, _chromaFormat);
    const unsigned scaleY = ::getComponentScaleY(compID, _chromaFormat);

    unsigned scaledHeight = extHeight >> scaleY;
    unsigned scaledWidth  = extWidth >> scaleX;
    unsigned ymargin      = _margin >> (_scaleChromaMargin ? scaleY : 0);
    unsigned xmargin      = _margin >> (_scaleChromaMargin ? scaleX : 0);
    unsigned totalWidth   = scaledWidth + 2 * xmargin;
    unsigned totalHeight  = scaledHeight + 2 * ymargin;

    if (_alignment)
    {
      // make sure buffer lines are align
      CHECK(_alignment != MEMORY_ALIGN_DEF_SIZE, "Unsupported alignment");
      totalWidth = ((totalWidth + _alignment - 1) / _alignment) * _alignment;
    }
    uint32_t area = totalWidth * totalHeight;
    CHECK(!area, "Trying to create a buffer with zero area");

    m_origin[i]  = (Pel *)xMalloc(Pel, area);
    Pel *topLeft = m_origin[i] + totalWidth * ymargin + xmargin;
    bufs.push_back(PelBuf(topLeft, totalWidth, _area.width >> scaleX, _area.height >> scaleY));
  }

  m_maxArea = UnitArea(_chromaFormat, _area);
}

void PelStorage::createFromBuf(PelUnitBuf buf)
{
  chromaFormat = buf.chromaFormat;

  const uint32_t numCh = ::getNumberValidComponents(chromaFormat);

  bufs.resize(numCh);

  for (uint32_t i = 0; i < numCh; i++)
  {
    PelBuf cPelBuf = buf.get(CompID(i));
    bufs[i]        = PelBuf(cPelBuf.bufAt(0, 0), cPelBuf.stride, cPelBuf.width, cPelBuf.height);
  }

  m_maxArea = UnitArea(chromaFormat, Area(Position { 0, 0 }, buf.bufs[0]));
}

void PelStorage::swap(PelStorage &other)
{
  const uint32_t numCh = ::getNumberValidComponents(chromaFormat);

  for (uint32_t i = 0; i < numCh; i++)
  {
    // check this otherwise it would turn out to get very weird
    CHECK(chromaFormat != other.chromaFormat, "Incompatible formats");
    CHECK(get(CompID(i)) != other.get(CompID(i)), "Incompatible formats");
    CHECK(get(CompID(i)).stride != other.get(CompID(i)).stride, "Incompatible formats");

    std::swap(bufs[i].buf, other.bufs[i].buf);
    std::swap(bufs[i].stride, other.bufs[i].stride);
    std::swap(m_origin[i], other.m_origin[i]);
  }

  std::swap(m_maxArea, other.m_maxArea);
}

void PelStorage::destroy()
{
  chromaFormat = ChromaFormat::UNDEFINED;
  for (uint32_t i = 0; i < MAX_NUM_COMP; i++)
  {
    if (m_origin[i])
    {
      xFree(m_origin[i]);
      m_origin[i] = nullptr;
    }
  }
  bufs.clear();
}

void PelStorage::compactResize(const UnitArea &area)
{
  CHECK(bufs.size() < area.blocks.size(), "Cannot increase buffer size when compacting!");

  for (uint32_t i = 0; i < area.blocks.size(); i++)
  {
    CHECK(m_maxArea.blocks[i].area() < area.blocks[i].area(), "Cannot increase buffer size when compacting!");

    bufs[i].Size::operator=(area.blocks[i].size());
    bufs[i].stride = bufs[i].width;
  }
}

PelUnitBuf PelStorage::getCompactBuf(const UnitArea &unit)
{
  CHECKD(unit.lwidth() > bufs[COMP_Y].width && unit.lheight() > bufs[COMP_Y].height, "unsuported request");

  PelUnitBuf ret;
  ret.chromaFormat = chromaFormat;
  ret.bufs.resize(chromaFormat == ChromaFormat::_400 ? 1 : 3);

  ret.Y().buf   = bufs[COMP_Y].buf;
  ret.Y().width = ret.Y().stride = unit.Y().width;
  ret.Y().height                 = unit.Y().height;
  if (chromaFormat != ChromaFormat::_400)
  {
    ret.Cb().buf   = bufs[COMP_Cb].buf;
    ret.Cb().width = ret.Cb().stride = unit.Cb().width;
    ret.Cb().height                  = unit.Cb().height;
    ret.Cr().buf                     = bufs[COMP_Cr].buf;
    ret.Cr().width = ret.Cr().stride = unit.Cr().width;
    ret.Cr().height                  = unit.Cr().height;
  }

  return ret;
}

PelBuf PelStorage::getCompactBuf(const CompArea &carea) { return PelBuf(bufs[carea.compID].buf, carea.width, carea); }

PelUnitBufPool::PelUnitBufPool() {}

PelUnitBufPool::~PelUnitBufPool() {}

void PelUnitBufPool::initPelUnitBufPool(ChromaFormat chromaFormat, int ctuWidth, int ctuHeight)
{
  m_chromaFormat   = chromaFormat;
  m_ctuArea.x      = 0;
  m_ctuArea.y      = 0;
  m_ctuArea.width  = ctuWidth;
  m_ctuArea.height = ctuHeight;
}

PelUnitBuf *PelUnitBufPool::getPelUnitBuf(const UnitArea &unitArea)
{
  PelStorage *pelStorage = m_pelStoragePool.get();
  if (pelStorage->bufs.empty())
  {
    pelStorage->create(m_chromaFormat, m_ctuArea);
  }

  PelUnitBuf *pelUnitBuf = m_pelUnitBufPool.get();
  *pelUnitBuf            = pelStorage->getCompactBuf(unitArea);   // stride==width different than getBuf

  if (m_map.find(pelUnitBuf) == m_map.end())
  {
    m_map[pelUnitBuf] = pelStorage;
  }
  else
  {
    CHECK(m_map[pelUnitBuf] != pelStorage, "Wrong mapping in PelUnitBufPool");
  }

  return pelUnitBuf;
}

void PelUnitBufPool::giveBack(PelUnitBuf *p)
{
  if (p)
  {
    CHECK(m_map.find(p) == m_map.end(), "Unknown PelUnitBuf in PelUnitBufPool");
    m_pelStoragePool.giveBack(m_map[p]);
    m_pelUnitBufPool.giveBack(p);
  }
}
