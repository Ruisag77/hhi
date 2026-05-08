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

/** \file     Buffer.h
 *  \brief    Low-overhead class describing 2D memory layout
 */

#ifndef __BUFFER__
#define __BUFFER__

#include "Common.h"
#include "CommonDef.h"
#include "ChromaFormat.h"
#include "MotionInfo.h"

#include <string.h>
#include <type_traits>
#include <typeinfo>
#include <unordered_map>

// ---------------------------------------------------------------------------
// AreaBuf struct
// ---------------------------------------------------------------------------

struct PelBufferOps
{
  PelBufferOps();

#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
  void                         initPelBufOpsX86();
  template<X86_VEXT vext> void _initPelBufOpsX86();
#endif
  void (*roundBD)(const Pel *srcp, const ptrdiff_t srcStride, Pel *dest, const ptrdiff_t destStride, int width,
                  int height, const ClpRng &clpRng);
  void (*weightedAvg)(const Pel *src0, const ptrdiff_t src0Stride, const Pel *src1, const ptrdiff_t src1Stride,
                      Pel *dest, const ptrdiff_t destStride, const int8_t w0, const int8_t w1, int shift, int offset,
                      int width, int height, const ClpRng &clpRng);
  void (*weightedAvg3)(const Pel *src0, const ptrdiff_t src0Stride, const Pel *src1, const ptrdiff_t src1Stride,
                       const Pel *src2, const ptrdiff_t src2Stride, Pel *dest, const ptrdiff_t destStride,
                       const int8_t w0, const int8_t w1, const int8_t w2, int shift, int offset, int width, int height,
                       const ClpRng &clpRng);

  void (*copyClip)(const Pel *srcp, const ptrdiff_t srcStride, Pel *dest, const ptrdiff_t destStride, int width,
                   int height, const ClpRng &clpRng);
  void (*addAvg4)(const Pel *src0, ptrdiff_t src0Stride, const Pel *src1, ptrdiff_t src1Stride, Pel *dst,
                  ptrdiff_t dstStride, int width, int height, int shift, int offset, const ClpRng &clpRng,
                  bool *mcMask[NUM_RPL01], int mcStride, bool *isOOB);
  void (*addAvg8)(const Pel *src0, ptrdiff_t src0Stride, const Pel *src1, ptrdiff_t src1Stride, Pel *dst,
                  ptrdiff_t dstStride, int width, int height, int shift, int offset, const ClpRng &clpRng,
                  bool *mcMask[NUM_RPL01], int mcStride, bool *isOOB);
  void (*toLast2)(Pel *src, ptrdiff_t srcStride, int width, int height, int shiftNum, int offset, const ClpRng &clpRng);
  void (*toLast4)(Pel *src, ptrdiff_t srcStride, int width, int height, int shiftNum, int offset, const ClpRng &clpRng);
  void (*licRemoveWeightHighFreq2)(const Pel *src0, const Pel *src1, Pel *dst, int length, int w0, int w1, int offset,
                                   const ClpRng &clpRng);
  void (*licRemoveWeightHighFreq4)(const Pel *src0, const Pel *src1, Pel *dst, int length, int w0, int w1, int offset,
                                   const ClpRng &clpRng);
  void (*reco4)(const Pel *src0, ptrdiff_t src0Stride, const Pel *src1, ptrdiff_t src1Stride, Pel *dst,
                ptrdiff_t dstStride, int width, int height, const ClpRng &clpRng);
  void (*reco8)(const Pel *src0, ptrdiff_t src0Stride, const Pel *src1, ptrdiff_t src1Stride, Pel *dst,
                ptrdiff_t dstStride, int width, int height, const ClpRng &clpRng);
  void (*linTf4)(const Pel *src0, ptrdiff_t src0Stride, Pel *dst, ptrdiff_t dstStride, int width, int height, int scale,
                 int shift, int offset, const ClpRng &clpRng, bool bClip);
  void (*linTf8)(const Pel *src0, ptrdiff_t src0Stride, Pel *dst, ptrdiff_t dstStride, int width, int height, int scale,
                 int shift, int offset, const ClpRng &clpRng, bool bClip);
  void (*copyClip4)(const Pel *src0, ptrdiff_t src0Stride, Pel *dst, ptrdiff_t dstStride, int width, int height,
                    const ClpRng &clpRng);
  void (*copyClip8)(const Pel *src0, ptrdiff_t src0Stride, Pel *dst, ptrdiff_t dstStride, int width, int height,
                    const ClpRng &clpRng);
  void (*calcBIOParameterHighPrecision)(const Pel *srcY0Tmp, const Pel *srcY1Tmp, Pel *gradX0, Pel *gradX1, Pel *gradY0,
                                        Pel *gradY1, int width, int height, const int src0Stride, const int src1Stride,
                                        const int widthG, const int bitDepth, int32_t *s1, int32_t *s2, int32_t *s3,
                                        int32_t *s5, int32_t *s6, Pel *dI, Pel *gX, Pel *gY);
  void (*calcBIOParamSum4HighPrecision)(int32_t *s1, int32_t *s2, int32_t *s3, int32_t *s5, int32_t *s6, int width,
                                        int height, const int widthG, int32_t *sumS1, int32_t *sumS2, int32_t *sumS3,
                                        int32_t *sumS5, int32_t *sumS6, Pel *dI, Pel *gX, Pel *gY, bool isGPM,
                                        bool isSub);
  void (*calcBIOParameter)(const Pel *srcY0Tmp, const Pel *srcY1Tmp, Pel *gradX0, Pel *gradX1, Pel *gradY0, Pel *gradY1,
                           int width, int height, const int src0Stride, const int src1Stride, const int widthG,
                           const int bitDepth, Pel *absGX, Pel *absGY, Pel *dIX, Pel *dIY, Pel *signGY_GX, Pel *dI);
  void (*calAbsSum)(const Pel *diff, int stride, int width, int height, int *absDiff);
  void (*calcBIOParamSum5)(Pel *absGX, Pel *absGY, Pel *dIX, Pel *dIY, Pel *signGY_GX, const int widthG,
                           const int width, const int height, int *sumAbsGX, int *sumAbsGY, int *sumDIX, int *sumDIY,
                           int *sumSignGY_GX);
  void (*calcBIOParamSum5NOSIM4)(int32_t *absGX, int32_t *absGY, int32_t *dIX, int32_t *dIY, int32_t *signGyGx,
                                 const int widthG, const int width, const int height, int *sumAbsGX, int *sumAbsGY,
                                 int *sumDIX, int *sumDIY, int *sumSignGyGx, Pel *dI, Pel *gX, Pel *gY);
  void (*calcBIOParamSum5NOSIM8)(int32_t *absGX, int32_t *absGY, int32_t *dIX, int32_t *dIY, int32_t *signGyGx,
                                 const int widthG, const int width, const int height, int *sumAbsGX, int *sumAbsGY,
                                 int *sumDIX, int *sumDIY, int *sumSignGyGx, Pel *dI, Pel *gX, Pel *gY);
  void (*calcBIOParamSum4)(Pel *absGX, Pel *absGY, Pel *dIX, Pel *dIY, Pel *signGY_GX, int width, int height,
                           const int widthG, int *sumAbsGX, int *sumAbsGY, int *sumDIX, int *sumDIY, int *sumSignGY_GX);
  void (*addBIOAvgN)(const Pel *src0, int src0Stride, const Pel *src1, int src1Stride, Pel *dst, int dstStride,
                     const Pel *gradX0, const Pel *gradX1, const Pel *gradY0, const Pel *gradY1, int gradStride,
                     int width, int height, int *tmpx, int *tmpy, int shift, int offset, const ClpRng &clpRng,
                     bool *mcMask[2], int mcStride, bool *isOOB);
  void (*calcBIOClippedVxVy)(int *sumDIX_pixel_32bit, int *sumAbsGX_pixel_32bit, int *sumDIY_pixel_32bit,
                             int *sumAbsGY_pixel_32bit, int *sumSignGY_GX_pixel_32bit, const int limit,
                             const int bioSubblockSize, int *tmpx_pixel_32bit, int *tmpy_pixel_32bit);
  void (*bioGradFilter)(Pel *pSrc, ptrdiff_t srcStride, int width, int height, ptrdiff_t gradStride, Pel *gradX,
                        Pel *gradY, const int bitDepth);
  void (*copyBuffer)(const Pel *src, ptrdiff_t srcStride, Pel *dst, ptrdiff_t dstStride, int width, int height);
#if ENABLE_SIMD_OPT_BCW
  void (*removeWeightHighFreq8)(Pel *src0, ptrdiff_t src0Stride, const Pel *src1, ptrdiff_t src1Stride, int width,
                                int height, int bcwWeight, const Pel minVal, const Pel maxVal);
  void (*removeWeightHighFreq4)(Pel *src0, ptrdiff_t src0Stride, const Pel *src1, ptrdiff_t src1Stride, int width,
                                int height, int bcwWeight, const Pel minVal, const Pel maxVal);
  void (*removeHighFreq8)(Pel *src0, ptrdiff_t src0Stride, const Pel *src1, ptrdiff_t src1Stride, int width,
                          int height);
  void (*removeHighFreq4)(Pel *src0, ptrdiff_t src0Stride, const Pel *src1, ptrdiff_t src1Stride, int width,
                          int height);
#endif
  void (*mipMatrixMul_4_4)(Pel *res, const Pel *input, const uint8_t *weight, const int maxVal, const int offset,
                           bool transpose);
  void (*mipMatrixMul_8_4)(Pel *res, const Pel *input, const uint8_t *weight, const int maxVal, const int offset,
                           bool transpose);
  void (*mipMatrixMul_8_8)(Pel *res, const Pel *input, const uint8_t *weight, const int maxVal, const int offset,
                           bool transpose);

  void (*profGradFilter)(Pel *pSrc, ptrdiff_t srcStride, int width, int height, ptrdiff_t gradStride, Pel *gradX,
                         Pel *gradY, const int bitDepth);
  void (*applyPROF)(Pel *dst, ptrdiff_t dstStride, const Pel *src, ptrdiff_t srcStride, int width, int height,
                    const Pel *gradX, const Pel *gradY, ptrdiff_t gradStride, const int *dMvX, const int *dMvY,
                    ptrdiff_t dMvStride, const bool bi, int shiftNum, Pel offset, const ClpRng &clpRng);
  void (*roundIntVector)(int *v, int size, unsigned int nShift, const int dmvLimit);
  bool (*isMvOOB)(const Mv &rcMv, const Position pos, const Size size, const SPS *sps, const PPS *pps, bool *mcMask,
                  bool *mcMaskChroma, bool lumaOnly, ChromaFormat componentID);
  bool (*isMvOOBSubBlk)(const Mv &rcMv, const Position pos, const Size size, const SPS *sps, const PPS *pps,
                        bool *mcMask, int mcStride, bool *mcMaskChroma, int mcCStride, bool lumaOnly,
                        ChromaFormat componentID);
  void (*computeDeltaAndShift)(const Position posLT, Mv firstMv, std::vector<RMVFInfo> &mvpInfoVecOri);
  void (*computeDeltaAndShiftAddi)(const Position posLT, Mv firstMv, std::vector<RMVFInfo> &mvpInfoVecOri,
                                   std::vector<RMVFInfo> &mvpInfoVecRes);
  void (*buildRegressionMatrix)(std::vector<RMVFInfo> &mvpInfoVecOri, int64_t sumbb[2][3][3], int64_t sumeb[2][3],
                                uint16_t addedSize);
};

extern PelBufferOps g_pelBufOP;

int  getMean(int sum, int div);
void paddingCore(Pel *ptr, ptrdiff_t stride, int width, int height, int padSize);
void copyBufferCore(const Pel *src, ptrdiff_t srcStride, Pel *Dst, ptrdiff_t dstStride, int width, int height);

template<typename T> struct AreaBuf : public Size
{
  T        *buf;
  ptrdiff_t stride;
  // the proper type causes awful lot of errors
  // ptrdiff_t stride;

  AreaBuf() : Size(), buf(nullptr), stride(0) {}
  AreaBuf(T *_buf, const Size &size) : Size(size), buf(_buf), stride(size.width) {}
  AreaBuf(T *_buf, const ptrdiff_t &_stride, const Size &size) : Size(size), buf(_buf), stride(_stride) {}
  AreaBuf(T *_buf, const SizeType &_width, const SizeType &_height) : Size(_width, _height), buf(_buf), stride(_width)
  {}
  AreaBuf(T *_buf, const ptrdiff_t &_stride, const SizeType &_width, const SizeType &_height)
    : Size(_width, _height)
    , buf(_buf)
    , stride(_stride)
  {}

  operator AreaBuf<const T>() const { return AreaBuf<const T>(buf, stride, width, height); }

  void fill(const T &val);
  void memset(const int val);

  void copyFrom(const AreaBuf<const T> &other);
  void padCopyFrom(const AreaBuf<const T> &other, int w, int h, int pw, int ph);
  void copyTranspose(const AreaBuf<const T> &other);
  void copyFromFill(const AreaBuf<const T> &other, int w, int h, T fill);
  void roundToOutputBitdepth(const AreaBuf<const T> &src, const ClpRng &clpRng);

  void reconstruct(const AreaBuf<const T> &pred, const AreaBuf<const T> &resi, const ClpRng &clpRng);
  void copyClip(const AreaBuf<const T> &src, const ClpRng &clpRng);

  void subtract(const AreaBuf<const T> &other);
  void extendSingleBorderPel();
  void extendBorderPel(unsigned margin);
  void extendBorderPel(unsigned marginX, unsigned marginY);
  void padBorderPel(unsigned marginX, unsigned marginY, int dir);
  void addMirrorExtension(const unsigned marginX, const unsigned marginY);
  void addMirrorExtension(const unsigned margin);
  void addWeightedAvg(const AreaBuf<const T> &other1, const AreaBuf<const T> &other2, const ClpRng &clpRng,
                      const int8_t bcwIdx, bool *mcMask[NUM_RPL01], int mcStride, bool *isOOB);
  void removeWeightHighFreq(const AreaBuf<T> &other, const bool bClip, const ClpRng &clpRng, const int8_t iBcwWeight);
  void addAvg(const AreaBuf<const T> &other1, const AreaBuf<const T> &other2, const ClpRng &clpRng,
              bool *mcMask[NUM_RPL01], int mcStride, bool *isOOB);
  void removeHighFreq(const AreaBuf<T> &other, const bool bClip, const ClpRng &clpRng);
  void updateHistogram(std::vector<int32_t> &hist) const;
  void subtractHistogram(std::vector<int32_t> &hist) const;

  T    meanDiff(const AreaBuf<const T> &other) const;
  void subtract(const T val);

  void linearTransform(const int scale, const int shift, const int offset, bool bClip, const ClpRng &clpRng);

  void transposedFrom(const AreaBuf<const T> &other);

  void toLast(const ClpRng &clpRng);

  void    rspSignal(const std::vector<Pel> &pLUT);
  void    rspSignal(const AreaBuf<Pel> &toReshape, std::vector<Pel> &pLUT);
  void    scaleSignal(const int scale, const bool dir, const ClpRng &clpRng);
  void    applyLumaCTI(std::vector<Pel> &pLUTY);
  void    applyChromaCTI(Pel *bufY, ptrdiff_t strideY, std::vector<Pel> &pLUTUV, int bitDepth, ChromaFormat chrFormat,
                         bool fwdMap);
  T       computeAvg() const;
  int64_t computeAbsSum() const;

  T       &at(const int &x, const int &y) { return buf[y * stride + x]; }
  const T &at(const int &x, const int &y) const { return buf[y * stride + x]; }

  T       &at(const Position &pos) { return buf[pos.y * stride + pos.x]; }
  const T &at(const Position &pos) const { return buf[pos.y * stride + pos.x]; }

  T       *bufAt(const int &x, const int &y) { return &at(x, y); }
  const T *bufAt(const int &x, const int &y) const { return &at(x, y); }

  T       *bufAt(const Position &pos) { return &at(pos); }
  const T *bufAt(const Position &pos) const { return &at(pos); }

  AreaBuf<T>       subBuf(const Position &pos, const Size &size) { return AreaBuf<T>(bufAt(pos), stride, size); }
  AreaBuf<const T> subBuf(const Position &pos, const Size &size) const
  {
    return AreaBuf<const T>(bufAt(pos), stride, size);
  }
  AreaBuf<T> subBuf(const int &x, const int &y, const unsigned &_w, const unsigned &_h)
  {
    return AreaBuf<T>(bufAt(x, y), stride, _w, _h);
  }
  AreaBuf<const T> subBuf(const int &x, const int &y, const unsigned &_w, const unsigned &_h) const
  {
    return AreaBuf<const T>(bufAt(x, y), stride, _w, _h);
  }
};

typedef AreaBuf<Pel>       PelBuf;
typedef AreaBuf<const Pel> CPelBuf;

typedef AreaBuf<TCoeff>       CoeffBuf;
typedef AreaBuf<const TCoeff> CCoeffBuf;

typedef AreaBuf<MotionInfo>       MotionBuf;
typedef AreaBuf<const MotionInfo> CMotionBuf;

typedef AreaBuf<TCoeff>       PLTescapeBuf;
typedef AreaBuf<const TCoeff> CPLTescapeBuf;

typedef AreaBuf<bool>       PLTtypeBuf;
typedef AreaBuf<const bool> CPLTtypeBuf;

typedef AreaBuf<SplitPred>       QTDepthBuf;
typedef AreaBuf<const SplitPred> CQTDepthBuf;

typedef AreaBuf<int>       EipModelIdxBuf;
typedef AreaBuf<const int> CEipModelIdxBuf;

#define SIZE_AWARE_PER_EL_OP(OP, INC)    \
  if ((width & 7) == 0)                  \
  {                                      \
    for (int y = 0; y < height; y++)     \
    {                                    \
      for (int x = 0; x < width; x += 8) \
      {                                  \
        OP(x + 0);                       \
        OP(x + 1);                       \
        OP(x + 2);                       \
        OP(x + 3);                       \
        OP(x + 4);                       \
        OP(x + 5);                       \
        OP(x + 6);                       \
        OP(x + 7);                       \
      }                                  \
                                         \
      INC;                               \
    }                                    \
  }                                      \
  else if ((width & 3) == 0)             \
  {                                      \
    for (int y = 0; y < height; y++)     \
    {                                    \
      for (int x = 0; x < width; x += 4) \
      {                                  \
        OP(x + 0);                       \
        OP(x + 1);                       \
        OP(x + 2);                       \
        OP(x + 3);                       \
      }                                  \
                                         \
      INC;                               \
    }                                    \
  }                                      \
  else if ((width & 1) == 0)             \
  {                                      \
    for (int y = 0; y < height; y++)     \
    {                                    \
      for (int x = 0; x < width; x += 2) \
      {                                  \
        OP(x + 0);                       \
        OP(x + 1);                       \
      }                                  \
                                         \
      INC;                               \
    }                                    \
  }                                      \
  else                                   \
  {                                      \
    for (int y = 0; y < height; y++)     \
    {                                    \
      for (int x = 0; x < width; x++)    \
      {                                  \
        OP(x);                           \
      }                                  \
                                         \
      INC;                               \
    }                                    \
  }

template<typename T> void AreaBuf<T>::fill(const T &val)
{
  if (width == stride)
  {
    std::fill_n(buf, width * height, val);
  }
  else
  {
    T *dest = buf;

    for (unsigned y = 0; y < height; y++)
    {
      std::fill_n(dest, width, val);

      dest += stride;
    }
  }
}

template<typename T> void AreaBuf<T>::memset(const int val)
{
  if (width == stride)
  {
    std::fill_n(reinterpret_cast<char *>(buf), width * height * sizeof(T), val);
  }
  else
  {
    T *dest = buf;

    for (int y = 0; y < height; y++)
    {
      std::fill_n(reinterpret_cast<char *>(dest), width * sizeof(T), val);

      dest += stride;
    }
  }
}

template<typename T> void AreaBuf<T>::copyFrom(const AreaBuf<const T> &other)
{
#if !defined(__GNUC__) || __GNUC__ > 5
  static_assert(std::is_trivially_copyable<T>::value, "Type T is not trivially_copyable");
#endif

  CHECK(width != other.width, "Incompatible size");
  CHECK(height != other.height, "Incompatible size");

  if (buf == other.buf)
  {
    return;
  }

  if (width == stride && stride == other.stride)
  {
    memcpy(buf, other.buf, width * height * sizeof(T));
  }
  else
  {
    T              *dst       = buf;
    const T        *src       = other.buf;
    const ptrdiff_t srcStride = other.stride;

    for (unsigned y = 0; y < height; y++)
    {
      memcpy(dst, src, width * sizeof(T));

      dst += stride;
      src += srcStride;
    }
  }
}

template<typename T> void AreaBuf<T>::padCopyFrom(const AreaBuf<const T> &other, int w, int h, int pw, int ph)
{
  subBuf(Position(0, 0), Size(w - pw, h - ph)).copyFrom(other.subBuf(Position(0, 0), Size(w - pw, h - ph)));

  if (pw)
  {
    for (auto i = 0; i < h - ph; ++i)
    {
      subBuf(Position(w - pw, i), Size(pw, 1)).fill(other.at(w - pw - 1, i));
    }
  }
  if (ph)
  {
    for (auto i = 0; i < w - pw; ++i)
    {
      subBuf(Position(i, h - ph), Size(1, ph)).fill(other.at(i, h - ph - 1));
    }
  }
  if (pw && ph)
  {
    subBuf(Position(w - pw, h - ph), Size(pw, ph)).fill(other.at(w - pw - 1, h - ph - 1));
  }
}

template<typename T> void AreaBuf<T>::copyTranspose(const AreaBuf<const T> &other)
{
  int tw = width;
  int th = height;
  int sw = other.width;
  int sh = other.height;

  if (tw != sh || th != sw)
  {
    CHECK(1, "Size not compatible");
  }

  for (auto i = 0; i < th; ++i)
  {
    for (auto j = 0; j < tw; ++j)
    {
      at(j, i) = other.at(i, j);
    }
  }
}
template<typename T> void AreaBuf<T>::copyFromFill(const AreaBuf<const T> &other, int w, int h, T fill)
{
  int pw = w - (int)other.width;
  int ph = h - (int)other.height;

  CHECK(pw < 0 || ph < 0, "Bad source/target buffer specified!");

  subBuf(Position(0, 0), Size(w - pw, h - ph)).copyFrom(other.subBuf(Position(0, 0), Size(w - pw, h - ph)));

  if (pw)
  {
    for (auto i = 0; i < h - ph; ++i)
    {
      subBuf(Position(w - pw, i), Size(pw, 1)).fill(fill);
    }
  }
  if (ph)
  {
    for (auto i = 0; i < w - pw; ++i)
    {
      subBuf(Position(i, h - ph), Size(1, ph)).fill(fill);
    }
  }
  if (pw && ph)
  {
    subBuf(Position(w - pw, h - ph), Size(pw, ph)).fill(fill);
  }
}

template<typename T> void AreaBuf<T>::subtract(const AreaBuf<const T> &other)
{
  CHECK(width != other.width, "Incompatible size");
  CHECK(height != other.height, "Incompatible size");

  T       *dest = buf;
  const T *subs = other.buf;

#define SUBS_INC  \
  dest += stride; \
  subs += other.stride;

#define SUBS_OP(ADDR) dest[ADDR] -= subs[ADDR]

  SIZE_AWARE_PER_EL_OP(SUBS_OP, SUBS_INC);

#undef SUBS_OP
#undef SUBS_INC
}

template<typename T> void AreaBuf<T>::copyClip(const AreaBuf<const T> &src, const ClpRng &clpRng)
{
  THROW("Type not supported");
}

template<> void AreaBuf<Pel>::copyClip(const AreaBuf<const Pel> &src, const ClpRng &clpRng);

template<typename T>
void AreaBuf<T>::reconstruct(const AreaBuf<const T> &pred, const AreaBuf<const T> &resi, const ClpRng &clpRng)
{
  THROW("Type not supported");
}

template<>
void AreaBuf<Pel>::reconstruct(const AreaBuf<const Pel> &pred, const AreaBuf<const Pel> &resi, const ClpRng &clpRng);

template<typename T> void AreaBuf<T>::addAvg(const AreaBuf<const T> &other1, const AreaBuf<const T> &other2,
                                             const ClpRng &clpRng, bool *mcMask[NUM_RPL01], int mcStride, bool *isOOB)
{
  THROW("Type not supported");
}

template<> void AreaBuf<Pel>::addAvg(const AreaBuf<const Pel> &other1, const AreaBuf<const Pel> &other2,
                                     const ClpRng &clpRng, bool *mcMask[NUM_RPL01], int mcStride, bool *isOOB);

template<typename T>
void AreaBuf<T>::linearTransform(const int scale, const int shift, const int offset, bool bClip, const ClpRng &clpRng)
{
  THROW("Type not supported");
}

template<> void AreaBuf<Pel>::linearTransform(const int scale, const int shift, const int offset, bool bClip,
                                              const ClpRng &clpRng);

template<typename T> void AreaBuf<T>::toLast(const ClpRng &clpRng) { THROW("Type not supported"); }

template<> void AreaBuf<Pel>::toLast(const ClpRng &clpRng);

template<typename T> void AreaBuf<T>::removeWeightHighFreq(const AreaBuf<T> &other, const bool clampToNominalRange,
                                                           const ClpRng &clpRng, const int8_t bcwWeight)
{
  const Pel      *src       = other.buf;
  const ptrdiff_t srcStride = other.stride;

  Pel            *dst       = buf;
  const ptrdiff_t dstStride = stride;

  const Pel minVal = clampToNominalRange ? clpRng.min : 5 * clpRng.min - 4 * clpRng.max;
  const Pel maxVal = clampToNominalRange ? clpRng.max : 5 * clpRng.max - 4 * clpRng.min;

#if ENABLE_SIMD_OPT_BCW
  if ((width & 7) == 0 && g_pelBufOP.removeWeightHighFreq8)
  {
    g_pelBufOP.removeWeightHighFreq8(dst, dstStride, src, srcStride, width, height, bcwWeight, minVal, maxVal);
  }
  else if ((width & 3) == 0 && g_pelBufOP.removeWeightHighFreq4)
  {
    g_pelBufOP.removeWeightHighFreq4(dst, dstStride, src, srcStride, width, height, bcwWeight, minVal, maxVal);
  }
  else
#endif
  {
    const int32_t w =
      ((BCW_WEIGHT_BASE << BCW_INV_BITS) + (bcwWeight > 0 ? (bcwWeight >> 1) : -(bcwWeight >> 1))) / bcwWeight;

#define REM_HF_INC  \
  src += srcStride; \
  dst += dstStride;

#define REM_HF_OP_CLIP(ADDR) \
  dst[ADDR] =                \
    Clip3<T>(minVal, maxVal, (((dst[ADDR] - src[ADDR]) * w + (1 << BCW_INV_BITS >> 1)) >> BCW_INV_BITS) + src[ADDR])
    SIZE_AWARE_PER_EL_OP(REM_HF_OP_CLIP, REM_HF_INC);
#undef REM_HF_OP_CLIP
#undef REM_HF_INC
  }
}

template<typename T>
void AreaBuf<T>::removeHighFreq(const AreaBuf<T> &other, const bool clampToNominalRange, const ClpRng &clpRng)
{
  const T        *src       = other.buf;
  const ptrdiff_t srcStride = other.stride;

  T              *dst       = buf;
  const ptrdiff_t dstStride = stride;

#define REM_HF_INC  \
  src += srcStride; \
  dst += dstStride;

  if (!clampToNominalRange)
  {
#if ENABLE_SIMD_OPT_BCW
    if (!(width & 7) && g_pelBufOP.removeHighFreq8)
    {
      g_pelBufOP.removeHighFreq8(dst, dstStride, src, srcStride, width, height);
      return;
    }
    else if (!(width & 3) && g_pelBufOP.removeHighFreq4)
    {
      g_pelBufOP.removeHighFreq4(dst, dstStride, src, srcStride, width, height);
      return;
    }
#endif
#define REM_HF_OP(ADDR) dst[ADDR] = 2 * dst[ADDR] - src[ADDR]
    SIZE_AWARE_PER_EL_OP(REM_HF_OP, REM_HF_INC);
  }
  else
  {
#define REM_HF_OP_CLIP(ADDR) dst[ADDR] = ClipPel<T>(2 * dst[ADDR] - src[ADDR], clpRng)
    SIZE_AWARE_PER_EL_OP(REM_HF_OP_CLIP, REM_HF_INC);
  }

#undef REM_HF_INC
#undef REM_HF_OP
#undef REM_HF_OP_CLIP
}

template<typename T> void AreaBuf<T>::updateHistogram(std::vector<int32_t> &hist) const
{
  const T *data = buf;
  for (std::size_t y = 0; y < height; y++, data += stride)
  {
    for (std::size_t x = 0; x < width; x++)
    {
      hist[data[x]]++;
    }
  }
}

template<typename T> void AreaBuf<T>::subtractHistogram(std::vector<int32_t> &hist) const
{
  const T *data = buf;
  for (std::size_t y = 0; y < height; y++, data += stride)
  {
    for (std::size_t x = 0; x < width; x++)
    {
      hist[data[x]]--;
    }
  }
}

/// Extend the buffer (outside) by repeating the boundary samples (inside).
template<typename T> void AreaBuf<T>::extendBorderPel(unsigned marginX, unsigned marginY)
{
  T        *p = buf;
  int       h = height;
  int       w = width;
  ptrdiff_t s = stride;

  CHECK((w + 2 * marginX) > s, "Size of buffer too small to extend");
  // do left and right margins
  for (int y = 0; y < h; y++)
  {
    for (int x = 0; x < marginX; x++)
    {
      *(p - marginX + x) = p[0];
      p[w + x]           = p[w - 1];
    }
    p += s;
  }

  // p is now the (0,height) (bottom left of image within bigger picture
  p -= (s + marginX);
  // p is now the (-margin, height-1)
  for (int y = 0; y < marginY; y++)
  {
    ::memcpy(p + (y + 1) * s, p, sizeof(T) * (w + (marginX << 1)));
  }

  // p is still (-marginX, height-1)
  p -= ((h - 1) * s);
  // p is now (-marginX, 0)
  for (int y = 0; y < marginY; y++)
  {
    ::memcpy(p - (y + 1) * s, p, sizeof(T) * (w + (marginX << 1)));
  }
}

/// Pad top-left or bottom-right margin (inside) by repeating samples from top or bottom margin (inside), respectively.
template<typename T> void AreaBuf<T>::padBorderPel(unsigned marginX, unsigned marginY, int dir)
{
  T        *p = buf;
  ptrdiff_t s = stride;
  int       h = height;
  int       w = width;

  CHECK(w > s, "Size of buffer too small to extend");

  // top-left margin
  if (dir == 1)
  {
    for (int y = 0; y < marginY; y++)
    {
      for (int x = 0; x < marginX; x++)
      {
        p[x] = p[marginX];
      }
      p += s;
    }
  }

  // bottom-right margin
  if (dir == 2)
  {
    p = buf + s * (h - marginY) + w - marginX;

    for (int y = 0; y < marginY; y++)
    {
      for (int x = 0; x < marginX; x++)
      {
        p[x] = p[-1];
      }
      p += s;
    }
  }
}

/// Extend the buffer (outside) by repeating the boundary samples (inside).
template<typename T> void AreaBuf<T>::extendBorderPel(unsigned margin)
{
  T        *p = buf;
  int       h = height;
  int       w = width;
  ptrdiff_t s = stride;

  CHECK((w + 2 * margin) > s, "Size of buffer too small to extend");
  // do left and right margins
  for (int y = 0; y < h; y++)
  {
    for (int x = 0; x < margin; x++)
    {
      *(p - margin + x) = p[0];
      p[w + x]          = p[w - 1];
    }
    p += s;
  }

  // p is now the (0,height) (bottom left of image within bigger picture
  p -= (s + margin);
  // p is now the (-margin, height-1)
  for (int y = 0; y < margin; y++)
  {
    ::memcpy(p + (y + 1) * s, p, sizeof(T) * (w + (margin << 1)));
  }

  // pi is still (-marginX, height-1)
  p -= ((h - 1) * s);
  // pi is now (-marginX, 0)
  for (int y = 0; y < margin; y++)
  {
    ::memcpy(p - (y + 1) * s, p, sizeof(T) * (w + (margin << 1)));
  }
}

/// Extend the buffer (outside) by mirroring the inside samples.
template<typename T> void AreaBuf<T>::addMirrorExtension(const unsigned marginX, const unsigned marginY)
{
  const int       height_     = height;
  const int       width_      = width;
  const unsigned  paddedWidth = width + 2 * marginX;
  const ptrdiff_t stride_     = stride;
  T              *bufPtr      = buf;

  CHECK(width + 2 * marginX > stride, "Size of buffer is to small for padding.");

  // Do the left and right margins.
  for (int y = 0; y < height_; ++y)
  {
    for (int x = 0; x < marginX; ++x)
    {
      bufPtr[-x - 1]     = bufPtr[x];
      bufPtr[width_ + x] = bufPtr[width_ - x - 1];
    }
    bufPtr += stride_;
  }

  // bufPtr is now at (0, height).
  bufPtr -= (stride + marginX);
  // bufPtr is now at (-margin, height - 1).

  for (int y = 0; y < marginY; ++y)
  {
    std::copy_n(bufPtr - y * stride_, paddedWidth, bufPtr + (y + 1) * stride_);
  }

  // bufPtr is still at (-margin, height - 1).
  bufPtr -= ((height - 1) * stride);
  // bufPtr is now at (-margin, 0).

  for (int y = 0; y < marginY; ++y)
  {
    std::copy_n(bufPtr + y * stride_, paddedWidth, bufPtr - (y + 1) * stride);
  }
}

/// Extend the buffer (outside) by mirroring the inside samples.
template<typename T> void AreaBuf<T>::addMirrorExtension(const unsigned margin) { addMirrorExtension(margin, margin); }

template<typename T> T AreaBuf<T>::meanDiff(const AreaBuf<const T> &other) const
{
  int64_t acc     = 0;
  int32_t lineAcc = 0;

  CHECK(width != other.width, "Incompatible size");
  CHECK(height != other.height, "Incompatible size");

  const T *src1 = buf;
  const T *src2 = other.buf;

#define MEAN_DIFF_INC     \
  {                       \
    src1 += stride;       \
    src2 += other.stride; \
    acc += lineAcc;       \
    lineAcc = 0;          \
  }

#define MEAN_DIFF_OP(ADDR) lineAcc += src1[ADDR] - src2[ADDR]

  SIZE_AWARE_PER_EL_OP(MEAN_DIFF_OP, MEAN_DIFF_INC);

#undef MEAN_DIFF_INC
#undef MEAN_DIFF_OP

  return T(acc / area());
}

#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
template<> void AreaBuf<Pel>::subtract(const Pel val);
#endif

template<typename T> void AreaBuf<T>::subtract(const T val)
{
  T *dst = buf;

#define OFFSET_INC      dst += stride
#define OFFSET_OP(ADDR) dst[ADDR] -= val

  SIZE_AWARE_PER_EL_OP(OFFSET_OP, OFFSET_INC);

#undef OFFSET_INC
#undef OFFSET_OP
}

template<typename T> void AreaBuf<T>::transposedFrom(const AreaBuf<const T> &other)
{
  CHECK(width * height != other.width * other.height, "Incompatible size");

  T       *dst = buf;
  const T *src = other.buf;
  width        = other.height;
  height       = other.width;
  stride       = stride < width ? width : stride;

  for (unsigned y = 0; y < other.height; y++)
  {
    for (unsigned x = 0; x < other.width; x++)
    {
      dst[y + x * stride] = src[x + y * other.stride];
    }
  }
}

template<typename T> T AreaBuf<T>::computeAvg() const
{
  const T *src     = buf;
  int64_t  acc     = 0;
  int32_t  lineAcc = 0;
#define AVG_INC     \
  {                 \
    src += stride;  \
    acc += lineAcc; \
    lineAcc = 0;    \
  }
#define AVG_OP(ADDR) lineAcc += src[ADDR]
  SIZE_AWARE_PER_EL_OP(AVG_OP, AVG_INC);
#undef AVG_INC
#undef AVG_OP
  return T((acc + (area() >> 1)) / area());
}

template<typename T> int64_t AreaBuf<T>::computeAbsSum() const
{
  const T *src     = buf;
  int32_t  lineAcc = 0;
  int64_t  acc     = 0;   // for picture-wise use in getGlaringColorQPOffset() and applyQPAdaptationChroma()
#define AVG_INC     \
  {                 \
    src += stride;  \
    acc += lineAcc; \
    lineAcc = 0;    \
  }
#define AVG_OP(ADDR) lineAcc += std::abs(src[ADDR])
  SIZE_AWARE_PER_EL_OP(AVG_OP, AVG_INC);
#undef AVG_INC
#undef AVG_OP
  return acc;
}

#ifndef DONT_UNDEF_SIZE_AWARE_PER_EL_OP
#undef SIZE_AWARE_PER_EL_OP
#endif // !DONT_UNDEF_SIZE_AWARE_PER_EL_OP

// ---------------------------------------------------------------------------
// UnitBuf struct
// ---------------------------------------------------------------------------

struct UnitArea;

template<typename T> struct UnitBuf
{
  typedef static_vector<AreaBuf<T>, MAX_NUM_COMP>       UnitBufBuffers;
  typedef static_vector<AreaBuf<const T>, MAX_NUM_COMP> ConstUnitBufBuffers;

  ChromaFormat   chromaFormat;
  UnitBufBuffers bufs;

  UnitBuf() : chromaFormat(ChromaFormat::UNDEFINED) {}
  UnitBuf(const ChromaFormat &_chromaFormat, const UnitBufBuffers &_bufs) : chromaFormat(_chromaFormat), bufs(_bufs) {}
  UnitBuf(const ChromaFormat &_chromaFormat, UnitBufBuffers &&_bufs)
    : chromaFormat(_chromaFormat)
    , bufs(std::forward<UnitBufBuffers>(_bufs))
  {}
  UnitBuf(const ChromaFormat &_chromaFormat, const AreaBuf<T> &blkY) : chromaFormat(_chromaFormat), bufs { blkY } {}
  UnitBuf(const ChromaFormat &_chromaFormat, AreaBuf<T> &&blkY)
    : chromaFormat(_chromaFormat)
    , bufs { std::forward<AreaBuf<T>>(blkY) }
  {}
  UnitBuf(const ChromaFormat &_chromaFormat, const AreaBuf<T> &blkY, const AreaBuf<T> &blkCb, const AreaBuf<T> &blkCr)
    : chromaFormat(_chromaFormat)
    , bufs { blkY, blkCb, blkCr }
  {}
  UnitBuf(const ChromaFormat &_chromaFormat, AreaBuf<T> &&blkY, AreaBuf<T> &&blkCb, AreaBuf<T> &&blkCr)
    : chromaFormat(_chromaFormat)
    , bufs { std::forward<AreaBuf<T>>(blkY), std::forward<AreaBuf<T>>(blkCb), std::forward<AreaBuf<T>>(blkCr) }
  {}
  UnitBuf(const UnitArea &_area, T *bufY, T *bufCb, T *bufCr)
    : chromaFormat(_area.chromaFormat)
    , bufs { AreaBuf<T>(bufY, _area.Y().size()) }
  {
    if (chromaFormat != ChromaFormat::_400)
    {
      bufs.push_back(AreaBuf<T>(bufCb, _area.Cb().size()));
      bufs.push_back(AreaBuf<T>(bufCr, _area.Cr().size()));
    }
  }

  operator UnitBuf<const T>() const
  {
    return UnitBuf<const T>(chromaFormat, ConstUnitBufBuffers(bufs.begin(), bufs.end()));
  }

  AreaBuf<T>       &get(const CompID comp) { return bufs[comp]; }
  const AreaBuf<T> &get(const CompID comp) const { return bufs[comp]; }

  AreaBuf<T>       &Y() { return bufs[0]; }
  const AreaBuf<T> &Y() const { return bufs[0]; }
  AreaBuf<T>       &Cb() { return bufs[1]; }
  const AreaBuf<T> &Cb() const { return bufs[1]; }
  AreaBuf<T>       &Cr() { return bufs[2]; }
  const AreaBuf<T> &Cr() const { return bufs[2]; }

  void fill(const T &val);
  void copyFrom(const UnitBuf<const T> &other, const bool luma = true, const bool chroma = true);
  void roundToOutputBitdepth(const UnitBuf<const T> &src, const ClpRngs &clpRngs);
  void reconstruct(const UnitBuf<const T> &pred, const UnitBuf<const T> &resi, const ClpRngs &clpRngs);
  void copyClip(const UnitBuf<const T> &src, const ClpRngs &clpRngs, const bool luma = true, const bool chroma = true);
  void subtract(const UnitBuf<const T> &other);
  void addWeightedAvg(const UnitBuf<const T> &other1, const UnitBuf<const T> &other2, const ClpRngs &clpRngs,
                      const uint8_t bcwIdx = BCW_DEFAULT, const bool chromaOnly = false, const bool lumaOnly = false,
                      bool *mcMask[NUM_RPL01] = nullptr, int mcStride = -1, bool *mcMaskChroma[NUM_RPL01] = nullptr,
                      int mcCStride = -1, bool *isOOB = nullptr);
  void addAvg(const UnitBuf<const T> &other1, const UnitBuf<const T> &other2, const ClpRngs &clpRngs,
              const bool chromaOnly = false, const bool lumaOnly = false, bool *mcMask[NUM_RPL01] = nullptr,
              int mcStride = -1, bool *mcMaskChroma[NUM_RPL01] = nullptr, int mcCStride = -1, bool *isOOB = nullptr);
  void extendSingleBorderPel();
  void extendBorderPel(unsigned marginX, unsigned marginY);
  void padBorderPel(unsigned margin, int dir);
  void extendBorderPel(unsigned margin);
  void addMirrorExtension(const unsigned marginX, const unsigned marginY, const bool scaleChromaMargin = true);
  void addMirrorExtension(const unsigned margin, const bool scaleChromaMargin = true);
  void removeHighFreq(const UnitBuf<T> &other, const bool bClip, const ClpRngs &clpRngs,
                      const int8_t bcwWeight = g_BcwWeights[BCW_DEFAULT]);

  UnitBuf<T>             subBuf(const UnitArea &subArea);
  const UnitBuf<const T> subBuf(const UnitArea &subArea) const;
};

typedef UnitBuf<Pel>       PelUnitBuf;
typedef UnitBuf<const Pel> CPelUnitBuf;

typedef UnitBuf<TCoeff>       CoeffUnitBuf;
typedef UnitBuf<const TCoeff> CCoeffUnitBuf;

template<typename T> void UnitBuf<T>::fill(const T &val)
{
  for (unsigned i = 0; i < bufs.size(); i++)
  {
    bufs[i].fill(val);
  }
}

template<typename T> void UnitBuf<T>::copyFrom(const UnitBuf<const T> &other, const bool luma, const bool chroma)
{
  CHECK(chromaFormat != other.chromaFormat, "Incompatible formats");
  CHECK(!luma && !chroma, "At least one channel has to be selected");

  const size_t compStart = luma ? 0 : 1;
  const size_t compEnd   = chroma ? bufs.size() : 1;
  for (size_t i = compStart; i < compEnd; i++)
  {
    bufs[i].copyFrom(other.bufs[i]);
  }
}

template<typename T> void UnitBuf<T>::subtract(const UnitBuf<const T> &other)
{
  CHECK(chromaFormat != other.chromaFormat, "Incompatible formats");

  for (unsigned i = 0; i < bufs.size(); i++)
  {
    bufs[i].subtract(other.bufs[i]);
  }
}

template<typename T>
void UnitBuf<T>::copyClip(const UnitBuf<const T> &src, const ClpRngs &clpRngs, const bool luma, const bool chroma)
{
  CHECK(chromaFormat != src.chromaFormat, "Incompatible formats");
  CHECK(!luma && !chroma, "At least one channel has to be selected");

  const size_t compStart = luma ? 0 : 1;
  const size_t compEnd   = chroma ? bufs.size() : 1;
  for (size_t i = compStart; i < compEnd; i++)
  {
    bufs[i].copyClip(src.bufs[i], clpRngs.comp[i]);
  }
}

template<typename T> void UnitBuf<T>::roundToOutputBitdepth(const UnitBuf<const T> &src, const ClpRngs &clpRngs)
{
  CHECK(chromaFormat != src.chromaFormat, "Incompatible formats");

  for (unsigned i = 0; i < bufs.size(); i++)
  {
    bufs[i].roundToOutputBitdepth(src.bufs[i], clpRngs.comp[i]);
  }
}

template<typename T>
void UnitBuf<T>::reconstruct(const UnitBuf<const T> &pred, const UnitBuf<const T> &resi, const ClpRngs &clpRngs)
{
  CHECK(chromaFormat != pred.chromaFormat, "Incompatible formats");
  CHECK(chromaFormat != resi.chromaFormat, "Incompatible formats");

  for (unsigned i = 0; i < bufs.size(); i++)
  {
    bufs[i].reconstruct(pred.bufs[i], resi.bufs[i], clpRngs.comp[i]);
  }
}

template<typename T>
void UnitBuf<T>::addWeightedAvg(const UnitBuf<const T> &other1, const UnitBuf<const T> &other2, const ClpRngs &clpRngs,
                                const uint8_t bcwIdx /* = BCW_DEFAULT */, const bool chromaOnly /* = false */,
                                const bool lumaOnly /* = false */, bool *mcMask[NUM_RPL01], int mcStride,
                                bool *mcMaskChroma[NUM_RPL01], int mcCStride, bool *isOOB)
{
  const size_t istart = chromaOnly ? 1 : 0;
  const size_t iend   = lumaOnly ? 1 : bufs.size();

  CHECK(lumaOnly && chromaOnly, "should not happen");

  for (size_t i = istart; i < iend; i++)
  {
    bufs[i].addWeightedAvg(other1.bufs[i], other2.bufs[i], clpRngs.comp[i], bcwIdx, i == 0 ? mcMask : mcMaskChroma,
                           i == 0 ? mcStride : mcCStride, isOOB);
  }
}

template<typename T> void UnitBuf<T>::addAvg(const UnitBuf<const T> &other1, const UnitBuf<const T> &other2,
                                             const ClpRngs &clpRngs, const bool chromaOnly /* = false */,
                                             const bool lumaOnly /* = false */, bool *mcMask[NUM_RPL01], int mcStride,
                                             bool *mcMaskChroma[NUM_RPL01], int mcCStride, bool *isOOB)
{
  const size_t istart = chromaOnly ? 1 : 0;
  const size_t iend   = lumaOnly ? 1 : bufs.size();

  CHECK(lumaOnly && chromaOnly, "should not happen");

  for (size_t i = istart; i < iend; i++)
  {
    bufs[i].addAvg(other1.bufs[i], other2.bufs[i], clpRngs.comp[i], i == 0 ? mcMask : mcMaskChroma,
                   i == 0 ? mcStride : mcCStride, isOOB);
  }
}

template<typename T> void UnitBuf<T>::extendSingleBorderPel()
{
  for (unsigned i = 0; i < bufs.size(); i++)
  {
    bufs[i].extendSingleBorderPel();
  }
}

/// Extend the buffer (outside) by repeating the boundary samples (inside).
template<typename T> void UnitBuf<T>::extendBorderPel(unsigned marginX, unsigned marginY)
{
  for (unsigned i = 0; i < bufs.size(); i++)
  {
    bufs[i].extendBorderPel(marginX >> getComponentScaleX(CompID(i), chromaFormat),
                            marginY >> getComponentScaleY(CompID(i), chromaFormat));
  }
}

/// Pad top-left or bottom-right margin (inside) by repeating samples from top or bottom margin (inside), respectively.
template<typename T> void UnitBuf<T>::padBorderPel(unsigned margin, int dir)
{
  for (unsigned i = 0; i < bufs.size(); i++)
  {
    bufs[i].padBorderPel(margin >> getComponentScaleX(CompID(i), chromaFormat),
                         margin >> getComponentScaleY(CompID(i), chromaFormat), dir);
  }
}

/// Extend the buffer (outside) by repeating the boundary samples (inside).
template<typename T> void UnitBuf<T>::extendBorderPel(unsigned margin)
{
  for (unsigned i = 0; i < bufs.size(); i++)
  {
    bufs[i].extendBorderPel(margin);
  }
}

/// Extend the buffer (outside) by mirroring the inside samples.
template<typename T> void UnitBuf<T>::addMirrorExtension(const unsigned marginX, const unsigned marginY,
                                                         const bool scaleChromaMargin /* = true*/)
{
  const unsigned numComp = getNumberValidComponents(chromaFormat);
  for (unsigned compIdx = 0; compIdx < numComp; ++compIdx)
  {
    const CompID   compID        = CompID(compIdx);
    const unsigned scaleX        = ::getComponentScaleX(compID, chromaFormat);
    const unsigned scaleY        = ::getComponentScaleY(compID, chromaFormat);
    const unsigned scaledMarginX = marginX >> (scaleChromaMargin ? scaleX : 0);
    const unsigned scaledMarginY = marginY >> (scaleChromaMargin ? scaleY : 0);
    bufs[compIdx].addMirrorExtension(scaledMarginX, scaledMarginY);
  }
}

// Extend the buffer (outside) by mirroring the inside samples.
template<typename T>
void UnitBuf<T>::addMirrorExtension(const unsigned margin, const bool scaleChromaMargin /* = true*/)
{
  addMirrorExtension(margin, margin, scaleChromaMargin);
}

template<typename T> void UnitBuf<T>::removeHighFreq(const UnitBuf<T> &other, const bool bClip, const ClpRngs &clpRngs,
                                                     const int8_t bcwWeight)
{
  if (bcwWeight != g_BcwWeights[BCW_DEFAULT])
  {
    bufs[0].removeWeightHighFreq(other.bufs[0], bClip, clpRngs.comp[0], bcwWeight);
    return;
  }
  bufs[0].removeHighFreq(other.bufs[0], bClip, clpRngs.comp[0]);
}

template<typename T> UnitBuf<T> UnitBuf<T>::subBuf(const UnitArea &subArea)
{
  UnitBuf<T> subBuf;
  subBuf.chromaFormat = chromaFormat;
  unsigned blockIdx   = 0;

  for (auto &subAreaBuf: bufs)
  {
    subBuf.bufs.push_back(subAreaBuf.subBuf(subArea.blocks[blockIdx].pos(), subArea.blocks[blockIdx].size()));
    blockIdx++;
  }

  return subBuf;
}

template<typename T> const UnitBuf<const T> UnitBuf<T>::subBuf(const UnitArea &subArea) const
{
  UnitBuf<const T> subBuf;
  subBuf.chromaFormat = chromaFormat;
  unsigned blockIdx   = 0;

  for (const auto &subAreaBuf: bufs)
  {
    subBuf.bufs.push_back(subAreaBuf.subBuf(subArea.blocks[blockIdx].pos(), subArea.blocks[blockIdx].size()));
    blockIdx++;
  }

  return subBuf;
}

// ---------------------------------------------------------------------------
// PelStorage struct (PelUnitBuf which allocates its own memory)
// ---------------------------------------------------------------------------

struct UnitArea;
struct CompArea;

struct PelStorage : public PelUnitBuf
{
  PelStorage();
  ~PelStorage();

  void swap(PelStorage &other);
  void createFromBuf(PelUnitBuf buf);
  void create(const UnitArea &_unit);
  void create(const ChromaFormat &_chromaFormat, const Area &_area, const unsigned _maxCUSize = 0,
              const unsigned _margin = 0, const unsigned _alignment = 0, const bool _scaleChromaMargin = true);
  void destroy();
  void compactResize(const UnitArea &area);

  PelBuf getBuf(const CompID compID) { return bufs[compID]; }

  const CPelBuf getBuf(const CompID compID) const { return bufs[compID]; }

  PelBuf getBuf(const CompArea &blk)
  {
    const PelBuf &r = bufs[blk.compID];

    CHECKD(rsAddr(blk.bottomRight(), r.stride) >= ((r.height - 1) * r.stride + r.width),
           "Trying to access a buf outside of bound!");

    return PelBuf(r.buf + rsAddr(blk, r.stride), r.stride, blk);
  }

  const CPelBuf getBuf(const CompArea &blk) const
  {
    const PelBuf &r = bufs[blk.compID];
    return CPelBuf(r.buf + rsAddr(blk, r.stride), r.stride, blk);
  }

  PelUnitBuf getBuf(const UnitArea &unit)
  {
    return !isChromaEnabled(chromaFormat)
      ? PelUnitBuf(chromaFormat, getBuf(unit.Y()))
      : PelUnitBuf(chromaFormat, getBuf(unit.Y()), getBuf(unit.Cb()), getBuf(unit.Cr()));
  }

  const CPelUnitBuf getBuf(const UnitArea &unit) const
  {
    return !isChromaEnabled(chromaFormat)
      ? CPelUnitBuf(chromaFormat, getBuf(unit.Y()))
      : CPelUnitBuf(chromaFormat, getBuf(unit.Y()), getBuf(unit.Cb()), getBuf(unit.Cr()));
  }

  Pel *getOrigin(const int id) const { return m_origin[id]; }

  PelUnitBuf getCompactBuf(const UnitArea &unit);
  PelBuf     getCompactBuf(const CompArea &blk);

private:
  Pel     *m_origin[MAX_NUM_COMP];
  UnitArea m_maxArea;
};

template<typename T> struct AreaStorage : public AreaBuf<T>
{
  AreaStorage() { m_memory = nullptr; }
  ~AreaStorage()
  {
    if (valid())
    {
      xFree(m_memory);
    }
  }

  void create(const Size &size, const unsigned maxCUSize = 0, const unsigned margin = 0, const unsigned alignment = 0);

  void destroy()
  {
    if (valid())
    {
      xFree(m_memory);
    }
    m_memory = nullptr;
  }
  bool valid() { return m_memory != nullptr; }

private:
  T *m_memory;
};

template<typename T> void AreaStorage<T>::create(const Size &size, const unsigned maxCUSize /*= 0*/,
                                                 const unsigned margin /*= 0*/, const unsigned alignment /*= 0*/)
{
  CHECK(m_memory, "Trying to re-create an already initialized buffer");

  unsigned extHeight = size.height;
  unsigned extWidth  = size.width;

  if (maxCUSize)
  {
    extHeight = ((extHeight + maxCUSize - 1) / maxCUSize) * maxCUSize;
    extWidth  = ((extWidth + maxCUSize - 1) / maxCUSize) * maxCUSize;
  }

  if (margin)
  {
    extHeight += 2 * margin;
    extWidth += 2 * margin;
  }

  if (alignment)
  {
    // Ensure that buffer rows are aligned.
    CHECK(alignment != MEMORY_ALIGN_DEF_SIZE, "Unsupported alignment.");
    extWidth = ((extWidth + alignment - 1) / alignment) * alignment;
  }

  size_t area = extHeight * extWidth;
  CHECK(area == 0, "Trying to create a buffer with zero size.");

  m_memory = reinterpret_cast<T *>(xMalloc(T, area));

  T *const topLeft                 = m_memory + margin * extWidth + margin;
  *static_cast<AreaBuf<T> *>(this) = AreaBuf<T>(topLeft, extWidth, size);
}

using CompStorage = AreaStorage<Pel>;

class PelUnitBufPool
{
private:
  Pool<PelStorage>                               m_pelStoragePool;
  Pool<PelUnitBuf>                               m_pelUnitBufPool;
  std::unordered_map<PelUnitBuf *, PelStorage *> m_map;
  ChromaFormat                                   m_chromaFormat;
  Area                                           m_ctuArea;

public:
  PelUnitBufPool();
  ~PelUnitBufPool();

  void        initPelUnitBufPool(ChromaFormat chromaFormat, int ctuWidth, int ctuHeight);
  PelUnitBuf *getPelUnitBuf(const UnitArea &unitArea);
  void        giveBack(PelUnitBuf *p);

  template<size_t N> void giveBack(static_vector<PelUnitBuf *, N> &v)
  {
    for (auto p: v)
    {
      giveBack(p);
    }
    v.clear();
  }
};

template<size_t N> class PelUnitBufVector : public static_vector<PelUnitBuf *, N>
{
private:
  PelUnitBufPool *m_pool;

public:
  PelUnitBufVector(PelUnitBufPool &pool) : m_pool(&pool) {}
  ~PelUnitBufVector() { m_pool->giveBack(*this); }
};

struct SortedPelUnitBufs
{
  SortedPelUnitBufs(PelUnitBufPool &pelUnitBufPool)
    : m_maxEntries(0)
    , m_multiBuffers(false)
    , m_pelUnitBufPool(pelUnitBufPool)
  {}

  ~SortedPelUnitBufs()
  {
    for (auto buf: m_sortedList)
    {
      m_pelUnitBufPool.giveBack(buf);
    }
  }

  void prepare(const UnitArea &ua, bool multiBuffers, int reserveBufferSize = 128)
  {
    m_sortedList.resize(0);
    m_sortedList.reserve(reserveBufferSize);
    m_maxEntries = 0;
    m_ua         = ua;
    m_sortedList.push_back(m_pelUnitBufPool.getPelUnitBuf(m_ua));
    m_multiBuffers = multiBuffers;
  }

  const PelUnitBuf *getBufFromSortedList(unsigned idx) const
  {
    return m_maxEntries > idx ? m_sortedList[idx] : nullptr;
  }
  PelUnitBuf       *getBufFromSortedList(unsigned idx) { return m_maxEntries > idx ? m_sortedList[idx] : nullptr; }
  const PelUnitBuf &getTestBuf() const { return *m_sortedList[m_maxEntries]; }
  PelUnitBuf       &getTestBuf() { return *m_sortedList[m_maxEntries]; }

  void insert(int insertPos, size_t RdListSize, bool forceKeepInBuffer = false)
  {
    if (m_multiBuffers)
    {
      m_maxEntries++;
      m_sortedList.push_back(m_pelUnitBufPool.getPelUnitBuf(m_ua));
    }
    else if (forceKeepInBuffer)
    {
      m_maxEntries++;
      m_sortedList.push_back(m_pelUnitBufPool.getPelUnitBuf(m_ua));
    }
    else
    {
      m_maxEntries++;
      m_sortedList.push_back(nullptr);
      std::swap(m_sortedList[m_maxEntries - 1], m_sortedList[m_maxEntries]);
    }
  }

private:
  unsigned                  m_maxEntries;
  bool                      m_multiBuffers;
  UnitArea                  m_ua;
  std::vector<PelUnitBuf *> m_sortedList;
  PelUnitBufPool           &m_pelUnitBufPool;
};

#endif
