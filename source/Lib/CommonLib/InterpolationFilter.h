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
 * \brief Declaration of InterpolationFilter class
 */

#ifndef __INTERPOLATIONFILTER__
#define __INTERPOLATIONFILTER__

#include "CommonDef.h"
#include "CacheModel.h"

//! \ingroup CommonLib
//! \{

static constexpr int IF_INTERNAL_PREC = 14;                         ///< Number of bits for internal precision
#if IF_12TAP
static constexpr int IF_FILTER_PREC_8 = 8;                         ///< Log2 of sum of filter taps
#if IF_12TAP
static constexpr int IF_FILTER_PREC = IF_FILTER_PREC_8;          ///< Log2 of sum of filter taps
#else
static constexpr int IF_FILTER_PREC = 6;                         ///< Log2 of sum of filter taps
#endif
#else
static constexpr int IF_FILTER_PREC = 6;                         ///< Log2 of sum of filter taps
#endif
static constexpr int IF_INTERNAL_OFFS          = (1 << (IF_INTERNAL_PREC - 1));  ///< Offset used internally
static constexpr int IF_INTERNAL_PREC_BILINEAR = 10;                ///< Number of bits for internal precision
static constexpr int IF_FILTER_PREC_BILINEAR =
  4;   ///< Bilinear filter coeff precision so that intermediate value will not exceed 16 bit for SIMD - bit exact
static inline int IF_INTERNAL_FRAC_BITS(const int bd) { return std::max(2, IF_INTERNAL_PREC - bd); }
/**
 * \brief Interpolation filter class
 */
class InterpolationFilter
{
#if IF_12TAP
#else
  static const TFilterCoeff m_affineLumaFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA];
#endif
public:
#if IF_12TAP
  static const TFilterCoeff m_lumaFilter12[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1]
                                          [12];   ///< Luma filter taps //+1 added by //kolya
#endif
  static const TFilterCoeff m_lumaFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS]
                                        [8];   ///< Luma filter taps, for affine?
#if JVET_Z0117_CHROMA_IF
  static const TFilterCoeff m_chromaFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1]
                                          [NTAPS_CHROMA];   ///< Chroma filter taps
  static const TFilterCoeff m_chromaFilter4[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS]
                                           [4];   ///< Chroma filter taps
#else
  static const TFilterCoeff m_chromaFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS]
                                          [NTAPS_CHROMA]; ///< Chroma filter taps
#endif
  static const TFilterCoeff m_chromaFilterRPR1[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS]
                                              [NTAPS_CHROMA];   ///< Chroma filter taps 1.5x
  static const TFilterCoeff m_chromaFilterRPR2[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS]
                                              [NTAPS_CHROMA];   ///< Chroma filter taps 2x
  static const TFilterCoeff m_lumaFilterRPR1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1]
                                            [12];   ///< Luma filter taps 1.5x
  static const TFilterCoeff m_lumaFilterRPR2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1]
                                            [12];   ///< Luma filter taps 2x
  static const TFilterCoeff m_affineLumaFilterRPR1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1]
                                                  [12];   ///< Luma filter taps 1.5x
  static const TFilterCoeff m_affineLumaFilterRPR2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1]
                                                  [12];   ///< Luma filter taps 2x
private:
  static const TFilterCoeff m_lumaAltHpelIFilter[8];   ///< Luma filter taps
  static const TFilterCoeff m_bilinearFilterPrec4[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS]
                                                 [NTAPS_BILINEAR];   ///< bilinear filter taps
  static const TFilterCoeff m_bilinearFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS]
                                            [NTAPS_BILINEAR];   ///< biliniear filter taps for TM
  static const TFilterCoeff m_lumaIntraFilterExt[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << 1]
                                                [6];   ///< Chroma filter 6 taps
#if JVET_Z0117_CHROMA_IF
  static const TFilterCoeff g_aiExtIntraCubicFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << 1]
                                                   [4];   ///< Chroma filter taps
  static const TFilterCoeff g_aiExtIntraGaussFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << 1]
                                                   [4];   ///< Chroma filter taps
#else
  static const TFilterCoeff g_aiExtIntraCubicFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << 1]
                                                   [NTAPS_CHROMA]; ///< Chroma filter taps
  static const TFilterCoeff g_aiExtIntraGaussFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << 1]
                                                   [NTAPS_CHROMA]; ///< Chroma filter taps
#endif
public:
  static const TFilterCoeff m_bilinearFilterChroma[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS]
                                                  [NTAPS_BILINEAR];   ///< bilinear filter taps for chroma
  // INTRA-6TAP (Intra 6-tap Cubic + 6-tap Gaussian + left side 4-tap weak filtering for intra)
  static const TFilterCoeff m_lumaIntraFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS]
                                             [6];   ///< 6-tap cubic IF for angular prediction
  static const TFilterCoeff m_weak4TapFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS]
                                            [4];   ///< 4-tap IF for extended intra reference

public:
  template<bool isFirst, bool isLast> static void filterCopy(const ClpRng &clpRng, const Pel *src, ptrdiff_t srcStride,
                                                             Pel *dst, ptrdiff_t dstStride, int width, int height,
                                                             bool biMCForDMVR);

  template<int N, bool isVertical, bool isFirst, bool isLast, bool biMCForDMVR>
  static void filter(const ClpRng &clpRng, Pel const *src, ptrdiff_t srcStride, Pel *dst, ptrdiff_t dstStride,
                     int width, int height, TFilterCoeff const *coeff);
  template<int N, bool biMCForDMVR> void filterHor(const ClpRng &clpRng, Pel const *src, ptrdiff_t srcStride, Pel *dst,
                                                   ptrdiff_t dstStride, int width, int height, bool isLast,
                                                   TFilterCoeff const *coeff) const;

  template<int N, bool biMCForDMVR> void filterVer(const ClpRng &clpRng, Pel const *src, ptrdiff_t srcStride, Pel *dst,
                                                   ptrdiff_t dstStride, int width, int height, bool isFirst,
                                                   bool isLast, TFilterCoeff const *coeff) const;

  template<bool isLast, int w> static void filterXxY_N4(const ClpRng &clpRng, Pel const *src, ptrdiff_t srcStride,
                                                        Pel *dst, ptrdiff_t dstStride, int width, int height,
                                                        TFilterCoeff const *coeffH, TFilterCoeff const *coeffV);
  template<bool isLast, int w> static void filterXxY_N6(const ClpRng &clpRng, Pel const *src, ptrdiff_t srcStride,
                                                        Pel *dst, ptrdiff_t dstStride, int width, int height,
                                                        TFilterCoeff const *coeffH, TFilterCoeff const *coeffV);
  template<bool isLast, int w> static void filterXxY_N8(const ClpRng &clpRng, Pel const *src, ptrdiff_t srcStride,
                                                        Pel *dst, ptrdiff_t dstStride, int width, int height,
                                                        TFilterCoeff const *coeffH, TFilterCoeff const *coeffV);
  template<bool isLast, int w> static void filterXxY_N12(const ClpRng &clpRng, Pel const *src, ptrdiff_t srcStride,
                                                         Pel *dst, ptrdiff_t dstStride, int width, int height,
                                                         TFilterCoeff const *coeffH, TFilterCoeff const *coeffV);

  static void xWeightedGeoBlk(const CodingUnit &cu, const uint32_t width, const uint32_t height, const CompID compIdx,
                              const uint8_t splitDir, const uint8_t bldIdx, PelUnitBuf &predDst, PelUnitBuf &predSrc0,
                              PelUnitBuf &predSrc1);
  void        weightedGeoBlk(const CodingUnit &cu, const uint32_t width, const uint32_t height, const CompID compIdx,
                             const uint8_t splitDir, const uint8_t bldIdx, PelUnitBuf &predDst, PelUnitBuf &predSrc0,
                             PelUnitBuf &predSrc1);
  static void xWeightedGeoBlkRounded(const CodingUnit &cu, const uint32_t width, const uint32_t height,
                                     const CompID compIdx, const uint8_t splitDir, const uint8_t bldIdx,
                                     PelUnitBuf &predDst, PelUnitBuf &predSrc0, PelUnitBuf &predSrc1);
  void weightedGeoBlkRounded(const CodingUnit &cu, const uint32_t width, const uint32_t height, const CompID compIdx,
                             const uint8_t splitDir, const uint8_t bldIdx, PelUnitBuf &predDst, PelUnitBuf &predSrc0,
                             PelUnitBuf &predSrc1);
  static void xWeightedSgpm(const CodingUnit &cu, const uint32_t width, const uint32_t height, const CompID compIdx,
                            const uint8_t splitDir, PelBuf &predDst, PelBuf &predSrc0, PelBuf &predSrc1);
  static int  xSadTM(const CodingUnit &cu, const int width, const int height, const int templateWidth,
                     const int templateHeight, const CompID compIdx, PelBuf &predBuf, PelBuf &recBuf, PelBuf &adBuf);
  static int  xSgpmSadTM(const CodingUnit &cu, const int width, const int height, const int templateWidth,
                         const int templateHeight, const CompID compIdx, const uint8_t splitDir, PelBuf &adBuf);

protected:
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  static CacheModel *m_cacheModel;
#endif
  enum
  {
#if IF_12TAP
    _12_TAPS,
#endif
    _8_TAPS,
    _4_TAPS,
    _2_TAPS_DMVR,
    _6_TAPS,
    _2_TAPS,
    NUM_TAP_MODES
  };

  static constexpr int tapToIdx(const int N, const bool biMCForDMVR)
  {
    return biMCForDMVR
      ? (N == 2 ? _2_TAPS_DMVR : NUM_TAP_MODES)
#if IF_12TAP
      : (N == 12 ? _12_TAPS
                 : (N == 8 ? _8_TAPS : (N == 4 ? _4_TAPS : (N == 6 ? _6_TAPS : (N == 2 ? _2_TAPS : NUM_TAP_MODES)))));
#else
      : (N == 8 ? _8_TAPS : (N == 4 ? _4_TAPS : (N == 6 ? _6_TAPS : (N == 2 ? _2_TAPS : NUM_TAP_MODES))));
#endif
  }

public:
  enum class Filter
  {
    DEFAULT = 0,
    DMVR,
    IBC,
    AFFINE,
    RPR1,
    RPR2,
    AFFINE_RPR1,
    AFFINE_RPR2,
    HALFPEL_ALT,
    TM
  };

  InterpolationFilter();
  ~InterpolationFilter() {}
  void (*m_filterHor[NUM_TAP_MODES][2][2])(const ClpRng &clpRng, Pel const *src, ptrdiff_t srcStride, Pel *dst,
                                           ptrdiff_t dstStride, int width, int height, TFilterCoeff const *coeff);
  void (*m_filterVer[NUM_TAP_MODES][2][2])(const ClpRng &clpRng, Pel const *src, ptrdiff_t srcStride, Pel *dst,
                                           ptrdiff_t dstStride, int width, int height, TFilterCoeff const *coeff);
  void (*m_filterCopy[2][2])(const ClpRng &clpRng, Pel const *src, ptrdiff_t srcStride, Pel *dst, ptrdiff_t dstStride,
                             int width, int height, bool biMCForDMVR);
  void (*m_filter4x4[2][2])(const ClpRng &clpRng, Pel const *src, ptrdiff_t srcStride, Pel *dst, ptrdiff_t dstStride,
                            int width, int height, TFilterCoeff const *coeffH, TFilterCoeff const *coeffV);
  void (*m_weightedGeoBlk)(const CodingUnit &cu, const uint32_t width, const uint32_t height, const CompID compIdx,
                           const uint8_t splitDir, const uint8_t bldIdx, PelUnitBuf &predDst, PelUnitBuf &predSrc0,
                           PelUnitBuf &predSrc1);
  void (*m_weightedGeoBlkRounded)(const CodingUnit &cu, const uint32_t width, const uint32_t height,
                                  const CompID compIdx, const uint8_t splitDir, const uint8_t bldIdx,
                                  PelUnitBuf &predDst, PelUnitBuf &predSrc0, PelUnitBuf &predSrc1);
  void (*m_weightedSgpm)(const CodingUnit &cu, const uint32_t width, const uint32_t height, const CompID compIdx,
                         const uint8_t splitDir, PelBuf &predDst, PelBuf &predSrc0, PelBuf &predSrc1);
  int (*m_sadTM)(const CodingUnit &cu, const int width, const int height, const int templateWidth,
                 const int templateHeight, const CompID compIdx, PelBuf &predBuf, PelBuf &recBuf, PelBuf &adBuf);
  int (*m_sgpmSadTM)(const CodingUnit &cu, const int width, const int height, const int templateWidth,
                     const int templateHeight, const CompID compIdx, const uint8_t splitDir, PelBuf &adBuf);

  void initInterpolationFilter(bool enable);
#ifdef TARGET_SIMD_X86
  void                         initInterpolationFilterX86();
  template<X86_VEXT vext> void _initInterpolationFilterX86();
#endif
  void filterHor(const CompID compID, Pel const *src, ptrdiff_t srcStride, Pel *dst, ptrdiff_t dstStride, int width,
                 int height, int frac, bool isLast, const ClpRng &clpRng, Filter nFilterIdx) const;
  void filterVer(const CompID compID, Pel const *src, ptrdiff_t srcStride, Pel *dst, ptrdiff_t dstStride, int width,
                 int height, int frac, bool isFirst, bool isLast, const ClpRng &clpRng, Filter nFilterIdx) const;
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  void cacheAssign(CacheModel *cache) { m_cacheModel = cache; }
#endif

  void filter4x4(const CompID compID, Pel const *src, ptrdiff_t srcStride, Pel *dst, ptrdiff_t dstStride, int width,
                 int height, int fracX, int fracY, bool isLast, const ChromaFormat fmt, const ClpRng &clpRng);
#if JVET_Z0117_CHROMA_IF
  static TFilterCoeff const *const getChromaFilterTable(const int deltaFract) { return m_chromaFilter4[deltaFract]; };
#else
  static TFilterCoeff const *const getChromaFilterTable(const int deltaFract) { return m_chromaFilter[deltaFract]; };
#endif
  // INTRA_6tap
  static TFilterCoeff const *const getIntraLumaFilterTable(const int deltaFract)
  {
    return m_lumaIntraFilter[deltaFract];
  };
  static TFilterCoeff const *const getWeak4TapFilterTable(const int deltaFract)
  {
    return m_weak4TapFilter[deltaFract];
  };
  static TFilterCoeff const *const getIntraLumaFilterTableExt(const int deltaFract)
  {
    return m_lumaIntraFilterExt[deltaFract];
  };
  static TFilterCoeff const *const getExtIntraCubicFilter(const int deltaFract)
  {
    return g_aiExtIntraCubicFilter[deltaFract];
  };
  static TFilterCoeff const *const getExtIntraGaussFilter(const int deltaFract)
  {
    return g_aiExtIntraGaussFilter[deltaFract];
  };
};

//! \}

#endif
