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

/** \file     AdaptiveLoopFilter.h
    \brief    adaptive loop filter class (header)
*/

#ifndef __ADAPTIVELOOPFILTERVTM__
#define __ADAPTIVELOOPFILTERVTM__

#include "CommonDef.h"

#include "AdaptiveLoopFilter.h"
#include "AlfParametersVtm.h"
#include "Unit.h"
#include "UnitTools.h"

class AdaptiveLoopFilterVtm : virtual public AdaptiveLoopFilter, virtual public AlfParametersVtm
{
public:
  // Constants

  static constexpr int CLASSIFICATION_BLK_SIZE = 32;   // non-normative, local buffer size
  static constexpr int CTB_MODE_LUMA_APS0      = LumaCtbModeHandler::createApsFilterMode(0);

  struct AlfClassifier
  {
    AlfClassifier() {}
    AlfClassifier(uint8_t cIdx, uint8_t tIdx) : classIdx(cIdx), transposeIdx(tIdx) {}

    uint8_t classIdx;
    uint8_t transposeIdx;
  };

  enum Direction : unsigned
  {
    HOR,
    VER,
    DIAG0,
    DIAG1,
    NUM_DIRECTIONS,
    FIRST = HOR,
    LAST  = DIAG1
  };

  template<typename T, unsigned size> class FilterRowPtrs : public AdaptiveLoopFilter::FilterRowPtrsBase<T, size>
  {
  public:
    class MaxNumNeighborRows
    {
    public:
      constexpr MaxNumNeighborRows() : m_val(static_cast<unsigned>(FilterRowPtrs::m_offset)) {}
      constexpr MaxNumNeighborRows(const int maxNumNeighborRows);
      constexpr MaxNumNeighborRows(const unsigned maxNumNeighborRows);

      constexpr operator unsigned() const { return m_val; }

    private:
      template<bool valueOk> struct ValueOk
      {};

      constexpr MaxNumNeighborRows(const int maxNumNeighborRows, ValueOk<false>);
      constexpr MaxNumNeighborRows(const unsigned maxNumNeighborRows, ValueOk<true>);
      constexpr MaxNumNeighborRows(const unsigned maxNumNeighborRows, ValueOk<false>);

      unsigned m_val;
    };

    constexpr FilterRowPtrs(T *const center, const ptrdiff_t stride,
                            const MaxNumNeighborRows maxNumNeighborRows = MaxNumNeighborRows());

    constexpr FilterRowPtrs<T, size> &set(T *const center, const ptrdiff_t stride,
                                          const MaxNumNeighborRows maxNumNeighborRows = MaxNumNeighborRows());
  };

  /// Buffer for laplacians.
  using LaplacianBuf =
    AdaptiveLoopFilter::LaplacianBuf<int, CLASSIFICATION_BLK_SIZE + 5, CLASSIFICATION_BLK_SIZE + 5, Direction>;

  /// Buffer for classification output.
  using ClassBuf = AdaptiveLoopFilter::ClassBuf<AlfClassifier>;

  // Setup

  AdaptiveLoopFilterVtm();

  void         create(const int picWidth, const int picHeight, const ChromaFormat format, const int maxCUWidth,
                      const int maxCUHeight, const int maxCUDepth, const BitDepths &inputBitDepth);
  virtual void destroy() override;

  // Processing

  void ALFProcess(CodingStructure &cs);

  // CCALF data access

  CcAlfFilterParam &getCcAlfFilterParam() { return m_ccAlfFilterParam; }

protected:
  // Frame-data-related values

  int m_alfVBLumaPos;
  int m_alfVBChmaPos;
  int m_alfVBLumaCTUHeight;
  int m_alfVBChmaCTUHeight;

  // Fixed filters

  static const int m_classToFilterMapping[ALF_NUM_FIXED_FILTER_SETS][MAX_NUM_ALF_CLASSES];
  static const int m_fixedFilterSetCoeff[ALF_FIXED_FILTER_NUM][MAX_NUM_ALF_LUMA_COEFF];

  // Non-frame-specific ALF parameters

  EnumArray<std::vector<AlfFilterShape>, ChannelType> m_filterShapes;

  // Filter coefficients buffers

  template<typename T> using LumaDataSingleAps = T[MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES];
  template<typename T> using LumaDataMultiAps  = LumaDataSingleAps<T>[ALF_CTB_MAX_NUM_APS];
  template<typename T> using ChromaData        = T[ALF_MAX_NUM_ALTERNATIVES_CHROMA][MAX_NUM_ALF_CHROMA_COEFF];

  using LumaCoeffSingleAps = LumaDataSingleAps<coeff_t>;
  using LumaClipSingleAps  = LumaDataSingleAps<clip_t>;
  using LumaCoeffMultiAps  = LumaDataMultiAps<coeff_t>;
  using LumaClipMultiAps   = LumaDataMultiAps<clip_t>;
  using ChromaCoeff        = ChromaData<coeff_t>;
  using ChromaClip         = ChromaData<clip_t>;

  LumaCoeffMultiAps  m_coeffApsLuma;
  LumaClipMultiAps   m_clippApsLuma;
  LumaCoeffSingleAps m_coeffFinal;
  LumaClipSingleAps  m_clippFinal;
  ChromaCoeff        m_chromaCoeffFinal;
  ChromaClip         m_chromaClippFinal;

  // Results buffers

  ClassBuf m_classifier;   ///< Frame-level classification buffer.

  // CCALF-related parameters

  std::vector<AlfFilterShape> m_filterShapesCcAlf;
  CcAlfFilterParam            m_ccAlfFilterParam;

  inline short *getCoeffVals(const int mode);
  inline Pel   *getClipVals(const int mode);

  void deriveClassification(const CPelBuf &srcLuma, const Area &blkDst, const Area &blk);

  void reconstructCoeffAPSs(CodingStructure &cs, bool luma, bool chroma, bool isRdo);
  void reconstructLumaCoeff(AlfParam &alfParam, const bool isRdo);
  void reconstructChromaCoeff(AlfParam &alfParam, const bool isRdo);

  void applyCcAlfFilter(CodingStructure &cs, CompID compID, const PelBuf &dstBuf, const PelUnitBuf &recYuvExt,
                        uint8_t    *filterControl,
                        const short filterSet[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF],
                        const int   selectedFilterIdx);

  using alfFilterFunc = void(const ClassBuf &classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc,
                             const Area &blkDst, const Area &blk, const CompID compId, const short *filterSet,
                             const Pel *fClipSet, const ClpRng &clpRng, const int vbCTUHeight, int vbPos);

  // Fixed filters

  short m_fixedFilterSetCoeffDec[ALF_NUM_FIXED_FILTER_SETS][MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
  Pel   m_clipDefault[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];

  LaplacianBuf m_laplacian;   ///< Buffer holding the laplacian gradients for classification.

  // Function pointers for classification

  void (*m_deriveClassificationBlk)(ClassBuf &classifier, LaplacianBuf &laplacian, const CPelBuf &srcLuma,
                                    const Area &blkDst, const Area &blk, const int shift, const int vbCTUHeight,
                                    int vbPos);

  // Function pointers for adaptive filtering

  alfFilterFunc *m_filter5x5Blk;
  alfFilterFunc *m_filter7x7Blk;

  // Function pointers for CCALF

  void (*m_filterCcAlf)(const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blkSrc,
                        const CompID compId, const int16_t *filterCoeff, const ClpRngs &clpRngs, CodingStructure &cs,
                        int vbCTUHeight, int vbPos);

  // Classification

  static void deriveClassificationBlk(ClassBuf &classifier, LaplacianBuf &laplacian, const CPelBuf &srcLuma,
                                      const Area &blkDst, const Area &blk, const int shift, const int vbCTUHeight,
                                      int vbPos);

  // Adaptive filtering

  template<AlfFilterType filtType> static alfFilterFunc filterBlk;

  // CCALF filtering

  static void filterBlkCcAlf(const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blkSrc,
                             const CompID compId, const int16_t *filterCoeff, const ClpRngs &clpRngs,
                             CodingStructure &cs, int vbCTUHeight, int vbPos);

private:
  // Initialization

  void initAdaptiveLoopFilter();      ///< Init function pointers for non-SIMD case.
#ifdef TARGET_SIMD_X86
  void                         initAdaptiveLoopFilterX86();   ///< Init function pointers for SIMD case.
  template<X86_VEXT vext> void _initAdaptiveLoopFilterX86();
#endif
};

Pel *AdaptiveLoopFilterVtm::getClipVals(const int mode)
{
  if (LumaCtbModeHandler::isFixedFilter(mode))
  {
    return m_clipDefault;
  }
  else
  {
    return m_clippApsLuma[LumaCtbModeHandler::getApsIdx(mode)];
  }
}

short *AdaptiveLoopFilterVtm::getCoeffVals(const int mode)
{
  if (LumaCtbModeHandler::isFixedFilter(mode))
  {
    return m_fixedFilterSetCoeffDec[LumaCtbModeHandler::getFixedFilterIdx(mode)];
  }
  else
  {
    return m_coeffApsLuma[LumaCtbModeHandler::getApsIdx(mode)];
  }
}

template<typename T, unsigned size>
constexpr AdaptiveLoopFilterVtm::FilterRowPtrs<T, size>::MaxNumNeighborRows::MaxNumNeighborRows(
  const int maxNumNeighborRows)
  : MaxNumNeighborRows(maxNumNeighborRows < 0 ? MaxNumNeighborRows(maxNumNeighborRows, ValueOk<false>())
                                              : MaxNumNeighborRows(static_cast<unsigned>(maxNumNeighborRows)))
{}

template<typename T, unsigned size>
constexpr AdaptiveLoopFilterVtm::FilterRowPtrs<T, size>::MaxNumNeighborRows::MaxNumNeighborRows(
  const unsigned maxNumNeighborRows)
  : MaxNumNeighborRows(maxNumNeighborRows > FilterRowPtrs::m_offset
                         ? MaxNumNeighborRows(maxNumNeighborRows, ValueOk<false>())
                         : MaxNumNeighborRows(maxNumNeighborRows, ValueOk<true>()))
{}

template<typename T, unsigned size>
constexpr AdaptiveLoopFilterVtm::FilterRowPtrs<T, size>::MaxNumNeighborRows::MaxNumNeighborRows(
  const int maxNumNeighborRows, ValueOk<false>)
  : m_val(0)
{
  THROW("Number of neighbor rows must not be negative.");
}

template<typename T, unsigned size>
constexpr AdaptiveLoopFilterVtm::FilterRowPtrs<T, size>::MaxNumNeighborRows::MaxNumNeighborRows(
  const unsigned maxNumNeighborRows, ValueOk<true>)
  : m_val(maxNumNeighborRows)
{}

template<typename T, unsigned size>
constexpr AdaptiveLoopFilterVtm::FilterRowPtrs<T, size>::MaxNumNeighborRows::MaxNumNeighborRows(
  const unsigned maxNumNeighborRows, ValueOk<false>)
  : m_val(0)
{
  THROW("Given number of neighbor rows is too large.");
}

template<typename T, unsigned size> constexpr AdaptiveLoopFilterVtm::FilterRowPtrs<T, size>::FilterRowPtrs(
  T *const center, const ptrdiff_t stride,
  const AdaptiveLoopFilterVtm::FilterRowPtrs<T, size>::MaxNumNeighborRows maxNumNeighborRows /*= MaxNumNeighborRows()*/)
{
  set(center, stride, maxNumNeighborRows);
}

template<typename T, unsigned size>
constexpr AdaptiveLoopFilterVtm::FilterRowPtrs<T, size> &AdaptiveLoopFilterVtm::FilterRowPtrs<T, size>::set(
  T *const center, const ptrdiff_t stride,
  const AdaptiveLoopFilterVtm::FilterRowPtrs<T, size>::MaxNumNeighborRows maxNumNeighborRows /*= MaxNumNeighborRows()*/)
{
  this->m_rows[this->m_offset] = center;
  for (int diff = 1; diff <= maxNumNeighborRows; ++diff)
  {
    this->m_rows[this->m_offset - diff] = this->m_rows[this->m_offset - diff + 1] - stride;
    this->m_rows[this->m_offset + diff] = this->m_rows[this->m_offset + diff - 1] + stride;
  }
  for (int diff = maxNumNeighborRows + 1; diff <= this->m_offset; ++diff)
  {
    this->m_rows[this->m_offset - diff] = this->m_rows[this->m_offset - diff + 1];
    this->m_rows[this->m_offset + diff] = this->m_rows[this->m_offset + diff - 1];
  }

  return *this;
}
#endif
