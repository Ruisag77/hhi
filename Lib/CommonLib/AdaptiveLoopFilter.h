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

#ifndef __ADAPTIVELOOPFILTER__
#define __ADAPTIVELOOPFILTER__

#include "CommonDef.h"

#include "AlfParameters.h"
#include "Unit.h"
#include "UnitTools.h"

class AdaptiveLoopFilter : virtual public AlfParameters
{
public:
  // Sub-types

  template<typename T, unsigned size> class FilterRowPtrsBase
  {
    static_assert((size & 1) == 1, "Size must be odd.");

  public:
    constexpr T *const &operator[](const int index) const;
    constexpr T       *&operator[](const int index);

    constexpr FilterRowPtrsBase<T, size> &operator++();

  protected:
    FilterRowPtrsBase() = default;

    static constexpr int  m_offset = static_cast<int>(size / 2);
    std::array<T *, size> m_rows;
  };

  /// Basic type for array that can be indexed by enums.
  template<typename ElemT, typename EnumT, EnumT min, EnumT max> class EnumBasedArray
    : public std::array<ElemT,
                        static_cast<typename std::underlying_type<EnumT>::type>(max) -
                          static_cast<typename std::underlying_type<EnumT>::type>(min) + 1>
  {
  public:
    using EnumDType = typename std::underlying_type<EnumT>::type;
    using BaseType  = typename std::array<ElemT, static_cast<EnumDType>(max) - static_cast<EnumDType>(min) + 1>;

    using const_reference = typename BaseType::const_reference;
    using reference       = typename BaseType::reference;

    static_assert(std::is_unsigned<EnumDType>::value, "Not implemented for signed underlying data types of the enum.");

    static_assert(static_cast<EnumDType>(max) >= static_cast<EnumDType>(min),
                  "Maximum value must not be less than minimum value.");

    constexpr const_reference at(EnumT pos) const;
    constexpr reference       at(EnumT pos);

    constexpr const_reference operator[](EnumT pos) const;
    constexpr reference       operator[](EnumT pos);
  };

  /// Basic type for arrays of buffers that can be indexed by enums.
  template<typename ElemT, typename EnumT, EnumT min, EnumT max> class EnumBasedBufArray
    : public AdaptiveLoopFilter::EnumBasedArray<ElemT, EnumT, min, max>
  {
  public:
    inline void create(const Size &size, const unsigned maxCUSize = 0, const unsigned margin = 0,
                       const unsigned alignment = 0);
    inline void destroy();
  };

  /// ALF buffer class allowing easier row access.
  template<typename T> class LineAccessAreaStorage : public AreaStorage<T>
  {
  public:
    LineAccessAreaStorage() : AreaStorage<T>(), m_margin(0), m_rows() {}
    LineAccessAreaStorage(const LineAccessAreaStorage<T> &other) = delete;
    inline LineAccessAreaStorage(LineAccessAreaStorage<T> &&other) noexcept;
    virtual ~LineAccessAreaStorage() { destroy(); }

    inline void create(const Size &size, const unsigned maxCUSize = 0, const unsigned margin = 0,
                       const unsigned alignment = 0);
    inline void destroy();

    LineAccessAreaStorage<T>        &operator=(const LineAccessAreaStorage<T> &other) = delete;
    inline LineAccessAreaStorage<T> &operator=(LineAccessAreaStorage<T> &&other) noexcept;

    inline const T *operator[](const int rowIdx) const;
    inline T       *operator[](const int rowIdx);
    inline const T *operator[](const size_t rowIdx) const;
    inline T       *operator[](const size_t rowIdx);

  private:
    int              m_margin;   ///< Margin. Signed integer type was chosen for convenience.
    std::vector<T *> m_rows;     ///< Vector of row start pointers. The i-th row is at index i + m_margin.
  };

  /// Buffer for laplacians.
  template<typename ElemT, size_t W, size_t H, typename DirT> class LaplacianBuf
    : public EnumBasedArray<ElemT[H][W], DirT, DirT::FIRST, DirT::LAST>
  {
  public:
    using DirBuf     = ElemT[H][W];
    using ParentType = EnumBasedArray<DirBuf, DirT, DirT::FIRST, DirT::LAST>;
    using BaseType   = typename ParentType::BaseType;

    using BaseType::operator[];
  };

  template<typename ClsT> using ClassBuf = LineAccessAreaStorage<ClsT>;

  // Constants

  static const EnumArray<int, ChannelType> ALF_NUM_CLIP_VALS;

  static constexpr int CTB_MODE_CHROMA_ALT0 = ChromaCtbModeHandler::createMode(0);
  static constexpr int CTB_MODE_OFF         = CtbModeHandler::createDisabled();

  // Setup

  AdaptiveLoopFilter();
  virtual ~AdaptiveLoopFilter() { destroy(); }

  void         create(const int picWidth, const int picHeight, const ChromaFormat format, const int maxCUWidth,
                      const int maxCUHeight, const int maxCUDepth, const BitDepths &inputBitDepth);
  virtual void destroy();

  // CCALF data access

  uint8_t *getCcAlfControlIdc(const CompID compID) { return m_ccAlfFilterControl[compID - 1]; }

protected:
  // Internal status

  bool m_created = false;

  // Frame-data-related values

  BitDepths    m_inputBitDepth;
  int          m_picWidth;
  int          m_picHeight;
  int          m_picWidthChroma;
  int          m_picHeightChroma;
  int          m_maxCUWidth;
  int          m_maxCUHeight;
  int          m_maxCUDepth;
  int          m_numCTUsInWidth;
  int          m_numCTUsInPic;
  ChromaFormat m_chromaFormat;
  ClpRngs      m_clpRngs;

  // Non-frame-specific ALF parameters

  EnumArray<std::array<Pel, MAX_ALF_NUM_CLIP_VALS>, ChannelType> m_alfClippingValues;

  // Frame-specific ALF parameters

  CtuModes *m_modes;

  // Filter coefficients buffers

  using coeff_t = short;
  using clip_t  = Pel;

  // Input buffers

  PelStorage m_tempBuf;    ///< Frame-level buffer for the reconstruction after SAO (all components).
  PelStorage m_tempBuf2;   ///< CTU-level buffer for the reconstruction after SAO (all components).

  // CCALF-related parameters

  uint8_t *m_ccAlfFilterControl[2];

  bool isCrossedBySubPicBoundaries(const CodingStructure &cs, const int xPos, const int yPos, const int width,
                                   const int height, bool &clipTop, bool &clipBottom, bool &clipLeft, bool &clipRight,
                                   int &rasterSliceAlfPad);

  void reconstructCoeffSingleFilter(const short *const coeffSrc, const Pel *const clippSrc, short *const coeffDst,
                                    Pel *const clippDst, const std::array<Pel, MAX_ALF_NUM_CLIP_VALS> &clipVals,
                                    const size_t numCoeffMinus1, const bool nonLinearFlag, const bool isRdo,
                                    const short factor);

  static inline Pel clipALF(const Pel clip, const Pel ref, const Pel val);
  static inline Pel clipALF(const Pel clip, const Pel ref, const Pel val0, const Pel val1);
};

Pel AdaptiveLoopFilter::clipALF(const Pel clip, const Pel ref, const Pel val)
{
  return Clip3<Pel>(-clip, clip, val - ref);
}

Pel AdaptiveLoopFilter::clipALF(const Pel clip, const Pel ref, const Pel val0, const Pel val1)
{
  return clipALF(clip, ref, val0) + clipALF(clip, ref, val1);
}

template<typename T, unsigned size>
constexpr T *const &AdaptiveLoopFilter::FilterRowPtrsBase<T, size>::operator[](const int index) const
{
  CHECKD(index < -m_offset || index > m_offset, "Index is out of range.");
  return m_rows[m_offset + index];
}

template<typename T, unsigned size>
constexpr T *&AdaptiveLoopFilter::FilterRowPtrsBase<T, size>::operator[](const int index)
{
  return const_cast<T *&>(const_cast<const FilterRowPtrsBase<T, size> *>(this)->operator[](index));
}

template<typename T, unsigned size>
constexpr AdaptiveLoopFilter::FilterRowPtrsBase<T, size> &AdaptiveLoopFilter::FilterRowPtrsBase<T, size>::operator++()
{
  for (size_t i = 0; i < size; ++i)
  {
    ++m_rows[i];
  }
  return *this;
}

template<typename T> AdaptiveLoopFilter::LineAccessAreaStorage<T>::LineAccessAreaStorage(
  AdaptiveLoopFilter::LineAccessAreaStorage<T> &&other) noexcept
  : AreaStorage<T>(std::move(other))
  , m_margin(std::move(other.m_margin))
  , m_rows(std::move(other.m_rows))
{}

template<typename T> AdaptiveLoopFilter::LineAccessAreaStorage<T> &
  AdaptiveLoopFilter::LineAccessAreaStorage<T>::operator=(AdaptiveLoopFilter::LineAccessAreaStorage<T> &&other) noexcept
{
  AreaStorage<T>::operator=(std::move(other));
  m_rows = std::move(other.m_rows);

  return *this;
}

template<typename T> void AdaptiveLoopFilter::LineAccessAreaStorage<T>::create(const Size    &size,
                                                                               const unsigned maxCUSize /*= 0*/,
                                                                               const unsigned margin /*= 0*/,
                                                                               const unsigned alignment /* = 0*/)
{
  AreaStorage<T>::create(size, maxCUSize, margin, alignment);

  CHECK(!m_rows.empty(), "Trying to re-create an already initialized buffer.");
  m_margin = static_cast<int>(margin);
  m_rows.resize(this->height + 2 * margin);
  for (int rowIdx = -m_margin; rowIdx < static_cast<int>(this->height + margin); ++rowIdx)
  {
    m_rows[rowIdx + m_margin] = this->bufAt(0, rowIdx);
  }
}

template<typename T> void AdaptiveLoopFilter::LineAccessAreaStorage<T>::destroy()
{
  AreaStorage<T>::destroy();
  m_margin = 0;
  m_rows.clear();
}

template<typename T> const T *AdaptiveLoopFilter::LineAccessAreaStorage<T>::operator[](const int rowIdx) const
{
  CHECKD(m_rows.empty(), "Attempting to access an uninitialized buffer.");
  CHECKD(rowIdx < -m_margin || rowIdx >= static_cast<int>(this->height) + m_margin, "Row index is out of range.");
  return m_rows[rowIdx + m_margin];
}

template<typename T> T *AdaptiveLoopFilter::LineAccessAreaStorage<T>::operator[](const int rowIdx)
{
  return const_cast<T *>(const_cast<const AdaptiveLoopFilter::LineAccessAreaStorage<T> *>(this)->operator[](rowIdx));
}

template<typename T> const T *AdaptiveLoopFilter::LineAccessAreaStorage<T>::operator[](const size_t rowIdx) const
{
  CHECKD(rowIdx > static_cast<size_t>(std::numeric_limits<int>::max()), "rowIdx is too big.");
  return operator[](static_cast<int>(rowIdx));
}

template<typename T> T *AdaptiveLoopFilter::LineAccessAreaStorage<T>::operator[](const size_t rowIdx)
{
  return const_cast<T *>(const_cast<const AdaptiveLoopFilter::LineAccessAreaStorage<T> *>(this)->operator[](rowIdx));
}

template<typename ElemT, typename EnumT, EnumT min, EnumT max> constexpr
  typename AdaptiveLoopFilter::EnumBasedArray<ElemT, EnumT, min, max>::const_reference
  AdaptiveLoopFilter::EnumBasedArray<ElemT, EnumT, min, max>::at(EnumT pos) const
{
  constexpr auto minVal = static_cast<EnumDType>(min);
  const auto     posVal = static_cast<EnumDType>(pos);

  CHECK((posVal < minVal) || (pos > static_cast<EnumDType>(max)), "Index is out of range.");

  return BaseType::at(posVal - minVal);
}

template<typename ElemT, typename EnumT, EnumT min, EnumT max> constexpr
  typename AdaptiveLoopFilter::EnumBasedArray<ElemT, EnumT, min, max>::reference
  AdaptiveLoopFilter::EnumBasedArray<ElemT, EnumT, min, max>::at(EnumT pos)
{
  return const_cast<reference>(const_cast<const EnumBasedArray<ElemT, EnumT, min, max> *>(this)->at(pos));
}

template<typename ElemT, typename EnumT, EnumT min, EnumT max> constexpr
  typename AdaptiveLoopFilter::EnumBasedArray<ElemT, EnumT, min, max>::const_reference
  AdaptiveLoopFilter::EnumBasedArray<ElemT, EnumT, min, max>::operator[](EnumT pos) const
{
  return BaseType::operator[](static_cast<EnumDType>(pos) - static_cast<EnumDType>(min));
}

template<typename ElemT, typename EnumT, EnumT min, EnumT max> constexpr
  typename AdaptiveLoopFilter::EnumBasedArray<ElemT, EnumT, min, max>::reference
  AdaptiveLoopFilter::EnumBasedArray<ElemT, EnumT, min, max>::operator[](EnumT pos)
{
  return const_cast<reference>(const_cast<const EnumBasedArray<ElemT, EnumT, min, max> *>(this)->operator[](pos));
}

template<typename ElemT, typename EnumT, EnumT min, EnumT max>
void AdaptiveLoopFilter::EnumBasedBufArray<ElemT, EnumT, min, max>::create(const Size    &size,
                                                                           const unsigned maxCUSize /*= 0*/,
                                                                           const unsigned margin /*= 0*/,
                                                                           const unsigned alignment /*= 0*/)
{
  for (auto &buf: *this)
  {
    buf.create(size, maxCUSize, margin, alignment);
  }
}

template<typename ElemT, typename EnumT, EnumT min, EnumT max>
void AdaptiveLoopFilter::EnumBasedBufArray<ElemT, EnumT, min, max>::destroy()
{
  for (auto &buf: *this)
  {
    buf.destroy();
  }
}
#endif
