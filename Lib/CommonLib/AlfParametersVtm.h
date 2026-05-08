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

/** \file     AlfParameters.h
    \brief    Define types for storing ALF parameters
*/

#ifndef __ALFPARAMETERSVTM__
#define __ALFPARAMETERSVTM__

#include <vector>
#include "CommonDef.h"
#include "AlfParameters.h"

//! \ingroup AlfParameters
//! \{

class AlfParametersVtm : public AlfParameters
{
public:
  static constexpr int COEFF_SCALE_BITS_ALF     = 7; // 8-bit signed values
  static constexpr int MAX_NUM_ALF_LUMA_COEFF   = 13;
  static constexpr int MAX_NUM_ALF_CHROMA_COEFF = 7;

  static constexpr int MAX_ALF_FILTER_LENGTH       = 7;
  static constexpr int MAX_ALF_PADDING_SIZE        = 4;
  static constexpr int MAX_NUM_CC_ALF_FILTERS      = 4;
  static constexpr int MAX_NUM_CC_ALF_CHROMA_COEFF = 8;

  static constexpr int ALF_FIXED_FILTER_NUM      = 64;
  static constexpr int ALF_NUM_FIXED_FILTER_SETS = 16;

  enum AlfFilterType
  {
    ALF_FILTER_5,
    ALF_FILTER_7,
    CC_ALF,
    ALF_NUM_OF_FILTER_TYPES
  };

  struct AlfFilterShape : public AlfParameters::AlfFilterShapeBase
  {
    inline AlfFilterShape(const AlfFilterType filterType);

    AlfFilterType filterType;
  };

  struct AlfParam : public AlfParameters::AlfParamBase
  {
  private:
    // Basic types for luma/chroma filter coefficients/clipping indices.
    template<typename T> using chromaDataAlt_t = std::array<T, MAX_NUM_ALF_CHROMA_COEFF>;
    template<typename T> using lumaDataAlt_t   = std::array<T, MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF>;

    // Data types for a single alternative.
    using chromaClipAlt_t  = chromaDataAlt_t<clip_t>;
    using chromaCoeffAlt_t = chromaDataAlt_t<coeff_t>;
    using lumaClipAlt_t    = lumaDataAlt_t<clip_t>;
    using lumaCoeffAlt_t   = lumaDataAlt_t<coeff_t>;

  public:
    // Data types for multiple alternatives (in case there are multiple).
    using chromaClip_t       = chromaMultiAlt_t<chromaClipAlt_t>;
    using chromaCoeff_t      = chromaMultiAlt_t<chromaCoeffAlt_t>;
    using chromaNonLinFlag_t = nonLinFlagAlt_t;
    using lumaClip_t         = lumaClipAlt_t;
    using lumaCoeff_t        = lumaCoeffAlt_t;
    using lumaCoeffDelta_t   = lumaCoeffDeltaAlt_t;
    using lumaNumFilt_t      = lumaNumFiltAlt_t;
    using lumaNonLinFlag_t   = nonLinFlagAlt_t;

    lumaNonLinFlag_t lumaNonLinearFlag;     // alf_luma_clip_flag
    lumaCoeff_t      lumaCoeff;             // alf_coeff_luma_delta[i][j]
    lumaClip_t       lumaClipp;             // alf_clipp_luma_[i][j]
    lumaCoeffDelta_t filterCoeffDeltaIdx;   // filter_coeff_delta[i]
    lumaNumFilt_t    numLumaFilters;        // number_of_filters_minus1 + 1

    chromaNonLinFlag_t chromaNonLinearFlag;   // alf_chroma_clip_flag
    chromaCoeff_t      chromaCoeff;           // alf_coeff_chroma[i]
    chromaClip_t       chromaClipp;           // alf_clipp_chroma[i]

    EnumArray<std::vector<AlfFilterShape>, ChannelType> *filterShapes;

    AlfParam() { reset(); }

    inline void            reset();
    inline const AlfParam &operator=(const AlfParam &src);

    friend inline bool operator==(const AlfParam &lhs, const AlfParam &rhs);
    friend bool        operator!=(const AlfParam &lhs, const AlfParam &rhs) { return !(lhs == rhs); }
  };

  struct CcAlfFilterParam : AlfParameters::CcAlfFilterParamBase
  {
    bool  ccAlfFilterIdxEnabled[2][MAX_NUM_CC_ALF_FILTERS];
    short ccAlfCoeff[2][MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF];

    CcAlfFilterParam() { reset(); }

    inline void                    reset();
    inline const CcAlfFilterParam &operator=(const CcAlfFilterParam &src);

    friend inline bool operator==(const CcAlfFilterParam &lhs, const CcAlfFilterParam &rhs);
    friend bool        operator!=(const CcAlfFilterParam &lhs, const CcAlfFilterParam &rhs);
  };

  using LumaCtbModeHandler = LumaCtbModeHandlerBase<ALF_NUM_FIXED_FILTER_SETS, ALF_CTB_MAX_NUM_APS>;
};

AlfParametersVtm::AlfFilterShape::AlfFilterShape(const AlfParametersVtm::AlfFilterType filterType)
  : filterType(filterType)
{
  switch (filterType)
  {
  case ALF_FILTER_5:
    initDiamondShape(5);

    // clang-format off
    pattern = {
                0,
            1,  2,  3,
        4,  5,  6,  5,  4,
            3,  2,  1,
                0
    };
    // clang-format on

    break;

  case ALF_FILTER_7:
    initDiamondShape(7);

    // clang-format off
    pattern = {
                    0,
                1,  2,  3,
            4,  5,  6,  7,  8,
        9, 10, 11, 12, 11, 10, 9,
            8,  7,  6,  5,  4,
                3,  2,  1,
                    0
    };
    // clang-format on

    break;

  case CC_ALF:
    filterLength = 8;
    numCoeff     = 8;

    break;

  default:
    THROW("Wrong ALF filter shape");
  }
}

void AlfParametersVtm::AlfParam::reset()
{
  AlfParameters::AlfParamBase::reset();

  fillBuf(lumaNonLinearFlag, false);
  fillBuf(lumaCoeff, static_cast<short>(0));
  fillBuf(lumaClipp, static_cast<Pel>(0));
  fillBuf(filterCoeffDeltaIdx, static_cast<short>(0));
  fillBuf(numLumaFilters, 1);

  fillBuf(chromaNonLinearFlag, false);
  fillBuf(chromaCoeff, static_cast<short>(0));
  fillBuf(chromaClipp, static_cast<short>(0));
}

const AlfParametersVtm::AlfParam &AlfParametersVtm::AlfParam::operator=(const AlfParametersVtm::AlfParam &src)
{
  AlfParameters::AlfParamBase::operator=(src);

  lumaNonLinearFlag = src.lumaNonLinearFlag;
  lumaCoeff         = src.lumaCoeff;
  lumaClipp         = src.lumaClipp;
  numLumaFilters    = src.numLumaFilters;
  std::copy(std::begin(src.filterCoeffDeltaIdx), std::end(src.filterCoeffDeltaIdx), std::begin(filterCoeffDeltaIdx));

  chromaNonLinearFlag = src.chromaNonLinearFlag;
  chromaCoeff         = src.chromaCoeff;
  chromaClipp         = src.chromaClipp;

  filterShapes = src.filterShapes;

  return *this;
}

bool operator==(const AlfParametersVtm::AlfParam &lhs, const AlfParametersVtm::AlfParam &rhs)
{
  if (!(static_cast<const AlfParameters::AlfParamBase &>(lhs) == static_cast<const AlfParameters::AlfParamBase &>(rhs)))
  {
    return false;
  }

  if (lhs.lumaNonLinearFlag != rhs.lumaNonLinearFlag)
  {
    return false;
  }
  if (lhs.lumaCoeff != rhs.lumaCoeff)
  {
    return false;
  }
  if (lhs.lumaClipp != rhs.lumaClipp)
  {
    return false;
  }
  if (lhs.numLumaFilters != rhs.numLumaFilters)
  {
    return false;
  }
  if (!std::equal(std::begin(lhs.filterCoeffDeltaIdx), std::end(lhs.filterCoeffDeltaIdx),
                  std::begin(rhs.filterCoeffDeltaIdx)))
  {
    return false;
  }

  if (lhs.chromaNonLinearFlag != rhs.chromaNonLinearFlag)
  {
    return false;
  }
  if (lhs.chromaCoeff != rhs.chromaCoeff)
  {
    return false;
  }
  if (lhs.chromaClipp != rhs.chromaClipp)
  {
    return false;
  }

  return true;
}

void AlfParametersVtm::CcAlfFilterParam::reset()
{
  AlfParameters::CcAlfFilterParamBase::reset();

  fillBuf(ccAlfFilterIdxEnabled, false);
  fillBuf(ccAlfCoeff, static_cast<short>(0));
  fillBuf(ccAlfFilterCount, static_cast<uint8_t>(MAX_NUM_CC_ALF_FILTERS));
}

const AlfParametersVtm::CcAlfFilterParam &
  AlfParametersVtm::CcAlfFilterParam::operator=(const AlfParametersVtm::CcAlfFilterParam &src)
{
  AlfParameters::CcAlfFilterParamBase::operator=(src);

  std::memcpy(ccAlfFilterIdxEnabled, src.ccAlfFilterIdxEnabled, sizeof(ccAlfFilterIdxEnabled));
  std::memcpy(ccAlfCoeff, src.ccAlfCoeff, sizeof(ccAlfCoeff));

  return *this;
}

bool operator==(const AlfParametersVtm::CcAlfFilterParam &lhs, const AlfParametersVtm::CcAlfFilterParam &rhs)
{
  if (!(static_cast<const AlfParameters::CcAlfFilterParamBase &>(lhs) ==
        static_cast<const AlfParameters::CcAlfFilterParamBase &>(rhs)))
  {
    return false;
  }
  if (std::memcmp(lhs.ccAlfFilterIdxEnabled, rhs.ccAlfFilterIdxEnabled, sizeof(lhs.ccAlfFilterIdxEnabled)))
  {
    return false;
  }
  if (std::memcmp(lhs.ccAlfCoeff, rhs.ccAlfCoeff, sizeof(lhs.ccAlfCoeff)))
  {
    return false;
  }
  return true;
}

//! \}

#endif  // end of #ifndef  __ALFPARAMETERSVTM__
