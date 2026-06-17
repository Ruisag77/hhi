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

/** \file     AlfParametersEcm.h
    \brief    Define types for storing ECM ALF parameters
*/

#ifndef __ALFPARAMETERSECM__
#define __ALFPARAMETERSECM__

#include <vector>
#include "CommonDef.h"
#include "AlfParameters.h"

//! \ingroup AlfParameters
//! \{

class AlfParametersEcm : virtual public AlfParameters
{
public:
  static constexpr int ALF_MAX_NUM_ALTERNATIVES_LUMA = 4;

  static constexpr int ALF_PADDING_SIZE_FIXED_RESULTS = 6;
  static constexpr int NUM_RESI_PAD                   = 8;
  static constexpr int ALF_PADDING_SIZE_GAUSS_RESULTS = 2;
  static constexpr int NUM_DB_PAD                     = 8;

  static constexpr int ALF_RESI_SHIFT_OFFSET                          = 4;
  static constexpr int ALF_PADDING_SIZE_PRED                          = 3;
  static constexpr int ALF_NUM_CLASSIFIER                             = 3;
  static constexpr int ALF_CLASSES_RESI                               = 25;
  static constexpr int ALF_CLASSES_NEW                                = 25;
  static constexpr int ALF_NUM_CLASSES_CLASSIFIER[ALF_NUM_CLASSIFIER] = { MAX_NUM_ALF_CLASSES, ALF_CLASSES_NEW,
                                                                          ALF_CLASSES_RESI };

  static constexpr int NUM_FIXED_FILTER_SETS      = 8;     ///< Overall number of fixed filter sets (for all QPs).
  static constexpr int NUM_FIXED_FILTERS_PER_SET  = 512;   ///< Number of fixed filters per fixed filter set.
  static constexpr int NUM_FIXED_FILTER_SET_CANDS = 2;   ///< Number of fixed filter set candidates for a given QP.
  static constexpr int NUM_FIXED_FILTERS          = 2;   ///< Number of fixed filters.

  static constexpr int ALF_CLASSIFIER_FL        = 5;
  static constexpr int ALF_CLASSIFIER_FL_CHROMA = 1;
  static constexpr int NUM_DIR_FIX              = 7;   ///< Number of directionality classes (fixed filter)
  static constexpr int NUM_ACT_FIX              = 16;   ///< Number of activitiy classes for the (fixed filter)
  static constexpr int DIST_CLASS               = 4;
  static constexpr int NUM_DIST_FIX             = 8;
  static constexpr int NUM_CLASSES_FIX          = ((NUM_DIR_FIX * (NUM_DIR_FIX + 1)) * NUM_ACT_FIX * NUM_DIST_FIX);
  static constexpr int MAX_FILTER_LENGTH_FIXED  = 13;

  static constexpr int ALF_NUM_COEFF_3x3_DIAMOND   = 3;   ///< Number of coeffs for a 3x3 diamond-shaped filter.
  static constexpr int ALF_NUM_COEFF_5x5_CROSS     = 5;   ///< Number of coeffs for a 5x5 cross-shaped filter.
  static constexpr int ALF_NUM_COEFF_9x9_CROSS     = 11;   ///< Number of coeffs for a 9x9 cross-shaped filter.
  static constexpr int ALF_NUM_COEFF_9x9_DIAMOND   = 21;   ///< Number of coeffs for a 9x9 diamond-shaped filter.
  static constexpr int ALF_NUM_COEFF_13x13_CROSS   = 19;   ///< Number of coeffs for a 13x13 cross-shaped filter.
  static constexpr int ALF_NUM_COEFF_13x13_DIAMOND = 43;   ///< Number of coeffs for a 13x13 diamond-shaped filter.

  /// Number of coeffs for the fixed filter combination using only 9x9 filters.
  static constexpr int FIX_FILTER_NUM_COEFF_9_DB_9 = ALF_NUM_COEFF_9x9_DIAMOND - 1   // reconstruction
    + ALF_NUM_COEFF_9x9_DIAMOND;   // pre-DBF reconstruction
  /// Number of coeffs for the fixed filter combination of 13x13 and 9x9.
  static constexpr int FIX_FILTER_NUM_COEFF_13_DB_9 = ALF_NUM_COEFF_13x13_DIAMOND   // first fixed filter
    + ALF_NUM_COEFF_9x9_DIAMOND;   // pre-DBF reconstruction
  /// Number of coeffs for the combined 9x9 fixed filter.
  static constexpr int FIX_FILTER_NUM_COEFF_DB_COMBINE_9_DB_9 = ALF_NUM_COEFF_9x9_DIAMOND;

  /// Maximum number of coefficients for a luma ALF filter.
  static constexpr int MAX_NUM_ALF_LUMA_COEFF = ALF_NUM_COEFF_9x9_CROSS   // reconstruction
    + 1   // first fixed filter
    + ALF_NUM_COEFF_13x13_CROSS   // second fixed filter
    + ALF_NUM_COEFF_3x3_DIAMOND   // pre-DBF reconstruction
    + ALF_NUM_COEFF_5x5_CROSS   // gaussian fixed filter
    + 1;
  /// Maximum number of coeffients for a chroma ALF filter.
  static constexpr int MAX_NUM_ALF_CHROMA_COEFF = ALF_NUM_COEFF_9x9_DIAMOND   // reconstruction
    + ALF_NUM_COEFF_5x5_CROSS;   // fixed filter

  static constexpr int MAX_ALF_FILTER_LENGTH       = 13;
  static constexpr int MAX_ALF_PADDING_SIZE        = 8;
  static constexpr int MAX_NUM_CC_ALF_FILTERS      = 16;
  static constexpr int CC_ALF_NUM_COEFF_LUMA       = 25;
  static constexpr int MAX_NUM_CC_ALF_CHROMA_COEFF = CC_ALF_NUM_COEFF_LUMA   // luma samples
    + 4;   // chroma samples

  // Coefficient bit scale parameters
  static constexpr int COEFF_SCALE_BITS_LUMA = 8;   // maximum value is 2^8, i.e. coefficients are in range [-256, 256]
  static constexpr int COEFF_SCALE_BITS_CHROMA = 7;
  static constexpr int COEFF_MANTISSA_BITS_LUMA =
    2;   // 3-bit signed values for mantissa component, i.e. in range [-3, 3]
  static constexpr int COEFF_MANTISSA_BITS_CHROMA = 1;

  // Scaling factor parameters
  static constexpr int ALF_SCALE_BITS_NUM                        = 4;
  static constexpr int ALF_SCALE_FACTOR[1 << ALF_SCALE_BITS_NUM] = {
    16, 17, 18, 19, 21, 23, 25, 27, 29, 15, 14, 13, 12, 11, 10, 9,
  };
  static constexpr int ALF_SCALE_SHIFT = 4;

  enum AlfFilterType
  {
    ALF_FILTER_9,
    ALF_FILTER_9_EXT_DB_RESI_DIRECT,
    ALF_FILTER_9_EXT_DB_RESI,
    CC_ALF,
    ALF_NUM_OF_FILTER_TYPES
  };

  struct AlfFilterShape : public AlfParameters::AlfFilterShapeBase
  {
    inline AlfFilterShape(const AlfFilterType filterType);

    AlfFilterType filterType;
    int           numOrder;
    int           indexSecOrder;
    int           offset0;
  };

  struct AlfParam : public AlfParameters::AlfParamBase
  {
  private:
    // Basic types for per-class parameters
    template<typename T> using chromaClassParamAlt_t = std::array<T, 1>;
    template<typename T> using lumaClassParamAlt_t   = std::array<T, MAX_NUM_ALF_CLASSES>;

    // Basic types for luma/chroma filter coefficients/clipping indices.
    template<typename T> using chromaDataAlt_t = std::array<T, MAX_NUM_ALF_CHROMA_COEFF>;
    template<typename T> using lumaDataAlt_t   = std::array<T, MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF>;

    // Data types for a single alternative.
    using chromaScaleIdxAlt_t    = chromaClassParamAlt_t<int8_t>;
    using chromaClipAlt_t        = chromaDataAlt_t<clip_t>;
    using chromaCoeffAlt_t       = chromaDataAlt_t<coeff_t>;
    using lumaClassifierIdxAlt_t = int8_t;
    using lumaScaleIdxAlt_t      = lumaClassParamAlt_t<int8_t>;
    using lumaClipAlt_t          = lumaDataAlt_t<clip_t>;
    using lumaCoeffAlt_t         = lumaDataAlt_t<coeff_t>;

    // Basic data types for multiple alternatives.
    template<typename T> using lumaMultiAlt_t = std::array<T, ALF_MAX_NUM_ALTERNATIVES_LUMA>;

  public:
    // Data types for multiple alternatives (in case there are multiple).
    using chromaScaleIdx_t    = chromaMultiAlt_t<chromaScaleIdxAlt_t>;
    using chromaClip_t        = chromaMultiAlt_t<chromaClipAlt_t>;
    using chromaCoeff_t       = chromaMultiAlt_t<chromaCoeffAlt_t>;
    using chromaNonLinFlag_t  = chromaMultiAlt_t<nonLinFlagAlt_t>;
    using lumaClassifierIdx_t = lumaMultiAlt_t<lumaClassifierIdxAlt_t>;
    using lumaScaleIdx_t      = lumaMultiAlt_t<lumaScaleIdxAlt_t>;
    using lumaClip_t          = lumaMultiAlt_t<lumaClipAlt_t>;
    using lumaCoeff_t         = lumaMultiAlt_t<lumaCoeffAlt_t>;
    using lumaCoeffDelta_t    = lumaMultiAlt_t<lumaCoeffDeltaAlt_t>;
    using lumaNumFilt_t       = lumaMultiAlt_t<lumaNumFiltAlt_t>;
    using lumaNonLinFlag_t    = lumaMultiAlt_t<nonLinFlagAlt_t>;

    EnumArray<AlfFilterType, ChannelType> filterType;

    lumaClassifierIdx_t lumaClassifierIdx;
    int                 numAlternativesLuma;
    lumaNonLinFlag_t    lumaNonLinearFlag;   // alf_luma_clip_flag
    lumaScaleIdx_t      lumaScaleIdx;   // alf_luma_scale_factor[i][j]
    lumaCoeff_t         lumaCoeff;   // alf_coeff_luma_delta[i][j]
    lumaClip_t          lumaClipp;   // alf_clipp_luma_[i][j]
    lumaCoeffDelta_t    filterCoeffDeltaIdx;   // filter_coeff_delta[i]
    lumaNumFilt_t       numLumaFilters;   // number_of_filters_minus1 + 1

    chromaNonLinFlag_t chromaNonLinearFlag;   // alf_chroma_clip_flag
    chromaScaleIdx_t   chromaScaleIdx;   // alf_chroma_scale_factor[i]
    chromaCoeff_t      chromaCoeff;   // alf_coeff_chroma[i]
    chromaClip_t       chromaClipp;   // alf_clipp_chroma[i]

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
    friend bool        operator!=(const CcAlfFilterParam &lhs, const CcAlfFilterParam &rhs) { return !(lhs == rhs); }
  };

  class LumaCtbModeHandler
    : public LumaCtbModeHandlerBase<NUM_FIXED_FILTERS, ALF_CTB_MAX_NUM_APS * ALF_MAX_NUM_ALTERNATIVES_LUMA>
  {
  public:
    LumaCtbModeHandler() = delete;

    static constexpr int createApsFilterMode(const unsigned apsIdx, const unsigned altIdx);

    static constexpr unsigned getAlternative(const int ctbMode) { return Base::getApsIdx(ctbMode) >> NUM_BITS_APS_IDX; }
    static constexpr unsigned getApsIdx(const int ctbMode);

    static constexpr void setAlternative(int &ctbMode, const unsigned altIdx);
    static constexpr void setApsFilter(int &ctbMode, const unsigned apsIdx, const unsigned altIdx);

  protected:
    static constexpr uint8_t NUM_BITS_APS_IDX = getNumBits(ALF_CTB_MAX_NUM_APS - 1);
    static constexpr uint8_t NUM_BITS_ALT_IDX = getNumBits(ALF_MAX_NUM_ALTERNATIVES_LUMA - 1);
    static constexpr uint8_t MASK_APS_IDX     = (1 << NUM_BITS_APS_IDX) - 1;

    static_assert(NUM_FIXED_FILTERS + ((ALF_MAX_NUM_ALTERNATIVES_LUMA - 1) << NUM_BITS_APS_IDX) +
                      (ALF_CTB_MAX_NUM_APS - 1) <=
                    static_cast<unsigned>(std::numeric_limits<int>::max()),
                  "Filter number is too large for int type.");

    static constexpr void setApsFilterNoCheck(int &ctbMode, unsigned apsIdx, unsigned altIdx);

  private:
    using Base = LumaCtbModeHandlerBase<NUM_FIXED_FILTERS, ALF_CTB_MAX_NUM_APS * ALF_MAX_NUM_ALTERNATIVES_LUMA>;
  };
};

AlfParametersEcm::AlfFilterShape::AlfFilterShape(const AlfFilterType filterType)
  : filterType(filterType)
  , numOrder(0)
  , indexSecOrder(0)
  , offset0(0)
{
  switch (filterType)
  {
  case ALF_FILTER_9:
    initDiamondShape(9);

    // clang-format off
    pattern = {
                       0,
                   1,  2,  3,
               4,  5,  6,  7,  8,
           9, 10, 11, 12, 13, 14, 15,
      16, 17, 18, 19, 20, 19, 18, 17, 16,
          15, 14, 13, 12, 11, 10,  9,
               8,  7,  6,  5,  4,
                   3,  2,  1,
                       0
    };
    // clang-format on

    numCoeff = ALF_NUM_COEFF_9x9_DIAMOND   // reconstruction
      + ALF_NUM_COEFF_5x5_CROSS;   // fixed filter
    numOrder      = 2;
    indexSecOrder = ALF_NUM_COEFF_9x9_DIAMOND - 1   // reconstruction
      + ALF_NUM_COEFF_5x5_CROSS - 1;   // fixed filter
    offset0 = 0;

    break;

  case ALF_FILTER_9_EXT_DB_RESI_DIRECT:
    numCoeff = ALF_NUM_COEFF_9x9_CROSS   // reconstruction
      + 1   // first fixed filter
      + ALF_NUM_COEFF_13x13_CROSS   // second fixed filter
      + ALF_NUM_COEFF_5x5_CROSS   // gaussian fixed filter
      + ALF_NUM_COEFF_3x3_DIAMOND   // pre-DBF reconstruction
      + 1;   // residual

    // XXX: filterLength and pattern are never used for this filter shape.
    filterLength = -1;
    pattern.clear();

    numOrder      = 2;
    indexSecOrder = ALF_NUM_COEFF_9x9_CROSS - 1   // reconstruction
      + 1 - 1   // first fixed filter
      + ALF_NUM_COEFF_13x13_CROSS - 1   // second fixed filter
      + ALF_NUM_COEFF_5x5_CROSS - 1   // gaussian fixed filter
      + ALF_NUM_COEFF_3x3_DIAMOND - 1   // pre-DBF reconstruction
      + 1 - 1;   // residual
    offset0 = 0;

    break;

  case ALF_FILTER_9_EXT_DB_RESI:
    numCoeff = ALF_NUM_COEFF_9x9_CROSS   // reconstruction
      + 1   // first fixed filter
      + ALF_NUM_COEFF_13x13_CROSS   // second fixed filter
      + 1   // gaussian fixed filter
      + ALF_NUM_COEFF_3x3_DIAMOND   // pre-DBF reconstruction
      + 1   // residual
      + 1;   // residual-based fixed filter

    // XXX: filterLength and pattern are never used for this filter shape.
    filterLength = -1;
    pattern.clear();

    numOrder      = 2;
    indexSecOrder = ALF_NUM_COEFF_9x9_CROSS - 1   // reconstruction
      + 1 - 1   // first fixed filter
      + ALF_NUM_COEFF_13x13_CROSS - 1   // second fixed filter
      + 1 - 1   // gaussian fixed filter
      + ALF_NUM_COEFF_3x3_DIAMOND - 1   // pre-DBF reconstruction
      + 1 - 1   // residual
      + 1 - 1;   // residual-based fixed filter
    offset0 = 0;

    break;

  case CC_ALF:
    filterLength = MAX_NUM_CC_ALF_CHROMA_COEFF;
    numCoeff     = MAX_NUM_CC_ALF_CHROMA_COEFF;

    break;

  default:
    THROW("Wrong ALF filter shape");
  }
};

void AlfParametersEcm::AlfParam::reset()
{
  AlfParameters::AlfParamBase::reset();

  fillBuf(filterType, AlfFilterType::ALF_NUM_OF_FILTER_TYPES);

  numAlternativesLuma = 1;
  fillBuf(lumaClassifierIdx, static_cast<int8_t>(0));
  fillBuf(lumaNonLinearFlag, false);
  fillBuf(lumaScaleIdx, static_cast<int8_t>(0));
  fillBuf(lumaCoeff, static_cast<short>(0));
  fillBuf(lumaClipp, static_cast<Pel>(0));
  fillBuf(filterCoeffDeltaIdx, static_cast<short>(0));
  fillBuf(numLumaFilters, 1);

  fillBuf(chromaNonLinearFlag, false);
  fillBuf(chromaScaleIdx, static_cast<int8_t>(0));
  fillBuf(chromaCoeff, static_cast<short>(0));
  fillBuf(chromaClipp, static_cast<short>(0));
}

const AlfParametersEcm::AlfParam &AlfParametersEcm::AlfParam::operator=(const AlfParametersEcm::AlfParam &src)
{
  AlfParameters::AlfParamBase::operator=(src);

  filterType = src.filterType;

  lumaClassifierIdx   = src.lumaClassifierIdx;
  numAlternativesLuma = src.numAlternativesLuma;
  lumaNonLinearFlag   = src.lumaNonLinearFlag;
  lumaScaleIdx        = src.lumaScaleIdx;
  lumaCoeff           = src.lumaCoeff;
  lumaClipp           = src.lumaClipp;
  filterCoeffDeltaIdx = src.filterCoeffDeltaIdx;
  numLumaFilters      = src.numLumaFilters;

  chromaNonLinearFlag = src.chromaNonLinearFlag;
  chromaScaleIdx      = src.chromaScaleIdx;
  chromaCoeff         = src.chromaCoeff;
  chromaClipp         = src.chromaClipp;

  filterShapes = src.filterShapes;

  return *this;
}

bool operator==(const AlfParametersEcm::AlfParam &lhs, const AlfParametersEcm::AlfParam &rhs)
{
  if (!(static_cast<const AlfParameters::AlfParamBase &>(lhs) == static_cast<const AlfParameters::AlfParamBase &>(rhs)))
  {
    return false;
  }

  if (lhs.filterType != rhs.filterType)
  {
    return false;
  }

  if (lhs.lumaClassifierIdx != rhs.lumaClassifierIdx)
  {
    return false;
  }
  if (lhs.numAlternativesLuma != rhs.numAlternativesLuma)
  {
    return false;
  }
  if (lhs.lumaNonLinearFlag != rhs.lumaNonLinearFlag)
  {
    return false;
  }
  if (lhs.lumaScaleIdx != rhs.lumaScaleIdx)
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

  for (int i = 0; i < AlfParametersEcm::ALF_MAX_NUM_ALTERNATIVES_LUMA; i++)
  {
    if (!std::equal(std::begin(lhs.filterCoeffDeltaIdx[i]), std::end(lhs.filterCoeffDeltaIdx[i]),
                    std::begin(rhs.filterCoeffDeltaIdx[i])))
    {
      return false;
    }
  }
  if (lhs.numLumaFilters != rhs.numLumaFilters)
  {
    return false;
  }

  if (lhs.chromaNonLinearFlag != rhs.chromaNonLinearFlag)
  {
    return false;
  }
  if (lhs.chromaScaleIdx != rhs.chromaScaleIdx)
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

void AlfParametersEcm::CcAlfFilterParam::reset()
{
  AlfParameters::CcAlfFilterParamBase::reset();

  fillBuf(ccAlfFilterIdxEnabled, false);
  fillBuf(ccAlfCoeff, static_cast<short>(0));
  fillBuf(ccAlfFilterCount, static_cast<uint8_t>(MAX_NUM_CC_ALF_FILTERS));
}

const AlfParametersEcm::CcAlfFilterParam &
  AlfParametersEcm::CcAlfFilterParam::operator=(const AlfParametersEcm::CcAlfFilterParam &src)
{
  AlfParameters::CcAlfFilterParamBase::operator=(src);

  std::memcpy(ccAlfFilterIdxEnabled, src.ccAlfFilterIdxEnabled, sizeof(ccAlfFilterIdxEnabled));
  std::memcpy(ccAlfCoeff, src.ccAlfCoeff, sizeof(ccAlfCoeff));

  return *this;
}

bool operator==(const AlfParametersEcm::CcAlfFilterParam &lhs, const AlfParametersEcm::CcAlfFilterParam &rhs)
{
  if (!(static_cast<const AlfParameters::CcAlfFilterParamBase &>(lhs) ==
        static_cast<const AlfParameters::CcAlfFilterParamBase &>(rhs)))
  {
    return false;
  }
  if (std::memcmp(lhs.ccAlfFilterEnabled, rhs.ccAlfFilterEnabled, sizeof(lhs.ccAlfFilterEnabled)))
  {
    return false;
  }
  if (std::memcmp(lhs.ccAlfCoeff, rhs.ccAlfCoeff, sizeof(lhs.ccAlfCoeff)))
  {
    return false;
  }
  return true;
}

constexpr int AlfParametersEcm::LumaCtbModeHandler::createApsFilterMode(const unsigned apsIdx, const unsigned altIdx)
{
  int mode = 0;
  setApsFilter(mode, apsIdx, altIdx);
  return mode;
}

constexpr unsigned AlfParametersEcm::LumaCtbModeHandler::getApsIdx(const int ctbMode)
{
  return Base::getApsIdx(ctbMode) & MASK_APS_IDX;
}

constexpr void AlfParametersEcm::LumaCtbModeHandler::setAlternative(int &ctbMode, const unsigned altIdx)
{
  CHECK(altIdx >= ALF_MAX_NUM_ALTERNATIVES_LUMA, "Alternative index out of range.");
  setApsFilterNoCheck(ctbMode, getApsIdx(ctbMode), altIdx);
}

constexpr void AlfParametersEcm::LumaCtbModeHandler::setApsFilter(int &ctbMode, const unsigned apsIdx,
                                                                  const unsigned altIdx)
{
  CHECK(apsIdx >= ALF_CTB_MAX_NUM_APS, "APS index out of range.");
  CHECK(altIdx >= ALF_MAX_NUM_ALTERNATIVES_LUMA, "Alternative index out of range.");
  setApsFilterNoCheck(ctbMode, apsIdx, altIdx);
}

constexpr void AlfParametersEcm::LumaCtbModeHandler::setApsFilterNoCheck(int &ctbMode, unsigned apsIdx, unsigned altIdx)
{
  Base::setApsFilterNoCheck(ctbMode, apsIdx | (altIdx << NUM_BITS_APS_IDX));
}

//! \}

#endif   // __ALFPARAMETERSECM__
