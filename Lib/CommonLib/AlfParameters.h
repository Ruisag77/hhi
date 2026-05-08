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

#ifndef __ALFPARAMETERS__
#define __ALFPARAMETERS__

#include <vector>
#include "CommonDef.h"

//! \ingroup AlfParameters
//! \{

class AlfParameters
{
public:
  static constexpr int ALF_MAX_NUM_ALTERNATIVES_CHROMA = 8;

  static constexpr int MAX_NUM_ALF_CLASSES = 25;

  static constexpr int COEFF_SCALE_BITS_CCALF     = 7; // 8-bit signed values
  static constexpr int CCALF_DYNAMIC_RANGE        = 6;
  static constexpr int CCALF_BITS_PER_COEFF_LEVEL = 3;

  static constexpr int ALF_CTB_MAX_NUM_APS = MAX_NUM_APS(ApsType::ALF);

  static constexpr int MAX_ALF_NUM_CLIP_VALS = 4;

  struct AlfFilterShapeBase
  {
    int              filterLength;
    int              numCoeff;
    std::vector<int> pattern;

  protected:
    AlfFilterShapeBase() = default;

    constexpr void initDiamondShape(const int size);
  };

  using AlfApsList = static_vector<int, ALF_CTB_MAX_NUM_APS>;

  struct AlfParamBase
  {
    virtual ~AlfParamBase() = default;

    bool                         enabledFlag[MAX_NUM_COMP];               // alf_slice_enable_flag, alf_chroma_idc
    int                          numAlternativesChroma;                   // alf_chroma_num_alts_minus_one + 1
    EnumArray<bool, ChannelType> newFilterFlag;

  protected:
    // Data types for a singe filter coefficient/clipping index.
    using clip_t  = Pel;
    using coeff_t = short;

    // Data types for a single alternative.
    using lumaCoeffDeltaAlt_t = short[MAX_NUM_ALF_CLASSES];
    using lumaNumFiltAlt_t    = int;
    using nonLinFlagAlt_t     = bool;

    // Basic data types for multiple alternatives.
    template<typename T> using chromaMultiAlt_t = std::array<T, ALF_MAX_NUM_ALTERNATIVES_CHROMA>;

    AlfParamBase() = default;

    inline void                reset();
    inline const AlfParamBase &operator=(const AlfParamBase &src);

    friend inline bool operator==(const AlfParamBase &lhs, const AlfParamBase &rhs);
    friend bool        operator!=(const AlfParamBase &lhs, const AlfParamBase &rhs) { return !(lhs == rhs); }
  };

  struct CcAlfFilterParamBase
  {
    virtual ~CcAlfFilterParamBase() = default;

    bool    ccAlfFilterEnabled[2];
    uint8_t ccAlfFilterCount[2];
    int     newCcAlfFilter[2];
    int     numberValidComponents;

  protected:
    CcAlfFilterParamBase() = default;

    inline void                        reset();
    inline const CcAlfFilterParamBase &operator=(const CcAlfFilterParamBase &src);

    friend inline bool operator==(const CcAlfFilterParamBase &lhs, const CcAlfFilterParamBase &rhs);
    friend bool operator!=(const CcAlfFilterParamBase &lhs, const CcAlfFilterParamBase &rhs) { return !(lhs == rhs); }
  };

  class CtbModeHandler
  {
  public:
    CtbModeHandler() = delete;

    static constexpr int createDisabled() { return -1; }

    static constexpr bool isEnabled(const int ctbMode) { return ctbMode >= 0; }
    static constexpr void setDisabled(int &ctbMode) { ctbMode = -1; }

  protected:
    static constexpr uint8_t getNumBits(unsigned i);
  };

  class ChromaCtbModeHandler : public CtbModeHandler
  {
  public:
    ChromaCtbModeHandler() = delete;

    static constexpr int createMode(unsigned altIdx);

    static constexpr unsigned getAlternative(const int ctbMode);

    static constexpr void setAlternative(int &ctbMode, const unsigned altIdx);
  };

  using CtbModes = std::vector<int>;

  class CtuModes : public std::array<CtbModes, MAX_NUM_COMP>
  {
  public:
    inline void resize(const size_t count);
  };

protected:
  template<unsigned NUM_FIXED_FILTERS, unsigned NUM_APS_FILTERS> class LumaCtbModeHandlerBase : public CtbModeHandler
  {
  public:
    LumaCtbModeHandlerBase() = delete;

    static constexpr int createApsFilterMode(const unsigned apsFilterIdx);
    static constexpr int createFixedFilterMode(const unsigned fixedFilterIdx);

    static constexpr unsigned getApsIdx(const int ctbMode);
    static constexpr unsigned getFixedFilterIdx(const int ctbMode);

    static constexpr bool isApsFilter(const int ctbMode);
    static constexpr bool isFixedFilter(const int ctbMode);

    static constexpr void setApsFilter(int &ctbMode, const unsigned apsIdx);
    static constexpr void setFixedFilter(int &ctbMode, const unsigned fixedFilterIdx);

  protected:
    static_assert(NUM_FIXED_FILTERS + (NUM_APS_FILTERS - 1) <= static_cast<unsigned>(std::numeric_limits<int>::max()),
                  "Filter number is too large for int type.");

    static constexpr void setApsFilterNoCheck(int &ctbMode, const unsigned apsIdx);
  };

protected:
  template<typename T> static constexpr void fillBuf(T &buf, const T &val) { buf = val; }

  template<typename BufT, typename ValT,
           decltype(std::begin(std::declval<BufT &>())) * = nullptr,   // Ensure that std::begin(BufT&) is valid.
           decltype(std::end(std::declval<BufT &>())) *   = nullptr>   // Ensure that std::end(BufT&) is valid.
  static constexpr void fillBuf(BufT &buf, const ValT &val);
};

template<typename BufT, typename ValT,
         decltype(std::begin(std::declval<BufT &>())) *,   // SFINAE
         decltype(std::end(std::declval<BufT &>())) *>     // SFINAE
constexpr void AlfParameters::fillBuf(BufT &buf, const ValT &val)
{
  std::for_each(std::begin(buf), std::end(buf), [&](auto &subBuf) { fillBuf(subBuf, val); });
}

constexpr void AlfParameters::AlfFilterShapeBase::initDiamondShape(const int size)
{
  filterLength = size;
  numCoeff     = size * size / 4 + 1;
}

void AlfParameters::AlfParamBase::reset()
{
  fillBuf(enabledFlag, false);
  numAlternativesChroma = 1;
  fillBuf(newFilterFlag, false);
}

const AlfParameters::AlfParamBase &AlfParameters::AlfParamBase::operator=(const AlfParameters::AlfParamBase &src)
{
  std::copy(std::begin(src.enabledFlag), std::end(src.enabledFlag), std::begin(enabledFlag));
  numAlternativesChroma = src.numAlternativesChroma;
  newFilterFlag         = src.newFilterFlag;

  return *this;
}

bool operator==(const AlfParameters::AlfParamBase &lhs, const AlfParameters::AlfParamBase &rhs)
{
  if (memcmp(lhs.enabledFlag, rhs.enabledFlag, sizeof(lhs.enabledFlag)))
  {
    return false;
  }
  if (lhs.numAlternativesChroma != rhs.numAlternativesChroma)
  {
    return false;
  }
  if (lhs.newFilterFlag != rhs.newFilterFlag)
  {
    return false;
  }

  return true;
}

void AlfParameters::CcAlfFilterParamBase::reset()
{
  fillBuf(ccAlfFilterEnabled, false);
  fillBuf(newCcAlfFilter, 0);
  numberValidComponents = 3;
}

const AlfParameters::CcAlfFilterParamBase &
  AlfParameters::CcAlfFilterParamBase::operator=(const AlfParameters::CcAlfFilterParamBase &src)
{
  std::copy(std::begin(src.ccAlfFilterEnabled), std::end(src.ccAlfFilterEnabled), std::begin(ccAlfFilterEnabled));
  std::copy(std::begin(src.ccAlfFilterCount), std::end(src.ccAlfFilterCount), std::begin(ccAlfFilterCount));
  std::copy(std::begin(src.newCcAlfFilter), std::end(src.newCcAlfFilter), std::begin(newCcAlfFilter));
  numberValidComponents = src.numberValidComponents;

  return *this;
}

bool operator==(const AlfParameters::CcAlfFilterParamBase &lhs, const AlfParameters::CcAlfFilterParamBase &rhs)
{
  if (!std::equal(std::begin(lhs.ccAlfFilterEnabled), std::end(lhs.ccAlfFilterEnabled),
                  std::begin(rhs.ccAlfFilterEnabled)))
  {
    return false;
  }
  if (!std::equal(std::begin(lhs.ccAlfFilterCount), std::end(lhs.ccAlfFilterCount), std::begin(rhs.ccAlfFilterCount)))
  {
    return false;
  }
  if (!std::equal(std::begin(lhs.newCcAlfFilter), std::end(lhs.newCcAlfFilter), std::begin(rhs.newCcAlfFilter)))
  {
    return false;
  }
  if (lhs.numberValidComponents != rhs.numberValidComponents)
  {
    return false;
  }

  return true;
}

constexpr uint8_t AlfParameters::CtbModeHandler::getNumBits(unsigned i)
{
  uint8_t count = 0;
  while (i > 0)
  {
    i >>= 1;
    ++count;
  }
  return count;
}

template<unsigned NUM_FIXED_FILTERS, unsigned NUM_APS_FILTERS>
constexpr int AlfParameters::LumaCtbModeHandlerBase<NUM_FIXED_FILTERS, NUM_APS_FILTERS>::createApsFilterMode(
  const unsigned apsFilterIdx)
{
  int mode = 0;
  setApsFilter(mode, apsFilterIdx);
  return mode;
}

template<unsigned NUM_FIXED_FILTERS, unsigned NUM_APS_FILTERS>
constexpr int AlfParameters::LumaCtbModeHandlerBase<NUM_FIXED_FILTERS, NUM_APS_FILTERS>::createFixedFilterMode(
  const unsigned fixedFilterIdx)
{
  int mode = 0;
  setFixedFilter(mode, fixedFilterIdx);
  return mode;
}

template<unsigned NUM_FIXED_FILTERS, unsigned NUM_APS_FILTERS> constexpr unsigned
  AlfParameters::LumaCtbModeHandlerBase<NUM_FIXED_FILTERS, NUM_APS_FILTERS>::getApsIdx(const int ctbMode)
{
  CHECK(!isApsFilter(ctbMode), "APS filter index must only be accessed if APS filter is selected.");
  return static_cast<unsigned>(ctbMode) - NUM_FIXED_FILTERS;
}

template<unsigned NUM_FIXED_FILTERS, unsigned NUM_APS_FILTERS> constexpr unsigned
  AlfParameters::LumaCtbModeHandlerBase<NUM_FIXED_FILTERS, NUM_APS_FILTERS>::getFixedFilterIdx(const int ctbMode)
{
  CHECK(!isFixedFilter(ctbMode), "Fixed filter index must only be accessed if fixed filter is selected.");
  return static_cast<unsigned>(ctbMode);
}

template<unsigned NUM_FIXED_FILTERS, unsigned NUM_APS_FILTERS>
constexpr bool AlfParameters::LumaCtbModeHandlerBase<NUM_FIXED_FILTERS, NUM_APS_FILTERS>::isApsFilter(const int ctbMode)
{
  return isEnabled(ctbMode) && (static_cast<unsigned>(ctbMode) >= NUM_FIXED_FILTERS);
}

template<unsigned NUM_FIXED_FILTERS, unsigned NUM_APS_FILTERS> constexpr bool
  AlfParameters::LumaCtbModeHandlerBase<NUM_FIXED_FILTERS, NUM_APS_FILTERS>::isFixedFilter(const int ctbMode)
{
  return isEnabled(ctbMode) && (static_cast<unsigned>(ctbMode) < NUM_FIXED_FILTERS);
}

template<unsigned NUM_FIXED_FILTERS, unsigned NUM_APS_FILTERS> constexpr void
  AlfParameters::LumaCtbModeHandlerBase<NUM_FIXED_FILTERS, NUM_APS_FILTERS>::setApsFilter(int           &ctbMode,
                                                                                          const unsigned apsIdx)
{
  CHECK(apsIdx >= NUM_APS_FILTERS, "APS filter index out of range.");
  setApsFilterNoCheck(ctbMode, apsIdx);
}

template<unsigned NUM_FIXED_FILTERS, unsigned NUM_APS_FILTERS>
constexpr void AlfParameters::LumaCtbModeHandlerBase<NUM_FIXED_FILTERS, NUM_APS_FILTERS>::setFixedFilter(
  int &ctbMode, const unsigned fixedFilterIdx)
{
  CHECK(fixedFilterIdx >= NUM_FIXED_FILTERS, "Fixed filter index out of range.");
  ctbMode = static_cast<int>(fixedFilterIdx);
}

template<unsigned NUM_FIXED_FILTERS, unsigned NUM_APS_FILTERS> constexpr void
  AlfParameters::LumaCtbModeHandlerBase<NUM_FIXED_FILTERS, NUM_APS_FILTERS>::setApsFilterNoCheck(int           &ctbMode,
                                                                                                 const unsigned apsIdx)
{
  ctbMode = static_cast<int>(apsIdx + NUM_FIXED_FILTERS);
}

constexpr int AlfParameters::ChromaCtbModeHandler::createMode(unsigned altIdx)
{
  int mode = 0;
  setAlternative(mode, altIdx);
  return mode;
}

constexpr unsigned AlfParameters::ChromaCtbModeHandler::getAlternative(const int ctbMode)
{
  CHECK(!isEnabled(ctbMode), "Cannot get the alternative index when ALF is disabled.");
  return static_cast<unsigned>(ctbMode);
}

constexpr void AlfParameters::ChromaCtbModeHandler::setAlternative(int &ctbMode, const unsigned altIdx)
{
  CHECK(altIdx >= ALF_MAX_NUM_ALTERNATIVES_CHROMA, "Alternative index out of range.");
  ctbMode = static_cast<int>(altIdx);
}

void AlfParameters::CtuModes::resize(const size_t count)
{
  std::for_each(begin(), end(), [=](AlfParameters::CtbModes &ctbModes) { ctbModes.resize(count); });
}

struct CcSaoComParam
{
  bool     enabled[MAX_NUM_COMP];
  bool     extChroma[MAX_NUM_COMP];
  bool     reusePrv[MAX_NUM_COMP];
  int      reusePrvId[MAX_NUM_COMP];
  uint8_t  setNum[MAX_NUM_COMP];
  bool     setEnabled[MAX_NUM_COMP][MAX_CCSAO_SET_NUM];
  bool     setType[MAX_NUM_COMP][MAX_CCSAO_SET_NUM];
  uint16_t candPos[MAX_NUM_COMP][MAX_CCSAO_SET_NUM]
                  [MAX_NUM_COMP];   // BO 0: candPosY, EO 0: edgeDir, 1: edgeCmp, 2: dummy
  uint16_t bandNum[MAX_NUM_COMP][MAX_CCSAO_SET_NUM][MAX_NUM_COMP];
  short    offset[MAX_NUM_COMP][MAX_CCSAO_SET_NUM][MAX_CCSAO_CLASS_NUM];
  CcSaoComParam() { reset(); }
  void reset()
  {
    std::memset(enabled, false, sizeof(enabled));
    std::memset(extChroma, false, sizeof(extChroma));
    std::memset(reusePrv, false, sizeof(reusePrv));
    std::memset(reusePrvId, 0, sizeof(reusePrvId));
    std::memset(setNum, 0, sizeof(setNum));
    std::memset(setEnabled, false, sizeof(setEnabled));
    std::memset(setType, 0, sizeof(setType));
    std::memset(candPos, 0, sizeof(candPos));
    std::memset(bandNum, 0, sizeof(bandNum));
    std::memset(offset, 0, sizeof(offset));
  }
  void reset(CompID compID)
  {
    enabled[compID]    = false;
    extChroma[compID]  = false;
    reusePrv[compID]   = false;
    reusePrvId[compID] = 0;
    setNum[compID]     = 0;
    std::memset(setEnabled[compID], false, sizeof(setEnabled[compID]));
    std::memset(setType[compID], 0, sizeof(setType[compID]));
    std::memset(candPos[compID], 0, sizeof(candPos[compID]));
    std::memset(bandNum[compID], 0, sizeof(bandNum[compID]));
    std::memset(offset[compID], 0, sizeof(offset[compID]));
  }
  const CcSaoComParam &operator=(const CcSaoComParam &src)
  {
    std::memcpy(enabled, src.enabled, sizeof(enabled));
    std::memcpy(extChroma, src.extChroma, sizeof(extChroma));
    std::memcpy(reusePrv, src.reusePrv, sizeof(reusePrv));
    std::memcpy(reusePrvId, src.reusePrvId, sizeof(reusePrvId));
    std::memcpy(setNum, src.setNum, sizeof(setNum));
    std::memcpy(setEnabled, src.setEnabled, sizeof(setEnabled));
    std::memcpy(setType, src.setType, sizeof(setType));
    std::memcpy(candPos, src.candPos, sizeof(candPos));
    std::memcpy(bandNum, src.bandNum, sizeof(bandNum));
    std::memcpy(offset, src.offset, sizeof(offset));
    return *this;
  }
};

struct CcSaoPrvParam
{
  bool     enabled;
  bool     extChroma;
  bool     reusePrv;
  int      reusePrvId;
  int      temporalId;
  uint8_t  setNum;
  bool     setEnabled[MAX_CCSAO_SET_NUM];
  bool     setType[MAX_CCSAO_SET_NUM];
  uint16_t candPos[MAX_CCSAO_SET_NUM][MAX_NUM_COMP];
  uint16_t bandNum[MAX_CCSAO_SET_NUM][MAX_NUM_COMP];
  short    offset[MAX_CCSAO_SET_NUM][MAX_CCSAO_CLASS_NUM];
  CcSaoPrvParam() { reset(); }
  void reset()
  {
    enabled    = false;
    extChroma  = false;
    reusePrv   = false;
    reusePrvId = 0;
    temporalId = 0;
    setNum     = 0;
    std::memset(setEnabled, false, sizeof(setEnabled));
    std::memset(setType, 0, sizeof(setType));
    std::memset(candPos, 0, sizeof(candPos));
    std::memset(bandNum, 0, sizeof(bandNum));
    std::memset(offset, 0, sizeof(offset));
  }
  const CcSaoPrvParam &operator=(const CcSaoPrvParam &src)
  {
    enabled    = src.enabled;
    extChroma  = src.extChroma;
    reusePrv   = src.reusePrv;
    reusePrvId = src.reusePrvId;
    temporalId = src.temporalId;
    setNum     = src.setNum;
    std::memcpy(setEnabled, src.setEnabled, sizeof(setEnabled));
    std::memcpy(setType, src.setType, sizeof(setType));
    std::memcpy(candPos, src.candPos, sizeof(candPos));
    std::memcpy(bandNum, src.bandNum, sizeof(bandNum));
    std::memcpy(offset, src.offset, sizeof(offset));
    return *this;
  }
};

//! \}

#endif  // end of #ifndef  __ALFPARAMETERS__
