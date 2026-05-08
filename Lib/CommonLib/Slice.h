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

#pragma once

#include <array>
#include <cstring>
#include <list>
#include <map>
#include <type_traits>
#include <vector>
#include <unordered_map>
#include "CommonDef.h"
#include "Rom.h"
#include "ChromaFormat.h"
#include "Common.h"
#include "HRD.h"
#include "AlfParameters.h"
#include "AlfParametersEcm.h"
#include "AlfParametersVtm.h"
#include "ConstraintInfo.h"
#include "ProfileTierLevel.h"
#include "VideoParameterSet.h"
#include "SequenceParameterSet.h"
#include "PictureParameterSet.h"

struct Picture;
class TrQuant;

typedef std::list<Picture *> PicList;

#if ENABLE_NNLF
// parameters signaled at slice level
struct NnlfSliceParameters
{
  int mode; // -1: off
  int scaleId;
  int scale[CompID::MAX_NUM_COMP][2];
  int offset[CompID::MAX_NUM_COMP][2][4];

  NnlfSliceParameters() { reset(); }
  void reset()
  {
    mode    = -1;
    scaleId = -1;
    std::memset(scale, 0, sizeof(scale));
    std::memset(offset, 0, sizeof(offset));
  }
};

// parameters used to filter the picture
struct NnlfFilterParameters
{
  int                 block_size;
  int                 extension;
  int                 nb_blocks_width;
  int                 nb_blocks_height;
  int                 prmNum;
  std::vector<int>    prmId; // -1: off
  NnlfSliceParameters sprm;
  bool                temporal;
};
#endif

/// SCALING_LIST class
class ScalingList
{
public:
  ScalingList();
  virtual ~ScalingList() {}

  int *getScalingListAddress(uint32_t scalingListId)
  {
    return &(m_scalingListCoef[scalingListId][0]);
  } //!< get matrix coefficient
  const int *getScalingListAddress(uint32_t scalingListId) const
  {
    return &(m_scalingListCoef[scalingListId][0]);
  } //!< get matrix coefficient
  void checkPredMode(uint32_t scalingListId);

  static const int *getScalingListDefaultAddress(uint32_t scalinListId);   //!< get default matrix coefficient
  void              processDefaultMatrix(uint32_t scalinListId);

  void processRefMatrix(uint32_t scalingListId, uint32_t refListId);

  int  lengthUvlc(int uiCode);
  int  lengthSvlc(int uiCode);
  void CheckBestPredScalingList(int scalingListId, int predListIdx, int &BitsCount);
  void codePredScalingList(int *scalingList, const int *scalingListPred, int scalingListDC, int scalingListPredDC,
                           int scalinListId, int &bitsCost);
  void codeScalingList(int *scalingList, int scalingListDC, int scalinListId, int &bitsCost);
  bool isLumaScalingList(int scalingListId) const;
  void checkDcOfMatrix();
  bool xParseScalingList(const std::string &fileName);
  void setDefaultScalingList();
  bool isNotDefaultScalingList();

  bool operator==(const ScalingList &other)
  {
    if (memcmp(m_scalingListPredModeFlagIsCopy, other.m_scalingListPredModeFlagIsCopy,
               sizeof(m_scalingListPredModeFlagIsCopy)))
    {
      return false;
    }
    if (memcmp(m_scalingListDC, other.m_scalingListDC, sizeof(m_scalingListDC)))
    {
      return false;
    }
    if (memcmp(m_refMatrixId, other.m_refMatrixId, sizeof(m_refMatrixId)))
    {
      return false;
    }
    if (memcmp(m_scalingListCoef, other.m_scalingListCoef, sizeof(m_scalingListCoef)))
    {
      return false;
    }

    return true;
  }

  bool operator!=(const ScalingList &other) { return !(*this == other); }

public:
  void             outputScalingLists(std::ostream &os) const;
  bool             m_scalingListPredModeFlagIsCopy[30];   //!< reference list index
  int              m_scalingListDC[30];   //!< the DC value of the matrix coefficient for 16x16
  uint32_t         m_refMatrixId[30];   //!< RefMatrixID
  bool             m_scalingListPreditorModeFlag[30];   //!< reference list index
  std::vector<int> m_scalingListCoef[30];   //!< quantization matrix
  bool             m_chromaScalingListPresentFlag;
};

struct CheckCRAFlags
{
  void clear()
  {
    seenTrailingFieldPic     = false;
    seenLeadingFieldPic      = false;
    trailingFieldHadRefIssue = false;
  }

  bool seenTrailingFieldPic { false };   ///< whether or not have seen trailing field picture after CRA
  bool seenLeadingFieldPic { false };   ///< whether or not have seen leading field picture after CRA
  bool trailingFieldHadRefIssue {
    false
  };   ///< whether or not first trailing field picture had forbidden references pictures
};

struct SliceReshapeInfo
{
  bool     sliceReshaperEnableFlag { false };
  bool     sliceReshaperModelPresentFlag { false };
  unsigned enableChromaAdj { 0 };
  uint32_t reshaperModelMinBinIdx { 0 };
  uint32_t reshaperModelMaxBinIdx { 0 };
  int      reshaperModelBinCWDelta[PIC_CODE_CW_BINS] {};
  int      maxNbitsNeededDeltaCW { 0 };
  int      chrResScalingOffset { 0 };

  bool operator==(const SliceReshapeInfo &other)
  {
    if ((sliceReshaperEnableFlag != other.sliceReshaperEnableFlag) ||
        (sliceReshaperModelPresentFlag != other.sliceReshaperModelPresentFlag) ||
        (enableChromaAdj != other.enableChromaAdj) || (reshaperModelMinBinIdx != other.reshaperModelMinBinIdx) ||
        (reshaperModelMaxBinIdx != other.reshaperModelMaxBinIdx) ||
        (maxNbitsNeededDeltaCW != other.maxNbitsNeededDeltaCW) || (chrResScalingOffset != other.chrResScalingOffset) ||
        (memcmp(reshaperModelBinCWDelta, other.reshaperModelBinCWDelta, sizeof(reshaperModelBinCWDelta))))
    {
      return false;
    }
    return true;
  }

  bool operator!=(const SliceReshapeInfo &other) { return !(*this == other); }
};

struct ReshapeCW
{
  std::vector<uint32_t> binCW {};
  int                   updateCtrl { 0 };
  int                   adpOption { 0 };
  uint32_t              initialCW { 0 };
  int                   rspPicSize { 0 };
  int                   rspFps { 0 };
  int                   rspBaseQP { 0 };
  int                   rspTid { 0 };
  int                   rspSliceQP { 0 };
  int                   rspFpsToIp { 0 };
};

struct DCI
{
  int                           m_maxSubLayers { 0 };
  std::vector<ProfileTierLevel> m_profileTierLevel;

  size_t getNumPTLs() const { return m_profileTierLevel.size(); }
  bool   IsIndenticalDCI(const DCI &comparedDCI) const
  {
    if (m_maxSubLayers != comparedDCI.m_maxSubLayers)
    {
      return false;
    }
    if (m_profileTierLevel != comparedDCI.m_profileTierLevel)
    {
      return false;
    }
    return true;
  }
};

struct OPI
{
  bool     m_olsinfopresentflag { false };
  bool     m_htidinfopresentflag { false };
  uint32_t m_opiolsidx { std::numeric_limits<uint32_t>::max() };
  uint32_t m_opihtidplus1 { std::numeric_limits<uint32_t>::max() };
};

/// @brief Helper class ensuring that VtmAlfParam and EcmAlfParam are sub-classes of BaseAlfParam.
template<class BaseAlfParam, class VtmAlfParam, class EcmAlfParam,
         std::enable_if_t<std::is_base_of<BaseAlfParam, VtmAlfParam>::value &&
                          std::is_base_of<BaseAlfParam, EcmAlfParam>::value> * = nullptr>
class ApsAlfParamSFINAE
{};

// Forward declaration of container class template needed for declaration of comparison function.
template<class BaseAlfParam, class VtmAlfParam, class EcmAlfParam> class ApsAlfParamHelper;

// Declaration of container class template comparison function needed for definition outside class.
template<class BaseAlfParam, class VtmAlfParam, class EcmAlfParam>
bool operator==(const ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam> &lhs,
                const ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam> &rhs);

/// @brief Container class template for VTM or new APS ALF parameters.
template<class BaseAlfParam, class VtmAlfParam, class EcmAlfParam> class ApsAlfParamHelper
  : private ApsAlfParamSFINAE<BaseAlfParam, VtmAlfParam, EcmAlfParam>
{
public:
  static void setAlfType(const bool isEcmAlf);

  ApsAlfParamHelper();
  ApsAlfParamHelper(const ApsAlfParamHelper &other);
  ApsAlfParamHelper(ApsAlfParamHelper &&other) noexcept;

  virtual ~ApsAlfParamHelper();

  ApsAlfParamHelper &operator=(const ApsAlfParamHelper &other);
  ApsAlfParamHelper &operator=(ApsAlfParamHelper &&other) noexcept;

  friend bool operator== <>(const ApsAlfParamHelper &lhs, const ApsAlfParamHelper &rhs);
  friend bool operator!=(const ApsAlfParamHelper &lhs, const ApsAlfParamHelper &rhs) { return !(lhs == rhs); }

  void create();

  inline BaseAlfParam       &getParam();
  inline const BaseAlfParam &getParam() const;
  inline BaseAlfParam       &getParamNoCheck();
  const BaseAlfParam        &getParamNoCheck() const { return *static_cast<const BaseAlfParam *>(m_param); }

  inline EcmAlfParam       &getEcmParam();
  inline const EcmAlfParam &getEcmParam() const;
  inline EcmAlfParam       &getEcmParamNoCheck();
  const EcmAlfParam        &getEcmParamNoCheck() const { return *static_cast<const EcmAlfParam *>(m_param); }

  inline VtmAlfParam       &getVtmParam();
  inline const VtmAlfParam &getVtmParam() const;
  inline VtmAlfParam       &getVtmParamNoCheck();
  const VtmAlfParam        &getVtmParamNoCheck() const { return *static_cast<const VtmAlfParam *>(m_param); }

  static bool isEcmAlf() { return m_alfType == AlfType::ECM; }
  static bool isVtmAlf() { return m_alfType == AlfType::VTM; }

  bool isValid() const { return m_param != nullptr; }

private:
  enum class AlfType : int8_t
  {
    UNDEFINED = -1,
    VTM       = 0,
    ECM       = 1,
  };

  inline static AlfType m_alfType = AlfType::UNDEFINED;

  BaseAlfParam *m_param;
};

template<class BaseAlfParam, class VtmAlfParam, class EcmAlfParam>
bool operator==(const ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam> &lhs,
                const ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam> &rhs)
{
  CHECK(!lhs.isValid() || !rhs.isValid(), "Trying to compare invalid parameters.");
  switch (ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam>::m_alfType)
  {
  case ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam>::AlfType::ECM:
    return lhs.getEcmParamNoCheck() == rhs.getEcmParamNoCheck();
  case ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam>::AlfType::VTM:
    return lhs.getVtmParamNoCheck() == rhs.getVtmParamNoCheck();
  default:
    THROW("Invalid ALF type.");
  }
}

template<class BaseAlfParam, class VtmAlfParam, class EcmAlfParam>
BaseAlfParam &ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam>::getParamNoCheck()
{
  return const_cast<BaseAlfParam &>(
    const_cast<const ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam> *>(this)->getParamNoCheck());
}

template<class BaseAlfParam, class VtmAlfParam, class EcmAlfParam>
EcmAlfParam &ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam>::getEcmParamNoCheck()
{
  return const_cast<EcmAlfParam &>(
    const_cast<const ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam> *>(this)->getEcmParamNoCheck());
}

template<class BaseAlfParam, class VtmAlfParam, class EcmAlfParam>
VtmAlfParam &ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam>::getVtmParamNoCheck()
{
  return const_cast<VtmAlfParam &>(
    const_cast<const ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam> *>(this)->getVtmParamNoCheck());
}

template<class BaseAlfParam, class VtmAlfParam, class EcmAlfParam>
const BaseAlfParam &ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam>::getParam() const
{
  CHECK(m_param == nullptr, "No parameters are available.");
  return getParamNoCheck();
}

template<class BaseAlfParam, class VtmAlfParam, class EcmAlfParam>
BaseAlfParam &ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam>::getParam()
{
  return const_cast<BaseAlfParam &>(
    const_cast<const ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam> *>(this)->getParam());
}

template<class BaseAlfParam, class VtmAlfParam, class EcmAlfParam>
const EcmAlfParam &ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam>::getEcmParam() const
{
  CHECK(m_param == nullptr, "No parameters are available.");
  CHECK(m_alfType != AlfType::ECM, "Container does not hold new ALF parameters.");
  return getEcmParamNoCheck();
}

template<class BaseAlfParam, class VtmAlfParam, class EcmAlfParam>
EcmAlfParam &ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam>::getEcmParam()
{
  return const_cast<EcmAlfParam &>(
    const_cast<const ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam> *>(this)->getEcmParam());
}

template<class BaseAlfParam, class VtmAlfParam, class EcmAlfParam>
const VtmAlfParam &ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam>::getVtmParam() const
{
  CHECK(m_param == nullptr, "No parameters are available.");
  CHECK(m_alfType != AlfType::VTM, "Container does not hold VTM ALF parameters.");
  return getVtmParamNoCheck();
}

template<class BaseAlfParam, class VtmAlfParam, class EcmAlfParam>
VtmAlfParam &ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam>::getVtmParam()
{
  return const_cast<VtmAlfParam &>(
    const_cast<const ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam> *>(this)->getVtmParam());
}

using ApsAlfParam =
  ApsAlfParamHelper<AlfParameters::AlfParamBase, AlfParametersVtm::AlfParam, AlfParametersEcm::AlfParam>;
using ApsCcAlfParam = ApsAlfParamHelper<AlfParameters::CcAlfFilterParamBase, AlfParametersVtm::CcAlfFilterParam,
                                        AlfParametersEcm::CcAlfFilterParam>;

struct APS
{
  int              m_APSId { 0 };   // adaptation_parameter_set_id
  int              m_temporalId { 0 };
  int              m_puCounter { 0 };
  int              m_layerId { 0 };
  ApsType          m_APSType { ApsType::ALF };   // aps_params_type
  ApsAlfParam      m_alfAPSParam {};
  SliceReshapeInfo m_reshapeAPSInfo {};
  ScalingList      m_scalingListApsInfo {};
  ApsCcAlfParam    m_ccAlfAPSParam {};
  bool             m_hasPrefixNalUnitType { false };
  bool             chromaPresentFlag { false };
};

struct WPScalingParam
{
  // Explicit weighted prediction parameters parsed in slice header,
  // or Implicit weighted prediction parameters (8 bits depth values).
  bool     presentFlag { false };
  uint32_t log2WeightDenom { 0 };
  int      codedWeight { 0 };
  int      codedOffset { 0 };

  // Weighted prediction scaling values built from above parameters (bitdepth scaled):
  int w { 0 };
  int o { 0 };
  int offset { 0 };
  int shift { 0 };
  int round { 0 };

  static bool isWeighted(const WPScalingParam *wp);
};

inline bool WPScalingParam::isWeighted(const WPScalingParam *wp)
{
  return wp != nullptr && (wp[COMP_Y].presentFlag || wp[COMP_Cb].presentFlag || wp[COMP_Cr].presentFlag);
}

struct WPACDCParam
{
  int64_t ac { 0 };
  int64_t dc { 0 };
};

// picture header class
struct PicHeader
{
  bool                 m_valid { false };   //!< picture header is valid yet or not
  Picture             *m_pic { nullptr };   //!< pointer to picture structure
  int                  m_pocLsb { 0 };   //!< least significant bits of picture order count
  bool                 m_nonReferencePictureFlag { false };   //!< non-reference picture flag
  bool                 m_gdrOrIrapPicFlag { false };   //!< gdr or irap picture flag
  bool                 m_gdrPicFlag { false };   //!< gradual decoding refresh picture flag
  uint32_t             m_recoveryPocCnt { uint32_t(-1) };   //!< recovery POC count
  bool                 m_noOutputBeforeRecoveryFlag { false };   //!< NoOutputBeforeRecoveryFlag
  bool                 m_handleCraAsCvsStartFlag { false };   //!< HandleCraAsCvsStartFlag
  bool                 m_handleGdrAsCvsStartFlag { false };   //!< HandleGdrAsCvsStartFlag
  int                  m_spsId { -1 };   //!< sequence parameter set ID
  int                  m_ppsId { -1 };   //!< picture parameter set ID
  bool                 m_pocMsbPresentFlag { false };   //!< ph_poc_msb_present_flag
  int                  m_pocMsbVal { 0 };   //!< poc_msb_val
  bool                 m_picOutputFlag { true };   //!< picture output flag
  ReferencePictureList m_rpl[NUM_RPL01] {};
  int      m_rplIdx[NUM_RPL01] {};   // index of used RPL in the SPS or -1 for local RPL in the picture header
  bool     m_picInterSliceAllowedFlag { false };   //!< inter slice allowed flag in PH
  bool     m_picIntraSliceAllowedFlag { false };   //!< intra slice allowed flag in PH
  bool     m_splitConsOverrideFlag { false };   //!< partitioning constraint override flag
  uint32_t m_cuQpDeltaSubdivIntra { 0 };   //!< CU QP delta maximum subdivision for intra slices
  uint32_t m_cuQpDeltaSubdivInter { 0 };   //!< CU QP delta maximum subdivision for inter slices
  uint32_t m_cuChromaQpOffsetSubdivIntra { 0 };   //!< CU chroma QP offset maximum subdivision for intra slices
  uint32_t m_cuChromaQpOffsetSubdivInter { 0 };   //!< CU chroma QP offset maximum subdivision for inter slices
  bool     m_enableTMVPFlag { true };   //!< enable temporal motion vector prediction
  bool     m_picColFromL0Flag { true };   //!< syntax element collocated_from_l0_flag
  uint32_t m_colRefIdx { 0 };
  bool     m_mvdL1ZeroFlag { false };   //!< L1 MVD set to zero flag
  uint32_t m_maxNumAffineMergeCand { AFFINE_MRG_MAX_NUM_CANDS };   //!< max number of sub-block merge candidates
  bool     m_disFracMMVD { false };   //!< fractional MMVD offsets disabled flag
  bool     m_bdofDisabledFlag { false };   //!< picture level BDOF disable flag
  bool     m_profDisabledFlag { false };   //!< picture level PROF disable flag
  bool     m_jointCbCrSignFlag { false };   //!< joint Cb/Cr residual sign flag
  bool     m_gpmMMVDTableFlag { false };   //!< gpm mmvd extension enable flag
  int      m_qpDelta { 0 };   //!< value of Qp delta
#if ENABLE_NNLF
  bool m_nnlfDisabled { false };
#endif
  EnumArray<bool, ChannelType> m_saoEnabledFlag;   // sao enabled flags for each channel
  bool                         m_ccSaoEnabledFlag[MAX_NUM_COMP] {};   // ccsao enabled flags for each channel
  bool                         m_alfEnabledFlag[MAX_NUM_COMP] {};   //!< alf enabled flags for each component
  int m_newAlfFixFiltSetCandIdx[MAX_NUM_COMP] {};   //!< fixed filter set candidate index for the new ALF
  int m_numAlfApsIdsLuma { 0 };   //!< number of alf aps active for the picture

  AlfParameters::AlfApsList m_alfApsIdsLuma;   // list of ALF APSs for the picture

  int      m_alfApsIdChroma { 0 };   //!< chroma alf aps ID
  bool     m_ccalfEnabledFlag[MAX_NUM_COMP] {};
  int      m_ccAlfCbApsId { 0 };
  int      m_ccAlfCrApsId { 0 };
  bool     m_deblockingFilterOverrideFlag { false };   //!< deblocking filter override controls enabled
  bool     m_deblockingFilterDisable { false };   //!< deblocking filter disabled flag
  int      m_deblockingFilterBetaOffsetDiv2 { 0 };   //!< beta offset for deblocking filter
  int      m_deblockingFilterTcOffsetDiv2 { 0 };   //!< tc offset for deblocking filter
  int      m_deblockingFilterCbBetaOffsetDiv2 { 0 };   //!< beta offset for deblocking filter
  int      m_deblockingFilterCbTcOffsetDiv2 { 0 };   //!< tc offset for deblocking filter
  int      m_deblockingFilterCrBetaOffsetDiv2 { 0 };   //!< beta offset for deblocking filter
  int      m_deblockingFilterCrTcOffsetDiv2 { 0 };   //!< tc offset for deblocking filter
  bool     m_lmcsEnabledFlag { false };   //!< lmcs enabled flag
  int      m_lmcsApsId { -1 };   //!< lmcs APS ID
  APS     *m_lmcsAps { nullptr };   //!< lmcs APS
  bool     m_lmcsChromaResidualScaleFlag { false };   //!< lmcs chroma residual scale flag
  bool     m_explicitScalingListEnabledFlag { false };   //!< explicit quantization scaling list enabled
  int      m_scalingListApsId { -1 };   //!< quantization scaling list APS ID
  APS     *m_scalingListAps { nullptr };   //!< quantization scaling list APS
  unsigned m_minQT[3] {};   //!< minimum quad-tree size  0: I slice luma; 1: P/B slice; 2: I slice chroma
  unsigned m_maxMTTHierarchyDepth[3] {};   //!< maximum MTT depth
  unsigned m_maxBTSize[3] {};   //!< maximum BT size
  unsigned m_maxTTSize[3] {};   //!< maximum TT size

  RefSetArray<WPScalingParam[MAX_NUM_COMP]> m_weightPredTable;

  int m_numWeights[NUM_RPL01];   // number of weights for each list

  PicHeader();
  ~PicHeader();
  void initPicHeader();

  void setMinQTSize(unsigned idx, unsigned minQT) { m_minQT[idx] = minQT; }
  void setMaxMTTHierarchyDepth(unsigned idx, unsigned maxMTT) { m_maxMTTHierarchyDepth[idx] = maxMTT; }
  void setMaxBTSize(unsigned idx, unsigned maxBT) { m_maxBTSize[idx] = maxBT; }
  void setMaxTTSize(unsigned idx, unsigned maxTT) { m_maxTTSize[idx] = maxTT; }

  void setMinQTSizes(const unsigned *minQT)
  {
    m_minQT[0] = minQT[0];
    m_minQT[1] = minQT[1];
    m_minQT[2] = minQT[2];
  }
  void setMaxMTTHierarchyDepths(const unsigned *maxMTT)
  {
    m_maxMTTHierarchyDepth[0] = maxMTT[0];
    m_maxMTTHierarchyDepth[1] = maxMTT[1];
    m_maxMTTHierarchyDepth[2] = maxMTT[2];
  }
  void setMaxBTSizes(const unsigned *maxBT)
  {
    m_maxBTSize[0] = maxBT[0];
    m_maxBTSize[1] = maxBT[1];
    m_maxBTSize[2] = maxBT[2];
  }
  void setMaxTTSizes(const unsigned *maxTT)
  {
    m_maxTTSize[0] = maxTT[0];
    m_maxTTSize[1] = maxTT[1];
    m_maxTTSize[2] = maxTT[2];
  }

  unsigned getMinQTSize(SliceType slicetype, ChannelType chType = ChannelType::LUMA) const
  {
    return slicetype == I_SLICE ? (isLuma(chType) ? m_minQT[0] : m_minQT[2]) : m_minQT[1];
  }
  unsigned getMaxMTTHierarchyDepth(SliceType slicetype, ChannelType chType = ChannelType::LUMA) const
  {
    return slicetype == I_SLICE ? (isLuma(chType) ? m_maxMTTHierarchyDepth[0] : m_maxMTTHierarchyDepth[2])
                                : m_maxMTTHierarchyDepth[1];
  }
  unsigned getMaxBTSize(SliceType slicetype, ChannelType chType = ChannelType::LUMA) const
  {
    return slicetype == I_SLICE ? (isLuma(chType) ? m_maxBTSize[0] : m_maxBTSize[2]) : m_maxBTSize[1];
  }
  unsigned getMaxTTSize(SliceType slicetype, ChannelType chType = ChannelType::LUMA) const
  {
    return slicetype == I_SLICE ? (isLuma(chType) ? m_maxTTSize[0] : m_maxTTSize[2]) : m_maxTTSize[1];
  }

  void setWpScaling(const WPScalingParam *wp)
  {
    memcpy(m_weightPredTable, wp, sizeof(WPScalingParam) * NUM_RPL01 * MAX_NUM_REF * MAX_NUM_COMP);
  }
  const WPScalingParam *getWpScaling(const RefPicList refPicList, const int refIdx) const;
  WPScalingParam       *getWpScaling(const RefPicList refPicList, const int refIdx);
  WPScalingParam       *getWpScalingAll() { return (WPScalingParam *)m_weightPredTable; }
  void                  resetWpScaling();
};

/// slice header class
struct Slice
{
  //  Bitstream writing
  EnumArray<bool, ChannelType> m_saoEnabledFlag {};
  bool                         m_ccSaoEnabledFlag[MAX_NUM_COMP] {};
  int                          m_poc { 0 };
  int                          m_iLastIDR { 0 };
  int                          m_prevGDRInSameLayerPOC { -MAX_INT };   //< the previous GDR in the same layer
  int                          m_iAssociatedIRAPPOC { 0 };
  NalUnitType                  m_iAssociatedIRAPType { NAL_UNIT_INVALID };
  int                          m_prevGDRSubpicPOC { -MAX_INT };
  int                          m_prevIRAPSubpicPOC { -MAX_INT };
  NalUnitType                  m_prevIRAPSubpicType { NAL_UNIT_INVALID };
  bool                         m_enableDRAPSEI { false };
  bool                         m_useLTforDRAP { false };
  bool                         m_isDRAP { false };
  int                          m_latestDRAPPOC { 0 };
  bool                         m_enableEdrapSEI { false };
  int                          m_edrapRapId { 0 };
  bool                         m_useLTforEdrap { false };
  int                          m_edrapNumRefRapPics { 0 };
  std::vector<int>             m_edrapRefRapIds {};
  int                          m_latestEDRAPPOC { 0 };
  bool                         m_latestEdrapLeadingPicDecodableFlag { false };
  ReferencePictureList         m_rpl[NUM_RPL01] {};
  int         m_rplIdx[NUM_RPL01] { -1, -1 };   //< index of used RPL in the SPS or -1 for local RPL in the slice header
  NalUnitType m_eNalUnitType { NAL_UNIT_CODED_SLICE_IDR_W_RADL };   ///< Nal unit type for the slice
  bool        m_pictureHeaderInSliceHeader { false };
  uint32_t    m_nuhLayerId { 0 };   ///< Nal unit layer id
  SliceType   m_eSliceType { I_SLICE };
  bool        m_noOutputOfPriorPicsFlag { false };   //!< no output of prior pictures flag
  int         m_iSliceQp { 0 };
  int         m_iSliceQpBase { 0 };
  bool        m_chromaQpAdjEnabled { false };
  bool        m_lmcsEnabledFlag { false };
  bool        m_explicitScalingListUsed { false };
  bool        m_deblockingFilterDisable { false };
  bool        m_deblockingFilterOverrideFlag { false };   //< offsets for deblocking filter inherit from PPS
  int         m_deblockingFilterBetaOffsetDiv2 { 0 };   //< beta offset for deblocking filter
  int         m_deblockingFilterTcOffsetDiv2 { 0 };   //< tc offset for deblocking filter
  int         m_deblockingFilterCbBetaOffsetDiv2 { 0 };   //< beta offset for deblocking filter
  int         m_deblockingFilterCbTcOffsetDiv2 { 0 };   //< tc offset for deblocking filter
  int         m_deblockingFilterCrBetaOffsetDiv2 { 0 };   //< beta offset for deblocking filter
  int         m_deblockingFilterCrTcOffsetDiv2 { 0 };   //< tc offset for deblocking filter
  int         m_depQuantEnabledIdc { 0 };   //!< dependent quantization enabled idc (0: off, 1: 4 states, 2: 8 states)
  int         m_riceBaseLevelValue { 0 };   //< baseLevel value for abs_remainder
  bool        m_reverseLastSigCoeffFlag { false };
  bool        m_signDataHidingEnabledFlag { false };   //!< sign data hiding enabled flag
  bool        m_tsResidualCodingDisabledFlag { false };
  int         m_list1IdxToList0Idx[MAX_NUM_REF] {};
  int         m_numRefIdx[NUM_RPL01] {};   //  for multiple reference of current slice
  bool        m_pendingRasInit { false };
  bool        m_checkLdc { false };
  bool        m_biDirPred { false };
  bool        m_lmChromaCheckDisable { false };
  int         m_symRefIdx[2] {};
  bool        m_meetBiPredT { false };

  //  Data
  int          m_sliceChromaQpDelta[MAX_NUM_COMP + 1] {};
  Picture     *m_refPicList[NUM_RPL01][MAX_NUM_REF + 1] {};
  int          m_refPOCList[NUM_RPL01][MAX_NUM_REF + 1] {};
  int          m_refRefIdxList[NUM_RPL01][MAX_NUM_REF][NUM_RPL01][MAX_NUM_REF + 1][NUM_RPL01];
  bool         m_isUsedAsLongTerm[NUM_RPL01][MAX_NUM_REF + 1] {};
  int          m_hierPredLayerIdx { 0 };   // hierarchical prediction layer index
  Picture     *m_scaledRefPicList[NUM_RPL01][MAX_NUM_REF + 1] {};
  Picture     *m_savedRefPicList[NUM_RPL01][MAX_NUM_REF + 1] {};
  ScalingRatio m_scalingRatio[NUM_RPL01][MAX_NUM_REF_PICS] {};

  // access channel
  const VPS       *m_vps { nullptr };
  const SPS       *m_sps { nullptr };
  const PPS       *m_pps { nullptr };
  Picture         *m_pic { nullptr };
  const PicHeader *m_picHeader { nullptr };   //!< pointer to picture header structure
  bool             m_colFromL0Flag { true };   //!< collocated picture from List0 flag
  uint32_t         m_colRefIdx { 0 };
  double           m_lambdas[MAX_NUM_COMP] {};
  uint32_t         m_uiTLayer { 0 };
  SliceMap         m_sliceMap {};   //!< list of CTUs in current slice - raster scan CTU addresses
  uint32_t         m_independentSliceIdx { 0 };
  bool             m_nextSlice { false };
  bool             m_testWeightPred { false };
  bool             m_testWeightBiPred { false };
  RefSetArray<WPScalingParam[MAX_NUM_COMP]> m_weightPredTable {};
  WPACDCParam                               m_weightACDCParam[MAX_NUM_COMP] {};
  ClpRngs                                   m_clpRngs {};
  std::vector<uint32_t>                     m_substreamSizes {};
  uint32_t                                  m_numEntryPoints { 0 };
  uint32_t                                  m_numSubstream { 0 };
  bool                                      m_cabacInitFlag { false };
#if ENABLE_CABAC_DUMP
  SliceType m_cabacInitSliceType { I_SLICE };
#endif
  uint32_t  m_sliceSubPicId {};
  SliceType m_encCABACTableIdx { I_SLICE };   //!< Used to transmit table selection across slices.
  clock_t   m_iProcessingStartTime {};
  double    m_dProcessingTime {};
  int       m_rpPicOrderCntVal { 0 };
  APS      *m_alfApss[AlfParameters::ALF_CTB_MAX_NUM_APS] {};
  bool      m_alfEnabledFlag[MAX_NUM_COMP] {};
  int       m_newAlfFixFiltSetCandIdx[MAX_NUM_COMP] {};   //!< fixed filter set candidate index for the new ALF
  int       m_numAlfApsIdsLuma { 0 };
  AlfParameters::AlfApsList m_alfApsIdsLuma {};
  int                       m_alfApsIdChroma { 0 };
  bool                      m_ccAlfCbEnabledFlag { false };
  bool                      m_ccAlfCrEnabledFlag { false };
  int                       m_ccAlfCbApsId { 0 };
  int                       m_ccAlfCrApsId { 0 };

  bool                    m_lfCccmEnabledFlag;
  std::vector<int8_t>     m_lfCccmEnabled;
  std::vector<int8_t>     m_lfCccmWindowSizeIndex;
  std::vector<int8_t>     m_lfCccmModelType;
  std::vector<int8_t>     m_lfCccmCTUMerge;
  int8_t                  m_lfCccmFrameLevelInherit;
  void                    lfCccmClearControlInformation(const int ctuRsAddr = -1);
  void                    lfCccmMerge(const int ctuRsAddr);
  const Picture          *lfCccmGetReferencePicture() const;
  lfCccmCand              lfCccmGetCandidate(const int ctuRsAddr) const;
  std::vector<lfCccmCand> lfCccmGetMergeCandidates(const int ctuRsAddr) const;

  bool     m_disableSATDForRd { false };
  bool     m_isLossless { false };
  int      m_tsrcIndex { 0 };
  unsigned m_riceBit[8] {};
  int      m_cntRightBottom { 0 };
  bool     m_ibcFlag { false };
  bool     m_useLic { false };
  int      m_lumaPelMax { 0 };
  int      m_lumaPelMin { 0 };
  bool     m_adaptiveClipQuant { 0 };
#if ENABLE_NNLF
  NnlfSliceParameters m_nnlfUnifiedParam {};
#endif

  Slice();
  ~Slice();
  void initSlice();
  void inheritFromPicHeader(PicHeader *picHeader, const PPS *pps, const SPS *sps);

  void setAlfAPSs(APS **apss) { memcpy(m_alfApss, apss, sizeof(m_alfApss)); }
  void checkSubpicTypeConstraints(PicList &rcListPic, const ReferencePictureList *pRPL0,
                                  const ReferencePictureList *pRPL1, const int prevIRAPSubpicDecOrderNo);
  bool getUseWeightedPrediction() const
  {
    return ((m_eSliceType == P_SLICE && m_testWeightPred) || (m_eSliceType == B_SLICE && m_testWeightBiPred));
  }
  int        getSliceChromaQpDelta(CompID compID) const { return isLuma(compID) ? 0 : m_sliceChromaQpDelta[compID]; }
  Picture   *getRefPic(RefPicList e, int refIdx) const { return m_refPicList[e][refIdx]; }
  int        getRefPOC(RefPicList e, int refIdx) const { return m_refPOCList[e][refIdx]; }
  const int *getRefRefIdx(RefPicList e, int iRefIdx, RefPicList refe, int iRefRefIdx) const
  {
    return m_refRefIdxList[e][iRefIdx][refe][iRefRefIdx];
  }
  void checkColRefIdx(uint32_t curSliceSegmentIdx, const Picture *pic);
  bool getRapPicFlag() const;
  bool getIdrPicFlag() const
  {
    return m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL || m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP;
  }
  bool isIRAP() const
  {
    return (m_eNalUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL) && (m_eNalUnitType <= NAL_UNIT_CODED_SLICE_CRA);
  }
  // CLVSS PU is either an IRAP PU with NoOutputBeforeRecoveryFlag equal to 1 or a GDR PU with
  // NoOutputBeforeRecoveryFlag equal to 1.
  bool isClvssPu() const
  {
    return m_eNalUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL && m_eNalUnitType <= NAL_UNIT_CODED_SLICE_GDR &&
      !m_pps->m_mixedNaluTypesInPicFlag && m_picHeader->m_noOutputBeforeRecoveryFlag;
  }
  bool isIDRorBLA() const
  {
    return (m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL) || (m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP);
  }
  bool isLeadingPic() const
  {
    return m_eNalUnitType == NAL_UNIT_CODED_SLICE_RADL || m_eNalUnitType == NAL_UNIT_CODED_SLICE_RASL;
  }
  void checkCRA(const ReferencePictureList *pRPL0, const ReferencePictureList *pRPL1, const int pocCRA,
                CheckCRAFlags &flags, PicList &rcListPic);
  void checkSTSA(PicList &rcListPic);
  void checkRPL(const ReferencePictureList *pRPL0, const ReferencePictureList *pRPL1,
                const int associatedIRAPDecodingOrderNumber, PicList &rcListPic);
  void decodingRefreshMarking(int &pocCRA, bool &bRefreshPending, PicList &rcListPic,
                              const bool bEfficientFieldIRAPEnabled);
  void setSliceChromaQpDelta(CompID compID, int i) { m_sliceChromaQpDelta[compID] = isLuma(compID) ? 0 : i; }

  void constructRefPicList(PicList &rcListPic);
  void setRefPOCList();
  void setRefRefIdxList();

  void setBiDirPred(bool b, int refIdx0, int refIdx1)
  {
    m_biDirPred    = b;
    m_symRefIdx[0] = refIdx0;
    m_symRefIdx[1] = refIdx1;
  }
  bool getBiDirPred() const { return m_biDirPred; }
  int  getSymRefIdx(int refList) const { return m_symRefIdx[refList]; }

  bool isIntra() const { return m_eSliceType == I_SLICE; }
  bool isInterB() const { return m_eSliceType == B_SLICE; }
  bool isInterP() const { return m_eSliceType == P_SLICE; }
  bool cvsHasPreviousDRAP() const { return m_latestDRAPPOC != MAX_INT; }
  bool isPocRestrictedByDRAP(int poc, bool precedingDRAPinDecodingOrder);
  bool isPOCInRefPicList(const ReferencePictureList *rpl, int poc);
  void checkConformanceForDRAP(uint32_t temporalId);
  int  getEdrapRefRapId(int idx) const { return m_edrapRefRapIds[idx]; }
  void addEdrapRefRapIds(int i) { m_edrapRefRapIds.push_back(i); }
  void deleteEdrapRefRapIds(int i)
  {
    m_edrapRefRapIds.erase(m_edrapRefRapIds.begin() + i);
    m_edrapNumRefRapPics--;
  }
  bool isPocRestrictedByEdrap(int poc);
  bool cvsHasPreviousEDRAP() const { return m_latestEDRAPPOC != MAX_INT; }
  void checkConformanceForEDRAP(uint32_t temporalId);

  void setLambdas(const double lambdas[MAX_NUM_COMP])
  {
    for (int component = 0; component < MAX_NUM_COMP; component++)
    {
      m_lambdas[component] = lambdas[component];
    }
  }
  const double *getLambdas() const { return m_lambdas; }

  uint32_t getCuQpDeltaSubdiv() const
  {
    return this->isIntra() ? m_picHeader->m_cuQpDeltaSubdivIntra : m_picHeader->m_cuQpDeltaSubdivInter;
  }
  uint32_t getCuChromaQpOffsetSubdiv() const
  {
    return this->isIntra() ? m_picHeader->m_cuChromaQpOffsetSubdivIntra : m_picHeader->m_cuChromaQpOffsetSubdivInter;
  }

  static void sortPicList(PicList &rcListPic);
  void        setList1IdxToList0Idx();

  void checkLeadingPictureRestrictions(PicList &rcListPic, const PPS &pps) const;
  int  checkThatAllRefPicsAreAvailable(PicList &rcListPic, const ReferencePictureList *pRPL, int rplIdx,
                                       bool printErrors, int *refPicIndex, int numActiveRefPics) const;

  void applyReferencePictureListBasedMarking(PicList &rcListPic, const ReferencePictureList *pRPL0,
                                             const ReferencePictureList *pRPL1, const int layerId,
                                             const PPS &pps) const;
  bool isTemporalLayerSwitchingPoint(PicList &rcListPic) const;
  bool isStepwiseTemporalLayerSwitchingPointCandidate(PicList &rcListPic) const;
  int  checkThatAllRefPicsAreAvailable(PicList &rcListPic, const ReferencePictureList *pRPL, int rplIdx,
                                       bool printErrors) const;

  void     setNumTilesInSlice(uint32_t u) { m_sliceMap.m_numTilesInSlice = u; }
  uint32_t getNumTilesInSlice() const { return m_sliceMap.m_numTilesInSlice; }
  void     setSliceMap(SliceMap map) { m_sliceMap = map; }
  uint32_t getFirstCtuRsAddrInSlice() const { return m_sliceMap.getCtuAddrInSlice(0); }
  void     setSliceID(uint32_t u) { m_sliceMap.m_sliceID = u; }
  uint32_t getSliceID() const { return m_sliceMap.m_sliceID; }
  uint32_t getNumCtuInSlice() const { return m_sliceMap.m_numCtuInSlice; }
  uint32_t getCtuAddrInSlice(int idx) const { return m_sliceMap.getCtuAddrInSlice(idx); }
  void     addCtusToSlice(uint32_t startX, uint32_t stopX, uint32_t startY, uint32_t stopY, uint32_t picWidthInCtbsY)
  {
    m_sliceMap.addCtusToSlice(startX, stopX, startY, stopY, picWidthInCtbsY);
  }
  void copySliceInfo(Slice *pcSliceSrc, bool cpyAlmostAll = true);

  void setWpScaling(RefSetArray<WPScalingParam[MAX_NUM_COMP]> &wp)
  {
    memcpy(m_weightPredTable, wp, sizeof(m_weightPredTable));
  }
  void            setWpScaling(const WPScalingParam *wp) { memcpy(m_weightPredTable, wp, sizeof(m_weightPredTable)); }
  WPScalingParam *getWpScalingAll() { return (WPScalingParam *)m_weightPredTable; }
  WPScalingParam *getWpScaling(const RefPicList refPicList, const int refIdx);
  const WPScalingParam *getWpScaling(const RefPicList refPicList, const int refIdx) const;

  void resetWpScaling();
  void initWpScaling(const SPS *sps);

  void setWpAcDcParam(WPACDCParam wp[MAX_NUM_COMP])
  {
    memcpy(m_weightACDCParam, wp, sizeof(WPACDCParam) * MAX_NUM_COMP);
  }

  void getWpAcDcParam(const WPACDCParam *&wp) const;
  void initWpAcDcParam();

  void          setDefaultClpRng(const SPS &sps);
  const ClpRng &clpRng(CompID id) const { return m_clpRngs.comp[id]; }
  unsigned      getMinPictureDistance(unsigned ibcFastMethod) const;
  void          startProcessingTimer();
  void          stopProcessingTimer();
  void          resetProcessingTime() { m_dProcessingTime = m_iProcessingStartTime = 0; }
  double        getProcessingTime() const { return m_dProcessingTime; }
  void          resetCcSaoEnabledFlag() { memset(m_ccSaoEnabledFlag, 0, sizeof(m_ccSaoEnabledFlag)); }
  void          resetAlfEnabledFlag() { memset(m_alfEnabledFlag, 0, sizeof(m_alfEnabledFlag)); }
  bool scaleRefPicList(Picture *scaledRefPic[], PicHeader *picHeader, APS **apss, APS *lmcsAps, APS *scalingListAps,
                       const bool isDecoder);
  void freeScaledRefPicList(Picture *scaledRefPic[]);
  bool checkRPR();
  const ScalingRatio &getScalingRatio(const RefPicList refPicList, const int refIdx) const
  {
    CHECK(refIdx < 0, "Invalid reference index");
    return m_scalingRatio[refPicList][refIdx];
  }
  void setNumSubstream(const SPS *sps, const PPS *pps);
  void setNumEntryPoints(const SPS *sps, const PPS *pps);
  bool isLastSliceInSubpic();

  bool checkAlfAPS(const int apsId);

  ApsCcAlfParam                                                  m_ccAlfFilterParam;
  uint8_t                                                       *m_ccAlfFilterControl[2];
  std::unordered_map<Position, std::unordered_map<Size, double>> m_mapPltCost[2];   // th fix this

  CcSaoComParam m_ccSaoComParam;
  uint8_t      *m_ccSaoControl[MAX_NUM_COMP];

  ClpRng setNewClipRange(bool considerReshaper, std::vector<Pel> *fwdReshaperMapping, CompID CompID);
  int    getClipDeltaShift() const;

private:
  Picture *xGetRefPic(PicList &rcListPic, const int poc, const int layerId);
  Picture *xGetLongTermRefPic(PicList &rcListPic, const int poc, const bool pocHasMsb, const int layerId);
  Picture *xGetLongTermRefPicCandidate(PicList &rcListPic, const int poc, const bool pocHasMsb, const int layerId);
};   // END CLASS DEFINITION Slice

struct PreCalcValues
{
  PreCalcValues(const SPS &sps, const PPS &pps, bool _isEncoder)
    : chrFormat(sps.m_chromaFormatIdc)
    , multiBlock422(false)
    , maxCUWidth(sps.m_maxCuWidth)
    , maxCUHeight(sps.m_maxCuHeight)
    , maxCUWidthMask(maxCUWidth - 1)
    , maxCUHeightMask(maxCUHeight - 1)
    , maxCUWidthLog2(floorLog2(maxCUWidth))
    , maxCUHeightLog2(floorLog2(maxCUHeight))
    , minCUWidth(1 << MIN_CU_LOG2)
    , minCUHeight(1 << MIN_CU_LOG2)
    , minCUWidthLog2(floorLog2(minCUWidth))
    , minCUHeightLog2(floorLog2(minCUHeight))
    , partsInCtuWidth(maxCUWidth >> MIN_CU_LOG2)
    , partsInCtuHeight(maxCUHeight >> MIN_CU_LOG2)
    , partsInCtu(partsInCtuWidth * partsInCtuHeight)
    , widthInCtus((pps.m_picWidthInLumaSamples + sps.m_maxCuWidth - 1) / sps.m_maxCuWidth)
    , heightInCtus((pps.m_picHeightInLumaSamples + sps.m_maxCuHeight - 1) / sps.m_maxCuHeight)
    , sizeInCtus(widthInCtus * heightInCtus)
    , lumaWidth(pps.m_picWidthInLumaSamples)
    , lumaHeight(pps.m_picHeightInLumaSamples)
    , fastDeltaQPCuMaxSize(Clip3(1u << sps.m_log2MinCodingBlockSize, sps.m_maxCuHeight, 32u))
    , noChroma2x2(false)
    , isEncoder(_isEncoder)
    , ISingleTree(!sps.m_dualITree)
    , maxBtDepth { sps.getMaxMTTHierarchyDepthI(), sps.getMaxMTTHierarchyDepth(), sps.getMaxMTTHierarchyDepthIChroma() }
    , minBtSize { 1u << sps.m_log2MinCodingBlockSize, 1u << sps.m_log2MinCodingBlockSize,
                  1u << sps.m_log2MinCodingBlockSize }
    , maxBtSize { sps.getMaxBTSizeI(), sps.getMaxBTSize(), sps.getMaxBTSizeIChroma() }
    , minTtSize { 1u << sps.m_log2MinCodingBlockSize, 1u << sps.m_log2MinCodingBlockSize,
                  1u << sps.m_log2MinCodingBlockSize }
    , maxTtSize { sps.getMaxTTSizeI(), sps.getMaxTTSize(), sps.getMaxTTSizeIChroma() }
    , minQtSize { sps.getMinQTSize(I_SLICE, ChannelType::LUMA), sps.getMinQTSize(B_SLICE, ChannelType::LUMA),
                  sps.getMinQTSize(I_SLICE, ChannelType::CHROMA) }
  {}

  const ChromaFormat chrFormat;
  const bool         multiBlock422;
  const unsigned     maxCUWidth;
  const unsigned     maxCUHeight;
  // to get CTU position, use (x & maxCUWidthMask) rather than (x % maxCUWidth)
  const unsigned     maxCUWidthMask;
  const unsigned     maxCUHeightMask;
  const unsigned     maxCUWidthLog2;
  const unsigned     maxCUHeightLog2;
  const unsigned     minCUWidth;
  const unsigned     minCUHeight;
  const unsigned     minCUWidthLog2;
  const unsigned     minCUHeightLog2;
  const unsigned     partsInCtuWidth;
  const unsigned     partsInCtuHeight;
  const unsigned     partsInCtu;
  const unsigned     widthInCtus;
  const unsigned     heightInCtus;
  const unsigned     sizeInCtus;
  const unsigned     lumaWidth;
  const unsigned     lumaHeight;
  const unsigned     fastDeltaQPCuMaxSize;
  const bool         noChroma2x2;
  const bool         isEncoder;
  const bool         ISingleTree;

private:
  const unsigned maxBtDepth[3];
  const unsigned minBtSize[3];
  const unsigned maxBtSize[3];
  const unsigned minTtSize[3];
  const unsigned maxTtSize[3];
  const unsigned minQtSize[3];

  unsigned getValIdx(const Slice &slice, const ChannelType chType) const;

public:
  unsigned getMaxBtDepth(const Slice &slice, const ChannelType chType) const;
  unsigned getMinBtSize(const Slice &slice, const ChannelType chType) const;
  unsigned getMaxBtSize(const Slice &slice, const ChannelType chType) const;
  unsigned getMinTtSize(const Slice &slice, const ChannelType chType) const;
  unsigned getMaxTtSize(const Slice &slice, const ChannelType chType) const;
  unsigned getMinQtSize(const Slice &slice, const ChannelType chType) const;
};

#if ENABLE_TRACING
void xTraceVPSHeader();
void xTraceOPIHeader();
void xTraceDCIHeader();
void xTraceSPSHeader();
void xTracePPSHeader();
void xTraceAPSHeader();
void xTracePictureHeader();
void xTraceSliceHeader();
void xTraceAccessUnitDelimiter();
void xTraceFillerData();
#endif
