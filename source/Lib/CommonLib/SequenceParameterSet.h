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

#include <map>
#include "HRD.h"
#include "ProfileTierLevel.h"
#include "ChromaFormat.h"

struct VUI
{
  bool m_progressiveSourceFlag { false };
  bool m_interlacedSourceFlag { false };
  bool m_nonPackedFlag { false };
  bool m_nonProjectedFlag { false };
  bool m_aspectRatioInfoPresentFlag { false };
  bool m_aspectRatioConstantFlag { false };
  int  m_aspectRatioIdc { 0 };
  int  m_sarWidth { 0 };
  int  m_sarHeight { 0 };
  bool m_overscanInfoPresentFlag { false };
  bool m_overscanAppropriateFlag { false };
  bool m_colourDescriptionPresentFlag { false };
  int  m_colourPrimaries { 2 };
  int  m_transferCharacteristics { 2 };
  int  m_matrixCoefficients { 2 };
  bool m_videoFullRangeFlag { false };
  bool m_chromaLocInfoPresentFlag { false };
  int  m_chromaSampleLocTypeTopField { 6 };
  int  m_chromaSampleLocTypeBottomField { 6 };
  int  m_chromaSampleLocType { 6 };
};

struct ReferencePictureList
{
public:
  int  m_numberOfShorttermPictures { 0 };
  int  m_numberOfLongtermPictures { 0 };
  int  m_numberOfActivePictures { 0 };
  int  m_numberOfInterLayerPictures { 0 };
  bool m_ltrpInSliceHeaderFlag { false };
  bool m_interLayerPresentFlag { false };
  bool m_isLongtermRefPic[MAX_NUM_REF_PICS] {};
  int  m_refPicIdentifier[MAX_NUM_REF_PICS] {};  // This can be delta POC for STRP or POC LSB for LTRP
  int  m_POC[MAX_NUM_REF_PICS] {};
  bool m_deltaPocMSBPresentFlag[MAX_NUM_REF_PICS] {};
  int  m_deltaPOCMSBCycleLT[MAX_NUM_REF_PICS] {};
  bool m_isInterLayerRefPic[MAX_NUM_REF_PICS] {};
  int  m_interLayerRefPicIdx[MAX_NUM_REF_PICS] {};
  int  m_POCvalue { 0 };

  ReferencePictureList(const bool interLayerPicPresentFlag = false) : m_interLayerPresentFlag(interLayerPicPresentFlag)
  {
    ::memset(m_isLongtermRefPic, 0, sizeof(m_isLongtermRefPic));
    ::memset(m_refPicIdentifier, 0, sizeof(m_refPicIdentifier));
    ::memset(m_POC, 0, sizeof(m_POC));
    ::memset(m_isInterLayerRefPic, 0, sizeof(m_isInterLayerRefPic));
    ::memset(m_interLayerRefPicIdx, 0, sizeof(m_interLayerRefPicIdx));
    ::memset(m_deltaPOCMSBCycleLT, 0, sizeof(m_deltaPOCMSBCycleLT));
    ::memset(m_deltaPocMSBPresentFlag, 0, sizeof(m_deltaPocMSBPresentFlag));
  }

  void setRefPicIdentifier(int idx, int identifier, bool isLongterm, bool isInterLayerRefPic, int interLayerIdx)
  {
    m_refPicIdentifier[idx]       = identifier;
    m_isLongtermRefPic[idx]       = isLongterm;
    m_deltaPocMSBPresentFlag[idx] = false;
    m_deltaPOCMSBCycleLT[idx]     = 0;
    m_isInterLayerRefPic[idx]     = isInterLayerRefPic;
    m_interLayerRefPicIdx[idx]    = interLayerIdx;
  }

  int getNumRefEntries() const
  {
    return m_numberOfShorttermPictures + m_numberOfLongtermPictures + m_numberOfInterLayerPictures;
  }
  void printRefPicInfo() const;
};

// Reference Picture List set class
struct RPLList
{
private:
  std::vector<ReferencePictureList> m_referencePictureLists;

public:
  void create(int numberOfEntries) { m_referencePictureLists.resize(numberOfEntries); }
  void destroy() {}

  ReferencePictureList *getReferencePictureList(int referencePictureListIdx)
  {
    return &m_referencePictureLists[referencePictureListIdx];
  }
  const ReferencePictureList *getReferencePictureList(int referencePictureListIdx) const
  {
    return &m_referencePictureLists[referencePictureListIdx];
  }
  int getNumberOfReferencePictureLists() const { return int(m_referencePictureLists.size()); }
};

struct Window
{
  bool m_enabledFlag { false };
  int  m_winLeftOffset { 0 };
  int  m_winRightOffset { 0 };
  int  m_winTopOffset { 0 };
  int  m_winBottomOffset { 0 };

  void setWindow(int offsetLeft, int offsetRight, int offsetTop, int offsetBottom)
  {
    m_enabledFlag     = (offsetLeft || offsetRight || offsetTop || offsetBottom);
    m_winLeftOffset   = offsetLeft;
    m_winRightOffset  = offsetRight;
    m_winTopOffset    = offsetTop;
    m_winBottomOffset = offsetBottom;
  }
};

struct ChromaQpMappingTableParams
{
  int              m_qpBdOffset { 12 };
  bool             m_sameCQPTableForAllChromaFlag { true };
  int              m_numQpTables { 1 };
  int              m_qpTableStartMinus26[MAX_NUM_CQP_MAPPING_TABLES] {};
  int              m_numPtsInCQPTableMinus1[MAX_NUM_CQP_MAPPING_TABLES] {};
  std::vector<int> m_deltaQpInValMinus1[MAX_NUM_CQP_MAPPING_TABLES] {};
  std::vector<int> m_deltaQpOutVal[MAX_NUM_CQP_MAPPING_TABLES] {};
};

struct ChromaQpMappingTable : ChromaQpMappingTableParams
{
  std::map<int, int> m_chromaQpMappingTables[MAX_NUM_CQP_MAPPING_TABLES];

  int getMappedChromaQpValue(CompID compID, const int qpVal) const
  {
    return m_chromaQpMappingTables[m_sameCQPTableForAllChromaFlag ? 0 : (int)compID - 1].at(qpVal);
  }
  void deriveChromaQPMappingTables();
  void setParams(const ChromaQpMappingTableParams &params, const int qpBdOffset);
};

// SPS RExt class
struct SPSRExt // Names aligned to text specification
{
  bool m_transformSkipRotationEnabledFlag { false };
  bool m_transformSkipContextEnabledFlag { false };
  bool m_extendedPrecisionProcessingFlag { false };
  bool m_tsrcRicePresentFlag { false };
  bool m_highPrecisionOffsetsEnabledFlag { false };
  bool m_rrcRiceExtensionEnableFlag { false };
  bool m_persistentRiceAdaptationEnabledFlag { false };
  bool m_reverseLastSigCoeffEnabledFlag { false };
  bool m_cabacBypassAlignmentEnabledFlag { false };

  bool settingsDifferFromDefaults() const
  {
    return m_transformSkipRotationEnabledFlag || m_transformSkipContextEnabledFlag ||
      m_extendedPrecisionProcessingFlag || m_tsrcRicePresentFlag || m_highPrecisionOffsetsEnabledFlag ||
      m_rrcRiceExtensionEnableFlag || m_persistentRiceAdaptationEnabledFlag || m_reverseLastSigCoeffEnabledFlag ||
      m_cabacBypassAlignmentEnabledFlag;
  }
};

// SPS class
struct SPS
{
  int          m_spsId { 0 };
  int          m_vpsId { 0 };
  int          m_layerId { 0 };
  bool         m_affineAmvrEnabledFlag { false };
  bool         m_useMMVD { false };
  bool         m_affineParaRefinement { false };
  bool         m_useSBT { false };
  ChromaFormat m_chromaFormatIdc { ChromaFormat::_420 };

  uint32_t m_maxSubLayers { 1 };            // maximum number of temporal layers
  bool     m_ptlDpbHrdParamsPresentFlag { true };
  bool     m_subLayerDpbParamsFlag { false };

  // Structure
  uint32_t m_maxWidthInLumaSamples { 352 };
  uint32_t m_maxHeightInLumaSamples { 288 };
  Window   m_conformanceWindow {};

  bool                  m_subPicInfoPresentFlag { false };        // indicates the presence of sub-picture info
  uint32_t              m_numSubPics { 1 };            // number of sub-pictures used
  bool                  m_independentSubPicsFlag { false };
  bool                  m_subPicSameSizeFlag { false };
  std::vector<uint32_t> m_subPicCtuTopLeftX {};
  std::vector<uint32_t> m_subPicCtuTopLeftY {};
  std::vector<uint32_t> m_subPicWidth {};
  std::vector<uint32_t> m_subPicHeight {};
  std::vector<bool>     m_subPicTreatedAsPicFlag {};
  std::vector<bool>     m_loopFilterAcrossSubpicEnabledFlag {};
  bool                  m_subPicIdMappingExplicitlySignalledFlag { false };
  bool                  m_subPicIdMappingPresentFlag { false };
  uint32_t              m_subPicIdLen { 16 };           // sub-picture ID length in bits
  std::vector<uint16_t> m_subPicId {};               // sub-picture ID for each sub-picture in the sequence
  bool                  m_useInterRPL { 0 };
  std::vector<int32_t>  m_QPoffsetRPL {};
  int                   m_log2MinCodingBlockSize { 2 };
  unsigned              m_ctuSize { 0 };
  unsigned              m_partitionOverrideEnabled { false };   // enable partition constraints override function
  unsigned              m_minQT[3] = { 0, 0, 0 };   // 0: I slice luma; 1: P/B slice; 2: I slice chroma
  unsigned              m_maxMTTHierarchyDepth[3] { MAX_BT_DEPTH, MAX_BT_DEPTH_INTER, MAX_BT_DEPTH_C };
  unsigned              m_maxBTSize[3] { 0, 0, 0 };
  unsigned              m_maxTTSize[3] { 0, 0, 0 };
  bool                  m_idrRefParamList { false };
  unsigned              m_dualITree { false };
  uint32_t              m_maxCuWidth { 64 };
  uint32_t              m_maxCuHeight { 64 };

  RPLList  m_rplList[NUM_RPL01] {};
  uint32_t m_numRpl[NUM_RPL01] { 0, 0 };

  bool m_rpl1CopyFromRpl0Flag { false };
  bool m_rpl1IdxPresentFlag { false };
  bool m_allRplEntriesHasSameSignFlag { true };
  bool m_longTermRefsPresent { false };
  bool m_temporalMvpEnabledFlag { false };
  int  m_maxNumReorderPics[MAX_TLAYER] {};

  // Tool list
  bool                        m_transformSkipEnabledFlag { false };
  int                         m_log2MaxTransformSkipBlockSize { false };
  bool                        m_bdpcmEnabledFlag { false };
  bool                        m_jointCbCrEnabledFlag { false };
  // Parameter
  BitDepths                   m_bitDepths;
  bool                        m_entropyCodingSyncEnabledFlag { false };   // Flag for enabling WPP
  bool                        m_entryPointPresentFlag { false };   // Flag for indicating the presence of entry points
  EnumArray<int, ChannelType> m_qpBDOffset {};
  BitDepths                   m_internalMinusInputBitDepth {};   //  max(0, internal bitdepth - input bitdepth)

  bool     m_sbtmvpEnabledFlag { false };
  bool     m_bdofEnabledFlag { false };
  bool     m_dmvdBDOFExt { false };
  bool     m_fpelMmvdEnabledFlag { false };
  bool     m_bdofControlPresentInPhFlag { false };
  bool     m_dmvrControlPresentInPhFlag { false };
  bool     m_profControlPresentInPhFlag { false };
  uint32_t m_bitsForPoc { 8 };
  bool     m_pocMsbCycleFlag { false };
  uint32_t m_pocMsbCycleLen { 1 };
  int      m_numExtraPHBytes { 0 };
  int      m_numExtraSHBytes { 0 };

  std::vector<bool>    m_extraPHBitPresentFlag { false };
  std::vector<bool>    m_extraSHBitPresentFlag { false };
  uint32_t             m_numLongTermRefPicSPS { 0 };
  uint32_t             m_ltRefPicPocLsbSps[MAX_NUM_LONG_TERM_REF_PICS];
  bool                 m_usedByCurrPicLtSPSFlag[MAX_NUM_LONG_TERM_REF_PICS];
  uint32_t             m_log2MaxTbSize { 6 };
  bool                 m_useWP { false };   // Use of Weighting Prediction (P_SLICE)
  bool                 m_useBiWP { false };   // Use of Weighting Bi-Prediction (B_SLICE)
  bool                 m_saoEnabledFlag { false };
  bool                 m_ccSaoEnabledFlag { false };
  bool                 m_ccSaoFastFlag { false };
  bool                 m_lfCccmEnabledFlag { false };
  bool                 m_temporalIdNestingFlag { false };   // temporal_id_nesting_flag
  bool                 m_scalingListEnabledFlag { false };
  bool                 m_depQuantEnabledFlag { false };   // dependent quantization enabled flag
  bool                 m_signDataHidingEnabledFlag { false };   // sign data hiding enabled flag
  int                  m_numPredSign { 0 };   // Number of predicted transform coefficient signs
  int                  m_log2SignPredArea { 0 };   // Log2 of width/height of area for sign prediction
  bool                 m_tempCabacInitMode { true };
  uint32_t             m_maxDecPicBuffering[MAX_TLAYER] {};
  uint32_t             m_maxLatencyIncreasePlus1[MAX_TLAYER] {};
  bool                 m_generalHrdParametersPresentFlag { false };
  GeneralHrdParams     m_generalHrdParams {};
  OlsHrdParams         m_olsHrdParams[8] {};
  bool                 m_fieldSeqFlag { false };
  bool                 m_vuiParametersPresentFlag { false };
  unsigned             m_vuiPayloadSize { 0 };
  VUI                  m_vuiParameters {};
  SPSRExt              m_spsRangeExtension {};
  ProfileTierLevel     m_profileTierLevel {};
  bool                 m_alfEnabledFlag { false };
  bool                 m_alfImprovementsEnabledFlag { false };
  bool                 m_ccalfEnabledFlag { false };
  bool                 m_wrapAroundEnabledFlag { false };
  bool                 m_ibcFlag { false };
  unsigned             m_ibcFracFlag { 0 };
  unsigned             m_ibcFlagInterSlice { 0 };
  bool                 m_ibcMerge { false };
  unsigned             m_PLTMode { false };
  bool                 m_lmcsEnabled { false };
  bool                 m_AMVREnabledFlag { false };
  bool                 m_LMChroma { false };
  bool                 m_CCCM { false };
  bool                 m_MCBP { false };
  bool                 m_TMBP { false };
  bool                 m_horCollocatedChromaFlag { true };
  bool                 m_verCollocatedChromaFlag { false };
  bool                 m_mtsEnabled { false };
  bool                 m_explicitMtsIntra { false };
  bool                 m_explicitMtsInter { false };
  bool                 m_useIntraLFNSTinISlice { false };
  bool                 m_useIntraLFNSTinPBSlice { false };
  bool                 m_useInterLFNST { false };
  bool                 m_useInterLFNSTSBT { false };
  bool                 m_useSMVD { false };
  bool                 m_useAffine { false };
  bool                 m_AffineType { false };
  bool                 m_AffineMmvdMode { false };
  bool                 m_log2MinAffineBlkSizeMinus3 { 1 };
  bool                 m_usePROF { false };
  bool                 m_affineSbMrgExt { false };
  bool                 m_useBcw { false };
  bool                 m_useCiip { false };
  bool                 m_useGeo { false };
  bool                 m_useSgpm { false };
  bool                 m_useDMVD { false };
  bool                 m_useObmc { false };
  bool                 m_ladfEnabled { false };
  int                  m_ladfNumIntervals { 0 };
  int                  m_ladfQpOffset[MAX_LADF_INTERVALS] {};
  int                  m_ladfIntervalLowerBound[MAX_LADF_INTERVALS] {};
  int                  m_interMTSMaxSize { 32 };
  bool                 m_bvgCccm { false };
  bool                 m_ccBoostFilter { false };
  bool                 m_ccBoostTplRefSel { false };
  bool                 m_ccMerge { false };
  bool                 m_ccMergeFusion { false };
  bool                 m_ccDecDerivedMode { false };
  bool                 m_useMRL { false };
  bool                 m_useMIP { false };
  bool                 m_usedirPlanar { false };
  bool                 m_useDIMD { false };
  bool                 m_useDIMDChroma { false };
  bool                 m_useTIMD { false };
  bool                 m_useTIMDSAD { false };
  bool                 m_useTIMDMerge { false };
  bool                 m_useOBIC { false };
  bool                 m_useEIP { false };
  bool                 m_useMMEIP { false };
  ChromaQpMappingTable m_chromaQpMappingTable {};
  bool                 m_GDREnabledFlag { true };
  bool                 m_SubLayerCbpParametersPresentFlag { false };
  bool                 m_rprEnabledFlag { false };
  bool                 m_resChangeInClvsEnabledFlag { false };
  bool                 m_interLayerPresentFlag { false };
  uint32_t             m_log2ParallelMergeLevelMinus2 { 0 };
  bool                 m_ppsValidFlag[MAX_NUM_PPS] {};
  Size                 m_scalingWindowSizeInPPS[MAX_NUM_PPS] {};
  uint32_t             m_maxNumMergeCand { MRG_MAX_NUM_CANDS };
  uint32_t             m_maxNumAffineMergeCand { AFFINE_MRG_MAX_NUM_CANDS };
  uint32_t             m_maxNumIBCMergeCand { IBC_MRG_MAX_NUM_CANDS };
  uint32_t             m_maxNumGeoCand { 0 };
  uint32_t             m_maxNumBMMergeCand { BM_MRG_MAX_NUM_CANDS };
  bool                 m_mergeOppositeLic { false };
  uint32_t             m_maxNumOppositeLicMergeCand {
    REG_MRG_MAX_NUM_CANDS_OPPOSITELIC
  };   ///< Max number of merge candidates with opposite LIC flag
  uint32_t m_maxNumAffineOppositeLicMergeCand {
    AFF_MRG_MAX_NUM_CANDS_OPPOSITELIC
  };   ///< Max number of affine merge candidates with opposite LIC flag
  bool m_disableScalingMatrixForLfnstBlks { true };
  bool m_licEnabledFlag { false };
  bool m_biLicEnabledFlag { false };
  bool m_pdpEnabledFlag { false };
  bool m_tempPartPredEnabledFlag { false };
#if ENABLE_NNLF
  int  m_nnlf { 0 };
  bool m_nnlfStore { false };
#endif
  bool m_useAdditionalCMVP { false };

  SPS();

  void setNumSubPics(uint32_t u);

  void setMinQTSizes(const unsigned *minQT)
  {
    m_minQT[0] = minQT[0];
    m_minQT[1] = minQT[1];
    m_minQT[2] = minQT[2];
  }
  unsigned getMinQTSize(SliceType slicetype, ChannelType chType = ChannelType::LUMA) const
  {
    return slicetype == I_SLICE ? (isLuma(chType) ? m_minQT[0] : m_minQT[2]) : m_minQT[1];
  }
  void setMaxMTTHierarchyDepth(unsigned maxMTTHierarchyDepth, unsigned maxMTTHierarchyDepthI,
                               unsigned maxMTTHierarchyDepthIChroma)
  {
    m_maxMTTHierarchyDepth[1] = maxMTTHierarchyDepth;
    m_maxMTTHierarchyDepth[0] = maxMTTHierarchyDepthI;
    m_maxMTTHierarchyDepth[2] = maxMTTHierarchyDepthIChroma;
  }

  unsigned getMaxMTTHierarchyDepth() const { return m_maxMTTHierarchyDepth[1]; }
  unsigned getMaxMTTHierarchyDepthI() const { return m_maxMTTHierarchyDepth[0]; }
  unsigned getMaxMTTHierarchyDepthIChroma() const { return m_maxMTTHierarchyDepth[2]; }

  void setMaxBTSize(unsigned maxBTSize, unsigned maxBTSizeI, unsigned maxBTSizeC)
  {
    m_maxBTSize[1] = maxBTSize;
    m_maxBTSize[0] = maxBTSizeI;
    m_maxBTSize[2] = maxBTSizeC;
  }

  unsigned getMaxBTSize() const { return m_maxBTSize[1]; }
  unsigned getMaxBTSizeI() const { return m_maxBTSize[0]; }
  unsigned getMaxBTSizeIChroma() const { return m_maxBTSize[2]; }

  void setMaxTTSize(unsigned maxTTSize, unsigned maxTTSizeI, unsigned maxTTSizeC)
  {
    m_maxTTSize[1] = maxTTSize;
    m_maxTTSize[0] = maxTTSizeI;
    m_maxTTSize[2] = maxTTSizeC;
  }

  unsigned getMaxTTSize() const { return m_maxTTSize[1]; }
  unsigned getMaxTTSizeI() const { return m_maxTTSize[0]; }
  unsigned getMaxTTSizeIChroma() const { return m_maxTTSize[2]; }

  void     createRplList(RefPicList l, int numRPL);
  uint32_t getMaxTbSize() const { return 1 << m_log2MaxTbSize; }
  int      getMaxLog2TrDynamicRange(ChannelType channelType) const
  {
    return m_spsRangeExtension.m_extendedPrecisionProcessingFlag ? std::min<int>(20, int(m_bitDepths[channelType] + 6))
                                                                 : 15;
  }

  void setChromaQpMappingTableFromParams(const ChromaQpMappingTableParams &params, const int qpBdOffset)
  {
    m_chromaQpMappingTable.setParams(params, qpBdOffset);
  }
  void deriveChromaQPMappingTables() { m_chromaQpMappingTable.deriveChromaQPMappingTables(); }
  int  getMappedChromaQpValue(CompID compID, int qpVal) const
  {
    return m_chromaQpMappingTable.getMappedChromaQpValue(compID, qpVal);
  }

  static int getWinUnitX(ChromaFormat cf)
  {
    return isChromaEnabled(cf) ? 1 << getChannelTypeScaleX(ChannelType::CHROMA, cf) : 1;
  }
  static int getWinUnitY(ChromaFormat cf) { return 1 << getChannelTypeScaleY(ChannelType::CHROMA, cf); }
};
