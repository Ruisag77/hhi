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

/** \file     EncCfg.h
    \brief    encoder configuration
*/

#pragma once

#include "CommonLib/AlfParameters.h"
#include "CommonLib/Slice.h"

#include <map>

//! \ingroup EncoderLib
//! \{

struct EncCfg;

int  autoDetermineProfile(EncCfg *encCfg);
bool hasNonZeroTemporalID(const EncCfg *encCfg);   // check presence of constant temporal ID in GOP structure
bool hasLeadingPicture(const EncCfg *encCfg);   // check presence of leading pictures in GOP structure

constexpr int TF_DEFAULT_REFS = 4;

struct CfgSEISubpictureLevel
{
  bool                     m_enabled { false };
  std::vector<Level::Name> m_refLevels {};
  bool                     m_explicitFraction { false };
  int                      m_numSubpictures { 1 };
  std::vector<int>         m_nonSubpicLayersFraction {};
  std::vector<int>         m_fractions {};
  int                      m_sliMaxSublayers { 1 };
  bool                     m_sliSublayerInfoPresentFlag { false };
};

struct SeiCfg
{
  HashType m_decodedPictureHashSEIType { HashType::NONE };
  HashType m_subpicDecodedPictureHashType { HashType::NONE };
  bool     m_bufferingPeriodSEIEnabled { false };
  bool     m_pictureTimingSEIEnabled { false };
  bool     m_frameFieldInfoSEIEnabled { false };
  bool     m_dependentRAPIndicationSEIEnabled { false };
  bool     m_edrapIndicationSEIEnabled { false };
  bool     m_framePackingSEIEnabled { false };
  int      m_framePackingSEIType { 0 };
  int      m_framePackingSEIId { 0 };
  int      m_framePackingSEIQuincunx { 0 };
  int      m_framePackingSEIInterpretation { 0 };
  bool     m_doSEIEnabled { false };
  bool     m_doSEICancelFlag { true };
  bool     m_doSEIPersistenceFlag { false };
  int      m_doSEITransformType { 0 };
  bool     m_parameterSetsInclusionIndicationSEIEnabled { false };
  bool     m_selfContainedClvsFlag { false };
  bool     m_bpDeltasGOPStructure { false };
  bool     m_decodingUnitInfoSEIEnabled { false };

  bool m_scalableNestingSEIEnabled { false };

  bool                  m_erpSEIEnabled { false };
  bool                  m_erpSEICancelFlag { true };
  bool                  m_erpSEIPersistenceFlag { false };
  bool                  m_erpSEIGuardBandFlag { false };
  uint32_t              m_erpSEIGuardBandType { 0 };
  uint32_t              m_erpSEILeftGuardBandWidth { 0 };
  uint32_t              m_erpSEIRightGuardBandWidth { 0 };
  bool                  m_sphereRotationSEIEnabled { false };
  bool                  m_sphereRotationSEICancelFlag { true };
  bool                  m_sphereRotationSEIPersistenceFlag { false };
  int                   m_sphereRotationSEIYaw { 0 };
  int                   m_sphereRotationSEIPitch { 0 };
  int                   m_sphereRotationSEIRoll { 0 };
  bool                  m_omniViewportSEIEnabled { false };
  uint32_t              m_omniViewportSEIId { 0 };
  bool                  m_omniViewportSEICancelFlag { true };
  bool                  m_omniViewportSEIPersistenceFlag { false };
  uint32_t              m_omniViewportSEICntMinus1 { 0 };
  std::vector<int>      m_omniViewportSEIAzimuthCentre {};
  std::vector<int>      m_omniViewportSEIElevationCentre {};
  std::vector<int>      m_omniViewportSEITiltCentre {};
  std::vector<uint32_t> m_omniViewportSEIHorRange {};
  std::vector<uint32_t> m_omniViewportSEIVerRange {};
  bool                  m_rwpSEIEnabled { false };
  bool                  m_rwpSEIRwpCancelFlag { true };
  bool                  m_rwpSEIRwpPersistenceFlag { false };
  bool                  m_rwpSEIConstituentPictureMatchingFlag { false };
  int                   m_rwpSEINumPackedRegions { 0 };
  int                   m_rwpSEIProjPictureWidth { 0 };
  int                   m_rwpSEIProjPictureHeight { 0 };
  int                   m_rwpSEIPackedPictureWidth { 0 };
  int                   m_rwpSEIPackedPictureHeight { 0 };
  std::vector<uint8_t>  m_rwpSEIRwpTransformType {};
  std::vector<bool>     m_rwpSEIRwpGuardBandFlag {};
  std::vector<uint32_t> m_rwpSEIProjRegionWidth {};
  std::vector<uint32_t> m_rwpSEIProjRegionHeight {};
  std::vector<uint32_t> m_rwpSEIRwpSEIProjRegionTop {};
  std::vector<uint32_t> m_rwpSEIProjRegionLeft {};
  std::vector<uint16_t> m_rwpSEIPackedRegionWidth {};
  std::vector<uint16_t> m_rwpSEIPackedRegionHeight {};
  std::vector<uint16_t> m_rwpSEIPackedRegionTop {};
  std::vector<uint16_t> m_rwpSEIPackedRegionLeft {};
  std::vector<uint8_t>  m_rwpSEIRwpLeftGuardBandWidth {};
  std::vector<uint8_t>  m_rwpSEIRwpRightGuardBandWidth {};
  std::vector<uint8_t>  m_rwpSEIRwpTopGuardBandHeight {};
  std::vector<uint8_t>  m_rwpSEIRwpBottomGuardBandHeight {};
  std::vector<bool>     m_rwpSEIRwpGuardBandNotUsedForPredFlag {};
  std::vector<uint8_t>  m_rwpSEIRwpGuardBandType {};
  bool                  m_gcmpSEIEnabled { false };
  bool                  m_gcmpSEICancelFlag { true };
  bool                  m_gcmpSEIPersistenceFlag { false };
  uint8_t               m_gcmpSEIPackingType { 0 };
  uint8_t               m_gcmpSEIMappingFunctionType { 0 };
  std::vector<uint8_t>  m_gcmpSEIFaceIndex {};
  std::vector<uint8_t>  m_gcmpSEIFaceRotation {};
  std::vector<double>   m_gcmpSEIFunctionCoeffU {};
  std::vector<bool>     m_gcmpSEIFunctionUAffectedByVFlag {};
  std::vector<double>   m_gcmpSEIFunctionCoeffV {};
  std::vector<bool>     m_gcmpSEIFunctionVAffectedByUFlag {};
  bool                  m_gcmpSEIGuardBandFlag { false };
  uint8_t               m_gcmpSEIGuardBandType { 0 };
  bool                  m_gcmpSEIGuardBandBoundaryExteriorFlag { false };
  uint8_t               m_gcmpSEIGuardBandSamplesMinus1 { 0 };
  CfgSEISubpictureLevel m_cfgSubpictureLevelInfoSEI {};
  bool                  m_sampleAspectRatioInfoSEIEnabled { false };
  bool                  m_sariCancelFlag { false };
  bool                  m_sariPersistenceFlag { true };
  int                   m_sariAspectRatioIdc { 0 };
  int                   m_sariSarWidth { 0 };
  int                   m_sariSarHeight { 0 };
  bool                  m_phaseIndicationSEIEnabledFullResolution { false };
  int                   m_horPhaseNumFullResolution { 0 };
  int                   m_horPhaseDenMinus1FullResolution { 0 };
  int                   m_verPhaseNumFullResolution { 0 };
  int                   m_verPhaseDenMinus1FullResolution { 0 };
  bool                  m_phaseIndicationSEIEnabledReducedResolution { false };
  int                   m_horPhaseNumReducedResolution { 0 };
  int                   m_horPhaseDenMinus1ReducedResolution { 0 };
  int                   m_verPhaseNumReducedResolution { 0 };
  int                   m_verPhaseDenMinus1ReducedResolution { 0 };
  bool                  m_MCTSEncConstraint { false };
  SEIMasteringDisplay   m_masteringDisplay {};
  bool                  m_alternativeTransferCharacteristicsSEIEnabled { false };
  int                   m_preferredTransferCharacteristics { -1 };

  bool                  m_siiSEIEnabled { false };
  uint32_t              m_siiSEINumUnitsInShutterInterval { 0 };
  uint32_t              m_siiSEITimeScale { 27000000 };
  std::vector<uint32_t> m_siiSEISubLayerNumUnitsInSI {};

  bool         m_nnPostFilterSEICharacteristicsEnabled { false };
  int          m_nnPostFilterSEICharacteristicsNumFilters { 0 };
  uint32_t     m_nnPostFilterSEICharacteristicsId[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t     m_nnPostFilterSEICharacteristicsModeIdc[MAX_NUM_NN_POST_FILTERS] {};
  bool         m_nnPostFilterSEICharacteristicsPropertyPresentFlag[MAX_NUM_NN_POST_FILTERS] {};
  bool         m_nnPostFilterSEICharacteristicsBaseFlag[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t     m_nnPostFilterSEICharacteristicsPurpose[MAX_NUM_NN_POST_FILTERS] {};
  bool         m_nnPostFilterSEICharacteristicsOutSubCFlag[MAX_NUM_NN_POST_FILTERS] {};
  ChromaFormat m_nnPostFilterSEICharacteristicsOutColourFormatIdc[MAX_NUM_NN_POST_FILTERS] {
    ChromaFormat::_420, ChromaFormat::_420, ChromaFormat::_420, ChromaFormat::_420,
    ChromaFormat::_420, ChromaFormat::_420, ChromaFormat::_420, ChromaFormat::_420
  };
  uint32_t              m_nnPostFilterSEICharacteristicsPicWidthInLumaSamples[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsPicHeightInLumaSamples[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsInpTensorBitDepthLumaMinus8[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsInpTensorBitDepthChromaMinus8[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsOutTensorBitDepthLumaMinus8[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsOutTensorBitDepthChromaMinus8[MAX_NUM_NN_POST_FILTERS] {};
  bool                  m_nnPostFilterSEICharacteristicsComponentLastFlag[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsInpFormatIdc[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsAuxInpIdc[MAX_NUM_NN_POST_FILTERS] {};
  bool                  m_nnPostFilterSEICharacteristicsSepColDescriptionFlag[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsColPrimaries[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsTransCharacteristics[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsMatrixCoeffs[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsInpOrderIdc[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsOutFormatIdc[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsOutOrderIdc[MAX_NUM_NN_POST_FILTERS] {};
  bool                  m_nnPostFilterSEICharacteristicsConstantPatchSizeFlag[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsPatchWidthMinus1[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsPatchHeightMinus1[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsExtendedPatchWidthCdDeltaMinus1[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsExtendedPatchHeightCdDeltaMinus1[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsOverlap[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsPaddingType[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsLumaPadding[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsCrPadding[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsCbPadding[MAX_NUM_NN_POST_FILTERS] {};
  std::string           m_nnPostFilterSEICharacteristicsPayloadFilename[MAX_NUM_NN_POST_FILTERS] {};
  bool                  m_nnPostFilterSEICharacteristicsComplexityInfoPresentFlag[MAX_NUM_NN_POST_FILTERS] {};
  std::string           m_nnPostFilterSEICharacteristicsUriTag[MAX_NUM_NN_POST_FILTERS] {};
  std::string           m_nnPostFilterSEICharacteristicsUri[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsParameterTypeIdc[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsLog2ParameterBitLengthMinus3[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsNumParametersIdc[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsNumKmacOperationsIdc[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsTotalKilobyteSize[MAX_NUM_NN_POST_FILTERS] {};
  uint32_t              m_nnPostFilterSEICharacteristicsNumberInputDecodedPicturesMinus1[MAX_NUM_NN_POST_FILTERS] {};
  std::vector<uint32_t> m_nnPostFilterSEICharacteristicsNumberInterpolatedPictures[MAX_NUM_NN_POST_FILTERS] {};
  std::vector<bool>     m_nnPostFilterSEICharacteristicsInputPicOutputFlag[MAX_NUM_NN_POST_FILTERS] {};

  bool     m_nnPostFilterSEIActivationEnabled { false };
  uint32_t m_nnPostFilterSEIActivationTargetId { 0 };
  bool     m_nnPostFilterSEIActivationCancelFlag { false };
  bool     m_nnPostFilterSEIActivationPersistenceFlag { false };

  // film grain characterstics sei
  bool                  m_fgcSEIEnabled { false };
  bool                  m_fgcSEICancelFlag { true };
  bool                  m_fgcSEIPersistenceFlag { false };
  uint8_t               m_fgcSEIModelID { 0 };
  bool                  m_fgcSEISepColourDescPresentFlag { false };
  uint8_t               m_fgcSEIBlendingModeID { 0 };
  uint8_t               m_fgcSEILog2ScaleFactor { 0 };
  bool                  m_fgcSEICompModelPresent[MAX_NUM_COMP] {};
  bool                  m_fgcSEIAnalysisEnabled { false };
  std::string           m_fgcSEIExternalMask { "" };
  std::string           m_fgcSEIExternalDenoised { "" };
  int                   m_fgcSEITemporalFilterPastRefs { TF_DEFAULT_REFS };
  int                   m_fgcSEITemporalFilterFutureRefs { TF_DEFAULT_REFS };
  std::map<int, double> m_fgcSEITemporalFilterStrengths {};
  bool                  m_fgcSEIPerPictureSEI { false };
  uint8_t               m_fgcSEINumModelValuesMinus1[MAX_NUM_COMP] {};
  uint8_t               m_fgcSEINumIntensityIntervalMinus1[MAX_NUM_COMP] {};
  uint8_t               m_fgcSEIIntensityIntervalLowerBound[MAX_NUM_COMP][MAX_NUM_INTENSITIES] {};
  uint8_t               m_fgcSEIIntensityIntervalUpperBound[MAX_NUM_COMP][MAX_NUM_INTENSITIES] {};
  uint32_t              m_fgcSEICompModelValue[MAX_NUM_COMP][MAX_NUM_INTENSITIES][MAX_NUM_MODEL_VALUES] {};
  // cll SEI
  bool                  m_cllSEIEnabled { false };
  uint16_t              m_cllSEIMaxContentLevel { 0 };
  uint16_t              m_cllSEIMaxPicAvgLevel { 0 };
  // ave sei
  bool                  m_aveSEIEnabled { false };
  uint32_t              m_aveSEIAmbientIlluminance { 100000 };
  uint16_t              m_aveSEIAmbientLightX { 15635 };
  uint16_t              m_aveSEIAmbientLightY { 16450 };
  // colour tranform information sei
  bool                  m_ctiSEIEnabled { false };
  uint32_t              m_ctiSEIId { 0 };
  bool                  m_ctiSEISignalInfoFlag { false };
  bool                  m_ctiSEIFullRangeFlag { false };
  uint32_t              m_ctiSEIPrimaries { 0 };
  uint32_t              m_ctiSEITransferFunction { 0 };
  uint32_t              m_ctiSEIMatrixCoefs { 0 };
  bool                  m_ctiSEICrossComponentFlag { true };
  bool                  m_ctiSEICrossComponentInferred { true };
  uint32_t              m_ctiSEINumberChromaLut { 0 };
  int                   m_ctiSEIChromaOffset { 0 };
  LutModel              m_ctiSEILut[MAX_NUM_COMP] {};
  // ccv sei
  bool                  m_ccvSEIEnabled { false };
  bool                  m_ccvSEICancelFlag { true };
  bool                  m_ccvSEIPersistenceFlag { false };
  bool                  m_ccvSEIPrimariesPresentFlag { true };
  bool                  m_ccvSEIMinLuminanceValuePresentFlag { true };
  bool                  m_ccvSEIMaxLuminanceValuePresentFlag { true };
  bool                  m_ccvSEIAvgLuminanceValuePresentFlag { true };
  double                m_ccvSEIPrimariesX[MAX_NUM_COMP] { 0.300, 0.150, 0.640 };
  double                m_ccvSEIPrimariesY[MAX_NUM_COMP] { 0.600, 0.060, 0.330 };
  double                m_ccvSEIMinLuminanceValue { 0.0 };
  double                m_ccvSEIMaxLuminanceValue { 0.1 };
  double                m_ccvSEIAvgLuminanceValue { 0.01 };
  // sdi sei
  bool                  m_sdiSEIEnabled { false };
  int                   m_sdiSEIMaxLayersMinus1 { 0 };
  bool                  m_sdiSEIMultiviewInfoFlag { false };
  bool                  m_sdiSEIAuxiliaryInfoFlag { false };
  int                   m_sdiSEIViewIdLenMinus1 { 0 };
  std::vector<uint32_t> m_sdiSEILayerId {};
  std::vector<uint32_t> m_sdiSEIViewIdVal {};
  std::vector<uint32_t> m_sdiSEIAuxId {};
  std::vector<uint32_t> m_sdiSEINumAssociatedPrimaryLayersMinus1 {};
  // mai sei
  bool                  m_maiSEIEnabled { false };
  bool                  m_maiSEIIntrinsicParamFlag { false };
  bool                  m_maiSEIExtrinsicParamFlag { false };
  int                   m_maiSEINumViewsMinus1 { 0 };
  bool                  m_maiSEIIntrinsicParamsEqualFlag { false };
  int                   m_maiSEIPrecFocalLength { 0 };
  int                   m_maiSEIPrecPrincipalPoint { 0 };
  int                   m_maiSEIPrecSkewFactor { 0 };
  std::vector<bool>     m_maiSEISignFocalLengthX {};
  std::vector<uint32_t> m_maiSEIExponentFocalLengthX {};
  std::vector<uint32_t> m_maiSEIMantissaFocalLengthX {};
  std::vector<bool>     m_maiSEISignFocalLengthY {};
  std::vector<uint32_t> m_maiSEIExponentFocalLengthY {};
  std::vector<uint32_t> m_maiSEIMantissaFocalLengthY {};
  std::vector<bool>     m_maiSEISignPrincipalPointX {};
  std::vector<uint32_t> m_maiSEIExponentPrincipalPointX {};
  std::vector<uint32_t> m_maiSEIMantissaPrincipalPointX {};
  std::vector<bool>     m_maiSEISignPrincipalPointY {};
  std::vector<uint32_t> m_maiSEIExponentPrincipalPointY {};
  std::vector<uint32_t> m_maiSEIMantissaPrincipalPointY {};
  std::vector<bool>     m_maiSEISignSkewFactor {};
  std::vector<uint32_t> m_maiSEIExponentSkewFactor {};
  std::vector<uint32_t> m_maiSEIMantissaSkewFactor {};
  int                   m_maiSEIPrecRotationParam { 0 };
  int                   m_maiSEIPrecTranslationParam { 0 };
  // mvp sei
  bool                  m_mvpSEIEnabled { false };
  int                   m_mvpSEINumViewsMinus1 { 0 };
  std::vector<uint32_t> m_mvpSEIViewPosition {};
  // aci sei
  bool                  m_aciSEIEnabled { false };
  bool                  m_aciSEICancelFlag { false };
  int                   m_aciSEIUseIdc { 0 };
  int                   m_aciSEIBitDepthMinus8 { 0 };
  int                   m_aciSEITransparentValue { 0 };
  int                   m_aciSEIOpaqueValue { 0 };
  bool                  m_aciSEIIncrFlag { false };
  bool                  m_aciSEIClipFlag { false };
  bool                  m_aciSEIClipTypeFlag { false };
  // dri sei
  bool                  m_driSEIEnabled { false };
  bool                  m_driSEIZNearFlag { false };
  bool                  m_driSEIZFarFlag { false };
  bool                  m_driSEIDMinFlag { false };
  bool                  m_driSEIDMaxFlag { false };
  double                m_driSEIZNear { 0.0 };
  double                m_driSEIZFar { 0.0 };
  double                m_driSEIDMin { 0.0 };
  double                m_driSEIDMax { 0.0 };
  int                   m_driSEIDepthRepresentationType { 0 };
  int                   m_driSEIDisparityRefViewId { 0 };
  int                   m_driSEINonlinearNumMinus1 { 0 };
  std::vector<uint32_t> m_driSEINonlinearModel {};
  std::string           m_arSEIFileRoot { "" };   // Annotated region SEI - initialized from external file

  bool                              m_SEIManifestSEIEnabled { false };
  bool                              m_SEIPrefixIndicationSEIEnabled { false };
  // SEI message processing order
  bool                              m_poSEIEnabled { false };
  std::vector<uint16_t>             m_poSEIPayloadType {};
  std::vector<uint16_t>             m_poSEIProcessingOrder {};
  std::vector<std::vector<uint8_t>> m_poSEIPrefixByte {};
  bool                              m_postFilterHintSEIEnabled { false };
  bool                              m_postFilterHintSEICancelFlag { false };
  bool                              m_postFilterHintSEIPersistenceFlag { false };
  uint32_t                          m_postFilterHintSEISizeY { 1 };
  uint32_t                          m_postFilterHintSEISizeX { 1 };
  uint32_t                          m_postFilterHintSEIType { 0 };
  bool                              m_postFilterHintSEIChromaCoeffPresentFlag { false };
  std::vector<int32_t>              m_postFilterHintValues {};

#if JVET_Z0120_SII_SEI_PROCESSING
  bool        m_ShutterFilterEnable { false };   // enable Pre-Filtering with Shutter Interval SEI
  int         m_SII_BlendingRatio { 0 };
  std::string m_shutterIntervalPreFileName { "" };
#endif
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct RPLEntry
{
  int    m_POC { -1 };
  int    m_temporalId { 0 };
  bool   m_refPic { false };
  int    m_numRefPicsActive { 0 };
  int8_t m_sliceType { 'P' };
  int    m_numRefPics { 0 };
  int    m_deltaRefPics[MAX_NUM_REF_PICS] {};
  bool   m_isEncoded { false };
  bool   m_ltrpInSliceHeaderFlag { false };
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct GOPEntry
{
  int    m_POC { -1 };
  int    m_QPOffset { 0 };
  double m_QPOffsetModelOffset { 0 };
  double m_QPOffsetModelScale { 0 };
#if W0038_CQP_ADJ
  int m_CbQPoffset { 0 };
  int m_CrQPoffset { 0 };
#endif
  double m_QPFactor { 0 };
  int    m_tcOffsetDiv2 { 0 };
  int    m_betaOffsetDiv2 { 0 };
  int    m_CbTcOffsetDiv2 { 0 };
  int    m_CbBetaOffsetDiv2 { 0 };
  int    m_CrTcOffsetDiv2 { 0 };
  int    m_CrBetaOffsetDiv2 { 0 };
  int    m_temporalId { 0 };
  bool   m_refPic { false };
  int8_t m_sliceType { 'P' };
  int    m_numRefPicsActive0 { 0 };
  int    m_numRefPics0 { 0 };
  int    m_deltaRefPics0[MAX_NUM_REF_PICS] {};
  int    m_numRefPicsActive1 { 0 };
  int    m_numRefPics1 { 0 };
  int    m_deltaRefPics1[MAX_NUM_REF_PICS] {};
  bool   m_isEncoded { false };
  bool   m_ltrpInSliceHeaderFlag { false };
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if ER_CHROMA_QP_WCG_PPS
struct WCGChromaQPControl
{
  bool   enabled { false };   // Enabled flag (0:default)
  double chromaCbQpScale { 1.0 };   // Chroma Cb QP Scale (1.0:default)
  double chromaCrQpScale { 1.0 };   // Chroma Cr QP Scale (1.0:default)
  double chromaQpScale { 0.0 };   // Chroma QP Scale (0.0:default)
  double chromaQpOffset { 0.0 };   // Chroma QP Offset (0.0:default)
};
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if SHARP_LUMA_DELTA_QP
struct LumaLevelToDeltaQPMapping
{
  LumaLevelToDQPMode               mode { LUMALVL_TO_DQP_DISABLED };   // use deltaQP determined by block luma level
  double                           maxMethodWeight { 1.0 };   // weight of max luma value when mode = 2
  std::vector<std::pair<int, int>> mapping { 0 };   // first=luma level, second=delta QP.
#if ENABLE_QPA
  bool isEnabled() const { return (mode != LUMALVL_TO_DQP_DISABLED && mode != LUMALVL_TO_DQP_NUM_MODES); }
#else
  bool isEnabled() const { return mode != LUMALVL_TO_DQP_DISABLED; }
#endif
};
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct EncCfg
{
  SeiCfg m_seiCfg {};

  int          m_maxLayers { 1 };
  int          m_targetOlsIdx { 500 };
  int          m_maxSublayers { 7 };
  int          m_olsModeIdc { 0 };
  int          m_numOutputLayerSets { 1 };
  int          m_numPtlsInVps { 1 };
  bool         m_defaultPtlDpbHrdMaxTidFlag { true };
  bool         m_allIndependentLayersFlag { true };
  bool         m_eachLayerIsAnOlsFlag { true };
  int          m_layerId[MAX_VPS_LAYERS] {};
  std::string  m_predDirectionArray { "" };
  std::string  m_refLayerIdxStr[MAX_VPS_LAYERS] {};
  std::string  m_olsOutputLayerStr[MAX_VPS_LAYERS] {};
  std::string  m_maxTidILRefPicsPlus1Str[MAX_VPS_LAYERS] {};
  int          m_ptPresentInPtl[MAX_NUM_OLSS] {};
  int          m_olsPtlIdx[MAX_NUM_OLSS] {};
  Level::Name  m_levelPtl[MAX_NUM_OLSS] {};
  int          m_conformanceWindowMode { 1 };
  int          m_confWinLeft { 0 };
  int          m_confWinRight { 0 };
  int          m_confWinTop { 0 };
  int          m_confWinBottom { 0 };
  ChromaFormat m_inputChromaFormatIDC { ChromaFormat::_420 };
  bool         m_rasterSliceFlag { false };
  BitDepths    m_inputBitDepth { 8, 0 };   // bit-depth of input file
  BitDepths    m_outputBitDepth { 0, 0 };   // bit depth of output file
  BitDepths    m_msbExtendedBitDepth { 0, 0 };   // bit depth of input samples after MSB extension
  BitDepths    m_internalBitDepth { 0, 0 };   // bit depth codec operates at (input/output files will be converted)
  int          m_mtsMode { 0 };
  int          m_log2ParallelMergeLevel { 2 };
  bool         m_disableLFCrossTileBoundaryFlag { false };
  bool         m_disableLFCrossSliceBoundaryFlag { false };
  double       m_sourceScalingRatioHor { 1.0 };   // source scaling ratio Horizontal
  double       m_sourceScalingRatioVer { 1.0 };   // source scaling ratio Vertical
  bool         m_snrInternalColourSpace { false };
  bool m_outputInternalColourSpace { false };   // if true, then no colour space conversion is applied for reconstructed
                                                // video, otherwise inverse of input is applied.
  bool m_picPartitionFlag { false };
  int  m_iSourceHeightOrg { 0 };   // original source height in pixel (when interlaced = frame height)
  int  m_sourceWidthBeforeScale { 0 };   // source width in pixel before applying source scaling ratio Horizontal
  int  m_sourceHeightBeforeScale {
    0
  };   // source height in pixel before applying source scaling ratio Vertical (when interlaced = field height)
  bool                       m_packedYUVMode { false };
  bool                       m_clipInputVideoToRec709Range { false };
  bool                       m_clipOutputVideoToRec709Range { false };
  InputColourSpaceConversion m_inputColourSpaceConvert { IPCOLOURSPACE_UNCHANGED };
  int                        m_verbosity {
    VERBOSE
  };   // source height in pixel before applying source scaling ratio Vertical (when interlaced = field height)
  uint32_t m_bitDepthConstraint { 0 };
  int      m_qpIncrementAtSourceFrame {
    -1
  };   // Optional source frame number at which all subsequent frames are to use an increased internal QP.
  bool                  m_useIdentityTableForNon420Chroma { true };
  std::string           m_dQPFileName { "" };
  int                   m_rectSliceFixedWidth { 0 };
  int                   m_rectSliceFixedHeight { 0 };
  double                m_fractionOfFrames { 1.0 };
  std::vector<uint32_t> m_rectSlicePos {};   // rectangular slice positions (pairs of top-left CTU address followed by
                                             // bottom-right CTU address)
  ChromaFormat          m_chromaFormatConstraint { ChromaFormat::_420 };
  uint32_t              m_numTileCols { 0 };   // derived number of tile columns
  uint32_t              m_numTileRows { 0 };   // derived number of tile rows
#if EXTENSION_360_VIDEO
  int m_inputFileWidth { 0 };   // width of image in input file  (this is equivalent to sourceWidth,  if sourceWidth  is
                                // not subsequently altered due to padding)
  int m_inputFileHeight { 0 };   // height of image in input file (this is equivalent to sourceHeight, if sourceHeight
                                 // is not subsequently altered due to padding)
#endif

  // file I/O
  std::string m_inputFileName { "" };   // source file name
  std::string m_bitstreamFileName { "" };   // output bitstream file
  std::string m_reconFileName { "" };   // output reconstruction file

  int      m_frameRate { 0 };
  int      m_frameSkip { 0 };
  uint32_t m_temporalSubsampleRatio { 1 };
  int      m_sourceWidth { 0 };
  int      m_sourceHeight { 0 };
  Window   m_conformanceWindow {};
  int      m_sourcePadding[2] {};
  int      m_framesToBeEncoded { 0 };
  int      m_firstValidFrame { 0 };
  int      m_lastValidFrame { MAX_INT };

  double              m_adLambdaModifier[MAX_TLAYER] { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
  std::vector<double> m_adIntraLambdaModifier {};
  double m_dIntraQpFactor { -1.0 };   // Intra Q Factor. If negative, use a default equation: 0.57*(1.0 - Clip3( 0.0,
                                      // 0.5, 0.05*(double)(isField ? (GopSize-1)/2 : GopSize-1) ))
  double m_lambdaScaleTowardsNextQP { 0.0 };
  bool   m_printMSEBasedSequencePSNR { false };
  bool   m_printHexPsnr { false };
  bool   m_printFrameMSE { false };
  bool   m_printSequenceMSE { false };
  bool   m_printMSSSIM { false };
  bool   m_printWPSNR { false };
  bool   m_cabacZeroWordPaddingEnabled { true };

  bool         m_gciPresentFlag { false };
  bool         m_onePictureOnlyConstraintFlag { false };
  uint32_t     m_maxBitDepthConstraintIdc { 16 };
  ChromaFormat m_maxChromaFormatConstraintIdc { ChromaFormat::_444 };
  bool         m_allLayersIndependentConstraintFlag { false };
  bool         m_noMrlConstraintFlag { false };
  bool         m_noMipConstraintFlag { false };
  bool         m_noDirPlanar { false };
  bool         m_noLfnstConstraintFlag { false };
  bool         m_noMmvdConstraintFlag { false };
  bool         m_noSmvdConstraintFlag { false };
  bool         m_noProfConstraintFlag { false };
  bool         m_noPaletteConstraintFlag { false };
  bool         m_noActConstraintFlag { false };
  bool         m_noLmcsConstraintFlag { false };
  bool         m_noExplicitScaleListConstraintFlag { false };
  bool         m_noMttConstraintFlag { false };
  bool         m_noChromaQpOffsetConstraintFlag { false };
  bool         m_noQtbttDualTreeIntraConstraintFlag { false };
  int          m_maxLog2CtuSizeConstraintIdc { 8 };
  bool         m_noPartitionConstraintsOverrideConstraintFlag { false };
  bool         m_noSaoConstraintFlag { false };
  bool         m_noCCSaoConstraintFlag { false };
  bool         m_noAlfConstraintFlag { false };
  bool         m_noCCAlfConstraintFlag { false };
  bool         m_noWeightedPredictionConstraintFlag { false };
  bool         m_noRefWraparoundConstraintFlag { false };
  bool         m_noTemporalMvpConstraintFlag { false };
  bool         m_noSbtmvpConstraintFlag { false };
  bool         m_noAmvrConstraintFlag { false };
  bool         m_noBdofConstraintFlag { false };
  bool         m_noCclmConstraintFlag { false };
  bool         m_noCCCMConstraintFlag { false };
  bool         m_noMtsConstraintFlag { false };
  bool         m_noSbtConstraintFlag { false };
  bool         m_noAffineMotionConstraintFlag { false };
  bool         m_noBcwConstraintFlag { false };
  bool         m_noIbcConstraintFlag { false };
  bool         m_noCiipConstraintFlag { false };
  bool         m_noGeoConstraintFlag { false };
  bool         m_noSgpmConstraintFlag { false };
  bool         m_noObmcConstraintFlag { false };
  bool         m_noLadfConstraintFlag { false };
  bool         m_noTransformSkipConstraintFlag { false };
  bool         m_noLumaTransformSize64ConstraintFlag { false };
  bool         m_noBDPCMConstraintFlag { false };
  bool         m_noJointCbCrConstraintFlag { false };
  bool         m_noCuQpDeltaConstraintFlag { false };
  bool         m_noDepQuantConstraintFlag { false };
  bool         m_noSignDataHidingConstraintFlag { false };
  bool         m_noTrailConstraintFlag { false };
  bool         m_noStsaConstraintFlag { false };
  bool         m_noRaslConstraintFlag { false };
  bool         m_noRadlConstraintFlag { false };
  bool         m_noIdrConstraintFlag { false };
  bool         m_noCraConstraintFlag { false };
  bool         m_noGdrConstraintFlag { false };
  bool         m_noApsConstraintFlag { false };
  bool         m_allRapPicturesFlag { false };
  bool         m_noExtendedPrecisionProcessingConstraintFlag { false };
  bool         m_noTsResidualCodingRiceConstraintFlag { false };
  bool         m_noRrcRiceExtensionConstraintFlag { false };
  bool         m_noPersistentRiceAdaptationConstraintFlag { false };
  bool         m_noReverseLastSigCoeffConstraintFlag { false };

  // profil, level
  Profile::Name         m_profile { Profile::NONE };
  Level::Tier           m_tier { Level::MAIN };
  Level::Name           m_level { Level::NONE };
  bool                  m_frameOnlyConstraintFlag { true };
  bool                  m_multiLayerEnabledFlag { false };
  std::vector<uint32_t> m_subProfile {};
  uint8_t               m_numSubProfile { 0 };
  bool                  m_nonPackedConstraintFlag { false };
  bool                  m_nonProjectedConstraintFlag { false };
  bool                  m_noRprConstraintFlag { false };
  bool                  m_noResChangeInClvsConstraintFlag { false };
  bool                  m_oneTilePerPicConstraintFlag { false };
  bool                  m_picHeaderInSliceHeaderConstraintFlag { false };
  bool                  m_oneSlicePerPicConstraintFlag { false };
  bool                  m_noIdrRplConstraintFlag { false };
  bool                  m_noRectSliceConstraintFlag { false };
  bool                  m_oneSlicePerSubpicConstraintFlag { false };
  bool                  m_noSubpicInfoConstraintFlag { false };
  bool                  m_intraOnlyConstraintFlag { false };

  // coding structure
  int              m_intraPeriod { -1 };
  uint32_t         m_decodingRefreshType { 0 };   // the type of decoding refresh employed for the random access.
  bool             m_rewriteParamSets { false };
  bool             m_idrRefParamList { false };
  mutable GOPEntry m_GOPList[MAX_GOP] {};   // th fix this
  mutable RPLEntry m_RPLList0[MAX_GOP] {};   // th fix this
  mutable RPLEntry m_RPLList1[MAX_GOP] {};   // th fix this
  int              m_gopSize { 1 };
  int              m_numRPLList0 { 0 };
  int              m_numRPLList1 { 0 };
  int              m_maxDecPicBuffering[MAX_TLAYER] {};
  int              m_maxNumReorderPics[MAX_TLAYER] {};
  int              m_drapPeriod { 0 };
  int              m_edrapPeriod { 0 };

  int                        m_iQP { 32 };
  ChromaQpMappingTableParams m_chromaQpMappingTableParams {};
  int                        m_intraQPOffset { 0 };   // QP offset for intra slice (integer)
  bool                       m_lambdaFromQPEnable { false };   // enable lambda derivation from QP

  bool m_AccessUnitDelimiter { false };   // add Access Unit Delimiter NAL units
  bool m_enablePictureHeaderInSliceHeader { true };   // Enable Picture Header in Slice Header

  int                   m_maxTempLayer { 0 };   // Max temporal layer
  bool                  m_isLowDelay { false };
  unsigned              m_CTUSize { MAX_CU_SIZE };
  bool                  m_subPicInfoPresentFlag { false };
  uint32_t              m_numSubPics { 0 };
  bool                  m_subPicSameSizeFlag { false };
  std::vector<uint32_t> m_subPicCtuTopLeftX {};
  std::vector<uint32_t> m_subPicCtuTopLeftY {};
  std::vector<uint32_t> m_subPicWidth {};
  std::vector<uint32_t> m_subPicHeight {};
  std::vector<bool>     m_subPicTreatedAsPicFlag {};
  std::vector<bool>     m_loopFilterAcrossSubpicEnabledFlag {};
  bool                  m_subPicIdMappingExplicitlySignalledFlag { false };
  bool                  m_subPicIdMappingInSpsFlag { false };
  unsigned              m_subPicIdLen { 0 };
  std::vector<uint16_t> m_subPicId {};
  bool                  m_useSplitConsOverride { true };
  unsigned              m_minQt[3] { 8, 8, 4 };   // 0: I slice 1: P/B slice, 2: I slice chroma
  unsigned              m_maxBt[3] { 32, 128, 64 };   // 0: I slice 1: P/B slice, 2: I slice chroma
  unsigned              m_maxTt[3] { 32, 64, 32 };   // 0: I slice 1: P/B slice, 2: I slice chroma
  unsigned              m_uiMaxMTTHierarchyDepth { 3 };
  unsigned              m_uiMaxMTTHierarchyDepthI { 3 };
  unsigned              m_uiMaxMTTHierarchyDepthIChroma { 3 };
  int                   m_ttFastSkip { 31 };
  double                m_ttFastSkipThr { 1.075 };
  bool                  m_tempPartPredEnabled { false };   // enable partitioning prediction based on reference frames

  bool     m_dualITree { false };
  unsigned m_log2MinCUSize { MIN_CU_LOG2 };

  int      m_LMChroma { 1 };
  bool     m_CCCM { false };
  bool     m_MCBP { false };
  bool     m_TMBP { false };
  bool     m_horCollocatedChromaFlag { true };
  bool     m_verCollocatedChromaFlag { false };
  bool     m_explicitMtsIntra { false };
  bool     m_explicitMtsInter { false };
  bool     m_implicitMtsIntra { false };
  bool     m_SBT { false };   // Sub-Block Transform for inter blocks
  int      m_SBTFast64WidthTh { 0 };   // when to enable size-64 SBT in encoder RDO check
  bool     m_intraLFNSTISlice { false };
  bool     m_intraLFNSTPBSlice { false };
  bool     m_interLFNST { false };
  bool     m_interLFNSTSBT { false };
  bool     m_sbTmvpEnableFlag { false };
  int      m_Affine { 0 };
  bool     m_AffineType { true };
  bool     m_AffineMmvdMode { false };
  bool     m_affineParaRefinement { false };
  bool     m_affineSbMrgExt { false };
  bool     m_adaptBypassAffineMe { false };
  unsigned m_minAffineBlkSize { 16 };
  bool     m_useDMVD { false };
  bool     m_PROF { false };
  bool     m_BIO { false };
  bool     m_DMVDBIOExt { false };
  int      m_SMVD { 0 };
  bool     m_compositeRefEnabled { false };
  bool     m_bcw { false };
  bool     m_BcwFast { false };
  bool     m_ladfEnabled { false };
  int      m_ladfNumIntervals { 3 };
  int      m_ladfQpOffset[MAX_LADF_INTERVALS] {};
  int      m_ladfIntervalLowerBound[MAX_LADF_INTERVALS] {};
  bool     m_ciip { false };
  int      m_Geo { 0 };
  bool     m_sgpm { false };
  bool     m_sgpmNoBlend { false };
  bool     m_allowDisFracMMVD { false };
  bool     m_AffineAmvr { false };
  bool     m_useHashMeInCurrentIntraPeriod { false };
  bool     m_HashMECfgEnable { false };
  bool     m_AffineAmvrEncOpt { false };
  bool     m_AffineAmvp { false };
  int      m_obmc { 0 };
  bool     m_MMVD { false };
  int      m_MmvdDisNum { 8 };
  bool     m_rgbFormat { false };
  unsigned m_PLTMode { 0 };
  bool     m_jointCbCrMode { false };
  unsigned m_ibcMode { 0 };
  unsigned m_ibcFracMode { 1 };
  bool     m_ibcMerge { false };
  unsigned m_ibcLocalSearchRangeX { 128 };
  unsigned m_ibcLocalSearchRangeY { 128 };
  unsigned m_ibcHashSearch { 0 };
  unsigned m_ibcHashSearchMaxCand { 256 };
  unsigned m_ibcHashSearchRange4SmallBlk { 256 };
  unsigned m_ibcFastMethod { 14 };

  bool      m_wrapAround { false };
  unsigned  m_wrapAroundOffset { 0 };
  bool      m_lmcsEnabled { false };
  unsigned  m_reshapeSignalType { 0 };
  unsigned  m_intraCMD { 0 };
  ReshapeCW m_reshapeCW {};
  int       m_updateCtrl { 0 };
  int       m_adpOption { 0 };
  uint32_t  m_initialCW { 0 };
  int       m_CSoffset { 0 };
  bool      m_encDbOpt { false };
  bool      m_useFastLCTU { false };
  bool      m_useFastMrg { false };
  int       m_maxMergeRdCandNumTotal { 15 };
  int       m_mergeRdCandQuotaRegular { NUM_MRG_SATD_CAND };
  int       m_mergeRdCandQuotaRegularSmallBlk { NUM_MRG_SATD_CAND };
  int       m_mergeRdCandQuotaSubBlk { NUM_AFF_MRG_SATD_CAND };
  int       m_mergeRdCandQuotaCiip { 1 };
  int       m_mergeRdCandQuotaGpm { GEO_MAX_TRY_WEIGHTED_SATD };
  int       m_mergeRdCandQuotaNonGeoPreserved { 5 };
  bool      m_usePbIntraFast { false };
  bool      m_useAMaxBT { false };
  bool      m_e0023FastEnc { true };
  bool      m_useCostBasedMttSkipping { false };
  bool      m_contentBasedFastQtbt { false };
  bool      m_useNonLinearAlfLuma { true };
  bool      m_useNonLinearAlfChroma { true };
  unsigned  m_maxNumAlfAlternativesChroma { AlfParameters::ALF_MAX_NUM_ALTERNATIVES_CHROMA };
  bool      m_MRL { false };
  bool      m_MIP { false };
  bool      m_dirPlanar { false };
  bool      m_DIMD { false };
  bool      m_DIMDChroma { false };
  bool      m_TIMD { false };
  bool      m_TIMDSAD { false };
  bool      m_OFTIMD { true };
  bool      m_OBIC { false };
  bool      m_EIP { false };
  bool      m_MMEIP { false };
  int       m_useFastMIP { 0 };
  int       m_fastAdaptCostPredMode { 0 };
  bool      m_disableFastDecisionTT { false };
  bool      m_qtbttSpeedUp { false };
  uint32_t  m_log2MaxTbSize { 7 };
  int       m_interMTSMaxSize { 0 };
  bool      m_bvgCccm { true };
  bool      m_ccBoostFilter { true };
  bool      m_ccBoostTplRefSel { true };
  bool      m_ccMerge { true };
  bool      m_ccMergeFusion { true };
  bool      m_ccDecDerivedMode { true };

  // loop/deblock filter
  bool   m_deblockingFilterDisable { false };
  bool   m_deblockingFilterOffsetInPPS { true };
  int    m_deblockingFilterBetaOffsetDiv2 { 0 };
  int    m_deblockingFilterTcOffsetDiv2 { 0 };
  int    m_deblockingFilterCbBetaOffsetDiv2 { 0 };
  int    m_deblockingFilterCbTcOffsetDiv2 { 0 };
  int    m_deblockingFilterCrBetaOffsetDiv2 { 0 };
  int    m_deblockingFilterCrTcOffsetDiv2 { 0 };
  int    m_deblockingFilterMetric { 0 };
  bool   m_useSao { true };
  int    m_CCSAO { 0 };
  bool   m_saoTrueOrg { false };
  bool   m_bTestSAODisableAtPictureLevel { false };
  double m_saoEncodingRate { 0.75 };   // When non-0 SAO early picture termination is enabled for luma and chroma
  double m_saoEncodingRateChroma { 0.5 };   // The SAO early picture termination rate to use for chroma (when
                                            // m_SaoEncodingRate is >0). If <=0, use results for luma.
  int    m_maxNumOffsetsPerPic { 2048 };
  bool   m_saoCtuBoundary { false };
  bool   m_saoGreedyMergeEnc { false };

  // motion search
  bool           m_bDisableIntraPUsInInterSlices { false };
  MESearchMethod m_motionEstimationSearchMethod { MESearchMethod::DIAMOND };
  int            m_searchRange { 96 };
  int            m_bipredSearchRange { 4 };
  bool           m_bClipForBiPredMeEnabled { false };
  bool           m_bFastMEAssumingSmootherMVEnabled { true };
  int            m_minSearchWindow { 8 };
  bool           m_bRestrictMESampling { false };

  // dqp
  int      m_iMaxDeltaQP { 0 };   // Max. absolute delta QP (1:default)
  int      m_cuQpDeltaSubdiv { 0 };   // Max. subdivision level for a CuDQP (0:default)
  unsigned m_cuChromaQpOffsetSubdiv { 0 };   // Max. subdivision level for a chroma QP adjustment (0:default)
  bool     m_cuChromaQpOffsetEnabled { true };   // Local chroma QP offset enable flag
  std::vector<ChromaQpAdj> m_cuChromaQpOffsetList {};   // Local chroma QP offsets list (to be signalled in PPS)

  int m_chromaCbQpOffset { 0 };   // Chroma Cb QP Offset (0:default)
  int m_chromaCrQpOffset { 0 };   // Chroma Cr Qp Offset (0:default)
  int m_chromaCbQpOffsetDualTree { 0 };   // Chroma Cb QP Offset for dual tree
  int m_chromaCrQpOffsetDualTree { 0 };   // Chroma Cr Qp Offset for dual tree
  int m_chromaCbCrQpOffset { -1 };   // QP Offset for the joint Cb-Cr mode
  int m_chromaCbCrQpOffsetDualTree { 0 };   // QP Offset for the joint Cb-Cr mode in dual tree
#if ER_CHROMA_QP_WCG_PPS
  WCGChromaQPControl m_wcgChromaQpControl {};   // Wide-colour-gamut chroma QP control.
#endif
#if W0038_CQP_ADJ
  uint32_t m_sliceChromaQpOffsetPeriodicity {
    0
  };   // Used in conjunction with Slice Cb/Cr QpOffsetIntraOrPeriodic. Use 0 (default) to disable periodic nature.
  mutable int
    m_sliceChromaQpOffsetIntraOrPeriodic[2] {};   // th fix this           // Chroma Cb QP Offset at slice level for I
                                                  // slice or for periodic inter slices as defined by
                                                  // SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.
#endif

  ChromaFormat m_chromaFormatIdc { ChromaFormat::_420 };

  bool m_extendedPrecisionProcessingFlag { false };
  bool m_tsrcRicePresentFlag { false };
  bool m_reverseLastSigCoeffEnabledFlag { false };
  bool m_highPrecisionOffsetsEnabledFlag { false };
  bool m_bUseAdaptiveQP { false };
  int  m_iQPAdaptationRange { 6 };
#if ENABLE_QPA
  bool m_bUsePerceptQPA { false };
  bool m_bUseWPSNR { false };
#endif

  //  tools
  bool                m_bUseASR { false };
  bool                m_bUseHADME { true };
  bool                m_useRDOQ { true };
  bool                m_useRDOQTS { true };
  bool                m_useSelectiveRDOQ { false };
  FastInterSearchMode m_fastInterSearchMode { FASTINTERSEARCH_DISABLED };
  bool                m_bUseEarlyCU { false };
  bool                m_useFastDecisionForMerge { true };
  bool                m_useEarlySkipDetection { false };
  bool                m_reconBasedCrossCPredictionEstimate { false };
  bool                m_useTransformSkip { false };
  bool                m_useTransformSkipFast { false };
  bool                m_useChromaTS { false };
  bool                m_useBDPCM { false };
  uint32_t            m_log2MaxTransformSkipBlockSize { 5 };
  bool                m_transformSkipRotationEnabledFlag { false };
  bool                m_transformSkipContextEnabledFlag { false };
  bool                m_rrcRiceExtensionEnableFlag { false };
  bool                m_persistentRiceAdaptationEnabledFlag { false };
  bool                m_cabacBypassAlignmentEnabledFlag { false };
#if SHARP_LUMA_DELTA_QP
  LumaLevelToDeltaQPMapping m_lumaLevelToDeltaQPMapping {};   // mapping from luma level to delta QP.
#endif
  bool   m_smoothQPReductionEnable { false };
  int    m_smoothQPReductionPeriodicity { 0 };
  double m_smoothQPReductionThresholdIntra { 3.0 };
  double m_smoothQPReductionModelScaleIntra { -1.0 };
  double m_smoothQPReductionModelOffsetIntra { 27.0 };
  int    m_smoothQPReductionLimitIntra { -16 };
  double m_smoothQPReductionThresholdInter { 3.0 };
  double m_smoothQPReductionModelScaleInter { -1.0 };
  double m_smoothQPReductionModelOffsetInter { 27.0 };
  int    m_smoothQPReductionLimitInter { -4 };
  bool   m_bUseAdditionalCMVP { false };

  std::vector<int> m_frameDeltaQps {};

  uint32_t m_uiDeltaQpRD { 0 };
  bool     m_bFastDeltaQP { false };

  bool m_bFastUDIUseMPMEnabled { true };
  bool m_bFastMEForGenBLowDelayEnabled { true };
  bool m_gopBasedTemporalFilterEnabled { false };
  int  m_gopBasedTemporalFilterPastRefs { TF_DEFAULT_REFS };
  int  m_gopBasedTemporalFilterFutureRefs { TF_DEFAULT_REFS };
  std::map<int, double>
      m_gopBasedTemporalFilterStrengths {};   // Filter strength per frame for the GOP-based Temporal Filter
  int m_gopBasedTemporalFilterUnitSize { 16 };
  int m_bimEnabled {
    0
  };   // Block Importance Mapping, 0=Off, 1=On with QP and lambda adaptation, 2=On with lambda adaptaion only
  int m_bimUnitSize { 32 };

  bool                  m_mixedLossyLossless { false };   // enable mixed lossy/lossless coding
  std::vector<uint16_t> m_sliceLosslessArray {};   // Slice lossless array
  std::vector<uint32_t> m_tileColumnWidth {};   // tile column widths in units of CTUs (last column width will be
                                                // repeated uniformly to cover any remaining picture width)
  std::vector<uint32_t> m_tileRowHeight {};   // tile row heights in units of CTUs (last row height will be repeated
                                              // uniformly to cover any remaining picture height)
  uint32_t              m_numSlicesInPic {
    0
  };   // number of rectangular slices in the picture (raster-scan slice specified at slice level)
  bool                   m_tileIdxDeltaPresentFlag { false };   // rectangular slice tile index delta present flag
  std::vector<RectSlice> m_rectSlices {};   // list of rectanglar slice syntax parameters
  std::vector<uint32_t>  m_rasterSliceSize {};   // raster-scan slice sizes in units of tiles

  // sub-picture, slices
  bool m_singleSlicePerSubPicFlag { false };
  bool m_entropyCodingSyncEnabledFlag { false };
  bool m_entryPointPresentFlag { true };   // flag for the presence of entry points

  bool m_constrainedRaslEncoding { false };

  // weighted prediction
  bool                     m_useWeightedPred { false };   // Use of Weighting Prediction (P_SLICE)
  bool                     m_useWeightedBiPred { false };   // Use of Bi-directional Weighting Prediction (B_SLICE)
  WeightedPredictionMethod m_weightedPredictionMethod { WP_PER_PICTURE_WITH_SIMPLE_DC_COMBINED_COMPONENT };
  uint32_t                 m_maxNumMergeCand { 10 };   // Maximum number of merge candidates
  uint32_t                 m_maxNumAffineMergeCand { 15 };   // Maximum number of affine merge candidates
  uint32_t                 m_maxNumGeoCand { 5 };
  uint32_t                 m_maxNumIBCMergeCand { 6 };   // Max number of IBC merge candidates
  uint32_t                 m_maxNumBMMergeCand { 4 };   ///< Max number of BM merge candidates
  bool                     m_mergeOppositeLic { false };
  uint32_t                 m_maxNumOppositeLicMergeCand { REG_MRG_MAX_NUM_CANDS_OPPOSITELIC };
  uint32_t                 m_maxNumAffineOppositeLicMergeCand { AFF_MRG_MAX_NUM_CANDS_OPPOSITELIC };
  ScalingListMode m_useScalingListId { SCALING_LIST_OFF };   // Using quantization matrix i.e. 0=off, 1=default, 2=file.
  std::string     m_scalingListFileName { "" };   // quantization matrix file name
  bool            m_disableScalingMatrixForAlternativeColourSpace { false };
  bool            m_scalingMatrixDesignatedColourSpace { true };
  bool m_sliceLevelRpl { true };   // code reference picture lists in slice headers rather than picture header
  bool m_sliceLevelDblk { true };   // code deblocking filter parameters in slice headers rather than picture header
  bool m_sliceLevelSao { true };   // code SAO parameters in slice headers rather than picture header
  bool m_sliceLevelAlf { true };   // code ALF parameters in slice headers rather than picture header
  bool m_sliceLevelWp { true };   // code weighted prediction parameters in slice headers rather than picture header
  bool m_sliceLevelDeltaQp { true };   // code delta in slice headers rather than picture header
  bool m_disableScalingMatrixForLfnstBlks { true };
  int  m_TMVPModeId { 1 };
  int  m_DepQuantEnabledIdc { 1 };
  bool m_SignDataHidingEnabledFlag { false };
  int  m_numPredSign { SIGN_PRED_MAX_NUM };
  int  m_log2SignPredArea { -1 };   // "-1" means it is set based on resolution
  bool m_pdp { true };
  bool m_tempCabacInitMode { true };

  // rate conttrol
  bool     m_RCEnableRateControl { false };
  int      m_RCTargetBitrate { 0 };
  int      m_RCKeepHierarchicalBit { 0 };
  bool     m_RCLCULevelRC { true };
  bool     m_RCUseLCUSeparateModel { true };
  int      m_RCInitialQP { 0 };
  bool     m_RCForceIntraQP { false };
  bool     m_RCCpbSaturationEnabled { false };
  uint32_t m_RCCpbSize { 0 };
  double   m_RCInitialCpbFullness { 0.9 };
  CostMode m_costMode {
    COST_STANDARD_LOSSY
  };   // The cost function to use, primarily when considering lossless coding.
  bool m_TSRCdisableLL { true };   // Disable TSRC for lossless

  OPI  m_opi {};
  bool m_OPIEnabled { false };   // enable Operating Point Information (OPI)
  bool m_rplOfDepLayerInSh { false };
  int  m_opiMaxTemporalLayer { 500 };

  DCI  m_dci {};
  bool m_DCIEnabled { false };   // enable Decoding Capability Information (DCI)

  bool m_recalculateQPAccordingToLambda { false };   // recalculate QP value according to the lambda value
  bool m_hrdParametersPresentFlag { false };   // enable generation of HRD parameters
  bool m_vuiParametersPresentFlag { false };   // enable generation of VUI parameters
  bool m_samePicTimingInAllOLS { true };   // same picture timing SEI message is used in all OLS
  bool m_aspectRatioInfoPresentFlag { false };   // Signals whether aspect_ratio_idc is present
  int  m_aspectRatioIdc { 0 };   // aspect_ratio_idc
  int  m_sarWidth { 0 };   // horizontal size of the sample aspect ratio
  int  m_sarHeight { 0 };   // vertical size of the sample aspect ratio
  bool m_colourDescriptionPresentFlag {
    false
  };   // Signals whether colour_primaries, transfer_characteristics and matrix_coefficients are present
  int m_colourPrimaries { 2 };   // Indicates chromaticity coordinates of the source primaries
  int m_transferCharacteristics { 2 };   // Indicates the opto-electronic transfer characteristics of the source
  int m_matrixCoefficients {
    2
  };   // Describes the matrix coefficients used in deriving luma and chroma from RGB primaries
  bool m_progressiveSourceFlag { false };   // Indicates if the content is progressive
  bool m_interlacedSourceFlag { false };   // Indicates if the content is interlaced
  bool m_chromaLocInfoPresentFlag {
    false
  };   // Signals whether chroma_sample_loc_type_top_field and chroma_sample_loc_type_bottom_field are present
  int  m_chromaSampleLocTypeTopField { 0 };   // Specifies the location of chroma samples for top field
  int  m_chromaSampleLocTypeBottomField { 0 };   // Specifies the location of chroma samples for bottom field
  int  m_chromaSampleLocType { 0 };   // Specifies the location of chroma samples for progressive content
  bool m_overscanInfoPresentFlag { false };   // Signals whether overscan_appropriate_flag is present
  bool m_overscanAppropriateFlag {
    false
  };   // Indicates whether conformant decoded pictures are suitable for display using overscan
  bool m_videoFullRangeFlag { false };   // Indicates the black level and range of luma and chroma signals
  bool m_fieldSeqFlag { false };
  bool m_isTopFieldFirst { false };
  bool m_efficientFieldIRAPEnabled {
    true
  };   // enable to code fields in a specific, potentially more efficient, order.
  bool m_harmonizeGopFirstFieldCoupleEnabled { true };

  std::string m_summaryOutFilename { "" };   // filename to use for producing summary output file.
  std::string m_summaryPicFilenameBase { "" };   // Base filename to use for producing summary picture output files. The
                                                 // actual filenames used will have I.txt, P.txt and B.txt appended.
  uint32_t    m_summaryVerboseness { 0 };   // Specifies the level of the verboseness of the text output.
  int         m_ImvMode { 1 };
  int         m_Imv4PelFast { 1 };
  std::string m_decodeBitstreams[2] {};   // filename for decode bitstreams.
  bool        m_forceDecodeBitstream1 { false };   // guess what it means
  int         m_switchPOC { -1 };   // dbg poc.
  int         m_switchDQP { 0 };   // dqp applied to  switchPOC and subsequent pictures.
  int         m_fastForwardToPOC { -1 };
  bool        m_stopAfterFFtoPOC { false };
  int  m_debugCTU { -2 };   // -1 off using build-in history based ME support, -2 off disable build in history based ME
                           // support - to reuse bitstreams with DebugCTU
  bool m_bs2ModPOCAndType { false };

  std::vector<std::vector<uint32_t>> m_maxTidILRefPicsPlus1 {};

  int    m_maxNumAlfAps { AlfParameters::ALF_CTB_MAX_NUM_APS };
  int    m_alfapsIDShift { 0 };
  bool   m_constantJointCbCrSignFlag { false };
  bool   m_alf { true };
  bool   m_lfCccm { true };
  bool   m_alfImprovements { true };
  bool   m_alfTrueOrg { true };
  double m_alfStrengthLuma { 1.0 };
  bool   m_alfAllowPredefinedFilters { true };
  double m_ccalfStrength { 1.0 };
  double m_alfStrengthChroma { 1.0 };
  double m_alfStrengthTargetLuma { 1.0 };
  double m_alfStrengthTargetChroma { 1.0 };
  double m_ccalfStrengthTarget { 1.0 };
  bool   m_ccalf { true };
  int    m_ccalfQpThreshold { 37 };
#if JVET_O0756_CONFIG_HDRMETRICS || JVET_O0756_CALCULATE_HDRMETRICS
  double m_whitePointDeltaE[3] { 100.0, 1000.0, 5000.0 };
  int    m_sampleRange { 0 };
  int    m_colorPrimaries { 1 };
  int    m_chromaLocation[2] { 2, 2 };
  double m_maxSampleValue { 10000.0 };
  bool   m_enableTFunctionLUT { false };
  int    m_chromaUPFilter { 1 };
  int    m_cropOffsetLeft { 0 };
  int    m_cropOffsetTop { 0 };
  int    m_cropOffsetRight { 0 };
  int    m_cropOffsetBottom { 0 };
  bool   m_calculateHdrMetrics { false };
#endif
  double m_scalingRatioHor { 1.0 };
  double m_scalingRatioVer { 1.0 };
  bool   m_gopBasedRPREnabledFlag { false };
  int    m_gopBasedRPRQPThreshold { 32 };
  double m_scalingRatioHor2 { 1.5 };
  double m_scalingRatioVer2 { 1.5 };
  double m_scalingRatioHor3 { 1.25 };
  double m_scalingRatioVer3 { 1.25 };
  double m_psnrThresholdRPR { 47.0 };
  double m_psnrThresholdRPR2 { 44.0 };
  double m_psnrThresholdRPR3 { 41.0 };
  int    m_qpOffsetRPR { -6 };
  int    m_qpOffsetRPR2 { -4 };
  int    m_qpOffsetRPR3 { -2 };
  int    m_qpOffsetChromaRPR { -6 };
  int    m_qpOffsetChromaRPR2 { -4 };
  int    m_qpOffsetChromaRPR3 { -2 };
  double m_psnrChromaOffsetRPR { 7.0 };
  int    m_rprSwitchingResolutionOrderList[MAX_RPR_SWITCHING_ORDER_LIST_SIZE] {};
  int    m_rprSwitchingQPOffsetOrderList[MAX_RPR_SWITCHING_ORDER_LIST_SIZE] {};
  int    m_rprSwitchingListSize { 0 };
  bool   m_rprFunctionalityTestingEnabledFlag { false };
  bool   m_rprPopulatePPSatIntraFlag { false };
  int    m_rprSwitchingSegmentSize { 32 };
  double m_rprSwitchingTime { 0.0 };
  bool   m_rprEnabledFlag { true };
  bool   m_resChangeInClvsEnabled { false };
  int    m_switchPocPeriod { 0 };
  int    m_upscaledOutput { 0 };
  int    m_upscaleFilterForDisplay { 1 };
  int    m_numRefLayers[MAX_VPS_LAYERS] {};
  bool   m_avoidIntraInDepLayer { true };
  bool   m_craAPSreset { false };
  bool   m_rprRASLtoolSwitch { false };
  bool   m_refLayerMetricsEnabled { false };
  bool   m_BIF { true };
  int    m_BIFStrength { 1u };
  int    m_BIFQPOffset { 0 };
  bool   m_chromaBIF { true };
  int    m_chromaBIFStrength { 1u };
  int    m_chromaBIFQPOffset { 0 };

  int  m_licMode { 0 };
  bool m_fastPicLevelLIC { false };
  int  m_fastLIC { 0 };
  bool m_fastLICAffine { false };
  int  m_fastLICMode { 2 };
#if ENABLE_NNLF
  int         m_nnlf { 0 };
  std::string m_nnlfModelName { "" };   // NNLF model file name
  int m_nnlfPocDivisibleByN { 1 };   // Encoder can only turn NNLF filter on if POC is divisible by this number. Set to
                                     // 2 to prevent NNLF on odd frames.
  int m_nnlfDebugOption {
    0
  };   // NNLF debug option: 0: default, 1: apply only on I slice, 2: apply on all slices using I type as input
  int m_nnlfStartPoc { 0 };
#endif
  int m_interRPL { 0 };
};

//! \}
