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

struct ConstraintInfo
{
  bool         m_gciPresentFlag { false };
  bool         m_noRprConstraintFlag { false };
  bool         m_noResChangeInClvsConstraintFlag { false };
  bool         m_oneTilePerPicConstraintFlag { false };
  bool         m_picHeaderInSliceHeaderConstraintFlag { false };
  bool         m_oneSlicePerPicConstraintFlag { false };
  bool         m_noIdrRplConstraintFlag { false };
  bool         m_noRectSliceConstraintFlag { false };
  bool         m_oneSlicePerSubpicConstraintFlag { false };
  bool         m_noSubpicInfoConstraintFlag { false };
  bool         m_intraOnlyConstraintFlag { false };
  uint32_t     m_maxBitDepthConstraintIdc { 16 };
  ChromaFormat m_maxChromaFormatConstraintIdc { ChromaFormat::_444 };
  bool         m_onePictureOnlyConstraintFlag { false };
  bool         m_allLayersIndependentConstraintFlag { false };
  bool         m_noMrlConstraintFlag { false };
  bool         m_noMipConstraintFlag { false };
  bool         m_noLfnstConstraintFlag { false };
  bool         m_noMmvdConstraintFlag { false };
  bool         m_noSmvdConstraintFlag { false };
  bool         m_noProfConstraintFlag { false };
  bool         m_noPaletteConstraintFlag { false };
  bool         m_noActConstraintFlag { false };
  bool         m_noLmcsConstraintFlag { false };

  bool m_noExplicitScaleListConstraintFlag { false };
  bool m_noMttConstraintFlag { false };
  bool m_noChromaQpOffsetConstraintFlag { false };
  bool m_noQtbttDualTreeIntraConstraintFlag { false };
  int  m_maxLog2CtuSizeConstraintIdc { 8 };
  bool m_noPartitionConstraintsOverrideConstraintFlag { false };
  bool m_noSaoConstraintFlag { false };
  bool m_noAlfConstraintFlag { false };
  bool m_noCCAlfConstraintFlag { false };
  bool m_noWeightedPredictionConstraintFlag { false };
  bool m_noRefWraparoundConstraintFlag { false };
  bool m_noTemporalMvpConstraintFlag { false };
  bool m_noSbtmvpConstraintFlag { false };
  bool m_noAmvrConstraintFlag { false };
  bool m_noBdofConstraintFlag { false };
  bool m_noDmvrConstraintFlag { false };
  bool m_noCclmConstraintFlag { false };
  bool m_noSgpmConstraintFlag { false };
  bool m_noMtsConstraintFlag { false };
  bool m_noSbtConstraintFlag { false };
  bool m_noAffineMotionConstraintFlag { false };
  bool m_noBcwConstraintFlag { false };
  bool m_noIbcConstraintFlag { false };
  bool m_noCiipConstraintFlag { false };
  bool m_noGeoConstraintFlag { false };
  bool m_noObmcConstraintFlag { false };
  bool m_noLadfConstraintFlag { false };
  bool m_noTransformSkipConstraintFlag { false };
  bool m_noLumaTransformSize64ConstraintFlag { false };
  bool m_noBDPCMConstraintFlag { false };
  bool m_noJointCbCrConstraintFlag { false };
  bool m_noCuQpDeltaConstraintFlag { false };
  bool m_noDepQuantConstraintFlag { false };
  bool m_noSignDataHidingConstraintFlag { false };
  bool m_noMixedNaluTypesInPicConstraintFlag { false };
  bool m_noTrailConstraintFlag { false };
  bool m_noStsaConstraintFlag { false };
  bool m_noRaslConstraintFlag { false };
  bool m_noRadlConstraintFlag { false };
  bool m_noIdrConstraintFlag { false };
  bool m_noCraConstraintFlag { false };
  bool m_noGdrConstraintFlag { false };
  bool m_noApsConstraintFlag { false };
  bool m_allRapPicturesFlag { false };
  bool m_noExtendedPrecisionProcessingConstraintFlag { false };
  bool m_noTsResidualCodingRiceConstraintFlag { false };
  bool m_noRrcRiceExtensionConstraintFlag { false };
  bool m_noPersistentRiceAdaptationConstraintFlag { false };
  bool m_noReverseLastSigCoeffConstraintFlag { false };

  bool m_noCCSaoConstraintFlag { false };

//  bool          getGciPresentFlag() const { return m_gciPresentFlag; }
//  void          setGciPresentFlag(bool b) { m_gciPresentFlag = b; }
//
//  uint32_t      getMaxBitDepthConstraintIdc() const { return m_maxBitDepthConstraintIdc; }
//  void          setMaxBitDepthConstraintIdc(uint32_t bitDepth) { m_maxBitDepthConstraintIdc = bitDepth; }
//
//  ChromaFormat getMaxChromaFormatConstraintIdc() const { return m_maxChromaFormatConstraintIdc; }
//  void         setMaxChromaFormatConstraintIdc(ChromaFormat fmt) { m_maxChromaFormatConstraintIdc = fmt; }
//
//  bool          getNoRprConstraintFlag() const { return m_noRprConstraintFlag; }
//  void          setNoRprConstraintFlag(bool b) { m_noRprConstraintFlag = b; }
//
//  bool          getNoResChangeInClvsConstraintFlag() const { return m_noResChangeInClvsConstraintFlag; }
//  void          setNoResChangeInClvsConstraintFlag(bool b) { m_noResChangeInClvsConstraintFlag = b; }
//
//  bool          getOneTilePerPicConstraintFlag() const { return m_oneTilePerPicConstraintFlag; }
//  void          setOneTilePerPicConstraintFlag(bool b) { m_oneTilePerPicConstraintFlag = b; }
//
//  bool          getPicHeaderInSliceHeaderConstraintFlag() const { return m_picHeaderInSliceHeaderConstraintFlag; }
//  void          setPicHeaderInSliceHeaderConstraintFlag(bool b) { m_picHeaderInSliceHeaderConstraintFlag = b; }
//
//  bool          getOneSlicePerPicConstraintFlag() const { return m_oneSlicePerPicConstraintFlag; }
//  void          setOneSlicePerPicConstraintFlag(bool b) { m_oneSlicePerPicConstraintFlag = b; }
//
//  bool          getNoIdrRplConstraintFlag() const          { return m_noIdrRplConstraintFlag; }
//  void          setNoIdrRplConstraintFlag(bool b)          { m_noIdrRplConstraintFlag = b; }
//
//  bool          getNoRectSliceConstraintFlag() const       { return m_noRectSliceConstraintFlag; }
//  void          setNoRectSliceConstraintFlag(bool b)       { m_noRectSliceConstraintFlag = b; }
//
//  bool          getOneSlicePerSubpicConstraintFlag() const { return m_oneSlicePerSubpicConstraintFlag; }
//  void          setOneSlicePerSubpicConstraintFlag(bool b) { m_oneSlicePerSubpicConstraintFlag = b; }
//
//  bool          getNoSubpicInfoConstraintFlag() const      { return m_noSubpicInfoConstraintFlag; }
//  void          setNoSubpicInfoConstraintFlag(bool b)      { m_noSubpicInfoConstraintFlag = b; }
//
//  bool          getIntraOnlyConstraintFlag() const { return m_intraOnlyConstraintFlag; }
//  void          setIntraOnlyConstraintFlag(bool b) { m_intraOnlyConstraintFlag = b; }
//
//  bool          getOnePictureOnlyConstraintFlag() const { return m_onePictureOnlyConstraintFlag; }
//  void          setOnePictureOnlyConstraintFlag(bool b) { m_onePictureOnlyConstraintFlag = b; }
//
//  bool          getAllLayersIndependentConstraintFlag() const { return m_allLayersIndependentConstraintFlag; }
//  void          setAllLayersIndependentConstraintFlag(bool b) { m_allLayersIndependentConstraintFlag = b; }
//  bool          getNoMrlConstraintFlag() const { return m_noMrlConstraintFlag; }
//  void          setNoMrlConstraintFlag(bool b) { m_noMrlConstraintFlag = b; }
//  bool          getNoMipConstraintFlag() const { return m_noMipConstraintFlag; }
//  void          setNoMipConstraintFlag(bool b) { m_noMipConstraintFlag = b; }
//  bool          getNoLfnstConstraintFlag() const { return m_noLfnstConstraintFlag; }
//  void          setNoLfnstConstraintFlag(bool b) { m_noLfnstConstraintFlag = b; }
//  bool          getNoMmvdConstraintFlag() const { return m_noMmvdConstraintFlag; }
//  void          setNoMmvdConstraintFlag(bool b) { m_noMmvdConstraintFlag = b; }
//  bool          getNoSmvdConstraintFlag() const { return m_noSmvdConstraintFlag; }
//  void          setNoSmvdConstraintFlag(bool b) { m_noSmvdConstraintFlag = b; }
//  bool          getNoProfConstraintFlag() const { return m_noProfConstraintFlag; }
//  void          setNoProfConstraintFlag(bool b) { m_noProfConstraintFlag = b; }
//  bool          getNoPaletteConstraintFlag() const { return m_noPaletteConstraintFlag; }
//  void          setNoPaletteConstraintFlag(bool b) { m_noPaletteConstraintFlag = b; }
//  bool          getNoActConstraintFlag() const { return m_noActConstraintFlag; }
//  void          setNoActConstraintFlag(bool b) { m_noActConstraintFlag = b; }
//  bool          getNoLmcsConstraintFlag() const { return m_noLmcsConstraintFlag; }
//  void          setNoLmcsConstraintFlag(bool b) { m_noLmcsConstraintFlag = b; }
//  bool          getNoExplicitScaleListConstraintFlag() const { return m_noExplicitScaleListConstraintFlag; }
//  void          setNoExplicitScaleListConstraintFlag(bool b) { m_noExplicitScaleListConstraintFlag = b; }
//  bool          getNoMttConstraintFlag() const { return m_noMttConstraintFlag; }
//  void          setNoMttConstraintFlag(bool bVal) { m_noMttConstraintFlag = bVal; }
//  bool          getNoChromaQpOffsetConstraintFlag() const { return m_noChromaQpOffsetConstraintFlag; }
//  void          setNoChromaQpOffsetConstraintFlag(bool b) { m_noChromaQpOffsetConstraintFlag = b; }
//  bool          getNoQtbttDualTreeIntraConstraintFlag() const { return m_noQtbttDualTreeIntraConstraintFlag; }
//  void          setNoQtbttDualTreeIntraConstraintFlag(bool bVal) { m_noQtbttDualTreeIntraConstraintFlag = bVal; }
//  int           getMaxLog2CtuSizeConstraintIdc() const { return m_maxLog2CtuSizeConstraintIdc; }
//  void          setMaxLog2CtuSizeConstraintIdc(int idc) { m_maxLog2CtuSizeConstraintIdc = idc; }
//  bool          getNoPartitionConstraintsOverrideConstraintFlag() const { return
  //  m_noPartitionConstraintsOverrideConstraintFlag; } void setNoPartitionConstraintsOverrideConstraintFlag(bool bVal)
  //  { m_noPartitionConstraintsOverrideConstraintFlag = bVal; } bool          getNoSaoConstraintFlag() const { return
  //  m_noSaoConstraintFlag; } void          setNoSaoConstraintFlag(bool bVal) { m_noSaoConstraintFlag = bVal; } bool
  //  getNoAlfConstraintFlag() const { return m_noAlfConstraintFlag; } void          setNoAlfConstraintFlag(bool bVal) {
  //  m_noAlfConstraintFlag = bVal; } bool          getNoCCAlfConstraintFlag() const { return m_noCCAlfConstraintFlag; }
  //  void          setNoCCAlfConstraintFlag(bool val) { m_noCCAlfConstraintFlag = val; }
  //  bool          getNoJointCbCrConstraintFlag() const { return m_noJointCbCrConstraintFlag; }
  //  void          setNoJointCbCrConstraintFlag(bool bVal) { m_noJointCbCrConstraintFlag = bVal; }
  //  bool          getNoWeightedPredictionConstraintFlag() const { return m_noWeightedPredictionConstraintFlag; }
  //  void          setNoWeightedPredictionConstraintFlag(bool bVal) { m_noWeightedPredictionConstraintFlag = bVal; }
  //  bool          getNoRefWraparoundConstraintFlag() const { return m_noRefWraparoundConstraintFlag; }
  //  void          setNoRefWraparoundConstraintFlag(bool bVal) { m_noRefWraparoundConstraintFlag = bVal; }
  //  bool          getNoTemporalMvpConstraintFlag() const { return m_noTemporalMvpConstraintFlag; }
  //  void          setNoTemporalMvpConstraintFlag(bool bVal) { m_noTemporalMvpConstraintFlag = bVal; }
  //  bool          getNoSbtmvpConstraintFlag() const { return m_noSbtmvpConstraintFlag; }
  //  void          setNoSbtmvpConstraintFlag(bool bVal) { m_noSbtmvpConstraintFlag = bVal; }
  //  bool          getNoAmvrConstraintFlag() const { return m_noAmvrConstraintFlag; }
  //  void          setNoAmvrConstraintFlag(bool bVal) { m_noAmvrConstraintFlag = bVal; }
  //  bool          getNoBdofConstraintFlag() const { return m_noBdofConstraintFlag; }
  //  void          setNoBdofConstraintFlag(bool bVal) { m_noBdofConstraintFlag = bVal; }
  //  bool          getNoDmvrConstraintFlag() const { return m_noDmvrConstraintFlag; }
  //  void          setNoDmvrConstraintFlag(bool bVal) { m_noDmvrConstraintFlag = bVal; }
  //  bool          getNoCclmConstraintFlag() const { return m_noCclmConstraintFlag; }
  //  void          setNoCclmConstraintFlag(bool bVal) { m_noCclmConstraintFlag = bVal; }
  //  bool          getNoMtsConstraintFlag() const { return m_noMtsConstraintFlag; }
  //  void          setNoMtsConstraintFlag(bool bVal) { m_noMtsConstraintFlag = bVal; }
  //  bool          getNoSbtConstraintFlag() const { return m_noSbtConstraintFlag; }
  //  void          setNoSbtConstraintFlag(bool bVal) { m_noSbtConstraintFlag = bVal; }
  //  bool          getNoAffineMotionConstraintFlag() const { return m_noAffineMotionConstraintFlag; }
  //  void          setNoAffineMotionConstraintFlag(bool bVal) { m_noAffineMotionConstraintFlag = bVal; }
  //  bool          getNoBcwConstraintFlag() const { return m_noBcwConstraintFlag; }
  //  void          setNoBcwConstraintFlag(bool bVal) { m_noBcwConstraintFlag = bVal; }
  //  bool          getNoIbcConstraintFlag() const { return m_noIbcConstraintFlag; }
  //  void          setNoIbcConstraintFlag(bool bVal) { m_noIbcConstraintFlag = bVal; }
  //  bool          getNoCiipConstraintFlag() const { return m_noCiipConstraintFlag; }
  //  void          setNoCiipConstraintFlag(bool bVal) { m_noCiipConstraintFlag = bVal; }
  //  bool          getNoGeoConstraintFlag() const { return m_noGeoConstraintFlag; }
  //  void          setNoGeoConstraintFlag(bool bVal) { m_noGeoConstraintFlag = bVal; }
  //  bool          getNoLadfConstraintFlag() const { return m_noLadfConstraintFlag; }
  //  void          setNoLadfConstraintFlag(bool bVal) { m_noLadfConstraintFlag = bVal; }
  //  bool          getNoTransformSkipConstraintFlag() const { return m_noTransformSkipConstraintFlag; }
  //  void          setNoTransformSkipConstraintFlag(bool bVal) { m_noTransformSkipConstraintFlag = bVal; }
  //  bool          getNoLumaTransformSize64ConstraintFlag() const { return m_noLumaTransformSize64ConstraintFlag; }
  //  void          setNoLumaTransformSize64ConstraintFlag(bool bVal) { m_noLumaTransformSize64ConstraintFlag = bVal; }
  //  bool          getNoBDPCMConstraintFlag() const { return m_noBDPCMConstraintFlag; }
  //  void          setNoBDPCMConstraintFlag(bool bVal) { m_noBDPCMConstraintFlag = bVal; }
  //  bool          getNoCuQpDeltaConstraintFlag() const { return m_noCuQpDeltaConstraintFlag; }
  //  void          setNoCuQpDeltaConstraintFlag(bool bVal) { m_noCuQpDeltaConstraintFlag = bVal; }
  //  bool          getNoDepQuantConstraintFlag() const { return m_noDepQuantConstraintFlag; }
  //  void          setNoDepQuantConstraintFlag(bool bVal) { m_noDepQuantConstraintFlag = bVal; }
  //  bool          getNoSignDataHidingConstraintFlag() const { return m_noSignDataHidingConstraintFlag; }
  //  void          setNoSignDataHidingConstraintFlag(bool bVal) { m_noSignDataHidingConstraintFlag = bVal; }
  //  bool          getNoMixedNaluTypesInPicConstraintFlag() const    { return m_noMixedNaluTypesInPicConstraintFlag; }
  //  void          setNoMixedNaluTypesInPicConstraintFlag(bool bVal) { m_noMixedNaluTypesInPicConstraintFlag = bVal; }
  //  bool          getNoTrailConstraintFlag() const { return m_noTrailConstraintFlag; }
  //  void          setNoTrailConstraintFlag(bool bVal) { m_noTrailConstraintFlag = bVal; }
  //  bool          getNoStsaConstraintFlag() const { return m_noStsaConstraintFlag; }
  //  void          setNoStsaConstraintFlag(bool bVal) { m_noStsaConstraintFlag = bVal; }
  //  bool          getNoRaslConstraintFlag() const { return m_noRaslConstraintFlag; }
  //  void          setNoRaslConstraintFlag(bool bVal) { m_noRaslConstraintFlag = bVal; }
  //  bool          getNoRadlConstraintFlag() const { return m_noRadlConstraintFlag; }
  //  void          setNoRadlConstraintFlag(bool bVal) { m_noRadlConstraintFlag = bVal; }
  //  bool          getNoIdrConstraintFlag() const { return m_noIdrConstraintFlag; }
  //  void          setNoIdrConstraintFlag(bool bVal) { m_noIdrConstraintFlag = bVal; }
  //  bool          getNoCraConstraintFlag() const { return m_noCraConstraintFlag; }
  //  void          setNoCraConstraintFlag(bool bVal) { m_noCraConstraintFlag = bVal; }
  //  bool          getNoGdrConstraintFlag() const { return m_noGdrConstraintFlag; }
  //  void          setNoGdrConstraintFlag(bool bVal) { m_noGdrConstraintFlag = bVal; }
  //  bool          getNoApsConstraintFlag() const { return m_noApsConstraintFlag; }
  //  void          setNoApsConstraintFlag(bool bVal) { m_noApsConstraintFlag = bVal; }
  //  bool          getAllRapPicturesFlag() const { return m_allRapPicturesFlag; }
  //  void          setAllRapPicturesFlag(bool bVal) { m_allRapPicturesFlag = bVal; }
  //  bool          getNoExtendedPrecisionProcessingConstraintFlag() const { return
  //  m_noExtendedPrecisionProcessingConstraintFlag; } void          setNoExtendedPrecisionProcessingConstraintFlag(bool
  //  val) { m_noExtendedPrecisionProcessingConstraintFlag = val; } bool getNoTsResidualCodingRiceConstraintFlag() const
  //  { return m_noTsResidualCodingRiceConstraintFlag; } void          setNoTsResidualCodingRiceConstraintFlag(bool val)
  //  { m_noTsResidualCodingRiceConstraintFlag = val; } bool          getNoRrcRiceExtensionConstraintFlag() const {
  //  return m_noRrcRiceExtensionConstraintFlag; } void          setNoRrcRiceExtensionConstraintFlag(bool val) {
  //  m_noRrcRiceExtensionConstraintFlag = val; } bool          getNoPersistentRiceAdaptationConstraintFlag() const {
  //  return m_noPersistentRiceAdaptationConstraintFlag; } void setNoPersistentRiceAdaptationConstraintFlag(bool val) {
  //  m_noPersistentRiceAdaptationConstraintFlag = val; } bool          getNoReverseLastSigCoeffConstraintFlag() const {
  //  return m_noReverseLastSigCoeffConstraintFlag; } void          setNoReverseLastSigCoeffConstraintFlag(bool val) {
  //  m_noReverseLastSigCoeffConstraintFlag = val; }
  //
  friend bool operator==(const ConstraintInfo &op1, const ConstraintInfo &op2);
  friend bool operator!=(const ConstraintInfo &op1, const ConstraintInfo &op2);
};
