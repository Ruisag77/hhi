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

#include "CommonLib/CommonDef.h"
#include "CommonLib/SEI.h"
#include "EncGOP.h"
#include "EncLib.h"
#include <fstream>

uint32_t    calcMD5(const CPelUnitBuf &pic, PictureHash &digest, const BitDepths &bitDepths);
uint32_t    calcCRC(const CPelUnitBuf &pic, PictureHash &digest, const BitDepths &bitDepths);
uint32_t    calcChecksum(const CPelUnitBuf &pic, PictureHash &digest, const BitDepths &bitDepths);
std::string hashToString(const PictureHash &digest, int numChar);

//! \ingroup EncoderLib
//! \{

void SEIEncoder::initSEIFramePacking(SEIFramePacking *seiFramePacking, int currPicNum)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiFramePacking != nullptr), "Unspecified error");

  seiFramePacking->m_arrangementId         = m_encCfg->m_seiCfg.m_framePackingSEIId;
  seiFramePacking->m_arrangementCancelFlag = 0;
  seiFramePacking->m_arrangementType       = m_encCfg->m_seiCfg.m_framePackingSEIType;
  CHECK(!((seiFramePacking->m_arrangementType > 2) && (seiFramePacking->m_arrangementType < 6)), "Unspecified error");
  seiFramePacking->m_quincunxSamplingFlag       = m_encCfg->m_seiCfg.m_framePackingSEIQuincunx;
  seiFramePacking->m_contentInterpretationType  = m_encCfg->m_seiCfg.m_framePackingSEIInterpretation;
  seiFramePacking->m_spatialFlippingFlag        = 0;
  seiFramePacking->m_frame0FlippedFlag          = 0;
  seiFramePacking->m_fieldViewsFlag             = (seiFramePacking->m_arrangementType == 2);
  seiFramePacking->m_currentFrameIsFrame0Flag   = ((seiFramePacking->m_arrangementType == 5) && (currPicNum & 1));
  seiFramePacking->m_frame0SelfContainedFlag    = 0;
  seiFramePacking->m_frame1SelfContainedFlag    = 0;
  seiFramePacking->m_frame0GridPositionX        = 0;
  seiFramePacking->m_frame0GridPositionY        = 0;
  seiFramePacking->m_frame1GridPositionX        = 0;
  seiFramePacking->m_frame1GridPositionY        = 0;
  seiFramePacking->m_arrangementReservedByte    = 0;
  seiFramePacking->m_arrangementPersistenceFlag = true;
  seiFramePacking->m_upsampledAspectRatio       = 0;
}

void SEIEncoder::initSEIParameterSetsInclusionIndication(
  SEIParameterSetsInclusionIndication *seiParameterSetsInclusionIndication)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiParameterSetsInclusionIndication != nullptr), "Unspecified error");

  seiParameterSetsInclusionIndication->m_selfContainedClvsFlag = m_encCfg->m_seiCfg.m_selfContainedClvsFlag;
}

void SEIEncoder::initSEIBufferingPeriod(SEIBufferingPeriod *bufferingPeriodSEI, bool noLeadingPictures)
{
  CHECK(!(m_isInitialized), "bufferingPeriodSEI already initialized");
  CHECK(!(bufferingPeriodSEI != nullptr), "Need a bufferingPeriodSEI for initialization (got nullptr)");

  uint32_t uiInitialCpbRemovalDelay               = (90000 / 2);                      // 0.5 sec
  bufferingPeriodSEI->m_bpNalCpbParamsPresentFlag = true;
  bufferingPeriodSEI->m_bpVclCpbParamsPresentFlag = true;
  bufferingPeriodSEI->m_bpMaxSubLayers            = m_encCfg->m_maxTempLayer;
  bufferingPeriodSEI->m_bpCpbCnt                  = 1;
  for (int i = 0; i < bufferingPeriodSEI->m_bpMaxSubLayers; i++)
  {
    for (int j = 0; j < bufferingPeriodSEI->m_bpCpbCnt; j++)
    {
      bufferingPeriodSEI->m_initialCpbRemovalDelay[i][j][0]  = uiInitialCpbRemovalDelay;
      bufferingPeriodSEI->m_initialCpbRemovalDelay[i][j][1]  = uiInitialCpbRemovalDelay;
      bufferingPeriodSEI->m_initialCpbRemovalOffset[i][j][0] = uiInitialCpbRemovalDelay;
      bufferingPeriodSEI->m_initialCpbRemovalOffset[i][j][1] = uiInitialCpbRemovalDelay;
    }
  }
  // We don't set concatenation_flag here. max_initial_removal_delay_for_concatenation depends on the usage scenario.
  // The parameters could be added to config file, but as long as the initialisation of generic buffering parameters is
  // not controllable, it does not seem to make sense to provide settings for these.
  bufferingPeriodSEI->m_concatenationFlag                      = false;
  bufferingPeriodSEI->m_maxInitialRemovalDelayForConcatenation = uiInitialCpbRemovalDelay;

  bufferingPeriodSEI->m_bpDecodingUnitHrdParamsPresentFlag      = m_encCfg->m_picPartitionFlag;
  bufferingPeriodSEI->m_decodingUnitCpbParamsInPicTimingSeiFlag = !m_encCfg->m_seiCfg.m_decodingUnitInfoSEIEnabled;

  bufferingPeriodSEI->m_initialCpbRemovalDelayLength = 16;   // assuming 0.5 sec, log2( 90,000 * 0.5 ) = 16-bit
  // Note: The following parameters require some knowledge about the GOP structure.
  //       Using getIntraPeriod() should be avoided though, because it assumes certain GOP
  //       properties, which are only valid in CTC.
  //       Still copying this setting from HM for consistency, improvements welcome
  bool isRandomAccess                                = m_encCfg->m_intraPeriod > 0;
  if (isRandomAccess)
  {
    bufferingPeriodSEI->m_cpbRemovalDelayLength = 6;   // 32 = 2^5 (plus 1)
    bufferingPeriodSEI->m_dpbOutputDelayLength  = 6;   // 32 + 3 = 2^6
  }
  else
  {
    bufferingPeriodSEI->m_cpbRemovalDelayLength = 9;   // max. 2^10
    bufferingPeriodSEI->m_dpbOutputDelayLength  = 9;   // max. 2^10
  }
  bufferingPeriodSEI->m_duCpbRemovalDelayIncrementLength = 7;   // ceil( log2( tick_divisor_minus2 + 2 ) )
  bufferingPeriodSEI->m_dpbOutputDelayDuLength =
    bufferingPeriodSEI->m_dpbOutputDelayLength + bufferingPeriodSEI->m_duCpbRemovalDelayIncrementLength;
  // for the concatenation, it can be set to one during splicing.
  bufferingPeriodSEI->m_concatenationFlag                = 0;
  // since the temporal layer HRDParameters is not ready, we assumed it is fixed
  bufferingPeriodSEI->m_auCpbRemovalDelayDelta           = 1;
  bufferingPeriodSEI->m_cpbRemovalDelayDeltasPresentFlag = m_encCfg->m_seiCfg.m_bpDeltasGOPStructure;
  if (bufferingPeriodSEI->m_cpbRemovalDelayDeltasPresentFlag)
  {
    switch (m_encCfg->m_gopSize)
    {
    case 8:
      {
        if (noLeadingPictures)
        {
          bufferingPeriodSEI->m_numCpbRemovalDelayDeltas = 5;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[0]  = 1;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[1]  = 2;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[2]  = 3;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[3]  = 6;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[4]  = 7;
        }
        else
        {
          bufferingPeriodSEI->m_numCpbRemovalDelayDeltas = 3;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[0]  = 1;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[1]  = 2;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[2]  = 3;
        }
      }
      break;
    case 16:
      {
        if (noLeadingPictures)
        {
          bufferingPeriodSEI->m_numCpbRemovalDelayDeltas = 9;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[0]  = 1;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[1]  = 2;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[2]  = 3;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[3]  = 4;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[4]  = 6;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[5]  = 7;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[6]  = 9;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[7]  = 14;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[8]  = 15;
        }
        else
        {
          bufferingPeriodSEI->m_numCpbRemovalDelayDeltas = 5;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[0]  = 1;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[1]  = 2;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[2]  = 3;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[3]  = 6;
          bufferingPeriodSEI->m_cpbRemovalDelayDelta[4]  = 7;
        }
      }
      break;
    default:
      {
        THROW("m_cpbRemovalDelayDelta not applicable for the GOP size");
      }
      break;
    }
  }
  bufferingPeriodSEI->m_sublayerDpbOutputOffsetsPresentFlag = true;
  for (int i = 0; i < bufferingPeriodSEI->m_bpMaxSubLayers; i++)
  {
    bufferingPeriodSEI->m_dpbOutputTidOffset[i] = m_encCfg->m_maxNumReorderPics[i] *
      static_cast<int>(pow(2, static_cast<double>(bufferingPeriodSEI->m_bpMaxSubLayers - 1 - i)));
    if (bufferingPeriodSEI->m_dpbOutputTidOffset[i] >=
        m_encCfg->m_maxNumReorderPics[bufferingPeriodSEI->m_bpMaxSubLayers - 1])
    {
      bufferingPeriodSEI->m_dpbOutputTidOffset[i] -=
        m_encCfg->m_maxNumReorderPics[bufferingPeriodSEI->m_bpMaxSubLayers - 1];
    }
    else
    {
      bufferingPeriodSEI->m_dpbOutputTidOffset[i] = 0;
    }
  }
  // A commercial encoder should track the buffer state for all layers and sub-layers
  // to ensure CPB conformance. Such tracking is required for calculating alternative
  // CPB parameters.
  // Unfortunately VTM does not have such tracking. Thus we cannot encode alternative
  // CPB parameters here.
  bufferingPeriodSEI->m_altCpbParamsPresentFlag = false;
  bufferingPeriodSEI->m_useAltCpbParamsFlag     = false;
}

void SEIEncoder::initSEIErp(SEIEquirectangularProjection *seiEquirectangularProjection)
{
  CHECK(!(m_isInitialized), "seiEquirectangularProjection already initialized");
  CHECK(!(seiEquirectangularProjection != nullptr),
        "Need a seiEquirectangularProjection for initialization (got nullptr)");

  seiEquirectangularProjection->m_erpCancelFlag = m_encCfg->m_seiCfg.m_erpSEICancelFlag;
  if (!seiEquirectangularProjection->m_erpCancelFlag)
  {
    seiEquirectangularProjection->m_erpPersistenceFlag = m_encCfg->m_seiCfg.m_erpSEIPersistenceFlag;
    seiEquirectangularProjection->m_erpGuardBandFlag   = m_encCfg->m_seiCfg.m_erpSEIGuardBandFlag;
    if (seiEquirectangularProjection->m_erpGuardBandFlag == 1)
    {
      seiEquirectangularProjection->m_erpGuardBandType       = m_encCfg->m_seiCfg.m_erpSEIGuardBandType;
      seiEquirectangularProjection->m_erpLeftGuardBandWidth  = m_encCfg->m_seiCfg.m_erpSEILeftGuardBandWidth;
      seiEquirectangularProjection->m_erpRightGuardBandWidth = m_encCfg->m_seiCfg.m_erpSEIRightGuardBandWidth;
    }
  }
}

void SEIEncoder::initSEISphereRotation(SEISphereRotation *seiSphereRotation)
{
  CHECK(!(m_isInitialized), "seiSphereRotation already initialized");
  CHECK(!(seiSphereRotation != nullptr), "Need a seiSphereRotation for initialization (got nullptr)");

  seiSphereRotation->m_sphereRotationCancelFlag = m_encCfg->m_seiCfg.m_sphereRotationSEICancelFlag;
  if (!seiSphereRotation->m_sphereRotationCancelFlag)
  {
    seiSphereRotation->m_sphereRotationPersistenceFlag = m_encCfg->m_seiCfg.m_sphereRotationSEIPersistenceFlag;
    seiSphereRotation->m_sphereRotationYaw             = m_encCfg->m_seiCfg.m_sphereRotationSEIYaw;
    seiSphereRotation->m_sphereRotationPitch           = m_encCfg->m_seiCfg.m_sphereRotationSEIPitch;
    seiSphereRotation->m_sphereRotationRoll            = m_encCfg->m_seiCfg.m_sphereRotationSEIRoll;
  }
}

void SEIEncoder::initSEIOmniViewport(SEIOmniViewport *seiOmniViewport)
{
  CHECK(!(m_isInitialized), "seiOmniViewport already initialized");
  CHECK(!(seiOmniViewport != nullptr), "Need a seiOmniViewport for initialization (got nullptr)");

  seiOmniViewport->m_omniViewportId         = m_encCfg->m_seiCfg.m_omniViewportSEIId;
  seiOmniViewport->m_omniViewportCancelFlag = m_encCfg->m_seiCfg.m_omniViewportSEICancelFlag;
  if (!seiOmniViewport->m_omniViewportCancelFlag)
  {
    seiOmniViewport->m_omniViewportPersistenceFlag = m_encCfg->m_seiCfg.m_omniViewportSEIPersistenceFlag;
    seiOmniViewport->m_omniViewportCntMinus1       = m_encCfg->m_seiCfg.m_omniViewportSEICntMinus1;

    seiOmniViewport->m_omniViewportRegions.resize(seiOmniViewport->m_omniViewportCntMinus1 + 1);
    for (uint32_t i = 0; i <= seiOmniViewport->m_omniViewportCntMinus1; i++)
    {
      SEIOmniViewport::OmniViewport &viewport = seiOmniViewport->m_omniViewportRegions[i];
      viewport.azimuthCentre                  = m_encCfg->m_seiCfg.m_omniViewportSEIAzimuthCentre[i];
      viewport.elevationCentre                = m_encCfg->m_seiCfg.m_omniViewportSEIElevationCentre[i];
      viewport.tiltCentre                     = m_encCfg->m_seiCfg.m_omniViewportSEITiltCentre[i];
      viewport.horRange                       = m_encCfg->m_seiCfg.m_omniViewportSEIHorRange[i];
      viewport.verRange                       = m_encCfg->m_seiCfg.m_omniViewportSEIVerRange[i];
    }
  }
}

void SEIEncoder::initSEIRegionWisePacking(SEIRegionWisePacking *seiRegionWisePacking)
{
  CHECK(!(m_isInitialized), "seiRegionWisePacking already initialized");
  CHECK(!(seiRegionWisePacking != nullptr), "Need a seiRegionWisePacking for initialization (got nullptr)");

  seiRegionWisePacking->m_rwpCancelFlag                  = m_encCfg->m_seiCfg.m_rwpSEIRwpCancelFlag;
  seiRegionWisePacking->m_rwpPersistenceFlag             = m_encCfg->m_seiCfg.m_rwpSEIRwpPersistenceFlag;
  seiRegionWisePacking->m_constituentPictureMatchingFlag = m_encCfg->m_seiCfg.m_rwpSEIConstituentPictureMatchingFlag;
  seiRegionWisePacking->m_numPackedRegions               = m_encCfg->m_seiCfg.m_rwpSEINumPackedRegions;
  seiRegionWisePacking->m_projPictureWidth               = m_encCfg->m_seiCfg.m_rwpSEIProjPictureWidth;
  seiRegionWisePacking->m_projPictureHeight              = m_encCfg->m_seiCfg.m_rwpSEIProjPictureHeight;
  seiRegionWisePacking->m_packedPictureWidth             = m_encCfg->m_seiCfg.m_rwpSEIPackedPictureWidth;
  seiRegionWisePacking->m_packedPictureHeight            = m_encCfg->m_seiCfg.m_rwpSEIPackedPictureHeight;
  seiRegionWisePacking->m_rwpTransformType.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpGuardBandFlag.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_projRegionWidth.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_projRegionHeight.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpProjRegionTop.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_projRegionLeft.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_packedRegionWidth.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_packedRegionHeight.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_packedRegionTop.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_packedRegionLeft.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpLeftGuardBandWidth.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpRightGuardBandWidth.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpTopGuardBandHeight.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpBottomGuardBandHeight.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpGuardBandNotUsedForPredFlag.resize(seiRegionWisePacking->m_numPackedRegions);
  seiRegionWisePacking->m_rwpGuardBandType.resize(4 * seiRegionWisePacking->m_numPackedRegions);
  for (int i = 0; i < seiRegionWisePacking->m_numPackedRegions; i++)
  {
    seiRegionWisePacking->m_rwpTransformType[i]   = m_encCfg->m_seiCfg.m_rwpSEIRwpTransformType[i];
    seiRegionWisePacking->m_rwpGuardBandFlag[i]   = m_encCfg->m_seiCfg.m_rwpSEIRwpGuardBandFlag[i];
    seiRegionWisePacking->m_projRegionWidth[i]    = m_encCfg->m_seiCfg.m_rwpSEIProjRegionWidth[i];
    seiRegionWisePacking->m_projRegionHeight[i]   = m_encCfg->m_seiCfg.m_rwpSEIProjRegionHeight[i];
    seiRegionWisePacking->m_rwpProjRegionTop[i]   = m_encCfg->m_seiCfg.m_rwpSEIRwpSEIProjRegionTop[i];
    seiRegionWisePacking->m_projRegionLeft[i]     = m_encCfg->m_seiCfg.m_rwpSEIProjRegionLeft[i];
    seiRegionWisePacking->m_packedRegionWidth[i]  = m_encCfg->m_seiCfg.m_rwpSEIPackedRegionWidth[i];
    seiRegionWisePacking->m_packedRegionHeight[i] = m_encCfg->m_seiCfg.m_rwpSEIPackedRegionHeight[i];
    seiRegionWisePacking->m_packedRegionTop[i]    = m_encCfg->m_seiCfg.m_rwpSEIPackedRegionTop[i];
    seiRegionWisePacking->m_packedRegionLeft[i]   = m_encCfg->m_seiCfg.m_rwpSEIPackedRegionLeft[i];
    if (seiRegionWisePacking->m_rwpGuardBandFlag[i])
    {
      seiRegionWisePacking->m_rwpLeftGuardBandWidth[i]    = m_encCfg->m_seiCfg.m_rwpSEIRwpLeftGuardBandWidth[i];
      seiRegionWisePacking->m_rwpRightGuardBandWidth[i]   = m_encCfg->m_seiCfg.m_rwpSEIRwpRightGuardBandWidth[i];
      seiRegionWisePacking->m_rwpTopGuardBandHeight[i]    = m_encCfg->m_seiCfg.m_rwpSEIRwpTopGuardBandHeight[i];
      seiRegionWisePacking->m_rwpBottomGuardBandHeight[i] = m_encCfg->m_seiCfg.m_rwpSEIRwpBottomGuardBandHeight[i];
      seiRegionWisePacking->m_rwpGuardBandNotUsedForPredFlag[i] =
        m_encCfg->m_seiCfg.m_rwpSEIRwpGuardBandNotUsedForPredFlag[i];
      for (int j = 0; j < 4; j++)
      {
        seiRegionWisePacking->m_rwpGuardBandType[i * 4 + j] = m_encCfg->m_seiCfg.m_rwpSEIRwpGuardBandType[i * 4 + j];
      }
    }
  }
}

void SEIEncoder::initSEIGcmp(SEIGeneralizedCubemapProjection *seiGeneralizedCubemapProjection)
{
  CHECK(!(m_isInitialized), "seiGeneralizedCubemapProjection already initialized");
  CHECK(!(seiGeneralizedCubemapProjection != nullptr),
        "Need a seiGeneralizedCubemapProjection for initialization (got nullptr)");

  seiGeneralizedCubemapProjection->m_gcmpCancelFlag = m_encCfg->m_seiCfg.m_gcmpSEICancelFlag;
  if (!seiGeneralizedCubemapProjection->m_gcmpCancelFlag)
  {
    seiGeneralizedCubemapProjection->m_gcmpPersistenceFlag     = m_encCfg->m_seiCfg.m_gcmpSEIPersistenceFlag;
    seiGeneralizedCubemapProjection->m_gcmpPackingType         = m_encCfg->m_seiCfg.m_gcmpSEIPackingType;
    seiGeneralizedCubemapProjection->m_gcmpMappingFunctionType = m_encCfg->m_seiCfg.m_gcmpSEIMappingFunctionType;

    int numFace =
      seiGeneralizedCubemapProjection->m_gcmpPackingType == 4 || seiGeneralizedCubemapProjection->m_gcmpPackingType == 5
      ? 5
      : 6;
    seiGeneralizedCubemapProjection->m_gcmpFaceIndex.resize(numFace);
    seiGeneralizedCubemapProjection->m_gcmpFaceRotation.resize(numFace);
    if (seiGeneralizedCubemapProjection->m_gcmpMappingFunctionType == 2)
    {
      seiGeneralizedCubemapProjection->m_gcmpFunctionCoeffU.resize(numFace);
      seiGeneralizedCubemapProjection->m_gcmpFunctionUAffectedByVFlag.resize(numFace);
      seiGeneralizedCubemapProjection->m_gcmpFunctionCoeffV.resize(numFace);
      seiGeneralizedCubemapProjection->m_gcmpFunctionVAffectedByUFlag.resize(numFace);
    }
    for (int i = 0; i < numFace; i++)
    {
      seiGeneralizedCubemapProjection->m_gcmpFaceIndex[i]    = m_encCfg->m_seiCfg.m_gcmpSEIFaceIndex[i];
      seiGeneralizedCubemapProjection->m_gcmpFaceRotation[i] = m_encCfg->m_seiCfg.m_gcmpSEIFaceRotation[i];
      if (seiGeneralizedCubemapProjection->m_gcmpMappingFunctionType == 2)
      {
        seiGeneralizedCubemapProjection->m_gcmpFunctionCoeffU[i] =
          std::max<uint8_t>(1, (uint8_t)(128.0 * m_encCfg->m_seiCfg.m_gcmpSEIFunctionCoeffU[i] + 0.5)) - 1;
        seiGeneralizedCubemapProjection->m_gcmpFunctionUAffectedByVFlag[i] =
          m_encCfg->m_seiCfg.m_gcmpSEIFunctionUAffectedByVFlag[i];
        seiGeneralizedCubemapProjection->m_gcmpFunctionCoeffV[i] =
          std::max<uint8_t>(1, (uint8_t)(128.0 * m_encCfg->m_seiCfg.m_gcmpSEIFunctionCoeffV[i] + 0.5)) - 1;
        seiGeneralizedCubemapProjection->m_gcmpFunctionVAffectedByUFlag[i] =
          m_encCfg->m_seiCfg.m_gcmpSEIFunctionVAffectedByUFlag[i];
      }
    }

    seiGeneralizedCubemapProjection->m_gcmpGuardBandFlag = m_encCfg->m_seiCfg.m_gcmpSEIGuardBandFlag;
    if (seiGeneralizedCubemapProjection->m_gcmpGuardBandFlag)
    {
      seiGeneralizedCubemapProjection->m_gcmpGuardBandType = m_encCfg->m_seiCfg.m_gcmpSEIGuardBandType;
      seiGeneralizedCubemapProjection->m_gcmpGuardBandBoundaryExteriorFlag =
        m_encCfg->m_seiCfg.m_gcmpSEIGuardBandBoundaryExteriorFlag;
      seiGeneralizedCubemapProjection->m_gcmpGuardBandSamplesMinus1 =
        m_encCfg->m_seiCfg.m_gcmpSEIGuardBandSamplesMinus1;
    }
  }
}

void SEIEncoder::initSEISampleAspectRatioInfo(SEISampleAspectRatioInfo *seiSampleAspectRatioInfo)
{
  CHECK(!(m_isInitialized), "seiSampleAspectRatioInfo already initialized");
  CHECK(!(seiSampleAspectRatioInfo != nullptr), "Need a seiSampleAspectRatioInfo for initialization (got nullptr)");

  seiSampleAspectRatioInfo->m_sariCancelFlag = m_encCfg->m_seiCfg.m_sariCancelFlag;
  if (!seiSampleAspectRatioInfo->m_sariCancelFlag)
  {
    seiSampleAspectRatioInfo->m_sariPersistenceFlag = m_encCfg->m_seiCfg.m_sariPersistenceFlag;
    seiSampleAspectRatioInfo->m_sariAspectRatioIdc  = m_encCfg->m_seiCfg.m_sariAspectRatioIdc;
    if (seiSampleAspectRatioInfo->m_sariAspectRatioIdc == 255)
    {
      seiSampleAspectRatioInfo->m_sariSarWidth  = m_encCfg->m_seiCfg.m_sariSarWidth;
      seiSampleAspectRatioInfo->m_sariSarHeight = m_encCfg->m_seiCfg.m_sariSarHeight;
    }
    else
    {
      seiSampleAspectRatioInfo->m_sariSarWidth  = 0;
      seiSampleAspectRatioInfo->m_sariSarHeight = 0;
    }
  }
}

void SEIEncoder::initSEIPhaseIndication(SEIPhaseIndication *seiPhaseIndication, int ppsId)
{
  CHECK(!(m_isInitialized), "seiPhaseIndication already initialized");
  CHECK(!(seiPhaseIndication != nullptr), "Need a seiPhaseIndication for initialization (got nullptr)");

  if (ppsId == 0)
  {
    seiPhaseIndication->m_horPhaseNum       = m_encCfg->m_seiCfg.m_horPhaseNumFullResolution;
    seiPhaseIndication->m_horPhaseDenMinus1 = m_encCfg->m_seiCfg.m_horPhaseDenMinus1FullResolution;
    seiPhaseIndication->m_verPhaseNum       = m_encCfg->m_seiCfg.m_verPhaseNumFullResolution;
    seiPhaseIndication->m_verPhaseDenMinus1 = m_encCfg->m_seiCfg.m_verPhaseDenMinus1FullResolution;
  }
  else if (ppsId == ENC_PPS_ID_RPR)
  {
    seiPhaseIndication->m_horPhaseNum       = m_encCfg->m_seiCfg.m_horPhaseNumReducedResolution;
    seiPhaseIndication->m_horPhaseDenMinus1 = m_encCfg->m_seiCfg.m_horPhaseDenMinus1ReducedResolution;
    seiPhaseIndication->m_verPhaseNum       = m_encCfg->m_seiCfg.m_verPhaseNumReducedResolution;
    seiPhaseIndication->m_verPhaseDenMinus1 = m_encCfg->m_seiCfg.m_verPhaseDenMinus1ReducedResolution;
  }
}

//! initialize scalable nesting SEI message.
//! Note: The SEI message structures input into this function will become part of the scalable nesting SEI and will be
//!       automatically freed, when the nesting SEI is disposed.
//  either targetOLS or targetLayer should be active, call with empty vector for the inactive mode
void SEIEncoder::initSEIScalableNesting(SEIScalableNesting *scalableNestingSEI, SEIMessages &nestedSEIs,
                                        const std::vector<int> &targetOLSs, const std::vector<int> &targetLayers,
                                        const std::vector<uint16_t> &subpictureIDs, uint16_t maxSubpicIdInPic)
{
  CHECK(!(m_isInitialized), "Scalable Nesting SEI already initialized ");
  CHECK(!(scalableNestingSEI != nullptr), "No Scalable Nesting SEI object passed");
  CHECK(targetOLSs.size() > 0 && targetLayers.size() > 0,
        "Scalable Nesting SEI can apply to either OLS or layer(s), not both");

  scalableNestingSEI->m_snOlsFlag = (targetOLSs.size() > 0)
    ? 1
    : 0;   // If the nested SEI messages are picture buffering SEI messages, picture timing SEI messages or sub-picture
           // timing SEI messages, nesting_ols_flag shall be equal to 1, by default case
  if (scalableNestingSEI->m_snOlsFlag)
  {
    scalableNestingSEI->m_snNumOlssMinus1 = (uint32_t)targetOLSs.size() - 1;
    // initialize absolute indexes
    for (int i = 0; i <= scalableNestingSEI->m_snNumOlssMinus1; i++)
    {
      scalableNestingSEI->m_snOlsIdx[i] = targetOLSs[i];
    }
    // calculate delta indexes from absolute ones
    for (int i = 0; i <= scalableNestingSEI->m_snNumOlssMinus1; i++)
    {
      if (i == 0)
      {
        CHECK(scalableNestingSEI->m_snOlsIdx[i] < 0, "OLS indexes must be  equal to or greater than 0");
        // no "-1" operation for the first index although the name implies one
        scalableNestingSEI->m_snOlsIdxDeltaMinus1[i] = scalableNestingSEI->m_snOlsIdx[i];
      }
      else
      {
        CHECK(scalableNestingSEI->m_snOlsIdx[i] <= scalableNestingSEI->m_snOlsIdx[i - 1],
              "OLS indexes must be in ascending order");
        scalableNestingSEI->m_snOlsIdxDeltaMinus1[i] =
          scalableNestingSEI->m_snOlsIdx[i] - scalableNestingSEI->m_snOlsIdx[i - 1] - 1;
      }
    }
  }
  else
  {
    scalableNestingSEI->m_snAllLayersFlag   = 0;   // nesting is not applied to all layers
    scalableNestingSEI->m_snNumLayersMinus1 = (uint32_t)targetLayers.size() - 1;   // nesting_num_layers_minus1
    for (int i = 0; i <= scalableNestingSEI->m_snNumLayersMinus1; i++)
    {
      scalableNestingSEI->m_snLayerId[i] = targetLayers[i];
    }
  }
  if (!subpictureIDs.empty())
  {
    scalableNestingSEI->m_snSubpicFlag  = 1;
    scalableNestingSEI->m_snNumSubpics  = (uint32_t)subpictureIDs.size();
    scalableNestingSEI->m_snSubpicId    = subpictureIDs;
    scalableNestingSEI->m_snSubpicIdLen = std::max(1, ceilLog2(maxSubpicIdInPic + 1));
    CHECK(scalableNestingSEI->m_snSubpicIdLen > 16, "Subpicture ID too large. Length must be <= 16 bits");
  }
  scalableNestingSEI->m_nestedSEIs.clear();
  for (SEIMessages::iterator it = nestedSEIs.begin(); it != nestedSEIs.end(); it++)
  {
    scalableNestingSEI->m_nestedSEIs.push_back((*it));
  }
}

//! calculate hashes for entire reconstructed picture
void SEIEncoder::initDecodedPictureHashSEI(SEIDecodedPictureHash *decodedPictureHashSEI, PelUnitBuf &pic,
                                           std::string &rHashString, const BitDepths &bitDepths)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(decodedPictureHashSEI != nullptr), "Unspecified error");

  decodedPictureHashSEI->method         = m_encCfg->m_seiCfg.m_decodedPictureHashSEIType;
  decodedPictureHashSEI->singleCompFlag = !isChromaEnabled(m_encCfg->m_chromaFormatIdc);
  switch (m_encCfg->m_seiCfg.m_decodedPictureHashSEIType)
  {
  case HashType::MD5:
    {
      uint32_t numChar = calcMD5(pic, decodedPictureHashSEI->m_pictureHash, bitDepths);
      rHashString      = hashToString(decodedPictureHashSEI->m_pictureHash, numChar);
      break;
    }
    break;
  case HashType::CRC:
    {
      uint32_t numChar = calcCRC(pic, decodedPictureHashSEI->m_pictureHash, bitDepths);
      rHashString      = hashToString(decodedPictureHashSEI->m_pictureHash, numChar);
      break;
    }
  case HashType::CHECKSUM:
  default:
    {
      uint32_t numChar = calcChecksum(pic, decodedPictureHashSEI->m_pictureHash, bitDepths);
      rHashString      = hashToString(decodedPictureHashSEI->m_pictureHash, numChar);
      break;
    }
  }
}

void SEIEncoder::initSEIDependentRAPIndication(SEIDependentRAPIndication *seiDependentRAPIndication)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiDependentRAPIndication != nullptr), "Unspecified error");
}

void SEIEncoder::initSEIExtendedDrapIndication(SEIExtendedDrapIndication *sei)
{
  CHECK(!(m_isInitialized), "Extended DRAP SEI already initialized");
  CHECK(!(sei != nullptr), "Need a seiExtendedDrapIndication for initialization (got nullptr)");
  sei->m_edrapIndicationRapIdMinus1                  = 0;
  sei->m_edrapIndicationLeadingPicturesDecodableFlag = false;
  sei->m_edrapIndicationReservedZero12Bits           = 0;
  sei->m_edrapIndicationNumRefRapPicsMinus1          = 0;
  sei->m_edrapIndicationRefRapId.resize(sei->m_edrapIndicationNumRefRapPicsMinus1 + 1);
  for (int i = 0; i <= sei->m_edrapIndicationNumRefRapPicsMinus1; i++)
  {
    sei->m_edrapIndicationRefRapId[i] = 0;
  }
}

void SEIEncoder::initSEIShutterIntervalInfo(SEIShutterIntervalInfo *seiShutterIntervalInfo)
{
  assert(m_isInitialized);
  assert(seiShutterIntervalInfo != nullptr);
  seiShutterIntervalInfo->m_siiTimeScale         = m_encCfg->m_seiCfg.m_siiSEITimeScale;
  seiShutterIntervalInfo->m_siiFixedSIwithinCLVS = m_encCfg->m_seiCfg.m_siiSEISubLayerNumUnitsInSI.empty();
  if (seiShutterIntervalInfo->m_siiFixedSIwithinCLVS == true)
  {
    seiShutterIntervalInfo->m_siiNumUnitsInShutterInterval = m_encCfg->m_seiCfg.m_siiSEINumUnitsInShutterInterval;
  }
  else
  {
    seiShutterIntervalInfo->m_siiMaxSubLayersMinus1 =
      std::max(1u, uint32_t(m_encCfg->m_seiCfg.m_siiSEISubLayerNumUnitsInSI.size())) - 1;
    seiShutterIntervalInfo->m_siiSubLayerNumUnitsInSI.resize(seiShutterIntervalInfo->m_siiMaxSubLayersMinus1 + 1);
    for (int32_t i = 0; i <= seiShutterIntervalInfo->m_siiMaxSubLayersMinus1; i++)
    {
      seiShutterIntervalInfo->m_siiSubLayerNumUnitsInSI[i] = m_encCfg->m_seiCfg.m_siiSEISubLayerNumUnitsInSI[i];
    }
  }
}

void SEIEncoder::initSEIProcessingOrderInfo(SEIProcessingOrderInfo *seiProcessingOrderInfo)
{
  assert(m_isInitialized);
  assert(seiProcessingOrderInfo != nullptr);

  seiProcessingOrderInfo->m_posEnabled = m_encCfg->m_seiCfg.m_poSEIEnabled;
  seiProcessingOrderInfo->m_posPayloadType.resize(m_encCfg->m_seiCfg.m_poSEIPayloadType.size());
  seiProcessingOrderInfo->m_posProcessingOrder.resize(m_encCfg->m_seiCfg.m_poSEIPayloadType.size());
  seiProcessingOrderInfo->m_posPrefixByte.resize(m_encCfg->m_seiCfg.m_poSEIPayloadType.size());
  for (uint32_t i = 0; i < (uint32_t)m_encCfg->m_seiCfg.m_poSEIPayloadType.size(); i++)
  {
    seiProcessingOrderInfo->m_posPayloadType[i]     = m_encCfg->m_seiCfg.m_poSEIPayloadType[i];
    seiProcessingOrderInfo->m_posProcessingOrder[i] = m_encCfg->m_seiCfg.m_poSEIProcessingOrder[i];
    if (seiProcessingOrderInfo->m_posPayloadType[i] == (uint16_t)SEI::PayloadType::USER_DATA_REGISTERED_ITU_T_T35)
    {
      seiProcessingOrderInfo->m_posPrefixByte[i] = m_encCfg->m_seiCfg.m_poSEIPrefixByte[i];
    }
  }
}

void SEIEncoder::initSEIPostFilterHint(SEIPostFilterHint *seiPostFilterHint)
{
  CHECK(!m_isInitialized, "The post-filter hint SEI message needs to be initialized");
  CHECK(seiPostFilterHint == nullptr, "Failed to get the handler to the SEI message");

  seiPostFilterHint->m_filterHintCancelFlag             = m_encCfg->m_seiCfg.m_postFilterHintSEICancelFlag;
  seiPostFilterHint->m_filterHintPersistenceFlag        = m_encCfg->m_seiCfg.m_postFilterHintSEIPersistenceFlag;
  seiPostFilterHint->m_filterHintSizeY                  = m_encCfg->m_seiCfg.m_postFilterHintSEISizeY;
  seiPostFilterHint->m_filterHintSizeX                  = m_encCfg->m_seiCfg.m_postFilterHintSEISizeX;
  seiPostFilterHint->m_filterHintType                   = m_encCfg->m_seiCfg.m_postFilterHintSEIType;
  seiPostFilterHint->m_filterHintChromaCoeffPresentFlag = m_encCfg->m_seiCfg.m_postFilterHintSEIChromaCoeffPresentFlag;

  seiPostFilterHint->m_filterHintValues.resize((seiPostFilterHint->m_filterHintChromaCoeffPresentFlag ? 3 : 1) *
                                               seiPostFilterHint->m_filterHintSizeY *
                                               seiPostFilterHint->m_filterHintSizeX);
  for (uint32_t i = 0; i < seiPostFilterHint->m_filterHintValues.size(); i++)
  {
    seiPostFilterHint->m_filterHintValues[i] = m_encCfg->m_seiCfg.m_postFilterHintValues[i];
  }
}

template<typename T> static void readTokenValue(T            &returnedValue,   /// value returned
                                                bool         &failed,   /// used and updated
                                                std::istream &is,   /// stream to read token from
                                                const char   *pToken)   /// token string
{
  returnedValue = T();
  if (failed)
  {
    return;
  }

  int c;
  // Ignore any whitespace
  while ((c = is.get()) != EOF && isspace(c))
    ;
  // test for comment mark
  while (c == '#')
  {
    // Ignore to the end of the line
    while ((c = is.get()) != EOF && (c != 10 && c != 13))
      ;
    // Ignore any white space at the start of the next line
    while ((c = is.get()) != EOF && isspace(c))
      ;
  }
  // test first character of token
  failed = (c != pToken[0]);
  // test remaining characters of token
  int pos;
  for (pos = 1; !failed && pToken[pos] != 0 && is.get() == pToken[pos]; pos++)
    ;
  failed |= (pToken[pos] != 0);
  // Ignore any whitespace before the ':'
  while (!failed && (c = is.get()) != EOF && isspace(c))
    ;
  failed |= (c != ':');
  // Now read the value associated with the token:
  if (!failed)
  {
    is >> returnedValue;
    failed = !is.good();
    if (!failed)
    {
      c      = is.get();
      failed = (c != EOF && !isspace(c));
    }
  }
  if (failed)
  {
    std::cerr << "Unable to read token '" << pToken << "'\n";
  }
}

template<typename T> static void readTokenValueAndValidate(T            &returnedValue,   /// value returned
                                                           bool         &failed,   /// used and updated
                                                           std::istream &is,   /// stream to read token from
                                                           const char   *pToken,   /// token string
                                                           const T &minInclusive,   /// minimum value allowed, inclusive
                                                           const T &maxInclusive)   /// maximum value allowed, inclusive
{
  readTokenValue(returnedValue, failed, is, pToken);
  if (!failed)
  {
    if (returnedValue < minInclusive || returnedValue > maxInclusive)
    {
      failed = true;
      std::cerr << "Value for token " << pToken << " must be in the range " << minInclusive << " to " << maxInclusive
                << " (inclusive); value read: " << returnedValue << std::endl;
    }
  }
}

void SEIEncoder::readAnnotatedRegionSEI(std::istream &fic, SEIAnnotatedRegions *seiAnnoRegion, bool &failed)
{
  readTokenValue(seiAnnoRegion->m_hdr.m_cancelFlag, failed, fic, "SEIArCancelFlag");
  if (!seiAnnoRegion->m_hdr.m_cancelFlag)
  {
    readTokenValue(seiAnnoRegion->m_hdr.m_notOptimizedForViewingFlag, failed, fic, "SEIArNotOptForViewingFlag");
    readTokenValue(seiAnnoRegion->m_hdr.m_trueMotionFlag, failed, fic, "SEIArTrueMotionFlag");
    readTokenValue(seiAnnoRegion->m_hdr.m_occludedObjectFlag, failed, fic, "SEIArOccludedObjsFlag");
    readTokenValue(seiAnnoRegion->m_hdr.m_partialObjectFlagPresentFlag, failed, fic, "SEIArPartialObjsFlagPresentFlag");
    readTokenValue(seiAnnoRegion->m_hdr.m_objectLabelPresentFlag, failed, fic, "SEIArObjLabelPresentFlag");
    readTokenValue(seiAnnoRegion->m_hdr.m_objectConfidenceInfoPresentFlag, failed, fic, "SEIArObjConfInfoPresentFlag");
    if (seiAnnoRegion->m_hdr.m_objectConfidenceInfoPresentFlag)
    {
      readTokenValueAndValidate<uint32_t>(seiAnnoRegion->m_hdr.m_objectConfidenceLength, failed, fic,
                                          "SEIArObjDetConfLength", uint32_t(0), uint32_t(255));
    }
    if (seiAnnoRegion->m_hdr.m_objectLabelPresentFlag)
    {
      readTokenValue(seiAnnoRegion->m_hdr.m_objectLabelLanguagePresentFlag, failed, fic,
                     "SEIArObjLabelLangPresentFlag");
      if (seiAnnoRegion->m_hdr.m_objectLabelLanguagePresentFlag)
      {
        readTokenValue(seiAnnoRegion->m_hdr.m_annotatedRegionsObjectLabelLang, failed, fic, "SEIArLabelLanguage");
      }
      uint32_t numLabelUpdates = 0;
      readTokenValueAndValidate<uint32_t>(numLabelUpdates, failed, fic, "SEIArNumLabelUpdates", uint32_t(0),
                                          uint32_t(255));
      seiAnnoRegion->m_annotatedLabels.resize(numLabelUpdates);
      for (auto it = seiAnnoRegion->m_annotatedLabels.begin(); it != seiAnnoRegion->m_annotatedLabels.end(); it++)
      {
        SEIAnnotatedRegions::AnnotatedRegionLabel &ar = it->second;
        readTokenValueAndValidate(it->first, failed, fic, "SEIArLabelIdc[c]", uint32_t(0), uint32_t(255));
        bool cancelFlag;
        readTokenValue(cancelFlag, failed, fic, "SEIArLabelCancelFlag[c]");
        ar.labelValid = !cancelFlag;
        if (ar.labelValid)
        {
          readTokenValue(ar.label, failed, fic, "SEIArLabel[c]");
        }
      }
    }

    uint32_t numObjectUpdates = 0;
    readTokenValueAndValidate<uint32_t>(numObjectUpdates, failed, fic, "SEIArNumObjUpdates", uint32_t(0),
                                        uint32_t(255));
    seiAnnoRegion->m_annotatedRegions.resize(numObjectUpdates);
    for (auto it = seiAnnoRegion->m_annotatedRegions.begin(); it != seiAnnoRegion->m_annotatedRegions.end(); it++)
    {
      SEIAnnotatedRegions::AnnotatedRegionObject &ar = it->second;
      readTokenValueAndValidate(it->first, failed, fic, "SEIArObjIdx[c]", uint32_t(0), uint32_t(255));
      readTokenValue(ar.objectCancelFlag, failed, fic, "SEIArObjCancelFlag[c]");
      ar.objectLabelValid      = false;
      ar.boundingBoxValid      = false;
      ar.boundingBoxCancelFlag = false;

      if (!ar.objectCancelFlag)
      {
        if (seiAnnoRegion->m_hdr.m_objectLabelPresentFlag)
        {
          readTokenValue(ar.objectLabelValid, failed, fic, "SEIArObjLabelUpdateFlag[c]");
          if (ar.objectLabelValid)
          {
            readTokenValueAndValidate<uint32_t>(ar.objLabelIdx, failed, fic, "SEIArObjectLabelIdc[c]", uint32_t(0),
                                                uint32_t(255));
          }
        }
        readTokenValue(ar.boundingBoxValid, failed, fic, "SEIArBoundBoxUpdateFlag[c]");
        if (ar.boundingBoxValid)
        {
          readTokenValue(ar.boundingBoxCancelFlag, failed, fic, "SEIArBoundBoxCancelFlag[c]");
          if (!ar.boundingBoxCancelFlag)
          {
            readTokenValueAndValidate<uint32_t>(ar.boundingBoxTop, failed, fic, "SEIArObjTop[c]", uint32_t(0),
                                                uint32_t(0x7fffffff));
            readTokenValueAndValidate<uint32_t>(ar.boundingBoxLeft, failed, fic, "SEIArObjLeft[c]", uint32_t(0),
                                                uint32_t(0x7fffffff));
            readTokenValueAndValidate<uint32_t>(ar.boundingBoxWidth, failed, fic, "SEIArObjWidth[c]", uint32_t(0),
                                                uint32_t(0x7fffffff));
            readTokenValueAndValidate<uint32_t>(ar.boundingBoxHeight, failed, fic, "SEIArObjHeight[c]", uint32_t(0),
                                                uint32_t(0x7fffffff));
            if (seiAnnoRegion->m_hdr.m_partialObjectFlagPresentFlag)
            {
              readTokenValue(ar.partialObjectFlag, failed, fic, "SEIArObjPartUpdateFlag[c]");
            }
            if (seiAnnoRegion->m_hdr.m_objectConfidenceInfoPresentFlag)
            {
              readTokenValueAndValidate<uint32_t>(ar.objectConfidence, failed, fic, "SEIArObjDetConf[c]", uint32_t(0),
                                                  uint32_t(1 << seiAnnoRegion->m_hdr.m_objectConfidenceLength) - 1);
            }
          }
        }
        // Compare with existing attributes to decide whether it's a static object
        // First check whether it's an existing object (or) new object
        auto destIt = m_arObjects.find(it->first);
        // New object
        if (destIt == m_arObjects.end())
        {
          // New object arrived, needs to be appended to the map of tracked objects
          m_arObjects[it->first] = ar;
        }
        // Existing object
        else
        {
          // Size remains the same
          if (m_arObjects[it->first].boundingBoxWidth == ar.boundingBoxWidth &&
              m_arObjects[it->first].boundingBoxHeight == ar.boundingBoxHeight)
          {
            if (m_arObjects[it->first].boundingBoxTop == ar.boundingBoxTop &&
                m_arObjects[it->first].boundingBoxLeft == ar.boundingBoxLeft)
            {
              ar.boundingBoxValid = 0;
            }
          }
        }
      }
    }
  }
}

bool SEIEncoder::initSEIAnnotatedRegions(SEIAnnotatedRegions *SEIAnnoReg, int currPOC)
{
  assert(m_isInitialized);
  assert(SEIAnnoReg != nullptr);

  // reading external Annotated Regions Information SEI message parameters from file
  if (!m_encCfg->m_seiCfg.m_arSEIFileRoot.empty())
  {
    bool        failed = false;
    // building the annotated regions file name with poc num in prefix "_poc.txt"
    std::string annoRegionSeiFileWithPoc(m_encCfg->m_seiCfg.m_arSEIFileRoot);
    {
      std::stringstream suffix;
      suffix << "_" << currPOC << ".txt";
      annoRegionSeiFileWithPoc += suffix.str();
    }
    std::ifstream fic(annoRegionSeiFileWithPoc.c_str());
    if (!fic.good() || !fic.is_open())
    {
      std::cerr << "No Annotated Regions SEI parameters file " << annoRegionSeiFileWithPoc << " for POC " << currPOC
                << std::endl;
      return false;
    }
    // Read annotated region SEI parameters from the cfg file
    readAnnotatedRegionSEI(fic, SEIAnnoReg, failed);
    if (failed)
    {
      std::cerr << "Error while reading Annotated Regions SEI parameters file '" << annoRegionSeiFileWithPoc << "'"
                << std::endl;
      exit(EXIT_FAILURE);
    }
  }
  return true;
}

void SEIEncoder::initSEIAlternativeTransferCharacteristics(
  SEIAlternativeTransferCharacteristics *seiAltTransCharacteristics)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiAltTransCharacteristics != nullptr), "Unspecified error");
  //  Set SEI message parameters read from command line options
  seiAltTransCharacteristics->m_preferredTransferCharacteristics =
    m_encCfg->m_seiCfg.m_preferredTransferCharacteristics;
}
void SEIEncoder::initSEIFilmGrainCharacteristics(SEIFilmGrainCharacteristics *seiFilmGrain)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiFilmGrain != nullptr), "Unspecified error");
  //  Set SEI message parameters read from command line options
  seiFilmGrain->m_filmGrainCharacteristicsCancelFlag      = m_encCfg->m_seiCfg.m_fgcSEICancelFlag;
  seiFilmGrain->m_filmGrainCharacteristicsPersistenceFlag = m_encCfg->m_seiCfg.m_fgcSEIPersistenceFlag;
  seiFilmGrain->m_filmGrainModelId                        = m_encCfg->m_seiCfg.m_fgcSEIModelID;
  seiFilmGrain->m_separateColourDescriptionPresentFlag    = m_encCfg->m_seiCfg.m_fgcSEISepColourDescPresentFlag;
  seiFilmGrain->m_blendingModeId                          = m_encCfg->m_seiCfg.m_fgcSEIBlendingModeID;
  seiFilmGrain->m_log2ScaleFactor                         = m_encCfg->m_seiCfg.m_fgcSEILog2ScaleFactor;
  for (int i = 0; i < MAX_NUM_COMP; i++)
  {
    seiFilmGrain->m_compModel[i].presentFlag = m_encCfg->m_seiCfg.m_fgcSEICompModelPresent[i];
    if (seiFilmGrain->m_compModel[i].presentFlag)
    {
      seiFilmGrain->m_compModel[i].numModelValues        = 1 + m_encCfg->m_seiCfg.m_fgcSEINumModelValuesMinus1[i];
      seiFilmGrain->m_compModel[i].numIntensityIntervals = 1 + m_encCfg->m_seiCfg.m_fgcSEINumIntensityIntervalMinus1[i];
      seiFilmGrain->m_compModel[i].intensityValues.resize(seiFilmGrain->m_compModel[i].numIntensityIntervals);
      for (int j = 0; j < seiFilmGrain->m_compModel[i].numIntensityIntervals; j++)
      {
        seiFilmGrain->m_compModel[i].intensityValues[j].intensityIntervalLowerBound =
          m_encCfg->m_seiCfg.m_fgcSEIIntensityIntervalLowerBound[i][j];
        seiFilmGrain->m_compModel[i].intensityValues[j].intensityIntervalUpperBound =
          m_encCfg->m_seiCfg.m_fgcSEIIntensityIntervalUpperBound[i][j];
        seiFilmGrain->m_compModel[i].intensityValues[j].compModelValue.resize(
          seiFilmGrain->m_compModel[i].numModelValues);
        for (int k = 0; k < seiFilmGrain->m_compModel[i].numModelValues; k++)
        {
          seiFilmGrain->m_compModel[i].intensityValues[j].compModelValue[k] =
            m_encCfg->m_seiCfg.m_fgcSEICompModelValue[i][j][k];
        }
      }
    }
  }
}

void SEIEncoder::initSEIMasteringDisplayColourVolume(SEIMasteringDisplayColourVolume *seiMDCV)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiMDCV != nullptr), "Unspecified error");
  //  Set SEI message parameters read from command line options
  for (int j = 0; j <= 1; j++)
  {
    for (int i = 0; i <= 2; i++)
    {
      seiMDCV->values.primaries[i][j] = m_encCfg->m_seiCfg.m_masteringDisplay.primaries[i][j];
    }
    seiMDCV->values.whitePoint[j] = m_encCfg->m_seiCfg.m_masteringDisplay.whitePoint[j];
  }
  seiMDCV->values.maxLuminance = m_encCfg->m_seiCfg.m_masteringDisplay.maxLuminance;
  seiMDCV->values.minLuminance = m_encCfg->m_seiCfg.m_masteringDisplay.minLuminance;
}

void SEIEncoder::initSEIContentLightLevel(SEIContentLightLevelInfo *seiCLL)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiCLL != nullptr), "Unspecified error");
  //  Set SEI message parameters read from command line options
  seiCLL->m_maxContentLightLevel    = m_encCfg->m_seiCfg.m_cllSEIMaxContentLevel;
  seiCLL->m_maxPicAverageLightLevel = m_encCfg->m_seiCfg.m_cllSEIMaxPicAvgLevel;
}

void SEIEncoder::initSEIAmbientViewingEnvironment(SEIAmbientViewingEnvironment *seiAmbViewEnvironment)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiAmbViewEnvironment != nullptr), "Unspecified error");
  //  Set SEI message parameters read from command line options
  seiAmbViewEnvironment->m_ambientIlluminance = m_encCfg->m_seiCfg.m_aveSEIAmbientIlluminance;
  seiAmbViewEnvironment->m_ambientLightX      = m_encCfg->m_seiCfg.m_aveSEIAmbientLightX;
  seiAmbViewEnvironment->m_ambientLightY      = m_encCfg->m_seiCfg.m_aveSEIAmbientLightY;
}

void SEIEncoder::initSEIContentColourVolume(SEIContentColourVolume *seiContentColourVolume)
{
  assert(m_isInitialized);
  assert(seiContentColourVolume != nullptr);
  seiContentColourVolume->m_ccvCancelFlag      = m_encCfg->m_seiCfg.m_ccvSEICancelFlag;
  seiContentColourVolume->m_ccvPersistenceFlag = m_encCfg->m_seiCfg.m_ccvSEIPersistenceFlag;

  seiContentColourVolume->m_ccvPrimariesPresentFlag         = m_encCfg->m_seiCfg.m_ccvSEIPrimariesPresentFlag;
  seiContentColourVolume->m_ccvMinLuminanceValuePresentFlag = m_encCfg->m_seiCfg.m_ccvSEIMinLuminanceValuePresentFlag;
  seiContentColourVolume->m_ccvMaxLuminanceValuePresentFlag = m_encCfg->m_seiCfg.m_ccvSEIMaxLuminanceValuePresentFlag;
  seiContentColourVolume->m_ccvAvgLuminanceValuePresentFlag = m_encCfg->m_seiCfg.m_ccvSEIAvgLuminanceValuePresentFlag;

  // Currently we are using a floor operation for setting up the "integer" values for this SEI.
  // This applies to both primaries and luminance limits.
  if (seiContentColourVolume->m_ccvPrimariesPresentFlag == true)
  {
    for (int i = 0; i < MAX_NUM_COMP; i++)
    {
      seiContentColourVolume->m_ccvPrimariesX[i] = (int32_t)(50000.0 * m_encCfg->m_seiCfg.m_ccvSEIPrimariesX[i]);
      seiContentColourVolume->m_ccvPrimariesY[i] = (int32_t)(50000.0 * m_encCfg->m_seiCfg.m_ccvSEIPrimariesY[i]);
    }
  }

  if (seiContentColourVolume->m_ccvMinLuminanceValuePresentFlag == true)
  {
    seiContentColourVolume->m_ccvMinLuminanceValue =
      (uint32_t)(10000000 * m_encCfg->m_seiCfg.m_ccvSEIMinLuminanceValue);
  }
  if (seiContentColourVolume->m_ccvMaxLuminanceValuePresentFlag == true)
  {
    seiContentColourVolume->m_ccvMaxLuminanceValue =
      (uint32_t)(10000000 * m_encCfg->m_seiCfg.m_ccvSEIMaxLuminanceValue);
  }
  if (seiContentColourVolume->m_ccvAvgLuminanceValuePresentFlag == true)
  {
    seiContentColourVolume->m_ccvAvgLuminanceValue =
      (uint32_t)(10000000 * m_encCfg->m_seiCfg.m_ccvSEIAvgLuminanceValue);
  }
}

void SEIEncoder::initSEIScalabilityDimensionInfo(SEIScalabilityDimensionInfo *sei)
{
  CHECK(!(m_isInitialized), "Scalability dimension information SEI already initialized");
  CHECK(!(sei != nullptr), "Need a seiScalabilityDimensionInfo for initialization (got nullptr)");
  sei->m_sdiMaxLayersMinus1   = m_encCfg->m_seiCfg.m_sdiSEIMaxLayersMinus1;
  sei->m_sdiMultiviewInfoFlag = m_encCfg->m_seiCfg.m_sdiSEIMultiviewInfoFlag;
  sei->m_sdiAuxiliaryInfoFlag = m_encCfg->m_seiCfg.m_sdiSEIAuxiliaryInfoFlag;
  if (sei->m_sdiMultiviewInfoFlag || sei->m_sdiAuxiliaryInfoFlag)
  {
    if (sei->m_sdiMultiviewInfoFlag)
    {
      sei->m_sdiViewIdLenMinus1 = m_encCfg->m_seiCfg.m_sdiSEIViewIdLenMinus1;
    }
    sei->m_sdiLayerId.resize(sei->m_sdiMaxLayersMinus1 + 1);
    for (int i = 0; i <= sei->m_sdiMaxLayersMinus1; i++)
    {
      sei->m_sdiLayerId[i] = m_encCfg->m_seiCfg.m_sdiSEILayerId[i];
      sei->m_sdiViewIdVal.resize(sei->m_sdiMaxLayersMinus1 + 1);
      if (sei->m_sdiMultiviewInfoFlag)
      {
        sei->m_sdiViewIdVal[i] = m_encCfg->m_seiCfg.m_sdiSEIViewIdVal[i];
      }
      sei->m_sdiAuxId.resize(sei->m_sdiMaxLayersMinus1 + 1);
      if (sei->m_sdiAuxiliaryInfoFlag)
      {
        sei->m_sdiAuxId[i] = m_encCfg->m_seiCfg.m_sdiSEIAuxId[i];
        sei->m_sdiNumAssociatedPrimaryLayersMinus1.resize(sei->m_sdiMaxLayersMinus1 + 1);
        sei->m_sdiAssociatedPrimaryLayerIdx.resize(sei->m_sdiMaxLayersMinus1 + 1);
        if (sei->m_sdiAuxId[i] > 0)
        {
          sei->m_sdiNumAssociatedPrimaryLayersMinus1[i] =
            m_encCfg->m_seiCfg.m_sdiSEINumAssociatedPrimaryLayersMinus1[i];
          sei->m_sdiAssociatedPrimaryLayerIdx[i].resize(sei->m_sdiNumAssociatedPrimaryLayersMinus1[i] + 1);
          for (int j = 0; j <= sei->m_sdiNumAssociatedPrimaryLayersMinus1[i]; j++)
          {
            sei->m_sdiAssociatedPrimaryLayerIdx[i][j] = 0;
          }
        }
      }
    }
    sei->m_sdiNumViews = 1;
    if (sei->m_sdiMultiviewInfoFlag)
    {
      for (int i = 1; i <= sei->m_sdiMaxLayersMinus1; i++)
      {
        bool newViewFlag = true;
        for (int j = 0; j < i; j++)
        {
          if (sei->m_sdiViewIdVal[i] == sei->m_sdiViewIdVal[j])
          {
            newViewFlag = false;
          }
        }
        if (newViewFlag)
        {
          sei->m_sdiNumViews++;
        }
      }
    }
  }
}

void SEIEncoder::initSEIMultiviewAcquisitionInfo(SEIMultiviewAcquisitionInfo *sei)
{
  CHECK(!(m_isInitialized), "Multiview acquisition information SEI already initialized");
  CHECK(!(sei != nullptr), "Need a seiMultiviewAcquisitionInfo for initialization (got nullptr)");
  sei->m_maiIntrinsicParamFlag = m_encCfg->m_seiCfg.m_maiSEIIntrinsicParamFlag;
  sei->m_maiExtrinsicParamFlag = m_encCfg->m_seiCfg.m_maiSEIExtrinsicParamFlag;
  sei->m_maiNumViewsMinus1     = m_encCfg->m_seiCfg.m_maiSEINumViewsMinus1;
  if (sei->m_maiIntrinsicParamFlag)
  {
    sei->m_maiIntrinsicParamsEqualFlag = m_encCfg->m_seiCfg.m_maiSEIIntrinsicParamsEqualFlag;
    sei->m_maiPrecFocalLength          = m_encCfg->m_seiCfg.m_maiSEIPrecFocalLength;
    sei->m_maiPrecPrincipalPoint       = m_encCfg->m_seiCfg.m_maiSEIPrecPrincipalPoint;
    sei->m_maiPrecSkewFactor           = m_encCfg->m_seiCfg.m_maiSEIPrecSkewFactor;
    int numViews                       = sei->m_maiIntrinsicParamsEqualFlag ? 1 : sei->m_maiNumViewsMinus1 + 1;
    sei->m_maiSignFocalLengthX.resize(numViews);
    sei->m_maiExponentFocalLengthX.resize(numViews);
    sei->m_maiMantissaFocalLengthX.resize(numViews);
    sei->m_maiSignFocalLengthY.resize(numViews);
    sei->m_maiExponentFocalLengthY.resize(numViews);
    sei->m_maiMantissaFocalLengthY.resize(numViews);
    sei->m_maiSignPrincipalPointX.resize(numViews);
    sei->m_maiExponentPrincipalPointX.resize(numViews);
    sei->m_maiMantissaPrincipalPointX.resize(numViews);
    sei->m_maiSignPrincipalPointY.resize(numViews);
    sei->m_maiExponentPrincipalPointY.resize(numViews);
    sei->m_maiMantissaPrincipalPointY.resize(numViews);
    sei->m_maiSignSkewFactor.resize(numViews);
    sei->m_maiExponentSkewFactor.resize(numViews);
    sei->m_maiMantissaSkewFactor.resize(numViews);
    for (int i = 0; i <= (sei->m_maiIntrinsicParamsEqualFlag ? 0 : sei->m_maiNumViewsMinus1); i++)
    {
      sei->m_maiSignFocalLengthX[i]        = m_encCfg->m_seiCfg.m_maiSEISignFocalLengthX[i];
      sei->m_maiExponentFocalLengthX[i]    = m_encCfg->m_seiCfg.m_maiSEIExponentFocalLengthX[i];
      sei->m_maiMantissaFocalLengthX[i]    = m_encCfg->m_seiCfg.m_maiSEIMantissaFocalLengthX[i];
      sei->m_maiSignFocalLengthY[i]        = m_encCfg->m_seiCfg.m_maiSEISignFocalLengthY[i];
      sei->m_maiExponentFocalLengthY[i]    = m_encCfg->m_seiCfg.m_maiSEIExponentFocalLengthY[i];
      sei->m_maiMantissaFocalLengthY[i]    = m_encCfg->m_seiCfg.m_maiSEIMantissaFocalLengthY[i];
      sei->m_maiSignPrincipalPointX[i]     = m_encCfg->m_seiCfg.m_maiSEISignPrincipalPointX[i];
      sei->m_maiExponentPrincipalPointX[i] = m_encCfg->m_seiCfg.m_maiSEIExponentPrincipalPointX[i];
      sei->m_maiMantissaPrincipalPointX[i] = m_encCfg->m_seiCfg.m_maiSEIMantissaPrincipalPointX[i];
      sei->m_maiSignPrincipalPointY[i]     = m_encCfg->m_seiCfg.m_maiSEISignPrincipalPointY[i];
      sei->m_maiExponentPrincipalPointY[i] = m_encCfg->m_seiCfg.m_maiSEIExponentPrincipalPointY[i];
      sei->m_maiMantissaPrincipalPointY[i] = m_encCfg->m_seiCfg.m_maiSEIMantissaPrincipalPointY[i];
      sei->m_maiSignSkewFactor[i]          = m_encCfg->m_seiCfg.m_maiSEISignSkewFactor[i];
      sei->m_maiExponentSkewFactor[i]      = m_encCfg->m_seiCfg.m_maiSEIExponentSkewFactor[i];
      sei->m_maiMantissaSkewFactor[i]      = m_encCfg->m_seiCfg.m_maiSEIMantissaSkewFactor[i];
    }
  }
  if (sei->m_maiExtrinsicParamFlag)
  {
    sei->m_maiPrecRotationParam    = m_encCfg->m_seiCfg.m_maiSEIPrecRotationParam;
    sei->m_maiPrecTranslationParam = m_encCfg->m_seiCfg.m_maiSEIPrecTranslationParam;
    sei->m_maiSignR.resize(sei->m_maiNumViewsMinus1 + 1);
    sei->m_maiExponentR.resize(sei->m_maiNumViewsMinus1 + 1);
    sei->m_maiMantissaR.resize(sei->m_maiNumViewsMinus1 + 1);
    sei->m_maiSignT.resize(sei->m_maiNumViewsMinus1 + 1);
    sei->m_maiExponentT.resize(sei->m_maiNumViewsMinus1 + 1);
    sei->m_maiMantissaT.resize(sei->m_maiNumViewsMinus1 + 1);
    for (int i = 0; i <= sei->m_maiNumViewsMinus1; i++)
    {
      sei->m_maiSignR[i].resize(3);
      sei->m_maiExponentR[i].resize(3);
      sei->m_maiMantissaR[i].resize(3);
      sei->m_maiSignT[i].resize(3);
      sei->m_maiExponentT[i].resize(3);
      sei->m_maiMantissaT[i].resize(3);
      for (int j = 0; j < 3; j++)
      {
        sei->m_maiSignR[i][j].resize(3);
        sei->m_maiExponentR[i][j].resize(3);
        sei->m_maiMantissaR[i][j].resize(3);
        for (int k = 0; k < 3; k++)
        {
          sei->m_maiSignR[i][j][k]     = 0;
          sei->m_maiExponentR[i][j][k] = 0;
          sei->m_maiMantissaR[i][j][k] = 0;
        }
        sei->m_maiSignT[i][j]     = 0;
        sei->m_maiExponentT[i][j] = 0;
        sei->m_maiMantissaT[i][j] = 0;
      }
    }
  }
}

void SEIEncoder::initSEIMultiviewViewPosition(SEIMultiviewViewPosition *sei)
{
  CHECK(!(m_isInitialized), "Multiview view position SEI already initialized");
  CHECK(!(sei != nullptr), "Need a seiMultiviewViewPosition for initialization (got nullptr)");
  sei->m_mvpNumViewsMinus1 = m_encCfg->m_seiCfg.m_mvpSEINumViewsMinus1;

  int numViews = sei->m_mvpNumViewsMinus1 + 1;
  sei->m_mvpViewPosition.resize(numViews);
  for (int i = 0; i <= sei->m_mvpNumViewsMinus1; i++)
  {
    sei->m_mvpViewPosition[i] = m_encCfg->m_seiCfg.m_mvpSEIViewPosition[i];
  }
}

void SEIEncoder::initSEIAlphaChannelInfo(SEIAlphaChannelInfo *sei)
{
  CHECK(!(m_isInitialized), "Alpha channel information SEI already initialized");
  CHECK(!(sei != nullptr), "Need a seiAlphaChannelInfo for initialization (got nullptr)");
  sei->m_aciCancelFlag       = m_encCfg->m_seiCfg.m_aciSEICancelFlag;
  sei->m_aciUseIdc           = m_encCfg->m_seiCfg.m_aciSEIUseIdc;
  sei->m_aciBitDepthMinus8   = m_encCfg->m_seiCfg.m_aciSEIBitDepthMinus8;
  sei->m_aciTransparentValue = m_encCfg->m_seiCfg.m_aciSEITransparentValue;
  sei->m_aciOpaqueValue      = m_encCfg->m_seiCfg.m_aciSEIOpaqueValue;
  sei->m_aciIncrFlag         = m_encCfg->m_seiCfg.m_aciSEIIncrFlag;
  sei->m_aciClipFlag         = m_encCfg->m_seiCfg.m_aciSEIClipFlag;
  sei->m_aciClipTypeFlag     = m_encCfg->m_seiCfg.m_aciSEIClipTypeFlag;
}

void SEIEncoder::initSEIDepthRepresentationInfo(SEIDepthRepresentationInfo *sei)
{
  CHECK(!(m_isInitialized), "Depth representation information SEI already initialized");
  CHECK(!(sei != nullptr), "Need a seiDepthRepresentationInfo for initialization (got nullptr)");
  sei->m_driZNearFlag                             = m_encCfg->m_seiCfg.m_driSEIZNearFlag;
  sei->m_driZFarFlag                              = m_encCfg->m_seiCfg.m_driSEIZFarFlag;
  sei->m_driDMinFlag                              = m_encCfg->m_seiCfg.m_driSEIDMinFlag;
  sei->m_driDMaxFlag                              = m_encCfg->m_seiCfg.m_driSEIDMaxFlag;
  sei->m_driZNear                                 = m_encCfg->m_seiCfg.m_driSEIZNear;
  sei->m_driZFar                                  = m_encCfg->m_seiCfg.m_driSEIZFar;
  sei->m_driDMin                                  = m_encCfg->m_seiCfg.m_driSEIDMin;
  sei->m_driDMax                                  = m_encCfg->m_seiCfg.m_driSEIDMax;
  sei->m_driDisparityRefViewId                    = m_encCfg->m_seiCfg.m_driSEIDisparityRefViewId;
  sei->m_driDepthRepresentationType               = m_encCfg->m_seiCfg.m_driSEIDepthRepresentationType;
  sei->m_driDepthNonlinearRepresentationNumMinus1 = m_encCfg->m_seiCfg.m_driSEINonlinearNumMinus1;
  sei->m_driDepthNonlinearRepresentationModel.resize(sei->m_driDepthNonlinearRepresentationNumMinus1 + 1);
  for (int i = 0; i < (sei->m_driDepthNonlinearRepresentationNumMinus1 + 1); i++)
  {
    sei->m_driDepthNonlinearRepresentationModel[i] = m_encCfg->m_seiCfg.m_driSEINonlinearModel[i];
  }
}

void SEIEncoder::initSEIColourTransformInfo(SEIColourTransformInfo *seiCTI)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiCTI != nullptr), "Unspecified error");

  //  Set SEI message parameters read from command line options
  seiCTI->m_id                     = m_encCfg->m_seiCfg.m_ctiSEIId;
  seiCTI->m_signalInfoFlag         = m_encCfg->m_seiCfg.m_ctiSEISignalInfoFlag;
  seiCTI->m_fullRangeFlag          = m_encCfg->m_seiCfg.m_ctiSEIFullRangeFlag;
  seiCTI->m_primaries              = m_encCfg->m_seiCfg.m_ctiSEIPrimaries;
  seiCTI->m_transferFunction       = m_encCfg->m_seiCfg.m_ctiSEITransferFunction;
  seiCTI->m_matrixCoefs            = m_encCfg->m_seiCfg.m_ctiSEIMatrixCoefs;
  seiCTI->m_crossComponentFlag     = m_encCfg->m_seiCfg.m_ctiSEICrossComponentFlag;
  seiCTI->m_crossComponentInferred = m_encCfg->m_seiCfg.m_ctiSEICrossComponentInferred;
  seiCTI->m_numberChromaLutMinus1  = m_encCfg->m_seiCfg.m_ctiSEINumberChromaLut - 1;
  seiCTI->m_chromaOffset           = m_encCfg->m_seiCfg.m_ctiSEIChromaOffset;

  seiCTI->m_bitdepth = m_encCfg->m_internalBitDepth[ChannelType::LUMA];

  for (int i = 0; i < MAX_NUM_COMP; i++)
  {
    seiCTI->m_lut[i] = m_encCfg->m_seiCfg.m_ctiSEILut[i];
  }
  seiCTI->m_log2NumberOfPointsPerLut = floorLog2(seiCTI->m_lut[0].numLutValues - 1);
}

void SEIEncoder::initSEISubpictureLevelInfo(SEISubpicureLevelInfo *sei, const SPS *sps)
{
  const CfgSEISubpictureLevel &cfgSubPicLevel = m_encCfg->m_seiCfg.m_cfgSubpictureLevelInfoSEI;

  sei->m_sliSublayerInfoPresentFlag  = cfgSubPicLevel.m_sliSublayerInfoPresentFlag;
  sei->m_sliMaxSublayers             = cfgSubPicLevel.m_sliMaxSublayers;
  sei->m_numRefLevels                = cfgSubPicLevel.m_sliSublayerInfoPresentFlag
                   ? (int)cfgSubPicLevel.m_refLevels.size() / cfgSubPicLevel.m_sliMaxSublayers
                   : (int)cfgSubPicLevel.m_refLevels.size();
  sei->m_numSubpics                  = cfgSubPicLevel.m_numSubpictures;
  sei->m_explicitFractionPresentFlag = cfgSubPicLevel.m_explicitFraction;

  // sei parameters initialization
  sei->m_nonSubpicLayersFraction.resize(sei->m_numRefLevels);
  sei->m_refLevelIdc.resize(sei->m_numRefLevels);
  for (int level = 0; level < sei->m_numRefLevels; level++)
  {
    sei->m_nonSubpicLayersFraction[level].resize(sei->m_sliMaxSublayers);
    sei->m_refLevelIdc[level].resize(sei->m_sliMaxSublayers);
    for (int sublayer = 0; sublayer < sei->m_sliMaxSublayers; sublayer++)
    {
      sei->m_refLevelIdc[level][sublayer] = Level::LEVEL15_5;
    }
  }
  if (sei->m_explicitFractionPresentFlag)
  {
    sei->m_refLevelFraction.resize(sei->m_numRefLevels);
    for (int level = 0; level < sei->m_numRefLevels; level++)
    {
      sei->m_refLevelFraction[level].resize(sei->m_numSubpics);
      for (int subpic = 0; subpic < sei->m_numSubpics; subpic++)
      {
        sei->m_refLevelFraction[level][subpic].resize(sei->m_sliMaxSublayers);
        for (int sublayer = 0; sublayer < sei->m_sliMaxSublayers; sublayer++)
        {
          sei->m_refLevelFraction[level][subpic][sublayer] = 0;
        }
      }
    }
  }

  // set sei parameters according to the configured values
  for (int sublayer = sei->m_sliSublayerInfoPresentFlag ? 0 : sei->m_sliMaxSublayers - 1, cnta = 0, cntb = 0;
       sublayer < sei->m_sliMaxSublayers; sublayer++)
  {
    for (int level = 0; level < sei->m_numRefLevels; level++)
    {
      sei->m_nonSubpicLayersFraction[level][sublayer] = cfgSubPicLevel.m_nonSubpicLayersFraction[cnta];
      sei->m_refLevelIdc[level][sublayer]             = cfgSubPicLevel.m_refLevels[cnta++];
      if (sei->m_explicitFractionPresentFlag)
      {
        for (int subpic = 0; subpic < sei->m_numSubpics; subpic++)
        {
          sei->m_refLevelFraction[level][subpic][sublayer] = cfgSubPicLevel.m_fractions[cntb++];
        }
      }
    }
  }

  // update the inference of m_refLevelIdc[][] and m_refLevelFraction[][][]
  if (!sei->m_sliSublayerInfoPresentFlag)
  {
    for (int sublayer = sei->m_sliMaxSublayers - 2; sublayer >= 0; sublayer--)
    {
      for (int level = 0; level < sei->m_numRefLevels; level++)
      {
        sei->m_nonSubpicLayersFraction[level][sublayer] =
          sei->m_nonSubpicLayersFraction[level][sei->m_sliMaxSublayers - 1];
        sei->m_refLevelIdc[level][sublayer] = sei->m_refLevelIdc[level][sei->m_sliMaxSublayers - 1];
        if (sei->m_explicitFractionPresentFlag)
        {
          for (int subpic = 0; subpic < sei->m_numSubpics; subpic++)
          {
            sei->m_refLevelFraction[level][subpic][sublayer] =
              sei->m_refLevelFraction[level][subpic][sei->m_sliMaxSublayers - 1];
          }
        }
      }
    }
  }
}

void SEIEncoder::initSEISEIManifest(SEIManifest *seiSeiManifest, const SEIMessages &seiMessages)
{
  assert(m_isInitialized);
  assert(seiSeiManifest != NULL);
  seiSeiManifest->m_manifestNumSeiMsgTypes = 0;
  for (auto &it: seiMessages)
  {
    seiSeiManifest->m_manifestNumSeiMsgTypes += 1;
    auto tempPayloadType = it->payloadType();
    seiSeiManifest->m_manifestSeiPayloadType.push_back(tempPayloadType);
    auto description = seiSeiManifest->getSEIMessageDescription(tempPayloadType);
    seiSeiManifest->m_manifestSeiDescription.push_back(description);
  }
  CHECK(seiSeiManifest->m_manifestNumSeiMsgTypes == 0, "No SEI messages available");
}

void SEIEncoder::initSEISEIPrefixIndication(SEIPrefixIndication *seiSeiPrefixIndications, const SEI *sei)
{
  assert(m_isInitialized);
  assert(seiSeiPrefixIndications != NULL);
  seiSeiPrefixIndications->m_prefixSeiPayloadType = sei->payloadType();
  seiSeiPrefixIndications->m_numSeiPrefixIndicationsMinus1 =
    seiSeiPrefixIndications->getNumsOfSeiPrefixIndications(sei) - 1;
  seiSeiPrefixIndications->m_payload = sei;
}

void SEIEncoder::initSEINeuralNetworkPostFilterCharacteristics(SEINeuralNetworkPostFilterCharacteristics *sei,
                                                               int                                        filterIdx)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(sei != nullptr), "Unspecified error");
  sei->m_purpose = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsPurpose[filterIdx];
  sei->m_id      = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsId[filterIdx];
  sei->m_modeIdc = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsModeIdc[filterIdx];
  if (sei->m_modeIdc == POST_FILTER_MODE::URI)
  {
    sei->m_uriTag = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsUriTag[filterIdx];
    sei->m_uri    = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsUri[filterIdx];
  }
  sei->m_propertyPresentFlag = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsPropertyPresentFlag[filterIdx];
  if (sei->m_propertyPresentFlag)
  {
    sei->m_baseFlag = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsBaseFlag[filterIdx];

    sei->m_numberInputDecodedPicturesMinus1 =
      m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsNumberInputDecodedPicturesMinus1[filterIdx];
    CHECK(sei->m_numberInputDecodedPicturesMinus1 > 63,
          "m_numberInputDecodedPicturesMinus1 shall be in the range of 0 to 63");

    if ((sei->m_purpose & NNPC_PurposeType::CHROMA_UPSAMPLING) != 0)
    {
      sei->m_outSubCFlag = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsOutSubCFlag[filterIdx];
    }
    if ((sei->m_purpose & NNPC_PurposeType::COLOURIZATION) != 0)
    {
      sei->m_outColourFormatIdc = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsOutColourFormatIdc[filterIdx];
    }
    if ((sei->m_purpose & NNPC_PurposeType::RESOLUTION_UPSAMPLING) != 0)
    {
      sei->m_picWidthInLumaSamples =
        m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsPicWidthInLumaSamples[filterIdx];
      sei->m_picHeightInLumaSamples =
        m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsPicHeightInLumaSamples[filterIdx];
      int confWinLeftOffset        = m_pcEncLib->getPPS(0)->m_conformanceWindow.m_winLeftOffset;
      int confWinTopOffset         = m_pcEncLib->getPPS(0)->m_conformanceWindow.m_winTopOffset;
      int confWinRightOffset       = m_pcEncLib->getPPS(0)->m_conformanceWindow.m_winRightOffset;
      int confWinBottomOffset      = m_pcEncLib->getPPS(0)->m_conformanceWindow.m_winBottomOffset;
      int ppsPicWidthInLumaSample  = m_pcEncLib->getPPS(0)->m_picWidthInLumaSamples;
      int ppsPicHeightInLumaSample = m_pcEncLib->getPPS(0)->m_picHeightInLumaSamples;

      const ChromaFormat chromaFormatIdc = m_pcEncLib->getSPS(0)->m_chromaFormatIdc;
      uint8_t            subWidthC;
      uint8_t            subHeightC;
      if (chromaFormatIdc == ChromaFormat::_420)
      {
        subWidthC  = 2;
        subHeightC = 2;
      }
      else if (chromaFormatIdc == ChromaFormat::_422)
      {
        subWidthC  = 2;
        subHeightC = 1;
      }
      else
      {
        subWidthC  = 1;
        subHeightC = 1;
      }

      int croppedWidth  = ppsPicWidthInLumaSample - subWidthC * (confWinRightOffset + confWinLeftOffset);
      int croppedHeight = ppsPicHeightInLumaSample - subHeightC * (confWinBottomOffset + confWinTopOffset);
      CHECK(!(sei->m_picWidthInLumaSamples >= croppedWidth && sei->m_picWidthInLumaSamples <= croppedWidth * 16 - 1),
            "m_picWidthInLumaSamples shall be in the range of croppedWidth to croppedWidth * 16 - 1");
      CHECK(
        !(sei->m_picHeightInLumaSamples >= croppedHeight && sei->m_picHeightInLumaSamples <= croppedHeight * 16 - 1),
        "m_picHeightInLumaSamples shall be in the range of croppedHeight to croppedHeight * 16 - 1");
    }
    if ((sei->m_purpose & NNPC_PurposeType::FRAME_RATE_UPSAMPLING) != 0)
    {
      sei->m_numberInterpolatedPictures =
        m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsNumberInterpolatedPictures[filterIdx];
      sei->m_inputPicOutputFlag = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsInputPicOutputFlag[filterIdx];
      CHECK(sei->m_numberInputDecodedPicturesMinus1 <= 0,
            "If nnpfc_purpose is FRAME_RATE_UPSAMPLING, m_numberInputDecodedPicturesMinus1 shall be greater than 0");
    }

    sei->m_componentLastFlag = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsComponentLastFlag[filterIdx];
    sei->m_inpFormatIdc      = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsInpFormatIdc[filterIdx];
    if (sei->m_inpFormatIdc == 1)
    {
      sei->m_inpTensorBitDepthLumaMinus8 =
        m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsInpTensorBitDepthLumaMinus8[filterIdx];
      sei->m_inpTensorBitDepthChromaMinus8 =
        m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsInpTensorBitDepthChromaMinus8[filterIdx];
    }

    sei->m_inpOrderIdc           = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsInpOrderIdc[filterIdx];
    sei->m_auxInpIdc             = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsAuxInpIdc[filterIdx];
    sei->m_sepColDescriptionFlag = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsSepColDescriptionFlag[filterIdx];
    if (sei->m_sepColDescriptionFlag)
    {
      sei->m_colPrimaries         = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsColPrimaries[filterIdx];
      sei->m_transCharacteristics = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsTransCharacteristics[filterIdx];
      sei->m_matrixCoeffs         = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsMatrixCoeffs[filterIdx];
    }

    sei->m_outFormatIdc = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsOutFormatIdc[filterIdx];
    if (sei->m_outFormatIdc == 1)
    {
      sei->m_outTensorBitDepthLumaMinus8 =
        m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsOutTensorBitDepthLumaMinus8[filterIdx];
      sei->m_outTensorBitDepthChromaMinus8 =
        m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsOutTensorBitDepthChromaMinus8[filterIdx];
    }
    sei->m_outOrderIdc           = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsOutOrderIdc[filterIdx];
    sei->m_constantPatchSizeFlag = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsConstantPatchSizeFlag[filterIdx];
    sei->m_patchWidthMinus1      = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsPatchWidthMinus1[filterIdx];
    sei->m_patchHeightMinus1     = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsPatchHeightMinus1[filterIdx];
    if (sei->m_constantPatchSizeFlag == 0)
    {
      sei->m_extendedPatchWidthCdDeltaMinus1 =
        m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsExtendedPatchWidthCdDeltaMinus1[filterIdx];
      sei->m_extendedPatchHeightCdDeltaMinus1 =
        m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsExtendedPatchHeightCdDeltaMinus1[filterIdx];
    }
    sei->m_overlap     = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsOverlap[filterIdx];
    sei->m_paddingType = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsPaddingType[filterIdx];
    sei->m_lumaPadding = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsLumaPadding[filterIdx];
    sei->m_cbPadding   = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsCbPadding[filterIdx];
    sei->m_crPadding   = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsCrPadding[filterIdx];

    sei->m_complexityInfoPresentFlag =
      m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsComplexityInfoPresentFlag[filterIdx];
    if (sei->m_complexityInfoPresentFlag)
    {
      sei->m_parameterTypeIdc = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsParameterTypeIdc[filterIdx];
      sei->m_log2ParameterBitLengthMinus3 =
        m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsLog2ParameterBitLengthMinus3[filterIdx];
      sei->m_numParametersIdc     = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsNumParametersIdc[filterIdx];
      sei->m_numKmacOperationsIdc = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsNumKmacOperationsIdc[filterIdx];
      sei->m_totalKilobyteSize    = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsTotalKilobyteSize[filterIdx];
    }
  }
  if (sei->m_modeIdc == POST_FILTER_MODE::ISO_IEC_15938_17)
  {
    const std::string payloadFilename = m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsPayloadFilename[filterIdx];
    std::ifstream     bitstreamFile(payloadFilename.c_str(), std::ifstream::in | std::ifstream::binary);
    if (!bitstreamFile)
    {
      EXIT("Failed to open bitstream file " << payloadFilename.c_str() << " for reading");
    }

    bitstreamFile.seekg(0, std::ifstream::end);
    sei->m_payloadLength = bitstreamFile.tellg();
    bitstreamFile.seekg(0, std::ifstream::beg);

    sei->m_payloadByte = new char[sei->m_payloadLength];
    bitstreamFile.read(sei->m_payloadByte, sei->m_payloadLength);
    bitstreamFile.close();
  }
}

void SEIEncoder::initSEINeuralNetworkPostFilterActivation(SEINeuralNetworkPostFilterActivation *sei)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(sei != nullptr), "Unspecified error");
  sei->m_targetId   = m_encCfg->m_seiCfg.m_nnPostFilterSEIActivationTargetId;
  sei->m_cancelFlag = m_encCfg->m_seiCfg.m_nnPostFilterSEIActivationCancelFlag;
  if (!sei->m_cancelFlag)
  {
    sei->m_persistenceFlag = m_encCfg->m_seiCfg.m_nnPostFilterSEIActivationPersistenceFlag;
  }
}

//! \}
