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

/** \file     EncApp.cpp
    \brief    Encoder application class
*/

#include <list>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <iomanip>

#include "EncApp.h"
#include "EncAppCfg.h"
#include "CommonLib/EipLog.h"
#include "EncoderLib/AnnexBwrite.h"
#include "EncoderLib/EncLibCommon.h"

//! \ingroup EncoderApp
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

EncApp::EncApp(std::fstream &bitStream, EncLibCommon *encLibCommon) : m_cEncLib(encLibCommon), m_bitstream(bitStream)
{
  m_frameRcvd      = 0;
  m_totalBytes     = 0;
  m_essentialBytes = 0;
#if JVET_O0756_CALCULATE_HDRMETRICS
  m_metricTime = std::chrono::milliseconds(0);
#endif
  m_numEncoded = 0;
  m_flush      = false;
}

EncApp::~EncApp()
{
  EipLog::finishEncoder();
#if ENABLE_TRACING
  tracing_uninit(g_trace_ctx);
  g_trace_ctx = nullptr;
#endif
}

void EncApp::xInitLibCfg(int layerIdx)
{
  EncCfg *encCfg = &m_cEncLib.m_encCfg;

  VPS &vps = *m_cEncLib.m_vps;
  if (encCfg->m_targetOlsIdx != 500)
  {
    vps.m_targetOlsIdx = encCfg->m_targetOlsIdx;
  }
  else
  {
    vps.m_targetOlsIdx = -1;
  }

  vps.m_maxLayers = encCfg->m_maxLayers;

  if (vps.m_maxLayers > 1)
  {
    vps.m_vpsId = 1;  // JVET_P0205 vps_video_parameter_set_id shall be greater than 0 for multi-layer coding
  }
  else
  {
    vps.m_vpsId                   = 0;
    vps.m_vpsEachLayerIsAnOlsFlag = 1; // If vps_max_layers_minus1 is equal to 0,
                                         // the value of vps_each_layer_is_an_ols_flag is inferred to be equal to 1.
                                         // Otherwise, when vps_all_independent_layers_flag is equal to 0,
                                         // the value of vps_each_layer_is_an_ols_flag is inferred to be equal to 0.
  }
  vps.m_vpsMaxSubLayers = encCfg->m_maxSublayers;
  if (vps.m_maxLayers > 1 && vps.m_vpsMaxSubLayers > 1)
  {
    vps.m_vpsDefaultPtlDpbHrdMaxTidFlag = encCfg->m_defaultPtlDpbHrdMaxTidFlag;
  }
  if (vps.m_maxLayers > 1)
  {
    vps.m_vpsAllIndependentLayersFlag = encCfg->m_allIndependentLayersFlag;
    if (!vps.m_vpsAllIndependentLayersFlag)
    {
      vps.m_vpsEachLayerIsAnOlsFlag = 0;
      for (int i = 0; i < encCfg->m_maxTempLayer; i++)
      {
        vps.m_vpsCfgPredDirection[i] = 0;
      }
      for (int i = 0; i < encCfg->m_predDirectionArray.size(); i++)
      {
        if (encCfg->m_predDirectionArray[i] != ' ')
        {
          vps.m_vpsCfgPredDirection[i >> 1] = int(encCfg->m_predDirectionArray[i] - 48);
        }
      }
    }
  }

  encCfg->m_maxTidILRefPicsPlus1.resize(vps.m_maxLayers, std::vector<uint32_t>(vps.m_maxLayers, MAX_TLAYER));
  for (int i = 0; i < vps.m_maxLayers; i++)
  {
    vps.m_generalLayerIdx[encCfg->m_layerId[i]] = i;
    vps.m_vpsLayerId[i]                         = encCfg->m_layerId[i];

    if (i > 0 && !vps.m_vpsAllIndependentLayersFlag)
    {
      vps.m_vpsIndependentLayerFlag[i] = encCfg->m_numRefLayers[i] ? false : true;

      if (!vps.m_vpsIndependentLayerFlag[i])
      {
        for (int j = 0, k = 0; j < i; j++)
        {
          if (encCfg->m_refLayerIdxStr[i].find(std::to_string(j)) != std::string::npos)
          {
            vps.m_vpsDirectRefLayerFlag[i][j] = true;
            vps.m_interLayerRefIdx[i][j]      = k;
            vps.m_directRefLayerIdx[i][k++]   = j;
          }
          else
          {
            vps.m_vpsDirectRefLayerFlag[i][j] = false;
          }
        }
        std::string::size_type beginStr = encCfg->m_maxTidILRefPicsPlus1Str[i].find_first_not_of(" ", 0);
        std::string::size_type endStr   = encCfg->m_maxTidILRefPicsPlus1Str[i].find_first_of(" ", beginStr);
        int                    t        = 0;
        while (std::string::npos != beginStr || std::string::npos != endStr)
        {
          encCfg->m_maxTidILRefPicsPlus1[i][t++] =
            std::stoi(encCfg->m_maxTidILRefPicsPlus1Str[i].substr(beginStr, endStr - beginStr));
          beginStr = encCfg->m_maxTidILRefPicsPlus1Str[i].find_first_not_of(" ", endStr);
          endStr   = encCfg->m_maxTidILRefPicsPlus1Str[i].find_first_of(" ", beginStr);
        }
      }
    }
  }

  if (vps.m_maxLayers > 1)
  {
    if (vps.m_vpsAllIndependentLayersFlag)
    {
      vps.m_vpsEachLayerIsAnOlsFlag = encCfg->m_eachLayerIsAnOlsFlag;
      if (vps.m_vpsEachLayerIsAnOlsFlag == 0)
      {
        vps.m_vpsOlsModeIdc =
          2;   // When vps_all_independent_layers_flag is equal to 1 and vps_each_layer_is_an_ols_flag is equal to 0,
               // the value of vps_ols_mode_idc is inferred to be equal to 2
      }
    }
    if (!vps.m_vpsEachLayerIsAnOlsFlag)
    {
      if (!vps.m_vpsAllIndependentLayersFlag)
      {
        vps.m_vpsOlsModeIdc = encCfg->m_olsModeIdc;
      }
      if (vps.m_vpsOlsModeIdc == 2)
      {
        vps.m_vpsNumOutputLayerSets = encCfg->m_numOutputLayerSets;
        for (int i = 1; i < vps.m_vpsNumOutputLayerSets; i++)
        {
          for (int j = 0; j < vps.m_maxLayers; j++)
          {
            if (encCfg->m_olsOutputLayerStr[i].find(std::to_string(j)) != std::string::npos)
            {
              vps.m_vpsOlsOutputLayerFlag[i][j] = 1;
            }
            else
            {
              vps.m_vpsOlsOutputLayerFlag[i][j] = 0;
            }
          }
        }
      }
    }
  }
  CHECK(encCfg->m_numPtlsInVps == 0, "There has to be at least one PTL structure in the VPS.");
  vps.setNumPtls(encCfg->m_numPtlsInVps);
  vps.m_ptPresentFlag[0] = true;
  for (int i = 0; i < vps.getNumPtls(); i++)
  {
    if (i > 0)
    {
      vps.m_ptPresentFlag[i] = encCfg->m_ptPresentInPtl[i] != 0;
    }
    vps.m_ptlMaxTemporalId[i] = vps.m_vpsMaxSubLayers - 1;
  }
  for (int i = 0; i < vps.m_vpsNumOutputLayerSets; i++)
  {
    vps.m_olsPtlIdx[i] = encCfg->m_olsPtlIdx[i];
  }

  ProfileTierLevel ptl;

  ptl.m_levelIdc                = encCfg->m_level;
  ptl.m_profileIdc              = encCfg->m_profile;
  ptl.m_tierFlag                = encCfg->m_tier;
  ptl.m_frameOnlyConstraintFlag = encCfg->m_frameOnlyConstraintFlag;

  CHECK(encCfg->m_numRefLayers[layerIdx] > 0 && !encCfg->m_multiLayerEnabledFlag,
        "ptl_multilayer_enabled_flag shall be equal to 1 when target layer use inter layer prediction");
  ptl.m_multiLayerEnabledFlag = encCfg->m_multiLayerEnabledFlag;
  CHECK((encCfg->m_profile == Profile::MAIN_10 || encCfg->m_profile == Profile::MAIN_10_444 ||
         encCfg->m_profile == Profile::MAIN_10_STILL_PICTURE ||
         encCfg->m_profile == Profile::MAIN_10_444_STILL_PICTURE || encCfg->m_profile == Profile::MAIN_12 ||
         encCfg->m_profile == Profile::MAIN_12_INTRA || encCfg->m_profile == Profile::MAIN_12_STILL_PICTURE ||
         encCfg->m_profile == Profile::MAIN_12_444 || encCfg->m_profile == Profile::MAIN_12_444_INTRA ||
         encCfg->m_profile == Profile::MAIN_12_444_STILL_PICTURE || encCfg->m_profile == Profile::MAIN_16_444 ||
         encCfg->m_profile == Profile::MAIN_16_444_INTRA || encCfg->m_profile == Profile::MAIN_16_444_STILL_PICTURE) &&
          encCfg->m_multiLayerEnabledFlag,
        "ptl_multilayer_enabled_flag shall be equal to 0 for non-multilayer profiles");
  ptl.m_subProfileIdc.resize(encCfg->m_numSubProfile);
  for (int i = 0; i < encCfg->m_numSubProfile; i++)
  {
    ptl.m_subProfileIdc[i] = encCfg->m_subProfile[i];
  }
  if (0 < layerIdx)
  {
    ptl.m_levelIdc = (encCfg->m_levelPtl[layerIdx]);
  }
  CHECK(layerIdx >= vps.getNumPtls(),
        "Insufficient number of Profile/Tier/Level entries in VPS. Consider increasing NumPTLsInVPS");
  vps.m_vpsProfileTierLevel[layerIdx] = ptl;
  vps.m_vpsExtensionFlag              = false;

  encCfg->m_conformanceWindow.setWindow(encCfg->m_confWinLeft / SPS::getWinUnitX(encCfg->m_inputChromaFormatIDC),
                                        encCfg->m_confWinRight / SPS::getWinUnitX(encCfg->m_inputChromaFormatIDC),
                                        encCfg->m_confWinTop / SPS::getWinUnitY(encCfg->m_inputChromaFormatIDC),
                                        encCfg->m_confWinBottom / SPS::getWinUnitY(encCfg->m_inputChromaFormatIDC));

  m_cEncLib.setRefLayerRescaledAvailable(false);

  //====== SPS constraint flags =======
  if (encCfg->m_gciPresentFlag)
  {
    CHECK(encCfg->m_noIdrRplConstraintFlag && encCfg->m_idrRefParamList,
          "IDR RPL shall be deactivated when gci_no_idr_rpl_constraint_flag equal to 1");
    CHECK(encCfg->m_noRectSliceConstraintFlag && !encCfg->m_rasterSliceFlag,
          "Rectangular slice shall be deactivated when gci_no_rectangular_slice_constraint_flag equal to 1");
    CHECK(encCfg->m_oneSlicePerSubpicConstraintFlag && !encCfg->m_singleSlicePerSubPicFlag,
          "Each picture shall consist of one and only one rectangular slice when "
          "gci_one_slice_per_subpic_constraint_flag equal to 1");
    CHECK(encCfg->m_noSubpicInfoConstraintFlag && encCfg->m_subPicInfoPresentFlag,
          "Subpicture information shall not present when gci_no_subpic_info_constraint_flag equal to 1");
    CHECK(encCfg->m_noTrailConstraintFlag && encCfg->m_intraPeriod != 1,
          "TRAIL shall be deactivated when m_noTrailConstraintFlag is equal to 1");
    CHECK(encCfg->m_noStsaConstraintFlag && (encCfg->m_intraPeriod != 1 || hasNonZeroTemporalID(encCfg)),
          "STSA shall be deactivated when m_noStsaConstraintFlag is equal to 1");
    CHECK(encCfg->m_noRaslConstraintFlag && (encCfg->m_intraPeriod != 1 || hasLeadingPicture(encCfg)),
          "RASL shall be deactivated when m_noRaslConstraintFlag is equal to 1");
    CHECK(encCfg->m_noRadlConstraintFlag && (encCfg->m_intraPeriod != 1 || hasLeadingPicture(encCfg)),
          "RADL shall be deactivated when m_noRadlConstraintFlag is equal to 1");
    CHECK(encCfg->m_noCraConstraintFlag && (encCfg->m_decodingRefreshType == 1),
          "CRA shall be deactivated when m_noCraConstraintFlag is equal to 1");
    CHECK(encCfg->m_noRprConstraintFlag && encCfg->m_rprEnabledFlag,
          "Reference picture resampling shall be deactivated when m_noRprConstraintFlag is equal to 1");
    CHECK(encCfg->m_noResChangeInClvsConstraintFlag && encCfg->m_resChangeInClvsEnabled,
          "Resolution change in CLVS shall be deactivated when m_noResChangeInClvsConstraintFlag is equal to 1");
    CHECK(encCfg->m_internalBitDepth[ChannelType::LUMA] > encCfg->m_maxBitDepthConstraintIdc,
          "Internal bit depth shall be less than or equal to m_maxBitDepthConstraintIdc");
    CHECK(encCfg->m_chromaFormatIdc > encCfg->m_maxChromaFormatConstraintIdc,
          "Chroma format Idc shall be less than or equal to m_maxBitDepthConstraintIdc");
    CHECK(encCfg->m_noMttConstraintFlag &&
            (encCfg->m_uiMaxMTTHierarchyDepth || encCfg->m_uiMaxMTTHierarchyDepthI ||
             encCfg->m_uiMaxMTTHierarchyDepthIChroma),
          "Mtt shall be deactivated when m_bNoMttConstraintFlag is equal to 1");
    CHECK(encCfg->m_noQtbttDualTreeIntraConstraintFlag && encCfg->m_dualITree,
          "Dual tree shall be deactivated when m_bNoQtbttDualTreeIntraConstraintFlag is equal to 1");
    CHECK(encCfg->m_CTUSize > (1 << (encCfg->m_maxLog2CtuSizeConstraintIdc)),
          "CTUSize shall be less than or equal to 1 << m_maxLog2CtuSize");
    CHECK(encCfg->m_noPartitionConstraintsOverrideConstraintFlag && encCfg->m_useSplitConsOverride,
          "Partition override shall be deactivated when m_noPartitionConstraintsOverrideConstraintFlag is equal to 1");
    CHECK(encCfg->m_noSaoConstraintFlag && encCfg->m_useSao,
          "SAO shall be deactivated when m_bNoSaoConstraintFlag is equal to 1");
    CHECK(encCfg->m_noCCSaoConstraintFlag && encCfg->m_CCSAO,
          "CCSAO shall be deactivated when m_noCCSaoConstraintFlag is equal to 1");
    CHECK(encCfg->m_noAlfConstraintFlag && encCfg->m_alf,
          "ALF shall be deactivated when m_bNoAlfConstraintFlag is equal to 1");
    CHECK(encCfg->m_noCCAlfConstraintFlag && encCfg->m_ccalf,
          "CCALF shall be deactivated when m_noCCAlfConstraintFlag is equal to 1");
    CHECK(encCfg->m_noWeightedPredictionConstraintFlag && (encCfg->m_useWeightedPred || encCfg->m_useWeightedBiPred),
          "Weighted Prediction shall be deactivated when m_bNoWeightedPredictionConstraintFlag is equal to 1");
    CHECK(encCfg->m_noRefWraparoundConstraintFlag && encCfg->m_wrapAround,
          "Wrap around shall be deactivated when m_bNoRefWraparoundConstraintFlag is equal to 1");
    CHECK(encCfg->m_noTemporalMvpConstraintFlag && encCfg->m_TMVPModeId,
          "Temporal MVP shall be deactivated when m_bNoTemporalMvpConstraintFlag is equal to 1");
    CHECK(encCfg->m_noSbtmvpConstraintFlag && encCfg->m_sbTmvpEnableFlag,
          "SbTMVP shall be deactivated when m_bNoSbtmvpConstraintFlag is equal to 1");
    CHECK(encCfg->m_noAmvrConstraintFlag && (encCfg->m_ImvMode != IMV_OFF || encCfg->m_AffineAmvr),
          "AMVR shall be deactivated when m_bNoAmvrConstraintFlag is equal to 1");
    CHECK(encCfg->m_noBdofConstraintFlag && encCfg->m_BIO,
          "BIO shall be deactivated when m_bNoBdofConstraintFlag is equal to 1");
    CHECK(encCfg->m_noCclmConstraintFlag && encCfg->m_LMChroma,
          "CCLM shall be deactivated when m_bNoCclmConstraintFlag is equal to 1");
    CHECK(encCfg->m_noCCCMConstraintFlag && encCfg->m_CCCM,
          "CCCM shall be deactivated when m_noCCCMConstraintFlag is equal to 1");
    CHECK(encCfg->m_noMtsConstraintFlag && (encCfg->m_mtsMode || encCfg->m_implicitMtsIntra),
          "MTS shall be deactivated when m_bNoMtsConstraintFlag is equal to 1");
    CHECK(encCfg->m_noSbtConstraintFlag && encCfg->m_SBT,
          "SBT shall be deactivated when m_noSbtConstraintFlag_nonPackedConstraintFlag is equal to 1");
    CHECK(encCfg->m_noAffineMotionConstraintFlag && encCfg->m_Affine,
          "Affine shall be deactivated when m_bNoAffineMotionConstraintFlag is equal to 1");
    CHECK(encCfg->m_noBcwConstraintFlag && encCfg->m_bcw,
          "BCW shall be deactivated when m_bNoBcwConstraintFlag is equal to 1");
    CHECK(encCfg->m_noIbcConstraintFlag && encCfg->m_ibcMode,
          "IBC shall be deactivated when m_noIbcConstraintFlag is equal to 1");
    CHECK(encCfg->m_noCiipConstraintFlag && encCfg->m_ciip,
          "CIIP shall be deactivated when m_bNoCiipConstraintFlag is equal to 1");
    CHECK(encCfg->m_noGeoConstraintFlag && encCfg->m_Geo,
          "GEO shall be deactivated when m_noGeoConstraintFlag is equal to 1");
    CHECK(encCfg->m_noSgpmConstraintFlag && encCfg->m_sgpm,
          "SGPM shall be deactivated when m_noSgpmConstraintFlag is equal to 1");
    CHECK(encCfg->m_noObmcConstraintFlag && encCfg->m_obmc,
          "OBMC shall be deactivated when m_noObmcConstraintFlag is equal to 1");
    CHECK(encCfg->m_noLadfConstraintFlag && encCfg->m_ladfEnabled,
          "LADF shall be deactivated when m_bNoLadfConstraintFlag is equal to 1");
    CHECK(encCfg->m_noTransformSkipConstraintFlag && encCfg->m_useTransformSkip,
          "Transform skip shall be deactivated when m_noTransformSkipConstraintFlag is equal to 1");
    CHECK(encCfg->m_noLumaTransformSize64ConstraintFlag && encCfg->m_log2MaxTbSize > 5,
          "Max transform size shall be less than 64 when m_noLumaTransformSize64ConstraintFlag is equal to 1");
    CHECK(encCfg->m_noBDPCMConstraintFlag && encCfg->m_useBDPCM,
          "BDPCM shall be deactivated when m_noBDPCMConstraintFlag is equal to 1");
    CHECK(encCfg->m_noJointCbCrConstraintFlag && encCfg->m_jointCbCrMode,
          "JCCR shall be deactivated when m_noJointCbCrConstraintFlag is equal to 1");
    CHECK(encCfg->m_noDepQuantConstraintFlag && encCfg->m_DepQuantEnabledIdc,
          "DQ shall be deactivated when m_bNoDepQuantConstraintFlag is equal to 1");
    CHECK(encCfg->m_noSignDataHidingConstraintFlag && encCfg->m_SignDataHidingEnabledFlag,
          "SDH shall be deactivated when m_bNoSignDataHidingConstraintFlag is equal to 1");
    CHECK(encCfg->m_noApsConstraintFlag && (encCfg->m_lmcsEnabled || (encCfg->m_useScalingListId != SCALING_LIST_OFF)),
          "LMCS and explict scaling list shall be deactivated when m_noApsConstraintFlag is equal to 1");
    CHECK(encCfg->m_noMrlConstraintFlag && encCfg->m_MRL,
          "MRL shall be deactivated when m_noMrlConstraintFlag is equal to 1");
    CHECK(encCfg->m_noMipConstraintFlag && encCfg->m_MIP,
          "MIP shall be deactivated when m_noMipConstraintFlag is equal to 1");
    CHECK(encCfg->m_noDirPlanar && encCfg->m_dirPlanar,
          "Directional Planar shall be deactivated when m_noDirPlanar is equal to 1");
    CHECK(encCfg->m_noLfnstConstraintFlag &&
            (encCfg->m_intraLFNSTISlice || encCfg->m_intraLFNSTPBSlice || encCfg->m_interLFNST),
          "LFNST shall be deactivated when m_noLfnstConstraintFlag is equal to 1");
    CHECK(encCfg->m_noMmvdConstraintFlag && encCfg->m_MMVD,
          "MMVD shall be deactivated when m_noMmvdConstraintFlag is equal to 1");
    CHECK(encCfg->m_noSmvdConstraintFlag && encCfg->m_SMVD,
          "SMVD shall be deactivated when m_noSmvdConstraintFlag is equal to 1");
    CHECK(encCfg->m_noProfConstraintFlag && encCfg->m_PROF,
          "PROF shall be deactivated when m_noProfConstraintFlag is equal to 1");
    CHECK(encCfg->m_noPaletteConstraintFlag && encCfg->m_PLTMode,
          "Palette shall be deactivated when m_noPaletteConstraintFlag is equal to 1");
    CHECK(encCfg->m_noLmcsConstraintFlag && encCfg->m_lmcsEnabled,
          "LMCS shall be deactivated when m_noLmcsConstraintFlag is equal to 1");
    CHECK(encCfg->m_noExplicitScaleListConstraintFlag && encCfg->m_useScalingListId != SCALING_LIST_OFF,
          "Explicit scaling list shall be deactivated when m_noExplicitScaleListConstraintFlag is equal to 1");
    CHECK(encCfg->m_noChromaQpOffsetConstraintFlag && encCfg->m_cuChromaQpOffsetSubdiv,
          "Chroma Qp offset shall be 0 when m_noChromaQpOffsetConstraintFlag is equal to 1");
    CHECK(encCfg->m_noExtendedPrecisionProcessingConstraintFlag && encCfg->m_extendedPrecisionProcessingFlag,
          "ExtendedPrecision shall be deactivated when m_noExtendedPrecisionProcessingConstraintFlag is equal to 1");
    CHECK(encCfg->m_noTsResidualCodingRiceConstraintFlag && encCfg->m_tsrcRicePresentFlag,
          "TSRCRicePresent shall be deactivated when m_noTsResidualCodingRiceConstraintFlag is equal to 1");
    CHECK(encCfg->m_noRrcRiceExtensionConstraintFlag && encCfg->m_rrcRiceExtensionEnableFlag,
          "ExtendedRiceRRC shall be deactivated when m_noRrcRiceExtensionConstraintFlag is equal to 1");
    CHECK(encCfg->m_noPersistentRiceAdaptationConstraintFlag && encCfg->m_persistentRiceAdaptationEnabledFlag,
          "GolombRiceParameterAdaptation shall be deactivated when m_noPersistentRiceAdaptationConstraintFlag is equal "
          "to 1");
    CHECK(encCfg->m_noReverseLastSigCoeffConstraintFlag && encCfg->m_reverseLastSigCoeffEnabledFlag,
          "ReverseLastSigCoeff shall be deactivated when m_noReverseLastSigCoeffConstraintFlag is equal to 1");
  }

  encCfg->m_numRPLList0 = 0;
  for (int i = 0; i < MAX_GOP; i++)
  {
    if (encCfg->m_RPLList0[i].m_POC != -1)
    {
      encCfg->m_numRPLList0++;
    }
  }
  encCfg->m_numRPLList1 = 0;
  for (int i = 0; i < MAX_GOP; i++)
  {
    if (encCfg->m_RPLList1[i].m_POC != -1)
    {
      encCfg->m_numRPLList1++;
    }
  }

#if ENABLE_QPA
  encCfg->m_bUsePerceptQPA &= !encCfg->m_bUseAdaptiveQP;
#endif

  if (encCfg->m_costMode == COST_LOSSLESS_CODING)
  {
    encCfg->m_uiDeltaQpRD = 0;
  }

  if (!encCfg->m_subPicInfoPresentFlag)
  {
    CHECK(encCfg->m_numSubPics != 0, "sub-picture info given but info present flag not set");
    encCfg->m_numSubPics = 1;
    encCfg->m_subPicCtuTopLeftX.resize(1);
    encCfg->m_subPicCtuTopLeftY.resize(1);
    encCfg->m_subPicWidth.resize(1);
    encCfg->m_subPicHeight.resize(1);
    encCfg->m_subPicTreatedAsPicFlag.resize(1);
    encCfg->m_loopFilterAcrossSubpicEnabledFlag.resize(1);
    encCfg->m_subPicId.resize(1);
  }

  encCfg->m_minQt[2] <<= getChannelTypeScaleX(ChannelType::CHROMA, encCfg->m_chromaFormatIdc);

  encCfg->m_explicitMtsIntra = (encCfg->m_mtsMode & 1) != 0;
  encCfg->m_explicitMtsInter = (encCfg->m_mtsMode & 2) != 0;
  encCfg->m_implicitMtsIntra |= (encCfg->m_mtsMode & 4) != 0;

  if (!encCfg->m_picPartitionFlag)
  {
    encCfg->m_rasterSliceFlag                 = false;
    encCfg->m_numSlicesInPic                  = 1;
    encCfg->m_tileIdxDeltaPresentFlag         = false;
    encCfg->m_disableLFCrossTileBoundaryFlag  = false;
    encCfg->m_disableLFCrossSliceBoundaryFlag = false;
  }

  encCfg->m_seiCfg.m_dependentRAPIndicationSEIEnabled = encCfg->m_drapPeriod > 0;
  encCfg->m_seiCfg.m_edrapIndicationSEIEnabled        = encCfg->m_edrapPeriod > 0;
  encCfg->m_seiCfg.m_alternativeTransferCharacteristicsSEIEnabled =
    encCfg->m_seiCfg.m_preferredTransferCharacteristics >= 0;

  for (int i = 0; i < encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsNumFilters; i++)
  {
    if (encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsPropertyPresentFlag[i])
    {
      if (!encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsBaseFlag[i])
      {
        bool baseFilterExist = false;
        for (int j = i - 1; j >= 0; j--)
        {
          if (encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsId[i] ==
              encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsId[j])
          {
            baseFilterExist = true;
            break;
          }
        }
        CHECK(!baseFilterExist, "No base filter found! Cannot have an update filter without base filter.")
      }
    }
  }

  if (encCfg->m_OPIEnabled)
  {
    if (encCfg->m_opiMaxTemporalLayer != 500)
    {
      encCfg->m_opi.m_htidinfopresentflag = true;
      encCfg->m_opi.m_opihtidplus1        = encCfg->m_opiMaxTemporalLayer + 1;
    }
    if (encCfg->m_targetOlsIdx != 500)
    {
      encCfg->m_opi.m_olsinfopresentflag = true;
      encCfg->m_opi.m_opiolsidx          = encCfg->m_targetOlsIdx;
    }
  }
#if JVET_O0756_CALCULATE_HDRMETRICS
  encCfg->m_chromaLocation[1] = encCfg->m_chromaLocation[0];
#endif

  encCfg->m_fastLIC       = 0;
  encCfg->m_fastLICAffine = false;
  const int picSize       = encCfg->m_sourceWidth * encCfg->m_sourceHeight;
  if (encCfg->m_intraPeriod > 1)
  {
    if (picSize >= (1280 * 720))
    {
      encCfg->m_fastLIC = 0x01;
    }
    else if (picSize >= (832 * 480))
    {
      encCfg->m_fastLIC = 0x02;
    }
    else if (picSize >= (416 * 240))
    {
      encCfg->m_fastLIC = 0x03;
    }
    if (picSize < (3840 * 2160))
    {
      encCfg->m_fastLICAffine = true;
    }
  }
  else if (encCfg->m_intraPeriod < 0)
  {
    if (picSize >= (1920 * 1080))
    {
      encCfg->m_fastLIC = 0x01;
    }
    else if (picSize >= (1280 * 720))
    {
      encCfg->m_fastLIC = 0x02;
    }
    else if (picSize >= (416 * 240))
    {
      encCfg->m_fastLIC = 0x03;
    }
    if (!encCfg->m_ibcMode)
    {
      encCfg->m_fastLIC += 0x04;
    }
    if (picSize >= (832 * 480) && picSize <= (1280 * 720))
    {
      encCfg->m_fastLICAffine = true;
    }
  }
}

void EncApp::xCreateLib(std::list<PelUnitBuf *> &recBufList, const int layerId)
{
  const EncCfg *encCfg = &m_cEncLib.m_encCfg;

  // Video I/O
  m_cVideoIOYuvInputFile.open(encCfg->m_inputFileName, false, encCfg->m_inputBitDepth, encCfg->m_msbExtendedBitDepth,
                              encCfg->m_internalBitDepth);   // read  mode
#if EXTENSION_360_VIDEO
  m_cVideoIOYuvInputFile.skipFrames(encCfg->m_frameSkip, encCfg->m_inputFileWidth, encCfg->m_inputFileHeight,
                                    encCfg->m_inputChromaFormatIDC);
#else
  const int sourceHeight = encCfg->m_fieldSeqFlag ? encCfg->m_iSourceHeightOrg : encCfg->m_sourceHeight;
  if (encCfg->m_sourceScalingRatioHor != 1.0 || encCfg->m_sourceScalingRatioVer != 1.0)
  {
    m_cVideoIOYuvInputFile.skipFrames(encCfg->m_frameSkip, encCfg->m_sourceWidthBeforeScale,
                                      encCfg->m_sourceHeightBeforeScale, encCfg->m_inputChromaFormatIDC);
  }
  else
  {
    m_cVideoIOYuvInputFile.skipFrames(encCfg->m_frameSkip, encCfg->m_sourceWidth - encCfg->m_sourcePadding[0],
                                      sourceHeight - encCfg->m_sourcePadding[1], encCfg->m_inputChromaFormatIDC);
  }
#endif
  if (!encCfg->m_reconFileName.empty())
  {
    if (encCfg->m_packedYUVMode &&
        ((encCfg->m_outputBitDepth[ChannelType::LUMA] != 10 && encCfg->m_outputBitDepth[ChannelType::LUMA] != 12) ||
         ((encCfg->m_sourceWidth & (1 + (encCfg->m_outputBitDepth[ChannelType::LUMA] & 3))) != 0)))
    {
      EXIT("Invalid output bit-depth or image width for packed YUV output, aborting\n");
    }
    if (encCfg->m_packedYUVMode && isChromaEnabled(encCfg->m_chromaFormatIdc) &&
        ((encCfg->m_outputBitDepth[ChannelType::CHROMA] != 10 && encCfg->m_outputBitDepth[ChannelType::CHROMA] != 12) ||
         (((encCfg->m_sourceWidth / SPS::getWinUnitX(encCfg->m_chromaFormatIdc)) &
           (1 + (encCfg->m_outputBitDepth[ChannelType::CHROMA] & 3))) != 0)))
    {
      EXIT("Invalid chroma output bit-depth or image width for packed YUV output, aborting\n");
    }

    std::string reconFileName = encCfg->m_reconFileName;
    if (encCfg->m_reconFileName.compare("/dev/null") && (encCfg->m_maxLayers > 1))
    {
      size_t pos = reconFileName.find_last_of('.');
      if (pos != std::string::npos)
      {
        reconFileName.insert(pos, std::to_string(layerId));
      }
      else
      {
        reconFileName.append(std::to_string(layerId));
      }
    }
    if (isY4mFileExt(reconFileName))
    {
      const auto sx = SPS::getWinUnitX(encCfg->m_chromaFormatIdc);
      const auto sy = SPS::getWinUnitY(encCfg->m_chromaFormatIdc);
      m_cVideoIOYuvReconFile.setOutputY4mInfo(
        encCfg->m_sourceWidth - (encCfg->m_confWinLeft + encCfg->m_confWinRight) * sx,
        encCfg->m_sourceHeight - (encCfg->m_confWinTop + encCfg->m_confWinBottom) * sy, encCfg->m_frameRate, 1,
        encCfg->m_internalBitDepth[ChannelType::LUMA], encCfg->m_chromaFormatIdc);
    }
    m_cVideoIOYuvReconFile.open(reconFileName, true, encCfg->m_outputBitDepth, encCfg->m_outputBitDepth,
                                encCfg->m_internalBitDepth);   // write mode
  }

#if JVET_Z0120_SII_SEI_PROCESSING
  if (encCfg->m_seiCfg.m_ShutterFilterEnable && !encCfg->m_seiCfg.m_shutterIntervalPreFileName.empty())
  {
    m_cTVideoIOYuvSIIPreFile.open(encCfg->m_seiCfg.m_shutterIntervalPreFileName, true, encCfg->m_outputBitDepth,
                                  encCfg->m_outputBitDepth, encCfg->m_internalBitDepth);   // write mode
  }
#endif
  // create the encoder
  m_cEncLib.create(layerId);

  // create the output buffer
  for (int i = 0; i < (encCfg->m_gopSize + 1 + (encCfg->m_fieldSeqFlag ? 1 : 0)); i++)
  {
    recBufList.push_back(new PelUnitBuf);
  }
}

void EncApp::xDestroyLib()
{
  // Video I/O
  m_cVideoIOYuvInputFile.close();
  m_cVideoIOYuvReconFile.close();
#if JVET_Z0120_SII_SEI_PROCESSING
  const EncCfg *encCfg = &m_cEncLib.m_encCfg;
  if (encCfg->m_seiCfg.m_ShutterFilterEnable && !encCfg->m_seiCfg.m_shutterIntervalPreFileName.empty())
  {
    m_cTVideoIOYuvSIIPreFile.close();
  }
#endif

  // Neo Decoder
  m_cEncLib.destroy();
}

void EncApp::xInitLib() { m_cEncLib.init(this); }

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

bool EncApp::parseCfg(int argc, char *argv[])
{
  EncAppCfg encCfgParser;
  bool      ret;
  ret = encCfgParser.parseCfg(argc, argv, &m_cEncLib.m_encCfg);
  if (ret)
  {
    const EncCfg &encCfg = m_cEncLib.m_encCfg;
    EipLog::initEncoder(encCfg.m_inputFileName, encCfg.m_bitstreamFileName, encCfg.m_iQP);
  }
  return ret;
}

void EncApp::createLib(const int layerIdx)
{
  EncCfg *encCfg = &m_cEncLib.m_encCfg;

  const int sourceHeight = encCfg->m_fieldSeqFlag ? encCfg->m_iSourceHeightOrg : encCfg->m_sourceHeight;
  UnitArea  unitArea(encCfg->m_chromaFormatIdc, Area(0, 0, encCfg->m_sourceWidth, sourceHeight));

  m_orgPic     = new PelStorage;
  m_trueOrgPic = new PelStorage;
  m_orgPic->create(unitArea);
  m_trueOrgPic->create(unitArea);
  if (encCfg->m_sourceScalingRatioHor != 1.0 || encCfg->m_sourceScalingRatioVer != 1.0)
  {
    UnitArea unitAreaPrescale(encCfg->m_chromaFormatIdc,
                              Area(0, 0, encCfg->m_sourceWidthBeforeScale, encCfg->m_sourceHeightBeforeScale));
    m_orgPicBeforeScale     = new PelStorage;
    m_trueOrgPicBeforeScale = new PelStorage;
    m_orgPicBeforeScale->create(unitAreaPrescale);
    m_trueOrgPicBeforeScale->create(unitAreaPrescale);
  }
  if (encCfg->m_gopBasedTemporalFilterEnabled || encCfg->m_bimEnabled)
  {
    m_filteredOrgPic = new PelStorage;
    m_filteredOrgPic->create(unitArea);
  }
  if (encCfg->m_resChangeInClvsEnabled && encCfg->m_gopBasedRPREnabledFlag)
  {
    UnitArea unitAreaRPR10(encCfg->m_chromaFormatIdc, Area(0, 0, encCfg->m_sourceWidth, sourceHeight));
    UnitArea unitAreaRPR20(encCfg->m_chromaFormatIdc, Area(0, 0, encCfg->m_sourceWidth / 2, sourceHeight / 2));
    m_rprPic[0] = new PelStorage;
    m_rprPic[0]->create(unitAreaRPR10);
    m_rprPic[1] = new PelStorage;
    m_rprPic[1]->create(unitAreaRPR20);
  }
  if (encCfg->m_seiCfg.m_fgcSEIAnalysisEnabled && encCfg->m_seiCfg.m_fgcSEIExternalDenoised.empty())
  {
    m_filteredOrgPicForFG = new PelStorage;
    m_filteredOrgPicForFG->create(unitArea);
  }

  if (!m_bitstream.is_open())
  {
    m_bitstream.open(encCfg->m_bitstreamFileName.c_str(), std::fstream::binary | std::fstream::out);
    if (!m_bitstream)
    {
      EXIT("Failed to open bitstream file " << encCfg->m_bitstreamFileName.c_str() << " for writing\n");
    }
  }

  // initialize internal class & member variables and VPS
  xInitLibCfg(layerIdx);
  const int layerId = m_cEncLib.m_vps == nullptr ? 0 : m_cEncLib.m_vps->m_vpsLayerId[layerIdx];
  xCreateLib(m_recBufList, layerId);
  xInitLib();

  printChromaFormat();

#if EXTENSION_360_VIDEO
  m_ext360 =
    new TExt360AppEncTop(*this, m_cEncLib.getGOPEncoder()->getExt360Data(), *(m_cEncLib.getGOPEncoder()), *m_orgPic);
#endif

  if (encCfg->m_gopBasedTemporalFilterEnabled || encCfg->m_bimEnabled)
  {
    m_temporalFilter.init(
      encCfg->m_frameSkip, encCfg->m_inputBitDepth, encCfg->m_msbExtendedBitDepth, encCfg->m_internalBitDepth,
      encCfg->m_sourceWidth, sourceHeight, encCfg->m_sourcePadding, encCfg->m_clipInputVideoToRec709Range,
      encCfg->m_inputFileName, encCfg->m_chromaFormatIdc, encCfg->m_inputChromaFormatIDC,
      encCfg->m_inputColourSpaceConvert, encCfg->m_iQP, encCfg->m_gopBasedTemporalFilterStrengths,
      encCfg->m_gopBasedTemporalFilterPastRefs, encCfg->m_gopBasedTemporalFilterFutureRefs, encCfg->m_firstValidFrame,
      encCfg->m_lastValidFrame, encCfg->m_gopBasedTemporalFilterEnabled, encCfg->m_gopBasedTemporalFilterUnitSize,
      &m_cEncLib.m_adaptQPmap, encCfg->m_bimEnabled, encCfg->m_bimUnitSize, &m_cEncLib.m_if);
  }
  if (encCfg->m_seiCfg.m_fgcSEIAnalysisEnabled && encCfg->m_seiCfg.m_fgcSEIExternalDenoised.empty())
  {
    m_temporalFilterForFG.init(
      encCfg->m_frameSkip, encCfg->m_inputBitDepth, encCfg->m_msbExtendedBitDepth, encCfg->m_internalBitDepth,
      encCfg->m_sourceWidth, sourceHeight, encCfg->m_sourcePadding, encCfg->m_clipInputVideoToRec709Range,
      encCfg->m_inputFileName, encCfg->m_chromaFormatIdc, encCfg->m_inputChromaFormatIDC,
      encCfg->m_inputColourSpaceConvert, encCfg->m_iQP, encCfg->m_seiCfg.m_fgcSEITemporalFilterStrengths,
      encCfg->m_seiCfg.m_fgcSEITemporalFilterPastRefs, encCfg->m_seiCfg.m_fgcSEITemporalFilterFutureRefs,
      encCfg->m_firstValidFrame, encCfg->m_lastValidFrame, true, encCfg->m_gopBasedTemporalFilterUnitSize,
      &m_cEncLib.m_adaptQPmap, encCfg->m_bimEnabled, encCfg->m_CTUSize, &m_cEncLib.m_if);
  }
}

void EncApp::destroyLib()
{
  const EncCfg *encCfg = &m_cEncLib.m_encCfg;

  printf("\nLayerId %2d", m_cEncLib.m_layerId);

  m_cEncLib.printSummary(encCfg->m_fieldSeqFlag);

  // delete used buffers in encoder class
  m_cEncLib.deletePicBuffer();

  for (auto &p: m_recBufList)
  {
    delete p;
  }
  m_recBufList.clear();

  xDestroyLib();

  if (m_bitstream.is_open())
  {
    m_bitstream.close();
  }

  m_orgPic->destroy();
  m_trueOrgPic->destroy();
  delete m_trueOrgPic;
  delete m_orgPic;

  if (encCfg->m_sourceScalingRatioHor != 1.0 || encCfg->m_sourceScalingRatioVer != 1.0)
  {
    m_orgPicBeforeScale->destroy();
    m_trueOrgPicBeforeScale->destroy();
    delete m_trueOrgPicBeforeScale;
    delete m_orgPicBeforeScale;
  }

  if (encCfg->m_resChangeInClvsEnabled && encCfg->m_gopBasedRPREnabledFlag)
  {
    for (int i = 0; i < 2; i++)
    {
      m_rprPic[i]->destroy();
      delete m_rprPic[i];
    }
  }
  if (encCfg->m_gopBasedTemporalFilterEnabled || encCfg->m_bimEnabled)
  {
    m_filteredOrgPic->destroy();
    delete m_filteredOrgPic;
  }

  if (encCfg->m_seiCfg.m_fgcSEIAnalysisEnabled && encCfg->m_seiCfg.m_fgcSEIExternalDenoised.empty())
  {
    m_filteredOrgPicForFG->destroy();
    delete m_filteredOrgPicForFG;
    m_filteredOrgPicForFG = nullptr;
  }
#if EXTENSION_360_VIDEO
  delete m_ext360;
#endif

  printRateSummary();
}

bool EncApp::encodePrep(bool &eos)
{
  PROFILER_SCOPE(0, g_timeProfiler, P_TOP_LEVEL);
  EncCfg *encCfg = &m_cEncLib.m_encCfg;

  // main encoder loop
  const InputColourSpaceConversion ipCSC = encCfg->m_inputColourSpaceConvert;
  const InputColourSpaceConversion snrCSC =
    (!encCfg->m_snrInternalColourSpace) ? encCfg->m_inputColourSpaceConvert : IPCOLOURSPACE_UNCHANGED;

  // read input YUV file
#if EXTENSION_360_VIDEO
  if (m_ext360->isEnabled())
  {
    m_ext360->read(m_cVideoIOYuvInputFile, *m_orgPic, *m_trueOrgPic, ipCSC);
  }
  else
  {
    m_cVideoIOYuvInputFile.read(*m_orgPic, *m_trueOrgPic, ipCSC, encCfg->m_sourcePadding,
                                encCfg->m_inputChromaFormatIDC, encCfg->m_clipInputVideoToRec709Range);
  }
#else
  if (encCfg->m_sourceScalingRatioHor != 1.0 || encCfg->m_sourceScalingRatioVer != 1.0)
  {
    int noPadding[2] = { 0 };
    m_cVideoIOYuvInputFile.read(*m_orgPicBeforeScale, *m_trueOrgPicBeforeScale, ipCSC, noPadding,
                                encCfg->m_inputChromaFormatIDC, encCfg->m_clipInputVideoToRec709Range);
    int w0 = encCfg->m_sourceWidthBeforeScale;
    int h0 = encCfg->m_sourceHeightBeforeScale;
    int w1 = m_orgPic->get(COMP_Y).width -
      SPS::getWinUnitX(encCfg->m_chromaFormatIdc) * (encCfg->m_confWinLeft + encCfg->m_confWinRight);
    int h1 = m_orgPic->get(COMP_Y).height -
      SPS::getWinUnitY(encCfg->m_chromaFormatIdc) * (encCfg->m_confWinTop + encCfg->m_confWinBottom);
    int          xScale       = ((w0 << ScalingRatio::BITS) + (w1 >> 1)) / w1;
    int          yScale       = ((h0 << ScalingRatio::BITS) + (h1 >> 1)) / h1;
    ScalingRatio scalingRatio = { xScale, yScale };
    Window       conformanceWindow1;
    conformanceWindow1.setWindow(encCfg->m_confWinLeft, encCfg->m_confWinRight, encCfg->m_confWinTop,
                                 encCfg->m_confWinBottom);

    bool downsampling = (encCfg->m_sourceWidthBeforeScale > encCfg->m_sourceWidth) ||
      (encCfg->m_sourceHeightBeforeScale > encCfg->m_sourceHeight);
    bool useLumaFilter = downsampling;
    Picture::rescalePicture(scalingRatio, *m_orgPicBeforeScale, Window(), *m_orgPic, conformanceWindow1,
                            encCfg->m_inputChromaFormatIDC, encCfg->m_internalBitDepth, useLumaFilter, downsampling,
                            encCfg->m_horCollocatedChromaFlag, encCfg->m_verCollocatedChromaFlag);
    m_trueOrgPic->copyFrom(*m_orgPic);
  }
  else
  {
    m_cVideoIOYuvInputFile.read(*m_orgPic, *m_trueOrgPic, ipCSC, encCfg->m_sourcePadding,
                                encCfg->m_inputChromaFormatIDC, encCfg->m_clipInputVideoToRec709Range);
  }
#endif

  if (encCfg->m_seiCfg.m_fgcSEIAnalysisEnabled && encCfg->m_seiCfg.m_fgcSEIExternalDenoised.empty())
  {
    m_filteredOrgPicForFG->copyFrom(*m_orgPic);
    m_temporalFilterForFG.filter(m_filteredOrgPicForFG, m_frameRcvd);
  }
  if (encCfg->m_gopBasedTemporalFilterEnabled || encCfg->m_bimEnabled)
  {
    m_temporalFilter.filter(m_orgPic, m_frameRcvd);
    m_filteredOrgPic->copyFrom(*m_orgPic);
  }

  // increase number of received frames
  m_frameRcvd++;

  eos = (encCfg->m_fieldSeqFlag && (m_frameRcvd == (encCfg->m_framesToBeEncoded >> 1))) ||
    (!encCfg->m_fieldSeqFlag && (m_frameRcvd == encCfg->m_framesToBeEncoded));

  // if end of file (which is only detected on a read failure) flush the encoder of any queued pictures
  if (m_cVideoIOYuvInputFile.isEof())
  {
    m_flush = true;
    eos     = true;
    m_frameRcvd--;
    encCfg->m_framesToBeEncoded = m_frameRcvd;
  }

  bool keepDoing = false;

  // call encoding function for one frame
  if (encCfg->m_fieldSeqFlag)
  {
    keepDoing =
      m_cEncLib.encodePrep(eos, m_flush ? 0 : m_orgPic, m_flush ? 0 : m_trueOrgPic, m_flush ? 0 : m_filteredOrgPic,
                           snrCSC, m_recBufList, m_numEncoded, encCfg->m_isTopFieldFirst);
  }
  else
  {
    keepDoing =
      m_cEncLib.encodePrep(eos, m_flush ? 0 : m_orgPic, m_flush ? 0 : m_trueOrgPic, m_flush ? 0 : m_filteredOrgPic,
                           m_flush ? 0 : m_filteredOrgPicForFG, snrCSC, m_recBufList, m_numEncoded, m_rprPic);
  }

#if JVET_Z0120_SII_SEI_PROCESSING
  if (encCfg->m_seiCfg.m_ShutterFilterEnable && !encCfg->m_seiCfg.m_shutterIntervalPreFileName.empty())
  {
    m_cTVideoIOYuvSIIPreFile.write(m_orgPic->get(COMP_Y).width, m_orgPic->get(COMP_Y).height, *m_orgPic,
                                   encCfg->m_inputColourSpaceConvert, encCfg->m_packedYUVMode, encCfg->m_confWinLeft,
                                   encCfg->m_confWinRight, encCfg->m_confWinTop, encCfg->m_confWinBottom,
                                   ChromaFormat::UNDEFINED, encCfg->m_clipOutputVideoToRec709Range);
  }
#endif

  return keepDoing;
}

bool EncApp::encode()
{
  PROFILER_SCOPE(0, g_timeProfiler, P_TOP_LEVEL);
  const EncCfg *encCfg = &m_cEncLib.m_encCfg;

  const InputColourSpaceConversion snrCSC =
    (!encCfg->m_snrInternalColourSpace) ? encCfg->m_inputColourSpaceConvert : IPCOLOURSPACE_UNCHANGED;
  bool keepDoing = false;

  // call encoding function for one frame
  if (encCfg->m_fieldSeqFlag)
  {
    keepDoing = m_cEncLib.encode(snrCSC, m_recBufList, m_numEncoded, encCfg->m_isTopFieldFirst);
  }
  else
  {
    keepDoing = m_cEncLib.encode(snrCSC, m_recBufList, m_numEncoded);
  }

#if JVET_O0756_CALCULATE_HDRMETRICS
  m_metricTime = m_cEncLib.getMetricTime();
#endif

  // output when the entire GOP was proccessed
  if (!keepDoing)
  {
    // write bistream to file if necessary
    if (m_numEncoded > 0)
    {
      xWriteOutput(m_numEncoded, m_recBufList);
    }
    // temporally skip frames
    if (encCfg->m_temporalSubsampleRatio > 1)
    {
#if EXTENSION_360_VIDEO
      m_cVideoIOYuvInputFile.skipFrames(encCfg->m_temporalSubsampleRatio - 1, encCfg->m_inputFileWidth,
                                        encCfg->m_inputFileHeight, encCfg->m_inputChromaFormatIDC);
#else
      const int sourceHeight = encCfg->m_fieldSeqFlag ? encCfg->m_iSourceHeightOrg : encCfg->m_sourceHeight;
      m_cVideoIOYuvInputFile.skipFrames(encCfg->m_temporalSubsampleRatio - 1,
                                        encCfg->m_sourceWidth - encCfg->m_sourcePadding[0],
                                        sourceHeight - encCfg->m_sourcePadding[1], encCfg->m_inputChromaFormatIDC);
#endif
    }
  }

  return keepDoing;
}

void EncApp::applyNnPostFilter() { m_cEncLib.applyNnPostFilter(); }

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/**
  Write access units to output file.
  \param bitstreamFile  target bitstream file
  \param numEncoded    number of encoded frames
  \param accessUnits    list of access units to be written
 */
void EncApp::xWriteOutput(int numEncoded, std::list<PelUnitBuf *> &recBufList)
{
  PROFILER_SCOPE(0, g_timeProfiler, P_TOP_LEVEL);
  const EncCfg *encCfg = &m_cEncLib.m_encCfg;

  const InputColourSpaceConversion ipCSC =
    (!encCfg->m_outputInternalColourSpace) ? encCfg->m_inputColourSpaceConvert : IPCOLOURSPACE_UNCHANGED;
  std::list<PelUnitBuf *>::iterator iterPicYuvRec = recBufList.end();
  int                               i;

  for (i = 0; i < numEncoded; i++)
  {
    --iterPicYuvRec;
  }

  if (encCfg->m_fieldSeqFlag)
  {
    // Reinterlace fields
    for (i = 0; i < numEncoded / 2; i++)
    {
      const PelUnitBuf *picYuvRecTop    = *(iterPicYuvRec++);
      const PelUnitBuf *picYuvRecBottom = *(iterPicYuvRec++);

      if (!encCfg->m_reconFileName.empty())
      {
        m_cVideoIOYuvReconFile.write(*picYuvRecTop, *picYuvRecBottom, ipCSC,
                                     false,   // TODO: encCfg->m_packedYUVMode,
                                     encCfg->m_confWinLeft, encCfg->m_confWinRight, encCfg->m_confWinTop,
                                     encCfg->m_confWinBottom, ChromaFormat::UNDEFINED, encCfg->m_isTopFieldFirst);
      }
    }
  }
  else
  {
    for (i = 0; i < numEncoded; i++)
    {
      const PelUnitBuf *picYuvRec = *(iterPicYuvRec++);
      if (!encCfg->m_reconFileName.empty())
      {
        const int  layerId = getVPS() ? getVPS()->m_generalLayerIdx[m_cEncLib.m_layerId] : 0;
        const SPS &sps     = *m_cEncLib.getSPS(layerId);
        int        ppsID   = layerId;
        if ((encCfg->m_gopBasedRPREnabledFlag && (encCfg->m_iQP >= encCfg->m_gopBasedRPRQPThreshold)) ||
            encCfg->m_rprFunctionalityTestingEnabledFlag)
        {
          const PPS &pps1 = *m_cEncLib.getPPS(ENC_PPS_ID_RPR);
          const PPS &pps2 = *m_cEncLib.getPPS(ENC_PPS_ID_RPR2);
          const PPS &pps3 = *m_cEncLib.getPPS(ENC_PPS_ID_RPR3);
          if (pps1.m_picWidthInLumaSamples == picYuvRec->get(COMP_Y).width &&
              pps1.m_picHeightInLumaSamples == picYuvRec->get(COMP_Y).height)
          {
            ppsID = ENC_PPS_ID_RPR;
          }
          else if (pps2.m_picWidthInLumaSamples == picYuvRec->get(COMP_Y).width &&
                   pps2.m_picHeightInLumaSamples == picYuvRec->get(COMP_Y).height)
          {
            ppsID = ENC_PPS_ID_RPR2;
          }
          else if (pps3.m_picWidthInLumaSamples == picYuvRec->get(COMP_Y).width &&
                   pps3.m_picHeightInLumaSamples == picYuvRec->get(COMP_Y).height)
          {
            ppsID = ENC_PPS_ID_RPR3;
          }
          else
          {
            ppsID = layerId;
          }
        }
        else
        {
          ppsID = (sps.m_maxWidthInLumaSamples != picYuvRec->get(COMP_Y).width ||
                   sps.m_maxHeightInLumaSamples != picYuvRec->get(COMP_Y).height)
            ? ENC_PPS_ID_RPR
            : layerId;
        }
        const PPS &pps = *m_cEncLib.getPPS(ppsID);
        if (encCfg->m_resChangeInClvsEnabled && encCfg->m_upscaledOutput)
        {
          m_cVideoIOYuvReconFile.writeUpscaledPicture(
            sps, pps, *picYuvRec, ipCSC, encCfg->m_packedYUVMode, encCfg->m_upscaledOutput, ChromaFormat::UNDEFINED,
            encCfg->m_clipOutputVideoToRec709Range, encCfg->m_upscaleFilterForDisplay);
        }
        else
        {
          Window confWindowPPS = pps.m_conformanceWindow;
          m_cVideoIOYuvReconFile.write(picYuvRec->get(COMP_Y).width, picYuvRec->get(COMP_Y).height, *picYuvRec, ipCSC,
                                       encCfg->m_packedYUVMode,
                                       confWindowPPS.m_winLeftOffset * SPS::getWinUnitX(encCfg->m_chromaFormatIdc),
                                       confWindowPPS.m_winRightOffset * SPS::getWinUnitX(encCfg->m_chromaFormatIdc),
                                       confWindowPPS.m_winTopOffset * SPS::getWinUnitY(encCfg->m_chromaFormatIdc),
                                       confWindowPPS.m_winBottomOffset * SPS::getWinUnitY(encCfg->m_chromaFormatIdc),
                                       ChromaFormat::UNDEFINED, encCfg->m_clipOutputVideoToRec709Range);
        }
      }
    }
  }
}

void EncApp::outputAU(const AccessUnit &au)
{
  const std::vector<uint32_t> &stats = writeAnnexBAccessUnit(m_bitstream, au);
  rateStatsAccum(au, stats);
  m_bitstream.flush();
}

/**
 *
 */
void EncApp::rateStatsAccum(const AccessUnit &au, const std::vector<uint32_t> &annexBsizes)
{
  AccessUnit::const_iterator            it_au    = au.begin();
  std::vector<uint32_t>::const_iterator it_stats = annexBsizes.begin();

  for (; it_au != au.end(); it_au++, it_stats++)
  {
    switch ((*it_au)->m_nalUnitType)
    {
    case NAL_UNIT_CODED_SLICE_TRAIL:
    case NAL_UNIT_CODED_SLICE_STSA:
    case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
    case NAL_UNIT_CODED_SLICE_IDR_N_LP:
    case NAL_UNIT_CODED_SLICE_CRA:
    case NAL_UNIT_CODED_SLICE_GDR:
    case NAL_UNIT_CODED_SLICE_RADL:
    case NAL_UNIT_CODED_SLICE_RASL:
    case NAL_UNIT_OPI:
    case NAL_UNIT_DCI:
    case NAL_UNIT_VPS:
    case NAL_UNIT_SPS:
    case NAL_UNIT_PPS:
    case NAL_UNIT_PH:
    case NAL_UNIT_PREFIX_APS:
    case NAL_UNIT_SUFFIX_APS:
      m_essentialBytes += *it_stats;
      break;
    default:
      break;
    }

    m_totalBytes += *it_stats;
  }
}

void EncApp::printRateSummary()
{
  const EncCfg *encCfg = &m_cEncLib.m_encCfg;

  double time = (double)m_frameRcvd / encCfg->m_frameRate * encCfg->m_temporalSubsampleRatio;
  msg(DETAILS, "Bytes written to file: %u (%.3f kbps)\n", m_totalBytes, 0.008 * m_totalBytes / time);
  if (encCfg->m_summaryVerboseness > 0)
  {
    msg(DETAILS, "Bytes for SPS/PPS/APS/Slice (Incl. Annex B): %u (%.3f kbps)\n", m_essentialBytes,
        0.008 * m_essentialBytes / time);
  }
}

void EncApp::printChromaFormat()
{
  const EncCfg *encCfg = &m_cEncLib.m_encCfg;

  if (g_verbosity >= DETAILS)
  {
    std::cout << std::setw(43) << "Input ChromaFormatIDC = ";
    switch (encCfg->m_inputChromaFormatIDC)
    {
    case ChromaFormat::_400:
      std::cout << "  4:0:0";
      break;
    case ChromaFormat::_420:
      std::cout << "  4:2:0";
      break;
    case ChromaFormat::_422:
      std::cout << "  4:2:2";
      break;
    case ChromaFormat::_444:
      std::cout << "  4:4:4";
      break;
    default:
      THROW("invalid chroma fomat");
    }
    std::cout << std::endl;

    std::cout << std::setw(43) << "Output (internal) ChromaFormatIDC = ";
    switch (encCfg->m_chromaFormatIdc)
    {
    case ChromaFormat::_400:
      std::cout << "  4:0:0";
      break;
    case ChromaFormat::_420:
      std::cout << "  4:2:0";
      break;
    case ChromaFormat::_422:
      std::cout << "  4:2:2";
      break;
    case ChromaFormat::_444:
      std::cout << "  4:4:4";
      break;
    default:
      THROW("invalid chroma fomat");
    }
    std::cout << "\n" << std::endl;
  }
}

//! \}
