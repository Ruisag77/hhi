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

/** \file     EncLib.cpp
    \brief    encoder class
*/
#include "EncLib.h"

#include "EncModeCtrl.h"
#include "AQp.h"
#include "EncCu.h"

#include "CommonLib/Picture.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/ChromaFormat.h"
#include "EncLibCommon.h"
#include "CommonLib/ProfileTierLevel.h"
#include "CommonLib/TimeProfiler.h"

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

EncLib::EncLib(EncLibCommon *encLibCommon)
  : m_cListPic(encLibCommon->getPictureBuffer())
  , m_spsMap(encLibCommon->getSpsMap())
  , m_ppsMap(encLibCommon->getPpsMap())
  , m_apsMaps(encLibCommon->getApsMaps())
  , m_AUWriterIf(nullptr)
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  , m_cacheModel()
#endif
  , m_lmcsAPS(nullptr)
  , m_scalinglistAPS(nullptr)
  , m_doPlt(true)
  , m_vps(&encLibCommon->m_vps)
  , m_layerDecPicBuffering(encLibCommon->getDecPicBuffering())
{
  m_pocLast          = -1;
  m_receivedPicCount = 0;
  m_codedPicCount    = 0;

#if ENABLE_SIMD_OPT_BUFFER
  g_pelBufOP.initPelBufOpsX86();
#endif

#if ENABLE_SIMD_TRAFO
  g_tCoeffOps.initTCoeffOpsX86();
#endif

#if JVET_O0756_CALCULATE_HDRMETRICS
  m_metricTime = std::chrono::milliseconds(0);
#endif

  memset(m_apss, 0, sizeof(m_apss));

  m_layerId    = NOT_VALID;
  m_picIdInGOP = NOT_VALID;
}

EncLib::~EncLib()
{
  if (m_encAlfEcm != nullptr)
  {
    delete m_encAlfEcm;
    m_encAlfEcm = nullptr;
  }
  if (m_encAlfVtm != nullptr)
  {
    delete m_encAlfVtm;
    m_encAlfVtm = nullptr;
  }
}

void EncLib::create(const int layerId)
{
  m_layerId = layerId;
  m_pocLast = m_encCfg.m_compositeRefEnabled ? -2 : -1;

#if ENABLE_TIME_PROFILING
  if (g_allTimeProfilers.empty())
  {
    g_allTimeProfilers.resize(NUMBER_OF_SLICE_TYPES);
    for (auto &p: g_allTimeProfilers)
    {
      p = new TimeProfiler();
    }
    g_timeProfiler = g_allTimeProfilers[I_SLICE];
  }
  msg(INFO, "\n Using runtime profiler\n");
#endif

  // create processing unit classes
  m_bilateralFilter.create();
  m_cGOPEncoder.create();
  m_cCuEncoder.create(&m_encCfg);
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  m_cInterSearch.cacheAssign(&m_cacheModel);
#endif
  m_cEncModeCtrl.create(&m_encCfg);

  m_deblockingFilter.create(floorLog2(m_encCfg.m_CTUSize) - MIN_CU_LOG2);

  if (!m_encCfg.m_deblockingFilterDisable && m_encCfg.m_encDbOpt)
  {
    m_deblockingFilter.initEncPicYuvBuffer(m_encCfg.m_chromaFormatIdc,
                                           Size(m_encCfg.m_sourceWidth, m_encCfg.m_sourceHeight), m_encCfg.m_CTUSize);
  }

  if (m_encCfg.m_lmcsEnabled)
  {
    m_cReshaper.createEnc(m_encCfg.m_sourceWidth, m_encCfg.m_sourceHeight, m_encCfg.m_CTUSize, m_encCfg.m_CTUSize,
                          m_encCfg.m_internalBitDepth[ChannelType::LUMA], isChromaEnabled(m_encCfg.m_chromaFormatIdc));
  }
  if (m_encCfg.m_RCEnableRateControl)
  {
    m_cRateCtrl.init(m_encCfg.m_framesToBeEncoded, m_encCfg.m_RCTargetBitrate,
                     (int)((double)m_encCfg.m_frameRate / m_encCfg.m_temporalSubsampleRatio + 0.5), m_encCfg.m_gopSize,
                     m_encCfg.m_intraPeriod, m_encCfg.m_sourceWidth, m_encCfg.m_sourceHeight, m_encCfg.m_CTUSize,
                     m_encCfg.m_CTUSize, m_encCfg.m_internalBitDepth[ChannelType::LUMA],
                     m_encCfg.m_RCKeepHierarchicalBit, m_encCfg.m_RCUseLCUSeparateModel, m_encCfg.m_GOPList);
  }

  if (m_encCfg.m_alf)
  {
    if (m_encCfg.m_alfImprovements)
    {
      CHECKD(m_encAlfEcm != nullptr, "ECM ALF encoder seems to be already initialized.");
      m_encAlfEcm = new EncAdaptiveLoopFilterEcm;
    }
    else
    {
      CHECKD(m_encAlfVtm != nullptr, "VTM ALF encoder seems to be already initialized.");
      m_encAlfVtm = new EncAdaptiveLoopFilterVtm;
    }
  }
}

void EncLib::destroy()
{
#if ENABLE_TIME_PROFILING
  if (!g_allTimeProfilers.empty())
  {
    TimeProfiler tprofTotal;
    std::cout << std::endl;
    int                                          used                = 0;
    std::array<SliceType, NUMBER_OF_SLICE_TYPES> sliceTypesReordered = { I_SLICE, P_SLICE, B_SLICE, L_SLICE };
    for (size_t i = 0; i < sliceTypesReordered.size(); i++)
    {
      SliceType     s = sliceTypesReordered[i];
      TimeProfiler *p = g_allTimeProfilers[s];
      if (p->isUsed())
      {
        std::cout << "Slices: " << (s == I_SLICE ? "I" : s == B_SLICE ? "B" : s == P_SLICE ? "P" : "L") << std::endl;
        p->output(std::cout);
        tprofTotal += *p;
        used++;
      }
      delete g_allTimeProfilers[s];
    }
    if (used > 1)
    {
      std::cout << "All Slices: " << std::endl;
      tprofTotal.output(std::cout);
    }
    g_timeProfiler = nullptr;
    g_allTimeProfilers.clear();
  }
#endif
  // destroy processing unit classes
  m_cGOPEncoder.destroy();
  m_cSliceEncoder.destroy();
  m_cCuEncoder.destroy();
  if (m_encAlfEcm != nullptr)
  {
    m_encAlfEcm->destroy();
  }
  if (m_encAlfVtm != nullptr)
  {
    m_encAlfVtm->destroy();
  }
  m_cEncSAO.destroyEncData();
  m_cEncSAO.destroy();
  m_deblockingFilter.destroy();
  m_cRateCtrl.destroy();
  m_cReshaper.destroy();
  m_cInterSearch.destroy();
  m_cIntraSearch.destroy();
  m_bilateralFilter.destroy();
#if REUSE_CU_RESULTS
  m_cEncModeCtrl.destroy();
#endif

  for (auto it = m_adaptQPmap.begin(); it != m_adaptQPmap.end(); ++it)
  {
    double *p = it->second;
    delete[] p;
  }
}

void EncLib::init(AUWriterIf *auWriterIf)
{
  m_AUWriterIf = auWriterIf;

  SPS &sps0 =
    *(m_spsMap.allocatePS(m_vps->m_generalLayerIdx[m_layerId]));   // NOTE: implementations that use more than 1 SPS
                                                                   // need to be aware of activation issues.
  PPS &pps0      = *(m_ppsMap.allocatePS(m_vps->m_generalLayerIdx[m_layerId]));
  APS &aps0      = *(m_apsMaps[ApsType::SCALING_LIST].allocatePS(0));
  aps0.m_APSId   = 0;
  aps0.m_APSType = ApsType::SCALING_LIST;

  if (m_encCfg.m_avoidIntraInDepLayer && m_encCfg.m_numRefLayers[m_vps->m_generalLayerIdx[getLayerId()]] > 0)
  {
    m_encCfg.m_idrRefParamList = true;
  }
  // initialize SPS
  xInitSPS(sps0);

  for (int i = 0; i < MAX_TLAYER; i++)
  {
    m_layerDecPicBuffering[m_layerId * MAX_TLAYER + i] = m_encCfg.m_maxDecPicBuffering[i];
  }

  xInitVPS(sps0);
  xInitOPI(m_encCfg.m_opi);
  xInitDCI(m_encCfg.m_dci, sps0);

  if (m_encCfg.m_compositeRefEnabled || m_encCfg.m_seiCfg.m_dependentRAPIndicationSEIEnabled)
  {
    sps0.m_longTermRefsPresent = true;
  }

  if (m_encCfg.m_RCCpbSaturationEnabled)
  {
    m_cRateCtrl.initHrdParam(&sps0.m_generalHrdParams, sps0.m_olsHrdParams, m_encCfg.m_frameRate,
                             m_encCfg.m_RCInitialCpbFullness);
  }
  m_cRdCost.setCostMode(m_encCfg.m_costMode);

  // initialize PPS
  pps0.m_picWidthInLumaSamples  = m_encCfg.m_sourceWidth;
  pps0.m_picHeightInLumaSamples = m_encCfg.m_sourceHeight;
  if (pps0.m_picWidthInLumaSamples == sps0.m_maxWidthInLumaSamples &&
      pps0.m_picHeightInLumaSamples == sps0.m_maxHeightInLumaSamples)
  {
    pps0.m_conformanceWindow     = sps0.m_conformanceWindow;
    pps0.m_conformanceWindowFlag = false;
  }
  else
  {
    pps0.m_conformanceWindow     = m_encCfg.m_conformanceWindow;
    pps0.m_conformanceWindowFlag = m_encCfg.m_conformanceWindow.m_enabledFlag;
  }
  if (!pps0.m_explicitScalingWindowFlag)
  {
    pps0.m_scalingWindow = pps0.m_conformanceWindow;
  }
  xInitPPS(pps0, sps0);
  // initialize APS
  xInitRPL(sps0);

  if (m_encCfg.m_resChangeInClvsEnabled)
  {
    PPS    &pps                = *(m_ppsMap.allocatePS(ENC_PPS_ID_RPR));
    Window &inputScalingWindow = pps0.m_scalingWindow;
    int     scaledWidth        = int((pps0.m_picWidthInLumaSamples -
                           SPS::getWinUnitX(sps0.m_chromaFormatIdc) *
                             (inputScalingWindow.m_winLeftOffset + inputScalingWindow.m_winRightOffset)) /
                          m_encCfg.m_scalingRatioHor);
    int     minSizeUnit        = std::max(8, 1 << sps0.m_log2MinCodingBlockSize);
    int     temp               = scaledWidth / minSizeUnit;
    int     width              = (scaledWidth - (temp * minSizeUnit) > 0 ? temp + 1 : temp) * minSizeUnit;

    int scaledHeight = int((pps0.m_picHeightInLumaSamples -
                            SPS::getWinUnitY(sps0.m_chromaFormatIdc) *
                              (inputScalingWindow.m_winTopOffset + inputScalingWindow.m_winBottomOffset)) /
                           m_encCfg.m_scalingRatioVer);
    temp             = scaledHeight / minSizeUnit;
    int height       = (scaledHeight - (temp * minSizeUnit) > 0 ? temp + 1 : temp) * minSizeUnit;

    pps.m_picWidthInLumaSamples  = width;
    pps.m_picHeightInLumaSamples = height;
    pps.m_sliceChromaQpFlag      = true;
    Window conformanceWindow;
    conformanceWindow.setWindow(0, (width - scaledWidth) / SPS::getWinUnitX(sps0.m_chromaFormatIdc), 0,
                                (height - scaledHeight) / SPS::getWinUnitY(sps0.m_chromaFormatIdc));
    if (pps.m_picWidthInLumaSamples == sps0.m_maxWidthInLumaSamples &&
        pps.m_picHeightInLumaSamples == sps0.m_maxHeightInLumaSamples)
    {
      pps.m_conformanceWindow     = sps0.m_conformanceWindow;
      pps.m_conformanceWindowFlag = false;
    }
    else
    {
      pps.m_conformanceWindow     = conformanceWindow;
      pps.m_conformanceWindowFlag = conformanceWindow.m_enabledFlag;
    }

    Window scalingWindow;
    scalingWindow.setWindow(0, (width - scaledWidth) / SPS::getWinUnitX(sps0.m_chromaFormatIdc), 0,
                            (height - scaledHeight) / SPS::getWinUnitY(sps0.m_chromaFormatIdc));
    pps.m_scalingWindow             = scalingWindow;
    pps.m_explicitScalingWindowFlag = scalingWindow.m_enabledFlag;

    // register the width/height of the current pic into reference SPS
    if (!sps0.m_ppsValidFlag[pps.m_ppsId])
    {
      sps0.m_ppsValidFlag[pps.m_ppsId]                  = true;
      sps0.m_scalingWindowSizeInPPS[pps.m_ppsId].width  = scaledWidth;
      sps0.m_scalingWindowSizeInPPS[pps.m_ppsId].height = scaledHeight;
    }
    int curSeqMaxPicWidthY  = sps0.m_maxWidthInLumaSamples;   // sps_pic_width_max_in_luma_samples
    int curSeqMaxPicHeightY = sps0.m_maxHeightInLumaSamples;   // sps_pic_height_max_in_luma_samples
    int curPicWidthY        = width;   // pps_pic_width_in_luma_samples
    int curPicHeightY       = height;   // pps_pic_height_in_luma_samples
    int max8MinCbSizeY      = std::max((int)8, (1 << sps0.m_log2MinCodingBlockSize));   // Max(8, MinCbSizeY)
    // Warning message of potential scaling window size violation
    for (int i = 0; i < MAX_NUM_PPS; i++)
    {
      if (sps0.m_ppsValidFlag[i])
      {
        if ((scaledWidth * curSeqMaxPicWidthY) <
            sps0.m_scalingWindowSizeInPPS[i].width * (curPicWidthY - max8MinCbSizeY))
        {
          printf("Potential violation: (curScaledWIdth * curSeqMaxPicWidthY) should be greater than or equal to "
                 "refScaledWidth * (curPicWidthY - max(8, MinCbSizeY)\n");
        }
        if ((scaledHeight * curSeqMaxPicHeightY) <
            sps0.m_scalingWindowSizeInPPS[i].height * (curPicHeightY - max8MinCbSizeY))
        {
          printf("Potential violation: (curScaledHeight * curSeqMaxPicHeightY) should be greater than or equal to "
                 "refScaledHeight * (curPicHeightY - max(8, MinCbSizeY)\n");
        }
      }
    }

    // disable picture partitioning for scaled RPR pictures (slice/tile config only provided for the original
    // resolution)
    m_encCfg.m_picPartitionFlag = false;

    xInitPPS(pps, sps0);   // will allocate memory for and initialize pps.pcv inside

    if (pps.m_wrapAroundEnabledFlag)
    {
      const int minCbSizeY = 1 << sps0.m_log2MinCodingBlockSize;
      pps.m_picWidthMinusWrapAroundOffset =
        ((pps.m_picWidthInLumaSamples / minCbSizeY) -
         (m_encCfg.m_wrapAroundOffset * pps.m_picWidthInLumaSamples / pps0.m_picWidthInLumaSamples / minCbSizeY));
      pps.m_wrapAroundOffset =
        minCbSizeY * (pps.m_picWidthInLumaSamples / minCbSizeY - pps.m_picWidthMinusWrapAroundOffset);
    }
    else
    {
      pps.m_picWidthMinusWrapAroundOffset = 0;
      pps.m_wrapAroundOffset              = 0;
    }
  }
  if (m_encCfg.m_resChangeInClvsEnabled &&
      ((m_encCfg.m_gopBasedRPREnabledFlag && (m_encCfg.m_iQP >= m_encCfg.m_gopBasedRPRQPThreshold)) ||
       m_encCfg.m_rprFunctionalityTestingEnabledFlag))
  {
    PPS    &pps                = *(m_ppsMap.allocatePS(ENC_PPS_ID_RPR2));
    Window &inputScalingWindow = pps0.m_scalingWindow;
    int     scaledWidth        = int((pps0.m_picWidthInLumaSamples -
                           SPS::getWinUnitX(sps0.m_chromaFormatIdc) *
                             (inputScalingWindow.m_winLeftOffset + inputScalingWindow.m_winRightOffset)) /
                          m_encCfg.m_scalingRatioHor2);
    int     minSizeUnit        = std::max(8, 1 << sps0.m_log2MinCodingBlockSize);
    int     temp               = scaledWidth / minSizeUnit;
    int     width              = (scaledWidth - (temp * minSizeUnit) > 0 ? temp + 1 : temp) * minSizeUnit;

    int scaledHeight = int((pps0.m_picHeightInLumaSamples -
                            SPS::getWinUnitY(sps0.m_chromaFormatIdc) *
                              (inputScalingWindow.m_winTopOffset + inputScalingWindow.m_winBottomOffset)) /
                           m_encCfg.m_scalingRatioVer2);
    temp             = scaledHeight / minSizeUnit;
    int height       = (scaledHeight - (temp * minSizeUnit) > 0 ? temp + 1 : temp) * minSizeUnit;

    pps.m_picWidthInLumaSamples  = width;
    pps.m_picHeightInLumaSamples = height;
    pps.m_sliceChromaQpFlag      = true;

    Window conformanceWindow;
    conformanceWindow.setWindow(0, (width - scaledWidth) / SPS::getWinUnitX(sps0.m_chromaFormatIdc), 0,
                                (height - scaledHeight) / SPS::getWinUnitY(sps0.m_chromaFormatIdc));
    if (pps.m_picWidthInLumaSamples == sps0.m_maxWidthInLumaSamples &&
        pps.m_picHeightInLumaSamples == sps0.m_maxHeightInLumaSamples)
    {
      pps.m_conformanceWindow     = sps0.m_conformanceWindow;
      pps.m_conformanceWindowFlag = false;
    }
    else
    {
      pps.m_conformanceWindow     = conformanceWindow;
      pps.m_conformanceWindowFlag = conformanceWindow.m_enabledFlag;
    }

    Window scalingWindow;
    scalingWindow.setWindow(0, (width - scaledWidth) / SPS::getWinUnitX(sps0.m_chromaFormatIdc), 0,
                            (height - scaledHeight) / SPS::getWinUnitY(sps0.m_chromaFormatIdc));
    pps.m_scalingWindow = scalingWindow;

    // register the width/height of the current pic into reference SPS
    if (!sps0.m_ppsValidFlag[pps.m_ppsId])
    {
      sps0.m_ppsValidFlag[pps.m_ppsId]                  = true;
      sps0.m_scalingWindowSizeInPPS[pps.m_ppsId].width  = scaledWidth;
      sps0.m_scalingWindowSizeInPPS[pps.m_ppsId].height = scaledHeight;
    }
    int curSeqMaxPicWidthY  = sps0.m_maxWidthInLumaSamples;   // sps_pic_width_max_in_luma_samples
    int curSeqMaxPicHeightY = sps0.m_maxHeightInLumaSamples;   // sps_pic_height_max_in_luma_samples
    int curPicWidthY        = width;   // pps_pic_width_in_luma_samples
    int curPicHeightY       = height;   // pps_pic_height_in_luma_samples
    int max8MinCbSizeY      = std::max(
      (int)8, (1 << sps0.m_log2MinCodingBlockSize));   // Max(8, MinCbSizeY)
                                                       // Warning message of potential scaling window size violation
    for (int i = 0; i < MAX_NUM_PPS; i++)
    {
      if (sps0.m_ppsValidFlag[i])
      {
        if ((scaledWidth * curSeqMaxPicWidthY) <
            sps0.m_scalingWindowSizeInPPS[i].width * (curPicWidthY - max8MinCbSizeY))
        {
          printf("Potential violation: (curScaledWIdth * curSeqMaxPicWidthY) should be greater than or equal to "
                 "refScaledWidth * (curPicWidthY - max(8, MinCbSizeY)\n");
        }
        if ((scaledHeight * curSeqMaxPicHeightY) <
            sps0.m_scalingWindowSizeInPPS[i].height * (curPicHeightY - max8MinCbSizeY))
        {
          printf("Potential violation: (curScaledHeight * curSeqMaxPicHeightY) should be greater than or equal to "
                 "refScaledHeight * (curPicHeightY - max(8, MinCbSizeY)\n");
        }
      }
    }

    // disable picture partitioning for scaled RPR pictures (slice/tile config only provided for the original
    // resolution)
    m_encCfg.m_picPartitionFlag = false;

    xInitPPS(pps, sps0);   // will allocate memory for and initialize pps.pcv inside

    if (pps.m_wrapAroundEnabledFlag)
    {
      int minCbSizeY                      = (1 << sps0.m_log2MinCodingBlockSize);
      pps.m_picWidthMinusWrapAroundOffset = (pps.m_picWidthInLumaSamples / minCbSizeY) -
        (m_encCfg.m_wrapAroundOffset * pps.m_picWidthInLumaSamples / pps0.m_picWidthInLumaSamples / minCbSizeY);
      pps.m_wrapAroundOffset =
        minCbSizeY * (pps.m_picWidthInLumaSamples / minCbSizeY - pps.m_picWidthMinusWrapAroundOffset);
    }
    else
    {
      pps.m_picWidthMinusWrapAroundOffset = 0;
      pps.m_wrapAroundOffset              = 0;
    }
  }
  if (m_encCfg.m_resChangeInClvsEnabled &&
      ((m_encCfg.m_gopBasedRPREnabledFlag && (m_encCfg.m_iQP >= m_encCfg.m_gopBasedRPRQPThreshold)) ||
       m_encCfg.m_rprFunctionalityTestingEnabledFlag))
  {
    PPS    &pps                = *(m_ppsMap.allocatePS(ENC_PPS_ID_RPR3));
    Window &inputScalingWindow = pps0.m_scalingWindow;
    int     scaledWidth        = int((pps0.m_picWidthInLumaSamples -
                           SPS::getWinUnitX(sps0.m_chromaFormatIdc) *
                             (inputScalingWindow.m_winLeftOffset + inputScalingWindow.m_winRightOffset)) /
                          m_encCfg.m_scalingRatioHor3);
    int     minSizeUnit        = std::max(8, 1 << sps0.m_log2MinCodingBlockSize);
    int     temp               = scaledWidth / minSizeUnit;
    int     width              = (scaledWidth - (temp * minSizeUnit) > 0 ? temp + 1 : temp) * minSizeUnit;

    int scaledHeight = int((pps0.m_picHeightInLumaSamples -
                            SPS::getWinUnitY(sps0.m_chromaFormatIdc) *
                              (inputScalingWindow.m_winTopOffset + inputScalingWindow.m_winBottomOffset)) /
                           m_encCfg.m_scalingRatioVer3);
    temp             = scaledHeight / minSizeUnit;
    int height       = (scaledHeight - (temp * minSizeUnit) > 0 ? temp + 1 : temp) * minSizeUnit;

    pps.m_picWidthInLumaSamples  = width;
    pps.m_picHeightInLumaSamples = height;
    pps.m_sliceChromaQpFlag      = true;

    Window conformanceWindow;
    conformanceWindow.setWindow(0, (width - scaledWidth) / SPS::getWinUnitX(sps0.m_chromaFormatIdc), 0,
                                (height - scaledHeight) / SPS::getWinUnitY(sps0.m_chromaFormatIdc));
    if (pps.m_picWidthInLumaSamples == sps0.m_maxWidthInLumaSamples &&
        pps.m_picHeightInLumaSamples == sps0.m_maxHeightInLumaSamples)
    {
      pps.m_conformanceWindow     = sps0.m_conformanceWindow;
      pps.m_conformanceWindowFlag = false;
    }
    else
    {
      pps.m_conformanceWindow     = conformanceWindow;
      pps.m_conformanceWindowFlag = conformanceWindow.m_enabledFlag;
    }

    Window scalingWindow;
    scalingWindow.setWindow(0, (width - scaledWidth) / SPS::getWinUnitX(sps0.m_chromaFormatIdc), 0,
                            (height - scaledHeight) / SPS::getWinUnitY(sps0.m_chromaFormatIdc));
    pps.m_scalingWindow = scalingWindow;

    // register the width/height of the current pic into reference SPS
    if (!sps0.m_ppsValidFlag[pps.m_ppsId])
    {
      sps0.m_ppsValidFlag[pps.m_ppsId]                  = true;
      sps0.m_scalingWindowSizeInPPS[pps.m_ppsId].width  = scaledWidth;
      sps0.m_scalingWindowSizeInPPS[pps.m_ppsId].height = scaledHeight;
    }
    int curSeqMaxPicWidthY  = sps0.m_maxWidthInLumaSamples;   // sps_pic_width_max_in_luma_samples
    int curSeqMaxPicHeightY = sps0.m_maxHeightInLumaSamples;   // sps_pic_height_max_in_luma_samples
    int curPicWidthY        = width;   // pps_pic_width_in_luma_samples
    int curPicHeightY       = height;   // pps_pic_height_in_luma_samples
    int max8MinCbSizeY      = std::max(
      (int)8, (1 << sps0.m_log2MinCodingBlockSize));   // Max(8, MinCbSizeY)
                                                       // Warning message of potential scaling window size violation
    for (int i = 0; i < MAX_NUM_PPS; i++)
    {
      if (sps0.m_ppsValidFlag[i])
      {
        if ((scaledWidth * curSeqMaxPicWidthY) <
            sps0.m_scalingWindowSizeInPPS[i].width * (curPicWidthY - max8MinCbSizeY))
        {
          printf("Potential violation: (curScaledWIdth * curSeqMaxPicWidthY) should be greater than or equal to "
                 "refScaledWidth * (curPicWidthY - max(8, MinCbSizeY)\n");
        }
        if ((scaledHeight * curSeqMaxPicHeightY) <
            sps0.m_scalingWindowSizeInPPS[i].height * (curPicHeightY - max8MinCbSizeY))
        {
          printf("Potential violation: (curScaledHeight * curSeqMaxPicHeightY) should be greater than or equal to "
                 "refScaledHeight * (curPicHeightY - max(8, MinCbSizeY)\n");
        }
      }
    }

    // disable picture partitioning for scaled RPR pictures (slice/tile config only provided for the original
    // resolution)
    m_encCfg.m_picPartitionFlag = false;

    xInitPPS(pps, sps0);   // will allocate memory for and initialize pps.pcv inside

    if (pps.m_wrapAroundEnabledFlag)
    {
      const int minCbSizeY                = 1 << sps0.m_log2MinCodingBlockSize;
      pps.m_picWidthMinusWrapAroundOffset = (pps.m_picWidthInLumaSamples / minCbSizeY) -
        (m_encCfg.m_wrapAroundOffset * pps.m_picWidthInLumaSamples / pps0.m_picWidthInLumaSamples / minCbSizeY);
      pps.m_wrapAroundOffset =
        minCbSizeY * (pps.m_picWidthInLumaSamples / minCbSizeY - pps.m_picWidthMinusWrapAroundOffset);
    }
    else
    {
      pps.m_picWidthMinusWrapAroundOffset = 0;
      pps.m_wrapAroundOffset              = 0;
    }
  }

#if ER_CHROMA_QP_WCG_PPS
  if (m_encCfg.m_wcgChromaQpControl.enabled)
  {
    PPS &pps1 = *(m_ppsMap.allocatePS(1));
    xInitPPS(pps1, sps0);
  }
#endif
  if (m_encCfg.m_compositeRefEnabled)
  {
    PPS &pps2 = *(m_ppsMap.allocatePS(2));
    xInitPPS(pps2, sps0);
    xInitPPSforLT(pps2);
  }
  if (this->m_encCfg.m_rprRASLtoolSwitch && m_encCfg.m_wrapAround)
  {
    PPS &pps4                     = *(m_ppsMap.allocatePS(4));
    pps4.m_picWidthInLumaSamples  = pps0.m_picWidthInLumaSamples;
    pps4.m_picHeightInLumaSamples = pps0.m_picHeightInLumaSamples;
    xInitPPS(pps4, sps0);
    pps4.m_wrapAroundEnabledFlag         = false;
    pps4.m_picWidthMinusWrapAroundOffset = 0;
    pps4.m_wrapAroundOffset              = 0;
  }
  xInitPicHeader(m_picHeader, sps0, pps0);

  // initialize processing unit classes
  m_cEncModeCtrl.init(&m_encCfg, &m_cRateCtrl, &m_cRdCost, &m_adaptQPmap);
  m_cGOPEncoder.init(this, &m_cEncModeCtrl);
  m_cSliceEncoder.init(this, sps0);
  m_cCuEncoder.init(this, &m_cEncModeCtrl, sps0);
  m_if.initInterpolationFilter(true);

  // initialize transform & quantization class
  m_cTrQuant.init(nullptr, 1 << m_encCfg.m_log2MaxTbSize, m_encCfg.m_useRDOQ, m_encCfg.m_useRDOQTS,
                  m_encCfg.m_useSelectiveRDOQ, true);

  // initialize encoder search class
  CABACWriter *cabacEstimator = m_CABACEncoder.getCABACEstimator(&sps0);
  m_cIntraSearch.init(&m_encCfg, &m_bilateralFilter, &m_cTrQuant, &m_cRdCost, &m_if, cabacEstimator, &m_cEncModeCtrl,
                      getCtxCache(), m_encCfg.m_CTUSize, m_encCfg.m_CTUSize,
                      floorLog2(m_encCfg.m_CTUSize) - m_encCfg.m_log2MinCUSize, &m_cReshaper,
                      sps0.m_bitDepths[ChannelType::LUMA]);
  m_cInterSearch.init(&m_encCfg, &m_bilateralFilter, &m_cTrQuant, &m_cEncModeCtrl, m_encCfg.m_searchRange,
                      m_encCfg.m_bipredSearchRange, m_encCfg.m_motionEstimationSearchMethod,
                      m_encCfg.m_compositeRefEnabled, m_encCfg.m_CTUSize, m_encCfg.m_CTUSize,
                      floorLog2(m_encCfg.m_CTUSize) - m_encCfg.m_log2MinCUSize, &m_cRdCost, cabacEstimator,
                      getCtxCache(), &m_cReshaper, pps0.m_picWidthInLumaSamples, &m_if);

  // link temporary buffets from intra search with inter search to avoid unneccessary memory overhead
  m_cInterSearch.setTempBuffers(m_cIntraSearch.getSaveCSBuf());

#if ER_CHROMA_QP_WCG_PPS
  if (m_encCfg.m_wcgChromaQpControl.enabled)
  {
    xInitScalingLists(sps0, *m_apsMaps[ApsType::SCALING_LIST].getPS(1));
    xInitScalingLists(sps0, aps0);
  }
  else
#endif
  {
    xInitScalingLists(sps0, aps0);
  }
  if (m_encCfg.m_resChangeInClvsEnabled)
  {
    xInitScalingLists(sps0, *m_apsMaps[ApsType::SCALING_LIST].getPS(ENC_PPS_ID_RPR));
  }
  if (m_encCfg.m_compositeRefEnabled)
  {
    Picture *picBg = new Picture;

#if !ENABLE_POST_CFE_CHANGES
    picBg->create(sps0.m_chromaFormatIdc, Size(pps0.m_picWidthInLumaSamples, pps0.m_picHeightInLumaSamples),
                  sps0.m_maxCuWidth, sps0.m_maxCuWidth + 16, false, m_layerId, sps0.m_rprEnabledFlag,
                  m_encCfg.m_gopBasedTemporalFilterEnabled, false
#else
    picBg->create(sps0.m_chromaFormatIdc, Size(pps0.m_picWidthInLumaSamples, pps0.m_picHeightInLumaSamples),
                  sps0.m_maxCuWidth, sps0.m_maxCuWidth + EXT_PICTURE_SIZE, false, m_layerId, sps0.m_rprEnabledFlag,
                  m_encCfg.m_gopBasedTemporalFilterEnabled, false
#endif
#if JVET_Z0120_SII_SEI_PROCESSING
                  ,
                  false
#endif
#if ENABLE_NNLF
                  ,
                  false
#endif
    );

    picBg->getRecoBuf().fill(0);
    picBg->finalInit(m_vps, sps0, pps0, &m_picHeader, m_apss, m_lmcsAPS, m_scalinglistAPS);
    picBg->allocateNewSlice();
    picBg->createSpliceIdx(pps0.pcv->sizeInCtus);
    m_cGOPEncoder.setPicBg(picBg);
    Picture *picOrig = new Picture;
#if !ENABLE_POST_CFE_CHANGES
    picOrig->create(sps0.m_chromaFormatIdc, Size(pps0.m_picWidthInLumaSamples, pps0.m_picHeightInLumaSamples),
                    sps0.m_maxCuWidth, sps0.m_maxCuWidth + 16, false, m_layerId, sps0.m_rprEnabledFlag,
                    m_encCfg.m_gopBasedTemporalFilterEnabled, false
#else
    picOrig->create(sps0.m_chromaFormatIdc, Size(pps0.m_picWidthInLumaSamples, pps0.m_picHeightInLumaSamples),
                    sps0.m_maxCuWidth, sps0.m_maxCuWidth + EXT_PICTURE_SIZE, false, m_layerId, sps0.m_rprEnabledFlag,
                    m_encCfg.m_gopBasedTemporalFilterEnabled, false
#endif
#if JVET_Z0120_SII_SEI_PROCESSING
                    ,
                    false
#endif
#if ENABLE_NNLF
                    ,
                    false
#endif
    );

    picOrig->getOrigBuf().fill(0);
    m_cGOPEncoder.setPicOrig(picOrig);
  }
}

void EncLib::xInitScalingLists(SPS &sps, APS &aps)
{
  // Initialise scaling lists
  // The encoder will only use the SPS scaling lists. The PPS will never be marked present.
  const int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE] = { sps.getMaxLog2TrDynamicRange(ChannelType::LUMA),
                                                            sps.getMaxLog2TrDynamicRange(ChannelType::CHROMA) };

  Quant *quant = getTrQuant()->getQuant();

  if (m_encCfg.m_useScalingListId == SCALING_LIST_OFF)
  {
    quant->setFlatScalingList(maxLog2TrDynamicRange, sps.m_bitDepths);
    quant->setUseScalingList(false);
  }
  else if (m_encCfg.m_useScalingListId == SCALING_LIST_DEFAULT)
  {
    aps.m_scalingListApsInfo.setDefaultScalingList();
    quant->setScalingList(&(aps.m_scalingListApsInfo), maxLog2TrDynamicRange, sps.m_bitDepths);
    quant->setUseScalingList(true);
  }
  else if (m_encCfg.m_useScalingListId == SCALING_LIST_FILE_READ)
  {
    aps.m_scalingListApsInfo.setDefaultScalingList();
    CHECK(aps.m_scalingListApsInfo.xParseScalingList(m_encCfg.m_scalingListFileName),
          "Error Parsing Scaling List Input File");
    aps.m_scalingListApsInfo.checkDcOfMatrix();
    if (!aps.m_scalingListApsInfo.isNotDefaultScalingList())
    {
      m_encCfg.m_useScalingListId = SCALING_LIST_DEFAULT;
    }
    aps.m_scalingListApsInfo.m_chromaScalingListPresentFlag = isChromaEnabled(sps.m_chromaFormatIdc);
    quant->setScalingList(&(aps.m_scalingListApsInfo), maxLog2TrDynamicRange, sps.m_bitDepths);
    quant->setUseScalingList(true);

    sps.m_disableScalingMatrixForLfnstBlks = m_encCfg.m_disableScalingMatrixForLfnstBlks;
  }
  else
  {
    THROW("error : ScalingList == " << m_encCfg.m_useScalingListId << " not supported\n");
  }

  if (m_encCfg.m_useScalingListId == SCALING_LIST_FILE_READ)
  {
    // Prepare delta's:
    for (uint32_t scalingListId = 0; scalingListId < 28; scalingListId++)
    {
      if (aps.m_scalingListApsInfo.m_chromaScalingListPresentFlag ||
          aps.m_scalingListApsInfo.isLumaScalingList(scalingListId))
      {
        aps.m_scalingListApsInfo.checkPredMode(scalingListId);
      }
    }
  }
}

void EncLib::xInitPPSforLT(PPS &pps)
{
  pps.m_outputFlagPresentFlag              = true;
  pps.m_deblockingFilterControlPresentFlag = true;
  pps.m_ppsDeblockingFilterDisabledFlag    = true;
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void EncLib::deletePicBuffer()
{
  PicList::iterator iterPic = m_cListPic.begin();
  int               size    = int(m_cListPic.size());

  for (int i = 0; i < size; i++)
  {
    Picture *pic = *(iterPic++);

    pic->destroy();

    // get rid of the qpadaption layer
    while (pic->m_aqlayer.size())
    {
      delete pic->m_aqlayer.back();
      pic->m_aqlayer.pop_back();
    }

    delete pic;
    pic = nullptr;
  }

  m_cListPic.clear();
}

bool EncLib::encodePrep(bool flush, PelStorage *picYuvOrg, PelStorage *cPicYuvTrueOrg, PelStorage *picYuvFilteredOrg,
                        PelStorage *picYuvFilteredOrgForFG, const InputColourSpaceConversion snrCSC,
                        std::list<PelUnitBuf *> &rcListPicYuvRecOut, int &numEncoded, PelStorage **ppicYuvRPR)
{
  if (m_encCfg.m_compositeRefEnabled && m_cGOPEncoder.getPicBg()->getSpliceFull() && m_pocLast >= 10 &&
      m_receivedPicCount == 0 && m_cGOPEncoder.getEncodedLTRef() == false)
  {
    Picture *picCurr = nullptr;
    xGetNewPicBuffer(rcListPicYuvRecOut, picCurr, 2);
    const PPS *pps = m_ppsMap.getPS(2);
    const SPS *sps = m_spsMap.getPS(pps->m_spsId);

    picCurr->m_bufs[PIC_ORIGINAL].copyFrom(m_cGOPEncoder.getPicBg()->getRecoBuf());
    picCurr->finalInit(m_vps, *sps, *pps, &m_picHeader, m_apss, m_lmcsAPS, m_scalinglistAPS);
    picCurr->m_poc = m_pocLast - 1;
    m_pocLast -= 2;

#if JVET_Z0120_SII_SEI_PROCESSING
    if (m_encCfg.m_seiCfg.m_ShutterFilterEnable)
    {
      int blendingRatio = m_encCfg.m_seiCfg.m_SII_BlendingRatio;
      picCurr->xOutputPreFilteredPic(picCurr, &m_cListPic, blendingRatio, m_encCfg.m_intraPeriod);
      picCurr->copyToPic(sps, &picCurr->m_bufs[PIC_ORIGINAL], picYuvOrg);
    }
#endif

    if (m_encCfg.m_bUseAdaptiveQP)
    {
      AQpPreanalyzer::preanalyze(picCurr);
    }
    if (m_encCfg.m_RCEnableRateControl)
    {
      m_cRateCtrl.initRCGOP(m_receivedPicCount);
    }

    m_cGOPEncoder.compressGOP(m_pocLast, m_receivedPicCount, m_cListPic, rcListPicYuvRecOut, false, false, snrCSC,
                              m_encCfg.m_printFrameMSE, m_encCfg.m_printMSSSIM, true, 0);

#if JVET_O0756_CALCULATE_HDRMETRICS
    m_metricTime = m_cGOPEncoder.getMetricTime();
#endif
    m_cGOPEncoder.setEncodedLTRef(true);
    if (m_encCfg.m_RCEnableRateControl)
    {
      m_cRateCtrl.destroyRCGOP();
    }

    numEncoded         = 0;
    m_receivedPicCount = 0;
  }

  if (picYuvOrg != nullptr)
  {
    // get original YUV
    Picture *picCurr = nullptr;

    int ppsID = -1;   // Use default PPS ID
#if ER_CHROMA_QP_WCG_PPS
    if (m_encCfg.m_wcgChromaQpControl.enabled)
    {
      ppsID = m_encCfg.m_frameDeltaQps[m_pocLast / (m_encCfg.m_compositeRefEnabled ? 2 : 1) + 1];
      ppsID += (m_encCfg.m_switchPOC != -1 && (m_pocLast + 1 >= m_encCfg.m_switchPOC) ? 1 : 0);
    }
#endif

    if (m_encCfg.m_resChangeInClvsEnabled && m_encCfg.m_gopBasedRPREnabledFlag &&
        (m_encCfg.m_iQP >= m_encCfg.m_gopBasedRPRQPThreshold))
    {
      const int poc                = m_pocLast + (m_encCfg.m_compositeRefEnabled ? 2 : 1);
      double    upscaledPSNR       = 0.0;
      double    upscaledPSNRcb     = 0.0;
      double    upscaledPSNRcr     = 0.0;
      double    upscaledPSNRchroma = 0.0;
      double    upscaledPSNRlim    = 0.0;
      if (poc % m_encCfg.m_gopSize == 0)
      {
        ScalingRatio downScalingRatio { 32768, 32768 };
        ScalingRatio upScalingRatio { 8192, 8192 };

        const PPS         *orgPPS      = m_ppsMap.getPS(0);
        const SPS         *orgSPS      = m_spsMap.getPS(orgPPS->m_spsId);
        const ChromaFormat chFormatIdc = orgSPS->m_chromaFormatIdc;

        const PPS *pTempPPS = m_ppsMap.getPS(ENC_PPS_ID_RPR);
        Picture::rescalePicture(downScalingRatio, *picYuvOrg, orgPPS->m_scalingWindow, *ppicYuvRPR[1],
                                pTempPPS->m_scalingWindow, chFormatIdc, orgSPS->m_bitDepths, true, true,
                                orgSPS->m_horCollocatedChromaFlag, orgSPS->m_verCollocatedChromaFlag);
        Picture::rescalePicture(upScalingRatio, *ppicYuvRPR[1], orgPPS->m_scalingWindow, *ppicYuvRPR[0],
                                pTempPPS->m_scalingWindow, chFormatIdc, orgSPS->m_bitDepths, true, false,
                                orgSPS->m_horCollocatedChromaFlag, orgSPS->m_verCollocatedChromaFlag);
        // Calculate PSNR
        uint64_t totalDiff[3] = { 0, 0, 0 };
        for (int i = 0; i < ::getNumberValidComponents(chFormatIdc); i++)
        {
          const Pel *pSrc0 = picYuvOrg->get((CompID)i).bufAt(0, 0);
          const Pel *pSrc1 = ppicYuvRPR[0]->get((CompID)i).bufAt(0, 0);

          // uint64_t totalDiff = 0;
          for (int y = 0; y < picYuvOrg->get((CompID)i).height; y++)
          {
            for (int x = 0; x < picYuvOrg->get((CompID)i).width; x++)
            {
              int diff = pSrc0[x] - pSrc1[x];
              totalDiff[i] += uint64_t(diff) * uint64_t(diff);
            }
            pSrc0 += picYuvOrg->get((CompID)i).stride;
            pSrc1 += ppicYuvRPR[0]->get((CompID)i).stride;
          }
        }
        const uint32_t maxval = 255 << (orgSPS->m_bitDepths[ChannelType::LUMA] - 8);
        upscaledPSNRcb        = totalDiff[1] ? 10.0 *
            log10((double)maxval * maxval * picYuvOrg->get((CompID)1).width * picYuvOrg->get((CompID)1).height /
                         (double)totalDiff[1])
                                             : 999.99;
        upscaledPSNRcr        = totalDiff[2] ? 10.0 *
            log10((double)maxval * maxval * picYuvOrg->get((CompID)2).width * picYuvOrg->get((CompID)2).height /
                         (double)totalDiff[2])
                                             : 999.99;
        upscaledPSNRchroma    = m_encCfg.m_psnrChromaOffsetRPR + std::min(upscaledPSNRcb, upscaledPSNRcr);
        upscaledPSNR          = totalDiff[0] ? 10.0 *
            log10((double)maxval * maxval * orgPPS->m_picWidthInLumaSamples * orgPPS->m_picHeightInLumaSamples /
                           (double)totalDiff[0])
                                             : 999.99;
        upscaledPSNRlim       = std::min(upscaledPSNRchroma, upscaledPSNR);
      }

      if (poc % m_encCfg.m_gopSize == 0)
      {
        const int qpBias = 37;
        if ((m_encCfg.m_psnrThresholdRPR - (m_encCfg.m_iQP - qpBias) * 0.5) < upscaledPSNRlim)
        {
          ppsID = ENC_PPS_ID_RPR;
        }
        else
        {
          if ((m_encCfg.m_psnrThresholdRPR2 - (m_encCfg.m_iQP - qpBias) * 0.5) < upscaledPSNRlim)
          {
            ppsID = ENC_PPS_ID_RPR2;
          }
          else
          {
            if ((m_encCfg.m_psnrThresholdRPR3 - (m_encCfg.m_iQP - qpBias) * 0.5) < upscaledPSNRlim)
            {
              ppsID = ENC_PPS_ID_RPR3;
            }
            else
            {
              ppsID = 0;
            }
          }
        }
        m_gopRprPpsId = ppsID;
      }
      else
      {
        ppsID = m_gopRprPpsId;
      }
    }

    if (m_encCfg.m_resChangeInClvsEnabled && m_encCfg.m_rprFunctionalityTestingEnabledFlag)
    {
      const int poc = m_pocLast + (m_encCfg.m_compositeRefEnabled ? 2 : 1);
      if (poc % m_encCfg.m_rprSwitchingSegmentSize == 0)
      {
        ppsID           = 0;
        bool applyRpr   = false;
        int  currPoc    = poc + m_encCfg.m_frameSkip;
        int  rprSegment = currPoc / m_encCfg.m_rprSwitchingSegmentSize % m_encCfg.m_rprSwitchingListSize;
        int  thePPSID   = RPR_PPS_ID[m_encCfg.m_rprSwitchingResolutionOrderList[rprSegment]];
        applyRpr        = thePPSID != 0;
        if (applyRpr)
        {
          ppsID = thePPSID;
        }
        m_gopRprPpsId = ppsID;
      }
      else
      {
        ppsID = m_gopRprPpsId;
      }
    }
    if (m_encCfg.m_resChangeInClvsEnabled && m_encCfg.m_intraPeriod == -1 && !m_encCfg.m_gopBasedRPREnabledFlag &&
        !m_encCfg.m_rprFunctionalityTestingEnabledFlag)
    {
      const int poc = m_pocLast + (m_encCfg.m_compositeRefEnabled ? 2 : 1);

      if (poc / m_encCfg.m_switchPocPeriod % 2)
      {
        ppsID = ENC_PPS_ID_RPR;
      }
      else
      {
        ppsID = 0;
      }
    }
    if (m_vps->m_maxLayers > 1)
    {
      ppsID = m_vps->m_generalLayerIdx[m_layerId];
    }

    xGetNewPicBuffer(rcListPicYuvRecOut, picCurr, ppsID);

    const PPS *pPPS = (ppsID < 0) ? m_ppsMap.getFirstPS() : m_ppsMap.getPS(ppsID);
    const SPS *pSPS = m_spsMap.getPS(pPPS->m_spsId);

    const ChromaFormat chromaFormatIdc = pSPS->m_chromaFormatIdc;

    if (m_encCfg.m_resChangeInClvsEnabled)
    {
      picCurr->m_bufs[PIC_ORIGINAL_INPUT].getBuf(COMP_Y).copyFrom(picYuvOrg->getBuf(COMP_Y));

      picCurr->m_bufs[PIC_TRUE_ORIGINAL_INPUT].getBuf(COMP_Y).copyFrom(cPicYuvTrueOrg->getBuf(COMP_Y));

      if (isChromaEnabled(chromaFormatIdc))
      {
        picCurr->m_bufs[PIC_ORIGINAL_INPUT].getBuf(COMP_Cb).copyFrom(picYuvOrg->getBuf(COMP_Cb));
        picCurr->m_bufs[PIC_ORIGINAL_INPUT].getBuf(COMP_Cr).copyFrom(picYuvOrg->getBuf(COMP_Cr));

        picCurr->m_bufs[PIC_TRUE_ORIGINAL_INPUT].getBuf(COMP_Cb).copyFrom(cPicYuvTrueOrg->getBuf(COMP_Cb));
        picCurr->m_bufs[PIC_TRUE_ORIGINAL_INPUT].getBuf(COMP_Cr).copyFrom(cPicYuvTrueOrg->getBuf(COMP_Cr));
      }

      if (m_encCfg.m_gopBasedTemporalFilterEnabled)
      {
        picCurr->m_bufs[PIC_FILTERED_ORIGINAL_INPUT].getBuf(COMP_Y).copyFrom(picYuvFilteredOrg->getBuf(COMP_Y));

        if (isChromaEnabled(chromaFormatIdc))
        {
          picCurr->m_bufs[PIC_FILTERED_ORIGINAL_INPUT].getBuf(COMP_Cb).copyFrom(picYuvFilteredOrg->getBuf(COMP_Cb));
          picCurr->m_bufs[PIC_FILTERED_ORIGINAL_INPUT].getBuf(COMP_Cr).copyFrom(picYuvFilteredOrg->getBuf(COMP_Cr));
        }
      }

      const PPS    *refPPS           = m_ppsMap.getPS(0);
      const Window &curScalingWindow = pPPS->m_scalingWindow;

      const int curPicWidth = pPPS->m_picWidthInLumaSamples -
        SPS::getWinUnitX(pSPS->m_chromaFormatIdc) *
          (curScalingWindow.m_winLeftOffset + curScalingWindow.m_winRightOffset);
      const int curPicHeight = pPPS->m_picHeightInLumaSamples -
        SPS::getWinUnitY(pSPS->m_chromaFormatIdc) *
          (curScalingWindow.m_winTopOffset + curScalingWindow.m_winBottomOffset);

      const Window &refScalingWindow = refPPS->m_scalingWindow;

      const int refPicWidth = refPPS->m_picWidthInLumaSamples -
        SPS::getWinUnitX(pSPS->m_chromaFormatIdc) *
          (refScalingWindow.m_winLeftOffset + refScalingWindow.m_winRightOffset);
      const int refPicHeight = refPPS->m_picHeightInLumaSamples -
        SPS::getWinUnitY(pSPS->m_chromaFormatIdc) *
          (refScalingWindow.m_winTopOffset + refScalingWindow.m_winBottomOffset);

      const int xScale = ((refPicWidth << ScalingRatio::BITS) + (curPicWidth >> 1)) / curPicWidth;
      const int yScale = ((refPicHeight << ScalingRatio::BITS) + (curPicHeight >> 1)) / curPicHeight;

      const ScalingRatio scalingRatio = { xScale, yScale };

      Picture::rescalePicture(scalingRatio, *picYuvOrg, refPPS->m_scalingWindow, picCurr->getOrigBuf(),
                              pPPS->m_scalingWindow, chromaFormatIdc, pSPS->m_bitDepths, true, true,
                              pSPS->m_horCollocatedChromaFlag, pSPS->m_verCollocatedChromaFlag);
      Picture::rescalePicture(scalingRatio, *cPicYuvTrueOrg, refPPS->m_scalingWindow, picCurr->getTrueOrigBuf(),
                              pPPS->m_scalingWindow, chromaFormatIdc, pSPS->m_bitDepths, true, true,
                              pSPS->m_horCollocatedChromaFlag, pSPS->m_verCollocatedChromaFlag);
      if (m_encCfg.m_gopBasedTemporalFilterEnabled)
      {
        Picture::rescalePicture(scalingRatio, *picYuvFilteredOrg, refPPS->m_scalingWindow,
                                picCurr->getFilteredOrigBuf(), pPPS->m_scalingWindow, chromaFormatIdc,
                                pSPS->m_bitDepths, true, true, pSPS->m_horCollocatedChromaFlag,
                                pSPS->m_verCollocatedChromaFlag);
      }
    }
    else
    {
      picCurr->m_bufs[PIC_ORIGINAL].swap(*picYuvOrg);
      picCurr->m_bufs[PIC_TRUE_ORIGINAL].swap(*cPicYuvTrueOrg);
      if (m_encCfg.m_gopBasedTemporalFilterEnabled)
      {
        picCurr->m_bufs[PIC_FILTERED_ORIGINAL].swap(*picYuvFilteredOrg);
      }
      if (m_encCfg.m_seiCfg.m_fgcSEIAnalysisEnabled && m_encCfg.m_seiCfg.m_fgcSEIExternalDenoised.empty())
      {
        picCurr->m_bufs[PIC_FILTERED_ORIGINAL_FG].swap(*picYuvFilteredOrgForFG);
      }
    }
    picCurr->finalInit(m_vps, *pSPS, *pPPS, &m_picHeader, m_apss, m_lmcsAPS, m_scalinglistAPS);

    picCurr->m_poc = m_pocLast;

#if JVET_Z0120_SII_SEI_PROCESSING
    if (m_encCfg.m_seiCfg.m_ShutterFilterEnable)
    {
      int blendingRatio = m_encCfg.m_seiCfg.m_SII_BlendingRatio;
      picCurr->xOutputPreFilteredPic(picCurr, &m_cListPic, blendingRatio, m_encCfg.m_intraPeriod);
      picCurr->copyToPic(pSPS, &picCurr->m_bufs[PIC_ORIGINAL], picYuvOrg);
    }
#endif

    // compute image characteristics
    if (m_encCfg.m_bUseAdaptiveQP)
    {
      AQpPreanalyzer::preanalyze(picCurr);
    }
  }

  if ((m_receivedPicCount == 0) ||
      (!flush && (m_pocLast != 0) && (m_receivedPicCount != m_encCfg.m_gopSize) && (m_encCfg.m_gopSize != 0)))
  {
    numEncoded = 0;
    return true;
  }

  if (m_encCfg.m_RCEnableRateControl)
  {
    m_cRateCtrl.initRCGOP(m_receivedPicCount);
  }

  m_picIdInGOP = 0;

  return false;
}

/**
 - Application has picture buffer list with size of GOP + 1
 - Picture buffer list acts like as ring buffer
 - End of the list has the latest picture
 .
 \param   flush               cause encoder to encode a partial GOP
 \param   picYuvOrg         original YUV picture
 \param   picYuvTrueOrg
 \param   snrCSC
 \retval  rcListPicYuvRecOut  list of reconstruction YUV pictures
 \retval  accessUnitsOut      list of output access units
 \retval  numEncoded         number of encoded pictures
 */

bool EncLib::encode(const InputColourSpaceConversion snrCSC, std::list<PelUnitBuf *> &rcListPicYuvRecOut,
                    int &numEncoded)
{
  // compress GOP
  m_cGOPEncoder.compressGOP(m_pocLast, m_receivedPicCount, m_cListPic, rcListPicYuvRecOut, false, false, snrCSC,
                            m_encCfg.m_printFrameMSE, m_encCfg.m_printMSSSIM, false, m_picIdInGOP);

  m_picIdInGOP++;

  // go over all pictures in a GOP excluding the first IRAP
  if (m_picIdInGOP != m_encCfg.m_gopSize && m_pocLast != 0)
  {
    return true;
  }

#if JVET_O0756_CALCULATE_HDRMETRICS
  m_metricTime = m_cGOPEncoder.getMetricTime();
#endif

  if (m_encCfg.m_RCEnableRateControl)
  {
    m_cRateCtrl.destroyRCGOP();
  }

  numEncoded         = m_receivedPicCount;
  m_receivedPicCount = 0;
  m_codedPicCount += numEncoded;

  return false;
}

/**------------------------------------------------
 Separate interlaced frame into two fields
 -------------------------------------------------**/
void separateFields(Pel *org, Pel *dstField, ptrdiff_t stride, uint32_t width, uint32_t height, bool isTop)
{
  if (!isTop)
  {
    org += stride;
  }
  for (int y = 0; y < height >> 1; y++)
  {
    for (int x = 0; x < width; x++)
    {
      dstField[x] = org[x];
    }

    dstField += stride;
    org += stride * 2;
  }
}

bool EncLib::encodePrep(bool flush, PelStorage *picYuvOrg, PelStorage *picYuvTrueOrg, PelStorage *picYuvFilteredOrg,
                        const InputColourSpaceConversion snrCSC, std::list<PelUnitBuf *> &rcListPicYuvRecOut,
                        int &numEncoded, bool isTff)
{
  numEncoded     = 0;
  bool keepDoing = true;

  for (int fieldNum = 0; fieldNum < 2; fieldNum++)
  {
    if (picYuvOrg)
    {
      /* -- field initialization -- */
      const bool isTopField = isTff == (fieldNum == 0);

      Picture *pcField;
      xGetNewPicBuffer(rcListPicYuvRecOut, pcField, -1);

      for (uint32_t comp = 0; comp < ::getNumberValidComponents(picYuvOrg->chromaFormat); comp++)
      {
        const CompID compID = CompID(comp);
        {
          PelBuf compBuf = picYuvOrg->get(compID);
          separateFields(compBuf.buf, pcField->getOrigBuf().get(compID).buf, compBuf.stride, compBuf.width,
                         compBuf.height, isTopField);
          // to get fields of true original buffer to avoid wrong PSNR calculation in summary
          compBuf = picYuvTrueOrg->get(compID);
          separateFields(compBuf.buf, pcField->getTrueOrigBuf().get(compID).buf, compBuf.stride, compBuf.width,
                         compBuf.height, isTopField);
          if (m_encCfg.m_gopBasedTemporalFilterEnabled)
          {
            compBuf = picYuvFilteredOrg->get(compID);
            separateFields(compBuf.buf, pcField->getTrueOrigBuf().get(compID).buf, compBuf.stride, compBuf.width,
                           compBuf.height, isTopField);
          }
        }
      }

      int        ppsID = -1;   // Use default PPS ID
      const PPS *pPPS  = (ppsID < 0) ? m_ppsMap.getFirstPS() : m_ppsMap.getPS(ppsID);
      const SPS *pSPS  = m_spsMap.getPS(pPPS->m_spsId);

      pcField->finalInit(m_vps, *pSPS, *pPPS, &m_picHeader, m_apss, m_lmcsAPS, m_scalinglistAPS);
      pcField->m_poc           = m_pocLast;
      pcField->m_reconstructed = false;

      pcField->m_extendedBorder = false;   // where is this normally?

      pcField->m_topField = isTopField;   // interlaced requirement

#if JVET_Z0120_SII_SEI_PROCESSING
      if (m_encCfg.m_seiCfg.m_ShutterFilterEnable)
      {
        int blendingRatio = m_encCfg.m_seiCfg.m_SII_BlendingRatio;
        pcField->xOutputPreFilteredPic(pcField, &m_cListPic, blendingRatio, m_encCfg.m_intraPeriod);
        pcField->copyToPic(pSPS, &pcField->m_bufs[PIC_ORIGINAL], picYuvOrg);
      }
#endif

      // compute image characteristics
      if (m_encCfg.m_bUseAdaptiveQP)
      {
        AQpPreanalyzer::preanalyze(pcField);
      }
    }
  }

  if (m_receivedPicCount && (flush || m_pocLast == 1 || m_receivedPicCount == m_encCfg.m_gopSize))
  {
    m_picIdInGOP = 0;
    keepDoing    = false;
  }

  return keepDoing;
}

bool EncLib::encode(const InputColourSpaceConversion snrCSC, std::list<PelUnitBuf *> &rcListPicYuvRecOut,
                    int &numEncoded, bool isTff)
{
  PROFILER_SCOPE(0, g_timeProfiler, P_TOP_LEVEL);
  numEncoded = 0;

  for (int fieldNum = 0; fieldNum < 2; fieldNum++)
  {
    m_pocLast = m_pocLast < 2 ? fieldNum : m_pocLast;

    // compress GOP
    m_cGOPEncoder.compressGOP(m_pocLast, m_pocLast < 2 ? m_pocLast + 1 : m_receivedPicCount, m_cListPic,
                              rcListPicYuvRecOut, true, isTff, snrCSC, m_encCfg.m_printFrameMSE, m_encCfg.m_printMSSSIM,
                              false, m_picIdInGOP);
#if JVET_O0756_CALCULATE_HDRMETRICS
    m_metricTime = m_cGOPEncoder.getMetricTime();
#endif

    m_picIdInGOP++;
  }

  // go over all pictures in a GOP excluding first top field and first bottom field
  if (m_picIdInGOP != m_encCfg.m_gopSize && m_pocLast > 1)
  {
    return true;
  }

  numEncoded += m_receivedPicCount;
  m_codedPicCount += m_receivedPicCount;
  m_receivedPicCount = 0;

  return false;
}

void EncLib::applyNnPostFilter()
{
  if (m_cListPic.empty())
  {
    return;
  }
  m_nnPostFiltering.filterPictures(m_cListPic);
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/**
 - Application has picture buffer list with size of GOP + 1
 - Picture buffer list acts like as ring buffer
 - End of the list has the latest picture
 .
 \retval pic obtained picture buffer
 */
void EncLib::xGetNewPicBuffer(std::list<PelUnitBuf *> &rcListPicYuvRecOut, Picture *&rpic, int ppsId)
{
  // rotate the output buffer
  rcListPicYuvRecOut.push_back(rcListPicYuvRecOut.front());
  rcListPicYuvRecOut.pop_front();

  rpic = nullptr;

  // At this point, the SPS and PPS can be considered activated - they are copied to the new Pic.
  const PPS *pPPS = (ppsId < 0) ? m_ppsMap.getFirstPS() : m_ppsMap.getPS(ppsId);
  CHECK(pPPS == nullptr, "PPS not found");
  const PPS &pps = *pPPS;

  const SPS *pSPS = m_spsMap.getPS(pps.m_spsId);
  CHECK(pSPS == nullptr, "SPS not found");
  const SPS &sps = *pSPS;

  Slice::sortPicList(m_cListPic);

  // use an entry in the buffered list if the maximum number that need buffering has been reached:
  int maxDecPicBuffering = (m_vps == nullptr || m_vps->m_numLayersInOls[m_vps->m_targetOlsIdx] == 1)
    ? sps.m_maxDecPicBuffering[MAX_TLAYER - 1]
    : m_vps->getMaxDecPicBuffering(MAX_TLAYER - 1);

  if (m_cListPic.size() >= (uint32_t)(m_encCfg.m_gopSize + maxDecPicBuffering + 2))
  {
    PicList::iterator iterPic = m_cListPic.begin();
    int               size    = int(m_cListPic.size());
    for (int i = 0; i < size; i++)
    {
      rpic = *iterPic;
      if (!rpic->m_referenced && rpic->m_layerId == m_layerId)
      {
        break;
      }
      else
      {
        rpic = nullptr;
      }
      iterPic++;
    }

    // If PPS ID is the same, we will assume that it has not changed since it was last used
    // and return the old object.
    if (rpic && pps.m_ppsId != rpic->m_cs->pps->m_ppsId)
    {
      // the IDs differ - free up an entry in the list, and then create a new one, as with the case where the max
      // buffering state has not been reached.
      rpic->destroy();
      delete rpic;
      m_cListPic.erase(iterPic);
      rpic = nullptr;
    }
  }

  if (rpic == nullptr)
  {
    rpic = new Picture;
    bool fgAnalysisEnabled =
      m_encCfg.m_seiCfg.m_fgcSEIAnalysisEnabled && m_encCfg.m_seiCfg.m_fgcSEIExternalDenoised.empty();
    rpic->create(sps.m_chromaFormatIdc, Size(pps.m_picWidthInLumaSamples, pps.m_picHeightInLumaSamples),
                 sps.m_maxCuWidth, sps.m_maxCuWidth + EXT_PICTURE_SIZE, false, m_layerId, sps.m_rprEnabledFlag,
                 m_encCfg.m_gopBasedTemporalFilterEnabled, fgAnalysisEnabled
#if JVET_Z0120_SII_SEI_PROCESSING
                 ,
                 m_encCfg.m_seiCfg.m_ShutterFilterEnable
#endif
#if ENABLE_NNLF
                 ,
                 sps.m_nnlfStore
#endif
    );

    if (m_encCfg.m_resChangeInClvsEnabled)
    {
      const PPS &pps0 = *m_ppsMap.getPS(0);
      rpic->m_bufs[PIC_ORIGINAL_INPUT].create(
        sps.m_chromaFormatIdc, Area(Position(), Size(pps0.m_picWidthInLumaSamples, pps0.m_picHeightInLumaSamples)));
      rpic->m_bufs[PIC_TRUE_ORIGINAL_INPUT].create(
        sps.m_chromaFormatIdc, Area(Position(), Size(pps0.m_picWidthInLumaSamples, pps0.m_picHeightInLumaSamples)));
      if (m_encCfg.m_gopBasedTemporalFilterEnabled)
      {
        rpic->m_bufs[PIC_FILTERED_ORIGINAL_INPUT].create(
          sps.m_chromaFormatIdc, Area(Position(), Size(pps0.m_picWidthInLumaSamples, pps0.m_picHeightInLumaSamples)));
      }
    }
    if (m_encCfg.m_bUseAdaptiveQP)
    {
      const uint32_t maxDqpLayer = m_picHeader.m_cuQpDeltaSubdivIntra / 2 + 1;
      rpic->m_aqlayer.resize(maxDqpLayer);
      for (uint32_t d = 0; d < maxDqpLayer; d++)
      {
        rpic->m_aqlayer[d] = new AQpLayer(pps.m_picWidthInLumaSamples, pps.m_picHeightInLumaSamples,
                                          sps.m_maxCuWidth >> d, sps.m_maxCuHeight >> d);
      }
    }

    m_cListPic.push_back(rpic);
  }

  rpic->m_extendedBorder = false;
  rpic->m_reconstructed  = false;
  rpic->m_referenced     = true;
  rpic->m_hashMap.clearAll();

  m_pocLast += (m_encCfg.m_compositeRefEnabled ? 2 : 1);
  m_receivedPicCount++;
}

void EncLib::xInitVPS(const SPS &sps)
{
  // The SPS must have already been set up.
  // set the VPS profile information.

  m_vps->m_olsHrdParams.clear();
  m_vps->m_olsHrdParams.resize(m_vps->m_numOlsTimingHrdParamsMinus1,
                               std::vector<OlsHrdParams>(m_vps->m_vpsMaxSubLayers));
  ProfileTierLevelFeatures profileTierLevelFeatures;
  profileTierLevelFeatures.extractPTLInformation(sps);
  m_vps->setMaxTidIlRefPicsPlus1(m_encCfg.m_maxTidILRefPicsPlus1);
  m_vps->deriveOutputLayerSets();
  m_vps->deriveTargetOutputLayerSet(m_vps->m_targetOlsIdx);

  // number of the DPB parameters is set equal to the number of OLS containing multi layers
  if (!m_vps->m_vpsEachLayerIsAnOlsFlag)
  {
    m_vps->m_numDpbParams = m_vps->m_numMultiLayeredOlss;
  }

  if (m_vps->m_dpbParameters.size() != m_vps->m_numDpbParams)
  {
    m_vps->m_dpbParameters.resize(m_vps->m_numDpbParams);
  }

  if (m_vps->m_dpbMaxTemporalId.size() != m_vps->m_numDpbParams)
  {
    m_vps->m_dpbMaxTemporalId.resize(m_vps->m_numDpbParams);
  }

  for (int olsIdx = 0, dpbIdx = 0; olsIdx < m_vps->m_numOutputLayersInOls.size(); olsIdx++)
  {
    if (m_vps->m_numLayersInOls[olsIdx] > 1)
    {
      if (std::find(m_vps->m_layerIdInOls[olsIdx].begin(), m_vps->m_layerIdInOls[olsIdx].end(), m_layerId) !=
          m_vps->m_layerIdInOls[olsIdx].end())
      {
        m_vps->m_olsDpbPicSize[olsIdx].width =
          std::max<int>(sps.m_maxWidthInLumaSamples, m_vps->m_olsDpbPicSize[olsIdx].width);
        m_vps->m_olsDpbPicSize[olsIdx].height =
          std::max<int>(sps.m_maxHeightInLumaSamples, m_vps->m_olsDpbPicSize[olsIdx].height);
        m_vps->m_olsDpbChromaFormatIdc[olsIdx] =
          std::max(sps.m_chromaFormatIdc, m_vps->m_olsDpbChromaFormatIdc[olsIdx]);
        m_vps->m_olsDpbBitDepthMinus8[olsIdx] =
          std::max<int>(sps.m_bitDepths[ChannelType::LUMA] - 8, m_vps->m_olsDpbBitDepthMinus8[olsIdx]);
      }

      m_vps->m_olsDpbParamsIdx[olsIdx] = dpbIdx;
      dpbIdx++;
    }
  }

  for (int i = 0; i < m_vps->m_numOutputLayersInOls.size(); i++)
  {
    if (m_vps->m_numLayersInOls[i] > 1)
    {
      const int dpbIdx = m_vps->m_olsDpbParamsIdx[i];

      if (m_vps->m_vpsMaxSubLayers == 1)
      {
        // When vps_max_sublayers_minus1 is equal to 0, the value of vps_dpb_max_tid[ dpbIdx ] is inferred to be equal
        // to 0.
        m_vps->m_dpbMaxTemporalId[dpbIdx] = 0;
      }
      else
      {
        if (m_vps->m_vpsDefaultPtlDpbHrdMaxTidFlag)
        {
          // When vps_max_sublayers_minus1 is greater than 0 and vps_all_layers_same_num_sublayers_flag is equal to 1,
          // the value of vps_dpb_max_tid[ dpbIdx ] is inferred to be equal to vps_max_sublayers_minus1.
          m_vps->m_dpbMaxTemporalId[dpbIdx] = m_vps->m_vpsMaxSubLayers - 1;
        }
        else
        {
          m_vps->m_dpbMaxTemporalId[dpbIdx] = m_vps->m_vpsMaxSubLayers - 1;
        }
      }

      int decPicBuffering[MAX_TLAYER] = { 0 };

      for (int lIdx = 0; lIdx < m_vps->m_numLayersInOls[i]; lIdx++)
      {
        for (int tId = 0; tId < MAX_TLAYER; tId++)
        {
          decPicBuffering[tId] += m_layerDecPicBuffering[m_vps->m_layerIdInOls[i][lIdx] * MAX_TLAYER + tId];
        }
      }

      for (int j = (m_vps->m_sublayerDpbParamsPresentFlag ? 0 : m_vps->m_dpbMaxTemporalId[dpbIdx]);
           j <= m_vps->m_dpbMaxTemporalId[dpbIdx]; j++)
      {
        m_vps->m_dpbParameters[dpbIdx].maxDecPicBuffering[j] = decPicBuffering[j] > 0
          ? decPicBuffering[j]
          : profileTierLevelFeatures.getMaxDpbSize(m_vps->m_olsDpbPicSize[i].width * m_vps->m_olsDpbPicSize[i].height);
        m_vps->m_dpbParameters[dpbIdx].maxNumReorderPics[j]  = m_vps->m_dpbParameters[dpbIdx].maxDecPicBuffering[j] - 1;
        m_vps->m_dpbParameters[dpbIdx].maxLatencyIncreasePlus1[j] = 0;

        CHECK(
          m_vps->m_dpbParameters[dpbIdx].maxDecPicBuffering[j] >
            profileTierLevelFeatures.getMaxDpbSize(m_vps->m_olsDpbPicSize[i].width * m_vps->m_olsDpbPicSize[i].height),
          "DPB size is not sufficient");
      }

      for (int j = (m_vps->m_sublayerDpbParamsPresentFlag ? m_vps->m_dpbMaxTemporalId[dpbIdx] : 0);
           j < m_vps->m_dpbMaxTemporalId[dpbIdx]; j++)
      {
        // When dpb_max_dec_pic_buffering_minus1[ dpbIdx ] is not present for dpbIdx in the range of 0 to
        // maxSubLayersMinus1 - 1, inclusive, due to subLayerInfoFlag being equal to 0, it is inferred to be equal to
        // dpb_max_dec_pic_buffering_minus1[ maxSubLayersMinus1 ].
        m_vps->m_dpbParameters[dpbIdx].maxDecPicBuffering[j] =
          m_vps->m_dpbParameters[dpbIdx].maxDecPicBuffering[m_vps->m_dpbMaxTemporalId[dpbIdx]];

        // When dpb_max_num_reorder_pics[ dpbIdx ] is not present for dpbIdx in the range of 0 to maxSubLayersMinus1 -
        // 1, inclusive, due to subLayerInfoFlag being equal to 0, it is inferred to be equal to
        // dpb_max_num_reorder_pics[ maxSubLayersMinus1 ].
        m_vps->m_dpbParameters[dpbIdx].maxNumReorderPics[j] =
          m_vps->m_dpbParameters[dpbIdx].maxNumReorderPics[m_vps->m_dpbMaxTemporalId[dpbIdx]];

        // When dpb_max_latency_increase_plus1[ dpbIdx ] is not present for dpbIdx in the range of 0 to
        // maxSubLayersMinus1 - 1, inclusive, due to subLayerInfoFlag being equal to 0, it is inferred to be equal to
        // dpb_max_latency_increase_plus1[ maxSubLayersMinus1 ].
        m_vps->m_dpbParameters[dpbIdx].maxLatencyIncreasePlus1[j] =
          m_vps->m_dpbParameters[dpbIdx].maxLatencyIncreasePlus1[m_vps->m_dpbMaxTemporalId[dpbIdx]];
      }
    }
  }
  for (int i = 0; i < m_vps->m_vpsNumOutputLayerSets; i++)
  {
    m_vps->m_hrdMaxTid[i] = m_vps->m_vpsMaxSubLayers - 1;
  }

  m_vps->checkVPS();
}

void EncLib::xInitOPI(OPI &opi)
{
  if (m_encCfg.m_OPIEnabled && m_vps)
  {
    if (!opi.m_olsinfopresentflag)
    {
      opi.m_opiolsidx          = m_vps->deriveTargetOLSIdx();
      opi.m_olsinfopresentflag = true;
    }
    if (!opi.m_htidinfopresentflag)
    {
      opi.m_opihtidplus1        = m_vps->getMaxTidinTOls(opi.m_opiolsidx) + 1;
      opi.m_htidinfopresentflag = true;
    }
  }
}

void EncLib::xInitDCI(DCI &dci, const SPS &sps)
{
  dci.m_maxSubLayers = sps.m_maxSubLayers;
  std::vector<ProfileTierLevel> ptls;
  ptls.push_back(sps.m_profileTierLevel);
  dci.m_profileTierLevel = ptls;
}

void EncLib::xInitSPS(SPS &sps)
{
  ProfileTierLevel *profileTierLevel = &sps.m_profileTierLevel;
  ConstraintInfo   *cinfo            = &profileTierLevel->m_constraintInfo;

  cinfo->m_gciPresentFlag                               = m_encCfg.m_gciPresentFlag;
  cinfo->m_noRprConstraintFlag                          = m_encCfg.m_noRprConstraintFlag;
  cinfo->m_noResChangeInClvsConstraintFlag              = m_encCfg.m_noResChangeInClvsConstraintFlag;
  cinfo->m_oneTilePerPicConstraintFlag                  = m_encCfg.m_oneTilePerPicConstraintFlag;
  cinfo->m_picHeaderInSliceHeaderConstraintFlag         = m_encCfg.m_picHeaderInSliceHeaderConstraintFlag;
  cinfo->m_oneSlicePerPicConstraintFlag                 = m_encCfg.m_oneSlicePerPicConstraintFlag;
  cinfo->m_noIdrRplConstraintFlag                       = m_encCfg.m_noIdrRplConstraintFlag;
  cinfo->m_noRectSliceConstraintFlag                    = m_encCfg.m_noRectSliceConstraintFlag;
  cinfo->m_oneSlicePerSubpicConstraintFlag              = m_encCfg.m_oneSlicePerSubpicConstraintFlag;
  cinfo->m_noSubpicInfoConstraintFlag                   = m_encCfg.m_noSubpicInfoConstraintFlag;
  cinfo->m_onePictureOnlyConstraintFlag                 = m_encCfg.m_onePictureOnlyConstraintFlag;
  cinfo->m_intraOnlyConstraintFlag                      = m_encCfg.m_intraOnlyConstraintFlag;
  cinfo->m_maxBitDepthConstraintIdc                     = m_encCfg.m_maxBitDepthConstraintIdc;
  cinfo->m_maxChromaFormatConstraintIdc                 = m_encCfg.m_maxChromaFormatConstraintIdc;
  cinfo->m_allLayersIndependentConstraintFlag           = m_encCfg.m_allLayersIndependentConstraintFlag;
  cinfo->m_noMrlConstraintFlag                          = m_encCfg.m_noMrlConstraintFlag;
  cinfo->m_noMipConstraintFlag                          = m_encCfg.m_noMipConstraintFlag;
  cinfo->m_noLfnstConstraintFlag                        = m_encCfg.m_noLfnstConstraintFlag;
  cinfo->m_noMmvdConstraintFlag                         = m_encCfg.m_noMmvdConstraintFlag;
  cinfo->m_noSmvdConstraintFlag                         = m_encCfg.m_noSmvdConstraintFlag;
  cinfo->m_noProfConstraintFlag                         = m_encCfg.m_noProfConstraintFlag;
  cinfo->m_noPaletteConstraintFlag                      = m_encCfg.m_noPaletteConstraintFlag;
  cinfo->m_noActConstraintFlag                          = m_encCfg.m_noActConstraintFlag;
  cinfo->m_noLmcsConstraintFlag                         = m_encCfg.m_noLmcsConstraintFlag;
  cinfo->m_noExplicitScaleListConstraintFlag            = m_encCfg.m_noExplicitScaleListConstraintFlag;
  cinfo->m_noMttConstraintFlag                          = m_encCfg.m_noMttConstraintFlag;
  cinfo->m_noChromaQpOffsetConstraintFlag               = m_encCfg.m_noChromaQpOffsetConstraintFlag;
  cinfo->m_noQtbttDualTreeIntraConstraintFlag           = m_encCfg.m_noQtbttDualTreeIntraConstraintFlag;
  cinfo->m_noPartitionConstraintsOverrideConstraintFlag = m_encCfg.m_noPartitionConstraintsOverrideConstraintFlag;
  cinfo->m_noSaoConstraintFlag                          = m_encCfg.m_noSaoConstraintFlag;
  cinfo->m_noCCSaoConstraintFlag                        = m_encCfg.m_noCCSaoConstraintFlag;
  cinfo->m_noAlfConstraintFlag                          = m_encCfg.m_noAlfConstraintFlag;
  cinfo->m_noCCAlfConstraintFlag                        = m_encCfg.m_noCCAlfConstraintFlag;
  cinfo->m_noWeightedPredictionConstraintFlag           = m_encCfg.m_noWeightedPredictionConstraintFlag;
  cinfo->m_noRefWraparoundConstraintFlag                = m_encCfg.m_noRefWraparoundConstraintFlag;
  cinfo->m_noTemporalMvpConstraintFlag                  = m_encCfg.m_noTemporalMvpConstraintFlag;
  cinfo->m_noSbtmvpConstraintFlag                       = m_encCfg.m_noSbtmvpConstraintFlag;
  cinfo->m_noAmvrConstraintFlag                         = m_encCfg.m_noAmvrConstraintFlag;
  cinfo->m_noBdofConstraintFlag                         = m_encCfg.m_noBdofConstraintFlag;
  cinfo->m_noCclmConstraintFlag                         = m_encCfg.m_noCclmConstraintFlag;
  cinfo->m_noMtsConstraintFlag                          = m_encCfg.m_noMtsConstraintFlag;
  cinfo->m_noSbtConstraintFlag                          = m_encCfg.m_noSbtConstraintFlag;
  cinfo->m_noAffineMotionConstraintFlag                 = m_encCfg.m_noAffineMotionConstraintFlag;
  cinfo->m_noBcwConstraintFlag                          = m_encCfg.m_noBcwConstraintFlag;
  cinfo->m_noIbcConstraintFlag                          = m_encCfg.m_noIbcConstraintFlag;
  cinfo->m_noCiipConstraintFlag                         = m_encCfg.m_noCiipConstraintFlag;
  cinfo->m_noGeoConstraintFlag                          = m_encCfg.m_noGeoConstraintFlag;
  cinfo->m_noSgpmConstraintFlag                         = m_encCfg.m_noSgpmConstraintFlag;
  cinfo->m_noObmcConstraintFlag                         = m_encCfg.m_noObmcConstraintFlag;
  cinfo->m_noLadfConstraintFlag                         = m_encCfg.m_noLadfConstraintFlag;
  cinfo->m_noTransformSkipConstraintFlag                = m_encCfg.m_noTransformSkipConstraintFlag;
  cinfo->m_noBDPCMConstraintFlag                        = m_encCfg.m_noBDPCMConstraintFlag;
  cinfo->m_noJointCbCrConstraintFlag                    = m_encCfg.m_noJointCbCrConstraintFlag;
  cinfo->m_noCuQpDeltaConstraintFlag                    = m_encCfg.m_noCuQpDeltaConstraintFlag;
  cinfo->m_noDepQuantConstraintFlag                     = m_encCfg.m_noDepQuantConstraintFlag;
  cinfo->m_noSignDataHidingConstraintFlag               = m_encCfg.m_noSignDataHidingConstraintFlag;
  cinfo->m_noTrailConstraintFlag                        = m_encCfg.m_noTrailConstraintFlag;
  cinfo->m_noStsaConstraintFlag                         = m_encCfg.m_noStsaConstraintFlag;
  cinfo->m_noRaslConstraintFlag                         = m_encCfg.m_noRaslConstraintFlag;
  cinfo->m_noRadlConstraintFlag                         = m_encCfg.m_noRadlConstraintFlag;
  cinfo->m_noIdrConstraintFlag                          = m_encCfg.m_noIdrConstraintFlag;
  cinfo->m_noCraConstraintFlag                          = m_encCfg.m_noCraConstraintFlag;
  cinfo->m_noGdrConstraintFlag                          = m_encCfg.m_noGdrConstraintFlag;
  cinfo->m_noApsConstraintFlag                          = m_encCfg.m_noApsConstraintFlag;
  cinfo->m_allRapPicturesFlag                           = m_encCfg.m_allRapPicturesFlag;
  cinfo->m_noExtendedPrecisionProcessingConstraintFlag  = m_encCfg.m_noExtendedPrecisionProcessingConstraintFlag;
  cinfo->m_noTsResidualCodingRiceConstraintFlag         = m_encCfg.m_noTsResidualCodingRiceConstraintFlag;
  cinfo->m_noRrcRiceExtensionConstraintFlag             = m_encCfg.m_noRrcRiceExtensionConstraintFlag;
  cinfo->m_noPersistentRiceAdaptationConstraintFlag     = m_encCfg.m_noPersistentRiceAdaptationConstraintFlag;
  cinfo->m_noReverseLastSigCoeffConstraintFlag          = m_encCfg.m_noReverseLastSigCoeffConstraintFlag;

  profileTierLevel->m_levelIdc                = m_encCfg.m_level;
  profileTierLevel->m_tierFlag                = m_encCfg.m_tier;
  profileTierLevel->m_profileIdc              = m_encCfg.m_profile;
  profileTierLevel->m_frameOnlyConstraintFlag = m_encCfg.m_frameOnlyConstraintFlag;
  profileTierLevel->m_multiLayerEnabledFlag   = m_encCfg.m_multiLayerEnabledFlag;
  profileTierLevel->m_subProfileIdc.resize(m_encCfg.m_numSubProfile);
  for (int k = 0; k < m_encCfg.m_numSubProfile; k++)
  {
    profileTierLevel->m_subProfileIdc[k] = m_encCfg.m_subProfile[k];
  }
  /* XXX: should Main be marked as compatible with still picture? */
  /* XXX: may be a good idea to refactor the above into a function
   * that chooses the actual compatibility based upon options */
  sps.m_vpsId = m_vps->m_vpsId;

  sps.m_GDREnabledFlag = false;

  sps.m_maxWidthInLumaSamples  = m_encCfg.m_sourceWidth;
  sps.m_maxHeightInLumaSamples = m_encCfg.m_sourceHeight;
  if (m_encCfg.m_resChangeInClvsEnabled)
  {
    int maxPicWidth =
      std::max(m_encCfg.m_sourceWidth, (int)((double)m_encCfg.m_sourceWidth / m_encCfg.m_scalingRatioHor + 0.5));
    int maxPicHeight =
      std::max(m_encCfg.m_sourceHeight, (int)((double)m_encCfg.m_sourceHeight / m_encCfg.m_scalingRatioVer + 0.5));
    if (m_encCfg.m_gopBasedRPREnabledFlag || m_encCfg.m_rprFunctionalityTestingEnabledFlag)
    {
      maxPicWidth  = std::max(maxPicWidth, (int)((double)m_encCfg.m_sourceWidth / m_encCfg.m_scalingRatioHor2 + 0.5));
      maxPicHeight = std::max(maxPicHeight, (int)((double)m_encCfg.m_sourceHeight / m_encCfg.m_scalingRatioVer2 + 0.5));
      maxPicWidth  = std::max(maxPicWidth, (int)((double)m_encCfg.m_sourceWidth / m_encCfg.m_scalingRatioHor3 + 0.5));
      maxPicHeight = std::max(maxPicHeight, (int)((double)m_encCfg.m_sourceHeight / m_encCfg.m_scalingRatioVer3 + 0.5));
    }
    const int minCuSize = std::max(8, 1 << m_encCfg.m_log2MinCUSize);
    if (maxPicWidth % minCuSize)
    {
      maxPicWidth += ((maxPicWidth / minCuSize) + 1) * minCuSize - maxPicWidth;
    }
    if (maxPicHeight % minCuSize)
    {
      maxPicHeight += ((maxPicHeight / minCuSize) + 1) * minCuSize - maxPicHeight;
    }
    sps.m_maxWidthInLumaSamples  = maxPicWidth;
    sps.m_maxHeightInLumaSamples = maxPicHeight;
  }
  sps.m_conformanceWindow = m_encCfg.m_conformanceWindow;

  sps.m_maxCuWidth             = m_encCfg.m_CTUSize;
  sps.m_maxCuHeight            = m_encCfg.m_CTUSize;
  sps.m_log2MinCodingBlockSize = m_encCfg.m_log2MinCUSize;
  sps.m_chromaFormatIdc        = m_encCfg.m_chromaFormatIdc;

  sps.m_ctuSize                  = m_encCfg.m_CTUSize;
  sps.m_partitionOverrideEnabled = m_encCfg.m_useSplitConsOverride;
  sps.setMinQTSizes(m_encCfg.m_minQt);
  sps.setMaxMTTHierarchyDepth(m_encCfg.m_uiMaxMTTHierarchyDepth %
                                10,   // write the value for the highest TL, as it is the one most commonly signalled
                              m_encCfg.m_uiMaxMTTHierarchyDepthI, m_encCfg.m_uiMaxMTTHierarchyDepthIChroma);
  sps.setMaxBTSize(m_encCfg.m_maxBt[1], m_encCfg.m_maxBt[0], m_encCfg.m_maxBt[2]);
  sps.setMaxTTSize(m_encCfg.m_maxTt[1], m_encCfg.m_maxTt[0], m_encCfg.m_maxTt[2]);
  sps.m_idrRefParamList                  = m_encCfg.m_idrRefParamList;
  sps.m_dualITree                        = m_encCfg.m_dualITree;
  sps.m_useIntraLFNSTinISlice            = m_encCfg.m_intraLFNSTISlice;
  sps.m_useIntraLFNSTinPBSlice           = m_encCfg.m_intraLFNSTPBSlice;
  sps.m_useInterLFNST                    = m_encCfg.m_interLFNST;
  sps.m_useInterLFNSTSBT                 = m_encCfg.m_interLFNSTSBT;
  sps.m_sbtmvpEnabledFlag                = m_encCfg.m_sbTmvpEnableFlag;
  sps.m_AMVREnabledFlag                  = m_encCfg.m_ImvMode != IMV_OFF;
  sps.m_bdofEnabledFlag                  = m_encCfg.m_BIO;
  sps.m_dmvdBDOFExt                      = m_encCfg.m_DMVDBIOExt;
  sps.m_maxNumMergeCand                  = m_encCfg.m_maxNumMergeCand;
  sps.m_maxNumAffineMergeCand            = m_encCfg.m_maxNumAffineMergeCand;
  sps.m_maxNumIBCMergeCand               = m_encCfg.m_maxNumIBCMergeCand;
  sps.m_maxNumGeoCand                    = m_encCfg.m_maxNumGeoCand;
  sps.m_useAffine                        = m_encCfg.m_Affine;
  sps.m_AffineType                       = m_encCfg.m_AffineType;
  sps.m_AffineMmvdMode                   = m_encCfg.m_AffineMmvdMode;
  sps.m_log2MinAffineBlkSizeMinus3       = floorLog2(m_encCfg.m_minAffineBlkSize) - 3;
  sps.m_usePROF                          = m_encCfg.m_PROF;
  sps.m_useDMVD                          = m_encCfg.m_useDMVD;
  sps.m_maxNumBMMergeCand                = m_encCfg.m_maxNumBMMergeCand;
  sps.m_mergeOppositeLic                 = m_encCfg.m_mergeOppositeLic;
  sps.m_maxNumOppositeLicMergeCand       = m_encCfg.m_maxNumOppositeLicMergeCand;
  sps.m_maxNumAffineOppositeLicMergeCand = m_encCfg.m_maxNumAffineOppositeLicMergeCand;
  sps.m_LMChroma                         = m_encCfg.m_LMChroma ? true : false;
  sps.m_CCCM                             = m_encCfg.m_CCCM;
  sps.m_MCBP                             = m_encCfg.m_MCBP;
  sps.m_TMBP                             = m_encCfg.m_TMBP;
  sps.m_horCollocatedChromaFlag          = m_encCfg.m_horCollocatedChromaFlag;
  sps.m_verCollocatedChromaFlag          = m_encCfg.m_verCollocatedChromaFlag;
  sps.m_mtsEnabled       = m_encCfg.m_explicitMtsIntra || m_encCfg.m_explicitMtsInter || m_encCfg.m_implicitMtsIntra;
  sps.m_explicitMtsIntra = m_encCfg.m_explicitMtsIntra;
  sps.m_explicitMtsInter = m_encCfg.m_explicitMtsInter;
  sps.m_useSBT           = m_encCfg.m_SBT;
  sps.m_useSMVD          = m_encCfg.m_SMVD;
  sps.m_useBcw           = m_encCfg.m_bcw;
  sps.m_ladfEnabled      = m_encCfg.m_ladfEnabled;
  if (m_encCfg.m_ladfEnabled)
  {
    sps.m_ladfNumIntervals = m_encCfg.m_ladfNumIntervals;
    for (int k = 0; k < m_encCfg.m_ladfNumIntervals; k++)
    {
      sps.m_ladfQpOffset[k]           = m_encCfg.m_ladfQpOffset[k];
      sps.m_ladfIntervalLowerBound[k] = m_encCfg.m_ladfIntervalLowerBound[k];
    }
    CHECK(m_encCfg.m_ladfIntervalLowerBound[0] != 0, "abnormal value set to LadfIntervalLowerBound[0]");
  }

  sps.m_useCiip                    = m_encCfg.m_ciip;
  sps.m_useGeo                     = m_encCfg.m_Geo ? true : false;
  sps.m_useSgpm                    = m_encCfg.m_sgpm;
  sps.m_useObmc                    = m_encCfg.m_obmc;
  sps.m_useMMVD                    = m_encCfg.m_MMVD;
  sps.m_affineParaRefinement       = m_encCfg.m_affineParaRefinement;
  sps.m_fpelMmvdEnabledFlag        = ((m_encCfg.m_MMVD) ? m_encCfg.m_allowDisFracMMVD : false);
  sps.m_bdofControlPresentInPhFlag = m_encCfg.m_BIO;
  sps.m_profControlPresentInPhFlag = m_encCfg.m_PROF;
  sps.m_affineAmvrEnabledFlag      = m_encCfg.m_AffineAmvr;
  sps.m_PLTMode                    = m_encCfg.m_PLTMode;
  sps.m_ibcFlag                    = m_encCfg.m_ibcMode & 0x01;
  sps.m_ibcFlagInterSlice          = m_encCfg.m_ibcMode & 0x02;
  sps.m_ibcFracFlag                = m_encCfg.m_ibcFracMode;
  sps.m_ibcMerge                   = m_encCfg.m_ibcMerge;
  sps.m_wrapAroundEnabledFlag      = m_encCfg.m_wrapAround;
  // ADD_NEW_TOOL : (encoder lib) set tool enabling flags and associated parameters here
  sps.m_lmcsEnabled                = m_encCfg.m_lmcsEnabled;
  sps.m_useMRL                     = m_encCfg.m_MRL;
  sps.m_useMIP                     = m_encCfg.m_MIP;
  sps.m_usedirPlanar               = m_encCfg.m_dirPlanar;
  sps.m_useDIMD                    = m_encCfg.m_DIMD;
  sps.m_useDIMDChroma              = m_encCfg.m_DIMDChroma;
  sps.m_useTIMD                    = m_encCfg.m_TIMD;
  sps.m_useTIMDSAD                 = m_encCfg.m_TIMDSAD;
  sps.m_useOBIC                    = m_encCfg.m_OBIC;
  sps.m_useEIP                     = m_encCfg.m_EIP;
  sps.m_useMMEIP                   = m_encCfg.m_MMEIP;
  sps.m_ccBoostFilter              = m_encCfg.m_ccBoostFilter;
  sps.m_ccBoostTplRefSel           = m_encCfg.m_ccBoostTplRefSel;
  sps.m_ccMerge                    = m_encCfg.m_ccMerge;
  sps.m_ccMergeFusion              = m_encCfg.m_ccMergeFusion;
  sps.m_ccDecDerivedMode           = m_encCfg.m_ccDecDerivedMode;
  sps.m_affineSbMrgExt             = m_encCfg.m_affineSbMrgExt;
  sps.m_bvgCccm                    = m_encCfg.m_bvgCccm;
  sps.m_tempPartPredEnabledFlag    = m_encCfg.m_tempPartPredEnabled;

  sps.m_interMTSMaxSize = m_encCfg.m_interMTSMaxSize;
  CHECK(m_encCfg.m_log2MinCUSize > std::min(6, floorLog2(sps.m_maxCuWidth)),
        "sps_log2_min_luma_coding_block_size_minus2 shall be in the range of 0 to min (4, log2_ctu_size - 2)");
  CHECK(sps.m_maxMTTHierarchyDepth[1] > 2 * (floorLog2(sps.m_ctuSize) - sps.m_log2MinCodingBlockSize),
        "sps_max_mtt_hierarchy_depth_inter_slice shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)");
  CHECK(m_encCfg.m_uiMaxMTTHierarchyDepthI > 2 * (floorLog2(sps.m_ctuSize) - sps.m_log2MinCodingBlockSize),
        "sps_max_mtt_hierarchy_depth_intra_slice_luma shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)");
  CHECK(m_encCfg.m_uiMaxMTTHierarchyDepthIChroma > 2 * (floorLog2(sps.m_ctuSize) - sps.m_log2MinCodingBlockSize),
        "sps_max_mtt_hierarchy_depth_intra_slice_chroma shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)");

  sps.m_transformSkipEnabledFlag      = m_encCfg.m_useTransformSkip;
  sps.m_log2MaxTransformSkipBlockSize = m_encCfg.m_log2MaxTransformSkipBlockSize;
  sps.m_bdpcmEnabledFlag              = m_encCfg.m_useBDPCM;

  sps.m_temporalMvpEnabledFlag = (m_encCfg.m_TMVPModeId == 2 || m_encCfg.m_TMVPModeId == 1);

  sps.m_log2MaxTbSize = m_encCfg.m_log2MaxTbSize;

  for (const auto channelType: { ChannelType::LUMA, ChannelType::CHROMA })
  {
    sps.m_bitDepths[channelType]  = m_encCfg.m_internalBitDepth[channelType];
    sps.m_qpBDOffset[channelType] = (6 * (m_encCfg.m_internalBitDepth[channelType] - 8));
    sps.m_internalMinusInputBitDepth[channelType] =
      std::max(0, (m_encCfg.m_internalBitDepth[channelType] - m_encCfg.m_inputBitDepth[channelType]));
  }

  sps.m_entropyCodingSyncEnabledFlag = m_encCfg.m_entropyCodingSyncEnabledFlag;
  sps.m_entryPointPresentFlag        = m_encCfg.m_entryPointPresentFlag;

  sps.m_useWP             = m_encCfg.m_useWeightedPred;
  sps.m_useBiWP           = m_encCfg.m_useWeightedBiPred;
  sps.m_useAdditionalCMVP = m_encCfg.m_bUseAdditionalCMVP;

#if ENABLE_NNLF
  sps.m_nnlf = m_encCfg.m_nnlf;
  CHECK(sps.m_nnlf < NNLFUnifiedID::OFF || sps.m_nnlf >= NNLFUnifiedID::MAX, "NNLFUnifiedID out of range");
  sps.m_nnlfStore = sps.m_nnlf > 0;
#endif

  sps.m_saoEnabledFlag       = m_encCfg.m_useSao;
  sps.m_ccSaoEnabledFlag     = (bool)m_encCfg.m_CCSAO;
  sps.m_ccSaoFastFlag        = ((m_encCfg.m_CCSAO == 2) ? true : false);
  sps.m_jointCbCrEnabledFlag = m_encCfg.m_jointCbCrMode;
  CHECK(m_encCfg.m_maxTempLayer > MAX_TLAYER, "Invalid number T-layers");
  sps.m_maxSubLayers          = m_encCfg.m_maxTempLayer;
  sps.m_temporalIdNestingFlag = ((m_encCfg.m_maxTempLayer == 1) ? true : false);

  for (int i = 0; i < std::min(sps.m_maxSubLayers, (uint32_t)MAX_TLAYER); i++)
  {
    sps.m_maxDecPicBuffering[i] = m_encCfg.m_maxDecPicBuffering[i];
    sps.m_maxNumReorderPics[i]  = m_encCfg.m_maxNumReorderPics[i];
  }

  sps.m_scalingListEnabledFlag     = (m_encCfg.m_useScalingListId == SCALING_LIST_OFF) ? 0 : 1;
  sps.m_alfEnabledFlag             = m_encCfg.m_alf;
  sps.m_alfImprovementsEnabledFlag = sps.m_alfEnabledFlag && m_encCfg.m_alfImprovements;
  sps.m_ccalfEnabledFlag           = m_encCfg.m_ccalf;
  sps.m_lfCccmEnabledFlag          = (m_encCfg.m_lfCccm && isChromaEnabled(sps.m_chromaFormatIdc));
  sps.m_fieldSeqFlag               = m_encCfg.m_fieldSeqFlag;
  sps.m_vuiParametersPresentFlag   = m_encCfg.m_vuiParametersPresentFlag;

  if (sps.m_vuiParametersPresentFlag)
  {
    VUI *pcVUI                              = &sps.m_vuiParameters;
    pcVUI->m_aspectRatioInfoPresentFlag     = m_encCfg.m_aspectRatioInfoPresentFlag;
    pcVUI->m_aspectRatioConstantFlag        = !m_encCfg.m_seiCfg.m_sampleAspectRatioInfoSEIEnabled;
    pcVUI->m_aspectRatioIdc                 = m_encCfg.m_aspectRatioIdc;
    pcVUI->m_sarWidth                       = m_encCfg.m_sarWidth;
    pcVUI->m_sarHeight                      = m_encCfg.m_sarHeight;
    pcVUI->m_colourDescriptionPresentFlag   = m_encCfg.m_colourDescriptionPresentFlag;
    pcVUI->m_colourPrimaries                = m_encCfg.m_colourPrimaries;
    pcVUI->m_transferCharacteristics        = m_encCfg.m_transferCharacteristics;
    pcVUI->m_matrixCoefficients             = m_encCfg.m_matrixCoefficients;
    pcVUI->m_progressiveSourceFlag          = m_encCfg.m_progressiveSourceFlag;
    pcVUI->m_interlacedSourceFlag           = m_encCfg.m_interlacedSourceFlag;
    pcVUI->m_nonPackedFlag                  = m_encCfg.m_nonPackedConstraintFlag;
    pcVUI->m_nonProjectedFlag               = m_encCfg.m_nonProjectedConstraintFlag;
    pcVUI->m_chromaLocInfoPresentFlag       = m_encCfg.m_chromaLocInfoPresentFlag;
    pcVUI->m_chromaSampleLocTypeTopField    = m_encCfg.m_chromaSampleLocTypeTopField;
    pcVUI->m_chromaSampleLocTypeBottomField = m_encCfg.m_chromaSampleLocTypeBottomField;
    pcVUI->m_chromaSampleLocType            = m_encCfg.m_chromaSampleLocType;
    pcVUI->m_overscanInfoPresentFlag        = m_encCfg.m_overscanInfoPresentFlag;
    pcVUI->m_overscanAppropriateFlag        = m_encCfg.m_overscanAppropriateFlag;
    pcVUI->m_videoFullRangeFlag             = m_encCfg.m_videoFullRangeFlag;
  }

  sps.m_numLongTermRefPicSPS = NUM_LONG_TERM_REF_PIC_SPS;
  CHECK(!(NUM_LONG_TERM_REF_PIC_SPS <= MAX_NUM_LONG_TERM_REF_PICS), "Unspecified error");
  for (int k = 0; k < NUM_LONG_TERM_REF_PIC_SPS; k++)
  {
    sps.m_ltRefPicPocLsbSps[k]      = 0;
    sps.m_usedByCurrPicLtSPSFlag[k] = 0;
  }
  int numQpTables =
    m_encCfg.m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag ? 1 : (sps.m_jointCbCrEnabledFlag ? 3 : 2);
  m_encCfg.m_chromaQpMappingTableParams.m_numQpTables = numQpTables;
  sps.setChromaQpMappingTableFromParams(m_encCfg.m_chromaQpMappingTableParams, sps.m_qpBDOffset[ChannelType::CHROMA]);
  sps.deriveChromaQPMappingTables();

  if (m_encCfg.m_seiCfg.m_pictureTimingSEIEnabled || m_encCfg.m_seiCfg.m_decodingUnitInfoSEIEnabled ||
      m_encCfg.m_RCCpbSaturationEnabled)
  {
    xInitHrdParameters(sps);
  }
  if (m_encCfg.m_seiCfg.m_bufferingPeriodSEIEnabled || m_encCfg.m_seiCfg.m_pictureTimingSEIEnabled ||
      m_encCfg.m_seiCfg.m_decodingUnitInfoSEIEnabled)
  {
    sps.m_generalHrdParametersPresentFlag = true;
  }

  // Set up SPS range extension settings
  sps.m_spsRangeExtension.m_transformSkipRotationEnabledFlag    = m_encCfg.m_transformSkipRotationEnabledFlag;
  sps.m_spsRangeExtension.m_transformSkipContextEnabledFlag     = m_encCfg.m_transformSkipContextEnabledFlag;
  sps.m_spsRangeExtension.m_extendedPrecisionProcessingFlag     = m_encCfg.m_extendedPrecisionProcessingFlag;
  sps.m_spsRangeExtension.m_tsrcRicePresentFlag                 = m_encCfg.m_tsrcRicePresentFlag;
  sps.m_spsRangeExtension.m_highPrecisionOffsetsEnabledFlag     = m_encCfg.m_highPrecisionOffsetsEnabledFlag;
  sps.m_spsRangeExtension.m_rrcRiceExtensionEnableFlag          = m_encCfg.m_rrcRiceExtensionEnableFlag;
  sps.m_spsRangeExtension.m_persistentRiceAdaptationEnabledFlag = m_encCfg.m_persistentRiceAdaptationEnabledFlag;
  sps.m_spsRangeExtension.m_reverseLastSigCoeffEnabledFlag      = m_encCfg.m_reverseLastSigCoeffEnabledFlag;
  sps.m_spsRangeExtension.m_cabacBypassAlignmentEnabledFlag     = m_encCfg.m_cabacBypassAlignmentEnabledFlag;

  sps.m_subPicInfoPresentFlag = m_encCfg.m_subPicInfoPresentFlag;
  if (m_encCfg.m_subPicInfoPresentFlag)
  {
    sps.setNumSubPics(m_encCfg.m_numSubPics);
    sps.m_subPicSameSizeFlag = m_encCfg.m_subPicSameSizeFlag;
    if (m_encCfg.m_subPicSameSizeFlag)
    {
      uint32_t numSubpicCols =
        (m_encCfg.m_sourceWidth + m_encCfg.m_CTUSize - 1) / m_encCfg.m_CTUSize / m_encCfg.m_subPicWidth[0];
      for (unsigned int i = 0; i < m_encCfg.m_numSubPics; i++)
      {
        sps.m_subPicCtuTopLeftX[i] = (i % numSubpicCols) * m_encCfg.m_subPicWidth[0];
        sps.m_subPicCtuTopLeftY[i] = (i / numSubpicCols) * m_encCfg.m_subPicHeight[0];
        sps.m_subPicWidth[i]       = m_encCfg.m_subPicWidth[0];
        sps.m_subPicHeight[i]      = m_encCfg.m_subPicHeight[0];
      }
    }
    else
    {
      sps.m_subPicCtuTopLeftX = m_encCfg.m_subPicCtuTopLeftX;
      sps.m_subPicCtuTopLeftY = m_encCfg.m_subPicCtuTopLeftY;
      sps.m_subPicWidth       = m_encCfg.m_subPicWidth;
      sps.m_subPicHeight      = m_encCfg.m_subPicHeight;
    }
    sps.m_subPicTreatedAsPicFlag                 = m_encCfg.m_subPicTreatedAsPicFlag;
    sps.m_loopFilterAcrossSubpicEnabledFlag      = m_encCfg.m_loopFilterAcrossSubpicEnabledFlag;
    sps.m_subPicIdLen                            = m_encCfg.m_subPicIdLen;
    sps.m_subPicIdMappingExplicitlySignalledFlag = m_encCfg.m_subPicIdMappingExplicitlySignalledFlag;
    if (m_encCfg.m_subPicIdMappingExplicitlySignalledFlag)
    {
      sps.m_subPicIdMappingPresentFlag = m_encCfg.m_subPicIdMappingInSpsFlag;
      if (m_encCfg.m_subPicIdMappingInSpsFlag)
      {
        sps.m_subPicId = m_encCfg.m_subPicId;
      }
    }
  }
  else   // In that case, there is only one subpicture that contains the whole picture
  {
    sps.setNumSubPics(1);
    sps.m_subPicCtuTopLeftX[0]                   = 0;
    sps.m_subPicCtuTopLeftY[0]                   = 0;
    sps.m_subPicWidth[0]                         = m_encCfg.m_sourceWidth;
    sps.m_subPicHeight[0]                        = m_encCfg.m_sourceHeight;
    sps.m_subPicTreatedAsPicFlag[0]              = 1;
    sps.m_loopFilterAcrossSubpicEnabledFlag[0]   = 0;
    sps.m_subPicIdLen                            = 0;
    sps.m_subPicIdMappingExplicitlySignalledFlag = false;
  }
  sps.m_depQuantEnabledFlag = m_encCfg.m_DepQuantEnabledIdc > 0;
  if (!sps.m_depQuantEnabledFlag)
  {
    sps.m_signDataHidingEnabledFlag = m_encCfg.m_SignDataHidingEnabledFlag;
  }
  else
  {
    sps.m_signDataHidingEnabledFlag = false;
  }
  sps.m_numPredSign       = m_encCfg.m_numPredSign;
  sps.m_log2SignPredArea  = m_encCfg.m_log2SignPredArea;
  sps.m_pdpEnabledFlag    = m_encCfg.m_pdp;
  sps.m_tempCabacInitMode = m_encCfg.m_tempCabacInitMode;
  sps.m_TMBP              = m_encCfg.m_TMBP;

  sps.m_interLayerPresentFlag = m_layerId > 0 && m_vps->m_maxLayers > 1 && !m_vps->m_vpsAllIndependentLayersFlag &&
    !m_vps->m_vpsIndependentLayerFlag[m_vps->m_generalLayerIdx[m_layerId]];
  CHECK(m_vps->m_vpsIndependentLayerFlag[m_vps->m_generalLayerIdx[m_layerId]] && sps.m_interLayerPresentFlag,
        " When vps_independent_layer_flag[GeneralLayerIdx[nuh_layer_id ]]  is equal to 1, the value of "
        "inter_layer_ref_pics_present_flag shall be equal to 0.");

  sps.m_resChangeInClvsEnabledFlag = m_encCfg.m_resChangeInClvsEnabled || m_encCfg.m_constrainedRaslEncoding;
  sps.m_rprEnabledFlag             = m_encCfg.m_rprEnabledFlag;

  sps.m_log2ParallelMergeLevelMinus2 = m_encCfg.m_log2ParallelMergeLevel - 2;
  sps.m_licEnabledFlag               = m_encCfg.m_licMode != 0;
  sps.m_biLicEnabledFlag             = m_encCfg.m_licMode == 2;
  sps.m_useInterRPL                  = m_encCfg.m_interRPL;
}

void EncLib::xInitHrdParameters(SPS &sps)
{
  m_encHRD.initHRDParameters(&m_encCfg);

  GeneralHrdParams *generalHrdParams = &sps.m_generalHrdParams;
  *generalHrdParams                  = m_encHRD.m_generalHrdParams;

  OlsHrdParams *spsOlsHrdParams = sps.m_olsHrdParams;
  for (int i = 0; i < MAX_TLAYER; i++)
  {
    *spsOlsHrdParams = m_encHRD.m_olsHrdParams[i];
    spsOlsHrdParams++;
  }
}

void EncLib::xInitPPS(PPS &pps, const SPS &sps)
{
  // pps ID already initialised.
  pps.m_spsId = sps.m_spsId;

  pps.setNumSubPics(sps.m_numSubPics);
  pps.m_subPicIdMappingInPpsFlag = false;
  pps.m_subPicIdLen              = sps.m_subPicIdLen;
  for (int picIdx = 0; picIdx < pps.m_numSubPics; picIdx++)
  {
    pps.m_subPicId[picIdx] = sps.m_subPicId[picIdx];
  }
  bool useDeltaQp = m_encCfg.m_cuQpDeltaSubdiv > 0;

  if (m_encCfg.m_iMaxDeltaQP != 0 || m_encCfg.m_bUseAdaptiveQP)
  {
    useDeltaQp = true;
  }

#if SHARP_LUMA_DELTA_QP
  if (m_encCfg.m_lumaLevelToDeltaQPMapping.isEnabled())
  {
    useDeltaQp = true;
  }
#endif
  if (m_encCfg.m_smoothQPReductionEnable)
  {
    useDeltaQp = true;
  }
#if ENABLE_QPA
  if (m_encCfg.m_bUsePerceptQPA && !useDeltaQp)
  {
    CHECK(m_encCfg.m_cuQpDeltaSubdiv != 0, "max. delta-QP subdiv must be zero!");
    useDeltaQp = (m_encCfg.m_iQP < 38) && (m_encCfg.m_sourceWidth > 512 || m_encCfg.m_sourceHeight > 320);
  }
#endif
  if (m_encCfg.m_bimEnabled == 1)
  {
    useDeltaQp = true;
  }

  if (m_encCfg.m_costMode == COST_SEQUENCE_LEVEL_LOSSLESS || m_encCfg.m_costMode == COST_LOSSLESS_CODING)
  {
    useDeltaQp = false;
  }

  pps.m_useDQP = (m_encCfg.m_RCEnableRateControl || useDeltaQp);

  if (m_encCfg.m_cuChromaQpOffsetList.size() > 0)
  {
    /* insert table entries from cfg parameters (NB, 0 should not be touched) */
    pps.m_chromaQpOffsetListLen = 0;
    for (int i = 0; i < m_encCfg.m_cuChromaQpOffsetList.size(); i++)
    {
      pps.setChromaQpOffsetListEntry(i + 1, m_encCfg.m_cuChromaQpOffsetList[i].u.comp.cbOffset,
                                     m_encCfg.m_cuChromaQpOffsetList[i].u.comp.crOffset,
                                     m_encCfg.m_cuChromaQpOffsetList[i].u.comp.jointCbCrOffset);
    }
  }
  else
  {
    pps.m_chromaQpOffsetListLen = 0;
  }
  {
    int baseQp = 26;
    if (16 == m_encCfg.m_gopSize)
    {
      baseQp = m_encCfg.m_iQP - 24;
    }
    else
    {
      baseQp = m_encCfg.m_iQP - 26;
    }
    if (pps.m_ppsId == ENC_PPS_ID_RPR)
    {
      baseQp += m_encCfg.m_qpOffsetRPR;
    }
    if (pps.m_ppsId == ENC_PPS_ID_RPR2)
    {
      baseQp += m_encCfg.m_qpOffsetRPR2;
    }
    if (pps.m_ppsId == ENC_PPS_ID_RPR3)
    {
      baseQp += m_encCfg.m_qpOffsetRPR3;
    }

    const int maxDQP = 37;
    const int minDQP = -26 + sps.m_qpBDOffset[ChannelType::LUMA];

    pps.m_picInitQPMinus26 = std::min(maxDQP, std::max(minDQP, baseQp));
  }

  if (!sps.m_jointCbCrEnabledFlag || !isChromaEnabled(m_encCfg.m_chromaFormatIdc))
  {
    pps.m_chromaJointCbCrQpOffsetPresentFlag = false;
  }
  else
  {
    bool enable = (m_encCfg.m_chromaCbCrQpOffset != 0);
    for (int i = 0; i < m_encCfg.m_cuChromaQpOffsetList.size(); i++)
    {
      enable |= (m_encCfg.m_cuChromaQpOffsetList[i].u.comp.jointCbCrOffset != 0);
    }
    pps.m_chromaJointCbCrQpOffsetPresentFlag = enable;
  }

#if ER_CHROMA_QP_WCG_PPS
  if (m_encCfg.m_wcgChromaQpControl.enabled)
  {
    const int    baseQp = m_encCfg.m_iQP + pps.m_ppsId;
    const double chromaQp =
      m_encCfg.m_wcgChromaQpControl.chromaQpScale * baseQp + m_encCfg.m_wcgChromaQpControl.chromaQpOffset;
    const double dcbQP = m_encCfg.m_wcgChromaQpControl.chromaCbQpScale * chromaQp;
    const double dcrQP = m_encCfg.m_wcgChromaQpControl.chromaCrQpScale * chromaQp;
    const int    cbQP  = (int)(dcbQP + (dcbQP < 0 ? -0.5 : 0.5));
    const int    crQP  = (int)(dcrQP + (dcrQP < 0 ? -0.5 : 0.5));
    pps.setQpOffset(COMP_Cb, Clip3(-12, 12, std::min(0, cbQP) + m_encCfg.m_chromaCbQpOffset));
    pps.setQpOffset(COMP_Cr, Clip3(-12, 12, std::min(0, crQP) + m_encCfg.m_chromaCrQpOffset));
    if (pps.m_chromaJointCbCrQpOffsetPresentFlag)
    {
      pps.setQpOffset(JOINT_CbCr,
                      Clip3(-12, 12, (std::min(0, cbQP) + std::min(0, crQP)) / 2 + m_encCfg.m_chromaCbCrQpOffset));
    }
    else
    {
      pps.setQpOffset(JOINT_CbCr, 0);
    }
  }
  else
  {
#endif
    pps.setQpOffset(COMP_Cb, m_encCfg.m_chromaCbQpOffset);
    pps.setQpOffset(COMP_Cr, m_encCfg.m_chromaCrQpOffset);
    if (pps.m_chromaJointCbCrQpOffsetPresentFlag)
    {
      pps.setQpOffset(JOINT_CbCr, m_encCfg.m_chromaCbCrQpOffset);
    }
    else
    {
      pps.setQpOffset(JOINT_CbCr, 0);
    }
#if ER_CHROMA_QP_WCG_PPS
  }
#endif
#if W0038_CQP_ADJ
  bool chromaDeltaQpEnabled = false;
  {
    chromaDeltaQpEnabled =
      (m_encCfg.m_sliceChromaQpOffsetIntraOrPeriodic[0] || m_encCfg.m_sliceChromaQpOffsetIntraOrPeriodic[1]);
    if (!chromaDeltaQpEnabled)
    {
      for (int i = 0; i < m_encCfg.m_gopSize; i++)
      {
        if (m_encCfg.m_GOPList[i].m_CbQPoffset || m_encCfg.m_GOPList[i].m_CrQPoffset)
        {
          chromaDeltaQpEnabled = true;
          break;
        }
      }
    }
  }
#if ENABLE_QPA
  if ((m_encCfg.m_bUsePerceptQPA || m_encCfg.m_sliceChromaQpOffsetPeriodicity > 0) &&
      isChromaEnabled(m_encCfg.m_chromaFormatIdc))
  {
    chromaDeltaQpEnabled = true;
  }
#endif
  pps.m_sliceChromaQpFlag = chromaDeltaQpEnabled;
#endif
  if (!pps.m_sliceChromaQpFlag && sps.m_dualITree && isChromaEnabled(m_encCfg.m_chromaFormatIdc))
  {
    pps.m_sliceChromaQpFlag = m_encCfg.m_chromaCbQpOffsetDualTree != 0 || m_encCfg.m_chromaCrQpOffsetDualTree != 0 ||
      m_encCfg.m_chromaCbCrQpOffsetDualTree != 0;
  }
  if (m_encCfg.m_gopBasedRPREnabledFlag || m_encCfg.m_rprFunctionalityTestingEnabledFlag)
  {
    bool isRprPPS = false;
    for (int nr = 0; nr < NUM_RPR_PPS; nr++)
    {
      if ((pps.m_ppsId == RPR_PPS_ID[nr]) && (RPR_PPS_ID[nr] != 0))
      {
        isRprPPS = true;
      }
    }
    if (isRprPPS && isChromaEnabled(m_encCfg.m_chromaFormatIdc))
    {
      pps.m_sliceChromaQpFlag = true;
    }
  }
  int minCbSizeY              = (1 << sps.m_log2MinCodingBlockSize);
  pps.m_wrapAroundEnabledFlag = m_encCfg.m_wrapAround;
  if (m_encCfg.m_wrapAround)
  {
    pps.m_picWidthMinusWrapAroundOffset =
      (pps.m_picWidthInLumaSamples / minCbSizeY) - (m_encCfg.m_wrapAroundOffset / minCbSizeY);
    pps.m_wrapAroundOffset =
      minCbSizeY * (pps.m_picWidthInLumaSamples / minCbSizeY - pps.m_picWidthMinusWrapAroundOffset);
  }
  else
  {
    pps.m_picWidthMinusWrapAroundOffset = 0;
    pps.m_wrapAroundOffset              = 0;
  }
  CHECK(!sps.m_wrapAroundEnabledFlag && pps.m_wrapAroundEnabledFlag,
        "When sps_ref_wraparound_enabled_flag is equal to 0, the value of pps_ref_wraparound_enabled_flag shall be "
        "equal to 0.");
  CHECK((((sps.m_ctuSize / minCbSizeY) + 1) > ((pps.m_picWidthInLumaSamples / minCbSizeY) - 1)) &&
          pps.m_wrapAroundEnabledFlag,
        "When the value of CtbSizeY / MinCbSizeY + 1 is greater than pps_pic_width_in_luma_samples / MinCbSizeY - 1, "
        "the value of pps_ref_wraparound_enabled_flag shall be equal to 0.");

  pps.m_noPicPartitionFlag = !m_encCfg.m_picPartitionFlag;
  if (m_encCfg.m_picPartitionFlag)
  {
    pps.setLog2CtuSize(ceilLog2(sps.m_ctuSize));
    pps.m_numExpTileCols = (uint32_t)m_encCfg.m_tileColumnWidth.size();
    pps.m_numExpTileRows = (uint32_t)m_encCfg.m_tileRowHeight.size();
    pps.m_tileColWidth   = m_encCfg.m_tileColumnWidth;
    pps.m_tileRowHeight  = m_encCfg.m_tileRowHeight;
    pps.initTiles();
    pps.m_rectSliceFlag = !m_encCfg.m_rasterSliceFlag;
    if (m_encCfg.m_rasterSliceFlag)
    {
      pps.initRasterSliceMap(m_encCfg.m_rasterSliceSize);
    }
    else
    {
      pps.m_singleSlicePerSubPicFlag = m_encCfg.m_singleSlicePerSubPicFlag;
      pps.m_numSlicesInPic           = m_encCfg.m_numSlicesInPic;
      CHECK(pps.m_numSlicesInPic > MAX_SLICES, "Number of slices in picture exceeds valid range");
      pps.m_tileIdxDeltaPresentFlag = m_encCfg.m_tileIdxDeltaPresentFlag;
      pps.m_rectSlices              = m_encCfg.m_rectSlices;
      pps.initRectSliceMap(&sps);
    }
    pps.initSubPic(sps);
    pps.m_loopFilterAcrossTilesEnabledFlag  = !m_encCfg.m_disableLFCrossTileBoundaryFlag;
    pps.m_loopFilterAcrossSlicesEnabledFlag = !m_encCfg.m_disableLFCrossSliceBoundaryFlag;
  }
  else
  {
    pps.setLog2CtuSize(ceilLog2(sps.m_ctuSize));
    pps.m_numExpTileCols = 1;
    pps.m_numExpTileRows = 1;
    pps.addTileColumnWidth(pps.m_picWidthInCtu);
    pps.addTileRowHeight(pps.m_picHeightInCtu);
    pps.initTiles();
    pps.m_rectSliceFlag  = 1;
    pps.m_numSlicesInPic = 1;
    pps.initRectSlices();
    pps.m_tileIdxDeltaPresentFlag = 0;
    pps.m_rectSlices[0].m_tileIdx = 0;
    pps.initRectSliceMap(&sps);
    pps.initSubPic(sps);
    pps.m_loopFilterAcrossTilesEnabledFlag  = true;
    pps.m_loopFilterAcrossSlicesEnabledFlag = true;
  }

  pps.m_useWP                 = m_encCfg.m_useWeightedPred;
  pps.m_useBiWP               = m_encCfg.m_useWeightedBiPred;
  pps.m_outputFlagPresentFlag = false;

  if (m_encCfg.m_deblockingFilterMetric)
  {
    pps.m_deblockingFilterOverrideEnabledFlag = true;
    pps.m_ppsDeblockingFilterDisabledFlag     = false;
  }
  else
  {
    pps.m_deblockingFilterOverrideEnabledFlag = !m_encCfg.m_deblockingFilterOffsetInPPS;
    pps.m_ppsDeblockingFilterDisabledFlag     = m_encCfg.m_deblockingFilterDisable;
  }

  if (!pps.m_ppsDeblockingFilterDisabledFlag)
  {
    pps.m_deblockingFilterBetaOffsetDiv2   = m_encCfg.m_deblockingFilterBetaOffsetDiv2;
    pps.m_deblockingFilterTcOffsetDiv2     = m_encCfg.m_deblockingFilterTcOffsetDiv2;
    pps.m_deblockingFilterCbBetaOffsetDiv2 = m_encCfg.m_deblockingFilterCbBetaOffsetDiv2;
    pps.m_deblockingFilterCbTcOffsetDiv2   = m_encCfg.m_deblockingFilterCbTcOffsetDiv2;
    pps.m_deblockingFilterCrBetaOffsetDiv2 = m_encCfg.m_deblockingFilterCrBetaOffsetDiv2;
    pps.m_deblockingFilterCrTcOffsetDiv2   = m_encCfg.m_deblockingFilterCrTcOffsetDiv2;
  }
  else
  {
    pps.m_deblockingFilterBetaOffsetDiv2   = 0;
    pps.m_deblockingFilterTcOffsetDiv2     = 0;
    pps.m_deblockingFilterCbBetaOffsetDiv2 = 0;
    pps.m_deblockingFilterCbTcOffsetDiv2   = 0;
    pps.m_deblockingFilterCrBetaOffsetDiv2 = 0;
    pps.m_deblockingFilterCrTcOffsetDiv2   = 0;
  }
  pps.m_BIF                                     = m_encCfg.m_BIF;
  pps.m_BIFStrength                             = m_encCfg.m_BIFStrength;
  pps.m_BIFQPOffset                             = m_encCfg.m_BIFQPOffset;
  pps.m_chromaBIF                               = m_encCfg.m_chromaBIF;
  pps.m_chromaBIFStrength                       = m_encCfg.m_chromaBIFStrength;
  pps.m_chromaBIFQPOffset                       = m_encCfg.m_chromaBIFQPOffset;
  // deblockingFilterControlPresentFlag is true if any of the settings differ from the inferred values:
  const bool deblockingFilterControlPresentFlag = pps.m_deblockingFilterOverrideEnabledFlag ||
    pps.m_ppsDeblockingFilterDisabledFlag || pps.m_deblockingFilterBetaOffsetDiv2 != 0 ||
    pps.m_deblockingFilterTcOffsetDiv2 != 0 || pps.m_deblockingFilterCbBetaOffsetDiv2 != 0 ||
    pps.m_deblockingFilterCbTcOffsetDiv2 != 0 || pps.m_deblockingFilterCrBetaOffsetDiv2 != 0 ||
    pps.m_deblockingFilterCrTcOffsetDiv2 != 0;

  pps.m_deblockingFilterControlPresentFlag = deblockingFilterControlPresentFlag;

  pps.m_cabacInitPresentFlag              = CABAC_INIT_PRESENT_FLAG;
  pps.m_loopFilterAcrossSlicesEnabledFlag = !m_encCfg.m_disableLFCrossSliceBoundaryFlag;

  bool chromaQPOffsetNotZero = false;
  if (pps.getQpOffset(COMP_Cb) != 0 || pps.getQpOffset(COMP_Cr) != 0 || pps.m_chromaJointCbCrQpOffsetPresentFlag ||
      pps.m_sliceChromaQpFlag || pps.getCuChromaQpOffsetListEnabledFlag())
  {
    chromaQPOffsetNotZero = true;
  }
  bool chromaDbfOffsetNotSameAsLuma = true;
  if (pps.m_deblockingFilterCbBetaOffsetDiv2 == pps.m_deblockingFilterBetaOffsetDiv2 &&
      pps.m_deblockingFilterCrBetaOffsetDiv2 == pps.m_deblockingFilterBetaOffsetDiv2 &&
      pps.m_deblockingFilterCbTcOffsetDiv2 == pps.m_deblockingFilterTcOffsetDiv2 &&
      pps.m_deblockingFilterCrTcOffsetDiv2 == pps.m_deblockingFilterTcOffsetDiv2)
  {
    chromaDbfOffsetNotSameAsLuma = false;
  }
  if (isChromaEnabled(sps.m_chromaFormatIdc) && (chromaQPOffsetNotZero || chromaDbfOffsetNotSameAsLuma))
  {
    pps.m_usePPSChromaTool = true;
  }
  else
  {
    pps.m_usePPSChromaTool = false;
  }

  int histogram[MAX_NUM_REF + 1];
  for (int i = 0; i <= MAX_NUM_REF; i++)
  {
    histogram[i] = 0;
  }
  for (int i = 0; i < m_encCfg.m_gopSize; i++)
  {
    CHECK(!(m_encCfg.m_RPLList0[i].m_numRefPicsActive >= 0 && m_encCfg.m_RPLList0[i].m_numRefPicsActive <= MAX_NUM_REF),
          "Unspecified error");
    histogram[m_encCfg.m_RPLList0[i].m_numRefPicsActive]++;
  }

  int maxHist = -1;
  int bestPos = 0;
  for (int i = 0; i <= MAX_NUM_REF; i++)
  {
    if (histogram[i] > maxHist)
    {
      maxHist = histogram[i];
      bestPos = i;
    }
  }
  CHECK(bestPos > 15, "Unspecified error");
  pps.m_numRefIdxDefaultActive[RPL0]      = bestPos;
  pps.m_numRefIdxDefaultActive[RPL1]      = bestPos;
  pps.m_pictureHeaderExtensionPresentFlag = false;

  pps.m_rplInfoInPhFlag     = m_encCfg.m_sliceLevelRpl ? false : true;
  pps.m_dbfInfoInPhFlag     = m_encCfg.m_sliceLevelDblk ? false : true;
  pps.m_saoInfoInPhFlag     = m_encCfg.m_sliceLevelSao ? false : true;
  pps.m_alfInfoInPhFlag     = m_encCfg.m_sliceLevelAlf ? false : true;
  pps.m_wpInfoInPhFlag      = m_encCfg.m_sliceLevelWp ? false : true;
  pps.m_qpDeltaInfoInPhFlag = m_encCfg.m_sliceLevelDeltaQp ? false : true;
  pps.m_useSgpmNoBlend      = m_encCfg.m_sgpmNoBlend;

  pps.pcv                  = new PreCalcValues(sps, pps, true);
  pps.m_rpl1IdxPresentFlag = sps.m_rpl1IdxPresentFlag;
}

void EncLib::xInitPicHeader(PicHeader &picHeader, const SPS &sps, const PPS &pps)
{
  picHeader.initPicHeader();

  // parameter sets
  picHeader.m_spsId = sps.m_spsId;
  picHeader.m_ppsId = pps.m_ppsId;

  // merge list sizes
  picHeader.m_maxNumAffineMergeCand = m_encCfg.m_maxNumAffineMergeCand;
  // copy partitioning constraints from SPS
  picHeader.m_splitConsOverrideFlag = false;
  picHeader.setMinQTSizes(sps.m_minQT);
  picHeader.setMaxMTTHierarchyDepths(sps.m_maxMTTHierarchyDepth);
  picHeader.setMaxBTSizes(sps.m_maxBTSize);
  picHeader.setMaxTTSizes(sps.m_maxTTSize);

  bool useDeltaQp = m_encCfg.m_cuQpDeltaSubdiv > 0;

  if ((m_encCfg.m_iMaxDeltaQP != 0) || m_encCfg.m_bUseAdaptiveQP)
  {
    useDeltaQp = true;
  }

#if SHARP_LUMA_DELTA_QP
  if (m_encCfg.m_lumaLevelToDeltaQPMapping.isEnabled())
  {
    useDeltaQp = true;
  }
#endif
  if (m_encCfg.m_smoothQPReductionEnable)
  {
    useDeltaQp = true;
  }
#if ENABLE_QPA
  if (m_encCfg.m_bUsePerceptQPA && !useDeltaQp)
  {
    CHECK(m_encCfg.m_cuQpDeltaSubdiv != 0, "max. delta-QP subdiv must be zero!");
    useDeltaQp = (m_encCfg.m_iQP < 38) && (m_encCfg.m_sourceWidth > 512 || m_encCfg.m_sourceHeight > 320);
  }
#endif

  if (m_encCfg.m_costMode == COST_SEQUENCE_LEVEL_LOSSLESS || m_encCfg.m_costMode == COST_LOSSLESS_CODING)
  {
    useDeltaQp = false;
  }

  if (m_encCfg.m_RCEnableRateControl)
  {
    picHeader.m_cuQpDeltaSubdivIntra = 0;
    picHeader.m_cuQpDeltaSubdivInter = 0;
  }
  else if (useDeltaQp)
  {
    picHeader.m_cuQpDeltaSubdivIntra = m_encCfg.m_cuQpDeltaSubdiv;
    picHeader.m_cuQpDeltaSubdivInter = m_encCfg.m_cuQpDeltaSubdiv;
  }
  else
  {
    picHeader.m_cuQpDeltaSubdivIntra = 0;
    picHeader.m_cuQpDeltaSubdivInter = 0;
  }

  picHeader.m_cuChromaQpOffsetSubdivIntra = m_encCfg.m_cuChromaQpOffsetSubdiv;
  picHeader.m_cuChromaQpOffsetSubdivInter = m_encCfg.m_cuChromaQpOffsetSubdiv;

  // gradual decoder refresh flag
  picHeader.m_gdrPicFlag = false;

  // BDOF / DMVR / PROF
  picHeader.m_bdofDisabledFlag = false;
  picHeader.m_profDisabledFlag = !sps.m_usePROF;

  picHeader.m_gpmMMVDTableFlag =
    sps.m_useGeo && (m_encCfg.m_intraPeriod > 0) && (m_encCfg.m_sourceWidth * m_encCfg.m_sourceHeight) <= (1920 * 1080);
  picHeader.m_gpmMMVDTableFlag = false;   // th disable for testing

#if ENABLE_NNLF
  picHeader.m_nnlfDisabled = false;
#endif
}

void EncLib::xInitAPS(APS &aps)
{
  // Do nothing now
}

void EncLib::xInitRPL(SPS &sps)
{
  const bool isFieldCoding = sps.m_fieldSeqFlag;

  int      numRPLCandidates = m_encCfg.m_numRPLList0;
  // To allocate one additional memory for RPL of POC1 (first bottom field) which is not specified in cfg file
  int      layerIdx         = m_vps == nullptr ? 0 : m_vps->m_generalLayerIdx[m_layerId];
  RPLList *rplLists[2];
  bool     codeRplInSH = layerIdx > 0 && m_encCfg.m_rplOfDepLayerInSh && m_encCfg.m_numRefLayers[layerIdx] > 0 &&
    (m_encCfg.m_avoidIntraInDepLayer || m_encCfg.m_intraPeriod > 1);
  m_encCfg.m_rplOfDepLayerInSh = codeRplInSH;
  if (codeRplInSH)
  {
    for (const auto l: { RPL0, RPL1 })
    {
      sps.createRplList(l, 0);
      getRplList(l)->destroy();
      getRplList(l)->create(numRPLCandidates + (isFieldCoding ? 1 : 0));
      rplLists[l] = getRplList(l);
    }
  }
  else
  {
    for (const auto l: { RPL0, RPL1 })
    {
      getRplList(l)->create(0);
      sps.createRplList(l, numRPLCandidates + (isFieldCoding ? 1 : 0));
      rplLists[l] = &sps.m_rplList[l];
    }
  }

  static_vector<int, MAX_VPS_LAYERS> refLayersIdx;
  if (layerIdx > 0 && !m_encCfg.m_rplOfDepLayerInSh)
  {
    if (m_encCfg.m_numRefLayers[layerIdx] > 0)
    {
      for (int refLayerIdx = 0; refLayerIdx < m_encCfg.m_numRefLayers[layerIdx]; refLayerIdx++)
      {
        if (m_vps->m_vpsDirectRefLayerFlag[layerIdx][refLayerIdx])
        {
          refLayersIdx.push_back(refLayerIdx);
        }
      }
    }
  }

  for (int i = 0; i < 2; i++)
  {
    RPLList *rplList = rplLists[i];
    for (int j = 0; j < numRPLCandidates; j++)
    {
      const RPLEntry       &ge          = (i == 0) ? m_encCfg.m_RPLList0[j] : m_encCfg.m_RPLList1[j];
      ReferencePictureList *rpl         = rplList->getReferencePictureList(j);
      rpl->m_numberOfShorttermPictures  = ge.m_numRefPics;
      rpl->m_numberOfLongtermPictures   = 0;   // Hardcoded as 0 for now. need to update this when implementing LTRP
      rpl->m_numberOfActivePictures     = ge.m_numRefPicsActive;
      rpl->m_ltrpInSliceHeaderFlag      = ge.m_ltrpInSliceHeaderFlag;
      rpl->m_interLayerPresentFlag      = sps.m_interLayerPresentFlag;
      // inter-layer reference picture is not signaled in SPS RPL, SPS is shared currently
      rpl->m_numberOfInterLayerPictures = 0;
      rpl->m_POCvalue                   = ge.m_POC;

      if (!m_encCfg.m_rplOfDepLayerInSh)
      {
        bool isIntraLayerPredAllowed = m_vps
          ? ((m_vps->m_vpsIndependentLayerFlag[layerIdx] || (m_vps->m_vpsCfgPredDirection[ge.m_temporalId] != 1)) &&
             (ge.m_POC % m_encCfg.m_intraPeriod) != 0)
          : true;
        bool isInterLayerPredAllowed = m_vps
          ? (!m_vps->m_vpsIndependentLayerFlag[layerIdx] && (m_vps->m_vpsCfgPredDirection[ge.m_temporalId] != 2) &&
             ((ge.m_POC % m_encCfg.m_intraPeriod) != 0 || (m_encCfg.m_avoidIntraInDepLayer && layerIdx)))
          : false;

        int numRefActive = 0;
        if (isIntraLayerPredAllowed)
        {
          for (int k = 0; k < ge.m_numRefPicsActive; k++)
          {
            rpl->setRefPicIdentifier(k, -ge.m_deltaRefPics[k], 0, false, 0);
          }
          numRefActive = ge.m_numRefPicsActive;
        }
        int validNumILRef = 0;
        if (isInterLayerPredAllowed)
        {
          for (int refLayerIdx: refLayersIdx)
          {
            rpl->setRefPicIdentifier(numRefActive + validNumILRef, 0, true, true,
                                     m_vps->m_interLayerRefIdx[layerIdx][refLayerIdx]);
            validNumILRef++;
          }
          rpl->m_numberOfInterLayerPictures = validNumILRef;
          rpl->m_numberOfActivePictures     = numRefActive + validNumILRef;
        }
        for (int k = numRefActive; k < ge.m_numRefPics; k++)
        {
          rpl->setRefPicIdentifier(k + validNumILRef, -ge.m_deltaRefPics[k], 0, false, 0);
        }
      }
      else
      {
        for (int k = 0; k < ge.m_numRefPics; k++)
        {
          rpl->setRefPicIdentifier(k, -ge.m_deltaRefPics[k], 0, false, 0);
        }
      }
    }
  }

  if (isFieldCoding)
  {
    // To set RPL of POC1 (first bottom field) which is not specified in cfg file
    for (int i = 0; i < 2; i++)
    {
      RPLList              *rplList    = rplLists[i];
      ReferencePictureList *rpl        = rplList->getReferencePictureList(numRPLCandidates);
      rpl->m_numberOfShorttermPictures = 1;
      rpl->m_numberOfLongtermPictures  = 0;
      rpl->m_numberOfActivePictures    = 1;
      rpl->m_ltrpInSliceHeaderFlag     = 0;
      rpl->setRefPicIdentifier(0, -1, 0, false, 0);
      rpl->m_POC[0] = 0;
    }
  }

  const int numRplsL0 = sps.m_numRpl[RPL0];
  const int numRplsL1 = sps.m_numRpl[RPL1];

  bool isRpl1CopiedFromRpl0 = numRplsL0 == numRplsL1;

  for (int i = 0; isRpl1CopiedFromRpl0 && i < numRplsL0; i++)
  {
    const int numEntriesL0 = sps.m_rplList[RPL0].getReferencePictureList(i)->getNumRefEntries();
    const int numEntriesL1 = sps.m_rplList[RPL1].getReferencePictureList(i)->getNumRefEntries();

    isRpl1CopiedFromRpl0 = numEntriesL0 == numEntriesL1;

    for (int j = 0; isRpl1CopiedFromRpl0 && j < numEntriesL0; j++)
    {
      const int entryL0 = sps.m_rplList[RPL0].getReferencePictureList(i)->m_refPicIdentifier[j];
      const int entryL1 = sps.m_rplList[RPL1].getReferencePictureList(i)->m_refPicIdentifier[j];

      isRpl1CopiedFromRpl0 = entryL0 == entryL1;
    }
  }

  sps.m_rpl1CopyFromRpl0Flag = isRpl1CopiedFromRpl0;

  // Check if all delta POC of STRP in each RPL has the same sign
  // Check RPLL0 first
  const RPLList *rplList0 = rplLists[0];
  const RPLList *rplList1 = rplLists[1];

  bool isAllEntriesinRPLHasSameSignFlag = true;
  for (uint32_t ii = 0; isAllEntriesinRPLHasSameSignFlag && ii < rplList0->getNumberOfReferencePictureLists(); ii++)
  {
    bool isFirstEntry   = true;
    bool prevSign       = true;
    int  prevIdentifier = 0;

    const ReferencePictureList *rpl = rplList0->getReferencePictureList(ii);
    for (uint32_t jj = 0; isAllEntriesinRPLHasSameSignFlag && jj < rpl->m_numberOfActivePictures; jj++)
    {
      if (!rpl->m_isLongtermRefPic[jj])
      {
        const int identifier = rpl->m_refPicIdentifier[jj];
        const int delta      = identifier - prevIdentifier;
        if (delta != 0)
        {
          const bool currentSign = delta >= 0;
          if (!isFirstEntry && currentSign != prevSign)
          {
            isAllEntriesinRPLHasSameSignFlag = false;
          }
          prevIdentifier = identifier;
          prevSign       = currentSign;
          isFirstEntry   = false;
        }
      }
    }
  }
  // Check RPLL1. Skip it if it is already found out that this flag is not true for RPL0 or if RPL1 is the same as RPL0
  for (uint32_t ii = 0; isAllEntriesinRPLHasSameSignFlag && !sps.m_rpl1CopyFromRpl0Flag &&
       ii < rplList1->getNumberOfReferencePictureLists();
       ii++)
  {
    bool isFirstEntry = true;
    bool lastSign     = true;

    const ReferencePictureList *rpl = rplList1->getReferencePictureList(ii);
    for (uint32_t jj = 0; isAllEntriesinRPLHasSameSignFlag && jj < rpl->m_numberOfActivePictures; jj++)
    {
      if (!rpl->m_isLongtermRefPic[jj])
      {
        if (isFirstEntry)
        {
          lastSign     = rpl->m_refPicIdentifier[jj] >= 0;
          isFirstEntry = false;
        }
        else
        {
          const bool currentSign = rpl->m_refPicIdentifier[jj] - rpl->m_refPicIdentifier[jj - 1] >= 0;
          if (currentSign != lastSign)
          {
            isAllEntriesinRPLHasSameSignFlag = false;
          }
        }
      }
    }
  }
  sps.m_allRplEntriesHasSameSignFlag = isAllEntriesinRPLHasSameSignFlag;

  // InterRPL
  if (sps.m_numRpl[RPL0] < 2)
  {
    sps.m_useInterRPL = 0;
  }
  if (sps.m_useInterRPL)
  {
    const int hierarchicalLevels = sps.m_maxSubLayers - 1;
    const int numberOfRPL        = sps.m_numRpl[RPL0];
    if (hierarchicalLevels > 0)
    {
      sps.m_QPoffsetRPL.resize(m_encCfg.m_gopSize);
    }
    else
    {
      sps.m_QPoffsetRPL.resize(numberOfRPL);
    }
    for (int pictureIdx = 0; pictureIdx < sps.m_QPoffsetRPL.size(); pictureIdx++)
    {
      int pictureQp;
      if (pictureIdx == 0 && hierarchicalLevels > 0)
      {
        pictureQp = getQPForPicture(pictureIdx, NULL, I_SLICE, NAL_UNIT_CODED_SLICE_CRA, 0);
      }
      else
      {
        pictureQp = getQPForPicture(pictureIdx % m_encCfg.m_gopSize, NULL, B_SLICE, NAL_UNIT_CODED_SLICE_TRAIL, 0);
      }
      sps.m_QPoffsetRPL[pictureIdx] = pictureQp - m_encCfg.m_iQP;
    }
  }
}

void EncLib::selectReferencePictureList(Slice *slice, int pocCurr, int gopId, int ltPoc)
{
  const bool isEncodeLtRef = (pocCurr == ltPoc);
  if (m_encCfg.m_compositeRefEnabled && isEncodeLtRef)
  {
    pocCurr++;
  }

  const RPLList *rplLists[NUM_RPL01];
  bool           codeRplInSH = m_encCfg.m_rplOfDepLayerInSh;
  int            rplIdx      = gopId;
  for (const auto l: { RPL0, RPL1 })
  {
    if (codeRplInSH)
    {
      rplLists[l]        = getRplList(l);
      slice->m_rplIdx[l] = -1;
    }
    else
    {
      rplLists[l] = &slice->m_sps->m_rplList[l];
    }
  }

  int fullListNum    = m_encCfg.m_gopSize;
  int partialListNum = m_encCfg.m_numRPLList0 - m_encCfg.m_gopSize;
  int extraNum       = fullListNum;

  int rplPeriod = m_encCfg.m_intraPeriod;
  if (rplPeriod < 0)   // Need to check if it is low delay or RA but with no RAP
  {
    if (rplLists[0]->getReferencePictureList(1)->m_refPicIdentifier[0] *
          rplLists[1]->getReferencePictureList(1)->m_refPicIdentifier[0] <
        0)
    {
      rplPeriod = m_encCfg.m_gopSize * 2;
    }
  }

  if (m_encCfg.m_isLowDelay)
  {
    const int currPOCsinceLastIDR = pocCurr - slice->m_iLastIDR;
    if (currPOCsinceLastIDR < (2 * m_encCfg.m_gopSize + 2))
    {
      int candidateIdx = (currPOCsinceLastIDR + m_encCfg.m_gopSize - 1 >= fullListNum + partialListNum)
        ? gopId
        : currPOCsinceLastIDR + m_encCfg.m_gopSize - 1;
      rplIdx           = candidateIdx;
    }
    else
    {
      rplIdx = (pocCurr % m_encCfg.m_gopSize == 0) ? m_encCfg.m_gopSize - 1 : pocCurr % m_encCfg.m_gopSize - 1;
    }
    extraNum = fullListNum + partialListNum;
  }
  for (; extraNum < fullListNum + partialListNum; extraNum++)
  {
    if (rplPeriod > 0)
    {
      int pocIndex = pocCurr % rplPeriod;
      if (pocIndex == 0)
      {
        pocIndex = rplPeriod;
      }
      if (pocIndex == m_encCfg.m_RPLList0[extraNum].m_POC)
      {
        rplIdx = extraNum;
        extraNum++;
      }
    }
  }

  if (slice->m_pic->m_fieldPic)
  {
    // To set RPL index of POC1 (first bottom field)
    if (pocCurr == 1)
    {
      slice->m_rplIdx[RPL0] = m_encCfg.m_numRPLList0;
      slice->m_rplIdx[RPL1] = m_encCfg.m_numRPLList0;
    }
    else if (rplPeriod < 0)
    {
      // To set RPL indexes for LD
      int numRPLCandidates = m_encCfg.m_numRPLList0;
      if (pocCurr < numRPLCandidates - m_encCfg.m_gopSize + 2)
      {
        rplIdx = pocCurr + m_encCfg.m_gopSize - 2;
      }
      else
      {
        if (pocCurr % m_encCfg.m_gopSize == 0)
        {
          rplIdx = m_encCfg.m_gopSize - 2;
        }
        else if (pocCurr % m_encCfg.m_gopSize == 1)
        {
          rplIdx = m_encCfg.m_gopSize - 1;
        }
        else
        {
          rplIdx = pocCurr % m_encCfg.m_gopSize - 2;
        }
      }
    }
  }

  slice->m_rpl[RPL0] = *rplLists[0]->getReferencePictureList(rplIdx);
  slice->m_rpl[RPL1] = *rplLists[1]->getReferencePictureList(rplIdx);

  if (!codeRplInSH)
  {
    slice->m_rplIdx[RPL0] = rplIdx;
    slice->m_rplIdx[RPL1] = rplIdx;
  }
}

void EncLib::setParamSetChanged(int spsId, int ppsId)
{
  m_ppsMap.setChangedFlag(ppsId);
  m_spsMap.setChangedFlag(spsId);
}

bool EncLib::PPSNeedsWriting(int ppsId)
{
  const bool changed = m_ppsMap.getChangedFlag(ppsId);
  m_ppsMap.clearChangedFlag(ppsId);
  return changed;
}

bool EncLib::SPSNeedsWriting(int spsId)
{
  const bool changed = m_spsMap.getChangedFlag(spsId);
  m_spsMap.clearChangedFlag(spsId);
  return changed;
}

void EncLib::checkPltStats(Picture *pic)
{
  int totalArea = 0;
  int pltArea   = 0;
  for (auto &apu: pic->m_cs->cus)
  {
    for (int i = 0; i < MAX_NUM_TBLOCKS; ++i)
    {
      int puArea = apu->blocks[i].width * apu->blocks[i].height;
      if (apu->blocks[i].width > 0 && apu->blocks[i].height > 0)
      {
        totalArea += puArea;
        if (CU::isPLT(*apu) || CU::isIBC(*apu))
        {
          pltArea += puArea;
        }
        break;
      }
    }
  }
  m_doPlt = pltArea * PLT_FAST_RATIO >= totalArea;
}

int EncLib::getQPForPicture(const uint32_t gopIndex, const Slice *pSlice, SliceType eSliceType,
                            NalUnitType eNalUnitType, int poc) const
{
  const int lumaQpBDOffset = (pSlice ? pSlice->m_sps->m_qpBDOffset[ChannelType::LUMA] : 0);
  int       qp;

  if (m_encCfg.m_costMode == COST_LOSSLESS_CODING)
  {
    qp = m_encCfg.m_iQP;
  }
  else
  {
    qp = m_encCfg.m_iQP;

    if (pSlice != NULL)
    {
      // switch at specific qp and keep this qp offset
      static int appliedSwitchDQQ = 0; /* TODO: MT */
      if (poc == m_encCfg.m_switchPOC)
      {
        appliedSwitchDQQ = m_encCfg.m_switchDQP;
      }
      qp += appliedSwitchDQQ;

      const std::vector<int> &deltaQps = m_encCfg.m_frameDeltaQps;
      if (deltaQps.size() != 0)
      {
        qp += deltaQps[poc / (m_encCfg.m_compositeRefEnabled ? 2 : 1)];
      }
    }

    if (eSliceType == I_SLICE)
    {
      qp += m_encCfg.m_intraQPOffset;
    }
    else
    {
      if (eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP || eNalUnitType == NAL_UNIT_CODED_SLICE_CRA)
      {
        qp += m_encCfg.m_intraQPOffset;
      }
      else
      {
        const GOPEntry &gopEntry = m_encCfg.m_GOPList[gopIndex];
        // adjust QP according to the QP offset for the GOP entry.
        qp += gopEntry.m_QPOffset;

        // adjust QP according to QPOffsetModel for the GOP entry.
        double dqpOffset = qp * gopEntry.m_QPOffsetModelScale + gopEntry.m_QPOffsetModelOffset + 0.5;
        int    qpOffset  = (int)floor(Clip3<double>(0.0, 3.0, dqpOffset));
        qp += qpOffset;
      }
    }
    if (pSlice != NULL)
    {
      if (m_encCfg.m_gopBasedRPREnabledFlag)
      {
        if (pSlice->m_pps->m_ppsId == ENC_PPS_ID_RPR)
        {
          qp += m_encCfg.m_qpOffsetRPR;
        }
        if (pSlice->m_pps->m_ppsId == ENC_PPS_ID_RPR2)
        {
          qp += m_encCfg.m_qpOffsetRPR2;
        }
        if (pSlice->m_pps->m_ppsId == ENC_PPS_ID_RPR3)
        {
          qp += m_encCfg.m_qpOffsetRPR3;
        }
      }
      if (!m_encCfg.m_gopBasedRPREnabledFlag && m_encCfg.m_rprFunctionalityTestingEnabledFlag)
      {
        int currPoc    = poc + m_encCfg.m_frameSkip;
        int rprSegment = currPoc / m_encCfg.m_rprSwitchingSegmentSize % m_encCfg.m_rprSwitchingListSize;
        qp += m_encCfg.m_rprSwitchingQPOffsetOrderList[rprSegment];
      }
    }
  }
  qp = Clip3(-lumaQpBDOffset, MAX_QP, qp);
  return qp;
}

//! \}
