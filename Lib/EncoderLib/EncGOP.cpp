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

/** \file     EncGOP.cpp
    \brief    GOP encoder class
*/

#include <list>
#include <algorithm>
#include <functional>

#include "EncLib.h"
#include "EncGOP.h"
#include "Analyze.h"
#include "libmd5/MD5.h"
#include "CommonLib/SEI.h"
#include "CommonLib/NAL.h"
#include "NALwrite.h"
#if ENABLE_NNLF
#include "EncNNFilterUnified.h"
#endif

#include <math.h>
#include <deque>
#include <chrono>
#include <cinttypes>

#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/ProfileTierLevel.h"

#include "DecoderLib/DecLib.h"

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================
EncGOP::EncGOP()
{
  m_iLastIDR                           = 0;
  m_iGopSize                           = 0;
  m_numPicsCoded                       = 0;
  m_first                              = true;
  m_latestDRAPPOC                      = MAX_INT;
  m_latestEDRAPPOC                     = MAX_INT;
  m_latestEdrapLeadingPicDecodableFlag = false;
  m_lastRasPoc                         = MAX_INT;
  ::memset(m_riceBit, 0, 8 * 2 * sizeof(unsigned));
  ::memset(m_preQP, MAX_INT, 2 * sizeof(int));
  m_preIPOC = 0;

  m_encCfg             = nullptr;
  m_pcSliceEncoder     = nullptr;
  m_picList            = nullptr;
  m_HLSWriter          = nullptr;
  m_seqFirst           = true;
  m_audIrapOrGdrAuFlag = false;

  m_refreshPending       = 0;
  m_pocCRA               = 0;
  m_numLongTermRefPicSPS = 0;
  ::memset(m_ltRefPicPocLsbSps, 0, sizeof(m_ltRefPicPocLsbSps));
  ::memset(m_ltRefPicUsedByCurrPicFlag, 0, sizeof(m_ltRefPicUsedByCurrPicFlag));
  ::memset(m_lastBPSEI, 0, sizeof(m_lastBPSEI));
  m_rapWithLeading                = false;
  m_bufferingPeriodSEIPresentInAU = false;
  for (int i = 0; i < MAX_VPS_LAYERS; i++)
  {
    m_associatedIRAPType[i] = NAL_UNIT_CODED_SLICE_IDR_N_LP;
  }
  ::memset(m_associatedIRAPPOC, 0, sizeof(m_associatedIRAPPOC));
  m_pcDeblockingTempPicYuv = nullptr;
  m_refLayerRescaledPicYuv = nullptr;

#if JVET_O0756_CALCULATE_HDRMETRICS
  m_ppcFrameOrg = nullptr;
  m_ppcFrameRec = nullptr;

  m_pcConvertFormat    = nullptr;
  m_pcConvertIQuantize = nullptr;
  m_pcColorTransform   = nullptr;
  m_pcDistortionDeltaE = nullptr;
  m_pcTransferFct      = nullptr;

  m_pcColorTransformParams = nullptr;
  m_pcFrameFormat          = nullptr;

  m_metricTime = std::chrono::milliseconds(0);
#endif

  m_blkStat.fill({ 0, 0 });
  m_bgPOC = -1;

  m_picBg   = nullptr;
  m_picOrig = nullptr;

  m_isEncodedLTRef      = false;
  m_isUseLTRef          = false;
  m_isPrepareLTRef      = true;
  m_lastLTRefPoc        = 0;
  m_cntRightBottom      = 0;
  m_cntRightBottomIntra = 0;

  m_useHashMeInCurrentIntraPeriod = false;
  m_HashMEPOC                     = 0;
  m_HashMEPOCchecked              = false;
  m_HashMEPOC2                    = 0;

#if ENABLE_NNLF
  m_nnlfEnabled = true;
#endif
}

EncGOP::~EncGOP()
{
  if (!m_encCfg->m_decodeBitstreams[0].empty() || !m_encCfg->m_decodeBitstreams[1].empty())
  {
    // reset potential decoder resources
    tryDecodePicture(nullptr, 0, std::string("")
#if ENABLE_NNLF
                                   ,
                     std::string(""), 0
#endif
    );
  }
#if JVET_O0756_CALCULATE_HDRMETRICS
  delete[] m_ppcFrameOrg;
  delete[] m_ppcFrameRec;

  m_ppcFrameOrg = m_ppcFrameRec = nullptr;

  delete m_pcConvertFormat;
  delete m_pcConvertIQuantize;
  delete m_pcColorTransform;
  delete m_pcDistortionDeltaE;
  delete m_pcTransferFct;
  delete m_pcColorTransformParams;
  delete m_pcFrameFormat;

  m_pcConvertFormat        = nullptr;
  m_pcConvertIQuantize     = nullptr;
  m_pcColorTransform       = nullptr;
  m_pcDistortionDeltaE     = nullptr;
  m_pcTransferFct          = nullptr;
  m_pcColorTransformParams = nullptr;
  m_pcFrameFormat          = nullptr;
#endif
}

/** Create list to contain pointers to CTU start addresses of slice.
 */
void EncGOP::create() {}

void EncGOP::destroy()
{
#if ENABLE_NNLF
  if (m_unifiedNnlf)
  {
    m_unifiedNnlf->destroy();
    delete m_unifiedNnlf;
    m_unifiedNnlf = nullptr;
  }
#endif
  if (m_pcDeblockingTempPicYuv)
  {
    m_pcDeblockingTempPicYuv->destroy();
    delete m_pcDeblockingTempPicYuv;
    m_pcDeblockingTempPicYuv = nullptr;
  }
  if (m_picBg)
  {
    m_picBg->destroy();
    delete m_picBg;
    m_picBg = nullptr;
  }
  if (m_picOrig)
  {
    m_picOrig->destroy();
    delete m_picOrig;
    m_picOrig = nullptr;
  }
  if (m_encCfg->m_seiCfg.m_fgcSEIAnalysisEnabled)
  {
    m_fgAnalyzer.destroy();
  }
  if (m_refLayerRescaledPicYuv)
  {
    m_refLayerRescaledPicYuv->destroy();
    delete m_refLayerRescaledPicYuv;
    m_refLayerRescaledPicYuv = nullptr;
  }
}

void EncGOP::init(EncLib *pcEncLib, EncModeCtrl *pcEncModeCtrl)
{
  m_encCfg   = &pcEncLib->m_encCfg;
  m_pcEncLib = pcEncLib;
  m_modeCtrl = pcEncModeCtrl;
  m_seiEncoder.init(m_encCfg, pcEncLib, this);
  m_pcSliceEncoder   = pcEncLib->getSliceEncoder();
  m_picList          = pcEncLib->getListPic();
  m_HLSWriter        = pcEncLib->getHLSWriter();
  m_pcLoopFilter     = pcEncLib->getDeblockingFilter();
  m_pcSAO            = pcEncLib->getSAO();
  m_alfEcm           = pcEncLib->getAlfEcm();
  m_alfVtm           = pcEncLib->getAlfVtm();
  m_pcRateCtrl       = pcEncLib->getRateCtrl();
  m_pcLoopFilterCccm = &pcEncLib->m_cEncLoopFilterCccm;

  ::memset(m_lastBPSEI, 0, sizeof(m_lastBPSEI));
  ::memset(m_totalCoded, 0, sizeof(m_totalCoded));
  m_HRD        = pcEncLib->getHRD();
  m_AUWriterIf = pcEncLib->getAUWriterIf();

  CHECK(m_encCfg->m_alf && m_encCfg->m_alfImprovements && (m_alfEcm == nullptr), "ECM ALF encoder is not set.");
  CHECK(m_encCfg->m_alf && !m_encCfg->m_alfImprovements && (m_alfVtm == nullptr), "VTM ALF encoder is not set.");

  if (m_encCfg->m_seiCfg.m_fgcSEIAnalysisEnabled)
  {
    m_fgAnalyzer.init(m_encCfg->m_sourceWidth, m_encCfg->m_sourceHeight, m_encCfg->m_sourcePadding[0],
                      m_encCfg->m_sourcePadding[1], IPCOLOURSPACE_UNCHANGED, false, m_encCfg->m_chromaFormatIdc,
                      m_encCfg->m_inputBitDepth, m_encCfg->m_internalBitDepth, m_encCfg->m_frameSkip,
                      m_encCfg->m_seiCfg.m_fgcSEICompModelPresent, m_encCfg->m_seiCfg.m_fgcSEIExternalMask,
                      m_encCfg->m_seiCfg.m_fgcSEIExternalDenoised);
  }

#if WCG_EXT
  if (m_encCfg->m_lmcsEnabled)
  {
    pcEncLib->getRdCost()->setReshapeInfo(m_encCfg->m_reshapeSignalType,
                                          m_encCfg->m_internalBitDepth[ChannelType::LUMA]);
    pcEncLib->getRdCost()->initLumaLevelToWeightTableReshape();
  }
  else if (m_encCfg->m_lumaLevelToDeltaQPMapping.mode)
  {
    pcEncLib->getRdCost()->setReshapeInfo(RESHAPE_SIGNAL_PQ, m_encCfg->m_internalBitDepth[ChannelType::LUMA]);
    pcEncLib->getRdCost()->initLumaLevelToWeightTableReshape();
  }
  else if (m_encCfg->m_printWPSNR)
  {
    pcEncLib->getRdCost()->initLumaLevelToWeightTable(m_encCfg->m_internalBitDepth[ChannelType::LUMA]);
  }

#if ENABLE_NNLF
  if (m_encCfg->m_nnlf)
  {
    m_nnlfEnabled = m_encCfg->m_nnlfStartPoc <= 0 ? true : false;
    if (m_unifiedNnlf == nullptr)
    {
      m_unifiedNnlf = new EncNNFilterUnified;
      CHECK(m_unifiedNnlf == nullptr, "out of memory");
    }
    m_unifiedNnlf->init(m_encCfg->m_nnlfModelName, m_encCfg->m_sourceWidth, m_encCfg->m_sourceHeight,
                        m_encCfg->m_chromaFormatIdc, NNLF_UNIFIED_MAX_NUM_PRMS);
  }
#endif

  if (m_encCfg->m_alf)
  {
    const bool alfWSSD = m_encCfg->m_lmcsEnabled && m_encCfg->m_reshapeSignalType == RESHAPE_SIGNAL_PQ;
    if (m_encCfg->m_alfImprovements)
    {
      m_alfEcm->setAlfWSSD(alfWSSD);
      if (alfWSSD)
      {
        m_alfEcm->setLumaLevelWeightTable(pcEncLib->getRdCost()->getLumaLevelWeightTable());
      }
    }
    else
    {
      m_alfVtm->setAlfWSSD(alfWSSD);
      if (alfWSSD)
      {
        m_alfVtm->setLumaLevelWeightTable(pcEncLib->getRdCost()->getLumaLevelWeightTable());
      }
    }
  }
#endif
  m_pcReshaper = pcEncLib->getReshaper();

#if JVET_O0756_CALCULATE_HDRMETRICS
  const bool calculateHdrMetrics = m_pcEncLib->m_calculateHdrMetrics;
  if (calculateHdrMetrics)
  {
    // allocate frame buffers and initialize class members
    const int chainNumber = 5;

    m_ppcFrameOrg = new hdrtoolslib::Frame *[chainNumber];
    m_ppcFrameRec = new hdrtoolslib::Frame *[chainNumber];

    double *whitePointDeltaE = new double[hdrtoolslib::NB_REF_WHITE];
    CHECK(hdrtoolslib::NB_REF_WHITE != 3, "config m_whitePointDeltaE array size mismatch");
    for (int i = 0; i < hdrtoolslib::NB_REF_WHITE; i++)
    {
      whitePointDeltaE[i] = m_encCfg->m_whitePointDeltaE[i];
    }

    double                   maxSampleValue    = m_encCfg->m_maxSampleValue;
    hdrtoolslib::SampleRange sampleRange       = static_cast<hdrtoolslib::SampleRange> m_encCfg->m_sampleRange;
    hdrtoolslib::ChromaFormat   chFmt          = hdrtoolslib::ChromaFormat(m_encCfg->m_chromaFormatIdc);
    int                         bitDepth       = m_encCfg->m_internalBitDepth[ChannelType::LUMA];
    hdrtoolslib::ColorPrimaries colorPrimaries = static_cast<hdrtoolslib::ColorPrimaries> m_encCfg->m_colorPrimaries;
    bool                         enableTFunctionLUT = m_encCfg->m_enableTFunctionLUT;
    hdrtoolslib::ChromaLocation *chromaLocation     = new hdrtoolslib::ChromaLocation[2];
    for (int i = 0; i < 2; i++)
    {
      chromaLocation[i] = static_cast<hdrtoolslib::ChromaLocation> m_encCfg->m_chromaLocation[i];
    }
    int chromaUpFilter   = m_encCfg->m_chromaUPFilter;
    int cropOffsetLeft   = m_encCfg->m_cropOffsetLeft;
    int cropOffsetTop    = m_encCfg->m_cropOffsetTop;
    int cropOffsetRight  = m_encCfg->m_cropOffsetRight;
    int cropOffsetBottom = m_encCfg->m_cropOffsetBottom;

    const int width  = m_encCfg->m_sourceWidth - cropOffsetLeft + cropOffsetRight;
    const int height = m_encCfg->m_sourceHeight - cropOffsetTop + cropOffsetBottom;

    m_ppcFrameOrg[0] = new hdrtoolslib::Frame(width, height, false, hdrtoolslib::CM_YCbCr, colorPrimaries, chFmt,
                                              sampleRange, bitDepth, false, hdrtoolslib::TF_PQ, 0);
    m_ppcFrameRec[0] = new hdrtoolslib::Frame(width, height, false, hdrtoolslib::CM_YCbCr, colorPrimaries, chFmt,
                                              sampleRange, bitDepth, false, hdrtoolslib::TF_PQ, 0);

    m_ppcFrameOrg[1] = new hdrtoolslib::Frame(
      m_ppcFrameOrg[0]->m_width[hdrtoolslib::Y_COMP], m_ppcFrameOrg[0]->m_height[hdrtoolslib::Y_COMP], false,
      hdrtoolslib::CM_YCbCr, colorPrimaries, hdrtoolslib::CF_444, sampleRange, bitDepth, false, hdrtoolslib::TF_PQ, 0);
    m_ppcFrameRec[1] = new hdrtoolslib::Frame(m_ppcFrameRec[0]->m_width[hdrtoolslib::Y_COMP],
                                              m_ppcFrameRec[0]->m_height[hdrtoolslib::Y_COMP], false,
                                              hdrtoolslib::CM_YCbCr, colorPrimaries, hdrtoolslib::CF_444, sampleRange,
                                              bitDepth, false, hdrtoolslib::TF_PQ, 0);   // 420 to 444 conversion

    m_ppcFrameOrg[2] = new hdrtoolslib::Frame(m_ppcFrameOrg[0]->m_width[hdrtoolslib::Y_COMP],
                                              m_ppcFrameOrg[0]->m_height[hdrtoolslib::Y_COMP], true,
                                              hdrtoolslib::CM_YCbCr, colorPrimaries, hdrtoolslib::CF_444,
                                              hdrtoolslib::SR_UNKNOWN, 32, false, hdrtoolslib::TF_PQ, 0);
    m_ppcFrameRec[2] = new hdrtoolslib::Frame(
      m_ppcFrameRec[0]->m_width[hdrtoolslib::Y_COMP], m_ppcFrameRec[0]->m_height[hdrtoolslib::Y_COMP], true,
      hdrtoolslib::CM_YCbCr, colorPrimaries, hdrtoolslib::CF_444, hdrtoolslib::SR_UNKNOWN, 32, false,
      hdrtoolslib::TF_PQ, 0);   // 444 to Float conversion

    m_ppcFrameOrg[3] = new hdrtoolslib::Frame(m_ppcFrameOrg[0]->m_width[hdrtoolslib::Y_COMP],
                                              m_ppcFrameOrg[0]->m_height[hdrtoolslib::Y_COMP], true,
                                              hdrtoolslib::CM_RGB, hdrtoolslib::CP_2020, hdrtoolslib::CF_444,
                                              hdrtoolslib::SR_UNKNOWN, 32, false, hdrtoolslib::TF_PQ, 0);
    m_ppcFrameRec[3] = new hdrtoolslib::Frame(
      m_ppcFrameRec[0]->m_width[hdrtoolslib::Y_COMP], m_ppcFrameRec[0]->m_height[hdrtoolslib::Y_COMP], true,
      hdrtoolslib::CM_RGB, hdrtoolslib::CP_2020, hdrtoolslib::CF_444, hdrtoolslib::SR_UNKNOWN, 32, false,
      hdrtoolslib::TF_PQ, 0);   // YCbCr to RGB conversion

    m_ppcFrameOrg[4] = new hdrtoolslib::Frame(m_ppcFrameOrg[0]->m_width[hdrtoolslib::Y_COMP],
                                              m_ppcFrameOrg[0]->m_height[hdrtoolslib::Y_COMP], true,
                                              hdrtoolslib::CM_RGB, hdrtoolslib::CP_2020, hdrtoolslib::CF_444,
                                              hdrtoolslib::SR_UNKNOWN, 32, false, hdrtoolslib::TF_NULL, 0);
    m_ppcFrameRec[4] = new hdrtoolslib::Frame(
      m_ppcFrameRec[0]->m_width[hdrtoolslib::Y_COMP], m_ppcFrameRec[0]->m_height[hdrtoolslib::Y_COMP], true,
      hdrtoolslib::CM_RGB, hdrtoolslib::CP_2020, hdrtoolslib::CF_444, hdrtoolslib::SR_UNKNOWN, 32, false,
      hdrtoolslib::TF_NULL, 0);   // Inverse Transfer Function

    m_pcFrameFormat                   = new hdrtoolslib::FrameFormat();
    m_pcFrameFormat->m_isFloat        = true;
    m_pcFrameFormat->m_chromaFormat   = hdrtoolslib::CF_UNKNOWN;
    m_pcFrameFormat->m_colorSpace     = hdrtoolslib::CM_RGB;
    m_pcFrameFormat->m_colorPrimaries = hdrtoolslib::CP_2020;
    m_pcFrameFormat->m_sampleRange    = hdrtoolslib::SR_UNKNOWN;

    m_pcConvertFormat    = hdrtoolslib::ConvertColorFormat::create(width, height, chFmt, hdrtoolslib::CF_444,
                                                                   chromaUpFilter, chromaLocation, chromaLocation);
    m_pcConvertIQuantize = hdrtoolslib::Convert::create(&m_ppcFrameOrg[1]->m_format, &m_ppcFrameOrg[2]->m_format);
    m_pcColorTransform =
      hdrtoolslib::ColorTransform::create(m_ppcFrameOrg[2]->m_colorSpace, m_ppcFrameOrg[2]->m_colorPrimaries,
                                          m_ppcFrameOrg[3]->m_colorSpace, m_ppcFrameOrg[3]->m_colorPrimaries, true, 1);
    m_pcDistortionDeltaE =
      new hdrtoolslib::DistortionMetricDeltaE(m_pcFrameFormat, false, maxSampleValue, whitePointDeltaE, 1);
    m_pcTransferFct = hdrtoolslib::TransferFunction::create(hdrtoolslib::TF_PQ, true, (float)maxSampleValue, 0, 0.0,
                                                            1.0, enableTFunctionLUT);
  }
#endif
  m_useHashMeInCurrentIntraPeriod = m_encCfg->m_HashMECfgEnable;
  m_HashMEPOC                     = 0;
  m_HashMEPOCchecked              = false;
  m_HashMEPOC2                    = 0;
}

int EncGOP::xWriteOPI(AccessUnit &accessUnit, const OPI *opi)
{
  OutputNALUnit nalu(NAL_UNIT_OPI);
  m_HLSWriter->setBitstream(&nalu.m_bitstream);
  CHECK(nalu.m_temporalId, "The value of TemporalId of OPI NAL units shall be equal to 0");
  m_HLSWriter->codeOPI(opi);
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}

int EncGOP::xWriteVPS(AccessUnit &accessUnit, const VPS *vps)
{
  OutputNALUnit nalu(NAL_UNIT_VPS);
  m_HLSWriter->setBitstream(&nalu.m_bitstream);
  CHECK(nalu.m_temporalId, "The value of TemporalId of VPS NAL units shall be equal to 0");
  m_HLSWriter->codeVPS(vps);
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}

int EncGOP::xWriteDCI(AccessUnit &accessUnit, const DCI *dci)
{
  OutputNALUnit nalu(NAL_UNIT_DCI);
  m_HLSWriter->setBitstream(&nalu.m_bitstream);
  CHECK(nalu.m_temporalId, "The value of TemporalId of DCI NAL units shall be equal to 0");
  m_HLSWriter->codeDCI(dci);
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}

int EncGOP::xWriteSPS(AccessUnit &accessUnit, const SPS *sps, const int layerId)
{
  OutputNALUnit nalu(NAL_UNIT_SPS);
  m_HLSWriter->setBitstream(&nalu.m_bitstream);
  nalu.m_nuhLayerId = layerId;
  CHECK(nalu.m_temporalId, "The value of TemporalId of SPS NAL units shall be equal to 0");
  m_HLSWriter->codeSPS(sps);
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}

int EncGOP::xWritePPS(AccessUnit &accessUnit, const PPS *pps, const int layerId)
{
  OutputNALUnit nalu(NAL_UNIT_PPS);
  m_HLSWriter->setBitstream(&nalu.m_bitstream);
  nalu.m_nuhLayerId = layerId;
  nalu.m_temporalId = accessUnit.temporalId;
  CHECK(nalu.m_temporalId < accessUnit.temporalId,
        "TemporalId shall be greater than or equal to the TemporalId of the layer access unit containing the NAL unit");
  m_HLSWriter->codePPS(pps);
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}

int EncGOP::xWriteAPS(AccessUnit &accessUnit, APS *aps, const int layerId, const bool isPrefixNUT)
{
  OutputNALUnit nalu(isPrefixNUT ? NAL_UNIT_PREFIX_APS : NAL_UNIT_SUFFIX_APS);
  m_HLSWriter->setBitstream(&nalu.m_bitstream);
  nalu.m_nuhLayerId = layerId;
  nalu.m_temporalId = aps->m_temporalId;
  aps->m_layerId    = layerId;
  CHECK(nalu.m_temporalId < accessUnit.temporalId,
        "TemporalId shall be greater than or equal to the TemporalId of the layer access unit containing the NAL unit");

  m_HLSWriter->codeAPS(aps);
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}

int EncGOP::xWriteParameterSets(AccessUnit &accessUnit, Slice *slice, const bool bSeqFirst, const int layerIdx,
                                bool newPPS)
{
  int actualTotalBits = 0;

  if (bSeqFirst)
  {
    if (layerIdx == 0)
    {
      if (m_encCfg->m_OPIEnabled)
      {
        actualTotalBits += xWriteOPI(accessUnit, &m_encCfg->m_opi);
      }
      if (m_encCfg->m_DCIEnabled)
      {
        actualTotalBits += xWriteDCI(accessUnit, &m_encCfg->m_dci);
      }
      if (slice->m_sps->m_vpsId != 0)
      {
        actualTotalBits += xWriteVPS(accessUnit, m_pcEncLib->m_vps);
      }
    }
    if (m_pcEncLib->SPSNeedsWriting(
          slice->m_sps->m_spsId))   // Note this assumes that all changes to the SPS are made at the EncLib level prior
                                    // to picture creation (EncLib::xGetNewPicBuffer).
    {
      CHECK(!(bSeqFirst),
            "Unspecified error");   // Implementations that use more than 1 SPS need to be aware of activation issues.
      actualTotalBits += xWriteSPS(accessUnit, slice->m_sps, m_pcEncLib->m_layerId);
    }
  }

  if (newPPS)   // Note this assumes that all changes to the PPS are made at the EncLib level prior to picture creation
                // (EncLib::xGetNewPicBuffer).
  {
    if (m_encCfg->m_rprPopulatePPSatIntraFlag)
    {
      if (slice->isIntra())
      {
        actualTotalBits += xWritePPS(accessUnit, slice->m_pps, m_pcEncLib->m_layerId);
        for (int nr = 0; nr < NUM_RPR_PPS; nr++)
        {
          if (slice->m_pps->m_ppsId != RPR_PPS_ID[nr])
          {
            const PPS *pPPS = m_pcEncLib->getPPS(RPR_PPS_ID[nr]);
            actualTotalBits += xWritePPS(accessUnit, pPPS, m_pcEncLib->m_layerId);
          }
        }
      }
      else
      {
        bool isRprPPS = false;
        for (int nr = 0; nr < NUM_RPR_PPS; nr++)
        {
          if (slice->m_pps->m_ppsId == RPR_PPS_ID[nr])
          {
            isRprPPS = true;
          }
        }
        if (!isRprPPS)
        {
          const PPS *pPPS = m_pcEncLib->getPPS(0);
          actualTotalBits += xWritePPS(accessUnit, pPPS, m_pcEncLib->m_layerId);
        }
      }
    }
    else
    {
      actualTotalBits += xWritePPS(accessUnit, slice->m_pps, m_pcEncLib->m_layerId);
    }
  }

  return actualTotalBits;
}

int EncGOP::xWritePicHeader(AccessUnit &accessUnit, PicHeader *picHeader)
{
  OutputNALUnit nalu(NAL_UNIT_PH);
  m_HLSWriter->setBitstream(&nalu.m_bitstream);
  nalu.m_temporalId = accessUnit.temporalId;
  nalu.m_nuhLayerId = m_pcEncLib->m_layerId;
  m_HLSWriter->codePictureHeader(picHeader, true);
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}

void EncGOP::xWriteAccessUnitDelimiter(AccessUnit &accessUnit, Slice *slice)
{
  AUDWriter     audWriter;
  OutputNALUnit nalu(NAL_UNIT_ACCESS_UNIT_DELIMITER);
  nalu.m_temporalId = slice->m_uiTLayer;
  const int vpsId   = slice->m_sps->m_vpsId;
  if (vpsId == 0)
  {
    nalu.m_nuhLayerId = 0;
  }
  else
  {
    nalu.m_nuhLayerId = slice->m_vps->m_vpsLayerId[0];
  }
  CHECK(nalu.m_temporalId != accessUnit.temporalId,
        "TemporalId shall be equal to the TemporalId of the AU containing the NAL unit");
  const int picType = slice->isIntra() ? 0 : (slice->isInterP() ? 1 : 2);
  audWriter.codeAUD(nalu.m_bitstream, m_audIrapOrGdrAuFlag, picType);
  accessUnit.push_front(new NALUnitEBSP(nalu));
}

void EncGOP::xWriteFillerData(AccessUnit &accessUnit, Slice *slice, uint32_t &fdSize)
{
  FDWriter      fdWriter;
  OutputNALUnit nalu(NAL_UNIT_FD);
  nalu.m_temporalId = slice->m_uiTLayer;
  const int vpsId   = slice->m_sps->m_vpsId;
  if (vpsId == 0)
  {
    nalu.m_nuhLayerId = 0;
  }
  else
  {
    nalu.m_nuhLayerId = slice->m_vps->m_vpsLayerId[0];
  }
  CHECK(nalu.m_temporalId != accessUnit.temporalId,
        "TemporalId shall be equal to the TemporalId of the AU containing the NAL unit");
  fdWriter.codeFD(nalu.m_bitstream, fdSize);
  accessUnit.push_back(new NALUnitEBSP(nalu));
}

// write SEI list into one NAL unit and add it to the Access unit at auPos
void EncGOP::xWriteSEI(NalUnitType naluType, SEIMessages &seiMessages, AccessUnit &accessUnit,
                       AccessUnit::iterator &auPos, int temporalId)
{
  // don't do anything, if we get an empty list
  if (seiMessages.empty())
  {
    return;
  }
  OutputNALUnit nalu(naluType, m_pcEncLib->m_layerId, temporalId);
  m_seiWriter.writeSEImessages(nalu.m_bitstream, seiMessages, *m_HRD, false, temporalId);
  auPos = accessUnit.insert(auPos, new NALUnitEBSP(nalu));
  auPos++;
}

uint32_t EncGOP::xWriteSEISeparately(NalUnitType naluType, SEIMessages &seiMessages, AccessUnit &accessUnit,
                                     AccessUnit::iterator &auPos, int temporalId)
{
  // don't do anything, if we get an empty list
  if (seiMessages.empty())
  {
    return 0;
  }

  uint32_t numBits = 0;

  for (SEIMessages::const_iterator sei = seiMessages.begin(); sei != seiMessages.end(); sei++)
  {
    SEIMessages tmpMessages;
    tmpMessages.push_back(*sei);
    OutputNALUnit nalu(naluType, m_pcEncLib->m_layerId, temporalId);
    numBits += m_seiWriter.writeSEImessages(nalu.m_bitstream, tmpMessages, *m_HRD, false, temporalId);
    auPos = accessUnit.insert(auPos, new NALUnitEBSP(nalu));
    auPos++;
  }

  return numBits;
}

void EncGOP::xClearSEIs(SEIMessages &seiMessages, bool deleteMessages)
{
  if (deleteMessages)
  {
    deleteSEIs(seiMessages);
  }
  else
  {
    seiMessages.clear();
  }
}

// write SEI messages as separate NAL units ordered
uint32_t EncGOP::xWriteLeadingSEIOrdered(SEIMessages &seiMessages, SEIMessages &duInfoSeiMessages,
                                         AccessUnit &accessUnit, int temporalId, bool testWrite)
{
  AccessUnit::iterator itNalu = accessUnit.begin();

  while ((itNalu != accessUnit.end()) &&
         ((*itNalu)->m_nalUnitType == NAL_UNIT_ACCESS_UNIT_DELIMITER || (*itNalu)->m_nalUnitType == NAL_UNIT_OPI ||
          (*itNalu)->m_nalUnitType == NAL_UNIT_VPS || (*itNalu)->m_nalUnitType == NAL_UNIT_DCI ||
          (*itNalu)->m_nalUnitType == NAL_UNIT_SPS || (*itNalu)->m_nalUnitType == NAL_UNIT_PPS))
  {
    itNalu++;
  }

  SEIMessages localMessages = seiMessages;
  SEIMessages currentMessages;

#if ENABLE_TRACING
  g_HLSTraceEnable = !testWrite;
#endif
  // The case that a specific SEI is not present is handled in xWriteSEI (empty list)

  // When SEI Manifest SEI message is present in an SEI NAL unit, the SEI Manifest SEI message shall be the first SEI
  // message in the SEI NAL unit (D3.45 in ISO/IEC 23008-2).
  if (m_encCfg->m_seiCfg.m_SEIManifestSEIEnabled)
  {
    currentMessages = extractSeisByType(localMessages, SEI::PayloadType::SEI_MANIFEST);
    CHECK(!(currentMessages.size() <= 1), "Unspecified error");
    xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId);
    xClearSEIs(currentMessages, !testWrite);
  }
  if (m_encCfg->m_seiCfg.m_SEIPrefixIndicationSEIEnabled)
  {
    // There may be multiple SEI prefix indication messages at the same time
    currentMessages = extractSeisByType(localMessages, SEI::PayloadType::SEI_PREFIX_INDICATION);
    xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId);
    xClearSEIs(currentMessages, !testWrite);
  }

  // Buffering period SEI must always be following active parameter sets
  currentMessages = extractSeisByType(localMessages, SEI::PayloadType::BUFFERING_PERIOD);
  CHECK(!(currentMessages.size() <= 1), "Unspecified error");
  xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId);
  xClearSEIs(currentMessages, !testWrite);

  // Picture timing SEI must always be following buffering period
  // Note: When general_same_pic_timing_in_all_ols_flag is equal to 1, PT SEI messages are required
  //       to be placed into separate NAL units. The code below conforms to the constraint even if
  //       general_same_pic_timing_in_all_ols_flag is equal to 0
  currentMessages = extractSeisByType(localMessages, SEI::PayloadType::PICTURE_TIMING);
  CHECK(!(currentMessages.size() <= 1), "Unspecified error");
  xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId);
  xClearSEIs(currentMessages, !testWrite);

  // Decoding unit info SEI must always be following picture timing
  if (!duInfoSeiMessages.empty())
  {
    currentMessages.push_back(duInfoSeiMessages.front());
    if (!testWrite)
    {
      duInfoSeiMessages.pop_front();
    }
    xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId);
    xClearSEIs(currentMessages, !testWrite);
  }

  if (m_encCfg->m_seiCfg.m_scalableNestingSEIEnabled)
  {
    // Scalable nesting SEI must always be the following DU info
    currentMessages = extractSeisByType(localMessages, SEI::PayloadType::SCALABLE_NESTING);
    xWriteSEISeparately(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId);
    xClearSEIs(currentMessages, !testWrite);
  }

  // And finally everything else one by one
  uint32_t numBits = xWriteSEISeparately(NAL_UNIT_PREFIX_SEI, localMessages, accessUnit, itNalu, temporalId);
  xClearSEIs(localMessages, !testWrite);

  if (!testWrite)
  {
    seiMessages.clear();
  }

  return numBits;
}

uint32_t EncGOP::xWriteLeadingSEIMessages(SEIMessages &seiMessages, SEIMessages &duInfoSeiMessages,
                                          AccessUnit &accessUnit, int temporalId, const SPS *sps,
                                          std::deque<DUData> &duData)
{
  AccessUnit  testAU;
  SEIMessages picTimingSEIs = getSeisByType(seiMessages, SEI::PayloadType::PICTURE_TIMING);
  CHECK(!(picTimingSEIs.size() < 2), "Unspecified error");
  SEIPictureTiming *picTiming = picTimingSEIs.empty() ? nullptr : (SEIPictureTiming *)picTimingSEIs.front();

  // test writing
  xWriteLeadingSEIOrdered(seiMessages, duInfoSeiMessages, testAU, temporalId, true);
  // update Timing and DU info SEI
  xUpdateDuData(testAU, duData);
  xUpdateTimingSEI(picTiming, duData, sps);
  xUpdateDuInfoSEI(duInfoSeiMessages, picTiming, sps->m_maxSubLayers);
  // actual writing
  return xWriteLeadingSEIOrdered(seiMessages, duInfoSeiMessages, accessUnit, temporalId, false);

  // testAU will automatically be cleaned up when losing scope
}

void EncGOP::xWriteTrailingSEIMessages(SEIMessages &seiMessages, AccessUnit &accessUnit, int temporalId)
{
  // Note: using accessUnit.end() works only as long as this function is called after slice coding and before EOS/EOB
  // NAL units
  AccessUnit::iterator pos = accessUnit.end();
  xWriteSEISeparately(NAL_UNIT_SUFFIX_SEI, seiMessages, accessUnit, pos, temporalId);
  deleteSEIs(seiMessages);
}

void EncGOP::xWriteDuSEIMessages(SEIMessages &duInfoSeiMessages, AccessUnit &accessUnit, int temporalId,
                                 std::deque<DUData> &duData)
{
  if (m_encCfg->m_seiCfg.m_decodingUnitInfoSEIEnabled &&
      m_HRD->getBufferingPeriodSEI()->m_decodingUnitCpbParamsInPicTimingSeiFlag)
  {
    int                  naluIdx = 0;
    AccessUnit::iterator nalu    = accessUnit.begin();

    // skip over first DU, we have a DU info SEI there already
    while (naluIdx < duData[0].accumNalsDU && nalu != accessUnit.end())
    {
      naluIdx++;
      nalu++;
    }

    SEIMessages::iterator duSEI = duInfoSeiMessages.begin();
    // loop over remaining DUs
    for (int duIdx = 1; duIdx < duData.size(); duIdx++)
    {
      CHECK(duSEI == duInfoSeiMessages.end(), "Number of generated SEIs should match number of DUs");

      // write the next SEI
      SEIMessages tmpSEI;
      tmpSEI.push_back(*duSEI);
      xWriteSEI(NAL_UNIT_PREFIX_SEI, tmpSEI, accessUnit, nalu, temporalId);
      // nalu points to the position after the SEI, so we have to increase the index as well
      naluIdx++;
      while ((naluIdx < duData[duIdx].accumNalsDU) && nalu != accessUnit.end())
      {
        naluIdx++;
        nalu++;
      }
      duSEI++;
    }
  }
  deleteSEIs(duInfoSeiMessages);
}

void EncGOP::xCreateIRAPLeadingSEIMessages(SEIMessages &seiMessages, const SPS *sps, const PPS *pps)
{
  OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);

  if (m_encCfg->m_seiCfg.m_framePackingSEIEnabled)
  {
    SEIFramePacking *sei = new SEIFramePacking;
    m_seiEncoder.initSEIFramePacking(sei, m_numPicsCoded);
    seiMessages.push_back(sei);
  }

  if (m_encCfg->m_seiCfg.m_parameterSetsInclusionIndicationSEIEnabled)
  {
    SEIParameterSetsInclusionIndication *sei = new SEIParameterSetsInclusionIndication;
    m_seiEncoder.initSEIParameterSetsInclusionIndication(sei);
    seiMessages.push_back(sei);
  }

  if (m_encCfg->m_seiCfg.m_alternativeTransferCharacteristicsSEIEnabled)
  {
    SEIAlternativeTransferCharacteristics *seiAlternativeTransferCharacteristics =
      new SEIAlternativeTransferCharacteristics;
    m_seiEncoder.initSEIAlternativeTransferCharacteristics(seiAlternativeTransferCharacteristics);
    seiMessages.push_back(seiAlternativeTransferCharacteristics);
  }
  if (m_encCfg->m_seiCfg.m_erpSEIEnabled)
  {
    SEIEquirectangularProjection *sei = new SEIEquirectangularProjection;
    m_seiEncoder.initSEIErp(sei);
    seiMessages.push_back(sei);
  }

  if (m_encCfg->m_seiCfg.m_sphereRotationSEIEnabled)
  {
    SEISphereRotation *sei = new SEISphereRotation;
    m_seiEncoder.initSEISphereRotation(sei);
    seiMessages.push_back(sei);
  }

  if (m_encCfg->m_seiCfg.m_omniViewportSEIEnabled)
  {
    SEIOmniViewport *sei = new SEIOmniViewport;
    m_seiEncoder.initSEIOmniViewport(sei);
    seiMessages.push_back(sei);
  }
  if (m_encCfg->m_seiCfg.m_rwpSEIEnabled)
  {
    SEIRegionWisePacking *seiRegionWisePacking = new SEIRegionWisePacking;
    m_seiEncoder.initSEIRegionWisePacking(seiRegionWisePacking);
    seiMessages.push_back(seiRegionWisePacking);
  }
  if (m_encCfg->m_seiCfg.m_gcmpSEIEnabled)
  {
    SEIGeneralizedCubemapProjection *sei = new SEIGeneralizedCubemapProjection;
    m_seiEncoder.initSEIGcmp(sei);
    seiMessages.push_back(sei);
  }
  if (m_encCfg->m_seiCfg.m_cfgSubpictureLevelInfoSEI.m_enabled)
  {
    SEISubpicureLevelInfo *seiSubpicureLevelInfo = new SEISubpicureLevelInfo;
    m_seiEncoder.initSEISubpictureLevelInfo(seiSubpicureLevelInfo, sps);
    seiMessages.push_back(seiSubpicureLevelInfo);
  }
  if (m_encCfg->m_seiCfg.m_sampleAspectRatioInfoSEIEnabled)
  {
    SEISampleAspectRatioInfo *seiSampleAspectRatioInfo = new SEISampleAspectRatioInfo;
    m_seiEncoder.initSEISampleAspectRatioInfo(seiSampleAspectRatioInfo);
    seiMessages.push_back(seiSampleAspectRatioInfo);
  }
  // film grain
  if (m_encCfg->m_seiCfg.m_fgcSEIEnabled && !m_encCfg->m_seiCfg.m_fgcSEIPerPictureSEI)
  {
    SEIFilmGrainCharacteristics *sei = new SEIFilmGrainCharacteristics;
    m_seiEncoder.initSEIFilmGrainCharacteristics(sei);
    if (m_encCfg->m_seiCfg.m_fgcSEIAnalysisEnabled)
    {
      sei->m_log2ScaleFactor = m_fgAnalyzer.getLog2scaleFactor();
      for (int compIdx = 0; compIdx < getNumberValidComponents(m_encCfg->m_chromaFormatIdc); compIdx++)
      {
        if (sei->m_compModel[compIdx].presentFlag)
        {   // higher importance of presentFlag is from cfg file
          sei->m_compModel[compIdx] = m_fgAnalyzer.getCompModel(compIdx);
        }
      }
    }
    seiMessages.push_back(sei);
  }

  // mastering display colour volume
  if (m_encCfg->m_seiCfg.m_masteringDisplay.colourVolumeSEIEnabled)
  {
    SEIMasteringDisplayColourVolume *sei = new SEIMasteringDisplayColourVolume;
    m_seiEncoder.initSEIMasteringDisplayColourVolume(sei);
    seiMessages.push_back(sei);
  }

  // content light level
  if (m_encCfg->m_seiCfg.m_cllSEIEnabled)
  {
    SEIContentLightLevelInfo *seiCLL = new SEIContentLightLevelInfo;
    m_seiEncoder.initSEIContentLightLevel(seiCLL);
    seiMessages.push_back(seiCLL);
  }

  // ambient viewing environment
  if (m_encCfg->m_seiCfg.m_aveSEIEnabled)
  {
    SEIAmbientViewingEnvironment *seiAVE = new SEIAmbientViewingEnvironment;
    m_seiEncoder.initSEIAmbientViewingEnvironment(seiAVE);
    seiMessages.push_back(seiAVE);
  }

  // content colour volume
  if (m_encCfg->m_seiCfg.m_ccvSEIEnabled)
  {
    SEIContentColourVolume *seiContentColourVolume = new SEIContentColourVolume;
    m_seiEncoder.initSEIContentColourVolume(seiContentColourVolume);
    seiMessages.push_back(seiContentColourVolume);
  }

  if (m_encCfg->m_seiCfg.m_sdiSEIEnabled)
  {
    SEIScalabilityDimensionInfo *seiScalabilityDimensionInfo = new SEIScalabilityDimensionInfo;
    m_seiEncoder.initSEIScalabilityDimensionInfo(seiScalabilityDimensionInfo);
    seiMessages.push_back(seiScalabilityDimensionInfo);
  }
  // multiview acquisition information
  if (m_encCfg->m_seiCfg.m_maiSEIEnabled)
  {
    SEIMultiviewAcquisitionInfo *seiMultiviewAcquisitionInfo = new SEIMultiviewAcquisitionInfo;
    m_seiEncoder.initSEIMultiviewAcquisitionInfo(seiMultiviewAcquisitionInfo);
    seiMessages.push_back(seiMultiviewAcquisitionInfo);
  }
  // multiview view position
  if (m_encCfg->m_seiCfg.m_mvpSEIEnabled)
  {
    SEIMultiviewViewPosition *seiMultiviewViewPosition = new SEIMultiviewViewPosition;
    m_seiEncoder.initSEIMultiviewViewPosition(seiMultiviewViewPosition);
    seiMessages.push_back(seiMultiviewViewPosition);
  }
  // alpha channel information
  if (m_encCfg->m_seiCfg.m_aciSEIEnabled)
  {
    SEIAlphaChannelInfo *seiAlphaChannelInfo = new SEIAlphaChannelInfo;
    m_seiEncoder.initSEIAlphaChannelInfo(seiAlphaChannelInfo);
    seiMessages.push_back(seiAlphaChannelInfo);
  }
  // depth representation information
  if (m_encCfg->m_seiCfg.m_driSEIEnabled)
  {
    SEIDepthRepresentationInfo *seiDepthRepresentationInfo = new SEIDepthRepresentationInfo;
    m_seiEncoder.initSEIDepthRepresentationInfo(seiDepthRepresentationInfo);
    seiMessages.push_back(seiDepthRepresentationInfo);
  }
  // colour transform information
  if (m_encCfg->m_seiCfg.m_ctiSEIEnabled)
  {
    SEIColourTransformInfo *seiCTI = new SEIColourTransformInfo;
    m_seiEncoder.initSEIColourTransformInfo(seiCTI);
    seiMessages.push_back(seiCTI);
  }

  // Make sure that sei_manifest and sei_prefix are the last two initialized sei_msg, otherwise it will cause these two
  // Sei messages to not be able to enter all SEI messages
  if (m_encCfg->m_seiCfg.m_SEIManifestSEIEnabled)
  {
    SEIManifest *seiSEIManifest = new SEIManifest;
    m_seiEncoder.initSEISEIManifest(seiSEIManifest, seiMessages);
    seiMessages.push_back(seiSEIManifest);
  }
  if (m_encCfg->m_seiCfg.m_SEIPrefixIndicationSEIEnabled)
  {
    int numSeiPrefixMsg = 0;
    for (auto &it: seiMessages)
    {
      if (it->payloadType() == SEI::PayloadType::SEI_MANIFEST)
      {
        break;
      }
      numSeiPrefixMsg++;
    }
    for (auto &it: seiMessages)
    {
      if (numSeiPrefixMsg == 0 || it->payloadType() == SEI::PayloadType::SEI_MANIFEST)
      {
        break;
      }
      SEIPrefixIndication *seiSEIPrefixIndication = new SEIPrefixIndication;
      m_seiEncoder.initSEISEIPrefixIndication(seiSEIPrefixIndication, it);
      seiMessages.push_back(seiSEIPrefixIndication);
      numSeiPrefixMsg--;
    }
  }

  if (m_encCfg->m_constrainedRaslEncoding)
  {
    SEIConstrainedRaslIndication *seiConstrainedRasl = new SEIConstrainedRaslIndication;
    seiMessages.push_back(seiConstrainedRasl);
  }
  if (m_encCfg->m_seiCfg.m_siiSEIEnabled)
  {
    SEIShutterIntervalInfo *seiShutterInterval = new SEIShutterIntervalInfo;
    m_seiEncoder.initSEIShutterIntervalInfo(seiShutterInterval);
    seiMessages.push_back(seiShutterInterval);
  }
  if (m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsEnabled)
  {
    for (int i = 0; i < m_encCfg->m_seiCfg.m_nnPostFilterSEICharacteristicsNumFilters; i++)
    {
      SEINeuralNetworkPostFilterCharacteristics *seiNNPostFilterCharacteristics =
        new SEINeuralNetworkPostFilterCharacteristics;
      m_seiEncoder.initSEINeuralNetworkPostFilterCharacteristics(seiNNPostFilterCharacteristics, i);
      seiMessages.push_back(seiNNPostFilterCharacteristics);
    }
  }
  if (m_encCfg->m_seiCfg.m_poSEIEnabled)
  {
    SEIProcessingOrderInfo *seiProcessingOrder = new SEIProcessingOrderInfo;
    m_seiEncoder.initSEIProcessingOrderInfo(seiProcessingOrder);
    seiMessages.push_back(seiProcessingOrder);
  }
}

void EncGOP::xCreatePerPictureSEIMessages(int picInGOP, SEIMessages &seiMessages, SEIMessages &nestedSeiMessages,
                                          Slice *slice)
{
  if ((m_encCfg->m_seiCfg.m_bufferingPeriodSEIEnabled) &&
      (slice->isIRAP() || slice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_GDR) &&
      slice->m_nuhLayerId == slice->m_vps->m_vpsLayerId[0] && (slice->m_sps->m_generalHrdParametersPresentFlag))
  {
    SEIBufferingPeriod *bufferingPeriodSEI = new SEIBufferingPeriod();
    const bool          noLeadingPictures =
      slice->m_eNalUnitType != NAL_UNIT_CODED_SLICE_IDR_W_RADL && slice->m_eNalUnitType != NAL_UNIT_CODED_SLICE_CRA;
    m_seiEncoder.initSEIBufferingPeriod(bufferingPeriodSEI, noLeadingPictures);
    m_HRD->setBufferingPeriodSEI(bufferingPeriodSEI);
    seiMessages.push_back(bufferingPeriodSEI);
    m_bufferingPeriodSEIPresentInAU = true;

    if (m_encCfg->m_seiCfg.m_scalableNestingSEIEnabled)
    {
      SEIBufferingPeriod *bufferingPeriodSEIcopy = new SEIBufferingPeriod();
      bufferingPeriodSEI->copyTo(*bufferingPeriodSEIcopy);
      nestedSeiMessages.push_back(bufferingPeriodSEIcopy);
    }
  }

  if (m_encCfg->m_seiCfg.m_dependentRAPIndicationSEIEnabled && slice->m_isDRAP)
  {
    SEIDependentRAPIndication *dependentRAPIndicationSEI = new SEIDependentRAPIndication();
    m_seiEncoder.initSEIDependentRAPIndication(dependentRAPIndicationSEI);
    seiMessages.push_back(dependentRAPIndicationSEI);
  }

  if (m_encCfg->m_seiCfg.m_edrapIndicationSEIEnabled && slice->m_edrapRapId > 0)
  {
    SEIExtendedDrapIndication *seiExtendedDrapIndication = new SEIExtendedDrapIndication();
    m_seiEncoder.initSEIExtendedDrapIndication(seiExtendedDrapIndication);
    // update EDRAP SEI message according to the reference lists of the slice
    seiExtendedDrapIndication->m_edrapIndicationRapIdMinus1 = slice->m_edrapRapId - 1;
    seiExtendedDrapIndication->m_edrapIndicationLeadingPicturesDecodableFlag =
      slice->m_latestEdrapLeadingPicDecodableFlag;
    seiExtendedDrapIndication->m_edrapIndicationNumRefRapPicsMinus1 = slice->m_edrapNumRefRapPics - 1;
    seiExtendedDrapIndication->m_edrapIndicationRefRapId.resize(
      seiExtendedDrapIndication->m_edrapIndicationNumRefRapPicsMinus1 + 1);
    for (int i = 0; i <= seiExtendedDrapIndication->m_edrapIndicationNumRefRapPicsMinus1; i++)
    {
      seiExtendedDrapIndication->m_edrapIndicationRefRapId[i] = slice->getEdrapRefRapId(i);
    }
    seiMessages.push_back(seiExtendedDrapIndication);
  }

  // insert one Annotated Region SEI for the picture (if the file exists)
  if (!m_encCfg->m_seiCfg.m_arSEIFileRoot.empty())
  {
    SEIAnnotatedRegions *seiAnnotatedRegions = new SEIAnnotatedRegions();
    const bool           success             = m_seiEncoder.initSEIAnnotatedRegions(seiAnnotatedRegions, slice->m_poc);

    if (success)
    {
      seiMessages.push_back(seiAnnotatedRegions);
    }
    else
    {
      delete seiAnnotatedRegions;
    }
  }

  if (m_encCfg->m_seiCfg.m_fgcSEIEnabled && m_encCfg->m_seiCfg.m_fgcSEIPerPictureSEI)
  {
    SEIFilmGrainCharacteristics *fgcSEI = new SEIFilmGrainCharacteristics;
    m_seiEncoder.initSEIFilmGrainCharacteristics(fgcSEI);
    if (m_encCfg->m_seiCfg.m_fgcSEIAnalysisEnabled)
    {
      fgcSEI->m_log2ScaleFactor = m_fgAnalyzer.getLog2scaleFactor();
      for (int compIdx = 0; compIdx < getNumberValidComponents(m_encCfg->m_chromaFormatIdc); compIdx++)
      {
        if (fgcSEI->m_compModel[compIdx].presentFlag)
        {   // higher importance of presentFlag is from cfg file
          fgcSEI->m_compModel[compIdx] = m_fgAnalyzer.getCompModel(compIdx);
        }
      }
    }
    seiMessages.push_back(fgcSEI);
  }

  if (m_encCfg->m_seiCfg.m_nnPostFilterSEIActivationEnabled)
  {
    SEINeuralNetworkPostFilterActivation *nnpfActivationSEI = new SEINeuralNetworkPostFilterActivation;
    m_seiEncoder.initSEINeuralNetworkPostFilterActivation(nnpfActivationSEI);
    seiMessages.push_back(nnpfActivationSEI);
  }

  if (m_encCfg->m_seiCfg.m_postFilterHintSEIEnabled)
  {
    SEIPostFilterHint *postFilterHintSEI = new SEIPostFilterHint;

    m_seiEncoder.initSEIPostFilterHint(postFilterHintSEI);
    seiMessages.push_back(postFilterHintSEI);
  }
}

void EncGOP::xCreatePhaseIndicationSEIMessages(SEIMessages &seiMessages, Slice *slice, int ppsId)
{
  if (m_encCfg->m_seiCfg.m_phaseIndicationSEIEnabledFullResolution && ppsId == 0)
  {
    SEIPhaseIndication *seiPhaseIndication = new SEIPhaseIndication;
    m_seiEncoder.initSEIPhaseIndication(seiPhaseIndication, ppsId);
    seiMessages.push_back(seiPhaseIndication);
  }
  else if (m_encCfg->m_seiCfg.m_phaseIndicationSEIEnabledReducedResolution && ppsId == ENC_PPS_ID_RPR)
  {
    SEIPhaseIndication *seiPhaseIndication = new SEIPhaseIndication;
    m_seiEncoder.initSEIPhaseIndication(seiPhaseIndication, ppsId);
    seiMessages.push_back(seiPhaseIndication);
  }
}

void EncGOP::xCreateScalableNestingSEI(SEIMessages &seiMessages, SEIMessages &nestedSeiMessages,
                                       const std::vector<int> &targetOLSs, const std::vector<int> &targetLayers,
                                       const std::vector<uint16_t> &subpicIDs, uint16_t maxSubpicIdInPic)
{
  SEIMessages tmpMessages;
  while (!nestedSeiMessages.empty())
  {
    SEI *sei = nestedSeiMessages.front();
    nestedSeiMessages.pop_front();
    tmpMessages.push_back(sei);
    SEIScalableNesting *nestingSEI = new SEIScalableNesting();
    m_seiEncoder.initSEIScalableNesting(nestingSEI, tmpMessages, targetOLSs, targetLayers, subpicIDs, maxSubpicIdInPic);
    seiMessages.push_back(nestingSEI);
    tmpMessages.clear();
  }
}

void EncGOP::xCreateFrameFieldInfoSEI(SEIMessages &seiMessages, Slice *slice, bool isField)
{
  if (m_encCfg->m_seiCfg.m_frameFieldInfoSEIEnabled)
  {
    SEIFrameFieldInfo *frameFieldInfoSEI = new SEIFrameFieldInfo();

    // encode only very basic information. if more feature are supported, this should be moved to SEIEncoder
    frameFieldInfoSEI->m_fieldPicFlag = isField;
    if (isField)
    {
      frameFieldInfoSEI->m_bottomFieldFlag = !slice->m_pic->m_topField;
    }
    seiMessages.push_back(frameFieldInfoSEI);
  }
}

void EncGOP::xCreatePictureTimingSEI(int irapGopId, SEIMessages &seiMessages, SEIMessages &nestedSeiMessages,
                                     SEIMessages &duInfoSeiMessages, Slice *slice, bool isField,
                                     std::deque<DUData> &duData)
{
  // Picture timing depends on buffering period. When either of those is not disabled,
  // initialization would fail. Needs more cleanup after DU timing is integrated.
  if (!(m_encCfg->m_seiCfg.m_pictureTimingSEIEnabled && m_encCfg->m_seiCfg.m_bufferingPeriodSEIEnabled))
  {
    return;
  }

  const GeneralHrdParams *hrd = &slice->m_sps->m_generalHrdParams;

  // update decoding unit parameters
  if ((m_encCfg->m_seiCfg.m_pictureTimingSEIEnabled || m_encCfg->m_seiCfg.m_decodingUnitInfoSEIEnabled) &&
      slice->m_nuhLayerId == slice->m_vps->m_vpsLayerId[0])
  {
    SEIPictureTiming *pictureTimingSEI = new SEIPictureTiming();

    // DU parameters
    if (hrd->m_generalDecodingUnitHrdParamsPresentFlag)
    {
      const uint32_t numDU                            = (uint32_t)duData.size();
      pictureTimingSEI->m_numDecodingUnitsMinus1      = (numDU - 1);
      pictureTimingSEI->m_duCommonCpbRemovalDelayFlag = false;
      pictureTimingSEI->m_numNalusInDuMinus1.resize(numDU);
      const uint32_t maxNumSubLayers = slice->m_sps->m_maxSubLayers;
      pictureTimingSEI->m_duCpbRemovalDelayMinus1.resize(numDU * maxNumSubLayers);
    }
    const uint32_t cpbRemovalDelayLegth = m_HRD->getBufferingPeriodSEI()->m_cpbRemovalDelayLength;
    const uint32_t maxNumSubLayers      = slice->m_sps->m_maxSubLayers;
    pictureTimingSEI->m_auCpbRemovalDelay[maxNumSubLayers - 1] = std::min<int>(
      std::max<int>(1, m_totalCoded[maxNumSubLayers - 1] - m_lastBPSEI[maxNumSubLayers - 1]),
      static_cast<int>(
        pow(2, static_cast<double>(cpbRemovalDelayLegth))));   // Syntax element signalled as minus, hence the .
    CHECK((m_totalCoded[maxNumSubLayers - 1] - m_lastBPSEI[maxNumSubLayers - 1]) >
            pow(2, static_cast<double>(cpbRemovalDelayLegth)),
          " cpbRemovalDelayLegth too small for m_auCpbRemovalDelay[pt_max_sub_layers_minus1] at picture timing SEI ");
    const uint32_t temporalId = slice->m_uiTLayer;
    if (maxNumSubLayers == 1)
    {
      pictureTimingSEI->m_ptSubLayerDelaysPresentFlag[0] = true;
    }
    for (int i = temporalId; i < maxNumSubLayers - 1; i++)
    {
      int indexWithinGOP = (m_totalCoded[maxNumSubLayers - 1] - m_lastBPSEI[maxNumSubLayers - 1]) % m_encCfg->m_gopSize;
      pictureTimingSEI->m_ptSubLayerDelaysPresentFlag[i] = true;
      if (((m_rapWithLeading == true) && (indexWithinGOP == 0)) || (m_totalCoded[maxNumSubLayers - 1] == 0) ||
          m_bufferingPeriodSEIPresentInAU || (slice->m_poc + m_encCfg->m_gopSize) > m_encCfg->m_framesToBeEncoded)
      {
        pictureTimingSEI->m_cpbRemovalDelayDeltaEnabledFlag[i] = false;
      }
      else
      {
        pictureTimingSEI->m_cpbRemovalDelayDeltaEnabledFlag[i] =
          m_HRD->getBufferingPeriodSEI()->m_cpbRemovalDelayDeltasPresentFlag;
      }
      if (pictureTimingSEI->m_cpbRemovalDelayDeltaEnabledFlag[i])
      {
        if (m_rapWithLeading == false)
        {
          switch (m_encCfg->m_gopSize)
          {
          case 8:
            {
              if ((indexWithinGOP == 1 && i == 2))
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 0;
              }
              else if ((indexWithinGOP == 2 && i == 2) || (indexWithinGOP == 6 && i == 2))
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 1;
              }
              else if ((indexWithinGOP == 1 && i == 1) || (indexWithinGOP == 3 && i == 2))
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 2;
              }
              else if (indexWithinGOP == 2 && i == 1)
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 3;
              }
              else if (indexWithinGOP == 1 && i == 0)
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 4;
              }
              else
              {
                THROW("m_cpbRemovalDelayDeltaIdx not applicable for the sub-layer and GOP size");
              }
            }
            break;
          case 16:
            {
              if ((indexWithinGOP == 1 && i == 3))
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 0;
              }
              else if ((indexWithinGOP == 2 && i == 3) || (indexWithinGOP == 10 && i == 3) ||
                       (indexWithinGOP == 14 && i == 3))
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 1;
              }
              else if ((indexWithinGOP == 1 && i == 2) || (indexWithinGOP == 3 && i == 3) ||
                       (indexWithinGOP == 7 && i == 3) || (indexWithinGOP == 11 && i == 3))
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 2;
              }
              else if (indexWithinGOP == 4 && i == 3)
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 3;
              }
              else if ((indexWithinGOP == 2 && i == 2) || (indexWithinGOP == 10 && i == 2))
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 4;
              }
              else if (indexWithinGOP == 1 && i == 1)
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 5;
              }
              else if (indexWithinGOP == 3 && i == 2)
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 6;
              }
              else if (indexWithinGOP == 2 && i == 1)
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 7;
              }
              else if (indexWithinGOP == 1 && i == 0)
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 8;
              }
              else
              {
                THROW("m_cpbRemovalDelayDeltaIdx not applicable for the sub-layer and GOP size");
              }
            }
            break;
          default:
            {
              THROW("m_cpbRemovalDelayDeltaIdx not supported for the current GOP size");
            }
            break;
          }
        }
        else
        {
          switch (m_encCfg->m_gopSize)
          {
          case 8:
            {
              if ((indexWithinGOP == 1 && i == 2) || (indexWithinGOP == 5 && i == 2))
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 0;
              }
              else if (indexWithinGOP == 2 && i == 2)
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 1;
              }
              else if (indexWithinGOP == 1 && i == 1)
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 2;
              }
              else
              {
                THROW("m_cpbRemovalDelayDeltaIdx not applicable for the sub-layer and GOP size");
              }
            }
            break;
          case 16:
            {
              if ((indexWithinGOP == 1 && i == 3) || (indexWithinGOP == 9 && i == 3) ||
                  (indexWithinGOP == 13 && i == 3))
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 0;
              }
              else if ((indexWithinGOP == 2 && i == 3) || (indexWithinGOP == 6 && i == 3) ||
                       (indexWithinGOP == 10 && i == 3))
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 1;
              }
              else if ((indexWithinGOP == 1 && i == 2) || (indexWithinGOP == 9 && i == 2) ||
                       (indexWithinGOP == 3 && i == 3))
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 2;
              }
              else if (indexWithinGOP == 2 && i == 2)
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 3;
              }
              else if (indexWithinGOP == 1 && i == 1)
              {
                pictureTimingSEI->m_cpbRemovalDelayDeltaIdx[i] = 4;
              }
              else
              {
                THROW("m_cpbRemovalDelayDeltaIdx not applicable for the sub-layer and GOP size");
              }
            }
            break;
          default:
            {
              THROW("m_cpbRemovalDelayDeltaIdx not applicable for the sub-layer and GOP size");
            }
            break;
          }
        }
      }
      else
      {
        int scaledDistToBuffPeriod =
          (m_totalCoded[i] - m_lastBPSEI[i]) * static_cast<int>(pow(2, static_cast<double>(maxNumSubLayers - 1 - i)));
        pictureTimingSEI->m_auCpbRemovalDelay[i] = std::min<int>(
          std::max<int>(1, scaledDistToBuffPeriod),
          static_cast<int>(
            pow(2, static_cast<double>(cpbRemovalDelayLegth))));   // Syntax element signalled as minus, hence the .
        CHECK((scaledDistToBuffPeriod) > pow(2, static_cast<double>(cpbRemovalDelayLegth)),
              " cpbRemovalDelayLegth too small for m_auCpbRemovalDelay[i] at picture timing SEI ");
      }
    }
    pictureTimingSEI->m_picDpbOutputDelay = slice->m_sps->m_maxNumReorderPics[slice->m_sps->m_maxSubLayers - 1] +
      slice->m_poc - m_totalCoded[maxNumSubLayers - 1];
    if (m_encCfg->m_efficientFieldIRAPEnabled && irapGopId > 0 && irapGopId < m_iGopSize)
    {
      // if pictures have been swapped there is likely one more picture delay on their tid. Very rough approximation
      pictureTimingSEI->m_picDpbOutputDelay++;
    }
    int factor                              = hrd->m_tickDivisorMinus2 + 2;
    pictureTimingSEI->m_picDpbOutputDuDelay = factor * pictureTimingSEI->m_picDpbOutputDelay;
    if (m_bufferingPeriodSEIPresentInAU)
    {
      for (int i = temporalId; i < maxNumSubLayers; i++)
      {
        m_lastBPSEI[i] = m_totalCoded[i];
      }
      if ((slice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL) ||
          (slice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA))
      {
        m_rapWithLeading = true;
      }
    }

    if (m_encCfg->m_seiCfg.m_pictureTimingSEIEnabled)
    {
      seiMessages.push_back(pictureTimingSEI);

      if (m_encCfg->m_seiCfg.m_scalableNestingSEIEnabled && !m_encCfg->m_samePicTimingInAllOLS)
      {
        SEIPictureTiming *pictureTimingSEIcopy = new SEIPictureTiming();
        pictureTimingSEI->copyTo(*pictureTimingSEIcopy);
        nestedSeiMessages.push_back(pictureTimingSEIcopy);
      }
    }

    if (m_encCfg->m_seiCfg.m_decodingUnitInfoSEIEnabled && hrd->m_generalDecodingUnitHrdParamsPresentFlag)
    {
      for (int i = 0; i < (pictureTimingSEI->m_numDecodingUnitsMinus1 + 1); i++)
      {
        SEIDecodingUnitInfo *duInfoSEI = new SEIDecodingUnitInfo();
        duInfoSEI->m_decodingUnitIdx   = i;
        for (int j = temporalId; j <= maxNumSubLayers; j++)
        {
          duInfoSEI->m_duSptCpbRemovalDelayIncrement[j] =
            pictureTimingSEI->m_duCpbRemovalDelayMinus1[i * maxNumSubLayers + j] + 1;
        }
        duInfoSEI->m_dpbOutputDuDelayPresentFlag = false;

        duInfoSeiMessages.push_back(duInfoSEI);
      }
    }

    if (!m_encCfg->m_seiCfg.m_pictureTimingSEIEnabled && pictureTimingSEI)
    {
      delete pictureTimingSEI;
    }
  }
}

void EncGOP::xUpdateDuData(AccessUnit &testAU, std::deque<DUData> &duData)
{
  if (duData.empty())
  {
    return;
  }
  // fix first
  uint32_t numNalUnits  = (uint32_t)testAU.size();
  uint32_t numRBSPBytes = 0;
  for (AccessUnit::const_iterator it = testAU.begin(); it != testAU.end(); it++)
  {
    numRBSPBytes += uint32_t((*it)->m_nalUnitData.str().size());
  }
  duData[0].accumBitsDU += 8 * numRBSPBytes;
  duData[0].accumNalsDU += numNalUnits;

  // adapt cumulative sums for all following DUs
  // and add one DU info SEI, if enabled
  for (int i = 1; i < duData.size(); i++)
  {
    if (m_encCfg->m_seiCfg.m_decodingUnitInfoSEIEnabled)
    {
      numNalUnits += 1;
      numRBSPBytes += 8 * 5;
    }
    duData[i].accumBitsDU += numRBSPBytes;   // probably around 5 bytes
    duData[i].accumNalsDU += numNalUnits;
  }

  // The last DU may have a trailing SEI
  if (m_encCfg->m_seiCfg.m_decodedPictureHashSEIType != HashType::NONE)
  {
    duData.back().accumBitsDU += 8 * 20;   // probably around 20 bytes - should be further adjusted, e.g. by type
    duData.back().accumNalsDU += 1;
  }
}
void EncGOP::xUpdateTimingSEI(SEIPictureTiming *pictureTimingSEI, std::deque<DUData> &duData, const SPS *sps)
{
  if (!pictureTimingSEI)
  {
    return;
  }
  const GeneralHrdParams *hrd = &sps->m_generalHrdParams;
  if (hrd->m_generalDecodingUnitHrdParamsPresentFlag)
  {
    int                    i;
    uint64_t               ui64Tmp;
    uint32_t               uiPrev                   = 0;
    uint32_t               numDU                    = (pictureTimingSEI->m_numDecodingUnitsMinus1 + 1);
    std::vector<uint32_t> &rDuCpbRemovalDelayMinus1 = pictureTimingSEI->m_duCpbRemovalDelayMinus1;
    uint32_t               maxDiff                  = (hrd->m_tickDivisorMinus2 + 2) - 1;

    int maxNumSubLayers = sps->m_maxSubLayers;
    for (int j = 0; j < maxNumSubLayers - 1; j++)
    {
      pictureTimingSEI->m_ptSubLayerDelaysPresentFlag[j] = false;
    }

    for (i = 0; i < numDU; i++)
    {
      pictureTimingSEI->m_numNalusInDuMinus1[i] =
        (i == 0) ? (duData[i].accumNalsDU - 1) : (duData[i].accumNalsDU - duData[i - 1].accumNalsDU - 1);
    }

    if (numDU == 1)
    {
      rDuCpbRemovalDelayMinus1[0 + maxNumSubLayers - 1] = 0; /* don't care */
    }
    else
    {
      rDuCpbRemovalDelayMinus1[(numDU - 1) * maxNumSubLayers + maxNumSubLayers - 1] = 0; /* by definition */
      uint32_t tmp                                                                  = 0;
      uint32_t accum                                                                = 0;

      for (i = (numDU - 2); i >= 0; i--)
      {
        ui64Tmp = (((duData[numDU - 1].accumBitsDU - duData[i].accumBitsDU) *
                    (sps->m_generalHrdParams.m_timeScale / sps->m_generalHrdParams.m_numUnitsInTick) *
                    (hrd->m_tickDivisorMinus2 + 2)) /
                   (m_encCfg->m_RCTargetBitrate));
        if ((uint32_t)ui64Tmp > maxDiff)
        {
          tmp++;
        }
      }
      uiPrev = 0;

      uint32_t flag = 0;
      for (i = (numDU - 2); i >= 0; i--)
      {
        flag    = 0;
        ui64Tmp = (((duData[numDU - 1].accumBitsDU - duData[i].accumBitsDU) *
                    (sps->m_generalHrdParams.m_timeScale / sps->m_generalHrdParams.m_numUnitsInTick) *
                    (hrd->m_tickDivisorMinus2 + 2)) /
                   (m_encCfg->m_RCTargetBitrate));

        if ((uint32_t)ui64Tmp > maxDiff)
        {
          if (uiPrev >= maxDiff - tmp)
          {
            ui64Tmp = uiPrev + 1;
            flag    = 1;
          }
          else
          {
            ui64Tmp = maxDiff - tmp + 1;
          }
        }
        rDuCpbRemovalDelayMinus1[i * maxNumSubLayers + maxNumSubLayers - 1] = (uint32_t)ui64Tmp - uiPrev - 1;
        if ((int)rDuCpbRemovalDelayMinus1[i * maxNumSubLayers + maxNumSubLayers - 1] < 0)
        {
          rDuCpbRemovalDelayMinus1[i * maxNumSubLayers + maxNumSubLayers - 1] = 0;
        }
        else if (tmp > 0 && flag == 1)
        {
          tmp--;
        }
        accum += rDuCpbRemovalDelayMinus1[i * maxNumSubLayers + maxNumSubLayers - 1] + 1;
        uiPrev = accum;
      }
    }
  }
}

void EncGOP::xUpdateDuInfoSEI(SEIMessages &duInfoSeiMessages, SEIPictureTiming *pictureTimingSEI, int maxSubLayers)
{
  if (duInfoSeiMessages.empty() || (pictureTimingSEI == nullptr))
  {
    return;
  }

  int i = 0;

  for (SEIMessages::iterator du = duInfoSeiMessages.begin(); du != duInfoSeiMessages.end(); du++)
  {
    SEIDecodingUnitInfo *duInfoSEI = (SEIDecodingUnitInfo *)(*du);
    duInfoSEI->m_decodingUnitIdx   = i;
    for (int j = 0; j < maxSubLayers; j++)
    {
      duInfoSEI->m_duiSubLayerDelaysPresentFlag[j] = pictureTimingSEI->m_ptSubLayerDelaysPresentFlag[j];
      duInfoSEI->m_duSptCpbRemovalDelayIncrement[j] =
        pictureTimingSEI->m_duCpbRemovalDelayMinus1[i * maxSubLayers + j] + 1;
    }
    duInfoSEI->m_dpbOutputDuDelayPresentFlag = false;
    i++;
  }
}

static void validateMinCrRequirements(const ProfileTierLevelFeatures &plt, std::size_t numBytesInVclNalUnits,
                                      const Picture *pPic, const EncCfg *encCfg)
{
  //  numBytesInVclNalUnits shall be less than or equal to
  //     FormatCapabilityFactor * MaxLumaSr * framePeriod / MinCr,
  //     ( = FormatCapabilityFactor * MaxLumaSr / (MinCr * frameRate),
  if (plt.getTierLevelFeatures() && plt.getProfileFeatures() && plt.getTierLevelFeatures()->level != Level::LEVEL15_5)
  {
    const uint32_t formatCapabilityFactorx1000 = plt.getProfileFeatures()->formatCapabilityFactorx1000;
    const uint64_t maxLumaSr                   = plt.getTierLevelFeatures()->maxLumaSr;
    const uint32_t frameRate                   = encCfg->m_frameRate;
    const double   minCr                       = plt.getMinCr();
    const double   denominator                 = (minCr * frameRate * 1000);
    if (denominator != 0)
    {
      const double threshold = (formatCapabilityFactorx1000 * maxLumaSr) / (denominator);

      if (numBytesInVclNalUnits > threshold)
      {
        msg(WARNING,
            "WARNING: Encoded stream does not meet MinCr requirements numBytesInVclNalUnits (%.0f) must be <= %.0f. "
            "Try increasing Qp, tier or level\n",
            (double)numBytesInVclNalUnits, threshold);
      }
    }
  }
}

static void validateMinCrRequirements(const ProfileTierLevelFeatures &plt, std::size_t numBytesInVclNalUnits,
                                      const Slice *pSlice, const EncCfg *encCfg, const SEISubpicureLevelInfo &seiSubpic,
                                      const int subPicIdx, const int layerId)
{
  if (plt.getTierLevelFeatures() && plt.getProfileFeatures())
  {
    if (plt.getTier() == Level::Tier::MAIN)
    {
      const uint32_t formatCapabilityFactorx1000 = plt.getProfileFeatures()->formatCapabilityFactorx1000;
      const uint64_t maxLumaSr                   = plt.getTierLevelFeatures()->maxLumaSr;
      const double   denomx1000x256              = (256 * plt.getMinCr() * encCfg->m_frameRate * 1000 * 256);

      for (int i = 0; i < seiSubpic.m_numRefLevels; i++)
      {
        Level::Name level = seiSubpic.m_refLevelIdc[i][layerId];
        if (level != Level::LEVEL15_5)
        {
          const int nonSubpicLayersFraction = seiSubpic.m_nonSubpicLayersFraction[i][layerId];
          const int refLevelFraction        = seiSubpic.m_refLevelFraction[i][subPicIdx][layerId] +
            1;   // m_refLevelFraction is actually sli_ref_level_fraction_minus1
          const uint32_t olsRefLevelFractionx256 =
            nonSubpicLayersFraction * 256 + (256 - nonSubpicLayersFraction) * refLevelFraction;

          const double threshold = formatCapabilityFactorx1000 * maxLumaSr * olsRefLevelFractionx256 / denomx1000x256;

          if (numBytesInVclNalUnits > threshold)
          {
            msg(WARNING,
                "WARNING: Encoded stream for sub-picture %d does not meet MinCr requirements numBytesInVclNalUnits "
                "(%.0f) must be <= %.0f. Try increasing Qp, tier or level\n",
                subPicIdx, (double)numBytesInVclNalUnits, threshold);
          }
        }
      }
    }
  }
}

static std::size_t cabac_zero_word_padding(const Slice *const pcSlice, const Picture *const pic,
                                           const std::size_t   binCountsInNalUnits,
                                           const std::size_t   numBytesInVclNalUnits,
                                           const std::size_t   numZeroWordsAlreadyInserted,
                                           std::ostringstream &nalUnitData, const bool cabacZeroWordPaddingEnabled,
                                           const ProfileTierLevelFeatures &plt)
{
  const SPS         &sps             = *(pcSlice->m_sps);
  const ChromaFormat format          = sps.m_chromaFormatIdc;
  const int log2subWidthCxsubHeightC = (::getComponentScaleX(COMP_Cb, format) + ::getComponentScaleY(COMP_Cb, format));
  const int minCuWidth               = 1 << pcSlice->m_sps->m_log2MinCodingBlockSize;
  const int minCuHeight              = 1 << pcSlice->m_sps->m_log2MinCodingBlockSize;
  const int paddedWidth  = ((pcSlice->m_pps->m_picWidthInLumaSamples + minCuWidth - 1) / minCuWidth) * minCuWidth;
  const int paddedHeight = ((pcSlice->m_pps->m_picHeightInLumaSamples + minCuHeight - 1) / minCuHeight) * minCuHeight;
  const int rawBits      = paddedWidth * paddedHeight *
    (sps.m_bitDepths[ChannelType::LUMA] + ((2 * sps.m_bitDepths[ChannelType::CHROMA]) >> log2subWidthCxsubHeightC));
  const int         vclByteScaleFactor_x3 = (32 + 4 * (plt.getTier() == Level::HIGH ? 1 : 0));
  const std::size_t threshold             = (vclByteScaleFactor_x3 * numBytesInVclNalUnits / 3) + (rawBits / 32);
  // "The value of BinCountsInPicNalUnits shall be less than or equal to vclByteScaleFactor * NumBytesInPicVclNalUnits
  // + ( RawMinCuBits * PicSizeInMinCbsY ) / 32."
  //               binCountsInNalUnits                  <=               vclByteScaleFactor_x3 * numBytesInVclNalUnits /
  //               3 +   rawBits / 32.
  // If it is currently not, then add cabac_zero_words to increase numBytesInVclNalUnits.
  if (binCountsInNalUnits >= threshold)
  {
    // need to add additional cabac zero words (each one accounts for 3 bytes (=00 00 03)) to increase
    // numBytesInVclNalUnits
    const std::size_t targetNumBytesInVclNalUnits =
      ((binCountsInNalUnits - (rawBits / 32)) * 3 + vclByteScaleFactor_x3 - 1) / vclByteScaleFactor_x3;

    if (targetNumBytesInVclNalUnits > numBytesInVclNalUnits)   // It should be!
    {
      const std::size_t numberOfAdditionalBytesNeeded =
        std::max<std::size_t>(0, targetNumBytesInVclNalUnits - numBytesInVclNalUnits - numZeroWordsAlreadyInserted * 3);
      const std::size_t numberOfAdditionalCabacZeroWords = (numberOfAdditionalBytesNeeded + 2) / 3;
      const std::size_t numberOfAdditionalCabacZeroBytes = numberOfAdditionalCabacZeroWords * 3;
      if (cabacZeroWordPaddingEnabled)
      {
        std::vector<uint8_t> zeroBytesPadding(numberOfAdditionalCabacZeroBytes, uint8_t(0));
        for (std::size_t i = 0; i < numberOfAdditionalCabacZeroWords; i++)
        {
          zeroBytesPadding[i * 3 + 2] = 3;   // 00 00 03
        }
        nalUnitData.write(reinterpret_cast<const char *>(&(zeroBytesPadding[0])), numberOfAdditionalCabacZeroBytes);
        msg(NOTICE, "Adding %d bytes of padding\n", uint32_t(numberOfAdditionalCabacZeroWords * 3));
      }
      else
      {
        msg(NOTICE, "Standard would normally require adding %d bytes of padding\n",
            uint32_t(numberOfAdditionalCabacZeroWords * 3));
      }
      return numberOfAdditionalCabacZeroWords;
    }
  }
  return 0;
}

class EfficientFieldIRAPMapping
{
private:
  int  irapGopId;
  bool IRAPtoReorder;
  bool swapIRAPForward;

public:
  EfficientFieldIRAPMapping() : irapGopId(-1), IRAPtoReorder(false), swapIRAPForward(false) {}

  void initialize(const bool isField, const int gopSize, const int POCLast, const int numPicRcvd, const int lastIDR,
                  EncGOP *pEncGop, const EncCfg *encCfg);

  int adjustGOPid(const int gopID);
  int restoreGOPid(const int gopID);
  int GetIRAPGOPid() const { return irapGopId; }
};

void EfficientFieldIRAPMapping::initialize(const bool isField, const int gopSize, const int POCLast,
                                           const int numPicRcvd, const int lastIDR, EncGOP *pEncGop,
                                           const EncCfg *encCfg)
{
  if (isField)
  {
    int pocCurr;
    for (int gopId = 0; gopId < gopSize; gopId++)
    {
      // determine actual POC
      if (POCLast == 0)   // case first frame or first top field
      {
        pocCurr = 0;
      }
      else if (POCLast == 1 && isField)   // case first bottom field, just like the first frame, the poc computation is
                                          // not right anymore, we set the right value
      {
        pocCurr = 1;
      }
      else
      {
        pocCurr = POCLast - numPicRcvd + encCfg->m_GOPList[gopId].m_POC - isField;
      }

      // check if POC corresponds to IRAP
      NalUnitType tmpUnitType = pEncGop->getNalUnitType(pocCurr, lastIDR, isField);
      if (tmpUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL &&
          tmpUnitType <= NAL_UNIT_CODED_SLICE_CRA)   // if picture is an IRAP
      {
        if (pocCurr % 2 == 0 && gopId < gopSize - 1 &&
            encCfg->m_GOPList[gopId].m_POC == encCfg->m_GOPList[gopId + 1].m_POC - 1)
        {   // if top field and following picture in enc order is associated bottom field
          irapGopId       = gopId;
          IRAPtoReorder   = true;
          swapIRAPForward = true;
          break;
        }
        if (pocCurr % 2 != 0 && gopId > 0 && encCfg->m_GOPList[gopId].m_POC == encCfg->m_GOPList[gopId - 1].m_POC + 1)
        {
          // if picture is an IRAP remember to process it first
          irapGopId       = gopId;
          IRAPtoReorder   = true;
          swapIRAPForward = false;
          break;
        }
      }
    }
  }
}

int EfficientFieldIRAPMapping::adjustGOPid(const int gopId)
{
  if (IRAPtoReorder)
  {
    if (swapIRAPForward)
    {
      if (gopId == irapGopId)
      {
        return irapGopId + 1;
      }
      else if (gopId == irapGopId + 1)
      {
        return irapGopId;
      }
    }
    else
    {
      if (gopId == irapGopId - 1)
      {
        return irapGopId;
      }
      else if (gopId == irapGopId)
      {
        return irapGopId - 1;
      }
    }
  }
  return gopId;
}

int EfficientFieldIRAPMapping::restoreGOPid(const int gopId)
{
  if (IRAPtoReorder)
  {
    if (swapIRAPForward)
    {
      if (gopId == irapGopId)
      {
        IRAPtoReorder = false;
        return irapGopId + 1;
      }
      else if (gopId == irapGopId + 1)
      {
        return gopId - 1;
      }
    }
    else
    {
      if (gopId == irapGopId)
      {
        return irapGopId - 1;
      }
      else if (gopId == irapGopId - 1)
      {
        IRAPtoReorder = false;
        return irapGopId;
      }
    }
  }
  return gopId;
}

static void printHash(const HashType hashType, const std::string &digestStr)
{
  const char *decodedPictureHashModeName;
  switch (hashType)
  {
  case HashType::MD5:
    decodedPictureHashModeName = "MD5";
    break;
  case HashType::CRC:
    decodedPictureHashModeName = "CRC";
    break;
  case HashType::CHECKSUM:
    decodedPictureHashModeName = "Checksum";
    break;
  default:
    decodedPictureHashModeName = nullptr;
    break;
  }
  if (decodedPictureHashModeName != nullptr)
  {
    if (digestStr.empty())
    {
      msg(NOTICE, " [%s:%s]", decodedPictureHashModeName, "?");
    }
    else
    {
      msg(NOTICE, " [%s:%s]", decodedPictureHashModeName, digestStr.c_str());
    }
  }
}

bool isPicEncoded(int targetPoc, int curPoc, int curTLayer, int gopSize, int intraPeriod)
{
  const int tarGop = targetPoc / gopSize;
  const int curGop = curPoc / gopSize;

  if (tarGop + 1 == curGop)
  {
    // part of next GOP only for tl0 pics
    return curTLayer == 0;
  }

  const int tarIFr = (targetPoc / intraPeriod) * intraPeriod;
  const int curIFr = (curPoc / intraPeriod) * intraPeriod;

  if (curIFr != tarIFr)
  {
    return false;
  }

  int tarId = targetPoc - tarGop * gopSize;

  if (tarGop > curGop)
  {
    return (tarId == 0) ? (0 == curTLayer) : (1 >= curTLayer);
  }

  if (tarGop + 1 < curGop)
  {
    return false;
  }

  int curId = curPoc - curGop * gopSize;
  int tarTL = 0;

  while (tarId != 0)
  {
    gopSize /= 2;
    if (tarId >= gopSize)
    {
      tarId -= gopSize;
      if (curId != 0)
      {
        curId -= gopSize;
      }
    }
    else if (curId == gopSize)
    {
      curId = 0;
    }
    tarTL++;
  }

  return curTLayer <= tarTL && curId == 0;
}

void trySkipOrDecodePicture(bool &decPic, bool &encPic, const EncCfg *encCfg, Picture *pic,
                            EnumArray<ParameterSetMap<APS>, ApsType> *apsMap)
{
  // check if we should decode a leading bitstream
  if (!encCfg->m_decodeBitstreams[0].empty())
  {
    static bool bDecode1stPart = true; /* TODO: MT */
    if (bDecode1stPart)
    {
      if (encCfg->m_forceDecodeBitstream1)
      {
        if ((bDecode1stPart = tryDecodePicture(pic, pic->m_poc, encCfg->m_decodeBitstreams[0],
#if ENABLE_NNLF
                                               encCfg->m_nnlfModelName, encCfg->m_nnlfDebugOption,
#endif
                                               apsMap, false)))
        {
          decPic = bDecode1stPart;
        }
      }
      else
      {
        // update decode decision
        bool dbgCTU = encCfg->m_debugCTU > -1 && encCfg->m_switchPOC == pic->m_poc;

        if ((bDecode1stPart = (encCfg->m_switchPOC != pic->m_poc) || dbgCTU) &&
            (bDecode1stPart = tryDecodePicture(pic, pic->m_poc, encCfg->m_decodeBitstreams[0],
#if ENABLE_NNLF
                                               encCfg->m_nnlfModelName, encCfg->m_nnlfDebugOption,
#endif
                                               apsMap, false, encCfg->m_debugCTU, encCfg->m_switchPOC)))
        {
          if (dbgCTU)
          {
            encPic         = true;
            decPic         = false;
            bDecode1stPart = false;

            return;
          }
          decPic = bDecode1stPart;
          return;
        }
        else if (pic->m_poc)
        {
          // reset decoder if used and not required any further
          tryDecodePicture(nullptr, 0, std::string("")
#if ENABLE_NNLF
                                         ,
                           std::string(""), 0
#endif
          );
        }
      }
    }

    encPic |= encCfg->m_forceDecodeBitstream1 && !decPic;
    if (encCfg->m_forceDecodeBitstream1)
    {
      return;
    }
  }

  // check if we should decode a trailing bitstream
  if (!encCfg->m_decodeBitstreams[1].empty())
  {
    const int iNextKeyPOC   = (1 + encCfg->m_switchPOC / encCfg->m_gopSize) * encCfg->m_gopSize;
    const int iNextIntraPOC = (1 + (encCfg->m_switchPOC / encCfg->m_intraPeriod)) * encCfg->m_intraPeriod;
    const int iRestartIntraPOC =
      iNextIntraPOC + (((iNextKeyPOC == iNextIntraPOC) && encCfg->m_switchDQP) ? encCfg->m_intraPeriod : 0);

    bool  bDecode2ndPart = (pic->m_poc >= iRestartIntraPOC);
    int   expectedPoc    = pic->m_poc;
    Slice slice0;
    if (encCfg->m_bs2ModPOCAndType)
    {
      expectedPoc = pic->m_poc - iRestartIntraPOC;
      slice0.copySliceInfo(pic->m_slices[0], false);
    }
    if (bDecode2ndPart &&
        (bDecode2ndPart = tryDecodePicture(pic, expectedPoc, encCfg->m_decodeBitstreams[1],
#if ENABLE_NNLF
                                           encCfg->m_nnlfModelName, encCfg->m_nnlfDebugOption,
#endif
                                           apsMap, true)))
    {
      decPic = bDecode2ndPart;
      if (encCfg->m_bs2ModPOCAndType)
      {
        for (int i = 0; i < pic->m_slices.size(); i++)
        {
          pic->m_slices[i]->m_poc = slice0.m_poc;
          if (pic->m_slices[i]->m_eNalUnitType != slice0.m_eNalUnitType && pic->m_slices[i]->getIdrPicFlag() &&
              slice0.getRapPicFlag() && slice0.isIntra())
          {
            // patch IDR-slice to CRA-Intra-slice
            pic->m_slices[i]->m_eNalUnitType = slice0.m_eNalUnitType;
            pic->m_slices[i]->m_iLastIDR     = slice0.m_iLastIDR;
            if (pic->m_cs->picHeader->m_picColFromL0Flag)
            {
              pic->m_slices[i]->m_colFromL0Flag = slice0.m_colFromL0Flag;
              pic->m_slices[i]->m_colRefIdx     = slice0.m_colRefIdx;
            }
          }
        }
      }
      return;
    }
  }

  // leave here if we do not use forward to poc
  if (encCfg->m_fastForwardToPOC < 0)
  {
    // let's encode
    encPic = true;
    return;
  }

  // this is the forward to poc section
  static bool bHitFastForwardPOC = false; /* TODO: MT */
  if (bHitFastForwardPOC ||
      isPicEncoded(encCfg->m_fastForwardToPOC, pic->m_poc, pic->m_temporalId, encCfg->m_gopSize, encCfg->m_intraPeriod))
  {
    bHitFastForwardPOC |= encCfg->m_fastForwardToPOC == pic->m_poc;   // once we hit the poc we continue encoding

    if (bHitFastForwardPOC && encCfg->m_stopAfterFFtoPOC && encCfg->m_fastForwardToPOC != pic->m_poc)
    {
      return;
    }

    // except if FastForwardtoPOC is meant to be a SwitchPOC in thist case drop all preceding pictures
    if (bHitFastForwardPOC && (encCfg->m_switchPOC == encCfg->m_fastForwardToPOC) &&
        (encCfg->m_fastForwardToPOC > pic->m_poc))
    {
      return;
    }
    // let's encode
    encPic = true;
  }
}

void EncGOP::xPicInitHashME(Picture *pic, const PPS *pps, PicList &rcListPic)
{
  if (!getUseHashME())
  {
    return;
  }

  PicList::iterator iterPic = rcListPic.begin();
  while (iterPic != rcListPic.end())
  {
    Picture *refPic = *(iterPic++);

    if (refPic->m_poc != pic->m_poc && refPic->m_referenced)
    {
      bool validPOC = ((refPic->m_poc == getUseHashMEPOCToCheck()) && !getUseHashMEPOCChecked());
      if (!refPic->m_hashMap.isInitial() || validPOC)
      {
        if (validPOC)
        {
          setUseHashMEPOCChecked(true);
          Pel      *picSrc    = refPic->getOrigBuf().get(COMP_Y).buf;
          ptrdiff_t stridePic = refPic->getOrigBuf().get(COMP_Y).stride;
          int       picWidth  = refPic->lwidth();
          int       picHeight = refPic->lheight();
          int       blockSize = 4;
          int       allNum    = 0;
          int       simpleNum = 0;
          for (int j = 0; j <= picHeight - blockSize; j += blockSize)
          {
            for (int i = 0; i <= picWidth - blockSize; i += blockSize)
            {
              Pel *curBlock  = picSrc + j * stridePic + i;
              bool isHorSame = true;
              for (int m = 0; m < blockSize && isHorSame; m++)
              {
                for (int n = 1; n < blockSize && isHorSame; n++)
                {
                  if (curBlock[m * stridePic] != curBlock[m * stridePic + n])
                  {
                    isHorSame = false;
                  }
                }
              }
              bool isVerSame = true;
              for (int m = 1; m < blockSize && isVerSame; m++)
              {
                for (int n = 0; n < blockSize && isVerSame; n++)
                {
                  if (curBlock[n] != curBlock[m * stridePic + n])
                  {
                    isVerSame = false;
                  }
                }
              }
              allNum++;
              if (isHorSame || isVerSame)
              {
                simpleNum++;
              }
            }
          }

          if (simpleNum < 0.3 * allNum)
          {
            setUseHashME(false);
            break;
          }
        }
        refPic->addPictureToHashMapForInter();
      }
    }
  }
}

void EncGOP::xPicInitRateControl(int &estimatedBits, int gopId, double &lambda, Picture *pic, Slice *slice)
{
  if (!m_encCfg->m_RCEnableRateControl)   // TODO: does this work with multiple slices and slice-segments?
  {
    return;
  }
  int frameLevel = m_pcRateCtrl->getRCSeq()->getGOPID2Level(gopId);
  if (pic->m_slices[0]->isIRAP())
  {
    frameLevel = 0;
  }
  m_pcRateCtrl->initRCPic(frameLevel);
  estimatedBits = m_pcRateCtrl->getRCPic()->getTargetBits();

  if (m_pcRateCtrl->getCpbSaturationEnabled() && frameLevel != 0)
  {
    int estimatedCpbFullness = m_pcRateCtrl->getCpbState() + m_pcRateCtrl->getBufferingRate();

    // prevent overflow
    if (estimatedCpbFullness - estimatedBits > (int)(m_pcRateCtrl->getCpbSize() * 0.9f))
    {
      estimatedBits = estimatedCpbFullness - (int)(m_pcRateCtrl->getCpbSize() * 0.9f);
    }

    estimatedCpbFullness -= m_pcRateCtrl->getBufferingRate();
    // prevent underflow
    if (estimatedCpbFullness - estimatedBits < m_pcRateCtrl->getRCPic()->getLowerBound())
    {
      estimatedBits = std::max(200, estimatedCpbFullness - m_pcRateCtrl->getRCPic()->getLowerBound());
    }

    m_pcRateCtrl->getRCPic()->setTargetBits(estimatedBits);
  }

  int sliceQP = m_encCfg->m_RCInitialQP;
  if ((slice->m_poc == 0 && m_encCfg->m_RCInitialQP > 0) ||
      (frameLevel == 0 && m_encCfg->m_RCForceIntraQP))   // QP is specified
  {
    int    numberBFrames       = (m_encCfg->m_gopSize - 1);
    double dLambdaScale        = 1.0 - Clip3(0.0, 0.5, 0.05 * (double)numberBFrames);
    double dQPFactor           = 0.57 * dLambdaScale;
    int    shiftQP             = 12;
    int    bitdepthLumaQPScale = 6 *
      (slice->m_sps->m_bitDepths[ChannelType::LUMA] - 8 -
       DISTORTION_PRECISION_ADJUSTMENT(slice->m_sps->m_internalBitDepth[ChannelType::LUMA]));
    double qpTemp = (double)sliceQP + bitdepthLumaQPScale - shiftQP;
    lambda        = dQPFactor * pow(2.0, qpTemp / 3.0);
  }
  else if (frameLevel == 0)   // intra case, but use the model
  {
    m_pcSliceEncoder->calCostPictureI(pic);
    if (m_encCfg->m_intraPeriod != 1)   // do not refine allocated bits for all intra case
    {
      int bits = m_pcRateCtrl->getRCSeq()->getLeftAverageBits();
      bits     = m_pcRateCtrl->getRCPic()->getRefineBitsForIntra(bits);

      if (m_pcRateCtrl->getCpbSaturationEnabled())
      {
        int estimatedCpbFullness = m_pcRateCtrl->getCpbState() + m_pcRateCtrl->getBufferingRate();

        // prevent overflow
        if (estimatedCpbFullness - bits > (int)(m_pcRateCtrl->getCpbSize() * 0.9f))
        {
          bits = estimatedCpbFullness - (int)(m_pcRateCtrl->getCpbSize() * 0.9f);
        }

        estimatedCpbFullness -= m_pcRateCtrl->getBufferingRate();
        // prevent underflow
        if (estimatedCpbFullness - bits < m_pcRateCtrl->getRCPic()->getLowerBound())
        {
          bits = estimatedCpbFullness - m_pcRateCtrl->getRCPic()->getLowerBound();
        }
      }

      if (bits < 200)
      {
        bits = 200;
      }
      m_pcRateCtrl->getRCPic()->setTargetBits(bits);
    }

    std::list<EncRCPic *> listPreviousPicture = m_pcRateCtrl->getPicList();
    m_pcRateCtrl->getRCPic()->getLCUInitTargetBits();
    lambda  = m_pcRateCtrl->getRCPic()->estimatePicLambda(listPreviousPicture, slice->isIRAP());
    sliceQP = m_pcRateCtrl->getRCPic()->estimatePicQP(lambda, listPreviousPicture);
  }
  else   // normal case
  {
    std::list<EncRCPic *> listPreviousPicture = m_pcRateCtrl->getPicList();
    lambda  = m_pcRateCtrl->getRCPic()->estimatePicLambda(listPreviousPicture, slice->isIRAP());
    sliceQP = m_pcRateCtrl->getRCPic()->estimatePicQP(lambda, listPreviousPicture);
  }

  sliceQP = Clip3(-slice->m_sps->m_qpBDOffset[ChannelType::LUMA], MAX_QP, sliceQP);
  m_pcRateCtrl->getRCPic()->setPicEstQP(sliceQP);

  m_pcSliceEncoder->resetQP(pic, sliceQP, lambda);
}

void EncGOP::xPicInitLMCS(Picture *pic, PicHeader *picHeader, Slice *slice)
{
  if (slice->m_sps->m_lmcsEnabled)
  {
    const SliceType realSliceType = slice->m_eSliceType;
    SliceType       condSliceType = realSliceType;

    if (condSliceType != I_SLICE && slice->m_nuhLayerId > 0 &&
        (slice->m_eNalUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL && slice->m_eNalUnitType <= NAL_UNIT_CODED_SLICE_CRA))
    {
      condSliceType = I_SLICE;
    }
    m_pcReshaper->getReshapeCW()->rspTid     = slice->m_uiTLayer + (slice->isIntra() ? 0 : 1);
    m_pcReshaper->getReshapeCW()->rspSliceQP = slice->m_iSliceQp;

    m_pcReshaper->setSrcReshaped(false);
    m_pcReshaper->m_recReshaped = true;

    m_pcReshaper->m_sliceReshapeInfo.chrResScalingOffset = m_encCfg->m_CSoffset;

    if (m_encCfg->m_reshapeSignalType == RESHAPE_SIGNAL_PQ)
    {
      m_pcReshaper->preAnalyzerHDR(pic, condSliceType, m_encCfg->m_reshapeCW, m_encCfg->m_dualITree);
    }
    else if (m_encCfg->m_reshapeSignalType == RESHAPE_SIGNAL_SDR || m_encCfg->m_reshapeSignalType == RESHAPE_SIGNAL_HLG)
    {
      m_pcReshaper->preAnalyzerLMCS(pic, m_encCfg->m_reshapeSignalType, condSliceType, m_encCfg->m_reshapeCW);
    }
    else
    {
      THROW("Reshaper for other signal currently not defined!");
    }
    CHECK(!isChromaEnabled(m_encCfg->m_chromaFormatIdc) && m_pcReshaper->m_sliceReshapeInfo.enableChromaAdj, "Error");
    if (condSliceType == I_SLICE)
    {
      if (m_encCfg->m_reshapeSignalType == RESHAPE_SIGNAL_PQ)
      {
        m_pcReshaper->initLUTfromdQPModel();
        m_pcEncLib->getRdCost()->updateReshapeLumaLevelToWeightTableChromaMD(m_pcReshaper->m_invLUT);
      }
      else if (m_encCfg->m_reshapeSignalType == RESHAPE_SIGNAL_SDR ||
               m_encCfg->m_reshapeSignalType == RESHAPE_SIGNAL_HLG)
      {
        if (m_pcReshaper->m_reshapeFlag)
        {
          m_pcReshaper->constructReshaperLMCS();
          m_pcEncLib->getRdCost()->updateReshapeLumaLevelToWeightTable(
            m_pcReshaper->m_sliceReshapeInfo, m_pcReshaper->getWeightTable(), m_pcReshaper->getCWeight());
        }
      }
      else
      {
        THROW("Reshaper for other signal currently not defined!");
      }

      m_pcReshaper->m_ctuFlag = false;
      if (realSliceType != condSliceType)
      {
        m_pcReshaper->m_ctuFlag = true;
      }
    }
    else
    {
      if (!m_pcReshaper->m_reshapeFlag)
      {
        m_pcReshaper->m_ctuFlag = false;
      }
      else
      {
        m_pcReshaper->m_ctuFlag = true;
      }

      m_pcReshaper->m_sliceReshapeInfo.sliceReshaperModelPresentFlag = false;

      if (m_encCfg->m_reshapeSignalType == RESHAPE_SIGNAL_PQ)
      {
        m_pcEncLib->getRdCost()->restoreReshapeLumaLevelToWeightTable();
      }
      else if (m_encCfg->m_reshapeSignalType == RESHAPE_SIGNAL_SDR ||
               m_encCfg->m_reshapeSignalType == RESHAPE_SIGNAL_HLG)
      {
        int modIP = pic->m_poc - pic->m_poc / m_encCfg->m_reshapeCW.rspFpsToIp * m_encCfg->m_reshapeCW.rspFpsToIp;
        if (m_pcReshaper->m_reshapeFlag && m_encCfg->m_reshapeCW.updateCtrl == 2 && modIP == 0)
        {
          m_pcReshaper->m_sliceReshapeInfo.sliceReshaperModelPresentFlag = true;
          m_pcReshaper->constructReshaperLMCS();
          m_pcEncLib->getRdCost()->updateReshapeLumaLevelToWeightTable(
            m_pcReshaper->m_sliceReshapeInfo, m_pcReshaper->getWeightTable(), m_pcReshaper->getCWeight());
        }
      }
      else
      {
        THROW("Reshaper for other signal currently not defined!");
      }
    }

    // set all necessary information in LMCS APS and picture header
    picHeader->m_lmcsEnabledFlag             = m_pcReshaper->m_sliceReshapeInfo.sliceReshaperEnableFlag;
    slice->m_lmcsEnabledFlag                 = m_pcReshaper->m_sliceReshapeInfo.sliceReshaperEnableFlag;
    picHeader->m_lmcsChromaResidualScaleFlag = m_pcReshaper->m_sliceReshapeInfo.enableChromaAdj == 1;
    if (m_pcReshaper->m_sliceReshapeInfo.sliceReshaperModelPresentFlag)
    {
      int apsId = std::min<int>(
        3, m_pcEncLib->m_vps == nullptr ? 0 : m_pcEncLib->m_vps->m_generalLayerIdx[m_pcEncLib->m_layerId]);
      picHeader->m_lmcsApsId = apsId;
      APS *lmcsAPS           = picHeader->m_lmcsAps;
      if (lmcsAPS == nullptr)
      {
        ParameterSetMap<APS> *apsMap = m_pcEncLib->getApsMap(ApsType::LMCS);
        lmcsAPS                      = apsMap->getPS(apsId);
        if (lmcsAPS == nullptr)
        {
          lmcsAPS            = apsMap->allocatePS(apsId);
          lmcsAPS->m_APSId   = apsId;
          lmcsAPS->m_APSType = ApsType::LMCS;
        }
        picHeader->m_lmcsAps   = lmcsAPS;
        picHeader->m_lmcsApsId = lmcsAPS->m_APSId;
      }
      SliceReshapeInfo &tInfo      = lmcsAPS->m_reshapeAPSInfo;
      SliceReshapeInfo &sInfo      = m_pcReshaper->m_sliceReshapeInfo;
      tInfo.reshaperModelMaxBinIdx = sInfo.reshaperModelMaxBinIdx;
      tInfo.reshaperModelMinBinIdx = sInfo.reshaperModelMinBinIdx;
      memcpy(tInfo.reshaperModelBinCWDelta, sInfo.reshaperModelBinCWDelta, sizeof(int) * (PIC_CODE_CW_BINS));
      tInfo.maxNbitsNeededDeltaCW = sInfo.maxNbitsNeededDeltaCW;
      tInfo.chrResScalingOffset   = sInfo.chrResScalingOffset;
      m_pcEncLib->getApsMap(ApsType::LMCS)->setChangedFlag(lmcsAPS->m_APSId);
    }

    if (picHeader->m_lmcsEnabledFlag)
    {
      const int apsId = std::min<int>(
        3, m_pcEncLib->m_vps == nullptr ? 0 : m_pcEncLib->m_vps->m_generalLayerIdx[m_pcEncLib->m_layerId]);
      picHeader->m_lmcsApsId = apsId;
    }
  }
  else
  {
    m_pcReshaper->m_ctuFlag = false;
  }
}

void EncGOP::computeSignalling(Picture *pic, Slice *pcSlice) const
{
  bool deriveETSRC =
    (!pcSlice->m_tsResidualCodingDisabledFlag && pcSlice->m_sps->m_spsRangeExtension.m_tsrcRicePresentFlag);
  bool deriveRLSCP = pcSlice->m_sps->m_spsRangeExtension.m_reverseLastSigCoeffEnabledFlag;

  if (deriveETSRC || deriveRLSCP)
  {
    int              total       = 0;
    int              ignored     = 0;
    uint32_t         freq[128]   = {};
    static const int offsetRLSCP = 15;   // Equivalent to 2.5 bits
    for (CompID compID = COMP_Y; compID <= (!isChromaEnabled(pic->chromaFormat) ? COMP_Y : COMP_Cr);
         compID        = CompID(compID + 1))
    {
      int bitDepth = pic->m_cs->sps->m_bitDepths[toChannelType(compID)];
      int qpBase   = offsetRLSCP + 4 - (bitDepth - 8) * 6;
      int qpOffs   = pcSlice->m_iSliceQp - qpBase;

      const CPelBuf   buffer = pic->getOrigBuf(compID);
      const ptrdiff_t stride = buffer.stride;
      const int       height = buffer.height;
      const int       width  = buffer.width;
      total += (height - 1) * (width - 1);

      const Pel *buf = buffer.buf;
      for (int h = 1; h < height; h++)
      {
        const Pel *above = buf;
        buf += stride;
        for (int w = 1; w < width; w++)
        {
          Pel residual = std::min(std::abs(buf[w] - buf[w - 1]), std::abs(buf[w] - above[w]));
          if (residual > 0)
          {
            int resLevel = (int)std::round(6 * std::log2(residual)) - qpOffs;
            freq[Clip3(0, 127, resLevel)]++;
          }
          else
          {
            ignored++;
          }
        }
      }
    }

    if (deriveRLSCP)
    {
      pcSlice->m_reverseLastSigCoeffFlag = ((freq[0] + ignored) < total / 2);
    }

    if (deriveETSRC)
    {
      total -= ignored;
      int target[3] = { total / 6, total / 3, total / 2 };
      int win[3];

      int winCount  = 0;
      int totalFreq = 0;
      for (int i = 0; i < 128 && winCount < 3; i++)
      {
        totalFreq += freq[i];
        while (totalFreq >= target[winCount] && winCount < 3)
        {
          win[winCount++] = i;
        }
      }

      int winCentre = ((win[0] + win[1] * 2 + win[2]) / 4) - offsetRLSCP;
      int tsrcIndex = Clip3<int>(0, 7, winCentre / 6);
      if (ignored > total)
      {
        tsrcIndex = std::min(tsrcIndex, std::max(0, pic->m_cs->sps->m_bitDepths[ChannelType::LUMA] - 9));
      }
      pcSlice->m_tsrcIndex = tsrcIndex;
    }
  }
}

int EncGOP::getRprResolutionIndex(int idx) const
{
  for (int i = 0; i < NUM_RPR_PPS; i++)
  {
    if (RPR_PPS_ID[i] == idx)
    {
      return i;
    }
  }
  return -1;
}
class BIFCabacEstImp : public BIFCabacEst
{
  CABACWriter *CABACEstimator;

public:
  BIFCabacEstImp(CABACWriter *_CABACEstimator) : CABACEstimator(_CABACEstimator) {};
  virtual ~BIFCabacEstImp() {};

#if ENABLE_CABAC_DUMP
  virtual uint64_t getBits(const CompID compID, Slice &slice, const BifParams &htdfParams)
#else
  virtual uint64_t getBits(const CompID compID, const Slice &slice, const BifParams &htdfParams)
#endif
  {
    CABACEstimator->initCtxModels(slice);
    CABACEstimator->resetBits();
    CABACEstimator->bif(compID, slice, htdfParams);
    return CABACEstimator->getEstFracBits();
  }
};
// ====================================================================================================================
// Public member functions
// ====================================================================================================================
void EncGOP::compressGOP(int pocLast, int numPicRcvd, PicList &rcListPic, std::list<PelUnitBuf *> &rcListPicYuvRecOut,
                         bool isField, bool isTff, const InputColourSpaceConversion snr_conversion,
                         const bool printFrameMSE, const bool printMSSSIM, bool isEncodeLtRef, const int picIdInGOP)
{
  // TODO: Split this function up.

  Picture   *pic       = nullptr;
  PicHeader *picHeader = nullptr;

  Slice           *pcSlice;
  OutputBitstream *pcBitstreamRedirect;
  pcBitstreamRedirect = new OutputBitstream;
  AccessUnit::iterator
    itLocationToPushSliceHeaderNALU;   // used to store location where NALU containing slice header is to be inserted
  Picture *scaledRefPic[MAX_NUM_REF] = {};

  xInitGOP(pocLast, numPicRcvd, isField, isEncodeLtRef);

  m_numPicsCoded = 0;
  SEIMessages        leadingSeiMessages;
  SEIMessages        nestedSeiMessages;
  SEIMessages        duInfoSeiMessages;
  SEIMessages        trailingSeiMessages;
  std::deque<DUData> duData;

  EfficientFieldIRAPMapping effFieldIRAPMap;
  if (m_encCfg->m_efficientFieldIRAPEnabled)
  {
    effFieldIRAPMap.initialize(isField, m_iGopSize, pocLast, numPicRcvd, m_iLastIDR, this, m_encCfg);
  }

  if (isField && picIdInGOP == 0)
  {
    for (int gopId = 0; gopId < std::max(2, m_iGopSize); gopId++)
    {
      m_encCfg->m_RPLList0[gopId].m_isEncoded = false;
      m_encCfg->m_RPLList1[gopId].m_isEncoded = false;
      m_encCfg->m_GOPList[gopId].m_isEncoded  = false;
    }
  }
  for (int gopId = picIdInGOP; gopId <= picIdInGOP; gopId++)
  {
    // reset flag indicating whether pictures have been encoded
    m_encCfg->m_RPLList0[gopId].m_isEncoded = false;
    m_encCfg->m_RPLList1[gopId].m_isEncoded = false;
    m_encCfg->m_GOPList[gopId].m_isEncoded  = false;
    if (m_encCfg->m_efficientFieldIRAPEnabled)
    {
      gopId = effFieldIRAPMap.adjustGOPid(gopId);
    }

    //-- For time output for each slice
    auto beforeTime = std::chrono::steady_clock::now();

    /////////////////////////////////////////////////////////////////////////////////////////////////// Initial to start
    /// encoding
    int timeOffset;
    int pocCurr;
    int multipleFactor = m_encCfg->m_compositeRefEnabled ? 2 : 1;

    if (pocLast == 0)   // case first frame or first top field
    {
      pocCurr    = 0;
      timeOffset = isField ? (1 - multipleFactor) : multipleFactor;
    }
    else if (pocLast == 1 && isField)   // case first bottom field, just like the first frame, the poc computation is
                                        // not right anymore, we set the right value
    {
      pocCurr    = 1;
      timeOffset = multipleFactor + 1;
    }
    else
    {
      pocCurr = pocLast - numPicRcvd * multipleFactor + m_encCfg->m_GOPList[gopId].m_POC -
        ((isField && m_iGopSize > 1) ? 1 : 0);
      timeOffset = m_encCfg->m_GOPList[gopId].m_POC;
    }

    if (m_encCfg->m_compositeRefEnabled && isEncodeLtRef)
    {
      pocCurr++;
      timeOffset--;
    }
    if (pocCurr / multipleFactor >= m_encCfg->m_framesToBeEncoded)
    {
      if (m_encCfg->m_efficientFieldIRAPEnabled)
      {
        gopId = effFieldIRAPMap.restoreGOPid(gopId);
      }
      continue;
    }

    if (getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
        getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_IDR_N_LP)
    {
      m_iLastIDR = pocCurr;
    }

    // start a new access unit: create an entry in the list of output access units
    AccessUnit accessUnit;
    accessUnit.temporalId = m_encCfg->m_GOPList[gopId].m_temporalId;
    xGetBuffer(rcListPic, rcListPicYuvRecOut, numPicRcvd, timeOffset, pic, pocCurr, isField);
    picHeader          = pic->m_cs->picHeader;
    picHeader->m_spsId = pic->m_cs->pps->m_spsId;
    if (getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_RASL && m_encCfg->m_rprRASLtoolSwitch &&
        m_encCfg->m_wrapAround)
    {
      picHeader->m_ppsId = 4;
      pic->m_cs->pps     = m_pcEncLib->getPPS(4);
    }
    else
    {
      picHeader->m_ppsId = pic->m_cs->pps->m_ppsId;
    }
    picHeader->m_splitConsOverrideFlag    = false;
    // initial two flags to be false
    picHeader->m_picInterSliceAllowedFlag = false;
    picHeader->m_picIntraSliceAllowedFlag = false;

#if ER_CHROMA_QP_WCG_PPS
    // th this is a hot fix for the choma qp control
    if (m_encCfg->m_wcgChromaQpControl.enabled && m_encCfg->m_switchPOC != -1)
    {
      static int usePPS = 0; /* TODO: MT */
      if (pocCurr == m_encCfg->m_switchPOC)
      {
        usePPS = 1;
      }
      const PPS *pPPS = m_pcEncLib->getPPS(usePPS);
      // replace the pps with a more appropriated one
      pic->m_cs->pps  = pPPS;
    }
#endif

    // create objects based on the picture size
    const int          picWidth        = pic->m_cs->pps->m_picWidthInLumaSamples;
    const int          picHeight       = pic->m_cs->pps->m_picHeightInLumaSamples;
    const int          maxCUWidth      = pic->m_cs->sps->m_maxCuWidth;
    const int          maxCUHeight     = pic->m_cs->sps->m_maxCuHeight;
    const ChromaFormat chromaFormatIdc = pic->m_cs->sps->m_chromaFormatIdc;
    const int          maxTotalCUDepth = floorLog2(maxCUWidth) - pic->m_cs->sps->m_log2MinCodingBlockSize;

    m_pcSliceEncoder->create(picWidth, picHeight, chromaFormatIdc, maxCUWidth, maxCUHeight, maxTotalCUDepth);

    pic->createTempBuffers(pic->m_cs->pps->pcv->maxCUWidth);
    pic->m_cs->createCoeffs((bool)pic->m_cs->sps->m_PLTMode);

    //  Slice data initialization
    pic->clearSliceBuffer();
    pic->allocateNewSlice();
    m_pcSliceEncoder->setSliceSegmentIdx(0);

    const NalUnitType naluType = getNalUnitType(pocCurr, m_iLastIDR, isField);
    pic->setPictureType(naluType);
    m_pcSliceEncoder->initEncSlice(pic, pocLast, pocCurr, gopId, pcSlice, isField, isEncodeLtRef, m_pcEncLib->m_layerId,
                                   naluType);

    DTRACE_UPDATE(g_trace_ctx, (std::make_pair("poc", pocCurr)));
    DTRACE_UPDATE(g_trace_ctx, (std::make_pair("final", 0)));

    getRealRange(pic);

#if !SHARP_LUMA_DELTA_QP
    // Set Frame/Field coding
    pic->m_fieldPic = isField;
#endif

    pcSlice->m_iLastIDR            = m_iLastIDR;
    pcSlice->m_independentSliceIdx = 0;

    if (pcSlice->m_eSliceType == B_SLICE && m_encCfg->m_GOPList[gopId].m_sliceType == 'P')
    {
      pcSlice->m_eSliceType = P_SLICE;
#if ENABLE_CABAC_DUMP
      pcSlice->m_cabacInitSliceType = P_SLICE;
#endif
    }
    if (pcSlice->m_eSliceType == B_SLICE && m_encCfg->m_GOPList[gopId].m_sliceType == 'I')
    {
      pcSlice->m_eSliceType = I_SLICE;
#if ENABLE_CABAC_DUMP
      pcSlice->m_cabacInitSliceType = I_SLICE;
#endif
    }
    pcSlice->m_uiTLayer = m_encCfg->m_GOPList[gopId].m_temporalId;

    // Set the nal unit type
    pcSlice->m_eNalUnitType = getNalUnitType(pocCurr, m_iLastIDR, isField);
    // set two flags according to slice type presented in the picture
    if (pcSlice->m_eSliceType != I_SLICE)
    {
      picHeader->m_picInterSliceAllowedFlag = true;
    }
    if (pcSlice->m_eSliceType == I_SLICE)
    {
      picHeader->m_picIntraSliceAllowedFlag = true;
    }
    picHeader->m_gdrOrIrapPicFlag = (picHeader->m_gdrPicFlag || pcSlice->isIRAP());
    if (!picHeader->m_gdrPicFlag)
    {
      picHeader->m_recoveryPocCnt = -1;   // th???
    }

#if ENABLE_NNLF
    if (pocCurr == m_encCfg->m_nnlfStartPoc)
    {
      m_nnlfEnabled = true;
    }
    // picHeader->m_nnlfDisabled = !m_nnlfEnabled || ( m_encCfg->m_nnlfDebugOption == 1 && pcSlice->m_eSliceType !=
    // I_SLICE );
    picHeader->m_nnlfDisabled = !m_nnlfEnabled ||
      (m_encCfg->m_nnlfDebugOption == 1 && pcSlice->m_eSliceType != I_SLICE) ||
      (pocCurr % m_encCfg->m_nnlfPocDivisibleByN != 0);
#endif

    PROFILER_SET(g_timeProfiler, g_allTimeProfilers[pcSlice->m_eSliceType]);
    PROFILER_START(g_timeProfiler, P_COMPRESS_GOP);

    if (m_encCfg->m_efficientFieldIRAPEnabled)
    {
      if (pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
          pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
          pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA)   // IRAP picture
      {
        m_associatedIRAPType[pic->m_layerId] = pcSlice->m_eNalUnitType;
        m_associatedIRAPPOC[pic->m_layerId]  = pocCurr;
        if (m_encCfg->m_seiCfg.m_edrapIndicationSEIEnabled)
        {
          m_latestEDRAPPOC  = MAX_INT;
          pic->m_edrapRapId = 0;
        }
      }
      pcSlice->m_iAssociatedIRAPType = m_associatedIRAPType[pic->m_layerId];
      pcSlice->m_iAssociatedIRAPPOC  = m_associatedIRAPPOC[pic->m_layerId];
    }

    pcSlice->decodingRefreshMarking(m_pocCRA, m_refreshPending, rcListPic, m_encCfg->m_efficientFieldIRAPEnabled);
    if (m_encCfg->m_compositeRefEnabled && isEncodeLtRef)
    {
      setUseLTRef(true);
      setPrepareLTRef(false);
      setNewestBgPOC(pocCurr);
      setLastLTRefPoc(pocCurr);
    }
    else if (m_encCfg->m_compositeRefEnabled && getLastLTRefPoc() >= 0 && getEncodedLTRef() == false &&
             !getPicBg()->getSpliceFull() && (pocCurr - getLastLTRefPoc()) > (m_encCfg->m_frameRate * 2))
    {
      setUseLTRef(false);
      setPrepareLTRef(false);
      setEncodedLTRef(true);
      setNewestBgPOC(-1);
      setLastLTRefPoc(-1);
    }

    if (m_encCfg->m_compositeRefEnabled && m_picBg->getSpliceFull() && getUseLTRef())
    {
      m_pcEncLib->selectReferencePictureList(pcSlice, pocCurr, gopId, m_bgPOC);
    }
    else
    {
      m_pcEncLib->selectReferencePictureList(pcSlice, pocCurr, gopId, -1);
    }
    if (!m_encCfg->m_efficientFieldIRAPEnabled)
    {
      if (pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
          pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
          pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA)   // IRAP picture
      {
        m_associatedIRAPType[pic->m_layerId] = pcSlice->m_eNalUnitType;
        m_associatedIRAPPOC[pic->m_layerId]  = pocCurr;
        if (m_encCfg->m_seiCfg.m_edrapIndicationSEIEnabled)
        {
          m_latestEDRAPPOC  = MAX_INT;
          pic->m_edrapRapId = 0;
        }
      }
      pcSlice->m_iAssociatedIRAPType = m_associatedIRAPType[pic->m_layerId];
      pcSlice->m_iAssociatedIRAPPOC  = m_associatedIRAPPOC[pic->m_layerId];
    }

    pcSlice->m_enableDRAPSEI = m_encCfg->m_seiCfg.m_dependentRAPIndicationSEIEnabled;
    if (m_encCfg->m_seiCfg.m_dependentRAPIndicationSEIEnabled)
    {
      // Only mark the picture as DRAP if all of the following applies:
      //  1) DRAP indication SEI messages are enabled
      //  2) The current picture is not an intra picture
      //  3) The current picture is in the DRAP period
      //  4) The current picture is a trailing picture
      pcSlice->m_isDRAP = m_encCfg->m_seiCfg.m_dependentRAPIndicationSEIEnabled && m_encCfg->m_drapPeriod > 0 &&
        !pcSlice->isIntra() && pocCurr % m_encCfg->m_drapPeriod == 0 && pocCurr > pcSlice->m_iAssociatedIRAPPOC;

      if (pcSlice->m_isDRAP)
      {
        int pocCycle = 1 << (pcSlice->m_sps->m_bitsForPoc);
        int deltaPOC = pocCurr > pcSlice->m_iAssociatedIRAPPOC
          ? pocCurr - pcSlice->m_iAssociatedIRAPPOC
          : pocCurr - (pcSlice->m_iAssociatedIRAPPOC & (pocCycle - 1));
        CHECK(deltaPOC > (pocCycle >> 1),
              "Use a greater value for POC wraparound to enable a POC distance between IRAP and DRAP of " << deltaPOC
                                                                                                          << ".");
        m_latestDRAPPOC     = pocCurr;
        pcSlice->m_uiTLayer = 0;   // Force DRAP picture to have temporal layer 0
      }
      pcSlice->m_latestDRAPPOC = m_latestDRAPPOC;
      pcSlice->m_useLTforDRAP =
        false;   // When set, sets the associated IRAP as long-term in RPL0 at slice level, unless the associated IRAP
                 // is already included in RPL0 or RPL1 defined in SPS

      PicList::iterator iterPic = rcListPic.begin();
      Picture          *pic;
      while (iterPic != rcListPic.end())
      {
        pic = *(iterPic++);
        if (pcSlice->m_isDRAP && pic->m_poc != pocCurr)
        {
          pic->m_precedingDRAP = true;
        }
        else if (!pcSlice->m_isDRAP && pic->m_poc == pocCurr)
        {
          pic->m_precedingDRAP = false;
        }
      }
    }

    pcSlice->m_enableEdrapSEI = m_encCfg->m_seiCfg.m_edrapIndicationSEIEnabled;
    if (m_encCfg->m_seiCfg.m_edrapIndicationSEIEnabled)
    {
      // Only mark the picture as Extended DRAP if all of the following applies:
      //  1) Extended DRAP indication SEI messages are enabled
      //  2) The current picture is not an intra picture
      //  3) The current picture is in the EDRAP period
      //  4) The current picture is a trailing picture
      if (m_encCfg->m_seiCfg.m_edrapIndicationSEIEnabled && m_encCfg->m_edrapPeriod > 0 && !pcSlice->isIntra() &&
          pocCurr % m_encCfg->m_edrapPeriod == 0 && pocCurr > pcSlice->m_iAssociatedIRAPPOC)
      {
        pcSlice->m_edrapRapId        = pocCurr / m_encCfg->m_edrapPeriod;
        pcSlice->m_pic->m_edrapRapId = (pocCurr / m_encCfg->m_edrapPeriod);
      }

      if (pcSlice->m_edrapRapId > 0)
      {
        m_latestEDRAPPOC                     = pocCurr;
        m_latestEdrapLeadingPicDecodableFlag = false;
        pcSlice->m_uiTLayer                  = 0;   // Force Extended DRAP picture to have temporal layer 0
        msg(NOTICE, "Force the temporal sublayer identifier of the EDRAP picture equal to 0.\n");
      }
      pcSlice->m_latestEDRAPPOC                     = m_latestEDRAPPOC;
      pcSlice->m_latestEdrapLeadingPicDecodableFlag = m_latestEdrapLeadingPicDecodableFlag;
      pcSlice->m_useLTforEdrap =
        false;   // When set, sets the associated IRAP/EDRAP as long-term in RPL0 at slice level, unless the associated
                 // IRAP/EDRAP is already included in RPL0 or RPL1 defined in SPS

      PicList::iterator iterPic = rcListPic.begin();
      Picture          *pic;
      while (iterPic != rcListPic.end())
      {
        pic = *(iterPic++);
        if (pcSlice->m_edrapRapId > 0 && pic->m_poc != pocCurr && pic->m_poc >= pcSlice->m_iAssociatedIRAPPOC)
        {
          if (pic->m_edrapRapId >= 0 && pic->m_poc % m_encCfg->m_edrapPeriod == 0)
          {
            bool refExists = false;
            for (int i = 0; i < pcSlice->m_edrapNumRefRapPics; i++)
            {
              if (pcSlice->getEdrapRefRapId(i) == pic->m_edrapRapId)
              {
                refExists = true;
              }
            }
            if (!refExists)
            {
              pcSlice->addEdrapRefRapIds(pic->m_poc / m_encCfg->m_edrapPeriod);
              pcSlice->m_edrapNumRefRapPics = pcSlice->m_edrapNumRefRapPics + 1;
            }
          }
        }
      }
    }

    if (pcSlice->checkThatAllRefPicsAreAvailable(rcListPic, &pcSlice->m_rpl[RPL0], 0, false) != 0 ||
        pcSlice->checkThatAllRefPicsAreAvailable(rcListPic, &pcSlice->m_rpl[RPL1], 1, false) != 0 ||
        (m_encCfg->m_seiCfg.m_dependentRAPIndicationSEIEnabled && !pcSlice->isIRAP() &&
         (pcSlice->m_isDRAP || !pcSlice->isPOCInRefPicList(&pcSlice->m_rpl[RPL0], pcSlice->m_iAssociatedIRAPPOC))) ||
        (m_encCfg->m_seiCfg.m_edrapIndicationSEIEnabled && !pcSlice->isIRAP() &&
         (pcSlice->m_edrapRapId > 0 ||
          !pcSlice->isPOCInRefPicList(&pcSlice->m_rpl[RPL0], pcSlice->m_iAssociatedIRAPPOC))) ||
        (((pcSlice->isIRAP() && m_encCfg->m_avoidIntraInDepLayer) ||
          (!pcSlice->isIRAP() && m_encCfg->m_rplOfDepLayerInSh)) &&
         pcSlice->m_pic->m_cs->vps &&
         m_encCfg->m_numRefLayers[pcSlice->m_pic->m_cs->vps->m_generalLayerIdx[m_pcEncLib->m_layerId]]))
    {
      xCreateExplicitReferencePictureSetFromReference(pcSlice, rcListPic, &pcSlice->m_rpl[RPL0], &pcSlice->m_rpl[RPL1]);
    }

    pcSlice->applyReferencePictureListBasedMarking(rcListPic, &pcSlice->m_rpl[RPL0], &pcSlice->m_rpl[RPL1],
                                                   pcSlice->m_pic->m_layerId, *(pcSlice->m_pps));

    if (pcSlice->m_uiTLayer > 0 && !pcSlice->isLeadingPic())
    {
      if (pcSlice->isStepwiseTemporalLayerSwitchingPointCandidate(rcListPic))
      {
        bool isSTSA = true;

        for (int ii = 0; ii < m_encCfg->m_gopSize && isSTSA; ii++)
        {
          int lTid = m_encCfg->m_RPLList0[ii].m_temporalId;
#if !ENABLE_POST_CFE_CHANGES
          if (lTid == pcSlice->m_uiTLayer)
#else
          if (lTid == pcSlice->m_uiTLayer && m_encCfg->m_RPLList0[ii].m_POC <= (pocLast % m_encCfg->m_gopSize))
#endif
          {
            for (const auto l: { RPL0, RPL1 })
            {
              const ReferencePictureList *rpl = m_encCfg->m_rplOfDepLayerInSh
                ? m_pcEncLib->getRplList(l)->getReferencePictureList(ii)
                : pcSlice->m_sps->m_rplList[l].getReferencePictureList(ii);
#if !ENABLE_POST_CFE_CHANGES
              for (int jj = 0; jj < pcSlice->m_rpl[l].m_numberOfActivePictures; jj++)
#else
              for (int jj = 0; jj < rpl->m_numberOfActivePictures && isSTSA; jj++)
#endif
              {
                // What about long-term and inter-layer?
#if !ENABLE_POST_CFE_CHANGES
                int tPoc = pcSlice->m_poc + rpl->m_refPicIdentifier[jj];
#else
                int tPoc = rpl->m_POCvalue + rpl->m_refPicIdentifier[jj];
#endif
                for (int kk = 0; kk < m_encCfg->m_gopSize; kk++)
                {
                  if (m_encCfg->m_RPLList0[kk].m_POC == tPoc)
                  {
                    int tTid = m_encCfg->m_RPLList0[kk].m_temporalId;
                    if (tTid >= pcSlice->m_uiTLayer)
                    {
                      isSTSA = false;
                      break;
                    }
                  }
                }
              }
            }
          }
        }
        if (isSTSA)
        {
          pcSlice->m_eNalUnitType = NAL_UNIT_CODED_SLICE_STSA;
        }
      }
    }

    if (m_encCfg->m_compositeRefEnabled && getUseLTRef() && (pocCurr > getLastLTRefPoc()))
    {
      pcSlice->m_numRefIdx[RPL0] = ((pcSlice->isIntra()) ? 0
                                                         : std::min(m_encCfg->m_RPLList0[gopId].m_numRefPicsActive + 1,
                                                                    pcSlice->m_rpl[RPL0].m_numberOfActivePictures));
      pcSlice->m_numRefIdx[RPL1] =
        ((!pcSlice->isInterB()) ? 0
                                : std::min(m_encCfg->m_RPLList1[gopId].m_numRefPicsActive + 1,
                                           pcSlice->m_rpl[RPL1].m_numberOfActivePictures));
    }
    else
    {
      pcSlice->m_numRefIdx[RPL0] = ((pcSlice->isIntra()) ? 0 : pcSlice->m_rpl[RPL0].m_numberOfActivePictures);
      pcSlice->m_numRefIdx[RPL1] = ((!pcSlice->isInterB()) ? 0 : pcSlice->m_rpl[RPL1].m_numberOfActivePictures);
    }
    if (m_encCfg->m_compositeRefEnabled && getPrepareLTRef())
    {
      arrangeCompositeReference(pcSlice, rcListPic, pocCurr);
    }
    //  Set reference list
    pcSlice->constructRefPicList(rcListPic);

    // store sub-picture numbers, sizes, and locations with a picture
    pcSlice->m_pic->m_subPictures.clear();

    for (int subPicIdx = 0; subPicIdx < pic->m_cs->pps->m_numSubPics; subPicIdx++)
    {
      pcSlice->m_pic->m_subPictures.push_back(pic->m_cs->pps->m_subPics[subPicIdx]);
    }

    const VPS *vps      = pic->m_cs->vps;
    int        layerIdx = vps == nullptr ? 0 : vps->m_generalLayerIdx[pic->m_layerId];
    if (vps && !vps->m_vpsIndependentLayerFlag[layerIdx] && pic->m_cs->pps->m_numSubPics > 1)
    {
      CU::checkConformanceILRP(pcSlice);
    }

    if (getUseHashMEPOCChecked())
    {
      if ((getUseHashMEPOCToCheck() != getUseHashMENextPOCToCheck()) && !getUseHashME())
      {
        // if first intra disables hashME also check second intra
        setUseHashMEPOCChecked(false);
        setUseHashMEPOCToCheck(getUseHashMENextPOCToCheck());
        setUseHashME(m_encCfg->m_HashMECfgEnable);   // initialize hashME for next intra picture
      }

      if (pic->m_poc > getUseHashMENextPOCToCheck())
      {
        if (getUseHashMEPOCToCheck() != getUseHashMENextPOCToCheck())
        {
          // now can we move the new intra poc in slot 2 to the active slot
          setUseHashMEPOCToCheck(getUseHashMENextPOCToCheck());
          setUseHashMEPOCChecked(false);
          setUseHashME(m_encCfg->m_HashMECfgEnable);   // initialize hashME for next intra picture
        }
      }
    }
    if (pcSlice->isIRAP())
    {
      // in-case the previous intra not has been checked we need to put the new intra poc in another slot
      setUseHashMENextPOCToCheck(pcSlice->m_poc);
    }
    xPicInitHashME(pic, pcSlice->m_pps, rcListPic);
    m_modeCtrl->useHashME = getUseHashME();

    xUpdateRasInit(pcSlice);

    picHeader->setMaxBTSizes(pcSlice->m_sps->m_maxBTSize);
    picHeader->setMaxTTSizes(pcSlice->m_sps->m_maxTTSize);

    if (m_encCfg->m_useAMaxBT)
    {
      const SliceType sliceType = pcSlice->m_eSliceType;
      const SPS      *sps       = pcSlice->m_sps;

      if (pcSlice->m_pendingRasInit)
      {
        m_blkStat.fill({ 0, 0 });
      }

      if (!pcSlice->isIRAP())
      {
        const int hierPredLayerIdx = std::min<int>(pcSlice->m_hierPredLayerIdx, (int)m_blkStat.size() - 1);

        if (hierPredLayerIdx >= 0 && m_blkStat[hierPredLayerIdx].count != 0)
        {
          picHeader->m_splitConsOverrideFlag = true;

          const double avgBlkSize = (double)m_blkStat[hierPredLayerIdx].area / m_blkStat[hierPredLayerIdx].count;

          unsigned newMaxBttSize = m_encCfg->m_CTUSize;
          if (avgBlkSize < AMAXBT_TH32 * AMAXBT_TH32)
          {
            newMaxBttSize = 32;
          }
          else if (avgBlkSize < AMAXBT_TH64 * AMAXBT_TH64 && m_encCfg->m_CTUSize >= 64)
          {
            newMaxBttSize = 64;
          }
          else if (avgBlkSize < AMAXBT_TH128 * AMAXBT_TH128 && m_encCfg->m_CTUSize >= 128)
          {
            newMaxBttSize = 128;
          }
          else if (m_encCfg->m_CTUSize >= 256)
          {
            newMaxBttSize = 256;
          }

#if ENABLE_POST_CFE_CHANGES
          picHeader->setMaxBTSize(1, Clip3(picHeader->getMinQTSize(sliceType), m_encCfg->m_maxBt[1], newMaxBttSize));
          picHeader->setMaxTTSize(1, Clip3(picHeader->getMinQTSize(sliceType), m_encCfg->m_maxTt[1], newMaxBttSize));
#else
          newMaxBttSize = Clip3(picHeader->getMinQTSize(sliceType), sps->m_ctuSize, newMaxBttSize);
          picHeader->setMaxBTSize(1, newMaxBttSize);
#endif
          m_blkStat[hierPredLayerIdx] = { 0, 0 };
        }
      }

      bool identicalToSps = true;

      if (identicalToSps && picHeader->m_picInterSliceAllowedFlag)
      {
        identicalToSps = picHeader->getMinQTSize(sliceType) == sps->getMinQTSize(sliceType) &&
          picHeader->getMaxMTTHierarchyDepth(sliceType) == sps->getMaxMTTHierarchyDepth() &&
          picHeader->getMaxBTSize(sliceType) == sps->getMaxBTSize() &&
          picHeader->getMaxTTSize(sliceType) == sps->getMaxTTSize();
      }

      if (identicalToSps && picHeader->m_picIntraSliceAllowedFlag)
      {
        identicalToSps = picHeader->getMinQTSize(I_SLICE) == sps->getMinQTSize(I_SLICE) &&
          picHeader->getMaxMTTHierarchyDepth(I_SLICE) == sps->getMaxMTTHierarchyDepthI() &&
          picHeader->getMaxBTSize(I_SLICE) == sps->getMaxBTSizeI() &&
          picHeader->getMaxTTSize(I_SLICE) == sps->getMaxTTSizeI();

        if (identicalToSps && sps->m_dualITree)
        {
          identicalToSps =
            picHeader->getMinQTSize(I_SLICE, ChannelType::CHROMA) == sps->getMinQTSize(I_SLICE, ChannelType::CHROMA) &&
            picHeader->getMaxMTTHierarchyDepth(I_SLICE, ChannelType::CHROMA) == sps->getMaxMTTHierarchyDepthIChroma() &&
            picHeader->getMaxBTSize(I_SLICE, ChannelType::CHROMA) == sps->getMaxBTSizeIChroma() &&
            picHeader->getMaxTTSize(I_SLICE, ChannelType::CHROMA) == sps->getMaxTTSizeIChroma();
        }
      }

      if (identicalToSps)
      {
        picHeader->m_splitConsOverrideFlag = false;
      }
    }

    if (m_encCfg->m_uiMaxMTTHierarchyDepth >= 10)
    {
      const int numLayers = floorLog2(m_encCfg->m_gopSize);
      picHeader->m_maxMTTHierarchyDepth[1] =
        int(m_encCfg->m_uiMaxMTTHierarchyDepth / pow(10, numLayers - pcSlice->m_hierPredLayerIdx)) % 10;
      picHeader->m_splitConsOverrideFlag |=
        pcSlice->m_sps->getMaxMTTHierarchyDepth() != picHeader->m_maxMTTHierarchyDepth[1];
    }

    //  Slice info. refinement
    if ((pcSlice->m_eSliceType == B_SLICE) && (pcSlice->m_numRefIdx[RPL1] == 0))
    {
      pcSlice->m_eSliceType = P_SLICE;
#if ENABLE_CABAC_DUMP
      pcSlice->m_cabacInitSliceType = P_SLICE;
#endif
    }

    if (pcSlice->m_pendingRasInit || pcSlice->isIRAP())
    {
      // this ensures that independently encoded bitstream chunks can be combined to bit-equal
      pcSlice->m_encCABACTableIdx = pcSlice->m_eSliceType;
    }
    else
    {
      pcSlice->m_encCABACTableIdx = m_pcSliceEncoder->getEncCABACTableIdx();
    }

    if (pcSlice->m_eSliceType == B_SLICE)
    {
      bool lowDelay = true;
      int  currPoc  = pcSlice->m_poc;
      int  refIdx   = 0;

      for (refIdx = 0; refIdx < pcSlice->m_numRefIdx[RPL0] && lowDelay; refIdx++)
      {
        if (pcSlice->getRefPic(RPL0, refIdx)->m_poc > currPoc)
        {
          lowDelay = false;
        }
      }
      for (refIdx = 0; refIdx < pcSlice->m_numRefIdx[RPL1] && lowDelay; refIdx++)
      {
        if (pcSlice->getRefPic(RPL1, refIdx)->m_poc > currPoc)
        {
          lowDelay = false;
        }
      }

      pcSlice->m_checkLdc = lowDelay;
    }
    else
    {
      pcSlice->m_checkLdc = true;
    }

    //-------------------------------------------------------------
    pcSlice->setRefPOCList();

    pcSlice->setList1IdxToList0Idx();

    switch (m_encCfg->m_TMVPModeId)
    {
    case 2:
      // disable TMVP for first picture in SOP (i.e. forward B)
      // Note: pcSlice->m_colFromL0Flag is assumed to be always 0 and m_colRefIdx is always 0.
      picHeader->m_enableTMVPFlag = gopId != 0;
      break;
    case 1:
      picHeader->m_enableTMVPFlag = true;
      break;
    default:
      picHeader->m_enableTMVPFlag = false;
      break;
    }

    // disable TMVP when current picture is the only ref picture
    if (pcSlice->isIRAP() && pcSlice->m_ibcFlag)
    {
      picHeader->m_enableTMVPFlag = false;
    }

    if (picHeader->m_enableTMVPFlag)
    {
      pcSlice->setRefRefIdxList();
    }
    if (pcSlice->m_eSliceType != I_SLICE && picHeader->m_enableTMVPFlag)
    {
      int colRefIdxL0 = -1, colRefIdxL1 = -1;

      for (int refIdx = 0; refIdx < pcSlice->m_numRefIdx[RPL0]; refIdx++)
      {
        CHECK(pcSlice->getRefPic(RPL0, refIdx)->m_unscaledPic == nullptr,
              "m_unscaledPic is not set for L0 reference picture");

        if (pcSlice->getRefPic(RPL0, refIdx)->isRefScaled(pcSlice->m_pps) == false)
        {
          colRefIdxL0 = refIdx;
          break;
        }
      }

      if (pcSlice->m_eSliceType == B_SLICE)
      {
        for (int refIdx = 0; refIdx < pcSlice->m_numRefIdx[RPL1]; refIdx++)
        {
          CHECK(pcSlice->getRefPic(RPL1, refIdx)->m_unscaledPic == nullptr,
                "m_unscaledPic is not set for L1 reference picture");

          if (pcSlice->getRefPic(RPL1, refIdx)->isRefScaled(pcSlice->m_pps) == false)
          {
            colRefIdxL1 = refIdx;
            break;
          }
        }
      }

      if (colRefIdxL0 >= 0 && colRefIdxL1 >= 0)
      {
        const Picture *refPicL0 = pcSlice->getRefPic(RPL0, colRefIdxL0);
        if (!refPicL0->m_slices.size())
        {
          refPicL0 = refPicL0->m_unscaledPic;
        }

        const Picture *refPicL1 = pcSlice->getRefPic(RPL1, colRefIdxL1);
        if (!refPicL1->m_slices.size())
        {
          refPicL1 = refPicL1->m_unscaledPic;
        }

        CHECK(!refPicL0->m_slices.size(), "Wrong L0 reference picture");
        CHECK(!refPicL1->m_slices.size(), "Wrong L1 reference picture");

        const uint32_t uiColFromL0    = refPicL0->m_slices[0]->m_iSliceQp > refPicL1->m_slices[0]->m_iSliceQp;
        picHeader->m_picColFromL0Flag = uiColFromL0;
        pcSlice->m_colFromL0Flag      = uiColFromL0;
        pcSlice->m_colRefIdx          = (uiColFromL0 ? colRefIdxL0 : colRefIdxL1);
        picHeader->m_colRefIdx        = (uiColFromL0 ? colRefIdxL0 : colRefIdxL1);
      }
      else if (colRefIdxL0 < 0 && colRefIdxL1 >= 0)
      {
        picHeader->m_picColFromL0Flag = false;
        pcSlice->m_colFromL0Flag      = false;
        pcSlice->m_colRefIdx          = colRefIdxL1;
        picHeader->m_colRefIdx        = colRefIdxL1;
      }
      else if (colRefIdxL0 >= 0 && colRefIdxL1 < 0)
      {
        picHeader->m_picColFromL0Flag = true;
        pcSlice->m_colFromL0Flag      = true;
        pcSlice->m_colRefIdx          = colRefIdxL0;
        picHeader->m_colRefIdx        = colRefIdxL0;
      }
      else
      {
        picHeader->m_enableTMVPFlag = false;
      }
    }

    if (!pcSlice->m_sps->m_useAffine)
    {
      picHeader->m_maxNumAffineMergeCand = (pcSlice->m_sps->m_sbtmvpEnabledFlag && picHeader->m_enableTMVPFlag) ? 1 : 0;
    }

    bool bDisableTMVP = pcSlice->scaleRefPicList(scaledRefPic, pic->m_cs->picHeader, m_pcEncLib->getApss(),
                                                 picHeader->m_lmcsAps, picHeader->m_scalingListAps, false);
    if (pic->m_cs->picHeader->m_enableTMVPFlag && bDisableTMVP)
    {
      pic->m_cs->picHeader->m_enableTMVPFlag = false;
    }

    // set adaptive search range for non-intra-slices
    if (m_encCfg->m_bUseASR && !pcSlice->isIntra())
    {
      m_pcSliceEncoder->setSearchRange(pcSlice);
    }

    bool identicalListsInSliceB = false;
    if (pcSlice->m_eSliceType == B_SLICE)
    {
      if (pcSlice->m_numRefIdx[RPL0] == pcSlice->m_numRefIdx[RPL1])
      {
        identicalListsInSliceB = true;
        for (int i = 0; i < pcSlice->m_numRefIdx[RPL1]; i++)
        {
          if (pcSlice->getRefPOC(RPL1, i) != pcSlice->getRefPOC(RPL0, i))
          {
            identicalListsInSliceB = false;
            break;
          }
        }
      }
    }
    picHeader->m_mvdL1ZeroFlag = identicalListsInSliceB;

    pcSlice->m_meetBiPredT = false;
    if (pcSlice->m_sps->m_useSMVD && !pcSlice->m_checkLdc && !picHeader->m_mvdL1ZeroFlag)
    {
      int currPOC = pcSlice->m_poc;

      int forwardPOC  = currPOC;
      int backwardPOC = currPOC;
      int refIdx0 = -1, refIdx1 = -1;

      // search nearest forward POC in List 0
      for (int ref = 0; ref < pcSlice->m_numRefIdx[RPL0]; ref++)
      {
        int        poc           = pcSlice->getRefPic(RPL0, ref)->m_poc;
        const bool isRefLongTerm = pcSlice->getRefPic(RPL0, ref)->m_longTerm;
        if (poc < currPOC && (poc > forwardPOC || refIdx0 == -1) && !isRefLongTerm)
        {
          forwardPOC = poc;
          refIdx0    = ref;
        }
      }

      // search nearest backward POC in List 1
      for (int ref = 0; ref < pcSlice->m_numRefIdx[RPL1]; ref++)
      {
        int        poc           = pcSlice->getRefPic(RPL1, ref)->m_poc;
        const bool isRefLongTerm = pcSlice->getRefPic(RPL1, ref)->m_longTerm;
        if (poc > currPOC && (poc < backwardPOC || refIdx1 == -1) && !isRefLongTerm)
        {
          backwardPOC = poc;
          refIdx1     = ref;
        }
      }

      if (!(forwardPOC < currPOC && backwardPOC > currPOC))
      {
        forwardPOC  = currPOC;
        backwardPOC = currPOC;
        refIdx0     = -1;
        refIdx1     = -1;

        // search nearest backward POC in List 0
        for (int ref = 0; ref < pcSlice->m_numRefIdx[RPL0]; ref++)
        {
          int        poc           = pcSlice->getRefPic(RPL0, ref)->m_poc;
          const bool isRefLongTerm = pcSlice->getRefPic(RPL0, ref)->m_longTerm;
          if (poc > currPOC && (poc < backwardPOC || refIdx0 == -1) && !isRefLongTerm)
          {
            backwardPOC = poc;
            refIdx0     = ref;
          }
        }

        // search nearest forward POC in List 1
        for (int ref = 0; ref < pcSlice->m_numRefIdx[RPL1]; ref++)
        {
          int        poc           = pcSlice->getRefPic(RPL1, ref)->m_poc;
          const bool isRefLongTerm = pcSlice->getRefPic(RPL1, ref)->m_longTerm;
          if (poc < currPOC && (poc > forwardPOC || refIdx1 == -1) && !isRefLongTerm)
          {
            forwardPOC = poc;
            refIdx1    = ref;
          }
        }
      }

      if (forwardPOC < currPOC && backwardPOC > currPOC)
      {
        pcSlice->setBiDirPred(true, refIdx0, refIdx1);
        constexpr int affineMeTBiPred = 1;
        pcSlice->m_meetBiPredT        = abs(forwardPOC - currPOC) <= affineMeTBiPred;
      }
      else
      {
        pcSlice->setBiDirPred(false, -1, -1);
      }
    }
    else
    {
      pcSlice->setBiDirPred(false, -1, -1);
    }

    if (pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_RASL && m_encCfg->m_rprRASLtoolSwitch)
    {
      pcSlice->m_lmChromaCheckDisable = true;
      xUpdateRPRtmvp(picHeader, pcSlice);
      CHECK(pcSlice->m_pps->m_wrapAroundEnabledFlag,
            "pps_ref_wraparound_enabled_flag should be 0 with constrained RASL encoding");
    }

    double lambda               = 0.0;
    int    actualHeadBits       = 0;
    int    actualTotalBits      = 0;
    int    estimatedBits        = 0;
    int    tmpBitsBeforeWriting = 0;

    xPicInitRateControl(estimatedBits, gopId, lambda, pic, pcSlice);

    uint32_t numSliceSegments = 1;

    pcSlice->setDefaultClpRng(*pcSlice->m_sps);

    // Allocate some coders, now the number of tiles are known.
    const uint32_t numberOfCtusInFrame  = pic->m_cs->pcv->sizeInCtus;
    const int      numSubstreamsColumns = pcSlice->m_pps->m_numTileCols;
    const int      numSubstreamRows =
      pcSlice->m_sps->m_entropyCodingSyncEnabledFlag ? pic->m_cs->pcv->heightInCtus : (pcSlice->m_pps->m_numTileRows);
    const int numSubstreams =
      std::max<int>(numSubstreamRows * numSubstreamsColumns, (int)pic->m_cs->pps->m_numSlicesInPic);
    std::vector<OutputBitstream> substreamsOut(numSubstreams);

#if ENABLE_QPA
    pic->m_uEnerHpCtu.resize(numberOfCtusInFrame);
    pic->m_iOffsetCtu.resize(numberOfCtusInFrame);
#if ENABLE_QPA_SUB_CTU
    if (pcSlice->m_pps->m_useDQP && pcSlice->getCuQpDeltaSubdiv() > 0)
    {
      const PreCalcValues &pcv     = *pic->m_cs->pcv;
      const unsigned       mtsLog2 = (unsigned)floorLog2(std::min(pic->m_cs->sps->getMaxTbSize(), pcv.maxCUWidth));
      pic->m_subCtuQP.resize((pcv.maxCUWidth >> mtsLog2) * (pcv.maxCUHeight >> mtsLog2));
    }
#endif
#endif

#if ENABLE_NNLF
    if (pcSlice->m_sps->m_nnlf)
    {
      pic->initPicprms(*pcSlice);
    }
#endif

    if (pcSlice->m_sps->m_saoEnabledFlag || pcSlice->m_pps->m_BIF || pcSlice->m_pps->m_chromaBIF)
    {
      pic->resizeSAO(numberOfCtusInFrame, 0);
      pic->resizeSAO(numberOfCtusInFrame, 1);
      pic->resizeBIF((CompID)0, numberOfCtusInFrame);
      pic->resizeBIF((CompID)1, numberOfCtusInFrame);
      pic->resizeBIF((CompID)2, numberOfCtusInFrame);
    }

    // it is used for signalling during CTU mode decision, i.e. before ALF processing
    if (pcSlice->m_sps->m_alfEnabledFlag)
    {
      pic->resizeAlfData(numberOfCtusInFrame);
    }

    bool decPic = false;
    bool encPic = false;
    // test if we can skip the picture entirely or decode instead of encoding
    trySkipOrDecodePicture(decPic, encPic, m_encCfg, pic, m_pcEncLib->getApsMaps());

    pic->m_cs->slice = pcSlice;   // please keep this
#if ENABLE_QPA
    if (pcSlice->m_pps->m_sliceChromaQpFlag && CS::isDualITree(*pcSlice->m_pic->m_cs) && !m_encCfg->m_bUsePerceptQPA &&
        (m_encCfg->m_sliceChromaQpOffsetPeriodicity == 0))
#else
    if (pcSlice->m_pps->m_sliceChromaQpFlag && CS::isDualITree(*pcSlice->m_pic->m_cs))
#endif
    {
      bool isRprPPS = false;
      for (int nr = 0; nr < NUM_RPR_PPS; nr++)
      {
        if ((pcSlice->m_pps->m_ppsId == RPR_PPS_ID[nr]) && (RPR_PPS_ID[nr] != 0))
        {
          isRprPPS = true;
        }
      }
      if (!isRprPPS)
      {
        // overwrite chroma qp offset for dual tree
        pcSlice->setSliceChromaQpDelta(COMP_Cb, m_encCfg->m_chromaCbQpOffsetDualTree);
        pcSlice->setSliceChromaQpDelta(COMP_Cr, m_encCfg->m_chromaCrQpOffsetDualTree);
        if (pcSlice->m_sps->m_jointCbCrEnabledFlag)
        {
          pcSlice->setSliceChromaQpDelta(JOINT_CbCr, m_encCfg->m_chromaCbCrQpOffsetDualTree);
        }
        m_pcSliceEncoder->setUpLambda(pcSlice, pcSlice->getLambdas()[0], pcSlice->m_iSliceQp);
      }
    }

    xPicInitLMCS(pic, picHeader, pcSlice);

    if (m_encCfg->m_intraPeriod == -1)
    {
      if (pic->m_cs->slice->isIntra())
      {
        pcSlice->m_lumaPelMax = (1 << pic->m_cs->sps->m_bitDepths[ChannelType::LUMA]) - 1;
        pcSlice->m_lumaPelMin = 0;
      }
    }

    if (pcSlice->m_sps->m_scalingListEnabledFlag && m_encCfg->m_useScalingListId == SCALING_LIST_FILE_READ)
    {
      picHeader->m_explicitScalingListEnabledFlag = true;
      pcSlice->m_explicitScalingListUsed          = true;

      const int apsId = std::min<int>(
        7, m_pcEncLib->m_vps == nullptr ? 0 : m_pcEncLib->m_vps->m_generalLayerIdx[m_pcEncLib->m_layerId]);
      picHeader->m_scalingListApsId = apsId;

      ParameterSetMap<APS> *apsMap         = m_pcEncLib->getApsMap(ApsType::SCALING_LIST);
      APS                  *scalingListAPS = apsMap->getPS(apsId);
      assert(scalingListAPS != nullptr);
      picHeader->m_scalingListAps   = scalingListAPS;
      picHeader->m_scalingListApsId = scalingListAPS->m_APSId;
    }

    pic->m_cs->picHeader->m_pic   = pic;
    pic->m_cs->picHeader->m_valid = true;
    if (pic->m_cs->sps->m_fpelMmvdEnabledFlag)
    {
      // cannot set ph_fpel_mmvd_enabled_flag at slice level - need new picture-level version of checkDisFracMmvd
      // algorithm? m_pcSliceEncoder->checkDisFracMmvd( pic, 0, numberOfCtusInFrame );
      const bool useIntegerMVD            = (pic->lwidth() * pic->lheight() > 1920 * 1080);
      pic->m_cs->picHeader->m_disFracMMVD = useIntegerMVD;
    }
    if (pcSlice->m_sps->m_jointCbCrEnabledFlag)
    {
      if (m_encCfg->m_constantJointCbCrSignFlag)
      {
        pic->m_cs->picHeader->m_jointCbCrSignFlag = false;
      }
      else
      {
        m_pcSliceEncoder->setJointCbCrModes(*pic->m_cs, Position(0, 0), pic->m_cs->area.lumaSize());
      }
    }
    if (!pcSlice->m_sps->m_spsRangeExtension.m_reverseLastSigCoeffEnabledFlag || pcSlice->m_iSliceQp > 12)
    {
      pcSlice->m_reverseLastSigCoeffFlag = false;
    }
    else
    {
      /*for RA serial and parallel alignment start*/
      if (m_encCfg->m_intraPeriod > 1)
      {
        if (pcSlice->isIntra())
        {
          m_cntRightBottom = 0;
        }
        if ((pocCurr % m_encCfg->m_intraPeriod) <= m_encCfg->m_gopSize && gopId == 0 && !pcSlice->isIntra())
        {
          m_cntRightBottom = m_cntRightBottomIntra;
        }
      }
      /*for RA serial and parallel alignment end*/
      pcSlice->m_reverseLastSigCoeffFlag = (m_cntRightBottom >= 0);
    }

    CHECKD((!pic->m_cs->sps->m_alfEnabledFlag) != (!m_encCfg->m_alf),
           "ALF flag mismatch between encoder config and SPS.");
    CHECKD((!pic->m_cs->sps->m_alfImprovementsEnabledFlag) != (!m_encCfg->m_alfImprovements),
           "ALF improvements flag mismatch between encoder config and SPS.");

    if (encPic)
    // now compress (trial encode) the various slice segments (slices, and dependent slices)
    {
      DTRACE_UPDATE(g_trace_ctx, (std::make_pair("poc", pocCurr)));
      const std::vector<uint16_t> sliceLosslessArray = m_encCfg->m_sliceLosslessArray;
      bool                        mixedLossyLossless = m_encCfg->m_mixedLossyLossless;
      if (m_encCfg->m_costMode == COST_LOSSLESS_CODING)
      {
        pic->fillSliceLossyLosslessArray(sliceLosslessArray, mixedLossyLossless);
      }

      for (uint32_t sliceIdx = 0; sliceIdx < pic->m_cs->pps->m_numSlicesInPic; sliceIdx++)
      {
        pcSlice->setSliceMap(pic->m_cs->pps->getSliceMap(sliceIdx));
        if (pcSlice->m_sps->m_spsRangeExtension.m_tsrcRicePresentFlag && (pic->m_cs->pps->m_numSlicesInPic == 1))
        {
          if (!pcSlice->isIntra())
          {
            int nextRice = 1;

            if (m_preIPOC < pocCurr)
            {
              for (int idx = 0; idx < MAX_TSRC_RICE; idx++)
              {
                m_riceBit[idx][0] = m_riceBit[idx][1];
              }
              m_preQP[0] = m_preQP[1];
              m_preIPOC  = MAX_INT;
            }

            if (m_preQP[0] != pcSlice->m_iSliceQp)
            {
              m_riceBit[pcSlice->m_tsrcIndex][0] = (int)(m_riceBit[pcSlice->m_tsrcIndex][0] * 9 / 10);
            }

            for (int idx = 2; idx < 9; idx++)
            {
              if (m_riceBit[idx - 2][0] > m_riceBit[idx - 1][0])
              {
                nextRice = idx;
              }
              else
              {
                m_riceBit[idx - 1][0] = m_riceBit[idx - 2][0];
              }
              m_riceBit[idx - 2][0] = 0;
            }
            m_riceBit[7][0]      = 0;
            pcSlice->m_tsrcIndex = nextRice - 1;
          }
          else
          {
            m_preIPOC  = pocCurr;
            m_preQP[0] = MAX_INT;
            m_preQP[1] = pcSlice->m_iSliceQp;
            for (int idx = 0; idx < MAX_TSRC_RICE; idx++)
            {
              m_riceBit[idx][0] = 0;
            }
          }
          for (int idx = 0; idx < MAX_TSRC_RICE; idx++)
          {
            pcSlice->m_riceBit[idx] = m_riceBit[idx][0];
          }
        }
        if (pic->m_cs->pps->m_rectSliceFlag)
        {
          Position firstCtu;
          firstCtu.x    = pcSlice->getFirstCtuRsAddrInSlice() % pic->m_cs->pps->m_picWidthInCtu;
          firstCtu.y    = pcSlice->getFirstCtuRsAddrInSlice() / pic->m_cs->pps->m_picWidthInCtu;
          int subPicIdx = NOT_VALID;
          for (int sp = 0; sp < pic->m_cs->pps->m_numSubPics; sp++)
          {
            if (pic->m_cs->pps->m_subPics[sp].containsCtu(firstCtu))
            {
              subPicIdx = sp;
              break;
            }
          }
          CHECK(subPicIdx == NOT_VALID, "Sub-picture was not found");

          pcSlice->m_sliceSubPicId = pic->m_cs->pps->m_subPics[subPicIdx].m_subPicID;
        }
        if (pic->m_cs->sps->m_lmcsEnabled)
        {
          pcSlice->m_lmcsEnabledFlag = picHeader->m_lmcsEnabledFlag;
          if (pcSlice->m_eSliceType == I_SLICE)
          {
            // reshape original signal
            if (m_encCfg->m_gopBasedTemporalFilterEnabled)
            {
              pic->getOrigBuf().copyFrom(pic->getFilteredOrigBuf());
            }
            else
            {
              pic->getOrigBuf().copyFrom(pic->getTrueOrigBuf());
            }

            if (pcSlice->m_lmcsEnabledFlag)
            {
              pic->getOrigBuf(COMP_Y).rspSignal(m_pcReshaper->m_fwdLUT);
              m_pcReshaper->setSrcReshaped(true);
              m_pcReshaper->m_recReshaped = true;
            }
            else
            {
              m_pcReshaper->setSrcReshaped(false);
              m_pcReshaper->m_recReshaped = false;
            }
          }
        }

        bool isLossless = false;
        if (m_encCfg->m_costMode == COST_LOSSLESS_CODING)
        {
          isLossless = pic->losslessSlice(sliceIdx);
        }
        m_pcSliceEncoder->setLosslessSlice(pic, isLossless);

        if (pcSlice->m_eSliceType != I_SLICE && pcSlice->getRefPic(RPL0, 0)->m_subPictures.size() > 1)
        {
          clipMv = clipMvInSubpic;
          m_pcEncLib->getInterSearch()->setClipMvInSubPic(true);
        }
        else
        {
          clipMv = clipMvInPic;
          m_pcEncLib->getInterSearch()->setClipMvInSubPic(false);
        }

        if (pcSlice->isIntra() && (pocLast == 0 || m_encCfg->m_intraPeriod > 1))
        {
          computeSignalling(pic, pcSlice);
        }
        m_pcSliceEncoder->precompressSlice(pic);
        m_pcSliceEncoder->compressSlice(pic, false, false);

        if (sliceIdx < pic->m_cs->pps->m_numSlicesInPic - 1)
        {
          uint32_t independentSliceIdx = pcSlice->m_independentSliceIdx;
          pic->allocateNewSlice();
          m_pcSliceEncoder->setSliceSegmentIdx(numSliceSegments);
          // prepare for next slice
          pcSlice = pic->m_slices[numSliceSegments];
          CHECK(!(pcSlice->m_pps != 0), "Unspecified error");
          pcSlice->copySliceInfo(pic->m_slices[numSliceSegments - 1]);
          independentSliceIdx++;
          pcSlice->m_independentSliceIdx = independentSliceIdx;
          numSliceSegments++;
        }
      }
      duData.clear();

      CodingStructure &cs = *pic->m_cs;
      pcSlice             = pic->m_slices[0];

      if (cs.sps->m_lmcsEnabled && m_pcReshaper->m_sliceReshapeInfo.sliceReshaperEnableFlag)
      {
        picHeader->m_lmcsEnabledFlag = true;
        int apsId                    = std::min<int>(
          3, m_pcEncLib->m_vps == nullptr ? 0 : m_pcEncLib->m_vps->m_generalLayerIdx[m_pcEncLib->m_layerId]);
        picHeader->m_lmcsApsId = apsId;

        const PreCalcValues &pcv = *cs.pcv;
        for (uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
        {
          for (uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
          {
            const CodingUnit *cu = cs.getCU(Position(xPos, yPos), ChannelType::LUMA);
            if (cu->slice->m_lmcsEnabledFlag)
            {
              const uint32_t width = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
              const uint32_t height =
                (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
              const UnitArea area(cs.area.chromaFormat, Area(xPos, yPos, width, height));
              cs.getRecoBuf(area).get(COMP_Y).rspSignal(m_pcReshaper->m_invLUT);
            }
          }
        }

#if ENABLE_NNLF
        if (cs.sps->m_nnlfStore)
        {
          uint64_t culength = cs.cus.size();
          for (uint64_t n = 0; n < culength; n++)
          {
            CodingUnit *cu = cs.cus.at(n);
            if (cu->slice->m_lmcsEnabledFlag)
            {
              if (((cu->predMode == MODE_INTRA || cu->predMode == MODE_IBC) && cu->chType != ChannelType::CHROMA) ||
                  (cu->predMode == MODE_INTER && m_pcReshaper->m_ctuFlag && cu->ciipFlag))
              {
                pic->getPredBufCustom(cu->block(CompID::COMP_Y)).rspSignal(m_pcReshaper->m_invLUT);
              }
            }
          }
        }
#endif

        m_pcReshaper->m_recReshaped = false;

        if (m_encCfg->m_gopBasedTemporalFilterEnabled)
        {
          pic->getOrigBuf().copyFrom(pic->getFilteredOrigBuf());
        }
        else
        {
          pic->getOrigBuf().copyFrom(pic->getTrueOrigBuf());
        }
      }

      if (m_encCfg->m_alf && m_encCfg->m_alfImprovements)
      {
        CHECK(m_alfEcm == nullptr, "ECM ALF encoder is not set yet.");
        m_alfEcm->destroy();
        m_alfEcm->create(m_encCfg, picWidth, picHeight, chromaFormatIdc, maxCUWidth, maxCUHeight, maxTotalCUDepth,
                         m_encCfg->m_internalBitDepth);
        m_alfEcm->copyAddInputsBeforeDBF(cs, m_pcEncLib->m_cTrQuant);
      }

      // create SAO object based on the picture size
      if (pcSlice->m_sps->m_saoEnabledFlag || pcSlice->m_sps->m_ccSaoEnabledFlag || pcSlice->m_pps->m_BIF ||
          pcSlice->m_pps->m_chromaBIF)
      {
        const uint32_t widthInCtus   = (picWidth + maxCUWidth - 1) / maxCUWidth;
        const uint32_t heightInCtus  = (picHeight + maxCUHeight - 1) / maxCUHeight;
        const uint32_t numCtuInFrame = widthInCtus * heightInCtus;
        const uint32_t log2SaoOffsetScaleLuma =
          (uint32_t)std::max(0, pcSlice->m_sps->m_bitDepths[ChannelType::LUMA] - MAX_SAO_TRUNCATED_BITDEPTH);
        const uint32_t log2SaoOffsetScaleChroma =
          (uint32_t)std::max(0, pcSlice->m_sps->m_bitDepths[ChannelType::CHROMA] - MAX_SAO_TRUNCATED_BITDEPTH);
        m_pcSAO->destroy();
        m_pcSAO->create(picWidth, picHeight, chromaFormatIdc, maxCUWidth, maxCUHeight, maxTotalCUDepth,
                        log2SaoOffsetScaleLuma, log2SaoOffsetScaleChroma);
        m_pcSAO->destroyEncData();
        m_pcSAO->createEncData(m_encCfg->m_saoCtuBoundary, numCtuInFrame);
        m_pcSAO->setReshaper(m_pcReshaper);
      }

      if (pcSlice->m_sps->m_scalingListEnabledFlag && m_encCfg->m_useScalingListId == SCALING_LIST_FILE_READ)
      {
        picHeader->m_explicitScalingListEnabledFlag = true;
        pcSlice->m_explicitScalingListUsed          = true;
        const int apsId                             = 0;
        picHeader->m_scalingListApsId               = apsId;
      }

      // SAO parameter estimation using non-deblocked pixels for CTU bottom and right boundary areas
      if (pcSlice->m_sps->m_saoEnabledFlag && m_encCfg->m_saoCtuBoundary)
      {
        m_pcSAO->getPreDBFStatistics(cs, m_encCfg->m_saoTrueOrg);
      }

      //-- Loop filter
      if (m_encCfg->m_deblockingFilterMetric)
      {
        if (m_encCfg->m_deblockingFilterMetric == 2)
        {
          applyDeblockingFilterParameterSelection(pic, numSliceSegments, gopId);
        }
        else
        {
          applyDeblockingFilterMetric(pic);
        }
      }
      if (m_encCfg->m_costMode == COST_LOSSLESS_CODING)
      {
        for (int s = 0; s < numSliceSegments; s++)
        {
          if (pic->m_slices[s]->m_isLossless)
          {
            pic->m_slices[s]->m_deblockingFilterDisable = true;
          }
        }
      }

#if ENABLE_NNLF
      if (pcSlice->m_sps->m_nnlfStore)
      {
        pic->getBsMapBuf().fill(0);
        pic->dumpPicBpmInfo();
        pic->dumpQpBlock();
        pic->getRecBeforeDbfBuf().copyFrom(pic->getRecoBuf());
      }
#endif

      m_pcLoopFilter->deblockingFilterPic(cs);
      if (cs.sps->m_ccSaoEnabledFlag)
      {
        m_pcSAO->getCcSaoBuf().copyFrom(cs.getRecoBuf());
      }

#if ENABLE_NNLF
      if (pcSlice->m_sps->m_nnlf && !picHeader->m_nnlfDisabled)
      {
        pic->paddingBsMapBufBorder(NNLF_UNIFIED_INFER_SIZE_EXT);
        pic->paddingRecBeforeDbfBufBorder(NNLF_UNIFIED_INFER_SIZE_EXT);
        pic->paddingPredBufBorder(NNLF_UNIFIED_INFER_SIZE_EXT);
        pic->paddingBPMBufBorder(NNLF_UNIFIED_INFER_SIZE_EXT);
        pic->paddingBlockQPBufBorder(NNLF_UNIFIED_INFER_SIZE_EXT, pcSlice->m_iSliceQp);

        const bool transInput =
          pcSlice->m_sps->m_nnlf == NNLFUnifiedID::VLOP || pcSlice->m_sps->m_nnlf == NNLFUnifiedID::LOP;
        m_unifiedNnlf->initCabac(m_pcEncLib->getCABACEncoder(), m_pcEncLib->getCtxCache(), *pcSlice);
        m_unifiedNnlf->setNnlfParams(m_encCfg->m_nnlfDebugOption == 2, transInput);
        m_unifiedNnlf->setPicprms(&pic->m_picprm);
        m_unifiedNnlf->chooseParameters(*pic);
        pcSlice->m_nnlfUnifiedParam = m_unifiedNnlf->getSliceprms();
      }
#endif

      if (pcSlice->m_sps->m_saoEnabledFlag || pcSlice->m_pps->m_BIF || pcSlice->m_pps->m_chromaBIF)
      {
        bool sliceEnabled[MAX_NUM_COMP];
        m_pcSAO->initCABACEstimator(m_pcEncLib->getCABACEncoder(), m_pcEncLib->getCtxCache(), pcSlice);
        BIFCabacEstImp est(m_pcEncLib->getCABACEncoder()->getCABACEstimator(cs.slice->m_sps));
        m_pcSAO->SAOProcess(cs, sliceEnabled, pcSlice->getLambdas(),
#if ENABLE_QPA
                            (m_encCfg->m_bUsePerceptQPA && !m_encCfg->m_RCEnableRateControl && pcSlice->m_pps->m_useDQP
                               ? m_pcEncLib->getRdCost()->getChromaWeight()
                               : 0.0),
#endif
                            m_encCfg->m_bTestSAODisableAtPictureLevel, m_encCfg->m_saoEncodingRate,
                            m_encCfg->m_saoEncodingRateChroma, m_encCfg->m_saoCtuBoundary,
                            m_encCfg->m_saoGreedyMergeEnc, m_encCfg->m_saoTrueOrg, &est);
        // assign SAO slice header
        if (pcSlice->m_sps->m_saoEnabledFlag)
        {
          for (int s = 0; s < numSliceSegments; s++)
          {
            if (pic->m_slices[s]->m_isLossless && m_encCfg->m_costMode == COST_LOSSLESS_CODING)
            {
              pic->m_slices[s]->m_saoEnabledFlag[ChannelType::LUMA]   = false;
              pic->m_slices[s]->m_saoEnabledFlag[ChannelType::CHROMA] = false;
            }
            else
            {
              pic->m_slices[s]->m_saoEnabledFlag[ChannelType::LUMA] = sliceEnabled[COMP_Y];
              CHECK(!(sliceEnabled[COMP_Cb] == sliceEnabled[COMP_Cr]), "Unspecified error");
              pic->m_slices[s]->m_saoEnabledFlag[ChannelType::CHROMA] = sliceEnabled[COMP_Cb];
            }
          }
        }
      }

      uint32_t uiNumSliceSegments = 1;

      if (pcSlice->m_sps->m_ccSaoEnabledFlag)
      {
        m_pcSAO->initCABACEstimator(m_pcEncLib->getCABACEncoder(), m_pcEncLib->getCtxCache(), pcSlice);
        m_pcSAO->CCSAOProcess(cs, pcSlice->getLambdas(), m_encCfg->m_intraPeriod, m_encCfg->m_CCSAO);

        // assign CCSAO slice header
        for (int s = 0; s < uiNumSliceSegments; s++)
        {
          pic->m_slices[s]->m_ccSaoComParam         = m_pcSAO->getCcSaoComParam();
          pic->m_slices[s]->m_ccSaoControl[COMP_Y]  = m_pcSAO->getCcSaoControlIdc(COMP_Y);
          pic->m_slices[s]->m_ccSaoControl[COMP_Cb] = m_pcSAO->getCcSaoControlIdc(COMP_Cb);
          pic->m_slices[s]->m_ccSaoControl[COMP_Cr] = m_pcSAO->getCcSaoControlIdc(COMP_Cr);
        }
      }
      m_pcSAO->jointClipSaoBifCcSao(cs);

      if (pcSlice->m_sps->m_saoEnabledFlag || pcSlice->m_sps->m_ccSaoEnabledFlag)
      {
        m_pcSAO->destroyEncData();
      }
      if (m_encCfg->m_alf)
      {
        if (!m_encCfg->m_alfImprovements)
        {
          CHECK(m_alfVtm == nullptr, "ALF encoder is not set yet.");
          m_alfVtm->destroy();
          m_alfVtm->create(m_encCfg, picWidth, picHeight, chromaFormatIdc, maxCUWidth, maxCUHeight, maxTotalCUDepth,
                           m_encCfg->m_internalBitDepth);
        }

        for (int s = 0; s < numSliceSegments; s++)
        {
          pic->m_slices[s]->m_alfEnabledFlag[COMP_Y] = false;
        }

        PelStorage cccmCorrection, newOrgBuf;
        if (pcSlice->m_sps->m_lfCccmEnabledFlag && !pcSlice->isIntra())
        {
          cccmCorrection.create(cs.pcv->chrFormat, Area(0, 0, cs.pcv->lumaWidth, cs.pcv->lumaHeight));
          cccmCorrection.copyFrom(cs.getRecoBuf());
          m_pcLoopFilterCccm->lfCccmInitIntraPred(m_pcEncLib->getIntraSearch());
          m_pcLoopFilterCccm->lfCccmRDO(cs, cs.getRecoBuf(), cccmCorrection, m_pcEncLib->getCtxCache(),
                                        m_pcEncLib->getCABACEncoder(), pcSlice);
          if (pcSlice->m_lfCccmEnabledFlag)
          {
            newOrgBuf.create(cs.pcv->chrFormat, Area(0, 0, cs.pcv->lumaWidth, cs.pcv->lumaHeight));
#if ALF_SAO_TRUE_ORG
            newOrgBuf.copyFrom(cs.getTrueOrgBuf());
#else
            newOrgBuf.copyFrom(cs.getOrgBuf());
#endif
            cccmCorrection.getBuf(COMP_Cb).subtract(cs.getRecoBuf(COMP_Cb));
            cccmCorrection.getBuf(COMP_Cr).subtract(cs.getRecoBuf(COMP_Cr));
            newOrgBuf.getBuf(COMP_Cb).subtract(cccmCorrection.getBuf(COMP_Cb));
            newOrgBuf.getBuf(COMP_Cr).subtract(cccmCorrection.getBuf(COMP_Cr));
          }
        }

        if (m_encCfg->m_alfImprovements)
        {
          m_alfEcm->initCABACEstimator(m_pcEncLib->getCABACEncoder(), m_pcEncLib->getCtxCache(), pcSlice,
                                       m_pcEncLib->getApsMap(ApsType::ALF));
          if (pcSlice->m_lfCccmEnabledFlag)
          {
            m_alfEcm->m_newOrgBuf = &newOrgBuf;
          }
          m_alfEcm->ALFProcess(
            cs, pcSlice->getLambdas(),
#if ENABLE_QPA
            (m_encCfg->m_bUsePerceptQPA && !m_encCfg->m_RCEnableRateControl && pcSlice->m_pps->m_useDQP
               ? m_pcEncLib->getRdCost()->getChromaWeight()
               : 0.0),
#endif
            pic, numSliceSegments);
        }
        else
        {
          m_alfVtm->initCABACEstimator(m_pcEncLib->getCABACEncoder(), m_pcEncLib->getCtxCache(), pcSlice,
                                       m_pcEncLib->getApsMap(ApsType::ALF));
          m_alfVtm->ALFProcess(
            cs, pcSlice->getLambdas(),
#if ENABLE_QPA
            (m_encCfg->m_bUsePerceptQPA && !m_encCfg->m_RCEnableRateControl && pcSlice->m_pps->m_useDQP
               ? m_pcEncLib->getRdCost()->getChromaWeight()
               : 0.0),
#endif
            pic, numSliceSegments);
        }
        DTRACE(g_trace_ctx, D_CRC, "ALF");
        DTRACE_CRC(g_trace_ctx, D_CRC, cs, cs.getRecoBuf());

        DTRACE_PIC_COMP(D_REC_CB_LUMA_ALF, cs, cs.getRecoBuf(), COMP_Y);
        DTRACE_PIC_COMP(D_REC_CB_CHROMA_ALF, cs, cs.getRecoBuf(), COMP_Cb);
        DTRACE_PIC_COMP(D_REC_CB_CHROMA_ALF, cs, cs.getRecoBuf(), COMP_Cr);

        // assign ALF slice header
        for (int s = 0; s < numSliceSegments; s++)
        {
          // For the first slice, even if it is lossless, slice level ALF is not disabled and ALF-APS is signaled so
          // that the later lossy slices can use APS of the first slice. However, if the first slice is lossless, the
          // ALF process is disabled for all of the CTUs ( m_ctuEnableFlag == 0) of that slice which is implemented in
          // the function void EncAdaptiveLoopFilter::ALFProcess.

          if (pic->m_slices[s]->m_isLossless && s && m_encCfg->m_costMode == COST_LOSSLESS_CODING)
          {
            pic->m_slices[s]->m_alfEnabledFlag[COMP_Y]  = false;
            pic->m_slices[s]->m_alfEnabledFlag[COMP_Cb] = false;
            pic->m_slices[s]->m_alfEnabledFlag[COMP_Cr] = false;
          }
          else
          {
            pic->m_slices[s]->m_alfEnabledFlag[COMP_Y]  = cs.slice->m_alfEnabledFlag[COMP_Y];
            pic->m_slices[s]->m_alfEnabledFlag[COMP_Cb] = cs.slice->m_alfEnabledFlag[COMP_Cb];
            pic->m_slices[s]->m_alfEnabledFlag[COMP_Cr] = cs.slice->m_alfEnabledFlag[COMP_Cr];
          }
          if (pic->m_slices[s]->m_alfEnabledFlag[COMP_Y])
          {
            if (m_encCfg->m_alfImprovements)
            {
              memcpy(pic->m_slices[s]->m_newAlfFixFiltSetCandIdx, cs.slice->m_newAlfFixFiltSetCandIdx,
                     sizeof(pic->m_slices[s]->m_newAlfFixFiltSetCandIdx));
            }
            else
            {
              std::fill_n(pic->m_slices[s]->m_newAlfFixFiltSetCandIdx, MAX_NUM_COMP, -1);
            }
            pic->m_slices[s]->m_numAlfApsIdsLuma = cs.slice->m_numAlfApsIdsLuma;
            pic->m_slices[s]->m_alfApsIdsLuma    = cs.slice->m_alfApsIdsLuma;
          }
          else
          {
            pic->m_slices[s]->m_numAlfApsIdsLuma = 0;
          }
          pic->m_slices[s]->setAlfAPSs(cs.slice->m_alfApss);
          pic->m_slices[s]->m_alfApsIdChroma = cs.slice->m_alfApsIdChroma;
          pic->m_slices[s]->m_ccAlfCbApsId   = cs.slice->m_ccAlfCbApsId;
          pic->m_slices[s]->m_ccAlfCrApsId   = cs.slice->m_ccAlfCrApsId;
          if (m_encCfg->m_alfImprovements)
          {
            pic->m_slices[s]->m_ccAlfFilterParam.getEcmParam() = m_alfEcm->getCcAlfFilterParam();
            pic->m_slices[s]->m_ccAlfFilterControl[0]          = m_alfEcm->getCcAlfControlIdc(COMP_Cb);
            pic->m_slices[s]->m_ccAlfFilterControl[1]          = m_alfEcm->getCcAlfControlIdc(COMP_Cr);
          }
          else
          {
            pic->m_slices[s]->m_ccAlfFilterParam.getVtmParam() = m_alfVtm->getCcAlfFilterParam();
            pic->m_slices[s]->m_ccAlfFilterControl[0]          = m_alfVtm->getCcAlfControlIdc(COMP_Cb);
            pic->m_slices[s]->m_ccAlfFilterControl[1]          = m_alfVtm->getCcAlfControlIdc(COMP_Cr);
          }
        }

        if (pcSlice->m_lfCccmEnabledFlag)
        {
          cs.getRecoBuf(COMP_Cb).reconstruct(cs.getRecoBuf(COMP_Cb), cccmCorrection.getBuf(COMP_Cb),
                                             cs.slice->clpRng(COMP_Cb));
          cs.getRecoBuf(COMP_Cr).reconstruct(cs.getRecoBuf(COMP_Cr), cccmCorrection.getBuf(COMP_Cr),
                                             cs.slice->clpRng(COMP_Cr));
          DTRACE(g_trace_ctx, D_CRC, "CCCM");
          DTRACE_CRC(g_trace_ctx, D_CRC, cs, cs.getRecoBuf());
        }
      }

      DTRACE_UPDATE(g_trace_ctx, (std::make_pair("final", 1)));
      if (m_encCfg->m_compositeRefEnabled && getPrepareLTRef())
      {
        updateCompositeReference(pcSlice, rcListPic, pocCurr);
      }
      pic->copyAdaptedLumaClip(false);
    }
    else   // skip enc picture
    {
      pcSlice->m_iSliceQpBase = pcSlice->m_iSliceQp;
      m_pcSliceEncoder->getCABACDataStore()->updateBufferState(pcSlice);
#if ENABLE_QPA
      if (m_encCfg->m_bUsePerceptQPA && !m_encCfg->m_RCEnableRateControl && pcSlice->m_pps->m_useDQP)
      {
        const double picLambda = pcSlice->getLambdas()[0];

        for (uint32_t ctuRsAddr = 0; ctuRsAddr < numberOfCtusInFrame; ctuRsAddr++)
        {
          pic->m_uEnerHpCtu[ctuRsAddr] = picLambda;   // initialize to slice lambda (just for safety)
        }
      }
#endif
      if (pcSlice->m_sps->m_saoEnabledFlag)
      {
        m_pcSAO->disabledRate(*pic->m_cs, pic->getSAO(1), m_encCfg->m_saoEncodingRate,
                              m_encCfg->m_saoEncodingRateChroma);
      }
      if (pcSlice->m_sps->m_ccSaoEnabledFlag)
      {
        m_pcSAO->getCcSaoComParam() = pcSlice->m_ccSaoComParam;
        m_pcSAO->setupCcSaoPrv(*pic->m_cs);
      }

      if (m_encCfg->m_alf)
      {
        // IRAP AU: reset APS map
        {
          if (pcSlice->m_pendingRasInit || pcSlice->isIDRorBLA())
          {
            // We have to reset all APS on IRAP, but in not encoding case we have to keep the parsed APS of current
            // slice Get active ALF APSs from picture/slice header
            const AlfParameters::AlfApsList &sliceApsIdsLuma = pcSlice->m_alfApsIdsLuma;

            if (m_encCfg->m_alfImprovements)
            {
              m_alfEcm->setApsIdStart(m_encCfg->m_alfapsIDShift + m_encCfg->m_maxNumAlfAps);
            }
            else
            {
              m_alfVtm->setApsIdStart(m_encCfg->m_alfapsIDShift + m_encCfg->m_maxNumAlfAps);
            }

            ParameterSetMap<APS> *apsMap = m_pcEncLib->getApsMap(ApsType::ALF);
            apsMap->clearActive();

            for (int apsId = m_encCfg->m_alfapsIDShift; apsId < m_encCfg->m_alfapsIDShift + m_encCfg->m_maxNumAlfAps;
                 apsId++)
            {
              APS *aps = apsMap->getPS(apsId);
              if (aps)
              {
                // Check if this APS is currently the active one (used in current slice)
                bool activeAps      = false;
                bool activeApsCcAlf = false;
                // Luma
                for (int i = 0; i < sliceApsIdsLuma.size(); i++)
                {
                  if (aps->m_APSId == sliceApsIdsLuma[i])
                  {
                    activeAps = true;
                    break;
                  }
                }
                // Chroma
                activeAps |= aps->m_APSId == pcSlice->m_alfApsIdChroma &&
                  (pcSlice->m_alfEnabledFlag[COMP_Cb] || pcSlice->m_alfEnabledFlag[COMP_Cr]);
                // CC-ALF
                activeApsCcAlf |= pcSlice->m_ccAlfCbEnabledFlag && aps->m_APSId == pcSlice->m_ccAlfCbApsId;
                activeApsCcAlf |= pcSlice->m_ccAlfCrEnabledFlag && aps->m_APSId == pcSlice->m_ccAlfCrApsId;
                if (!activeAps && !activeApsCcAlf)
                {
                  apsMap->clearChangedFlag(apsId);
                }
                if (!activeAps)
                {
                  if (m_encCfg->m_alfImprovements)
                  {
                    aps->m_alfAPSParam.getEcmParam().reset();
                  }
                  else
                  {
                    aps->m_alfAPSParam.getVtmParam().reset();
                  }
                }
                if (!activeApsCcAlf)
                {
                  if (m_encCfg->m_alfImprovements)
                  {
                    aps->m_ccAlfAPSParam.getEcmParam().reset();
                  }
                  else
                  {
                    aps->m_ccAlfAPSParam.getVtmParam().reset();
                  }
                }
              }
            }
          }
        }

        // Assign tne correct APS to slice and emulate the setting of ALF start APS ID
        int changedApsId = -1;
        for (int apsId = m_encCfg->m_alfapsIDShift + m_encCfg->m_maxNumAlfAps - 1; apsId >= m_encCfg->m_alfapsIDShift;
             apsId--)
        {
          ParameterSetMap<APS> *apsMap = m_pcEncLib->getApsMap(ApsType::ALF);
          APS                  *aps    = apsMap->getPS(apsId);
          if (aps)
          {
            // In slice, replace the old APS (from decoder map) with the APS from encoder map due to later checks while
            // bitstream writing
            //            if( pcSlice->m_alfApss && pcSlice->m_alfApss[apsId] )
            if (pcSlice->m_alfApss[apsId])
            {
              pcSlice->m_alfApss[apsId] = aps;
            }
            else
            {
              // fill up for the check that follows
              pcSlice->m_alfApss[apsId] = aps;
            }
            if (apsMap->getChangedFlag(apsId) && (pcSlice->checkAlfAPS(apsId)))
            {
              changedApsId = apsId;
            }
          }
        }
        if (changedApsId >= 0)
        {
          if (m_encCfg->m_alfImprovements)
          {
            m_alfEcm->setApsIdStart(changedApsId);
          }
          else
          {
            m_alfVtm->setApsIdStart(changedApsId);
          }
        }
      }
    }

    if (m_encCfg->m_useAMaxBT && !pcSlice->isIntra())
    {
      const int hierPredLayerIdx = std::min<int>(pcSlice->m_hierPredLayerIdx, (int)m_blkStat.size() - 1);

      for (const CodingUnit *cu: pic->m_cs->cus)
      {
        m_blkStat[hierPredLayerIdx].area += cu->Y().area();
        m_blkStat[hierPredLayerIdx].count++;
      }
    }

    if (m_encCfg->m_seiCfg.m_fgcSEIAnalysisEnabled)
    {
      int  filteredFrame  = m_encCfg->m_intraPeriod < 1 ? 2 * m_encCfg->m_frameRate : m_encCfg->m_intraPeriod;
      bool readyToAnalyze = pic->m_poc % filteredFrame
        ? false
        : true;   // either it is mctf denoising or external source for film grain analysis. note:
                  // if mctf is used, it is different from mctf for encoding.
      if (readyToAnalyze)
      {
        m_fgAnalyzer.initBufs(pic);
        m_fgAnalyzer.estimate_grain(pic);
      }
    }

    if (encPic || decPic)
    {
      pcSlice = pic->m_slices[0];

      /////////////////////////////////////////////////////////////////////////////////////////////////// File writing

      // write various parameter sets
      bool writePS = m_seqFirst || (m_encCfg->m_rewriteParamSets && (pcSlice->isIRAP()));
      if (writePS)
      {
        m_pcEncLib->setParamSetChanged(pcSlice->m_sps->m_spsId, pcSlice->m_pps->m_ppsId);
      }

      int layerIdx = m_pcEncLib->m_vps == nullptr ? 0 : m_pcEncLib->m_vps->m_generalLayerIdx[m_pcEncLib->m_layerId];

      // it is assumed that layerIdx equal to 0 is always present
      m_audIrapOrGdrAuFlag =
        pcSlice->m_picHeader->m_gdrPicFlag || (pcSlice->isIRAP() && !pcSlice->m_pps->m_mixedNaluTypesInPicFlag);
      if (((m_pcEncLib->m_vps->m_maxLayers > 1 && m_audIrapOrGdrAuFlag) || m_encCfg->m_AccessUnitDelimiter) &&
          !layerIdx)
      {
        xWriteAccessUnitDelimiter(accessUnit, pcSlice);
      }

      // it is assumed that layerIdx equal to 0 is always present
      bool newPPS = m_pcEncLib->PPSNeedsWriting(pcSlice->m_pps->m_ppsId);
      if (m_encCfg->m_rprFunctionalityTestingEnabledFlag || m_encCfg->m_gopBasedRPREnabledFlag)
      {
        if (newPPS)
        {
          m_pcEncLib->setRprPPSCodedAfterIntra(getRprResolutionIndex(pcSlice->m_pps->m_ppsId), true);
        }
        // here a PPS needs to be encoded for an inter picture if PPS is different from any RPR PPS written after and
        // including the intra
        if ((m_encCfg->m_rprFunctionalityTestingEnabledFlag &&
             (pcSlice->m_poc % m_encCfg->m_rprSwitchingSegmentSize) == 0) ||
            (m_encCfg->m_gopBasedRPREnabledFlag && (pcSlice->m_poc % m_encCfg->m_gopSize) == 0))
        {
          if (pcSlice->isIntra())
          {
            for (int nr = 0; nr < NUM_RPR_PPS; nr++)
            {
              // at intra all PPS coded after Intra is reset except the current one
              if (pcSlice->m_pps->m_ppsId != RPR_PPS_ID[nr])
              {
                m_pcEncLib->setRprPPSCodedAfterIntra(getRprResolutionIndex(RPR_PPS_ID[nr]), false);
              }
            }
          }
          else
          {
            if (!m_pcEncLib->getRprPPSCodedAfterIntra(getRprResolutionIndex(pcSlice->m_pps->m_ppsId)))
            {
              // here a forced coding of a pps is enabled
              newPPS = true;
              m_pcEncLib->setRprPPSCodedAfterIntra(getRprResolutionIndex(pcSlice->m_pps->m_ppsId), true);
            }
          }
        }
      }
      actualTotalBits += xWriteParameterSets(accessUnit, pcSlice, writePS, layerIdx, newPPS);

      if (writePS)
      {
        // create prefix SEI messages at the beginning of the sequence
        CHECK(!(leadingSeiMessages.empty()), "Unspecified error");
        xCreateIRAPLeadingSEIMessages(leadingSeiMessages, pcSlice->m_sps, pcSlice->m_pps);

        m_seqFirst = false;
      }

      // send LMCS APS when LMCSModel is updated. It can be updated even current slice does not enable reshaper.
      // For example, in RA, update is on intra slice, but intra slice may not use reshaper
      if (pcSlice->m_sps->m_lmcsEnabled)
      {
        // only 1 LMCS data for 1 picture
        int apsId = picHeader->m_lmcsApsId;

        ParameterSetMap<APS> *apsMapLmcs = m_pcEncLib->getApsMap(ApsType::LMCS);

        APS *aps = apsId >= 0 ? apsMapLmcs->getPS(apsId) : nullptr;

        bool writeAPS = aps && apsMapLmcs->getChangedFlag(apsId);
        if (!picHeader->m_lmcsAps)
        {
          // there is a reset at the decoder
          picHeader->m_lmcsAps = aps;
        }

        if (writeAPS)
        {
          aps->chromaPresentFlag = isChromaEnabled(pcSlice->m_sps->m_chromaFormatIdc);
          actualTotalBits += xWriteAPS(accessUnit, aps, m_pcEncLib->m_layerId, true);
          apsMapLmcs->clearChangedFlag(apsId);
          CHECK(!picHeader->m_lmcsAps, "picHeader->m_lmcsAps not set");
          CHECK(aps != picHeader->m_lmcsAps, "Wrong LMCS APS pointer in compressGOP");
        }
      }

      // only 1 SCALING LIST data for 1 picture
      if (pcSlice->m_sps->m_scalingListEnabledFlag && (m_encCfg->m_useScalingListId == SCALING_LIST_FILE_READ))
      {
        const int             apsId    = picHeader->m_scalingListApsId;
        ParameterSetMap<APS> *apsMapSl = m_pcEncLib->getApsMap(ApsType::SCALING_LIST);
        APS                  *aps      = apsMapSl->getPS(apsId);
        bool                  writeAPS = aps && apsMapSl->getChangedFlag(apsId);
        if (writeAPS)
        {
          aps->chromaPresentFlag = isChromaEnabled(pcSlice->m_sps->m_chromaFormatIdc);
          actualTotalBits += xWriteAPS(accessUnit, aps, m_pcEncLib->m_layerId, true);
          apsMapSl->clearChangedFlag(apsId);
          CHECK(aps != picHeader->m_scalingListAps, "Wrong SCALING LIST APS pointer in compressGOP");
        }
      }

      if (m_encCfg->m_alf &&
          (pcSlice->m_alfEnabledFlag[COMP_Y] || pcSlice->m_ccAlfCbEnabledFlag || pcSlice->m_ccAlfCrEnabledFlag))
      {
        for (int apsId = m_encCfg->m_alfapsIDShift; apsId < m_encCfg->m_alfapsIDShift + m_encCfg->m_maxNumAlfAps;
             apsId++)
        {
          ParameterSetMap<APS> *apsMapAlf = m_pcEncLib->getApsMap(ApsType::ALF);

          APS *aps      = apsMapAlf->getPS(apsId);
          bool writeAPS = aps && apsMapAlf->getChangedFlag(apsId);
          //          if (!aps && pcSlice->m_alfApss && pcSlice->m_alfApss[apsId])
          if (!aps && pcSlice->m_alfApss[apsId])
          {
            writeAPS                      = true;
            aps                           = pcSlice->m_alfApss[apsId];   // use asp from slice header
            *apsMapAlf->allocatePS(apsId) = *aps;   // allocate and cpy
            if (m_encCfg->m_alfImprovements)
            {
              m_alfEcm->setApsIdStart(apsId);
            }
            else
            {
              m_alfVtm->setApsIdStart(apsId);
            }
          }
          else if (pcSlice->m_ccAlfCbEnabledFlag && !aps && apsId == pcSlice->m_ccAlfCbApsId)
          {
            writeAPS = true;
            aps      = apsMapAlf->getPS(pcSlice->m_ccAlfCbApsId);
          }
          else if (pcSlice->m_ccAlfCrEnabledFlag && !aps && apsId == pcSlice->m_ccAlfCrApsId)
          {
            writeAPS = true;
            aps      = apsMapAlf->getPS(pcSlice->m_ccAlfCrApsId);
          }
          if (writeAPS)
          {
            aps->chromaPresentFlag = isChromaEnabled(pcSlice->m_sps->m_chromaFormatIdc);
            actualTotalBits += xWriteAPS(accessUnit, aps, m_pcEncLib->m_layerId, true);
            apsMapAlf->clearChangedFlag(apsId);
            CHECK(aps != pcSlice->m_alfApss[apsId] && apsId != pcSlice->m_ccAlfCbApsId &&
                    apsId != pcSlice->m_ccAlfCrApsId,
                  "Wrong APS pointer in compressGOP");
          }
        }
      }

      // reset presence of BP SEI indication
      m_bufferingPeriodSEIPresentInAU = false;
      // create prefix SEI associated with a picture
      xCreatePerPictureSEIMessages(gopId, leadingSeiMessages, nestedSeiMessages, pcSlice);

      if (newPPS)
      {
        xCreatePhaseIndicationSEIMessages(leadingSeiMessages, pcSlice, pcSlice->m_pps->m_ppsId);
      }
      // pcSlice is currently slice 0.
      std::size_t binCountsInNalUnits   = 0;   // For implementation of cabac_zero_word stuffing (section 7.4.3.10)
      std::size_t numBytesInVclNalUnits = 0;   // For implementation of cabac_zero_word stuffing (section 7.4.3.10)
      std::size_t sumZeroWords          = 0;   // sum of cabac_zero_word inserted per sub-picture
      std::vector<EncBitstreamParams> subPicStats(pic->m_cs->pps->m_numSubPics);

      for (uint32_t sliceSegmentIdxCount = 0; sliceSegmentIdxCount < pic->m_cs->pps->m_numSlicesInPic;
           sliceSegmentIdxCount++)
      {
        pcSlice = pic->m_slices[sliceSegmentIdxCount];
        if (sliceSegmentIdxCount > 0 && pcSlice->m_eSliceType != I_SLICE)
        {
          pcSlice->checkColRefIdx(sliceSegmentIdxCount, pic);
        }
        m_pcSliceEncoder->setSliceSegmentIdx(sliceSegmentIdxCount);

        pcSlice->m_rpl[RPL0]    = pic->m_slices[0]->m_rpl[RPL0];
        pcSlice->m_rpl[RPL1]    = pic->m_slices[0]->m_rpl[RPL1];
        pcSlice->m_rplIdx[RPL0] = pic->m_slices[0]->m_rplIdx[RPL0];
        pcSlice->m_rplIdx[RPL1] = pic->m_slices[0]->m_rplIdx[RPL1];

        picHeader->m_noOutputBeforeRecoveryFlag = false;
        if (pcSlice->isIRAP())
        {
          if (pcSlice->m_eNalUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL &&
              pcSlice->m_eNalUnitType <= NAL_UNIT_CODED_SLICE_IDR_N_LP)
          {
            picHeader->m_noOutputBeforeRecoveryFlag = true;
          }
          // the inference for NoOutputPriorPicsFlag
          //  KJS: This cannot happen at the encoder
          if (!m_first && (pcSlice->isIRAP() || pcSlice->m_eNalUnitType >= NAL_UNIT_CODED_SLICE_GDR) &&
              picHeader->m_noOutputBeforeRecoveryFlag)
          {
            if (pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA ||
                pcSlice->m_eNalUnitType >= NAL_UNIT_CODED_SLICE_GDR)
            {
              pcSlice->m_noOutputOfPriorPicsFlag = true;
            }
          }
        }

        // code picture header before first slice
        if (sliceSegmentIdxCount == 0)
        {
          // code RPL in picture header or slice headers
          if (!m_encCfg->m_sliceLevelRpl && (!pcSlice->getIdrPicFlag() || pcSlice->m_sps->m_idrRefParamList))
          {
            picHeader->m_rplIdx[RPL0] = pcSlice->m_rplIdx[RPL0];
            picHeader->m_rplIdx[RPL1] = pcSlice->m_rplIdx[RPL1];
            picHeader->m_rpl[RPL0]    = pcSlice->m_rpl[RPL0];
            picHeader->m_rpl[RPL1]    = pcSlice->m_rpl[RPL1];
          }

          // code DBLK in picture header or slice headers
          if (!m_encCfg->m_sliceLevelDblk)
          {
            picHeader->m_deblockingFilterOverrideFlag     = pcSlice->m_deblockingFilterOverrideFlag;
            picHeader->m_deblockingFilterDisable          = pcSlice->m_deblockingFilterDisable;
            picHeader->m_deblockingFilterBetaOffsetDiv2   = pcSlice->m_deblockingFilterBetaOffsetDiv2;
            picHeader->m_deblockingFilterTcOffsetDiv2     = pcSlice->m_deblockingFilterTcOffsetDiv2;
            picHeader->m_deblockingFilterCbBetaOffsetDiv2 = pcSlice->m_deblockingFilterCbBetaOffsetDiv2;
            picHeader->m_deblockingFilterCbTcOffsetDiv2   = pcSlice->m_deblockingFilterCbTcOffsetDiv2;
            picHeader->m_deblockingFilterCrBetaOffsetDiv2 = pcSlice->m_deblockingFilterCrBetaOffsetDiv2;
            picHeader->m_deblockingFilterCrTcOffsetDiv2   = pcSlice->m_deblockingFilterCrTcOffsetDiv2;
          }

          if (!m_encCfg->m_sliceLevelDeltaQp)
          {
            picHeader->m_qpDelta = (pcSlice->m_iSliceQp - (pcSlice->m_pps->m_picInitQPMinus26 + 26));
          }

          // code SAO parameters in picture header or slice headers
          if (!m_encCfg->m_sliceLevelSao)
          {
            picHeader->m_saoEnabledFlag[ChannelType::LUMA]   = pcSlice->m_saoEnabledFlag[ChannelType::LUMA];
            picHeader->m_saoEnabledFlag[ChannelType::CHROMA] = pcSlice->m_saoEnabledFlag[ChannelType::CHROMA];
            picHeader->m_ccSaoEnabledFlag[COMP_Y]            = pcSlice->m_ccSaoEnabledFlag[COMP_Y];
            picHeader->m_ccSaoEnabledFlag[COMP_Cb]           = pcSlice->m_ccSaoEnabledFlag[COMP_Cb];
            picHeader->m_ccSaoEnabledFlag[COMP_Cr]           = pcSlice->m_ccSaoEnabledFlag[COMP_Cr];
          }
          // code ALF parameters in picture header or slice headers
          if (!m_encCfg->m_sliceLevelAlf)
          {
            picHeader->m_alfEnabledFlag[COMP_Y]  = pcSlice->m_alfEnabledFlag[COMP_Y];
            picHeader->m_alfEnabledFlag[COMP_Cb] = pcSlice->m_alfEnabledFlag[COMP_Cb];
            picHeader->m_alfEnabledFlag[COMP_Cr] = pcSlice->m_alfEnabledFlag[COMP_Cr];
            if (m_encCfg->m_alfImprovements)
            {
              memcpy(picHeader->m_newAlfFixFiltSetCandIdx, pcSlice->m_newAlfFixFiltSetCandIdx,
                     sizeof(picHeader->m_newAlfFixFiltSetCandIdx));
            }
            else
            {
              std::fill_n(picHeader->m_newAlfFixFiltSetCandIdx, MAX_NUM_COMP, -1);
            }
            picHeader->m_numAlfApsIdsLuma          = pcSlice->m_numAlfApsIdsLuma;
            picHeader->m_alfApsIdsLuma             = pcSlice->m_alfApsIdsLuma;
            picHeader->m_alfApsIdChroma            = pcSlice->m_alfApsIdChroma;
            picHeader->m_ccalfEnabledFlag[COMP_Cb] = pcSlice->m_ccAlfCbEnabledFlag;
            picHeader->m_ccalfEnabledFlag[COMP_Cr] = pcSlice->m_ccAlfCrEnabledFlag;
            picHeader->m_ccAlfCbApsId              = pcSlice->m_ccAlfCbApsId;
            picHeader->m_ccAlfCrApsId              = pcSlice->m_ccAlfCrApsId;
          }

          if (pcSlice->m_sps->m_alfEnabledFlag)
          {
            if (!pcSlice->m_pps->m_alfInfoInPhFlag)
            {
              picHeader->m_alfEnabledFlag[COMP_Y]    = true;
              picHeader->m_alfEnabledFlag[COMP_Cb]   = true;
              picHeader->m_alfEnabledFlag[COMP_Cr]   = true;
              picHeader->m_ccalfEnabledFlag[COMP_Cb] = pcSlice->m_sps->m_ccalfEnabledFlag;
              picHeader->m_ccalfEnabledFlag[COMP_Cr] = pcSlice->m_sps->m_ccalfEnabledFlag;
            }
          }
          else
          {
            picHeader->m_alfEnabledFlag[COMP_Y]    = false;
            picHeader->m_alfEnabledFlag[COMP_Cb]   = false;
            picHeader->m_alfEnabledFlag[COMP_Cr]   = false;
            picHeader->m_ccalfEnabledFlag[COMP_Cb] = false;
            picHeader->m_ccalfEnabledFlag[COMP_Cr] = false;
          }

          if (pcSlice->m_sps->m_saoEnabledFlag)
          {
            if (!pcSlice->m_pps->m_saoInfoInPhFlag)
            {
              picHeader->m_saoEnabledFlag[ChannelType::LUMA]   = true;
              picHeader->m_saoEnabledFlag[ChannelType::CHROMA] = true;
            }
          }
          else
          {
            picHeader->m_saoEnabledFlag[ChannelType::LUMA]   = false;
            picHeader->m_saoEnabledFlag[ChannelType::CHROMA] = false;
          }

          if (!pcSlice->m_sps->m_partitionOverrideEnabled || !picHeader->m_splitConsOverrideFlag)
          {
            picHeader->setMinQTSizes(pcSlice->m_sps->m_minQT);
            picHeader->setMaxMTTHierarchyDepths(pcSlice->m_sps->m_maxMTTHierarchyDepth);
            picHeader->setMaxBTSizes(pcSlice->m_sps->m_maxBTSize);
            picHeader->setMaxTTSizes(pcSlice->m_sps->m_maxTTSize);
          }

          if (pcSlice->m_pps->m_deblockingFilterControlPresentFlag)
          {
            if (!pcSlice->m_pps->m_dbfInfoInPhFlag)
            {
              picHeader->m_deblockingFilterOverrideFlag = false;
            }

            if (!picHeader->m_deblockingFilterOverrideFlag)
            {
              picHeader->m_deblockingFilterDisable          = pcSlice->m_pps->m_ppsDeblockingFilterDisabledFlag;
              picHeader->m_deblockingFilterBetaOffsetDiv2   = pcSlice->m_pps->m_deblockingFilterBetaOffsetDiv2;
              picHeader->m_deblockingFilterTcOffsetDiv2     = pcSlice->m_pps->m_deblockingFilterTcOffsetDiv2;
              picHeader->m_deblockingFilterCbBetaOffsetDiv2 = pcSlice->m_pps->m_deblockingFilterCbBetaOffsetDiv2;
              picHeader->m_deblockingFilterCbTcOffsetDiv2   = pcSlice->m_pps->m_deblockingFilterCbTcOffsetDiv2;
              picHeader->m_deblockingFilterCrBetaOffsetDiv2 = pcSlice->m_pps->m_deblockingFilterCrBetaOffsetDiv2;
              picHeader->m_deblockingFilterCrTcOffsetDiv2   = pcSlice->m_pps->m_deblockingFilterCrTcOffsetDiv2;
            }
          }
          else
          {
            picHeader->m_deblockingFilterDisable          = false;
            picHeader->m_deblockingFilterBetaOffsetDiv2   = 0;
            picHeader->m_deblockingFilterTcOffsetDiv2     = 0;
            picHeader->m_deblockingFilterCbBetaOffsetDiv2 = 0;
            picHeader->m_deblockingFilterCbTcOffsetDiv2   = 0;
            picHeader->m_deblockingFilterCrBetaOffsetDiv2 = 0;
            picHeader->m_deblockingFilterCrTcOffsetDiv2   = 0;
          }

          // code WP parameters in picture header or slice headers
          if (!m_encCfg->m_sliceLevelWp)
          {
            picHeader->setWpScaling(pcSlice->getWpScalingAll());
            picHeader->m_numWeights[RPL0] = pcSlice->m_numRefIdx[RPL0];
            picHeader->m_numWeights[RPL1] = pcSlice->m_numRefIdx[RPL1];
          }

          pic->m_cs->picHeader->m_pic   = pic;
          pic->m_cs->picHeader->m_valid = true;
          if (pic->m_cs->pps->m_numSlicesInPic > 1 || !m_encCfg->m_enablePictureHeaderInSliceHeader)
          {
            pcSlice->m_pictureHeaderInSliceHeader = false;
            actualTotalBits += xWritePicHeader(accessUnit, pic->m_cs->picHeader);
          }
          else
          {
            pcSlice->m_pictureHeaderInSliceHeader = true;
          }
          if (pcSlice->m_sps->m_profileTierLevel.m_constraintInfo.m_picHeaderInSliceHeaderConstraintFlag)
          {
            CHECK(pcSlice->m_pictureHeaderInSliceHeader == false,
                  "PH shall be present in SH, when pic_header_in_slice_header_constraint_flag is equal to 1");
          }
        }
        pcSlice->m_picHeader  = pic->m_cs->picHeader;
        pcSlice->m_nuhLayerId = m_pcEncLib->m_layerId;

        for (uint32_t ui = 0; ui < numSubstreams; ui++)
        {
          substreamsOut[ui].clear();
        }

        /* start slice NALunit */
        OutputNALUnit nalu(pcSlice->m_eNalUnitType, m_pcEncLib->m_layerId, pcSlice->m_uiTLayer);
        m_HLSWriter->setBitstream(&nalu.m_bitstream);

        tmpBitsBeforeWriting = m_HLSWriter->getNumberOfWrittenBits();
        m_HLSWriter->codeSliceHeader(pcSlice);
        actualHeadBits += (m_HLSWriter->getNumberOfWrittenBits() - tmpBitsBeforeWriting);

        pcSlice->m_numSubstream = 0;
        pcSlice->setNumSubstream(pcSlice->m_sps, pcSlice->m_pps);
        pcSlice->m_substreamSizes.clear();
        const int subpicIdx = pic->m_cs->pps->getSubPicIdxFromSubPicId(pcSlice->m_sliceSubPicId);
        {
          uint32_t numBinsCoded = 0;
          m_pcSliceEncoder->encodeSlice(pic, &(substreamsOut[0]), numBinsCoded);
          binCountsInNalUnits += numBinsCoded;
          subPicStats[subpicIdx].numBinsWritten += numBinsCoded;
        }
        if (pcSlice->m_sps->m_spsRangeExtension.m_tsrcRicePresentFlag && (pic->m_cs->pps->m_numSlicesInPic == 1))
        {
          if (pcSlice->m_eSliceType == I_SLICE)
          {
            for (int idx = 0; idx < MAX_TSRC_RICE; idx++)
            {
              m_riceBit[idx][1] = pcSlice->m_riceBit[idx];
            }
          }
          for (int idx = 0; idx < MAX_TSRC_RICE; idx++)
          {
            m_riceBit[idx][0] = pcSlice->m_riceBit[idx];
          }
          m_preQP[0] = pcSlice->m_iSliceQp;
        }
        {
          // Construct the final bitstream by concatenating substreams.
          // The final bitstream is either nalu.m_bitstream or pcBitstreamRedirect;
          // Complete the slice header info.
          m_HLSWriter->setBitstream(&nalu.m_bitstream);
          pcSlice->setNumEntryPoints(pcSlice->m_sps, pcSlice->m_pps);
          m_HLSWriter->codeTilesWPPEntryPoint(pcSlice);

          // Append substreams...
          OutputBitstream *pcOut               = pcBitstreamRedirect;
          const int        numSubstreamsToCode = pcSlice->m_numSubstream + 1;

          for (uint32_t ui = 0; ui < numSubstreamsToCode; ui++)
          {
            pcOut->addSubstream(&(substreamsOut[ui]));
          }
        }

        // If current NALU is the first NALU of slice (containing slice header) and more NALUs exist (due to multiple
        // dependent slices) then buffer it. If current NALU is the last NALU of slice and a NALU was buffered, then (a)
        // Write current NALU (b) Update an write buffered NALU at approproate location in NALU list.
        bool naluAlignedWrittenToList =
          false;   // used to ensure current NALU is not written more than once to the NALU list.
        xAttachSliceDataToNalUnit(nalu, pcBitstreamRedirect);
        accessUnit.push_back(new NALUnitEBSP(nalu));
        actualTotalBits += uint32_t(accessUnit.back()->m_nalUnitData.str().size()) * 8;
        numBytesInVclNalUnits += (std::size_t)(accessUnit.back()->m_nalUnitData.str().size());
        subPicStats[subpicIdx].numBytesInVclNalUnits += (std::size_t)(accessUnit.back()->m_nalUnitData.str().size());
        naluAlignedWrittenToList = true;

        if (!naluAlignedWrittenToList)
        {
          nalu.m_bitstream.writeAlignZero();
          accessUnit.push_back(new NALUnitEBSP(nalu));
        }

        if ((m_encCfg->m_seiCfg.m_pictureTimingSEIEnabled || m_encCfg->m_seiCfg.m_decodingUnitInfoSEIEnabled) &&
            ((pcSlice->m_sps->m_generalHrdParams.m_generalNalHrdParamsPresentFlag) ||
             (pcSlice->m_sps->m_generalHrdParams.m_generalVclHrdParamsPresentFlag)) &&
            (pcSlice->m_sps->m_generalHrdParams.m_generalDecodingUnitHrdParamsPresentFlag))
        {
          uint32_t numNalus     = 0;
          uint32_t numRBSPBytes = 0;
          for (AccessUnit::const_iterator it = accessUnit.begin(); it != accessUnit.end(); it++)
          {
            numRBSPBytes += uint32_t((*it)->m_nalUnitData.str().size());
            numNalus++;
          }
          duData.push_back(DUData());
          duData.back().accumBitsDU = (numRBSPBytes << 3);
          duData.back().accumNalsDU = numNalus;
        }
        if (pcSlice->isLastSliceInSubpic())
        {
          // Check picture level encoding constraints/requirements
          ProfileTierLevelFeatures profileTierLevelFeatures;
          profileTierLevelFeatures.extractPTLInformation(*(pcSlice->m_sps));
          const SEIMessages &subPictureLevelInfoSEIs =
            getSeisByType(leadingSeiMessages, SEI::PayloadType::SUBPICTURE_LEVEL_INFO);
          if (!subPictureLevelInfoSEIs.empty())
          {
            const SEISubpicureLevelInfo &seiSubpic =
              static_cast<const SEISubpicureLevelInfo &>(*subPictureLevelInfoSEIs.front());
            validateMinCrRequirements(profileTierLevelFeatures, subPicStats[subpicIdx].numBytesInVclNalUnits, pcSlice,
                                      m_encCfg, seiSubpic, subpicIdx, m_pcEncLib->m_layerId);
          }
          sumZeroWords += cabac_zero_word_padding(
            pcSlice, pic, subPicStats[subpicIdx].numBinsWritten, subPicStats[subpicIdx].numBytesInVclNalUnits, 0,
            accessUnit.back()->m_nalUnitData, m_encCfg->m_cabacZeroWordPaddingEnabled, profileTierLevelFeatures);
        }
      }   // end iteration over slices

      if ((pic->m_temporalId == 0) || (pic->m_temporalId < pcSlice->m_sps->m_maxSubLayers - 1))
      {
        CS::saveTemporalEipModel(*pic->m_cs);
      }

      {
        // Check picture level encoding constraints/requirements
        ProfileTierLevelFeatures profileTierLevelFeatures;
        profileTierLevelFeatures.extractPTLInformation(*(pcSlice->m_sps));
        validateMinCrRequirements(profileTierLevelFeatures, numBytesInVclNalUnits, pic, m_encCfg);
        // cabac_zero_words processing
        cabac_zero_word_padding(pcSlice, pic, binCountsInNalUnits, numBytesInVclNalUnits, sumZeroWords,
                                accessUnit.back()->m_nalUnitData, m_encCfg->m_cabacZeroWordPaddingEnabled,
                                profileTierLevelFeatures);
      }

      //-- For time output for each slice
      auto elapsed = std::chrono::steady_clock::now() - beforeTime;
      auto encTime = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

      std::string digestStr;
      if (m_encCfg->m_seiCfg.m_decodedPictureHashSEIType != HashType::NONE)
      {
        SEIDecodedPictureHash *decodedPictureHashSei = new SEIDecodedPictureHash();
        PelUnitBuf             recoBuf               = pic->m_cs->getRecoBuf();
        m_seiEncoder.initDecodedPictureHashSEI(decodedPictureHashSei, recoBuf, digestStr, pcSlice->m_sps->m_bitDepths);
        trailingSeiMessages.push_back(decodedPictureHashSei);
      }
      // create per-subpicture decoded picture hash SEI messages, if more than one subpicture is enabled
      const PPS  *pps        = pic->m_cs->pps;
      const int   numSubpics = pps->m_numSubPics;
      std::string subPicDigest;
      if (numSubpics > 1 && m_encCfg->m_seiCfg.m_subpicDecodedPictureHashType != HashType::NONE)
      {
        std::vector<uint16_t> subPicIdsInPic;
        xGetSubpicIdsInPic(subPicIdsInPic, pic->m_cs->sps, pps);
        uint16_t maxSubpicIdInPic =
          subPicIdsInPic.size() == 0 ? 0 : *std::max_element(subPicIdsInPic.begin(), subPicIdsInPic.end());
        for (int subPicIdx = 0; subPicIdx < numSubpics; subPicIdx++)
        {
          const SubPic          &subpic                = pps->m_subPics[subPicIdx];
          const UnitArea         area                  = UnitArea(pcSlice->m_sps->m_chromaFormatIdc,
                                                                  Area(subpic.m_subPicLeft, subpic.m_subPicTop, subpic.m_subPicWidthInLumaSample,
                                                                       subpic.m_subPicHeightInLumaSample));
          PelUnitBuf             recoBuf               = pic->m_cs->getRecoBuf(area);
          SEIDecodedPictureHash *decodedPictureHashSEI = new SEIDecodedPictureHash();
          m_seiEncoder.initDecodedPictureHashSEI(decodedPictureHashSEI, recoBuf, subPicDigest,
                                                 pcSlice->m_sps->m_bitDepths);
          SEIMessages nestedSEI;
          nestedSEI.push_back(decodedPictureHashSEI);
          const std::vector<uint16_t> subPicIds = { (uint16_t)subpic.m_subPicID };
          std::vector<int>            targetOLS;
          std::vector<int>            targetLayers = { pic->m_layerId };
          xCreateScalableNestingSEI(trailingSeiMessages, nestedSEI, targetOLS, targetLayers, subPicIds,
                                    maxSubpicIdInPic);
        }
      }

      m_encCfg->m_RPLList0[gopId].m_isEncoded = true;
      m_encCfg->m_RPLList1[gopId].m_isEncoded = true;
      m_encCfg->m_GOPList[gopId].m_isEncoded  = true;

      double PSNR_Y;
      xCalculateAddPSNRs(isField, isTff, gopId, pic, accessUnit, rcListPic, encTime, snr_conversion, printFrameMSE,
                         printMSSSIM, &PSNR_Y, isEncodeLtRef);

      xWriteTrailingSEIMessages(trailingSeiMessages, accessUnit, pcSlice->m_uiTLayer);

      printHash(m_encCfg->m_seiCfg.m_decodedPictureHashSEIType, digestStr);

      if (m_encCfg->m_RCEnableRateControl)
      {
        double avgQP     = m_pcRateCtrl->getRCPic()->calAverageQP();
        double avgLambda = m_pcRateCtrl->getRCPic()->calAverageLambda();
        if (avgLambda < 0.0)
        {
          avgLambda = lambda;
        }

        m_pcRateCtrl->getRCPic()->updateAfterPicture(actualHeadBits, actualTotalBits, avgQP, avgLambda,
                                                     pcSlice->isIRAP());
        m_pcRateCtrl->getRCPic()->addToPictureLsit(m_pcRateCtrl->getPicList());

        m_pcRateCtrl->getRCSeq()->updateAfterPic(actualTotalBits);
        if (!pcSlice->isIRAP())
        {
          m_pcRateCtrl->getRCGOP()->updateAfterPicture(actualTotalBits);
        }
        else   // for intra picture, the estimated bits are used to update the current status in the GOP
        {
          m_pcRateCtrl->getRCGOP()->updateAfterPicture(estimatedBits);
        }
        if (m_pcRateCtrl->getCpbSaturationEnabled())
        {
          m_pcRateCtrl->updateCpbState(actualTotalBits);
          msg(NOTICE, " [CPB %6d bits]", m_pcRateCtrl->getCpbState());
        }
      }
      xCreateFrameFieldInfoSEI(leadingSeiMessages, pcSlice, isField);
      xCreatePictureTimingSEI(m_encCfg->m_efficientFieldIRAPEnabled ? effFieldIRAPMap.GetIRAPGOPid() : 0,
                              leadingSeiMessages, nestedSeiMessages, duInfoSeiMessages, pcSlice, isField, duData);

      if (m_encCfg->m_seiCfg.m_scalableNestingSEIEnabled)
      {
        const SPS *sps = pcSlice->m_sps;
        const PPS *pps = pcSlice->m_pps;

        std::vector<uint16_t> subpicIDs;
        xGetSubpicIdsInPic(subpicIDs, sps, pps);
        uint16_t maxSubpicIdInPic  = subpicIDs.size() == 0 ? 0 : *std::max_element(subpicIDs.begin(), subpicIDs.end());
        // Note (KJS): Using targetOLS = 0, 1 is as random as encapsulating the same SEIs in scalable nesting.
        //             This can just be seen as example regarding how to write scalable nesting, not what to write.
        std::vector<int> targetOLS = { 0, 1 };
        std::vector<int> targetLayers;
        xCreateScalableNestingSEI(leadingSeiMessages, nestedSeiMessages, targetOLS, targetLayers, subpicIDs,
                                  maxSubpicIdInPic);
      }

      SEIMessages seiMessages =
        getSeisByType(leadingSeiMessages, SEI::PayloadType::NEURAL_NETWORK_POST_FILTER_CHARACTERISTICS);
      for (auto it = seiMessages.cbegin(); it != seiMessages.cend(); it++)
      {
        pic->m_SEIs.push_back(
          new SEINeuralNetworkPostFilterCharacteristics(*(SEINeuralNetworkPostFilterCharacteristics *)*it));
      }

      seiMessages = getSeisByType(leadingSeiMessages, SEI::PayloadType::NEURAL_NETWORK_POST_FILTER_ACTIVATION);
      for (auto it = seiMessages.cbegin(); it != seiMessages.cend(); it++)
      {
        pic->m_SEIs.push_back(new SEINeuralNetworkPostFilterActivation(*(SEINeuralNetworkPostFilterActivation *)*it));
      }

      seiMessages = getSeisByType(leadingSeiMessages, SEI::PayloadType::FRAME_PACKING);
      for (auto it = seiMessages.cbegin(); it != seiMessages.cend(); it++)
      {
        pic->m_SEIs.push_back(new SEIFramePacking(*(SEIFramePacking *)*it));
      }

      double seiBits = (double)xWriteLeadingSEIMessages(leadingSeiMessages, duInfoSeiMessages, accessUnit,
                                                        pcSlice->m_uiTLayer, pcSlice->m_sps, duData);
      m_gcAnalyzeAll.addBits(seiBits);
      xWriteDuSEIMessages(duInfoSeiMessages, accessUnit, pcSlice->m_uiTLayer, duData);

      m_AUWriterIf->outputAU(accessUnit);

      msg(NOTICE, "\n");
      fflush(stdout);
    }

    m_cntRightBottom = pcSlice->m_cntRightBottom;
    if (m_encCfg->m_intraPeriod > 1 && pcSlice->isIntra())
    {
      m_cntRightBottomIntra = m_cntRightBottom;
    }

    DTRACE_UPDATE(g_trace_ctx, (std::make_pair("final", 0)));

    pic->m_reconstructed = true;
    m_first              = false;
    m_numPicsCoded++;
    if (!(m_encCfg->m_compositeRefEnabled && isEncodeLtRef))
    {
      for (int i = pcSlice->m_uiTLayer; i < pcSlice->m_sps->m_maxSubLayers; i++)
      {
        m_totalCoded[i]++;
      }
    }
    /* logging: insert a newline at end of picture period */

    if (m_encCfg->m_efficientFieldIRAPEnabled)
    {
      gopId = effFieldIRAPMap.restoreGOPid(gopId);
    }

    bool bMCBP = pic->m_cs->sps->m_MCBP;
    bool bTMBP = pic->m_cs->sps->m_TMBP;
    if (!bMCBP && !bTMBP)
    {
      // use repetitive padding
      pic->extendPicBorder(pcSlice->m_pps);
    }
    else if (!bMCBP && bTMBP)
    {
      // use TM-padding (in extendPicBorder)
      pic->extendPicBorder(pcSlice->m_pps);
    }
    else if (bMCBP && !bTMBP)
    {
      // use MC-padding
      m_pcEncLib->m_cInterSearch.mcFramePad(pic, *(pic->m_cs->slice));
    }
    else
    {
      // use TM-padding for I-frames only
      if (pcSlice->isIntra())
      {
        pic->extendPicBorder(pcSlice->m_pps);
      }
      else
      {
        m_pcEncLib->m_cInterSearch.mcFramePad(pic, *(pic->m_cs->slice));
      }
    }

    pcSlice->freeScaledRefPicList(scaledRefPic);

    pic->destroyTempBuffers();
    pic->m_cs->destroyCoeffs();
    if (pic->m_referenced && pcSlice->m_sps->m_tempPartPredEnabledFlag)
    {
      pic->m_cs->setSplitPred();
    }
    pic->m_cs->releaseIntermediateData();
    PROFILER_STOP(g_timeProfiler);
  }   // gopId-loop

  delete pcBitstreamRedirect;

  CHECK(m_numPicsCoded > 1, "Unspecified error");
}

void EncGOP::printOutSummary(uint32_t numAllPicCoded, bool isField, const bool printMSEBasedSNR,
                             const bool printSequenceMSE, const bool printMSSSIM, const bool printHexPsnr,
                             const bool printRprPsnr, const BitDepths &bitDepths, int layerId)
{
#if ENABLE_QPA
  const bool useWPSNR = m_encCfg->m_bUseWPSNR;
#endif
#if WCG_WPSNR
  const bool useLumaWPSNR = m_encCfg->m_printWPSNR;
#endif

  if (m_encCfg->m_decodeBitstreams[0].empty() && m_encCfg->m_decodeBitstreams[1].empty() &&
      m_encCfg->m_fastForwardToPOC < 0)
  {
    CHECK(!(numAllPicCoded == m_gcAnalyzeAll.getNumPic()), "Unspecified error");
  }

  const double picRate = m_encCfg->m_frameRate * (isField ? 2.0 : 1.0) / m_encCfg->m_temporalSubsampleRatio;

  m_gcAnalyzeAll.setFrameRate(picRate);
  m_gcAnalyzeI.setFrameRate(picRate);
  m_gcAnalyzeP.setFrameRate(picRate);
  m_gcAnalyzeB.setFrameRate(picRate);
#if WCG_WPSNR
  if (useLumaWPSNR)
  {
    m_gcAnalyzeWPSNR.setFrameRate(picRate);
  }
#endif

  const ChromaFormat chFmt = m_encCfg->m_chromaFormatIdc;

  //-- all
  msg(INFO, "\n");
  msg(DETAILS, "\nSUMMARY --------------------------------------------------------\n");
#if JVET_O0756_CALCULATE_HDRMETRICS
  const bool calculateHdrMetrics = m_pcEncLib->m_calculateHdrMetrics;
#else
  const bool calculateHdrMetrics = false;
#endif

  std::string header, metrics;
  std::string id = "a";
  id += layerId == 0 ? " " : std::to_string(layerId);
  m_gcAnalyzeAll.printOut(header, metrics, id, chFmt, printMSEBasedSNR, printSequenceMSE, printMSSSIM, printHexPsnr,
                          printRprPsnr, bitDepths, useWPSNR, calculateHdrMetrics);
  if (g_verbosity >= INFO)
  {
    std::cout << header << '\n' << metrics << std::endl;
  }

  id = "i";
  id += layerId == 0 ? " " : std::to_string(layerId);
  m_gcAnalyzeI.printOut(header, metrics, id, chFmt, printMSEBasedSNR, printSequenceMSE, printMSSSIM, printHexPsnr,
                        printRprPsnr, bitDepths, false, false);
  if (g_verbosity >= DETAILS)
  {
    std::cout << "\n\nI Slices--------------------------------------------------------\n"
              << header << '\n'
              << metrics << std::endl;
  }

  id = "p";
  id += layerId == 0 ? " " : std::to_string(layerId);
  m_gcAnalyzeP.printOut(header, metrics, id, chFmt, printMSEBasedSNR, printSequenceMSE, printMSSSIM, printHexPsnr,
                        printRprPsnr, bitDepths, false, false);
  if (g_verbosity >= DETAILS)
  {
    std::cout << "\n\nP Slices--------------------------------------------------------\n"
              << header << '\n'
              << metrics << std::endl;
  }

  id = "b";
  id += layerId == 0 ? " " : std::to_string(layerId);
  m_gcAnalyzeB.printOut(header, metrics, id, chFmt, printMSEBasedSNR, printSequenceMSE, printMSSSIM, printHexPsnr,
                        printRprPsnr, bitDepths, false, false);
  if (g_verbosity >= DETAILS)
  {
    std::cout << "\n\nB Slices--------------------------------------------------------\n"
              << header << '\n'
              << metrics << std::endl;
  }

#if WCG_WPSNR
  if (useLumaWPSNR)
  {
    id = "w";
    id += layerId == 0 ? " " : std::to_string(layerId);
    m_gcAnalyzeWPSNR.printOut(header, metrics, id, chFmt, printMSEBasedSNR, printSequenceMSE, printMSSSIM, printHexPsnr,
                              printRprPsnr, bitDepths, useLumaWPSNR, false);
    if (g_verbosity >= DETAILS)
    {
      std::cout << "\nWPSNR SUMMARY --------------------------------------------------------\n"
                << header << '\n'
                << metrics << std::endl;
    }
  }
#endif

  if (!m_encCfg->m_summaryOutFilename.empty())
  {
    m_gcAnalyzeAll.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths, m_encCfg->m_summaryOutFilename);
  }

  if (!m_encCfg->m_summaryPicFilenameBase.empty())
  {
    m_gcAnalyzeI.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths,
                              m_encCfg->m_summaryPicFilenameBase + "I.txt");
    m_gcAnalyzeP.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths,
                              m_encCfg->m_summaryPicFilenameBase + "P.txt");
    m_gcAnalyzeB.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths,
                              m_encCfg->m_summaryPicFilenameBase + "B.txt");
  }

#if WCG_WPSNR
  if (!m_encCfg->m_summaryOutFilename.empty() && useLumaWPSNR)
  {
    m_gcAnalyzeWPSNR.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths, m_encCfg->m_summaryOutFilename);
  }
#endif
  if (isField)
  {
    //-- interlaced summary
    m_gcAnalyzeAllField.setFrameRate(m_encCfg->m_frameRate / (double)m_encCfg->m_temporalSubsampleRatio);
    m_gcAnalyzeAllField.setBits(m_gcAnalyzeAll.getBits());
    // prior to the above statement, the interlace analyser does not contain the correct total number of bits.
    id = "a";
    id += layerId == 0 ? " " : std::to_string(layerId);
    m_gcAnalyzeAllField.printOut(header, metrics, id, chFmt, printMSEBasedSNR, printSequenceMSE, printMSSSIM,
                                 printHexPsnr, printRprPsnr, bitDepths, useWPSNR, false);
    if (g_verbosity >= DETAILS)
    {
      std::cout << "\n\nSUMMARY INTERLACED ---------------------------------------------\n"
                << header << '\n'
                << metrics << std::endl;
    }
    if (!m_encCfg->m_summaryOutFilename.empty())
    {
      m_gcAnalyzeAllField.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths,
                                       m_encCfg->m_summaryOutFilename);
#if WCG_WPSNR
      if (useLumaWPSNR)
      {
        m_gcAnalyzeWPSNR.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths, m_encCfg->m_summaryOutFilename);
      }
#endif
    }
  }

  msg(DETAILS, "\nRVM: %.3lf\n", xCalculateRVM());
}

uint64_t EncGOP::preLoopFilterPicAndCalcDist(Picture *pic)
{
  CodingStructure &cs = *pic->m_cs;
  m_pcLoopFilter->deblockingFilterPic(cs);

  const CPelUnitBuf picOrg = pic->getRecoBuf();
  const CPelUnitBuf picRec = cs.getRecoBuf();

  uint64_t dist = 0;
  for (uint32_t comp = 0; comp < (uint32_t)picRec.bufs.size(); comp++)
  {
    const CompID   compID = CompID(comp);
    const uint32_t rshift = 2 * DISTORTION_PRECISION_ADJUSTMENT(cs.sps->m_bitDepths[toChannelType(compID)]);
#if ENABLE_QPA
    CHECK(rshift >= 8, "shifts greater than 7 are not supported.");
#endif
    dist += xFindDistortionPlane(picOrg.get(compID), picRec.get(compID), rshift);
  }
  return dist;
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================
void EncGOP::xInitGOP(int pocLast, int numPicRcvd, bool isField, bool isEncodeLtRef)
{
  CHECK(!(numPicRcvd > 0), "Unspecified error");
  //  Exception for the first frames
  if ((isField && (pocLast == 0 || pocLast == 1)) || (!isField && (pocLast == 0)) || isEncodeLtRef)
  {
    m_iGopSize = 1;
  }
  else
  {
    m_iGopSize = m_encCfg->m_gopSize;
  }
  CHECK(!(m_iGopSize > 0), "Unspecified error");

  return;
}

void EncGOP::xGetBuffer(PicList &rcListPic, std::list<PelUnitBuf *> &rcListPicYuvRecOut, int numPicRcvd, int timeOffset,
                        Picture *&rpic, int pocCurr, bool isField)
{
  int                               i;
  //  Rec. output
  std::list<PelUnitBuf *>::iterator iterPicYuvRec = rcListPicYuvRecOut.end();

  if (isField && pocCurr > 1 && m_iGopSize != 1)
  {
    timeOffset--;
  }

  int multipleFactor = m_encCfg->m_compositeRefEnabled ? 2 : 1;
  for (i = 0; i < (numPicRcvd * multipleFactor - timeOffset + 1); i += multipleFactor)
  {
    iterPicYuvRec--;
  }

  //  Current pic.
  PicList::iterator iterPic = rcListPic.begin();
  while (iterPic != rcListPic.end())
  {
    rpic = *(iterPic);
    if (rpic->m_poc == pocCurr && rpic->m_layerId == m_pcEncLib->m_layerId)
    {
      break;
    }
    iterPic++;
  }

  CHECK(!(rpic != nullptr), "Unspecified error");
  CHECK(!(rpic->m_poc == pocCurr), "Unspecified error");

  (**iterPicYuvRec) = rpic->getRecoBuf();
  return;
}

void EncGOP::xGetSubpicIdsInPic(std::vector<uint16_t> &subpicIDs, const SPS *sps, const PPS *pps)
{
  subpicIDs.clear();

  if (sps->m_subPicInfoPresentFlag)
  {
    if (sps->m_subPicIdMappingExplicitlySignalledFlag)
    {
      if (sps->m_subPicIdMappingPresentFlag)
      {
        subpicIDs = sps->m_subPicId;
      }
      else
      {
        subpicIDs = pps->m_subPicId;
      }
    }
    else
    {
      const int numSubPics = sps->m_numSubPics;
      subpicIDs.resize(numSubPics);
      for (int i = 0; i < numSubPics; i++)
      {
        subpicIDs[i] = (uint16_t)i;
      }
    }
  }
}

#if ENABLE_QPA

#ifndef BETA
#define BETA 0.5   // value between 0.0 and 1; use 0.0 to obtain traditional PSNR
#endif

static inline double calcWeightedSquaredError(const CPelBuf &org, const CPelBuf &rec, double &sumAct,
                                              const uint32_t bitDepth, const uint32_t imageWidth,
                                              const uint32_t imageHeight, const uint32_t offsetX,
                                              const uint32_t offsetY, int blockWidth, int blockHeight)
{
  const ptrdiff_t O    = org.stride;
  const ptrdiff_t R    = rec.stride;
  const Pel      *o    = org.bufAt(offsetX, offsetY);
  const Pel      *r    = rec.bufAt(offsetX, offsetY);
  const int       yAct = offsetY > 0 ? 0 : 1;
  const int       xAct = offsetX > 0 ? 0 : 1;

  if (offsetY + (uint32_t)blockHeight > imageHeight)
  {
    blockHeight = imageHeight - offsetY;
  }
  if (offsetX + (uint32_t)blockWidth > imageWidth)
  {
    blockWidth = imageWidth - offsetX;
  }

  const int hAct  = offsetY + (uint32_t)blockHeight < imageHeight ? blockHeight : blockHeight - 1;
  const int wAct  = offsetX + (uint32_t)blockWidth < imageWidth ? blockWidth : blockWidth - 1;
  uint64_t  ssErr = 0;   // sum of squared diffs
  uint64_t  saAct = 0;   // sum of abs. activity
  double    msAct;
  int       x, y;

  // calculate image differences and activity
  for (y = 0; y < blockHeight; y++)   // error
  {
    for (x = 0; x < blockWidth; x++)
    {
      const int64_t iDiff = (int64_t)o[y * O + x] - (int64_t)r[y * R + x];
      ssErr += uint64_t(iDiff * iDiff);
    }
  }
  if (wAct <= xAct || hAct <= yAct)
  {
    return (double)ssErr;
  }

  for (y = yAct; y < hAct; y++)   // activity
  {
    for (x = xAct; x < wAct; x++)
    {
      const int f = 12 * (int)o[y * O + x] -
        2 * ((int)o[y * O + x - 1] + (int)o[y * O + x + 1] + (int)o[(y - 1) * O + x] + (int)o[(y + 1) * O + x]) -
        (int)o[(y - 1) * O + x - 1] - (int)o[(y - 1) * O + x + 1] - (int)o[(y + 1) * O + x - 1] -
        (int)o[(y + 1) * O + x + 1];
      saAct += abs(f);
    }
  }

  // calculate weight (mean squared activity)
  msAct = (double)saAct / (double(wAct - xAct) * double(hAct - yAct));

  // lower limit, accounts for high-pass gain
  if (msAct < double(1 << (bitDepth - 4)))
  {
    msAct = double(1 << (bitDepth - 4));
  }

  msAct *= msAct;   // because ssErr is squared

  sumAct += msAct;   // includes high-pass gain

  // calculate activity weighted error square
  return (double)ssErr * pow(msAct, -1.0 * BETA);
}
#endif   // ENABLE_QPA

uint64_t EncGOP::xFindDistortionPlane(const CPelBuf &pic0, const CPelBuf &pic1, const uint32_t rshift
#if ENABLE_QPA
                                      ,
                                      const uint32_t chromaShiftHor /*= 0*/, const uint32_t chromaShiftVer /*= 0*/
#endif
)
{
  uint64_t   totalDiff;
  const Pel *pSrc0 = pic0.bufAt(0, 0);
  const Pel *pSrc1 = pic1.bufAt(0, 0);

  CHECK(pic0.width != pic1.width, "Unspecified error");
  CHECK(pic0.height != pic1.height, "Unspecified error");

  if (rshift > 0)
  {
#if ENABLE_QPA
    const uint32_t BD = rshift;   // image bit-depth
    if (BD >= 8)
    {
      const uint32_t W = pic0.width;   // image width
      const uint32_t H = pic0.height;   // image height
      const double   R = double(W * H) / (1920.0 * 1080.0);
      const uint32_t B = Clip3<uint32_t>(
        0, 128 >> chromaShiftVer,
        4 * uint32_t(16.0 * sqrt(R) + 0.5));   // WPSNR block size in integer multiple of 4 (for SIMD, = 64 at full-HD)

      uint32_t x, y;

      if (B < 4)   // image is too small to use WPSNR, resort to traditional PSNR
      {
        totalDiff = 0;
        for (y = 0; y < H; y++)
        {
          for (x = 0; x < W; x++)
          {
            const int64_t iDiff = (int64_t)pSrc0[x] - (int64_t)pSrc1[x];
            totalDiff += uint64_t(iDiff * iDiff);
          }
          pSrc0 += pic0.stride;
          pSrc1 += pic1.stride;
        }
        return totalDiff;
      }

      double wmse = 0.0, sumAct = 0.0;   // compute activity normalized SNR value

      for (y = 0; y < H; y += B)
      {
        for (x = 0; x < W; x += B)
        {
          wmse += calcWeightedSquaredError(pic1, pic0, sumAct, BD, W, H, x, y, B, B);
        }
      }

      // integer weighted distortion
      sumAct = 16.0 * sqrt((3840.0 * 2160.0) / double((W << chromaShiftHor) * (H << chromaShiftVer))) *
        double(1 << (2 * BD - 10));

      return (wmse <= 0.0) ? 0 : uint64_t(wmse * pow(sumAct, BETA) + 0.5);
    }
#endif   // ENABLE_QPA
    totalDiff = 0;
    for (int y = 0; y < pic0.height; y++)
    {
      for (int x = 0; x < pic0.width; x++)
      {
        Intermediate_Int temp = pSrc0[x] - pSrc1[x];
        totalDiff += uint64_t((temp * temp) >> rshift);
      }
      pSrc0 += pic0.stride;
      pSrc1 += pic1.stride;
    }
  }
  else
  {
    totalDiff = 0;
    for (int y = 0; y < pic0.height; y++)
    {
      for (int x = 0; x < pic0.width; x++)
      {
        Intermediate_Int temp = pSrc0[x] - pSrc1[x];
        totalDiff += uint64_t(temp * temp);
      }
      pSrc0 += pic0.stride;
      pSrc1 += pic1.stride;
    }
  }

  return totalDiff;
}
#if WCG_WPSNR
double EncGOP::xFindDistortionPlaneWPSNR(const CPelBuf &pic0, const CPelBuf &pic1, const uint32_t rshift,
                                         const CPelBuf &picLuma0, CompID compID, const ChromaFormat chfmt)
{
  const bool useLumaWPSNR = m_encCfg->m_printWPSNR;
  if (!useLumaWPSNR)
  {
    return 0;
  }

  double     totalDiffWpsnr;
  const Pel *pSrc0    = pic0.bufAt(0, 0);
  const Pel *pSrc1    = pic1.bufAt(0, 0);
  const Pel *pSrcLuma = picLuma0.bufAt(0, 0);
  CHECK(pic0.width != pic1.width, "Unspecified error");
  CHECK(pic0.height != pic1.height, "Unspecified error");

  if (rshift > 0)
  {
    totalDiffWpsnr = 0;
    for (int y = 0; y < pic0.height; y++)
    {
      for (int x = 0; x < pic0.width; x++)
      {
        Intermediate_Int temp = pSrc0[x] - pSrc1[x];
        double           dW =
          m_pcEncLib->getRdCost()->getWPSNRLumaLevelWeight(pSrcLuma[(x << getComponentScaleX(compID, chfmt))]);
        totalDiffWpsnr += ((dW * (double)temp * (double)temp)) * (double)(1 >> rshift);
      }
      pSrc0 += pic0.stride;
      pSrc1 += pic1.stride;
      pSrcLuma += picLuma0.stride << getComponentScaleY(compID, chfmt);
    }
  }
  else
  {
    totalDiffWpsnr = 0;
    for (int y = 0; y < pic0.height; y++)
    {
      for (int x = 0; x < pic0.width; x++)
      {
        Intermediate_Int temp = pSrc0[x] - pSrc1[x];
        double dW = m_pcEncLib->getRdCost()->getWPSNRLumaLevelWeight(pSrcLuma[x << getComponentScaleX(compID, chfmt)]);
        totalDiffWpsnr += dW * (double)temp * (double)temp;
      }
      pSrc0 += pic0.stride;
      pSrc1 += pic1.stride;
      pSrcLuma += picLuma0.stride << getComponentScaleY(compID, chfmt);
    }
  }

  return totalDiffWpsnr;
}
#endif

void EncGOP::xCalculateAddPSNRs(const bool isField, const bool isFieldTopFieldFirst, const int gopId, Picture *pic,
                                const AccessUnit &accessUnit, PicList &rcListPic, const int64_t dEncTime,
                                const InputColourSpaceConversion snr_conversion, const bool printFrameMSE,
                                const bool printMSSSIM, double *PSNR_Y, bool isEncodeLtRef)
{
  xCalculateAddPSNR(pic, pic->getRecoBuf(), accessUnit, (double)dEncTime, snr_conversion, printFrameMSE, printMSSSIM,
                    PSNR_Y, isEncodeLtRef);

  // In case of field coding, compute the interlaced PSNR for both fields
  if (isField)
  {
    bool bothFieldsAreEncoded  = false;
    int  correspondingFieldPOC = pic->m_poc;
    int  currentPicGOPPoc      = m_encCfg->m_GOPList[gopId].m_POC;
    if (pic->m_poc == 0)
    {
      // particular case for POC 0 and 1.
      // If they are not encoded first and separately from other pictures, we need to change this
      // POC 0 is always encoded first then POC 1 is encoded
      bothFieldsAreEncoded = false;
    }
    else if (pic->m_poc == 1)
    {
      // if we are at POC 1, POC 0 has been encoded for sure
      correspondingFieldPOC = 0;
      bothFieldsAreEncoded  = true;
    }
    else
    {
      if (pic->m_poc % 2 == 1)
      {
        correspondingFieldPOC -=
          1;   // all odd POC are associated with the preceding even POC (e.g poc 1 is associated to poc 0)
        currentPicGOPPoc -= 1;
      }
      else
      {
        correspondingFieldPOC +=
          1;   // all even POC are associated with the following odd POC (e.g poc 0 is associated to poc 1)
        currentPicGOPPoc += 1;
      }
      for (int i = 0; i < m_iGopSize; i++)
      {
        if (m_encCfg->m_GOPList[i].m_POC == currentPicGOPPoc)
        {
          bothFieldsAreEncoded = m_encCfg->m_GOPList[i].m_isEncoded;
          break;
        }
      }
    }

    if (bothFieldsAreEncoded)
    {
      // get complementary top field
      PicList::iterator iterPic = rcListPic.begin();
      while ((*iterPic)->m_poc != correspondingFieldPOC)
      {
        iterPic++;
      }
      Picture *correspondingFieldPic = *(iterPic);

      if ((pic->m_topField && isFieldTopFieldFirst) || (!pic->m_topField && !isFieldTopFieldFirst))
      {
        xCalculateInterlacedAddPSNR(pic, correspondingFieldPic, pic->getRecoBuf(), correspondingFieldPic->getRecoBuf(),
                                    snr_conversion, printFrameMSE, printMSSSIM, PSNR_Y, isEncodeLtRef);
      }
      else
      {
        xCalculateInterlacedAddPSNR(correspondingFieldPic, pic, correspondingFieldPic->getRecoBuf(), pic->getRecoBuf(),
                                    snr_conversion, printFrameMSE, printMSSSIM, PSNR_Y, isEncodeLtRef);
      }
    }
  }
}

void EncGOP::xCalculateAddPSNR(Picture *pic, PelUnitBuf cPicD, const AccessUnit &accessUnit, double dEncTime,
                               const InputColourSpaceConversion conversion, const bool printFrameMSE,
                               const bool printMSSSIM, double *PSNR_Y, bool isEncodeLtRef)
{
  const SPS         &sps = *pic->m_cs->sps;
  const CPelUnitBuf &rec = cPicD;
  CHECK(!(conversion == IPCOLOURSPACE_UNCHANGED), "Unspecified error");
  //  const CPelUnitBuf& org = (conversion != IPCOLOURSPACE_UNCHANGED) ? rec->getPicYuvTrueOrg()->getBuf() :
  //  rec->getPicYuvOrg()->getBuf();
  const CPelUnitBuf &org =
    (sps.m_lmcsEnabled || m_encCfg->m_gopBasedTemporalFilterEnabled) ? pic->getTrueOrigBuf() : pic->getOrigBuf();
#if ENABLE_QPA
  const bool useWPSNR = m_encCfg->m_bUseWPSNR;
#endif
  double dPSNR[MAX_NUM_COMP];
  double msssim[MAX_NUM_COMP] = { 0.0 };
#if WCG_WPSNR
  const bool useLumaWPSNR = m_encCfg->m_printWPSNR;
  double     dPSNRWeighted[MAX_NUM_COMP];
  double     MSEyuvframeWeighted[MAX_NUM_COMP];
  double     upscaledPSNRWeighted[MAX_NUM_COMP];
#endif
  double upscaledPSNR[MAX_NUM_COMP];
  double upscaledMsssim[MAX_NUM_COMP];
  for (int i = 0; i < MAX_NUM_COMP; i++)
  {
    dPSNR[i] = 0.0;
#if WCG_WPSNR
    dPSNRWeighted[i]        = 0.0;
    MSEyuvframeWeighted[i]  = 0.0;
    upscaledPSNRWeighted[i] = 0.0;
#endif
    upscaledPSNR[i]   = 0.0;
    upscaledMsssim[i] = 0.0;
  }
#if JVET_O0756_CALCULATE_HDRMETRICS
  double deltaE[hdrtoolslib::NB_REF_WHITE];
  double psnrL[hdrtoolslib::NB_REF_WHITE];
  for (int i = 0; i < hdrtoolslib::NB_REF_WHITE; i++)
  {
    deltaE[i] = 0.0;
    psnrL[i]  = 0.0;
  }
#endif

  PelStorage interm;

  if (conversion != IPCOLOURSPACE_UNCHANGED)
  {
    interm.create(rec.chromaFormat, Area(Position(), rec.Y()));
    VideoIOYuv::ColourSpaceConvert(rec, interm, conversion, false);
  }

  const CPelUnitBuf &picC = (conversion == IPCOLOURSPACE_UNCHANGED) ? rec : interm;

  //===== calculate PSNR =====
  double             mseYuvFrame[MAX_NUM_COMP] = { 0, 0, 0 };
  const ChromaFormat formatD                   = rec.chromaFormat;
  const ChromaFormat format                    = sps.m_chromaFormatIdc;

  const bool   bPicIsField = pic->m_fieldPic;
  const Slice *pcSlice     = pic->m_slices[0];

  PelStorage upscaledRec;
  uint32_t   decodedLumaWidth  = 0;
  uint32_t   decodedLumaHeight = 0;

  if (m_encCfg->m_resChangeInClvsEnabled)
  {
    const CPelBuf &upscaledOrg = (sps.m_lmcsEnabled || m_encCfg->m_gopBasedTemporalFilterEnabled)
      ? pic->m_bufs[PIC_TRUE_ORIGINAL_INPUT].get(COMP_Y)
      : pic->m_bufs[PIC_ORIGINAL_INPUT].get(COMP_Y);
    upscaledRec.create(rec.chromaFormat, Area(Position(), upscaledOrg));

    ScalingRatio scalingRatio;
    // it is assumed that full resolution picture PPS has ppsId 0
    const PPS   *pps = m_pcEncLib->getPPS(0);
    CU::getRprScaling(&sps, pps, pic, scalingRatio);

    bool rescaleForDisplay = true;
    Picture::rescalePicture(scalingRatio, picC, pic->m_scalingWindow, upscaledRec, pps->m_scalingWindow, format,
                            sps.m_bitDepths, false, false, sps.m_horCollocatedChromaFlag, sps.m_verCollocatedChromaFlag,
                            rescaleForDisplay, m_encCfg->m_upscaleFilterForDisplay);
  }

  Picture *picRefLayer = nullptr;
  if (m_encCfg->m_refLayerMetricsEnabled)
  {
    const VPS *vps = pic->m_cs->vps;
    if (vps && m_encCfg->m_numRefLayers[vps->m_generalLayerIdx[pic->m_layerId]] > 0)
    {
      int layerIdx   = vps->m_generalLayerIdx[pic->m_layerId];
      int refLayerId = vps->m_vpsLayerId[vps->m_directRefLayerIdx[layerIdx][0]];

      for (Picture *p: *m_pcEncLib->getListPic())
      {
        if (p->m_layerId == refLayerId && p->m_poc == pic->m_poc)
        {
          picRefLayer = p;
          break;
        }
      }
      if (picRefLayer)
      {
        const CPelUnitBuf &pub1      = org;
        const CPelUnitBuf &pub0      = picRefLayer->getRecoBuf();
        Window            &wScaling0 = picRefLayer->m_scalingWindow;
        Window            &wScaling1 = pic->m_scalingWindow;
        int                w0        = pub0.get(COMP_Y).width -
          SPS::getWinUnitX(sps.m_chromaFormatIdc) * (wScaling0.m_winLeftOffset + wScaling0.m_winRightOffset);
        int h0 = pub0.get(COMP_Y).height -
          SPS::getWinUnitY(sps.m_chromaFormatIdc) * (wScaling0.m_winTopOffset + wScaling0.m_winBottomOffset);
        int w1 = pub1.get(COMP_Y).width -
          SPS::getWinUnitX(sps.m_chromaFormatIdc) * (wScaling1.m_winLeftOffset + wScaling1.m_winRightOffset);
        int h1 = pub1.get(COMP_Y).height -
          SPS::getWinUnitY(sps.m_chromaFormatIdc) * (wScaling1.m_winTopOffset + wScaling1.m_winBottomOffset);
        int          xScale       = ((w0 << ScalingRatio::BITS) + (w1 >> 1)) / w1;
        int          yScale       = ((h0 << ScalingRatio::BITS) + (h1 >> 1)) / h1;
        ScalingRatio scalingRatio = { xScale, yScale };

        if (m_refLayerRescaledPicYuv == nullptr)
        {
          m_refLayerRescaledPicYuv = new PelStorage();
          m_refLayerRescaledPicYuv->create(pub1.chromaFormat, Area(Position(), pub1.get(COMP_Y)));
        }

        Picture::rescalePicture(scalingRatio, pub0, wScaling0, *m_refLayerRescaledPicYuv, wScaling1, format,
                                sps.m_bitDepths, false, false, sps.m_horCollocatedChromaFlag,
                                sps.m_verCollocatedChromaFlag);
        m_pcEncLib->setRefLayerRescaledAvailable(true);
      }
    }
  }

  for (int comp = 0; comp < ::getNumberValidComponents(formatD); comp++)
  {
    const CompID   compID = CompID(comp);
    const CPelBuf &p      = picC.get(compID);
    const CPelBuf &o      = org.get(compID);

    CHECK(!(p.width == o.width), "Unspecified error");
    CHECK(!(p.height == o.height), "Unspecified error");

    int padX = m_encCfg->m_sourcePadding[0];
    int padY = m_encCfg->m_sourcePadding[1];

    // when RPR is enabled, picture padding is picture specific due to possible different picture resoluitons, however
    // only full resolution padding is stored in EncLib get per picture padding from the conformance window, in this
    // case if conformance window is set not equal to the padding then PSNR results may be inaccurate
    if (m_encCfg->m_resChangeInClvsEnabled)
    {
      Window &conf = pic->m_conformanceWindow;
      padX         = conf.m_winRightOffset * SPS::getWinUnitX(format);
      padY         = conf.m_winBottomOffset * SPS::getWinUnitY(format);
    }

    const uint32_t width  = p.width - (padX >> ::getComponentScaleX(compID, format));
    const uint32_t height = p.height - (padY >> (!!bPicIsField + ::getComponentScaleY(compID, format)));
    if (comp == 0)
    {
      decodedLumaWidth  = width;
      decodedLumaHeight = height;
    }
    // create new buffers with correct dimensions
    const CPelBuf  recPB(p.bufAt(0, 0), p.stride, width, height);
    const CPelBuf  orgPB(o.bufAt(0, 0), o.stride, width, height);
    const uint32_t bitDepth = sps.m_bitDepths[toChannelType(compID)];
#if ENABLE_QPA
    const uint64_t ssdTemp =
      xFindDistortionPlane(recPB, orgPB, useWPSNR ? bitDepth : 0, ::getComponentScaleX(compID, format),
                           ::getComponentScaleY(compID, format));
#else
    const uint64_t ssdTemp = xFindDistortionPlane(recPB, orgPB, 0);
#endif
    const uint32_t maxval    = 255 << (bitDepth - 8);
    const uint32_t size      = width * height;
    const double   fRefValue = (double)maxval * maxval * size;
    dPSNR[comp]              = ssdTemp ? 10.0 * log10(fRefValue / (double)ssdTemp) : 999.99;
    mseYuvFrame[comp]        = (double)ssdTemp / size;
    if (printMSSSIM)
    {
      msssim[comp] = xCalculateMSSSIM(o.bufAt(0, 0), o.stride, p.bufAt(0, 0), p.stride, width, height, bitDepth);
    }
#if WCG_WPSNR
    const double uiSSDtempWeighted = xFindDistortionPlaneWPSNR(recPB, orgPB, 0, org.get(COMP_Y), compID, format);
    if (useLumaWPSNR)
    {
      dPSNRWeighted[comp]       = uiSSDtempWeighted ? 10.0 * log10(fRefValue / (double)uiSSDtempWeighted) : 999.99;
      MSEyuvframeWeighted[comp] = (double)uiSSDtempWeighted / size;
    }
#endif

    if (m_encCfg->m_resChangeInClvsEnabled)
    {
      const CPelBuf &upscaledOrg = (sps.m_lmcsEnabled || m_encCfg->m_gopBasedTemporalFilterEnabled)
        ? pic->m_bufs[PIC_TRUE_ORIGINAL_INPUT].get(compID)
        : pic->m_bufs[PIC_ORIGINAL_INPUT].get(compID);

      const uint32_t upscaledWidth =
        upscaledOrg.width - (m_encCfg->m_sourcePadding[0] >> ::getComponentScaleX(compID, format));
      const uint32_t upscaledHeight =
        upscaledOrg.height - (m_encCfg->m_sourcePadding[1] >> (!!bPicIsField + ::getComponentScaleY(compID, format)));

      // create new buffers with correct dimensions
      const CPelBuf upscaledRecPB(upscaledRec.get(compID).bufAt(0, 0), upscaledRec.get(compID).stride, upscaledWidth,
                                  upscaledHeight);
      const CPelBuf upscaledOrgPB(upscaledOrg.bufAt(0, 0), upscaledOrg.stride, upscaledWidth, upscaledHeight);

#if ENABLE_QPA
      const uint64_t upscaledSSD = xFindDistortionPlane(upscaledRecPB, upscaledOrgPB, useWPSNR ? bitDepth : 0,
                                                        ::getComponentScaleX(compID, format));
#else
      const uint64_t scaledSSD = xFindDistortionPlane(upsacledRecPB, upsacledOrgPB, 0);
#endif

      upscaledPSNR[comp] = upscaledSSD
        ? 10.0 * log10((double)maxval * maxval * upscaledWidth * upscaledHeight / (double)upscaledSSD)
        : 999.99;
      if (printMSSSIM)
      {
        upscaledMsssim[comp] =
          xCalculateMSSSIM(upscaledOrgPB.bufAt(0, 0), upscaledOrgPB.stride, upscaledRecPB.bufAt(0, 0),
                           upscaledRecPB.stride, upscaledWidth, upscaledHeight, bitDepth);
      }
#if WCG_WPSNR
      const double uiSSDtempWeighted =
        xFindDistortionPlaneWPSNR(upscaledRecPB, upscaledOrgPB, 0,
                                  (sps.m_lmcsEnabled || m_encCfg->m_gopBasedTemporalFilterEnabled)
                                    ? pic->m_bufs[PIC_TRUE_ORIGINAL_INPUT].get(COMP_Y)
                                    : pic->m_bufs[PIC_ORIGINAL_INPUT].get(COMP_Y),
                                  compID, format);
      if (useLumaWPSNR)
      {
        upscaledPSNRWeighted[comp] = uiSSDtempWeighted
          ? 10.0 * log10((double)maxval * maxval * upscaledWidth * upscaledHeight / (double)uiSSDtempWeighted)
          : 999.99;
      }
#endif
    }
    else if (picRefLayer)
    {
      const CPelBuf &p = m_refLayerRescaledPicYuv->get(compID);
      const CPelBuf &o = org.get(compID);
#if ENABLE_QPA
      const uint64_t upscaledSSD = xFindDistortionPlane(
        p, o, useWPSNR ? bitDepth : 0, ::getComponentScaleX(compID, format), ::getComponentScaleY(compID, format));
#else
      const uint64_t upscaledSSD = xFindDistortionPlane(p, o, 0);
#endif
      upscaledPSNR[comp] = upscaledSSD ? 10.0 * log10((double)fRefValue / (double)upscaledSSD) : 999.99;
    }
  }

#if EXTENSION_360_VIDEO
  m_ext360.calculatePSNRs(pic);
#endif

#if JVET_O0756_CALCULATE_HDRMETRICS
  const bool calculateHdrMetrics = m_pcEncLib->m_calculateHdrMetrics;
  if (calculateHdrMetrics)
  {
    auto beforeTime = std::chrono::steady_clock::now();
    xCalculateHDRMetrics(pic, deltaE, psnrL);
    auto elapsed = std::chrono::steady_clock::now() - beforeTime;
    m_metricTime += elapsed;
  }
#endif

  /* calculate the size of the access unit, excluding:
   *  - any AnnexB contributions (start_code_prefix, zero_byte, etc.,)
   *  - SEI NAL units
   */
  uint32_t numRBSPBytes = 0;
  for (AccessUnit::const_iterator it = accessUnit.begin(); it != accessUnit.end(); it++)
  {
    uint32_t numRBSPBytesNal = uint32_t((*it)->m_nalUnitData.str().size());
    if (m_encCfg->m_summaryVerboseness > 0)
    {
      msg(NOTICE, "*** %6s numBytesInNALunit: %u\n", nalUnitTypeToString((*it)->m_nalUnitType), numRBSPBytesNal);
    }
    if ((*it)->m_nalUnitType != NAL_UNIT_PREFIX_SEI && (*it)->m_nalUnitType != NAL_UNIT_SUFFIX_SEI)
    {
      numRBSPBytes += numRBSPBytesNal;
      if (it == accessUnit.begin() || (*it)->m_nalUnitType == NAL_UNIT_OPI || (*it)->m_nalUnitType == NAL_UNIT_VPS ||
          (*it)->m_nalUnitType == NAL_UNIT_DCI || (*it)->m_nalUnitType == NAL_UNIT_SPS ||
          (*it)->m_nalUnitType == NAL_UNIT_PPS || (*it)->m_nalUnitType == NAL_UNIT_PREFIX_APS ||
          (*it)->m_nalUnitType == NAL_UNIT_SUFFIX_APS)
      {
        numRBSPBytes += 4;
      }
      else
      {
        numRBSPBytes += 3;
      }
    }
  }

  uint32_t uibits = numRBSPBytes * 8;
  m_rvm.push_back(uibits);

  //===== add PSNR =====
  m_gcAnalyzeAll.setUpscaledOutput(m_encCfg->m_upscaledOutput);
  m_gcAnalyzeAll.addResult(dPSNR, (double)uibits, mseYuvFrame, upscaledPSNR, msssim, upscaledMsssim, isEncodeLtRef);
#if EXTENSION_360_VIDEO
  m_ext360.addResult(m_gcAnalyzeAll);
#endif
#if JVET_O0756_CALCULATE_HDRMETRICS
  if (calculateHdrMetrics)
  {
    m_gcAnalyzeAll.addHDRMetricsResult(deltaE, psnrL);
  }
#endif
  if (pcSlice->isIntra())
  {
    m_gcAnalyzeI.setUpscaledOutput(m_encCfg->m_upscaledOutput);
    m_gcAnalyzeI.addResult(dPSNR, (double)uibits, mseYuvFrame, upscaledPSNR, msssim, upscaledMsssim, isEncodeLtRef);
    *PSNR_Y = dPSNR[COMP_Y];
#if EXTENSION_360_VIDEO
    m_ext360.addResult(m_gcAnalyzeI);
#endif
#if JVET_O0756_CALCULATE_HDRMETRICS
    if (calculateHdrMetrics)
    {
      m_gcAnalyzeI.addHDRMetricsResult(deltaE, psnrL);
    }
#endif
  }
  if (pcSlice->isInterP())
  {
    m_gcAnalyzeP.setUpscaledOutput(m_encCfg->m_upscaledOutput);
    m_gcAnalyzeP.addResult(dPSNR, (double)uibits, mseYuvFrame, upscaledPSNR, msssim, upscaledMsssim, isEncodeLtRef);
    *PSNR_Y = dPSNR[COMP_Y];
#if EXTENSION_360_VIDEO
    m_ext360.addResult(m_gcAnalyzeP);
#endif
#if JVET_O0756_CALCULATE_HDRMETRICS
    if (calculateHdrMetrics)
    {
      m_gcAnalyzeP.addHDRMetricsResult(deltaE, psnrL);
    }
#endif
  }
  if (pcSlice->isInterB())
  {
    m_gcAnalyzeB.setUpscaledOutput(m_encCfg->m_upscaledOutput);
    m_gcAnalyzeB.addResult(dPSNR, (double)uibits, mseYuvFrame, upscaledPSNR, msssim, upscaledMsssim, isEncodeLtRef);
    *PSNR_Y = dPSNR[COMP_Y];
#if EXTENSION_360_VIDEO
    m_ext360.addResult(m_gcAnalyzeB);
#endif
#if JVET_O0756_CALCULATE_HDRMETRICS
    if (calculateHdrMetrics)
    {
      m_gcAnalyzeB.addHDRMetricsResult(deltaE, psnrL);
    }
#endif
  }
#if WCG_WPSNR
  if (useLumaWPSNR)
  {
    m_gcAnalyzeWPSNR.setUpscaledOutput(m_encCfg->m_upscaledOutput);
    m_gcAnalyzeWPSNR.addResult(dPSNRWeighted, (double)uibits, MSEyuvframeWeighted, upscaledPSNRWeighted, msssim,
                               upscaledMsssim, isEncodeLtRef);
  }
#endif

  char c = (pcSlice->isIntra() ? 'I' : pcSlice->isInterP() ? 'P' : 'B');
  if (!pic->m_referenced)
  {
    c += 32;
  }
  if (m_encCfg->m_seiCfg.m_dependentRAPIndicationSEIEnabled && pcSlice->m_isDRAP)
  {
    c = 'D';
  }
  if (m_encCfg->m_seiCfg.m_edrapIndicationSEIEnabled && pcSlice->m_edrapRapId > 0)
  {
    c = 'E';
  }

  if (g_verbosity >= NOTICE)
  {
    msg(NOTICE, "POC %4d LId: %2d TId: %1d ( %s, %c-SLICE, QP %d ) %10d bits", pcSlice->m_poc,
        pcSlice->m_pic->m_layerId, pcSlice->m_uiTLayer, nalUnitTypeToString(pcSlice->m_eNalUnitType), c,
        pcSlice->m_iSliceQp, uibits);

    if (m_encCfg->m_resChangeInClvsEnabled && m_encCfg->m_upscaledOutput == 2)
    {
      msg(NOTICE, " [Y %6.4lf dB  U %6.4lf dB  V %6.4lf dB]", upscaledPSNR[COMP_Y], upscaledPSNR[COMP_Cb],
          upscaledPSNR[COMP_Cr]);
    }
    else
    {
      msg(NOTICE, " [Y %6.4lf dB    U %6.4lf dB    V %6.4lf dB]", dPSNR[COMP_Y], dPSNR[COMP_Cb], dPSNR[COMP_Cr]);
    }
#if EXTENSION_360_VIDEO
    m_ext360.printPerPOCInfo(NOTICE);
#endif

    if (m_encCfg->m_printHexPsnr)
    {
      uint64_t xPsnr[MAX_NUM_COMP];
      for (int i = 0; i < MAX_NUM_COMP; i++)
      {
        if (m_encCfg->m_resChangeInClvsEnabled && m_encCfg->m_upscaledOutput == 2)
        {
          std::copy(reinterpret_cast<uint8_t *>(&upscaledPSNR[i]),
                    reinterpret_cast<uint8_t *>(&upscaledPSNR[i]) + sizeof(upscaledPSNR[i]),
                    reinterpret_cast<uint8_t *>(&xPsnr[i]));
        }
        else
        {
          std::copy(reinterpret_cast<uint8_t *>(&dPSNR[i]), reinterpret_cast<uint8_t *>(&dPSNR[i]) + sizeof(dPSNR[i]),
                    reinterpret_cast<uint8_t *>(&xPsnr[i]));
        }
      }
      msg(NOTICE, " [xY %16" PRIx64 " xU %16" PRIx64 " xV %16" PRIx64 "]", xPsnr[COMP_Y], xPsnr[COMP_Cb],
          xPsnr[COMP_Cr]);

#if EXTENSION_360_VIDEO
      m_ext360.printPerPOCInfo(NOTICE, true);
#endif
    }
    if (printMSSSIM)
    {
      if (m_encCfg->m_resChangeInClvsEnabled && m_encCfg->m_upscaledOutput == 2)
      {
        msg(NOTICE, " [MS-SSIM Y %1.6lf    U %1.6lf    V %1.6lf]", upscaledMsssim[COMP_Y], upscaledMsssim[COMP_Cb],
            upscaledMsssim[COMP_Cr]);
      }
      else
      {
        msg(NOTICE, " [MS-SSIM Y %1.6lf    U %1.6lf    V %1.6lf]", msssim[COMP_Y], msssim[COMP_Cb], msssim[COMP_Cr]);
      }
    }

    if (printFrameMSE)
    {
      msg(NOTICE, " [Y MSE %6.4lf  U MSE %6.4lf  V MSE %6.4lf]", mseYuvFrame[COMP_Y], mseYuvFrame[COMP_Cb],
          mseYuvFrame[COMP_Cr]);
    }
#if WCG_WPSNR
    if (useLumaWPSNR)
    {
      if (m_encCfg->m_resChangeInClvsEnabled && m_encCfg->m_upscaledOutput == 2)
      {
        msg(NOTICE, " [WY %6.4lf dB    WU %6.4lf dB    WV %6.4lf dB]", upscaledPSNRWeighted[COMP_Y],
            upscaledPSNRWeighted[COMP_Cb], upscaledPSNRWeighted[COMP_Cr]);
      }
      else
      {
        msg(NOTICE, " [WY %6.4lf dB    WU %6.4lf dB    WV %6.4lf dB]", dPSNRWeighted[COMP_Y], dPSNRWeighted[COMP_Cb],
            dPSNRWeighted[COMP_Cr]);
      }
      if (m_encCfg->m_printHexPsnr)
      {
        uint64_t xPsnrWeighted[MAX_NUM_COMP];
        for (int i = 0; i < MAX_NUM_COMP; i++)
        {
          if (m_encCfg->m_resChangeInClvsEnabled && m_encCfg->m_upscaledOutput == 2)
          {
            std::copy(reinterpret_cast<uint8_t *>(&upscaledPSNRWeighted[i]),
                      reinterpret_cast<uint8_t *>(&upscaledPSNRWeighted[i]) + sizeof(upscaledPSNRWeighted[i]),
                      reinterpret_cast<uint8_t *>(&xPsnrWeighted[i]));
          }
          else
          {
            std::copy(reinterpret_cast<uint8_t *>(&dPSNRWeighted[i]),
                      reinterpret_cast<uint8_t *>(&dPSNRWeighted[i]) + sizeof(dPSNRWeighted[i]),
                      reinterpret_cast<uint8_t *>(&xPsnrWeighted[i]));
          }
        }
        msg(NOTICE, " [xWY %16" PRIx64 " xWU %16" PRIx64 " xWV %16" PRIx64 "]", xPsnrWeighted[COMP_Y],
            xPsnrWeighted[COMP_Cb], xPsnrWeighted[COMP_Cr]);
      }
    }
#endif
#if JVET_O0756_CALCULATE_HDRMETRICS
    if (calculateHdrMetrics)
    {
      for (int i = 0; i < 1; i++)
      {
        msg(NOTICE, " [DeltaE%d %6.4lf dB]", (int)m_encCfg->m_whitePointDeltaE[i], deltaE[i]);
        if (m_pcEncLib->getPrintHexPsnr())
        {
          int64_t xdeltaE[MAX_NUM_COMP];
          for (int i = 0; i < 1; i++)
          {
            std::copy_n(reinterpret_cast<uint8_t *>(&deltaE[i]), sizeof(deltaE[i]),
                        reinterpret_cast<uint8_t *>(&xdeltaE[i]));
          }
          msg(NOTICE, " [xDeltaE%d %16" PRIx64 "]", (int)m_encCfg->m_whitePointDeltaE[i], xdeltaE[0]);
        }
      }
      for (int i = 0; i < 1; i++)
      {
        msg(NOTICE, " [PSNRL%d %6.4lf dB]", (int)m_encCfg->m_whitePointDeltaE[i], psnrL[i]);

        if (m_pcEncLib->getPrintHexPsnr())
        {
          int64_t xpsnrL[MAX_NUM_COMP];
          for (int i = 0; i < 1; i++)
          {
            std::copy_n(reinterpret_cast<uint8_t *>(&psnrL[i]), sizeof(psnrL[i]),
                        reinterpret_cast<uint8_t *>(&xpsnrL[i]));
          }

          msg(NOTICE, " [xPSNRL%d %16" PRIx64 "]", (int)m_encCfg->m_whitePointDeltaE[i], xpsnrL[0]);
        }
      }
    }
#endif
    msg(NOTICE, " [ET %5.0f ]", dEncTime);

    // msg( SOME, " [WP %d]", pcSlice->getUseWeightedPrediction());

    for (int refList = 0; refList < 2; refList++)
    {
      msg(NOTICE, " [L%d", refList);
      for (int refIndex = 0; refIndex < pcSlice->m_numRefIdx[RefPicList(refList)]; refIndex++)
      {
        const ScalingRatio &scaleRatio = pcSlice->getScalingRatio(RefPicList(refList), refIndex);

        if (pic->m_cs->picHeader->m_enableTMVPFlag && pcSlice->m_colFromL0Flag == bool(1 - refList) &&
            pcSlice->m_colRefIdx == refIndex)
        {
          if (scaleRatio != SCALE_1X)
          {
            msg(NOTICE, " %dc(%1.2lfx, %1.2lfx)", pcSlice->getRefPOC(RefPicList(refList), refIndex),
                double(scaleRatio.x) / (1 << ScalingRatio::BITS), double(scaleRatio.y) / (1 << ScalingRatio::BITS));
          }
          else
          {
            msg(NOTICE, " %dc", pcSlice->getRefPOC(RefPicList(refList), refIndex));
          }
        }
        else
        {
          if (scaleRatio != SCALE_1X)
          {
            msg(NOTICE, " %d(%1.2lfx, %1.2lfx)", pcSlice->getRefPOC(RefPicList(refList), refIndex),
                double(scaleRatio.x) / (1 << ScalingRatio::BITS), double(scaleRatio.y) / (1 << ScalingRatio::BITS));
          }
          else
          {
            msg(NOTICE, " %d", pcSlice->getRefPOC(RefPicList(refList), refIndex));
          }
        }

        if (pcSlice->getRefPOC(RefPicList(refList), refIndex) == pcSlice->m_poc)
        {
          msg(NOTICE, ".%d", pcSlice->getRefPic(RefPicList(refList), refIndex)->m_layerId);
        }
      }
      msg(NOTICE, "]");
    }
    if (m_encCfg->m_resChangeInClvsEnabled || m_pcEncLib->isRefLayerRescaledAvailable())
    {
      if (m_encCfg->m_upscaledOutput == 2)
      {
        msg(NOTICE, " [dpb wxh %dx%d] [Y %6.4lf dB  U %6.4lf dB  V %6.4lf dB]", decodedLumaWidth, decodedLumaHeight,
            dPSNR[COMP_Y], dPSNR[COMP_Cb], dPSNR[COMP_Cr]);
      }
    }
  }
  else if (g_verbosity >= INFO)
  {
    std::cout << "\r\t" << pcSlice->m_poc;
    std::cout.flush();
  }
}

double EncGOP::xCalculateMSSSIM(const Pel *org, const ptrdiff_t orgStride, const Pel *rec, const ptrdiff_t recStride,
                                const int width, const int height, const uint32_t bitDepth)
{
  const int MAX_MSSSIM_SCALE  = 5;
  const int WEIGHTING_MID_TAP = 5;
  const int WEIGHTING_SIZE    = WEIGHTING_MID_TAP * 2 + 1;

  uint32_t maxScale;

  // For low resolution videos determine number of scales
  if (width < 22 || height < 22)
  {
    maxScale = 1;
  }
  else if (width < 44 || height < 44)
  {
    maxScale = 2;
  }
  else if (width < 88 || height < 88)
  {
    maxScale = 3;
  }
  else if (width < 176 || height < 176)
  {
    maxScale = 4;
  }
  else
  {
    maxScale = 5;
  }

  assert(maxScale > 0 && maxScale <= MAX_MSSSIM_SCALE);

  // Normalized Gaussian mask design, 11*11, s.d. 1.5
  double weights[WEIGHTING_SIZE][WEIGHTING_SIZE];
  double coeffSum = 0.0;
  for (int y = 0; y < WEIGHTING_SIZE; y++)
  {
    for (int x = 0; x < WEIGHTING_SIZE; x++)
    {
      weights[y][x] =
        exp(-((y - WEIGHTING_MID_TAP) * (y - WEIGHTING_MID_TAP) + (x - WEIGHTING_MID_TAP) * (x - WEIGHTING_MID_TAP)) /
            (WEIGHTING_MID_TAP - 0.5));
      coeffSum += weights[y][x];
    }
  }

  for (int y = 0; y < WEIGHTING_SIZE; y++)
  {
    for (int x = 0; x < WEIGHTING_SIZE; x++)
    {
      weights[y][x] /= coeffSum;
    }
  }

  // Resolution based weights
  const double exponentWeights[MAX_MSSSIM_SCALE][MAX_MSSSIM_SCALE] = { { 1.0, 0, 0, 0, 0 },
                                                                       { 0.1356, 0.8644, 0, 0, 0 },
                                                                       { 0.0711, 0.4530, 0.4760, 0, 0 },
                                                                       { 0.0517, 0.3295, 0.3462, 0.2726, 0 },
                                                                       { 0.0448, 0.2856, 0.3001, 0.2363, 0.1333 } };

  // Downsampling of data:
  std::vector<double> original[MAX_MSSSIM_SCALE];
  std::vector<double> recon[MAX_MSSSIM_SCALE];

  for (uint32_t scale = 0; scale < maxScale; scale++)
  {
    const int scaledHeight = height >> scale;
    const int scaledWidth  = width >> scale;
    original[scale].resize(scaledHeight * scaledWidth, double(0));
    recon[scale].resize(scaledHeight * scaledWidth, double(0));
  }

  // Initial [0] arrays to be a copy of the source data (but stored in array "double", not Pel array).
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      original[0][y * width + x] = org[y * orgStride + x];
      recon[0][y * width + x]    = rec[y * recStride + x];
    }
  }

  // Set up other arrays to be average value of each 2x2 sample.
  for (uint32_t scale = 1; scale < maxScale; scale++)
  {
    const int scaledHeight = height >> scale;
    const int scaledWidth  = width >> scale;
    for (int y = 0; y < scaledHeight; y++)
    {
      for (int x = 0; x < scaledWidth; x++)
      {
        original[scale][y * scaledWidth + x] = (original[scale - 1][2 * y * (2 * scaledWidth) + 2 * x] +
                                                original[scale - 1][2 * y * (2 * scaledWidth) + 2 * x + 1] +
                                                original[scale - 1][(2 * y + 1) * (2 * scaledWidth) + 2 * x] +
                                                original[scale - 1][(2 * y + 1) * (2 * scaledWidth) + 2 * x + 1]) /
          4.0;
        recon[scale][y * scaledWidth + x] = (recon[scale - 1][2 * y * (2 * scaledWidth) + 2 * x] +
                                             recon[scale - 1][2 * y * (2 * scaledWidth) + 2 * x + 1] +
                                             recon[scale - 1][(2 * y + 1) * (2 * scaledWidth) + 2 * x] +
                                             recon[scale - 1][(2 * y + 1) * (2 * scaledWidth) + 2 * x + 1]) /
          4.0;
      }
    }
  }

  // Calculate MS-SSIM:
  const uint32_t maxValue = (1 << bitDepth) - 1;
  const double   c1       = (0.01 * maxValue) * (0.01 * maxValue);
  const double   c2       = (0.03 * maxValue) * (0.03 * maxValue);

  double finalMSSSIM = 1.0;

  for (uint32_t scale = 0; scale < maxScale; scale++)
  {
    const int scaledHeight    = height >> scale;
    const int scaledWidth     = width >> scale;
    const int blocksPerRow    = scaledWidth - WEIGHTING_SIZE + 1;
    const int blocksPerColumn = scaledHeight - WEIGHTING_SIZE + 1;
    const int totalBlocks     = blocksPerRow * blocksPerColumn;

    double meanSSIM = 0.0;

    for (int blockIndexY = 0; blockIndexY < blocksPerColumn; blockIndexY++)
    {
      for (int blockIndexX = 0; blockIndexX < blocksPerRow; blockIndexX++)
      {
        double muOrg         = 0.0;
        double muRec         = 0.0;
        double muOrigSqr     = 0.0;
        double muRecSqr      = 0.0;
        double muOrigMultRec = 0.0;

        for (int y = 0; y < WEIGHTING_SIZE; y++)
        {
          for (int x = 0; x < WEIGHTING_SIZE; x++)
          {
            const double gaussianWeight = weights[y][x];
            const int    sampleOffset   = (blockIndexY + y) * scaledWidth + (blockIndexX + x);
            const double orgPel         = original[scale][sampleOffset];
            const double recPel         = recon[scale][sampleOffset];

            muOrg += orgPel * gaussianWeight;
            muRec += recPel * gaussianWeight;
            muOrigSqr += orgPel * orgPel * gaussianWeight;
            muRecSqr += recPel * recPel * gaussianWeight;
            muOrigMultRec += orgPel * recPel * gaussianWeight;
          }
        }

        const double sigmaSqrOrig = muOrigSqr - (muOrg * muOrg);
        const double sigmaSqrRec  = muRecSqr - (muRec * muRec);
        const double sigmaOrigRec = muOrigMultRec - (muOrg * muRec);

        double blockSSIMVal = ((2.0 * sigmaOrigRec + c2) / (sigmaSqrOrig + sigmaSqrRec + c2));
        if (scale == maxScale - 1)
        {
          blockSSIMVal *= (2.0 * muOrg * muRec + c1) / (muOrg * muOrg + muRec * muRec + c1);
        }

        meanSSIM += blockSSIMVal;
      }
    }

    meanSSIM /= totalBlocks;

    finalMSSSIM *= pow(meanSSIM, exponentWeights[maxScale - 1][scale]);
  }

  return finalMSSSIM;
}

#if JVET_O0756_CALCULATE_HDRMETRICS
void EncGOP::xCalculateHDRMetrics(Picture *pic, double deltaE[hdrtoolslib::NB_REF_WHITE],
                                  double psnrL[hdrtoolslib::NB_REF_WHITE])
{
  copyBuftoFrame(pic);

  ChromaFormat chFmt = pic->chromaFormat;

  if (chFmt != ChromaFormat::_444)
  {
    m_pcConvertFormat->process(m_ppcFrameOrg[1], m_ppcFrameOrg[0]);
    m_pcConvertFormat->process(m_ppcFrameRec[1], m_ppcFrameRec[0]);
  }

  m_pcConvertIQuantize->process(m_ppcFrameOrg[2], m_ppcFrameOrg[1]);
  m_pcConvertIQuantize->process(m_ppcFrameRec[2], m_ppcFrameRec[1]);

  m_pcColorTransform->process(m_ppcFrameOrg[3], m_ppcFrameOrg[2]);
  m_pcColorTransform->process(m_ppcFrameRec[3], m_ppcFrameRec[2]);

  m_pcTransferFct->forward(m_ppcFrameOrg[4], m_ppcFrameOrg[3]);
  m_pcTransferFct->forward(m_ppcFrameRec[4], m_ppcFrameRec[3]);

  // Calculate the Metrics
  m_pcDistortionDeltaE->computeMetric(m_ppcFrameOrg[4], m_ppcFrameRec[4]);

  *deltaE = m_pcDistortionDeltaE->getDeltaE();
  *psnrL  = m_pcDistortionDeltaE->getPsnrL();
}

void EncGOP::copyBuftoFrame(Picture *pic)
{
  int cropOffsetLeft   = m_encCfg->m_cropOffsetLeft;
  int cropOffsetTop    = m_encCfg->m_cropOffsetTop;
  int cropOffsetRight  = m_encCfg->m_cropOffsetRight;
  int cropOffsetBottom = m_encCfg->m_cropOffsetBottom;

  int height = pic->getTrueOrigBuf(COMP_Y).height - cropOffsetLeft + cropOffsetRight;
  int width  = pic->getTrueOrigBuf(COMP_Y).width - cropOffsetTop + cropOffsetBottom;

  ChromaFormat chFmt = pic->chromaFormat;

  Pel *pOrg = pic->getTrueOrigBuf(COMP_Y).buf;
  Pel *pRec = pic->getRecoBuf(COMP_Y).buf;

  uint16_t *yOrg = m_ppcFrameOrg[0]->m_ui16Comp[hdrtoolslib::Y_COMP];
  uint16_t *yRec = m_ppcFrameRec[0]->m_ui16Comp[hdrtoolslib::Y_COMP];
  uint16_t *uOrg = m_ppcFrameOrg[0]->m_ui16Comp[hdrtoolslib::Cb_COMP];
  uint16_t *uRec = m_ppcFrameRec[0]->m_ui16Comp[hdrtoolslib::Cb_COMP];
  uint16_t *vOrg = m_ppcFrameOrg[0]->m_ui16Comp[hdrtoolslib::Cr_COMP];
  uint16_t *vRec = m_ppcFrameRec[0]->m_ui16Comp[hdrtoolslib::Cr_COMP];

  if (chFmt == ChromaFormat::_444)
  {
    yOrg = m_ppcFrameOrg[1]->m_ui16Comp[hdrtoolslib::Y_COMP];
    yRec = m_ppcFrameRec[1]->m_ui16Comp[hdrtoolslib::Y_COMP];
    uOrg = m_ppcFrameOrg[1]->m_ui16Comp[hdrtoolslib::Cb_COMP];
    uRec = m_ppcFrameRec[1]->m_ui16Comp[hdrtoolslib::Cb_COMP];
    vOrg = m_ppcFrameOrg[1]->m_ui16Comp[hdrtoolslib::Cr_COMP];
    vRec = m_ppcFrameRec[1]->m_ui16Comp[hdrtoolslib::Cr_COMP];
  }

  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      yOrg[i * width + j] =
        static_cast<uint16_t>(pOrg[(i + cropOffsetTop) * pic->getTrueOrigBuf(COMP_Y).stride + j + cropOffsetLeft]);
      yRec[i * width + j] =
        static_cast<uint16_t>(pRec[(i + cropOffsetTop) * pic->getRecoBuf(COMP_Y).stride + j + cropOffsetLeft]);
    }
  }

  if (chFmt != ChromaFormat::_444)
  {
    height >>= 1;
    width >>= 1;
    cropOffsetLeft >>= 1;
    cropOffsetTop >>= 1;
  }

  pOrg = pic->getTrueOrigBuf(COMP_Cb).buf;
  pRec = pic->getRecoBuf(COMP_Cb).buf;

  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      uOrg[i * width + j] =
        static_cast<uint16_t>(pOrg[(i + cropOffsetTop) * pic->getTrueOrigBuf(COMP_Cb).stride + j + cropOffsetLeft]);
      uRec[i * width + j] =
        static_cast<uint16_t>(pRec[(i + cropOffsetTop) * pic->getRecoBuf(COMP_Cb).stride + j + cropOffsetLeft]);
    }
  }

  pOrg = pic->getTrueOrigBuf(COMP_Cr).buf;
  pRec = pic->getRecoBuf(COMP_Cr).buf;

  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      vOrg[i * width + j] =
        static_cast<uint16_t>(pOrg[(i + cropOffsetTop) * pic->getTrueOrigBuf(COMP_Cr).stride + j + cropOffsetLeft]);
      vRec[i * width + j] =
        static_cast<uint16_t>(pRec[(i + cropOffsetTop) * pic->getRecoBuf(COMP_Cr).stride + j + cropOffsetLeft]);
    }
  }
}
#endif

void EncGOP::xCalculateInterlacedAddPSNR(Picture *picOrgFirstField, Picture *picOrgSecondField,
                                         PelUnitBuf cPicRecFirstField, PelUnitBuf cPicRecSecondField,
                                         const InputColourSpaceConversion conversion, const bool printFrameMSE,
                                         const bool printMSSSIM, double *PSNR_Y, bool isEncodeLtRef)
{
  const SPS         &sps    = *picOrgFirstField->m_cs->sps;
  const ChromaFormat format = sps.m_chromaFormatIdc;
  double             dPSNR[MAX_NUM_COMP];
  Picture           *apicOrgFields[2]  = { picOrgFirstField, picOrgSecondField };
  PelUnitBuf         acPicRecFields[2] = { cPicRecFirstField, cPicRecSecondField };
#if ENABLE_QPA
  const bool useWPSNR = m_encCfg->m_bUseWPSNR;
#endif
  for (int i = 0; i < MAX_NUM_COMP; i++)
  {
    dPSNR[i] = 0.0;
  }

  PelStorage cscd[2 /* first/second field */];
  if (conversion != IPCOLOURSPACE_UNCHANGED)
  {
    for (uint32_t fieldNum = 0; fieldNum < 2; fieldNum++)
    {
      PelUnitBuf &reconField = (acPicRecFields[fieldNum]);
      cscd[fieldNum].create(reconField.chromaFormat, Area(Position(), reconField.Y()));
      VideoIOYuv::ColourSpaceConvert(reconField, cscd[fieldNum], conversion, false);
      acPicRecFields[fieldNum] = cscd[fieldNum];
    }
  }

  //===== calculate PSNR =====
  double mseYuvFrame[MAX_NUM_COMP] = { 0, 0, 0 };
  double msssim[MAX_NUM_COMP]      = { 0.0 };

  CHECK(!(acPicRecFields[0].chromaFormat == acPicRecFields[1].chromaFormat), "Unspecified error");
  const uint32_t numValidComponents = ::getNumberValidComponents(acPicRecFields[0].chromaFormat);

  for (int chan = 0; chan < numValidComponents; chan++)
  {
    const CompID ch = CompID(chan);
    CHECK(!(acPicRecFields[0].get(ch).width == acPicRecFields[1].get(ch).width), "Unspecified error");
    CHECK(!(acPicRecFields[0].get(ch).height == acPicRecFields[0].get(ch).height), "Unspecified error");

    uint64_t       ssdTemp = 0;
    const uint32_t width =
      acPicRecFields[0].get(ch).width - (m_encCfg->m_sourcePadding[0] >> ::getComponentScaleX(ch, format));
    const uint32_t height =
      acPicRecFields[0].get(ch).height - ((m_encCfg->m_sourcePadding[1] >> 1) >> ::getComponentScaleY(ch, format));
    const uint32_t bitDepth = sps.m_bitDepths[toChannelType(ch)];

    double sumOverFieldsMSSSIM = 0;
    for (uint32_t fieldNum = 0; fieldNum < 2; fieldNum++)
    {
      CHECK(!(conversion == IPCOLOURSPACE_UNCHANGED), "Unspecified error");
#if ENABLE_QPA
      ssdTemp += xFindDistortionPlane(acPicRecFields[fieldNum].get(ch), apicOrgFields[fieldNum]->getOrigBuf().get(ch),
                                      useWPSNR ? bitDepth : 0, ::getComponentScaleX(ch, format),
                                      ::getComponentScaleY(ch, format));
#else
      ssdTemp +=
        xFindDistortionPlane(acPicRecFields[fieldNum].get(ch), apicOrgFields[fieldNum]->getOrigBuf().get(ch), 0);
#endif
      if (printMSSSIM)
      {
        CPelBuf o = apicOrgFields[fieldNum]->getOrigBuf().get(ch);
        CPelBuf p = acPicRecFields[fieldNum].get(ch);
        sumOverFieldsMSSSIM +=
          xCalculateMSSSIM(o.bufAt(0, 0), o.stride, p.bufAt(0, 0), p.stride, width, height, bitDepth);
      }
    }
    if (printMSSSIM)
    {
      msssim[ch] = sumOverFieldsMSSSIM / 2;
    }
    const uint32_t maxval    = 255 << (bitDepth - 8);
    const uint32_t size      = width * height * 2;
    const double   fRefValue = (double)maxval * maxval * size;
    dPSNR[ch]                = ssdTemp ? 10.0 * log10(fRefValue / (double)ssdTemp) : 999.99;
    mseYuvFrame[ch]          = (double)ssdTemp / size;
  }

  uint32_t uibits =
    0;   // the number of bits for the pair is not calculated here - instead the overall total is used elsewhere.

  //===== add PSNR =====
  m_gcAnalyzeAllField.setUpscaledOutput(m_encCfg->m_upscaledOutput);
  m_gcAnalyzeAllField.addResult(dPSNR, (double)uibits, mseYuvFrame, mseYuvFrame, msssim, msssim, isEncodeLtRef);

  *PSNR_Y = dPSNR[COMP_Y];

  msg(INFO, "\n                                      Interlaced frame %d: [Y %6.4lf dB    U %6.4lf dB    V %6.4lf dB]",
      picOrgSecondField->m_poc / 2, dPSNR[COMP_Y], dPSNR[COMP_Cb], dPSNR[COMP_Cr]);
  if (printMSSSIM)
  {
    printf(" [MS-SSIM Y %1.6lf    U %1.6lf    V %1.6lf]", msssim[COMP_Y], msssim[COMP_Cb], msssim[COMP_Cr]);
  }
  if (printFrameMSE)
  {
    msg(DETAILS, " [Y MSE %6.4lf  U MSE %6.4lf  V MSE %6.4lf]", mseYuvFrame[COMP_Y], mseYuvFrame[COMP_Cb],
        mseYuvFrame[COMP_Cr]);
  }

  for (uint32_t fieldNum = 0; fieldNum < 2; fieldNum++)
  {
    cscd[fieldNum].destroy();
  }
}

/** Function for deciding the nal_unit_type.
 * \param pocCurr POC of the current picture
 * \param lastIDR  POC of the last IDR picture
 * \param isField  true to indicate field coding
 * \returns the NAL unit type of the picture
 * This function checks the configuration and returns the appropriate nal_unit_type for the picture.
 */
NalUnitType EncGOP::getNalUnitType(int pocCurr, int lastIDR, bool isField)
{

  if (pocCurr == 0)
  {
    return NAL_UNIT_CODED_SLICE_IDR_N_LP;
  }

  if (m_encCfg->m_efficientFieldIRAPEnabled && isField && pocCurr == (m_encCfg->m_compositeRefEnabled ? 2 : 1))
  {
    // to avoid the picture becoming an IRAP
    return NAL_UNIT_CODED_SLICE_TRAIL;
  }

  if (m_encCfg->m_decodingRefreshType != 3 &&
      (pocCurr - isField) % (m_encCfg->m_intraPeriod * (m_encCfg->m_compositeRefEnabled ? 2 : 1)) == 0)
  {
    if (m_encCfg->m_decodingRefreshType == 1)
    {
      return NAL_UNIT_CODED_SLICE_CRA;
    }
    else if (m_encCfg->m_decodingRefreshType == 2)
    {
      SPS *sps = m_pcEncLib->getSPS(m_pcEncLib->m_layerId);
      if (sps != nullptr)
      {
        const int maxTLayer = sps->m_maxSubLayers - 1;
        return sps->m_maxNumReorderPics[maxTLayer] > 0 ? NAL_UNIT_CODED_SLICE_IDR_W_RADL
                                                       : NAL_UNIT_CODED_SLICE_IDR_N_LP;
      }
      else
      {
        return NAL_UNIT_CODED_SLICE_IDR_W_RADL;
      }
    }
  }
  if (m_pocCRA > 0)
  {
    if (pocCurr < m_pocCRA)
    {
      // All leading pictures are being marked as TFD pictures here since current encoder uses all
      // reference pictures while encoding leading pictures. An encoder can ensure that a leading
      // picture can be still decodable when random accessing to a CRA/CRANT/BLA/BLANT picture by
      // controlling the reference pictures used for encoding that leading picture. Such a leading
      // picture need not be marked as a TFD picture.
      return NAL_UNIT_CODED_SLICE_RASL;
    }
  }
  if (lastIDR > 0)
  {
    if (pocCurr < lastIDR)
    {
      return NAL_UNIT_CODED_SLICE_RADL;
    }
  }
  return NAL_UNIT_CODED_SLICE_TRAIL;
}

void EncGOP::xUpdateRasInit(Slice *slice)
{
  slice->m_pendingRasInit = false;
  if (slice->m_poc > m_lastRasPoc)
  {
    m_lastRasPoc            = MAX_INT;
    slice->m_pendingRasInit = true;
  }
  if (slice->isIRAP())
  {
    m_lastRasPoc = slice->m_poc;
  }
}

void EncGOP::xUpdateRPRtmvp(PicHeader *picHeader, Slice *pcSlice)
{
  if (picHeader->m_enableTMVPFlag)
  {
    int colRefIdxL0 = -1, colRefIdxL1 = -1;

    for (int refIdx = 0; refIdx < pcSlice->m_numRefIdx[RPL0]; refIdx++)
    {
      if (!(pcSlice->getRefPic(RPL0, refIdx)->m_slices[0]->m_eNalUnitType != NAL_UNIT_CODED_SLICE_RASL &&
            pcSlice->getRefPic(RPL0, refIdx)->m_poc <= m_pocCRA))
      {
        colRefIdxL0 = refIdx;
        break;
      }
    }

    for (int refIdx = 0; refIdx < pcSlice->m_numRefIdx[RPL1]; refIdx++)
    {
      if (!(pcSlice->getRefPic(RPL1, refIdx)->m_slices[0]->m_eNalUnitType != NAL_UNIT_CODED_SLICE_RASL &&
            pcSlice->getRefPic(RPL1, refIdx)->m_poc <= m_pocCRA))
      {
        colRefIdxL1 = refIdx;
        break;
      }
    }

    if (colRefIdxL0 >= 0 && colRefIdxL1 >= 0)
    {
      const Picture *refPicL0 = pcSlice->getRefPic(RPL0, colRefIdxL0);
      const Picture *refPicL1 = pcSlice->getRefPic(RPL1, colRefIdxL1);

      CHECK(!refPicL0->m_slices.size(), "Wrong L0 reference picture");
      CHECK(!refPicL1->m_slices.size(), "Wrong L1 reference picture");

      const uint32_t colFromL0      = refPicL0->m_slices[0]->m_iSliceQp > refPicL1->m_slices[0]->m_iSliceQp;
      picHeader->m_picColFromL0Flag = colFromL0;
      pcSlice->m_colFromL0Flag      = colFromL0;
      pcSlice->m_colRefIdx          = (colFromL0 ? colRefIdxL0 : colRefIdxL1);
      picHeader->m_colRefIdx        = (colFromL0 ? colRefIdxL0 : colRefIdxL1);
    }
    else if (colRefIdxL0 < 0 && colRefIdxL1 >= 0)
    {
      picHeader->m_picColFromL0Flag = false;
      pcSlice->m_colFromL0Flag      = false;
      pcSlice->m_colRefIdx          = colRefIdxL1;
      picHeader->m_colRefIdx        = colRefIdxL1;
    }
    else if (colRefIdxL0 >= 0 && colRefIdxL1 < 0)
    {
      picHeader->m_picColFromL0Flag = true;
      pcSlice->m_colFromL0Flag      = true;
      pcSlice->m_colRefIdx          = colRefIdxL0;
      picHeader->m_colRefIdx        = colRefIdxL0;
    }
    else
    {
      picHeader->m_enableTMVPFlag = false;
    }
  }
}

double EncGOP::xCalculateRVM()
{
  double dRVM = 0;

  if (m_encCfg->m_gopSize == 1 && m_encCfg->m_intraPeriod != 1 && m_encCfg->m_framesToBeEncoded > RVM_VCEGAM10_M * 2)
  {
    // calculate RVM only for lowdelay configurations

    size_t n = m_rvm.size();

    std::vector<double> vRL(n);
    std::vector<double> vB(n);

    int    i;
    double dRavg = 0, dBavg = 0;
    vB[RVM_VCEGAM10_M] = 0;
    for (i = RVM_VCEGAM10_M + 1; i < n - RVM_VCEGAM10_M + 1; i++)
    {
      vRL[i] = 0;
      for (int j = i - RVM_VCEGAM10_M; j <= i + RVM_VCEGAM10_M - 1; j++)
      {
        vRL[i] += m_rvm[j];
      }
      vRL[i] /= (2 * RVM_VCEGAM10_M);
      vB[i] = vB[i - 1] + m_rvm[i] - vRL[i];
      dRavg += m_rvm[i];
      dBavg += vB[i];
    }

    dRavg /= (n - 2 * RVM_VCEGAM10_M);
    dBavg /= (n - 2 * RVM_VCEGAM10_M);

    double dSigamB = 0;
    for (i = RVM_VCEGAM10_M + 1; i < n - RVM_VCEGAM10_M + 1; i++)
    {
      double tmp = vB[i] - dBavg;
      dSigamB += tmp * tmp;
    }
    dSigamB = sqrt(dSigamB / (n - 2 * RVM_VCEGAM10_M));

    double f = sqrt(12.0 * (RVM_VCEGAM10_M - 1) / (RVM_VCEGAM10_M + 1));

    dRVM = dSigamB / dRavg * f;
  }

  return (dRVM);
}

/** Attaches the input bitstream to the stream in the output NAL unit
    Updates rNalu to contain concatenated bitstream. rpcBitstreamRedirect is cleared at the end of this function call.
 *  \param codedSliceData contains the coded slice data (bitstream) to be concatenated to rNalu
 *  \param rNalu          target NAL unit
 */
void EncGOP::xAttachSliceDataToNalUnit(OutputNALUnit &rNalu, OutputBitstream *codedSliceData)
{
  // Byte-align
  rNalu.m_bitstream.writeByteAlignment();   // Slice header byte-alignment

  // Perform bitstream concatenation
  if (codedSliceData->getNumberOfWrittenBits() > 0)
  {
    rNalu.m_bitstream.addSubstream(codedSliceData);
  }
  codedSliceData->clear();
}

void EncGOP::getRealRange(Picture *pcPic)
{
  int  width     = pcPic->m_cs->pps->m_picWidthInLumaSamples;
  int  height    = pcPic->m_cs->pps->m_picHeightInLumaSamples;
  auto oriStride = pcPic->getOrigBuf().get(COMP_Y).stride;
  Pel *oriPel    = pcPic->getOrigBuf().get(COMP_Y).buf;
  int  pelMax    = 0;
  int  pelMin    = (1 << pcPic->m_cs->sps->m_bitDepths[ChannelType::LUMA]) - 1;
  for (uint32_t yPos = 0; yPos < height; yPos++)
  {
    for (uint32_t xPos = 0; xPos < width; xPos++)
    {
      int tmpPel = oriPel[yPos * oriStride + xPos];
      if (tmpPel > pelMax)
      {
        pelMax = tmpPel;
      }
      if (tmpPel < pelMin)
      {
        pelMin = tmpPel;
      }
    }
  }
  pcPic->m_lumaClpRng.min = pelMin;
  pcPic->m_lumaClpRng.max = pelMax;
}

void EncGOP::arrangeCompositeReference(Slice *pcSlice, PicList &rcListPic, int pocCurr)
{
  Picture             *curPic  = nullptr;
  PicList::iterator    iterPic = rcListPic.begin();
  const PreCalcValues *pcv     = pcSlice->m_pps->pcv;
  m_bgPOC                      = pocCurr + 1;
  if (m_picBg->getSpliceFull())
  {
    return;
  }
  while (iterPic != rcListPic.end())
  {
    curPic = *(iterPic++);
    if (curPic->m_poc == pocCurr)
    {
      break;
    }
  }
  if (pcSlice->isIRAP())
  {
    return;
  }

  int       width         = pcv->lumaWidth;
  int       height        = pcv->lumaHeight;
  ptrdiff_t stride        = curPic->getOrigBuf().get(COMP_Y).stride;
  ptrdiff_t cStride       = curPic->getOrigBuf().get(COMP_Cb).stride;
  Pel      *curLumaAddr   = curPic->getOrigBuf().get(COMP_Y).buf;
  Pel      *curCbAddr     = curPic->getOrigBuf().get(COMP_Cb).buf;
  Pel      *curCrAddr     = curPic->getOrigBuf().get(COMP_Cr).buf;
  Pel      *bgOrgLumaAddr = m_picOrig->getOrigBuf().get(COMP_Y).buf;
  Pel      *bgOrgCbAddr   = m_picOrig->getOrigBuf().get(COMP_Cb).buf;
  Pel      *bgOrgCrAddr   = m_picOrig->getOrigBuf().get(COMP_Cr).buf;
  int       cuMaxWidth    = pcv->maxCUWidth;
  int       cuMaxHeight   = pcv->maxCUHeight;
  int       maxReplace    = (pcv->sizeInCtus) / 2;
  maxReplace              = maxReplace < 1 ? 1 : maxReplace;
  struct CostStr
  {
    double cost;
    int    ctuIdx;
  };
  CostStr *minCtuCost = new CostStr[maxReplace];
  for (int i = 0; i < maxReplace; i++)
  {
    minCtuCost[i].cost   = 1e10;
    minCtuCost[i].ctuIdx = -1;
  }
  int bitIncrementY  = pcSlice->m_sps->m_bitDepths[ChannelType::LUMA] - 8;
  int bitIncrementUV = pcSlice->m_sps->m_bitDepths[ChannelType::CHROMA] - 8;
  for (int y = 0; y < height; y += cuMaxHeight)
  {
    for (int x = 0; x < width; x += cuMaxWidth)
    {
      double lcuDist      = 0.0;
      double lcuDistCb    = 0.0;
      double lcuDistCr    = 0.0;
      int    realPixelCnt = 0;
      double lcuCost      = 1e10;
      int    largeDist    = 0;

      for (int tmpy = 0; tmpy < cuMaxHeight; tmpy++)
      {
        if (y + tmpy >= height)
        {
          break;
        }
        for (int tmpx = 0; tmpx < cuMaxWidth; tmpx++)
        {
          if (x + tmpx >= width)
          {
            break;
          }

          realPixelCnt++;
          lcuDist += abs(curLumaAddr[(y + tmpy) * stride + x + tmpx] - bgOrgLumaAddr[(y + tmpy) * stride + x + tmpx]);
          if (abs(curLumaAddr[(y + tmpy) * stride + x + tmpx] - bgOrgLumaAddr[(y + tmpy) * stride + x + tmpx]) >
              (20 << bitIncrementY))
          {
            largeDist++;
          }

          if (tmpy % 2 == 0 && tmpx % 2 == 0)
          {
            lcuDistCb += abs(curCbAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2] -
                             bgOrgCbAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2]);
            lcuDistCr += abs(curCrAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2] -
                             bgOrgCrAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2]);
          }
        }
      }

      // Test the vertical or horizontal edge for background patches candidates
      int yInLCU  = y / cuMaxHeight;
      int xInLCU  = x / cuMaxWidth;
      int iLCUIdx = yInLCU * pcv->widthInCtus + xInLCU;
      if ((largeDist / (double)realPixelCnt < 0.01 && lcuDist / realPixelCnt < (3.5 * (1 << bitIncrementY)) &&
           lcuDistCb / realPixelCnt < (0.5 * (1 << bitIncrementUV)) &&
           lcuDistCr / realPixelCnt < (0.5 * (1 << bitIncrementUV)) && m_picBg->m_spliceIdx[iLCUIdx] == 0))
      {
        lcuCost = lcuDist / realPixelCnt + lcuDistCb / realPixelCnt + lcuDistCr / realPixelCnt;
        // obtain the maxReplace smallest cost
        // 1) find the largest cost in the maxReplace candidates
        for (int i = 0; i < maxReplace - 1; i++)
        {
          if (minCtuCost[i].cost > minCtuCost[i + 1].cost)
          {
            std::swap(minCtuCost[i].cost, minCtuCost[i + 1].cost);
            std::swap(minCtuCost[i].ctuIdx, minCtuCost[i + 1].ctuIdx);
          }
        }
        // 2) compare the current cost with the largest cost
        if (lcuCost < minCtuCost[maxReplace - 1].cost)
        {
          minCtuCost[maxReplace - 1].cost   = lcuCost;
          minCtuCost[maxReplace - 1].ctuIdx = iLCUIdx;
        }
      }
    }
  }

  // modify QP for background CTU
  for (int i = 0; i < maxReplace; i++)
  {
    if (minCtuCost[i].ctuIdx != -1)
    {
      m_picBg->m_spliceIdx[minCtuCost[i].ctuIdx] = pocCurr;
    }
  }

  delete[] minCtuCost;
}

void EncGOP::updateCompositeReference(Slice *pcSlice, PicList &rcListPic, int pocCurr)
{
  Picture             *curPic  = nullptr;
  const PreCalcValues *pcv     = pcSlice->m_pps->pcv;
  PicList::iterator    iterPic = rcListPic.begin();
  iterPic                      = rcListPic.begin();
  while (iterPic != rcListPic.end())
  {
    curPic = *(iterPic++);
    if (curPic->m_poc == pocCurr)
    {
      break;
    }
  }
  assert(curPic->m_poc == pocCurr);

  int       width   = pcv->lumaWidth;
  int       height  = pcv->lumaHeight;
  ptrdiff_t stride  = curPic->getRecoBuf().get(COMP_Y).stride;
  ptrdiff_t cStride = curPic->getRecoBuf().get(COMP_Cb).stride;

  Pel *bgLumaAddr  = m_picBg->getRecoBuf().get(COMP_Y).buf;
  Pel *bgCbAddr    = m_picBg->getRecoBuf().get(COMP_Cb).buf;
  Pel *bgCrAddr    = m_picBg->getRecoBuf().get(COMP_Cr).buf;
  Pel *curLumaAddr = curPic->getRecoBuf().get(COMP_Y).buf;
  Pel *curCbAddr   = curPic->getRecoBuf().get(COMP_Cb).buf;
  Pel *curCrAddr   = curPic->getRecoBuf().get(COMP_Cr).buf;

  int maxCuWidth  = pcv->maxCUWidth;
  int maxCuHeight = pcv->maxCUHeight;

  // Update background reference
  if (pcSlice->isIRAP())   //(pocCurr == 0)
  {
    curPic->extendPicBorder(pcSlice->m_pps);
    curPic->m_extendedBorder = true;

    m_picBg->getRecoBuf().copyFrom(curPic->getRecoBuf());
    m_picOrig->getOrigBuf().copyFrom(curPic->getOrigBuf());
  }
  else
  {
    // cout << "update B" << pocCurr << endl;
    for (int y = 0; y < height; y += maxCuHeight)
    {
      for (int x = 0; x < width; x += maxCuWidth)
      {
        if (m_picBg->m_spliceIdx[(y / maxCuHeight) * pcv->widthInCtus + x / maxCuWidth] == pocCurr)
        {
          for (int tmpy = 0; tmpy < maxCuHeight; tmpy++)
          {
            if (y + tmpy >= height)
            {
              break;
            }
            for (int tmpx = 0; tmpx < maxCuWidth; tmpx++)
            {
              if (x + tmpx >= width)
              {
                break;
              }
              bgLumaAddr[(y + tmpy) * stride + x + tmpx] = curLumaAddr[(y + tmpy) * stride + x + tmpx];
              if (tmpy % 2 == 0 && tmpx % 2 == 0)
              {
                bgCbAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2] =
                  curCbAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2];
                bgCrAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2] =
                  curCrAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2];
              }
            }
          }
        }
      }
    }
    m_picBg->m_extendedBorder = false;
    m_picBg->extendPicBorder(pcSlice->m_pps);
    m_picBg->m_extendedBorder = true;

    curPic->extendPicBorder(pcSlice->m_pps);
    curPic->m_extendedBorder = true;
    m_picOrig->getOrigBuf().copyFrom(curPic->getOrigBuf());

    m_picBg->m_extendedBorder = false;
    m_picBg->extendPicBorder(pcSlice->m_pps);
    m_picBg->m_extendedBorder = true;
  }
}

void EncGOP::applyDeblockingFilterMetric(Picture *pic)
{
  CPelBuf pelBuf = pic->getRecoBuf().get(COMP_Y);

  const Pel      *rec       = pelBuf.buf;
  const ptrdiff_t stride    = pelBuf.stride;
  const uint32_t  picWidth  = pelBuf.width;
  const uint32_t  picHeight = pelBuf.height;

  const Pel   *tempRec    = rec;
  const Slice *firstSlice = pic->m_slices.front();

  const uint32_t log2maxTB       = firstSlice->m_sps->m_log2MaxTbSize;
  const uint32_t maxTBsize       = (1 << log2maxTB);
  const uint32_t minBlockArtSize = 8;
  const uint32_t noCol           = (picWidth >> log2maxTB);
  const uint32_t noRows          = (picHeight >> log2maxTB);
  CHECK(!(noCol > 1), "Unspecified error");
  CHECK(!(noRows > 1), "Unspecified error");
  std::vector<uint64_t> colSAD(noCol, uint64_t(0));
  std::vector<uint64_t> rowSAD(noRows, uint64_t(0));
  uint32_t              colIdx = 0;
  uint32_t              rowIdx = 0;
  Pel                   p0, p1, p2, q0, q1, q2;

  const int qp            = firstSlice->m_iSliceQp;
  const int bitDepthLuma  = firstSlice->m_sps->m_bitDepths[ChannelType::LUMA];
  const int bitdepthScale = 1 << (bitDepthLuma - 8);
  const int beta          = DeblockingFilter::getBeta(qp) * bitdepthScale;
  const int thr2          = (beta >> 2);
  const int thr1          = 2 * bitdepthScale;
  uint32_t  a             = 0;

  if (maxTBsize > minBlockArtSize)
  {
    // Analyze vertical artifact edges
    for (int c = maxTBsize; c < picWidth; c += maxTBsize)
    {
      for (int r = 0; r < picHeight; r++)
      {
        p2 = rec[c - 3];
        p1 = rec[c - 2];
        p0 = rec[c - 1];
        q0 = rec[c];
        q1 = rec[c + 1];
        q2 = rec[c + 2];
        a  = ((abs(p2 - (p1 << 1) + p0) + abs(q0 - (q1 << 1) + q2)) << 1);
        if (thr1 < a && a < thr2)
        {
          colSAD[colIdx] += abs(p0 - q0);
        }
        rec += stride;
      }
      colIdx++;
      rec = tempRec;
    }

    // Analyze horizontal artifact edges
    for (int r = maxTBsize; r < picHeight; r += maxTBsize)
    {
      for (int c = 0; c < picWidth; c++)
      {
        p2 = rec[c + (r - 3) * stride];
        p1 = rec[c + (r - 2) * stride];
        p0 = rec[c + (r - 1) * stride];
        q0 = rec[c + r * stride];
        q1 = rec[c + (r + 1) * stride];
        q2 = rec[c + (r + 2) * stride];
        a  = ((abs(p2 - (p1 << 1) + p0) + abs(q0 - (q1 << 1) + q2)) << 1);
        if (thr1 < a && a < thr2)
        {
          rowSAD[rowIdx] += abs(p0 - q0);
        }
      }
      rowIdx++;
    }
  }

  uint64_t colSADsum = 0;
  uint64_t rowSADsum = 0;
  for (int c = 0; c < noCol - 1; c++)
  {
    colSADsum += colSAD[c];
  }
  for (int r = 0; r < noRows - 1; r++)
  {
    rowSADsum += rowSAD[r];
  }

  colSADsum <<= 10;
  rowSADsum <<= 10;
  colSADsum /= (noCol - 1);
  colSADsum /= picHeight;
  rowSADsum /= (noRows - 1);
  rowSADsum /= picWidth;

  uint64_t avgSAD = ((colSADsum + rowSADsum) >> 1);
  avgSAD >>= (bitDepthLuma - 8);

  if (avgSAD > 2048)
  {
    avgSAD >>= 9;
    int offset = Clip3(2, 6, (int)avgSAD);
    for (Slice *slice: pic->m_slices)
    {
      slice->m_deblockingFilterOverrideFlag     = true;
      slice->m_deblockingFilterDisable          = false;
      slice->m_deblockingFilterBetaOffsetDiv2   = offset;
      slice->m_deblockingFilterTcOffsetDiv2     = offset;
      slice->m_deblockingFilterCbBetaOffsetDiv2 = offset;
      slice->m_deblockingFilterCbTcOffsetDiv2   = offset;
      slice->m_deblockingFilterCrBetaOffsetDiv2 = offset;
      slice->m_deblockingFilterCrTcOffsetDiv2   = offset;
    }
  }
  else
  {
    const PPS *pps = firstSlice->m_pps;

    for (Slice *slice: pic->m_slices)
    {
      slice->m_deblockingFilterOverrideFlag     = false;
      slice->m_deblockingFilterDisable          = pps->m_ppsDeblockingFilterDisabledFlag;
      slice->m_deblockingFilterBetaOffsetDiv2   = pps->m_deblockingFilterBetaOffsetDiv2;
      slice->m_deblockingFilterTcOffsetDiv2     = pps->m_deblockingFilterTcOffsetDiv2;
      slice->m_deblockingFilterCbBetaOffsetDiv2 = pps->m_deblockingFilterCbBetaOffsetDiv2;
      slice->m_deblockingFilterCbTcOffsetDiv2   = pps->m_deblockingFilterCbTcOffsetDiv2;
      slice->m_deblockingFilterCrBetaOffsetDiv2 = pps->m_deblockingFilterCrBetaOffsetDiv2;
      slice->m_deblockingFilterCrTcOffsetDiv2   = pps->m_deblockingFilterCrTcOffsetDiv2;
    }
  }
}

void EncGOP::applyDeblockingFilterParameterSelection(Picture *pic, const uint32_t numSlices, const int gopID)
{
  constexpr int MAX_BETA_OFFSET = 3;
  constexpr int MIN_BETA_OFFSET = -3;
  constexpr int MAX_TC_OFFSET   = 3;
  constexpr int MIN_TC_OFFSET   = -3;

  PelUnitBuf reco = pic->getRecoBuf();

  const int currQualityLayer = !pic->m_slices[0]->isIRAP() ? m_encCfg->m_GOPList[gopID].m_temporalId + 1 : 0;
  CHECK(currQualityLayer >= MAX_ENCODER_DEBLOCKING_QUALITY_LAYERS, "currQualityLayer is too large");

  CodingStructure &cs = *pic->m_cs;

  if (!m_pcDeblockingTempPicYuv)
  {
    m_pcDeblockingTempPicYuv = new PelStorage;
    m_pcDeblockingTempPicYuv->create(cs.area);

    for (auto &p: m_deblockParam)
    {
      p.available = false;
    }
  }

  // preserve current reconstruction
  m_pcDeblockingTempPicYuv->copyFrom(reco);

  auto &deblockParam = m_deblockParam[currQualityLayer];

  const bool hasBetaTc = deblockParam.available && !deblockParam.disabled;

  const int maxBetaOffsetDiv2 =
    hasBetaTc ? Clip3(MIN_BETA_OFFSET, MAX_BETA_OFFSET, deblockParam.betaOffsetDiv2 + 1) : MAX_BETA_OFFSET;
  const int minBetaOffsetDiv2 =
    hasBetaTc ? Clip3(MIN_BETA_OFFSET, MAX_BETA_OFFSET, deblockParam.betaOffsetDiv2 - 1) : MIN_BETA_OFFSET;

  const int maxTcOffsetDiv2 =
    hasBetaTc ? Clip3(MIN_TC_OFFSET, MAX_TC_OFFSET, deblockParam.tcOffsetDiv2 + 2) : MAX_TC_OFFSET;
  const int minTcOffsetDiv2 =
    hasBetaTc ? Clip3(MIN_TC_OFFSET, MAX_TC_OFFSET, deblockParam.tcOffsetDiv2 - 2) : MIN_TC_OFFSET;

  uint64_t distBetaPrevious = std::numeric_limits<uint64_t>::max();
  uint64_t distMin          = std::numeric_limits<uint64_t>::max();

  bool dbFilterDisabledBest = true;
  int  betaOffsetDiv2Best   = 0;
  int  tcOffsetDiv2Best     = 0;

  for (int betaOffsetDiv2 = maxBetaOffsetDiv2; betaOffsetDiv2 >= minBetaOffsetDiv2; betaOffsetDiv2--)
  {
    uint64_t distTcMin = std::numeric_limits<uint64_t>::max();

    for (int tcOffsetDiv2 = maxTcOffsetDiv2; tcOffsetDiv2 >= minTcOffsetDiv2; tcOffsetDiv2--)
    {
      for (int i = 0; i < numSlices; i++)
      {
        Slice *slice = pic->m_slices[i];

        slice->m_deblockingFilterOverrideFlag     = true;
        slice->m_deblockingFilterDisable          = false;
        slice->m_deblockingFilterBetaOffsetDiv2   = betaOffsetDiv2;
        slice->m_deblockingFilterTcOffsetDiv2     = tcOffsetDiv2;
        slice->m_deblockingFilterCbBetaOffsetDiv2 = betaOffsetDiv2;
        slice->m_deblockingFilterCbTcOffsetDiv2   = tcOffsetDiv2;
        slice->m_deblockingFilterCrBetaOffsetDiv2 = betaOffsetDiv2;
        slice->m_deblockingFilterCrTcOffsetDiv2   = tcOffsetDiv2;
      }

      // restore reconstruction
      reco.copyFrom(*m_pcDeblockingTempPicYuv);

      const uint64_t dist = preLoopFilterPicAndCalcDist(pic);

      if (dist < distMin)
      {
        distMin              = dist;
        dbFilterDisabledBest = false;
        betaOffsetDiv2Best   = betaOffsetDiv2;
        tcOffsetDiv2Best     = tcOffsetDiv2;
      }

      if (dist < distTcMin)
      {
        distTcMin = dist;
      }
      else if (tcOffsetDiv2 < -2)
      {
        break;
      }
    }

    if (betaOffsetDiv2 < -1 && distTcMin >= distBetaPrevious)
    {
      break;
    }
    distBetaPrevious = distTcMin;
  }

  // update
  deblockParam.available      = true;
  deblockParam.disabled       = dbFilterDisabledBest;
  deblockParam.betaOffsetDiv2 = betaOffsetDiv2Best;
  deblockParam.tcOffsetDiv2   = tcOffsetDiv2Best;

  // restore reconstruction
  reco.copyFrom(*m_pcDeblockingTempPicYuv);

  const PPS *pps = pic->m_slices.front()->m_pps;
  if (dbFilterDisabledBest)
  {
    for (int i = 0; i < numSlices; i++)
    {
      Slice *slice = pic->m_slices[i];

      slice->m_deblockingFilterOverrideFlag = !pps->m_ppsDeblockingFilterDisabledFlag;
      slice->m_deblockingFilterDisable      = true;
    }
  }
  else if (!pps->m_ppsDeblockingFilterDisabledFlag && betaOffsetDiv2Best == pps->m_deblockingFilterBetaOffsetDiv2 &&
           tcOffsetDiv2Best == pps->m_deblockingFilterTcOffsetDiv2)
  {
    for (int i = 0; i < numSlices; i++)
    {
      Slice *slice = pic->m_slices[i];

      slice->m_deblockingFilterOverrideFlag     = false;
      slice->m_deblockingFilterDisable          = false;
      slice->m_deblockingFilterBetaOffsetDiv2   = pps->m_deblockingFilterBetaOffsetDiv2;
      slice->m_deblockingFilterTcOffsetDiv2     = pps->m_deblockingFilterTcOffsetDiv2;
      slice->m_deblockingFilterCbBetaOffsetDiv2 = pps->m_deblockingFilterBetaOffsetDiv2;
      slice->m_deblockingFilterCbTcOffsetDiv2   = pps->m_deblockingFilterTcOffsetDiv2;
      slice->m_deblockingFilterCrBetaOffsetDiv2 = pps->m_deblockingFilterBetaOffsetDiv2;
      slice->m_deblockingFilterCrTcOffsetDiv2   = pps->m_deblockingFilterTcOffsetDiv2;
    }
  }
  else
  {
    for (int i = 0; i < numSlices; i++)
    {
      Slice *slice = pic->m_slices[i];

      slice->m_deblockingFilterOverrideFlag     = true;
      slice->m_deblockingFilterDisable          = false;
      slice->m_deblockingFilterBetaOffsetDiv2   = betaOffsetDiv2Best;
      slice->m_deblockingFilterTcOffsetDiv2     = tcOffsetDiv2Best;
      slice->m_deblockingFilterCbBetaOffsetDiv2 = betaOffsetDiv2Best;
      slice->m_deblockingFilterCbTcOffsetDiv2   = tcOffsetDiv2Best;
      slice->m_deblockingFilterCrBetaOffsetDiv2 = betaOffsetDiv2Best;
      slice->m_deblockingFilterCrTcOffsetDiv2   = tcOffsetDiv2Best;
    }
  }
}

bool EncGOP::xCheckMaxTidILRefPics(int layerIdx, Picture *refPic, bool currentPicIsIRAP)
{
  const VPS *vps                  = refPic->m_cs->vps;
  const int  refLayerIdx          = vps == nullptr ? 0 : vps->m_generalLayerIdx[refPic->m_layerId];
  const int  maxTidILRefPicsPlus1 = vps->getMaxTidIlRefPicsPlus1(layerIdx, refLayerIdx);

  // -1 means not set
  if (maxTidILRefPicsPlus1 < 0)
  {
    return true;
  }

  // 0 allows only IRAP pictures to use inter-layer prediction
  if (maxTidILRefPicsPlus1 == 0)
  {
    return currentPicIsIRAP;
  }

  // all other cases filter by temporalID
  return (refPic->m_temporalId < maxTidILRefPicsPlus1);
}

void EncGOP::xCreateExplicitReferencePictureSetFromReference(Slice *slice, PicList &rcListPic,
                                                             const ReferencePictureList *rpl0,
                                                             const ReferencePictureList *rpl1)
{
  const int pocCycle = 1 << slice->m_sps->m_bitsForPoc;

  const bool interLayerPresent = slice->m_sps->m_interLayerPresentFlag;

  Picture   *curPic   = slice->m_pic;
  const VPS *vps      = curPic->m_cs->vps;
  int        layerIdx = vps->m_generalLayerIdx[curPic->m_layerId];

  const bool isIntraLayerPredAllowed =
    (vps->m_vpsIndependentLayerFlag[layerIdx] || vps->m_vpsCfgPredDirection[slice->m_uiTLayer] != 1) &&
    (!slice->isIRAP() || (m_encCfg->m_avoidIntraInDepLayer && layerIdx != 0));
  const bool isInterLayerPredAllowed =
    !vps->m_vpsIndependentLayerFlag[layerIdx] && vps->m_vpsCfgPredDirection[slice->m_uiTLayer] != 2;

  ReferencePictureList localRpl[NUM_RPL01] = { ReferencePictureList(interLayerPresent),
                                               ReferencePictureList(interLayerPresent) };

  uint32_t numStrp[NUM_RPL01] = { 0, 0 };
  uint32_t numLtrp[NUM_RPL01] = { 0, 0 };
  uint32_t numIlrp[NUM_RPL01] = { 0, 0 };
  uint32_t num[NUM_RPL01]     = { 0, 0 };

  static_vector<int, MAX_NUM_REF_PICS> higherTLayerRefs[NUM_RPL01];
  static_vector<int, MAX_NUM_REF_PICS> inactiveRefs[NUM_RPL01];

  for (const auto l: { RPL0, RPL1 })
  {
    const ReferencePictureList *rpl = l == RPL0 ? rpl0 : rpl1;

    if (isIntraLayerPredAllowed)
    {
      for (int ii = 0; ii < rpl->getNumRefEntries(); ii++)
      {
        if (!rpl->m_isInterLayerRefPic[ii])
        {
          for (const auto &pic: rcListPic)
          {
            if (pic->m_layerId == curPic->m_layerId && pic->m_referenced &&
                !slice->isPocRestrictedByDRAP(pic->m_poc, pic->m_precedingDRAP) &&
                !slice->isPocRestrictedByEdrap(pic->m_poc))
            {
              const bool isAvailable = !rpl->m_isLongtermRefPic[ii]
                ? pic->m_poc == slice->m_poc + rpl->m_refPicIdentifier[ii]
                : (pic->m_poc & (pocCycle - 1)) == rpl->m_refPicIdentifier[ii];
              if (isAvailable)
              {
                if (slice->isIRAP())
                {
                  inactiveRefs[l].push_back(ii);
                }
                else if (pic->m_temporalId > curPic->m_temporalId)
                {
                  higherTLayerRefs[l].push_back(ii);
                }
                else if (num[l] >= rpl->m_numberOfActivePictures - rpl->m_numberOfInterLayerPictures && layerIdx != 0 &&
                         vps != nullptr && !vps->m_vpsAllIndependentLayersFlag && isInterLayerPredAllowed)
                {
                  inactiveRefs[l].push_back(ii);
                }
                else
                {
                  localRpl[l].setRefPicIdentifier(num[l], rpl->m_refPicIdentifier[ii], rpl->m_isLongtermRefPic[ii],
                                                  false, NOT_VALID);
                  num[l]++;
                  numStrp[l] += rpl->m_isLongtermRefPic[ii] ? 0 : 1;
                  numLtrp[l] += rpl->m_isLongtermRefPic[ii] && !rpl->m_isInterLayerRefPic[ii] ? 1 : 0;
                }
                break;
              }
            }
          }
        }
      }
    }

    // inter-layer reference pictures are added to the end of the reference picture list
    if (layerIdx != 0 && vps != nullptr && !vps->m_vpsAllIndependentLayersFlag && isInterLayerPredAllowed)
    {
      for (const auto &pic: rcListPic)
      {
        int refLayerIdx = vps->m_generalLayerIdx[pic->m_layerId];
        if (pic->m_referenced && pic->m_poc == curPic->m_poc && vps->m_vpsDirectRefLayerFlag[layerIdx][refLayerIdx] &&
            xCheckMaxTidILRefPics(layerIdx, pic, slice->isIRAP()))
        {
          localRpl[l].setRefPicIdentifier(num[l], 0, true, true, vps->m_interLayerRefIdx[layerIdx][refLayerIdx]);
          num[l]++;
          numIlrp[l]++;
        }
      }
    }
  }

  uint32_t numPrev[NUM_RPL01] = { num[RPL0], num[RPL1] };

  // Copy from other list if we have fewer than active ref pics

  bool isDisallowMixedRefPic = slice->m_sps->m_allRplEntriesHasSameSignFlag;

  for (const auto l: { RPL0, RPL1 })
  {
    const ReferencePictureList *rpl = l == RPL0 ? rpl0 : rpl1;
    const auto                  k   = l == RPL0 ? RPL1 : RPL0;

    int numOfNeedToFill = rpl->m_numberOfActivePictures - num[l];

    for (int ii = 0; numOfNeedToFill > 0 && ii < numPrev[k]; ii++)
    {
      const int  identifier   = localRpl[k].m_refPicIdentifier[ii];
      const bool isLongTerm   = localRpl[k].m_isLongtermRefPic[ii];
      const bool isInterLayer = localRpl[k].m_isInterLayerRefPic[ii];

      // Make sure this copy is not already present
      bool canIncludeThis = true;
      for (int jj = 0; jj < num[l]; jj++)
      {
        if (identifier == localRpl[l].m_refPicIdentifier[jj] && isLongTerm == localRpl[l].m_isLongtermRefPic[jj] &&
            isInterLayer == localRpl[l].m_isInterLayerRefPic[jj])
        {
          canIncludeThis = false;
          break;
        }

        if (isDisallowMixedRefPic && !isLongTerm && !localRpl[l].m_isLongtermRefPic[jj])
        {
          const bool sameSign = (identifier ^ localRpl[l].m_refPicIdentifier[jj]) >= 0;
          if (!sameSign)
          {
            canIncludeThis = false;
            break;
          }
        }
      }
      if (canIncludeThis)
      {
        localRpl[l].setRefPicIdentifier(num[l], identifier, isLongTerm, isInterLayer,
                                        localRpl[k].m_interLayerRefPicIdx[ii]);
        num[l]++;
        numStrp[l] += isLongTerm ? 0 : 1;
        numLtrp[l] += isLongTerm && !isInterLayer ? 1 : 0;
        numIlrp[l] += isInterLayer ? 1 : 0;
        numOfNeedToFill--;
      }
    }
  }

  const uint32_t numValidRefs[NUM_RPL01] = { num[RPL0], num[RPL1] };

  for (const auto l: { RPL0, RPL1 })
  {
    const ReferencePictureList *rpl = l == RPL0 ? rpl0 : rpl1;

    // now add inactive refs
    for (const int i: inactiveRefs[l])
    {
      localRpl[l].setRefPicIdentifier(num[l], rpl->m_refPicIdentifier[i], rpl->m_isLongtermRefPic[i], false, NOT_VALID);
      num[l]++;
      numStrp[l] += rpl->m_isLongtermRefPic[i] ? 0 : 1;
      numLtrp[l] += rpl->m_isLongtermRefPic[i] && !rpl->m_isInterLayerRefPic[i] ? 1 : 0;
    }

    if (slice->m_enableDRAPSEI && l == RPL0)
    {
      localRpl[l].m_numberOfShorttermPictures  = numStrp[l];
      localRpl[l].m_numberOfLongtermPictures   = numLtrp[l];
      localRpl[l].m_numberOfInterLayerPictures = numIlrp[l];

      if (!slice->isIRAP() && !slice->isPOCInRefPicList(&localRpl[l], slice->m_iAssociatedIRAPPOC))
      {
        if (slice->m_useLTforDRAP && !slice->isPOCInRefPicList(rpl1, slice->m_iAssociatedIRAPPOC))
        {
          // Adding associated IRAP as longterm picture
          localRpl[l].setRefPicIdentifier(num[l], slice->m_iAssociatedIRAPPOC, true, false, 0);
          num[l]++;
          numLtrp[l]++;
        }
        else
        {
          // Adding associated IRAP as shortterm picture
          localRpl[l].setRefPicIdentifier(num[l], slice->m_iAssociatedIRAPPOC - slice->m_poc, false, false, 0);
          num[l]++;
          numStrp[l]++;
        }
      }
    }
    if (slice->m_enableEdrapSEI && l == RPL0)
    {
      localRpl[l].m_numberOfShorttermPictures  = numStrp[l];
      localRpl[l].m_numberOfLongtermPictures   = numLtrp[l];
      localRpl[l].m_numberOfInterLayerPictures = numIlrp[l];

      for (int i = 0; i < slice->m_edrapNumRefRapPics; i++)
      {
        int refPoc = slice->getEdrapRefRapId(i) == 0 ? slice->m_iAssociatedIRAPPOC
                                                     : slice->getEdrapRefRapId(i) * m_encCfg->m_edrapPeriod;
        if (slice->isPOCInRefPicList(&localRpl[l], refPoc))
        {
          continue;
        }
        if (slice->m_useLTforEdrap && !slice->isPOCInRefPicList(rpl1, refPoc))
        {
          // Added as longterm picture
          localRpl[l].setRefPicIdentifier(num[l], refPoc, true, false, 0);
          num[l]++;
          numLtrp[l]++;
        }
        else
        {
          // Added as shortterm picture
          localRpl[l].setRefPicIdentifier(num[l], refPoc - slice->m_poc, false, false, 0);
          num[l]++;
          numStrp[l]++;
        }
      }
    }

    // now add higher TId refs
    for (const int i: higherTLayerRefs[l])
    {
      localRpl[l].setRefPicIdentifier(num[l], rpl->m_refPicIdentifier[i], rpl->m_isLongtermRefPic[i], false, NOT_VALID);
      num[l]++;
      numStrp[l] += rpl->m_isLongtermRefPic[i] ? 0 : 1;
      numLtrp[l] += rpl->m_isLongtermRefPic[i] && !rpl->m_isInterLayerRefPic[i] ? 1 : 0;
    }
  }

  for (const auto l: { RPL0, RPL1 })
  {
    const ReferencePictureList *rpl = l == RPL0 ? rpl0 : rpl1;

    localRpl[l].m_numberOfLongtermPictures   = numLtrp[l];
    localRpl[l].m_numberOfShorttermPictures  = numStrp[l];
    localRpl[l].m_numberOfInterLayerPictures = numIlrp[l];
    localRpl[l].m_numberOfActivePictures     = (std::min<int>(numValidRefs[l], rpl->m_numberOfActivePictures));
    localRpl[l].m_ltrpInSliceHeaderFlag      = true;
    slice->m_rplIdx[l]                       = -1;
    slice->m_rpl[l]                          = localRpl[l];
  }

  // Ensure that all pictures in the RefRapIds are included in a reference list.
  for (int i = 0; i < slice->m_edrapNumRefRapPics; i++)
  {
    int refPoc = slice->getEdrapRefRapId(i) == 0 ? slice->m_iAssociatedIRAPPOC
                                                 : slice->getEdrapRefRapId(i) * m_encCfg->m_edrapPeriod;
    if (!slice->isPOCInRefPicList(&localRpl[RPL0], refPoc) && !slice->isPOCInRefPicList(&localRpl[RPL1], refPoc))
    {
      slice->deleteEdrapRefRapIds(i);
    }
  }
}

//! \}
