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

#include <vector>
#include "HRD.h"
#include "ProfileTierLevel.h"

struct DpbParameters
{
  int maxDecPicBuffering[MAX_TLAYER] { 0 };
  int maxNumReorderPics[MAX_TLAYER] { 0 };
  int maxLatencyIncreasePlus1[MAX_TLAYER] { 0 };
};

struct VPS
{
  int                                m_vpsId { 0 };
  uint32_t                           m_maxLayers { 1 };
  uint32_t                           m_vpsMaxSubLayers { 7 };
  uint32_t                           m_vpsLayerId[MAX_VPS_LAYERS] {};
  bool                               m_vpsDefaultPtlDpbHrdMaxTidFlag { true };
  bool                               m_vpsAllIndependentLayersFlag { true };
  uint32_t                           m_vpsCfgPredDirection[MAX_VPS_SUBLAYERS] {};
  bool                               m_vpsIndependentLayerFlag[MAX_VPS_LAYERS] {};
  bool                               m_vpsDirectRefLayerFlag[MAX_VPS_LAYERS][MAX_VPS_LAYERS] {};
  std::vector<std::vector<uint32_t>> m_vpsMaxTidIlRefPicsPlus1 {};
  bool                               m_vpsEachLayerIsAnOlsFlag { true };
  uint32_t                           m_vpsOlsModeIdc { 0 };
  uint32_t                           m_vpsNumOutputLayerSets { 1 };
  bool                               m_vpsOlsOutputLayerFlag[MAX_NUM_OLSS][MAX_VPS_LAYERS] {};
  uint32_t                           m_directRefLayerIdx[MAX_VPS_LAYERS][MAX_VPS_LAYERS] {};
  uint32_t                           m_generalLayerIdx[MAX_VPS_LAYERS] {};
  bool                               m_ptPresentFlag[MAX_NUM_OLSS] {};
  uint32_t                           m_ptlMaxTemporalId[MAX_NUM_OLSS] {};
  std::vector<ProfileTierLevel>      m_vpsProfileTierLevel {};
  uint32_t                           m_olsPtlIdx[MAX_NUM_OLSS] {};

  // stores index ( ilrp_idx within 0 .. NumDirectRefLayers ) of the dependent reference layers
  uint32_t                      m_interLayerRefIdx[MAX_VPS_LAYERS][MAX_VPS_LAYERS] {};
  bool                          m_vpsExtensionFlag { false };
  bool                          m_vpsGeneralHrdParamsPresentFlag { false };
  bool                          m_vpsSublayerCpbParamsPresentFlag { false };
  uint32_t                      m_numOlsTimingHrdParamsMinus1 { 0 };
  uint32_t                      m_hrdMaxTid[MAX_NUM_OLSS] {};
  uint32_t                      m_olsTimingHrdIdx[MAX_NUM_OLSS] {};
  GeneralHrdParams              m_generalHrdParams {};
  std::vector<Size>             m_olsDpbPicSize {};
  std::vector<int>              m_olsDpbParamsIdx {};
  std::vector<std::vector<int>> m_outputLayerIdInOls {};
  std::vector<std::vector<int>> m_numSubLayersInLayerInOLS {};

  // mapping from multi-layer OLS index to OLS index. Initialized in deriveOutputLayerSets()
  // m_multiLayerOlsIdxToOlsIdx[n] is the OLSidx of the n-th multi-layer OLS.
  std::vector<int> m_multiLayerOlsIdxToOlsIdx {};

  std::vector<std::vector<OlsHrdParams>> m_olsHrdParams {};

  int                        m_totalNumOLSs { 1 };
  int                        m_numMultiLayeredOlss { 0 };
  uint32_t                   m_multiLayerOlsIdx[MAX_NUM_OLSS] {};
  int                        m_numDpbParams { 0 };
  std::vector<DpbParameters> m_dpbParameters {};
  bool                       m_sublayerDpbParamsPresentFlag { false };
  std::vector<int>           m_dpbMaxTemporalId {};
  std::vector<int>           m_targetOutputLayerIdSet {};          // set of LayerIds to be outputted
  std::vector<int> m_targetLayerIdSet {};   // set of LayerIds to be included in the sub-bitstream extraction process.
  int              m_targetOlsIdx { 0 };
  std::vector<int> m_numOutputLayersInOls {};
  std::vector<int> m_numLayersInOls {};
  std::vector<std::vector<int>> m_layerIdInOls {};
  std::vector<ChromaFormat>     m_olsDpbChromaFormatIdc {};
  std::vector<int>              m_olsDpbBitDepthMinus8 {};

  VPS();

  uint32_t getMaxTidIlRefPicsPlus1(const uint32_t layerIdx, const uint32_t refLayerIdx) const
  {
    CHECK(layerIdx >= m_vpsMaxTidIlRefPicsPlus1.size(), "layerIdx out of bounds");
    CHECK(refLayerIdx >= m_vpsMaxTidIlRefPicsPlus1[layerIdx].size(), "refLayerIdx out of bounds");
    return m_vpsMaxTidIlRefPicsPlus1[layerIdx][refLayerIdx];
  }
  void setMaxTidIlRefPicsPlus1(const uint32_t layerIdx, const uint32_t refLayerIdx, const uint32_t i)
  {
    CHECK(layerIdx >= m_vpsMaxTidIlRefPicsPlus1.size(), "layerIdx out of bounds");
    CHECK(refLayerIdx >= m_vpsMaxTidIlRefPicsPlus1[layerIdx].size(), "refLayerIdx out of bounds");
    m_vpsMaxTidIlRefPicsPlus1[layerIdx][refLayerIdx] = i;
  }

  void     setMaxTidIlRefPicsPlus1(std::vector<std::vector<uint32_t>> i) { m_vpsMaxTidIlRefPicsPlus1 = i; }
  uint32_t getNumPtls() const { return (uint32_t)m_vpsProfileTierLevel.size(); }
  void     setNumPtls(uint32_t val) { m_vpsProfileTierLevel.resize(val); }
  int      getMaxDecPicBuffering(int temporalId) const
  {
    return m_dpbParameters[m_olsDpbParamsIdx[m_targetOlsIdx]].maxDecPicBuffering[temporalId];
  }
  int getMaxNumReorderPics(int temporalId) const
  {
    return m_dpbParameters[m_olsDpbParamsIdx[m_targetOlsIdx]].maxNumReorderPics[temporalId];
  }
  void     deriveOutputLayerSets();
  void     deriveTargetOutputLayerSet(int targetOlsIdx);
  int      deriveTargetOLSIdx();
  uint32_t getMaxTidinTOls(int m_targetOlsIdx);
  void     checkVPS();
};
