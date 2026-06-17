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

#include "CommonDef.h"
#include "Unit.h"
#include "Slice.h"
#include "Picture.h"
#include "dtrace_next.h"

#include "UnitTools.h"

template<class BaseAlfParam, class VtmAlfParam, class EcmAlfParam>
ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam>::ApsAlfParamHelper()
{
  switch (m_alfType)
  {
  case AlfType::ECM:
    m_param = new EcmAlfParam;
    break;
  case AlfType::UNDEFINED:
    m_param = nullptr;
    break;
  case AlfType::VTM:
    m_param = new VtmAlfParam;
    break;
  default:
    THROW("Invalid ALF type.");
  }
}

template<class BaseAlfParam, class VtmAlfParam, class EcmAlfParam>
ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam>::ApsAlfParamHelper(const ApsAlfParamHelper &other)
  : m_param(nullptr)
{
  CHECKD(other.m_param == nullptr, "Trying to copy invalid parameters.");
  CHECKD(m_alfType == AlfType::UNDEFINED, "ALF type is not defined yet.");
  if (m_alfType == AlfType::ECM)
  {
    auto *param = new EcmAlfParam;
    *param      = other.getEcmParamNoCheck();
    m_param     = param;
  }
  else
  {
    auto *param = new VtmAlfParam;
    *param      = other.getVtmParamNoCheck();
    m_param     = param;
  }
}

template<class BaseAlfParam, class VtmAlfParam, class EcmAlfParam>
ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam>::ApsAlfParamHelper(ApsAlfParamHelper &&other) noexcept
  : m_param(other.m_param)
{
  other.m_param = nullptr;
}

template<class BaseAlfParam, class VtmAlfParam, class EcmAlfParam>
ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam>::~ApsAlfParamHelper()
{
  if (m_param != nullptr)
  {
    delete m_param;
    m_param = nullptr;
  }
}

template<class BaseAlfParam, class VtmAlfParam, class EcmAlfParam>
ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam> &
  ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam>::operator=(const ApsAlfParamHelper &other)
{
  if (this == &other)
  {
    return *this;
  }
  CHECKD(other.m_param == nullptr, "Trying to copy invalid parameters.");
  CHECKD(m_alfType == AlfType::UNDEFINED, "ALF type is not defined yet.");
  if (m_alfType == AlfType::ECM)
  {
    getEcmParamNoCheck() = other.getEcmParamNoCheck();
  }
  else
  {
    getVtmParamNoCheck() = other.getVtmParamNoCheck();
  }

  return *this;
}

template<class BaseAlfParam, class VtmAlfParam, class EcmAlfParam>
ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam> &
  ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam>::operator=(ApsAlfParamHelper &&other) noexcept
{
  if (m_param != nullptr)
  {
    delete m_param;
  }
  m_param       = other.m_param;
  other.m_param = nullptr;

  return *this;
}

template<class BaseAlfParam, class VtmAlfParam, class EcmAlfParam>
void ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam>::setAlfType(const bool isEcmAlf)
{
  const AlfType alfType = isEcmAlf ? AlfType::ECM : AlfType::VTM;
  CHECK((m_alfType != alfType) && (m_alfType != AlfType::UNDEFINED), "Some other ALF type has already been defined.");
  m_alfType = alfType;
}

template<class BaseAlfParam, class VtmAlfParam, class EcmAlfParam>
void ApsAlfParamHelper<BaseAlfParam, VtmAlfParam, EcmAlfParam>::create()
{
  if (m_param == nullptr)
  {
    switch (m_alfType)
    {
    case AlfType::ECM:
      m_param = new EcmAlfParam;
      break;
    case AlfType::VTM:
      m_param = new VtmAlfParam;
      break;
    default:
      THROW("ALF type is not defined yet.");
    }
  }
}

// Instantiate ApsAlfParamHelper for ALF (alias ApsAlfParam) and CCALF (alias ApsCcAlfParam).
template class ApsAlfParamHelper<AlfParameters::AlfParamBase, AlfParametersVtm::AlfParam, AlfParametersEcm::AlfParam>;
template class ApsAlfParamHelper<AlfParameters::CcAlfFilterParamBase, AlfParametersVtm::CcAlfFilterParam,
                                 AlfParametersEcm::CcAlfFilterParam>;
// Instantiate the comparison operators for ApsAlfParam and ApsCcAlfParam.
template bool operator==(const ApsAlfParam &, const ApsAlfParam &);
template bool operator==(const ApsCcAlfParam &, const ApsCcAlfParam &);

Slice::Slice()
{
  for (uint32_t i = 0; i < MAX_TSRC_RICE; i++)
  {
    m_riceBit[i] = 0;
  }

  m_cntRightBottom = 0;

  for (uint32_t i = 0; i < NUM_RPL01; i++)
  {
    m_numRefIdx[i] = 0;
  }

  for (uint32_t component = 0; component < MAX_NUM_COMP; component++)
  {
    m_lambdas[component]            = 0.0;
    m_sliceChromaQpDelta[component] = 0;
  }
  m_sliceChromaQpDelta[JOINT_CbCr] = 0;

  for (int idx = 0; idx < MAX_NUM_REF; idx++)
  {
    m_list1IdxToList0Idx[idx] = -1;
  }

  for (int iNumCount = 0; iNumCount < MAX_NUM_REF; iNumCount++)
  {
    for (uint32_t i = 0; i < NUM_RPL01; i++)
    {
      m_refPicList[i][iNumCount] = nullptr;
      m_refPOCList[i][iNumCount] = 0;
    }
  }

  resetWpScaling();
  initWpAcDcParam();

  m_saoEnabledFlag.fill(false);
  m_ccSaoComParam.reset();
  resetCcSaoEnabledFlag();

  memset(m_alfApss, 0, sizeof(m_alfApss));
  resetAlfEnabledFlag();
  std::fill_n(m_newAlfFixFiltSetCandIdx, MAX_NUM_COMP, -1);
  m_ccSaoComParam.reset();
  resetCcSaoEnabledFlag();
  m_ccAlfCbEnabledFlag = 0;
  m_ccAlfCrEnabledFlag = 0;

  m_sliceMap.initSliceMap();
}

Slice::~Slice() { m_sliceMap.initSliceMap(); }

void Slice::initSlice()
{
  for (uint32_t i = 0; i < NUM_RPL01; i++)
  {
    m_numRefIdx[i] = 0;
  }
  m_colFromL0Flag           = true;
  m_colRefIdx               = 0;
  m_lmcsEnabledFlag         = 0;
  m_explicitScalingListUsed = 0;
  m_noOutputOfPriorPicsFlag = 0;

  m_checkLdc = false;

  m_biDirPred            = false;
  m_lmChromaCheckDisable = false;
  m_symRefIdx[0]         = -1;
  m_symRefIdx[1]         = -1;

  for (uint32_t component = 0; component < MAX_NUM_COMP; component++)
  {
    m_sliceChromaQpDelta[component] = 0;
  }
  m_sliceChromaQpDelta[JOINT_CbCr] = 0;

  m_substreamSizes.clear();
  m_cabacInitFlag = false;
#if ENABLE_CABAC_DUMP
  m_cabacInitSliceType = I_SLICE;
#endif
  m_enableDRAPSEI      = false;
  m_useLTforDRAP       = false;
  m_isDRAP             = false;
  m_latestDRAPPOC      = MAX_INT;
  m_lumaPelMax         = 0;
  m_lumaPelMin         = 0;
  m_edrapRapId         = 0;
  m_enableEdrapSEI     = false;
  m_edrapRapId         = 0;
  m_useLTforEdrap      = false;
  m_edrapNumRefRapPics = 0;
  m_edrapRefRapIds.resize(0);
  m_latestEDRAPPOC                     = MAX_INT;
  m_latestEdrapLeadingPicDecodableFlag = false;
  resetAlfEnabledFlag();
  std::fill_n(m_newAlfFixFiltSetCandIdx, MAX_NUM_COMP, -1);
  m_ccAlfCbEnabledFlag = 0;
  m_ccAlfCrEnabledFlag = 0;
  m_ccAlfCbApsId       = -1;
  m_ccAlfCrApsId       = -1;
  m_nuhLayerId         = 0;
  m_lfCccmEnabledFlag  = false;

#if ENABLE_NNLF
  m_nnlfUnifiedParam.reset();
#endif
}

void Slice::inheritFromPicHeader(PicHeader *picHeader, const PPS *pps, const SPS *sps)
{
  if (pps->m_rplInfoInPhFlag)
  {
    for (const auto l: { RPL0, RPL1 })
    {
      const int rplIdx = picHeader->m_rplIdx[l];
      m_rplIdx[l]      = rplIdx;
      m_rpl[l]         = rplIdx == -1 ? picHeader->m_rpl[l] : *sps->m_rplList[l].getReferencePictureList(rplIdx);
    }
  }

  m_deblockingFilterDisable        = picHeader->m_deblockingFilterDisable;
  m_deblockingFilterBetaOffsetDiv2 = picHeader->m_deblockingFilterBetaOffsetDiv2;
  m_deblockingFilterTcOffsetDiv2   = picHeader->m_deblockingFilterTcOffsetDiv2;
  if (pps->m_usePPSChromaTool)
  {
    m_deblockingFilterCbBetaOffsetDiv2 = picHeader->m_deblockingFilterCbBetaOffsetDiv2;
    m_deblockingFilterCbTcOffsetDiv2   = picHeader->m_deblockingFilterCbTcOffsetDiv2;
    m_deblockingFilterCrBetaOffsetDiv2 = picHeader->m_deblockingFilterCrBetaOffsetDiv2;
    m_deblockingFilterCrTcOffsetDiv2   = picHeader->m_deblockingFilterCrTcOffsetDiv2;
  }
  else
  {
    m_deblockingFilterCbBetaOffsetDiv2 = m_deblockingFilterBetaOffsetDiv2;
    m_deblockingFilterCbTcOffsetDiv2   = m_deblockingFilterTcOffsetDiv2;
    m_deblockingFilterCrBetaOffsetDiv2 = m_deblockingFilterBetaOffsetDiv2;
    m_deblockingFilterCrTcOffsetDiv2   = m_deblockingFilterTcOffsetDiv2;
  }

  m_saoEnabledFlag[ChannelType::LUMA]   = picHeader->m_saoEnabledFlag[ChannelType::LUMA];
  m_saoEnabledFlag[ChannelType::CHROMA] = picHeader->m_saoEnabledFlag[ChannelType::CHROMA];
  m_ccSaoComParam.enabled[COMP_Y]       = picHeader->m_ccSaoEnabledFlag[COMP_Y];
  m_ccSaoComParam.enabled[COMP_Cb]      = picHeader->m_ccSaoEnabledFlag[COMP_Cb];
  m_ccSaoComParam.enabled[COMP_Cr]      = picHeader->m_ccSaoEnabledFlag[COMP_Cr];
  m_alfEnabledFlag[COMP_Y]              = picHeader->m_alfEnabledFlag[COMP_Y];
  m_alfEnabledFlag[COMP_Cb]             = picHeader->m_alfEnabledFlag[COMP_Cb];
  m_alfEnabledFlag[COMP_Cr]             = picHeader->m_alfEnabledFlag[COMP_Cr];
  if (sps->m_alfImprovementsEnabledFlag)
  {
    memcpy(m_newAlfFixFiltSetCandIdx, picHeader->m_newAlfFixFiltSetCandIdx, sizeof(m_newAlfFixFiltSetCandIdx));
  }
  else
  {
    std::fill_n(m_newAlfFixFiltSetCandIdx, MAX_NUM_COMP, -1);
  }
  m_numAlfApsIdsLuma   = picHeader->m_numAlfApsIdsLuma;
  m_alfApsIdsLuma      = picHeader->m_alfApsIdsLuma;
  m_alfApsIdChroma     = picHeader->m_alfApsIdChroma;
  m_ccAlfCbEnabledFlag = picHeader->m_ccalfEnabledFlag[COMP_Cb];
  m_ccAlfCrEnabledFlag = picHeader->m_ccalfEnabledFlag[COMP_Cr];
  m_ccAlfCbApsId       = picHeader->m_ccAlfCbApsId;
  m_ccAlfCrApsId       = picHeader->m_ccAlfCrApsId;

  AlfParameters::CcAlfFilterParamBase &ccAlfFilterParam = m_ccAlfFilterParam.getParam();
  ccAlfFilterParam.ccAlfFilterEnabled[COMP_Cb - 1]      = picHeader->m_ccalfEnabledFlag[COMP_Cb];
  ccAlfFilterParam.ccAlfFilterEnabled[COMP_Cr - 1]      = picHeader->m_ccalfEnabledFlag[COMP_Cr];
}

void Slice::setNumSubstream(const SPS *sps, const PPS *pps)
{
  uint32_t ctuAddr, ctuX, ctuY;
  m_numSubstream = 0;

  // count the number of CTUs that align with either the start of a tile, or with an entropy coding sync point
  // ignore the first CTU since it doesn't count as an entry point
  for (uint32_t i = 1; i < m_sliceMap.m_numCtuInSlice; i++)
  {
    ctuAddr = m_sliceMap.getCtuAddrInSlice(i);
    ctuX    = (ctuAddr % pps->m_picWidthInCtu);
    ctuY    = (ctuAddr / pps->m_picWidthInCtu);

    if (pps->ctuIsTileColBd(ctuX) && (pps->ctuIsTileRowBd(ctuY) || sps->m_entropyCodingSyncEnabledFlag))
    {
      m_numSubstream++;
    }
  }
}

void Slice::setNumEntryPoints(const SPS *sps, const PPS *pps)
{
  uint32_t ctuAddr, ctuX, ctuY;
  uint32_t prevCtuAddr, prevCtuX, prevCtuY;
  m_numEntryPoints = 0;

  if (!sps->m_entryPointPresentFlag)
  {
    return;
  }

  // count the number of CTUs that align with either the start of a tile, or with an entropy coding sync point
  // ignore the first CTU since it doesn't count as an entry point
  for (uint32_t i = 1; i < m_sliceMap.m_numCtuInSlice; i++)
  {
    ctuAddr     = m_sliceMap.getCtuAddrInSlice(i);
    ctuX        = (ctuAddr % pps->m_picWidthInCtu);
    ctuY        = (ctuAddr / pps->m_picWidthInCtu);
    prevCtuAddr = m_sliceMap.getCtuAddrInSlice(i - 1);
    prevCtuX    = (prevCtuAddr % pps->m_picWidthInCtu);
    prevCtuY    = (prevCtuAddr / pps->m_picWidthInCtu);

    if (pps->ctuToTileRowBd(ctuY) != pps->ctuToTileRowBd(prevCtuY) ||
        pps->ctuToTileColBd(ctuX) != pps->ctuToTileColBd(prevCtuX) ||
        (ctuY != prevCtuY && sps->m_entropyCodingSyncEnabledFlag))
    {
      m_numEntryPoints++;
    }
  }
}

void Slice::setDefaultClpRng(const SPS &sps)
{
  m_clpRngs.comp[COMP_Y].min = m_clpRngs.comp[COMP_Cb].min = m_clpRngs.comp[COMP_Cr].min = 0;
  m_clpRngs.comp[COMP_Y].max  = (1 << sps.m_bitDepths[ChannelType::LUMA]) - 1;
  m_clpRngs.comp[COMP_Y].bd   = sps.m_bitDepths[ChannelType::LUMA];
  m_clpRngs.comp[COMP_Y].n    = 0;
  m_clpRngs.comp[COMP_Cb].max = m_clpRngs.comp[COMP_Cr].max = (1 << sps.m_bitDepths[ChannelType::CHROMA]) - 1;
  m_clpRngs.comp[COMP_Cb].bd = m_clpRngs.comp[COMP_Cr].bd = sps.m_bitDepths[ChannelType::CHROMA];
  m_clpRngs.comp[COMP_Cb].n = m_clpRngs.comp[COMP_Cr].n = 0;
  m_clpRngs.used = m_clpRngs.chroma = false;
}

bool Slice::getRapPicFlag() const
{
  return m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL || m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
    m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA;
}

void Slice::sortPicList(PicList &picList)
{
  picList.sort([](Picture *const &a, Picture *const &b)
               { return a->m_poc < b->m_poc || (a->m_poc == b->m_poc && a->m_layerId < b->m_layerId); });
}

ClpRng Slice::setNewClipRange(bool considerReshaper, std::vector<Pel> *fwdReshaperMapping, CompID CompID)
{
  ClpRng outputClpRng = clpRng(CompID);

  if (isLuma(CompID))
  {
    if (m_lmcsEnabledFlag && considerReshaper)
    {
      outputClpRng.min = fwdReshaperMapping->at(m_lumaPelMin);
      outputClpRng.max = fwdReshaperMapping->at(m_lumaPelMax);
    }
    else
    {
      outputClpRng.min = m_lumaPelMin;
      outputClpRng.max = m_lumaPelMax;
    }
  }

  return outputClpRng;
}

int Slice::getClipDeltaShift() const
{
  int clipDeltaShift = 0;
  if (m_adaptiveClipQuant)
  {
    clipDeltaShift = ADAPTIVE_CLIP_SHIFT_DELTA_VALUE_1;
  }
  else
  {
    clipDeltaShift = ADAPTIVE_CLIP_SHIFT_DELTA_VALUE_0;
  }
  clipDeltaShift += std::max(0, m_sps->m_bitDepths[ChannelType::LUMA] - 10);

  return clipDeltaShift;
}

Picture *Slice::xGetRefPic(PicList &rcListPic, const int poc, const int layerId)
{
  // return a nullptr, if picture is not found
  Picture *refPic = nullptr;

  for (auto &currPic: rcListPic)
  {
    if (currPic->m_poc == poc && currPic->m_layerId == layerId)
    {
      refPic = currPic;
      break;
    }
  }
  return refPic;
}

Picture *Slice::xGetLongTermRefPic(PicList &rcListPic, const int poc, const bool pocHasMsb, const int layerId)
{
  // return a nullptr, if picture is not found or the found picture is not long-term
  Picture  *refPic   = nullptr;
  const int pocCycle = 1 << m_sps->m_bitsForPoc;

  const int refPoc = pocHasMsb ? poc : (poc & (pocCycle - 1));

  for (auto &currPic: rcListPic)
  {
    if (currPic->m_poc != this->m_poc && currPic->m_referenced && currPic->m_layerId == layerId)
    {
      int currPicPoc = pocHasMsb ? currPic->m_poc : (currPic->m_poc & (pocCycle - 1));
      if (refPoc == currPicPoc)
      {
        if (currPic->m_longTerm)
        {
          refPic = currPic;
        }
        break;
      }
    }
  }

  return refPic;
}

Picture *Slice::xGetLongTermRefPicCandidate(PicList &rcListPic, const int poc, const bool pocHasMsb, const int layerId)
{
  // return a nullptr, if picture is not found (might be a short-term or a long-term)
  Picture  *refPic   = nullptr;
  const int pocCycle = 1 << m_sps->m_bitsForPoc;

  const int refPoc = pocHasMsb ? poc : (poc & (pocCycle - 1));

  for (auto &currPic: rcListPic)
  {
    if (currPic->m_poc != this->m_poc && currPic->m_referenced && currPic->m_layerId == layerId)
    {
      int currPicPoc = pocHasMsb ? currPic->m_poc : (currPic->m_poc & (pocCycle - 1));
      if (refPoc == currPicPoc)
      {
        refPic = currPic;
        break;
      }
    }
  }

  return refPic;
}

void Slice::setRefPOCList()
{
  for (int dir = 0; dir < NUM_RPL01; dir++)
  {
    for (int numRefIdx = 0; numRefIdx < m_numRefIdx[dir]; numRefIdx++)
    {
      m_refPOCList[dir][numRefIdx] = m_refPicList[dir][numRefIdx]->m_poc;
    }
  }
}

void Slice::setRefRefIdxList()
{
  memset(m_refRefIdxList, -1, sizeof(int) * NUM_RPL01 * MAX_NUM_REF * NUM_RPL01 * (MAX_NUM_REF + 1) * NUM_RPL01);
  for (int iDir = 0; iDir < NUM_RPL01; iDir++)
  {
    for (int iNumRefIdx = 0; iNumRefIdx < m_numRefIdx[iDir]; iNumRefIdx++)
    {
      const Picture *const refPic   = getRefPic(RefPicList(iDir), iNumRefIdx);
      const Slice *const   refSlice = refPic->m_slices[0];
      for (int iRefDir = 0; iRefDir < NUM_RPL01; iRefDir++)
      {
        for (int iNumRefRefIdx = 0;
             iNumRefRefIdx < refSlice->m_numRefIdx[RefPicList(iRefDir)] + (refSlice->m_ibcFlag ? 1 : 0);
             iNumRefRefIdx++)
        {
          int refRefPOC;
          if (iNumRefRefIdx == refSlice->m_numRefIdx[RefPicList(iRefDir)])
          {
            iNumRefRefIdx = MAX_NUM_REF;
            refRefPOC     = refSlice->m_poc;
          }
          else
          {
            refRefPOC = refSlice->getRefPOC(RefPicList(iRefDir), iNumRefRefIdx);
          }
          for (int iCurrDir = 0; iCurrDir < NUM_RPL01; iCurrDir++)
          {
            for (int iCurrRefIdx = 0; iCurrRefIdx < m_numRefIdx[iCurrDir]; iCurrRefIdx++)
            {
              if (refRefPOC == getRefPOC(RefPicList(iCurrDir), iCurrRefIdx))
              {
                m_refRefIdxList[iDir][iNumRefIdx][iRefDir][iNumRefRefIdx][iCurrDir] = iCurrRefIdx;
                break;
              }
            }
          }
        }
      }
    }
  }
}

void Slice::setList1IdxToList0Idx()
{
  for (int idxL1 = 0; idxL1 < m_numRefIdx[RPL1]; idxL1++)
  {
    m_list1IdxToList0Idx[idxL1] = -1;
    for (int idxL0 = 0; idxL0 < m_numRefIdx[RPL0]; idxL0++)
    {
      if (m_refPicList[RPL0][idxL0]->m_poc == m_refPicList[RPL1][idxL1]->m_poc)
      {
        m_list1IdxToList0Idx[idxL1] = idxL0;
        break;
      }
    }
  }
}

void Slice::constructRefPicList(PicList &rcListPic)
{
  ::memset(m_isUsedAsLongTerm, 0, sizeof(m_isUsedAsLongTerm));
  if (m_eSliceType == I_SLICE)
  {
    ::memset(m_refPicList, 0, sizeof(m_refPicList));
    ::memset(m_numRefIdx, 0, sizeof(m_numRefIdx));
    return;
  }

  for (const auto l: { RPL0, RPL1 })
  {
    const uint32_t numOfActiveRef = m_numRefIdx[l];

    for (int ii = 0; ii < m_rpl[l].getNumRefEntries(); ii++)
    {
      Picture *refPic = nullptr;

      if (m_rpl[l].m_isInterLayerRefPic[ii])
      {
        const VPS *vps = m_pic->m_cs->vps;

        const int interLayerIdx = m_rpl[l].m_interLayerRefPicIdx[ii];
        CHECK(interLayerIdx == NOT_VALID, "Wrong ILRP index");

        const int layerIdx   = vps->m_generalLayerIdx[m_pic->m_layerId];
        const int refLayerId = vps->m_vpsLayerId[vps->m_directRefLayerIdx[layerIdx][interLayerIdx]];

        refPic = xGetRefPic(rcListPic, m_poc, refLayerId);

        refPic->m_longTerm = true;
      }
      else if (!m_rpl[l].m_isLongtermRefPic[ii])
      {
        refPic = xGetRefPic(rcListPic, m_poc + m_rpl[l].m_refPicIdentifier[ii], m_pic->m_layerId);

        refPic->m_longTerm = false;
      }
      else
      {
        const int  pocBits   = m_sps->m_bitsForPoc;
        const int  pocMask   = (1 << pocBits) - 1;
        int        ltrpPoc   = m_rpl[l].m_refPicIdentifier[ii] & pocMask;
        const bool pocHasMsb = m_rpl[l].m_deltaPocMSBPresentFlag[ii];
        if (pocHasMsb)
        {
          ltrpPoc += (m_poc & ~pocMask) - m_rpl[l].m_deltaPOCMSBCycleLT[ii] * (pocMask + 1);
        }
        refPic = xGetLongTermRefPicCandidate(rcListPic, ltrpPoc, pocHasMsb, m_pic->m_layerId);

        refPic->m_longTerm = true;
      }
      if (ii < numOfActiveRef)
      {
        m_refPicList[l][ii]       = refPic;
        m_isUsedAsLongTerm[l][ii] = refPic->m_longTerm;
      }
    }
  }
}

void Slice::checkColRefIdx(uint32_t curSliceSegmentIdx, const Picture *pic)
{
  int    i;
  Slice *curSlice      = pic->m_slices[curSliceSegmentIdx];
  int    currColRefPOC = curSlice->getRefPOC(RefPicList(1 - curSlice->m_colFromL0Flag), curSlice->m_colRefIdx);

  for (i = curSliceSegmentIdx - 1; i >= 0; i--)
  {
    const Slice *preSlice = pic->m_slices[i];
    if (preSlice->m_eSliceType != I_SLICE)
    {
      const int preColRefPOC = preSlice->getRefPOC(RefPicList(1 - preSlice->m_colFromL0Flag), preSlice->m_colRefIdx);
      if (currColRefPOC != preColRefPOC)
      {
        THROW("sh_collocated_ref_idx shall always be the same for all slices of a coded picture!");
      }
      else
      {
        break;
      }
    }
  }
}

void Slice::checkCRA(const ReferencePictureList *pRPL0, const ReferencePictureList *pRPL1, const int pocCRA,
                     CheckCRAFlags &flags, PicList &rcListPic)
{
  if (pocCRA < MAX_UINT && m_poc > pocCRA)
  {
    if (flags.seenLeadingFieldPic && flags.trailingFieldHadRefIssue)
    {
      THROW("Invalid state");
    }

    uint32_t numRefPic = pRPL0->getNumRefEntries();
    for (int i = 0; i < numRefPic; i++)
    {
      if (!pRPL0->m_isLongtermRefPic[i])
      {
        if (m_poc + pRPL0->m_refPicIdentifier[i] < pocCRA)
        {
          // report error immediately if we are
          //   processing frames
          //   or this is second trailing field picture
          //   or this is trailing field picture after leading field picture
          //   or this is active reference picture of trailing field picture
          CHECK(!m_sps->m_fieldSeqFlag || flags.seenTrailingFieldPic || flags.seenLeadingFieldPic ||
                  i < m_numRefIdx[RPL0],
                "Invalid state");

          // otherwise, we are checking non-active reference picture of first trailing field picture
          flags.trailingFieldHadRefIssue = true;
          return;
        }
      }
      else if (!pRPL0->m_isInterLayerRefPic[i])
      {
        int pocBits = m_sps->m_bitsForPoc;
        int pocMask = (1 << pocBits) - 1;
        int ltrpPoc = pRPL0->m_refPicIdentifier[i] & pocMask;
        if (pRPL0->m_deltaPocMSBPresentFlag[i])
        {
          ltrpPoc += m_poc - pRPL0->m_deltaPOCMSBCycleLT[i] * (pocMask + 1) - (m_poc & pocMask);
        }
        const Picture *ltrp =
          xGetLongTermRefPic(rcListPic, ltrpPoc, pRPL0->m_deltaPocMSBPresentFlag[i], m_pic->m_layerId);
        if (ltrp == nullptr || ltrp->m_poc < pocCRA)
        {
          CHECK(!m_sps->m_fieldSeqFlag || flags.seenTrailingFieldPic || flags.seenLeadingFieldPic ||
                  i < m_numRefIdx[RPL0],
                "Invalid state");
          flags.trailingFieldHadRefIssue = true;
          return;
        }
      }
    }
    numRefPic = pRPL1->getNumRefEntries();
    for (int i = 0; i < numRefPic; i++)
    {
      if (!pRPL1->m_isLongtermRefPic[i])
      {
        if (m_poc + pRPL1->m_refPicIdentifier[i] < pocCRA)
        {
          CHECK(!m_sps->m_fieldSeqFlag || flags.seenTrailingFieldPic || flags.seenLeadingFieldPic ||
                  i < m_numRefIdx[RPL1],
                "Invalid state");
          flags.trailingFieldHadRefIssue = true;
          return;
        }
      }
      else if (!pRPL1->m_isInterLayerRefPic[i])
      {
        int pocBits = m_sps->m_bitsForPoc;
        int pocMask = (1 << pocBits) - 1;
        int ltrpPoc = pRPL1->m_refPicIdentifier[i] & pocMask;
        if (pRPL1->m_deltaPocMSBPresentFlag[i])
        {
          ltrpPoc += m_poc - pRPL1->m_deltaPOCMSBCycleLT[i] * (pocMask + 1) - (m_poc & pocMask);
        }
        const Picture *ltrp =
          xGetLongTermRefPic(rcListPic, ltrpPoc, pRPL1->m_deltaPocMSBPresentFlag[i], m_pic->m_layerId);
        if (ltrp == nullptr || ltrp->m_poc < pocCRA)
        {
          CHECK(!m_sps->m_fieldSeqFlag || flags.seenTrailingFieldPic || flags.seenLeadingFieldPic ||
                  i < m_numRefIdx[RPL1],
                "Invalid state");
          flags.trailingFieldHadRefIssue = true;
          return;
        }
      }
    }

    if (m_sps->m_fieldSeqFlag)
    {
      flags.seenTrailingFieldPic = true;
    }
  }
  else if (m_sps->m_fieldSeqFlag && m_poc < pocCRA)
  {
    flags.seenLeadingFieldPic      = true;
    flags.trailingFieldHadRefIssue = false;
  }
}

void Slice::checkRPL(const ReferencePictureList *pRPL0, const ReferencePictureList *pRPL1,
                     const int associatedIRAPDecodingOrderNumber, PicList &rcListPic)
{
  Picture *refPic;
  int      refPicPOC;
  int      refPicDecodingOrderNumber;

  int irapPOC = m_iAssociatedIRAPPOC;

  const int numEntries[NUM_RPL01]       = { pRPL0->getNumRefEntries(), pRPL1->getNumRefEntries() };
  const int numActiveEntries[NUM_RPL01] = { m_numRefIdx[RPL0], m_numRefIdx[RPL1] };

  const ReferencePictureList *rpl[NUM_RPL01] = { pRPL0, pRPL1 };

  const bool fieldSeqFlag = m_sps->m_fieldSeqFlag;
  const int  layerIdx     = m_pic->m_cs->vps == nullptr ? 0 : m_pic->m_cs->vps->m_generalLayerIdx[m_pic->m_layerId];

  for (int refPicList = 0; refPicList < 2; refPicList++)
  {
    for (int i = 0; i < numEntries[refPicList]; i++)
    {
      if (rpl[refPicList]->m_isInterLayerRefPic[i])
      {
        int refLayerId =
          m_pic->m_cs->vps
            ->m_vpsLayerId[m_pic->m_cs->vps->m_directRefLayerIdx[layerIdx][rpl[refPicList]->m_interLayerRefPicIdx[i]]];
        refPic    = xGetRefPic(rcListPic, m_poc, refLayerId);
        refPicPOC = refPic->m_poc;
      }
      else if (!rpl[refPicList]->m_isLongtermRefPic[i])
      {
        refPicPOC = m_poc + rpl[refPicList]->m_refPicIdentifier[i];
        refPic    = xGetRefPic(rcListPic, refPicPOC, m_pic->m_layerId);
      }
      else
      {
        int pocBits = m_sps->m_bitsForPoc;
        int pocMask = (1 << pocBits) - 1;
        int ltrpPoc = rpl[refPicList]->m_refPicIdentifier[i] & pocMask;
        if (rpl[refPicList]->m_deltaPocMSBPresentFlag[i])
        {
          ltrpPoc += m_poc - rpl[refPicList]->m_deltaPOCMSBCycleLT[i] * (pocMask + 1) - (m_poc & pocMask);
        }
        refPic = xGetLongTermRefPic(rcListPic, ltrpPoc, rpl[refPicList]->m_deltaPocMSBPresentFlag[i], m_pic->m_layerId);
        refPicPOC = refPic->m_poc;
      }
      if (refPic) // the checks are for all reference picture, but we may not have an inactive reference picture, if
                  // starting with a CRA
      {
        refPicDecodingOrderNumber = refPic->m_decodingOrderNumber;

        if (m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA || m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
            m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP)
        {
          CHECK(refPicPOC < irapPOC || refPicDecodingOrderNumber < associatedIRAPDecodingOrderNumber,
                "When the current picture, with nuh_layer_id equal to a particular value layerId, "
                "is an IRAP picture, there shall be no picture referred to by an entry in RefPicList[ 0 ] that "
                "precedes, in output order or decoding order, any preceding IRAP picture "
                "with nuh_layer_id equal to layerId in decoding order (when present).");
        }

        if (irapPOC < m_poc && !fieldSeqFlag)
        {
          CHECK(refPicPOC < irapPOC || refPicDecodingOrderNumber < associatedIRAPDecodingOrderNumber,
                "When the current picture follows an IRAP picture having the same value "
                "of nuh_layer_id and the leading pictures, if any, associated with that IRAP picture, in both decoding "
                "order and output order, there shall be no picture referred "
                "to by an entry in RefPicList[ 0 ] or RefPicList[ 1 ] that precedes that IRAP picture in output order "
                "or decoding order.");
        }

        // Generated reference picture does not have picture header
        const bool nonReferencePictureFlag = refPic->m_nonReferencePictureFlag;
        CHECK(refPic == m_pic || nonReferencePictureFlag,
              "The picture referred to by each entry in RefPicList[ 0 ] or RefPicList[ 1 ] shall not be the current "
              "picture and shall have ph_non_ref_pic_flag equal to 0");

        if (i < numActiveEntries[refPicList])
        {
          if (irapPOC < m_poc)
          {
            CHECK(refPicPOC < irapPOC || refPicDecodingOrderNumber < associatedIRAPDecodingOrderNumber,
                  "When the current picture follows an IRAP picture having the same value "
                  "of nuh_layer_id in both decoding order and output order, there shall be no picture referred to by "
                  "an active entry in RefPicList[ 0 ] or RefPicList[ 1 ] that "
                  "precedes that IRAP picture in output order or decoding order.");
          }

          // Checking this: "When the current picture is a RADL picture, there shall be no active entry in RefPicList[ 0
          // ] or RefPicList[ 1 ] that is any of the following: A picture that precedes the associated IRAP picture in
          // decoding order"
          if (m_eNalUnitType == NAL_UNIT_CODED_SLICE_RADL)
          {
            CHECK(refPicDecodingOrderNumber < associatedIRAPDecodingOrderNumber,
                  "RADL picture detected that violate the rule that no active entry in RefPicList[] shall precede the "
                  "associated IRAP picture in decoding order");
            // Checking this: "When the current picture is a RADL picture, there shall be no active entry in RefPicList[
            // 0 ] or RefPicList[ 1 ] that is any of the following: A RASL picture with pps_mixed_nalu_types_in_pic_flag
            // is equal to 0
            for (int i = 0; i < refPic->m_numSlices; i++)
            {
              if (!refPic->m_mixedNaluTypesInPicFlag)
              {
                CHECK(refPic->m_slices[i]->m_eNalUnitType == NAL_UNIT_CODED_SLICE_RASL,
                      "When the current picture is a RADL picture, there shall be no active entry in RefPicList[ 0 ] "
                      "or RefPicList[ 1 ] that is a RASL picture with pps_mixed_nalu_types_in_pic_flag is equal to 0");
              }
            }
          }

          CHECK(refPic->m_temporalId > m_pic->m_temporalId,
                "The picture referred to by each active entry in RefPicList[ 0 ] or RefPicList[ 1 ] shall be present "
                "in the DPB and shall have TemporalId less than or equal to that of the current picture.");
        }
        // Add a constraint on an ILRP being either an IRAP picture or having TemporalId less than or equal to
        // Max (0, vps_max_tid_il_ref_pics_plus1[ refPicVpsLayerId ] - 1 ), with refPicVpsLayerId equal to the value of
        // the nuh_layer_id of the referenced picture.
        if (rpl[refPicList]->m_isInterLayerRefPic[i])
        {
          bool cond1 = (refPic->getPictureType() == NAL_UNIT_CODED_SLICE_GDR);
          bool cond2 = (refPic->m_slices[0]->m_picHeader->m_recoveryPocCnt == 0);
          bool cond3 = (refPic->m_cs->slice->isIRAP());

          const VPS *vps = refPic->m_cs->vps;
          const int  maxTidILRefPicsPlus1 =
            vps->getMaxTidIlRefPicsPlus1(layerIdx, vps->m_generalLayerIdx[refPic->m_layerId]);
          bool cond4 = (refPic->m_temporalId < maxTidILRefPicsPlus1);

          CHECK(!((cond1 && cond2) || cond3 || cond4),
                "Either of the following conditions shall apply for the picture referred to by each ILRP entry, when "
                "present, in RefPicList[ 0 ] or RefPicList[ 1 ] of a slice of the current picture:-The picture is a "
                "GDR picture with "
                "ph_recovery_poc_cnt equal to 0 or an IRAP picture."
                "-The picture has TemporalId less than vps_max_tid_il_ref_pics_plus1[ currLayerIdx ][ refLayerIdx ], "
                "where currLayerIdx and refLayerIdx are equal to "
                "GeneralLayerIdx[ nuh_layer_id ] and GeneralLayerIdx[ refpicLayerId ], respectively. ");
        }
      }
    }
  }
}

void Slice::checkSTSA(PicList &rcListPic)
{
  int      ii;
  Picture *refPic = nullptr;

  int numOfActiveRef = m_numRefIdx[RPL0];

  for (ii = 0; ii < numOfActiveRef; ii++)
  {
    refPic = m_refPicList[RPL0][ii];

    if (m_eNalUnitType == NAL_UNIT_CODED_SLICE_STSA && refPic->m_layerId == m_pic->m_layerId)
    {
      CHECK(refPic->m_temporalId == m_uiTLayer,
            "When the current picture is an STSA picture and nuh_layer_id equal to that of the current picture, there "
            "shall be no active entry in the RPL that has TemporalId equal to that of the current picture");
    }

    // Checking this: "When the current picture is a picture that follows, in decoding order, an STSA picture that has
    // TemporalId equal to that of the current picture, there shall be no picture that has TemporalId equal to that of
    // the current picture included as an active entry in RefPicList[ 0 ] or RefPicList[ 1 ] that precedes the STSA
    // picture in decoding order."
    CHECK(refPic->m_subLayerNonReferencePictureDueToSTSA,
          "The RPL of the current picture contains a picture that is not allowed in this temporal layer due to an "
          "earlier STSA picture");
  }

  numOfActiveRef = m_numRefIdx[RPL1];
  for (ii = 0; ii < numOfActiveRef; ii++)
  {
    refPic = m_refPicList[RPL1][ii];

    if (m_eNalUnitType == NAL_UNIT_CODED_SLICE_STSA && refPic->m_layerId == m_pic->m_layerId)
    {
      CHECK(refPic->m_temporalId == m_uiTLayer,
            "When the current picture is an STSA picture and nuh_layer_id equal to that of the current picture, there "
            "shall be no active entry in the RPL that has TemporalId equal to that of the current picture");
    }

    // Checking this: "When the current picture is a picture that follows, in decoding order, an STSA picture that has
    // TemporalId equal to that of the current picture, there shall be no picture that has TemporalId equal to that of
    // the current picture included as an active entry in RefPicList[ 0 ] or RefPicList[ 1 ] that precedes the STSA
    // picture in decoding order."
    CHECK(refPic->m_subLayerNonReferencePictureDueToSTSA,
          "The active RPL part of the current picture contains a picture that is not allowed in this temporal layer "
          "due to an earlier STSA picture");
  }

  // If the current picture is an STSA picture, make all reference pictures in the DPB with temporal
  // id equal to the temproal id of the current picture sub-layer non-reference pictures. The flag
  // subLayerNonReferencePictureDueToSTSA equal to true means that the picture may not be used for
  // reference by a picture that follows the current STSA picture in decoding order
  if (m_eNalUnitType == NAL_UNIT_CODED_SLICE_STSA)
  {
    PicList::iterator iterPic = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      refPic = *(iterPic++);
      if (!refPic->m_referenced || refPic->m_poc == m_poc)
      {
        continue;
      }

      if (refPic->m_temporalId == m_uiTLayer)
      {
        refPic->m_subLayerNonReferencePictureDueToSTSA = true;
      }
    }
  }
}

/** Function for marking the reference pictures when an IDR/CRA/CRANT/BLA/BLANT is encountered.
 * \param pocCRA POC of the CRA/CRANT/BLA/BLANT picture
 * \param bRefreshPending flag indicating if a deferred decoding refresh is pending
 * \param rcListPic reference to the reference picture list
 * This function marks the reference pictures as "unused for reference" in the following conditions.
 * If the nal_unit_type is IDR/BLA/BLANT, all pictures in the reference picture list
 * are marked as "unused for reference"
 *    If the nal_unit_type is BLA/BLANT, set the pocCRA to the temporal reference of the current picture.
 * Otherwise
 *    If the bRefreshPending flag is true (a deferred decoding refresh is pending) and the current
 *    temporal reference is greater than the temporal reference of the latest CRA/CRANT/BLA/BLANT picture (pocCRA),
 *    mark all reference pictures except the latest CRA/CRANT/BLA/BLANT picture as "unused for reference" and set
 *    the bRefreshPending flag to false.
 *    If the nal_unit_type is CRA/CRANT, set the bRefreshPending flag to true and pocCRA to the temporal
 *    reference of the current picture.
 * Note that the current picture is already placed in the reference list and its marking is not changed.
 * If the current picture has a nal_ref_idc that is not 0, it will remain marked as "used for reference".
 */
void Slice::decodingRefreshMarking(int &pocCRA, bool &bRefreshPending, PicList &rcListPic,
                                   const bool bEfficientFieldIRAPEnabled)
{
  Picture *pic;
  int      pocCurr = m_poc;

  if (m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
      m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP)   // IDR picture
  {
    // mark all pictures as not used for reference
    PicList::iterator iterPic = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      pic = *(iterPic);
      if (pic->m_poc != pocCurr)
      {
        pic->m_referenced = false;
        pic->m_hashMap.clearAll();
      }
      iterPic++;
    }
    if (bEfficientFieldIRAPEnabled)
    {
      bRefreshPending = true;
    }
  }
  else   // CRA or No DR
  {
    if (bEfficientFieldIRAPEnabled &&
        (m_iAssociatedIRAPType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
         m_iAssociatedIRAPType == NAL_UNIT_CODED_SLICE_IDR_W_RADL))
    {
      if (bRefreshPending == true && pocCurr > m_iLastIDR)   // IDR reference marking pending
      {
        PicList::iterator iterPic = rcListPic.begin();
        while (iterPic != rcListPic.end())
        {
          pic = *(iterPic);
          if (pic->m_poc != pocCurr && pic->m_poc != m_iLastIDR)
          {
            pic->m_referenced = false;
            pic->m_hashMap.clearAll();
          }
          iterPic++;
        }
        bRefreshPending = false;
      }
    }
    else
    {
      if (bRefreshPending == true && pocCurr > pocCRA)   // CRA reference marking pending
      {
        PicList::iterator iterPic = rcListPic.begin();
        while (iterPic != rcListPic.end())
        {
          pic = *(iterPic);
          if (pic->m_poc != pocCurr && pic->m_poc != pocCRA)
          {
            pic->m_referenced = false;
            pic->m_hashMap.clearAll();
          }
          iterPic++;
        }
        bRefreshPending = false;
      }
    }
    if (m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA)   // CRA picture found
    {
      bRefreshPending = true;
      pocCRA          = pocCurr;
    }
  }
}

void Slice::copySliceInfo(Slice *pSrc, bool cpyAlmostAll)
{
  CHECK(!pSrc, "Source is nullptr");

  int i, j;

  m_poc                              = pSrc->m_poc;
  m_eNalUnitType                     = pSrc->m_eNalUnitType;
  m_eSliceType                       = pSrc->m_eSliceType;
  m_iSliceQp                         = pSrc->m_iSliceQp;
  m_iSliceQpBase                     = pSrc->m_iSliceQpBase;
  m_chromaQpAdjEnabled               = pSrc->m_chromaQpAdjEnabled;
  m_deblockingFilterDisable          = pSrc->m_deblockingFilterDisable;
  m_deblockingFilterOverrideFlag     = pSrc->m_deblockingFilterOverrideFlag;
  m_deblockingFilterBetaOffsetDiv2   = pSrc->m_deblockingFilterBetaOffsetDiv2;
  m_deblockingFilterTcOffsetDiv2     = pSrc->m_deblockingFilterTcOffsetDiv2;
  m_deblockingFilterCbBetaOffsetDiv2 = pSrc->m_deblockingFilterCbBetaOffsetDiv2;
  m_deblockingFilterCbTcOffsetDiv2   = pSrc->m_deblockingFilterCbTcOffsetDiv2;
  m_deblockingFilterCrBetaOffsetDiv2 = pSrc->m_deblockingFilterCrBetaOffsetDiv2;
  m_deblockingFilterCrTcOffsetDiv2   = pSrc->m_deblockingFilterCrTcOffsetDiv2;
  m_depQuantEnabledIdc               = pSrc->m_depQuantEnabledIdc;
  m_signDataHidingEnabledFlag        = pSrc->m_signDataHidingEnabledFlag;
  m_tsResidualCodingDisabledFlag     = pSrc->m_tsResidualCodingDisabledFlag;
  m_tsrcIndex                        = pSrc->m_tsrcIndex;

  for (i = 0; i < MAX_TSRC_RICE; i++)
  {
    m_riceBit[i] = pSrc->m_riceBit[i];
  }
  m_reverseLastSigCoeffFlag = pSrc->m_reverseLastSigCoeffFlag;
  m_cntRightBottom          = pSrc->m_cntRightBottom;

  for (i = 0; i < NUM_RPL01; i++)
  {
    m_numRefIdx[i] = pSrc->m_numRefIdx[i];
  }

  for (i = 0; i < MAX_NUM_REF; i++)
  {
    m_list1IdxToList0Idx[i] = pSrc->m_list1IdxToList0Idx[i];
  }

  m_checkLdc = pSrc->m_checkLdc;

  m_biDirPred            = pSrc->m_biDirPred;
  m_lmChromaCheckDisable = pSrc->m_lmChromaCheckDisable;
  ;
  m_symRefIdx[0] = pSrc->m_symRefIdx[0];
  m_symRefIdx[1] = pSrc->m_symRefIdx[1];

  for (uint32_t component = 0; component < MAX_NUM_COMP; component++)
  {
    m_sliceChromaQpDelta[component] = pSrc->m_sliceChromaQpDelta[component];
  }
  m_sliceChromaQpDelta[JOINT_CbCr] = pSrc->m_sliceChromaQpDelta[JOINT_CbCr];

  for (i = 0; i < NUM_RPL01; i++)
  {
    for (j = 0; j < MAX_NUM_REF; j++)
    {
      m_refPicList[i][j]       = pSrc->m_refPicList[i][j];
      m_refPOCList[i][j]       = pSrc->m_refPOCList[i][j];
      m_isUsedAsLongTerm[i][j] = pSrc->m_isUsedAsLongTerm[i][j];
    }
    m_isUsedAsLongTerm[i][MAX_NUM_REF] = pSrc->m_isUsedAsLongTerm[i][MAX_NUM_REF];
  }
  if (cpyAlmostAll)
  {
    m_hierPredLayerIdx = pSrc->m_hierPredLayerIdx;
  }

  // access channel
  if (cpyAlmostAll)
  {
    m_rpl[RPL0] = pSrc->m_rpl[RPL0];
    m_rpl[RPL1] = pSrc->m_rpl[RPL1];
  }
  m_iLastIDR = pSrc->m_iLastIDR;

  if (cpyAlmostAll)
  {
    m_pic = pSrc->m_pic;
  }

  m_picHeader     = pSrc->m_picHeader;
  m_colFromL0Flag = pSrc->m_colFromL0Flag;
  m_colRefIdx     = pSrc->m_colRefIdx;

  if (cpyAlmostAll)
  {
    setLambdas(pSrc->getLambdas());
  }

  m_uiTLayer = pSrc->m_uiTLayer;

  m_sliceMap                = pSrc->m_sliceMap;
  m_independentSliceIdx     = pSrc->m_independentSliceIdx;
  m_nextSlice               = pSrc->m_nextSlice;
  m_clpRngs                 = pSrc->m_clpRngs;
  m_lmcsEnabledFlag         = pSrc->m_lmcsEnabledFlag;
  m_explicitScalingListUsed = pSrc->m_explicitScalingListUsed;

  m_pendingRasInit = pSrc->m_pendingRasInit;

  m_ibcFlag = pSrc->m_ibcFlag;

  for (uint32_t e = 0; e < NUM_RPL01; e++)
  {
    for (uint32_t n = 0; n < MAX_NUM_REF; n++)
    {
      memcpy(m_weightPredTable[e][n], pSrc->m_weightPredTable[e][n], sizeof(WPScalingParam) * MAX_NUM_COMP);
    }
  }

  m_saoEnabledFlag            = pSrc->m_saoEnabledFlag;
  m_ccSaoComParam             = pSrc->m_ccSaoComParam;
  m_ccSaoControl[COMP_Y]      = pSrc->m_ccSaoControl[COMP_Y];
  m_ccSaoControl[COMP_Cb]     = pSrc->m_ccSaoControl[COMP_Cb];
  m_ccSaoControl[COMP_Cr]     = pSrc->m_ccSaoControl[COMP_Cr];
  m_ccSaoEnabledFlag[COMP_Y]  = pSrc->m_ccSaoEnabledFlag[COMP_Y];
  m_ccSaoEnabledFlag[COMP_Cb] = pSrc->m_ccSaoEnabledFlag[COMP_Cb];
  m_ccSaoEnabledFlag[COMP_Cr] = pSrc->m_ccSaoEnabledFlag[COMP_Cr];

  m_cabacInitFlag = pSrc->m_cabacInitFlag;
#if ENABLE_CABAC_DUMP
  m_cabacInitSliceType = pSrc->m_cabacInitSliceType;
#endif
  memcpy(m_alfApss, pSrc->m_alfApss, sizeof(m_alfApss));   // this might be quite unsafe
  memcpy(m_alfEnabledFlag, pSrc->m_alfEnabledFlag, sizeof(m_alfEnabledFlag));
  memcpy(m_newAlfFixFiltSetCandIdx, pSrc->m_newAlfFixFiltSetCandIdx, sizeof(m_newAlfFixFiltSetCandIdx));
  m_numAlfApsIdsLuma = pSrc->m_numAlfApsIdsLuma;
  m_alfApsIdsLuma    = pSrc->m_alfApsIdsLuma;
  m_alfApsIdChroma   = pSrc->m_alfApsIdChroma;
  m_disableSATDForRd = pSrc->m_disableSATDForRd;
  m_isLossless       = pSrc->m_isLossless;

  if (cpyAlmostAll)
  {
    m_encCABACTableIdx = pSrc->m_encCABACTableIdx;
  }
  for (int i = 0; i < NUM_RPL01; i++)
  {
    for (int j = 0; j < MAX_NUM_REF_PICS; j++)
    {
      m_scalingRatio[i][j] = pSrc->m_scalingRatio[i][j];
    }
  }

  if (pSrc->m_sps->m_ccalfEnabledFlag)
  {
    m_ccAlfFilterParam      = pSrc->m_ccAlfFilterParam;
    m_ccAlfFilterControl[0] = pSrc->m_ccAlfFilterControl[0];
    m_ccAlfFilterControl[1] = pSrc->m_ccAlfFilterControl[1];
    m_ccAlfCbEnabledFlag    = pSrc->m_ccAlfCbEnabledFlag;
    m_ccAlfCrEnabledFlag    = pSrc->m_ccAlfCrEnabledFlag;
    m_ccAlfCbApsId          = pSrc->m_ccAlfCbApsId;
    m_ccAlfCrApsId          = pSrc->m_ccAlfCrApsId;
  }

  m_useLic            = pSrc->m_useLic;
  m_lumaPelMax        = pSrc->m_lumaPelMax;
  m_lumaPelMin        = pSrc->m_lumaPelMin;
  m_adaptiveClipQuant = pSrc->m_adaptiveClipQuant;

  if (pSrc->m_sps->m_lfCccmEnabledFlag)
  {
    m_lfCccmEnabledFlag       = pSrc->m_lfCccmEnabledFlag;
    m_lfCccmEnabled           = pSrc->m_lfCccmEnabled;
    m_lfCccmWindowSizeIndex   = pSrc->m_lfCccmWindowSizeIndex;
    m_lfCccmModelType         = pSrc->m_lfCccmModelType;
    m_lfCccmCTUMerge          = pSrc->m_lfCccmCTUMerge;
    m_lfCccmFrameLevelInherit = pSrc->m_lfCccmFrameLevelInherit;
  }
}

/** Function for checking if this is a switching-point
 */
bool Slice::isTemporalLayerSwitchingPoint(PicList &rcListPic) const
{
  // loop through all pictures in the reference picture buffer
  PicList::iterator iterPic = rcListPic.begin();
  while (iterPic != rcListPic.end())
  {
    const Picture *pic = *(iterPic++);
    if (pic->m_referenced && pic->m_poc != m_poc)
    {
      if (pic->m_temporalId >= m_uiTLayer)
      {
        return false;
      }
    }
  }
  return true;
}

/** Function for checking if this is a STSA candidate
 */
bool Slice::isStepwiseTemporalLayerSwitchingPointCandidate(PicList &rcListPic) const
{
  PicList::iterator iterPic = rcListPic.begin();
  while (iterPic != rcListPic.end())
  {
    const Picture *pic = *(iterPic++);
    if (pic->m_referenced && pic->m_poc != m_poc)
    {
      if (pic->m_temporalId >= m_uiTLayer)
      {
        return false;
      }
    }
  }
  return true;
}

void Slice::checkLeadingPictureRestrictions(PicList &rcListPic, const PPS &pps) const
{
  int nalUnitType = this->m_eNalUnitType;

  // When a picture is a leading picture, it shall be a RADL or RASL picture.
  if (this->m_iAssociatedIRAPPOC > this->m_poc)
  {
    // check this only when pps_mixed_nalu_types_in_pic_flag is equal to 0
    if (!pps.m_mixedNaluTypesInPicFlag)
    {
      // Do not check IRAP pictures since they may get a POC lower than their associated IRAP
      if (nalUnitType < NAL_UNIT_CODED_SLICE_IDR_W_RADL || nalUnitType > NAL_UNIT_CODED_SLICE_CRA)
      {
        CHECK(nalUnitType != NAL_UNIT_CODED_SLICE_RASL && nalUnitType != NAL_UNIT_CODED_SLICE_RADL,
              "Invalid NAL unit type");
      }
    }
  }

  if (this->m_iAssociatedIRAPPOC <= this->m_poc)
  {
    if (!pps.m_mixedNaluTypesInPicFlag)
    {
      CHECK(nalUnitType == NAL_UNIT_CODED_SLICE_RASL || nalUnitType == NAL_UNIT_CODED_SLICE_RADL,
            "When a picture is not a leading picture, it shall not be a RADL or RASL picture.");
    }
  }

  // No RASL pictures shall be present in the bitstream that are associated with
  // an IDR picture.
  if (nalUnitType == NAL_UNIT_CODED_SLICE_RASL && !pps.m_mixedNaluTypesInPicFlag)
  {
    CHECK(this->m_iAssociatedIRAPType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
            this->m_iAssociatedIRAPType == NAL_UNIT_CODED_SLICE_IDR_W_RADL,
          "Invalid NAL unit type");
  }

  // No RADL pictures shall be present in the bitstream that are associated with
  // a BLA picture having nal_unit_type equal to BLA_N_LP or that are associated
  // with an IDR picture having nal_unit_type equal to IDR_N_LP.
  if (nalUnitType == NAL_UNIT_CODED_SLICE_RADL && !pps.m_mixedNaluTypesInPicFlag)
  {
    CHECK(this->m_iAssociatedIRAPType == NAL_UNIT_CODED_SLICE_IDR_N_LP, "Invalid NAL unit type");
  }

  // loop through all pictures in the reference picture buffer
  PicList::iterator iterPic       = rcListPic.begin();
  int               numNonLPFound = 0;
  while (iterPic != rcListPic.end())
  {
    Picture *pic = *(iterPic++);
    if (!pic->m_reconstructed)
    {
      continue;
    }
    if (pic->m_poc == this->m_poc)
    {
      continue;
    }
    const Slice *pcSlice = pic->m_slices[0];

    if (pcSlice->m_picHeader)   // Generated reference picture does not have picture header
    {
      if (pcSlice->m_picHeader->m_picOutputFlag == 1 && !this->m_noOutputOfPriorPicsFlag &&
          pic->m_layerId == this->m_nuhLayerId)
      {
        if ((nalUnitType == NAL_UNIT_CODED_SLICE_CRA || nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
             nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL) &&
            !pps.m_mixedNaluTypesInPicFlag)
        {
          CHECK(pic->m_poc >= this->m_poc,
                "Any picture, with nuh_layer_id equal to a particular value layerId, that precedes an IRAP picture "
                "with nuh_layer_id "
                "equal to layerId in decoding order shall precede the IRAP picture in output order.");
        }
      }

      if (pcSlice->m_picHeader->m_picOutputFlag == 1 && pic->m_layerId == this->m_nuhLayerId)
      {
        if (nalUnitType == NAL_UNIT_CODED_SLICE_RADL)
        {
          if (this->m_iAssociatedIRAPPOC > pcSlice->m_iAssociatedIRAPPOC && !pps.m_mixedNaluTypesInPicFlag)
          {
            if (this->m_iAssociatedIRAPPOC != pic->m_poc)
            {
              CHECK(pic->m_poc >= this->m_poc,
                    "Any picture, with nuh_layer_id equal to a particular value layerId, that precedes an IRAP picture "
                    "with nuh_layer_id "
                    "equal to layerId in decoding order shall precede any RADL picture associated with the IRAP "
                    "picture in output order.");
            }
          }
        }
      }

      if (pcSlice->m_picHeader->m_picOutputFlag == 1 && !this->m_picHeader->m_noOutputBeforeRecoveryFlag &&
          pic->m_layerId == this->m_nuhLayerId && nalUnitType != NAL_UNIT_CODED_SLICE_GDR &&
          this->m_picHeader->m_recoveryPocCnt != -1)
      {
        if (this->m_poc == this->m_picHeader->m_recoveryPocCnt + this->m_prevGDRInSameLayerPOC)
        {
          CHECK(pic->m_poc >= this->m_poc,
                "Any picture, with nuh_layer_id equal to a particular value layerId, that precedes a recovery point "
                "picture with "
                "nuh_layer_id equal to layerId in decoding order shall precede the recovery point picture in output "
                "order.");
        }
      }
    }

    if ((nalUnitType == NAL_UNIT_CODED_SLICE_RASL || nalUnitType == NAL_UNIT_CODED_SLICE_RADL) &&
        (pcSlice->m_eNalUnitType != NAL_UNIT_CODED_SLICE_RASL &&
         pcSlice->m_eNalUnitType != NAL_UNIT_CODED_SLICE_RADL) &&
        !pps.m_mixedNaluTypesInPicFlag)
    {
      if (pcSlice->m_iAssociatedIRAPPOC == this->m_iAssociatedIRAPPOC && pic->m_layerId == this->m_nuhLayerId)
      {
        numNonLPFound++;
        int limitNonLP = 0;
        if (pcSlice->m_sps->m_fieldSeqFlag)
        {
          limitNonLP = 1;
        }
        CHECK(pic->m_poc > this->m_iAssociatedIRAPPOC && numNonLPFound > limitNonLP,
              "If sps_field_seq_flag is equal to 0 and the current picture, with nuh_layer_id "
              "equal to a particular value layerId, is a leading picture associated with an IRAP picture, it shall "
              "precede, in decoding order, all non-leading "
              "pictures that are associated with the same IRAP picture.Otherwise, let picA and picB be the first and "
              "the last leading pictures, in decoding order, "
              "associated with an IRAP picture, respectively, there shall be at most one non-leading picture with "
              "nuh_layer_id equal to layerId preceding picA in "
              "decoding order, and there shall be no non-leading picture with nuh_layer_id equal to layerId between "
              "picA and picB in decoding order.");
      }
    }

    if (nalUnitType == NAL_UNIT_CODED_SLICE_RASL && !pps.m_mixedNaluTypesInPicFlag)
    {
      if ((this->m_iAssociatedIRAPType == NAL_UNIT_CODED_SLICE_CRA) &&
          this->m_iAssociatedIRAPPOC == pcSlice->m_iAssociatedIRAPPOC)
      {
        if (pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_RADL)
        {
          CHECK(pic->m_poc <= this->m_poc,
                "Any RASL picture associated with a CRA picture shall precede any RADL picture associated with the CRA "
                "picture in output order.");
        }
      }
    }

    if (nalUnitType == NAL_UNIT_CODED_SLICE_RASL && !pps.m_mixedNaluTypesInPicFlag)
    {
      if (this->m_iAssociatedIRAPType == NAL_UNIT_CODED_SLICE_CRA)
      {
        if (pcSlice->m_poc < this->m_iAssociatedIRAPPOC &&
            (pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
             pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
             pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA ||
             pcSlice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_GDR) &&
            pic->m_layerId == this->m_nuhLayerId)
        {
          CHECK(this->m_poc <= pcSlice->m_poc,
                "Any RASL picture, with nuh_layer_id equal to a particular value layerId, associated with a CRA "
                "picture shall follow, "
                "in output order, any IRAP or GDR picture with nuh_layer_id equal to layerId that precedes the CRA "
                "picture in decoding order.");
        }
      }
    }
  }
}

void Slice::checkSubpicTypeConstraints(PicList &rcListPic, const ReferencePictureList *pRPL0,
                                       const ReferencePictureList *pRPL1, const int prevIRAPSubpicDecOrderNo)
{
  int curSubpicIdx = m_pps->getSubPicIdxFromSubPicId(m_sliceSubPicId);

  if (m_pps->m_mixedNaluTypesInPicFlag && m_eSliceType != I_SLICE)
  {
    CHECK(!m_sps->m_subPicTreatedAsPicFlag[curSubpicIdx],
          "When pps_mixed_nalu_types_in_pic_flag is equal 1, the value of sps_subpic_treated_as_pic_flag shall be "
          "equal to 1 "
          "for all the subpictures that are in the picture and contain at least one P or B slice");
  }

  int nalUnitType       = m_eNalUnitType;
  int prevIRAPSubpicPOC = this->m_prevIRAPSubpicPOC;

  if (getCtuAddrInSlice(0) == m_pps->m_subPics[curSubpicIdx].m_firstCtuInSubPic)
  {
    // subpicture type related constraints invoked only if the current slice is the first slice of a subpicture
    int prevGDRSubpicPOC   = this->m_prevGDRSubpicPOC;
    int prevIRAPSubpicType = this->m_prevIRAPSubpicType;

    if (prevIRAPSubpicPOC > m_poc &&
        (nalUnitType < NAL_UNIT_CODED_SLICE_IDR_W_RADL || nalUnitType > NAL_UNIT_CODED_SLICE_CRA))
    {
      CHECK(nalUnitType != NAL_UNIT_CODED_SLICE_RASL && nalUnitType != NAL_UNIT_CODED_SLICE_RADL,
            "When a subpicture is a leading subpicture of an IRAP subpicture, it shall be a RADL or RASL subpicture");
    }

    if (prevIRAPSubpicPOC <= m_poc)
    {
      CHECK(nalUnitType == NAL_UNIT_CODED_SLICE_RASL || nalUnitType == NAL_UNIT_CODED_SLICE_RADL,
            "When a subpicture is not a leading subpicture of an IRAP subpicture, it shall not be a RADL or RASL "
            "subpicture");
    }

    CHECK(
      nalUnitType == NAL_UNIT_CODED_SLICE_RASL &&
        (prevIRAPSubpicType == NAL_UNIT_CODED_SLICE_IDR_N_LP || prevIRAPSubpicType == NAL_UNIT_CODED_SLICE_IDR_W_RADL),
      "No RASL subpictures shall be present in the bitstream that are associated with an IDR subpicture");

    CHECK(nalUnitType == NAL_UNIT_CODED_SLICE_RADL && prevIRAPSubpicType == NAL_UNIT_CODED_SLICE_IDR_N_LP,
          "No RADL subpictures shall be present in the bitstream that are associated with an IDR subpicture having "
          "nal_unit_type equal to IDR_N_LP");

    // constraints related to current subpicture type and its preceding subpicture types
    PicList::iterator iterPic          = rcListPic.begin();
    int               numNonLeadingPic = 0;
    while (iterPic != rcListPic.end())
    {
      Picture *bufPic = *(iterPic++);
      if (!bufPic->m_reconstructed)
      {
        continue;
      }
      if (bufPic->m_poc == m_poc)
      {
        continue;
      }

      // identify the subpicture in the reference picture buffer that with nuh_layer_id equal to current subpicture
      // layerId and subpicture index equal to current subpicIdx
      bool isBufPicOutput             = false;
      int  bufSubpicType              = NAL_UNIT_INVALID;
      int  bufSubpicPrevIRAPSubpicPOC = 0;

      if (bufPic->m_slices[0]->m_picHeader != nullptr)   // Generated reference picture does not have picture header
      {
        for (int i = 0; i < bufPic->m_numSlices; i++)
        {
          if (bufPic->m_sliceSubpicIdx[i] == curSubpicIdx)
          {
            isBufPicOutput             = bufPic->m_slices[i]->m_picHeader->m_picOutputFlag;
            bufSubpicType              = bufPic->m_slices[i]->m_eNalUnitType;
            bufSubpicPrevIRAPSubpicPOC = bufPic->m_slices[i]->m_prevIRAPSubpicPOC;
            break;
          }
        }
      }

      if ((nalUnitType == NAL_UNIT_CODED_SLICE_CRA || nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
           nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL) &&
          !this->m_noOutputOfPriorPicsFlag && isBufPicOutput == 1 && bufPic->m_layerId == m_nuhLayerId)
      {
        CHECK(bufPic->m_poc >= m_poc,
              "Any subpicture, with nuh_layer_id equal to a particular value layerId and subpicture index equal to a "
              "particular value subpicIdx, that "
              "precedes, in decoding order, an IRAP subpicture with nuh_layer_id equal to layerId and subpicture index "
              "equal to subpicIdx shall precede, in output order, the "
              "IRAP subpicture");
      }

      if (nalUnitType == NAL_UNIT_CODED_SLICE_RADL && isBufPicOutput == 1 && bufPic->m_layerId == m_nuhLayerId &&
          prevIRAPSubpicPOC > bufSubpicPrevIRAPSubpicPOC && prevIRAPSubpicPOC != bufPic->m_poc)
      {
        CHECK(bufPic->m_poc >= m_poc,
              "Any subpicture, with nuh_layer_id equal to a particular value layerId and subpicture index equal to a "
              "particular value subpicIdx, that "
              "precedes, in decoding order, an IRAP subpicture with nuh_layer_id equal to layerId and subpicture index "
              "equal to subpicIdx shall precede, in output order, all "
              "its associated RADL subpictures");
      }

      if ((m_poc == m_picHeader->m_recoveryPocCnt + prevGDRSubpicPOC) && !this->m_noOutputOfPriorPicsFlag &&
          isBufPicOutput == 1 && bufPic->m_layerId == m_nuhLayerId && nalUnitType != NAL_UNIT_CODED_SLICE_GDR &&
          m_picHeader->m_recoveryPocCnt != -1)
      {
        CHECK(bufPic->m_poc >= m_poc,
              "Any subpicture, with nuh_layer_id equal to a particular value layerId and subpicture index equal to a "
              "particular value subpicIdx, that "
              "precedes, in decoding order, a subpicture with nuh_layer_id equal to layerId and subpicture index equal "
              "to subpicIdx in a recovery point picture shall precede "
              "that subpicture in the recovery point picture in output order");
      }

      if (nalUnitType == NAL_UNIT_CODED_SLICE_RASL && prevIRAPSubpicType == NAL_UNIT_CODED_SLICE_CRA &&
          bufSubpicType == NAL_UNIT_CODED_SLICE_RADL && prevIRAPSubpicPOC == bufSubpicPrevIRAPSubpicPOC)
      {
        CHECK(bufPic->m_poc <= m_poc,
              "Any RASL subpicture associated with a CRA subpicture shall precede any RADL subpicture associated with "
              "the CRA subpicture in output order");
      }

      if (nalUnitType == NAL_UNIT_CODED_SLICE_RASL && prevIRAPSubpicType == NAL_UNIT_CODED_SLICE_CRA &&
          bufPic->m_layerId == m_nuhLayerId && bufPic->m_poc < prevIRAPSubpicPOC)
      {
        if (bufSubpicType == NAL_UNIT_CODED_SLICE_IDR_N_LP || bufSubpicType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
            bufSubpicType == NAL_UNIT_CODED_SLICE_CRA || bufSubpicType == NAL_UNIT_CODED_SLICE_GDR)
        {
          CHECK(bufPic->m_poc >= m_poc,
                "Any RASL subpicture, with nuh_layer_id equal to a particular value layerId and subpicture index equal "
                "to a particular value subpicIdx, "
                "associated with a CRA subpicture shall follow, in output order, any IRAP or GDR subpicture , with "
                "nuh_layer_id equal to layerId and subpicture index equal to "
                "subpicIdx, that precedes the CRA subpicture in decoding order");
        }
      }

      if ((nalUnitType == NAL_UNIT_CODED_SLICE_RASL || nalUnitType == NAL_UNIT_CODED_SLICE_RADL) &&
          bufSubpicType != NAL_UNIT_CODED_SLICE_RASL && bufSubpicType != NAL_UNIT_CODED_SLICE_RADL &&
          bufSubpicPrevIRAPSubpicPOC == prevIRAPSubpicPOC && bufPic->m_layerId == m_nuhLayerId)
      {
        numNonLeadingPic++;
        int th = bufPic->m_cs->sps->m_fieldSeqFlag ? 1 : 0;
        CHECK(bufPic->m_poc > prevIRAPSubpicPOC && numNonLeadingPic > th,
              "If sps_field_seq_flag is equal to 0 and the current subpicture, with nuh_layer_id equal to a particular "
              "value "
              "layerId and subpicture index equal to a particular value subpicIdx, is a leading subpicture associated "
              "with an IRAP subpicture, it shall precede, in decoding order, "
              "all non-leading subpictures that are associated with the same IRAP subpicture. Otherwise, let subpicA "
              "and subpicB be the first and the last leading subpictures, in "
              "decoding order, associated with an IRAP subpicture, respectively, there shall be at most one "
              "non-leading subpicture with nuh_layer_id equal to layerId and subpicture "
              "index equal to subpicIdx preceding subpicA in decoding order, and there shall be no non-leading picture "
              "with nuh_layer_id equal to layerId and subpicture index equal "
              "to subpicIdx between picA and picB in decoding order");
      }
    }
  }

  // subpic RPL related constraints
  for (const auto l: { RPL0, RPL1 })
  {
    const ReferencePictureList *rpl = l == RPL0 ? pRPL0 : pRPL1;

    const int numEntries       = rpl->getNumRefEntries();
    const int numActiveEntries = m_numRefIdx[l];

    for (int i = 0; i < numEntries; i++)
    {
      Picture *refPic;
      int      refPicPOC;

      if (rpl->m_isInterLayerRefPic[i])
      {
        const VPS *vps        = m_pic->m_cs->vps;
        const int  layerIdx   = vps->m_generalLayerIdx[m_pic->m_layerId];
        const int  refLayerId = vps->m_vpsLayerId[vps->m_directRefLayerIdx[layerIdx][rpl->m_interLayerRefPicIdx[i]]];

        refPic    = xGetRefPic(rcListPic, m_poc, refLayerId);
        refPicPOC = refPic->m_poc;
      }
      else if (!rpl->m_isLongtermRefPic[i])
      {
        refPicPOC = m_poc + rpl->m_refPicIdentifier[i];
        refPic    = xGetRefPic(rcListPic, refPicPOC, m_pic->m_layerId);
      }
      else
      {
        int pocBits = m_sps->m_bitsForPoc;
        int pocMask = (1 << pocBits) - 1;
        int ltrpPoc = rpl->m_refPicIdentifier[i] & pocMask;
        if (rpl->m_deltaPocMSBPresentFlag[i])
        {
          ltrpPoc += m_poc - rpl->m_deltaPOCMSBCycleLT[i] * (pocMask + 1) - (m_poc & pocMask);
        }
        refPic    = xGetLongTermRefPic(rcListPic, ltrpPoc, rpl->m_deltaPocMSBPresentFlag[i], m_pic->m_layerId);
        refPicPOC = refPic->m_poc;
      }

      // checks are for all reference pictures, but inactive reference pictures may be missing if starting with a CRA
      if (refPic != nullptr)
      {
        const int refPicDecodingOrderNumber = refPic->m_decodingOrderNumber;

        if (nalUnitType == NAL_UNIT_CODED_SLICE_CRA || nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
            nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP)
        {
          CHECK(refPicPOC < prevIRAPSubpicPOC || refPicDecodingOrderNumber < prevIRAPSubpicDecOrderNo,
                "When the current subpicture, with nuh_layer_id equal to a particular value layerId and subpicture "
                "index equal to a particular value subpicIdx, is an IRAP subpicture, there shall be no picture "
                "referred to by an entry in RefPicList[i] that precedes, in output order or decoding order,any "
                "preceding picture, in decoding order (when present), containing an IRAP subpicture with nuh_layer_id "
                "equal to layerId and subpicture index equal to subpicIdx");
        }

        if (prevIRAPSubpicPOC < m_poc && !m_sps->m_fieldSeqFlag)
        {
          CHECK(refPicPOC < prevIRAPSubpicPOC || refPicDecodingOrderNumber < prevIRAPSubpicDecOrderNo,
                "When the current subpicture follows an IRAP subpicture having the same value of nuh_layer_id and the "
                "same value of subpicture index in both decoding and output order, there shall be no picture referred "
                "to by an active entry in RefPicList[ i ] that precedes the picture containing that IRAP subpicture in "
                "output order or decoding order");
        }

        if (i < numActiveEntries)
        {
          if (prevIRAPSubpicPOC < m_poc)
          {
            CHECK(refPicPOC < prevIRAPSubpicPOC || refPicDecodingOrderNumber < prevIRAPSubpicDecOrderNo,
                  "When the current subpicture follows an IRAP subpicture having the same value "
                  "of nuh_layer_id and the same value of subpicture index and the leading subpictures, if any, "
                  "associated with that IRAP subpicture in both decoding and output order, "
                  "there shall be no picture referred to by an entry in RefPicList[ i ] that precedes the picture "
                  "containing that IRAP subpicture in output order or decoding order");
          }

          if (nalUnitType == NAL_UNIT_CODED_SLICE_RADL)
          {
            CHECK(refPicDecodingOrderNumber < prevIRAPSubpicDecOrderNo,
                  "When the current subpicture, with nuh_layer_id equal to a particular value layerId and subpicture "
                  "index equal to a particular value subpicIdx, is a RADL subpicture, there shall be no active entry "
                  "in RefPicList[ i ] that is a picture that precedes the picture containing the associated IRAP "
                  "subpicture in decoding order");

            if (refPic->m_layerId == m_nuhLayerId)
            {
              for (int i = 0; i < refPic->m_numSlices; i++)
              {
                if (refPic->m_sliceSubpicIdx[i] == curSubpicIdx)
                {
                  CHECK(refPic->m_slices[i]->m_eNalUnitType == NAL_UNIT_CODED_SLICE_RASL,
                        "When the current subpicture, with nuh_layer_id equal to a particular value layerId and "
                        "subpicture index equal to a particular value subpicIdx, is a RADL subpicture, there shall be "
                        "no active entry in RefPicList[ i ] that is a picture with nuh_layer_id equal to layerId "
                        "containing a RASL subpicture with subpicture index equal to subpicIdx");
                }
              }
            }
          }
        }
      }
    }
  }
}

// Function for applying picture marking based on the Reference Picture List
void Slice::applyReferencePictureListBasedMarking(PicList &rcListPic, const ReferencePictureList *pRPL0,
                                                  const ReferencePictureList *pRPL1, const int layerId,
                                                  const PPS &pps) const
{
  checkLeadingPictureRestrictions(rcListPic, pps);

  // mark long-term reference pictures in List0
  for (const auto l: { RPL0, RPL1 })
  {
    const ReferencePictureList *rpl = l == RPL0 ? pRPL0 : pRPL1;

    for (int i = 0; i < rpl->getNumRefEntries(); i++)
    {
      if (!rpl->m_isLongtermRefPic[i] || rpl->m_isInterLayerRefPic[i])
      {
        continue;
      }

      bool isAvailable = false;
      for (const Picture *pic: rcListPic)
      {
        if (!pic->m_referenced)
        {
          continue;
        }
        int pocCycle = 1 << (pic->m_cs->sps->m_bitsForPoc);
        int curPoc   = pic->m_poc;
        int refPoc   = rpl->m_refPicIdentifier[i] & (pocCycle - 1);
        if (rpl->m_deltaPocMSBPresentFlag[i])
        {
          refPoc += m_poc - rpl->m_deltaPOCMSBCycleLT[i] * pocCycle - (m_poc & (pocCycle - 1));
        }
        else
        {
          curPoc = curPoc & (pocCycle - 1);
        }
        if (pic->m_longTerm && curPoc == refPoc && pic->m_referenced)
        {
          isAvailable = true;
          break;
        }
      }
      // if there was no such long-term check the short terms
      if (!isAvailable)
      {
        for (Picture *pic: rcListPic)
        {
          if (!pic->m_referenced)
          {
            continue;
          }
          int pocCycle = 1 << (pic->m_cs->sps->m_bitsForPoc);
          int curPoc   = pic->m_poc;
          int refPoc   = rpl->m_refPicIdentifier[i] & (pocCycle - 1);
          if (rpl->m_deltaPocMSBPresentFlag[i])
          {
            refPoc += m_poc - rpl->m_deltaPOCMSBCycleLT[i] * pocCycle - (m_poc & (pocCycle - 1));
          }
          else
          {
            curPoc = curPoc & (pocCycle - 1);
          }
          if (!pic->m_longTerm && curPoc == refPoc && pic->m_referenced)
          {
            isAvailable     = true;
            pic->m_longTerm = true;
            break;
          }
        }
      }
    }
  }

  if (isIDRorBLA() && !pps.m_mixedNaluTypesInPicFlag)
  {
    return;
  }

  // loop through all pictures in the reference picture buffer
  for (Picture *pic: rcListPic)
  {
    if (!pic->m_referenced)
    {
      continue;
    }

    bool isReference = false;

    for (const auto l: { RPL0, RPL1 })
    {
      const ReferencePictureList *rpl = l == RPL0 ? pRPL0 : pRPL1;

      // loop through all pictures in the Reference Picture Set
      // to see if the picture should be kept as reference picture
      for (int i = 0; !isReference && i < rpl->getNumRefEntries(); i++)
      {
        if (rpl->m_isInterLayerRefPic[i])
        {
          // Diagonal inter-layer prediction is not allowed
          CHECK(rpl->m_refPicIdentifier[i], "ILRP identifier should be 0");

          if (pic->m_poc == m_poc)
          {
            isReference     = true;
            pic->m_longTerm = true;
          }
        }
        else if (pic->m_layerId == layerId)
        {
          if (!rpl->m_isLongtermRefPic[i])
          {
            if (pic->m_poc == m_poc + rpl->m_refPicIdentifier[i])
            {
              isReference     = true;
              pic->m_longTerm = false;
            }
          }
          else
          {
            const int pocCycle = 1 << pic->m_cs->sps->m_bitsForPoc;
            const int pocMask  = pocCycle - 1;
            int       curPoc   = pic->m_poc;
            int       refPoc   = rpl->m_refPicIdentifier[i] & pocMask;
            if (rpl->m_deltaPocMSBPresentFlag[i])
            {
              refPoc += (m_poc & ~pocMask) - rpl->m_deltaPOCMSBCycleLT[i] * pocCycle;
            }
            else
            {
              curPoc &= pocMask;
            }
            if (pic->m_longTerm && curPoc == refPoc)
            {
              isReference     = true;
              pic->m_longTerm = true;
            }
          }
        }
      }
    }

    // mark the picture as "unused for reference" if it is not in
    // the Reference Picture List
    if (pic->m_layerId == layerId && pic->m_poc != m_poc && !isReference)
    {
      pic->m_referenced = false;
      pic->m_longTerm   = false;
    }

    // sanity checks
    if (pic->m_referenced)
    {
      // check that pictures of higher temporal layers are not used
      CHECK(pic->m_usedByCurr && pic->m_temporalId > this->m_uiTLayer, "Invalid state");
    }
  }
}

int Slice::checkThatAllRefPicsAreAvailable(PicList &rcListPic, const ReferencePictureList *pRPL, int rplIdx,
                                           bool printErrors) const
{
  Picture *pic;
  int      isAvailable   = 0;
  int      notPresentPoc = 0;

  if (this->isIDRorBLA())
  {
    return 0;   // Assume that all pic in the DPB will be flushed anyway so no need to check.
  }

  int numberOfPictures = pRPL->getNumRefEntries();
  // Check long term ref pics
  for (int ii = 0; pRPL->m_numberOfLongtermPictures > 0 && ii < numberOfPictures; ii++)
  {
    if (!pRPL->m_isLongtermRefPic[ii] || pRPL->m_isInterLayerRefPic[ii])
    {
      continue;
    }

    notPresentPoc             = pRPL->m_refPicIdentifier[ii];
    isAvailable               = 0;
    PicList::iterator iterPic = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      pic          = *(iterPic++);
      int pocCycle = 1 << (pic->m_cs->sps->m_bitsForPoc);
      int curPoc   = pic->m_poc;
      int refPoc   = pRPL->m_refPicIdentifier[ii] & (pocCycle - 1);
      if (pRPL->m_deltaPocMSBPresentFlag[ii])
      {
        refPoc += m_poc - pRPL->m_deltaPOCMSBCycleLT[ii] * pocCycle - (m_poc & (pocCycle - 1));
      }
      else
      {
        curPoc = curPoc & (pocCycle - 1);
      }
      if (pic->m_longTerm && curPoc == refPoc && pic->m_referenced && pic->m_reconstructed &&
          pic->m_layerId == m_nuhLayerId)
      {
        isAvailable = 1;
        break;
      }
    }
    // if there was no such long-term check the short terms
    if (!isAvailable)
    {
      iterPic = rcListPic.begin();
      while (iterPic != rcListPic.end())
      {
        pic          = *(iterPic++);
        int pocCycle = 1 << (pic->m_cs->sps->m_bitsForPoc);
        int curPoc   = pic->m_poc;
        int refPoc   = pRPL->m_refPicIdentifier[ii] & (pocCycle - 1);
        if (pRPL->m_deltaPocMSBPresentFlag[ii])
        {
          refPoc += m_poc - pRPL->m_deltaPOCMSBCycleLT[ii] * pocCycle - (m_poc & (pocCycle - 1));
        }
        else
        {
          curPoc = curPoc & (pocCycle - 1);
        }
        if (!pic->m_longTerm && curPoc == refPoc && pic->m_referenced && pic->m_reconstructed &&
            pic->m_layerId == m_nuhLayerId)
        {
          isAvailable     = 1;
          pic->m_longTerm = true;
          break;
        }
      }
    }
    if (!isAvailable)
    {
      if (printErrors)
      {
        msg(ERROR,
            "Error: Current picture: %d Long-term reference picture with POC = %3d seems to have been removed or not "
            "correctly decoded.\n",
            this->m_poc, notPresentPoc);
      }
      return notPresentPoc;
    }
  }
  // report that a picture is lost if it is in the Reference Picture List but not in the DPB

  isAvailable = 0;
  // Check short term ref pics
  for (int ii = 0; ii < numberOfPictures; ii++)
  {
    if (pRPL->m_isLongtermRefPic[ii])
    {
      continue;
    }

    notPresentPoc             = this->m_poc + pRPL->m_refPicIdentifier[ii];
    isAvailable               = 0;
    PicList::iterator iterPic = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      pic = *(iterPic++);
      if (pic->m_poc == this->m_poc + pRPL->m_refPicIdentifier[ii] && pic->m_referenced &&
          pic->m_layerId == m_nuhLayerId)
      {
        isAvailable = 1;
        break;
      }
    }
    // report that a picture is lost if it is in the Reference Picture List but not in the DPB
    if (isAvailable == 0 && pRPL->m_numberOfShorttermPictures > 0)
    {
      if (printErrors)
      {
        msg(ERROR,
            "Error: Current picture: %d Short-term reference picture with POC = %3d seems to have been removed or not "
            "correctly decoded.\n",
            this->m_poc, notPresentPoc);
      }
      return notPresentPoc;
    }
  }
  return 0;
}

int Slice::checkThatAllRefPicsAreAvailable(PicList &rcListPic, const ReferencePictureList *pRPL, int rplIdx,
                                           bool printErrors, int *refPicIndex, int numActiveRefPics) const
{
  Picture *pic;
  int      isAvailable   = 0;
  int      notPresentPoc = 0;
  *refPicIndex           = 0;

  if (this->isIDRorBLA())
  {
    return 0;   // Assume that all pic in the DPB will be flushed anyway so no need to check.
  }

  int numberOfPictures = numActiveRefPics;
  // Check long term ref pics
  for (int ii = 0; pRPL->m_numberOfLongtermPictures > 0 && ii < numberOfPictures; ii++)
  {
    if (!pRPL->m_isLongtermRefPic[ii] || pRPL->m_isInterLayerRefPic[ii])
    {
      continue;
    }

    notPresentPoc             = pRPL->m_refPicIdentifier[ii];
    isAvailable               = 0;
    PicList::iterator iterPic = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      pic          = *(iterPic++);
      int pocCycle = 1 << (pic->m_cs->sps->m_bitsForPoc);
      int curPoc   = pic->m_poc;
      int refPoc   = pRPL->m_refPicIdentifier[ii] & (pocCycle - 1);
      if (pRPL->m_deltaPocMSBPresentFlag[ii])
      {
        refPoc += m_poc - pRPL->m_deltaPOCMSBCycleLT[ii] * pocCycle - (m_poc & (pocCycle - 1));
      }
      else
      {
        curPoc = curPoc & (pocCycle - 1);
      }
      if (pic->m_longTerm && curPoc == refPoc && pic->m_referenced && pic->m_reconstructed &&
          pic->m_layerId == m_nuhLayerId)
      {
        isAvailable = 1;
        break;
      }
    }
    // if there was no such long-term check the short terms
    if (!isAvailable)
    {
      iterPic = rcListPic.begin();
      while (iterPic != rcListPic.end())
      {
        pic          = *(iterPic++);
        int pocCycle = 1 << (pic->m_cs->sps->m_bitsForPoc);
        int curPoc   = pic->m_poc;
        int refPoc   = pRPL->m_refPicIdentifier[ii] & (pocCycle - 1);
        if (pRPL->m_deltaPocMSBPresentFlag[ii])
        {
          refPoc += m_poc - pRPL->m_deltaPOCMSBCycleLT[ii] * pocCycle - (m_poc & (pocCycle - 1));
        }
        else
        {
          curPoc = curPoc & (pocCycle - 1);
        }
        if (!pic->m_longTerm && curPoc == refPoc && pic->m_referenced && pic->m_reconstructed &&
            pic->m_layerId == m_nuhLayerId)
        {
          isAvailable     = 1;
          pic->m_longTerm = true;
          break;
        }
      }
    }
    if (!isAvailable)
    {
      if (printErrors)
      {
        msg(ERROR,
            "Error: Current picture: %d Long-term reference picture with POC = %3d seems to have been removed or not "
            "correctly decoded.\n",
            this->m_poc, notPresentPoc);
      }
      *refPicIndex = ii;
      return notPresentPoc;
    }
  }
  // report that a picture is lost if it is in the Reference Picture List but not in the DPB

  isAvailable = 0;
  // Check short term ref pics
  for (int ii = 0; ii < numberOfPictures; ii++)
  {
    if (pRPL->m_isLongtermRefPic[ii])
    {
      continue;
    }

    notPresentPoc             = this->m_poc + pRPL->m_refPicIdentifier[ii];
    isAvailable               = 0;
    PicList::iterator iterPic = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      pic = *(iterPic++);
      if (pic->m_poc == this->m_poc + pRPL->m_refPicIdentifier[ii] && pic->m_referenced &&
          pic->m_layerId == m_nuhLayerId)
      {
        isAvailable = 1;
        break;
      }
    }
    // report that a picture is lost if it is in the Reference Picture List but not in the DPB
    if (isAvailable == 0 && pRPL->m_numberOfShorttermPictures > 0)
    {
      if (printErrors)
      {
        msg(ERROR,
            "Error: Current picture: %d Short-term reference picture with POC = %3d seems to have been removed or not "
            "correctly decoded.\n",
            this->m_poc, notPresentPoc);
      }
      *refPicIndex = ii;
      return notPresentPoc;
    }
  }
  return 0;
}

bool Slice::isPOCInRefPicList(const ReferencePictureList *rpl, int poc)
{
  for (int i = 0; i < rpl->getNumRefEntries(); i++)
  {
    if (rpl->m_isInterLayerRefPic[i])
    {
      // Diagonal inter-layer prediction is not allowed
      CHECK(rpl->m_refPicIdentifier[i], "ILRP identifier should be 0");

      if (poc == m_poc)
      {
        return true;
      }
    }
    else if (rpl->m_isLongtermRefPic[i])
    {
      if (poc == rpl->m_refPicIdentifier[i])
      {
        return true;
      }
    }
    else
    {
      if (poc == m_poc + rpl->m_refPicIdentifier[i])
      {
        return true;
      }
    }
  }
  return false;
}

bool Slice::isPocRestrictedByDRAP(int poc, bool precedingDRAPInDecodingOrder)
{
  if (!m_enableDRAPSEI)
  {
    return false;
  }
  return (m_isDRAP && poc != m_iAssociatedIRAPPOC) ||
    (cvsHasPreviousDRAP() && m_poc > m_latestDRAPPOC && (precedingDRAPInDecodingOrder || poc < m_latestDRAPPOC));
}

bool Slice::isPocRestrictedByEdrap(int poc)
{
  if (!m_enableEdrapSEI)
  {
    return false;
  }
  return m_edrapRapId > 0 && poc != m_iAssociatedIRAPPOC;
}

void Slice::checkConformanceForDRAP(uint32_t temporalId)
{
  if (!(m_isDRAP || cvsHasPreviousDRAP()))
  {
    return;
  }

  if (m_isDRAP)
  {
    if (!(m_eNalUnitType == NalUnitType::NAL_UNIT_CODED_SLICE_TRAIL ||
          m_eNalUnitType == NalUnitType::NAL_UNIT_CODED_SLICE_STSA))
    {
      msg(WARNING, "Warning, non-conforming bitstream. The DRAP picture should be a trailing picture.\n");
    }
    if (temporalId != 0)
    {
      msg(
        WARNING,
        "Warning, non-conforming bitstream. The DRAP picture shall have a temporal sublayer identifier equal to 0.\n");
    }
    for (int i = 0; i < m_numRefIdx[RPL0]; i++)
    {
      if (getRefPic(RPL0, i)->m_poc != m_iAssociatedIRAPPOC)
      {
        msg(WARNING,
            "Warning, non-conforming bitstream. The DRAP picture shall not include any pictures in the active "
            "entries of its reference picture lists except the preceding IRAP picture in decoding order.\n");
      }
    }
    for (int i = 0; i < m_numRefIdx[RPL1]; i++)
    {
      if (getRefPic(RPL1, i)->m_poc != m_iAssociatedIRAPPOC)
      {
        msg(WARNING,
            "Warning, non-conforming bitstream. The DRAP picture shall not include any pictures in the active "
            "entries of its reference picture lists except the preceding IRAP picture in decoding order.\n");
      }
    }
  }

  if (cvsHasPreviousDRAP() && m_poc > m_latestDRAPPOC)
  {
    for (int i = 0; i < m_numRefIdx[RPL0]; i++)
    {
      if (getRefPic(RPL0, i)->m_poc < m_latestDRAPPOC && getRefPic(RPL0, i)->m_poc != m_iAssociatedIRAPPOC)
      {
        msg(WARNING,
            "Warning, non-conforming bitstream. Any picture that follows the DRAP picture in both decoding order "
            "and output order shall not include, in the active entries of its reference picture lists, any picture "
            "that precedes the DRAP picture in decoding order or output order, with the exception of the preceding "
            "IRAP picture in decoding order. Problem is POC %d in RPL0.\n",
            getRefPic(RPL0, i)->m_poc);
      }
    }
    for (int i = 0; i < m_numRefIdx[RPL1]; i++)
    {
      if (getRefPic(RPL1, i)->m_poc < m_latestDRAPPOC && getRefPic(RPL1, i)->m_poc != m_iAssociatedIRAPPOC)
      {
        msg(WARNING,
            "Warning, non-conforming bitstream. Any picture that follows the DRAP picture in both decoding order "
            "and output order shall not include, in the active entries of its reference picture lists, any picture "
            "that precedes the DRAP picture in decoding order or output order, with the exception of the preceding "
            "IRAP picture in decoding order. Problem is POC %d in RPL1",
            getRefPic(RPL1, i)->m_poc);
      }
    }
  }
}

void Slice::checkConformanceForEDRAP(uint32_t temporalId)
{
  if (!(m_edrapRapId > 0 || cvsHasPreviousEDRAP()))
  {
    return;
  }

  if (m_edrapRapId > 0)
  {
    if (!(m_eNalUnitType == NalUnitType::NAL_UNIT_CODED_SLICE_TRAIL ||
          m_eNalUnitType == NalUnitType::NAL_UNIT_CODED_SLICE_STSA))
    {
      msg(WARNING, "Warning, non-conforming bitstream. The EDRAP picture should be a trailing picture.\n");
    }
    if (temporalId != 0)
    {
      msg(
        WARNING,
        "Warning, non-conforming bitstream. The EDRAP picture shall have a temporal sublayer identifier equal to 0.\n");
    }
    for (int i = 0; i < m_numRefIdx[RPL0]; i++)
    {
      if (getRefPic(RPL0, i)->m_edrapRapId < 0)
      {
        msg(WARNING,
            "Warning, non-conforming bitstream. Any picture that is in the same layer and follows the EDRAP picture in "
            "both decoding order and output order does not include, in the active entries of its reference picture "
            "lists, any picture that is in the same layer and precedes the EDRAP picture in decoding order or output "
            "order, with the exception of the referenceablePictures.\n");
      }
    }
    for (int i = 0; i < m_numRefIdx[RPL1]; i++)
    {
      if (getRefPic(RPL1, i)->m_edrapRapId < 0)
      {
        msg(WARNING,
            "Warning, non-conforming bitstream. Any picture that is in the same layer and follows the EDRAP picture in "
            "both decoding order and output order does not include, in the active entries of its reference picture "
            "lists, any picture that is in the same layer and precedes the EDRAP picture in decoding order or output "
            "order, with the exception of the referenceablePictures.\n");
      }
    }
  }

  if (cvsHasPreviousEDRAP() && m_poc > m_latestEDRAPPOC && m_latestEdrapLeadingPicDecodableFlag)
  {
    for (int i = 0; i < m_numRefIdx[RPL0]; i++)
    {
      if (getRefPic(RPL0, i)->m_poc < m_latestEDRAPPOC && getRefPic(RPL0, i)->m_edrapRapId < 0)
      {
        msg(WARNING,
            "Warning, non-conforming bitstream. Any picture that is in the same layer and follows the EDRAP picture in "
            "decoding order and precedes the EDRAP picture in output order does not include, in the active entries of "
            "its reference picture lists, any picture that is in the same layer and precedes the EDRAP picture in "
            "decoding order, with the exception of the referenceablePictures. Problem is POC %d in RPL0.\n",
            getRefPic(RPL0, i)->m_poc);
      }
    }
    for (int i = 0; i < m_numRefIdx[RPL1]; i++)
    {
      if (getRefPic(RPL1, i)->m_poc < m_latestEDRAPPOC && getRefPic(RPL1, i)->m_edrapRapId < 0)
      {
        msg(WARNING,
            "Warning, non-conforming bitstream. Any picture that is in the same layer and follows the EDRAP picture in "
            "decoding order and precedes the EDRAP picture in output order does not include, in the active entries of "
            "its reference picture lists, any picture that is in the same layer and precedes the EDRAP picture in "
            "decoding order, with the exception of the referenceablePictures. Problem is POC %d in RPL1\n",
            getRefPic(RPL1, i)->m_poc);
      }
    }
  }
}

//! get AC and DC values for weighted pred
void Slice::getWpAcDcParam(const WPACDCParam *&wp) const { wp = m_weightACDCParam; }

//! init AC and DC values for weighted pred
void Slice::initWpAcDcParam()
{
  for (int iComp = 0; iComp < MAX_NUM_COMP; iComp++)
  {
    m_weightACDCParam[iComp].ac = 0;
    m_weightACDCParam[iComp].dc = 0;
  }
}

//! get tables for weighted prediction
const WPScalingParam *Slice::getWpScaling(const RefPicList refPicList, const int refIdx) const
{
  CHECK(refPicList >= NUM_RPL01, "Invalid picture reference list");
  return (refIdx < 0) ? nullptr : m_weightPredTable[refPicList][refIdx];
}

WPScalingParam *Slice::getWpScaling(const RefPicList refPicList, const int refIdx)
{
  CHECK(refPicList >= NUM_RPL01, "Invalid picture reference list");
  return (refIdx < 0) ? nullptr : m_weightPredTable[refPicList][refIdx];
}

//! reset Default WP tables settings : no weight.
void Slice::resetWpScaling()
{
  for (int e = 0; e < NUM_RPL01; e++)
  {
    for (int i = 0; i < MAX_NUM_REF; i++)
    {
      for (int yuv = 0; yuv < MAX_NUM_COMP; yuv++)
      {
        WPScalingParam *pwp  = &(m_weightPredTable[e][i][yuv]);
        pwp->presentFlag     = false;
        pwp->log2WeightDenom = 0;
        pwp->log2WeightDenom = 0;
        pwp->codedWeight     = 1;
        pwp->codedOffset     = 0;
      }
    }
  }
}

//! init WP table
void Slice::initWpScaling(const SPS *sps)
{
  const bool useHighPrecisionPredictionWeighting = sps->m_spsRangeExtension.m_highPrecisionOffsetsEnabledFlag;
  for (int e = 0; e < NUM_RPL01; e++)
  {
    for (int i = 0; i < MAX_NUM_REF; i++)
    {
      for (int yuv = 0; yuv < MAX_NUM_COMP; yuv++)
      {
        WPScalingParam *pwp = &(m_weightPredTable[e][i][yuv]);
        if (!pwp->presentFlag)
        {
          // Inferring values not present :
          pwp->codedWeight = (1 << pwp->log2WeightDenom);
          pwp->codedOffset = 0;
        }

        const int offsetScalingFactor =
          useHighPrecisionPredictionWeighting ? 1 : (1 << (sps->m_bitDepths[toChannelType(CompID(yuv))] - 8));

        pwp->w = pwp->codedWeight;
        pwp->o = pwp->codedOffset * offsetScalingFactor;   // NOTE: This value of the ".o" variable is never used - .o
                                                           // is set immediately before it gets used
        pwp->shift = pwp->log2WeightDenom;
        pwp->round = (pwp->log2WeightDenom >= 1) ? (1 << (pwp->log2WeightDenom - 1)) : (0);
      }
    }
  }
}

void Slice::startProcessingTimer() { m_iProcessingStartTime = clock(); }

void Slice::stopProcessingTimer()
{
  m_dProcessingTime += (double)(clock() - m_iProcessingStartTime) / CLOCKS_PER_SEC;
  m_iProcessingStartTime = 0;
}

unsigned Slice::getMinPictureDistance(unsigned ibcFastMethod) const
{
  int minPicDist = MAX_INT;
  if (m_ibcFlag)
  {
    minPicDist = 0;
  }
  if (!isIntra() && (!m_ibcFlag || (ibcFastMethod & IBC_FAST_METHOD_NONSCC)))
  {
    minPicDist        = MAX_INT;
    const int currPOC = m_poc;
    for (int refIdx = 0; refIdx < m_numRefIdx[RPL0]; refIdx++)
    {
      if (getRefPic(RPL0, refIdx)->m_layerId == m_nuhLayerId)
      {
        minPicDist = std::min(minPicDist, std::abs(currPOC - getRefPic(RPL0, refIdx)->m_poc));
      }
    }
    if (m_eSliceType == B_SLICE)
    {
      for (int refIdx = 0; refIdx < m_numRefIdx[RPL1]; refIdx++)
      {
        if (getRefPic(RPL1, refIdx)->m_layerId == m_nuhLayerId)
        {
          minPicDist = std::min(minPicDist, std::abs(currPOC - getRefPic(RPL1, refIdx)->m_poc));
        }
      }
    }
  }
  return (unsigned)minPicDist;
}

// ------------------------------------------------------------------------------------------------
// Picture Header
// ------------------------------------------------------------------------------------------------

PicHeader::PicHeader()
{
  m_saoEnabledFlag.fill(false);
  m_alfApsIdsLuma.resize(0);
  resetWpScaling();
}

PicHeader::~PicHeader() { m_alfApsIdsLuma.resize(0); }

/**
 - initialize picture header to defaut state
 */
void PicHeader::initPicHeader()
{
  m_valid                       = 0;
  m_nonReferencePictureFlag     = 0;
  m_gdrPicFlag                  = 0;
  m_recoveryPocCnt              = -1;
  m_spsId                       = -1;
  m_ppsId                       = -1;
  m_pocMsbPresentFlag           = 0;
  m_pocMsbVal                   = 0;
  m_picOutputFlag               = true;
  m_rplIdx[RPL0]                = 0;
  m_rplIdx[RPL1]                = 0;
  m_splitConsOverrideFlag       = 0;
  m_cuQpDeltaSubdivIntra        = 0;
  m_cuQpDeltaSubdivInter        = 0;
  m_cuChromaQpOffsetSubdivIntra = 0;
  m_cuChromaQpOffsetSubdivInter = 0;
  m_enableTMVPFlag              = true;
  m_picColFromL0Flag            = true;
  m_mvdL1ZeroFlag               = 0;
  m_maxNumAffineMergeCand       = AFFINE_MRG_MAX_NUM_CANDS;
  m_disFracMMVD                 = 0;
  m_bdofDisabledFlag            = 0;
  m_profDisabledFlag            = 0;
  m_jointCbCrSignFlag           = 0;
  m_qpDelta                     = 0;
#if ENABLE_NNLF
  m_nnlfDisabled = false;
#endif
  std::fill_n(m_newAlfFixFiltSetCandIdx, MAX_NUM_COMP, -1);
  m_numAlfApsIdsLuma                 = 0;
  m_alfApsIdChroma                   = 0;
  m_deblockingFilterOverrideFlag     = 0;
  m_deblockingFilterDisable          = 0;
  m_deblockingFilterBetaOffsetDiv2   = 0;
  m_deblockingFilterTcOffsetDiv2     = 0;
  m_deblockingFilterCbBetaOffsetDiv2 = 0;
  m_deblockingFilterCbTcOffsetDiv2   = 0;
  m_deblockingFilterCrBetaOffsetDiv2 = 0;
  m_deblockingFilterCrTcOffsetDiv2   = 0;
  m_lmcsEnabledFlag                  = 0;
  m_lmcsApsId                        = -1;
  m_lmcsAps                          = nullptr;
  m_lmcsChromaResidualScaleFlag      = 0;
  m_explicitScalingListEnabledFlag   = 0;
  m_scalingListApsId                 = -1;
  m_scalingListAps                   = nullptr;
  m_numWeights[RPL0]                 = 0;
  m_numWeights[RPL1]                 = 0;
  m_saoEnabledFlag.fill(false);
  memset(m_alfEnabledFlag, 0, sizeof(m_alfEnabledFlag));
  memset(m_minQT, 0, sizeof(m_minQT));
  memset(m_maxMTTHierarchyDepth, 0, sizeof(m_maxMTTHierarchyDepth));
  memset(m_maxBTSize, 0, sizeof(m_maxBTSize));
  memset(m_maxTTSize, 0, sizeof(m_maxTTSize));

  for (const auto l: { RPL0, RPL1 })
  {
    m_rpl[l].m_numberOfActivePictures    = 0;
    m_rpl[l].m_numberOfShorttermPictures = 0;
    m_rpl[l].m_numberOfLongtermPictures  = 0;
    m_rpl[l].m_ltrpInSliceHeaderFlag     = 0;
  }

  m_alfApsIdsLuma.resize(0);
}

const WPScalingParam *PicHeader::getWpScaling(const RefPicList refPicList, const int refIdx) const
{
  CHECK(refPicList >= NUM_RPL01, "Invalid picture reference list");
  if (refIdx < 0)
  {
    return nullptr;
  }
  else
  {
    return m_weightPredTable[refPicList][refIdx];
  }
}

WPScalingParam *PicHeader::getWpScaling(const RefPicList refPicList, const int refIdx)
{
  CHECK(refPicList >= NUM_RPL01, "Invalid picture reference list");
  if (refIdx < 0)
  {
    return nullptr;
  }
  else
  {
    return m_weightPredTable[refPicList][refIdx];
  }
}

void PicHeader::resetWpScaling()
{
  for (int e = 0; e < NUM_RPL01; e++)
  {
    for (int i = 0; i < MAX_NUM_REF; i++)
    {
      for (int yuv = 0; yuv < MAX_NUM_COMP; yuv++)
      {
        WPScalingParam *pwp  = &(m_weightPredTable[e][i][yuv]);
        pwp->presentFlag     = false;
        pwp->log2WeightDenom = 0;
        pwp->codedWeight     = 1;
        pwp->codedOffset     = 0;
      }
    }
  }
}

ScalingList::ScalingList()
{
  m_chromaScalingListPresentFlag = true;
  for (uint32_t scalingListId = 0; scalingListId < 28; scalingListId++)
  {
    int matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2
      : (scalingListId < SCALING_LIST_1D_START_8x8)              ? 4
                                                                 : 8;
    m_scalingListCoef[scalingListId].resize(matrixSize * matrixSize);
  }
}

/** set default quantization matrix to array
 */
void ScalingList::setDefaultScalingList()
{
  for (uint32_t scalingListId = 0; scalingListId < 28; scalingListId++)
  {
    processDefaultMatrix(scalingListId);
  }
}
/** check if use default quantization matrix
 * \returns true if the scaling list is not equal to the default quantization matrix
 */
bool ScalingList::isNotDefaultScalingList()
{
  bool isAllDefault = true;
  for (uint32_t scalingListId = 0; scalingListId < 28; scalingListId++)
  {
    int matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2
      : (scalingListId < SCALING_LIST_1D_START_8x8)              ? 4
                                                                 : 8;
    if (scalingListId < SCALING_LIST_1D_START_16x16)
    {
      if (::memcmp(getScalingListAddress(scalingListId), getScalingListDefaultAddress(scalingListId),
                   sizeof(int) * matrixSize * matrixSize))
      {
        isAllDefault = false;
        break;
      }
    }
    else
    {
      if ((::memcmp(getScalingListAddress(scalingListId), getScalingListDefaultAddress(scalingListId),
                    sizeof(int) * MAX_MATRIX_COEF_NUM)) ||
          (m_scalingListDC[scalingListId] != 16))
      {
        isAllDefault = false;
        break;
      }
    }
    if (!isAllDefault)
    {
      break;
    }
  }

  return !isAllDefault;
}

int ScalingList::lengthUvlc(int code)
{
  CHECK(code < 0, "Unsigned VLC cannot be negative");
  CHECK(code == MAX_INT, "Maximum supported UVLC code is MAX_INT-1");

  int length = 1;
  int temp   = ++code;

  while (1 != temp)
  {
    temp >>= 1;
    length += 2;
  }
  return (length >> 1) + ((length + 1) >> 1);
}

int ScalingList::lengthSvlc(int code)
{
  uint32_t code2  = uint32_t(code <= 0 ? (-code) << 1 : (code << 1) - 1);
  int      length = 1;
  int      temp   = ++code2;

  CHECK(temp < 0, "Integer overflow constructing SVLC code");

  while (1 != temp)
  {
    temp >>= 1;
    length += 2;
  }
  return (length >> 1) + ((length + 1) >> 1);
}

void ScalingList::codePredScalingList(int *scalingList, const int *scalingListPred, int scalingListDC,
                                      int scalingListPredDC, int scalingListId,
                                      int &bitsCost)   // sizeId, listId is current to-be-coded matrix idx
{
  int          deltaValue = 0;
  int          matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2
             : (scalingListId < SCALING_LIST_1D_START_8x8)              ? 4
                                                                        : 8;
  int          coefNum    = matrixSize * matrixSize;
  ScanElement *scan       = g_scanOrder[SCAN_UNGROUPED][CoeffScanType::DIAG][gp_sizeIdxInfo->idxFrom(matrixSize)]
                                 [gp_sizeIdxInfo->idxFrom(matrixSize)];
  int nextCoef = 0;

  int8_t     data;
  const int *src     = scalingList;
  const int *srcPred = scalingListPred;
  if (scalingListDC != -1 && scalingListPredDC != -1)
  {
    bitsCost += lengthSvlc((int8_t)(scalingListDC - scalingListPredDC - nextCoef));
    nextCoef = scalingListDC - scalingListPredDC;
  }
  else if ((scalingListDC != -1 && scalingListPredDC == -1))
  {
    bitsCost += lengthSvlc((int8_t)(scalingListDC - srcPred[scan[0].idx] - nextCoef));
    nextCoef = scalingListDC - srcPred[scan[0].idx];
  }
  else if ((scalingListDC == -1 && scalingListPredDC == -1)) {}
  else
  {
    printf("Predictor DC mismatch! \n");
  }
  for (int i = 0; i < coefNum; i++)
  {
    if (scalingListId >= SCALING_LIST_1D_START_64x64 && scan[i].x >= 4 && scan[i].y >= 4)
    {
      continue;
    }
    deltaValue = (src[scan[i].idx] - srcPred[scan[i].idx]);
    data       = (int8_t)(deltaValue - nextCoef);
    nextCoef   = deltaValue;

    bitsCost += lengthSvlc(data);
  }
}

void ScalingList::codeScalingList(int *scalingList, int scalingListDC, int scalingListId,
                                  int &bitsCost)   // sizeId, listId is current to-be-coded matrix idx
{
  int          matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2
             : (scalingListId < SCALING_LIST_1D_START_8x8)              ? 4
                                                                        : 8;
  int          coefNum    = matrixSize * matrixSize;
  ScanElement *scan       = g_scanOrder[SCAN_UNGROUPED][CoeffScanType::DIAG][gp_sizeIdxInfo->idxFrom(matrixSize)]
                                 [gp_sizeIdxInfo->idxFrom(matrixSize)];
  int        nextCoef = SCALING_LIST_START_VALUE;
  int8_t     data;
  const int *src = scalingList;

  if (scalingListId >= SCALING_LIST_1D_START_16x16)
  {
    bitsCost += lengthSvlc(int8_t(m_scalingListDC[scalingListId] - nextCoef));
    nextCoef = m_scalingListDC[scalingListId];
  }

  for (int i = 0; i < coefNum; i++)
  {
    if (scalingListId >= SCALING_LIST_1D_START_64x64 && scan[i].x >= 4 && scan[i].y >= 4)
    {
      continue;
    }
    data     = int8_t(src[scan[i].idx] - nextCoef);
    nextCoef = src[scan[i].idx];

    bitsCost += lengthSvlc(data);
  }
}
void ScalingList::CheckBestPredScalingList(int scalingListId, int predListId, int &BitsCount)
{
  // check previously coded matrix as a predictor, code "lengthUvlc" function
  int       *scalingList       = getScalingListAddress(scalingListId);
  const int *scalingListPred   = (scalingListId == predListId)
      ? ((predListId < SCALING_LIST_1D_START_8x8) ? g_quantTSDefault4x4 : g_quantIntraDefault8x8)
      : getScalingListAddress(predListId);
  int        scalingListDC     = (scalingListId >= SCALING_LIST_1D_START_16x16) ? m_scalingListDC[scalingListId] : -1;
  int        scalingListPredDC = (predListId >= SCALING_LIST_1D_START_16x16)
           ? ((scalingListId == predListId) ? 16 : m_scalingListDC[predListId])
           : -1;

  int bitsCost       = 0;
  int matrixSize     = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2
        : (scalingListId < SCALING_LIST_1D_START_8x8)              ? 4
                                                                   : 8;
  int predMatrixSize = (predListId < SCALING_LIST_1D_START_4x4) ? 2 : (predListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;

  CHECK(matrixSize != predMatrixSize, "Predictor size mismatch");

  bitsCost = 2 + lengthUvlc(scalingListId - predListId);
  // copy-flag + predictor-mode-flag + deltaListId
  codePredScalingList(scalingList, scalingListPred, scalingListDC, scalingListPredDC, scalingListId, bitsCost);
  BitsCount = bitsCost;
}

void ScalingList::processRefMatrix(uint32_t scalinListId, uint32_t refListId)
{
  int matrixSize = (scalinListId < SCALING_LIST_1D_START_4x4) ? 2 : (scalinListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;
  ::memcpy(getScalingListAddress(scalinListId),
           ((scalinListId == refListId) ? getScalingListDefaultAddress(refListId) : getScalingListAddress(refListId)),
           sizeof(int) * matrixSize * matrixSize);
}

void ScalingList::checkPredMode(uint32_t scalingListId)
{
  int bestBitsCount                            = MAX_INT;
  int bitsCount                                = 2;
  m_scalingListPreditorModeFlag[scalingListId] = false;
  codeScalingList(getScalingListAddress(scalingListId),
                  ((scalingListId >= SCALING_LIST_1D_START_16x16) ? m_scalingListDC[scalingListId] : -1), scalingListId,
                  bitsCount);
  bestBitsCount = bitsCount;

  for (int predListIdx = (int)scalingListId; predListIdx >= 0; predListIdx--)
  {

    int matrixSize     = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2
          : (scalingListId < SCALING_LIST_1D_START_8x8)              ? 4
                                                                     : 8;
    int predMatrixSize = (predListIdx < SCALING_LIST_1D_START_4x4) ? 2
      : (predListIdx < SCALING_LIST_1D_START_8x8)                  ? 4
                                                                   : 8;
    if (((scalingListId == SCALING_LIST_1D_START_2x2 || scalingListId == SCALING_LIST_1D_START_4x4 ||
          scalingListId == SCALING_LIST_1D_START_8x8) &&
         predListIdx != (int)scalingListId) ||
        matrixSize != predMatrixSize)
    {
      continue;
    }
    const int *refScalingList =
      (scalingListId == predListIdx) ? getScalingListDefaultAddress(predListIdx) : getScalingListAddress(predListIdx);
    const int refDC = (predListIdx < SCALING_LIST_1D_START_16x16) ? refScalingList[0]
      : (scalingListId == predListIdx)                            ? 16
                                                                  : m_scalingListDC[predListIdx];
    if (!::memcmp(getScalingListAddress(scalingListId), refScalingList,
                  sizeof(int) * matrixSize * matrixSize)   // check value of matrix
                                                           // check DC value
        && (scalingListId < SCALING_LIST_1D_START_16x16 || m_scalingListDC[scalingListId] == refDC))
    {
      // copy mode
      m_refMatrixId[scalingListId]                   = predListIdx;
      m_scalingListPredModeFlagIsCopy[scalingListId] = true;
      m_scalingListPreditorModeFlag[scalingListId]   = false;
      return;
    }
    else
    {
      // predictor mode
      // use previously coded matrix as a predictor
      CheckBestPredScalingList(scalingListId, predListIdx, bitsCount);
      if (bitsCount < bestBitsCount)
      {
        bestBitsCount                                  = bitsCount;
        m_scalingListPredModeFlagIsCopy[scalingListId] = false;
        m_scalingListPreditorModeFlag[scalingListId]   = true;
        m_refMatrixId[scalingListId]                   = predListIdx;
      }
    }
  }
  m_scalingListPredModeFlagIsCopy[scalingListId] = false;
}

static void outputScalingListHelp(std::ostream &os)
{
  os << "The scaling list file specifies all matrices and their DC values; none can be missing,\n"
        "but their order is arbitrary.\n\n"
        "The matrices are specified by:\n"
        "<matrix name><unchecked data>\n"
        "  <value>,<value>,<value>,....\n\n"
        "  Line-feeds can be added arbitrarily between values, and the number of values needs to be\n"
        "  at least the number of entries for the matrix (superfluous entries are ignored).\n"
        "  The <unchecked data> is text on the same line as the matrix that is not checked\n"
        "  except to ensure that the matrix name token is unique. It is recommended that it is ' ='\n"
        "  The values in the matrices are the absolute values (0-255), not the delta values as\n"
        "  exchanged between the encoder and decoder\n\n"
        "The DC values (for matrix sizes larger than 8x8) are specified by:\n"
        "<matrix name>_DC<unchecked data>\n"
        "  <value>\n";

  os << "The permitted matrix names are:\n";
  for (uint32_t sizeIdc = SCALING_LIST_2x2; sizeIdc <= SCALING_LIST_64x64; sizeIdc++)
  {
    for (uint32_t listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++)
    {
      if (!(((sizeIdc == SCALING_LIST_64x64) && (listIdc % (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES) != 0)) ||
            ((sizeIdc == SCALING_LIST_2x2) && (listIdc % (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES) == 0))))
      {
        os << "  " << matrixType[sizeIdc][listIdc] << '\n';
      }
    }
  }
}

void ScalingList::outputScalingLists(std::ostream &os) const
{
  int scalingListId = 0;
  for (uint32_t sizeIdc = SCALING_LIST_2x2; sizeIdc <= SCALING_LIST_64x64; sizeIdc++)
  {
    const uint32_t size = (sizeIdc == 1) ? 2 : ((sizeIdc == 2) ? 4 : 8);
    for (uint32_t listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++)
    {
      if (!((sizeIdc == SCALING_LIST_64x64 && listIdc % (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES) != 0) ||
            (sizeIdc == SCALING_LIST_2x2 && listIdc < 4)))
      {
        const int *src = getScalingListAddress(scalingListId);
        os << (matrixType[sizeIdc][listIdc]) << " =\n  ";
        for (uint32_t y = 0; y < size; y++)
        {
          for (uint32_t x = 0; x < size; x++, src++)
          {
            os << std::setw(3) << (*src) << ", ";
          }
          os << (y + 1 < size ? "\n  " : "\n");
        }
        if (sizeIdc > SCALING_LIST_8x8)
        {
          os << matrixTypeDc[sizeIdc][listIdc] << " = \n  " << std::setw(3) << m_scalingListDC[scalingListId] << "\n";
        }
        os << "\n";
        scalingListId++;
      }
    }
  }
}

bool ScalingList::xParseScalingList(const std::string &fileName)
{
  static const int LINE_SIZE = 1024;

  FILE *fp = nullptr;

  char line[LINE_SIZE];

  if (fileName.empty())
  {
    msg(ERROR, "Error: no scaling list file specified. Help on scaling lists being output\n");
    outputScalingListHelp(std::cout);
    std::cout << "\n\nExample scaling list file using default values:\n\n";
    outputScalingLists(std::cout);
    return true;
  }
  else if ((fp = fopen(fileName.c_str(), "r")) == nullptr)
  {
    msg(ERROR, "Error: cannot open scaling list file %s for reading\n", fileName.c_str());
    return true;
  }

  int scalingListId = 0;
  for (uint32_t sizeIdc = SCALING_LIST_2x2; sizeIdc <= SCALING_LIST_64x64; sizeIdc++)   // 2x2-128x128
  {
    const uint32_t size = std::min(MAX_MATRIX_COEF_NUM, (int)g_scalingListSize[sizeIdc]);

    for (uint32_t listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++)
    {

      if ((sizeIdc == SCALING_LIST_64x64 && listIdc % (SCALING_LIST_NUM / SCALING_LIST_PRED_MODES) != 0) ||
          (sizeIdc == SCALING_LIST_2x2 && listIdc < 4))
      {
        continue;
      }
      else
      {
        int *const src = getScalingListAddress(scalingListId);
        {
          fseek(fp, 0, SEEK_SET);
          bool found = false;
          while ((!feof(fp)) && (!found))
          {
            char *ret              = fgets(line, LINE_SIZE, fp);
            char *findNamePosition = ret == nullptr ? nullptr : strstr(line, matrixType[sizeIdc][listIdc]);
            // This could be a match against the DC string as well, so verify it isn't
            if (findNamePosition != nullptr &&
                (matrixTypeDc[sizeIdc][listIdc] == nullptr || strstr(line, matrixTypeDc[sizeIdc][listIdc]) == nullptr))
            {
              found = true;
            }
          }
          if (!found)
          {
            msg(ERROR, "Error: cannot find Matrix %s from scaling list file %s\n", matrixType[sizeIdc][listIdc],
                fileName.c_str());
            return true;
          }
        }
        for (uint32_t i = 0; i < size; i++)
        {
          int data;
          if (fscanf(fp, "%d,", &data) != 1)
          {
            msg(ERROR, "Error: cannot read value #%d for Matrix %s from scaling list file %s at file position %ld\n", i,
                matrixType[sizeIdc][listIdc], fileName.c_str(), ftell(fp));
            return true;
          }
          if (data < 0 || data > 255)
          {
            msg(ERROR,
                "Error: QMatrix entry #%d of value %d for Matrix %s from scaling list file %s at file position %ld is "
                "out of range (0 to 255)\n",
                i, data, matrixType[sizeIdc][listIdc], fileName.c_str(), ftell(fp));
            return true;
          }
          src[i] = data;
        }

        // set DC value for default matrix check
        m_scalingListDC[scalingListId] = src[0];

        if (sizeIdc > SCALING_LIST_8x8)
        {
          {
            fseek(fp, 0, SEEK_SET);
            bool found = false;
            while ((!feof(fp)) && (!found))
            {
              char *ret              = fgets(line, LINE_SIZE, fp);
              char *findNamePosition = ret == nullptr ? nullptr : strstr(line, matrixTypeDc[sizeIdc][listIdc]);
              if (findNamePosition != nullptr)
              {
                // This won't be a match against the non-DC string.
                found = true;
              }
            }
            if (!found)
            {
              msg(ERROR, "Error: cannot find DC Matrix %s from scaling list file %s\n", matrixTypeDc[sizeIdc][listIdc],
                  fileName.c_str());
              return true;
            }
          }
          int data;
          if (fscanf(fp, "%d,", &data) != 1)
          {
            msg(ERROR, "Error: cannot read DC %s from scaling list file %s at file position %ld\n",
                matrixTypeDc[sizeIdc][listIdc], fileName.c_str(), ftell(fp));
            return true;
          }
          if (data < 0 || data > 255)
          {
            msg(ERROR,
                "Error: DC value %d for Matrix %s from scaling list file %s at file position %ld is out of range (0 to "
                "255)\n",
                data, matrixType[sizeIdc][listIdc], fileName.c_str(), ftell(fp));
            return true;
          }
          // overwrite DC value when size of matrix is larger than 16x16
          m_scalingListDC[scalingListId] = data;
        }
      }
      scalingListId++;
    }
  }
  //  std::cout << "\n\nRead scaling lists of:\n\n";
  //  outputScalingLists(std::cout);

  fclose(fp);
  return false;
}

/** get default address of quantization matrix
 * \param sizeId size index
 * \param listId list index
 * \returns pointer of quantization matrix
 */
const int *ScalingList::getScalingListDefaultAddress(uint32_t scalingListId)
{
  const int *src    = 0;
  int        sizeId = (scalingListId < SCALING_LIST_1D_START_8x8) ? 2 : 3;
  switch (sizeId)
  {
  case SCALING_LIST_1x1:
  case SCALING_LIST_2x2:
  case SCALING_LIST_4x4:
    src = g_quantTSDefault4x4;
    break;
  case SCALING_LIST_8x8:
  case SCALING_LIST_16x16:
  case SCALING_LIST_32x32:
  case SCALING_LIST_64x64:
  case SCALING_LIST_128x128:
    src = g_quantInterDefault8x8;
    break;
  default:
    THROW("Invalid scaling list");
    src = nullptr;
    break;
  }
  return src;
}

/** process of default matrix
 * \param sizeId size index
 * \param listId index of input matrix
 */
void ScalingList::processDefaultMatrix(uint32_t scalingListId)
{
  int matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2
    : (scalingListId < SCALING_LIST_1D_START_8x8)              ? 4
                                                               : 8;
  ::memcpy(getScalingListAddress(scalingListId), getScalingListDefaultAddress(scalingListId),
           sizeof(int) * matrixSize * matrixSize);
  m_scalingListDC[scalingListId] = SCALING_LIST_DC;
}

/** check DC value of matrix for default matrix signaling
 */
void ScalingList::checkDcOfMatrix()
{
  for (uint32_t scalingListId = 0; scalingListId < 28; scalingListId++)
  {
    // check default matrix?
    if (m_scalingListDC[scalingListId] == 0)
    {
      processDefaultMatrix(scalingListId);
    }
  }
}

bool ScalingList::isLumaScalingList(int scalingListId) const
{
  return (scalingListId % MAX_NUM_COMP == SCALING_LIST_1D_START_4x4 ||
          scalingListId == SCALING_LIST_1D_START_64x64 + 1);
}

uint32_t PreCalcValues::getValIdx(const Slice &slice, const ChannelType chType) const
{
  return slice.isIntra() ? (ISingleTree || isLuma(chType) ? 0 : 2) : 1;
}

uint32_t PreCalcValues::getMaxBtDepth(const Slice &slice, const ChannelType chType) const
{
  if (slice.m_picHeader->m_splitConsOverrideFlag)
  {
    return slice.m_picHeader->getMaxMTTHierarchyDepth(slice.m_eSliceType, ISingleTree ? ChannelType::LUMA : chType);
  }
  else
  {
    return maxBtDepth[getValIdx(slice, chType)];
  }
}

uint32_t PreCalcValues::getMinBtSize(const Slice &slice, const ChannelType chType) const
{
  return minBtSize[getValIdx(slice, chType)];
}

uint32_t PreCalcValues::getMaxBtSize(const Slice &slice, const ChannelType chType) const
{
  if (slice.m_picHeader->m_splitConsOverrideFlag)
  {
    return slice.m_picHeader->getMaxBTSize(slice.m_eSliceType, ISingleTree ? ChannelType::LUMA : chType);
  }
  else
  {
    return maxBtSize[getValIdx(slice, chType)];
  }
}

uint32_t PreCalcValues::getMinTtSize(const Slice &slice, const ChannelType chType) const
{
  return minTtSize[getValIdx(slice, chType)];
}

uint32_t PreCalcValues::getMaxTtSize(const Slice &slice, const ChannelType chType) const
{
  if (slice.m_picHeader->m_splitConsOverrideFlag)
  {
    return slice.m_picHeader->getMaxTTSize(slice.m_eSliceType, ISingleTree ? ChannelType::LUMA : chType);
  }
  else
  {
    return maxTtSize[getValIdx(slice, chType)];
  }
}
uint32_t PreCalcValues::getMinQtSize(const Slice &slice, const ChannelType chType) const
{
  if (slice.m_picHeader->m_splitConsOverrideFlag)
  {
    return slice.m_picHeader->getMinQTSize(slice.m_eSliceType, ISingleTree ? ChannelType::LUMA : chType);
  }
  else
  {
    return minQtSize[getValIdx(slice, chType)];
  }
}

bool Slice::scaleRefPicList(Picture *scaledRefPic[], PicHeader *picHeader, APS **apss, APS *lmcsAps,
                            APS *scalingListAps, const bool isDecoder)
{
  int        i;
  const SPS *sps = m_sps;
  const PPS *pps = m_pps;

  bool refPicIsSameRes = false;

  // this is needed for IBC
  m_pic->m_unscaledPic = m_pic;

  if (m_eSliceType == I_SLICE)
  {
    return false;
  }

  freeScaledRefPicList(scaledRefPic);

  for (int refList = 0; refList < NUM_RPL01; refList++)
  {
    if (refList == 1 && m_eSliceType != B_SLICE)
    {
      continue;
    }

    for (int rIdx = 0; rIdx < m_numRefIdx[refList]; rIdx++)
    {
      // if rescaling is needed, otherwise just reuse the original picture pointer; it is needed for motion field,
      // otherwise motion field requires a copy as well reference resampling for the whole picture is not applied at
      // decoder

      CU::getRprScaling(sps, pps, m_refPicList[refList][rIdx], m_scalingRatio[refList][rIdx]);

      CHECK(m_refPicList[refList][rIdx]->m_unscaledPic == nullptr, "m_unscaledPic is not properly set");

      if (m_refPicList[refList][rIdx]->isRefScaled(pps) == false)
      {
        refPicIsSameRes = true;
      }

      if (m_scalingRatio[refList][rIdx] == SCALE_1X || isDecoder)
      {
        m_scaledRefPicList[refList][rIdx] = m_refPicList[refList][rIdx];
      }
      else
      {
        int poc     = m_refPicList[refList][rIdx]->m_poc;
        int layerId = m_refPicList[refList][rIdx]->m_layerId;

        // check whether the reference picture has already been scaled
        for (i = 0; i < MAX_NUM_REF; i++)
        {
          if (scaledRefPic[i] != nullptr && scaledRefPic[i]->m_poc == poc && scaledRefPic[i]->m_layerId == layerId)
          {
            break;
          }
        }

        if (i == MAX_NUM_REF)
        {
          int j;
          // search for unused Picture structure in scaledRefPic
          for (j = 0; j < MAX_NUM_REF; j++)
          {
            if (scaledRefPic[j] == nullptr)
            {
              break;
            }
          }

          CHECK(j >= MAX_NUM_REF, "scaledRefPic can not hold all reference pictures!");

          if (j >= MAX_NUM_REF)
          {
            j = 0;
          }

          if (scaledRefPic[j] == nullptr)
          {
            scaledRefPic[j] = new Picture;

            scaledRefPic[j]->m_extendedBorder = false;
            scaledRefPic[j]->m_reconstructed  = false;
            scaledRefPic[j]->m_referenced     = true;

            scaledRefPic[j]->finalInit(m_pic->m_cs->vps, *sps, *pps, picHeader, apss, lmcsAps, scalingListAps);

            scaledRefPic[j]->m_poc = NOT_VALID;

#if !ENABLE_POST_CFE_CHANGES
            scaledRefPic[j]->create(
              sps->m_chromaFormatIdc, Size(pps->m_picWidthInLumaSamples, pps->m_picHeightInLumaSamples),
              sps->m_maxCuWidth, sps->m_maxCuWidth + 16, isDecoder, layerId, sps->m_rprEnabledFlag, false, false
#else
            scaledRefPic[j]->create(sps->m_chromaFormatIdc,
                                    Size(pps->m_picWidthInLumaSamples, pps->m_picHeightInLumaSamples),
                                    sps->m_maxCuWidth, sps->m_maxCuWidth + EXT_PICTURE_SIZE, isDecoder, layerId,
                                    sps->m_rprEnabledFlag, false, false
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
          }

          scaledRefPic[j]->m_poc      = poc;
          scaledRefPic[j]->m_longTerm = m_refPicList[refList][rIdx]->m_longTerm;

          // rescale the reference picture
          const bool downsampling =
            m_refPicList[refList][rIdx]->getRecoBuf().Y().width >= scaledRefPic[j]->getRecoBuf().Y().width &&
            m_refPicList[refList][rIdx]->getRecoBuf().Y().height >= scaledRefPic[j]->getRecoBuf().Y().height;
          Picture::rescalePicture(m_scalingRatio[refList][rIdx], m_refPicList[refList][rIdx]->getRecoBuf(),
                                  m_refPicList[refList][rIdx]->m_slices[0]->m_pps->m_scalingWindow,
                                  scaledRefPic[j]->getRecoBuf(), pps->m_scalingWindow, sps->m_chromaFormatIdc,
                                  sps->m_bitDepths, true, downsampling, sps->m_horCollocatedChromaFlag,
                                  sps->m_verCollocatedChromaFlag);
          scaledRefPic[j]->m_unscaledPic = m_refPicList[refList][rIdx];
          scaledRefPic[j]->extendPicBorder(m_pps);

          m_scaledRefPicList[refList][rIdx] = scaledRefPic[j];
        }
        else
        {
          m_scaledRefPicList[refList][rIdx] = scaledRefPic[i];
        }
      }
    }
  }

  // make the scaled reference picture list as the default reference picture list
  for (int refList = 0; refList < NUM_RPL01; refList++)
  {
    if (refList == 1 && m_eSliceType != B_SLICE)
    {
      continue;
    }

    for (int rIdx = 0; rIdx < m_numRefIdx[refList]; rIdx++)
    {
      m_savedRefPicList[refList][rIdx] = m_refPicList[refList][rIdx];
      m_refPicList[refList][rIdx]      = m_scaledRefPicList[refList][rIdx];

      // allow the access of the unscaled version in xPredInterBlk()
      m_refPicList[refList][rIdx]->m_unscaledPic = m_savedRefPicList[refList][rIdx];
    }
  }

  // Make sure that TMVP is disabled when there are no reference pictures with the same resolution
  return (!refPicIsSameRes);
}

void Slice::freeScaledRefPicList(Picture *scaledRefPic[])
{
  if (m_eSliceType == I_SLICE)
  {
    return;
  }
  for (int i = 0; i < MAX_NUM_REF; i++)
  {
    if (scaledRefPic[i] != nullptr)
    {
      scaledRefPic[i]->destroy();
      delete scaledRefPic[i];
      scaledRefPic[i] = nullptr;
    }
  }
}

bool Slice::checkRPR()
{
  const PPS *pps = m_pps;

  for (int refList = 0; refList < NUM_RPL01; refList++)
  {

    if (refList == 1 && m_eSliceType != B_SLICE)
    {
      continue;
    }

    for (int rIdx = 0; rIdx < m_numRefIdx[refList]; rIdx++)
    {
      if (m_scaledRefPicList[refList][rIdx]->m_cs->pcv->lumaWidth != pps->m_picWidthInLumaSamples ||
          m_scaledRefPicList[refList][rIdx]->m_cs->pcv->lumaHeight != pps->m_picHeightInLumaSamples)
      {
        return true;
      }
    }
  }

  return false;
}

bool Slice::isLastSliceInSubpic()
{
  CHECK(m_pps == nullptr, "PPS pointer not initialized");

  int lastCTUAddrInSlice = m_sliceMap.m_ctuAddrInSlice.back();

  if (m_pps->m_numSubPics > 1)
  {
    const SubPic &subpic = m_pps->m_subPics[m_pps->getSubPicIdxFromSubPicId(m_sliceSubPicId)];
    return subpic.m_lastCtuInSubPic == lastCTUAddrInSlice;
  }
  else
  {
    const CodingStructure *cs            = m_pic->m_cs;
    const PreCalcValues   *pcv           = cs->pcv;
    const uint32_t         picSizeInCtus = pcv->heightInCtus * pcv->widthInCtus;
    return lastCTUAddrInSlice == (picSizeInCtus - 1);
  }
}

bool Slice::checkAlfAPS(const int apsId)
{
  for (int id = 0; id < m_numAlfApsIdsLuma; id++)
  {
    if (apsId == m_alfApsIdsLuma[id])
    {
      return true;
    }
  }
  if (apsId == m_alfApsIdChroma)
  {
    return true;
  }
  return false;
}

void Slice::lfCccmClearControlInformation(const int ctuRsAddr)
{
  if (ctuRsAddr > -1)
  {
    m_lfCccmEnabled.at(ctuRsAddr)         = 0;
    m_lfCccmWindowSizeIndex.at(ctuRsAddr) = 0;
    m_lfCccmModelType.at(ctuRsAddr)       = 0;
    m_lfCccmCTUMerge.at(ctuRsAddr)        = 0;
    return;
  }
  m_lfCccmEnabled.resize(m_pic->m_ctuNums, 0);
  m_lfCccmWindowSizeIndex.resize(m_pic->m_ctuNums, 0);
  m_lfCccmModelType.resize(m_pic->m_ctuNums, 0);
  m_lfCccmCTUMerge.resize(m_pic->m_ctuNums, 0);
  m_lfCccmFrameLevelInherit = 0;
}

const Picture *Slice::lfCccmGetReferencePicture() const
{
  int            bestDist = MAX_INT;
  const Picture *refPic   = nullptr;
  const int      curPoc   = m_poc;

  auto getLfCccmRefPic = [&](const RefPicList refPicList)
  {
    for (int refIdxInList = 0; refIdxInList < m_numRefIdx[refPicList]; refIdxInList++)
    {
      const int      refPoc      = getRefPOC(refPicList, refIdxInList);
      const Picture *refPicTemp  = getRefPic(refPicList, refIdxInList);
      const int      cDist       = abs(refPoc - curPoc);
      bool           isRefScaled = false;
      if (refPicTemp)
      {
        isRefScaled = refPicTemp->isRefScaled(m_pps);
      }
      if (refPicTemp && !isRefScaled && refPicTemp->m_cs && refPicTemp->m_cs->slice && cDist < bestDist &&
          refPicTemp->m_cs->slice->m_lfCccmEnabledFlag)
      {
        bestDist = cDist;
        refPic   = refPicTemp;
        return;
      }
    }
  };

  getLfCccmRefPic(RPL0);
  getLfCccmRefPic(RPL1);

  return refPic;
}
lfCccmCand Slice::lfCccmGetCandidate(const int ctuRsAddr) const
{
  lfCccmCand tmpCand;
  tmpCand.windowSize = m_lfCccmWindowSizeIndex.at(ctuRsAddr);
  tmpCand.modelType  = m_lfCccmModelType.at(ctuRsAddr);
  return tmpCand;
}
std::vector<lfCccmCand> Slice::lfCccmGetMergeCandidates(const int ctuRsAddr) const
{
  std::vector<lfCccmCand> candidates;

  const int ctuX = ctuRsAddr % m_pic->m_cs->pcv->widthInCtus;
  const int ctuY = ctuRsAddr / m_pic->m_cs->pcv->widthInCtus;

  if (ctuX && m_lfCccmEnabled.at(ctuRsAddr - 1))
  {
    candidates.push_back(lfCccmGetCandidate(ctuRsAddr - 1));
  }

  if (ctuY && m_lfCccmEnabled.at(ctuRsAddr - m_pic->m_cs->pcv->widthInCtus))
  {
    candidates.push_back(lfCccmGetCandidate(ctuRsAddr - m_pic->m_cs->pcv->widthInCtus));
  }

  if (candidates.size() > lfCccmMaxNumCands)
  {
    return std::vector<lfCccmCand>(candidates.begin(), candidates.begin() + lfCccmMaxNumCands);
  }

  return candidates;
}
void Slice::lfCccmMerge(const int ctuRsAddr)
{
  const std::vector<lfCccmCand> candidates = lfCccmGetMergeCandidates(ctuRsAddr);
  CHECK(candidates.size() == 0, "error empty candidates")
  CHECK(candidates.size() > lfCccmMaxNumCands, "too many candidates")
  const int8_t     candIdx              = !!(m_lfCccmCTUMerge.at(ctuRsAddr) & 2);
  const lfCccmCand curCand              = candidates.at(candIdx);
  m_lfCccmEnabled.at(ctuRsAddr)         = 1;
  m_lfCccmWindowSizeIndex.at(ctuRsAddr) = curCand.windowSize;
  m_lfCccmModelType.at(ctuRsAddr)       = curCand.modelType;
}

#if ENABLE_TRACING
void xTraceVPSHeader() { DTRACE(g_trace_ctx, D_HEADER, "=========== Video Parameter Set     ===========\n"); }

void xTraceOPIHeader() { DTRACE(g_trace_ctx, D_HEADER, "=========== Operating Point Information     ===========\n"); }

void xTraceDCIHeader()
{
  DTRACE(g_trace_ctx, D_HEADER, "=========== Decoding Capability Information     ===========\n");
}

void xTraceSPSHeader() { DTRACE(g_trace_ctx, D_HEADER, "=========== Sequence Parameter Set  ===========\n"); }

void xTracePPSHeader() { DTRACE(g_trace_ctx, D_HEADER, "=========== Picture Parameter Set  ===========\n"); }

void xTraceAPSHeader() { DTRACE(g_trace_ctx, D_HEADER, "=========== Adaptation Parameter Set  ===========\n"); }

void xTracePictureHeader() { DTRACE(g_trace_ctx, D_HEADER, "=========== Picture Header ===========\n"); }

void xTraceSliceHeader() { DTRACE(g_trace_ctx, D_HEADER, "=========== Slice ===========\n"); }

void xTraceAccessUnitDelimiter() { DTRACE(g_trace_ctx, D_HEADER, "=========== Access Unit Delimiter ===========\n"); }

void xTraceFillerData() { DTRACE(g_trace_ctx, D_HEADER, "=========== Filler Data ===========\n"); }
#endif
