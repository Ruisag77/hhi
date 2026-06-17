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

/** \file     EncCfg.cpp
    \brief
*/

#include "EncCfg.h"

//! \ingroup EncoderLib
//! \{

// auto determine the profile to use given the other configuration settings. Returns 1 if erred. Can select profile
// 'NONE'
int autoDetermineProfile(EncCfg *encCfg)
{
  const int maxBitDepth = std::max(encCfg->m_internalBitDepth[ChannelType::LUMA],
                                   encCfg->m_internalBitDepth[getLastChannel(encCfg->m_chromaFormatIdc)]);
  encCfg->m_profile     = Profile::NONE;

  switch (encCfg->m_chromaFormatIdc)
  {
  case ChromaFormat::_400:
  case ChromaFormat::_420:
    if (maxBitDepth <= 10)
    {
      if (encCfg->m_level == Level::LEVEL15_5 && encCfg->m_framesToBeEncoded == 1)
      {
        encCfg->m_profile =
          encCfg->m_maxLayers > 1 ? Profile::MULTILAYER_MAIN_10_STILL_PICTURE : Profile::MAIN_10_STILL_PICTURE;
      }
      else
      {
        encCfg->m_profile = encCfg->m_maxLayers > 1 ? Profile::MULTILAYER_MAIN_10 : Profile::MAIN_10;
      }
    }
    else if (maxBitDepth <= 12)
    {
      encCfg->m_profile = (encCfg->m_level == Level::LEVEL15_5 && encCfg->m_framesToBeEncoded == 1)
        ? Profile::MAIN_12_STILL_PICTURE
        : (encCfg->m_intraPeriod == 1) ? Profile::MAIN_12_INTRA
                                       : Profile::MAIN_12;
    }
    else if (maxBitDepth <= 16)
    {
      // Since there's no 16bit 420 profiles in VVC, we use 444 profiles.
      encCfg->m_profile = (encCfg->m_level == Level::LEVEL15_5 && encCfg->m_framesToBeEncoded == 1)
        ? Profile::MAIN_16_444_STILL_PICTURE
        : (encCfg->m_intraPeriod == 1) ? Profile::MAIN_16_444_INTRA
                                       : Profile::MAIN_16_444;
    }
    break;

  case ChromaFormat::_422:
  case ChromaFormat::_444:
    if (maxBitDepth <= 10)
    {
      if (encCfg->m_level == Level::LEVEL15_5 && encCfg->m_framesToBeEncoded == 1)
      {
        encCfg->m_profile =
          encCfg->m_maxLayers > 1 ? Profile::MULTILAYER_MAIN_10_444_STILL_PICTURE : Profile::MAIN_10_444_STILL_PICTURE;
      }
      else
      {
        encCfg->m_profile = encCfg->m_maxLayers > 1 ? Profile::MULTILAYER_MAIN_10_444 : Profile::MAIN_10_444;
      }
    }
    else if (maxBitDepth <= 12)
    {
      encCfg->m_profile = (encCfg->m_level == Level::LEVEL15_5 && encCfg->m_framesToBeEncoded == 1)
        ? Profile::MAIN_12_444_STILL_PICTURE
        : (encCfg->m_intraPeriod == 1) ? Profile::MAIN_12_444_INTRA
                                       : Profile::MAIN_12_444;
    }
    else if (maxBitDepth <= 16)
    {
      encCfg->m_profile = (encCfg->m_level == Level::LEVEL15_5 && encCfg->m_framesToBeEncoded == 1)
        ? Profile::MAIN_16_444_STILL_PICTURE
        : (encCfg->m_intraPeriod == 1) ? Profile::MAIN_16_444_INTRA
                                       : Profile::MAIN_16_444;
    }
    break;

  default:
    return 1;
  }
  if (encCfg->m_profile == Profile::MAIN_12_INTRA || encCfg->m_profile == Profile::MAIN_12_444_INTRA ||
      encCfg->m_profile == Profile::MAIN_16_444_INTRA || encCfg->m_profile == Profile::MAIN_12_STILL_PICTURE ||
      encCfg->m_profile == Profile::MAIN_12_444_STILL_PICTURE ||
      encCfg->m_profile == Profile::MAIN_16_444_STILL_PICTURE)
  {
    encCfg->m_allRapPicturesFlag = 1;
  }
  return 0;
}

bool hasNonZeroTemporalID(const EncCfg *encCfg)
{
  for (unsigned int i = 0; i < encCfg->m_gopSize; i++)
  {
    if (encCfg->m_GOPList[i].m_temporalId != 0)
    {
      return true;
    }
  }
  return false;
}

bool hasLeadingPicture(const EncCfg *encCfg)
{
  for (unsigned int i = 0; i < encCfg->m_gopSize; i++)
  {
    for (unsigned int j = 0; j < encCfg->m_GOPList[i].m_numRefPics0; j++)
    {
      if (encCfg->m_GOPList[i].m_deltaRefPics0[j] < 0)
      {
        return true;
      }
    }
    for (unsigned int j = 0; j < encCfg->m_GOPList[i].m_numRefPics1; j++)
    {
      if (encCfg->m_GOPList[i].m_deltaRefPics1[j] < 0)
      {
        return true;
      }
    }
  }
  return false;
}

//! \}
