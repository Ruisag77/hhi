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

#include "CommonLib/CommonDef.h"
#include "CommonLib/ConstraintInfo.h"
#include <stdint.h>

struct SPS; // Forward declaration.

struct ProfileTierLevel
{
  Level::Tier    m_tierFlag { Level::MAIN };
  Profile::Name  m_profileIdc { Profile::NONE };
  Level::Name    m_levelIdc { Level::NONE };
  bool           m_frameOnlyConstraintFlag { true };
  bool           m_multiLayerEnabledFlag { false };
  ConstraintInfo m_constraintInfo;

  std::array<bool, MAX_TLAYER - 1>    m_subLayerLevelPresentFlag;
  std::array<Level::Name, MAX_TLAYER> m_subLayerLevelIdc;
  std::vector<uint32_t>               m_subProfileIdc;

  ProfileTierLevel()
  {
    m_subLayerLevelPresentFlag.fill(false);
    m_subLayerLevelIdc.fill(Level::NONE);
  }

  friend bool operator==(const ProfileTierLevel &op1, const ProfileTierLevel &op2);
  friend bool operator!=(const ProfileTierLevel &op1, const ProfileTierLevel &op2);

  void copyProfileTierConstraintsFrom(const ProfileTierLevel &ptl)
  {
    m_profileIdc     = ptl.m_profileIdc;
    m_tierFlag       = ptl.m_tierFlag;
    m_constraintInfo = ptl.m_constraintInfo;
  }
};

struct TierLevelFeatures
{
  Level::Name level;
  uint32_t    maxLumaPs;
  uint32_t    maxCpb[Level::NUMBER_OF_TIERS];    // in units of CpbVclFactor or CpbNalFactor bits
  uint32_t    maxSlicesPerAu;
  uint32_t    maxTilesPerAu;
  uint32_t    maxTileCols;
  uint64_t    maxLumaSr;
  uint32_t    maxBr[Level::NUMBER_OF_TIERS];     // in units of BrVclFactor or BrNalFactor bits/s
  uint32_t    minCrBase[Level::NUMBER_OF_TIERS];
  uint32_t    getMaxPicWidthInLumaSamples() const;
  uint32_t    getMaxPicHeightInLumaSamples() const;
};

struct ProfileFeatures
{
  Profile::Name profile;
  const char   *pNameString;
  uint32_t      maxBitDepth;
  ChromaFormat  maxChromaFormat;

  bool                     canUseLevel15p5;
  uint32_t                 cpbVclFactor;
  uint32_t                 cpbNalFactor;
  uint32_t                 formatCapabilityFactorx1000;
  uint32_t                 minCrScaleFactorx100;
  const TierLevelFeatures *tierLevelListInfo;
  bool                     onePictureOnlyFlagMustBe1;

  static const ProfileFeatures *getProfileFeatures(const Profile::Name p);
};

class ProfileTierLevelFeatures
{
private:
  const ProfileFeatures   *m_profile { nullptr };
  const TierLevelFeatures *m_tierLevel { nullptr };
  Level::Tier              m_tier { Level::MAIN };
  int                      m_hbrFactor { 0 };

public:
  void extractPTLInformation(const SPS &sps);
  void extractPTLInformation(const ProfileTierLevel &ptl);

  const ProfileFeatures   *getProfileFeatures() const { return m_profile; }
  const TierLevelFeatures *getTierLevelFeatures() const { return m_tierLevel; }
  Level::Tier              getTier() const { return m_tier; }
  uint64_t                 getCpbSizeInBits() const;
  double                   getMinCr() const;
  uint32_t                 getMaxDpbSize(uint32_t picSizeMaxInSamplesY) const;
};
