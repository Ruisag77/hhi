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

/** \file     MotionInfo.h
    \brief    motion information handling classes (header)
    \todo     MvField seems to be better to be inherited from Mv
*/

#ifndef __MOTIONINFO__
#define __MOTIONINFO__

#include "CommonDef.h"
#include "Mv.h"

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Type definition
// ====================================================================================================================

/// parameters for AMVP
struct AMVPInfo
{
  Mv       mvCand[AMVP_MAX_NUM_CANDS_MEM];  ///< array of motion vector predictor candidates
  unsigned numCand;                       ///< number of motion vector predictor candidates
};

struct AffineAMVPInfo
{
  Mv mvCandLT[AMVP_MAX_NUM_CANDS_MEM];  ///< array of affine motion vector predictor candidates for left-top corner
  Mv mvCandRT[AMVP_MAX_NUM_CANDS_MEM];  ///< array of affine motion vector predictor candidates for right-top corner
  Mv mvCandLB[AMVP_MAX_NUM_CANDS_MEM];   ///< array of affine motion vector predictor candidates for left-bottom corner
  unsigned numCand;   ///< number of motion vector predictor candidates
};

struct RMVFInfo
{
  Mv       mvp;
  Position pos;
  int8_t   refIdx;
  RMVFInfo() : mvp(Mv(0, 0)), pos(Position(0, 0)), refIdx(-1) {};
  RMVFInfo(Mv const cMv, Position const cPos, int8_t const cIdx) : mvp(cMv), pos(cPos), refIdx(cIdx) {};
};

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// class for motion vector with reference index
struct MvField
{
  Mv      mv;
  int16_t refIdx;

  MvField() : refIdx(NOT_VALID) {}
  MvField(Mv const &cMv, const int _refIdx) : mv(cMv), refIdx(_refIdx) {}

  void setMvField(Mv const &cMv, const int _refIdx)
  {
    CHECK(_refIdx == -1 && cMv != Mv(0, 0), "Must not happen.");
    mv     = cMv;
    refIdx = _refIdx;
  }

  bool operator==(const MvField &other) const
  {
    CHECK(refIdx == -1 && mv != Mv(0, 0), "Error in operator== of MvField.");
    CHECK(other.refIdx == -1 && other.mv != Mv(0, 0), "Error in operator== of MvField.");
    return refIdx == other.refIdx && mv == other.mv;
  }
  bool operator!=(const MvField &other) const
  {
    CHECK(refIdx == -1 && mv != Mv(0, 0), "Error in operator!= of MvField.");
    CHECK(other.refIdx == -1 && other.mv != Mv(0, 0), "Error in operator!= of MvField.");
    return refIdx != other.refIdx || mv != other.mv;
  }
};

struct MotionInfo
{
  Mv       mv[NUM_RPL01] {};
  Mv       bv {};
  int16_t  refIdx[NUM_RPL01] { NOT_VALID, NOT_VALID };
  bool     isInter { false };
  bool     isIBCmot { false };
  int8_t   interDir { 0 };
  bool     useAltHpelIf { false };
  uint16_t sliceIdx { 0 };
  uint8_t  bcwIdx { 0 };
  bool     usesLIC { false };

  MotionInfo() {}
  // ensure that MotionInfo(0) produces '\x000....' bit pattern - needed to work with AreaBuf - don't use this
  // constructor for anything else
  MotionInfo(int i)
    : refIdx { 0, 0 }
    , isInter(i != 0)
    , isIBCmot(false)
    , interDir(0)
    , useAltHpelIf(false)
    , sliceIdx(0)
    , bcwIdx(0)
    , usesLIC { false }
  {
    CHECKD(i != 0, "The argument for this constructor has to be '0'");
  }

  bool operator==(const MotionInfo &mi) const
  {
    if (isInter != mi.isInter)
    {
      return false;
    }
    if (isIBCmot != mi.isIBCmot)
    {
      return false;
    }
    if (isInter)
    {
      if (sliceIdx != mi.sliceIdx)
      {
        return false;
      }
      if (interDir != mi.interDir)
      {
        return false;
      }

      if (interDir != 2)
      {
        if (refIdx[0] != mi.refIdx[0])
        {
          return false;
        }
        if (mv[0] != mi.mv[0])
        {
          return false;
        }
      }

      if (interDir != 1)
      {
        if (refIdx[1] != mi.refIdx[1])
        {
          return false;
        }
        if (mv[1] != mi.mv[1])
        {
          return false;
        }
      }
    }

    return true;
  }

  bool operator!=(const MotionInfo &mi) const { return !(*this == mi); }

  bool isSameMiObmc(const MotionInfo &mi) const { return (*this == mi && (interDir != 3 || bcwIdx == mi.bcwIdx)); }
};

class BcwMotionParam
{
  RefSetArray<bool>       m_readOnly;
  RefSetArray<Mv>         m_mv;
  RefSetArray<Distortion> m_dist;

  EnumArray<RefSetArray<bool>, AffineModel>       m_readOnlyAffine;
  EnumArray<RefSetArray<Mv[3]>, AffineModel>      m_mvAffine;
  EnumArray<RefSetArray<Distortion>, AffineModel> m_distAffine;
  EnumArray<RefSetArray<int>, AffineModel>        m_mvpIdx;

public:
  void reset()
  {
    Mv *pMv = &(m_mv[0][0]);
    for (int ui = 0; ui < NUM_RPL01 * MAX_NUM_REF; ++ui, ++pMv)
    {
      pMv->set(std::numeric_limits<int16_t>::max(), std::numeric_limits<int16_t>::max());
    }

    Mv *pAffineMv = &(m_mvAffine[AffineModel::_4_PARAMS][0][0][0]);
    for (int ui = 0; ui < 2 * NUM_RPL01 * MAX_NUM_REF * 3; ++ui, ++pAffineMv)
    {
      pAffineMv->set(0, 0);
    }

    std::fill_n(m_readOnly[0], sizeof(m_readOnly) / sizeof(bool), false);
    std::fill_n(m_dist[0], sizeof(m_dist) / sizeof(Distortion), MAX_UINT64);
    std::fill_n(m_readOnlyAffine[AffineModel::_4_PARAMS][0], sizeof(m_readOnlyAffine) / sizeof(bool), false);
    std::fill_n(m_distAffine[AffineModel::_4_PARAMS][0], sizeof(m_distAffine) / sizeof(Distortion), MAX_UINT64);
    std::fill_n(m_mvpIdx[AffineModel::_4_PARAMS][0], sizeof(m_mvpIdx) / sizeof(int), 0);
  }

  void setReadMode(bool b, uint32_t refList, uint32_t refIdx) { m_readOnly[refList][refIdx] = b; }
  bool isReadMode(uint32_t refList, uint32_t refIdx) { return m_readOnly[refList][refIdx]; }

  void setReadModeAffine(bool b, uint32_t refList, uint32_t refIdx, AffineModel am)
  {
    m_readOnlyAffine[am][refList][refIdx] = b;
  }
  bool isReadModeAffine(uint32_t refList, uint32_t refIdx, AffineModel am)
  {
    return m_readOnlyAffine[am][refList][refIdx];
  }

  Mv &getMv(uint32_t refList, uint32_t refIdx) { return m_mv[refList][refIdx]; }

  void copyFrom(Mv &rcMv, Distortion dist, uint32_t refList, uint32_t refIdx)
  {
    m_mv[refList][refIdx]   = rcMv;
    m_dist[refList][refIdx] = dist;
  }

  void copyTo(Mv &rcMv, Distortion &dist, uint32_t refList, uint32_t refIdx)
  {
    rcMv = m_mv[refList][refIdx];
    dist = m_dist[refList][refIdx];
  }

  void copyAffineMvFrom(Mv (&racAffineMvs)[3], Distortion dist, uint32_t refList, uint32_t refIdx, AffineModel am,
                        const int mvpIdx)
  {
    memcpy(m_mvAffine[am][refList][refIdx], racAffineMvs, 3 * sizeof(Mv));
    m_distAffine[am][refList][refIdx] = dist;
    m_mvpIdx[am][refList][refIdx]     = mvpIdx;
  }

  void copyAffineMvTo(Mv acAffineMvs[3], Distortion &dist, uint32_t refList, uint32_t refIdx, AffineModel am,
                      int &mvpIdx)
  {
    memcpy(acAffineMvs, m_mvAffine[am][refList][refIdx], 3 * sizeof(Mv));
    dist   = m_distAffine[am][refList][refIdx];
    mvpIdx = m_mvpIdx[am][refList][refIdx];
  }
};

struct AffineMotionInfo
{
  union
  {
    uint64_t oneSetAffineParametersPattern;
    short    oneSetAffineParameters[4];
  };
  AffineMotionInfo(uint64_t p) { oneSetAffineParametersPattern = p; };
  AffineMotionInfo() { oneSetAffineParametersPattern = 0; };

  bool operator==(const AffineMotionInfo &motInfo)
  {
    return oneSetAffineParametersPattern == motInfo.oneSetAffineParametersPattern;
  }
};

struct AffineInheritInfo
{
  Position basePos;
  MvField  baseMV[2];

  union
  {
    uint64_t oneSetAffineParametersPattern0;
    short    oneSetAffineParameters0[4];
  };
  union
  {
    uint64_t oneSetAffineParametersPattern1;
    short    oneSetAffineParameters1[4];
  };
  AffineInheritInfo() { baseMV[0].refIdx = baseMV[1].refIdx = -1; };

  bool operator==(const AffineInheritInfo &motInfo)
  {
    if (baseMV[0].refIdx != motInfo.baseMV[0].refIdx)
    {
      return false;
    }
    if (baseMV[1].refIdx != motInfo.baseMV[1].refIdx)
    {
      return false;
    }
    if (baseMV[0].refIdx != -1 && oneSetAffineParametersPattern0 != motInfo.oneSetAffineParametersPattern0)
    {
      return false;
    }
    if (baseMV[1].refIdx != -1 && oneSetAffineParametersPattern1 != motInfo.oneSetAffineParametersPattern1)
    {
      return false;
    }
    return true;
  }
};

struct LutMotionCand
{
  static_vector<MotionInfo, MAX_NUM_HMVP_CANDS>                    lut;
  static_vector<MotionInfo, MAX_NUM_HMVP_CANDS>                    lutIbc;
  static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS>          lutAff[MAX_NUM_AFFHMVP_ENTRIES];
  static_vector<AffineInheritInfo, MAX_NUM_AFF_INHERIT_HMVP_CANDS> lutAffInherit;
};
#endif   // __MOTIONINFO__
