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

/** \file     Mv.cpp
    \brief    motion vector class
*/

#include "Mv.h"

#include "Common.h"
#include "Slice.h"

const MvPrecision Mv::m_amvrPrecision[4]  = { MvPrecision::QUARTER, MvPrecision::ONE, MvPrecision::FOUR,
                                              MvPrecision::HALF };   // for cu.imv=0, 1, 2 and 3
const MvPrecision Mv::m_amvrPrecAffine[3] = { MvPrecision::QUARTER, MvPrecision::SIXTEENTH,
                                              MvPrecision::ONE };   // for cu.imv=0, 1 and 2
const MvPrecision Mv::m_amvrPrecIbc[4]    = { MvPrecision::QUARTER, MvPrecision::ONE, MvPrecision::FOUR,
                                              MvPrecision::HALF }; // for cu.imv=0, 1, 2 and 3

void roundAffineMv(int &mvx, int &mvy, int nShift)
{
  const int nOffset = 1 << (nShift - 1);
  mvx               = (mvx + nOffset - (mvx >= 0)) >> nShift;
  mvy               = (mvy + nOffset - (mvy >= 0)) >> nShift;
}

void (*clipMv)(Mv &rcMv, const Position &pos, const Size &size, const SPS &sps, const PPS &pps);

void clipMvInPic(Mv &rcMv, const Position &pos, const Size &size, const SPS &sps, const PPS &pps)
{
  if (pps.m_wrapAroundEnabledFlag)
  {
    wrapClipMv(rcMv, pos, size, &sps, &pps);
    return;
  }

  int mvShift = MV_FRACTIONAL_BITS_INTERNAL;

  const int offset = sps.m_MCBP ? (PIC_MARGIN / 2) + MC_PAD_SIZE : PIC_MARGIN / 2;

  int horMax = (pps.m_picWidthInLumaSamples + offset - (int)pos.x - 1) << mvShift;
  int horMin = (-(int)sps.m_maxCuWidth - offset - (int)pos.x + 1) * (1 << mvShift);

  int verMax = (pps.m_picHeightInLumaSamples + offset - (int)pos.y - 1) << mvShift;
  int verMin = (-(int)sps.m_maxCuHeight - offset - (int)pos.y + 1) * (1 << mvShift);

  // Keep LSBs such as to not change filter phase
  const int mask = (1 << MV_FRACTIONAL_BITS_INTERNAL) - 1;
  rcMv.setHor(std::min(horMax, std::max(horMin, rcMv.getHor())) | (rcMv.getHor() & mask));
  rcMv.setVer(std::min(verMax, std::max(verMin, rcMv.getVer())) | (rcMv.getVer() & mask));
}

void clipMvInSubpic(Mv &rcMv, const Position &pos, const Size &size, const SPS &sps, const PPS &pps)
{
  if (pps.m_wrapAroundEnabledFlag)
  {
    wrapClipMv(rcMv, pos, size, &sps, &pps);
    return;
  }

  int       mvShift = MV_FRACTIONAL_BITS_INTERNAL;
  const int offset  = sps.m_MCBP ? (PIC_MARGIN / 2) + MC_PAD_SIZE : PIC_MARGIN / 2;

  int horMax = (pps.m_picWidthInLumaSamples + offset - (int)pos.x - 1) << mvShift;
  int horMin = (-(int)sps.m_maxCuWidth - offset - (int)pos.x + 1) * (1 << mvShift);

  int verMax = (pps.m_picHeightInLumaSamples + offset - (int)pos.y - 1) << mvShift;
  int verMin = (-(int)sps.m_maxCuHeight - offset - (int)pos.y + 1) * (1 << mvShift);

  const SubPic &curSubPic = pps.getSubPicFromPos(pos);
  if (curSubPic.m_treatedAsPicFlag)
  {
    horMax = ((curSubPic.m_subPicRight + 1) + offset - (int)pos.x - 1) << mvShift;
    horMin = (-(int)sps.m_maxCuWidth - offset - ((int)pos.x - curSubPic.m_subPicLeft) + 1) * (1 << mvShift);

    verMax = ((curSubPic.m_subPicBottom + 1) + offset - (int)pos.y - 1) << mvShift;
    verMin = (-(int)sps.m_maxCuHeight - offset - ((int)pos.y - curSubPic.m_subPicTop) + 1) * (1 << mvShift);
  }

  // Keep LSBs such as to not change filter phase
  const int mask = (1 << MV_FRACTIONAL_BITS_INTERNAL) - 1;
  rcMv.setHor(std::min(horMax, std::max(horMin, rcMv.getHor())) | (rcMv.getHor() & mask));
  rcMv.setVer(std::min(verMax, std::max(verMin, rcMv.getVer())) | (rcMv.getVer() & mask));
}

bool wrapClipMv(Mv &rcMv, const Position &pos, const struct Size &size, const SPS *sps, const PPS *pps)
{
  bool wrapRef = true;

  const int mvShift = MV_FRACTIONAL_BITS_INTERNAL;
  const int offset  = PIC_MARGIN / 2;

  int horMax = (pps->m_picWidthInLumaSamples + sps->m_maxCuWidth - size.width + offset - (int)pos.x - 1) << mvShift;
  int horMin = (-(int)sps->m_maxCuWidth - offset - (int)pos.x + 1) * (1 << mvShift);

  int verMax = (pps->m_picHeightInLumaSamples + offset - (int)pos.y - 1) << mvShift;
  int verMin = (-(int)sps->m_maxCuHeight - offset - (int)pos.y + 1) * (1 << mvShift);

  const SubPic &curSubPic = pps->getSubPicFromPos(pos);
  if (curSubPic.m_treatedAsPicFlag)
  {
    verMax = ((curSubPic.m_subPicBottom + 1) + offset - (int)pos.y - 1) << mvShift;
    verMin = (-(int)sps->m_maxCuHeight - offset - ((int)pos.y - curSubPic.m_subPicTop) + 1) * (1 << mvShift);
  }
  int mvX = rcMv.getHor();

  if (mvX > horMax)
  {
    mvX -= pps->m_wrapAroundOffset << mvShift;
    wrapRef = false;
  }
  if (mvX < horMin)
  {
    mvX += pps->m_wrapAroundOffset << mvShift;
    wrapRef = false;
  }

  // Keep LSBs such as to not change filter phase
  const int mask = (1 << MV_FRACTIONAL_BITS_INTERNAL) - 1;
  rcMv.setHor(std::min(horMax, std::max(horMin, mvX)) | (mvX & mask));
  rcMv.setVer(std::min(verMax, std::max(verMin, rcMv.getVer())) | (rcMv.getVer() & mask));
  return wrapRef;
}

//! \}
