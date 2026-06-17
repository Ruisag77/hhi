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

 /** \file     Reshape.h
   \brief    reshaping header and class (header)
*/

#ifndef __RESHAPE__
#define __RESHAPE__

#pragma once

#include "CommonDef.h"
#include "Rom.h"
#include "CommonLib/Picture.h"
//! \ingroup CommonLib
//! \{
// ====================================================================================================================
// Class definition
// ====================================================================================================================

class Reshape
{

public:
  SliceReshapeInfo      m_sliceReshapeInfo {};
  bool                  m_ctuFlag { false };
  bool                  m_recReshaped { false };
  std::vector<Pel>      m_invLUT {};
  std::vector<Pel>      m_fwdLUT {};
  std::vector<int>      m_chromaAdjHelpLUT {};
  std::vector<uint16_t> m_binCW {};
  uint16_t              m_initCW { 0 };
  bool                  m_reshapeFlag { true };
  std::vector<Pel>      m_reshapePivot {};
  std::vector<Pel>      m_inputPivot {};
  std::vector<int32_t>  m_fwdScaleCoef {};
  std::vector<int32_t>  m_invScaleCoef {};
  int                   m_lumaBD { 0 };
  int                   m_reshapeLUTSize { 0 };
  Reshape() {}
  ~Reshape() {}

  void createDec(int bitDepth);
  void destroy();

  int calculateChromaAdj(Pel avgLuma);
  int getPWLIdxInv(int lumaVal);

  void constructReshaper();
  int  calculateChromaAdjVpduNei(TransformUnit &tu, const CompArea &areaY);

  static Pel scalePel(Pel val, const int16_t scale, const Pel maxVal)
  {
    Pel sign    = val >> (8 * sizeof(Pel) - 1);
    val         = Clip3<Pel>(~maxVal, maxVal, val);
    val         = (val ^ sign) - sign;   // abs
    int32_t tmp = (val * scale + (1 << CSCALE_FP_PREC >> 1)) >> CSCALE_FP_PREC;
    tmp         = (tmp ^ sign) - sign;   // restore sign
    if constexpr (sizeof(Pel) < 4)
    {
      // avoid overflow when storing data
      val = Clip3<int>(std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max(), tmp);
    }
    else
    {
      val = tmp;
    }
    return val;
  }
};// END CLASS DEFINITION Reshape

//! \}
#endif // __RESHAPE__
