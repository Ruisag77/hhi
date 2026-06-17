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
#include <array>
#include "CommonLib/CommonDef.h"
#include "CommonLib/Contexts.h"
#include "CommonLib/NNFilterUnified.h"

#if ENABLE_NNLF

// fwd
namespace sadl
{
template<typename T> class Model;
template<typename T> class Tensor;
}   // namespace sadl

class CABACEncoder;
class CABACWriter;

class EncNNFilterUnified : public NNFilterUnified
{
public:
  void initCabac(CABACEncoder *cabacEncoder, CtxPool *ctxPool, Slice &slice);
  void chooseParameters(Picture &pic);

private:
  void   scaleFactorDerivation(Picture &pic, NnlfFilterParameters &prms, int prmId, int scaleId);
  void   scalePicture(Picture &pic, int scaleId);
  void   parameterSearch(Picture &pic, double &minCost, std::vector<int> &bestPrmId, int scaleId);
  double getSignalingCost(Picture &pic);

  std::array<double, CompID::MAX_NUM_COMP> m_lambda;
  CABACWriter                             *m_CABACEstimator = nullptr;
  CtxPool                                 *m_CtxPool        = nullptr;
  bool                                     useNnlfDecOpt    = false;
};

#endif
