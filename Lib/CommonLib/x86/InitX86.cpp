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

/*
 * \ingroup CommonLib
 * \file    InitX86.cpp
 * \brief   Initialize encoder SIMD functions.
 */

#include "CommonLib/CommonDef.h"
#include "CommonLib/InterpolationFilter.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/RdCost.h"
#include "CommonLib/Buffer.h"

#include "CommonLib/AffineGradientSearch.h"

#include "CommonLib/AdaptiveLoopFilterEcm.h"
#include "CommonLib/AdaptiveLoopFilterVtm.h"
#include "CommonLib/SampleAdaptiveOffset.h"

#include "CommonLib/IbcHashMap.h"
#if ENABLE_SIMD_BILATERAL_FILTER
#include "CommonLib/BilateralFilter.h"
#endif
#ifdef TARGET_SIMD_X86

#if ENABLE_SIMD_OPT_MCIF
void InterpolationFilter::initInterpolationFilterX86(/*int iBitDepthY, int iBitDepthC*/)
{
  auto vext = read_x86_extension_flags();
  switch (vext)
  {
  case AVX512:
  case AVX2:
    _initInterpolationFilterX86<AVX2>(/*iBitDepthY, iBitDepthC*/);
    break;
  case AVX:
    _initInterpolationFilterX86<AVX>(/*iBitDepthY, iBitDepthC*/);
    break;
  case SSE42:
  case SSE41:
    _initInterpolationFilterX86<SSE41>(/*iBitDepthY, iBitDepthC*/);
    break;
  default:
    break;
  }
}
#endif

#if ENABLE_SIMD_OPT_BUFFER
void PelBufferOps::initPelBufOpsX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext)
  {
  case AVX512:
  case AVX2:
    _initPelBufOpsX86<AVX2>();
    break;
  case AVX:
    _initPelBufOpsX86<AVX>();
    break;
  case SSE42:
  case SSE41:
    _initPelBufOpsX86<SSE41>();
    break;
  default:
    break;
  }
}
#endif

#if ENABLE_SIMD_OPT_DIST
void RdCost::initRdCostX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext)
  {
  case AVX512:
  case AVX2:
    _initRdCostX86<AVX2>();
    break;
  case AVX:
    _initRdCostX86<AVX>();
    break;
  case SSE42:
  case SSE41:
    _initRdCostX86<SSE41>();
    break;
  default:
    break;
  }
}
#endif

#if ENABLE_SIMD_OPT_AFFINE_ME
void AffineGradientSearch::initAffineGradientSearchX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext)
  {
  case AVX512:
  case AVX2:
    _initAffineGradientSearchX86<AVX2>();
    break;
  case AVX:
    _initAffineGradientSearchX86<AVX>();
    break;
  case SSE42:
  case SSE41:
    _initAffineGradientSearchX86<SSE41>();
    break;
  default:
    break;
  }
}
#endif

#if ENABLE_SIMD_OPT_ALF
void AdaptiveLoopFilterVtm::initAdaptiveLoopFilterX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext)
  {
  case AVX512:
  case AVX2:
    _initAdaptiveLoopFilterX86<AVX2>();
    break;
  case AVX:
    _initAdaptiveLoopFilterX86<AVX>();
    break;
  case SSE42:
  case SSE41:
    _initAdaptiveLoopFilterX86<SSE41>();
    break;
  default:
    break;
  }
}

void AdaptiveLoopFilterEcm::initAdaptiveLoopFilterX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext)
  {
  case AVX512:
  case AVX2:
    _initAdaptiveLoopFilterX86<AVX2>();
    break;
  case AVX:
    _initAdaptiveLoopFilterX86<AVX>();
    break;
  case SSE42:
  case SSE41:
    _initAdaptiveLoopFilterX86<SSE41>();
    break;
  default:
    break;
  }
}
#endif   // ENABLE_SIMD_OPT_ALF

#if ENABLE_SIMD_OPT_IBC
void IbcHashMap::initIbcHashMapX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext)
  {
  case AVX512:
  case AVX2:
  case AVX:
  case SSE42:
    _initIbcHashMapX86<SSE42>();
    break;
  case SSE41:
  default:
    break;
  }
}
#endif

#if ENABLE_SIMD_OPT_INTRAPRED
void IntraPrediction::initIntraPredictionX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext)
  {
  case AVX512:
  case AVX2:
    _initIntraPredictionX86<AVX2>();
    break;
  case AVX:
    _initIntraPredictionX86<AVX>();
    break;
  case SSE42:
  case SSE41:
    _initIntraPredictionX86<SSE41>();
    break;
  default:
    break;
  }
}
#endif

#if ENABLE_SIMD_TRAFO
void TCoeffOps::initTCoeffOpsX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext)
  {
  case AVX512:
  case AVX2:
    _initTCoeffOpsX86<AVX2>();
    break;
  case AVX:
    _initTCoeffOpsX86<AVX>();
    break;
  case SSE42:
    _initTCoeffOpsX86<SSE42>();
    break;
  case SSE41:
    _initTCoeffOpsX86<SSE41>();
    break;
  default:
    break;
  }
}

void TrQuant::initTrQuantX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext)
  {
  case AVX512:
  case AVX2:
    _initTrQuantX86<AVX2>();
    break;
  case AVX:
    _initTrQuantX86<AVX>();
    break;
  case SSE42:
    _initTrQuantX86<SSE42>();
    break;
  case SSE41:
    _initTrQuantX86<SSE41>();
    break;
  default:
    break;
  }
}

#endif
#if ENABLE_SIMD_BILATERAL_FILTER
void BilateralFilter::initBilateralFilterX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext)
  {
  case AVX512:
  case AVX2:
    _initBilateralFilterX86<AVX2>();
    break;
  case AVX:
    _initBilateralFilterX86<AVX>();
    break;
  case SSE42:
  case SSE41:
    _initBilateralFilterX86<SSE41>();
    break;
  default:
    break;
  }
}
#endif

#if ENABLE_SIMD_OPT_CCSAO
void SampleAdaptiveOffset::initCCSAOX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext)
  {
  case AVX512:
  case AVX2:
    _initCCSAOX86<AVX2>();
    break;
  case AVX:
    _initCCSAOX86<AVX>();
    break;
  case SSE42:
  case SSE41:
    _initCCSAOX86<SSE41>();
    break;
  default:
    break;
  }
}
#endif
#endif
