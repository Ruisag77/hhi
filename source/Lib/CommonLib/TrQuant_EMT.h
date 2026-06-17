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

/** \file     TrQuant_EMT.h
    \brief    transform and quantization class (header)
*/

#ifndef __TRQUANT_EMT__
#define __TRQUANT_EMT__

#include "CommonDef.h"

#if ENABLE_SIMD_TRAFO
struct TCoeffOps
{
  TCoeffOps();

#if defined(TARGET_SIMD_X86)
  void                         initTCoeffOpsX86();
  template<X86_VEXT vext> void _initTCoeffOpsX86();

#endif
  void (*cpyResi8)(const TCoeff *src, Pel *dst, ptrdiff_t stride, unsigned width, unsigned height);
  void (*cpyResi4)(const TCoeff *src, Pel *dst, ptrdiff_t stride, unsigned width, unsigned height);
  void (*cpyCoeff8)(const Pel *src, ptrdiff_t stride, TCoeff *dst, unsigned width, unsigned height);
  void (*cpyCoeff4)(const Pel *src, ptrdiff_t stride, TCoeff *dst, unsigned width, unsigned height);
  void (*fastInvCore[7])(const TMatrixCoeff *it, const TCoeff *src, TCoeff *dst, unsigned lines, unsigned reducedLines,
                         unsigned rows);
  void (*fastFwdCore_2D[7])(const TMatrixCoeff *it, const TCoeff *src, TCoeff *dst, unsigned lines,
                            unsigned reducedLines, unsigned cutoff, int shift);
  void (*fastFwdCore_1D[7])(const TMatrixCoeff *it, const TCoeff *src, TCoeff *dst, unsigned lines,
                            unsigned reducedLines, unsigned cutoff, int shift);
  void (*roundClip4)(TCoeff *dst, unsigned width, unsigned height, unsigned stride, const TCoeff outputMin,
                     const TCoeff outputMax, const TCoeff round, const TCoeff shift);
  void (*roundClip8)(TCoeff *dst, unsigned width, unsigned height, unsigned stride, const TCoeff outputMin,
                     const TCoeff outputMax, const TCoeff round, const TCoeff shift);
};

extern TCoeffOps g_tCoeffOps;

#endif

////DCT-II transforms
void fastForwardDCT2_B2(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT2_B2(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                        const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT2_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                        const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT2_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                        const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT2_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT2_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT2_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B128(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT2_B128(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                          const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT2_B256(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT2_B256(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                          const TCoeff outputMinimum, const TCoeff outputMaximum);

// DST-VII transforms (EMT)
void fastForwardDST7_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST7_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                        const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST7_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                        const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST7_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST7_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST7_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B128(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST7_B128(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                          const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST7_B256(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST7_B256(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                          const TCoeff outputMinimum, const TCoeff outputMaximum);

// DCT-VIII transforms (EMT)
void fastForwardDCT8_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT8_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                        const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT8_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                        const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT8_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT8_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT8_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B128(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT8_B128(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                          const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT8_B256(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT8_B256(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                          const TCoeff outputMinimum, const TCoeff outputMaximum);

// DCT-V
void fastForwardDCT5_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT5_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                        const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT5_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT5_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                        const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT5_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT5_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT5_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT5_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT5_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT5_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT5_B128(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT5_B128(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                          const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDCT5_B256(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDCT5_B256(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                          const TCoeff outputMinimum, const TCoeff outputMaximum);

// DST-IV
void fastForwardDST4_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST4_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                        const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST4_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST4_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                        const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST4_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST4_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST4_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST4_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST4_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST4_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST4_B128(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST4_B128(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                          const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST4_B256(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST4_B256(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                          const TCoeff outputMinimum, const TCoeff outputMaximum);

// DST-I
void fastForwardDST1_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST1_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                        const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST1_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST1_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                        const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST1_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST1_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST1_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST1_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST1_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST1_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST1_B128(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST1_B128(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                          const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardDST1_B256(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseDST1_B256(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                          const TCoeff outputMinimum, const TCoeff outputMaximum);

// IDENTITY
void fastForwardIDTR_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseIDTR_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                        const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardIDTR_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseIDTR_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                        const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardIDTR_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseIDTR_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardIDTR_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseIDTR_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardIDTR_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseIDTR_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardIDTR_B128(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseIDTR_B128(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                          const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardIDTR_B256(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseIDTR_B256(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                          const TCoeff outputMinimum, const TCoeff outputMaximum);

// KLT 0
void fastForwardKLT0_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseKLT0_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                        const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardKLT0_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseKLT0_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                        const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardKLT0_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseKLT0_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);

// KLT 1
void fastForwardKLT1_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseKLT1_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                        const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardKLT1_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseKLT1_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                        const TCoeff outputMinimum, const TCoeff outputMaximum);
void fastForwardKLT1_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2);
void fastInverseKLT1_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2,
                         const TCoeff outputMinimum, const TCoeff outputMaximum);

#endif // __TRQUANT__
