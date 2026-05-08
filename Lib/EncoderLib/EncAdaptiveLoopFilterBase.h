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

/** \file     EncAdaptiveLoopFilterBase.h
 \brief    Class templates for ALF encoder (declarations only).
 */

#ifndef __ENCADAPTIVELOOPFILTERBASE__
#define __ENCADAPTIVELOOPFILTERBASE__

#include "EncAdaptiveLoopFilter.h"

/// Helper class doing the SFINAE member checks for ALF covariance
template<class ParamsT, decltype(ParamsT::MAX_NUM_ALF_LUMA_COEFF) * = nullptr,
         typename ParamsT::AlfFilterShape * = nullptr>
struct AlfCovarianceHelper
{};

template<class ParamsT> struct AlfCovarianceBase : virtual public AlfCovariance,
                                                   virtual public AlfCovarianceHelper<ParamsT>
{
  static constexpr auto &MAX_NUM_ALF_LUMA_COEFF = ParamsT::MAX_NUM_ALF_LUMA_COEFF;
  static constexpr auto &MAX_ALF_NUM_CLIP_VALS  = ParamsT::MAX_ALF_NUM_CLIP_VALS;

  using TE = float[MAX_NUM_ALF_LUMA_COEFF][MAX_NUM_ALF_LUMA_COEFF];
  using Ty = float[MAX_NUM_ALF_LUMA_COEFF];

  static constexpr int   ROUND(float a) { return (a < 0) ? (int)(a - 0.5f) : (int)(a + 0.5f); }
  static constexpr float REG     = 0.0001f;
  static constexpr float REG_SQR = 0.0000001f;

  void setEyFromClip(const int *clip, TE _E, Ty _y, int size) const
  {
    CHECK(size != numCoeff, "AlfCovariance size mismatch");

    for (ptrdiff_t k = 0; k < size; k++)
    {
      _y[k] = y(clip[k], k);
      for (ptrdiff_t l = 0; l < size; l++)
      {
        _E[k][l] = E(clip[k], clip[l], k, l);
      }
    }
  }

  float optimizeFilter(const int *clip, float *f, int size) const
  {
    gnsSolveByChol(clip, f, size);
    return calculateError(clip, f);
  }

  using AlfCovariance::calculateError;
  float calculateError(const int *clip) const;

  int gnsSolveByChol(TE LHS, float *rhs, float *x, int numEq) const;

protected:
  template<bool useEnableLessClip>
  float optimizeFilter(const int size, int *clip, float *f, bool optimizeClip, const bool enableLessClip = false) const;

  template<bool useEnableLessClip>
  float optimizeFilterClip(const int size, int *clip, const bool enableLessClip = false) const;

private:
  // Cholesky decomposition

  int  gnsSolveByChol(const int *clip, float *x, int numEq) const;
  void gnsBacksubstitution(TE R, float *z, int size, float *A) const;
  void gnsTransposeBacksubstitution(TE U, float *rhs, float *x, int order) const;
  int  gnsCholeskyDec(TE inpMatr, TE outMatr, int numEq) const;
};

template<class ParamsT> class EncAdaptiveLoopFilterBase : virtual public EncAdaptiveLoopFilter
{
protected:
  template<class T> using vector2D = std::vector<std::vector<T>>;

  void roundFiltCoeff(int *const filterCoeffQuant, const float *const filterCoeff, const int numCoeff,
                      const int factor) const;

  int getCoeffRateCcAlf(short chromaCoeff[ParamsT::MAX_NUM_CC_ALF_FILTERS][ParamsT::MAX_NUM_CC_ALF_CHROMA_COEFF],
                        bool filterEnabled[ParamsT::MAX_NUM_CC_ALF_FILTERS], uint8_t filterCount, CompID compID) const;

  void deriveCcAlfFilterCoeff(CompID compID, const PelUnitBuf &recYuv, const PelUnitBuf &recYuvExt,
                              short filterCoeff[ParamsT::MAX_NUM_CC_ALF_FILTERS][ParamsT::MAX_NUM_CC_ALF_CHROMA_COEFF],
                              const uint8_t filterIdx, const AlfCovarianceBase<ParamsT> &alfCovarianceFrameCcAlf,
                              const int numCoeff) const;

  void determineControlIdcValues(CodingStructure &cs, const CompID compID, const PelBuf *buf, const int ctuWidthC,
                                 const int ctuHeightC, const int picWidthC, const int picHeightC,
                                 float   **unfilteredDistortion,
                                 uint64_t *trainingDistortion[ParamsT::MAX_NUM_CC_ALF_FILTERS],
                                 uint64_t *lumaSwingGreaterThanThresholdCount, uint64_t *chromaSampleCountNearMidPoint,
                                 bool reuseTemporalFilterCoeff, uint8_t *trainingCovControl, uint8_t *filterControl,
                                 uint64_t &curTotalDistortion, float &curTotalRate,
                                 bool     filterEnabled[ParamsT::MAX_NUM_CC_ALF_FILTERS],
                                 uint8_t  mapFilterIdxToFilterIdc[ParamsT::MAX_NUM_CC_ALF_FILTERS + 1],
                                 uint8_t &ccAlfFilterCount);

  template<class Covariance> void getFrameStatsCcalf(std::vector<Covariance>    &alfCovarianceFrameCcAlf,
                                                     const vector2D<Covariance> &alfCovarianceCcAlf, CompID compIdx,
                                                     int filterIdc, const int numShapesCcAlf);
};

#endif   // __ENCADAPTIVELOOPFILTERBASE__