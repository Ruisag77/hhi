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

#ifndef __LOOPFILTERCCCM__
#define __LOOPFILTERCCCM__

#include "CommonDef.h"
#include "Unit.h"
#include "UnitTools.h"
#include "IntraPrediction.h"

class LoopFilterCccm
{
public:
  typedef void (*CalculateCorrelationMatrix)(int nRows, int nCols, int nSamples, const int16_t *input[],
                                             int64_t *output, int outputStride);
  CalculateCorrelationMatrix calculateCorrelationMatrix {};

  void lfCccmFillDownsampledLumaBuffers(const CodingStructure &cs);
  void lfCccmCtuProcess(const CodingStructure &cs, const PelUnitBuf &recSao, const int ctuRsAddr, PelBuf &updateCb,
                        PelBuf &updateCr, bool *ctuProcessedEnc = nullptr);
  void lfCccmSetFrameLevelInheritedParameters(CodingStructure &cs, const int ctuRsAddr0 = -1);
  void lfCccmInitIntraPred(IntraPrediction *intraPred)
  {
    m_lfCccmIntraPred = intraPred;
    m_lfCccmA         = m_lfCccmIntraPred->m_a;
    m_lfCccmYCb       = m_lfCccmIntraPred->m_cb;
    m_lfCccmYCr       = m_lfCccmIntraPred->m_cr;
  }
  void lfCccmCreatePelStorage(const CodingStructure &cs)
  {
    if (m_lfCccmPelStorage.bufs.empty() ||
        (m_lfCccmPelStorage.Y().width != cs.picture->lwidth() ||
         m_lfCccmPelStorage.Y().height != cs.picture->lheight()))
    {
      m_lfCccmPelStorage.destroy();
      m_lfCccmPelStorage.create(ChromaFormat::_400,
                                Area(0, 0, cs.picture->chromaSize().width, cs.picture->chromaSize().height), 0,
                                m_lfCccmFilterPadding, 0, false);
    }
    m_lfCccmLumaSaoDownsampled = m_lfCccmPelStorage.Y();
  }

protected:
  int m_lfCccmLumaOffset;
  int m_lfCccmModelType;
  int m_lfCccmModelSize;

  static constexpr int m_lfCccmMaxNumModels  = 8;
  static constexpr int m_lfCccmMaxNumWindows = 8;
  static constexpr int m_lfCccmArrayStride   = 16;

  static constexpr int m_lfCccmFilterPadding     = 1;
  static constexpr int m_lfCccmBadBlockThreshold = 32;
  static constexpr int m_lfCccmModelMap[8][11]   = {
    { 0, 1, 2, 3, 4, 9, 10, 0, 0, 0, 0 }, { 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, { 0, 9, 10, 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 1, 2, 3, 4, 10, 0, 0, 0, 0, 0 }, { 0, 3, 4, 9, 10, 0, 0, 0, 0, 0, 0 }, { 0, 1, 2, 9, 10, 0, 0, 0, 0, 0, 0 },
    { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 }, { 0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 0 },
  };

  const std::vector<Size> m_lfCccmWindowSizes = { Size(4, 4),   Size(8, 2),   Size(2, 8),   Size(8, 8),
                                                  Size(16, 16), Size(32, 32), Size(64, 64), Size(128, 128) };

  CccmModel      m_lfCccmModelCb;
  CccmModel      m_lfCccmModelCr;
  CccmCovariance m_lfCccmSolver;

  CPelBuf m_lfCccmSaoCb;
  CPelBuf m_lfCccmSaoCr;
  CPelBuf m_lfCccmSaoY;
  PelBuf  m_lfCccmUpdateCb;
  PelBuf  m_lfCccmUpdateCr;
  PelBuf  m_lfCccmLumaSaoDownsampled;

  PelStorage m_lfCccmPelStorage;

  Size m_lfCccmWindowSize;
  Area m_lfCccmPartitionArea;
  Area m_lfCccmWindowArea;
  Area m_lfCccmRegressionWindow;

  ClpRng m_lfCccmClpRngCb;
  ClpRng m_lfCccmClpRngCr;

  IntraPrediction *m_lfCccmIntraPred;

  std::vector<std::vector<int32_t>>              m_lfCccmXCorr;
  std::vector<std::vector<std::vector<int32_t>>> m_lfCccmAutoCorr;

  TCccmCoeff (&m_lfCccmATA)[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX] = m_lfCccmSolver.ATA;
  TCccmCoeff (&m_lfCccmATCb)[CCCM_NUM_PARAMS_MAX]                     = m_lfCccmSolver.ATCb;
  TCccmCoeff (&m_lfCccmATCr)[CCCM_NUM_PARAMS_MAX]                     = m_lfCccmSolver.ATCr;

  Pel (*m_lfCccmA)[CCCM_REF_SAMPLES_MAX];
  Pel       *m_lfCccmYCb;
  Pel       *m_lfCccmYCr;
  Pel        m_lfCccmSamples[m_lfCccmArrayStride];
  Pel        m_lfCccmChromaOffsetCb;
  Pel        m_lfCccmChromaOffsetCr;
  const int *m_lfCccmMmap;
  int        m_lfCccmMmap2[11];
  bool       m_lfCccmBadWindow;
  bool       m_lfCccmIsEncoder;

  int m_lfCccmMultiModel;
  Pel m_lfCccmThreshold;

  void lfCccmSetMultiModelParameters()
  {
    m_lfCccmThreshold =
      m_lfCccmLumaSaoDownsampled.subBuf(m_lfCccmRegressionWindow, m_lfCccmRegressionWindow).computeAvg();
  }

  void lfCccmInitBd(const SPS &sps) { m_lfCccmLumaOffset = 1 << (sps.m_bitDepths[ChannelType::LUMA] - 1); }

  void lfCccmResetArrays()
  {
    for (int coli0 = 0; coli0 < m_lfCccmModelSize; coli0++)
    {
      for (int coli1 = coli0; coli1 < m_lfCccmModelSize; coli1++)
      {
        m_lfCccmATA[coli0][coli1] = 0;
      }
    }
    memset(m_lfCccmATCb, 0, m_lfCccmModelSize * sizeof(TCccmCoeff));
    memset(m_lfCccmATCr, 0, m_lfCccmModelSize * sizeof(TCccmCoeff));
  }

  void lfCccmConvolveModel(const Pel *src0, const Pel *src1, const Pel *src2, Pel &outCb, Pel &outCr,
                           const CccmModel &modelCb, const CccmModel &modelCr)
  {
    switch (m_lfCccmModelType)
    {
    case 0:
      m_lfCccmSamples[0] = src0[0]; // C
      m_lfCccmSamples[1] = src1[0]; // N
      m_lfCccmSamples[2] = src2[0]; // S
      m_lfCccmSamples[3] = src0[-1]; // W
      m_lfCccmSamples[4] = src0[1]; // E
      m_lfCccmSamples[5] = modelCb.nonlinear(src0[0]);
      break;
    case 1:
      m_lfCccmSamples[0] = src0[0]; // C
      break;
    case 2:
      m_lfCccmSamples[0] = src0[0]; // C
      m_lfCccmSamples[1] = modelCb.nonlinear(src0[0]);
      break;
    case 3:
      m_lfCccmSamples[0] = src0[0]; // C
      m_lfCccmSamples[1] = src1[0]; // N
      m_lfCccmSamples[2] = src2[0]; // S
      m_lfCccmSamples[3] = src0[-1]; // W
      m_lfCccmSamples[4] = src0[1]; // E
      break;
    case 4:
      m_lfCccmSamples[0] = src0[0]; // C
      m_lfCccmSamples[1] = src0[-1]; // W
      m_lfCccmSamples[2] = src0[1]; // E
      m_lfCccmSamples[3] = modelCb.nonlinear(src0[0]);
      break;
    case 5:
      m_lfCccmSamples[0] = src0[0]; // C
      m_lfCccmSamples[1] = src1[0]; // N
      m_lfCccmSamples[2] = src2[0]; // S
      m_lfCccmSamples[3] = modelCb.nonlinear(src0[0]);
      break;
    case 6:
      m_lfCccmSamples[0] = src0[0]; // C
      m_lfCccmSamples[1] = src1[0]; // N
      m_lfCccmSamples[2] = src2[0]; // S
      m_lfCccmSamples[3] = src0[-1]; // W
      m_lfCccmSamples[4] = src0[1]; // E
      m_lfCccmSamples[5] = src1[-1]; // NW
      m_lfCccmSamples[6] = src1[1]; // NE
      m_lfCccmSamples[7] = src2[-1]; // SW
      m_lfCccmSamples[8] = src2[1]; // SE
      m_lfCccmSamples[9] = modelCb.nonlinear(src0[0]);
      break;
    case 7:
      m_lfCccmSamples[0] = src0[0]; // C
      m_lfCccmSamples[1] = src1[0]; // N
      m_lfCccmSamples[2] = src2[0]; // S
      m_lfCccmSamples[3] = src0[-1]; // W
      m_lfCccmSamples[4] = src0[1]; // E
      m_lfCccmSamples[5] = src1[-1]; // NW
      m_lfCccmSamples[6] = src1[1]; // NE
      m_lfCccmSamples[7] = src2[-1]; // SW
      m_lfCccmSamples[8] = src2[1]; // SE
      break;
    default:
      THROW("modeltype not supported");
      break;
    }
    TCccmCoeff tmpCb = 0;
    TCccmCoeff tmpCr = 0;
    for (int i = 0; i < m_lfCccmModelSize; i++)
    {
      tmpCb += modelCb.params[i] * m_lfCccmSamples[i];
      tmpCr += modelCr.params[i] * m_lfCccmSamples[i];
    }
    outCb = Pel((tmpCb + modelCb.decimRound) >> (modelCb.decimBits));
    outCr = Pel((tmpCr + modelCr.decimRound) >> (modelCr.decimBits));
    outCb = ClipPel(outCb, m_lfCccmClpRngCb);
    outCr = ClipPel(outCr, m_lfCccmClpRngCr);
  }

  void lfCccmCollectData2(const Pel *src0, const Pel *src1, const Pel *src2, int numSamples, const int nx)
  {
    switch (m_lfCccmModelType)
    {
    case 0:
      std::memcpy(m_lfCccmA[0] + numSamples, src0, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[1] + numSamples, src1, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[2] + numSamples, src2, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[3] + numSamples, src0 - 1, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[4] + numSamples, src0 + 1, nx * sizeof(Pel));
      break;
    case 1:
      std::memcpy(m_lfCccmA[0] + numSamples, src0, nx * sizeof(Pel));
      break;
    case 2:
      std::memcpy(m_lfCccmA[0] + numSamples, src0, nx * sizeof(Pel));
      break;
    case 3:
      std::memcpy(m_lfCccmA[0] + numSamples, src0, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[1] + numSamples, src1, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[2] + numSamples, src2, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[3] + numSamples, src0 - 1, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[4] + numSamples, src0 + 1, nx * sizeof(Pel));
      break;
    case 4:
      std::memcpy(m_lfCccmA[0] + numSamples, src0, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[1] + numSamples, src0 - 1, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[2] + numSamples, src0 + 1, nx * sizeof(Pel));
      break;
    case 5:
      std::memcpy(m_lfCccmA[0] + numSamples, src0, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[1] + numSamples, src1, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[2] + numSamples, src2, nx * sizeof(Pel));
      break;
    case 6:
      std::memcpy(m_lfCccmA[0] + numSamples, src0, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[1] + numSamples, src1, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[2] + numSamples, src2, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[3] + numSamples, src0 - 1, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[4] + numSamples, src0 + 1, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[5] + numSamples, src1 - 1, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[6] + numSamples, src1 + 1, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[7] + numSamples, src2 - 1, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[8] + numSamples, src2 + 1, nx * sizeof(Pel));
      break;
    case 7:
      std::memcpy(m_lfCccmA[0] + numSamples, src0, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[1] + numSamples, src1, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[2] + numSamples, src2, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[3] + numSamples, src0 - 1, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[4] + numSamples, src0 + 1, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[5] + numSamples, src1 - 1, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[6] + numSamples, src1 + 1, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[7] + numSamples, src2 - 1, nx * sizeof(Pel));
      std::memcpy(m_lfCccmA[8] + numSamples, src2 + 1, nx * sizeof(Pel));
      break;
    default:
      THROW("modeltype not supported");
      break;
    }
  };

  void lfCccmCollectData(const Pel *src0, const Pel *src1, const Pel *src2, int numSamples)
  {
    switch (m_lfCccmModelType)
    {
    case 0:
      m_lfCccmA[0][numSamples] = src0[0]; // C
      m_lfCccmA[1][numSamples] = src1[0]; // N
      m_lfCccmA[2][numSamples] = src2[0]; // S
      m_lfCccmA[3][numSamples] = src0[-1]; // W
      m_lfCccmA[4][numSamples] = src0[1]; // E
      m_lfCccmA[5][numSamples] = m_lfCccmModelCb.nonlinear(src0[0]);
      m_lfCccmA[6][numSamples] = m_lfCccmLumaOffset;
      break;
    case 1:
      m_lfCccmA[0][numSamples] = src0[0]; // C
      m_lfCccmA[1][numSamples] = m_lfCccmLumaOffset;
      break;
    case 2:
      m_lfCccmA[0][numSamples] = src0[0]; // C
      m_lfCccmA[1][numSamples] = m_lfCccmModelCb.nonlinear(src0[0]);
      m_lfCccmA[2][numSamples] = m_lfCccmLumaOffset;
      break;
    case 3:
      m_lfCccmA[0][numSamples] = src0[0]; // C
      m_lfCccmA[1][numSamples] = src1[0]; // N
      m_lfCccmA[2][numSamples] = src2[0]; // S
      m_lfCccmA[3][numSamples] = src0[-1]; // W
      m_lfCccmA[4][numSamples] = src0[1]; // E
      m_lfCccmA[5][numSamples] = m_lfCccmLumaOffset;
      break;
    case 4:
      m_lfCccmA[0][numSamples] = src0[0]; // C
      m_lfCccmA[1][numSamples] = src0[-1]; // W
      m_lfCccmA[2][numSamples] = src0[1]; // E
      m_lfCccmA[3][numSamples] = m_lfCccmModelCb.nonlinear(src0[0]);
      m_lfCccmA[4][numSamples] = m_lfCccmLumaOffset;
      break;
    case 5:
      m_lfCccmA[0][numSamples] = src0[0]; // C
      m_lfCccmA[1][numSamples] = src1[0]; // N
      m_lfCccmA[2][numSamples] = src2[0]; // S
      m_lfCccmA[3][numSamples] = m_lfCccmModelCb.nonlinear(src0[0]);
      m_lfCccmA[4][numSamples] = m_lfCccmLumaOffset;
      break;
    case 6:
      m_lfCccmA[0][numSamples]  = src0[0]; // C
      m_lfCccmA[1][numSamples]  = src1[0]; // N
      m_lfCccmA[2][numSamples]  = src2[0]; // S
      m_lfCccmA[3][numSamples]  = src0[-1]; // W
      m_lfCccmA[4][numSamples]  = src0[1]; // E
      m_lfCccmA[5][numSamples]  = src1[-1]; // NW
      m_lfCccmA[6][numSamples]  = src1[1]; // NE
      m_lfCccmA[7][numSamples]  = src2[-1]; // SW
      m_lfCccmA[8][numSamples]  = src2[1]; // SE
      m_lfCccmA[9][numSamples]  = m_lfCccmModelCb.nonlinear(src0[0]);
      m_lfCccmA[10][numSamples] = m_lfCccmLumaOffset;
      break;
    case 7:
      m_lfCccmA[0][numSamples] = src0[0]; // C
      m_lfCccmA[1][numSamples] = src1[0]; // N
      m_lfCccmA[2][numSamples] = src2[0]; // S
      m_lfCccmA[3][numSamples] = src0[-1]; // W
      m_lfCccmA[4][numSamples] = src0[1]; // E
      m_lfCccmA[5][numSamples] = src1[-1]; // NW
      m_lfCccmA[6][numSamples] = src1[1]; // NE
      m_lfCccmA[7][numSamples] = src2[-1]; // SW
      m_lfCccmA[8][numSamples] = src2[1]; // SE
      m_lfCccmA[9][numSamples] = m_lfCccmLumaOffset;
      break;
    default:
      THROW("modeltype not supported");
      break;
    }
  };

  Pel lfCccmDownsample(const Pel *piSrc, const int iRecStride, const int x, const int y) const
  {
    const int offLeft = x > 0 ? -1 : 0;
    int       s       = 4;
    s += piSrc[2 * x + iRecStride * y * 2] * 2;
    s += piSrc[2 * x + offLeft + iRecStride * y * 2];
    s += piSrc[2 * x + 1 + iRecStride * y * 2];
    s += piSrc[2 * x + iRecStride * (y * 2 + 1)] * 2;
    s += piSrc[2 * x + offLeft + iRecStride * (y * 2 + 1)];
    s += piSrc[2 * x + 1 + iRecStride * (y * 2 + 1)];
    return (s >> 3) - m_lfCccmLumaOffset;
  }

  void (*m_lfCccmCalcStats)(const PelBuf &saoY, const CPelBuf &recoCb, const CPelBuf &recoCr, CccmModel &cccmModelCb,
                            const Area partitionArea, std::vector<std::vector<int32_t>> &xCorr,
                            std::vector<std::vector<std::vector<int32_t>>> &aCorr, Pel *samples);
  static void lfCccmCalcStats(const PelBuf &saoY, const CPelBuf &recoCb, const CPelBuf &recoCr, CccmModel &cccmModelCb,
                              const Area partitionArea, std::vector<std::vector<int32_t>> &xCorr,
                              std::vector<std::vector<std::vector<int32_t>>> &aCorr, Pel *samples);

  void lfCccmSumStatisticsEncoder(int &numSamples, const Pel *src00, const Pel *srcCb, const Pel *srcCr,
                                  const int strideY0, const int strideCb, const int strideCr);

  void lfCccmSumStatisticsDecoder(int &numSamples, const Pel *src0, const Pel *src1, const Pel *src2, const Pel *srcCb,
                                  const Pel *srcCr, const int stride, const int strideCb, const int strideCr);

  void lfCccmSetWindows(const int xi, const int yi);

  void lfCccmWindowProcess();
  void lfCccmFiltersConvolution();
};
#endif
