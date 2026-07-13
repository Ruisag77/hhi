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

/** \file     Prediction.cpp
    \brief    prediction class
*/

#include "IntraPrediction.h"

#include "Unit.h"
#include "UnitTools.h"
#include "Buffer.h"

#include "dtrace_next.h"
#include "dtrace_buffer.h"
#include "Rom.h"

#include <memory.h>

#include "CommonLib/InterpolationFilter.h"
//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Tables
// ====================================================================================================================

const uint8_t IntraPrediction::aucIntraFilter[MAX_INTRA_FILTER_DEPTHS] = {
  24, //   1xn
  24, //   2xn
  24, //   4xn
  14, //   8xn
  2,  //  16xn
  0,  //  32xn
  0,  //  64xn
  0   // 128xn
};

const uint8_t IntraPrediction::aucIntraFilterExt[MAX_INTRA_FILTER_DEPTHS] = {
  48,   //   1xn
  48,   //   2xn
  48,   //   4xn
  28,   //   8xn
  4,    //  16xn
  0,    //  32xn
  0,    //  64xn
  0     // 128xn
};

// ====================================================================================================================
// Function macros for CCCM
// ====================================================================================================================

#define FIXED_MULT(x, y) TCccmCoeff((int64_t(x) * (y) + CCCM_DECIM_ROUND) >> CCCM_DECIM_BITS)
#define FIXED_DIV(x, y)  TCccmCoeff((int64_t(x) << CCCM_DECIM_BITS) / (y))

void IntraPredAngleCpy_Core(const Pel *refMain, Pel *pDsty, const int width, const int height,
                            const ptrdiff_t dstStride, const int intraPredAngle, const int multiRefIdx, bool isExt)
{
  int x, y, deltaPos;
  for (y = 0, deltaPos = intraPredAngle * (1 + multiRefIdx); y < height;
       y++, deltaPos += intraPredAngle, pDsty += dstStride)
  {
    const int deltaInt = isExt ? deltaPos >> 6 : deltaPos >> 5;
    // Just copy the integer samples
    for (x = 0; x < width; x++)
    {
      pDsty[x] = refMain[x + deltaInt + 1];
    }
  }
}

void IntraAnglePDPC_Core(Pel *pDsty, const ptrdiff_t dstStride, const Pel *refSide, const int width, const int height,
                         const int scale, const int absInvAngle)
{
  for (int y = 0; y < height; y++, pDsty += dstStride)
  {
    int invAngleSum = 256;
    for (int x = 0; x < std::min(3 << scale, width); x++)
    {
      invAngleSum += absInvAngle;
      int wL   = 32 >> (2 * x >> scale);
      Pel left = refSide[y + (invAngleSum >> 9) + 1];
      pDsty[x] = pDsty[x] + ((wL * (left - pDsty[x]) + 32) >> 6);
    }
  }
}

void IntraAngleGradPDPC_Core(Pel *pDsty, const ptrdiff_t dstStride, const Pel *refSide, const Pel *refMain,
                             const int width, const int height, int scale, const int intraPredAngle,
                             const int multiRefIdx, const ClpRng &clpRng, bool useExt)
{
  for (int y = 0, deltaPos = intraPredAngle * (1 + multiRefIdx); y < height;
       y++, deltaPos += intraPredAngle, pDsty += dstStride)
  {
    const int deltaInt   = useExt ? deltaPos >> 6 : deltaPos >> 5;
    const int deltaFract = useExt ? deltaPos & 63 : deltaPos & 31;
    const Pel left       = refSide[1 + y];
    int       gradOffset = useExt ? 32 : 16;
    int       gradShift  = useExt ? 6 : 5;
    const Pel topLeft =
      refMain[deltaInt] + ((deltaFract * (refMain[deltaInt + 1] - refMain[deltaInt]) + gradOffset) >> gradShift);

    for (int x = 0; x < std::min(3 << scale, width); x++)
    {
      int       wL  = 32 >> (2 * x >> scale);
      const Pel val = pDsty[x];
      pDsty[x]      = ClipPel(val + ((wL * (left - topLeft) + 32) >> 6), clpRng);
    }
  }
}

void IntraPredAngleLuma_Core(const Pel *refMain, Pel *pDsty, const int width, const int height,
                             const ptrdiff_t dstStride, const int intraPredAngle, const int multiRefIdx,
                             const ClpRng &clpRng, const bool interpolationFlag, bool useExt)
{
  for (int y = 0, deltaPos = intraPredAngle * (1 + multiRefIdx); y < height;
       y++, deltaPos += intraPredAngle, pDsty += dstStride)
  {

    const int deltaInt   = useExt ? deltaPos >> 6 : deltaPos >> 5;
    const int deltaFract = useExt ? deltaPos & 63 : deltaPos & 31;

    const bool useCubicFilter = !interpolationFlag;

     // 4-tap Gaussian.
    const TFilterCoeff intraSmoothingFilter[6]  = { TFilterCoeff(0),
                                                    TFilterCoeff(64 - (deltaFract << 1)),
                                                    TFilterCoeff(128 - (deltaFract << 1)),
                                                    TFilterCoeff(64 + (deltaFract << 1)),
                                                    TFilterCoeff(deltaFract << 1),
                                                    TFilterCoeff(0) };
    // 6-tap Gaussian (for larger blocks).
    const TFilterCoeff intraSmoothingFilter2[6] = {
      TFilterCoeff(16 - (deltaFract >> 1)), TFilterCoeff(64 - 3 * (deltaFract >> 1)), TFilterCoeff(96 - (deltaFract)),
      TFilterCoeff(64 + (deltaFract)),      TFilterCoeff(16 + 3 * (deltaFract >> 1)), TFilterCoeff((deltaFract >> 1))
    };

    const TFilterCoeff intraSmoothingFilterExt[6]  = { TFilterCoeff(0),
                                                       TFilterCoeff(64 - (deltaFract)),
                                                       TFilterCoeff(128 - (deltaFract)),
                                                       TFilterCoeff(64 + (deltaFract)),
                                                       TFilterCoeff(deltaFract),
                                                       TFilterCoeff(0) };
    const TFilterCoeff intraSmoothingFilter2Ext[6] = {
      TFilterCoeff(16 - (deltaFract >> 2)),     TFilterCoeff(64 - 3 * (deltaFract >> 2)),
      TFilterCoeff(96 - (deltaFract >> 1)),     TFilterCoeff(64 + (deltaFract >> 1)),
      TFilterCoeff(16 + 3 * (deltaFract >> 2)), TFilterCoeff((deltaFract >> 2))
    };
    const TFilterCoeff *const f = (useCubicFilter)
      ? (useExt ? InterpolationFilter::getIntraLumaFilterTableExt(deltaFract)
                : InterpolationFilter::getIntraLumaFilterTable(deltaFract))
      : ((width >= 32 && height >= 32) ? (useExt ? intraSmoothingFilter2Ext : intraSmoothingFilter2)
                                       : (useExt ? intraSmoothingFilterExt : intraSmoothingFilter));

    // TODO: Check if correct if IF_12TAP==0

    for (int x = 0; x < width; x++)
    {
      Pel p[6];
      p[0] = refMain[deltaInt + x - 1];
      p[1] = refMain[deltaInt + x];
      p[2] = refMain[deltaInt + x + 1];
      p[3] = refMain[deltaInt + x + 2];
      p[4] = refMain[deltaInt + x + 3];
      p[5] = refMain[deltaInt + x + 4];

      Pel val  = (f[0] * p[0] + f[1] * p[1] + f[2] * p[2] + f[3] * p[3] + f[4] * p[4] + f[5] * p[5] + 128) >> 8;
      pDsty[x] = ClipPel(val, clpRng);  // always clip even though not always needed
    }
  }
}
void IntraPredAngleChroma_Core(const Pel *refMain, Pel *pDsty, const int width, const int height,
                               const ptrdiff_t dstStride, const int intraPredAngle, const int multiRefIdx)
{
  for (int y = 0, deltaPos = intraPredAngle * (1 + multiRefIdx); y < height;
       y++, deltaPos += intraPredAngle, pDsty += dstStride)
  {
    const int deltaInt   = deltaPos >> 5;
    const int deltaFract = deltaPos & 31;

    // Do linear filtering
    for (int x = 0; x < width; x++)
    {
      Pel p[2];

      p[0] = refMain[deltaInt + x + 1];
      p[1] = refMain[deltaInt + x + 2];

      pDsty[x] = p[0] + ((deltaFract * (p[1] - p[0]) + 16) >> 5);
    }
  }
}

void IntraHorVerPDPC_Core(Pel *pDsty, const ptrdiff_t dstStride, Pel *refSide, const int width, const int height,
                          int scale, const Pel *refMain, const ClpRng &clpRng)
{
  const Pel topLeft = refMain[0];

  for (int y = 0; y < height; y++)
  {
    memcpy(pDsty, &refMain[1], width * sizeof(Pel));
    const Pel left = refSide[1 + y];
    for (int x = 0; x < std::min(3 << scale, width); x++)
    {
      const int wL  = 32 >> (2 * x >> scale);
      const Pel val = pDsty[x];
      pDsty[x]      = ClipPel(val + ((wL * (left - topLeft) + 32) >> 6), clpRng);
    }
    pDsty += dstStride;
  }
}

/** Function for deriving planar intra prediction. This function derives the prediction samples for planar mode (intra
 * coding).
 */

// NOTE: Bit-Limit - 24-bit source
void PredIntraPlanar_Core(const CPelBuf &pSrc, PelBuf &pDst, const PlanarDirType &plDir)
{
  const uint32_t width  = pDst.width;
  const uint32_t height = pDst.height;
  const uint32_t log2W  = floorLog2(width);
  const uint32_t log2H  = floorLog2(height);

  int leftColumn[MAX_CU_SIZE + 1], topRow[MAX_CU_SIZE + 1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];

  // Get left and above reference column and row
  CHECK(width > MAX_CU_SIZE, "width greater than limit");
  for (int k = 0; k < width + 1; k++)
  {
    topRow[k] = pSrc.at(k + 1, 0);
  }

  CHECK(height > MAX_CU_SIZE, "height greater than limit");
  for (int k = 0; k < height + 1; k++)
  {
    leftColumn[k] = pSrc.at(k + 1, 1);
  }
  if (plDir == PlanarDirType::NO_DIR)   // original planar
  {
    // Prepare intermediate variables used in interpolation
    const uint32_t offset     = 1 << (log2W + log2H);
    int            bottomLeft = leftColumn[height];
    int            topRight   = topRow[width];
    for (int k = 0; k < width; k++)
    {
      bottomRow[k] = bottomLeft - topRow[k];
      topRow[k]    = topRow[k] << log2H;
    }

    for (int k = 0; k < height; k++)
    {
      rightColumn[k] = topRight - leftColumn[k];
      leftColumn[k]  = leftColumn[k] << log2W;
    }

    const uint32_t  finalShift = 1 + log2W + log2H;
    const ptrdiff_t stride     = pDst.stride;
    Pel            *pred       = pDst.buf;
    for (int y = 0; y < height; y++, pred += stride)
    {
      int horPred = leftColumn[y];

      for (int x = 0; x < width; x++)
      {
        horPred += rightColumn[y];
        topRow[x] += bottomRow[x];

        int vertPred = topRow[x];
        pred[x]      = ((horPred << log2H) + (vertPred << log2W) + offset) >> finalShift;
      }
    }
  }
  else if (plDir == PlanarDirType::HOR)   // planar hor
  {
    // Prepare intermediate variables used in interpolation
    int topRight = topRow[width];
    for (int k = 0; k < height; k++)
    {
      rightColumn[k] = topRight - leftColumn[k];
      leftColumn[k]  = leftColumn[k] << log2W;
    }

    const ptrdiff_t stride = pDst.stride;
    Pel            *pred   = pDst.buf;
    int             horPred;
    for (int y = 0; y < height; y++, pred += stride)
    {
      horPred = leftColumn[y];
      for (int x = 0; x < width; x++)
      {
        horPred += rightColumn[y];
        pred[x] = (horPred + (1 << (log2W - 1))) >> log2W;
      }
    }
  }
  else   // planar ver
  {
    CHECK(plDir != PlanarDirType::VER, "error, wrong planar dir");
    // Prepare intermediate variables used in interpolation
    int bottomLeft = leftColumn[height];
    for (int k = 0; k < width; k++)
    {
      bottomRow[k] = bottomLeft - topRow[k];
      topRow[k]    = topRow[k] << log2H;
    }

    const ptrdiff_t stride = pDst.stride;
    Pel            *pred   = pDst.buf;
    int             vertPred;
    for (int y = 0; y < height; y++, pred += stride)
    {
      for (int x = 0; x < width; x++)
      {
        topRow[x] += bottomRow[x];
        vertPred = topRow[x];
        pred[x]  = (vertPred + (1 << (log2H - 1))) >> log2H;
      }
    }
  }
}

void IntraPredSampleFilter_Core(const CPelBuf &srcBuf, PelBuf &dstBuf)
{
  const int width  = dstBuf.width;
  const int height = dstBuf.height;
  const int scale  = ((floorLog2(width) - 2 + floorLog2(height) - 2 + 2) >> 2);
  CHECK(scale < 0 || scale > 31, "PDPC: scale < 0 || scale > 31");

  for (int y = 0; y < height; y++)
  {
    const int wT   = 32 >> std::min(31, ((y << 1) >> scale));
    const Pel left = srcBuf.at(y + 1, 1);
    for (int x = 0; x < width; x++)
    {
      const int wL    = 32 >> std::min(31, ((x << 1) >> scale));
      const Pel top   = srcBuf.at(x + 1, 0);
      const Pel val   = dstBuf.at(x, y);
      dstBuf.at(x, y) = val + ((wL * (left - val) + wT * (top - val) + 32) >> 6);
    }
  }
}

bool predIntraOpt_Core(PelBuf &pDst, const CodingUnit &cu, const uint32_t modeIdx, const ClpRng &clpRng, Pel *refF,
                       Pel *refS)
{
  const uint32_t width   = pDst.width;
  const uint32_t height  = pDst.height;
  const int      sizeKey = (width << 8) + height;
  const int      sizeIdx = g_size.find(sizeKey) != g_size.end() ? g_size[sizeKey] : -1;

  if (sizeIdx < 0)
  {
    return false;
  }
  if (!cu.mipFlag && (sizeIdx < 0 || (sizeIdx > 12 && modeIdx > 1 && (modeIdx % 4 != 2))))
  {
    return false;
  }

  const int stride   = (int)pDst.stride;
  Pel      *pred     = pDst.buf;
  const int xShift   = g_sizeData[sizeIdx][8];
  const int yShift   = g_sizeData[sizeIdx][9];
  bool      shortLen = modeIdx < PDP_SHORT_TH[0] || (modeIdx >= PDP_SHORT_TH[1] && modeIdx <= PDP_SHORT_TH[2]);
  const int refLen   = shortLen ? g_sizeData[sizeIdx][10] : g_sizeData[sizeIdx][7];
  auto      ref      = shortLen ? refS : refF;
  int16_t  *filter =
    (cu.mipFlag ? (cu.mipTransposedFlag ? g_pdpFiltersMip[modeIdx + 16][sizeIdx] : g_pdpFiltersMip[modeIdx][sizeIdx])
                : g_pdpFilters[modeIdx][sizeIdx]);
  const int addShift = 1 << 13;
  auto      len4     = (((refLen + 7) / 8) * 8) * 4;

  for (int y = 0; y < height; y++, pred += stride)
  {
    for (int x = 0; x < width; x++)
    {
      int            sum = 0;
      const int16_t *f   = &filter[(y >> yShift) * width / 4 * len4 + ((x >> xShift) >> 2) * len4];
      for (int i = 0; i < refLen; ++i)
      {
        auto xs = x >> xShift;
        sum += ref[i] * f[((i >> 3) << 5) + ((xs & 3) << 3) + (i & 7)];
      }
      pred[x] = (Pel)Clip3(clpRng.min, clpRng.max, (std::max(0, sum) + addShift) >> 14);
    }
  }

  if (sizeIdx > 12 && !cu.mipFlag)
  {
    int  sampFacHor = pDst.width / 16;
    int  sampFacVer = pDst.height / 16;
    int  numMRLLeft = g_sizeData[sizeIdx][5];
    int  numMRLTop  = g_sizeData[sizeIdx][6];
    int  strideDst  = (pDst.width << 1) + numMRLLeft; // fetching from g_ref always.
    Pel *ptrSrc     = refF + (strideDst * numMRLTop) + numMRLLeft - 1;
    Pel *ref2       = refF + (strideDst * (numMRLTop - 1)) + numMRLLeft;
    pred            = pDst.buf;

    for (int y = 0; y < pDst.height; y++)
    {
      if (0 != (sampFacHor - 1) && ((y & (sampFacVer - 1)) == (sampFacVer - 1)))
      {
        pred[0] = (ptrSrc[0] + pred[1] + 1) >> 1;
      }
      for (int x = 1; x < pDst.width; x++)
      {
        if (((x & (sampFacHor - 1)) != (sampFacHor - 1)) && ((y & (sampFacVer - 1)) == (sampFacVer - 1)))
        {
          pred[x] = (pred[x - 1] + pred[x + 1] + 1) >> 1;
        }
      }
      pred += stride;
      ptrSrc += numMRLLeft;
    }

    pred       = pDst.buf;
    Pel *pred1 = pred + stride;
    if (0 != (sampFacVer - 1))
    {
      for (int x = 0; x < pDst.width; x++)
      {
        pred[x] = (ref2[x] + pred1[x] + 1) >> 1;
      }
    }

    pred += stride;
    Pel *pred0 = pred - stride;
    pred1      = pred + stride;

    for (int y = 1; y < pDst.height; y++)
    {
      if (((y & (sampFacVer - 1)) != (sampFacVer - 1)))
      {
        for (int x = 0; x < pDst.width; x++)
        {
          pred[x] = (pred0[x] + pred1[x] + 1) >> 1;
        }
      }
      pred += stride;
      pred0 += stride;
      pred1 += stride;
    }
  }
  return true;
}

void calculateCorrelationMatrix_Core(int nRows, int nCols, int nSamples, const int16_t *input[], int64_t *output,
                                     int outputStride)
{
  for (int y = 0; y < nRows; ++y)
  {
    for (int x = y; x < nCols; ++x)
    {
      auto &sum = output[x + y * outputStride];
      sum       = 0;
      for (int i = 0; i < nSamples; ++i)
      {
        sum += input[x][i] * input[y][i];
      }
    }
  }
}

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

IntraPrediction::IntraPrediction() : m_currChromaFormat(ChromaFormat::UNDEFINED)
{
  for (uint32_t ch = 0; ch < MAX_NUM_COMP; ch++)
  {
    for (uint32_t buf = 0; buf < 4; buf++)
    {
      m_yuvExt2[ch][buf] = nullptr;
    }
  }

  for (int i = 0; i < CCCM_NUM_LUMA_BUFS; i++)
  {
    m_cccmLumaBuf[i] = nullptr;
  }
  for (int i = 0; i < NUM_BVG_CCCM_CANDS; i++)
  {
    m_bvgCccmLumaBuf[i]      = nullptr;
    m_bvgCccmChromaBuf[i][0] = nullptr;
    m_bvgCccmChromaBuf[i][1] = nullptr;
  }

  for (int i = 0; i < MAX_CCP_CAND_LIST_SIZE; i++)
  {
    m_ccTemplPredCb[i] = nullptr;
    m_ccTemplPredCr[i] = nullptr;
  }

  m_piTemp         = nullptr;
  m_refMatrixA     = nullptr;
  m_targetVectorCb = nullptr;
  m_targetVectorCr = nullptr;

  std::memset(m_ref, 0, sizeof(m_ref));
  std::memset(m_refShort, 0, sizeof(m_refShort));
  m_refAvailable = false;

  m_IntraPredAngleCpy          = IntraPredAngleCpy_Core;
  m_IntraPredAngleChroma       = IntraPredAngleChroma_Core;
  m_IntraPredAngleLuma         = IntraPredAngleLuma_Core;
  m_IntraAnglePDPC             = IntraAnglePDPC_Core;
  m_IntraAngleGradPDPC         = IntraAngleGradPDPC_Core;
  m_IntraHorVerPDPC            = IntraHorVerPDPC_Core;
  m_PredIntraPlanar            = PredIntraPlanar_Core;
  m_IntraPredSampleFilter      = IntraPredSampleFilter_Core;
  m_calculateCorrelationMatrix = calculateCorrelationMatrix_Core;
  m_xPredIntraOpt              = predIntraOpt_Core;

#if ENABLE_SIMD_OPT_INTRAPRED
#ifdef TARGET_SIMD_X86
  initIntraPredictionX86();
#endif
#endif
}

IntraPrediction::~IntraPrediction() { destroy(); }

void IntraPrediction::destroy()
{
  for (uint32_t ch = 0; ch < MAX_NUM_COMP; ch++)
  {
    for (uint32_t buf = 0; buf < 4; buf++)
    {
      delete[] m_yuvExt2[ch][buf];
      m_yuvExt2[ch][buf] = nullptr;
    }
  }

  for (int i = 0; i < CCCM_NUM_LUMA_BUFS; i++)
  {
    delete[] m_cccmLumaBuf[i];
    m_cccmLumaBuf[i] = nullptr;
  }

  for (int i = 0; i < NUM_BVG_CCCM_CANDS; i++)
  {
    delete[] m_bvgCccmLumaBuf[i];
    m_bvgCccmLumaBuf[i] = nullptr;
    delete[] m_bvgCccmChromaBuf[i][0];
    m_bvgCccmChromaBuf[i][0] = nullptr;
    delete[] m_bvgCccmChromaBuf[i][1];
    m_bvgCccmChromaBuf[i][1] = nullptr;
  }

  for (int i = 0; i < MAX_CCP_CAND_LIST_SIZE; i++)
  {
    delete[] m_ccTemplPredCb[i];
    m_ccTemplPredCb[i] = nullptr;
    delete[] m_ccTemplPredCr[i];
    m_ccTemplPredCr[i] = nullptr;
  }

  delete[] m_piTemp;
  m_piTemp = nullptr;
  delete[] m_refMatrixA;
  m_refMatrixA = nullptr;
  delete[] m_targetVectorCb;
  m_targetVectorCb = nullptr;
  delete[] m_targetVectorCr;
  m_targetVectorCr = nullptr;
  if (m_timdSatdCost != nullptr)
  {
    delete m_timdSatdCost;
    m_timdSatdCost = nullptr;
  }
  for (int i = 0; i < NUM_EIP_MODELS; i++)
  {
    if (m_eipRefMatrixA[i] != nullptr)
    {
      delete[] m_eipRefMatrixA[i];
      m_eipRefMatrixA[i] = nullptr;
    }
    if (m_eipTargetVectorY[i] != nullptr)
    {
      delete[] m_eipTargetVectorY[i];
      m_eipTargetVectorY[i] = nullptr;
    }
  }
  for (auto &buffer: m_tempBufferDIMD)
  {
    buffer.destroy();
  }
  m_tempBufferDIMD.clear();
  m_sgpmBuffer.destroy();
}

void IntraPrediction::init(ChromaFormat chromaFormatIdc, const unsigned bitDepthY, InterpolationFilter *pIf)
{
  if (m_yuvExt2[COMP_Y][0] != nullptr && m_currChromaFormat != chromaFormatIdc)
  {
    destroy();
  }

  m_currChromaFormat = chromaFormatIdc;
  m_pIf              = pIf;

  if (m_yuvExt2[COMP_Y][0] == nullptr) // check if first is null (in which case, nothing initialised yet)
  {
    m_yuvExtSize2 = (MAX_CU_SIZE) * (MAX_CU_SIZE);

    for (uint32_t ch = 0; ch < MAX_NUM_COMP; ch++)
    {
      for (uint32_t buf = 0; buf < 4; buf++)
      {
        m_yuvExt2[ch][buf] = new Pel[m_yuvExtSize2];
      }
    }
  }

  int shift = bitDepthY + 4;
  for (int i = 32; i < 64; i++)
  {
    m_auShiftLM[i - 32] = ((1 << shift) + i / 2) / i;
  }

  for (int i = 0; i < MAX_CCP_CAND_LIST_SIZE; i++)
  {
    if (m_ccTemplPredCb[i] == nullptr)
    {
      m_ccTemplPredCb[i] = new Pel[2 * MAX_CU_SIZE];
    }
    if (m_ccTemplPredCr[i] == nullptr)
    {
      m_ccTemplPredCr[i] = new Pel[2 * MAX_CU_SIZE];
    }
  }

  if (m_piTemp == nullptr)
  {
    m_piTemp =
      new Pel[(2 * MAX_CU_SIZE + 1) * (2 * MAX_CU_SIZE + 1)];// MDLM will use top-above and left-below samples.
  }

  for (int i = 0; i < CCCM_NUM_LUMA_BUFS; i++)
  {
    if (m_cccmLumaBuf[i] == nullptr)
    {
      m_cccmLumaBuf[i] = new Pel[(2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE + 2 * CCCM_FILTER_PADDING) *
                                 (2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE + 2 * CCCM_FILTER_PADDING)];
    }
  }

  for (int i = 0; i < NUM_BVG_CCCM_CANDS; i++)
  {
    if (m_bvgCccmLumaBuf[i] == nullptr)
    {
      m_bvgCccmLumaBuf[i] = new Pel[(2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE + 2 * CCCM_FILTER_PADDING) *
                                    (2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE + 2 * CCCM_FILTER_PADDING)];
    }
    if (m_bvgCccmChromaBuf[i][0] == nullptr)
    {
      m_bvgCccmChromaBuf[i][0] = new Pel[(2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE + 2 * CCCM_FILTER_PADDING) *
                                         (2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE + 2 * CCCM_FILTER_PADDING)];
    }
    if (m_bvgCccmChromaBuf[i][1] == nullptr)
    {
      m_bvgCccmChromaBuf[i][1] = new Pel[(2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE + 2 * CCCM_FILTER_PADDING) *
                                         (2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE + 2 * CCCM_FILTER_PADDING)];
    }
  }

  if (m_refMatrixA == nullptr)
  {
    m_refMatrixA = new Pel[CCCM_NUM_PARAMS_MAX * CCCM_MAX_REF_SAMPLES];
  }
  if (m_targetVectorCb == nullptr)
  {
    m_targetVectorCb = new Pel[CCCM_MAX_REF_SAMPLES];
  }
  if (m_targetVectorCr == nullptr)
  {
    m_targetVectorCr = new Pel[CCCM_MAX_REF_SAMPLES];
  }
  if (m_timdSatdCost == nullptr)
  {
    m_timdSatdCost = new RdCost;
  }
  for (int i = 0; i < NUM_EIP_MODELS; i++)
  {
    if (m_eipRefMatrixA[i] == nullptr)
    {
      m_eipRefMatrixA[i] = new Pel[EIP_FILTER_TAP * (5 * MAX_EIP_SIZE * MAX_EIP_SIZE)];
    }
    if (m_eipTargetVectorY[i] == nullptr)
    {
      m_eipTargetVectorY[i] = new Pel[5 * MAX_EIP_SIZE * MAX_EIP_SIZE];
    }
  }
  for (auto &buffer: m_tempBufferDIMD)
  {
    buffer.destroy();
  }
  m_tempBufferDIMD.resize(MAX_IPM_FUSION_NUM + 2);
  for (auto &buffer: m_tempBufferDIMD)
  {
    buffer.create(chromaFormatIdc, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE));
#if ENABLE_VALGRIND_CODE
    buffer.getBuf(COMP_Y).memset(0);
#endif
  }
  m_sgpmBuffer.destroy();
  m_sgpmBuffer.create(ChromaFormat::_400,
                      Area(0, 0, MAX_CU_SIZE + DIMD_MAX_TEMP_SIZE, MAX_CU_SIZE + DIMD_MAX_TEMP_SIZE));
}

inline bool isAboveLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT, int mrlIdx);
inline int  isAboveAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT,
                             const uint32_t numUnitsInPu, const uint32_t unitWidth, bool *validFlags, int mrlIdx);
inline int  isLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT,
                            const uint32_t numUnitsInPu, const uint32_t unitWidth, bool *validFlags, int mrlIdx);
inline int  isAboveRightAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posRT,
                                  const uint32_t numUnitsInPu, const uint32_t unitHeight, bool *validFlags, int mrlIdx);
inline int  isBelowLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLB,
                                 const uint32_t numUnitsInPu, const uint32_t unitHeight, bool *validFlags, int mrlIdx);

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// Function for calculating DC value of the reference samples used in Intra prediction
// NOTE: Bit-Limit - 25-bit source
Pel IntraPrediction::xGetPredValDc(const CPelBuf &pSrc, const Size &dstSize)
{
  CHECK(dstSize.width == 0 || dstSize.height == 0, "Empty area provided");

  int        idx, sum = 0;
  Pel        dcVal;
  const int  width     = dstSize.width;
  const int  height    = dstSize.height;
  const auto denom     = (width == height) ? (width << 1) : std::max(width, height);
  const auto divShift  = floorLog2(denom);
  const auto divOffset = (denom >> 1);

  if (width >= height)
  {
    for (idx = 0; idx < width; idx++)
    {
      sum += pSrc.at(m_ipaParam.multiRefIndex + 1 + idx, 0);
    }
  }
  if (width <= height)
  {
    for (idx = 0; idx < height; idx++)
    {
      sum += pSrc.at(m_ipaParam.multiRefIndex + 1 + idx, 1);
    }
  }

  dcVal = (sum + divOffset) >> divShift;
  return dcVal;
}

int IntraPrediction::getModifiedWideAngle(int width, int height, int predMode)
{
  // The function returns a 'modified' wide angle index, given that it is not necessary
  // in this software implementation to reserve the values 0 and 1 for Planar and DC to generate the prediction signal.
  // It should only be used to obtain the intraPredAngle parameter.
  // To simply obtain the wide angle index, the function PU::getWideAngle should be used instead.
  if (predMode > DC_IDX && predMode <= VDIA_IDX)
  {
    int modeShift[] = { 0, 6, 10, 12, 14, 15 };
    int deltaSize   = abs(floorLog2(width) - floorLog2(height));
    if (width > height && predMode < 2 + modeShift[deltaSize])
    {
      predMode += (VDIA_IDX - 1);
    }
    else if (height > width && predMode > VDIA_IDX - modeShift[deltaSize])
    {
      predMode -= (VDIA_IDX - 1);
    }
  }
  return predMode;
}

void IntraPrediction::setReferenceArrayLengths(const CompArea &area)
{
  // set Top and Left reference samples length
  const int width  = area.width;
  const int height = area.height;

  m_leftRefLength = (height << 1);
  m_topRefLength  = (width << 1);
}

int IntraPrediction::getWideAngleExt(int width, int height, int predMode, bool bSgpm)
{
  if (predMode > DC_IDX && predMode <= EXT_VDIA_IDX)
  {
    int modeShift[] = { 0, 11, 19, 23, 27, 29 };
    int deltaSize   = abs(floorLog2(width) - floorLog2(height));
    if (width > height && predMode < 2 + modeShift[deltaSize])
    {
      predMode += bSgpm ? EXT_VDIA_IDX : (EXT_VDIA_IDX - 1);
    }
    else if (height > width && predMode > EXT_VDIA_IDX - modeShift[deltaSize])
    {
      predMode -= bSgpm ? EXT_VDIA_IDX : (EXT_VDIA_IDX - 1);
    }
  }
  return predMode;
}

bool IntraPrediction::predIntraAng(const CompID compId, PelBuf &piPred, const CodingUnit &cu, const bool applyPDPFilter,
                                   const bool useExt)
{
  const CompID      compID      = MAP_CHROMA(compId);
  const ChannelType channelType = toChannelType(compID);
  const int         width       = piPred.width;
  CHECK(PU::isMIP(cu, toChannelType(compId)), "We should not get here for MIP.");
  const uint32_t dirMode =
    cu.getBdpcmMode(compID) != BdpcmMode::NONE ? BDPCM_IDX : PU::getFinalIntraMode(cu, channelType);
  CHECK(floorLog2(width) < 2 && cu.cs->pcv->noChroma2x2, "Size not allowed");
  CHECK(floorLog2(width) > 7, "Size not allowed");

  const int srcStride  = m_refBufferStride[compID];
  const int srcHStride = 2;

  const CPelBuf &srcBuf = CPelBuf(getPredictorPtr(compID), srcStride, srcHStride);
  const ClpRng  &clpRng(cu.slice->clpRng(compID));
  CHECK(compID != COMP_Y && cu.slice->m_sps->m_dualITree && cu.slice->m_eSliceType == I_SLICE &&
          cu.plDir != PlanarDirType::NO_DIR,
        "Error, directional planar only allowed for luma");
  if (cu.ciipFlag != 0 && cu.ciipMode == CIIP_Type::WITH_PDPC)
  {
    // we generate a zero-signal prediction that will be filtered with PDPC
    piPred.fill(0);
  }
  else
  {
    if (cu.cs->sps->m_pdpEnabledFlag && applyPDPFilter && isLuma(compId) && dirMode != BDPCM_IDX &&
        cu.plDir == PlanarDirType::NO_DIR && !cu.sgpm && !cu.dimdFlag && !cu.timdFlag && !cu.multiRefIdx &&
        m_refAvailable && !(dirMode > 1 && (dirMode & 1)))
    {
      if (m_xPredIntraOpt(piPred, cu, dirMode, clpRng, m_ref, m_refShort))
      {
        return true;
      }
    }

    switch (dirMode)
    {
    case (PLANAR_IDX):
      m_PredIntraPlanar(srcBuf, piPred, isLuma(compID) ? cu.plDir : PlanarDirType::NO_DIR);
      break;
    case (DC_IDX):
      xPredIntraDc(srcBuf, piPred, channelType, false);
      break;
    case BDPCM_IDX:
      xPredIntraBDPCM(srcBuf, piPred, cu.getBdpcmMode(compID), clpRng);
      break;
    default:
      xPredIntraAng(srcBuf, piPred, channelType, clpRng, useExt);
      break;
    }
  }
  if (m_ipaParam.applyPDPC || (cu.ciipFlag != 0 && cu.ciipMode == CIIP_Type::WITH_PDPC))
  {
    if (dirMode == PLANAR_IDX || dirMode == DC_IDX)
    {
      PelBuf dstBuf = piPred;
      m_IntraPredSampleFilter(srcBuf, dstBuf);
    }
  }
  return false;
}

void IntraPrediction::xUpdateCclmModel(CclmModelSingle &model, int delta)
{
  if (delta)
  {
    const int dShift = 3;   // For 1/8 sample value adjustment

    delta = model.a > 0 ? -delta : delta;

    // Make final shift at least the size of the precision of the update
    if (model.shift < dShift)
    {
      model.a <<= (dShift - model.shift);
      model.shift = dShift;
    }
    else if (model.shift > dShift)
    {
      // Final shift is larger than the precision of the update: scale the update up to the final precision
      delta <<= (model.shift - dShift);
    }

    model.a += delta;
    model.b -= (delta * model.midLuma) >> model.shift;
  }
}

void IntraPrediction::predIntraChromaLM(const CompID compID, PelBuf &piPred, CodingUnit &cu, const CompArea &chromaArea,
                                        int intraDir, bool createModel)
{
  CHECK(createModel && cu.idxNonLocalCCP, "Cross-component merge should use already created models");

  const ptrdiff_t lumaStride = 2 * MAX_CU_SIZE + 1;

  PelBuf temp = PelBuf(m_piTemp + lumaStride + 1, lumaStride, Size(chromaArea));

  CclmModel cclmModel;

  CclmModelSingle &model0 = cclmModel.model[0];
  CclmModelSingle &model1 = cclmModel.model[1];

  if (createModel)
  {
    xGetLMParameters_LMS(cu, compID, chromaArea, cclmModel);

    xUpdateCclmModel(model0, compID == COMP_Cb ? cu.cclmOffsets.cb0 : cu.cclmOffsets.cr0);
    xUpdateCclmModel(model1, compID == COMP_Cb ? cu.cclmOffsets.cb1 : cu.cclmOffsets.cr1);

    // Store the model(s) (as a future cc-merge candidate or to be reused in RDO)
    cu.ccModels.setCclmModel(cclmModel, compID, PU::isMultiModeLM(cu.intraDir[ChannelType::CHROMA]));

    cu.ccModels.filter = cu.ccFilterFlag;
  }
  else
  {
    // Retrieve the already created model(s)
    cu.ccModels.getCclmModel(cclmModel, compID);
  }

  // final prediction
  piPred.copyFrom(temp);
  if (PU::isMultiModeLM(cu.intraDir[ChannelType::CHROMA]))
  {
    Pel      *pPred        = piPred.bufAt(0, 0);
    Pel      *pLuma        = temp.bufAt(0, 0);
    ptrdiff_t uiPredStride = piPred.stride;
    SizeType  uiCWidth     = chromaArea.width;
    SizeType  uiCHeight    = chromaArea.height;

    for (SizeType i = 0; i < uiCHeight; i++)
    {
      for (SizeType j = 0; j < uiCWidth; j++)
      {
        if (pLuma[j] <= cclmModel.multiModelLumaThr)
        {
          pPred[j] = (Pel)ClipPel(((model0.a * pLuma[j]) >> model0.shift) + model0.b, cu.cs->slice->clpRng(compID));
        }
        else
        {
          pPred[j] = (Pel)ClipPel(((model1.a * pLuma[j]) >> model1.shift) + model1.b, cu.cs->slice->clpRng(compID));
        }
      }
      pPred += uiPredStride;
      pLuma += lumaStride;
    }

    if (cu.ccModels.filter)
    {
      filterPredInside(compID, piPred, cu);
    }
  }
  else
  {
    piPred.linearTransform(model0.a, model0.shift, model0.b, true, cu.cs->slice->clpRng(compID));
  }
}

/** Function for deriving planar intra prediction. This function derives the prediction samples for planar mode (intra
 * coding).
 */

void IntraPrediction::xPredIntraDc(const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType,
                                   const bool enableBoundaryFilter)
{
  const Pel dcval = xGetPredValDc(pSrc, pDst);
  pDst.fill(dcval);
}

const int IntraPrediction::angTable[32]    = { 0,  1,  2,  3,  4,  6,  8,  10, 12, 14,  16,  18,  20,  23,  26,  29,
                                               32, 35, 39, 45, 51, 57, 64, 73, 86, 102, 128, 171, 256, 341, 512, 1024 };
const int IntraPrediction::invAngTable[32] = {
  0,   16384, 8192, 5461, 4096, 2731, 2048, 1638, 1365, 1170, 1024, 910, 819, 712, 630, 565,
  512, 468,   420,  364,  321,  287,  256,  224,  191,  161,  128,  96,  64,  48,  32,  16
};   // (512 * 32) / Angle

const int IntraPrediction::extAngTable[64]    = { 0,   1,   2,   3,   4,   5,   6,   7,   8,    10,   12,   14,  16,
                                                  18,  20,  22,  24,  26,  28,  30,  32,  34,   36,   38,   40,  43,
                                                  46,  49,  52,  55,  58,  61,  64,  67,  70,   74,   78,   84,  90,
                                                  96,  102, 108, 114, 121, 128, 137, 146, 159,  172,  188,  204, 230,
                                                  256, 299, 342, 427, 512, 597, 682, 853, 1024, 1536, 2048, 3072 };
const int IntraPrediction::extInvAngTable[64] = {
  0,    32768, 16384, 10923, 8192, 6554, 5461, 4681, 4096, 3277, 2731, 2341, 2048, 1820, 1638, 1489,
  1365, 1260,  1170,  1092,  1024, 964,  910,  862,  819,  762,  712,  669,  630,  596,  565,  537,
  512,  489,   468,   443,   420,  390,  364,  341,  321,  303,  287,  271,  256,  239,  224,  206,
  191,  174,   161,   142,   128,  110,  96,   77,   64,   55,   48,   38,   32,   21,   16,   11
};   // (512 * 64) / Angle

// Function for initialization of intra prediction parameters
void IntraPrediction::initPredIntraParams(const CodingUnit &cu, const CompArea area, const SPS &sps)
{
  const CompID      compId       = area.compID;
  const ChannelType chType       = toChannelType(compId);
  bool              bExtIntraDir = cu.timdFlag && isLuma(chType);
  const Size        puSize       = Size(area.width, area.height);
  const Size       &blockSize    = puSize;
  const int         dirMode      = PU::getFinalIntraMode(cu, chType);
  const int         predMode     = bExtIntraDir ? getWideAngleExt(blockSize.width, blockSize.height, dirMode, false)
                                                : getModifiedWideAngle(blockSize.width, blockSize.height, dirMode);
  m_ipaParam.isModeVer           = bExtIntraDir ? (predMode >= EXT_DIA_IDX) : (predMode >= DIA_IDX);

  m_ipaParam.multiRefIndex     = isLuma(chType) ? cu.multiRefIdx : 0;
  m_ipaParam.refFilterFlag     = false;
  m_ipaParam.interpolationFlag = false;
  m_ipaParam.applyPDPC =
    (puSize.width >= MIN_TB_SIZEY && puSize.height >= MIN_TB_SIZEY) && m_ipaParam.multiRefIndex == 0;

  const int intraPredAngleMode = (m_ipaParam.isModeVer) ? (predMode - (bExtIntraDir ? EXT_VER_IDX : VER_IDX))
                                                        : (-(predMode - (bExtIntraDir ? EXT_HOR_IDX : HOR_IDX)));
  int       absAng             = 0;

  if (dirMode > DC_IDX &&
      dirMode < (bExtIntraDir ? EXT_VDIA_IDX + 1 : NUM_LUMA_MODE))   // intraPredAngle for directional modes
  {
    const int absAngMode = abs(intraPredAngleMode);
    const int signAng    = intraPredAngleMode < 0 ? -1 : 1;

    absAng                 = bExtIntraDir ? extAngTable[absAngMode] : angTable[absAngMode];
    m_ipaParam.absInvAngle = bExtIntraDir ? extInvAngTable[absAngMode] : invAngTable[absAngMode];

    m_ipaParam.intraPredAngle = signAng * absAng;
    if (intraPredAngleMode < 0)
    {
      m_ipaParam.applyPDPC = false;
    }
    else if (intraPredAngleMode > 0)
    {
      const int sideSize = m_ipaParam.isModeVer ? puSize.height : puSize.width;
      const int maxScale = 2;

      m_ipaParam.useGradPDPC = false;
      m_ipaParam.angularScale =
        std::min(maxScale, floorLog2(sideSize) - (floorLog2(3 * m_ipaParam.absInvAngle - 2) - 8));
      if ((m_ipaParam.angularScale < 0) && (isLuma(compId)))
      {
        m_ipaParam.angularScale = (floorLog2(puSize.width) + floorLog2(puSize.height) - 2) >> 2;
        m_ipaParam.useGradPDPC  = true;
      }
      m_ipaParam.applyPDPC &= m_ipaParam.angularScale >= 0;
    }
  }

  // high level conditions and DC intra prediction
  if (isLuma(chType) && !PU::isMIP(cu, chType) && m_ipaParam.multiRefIndex == 0 && DC_IDX != dirMode &&
      cu.bdpcmMode[0] == BdpcmMode::NONE)
  {
    if (dirMode == PLANAR_IDX)   // Planar intra prediction
    {
      m_ipaParam.refFilterFlag = puSize.width * puSize.height > 32;
    }
    else
    {

      const int diff = std::min<int>(abs(predMode - (bExtIntraDir ? EXT_HOR_IDX : HOR_IDX)),
                                     abs(predMode - (bExtIntraDir ? EXT_VER_IDX : VER_IDX)));

      const int log2Size = (floorLog2(puSize.width) + floorLog2(puSize.height)) >> 1;
      CHECK(log2Size >= MAX_INTRA_FILTER_DEPTHS, "Size not supported");

      // Selelection of either ([1 2 1] / 4 ) refrence filter OR Gaussian 4-tap interpolation filter
      if (diff > (bExtIntraDir ? aucIntraFilterExt[log2Size] : aucIntraFilter[log2Size]))
      {
        const bool isRefFilter = bExtIntraDir ? isIntegerSlopeExt(absAng) : isIntegerSlope(absAng);
        CHECK(puSize.width * puSize.height <= 32,
              "DCT-IF interpolation filter is always used for 4x4, 4x8, and 8x4 luma CB");
        m_ipaParam.refFilterFlag     = isRefFilter;
        m_ipaParam.interpolationFlag = !isRefFilter;
      }
    }
  }
}

/** Function for deriving the simplified angular intra predictions.
 *
 * This function derives the prediction samples for the angular mode based on the prediction direction indicated by
 * the prediction mode index. The prediction direction is given by the displacement of the bottom row of the block and
 * the reference row above the block in the case of vertical prediction or displacement of the rightmost column
 * of the block and reference column left from the block in the case of the horizontal prediction. The displacement
 * is signalled at 1/32 pixel accuracy. When projection of the predicted pixel falls inbetween reference samples,
 * the predicted value for the pixel is linearly interpolated from the reference samples. All reference samples are
 * taken from the extended main reference.
 */
// NOTE: Bit-Limit - 25-bit source
void IntraPrediction::xPredIntraAng(const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType,
                                    const ClpRng &clpRng, const bool useExt)
{
  int width  = int(pDst.width);
  int height = int(pDst.height);

  const bool isModeVer      = m_ipaParam.isModeVer;
  const int  multiRefIdx    = m_ipaParam.multiRefIndex;
  const int  intraPredAngle = m_ipaParam.intraPredAngle;
  const int  absInvAngle    = m_ipaParam.absInvAngle;

  Pel *refMain;
  Pel *refSide;

  // 2 pixels more for 6 tap filter.
  Pel refAbove[2 * MAX_CU_SIZE + 5 + 33 * MAX_REF_LINE_IDX];
  Pel refLeft[2 * MAX_CU_SIZE + 5 + 33 * MAX_REF_LINE_IDX];

  // Initialize the Main and Left reference array.
  if (intraPredAngle < 0)
  {
    // x, y range increase by 1 (right extend).
    const Pel *src = pSrc.buf;
    Pel       *dst = refAbove + height + 1;
    ::memcpy(dst, src, sizeof(Pel) * (width + 2 + multiRefIdx + 1));

    src = pSrc.buf + pSrc.stride;
    dst = refLeft + width + 1;
    ::memcpy(dst, src, sizeof(Pel) * (height + 2 + multiRefIdx + 1));

    refMain = isModeVer ? refAbove + height + 1 : refLeft + width + 1;
    refSide = isModeVer ? refLeft + width + 1 : refAbove + height + 1;

    // Extend the Main reference to the left.
    int sizeSide = isModeVer ? height : width;

    // 4-tap filter for extended referenz.
    for (int k = -(sizeSide + 1); k <= -1; k++)
    {
      int frac32precision = (-k * absInvAngle + 8) >> 4;
      int intpel          = frac32precision >> 5;
      int fracpel         = frac32precision & 31;

      int leftMinus1 = refSide[Clip3(0, sizeSide + 2 + multiRefIdx, intpel - 1)];
      int left       = refSide[Clip3(0, sizeSide + 2 + multiRefIdx, intpel)];
      int right      = refSide[Clip3(0, sizeSide + 2 + multiRefIdx, intpel + 1)];
      int rightPlus1 = refSide[Clip3(0, sizeSide + 2 + multiRefIdx, intpel + 2)];

      const TFilterCoeff *f = InterpolationFilter::getWeak4TapFilterTable(fracpel);
      int val    = ((int)f[0] * leftMinus1 + (int)f[1] * left + (int)f[2] * right + f[3] * (int)rightPlus1 + 32) >> 6;
      refMain[k] = (Pel)ClipPel(val, clpRng);
    }
  }
  else
  {
    const Pel *src = pSrc.buf;
    Pel       *dst = refAbove + 1;
    ::memcpy(dst, src, sizeof(Pel) * (m_topRefLength + multiRefIdx + 1));

    src = pSrc.buf + pSrc.stride;
    dst = refLeft + 1;
    ::memcpy(dst, src, sizeof(Pel) * (m_leftRefLength + multiRefIdx + 1));

    // left extended by 1
    refAbove[0] = refAbove[1];
    refLeft[0]  = refLeft[1];

    refMain = isModeVer ? refAbove + 1 : refLeft + 1;
    refSide = isModeVer ? refLeft + 1 : refAbove + 1;

    // Extend main reference to right using replication
    const int log2Ratio = floorLog2(width) - floorLog2(height);
    const int s         = std::max<int>(0, isModeVer ? log2Ratio : -log2Ratio);
    const int maxIndex  = (multiRefIdx << s) + 6;
    const int refLength = isModeVer ? m_topRefLength : m_leftRefLength;
    const Pel val       = refMain[refLength + multiRefIdx];
    // right extended by 1 (z range)
    for (int z = 1; z <= (maxIndex + 1); z++)
    {
      refMain[refLength + multiRefIdx + z] = val;
    }
  }

  // swap width/height if we are doing a horizontal mode:
  if (!isModeVer)
  {
    std::swap(width, height);
  }
  Pel             tempArray[MAX_CU_SIZE * MAX_CU_SIZE];
  const ptrdiff_t dstStride = isModeVer ? pDst.stride : width;
  Pel            *pDstBuf   = isModeVer ? pDst.buf : tempArray;

  // compensate for line offset in reference line buffers
  refMain += multiRefIdx;
  refSide += multiRefIdx;

  Pel *pDsty = pDstBuf;

  if (intraPredAngle == 0)   // pure vertical or pure horizontal
  {
    if (m_ipaParam.applyPDPC)
    {
      const int scale = (floorLog2(width) + floorLog2(height) - 2) >> 2;
      m_IntraHorVerPDPC(pDsty, dstStride, refSide, width, height, scale, refMain, clpRng);
    }
    else
    {
      for (int y = 0; y < height; y++)
      {
        memcpy(pDsty, &refMain[1], width * sizeof(Pel));
        pDsty += dstStride;
      }
    }
  }
  else
  {
    bool isIntSlope = useExt ? isIntegerSlopeExt(abs(intraPredAngle)) : isIntegerSlope(abs(intraPredAngle));
    if (!isIntSlope)
    {
      if (isLuma(channelType))
      {
        m_IntraPredAngleLuma(refMain, pDsty, width, height, dstStride, intraPredAngle, multiRefIdx, clpRng,
                             m_ipaParam.interpolationFlag, useExt);
      }
      else
      {
        m_IntraPredAngleChroma(refMain, pDsty, width, height, dstStride, intraPredAngle, multiRefIdx);
      }
    }
    else
    {
      m_IntraPredAngleCpy(refMain, pDsty, width, height, dstStride, intraPredAngle, multiRefIdx, useExt);
    }

    if (m_ipaParam.applyPDPC && m_ipaParam.useGradPDPC)
    {
      m_IntraAngleGradPDPC(pDstBuf, dstStride, refSide, refMain, width, height, m_ipaParam.angularScale, intraPredAngle,
                           multiRefIdx, clpRng, useExt);
    }
    else if (m_ipaParam.applyPDPC)
    {
      m_IntraAnglePDPC(pDstBuf, dstStride, refSide, width, height, m_ipaParam.angularScale, absInvAngle);
    }
  }
  // Flip the block if this is the horizontal mode
  if (!isModeVer)
  {
    pDst.transposedFrom(CPelBuf(pDstBuf, dstStride, width, height));
  }
}

void IntraPrediction::predIntraSGPM(PelBuf &piPred, CodingUnit &cu, const CompArea &area, bool skipDerivation)
{
  CHECK(cu.sgpm == false, "we should'nt be here");

  if (!skipDerivation)
  {
    const CompArea                   &area = cu.Y();
    static_vector<SgpmInfo, SGPM_NUM> sgpmInfoList;
    static_vector<double, SGPM_NUM>   sgpmCostList;

    if (cu.lwidth() * cu.lheight() <= 1024)
    {
      deriveTimdMode(cu.cs->picture->getRecoBuf(area), area, cu, TimdDerivationMethod::HorizontalVertical);
    }

    if (cu.slice->m_sps->m_useDIMD)
    {
      deriveDimdMode(cu.dimdData, cu.cs->picture->getRecoBuf(area), area, cu);
    }

    deriveSgpmModeOrdered(cu.cs->picture->getRecoBuf(area), area, cu, sgpmInfoList, sgpmCostList, cu.dimdData);

    cu.sgpmSplitDir = sgpmInfoList[cu.sgpmIdx].sgpmSplitDir;
    cu.sgpmMode0    = sgpmInfoList[cu.sgpmIdx].sgpmMode0;
    cu.sgpmMode1    = sgpmInfoList[cu.sgpmIdx].sgpmMode1;
  }

  constexpr CompID compID        = COMP_Y;
  // do the first prediction
  cu.intraDir[ChannelType::LUMA] = cu.sgpmMode0;
  initIntraPatternChType(cu, area);
  predIntraAng(compID, piPred, cu, true, false);

  // do the second prediction
  PelBuf piPred2                 = m_sgpmBuffer.getCompactBuf(area);
  cu.intraDir[ChannelType::LUMA] = cu.sgpmMode1;
  initIntraPatternChType(cu, area, false, 1);
  predIntraAng(compID, piPred2, cu, true, false);

  // combine the prediction
  m_pIf->m_weightedSgpm(cu, cu.lwidth(), cu.lheight(), compID, cu.sgpmSplitDir, piPred, piPred, piPred2);

  cu.intraDir[ChannelType::LUMA] = cu.sgpmMode0;
}

// TIMD prediction
void IntraPrediction::predIntraTimd(PelBuf &piPred, CodingUnit &cu, const CompArea &area, bool skipDerivation,
                                    const TimdMode timdMode, const bool alreadyExecutedForNormalMode)
{
  if (!skipDerivation)
  {
    if (timdMode == TimdMode::Normal)
    {
      CHECK(alreadyExecutedForNormalMode, "Flag only relevant for SAD Mode");

      deriveTimdMode(cu.cs->picture->getRecoBuf(area), area, cu, TimdDerivationMethod::Full);
      cu.derivedIpm[0] = (int8_t)MAP131TO67(cu.timdData.blendMode[0]);
      cu.derivedIpm[1] = (cu.timdData.isBlend ? (int8_t)MAP131TO67(cu.timdData.blendMode[1]) : cu.derivedIpm[0]);
    }
    else
    {
      CHECK(!CU::allowTimdSad(cu), "Timd SAD is not allowed for this CU");
      if (!alreadyExecutedForNormalMode)
      {
        m_timdModeCostList.clear();
        deriveTimdMode(cu.cs->picture->getRecoBuf(area), area, cu, TimdDerivationMethod::Full);
      }
      std::stable_sort(m_timdModeCostList.begin(), m_timdModeCostList.end());

      deriveTimdMode(cu.cs->picture->getRecoBuf(area), area, cu, TimdDerivationMethod::FullWithSAD);
      cu.derivedIpm[0] = (int8_t)MAP131TO67(cu.timdSadData.blendMode[0]);
      cu.derivedIpm[1] = (cu.timdSadData.isBlend ? (int8_t)MAP131TO67(cu.timdSadData.blendMode[1]) : cu.derivedIpm[0]);
    }
  }

  PROFILER_SCOPE(1, g_timeProfiler, P_INTRA_EST_CAND_LUMA_TIMD);

  const auto &timdData = (timdMode == TimdMode::Normal ? cu.timdData : cu.timdSadData);

  // Do First Prediction
  int            width  = piPred.width;
  int            height = piPred.height;
  const UnitArea localUnitArea(cu.chromaFormat, Area(0, 0, width, height));
  cu.intraDir[ChannelType::LUMA] = timdData.blendMode[0];
  initIntraPatternChType(cu, area);
  predIntraAng(COMP_Y, piPred, cu, true, true);
  if (!timdData.isBlend)
  {
    cu.intraDir[ChannelType::LUMA] = MAP131TO67(cu.intraDir[ChannelType::LUMA]);
    return;
  }
  // Do second prediction
  PelBuf     predFusion          = m_tempBufferDIMD[0].getBuf(localUnitArea.Y());
  const bool applyPdpc           = m_ipaParam.applyPDPC;
  cu.intraDir[ChannelType::LUMA] = timdData.blendMode[1];
  initIntraPatternChType(cu, area);
  predIntraAng(COMP_Y, predFusion, cu, true, true);

  // Do non angular prediction
  PelBuf nonAngBuffer;
  if (timdData.relWeight[2] > 0)
  {
    nonAngBuffer                   = m_tempBufferDIMD[1].getBuf(localUnitArea.Y());
    cu.intraDir[ChannelType::LUMA] = timdData.blendMode[2];
    initIntraPatternChType(cu, area);
    predIntraAng(COMP_Y, nonAngBuffer, cu, true, true);
  }

  m_ipaParam.applyPDPC           = applyPdpc;
  cu.intraDir[ChannelType::LUMA] = MAP131TO67(timdData.blendMode[0]);
  // do blending
  Pel *pelPred                   = piPred.buf;
  Pel *pelPredFusion             = predFusion.buf;
  Pel *pelPredNonAng             = timdData.relWeight[2] > 0 ? nonAngBuffer.buf : nullptr;

  bool useLocDepBlending = false;
  int  weightVer = 0, weightHor = 0, weightNonLocDep = 0;
  for (int i = 0; i < TIMD_FUSION_NUM; i++)
  {
    if (timdData.locDep[i] == 1)
    {
      weightVer += timdData.relWeight[i];
    }
    else if (timdData.locDep[i] == 2)
    {
      weightHor += timdData.relWeight[i];
    }
    else
    {
      weightNonLocDep += timdData.relWeight[i];
    }
  }

  const int log2WeightSum = 6;
  const int weightSum     = 1 << log2WeightSum;
  if ((weightHor & (weightSum - 1)) || (weightVer & (weightSum - 1)))   // either weight is != 0 or 64
  {
    useLocDepBlending = true;
  }

  PelBuf    predAngVer       = m_tempBufferDIMD[2].getBuf(localUnitArea.Y());
  PelBuf    predAngHor       = m_tempBufferDIMD[3].getBuf(localUnitArea.Y());
  PelBuf    predAngNonLocDep = m_tempBufferDIMD[4].getBuf(localUnitArea.Y());
  Pel      *pelVer           = predAngVer.buf;
  ptrdiff_t strideVer        = predAngVer.stride;
  Pel      *pelHor           = predAngHor.buf;
  ptrdiff_t strideHor        = predAngHor.stride;
  Pel      *pelNonLocDep     = (!useLocDepBlending) ? piPred.buf : predAngNonLocDep.buf;
  ptrdiff_t strideNonLocDep  = (!useLocDepBlending) ? piPred.stride : predAngNonLocDep.stride;

  const ClpRng &clpRng(cu.slice->clpRng(COMP_Y));

  for (int locDep = 0; locDep < 3; locDep++)
  {
    int totWeight = (locDep == 0 ? weightNonLocDep : (locDep == 1 ? weightVer : weightHor));
    if (totWeight == 0)
    {
      continue;
    }

    int weights[TIMD_FUSION_NUM] = { 0 };
    for (int i = 0; i < TIMD_FUSION_NUM; i++)
    {
      weights[i] = (timdData.locDep[i] == locDep) ? timdData.relWeight[i] : 0;
    }

    int num2blend       = 0;
    int blendIndexes[3] = { 0 };
    for (int i = 0; i < TIMD_FUSION_NUM; i++)
    {
      if (weights[i] != 0)
      {
        blendIndexes[num2blend] = i;
        num2blend++;
      }
    }

    if ((num2blend == 1) || (num2blend <= 3 && isPowerOf2(totWeight)))
    {
      int index = blendIndexes[0];
      if (locDep == 0)
      {
        pelNonLocDep    = (index == 0 ? pelPred : (index == 2 ? pelPredNonAng : pelPredFusion));
        strideNonLocDep = (index == 0 ? piPred.stride : (index == 2 ? nonAngBuffer.stride : predFusion.stride));
      }
      else if (locDep == 1)
      {
        pelVer    = (index == 0 ? pelPred : (index == 2 ? pelPredNonAng : pelPredFusion));
        strideVer = (index == 0 ? piPred.stride : (index == 2 ? nonAngBuffer.stride : predFusion.stride));
      }
      else
      {
        pelHor    = (index == 0 ? pelPred : (index == 2 ? pelPredNonAng : pelPredFusion));
        strideHor = (index == 0 ? piPred.stride : (index == 2 ? nonAngBuffer.stride : predFusion.stride));
      }
      Pel      *pCur      = (locDep == 0 ? pelNonLocDep : (locDep == 1 ? pelVer : pelHor));
      ptrdiff_t strideCur = (locDep == 0 ? strideNonLocDep : (locDep == 1 ? strideVer : strideHor));

      int log2TotWeight = floorLog2(totWeight);
      int factor        = weightSum >> log2TotWeight;

      if (num2blend == 2)
      {
        int       index1  = blendIndexes[1];
        Pel      *p1      = (index1 == 0 ? pelPred : (index1 == 2 ? pelPredNonAng : pelPredFusion));
        ptrdiff_t stride1 = (index1 == 0 ? piPred.stride : (index1 == 2 ? nonAngBuffer.stride : predFusion.stride));

        int w0 = (weights[index] * factor);
        int w1 = weightSum - w0;
        g_pelBufOP.weightedAvg(pCur, strideCur, p1, stride1, pCur, strideCur, w0, w1, log2WeightSum, 0, width, height,
                               clpRng);
      }
      else if (num2blend == 3)
      {
        int       index1  = blendIndexes[1];
        Pel      *p1      = (index1 == 0 ? pelPred : (index1 == 2 ? pelPredNonAng : pelPredFusion));
        ptrdiff_t stride1 = (index1 == 0 ? piPred.stride : (index1 == 2 ? nonAngBuffer.stride : predFusion.stride));

        int       index2  = blendIndexes[2];
        Pel      *p2      = (index2 == 0 ? pelPred : (index2 == 2 ? pelPredNonAng : pelPredFusion));
        ptrdiff_t stride2 = (index2 == 0 ? piPred.stride : (index2 == 2 ? nonAngBuffer.stride : predFusion.stride));

        int w0 = (weights[index] * factor);
        int w1 = (weights[index1] * factor);
        int w2 = weightSum - w0 - w1;
        g_pelBufOP.weightedAvg3(pCur, strideCur, p1, stride1, p2, stride2, pCur, strideCur, w0, w1, w2, log2WeightSum,
                                0, width, height, clpRng);
      }
    }
    else
    {
      Pel      *pCur      = (locDep == 0 ? pelNonLocDep : (locDep == 1 ? pelVer : pelHor));
      ptrdiff_t strideCur = (locDep == 0 ? strideNonLocDep : (locDep == 1 ? strideVer : strideHor));

      Pel      *pelPredAng0      = piPred.buf;
      Pel      *pelPredAng1      = predFusion.buf;
      Pel      *pelNonAngBuf     = nonAngBuffer.buf;
      ptrdiff_t stride0          = piPred.stride;
      ptrdiff_t stride1          = predFusion.stride;
      ptrdiff_t stride2          = nonAngBuffer.stride;
      auto [scale, round, shift] = lutDivideGetScaleRoundShift(totWeight, INTRA_FUSION_BITS);
      for (int y = 0; y < height; y++)
      {
        for (int x = 0; x < width; x++)
        {
          int blend = pelPredAng0[x] * weights[0];
          blend += pelPredAng1[x] * weights[1];
          blend += pelNonAngBuf[x] * weights[2];
          pCur[x] = (Pel)((uint64_t(blend) * scale + round) >> (INTRA_FUSION_BITS + shift));
        }
        pCur += strideCur;
        pelPredAng0 += stride0;
        pelPredAng1 += stride1;
        pelNonAngBuf += stride2;
      }
    }
  }

  if (useLocDepBlending)
  {
    int       mode      = ((weightHor > 0 && weightVer > 0) ? 0 : (weightVer > 0 ? 1 : 2));
    Pel      *pelDst    = piPred.buf;
    ptrdiff_t strideDst = piPred.stride;
    int       range     = width * height > 127 ? 8 : 10;

    locDepBlending(pelDst, strideDst, pelVer, strideVer, pelHor, strideHor, pelNonLocDep, strideNonLocDep, width,
                   height, mode, weightVer, weightHor, weightNonLocDep, range);
  }
}

void IntraPrediction::predIntraDimd(PelBuf &piPred, CodingUnit &cu, const CompArea &area)
{
  PROFILER_SCOPE(1, g_timeProfiler, P_INTRA_EST_CAND_LUMA_DIMD);
  int width  = piPred.width;
  int height = piPred.height;

  // Derive DIMD Modes and DIMD Weights
  auto &dimdData = cu.dimdData;
  deriveDimdMode(cu.dimdData, cu.cs->picture->getRecoBuf(area), area, cu);
  cu.derivedIpm[0] = dimdData.blendMode[0];
  cu.derivedIpm[1] = dimdData.blendMode[1];

  CompArea areaForBuf = area;
  areaForBuf.repositionTo(Position(0, 0));

  // Do Planar Prediction
  const bool dimdIsPlanar = (dimdData.blendMode[0] == PLANAR_IDX);
  PelBuf     planarBuffer = dimdIsPlanar ? piPred : m_tempBufferDIMD[0].getBuf(areaForBuf);
  CHECK(cu.chType != ChannelType::LUMA, "Error");
  cu.intraDir[ChannelType::LUMA] = PLANAR_IDX;
  initIntraPatternChType(cu, area, true);
  const CodingStructure &cs = *cu.cs;
  if (dimdData.relWeight[0] != 0 || dimdIsPlanar)
  {
    initPredIntraParams(cu, area, *cs.sps);
    predIntraAng(COMP_Y, planarBuffer, cu, true, false);
    CHECK(m_ipaParam.multiRefIndex, "Error");
  }
  // If no weights found, planar only and return
  if (dimdIsPlanar)
  {
    cu.intraDir[ChannelType::LUMA] = PLANAR_IDX;
    return;
  }

  // Do First Angular Prediction
  cu.intraDir[ChannelType::LUMA] = dimdData.blendMode[0];
  initPredIntraParams(cu, area, *cs.sps);
  predIntraAng(COMP_Y, piPred, cu, true, false);

  // Do Further Angular Prediction
  PelBuf predAngBuffer[DIMD_FUSION_NUM];

  if (dimdData.isBlend)
  {
    for (int i = 2; i < DIMD_FUSION_NUM; i++)
    {
      if (dimdData.relWeight[i] != 0)
      {
        predAngBuffer[i]               = m_tempBufferDIMD[i - 1].getBuf(areaForBuf);
        cu.intraDir[ChannelType::LUMA] = dimdData.blendMode[i - 1];
        initPredIntraParams(cu, area, *cs.sps);
        predIntraAng(COMP_Y, predAngBuffer[i], cu, true, false);
      }
    }
    cu.intraDir[ChannelType::LUMA] = dimdData.blendMode[0];
  }

  // Do Blending
  PelBuf predAngVer       = m_tempBufferDIMD[DIMD_FUSION_NUM + 1].getBuf(areaForBuf);
  PelBuf predAngHor       = m_tempBufferDIMD[DIMD_FUSION_NUM].getBuf(areaForBuf);
  PelBuf predAngNonLocDep = m_tempBufferDIMD[DIMD_FUSION_NUM - 1].getBuf(areaForBuf);

  Pel      *pelVer          = predAngVer.buf;
  ptrdiff_t strideVer       = predAngVer.stride;
  Pel      *pelHor          = predAngHor.buf;
  ptrdiff_t strideHor       = predAngHor.stride;
  Pel      *pelNonLocDep    = predAngNonLocDep.buf;
  ptrdiff_t strideNonLocDep = predAngNonLocDep.stride;

  bool useLocDepBlending = false;
  int  weightVer = 0, weightHor = 0, weightNonLocDep = 0;
  weightNonLocDep += dimdData.relWeight[0];   // Planar non-location dependent
  for (int i = 1; i < DIMD_FUSION_NUM; i++)
  {
    if (i == 1 || (dimdData.relWeight[i] != 0))
    {
      if (dimdData.locDep[i] == 1)
      {
        weightVer += dimdData.relWeight[i];
      }
      else if (dimdData.locDep[i] == 2)
      {
        weightHor += dimdData.relWeight[i];
      }
      else
      {
        weightNonLocDep += dimdData.relWeight[i];
      }
    }
  }

  const int log2WeightSum = 6;
  const int weightSum     = 1 << log2WeightSum;
  CHECKD(weightHor + weightNonLocDep + weightVer != weightSum, "Weights in Location-dependent DIMD do not add up")
  if ((weightHor & (weightSum - 1)) || (weightVer & (weightSum - 1)))   // either weight is != 0 or 64
  {
    useLocDepBlending = true;
  }
  if (!useLocDepBlending)
  {
    pelNonLocDep    = piPred.buf;
    strideNonLocDep = piPred.stride;
  }

  for (int locDep = 0; locDep < 3; locDep++)
  {
    int totalWeight = (locDep == 0 ? weightNonLocDep : (locDep == 1 ? weightVer : weightHor));
    if (totalWeight == 0)
    {
      continue;
    }

    int weights[DIMD_FUSION_NUM] = { 0 };
    weights[0]                   = (locDep == 0) ? dimdData.relWeight[0] : 0;
    weights[1]                   = (dimdData.locDep[1] == locDep) ? dimdData.relWeight[1] : 0;
    for (int i = 2; i < DIMD_FUSION_NUM; i++)
    {
      weights[i] = ((dimdData.relWeight[i] != 0) && dimdData.locDep[i] == locDep) ? dimdData.relWeight[i] : 0;
    }

    bool blendNow[DIMD_FUSION_NUM]     = { false };
    int  num2blend                     = 0;
    int  blendIndexes[DIMD_FUSION_NUM] = { 0 };
    for (int i = 0; i < DIMD_FUSION_NUM; i++)
    {
      if (weights[i] != 0)
      {
        blendNow[i]             = true;
        blendIndexes[num2blend] = i;
        num2blend++;
      }
    }
    if (num2blend == 1)
    {
      int index = blendIndexes[0];
      if (locDep == 0)
      {
        pelNonLocDep = (index == 1 ? piPred.buf : (index == 0 ? planarBuffer.buf : predAngBuffer[index].buf));
        strideNonLocDep =
          (index == 1 ? piPred.stride : (index == 0 ? planarBuffer.stride : predAngBuffer[index].stride));
      }
      else if (locDep == 1)
      {
        pelVer    = (index == 1 ? piPred.buf : predAngBuffer[index].buf);
        strideVer = (index == 1 ? piPred.stride : predAngBuffer[index].stride);
      }
      else   // locDep==2
      {
        pelHor    = (index == 1 ? piPred.buf : predAngBuffer[index].buf);
        strideHor = (index == 1 ? piPred.stride : predAngBuffer[index].stride);
      }
    }
    else
    {
      Pel      *pelCur    = (locDep == 0 ? pelNonLocDep : (locDep == 1 ? pelVer : pelHor));
      ptrdiff_t strideCur = (locDep == 0 ? strideNonLocDep : (locDep == 1 ? strideVer : strideHor));

      Pel      *pelPredAng       = piPred.buf;
      Pel      *pelPlanar        = planarBuffer.buf;
      ptrdiff_t strideAng        = piPred.stride;
      ptrdiff_t stridePlanar     = planarBuffer.stride;
      auto [scale, round, shift] = lutDivideGetScaleRoundShift(totalWeight, INTRA_FUSION_BITS);
      for (int y = 0; y < height; y++)
      {
        for (int x = 0; x < width; x++)
        {
          int blend = pelPredAng[x] * weights[1];
          blend += pelPlanar[x] * weights[0];
          for (int i = 2; i < DIMD_FUSION_NUM; i++)
          {
            blend += blendNow[i] ? (predAngBuffer[i].buf)[x] * weights[i] : 0;
          }
          pelCur[x] = (Pel)((uint64_t(blend) * scale + round) >> (INTRA_FUSION_BITS + shift));
        }
        pelCur += strideCur;
        pelPredAng += strideAng;
        pelPlanar += stridePlanar;
        for (int i = 2; i < DIMD_FUSION_NUM; i++)
        {
          predAngBuffer[i].buf += blendNow[i] ? predAngBuffer[i].stride : 0;
        }
      }
    }
  }
  if (useLocDepBlending)
  {
    int       mode      = ((weightHor > 0 && weightVer > 0) ? 0 : (weightVer > 0 ? 1 : 2));
    Pel      *pelDst    = piPred.buf;
    ptrdiff_t strideDst = piPred.stride;
    locDepBlending(pelDst, strideDst, pelVer, strideVer, pelHor, strideHor, pelNonLocDep, strideNonLocDep, width,
                   height, mode, weightVer, weightHor, weightNonLocDep);
  }
}

void IntraPrediction::predIntraDimdChroma(PelBuf &piPred, CodingUnit &cu, const CompID compID, const CompArea &area)
{
  const CodingStructure &cs = *cu.cs;
  DimdData               dimdData;
  deriveDimdChromaMode(dimdData, cu);

  // Note: the original DCIMD paper proposes a single mode for chroma, unlike the two for luma, because of the relative
  // simplicity of chroma textures initialize reconstructed samples and force filtering them (not filtered by default)
  initIntraPatternChType(cu, area, true);

  cu.intraDir[ChannelType::CHROMA] = dimdData.blendMode[0];

  // angular prediction
  initPredIntraParams(cu, area, *cs.sps);
  predIntraAng(compID, piPred, cu, true, false);
}

void IntraPrediction::locDepBlending(Pel *pDst, ptrdiff_t strideDst, Pel *pVer, ptrdiff_t strideVer, Pel *pHor,
                                     ptrdiff_t strideHor, Pel *pNonLocDep, ptrdiff_t strideNonLocDep, int width,
                                     int height, int mode, int wVer, int wHor, int wNonLocDep, int range)
{
  int maxWeight      = (1 << 6);
  int weightShift    = 6;
  int weightOffset   = 1 << (weightShift - 1);
  int sizeThreshold  = 64;
  int heightMinusOne = (height - 1);
  int widthMinusOne  = (width - 1);

  auto divide = [](int num, int scale, int round, int shift)
  { return int((uint64_t(num) * scale + round) >> (INTRA_FUSION_BITS + shift)); };

  if (mode == 0)   // diagonal blending
  {
    int totRange               = wNonLocDep;
    int totDirWeight           = wVer + wHor;
    auto [scale, round, shift] = lutDivideGetScaleRoundShift(totDirWeight, INTRA_FUSION_BITS);
    int clipRangeVer           = wVer + divide(wVer, scale, round, shift) * totRange;
    int clipRangeHor           = wHor + divide(wHor, scale, round, shift) * totRange;

    int rangeVer = range;
    int rangeHor = range;

    if (height > sizeThreshold)
    {
      rangeVer *= 2;
    }
    if (width > sizeThreshold)
    {
      rangeHor *= 2;
    }

    bool needClipVer = (((wVer + rangeVer) > clipRangeVer) || ((wVer - rangeVer) < 0));
    bool needClipHor = (((wHor + rangeHor) > clipRangeHor) || ((wHor - rangeHor) < 0));
    int  weightVer;
    int  weightHor;

    auto [scaleVer, roundVer, shiftVer] = lutDivideGetScaleRoundShift(heightMinusOne, INTRA_FUSION_BITS);
    auto [scaleHor, roundHor, shiftHor] = lutDivideGetScaleRoundShift(widthMinusOne, INTRA_FUSION_BITS);
    int weightVerDecr =
      int((uint64_t(rangeVer << 1) * scaleVer + roundVer) >> (INTRA_FUSION_BITS - LOC_DEP_BITS + shiftVer));
    int weightHorDecr =
      int((uint64_t(rangeHor << 1) * scaleHor + roundHor) >> (INTRA_FUSION_BITS - LOC_DEP_BITS + shiftHor));
    for (int y = 0; y < height; y++)
    {
      weightVer = (((wVer + rangeVer) << 5) - y * weightVerDecr) >> 5;
      if (needClipVer)
      {
        weightVer = Clip3(0, clipRangeVer, weightVer);
      }

      for (int x = 0; x < width; x++)
      {
        weightHor = wNonLocDep ? (((wHor + rangeHor) << LOC_DEP_BITS) - x * weightHorDecr) >> LOC_DEP_BITS
                               : maxWeight - weightVer;
        if (needClipHor && wNonLocDep)
        {
          weightHor = Clip3(0, clipRangeHor, weightHor);
        }
        int weightNonLocDep = maxWeight - weightVer - weightHor;
        int blend =
          (int)pVer[x] * weightVer + (int)pHor[x] * weightHor + (int)pNonLocDep[x] * weightNonLocDep + weightOffset;
        pDst[x] = (Pel)(blend >> weightShift);
      }
      pDst += strideDst;
      pVer += strideVer;
      pHor += strideHor;
      pNonLocDep += strideNonLocDep;
    }
  }
  else if (mode == 1)   // vertical blending
  {
    int clipRangeVer = 64;
    int rangeVer     = range;

    if (height > sizeThreshold)
    {
      rangeVer *= 2;
    }
    bool needClipVer = (((wVer + rangeVer) > clipRangeVer) || ((wVer - rangeVer) < 0));
    int  weightVer;
    auto [scaleVer, roundVer, shiftVer] = lutDivideGetScaleRoundShift(heightMinusOne, INTRA_FUSION_BITS);
    int weightVerDecr =
      int((uint64_t(rangeVer << 1) * scaleVer + roundVer) >> (INTRA_FUSION_BITS - LOC_DEP_BITS + shiftVer));
    for (int y = 0; y < height; y++)
    {
      weightVer = (((wVer + rangeVer) << LOC_DEP_BITS) - y * weightVerDecr) >> LOC_DEP_BITS;
      if (needClipVer)
      {
        weightVer = Clip3(0, clipRangeVer, weightVer);
      }
      for (int x = 0; x < width; x++)
      {
        int weightNonLocDep = maxWeight - weightVer;
        int blend           = (int)pVer[x] * weightVer + (int)pNonLocDep[x] * weightNonLocDep + weightOffset;
        pDst[x]             = (Pel)(blend >> weightShift);
      }
      pDst += strideDst;
      pVer += strideVer;
      pNonLocDep += strideNonLocDep;
    }
  }
  else   // if(mode == 2) //horizontal blending
  {
    int clipRangeHor = 64;
    int rangeHor     = range;

    if (width > sizeThreshold)
    {
      rangeHor *= 2;
    }
    bool needClipHor = (((wHor + rangeHor) > clipRangeHor) || ((wHor - rangeHor) < 0));
    int  weightHor;
    auto [scaleHor, roundHor, shiftHor] = lutDivideGetScaleRoundShift(widthMinusOne, INTRA_FUSION_BITS);
    int weightHorDecr =
      int((uint64_t(rangeHor << 1) * scaleHor + roundHor) >> (INTRA_FUSION_BITS - LOC_DEP_BITS + shiftHor));
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        weightHor = (((wHor + rangeHor) << LOC_DEP_BITS) - x * weightHorDecr) >> LOC_DEP_BITS;
        if (needClipHor)
        {
          weightHor = Clip3(0, clipRangeHor, weightHor);
        }
        int weightNonLocDep = maxWeight - weightHor;
        int blend           = (int)pHor[x] * weightHor + (int)pNonLocDep[x] * weightNonLocDep + weightOffset;
        pDst[x]             = (Pel)(blend >> weightShift);
      }
      pDst += strideDst;
      pHor += strideHor;
      pNonLocDep += strideNonLocDep;
    }
  }
}

void IntraPrediction::deriveDimdMode(DimdData &dimdData, const CPelBuf &recoBuf, const CompArea &area,
                                     const CodingUnit &cu)
{
  const CodingStructure &cs  = *cu.cs;
  const SPS             &sps = *cs.sps;
  const PreCalcValues   &pcv = *cs.pcv;

  const Pel     *pReco     = recoBuf.buf;
  const uint32_t uiWidth   = area.width;
  const uint32_t uiHeight  = area.height;
  const int      iStride   = static_cast<int>(recoBuf.stride);
  const int      predSize  = uiWidth + 1;
  const int      predHSize = uiHeight + 1;

  const bool noShift      = pcv.noChroma2x2 && uiWidth == 4;   // don't shift on the lowest level (chroma not-split)
  const int unitWidthLog2 = pcv.minCUWidthLog2 - (noShift ? 0 : getComponentScaleX(area.compID, sps.m_chromaFormatIdc));
  const int unitHeightLog2 =
    pcv.minCUHeightLog2 - (noShift ? 0 : getComponentScaleY(area.compID, sps.m_chromaFormatIdc));
  const int unitWidth  = 1 << unitWidthLog2;
  const int unitHeight = 1 << unitHeightLog2;

  const int totalAboveUnits    = (predSize + (unitWidth - 1)) >> unitWidthLog2;
  const int totalLeftUnits     = (predHSize + (unitHeight - 1)) >> unitHeightLog2;
  const int totalUnits         = totalAboveUnits + totalLeftUnits + 1;   //+1 for top-left
  const int numAboveUnits      = std::max<int>(uiWidth >> unitWidthLog2, 1);
  const int numLeftUnits       = std::max<int>(uiHeight >> unitHeightLog2, 1);
  const int numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int numLeftBelowUnits  = totalLeftUnits - numLeftUnits;

  CHECK(numAboveUnits <= 0 || numLeftUnits <= 0 || numAboveRightUnits <= 0 || numLeftBelowUnits <= 0,
        "Size not supported");

  // ----- Step 1: analyze neighborhood -----
  const Position posLT = area;

  bool neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  memset(neighborFlags, 0, totalUnits);

  int numIntraAbove =
    isAboveAvailable(cu, toChannelType(area.compID), posLT, numAboveUnits, unitWidth,
                     (neighborFlags + totalLeftUnits + 1), isChroma(area.compID) ? 0 : (int)cu.multiRefIdx);
  int numIntraLeft =
    isLeftAvailable(cu, toChannelType(area.compID), posLT, numLeftUnits, unitHeight,
                    (neighborFlags + totalLeftUnits - 1), isChroma(area.compID) ? 0 : (int)cu.multiRefIdx);
  const int numIntraAboveRight = isAboveRightAvailable(
    cu, toChannelType(area.compID), area.topRight(), numAboveRightUnits, unitWidth,
    (neighborFlags + totalLeftUnits + 1 + numAboveUnits), isChroma(area.compID) ? 0 : (int)cu.multiRefIdx);
  const int numIntraBottomLeft = isBelowLeftAvailable(
    cu, toChannelType(area.compID), area.bottomLeft(), numLeftBelowUnits, unitHeight,
    (neighborFlags + totalLeftUnits - 1 - numLeftUnits), isChroma(area.compID) ? 0 : (int)cu.multiRefIdx);

  if (numIntraLeft == 0 && numIntraAbove == 0)
  {   // no Histogram possible
    dimdData.blendMode[0] = PLANAR_IDX;
    dimdData.blendMode[1] = DC_IDX;
    dimdData.isBlend      = false;
    dimdData.relWeight[0] = 1;
    return;
  }

  // ----- Step 2: build histogram of gradients -----
  int aiHistogram[NUM_LUMA_MODE]      = { 0 };
  int histogramLeft[NUM_LUMA_MODE]    = { 0 };
  int histogramTop[NUM_LUMA_MODE]     = { 0 };
  int histogramTopLeft[NUM_LUMA_MODE] = { 0 };

  if (numIntraLeft)
  {
    const uint32_t uiHeightLeft = (numIntraLeft + numIntraBottomLeft) * unitHeight - 1 - (numIntraAbove ? 0 : 1);
    const Pel     *pRecoLeft    = pReco - 2 + iStride * (numIntraAbove ? 0 : 1);
    buildHistogram(pRecoLeft, iStride, uiHeightLeft, 1, histogramLeft);
  }

  if (numIntraAbove)
  {
    const uint32_t uiWidthAbove = (numIntraAbove + numIntraAboveRight) * unitWidth - 1 - (numIntraLeft ? 0 : 1);
    const Pel     *pRecoAbove   = pReco - iStride * 2 + (numIntraLeft ? 0 : 1);
    buildHistogram(pRecoAbove, iStride, 1, uiWidthAbove, histogramTop);
  }

  if (numIntraAbove && numIntraLeft)
  {
    const Pel *pRecoTopLeft = pReco - iStride * 2 - 2;
    buildHistogram(pRecoTopLeft, iStride, 2, 2, histogramTopLeft, true);
  }
  for (int i = 0; i < NUM_LUMA_MODE; i++)
  {
    aiHistogram[i] = histogramTop[i] + histogramLeft[i] + histogramTopLeft[i];
  }

  // ----- Step 3: derive best modes from histogram of gradients -----
  int amp[DIMD_FUSION_NUM - 1]  = { 0 };
  int mode[DIMD_FUSION_NUM - 1] = { 0 };
  for (int i = 0; i < NUM_LUMA_MODE; i++)
  {
    int curAmp  = aiHistogram[i];
    int curMode = i;
    for (int j = 0; j < DIMD_FUSION_NUM - 1; j++)
    {
      if (curAmp > amp[j])
      {
        for (int k = DIMD_FUSION_NUM - 2; k > j; k--)
        {
          amp[k]  = amp[k - 1];
          mode[k] = mode[k - 1];
        }
        amp[j]  = curAmp;
        mode[j] = curMode;
        break;
      }
    }
  }

  // check location dependendency
  for (int i = 1; i < DIMD_FUSION_NUM; i++)
  {
    dimdData.locDep[i] = 0;
    int curMode        = mode[i - 1];
    if (curMode > DC_IDX)
    {
      uint32_t amp      = aiHistogram[curMode];
      uint32_t ampLeft  = histogramLeft[curMode];
      uint32_t ampAbove = histogramTop[curMode];
      // explicit amp/3 by Montgomery modular multiplication, as done by optimizing compilers
      uint32_t ampD3    = (uint64_t(amp) * 0x55555556) >> 32;
      if (ampLeft < ampD3)
      {
        dimdData.locDep[i] = 1;   // vertical mode
      }
      else if (ampAbove < ampD3)
      {
        dimdData.locDep[i] = 2;   // horizontal mode
      }
    }
  }

  dimdData.blendMode[0] = mode[0];
  dimdData.isBlend      = true;
  dimdData.isBlend &= amp[1] > 0;
  dimdData.isBlend &= mode[1] > DC_IDX;
  dimdData.isBlend &= mode[0] > DC_IDX;

  if (dimdData.locDep[1] != 0 && amp[1] == 0)   // only 1 mode, but location dependent, blend with planar
  {
    dimdData.isBlend = true;
    dimdData.isBlend &= mode[0] > DC_IDX;
    mode[1] = PLANAR_IDX;
    CHECK(DIMD_FUSION_NUM > 3 && mode[2] != 0, "Wrong logic");
  }

  int countBlendModes = 2;

  if (dimdData.isBlend)
  {
    for (int i = 1; i < DIMD_FUSION_NUM - 1; i++)
    {
      dimdData.blendMode[i] = mode[i];
      if (dimdData.blendMode[i] != PLANAR_IDX)
      {
        countBlendModes++;
      }
    }
  }

  // ----- Step 4: choose weights according to amplitudes -----
  const int blendSumWeight = 6;
  int       sumWeight      = 1 << blendSumWeight;
  memset(dimdData.relWeight, 0, sizeof(dimdData.relWeight));   // initalize weights

  if (dimdData.isBlend)
  {
    if (mode[1] == 0)   // special case: only one mode, but location dependent
    {
      CHECKD(dimdData.locDep[1] == 0, "Wrong logic");
      dimdData.relWeight[0] = 21;   // planar weight
      dimdData.relWeight[1] = sumWeight - 21;
    }
    else
    {
      int planarWeight = 16;   // ~ 1/4 of the weight reserved for planar
      sumWeight -= planarWeight;

      // compute iRatio[j] = sum_weight * amp[j] / s1 with s1= amp[0] + ... + amp[DIMD_FUSION_NUM-1]
      int s1 = 0;
      for (int i = 0; i < DIMD_FUSION_NUM - 1; i++)
      {
        s1 += amp[i];
      }
      int x      = floorLog2(s1);
      int normS1 = (s1 << 4 >> x) & 15;
      int v      = g_gradDivTable[normS1];

      x += (normS1 != 0);
      int shift                       = x + 3;
      int add                         = (1 << (shift - 1));
      int iRatio[DIMD_FUSION_NUM - 1] = { 0 };

      for (int i = 0; i < DIMD_FUSION_NUM - 1; i++)
      {
        iRatio[i] = (amp[i] * v * sumWeight + add) >> shift;
        if (amp[i] == 0)
        {
          iRatio[i] = 0;
        }
        if (iRatio[i] > sumWeight)
        {
          iRatio[i] = sumWeight;
        }
        CHECK(iRatio[i] > sumWeight, "Wrong DIMD ratio");
      }

      dimdData.relWeight[1] = iRatio[0];

      int sumWeightReal      = iRatio[0];
      int countBlendModesNew = countBlendModes;
      int lastFilled         = 0;

      for (int i = 1; i < countBlendModes - 2; i++)
      {
        dimdData.relWeight[i + 1] = iRatio[i];
        sumWeightReal += iRatio[i];
        if (sumWeightReal > sumWeight)
        {
          lastFilled = i;
          break;
        }
      }

      if (sumWeightReal > sumWeight)
      {
        for (int i = lastFilled + 1; i <= countBlendModes - 2; i++)
        {
          iRatio[i]                 = 0;
          dimdData.relWeight[i + 1] = 0;
          dimdData.blendMode[i + 1] = 0;
          countBlendModesNew--;
        }
      }
      countBlendModes = countBlendModesNew;
      if (sumWeightReal > sumWeight)
      {
        int diff = sumWeightReal - sumWeight;
        for (int j = 0; j < sumWeight; j++)
        {
          for (int i = lastFilled; i >= 1; i--)
          {
            CHECK(i < 0, "Wrong index");
            CHECK(i >= DIMD_FUSION_NUM - 1, "Wrong index");

            iRatio[i] -= 1;
            dimdData.relWeight[i + 1] = iRatio[i];
            if (dimdData.relWeight[i + 1] == 0)
            {
              CHECK(i < 1, "Wrong index");
              dimdData.blendMode[i + 1] = 0;
              countBlendModes--;
            }
            diff--;
            if (diff == 0)
            {
              break;
            }
          }
          if (diff == 0)
          {
            break;
          }
        }
      }
      dimdData.relWeight[countBlendModes - 1] = sumWeight;
      for (int i = 0; i < countBlendModes - 2; i++)
      {
        dimdData.relWeight[countBlendModes - 1] = dimdData.relWeight[countBlendModes - 1] - iRatio[i];
        if (dimdData.relWeight[countBlendModes - 1] == 0)
        {
          dimdData.blendMode[countBlendModes - 1] = 0;
        }
      }
      dimdData.relWeight[0] = planarWeight;
      for (int i = 2; i < DIMD_FUSION_NUM; i++)
      {
        if (dimdData.relWeight[i] == 0)
        {
          dimdData.blendMode[i] = PLANAR_IDX;
        }
      }
    }
  }
  else
  {
    dimdData.relWeight[0] = sumWeight;   // weight for DIMD mode = 1
  }
}

void IntraPrediction::computeHistogramForAllComponents(const CPelBuf &recoBufY, const CPelBuf &recoBufCb,
                                                       const CPelBuf &recoBufCr, const CompArea &areaY,
                                                       const CompArea &areaCb, const CompArea &areaCr, CodingUnit &cu,
                                                       int *histogram)
{
  const CodingStructure &cs  = *cu.cs;
  const SPS             &sps = *cs.sps;
  const PreCalcValues   &pcv = *cs.pcv;

  const Pel *recoY   = recoBufY.buf;
  const int  strideY = static_cast<int>(recoBufY.stride);

  const Pel     *recoCb   = recoBufCb.buf;
  const uint32_t widthCb  = areaCb.width;
  const uint32_t heightCb = areaCb.height;
  const int      strideCb = static_cast<int>(recoBufCb.stride);

  const Pel *recoCr   = recoBufCr.buf;
  const int  strideCr = static_cast<int>(recoBufCr.stride);

  // get the availability of the neighboring chroma samples
  const int predWidth  = (widthCb << 1);
  const int predHeight = (heightCb << 1);

  const bool noShift = pcv.noChroma2x2 && widthCb == 4;   // don't shift on the lowest level (chroma not-split)
  const int  unitWidthLog2 =
    pcv.minCUWidthLog2 - (noShift ? 0 : getComponentScaleX(areaCb.compID, sps.m_chromaFormatIdc));
  const int unitHeightLog2 =
    pcv.minCUHeightLog2 - (noShift ? 0 : getComponentScaleY(areaCb.compID, sps.m_chromaFormatIdc));
  const int unitWidth  = 1 << unitWidthLog2;
  const int unitHeight = 1 << unitHeightLog2;

  const int totalAboveUnits    = (predWidth + (unitWidth - 1)) >> unitWidthLog2;
  const int totalLeftUnits     = (predHeight + (unitHeight - 1)) >> unitHeightLog2;
  const int totalUnits         = totalAboveUnits + totalLeftUnits + 1;   //+1 for top-left
  const int numAboveUnits      = std::max<int>(widthCb >> unitWidthLog2, 1);
  const int numLeftUnits       = std::max<int>(heightCb >> unitHeightLog2, 1);
  const int numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int numLeftBelowUnits  = totalLeftUnits - numLeftUnits;

  const bool useSmallFilterY  = shouldUse2x2EdgeOperator(areaY);
  // check only the Cb area, as it is identical to the Cr
  const bool useSmallFilterCb = shouldUse2x2EdgeOperator(areaCb);

  CHECK(numAboveUnits <= 0 || numLeftUnits <= 0 || numAboveRightUnits <= 0 || numLeftBelowUnits <= 0,
        "Size not supported");

  const Position posLT = areaCb;

  bool neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  memset(neighborFlags, 0, totalUnits);

  // Note: we are only checking the availability on the chroma channel, as we can assume the luma samples are there
  // ideally, we would check for the availability of luma samples, but this is difficult in dual-tree mode, as the
  // coding structures are dedicated to either luma or chroma
  int numIntraAbove = isAboveAvailable(cu, ChannelType::CHROMA, posLT.offset(0, -2), numAboveUnits, unitWidth,
                                       (neighborFlags + totalLeftUnits + 1), 0);
  int numIntraLeft  = isLeftAvailable(cu, ChannelType::CHROMA, posLT.offset(-2, 0), numLeftUnits, unitHeight,
                                      (neighborFlags + totalLeftUnits - 1), 0);

  // JVET AC0094 extends the chroma reference samples to 2xW above and 2xH to the left, so we need to check for those as
  // well
  const int numIntraAboveRight =
    std::min(isAboveRightAvailable(cu, ChannelType::CHROMA, areaCb.topRight().offset(0, -2), numAboveRightUnits,
                                   unitWidth, neighborFlags + totalLeftUnits + 1 + numAboveUnits, 0),
             isAboveRightAvailable(cu, ChannelType::CHROMA, areaCb.topRight(), numAboveRightUnits, unitWidth,
                                   neighborFlags + totalLeftUnits + 1 + numAboveUnits, 0));
  const int numIntraBottomLeft =
    std::min(isBelowLeftAvailable(cu, ChannelType::CHROMA, areaCb.bottomLeft().offset(-2, 0), numLeftBelowUnits,
                                  unitHeight, neighborFlags + totalLeftUnits - 1 - numLeftUnits, 0),
             isBelowLeftAvailable(cu, ChannelType::CHROMA, areaCb.bottomLeft(), numLeftBelowUnits, unitHeight,
                                  neighborFlags + totalLeftUnits - 1 - numLeftUnits, 0));

  // JVET AC0094 changes the luma samples used for the histogram
  // instead of the top/left border, the top and left inner edges of the co-located luma sample are used
  // again, we assume the co-located luma samples have been reconstructed
  const uint32_t heightLeftY =
    (numLeftUnits * unitHeight << getChannelTypeScaleY(ChannelType::CHROMA, sps.m_chromaFormatIdc)) - 2;
  // the buffer should be positioned as if it's the top-left corner of the 2x2 filter (the center for the 3x3 filter)
  // recoY is already referencing the top-left position in the reconstruction buffer, so this places the filter 2x2
  // perfectly for 3x3, the center is the reference point => x += 1, y += 1
  const int  smallFilterOffset = useSmallFilterY ? -1 : strideY;
  const Pel *recoLeftY         = recoY + 1 + smallFilterOffset;
  buildHistogram(recoLeftY, strideY, heightLeftY, 2, histogram, false, useSmallFilterY);

  const uint32_t widthAboveY =
    (numAboveUnits * unitWidth << getChannelTypeScaleX(ChannelType::CHROMA, sps.m_chromaFormatIdc)) - 4;
  // for the top part of the co-located luma sample, displace the filter to the right by its size
  // (x += 3 for 3x3, x += 2 for 2x2)
  const Pel *recoAboveY = recoY + 3 + smallFilterOffset;
  buildHistogram(recoAboveY, strideY, 2, widthAboveY, histogram, false, useSmallFilterY);

  if (numIntraLeft)
  {
    const uint32_t heightLeftC = (numIntraLeft + numIntraBottomLeft) * unitHeight - 1 - (!numIntraAbove ? 1 : 0);

    const Pel *recoLeftCb = recoCb - 2 + strideCb * (!numIntraAbove ? 1 : 0);
    const Pel *recoLeftCr = recoCr - 2 + strideCr * (!numIntraAbove ? 1 : 0);

    buildHistogram(recoLeftCb, strideCb, heightLeftC, 1, histogram, false, useSmallFilterCb);
    buildHistogram(recoLeftCr, strideCr, heightLeftC, 1, histogram, false, useSmallFilterCb);
  }

  if (numIntraAbove)
  {
    const uint32_t widthAboveC = (numIntraAbove + numIntraAboveRight) * unitWidth - 1 - (!numIntraLeft ? 1 : 0);

    const Pel *recoAboveCb = recoCb - strideCb * 2 + (!numIntraLeft ? 1 : 0);
    const Pel *recoAboveCr = recoCr - strideCr * 2 + (!numIntraLeft ? 1 : 0);

    buildHistogram(recoAboveCb, strideCb, 1, widthAboveC, histogram, false, useSmallFilterCb);
    buildHistogram(recoAboveCr, strideCr, 1, widthAboveC, histogram, false, useSmallFilterCb);
  }
  if (numIntraLeft && numIntraAbove)
  {
    const Pel *recoAboveLeftCb = recoCb - 2 - strideCb * 2;
    const Pel *recoAboveLeftCr = recoCr - 2 - strideCr * 2;

    buildHistogram(recoAboveLeftCb, strideCb, 2, 2, histogram, true, useSmallFilterCb);
    buildHistogram(recoAboveLeftCr, strideCr, 2, 2, histogram, true, useSmallFilterCb);
  }
}

void IntraPrediction::deriveDimdChromaMode(DimdData &dimdData, CodingUnit &cu)
{
  const CodingStructure &cs      = *cu.cs;
  const Picture         &picture = *cs.picture;

  // create separate buffers and do the prediction for each component, as they are used together for the histogram
  // calculation
  const CompArea areaCb = cu.Cb();
  const CompArea areaCr = cu.Cr();
  // build a different area for luma, as luma may have higher resolution than chroma and may also have different
  // partitioning
  // -> not enough to get the colocated luma CU
  const CompArea areaY  = CompArea(COMP_Y, cu.chromaFormat, areaCb.lumaPos(), areaCb.lumaSize());

  CPelBuf recoBufCb = picture.getRecoBuf(areaCb);
  CPelBuf recoBufCr = picture.getRecoBuf(areaCr);
  CPelBuf recoBufY  = picture.getRecoBuf(areaY);

  int histogram[NUM_LUMA_MODE] = { 0 };
  computeHistogramForAllComponents(recoBufY, recoBufCb, recoBufCr, areaY, areaCb, areaCr, cu, histogram);

  int bestMode = 0;
  findBestModesInHistogram(histogram, bestMode, cu);

  dimdData.blendMode[0] = bestMode;
}

/**
 * @brief Derive the best mode for intra prediction (luma)
 * @param histogram the histogram of mode occurance
 * @param bestMode the mode with the highest amplitude
 * @param cu
 */
void IntraPrediction::findBestModesInHistogram(int *histogram, int &bestMode, const CodingUnit &cu)
{
  int amplitudeFirstMode = 0, amplitudeSecondMode = 0, currentAmplitude = 0;
  int secondBestMode = 0, currentMode = 0;

  for (int i = 0; i < NUM_LUMA_MODE; i++)
  {
    currentAmplitude = histogram[i];
    currentMode      = i;
    if (currentAmplitude > amplitudeFirstMode)
    {
      amplitudeSecondMode = amplitudeFirstMode;
      secondBestMode      = bestMode;
      amplitudeFirstMode  = currentAmplitude;
      bestMode            = currentMode;
    }
    else
    {
      if (currentAmplitude > amplitudeSecondMode)
      {
        amplitudeSecondMode = currentAmplitude;
        secondBestMode      = currentMode;
      }
    }
  }

  pruneDimdChromaModes(bestMode, secondBestMode, cu);
}

/**
 * @brief Avoid reedundancy with the DM mode by using the second best mode, if the best mode is the DM mode
 * @param bestMode the best mode
 * @param secondBestMode the second best mode
 * @param cu the coding unit
 */
void IntraPrediction::pruneDimdChromaModes(int &bestMode, int &secondBestMode, const CodingUnit &cu)
{
  int dmMode = PU::getCoLocatedIntraLumaMode(cu);
  if (dmMode == bestMode)
  {
    if (bestMode == secondBestMode)
    {
      bestMode = DC_IDX;
    }
    else
    {
      bestMode = secondBestMode;
    }
  }
}

int IntraPrediction::buildHistogram(const Pel *pReco, int iStride, uint32_t uiHeight, uint32_t uiWidth,
                                    int *piHistogram, bool withoutBottomRight, bool useSmallFilter)
{
  constexpr int        wStep = 1, hStep = 1;
  static constexpr int angTable[17]   = { 0,     2048,  4096,  6144,  8192,  12288, 16384, 20480, 24576,
                                          28672, 32768, 36864, 40960, 47104, 53248, 59392, 65536 };
  static constexpr int offsets[4]     = { HOR_IDX, HOR_IDX, VER_IDX, VER_IDX };
  static constexpr int dirs[4]        = { -1, 1, -1, 1 };
  static constexpr int mapXGrY1[2][2] = { { 1, 0 }, { 0, 1 } };
  static constexpr int mapXGrY0[2][2] = { { 2, 3 }, { 3, 2 } };

  for (uint32_t y = 0; y < uiHeight; y += hStep)
  {
    for (uint32_t x = 0; x < uiWidth; x += wStep)
    {
      if (withoutBottomRight && x == uiWidth - 1 && y == uiHeight - 1)
      {
        continue;
      }

      const Pel *pRec = pReco + y * iStride + x;

      int iDy, iDx;
      if (useSmallFilter)
      {
        // used for small blocks (i.e. 4x4, 8x4 and 4x8)
        // DIMD shifts the reconstruction buffer pointer 2 units in the direction of the ref samples and 1 unit
        // orthogonally (e.g. for above samples, x += 1, y -= 2) that point is the center of the 3x3 filter, but is the
        // top-left corner of the 2x2 one
        iDx = -pRec[0] + pRec[iStride] - pRec[1] + pRec[iStride + 1];
        iDy = pRec[0] + pRec[iStride] - pRec[1] - pRec[iStride + 1];
      }
      else
      {
        iDy =
          pRec[-iStride - 1] + 2 * pRec[-1] + pRec[iStride - 1] - pRec[-iStride + 1] - 2 * pRec[+1] - pRec[iStride + 1];
        iDx = pRec[iStride - 1] + 2 * pRec[iStride] + pRec[iStride + 1] - pRec[-iStride - 1] - 2 * pRec[-iStride] -
          pRec[-iStride + 1];
      }

      if (iDy == 0 && iDx == 0)
      {
        continue;
      }

      int iAmp       = (int)(abs(iDx) + abs(iDy));
      int iAngUneven = -1;
      // for determining region
      if (iDx != 0 && iDy != 0)   // pure angles are not concerned
      {
        // get the region
        int signx  = iDx < 0 ? 1 : 0;
        int signy  = iDy < 0 ? 1 : 0;
        int absx   = iDx < 0 ? -iDx : iDx;
        int absy   = iDy < 0 ? -iDy : iDy;
        int xGrY   = absx > absy ? 1 : 0;
        int region = xGrY ? mapXGrY1[signy][signx] : mapXGrY0[signy][signx];

        // compute ratio
        int s0   = xGrY ? absy : absx;
        int s1   = xGrY ? absx : absy;
        int x    = floorLog2(s1);
        int norm = (s1 << 4 >> x) & 15;
        int v    = g_gradDivTable[norm];
        x += (norm != 0);
        int shift = 13 - x;
        int iRatio;
        if (shift < 0)
        {
          shift   = -shift;
          int add = (1 << (shift - 1));
          iRatio  = (s0 * v + add) >> shift;
        }
        else
        {
          iRatio = (s0 * v) << shift;
        }
        // get ang_idx
        int idx = 16;
        for (int i = 1; i < 17; i++)
        {
          if (iRatio < angTable[i])
          {
            idx = iRatio - angTable[i - 1] < angTable[i] - iRatio ? i - 1 : i;
            break;
          }
        }

        iAngUneven = offsets[region] + dirs[region] * idx;
      }
      else
      {
        iAngUneven = iDx == 0 ? VER_IDX : HOR_IDX;
      }
      piHistogram[iAngUneven] += iAmp;
    }
  }
  return 0;
}

std::pair<int8_t, int8_t> IntraPrediction::deriveIpmForTransform(CPelBuf predBuf, CodingUnit &cu)
{
  std::pair<int8_t, int8_t> ipModes = { 0, 1 };

  int histogram[NUM_LUMA_MODE] = {};
  buildHistogram(predBuf.buf + predBuf.stride + 1, (int)predBuf.stride, predBuf.height - 2, predBuf.width - 2,
                 histogram);

  int firstAmp = 0, secondAmp = 0;
  for (int8_t i = 0; i < (int8_t)NUM_LUMA_MODE; i++)
  {
    int curAmp = histogram[i];
    if (curAmp > firstAmp)
    {
      secondAmp      = firstAmp;
      firstAmp       = curAmp;
      ipModes.second = ipModes.first;
      ipModes.first  = i;
    }
    else if (curAmp > secondAmp)
    {
      secondAmp      = curAmp;
      ipModes.second = i;
    }
  }
  return ipModes;
}

int8_t IntraPrediction::deriveIpmForChromaTransform(CPelBuf predCb, CPelBuf predCr, CodingUnit &cu)
{
  if (!cu.slice->m_sps->m_useDIMD)
  {
    if (PU::isLMCMode(cu.intraDir[ChannelType::CHROMA]))
    {
      return PU::getCoLocatedIntraLumaMode(cu);
    }
    return int8_t { 0 };
  }

  int histogram[NUM_LUMA_MODE] = {};
  if (cu.firstTU->jointCbCr != 2)
  {
    buildHistogram(predCb.buf + predCb.stride + 1, (int)predCb.stride, predCb.height - 2, predCb.width - 2, histogram);
  }
  if (cu.firstTU->jointCbCr != 1)
  {
    buildHistogram(predCr.buf + predCr.stride + 1, (int)predCr.stride, predCr.height - 2, predCr.width - 2, histogram);
  }

  int8_t bestMode = int8_t { 0 };
  int    firstAmp = 0;
  for (int8_t i = 0; i < (int8_t)NUM_LUMA_MODE; i++)
  {
    int curAmp = histogram[i];
    if (curAmp > firstAmp)
    {
      firstAmp = curAmp;
      bestMode = i;
    }
  }
  return bestMode;
}

void IntraPrediction::predTimdIntraAng(const CompID compId, const CodingUnit &cu, uint32_t uiDirMode, Pel *pPred,
                                       ptrdiff_t uiStride, uint32_t iWidth, uint32_t iHeight, TemplateType eTempType,
                                       int32_t iTemplateWidth, int32_t iTemplateHeight)
{
  const CompID compID = MAP_CHROMA(compId);

  const int srcStride  = m_refBufferStride[compID];
  const int srcHStride = 2;

  const CPelBuf &srcBuf = CPelBuf(getPredictorPtr(compID), srcStride, srcHStride);
  const ClpRng  &clpRng(cu.cs->slice->clpRng(compID));

  switch (uiDirMode)
  {
  case (PLANAR_IDX):
    xPredTimdIntraPlanar(srcBuf, pPred, uiStride, iWidth, iHeight, eTempType, iTemplateWidth, iTemplateHeight);
    break;
  case (DC_IDX):
    xPredTimdIntraDc(cu, srcBuf, pPred, uiStride, iWidth, iHeight, eTempType, iTemplateWidth, iTemplateHeight);
    break;
  default:
    xPredTimdIntraAng(srcBuf, clpRng, pPred, uiStride, iWidth, iHeight, eTempType, iTemplateWidth, iTemplateHeight,
                      uiDirMode);
    break;
  }

  if (m_ipaParam.applyPDPC && (uiDirMode == PLANAR_IDX || uiDirMode == DC_IDX))
  {
    xIntraPredTimdPlanarDcPdpc(srcBuf, pPred, uiStride, iWidth, iHeight, eTempType, iTemplateWidth, iTemplateHeight);
  }
}
void IntraPrediction::xIntraPredTimdPlanarDcPdpc(const CPelBuf &pSrc, Pel *pDst, ptrdiff_t iDstStride, int width,
                                                 int height, TemplateType eTempType, int iTemplateWidth,
                                                 int iTemplateHeight)
{
  if (eTempType == LEFT_ABOVE_NEIGHBOR)
  {
    int xOffset = 0;
    int yOffset = 0;
    // PDPC for above template
    {
      const int iWidth  = width;
      const int iHeight = iTemplateHeight;
      xOffset           = iTemplateWidth;
      const int scale   = ((floorLog2(width) - 2 + floorLog2(height) - 2 + 2) >> 2);
      for (int y = 0; y < iHeight; y++)
      {
        const int wT   = 32 >> std::min(31, ((y << 1) >> scale));
        const Pel left = pSrc.at(y + 1, 1);
        for (int x = xOffset; x < iWidth; x++)
        {
          const int wL             = 32 >> std::min(31, ((x << 1) >> scale));
          const Pel top            = pSrc.at(x + 1, 0);
          const Pel val            = pDst[y * iDstStride + x];
          pDst[y * iDstStride + x] = val + ((wL * (left - val) + wT * (top - val) + 32) >> 6);
        }
      }
    }

    // PDPC for left template
    {
      const int iWidth  = iTemplateWidth;
      const int iHeight = height;
      yOffset           = iTemplateHeight;
      const int scale   = ((floorLog2(width) - 2 + floorLog2(height) - 2 + 2) >> 2);
      for (int y = yOffset; y < iHeight; y++)
      {
        const int wT   = 32 >> std::min(31, ((y << 1) >> scale));
        const Pel left = pSrc.at(y + 1, 1);
        for (int x = 0; x < iWidth; x++)
        {
          const int wL             = 32 >> std::min(31, ((x << 1) >> scale));
          const Pel top            = pSrc.at(x + 1, 0);
          const Pel val            = pDst[y * iDstStride + x];
          pDst[y * iDstStride + x] = val + ((wL * (left - val) + wT * (top - val) + 32) >> 6);
        }
      }
    }
  }
  else if (eTempType == LEFT_NEIGHBOR)
  {
    const int iHeight = height;
    const int scale   = ((floorLog2(width) - 2 + floorLog2(height) - 2 + 2) >> 2);
    for (int y = 0; y < iHeight; y++)
    {
      const int wT   = 32 >> std::min(31, ((y << 1) >> scale));
      const Pel left = pSrc.at(y + 1, 1);
      for (int x = 0; x < iTemplateWidth; x++)
      {
        const int wL             = 32 >> std::min(31, ((x << 1) >> scale));
        const Pel top            = pSrc.at(x + 1, 0);
        const Pel val            = pDst[y * iDstStride + x];
        pDst[y * iDstStride + x] = val + ((wL * (left - val) + wT * (top - val) + 32) >> 6);
      }
    }
  }
  else   // eTempType == ABOVE_NEIGHBOR
  {
    const int iWidth = width;
    const int scale  = ((floorLog2(width) - 2 + floorLog2(height) - 2 + 2) >> 2);
    for (int y = 0; y < iTemplateHeight; y++)
    {
      const int wT   = 32 >> std::min(31, ((y << 1) >> scale));
      const Pel left = pSrc.at(y + 1, 1);
      for (int x = 0; x < iWidth; x++)
      {
        const int wL             = 32 >> std::min(31, ((x << 1) >> scale));
        const Pel top            = pSrc.at(x + 1, 0);
        const Pel val            = pDst[y * iDstStride + x];
        pDst[y * iDstStride + x] = val + ((wL * (left - val) + wT * (top - val) + 32) >> 6);
      }
    }
  }
}

void IntraPrediction::xPredTimdIntraPlanar(const CPelBuf &pSrc, Pel *rpDst, ptrdiff_t iDstStride, int width, int height,
                                           TemplateType eTempType, int iTemplateWidth, int iTemplateHeight)
{
  static int leftColumn[MAX_CU_SIZE + TIMDDIFF_MAX_TEMP_SIZE + 1] = { 0 };
  static int topRow[MAX_CU_SIZE + TIMDDIFF_MAX_TEMP_SIZE + 1]     = { 0 };
  static int bottomRow[MAX_CU_SIZE + TIMDDIFF_MAX_TEMP_SIZE]      = { 0 };
  static int rightColumn[MAX_CU_SIZE + TIMDDIFF_MAX_TEMP_SIZE]    = { 0 };
  if (eTempType == LEFT_ABOVE_NEIGHBOR)
  {
    // predict above template
    {
      uint32_t       w      = width - iTemplateWidth;
      const uint32_t log2W  = floorLog2(w);
      const uint32_t log2H  = floorLog2(iTemplateHeight);
      const uint32_t offset = 1 << (log2W + log2H);
      for (int k = 0; k < w + 1; k++)
      {
        topRow[k] = pSrc.at(k + iTemplateWidth + 1, 0);
      }
      for (int k = 0; k < iTemplateHeight + 1; k++)
      {
        leftColumn[k] = pSrc.at(k + 1, 1);
      }

      int bottomLeft = leftColumn[iTemplateHeight];
      int topRight   = topRow[w];
      for (int k = 0; k < w; k++)
      {
        bottomRow[k] = bottomLeft - topRow[k];
        topRow[k]    = topRow[k] << log2H;
      }
      for (int k = 0; k < iTemplateHeight; k++)
      {
        rightColumn[k] = topRight - leftColumn[k];
        leftColumn[k]  = leftColumn[k] << log2W;
      }

      const uint32_t finalShift = 1 + log2W + log2H;
      for (int y = 0; y < iTemplateHeight; y++)
      {
        int horPred = leftColumn[y];
        for (int x = 0; x < w; x++)
        {
          horPred += rightColumn[y];
          topRow[x] += bottomRow[x];
          int vertPred = topRow[x];
          rpDst[y * iDstStride + x + iTemplateWidth] =
            ((horPred << log2H) + (vertPred << log2W) + offset) >> finalShift;
        }
      }
    }

    // predict left template
    {
      uint32_t       h      = height - iTemplateHeight;
      const uint32_t log2W  = floorLog2(iTemplateWidth);
      const uint32_t log2H  = floorLog2(h);
      const uint32_t offset = 1 << (log2W + log2H);
      for (int k = 0; k < h + 1; k++)
      {
        leftColumn[k] = pSrc.at(k + iTemplateHeight + 1, 1);
      }
      for (int k = 0; k < iTemplateWidth + 1; k++)
      {
        topRow[k] = pSrc.at(k + 1, 0);
      }
      int bottomLeft = leftColumn[h];
      int topRight   = topRow[iTemplateWidth];
      for (int k = 0; k < iTemplateWidth; k++)
      {
        bottomRow[k] = bottomLeft - topRow[k];
        topRow[k]    = topRow[k] << log2H;
      }
      for (int k = 0; k < h; k++)
      {
        rightColumn[k] = topRight - leftColumn[k];
        leftColumn[k]  = leftColumn[k] << log2W;
      }
      const uint32_t finalShift = 1 + log2W + log2H;
      for (int y = 0; y < height; y++)
      {
        int horPred = leftColumn[y];
        for (int x = 0; x < iTemplateWidth; x++)
        {
          horPred += rightColumn[y];
          topRow[x] += bottomRow[x];
          int vertPred = topRow[x];
          rpDst[(y + iTemplateHeight) * iDstStride + x] =
            ((horPred << log2H) + (vertPred << log2W) + offset) >> finalShift;
        }
      }
    }
  }
  else if (eTempType == LEFT_NEIGHBOR)
  {
    const uint32_t log2W  = floorLog2(iTemplateWidth);
    const uint32_t log2H  = floorLog2(height);
    const uint32_t offset = 1 << (log2W + log2H);
    for (int k = 0; k < height + 1; k++)
    {
      leftColumn[k] = pSrc.at(k + iTemplateHeight + 1, 1);
    }
    for (int k = 0; k < iTemplateWidth + 1; k++)
    {
      topRow[k] = pSrc.at(k + 1, 0);
    }

    int bottomLeft = leftColumn[height];
    int topRight   = topRow[iTemplateWidth];
    for (int k = 0; k < iTemplateWidth; k++)
    {
      bottomRow[k] = bottomLeft - topRow[k];
      topRow[k]    = topRow[k] << log2H;
    }
    for (int k = 0; k < height; k++)
    {
      rightColumn[k] = topRight - leftColumn[k];
      leftColumn[k]  = leftColumn[k] << log2W;
    }

    const uint32_t finalShift = 1 + log2W + log2H;
    for (int y = 0; y < height; y++)
    {
      int horPred = leftColumn[y];
      for (int x = 0; x < iTemplateWidth; x++)
      {
        horPred += rightColumn[y];
        topRow[x] += bottomRow[x];
        int vertPred              = topRow[x];
        rpDst[y * iDstStride + x] = ((horPred << log2H) + (vertPred << log2W) + offset) >> finalShift;
      }
    }
  }
  else if (eTempType == ABOVE_NEIGHBOR)
  {
    const uint32_t log2W  = floorLog2(width);
    const uint32_t log2H  = floorLog2(iTemplateHeight);
    const uint32_t offset = 1 << (log2W + log2H);
    for (int k = 0; k < width + 1; k++)
    {
      topRow[k] = pSrc.at(k + iTemplateWidth + 1, 0);
    }
    for (int k = 0; k < iTemplateHeight + 1; k++)
    {
      leftColumn[k] = pSrc.at(k + 1, 1);
    }

    int bottomLeft = leftColumn[iTemplateHeight];
    int topRight   = topRow[width];
    for (int k = 0; k < width; k++)
    {
      bottomRow[k] = bottomLeft - topRow[k];
      topRow[k]    = topRow[k] << log2H;
    }
    for (int k = 0; k < iTemplateHeight; k++)
    {
      rightColumn[k] = topRight - leftColumn[k];
      leftColumn[k]  = leftColumn[k] << log2W;
    }

    const uint32_t finalShift = 1 + log2W + log2H;
    for (int y = 0; y < iTemplateHeight; y++)
    {
      int horPred = leftColumn[y];
      for (int x = 0; x < width; x++)
      {
        horPred += rightColumn[y];
        topRow[x] += bottomRow[x];
        int vertPred              = topRow[x];
        rpDst[y * iDstStride + x] = ((horPred << log2H) + (vertPred << log2W) + offset) >> finalShift;
      }
    }
  }
  else
  {
    assert(0);
  }
}
void IntraPrediction::xFillTimdReferenceSamples(const CPelBuf &recoBuf, Pel *refBufUnfiltered, const CompArea &area,
                                                const CodingUnit &cu, int iTemplateWidth, int iTemplateHeight)
{
  const ChannelType      chType = toChannelType(area.compID);
  const CodingStructure &cs     = *cu.cs;
  const SPS             &sps    = *cs.sps;
  const PreCalcValues   &pcv    = *cs.pcv;

  const int tuWidth              = area.width;
  const int tuHeight             = area.height;
  const int predSize             = m_topRefLength;
  const int predHSize            = m_leftRefLength;
  const int predStride           = predSize + 1;
  m_refBufferStride[area.compID] = predStride;

  const bool noShift      = pcv.noChroma2x2 && area.width == 4;   // don't shift on the lowest level (chroma not-split)
  const int unitWidthLog2 = pcv.minCUWidthLog2 - (noShift ? 0 : getComponentScaleX(area.compID, sps.m_chromaFormatIdc));
  const int unitHeightLog2 =
    pcv.minCUHeightLog2 - (noShift ? 0 : getComponentScaleY(area.compID, sps.m_chromaFormatIdc));
  const int unitWidth        = 1 << unitWidthLog2;
  const int unitHeight       = 1 << unitHeightLog2;
  int       leftTempUnitNum  = 0;
  int       aboveTempUnitNum = 0;
  if (iTemplateHeight >= 4)
  {
    leftTempUnitNum = iTemplateHeight >> unitHeightLog2;
  }
  if (iTemplateWidth >= 4)
  {
    aboveTempUnitNum = iTemplateWidth >> unitWidthLog2;
  }

  const int totalAboveUnits = ((predSize + (unitWidth - 1)) >> unitWidthLog2) - aboveTempUnitNum;
  const int totalLeftUnits  = ((predHSize + (unitHeight - 1)) >> unitHeightLog2) - leftTempUnitNum;
  const int totalUnits = totalAboveUnits + totalLeftUnits + 1 + aboveTempUnitNum + leftTempUnitNum;   //+1 for top-left
  const int numAboveUnits      = std::max<int>(tuWidth >> unitWidthLog2, 1);
  const int numLeftUnits       = std::max<int>(tuHeight >> unitHeightLog2, 1);
  const int numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int numLeftBelowUnits  = totalLeftUnits - numLeftUnits;

  // ----- Step 1: analyze neighborhood -----
  const Position posLT = area;
  const Position posRT = area.topRight();
  const Position posLB = area.bottomLeft();

  bool neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];

  int numIntraNeighbor = 0;

  memset(neighborFlags, 0, totalUnits);

  neighborFlags[totalLeftUnits + leftTempUnitNum] =
    isAboveLeftAvailable(cu, chType, posLT.offset(-iTemplateWidth, -iTemplateHeight), 0);
  numIntraNeighbor += neighborFlags[totalLeftUnits + leftTempUnitNum] ? 1 : 0;
  numIntraNeighbor +=
    isLeftAvailable(cu, chType, posLT.offset(-iTemplateWidth, -leftTempUnitNum * unitHeight), leftTempUnitNum,
                    unitHeight, (neighborFlags + totalLeftUnits + leftTempUnitNum - 1), 0);
  numIntraNeighbor +=
    isAboveAvailable(cu, chType, posLT.offset(-aboveTempUnitNum * unitWidth, -iTemplateHeight), aboveTempUnitNum,
                     unitWidth, (neighborFlags + totalLeftUnits + 1 + leftTempUnitNum), 0);
  numIntraNeighbor += isAboveAvailable(cu, chType, posLT.offset(0, -iTemplateHeight), numAboveUnits, unitWidth,
                                       (neighborFlags + totalLeftUnits + 1 + leftTempUnitNum + aboveTempUnitNum), 0);
  numIntraNeighbor +=
    isAboveRightAvailable(cu, chType, posRT.offset(0, -iTemplateHeight), numAboveRightUnits, unitWidth,
                          (neighborFlags + totalLeftUnits + 1 + leftTempUnitNum + aboveTempUnitNum + numAboveUnits), 0);
  numIntraNeighbor += isLeftAvailable(cu, chType, posLT.offset(-iTemplateWidth, 0), numLeftUnits, unitHeight,
                                      (neighborFlags + totalLeftUnits - 1), 0);
  numIntraNeighbor += isBelowLeftAvailable(cu, chType, posLB.offset(-iTemplateWidth, 0), numLeftBelowUnits, unitHeight,
                                           (neighborFlags + totalLeftUnits - 1 - numLeftUnits), 0);

  // ----- Step 2: fill reference samples (depending on neighborhood) -----

  const Pel      *srcBuf    = recoBuf.buf;
  const ptrdiff_t srcStride = recoBuf.stride;
  Pel            *ptrDst    = refBufUnfiltered;
  const Pel      *ptrSrc;
  const Pel       valueDC = 1 << (sps.m_bitDepths[chType] - 1);
  if (numIntraNeighbor == 0)
  {
    // Fill border with DC value
    for (int j = 0; j <= predSize; j++)
    {
      ptrDst[j] = valueDC;
    }
    for (int i = 0; i <= predHSize; i++)
    {
      ptrDst[i + predStride] = valueDC;
    }
  }
  else if (numIntraNeighbor == totalUnits)
  {
    // Fill top-left border and top and top right with rec. samples
    ptrSrc = srcBuf - (1 + iTemplateHeight) * srcStride - (1 + iTemplateWidth);
    for (int j = 0; j <= predSize; j++)
    {
      ptrDst[j] = ptrSrc[j];
    }
    for (int i = 0; i <= predHSize; i++)
    {
      ptrDst[i + predStride] = ptrSrc[i * srcStride];
    }
  }
  else   // reference samples are partially available
  {
    // Fill top-left sample(s) if available
    ptrSrc = srcBuf - (1 + iTemplateHeight) * srcStride - (1 + iTemplateWidth);
    ptrDst = refBufUnfiltered;
    if (neighborFlags[totalLeftUnits + leftTempUnitNum])
    {
      for (int i = 0; i <= iTemplateWidth - (aboveTempUnitNum * unitWidth); i++)
      {
        ptrDst[i] = ptrSrc[i];
      }
      for (int i = 0; i <= iTemplateHeight - leftTempUnitNum; i++)
      {
        ptrDst[i + predStride] = ptrSrc[i * srcStride];
      }
    }

    // Fill left & below-left samples if available (downwards)
    ptrSrc += (1 + iTemplateHeight - leftTempUnitNum * unitHeight) * srcStride;
    ptrDst += (1 + iTemplateHeight - leftTempUnitNum * unitHeight) + predStride;
    for (int unitIdx = totalLeftUnits + leftTempUnitNum - 1; unitIdx > 0; unitIdx--)
    {
      if (neighborFlags[unitIdx])
      {
        for (int i = 0; i < unitHeight; i++)
        {
          ptrDst[i] = ptrSrc[i * srcStride];
        }
      }
      ptrSrc += unitHeight * srcStride;
      ptrDst += unitHeight;
    }
    // Fill last below-left sample(s)
    if (neighborFlags[0])
    {
      int lastSample = (((predHSize - iTemplateHeight) & (unitHeight - 1)) == 0)
        ? unitHeight
        : (predHSize - iTemplateHeight) & (unitHeight - 1);
      for (int i = 0; i < lastSample; i++)
      {
        ptrDst[i] = ptrSrc[i * srcStride];
      }
    }

    // Fill above & above-right samples if available (left-to-right)
    ptrSrc = srcBuf - srcStride * (1 + iTemplateHeight) - aboveTempUnitNum * unitWidth;
    ptrDst = refBufUnfiltered + 1 + iTemplateWidth - aboveTempUnitNum * unitWidth;
    for (int unitIdx = totalLeftUnits + 1 + leftTempUnitNum; unitIdx < totalUnits - 1; unitIdx++)
    {
      if (neighborFlags[unitIdx])
      {
        for (int j = 0; j < unitWidth; j++)
        {
          ptrDst[j] = ptrSrc[j];
        }
      }
      ptrSrc += unitWidth;
      ptrDst += unitWidth;
    }
    // Fill last above-right sample(s)
    if (neighborFlags[totalUnits - 1])
    {
      int lastSample = (((predSize - iTemplateWidth) & (unitWidth - 1)) == 0)
        ? unitWidth
        : (predSize - iTemplateWidth) & (unitWidth - 1);
      for (int j = 0; j < lastSample; j++)
      {
        ptrDst[j] = ptrSrc[j];
      }
    }

    // pad from first available down to the last below-left
    ptrDst            = refBufUnfiltered;
    int lastAvailUnit = 0;
    if (!neighborFlags[0])
    {
      int firstAvailUnit = 1;
      while (firstAvailUnit < totalUnits && !neighborFlags[firstAvailUnit])
      {
        firstAvailUnit++;
      }

      // first available sample
      int firstAvailRow = -1;
      int firstAvailCol = 0;
      if (firstAvailUnit < totalLeftUnits + leftTempUnitNum)
      {
        firstAvailRow = (totalLeftUnits - firstAvailUnit) * unitHeight + iTemplateHeight;
      }
      else if (firstAvailUnit == totalLeftUnits + leftTempUnitNum)
      {
        firstAvailRow = iTemplateHeight - leftTempUnitNum * unitHeight;
      }
      else
      {
        firstAvailCol = (firstAvailUnit - totalLeftUnits - leftTempUnitNum - 1) * unitWidth + 1 + iTemplateWidth -
          aboveTempUnitNum * unitWidth;
      }
      const Pel firstAvailSample = ptrDst[firstAvailRow < 0 ? firstAvailCol : firstAvailRow + predStride];

      // last sample below-left (n.a.)
      int lastRow = predHSize;

      // fill left column
      for (int i = lastRow; i > firstAvailRow; i--)
      {
        ptrDst[i + predStride] = firstAvailSample;
      }
      // fill top row
      if (firstAvailCol > 0)
      {
        for (int j = 0; j < firstAvailCol; j++)
        {
          ptrDst[j] = firstAvailSample;
        }
      }
      lastAvailUnit = firstAvailUnit;
    }

    // pad all other reference samples.
    int currUnit = lastAvailUnit + 1;
    while (currUnit < totalUnits)
    {
      if (!neighborFlags[currUnit])   // samples not available
      {
        // last available sample
        int lastAvailRow = -1;
        int lastAvailCol = 0;
        if (lastAvailUnit < totalLeftUnits + leftTempUnitNum)
        {
          lastAvailRow = (totalLeftUnits + leftTempUnitNum - lastAvailUnit - 1) * unitHeight + iTemplateHeight -
            leftTempUnitNum * unitHeight + 1;
        }
        else if (lastAvailUnit == totalLeftUnits + leftTempUnitNum)
        {
          lastAvailCol = iTemplateWidth - aboveTempUnitNum * unitWidth;
        }
        else
        {
          lastAvailCol = (lastAvailUnit - totalLeftUnits - leftTempUnitNum) * unitWidth + iTemplateWidth -
            aboveTempUnitNum * unitWidth;
        }
        const Pel lastAvailSample = ptrDst[lastAvailRow < 0 ? lastAvailCol : lastAvailRow + predStride];

        // fill current unit with last available sample
        if (currUnit < totalLeftUnits + leftTempUnitNum)
        {
          for (int i = lastAvailRow - 1; i >= lastAvailRow - unitHeight; i--)
          {
            ptrDst[i + predStride] = lastAvailSample;
          }
        }
        else if (currUnit == totalLeftUnits + leftTempUnitNum)
        {
          for (int i = 0; i < iTemplateHeight - leftTempUnitNum * unitHeight + 1; i++)
          {
            ptrDst[i + predStride] = lastAvailSample;
          }
          for (int j = 0; j < iTemplateWidth - aboveTempUnitNum * unitWidth + 1; j++)
          {
            ptrDst[j] = lastAvailSample;
          }
        }
        else
        {
          int numSamplesInUnit = (currUnit == totalUnits - 1)
            ? ((((predSize - iTemplateWidth) & (unitWidth - 1)) == 0) ? unitWidth
                                                                      : (predSize - iTemplateWidth) & (unitWidth - 1))
            : unitWidth;
          for (int j = lastAvailCol + 1; j <= lastAvailCol + numSamplesInUnit; j++)
          {
            ptrDst[j] = lastAvailSample;
          }
        }
      }
      lastAvailUnit = currUnit;
      currUnit++;
    }
  }
}
void IntraPrediction::xIntraPredTimdHorVerPdpc(Pel *pDsty, const ptrdiff_t dstStride, Pel *refSide, const int width,
                                               const int height, int xOffset, int yOffset, int scale,
                                               const Pel *refMain, const ClpRng &clpRng)
{
  const Pel topLeft = refMain[0];

  for (int y = yOffset; y < height; y++)
  {
    memcpy(pDsty, &refMain[1], width * sizeof(Pel));
    const Pel left = refSide[1 + y];
    for (int x = xOffset; x < std::min(3 << scale, width); x++)
    {
      const int wL  = 32 >> (2 * x >> scale);
      const Pel val = pDsty[x];
      pDsty[x]      = ClipPel(val + ((wL * (left - topLeft) + 32) >> 6), clpRng);
    }
    pDsty += dstStride;
  }
}
Pel IntraPrediction::xGetPredTimdValDc(const CPelBuf &pSrc, const Size &dstSize, TemplateType eTempType,
                                       int iTempHeight, int iTempWidth)
{
  int       idx, sum = 0;
  Pel       dcVal;
  const int width     = dstSize.width;
  const int height    = dstSize.height;
  auto      denom     = (width == height) ? (width << 1) : std::max(width, height);
  auto      divShift  = floorLog2(denom);
  auto      divOffset = (denom >> 1);

  if (eTempType == LEFT_NEIGHBOR)
  {
    denom     = height;
    divShift  = floorLog2(denom);
    divOffset = (denom >> 1);
    for (idx = 0; idx < height; idx++)
    {
      sum += pSrc.at(1 + idx, 1);
    }
    dcVal = (sum + divOffset) >> divShift;
    return dcVal;
  }
  else if (eTempType == ABOVE_NEIGHBOR)
  {
    denom     = width;
    divShift  = floorLog2(denom);
    divOffset = (denom >> 1);
    for (idx = 0; idx < width; idx++)
    {
      sum += pSrc.at(1 + idx, 0);
    }
    dcVal = (sum + divOffset) >> divShift;
    return dcVal;
  }

  if (width >= height)
  {
    for (idx = 0; idx < width; idx++)
    {
      sum += pSrc.at(iTempWidth + 1 + idx, 0);
    }
  }
  if (width <= height)
  {
    for (idx = 0; idx < height; idx++)
    {
      sum += pSrc.at(iTempHeight + 1 + idx, 1);
    }
  }
  dcVal = (sum + divOffset) >> divShift;
  return dcVal;
}
void IntraPrediction::xPredTimdIntraDc(const CodingUnit &cu, const CPelBuf &pSrc, Pel *pDst, ptrdiff_t iDstStride,
                                       int iWidth, int iHeight, TemplateType eTempType, int iTemplateWidth,
                                       int iTemplateHeight)
{
  const Size &dstSize = Size(cu.lwidth(), cu.lheight());

  const Pel dcval = xGetPredTimdValDc(pSrc, dstSize, eTempType, iTemplateHeight, iTemplateWidth);
  if (eTempType == LEFT_ABOVE_NEIGHBOR)
  {
    for (int y = 0; y < iHeight; y++, pDst += iDstStride)
    {
      if (y < iTemplateHeight)
      {
        for (int x = iTemplateWidth; x < iWidth; x++)
        {
          pDst[x] = dcval;
        }
      }
      else
      {
        for (int x = 0; x < iTemplateWidth; x++)
        {
          pDst[x] = dcval;
        }
      }
    }
  }
  else if (eTempType == LEFT_NEIGHBOR)
  {
    for (int y = 0; y < iHeight; y++, pDst += iDstStride)
    {
      for (int x = 0; x < iTemplateWidth; x++)
      {
        pDst[x] = dcval;
      }
    }
  }
  else if (eTempType == ABOVE_NEIGHBOR)
  {
    for (int y = 0; y < iTemplateHeight; y++, pDst += iDstStride)
    {
      for (int x = 0; x < iWidth; x++)
      {
        pDst[x] = dcval;
      }
    }
  }
  else
  {
    assert(0);
  }
}
void IntraPrediction::xIntraPredTimdAngPdpc(Pel *pDsty, const ptrdiff_t dstStride, Pel *refSide, const int width,
                                            const int height, int xOffset, int yOffset, int scale, int invAngle)
{
  int xlim = std::min(3 << scale, width);
  for (int y = yOffset; y < height; y++)
  {
    int invAngleSum = 256;
    if (width < 4)
    {
      for (int x = xOffset; x < 2; x++)
      {
        invAngleSum += invAngle;
        int wL   = 32 >> (2 * x >> scale);
        Pel left = refSide[y + (invAngleSum >> 9) + 1];
        pDsty[x] = pDsty[x] + ((wL * (left - pDsty[x]) + 32) >> 6);
      }
    }
    else
    {
      for (int x = xOffset; x < xlim; x++)
      {
        invAngleSum += invAngle;
        int wL   = 32 >> (2 * x >> scale);
        Pel left = refSide[y + (invAngleSum >> 9) + 1];
        pDsty[x] = pDsty[x] + ((wL * (left - pDsty[x]) + 32) >> 6);
      }
    }
    pDsty += dstStride;
  }
}

void IntraPrediction::xIntraPredTimdAngGradPdpc(Pel *pDsty, const ptrdiff_t dstStride, Pel *refMain, Pel *refSide,
                                                const int width, const int height, int xOffset, int yOffset, int scale,
                                                int deltaPos, int intraPredAngle, const ClpRng &clpRng)
{
  for (int y = yOffset; y < height; y++)
  {
    const int deltaInt   = deltaPos >> 6;
    const int deltaFract = deltaPos & 63;
    const Pel left       = refSide[1 + y];
    const Pel topLeft    = refMain[deltaInt] + ((deltaFract * (refMain[deltaInt + 1] - refMain[deltaInt]) + 32) >> 6);
    for (int x = xOffset; x < std::min(3 << scale, width); x++)
    {
      int wL   = 32 >> (2 * (x - xOffset) >> scale);
      pDsty[x] = ClipPel(pDsty[x] + ((wL * (left - topLeft) + 32) >> 6), clpRng);
    }
    pDsty += dstStride;
    deltaPos += intraPredAngle;
  }
}

void IntraPrediction::xIntraPredTimdAngLuma(Pel *pDstBuf, const ptrdiff_t dstStride, Pel *refMain, int width,
                                            int height, int deltaPos, int intraPredAngle, const ClpRng &clpRng,
                                            int xOffset, int yOffset)
{
  for (int y = yOffset; y < height; y++)
  {
    const int                 deltaInt     = deltaPos >> 6;
    const int                 deltaFract   = deltaPos & 63;
    const TFilterCoeff *const f            = InterpolationFilter::getExtIntraCubicFilter(deltaFract);
    int                       refMainIndex = deltaInt + 1 + xOffset;
    for (int x = xOffset; x < width; x++, refMainIndex++)
    {
      pDstBuf[y * dstStride + x] = (f[0] * refMain[refMainIndex - 1] + f[1] * refMain[refMainIndex] +
                                    f[2] * refMain[refMainIndex + 1] + f[3] * refMain[refMainIndex + 2] + 128) >>
        8;
      pDstBuf[y * dstStride + x] =
        ClipPel(pDstBuf[y * dstStride + x], clpRng);   // always clip even though not always needed
    }
    deltaPos += intraPredAngle;
  }
}
void IntraPrediction::xPredTimdIntraAng(const CPelBuf &pSrc, const ClpRng &clpRng, Pel *pTrueDst, ptrdiff_t iDstStride,
                                        int iWidth, int iHeight, TemplateType eTempType, int iTemplateWidth,
                                        int iTemplateHeight, uint32_t dirMode)
{
  int        width          = iWidth;
  int        height         = iHeight;
  const bool bIsModeVer     = m_ipaParam.isModeVer;
  const int  intraPredAngle = m_ipaParam.intraPredAngle;
  const int  invAngle       = m_ipaParam.absInvAngle;
  Pel       *refMain;
  Pel       *refSide;
  static Pel refAbove[2 * MAX_CU_SIZE + 5 + 33 * MAX_REF_LINE_IDX];
  static Pel refLeft[2 * MAX_CU_SIZE + 5 + 33 * MAX_REF_LINE_IDX];

  // Initialize the Main and Left reference array.
  if (intraPredAngle < 0)
  {
    for (int x = 0; x <= width + 1; x++)
    {
      refAbove[x + height] = pSrc.at(x, 0);
    }
    for (int y = 0; y <= height + 1; y++)
    {
      refLeft[y + width] = pSrc.at(y, 1);
    }
    refMain      = bIsModeVer ? refAbove + height : refLeft + width;
    refSide      = bIsModeVer ? refLeft + width : refAbove + height;
    // Extend the Main reference to the left.
    int sizeSide = bIsModeVer ? height : width;
    for (int k = -sizeSide; k <= -1; k++)
    {
      refMain[k] = refSide[std::min((-k * invAngle + 256) >> 9, sizeSide)];
    }
  }
  else
  {
    for (int x = 0; x <= m_topRefLength; x++)
    {
      refAbove[x] = pSrc.at(x, 0);
    }
    for (int y = 0; y <= m_leftRefLength; y++)
    {
      refLeft[y] = pSrc.at(y, 1);
    }
    refMain             = bIsModeVer ? refAbove : refLeft;
    refSide             = bIsModeVer ? refLeft : refAbove;
    // Extend main reference to right using replication
    const int log2Ratio = floorLog2(width - iTemplateWidth) - floorLog2(height - iTemplateHeight);
    const int s         = std::max<int>(0, bIsModeVer ? log2Ratio : -log2Ratio);
    const int maxIndex =
      (std::max(iTemplateWidth, iTemplateHeight) << s) + 2 + std::max(iTemplateWidth, iTemplateHeight);
    const int refLength = bIsModeVer ? m_topRefLength : m_leftRefLength;
    const Pel val       = refMain[refLength];
    for (int z = 1; z <= maxIndex; z++)
    {
      refMain[refLength + z] = val;
    }
  }

  // swap width/height if we are doing a horizontal mode:
  static Pel      tempArray[(MAX_CU_SIZE + TIMDDIFF_MAX_TEMP_SIZE) *
                       (MAX_CU_SIZE + TIMDDIFF_MAX_TEMP_SIZE)];   ///< buffer size may not be big enough
  const ptrdiff_t dstStride = bIsModeVer ? iDstStride : (MAX_CU_SIZE + TIMDDIFF_MAX_TEMP_SIZE);
  Pel            *pDst      = bIsModeVer ? pTrueDst : tempArray;
  if (!bIsModeVer)
  {
    std::swap(width, height);
    std::swap(iTemplateWidth, iTemplateHeight);
  }

  if (intraPredAngle == 0)   // pure vertical or pure horizontal
  {
    if (eTempType == LEFT_ABOVE_NEIGHBOR)
    {
      if (m_ipaParam.applyPDPC)
      {
        int scale = (floorLog2(width) + floorLog2(height) - 2) >> 2;
        xIntraPredTimdHorVerPdpc(pDst, dstStride, refSide, width, iTemplateHeight, iTemplateWidth, 0, scale, refMain,
                                 clpRng);
        xIntraPredTimdHorVerPdpc(pDst + iTemplateHeight * dstStride, dstStride, refSide, iTemplateWidth, height, 0,
                                 iTemplateHeight, scale, refMain, clpRng);
      }
      else
      {
        for (int y = 0; y < iTemplateHeight; y++)
        {
          memcpy(pDst + y * dstStride + iTemplateWidth, &refMain[iTemplateWidth + 1],
                 (width - iTemplateWidth) * sizeof(Pel));
        }
        for (int y = iTemplateHeight; y < height; y++)
        {
          memcpy(pDst + y * dstStride, &refMain[1], iTemplateWidth * sizeof(Pel));
        }
      }
    }
    else if (eTempType == LEFT_NEIGHBOR || eTempType == ABOVE_NEIGHBOR)
    {
      if ((eTempType == LEFT_NEIGHBOR && bIsModeVer) || (eTempType == ABOVE_NEIGHBOR && !bIsModeVer))
      {
        if (m_ipaParam.applyPDPC)
        {
          const int scale = (floorLog2(width) + floorLog2(height) - 2) >> 2;
          xIntraPredTimdHorVerPdpc(pDst, dstStride, refSide, iTemplateWidth, height, 0, 0, scale, refMain, clpRng);
        }
        else
        {
          for (int y = 0; y < height; y++)
          {
            for (int x = 0; x < iTemplateWidth; x++)
            {
              pDst[y * dstStride + x] = refMain[x + 1];
            }
          }
        }
      }
      else
      {
        if (m_ipaParam.applyPDPC)
        {
          const int scale = (floorLog2(width) + floorLog2(height) - 2) >> 2;
          xIntraPredTimdHorVerPdpc(pDst, dstStride, refSide, width, iTemplateHeight, 0, 0, scale, refMain, clpRng);
        }
        else
        {
          for (int y = 0; y < iTemplateHeight; y++)
          {
            memcpy(pDst + y * dstStride, &refMain[1], width * sizeof(Pel));
          }
        }
      }
    }
    else
    {
      assert(0);
    }
  }
  else
  {
    Pel *pDsty = pDst;
    if (!isIntegerSlopeExt(abs(intraPredAngle)))
    {
      int deltaPos = intraPredAngle;
      if (eTempType == LEFT_ABOVE_NEIGHBOR)
      {
        Pel *pDsty = pDst;
        // Above template
        xIntraPredTimdAngLuma(pDsty, dstStride, refMain, width, iTemplateHeight, deltaPos, intraPredAngle, clpRng,
                              iTemplateWidth, 0);
        // Left template
        for (int y = 0; y < iTemplateHeight; y++)
        {
          deltaPos += intraPredAngle;
        }
        xIntraPredTimdAngLuma(pDsty, dstStride, refMain, iTemplateWidth, height, deltaPos, intraPredAngle, clpRng, 0,
                              iTemplateHeight);

        if (m_ipaParam.applyPDPC && m_ipaParam.useGradPDPC)
        {
          int       deltaPos2 = intraPredAngle;
          const int scale     = m_ipaParam.angularScale;
          xIntraPredTimdAngGradPdpc(pDst, dstStride, refMain, refSide, width, iTemplateHeight, iTemplateWidth, 0, scale,
                                    deltaPos2, intraPredAngle, clpRng);
          for (int y = 0; y < iTemplateHeight; y++)
          {
            deltaPos2 += intraPredAngle;
          }
          xIntraPredTimdAngGradPdpc(pDst + iTemplateHeight * dstStride, dstStride, refMain, refSide, iTemplateWidth,
                                    height, 0, iTemplateHeight, scale, deltaPos2, intraPredAngle, clpRng);
        }
        else if (m_ipaParam.applyPDPC)
        {
          const int scale = m_ipaParam.angularScale;
          xIntraPredTimdAngPdpc(pDst, dstStride, refSide, width, iTemplateHeight, iTemplateWidth, 0, scale, invAngle);
          xIntraPredTimdAngPdpc(pDst + iTemplateHeight * dstStride, dstStride, refSide, iTemplateWidth, height, 0,
                                iTemplateHeight, scale, invAngle);
        }
      }
      else if (eTempType == LEFT_NEIGHBOR || eTempType == ABOVE_NEIGHBOR)
      {
        int iRegionWidth, iRegionHeight;
        if ((eTempType == LEFT_NEIGHBOR && bIsModeVer) || (eTempType == ABOVE_NEIGHBOR && !bIsModeVer))
        {
          iRegionWidth  = iTemplateWidth;
          iRegionHeight = height;
        }
        else
        {
          iRegionWidth  = width;
          iRegionHeight = iTemplateHeight;
        }
        xIntraPredTimdAngLuma(pDsty, dstStride, refMain, iRegionWidth, iRegionHeight, deltaPos, intraPredAngle, clpRng,
                              0, 0);
        if (m_ipaParam.applyPDPC && m_ipaParam.useGradPDPC)
        {
          int       deltaPos2 = intraPredAngle;
          const int scale     = m_ipaParam.angularScale;
          xIntraPredTimdAngGradPdpc(pDst, dstStride, refMain, refSide, iRegionWidth, iRegionHeight, 0, 0, scale,
                                    deltaPos2, intraPredAngle, clpRng);
        }
        else if (m_ipaParam.applyPDPC)
        {
          const int scale = m_ipaParam.angularScale;
          xIntraPredTimdAngPdpc(pDst, dstStride, refSide, iRegionWidth, iRegionHeight, 0, 0, scale, invAngle);
        }
      }
    }
    else
    {
      if (eTempType == LEFT_ABOVE_NEIGHBOR)
      {
        Pel *pDsty = pDst;
        for (int y = 0, deltaPos = intraPredAngle; y < height; y++, deltaPos += intraPredAngle, pDsty += dstStride)
        {
          const int deltaInt = deltaPos >> 6;
          int       iStartIdx, iEndIdx;
          if (y < iTemplateHeight)
          {
            iStartIdx = iTemplateWidth;
            iEndIdx   = width - 1;
          }
          else
          {
            iStartIdx = 0;
            iEndIdx   = iTemplateWidth - 1;
          }
          memcpy(pDsty + iStartIdx, &refMain[iStartIdx + deltaInt + 1], (iEndIdx - iStartIdx + 1) * sizeof(Pel));
        }

        if (m_ipaParam.applyPDPC && m_ipaParam.useGradPDPC)
        {
          int       deltaPos2 = intraPredAngle;
          const int scale     = m_ipaParam.angularScale;
          xIntraPredTimdAngGradPdpc(pDst, dstStride, refMain, refSide, width, iTemplateHeight, iTemplateWidth, 0, scale,
                                    deltaPos2, intraPredAngle, clpRng);
          for (int y = 0; y < iTemplateHeight; y++)
          {
            deltaPos2 += intraPredAngle;
          }
          xIntraPredTimdAngGradPdpc(pDst + iTemplateHeight * dstStride, dstStride, refMain, refSide, iTemplateWidth,
                                    height, 0, iTemplateHeight, scale, deltaPos2, intraPredAngle, clpRng);
        }
        else

          if (m_ipaParam.applyPDPC)
        {
          const int scale = m_ipaParam.angularScale;
          xIntraPredTimdAngPdpc(pDst, dstStride, refSide, width, iTemplateHeight, iTemplateWidth, 0, scale, invAngle);
          xIntraPredTimdAngPdpc(pDst + iTemplateHeight * dstStride, dstStride, refSide, iTemplateWidth, height, 0,
                                iTemplateHeight, scale, invAngle);
        }
      }
      else   // if (eTempType == LEFT_NEIGHBOR || eTempType == ABOVE_NEIGHBOR)
      {
        Pel *pDsty = pDst;
        assert(eTempType == LEFT_NEIGHBOR || eTempType == ABOVE_NEIGHBOR);
        int iRegionWidth, iRegionHeight;
        if ((eTempType == LEFT_NEIGHBOR && bIsModeVer) || (eTempType == ABOVE_NEIGHBOR && !bIsModeVer))
        {
          iRegionWidth  = iTemplateWidth;
          iRegionHeight = height;
        }
        else
        {
          iRegionWidth  = width;
          iRegionHeight = iTemplateHeight;
        }
        for (int y = 0, deltaPos = intraPredAngle; y < iRegionHeight;
             y++, deltaPos += intraPredAngle, pDsty += dstStride)
        {
          const int deltaInt = deltaPos >> 6;

          memcpy(pDsty, &refMain[deltaInt + 1], iRegionWidth * sizeof(Pel));
        }

        if (m_ipaParam.applyPDPC && m_ipaParam.useGradPDPC)
        {
          int       deltaPos2 = intraPredAngle;
          const int scale     = m_ipaParam.angularScale;
          xIntraPredTimdAngGradPdpc(pDst, dstStride, refMain, refSide, iRegionWidth, iRegionHeight, 0, 0, scale,
                                    deltaPos2, intraPredAngle, clpRng);
        }
        else

          if (m_ipaParam.applyPDPC)
        {
          const int scale = m_ipaParam.angularScale;
          xIntraPredTimdAngPdpc(pDst, dstStride, refSide, iRegionWidth, iRegionHeight, 0, 0, scale, invAngle);
        }
      }
    }
  }

  // Flip the block if this is the horizontal mode
  if (!bIsModeVer)
  {
    if (eTempType == LEFT_ABOVE_NEIGHBOR)
    {
      for (int y = 0; y < height; y++)
      {
        int iStartIdx, iEndIdx;
        if (y < iTemplateHeight)
        {
          iStartIdx = iTemplateWidth;
          iEndIdx   = width - 1;
        }
        else
        {
          iStartIdx = 0;
          iEndIdx   = iTemplateWidth - 1;
        }
        for (int x = iStartIdx; x <= iEndIdx; x++)
        {
          pTrueDst[x * iDstStride + y] = pDst[y * dstStride + x];
        }
      }
    }
    else if (eTempType == LEFT_NEIGHBOR)
    {
      for (int y = 0; y < iTemplateHeight; y++)
      {
        for (int x = 0; x < width; x++)
        {
          pTrueDst[x * iDstStride + y] = pDst[y * dstStride + x];
        }
      }
    }
    else if (eTempType == ABOVE_NEIGHBOR)
    {
      for (int y = 0; y < height; y++)
      {
        for (int x = 0; x < iTemplateWidth; x++)
        {
          pTrueDst[x * iDstStride + y] = pDst[y * dstStride + x];
        }
      }
    }
    else
    {
      assert(0);
    }
  }
}
void IntraPrediction::initPredTimdIntraParams(const CodingUnit &cu, const CompArea area, int dirMode, bool bSgpm)
{
  const Size  puSize    = Size(area.width, area.height);
  const Size &blockSize = puSize;
  const int   predMode  = getWideAngleExt(blockSize.width, blockSize.height, dirMode, bSgpm);

  m_ipaParam.isModeVer         = predMode >= EXT_DIA_IDX;
  m_ipaParam.refFilterFlag     = false;
  m_ipaParam.interpolationFlag = false;
  m_ipaParam.applyPDPC         = puSize.width >= MIN_TB_SIZEY && puSize.height >= MIN_TB_SIZEY;
  const int intraPredAngleMode = (m_ipaParam.isModeVer) ? predMode - EXT_VER_IDX : -(predMode - EXT_HOR_IDX);

  int              absAng             = 0;
  static const int extAngTable[64]    = { 0,   1,   2,   3,   4,   5,   6,   7,   8,    10,   12,   14,  16,
                                          18,  20,  22,  24,  26,  28,  30,  32,  34,   36,   38,   40,  43,
                                          46,  49,  52,  55,  58,  61,  64,  67,  70,   74,   78,   84,  90,
                                          96,  102, 108, 114, 121, 128, 137, 146, 159,  172,  188,  204, 230,
                                          256, 299, 342, 427, 512, 597, 682, 853, 1024, 1536, 2048, 3072 };
  static const int extInvAngTable[64] = {
    0,    32768, 16384, 10923, 8192, 6554, 5461, 4681, 4096, 3277, 2731, 2341, 2048, 1820, 1638, 1489,
    1365, 1260,  1170,  1092,  1024, 964,  910,  862,  819,  762,  712,  669,  630,  596,  565,  537,
    512,  489,   468,   443,   420,  390,  364,  341,  321,  303,  287,  271,  256,  239,  224,  206,
    191,  174,   161,   142,   128,  110,  96,   77,   64,   55,   48,   38,   32,   21,   16,   11
  };   // (512 * 64) / Angle

  const int absAngMode = abs(intraPredAngleMode);
  const int signAng    = intraPredAngleMode < 0 ? -1 : 1;
  absAng               = extAngTable[absAngMode];

  m_ipaParam.absInvAngle    = extInvAngTable[absAngMode];
  m_ipaParam.intraPredAngle = signAng * absAng;

  if (dirMode > 1)
  {
    if (intraPredAngleMode < 0)
    {
      m_ipaParam.applyPDPC = false;
    }
    else if (intraPredAngleMode > 0)
    {
      const int sideSize = m_ipaParam.isModeVer ? puSize.height : puSize.width;
      const int maxScale = 2;

      m_ipaParam.useGradPDPC = false;

      m_ipaParam.angularScale =
        std::min(maxScale, floorLog2(sideSize) - (floorLog2(3 * m_ipaParam.absInvAngle - 2) - 8));

      if (m_ipaParam.angularScale < 0)
      {
        m_ipaParam.angularScale = (floorLog2(puSize.width) + floorLog2(puSize.height) - 2) >> 2;
        m_ipaParam.useGradPDPC  = true;
      }

      m_ipaParam.applyPDPC &= m_ipaParam.angularScale >= 0;
    }
  }
}
void IntraPrediction::initTimdIntraPatternLuma(const CodingUnit &cu, const CompArea &area, int iTemplateWidth,
                                               int iTemplateHeight, uint32_t uiRefWidth, uint32_t uiRefHeight)
{
  const CodingStructure &cs               = *cu.cs;
  Pel                   *refBufUnfiltered = m_refBuffer[area.compID][PRED_BUF_UNFILTERED];
  bool                   bLeftAbove       = iTemplateHeight > 0 && iTemplateWidth > 0;
  m_leftRefLength                         = bLeftAbove ? (uiRefHeight << 1) : ((uiRefHeight + iTemplateHeight) << 1);
  m_topRefLength                          = bLeftAbove ? (uiRefWidth << 1) : ((uiRefWidth + iTemplateWidth) << 1);
  xFillTimdReferenceSamples(cs.picture->getRecoBuf(area), refBufUnfiltered, area, cu, iTemplateWidth, iTemplateHeight);
}

std::pair<int, int> calculateTimdTemplateSize(const CodingUnit                           &cu,
                                              const IntraPrediction::TimdDerivationMethod timdDerivationMode)
{
  int width  = (cu.lwidth() <= 8 ? 2 : 4);
  int height = (cu.lheight() <= 8 ? 2 : 4);

  if (timdDerivationMode != IntraPrediction::TimdDerivationMethod::FullWithSAD)
  {
    return { width, height };
  }

  width *= 2;
  width *= 2;

  // clip templates:
  width  = std::min(std::max(cu.lx(), 2), width);
  height = std::min(std::max(cu.ly(), 2), height);

  const auto isPositionAvailabe = [&cu](const Position &pos)
  { return cu.cs->isDecomp(pos, ChannelType::LUMA) && cu.cs->getCURestricted(pos, cu, ChannelType::LUMA) != nullptr; };

  const int maxTempHeight = std::min(height, TIMDDIFF_MAX_TEMP_SIZE);
  for (height = maxTempHeight; height > 4; height -= 4)
  {
    const auto refPosL = cu.Y().offset(0, -height);
    const auto refPosR = cu.Y().offset(cu.lwidth() - 1, -height);

    if (isPositionAvailabe(refPosL) && isPositionAvailabe(refPosR))
    {
      break;
    }
  }

  const int maxTempWidth = std::min(width, TIMDDIFF_MAX_TEMP_SIZE);
  for (width = maxTempWidth; width > 4; width -= 4)
  {
    const auto refPosT = cu.Y().offset(-width, 0);
    const auto refPosB = cu.Y().offset(-width, cu.lheight() - 1);

    if (isPositionAvailabe(refPosT) && isPositionAvailabe(refPosB))
    {
      break;
    }
  }

  return { width, height };
}

bool doesNeighbourhoodUseBothPlanarAndDC(const CodingUnit &pu)
{
  const auto topLeft    = pu.Y().topLeft();
  const auto topRight   = pu.Y().topRight();
  const auto bottomLeft = pu.Y().bottomLeft();

  bool planarFound {};
  bool dcFound {};

  struct Neighbour
  {
    Position offset {};
    bool     checkForSameCtu {};
  };

  const std::initializer_list<Neighbour> neighbourhood { { bottomLeft.offset(-1, 0), false },   // left
                                                         { topRight.offset(0, -1), true },   // above
                                                         { bottomLeft.offset(-1, 1), false },   // below left
                                                         { topRight.offset(1, -1), false },   // above right
                                                         { topLeft.offset(-1, -1), false } };   // above left

  for (const auto &neighbour: neighbourhood)
  {
    const auto neighbourPu = pu.cs->getCURestricted(neighbour.offset, pu, pu.chType);
    if (neighbourPu == nullptr || !CU::isIntra(*neighbourPu))
    {
      continue;
    }
    if (neighbour.checkForSameCtu && !CU::isSameCtu(pu, *neighbourPu))
    {
      continue;
    }

    const auto iMode = PU::getIntraDirLuma(*neighbourPu);
    if (iMode == PLANAR_IDX)
    {
      planarFound = true;
    }
    else if (iMode == DC_IDX)
    {
      dcFound = true;
    }
    else
    {
      return false;
    }
  }

  return (planarFound && dcFound);
}

static_vector<unsigned, NUM_MOST_PROBABLE_MODES + 3> createMostProbableModeListWithDCVerHor(const CodingUnit &pu)
{
  std::array<uint8_t, NUM_MOST_PROBABLE_MODES> mpmList;
  std::array<uint8_t, NUM_NON_MPM_MODES>       intraNonMPM;

  PU::getIntraMPMs(pu, mpmList.data(), intraNonMPM.data());

  static_vector<unsigned, NUM_MOST_PROBABLE_MODES + 3> mpmExtraList;

  bool foundDC  = false;
  bool foundHor = false;
  bool foundVer = false;

  for (const auto iMode: mpmList)
  {
    if (iMode == DC_IDX)
    {
      foundDC = true;
    }
    if (iMode == HOR_IDX)
    {
      foundHor = true;
    }
    if (iMode == VER_IDX)
    {
      foundVer = true;
    }

    mpmExtraList.push_back(iMode);
  }

  if (!foundDC)
  {
    mpmExtraList.push_back(DC_IDX);
  }
  if (!foundHor)
  {
    mpmExtraList.push_back(HOR_IDX);
  }
  if (!foundVer)
  {
    mpmExtraList.push_back(VER_IDX);
  }

  return mpmExtraList;
}

static_vector<unsigned, NUM_MOST_PROBABLE_MODES + 3>
  createMostProbableModeListBasedOnTimdModeCostList(const CodingUnit                        &cu,
                                                    const IntraPrediction::TimdModeCostList &timdModeCostList)
{
  static_vector<unsigned, NUM_MOST_PROBABLE_MODES + 3> mpmExtraList;
  std::array<bool, NUM_LUMA_MODE>                      modeUsedOrIgnored {};

  enum class AddModeToListResult
  {
    Added,
    NotAdded,
    ListFull
  };

  auto checkModeAndAddToList = [&mpmExtraList, &modeUsedOrIgnored, &cu](const int iMode, const int maxNrModes)
  {
    constexpr bool IGNORE_MODE  = true;
    constexpr bool MODE_IN_LIST = true;

    if (mpmExtraList.size() >= maxNrModes)
    {
      return AddModeToListResult::ListFull;
    }

    const auto isPrimaryOrSecondaryTimdMode = (MAP67TO131(iMode) == cu.timdData.blendMode[0] ||
                                               (cu.timdData.isBlend && MAP67TO131(iMode) == cu.timdData.blendMode[1]));
    if (isPrimaryOrSecondaryTimdMode)
    {
      modeUsedOrIgnored[iMode] = IGNORE_MODE;
    }

    if (modeUsedOrIgnored[iMode] || iMode < 0 || iMode >= NUM_LUMA_MODE)
    {
      return AddModeToListResult::NotAdded;
    }

    mpmExtraList.push_back(iMode);
    modeUsedOrIgnored[iMode] = MODE_IN_LIST;

    return AddModeToListResult::Added;
  };

  const auto maxModeNum = std::min(static_cast<int>(timdModeCostList.size()), TIMD_NUM_MODES_SORTED);
  for (const auto &[cost, iMode]: timdModeCostList)
  {
    if (iMode <= DC_IDX)
    {
      continue;
    }

    if (checkModeAndAddToList(iMode, maxModeNum) == AddModeToListResult::ListFull)
    {
      break;
    }
  }

  const int maximumNumberCandidates = TIMD_NUM_MODES_SORTED + 10;
  for (int i = 0; i < DIMD_FUSION_NUM - 1; i++)
  {
    if (i == 1 && !cu.dimdData.isBlend)
    {
      break;
    }

    const auto iMode = cu.dimdData.blendMode[i];
    if (iMode <= DC_IDX)
    {
      continue;
    }

    if (checkModeAndAddToList(iMode, maximumNumberCandidates) == AddModeToListResult::ListFull)
    {
      return mpmExtraList;
    }
  }

  for (const auto iMode: { HOR_IDX, VER_IDX })
  {
    if (checkModeAndAddToList(iMode, maximumNumberCandidates) == AddModeToListResult::ListFull)
    {
      return mpmExtraList;
    }
  }

  const auto AdditionalModes = { MAP131TO67(cu.timdData.blendMode[0]) + 1, MAP131TO67(cu.timdData.blendMode[0]) - 1,
                                 MAP131TO67(cu.timdData.blendMode[1]) + 1, MAP131TO67(cu.timdData.blendMode[1]) - 1,
                                 MAP131TO67(cu.timdData.blendMode[0]) + 2, MAP131TO67(cu.timdData.blendMode[0]) - 2,
                                 MAP131TO67(cu.timdData.blendMode[1]) + 2, MAP131TO67(cu.timdData.blendMode[1]) - 2 };
  for (const auto iMode: AdditionalModes)
  {
    if (iMode < DC_IDX || iMode >= NUM_LUMA_MODE)
    {
      continue;
    }

    if (checkModeAndAddToList(iMode, maximumNumberCandidates) == AddModeToListResult::ListFull)
    {
      return mpmExtraList;
    }
  }

  return mpmExtraList;
}

int IntraPrediction::deriveTimdMode(const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu,
                                    const TimdDerivationMethod timdDerivationMethod)
{
  auto &timdData = (timdDerivationMethod == TimdDerivationMethod::FullWithSAD ? cu.timdSadData : cu.timdData);

  int      channelBitDepth = cu.slice->m_sps->m_bitDepths[toChannelType(COMP_Y)];
  SizeType uiWidth         = cu.lwidth();
  SizeType uiHeight        = cu.lheight();

  const auto TIMD_SAD_MAX_TEMP_SIZE = 8;
  static Pel predLuma[(MAX_CU_SIZE + TIMD_SAD_MAX_TEMP_SIZE) * (MAX_CU_SIZE + TIMD_SAD_MAX_TEMP_SIZE)];
  Pel       *piPred       = predLuma;
  ptrdiff_t  uiPredStride = MAX_CU_SIZE + TIMD_SAD_MAX_TEMP_SIZE;

  const auto [iTemplateWidth, iTemplateHeight] = calculateTimdTemplateSize(cu, timdDerivationMethod);

  const auto templateTypePositionAndSize = CU::deriveTimdRefTypePositionAndSize(cu, iTemplateWidth, iTemplateHeight);
  const auto eTemplateType               = templateTypePositionAndSize.eTemplateType;
  const auto [iRefX, iRefY]              = templateTypePositionAndSize.iRefPosition;
  const auto [uiRefWidth, uiRefHeight]   = templateTypePositionAndSize.uiRefSize;

  std::array<bool, NUM_LUMA_MODE> isModeAddedToTimdModeCostList {};
  const bool fillTimdModeCostList = (timdDerivationMethod == TimdDerivationMethod::Full && CU::allowTimdSad(cu));
  if (fillTimdModeCostList)
  {
    m_timdModeCostList.clear();
  }

  const uint32_t log2A = floorLog2(iTemplateHeight) + floorLog2(uiWidth);
  const uint32_t log2L = floorLog2(iTemplateWidth) + floorLog2(uiHeight);

  if (eTemplateType != NO_NEIGHBOR)
  {
    const CodingStructure &cs = *cu.cs;
    m_ipaParam.multiRefIndex  = iTemplateWidth;
    Pel *piOrg                = cs.picture->getRecoBuf(area).buf;
    int  iOrgStride           = static_cast<int>(cs.picture->getRecoBuf(area).stride);
    piOrg += (iRefY - cu.ly()) * iOrgStride + (iRefX - cu.lx());
    DistParam distParamSad[2];   // above, left
    distParamSad[0].applyWeight = false;
    distParamSad[0].useMR       = (timdDerivationMethod == TimdDerivationMethod::FullWithSAD);
    distParamSad[1].applyWeight = false;
    distParamSad[1].useMR       = (timdDerivationMethod == TimdDerivationMethod::FullWithSAD);
    const auto useHadamard      = (timdDerivationMethod == TimdDerivationMethod::FullWithSAD ? 0 : 1);
    if (eTemplateType == LEFT_ABOVE_NEIGHBOR)
    {
      m_timdSatdCost->setTimdDistParam(distParamSad[0], piOrg + iTemplateWidth, piPred + iTemplateWidth, iOrgStride,
                                       uiPredStride, channelBitDepth, COMP_Y, uiWidth, iTemplateHeight, 0, 1,
                                       useHadamard);   // Use HAD (SATD) cost
      m_timdSatdCost->setTimdDistParam(
        distParamSad[1], piOrg + iTemplateHeight * iOrgStride, piPred + iTemplateHeight * uiPredStride, iOrgStride,
        uiPredStride, channelBitDepth, COMP_Y, iTemplateWidth, uiHeight, 0, 1, useHadamard);   // Use HAD (SATD) cost
    }
    else if (eTemplateType == LEFT_NEIGHBOR)
    {
      m_timdSatdCost->setTimdDistParam(distParamSad[1], piOrg, piPred, iOrgStride, uiPredStride, channelBitDepth,
                                       COMP_Y, iTemplateWidth, uiHeight, 0, 1, useHadamard);
    }
    else if (eTemplateType == ABOVE_NEIGHBOR)
    {
      m_timdSatdCost->setTimdDistParam(distParamSad[0], piOrg, piPred, iOrgStride, uiPredStride, channelBitDepth,
                                       COMP_Y, uiWidth, iTemplateHeight, 0, 1, useHadamard);
    }
    initTimdIntraPatternLuma(cu, area, eTemplateType != ABOVE_NEIGHBOR ? iTemplateWidth : 0,
                             eTemplateType != LEFT_NEIGHBOR ? iTemplateHeight : 0, uiRefWidth, uiRefHeight);

    auto    &pu      = cu;
    uint32_t uiRealW = uiRefWidth + (eTemplateType == LEFT_NEIGHBOR ? iTemplateWidth : 0);
    uint32_t uiRealH = uiRefHeight + (eTemplateType == ABOVE_NEIGHBOR ? iTemplateHeight : 0);
    uint64_t maxCost = (uint64_t)(iTemplateWidth * cu.lheight() + iTemplateHeight * cu.lwidth());

    uint64_t uiBestCost      = MAX_UINT64;
    int      iBestMode       = PLANAR_IDX;
    uint64_t uiSecondaryCost = MAX_UINT64;
    int      iSecondaryMode  = PLANAR_IDX;
    uint64_t uiNonAngCost    = MAX_UINT64;
    int      iNonAngMode     = PLANAR_IDX;

    uint64_t uiBestCostHor = MAX_UINT64;
    uint64_t uiBestCostVer = MAX_UINT64;
    int      iBestModeHor  = PLANAR_IDX;
    int      iBestModeVer  = PLANAR_IDX;

    auto calculateCost = [eTemplateType, &distParamSad]()
    {
      std::pair<uint64_t, uint64_t> cost;
      if (eTemplateType == LEFT_ABOVE_NEIGHBOR)
      {
        cost.first  = distParamSad[0].distFunc(distParamSad[0]);
        cost.second = distParamSad[1].distFunc(distParamSad[1]);
      }
      else if (eTemplateType == ABOVE_NEIGHBOR)
      {
        cost.first = distParamSad[0].distFunc(distParamSad[0]);
      }
      else if (eTemplateType == LEFT_NEIGHBOR)
      {
        cost.second = distParamSad[1].distFunc(distParamSad[1]);
      }
      else
      {
        CHECK(true, "invalid case");
      }
      return cost;
    };

    if (timdDerivationMethod == TimdDerivationMethod::Full || timdDerivationMethod == TimdDerivationMethod::FullWithSAD)
    {
      for (int iMode = 0; iMode <= 1; iMode++)
      {
        initPredTimdIntraParams(pu, area, iMode, false);
        predTimdIntraAng(COMP_Y, pu, iMode, piPred, uiPredStride, uiRealW, uiRealH, eTemplateType,
                         (eTemplateType == ABOVE_NEIGHBOR) ? 0 : iTemplateWidth,
                         (eTemplateType == LEFT_NEIGHBOR) ? 0 : iTemplateHeight);

        const auto [tmpCost0, tmpCost1] = calculateCost();
        const auto uiCost               = tmpCost0 + tmpCost1;

        if (uiCost < uiBestCost)
        {
          uiSecondaryCost    = uiBestCost;   // secondary <= previous best
          iSecondaryMode     = iBestMode;
          uiBestCost         = uiCost;
          iBestMode          = iMode;
          uiNonAngCost       = uiCost;
          iNonAngMode        = iMode;
          timdData.locDep[1] = timdData.locDep[0];   // secondary <= previous best
          timdData.locDep[0] = (eTemplateType != LEFT_ABOVE_NEIGHBOR) ? ((eTemplateType == LEFT_NEIGHBOR) ? 2 : 1)
            : (tmpCost0 >> log2A < (tmpCost1 >> (log2L + 1)))         ? 1
            : (tmpCost1 >> log2L < (tmpCost0 >> (log2A + 1)))         ? 2
                                                                      : 0;
          timdData.locDep[2] = timdData.locDep[0];   // non angular <= current best
        }
        else if (uiCost < uiSecondaryCost)
        {
          uiSecondaryCost    = uiCost;
          iSecondaryMode     = iMode;
          timdData.locDep[1] = (eTemplateType != LEFT_ABOVE_NEIGHBOR) ? ((eTemplateType == LEFT_NEIGHBOR) ? 2 : 1)
            : (tmpCost0 >> log2A < (tmpCost1 >> (log2L + 1)))         ? 1
            : (tmpCost1 >> log2L < (tmpCost0 >> (log2A + 1)))         ? 2
                                                                      : 0;
        }
      }
    }

    const auto mpmList = (timdDerivationMethod == TimdDerivationMethod::FullWithSAD)
      ? createMostProbableModeListBasedOnTimdModeCostList(pu, m_timdModeCostList)
      : createMostProbableModeListWithDCVerHor(pu);

    bool updateFull = true;
    for (const auto mpmListMode: mpmList)
    {
      uint64_t uiCostVer = -1;
      uint64_t uiCostHor = -1;
      uint64_t tmpCost0  = 0;
      uint64_t tmpCost1  = 0;
      if (mpmListMode <= DC_IDX)
      {
        continue;
      }

      const auto iMode = MAP67TO131(mpmListMode);

      initPredTimdIntraParams(pu, area, iMode, false);
      predTimdIntraAng(COMP_Y, pu, iMode, piPred, uiPredStride, uiRealW, uiRealH, eTemplateType,
                       (eTemplateType == ABOVE_NEIGHBOR) ? 0 : iTemplateWidth,
                       (eTemplateType == LEFT_NEIGHBOR) ? 0 : iTemplateHeight);
      if (eTemplateType == LEFT_ABOVE_NEIGHBOR)
      {
        if ((timdDerivationMethod == TimdDerivationMethod::Full ||
             timdDerivationMethod == TimdDerivationMethod::FullWithSAD) &&
            updateFull)
        {
          tmpCost0 = distParamSad[0].distFunc(distParamSad[0]);
          tmpCost1 = distParamSad[1].distFunc(distParamSad[1]);
        }
        else
        {
          if (iMode > EXT_DIA_IDX)
          {
            tmpCost0 = distParamSad[0].distFunc(distParamSad[0]);
          }
          else
          {
            tmpCost1 = distParamSad[1].distFunc(distParamSad[1]);
          }
        }
      }
      else if (eTemplateType == LEFT_NEIGHBOR)
      {
        tmpCost1 = distParamSad[1].distFunc(distParamSad[1]);
      }
      else if (eTemplateType == ABOVE_NEIGHBOR)
      {
        tmpCost0 = distParamSad[0].distFunc(distParamSad[0]);
      }
      else
      {
        CHECK(true, "invalid case");
      }

      if ((timdDerivationMethod == TimdDerivationMethod::Full ||
           timdDerivationMethod == TimdDerivationMethod::FullWithSAD) &&
          updateFull)
      {
        uint64_t uiCost = tmpCost0 + tmpCost1;

        if (fillTimdModeCostList && !isModeAddedToTimdModeCostList.at(mpmListMode))
        {
          m_timdModeCostList.push_back(std::pair(uiCost, mpmListMode));
          isModeAddedToTimdModeCostList[mpmListMode] = true;
        }

        if (uiCost < uiBestCost)
        {
          uiSecondaryCost    = uiBestCost;
          iSecondaryMode     = iBestMode;
          uiBestCost         = uiCost;
          iBestMode          = iMode;
          timdData.locDep[1] = timdData.locDep[0];
          timdData.locDep[0] = (eTemplateType != LEFT_ABOVE_NEIGHBOR) ? ((eTemplateType == LEFT_NEIGHBOR) ? 2 : 1)
            : (tmpCost0 >> log2A < (tmpCost1 >> (log2L + 1)))         ? 1
            : (tmpCost1 >> log2L < (tmpCost0 >> (log2A + 1)))         ? 2
                                                                      : 0;
        }
        else if (uiCost < uiSecondaryCost)
        {
          uiSecondaryCost    = uiCost;
          iSecondaryMode     = iMode;
          timdData.locDep[1] = (eTemplateType != LEFT_ABOVE_NEIGHBOR) ? ((eTemplateType == LEFT_NEIGHBOR) ? 2 : 1)
            : (tmpCost0 >> log2A < (tmpCost1 >> (log2L + 1)))         ? 1
            : (tmpCost1 >> log2L < (tmpCost0 >> (log2A + 1)))         ? 2
                                                                      : 0;
        }
        if (uiSecondaryCost <= maxCost)
        {
          updateFull = false;
          break;
        }
      }
      if (timdDerivationMethod == TimdDerivationMethod::HorizontalVertical && iMode > DC_IDX)
      {
        if (eTemplateType == LEFT_ABOVE_NEIGHBOR)
        {
          if (iMode > EXT_DIA_IDX)
          {
            uiCostVer = tmpCost0;
          }
          else
          {
            uiCostHor = tmpCost1;
          }
        }
        else if (eTemplateType == LEFT_NEIGHBOR)
        {
          uiCostHor = tmpCost1;
        }
        else if (eTemplateType == ABOVE_NEIGHBOR)
        {
          uiCostVer = tmpCost0;
        }
        if (uiCostHor < uiBestCostHor)
        {
          uiBestCostHor = uiCostHor;
          iBestModeHor  = iMode;
        }
        if (uiCostVer < uiBestCostVer)
        {
          uiBestCostVer = uiCostVer;
          iBestModeVer  = iMode;
        }
      }
    }

    if (timdDerivationMethod == TimdDerivationMethod::Full || timdDerivationMethod == TimdDerivationMethod::FullWithSAD)
    {
      int midMode = iBestMode;
      if (midMode > DC_IDX && uiBestCost > maxCost)
      {
        for (const auto i: { -1, 1 })
        {
          int iMode = midMode + i;
          if (iMode <= DC_IDX || iMode > EXT_VDIA_IDX)
          {
            continue;
          }
          initPredTimdIntraParams(pu, area, iMode, false);
          predTimdIntraAng(COMP_Y, pu, iMode, piPred, uiPredStride, uiRealW, uiRealH, eTemplateType,
                           (eTemplateType == ABOVE_NEIGHBOR) ? 0 : iTemplateWidth,
                           (eTemplateType == LEFT_NEIGHBOR) ? 0 : iTemplateHeight);

          const auto [tmpCost0, tmpCost1] = calculateCost();
          const auto uiCost               = tmpCost0 + tmpCost1;

          if (uiCost < uiBestCost)
          {
            uiBestCost         = uiCost;
            iBestMode          = iMode;
            timdData.locDep[0] = (eTemplateType != LEFT_ABOVE_NEIGHBOR) ? ((eTemplateType == LEFT_NEIGHBOR) ? 2 : 1)
              : (tmpCost0 >> log2A < (tmpCost1 >> (log2L + 1)))         ? 1
              : (tmpCost1 >> log2L < (tmpCost0 >> (log2A + 1)))         ? 2
                                                                        : 0;
          }
          if (uiBestCost <= maxCost)
          {
            break;
          }
        }
      }

      midMode = iSecondaryMode;
      if (midMode > DC_IDX && uiSecondaryCost > maxCost)
      {
        for (const auto i: { -1, 1 })
        {
          int iMode = midMode + i;
          if (iMode <= DC_IDX || iMode > EXT_VDIA_IDX)
          {
            continue;
          }
          initPredTimdIntraParams(pu, area, iMode, false);
          predTimdIntraAng(COMP_Y, pu, iMode, piPred, uiPredStride, uiRealW, uiRealH, eTemplateType,
                           (eTemplateType == ABOVE_NEIGHBOR) ? 0 : iTemplateWidth,
                           (eTemplateType == LEFT_NEIGHBOR) ? 0 : iTemplateHeight);

          const auto [tmpCost0, tmpCost1] = calculateCost();
          const auto uiCost               = tmpCost0 + tmpCost1;

          if (uiCost < uiSecondaryCost)
          {
            uiSecondaryCost    = uiCost;
            iSecondaryMode     = iMode;
            timdData.locDep[1] = ((eTemplateType != LEFT_ABOVE_NEIGHBOR) ? ((eTemplateType == LEFT_NEIGHBOR) ? 2 : 1)
                                    : (tmpCost0 >> log2A < (tmpCost1 >> (log2L + 1))) ? 1
                                    : (tmpCost1 >> log2L < (tmpCost0 >> (log2A + 1))) ? 2
                                                                                      : 0);
          }
          if (uiSecondaryCost <= maxCost)
          {
            break;
          }
        }
      }

      if (uiSecondaryCost < uiBestCost)
      {
        std::swap(timdData.locDep[1], timdData.locDep[0]);
        std::swap(uiSecondaryCost, uiBestCost);
        std::swap(iSecondaryMode, iBestMode);
      }

      // if( uiSecondaryCost < 2 * uiBestCost ), 2 * uiBestCost can overflow uint64_t
      timdData.isBlend = (uiSecondaryCost - uiBestCost < uiBestCost);

      if (!timdData.isBlend && (iBestMode > DC_IDX))
      {
        uiSecondaryCost  = uiBestCost;
        iSecondaryMode   = iBestMode;
        timdData.isBlend = (uiSecondaryCost - uiBestCost < uiBestCost);
      }

      timdData.blendMode[0] = iBestMode;
      if (timdData.isBlend)
      {
        timdData.blendMode[1] = iSecondaryMode;

        const int blendSumWeight = 6;
        int       sumWeight      = 1 << blendSumWeight;
        bool      useNonAngFusionCandidate = false;

        if (((iBestMode != iNonAngMode) && (iSecondaryMode != iNonAngMode)) &&
            ((uiNonAngCost < uiBestCost) || (uiNonAngCost - uiBestCost < (uiBestCost >> 1))))
        {
          useNonAngFusionCandidate = true;
          int iRatio[2];

          // compute 2*sum while checking for overflows
          uint64_t s1 = (MAX_UINT64 - uiSecondaryCost < uiBestCost) ? MAX_UINT64 : (uiBestCost + uiSecondaryCost);
          uint64_t s2 = (MAX_UINT64 - uiNonAngCost < s1) ? MAX_UINT64 : (uiBestCost + uiSecondaryCost + uiNonAngCost);
          uint64_t s3 = ((2 * s2) < s2) ? MAX_UINT64 : (2 * s2);

          // reciprocal of 2*sum
          int x = floorLog2Uint64(s3);
          CHECK(x < 0, "floor log2 value should be no negative");
          int normS3 = int(s3 << 4 >> x) & 15;
          int v      = g_gradDivTable[normS3];
          x += (normS3 != 0);
          int shift = x + 3;
          int add   = (1 << (shift - 1));

          // weight = (sum - cost) / (2*sum)
          iRatio[0] = int(((s2 - uiBestCost) * v * sumWeight + add) >> shift);
          if (iRatio[0] > sumWeight)
          {
            iRatio[0] = sumWeight;
          }
          iRatio[1] = int(((s2 - uiSecondaryCost) * v * sumWeight + add) >> shift);
          if (iRatio[1] > sumWeight)
          {
            iRatio[1] = sumWeight;
          }

          timdData.blendMode[2] = iNonAngMode;
          timdData.relWeight[0] = iRatio[0];
          timdData.relWeight[1] = iRatio[1];
          timdData.relWeight[2] = sumWeight - iRatio[0] - iRatio[1];
        }
        else
        {
          uint64_t s0 = uiSecondaryCost;
          // uiBestCost + uiSecondaryCost can overlow uint64_t
          uint64_t s1 = (MAX_UINT64 - uiSecondaryCost < uiBestCost) ? MAX_UINT64 : (uiBestCost + uiSecondaryCost);
          int      x  = floorLog2Uint64(s1);
          CHECK(x < 0, "floor log2 value should be no negative");
          int normS1 = int(s1 << 4 >> x) & 15;
          int v      = g_gradDivTable[normS1];
          x += (normS1 != 0);
          int shift  = x + 3;
          int add    = (1 << (shift - 1));
          int iRatio = int((s0 * v * sumWeight + add) >> shift);

          if (iRatio > sumWeight)
          {
            iRatio = sumWeight;
          }

          CHECK(iRatio > sumWeight, "Wrong TIMD ratio");

          timdData.relWeight[0] = iRatio;
          timdData.relWeight[1] = sumWeight - iRatio;
          timdData.relWeight[2] = 0;
        }

        // Refine the cost-derived weights by directly testing the fused prediction on the reconstructed template.
        // The original weights are retained unless a searched weight tuple gives a strictly lower template cost.
        constexpr int fusionWeightStep = 8;
        static Pel    fusionTemplatePred[TIMD_FUSION_NUM]
                                           [(MAX_CU_SIZE + TIMD_SAD_MAX_TEMP_SIZE) *
                                            (MAX_CU_SIZE + TIMD_SAD_MAX_TEMP_SIZE)];

        const int fusionTemplateWidth  = iTemplateWidth;
        const int fusionTemplateHeight = iTemplateHeight;
        const int numFusionCandidates  = useNonAngFusionCandidate ? TIMD_FUSION_NUM : 2;
        for (int candIdx = 0; candIdx < numFusionCandidates; candIdx++)
        {
          const int mode = timdData.blendMode[candIdx];
          initPredTimdIntraParams(pu, area, mode, false);
          predTimdIntraAng(COMP_Y, pu, mode, fusionTemplatePred[candIdx], uiPredStride, uiRealW, uiRealH,
                           eTemplateType, (eTemplateType == ABOVE_NEIGHBOR) ? 0 : iTemplateWidth,
                           (eTemplateType == LEFT_NEIGHBOR) ? 0 : iTemplateHeight);
        }

        auto getFusedTemplateCost = [&](const std::array<int, TIMD_FUSION_NUM> &weights)
        {
          auto blendTemplateRegion = [&](const int startX, const int startY, const int width, const int height)
          {
            for (int y = startY; y < startY + height; y++)
            {
              Pel       *dst = piPred + y * uiPredStride + startX;
              const Pel *src[TIMD_FUSION_NUM];
              for (int candIdx = 0; candIdx < numFusionCandidates; candIdx++)
              {
                src[candIdx] = fusionTemplatePred[candIdx] + y * uiPredStride + startX;
              }

              for (int x = 0; x < width; x++)
              {
                int value = 0;
                for (int candIdx = 0; candIdx < numFusionCandidates; candIdx++)
                {
                  value += weights[candIdx] * src[candIdx][x];
                }
                dst[x] = Pel((value + (sumWeight >> 1)) >> blendSumWeight);
              }
            }
          };

          if (eTemplateType == LEFT_ABOVE_NEIGHBOR)
          {
            blendTemplateRegion(fusionTemplateWidth, 0, uiWidth, fusionTemplateHeight);
            blendTemplateRegion(0, fusionTemplateHeight, fusionTemplateWidth, uiHeight);
          }
          else if (eTemplateType == ABOVE_NEIGHBOR)
          {
            blendTemplateRegion(0, 0, uiWidth, fusionTemplateHeight);
          }
          else
          {
            CHECK(eTemplateType != LEFT_NEIGHBOR, "Invalid TIMD template type");
            blendTemplateRegion(0, 0, fusionTemplateWidth, uiHeight);
          }

          const auto [aboveCost, leftCost] = calculateCost();
          return aboveCost + leftCost;
        };

        std::array<int, TIMD_FUSION_NUM> bestWeights = { timdData.relWeight[0], timdData.relWeight[1],
                                                         timdData.relWeight[2] };
        uint64_t bestFusionCost = getFusedTemplateCost(bestWeights);

        // Keep the primary candidate active: predIntraTimd uses its buffer as the destination of the final blend.
        for (int weight0 = fusionWeightStep; weight0 <= sumWeight; weight0 += fusionWeightStep)
        {
          const int maxWeight1 = sumWeight - weight0;
          const int minWeight1 = useNonAngFusionCandidate ? 0 : maxWeight1;
          for (int weight1 = minWeight1; weight1 <= maxWeight1; weight1 += fusionWeightStep)
          {
            std::array<int, TIMD_FUSION_NUM> weights = { weight0, weight1,
                                                         sumWeight - weight0 - weight1 };
            const uint64_t fusionCost = getFusedTemplateCost(weights);
            if (fusionCost < bestFusionCost)
            {
              bestFusionCost = fusionCost;
              bestWeights    = weights;
            }
          }
        }

        for (int candIdx = 0; candIdx < TIMD_FUSION_NUM; candIdx++)
        {
          timdData.relWeight[candIdx] = int8_t(bestWeights[candIdx]);
        }
      }
    }
    if (timdDerivationMethod == TimdDerivationMethod::HorizontalVertical)
    {
      cu.timdHor = iBestModeHor;
      cu.timdVer = iBestModeVer;
    }

    return iBestMode;
  }
  else
  {
    if (timdDerivationMethod == TimdDerivationMethod::Full || timdDerivationMethod == TimdDerivationMethod::FullWithSAD)
    {
      timdData.blendMode[0] = PLANAR_IDX;
      timdData.isBlend      = false;
    }
    if (timdDerivationMethod == TimdDerivationMethod::HorizontalVertical)
    {
      cu.timdHor = PLANAR_IDX;
      cu.timdVer = PLANAR_IDX;
    }
    return PLANAR_IDX;
  }
}

void IntraPrediction::deriveSgpmModeOrdered(const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu,
                                            static_vector<SgpmInfo, SGPM_NUM> &candModeList,
                                            static_vector<double, SGPM_NUM> &candCostList, const DimdData &dimd)
{
  const SizeType uiWidth  = cu.lwidth();
  const SizeType uiHeight = cu.lheight();
  const int      iCurX    = cu.lx();
  const int      iCurY    = cu.ly();

  const int iTempWidth = SGPM_TEMPLATE_SIZE, iTempHeight = SGPM_TEMPLATE_SIZE;

  const auto templateTypePositionAndSize = CU::deriveTimdRefTypePositionAndSize(cu, iTempWidth, iTempHeight);
  const auto eTempType                   = templateTypePositionAndSize.eTemplateType;
  const auto [iRefX, iRefY]              = templateTypePositionAndSize.iRefPosition;
  const auto [uiRefWidth, uiRefHeight]   = templateTypePositionAndSize.uiRefSize;

  uint32_t uiRealW = uiRefWidth + (eTempType == LEFT_NEIGHBOR ? iTempWidth : 0);
  uint32_t uiRealH = uiRefHeight + (eTempType == ABOVE_NEIGHBOR ? iTempHeight : 0);

  const UnitArea localUnitArea(cu.chromaFormat, Area(0, 0, uiRealW, uiRealH));
  ptrdiff_t      uiPredStride = m_sgpmBuffer.getBuf(localUnitArea.Y()).stride;
  CHECK(eTempType != LEFT_ABOVE_NEIGHBOR, "left and above both should exist");

  const CodingStructure &cs = *cu.cs;
  m_ipaParam.multiRefIndex  = iTempWidth;
  Pel      *piOrg           = cs.picture->getRecoBuf(area).buf;
  ptrdiff_t iOrgStride      = cs.picture->getRecoBuf(area).stride;
  piOrg += (iRefY - iCurY) * iOrgStride + (iRefX - iCurX);

  initTimdIntraPatternLuma(cu, area, eTempType != ABOVE_NEIGHBOR ? iTempWidth : 0,
                           eTempType != LEFT_NEIGHBOR ? iTempHeight : 0, uiRefWidth, uiRefHeight);

  Distortion sadWholeTM[NUM_LUMA_MODE];
  Distortion sadPartsTM[NUM_LUMA_MODE][GEO_NUM_PARTITION_MODE];
  uint8_t    ipmList[GEO_NUM_PARTITION_MODE][2][SGPM_NUM_MPM];
  bool       sadPartsNeeded[NUM_LUMA_MODE][GEO_NUM_PARTITION_MODE] = {};
  bool       ipmNeeded[NUM_LUMA_MODE]                              = {};

  for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
  {
    if (!g_sgpm_splitDir[splitDir])
    {
      continue;
    }

    int16_t angle = g_geoParams[splitDir].angleIdx;
    for (int partIdx = 0; partIdx < 2; partIdx++)
    {
      PU::getSgpmIntraMPMs(cu, ipmList[splitDir][partIdx], splitDir, g_geoTmShape[partIdx][angle], dimd.blendMode[0]);
      for (int modeIdx = 0; modeIdx < SGPM_NUM_MPM; modeIdx++)
      {
        int ipmIdx                       = ipmList[splitDir][partIdx][modeIdx];
        ipmNeeded[ipmIdx]                = true;
        sadPartsNeeded[ipmIdx][splitDir] = true;
      }
    }
  }

  for (int ipmIdx = 0; ipmIdx < NUM_LUMA_MODE; ipmIdx++)
  {
    if (ipmNeeded[ipmIdx])
    {
      int iMode = MAP67TO131(ipmIdx);
      initPredTimdIntraParams(cu, area, iMode, true);
      Pel *tempPred = m_sgpmBuffer.getBuf(localUnitArea.Y()).buf;
      predTimdIntraAng(COMP_Y, cu, iMode, tempPred, uiPredStride, uiRealW, uiRealH, eTempType,
                       (eTempType == ABOVE_NEIGHBOR) ? 0 : iTempWidth, (eTempType == LEFT_NEIGHBOR) ? 0 : iTempHeight);

      PelBuf predBuf = m_sgpmBuffer.getBuf(localUnitArea.Y());
      PelBuf recBuf  = cs.picture->getRecoBuf(area);
      PelBuf adBuf   = m_sgpmBuffer.getBuf(localUnitArea.Y());

      sadWholeTM[ipmIdx] =
        m_pIf->m_sadTM(cu, uiWidth, uiHeight, iTempWidth, iTempHeight, COMP_Y, predBuf, recBuf, adBuf);

      for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
      {
        if (sadPartsNeeded[ipmIdx][splitDir])
        {
          sadPartsTM[ipmIdx][splitDir] =
            m_pIf->m_sgpmSadTM(cu, uiWidth, uiHeight, iTempWidth, iTempHeight, COMP_Y, splitDir, adBuf);
        }
      }
    }
  }
  // check every possible combination
  uint32_t cntComb = 0;
  for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
  {
    if (!g_sgpm_splitDir[splitDir])
    {
      continue;
    }

    for (int mode0Idx = 0; mode0Idx < SGPM_NUM_MPM; mode0Idx++)
    {
      for (int mode1Idx = 0; mode1Idx < SGPM_NUM_MPM; mode1Idx++)
      {
        int ipm0Idx = ipmList[splitDir][0][mode0Idx];
        int ipm1Idx = ipmList[splitDir][1][mode1Idx];
        if (ipm0Idx == ipm1Idx)
        {
          continue;
        }

        double cost = static_cast<double>(sadPartsTM[ipm0Idx][splitDir]) + static_cast<double>(sadWholeTM[ipm1Idx]) -
          static_cast<double>(sadPartsTM[ipm1Idx][splitDir]);

        cntComb++;

        if ((cntComb > SGPM_NUM && cost < candCostList[SGPM_NUM - 1]) || cntComb <= SGPM_NUM)
        {
          updateCandList(SgpmInfo(splitDir, ipm0Idx, ipm1Idx), cost, candModeList, candCostList, SGPM_NUM);
        }
      }
    }
  }
}

void IntraPrediction::xPredIntraBDPCM(const CPelBuf &pSrc, PelBuf &pDst, const BdpcmMode dirMode, const ClpRng &clpRng)
{
  const int wdt = pDst.width;
  const int hgt = pDst.height;

  const ptrdiff_t strideP = pDst.stride;
  const ptrdiff_t strideS = pSrc.stride;

  CHECK(dirMode != BdpcmMode::HOR && dirMode != BdpcmMode::VER, "Incorrect BDPCM mode parameter.");

  Pel *pred = &pDst.buf[0];
  if (dirMode == BdpcmMode::HOR)
  {
    Pel val;
    for (int y = 0; y < hgt; y++)
    {
      val = pSrc.buf[(y + 1) + strideS];
      for (int x = 0; x < wdt; x++)
      {
        pred[x] = val;
      }
      pred += strideP;
    }
  }
  else
  {
    for (int y = 0; y < hgt; y++)
    {
      for (int x = 0; x < wdt; x++)
      {
        pred[x] = pSrc.buf[x + 1];
      }
      pred += strideP;
    }
  }
}

void IntraPrediction::geneWeightedPred(PelBuf &pred, const CodingUnit &cu, const Pel *srcBuf, const CompID compID)
{
  const int width  = pred.width;
  const int height = pred.height;

  const ptrdiff_t srcStride = width;
  const ptrdiff_t dstStride = pred.stride;

  Pel *dstBuf = pred.buf;

  CHECK(!cu.ciipFlag, "error in CIIP flag!");
  if (cu.ciipMode == CIIP_Type::NORMAL)
  {
    const Position posBL = cu.Y().bottomLeft();
    const Position posTR = cu.Y().topRight();

    const CodingUnit *neigh0 = cu.cs->getCURestricted(posBL.offset(-1, 0), cu, ChannelType::LUMA);
    const CodingUnit *neigh1 = cu.cs->getCURestricted(posTR.offset(0, -1), cu, ChannelType::LUMA);

    const bool isNeigh0Intra = neigh0 != nullptr && (CU::isIntra(*neigh0));
    const bool isNeigh1Intra = neigh1 != nullptr && (CU::isIntra(*neigh1));

    const int wIntra = 1 + (isNeigh0Intra ? 1 : 0) + (isNeigh1Intra ? 1 : 0);

    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        dstBuf[y * dstStride + x] += (wIntra * (srcBuf[y * srcStride + x] - dstBuf[y * dstStride + x]) + 2) >> 2;
      }
    }
  }
  else if (cu.ciipMode == CIIP_Type::WITH_PDPC)
  {
    const int scale = ((floorLog2(width * height) - 2) >> 2);
    for (int y = 0; y < height; y++)
    {
      const int wT = 32 >> std::min(31, ((y << 1) >> scale));
      for (int x = 0; x < width; x++)
      {
        const int wL = 32 >> std::min(31, ((x << 1) >> scale));

        dstBuf[y * dstStride + x] = (Pel)ClipPel(
          (((int)(srcBuf[y * srcStride + x]) << 6) + (64 - wL - wT) * (int)(dstBuf[y * dstStride + x]) + 32) >> 6,
          cu.cs->slice->clpRng(compID));
      }
    }
  }
  else
  {
    THROW("cu.ciip cannot be 0!");
  }
}

void IntraPrediction::switchBuffer(const CodingUnit &cu, CompID compID, PelBuf srcBuff, Pel *dst)
{
  Pel *src = srcBuff.bufAt(0, 0);

  int compWidth  = compID == COMP_Y ? cu.lwidth() : cu.Cb().width;
  int compHeight = compID == COMP_Y ? cu.lheight() : cu.Cb().height;

  for (int i = 0; i < compHeight; i++)
  {
    std::copy_n(src, compWidth, dst);
    src += srcBuff.stride;
    dst += compWidth;
  }
}

void IntraPrediction::geneIntrainterPred(const CodingUnit &cu)
{
  if (!cu.ciipFlag)
  {
    return;
  }

  initIntraPatternChType(cu, cu.Y());
  predIntraAng(COMP_Y, cu.cs->getPredBuf(cu).Y(), cu, true, false);
  int maxCompID = 1;
  if (isChromaEnabled(cu.chromaFormat))
  {
    maxCompID = MAX_NUM_COMP;

    initIntraPatternChType(cu, cu.Cb());
    predIntraAng(COMP_Cb, cu.cs->getPredBuf(cu).Cb(), cu, true, false);

    initIntraPatternChType(cu, cu.Cr());
    predIntraAng(COMP_Cr, cu.cs->getPredBuf(cu).Cr(), cu, true, false);
  }
  for (int currCompID = 0; currCompID < maxCompID; currCompID++)
  {
    CompID currCompID2 = (CompID)currCompID;
    PelBuf tmpBuf      = currCompID == 0 ? cu.cs->getPredBuf(cu).Y()
                                         : (currCompID == 1 ? cu.cs->getPredBuf(cu).Cb() : cu.cs->getPredBuf(cu).Cr());
    switchBuffer(cu, currCompID2, tmpBuf, getPredictorPtr2(currCompID2, (uint32_t)cu.ciipMode));
  }
}

void IntraPrediction::initIntraPatternChType(const CodingUnit &cu, const CompArea &area, const bool forceRefFilterFlag,
                                             int partIdx)
{
  const CodingStructure &cs = *cu.cs;

  if (!forceRefFilterFlag)
  {
    initPredIntraParams(cu, area, *cs.sps);
  }

  Pel *refBufUnfiltered = m_refBuffer[area.compID][PRED_BUF_UNFILTERED];
  Pel *refBufFiltered   = m_refBuffer[area.compID][PRED_BUF_FILTERED];

  setReferenceArrayLengths(area);
  m_refAvailable = false;

  // ----- Step 1: unfiltered reference samples -----
  if (!partIdx)
  {
    xFillReferenceSamples(cs.picture->getRecoBuf(area), refBufUnfiltered, area, cu);
  }
  // ----- Step 2: filtered reference samples -----
  if (m_ipaParam.refFilterFlag || forceRefFilterFlag)
  {
    xFilterReferenceSamples(refBufUnfiltered, refBufFiltered, area, *cs.sps, cu.multiRefIdx);
  }
}

bool xFillReferenceSamplesL(const CPelBuf &refBuf, Pel *ref, int numTemplateLines, int w, int h, int pw, int ph)
{
  int numMRLLeft = numTemplateLines;
  int numMRLTop  = numTemplateLines;

  // buffer for top neighbors ( corner included )
  int buf1W = numMRLLeft + w;
  int buf1H = numMRLTop;
  int bs1   = buf1W * buf1H;

  // buffer for left neighbors
  int buf2W = numMRLLeft;
  int buf2H = h;

  CPelBuf refBuf1 = refBuf.subBuf(Position(0, 0), Size(buf1W, buf1H));
  CPelBuf refBuf2 = refBuf.subBuf(Position(0, numMRLTop), Size(buf2W, buf2H));

  PelBuf targetBuf1 = PelBuf(ref, buf1W, Size(buf1W, buf1H));
  PelBuf targetBuf2 = PelBuf(ref + bs1, buf2W, Size(buf2W, buf2H));

  targetBuf1.padCopyFrom(refBuf1, buf1W, buf1H, pw, 0);
  targetBuf2.padCopyFrom(refBuf2, buf2W, buf2H, 0, ph);

  return true;
}

void IntraPrediction::xFillReferenceSamples(const CPelBuf &recoBuf, Pel *refBufUnfiltered, const CompArea &area,
                                            const CodingUnit &cu)
{
  const ChannelType      chType = toChannelType(area.compID);
  const CodingStructure &cs     = *cu.cs;
  const SPS             &sps    = *cs.sps;
  const PreCalcValues   &pcv    = *cs.pcv;

  const int multiRefIdx = (area.compID == COMP_Y) ? cu.multiRefIdx : 0;

  const int tuWidth    = area.width;
  const int tuHeight   = area.height;
  const int predSize   = m_topRefLength;
  const int predHSize  = m_leftRefLength;
  const int predStride = predSize + 1 + multiRefIdx;

  m_refBufferStride[area.compID] = predStride;

  const bool noShift      = pcv.noChroma2x2 && area.width == 4;   // don't shift on the lowest level (chroma not-split)
  const int unitWidthLog2 = pcv.minCUWidthLog2 - (noShift ? 0 : getComponentScaleX(area.compID, sps.m_chromaFormatIdc));
  const int unitHeightLog2 =
    pcv.minCUHeightLog2 - (noShift ? 0 : getComponentScaleY(area.compID, sps.m_chromaFormatIdc));
  const int unitWidth  = 1 << unitWidthLog2;
  const int unitHeight = 1 << unitHeightLog2;

  const int totalAboveUnits    = (predSize + (unitWidth - 1)) >> unitWidthLog2;
  const int totalLeftUnits     = (predHSize + (unitHeight - 1)) >> unitHeightLog2;
  const int totalUnits         = totalAboveUnits + totalLeftUnits + 1;   //+1 for top-left
  const int numAboveUnits      = std::max<int>(tuWidth >> unitWidthLog2, 1);
  const int numLeftUnits       = std::max<int>(tuHeight >> unitHeightLog2, 1);
  const int numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int numLeftBelowUnits  = totalLeftUnits - numLeftUnits;

  CHECK(numAboveUnits <= 0 || numLeftUnits <= 0 || numAboveRightUnits <= 0 || numLeftBelowUnits <= 0,
        "Size not supported");

  // ----- Step 1: analyze neighborhood -----
  const Position posLT = area;
  const Position posRT = area.topRight();
  const Position posLB = area.bottomLeft();

  bool neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  int  numIntraNeighbor = 0;

  std::fill_n(neighborFlags, totalUnits, false);

  neighborFlags[totalLeftUnits] = isAboveLeftAvailable(cu, chType, posLT, multiRefIdx);
  numIntraNeighbor += neighborFlags[totalLeftUnits] ? 1 : 0;
  numIntraNeighbor +=
    isAboveAvailable(cu, chType, posLT, numAboveUnits, unitWidth, (neighborFlags + totalLeftUnits + 1), multiRefIdx);
  numIntraNeighbor += isAboveRightAvailable(cu, chType, posRT, numAboveRightUnits, unitWidth,
                                            (neighborFlags + totalLeftUnits + 1 + numAboveUnits), multiRefIdx);
  numIntraNeighbor +=
    isLeftAvailable(cu, chType, posLT, numLeftUnits, unitHeight, (neighborFlags + totalLeftUnits - 1), multiRefIdx);
  numIntraNeighbor += isBelowLeftAvailable(cu, chType, posLB, numLeftBelowUnits, unitHeight,
                                           (neighborFlags + totalLeftUnits - 1 - numLeftUnits), multiRefIdx);

  // ----- Step 2: fill reference samples (depending on neighborhood) -----

  const Pel      *srcBuf    = recoBuf.buf;
  const ptrdiff_t srcStride = recoBuf.stride;
  Pel            *ptrDst    = refBufUnfiltered;
  const Pel      *ptrSrc;
  const Pel       valueDC = 1 << (sps.m_bitDepths[chType] - 1);

  if (numIntraNeighbor == 0)
  {
    // Fill border with DC value
    for (int j = 0; j <= predSize + multiRefIdx; j++)
    {
      ptrDst[j] = valueDC;
    }
    for (int i = 0; i <= predHSize + multiRefIdx; i++)
    {
      ptrDst[i + predStride] = valueDC;
    }
  }
  else if (numIntraNeighbor == totalUnits)
  {
    // Fill top-left border and top and top right with rec. samples
    ptrSrc = srcBuf - (1 + multiRefIdx) * srcStride - (1 + multiRefIdx);
    for (int j = 0; j <= predSize + multiRefIdx; j++)
    {
      ptrDst[j] = ptrSrc[j];
    }
    for (int i = 0; i <= predHSize + multiRefIdx; i++)
    {
      ptrDst[i + predStride] = ptrSrc[i * srcStride];
    }
  }
  else   // reference samples are partially available
  {
    // Fill top-left sample(s) if available
    ptrSrc = srcBuf - (1 + multiRefIdx) * srcStride - (1 + multiRefIdx);
    ptrDst = refBufUnfiltered;
    if (neighborFlags[totalLeftUnits])
    {
      ptrDst[0]          = ptrSrc[0];
      ptrDst[predStride] = ptrSrc[0];
      for (int i = 1; i <= multiRefIdx; i++)
      {
        ptrDst[i]              = ptrSrc[i];
        ptrDst[i + predStride] = ptrSrc[i * srcStride];
      }
    }

    // Fill left & below-left samples if available (downwards)
    ptrSrc += (1 + multiRefIdx) * srcStride;
    ptrDst += (1 + multiRefIdx) + predStride;
    for (int unitIdx = totalLeftUnits - 1; unitIdx > 0; unitIdx--)
    {
      if (neighborFlags[unitIdx])
      {
        for (int i = 0; i < unitHeight; i++)
        {
          ptrDst[i] = ptrSrc[i * srcStride];
        }
      }
      ptrSrc += unitHeight * srcStride;
      ptrDst += unitHeight;
    }
    // Fill last below-left sample(s)
    if (neighborFlags[0])
    {
      int lastSample = ((predHSize & (unitHeight - 1)) == 0) ? unitHeight : predHSize & (unitHeight - 1);
      for (int i = 0; i < lastSample; i++)
      {
        ptrDst[i] = ptrSrc[i * srcStride];
      }
    }

    // Fill above & above-right samples if available (left-to-right)
    ptrSrc = srcBuf - srcStride * (1 + multiRefIdx);
    ptrDst = refBufUnfiltered + 1 + multiRefIdx;
    for (int unitIdx = totalLeftUnits + 1; unitIdx < totalUnits - 1; unitIdx++)
    {
      if (neighborFlags[unitIdx])
      {
        for (int j = 0; j < unitWidth; j++)
        {
          ptrDst[j] = ptrSrc[j];
        }
      }
      ptrSrc += unitWidth;
      ptrDst += unitWidth;
    }
    // Fill last above-right sample(s)
    if (neighborFlags[totalUnits - 1])
    {
      int lastSample = ((predSize & (unitWidth - 1)) == 0) ? unitWidth : predSize & (unitWidth - 1);
      for (int j = 0; j < lastSample; j++)
      {
        ptrDst[j] = ptrSrc[j];
      }
    }

    // pad from first available down to the last below-left
    ptrDst            = refBufUnfiltered;
    int lastAvailUnit = 0;
    if (!neighborFlags[0])
    {
      int firstAvailUnit = 1;
      while (firstAvailUnit < totalUnits && !neighborFlags[firstAvailUnit])
      {
        firstAvailUnit++;
      }

      // first available sample
      int firstAvailRow = -1;
      int firstAvailCol = 0;
      if (firstAvailUnit < totalLeftUnits)
      {
        firstAvailRow = (totalLeftUnits - firstAvailUnit) * unitHeight + multiRefIdx;
      }
      else if (firstAvailUnit == totalLeftUnits)
      {
        firstAvailRow = multiRefIdx;
      }
      else
      {
        firstAvailCol = (firstAvailUnit - totalLeftUnits - 1) * unitWidth + 1 + multiRefIdx;
      }
      const Pel firstAvailSample = ptrDst[firstAvailRow < 0 ? firstAvailCol : firstAvailRow + predStride];

      // last sample below-left (n.a.)
      int lastRow = predHSize + multiRefIdx;

      // fill left column
      for (int i = lastRow; i > firstAvailRow; i--)
      {
        ptrDst[i + predStride] = firstAvailSample;
      }
      // fill top row
      if (firstAvailCol > 0)
      {
        for (int j = 0; j < firstAvailCol; j++)
        {
          ptrDst[j] = firstAvailSample;
        }
      }
      lastAvailUnit = firstAvailUnit;
    }

    // pad all other reference samples.
    int currUnit = lastAvailUnit + 1;
    while (currUnit < totalUnits)
    {
      if (!neighborFlags[currUnit])   // samples not available
      {
        // last available sample
        int lastAvailRow = -1;
        int lastAvailCol = 0;
        if (lastAvailUnit < totalLeftUnits)
        {
          lastAvailRow = (totalLeftUnits - lastAvailUnit - 1) * unitHeight + multiRefIdx + 1;
        }
        else if (lastAvailUnit == totalLeftUnits)
        {
          lastAvailCol = multiRefIdx;
        }
        else
        {
          lastAvailCol = (lastAvailUnit - totalLeftUnits) * unitWidth + multiRefIdx;
        }
        const Pel lastAvailSample = ptrDst[lastAvailRow < 0 ? lastAvailCol : lastAvailRow + predStride];

        // fill current unit with last available sample
        if (currUnit < totalLeftUnits)
        {
          for (int i = lastAvailRow - 1; i >= lastAvailRow - unitHeight; i--)
          {
            ptrDst[i + predStride] = lastAvailSample;
          }
        }
        else if (currUnit == totalLeftUnits)
        {
          for (int i = 0; i < multiRefIdx + 1; i++)
          {
            ptrDst[i + predStride] = lastAvailSample;
          }
          for (int j = 0; j < multiRefIdx + 1; j++)
          {
            ptrDst[j] = lastAvailSample;
          }
        }
        else
        {
          int numSamplesInUnit = (currUnit == totalUnits - 1)
            ? (((predSize & (unitWidth - 1)) == 0) ? unitWidth : (predSize & (unitWidth - 1)))
            : unitWidth;
          for (int j = lastAvailCol + 1; j <= lastAvailCol + numSamplesInUnit; j++)
          {
            ptrDst[j] = lastAvailSample;
          }
        }
      }
      lastAvailUnit = currUnit;
      currUnit++;
    }
  }

  // fill reference samples for PDP
  const int  sizeKey       = (tuWidth << 8) + tuHeight;
  const int  sizeIdx       = g_size.find(sizeKey) != g_size.end() ? g_size[sizeKey] : -1;
  const bool sizeAvailable = (sizeIdx >= 0 && (g_validSizePdp[g_size[sizeKey]] || g_validSizeMip[g_size[sizeKey]]));
  const bool minAvailable  = numIntraNeighbor > numAboveUnits + numLeftUnits;
  const bool pdpSupported  = (cu.cs->sps->m_pdpEnabledFlag && isLuma(area.compID) && !cu.dimdFlag && !cu.timdFlag &&
                             !cu.sgpm && cu.plDir == PlanarDirType::NO_DIR && !cu.multiRefIdx);
  if (pdpSupported && sizeAvailable && minAvailable)
  {
    m_refAvailable = true;
    for (int i = numLeftUnits; i <= (totalLeftUnits + numAboveUnits); ++i)
    {
      if (!neighborFlags[i])
      {
        m_refAvailable = false;
        break;
      }
    }
    if (m_refAvailable)
    {
      int endUnit = totalLeftUnits + 1 + numAboveUnits;
      while (neighborFlags[endUnit] && endUnit != totalUnits)
      {
        ++endUnit;
      }

      int startUnit = 0;
      while (!neighborFlags[startUnit])
      {
        ++startUnit;
      }

      const int sizeID     = g_size[sizeKey];
      const int numMRLLeft = g_sizeData[sizeID][5];
      const int numMRLTop  = g_sizeData[sizeID][6];
      const int padW       = (totalUnits - endUnit) * 4;
      const int padH       = startUnit * 4;
      const int sizeW      = tuWidth << 1;
      const int sizeH      = tuHeight << 1;
      CPelBuf   refBuf     = CPelBuf(recoBuf.buf - numMRLTop * srcStride - numMRLLeft, recoBuf.stride,
                                     Size(sizeW + numMRLLeft, sizeH + numMRLTop));
      xFillReferenceSamplesL(refBuf, m_ref, numMRLTop, sizeW, sizeH, padW, padH);
      xFillReferenceSamplesL(refBuf, m_refShort, numMRLTop, tuWidth, tuHeight, 0, 0);
    }
  }
}

void IntraPrediction::xFilterReferenceSamples(const Pel *refBufUnfiltered, Pel *refBufFiltered, const CompArea &area,
                                              const SPS &sps, int multiRefIdx)
{
  if (area.compID != COMP_Y)
  {
    multiRefIdx = 0;
  }
  const int predSize  = m_topRefLength + multiRefIdx;
  const int predHSize = m_leftRefLength + multiRefIdx;

  const ptrdiff_t predStride = m_refBufferStride[area.compID];

  const Pel topLeft =
    (refBufUnfiltered[0] + refBufUnfiltered[1] + refBufUnfiltered[predStride] + refBufUnfiltered[predStride + 1] + 2) >>
    2;

  refBufFiltered[0] = topLeft;

  for (int i = 1; i < predSize; i++)
  {
    refBufFiltered[i] = (refBufUnfiltered[i - 1] + 2 * refBufUnfiltered[i] + refBufUnfiltered[i + 1] + 2) >> 2;
  }
  refBufFiltered[predSize] = refBufUnfiltered[predSize];

  refBufFiltered += predStride;
  refBufUnfiltered += predStride;

  refBufFiltered[0] = topLeft;

  for (int i = 1; i < predHSize; i++)
  {
    refBufFiltered[i] = (refBufUnfiltered[i - 1] + 2 * refBufUnfiltered[i] + refBufUnfiltered[i + 1] + 2) >> 2;
  }
  refBufFiltered[predHSize] = refBufUnfiltered[predHSize];
}

bool isAboveLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT, int mrlIdx)
{
  const CodingStructure &cs     = *cu.cs;
  const Position         refPos = posLT.offset(-mrlIdx - 1, -mrlIdx - 1);

  if (!cs.isDecomp(refPos, chType))
  {
    return false;
  }

  return (cs.getCURestricted(refPos, cu, chType) != nullptr);
}

int isAboveAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT,
                     const uint32_t numUnitsInPu, const uint32_t unitWidth, bool *validFlags, int mrlIdx)
{
  const CodingStructure &cs = *cu.cs;

  int       numIntra = 0;
  const int maxDx    = numUnitsInPu * unitWidth;

  for (int dx = 0; dx < maxDx; dx += unitWidth)
  {
    const Position refPos = posLT.offset(dx, -mrlIdx - 1);

    if (!cs.isDecomp(refPos, chType))
    {
      break;
    }

    const bool valid = (cs.getCURestricted(refPos, cu, chType) != nullptr);
    numIntra += valid ? 1 : 0;
    *validFlags = valid;

    validFlags++;
  }

  return numIntra;
}

int isLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const uint32_t numUnitsInPu,
                    const uint32_t unitHeight, bool *validFlags, int mrlIdx)
{
  const CodingStructure &cs = *cu.cs;

  int       numIntra = 0;
  const int maxDy    = numUnitsInPu * unitHeight;

  for (int dy = 0; dy < maxDy; dy += unitHeight)
  {
    const Position refPos = posLT.offset(-mrlIdx - 1, dy);

    if (!cs.isDecomp(refPos, chType))
    {
      break;
    }

    const bool valid = (cs.getCURestricted(refPos, cu, chType) != nullptr);
    numIntra += valid ? 1 : 0;
    *validFlags = valid;

    validFlags--;
  }

  return numIntra;
}

int isAboveRightAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posRT,
                          const uint32_t numUnitsInPu, const uint32_t unitWidth, bool *validFlags, int mrlIdx)
{
  const CodingStructure &cs = *cu.cs;

  int       numIntra = 0;
  const int maxDx    = numUnitsInPu * unitWidth;

  for (int dx = 0; dx < maxDx; dx += unitWidth)
  {
    const Position refPos = posRT.offset(unitWidth + dx, -mrlIdx - 1);

    if (!cs.isDecomp(refPos, chType))
    {
      break;
    }

    const bool valid = (cs.getCURestricted(refPos, cu, chType) != nullptr);
    numIntra += valid ? 1 : 0;
    *validFlags = valid;

    validFlags++;
  }

  return numIntra;
}

int isBelowLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLB,
                         const uint32_t numUnitsInPu, const uint32_t unitHeight, bool *validFlags, int mrlIdx)
{
  const CodingStructure &cs = *cu.cs;

  int       numIntra = 0;
  const int maxDy    = numUnitsInPu * unitHeight;

  for (int dy = 0; dy < maxDy; dy += unitHeight)
  {
    const Position refPos = posLB.offset(-mrlIdx - 1, unitHeight + dy);

    if (!cs.isDecomp(refPos, chType))
    {
      break;
    }

    const bool valid = (cs.getCURestricted(refPos, cu, chType) != nullptr);
    numIntra += valid ? 1 : 0;
    *validFlags = valid;

    validFlags--;
  }

  return numIntra;
}

// LumaRecPixels
void IntraPrediction::xGetLumaRecPixels(const CodingUnit &cu, CompArea chromaArea, bool createAllRefs)
{
  CHECK(cu.cccmFlag, "Call cccmCreateLumaRefs instead");

  const int curChromaMode = cu.intraDir[ChannelType::CHROMA];
  const int dstStride     = 2 * MAX_CU_SIZE + 1;

  Pel *pDst0 = m_piTemp + dstStride + 1;

  // assert 420 chroma subsampling
  CompArea lumaArea = CompArea(COMP_Y, cu.chromaFormat, chromaArea.lumaPos(),
                               recalcSize(cu.chromaFormat, ChannelType::CHROMA, ChannelType::LUMA,
                                          chromaArea.size()));   // needed for correct pos/size (4x4 Tus)

  CHECK(lumaArea.width == chromaArea.width && ChromaFormat::_444 != cu.chromaFormat, "");
  CHECK(lumaArea.height == chromaArea.height && ChromaFormat::_444 != cu.chromaFormat &&
          ChromaFormat::_422 != cu.chromaFormat,
        "");

  const SizeType chromaWidth  = chromaArea.width;
  const SizeType chromaHeight = chromaArea.height;

  const CPelBuf srcBuf    = cu.cs->picture->getRecoBuf(lumaArea);
  Pel const    *pRecSrc0  = srcBuf.bufAt(0, 0);
  ptrdiff_t     recStride = srcBuf.stride;

  int logSubWidthC  = getChannelTypeScaleX(ChannelType::CHROMA, cu.chromaFormat);
  int logSubHeightC = getChannelTypeScaleY(ChannelType::CHROMA, cu.chromaFormat);

  const ptrdiff_t recStride2 = recStride << logSubHeightC;

  const CodingUnit &lumaCU = isChroma(cu.chType) ? *cu.cs->picture->m_cs->getCU(lumaArea.pos(), ChannelType::LUMA) : cu;

  const CompArea &area = isChroma(cu.chType) ? chromaArea : lumaArea;

  const uint32_t tuWidth  = area.width;
  const uint32_t tuHeight = area.height;

  const int unitWidthLog2        = MIN_CU_LOG2 - getComponentScaleX(area.compID, area.chromaFormat);
  const int unitHeightLog2       = MIN_CU_LOG2 - getComponentScaleY(area.compID, area.chromaFormat);
  const int unitWidth            = 1 << unitWidthLog2;
  const int unitHeight           = 1 << unitHeightLog2;
  const int tuWidthInUnits       = tuWidth >> unitWidthLog2;
  const int tuHeightInUnits      = tuHeight >> unitHeightLog2;
  const int aboveUnits           = tuWidthInUnits;
  const int leftUnits            = tuHeightInUnits;
  const int chromaUnitWidthLog2  = MIN_CU_LOG2 - getComponentScaleX(COMP_Cb, area.chromaFormat);
  const int chromaUnitHeightLog2 = MIN_CU_LOG2 - getComponentScaleY(COMP_Cb, area.chromaFormat);
  const int chromaUnitWidth      = 1 << chromaUnitWidthLog2;
  const int chromaUnitHeight     = 1 << chromaUnitHeightLog2;
  const int topTemplateSampNum   = 2 * chromaWidth;   // for MDLM, the number of template samples is 2W or 2H.
  const int leftTemplateSampNum  = 2 * chromaHeight;
  assert(m_topRefLength >= topTemplateSampNum);
  assert(m_leftRefLength >= leftTemplateSampNum);

  const int totalAboveUnits = (topTemplateSampNum + (chromaUnitWidth - 1)) >> chromaUnitWidthLog2;
  const int totalLeftUnits  = (leftTemplateSampNum + (chromaUnitHeight - 1)) >> chromaUnitHeightLog2;
  const int totalUnits      = totalLeftUnits + totalAboveUnits + 1;
  const int aboveRightUnits = totalAboveUnits - aboveUnits;
  const int leftBelowUnits  = totalLeftUnits - leftUnits;

  int  avaiAboveRightUnits = 0;
  int  avaiLeftBelowUnits  = 0;
  bool neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  std::fill_n(neighborFlags, totalUnits, false);
  bool aboveIsAvailable, leftIsAvailable;

  int availableUnit = isLeftAvailable(isChroma(cu.chType) ? cu : lumaCU, toChannelType(area.compID), area.pos(),
                                      leftUnits, unitHeight, (neighborFlags + leftUnits + leftBelowUnits - 1), 0);

  leftIsAvailable = availableUnit == tuHeightInUnits;

  availableUnit = isAboveAvailable(isChroma(cu.chType) ? cu : lumaCU, toChannelType(area.compID), area.pos(),
                                   aboveUnits, unitWidth, (neighborFlags + leftUnits + leftBelowUnits + 1), 0);

  aboveIsAvailable = availableUnit == tuWidthInUnits;

  if (leftIsAvailable)   // if left is not available, then the below left is not available
  {
    avaiLeftBelowUnits = isBelowLeftAvailable(isChroma(cu.chType) ? cu : lumaCU, toChannelType(area.compID),
                                              area.bottomLeftComp(area.compID), leftBelowUnits, unitHeight,
                                              (neighborFlags + leftBelowUnits - 1), 0);
  }

  if (aboveIsAvailable)   // if above is not available, then  the above right is not available.
  {
    avaiAboveRightUnits = isAboveRightAvailable(isChroma(cu.chType) ? cu : lumaCU, toChannelType(area.compID),
                                                area.topRightComp(area.compID), aboveRightUnits, unitWidth,
                                                (neighborFlags + leftUnits + leftBelowUnits + aboveUnits + 1), 0);
  }

  Pel       *pDst = nullptr;
  Pel const *src  = nullptr;

  bool isFirstRowOfCtu = (lumaArea.y & ((cu.cs->sps)->m_ctuSize - 1)) == 0;

  if (aboveIsAvailable)
  {
    pDst = pDst0 - dstStride;

    int addedAboveRight = 0;

    if (createAllRefs || curChromaMode == MDLM_L_IDX || curChromaMode == MDLM_T_IDX || curChromaMode == MMLM_L_IDX ||
        curChromaMode == MMLM_T_IDX)
    {
      addedAboveRight = avaiAboveRightUnits * chromaUnitWidth;
    }
    for (int i = 0; i < chromaWidth + addedAboveRight; i++)
    {
      const bool leftPadding = i == 0 && !leftIsAvailable;
      if (cu.chromaFormat == ChromaFormat::_444)
      {
        src     = pRecSrc0 - recStride;
        pDst[i] = src[i];
      }
      else if (isFirstRowOfCtu)
      {
        src     = pRecSrc0 - recStride;
        pDst[i] = (src[2 * i] * 2 + src[2 * i - (leftPadding ? 0 : 1)] + src[2 * i + 1] + 2) >> 2;
      }
      else if (cu.chromaFormat == ChromaFormat::_422)
      {
        src = pRecSrc0 - recStride2;

        int s = 2;
        s += src[2 * i] * 2;
        s += src[2 * i - (leftPadding ? 0 : 1)];
        s += src[2 * i + 1];
        pDst[i] = s >> 2;
      }
      else if (cu.cs->sps->m_verCollocatedChromaFlag)
      {
        src = pRecSrc0 - recStride2;

        int s = 4;
        s += src[2 * i - recStride];
        s += src[2 * i] * 4;
        s += src[2 * i - (leftPadding ? 0 : 1)];
        s += src[2 * i + 1];
        s += src[2 * i + recStride];
        pDst[i] = s >> 3;
      }
      else
      {
        src   = pRecSrc0 - recStride2;
        int s = 4;
        s += src[2 * i] * 2;
        s += src[2 * i + 1];
        s += src[2 * i - (leftPadding ? 0 : 1)];
        s += src[2 * i + recStride] * 2;
        s += src[2 * i + 1 + recStride];
        s += src[2 * i + recStride - (leftPadding ? 0 : 1)];
        pDst[i] = s >> 3;
      }
    }
  }

  if (leftIsAvailable)
  {
    pDst = pDst0 - 1;
    src  = pRecSrc0 - 1 - logSubWidthC;

    int addedLeftBelow = 0;

    if (createAllRefs || curChromaMode == MDLM_L_IDX || curChromaMode == MDLM_T_IDX || curChromaMode == MMLM_L_IDX ||
        curChromaMode == MMLM_T_IDX)
    {
      addedLeftBelow = avaiLeftBelowUnits * chromaUnitHeight;
    }

    for (int j = 0; j < chromaHeight + addedLeftBelow; j++)
    {
      if (cu.chromaFormat == ChromaFormat::_444)
      {
        pDst[0] = src[0];
      }
      else if (cu.chromaFormat == ChromaFormat::_422)
      {
        int s = 2;
        s += src[0] * 2;
        s += src[-1];
        s += src[1];
        pDst[0] = s >> 2;
      }
      else if (cu.cs->sps->m_verCollocatedChromaFlag)
      {
        const bool abovePadding = j == 0 && !aboveIsAvailable;

        int s = 4;
        s += src[-(abovePadding ? 0 : recStride)];
        s += src[0] * 4;
        s += src[-1];
        s += src[1];
        s += src[recStride];
        pDst[0] = s >> 3;
      }
      else
      {
        int s = 4;
        s += src[0] * 2;
        s += src[1];
        s += src[-1];
        s += src[recStride] * 2;
        s += src[recStride + 1];
        s += src[recStride - 1];
        pDst[0] = s >> 3;
      }

      src += recStride2;
      pDst += dstStride;
    }
  }

  // inner part from reconstructed picture buffer
  for (int j = 0; j < chromaHeight; j++)
  {
    for (int i = 0; i < chromaWidth; i++)
    {
      if (cu.chromaFormat == ChromaFormat::_444)
      {
        pDst0[i] = pRecSrc0[i];
      }
      else if (cu.chromaFormat == ChromaFormat::_422)
      {
        const bool leftPadding = i == 0 && !leftIsAvailable;

        int s = 2;
        s += pRecSrc0[2 * i] * 2;
        s += pRecSrc0[2 * i - (leftPadding ? 0 : 1)];
        s += pRecSrc0[2 * i + 1];
        pDst0[i] = s >> 2;
      }
      else if (cu.cs->sps->m_verCollocatedChromaFlag)
      {
        const bool leftPadding  = i == 0 && !leftIsAvailable;
        const bool abovePadding = j == 0 && !aboveIsAvailable;

        int s = 4;
        s += pRecSrc0[2 * i - (abovePadding ? 0 : recStride)];
        s += pRecSrc0[2 * i] * 4;
        s += pRecSrc0[2 * i - (leftPadding ? 0 : 1)];
        s += pRecSrc0[2 * i + 1];
        s += pRecSrc0[2 * i + recStride];
        pDst0[i] = s >> 3;
      }
      else
      {
        CHECK(cu.chromaFormat != ChromaFormat::_420, "Chroma format must be 4:2:0 for vertical filtering");
        const bool leftPadding = i == 0 && !leftIsAvailable;

        int s = 4;
        s += pRecSrc0[2 * i] * 2;
        s += pRecSrc0[2 * i + 1];
        s += pRecSrc0[2 * i - (leftPadding ? 0 : 1)];
        s += pRecSrc0[2 * i + recStride] * 2;
        s += pRecSrc0[2 * i + 1 + recStride];
        s += pRecSrc0[2 * i + recStride - (leftPadding ? 0 : 1)];
        pDst0[i] = s >> 3;
      }
    }

    pDst0 += dstStride;
    pRecSrc0 += recStride2;
  }
}

void IntraPrediction::initIntraMip(const CodingUnit &cu, const CompArea &area)
{
  CHECK(area.width > MIP_MAX_WIDTH || area.height > MIP_MAX_HEIGHT, "Error: block size not supported for MIP");

  // prepare input (boundary) data for prediction
  CHECK(m_ipaParam.refFilterFlag, "ERROR: unfiltered refs expected for MIP");
  Pel      *ptrSrc     = getPredictorPtr(area.compID);
  const int srcStride  = m_refBufferStride[area.compID];
  const int srcHStride = 2;

  m_matrixIntraPred.prepareInputForPred(CPelBuf(ptrSrc, srcStride, srcHStride), area,
                                        cu.slice->m_sps->m_bitDepths[toChannelType(area.compID)], area.compID);
}

void IntraPrediction::predIntraMip(const CompID compId, PelBuf &piPred, const CodingUnit &cu)
{
  CHECK(piPred.width > MIP_MAX_WIDTH || piPred.height > MIP_MAX_HEIGHT, "Error: block size not supported for MIP");
  CHECK(!isPowerOf2(piPred.width) || !isPowerOf2(piPred.height), "Error: expecting blocks of size 2^M x 2^N");

  // generate mode-specific prediction
  uint32_t modeIdx       = MAX_NUM_MIP_MODE;
  bool     transposeFlag = false;
  if (compId == COMP_Y)
  {
    modeIdx       = cu.intraDir[ChannelType::LUMA];
    transposeFlag = cu.mipTransposedFlag;

    const uint32_t width   = cu.lwidth();
    const uint32_t height  = cu.lheight();
    const int      sizeKey = (width << 8) + height;
    const int      sizeIdx = g_size.find(sizeKey) != g_size.end() ? g_size[sizeKey] : -1;
    int16_t       *filter  = sizeIdx >= 0
             ? (cu.mipTransposedFlag ? g_pdpFiltersMip[modeIdx + 16][sizeIdx] : g_pdpFiltersMip[modeIdx][sizeIdx])
             : nullptr;
    if (cu.cs->sps->m_pdpEnabledFlag && m_refAvailable && filter && cu.plDir == PlanarDirType::NO_DIR && !cu.sgpm &&
        !cu.dimdFlag && !cu.timdFlag && !cu.multiRefIdx)
    {
      const ClpRng &clpRng(cu.cs->slice->clpRng(compId));
      if (m_xPredIntraOpt(piPred, cu, modeIdx, clpRng, m_ref, m_refShort))
      {
        return;
      }
    }
  }
  else
  {
    const CodingUnit &coLocatedLumaPU = PU::getCoLocatedLumaCU(cu);

    CHECK(cu.intraDir[ChannelType::CHROMA] != DM_CHROMA_IDX, "Error: MIP is only supported for chroma with DM_CHROMA.");
    CHECK(!coLocatedLumaPU.mipFlag, "Error: Co-located luma CU should use MIP.");

    modeIdx       = coLocatedLumaPU.intraDir[ChannelType::LUMA];
    transposeFlag = coLocatedLumaPU.mipTransposedFlag;
  }
  const int bitDepth = cu.slice->m_sps->m_bitDepths[toChannelType(compId)];

  CHECK(modeIdx >= MatrixIntraPrediction::getNumModesMip(piPred), "Error: Wrong MIP mode index");

  if (piPred.width != piPred.stride)
  {
    // need a continuous buffer for MIP
    PelBuf tmpMem(m_piTemp, piPred);
    m_matrixIntraPred.predBlock(tmpMem.bufAt(0, 0), modeIdx, transposeFlag, bitDepth, compId);
    piPred.copyFrom(tmpMem);
    return;
  }
  m_matrixIntraPred.predBlock(piPred.bufAt(0, 0), modeIdx, transposeFlag, bitDepth,
                              compId);   // could cause problems on HighDef
}

void IntraPrediction::reorderPLT(CodingStructure &cs, Partitioner &partitioner, CompID compBegin, uint32_t numComp)
{
  CodingUnit &cu = *cs.getCU(partitioner.chType);

  uint8_t reusePLTSizetmp = 0;
  uint8_t pltSizetmp      = 0;
  Pel     curPLTtmp[MAX_NUM_COMP][MAXPLTSIZE];
  bool    curPLTpred[MAXPLTPREDSIZE];

  for (int idx = 0; idx < MAXPLTPREDSIZE; idx++)
  {
    curPLTpred[idx]              = false;
    cu.reuseflag[compBegin][idx] = false;
  }
  for (int idx = 0; idx < MAXPLTSIZE; idx++)
  {
    curPLTpred[idx] = false;
  }

  for (int predidx = 0; predidx < cs.prevPLT.curPLTSize[compBegin]; predidx++)
  {
    bool match  = false;
    int  curidx = 0;

    for (curidx = 0; curidx < cu.curPLTSize[compBegin]; curidx++)
    {
      if (curPLTpred[curidx])
      {
        continue;
      }
      bool matchTmp = true;
      for (int comp = compBegin; comp < (compBegin + numComp); comp++)
      {
        matchTmp = matchTmp && (cu.curPLT[comp][curidx] == cs.prevPLT.curPLT[comp][predidx]);
      }
      if (matchTmp)
      {
        match = true;
        break;
      }
    }

    if (match)
    {
      cu.reuseflag[compBegin][predidx] = true;
      curPLTpred[curidx]               = true;
      if (CS::isDualITree(*cu.cs))
      {
        cu.reuseflag[COMP_Y][predidx] = true;
        for (int comp = COMP_Y; comp < MAX_NUM_COMP; comp++)
        {
          curPLTtmp[comp][reusePLTSizetmp] = cs.prevPLT.curPLT[comp][predidx];
        }
      }
      else
      {
        for (int comp = compBegin; comp < (compBegin + numComp); comp++)
        {
          curPLTtmp[comp][reusePLTSizetmp] = cs.prevPLT.curPLT[comp][predidx];
        }
      }
      reusePLTSizetmp++;
      pltSizetmp++;
    }
  }
  cu.reusePLTSize[compBegin] = reusePLTSizetmp;
  for (int curidx = 0; curidx < cu.curPLTSize[compBegin]; curidx++)
  {
    if (!curPLTpred[curidx])
    {
      for (int comp = compBegin; comp < (compBegin + numComp); comp++)
      {
        curPLTtmp[comp][pltSizetmp] = cu.curPLT[comp][curidx];
      }
      pltSizetmp++;
    }
  }
  assert(pltSizetmp == cu.curPLTSize[compBegin]);
  for (int curidx = 0; curidx < cu.curPLTSize[compBegin]; curidx++)
  {
    if (CS::isDualITree(*cu.cs))
    {
      for (int comp = COMP_Y; comp < MAX_NUM_COMP; comp++)
      {
        cu.curPLT[comp][curidx] = curPLTtmp[comp][curidx];
      }
    }
    else
    {
      for (int comp = compBegin; comp < (compBegin + numComp); comp++)
      {
        cu.curPLT[comp][curidx] = curPLTtmp[comp][curidx];
      }
    }
  }
}

int IntraPrediction::xCalcLMParametersGeneralized(int x, int y, int xx, int xy, int count, int bitDepth, int &a, int &b,
                                                  int &shift)
{

  uint32_t uiInternalBitDepth = bitDepth;
  if (count == 0)
  {
    a     = 0;
    b     = 1 << (uiInternalBitDepth - 1);
    shift = 0;
    return -1;
  }
  CHECK(count > 512, "");

  int iCountShift = floorLog2(count);

  int iTempShift = uiInternalBitDepth + iCountShift - 15;

  if (iTempShift > 0)
  {
    x  = (x + (1 << (iTempShift - 1))) >> iTempShift;
    y  = (y + (1 << (iTempShift - 1))) >> iTempShift;
    xx = (xx + (1 << (iTempShift - 1))) >> iTempShift;
    xy = (xy + (1 << (iTempShift - 1))) >> iTempShift;
    iCountShift -= iTempShift;
  }
  /////// xCalcLMParameters

  int avgX = x >> iCountShift;
  int avgY = y >> iCountShift;

  int rErrX = x & ((1 << iCountShift) - 1);
  int rErrY = y & ((1 << iCountShift) - 1);

  int iB = 7;
  shift  = 13 - iB;

  if (iCountShift == 0)
  {
    a     = 0;
    b     = 1 << (uiInternalBitDepth - 1);
    shift = 0;
  }
  else
  {
    int       a1             = xy - (avgX * avgY << iCountShift) - avgX * rErrY - avgY * rErrX;
    int       a2             = xx - (avgX * avgX << iCountShift) - 2 * avgX * rErrX;
    const int iShiftA1       = uiInternalBitDepth - 2;
    const int iShiftA2       = 5;
    const int iAccuracyShift = uiInternalBitDepth + 4;

    int iScaleShiftA2 = 0;
    int iScaleShiftA1 = 0;
    int a1s           = a1;
    int a2s           = a2;

    iScaleShiftA1 = a1 == 0 ? 0 : floorLog2(abs(a1)) - iShiftA1;
    iScaleShiftA2 = a2 == 0 ? 0 : floorLog2(abs(a2)) - iShiftA2;

    if (iScaleShiftA1 < 0)
    {
      iScaleShiftA1 = 0;
    }

    if (iScaleShiftA2 < 0)
    {
      iScaleShiftA2 = 0;
    }

    int iScaleShiftA = iScaleShiftA2 + iAccuracyShift - shift - iScaleShiftA1;

    a2s = a2 >> iScaleShiftA2;

    a1s = a1 >> iScaleShiftA1;

    if (a2s >= 32)
    {
      uint32_t a2t = m_auShiftLM[a2s - 32];
      a            = a1s * a2t;
    }
    else
    {
      a = 0;
    }

    if (iScaleShiftA < 0)
    {
      a = a << -iScaleShiftA;
    }
    else
    {
      a = a >> iScaleShiftA;
    }
    a = Clip3(-(1 << (15 - iB)), (1 << (15 - iB)) - 1, a);
    a = a << iB;

    int16_t n = 0;
    if (a != 0)
    {
      n = floorLog2(abs(a) + ((a < 0 ? -1 : 1) - 1) / 2) - 5;
    }

    shift = (shift + iB) - n;
    a     = a >> n;

    b = avgY - ((a * avgX) >> shift);
  }
  return 0;
}

int IntraPrediction::xLMSampleClassifiedTraining(int count, int mean, int meanC, int LumaSamples[], int ChrmSamples[],
                                                 int bitDepth, MMLM_parameter parameters[])
{

  // Initialize

  for (int i = 0; i < 2; i++)
  {
    parameters[i].a     = 0;
    parameters[i].b     = 1 << (bitDepth - 1);
    parameters[i].shift = 0;
  }

  if (count < 4)   //
  {
    return -1;
  }
  int groupCount[2] = { 0, 0 };

  CHECK(count > 512, "");

  int meanDiff = meanC - mean;
  mean         = std::max(1, mean);

  int lumaPower2[2][128];
  int chromaPower2[2][128];
  // int GroupCount[2] = { 0, 0 };
  for (int i = 0; i < count; i++)
  {
    if (LumaSamples[i] <= mean)
    {
      lumaPower2[0][groupCount[0]]   = LumaSamples[i];
      chromaPower2[0][groupCount[0]] = ChrmSamples[i];
      groupCount[0]++;
    }
    else
    {
      lumaPower2[1][groupCount[1]]   = LumaSamples[i];
      chromaPower2[1][groupCount[1]] = ChrmSamples[i];
      groupCount[1]++;
    }
  }

  // Take power of two
  for (int group = 0; group < 2; group++)
  {
    int existSampNum = groupCount[group];
    if (existSampNum < 2)
    {
      continue;
    }

    int upperPower2 = 1 << (floorLog2(existSampNum - 1) + 1);
    int lowerPower2 = 1 << (floorLog2(existSampNum));

    if (upperPower2 != lowerPower2)
    {
      int numPaddedSamples = std::min(existSampNum, upperPower2 - existSampNum);
      groupCount[group]    = upperPower2;
      int step             = (int)(existSampNum / numPaddedSamples);
      for (int i = 0; i < numPaddedSamples; i++)
      {
        lumaPower2[group][existSampNum + i]   = lumaPower2[group][i * step];
        chromaPower2[group][existSampNum + i] = chromaPower2[group][i * step];
      }
    }
  }

  int x[2], y[2], xy[2], xx[2];
  for (int group = 0; group < 2; group++)
  {
    x[group] = y[group] = xy[group] = xx[group] = 0;
  }

  for (int group = 0; group < 2; group++)
  {

    for (int i = 0; i < groupCount[group]; i++)
    {
      x[group] += lumaPower2[group][i];
      y[group] += chromaPower2[group][i];
      xx[group] += lumaPower2[group][i] * lumaPower2[group][i];
      xy[group] += lumaPower2[group][i] * chromaPower2[group][i];
    }
  }
  for (int group = 0; group < 2; group++)
  {
    int a, b, shift;
    if (groupCount[group] > 1)
    {
      xCalcLMParametersGeneralized(x[group], y[group], xx[group], xy[group], groupCount[group], bitDepth, a, b, shift);

      parameters[group].a     = a;
      parameters[group].b     = b;
      parameters[group].shift = shift;
    }
    else
    {
      parameters[group].a     = 0;
      parameters[group].b     = meanDiff;
      parameters[group].shift = 0;
    }
  }
  return 0;
}
void IntraPrediction::xPadMdlmTemplateSample(Pel *pSrc, Pel *pCur, int cWidth, int cHeight, int existSampNum,
                                             int targetSampNum)
{
  int  sampNumToBeAdd = targetSampNum - existSampNum;
  Pel *pTempSrc       = pSrc + existSampNum;
  Pel *pTempCur       = pCur + existSampNum;

  int step = (int)(existSampNum / sampNumToBeAdd);

  for (int i = 0; i < sampNumToBeAdd; i++)
  {
    pTempSrc[i] = pSrc[i * step];
    pTempCur[i] = pCur[i * step];
  }
}

void IntraPrediction::xGetLMParameters_LMS(const CodingUnit &pu, const CompID compID, const CompArea &chromaArea,
                                           CclmModel &cclmModel)
{
  CHECK(compID == COMP_Y, "");
  const SizeType cWidth  = chromaArea.width;
  const SizeType cHeight = chromaArea.height;

  const Position posLT = chromaArea;

  CodingStructure &cs = *(pu.cs);

  const SPS         &sps           = *cs.sps;
  const uint32_t     tuWidth       = chromaArea.width;
  const uint32_t     tuHeight      = chromaArea.height;
  const ChromaFormat nChromaFormat = sps.m_chromaFormatIdc;

  const int unitWidthLog2  = MIN_CU_LOG2 - getComponentScaleX(chromaArea.compID, nChromaFormat);
  const int unitHeightLog2 = MIN_CU_LOG2 - getComponentScaleY(chromaArea.compID, nChromaFormat);
  const int unitWidth      = 1 << unitWidthLog2;
  const int unitHeight     = 1 << unitHeightLog2;

  const int tuWidthInUnits  = tuWidth >> unitWidthLog2;
  const int tuHeightInUnits = tuHeight >> unitHeightLog2;

  const int aboveUnits          = tuWidthInUnits;
  const int leftUnits           = tuHeightInUnits;
  int       topTemplateSampNum  = 2 * cWidth;   // for MDLM, the template sample number is 2W or 2H;
  int       leftTemplateSampNum = 2 * cHeight;
  assert(m_topRefLength >= topTemplateSampNum);
  assert(m_leftRefLength >= leftTemplateSampNum);
  int totalAboveUnits     = (topTemplateSampNum + (unitWidth - 1)) >> unitWidthLog2;
  int totalLeftUnits      = (leftTemplateSampNum + (unitHeight - 1)) >> unitHeightLog2;
  int totalUnits          = totalLeftUnits + totalAboveUnits + 1;
  int aboveRightUnits     = totalAboveUnits - aboveUnits;
  int leftBelowUnits      = totalLeftUnits - leftUnits;
  int avaiAboveRightUnits = 0;
  int avaiLeftBelowUnits  = 0;
  int avaiAboveUnits      = 0;
  int avaiLeftUnits       = 0;

  int  curChromaMode = pu.intraDir[ChannelType::CHROMA];
  bool neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  memset(neighborFlags, 0, totalUnits);

  bool aboveAvailable, leftAvailable;

  int availableUnit = isAboveAvailable(pu, ChannelType::CHROMA, posLT, aboveUnits, unitWidth,
                                       (neighborFlags + leftUnits + leftBelowUnits + 1), 0);
  aboveAvailable    = availableUnit == tuWidthInUnits;

  availableUnit = isLeftAvailable(pu, ChannelType::CHROMA, posLT, leftUnits, unitHeight,
                                  (neighborFlags + leftUnits + leftBelowUnits - 1), 0);
  leftAvailable = availableUnit == tuHeightInUnits;
  if (leftAvailable)   // if left is not available, then the below left is not available
  {
    avaiLeftUnits      = tuHeightInUnits;
    avaiLeftBelowUnits = isBelowLeftAvailable(pu, ChannelType::CHROMA, chromaArea.bottomLeftComp(chromaArea.compID),
                                              leftBelowUnits, unitHeight, (neighborFlags + leftBelowUnits - 1), 0);
  }
  if (aboveAvailable)   // if above is not available, then  the above right is not available.
  {
    avaiAboveUnits = tuWidthInUnits;
    avaiAboveRightUnits =
      isAboveRightAvailable(pu, ChannelType::CHROMA, chromaArea.topRightComp(chromaArea.compID), aboveRightUnits,
                            unitWidth, (neighborFlags + leftUnits + leftBelowUnits + aboveUnits + 1), 0);
  }

  int srcStride = 2 * MAX_CU_SIZE + 1;

  PelBuf temp = PelBuf(m_piTemp + srcStride + 1, srcStride, Size(chromaArea));

  Pel *srcColor0  = temp.bufAt(0, 0);
  Pel *curChroma0 = getPredictorPtr(compID);

  int      x = 0, y = 0, xx = 0, xy = 0;
  int      iCountShift        = 0;
  unsigned uiInternalBitDepth = sps.m_bitDepths[ChannelType::CHROMA];

  Pel *src                       = srcColor0 - srcStride;
  int  actualTopTemplateSampNum  = 0;
  int  actualLeftTemplateSampNum = 0;

  // get the temp buffer to store the downsampled luma and chroma
  Pel pTempBufferSrc[2 * MAX_CU_SIZE];   // for MDLM, use tempalte size 2W or 2H,
  Pel pTempBufferCur[2 * MAX_CU_SIZE];
  int minDim = 1;
  int cntT   = 0;
  int cntL   = 0;

  if (curChromaMode == MDLM_T_IDX || curChromaMode == MMLM_T_IDX)
  {
    leftAvailable            = 0;
    actualTopTemplateSampNum = unitWidth * (avaiAboveUnits + avaiAboveRightUnits);
    minDim                   = actualTopTemplateSampNum;
  }
  else if (curChromaMode == MDLM_L_IDX || curChromaMode == MMLM_L_IDX)
  {
    aboveAvailable            = 0;
    actualLeftTemplateSampNum = unitHeight * (avaiLeftUnits + avaiLeftBelowUnits);
    minDim                    = actualLeftTemplateSampNum;
  }
  else if (curChromaMode == LM_CHROMA_IDX || curChromaMode == MMLM_CHROMA_IDX)
  {
    actualTopTemplateSampNum  = cWidth;
    actualLeftTemplateSampNum = cHeight;
    minDim                    = leftAvailable && aboveAvailable
                         ? 1 << floorLog2(std::min(actualLeftTemplateSampNum, actualTopTemplateSampNum))
                         : 1 << floorLog2(leftAvailable ? actualLeftTemplateSampNum : actualTopTemplateSampNum);
  }
  int numSteps = minDim;

  int sumLuma = 0;
  int numPels = aboveAvailable && leftAvailable ? 2 * numSteps : aboveAvailable || leftAvailable ? numSteps : 0;

  if (aboveAvailable)
  {
    cntT           = numSteps;
    src            = srcColor0 - srcStride;
    const Pel *cur = curChroma0 + 1;

    for (int j = 0; j < numSteps; j++)
    {
      int idx = (j * actualTopTemplateSampNum) / minDim;

      pTempBufferSrc[j] = src[idx];
      pTempBufferCur[j] = cur[idx];
      sumLuma += src[idx];
    }
  }

  if (leftAvailable)
  {
    cntL           = numSteps;
    src            = srcColor0 - 1;
    const Pel *cur = curChroma0 + m_refBufferStride[compID] + 1;

    for (int i = 0; i < numSteps; i++)
    {
      int idx = (i * actualLeftTemplateSampNum) / minDim;

      pTempBufferSrc[i + cntT] = src[srcStride * idx];
      pTempBufferCur[i + cntT] = cur[idx];
      sumLuma += src[srcStride * idx];
    }
  }

  cclmModel.model[0].midLuma = numPels ? lutDivideInteger(sumLuma, numPels) : 1 << (uiInternalBitDepth - 1);

  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
  {
    // pad the temple sample to targetSampNum.
    int orgNumSample = (curChromaMode == MDLM_T_IDX) ? (avaiAboveUnits * unitWidth) : (avaiLeftUnits * unitHeight);
    int existSampNum = (curChromaMode == MDLM_T_IDX) ? actualTopTemplateSampNum : actualLeftTemplateSampNum;

    if (!orgNumSample || !existSampNum)
    {
      cclmModel.setFirstModel(0, 1 << (uiInternalBitDepth - 1), 0);
      return;
    }

    int targetSampNum = 1 << (floorLog2(existSampNum - 1) + 1);

    if (targetSampNum != existSampNum)   // if existSampNum not a value of power of 2
    {
      xPadMdlmTemplateSample(pTempBufferSrc, pTempBufferCur, cWidth, cHeight, existSampNum, targetSampNum);
    }
    for (int j = 0; j < targetSampNum; j++)
    {
      x += pTempBufferSrc[j];
      y += pTempBufferCur[j];
      xx += pTempBufferSrc[j] * pTempBufferSrc[j];
      xy += pTempBufferSrc[j] * pTempBufferCur[j];
    }
    iCountShift = floorLog2(targetSampNum);
  }
  else if (curChromaMode == LM_CHROMA_IDX)
  {
    int minStep = 1;
    // int       numSteps = minDim;

    if (aboveAvailable)
    {
      iCountShift = floorLog2(minDim / minStep);
    }

    if (leftAvailable)
    {
      iCountShift += aboveAvailable ? 1 : floorLog2(minDim / minStep);
    }

    for (int i = 0; i < (cntT + cntL); i++)
    {
      x += pTempBufferSrc[i];
      y += pTempBufferCur[i];
      xx += pTempBufferSrc[i] * pTempBufferSrc[i];
      xy += pTempBufferSrc[i] * pTempBufferCur[i];
    }
  }
  if (PU::isMultiModeLM(pu.intraDir[ChannelType::CHROMA]))
  {
    // Classify and training
    MMLM_parameter parameters[2];
    int            lumaSamples[512];
    int            chrmSamples[512];
    int            meanC  = 0;
    int            mean   = 0;
    int            avgCnt = cntT + cntL;

    for (int i = 0; i < avgCnt; i++)
    {
      mean += pTempBufferSrc[i];
      meanC += pTempBufferCur[i];
      lumaSamples[i] = pTempBufferSrc[i];
      chrmSamples[i] = pTempBufferCur[i];
    }

    if (avgCnt)
    {
      int                  x                   = floorLog2(avgCnt);
      static const uint8_t divSigTable[1 << 4] = { // 4bit significands - 8 ( MSB is omitted )
                                                   0, 7, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 1, 1, 0
      };
      int normDiff = (avgCnt << 4 >> x) & 15;
      int v        = divSigTable[normDiff] | 8;
      x += normDiff != 0;

      mean  = (mean * v) >> (x + 3);
      meanC = (meanC * v) >> (x + 3);
    }

    xLMSampleClassifiedTraining(avgCnt, mean, meanC, lumaSamples, chrmSamples, uiInternalBitDepth, parameters);

    cclmModel.setFirstModel(parameters[0].a, parameters[0].b, parameters[0].shift);
    cclmModel.setSecondModel(parameters[1].a, parameters[1].b, parameters[1].shift, mean);

    // Middle luma values for the two models
    int sumLuma0 = 0;
    int sumLuma1 = 0;
    int numPels0 = 0;
    int numPels1 = 0;

    for (int i = 0; i < avgCnt; i++)
    {
      if (lumaSamples[i] <= mean)
      {
        sumLuma0 += lumaSamples[i];
        numPels0 += 1;
      }
      else
      {
        sumLuma1 += lumaSamples[i];
        numPels1 += 1;
      }
    }

    cclmModel.model[0].midLuma = numPels0 ? lutDivideInteger(sumLuma0, numPels0) : mean;
    cclmModel.model[1].midLuma = numPels1 ? lutDivideInteger(sumLuma1, numPels1) : mean;

    return;
  }

  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
  {
    if ((curChromaMode == MDLM_L_IDX) ? (!leftAvailable) : (!aboveAvailable))
    {
      cclmModel.setFirstModel(0, 1 << (uiInternalBitDepth - 1), 0);
      return;
    }
  }
  else
  {
    if (!leftAvailable && !aboveAvailable)
    {
      cclmModel.setFirstModel(0, 1 << (uiInternalBitDepth - 1), 0);
      return;
    }
  }

  int iTempShift = uiInternalBitDepth + iCountShift - 15;

  if (iTempShift > 0)
  {
    x  = (x + (1 << (iTempShift - 1))) >> iTempShift;
    y  = (y + (1 << (iTempShift - 1))) >> iTempShift;
    xx = (xx + (1 << (iTempShift - 1))) >> iTempShift;
    xy = (xy + (1 << (iTempShift - 1))) >> iTempShift;
    iCountShift -= iTempShift;
  }

  /////// xCalcLMParameters

  int avgX = x >> iCountShift;
  int avgY = y >> iCountShift;

  int rErrX = x & ((1 << iCountShift) - 1);
  int rErrY = y & ((1 << iCountShift) - 1);

  int iB    = 7;
  int a     = 0;
  int b     = 0;
  int shift = 13 - iB;

  if (iCountShift == 0)
  {
    cclmModel.setFirstModel(0, 1 << (uiInternalBitDepth - 1), 0);
  }
  else
  {
    int       a1             = xy - (avgX * avgY << iCountShift) - avgX * rErrY - avgY * rErrX;
    int       a2             = xx - (avgX * avgX << iCountShift) - 2 * avgX * rErrX;
    const int iShiftA1       = uiInternalBitDepth - 2;
    const int iShiftA2       = 5;
    const int iAccuracyShift = uiInternalBitDepth + 4;

    int iScaleShiftA2 = 0;
    int iScaleShiftA1 = 0;
    int a1s           = a1;
    int a2s           = a2;

    iScaleShiftA1 = a1 == 0 ? 0 : floorLog2(abs(a1)) - iShiftA1;
    iScaleShiftA2 = a2 == 0 ? 0 : floorLog2(abs(a2)) - iShiftA2;

    if (iScaleShiftA1 < 0)
    {
      iScaleShiftA1 = 0;
    }

    if (iScaleShiftA2 < 0)
    {
      iScaleShiftA2 = 0;
    }

    int iScaleShiftA = iScaleShiftA2 + iAccuracyShift - shift - iScaleShiftA1;

    a2s = a2 >> iScaleShiftA2;

    a1s = a1 >> iScaleShiftA1;

    if (a2s >= 32)
    {
      uint32_t a2t = m_auShiftLM[a2s - 32];
      a            = a1s * a2t;
    }
    else
    {
      a = 0;
    }

    if (iScaleShiftA < 0)
    {
      a = a << -iScaleShiftA;
    }
    else
    {
      a = a >> iScaleShiftA;
    }
    a = Clip3(-(1 << (15 - iB)), (1 << (15 - iB)) - 1, a);
    a = a << iB;

    int16_t n = 0;
    if (a != 0)
    {
      n = floorLog2(abs(a) + ((a < 0 ? -1 : 1) - 1) / 2) - 5;
    }

    shift = (shift + iB) - n;
    a     = a >> n;

    b = avgY - ((a * avgX) >> shift);

    cclmModel.setFirstModel(a, b, shift);
  }
}

bool isPosAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &refPos)
{
  const CodingStructure &cs = *cu.cs;

  if (!cs.isDecomp(refPos, chType))
  {
    return false;
  }

  return (cs.getCURestricted(refPos, cu, chType) != NULL);
}

// Note: function copied from ECM-14, memory allocated locally, would benefit from a rewrite
void IntraPrediction::filterPredInside(const CompID compID, PelBuf &piPred, const CodingUnit &cu)
{
  static Pel tempBuffer[(MAX_CU_SIZE + 2) * (MAX_CU_SIZE + 2)];

  ChannelType   channelType = toChannelType(compID);
  int           w           = cu.blocks[compID].width;
  int           h           = cu.blocks[compID].height;
  const CPelBuf recoBuf     = cu.cs->picture->getRecoBuf(compID);

  const int  recStride = (int)recoBuf.stride;
  const Pel *pRec      = &recoBuf.buf[cu.blocks[compID].y * recStride + cu.blocks[compID].x];
  const int  tmpStride = MAX_CU_SIZE + 2;
  Pel       *pTmp      = tempBuffer;
  const int  preStride = (int)piPred.stride;
  Pel       *pPre      = piPred.buf;

  const bool aboveAvailable = isPosAvailable(cu, channelType, cu.blocks[compID].pos().offset(0, -1)) ? true : false;
  const bool leftAvailable  = isPosAvailable(cu, channelType, cu.blocks[compID].pos().offset(-1, 0)) ? true : false;
  const bool leftAboveAvailable =
    isPosAvailable(cu, channelType, cu.blocks[compID].pos().offset(-1, -1)) ? true : false;
  const bool aboveRightAvailable =
    isPosAvailable(cu, channelType, cu.blocks[compID].pos().offset(w, -1)) ? true : false;
  const bool leftBelowAvailable = isPosAvailable(cu, channelType, cu.blocks[compID].pos().offset(-1, h)) ? true : false;

  for (int y = 0; y < h; y++)
  {
    memcpy(&pTmp[1 + (y + 1) * tmpStride], &pPre[y * preStride], sizeof(Pel) * w);
    pTmp[(y + 1) * tmpStride + 1 + w] = pTmp[(y + 1) * tmpStride + w];
  }
  memcpy(&pTmp[1 + (h + 1) * tmpStride], &pTmp[1 + h * tmpStride], sizeof(Pel) * (w + 1));

  if (aboveAvailable)
  {
    memcpy(&pTmp[1], &pRec[-1 * recStride], sizeof(Pel) * w);
  }
  else
  {
    memcpy(&pTmp[1], &pTmp[1 + tmpStride], sizeof(Pel) * w);
  }

  if (leftAvailable)
  {
    for (int y = 0; y < h; y++)
    {
      pTmp[(y + 1) * tmpStride] = pRec[y * recStride - 1];
    }
  }
  else
  {
    for (int y = 0; y < h; y++)
    {
      pTmp[(y + 1) * tmpStride] = pTmp[(y + 1) * tmpStride + 1];
    }
  }

  if (leftAboveAvailable)
  {
    pTmp[0] = pRec[-recStride - 1];
  }
  else
  {
    pTmp[0] = (pTmp[1] + pTmp[tmpStride] + 1) >> 1;
  }

  if (aboveRightAvailable)
  {
    pTmp[w + 1] = pRec[-recStride + w];
  }
  else
  {
    pTmp[w + 1] = pTmp[w];
  }

  if (leftBelowAvailable)
  {
    pTmp[(h + 1) * tmpStride] = pRec[-1 + h * recStride];
  }
  else
  {
    pTmp[(h + 1) * tmpStride] = pTmp[h * tmpStride];
  }

  pTmp = &tempBuffer[1 + tmpStride];

  for (int y = 0; y < h; y++)
  {
    for (int x = 0; x < w; x++)
    {
      int sum = pTmp[(y - 1) * tmpStride + x - 1] + pTmp[(y - 1) * tmpStride + x] + pTmp[(y - 1) * tmpStride + x + 1] +
        pTmp[y * tmpStride + x - 1] + 8 * pTmp[y * tmpStride + x] + pTmp[y * tmpStride + x + 1] +
        pTmp[(y + 1) * tmpStride + x - 1] + pTmp[(y + 1) * tmpStride + x] + pTmp[(y + 1) * tmpStride + x + 1];
      pPre[y * preStride + x] = (sum + 8) >> 4;
    }
  }
}

void IntraPrediction::predIntraCCCM(CodingUnit &cu, PelBuf &predCb, PelBuf &predCr, const bool createModels)
{
  if (createModels)
  {
    if (cu.cccmType == CONV_MODEL_CCCM_BVG)
    {
      cu.ccModels.init(PU::isMultiModeLM(cu.intraDir[ChannelType::CHROMA]));
      cu.ccModels.modelCb[0] = ConvModel(cu.cccmType, cu.cs->sps->m_bitDepths[ChannelType::LUMA]);
      cu.ccModels.modelCr[0] = ConvModel(cu.cccmType, cu.cs->sps->m_bitDepths[ChannelType::LUMA]);
      cu.ccModels.lumaOffset = m_cccmLumaOffset;
      cu.ccModels.valid      = true;

      int minVal = 0, maxVal = 0;
      if (!PU::isMultiModeLM(cu.intraDir[ChannelType::CHROMA]))   // ?if (!cu.ccModels.dualModel)
      {
        xBvgCccmCalcBlkRange(cu, minVal, maxVal);
        xBvgCccmCalcModels(cu, cu.ccModels.modelCb[0], cu.ccModels.modelCr[0], 0, 0, minVal, maxVal);
      }
      else
      {
        cu.ccModels.modelCb[1] = ConvModel(cu.cccmType, cu.cs->sps->m_bitDepths[ChannelType::LUMA]);
        cu.ccModels.modelCr[1] = ConvModel(cu.cccmType, cu.cs->sps->m_bitDepths[ChannelType::LUMA]);
        cu.ccModels.lumaThres  = xBvgCccmCalcBlkAver(cu);
        xBvgCccmCalcBlkRange(cu, minVal, maxVal);

        xBvgCccmCalcModels(cu, cu.ccModels.modelCb[0], cu.ccModels.modelCr[0], 1, cu.ccModels.lumaThres, minVal,
                           maxVal);
        xBvgCccmCalcModels(cu, cu.ccModels.modelCb[1], cu.ccModels.modelCr[1], 2, cu.ccModels.lumaThres, minVal,
                           maxVal);
      }
    }
    else
    {
      cu.ccModels.init(PU::isMultiModeLM(cu.intraDir[ChannelType::CHROMA]));
      cu.ccModels.modelCb[0] = ConvModel(cu.cccmType, cu.cs->sps->m_bitDepths[ChannelType::LUMA]);
      cu.ccModels.modelCr[0] = ConvModel(cu.cccmType, cu.cs->sps->m_bitDepths[ChannelType::LUMA]);
      cu.ccModels.lumaOffset = m_cccmLumaOffset;

      if (!cu.ccModels.dualModel)
      {
        xCccmCalcModels(cu, cu.ccModels.modelCb[0], cu.ccModels.modelCr[0], 0, 0);

        if (cu.cs->slice->m_sps->m_ccBoostTplRefSel && cu.cccmType == CONV_MODEL_CCCM)
        {
          CrossCompModels ccModelsAlternative = cu.ccModels;

          xCccmCalcModels(cu, ccModelsAlternative.modelCb[0], ccModelsAlternative.modelCr[0], 0, 0, 2);

          int templCostOriginal    = xGetOneCCPCandCost(cu, cu.ccModels, 0);
          int templCostAlternative = xGetOneCCPCandCost(cu, ccModelsAlternative, 0);

          if (templCostAlternative < templCostOriginal)
          {
            cu.ccModels = ccModelsAlternative;
          }
        }
      }
      else
      {
        // Multimode case
        cu.ccModels.modelCb[1] = ConvModel(cu.cccmType, cu.cs->sps->m_bitDepths[ChannelType::LUMA]);
        cu.ccModels.modelCr[1] = ConvModel(cu.cccmType, cu.cs->sps->m_bitDepths[ChannelType::LUMA]);
        cu.ccModels.lumaThres  = xCccmCalcRefAver(cu);
        cu.ccModels.filter     = cu.ccFilterFlag;

        xCccmCalcModels(cu, cu.ccModels.modelCb[0], cu.ccModels.modelCr[0], 1, cu.ccModels.lumaThres);
        xCccmCalcModels(cu, cu.ccModels.modelCb[1], cu.ccModels.modelCr[1], 2, cu.ccModels.lumaThres);
      }
    }
  }

  // Apply models
  xCccmApplyModels(cu, cu.ccModels, predCb, predCr);

  if (cu.ccModels.filter)
  {
    filterPredInside(COMP_Cb, predCb, cu);
    filterPredInside(COMP_Cr, predCr, cu);
  }
}

template<ConvModelType modelType, bool dualModel>
void IntraPrediction::xCccmApplyModelsTemplate(const CodingUnit &cu, const CrossCompModels &ccModels, PelBuf &predCb,
                                               PelBuf &predCr, const bool isTempl, const bool isTopTempl) const
{
  if (ccModels.dualModel && !dualModel)
  {
    xCccmApplyModelsTemplate<modelType, true>(cu, ccModels, predCb, predCr, isTempl, isTopTempl);
    return;
  }

  std::vector<Pel> samples(CCCM_NUM_PARAMS_MAX);

  // Merge compensates for the difference between original and current luma offset
  const int lumaOffset = m_cccmLumaOffset - ccModels.lumaOffset;

  const int locScaleX = std::max(CCCM_LOC_SHIFT_MIN, ccModels.modelCb[0].bd - floorLog2(cu.blocks[COMP_Cb].width));
  const int locScaleY = std::max(CCCM_LOC_SHIFT_MIN, ccModels.modelCb[0].bd - floorLog2(cu.blocks[COMP_Cb].height));

  const ClpRng &clpRngCb(cu.cs->slice->clpRng(COMP_Cb));
  const ClpRng &clpRngCr(cu.cs->slice->clpRng(COMP_Cr));

  const ConvModel &cccmModelCb0 = ccModels.modelCb[0];
  const ConvModel &cccmModelCb1 = ccModels.modelCb[1];
  const ConvModel &cccmModelCr0 = ccModels.modelCr[0];
  const ConvModel &cccmModelCr1 = ccModels.modelCr[1];

  const int modelThr = ccModels.lumaThres;

  const CccmAreaInfo &areaInfo    = modelType == CONV_MODEL_CCCM_NOSUBS ? m_cccmAreaInfoNoSubs : m_cccmAreaInfo;
  const int           filterIndex = modelType == CONV_MODEL_CCCM_NOSUBS ? 4 : 0;

  CPelBuf refLumaBlk =
    areaInfo.getPredPelBufFor(m_cccmLumaBuf[filterIndex]);   // Main reference is different for CONV_MODEL_CCCM_NOSUBS
  CPelBuf refLumaBlk1 =
    m_cccmAreaInfo.getPredPelBufFor(m_cccmLumaBuf[1]);   // These three only applicable for CONV_MODEL_CCCM_MULTIF_1...3
  CPelBuf refLumaBlk2 = m_cccmAreaInfo.getPredPelBufFor(m_cccmLumaBuf[2]);
  CPelBuf refLumaBlk3 = m_cccmAreaInfo.getPredPelBufFor(m_cccmLumaBuf[3]);

  // For CONV_MODEL_CCCM_NOSUBS
  const int compScaleX   = getComponentScaleX(COMP_Cb, cu.chromaFormat);
  const int compScaleY   = getComponentScaleY(COMP_Cb, cu.chromaFormat);
  const int stepX        = modelType == CONV_MODEL_CCCM_NOSUBS ? 1 << compScaleX : 1;
  const int stepY        = modelType == CONV_MODEL_CCCM_NOSUBS ? 1 << compScaleY : 1;
  const int chromaScaleX = modelType == CONV_MODEL_CCCM_NOSUBS ? compScaleX : 0;
  const int chromaScaleY = modelType == CONV_MODEL_CCCM_NOSUBS ? compScaleY : 0;
  ;

  int xStart  = 0;
  int yStart  = 0;
  int xEnd    = refLumaBlk.width;
  int yEnd    = refLumaBlk.height;
  int xOffset = 0;
  int yOffset = 0;

  // Adjust prediction area if calculating a template cost outside the CU
  if (isTempl)
  {
    if (isTopTempl)
    {
      yStart  = -(1 << chromaScaleY);
      yEnd    = 0;
      yOffset = 1;
    }
    else
    {
      xStart  = -(1 << chromaScaleX);
      xEnd    = 0;
      xOffset = 1;
    }
  }

  for (int y = yStart; y < yEnd; y += stepY)
  {
    for (int x = xStart; x < xEnd; x += stepX)
    {
      const Pel *src0 = refLumaBlk.bufAt(x, y);
      const Pel *src1 = refLumaBlk.bufAt(x, y + 1);
      const Pel *src2 = refLumaBlk.bufAt(x, y - 1);

      if (modelType == CONV_MODEL_CCCM)
      {
        // 7-tap cross
        samples[0] = src0[0] + lumaOffset;   // C
        samples[1] = src2[0] + lumaOffset;   // N
        samples[2] = src1[0] + lumaOffset;   // S
        samples[3] = src0[-1] + lumaOffset;   // W
        samples[4] = src0[1] + lumaOffset;   // E
        samples[5] = cccmModelCb0.nonlinear(src0[0] + lumaOffset);
        samples[6] = cccmModelCb0.bias();
      }
      else if (modelType == CONV_MODEL_CCCM_GRADLOC)
      {
        samples[0] = src0[0] + lumaOffset;   // C
        samples[1] = (2 * src2[0] + src2[-1] + src2[1]) - (2 * src1[0] + src1[-1] + src1[1]);   // Vertical gradient
        samples[2] = (2 * src0[-1] + src2[-1] + src1[-1]) - (2 * src0[1] + src2[1] + src1[1]);   // Horizontal gradient
        samples[3] = y << locScaleY;   // Y coordinate
        samples[4] = x << locScaleX;   // X coordinate
        samples[5] = cccmModelCb0.nonlinear(src0[0] + lumaOffset);
        samples[6] = cccmModelCb0.bias();
      }
      else if (modelType == CONV_MODEL_CCCM_MULTIF_1)
      {
        // 7-tap cross
        samples[0] = src0[0] + lumaOffset;   // C
        samples[1] = refLumaBlk1.at(x, y) + lumaOffset;   // W
        samples[2] = refLumaBlk2.at(x, y) + lumaOffset;   // E
        samples[3] = refLumaBlk3.at(x, y) + lumaOffset;
        samples[4] = cccmModelCb0.nonlinear(src0[0] + lumaOffset);
        samples[5] = cccmModelCb0.nonlinear(refLumaBlk1.at(x, y) + lumaOffset);
        samples[6] = cccmModelCb0.nonlinear(refLumaBlk2.at(x, y) + lumaOffset);
        samples[7] = y << locScaleY;   // Y coordinate
        samples[8] = x << locScaleX;   // X coordinate
        samples[9] = cccmModelCb0.bias();
      }
      else if (modelType == CONV_MODEL_CCCM_MULTIF_2)
      {
        samples[0]  = src0[0] + lumaOffset;   // C
        samples[1]  = src0[-1] + lumaOffset;   // W
        samples[2]  = src0[1] + lumaOffset;   // E
        samples[3]  = refLumaBlk1.at(x, y) + lumaOffset;   // C
        samples[4]  = refLumaBlk1.at(x - 1, y) + lumaOffset;   // W
        samples[5]  = refLumaBlk1.at(x + 1, y) + lumaOffset;   // E
        samples[6]  = cccmModelCb0.nonlinear(src0[0] + lumaOffset);
        samples[7]  = cccmModelCb0.nonlinear(src0[-1] + lumaOffset);
        samples[8]  = cccmModelCb0.nonlinear(src0[1] + lumaOffset);
        samples[9]  = x << locScaleX;   // X coordinate
        samples[10] = cccmModelCb0.bias();
      }
      else if (modelType == CONV_MODEL_CCCM_MULTIF_3)
      {
        samples[0]  = src0[0] + lumaOffset;   // C
        samples[1]  = src2[1] + lumaOffset;   // EN
        samples[2]  = src1[-1] + lumaOffset;   // WS
        samples[3]  = refLumaBlk3.at(x, y) + lumaOffset;   // C
        samples[4]  = refLumaBlk3.at(x + 1, y - 1) + lumaOffset;   // EN
        samples[5]  = refLumaBlk3.at(x - 1, y + 1) + lumaOffset;   // WS
        samples[6]  = cccmModelCb0.nonlinear(src0[0] + lumaOffset);
        samples[7]  = cccmModelCb0.nonlinear(src2[1] + lumaOffset);
        samples[8]  = cccmModelCb0.nonlinear(src1[-1] + lumaOffset);
        samples[9]  = y << locScaleY;   // Y coordinate
        samples[10] = cccmModelCb0.bias();
      }
      else if (modelType == CONV_MODEL_CCCM_BVG)
      {
        // 11-tap filter
        samples[0]  = refLumaBlk.at(x, y) + lumaOffset;   // C
        samples[1]  = refLumaBlk.at(x, y - 1) + lumaOffset;   // N
        samples[2]  = refLumaBlk.at(x, y + 1) + lumaOffset;   // S
        samples[3]  = refLumaBlk.at(x - 1, y) + lumaOffset;   // W
        samples[4]  = refLumaBlk.at(x + 1, y) + lumaOffset;   // E
        samples[5]  = cccmModelCb0.nonlinear(refLumaBlk.at(x, y) + lumaOffset);   // nonlinear(C)
        samples[6]  = cccmModelCb0.nonlinear(refLumaBlk.at(x, y - 1) + lumaOffset);   // nonlinear(N)
        samples[7]  = cccmModelCb0.nonlinear(refLumaBlk.at(x, y + 1) + lumaOffset);   // nonlinear(S)
        samples[8]  = cccmModelCb0.nonlinear(refLumaBlk.at(x - 1, y) + lumaOffset);   // nonlinear(W)
        samples[9]  = cccmModelCb0.nonlinear(refLumaBlk.at(x + 1, y) + lumaOffset);   // nonlinear(E)
        samples[10] = cccmModelCb0.bias();
      }
      else   // if(modelType == CONV_MODEL_CCCM_NOSUBS)
      {
        samples[0]  = src0[0] + lumaOffset;
        samples[1]  = src0[-1] + lumaOffset;
        samples[2]  = src0[1] + lumaOffset;
        samples[3]  = src1[0] + lumaOffset;
        samples[4]  = src1[-1] + lumaOffset;
        samples[5]  = src1[1] + lumaOffset;
        samples[6]  = cccmModelCb0.nonlinear(src0[0] + lumaOffset);
        samples[7]  = cccmModelCb0.nonlinear(src1[0] + lumaOffset);
        samples[8]  = cccmModelCb0.nonlinear(src0[1] + lumaOffset);
        samples[9]  = cccmModelCb0.nonlinear(src0[-1] + lumaOffset);
        samples[10] = cccmModelCb0.bias();
      }

      if (dualModel &&
          src0[0] + lumaOffset > modelThr)   // Model 2 activated when reference luma is above the threshold
      {
        predCb.at((x >> chromaScaleX) + xOffset, (y >> chromaScaleY) + yOffset) =
          ClipPel<Pel>(cccmModelCb1.convolve(samples.data()), clpRngCb);
        predCr.at((x >> chromaScaleX) + xOffset, (y >> chromaScaleY) + yOffset) =
          ClipPel<Pel>(cccmModelCr1.convolve(samples.data()), clpRngCr);
      }
      else
      {
        predCb.at((x >> chromaScaleX) + xOffset, (y >> chromaScaleY) + yOffset) =
          ClipPel<Pel>(cccmModelCb0.convolve(samples.data()), clpRngCb);
        predCr.at((x >> chromaScaleX) + xOffset, (y >> chromaScaleY) + yOffset) =
          ClipPel<Pel>(cccmModelCr0.convolve(samples.data()), clpRngCr);
      }
    }
  }
}

void IntraPrediction::xCccmApplyModels(const CodingUnit &cu, const CrossCompModels &ccModels, PelBuf &predCb,
                                       PelBuf &predCr, const bool isTempl, const bool isTopTempl) const
{
  const ConvModel    &cccmModelCb0 = ccModels.modelCb[0];
  const ConvModelType modelType    = cccmModelCb0.modelType;
  if (modelType == CONV_MODEL_CCCM)
  {
    IntraPrediction::xCccmApplyModelsTemplate<CONV_MODEL_CCCM>(cu, ccModels, predCb, predCr, isTempl, isTopTempl);
  }
  else if (modelType == CONV_MODEL_CCCM_GRADLOC)
  {
    IntraPrediction::xCccmApplyModelsTemplate<CONV_MODEL_CCCM_GRADLOC>(cu, ccModels, predCb, predCr, isTempl,
                                                                       isTopTempl);
  }
  else if (modelType == CONV_MODEL_CCCM_MULTIF_1)
  {
    IntraPrediction::xCccmApplyModelsTemplate<CONV_MODEL_CCCM_MULTIF_1>(cu, ccModels, predCb, predCr, isTempl,
                                                                        isTopTempl);
  }
  else if (modelType == CONV_MODEL_CCCM_MULTIF_2)
  {
    IntraPrediction::xCccmApplyModelsTemplate<CONV_MODEL_CCCM_MULTIF_2>(cu, ccModels, predCb, predCr, isTempl,
                                                                        isTopTempl);
  }
  else if (modelType == CONV_MODEL_CCCM_MULTIF_3)
  {
    IntraPrediction::xCccmApplyModelsTemplate<CONV_MODEL_CCCM_MULTIF_3>(cu, ccModels, predCb, predCr, isTempl,
                                                                        isTopTempl);
  }
  else if (modelType == CONV_MODEL_CCCM_BVG)
  {
    IntraPrediction::xCccmApplyModelsTemplate<CONV_MODEL_CCCM_BVG>(cu, ccModels, predCb, predCr, isTempl, isTopTempl);
  }
  else
  {
    IntraPrediction::xCccmApplyModelsTemplate<CONV_MODEL_CCCM_NOSUBS>(cu, ccModels, predCb, predCr, isTempl,
                                                                      isTopTempl);
  }
}

template<ConvModelType modelType, int modelId>
inline void IntraPrediction::xCccmCalcModelsTemplate(CodingUnit &cu, ConvModel &cccmModelCb, ConvModel &cccmModelCr,
                                                     int modelIdD, int modelThr, int refSizeLimit)
{
  if (modelIdD != modelId)
  {
    if (modelIdD == 1)
    {
      xCccmCalcModelsTemplate<modelType, 1>(cu, cccmModelCb, cccmModelCr, modelIdD, modelThr, refSizeLimit);
    }
    else
    {
      xCccmCalcModelsTemplate<modelType, 2>(cu, cccmModelCb, cccmModelCr, modelIdD, modelThr, refSizeLimit);
    }
    return;
  }

  int areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY;

  const CPelBuf recoCb = cu.cs->picture->getRecoBuf(COMP_Cb);
  const CPelBuf recoCr = cu.cs->picture->getRecoBuf(COMP_Cr);

  const CccmAreaInfo &areaInfo    = modelType == CONV_MODEL_CCCM_NOSUBS ? m_cccmAreaInfoNoSubs : m_cccmAreaInfo;
  const int           filterIndex = modelType == CONV_MODEL_CCCM_NOSUBS ? 4 : 0;

  areaInfo.getRefParams(areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY);

  const int locScaleX = std::max(CCCM_LOC_SHIFT_MIN, cccmModelCb.bd - floorLog2(cu.blocks[COMP_Cb].width));
  const int locScaleY = std::max(CCCM_LOC_SHIFT_MIN, cccmModelCb.bd - floorLog2(cu.blocks[COMP_Cb].height));

  CPelBuf refLuma =
    areaInfo.getRefPelBufFor(m_cccmLumaBuf[filterIndex]);   // Main reference is different for CONV_MODEL_CCCM_NOSUBS
  CPelBuf refLuma1 = m_cccmAreaInfo.getRefPelBufFor(m_cccmLumaBuf[1]);
  CPelBuf refLuma2 = m_cccmAreaInfo.getRefPelBufFor(m_cccmLumaBuf[2]);
  CPelBuf refLuma3 = m_cccmAreaInfo.getRefPelBufFor(m_cccmLumaBuf[3]);

  int chromaOffsetCb = 1 << (cu.slice->m_sps->m_bitDepths[toChannelType(COMP_Cb)] - 1);
  int chromaOffsetCr = 1 << (cu.slice->m_sps->m_bitDepths[toChannelType(COMP_Cb)] - 1);

  const int blkPosChromaX = m_cccmAreaInfo.blkArea.x;
  const int blkPosChromaY = m_cccmAreaInfo.blkArea.y;

  if (blkPosChromaX || blkPosChromaY)
  {
    int posX = blkPosChromaX ? blkPosChromaX - 1 : 0;
    int posY = blkPosChromaY ? blkPosChromaY - 1 : 0;

    chromaOffsetCb = recoCb.at(posX, posY);
    chromaOffsetCr = recoCr.at(posX, posY);
  }

  int sampleNum = areaWidth * areaHeight - cu.blocks[COMP_Cb].width * cu.blocks[COMP_Cb].height;
  int sampleInd = 0;

  // Collect reference data to input matrix A and target vector Y
  NeighAreaType refAreaType = PU::crossCompNeighType(cu.intraDir[ChannelType::CHROMA]);

  const int yStartRestr = refSizeLimit && (refSizeY > refSizeLimit) ? refSizeY - refSizeLimit : 0;
  const int xStartRestr = refSizeLimit && (refSizeX > refSizeLimit) ? refSizeX - refSizeLimit : 0;

  // For CONV_MODEL_CCCM_NOSUBS
  const int compScaleX   = getComponentScaleX(COMP_Cb, cu.chromaFormat);
  const int compScaleY   = getComponentScaleY(COMP_Cb, cu.chromaFormat);
  const int stepX        = modelType == CONV_MODEL_CCCM_NOSUBS ? 1 << compScaleX : 1;
  const int stepY        = modelType == CONV_MODEL_CCCM_NOSUBS ? 1 << compScaleY : 1;
  const int chromaScaleX = modelType == CONV_MODEL_CCCM_NOSUBS ? compScaleX : 0;
  const int chromaScaleY = modelType == CONV_MODEL_CCCM_NOSUBS ? compScaleY : 0;
  ;

  const int yStart = refAreaType == NEIGH_AREA_LEFT ? refSizeY : yStartRestr;
  const int yEnd   = refAreaType == NEIGH_AREA_TOP ? refSizeY : areaHeight;
  for (int y = yStart; y < yEnd; y += stepY)
  {
    const int xStart = refAreaType == NEIGH_AREA_TOP ? refSizeX : xStartRestr;
    const int xEnd   = refAreaType == NEIGH_AREA_LEFT || y >= refSizeY ? refSizeX : areaWidth;
    for (int x = xStart; x < xEnd; x += stepX)
    {
      const Pel *src0 = refLuma.bufAt(x, y);

      if (modelId == 1 && src0[0] > modelThr)   // Model 1: Include only samples below or equal to the threshold
      {
        continue;
      }
      if (modelId == 2 && src0[0] <= modelThr)   // Model 2: Include only samples above the threshold
      {
        continue;
      }

      const Pel *src1 = refLuma.bufAt(x, y + 1);
      const Pel *src2 = refLuma.bufAt(x, y - 1);

      if (modelType == CONV_MODEL_CCCM)
      {
        // 7-tap cross
        m_refMatrixA[0 * strideRefMatrixA + sampleInd] = src0[0];   // C
        m_refMatrixA[1 * strideRefMatrixA + sampleInd] = src2[0];   // N
        m_refMatrixA[2 * strideRefMatrixA + sampleInd] = src1[0];   // S
        m_refMatrixA[3 * strideRefMatrixA + sampleInd] = src0[-1];   // W
        m_refMatrixA[4 * strideRefMatrixA + sampleInd] = src0[1];   // E
        m_refMatrixA[5 * strideRefMatrixA + sampleInd] = cccmModelCb.nonlinear(src0[0]);
        m_refMatrixA[6 * strideRefMatrixA + sampleInd] = cccmModelCb.bias();
      }
      else if (modelType == CONV_MODEL_CCCM_GRADLOC)
      {
        m_refMatrixA[0 * strideRefMatrixA + sampleInd] = src0[0];   // C
        m_refMatrixA[1 * strideRefMatrixA + sampleInd] =
          (2 * src2[0] + src2[-1] + src2[1]) - (2 * src1[0] + src1[-1] + src1[1]);   // Vertical gradient
        m_refMatrixA[2 * strideRefMatrixA + sampleInd] =
          (2 * src0[-1] + src2[-1] + src1[-1]) - (2 * src0[1] + src2[1] + src1[1]);   // Horizontal gradient
        m_refMatrixA[3 * strideRefMatrixA + sampleInd] = (y - refSizeY) << locScaleY;   // Y coordinate
        m_refMatrixA[4 * strideRefMatrixA + sampleInd] = (x - refSizeX) << locScaleX;   // X coordinate
        m_refMatrixA[5 * strideRefMatrixA + sampleInd] = cccmModelCb.nonlinear(src0[0]);
        m_refMatrixA[6 * strideRefMatrixA + sampleInd] = cccmModelCb.bias();
      }
      else if (modelType == CONV_MODEL_CCCM_MULTIF_1)
      {
        // 7-tap cross
        m_refMatrixA[0 * strideRefMatrixA + sampleInd] = src0[0];   // C
        m_refMatrixA[1 * strideRefMatrixA + sampleInd] = refLuma1.at(x, y);   // W
        m_refMatrixA[2 * strideRefMatrixA + sampleInd] = refLuma2.at(x, y);   // E
        m_refMatrixA[3 * strideRefMatrixA + sampleInd] = refLuma3.at(x, y);
        m_refMatrixA[4 * strideRefMatrixA + sampleInd] = cccmModelCb.nonlinear(src0[0]);
        m_refMatrixA[5 * strideRefMatrixA + sampleInd] = cccmModelCb.nonlinear(refLuma1.at(x, y));
        m_refMatrixA[6 * strideRefMatrixA + sampleInd] = cccmModelCb.nonlinear(refLuma2.at(x, y));
        m_refMatrixA[7 * strideRefMatrixA + sampleInd] = (y - refSizeY) << locScaleY;   // Y coordinate
        m_refMatrixA[8 * strideRefMatrixA + sampleInd] = (x - refSizeX) << locScaleX;   // X coordinate
        m_refMatrixA[9 * strideRefMatrixA + sampleInd] = cccmModelCb.bias();
      }
      else if (modelType == CONV_MODEL_CCCM_MULTIF_2)
      {
        m_refMatrixA[0 * strideRefMatrixA + sampleInd]  = src0[0];   // C
        m_refMatrixA[1 * strideRefMatrixA + sampleInd]  = src0[-1];   // W
        m_refMatrixA[2 * strideRefMatrixA + sampleInd]  = src0[1];   // E
        m_refMatrixA[3 * strideRefMatrixA + sampleInd]  = refLuma1.at(x, y);   // C
        m_refMatrixA[4 * strideRefMatrixA + sampleInd]  = refLuma1.at(x - 1, y);   // W
        m_refMatrixA[5 * strideRefMatrixA + sampleInd]  = refLuma1.at(x + 1, y);   // E
        m_refMatrixA[6 * strideRefMatrixA + sampleInd]  = cccmModelCb.nonlinear(src0[0]);
        m_refMatrixA[7 * strideRefMatrixA + sampleInd]  = cccmModelCb.nonlinear(src0[-1]);
        m_refMatrixA[8 * strideRefMatrixA + sampleInd]  = cccmModelCb.nonlinear(src0[1]);
        m_refMatrixA[9 * strideRefMatrixA + sampleInd]  = (x - refSizeX) << locScaleX;   // X coordinate
        m_refMatrixA[10 * strideRefMatrixA + sampleInd] = cccmModelCb.bias();
      }
      else if (modelType == CONV_MODEL_CCCM_MULTIF_3)
      {
        m_refMatrixA[0 * strideRefMatrixA + sampleInd]  = src0[0];   // C
        m_refMatrixA[1 * strideRefMatrixA + sampleInd]  = src2[1];   // EN
        m_refMatrixA[2 * strideRefMatrixA + sampleInd]  = src1[-1];   // WS
        m_refMatrixA[3 * strideRefMatrixA + sampleInd]  = refLuma3.at(x, y);   // C
        m_refMatrixA[4 * strideRefMatrixA + sampleInd]  = refLuma3.at(x + 1, y - 1);   // EN
        m_refMatrixA[5 * strideRefMatrixA + sampleInd]  = refLuma3.at(x - 1, y + 1);   // WS
        m_refMatrixA[6 * strideRefMatrixA + sampleInd]  = cccmModelCb.nonlinear(src0[0]);
        m_refMatrixA[7 * strideRefMatrixA + sampleInd]  = cccmModelCb.nonlinear(src2[1]);
        m_refMatrixA[8 * strideRefMatrixA + sampleInd]  = cccmModelCb.nonlinear(src1[-1]);
        m_refMatrixA[9 * strideRefMatrixA + sampleInd]  = (y - refSizeY) << locScaleY;   // Y coordinate
        m_refMatrixA[10 * strideRefMatrixA + sampleInd] = cccmModelCb.bias();
      }
      else   // if(modelType == CONV_MODEL_CCCM_NOSUBS)
      {
        m_refMatrixA[0 * strideRefMatrixA + sampleInd]  = src0[0];
        m_refMatrixA[1 * strideRefMatrixA + sampleInd]  = src0[-1];
        m_refMatrixA[2 * strideRefMatrixA + sampleInd]  = src0[1];
        m_refMatrixA[3 * strideRefMatrixA + sampleInd]  = src1[0];
        m_refMatrixA[4 * strideRefMatrixA + sampleInd]  = src1[-1];
        m_refMatrixA[5 * strideRefMatrixA + sampleInd]  = src1[1];
        m_refMatrixA[6 * strideRefMatrixA + sampleInd]  = cccmModelCb.nonlinear(src0[0]);
        m_refMatrixA[7 * strideRefMatrixA + sampleInd]  = cccmModelCb.nonlinear(src1[0]);
        m_refMatrixA[8 * strideRefMatrixA + sampleInd]  = cccmModelCb.nonlinear(src0[1]);
        m_refMatrixA[9 * strideRefMatrixA + sampleInd]  = cccmModelCb.nonlinear(src0[-1]);
        m_refMatrixA[10 * strideRefMatrixA + sampleInd] = cccmModelCb.bias();
      }

      const int refPosX = (refPosPicX + x) >> chromaScaleX;
      const int refPosY = (refPosPicY + y) >> chromaScaleY;

      m_targetVectorCb[sampleInd]   = recoCb.at(refPosX, refPosY);
      m_targetVectorCr[sampleInd++] = recoCr.at(refPosX, refPosY);
    }
  }

  if (sampleInd == 0)   // Number of samples can go to zero in the multimode case
  {
    cccmModelCb.clearModel();
    cccmModelCr.clearModel();
    return;
  }
  else
  {
    sampleNum = sampleInd;
  }

  m_cccmSolver.solve(*this, sampleNum, m_refMatrixA, strideRefMatrixA, 0, m_targetVectorCb, chromaOffsetCb,
                     &cccmModelCb, m_targetVectorCr, chromaOffsetCr, &cccmModelCr);
}

void IntraPrediction::xCccmCalcModels(CodingUnit &cu, ConvModel &cccmModelCb, ConvModel &cccmModelCr, int modelId,
                                      int modelThr, int refSizeLimit)
{
  const auto modelType = cccmModelCb.modelType;

  if (modelType == CONV_MODEL_CCCM)
  {
    IntraPrediction::xCccmCalcModelsTemplate<CONV_MODEL_CCCM>(cu, cccmModelCb, cccmModelCr, modelId, modelThr,
                                                              refSizeLimit);
  }
  else if (modelType == CONV_MODEL_CCCM_GRADLOC)
  {
    IntraPrediction::xCccmCalcModelsTemplate<CONV_MODEL_CCCM_GRADLOC>(cu, cccmModelCb, cccmModelCr, modelId, modelThr,
                                                                      refSizeLimit);
  }
  else if (modelType == CONV_MODEL_CCCM_MULTIF_1)
  {
    IntraPrediction::xCccmCalcModelsTemplate<CONV_MODEL_CCCM_MULTIF_1>(cu, cccmModelCb, cccmModelCr, modelId, modelThr,
                                                                       refSizeLimit);
  }
  else if (modelType == CONV_MODEL_CCCM_MULTIF_2)
  {
    IntraPrediction::xCccmCalcModelsTemplate<CONV_MODEL_CCCM_MULTIF_2>(cu, cccmModelCb, cccmModelCr, modelId, modelThr,
                                                                       refSizeLimit);
  }
  else if (modelType == CONV_MODEL_CCCM_MULTIF_3)
  {
    IntraPrediction::xCccmCalcModelsTemplate<CONV_MODEL_CCCM_MULTIF_3>(cu, cccmModelCb, cccmModelCr, modelId, modelThr,
                                                                       refSizeLimit);
  }
  else if (modelType == CONV_MODEL_CCCM_BVG)
  {
    IntraPrediction::xCccmCalcModelsTemplate<CONV_MODEL_CCCM_BVG>(cu, cccmModelCb, cccmModelCr, modelId, modelThr,
                                                                  refSizeLimit);
  }
  else
  {
    IntraPrediction::xCccmCalcModelsTemplate<CONV_MODEL_CCCM_NOSUBS>(cu, cccmModelCb, cccmModelCr, modelId, modelThr,
                                                                     refSizeLimit);
  }
}

// Calculate a single downsampled luma reference value (copied from IntraPrediction::xGetLumaRecPixels)
Pel IntraPrediction::xCccmGetLumaVal(const CodingUnit &cu, const CPelBuf pi, const int x, const int y,
                                     const int filterIndex) const
{
  const Pel *piSrc      = pi.buf;
  const int  iRecStride = static_cast<int>(pi.stride);
  Pel        ypval      = 0;

  if (cu.chromaFormat == ChromaFormat::_420 && !cu.cs->sps->m_verCollocatedChromaFlag)
  {
    const int lumaPosPicX1 = 2 * x;
    const int lumaPosPicY1 = 2 * y;
    const int lumaPosPicX0 = lumaPosPicX1 - 1 < 0 ? 0 : lumaPosPicX1 - 1;
    const int lumaPosPicX2 = lumaPosPicX1 + 1;
    const int lumaPosPicY2 = lumaPosPicY1 + 1;
    const int shift0       = iRecStride * lumaPosPicY1;
    const int shift1       = iRecStride * lumaPosPicY2;

    if (filterIndex == 0)
    {
      int s = 4;

      s += piSrc[lumaPosPicX1 + shift0] * 2;
      s += piSrc[lumaPosPicX0 + shift0];
      s += piSrc[lumaPosPicX2 + shift0];
      s += piSrc[lumaPosPicX1 + shift1] * 2;
      s += piSrc[lumaPosPicX0 + shift1];
      s += piSrc[lumaPosPicX2 + shift1];
      ypval = s >> 3;
    }
    else if (filterIndex == 1)
    {
      int s = 0;

      s += piSrc[lumaPosPicX0 + shift0];
      s -= piSrc[lumaPosPicX2 + shift0];
      s += piSrc[lumaPosPicX0 + shift1];
      s -= piSrc[lumaPosPicX2 + shift1];

      ypval = s < 0 ? 0 : s;
    }
    else if (filterIndex == 2)
    {
      int s = 0;

      s += piSrc[lumaPosPicX0 + shift0];
      s += piSrc[lumaPosPicX1 + shift0] * 2;
      s += piSrc[lumaPosPicX2 + shift0];
      s -= piSrc[lumaPosPicX0 + shift1];
      s -= piSrc[lumaPosPicX1 + shift1] * 2;
      s -= piSrc[lumaPosPicX2 + shift1];

      ypval = s < 0 ? 0 : s;
    }
    else   // if (filterIndex == 3)
    {
      int s = 0;

      s -= piSrc[lumaPosPicX0 + shift0];
      s += piSrc[lumaPosPicX1 + shift0];
      s += piSrc[lumaPosPicX2 + shift0] * 2;
      s -= piSrc[lumaPosPicX0 + shift1] * 2;
      s -= piSrc[lumaPosPicX1 + shift1];
      s += piSrc[lumaPosPicX2 + shift1];

      ypval = s < 0 ? 0 : s;
    }
  }
  else if (cu.chromaFormat == ChromaFormat::_420)
  {
    int s        = 4;
    int offLeft  = x > 0 ? -1 : 0;
    int offAbove = y > 0 ? -1 : 0;
    s += piSrc[2 * x + iRecStride * 2 * y] * 4;
    s += piSrc[2 * x + offLeft + iRecStride * 2 * y];
    s += piSrc[2 * x + 1 + iRecStride * 2 * y];
    s += piSrc[2 * x + iRecStride * (2 * y + 1)];
    s += piSrc[2 * x + iRecStride * (2 * y + offAbove)];
    ypval = s >> 3;
  }
  else if (cu.chromaFormat == ChromaFormat::_444)
  {
    ypval = piSrc[x + iRecStride * y];
  }
  else   // ChromaFormat::_422
  {
    int s       = 2;
    int offLeft = x > 0 ? -1 : 0;
    s += piSrc[2 * x + iRecStride * y] * 2;
    s += piSrc[2 * x + offLeft + iRecStride * y];
    s += piSrc[2 * x + 1 + iRecStride * y];
    ypval = s >> 2;
  }

  return ypval - m_cccmLumaOffset;   // Note: this could have also been included in the rounding offset s to avoid the
                                     // extra sample based operation
}

// Using the same availability checking as in IntraPrediction::xGetLumaRecPixels
void IntraPrediction::xCccmCalcRefArea(const CodingUnit &cu)
{
  CompArea chromaArea = cu.Cb();
  CompArea lumaArea   = CompArea(COMP_Y, cu.chromaFormat, chromaArea.lumaPos(),
                                 recalcSize(cu.chromaFormat, ChannelType::CHROMA, ChannelType::LUMA,
                                            chromaArea.size()));   // needed for correct pos/size (4x4 Tus)

  const SizeType uiCWidth  = chromaArea.width;
  const SizeType uiCHeight = chromaArea.height;

  const CodingUnit &lumaCU = isChroma(cu.chType) ? *cu.cs->picture->m_cs->getCU(lumaArea.pos(), ChannelType::LUMA) : cu;
  const CompArea   &area   = isChroma(cu.chType) ? chromaArea : lumaArea;

  const uint32_t uiTuWidth  = area.width;
  const uint32_t uiTuHeight = area.height;

  const int unitWidthLog2  = MIN_CU_LOG2 - getComponentScaleX(area.compID, area.chromaFormat);
  const int unitHeightLog2 = MIN_CU_LOG2 - getComponentScaleY(area.compID, area.chromaFormat);
  const int iUnitWidth     = 1 << unitWidthLog2;
  const int iUnitHeight    = 1 << unitHeightLog2;

  const int iTUWidthInUnits      = uiTuWidth >> unitWidthLog2;
  const int iTUHeightInUnits     = uiTuHeight >> unitHeightLog2;
  const int iAboveUnits          = iTUWidthInUnits;
  const int iLeftUnits           = iTUHeightInUnits;
  const int chromaUnitWidthLog2  = MIN_CU_LOG2 - getComponentScaleX(COMP_Cb, area.chromaFormat);
  const int chromaUnitHeightLog2 = MIN_CU_LOG2 - getComponentScaleY(COMP_Cb, area.chromaFormat);
  const int chromaUnitWidth      = 1 << chromaUnitWidthLog2;
  const int chromaUnitHeight     = 1 << chromaUnitHeightLog2;
  const int topTemplateSampNum   = 2 * uiCWidth;
  const int leftTemplateSampNum  = 2 * uiCHeight;
  const int totalAboveUnits      = (topTemplateSampNum + (chromaUnitWidth - 1)) >> chromaUnitWidthLog2;
  const int totalLeftUnits       = (leftTemplateSampNum + (chromaUnitHeight - 1)) >> chromaUnitHeightLog2;
  const int totalUnits           = totalLeftUnits + totalAboveUnits + 1;
  const int aboveRightUnits      = totalAboveUnits - iAboveUnits;
  const int leftBelowUnits       = totalLeftUnits - iLeftUnits;

  int  avaiAboveRightUnits = 0;
  int  avaiLeftBelowUnits  = 0;
  bool bNeighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  memset(bNeighborFlags, 0, totalUnits);
  bool aboveIsAvailable, leftIsAvailable;

  int availlableUnit = isLeftAvailable(isChroma(cu.chType) ? cu : lumaCU, toChannelType(area.compID), area.pos(),
                                       iLeftUnits, iUnitHeight, (bNeighborFlags + iLeftUnits + leftBelowUnits - 1), 0);

  leftIsAvailable = availlableUnit == iTUHeightInUnits;

  availlableUnit = isAboveAvailable(isChroma(cu.chType) ? cu : lumaCU, toChannelType(area.compID), area.pos(),
                                    iAboveUnits, iUnitWidth, (bNeighborFlags + iLeftUnits + leftBelowUnits + 1), 0);

  aboveIsAvailable = availlableUnit == iTUWidthInUnits;

  if (leftIsAvailable)   // if left is not available, then the below left is not available
  {
    avaiLeftBelowUnits = isBelowLeftAvailable(isChroma(cu.chType) ? cu : lumaCU, toChannelType(area.compID),
                                              area.bottomLeftComp(area.compID), leftBelowUnits, iUnitHeight,
                                              (bNeighborFlags + leftBelowUnits - 1), 0);
  }

  if (aboveIsAvailable)   // if above is not available, then  the above right is not available.
  {
    avaiAboveRightUnits = isAboveRightAvailable(isChroma(cu.chType) ? cu : lumaCU, toChannelType(area.compID),
                                                area.topRightComp(area.compID), aboveRightUnits, iUnitWidth,
                                                (bNeighborFlags + iLeftUnits + leftBelowUnits + iAboveUnits + 1), 0);
  }

  int refSizeX, refSizeY;

  PU::getCccmRefLineNum(cu, refSizeX, refSizeY);   // Reference lines available left and above

  int extWidth  = refSizeY ? avaiAboveRightUnits * chromaUnitWidth : 0;
  int extHeight = refSizeX ? avaiLeftBelowUnits * chromaUnitHeight : 0;

  int refWidth  = chromaArea.width + refSizeX + extWidth;   // Reference buffer size excluding paddings
  int refHeight = chromaArea.height + refSizeY + extHeight;

  m_cccmAreaInfo = CccmAreaInfo(chromaArea, refSizeX, refSizeY, refWidth, refHeight);

  const int scaleX = getComponentScaleX(COMP_Cb, area.chromaFormat);
  const int scaleY = getComponentScaleY(COMP_Cb, area.chromaFormat);

  const Area lumaBlkArea =
    Area(chromaArea.x << scaleX, chromaArea.y << scaleY, chromaArea.width << scaleX, chromaArea.height << scaleY);

  m_cccmAreaInfoNoSubs =
    CccmAreaInfo(lumaBlkArea, refSizeX << scaleX, refSizeY << scaleY, refWidth << scaleX, refHeight << scaleY);
}

int IntraPrediction::xCccmCalcRefAver(const CodingUnit &cu) const
{
  int areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY;

  const NeighAreaType refAreaType = PU::crossCompNeighType(cu.intraDir[ChannelType::CHROMA]);
  const CccmAreaInfo &areaInfo    = cu.cccmType == CONV_MODEL_CCCM_NOSUBS ? m_cccmAreaInfoNoSubs : m_cccmAreaInfo;
  const int           filterIndex = cu.cccmType == CONV_MODEL_CCCM_NOSUBS ? 4 : 0;
  const PelBuf        refLuma     = areaInfo.getRefPelBufFor(m_cccmLumaBuf[filterIndex]);

  areaInfo.getRefParams(areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY);

  int numSamples = 0;
  int sumSamples = 0;

  // Top samples
  if (refAreaType == NEIGH_AREA_TOPLEFT || refAreaType == NEIGH_AREA_TOP)
  {
    for (int y = 0; y < refSizeY; y++)
    {
      for (int x = refAreaType == NEIGH_AREA_TOP ? refSizeX : 0; x < areaWidth; x++)
      {
        sumSamples += refLuma.at(x, y);
        numSamples++;
      }
    }
  }

  // Left samples
  if (refAreaType == NEIGH_AREA_TOPLEFT || refAreaType == NEIGH_AREA_LEFT)
  {
    for (int y = refSizeY; y < areaHeight; y++)
    {
      for (int x = 0; x < refSizeX; x++)
      {
        sumSamples += refLuma.at(x, y);
        numSamples++;
      }
    }
  }

  return numSamples == 0 ? cu.slice->m_sps->m_bitDepths[toChannelType(COMP_Y)]
                         : lutDivideInteger(sumSamples, numSamples);
}

void IntraPrediction::xCccmSetLumaRefValue(const CodingUnit &cu)
{
  int lumaPosX = m_cccmAreaInfo.blkArea.x << getComponentScaleX(COMP_Cb, cu.chromaFormat);
  int lumaPosY = m_cccmAreaInfo.blkArea.y << getComponentScaleY(COMP_Cb, cu.chromaFormat);

  if (lumaPosX || lumaPosY)
  {
    lumaPosX = lumaPosX ? lumaPosX - 1 : 0;
    lumaPosY = lumaPosY ? lumaPosY - 1 : 0;

    m_cccmLumaOffset = cu.cs->picture->getRecoBuf(COMP_Y).at(lumaPosX, lumaPosY);
  }
  else
  {
    m_cccmLumaOffset = 1 << (cu.slice->m_sps->m_bitDepths[toChannelType(COMP_Y)] - 1);
  }
}

void IntraPrediction::xBvgCccmCalcModels(const CodingUnit &cu, ConvModel &cccmModelCb, ConvModel &cccmModelCr,
                                         int modelId, int modelThr, int minVal, int maxVal)
{
  const CPelBuf recoCb         = cu.cs->picture->getRecoBuf(COMP_Cb);
  const CPelBuf recoCr         = cu.cs->picture->getRecoBuf(COMP_Cr);
  // const int filterIndex = 0;
  int           chromaOffsetCb = 1 << (cu.slice->m_sps->m_bitDepths[toChannelType(COMP_Cb)] - 1);
  int           chromaOffsetCr = 1 << (cu.slice->m_sps->m_bitDepths[toChannelType(COMP_Cr)] - 1);

  int refSizeX   = m_cccmAreaInfo.refSizeX;   // Reference lines available left and above
  int refSizeY   = m_cccmAreaInfo.refSizeY;
  int refPosPicX = m_cccmAreaInfo.refArea.x;   // Position of the reference area in picture coordinates
  int refPosPicY = m_cccmAreaInfo.refArea.y;

  if (refSizeX || refSizeY)
  {
    int refPosX = refSizeX > 0 ? refSizeX - 1 : 0;
    int refPosY = refSizeY > 0 ? refSizeY - 1 : 0;

    chromaOffsetCb = recoCb.at(refPosPicX + refPosX, refPosPicY + refPosY);
    chromaOffsetCr = recoCr.at(refPosPicX + refPosX, refPosPicY + refPosY);
  }

  int sampleNum = 0;
  int sampleInd = 0;

  int strX[NUM_BVG_CCCM_CANDS], endX[NUM_BVG_CCCM_CANDS], strY[NUM_BVG_CCCM_CANDS], endY[NUM_BVG_CCCM_CANDS];
  for (int candIdx = 0; candIdx < cu.numBvgCands; candIdx++)
  {
    Mv     chromaBv   = cu.bvList[candIdx];
    PelBuf refLuma    = xBvgCccmGetLumaCuBuf(candIdx);
    int    refPosPicX = cu.blocks[COMP_Cb].x + chromaBv.hor;
    int    refPosPicY = cu.blocks[COMP_Cb].y + chromaBv.ver;
    strX[candIdx]     = refPosPicX;
    strY[candIdx]     = refPosPicY;
    endX[candIdx]     = refPosPicX + refLuma.width;
    endY[candIdx]     = refPosPicY + refLuma.height;
  }

  for (int candIdx = 0; candIdx < cu.numBvgCands; candIdx++)
  {
    PelBuf refLuma = xBvgCccmGetLumaCuBuf(candIdx);
    PelBuf refCb   = xBvgCccmGetChromaCuBuf(COMP_Cb, candIdx);
    PelBuf refCr   = xBvgCccmGetChromaCuBuf(COMP_Cr, candIdx);
    //-- collect data
    for (int y = 0; y < refLuma.height; y++)
    {
      for (int x = 0; x < refLuma.width; x++)
      {
        if (modelId == 1 &&
            refLuma.at(x, y) > modelThr)   // Model 1: Include only samples below or equal to the threshold
        {
          continue;
        }
        if (modelId == 2 && refLuma.at(x, y) <= modelThr)   // Model 2: Include only samples above the threshold
        {
          continue;
        }
        if (refLuma.at(x, y) < minVal || refLuma.at(x, y) > maxVal)
        {
          continue;
        }
        bool exist = false;
        if (candIdx > 0)
        {
          for (int i = 0; i < candIdx; i++)
          {
            int xx = strX[candIdx] + x;
            int yy = strY[candIdx] + y;
            if (xx >= strX[i] && xx < endX[i] && yy >= strY[i] && yy < endY[i])
            {
              exist = true;
              break;
            }
          }
        }
        if (exist)
        {
          continue;
        }
        // 11-tap filter
        m_refMatrixA[0 * strideRefMatrixA + sampleInd]  = refLuma.at(x, y);   // C
        m_refMatrixA[1 * strideRefMatrixA + sampleInd]  = refLuma.at(x, y - 1);   // N
        m_refMatrixA[2 * strideRefMatrixA + sampleInd]  = refLuma.at(x, y + 1);   // S
        m_refMatrixA[3 * strideRefMatrixA + sampleInd]  = refLuma.at(x - 1, y);   // W
        m_refMatrixA[4 * strideRefMatrixA + sampleInd]  = refLuma.at(x + 1, y);   // E
        m_refMatrixA[5 * strideRefMatrixA + sampleInd]  = cccmModelCb.nonlinear(refLuma.at(x, y));   // nonlinear(C)
        m_refMatrixA[6 * strideRefMatrixA + sampleInd]  = cccmModelCb.nonlinear(refLuma.at(x, y - 1));   // nonlinear(N)
        m_refMatrixA[7 * strideRefMatrixA + sampleInd]  = cccmModelCb.nonlinear(refLuma.at(x, y + 1));   // nonlinear(S)
        m_refMatrixA[8 * strideRefMatrixA + sampleInd]  = cccmModelCb.nonlinear(refLuma.at(x - 1, y));   // nonlinear(W)
        m_refMatrixA[9 * strideRefMatrixA + sampleInd]  = cccmModelCb.nonlinear(refLuma.at(x + 1, y));   // nonlinear(E)
        m_refMatrixA[10 * strideRefMatrixA + sampleInd] = cccmModelCb.bias();

        m_targetVectorCb[sampleInd]   = refCb.at(x, y);
        m_targetVectorCr[sampleInd++] = refCr.at(x, y);
      }
    }
  }

  if (sampleInd == 0)   // Number of samples can go to zero in the multimode case
  {
    cccmModelCb.clearModel();
    cccmModelCr.clearModel();
    return;
  }
  else
  {
    sampleNum = sampleInd;
  }

  m_cccmSolver.solve(*this, sampleNum, m_refMatrixA, strideRefMatrixA, 0, m_targetVectorCb, chromaOffsetCb,
                     &cccmModelCb, m_targetVectorCr, chromaOffsetCr, &cccmModelCr);
}

int IntraPrediction::xBvgCccmCalcBlkAver(const CodingUnit &cu) const
{
  int     numSamples = 0;
  int     sumSamples = 0;
  CPelBuf refLumaBlk = m_cccmAreaInfo.getPredPelBufFor(m_cccmLumaBuf[0]);

  for (int y = 0; y < refLumaBlk.height; y++)
  {
    for (int x = 0; x < refLumaBlk.width; x++)
    {
      sumSamples += refLumaBlk.at(x, y);
      numSamples += 1;
    }
  }
  int bd = cu.slice->m_sps->m_bitDepths[toChannelType(COMP_Y)];
  return numSamples == 0 ? bd : lutDivideInteger(sumSamples, numSamples);
}

void IntraPrediction::xBvgCccmCalcBlkRange(const CodingUnit &cu, int &minVal, int &maxVal) const
{
  minVal = MAX_INT, maxVal = -MAX_INT;
  CPelBuf refLumaBlk = m_cccmAreaInfo.getPredPelBufFor(m_cccmLumaBuf[0]);
  for (int y = 0; y < refLumaBlk.height; y++)
  {
    for (int x = 0; x < refLumaBlk.width; x++)
    {
      if (refLumaBlk.at(x, y) < minVal)
      {
        minVal = refLumaBlk.at(x, y);
      }
      if (refLumaBlk.at(x, y) > maxVal)
      {
        maxVal = refLumaBlk.at(x, y);
      }
    }
  }
  int range = abs(maxVal - minVal) >> 4;
  minVal -= range;
  maxVal += range;
}

void IntraPrediction::xBvgCccmCalcRefArea(const CodingUnit &pu, CompArea chromaArea)
{
  m_bvgCccmBlkArea = chromaArea;
  m_bvgCccmRefArea = chromaArea;
  int refWidth     = chromaArea.width + 2 * CCCM_FILTER_PADDING;
  int refHeight    = chromaArea.height + 2 * CCCM_FILTER_PADDING;
  m_bvgCccmRefArea = Area(chromaArea.x - CCCM_FILTER_PADDING, chromaArea.y - CCCM_FILTER_PADDING, refWidth, refHeight);
}

PelBuf IntraPrediction::xBvgCccmGetLumaCuBufFul(int candIdx) const
{
  int refStride = m_bvgCccmRefArea.width;
  int tuWidth   = m_bvgCccmRefArea.width;
  int tuHeight  = m_bvgCccmRefArea.height;
  return PelBuf(m_bvgCccmLumaBuf[candIdx], refStride, tuWidth, tuHeight);   // Points to the top-left corner
}

PelBuf IntraPrediction::xBvgCccmGetLumaCuBuf(int candIdx) const
{
  int refSizeX  = 0;   // Reference lines available left and above
  int refSizeY  = 0;
  int tuWidth   = m_bvgCccmBlkArea.width;
  int tuHeight  = m_bvgCccmBlkArea.height;
  int refStride = m_bvgCccmBlkArea.width + 2 * CCCM_FILTER_PADDING;   // Including paddings required for the 2D filter
  int refOrigin = refStride * (refSizeY + CCCM_FILTER_PADDING) + refSizeX + CCCM_FILTER_PADDING;

  return PelBuf(m_bvgCccmLumaBuf[candIdx] + refOrigin, refStride, tuWidth,
                tuHeight);   // Points to the top-left corner of the block
}

PelBuf IntraPrediction::xBvgCccmGetChromaCuBuf(const CompID compId, int candIdx) const
{
  int refSizeX  = 0;   // m_bvgCccmBlkArea.x; // Reference lines available left and above
  int refSizeY  = 0;   // m_bvgCccmBlkArea.y;
  int tuWidth   = m_bvgCccmBlkArea.width;
  int tuHeight  = m_bvgCccmBlkArea.height;
  int refStride = m_bvgCccmBlkArea.width + 2 * CCCM_FILTER_PADDING;   // Including paddings required for the 2D filter
  int refOrigin = refStride * (refSizeY + CCCM_FILTER_PADDING) + refSizeX + CCCM_FILTER_PADDING;
  if (compId == COMP_Cb)
  {
    return PelBuf(m_bvgCccmChromaBuf[candIdx][0] + refOrigin, refStride, tuWidth,
                  tuHeight);   // Points to the top-left corner of the block
  }
  else
  {
    return PelBuf(m_bvgCccmChromaBuf[candIdx][1] + refOrigin, refStride, tuWidth,
                  tuHeight);   // Points to the top-left corner of the block
  }
}

void IntraPrediction::xBvgCccmCreateLumaRef(const CodingUnit &cu)
{
  const CPelBuf recoLuma = cu.cs->picture->getRecoBuf(COMP_Y);
  const CPelBuf recoCb   = cu.cs->picture->getRecoBuf(COMP_Cb);
  const CPelBuf recoCr   = cu.cs->picture->getRecoBuf(COMP_Cr);

  const int maxPosPicX = cu.cs->picture->chromaSize().width - 1;
  const int maxPosPicY = cu.cs->picture->chromaSize().height - 1;
  int       ctuWidth   = cu.slice->m_sps->m_maxCuWidth >> getComponentScaleX(COMP_Cb, cu.chromaFormat);
  int       ctuHeight  = cu.slice->m_sps->m_maxCuHeight >> getComponentScaleY(COMP_Cb, cu.chromaFormat);

  CompArea chromaArea = cu.blocks[COMP_Cb];
  int      filterIdx  = 0;
  for (int candIdx = 0; candIdx < cu.numBvgCands; candIdx++)
  {
    Mv chromaBv = cu.bvList[candIdx];
    xBvgCccmCalcRefArea(cu, chromaArea);   // Find the reference area
    PelBuf refLuma = xBvgCccmGetLumaCuBuf(candIdx);
    PelBuf refCb   = xBvgCccmGetChromaCuBuf(COMP_Cb, candIdx);
    PelBuf refCr   = xBvgCccmGetChromaCuBuf(COMP_Cr, candIdx);

    // xCccmSetLumaRefValue( cu );

    int refPosPicX = cu.blocks[COMP_Cb].x + chromaBv.hor;
    int refPosPicY = cu.blocks[COMP_Cb].y + chromaBv.ver;
    int maxWidth   = refLuma.width;
    int maxHeight  = refLuma.height;
    PU::checkIsChromaBvCandidateValid(cu, chromaBv, maxWidth, maxHeight);
    // Generate down-sampled luma for the area covering both the PU and the top/left reference areas (+ top and left
    // paddings)
    for (int y = 0; y < refLuma.height; y++)
    {
      for (int x = 0; x < refLuma.width; x++)
      {
        int chromaPosPicX = refPosPicX + x;
        int chromaPosPicY = refPosPicY + y;

        chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
        chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

        refLuma.at(x, y) = xCccmGetLumaVal(cu, recoLuma, chromaPosPicX, chromaPosPicY, filterIdx);
        refCb.at(x, y)   = recoCb.at(chromaPosPicX, chromaPosPicY);
        refCr.at(x, y)   = recoCr.at(chromaPosPicX, chromaPosPicY);
        // pad if not available
        if (x >= maxWidth)
        {
          refLuma.at(x, y) = refLuma.at(x - 1, y);
          refCb.at(x, y)   = refCb.at(x - 1, y);
          refCr.at(x, y)   = refCr.at(x - 1, y);
        }
        else if (y >= maxHeight)
        {
          refLuma.at(x, y) = refLuma.at(x, y - 1);
          refCb.at(x, y)   = refCb.at(x, y - 1);
          refCr.at(x, y)   = refCr.at(x, y - 1);
        }
      }
    }
    //-- Now fill the out of block samples (North)
    for (int x = 0; x < refLuma.width; x++)
    {
      int y             = -1;
      int chromaPosPicX = refPosPicX + x;
      int chromaPosPicY = refPosPicY + y;

      chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
      chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

      refLuma.at(x, y) = xCccmGetLumaVal(cu, recoLuma, chromaPosPicX, chromaPosPicY, filterIdx);
      if (chromaPosPicY < 0)
      {
        refLuma.at(x, y) = refLuma.at(x, y + 1);
      }
      if (x >= maxWidth)
      {
        refLuma.at(x, y) = refLuma.at(x - 1, y);
      }
    }
    //-- Now fill the out of block samples (South)
    for (int x = 0; x < refLuma.width; x++)
    {
      int y             = refLuma.height;
      int chromaPosPicX = refPosPicX + x;
      int chromaPosPicY = refPosPicY + y;

      chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
      chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

      refLuma.at(x, y) = xCccmGetLumaVal(cu, recoLuma, chromaPosPicX, chromaPosPicY, filterIdx);

      if (chromaPosPicY >= maxPosPicY)
      {
        refLuma.at(x, y) = refLuma.at(x, y - 1);
      }
      if (!(chromaPosPicY <= maxPosPicY && (chromaPosPicY & (ctuHeight - 1))))
      {
        refLuma.at(x, y) = refLuma.at(x, y - 1);
      }
      if (maxHeight < refLuma.height)
      {
        refLuma.at(x, y) = refLuma.at(x, y - 1);
      }
      if (x >= maxWidth)
      {
        refLuma.at(x, y) = refLuma.at(x - 1, y);
      }
    }
    //-- Now fill the out of block samples (West)
    for (int y = 0; y < refLuma.height; y++)
    {
      int x             = -1;
      int chromaPosPicX = refPosPicX + x;
      int chromaPosPicY = refPosPicY + y;

      chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
      chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

      refLuma.at(x, y) = xCccmGetLumaVal(cu, recoLuma, chromaPosPicX, chromaPosPicY, filterIdx);
      if (chromaPosPicX < 0)
      {
        refLuma.at(x, y) = refLuma.at(x + 1, y);
      }
      if (y >= maxHeight)
      {
        refLuma.at(x, y) = refLuma.at(x, y - 1);
      }
    }
    //-- Now fill the out of block samples (East)
    for (int y = 0; y < refLuma.height; y++)
    {
      int x             = refLuma.width;
      int chromaPosPicX = refPosPicX + x;
      int chromaPosPicY = refPosPicY + y;

      chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
      chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

      refLuma.at(x, y) = xCccmGetLumaVal(cu, recoLuma, chromaPosPicX, chromaPosPicY, filterIdx);
      if (chromaPosPicX >= maxPosPicX)
      {
        refLuma.at(x, y) = refLuma.at(x - 1, y);
      }
      if (!(chromaPosPicX <= maxPosPicX && (chromaPosPicX & (ctuWidth - 1))))
      {
        refLuma.at(x, y) = refLuma.at(x - 1, y);
      }
      if (maxWidth < refLuma.width)
      {
        refLuma.at(x, y) = refLuma.at(x - 1, y);
      }
      if (y >= maxHeight)
      {
        refLuma.at(x, y) = refLuma.at(x, y - 1);
      }
    }
    /*
    int rribcFlipType = pu.rrIbcList[candIdx];
    if (rribcFlipType > 0)
    {
      PelBuf refLumaExt = xBvgCccmGetLumaPuBufFul(pu, candIdx);
      refLumaExt.flipSignal(rribcFlipType == 1);
      refCb.flipSignal(rribcFlipType == 1);
      refCr.flipSignal(rribcFlipType == 1);
    }*/
  }
}

void IntraPrediction::cccmCreateLumaRefsForMergeList(const CodingUnit &cu, const CrossCompModels *mergeList,
                                                     const int mergeListSize)
{
  std::vector<ConvModelType> cccmTypeVec;

  for (int i = 0; i < mergeListSize; i++)
  {
    const ConvModelType modelType = mergeList[i].modelCb[0].modelType;

    if (modelType >= CONV_MODEL_CCCM_INTRA_FIRST && modelType <= CONV_MODEL_CCCM_INTRA_LAST)
    {
      if (std::find(cccmTypeVec.begin(), cccmTypeVec.end(), modelType) == cccmTypeVec.end())
      {
        cccmTypeVec.push_back(modelType);
      }
    }
  }

  for (int i = 0; i < cccmTypeVec.size(); i++)
  {
    cccmCreateLumaRefs(cu, false, cccmTypeVec[i]);
  }
}

void IntraPrediction::cccmCreateLumaRefs(const CodingUnit &cu, const bool createAll, const ConvModelType cccmTypeReq)
{
  ConvModelType cccmType = cccmTypeReq == CONV_MODEL_UNDEFINED ? cu.cccmType : cccmTypeReq;

  xCccmCalcRefArea(cu);
  xCccmSetLumaRefValue(cu);

  if (createAll)   // For the encoder
  {
    xCccmCreateLumaRef(cu, 0);
    xCccmCreateLumaRef(cu, 1);
    xCccmCreateLumaRef(cu, 2);
    xCccmCreateLumaRef(cu, 3);
    xCccmCreateLumaRefNoSubs(cu, 4);
    xBvgCccmCreateLumaRef(cu);
  }
  else if (cccmType == CONV_MODEL_CCCM_BVG)
  {
    xCccmCreateLumaRef(cu, 0);
    xBvgCccmCreateLumaRef(cu);
  }
  else if (cccmType == CONV_MODEL_CCCM || cccmType == CONV_MODEL_CCCM_GRADLOC)
  {
    xCccmCreateLumaRef(cu, 0);
  }
  else if (cccmType == CONV_MODEL_CCCM_NOSUBS)
  {
    xCccmCreateLumaRefNoSubs(cu, 4);
  }
  else
  {
    xCccmCreateLumaRef(cu, 0);

    if (cccmType == CONV_MODEL_CCCM_MULTIF_1)
    {
      xCccmCreateLumaRef(cu, 1);
      xCccmCreateLumaRef(cu, 2);
      xCccmCreateLumaRef(cu, 3);
    }
    else if (cccmType == CONV_MODEL_CCCM_MULTIF_2)
    {
      xCccmCreateLumaRef(cu, 1);
    }
    else   // if (cccmType == CONV_MODEL_CCCM_MULTIF_3)
    {
      xCccmCreateLumaRef(cu, 3);
    }
  }
}

void IntraPrediction::xCccmCreateLumaRefNoSubs(const CodingUnit &cu, const int filterIndex)
{
  const CPelBuf recoLuma = cu.cs->picture->getRecoBuf(COMP_Y);

  const int maxPosPicX = cu.cs->picture->lumaSize().width - 1;
  const int maxPosPicY = cu.cs->picture->lumaSize().height - 1;

  int areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY;

  m_cccmAreaInfoNoSubs.getRefParams(areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY);

  PelBuf refLuma = m_cccmAreaInfoNoSubs.getRefPelBufFor(m_cccmLumaBuf[filterIndex]);

  int puBorderX = refSizeX + m_cccmAreaInfoNoSubs.blkArea.width;
  int puBorderY = refSizeY + m_cccmAreaInfoNoSubs.blkArea.height;

  // Generate non-downsampled luma for the area covering both the PU and the top/left reference areas (+ top and left
  // paddings) Note: no padding is needed on the right as the right luma neighbor always exists
  for (int y = -CCCM_FILTER_PADDING; y < areaHeight; y++)
  {
    for (int x = -CCCM_FILTER_PADDING; x < areaWidth; x++)
    {
      if ((x >= puBorderX && y >= refSizeY) || (y >= puBorderY && x >= refSizeX))
      {
        continue;
      }

      int lumaPosPicX = refPosPicX + x;
      int lumaPosPicY = refPosPicY + y;

      lumaPosPicX = lumaPosPicX < 0 ? 0 : lumaPosPicX > maxPosPicX ? maxPosPicX : lumaPosPicX;
      lumaPosPicY = lumaPosPicY < 0 ? 0 : lumaPosPicY > maxPosPicY ? maxPosPicY : lumaPosPicY;

      refLuma.at(x, y) = recoLuma.at(lumaPosPicX, lumaPosPicY);
    }
  }
}

void IntraPrediction::xCccmCreateLumaRef(const CodingUnit &cu, const int filterIndex)
{
  const CPelBuf recoLuma   = cu.cs->picture->getRecoBuf(COMP_Y);
  const int     maxPosPicX = cu.cs->picture->chromaSize().width - 1;
  const int     maxPosPicY = cu.cs->picture->chromaSize().height - 1;

  int areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY;

  m_cccmAreaInfo.getRefParams(areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY);

  PelBuf refLuma = m_cccmAreaInfo.getRefPelBufFor(m_cccmLumaBuf[filterIndex]);

  int puBorderX = refSizeX + cu.blocks[COMP_Cb].width;
  int puBorderY = refSizeY + cu.blocks[COMP_Cb].height;

  // Generate down-sampled luma for the area covering both the PU and the top/left reference areas (+ top and left
  // paddings)
  int xEnd = areaWidth;
  for (int y = -CCCM_FILTER_PADDING; y < areaHeight; y++)
  {
    if (y == refSizeY)
    {
      xEnd = puBorderX;
    }

    if (y == puBorderY)
    {
      xEnd = refSizeX;
    }

    CHECKD(xEnd > areaWidth, "");

    for (int x = -CCCM_FILTER_PADDING; x < xEnd; x++)
    {
      int chromaPosPicX = refPosPicX + x;
      int chromaPosPicY = refPosPicY + y;

      chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
      chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

      refLuma.at(x, y) = xCccmGetLumaVal(cu, recoLuma, chromaPosPicX, chromaPosPicY, filterIndex);
    }
  }

  CHECK(CCCM_FILTER_PADDING != 1, "Only padding with one sample implemented");

  // Pad right of top reference area
  for (int y = -1; y < refSizeY; y++)
  {
    refLuma.at(areaWidth, y) = refLuma.at(areaWidth - 1, y);
  }

  // Pad right of PU
  for (int y = refSizeY; y < puBorderY; y++)
  {
    refLuma.at(puBorderX, y) = refLuma.at(puBorderX - 1, y);
  }

  // Pad right of left reference area
  for (int y = puBorderY; y < areaHeight; y++)
  {
    refLuma.at(refSizeX, y) = refLuma.at(refSizeX - 1, y);
  }

  // Pad below left reference area
  for (int x = -1; x < refSizeX + 1; x++)
  {
    refLuma.at(x, areaHeight) = refLuma.at(x, areaHeight - 1);
  }

  // Pad below PU
  for (int x = refSizeX; x < puBorderX + 1; x++)
  {
    refLuma.at(x, puBorderY) = refLuma.at(x, puBorderY - 1);
  }

  // Pad below right reference area
  for (int x = puBorderX + 1; x < areaWidth + 1; x++)
  {
    refLuma.at(x, refSizeY) = refLuma.at(x, refSizeY - 1);
  }

  // In dualtree we can also use luma from the right and below (if not on CTU/picture boundary)
  if (CS::isDualITree(*cu.cs))
  {
    int ctuWidth   = cu.slice->m_sps->m_maxCuWidth >> getComponentScaleX(COMP_Cb, cu.chromaFormat);
    int ctuHeight  = cu.slice->m_sps->m_maxCuHeight >> getComponentScaleY(COMP_Cb, cu.chromaFormat);
    // Samples right of top reference area
    int padPosPicX = refPosPicX + areaWidth;
    if (padPosPicX <= maxPosPicX && (padPosPicX & (ctuWidth - 1)))
    {
      for (int y = -1; y < refSizeY; y++)
      {
        int chromaPosPicY = refPosPicY + y;
        chromaPosPicY     = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

        refLuma.at(areaWidth, y) = xCccmGetLumaVal(cu, recoLuma, padPosPicX, chromaPosPicY, filterIndex);
      }
    }

    // Samples right of PU
    padPosPicX = refPosPicX + puBorderX;
    if (padPosPicX <= maxPosPicX && (padPosPicX & (ctuWidth - 1)))
    {
      for (int y = refSizeY; y < puBorderY; y++)
      {
        int chromaPosPicY = refPosPicY + y;
        chromaPosPicY     = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

        refLuma.at(puBorderX, y) = xCccmGetLumaVal(cu, recoLuma, padPosPicX, chromaPosPicY, filterIndex);
      }
    }

    // Samples right of left reference area
    padPosPicX = refPosPicX + refSizeX;

    if (padPosPicX <= maxPosPicX)
    {
      for (int y = puBorderY; y < areaHeight; y++)
      {
        int chromaPosPicY = refPosPicY + y;
        chromaPosPicY     = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

        refLuma.at(refSizeX, y) = xCccmGetLumaVal(cu, recoLuma, padPosPicX, chromaPosPicY, filterIndex);
      }
    }

    // Samples below left reference area
    int padPosPicY = refPosPicY + areaHeight;
    if (padPosPicY <= maxPosPicY && (padPosPicY & (ctuHeight - 1)))
    {
      for (int x = -1; x < refSizeX + 1; x++)
      {
        int chromaPosPicX = refPosPicX + x;
        chromaPosPicX     = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;

        refLuma.at(x, areaHeight) = xCccmGetLumaVal(cu, recoLuma, chromaPosPicX, padPosPicY, filterIndex);
      }
    }

    // Samples below PU
    padPosPicY = refPosPicY + puBorderY;
    if (padPosPicY <= maxPosPicY && (padPosPicY & (ctuHeight - 1)))
    {
      for (int x = refSizeX; x < puBorderX;
           x++)   // Just go to PU border as the next sample may be out of CTU (and not needed anyways)
      {
        int chromaPosPicX = refPosPicX + x;
        chromaPosPicX     = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;

        refLuma.at(x, puBorderY) = xCccmGetLumaVal(cu, recoLuma, chromaPosPicX, padPosPicY, filterIndex);
      }
    }

    // Samples below right reference area
    padPosPicY = refPosPicY + refSizeY;

    if (padPosPicY <= maxPosPicY)
    {
      // Avoid going outside of right CTU border where these samples are not yet available
      int puPosPicX        = cu.blocks[COMP_Cb].x;
      int ctuRightEdgeDist = ctuWidth - (puPosPicX & (ctuWidth - 1)) + refSizeX;
      int lastPosX         = ctuRightEdgeDist < areaWidth ? ctuRightEdgeDist : areaWidth;

      for (int x = puBorderX + 1; x < lastPosX;
           x++)   // Just go to ref area border as the next sample may be out of CTU (and not needed anyways)
      {
        int chromaPosPicX = refPosPicX + x;
        chromaPosPicX     = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;

        refLuma.at(x, refSizeY) = xCccmGetLumaVal(cu, recoLuma, chromaPosPicX, padPosPicY, filterIndex);
      }
    }
  }
}

void IntraPrediction::initIntraEip(CodingUnit &cu, const CompArea &area)
{
  const ChannelType      chType = toChannelType(area.compID);
  const CodingStructure &cs     = *cu.cs;
  const SPS             &sps    = *cs.sps;
  const PreCalcValues   &pcv    = *cs.pcv;
  const int              height = area.height;
  const int              width  = area.width;

  const bool noShift      = pcv.noChroma2x2 && width == 4;   // don't shift on the lowest level (chroma not-split)
  const int unitWidthLog2 = pcv.minCUWidthLog2 - (noShift ? 0 : getComponentScaleX(area.compID, sps.m_chromaFormatIdc));
  const int unitHeightLog2 =
    pcv.minCUHeightLog2 - (noShift ? 0 : getComponentScaleY(area.compID, sps.m_chromaFormatIdc));
  const int unitWidth  = 1 << unitWidthLog2;
  const int unitHeight = 1 << unitHeightLog2;

  const int totalAboveUnits    = (2 * width + (unitWidth - 1)) >> unitWidthLog2;
  const int totalLeftUnits     = (2 * height + (unitHeight - 1)) >> unitHeightLog2;
  const int numAboveUnits      = std::max<int>(width >> unitWidthLog2, 1);
  const int numLeftUnits       = std::max<int>(height >> unitHeightLog2, 1);
  const int numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int numLeftBelowUnits  = totalLeftUnits - numLeftUnits;

  static bool neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1] {};   // Just a dummy array here, content not used
  int         avaiAboveRightUnits = isAboveRightAvailable(cu, chType, area.topRight(), numAboveRightUnits, unitWidth,
                                                          (neighborFlags + totalLeftUnits + 1 + numAboveUnits), 0);
  int         avaiLeftBelowUnits  = isBelowLeftAvailable(cu, chType, area.bottomLeft(), numLeftBelowUnits, unitHeight,
                                                         (neighborFlags + totalLeftUnits - 1 - numLeftUnits), 0);

  const Position tplSize = getEipRecoPosition(cu, area.compID);
  m_eipBlkArea.x         = tplSize.x + EIP_FILTER_SIZE;
  m_eipBlkArea.y         = tplSize.y + EIP_FILTER_SIZE;
  m_eipBlkArea.width     = width + m_eipBlkArea.x + avaiAboveRightUnits * unitWidth;
  m_eipBlkArea.height    = height + m_eipBlkArea.y + avaiLeftBelowUnits * unitHeight;
  m_eipLumaOffset        = cs.picture->getRecoBuf(cu.blocks[area.compID]).at(-1, -1);

  // fill reconstruction buffer
  const int refSizeY   = m_eipBlkArea.y;
  const int refSizeX   = m_eipBlkArea.x;
  const int refHeight  = m_eipBlkArea.height;
  const int refWidth   = m_eipBlkArea.width;
  const int refPosPicX = cu.blocks[area.compID].x - refSizeX;
  const int refPosPicY = cu.blocks[area.compID].y - refSizeY;
  CPelBuf   piReco     = cs.picture->getRecoBuf(area.compID);

  CPelBuf recoTopAndTopLeft = piReco.subBuf(refPosPicX, refPosPicY, refWidth, refSizeY);
  PelBuf  refTopAndTopLeft(m_eipBuffer, refWidth, refWidth, refSizeY);
  refTopAndTopLeft.copyFrom(recoTopAndTopLeft);

  CPelBuf recoLeft = piReco.subBuf(refPosPicX, refPosPicY + refSizeY, refSizeX, refHeight - refSizeY);
  PelBuf  refLeft(m_eipBuffer + refWidth * refSizeY, refWidth, refSizeX, refHeight - refSizeY);
  refLeft.copyFrom(recoLeft);

  int       min        = MAX_INT;
  int       max        = 0;
  int       startY     = refPosPicY < 0 ? -refPosPicY : 0;
  const int startX     = refPosPicX < 0 ? -refPosPicX : 0;
  int       numSamples = 0;
  uint64_t  sum        = 0;
  for (int y = startY; y < refTopAndTopLeft.height; y++)
  {
    for (int x = startX; x < refTopAndTopLeft.width; x++)
    {
      int sample = refTopAndTopLeft.at(x, y);
      min        = sample < min ? sample : min;
      max        = sample > max ? sample : max;
      if (y >= (refTopAndTopLeft.height >> 1) && x >= (refSizeX >> 1))
      {
        sum += sample;
        numSamples += 1;
      }
    }
  }
  startY = (refPosPicY + refSizeY) < 0 ? -(refPosPicY + refSizeY) : 0;
  for (int y = startY; y < refLeft.height; y++)
  {
    for (int x = startX; x < refLeft.width; x++)
    {
      int sample = refLeft.at(x, y);
      min        = sample < min ? sample : min;
      max        = sample > max ? sample : max;
      if (x >= (refLeft.width >> 1))
      {
        sum += sample;
        numSamples += 1;
      }
    }
  }
  m_eipClipMin = min;
  m_eipClipMax = max;
  m_eipBias    = 1 << (cu.slice->m_sps->m_bitDepths[chType] - 1);
  m_eipAvg     = Pel(sum / numSamples);
}

void LeastSquaresSolver::solve(const IntraPrediction &intraPrediction, const int sampleNum, const Pel *inputMat,
                               const int inputStride, const int extraRegularizationParam, const Pel *targetVec0,
                               const int offset0, ConvModel *model0, const Pel *targetVec1, const int offset1,
                               ConvModel *model1)
{
  static TCccmCoeff ata[SOLVER_NUM_PARAMS_MAX][SOLVER_NUM_PARAMS_MAX + 2];

  const int numParams = model0->getNumParams();
  const int bitdepth  = model0->bd;
  const int numModels = model1 ? 2 : 1;

  CHECK(model1 && model1->getNumParams() != numParams, "Chroma number of parameters don't match");
  CHECK(model1 && model1->bd != bitdepth, "Chroma bitdepths don't match");
  CHECK(CCCM_MAX_REF_SAMPLES < sampleNum, "Insufficient buffer size");
  CHECK(SOLVER_NUM_PARAMS_MAX < numParams, "Insufficient buffer size");

  int  matrixBits  = CCCM_MATRIX_BITS;   // May be different for different tools
  int  matrixShift = matrixBits - 2 * bitdepth - ceilLog2(sampleNum);
  bool shiftUp     = matrixShift >= 0;
  int  shift       = shiftUp ? matrixShift : -matrixShift;

  const Pel *data[SOLVER_NUM_PARAMS_MAX + 2];
  for (int y = 0; y < numParams; ++y)
  {
    data[y] = &inputMat[y * inputStride];
  }

  // Append data for additional "columns"
  data[numParams + 0] = targetVec0;
  data[numParams + 1] = targetVec1;

  // Calculate autocorrelation matrix along with additional columns that will become cross-correlation vector(s)
  intraPrediction.m_calculateCorrelationMatrix(numParams, numParams + numModels, sampleNum, data, &ata[0][0],
                                               &ata[1][0] - &ata[0][0]);

  // Scale cross-correlation vectors with matrixShift
  for (int m = 0; m < numModels; ++m)
  {
    for (int y = 0; y < numParams; ++y)
    {
      // Deduct impact of bias offsets from the cross-correlation vectors
      auto &sum = ata[y][numParams + m];
      sum -= ((ata[y][numParams - 1] * (m ? offset1 : offset0)) >> (bitdepth - 1));
      sum = shiftUp ? sum << shift : sum >> shift;
    }
  }

  // Scale also ATA, populate lower triangular area (ATA is symmetric) and apply regularizations.
  int regBase  = 2 << (bitdepth - 8);
  int regExtra = extraRegularizationParam;

  for (int y = 0; y < numParams; y++)
  {
    for (int x = y; x < numParams; x++)
    {
      ata[y][x] = ata[x][y] = shiftUp ? ata[y][x] << shift : ata[y][x] >> shift;
    }

    ata[y][y] += regBase + (y < numParams - 1 ? regExtra : 0);
  }

  // Solve the filter coefficients
  gaussElimination(ata, model0->params.data(), numModels == 2 ? model1->params.data() : nullptr, numParams, numModels);

  // Bring offsets back to the bias terms (after shifting up by CCCM_DECIM_BITS and down by bitdepth - 1)
  if (numModels == 2)
  {
    model0->params[numParams - 1] += offset0 << (CCCM_DECIM_BITS - (bitdepth - 1));
    model1->params[numParams - 1] += offset1 << (CCCM_DECIM_BITS - (bitdepth - 1));
  }
  else
  {
    model0->params[numParams - 1] += offset0 << (CCCM_DECIM_BITS - (bitdepth - 1));
  }
}

void LeastSquaresSolver::gaussBacksubstitution(const TCccmCoeff C[SOLVER_NUM_PARAMS_MAX][SOLVER_NUM_PARAMS_MAX + 2],
                                               TCccmCoeff *x, int numParams, int col)
{
  x[numParams - 1] = C[numParams - 1][col];

  for (int i = numParams - 2; i >= 0; i--)
  {
    x[i] = C[i][col];

    for (int j = i + 1; j < numParams; j++)
    {
      x[i] -= FIXED_MULT(C[i][j], x[j]);
    }
  }
}

void LeastSquaresSolver::gaussElimination(TCccmCoeff  C[SOLVER_NUM_PARAMS_MAX][SOLVER_NUM_PARAMS_MAX + 2],
                                          TCccmCoeff *x0, TCccmCoeff *x1, int numParams, int numModels)
{
  int colChr0 = numParams;
  int colChr1 = numParams + 1;

  for (int i = 0; i < numParams; i++)
  {
    TCccmCoeff *src  = C[i];
    TCccmCoeff  diag = src[i] < 1 ? 1 : src[i];

    auto [scale, round, shift] = lutDivideGetScaleRoundShift(diag, CCCM_DECIM_BITS);

    for (int j = i + 1; j < numParams + numModels; j++)
    {
      src[j] = (int64_t(src[j]) * scale + round) >> shift;
    }

    for (int j = i + 1; j < numParams; j++)
    {
      TCccmCoeff *dst   = C[j];
      TCccmCoeff  scale = dst[i];

      // On row j all elements with k < i+1 are now zero (not zeroing those here as backsubstitution does not need them)
      for (int k = i + 1; k < numParams + numModels; k++)
      {
        dst[k] -= FIXED_MULT(scale, src[k]);
      }
    }
  }

  // Solve with backsubstitution
  if (numModels == 2)
  {
    gaussBacksubstitution(C, x0, numParams, colChr0);
    gaussBacksubstitution(C, x1, numParams, colChr1);
  }
  else
  {
    gaussBacksubstitution(C, x0, numParams, colChr0);
  }
}

void CccmCovariance::gaussElimination(TCccmCoeff A[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff *y0,
                                      TCccmCoeff *x0, TCccmCoeff *y1, TCccmCoeff *x1, int numEq, int numFilters, int bd,
                                      const bool interCccmMode)
{
  int colChr0 = numEq;
  int colChr1 = numEq + 1;
  int reg     = interCccmMode ? 1 : 2 << (bd - 8);

  const int decimBits  = DECIM_BITS(bd);
  const int decimRound = (1 << (decimBits - 1));

  // Create an [M][M+2] matrix system (could have been done already when calculating auto/cross-correlations)
  for (int i = 0; i < numEq; i++)
  {
    for (int j = 0; j < numEq; j++)
    {
      C[i][j] = j >= i ? A[i][j] : A[j][i];
    }

    C[i][i] += reg;   // Regularization
    C[i][colChr0] = y0[i];
    C[i][colChr1] = numFilters == 2 ? y1[i] : 0;   // Only applicable if solving for 2 filters at the same time
  }

  for (int i = 0; i < numEq; i++)
  {
    TCccmCoeff *src  = C[i];
    TCccmCoeff  diag = src[i] < 1 ? 1 : src[i];

    auto [scale, round, shift] = lutDivideGetScaleRoundShift(diag, decimBits);

    for (int j = i + 1; j < numEq + numFilters; j++)
    {
      src[j] = (int64_t(src[j]) * scale + round) >> shift;
    }

    for (int j = i + 1; j < numEq; j++)
    {
      TCccmCoeff *dst   = C[j];
      TCccmCoeff  scale = dst[i];

      // On row j all elements with k < i+1 are now zero (not zeroing those here as backsubstitution does not need them)
      for (int k = i + 1; k < numEq + numFilters; k++)
      {
#define FIXED_MULT4(x, y, round, bits) TCccmCoeff((int64_t(x) * (y) + round) >> bits)
        dst[k] -= FIXED_MULT4(scale, src[k], decimRound, decimBits);
      }
    }
  }

  // Solve with backsubstitution
  if (numFilters == 2)
  {
    gaussBacksubstitution(x0, numEq, colChr0, decimRound, decimBits);
    gaussBacksubstitution(x1, numEq, colChr1, decimRound, decimBits);
  }
  else
  {
    gaussBacksubstitution(x0, numEq, colChr0, decimRound, decimBits);
  }
}

void CccmCovariance::gaussBacksubstitution(TCccmCoeff *x, int numEq, int col, int round, int bits)
{
  x[numEq - 1] = C[numEq - 1][col];

  for (int i = numEq - 2; i >= 0; i--)
  {
    x[i] = C[i][col];

    for (int j = i + 1; j < numEq; j++)
    {
      x[i] -= FIXED_MULT4(C[i][j], x[j], round, bits);
    }
  }
}

void CccmCovariance::solve3(TCccmCoeff ATA[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX],
                            TCccmCoeff ATCb[CCCM_NUM_PARAMS_MAX], TCccmCoeff ATCr[CCCM_NUM_PARAMS_MAX],
                            const int sampleNum, const int chromaOffsetCb, const int chromaOffsetCr, CccmModel &modelCb,
                            CccmModel &modelCr, const bool interCccmMode)
{

  const int numParams = modelCb.getNumParams();

  CHECK(modelCr.getNumParams() != numParams, "Chroma number of parameters don't match");
  CHECK(CCCM_REF_SAMPLES_MAX < sampleNum, "Insufficient buffer size");
  CHECK(CCCM_NUM_PARAMS_MAX < numParams, "Insufficient buffer size");

  // Remove chromaOffset from stats to update cross-correlation
  for (int coli = 0; coli < numParams; coli++)
  {
    ATCb[coli] = ATCb[coli] - ((ATA[coli][numParams - 1] * chromaOffsetCb) >> (modelCb.bd - 1));
    ATCr[coli] = ATCr[coli] - ((ATA[coli][numParams - 1] * chromaOffsetCr) >> (modelCr.bd - 1));
  }

  // Scale the matrix and vector to selected dynamic range
  CHECK(modelCb.bd != modelCr.bd, "Bitdepth of Cb and Cr is different");
  static const int CCCM_MATRIX_BITS_HBD = 32;
  int              matrixShift = ((modelCb.bd > 10) ? CCCM_MATRIX_BITS_HBD : (interCccmMode ? 28 : CCCM_MATRIX_BITS)) -
    2 * modelCb.bd - ceilLog2(sampleNum);

  if (matrixShift > 0)
  {
    for (int coli0 = 0; coli0 < numParams; coli0++)
    {
      for (int coli1 = coli0; coli1 < numParams; coli1++)
      {
        ATA[coli0][coli1] <<= matrixShift;
      }
    }

    for (int coli = 0; coli < numParams; coli++)
    {
      ATCb[coli] <<= matrixShift;
    }

    for (int coli = 0; coli < numParams; coli++)
    {
      ATCr[coli] <<= matrixShift;
    }
  }
  else if (matrixShift < 0)
  {
    matrixShift = -matrixShift;

    for (int coli0 = 0; coli0 < numParams; coli0++)
    {
      for (int coli1 = coli0; coli1 < numParams; coli1++)
      {
        ATA[coli0][coli1] >>= matrixShift;
      }
    }

    for (int coli = 0; coli < numParams; coli++)
    {
      ATCb[coli] >>= matrixShift;
    }

    for (int coli = 0; coli < numParams; coli++)
    {
      ATCr[coli] >>= matrixShift;
    }
  }

  // Solve the filter coefficients
  gaussElimination(ATA, ATCb, modelCb.params.data(), ATCr, modelCr.params.data(), numParams, 2, modelCb.bd,
                   interCccmMode);

  // Add the chroma offset to bias term (after shifting up by CCCM_DECIM_BITS and down by cccmModelCb.bd - 1)
  modelCb.params[numParams - 1] += chromaOffsetCb << (modelCb.decimBits - (modelCb.bd - 1));
  modelCr.params[numParams - 1] += chromaOffsetCr << (modelCr.decimBits - (modelCr.bd - 1));
}

//! \}
