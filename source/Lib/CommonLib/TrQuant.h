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

/** \file     TrQuant.h
    \brief    transform and quantization class (header)
*/

#ifndef __TRQUANT__
#define __TRQUANT__

#include "CommonDef.h"
#include "Unit.h"
#include "ChromaFormat.h"
#include "Contexts.h"
#include "ContextModelling.h"

#include "UnitPartitioner.h"
#include "Quant.h"

#include "DepQuant.h"
//! \ingroup CommonLib
//! \{

typedef void FwdTrans(const TCoeff *, TCoeff *, int, int, int, int);
typedef void InvTrans(const TCoeff *, TCoeff *, int, int, int, int, const TCoeff, const TCoeff);

// ====================================================================================================================
// Class definition
// ====================================================================================================================

using CbfMaskList = static_vector<int, 3>;
using TransList   = static_vector<MtsType, size_t(MtsType::NUM)>;
using TransBuffer = EnumArray<TCoeff[MAX_TB_SIZEY * MAX_TB_SIZEY], MtsType>;

inline bool includesNST(const TransList &tl)
{
  for (const auto &t: tl)
  {
    if (isNST(t))
    {
      return true;
    }
  }
  return false;
}

/// transform and quantization class
class TrQuant
{
public:
  TrQuant();
  ~TrQuant();

  // initialize class
  void init(const Quant *otherQuant, const uint32_t uiMaxTrSize, const bool bUseRDOQ, const bool bUseRDOQTS,
            const bool useSelectiveRDOQ, const bool bEnc);
  static void getTrTypes(const TransformUnit &tu, const CompID compID, TransType &trTypeHor, TransType &trTypeVer);

  bool getTransposeFlag(uint32_t intraMode);

  static const int8_t *getNsptMatrix(const uint32_t mode, const uint32_t width, const uint32_t height, int nsptIdx,
                                     int setIdx);

protected:
  void xFwdLfnst(const TransformUnit &tu, const CompID compID, TCoeff *buf = nullptr);
  void xInvLfnst(const TransformUnit &tu, const CompID compID);
  void xFwdNspt(const TransformUnit &tu, TCoeff *src, TCoeff *dst, const CompID compID, const int shift, int lfnstIdx);
  void xInvNspt(const TransformUnit &tu, const TCoeff *src, TCoeff *dst, const CompID compID, const int shift,
                int lfnstIdx);

public:
  void invTransformNxN(TransformUnit &tu, const CompID &compID, PelBuf &pResi, const QpParam &cQPs,
                       bool skipDequant = false);
  void preCalcTrans(TransformUnit &tu, const CompID &compID, const CPelBuf &res, const TransList &tl, TransBuffer &tbuf,
                    const bool dctAvailable = false);
  void quantNxN(TransformUnit &tu, const CompID &compID, const QpParam &cQP, TCoeff &absSum, const Ctx &ctx,
                TransBuffer &tcoeff);

  void transformSkipQuantOneSample(TransformUnit &tu, const CompID &compID, const TCoeff &resiDiff, TCoeff &coeff,
                                   const uint32_t &uiPos, const QpParam &cQP, const bool bUseHalfRoundingPoint);
  void invTrSkipDeQuantOneSample(TransformUnit &tu, const CompID &compID, const TCoeff &pcCoeff, Pel &reconSample,
                                 const uint32_t &uiPos, const QpParam &cQP);

  void                        invTransformICT(const TransformUnit &tu, PelBuf &resCb, PelBuf &resCr);
  std::pair<int64_t, int64_t> fwdTransformICT(const TransformUnit &tu, const PelBuf **resCbCr, PelBuf &resJCCR,
                                              int jointCbCr = -1);

  CbfMaskList selectICTCandidates(const TransformUnit &tu, const PelBuf **resOrg, PelBuf **resJCCR);

#if RDOQ_CHROMA_LAMBDA
  void setLambdas(const double lambdas[MAX_NUM_COMP]) { m_quant->setLambdas(lambdas); }
  void selectLambda(const CompID compIdx) { m_quant->selectLambda(compIdx); }
  void getLambdas(double (&lambdas)[MAX_NUM_COMP]) const { m_quant->getLambdas(lambdas); }
#endif
  void   setLambda(const double dLambda) { m_quant->setLambda(dLambda); }
  double getLambda() const { return m_quant->getLambda(); }

  DepQuant *getQuant() { return m_quant; }
  void      resetStore() { m_quant->resetStore(); }

  bool prdCoeffSigns(TransformUnit &tu, const CompID compID, const std::vector<Pel> *fwdLUT = nullptr);
  bool recCoeffSigns(TransformUnit &tu, const CompID compID, const std::vector<Pel> *fwdLUT = nullptr);

protected:
  TCoeff               *m_tempCoeff;
  TCoeff               *m_blk;
  TCoeff               *m_tmp;
  Pel                  *m_tempResi;
  std::vector<Position> m_predSignPos;

private:
  DepQuant *m_quant;          //!< Quantizer

  TCoeff m_tempInMatrix[L16W_ZO];
  TCoeff m_tempOutMatrix[L16W_ZO];
  TCoeff m_nsptTempInMatrix[256];
  TCoeff m_nsptTempOutMatrix[256];

  static const int maxAbsIctMode = 3;
  void (*m_invICTMem[1 + 2 * maxAbsIctMode])(PelBuf &, PelBuf &);
  std::pair<int64_t, int64_t> (*m_fwdICTMem[1 + 2 * maxAbsIctMode])(const CPelBuf &, const CPelBuf &, PelBuf &);
  void (**m_invICT)(PelBuf &, PelBuf &);
  std::pair<int64_t, int64_t> (**m_fwdICT)(const CPelBuf &, const CPelBuf &, PelBuf &);

  void (*m_fwdLfnstNxN)(TCoeff *src, TCoeff *dst, const uint32_t mode, const uint32_t index, const uint32_t size,
                        int zeroOutSize);
  void (*m_invLfnstNxN)(TCoeff *src, TCoeff *dst, const uint32_t mode, const uint32_t index, const uint32_t size,
                        int zeroOutSize, const int maxLog2TrDynamicRange);

  void (*m_fwdNsptNxN)(TCoeff *src, TCoeff *dst, const uint32_t mode, const uint32_t width, const uint32_t height,
                       const int shift, int zeroOutSize, int nsptIdx, int setIdx);
  void (*m_invNsptNxN)(TCoeff *src, TCoeff *dst, const uint32_t mode, const uint32_t width, const uint32_t height,
                       const int shift, int zeroOutSize, int nsptIdx, int setIdx, const int maxLog2TrDynamicRange);

  // forward Transform
  void xT(const TransformUnit &tu, const CompID &compID, const CPelBuf &resi, CoeffBuf &dstCoeff, const int width,
          const int height);

  // skipping Transform
  void xTransformSkip(const TransformUnit &tu, const CompID &compID, const CPelBuf &resi, TCoeff *psCoeff);

  // quantization
  void xQuant(TransformUnit &tu, const CompID &compID, const CCoeffBuf &pSrc, TCoeff &absSum, const QpParam &cQP,
              const Ctx &ctx);

  // dequantization
  void xDeQuant(const TransformUnit &tu, CoeffBuf &dstCoeff, const CompID &compID, const QpParam &cQP);

  // inverse transform
  void xIT(const TransformUnit &tu, const CompID &compID, const CCoeffBuf &pCoeff, PelBuf &pResidual);

  // inverse skipping transform
  void xITransformSkip(const CCoeffBuf &plCoef, PelBuf &pResidual, const TransformUnit &tu, const CompID &component);

  class TrBrdTemplate
  {
  public:
    TrBrdTemplate(const AreaBuf<const int8_t> &buf, const Size &log2Size) : templateBuf(buf), maxLog2SPSize(log2Size) {}
    const int8_t *operator[](const Position &pos) const
    {
      CHECKD(pos.y >= (1 << maxLog2SPSize.height), "y-pos outside allowed range");
      CHECKD(pos.x >= (1 << maxLog2SPSize.width), "x-pos outside allowed range");
      return templateBuf.bufAt(0, (pos.y << maxLog2SPSize.width) + pos.x);
    }

  private:
    const AreaBuf<const int8_t> templateBuf;
    const Size                  maxLog2SPSize;
  };
  TrBrdTemplate                                   xGetSignPredTemplate(const TransformUnit &tu, const CompID compID);
  static_vector<uint32_t, 1 << SIGN_PRED_MAX_NUM> xCalcSignPredCosts(TransformUnit &tu, const CompID compID,
                                                                     const std::vector<Position> &sprdPos,
                                                                     const std::vector<Pel>      *fwdLUT);
  int                                             xGetUniTransformIdx(const TransformUnit &tu, CompID compID);

#ifdef TARGET_SIMD_X86
  template<X86_VEXT vext> void _initTrQuantX86();
  void                         initTrQuantX86();
#endif
};// END CLASS DEFINITION TrQuant

namespace TrEst
{
struct Cost
{
  Cost(int i, MtsType t, double c) : id(i), tr(t), cost(c) {}
  int     id;
  MtsType tr;
  double  cost;
};
struct CostCmp
{
  bool operator()(const Cost &a, const Cost &b) { return a.cost < b.cost; }
};

std::vector<Cost> &sortCL(std::vector<Cost> &cl);
std::vector<Cost> &addCL(std::vector<Cost> &cl, const std::vector<Cost> &add);

struct ZeroOut
{
  ZeroOut(const Size &, const MtsType);
  uint32_t nzW;
  uint32_t nzH;
  uint32_t num;
};

struct DSums
{
  DSums(const double _abs, const double _sqr) : abs(_abs), sqr(_sqr) {}
  double abs;
  double sqr;
};

class PreCostBase
{
protected:
  PreCostBase(const CompID, TransformUnit &, const double l1Lambda, const double l1Scale, const double skipScale);
  double calcOrthoScale();
  bool   includesZeroOut(const TransList &);
  void   initResidual(const TransList &, const Pel *);
  void   initResidual(const TransList &, const Pel *, const Pel *);
  double getEstTrCost(const MtsType, const TransBuffer &);
  double getEstTrCost(const MtsType, const TransBuffer &, const TransBuffer &);

public:
  int blkSizeId() const { return bsizeId; }

private:
  template<typename T> double sumResAbs(const T *);
  template<typename T> double sumResSqr(const T *);
  template<typename T> double sumTrnAbs(const T *);
  template<typename T> DSums  sumTrnZO(const T *, const ZeroOut &);

protected:
  const CompID   compId;
  TransformUnit &tu;
  CUCtx          cuCtx;
  const Size     sz;
  const SizeType num;
  const double   orthoScaleAbs;
  const double   orthoScaleSqr;
  const double   invL1Lambda;
  const double   distScaleAbs;
  const double   distScaleSqr;
  const double   skipFactor;
  const int      bsizeId;
  double         sumSqrRes = 0.0;
};
}   // namespace TrEst

//! \}

#endif // __TRQUANT__
