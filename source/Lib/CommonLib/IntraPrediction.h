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

/** \file     IntraPrediction.h
    \brief    prediction class (header)
*/

#ifndef __INTRAPREDICTION__
#define __INTRAPREDICTION__

// Include files
#include "Common.h"
#include "CommonDef.h"
#include "Unit.h"
#include "Buffer.h"
#include "Picture.h"

#include "MatrixIntraPrediction.h"
#include "RdCost.h"
#include "CommonLib/InterpolationFilter.h"

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// prediction class
enum PredBuf
{
  PRED_BUF_UNFILTERED = 0,
  PRED_BUF_FILTERED   = 1,
  NUM_PRED_BUF        = 2
};

static constexpr uint32_t MAX_INTRA_FILTER_DEPTHS = 8;

struct LeastSquaresSolver
{
  LeastSquaresSolver() {}
  ~LeastSquaresSolver() {}

  void solve(const class IntraPrediction &intraPrediction, const int sampleNum, const Pel *inputMat,
             const int inputStride, const int extraRegularizationParam, const Pel *targetVec0, const int offset0,
             ConvModel *model0, const Pel *targetVec1 = nullptr, const int offset1 = 0, ConvModel *model1 = nullptr);

private:
  void gaussBacksubstitution(const TCccmCoeff C[SOLVER_NUM_PARAMS_MAX][SOLVER_NUM_PARAMS_MAX + 2], TCccmCoeff *x,
                             int numParams, int col);
  void gaussElimination(TCccmCoeff C[SOLVER_NUM_PARAMS_MAX][SOLVER_NUM_PARAMS_MAX + 2], TCccmCoeff *x0, TCccmCoeff *x1,
                        int numParams, int numModels);
};

class IntraPrediction
{
  friend struct LeastSquaresSolver;

protected:
  Pel      m_refBuffer[MAX_NUM_COMP][NUM_PRED_BUF][(MAX_CU_SIZE * 2 + 1 + MAX_REF_LINE_IDX) * 2];
  uint32_t m_refBufferStride[MAX_NUM_COMP];

  static const int angTable[32];
  static const int invAngTable[32];

protected:
  InterpolationFilter *m_pIf;
  static const int     extAngTable[64];
  static const int     extInvAngTable[64];

private:
  Pel *m_yuvExt2[MAX_NUM_COMP][4];
  int  m_yuvExtSize2;

  LeastSquaresSolver m_cccmSolver;
  RdCost             m_rdCost;

  struct CccmAreaInfo
  {
    Area blkArea;
    Area refArea;
    int  refSizeX  = 0; // Reference lines available left
    int  refSizeY  = 0; // Reference lines available above
    int  refStride = 0; // Stride of the reference area
    int  refOrigin = 0; // Offset to the top-left corner of the reference area
    int  blkOrigin = 0; // Offset to the top-left corner of the prediction area

    CccmAreaInfo() {}
    CccmAreaInfo(Area bArea, int rSizeX, int rSizeY, int rWidth, int rHeight)
    {
      refArea   = Area(bArea.x - rSizeX, bArea.y - rSizeY, rWidth, rHeight);
      blkArea   = bArea;
      refSizeX  = rSizeX;
      refSizeY  = rSizeY;
      refStride = rWidth + 2 * CCCM_FILTER_PADDING; // Including paddings required for the 2D filter
      refOrigin = refStride * CCCM_FILTER_PADDING + CCCM_FILTER_PADDING;
      blkOrigin = refStride * (refSizeY + CCCM_FILTER_PADDING) + refSizeX + CCCM_FILTER_PADDING;
    }

    PelBuf getPredPelBufFor(Pel *lumaBuf) const
    {
      return PelBuf(lumaBuf + blkOrigin, refStride, blkArea.width, blkArea.height);
    }
    PelBuf getRefPelBufFor(Pel *lumaBuf) const
    {
      return PelBuf(lumaBuf + refOrigin, refStride, refArea.width, refArea.height);
    }

    void getRefParams(int &rAreaWidth, int &rAreaHeight, int &rSizeX, int &rSizeY, int &rPosPicX, int &rPosPicY) const
    {
      rAreaWidth  = refArea.width;
      rAreaHeight = refArea.height;
      rSizeX      = refSizeX;
      rSizeY      = refSizeY;
      rPosPicX    = refArea.x;
      rPosPicY    = refArea.y;
    }
  };

  CccmAreaInfo m_cccmAreaInfo;
  CccmAreaInfo m_cccmAreaInfoNoSubs; // Used for the non-subsampled cases

  int  m_cccmLumaOffset;
  Pel *m_cccmLumaBuf[CCCM_NUM_LUMA_BUFS];

  Pel *m_refMatrixA;
  Pel *m_targetVectorCb;
  Pel *m_targetVectorCr;

  Pel *m_bvgCccmLumaBuf[NUM_BVG_CCCM_CANDS];
  Pel *m_bvgCccmChromaBuf[NUM_BVG_CCCM_CANDS][2];
  Area m_bvgCccmBlkArea;
  Area m_bvgCccmRefArea;

  static const int strideRefMatrixA = CCCM_MAX_REF_SAMPLES;

  Pel *m_ccTemplPredCb[MAX_CCP_CAND_LIST_SIZE];
  Pel *m_ccTemplPredCr[MAX_CCP_CAND_LIST_SIZE];

  static const uint8_t aucIntraFilter[MAX_INTRA_FILTER_DEPTHS];

  static const uint8_t aucIntraFilterExt[MAX_INTRA_FILTER_DEPTHS];

  unsigned m_auShiftLM[32]; // Table for substituting division operation by multiplication

  struct IntraPredParam // parameters of Intra Prediction
  {
    bool refFilterFlag { false };
    bool applyPDPC { false };
    bool useGradPDPC { false };
    bool isModeVer { false };
    int  multiRefIndex { -1 };
    int  intraPredAngle { std::numeric_limits<int>::max() };
    int  absInvAngle { std::numeric_limits<int>::max() };
    bool interpolationFlag { false };
    int  angularScale { -1 };
  };

  IntraPredParam m_ipaParam;

  Pel                  *m_piTemp;
  MatrixIntraPrediction m_matrixIntraPred;
  RdCost               *m_timdSatdCost = nullptr;

  Area m_eipBlkArea;
  int  m_eipLumaOffset;
  Pel *m_eipRefMatrixA[NUM_EIP_MODELS] {};
  Pel  m_eipBuffer[(MAX_EIP_SIZE * 2 + MAX_EIP_REF_SIZE) * (MAX_EIP_SIZE * 2 + MAX_EIP_REF_SIZE)];
  Pel *m_eipTargetVectorY[NUM_EIP_MODELS] {};
  Pel  m_eipPredTpl[2][MAX_EIP_SIZE * EIP_TPL_SIZE];
  Pel  m_eipBias;
  Pel  m_eipAvg;
  int  m_eipClipMin;
  int  m_eipClipMax;

  static const int strideEipRefMatrixA = (5 * MAX_EIP_SIZE * MAX_EIP_SIZE);

protected:
  ChromaFormat m_currChromaFormat;

  int                     m_topRefLength;
  int                     m_leftRefLength;
  ScanElement            *m_scanOrder;
  bool                    m_bestScanRotationMode;
  PelStorage              m_sgpmBuffer;
  std::vector<PelStorage> m_tempBufferDIMD;

  Pel  m_ref[MAX_PDP_SIZE * 6]      = {};
  Pel  m_refShort[MAX_PDP_SIZE * 6] = {};
  bool m_refAvailable               = {};

  // prediction
  void xPredIntraDc(const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType,
                    const bool enableBoundaryFilter = true);

  void xPredIntraAng(const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const ClpRng &clpRng,
                     const bool bExtIntraDir = false);

  void (*m_IntraPredAngleCpy)(const Pel *refMain, Pel *pDsty, const int width, const int height,
                              const ptrdiff_t dstStride, const int intraPredAngle, const int multiRefIdx,
                              const bool isExt);
  void (*m_IntraPredAngleChroma)(const Pel *refMain, Pel *pDsty, const int width, const int height,
                                 const ptrdiff_t dstStride, const int intraPredAngle, const int multiRefIdx);

  void (*m_IntraPredAngleLuma)(const Pel *refMain, Pel *pDsty, const int width, const int height,
                               const ptrdiff_t dstStride, const int intraPredAngle, const int multiRefIdx,
                               const ClpRng &clpRng, const bool interpolationFlag, const bool useExt);
  void (*m_IntraAnglePDPC)(Pel *pDsty, const ptrdiff_t dstStride, const Pel *refSide, const int width, const int height,
                           const int scale, const int absInvAngle);

  void (*m_IntraAngleGradPDPC)(Pel *pDsty, const ptrdiff_t dstStride, const Pel *refSide, const Pel *refMain,
                               const int width, const int height, int scale, const int intraPredAngle,
                               const int multiRefIdx, const ClpRng &clpRng, const bool isExt);
  void (*m_IntraHorVerPDPC)(Pel *pDsty, const ptrdiff_t dstStride, Pel *refSide, const int width, const int height,
                            int scale, const Pel *refMain, const ClpRng &clpRng);
  void (*m_PredIntraPlanar)(const CPelBuf &pSrc, PelBuf &pDst, const PlanarDirType &plDir);
  void (*m_IntraPredSampleFilter)(const CPelBuf &srcBuf, PelBuf &dstBuf);

public:
  void (*m_calculateCorrelationMatrix)(int nRows, int nCols, int nSamples, const int16_t *input[], int64_t *output,
                                       int outputStride);

protected:
  bool (*m_xPredIntraOpt)(PelBuf &pDst, const CodingUnit &cu, const uint32_t modeIdx, const ClpRng &clpRng, Pel *refF,
                          Pel *refS);

  static int buildHistogram(const Pel *pReco, int iStride, uint32_t uiHeight, uint32_t uiWidth, int *piHistogram,
                            bool withoutBottomRight = false, bool useSmallFilter = false);
  void locDepBlending(Pel *pDst, ptrdiff_t strideDst, Pel *pVer, ptrdiff_t strideVer, Pel *pHor, ptrdiff_t strideHor,
                      Pel *pNonLocDep, ptrdiff_t strideNonLocDep, int width, int height, int mode, int wVer, int wHor,
                      int wNonLocDep, int range = 10);
  void predTimdIntraAng(const CompID compId, const CodingUnit &cu, uint32_t uiDirMode, Pel *pPred, ptrdiff_t uiStride,
                        uint32_t iWidth, uint32_t iHeight, TemplateType eTempType, int32_t iTemplateWidth,
                        int32_t iTemplateHeight);
  void initPredTimdIntraParams(const CodingUnit &cu, const CompArea area, int dirMode, bool bSgpm);
  void initTimdIntraPatternLuma(const CodingUnit &cu, const CompArea &area, int iTemplateWidth, int iTemplateHeight,
                                uint32_t uiRefWidth, uint32_t uiRefHeight);
  void initPredIntraParams(const CodingUnit &cu, const CompArea compArea, const SPS &sps);

  static bool isIntegerSlope(const int absAng) { return (0 == (absAng & 0x1F)); }
  static bool isIntegerSlopeExt(const int absAng) { return (0 == (absAng & 0x3F)); }

  void xPredIntraBDPCM(const CPelBuf &pSrc, PelBuf &pDst, BdpcmMode dirMode, const ClpRng &clpRng);
  Pel  xGetPredValDc(const CPelBuf &pSrc, const Size &dstSize);

  void xFillReferenceSamples(const CPelBuf &recoBuf, Pel *refBufUnfiltered, const CompArea &area, const CodingUnit &cu);
  void xFilterReferenceSamples(const Pel *refBufUnfiltered, Pel *refBufFiltered, const CompArea &area, const SPS &sps,
                               int multiRefIdx);

  static int getModifiedWideAngle(int width, int height, int predMode);
  void       setReferenceArrayLengths(const CompArea &area);

  static int getWideAngleExt(int width, int height, int predMode, bool bSgpm);
  void       destroy();

  void xPadMdlmTemplateSample(Pel *pSrc, Pel *pCur, int cWidth, int cHeight, int existSampNum, int targetSampNum);
  void xGetLMParameters_LMS(const CodingUnit &pu, const CompID compID, const CompArea &chromaArea,
                            CclmModel &cclmModel);
  struct MMLM_parameter
  {
    int a;
    int b;
    int shift;
  };
  void xUpdateCclmModel(CclmModelSingle &model, int delta);
  int  xCalcLMParametersGeneralized(int x, int y, int xx, int xy, int count, int bitDepth, int &a, int &b, int &iShift);
  int  xLMSampleClassifiedTraining(int count, int mean, int meanC, int LumaSamples[], int ChrmSamples[], int bitDepth,
                                   MMLM_parameter parameters[]);

  template<ConvModelType modelType, int modelId = 0>
  inline void xCccmCalcModelsTemplate(CodingUnit &cu, ConvModel &cccmModelCb, ConvModel &cccmModelCr, int modelIdD,
                                      int modelThr, int refSizeLimit);
  void        xCccmCalcModels(CodingUnit &cu, ConvModel &cccmModelCb, ConvModel &cccmModelCr, int modelId, int modelThr,
                              int refSizeLimit = 0);

  template<ConvModelType modelType, bool dualModel = false>
  void xCccmApplyModelsTemplate(const CodingUnit &cu, const CrossCompModels &ccModels, PelBuf &predCb, PelBuf &predCr,
                                const bool isTempl = false, const bool isTopTempl = false) const;
  void xCccmApplyModels(const CodingUnit &cu, const CrossCompModels &ccModels, PelBuf &predCb, PelBuf &predCr,
                        const bool isTempl = false, const bool isTopTempl = false) const;

  void xCccmSetLumaRefValue(const CodingUnit &cu);
  void xCccmCreateLumaRef(const CodingUnit &cu, const int filterIndex);
  void xCccmCreateLumaRefNoSubs(const CodingUnit &cu, const int filterIndex);
  Pel  xCccmGetLumaVal(const CodingUnit &cu, const CPelBuf pi, const int x, const int y, const int filterIndex) const;
  int  xCccmCalcRefAver(const CodingUnit &cu) const;
  void xCccmCalcRefArea(const CodingUnit &cu);

  Position getEipRecoPosition(const CodingUnit &cu, const CompID compId) const;
  void     setEipInputVector(PelBuf &reco, int w, int h, int filterShape, Pel *inputs);
  Pel      getEipInputsAvg(Pel *inputs, ConvModelType filterShape) const;

public:
  IntraPrediction();
  virtual ~IntraPrediction();

  void init(ChromaFormat chromaFormatIdc, const unsigned bitDepthY, InterpolationFilter *pIf);

  void   xBvgCccmCalcRefArea(const CodingUnit &cu, CompArea chromaArea);
  PelBuf xBvgCccmGetLumaCuBuf(int candIdx = 0) const;
  PelBuf xBvgCccmGetLumaCuBufFul(int candIdx = 0) const;
  PelBuf xBvgCccmGetChromaCuBuf(const CompID compId, int candIdx = 0) const;
  void   xBvgCccmCreateLumaRef(const CodingUnit &cu);
  int    xBvgCccmCalcBlkAver(const CodingUnit &cu) const;
  void   xBvgCccmCalcBlkRange(const CodingUnit &cu, int &minVal, int &maxVal) const;
  void   xBvgCccmCalcModels(const CodingUnit &cu, ConvModel &cccmModelCb, ConvModel &cccmModelCr, int modelId,
                            int modelThr, int minVal, int maxVal);

  // Angular Intra
  bool predIntraAng(const CompID compId, PelBuf &piPred, const CodingUnit &cu, const bool applyPDPFilter,
                    const bool useExt);
  Pel *getPredictorPtr(const CompID compId)
  {
    return m_refBuffer[compId][m_ipaParam.refFilterFlag ? PRED_BUF_FILTERED : PRED_BUF_UNFILTERED];
  }

  // Cross-component Chroma
  void predIntraChromaLM(const CompID compID, PelBuf &piPred, CodingUnit &cu, const CompArea &chromaArea, int intraDir,
                         bool createModel = true);
  void xGetLumaRecPixels(const CodingUnit &cu, CompArea chromaArea, bool createAllRefs = false);

  void predIntraCCCM(CodingUnit &cu, PelBuf &predCb, PelBuf &predCr, const bool createModels = true);
  void cccmCreateLumaRefs(const CodingUnit &cu, const bool createAll = false,
                          const ConvModelType cccmType = CONV_MODEL_UNDEFINED);
  void cccmCreateLumaRefsForMergeList(const CodingUnit &cu, const CrossCompModels *mergeList, const int mergeListSize);

  void   reorderCCPCandidates(CodingUnit &cu, CrossCompModels candList[], int reorderlistSize, int *fusionList);
  void   xDecDerivedCcpFusionTemplFilter(Pel *predBuf, int numSamples);
  int    xGetOneCCPCandCost(CodingUnit &cu, CrossCompModels &ccpCand, const int candIdx = 0, const bool filter = false);
  void   setAndPredCcMergeCand(CodingUnit &cu, CrossCompModels &ccpCand, PelBuf &predCb, PelBuf &predCr);
  void   setAndPredCcMergeFusionCand(CodingUnit &cu, CrossCompModels model0, CrossCompModels model1, PelBuf &predCb,
                                     PelBuf &predCr);
  int    xGetCostCCPFusion(const CodingUnit &cu, const CompID compID, const CompArea &chromaArea, int candIdx0,
                           int candIdx1);
  void   findDecoderDerivedCcpModel(CodingUnit &cu, CrossCompModels &ccModel0, CrossCompModels &ccModel1);
  int    xDecDerivedFusionTemplateCost(const CodingUnit &cu, int candIdxOrig0, int candIdxOrig1, int cost0, int cost1);
  int    xTemplateCostCCLM(const CodingUnit &cu, const CompID compID, const CompArea &chromaArea,
                           const CclmModel &cclmModel, const int modelNum, const int candIdx);
  PelBuf xGetCcTemplPredBuf(const CompID compID, const int candIdx, const bool top, const Area &area);
  void   filterPredInside(const CompID compID, PelBuf &piPred, const CodingUnit &cu);

  /// set parameters from CU data for accessing intra data
  void initIntraPatternChType(const CodingUnit &cu, const CompArea &area, const bool forceRefFilterFlag = false,
                              int partIdx = 0);   // use forceRefFilterFlag to get both filtered and unfiltered buffers

  // Matrix-based intra prediction
  void initIntraMip(const CodingUnit &cu, const CompArea &area);
  void predIntraMip(const CompID compId, PelBuf &piPred, const CodingUnit &cu);

  static std::pair<int8_t, int8_t> deriveIpmForTransform(CPelBuf predBuf, CodingUnit &cu);
  static int8_t                    deriveIpmForChromaTransform(CPelBuf predCb, CPelBuf predCr, CodingUnit &cu);

  // DIMD prediction
  void predIntraDimd(PelBuf &piPred, CodingUnit &cu, const CompArea &area);
  void deriveDimdMode(DimdData &dimdData, const CPelBuf &recoBuf, const CompArea &area, const CodingUnit &cu);
  void computeHistogramForAllComponents(const CPelBuf &recoBufY, const CPelBuf &recoBufCb, const CPelBuf &recoBufCr,
                                        const CompArea &areaY, const CompArea &areaCb, const CompArea &areaCr,
                                        CodingUnit &cu, int *piHistogram);

  void predIntraDimdChroma(PelBuf &piPred, CodingUnit &cu, const CompID compId, const CompArea &area);
  void deriveDimdChromaMode(DimdData &dimdData, CodingUnit &cu);

  void findBestModesInHistogram(int *histogram, int &bestMode, const CodingUnit &cu);
  void pruneDimdChromaModes(int &bestMode, int &secondBestMode, const CodingUnit &cu);

  enum class TimdMode
  {
    Normal,
    SAD,
    Merge
  };
  void predIntraTimd(PelBuf &piPred, CodingUnit &cu, const CompArea &area, bool skipDerivation, const TimdMode timdMode,
                     const bool alreadyExecutedForNormalMode);
  bool deriveTimdMergeMode(CodingUnit &cu, const CompArea &area);

  enum class TimdDerivationMethod
  {
    Full,
    HorizontalVertical,
    FullWithSAD
  };
  int deriveTimdMode(const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu,
                     const TimdDerivationMethod timdDerivationMode);

  using TimdModeCostList = static_vector<std::pair<uint64_t, int>, NUM_LUMA_MODE>;
  TimdModeCostList m_timdModeCostList;

  // OBIC prediction
  void predIntraObic(PelBuf &piPred, CodingUnit &cu, const CompArea &area, bool skipDerivation);
  void deriveObicMode(const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu);
  // SGPM prediction
  void predIntraSGPM(PelBuf &piPred, CodingUnit &cu, const CompArea &area, bool skipDerivation);

  // EIP prediction
  void initIntraEip(CodingUnit &cu, const CompArea &area);
  int  getCurEipCands(const CodingUnit &cu, static_vector<EipModels, NUM_DERIVED_EIP> &candList, const CompID compId);
  void getNeighborsEipCands(const CodingUnit &cu, static_vector<EipModels, MAX_MERGE_EIP> &candList,
                            const CompID compId);
  void reorderEipCands(const CodingUnit &cu, static_vector<EipModels, MAX_MERGE_EIP> &candList,
                       const CompID compId = CompID::COMP_Y);
  void eipPred(const CodingUnit &cu, PelBuf &pred, const CompID compId);

  void geneWeightedPred(PelBuf &pred, const CodingUnit &cu, const Pel *srcBuf, const CompID compID);
  Pel *getPredictorPtr2(const CompID compID, uint32_t idx) { return m_yuvExt2[compID][idx]; }
  void switchBuffer(const CodingUnit &cu, CompID compID, PelBuf srcBuff, Pel *dst);
  void geneIntrainterPred(const CodingUnit &cu);
  void reorderPLT(CodingStructure &cs, Partitioner &partitioner, CompID compBegin, uint32_t numComp);

  void deriveSgpmModeOrdered(const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu,
                             static_vector<SgpmInfo, SGPM_NUM> &candModeList,
                             static_vector<double, SGPM_NUM> &candCostList, const DimdData &dimd);

#ifdef TARGET_SIMD_X86
  void                         initIntraPredictionX86();
  template<X86_VEXT vext> void _initIntraPredictionX86();
#endif

  void xFillTimdReferenceSamples(const CPelBuf &recoBuf, Pel *refBufUnfiltered, const CompArea &area,
                                 const CodingUnit &cu, int iTemplateWidth, int iTemplateHeight);
  Pel  xGetPredTimdValDc(const CPelBuf &pSrc, const Size &dstSize, TemplateType eTempType, int iTempHeight,
                         int iTempWidth);
  void xIntraPredTimdHorVerPdpc(Pel *pDsty, const ptrdiff_t dstStride, Pel *refSide, const int width, const int height,
                                int xOffset, int yOffset, int scale, const Pel *refMain, const ClpRng &clpRng);
  void xIntraPredTimdPlanarDcPdpc(const CPelBuf &pSrc, Pel *pDst, ptrdiff_t iDstStride, int width, int height,
                                  TemplateType eTempType, int iTemplateWidth, int iTemplateHeight);
  void xPredTimdIntraPlanar(const CPelBuf &pSrc, Pel *pDst, ptrdiff_t iDstStride, int width, int height,
                            TemplateType eTempType, int iTemplateWidth, int iTemplateHeight);
  void xPredTimdIntraDc(const CodingUnit &cu, const CPelBuf &pSrc, Pel *pDst, ptrdiff_t iDstStride, int iWidth,
                        int iHeight, TemplateType eTempType, int iTemplateWidth, int iTemplateHeight);
  void xIntraPredTimdAngPdpc(Pel *pDsty, const ptrdiff_t dstStride, Pel *refSide, const int width, const int height,
                             int xOffset, int yOffset, int scale, int invAngle);
  void xIntraPredTimdAngGradPdpc(Pel *pDsty, const ptrdiff_t dstStride, Pel *refMain, Pel *refSide, const int width,
                                 const int height, int xOffset, int yOffset, int scale, int deltaPos,
                                 int intraPredAngle, const ClpRng &clpRng);
  void xIntraPredTimdAngLuma(Pel *pDstBuf, const ptrdiff_t dstStride, Pel *refMain, int width, int height, int deltaPos,
                             int intraPredAngle, const ClpRng &clpRng, int xOffset, int yOffset);
  void xPredTimdIntraAng(const CPelBuf &pSrc, const ClpRng &clpRng, Pel *pTrueDst, ptrdiff_t iDstStride, int iWidth,
                         int iHeight, TemplateType eTempType, int iTemplateWidth, int iTemplateHeight,
                         uint32_t dirMode);

  Pel m_a[CCCM_NUM_PARAMS_MAX][CCCM_REF_SAMPLES_MAX];
  Pel m_cb[CCCM_REF_SAMPLES_MAX];
  Pel m_cr[CCCM_REF_SAMPLES_MAX];
};

//! \}

// temporary:
static const int CCCM_DECIM_BITS_HBD = 22;

#define DECIM_BITS(x) ((x) > 10 ? CCCM_DECIM_BITS_HBD : CCCM_DECIM_BITS)

struct CccmModel
{
  CccmModel(int num, int bitdepth)
  {
    bd     = bitdepth;
    midVal = (1 << (bitdepth - 1));
    params.resize(num);
    decimBits  = DECIM_BITS(bd);
    decimRound = (1 << (decimBits - 1));
  }

  ~CccmModel() {}

  std::vector<TCccmCoeff> params;
  int                     bd;
  int                     midVal;
  int                     decimRound;
  int                     decimBits;

  const int getNumParams() const { return (int)params.size(); }

  void clearModel()
  {
    const int numParams = (int)params.size();

    std::fill(params.begin(), params.end(), 0);

    params[numParams - 1] = (TCccmCoeff)1 << decimBits;   // Default bias to 1
  }

  Pel convolve(Pel *vector)
  {
    TCccmCoeff sum = 0;

    for (int i = 0; i < params.size(); i++)
    {
      sum += params[i] * vector[i];
    }

    return Pel((sum + decimRound) >> decimBits);
  }

  Pel nonlinear(const Pel val) const { return (val * val + midVal) >> bd; }
  Pel bias() const { return midVal; }
  CccmModel() {}
  bool valid()
  {
    return std::none_of(params.begin(), params.end(), [](TCccmCoeff x) { return abs(x) > (4 << CCCM_DECIM_BITS); });
  }
};

struct CccmCovariance
{
  TCccmCoeff ATA[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX];
  TCccmCoeff ATCb[CCCM_NUM_PARAMS_MAX];
  TCccmCoeff ATCr[CCCM_NUM_PARAMS_MAX];
  TCccmCoeff C[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX + 2];

  void solve3(TCccmCoeff ATA[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff ATCb[CCCM_NUM_PARAMS_MAX],
              TCccmCoeff ATCr[CCCM_NUM_PARAMS_MAX], const int sampleNum, const int chromaOffsetCb,
              const int chromaOffsetCr, CccmModel &modelCb, CccmModel &modelCr, const bool interCccmMode);
  void gaussBacksubstitution(TCccmCoeff *x, int numEq, int col, int round, int bits);
  void gaussElimination(TCccmCoeff A[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff *y0, TCccmCoeff *x0,
                        TCccmCoeff *y1, TCccmCoeff *x1, int numEq, int numFilters, int bd,
                        const bool interCccmMode = false);
};

#endif   // __INTRAPREDICTION__
