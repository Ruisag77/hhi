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

/** \file     AdaptiveLoopFilterEcm.h
    \brief    ECM adaptive loop filter class (header)
*/

#ifndef __ADAPTIVELOOPFILTERECM__
#define __ADAPTIVELOOPFILTERECM__

#include "CommonDef.h"

#include "AdaptiveLoopFilter.h"
#include "AlfParametersEcm.h"
#include "Unit.h"
#include "UnitTools.h"

class AdaptiveLoopFilterEcm : virtual public AdaptiveLoopFilter, virtual public AlfParametersEcm
{
public:
  // Constants

  static constexpr int CLASSIFICATION_BLK_SIZE     = 256;   // non-normative, local buffer size
  static constexpr int FIX_FILTER_COEFF_SCALE_BITS = 11;    // 12-bit signed values
  static constexpr int CTB_MODE_LUMA_APS0_ALT0     = LumaCtbModeHandler::createApsFilterMode(0, 0);

  // Sub-types

  // TODO: ALF: Maybe switch back to some more explanatory type. BS 2023-09-08
  typedef short AlfClassifier;

  enum Direction : unsigned
  {
    HOR,
    VER,
    DIAG0,
    DIAG1,
    VARIANCE,
    NUM_DIRECTIONS,
    FIRST = HOR,
    LAST  = VARIANCE,
  };

  /// ALF classifier index.
  enum class ClassifierIndex : unsigned
  {
    ADAPT_FILTER_GRAD_BASED      = 0,
    FIXED_FILTER_4x4             = 0,
    FIXED_FILTER_12x12           = 3,
    FIXED_FILTER_CHROMA          = 4,
    ADAPT_FILTER_BAND_BASED_RECO = 1,
    ADAPT_FILTER_BAND_BASED_RESI = 2,
    FIRST                        = ADAPT_FILTER_GRAD_BASED,
    LAST                         = FIXED_FILTER_CHROMA,
  };
  static_assert(static_cast<int>(ClassifierIndex::LAST) + 1 >= ALF_NUM_CLASSIFIER,
                "Mismatch in number of classifiers.");

  /// ALF classification window size.
  enum class ClsWndwSize : unsigned
  {
    WNDW_4x4,
    WNDW_12x12,
    WNDW_CHROMA,
    LAST = WNDW_CHROMA
  };
  static constexpr unsigned NUM_CLS_WNDW_SIZES = static_cast<unsigned>(ClsWndwSize::LAST) + 1;

  /// ALF fixed filter index.
  enum class FixFiltIdx : unsigned
  {
    FIRST,
    SECOND,
    LAST = SECOND
  };
  static_assert((static_cast<int>(FixFiltIdx::LAST) + 1) == NUM_FIXED_FILTERS, "Mismatch in number of fixed filters.");

  /// ALF fixed filter candidate.
  enum class FixFiltSetCand : unsigned
  {
    FIRST,
    SECOND,
    LAST = SECOND
  };
  static_assert((static_cast<int>(FixFiltSetCand::LAST) + 1) == NUM_FIXED_FILTER_SET_CANDS,
                "Mismatch in number of fixed filter set candidates.");

  /// QP-based fixed filter set candidates.
  class FixedFilterSetCands
  {
  public:
    constexpr FixedFilterSetCands(const int qp);

    constexpr const unsigned &operator[](const FixFiltSetCand &fixFiltSetCand) const;
    constexpr unsigned       &operator[](const FixFiltSetCand &fixFiltSetCand);

    /// Get the index of the selected fixed filter set.
    static inline unsigned getFixedFilterSet(const int qp, const FixFiltSetCand fixedFilterSetCandIdx);

  private:
    template<typename T> static constexpr const T &min(const T &a, const T &b) { return (a <= b) ? a : b; }

    unsigned m_cands[NUM_FIXED_FILTER_SET_CANDS];
  };

  template<typename T, unsigned size> class FilterRowPtrs : public AdaptiveLoopFilter::FilterRowPtrsBase<T, size>
  {
  public:
    constexpr FilterRowPtrs(T *const center, const ptrdiff_t stride);

    constexpr FilterRowPtrs<T, size> &set(T *const center, const ptrdiff_t stride);
  };

  /// Image component buffer class allowing easier row access.
  using CompBuf = LineAccessAreaStorage<Pel>;

  /// Type for ALF fixed filter output buffers for a single filter set and both filters.
  using FixFiltBuf = EnumBasedBufArray<CompBuf, FixFiltIdx, FixFiltIdx::FIRST, FixFiltIdx::LAST>;

  /// Helper type for fixed filter output buffers for both fixed filter set candidates.
  template<typename ElemT> using FixFiltCandBufHlp =
    EnumBasedBufArray<ElemT, FixFiltSetCand, FixFiltSetCand::FIRST, FixFiltSetCand::LAST>;

  /// Type for fixed filter output buffers for both fixed filter set candidates and both filters.
  using FixFiltCandBuf = FixFiltCandBufHlp<FixFiltBuf>;

  /// Type for residual-based fixed filter output buffers for both filters.
  using FixFiltResiBuf = FixFiltCandBufHlp<CompBuf>;

  /// Buffer for laplacians.
  using LaplacianBuf = AdaptiveLoopFilter::LaplacianBuf<uint32_t, ((CLASSIFICATION_BLK_SIZE + 16) >> 1) + 8,
                                                        ((CLASSIFICATION_BLK_SIZE + 10) >> 1), Direction>;

    /// Buffer for classification output.
  using ClassBuf = AdaptiveLoopFilter::ClassBuf<AlfClassifier>;

  /// Buffer for multiple classification outputs.
  using MultiClassBuf = EnumBasedBufArray<ClassBuf, ClassifierIndex, ClassifierIndex::FIRST, ClassifierIndex::LAST>;

  // Setup

  AdaptiveLoopFilterEcm();

  void         create(const int picWidth, const int picHeight, const ChromaFormat format, const int maxCUWidth,
                      const int maxCUHeight, const int maxCUDepth, const BitDepths &inputBitDepth);
  virtual void destroy() override;

  // Processing

  void ALFProcess(CodingStructure &cs);

  /// Copy the additional input data before the DBF is applied.
  void copyAddInputsBeforeDBF(const CodingStructure &cs, TrQuant &trQuant);

  // CCALF data access

  CcAlfFilterParam &getCcAlfFilterParam() { return m_ccAlfFilterParam; }

protected:
  // Non-frame-specific ALF parameters

  EnumArray<std::vector<AlfFilterShape>, ChannelType>   m_filterShapes;
  EnumArray<bool[ALF_NUM_OF_FILTER_TYPES], ChannelType> m_filterTypeTest;
  EnumArray<int[ALF_NUM_OF_FILTER_TYPES], ChannelType>  m_filterTypeToStatIndex;

  // Frame-specific ALF parameters

  int           m_numLumaAltAps[ALF_CTB_MAX_NUM_APS];
  int8_t        m_classifierIdxApsLuma[ALF_CTB_MAX_NUM_APS][ALF_MAX_NUM_ALTERNATIVES_LUMA];
  int8_t        m_classifierFinal[ALF_MAX_NUM_ALTERNATIVES_LUMA];
  AlfFilterType m_filterTypeApsLuma[ALF_CTB_MAX_NUM_APS];
  AlfFilterType m_filterTypeApsChroma;

  // Filter coefficients buffers

  template<typename T> using LumaClassParamSingleAps = T[ALF_MAX_NUM_ALTERNATIVES_LUMA][MAX_NUM_ALF_CLASSES];
  template<typename T> using LumaDataSingleAps =
    T[ALF_MAX_NUM_ALTERNATIVES_LUMA][MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES];
  template<typename T> using LumaClassParamMultiAps = LumaClassParamSingleAps<T>[ALF_CTB_MAX_NUM_APS];
  template<typename T> using LumaDataMultiAps       = LumaDataSingleAps<T>[ALF_CTB_MAX_NUM_APS];
  template<typename T> using ChromaClassParam       = T[ALF_MAX_NUM_ALTERNATIVES_CHROMA][1];
  template<typename T> using ChromaData             = T[ALF_MAX_NUM_ALTERNATIVES_CHROMA][MAX_NUM_ALF_CHROMA_COEFF];

  using LumaScaleIdxSingleAps = LumaClassParamSingleAps<int8_t>;
  using LumaCoeffSingleAps    = LumaDataSingleAps<coeff_t>;
  using LumaClipSingleAps     = LumaDataSingleAps<clip_t>;
  using LumaScaleIdxMultiAps  = LumaClassParamMultiAps<int8_t>;
  using LumaCoeffMultiAps     = LumaDataMultiAps<coeff_t>;
  using LumaClipMultiAps      = LumaDataMultiAps<clip_t>;
  using ChromaScaleIdx        = ChromaClassParam<int8_t>;
  using ChromaCoeff           = ChromaData<coeff_t>;
  using ChromaClip            = ChromaData<clip_t>;

  LumaScaleIdxMultiAps  m_scaleIdxApsLuma;
  LumaCoeffMultiAps     m_coeffApsLuma;
  LumaClipMultiAps      m_clippApsLuma;
  LumaScaleIdxSingleAps m_scaleIdxFinal;
  LumaCoeffSingleAps    m_coeffFinal;
  LumaClipSingleAps     m_clippFinal;
  ChromaScaleIdx        m_chromaScaleIdxFinal;
  ChromaCoeff           m_chromaCoeffFinal;
  ChromaClip            m_chromaClippFinal;

  // Input buffers

  // Buffers for DBF input.
  PelStorage m_tempBufBeforeDb;      ///< Frame-level buffer for the reconstruction before DBF (all components).
  PelStorage m_tempBufBeforeDbCtu;   ///< CTU-level buffer for the reconstruction before DBF (all components).

  // Buffers for residual.
  CompStorage m_tempBufResi;      ///< Frame-level buffer for the residual (luma).
  CompStorage m_tempBufResiCtu;   ///< CTU-level buffer for the residual (luma).

  // Buffers for SAO output.
  // TODO: don't process luma component in this buffers since it's not used
  PelStorage m_tempBufSao;      ///< Frame-level buffer for the SAO output.
  PelStorage m_tempBufSaoCtu;   ///< CTU-level buffer for the SAO output.

  // Results buffers

  MultiClassBuf  m_classifier;                    ///< Frame-level classification buffer (multiple classifiers).
  FixFiltCandBuf m_fixFilterResult[MAX_NUM_COMP]; ///< Buffers for frame-level fixed filter results.
  FixFiltCandBuf m_fixedFilterResultPerCtu;       ///< Buffers for CTU-level fixed filter results.
  CompBuf        m_gaussPic;                      ///< Buffer for frame-level Gaussian filter results.
  CompBuf        m_gaussCtu;                      ///< Buffer for CTU-level Gaussian fixed filter results.
  FixFiltResiBuf m_fixFilterResiResult;           ///< Buffers for frame-level residual-based fixed filter results.

  /// Flag indicating if a CTU's fixed filter results were partially calculated while deriving the neighbor's results.
  /// m_ctuPadFlag[ctuIdx] & 0x01: Left boundary has been calculated while deriving the left neighbor's results.
  /// m_ctuPadFlag[ctuIdx] & 0x02: Top boundary has been calculates while deriving the top neighbor's results.
  std::vector<uint8_t> m_ctuPadFlag;

  /// Flag indicating if a CTU's component has already been processed by the ALF decoder.
  /// If the flag is true, the CTU's fixed filter results may be reused by the CTUs neighbors.
  std::vector<bool> m_ctuAlreadyProcessedFlag[MAX_NUM_COMP];

  // CCALF-related parameters

  std::vector<AlfFilterShape> m_filterShapesCcAlf;
  CcAlfFilterParam            m_ccAlfFilterParam;

  inline int8_t *getScaleIdxVals(const int mode);
  inline short  *getCoeffVals(const int mode);
  inline Pel    *getClipVals(const int mode);

  /// Derive the classification and the small window size's fixed filter results.
  /// The fixed filter classification is derived for both window sizes as well as the adaptive filter classification.
  /// The fixed filter outputs are only derived for the small window size but for reconstruction and residual.
  void deriveClassification(const CPelBuf &srcLuma, const bool bResiFixed, const CPelBuf &srcResiLuma,
                            const CPelBuf &srcLumaBeforeDb, const uint8_t ctuPadFlag, const Area &blkDst,
                            const Area &blk, CodingStructure &cs, const int multipleClassifierIdx);

  void reconstructCoeffAPSs(CodingStructure &cs, bool luma, bool chroma, bool isRdo);
  void reconstructLumaCoeff(AlfParam &alfParam, const bool isRdo);
  void reconstructChromaCoeff(AlfParam &alfParam, const bool isRdo);

  /// Check if per-CTU padding is needed for the fixed filter results.
  /// Fixed filter results from a neighbor CTU cannot be used if a different QP might be used there.
  static bool isFixedFilterPaddedPerCtu(const Slice &slice) { return slice.getCuQpDeltaSubdiv(); }

  /// Calculate the second fixed filter results with possible per-CU QP for one or both candidate sets.
  void deriveSecondFixedFilterResults(const CPelBuf &srcLuma, const CPelBuf &srcLumaBeforeDb,
                                      const FixFiltCandBuf &firstFixFilterResults, const Area &blkSrc,
                                      const Area &blkDst, const CodingStructure &cs, const int fixedFilterSetCandIdx);

  /// Calculate the chroma fixed filter results with possible per-CU QP for one or both candidate sets.
  void deriveFixedFilterResultsChroma(const CPelBuf &src, const CPelBuf &srcBeforeDb, const Area &blkDst,
                                      const Area &blk, CodingStructure &cs, const int fixedFilterSetCandIdx,
                                      const CompID compId);

  /// Copy the fixed filter results to the target buffer for a given block area.
  void copyFixedFilterResults(PelUnitBuf &recDst, const Area &blkDst, const CompID compId,
                              const FixFiltSetCand fixedFilterSetCandIdx, const FixFiltIdx fixedFilterIdx);

  /// Copy the fixed filter results from the picture buffer to the CTU buffer and add some padding.
  void copyAndExtendFixedFilterResultsCtu(const FixFiltSetCand fixedFilterSetCandIdx, const FixFiltIdx fixedFilterIdx,
                                          const Area &blk);

  /// Add padding for the frame-level results of the given fixed filter.
  void extendFixedFilterResultsPic(const CompID compId, const FixFiltSetCand fixedFilterSetCandIdx,
                                   const FixFiltIdx fixedFilterIdx);

  // Calculate the output of the Gaussian fixed filter.
  void deriveGaussResults(const CPelBuf &srcLumaDb, const Area &blkDst, const Area &blk);

  /// Copy the Gaussian fixed filter results for the given area to the CTU buffer and add padding.
  void copyAndExtendGaussResultsCtu(const Area &blk);

  /// Add padding for the frame-level results of the Gaussian fixed filter.
  void extendGaussResultsPic();

  // TODO: ALF: Do we need both, the frame-level and the CTU-level buffers? BS 2023-09-21
  // TODO: ALF: Can we use only the FixFiltBuf for the selected set? BS 2023-09-21
  /// Calculate the output of the adaptive filter.
  void alfFiltering(const ClassBuf &classifier, const PelUnitBuf &recDst, const CPelBuf &recBeforeDb,
                    const CPelBuf &resiLuma, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk,
                    const CompID compId, const int8_t *scaleIdxSet, const short *filterSet, const Pel *fClipSet,
                    AlfFilterType filterType, bool isFixFiltPaddedPerCtu, const FixFiltSetCand fixedFilterSetCandIdx);

  void applyCcAlfFilter(CodingStructure &cs, CompID compID, const PelBuf &dstBuf, const PelUnitBuf &recYuvExt,
                        uint8_t    *filterControl,
                        const short filterSet[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF],
                        const int   selectedFilterIdx);

  static constexpr ClassifierIndex getClassifierIdxEnum(const int classifierIdx);
  static constexpr unsigned        numFixedFilterSetCands(AlfFilterType filterType)
  {
    return (filterType >= ALF_FILTER_9) ? NUM_FIXED_FILTER_SET_CANDS : 1;
  }
  static constexpr bool isCoeffRestricted(short coeff, bool luma);

private:
  /// Main block boundary location.
  enum class BndryLocMain : unsigned
  {
    L,   ///< left
    R,   ///< right
    T,   ///< top
    B,   ///< bottom
    NUM   ///< Number of main boundary locations
  };

  /// Block boundary location.
  enum class BndryLoc : unsigned
  {
    L,   ///< left
    R,   ///< right
    T,   ///< top
    B,   ///< bottom
    TL,   ///< top-left
    TR,   ///< top-right
    BL,   ///< bottom-left
    BR,   ///< bottom-right
    NUM   ///< number of boundary locations
  };

  struct NeighborArea : public Area
  {
    bool m_valid;

    inline constexpr NeighborArea &operator=(const Area &other);
    inline constexpr NeighborArea &operator=(Area &&other);
  };

  struct CalculationArea : public NeighborArea
  {
    int m_ctuIdxOffset;

    using NeighborArea::operator=;
  };

  using CalculationAreas = std::array<CalculationArea, static_cast<size_t>(BndryLoc::NUM)>;
  using PaddingAreas     = std::array<NeighborArea, static_cast<size_t>(BndryLocMain::NUM)>;

  struct NeighborCalcAndPadAreas
  {
    CalculationAreas m_calc;
    PaddingAreas     m_pad;
  };

  using alfFilterFunc = void(const ClassBuf &classifier, const PelUnitBuf &recDst, const CPelBuf &recBeforeDbLuma,
                             const CPelBuf &resiLuma, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk,
                             const CompID compId, const int8_t *scaleIdxSet, const short *filterSet,
                             const Pel *fClipSet, const ClpRng &clpRng, const FixFiltBuf &fixedFilterResults,
                             const FixFiltBuf &fixedFilterResultsPerCtu, const CompBuf &fixedFilterResiResults,
                             const CompBuf &gaussPic, const CompBuf &gaussCtu, bool isFixFiltPaddedPerCtu,
                             const FixFiltSetCand fixedFilterSetCandIdx);

  LaplacianBuf m_laplacian;   ///< Buffer holding the laplacian gradients for classification.

  // Function pointers for classification

  void (*m_deriveClassificationLaplacian)(const CPelBuf &srcLuma, const Area &blk, LaplacianBuf &laplacian,
                                          const int side);
  void (*m_deriveClassificationLaplacianBig)(const Area &curBlk, LaplacianBuf &laplacian);
  void (*m_deriveVariance)(const CPelBuf &srcLuma, const Area &blk, LaplacianBuf &variance);

  /// Function calculating the gradient-based classification for the fixed filters.
  void (*m_calcClassGradBasedFixedFilt)(ClassBuf &classifier, const Area &blkDst, const Area &cu,
                                        const ClsWndwSize clsWndwSize, int bitDepth, const LaplacianBuf &laplacian);

  /// Function calculating the gradient-based classification for the adaptive filters.
  void (*m_calcClassGradBasedAdaptFilt)(ClassBuf &classifier, const Area &blkDst, const Area &cu,
                                        const ClsWndwSize clsWndwSize, int bitDepth, const LaplacianBuf &laplacian);

  /// Function calculating the band-based classification for either the reconstruction or the residual.
  void (*m_calcClassBandBased)(ClassBuf &classifier, const Area &blkDst, const Area &cu, const CPelBuf &srcLuma,
                               const int classifierIdx, const int bitDepth, const CPelBuf &srcLumaResi,
                               LaplacianBuf::DirBuf &buffer);

  // Function pointers for fixed filtering

  void (*m_fixFilter9x9Db9Blk)(const ClassBuf &classifier, const CPelBuf &srcLuma, const CPelBuf &srcLumaBeforeDb,
                               const Area &curBlk, CompBuf &fixedFilterResult, const Area &blkDst, int picWidth,
                               const unsigned fixedFilterSetIdx, const ClpRng &clpRng,
                               const std::array<Pel, MAX_ALF_NUM_CLIP_VALS> &clippingValues);

  void (*m_fixFilter13x13Db9Blk)(const ClassBuf &classifier, const CPelBuf &srcLuma, const CPelBuf &srcLumaBeforeDb,
                                 const CompBuf &firstFixedFilterResult, const Area &curBlk, CompBuf &fixedFilterResult,
                                 const Area &blkDst, int picWidth, const unsigned fixedFilterSetIdx,
                                 const ClpRng &clpRng, const std::array<Pel, MAX_ALF_NUM_CLIP_VALS> &clippingValues);

  void (*m_gaussFiltering)(CompBuf &gaussPic, const CPelBuf &srcLuma, const Area &blkDst, const Area &blk,
                           const ClpRng &clpRng, const std::array<Pel, MAX_ALF_NUM_CLIP_VALS> &clippingValues);

  void (*m_filterResi9x9Blk)(const ClassBuf &classifier, const CPelBuf &srcResiLuma, const Area &curBlk,
                             const Area &blkDst, CompBuf &fixedFilterResiResults, int picWidth,
                             const unsigned fixedFilterSetIdx, const short (&classIndFixed)[NUM_CLASSES_FIX],
                             const ClpRng &clpRng, const std::array<Pel, MAX_ALF_NUM_CLIP_VALS> &clippingValues);

  // Function pointers for adaptive filtering

  alfFilterFunc *m_filter9x9Blk;
  alfFilterFunc *m_filter9x9BlkExtDbResi;
  alfFilterFunc *m_filter9x9BlkExtDbResiDirect;

  // Function pointers for CCALF

  void (*m_filterCcAlf)(const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const CPelUnitBuf &recSaoSrc,
                        const Area &blkDst, const Area &blkSrc, const CompID compId, const int16_t *filterCoeff,
                        const ClpRngs &clpRngs, CodingStructure &cs);

public:
  /// Derive the sums of the laplacian gradients for a 4x4 sliding window with 2x2 granularity.
  static void deriveClassificationLaplacian(const CPelBuf &srcLuma, const Area &blk, LaplacianBuf &laplacian,
                                            const int side);

  /// Calculate the sums of the laplacian gradients for a 12x12 sliding window with 2x2 granularity.
  static void deriveClassificationLaplacianBig(const Area &curBlk, LaplacianBuf &laplacian);

  /// Derive the variance for a sliding 10x10 window with 2x2 granularity.
  /// Note that all components of the laplacian buffer are used temporarily.
  static void deriveVariance(const CPelBuf &srcLuma, const Area &blk, LaplacianBuf &laplacian);

  /// Calculate the band-based classification for either the reconstruction or the residual.
  static void calcClassBandBased(ClassBuf &classifier, const Area &blkDst, const Area &cu, const CPelBuf &srcLuma,
                                 const int classifierIdx, const int bitDepth, const CPelBuf &srcResiLuma,
                                 LaplacianBuf::DirBuf &buffer);

  /// Calculate the gradient-based classification for the adaptive filters.
  static inline void calcClassGradBasedAdaptFilt(ClassBuf &classifier, const Area &blkDst, const Area &cu,
                                                 const ClsWndwSize clsWndwSize, int bitDepth,
                                                 const LaplacianBuf &laplacian);

  /// Calculate the gradient-based classification for the fixed filters.
  static inline void calcClassGradBasedFixedFilt(ClassBuf &classifier, const Area &blkDst, const Area &cu,
                                                 const ClsWndwSize clsWndwSize, int bitDepth,
                                                 const LaplacianBuf &laplacian);

  // Adaptive filtering

  template<AlfFilterType filtType> static alfFilterFunc filterBlk;

  // CCALF filtering

  static void fixedFilter9x9Db9Blk(const ClassBuf &classifier, const CPelBuf &src, const CPelBuf &srcBeforeDb,
                                   const Area &curBlk, CompBuf &fixedFilterResult, const Area &blkDst, int picWidth,
                                   const unsigned fixedFilterSetIdx, const ClpRng &clpRng,
                                   const std::array<Pel, MAX_ALF_NUM_CLIP_VALS> &clippingValues);

  static void fixedFilter13x13Db9Blk(const ClassBuf &classifier, const CPelBuf &src, const CPelBuf &srcBeforeDb,
                                     const CompBuf &firstFixedFilterResult, const Area &curBlk,
                                     CompBuf &fixedFilterResult, const Area &blkDst, int picWidth,
                                     const unsigned fixedFilterSetIdx, const ClpRng &clpRng,
                                     const std::array<Pel, MAX_ALF_NUM_CLIP_VALS> &clippingValues);

  static void gaussFiltering(CompBuf &gaussPic, const CPelBuf &srcLuma, const Area &blkDst, const Area &blk,
                             const ClpRng &clpRng, const std::array<Pel, MAX_ALF_NUM_CLIP_VALS> &clippingValues);

  static void fixedFilteringResi(const ClassBuf &classifier, const CPelBuf &srcResiLuma, const Area &curBlk,
                                 const Area &blkDst, CompBuf &fixedFilterResiResult, int picWidth,
                                 const unsigned fixedFilterSetIdx, const short (&classIndFixed)[NUM_CLASSES_FIX],
                                 const ClpRng &clpRng, const std::array<Pel, MAX_ALF_NUM_CLIP_VALS> &clippingValues);

  static void filterBlkCcAlf(const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const CPelUnitBuf &recSaoSrc,
                             const Area &blkDst, const Area &blkSrc, const CompID compId, const int16_t *filterCoeff,
                             const ClpRngs &clpRngs, CodingStructure &cs);

  static void extendBlockBoundaries(CompBuf &buf, const PaddingAreas &padAreas);

private:
  // Initialization

  void initAdaptiveLoopFilter();   ///< Init function pointers for non-SIMD case.
#ifdef TARGET_SIMD_X86
  void                         initAdaptiveLoopFilterX86();   ///< Init function pointers for SIMD case.
  template<X86_VEXT vext> void _initAdaptiveLoopFilterX86();
#endif

  // Classification

  /// Derive the classification and the small window size's fixed filter results for a single block.
  /// The fixed filter classification is derived for both window sizes as well as the adaptive filter classification.
  /// The fixed filter outputs are only derived for the small window size but for reconstruction and residual.
  void deriveClassificationAndFixFilterResultsBlk(const CPelBuf &srcLuma, const bool bResiFixed,
                                                  const CPelBuf &srcResiLuma, const CPelBuf &srcLumaBeforeDb,
                                                  const uint8_t ctuPadFlag, const Area &blkDst, const Area &blk,
                                                  CodingStructure &cs, int qp, const int multipleClassifierIdx);

  // Fixed filtering

  /// Calculate the first fixed filter results
  inline void alfFirstFixedFilterBlk(const CPelBuf &srcLuma, const CPelBuf &srcLumaBeforeDb, const Area &curBlk,
                                     CompBuf &fixedFilterResults, const Area &blkDst, const unsigned fixedFilterSetIdx);

  /// Calculate the second fixed filter results
  inline void alfSecondFixedFilterBlk(const CPelBuf &srcLuma, const CPelBuf &srcLumaBeforeDb,
                                      const CompBuf &firstFixedFilterResult, const Area &curBlk,
                                      CompBuf &fixedFilterResults, const Area &blkDst,
                                      const unsigned fixedFilterSetIdx);

  /// Calculate the first fixed filter results
  inline void alfChromaFixedFilterBlk(const CPelBuf &src, const CPelBuf &srcBeforeDb, const Area &curBlk,
                                      CompBuf &fixedFilterResults, const Area &blkDst, const unsigned fixedFilterSetIdx,
                                      const CompID compId);

  /// Calculate the second fixed filter results for a single block and one or both candidate sets.
  void deriveSecondFixedFilterResultsBlk(const CPelBuf &srcLuma, const CPelBuf &srcLumaBeforeDb,
                                         const FixFiltCandBuf &firstFixedFilterResults, const Area &blkSrc,
                                         const Area &blkDst, const int qp, const int fixedFilterSetCandIdx);

  /// Calculate the chroma fixed filter results for a single block and one or both candidate sets.
  void deriveFixedFilterResultsBlkChroma(const CPelBuf &src, const CPelBuf &srcBeforeDb, const Area &blkSrc,
                                         const Area &blkDst, const int qp, const int fixedFilterSetCandIdx,
                                         const CompID compId);

  /// Derive the fixed filter results for the blocks surrounding an area by calculating them or by padding.
  void deriveFixedFilterResultsCtuBoundary(const CPelBuf &srcLuma, const CPelBuf &srcLumaBeforeDb, const Area &blkDst,
                                           const int qp, int ctuIdx, const FixFiltSetCand fixedFilterSetCandIdx,
                                           const FixFiltIdx fixedFilterIdx);
  void deriveFixedFilterResultsCtuBoundaryChroma(const CPelBuf &src, const CPelBuf &srcBeforeDb, const Area &blkDst,
                                                 const int qp, int ctuIdx, const FixFiltSetCand fixedFilterSetCandIdx,
                                                 const CompID compId);

  void deriveFixedFilterResultsPerBlk(const CPelBuf &srcLuma, const CPelBuf &srcLumaBeforeDb, const Area &blkCur,
                                      const int qp, const FixFiltSetCand fixedFilterSetCandIdx,
                                      const FixFiltIdx fixedFilterIdx);
  void deriveFixedFilterResultsPerBlkChroma(const CPelBuf &src, const CPelBuf &srcBeforeDb, const Area &blkCur,
                                            const int qp, const FixFiltSetCand fixedFilterSetCandIdx,
                                            const CompID compId);

  void deriveGaussResultsCtuBoundary(const CPelBuf &srcLuma, const Area &blkDst, int ctuIdx);

  void deriveGaussResultsBlk(CompBuf &gaussPic, const CPelBuf &srcLuma, const Area &blkDst, const Area &blk,
                             const ClpRng &clpRng, const std::array<Pel, MAX_ALF_NUM_CLIP_VALS> &clippingValues);

  /// Helper function calculating the gradient-based classification.
  template<bool isFixedFilterClassifier> static void calcClassHelper(ClassBuf &classifier, const Area &blkDst,
                                                                     const Area &curBlk, const ClsWndwSize dirWindSize,
                                                                     unsigned bitDepth, const LaplacianBuf &laplacian);

  // Neighbor and boundary handling

  NeighborCalcAndPadAreas calcCtuBoundaryCalcAndPadAreas(const Area &ctuArea, const unsigned requiredSize,
                                                         const unsigned desiredWidth, const unsigned desiredHeight,
                                                         const unsigned maxPadSize, const bool luma) const;
};

void AdaptiveLoopFilterEcm::alfFirstFixedFilterBlk(const CPelBuf &srcLuma, const CPelBuf &srcLumaBeforeDb,
                                                   const Area &curBlk, CompBuf &fixedFilterResult, const Area &blkDst,
                                                   const unsigned fixedFilterSetIdx)
{
  m_fixFilter9x9Db9Blk(m_classifier[ClassifierIndex::FIXED_FILTER_4x4], srcLuma, srcLumaBeforeDb, curBlk,
                       fixedFilterResult, blkDst, m_picWidth, fixedFilterSetIdx, m_clpRngs.comp[CompID::COMP_Y],
                       m_alfClippingValues[ChannelType::LUMA]);
}

void AdaptiveLoopFilterEcm::alfSecondFixedFilterBlk(const CPelBuf &srcLuma, const CPelBuf &srcLumaBeforeDb,
                                                    const CompBuf &firstFixedFilterResult, const Area &curBlk,
                                                    CompBuf &fixedFilterResult, const Area &blkDst,
                                                    const unsigned fixedFilterSetIdx)
{
  m_fixFilter13x13Db9Blk(m_classifier[ClassifierIndex::FIXED_FILTER_12x12], srcLuma, srcLumaBeforeDb,
                         firstFixedFilterResult, curBlk, fixedFilterResult, blkDst, m_picWidth, fixedFilterSetIdx,
                         m_clpRngs.comp[CompID::COMP_Y], m_alfClippingValues[ChannelType::LUMA]);
}

void AdaptiveLoopFilterEcm::alfChromaFixedFilterBlk(const CPelBuf &src, const CPelBuf &srcBeforeDb, const Area &curBlk,
                                                    CompBuf &fixedFilterResult, const Area &blkDst,
                                                    const unsigned fixedFilterSetIdx, const CompID compId)
{
  m_fixFilter9x9Db9Blk(m_classifier[ClassifierIndex::FIXED_FILTER_CHROMA], src, srcBeforeDb, curBlk, fixedFilterResult,
                       blkDst, m_picWidth, fixedFilterSetIdx, m_clpRngs.comp[compId],
                       m_alfClippingValues[toChannelType(compId)]);
}

void AdaptiveLoopFilterEcm::calcClassGradBasedAdaptFilt(ClassBuf &classifier, const Area &blkDst, const Area &cu,
                                                        const ClsWndwSize clsWndwSize, int bitDepth,
                                                        const LaplacianBuf &laplacian)
{
  return calcClassHelper<false>(classifier, blkDst, cu, clsWndwSize, bitDepth, laplacian);
}

void AdaptiveLoopFilterEcm::calcClassGradBasedFixedFilt(ClassBuf &classifier, const Area &blkDst, const Area &cu,
                                                        const ClsWndwSize clsWndwSize, int bitDepth,
                                                        const LaplacianBuf &laplacian)
{
  return calcClassHelper<true>(classifier, blkDst, cu, clsWndwSize, bitDepth, laplacian);
}

constexpr AdaptiveLoopFilterEcm::ClassifierIndex AdaptiveLoopFilterEcm::getClassifierIdxEnum(const int classifierIdx)
{
  switch (classifierIdx)
  {
  case 0:
    return ClassifierIndex::ADAPT_FILTER_GRAD_BASED;
  case 1:
    return ClassifierIndex::ADAPT_FILTER_BAND_BASED_RECO;
  case 2:
    return ClassifierIndex::ADAPT_FILTER_BAND_BASED_RESI;
  default:
    THROW("Classifier index is out of range.");
  }
}

constexpr bool AdaptiveLoopFilterEcm::isCoeffRestricted(short coeff, bool luma)
{
  if (coeff == 0)
  {
    return true;
  }
  coeff      = std::abs(coeff);
  short log2 = floorLog2(coeff);
  if (luma)
  {
    if (log2 > 0)
    {
      if (((short)1 << log2) + ((short)1 << (log2 - 1)) == coeff)
      {
        return true;
      }
    }
  }
  return ((short)1 << log2) == coeff;
}

int8_t *AdaptiveLoopFilterEcm::getScaleIdxVals(const int mode)
{
  CHECKD(LumaCtbModeHandler::isFixedFilter(mode), "We should not get here for a fixed filter.");
  return m_scaleIdxApsLuma[LumaCtbModeHandler::getApsIdx(mode)][LumaCtbModeHandler::getAlternative(mode)];
}

short *AdaptiveLoopFilterEcm::getCoeffVals(const int mode)
{
  CHECKD(LumaCtbModeHandler::isFixedFilter(mode), "We should not get here for a fixed filter.");
  return m_coeffApsLuma[LumaCtbModeHandler::getApsIdx(mode)][LumaCtbModeHandler::getAlternative(mode)];
}

Pel *AdaptiveLoopFilterEcm::getClipVals(const int mode)
{
  CHECKD(LumaCtbModeHandler::isFixedFilter(mode), "We should not get here for a fixed filter.");
  return m_clippApsLuma[LumaCtbModeHandler::getApsIdx(mode)][LumaCtbModeHandler::getAlternative(mode)];
}

constexpr AdaptiveLoopFilterEcm::FixedFilterSetCands::FixedFilterSetCands(const int qp)
  : m_cands { static_cast<unsigned>((qp < 22) ? 0 : min((qp - 22) / 4, NUM_FIXED_FILTER_SETS - 2)), 0 }
{
  static_assert(NUM_FIXED_FILTER_SET_CANDS == 2, "Only implemented for 2 fixed filter set candidates.");
  m_cands[1] = m_cands[0] + 1;
}

constexpr const unsigned &AdaptiveLoopFilterEcm::FixedFilterSetCands::operator[](
  const AdaptiveLoopFilterEcm::FixFiltSetCand &fixedFilterSetCandIdx) const
{
  return m_cands[static_cast<size_t>(fixedFilterSetCandIdx)];
}

constexpr unsigned &AdaptiveLoopFilterEcm::FixedFilterSetCands::operator[](
  const AdaptiveLoopFilterEcm::FixFiltSetCand &fixedFilterSetCandIdx)
{
  return const_cast<unsigned &>(const_cast<const FixedFilterSetCands *>(this)->operator[](fixedFilterSetCandIdx));
}

unsigned AdaptiveLoopFilterEcm::FixedFilterSetCands::getFixedFilterSet(
  const int qp, const AdaptiveLoopFilterEcm::FixFiltSetCand fixedFilterSetCandIdx)
{
  return FixedFilterSetCands(qp)[fixedFilterSetCandIdx];
}

template<typename T, unsigned size>
constexpr AdaptiveLoopFilterEcm::FilterRowPtrs<T, size>::FilterRowPtrs(T *const center, const ptrdiff_t stride)
{
  set(center, stride);
}

template<typename T, unsigned size> constexpr AdaptiveLoopFilterEcm::FilterRowPtrs<T, size> &
  AdaptiveLoopFilterEcm::FilterRowPtrs<T, size>::set(T *const center, const ptrdiff_t stride)
{
  this->m_rows[this->m_offset] = center;
  for (int diff = 1; diff <= this->m_offset; ++diff)
  {
    this->m_rows[this->m_offset - diff] = this->m_rows[this->m_offset - diff + 1] - stride;
    this->m_rows[this->m_offset + diff] = this->m_rows[this->m_offset + diff - 1] + stride;
  }

  return *this;
}

constexpr AdaptiveLoopFilterEcm::NeighborArea &AdaptiveLoopFilterEcm::NeighborArea::operator=(const Area &other)
{
  Area::operator=(other);
  return *this;
}

constexpr AdaptiveLoopFilterEcm::NeighborArea &AdaptiveLoopFilterEcm::NeighborArea::operator=(Area &&other)
{
  Area::operator=(std::move(other));
  return *this;
}

#endif   // __ADAPTIVELOOPFILTERECM__
