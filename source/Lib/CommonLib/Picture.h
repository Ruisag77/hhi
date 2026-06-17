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

/** \file     Picture.h
 *  \brief    Description of a coded picture
 */

#ifndef __PICTURE__
#define __PICTURE__

#include "CommonDef.h"

#include "Common.h"
#include "Unit.h"
#include "Buffer.h"
#include "Unit.h"
#include "Slice.h"
#include "CodingStructure.h"
#include "Hash.h"
#include "MCTS.h"
#include "SEIColourTransform.h"
#include <deque>
#include "SEIFilmGrainSynthesizer.h"
#include "AlfParameters.h"

class SEI;
class AQpLayer;

typedef std::list<SEI *> SEIMessages;

struct Picture : public UnitArea
{
  uint32_t margin;
  Picture();

  void create(const ChromaFormat &_chromaFormat, const Size &size, const unsigned _maxCUSize, const unsigned _margin,
              const bool _decoder, const int _layerId, const bool rprEnabled, const bool gopBasedTemporalFilterEnabled,
              const bool fgcSEIAnalysisEnabled
#if JVET_Z0120_SII_SEI_PROCESSING
              ,
              const bool enablePostFilteringForHFR
#endif
#if ENABLE_NNLF
              ,
              bool nnlfStore
#endif
  );

  void destroy();

  void createTempBuffers(const unsigned _maxCUSize);
  void destroyTempBuffers();

  int                      m_padValue;
  bool                     m_isMctfFiltered;
  SEIFilmGrainSynthesizer *m_grainCharacteristic;
  PelStorage              *m_grainBuf;
  void       createGrainSynthesizer(bool firstPictureInSequence, SEIFilmGrainSynthesizer *grainCharacteristics,
                                    PelStorage *grainBuf, int width, int height, ChromaFormat fmt, int bitDepth);
  PelUnitBuf getDisplayBufFG(bool wrap = false);

  SEIColourTransformApply *m_colourTranfParams;
  PelStorage              *m_invColourTransfBuf;
  void       createColourTransfProcessor(bool firstPictureInSequence, SEIColourTransformApply *ctiCharacteristics,
                                         PelStorage *ctiBuf, int width, int height, ChromaFormat fmt, int bitDepth);
  PelUnitBuf getDisplayBuf();

  SEIMessages m_nnpfcActivated;

#if JVET_Z0120_SII_SEI_PROCESSING
  void     copyToPic(const SPS *sps, PelStorage *picYuvSrc, PelStorage *picYuvDst);
  Picture *findPrevPicPOC(Picture *pic, PicList *picList);
  Picture *findNextPicPOC(Picture *pic, PicList *picList);
  void     xOutputPostFilteredPic(Picture *pic, PicList *picList, int blendingRatio);
  void     xOutputPreFilteredPic(Picture *pic, PicList *picList, int blendingRatio, int intraPeriod);
#endif

  PelBuf            getOrigBuf(const CompArea &blk);
  const CPelBuf     getOrigBuf(const CompArea &blk) const;
  PelUnitBuf        getOrigBuf(const UnitArea &unit);
  const CPelUnitBuf getOrigBuf(const UnitArea &unit) const;
  PelUnitBuf        getOrigBuf();
  const CPelUnitBuf getOrigBuf() const;
  PelBuf            getOrigBuf(const CompID compID);
  const CPelBuf     getOrigBuf(const CompID compID) const;
  PelBuf            getTrueOrigBuf(const CompID compID);
  const CPelBuf     getTrueOrigBuf(const CompID compID) const;
  PelUnitBuf        getTrueOrigBuf();
  const CPelUnitBuf getTrueOrigBuf() const;
  PelBuf            getTrueOrigBuf(const CompArea &blk);
  const CPelBuf     getTrueOrigBuf(const CompArea &blk) const;

  PelUnitBuf        getFilteredOrigBuf();
  const CPelUnitBuf getFilteredOrigBuf() const;
  PelBuf            getFilteredOrigBuf(const CompArea &blk);
  const CPelBuf     getFilteredOrigBuf(const CompArea &blk) const;

  PelBuf            getPredBuf(const CompArea &blk);
  const CPelBuf     getPredBuf(const CompArea &blk) const;
  PelUnitBuf        getPredBuf(const UnitArea &unit);
  const CPelUnitBuf getPredBuf(const UnitArea &unit) const;

  PelBuf            getResiBuf(const CompArea &blk);
  const CPelBuf     getResiBuf(const CompArea &blk) const;
  PelUnitBuf        getResiBuf(const UnitArea &unit);
  const CPelUnitBuf getResiBuf(const UnitArea &unit) const;

  PelBuf            getRecoBuf(const CompID compID, bool wrap = false);
  const CPelBuf     getRecoBuf(const CompID compID, bool wrap = false) const;
  PelBuf            getRecoBuf(const CompArea &blk, bool wrap = false);
  const CPelBuf     getRecoBuf(const CompArea &blk, bool wrap = false) const;
  PelUnitBuf        getRecoBuf(const UnitArea &unit, bool wrap = false);
  const CPelUnitBuf getRecoBuf(const UnitArea &unit, bool wrap = false) const;
  PelUnitBuf        getRecoBuf(bool wrap = false);
  const CPelUnitBuf getRecoBuf(bool wrap = false) const;

  PelBuf            getBuf(const CompID compID, const PictureType &type);
  const CPelBuf     getBuf(const CompID compID, const PictureType &type) const;
  PelBuf            getBuf(const CompArea &blk, const PictureType &type);
  const CPelBuf     getBuf(const CompArea &blk, const PictureType &type) const;
  PelUnitBuf        getBuf(const UnitArea &unit, const PictureType &type);
  const CPelUnitBuf getBuf(const UnitArea &unit, const PictureType &type) const;

#if ENABLE_NNLF
  PelBuf            getBsMapBuf(const CompID compID);
  PelUnitBuf        getBsMapBuf();
  const CPelUnitBuf getBsMapBuf() const;
  PelUnitBuf        getBsMapBuf(const UnitArea &unit);
  const CPelUnitBuf getBsMapBuf(const UnitArea &unit) const;
  PelBuf            getBsMapBuf(const CompArea &blk);
  const CPelBuf     getBsMapBuf(const CompArea &blk) const;

  PelBuf            getPredBufCustom(const CompID compID);
  PelUnitBuf        getPredBufCustom();
  const CPelUnitBuf getPredBufCustom() const;
  PelBuf            getPredBufCustom(const CompArea &blk);
  const CPelBuf     getPredBufCustom(const CompArea &blk) const;
  PelUnitBuf        getPredBufCustom(const UnitArea &unit);
  const CPelUnitBuf getPredBufCustom(const UnitArea &unit) const;

  PelBuf            getRecBeforeDbfBuf(const CompID compID);
  PelUnitBuf        getRecBeforeDbfBuf();
  const CPelUnitBuf getRecBeforeDbfBuf() const;
  PelBuf            getRecBeforeDbfBuf(const CompArea &blk);
  const CPelBuf     getRecBeforeDbfBuf(const CompArea &blk) const;
  PelUnitBuf        getRecBeforeDbfBuf(const UnitArea &unit);
  const CPelUnitBuf getRecBeforeDbfBuf(const UnitArea &unit) const;

  PelBuf            getBlockPredModeBuf(const CompID compID);
  PelUnitBuf        getBlockPredModeBuf();
  const CPelUnitBuf getBlockPredModeBuf() const;
  PelBuf            getBlockPredModeBuf(const CompArea &blk);
  const CPelBuf     getBlockPredModeBuf(const CompArea &blk) const;
  PelUnitBuf        getBlockPredModeBuf(const UnitArea &unit);
  const CPelUnitBuf getBlockPredModeBuf(const UnitArea &unit) const;

  PelBuf            getBlockQpBuf(const CompID compID);
  PelUnitBuf        getBlockQpBuf();
  const CPelUnitBuf getBlockQpBuf() const;
  PelBuf            getBlockQpBuf(const CompArea &blk);
  const CPelBuf     getBlockQpBuf(const CompArea &blk) const;
  PelUnitBuf        getBlockQpBuf(const UnitArea &unit);
  const CPelUnitBuf getBlockQpBuf(const UnitArea &unit) const;

  void dumpPicBpmInfo();
  void dumpQpBlock();

  void paddingPicBufBorder(const PictureType picType, const int padSize, const int value = 0);
  void paddingBsMapBufBorder(const int padSize) { paddingPicBufBorder(PIC_BS_MAP, padSize); }
  void paddingRecBeforeDbfBufBorder(const int padSize) { paddingPicBufBorder(PIC_REC_BEFORE_DBF, padSize); }
  void paddingPredBufBorder(const int padSize) { paddingPicBufBorder(PIC_PREDICTION_CUSTOM, padSize); }
  void paddingBPMBufBorder(const int padSize) { paddingPicBufBorder(PIC_BLOCK_PRED_MODE, padSize); }
  void paddingBlockQPBufBorder(const int padSize, const int value)
  {
    paddingPicBufBorder(PIC_BLOCK_QP, padSize, value);
  }

  NnlfFilterParameters m_picprm;
  NNLFInferSize        getInferSize(const Slice &slice);
  void                 initPicprms(const Slice &slice);
#endif

#if JVET_Z0120_SII_SEI_PROCESSING
  PelUnitBuf        getPostRecBuf();
  const CPelUnitBuf getPostRecBuf() const;
#endif

  void extendPicBorder(const PPS *pps);
  void extendWrapBorder(const PPS *pps);
  void extendMcPaddedBorder(int end);
  void finalInit(const VPS *vps, const SPS &sps, const PPS &pps, PicHeader *picHeader, APS **alfApss, APS *lmcsAps,
                 APS *scalingListAps);

  void calcLumaClpParams();

  NalUnitType getPictureType() const { return m_pictureType; }
  void        setPictureType(const NalUnitType val) { m_pictureType = val; }
  Pel        *getOrigin(const PictureType &type, const CompID compID) const;

  void fillSliceLossyLosslessArray(std::vector<uint16_t> sliceLosslessArray, bool mixedLossyLossless);
  bool losslessSlice(uint32_t sliceIdx) const { return m_lossylosslessSliceArray[sliceIdx]; }

  void        createSpliceIdx(int nums);
  bool        getSpliceFull();
  static void sampleRateConv(const ScalingRatio scalingRatio, int scaleX, int scaleY, const CPelBuf &beforeScale,
                             const int beforeScaleLeftOffset, const int beforeScaleTopOffset, const PelBuf &afterScale,
                             const int afterScaleLeftOffset, const int afterScaleTopOffset, const int bitDepth,
                             const bool useLumaFilter, const bool downsampling, const bool horCollocatedPositionFlag,
                             const bool verCollocatedPositionFlag, const bool rescaleForDisplay,
                             const int upscaleFilterForDisplay);

  static void rescalePicture(const ScalingRatio scalingRatio, const CPelUnitBuf &beforeScaling,
                             const Window &scalingWindowBefore, const PelUnitBuf &afterScaling,
                             const Window &scalingWindowAfter, const ChromaFormat chromaFormatIdc,
                             const BitDepths &bitDepths, const bool useLumaFilter, const bool downsampling,
                             const bool horCollocatedChromaFlag, const bool verCollocatedChromaFlag,
                             bool rescaleForDisplay = false, int upscaleFilterForDisplay = 0);

public:
  Window      m_conformanceWindow;
  Window      m_scalingWindow;
  int         m_decodingOrderNumber;
  NalUnitType m_pictureType;

  bool m_isSubPicBorderSaved;

  PelStorage m_bufSubPicAbove;
  PelStorage m_bufSubPicBelow;
  PelStorage m_bufSubPicLeft;
  PelStorage m_bufSubPicRight;

  PelStorage m_bufWrapSubPicAbove;
  PelStorage m_bufWrapSubPicBelow;

  void saveSubPicBorder(int POC, int subPicX0, int subPicY0, int subPicWidth, int subPicHeight);
  void extendSubPicBorder(int POC, int subPicX0, int subPicY0, int subPicWidth, int subPicHeight);
  void restoreSubPicBorder(int POC, int subPicX0, int subPicY0, int subPicWidth, int subPicHeight);

  bool                        m_extendedBorder;
  bool                        m_wrapAroundValid;
  unsigned                    m_wrapAroundOffset;
  bool                        m_referenced;
  bool                        m_reconstructed;
  bool                        m_neededForOutput;
  bool                        m_usedByCurr;
  bool                        m_longTerm;
  bool                        m_topField;
  bool                        m_fieldPic;
  EnumArray<int, ChannelType> m_prevQP;
  bool                        m_precedingDRAP; // preceding a DRAP picture in decoding order
  int                         m_edrapRapId;
  bool                        m_nonReferencePictureFlag;
  uint8_t                     m_maxTemporalBtDepth;

  int                 m_poc;
  uint32_t            m_temporalId;
  ClpRng              m_lumaClpRng;
  ClpRng              m_lumaClpRngforQuant;
  int                 m_layerId;
  std::vector<SubPic> m_subPictures;
  int                 m_numSlices;
  std::vector<int>    m_sliceSubpicIdx;

  bool m_subLayerNonReferencePictureDueToSTSA;

  int              *m_spliceIdx;
  int               m_ctuNums;
  int               m_lossyQP;
  std::vector<bool> m_lossylosslessSliceArray;
  bool              m_interLayerRefPicFlag;
  bool              m_mixedNaluTypesInPicFlag;

  PelStorage     m_bufs[NUM_PIC_TYPES];
  const Picture *m_unscaledPic;

  Hash m_hashMap;
  void addPictureToHashMapForInter();

  CodingStructure    *m_cs;
  std::deque<Slice *> m_slices;
  SEIMessages         m_SEIs;

  uint32_t getPicWidthInLumaSamples() const { return getRecoBuf(COMP_Y).width; }
  uint32_t getPicHeightInLumaSamples() const { return getRecoBuf(COMP_Y).height; }
  bool     isRefScaled(const PPS *pps) const
  {
    return m_unscaledPic->getPicWidthInLumaSamples() != pps->m_picWidthInLumaSamples ||
      m_unscaledPic->getPicHeightInLumaSamples() != pps->m_picHeightInLumaSamples ||
      m_scalingWindow.m_winLeftOffset != pps->m_scalingWindow.m_winLeftOffset ||
      m_scalingWindow.m_winRightOffset != pps->m_scalingWindow.m_winRightOffset ||
      m_scalingWindow.m_winTopOffset != pps->m_scalingWindow.m_winTopOffset ||
      m_scalingWindow.m_winBottomOffset != pps->m_scalingWindow.m_winBottomOffset;
  }
  bool isWrapAroundEnabled(const PPS *pps) const { return pps->m_wrapAroundEnabledFlag && !isRefScaled(pps); }

  void   allocateNewSlice();
  Slice *swapSliceObject(Slice *p, uint32_t i);
  void   clearSliceBuffer();

  void copyAdaptedLumaClip(bool copyRange = true);

  MCTSInfo                m_mctsInfo;
  std::vector<AQpLayer *> m_aqlayer;

  ChromaFormat m_chromaFormatIdc;
  BitDepths    m_bitDepths;

#if !KEEP_PRED_AND_RESI_SIGNALS
private:
  UnitArea m_ctuArea;
#endif

  AlfParameters::CtuModes m_alfModes;

  std::vector<SAOBlkParam> m_sao[2];

  template<boundaryDirection T> void TemplateMatchingPadding(Area subpicArea);

public:
  SAOBlkParam *getSAO(int id = 0) { return &m_sao[id][0]; };
  void         resizeSAO(unsigned numEntries, int dstid) { m_sao[dstid].resize(numEntries); }
  void         copySAO(const Picture &src, int dstid)
  {
    std::copy(src.m_sao[0].begin(), src.m_sao[0].end(), m_sao[dstid].begin());
  }

  BifParams &getBifParam(const CompID compID) { return m_bifParams[compID]; }
  void       resizeBIF(const CompID compID, unsigned numEntries)
  {
    m_bifParams[compID].numBlocks = numEntries;
    m_bifParams[compID].ctuOn.resize(numEntries);
    std::fill(m_bifParams[compID].ctuOn.begin(), m_bifParams[compID].ctuOn.end(), 0);
  };

  void copyBIF(const Picture &src)
  {
    m_bifParams[COMP_Y]  = src.m_bifParams[COMP_Y];
    m_bifParams[COMP_Cb] = src.m_bifParams[COMP_Cb];
    m_bifParams[COMP_Cr] = src.m_bifParams[COMP_Cr];
  }

  BifParams m_bifParams[MAX_NUM_COMP];

#if ENABLE_QPA
  std::vector<double> m_uEnerHpCtu;   ///< CTU-wise L2 or squared L1 norm of high-passed luma input
  std::vector<Pel>    m_iOffsetCtu;   ///< CTU-wise DC offset (later QP index offset) of luma input
#if ENABLE_QPA_SUB_CTU
  std::vector<int8_t> m_subCtuQP;   ///< sub-CTU-wise adapted QPs for delta-QP depth of 1 or more
#endif
#endif

  void copyAlfData(const Picture &p);
  void resizeAlfData(int numEntries);

  const AlfParameters::CtuModes &getAlfModes() const { return m_alfModes; }
  AlfParameters::CtuModes       &getAlfModes() { return m_alfModes; }
  const AlfParameters::CtbModes &getAlfModes(const int compIdx) const { return m_alfModes[compIdx]; }
  AlfParameters::CtbModes       &getAlfModes(const int compIdx) { return m_alfModes[compIdx]; }
};

int  calcAndPrintHashStatus(const CPelUnitBuf &pic, const class SEIDecodedPictureHash *pictureHashSEI,
                            const BitDepths &bitDepths, const MsgLevel msgl);
void calcAndPrintHashValue(const CPelUnitBuf &pic, HashType hash, const BitDepths &bitDepths, const MsgLevel msgl);

uint32_t calcMD5(const CPelUnitBuf &pic, PictureHash &digest, const BitDepths &bitDepths);
uint32_t calcMD5WithCropping(const CPelUnitBuf &pic, PictureHash &digest, const BitDepths &bitDepths,
                             const int leftOffset, const int rightOffset, const int topOffset, const int bottomOffset);

std::string hashToString(const PictureHash &digest, int numChar);

typedef std::list<Picture *> PicList;

#endif
