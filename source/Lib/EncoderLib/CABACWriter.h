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

/** \file     CABACWriter.h
 *  \brief    Writer for low level syntax
 */

#ifndef __CABACWRITER__
#define __CABACWRITER__

#include "CommonLib/BitStream.h"
#include "CommonLib/ContextModelling.h"
#include "BinEncoder.h"

//! \ingroup EncoderLib
//! \{

class EncCu;
class CABACWriter : public DeriveCtx
{
public:
  CABACWriter(BinEncIf &binEncoder, CABACDataStore *cabacDataStore)
    : m_CABACDataStore(cabacDataStore)
    , m_binEncoder(binEncoder)
    , m_bitstream(nullptr)
  {
    m_testCtx = m_binEncoder.getCtx();
    m_encCu   = nullptr;
  }
  virtual ~CABACWriter() {}

public:
#if ENABLE_CABAC_DUMP
  void initCtxModels(Slice &slice);
#else
  void initCtxModels(const Slice &slice);
#endif
  void      setEncCu(EncCu *pcEncCu) { m_encCu = pcEncCu; }
  SliceType getCtxInitId(const Slice &slice);
  void      initBitstream(OutputBitstream *bitstream)
  {
    m_bitstream = bitstream;
    m_binEncoder.init(m_bitstream);
  }

  const Ctx &getCtx() const { return m_binEncoder.getCtx(); }
  Ctx       &getCtx() { return m_binEncoder.getCtx(); }

  void                  start() { m_binEncoder.start(); }
  bool                  countWithUpdate(bool useUpdate = true) { return m_binEncoder.countWithUpdate(useUpdate); }
  void                  resetBits() { m_binEncoder.resetBits(); }
  uint64_t              getEstFracBits() const { return m_binEncoder.getEstFracBits(); }
  uint32_t              getNumBins() { return m_binEncoder.getNumBins(); }
  bool                  isEncoding() { return m_binEncoder.isEncoding(); }
  void                  setBinBufferActive(bool b) { m_binEncoder.setBinBufferActive(b); }
  void                  setBinBuffer(BinStoreVector *bb) { m_binEncoder.setBinBuffer(bb); }
  const BinStoreVector *getBinBuffer() const { return m_binEncoder.getBinBuffer(); }
  void                  updateCtxs(BinStoreVector *bb) { m_binEncoder.updateCtxs(bb); }

public:
  // slice segment data (clause 7.3.8.1)
  void end_of_slice();

  // coding tree unit (clause 7.3.8.2)
  void coding_tree_unit(CodingStructure &cs, const UnitArea &area, EnumArray<int, ChannelType> &qps, unsigned ctuRsAddr,
                        bool skipSao = false, bool skipAlf = false, bool skipLfCccm = false
#if ENABLE_NNLF
                        ,
                        bool skipNnlf = false
#endif
  );

  // sao (clause 7.3.8.3)
  void sao(const Slice &slice, unsigned ctuRsAddr);
  void sao_block_params(const SAOBlkParam &saoPars, const BitDepths &bitDepths, const bool *sliceEnabled,
                        bool leftMergeAvail, bool aboveMergeAvail, bool onlyEstMergeInfo);
  void sao_offset_params(const SAOOffset &ctbPars, CompID compID, bool sliceEnabled, int bitDepth);

  void bif(const CompID compID, const Slice &slice, const BifParams &bifParams);
  void bif(const CompID compID, const Slice &slice, const BifParams &bifParams, unsigned ctuRsAddr);
  void codeCcSaoControlIdc(uint8_t idcVal, CodingStructure &cs, const CompID compID, const int curIdx,
                           const uint8_t *controlIdc, Position lumaPos, const int setNum);
  // coding (quad)tree (clause 7.3.8.4)
  void coding_tree(const CodingStructure &cs, Partitioner &pm, CUCtx &cuCtx, Partitioner *pPartitionerChroma = nullptr,
                   CUCtx *pCuCtxChroma = nullptr);
  void split_cu_mode(const PartSplit split, const CodingStructure &cs, Partitioner &pm);

  // coding unit (clause 7.3.8.5)
  void coding_unit(const CodingUnit &cu, Partitioner &pm, CUCtx &cuCtx);
  void cu_skip_flag(const CodingUnit &cu);
  void pred_mode(const CodingUnit &cu);
  void bdpcm_mode(const CodingUnit &cu, const CompID compID);
  void cu_pred_data_intra(const CodingUnit &cu, const CUCtxIntra &cuCtxIntra);

  void cu_pred_data(const CodingUnit &cu);
  void cu_bcw_flag(const CodingUnit &cu);
  void obmc_flag(const CodingUnit &cu);
  void extend_ref_line(const CodingUnit &cu);
  void sgpm_flag(const CodingUnit &cu);
  void intra_luma_pred_mode(const CodingUnit &cu, const CUCtxIntra &cuCtxIntra);
  void intra_chroma_lmc_mode(const CodingUnit &cu);
  void intra_chroma_pred_mode(const CodingUnit &cu);
  void cclmDelta(const int8_t delta);
  void cclmDeltaSlope(const CodingUnit &cu);
  void cccmFlag(const CodingUnit &cu);
  void nonLocalCCPIndex(const CodingUnit &cu);
  void decoderDerivedCcpModes(const CodingUnit &cu);
  void ccFilterFlag(const CodingUnit &cu);
  void cu_residual(const CodingUnit &cu, Partitioner &pm, CUCtx &cuCtx);
  void rqt_root_cbf(const CodingUnit &cu);
  void sbt_mode(const CodingUnit &cu);
  void end_of_ctu(const CodingUnit &cu, CUCtx &cuCtx);
  void mip_flag(const CodingUnit &cu);
  void bvg_mip_flag(const CodingUnit &cu);
  void mip_pred_mode(const CodingUnit &cu);
  void cu_palette_info(const CodingUnit &cu, CompID compBegin, uint32_t numComp, CUCtx &cuCtx);
  void cuPaletteSubblockInfo(const CodingUnit &cu, CompID compBegin, uint32_t numComp, int subSetId,
                             uint32_t &prevRunPos, unsigned &prevRunType);
  void planarDir(const CodingUnit &cu);
  void dimd_flag(const CodingUnit &cu);
  void dimdChromaFlag(const CodingUnit &cu);
  void timd_flag(const CodingUnit &cu);
  void timd_sad_flag(const CodingUnit &cu);
  void obic_flag(const CodingUnit &cu);
  void eip_flag(const CodingUnit &cu);

  Pel  writePLTIndex(const CodingUnit &cu, uint32_t idx, PelBuf &paletteIdx, PLTtypeBuf &paletteRunType, int maxSymbol,
                     CompID compBegin);
  void cu_lic_flag(const CodingUnit &cu);

  // prediction unit (clause 7.3.8.6)
  void prediction_unit(const CodingUnit &cu);
  void merge_flag(const CodingUnit &cu);
  void merge_data(const CodingUnit &cu);
  void affine_flag(const CodingUnit &cu);
  void subblock_merge_flag(const CodingUnit &cu);
  void merge_idx(const CodingUnit &cu);
  void mmvd_merge_idx(const CodingUnit &cu);
  void affine_mmvd_data(const CodingUnit &cu);
  void bm_merge_flag(const CodingUnit &cu);
  void geo_mmvd_idx(const CodingUnit &cu, RefPicList eRefPicList);
  void geo_merge_idx(const CodingUnit &cu);
  void geo_merge_idx1(const CodingUnit &cu);

  double geo_mode_est(const TempCtx &ctxStart, const int geoMode);
  double geo_mergeIdx_est(const TempCtx &ctxStart, const int candIdx, const int maxNumGeoCand);
  double geo_mmvdFlag_est(const TempCtx &ctxStart, const int flag);
  double geo_mmvdIdx_est(const TempCtx &ctxStart, const int mmvdIdx, const bool extMMVD);

  double   geo_intraFlag_est(const TempCtx &ctxStart, const int intraFlag);
  uint64_t geo_bld_flag_est(const TempCtx &ctxStart, const int flag);
  void     geo_adaptive_blending_idx(const int idx);

  void imv_mode(const CodingUnit &cu);
  void affine_amvr_mode(const CodingUnit &cu);
  void inter_pred_idc(const CodingUnit &cu);
  void ref_idx(const CodingUnit &cu, RefPicList eRefList);
  void mvp_flag(const CodingUnit &cu, RefPicList eRefList);

  void ciip_flag(const CodingUnit &cu, bool usageIsInferred);
  void smvd_mode(const CodingUnit &cu);

  void lfCccm(const CodingStructure &cs, const uint32_t ctuRsAddr);

  // transform tree (clause 7.3.8.8)
  void transform_tree(const CodingStructure &cs, Partitioner &pm, CUCtx &cuCtx);
  void cbf_comp(bool cbf, const CompArea &area, unsigned depth, bool prevCbf, BdpcmMode bdpcmMode);

  // mvd coding (clause 7.3.8.9)
  void mvd_coding(const CodingUnit &cu, Mv mvd, int amvr);
  void bvdCoding(const Mv &rMvd);
  void xWriteBvdContext(unsigned uiSymbol, unsigned ctxT, int offset, int param);
  // transform unit (clause 7.3.8.10)

  void transform_unit_last(const TransformUnit &tu, CUCtx &cuCtx, Partitioner &pm);
  void transform_unit_coef(const TransformUnit &tu, CUCtx &cuCtx, Partitioner &pm);
  void transform_unit_sign(const TransformUnit &tu, Partitioner &pm);
  void cu_qp_delta(const CodingUnit &cu, int predQP, const int8_t qp);
  void cu_chroma_qp_offset(const CodingUnit &cu);

  // residual coding (clause 7.3.8.11)
  void residual_coding_last(const TransformUnit &tu, CompID compID, CUCtx *cuCtx = nullptr);
  void residual_coding_coef(const TransformUnit &tu, CompID compID, CUCtx *cuCtx = nullptr);
  void residual_coding_sign(const TransformUnit &tu, CompID compID);
  void ts_flag(const TransformUnit &tu, CompID compID);
  void nst_idx(const TransformUnit &tu, CUCtx &cuCtx);
  void mts_idx(const TransformUnit &tu, CUCtx &cuCtx);
  void last_sig_coeff(CoeffCodingContext &cctx, const TransformUnit &tu, CompID compID);
  void residual_coding_subblock(CoeffCodingContext &cctx, const TCoeff *coeff, const uint64_t stateTransTable,
                                int &state);
  void residual_codingTS(const TransformUnit &tu, CompID compID);
  void residual_coding_subblockTS(CoeffCodingContext &cctx, const TCoeff *coeff, unsigned (&RiceBit)[8], int riceParam,
                                  bool ricePresentFlag);
  void joint_cb_cr(const TransformUnit &tu, const int cbfMask);

  void codeAlfCtuEnableFlags(CodingStructure &cs, ChannelType channel, const AlfParameters::AlfParamBase *alfParam);
  void codeAlfCtuEnableFlags(CodingStructure &cs, CompID compID, const AlfParameters::AlfParamBase *alfParam);
  void codeAlfCtuEnableFlag(CodingStructure &cs, uint32_t ctuRsAddr, const int compIdx,
                            const AlfParameters::AlfParamBase *alfParam);
  void codeAlfCtuFilterIndex(CodingStructure &cs, uint32_t ctuRsAddr, bool alfEnableLuma);

  void codeAlfCtuChromaAlternatives(const CodingStructure &cs, const AlfParameters::AlfParamBase &alfParam);
  void codeAlfCtuChromaAlternatives(const CodingStructure &cs, const CompID compID,
                                    const AlfParameters::AlfParamBase &alfParam);
  void codeAlfCtuChromaAlternative(const CodingStructure &cs, const uint32_t ctuRsAddr, const CompID compId);
  void codeAlfCtuChromaAlternative(const CodingStructure &cs, const uint32_t ctuRsAddr, const CompID compId,
                                   const AlfParameters::AlfParamBase &alfParam);
  void codeAlfCtuLumaAlternative(const CodingStructure &cs, const uint32_t ctuRsAddr);
  void codeAlfCtuLumaAlternative(const CodingStructure &cs, const uint32_t ctuRsAddr, const int numAlts);

  void codeCcAlfFilterControlIdc(uint8_t idcVal, CodingStructure &cs, const CompID compID, const int curIdx,
                                 const uint8_t *filterControlIdc, Position lumaPos, const int filterCount);

#if ENABLE_NNLF
  void writeNnlfUnifiedParameters(const CodingStructure &cs);
#endif

private:
  void codeAlfCtuAlternative(const CodingStructure &cs, const int altIdx, const int numAlts, const CompID compId);
  static const int &getAlfCtbMode(const CodingStructure &cs, const uint32_t ctuRsAddr, const CompID compId);
  static bool       isAlfEnabledInSpsAndSlice(const CodingStructure &cs, const CompID compId);

  void unary_max_symbol(unsigned symbol, unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol);
  void unary_max_eqprob(unsigned symbol, unsigned maxSymbol);
  void exp_golomb_eqprob(unsigned symbol, unsigned count);

  void xWriteTruncBinCode(uint32_t symbol, uint32_t numSymbols);
  void codeScanRotationModeFlag(const CodingUnit &cu, CompID compBegin);
  void xEncodePLTPredIndicator(const CodingUnit &cu, uint32_t maxPltSize, CompID compBegin);

public:
  CABACDataStore *m_CABACDataStore;

private:
  BinEncIf        &m_binEncoder;
  OutputBitstream *m_bitstream;
  Ctx              m_testCtx;
  EncCu           *m_encCu;
  ScanElement     *m_scanOrder;
};

class CABACEncoder
{
public:
  CABACEncoder()
    : m_CABACWriterStd(m_BinEncoderStd, nullptr)
    , m_CABACEstimatorStd(m_BitEstimatorStd, nullptr)
    , m_CABACWriter { &m_CABACWriterStd }
    , m_CABACEstimator { &m_CABACEstimatorStd }
    , m_CABACDataStore(nullptr)
  {
    m_CABACDataStore = new CABACDataStore;

    m_CABACWriterStd.m_CABACDataStore    = m_CABACDataStore;
    m_CABACEstimatorStd.m_CABACDataStore = m_CABACDataStore;

    for (int i = 0; i < to_underlying(BpmType::NUM) - 1; i++)
    {
      m_CABACWriter.at(i)->m_CABACDataStore    = m_CABACDataStore;
      m_CABACEstimator.at(i)->m_CABACDataStore = m_CABACDataStore;
    }

    m_BinEncoderStd.initBufferer(CABAC_SPATIAL_MAX_BINS, Ctx::NumberOfContexts, CABAC_SPATIAL_MAX_BINS_PER_CTX);
    m_BitEstimatorStd.initBufferer(CABAC_SPATIAL_MAX_BINS, Ctx::NumberOfContexts, CABAC_SPATIAL_MAX_BINS_PER_CTX);
  }

  virtual ~CABACEncoder()
  {
    if (m_CABACDataStore)
    {
      delete m_CABACDataStore;
    }
  }

  CABACWriter *getCABACWriter(const SPS *sps) { return m_CABACWriter[BpmType::STD]; }
  CABACWriter *getCABACEstimator(const SPS *sps) { return m_CABACEstimator[BpmType::STD]; }

private:
  BinEncoder_Std   m_BinEncoderStd;
  BitEstimator_Std m_BitEstimatorStd;
  CABACWriter      m_CABACWriterStd;
  CABACWriter      m_CABACEstimatorStd;

  EnumArray<CABACWriter *, BpmType> m_CABACWriter;
  EnumArray<CABACWriter *, BpmType> m_CABACEstimator;
  CABACDataStore                   *m_CABACDataStore;
};

//! \}

#endif //__CABACWRITER__
