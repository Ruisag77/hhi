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

/** \file     CABACReader.h
 *  \brief    Reader for low level syntax
 */

#ifndef __CABACREADER__
#define __CABACREADER__

#include "BinDecoder.h"

#include "CommonLib/ContextModelling.h"
#include "CommonLib/MotionInfo.h"
#include "CommonLib/UnitPartitioner.h"

class CABACReader : DeriveCtx
{
public:
  CABACReader(BinDecoderBase &binDecoder, CABACDataStore *cabacDataStore)
    : m_CABACDataStore(cabacDataStore)
    , m_binDecoder(binDecoder)
    , m_bitstream(nullptr)
  {}
  virtual ~CABACReader() {}

public:
  void initCtxModels(Slice &slice);
  void initBitstream(InputBitstream *bitstream)
  {
    m_bitstream = bitstream;
    m_binDecoder.init(m_bitstream);
  }
  const Ctx &getCtx() const { return m_binDecoder.getCtx(); }
  Ctx       &getCtx() { return m_binDecoder.getCtx(); }

  void                  setBinBufferActive(bool b) { m_binDecoder.setBinBufferActive(b); }
  void                  setBinBuffer(BinStoreVector *bb) { m_binDecoder.setBinBuffer(bb); }
  const BinStoreVector *getBinBuffer() const { return m_binDecoder.getBinBuffer(); }
  void                  updateCtxs(BinStoreVector *bb) { m_binDecoder.updateCtxs(bb); }

public:
  // slice segment data (clause 7.3.8.1)
  bool terminating_bit();
  void remaining_bytes(bool noTrailingBytesExpected);

  // coding tree unit (clause 7.3.8.2)
  void coding_tree_unit(CodingStructure &cs, const UnitArea &area, EnumArray<int, ChannelType> &qps,
                        unsigned ctuRsAddr);

  // sao (clause 7.3.8.3)
  void sao(CodingStructure &cs, unsigned ctuRsAddr);

  void bif(const CompID compID, CodingStructure &cs);
  void bif(const CompID compID, CodingStructure &cs, unsigned ctuRsAddr);
  void ccSaoControlIdc(CodingStructure &cs, const CompID compID, const int curIdx, uint8_t *controlIdc,
                       Position lumaPos, int setNum);

#if ENABLE_NNLF
  void readNnlfUnifiedParameters(CodingStructure &cs);
#endif
  void    readAlfCtuFilterIndex(CodingStructure &cs, unsigned ctuRsAddr);
  uint8_t readAlfCtuAlternative(const CodingStructure &cs, const int apsId, const CompID compId);

  void ccAlfFilterControlIdc(CodingStructure &cs, const CompID compID, const int curIdx, uint8_t *filterControlIdc,
                             Position lumaPos, int filterCount);

  // coding (quad)tree (clause 7.3.8.4)
  void      coding_tree(CodingStructure &cs, Partitioner &pm, CUCtx &cuCtx, Partitioner *pPartitionerChroma = nullptr,
                        CUCtx *pCuCtxChroma = nullptr);
  PartSplit split_cu_mode(CodingStructure &cs, Partitioner &pm);

  // coding unit (clause 7.3.8.5)
  void coding_unit(CodingUnit &cu, Partitioner &pm, CUCtx &cuCtx);
  void cu_skip_flag(CodingUnit &cu);
  void pred_mode(CodingUnit &cu);
  void bdpcm_mode(CodingUnit &cu, const CompID compID);
  void cu_pred_data(CodingUnit &cu);
  void cu_bcw_flag(CodingUnit &cu);
  void obmc_flag(CodingUnit &cu);
  void extend_ref_line(CodingUnit &cu);
  void sgpm_flag(CodingUnit &cu);
  void intra_luma_pred_mode(CodingUnit &cu);
  bool intra_chroma_lmc_mode(CodingUnit &cu);
  void intra_chroma_pred_mode(CodingUnit &cu);
  void cclmDelta(int8_t &delta);
  void cclmDeltaSlope(CodingUnit &cu);
  void cccmFlag(CodingUnit &cu);
  void nonLocalCCPIndex(CodingUnit &cu);
  void decoderDerivedCcpModes(CodingUnit &cu);
  void ccFilterFlag(CodingUnit &cu);
  void cu_residual(CodingUnit &cu, Partitioner &pm, CUCtx &cuCtx);
  void rqt_root_cbf(CodingUnit &cu);
  void sbt_mode(CodingUnit &cu);
  void end_of_ctu(CodingUnit &cu, CUCtx &cuCtx);
  void mip_flag(CodingUnit &cu);
  void mip_pred_modes(CodingUnit &cu);
  void mip_pred_mode(CodingUnit &cu);
  void cu_palette_info(CodingUnit &cu, CompID compBegin, uint32_t numComp, CUCtx &cuCtx);
  void cuPaletteSubblockInfo(CodingUnit &cu, CompID compBegin, uint32_t numComp, int subSetId, uint32_t &prevRunPos,
                             unsigned &prevRunType);
  void planarDir(CodingUnit &cu);
  void dimd_flag(CodingUnit &cu);
  void dimdChromaFlag(CodingUnit &cu);
  void timd_flag(CodingUnit &cu);
  void timd_merge_flag(CodingUnit &cu);
  void timd_sad_flag(CodingUnit &cu);
  void obic_flag(CodingUnit &cu);
  void eip_flag(CodingUnit &cu);
  void cu_lic_flag(CodingUnit &cu);

  // prediction unit (clause 7.3.8.6)
  void prediction_unit(CodingUnit &cu);
  void merge_flag(CodingUnit &cu);
  void merge_data(CodingUnit &cu);
  void affine_flag(CodingUnit &cu);
  void subblock_merge_flag(CodingUnit &cu);
  void merge_idx(CodingUnit &cu);
  void mmvd_merge_idx(CodingUnit &cu);
  void affine_mmvd_data(CodingUnit &cu);
  void bm_merge_flag(CodingUnit &cu);

  void geo_mmvd_idx(CodingUnit &cu, RefPicList eRefPicList);
  void geo_merge_idx(CodingUnit &cu);
  void geo_merge_idx1(CodingUnit &cu, bool isIntra0, bool isIntra1);
  void geo_adaptive_blending_idx(CodingUnit &cu);
  void imv_mode(CodingUnit &cu);
  void affine_amvr_mode(CodingUnit &cu);
  void inter_pred_idc(CodingUnit &cu);
  void ref_idx(CodingUnit &cu, RefPicList eRefList);
  void mvp_flag(CodingUnit &cu, RefPicList eRefList);
  void ciip_flag(CodingUnit &cu, bool usageIsInferred);
  void smvd_mode(CodingUnit &cu);

  void lfCccm(CodingStructure &cs, const uint32_t ctuRsAddr);

  // transform tree (clause 7.3.8.8)
  void transform_tree(CodingStructure &cs, Partitioner &pm, CUCtx &cuCtx);
  bool cbf_comp(const CompArea &area, unsigned depth, bool prevCbf, BdpcmMode bdpcmMode);

  // mvd coding (clause 7.3.8.9)
  void     mvd_coding(Mv &rMvd);
  void     bvdCoding(Mv &rMvd);
  unsigned xReadBvdContext(unsigned ctxT, int offset, int param);

  // transform unit (clause 7.3.8.10)
  void transform_unit_last(TransformUnit &tu, CUCtx &cuCtx, Partitioner &pm);
  void transform_unit_coef(TransformUnit &tu, CUCtx &cuCtx, Partitioner &pm);
  void transform_unit_sign(TransformUnit &tu, Partitioner &pm);
  void cu_qp_delta(CodingUnit &cu, int predQP, int8_t &qp);
  void cu_chroma_qp_offset(CodingUnit &cu);

  // residual coding (clause 7.3.8.11)
  void residual_coding_last(TransformUnit &tu, CompID compID, CUCtx &cuCtx);
  void residual_coding_coef(TransformUnit &tu, CompID compID, CUCtx &cuCtx);
  void residual_coding_sign(TransformUnit &tu, CompID compID);
  void ts_flag(TransformUnit &tu, CompID compID);
  void nst_idx(TransformUnit &tu, CUCtx &cuCtx);
  void mts_idx(TransformUnit &tu, CUCtx &cuCtx);
  int  last_sig_coeff(CoeffCodingContext &cctx, TransformUnit &tu, CompID compID);
  void residual_coding_subblock(CoeffCodingContext &cctx, TCoeff *coeff, const uint64_t stateTransTable, int &state);
  void residual_codingTS(TransformUnit &tu, CompID compID);
  void residual_coding_subblockTS(CoeffCodingContext &cctx, TCoeff *coeff, int riceParam);
  void joint_cb_cr(TransformUnit &tu, const int cbfMask);

private:
  unsigned unary_max_symbol(unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol);
  unsigned unary_max_eqprob(unsigned maxSymbol);
  unsigned exp_golomb_eqprob(unsigned count);
  unsigned get_num_bits_read() { return m_binDecoder.getNumBitsRead(); }

  void xReadTruncBinCode(uint32_t &symbol, uint32_t numSymbols);
  void parseScanRotationModeFlag(CodingUnit &cu, CompID compBegin);
  void xDecodePLTPredIndicator(CodingUnit &cu, uint32_t maxPLTSize, CompID compBegin);
  void xAdjustPLTIndex(CodingUnit &cu, Pel curLevel, uint32_t idx, PelBuf &paletteIdx, PLTtypeBuf &paletteRunType,
                       int maxSymbol, CompID compBegin);

public:
  CABACDataStore *m_CABACDataStore;

private:
  BinDecoderBase &m_binDecoder;
  InputBitstream *m_bitstream;
  ScanElement    *m_scanOrder;
};

class CABACDecoder
{
public:
  CABACDecoder()
    : m_CABACReaderStd(m_BinDecoderStd, nullptr)
    , m_CABACReader { &m_CABACReaderStd }
    , m_CABACDataStore(nullptr)
  {
    m_CABACDataStore = new CABACDataStore;

    m_CABACReaderStd.m_CABACDataStore = m_CABACDataStore;

    m_BinDecoderStd.initBufferer(CABAC_SPATIAL_MAX_BINS, Ctx::NumberOfContexts, CABAC_SPATIAL_MAX_BINS_PER_CTX);

    for (int i = 0; i < to_underlying(BpmType::NUM) - 1; i++)
    {
      m_CABACReader.at(i)->m_CABACDataStore = m_CABACDataStore;
    }
  }

  virtual ~CABACDecoder()
  {
    if (m_CABACDataStore)
    {
      delete m_CABACDataStore;
    }
  }

  CABACReader *getCABACReader(BpmType id) { return m_CABACReader[id]; }

private:
  BinDecoder_Std m_BinDecoderStd;
  CABACReader    m_CABACReaderStd;

  EnumArray<CABACReader *, BpmType> m_CABACReader;
  CABACDataStore                   *m_CABACDataStore;
};

#endif
