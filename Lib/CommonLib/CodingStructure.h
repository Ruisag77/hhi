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

/** \file     CodingStructure.h
 *  \brief    A class managing the coding information for a specific image part
 */

#ifndef __CODINGSTRUCTURE__
#define __CODINGSTRUCTURE__

#include "Unit.h"
#include "Buffer.h"
#include "CommonDef.h"
#include "UnitPartitioner.h"
#include "AlfParameters.h"
#include "Slice.h"
#include <vector>

struct Picture;

enum PictureType
{
  PIC_RECONSTRUCTION = 0,
  PIC_ORIGINAL,
  PIC_TRUE_ORIGINAL,
  PIC_FILTERED_ORIGINAL,
  PIC_FILTERED_ORIGINAL_FG,
  PIC_PREDICTION,
  PIC_RESIDUAL,
  PIC_ORG_RESI,
  PIC_RECON_WRAP,
  PIC_ORIGINAL_INPUT,
  PIC_TRUE_ORIGINAL_INPUT,
  PIC_FILTERED_ORIGINAL_INPUT,
#if JVET_Z0120_SII_SEI_PROCESSING
  PIC_YUV_POST_REC,
#endif
#if ENABLE_NNLF
  PIC_BS_MAP,
  PIC_PREDICTION_CUSTOM,
  PIC_REC_BEFORE_DBF,
  PIC_BLOCK_PRED_MODE,
  PIC_BLOCK_QP,
#endif
  NUM_PIC_TYPES
};
extern XuPool g_xuPool;

// ---------------------------------------------------------------------------
// coding structure
// ---------------------------------------------------------------------------

class CodingStructure
{
public:
  UnitArea area;
  UnitArea m_maxArea;

  Picture         *picture;
  CodingStructure *parent;
  Slice           *slice;

  std::array<UnitScale, MAX_NUM_COMP> unitScale;

  int                         baseQP;
  EnumArray<int, ChannelType> prevQP;
  EnumArray<int, ChannelType> currQP;

  int                  chromaQpAdj;
  const SPS           *sps;
  const PPS           *pps;
  PicHeader           *picHeader;
  APS                 *alfApss[AlfParameters::ALF_CTB_MAX_NUM_APS];
  APS                 *lmcsAps;
  APS                 *scalinglistAps;
  const VPS           *vps;
  const PreCalcValues *pcv;

  CodingStructure(XuPool &);

  void create(const UnitArea &_unit, const bool isTopLayer, const bool isPLTused,
              const bool isTempPartPredUsed = false);
  void create(const ChromaFormat &_chromaFormat, const Area &_area, const bool isTopLayer, const bool isPLTused,
              const bool isTempPartPredUsed = false);

  void destroy();
  void releaseIntermediateData();

  void rebindPicBufs();

  void createCoeffs(const bool isPLTused);
  void destroyCoeffs();

  void allocateVectorsAtPicLevel();

  // ---------------------------------------------------------------------------
  // global accessors
  // ---------------------------------------------------------------------------

  bool isDecomp(const Position &pos, const ChannelType _chType) const;
  bool isDecomp(const Position &pos, const ChannelType _chType);
  void setDecomp(const CompArea &area, const bool _isCoded = true);
  void setDecomp(const UnitArea &area, const bool _isCoded = true);

  const CodingUnit    *getCU(const Position &pos, const ChannelType _chType) const;
  const TransformUnit *getTU(const Position &pos, const ChannelType _chType) const;

  CodingUnit    *getCU(const Position &pos, const ChannelType _chType);
  CodingUnit    *getLumaCU(const Position &pos);
  TransformUnit *getTU(const Position &pos, const ChannelType _chType);

  const CodingUnit    *getCU(const ChannelType _chType) const { return getCU(area.block(_chType).pos(), _chType); }
  const TransformUnit *getTU(const ChannelType _chType) const { return getTU(area.block(_chType).pos(), _chType); }

  CodingUnit    *getCU(const ChannelType _chType) { return getCU(area.block(_chType).pos(), _chType); }
  TransformUnit *getTU(const ChannelType _chType) { return getTU(area.block(_chType).pos(), _chType); }

  const CodingUnit    *getCURestricted(const Position &pos, const Position curPos, const unsigned curSliceIdx,
                                       const unsigned curTileIdx, const ChannelType _chType) const;
  const CodingUnit    *getCURestricted(const Position &pos, const CodingUnit &curCu, const ChannelType _chType) const;
  const TransformUnit *getTURestricted(const Position &pos, const TransformUnit &curTu,
                                       const ChannelType _chType) const;

  CodingUnit    &addCU(const UnitArea &unit, const ChannelType _chType);
  TransformUnit &addTU(const UnitArea &unit, const ChannelType _chType);
  void           addEmptyTUs(Partitioner &partitioner);

  CUTraverser traverseCUs(const UnitArea &_unit, const ChannelType _chType);
  TUTraverser traverseTUs(const UnitArea &_unit, const ChannelType _chType);

  cCUTraverser traverseCUs(const UnitArea &_unit, const ChannelType _chType) const;
  cTUTraverser traverseTUs(const UnitArea &_unit, const ChannelType _chType) const;
  // ---------------------------------------------------------------------------
  // encoding search utilities
  // ---------------------------------------------------------------------------

  double     cost;
  double     costDbOffset;
  double     lumaCost;
  uint64_t   fracBits;
  Distortion dist;
  Distortion interHad;
  int        etmType;
  int        etmOpts;

  void initStructData(const int &QP = MAX_INT, const bool &skipMotBuf = false, const UnitArea *area = nullptr);
  void initSubStructure(CodingStructure &cs, const ChannelType chType, const UnitArea &subArea, const bool &isTuEnc);
  void compactResize(const UnitArea &area);

  void copyStructure(const CodingStructure &cs, const ChannelType chType, const bool copyTUs = false,
                     const bool copyRecoBuffer = false);
  void useSubStructure(const CodingStructure &cs, const ChannelType chType, const UnitArea &subArea, const bool cpyPred,
                       const bool cpyReco, const bool cpyOrgResi, const bool cpyResi, const bool updateCost);
  void useSubStructure(const CodingStructure &cs, const ChannelType chType, const bool cpyPred, const bool cpyReco,
                       const bool cpyOrgResi, const bool cpyResi, const bool updateCost)
  {
    useSubStructure(cs, chType, cs.area, cpyPred, cpyReco, cpyOrgResi, cpyResi, updateCost);
  }

  void clearTUs();
  void clearCUs();

private:
  void createInternals(const UnitArea &_unit, const bool isTopLayer, const bool isPLTused,
                       const bool isTempPartPredUsed);

public:
  std::vector<CodingUnit *>    cus;
  std::vector<TransformUnit *> tus;

  LutMotionCand motionLut;

  void addMiToLut(static_vector<MotionInfo, MAX_NUM_HMVP_CANDS> &lut, const MotionInfo &mi);
  void addAffMiToLut(static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS> *lutSet, const AffineMotionInfo addMi[2],
                     int refIdx[2]);
  void addAffInheritToLut(static_vector<AffineInheritInfo, MAX_NUM_AFF_INHERIT_HMVP_CANDS> &lut,
                          const AffineInheritInfo                                          &mi);

  LutCCP ccpLut;

  template<class T> void addCCPToLut(static_vector<T, MAX_NUM_HCCP_CANDS> &lut, const T &model, int reusePos);
  template<class T> void getOneModelFromCCPLut(const static_vector<T, MAX_NUM_HCCP_CANDS> &lut, T &model, int pos);

  int                   *m_eipIdxBuf;
  std::vector<EipModels> m_eipModelLUT;
  LutEIP                 eipLut;
  template<class T> void addEipToLut(static_vector<T, MAX_NUM_HEIP_CANDS> &lut, const T &model, int reusePos);
  template<class T> void getOneModelFromEipLut(const static_vector<T, MAX_NUM_HEIP_CANDS> &lut, T &model, int pos);

  PLTBuf prevPLT;

  void resetPrevPLT(PLTBuf &prevPLT);
  void reorderPrevPLT(PLTBuf &prevPLT, uint8_t curPLTSize[MAX_NUM_CHANNEL_TYPE], Pel curPLT[MAX_NUM_COMP][MAXPLTSIZE],
                      bool reuseflag[MAX_NUM_CHANNEL_TYPE][MAXPLTPREDSIZE], uint32_t compBegin, uint32_t numComp,
                      bool jointPLT);
  void setPrevPLT(PLTBuf predictor);
  void storePrevPLT(PLTBuf &predictor);

private:
  // needed for TU encoding
  bool m_isTuEnc;

  EnumArray<CodingUnit **, ChannelType> m_cuArr;
  EnumArray<bool *, ChannelType>        m_isDecomp;

  unsigned m_numCUs;
  unsigned m_numTUs;

  CuPool &m_cuPool;
  TuPool &m_tuPool;

  std::vector<SAOBlkParam> m_sao;

  PelStorage m_pred;
  PelStorage m_resi;
  PelStorage m_reco;
  PelStorage m_orgr;
#if ENABLE_NNLF
  PelStorage m_predCustom;
#endif

  TCoeff                        *m_coeffs[MAX_NUM_COMP];
  uint8_t                       *m_signsPredArea[MAX_NUM_COMP];
  EnumArray<Pel *, ChannelType>  m_pltIdx;
  EnumArray<bool *, ChannelType> m_runType;
  int                            m_offsets[MAX_NUM_COMP];

  MotionInfo *m_motionBuf;
  SplitPred  *m_currQtDepthBuf;

public:
  bool resetIBCBuffer;

  MotionBuf getMotionBuf(const Area &_area);
  MotionBuf getMotionBuf(const UnitArea &_area) { return getMotionBuf(_area.Y()); }
  MotionBuf getMotionBuf() { return getMotionBuf(area.Y()); }

  const CMotionBuf getMotionBuf(const Area &_area) const;
  const CMotionBuf getMotionBuf(const UnitArea &_area) const { return getMotionBuf(_area.Y()); }
  const CMotionBuf getMotionBuf() const { return getMotionBuf(area.Y()); }

  MotionInfo       &getMotionInfo(const Position &pos);
  const MotionInfo &getMotionInfo(const Position &pos) const;

  void       setSplitPred();
  QTDepthBuf getQtDepthBuf(const Area &_area);
  QTDepthBuf getQTDepthBuf() { return getQtDepthBuf(area.Y()); }
  SplitPred &getQtDepthInfo(const Position &pos);

  EipModelIdxBuf        getEipIdxBuf(const Area &bufArea);
  EipModelIdxBuf        getEipIdxBuf(const UnitArea &bufArea) { return getEipIdxBuf(bufArea.Y()); }
  EipModelIdxBuf        getEipIdxBuf() { return getEipIdxBuf(area.Y()); }
  const CEipModelIdxBuf getEipIdxBuf(const Area &bufArea) const;
  const CEipModelIdxBuf getEipIdxBuf(const UnitArea &bufArea) const { return getEipIdxBuf(bufArea.Y()); }
  const CEipModelIdxBuf getEipIdxBuf() const { return getEipIdxBuf(area.Y()); }

  int       &getEipIdxInfo(const Position &pos);
  const int &getEipIdxInfo(const Position &pos) const;

public:
  // ---------------------------------------------------------------------------
  // temporary (shadowed) data accessors
  // ---------------------------------------------------------------------------
  PelBuf            getPredBuf(const CompArea &blk);
  const CPelBuf     getPredBuf(const CompArea &blk) const;
  PelUnitBuf        getPredBuf(const UnitArea &unit);
  const CPelUnitBuf getPredBuf(const UnitArea &unit) const;

  PelBuf            getResiBuf(const CompArea &blk);
  const CPelBuf     getResiBuf(const CompArea &blk) const;
  PelUnitBuf        getResiBuf(const UnitArea &unit);
  const CPelUnitBuf getResiBuf(const UnitArea &unit) const;

  PelBuf            getRecoBuf(const CompArea &blk);
  const CPelBuf     getRecoBuf(const CompArea &blk) const;
  PelUnitBuf        getRecoBuf(const UnitArea &unit);
  const CPelUnitBuf getRecoBuf(const UnitArea &unit) const;
  PelUnitBuf       &getRecoBufRef() { return m_reco; }

  PelBuf            getOrgResiBuf(const CompArea &blk);
  const CPelBuf     getOrgResiBuf(const CompArea &blk) const;
  PelUnitBuf        getOrgResiBuf(const UnitArea &unit);
  const CPelUnitBuf getOrgResiBuf(const UnitArea &unit) const;

  PelBuf            getOrgBuf(const CompArea &blk);
  const CPelBuf     getOrgBuf(const CompArea &blk) const;
  PelUnitBuf        getOrgBuf(const UnitArea &unit);
  const CPelUnitBuf getOrgBuf(const UnitArea &unit) const;

  PelBuf            getOrgBuf(const CompID &compID);
  const CPelBuf     getOrgBuf(const CompID &compID) const;
  PelUnitBuf        getOrgBuf();
  const CPelUnitBuf getOrgBuf() const;
  PelUnitBuf        getTrueOrgBuf();
  const CPelUnitBuf getTrueOrgBuf() const;

  // pred buffer
  PelBuf            getPredBuf(const CompID &compID) { return m_pred.get(compID); }
  const CPelBuf     getPredBuf(const CompID &compID) const { return m_pred.get(compID); }
  PelUnitBuf        getPredBuf() { return m_pred; }
  const CPelUnitBuf getPredBuf() const { return m_pred; }

  // resi buffer
  PelBuf            getResiBuf(const CompID compID) { return m_resi.get(compID); }
  const CPelBuf     getResiBuf(const CompID compID) const { return m_resi.get(compID); }
  PelUnitBuf        getResiBuf() { return m_resi; }
  const CPelUnitBuf getResiBuf() const { return m_resi; }

  // org-resi buffer
  PelBuf            getOrgResiBuf(const CompID &compID) { return m_orgr.get(compID); }
  const CPelBuf     getOrgResiBuf(const CompID &compID) const { return m_orgr.get(compID); }
  PelUnitBuf        getOrgResiBuf() { return m_orgr; }
  const CPelUnitBuf getOrgResiBuf() const { return m_orgr; }

  // reco buffer
  PelBuf            getRecoBuf(const CompID compID) { return m_reco.get(compID); }
  const CPelBuf     getRecoBuf(const CompID compID) const { return m_reco.get(compID); }
  PelUnitBuf        getRecoBuf() { return m_reco; }
  const CPelUnitBuf getRecoBuf() const { return m_reco; }

#if ENABLE_NNLF
  PelBuf            getPredBufCustom(const CompArea &blk);
  const CPelBuf     getPredBufCustom(const CompArea &blk) const;
  PelUnitBuf        getPredBufCustom(const UnitArea &unit);
  const CPelUnitBuf getPredBufCustom(const UnitArea &unit) const;
#endif

private:
  inline PelBuf            getBuf(const CompArea &blk, const PictureType &type);
  inline const CPelBuf     getBuf(const CompArea &blk, const PictureType &type) const;
  inline PelUnitBuf        getBuf(const UnitArea &unit, const PictureType &type);
  inline const CPelUnitBuf getBuf(const UnitArea &unit, const PictureType &type) const;
};

static inline uint32_t getNumberValidTBlocks(const PreCalcValues &pcv)
{
  return !isChromaEnabled(pcv.chrFormat) ? 1 : (pcv.multiBlock422 ? MAX_NUM_TBLOCKS : MAX_NUM_COMP);
}

template<class T>
void CodingStructure::addCCPToLut(static_vector<T, MAX_NUM_HCCP_CANDS> &lut, const T &model, int reusePos)
{
  int currCnt = (int)lut.size();

  int erasePos = 0;

  if (reusePos == -1)
  {
    for (int j = 0; j < currCnt; j++)
    {
      if (lut[currCnt - j - 1] == model)
      {
        reusePos = j;
        break;
      }
    }
  }
  if (reusePos != -1)
  {
    erasePos = currCnt - 1 - reusePos;   // reverse the order
  }
  if (reusePos != -1 || currCnt == lut.capacity())
  {
    lut.erase(lut.begin() + erasePos);
  }
  lut.push_back(model);
}

template<class T>
void CodingStructure::getOneModelFromCCPLut(const static_vector<T, MAX_NUM_HCCP_CANDS> &lut, T &model, int pos)
{
  size_t currCnt = lut.size();
  CHECK(pos >= currCnt, "Invalid entry in CCP LUT");
  model = lut[currCnt - pos - 1];
}

template<class T>
void CodingStructure::addEipToLut(static_vector<T, MAX_NUM_HEIP_CANDS> &lut, const T &model, int reusePos)
{
  int currCnt = (int)lut.size();

  int erasePos = 0;

  if (reusePos == -1)
  {
    for (int j = 0; j < currCnt; j++)
    {
      if (lut[currCnt - j - 1] == model)
      {
        reusePos = j;
        break;
      }
    }
  }
  if (reusePos != -1)
  {
    erasePos = currCnt - 1 - reusePos;   // reverse the order
  }
  if (reusePos != -1 || currCnt == lut.capacity())
  {
    lut.erase(lut.begin() + erasePos);
  }
  lut.push_back(model);
}

template<class T>
void CodingStructure::getOneModelFromEipLut(const static_vector<T, MAX_NUM_HEIP_CANDS> &lut, T &model, int pos)
{
  size_t currCnt = lut.size();
  CHECK(pos >= currCnt, "Invalid entry in EIP LUT");
  model = lut[currCnt - pos - 1];
}

#endif
