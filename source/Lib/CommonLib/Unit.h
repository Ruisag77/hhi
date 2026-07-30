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

/** \file     Unit.h
 *  \brief    defines unit as a set of blocks and basic unit types (coding, prediction, transform)
 */

#ifndef __UNIT__
#define __UNIT__

#include "CommonDef.h"
#include "Common.h"
#include "Mv.h"
#include "MotionInfo.h"
#include "ChromaFormat.h"

// ---------------------------------------------------------------------------
// tools
// ---------------------------------------------------------------------------
struct PLTBuf
{
  uint8_t curPLTSize[MAX_NUM_CHANNEL_TYPE];
  Pel     curPLT[MAX_NUM_COMP][MAXPLTPREDSIZE];
};
inline Position recalcPosition(const ChromaFormat _cf, const CompID srcCId, const CompID dstCId, const Position &pos)
{
  if (toChannelType(srcCId) == toChannelType(dstCId))
  {
    return pos;
  }
  else if (isLuma(srcCId) && isChroma(dstCId))
  {
    return Position(pos.x >> getComponentScaleX(dstCId, _cf), pos.y >> getComponentScaleY(dstCId, _cf));
  }
  else
  {
    return Position(pos.x << getComponentScaleX(srcCId, _cf), pos.y << getComponentScaleY(srcCId, _cf));
  }
}

inline Position recalcPosition(const ChromaFormat _cf, const ChannelType srcCHt, const ChannelType dstCHt,
                               const Position &pos)
{
  if (srcCHt == dstCHt)
  {
    return pos;
  }
  else if (isLuma(srcCHt) && isChroma(dstCHt))
  {
    return Position(pos.x >> getChannelTypeScaleX(dstCHt, _cf), pos.y >> getChannelTypeScaleY(dstCHt, _cf));
  }
  else
  {
    return Position(pos.x << getChannelTypeScaleX(srcCHt, _cf), pos.y << getChannelTypeScaleY(srcCHt, _cf));
  }
}

inline Size recalcSize(const ChromaFormat _cf, const CompID srcCId, const CompID dstCId, const Size &size)
{
  if (toChannelType(srcCId) == toChannelType(dstCId))
  {
    return size;
  }
  else if (isLuma(srcCId) && isChroma(dstCId))
  {
    return Size(size.width >> getComponentScaleX(dstCId, _cf), size.height >> getComponentScaleY(dstCId, _cf));
  }
  else
  {
    return Size(size.width << getComponentScaleX(srcCId, _cf), size.height << getComponentScaleY(srcCId, _cf));
  }
}

inline Size recalcSize(const ChromaFormat _cf, const ChannelType srcCHt, const ChannelType dstCHt, const Size &size)
{
  if (srcCHt == dstCHt)
  {
    return size;
  }
  else if (isLuma(srcCHt) && isChroma(dstCHt))
  {
    return Size(size.width >> getChannelTypeScaleX(dstCHt, _cf), size.height >> getChannelTypeScaleY(dstCHt, _cf));
  }
  else
  {
    return Size(size.width << getChannelTypeScaleX(srcCHt, _cf), size.height << getChannelTypeScaleY(srcCHt, _cf));
  }
}

// ---------------------------------------------------------------------------
// block definition
// ---------------------------------------------------------------------------

struct CompArea : public Area
{
  CompArea() : Area(), chromaFormat(ChromaFormat::UNDEFINED), compID(MAX_NUM_TBLOCKS) {}
  CompArea(const CompID _compID, const ChromaFormat _cf, const Area &_area, const bool isLuma = false)
    : Area(_area)
    , chromaFormat(_cf)
    , compID(_compID)
  {
    if (isLuma)
    {
      xRecalcLumaToChroma();
    }
  }
  CompArea(const CompID _compID, const ChromaFormat _cf, const Position &_pos, const Size &_size,
           const bool isLuma = false)
    : Area(_pos, _size)
    , chromaFormat(_cf)
    , compID(_compID)
  {
    if (isLuma)
    {
      xRecalcLumaToChroma();
    }
  }
  CompArea(const CompID _compID, const ChromaFormat _cf, const uint32_t _x, const uint32_t _y, const uint32_t _w,
           const uint32_t _h, const bool isLuma = false)
    : Area(_x, _y, _w, _h)
    , chromaFormat(_cf)
    , compID(_compID)
  {
    if (isLuma)
    {
      xRecalcLumaToChroma();
    }
  }

  ChromaFormat chromaFormat;
  CompID       compID;

  Position chromaPos() const;
  Position lumaPos() const;

  Size chromaSize() const;
  Size lumaSize() const;

  Position compPos(const CompID compID) const;
  Position chanPos(const ChannelType chType) const;

  Position topLeftComp(const CompID _compID) const { return recalcPosition(chromaFormat, compID, _compID, *this); }
  Position topRightComp(const CompID _compID) const
  {
    return recalcPosition(chromaFormat, compID, _compID, { (PosType)(x + width - 1), y });
  }
  Position bottomLeftComp(const CompID _compID) const
  {
    return recalcPosition(chromaFormat, compID, _compID, { x, (PosType)(y + height - 1) });
  }
  Position bottomRightComp(const CompID _compID) const
  {
    return recalcPosition(chromaFormat, compID, _compID, { (PosType)(x + width - 1), (PosType)(y + height - 1) });
  }

  bool valid() const
  {
    return chromaFormat != ChromaFormat::UNDEFINED && compID < MAX_NUM_TBLOCKS && width != 0 && height != 0;
  }

  const bool operator==(const CompArea &other) const
  {
    if (chromaFormat != other.chromaFormat)
    {
      return false;
    }
    if (compID != other.compID)
    {
      return false;
    }

    return Position::operator==(other) && Size::operator==(other);
  }

  const bool operator!=(const CompArea &other) const { return !(operator==(other)); }

#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS
  void resizeTo(const Size &newSize) { Size::resizeTo(newSize); }
#endif
  void repositionTo(const Position &newPos) { Position::repositionTo(newPos); }
  void positionRelativeTo(const CompArea &origCompArea) { Position::relativeTo(origCompArea); }

private:
  void xRecalcLumaToChroma();
};

inline CompArea clipArea(const CompArea &compArea, const Area &boundingBox)
{
  return CompArea(compArea.compID, compArea.chromaFormat, clipArea((const Area &)compArea, boundingBox));
}

// ---------------------------------------------------------------------------
// unit definition
// ---------------------------------------------------------------------------

using UnitBlocksType = static_vector<CompArea, MAX_NUM_TBLOCKS>;

struct UnitArea
{
  ChromaFormat   chromaFormat;
  UnitBlocksType blocks;

  UnitArea() : chromaFormat(ChromaFormat::UNDEFINED) {}
  UnitArea(const ChromaFormat _chromaFormat);
  UnitArea(const ChromaFormat _chromaFormat, const Area &area);
  UnitArea(const ChromaFormat _chromaFormat, const CompArea &blkY);
  UnitArea(const ChromaFormat _chromaFormat, CompArea &&blkY);
  UnitArea(const ChromaFormat _chromaFormat, const CompArea &blkY, const CompArea &blkCb, const CompArea &blkCr);
  UnitArea(const ChromaFormat _chromaFormat, CompArea &&blkY, CompArea &&blkCb, CompArea &&blkCr);

  CompArea       &Y() { return blocks[COMP_Y]; }
  const CompArea &Y() const { return blocks[COMP_Y]; }
  CompArea       &Cb() { return blocks[COMP_Cb]; }
  const CompArea &Cb() const { return blocks[COMP_Cb]; }
  CompArea       &Cr() { return blocks[COMP_Cr]; }
  const CompArea &Cr() const { return blocks[COMP_Cr]; }

  CompArea       &block(const ChannelType ct) { return blocks[getFirstComponentOfChannel(ct)]; }
  CompArea const &block(const ChannelType ct) const { return blocks[getFirstComponentOfChannel(ct)]; }

  CompArea       &block(const CompID comp) { return blocks[comp]; }
  const CompArea &block(const CompID comp) const { return blocks[comp]; }

  bool contains(const UnitArea &other) const;
  bool contains(const UnitArea &other, const ChannelType chType) const;

  CompArea       &operator[](const int n) { return blocks[n]; }
  const CompArea &operator[](const int n) const { return blocks[n]; }

  const bool operator==(const UnitArea &other) const
  {
    if (chromaFormat != other.chromaFormat)
    {
      return false;
    }
    if (blocks.size() != other.blocks.size())
    {
      return false;
    }

    for (uint32_t i = 0; i < blocks.size(); i++)
    {
      if (blocks[i] != other.blocks[i])
      {
        return false;
      }
    }

    return true;
  }

#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS
  void resizeTo(const UnitArea &unit);
#endif
  void repositionTo(const UnitArea &unit);

  const bool operator!=(const UnitArea &other) const { return !(*this == other); }

  const Position &lumaPos() const { return Y(); }
  const Size     &lumaSize() const { return Y(); }

  const Position &chromaPos() const { return Cb(); }
  const Size     &chromaSize() const { return Cb(); }

  UnitArea singleChan(const ChannelType chType) const;

  const SizeType lwidth() const { return Y().width; }  /*! luma width  */
  const SizeType lheight() const { return Y().height; } /*! luma height */

  const PosType lx() const { return Y().x; }           /*! luma x-pos */
  const PosType ly() const { return Y().y; }           /*! luma y-pos */

  bool valid() const { return chromaFormat != ChromaFormat::UNDEFINED && blocks.size() > 0; }
};

inline UnitArea clipArea(const UnitArea &area, const UnitArea &boundingBox)
{
  UnitArea ret(area.chromaFormat);

  for (uint32_t i = 0; i < area.blocks.size(); i++)
  {
    ret.blocks.push_back(clipArea(area.blocks[i], boundingBox.blocks[i]));
  }

  return ret;
}

struct UnitAreaRelative : public UnitArea
{
  UnitAreaRelative(const UnitArea &origUnit, const UnitArea &unit)
  {
    *((UnitArea *)this) = unit;
    for (uint32_t i = 0; i < blocks.size(); i++)
    {
      blocks[i].positionRelativeTo(origUnit.blocks[i]);
    }
  }
};

struct SPS;
struct VPS;
struct DCI;
struct PPS;
struct Slice;

// ---------------------------------------------------------------------------
// coding unit
// ---------------------------------------------------------------------------

#include "Buffer.h"

struct TransformUnit;
class CodingStructure;

// ---------------------------------------------------------------------------
// prediction unit
// ---------------------------------------------------------------------------

using MergeIdxPair = std::array<uint16_t, 2>;

struct IntraPredictionData
{
  EnumArray<uint32_t, ChannelType> intraDir;
  bool                             mipTransposedFlag;
  uint8_t                          multiRefIdx;
  int                              cccmFlag;
  ConvModelType                    cccmType;
  CclmOffsets                      cclmOffsets;
  CrossCompModels                  ccModels;
  int                              idxNonLocalCCP;
  int                              ccMergeFusionIdx;
  bool                             decDerivedCcpMode;
  bool                             ccFilterFlag;
  int                              numBvgCands;
  Mv                               bvList[NUM_BVG_CCCM_CANDS];
};

template<typename MODE_TYPE, int FUSION_NUM> struct IPMFusionData
{
  bool      isBlend;
  MODE_TYPE blendMode[FUSION_NUM];
  int8_t    relWeight[FUSION_NUM];
  int8_t    locDep[FUSION_NUM];
};
using DimdData = IPMFusionData<int8_t, DIMD_FUSION_NUM>;
using TimdData = IPMFusionData<int16_t, TIMD_FUSION_NUM>;
using ObicData = IPMFusionData<int8_t, OBIC_FUSION_NUM>;

struct LicScaleAndOffset
{
  int scale[NUM_RPL01][MAX_NUM_COMP];
  int offset[NUM_RPL01][MAX_NUM_COMP];

  LicScaleAndOffset() { reset(); }

  void reset()
  {
    std::fill_n(&scale[0][0], NUM_RPL01 * MAX_NUM_COMP, MAX_INT);
    std::fill_n(&offset[0][0], NUM_RPL01 * MAX_NUM_COMP, MAX_INT);
  }

  void copyFrom(const LicScaleAndOffset &other, const bool luma, const bool chroma)
  {
    for (const int ref: { 0, 1 })
    {
      for (int comp = luma ? 0 : 1; comp < (chroma ? 3 : 1); comp++)
      {
        scale[ref][comp]  = other.scale[ref][comp];
        offset[ref][comp] = other.offset[ref][comp];
      }
    }
  }
};

struct InterPredictionData
{
  bool    mergeFlag;
  bool    regularMergeFlag;
  uint8_t mergeIdx;
  uint8_t geoSplitDir;

  MergeIdxPair geoMergeIdx;

  bool                      mmvdMergeFlag;
  MmvdIdx                   mmvdMergeIdx;
  bool                      bmMergeFlag;
  uint8_t                   bmDir;
  uint8_t                   interDir;
  uint8_t                   mvpIdx[NUM_RPL01];
  uint8_t                   mvpNum[NUM_RPL01];
  Mv                        mvd[NUM_RPL01];
  Mv                        mv[NUM_RPL01];
  int16_t                   refIdx[NUM_RPL01];
  MergeType                 mergeType;
  std::array<bool, 2>       geoMMVDFlag = { false, false };
  std::array<uint8_t, 2>    geoMMVDIdx  = { std::numeric_limits<uint8_t>::max(), std::numeric_limits<uint8_t>::max() };
  bool                      gpmIntraFlag;
  uint8_t                   geoBldIdx;
  bool                      bdmvrRefine;
  Mv                        mvdAffi[NUM_RPL01][3];
  Mv                        mvAffi[NUM_RPL01][3];
  bool                      ciipFlag;
  CIIP_Type                 ciipMode;
  bool                      obmcFlag;
  bool                      licFlag;
  // need write access to licScaleAndOffset in InterPrediction::xLocalIlluComp(), therefore mutable
  mutable LicScaleAndOffset licScaleAndOffset;
  bool                      oppositeLicFlag;

  Mv      bv;               // block vector for IBC
  Mv      bvd;              // block vector difference for IBC
  uint8_t mmvdEncOptMode;   // 0: no action 1: skip chroma MC for MMVD candidate pre-selection 2: skip chroma MC and BIO
                            // for MMVD candidate pre-selection

  int8_t gpmDirMode;   // 0: use uni list gen, 1 : use bi list gen
};

struct CodingUnit : public UnitArea, public IntraPredictionData, public InterPredictionData
{
  CodingStructure *cs;
  Slice           *slice;
  ChannelType      chType;

  PredMode predMode;

  uint8_t     depth;   // number of all splits, applied with generalized splits
  uint8_t     qtDepth;   // number of applied quad-splits, before switching to the multi-type-tree (mtt)
                     // a triple split would increase the mtDepth by 1, but the qtDepth by 2 in the first and last part
                     // and by 1 in the middle part (because of the 1-2-1 split proportions)
  uint8_t     btDepth;   // number of binary splits after switching to MTT
  uint8_t     mtDepth;   // number of splits after switching to MTT (equals btDepth if only binary splits)
  uint8_t     mtImplicitDepth;
  int8_t      chromaQpAdj;
  int8_t      qp;
  SplitSeries splitSeries;
  bool        skip;
  bool        mmvdSkip;
  bool        affine;
  AffineModel affineType;
  bool        geoFlag;
  BdpcmMode   bdpcmMode[MAX_NUM_CHANNEL_TYPE];
  uint8_t     imv;
  bool        rootCbf;
  uint8_t     sbtInfo;
  uint32_t    tileIdx;
  uint8_t     bcwIdx;
  int         refIdxBi[2];
  int8_t      derivedIpm[2];
  bool        mipFlag;
  bool        dimdFlag;
  DimdData    dimdData;
  bool        dimdChromaFlag;
  bool        mpmFlag;
  bool        secondMpmFlag;
  bool        timdFlag;
  TimdData    timdData;
  bool        timdSadFlag;
  TimdData    timdSadData;
  bool        timdMergeFlag;
  bool        timdMergeAvailable;
  TimdData    timdMergeData;
  TransType   timdMergeTrType[2];   // horizontal, vertical
  bool        obicAvailFlag;
  bool        obicFlag;
  ObicData    obicData;
  std::vector<const CodingUnit *> obicNeighbours;
  bool                            eipFlag;
  bool                            eipMerge;
  bool                            eipMultiModel;
  EipModels                       eipModels;
  uint8_t                         inferredDimdMode;
  uint32_t                        lumaModeIdx;
  uint32_t                        chromaModeIdx;
  uint8_t                         smvdMode;
  PlanarDirType                   plDir;
  uint8_t                         timdHor;
  uint8_t                         timdVer;
  bool                            sgpm;
  int8_t                          sgpmIdx;
  int8_t                          sgpmSplitDir;
  int8_t                          sgpmMode0;
  int8_t                          sgpmMode1;
  bool                            useEscape[MAX_NUM_CHANNEL_TYPE];
  bool                            useRotation[MAX_NUM_CHANNEL_TYPE];
  bool                            reuseflag[MAX_NUM_CHANNEL_TYPE][MAXPLTPREDSIZE];
  uint8_t                         lastPLTSize[MAX_NUM_CHANNEL_TYPE];
  uint8_t                         reusePLTSize[MAX_NUM_CHANNEL_TYPE];
  uint8_t                         curPLTSize[MAX_NUM_CHANNEL_TYPE];
  Pel                             curPLT[MAX_NUM_COMP][MAXPLTSIZE];

  CodingUnit() : chType(ChannelType::LUMA) { initPuData(); }
  CodingUnit(const UnitArea &unit);
  CodingUnit(const ChromaFormat _chromaFormat, const Area &area);

  CodingUnit &operator=(const CodingUnit &other);

  void initCuData();
  void initPuData();
  void initData()
  {
    initCuData();
    initPuData();
  }

  unsigned    idx;
  CodingUnit *next;

  TransformUnit *firstTU;
  TransformUnit *lastTU;

  const uint8_t getSbtIdx() const
  {
    assert(((sbtInfo >> 0) & 0xf) < NUMBER_SBT_IDX);
    return (sbtInfo >> 0) & 0xf;
  }
  const uint8_t getSbtPos() const { return (sbtInfo >> 4) & 0x3; }
  void          setSbtIdx(uint8_t idx)
  {
    CHECK(idx >= NUMBER_SBT_IDX, "sbt_idx wrong");
    sbtInfo = (idx << 0) + (sbtInfo & 0xf0);
  }
  void setSbtPos(uint8_t pos)
  {
    CHECK(pos >= 4, "sbt_pos wrong");
    sbtInfo = (pos << 4) + (sbtInfo & 0xcf);
  }
  uint8_t       getSbtTuSplit() const;
  const uint8_t checkAllowedSbt() const;
  int           getSbtTuIdx() const;
  BdpcmMode     getBdpcmMode(const CompID compId) const { return isLuma(compId) ? bdpcmMode[0] : bdpcmMode[1]; }

  int getNumAffineMvs() const { return affineType == AffineModel::_6_PARAMS ? 3 : 2; }

  CodingUnit &operator=(const IntraPredictionData &predData);
  CodingUnit &operator=(const InterPredictionData &predData);
  CodingUnit &operator=(const MotionInfo &mi);

  void getAffineMotionInfo(AffineMotionInfo affineMiOut[2], int refIdxOut[2]) const;

  // for accessing motion information, which can have higher resolution than PUs (should always be used, when accessing
  // neighboring motion information)
  const MotionInfo &getMotionInfo() const;
  const MotionInfo &getMotionInfo(const Position &pos) const;
  MotionBuf         getMotionBuf();
  CMotionBuf        getMotionBuf() const;

  bool isAffineBlock() const { return affine && mergeType != MergeType::SUBPU_ATMVP; }

  bool hasSameLicParameters(const CodingUnit &other) const;
};

// ---------------------------------------------------------------------------
// transform unit
// ---------------------------------------------------------------------------

struct TransformUnit : public UnitArea
{
  CodingUnit      *cu;
  CodingStructure *cs;
  ChannelType      chType;
  int              m_chromaResScaleInv;

  uint8_t depth;

  std::array<MtsType, MAX_NUM_TBLOCKS> mtsIdx;

  bool                      noResidual;
  uint8_t                   jointCbCr;
  uint8_t                   cbf[MAX_NUM_TBLOCKS];
  int                       lastPos[MAX_NUM_TBLOCKS];
  uint32_t                  numPredAreaSigns[MAX_NUM_TBLOCKS];
  std::pair<int8_t, int8_t> derivedIntraDirsLuma;
  int8_t                    derivedIntraDirChroma;

  TransformUnit() : chType(ChannelType::LUMA) {}
  TransformUnit(const UnitArea &unit);
  TransformUnit(const ChromaFormat _chromaFormat, const Area &area);

  void initData();

  unsigned       idx;
  TransformUnit *next;
  TransformUnit *prev;
  void           init(TCoeff **coeffs, uint8_t **signsPredArea, EnumArray<Pel *, ChannelType> &pltIdx,
                      EnumArray<bool *, ChannelType> &runType);

  TransformUnit &operator=(const TransformUnit &other);
  void           copyComponentFrom(const TransformUnit &other, const CompID compID);
  void           checkTuNoResidual(unsigned idx);
  int            countNonZero();
  int            getTbAreaAfterCoefZeroOut(CompID compID) const;

  bool checkNSTApplied(CompID compID) const;
  int  maxNumPredSigns(CompID compID) const;

  uint8_t       *getSignsPredArea(const CompID id) { return m_signsPredArea[id]; }
  const uint8_t *getSignsPredArea(const CompID id) const { return m_signsPredArea[id]; }

  CoeffBuf            getCoeffs(const CompID id);
  const CCoeffBuf     getCoeffs(const CompID id) const;
  int                 getChromaAdj() const;
  void                setChromaAdj(int i);
  PelBuf              getcurPLTIdx(const ChannelType id);
  const CPelBuf       getcurPLTIdx(const ChannelType id) const;
  PLTtypeBuf          getrunType(const ChannelType id);
  const CPLTtypeBuf   getrunType(const ChannelType id) const;
  PLTescapeBuf        getescapeValue(const CompID id);
  const CPLTescapeBuf getescapeValue(const CompID id) const;
  Pel                *getPLTIndex(const ChannelType id);
  bool               *getRunTypes(const ChannelType id);

private:
  TCoeff                       *m_coeffs[MAX_NUM_TBLOCKS];
  uint8_t                      *m_signsPredArea[MAX_NUM_TBLOCKS];
  EnumArray<Pel *, ChannelType> m_pltIdx;

  EnumArray<bool *, ChannelType> m_runType;
};

// ---------------------------------------------------------------------------
// Utility class for easy for-each like unit traversing
// ---------------------------------------------------------------------------

#include <iterator>

template<typename T> class UnitIterator
{
private:
  T *m_punit;

public:
  UnitIterator() : m_punit(nullptr) {}
  UnitIterator(T *_punit) : m_punit(_punit) {}

  typedef T       &reference;
  typedef T const &const_reference;
  typedef T       *pointer;
  typedef T const *const_pointer;

  reference       operator*() { return *m_punit; }
  const_reference operator*() const { return *m_punit; }
  pointer         operator->() { return m_punit; }
  const_pointer   operator->() const { return m_punit; }

  UnitIterator<T> &operator++()
  {
    m_punit = m_punit->next;
    return *this;
  }
  UnitIterator<T> operator++(int)
  {
    auto x = *this;
    ++(*this);
    return x;
  }
  bool operator!=(const UnitIterator<T> &other) const { return m_punit != other.m_punit; }
  bool operator==(const UnitIterator<T> &other) const { return m_punit == other.m_punit; }
};

template<typename T> class UnitTraverser
{
private:
  T *m_begin;
  T *m_end;

public:
  UnitTraverser() : m_begin(nullptr), m_end(nullptr) {}
  UnitTraverser(T *_begin, T *_end) : m_begin(_begin), m_end(_end) {}

  typedef T                     value_type;
  typedef size_t                size_type;
  typedef T                    &reference;
  typedef T const              &const_reference;
  typedef T                    *pointer;
  typedef T const              *const_pointer;
  typedef UnitIterator<T>       iterator;
  typedef UnitIterator<const T> const_iterator;

  iterator       begin() { return UnitIterator<T>(m_begin); }
  const_iterator begin() const { return UnitIterator<T>(m_begin); }
  const_iterator cbegin() const { return UnitIterator<T>(m_begin); }
  iterator       end() { return UnitIterator<T>(m_end); }
  const_iterator end() const { return UnitIterator<T>(m_end); }
  const_iterator cend() const { return UnitIterator<T>(m_end); }
};

typedef UnitTraverser<CodingUnit>    CUTraverser;
typedef UnitTraverser<TransformUnit> TUTraverser;

typedef UnitTraverser<const CodingUnit>    cCUTraverser;
typedef UnitTraverser<const TransformUnit> cTUTraverser;

#endif
