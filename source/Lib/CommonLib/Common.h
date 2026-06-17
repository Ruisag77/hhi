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

/** \file     Common.h
 *  \brief    Common 2D-geometrical structures
 */

#ifndef __COMMON__
#define __COMMON__

#include "CommonDef.h"

using PosType  = int32_t;
using SizeType = uint32_t;

struct Position
{
  PosType x;
  PosType y;

  Position() : x(0), y(0) {}
  Position(const PosType _x, const PosType _y) : x(_x), y(_y) {}

  bool operator!=(const Position &other) const { return x != other.x || y != other.y; }
  bool operator==(const Position &other) const { return x == other.x && y == other.y; }

  Position offset(const Position pos) const { return Position(x + pos.x, y + pos.y); }
  Position offset(const PosType _x, const PosType _y) const { return Position(x + _x, y + _y); }
  void     repositionTo(const Position newPos)
  {
    x = newPos.x;
    y = newPos.y;
  }
  void relativeTo(const Position origin)
  {
    x -= origin.x;
    y -= origin.y;
  }

  Position operator-(const Position &other) const { return { x - other.x, y - other.y }; }
};

struct Size
{
  SizeType width;
  SizeType height;

  Size() : width(0), height(0) {}
  Size(const SizeType _width, const SizeType _height) : width(_width), height(_height) {}

  bool     operator!=(const Size &other) const { return (width != other.width) || (height != other.height); }
  bool     operator==(const Size &other) const { return (width == other.width) && (height == other.height); }
  uint32_t area() const { return (uint32_t)width * (uint32_t)height; }
#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS
  void resizeTo(const Size newSize)
  {
    width  = newSize.width;
    height = newSize.height;
  }
#endif
};

struct Area : public Position, public Size
{
  Area() : Position(), Size() {}
  Area(const Position &_pos, const Size &_size) : Position(_pos), Size(_size) {}
  Area(const PosType _x, const PosType _y, const SizeType _w, const SizeType _h) : Position(_x, _y), Size(_w, _h) {}

  Position       &pos() { return *this; }
  const Position &pos() const { return *this; }
  Size           &size() { return *this; }
  const Size     &size() const { return *this; }

  const Position &topLeft() const { return *this; }
  Position        topRight() const { return { (PosType)(x + width - 1), y }; }
  Position        bottomLeft() const { return { x, (PosType)(y + height - 1) }; }
  Position        bottomRight() const { return { (PosType)(x + width - 1), (PosType)(y + height - 1) }; }
  Position        center() const { return { (PosType)(x + width / 2), (PosType)(y + height / 2) }; }

  bool contains(const Area &_area) const { return contains(_area.pos()) && contains(_area.bottomRight()); }
  bool contains(const Position &_pos) const
  {
    static_assert(std::is_unsigned<SizeType>::value, "SizeType should be unsigned");
    // NOTE: when _pos.x is smaller than x, SizeType(_pos.x - x) is a large unsigned number
    const bool condX = SizeType(_pos.x - x) < width;
    const bool condY = SizeType(_pos.y - y) < height;

    return condX && condY;
  }

  bool operator!=(const Area &other) const { return (Size::operator!=(other)) || (Position::operator!=(other)); }
  bool operator==(const Area &other) const { return (Size::operator==(other)) && (Position::operator==(other)); }
};

struct UnitScale
{
  UnitScale() : posx(0), posy(0), area(posx + posy) {}
  UnitScale(int sx, int sy) : posx(sx), posy(sy), area(posx + posy) {}
  int posx;
  int posy;
  int area;

  template<typename T> T scaleHor(const T &in) const { return in >> posx; }
  template<typename T> T scaleVer(const T &in) const { return in >> posy; }
  template<typename T> T scaleArea(const T &in) const { return in >> area; }

  Position scale(const Position &pos) const { return { pos.x >> posx, pos.y >> posy }; }
  Size     scale(const Size &size) const { return { size.width >> posx, size.height >> posy }; }
  Area     scale(const Area &_area) const { return Area(scale(_area.pos()), scale(_area.size())); }
};

struct SplitPred
{
  uint8_t minQtDepth = 0;
  uint8_t qtDepth    = 0;
  uint8_t maxBtDepth = 0;
  uint8_t mttDepth   = 0;
};

namespace std
{
template<> struct hash<Position>
{
  size_t operator()(const Position &value) const { return (((size_t)value.x << 32) + value.y); }
};

template<> struct hash<Size>
{
  size_t operator()(const Size &value) const { return (((size_t)value.width << 32) + value.height); }
};
}   // namespace std

inline ptrdiff_t rsAddr(const Position &pos, const uint32_t stride, const UnitScale &unitScale)
{
  return (ptrdiff_t)(stride >> unitScale.posx) * (ptrdiff_t)(pos.y >> unitScale.posy) +
    (ptrdiff_t)(pos.x >> unitScale.posx);
}

inline size_t rsAddr(const Position &pos, const Position &origin, const uint32_t stride, const UnitScale &unitScale)
{
  return (stride >> unitScale.posx) * ((pos.y - origin.y) >> unitScale.posy) + ((pos.x - origin.x) >> unitScale.posx);
}

inline ptrdiff_t rsAddr(const Position &pos, const ptrdiff_t stride)
{
  return stride * (ptrdiff_t)pos.y + (ptrdiff_t)pos.x;
}

inline size_t rsAddr(const Position &pos, const Position &origin, const ptrdiff_t stride)
{
  return stride * (pos.y - origin.y) + (pos.x - origin.x);
}

inline Area clipArea(const Area &_area, const Area &boundingBox)
{
  Area area = _area;

  if (area.x + area.width > boundingBox.x + boundingBox.width)
  {
    area.width = boundingBox.x + boundingBox.width - area.x;
  }

  if (area.y + area.height > boundingBox.y + boundingBox.height)
  {
    area.height = boundingBox.y + boundingBox.height - area.y;
  }

  return area;
}

class SizeIndexInfo
{
protected:
  static constexpr SizeType UNUSED = std::numeric_limits<SizeType>::max();

public:
  SizeIndexInfo() {}
  virtual ~SizeIndexInfo() {}
  SizeType numAllWidths() const { return (SizeType)m_idxToSizeTab.size(); }
  SizeType numAllHeights() const { return (SizeType)m_idxToSizeTab.size(); }
  SizeType numWidths() const { return (SizeType)m_numBlkSizes; }
  SizeType numHeights() const { return (SizeType)m_numBlkSizes; }
  SizeType sizeFrom(SizeType idx) const { return m_idxToSizeTab[idx]; }
  SizeType idxFrom(SizeType size) const
  {
    CHECKD(m_sizeToIdxTab[size] == UNUSED, "Index of given size does NOT EXIST!");
    return m_sizeToIdxTab[size];
  }
  bool         isCuSize(SizeType size) const { return m_isCuSize[size]; }
  virtual void init(SizeType maxSize) {}
  virtual bool hasSizeAtOffset(PosType offset, SizeType size) const = 0;

protected:
  void xInit()
  {
    m_isCuSize.resize(m_sizeToIdxTab.size(), false);

    std::vector<SizeType> grpSizes;

    int n = 0;

    for (int i = 0; i < m_sizeToIdxTab.size(); i++)
    {
      if (m_sizeToIdxTab[i] != UNUSED)
      {
        m_sizeToIdxTab[i] = n;
        m_idxToSizeTab.push_back(i);
        n++;
      }

      if (m_sizeToIdxTab[i] != UNUSED && m_sizeToIdxTab[i >> 1] != UNUSED && i >= MIN_CU_SIZE)
      {
        m_isCuSize[i] = true;
      }

      // collect group sizes (for coefficient group coding)
      SizeType grpSize = i >> ((i & 3) != 0 ? 1 : 2);
      if (m_sizeToIdxTab[i] != UNUSED && m_sizeToIdxTab[grpSize] == UNUSED)
      {
        grpSizes.push_back(grpSize);
      }
    }

    CHECK(n > MAX_NUM_SIZES, "MAX_NUM_SIZES is too small");

    m_numBlkSizes = (SizeType)m_idxToSizeTab.size();

    for (SizeType grpSize: grpSizes)
    {
      if (grpSize > 0 && m_sizeToIdxTab[grpSize] == UNUSED)
      {
        m_sizeToIdxTab[grpSize] = (SizeType)m_idxToSizeTab.size();
        m_idxToSizeTab.push_back(grpSize);
      }
    }
  };

  std::vector<bool>     m_isCuSize;
  int                   m_numBlkSizes; // as opposed to number all sizes, which also contains grouped sizes
  std::vector<SizeType> m_sizeToIdxTab;
  std::vector<SizeType> m_idxToSizeTab;
};

class SizeIndexInfoMtt : public SizeIndexInfo
{
public:
  SizeIndexInfoMtt() {}
  ~SizeIndexInfoMtt() {};

  void init(SizeType maxSize)
  {
    for (int i = 0, n = 0; i <= maxSize; i++)
    {
      SizeType val = UNUSED;
      if (i == (1 << n))
      {
        n++;
        val = i;
      }
      m_sizeToIdxTab.push_back(val);
    }
    SizeIndexInfo::xInit();
  }

  bool hasSizeAtOffset(PosType offset, SizeType size) const
  {
    if (offset + size > MAX_CU_SIZE)
    {
      return false;
    }
    if ((offset & ((1 << (floorLog2(size) - 1)) - 1)) != 0)
    {
      return false;
    }

    return true;
  }
};

typedef int64_t TCccmCoeff;

struct ConvModel
{
  std::vector<TCccmCoeff> params;
  ConvModelType           modelType = CONV_MODEL_UNDEFINED;
  int                     bd        = 0;
  int                     midVal    = 0;

  ConvModel() {}

  ConvModel(int num, int bitdepth) { initModel(num, bitdepth, CONV_MODEL_UNDEFINED); }

  ConvModel(CclmModelSingle &cclmModel)
  {
    modelType = CONV_MODEL_CCLM;

    params.push_back(cclmModel.a);
    params.push_back(cclmModel.b);
    params.push_back(cclmModel.shift);
  }

  void getCclmModelSingle(CclmModelSingle &cclmModel)
  {
    cclmModel.a     = int(params[0]);
    cclmModel.b     = int(params[1]);
    cclmModel.shift = int(params[2]);
  }

  ConvModel(ConvModelType type, int bitdepth)
  {
    switch (type)
    {
    case CONV_MODEL_CCCM:
      initModel(CCCM_NUM_PARAMS, bitdepth, type);
      break;
    case CONV_MODEL_CCCM_GRADLOC:
      initModel(CCCM_NUM_PARAMS, bitdepth, type);
      break;
    case CONV_MODEL_CCCM_MULTIF_1:
      initModel(CCCM_NUM_PARAMS_MF1, bitdepth, type);
      break;
    case CONV_MODEL_CCCM_MULTIF_2:
      initModel(CCCM_NUM_PARAMS_MF2, bitdepth, type);
      break;
    case CONV_MODEL_CCCM_MULTIF_3:
      initModel(CCCM_NUM_PARAMS_MF3, bitdepth, type);
      break;
    case CONV_MODEL_CCCM_BVG:
      initModel(CCCM_NUM_PARAMS_BVG, bitdepth, type);
      break;
    case CONV_MODEL_CCCM_NOSUBS:
      initModel(CCCM_NUM_PARAMS_NOSUB, bitdepth, type);
      break;
    case CONV_MODEL_EIP_S:
      initModel(EIP_FILTER_TAP, bitdepth, type);
      break;
    case CONV_MODEL_EIP_H:
      initModel(EIP_FILTER_TAP, bitdepth, type);
      break;
    case CONV_MODEL_EIP_V:
      initModel(EIP_FILTER_TAP, bitdepth, type);
      break;
    default:
      fprintf(stdout, "No initializer for convolutional model type: %d\n", int(type));
      exit(0);
    }
  }

  void printModel()
  {
    fprintf(stdout, " %d", modelType);
    for (int i = 0; i < params.size(); i++)
    {
      fprintf(stdout, " %10d", int(params[i]));
    }
    fprintf(stdout, "\n");
  }

  void initModel(int num, int bitdepth, ConvModelType type)
  {
    modelType = type;
    bd        = bitdepth;
    midVal    = (1 << (bitdepth - 1));

    params.resize(num, 0);
  }

  void clearModel()
  {
    std::fill(params.begin(), params.end(), 0);

    params[params.size() - 1] = 1 << CCCM_DECIM_BITS; // Default bias to 1
  }

  Pel convolve(Pel *vector) const
  {
    TCccmCoeff sum = 0;

    for (int i = 0; i < params.size(); i++)
    {
      sum += params[i] * vector[i];
    }

    return Pel((sum + CCCM_DECIM_ROUND) >> CCCM_DECIM_BITS);
  }

  Pel nonlinear(const Pel val) const { return (val * val + midVal) >> bd; }
  Pel bias() const { return midVal; }

  const int getNumParams() const { return int(params.size()); }

  bool operator==(const ConvModel &cand) const
  {
    if (modelType == cand.modelType && bd == cand.bd && midVal == cand.midVal)
    {
      return params == cand.params ? true : false;
    }
    else
    {
      return false;
    }
  }
};

struct CrossCompModels
{
  ConvModel modelCb[2] = {};
  ConvModel modelCr[2] = {};

  bool valid      = false;
  bool dualModel  = false;
  bool filter     = false; // Filter for dual mode cases
  int  lumaThres  = 0;     // Luma threshold for dual mode cases
  int  lumaOffset = 0;     // Offset for luma reference samples
  int  refSizeX   = 0;
  int  refSizeY   = 0;

  void init(bool _dualModel, int _lumaThres = 0, bool _filter = false)
  {
    valid      = true;
    dualModel  = _dualModel;
    filter     = _filter;
    lumaThres  = _lumaThres;
    lumaOffset = 0;
  }

  void getCclmModel(CclmModel &cclmModel, CompID compID)
  {
    auto srcModels = compID == COMP_Cb ? modelCb : modelCr;

    if (dualModel)
    {
      cclmModel.multiModelLumaThr = lumaThres;

      srcModels[0].getCclmModelSingle(cclmModel.model[0]);
      srcModels[1].getCclmModelSingle(cclmModel.model[1]);
    }
    else
    {
      srcModels[0].getCclmModelSingle(cclmModel.model[0]);
    }
  }

  void setCclmModel(CclmModel &cclmModel, CompID compID, bool _dualModel = false, bool _filter = false)
  {
    auto dstModels = compID == COMP_Cb ? modelCb : modelCr;

    init(_dualModel, _dualModel ? cclmModel.multiModelLumaThr : 0, _filter);

    if (dualModel)
    {
      dstModels[0] = ConvModel(cclmModel.model[0]);
      dstModels[1] = ConvModel(cclmModel.model[1]);
    }
    else
    {
      dstModels[0] = ConvModel(cclmModel.model[0]);
    }
  }

  void printModels()
  {
    fprintf(stdout, "--\n");
    fprintf(stdout, "valid: %d, dual: %d, thr: %3d, off: %4d\n", valid, dualModel, lumaThres, lumaOffset);
    fprintf(stdout, "Cb0: ");
    modelCb[0].printModel();
    if (dualModel)
    {
      fprintf(stdout, "Cb1: ");
      modelCb[1].printModel();
    }
    fprintf(stdout, "Cr0: ");
    modelCr[0].printModel();
    if (dualModel)
    {
      fprintf(stdout, "Cr1: ");
      modelCr[1].printModel();
    }
  }

  bool operator==(const CrossCompModels &cand) const
  {
    if (dualModel == cand.dualModel && filter == cand.filter && lumaThres == cand.lumaThres &&
        lumaOffset == cand.lumaOffset)
    {
      if (dualModel)
      {
        if (modelCb[0] == cand.modelCb[0] && modelCb[1] == cand.modelCb[1] && modelCr[0] == cand.modelCr[0] &&
            modelCr[1] == cand.modelCr[1])
        {
          return true;
        }
      }
      else
      {
        if (modelCb[0] == cand.modelCb[0] && modelCr[0] == cand.modelCr[0])
        {
          return true;
        }
      }
    }

    return false;
  }
};

struct EipModels
{
  ConvModel model[2]  = {};
  Pel       threshold = 0;

  EipModels() = default;

  EipModels(ConvModelType filterShape, int bitdepth, bool multiModel)
  {
    model[0] = ConvModel(filterShape, bitdepth);
    if (multiModel == true)
    {
      model[1] = ConvModel(filterShape, bitdepth);
    }
  }

  bool isMultiModel() const { return model[1].getNumParams() > 0; }

  bool operator==(const EipModels &other) const
  {
    return (model[0] == other.model[0]) && (model[1] == other.model[1]) && (threshold == other.threshold);
  }
};

struct LutCCP
{
  static_vector<CrossCompModels, MAX_NUM_HCCP_CANDS> lutCCP;
};

struct LutEIP
{
  static_vector<EipModels, MAX_NUM_HEIP_CANDS> lutEip;
};

#endif
