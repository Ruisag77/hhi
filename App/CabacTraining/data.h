#pragma once
#include <vector>
#include <array>
#include "Contexts.h"

using QP   = int;
using Bin  = uint8_t;
using Bins = std::vector<Bin>;

// as read in the Contexts.cpp file
struct ModelParameters
{
  int initId;
  int log2windowsize;
  int adaptweight;
  int rateoffset[2];
};

constexpr int kNbModels = 4;
using Models            = std::array<ModelParameters, kNbModels>;

struct DataFrame
{
  Bins      bins;
  SliceType type;
  SliceType reportslice;
  int       poc;
  bool      tempCABAC;
  bool      switchBp;
  QP        qp;

  // parameters as set internally inside cabac
  uint16_t p0, p1;
  int      rate;
  int      weight;
  int      drate0;
  int      drate1;
};

struct DataSequence
{
  std::vector<DataFrame> v;
  uint64_t               filesize = 0;
};

using DataSequences = std::vector<DataSequence>;

struct DataDb
{
  int           ctxidx = -1;
  Models        modelsBPI;
  DataSequences v;
};

inline int window2Coding(int w0, int w1)
{
  int log2WindowSize = ((w0 & 3) << 2) | (w1 & 3); // stupid encoding
  return log2WindowSize;
}

inline std::tuple<int, int> coding2Window(int rate)
{
  int rate0 = 2 + ((rate >> 2) & 3);
  int rate1 = 3 + rate0 + (rate & 3);
  return std::make_tuple(rate0, rate1);
}

inline int alpha2Coding(int alpha0, int alpha1) { return (alpha0 << 3) | (alpha1 & 0x07); }

inline std::tuple<int, int> coding2alpha(int code) { return std::make_tuple(code >> 3, code & 0x07); }

inline int drate2Coding(int dr0, int dr1)
{
// for a bin 0 or 1
//  int drate0=  (mi.rateoffset0>>4)-ADJUSTMENT_RANGE;
//  int drate1=  (mi.rateoffset0&15)-ADJUSTMENT_RANGE;
//  prm[ctx].dr[0][0] = drate0;
//  prm[ctx].dr[0][1] = drate1;

  return ((dr0 + ADJUSTMENT_RANGE) << 4) | ((dr1 + ADJUSTMENT_RANGE) & 15);
}

inline std::tuple<int, int> coding2drate(int code)
{
  return std::make_tuple((code >> 4) - ADJUSTMENT_RANGE, (code & 15) - ADJUSTMENT_RANGE);
}

inline int probaInitConstant2coding(int p0, int p1)
{
// get InitiId assuming slope=0
//  int slope = (initId >> 3) - 4;
//  int offset = ((initId & 7) * 18) + 1;
//  int inistate = ((slope   * (qp - 16)) >> 1) + offset;
//  int stateClip = inistate < 1 ? 1 : inistate > 127 ? 127 : inistate;

  int offset = (128 * (p0 + p1)) / ((1 << PROB_BITS) + 1);
  offset -= 1;

  if (offset < 0)
  {
    offset = 0;
  }
  offset /= 18;

  if (offset > 7)
  {
    offset = 7;
  }
  return (4 << 3) | offset;
}
