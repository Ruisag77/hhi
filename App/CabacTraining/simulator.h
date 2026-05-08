#pragma once
#include <iostream>
#include <vector>
#include <tuple>
#include "Contexts.h"
#include "data.h"
using namespace std;

inline void initDefaultParameters(const ModelParameters &prms, const DataFrame &slice, BinProbModel_Std &cabac)
{
  // use frame extracted parameters (should be the same as default one, except for proba)
  cabac.setState({ slice.p0, slice.p1 });
  cabac.setWinSizes(slice.rate);
  cabac.setAdaptRateWeight(slice.weight);
  cabac.setAdaptRateOffset(slice.drate0, 0);
  cabac.setAdaptRateOffset(slice.drate1, 1);
}

inline std::pair<uint16_t, uint16_t> coding2ProbaInit(int initId, int qp)
{
  // getP0
  int      slope     = (initId >> 3) - 4;
  int      offset    = ((initId & 7) * 18) + 1;
  int      inistate  = ((slope * (qp - 16)) >> 1) + offset;
  int      stateClip = (inistate < 1) ? 1 : (inistate > 127 ? 127 : inistate);
  uint16_t p0        = (stateClip << 8) & MASK_0;
  uint16_t p1        = (stateClip << 8) & MASK_1;

  return std::make_pair(p0, p1);
}

inline void initNewParameters(const ModelParameters &prms, const DataFrame &slice, BinProbModel_Std &cabac)
{
  if (!slice.tempCABAC)
  {
    // use the proba model
    cabac.setState(coding2ProbaInit(prms.initId, slice.qp));
  }

  cabac.setLog2WindowSize(prms.log2windowsize);
  cabac.setAdaptRateWeight(prms.adaptweight);
  cabac.setAdaptRateOffset(prms.rateoffset[0], 0);
  cabac.setAdaptRateOffset(prms.rateoffset[1], 1);
}

inline uint64_t cost1Frame(const DataFrame &slice, BinProbModel_Std &cabac)
{
  uint64_t cost {};
  for (auto b: slice.bins)
  {
    cabac.estFracBitsUpdate(b, cost);
  }
  return cost;
}

inline uint64_t cost1Sequence(const DataSequence &seq, const ModelParameters &prms, SliceType st)
{
  BinProbModel_Std cabac;
  uint64_t         cost = 0;

  for (const auto &slice: seq.v)
  {
    if (slice.reportslice == st)
    {
      initDefaultParameters(prms, slice, cabac);
      initNewParameters(prms, slice, cabac);
      cost += cost1Frame(slice, cabac);
    }
  }
  return cost;
}

inline bool hasBin1Frame(const DataFrame &slice) { return !slice.bins.empty(); }

inline bool hasBin1Sequence(const DataSequence &seq, SliceType st)
{
  for (const auto &slice: seq.v)
  {
    if (slice.reportslice == st)
    {
      if (hasBin1Frame(slice))
      {
        return true;
      }
    }
  }
  return false;
}

inline bool hasBin1Db(const DataSequences &seqs, SliceType st)
{
  for (const auto &seq: seqs)
  {
    if (hasBin1Sequence(seq, st))
    {
      return true;
    }
  }
  return false;
}

inline double cost1Db(const DataSequences &seqs, const ModelParameters &prms, SliceType st)
{
  double costr = 0.;
  for (const auto &seq: seqs)
  {
    auto c = cost1Sequence(seq, prms, st);
    costr += (double)c / (8 * seq.filesize);
  }
  return costr;
}

inline double cost1Db(const DataSequences &seqs, const std::array<ModelParameters, kNbModels> &prms,
                      const std::array<bool, kNbModels> &sliceActivation)
{
  double costr = 1;
  for (const auto &seq: seqs)
  {
    double costr2 = 0.;

    for (int i = 0; i < kNbModels; ++i)
    {
      if (sliceActivation[i])
      {
        costr2 += cost1Sequence(seq, prms[i], (SliceType)i);
      }
    }
    costr += costr2 / (8 * seq.filesize);
  }
  return costr;
}
