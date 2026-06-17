#pragma once
#include "simulator.h"

static constexpr int log2winsize[13] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13 };
static constexpr int adaptweight[5]  = { 4, 11, 18, 25, 32 };

// get best model for each ctx given a set of frames
inline ModelParameters getBestGreedy(const DataDb &db, SliceType st)
{
  constexpr int   slopeStart       = 0;
  constexpr int   slopeEnd         = 7;
  constexpr int   offsetstart      = 0;
  constexpr int   offsetend        = 7;
  constexpr int   winSizesIdxStart = 0;
  constexpr int   winSizesIdxEnd   = 12;
  constexpr int   weightIdxStart   = 0;
  constexpr int   weightIdxEnd     = 4;
  ModelParameters prm              = db.modelsBPI[st];
  ModelParameters bestprm          = prm;
  double          bestc            = std::numeric_limits<double>::max();

  for (int slope = slopeStart; slope <= slopeEnd; ++slope)
  {
    for (int offset = offsetstart; offset <= offsetend; ++offset)
    {
      for (int winSizesIdx = winSizesIdxStart; winSizesIdx <= winSizesIdxEnd; ++winSizesIdx)
      {
        for (int weightIdx = weightIdxStart; weightIdx <= weightIdxEnd; ++weightIdx)
        {
          prm.initId         = (slope << 3) | (offset);
          prm.log2windowsize = log2winsize[winSizesIdx];
          prm.adaptweight    = adaptweight[weightIdx];
          double c           = cost1Db(db.v, prm, st);

          if (c < bestc)
          {
            bestc   = c;
            bestprm = prm;
          }
        }
      }
    }
  }
  return bestprm;
}

// get best model optimizing drate
inline std::array<int, 2> getBestGreedyDrate(const DataDb &db, const SliceType st)
{
  constexpr int drateStartIdx = 0;
  constexpr int drateEndIdx   = 15;
  constexpr int kMaxRate      = 12; // theoritcal is PROB_BITS=15
  int           bestdrate[4]  = { 7, 7, 7, 7 };
  double        bestc         = std::numeric_limits<double>::max();

  for (int drate00 = drateStartIdx; drate00 <= drateEndIdx; drate00++)
  {
    for (int drate01 = drateStartIdx; drate01 <= drateEndIdx; drate01++)
    {
      for (int drate10 = drateStartIdx; drate10 <= drateEndIdx; drate10++)
      {
        for (int drate11 = drateStartIdx; drate11 <= drateEndIdx; drate11++)
        {
          // check if ok for one slice type at least
          bool notOk = true;

          ModelParameters prm       = db.modelsBPI[st];
          int             rate0     = 2 + ((prm.log2windowsize >> 2) & 3);
          int             rate1     = 3 + rate0 + (prm.log2windowsize & 3);
          // avoid testing redundant prms
          int             rateUsed0 = rate0 + drate00 - ADJUSTMENT_RANGE;
          int             rateUsed1 = rate1 + drate01 - ADJUSTMENT_RANGE;

          if (rateUsed0 < 2 || rateUsed1 < 2 || rateUsed0 > kMaxRate || rateUsed1 > kMaxRate)
          {
            continue;
          }

          rateUsed0 = rate0 + drate10 - ADJUSTMENT_RANGE;
          rateUsed1 = rate1 + drate11 - ADJUSTMENT_RANGE;

          if (rateUsed0 < 2 || rateUsed1 < 2 || rateUsed0 > kMaxRate || rateUsed1 > kMaxRate)
          {
            continue;
          }
          notOk = false;

          if (notOk)
          {
            continue;
          }

          auto prms = db.modelsBPI[st];

          prms.rateoffset[0] = (drate00 << 4) | (drate01);
          prms.rateoffset[1] = (drate10 << 4) | (drate11);

          double c = cost1Db(db.v, prms, st);

          if (c < bestc)
          {
            bestc        = c;
            bestdrate[0] = drate00;
            bestdrate[1] = drate01;
            bestdrate[2] = drate10;
            bestdrate[3] = drate11;
          }
        }
      }
    }
  }

  std::array<int, 2> bestDrate;
  bestDrate[0] = (bestdrate[0] << 4) | bestdrate[1];
  bestDrate[1] = (bestdrate[2] << 4) | bestdrate[3];
  return bestDrate;
}
