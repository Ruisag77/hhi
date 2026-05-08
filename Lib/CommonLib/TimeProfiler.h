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
/** \file     TimeProfiler.h
    \brief    profiling of run-time behavior (header)
*/
#pragma once

#include "CommonDef.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <stack>
#include <array>
#include <memory>
#include <chrono>
#include <numeric>
#include <ostream>
#include <sstream>

#if ENABLE_TIME_PROFILING

#define ONLY_EXLUSIVE_TIME 0  // light-weight as possible

// Here, users can add profiling stages
#define E_TIME_PROF_STAGES(E_)      \
  E_(P_HOST_LEVEL)                  \
  E_(P_TOP_LEVEL)                   \
  E_(P_DECODER)                     \
  E_(P_COMPRESS_GOP)                \
  E_(P_DECODE_SLICE)                \
  E_(P_PARSE_CTU)                   \
  E_(P_COMPRESS_SLICE)              \
  E_(P_REC_CTU)                     \
  E_(P_COMPRESS_CU)                 \
  E_(P_REC_INTER)                   \
  E_(P_INTER_MRG)                   \
  E_(P_INTER_MRG_EST_CAND)          \
  E_(P_INTER_MRG_EST_CAND_REG)      \
  E_(P_INTER_MRG_EST_CAND_BM)       \
  E_(P_INTER_MRG_EST_CAND_MMVD)     \
  E_(P_INTER_MRG_EST_CAND_AFFINE)   \
  E_(P_INTER_MRG_EST_CAND_AFF_MMVD) \
  E_(P_INTER_MRG_EST_CAND_GPM)      \
  E_(P_INTER_MRG_EST_CAND_GPM_MMVD) \
  E_(P_INTER_MRG_EST_CAND_CIIP)     \
  E_(P_INTER_MRG_RD_CHECK)          \
  E_(P_DMVR)                        \
  E_(P_INTER_MVD)                   \
  E_(P_INTER_MVD_AMVR)              \
  E_(P_INTER_MVD_WO_OBMC)           \
  E_(P_INTER_MVD_SEARCH)            \
  E_(P_INTER_MVD_SEARCH_B)          \
  E_(P_INTER_MVD_SEARCH_HPEL)       \
  E_(P_INTER_MVD_SEARCH_QPEL)       \
  E_(P_INTER_MVD_SEARCH_AFFINE)     \
  E_(P_INTER_HASH)                  \
  E_(P_MOTION_COMPENSATION)         \
  E_(P_MOTION_COMP_AFFINE)          \
  E_(P_OBMC)                        \
  E_(P_MOTION_COMP_WEIGHT_AVG)      \
  E_(P_BDOF)                        \
  E_(P_FRAC_PEL_SEARCH)             \
  E_(P_INTRA)                       \
  E_(P_REC_INTRA)                   \
  E_(P_INTRA_EST_CAND_LUMA)         \
  E_(P_INTRA_EST_CAND_LUMA_DIMD)    \
  E_(P_INTRA_EST_CAND_LUMA_TIMD)    \
  E_(P_INTRA_EST_CAND_LUMA_OBIC)    \
  E_(P_INTRA_EST_TRAFO)             \
  E_(P_INTRA_RD_CHECK_LUMA)         \
  E_(P_INTRA_CHROMA)                \
  E_(P_IBC)                         \
  E_(P_TRAFO_QUANT)                 \
  E_(P_TRAFO)                       \
  E_(P_QUANT)                       \
  E_(P_INTRA_FRAC_BITS)             \
  E_(P_LOOPFILTERS)                 \
  E_(P_RESHAPER)                    \
  E_(P_DEBLOCK_FILTER)              \
  E_(P_SAO)                         \
  E_(P_ALF)                         \
  E_(P_OTHER)                       \
  E_(P_STAGES)                      \
  E_(P_IGNORE)
MAKE_ENUM_AND_STRINGS(E_TIME_PROF_STAGES, STAGE, stageNames)

// enable low-level groups of stages
#define TP_ENABLE_INTER_MERGE_EST_STAGES       1
#define TP_ENABLE_INTER_MVD_SEARCH_STAGES      0
#define TP_ENABLE_INTER_FRAC_PEL_SEARCH_STAGES 0
#define TP_ENABLE_TRQUANT_STAGES               0

template<class Rep> std::ostream &operator<<(std::ostream &os, const std::chrono::duration<double, Rep> d)
{
  os << d.count();
  return os;
}

class TimeProfiler
{
public:
  using rep        = std::milli;
  using duration   = std::chrono::duration<double, rep>;
  using clock      = std::chrono::steady_clock;
  using time_point = std::chrono::time_point<clock, duration>;

protected:
  STAGE                 m_eStage;
  time_point            m_previous  = clock::now();
  const unsigned        m_numStages = sizeof(stageNames) / sizeof(stageNames[0]);
  std::vector<duration> m_exclDurations;
  std::vector<duration> m_inclDurations;
  std::vector<duration> m_scopeDuration;

public:
  TimeProfiler(STAGE initStage = P_IGNORE) : m_eStage(initStage)
  {
    init();
    m_scopeDuration.push_back(duration::zero());
  }
  ~TimeProfiler() {}
  void init()
  {
    m_exclDurations.resize(m_numStages, duration::zero());
    m_inclDurations.resize(m_numStages, duration::zero());
  }
  TimeProfiler &operator()(STAGE s)
  {
    time_point now = clock::now();
    m_exclDurations[m_eStage] += (now - m_previous);
    m_previous = now;
    m_eStage   = s;
    return *this;
  }
  void toNextInclusiveTimeScope(STAGE s)
  {
    time_point now = clock::now();
    duration   dur = (now - m_previous);
    m_exclDurations[m_eStage] += dur;
    m_previous = now;
    m_eStage   = s;

    m_scopeDuration.back() += dur;       // last duration from parent before new child starts
    m_scopeDuration.push_back(duration::zero());  // time slot for new child
  }
  void toPrevInclusiveTimeScope(STAGE s)
  {
    time_point now = clock::now();
    duration   dur = (now - m_previous);
    m_exclDurations[m_eStage] += dur;
    m_previous = now;
    m_eStage   = s;

    m_scopeDuration.back() += dur;        // child final time
    duration child_dur = m_scopeDuration.back();
    m_inclDurations[s] += child_dur;       // parent accumulates durations of children (inclusive-time)
    m_scopeDuration.pop_back();           // remove child, we move to parent
    m_scopeDuration.back() +=
      child_dur;  // add child scope duration to parent scope (bottom-top inclusive-time propagation)
  }
  TimeProfiler &operator+=(const TimeProfiler &other)
  {
    {
      auto i1 = m_exclDurations.begin();
      auto i2 = other.m_exclDurations.cbegin();
      for (; i1 != m_exclDurations.end() && i2 != other.m_exclDurations.cend(); ++i1, ++i2)
      {
        *i1 += *i2;
      }
    }
    {
      auto i1 = m_inclDurations.begin();
      auto i2 = other.m_inclDurations.cbegin();
      for (; i1 != m_inclDurations.end() && i2 != other.m_inclDurations.cend(); ++i1, ++i2)
      {
        *i1 += *i2;
      }
    }
    return *this;
  }

  void start(STAGE s)
  {
    m_previous = clock::now();
    m_eStage   = s;
  }
  void stop()
  {
    time_point now = clock::now();
    m_exclDurations[m_eStage] += (now - m_previous);
  }
  STAGE  curStage() { return m_eStage; }
  size_t numStages() { return m_numStages; }
  bool   isUsed()
  {
    return std::accumulate(m_exclDurations.begin(), m_exclDurations.begin() + P_STAGES - 1, duration {}) !=
      duration::zero();
  }

  friend std::ostream &operator<<(std::ostream &os, const TimeProfiler &prof);

  void output(std::ostream &os) { os << *this; }
};

class ScopedTimeProfiler
{
  STAGE         m_ePrevStage;
  TimeProfiler *m_profiler = nullptr;

public:
  ScopedTimeProfiler(TimeProfiler *pcProfiler, STAGE eStage)
  {
    m_ePrevStage = pcProfiler->curStage();
    if (eStage != m_ePrevStage)
    {
      m_profiler = pcProfiler;
#if ONLY_EXLUSIVE_TIME
      (*m_profiler)(eStage);
#else
      m_profiler->toNextInclusiveTimeScope(eStage);
#endif
    }
  }
  ~ScopedTimeProfiler()
  {
    if (m_profiler)
    {
#if ONLY_EXLUSIVE_TIME
      (*m_profiler)(m_ePrevStage);
#else
      m_profiler->toPrevInclusiveTimeScope(m_ePrevStage);
#endif
    }
  }
};

#if ONLY_EXLUSIVE_TIME
#define PROF_TO_NEXT_SCOPE(p, s) (*(p))(s)
#define PROF_TO_PREV_SCOPE(p, s) (*(p))(s)
#else
#define PROF_TO_NEXT_SCOPE(p, s) (*(p)).toNextInclusiveTimeScope(s)
#define PROF_TO_PREV_SCOPE(p, s) (*(p)).toPrevInclusiveTimeScope(s)
#endif

#define PROF_TO_NEXT_SCOPE_COND_0(p, s)
#define PROF_TO_NEXT_SCOPE_COND_1(p, s)     PROF_TO_NEXT_SCOPE(p, s)
#define PROF_TO_NEXT_SCOPE_COND(cond, p, s) PROF_TO_NEXT_SCOPE_COND_##cond(p, s)
#define PROFILER_TO_NEXT_SCOPE_(cond, p, s) PROF_TO_NEXT_SCOPE_COND(cond, p, s)

#define PROF_TO_PREV_SCOPE_COND_0(p, s)
#define PROF_TO_PREV_SCOPE_COND_1(p, s)     PROF_TO_PREV_SCOPE(p, s)
#define PROF_TO_PREV_SCOPE_COND(cond, p, s) PROF_TO_PREV_SCOPE_COND_##cond(p, s)
#define PROFILER_TO_PREV_SCOPE_(cond, p, s) PROF_TO_PREV_SCOPE_COND(cond, p, s)

#define PROF_SCOPE_COND_0(p, s)
#define PROF_SCOPE_COND_1(p, s)     ScopedTimeProfiler cScopedProfiler##s((p), (s))
#define PROF_SCOPE_COND(cond, p, s) PROF_SCOPE_COND_##cond(p, s)
#define PROFILER_SCOPE_(cond, p, s) PROF_SCOPE_COND(cond, p, s)

#define PROFILER_START(p, s)               (*(p)).start(s)
#define PROFILER_STOP(p)                   (*(p)).stop()
#define PROFILER_TO_NEXT_SCOPE(cond, p, s) PROFILER_TO_NEXT_SCOPE_(cond, p, s)
#define PROFILER_TO_PREV_SCOPE(cond, p, s) PROFILER_TO_PREV_SCOPE_(cond, p, s)
#define PROFILER_SCOPE(cond, p, s)         PROFILER_SCOPE_(cond, p, s)
#define PROFILER_SET(p, p_next)            p = p_next;
#define PROFILER_SWITCH(p, p_next, p_next_stage) \
  PROFILER_STOP(p);                              \
  p = p_next;                                    \
  PROFILER_START(p_next, p_next_stage);

#else
#define PROFILER_START(p, s)
#define PROFILER_STOP(p)
#define PROFILER_TO_NEXT_SCOPE(cond, p, s)
#define PROFILER_TO_PREV_SCOPE(cond, p, s)
#define PROFILER_SCOPE(cond, p, s)
#define PROFILER_SET(p, p_next)
#define PROFILER_SWITCH(p, p_next, p_next_stage)
#endif

//! \}
