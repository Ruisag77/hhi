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
/** \file     TimeProfiler.cpp
    \brief    profiling of run-time behavior
*/
#include "TimeProfiler.h"

#if ENABLE_TIME_PROFILING
#ifndef EOL
#define EOL "\n"
#endif

std::ostream &operator<<(std::ostream &os, const TimeProfiler &prof)
{
  const TimeProfiler::duration counted = std::accumulate(
    prof.m_exclDurations.begin(), prof.m_exclDurations.begin() + P_STAGES - 1, TimeProfiler::duration {});
  const double      scale = 1.0;
  const int         prec  = 1;
  std::stringstream ss;
  ss << std::fixed << std::setprecision(prec) << (counted / scale);
  const int    spaces[5] = { 10, 30, 8, 12, 15 };
  const size_t ts        = 1 + std::max((int)ss.str().size(), spaces[4]);

  os << std::setw(spaces[0]) << " " << std::setw(spaces[1]) << std::left << "stages" << std::internal;

  bool inclTimeExists = !std::all_of(prof.m_inclDurations.begin(), prof.m_inclDurations.end(),
                                     [](TimeProfiler::duration d) { return d == TimeProfiler::duration::zero(); });
  if (inclTimeExists)
  {
    os << std::setw((int)ts) << "incl.time(ms)" << std::setw(spaces[2]) << "%";
  }

  os << std::setw((int)ts) << "excl.time(ms)" << std::setw(spaces[2]) << "%";
  os << EOL;

  for (size_t i = 0; i < P_STAGES; ++i)
  {
    auto v = prof.m_exclDurations[i];
    if (v.count() != 0.0)
    {
      os << std::setw(spaces[0]) << " " << std::setw(spaces[1]) << std::left << stageNames[i] << std::internal;
      if (inclTimeExists)
      {
        auto v2 = prof.m_inclDurations[i] + v;
        os << std::fixed << std::setw((int)ts) << std::setprecision(prec) << (v2 / scale)/* * total*/
           << std::fixed << std::setw(spaces[2]) << std::setprecision(prec) << (v2 / counted) * 100.0;
      }

      os << std::fixed << std::setw((int)ts) << std::setprecision(prec) << (v / scale)/* * total*/
         << std::fixed << std::setw(spaces[2]) << std::setprecision(prec) << (v / counted) * 100.0;
      os << EOL;
    }
  }
  os << EOL;

  os << std::setw(spaces[0]) << " " << std::setw(spaces[1]) << std::left << "TOTAL" << std::internal << std::fixed
     << std::setw((int)ts) << " " << std::fixed << std::setw(spaces[2]) << " " << std::fixed << std::setw((int)ts)
     << std::setprecision(prec) << (counted / scale)/*total*/
     << std::fixed << std::setw(spaces[2]) << std::setprecision(prec) << 100.00;
  os << EOL;

#if 0
  // total counted
  const TimeProfiler::duration total_counted = std::accumulate(prof.m_exclDurations.begin(), prof.m_exclDurations.end(), TimeProfiler::duration{});
  os << std::setw( spaces[0] ) << " "
      << std::setw( spaces[1] ) << std::left << "TOTAL + P_IGNORE" << std::internal
      << std::fixed << std::setw((int)ts) << std::setprecision(prec) << (total_counted / scale)/*total*/;
  os << EOL;
#endif
  return os;
}

#endif
//! \}
