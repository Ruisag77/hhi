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

/** \file     BinDecoder.cpp
 *  \brief    Low level binary symbol writer
 */

#include "BinDecoder.h"
#include "CommonLib/Rom.h"
#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif

#if ENABLE_CABAC_DUMP
#include <memory>
#include <fstream>
#endif

#include "CommonLib/dtrace_next.h"

#if ENABLE_CABAC_DUMP
namespace CabacRetrain
{
extern std::vector<std::pair<uint16_t, uint16_t>>  vprobaInit;
extern std::vector<int>                            vrate;
extern std::vector<int>                            vdrate0;
extern std::vector<int>                            vdrate1;
extern std::vector<int>                            vweight;
extern int                                         ctx;
extern bool                                        tempCABAC;
extern bool                                        activate;
static std::vector<std::vector<int8_t>>            vbins;
uint64_t                                           filesize = 0;
int                                                startctx = 0;
int                                                endctx   = 0;
SliceType                                          sliceReport;
static std::vector<std::unique_ptr<std::ofstream>> files;

static void dumpBin(int bin, int ctx_)
{
  if (!activate)
  {
    return;
  }

  vbins[ctx_].push_back(bin);
}

void endFrame(int poc, int qp, bool switchBp, SliceType st)
{
  if (!activate)
  {
    return;
  }

  for (int ctxi = startctx; ctxi < endctx; ++ctxi)
  {
    auto &file = *files[ctxi];
    file << poc << ' ';
    switch (st)
    {
    case B_SLICE:
      file << "B ";
      break;
    case P_SLICE:
      file << "P ";
      break;
    case I_SLICE:
      file << "I ";
      break;
    case L_SLICE:
      file << "L ";
      break;
    default:
      file << "? ";
      break;
    }
    file << qp << " " << (int)switchBp << ' ' << (int)tempCABAC << ' ';
    switch (sliceReport)
    {
    case B_SLICE:
      file << "B ";
      break;
    case P_SLICE:
      file << "P ";
      break;
    case I_SLICE:
      file << "I ";
      break;
    case L_SLICE:
      file << "L ";
      break;
    default:
      file << "? ";
      break;
    }

    file << vprobaInit[ctxi].first << ' ' << vprobaInit[ctxi].second << ' ' << (int)vrate[ctxi] << ' '
         << (int)vweight[ctxi] << ' ' << (int)vdrate0[ctxi] << ' ' << (int)vdrate1[ctxi] << ' ';

    for (auto b: vbins[ctxi])
    {
      file << (int)b;
    }

    file << std::endl;
    vbins[ctxi].clear();
    vprobaInit[ctxi] = { 0, 0 };
  }

  tempCABAC = false;
}

void init(const std::string &fn, bool iactivate)
{
  activate = iactivate;
  if (!activate)
  {
    return;
  }
    // get file size
  std::ifstream file(fn, std::ios::binary);
  file.seekg(0, std::ios_base::end);
  filesize = file.tellg();

  auto        n = fn.find_last_of("/\\");
  std::string s = fn;

  if (n != std::string::npos)
  {
    s = fn.substr(n + 1);
  }

  n        = s.find_last_of(".");
  startctx = 0;
  endctx   = (int)ContextSetCfg::getInitTable(0).size();
  files.resize(endctx - startctx);
  vbins.resize(endctx - startctx);
  vprobaInit.resize(endctx - startctx);
  vdrate0.resize(endctx - startctx);
  vdrate1.resize(endctx - startctx);
  vrate.resize(endctx - startctx);
  vweight.resize(endctx - startctx);

  for (int ctxi = startctx; ctxi < endctx; ++ctxi)
  {
    s = s.substr(0, n) + ".cabac_" + std::to_string(ctxi);
    files[ctxi].reset(new std::ofstream { s, std::ios::binary });
    auto &file = *files[ctxi];
    file << "ctx " << ctxi << "\n";
    if (ctxi < 0 || ctxi >= (int)ContextSetCfg::getInitTable(0).size())
    {
      std::cerr << "[ERROR] ctx idx out or range ([0-" << ContextSetCfg::getInitTable(0).size() << "[ )" << std::endl;
      exit(-1);
    }
    constexpr int N = 4 * 3 + 4 * 2;
    for (int k = 0; k < N; ++k)
    {
      file << (int)ContextSetCfg::getInitTable(k)[ctxi] << ' ';
    }
    file << "\n";
    file << "# SIZE " << filesize << '\n';
    file << "# POC SLICETYPE QP SWITCHBP TEMPCABAC SLICEREPORT S0 S1 RATE WEIGHT DRATE0 DRATE1 BINS[]\n";
  }
}
}   // namespace CabacRetrain
#endif

template<class BinProbModel> BinDecoderBase::BinDecoderBase(const BinProbModel *dummy)
  : Ctx(dummy)
  , m_bitstream(nullptr)
  , m_range(0)
  , m_value(0)
  , m_bitsNeeded(0)
{}

void BinDecoderBase::init(InputBitstream *bitstream) { m_bitstream = bitstream; }

void BinDecoderBase::uninit() { m_bitstream = nullptr; }

void BinDecoderBase::start()
{
  CHECK(m_bitstream->getNumBitsUntilByteAligned(), "Bitstream is not byte aligned.");
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::UpdateCABACStat(STATS__CABAC_INITIALISATION, 512, 510, 0);
#endif
  m_range      = 510;
  m_value      = (m_bitstream->readByte() << 8) + m_bitstream->readByte();
  m_bitsNeeded = -8;
}

void BinDecoderBase::finish()
{
  unsigned lastByte;
  m_bitstream->peekPreviousByte(lastByte);
  CHECK(((lastByte << (8 + m_bitsNeeded)) & 0xff) != 0x80, "No proper stop/alignment pattern at end of CABAC stream.");
}

void BinDecoderBase::reset(int qp, int initId)
{
  Ctx::init(qp, initId);
  start();
}

void BinDecoderBase::riceStatReset(int bitDepth, bool persistentRiceAdaptationEnabledFlag)
{
  Ctx::riceStatReset(bitDepth, persistentRiceAdaptationEnabledFlag);
}

unsigned BinDecoderBase::decodeBinEP()
{
  m_value += m_value;
  if (++m_bitsNeeded >= 0)
  {
    m_value += m_bitstream->readByte();
    m_bitsNeeded = -8;
  }

  unsigned bin         = 0;
  unsigned scaledRange = m_range << 7;
  if (m_value >= scaledRange)
  {
    m_value -= scaledRange;
    bin = 1;
  }
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(*ptype, 1, int(bin));
#endif
  DTRACE(g_trace_ctx, D_CABAC, "%d  %d  EP=%d \n", DTRACE_GET_COUNTER(g_trace_ctx, D_CABAC), m_range, bin);
  return bin;
}

unsigned BinDecoderBase::decodeBinsEP(unsigned numBins)
{
#if ENABLE_TRACING
  int numBinsOrig = numBins;
#endif

  if (m_range == 256)
  {
    return decodeAlignedBinsEP(numBins);
  }
  unsigned remBins = numBins;
  unsigned bins    = 0;
  while (remBins > 8)
  {
    m_value              = (m_value << 8) + (m_bitstream->readByte() << (8 + m_bitsNeeded));
    unsigned scaledRange = m_range << 15;
    for (int i = 0; i < 8; i++)
    {
      bins += bins;
      scaledRange >>= 1;
      if (m_value >= scaledRange)
      {
        bins++;
        m_value -= scaledRange;
      }
    }
    remBins -= 8;
  }
  m_bitsNeeded += remBins;
  m_value <<= remBins;
  if (m_bitsNeeded >= 0)
  {
    m_value += m_bitstream->readByte() << m_bitsNeeded;
    m_bitsNeeded -= 8;
  }
  unsigned scaledRange = m_range << (remBins + 7);
  for (int i = 0; i < remBins; i++)
  {
    bins += bins;
    scaledRange >>= 1;
    if (m_value >= scaledRange)
    {
      bins++;
      m_value -= scaledRange;
    }
  }
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(*ptype, numBins, int(bins));
#endif
#if ENABLE_TRACING
  for (int i = 0; i < numBinsOrig; i++)
  {
    DTRACE(g_trace_ctx, D_CABAC, "%d  %d  EP=%d \n", DTRACE_GET_COUNTER(g_trace_ctx, D_CABAC), m_range,
           (bins >> (numBinsOrig - 1 - i)) & 1);
  }
#endif
  return bins;
}

unsigned BinDecoderBase::decodeRemAbsEP(unsigned goRicePar, unsigned cutoff, int maxLog2TrDynamicRange)
{
  unsigned prefix = 0;
  {
    const unsigned maxPrefix = 32 - maxLog2TrDynamicRange;
    unsigned       codeWord  = 0;
    do
    {
      prefix++;
      codeWord = decodeBinEP();
    } while (codeWord && prefix < maxPrefix);
    prefix -= 1 - codeWord;
  }

  unsigned length = goRicePar, offset;
  if (prefix < cutoff)
  {
    offset = prefix << goRicePar;
  }
  else
  {
    offset = (((1 << (prefix - cutoff)) + cutoff - 1) << goRicePar);
    {
      length += (prefix == (32 - maxLog2TrDynamicRange) ? maxLog2TrDynamicRange - goRicePar : prefix - cutoff);
    }
  }
  return offset + decodeBinsEP(length);
}

unsigned BinDecoderBase::decodeBinTrm()
{
  m_range -= 2;
  unsigned scaledRange = m_range << 7;
  if (m_value >= scaledRange)
  {
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    CodingStatistics::UpdateCABACStat(STATS__CABAC_TRM_BITS, m_range + 2, 2, 1);
    CodingStatistics::IncrementStatisticEP(STATS__BYTE_ALIGNMENT_BITS, -m_bitsNeeded, 0);
#endif
    return 1;
  }
  else
  {
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    CodingStatistics::UpdateCABACStat(STATS__CABAC_TRM_BITS, m_range + 2, m_range, 0);
#endif
    if (m_range < 256)
    {
      m_range += m_range;
      m_value += m_value;
      if (++m_bitsNeeded == 0)
      {
        m_value += m_bitstream->readByte();
        m_bitsNeeded = -8;
      }
    }
    return 0;
  }
}

void BinDecoderBase::align()
{
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::UpdateCABACStat(STATS__CABAC_EP_BIT_ALIGNMENT, m_range, 256, 0);
#endif
  m_range = 256;
}

unsigned BinDecoderBase::decodeAlignedBinsEP(unsigned numBins)
{
#if ENABLE_TRACING
  int numBinsOrig = numBins;
#endif
  unsigned remBins = numBins;
  unsigned bins    = 0;
  while (remBins > 0)
  {
    // The MSB of m_value is known to be 0 because range is 256. Therefore:
    //   > The comparison against the symbol range of 128 is simply a test on the next-most-significant bit
    //   > "Subtracting" the symbol range if the decoded bin is 1 simply involves clearing that bit.
    //  As a result, the required bins are simply the <binsToRead> next-most-significant bits of m_value
    //  (m_value is stored MSB-aligned in a 16-bit buffer - hence the shift of 15)
    //
    //    m_value = |0|V|V|V|V|V|V|V|V|B|B|B|B|B|B|B|
    //    (V = usable bit, B = potential buffered bit (buffer refills when m_bitsNeeded >= 0))
    //
    unsigned binsToRead =
      std::min<unsigned>(remBins, 8); // read bytes if able to take advantage of the system's byte-read function
    unsigned binMask = (1 << binsToRead) - 1;
    unsigned newBins = (m_value >> (15 - binsToRead)) & binMask;
    bins             = (bins << binsToRead) | newBins;
    m_value          = (m_value << binsToRead) & 0x7FFF;
    remBins -= binsToRead;
    m_bitsNeeded += binsToRead;
    if (m_bitsNeeded >= 0)
    {
      m_value |= m_bitstream->readByte() << m_bitsNeeded;
      m_bitsNeeded -= 8;
    }
  }
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(*ptype, numBins, int(bins));
#endif
#if ENABLE_TRACING
  for (int i = 0; i < numBinsOrig; i++)
  {
    DTRACE(g_trace_ctx, D_CABAC, "%d  %d  EP=%d \n", DTRACE_GET_COUNTER(g_trace_ctx, D_CABAC), m_range,
           (bins >> (numBinsOrig - 1 - i)) & 1);
  }
#endif
  return bins;
}

template<class BinProbModel> TBinDecoder<BinProbModel>::TBinDecoder()
  : BinDecoderBase(static_cast<const BinProbModel *>(nullptr))
  , m_ctx(static_cast<CtxStore<BinProbModel> &>(*this))
{}

template<class BinProbModel> unsigned TBinDecoder<BinProbModel>::decodeBin(unsigned ctxId)
{
  BinProbModel &probModel = m_ctx[ctxId];
  unsigned      bin       = probModel.mps();
  uint32_t      lpsRange  = probModel.getLPS(m_range);

  DTRACE(g_trace_ctx, D_CABAC, "%d %d %d  [%d:%d]  %2d(MPS=%d)  ", DTRACE_GET_COUNTER(g_trace_ctx, D_CABAC), ctxId,
         m_range, m_range - lpsRange, lpsRange, (unsigned int)(probModel.state()),
         m_value < ((m_range - lpsRange) << 7));

  m_range -= lpsRange;
  uint32_t scaledRange = m_range << 7;
  if (m_value < scaledRange)
  {
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    CodingStatistics::UpdateCABACStat(*ptype, m_range + lpsRange, m_range, int(bin));
#endif
    // MPS path
    if (m_range < 256)
    {
      int numBits = probModel.getRenormBitsRange(m_range);
      m_range <<= numBits;
      m_value <<= numBits;
      m_bitsNeeded += numBits;
      if (m_bitsNeeded >= 0)
      {
        m_value += m_bitstream->readByte() << m_bitsNeeded;
        m_bitsNeeded -= 8;
      }
    }
  }
  else
  {
    bin = 1 - bin;
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    CodingStatistics::UpdateCABACStat(*ptype, m_range + lpsRange, lpsRange, int(bin));
#endif
    // LPS path
    int numBits = probModel.getRenormBitsLPS(lpsRange);
    m_value -= scaledRange;
    m_value = m_value << numBits;
    m_range = lpsRange << numBits;
    m_bitsNeeded += numBits;
    if (m_bitsNeeded >= 0)
    {
      m_value += m_bitstream->readByte() << m_bitsNeeded;
      m_bitsNeeded -= 8;
    }
  }
#if ENABLE_CABAC_DUMP
  CabacRetrain::dumpBin(bin, ctxId);
#endif
  probModel.update(bin);
  m_binBuffer.addBin(bin, ctxId);

  // DTRACE_DECR_COUNTER( g_trace_ctx, D_CABAC );
  DTRACE_WITHOUT_COUNT(g_trace_ctx, D_CABAC,
                       "  -  "
                       "%d"
                       "\n",
                       bin);
  return bin;
}

template class TBinDecoder<BinProbModel_Std>;
