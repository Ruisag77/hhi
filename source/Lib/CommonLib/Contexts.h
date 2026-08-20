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

/** \file     Contexts.h
 *  \brief    Classes providing probability descriptions and contexts (header)
 */

#ifndef __CONTEXTS__
#define __CONTEXTS__

#include "CommonDef.h"
#include "Slice.h"

#include <vector>

static constexpr int     PROB_BITS   = 15;   // Nominal number of bits to represent probabilities
static constexpr int     PROB_BITS_0 = 15;   // Number of bits to represent 1st estimate
static constexpr int     PROB_BITS_1 = 15;   // Number of bits to represent 2nd estimate
static constexpr int     MASK_0      = ~(~0u << PROB_BITS_0) << (PROB_BITS - PROB_BITS_0);
static constexpr int     MASK_1      = ~(~0u << PROB_BITS_1) << (PROB_BITS - PROB_BITS_1);
static constexpr uint8_t DWS         = 8;   // 0x47 Default window sizes
static constexpr uint8_t DWE         = 18;  // default weights
static constexpr uint8_t DWO         = 119; // default window offsets

struct BinFracBits
{
  uint32_t intBits[2];
};

enum class BpmType : int
{
  NONE = -1,
  // List of Binary Probability Models for entropy coding
  // The VVC standard currently defines a single model (STD)
  STD  = 0,
  NUM
};

struct BinStoreElem
{
  uint8_t  bins;
  uint8_t  count;
  unsigned ctxId;

  BinStoreElem(unsigned b, unsigned c) : bins(b), count(1), ctxId(c) {}

  unsigned bin(int i) const { return (bins >> i) & 1; }

  void addBin(unsigned bin) { bins |= bin << count++; }
};

typedef std::vector<BinStoreElem> BinStoreVector;

class BinBuffer
{
public:
  BinBuffer() : m_active(false), m_maxSize(0), m_numCtx(0), m_maxBinsPerCtx(0), m_buffer(nullptr) {}
  ~BinBuffer() {}

  void init(int size, int numCtx, int maxBinsPerCtx)
  {
    m_active           = false;
    m_maxSize          = size;
    m_numCtx           = numCtx;
    m_maxBinsPerCtx    = maxBinsPerCtx;
    m_indOfCtxInBuffer = std::vector<int16_t>(numCtx, -1);

    if (m_maxBinsPerCtx > 8)
    {
      fprintf(stdout, "Chance type of BinStoreElem/bins to support over 8 bins per context\n");
      exit(0);
    }
  }

  void addBin(unsigned bin, unsigned ctxId)
  {
    if (m_active)
    {
      int bufferInd = m_indOfCtxInBuffer[ctxId];

      if (bufferInd >= 0) // Already active context type
      {
        BinStoreElem &store = (*m_buffer)[bufferInd];

        if (store.count < m_maxBinsPerCtx) // Room to add more bins
        {
          store.addBin(bin);
        }
      }
      else if (m_buffer->size() < m_maxSize) // Room to add more context types
      {
        m_indOfCtxInBuffer[ctxId] = static_cast<int16_t>(m_buffer->size());
        m_buffer->push_back(BinStoreElem(bin, ctxId));
      }
    }
  }

  void clear()
  {
    m_buffer->clear();
    std::fill(m_indOfCtxInBuffer.begin(), m_indOfCtxInBuffer.end(), -1);
  }
  void setActive(bool b) { m_active = b; }

  const BinStoreVector *getBinStoreVector() const { return m_buffer; }
  void                  setBinStoreVector(BinStoreVector *bb)
  {
    m_buffer = bb;
    if (bb)
    {
      clear();
    }
  }

private:
  bool                 m_active;
  std::size_t          m_maxSize;
  int                  m_numCtx;
  int                  m_maxBinsPerCtx;
  std::vector<int16_t> m_indOfCtxInBuffer; // Only used in active CTU for convenience (8 bit enough)
  BinStoreVector      *m_buffer;
};

class BinBufferer
{
public:
  BinBufferer() : m_binBuffer() {}

  void                  setBinBufferActive(bool b) { m_binBuffer.setActive(b); }
  void                  setBinBuffer(BinStoreVector *bb) { m_binBuffer.setBinStoreVector(bb); }
  const BinStoreVector *getBinBuffer() const { return m_binBuffer.getBinStoreVector(); }
  void         initBufferer(int size, int numCtx, int maxBinsPerCtx) { m_binBuffer.init(size, numCtx, maxBinsPerCtx); }
  virtual void updateCtxs(BinStoreVector *bb) = 0;

protected:
  BinBuffer m_binBuffer;
};

class ProbModelTables
{
protected:
  static const BinFracBits m_binFracBits[512];
  static const uint8_t     m_RenormTable_32[32];          // Std         MP   MPI
};

class BinProbModelBase : public ProbModelTables
{
public:
  BinProbModelBase() {}
  ~BinProbModelBase() {}
  static uint32_t estFracBitsEP() { return (1 << SCALE_BITS); }
  static uint32_t estFracBitsEP(unsigned numBins) { return (numBins << SCALE_BITS); }
};

const uint8_t weightedAdaptRate[5] = { 10, 12, 16, 20, 22 };

class BinProbModel_Std : public BinProbModelBase
{
public:
  BinProbModel_Std()
  {
    uint16_t half   = 1 << (PROB_BITS - 1);
    m_state[0]      = half;
    m_state[1]      = half;
    m_rate          = DWS;
    m_weight        = DWE;
    m_stateUsed[0]  = half;
    m_stateUsed[1]  = half;
    m_rateOffset[0] = DWO;
    m_rateOffset[1] = DWO;
  }
  ~BinProbModel_Std() {}

public:
  void init(int qp, int initId);

  void update(unsigned bin)
  {
    int rate0 = m_rate >> 4;
    int rate1 = m_rate & 15;

    auto ws = m_rateOffset[bin];

    int rateUsed0 = std::max(2, rate0 + (ws >> 4) - ADJUSTMENT_RANGE);
    int rateUsed1 = std::max(2, rate1 + (ws & 15) - ADJUSTMENT_RANGE);

    m_stateUsed[0] = m_state[0] - ((m_state[0] >> rateUsed0) & MASK_0);
    m_stateUsed[1] = m_state[1] - ((m_state[1] >> rateUsed1) & MASK_1);

    m_state[0] -= (m_state[0] >> rate0) & MASK_0;
    m_state[1] -= (m_state[1] >> rate1) & MASK_1;

    if (bin)
    {
      m_stateUsed[0] += (0x7FFFU >> rateUsed0) & MASK_0;
      m_stateUsed[1] += (0x7FFFU >> rateUsed1) & MASK_1;

      m_state[0] += (0x7fffu >> rate0) & MASK_0;
      m_state[1] += (0x7fffu >> rate1) & MASK_1;
    }
  }

  void updateShortWin(unsigned bin)
  {
    int  rate0     = m_rate >> 4;
    auto ws        = m_rateOffset[bin];
    int  rateUsed0 = std::max(2, rate0 + (ws >> 4) - ADJUSTMENT_RANGE);

    m_stateUsed[0] = m_state[0] - ((m_state[0] >> rateUsed0) & MASK_0);
    m_state[0] -= (m_state[0] >> rate0) & MASK_0;

    if (bin)
    {
      m_stateUsed[0] += (0x7FFFU >> rateUsed0) & MASK_0;
      m_state[0] += (0x7fffu >> rate0) & MASK_0;
    }
  }

  void setLog2WindowSize(uint8_t log2WindowSize)
  {
    int rate0 = 2 + ((log2WindowSize >> 2) & 3);
    int rate1 = 3 + rate0 + (log2WindowSize & 3);
    m_rate    = 16 * rate0 + rate1;
    CHECK(rate1 > 9, "Second window size is too large!");
  }
  void    setAdaptRateWeight(uint8_t weight) { m_weight = weight; }
  uint8_t getAdaptRateWeight() const { return m_weight; }
  void    setWinSizes(uint8_t rate) { m_rate = rate; }
  uint8_t getWinSizes() const { return m_rate; }
  void    setAdaptRateOffset(uint8_t rateOffset, bool bin) { m_rateOffset[bin] = rateOffset; }
  uint8_t getAdaptRateOffset(bool bin) const { return m_rateOffset[bin]; }
  void    estFracBitsUpdate(unsigned bin, uint64_t &b)
  {
    b += estFracBits(bin);
    update(bin);
  }
  uint32_t           estFracBits(unsigned bin) const { return getFracBitsArray().intBits[bin]; }
  static uint32_t    estFracBitsTrm(unsigned bin) { return (bin ? 0x3bfbb : 0x0010c); }
  const BinFracBits &getFracBitsArray() const { return m_binFracBits[state_est()]; }

public:
  uint16_t state_est() const
  {
    uint8_t  wIdx0 = (m_weight >> 3);
    uint8_t  wIdx1 = (m_weight & 0x07);
    uint32_t w0    = weightedAdaptRate[wIdx0];
    uint32_t w1    = weightedAdaptRate[wIdx1];
    uint32_t pd;
    if ((w0 + w1) <= 32)
    {
      pd = (uint32_t(m_stateUsed[0]) * w0 + uint32_t(m_stateUsed[1]) * w1) >> 11;
    }
    else
    {
      pd = (uint32_t(m_stateUsed[0]) * w0 + uint32_t(m_stateUsed[1]) * w1) >> 12;
    }
    return uint16_t(pd);
  }

  uint16_t state() const
  {
    uint8_t  wIdx0 = (m_weight >> 3);
    uint8_t  wIdx1 = (m_weight & 0x07);
    uint32_t w0    = weightedAdaptRate[wIdx0];
    uint32_t w1    = weightedAdaptRate[wIdx1];
    uint32_t pd;
    if ((w0 + w1) <= 32)
    {
      pd = (uint32_t(m_stateUsed[0]) * w0 + uint32_t(m_stateUsed[1]) * w1) >> 5;
    }
    else
    {
      pd = (uint32_t(m_stateUsed[0]) * w0 + uint32_t(m_stateUsed[1]) * w1) >> 6;
    }
    return uint16_t(pd);
  }

  uint8_t mps() const { return state() >> 14; }
  uint8_t getLPS(unsigned range) const
  {
    uint16_t q = state();
    if (q & 0x4000)
    {
      q = q ^ 0x7fff;
    }
    return ((range * (q >> 6)) >> 9) + 1;
  }

  static uint8_t                getRenormBitsLPS(unsigned lpsRange) { return m_RenormTable_32[lpsRange >> 3]; }
  static uint8_t                getRenormBitsRange(unsigned range) { return 1; }
  std::pair<uint16_t, uint16_t> getState() const { return std::pair<uint16_t, uint16_t>(m_state[0], m_state[1]); }

  void setState(std::pair<uint16_t, uint16_t> pState)
  {
    m_state[0]     = pState.first;
    m_state[1]     = pState.second;
    m_stateUsed[0] = m_state[0];
    m_stateUsed[1] = m_state[1];
  }

public:
  uint64_t estFracExcessBits(const BinProbModel_Std &r) const
  {
    int n = 2 * state_est() + 1;
    return ((1024 - n) * r.estFracBits(0) + n * r.estFracBits(1) + 512) >> 10;
  }

private:
  uint16_t m_state[2];
  uint16_t m_stateUsed[2];
  uint8_t  m_rateOffset[2];
  uint8_t  m_weight;
  uint8_t  m_rate;
};

class CtxSet
{
public:
  CtxSet(uint16_t offset, uint16_t size) : Offset(offset), Size(size) {}
  CtxSet(const CtxSet &ctxSet) : Offset(ctxSet.Offset), Size(ctxSet.Size) {}
  CtxSet(std::initializer_list<CtxSet> ctxSets);

public:
  uint16_t operator()() const { return Offset; }
  uint16_t operator()(uint16_t inc) const
  {
    CHECKD(inc >= Size,
           "Specified context increment (" << inc << ") exceed range of context set [0;" << Size - 1 << "].");
    return Offset + inc;
  }
  bool operator==(const CtxSet &ctxSet) const { return (Offset == ctxSet.Offset && Size == ctxSet.Size); }
  bool operator!=(const CtxSet &ctxSet) const { return (Offset != ctxSet.Offset || Size != ctxSet.Size); }

public:
  uint16_t Offset;
  uint16_t Size;
};

class ContextSetCfg
{
public:
  // context sets: specify offset and size
  static const CtxSet SplitFlag;
  static const CtxSet SplitQtFlag;
  static const CtxSet SplitHvFlag;
  static const CtxSet Split12Flag;
  static const CtxSet SkipFlag;
  static const CtxSet MergeFlag;
  static const CtxSet RegularMergeFlag;
  static const CtxSet MergeIdx;
  static const CtxSet BMMergeFlag;
  static const CtxSet PredMode;
  static const CtxSet MultiRefLineIdx;
  static const CtxSet IntraLumaMpmFlag;
  static const CtxSet IntraLumaSecondMpmFlag;
  static const CtxSet IntraLumaPlanarFlag;
  static const CtxSet IntraLumaMPMIdx;
  static const CtxSet CclmModeFlag;
  static const CtxSet CclmModeIdx;
  static const CtxSet IntraChromaPredMode;
  static const CtxSet MipFlag;
  static const CtxSet DimdFlag;
  static const CtxSet DimdChromaFlag;
  static const CtxSet TimdFlag;
  static const CtxSet TimdMergeFlag;
  static const CtxSet TimdSadFlag;
  static const CtxSet ObicFlag;
  static const CtxSet EipFlag;
  static const CtxSet MMLMFlag;
  static const CtxSet DeltaQP;
  static const CtxSet InterDir;
  static const CtxSet RefPic;
  static const CtxSet MmvdFlag;
  static const CtxSet MmvdMergeIdx;
  static const CtxSet MmvdStepMvpIdx;
  static const CtxSet GeoMmvdFlag;
  static const CtxSet GeoMmvdStepMvpIdx;
  static const CtxSet GPMIntraFlag;
  static const CtxSet GeoBldFlag;
  static const CtxSet SubblockMergeFlag;
  static const CtxSet AffineFlag;
  static const CtxSet AffineType;
  static const CtxSet AffMergeIdx;
  static const CtxSet AffMmvdFlag;
  static const CtxSet AffMmvdIdx;
  static const CtxSet AffMmvdOffsetStep;
  static const CtxSet Mvd;
  static const CtxSet Bvd;
  static const CtxSet BDPCMMode;
  static const CtxSet QtRootCbf;
  static const CtxSet QtCbf[3];    // [ channel ]
  static const CtxSet SigCoeffGroup[2];    // [ ChannelType ]
  static const CtxSet LastX[2];    // [ ChannelType ]
  static const CtxSet LastY[2];    // [ ChannelType ]
  static const CtxSet lastXSecondaryPrefix;
  static const CtxSet lastYSecondaryPrefix;
  static const CtxSet lastXSuffix[3];    // [ channel ]
  static const CtxSet lastYSuffix[3];    // [ channel ]
  static const CtxSet SigFlag[6];    // [ ChannelType + State ]
  static const CtxSet GtxFlag[8];    // [ ChannelType + x ]
  static const CtxSet SigFlagNST[6];    // [ ChannelType + State ]
  static const CtxSet GtxFlagNST[8];    // [ ChannelType + x ]
  static const CtxSet TsSigCoeffGroup;
  static const CtxSet TsSigFlag;
  static const CtxSet TsParFlag;
  static const CtxSet TsGtxFlag;
  static const CtxSet TsLrg1Flag;
  static const CtxSet TsResidualSign;
  static const CtxSet SigCoeffGroupCtxSetSwitch[2];   // [ ChannelType ]
  static const CtxSet SigFlagCtxSetSwitch[6];         // [ ChannelType + State ]
  static const CtxSet GtxFlagCtxSetSwitch[8];         // [ ChannelType + x ]
  static const CtxSet LastXCtxSetSwitch[2];           // [ ChannelType ]
  static const CtxSet LastYCtxSetSwitch[2];           // [ ChannelType ]
  static const CtxSet TsSigCoeffGroupCtxSetSwitch;
  static const CtxSet TsSigFlagCtxSetSwitch;
  static const CtxSet TsParFlagCtxSetSwitch;
  static const CtxSet TsGtxFlagCtxSetSwitch;
  static const CtxSet TsLrg1FlagCtxSetSwitch;
  static const CtxSet TsResidualSignCtxSetSwitch;
  static const CtxSet MVPIdx;
  static const CtxSet SaoMergeFlag;
  static const CtxSet SaoTypeIdx;
  static const CtxSet BifCtrlFlags[];
  static const CtxSet TransformSkipFlag;
  static const CtxSet MTSIdx;
  static const CtxSet CcSaoControlIdc;
  static const CtxSet LFNSTIdxIntra;
  static const CtxSet LFNSTIdxInter;
  static const CtxSet PLTFlag;
  static const CtxSet RotationFlag;
  static const CtxSet RunTypeFlag;
  static const CtxSet IdxRunModel;
  static const CtxSet CopyRunModel;
  static const CtxSet SbtFlag;
  static const CtxSet SbtQuadFlag;
  static const CtxSet SbtHorFlag;
  static const CtxSet SbtPosFlag;
  static const CtxSet ChromaQpAdjFlag;
  static const CtxSet ChromaQpAdjIdc;
  static const CtxSet ImvFlag;
  static const CtxSet ImvFlagIBC;
  static const CtxSet bcwIdx;
  static const CtxSet SgpmFlag;
  static const CtxSet ObmcFlag;
  static const CtxSet alfCtbFlag;
  static const CtxSet ctbAlfAlternativeEcm;
  static const CtxSet ctbAlfAlternativeVtm;
  static const CtxSet alfUseApsFlag;
  static const CtxSet CcAlfFilterControlFlag;
  static const CtxSet CiipFlag;
  static const CtxSet SmvdFlag;
  static const CtxSet IBCFlag;
  static const CtxSet JointCbCrFlag;
  static const CtxSet SignPredFlag[2];
  static const CtxSet CccmFlag;
  static const CtxSet CccmMpfFlag;
  static const CtxSet BvgCccmFlag;
  static const CtxSet PlanarDir;
  static const CtxSet CclmDeltaFlags;
  static const CtxSet LICFlag;
  static const CtxSet MergeFlagOppositeLic;
  static const CtxSet AffineFlagOppositeLic;
  static const CtxSet nonLocalCCP;
  static const CtxSet CcMergeFusionFlag;
  static const CtxSet decoderDerivedCCP;
  static const CtxSet CcInsideFilterFlag;
#if ENABLE_NNLF
  static const CtxSet nnlfUnifiedParams;
#endif
  static const CtxSet   LfCccmFlag;
  static const unsigned NumberOfContexts;

  // combined sets for less complex copying
  // NOTE: The contained CtxSet's should directly follow each other in the initalization list;
  //       otherwise, you will copy more elements than you want !!!
  static const CtxSet Sao;
  static const CtxSet Alf;
  static const CtxSet Palette;
  static const CtxSet ctxPartition;

public:
  static const std::vector<uint8_t> &getInitTable(unsigned initId);

private:
  static std::vector<std::vector<uint8_t>> sm_InitTables;
  static CtxSet                            addCtxSet(std::initializer_list<std::initializer_list<uint8_t>> initSet2d);
};

class FracBitsAccess
{
public:
  virtual const BinFracBits &getFracBitsArray(unsigned ctxId) const = 0;
};

template<class BinProbModel> class CtxStore : public FracBitsAccess
{
public:
  CtxStore();
  CtxStore(bool dummy);
  CtxStore(const CtxStore<BinProbModel> &ctxStore);

public:
  void copyFrom(const CtxStore<BinProbModel> &src)
  {
    checkInit();
    std::copy_n(reinterpret_cast<const char *>(src.m_ctx), sizeof(BinProbModel) * ContextSetCfg::NumberOfContexts,
                reinterpret_cast<char *>(m_ctx));
  }
  void copyFrom(const CtxStore<BinProbModel> &src, const CtxSet &ctxSet)
  {
    checkInit();
    std::copy_n(reinterpret_cast<const char *>(src.m_ctx + ctxSet.Offset), sizeof(BinProbModel) * ctxSet.Size,
                reinterpret_cast<char *>(m_ctx + ctxSet.Offset));
  }
  void init(int qp, int initId);
  void setWinSizes(const std::vector<uint8_t> &log2WindowSizes);
  void loadWinSizes(const std::vector<uint8_t> &windows);
  void saveWinSizes(std::vector<uint8_t> &windows) const;
  void loadWeights(const std::vector<uint8_t> &weights);
  void saveWeights(std::vector<uint8_t> &weights) const;
  void loadPStates(const std::vector<std::pair<uint16_t, uint16_t>> &probStates);
  void savePStates(std::vector<std::pair<uint16_t, uint16_t>> &probStates) const;
  void loadRateOffsets(const std::vector<uint8_t> &rateOffsets0, const std::vector<uint8_t> &rateOffsets1);
  void saveRateOffsets(std::vector<uint8_t> &rateOffsets0, std::vector<uint8_t> &rateOffsets1) const;

  void updateCtxs(BinStoreVector *binStoreElemVector)
  {
    BinStoreVector &vector = *binStoreElemVector;

    int numCtxTypes = int(vector.size());

    for (int ctxType = 0; ctxType < numCtxTypes; ctxType++)
    {
      const BinStoreElem &store = vector[ctxType];

      for (int i = 0; i < store.count; i++)
      {
        m_ctxBuffer[store.ctxId].updateShortWin(store.bin(i));
      }
    }
  }

  const BinProbModel &operator[](unsigned ctxId) const { return m_ctx[ctxId]; }
  BinProbModel       &operator[](unsigned ctxId) { return m_ctx[ctxId]; }
  uint32_t            estFracBits(unsigned bin, unsigned ctxId) const { return m_ctx[ctxId].estFracBits(bin); }

  const BinFracBits &getFracBitsArray(unsigned ctxId) const { return m_ctx[ctxId].getFracBitsArray(); }

private:
  inline void checkInit()
  {
    if (m_ctx)
    {
      return;
    }
    m_ctxBuffer.resize(ContextSetCfg::NumberOfContexts);
    m_ctx = m_ctxBuffer.data();
  }

private:
  std::vector<BinProbModel> m_ctxBuffer;
  BinProbModel             *m_ctx;
};

class Ctx;
class SubCtx
{
  friend class Ctx;

public:
  SubCtx(const CtxSet &ctxSet, const Ctx &ctx) : m_CtxSet(ctxSet), m_ctx(ctx) {}
  SubCtx(const SubCtx &subCtx) : m_CtxSet(subCtx.m_CtxSet), m_ctx(subCtx.m_ctx) {}
  const SubCtx &operator=(const SubCtx &) = delete;

private:
  const CtxSet m_CtxSet;
  const Ctx   &m_ctx;
};

class Ctx : public ContextSetCfg
{
public:
  Ctx();
  Ctx(const BinProbModel_Std *dummy);
  Ctx(const Ctx &ctx);

public:
  const Ctx &operator=(const Ctx &ctx)
  {
    m_bpmType = ctx.m_bpmType;
    switch (m_bpmType)
    {
    case BpmType::STD:
      m_CtxStore_Std.copyFrom(ctx.m_CtxStore_Std);
      break;
    default:
      break;
    }
    ::memcpy(m_GRAdaptStats, ctx.m_GRAdaptStats, sizeof(unsigned) * RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS);
    return *this;
  }

  SubCtx operator=(SubCtx &&subCtx)
  {
    m_bpmType = subCtx.m_ctx.m_bpmType;
    switch (m_bpmType)
    {
    case BpmType::STD:
      m_CtxStore_Std.copyFrom(subCtx.m_ctx.m_CtxStore_Std, subCtx.m_CtxSet);
      break;
    default:
      break;
    }
    return std::move(subCtx);
  }

  void init(int qp, int initId)
  {
    switch (m_bpmType)
    {
    case BpmType::STD:
      m_CtxStore_Std.init(qp, initId);
      break;
    default:
      break;
    }
    for (std::size_t k = 0; k < RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS; k++)
    {
      m_GRAdaptStats[k] = 0;
    }
  }

  void riceStatReset(int bitDepth, bool persistentRiceAdaptationEnabledFlag)
  {
    for (std::size_t k = 0; k < RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS; k++)
    {
      if (persistentRiceAdaptationEnabledFlag)
      {
        CHECK(bitDepth <= 10, "BitDepth shall be larger than 10.");
        m_GRAdaptStats[k] = 2 * floorLog2(bitDepth - 10);
      }
      else
      {
        m_GRAdaptStats[k] = 0;
      }
    }
  }

  void loadWeights(const std::vector<uint8_t> &weights)
  {
    switch (m_bpmType)
    {
    case BpmType::STD:
      m_CtxStore_Std.loadWeights(weights);
      break;
    default:
      break;
    }
  }

  void saveWeights(std::vector<uint8_t> &weights) const
  {
    switch (m_bpmType)
    {
    case BpmType::STD:
      m_CtxStore_Std.saveWeights(weights);
      break;
    default:
      break;
    }
  }

  void loadWinSizes(const std::vector<uint8_t> &windows)
  {
    switch (m_bpmType)
    {
    case BpmType::STD:
      m_CtxStore_Std.loadWinSizes(windows);
      break;
    default:
      break;
    }
  }

  void saveWinSizes(std::vector<uint8_t> &windows) const
  {
    switch (m_bpmType)
    {
    case BpmType::STD:
      m_CtxStore_Std.saveWinSizes(windows);
      break;
    default:
      break;
    }
  }

  void loadPStates(const std::vector<std::pair<uint16_t, uint16_t>> &probStates)
  {
    switch (m_bpmType)
    {
    case BpmType::STD:
      m_CtxStore_Std.loadPStates(probStates);
      break;
    default:
      break;
    }
  }

  void savePStates(std::vector<std::pair<uint16_t, uint16_t>> &probStates) const
  {
    switch (m_bpmType)
    {
    case BpmType::STD:
      m_CtxStore_Std.savePStates(probStates);
      break;
    default:
      break;
    }
  }

  void loadRateOffsets(const std::vector<uint8_t> &rateOffsets0, const std::vector<uint8_t> &rateOffsets1)
  {
    switch (m_bpmType)
    {
    case BpmType::STD:
      m_CtxStore_Std.loadRateOffsets(rateOffsets0, rateOffsets1);
      break;
    default:
      break;
    }
  }

  void saveRateOffsets(std::vector<uint8_t> &rateOffsets0, std::vector<uint8_t> &rateOffsets1) const
  {
    switch (m_bpmType)
    {
    case BpmType::STD:
      m_CtxStore_Std.saveRateOffsets(rateOffsets0, rateOffsets1);
      break;
    default:
      break;
    }
  }

  void initCtxAndWinSize(unsigned ctxId, const Ctx &ctx, const uint8_t winSize)
  {
    switch (m_bpmType)
    {
    case BpmType::STD:
      m_CtxStore_Std[ctxId] = ctx.m_CtxStore_Std[ctxId];
      m_CtxStore_Std[ctxId].setLog2WindowSize(winSize);
      break;
    default:
      break;
    }
  }

  const unsigned &getGRAdaptStats(unsigned id) const { return m_GRAdaptStats[id]; }
  unsigned       &getGRAdaptStats(unsigned id) { return m_GRAdaptStats[id]; }

  const unsigned getBaseLevel() const { return m_baseLevel; }
  void           setBaseLevel(int value) { m_baseLevel = value; }

public:
  BpmType    getBpmType() const { return m_bpmType; }
  const Ctx &getCtx() const { return *this; }
  Ctx       &getCtx() { return *this; }

  explicit operator const CtxStore<BinProbModel_Std> &() const { return m_CtxStore_Std; }
  explicit operator CtxStore<BinProbModel_Std> &() { return m_CtxStore_Std; }

  const FracBitsAccess &getFracBitsAcess() const
  {
    switch (m_bpmType)
    {
    case BpmType::STD:
      return m_CtxStore_Std;
    default:
      THROW("BPMType out of range");
    }
  }

#if ENABLE_CABAC_DUMP
  // direct access to a model prm
  int getRate(int ctxidx) const { return m_CtxStore_Std[ctxidx].getWinSizes(); }
#endif
private:
  BpmType                    m_bpmType;
  CtxStore<BinProbModel_Std> m_CtxStore_Std;

protected:
  unsigned m_GRAdaptStats[RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS];
  int      m_baseLevel;
};

typedef Pool<Ctx> CtxPool;

class TempCtx
{
  TempCtx(const TempCtx &)                  = delete;
  const TempCtx &operator=(const TempCtx &) = delete;

public:
  TempCtx(CtxPool *pool) : m_ctx(*pool->get()), m_pool(pool) {}
  TempCtx(CtxPool *pool, const Ctx &ctx) : m_ctx(*pool->get()), m_pool(pool) { m_ctx = ctx; }
  TempCtx(CtxPool *pool, SubCtx &&subCtx) : m_ctx(*pool->get()), m_pool(pool) { m_ctx = std::forward<SubCtx>(subCtx); }
  ~TempCtx() { m_pool->giveBack(&m_ctx); }
  const Ctx &operator=(const Ctx &ctx) { return (m_ctx = ctx); }
  SubCtx     operator=(SubCtx &&subCtx) { return m_ctx = std::forward<SubCtx>(subCtx); }
  operator const Ctx &() const { return m_ctx; }
  operator Ctx &() { return m_ctx; }

private:
  Ctx     &m_ctx;
  CtxPool *m_pool;
};

class CtxStateBuf
{
public:
  CtxStateBuf() : m_valid(false) {}
  ~CtxStateBuf() {}
  inline void reset() { m_valid = false; }
  inline bool getIfValid(Ctx &ctx) const
  {
    if (m_valid)
    {
      ctx.loadPStates(m_states);
      ctx.loadWeights(m_weights);
      ctx.loadWinSizes(m_rate);
      ctx.loadRateOffsets(m_rateOffset[0], m_rateOffset[1]);
      return true;
    }
    return false;
  }
  inline void store(const Ctx &ctx)
  {
    ctx.savePStates(m_states);
    ctx.saveWeights(m_weights);
    ctx.saveWinSizes(m_rate);
    ctx.saveRateOffsets(m_rateOffset[0], m_rateOffset[1]);
    m_valid = true;
  }

private:
  std::vector<std::pair<uint16_t, uint16_t>> m_states;
  bool                                       m_valid;
  std::vector<uint8_t>                       m_weights;
  std::vector<uint8_t>                       m_rate;
  std::vector<uint8_t>                       m_rateOffset[2];
};

class CtxStateArray
{
public:
  CtxStateArray() {}
  ~CtxStateArray() {}

  inline void resetAll()
  {
    for (std::size_t k = 0; k < m_data.size(); k++)
    {
      m_data[k].reset();
    }
  }

  inline void resize(std::size_t reqSize)
  {
    if (m_data.size() < reqSize)
    {
      m_data.resize(reqSize);
    }
  }

  inline bool getIfValid(Ctx &ctx) const
  {
    if (m_data.size())
    {
      return m_data[0].getIfValid(ctx);
    }
    return false;
  }

#if ENABLE_CABAC_DUMP
  inline void store(const Ctx &ctx, const SliceType sliceType)
#else
  inline void store(const Ctx &ctx)
#endif
  {
    if (!m_data.size())
    {
      resize(1);
    }
    m_data[0].store(ctx);

#if ENABLE_CABAC_DUMP
    m_sliceType = sliceType;
#endif
  }

  inline size_t size() const { return m_data.size(); }

#if ENABLE_CABAC_DUMP
  SliceType getSliceType() const { return m_sliceType; }
#endif

private:
  std::vector<CtxStateBuf> m_data;
#if ENABLE_CABAC_DUMP
  SliceType m_sliceType;
#endif
};

class CtxStateStore
{
public:
  CtxStateStore()
  {
    static_assert((B_SLICE < NUMBER_OF_SLICE_TYPES - 1) && (P_SLICE < NUMBER_OF_SLICE_TYPES - 1), "index out of bound");
  }
  ~CtxStateStore() {}

  void storeCtx(const Slice *slice, const Ctx &ctx)
  {
    SliceType s = slice->m_eSliceType;

    if (s != I_SLICE)
    {
      int                 t             = (s == P_SLICE ? 1 : 0);
      CtxStateArray      *ctxStateArray = nullptr;
      std::pair<int, int> entry(slice->m_uiTLayer, slice->m_iSliceQp);

      if (m_stateBuf[t].find(entry) != m_stateBuf[t].end() || m_stateBuf[t].size() < TEMP_CABAC_BUFFER_SIZE)
      {
        ctxStateArray = &m_stateBuf[t][entry];
      }
      else
      {
        if (m_stateBuf[t].size() == TEMP_CABAC_BUFFER_SIZE)
        {
          m_stateBuf[t].erase(m_stateBuf[t].begin());
        }

        ctxStateArray = &m_stateBuf[t][entry];

        CHECK(m_stateBuf[t].size() > TEMP_CABAC_BUFFER_SIZE, "Wrong buffer size");
      }

#if ENABLE_CABAC_DUMP
      ctxStateArray->store(ctx, slice->m_cabacInitSliceType);
#else
      ctxStateArray->store(ctx);
#endif
    }
  }

#if ENABLE_CABAC_DUMP
  bool loadCtx(Slice *slice, Ctx &ctx)
#else
  bool loadCtx(const Slice *slice, Ctx &ctx)
#endif
  {
    SliceType s = slice->m_eSliceType;

    if (s != I_SLICE)
    {
      int                 t = (s == P_SLICE ? 1 : 0);
      std::pair<int, int> entry(slice->m_uiTLayer, slice->m_iSliceQp);

      if (m_stateBuf[t].find(entry) != m_stateBuf[t].end())
      {
        const CtxStateArray &ctxStateArray = m_stateBuf[t][entry];
#if ENABLE_CABAC_DUMP
        slice->m_cabacInitSliceType = m_stateBuf[t][entry].getSliceType();
#endif
        return ctxStateArray.getIfValid(ctx);
      }
    }
    return false;
  }

  void clearValid()
  {
    m_stateBuf[0].clear();
    m_stateBuf[1].clear();
  }

private:
  std::map<std::pair<int, int>, CtxStateArray> m_stateBuf[2];
};

class CABACDataStore
{
public:
#if ENABLE_CABAC_DUMP
  bool loadCtxStates(Slice *slice, Ctx &ctx) { return m_ctxStateStore.loadCtx(slice, ctx); }
#else
  bool loadCtxStates(const Slice *slice, Ctx &ctx) { return m_ctxStateStore.loadCtx(slice, ctx); }
#endif
  void storeCtxStates(const Slice *slice, const Ctx &ctx) { m_ctxStateStore.storeCtx(slice, ctx); }

  void updateBufferState(const Slice *slice)
  {
    if (slice->m_pendingRasInit ||
        (slice->m_eSliceType == B_SLICE &&
         slice->m_eNalUnitType == NAL_UNIT_CODED_SLICE_GDR)) // Note: isInterGDR in ECM, requires JVET_Z0118_GDR
    {
      m_ctxStateStore.clearValid();
    }
  }

private:
  CtxStateStore m_ctxStateStore;
};

#endif
