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

/** \file     EncSearch.cpp
 *  \brief    encoder intra search class
 */

#include "IntraSearch.h"
#include "EncModeCtrl.h"
#include "EncCfg.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Rom.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/BilateralFilter.h"
#include "CommonLib/dtrace_next.h"
#include "CommonLib/dtrace_buffer.h"

#include <math.h>
#include <limits>
#include <algorithm>

 //! \ingroup EncoderLib
 //! \{
#define PLTCtx(c) SubCtx(Ctx::Palette, c)
IntraSearch::IntraSearch()
  : m_encCfg(nullptr)
  , m_bilateralFilter(nullptr)
  , m_pcTrQuant(nullptr)
  , m_pcRdCost(nullptr)
  , m_pcReshape(nullptr)
  , m_CABACEstimator(nullptr)
  , m_ctxPool(nullptr)
  , m_isInitialized(false)
{
  m_minErrorIndexMap = nullptr;
  for (unsigned i = 0; i < (MAXPLTSIZE + 1); i++)
  {
    m_indexError[i] = nullptr;
  }
  for (unsigned i = 0; i < NUM_TRELLIS_STATE; i++)
  {
    m_statePtRDOQ[i] = nullptr;
  }
}

void IntraSearch::destroy()
{
  CHECK(!m_isInitialized, "Not initialized");

  const int numSaveLayersToAllocate = 2;

  if (m_pSaveCS)
  {
    for (uint32_t layer = 0; layer < numSaveLayersToAllocate; layer++)
    {
      m_pSaveCS[layer]->destroy();
      delete m_pSaveCS[layer];
    }

    delete[] m_pSaveCS;
  }

  if (m_pTempCS)
  {
    m_pTempCS->destroy();
    delete m_pTempCS;
  }

  if (m_pBestCS)
  {
    m_pBestCS->destroy();
    delete m_pBestCS;
  }

  m_pTempCS = nullptr;
  m_pBestCS = nullptr;
  m_pSaveCS = nullptr;

  m_tmpStorageCtu.destroy();
  m_isInitialized = false;
  if (m_indexError[0] != nullptr)
  {
    for (unsigned i = 0; i < (MAXPLTSIZE + 1); i++)
    {
      delete[] m_indexError[i];
      m_indexError[i] = nullptr;
    }
  }
  if (m_minErrorIndexMap != nullptr)
  {
    delete[] m_minErrorIndexMap;
    m_minErrorIndexMap = nullptr;
  }
  if (m_statePtRDOQ[0] != nullptr)
  {
    for (unsigned i = 0; i < NUM_TRELLIS_STATE; i++)
    {
      delete[] m_statePtRDOQ[i];
      m_statePtRDOQ[i] = nullptr;
    }
  }
}

IntraSearch::~IntraSearch()
{
  if (m_isInitialized)
  {
    destroy();
  }
}

void IntraSearch::init(const EncCfg *encCfg, BilateralFilter *bilateralFilter, TrQuant *pcTrQuant, RdCost *pcRdCost,
                       InterpolationFilter *pIf, CABACWriter *CABACEstimator, EncModeCtrl *pcEncModeCtrl,
                       CtxPool *ctxPool, const uint32_t maxCUWidth, const uint32_t maxCUHeight,
                       const uint32_t maxTotalCUDepth, EncReshape *pcReshape, const unsigned bitDepthY)
{
  CHECK(m_isInitialized, "Already initialized");

  m_encCfg          = encCfg;
  m_bilateralFilter = bilateralFilter;
  m_pcTrQuant       = pcTrQuant;
  m_pcRdCost        = pcRdCost;
  m_CABACEstimator  = CABACEstimator;
  m_ctxPool         = ctxPool;
  m_pcReshape       = pcReshape;
  m_modeCtrl        = pcEncModeCtrl;

  const ChromaFormat cform = encCfg->m_chromaFormatIdc;

  IntraPrediction::init(cform, encCfg->m_internalBitDepth[ChannelType::LUMA], pIf);
  m_tmpStorageCtu.create(UnitArea(cform, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));

  const int  ctuSize = encCfg->m_CTUSize;
  const Area ctuArea = Area(0, 0, ctuSize, ctuSize);

  m_pTempCS = new CodingStructure(m_unitPool);
  m_pBestCS = new CodingStructure(m_unitPool);

  m_pTempCS->create(cform, ctuArea, false, (bool)encCfg->m_PLTMode);
  m_pBestCS->create(cform, ctuArea, false, (bool)encCfg->m_PLTMode);

  const int numSaveLayersToAllocate = 2;

  m_pSaveCS = new CodingStructure *[numSaveLayersToAllocate];

  for (uint32_t depth = 0; depth < numSaveLayersToAllocate; depth++)
  {
    m_pSaveCS[depth] = new CodingStructure(m_unitPool);
    m_pSaveCS[depth]->create(UnitArea(cform, ctuArea), false, (bool)encCfg->m_PLTMode);
  }

  m_isInitialized = true;
  if (encCfg->m_PLTMode)
  {
    if (m_indexError[0] == nullptr)
    {
      for (unsigned i = 0; i < (MAXPLTSIZE + 1); i++)
      {
        m_indexError[i] = new double[MAX_CU_BLKSIZE_PLT * MAX_CU_BLKSIZE_PLT];
      }
    }
    if (m_minErrorIndexMap == nullptr)
    {
      m_minErrorIndexMap = new uint8_t[MAX_CU_BLKSIZE_PLT * MAX_CU_BLKSIZE_PLT];
    }
    if (m_statePtRDOQ[0] == nullptr)
    {
      for (unsigned i = 0; i < NUM_TRELLIS_STATE; i++)
      {
        m_statePtRDOQ[i] = new uint8_t[MAX_CU_BLKSIZE_PLT * MAX_CU_BLKSIZE_PLT];
      }
    }
  }
  memset(m_indexMapRDOQ, 0, sizeof(m_indexMapRDOQ));
  memset(m_runMapRDOQ, 0, sizeof(m_runMapRDOQ));
}

//////////////////////////////////////////////////////////////////////////
// INTRA PREDICTION
//////////////////////////////////////////////////////////////////////////

bool IntraSearch::estIntraPredLumaQT(CodingUnit &cu, Partitioner &partitioner, CUCtxIntra &cuCtxIntra,
                                     const double bestCostSoFar, CodingStructure *bestCS,
                                     PelUnitBufPool *pelUnitBufPool)
{
  CodingStructure &cs  = *cu.cs;
  const SPS       &sps = *cs.sps;

  const uint32_t logWidth  = floorLog2(partitioner.currArea().lwidth());
  const uint32_t logHeight = floorLog2(partitioner.currArea().lheight());

  // Lambda calculation at equivalent Qp of 4 is recommended because at that Qp, the quantization divisor is 1.
  const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda() * FRAC_BITS_SCALE;

  //===== loop over partitions =====
  const TempCtx ctxStart(m_ctxPool, m_CABACEstimator->getCtx());

  double bestCurrentCost = bestCostSoFar;

  const bool testBDPCM = CU::bdpcmAllowed(cu, CompID(partitioner.chType));

  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> hadModeList;
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM>   candCostList;
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM>   candHadList;

  bool validReturn = false;

  hadModeList.clear();
  candHadList.clear();
  candCostList.clear();

  int        numModesAvailable = NUM_LUMA_MODE; // total number of Intra modes
  const int  fastMip           = sps.m_useMIP ? m_encCfg->m_useFastMIP : 0;
  const bool mipAllowed        = sps.m_useMIP && isLuma(partitioner.chType);
  const bool testMip           = mipAllowed && !(cu.lwidth() > (8 * cu.lheight()) || cu.lheight() > (8 * cu.lwidth()));
  const bool supportedMipBlkSize = cu.lwidth() <= MIP_MAX_WIDTH && cu.lheight() <= MIP_MAX_HEIGHT;

  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> rdModeList;
  const bool                                       testSgpm = isLuma(partitioner.chType) && CU::isSgpmCoded(cu);

  int numModesForFullRD = g_intraModeNumFastUseMPM2D[logWidth - MIN_CU_LOG2][logHeight - MIN_CU_LOG2];
  if (cu.slice->m_ibcFlag && (m_encCfg->m_ibcFastMethod & IBC_FAST_METHOD_NONSCC))
  {
    numModesForFullRD = (numModesForFullRD > 1) ? (numModesForFullRD - 1) : numModesForFullRD;
  }
  SortedPelUnitBufs sortedPelUnitBufs(*pelUnitBufPool);
  int               bufferIdx = 0;
  CHECK(!cu.Y().valid(), "CU is not valid"); // this should always be true
  int numOfPassesExtendRef = ((!sps.m_useMRL) ? 1 : MRL_NUM_REF_LINES);
  cu.multiRefIdx           = 0;

  static_vector<SgpmInfo, SGPM_NUM> sgpmInfoList;
  static_vector<double, SGPM_NUM>   sgpmCostList;
  int                               sgpmNeededMode[NUM_LUMA_MODE] = { 0 };
  int                               sgpmBufferIdx[NUM_LUMA_MODE];

  if (testSgpm)
  {
    std::fill_n(sgpmBufferIdx, NUM_LUMA_MODE, -1);

    const CompArea &area = cu.Y();

    if (area.width * area.height <= 1024)
    {
      deriveTimdMode(cu.cs->picture->getRecoBuf(area), area, cu, TimdDerivationMethod::HorizontalVertical);
      deriveTimdMode(cu.cs->picture->getRecoBuf(area), area, cu, TimdDerivationMethod::FullWithSAD);
    }

    DimdData dimd;
    if (cu.slice->m_sps->m_useDIMD)
    {
      deriveDimdMode(dimd, bestCS->picture->getRecoBuf(area), area, cu);
    }

    deriveSgpmModeOrdered(bestCS->picture->getRecoBuf(area), area, cu, sgpmInfoList, sgpmCostList, dimd);
    for (int sgpmIdx = 0; sgpmIdx < SGPM_NUM; sgpmIdx++)
    {
      sgpmNeededMode[sgpmInfoList[sgpmIdx].sgpmMode0] = 1;
      sgpmNeededMode[sgpmInfoList[sgpmIdx].sgpmMode1] = 1;
    }
  }

  //===== determine set of modes to be tested (using prediction signal only) =====
  {
    PROFILER_SCOPE(1, g_timeProfiler, P_INTRA_EST_CAND_LUMA);

    if (numModesForFullRD != numModesAvailable)
    {
      CHECK(numModesForFullRD >= numModesAvailable, "Too many modes for full RD search");

      const CompArea &area = cu.Y();

      PelBuf piOrg  = cs.getOrgBuf(area);
      PelBuf piPred = cs.getPredBuf(area);

      DistParam distParamSad;
      DistParam distParamHad;

      if (cu.slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag)
      {
        CompArea tmpArea(COMP_Y, area.chromaFormat, Position(0, 0), area.size());
        PelBuf   tmpOrg = m_tmpStorageCtu.getBuf(tmpArea);
        tmpOrg.copyFrom(piOrg);
        tmpOrg.rspSignal(m_pcReshape->m_fwdLUT);
        m_pcRdCost->setDistParam(distParamSad, tmpOrg, piPred, sps.m_bitDepths[ChannelType::LUMA], COMP_Y,
                                 0);   // Use SAD cost
        m_pcRdCost->setDistParam(distParamHad, tmpOrg, piPred, sps.m_bitDepths[ChannelType::LUMA], COMP_Y,
                                 1);   // Use HAD (SATD) cost
      }
      else
      {
        m_pcRdCost->setDistParam(distParamSad, piOrg, piPred, sps.m_bitDepths[ChannelType::LUMA], COMP_Y,
                                 0);   // Use SAD cost
        m_pcRdCost->setDistParam(distParamHad, piOrg, piPred, sps.m_bitDepths[ChannelType::LUMA], COMP_Y,
                                 1);   // Use HAD (SATD) cost
      }

      distParamSad.applyWeight = false;
      distParamHad.applyWeight = false;
      const UnitArea localUnitArea(area.chromaFormat, Area(0, 0, area.width, area.height));

      if (testMip && supportedMipBlkSize)
      {
        numModesForFullRD += fastMip > 1
          ? numModesForFullRD - std::min(fastMip - 1, numModesForFullRD)
          : (fastMip ? std::max(numModesForFullRD, floorLog2(std::min(cu.lwidth(), cu.lheight())) - 1)
                     : numModesForFullRD);
      }
      sortedPelUnitBufs.prepare(localUnitArea, area.area() > 128);
      const int numHadCand = (testMip ? 2 : 1) * 3;

      cu.mipFlag = false; //*** Derive (regular) candidates using Hadamard

      //===== init pattern for luma prediction =====
      initIntraPatternChType(cu, cu.Y(), true);
      bool satdChecked[NUM_INTRA_MODE];
      std::fill_n(satdChecked, NUM_INTRA_MODE, false);

      const int startModeLoop = cu.cs->sps->m_usedirPlanar ? -2 : 0;
      for (int modeIdx = startModeLoop; modeIdx < numModesAvailable; modeIdx++)
      {
        uint32_t   mode      = modeIdx >= 0 ? static_cast<uint32_t>(modeIdx) : 0;
        Distortion minSadHad = 0;

        // Skip checking extended Angular modes in the first round of SATD
        if (mode > DC_IDX && (mode & 1))
        {
          continue;
        }

        satdChecked[mode] = true;
        const PlanarDirType plDirCurr =
          (modeIdx == -2) ? PlanarDirType::HOR : ((modeIdx == -1) ? PlanarDirType::VER : PlanarDirType::NO_DIR);
        cu.intraDir[ChannelType::LUMA] = mode;
        cu.plDir                       = plDirCurr;
        initPredIntraParams(cu, cu.Y(), sps);

        distParamHad.cur.buf = distParamSad.cur.buf = piPred.buf = sortedPelUnitBufs.getTestBuf().Y().buf;
        const bool isPDP                                         = predIntraAng(COMP_Y, piPred, cu, true, false);
        const bool usedForSGM = sgpmNeededMode[mode] && !isPDP && cu.plDir == PlanarDirType::NO_DIR;
        if (usedForSGM)
        {
          sgpmBufferIdx[mode] = bufferIdx;
        }

        // Use the min between SAD and HAD as the cost criterion
        // SAD is scaled by 2 to align with the scaling of HAD
        minSadHad += std::min(distParamSad.distFunc(distParamSad) * 2, distParamHad.distFunc(distParamHad));

        uint64_t fracModeBits = xFracModeBitsIntra(cu, mode, ChannelType::LUMA, cuCtxIntra);

        double cost = (double)minSadHad + (double)fracModeBits * sqrtLambdaForFirstPass;
        DTRACE(g_trace_ctx, D_INTRA_COST, "IntraHAD: %u, %llu, %f (%d)\n", minSadHad, fracModeBits, cost, mode);

        const ModeInfo mi(false, false, 0, mode, plDirCurr, bufferIdx++);
        int            insertPos = -1;
        updateCandList(mi, cost, rdModeList, candCostList, numModesForFullRD, &insertPos);
        updateCandList(mi, (double)minSadHad, hadModeList, candHadList, numHadCand);
        sortedPelUnitBufs.insert(insertPos, rdModeList.size(), usedForSGM);
      }

      static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> parentCandList = rdModeList;

      // Second round of SATD for extended Angular modes
      for (int modeIdx = 0; modeIdx < numModesForFullRD; modeIdx++)
      {
        unsigned parentMode = parentCandList[modeIdx].modeId;
        if (parentMode > (DC_IDX + 1) && parentMode < (NUM_LUMA_MODE - 1))
        {
          for (int subModeIdx = -1; subModeIdx <= 1; subModeIdx += 2)
          {
            unsigned mode = parentMode + subModeIdx;

            if (!satdChecked[mode])
            {
              cu.intraDir[ChannelType::LUMA] = mode;
              initPredIntraParams(cu, cu.Y(), sps);
              distParamHad.cur.buf = distParamSad.cur.buf = piPred.buf = sortedPelUnitBufs.getTestBuf().Y().buf;
              const bool isPDP                                         = predIntraAng(COMP_Y, piPred, cu, true, false);
              const bool usedForSGM                                    = sgpmNeededMode[mode] && !isPDP;
              if (usedForSGM)
              {
                sgpmBufferIdx[mode] = bufferIdx;
              }

              // Use the min between SAD and SATD as the cost criterion
              // SAD is scaled by 2 to align with the scaling of HAD
              Distortion minSadHad =
                std::min(distParamSad.distFunc(distParamSad) * 2, distParamHad.distFunc(distParamHad));

              // NB xFracModeBitsIntra will not affect the mode for chroma that may have already been
              // pre-estimated.
              uint64_t fracModeBits = xFracModeBitsIntra(cu, mode, ChannelType::LUMA, cuCtxIntra);
              double   cost         = (double)minSadHad + (double)fracModeBits * sqrtLambdaForFirstPass;

              const ModeInfo mi(false, false, 0, mode, bufferIdx++);
              int            insertPos = -1;
              updateCandList(mi, cost, rdModeList, candCostList, numModesForFullRD, &insertPos);
              updateCandList(mi, double(minSadHad), hadModeList, candHadList, numHadCand);
              sortedPelUnitBufs.insert(insertPos, rdModeList.size(), usedForSGM);
              satdChecked[mode] = true;
            }
          }
        }
      }

      if (testSgpm)
      {
        // get missing predictions if any
        cu.sgpm = true;
        for (int mode = 0; mode < NUM_LUMA_MODE; mode++)
        {
          if (sgpmNeededMode[mode] && sgpmBufferIdx[mode] == -1)
          {
            cu.intraDir[ChannelType::LUMA] = mode;
            initPredIntraParams(cu, cu.Y(), sps);
            piPred.buf = sortedPelUnitBufs.getTestBuf().Y().buf;
            predIntraAng(COMP_Y, piPred, cu, true, false);
            int insertPos = -1; // we have to store the buffer in order to
            sortedPelUnitBufs.insert(insertPos, rdModeList.size(), true);
            sgpmBufferIdx[mode] = bufferIdx++;
          }
        }
        cu.sgpm = false;
      }

      cu.multiRefIdx             = 1;
      const int      numMPMs     = NUM_PRIMARY_MOST_PROBABLE_MODES;
      const uint8_t *multiRefMPM = cuCtxIntra.mpmList;

      for (int mRefNum = 1; mRefNum < numOfPassesExtendRef; mRefNum++)
      {
        int multiRefIdx = MULTI_REF_LINE_IDX[mRefNum];
        cu.multiRefIdx  = multiRefIdx;

        initIntraPatternChType(cu, cu.Y(), true);

        for (int x = 1; x < numMPMs; x++)
        {
          uint32_t mode = multiRefMPM[x];
          {
            cu.intraDir[ChannelType::LUMA] = mode;
            distParamHad.cur.buf = distParamSad.cur.buf = piPred.buf = sortedPelUnitBufs.getTestBuf().Y().buf;
            initPredIntraParams(cu, cu.Y(), sps);
            predIntraAng(COMP_Y, piPred, cu, true, false);

            // Use the min between SAD and SATD as the cost criterion
            // SAD is scaled by 2 to align with the scaling of HAD
            Distortion minSadHad =
              std::min(distParamSad.distFunc(distParamSad) * 2, distParamHad.distFunc(distParamHad));
            uint64_t fracModeBits = xFracModeBitsIntra(cu, mode, ChannelType::LUMA, cuCtxIntra);

            double         cost = (double)minSadHad + (double)fracModeBits * sqrtLambdaForFirstPass;
            const ModeInfo mi(false, false, multiRefIdx, mode, bufferIdx++);
            int            insertPos = -1;
            updateCandList(mi, cost, rdModeList, candCostList, numModesForFullRD, &insertPos);
            updateCandList(mi, (double)minSadHad, hadModeList, candHadList, numHadCand);
            sortedPelUnitBufs.insert(insertPos, rdModeList.size());
          }
        }
      }

      CHECKD(rdModeList.size() != numModesForFullRD, "Error: RD mode list size");

      //*** Derive MIP candidates using Hadamard
      if (testMip && !supportedMipBlkSize)
      {
        // avoid estimation for unsupported blk sizes
        const int transpOff    = MatrixIntraPrediction::getNumModesMip(cu.Y());
        const int numModesFull = (transpOff << 1);
        for (uint32_t modeFull = 0; modeFull < numModesFull; modeFull++)
        {
          const bool     isTransposed = modeFull >= transpOff;
          const uint32_t mode         = (isTransposed ? modeFull - transpOff : modeFull);

          numModesForFullRD++;
          ModeInfo mi(true, isTransposed, 0, mode, -1);
          updateCandList(mi, 0, rdModeList, candCostList, numModesForFullRD);
        }
      }
      else if (testMip)
      {
        cu.mipFlag     = true;
        cu.multiRefIdx = 0;

        double mipHadCost[MAX_NUM_MIP_MODE];
        std::fill_n(mipHadCost, MAX_NUM_MIP_MODE, MAX_DOUBLE);

        initIntraPatternChType(cu, cu.Y());
        cu.bvgMipFlag = PU::bvgMipAvailable(cu);
        initIntraMip(cu, cu.Y());

        const int transpOff    = MatrixIntraPrediction::getNumModesMip(cu.Y());
        const int numModesFull = (transpOff << 1);
        for (uint32_t modeFull = 0; modeFull < numModesFull; modeFull++)
        {
          const bool     isTransposed = modeFull >= transpOff;
          const uint32_t mode         = isTransposed ? modeFull - transpOff : modeFull;

          cu.mipTransposedFlag           = isTransposed;
          cu.intraDir[ChannelType::LUMA] = mode;
          distParamHad.cur.buf = distParamSad.cur.buf = piPred.buf = sortedPelUnitBufs.getTestBuf().Y().buf;
          predIntraMip(COMP_Y, piPred, cu);

          // Use the min between SAD and HAD as the cost criterion
          // SAD is scaled by 2 to align with the scaling of HAD
          Distortion minSadHad =
            std::min(distParamSad.distFunc(distParamSad) * 2, distParamHad.distFunc(distParamHad));
          uint64_t fracModeBits = xFracModeBitsIntra(cu, mode, ChannelType::LUMA, cuCtxIntra);

          double cost          = (double)minSadHad + (double)fracModeBits * sqrtLambdaForFirstPass;
          mipHadCost[modeFull] = cost;
          DTRACE(g_trace_ctx, D_INTRA_COST, "%s: %u, %llu, %f (%d)\n",
                 cu.bvgMipFlag ? "IntraBvgMIP" : "IntraMIP", minSadHad, fracModeBits, cost, modeFull);

          const ModeInfo mi(true, isTransposed, 0, mode, bufferIdx++);
          int            insertPos = -1;
          updateCandList(mi, cost, rdModeList, candCostList, numModesForFullRD + 1, &insertPos);
          updateCandList(mi, 0.8 * (double)minSadHad, hadModeList, candHadList, numHadCand);
          sortedPelUnitBufs.insert(insertPos, rdModeList.size());
        }

        const double thresholdHadCost = 1.0 + 1.4 / sqrt((double)(cu.lwidth() * cu.lheight()));
        xReduceHadCandList(rdModeList, candCostList, sortedPelUnitBufs, numModesForFullRD, thresholdHadCost, mipHadCost,
                           cu, fastMip);
        cu.mipFlag    = false;
        cu.bvgMipFlag = false;
      }

      if (testSgpm)
      {
        static_vector<ModeInfo, SGPM_NUM> hadModeSGPMList;
        static_vector<double, SGPM_NUM>   candCostSGPMList;
        static_vector<double, SGPM_NUM>   candHadSGPMList;
        static_vector<ModeInfo, SGPM_NUM> rdModeSGPMList;

        int numSGPMCand = (numModesForFullRD + 1) / 2;

        cu.sgpm                        = true;
        // frac bits calculate once because all are the same
        cu.sgpmIdx                     = 0;
        cu.sgpmSplitDir                = sgpmInfoList[0].sgpmSplitDir;
        cu.sgpmMode0                   = sgpmInfoList[0].sgpmMode0;
        cu.sgpmMode1                   = sgpmInfoList[0].sgpmMode1;
        cu.intraDir[ChannelType::LUMA] = cu.sgpmMode0;

        uint64_t fracModeBits = xFracModeBitsIntra(cu, 0, ChannelType::LUMA, cuCtxIntra);
        for (int sgpmIdx = 0; sgpmIdx < SGPM_NUM; sgpmIdx++)
        {
          distParamHad.cur.buf = distParamSad.cur.buf = piPred.buf = sortedPelUnitBufs.getTestBuf().Y().buf;
          m_pIf->m_weightedSgpm(
            cu, cu.lwidth(), cu.lheight(), COMP_Y, sgpmInfoList[sgpmIdx].sgpmSplitDir, piPred,
            sortedPelUnitBufs.getBufFromSortedList(sgpmBufferIdx[sgpmInfoList[sgpmIdx].sgpmMode0])->Y(),
            sortedPelUnitBufs.getBufFromSortedList(sgpmBufferIdx[sgpmInfoList[sgpmIdx].sgpmMode1])->Y());

          Distortion minSadHad = 0;
          minSadHad += std::min(distParamSad.distFunc(distParamSad) * 2, distParamHad.distFunc(distParamHad));
          double cost = (double)minSadHad + (double)fracModeBits * sqrtLambdaForFirstPass;

          const ModeInfo mi(true, sgpmIdx, sgpmInfoList[sgpmIdx], bufferIdx++);
          int            insertPos = -1;
          updateCandList(mi, cost, rdModeSGPMList, candCostSGPMList, numSGPMCand, &insertPos);
          updateCandList(mi, (double)minSadHad, hadModeSGPMList, candHadSGPMList, numSGPMCand);
          sortedPelUnitBufs.insert(insertPos, rdModeList.size());
        }

        if (candCostSGPMList[0] < candCostList[numModesForFullRD - 1])
        {
          for (auto listIdx = 0; listIdx < numSGPMCand; listIdx++)
          {
            updateCandList(rdModeSGPMList[listIdx], candCostSGPMList[listIdx], rdModeList, candCostList,
                           numModesForFullRD);
            updateCandList(hadModeSGPMList[listIdx], candHadSGPMList[listIdx], hadModeList, candHadList, numHadCand);
          }
        }

        cu.sgpm = false;
      }
      if (isLuma(partitioner.chType) && cs.sps->m_useEIP &&
          (CU::eipAllowed(cu, COMP_Y) || CU::eipMergeAllowed(cu, COMP_Y)))
      {
        static_vector<ModeInfo, NUM_DERIVED_EIP + MAX_MERGE_EIP> hadModeEIPList;
        static_vector<double, NUM_DERIVED_EIP + MAX_MERGE_EIP>   candCostEIPList;
        static_vector<double, NUM_DERIVED_EIP + MAX_MERGE_EIP>   candHadEIPList;
        static_vector<ModeInfo, NUM_DERIVED_EIP + MAX_MERGE_EIP> rdModeEIPList;

        cu.mipFlag  = false;
        cu.eipFlag  = true;
        cu.eipMerge = false;
        static_vector<EipModels, NUM_DERIVED_EIP> eipModelCandList;
        static_vector<EipModels, MAX_MERGE_EIP>   eipMergeCandList;

        initIntraEip(cu, area);
        const int numNonMM = getCurEipCands(cu, eipModelCandList, COMP_Y);
        getNeighborsEipCands(cu, eipMergeCandList, COMP_Y);
        reorderEipCands(cu, eipMergeCandList);

        const int maxNumRdEIP = std::max(NUM_EIP_MERGE_SIGNAL + NUM_DERIVED_EIP, (numModesForFullRD + 1) / 2);
        for (int mergeFlag = 0; mergeFlag < 2; mergeFlag++)
        {
          cu.eipMerge = bool(mergeFlag);
          for (int i = 0; i < (cu.eipMerge ? eipMergeCandList.size() : eipModelCandList.size()); i++)
          {
            distParamHad.cur.buf = distParamSad.cur.buf = piPred.buf = sortedPelUnitBufs.getTestBuf().Y().buf;

            cu.eipModels     = cu.eipMerge ? eipMergeCandList[i] : eipModelCandList[i];
            cu.eipMultiModel = cu.eipModels.isMultiModel();

            eipPred(cu, piPred, area.compID);

            // intraMode is used as the index in either the merge candidate list, the currently allowed list
            // or the currently allowed multi-model list (when MMEIP is allowed)
            int        intraMode    = (cu.eipMerge == false && cu.eipMultiModel) ? i - numNonMM : i;
            uint64_t   fracModeBits = xFracModeBitsIntra(cu, intraMode, ChannelType::LUMA, cuCtxIntra);
            Distortion minSadHad =
              std::min(distParamSad.distFunc(distParamSad) * 2, distParamHad.distFunc(distParamHad));
            double cost = double(minSadHad) + double(fracModeBits) * sqrtLambdaForFirstPass;

            ModeInfo mi;
            mi.eipFlg      = true;
            mi.eipMergeFlg = cu.eipMerge;
            mi.modeId      = EIP_IDX + i;
            mi.bufferIdx   = bufferIdx++;
            int  insertPos = -1;
            bool isCand    = updateCandList(mi, cost, rdModeEIPList, candCostEIPList, maxNumRdEIP, &insertPos);
            updateCandList(mi, double(minSadHad * 0.8), hadModeEIPList, candHadEIPList, maxNumRdEIP);
            if (isCand)
            {
              sortedPelUnitBufs.insert(insertPos, rdModeList.size(), true);
            }
          }
        }

        int    numRdEip    = 0;
        int    lastRdEip   = -1;
        double bestEipCost = candCostEIPList.size() > 0 ? candCostEIPList[0] : 0;
        for (auto listIdx = 0; listIdx < candCostEIPList.size(); listIdx++)
        {
          int  candPos = -1;
          bool isCand  = updateCandList(rdModeEIPList[listIdx], candCostEIPList[listIdx], rdModeList, candCostList,
                                        numModesForFullRD, &candPos);
          updateCandList(hadModeEIPList[listIdx], candHadEIPList[listIdx], hadModeList, candHadList, numHadCand);

          if (isCand == true)
          {
            ModeInfo &mi       = rdModeEIPList[listIdx];
            int       modelIdx = mi.modeId - EIP_IDX;
            CPelBuf   piPred   = sortedPelUnitBufs.getBufFromSortedList(mi.bufferIdx)->Y();

            lastRdEip = candPos;
            numRdEip++;
            const auto derivedIPrdModes          = IntraPrediction::deriveIpmForTransform(piPred, cu);
            rdModeList[candPos].inferredDimdMode = derivedIPrdModes.first;
            if (mi.eipMergeFlg)
            {
              rdModeList[candPos].eipModels = eipMergeCandList[modelIdx];
            }
            else
            {
              rdModeList[candPos].eipModels = eipModelCandList[modelIdx];
              // modeId will be used later as the index in the candidate list
              // when in mm mode the mm candidate list is not appended to the non-mm candidate list on the decoder side
              // we need to adjust the index in this case
              rdModeList[candPos].modeId -= (rdModeList[candPos].modeId - EIP_IDX) >= numNonMM ? numNonMM : 0;
            }
          }
          else
          {
            // candCostEIPList is sorted, as soon as an eip candidate does not get into rdModeList we can break this
            // loop since all other are worse
            break;
          }
        }

        CHECK((numModesForFullRD < 1), "not enough rd candidates");
        int  numNonEip     = numModesForFullRD - numRdEip;
        bool lastModeIsEip = lastRdEip == (numModesForFullRD - 1);
        bool reduceRD      = candCostEIPList.size() ? (cu.Y().area() < 256) &&
            (bestEipCost < candCostList[numModesForFullRD - 1]) && (lastModeIsEip || (numNonEip > 1))
                                                    : false;
        if (reduceRD && cu.cs->slice->isIntra())
        {
          rdModeList.pop_back();
          candCostList.pop_back();
          numModesForFullRD = int(rdModeList.size());
        }

        cu.eipFlag = false;
      }
      if ((!partitioner.canSplit(TU_MAX_TR_SPLIT,
                                 cs))   // For Splitting of CU into multiple TUs, DIMD would yield different intra modes
                                        // for MPM generation of next blocks, unclear how to handle this
          && cs.sps->m_useDIMD)
      {
        cu.mipFlag     = false;
        cu.multiRefIdx = 0;
        cu.dimdFlag    = true;
        piPred.buf     = sortedPelUnitBufs.getTestBuf().Y().buf;
        predIntraDimd(piPred, cu, cu.blocks[COMP_Y]);
        numModesForFullRD++;
        ModeInfo miDIMD(PlanarDirType::NO_DIR, -1);
        miDIMD.dimdFlg = true;
        miDIMD.modeId  = cu.intraDir[ChannelType::LUMA];   // intraMode (possibly used for MPM-generation of subsequent
                                                          // blocks) is computed inside predIntraDimd
        miDIMD.derivedIpm[0] = cu.derivedIpm[0];
        miDIMD.derivedIpm[1] = cu.derivedIpm[1];

        if (cs.sps->m_useOBIC && cu.Y().area() > 32)
        {
          PU::getObicNeighbours(cu, cu.obicNeighbours);
          cu.obicAvailFlag    = PU::isObicAvail(cu);
          miDIMD.obicAvailFlg = cu.obicAvailFlag;   // DIMD
          if (cu.obicAvailFlag)
          {
            cu.mipFlag     = false;
            cu.multiRefIdx = 0;
            piPred.buf     = sortedPelUnitBufs.getTestBuf().Y().buf;
            predIntraObic(piPred, cu, cu.blocks[COMP_Y], false);
            numModesForFullRD++;
            ModeInfo miOBIC(PlanarDirType::NO_DIR, -1);
            miOBIC.dimdFlg       = true;
            miOBIC.obicFlg       = true;
            miOBIC.obicAvailFlg  = true;
            miOBIC.modeId        = cu.intraDir[ChannelType::LUMA];
            miOBIC.derivedIpm[0] = cu.derivedIpm[0];
            miOBIC.derivedIpm[1] = cu.derivedIpm[1];
            miOBIC.bufferIdx     = bufferIdx++;
            updateCandList(miOBIC, 0, rdModeList, candCostList, numModesForFullRD);
            sortedPelUnitBufs.insert(-1, rdModeList.size(), true);
            cu.obicFlag = false;   // Not needed, just to be sure
          }
        }

        updateCandList(miDIMD, 0, rdModeList, candCostList, numModesForFullRD);
        cu.dimdFlag = false;   // Not needed, just to be sure
      }
      if ((!partitioner.canSplit(TU_MAX_TR_SPLIT, cs)) && cs.sps->m_useTIMD)
      {
        cu.mipFlag     = false;
        cu.multiRefIdx = 0;
        cu.timdFlag    = true;
        piPred.buf     = sortedPelUnitBufs.getTestBuf().Y().buf;

        for (const auto timdMode: { IntraPrediction::TimdMode::Normal, IntraPrediction::TimdMode::SAD })
        {
          if (timdMode == IntraPrediction::TimdMode::SAD && (!CU::allowTimdSad(cu) || !cs.sps->m_useTIMDSAD))
          {
            continue;
          }

          const auto alreadyExecutedForNormalMode = (timdMode == IntraPrediction::TimdMode::SAD);

          predIntraTimd(piPred, cu, cu.blocks[COMP_Y], false, timdMode, alreadyExecutedForNormalMode);
          numModesForFullRD++;
          ModeInfo miTIMD(PlanarDirType::NO_DIR, -1);
          miTIMD.timdFlg       = true;
          miTIMD.timdSadFlg    = (timdMode == IntraPrediction::TimdMode::SAD);
          miTIMD.modeId        = cu.intraDir[ChannelType::LUMA];
          miTIMD.derivedIpm[0] = cu.derivedIpm[0];
          miTIMD.derivedIpm[1] = cu.derivedIpm[1];
          updateCandList(miTIMD, 0, rdModeList, candCostList, numModesForFullRD);
        }

        cu.timdFlag    = false;   // Not needed, just to be sure
        cu.timdSadFlag = false;
      }
      if (m_encCfg->m_bFastUDIUseMPMEnabled)
      {
        int numCand    = cuCtxIntra.mpmListSize;
        numCand        = (numCand > 2) ? 2 : numCand;
        cu.multiRefIdx = 0;

        for (int j = 0; j < numCand; j++)
        {
          bool     mostProbableModeIncluded = false;
          ModeInfo mostProbableMode(false, false, 0, cuCtxIntra.mpmList[j], -1);

          for (int i = 0; i < numModesForFullRD; i++)
          {
            mostProbableModeIncluded |= (mostProbableMode == rdModeList[i]);
          }
          if (!mostProbableModeIncluded)
          {
            numModesForFullRD++;
            updateCandList(mostProbableMode, 0, rdModeList, candCostList, numModesForFullRD);
          }
        }
      }
    }
    else
    {
      THROW("Full search not supported for MIP");
    }

    CHECK(numModesForFullRD != rdModeList.size(), "Inconsistent state!");

    // after this point, don't use numModesForFullRD
    // PBINTRA fast
    if (m_encCfg->m_usePbIntraFast && !cs.slice->isIntra() && rdModeList.size() < numModesAvailable &&
        !cs.slice->m_disableSATDForRd)
    {
      double   pbintraRatio = cs.sps->m_useIntraLFNSTinPBSlice ? 1.25 : PBINTRA_RATIO;
      int      maxSize      = -1;
      int      bestMipIdx   = -1;
      ModeInfo bestMipMode;

      for (int idx = 0; idx < rdModeList.size(); idx++)
      {
        if (rdModeList[idx].mipFlg)
        {
          bestMipMode = rdModeList[idx];
          bestMipIdx  = idx;
          break;
        }
      }
      const int numHadCand = 3;
      for (int k = numHadCand - 1; k >= 0; k--)
      {
        if (candHadList.size() < (k + 1) || candHadList[k] > cs.interHad * pbintraRatio)
        {
          maxSize = k;
        }
      }
      if (maxSize > 0)
      {
        rdModeList.resize(std::min<size_t>(rdModeList.size(), maxSize));
        if (bestMipIdx >= 0)
        {
          if (rdModeList.size() <= bestMipIdx)
          {
            rdModeList.push_back(bestMipMode);
          }
        }
      }
      if (0 == maxSize)
      {
        cs.dist     = std::numeric_limits<Distortion>::max();
        cs.interHad = 0;
        return false;
      }
    }
  }
  {
    //===== check modes (using r-d costs) =====
    ModeInfo bestPuMode;

    CodingStructure *csTemp = m_pTempCS;
    CodingStructure *csBest = m_pBestCS;

    csTemp->slice = cs.slice;
    csBest->slice = cs.slice;
    csTemp->compactResize(cs.area);
    csBest->compactResize(cs.area);
    csTemp->initStructData();
    csBest->initStructData();
    csTemp->picture = cs.picture;
    csBest->picture = cs.picture;

    numModesForFullRD = (int)rdModeList.size();

    if (testBDPCM)
    {
      rdModeList.push_back(ModeInfo(BdpcmMode::HOR));
      rdModeList.push_back(ModeInfo(BdpcmMode::VER));
    }

    xSelectMTCandLuma(cs, partitioner, m_modeTrCandListLuma, rdModeList, cuCtxIntra, &sortedPelUnitBufs);

    PROFILER_SCOPE(1, g_timeProfiler, P_INTRA_RD_CHECK_LUMA);
    for (auto &cand: m_modeTrCandListLuma)
    {
      if (cand.trTypes.empty() && cand.valid)
      {
        continue;
      }
      setCuPredDataLuma(cu, cand);
      CHECK(cu.mipFlag && cu.multiRefIdx, "Error: combination of MIP and MRL not supported");
      CHECK(cu.multiRefIdx && (cu.intraDir[ChannelType::LUMA] == PLANAR_IDX),
            "Error: combination of MRL and Planar mode not supported");

      // set context models
      m_CABACEstimator->getCtx() = ctxStart;

      // determine residual for partition
      cs.initSubStructure(*csTemp, partitioner.chType, cs.area, true);

      bool tmpValidReturn = xRecurIntraCodingLumaQT(*csTemp, partitioner, cand, cuCtxIntra);
      validReturn |= tmpValidReturn;

      DTRACE(g_trace_ctx, D_INTRA_COST, "IntraCost T [x=%d,y=%d,w=%d,h=%d] %f (%d,%d,%d,%d) \n", cu.blocks[0].x,
             cu.blocks[0].y, (int)partitioner.currArea().lwidth(), (int)partitioner.currArea().lheight(), csTemp->cost,
             cand.modeId, cu.multiRefIdx, cu.mipFlag, cu.bvgMipFlag);

      if (tmpValidReturn)
      {
        if (csTemp->cost < csBest->cost)   // check r-d cost
        {
          std::swap(csTemp, csBest);
          bestPuMode = cand;
          if (csBest->cost < bestCurrentCost)
          {
            bestCurrentCost = csBest->cost;
          }
        }
      }

      csTemp->releaseIntermediateData();
    }   // Mode loop

    if (validReturn)
    {
      cs.useSubStructure(*csBest, partitioner.chType, cu.singleChan(ChannelType::LUMA), true, true,
                         KEEP_PRED_AND_RESI_SIGNALS, KEEP_PRED_AND_RESI_SIGNALS, true);
    }

    csBest->releaseIntermediateData();

    if (validReturn)
    {
      //=== update PU data ====
      setCuPredDataLuma(cu, bestPuMode);
      PU::setLumaIntraModeFlags(cu, cuCtxIntra);   // Flags needed when calling Decoder in Encoder
    }
  }

  //===== reset context models =====
  m_CABACEstimator->getCtx() = ctxStart;

  return validReturn;
}

void IntraSearch::setCuPredDataLuma(CodingUnit &cu, const ModeInfo &mi)
{
  CHECK(mi.plIdx != PlanarDirType::NO_DIR && mi.modeId != 0, "Error, directional index only for planar");
  cu.mipFlag                     = mi.mipFlg;
  cu.mipTransposedFlag           = mi.mipTrFlg;
  cu.bvgMipFlag                  = cu.mipFlag && PU::bvgMipAvailable(cu);
  cu.multiRefIdx                 = mi.mRefId;
  cu.intraDir[ChannelType::LUMA] = mi.modeId;
  cu.bdpcmMode[0]                = mi.bdpcm;
  cu.plDir                       = mi.plIdx;
  cu.derivedIpm[0]               = mi.derivedIpm[0];
  cu.derivedIpm[1]               = mi.derivedIpm[1];
  cu.dimdFlag                    = mi.dimdFlg;
  cu.timdFlag                    = mi.timdFlg;
  cu.timdSadFlag                 = mi.timdSadFlg;
  cu.obicFlag                    = mi.obicFlg;
  cu.obicAvailFlag               = mi.obicAvailFlg;
  cu.eipFlag                     = mi.eipFlg;
  cu.sgpm                        = mi.sgpmFlg;
  cu.sgpmIdx                     = mi.modeId;
  cu.sgpmSplitDir                = mi.sgpmInfo.sgpmSplitDir;
  cu.sgpmMode0                   = mi.sgpmInfo.sgpmMode0;
  cu.sgpmMode1                   = mi.sgpmInfo.sgpmMode1;

  if (cu.sgpm)
  {
    cu.intraDir[ChannelType::LUMA] = mi.sgpmInfo.sgpmMode0;
  }
  if (cu.eipFlag)
  {
    cu.eipMerge                    = mi.eipMergeFlg;
    cu.eipModels                   = mi.eipModels;
    cu.eipMultiModel               = mi.eipModels.isMultiModel();
    cu.inferredDimdMode            = mi.inferredDimdMode;
    cu.intraDir[ChannelType::LUMA] = mi.modeId - EIP_IDX;
  }

  CHECK(mi.dimdFlg && (mi.mipFlg || (mi.mRefId != 0)), "Error, combination with dimd not supported");
  CHECK(mi.timdFlg && (mi.dimdFlg || mi.mipFlg || (mi.mRefId != 0)), "Error, combination with timd not supported");
  CHECK(mi.timdSadFlg && !mi.timdFlg, "In case of Sad mode, both flags must be set");
}

void IntraSearch::setCuPredDataChroma(CodingUnit &cu, const ChromaModeInfo &miCh, const uint32_t *chromaCandModes)
{
  const int mi         = miCh.modeId;
  cu.cccmFlag          = miCh.cccmFlag;
  cu.cccmType          = miCh.cccmType;
  cu.cclmOffsets       = miCh.cclmOffsets;
  cu.dimdChromaFlag    = miCh.dimdFlag;
  cu.ccModels          = miCh.ccModels;
  cu.idxNonLocalCCP    = miCh.ccMergeInd;
  cu.ccMergeFusionIdx  = miCh.ccFusionInd;
  cu.ccFilterFlag      = miCh.ccFilterFlag;
  cu.decDerivedCcpMode = miCh.decDerivedCcpMode;

  if (mi >= BDPCM_IDX)
  {
    cu.bdpcmMode[1]                  = BdpcmMode(mi - BDPCM_IDX);
    cu.intraDir[ChannelType::CHROMA] = cu.bdpcmMode[1] == BdpcmMode::VER ? chromaCandModes[1] : chromaCandModes[2];
  }
  else
  {
    cu.bdpcmMode[1]                  = BdpcmMode::NONE;
    cu.intraDir[ChannelType::CHROMA] = mi;
  }
}

void IntraSearch::xPreCalcPrdTransLuma(TransformUnit &tu, PreTrListLuma &tl, const PelBuf *prdBuf)
{
  const CompArea &area     = tu.blocks[COMP_Y];
  const bool      lossless = tu.cs->slice->m_isLossless && m_encCfg->m_costMode == COST_LOSSLESS_CODING;

  //--- intra prediction ---
  PelBuf prd(tl.prd, area);
  if (prdBuf)
  {
    prd.copyFrom(*prdBuf);
  }
  else
  {
    xPredTuLuma(tu, prd);
  }

  //--- calculate residual ---
  PelBuf res(tl.res, area);
  res.copyFrom(tu.cs->getOrgBuf(area));
  if (tu.cs->slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag)
  {
    res.rspSignal(m_pcReshape->m_fwdLUT);
  }
  res.subtract(prd);

  //--- create transform list ---
  tl.trTypes = TU::getTransCandIntra(tu, COMP_Y, lossless, m_encCfg->m_useChromaTS);

  //--- derive dimd directions for transform ---
  if (includesNST(tl.trTypes) || (tu.cs->sps->m_mtsEnabled && !tu.cs->sps->m_explicitMtsIntra))
  {
    if (PU::isMIP(*tu.cu, ChannelType::LUMA) || PU::isSgpm(*tu.cu, ChannelType::LUMA))
    {
      tu.derivedIntraDirsLuma = tl.derivedIntraDirs = IntraPrediction::deriveIpmForTransform(prd, *tu.cu);
    }
  }

  //--- do all transforms and set valid ---
  m_pcTrQuant->preCalcTrans(tu, COMP_Y, res, tl.trTypes, tl.trCoeffs);
  tl.valid = true;
}

void IntraSearch::xPreCalcPrdTransChroma(TransformUnit &tu, IModeTrCandChroma &tl)
{
  const CompArea &areaCb   = tu.blocks[COMP_Cb];
  const CompArea &areaCr   = tu.blocks[COMP_Cr];
  const bool      lossless = tu.cs->slice->m_isLossless && m_encCfg->m_costMode == COST_LOSSLESS_CODING;
  const bool      reshape  = (tu.cs->slice->m_lmcsEnabledFlag && tu.cs->picHeader->m_lmcsChromaResidualScaleFlag &&
                        (tu.cs->slice->isIntra() || m_pcReshape->m_ctuFlag) && (areaCb.area() > 4));

  //--- intra prediction ---
  PelBuf prdCb(tl.prd[0], areaCb);
  PelBuf prdCr(tl.prd[1], areaCr);
  xPredTuChroma(tu, prdCb, prdCr, tl);

  //--- calculate residual ---
  PelBuf resCb(tl.res[0], areaCb);
  PelBuf resCr(tl.res[1], areaCr);
  resCb.copyFrom(tu.cs->getOrgBuf(areaCb));
  resCr.copyFrom(tu.cs->getOrgBuf(areaCr));
  resCb.subtract(prdCb);
  resCr.subtract(prdCr);
  tl.chromaResScale = 0;
  if (reshape)
  {
    const Area      area  = tu.cu->Y().valid()
            ? tu.cu->Y()
            : Area(recalcPosition(tu.chromaFormat, tu.chType, ChannelType::LUMA, tu.cu->block(tu.chType).pos()),
                   recalcSize(tu.chromaFormat, tu.chType, ChannelType::LUMA, tu.cu->block(tu.chType).size()));
    const CompArea &areaY = CompArea(COMP_Y, tu.chromaFormat, area);
    tl.chromaResScale     = m_pcReshape->calculateChromaAdjVpduNei(tu, areaY);
    resCb.scaleSignal(tl.chromaResScale, 1, tu.cs->slice->clpRng(COMP_Cb));
    resCr.scaleSignal(tl.chromaResScale, 1, tu.cs->slice->clpRng(COMP_Cr));
  }
  tu.setChromaAdj(tl.chromaResScale);

  //--- create transform lists ---
  const bool skipTS = isNST(tu.mtsIdx[COMP_Y]) && tu.cu->bdpcmMode[1] == BdpcmMode::NONE;
  TransList  trList = TU::getTransCandIntra(tu, COMP_Cb, lossless, m_encCfg->m_useChromaTS);
  tl.trTypesSep.clear();
  tl.trTypesJnt.clear();
  for (const auto t: trList)
  {
    if (t == MtsType::SKIP && skipTS)
    {
      continue;
    }
    if (isNST(t))
    {
      tl.trTypesJnt.push_back(t);
    }
    else
    {
      tl.trTypesSep.push_back(t);
    }
  }
  tl.trTypesCmb = tl.trTypesSep;
  tl.trTypesCmb.insert(tl.trTypesCmb.end(), tl.trTypesJnt.cbegin(), tl.trTypesJnt.cend());

  //--- derive intra mode for transform ---
  if (PU::isLMCMode(tl.chromaMode.modeId) && includesNST(tl.trTypesJnt))
  {
    tu.derivedIntraDirChroma = IntraPrediction::deriveIpmForChromaTransform(prdCb, prdCr, *tu.cu);
    tl.derivedIntraDirs[0]   = tu.derivedIntraDirChroma;
    tl.derivedIntraDirs[3]   = tu.derivedIntraDirChroma;
  }

  //--- do all transforms and set valid ---
  m_pcTrQuant->preCalcTrans(tu, COMP_Cb, resCb, tl.trTypesCmb, tl.trCoeffs[0]);
  m_pcTrQuant->preCalcTrans(tu, COMP_Cr, resCr, tl.trTypesCmb, tl.trCoeffs[1]);
  tl.valid = true;
}

void IntraSearch::xPreCalcPrdTransJCCR(TransformUnit &tu, IModeTrCandChroma &tl)
{
  //--- check whether to check JCCR at all ---
  tl.cbfMaskJCCR.clear();
  const bool cbfCb = TU::getCbf(tu, COMP_Cb);
  const bool cbfCr = TU::getCbf(tu, COMP_Cr);
  if (!cbfCb && !cbfCr)
  {
    return;
  }

  //--- calculate residuals and determine JCCR types to be tested ---
  const CompArea &area       = tu.blocks[COMP_Cb];
  const PelBuf    resCb      = PelBuf(tl.res[0], area);
  const PelBuf    resCr      = PelBuf(tl.res[1], area);
  PelBuf          resJ1      = PelBuf(tl.res[2], area);
  PelBuf          resJ2      = PelBuf(tl.res[3], area);
  PelBuf          resJ3      = PelBuf(tl.res[4], area);
  const PelBuf   *resCbCr[2] = { &resCb, &resCr };
  PelBuf         *resJCCR[3] = { &resJ1, &resJ2, &resJ3 };
  tl.cbfMaskJCCR             = m_pcTrQuant->selectICTCandidates(tu, resCbCr, resJCCR);
  if (tl.cbfMaskJCCR.empty())
  {
    return;
  }

  //--- determine transforms to be tested (based on non-JCCR data) ---
  const bool      trCb    = cbfCb && tu.mtsIdx[COMP_Cb] != MtsType::SKIP;
  const bool      trCr    = cbfCr && tu.mtsIdx[COMP_Cr] != MtsType::SKIP;
  const bool      tsCb    = cbfCb && tu.mtsIdx[COMP_Cb] == MtsType::SKIP;
  const bool      tsCr    = cbfCr && tu.mtsIdx[COMP_Cr] == MtsType::SKIP;
  const bool      trOnly  = (trCb && !cbfCr) || (trCr && !cbfCb) || (trCb && trCr);
  const bool      tsOnly  = (tsCb && !cbfCr) || (tsCr && !cbfCb) || (tsCb && tsCr);
  const TransList trTypes = tl.trTypesCmb;   // re-use non-JCCR transform list as start
  tl.trTypesJCCR.clear();
  for (const auto &tr: trTypes)
  {
    if ((tr == MtsType::SKIP && trOnly) || (tr != MtsType::SKIP && tsOnly))
    {
      continue;
    }
    tl.trTypesJCCR.push_back(tr);
  }
  CHECK(tl.trTypesJCCR.empty(), "Empty transform list for JCCR");

  //--- derive transform direction ---
  if (PU::isLMCMode(tl.chromaMode.modeId) && includesNST(tl.trTypesJCCR))
  {
    PelBuf prdCb(tl.prd[0], tu.blocks[COMP_Cb]);
    PelBuf prdCr(tl.prd[1], tu.blocks[COMP_Cr]);
    for (const auto &cbfMask: tl.cbfMaskJCCR)
    {
      if (cbfMask == 1 || cbfMask == 2)
      {
        tu.jointCbCr                 = static_cast<uint8_t>(cbfMask);
        tl.derivedIntraDirs[cbfMask] = IntraPrediction::deriveIpmForChromaTransform(prdCb, prdCr, *tu.cu);
      }
    }
  }

  //--- do all transforms ---
  for (const auto &cbfMask: tl.cbfMaskJCCR)
  {
    //--- calculate transforms ---
    tu.jointCbCr = static_cast<uint8_t>(cbfMask);
    ;
    tu.derivedIntraDirChroma = tl.derivedIntraDirs[tu.jointCbCr];
    m_pcTrQuant->preCalcTrans(tu, COMP_Cb, *resJCCR[cbfMask - 1], tl.trTypesJCCR, tl.trCoeffs[1 + cbfMask]);
  }
}

void IntraSearch::xSelectMTCandLuma(CodingStructure &cs, Partitioner &pt, IModeTrCandListLuma &modeTrCandList,
                                    const ModeInfoList &imodeList, const CUCtxIntra &cuCtxIntra,
                                    const SortedPelUnitBufs *sortedBufs)
{
  PROFILER_SCOPE(1, g_timeProfiler, P_INTRA_EST_TRAFO);
  CHECK(imodeList.size() == 0, "empty intra mode list");
  CHECK(imodeList.size() > modeTrCandList.capacity(), "intra mode list exceeds buffer capacity");

  //===== initialize list with mode info only =====
  modeTrCandList.clear();
  for (const auto &m: imodeList)
  {
    modeTrCandList.add(m);
  }
  if (pt.canSplit(TU_MAX_TR_SPLIT, cs))
  {
    return;
  }

  //===== create TU =====
  TransformUnit &tu = cs.addTU(CS::getArea(cs, pt.currArea(), pt.chType), pt.chType);
  tu.depth          = pt.currTrDepth;

  //===== calculate all predictions and transforms =====
  const int numPred = (int)modeTrCandList.size();
  for (int pid = 0; pid < numPred; pid++)
  {
    //--- add intra mode entry ---
    auto             &modeTrCand = modeTrCandList[pid];
    const bool        hasBuf     = sortedBufs && modeTrCand.bdpcm == BdpcmMode::NONE;
    const PelUnitBuf *unitBuf    = hasBuf ? sortedBufs->getBufFromSortedList(modeTrCand.bufferIdx) : nullptr;
    const PelBuf     *prdBuf     = unitBuf ? &unitBuf->Y() : nullptr;
    setCuPredDataLuma(*tu.cu, modeTrCand);
    xPreCalcPrdTransLuma(tu, modeTrCand, prdBuf);
  }

  //===== remove unlikely pred/trans combinations
  xSelectPrTrCandLuma(tu, modeTrCandList, cuCtxIntra);

  //===== clean-up =====
  cs.clearTUs();
}

void IntraSearch::xSelectMTCandChroma(CodingStructure &cs, Partitioner &pt, IModeTrCandListChroma &modeTrCandList,
                                      const std::vector<ChromaModeInfo> &imodeList, const uint32_t *chromaCandModes)
{
  CHECK(imodeList.size() == 0, "empty intra mode list");
  CHECK(imodeList.size() > modeTrCandList.capacity(), "intra mode list exceeds buffer capacity");

  UnitArea       currArea = pt.currArea();
  TransformUnit &tu       = *cs.getTU(currArea.chromaPos(), ChannelType::CHROMA);
  CodingUnit    &cu       = *cs.getCU(currArea.chromaPos(), ChannelType::CHROMA);

  //===== initialize list with mode info only =====
  modeTrCandList.clear();
  for (const auto &m: imodeList)
  {
    modeTrCandList.add(m);
  }
  if (pt.currTrDepth != tu.depth)
  {
    return;
  }

  //===== calculate all predictions and transforms =====
  for (auto &modeTrCand: modeTrCandList)
  {
    //--- add intra mode entry ---
    setCuPredDataChroma(cu, modeTrCand, chromaCandModes);
    xPreCalcPrdTransChroma(tu, modeTrCand);
  }

  //===== remove unlikely pred/trans combinations
  xSelectPrTrCandChroma(tu, modeTrCandList, chromaCandModes);
}

void IntraSearch::xSelectPrTrCandLuma(TransformUnit &tu, IModeTrCandListLuma &ptl, const CUCtxIntra &cuCtxIntra)
{
  CHECK(ptl.empty(), "empty candidate list");

  //----- calculate simple cost measures and empty transform lists -----
  PreCostLuma              preCost(tu, m_CABACEstimator, m_pcRdCost);
  std::vector<TrEst::Cost> clDCT, cl;
  {
    clDCT.reserve(ptl.size());
    cl.reserve(ptl.size() * to_underlying(MtsType::NUM));
    int    id = 0;
    size_t n  = 0;
    for (auto &tl: ptl)
    {
      CHECK(tl.trTypes.empty(), "empty candidate list");
      if (tl.trTypes.size() > 1 || tl.trTypes.front() != MtsType::SKIP)
      {
        setCuPredDataLuma(*tu.cu, tl);
        preCost.init(ptl, id, cuCtxIntra);
        for (const MtsType tr: tl.trTypes)
        {
          (tr == MtsType::DCT2_DCT2 ? clDCT : cl).push_back(preCost(tr));
        }
        n++;
        CHECK(clDCT.size() != n, "Initial transform list must contain DCT, unless it only contain TSKIP");
        tl.trTypes.clear();
      }
      id++;
    }
  }

  //----- derive cost threshold and get single sorted list -----
  double costThreshold = std::numeric_limits<double>::max();
  {
    for (const auto &c: clDCT)
    {
      costThreshold = std::min<double>(costThreshold, c.cost);
      ptl[c.id].trTypes.push_back(c.tr);
    }
    costThreshold *= 1.25;   // precost threshold : intra luma together
    sortCL(cl);
  }

  //---- remove transform candidates -----
  while (!cl.empty() && cl.back().cost > costThreshold)
  {
    cl.pop_back();
  }

  //----- add to transform lists -----
  for (const auto &cand: cl)
  {
    ptl[cand.id].trTypes.push_back(cand.tr);
  }

  //----- completely remove unlikely intra modes -----
  for (const auto &c: clDCT)
  {
    if (c.cost > costThreshold && ptl[c.id].trTypes.size() == 1)
    {
      ptl[c.id].trTypes.clear();
    }
  }
}

void IntraSearch::xSelectPrTrCandChroma(TransformUnit &tu, IModeTrCandListChroma &ptl, const uint32_t *chromaCandModes)
{
  CHECK(ptl.empty(), "empty candidate list");

  //----- calculate simple cost measures and empty transform lists -----
  PreCostChroma            preCost(tu, m_CABACEstimator, m_pcRdCost);
  std::vector<TrEst::Cost> clDCT, cl;
  {
    clDCT.reserve(ptl.size());
    cl.reserve(ptl.size() * to_underlying(MtsType::NUM));
    int    id = 0;
    size_t n  = 0;
    for (auto &tl: ptl)
    {
      CHECK(tl.trTypesCmb.empty(), "empty candidate list");
      if (tl.trTypesCmb.size() > 1 || tl.trTypesCmb.front() != MtsType::SKIP)
      {
        setCuPredDataChroma(*tu.cu, tl.chromaMode, chromaCandModes);
        preCost.init(ptl, id, chromaCandModes);
        for (const MtsType tr: tl.trTypesCmb)
        {
          (tr == MtsType::DCT2_DCT2 ? clDCT : cl).push_back(preCost(tr));
        }
        n++;
        CHECK(clDCT.size() != n, "Initial transform list must contain DCT, unless it only contain TSKIP");
        tl.trTypesCmb.clear();
        tl.trTypesSep.clear();
        tl.trTypesJnt.clear();
      }
      id++;
    }
  }

  //----- derive cost threshold and get single sorted list -----
  double costThreshold = std::numeric_limits<double>::max();
  {
    for (const auto &c: clDCT)
    {
      costThreshold = std::min<double>(costThreshold, c.cost);
      ptl[c.id].trTypesCmb.push_back(c.tr);
      ptl[c.id].trTypesSep.push_back(c.tr);
    }
    costThreshold *= 1.35;   // precost threshold : intra chroma together
    sortCL(cl);
  }

  //---- remove transform candidates -----
  while (!cl.empty() && cl.back().cost > costThreshold)
  {
    cl.pop_back();
  }

  //----- add to transform lists -----
  for (const auto &cand: cl)
  {
    auto &trListSepJnt =
      (cand.tr == MtsType::DCT2_DCT2 || cand.tr == MtsType::SKIP ? ptl[cand.id].trTypesSep : ptl[cand.id].trTypesJnt);
    ptl[cand.id].trTypesCmb.push_back(cand.tr);
    trListSepJnt.push_back(cand.tr);
  }

  //----- completely remove unlikely intra modes -----
  for (const auto &c: clDCT)
  {
    if (c.cost > costThreshold && ptl[c.id].trTypesCmb.size() == 1)
    {
      ptl[c.id].trTypesCmb.clear();
      ptl[c.id].trTypesSep.clear();
    }
  }
}

void IntraSearch::xSelectTrCandLuma(TransformUnit &tu, PreTrListLuma &tl)
{
  CHECK(tl.trTypes.empty(), "empty candidate list");
  if (tl.trTypes.size() == 1)
  {
    return;
  }

  //----- calculate simple cost measures and empty transform lists -----
  PreCostLuma              preCost(tu, m_CABACEstimator, m_pcRdCost);
  std::vector<TrEst::Cost> cl;
  TrEst::Cost              costDCT(0, MtsType::NONE, std::numeric_limits<double>::max());
  {
    cl.reserve(to_underlying(MtsType::NUM));
    preCost.init(tl);
    for (const MtsType tr: tl.trTypes)
    {
      if (tr == MtsType::DCT2_DCT2)
      {
        costDCT = preCost(tr);
      }
      else
      {
        cl.push_back(preCost(tr));
      }
    }
    tl.trTypes.clear();
  }
  CHECK(costDCT.tr != MtsType::DCT2_DCT2, "Initial transform list must contain DCT, unless it only contains TSKIP");

  //----- derive cost threshold and get single sorted list -----
  const double costThreshold = costDCT.cost * 1.1;   // precost threshold : intra luma per mode
  tl.trTypes.push_back(costDCT.tr);
  sortCL(cl);

  //---- remove transform candidates -----
  while (!cl.empty() && cl.back().cost > costThreshold)
  {
    cl.pop_back();
  }

  //----- add to transform lists -----
  for (const auto &cand: cl)
  {
    tl.trTypes.push_back(cand.tr);
  }
}

void IntraSearch::xSelectTrCandChroma(TransformUnit &tu, PreTrListChroma &tl)
{
  CHECK(tl.trTypesCmb.empty(), "empty candidate list");
  if (tl.trTypesCmb.size() == 1)
  {
    return;
  }

  //----- calculate simple cost measures and empty transform lists -----
  PreCostChroma            preCost(tu, m_CABACEstimator, m_pcRdCost);
  std::vector<TrEst::Cost> cl;
  TrEst::Cost              costDCT(0, MtsType::NONE, std::numeric_limits<double>::max());
  {
    cl.reserve(to_underlying(MtsType::NUM));
    preCost.init(tl);
    for (const MtsType tr: tl.trTypesCmb)
    {
      if (tr == MtsType::DCT2_DCT2)
      {
        costDCT = preCost(tr);
      }
      else
      {
        cl.push_back(preCost(tr));
      }
    }
    tl.trTypesCmb.clear();
    tl.trTypesSep.clear();
    tl.trTypesJnt.clear();
  }
  CHECK(costDCT.tr != MtsType::DCT2_DCT2, "Initial transform list must contain DCT, unless it only contain TSKIP");

  //----- derive cost threshold and get single sorted list -----
  const double costThreshold = costDCT.cost * 1.1;   // precost threshold : intra chroma per mode
  tl.trTypesCmb.push_back(costDCT.tr);
  tl.trTypesSep.push_back(costDCT.tr);
  sortCL(cl);

  //---- remove transform candidates -----
  while (!cl.empty() && cl.back().cost > costThreshold)
  {
    cl.pop_back();
  }

  //----- add to transform lists -----
  for (const auto &cand: cl)
  {
    auto &trListSepJnt = (cand.tr == MtsType::DCT2_DCT2 || cand.tr == MtsType::SKIP ? tl.trTypesSep : tl.trTypesJnt);
    tl.trTypesCmb.push_back(cand.tr);
    trListSepJnt.push_back(cand.tr);
  }
}

IntraSearch::PreCostLuma::PreCostLuma(TransformUnit &_tu, CABACWriter *_cabacEst, RdCost *_rdCost)
  : TrEst::PreCostBase(CompID::COMP_Y, _tu, _rdCost->getMotionLambda(), 1.15, 1.0)
  , bitEst(_cabacEst)
{}

void IntraSearch::PreCostLuma::init(const IModeTrCandListLuma &il, const int id, const CUCtxIntra &cuCtxIntra)
{
  const auto &im = il[id];
  trList         = &im;
  imodeId        = id;
  imodeBits      = estIModeBits(im, cuCtxIntra);
  TrEst::PreCostBase::initResidual(trList->trTypes, trList->res);
}

void IntraSearch::PreCostLuma::init(const PreTrListLuma &tl)
{
  trList    = &tl;
  imodeId   = 0;
  imodeBits = 0;
  TrEst::PreCostBase::initResidual(trList->trTypes, trList->res);
}

TrEst::Cost IntraSearch::PreCostLuma::operator()(const MtsType tr)
{
  double cost = imodeBits + estTransBits(tr);
  cost += TrEst::PreCostBase::getEstTrCost(tr, trList->trCoeffs);
  return TrEst::Cost(imodeId, tr, cost);
}

double IntraSearch::PreCostLuma::estIModeBits(const ModeInfo &mi, const CUCtxIntra &cuCtxIntra)
{
  CodingUnit &cu  = *tu.cu;
  const bool  upd = bitEst->countWithUpdate(false);
  setCuPredDataLuma(cu, mi);
  bitEst->resetBits();
  bitEst->bdpcm_mode(cu, CompID::COMP_Y);
  bitEst->intra_luma_pred_mode(cu, cuCtxIntra);
  bitEst->countWithUpdate(upd);
  return FRAC_BITS_SCALE * double(bitEst->getEstFracBits());
}

double IntraSearch::PreCostLuma::estTransBits(const MtsType tr)
{
  const auto cbf    = tu.cbf[COMP_Y];
  tu.mtsIdx[COMP_Y] = tr;
  tu.cbf[COMP_Y]    = 1;   // for mts_idx coding
  const bool upd    = bitEst->countWithUpdate(false);
  bitEst->resetBits();
  bitEst->ts_flag(tu, CompID::COMP_Y);
  bitEst->nst_idx(tu, cuCtx);
  bitEst->mts_idx(tu, cuCtx);
  bitEst->countWithUpdate(upd);
  tu.cbf[COMP_Y] = cbf;
  return FRAC_BITS_SCALE * double(bitEst->getEstFracBits());
}

IntraSearch::PreCostChroma::PreCostChroma(TransformUnit &_tu, CABACWriter *_cabacEst, RdCost *_rdCost)
  : TrEst::PreCostBase(CompID::COMP_Cb, _tu, _rdCost->getMotionLambda(), 1.15, 1.0)
  , bitEst(_cabacEst)
{}

void IntraSearch::PreCostChroma::init(const IModeTrCandListChroma &il, const int id, const uint32_t *chromaCandModes)
{
  const auto &im = il[id];
  trList         = &im;
  imodeId        = id;
  imodeBits      = estIModeBits(im.chromaMode, chromaCandModes);
  TrEst::PreCostBase::initResidual(trList->trTypesCmb, trList->res[0], trList->res[1]);
}

void IntraSearch::PreCostChroma::init(const PreTrListChroma &tl)
{
  trList    = &tl;
  imodeId   = 0;
  imodeBits = 0;
  TrEst::PreCostBase::initResidual(trList->trTypesCmb, trList->res[0], trList->res[1]);
}

TrEst::Cost IntraSearch::PreCostChroma::operator()(const MtsType tr)
{
  double cost = imodeBits + estTransBits(tr);
  cost += TrEst::PreCostBase::getEstTrCost(tr, trList->trCoeffs[0], trList->trCoeffs[1]);
  return TrEst::Cost(imodeId, tr, cost);
}

double IntraSearch::PreCostChroma::estIModeBits(const ChromaModeInfo mi, const uint32_t *chromaCandModes)
{
  CodingUnit &cu  = *tu.cu;
  const bool  upd = bitEst->countWithUpdate(false);
  setCuPredDataChroma(cu, mi, chromaCandModes);
  bitEst->resetBits();
  bitEst->bdpcm_mode(cu, CompID::COMP_Cb);
  bitEst->intra_chroma_pred_mode(cu);
  bitEst->countWithUpdate(upd);
  return FRAC_BITS_SCALE * double(bitEst->getEstFracBits());
}

double IntraSearch::PreCostChroma::estTransBits(const MtsType tr)
{
  tu.mtsIdx[COMP_Cb] = tr;
  tu.mtsIdx[COMP_Cr] = tr;
  const bool upd     = bitEst->countWithUpdate(false);
  bitEst->resetBits();
  bitEst->ts_flag(tu, CompID::COMP_Cb);
  bitEst->ts_flag(tu, CompID::COMP_Cr);
  bitEst->nst_idx(tu, cuCtx);
  bitEst->mts_idx(tu, cuCtx);
  bitEst->countWithUpdate(upd);
  return FRAC_BITS_SCALE * double(bitEst->getEstFracBits());
}

void IntraSearch::estIntraPredChromaQT(CodingUnit &cu, Partitioner &partitioner, const double maxCostAllowed)
{
  const ChromaFormat format                = cu.chromaFormat;
  const uint32_t     numberValidComponents = getNumberValidComponents(format);
  CodingStructure   &cs                    = *cu.cs;
  const TempCtx      ctxStart(m_ctxPool, m_CABACEstimator->getCtx());

  cs.setDecomp(cs.area.Cb(), false);

  Distortion bestDist = 0;
  double     bestCost = MAX_DOUBLE;

  //----- check chroma modes -----
  uint32_t chromaCandModes[NUM_CHROMA_MODE];
  PU::getIntraChromaCandModes(cu, chromaCandModes);

  // create a temporary CS
  CodingStructure &saveCS = *m_pSaveCS[0];
  saveCS.pcv              = cs.pcv;
  saveCS.picture          = cs.picture;
  saveCS.compactResize(cs.area);
  saveCS.clearTUs();

  if (CS::isDualITree(cs))
  {
    if (partitioner.canSplit(TU_MAX_TR_SPLIT, cs))
    {
      partitioner.splitCurrArea(TU_MAX_TR_SPLIT, cs);

      do
      {
        cs.addTU(CS::getArea(cs, partitioner.currArea(), partitioner.chType), partitioner.chType).depth =
          partitioner.currTrDepth;
      } while (partitioner.nextPart(cs));

      partitioner.exitCurrSplit();
    }
    else
    {
      cs.addTU(CS::getArea(cs, partitioner.currArea(), partitioner.chType), partitioner.chType);
    }
  }

  auto &orgTUs = m_orgTUs;
  orgTUs.clear();

  // create a store for the TUs
  for (const auto &ptu: cs.tus)
  {
    // for split TUs in HEVC, add the TUs without Chroma parts for correct setting of Cbfs
    if (cu.contains(*ptu, ChannelType::CHROMA))
    {
      saveCS.addTU(*ptu, partitioner.chType);
      orgTUs.push_back(ptu);
    }
  }
  // SATD pre-selecting.
  std::vector<ChromaModeInfo> modesToTest;

  modesToTest.reserve(ENCODER_INTRA_CHROMA_NUM_RDO + 2);   // +2 for the BDPCM when available

  CPelBuf origCb = cs.getOrgBuf(cu.Cb());
  CPelBuf origCr = cs.getOrgBuf(cu.Cr());
  CPelBuf predCb = cs.getPredBuf(cu.Cb());
  CPelBuf predCr = cs.getPredBuf(cu.Cr());

  m_satdCheckerCbCr.init(*m_pcRdCost, origCb, origCr, predCb, predCr, cu.cs->sps->m_bitDepths[ChannelType::CHROMA]);

  xAddChromaCands(cu, modesToTest);
  xAddCclmDeltaSlopeCands(cu, modesToTest);
  xAddCccmCands(cu, modesToTest);
  xAddCcMergeCands(cu, modesToTest);

  // BDPCM candidates are force included to the list when available
  if (CU::bdpcmAllowed(cu, COMP_Cb))
  {
    modesToTest.push_back(ChromaModeInfo(BDPCM_IDX + 1));
    modesToTest.push_back(ChromaModeInfo(BDPCM_IDX + 2));
  }

  xSelectMTCandChroma(cs, partitioner, m_modeTrCandListChroma, modesToTest, chromaCandModes);

  Distortion     baseDist = cs.dist;
  ChromaModeInfo bestMode(-1);
  for (auto &cand: m_modeTrCandListChroma)
  {
    if (cand.trTypesCmb.empty() && cand.valid)
    {
      continue;
    }
    setCuPredDataChroma(cu, cand, chromaCandModes);
    cs.setDecomp(cu.Cb(), false);
    cs.dist = baseDist;

    m_CABACEstimator->getCtx() = ctxStart;
    xRecurIntraCodingChromaQT(cs, partitioner, cand);
    m_CABACEstimator->getCtx() = ctxStart;

    uint64_t   fracBits = xGetIntraFracBitsQT(cs, partitioner, false, true);
    Distortion dist     = cs.dist;
    double     cost     = m_pcRdCost->calcRdCost(fracBits, dist - baseDist);

    //----- compare -----
    if (cost < bestCost)
    {
      for (uint32_t i = getFirstComponentOfChannel(ChannelType::CHROMA); i < numberValidComponents; i++)
      {
        const CompArea &area = cu.blocks[i];

        saveCS.getRecoBuf(area).copyFrom(cs.getRecoBuf(area));
#if KEEP_PRED_AND_RESI_SIGNALS
        saveCS.getPredBuf(area).copyFrom(cs.getPredBuf(area));
        saveCS.getResiBuf(area).copyFrom(cs.getResiBuf(area));
#endif
        saveCS.getPredBuf(area).copyFrom(cs.getPredBuf(area));
        cs.picture->getPredBuf(area).copyFrom(cs.getPredBuf(area));
        cs.picture->getRecoBuf(area).copyFrom(cs.getRecoBuf(area));

        for (uint32_t j = 0; j < saveCS.tus.size(); j++)
        {
          saveCS.tus[j]->copyComponentFrom(*orgTUs[j], area.compID);
        }
      }

      bestCost = cost;
      bestDist = dist;
      bestMode = cand;
    }
  }

  for (uint32_t i = getFirstComponentOfChannel(ChannelType::CHROMA); i < numberValidComponents; i++)
  {
    const CompArea &area = cu.blocks[i];

    cs.getRecoBuf(area).copyFrom(saveCS.getRecoBuf(area));
#if KEEP_PRED_AND_RESI_SIGNALS
    cs.getPredBuf(area).copyFrom(saveCS.getPredBuf(area));
    cs.getResiBuf(area).copyFrom(saveCS.getResiBuf(area));
#endif
    cs.getPredBuf(area).copyFrom(saveCS.getPredBuf(area));
    cs.picture->getPredBuf(area).copyFrom(cs.getPredBuf(area));

    cs.picture->getRecoBuf(area).copyFrom(cs.getRecoBuf(area));

    for (uint32_t j = 0; j < saveCS.tus.size(); j++)
    {
      orgTUs[j]->copyComponentFrom(*saveCS.tus[j], area.compID);
    }
  }

  setCuPredDataChroma(cu, bestMode, chromaCandModes);
  PU::setChromaIntraModeFlag(cu);
  cs.dist = bestDist;

  //----- restore context models -----
  m_CABACEstimator->getCtx() = ctxStart;
}

void IntraSearch::tryAddingToModeList(std::vector<ChromaModeInfo> &modeList, ChromaModeInfo mode, int maxListSize)
{
  if (modeList.size() == 0)
  {
    modeList.push_back(mode);
  }
  else
  {
    auto index = modeList.size();

    while (index > 0 && mode.satdCost < modeList[index - 1].satdCost)
    {
      index--;
    }

    if (index < maxListSize)
    {
      modeList.insert(modeList.begin() + index, mode);

      if (modeList.size() > maxListSize)
      {
        modeList.pop_back();
      }
    }
  }
}

void IntraSearch::xAddCcMergeCands(CodingUnit &cu, std::vector<ChromaModeInfo> &candList)
{
  CompArea areaCb = cu.Cb();
  CompArea areaCr = cu.Cr();

  PelBuf predCb = cu.cs->getPredBuf(areaCb);
  PelBuf predCr = cu.cs->getPredBuf(areaCr);

  if (CU::hasNonLocalCCP(cu))
  {
    CrossCompModels mergeList[MAX_CCP_CAND_LIST_SIZE] = {};

    int numPos = CU::getCCPModelCandidateList(cu, mergeList);

    // Note: Assuming here the luma reference samples are already generated
    int fusionList[MAX_CCP_FUSION_NUM * 2] = { MAX_CCP_FUSION_NUM };
    int numFusionCands                     = cu.cs->slice->m_sps->m_ccMergeFusion ? MAX_CCP_FUSION_NUM : 0;

    reorderCCPCandidates(cu, mergeList, numPos, fusionList);

    for (int ccpMergeIdx = 0; ccpMergeIdx < numPos; ccpMergeIdx++)
    {
      int modeId = mergeList[ccpMergeIdx].dualModel ? MMLM_CHROMA_IDX : LM_CHROMA_IDX;

      ChromaModeInfo modeInfo = ChromaModeInfo(modeId, mergeList[ccpMergeIdx], ccpMergeIdx + 1, false, int64_t(0));

      setCuPredDataChroma(cu, modeInfo, nullptr);

      if (cu.cccmFlag)
      {
        predIntraCCCM(cu, predCb, predCr, false);
      }
      else
      {
        predIntraChromaLM(COMP_Cb, predCb, cu, areaCb, cu.intraDir[ChannelType::CHROMA], false);
        predIntraChromaLM(COMP_Cr, predCr, cu, areaCr, cu.intraDir[ChannelType::CHROMA], false);
      }

      modeInfo.satdCost = m_satdCheckerCbCr.getCost();

      tryAddingToModeList(candList, modeInfo, ENCODER_INTRA_CHROMA_NUM_RDO);
    }

    for (int ccpFusionIdx = 1; ccpFusionIdx <= numFusionCands; ccpFusionIdx++)
    {
      int modeId = LM_CHROMA_IDX;
      int i      = 2 * (ccpFusionIdx - 1);

      ChromaModeInfo modeInfo = ChromaModeInfo(IPRED_CAT_CROSSCOMP_MERGE_FUSION, modeId, mergeList[fusionList[i]],
                                               mergeList[fusionList[i + 1]], ccpFusionIdx, int64_t(0));

      setCuPredDataChroma(cu, modeInfo, nullptr);

      setAndPredCcMergeFusionCand(cu, mergeList[fusionList[i]], mergeList[fusionList[i + 1]], predCb, predCr);

      modeInfo.satdCost = m_satdCheckerCbCr.getCost();

      tryAddingToModeList(candList, modeInfo, ENCODER_INTRA_CHROMA_NUM_RDO);
    }

    cu.idxNonLocalCCP = 0;
  }

  if (CU::hasDecoderDerivedCCP(cu))
  {
    CrossCompModels ccModels[2] = {};

    findDecoderDerivedCcpModel(cu, ccModels[0], ccModels[1]);

    ChromaModeInfo modeInfo =
      ChromaModeInfo(IPRED_CAT_CROSSCOMP_DEC_DERIVED, LM_CHROMA_IDX, ccModels[0], ccModels[1], 1, int64_t(0));

    setCuPredDataChroma(cu, modeInfo, nullptr);

    setAndPredCcMergeFusionCand(cu, ccModels[0], ccModels[1], predCb, predCr);

    modeInfo.satdCost = m_satdCheckerCbCr.getCost();

    tryAddingToModeList(candList, modeInfo, ENCODER_INTRA_CHROMA_NUM_RDO);

    cu.decDerivedCcpMode = 0;
  }
}

void IntraSearch::xAddChromaCands(CodingUnit &cu, std::vector<ChromaModeInfo> &candList)
{
  unsigned chromaCandModes[NUM_CHROMA_MODE];

  PU::getIntraChromaCandModes(cu, chromaCandModes);

  CompArea areaCb = cu.Cb();
  CompArea areaCr = cu.Cr();

  PelBuf predCb = cu.cs->getPredBuf(areaCb);
  PelBuf predCr = cu.cs->getPredBuf(areaCr);

  initIntraPatternChType(cu, areaCb);
  initIntraPatternChType(cu, areaCr);

  xGetLumaRecPixels(cu, areaCb, true);   // Create luma reference for all the CCLM modes

  for (int idx = 0; idx < NUM_CHROMA_MODE; idx++)
  {
    int mode = chromaCandModes[idx];

    if (PU::isLMCMode(mode) && (!PU::isLMCModeEnabled(cu, mode) || cu.slice->m_lmChromaCheckDisable))
    {
      continue;
    }

    if (mode == PLANAR_IDX || mode == DM_CHROMA_IDX)   // DM, Planar (and later also LM) go straight to the RDO
    {
      tryAddingToModeList(candList, ChromaModeInfo(mode, int64_t(0)), ENCODER_INTRA_CHROMA_NUM_RDO);

      continue;
    }

    cu.intraDir[ChannelType::CHROMA] = mode;

    if (PU::isLMCMode(mode))
    {
      predIntraChromaLM(COMP_Cb, predCb, cu, areaCb, mode);
      predIntraChromaLM(COMP_Cr, predCr, cu, areaCr, mode);

      int64_t cost = mode == LM_CHROMA_IDX ? 0 : m_satdCheckerCbCr.getCost();   // LM goes straight to the RDO

      tryAddingToModeList(candList, ChromaModeInfo(mode, cu.ccModels, 0, false, cost), ENCODER_INTRA_CHROMA_NUM_RDO);

      if (CU::hasCcFilterFlag(cu))
      {
        cu.ccFilterFlag    = true;
        cu.ccModels.filter = true;

        predIntraChromaLM(COMP_Cb, predCb, cu, areaCb, mode, false);
        predIntraChromaLM(COMP_Cr, predCr, cu, areaCr, mode, false);

        tryAddingToModeList(candList,
                            ChromaModeInfo(mode, cu.ccModels, 0, cu.ccFilterFlag, m_satdCheckerCbCr.getCost()),
                            ENCODER_INTRA_CHROMA_NUM_RDO);

        cu.ccFilterFlag = false;
      }
    }
    else
    {
      initPredIntraParams(cu, areaCb, *cu.cs->sps);
      predIntraAng(COMP_Cb, predCb, cu, true, false);
      initPredIntraParams(cu, areaCr, *cu.cs->sps);
      predIntraAng(COMP_Cr, predCr, cu, true, false);

      tryAddingToModeList(candList, ChromaModeInfo(mode, m_satdCheckerCbCr.getCost()), ENCODER_INTRA_CHROMA_NUM_RDO);
    }
  }

  addDimdChromaAsCandidate(cu, areaCb, areaCr, predCb, predCr, candList);
}

void IntraSearch::addDimdChromaAsCandidate(CodingUnit &cu, CompArea &areaCb, CompArea &areaCr, PelBuf &predCb,
                                           PelBuf &predCr, std::vector<ChromaModeInfo> &candList)
{
  if (!cu.slice->m_sps->m_useDIMDChroma)
  {
    return;
  }

  DimdData dimdData;
  deriveDimdChromaMode(dimdData, cu);

  // in the case of DIMD, we have to set the intra mode to the derived one
  cu.intraDir[ChannelType::CHROMA] = dimdData.blendMode[0];

  initPredIntraParams(cu, areaCb, *cu.cs->sps);
  predIntraAng(COMP_Cb, predCb, cu, true, false);
  initPredIntraParams(cu, areaCr, *cu.cs->sps);
  predIntraAng(COMP_Cr, predCr, cu, true, false);

  ChromaModeInfo modeInfo(dimdData.blendMode[0], m_satdCheckerCbCr.getCost());
  modeInfo.dimdFlag = true;

  tryAddingToModeList(candList, modeInfo, ENCODER_INTRA_CHROMA_NUM_RDO);
}

void IntraSearch::xAddCccmCands(CodingUnit &cu, std::vector<ChromaModeInfo> &candList)
{
  if (!cu.cs->sps->m_CCCM)
  {
    return;
  }

  const int cccmCandIntraModes[NUM_LMC_MODE] = { LM_CHROMA_IDX, MMLM_CHROMA_IDX, MDLM_L_IDX,
                                                 MDLM_T_IDX,    MMLM_L_IDX,      MMLM_T_IDX };

  CodingStructure &cs = *(cu.cs);

  PelBuf predCb = cs.getPredBuf(cu.Cb());
  PelBuf predCr = cs.getPredBuf(cu.Cr());

  if (PU::hasBvgCccmFlag(cu))   // bvList needs to be fetched before filling buffers
  {
    PU::getBvgCccmCands(cu);
  }

  cccmCreateLumaRefs(cu, true);

  int64_t bestBaseSatd = 0;

  for (int candCccmMode = CONV_MODEL_CCCM_INTRA_FIRST; candCccmMode <= CONV_MODEL_CCCM_INTRA_LAST; candCccmMode++)
  {
    cu.cccmFlag = true;
    cu.cccmType = ConvModelType(candCccmMode);

    if (cu.cccmType == CONV_MODEL_CCCM_BVG && cu.numBvgCands == 0)
    {
      continue;
    }

    for (int candIntraMode: cccmCandIntraModes)
    {
      if (PU::cccmAvailable(cu, candIntraMode, cu.cccmType))
      {
        cu.intraDir[ChannelType::CHROMA] = candIntraMode;

        predIntraCCCM(cu, predCb, predCr);

        int64_t satdCost = m_satdCheckerCbCr.getCost();

        tryAddingToModeList(candList, ChromaModeInfo(candIntraMode, cu.cccmType, cu.ccModels, false, satdCost),
                            ENCODER_INTRA_CHROMA_NUM_RDO);

        if (CU::hasCcFilterFlag(cu))
        {
          cu.ccFilterFlag    = true;
          cu.ccModels.filter = true;

          predIntraCCCM(cu, predCb, predCr, false);

          int64_t satdCostFiltered = m_satdCheckerCbCr.getCost();

          tryAddingToModeList(
            candList, ChromaModeInfo(candIntraMode, cu.cccmType, cu.ccModels, cu.ccFilterFlag, satdCostFiltered),
            ENCODER_INTRA_CHROMA_NUM_RDO);

          cu.ccFilterFlag = false;

          satdCost = std::min(satdCostFiltered, satdCost);
        }

        // todo: check if below conditions still make sense

        // Move on to the next CCCM type if base model for this type is not competitive against earlier base models
        if (candIntraMode == LM_CHROMA_IDX)
        {
          if (candCccmMode == CONV_MODEL_CCCM_INTRA_FIRST || satdCost < bestBaseSatd)
          {
            bestBaseSatd = satdCost;
          }
          else
          {
            break;
          }
        }
      }
    }
  }

  cu.cccmFlag = false;
  cu.cccmType = CONV_MODEL_UNDEFINED;
}

void IntraSearch::xAddCclmDeltaSlopeCands(CodingUnit &cu, std::vector<ChromaModeInfo> &candList)
{
  if (!cu.cs->sps->m_LMChroma)
  {
    return;
  }

  for (int mode = LM_CHROMA_IDX; mode <= MMLM_T_IDX; mode++)
  {
    if (PU::hasCclmDeltaFlag(cu, mode))
    {
      int64_t satdCbCr = 0;

      cu.cclmOffsets.setAllZero();
      cu.intraDir[ChannelType::CHROMA] = mode;

      for (int comp = COMP_Cb; comp <= COMP_Cr; comp++)
      {
        CompID  compID    = CompID(comp);
        int     compDelta = 0;
        int64_t compSatd  = 0;

        CclmModel cclmModelBase;

        xGetLMParameters_LMS(cu, compID, compID == COMP_Cb ? cu.Cb() : cu.Cr(), cclmModelBase);

        cu.ccModels.setCclmModel(cclmModelBase, compID,
                                 PU::isMultiModeLM(mode));   // Start with unadjusted parameters of this color component

        xFindBestCclmDeltaSlopeSATD(cu, compID, cclmModelBase, 0, compDelta, compSatd);

        if (PU::isMultiModeLM(mode))
        {
          xFindBestCclmDeltaSlopeSATD(cu, compID, cclmModelBase, 1, compDelta, compSatd);
        }

        satdCbCr += compSatd;
      }

      if (cu.cclmOffsets.isActive())
      {
        tryAddingToModeList(candList, ChromaModeInfo(mode, cu.cclmOffsets, cu.ccModels, satdCbCr),
                            ENCODER_INTRA_CHROMA_NUM_RDO);
      }
    }
  }

  cu.cclmOffsets.setAllZero();
}

void IntraSearch::xFindBestCclmDeltaSlopeSATD(CodingUnit &cu, CompID compID, CclmModel &cclmModelBase, int cclmModelInd,
                                              int &deltaBest, int64_t &sadBest)
{
  CodingStructure &cs        = *(cu.cs);
  CompArea         area      = compID == COMP_Cb ? cu.Cb() : cu.Cr();
  PelBuf           orgBuf    = cs.getOrgBuf(area);
  PelBuf           predBuf   = cs.getPredBuf(area);
  int              maxOffset = 4;
  int              mode      = cu.intraDir[ChannelType::CHROMA];

  DistParam distParamSad;
  DistParam distParamSatd;

  m_pcRdCost->setDistParam(distParamSad, orgBuf, predBuf, cu.cs->sps->m_bitDepths[ChannelType::CHROMA], compID, 0);
  m_pcRdCost->setDistParam(distParamSatd, orgBuf, predBuf, cu.cs->sps->m_bitDepths[ChannelType::CHROMA], compID, 1);

  distParamSad.applyWeight  = false;
  distParamSatd.applyWeight = false;

  sadBest = -1;

  auto updateModelAndSetForCU = [&](const int offset)
  {
    CclmModelSingle cclmModelCand = cclmModelBase.model[cclmModelInd];

    xUpdateCclmModel(cclmModelCand, offset);

    if (compID == COMP_Cb)
    {
      cu.ccModels.modelCb[cclmModelInd] = ConvModel(cclmModelCand);
    }
    else
    {
      cu.ccModels.modelCr[cclmModelInd] = ConvModel(cclmModelCand);
    }

    cu.cclmOffsets.setOffset(compID, cclmModelInd, offset);
  };

  // Search positive offsets
  for (int offset = 0; offset <= maxOffset; offset++)
  {
    updateModelAndSetForCU(offset);

    predIntraChromaLM(compID, predBuf, cu, area, mode, false);

    int64_t sad     = distParamSad.distFunc(distParamSad) * 2;
    int64_t satd    = distParamSatd.distFunc(distParamSatd);
    int64_t sadThis = std::min(sad, satd);

    if (sadBest == -1 || sadThis < sadBest)
    {
      sadBest   = sadThis;
      deltaBest = offset;
    }
    else
    {
      break;
    }
  }

  // Search negative offsets only if positives didn't help
  if (deltaBest == 0)
  {
    for (int offset = -1; offset >= -maxOffset; offset--)
    {
      updateModelAndSetForCU(offset);

      predIntraChromaLM(compID, predBuf, cu, area, mode, false);

      int64_t sad     = distParamSad.distFunc(distParamSad) * 2;
      int64_t satd    = distParamSatd.distFunc(distParamSatd);
      int64_t sadThis = std::min(sad, satd);

      if (sadThis < sadBest)
      {
        sadBest   = sadThis;
        deltaBest = offset;
      }
      else
      {
        break;
      }
    }
  }

  updateModelAndSetForCU(deltaBest);
}

void IntraSearch::PLTSearch(CodingStructure &cs, Partitioner &partitioner, CompID compBegin, uint32_t numComp)
{
  CodingUnit    &cu     = *cs.getCU(partitioner.chType);
  TransformUnit &tu     = *cs.getTU(partitioner.chType);
  uint32_t       height = cu.block(compBegin).height;
  uint32_t       width  = cu.block(compBegin).width;
  if (m_encCfg->m_lmcsEnabled && (cs.slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag))
  {
    cs.getPredBuf().copyFrom(cs.getOrgBuf());
    cs.getPredBuf().Y().rspSignal(m_pcReshape->m_fwdLUT);
  }
  cu.lastPLTSize[compBegin] = cs.prevPLT.curPLTSize[compBegin];
  // derive palette
  xDerivePLTLossy(cs, partitioner, compBegin, numComp);
  reorderPLT(cs, partitioner, compBegin, numComp);

  bool idxExist[MAXPLTSIZE + 1] = { false };
  xPreCalcPLTIndexRD(cs, partitioner, compBegin, numComp);   // Pre-calculate distortions for each pixel
  double rdCost = MAX_DOUBLE;
  xDeriveIndexMap(cs, partitioner, compBegin, numComp, PLT_SCAN_HORTRAV, rdCost,
                  idxExist);   // Optimize palette index map (horizontal scan)
  if ((cu.curPLTSize[compBegin] + cu.useEscape[compBegin]) > 1)
  {
    xDeriveIndexMap(cs, partitioner, compBegin, numComp, PLT_SCAN_VERTRAV, rdCost,
                    idxExist);   // Optimize palette index map (vertical scan)
  }
  // Remove unused palette entries
  uint8_t newPLTSize = 0;
  int     idxMapping[MAXPLTSIZE + 1];
  memset(idxMapping, -1, sizeof(int) * (MAXPLTSIZE + 1));
  for (int i = 0; i < cu.curPLTSize[compBegin]; i++)
  {
    if (idxExist[i])
    {
      idxMapping[i] = newPLTSize;
      newPLTSize++;
    }
  }
  idxMapping[cu.curPLTSize[compBegin]] = cu.useEscape[compBegin] ? newPLTSize : -1;
  if (newPLTSize != cu.curPLTSize[compBegin])   // there exist unused palette entries
  {   // update palette table and reuseflag
    Pel curPLTtmp[MAX_NUM_COMP][MAXPLTSIZE];
    int reuseFlagIdx = 0, curPLTtmpIdx = 0, reuseEntrySize = 0;
    memset(cu.reuseflag[compBegin], false, sizeof(bool) * MAXPLTPREDSIZE);
    int compBeginTmp = compBegin;
    int numCompTmp   = numComp;
    for (int curIdx = 0; curIdx < cu.curPLTSize[compBegin]; curIdx++)
    {
      if (idxExist[curIdx])
      {
        for (int comp = compBeginTmp; comp < (compBeginTmp + numCompTmp); comp++)
        {
          curPLTtmp[comp][curPLTtmpIdx] = cu.curPLT[comp][curIdx];
        }

        // Update reuse flags
        if (curIdx < cu.reusePLTSize[compBegin])
        {
          bool match = false;
          for (; reuseFlagIdx < cs.prevPLT.curPLTSize[compBegin]; reuseFlagIdx++)
          {
            bool matchTmp = true;
            for (int comp = compBegin; comp < (compBegin + numComp); comp++)
            {
              matchTmp = matchTmp && (curPLTtmp[comp][curPLTtmpIdx] == cs.prevPLT.curPLT[comp][reuseFlagIdx]);
            }
            if (matchTmp)
            {
              match = true;
              break;
            }
          }
          if (match)
          {
            cu.reuseflag[compBegin][reuseFlagIdx] = true;
            reuseEntrySize++;
          }
        }
        curPLTtmpIdx++;
      }
    }
    cu.reusePLTSize[compBegin] = reuseEntrySize;
    // update palette table
    cu.curPLTSize[compBegin]   = newPLTSize;
    for (int comp = compBeginTmp; comp < (compBeginTmp + numCompTmp); comp++)
    {
      memcpy(cu.curPLT[comp], curPLTtmp[comp], sizeof(Pel) * cu.curPLTSize[compBegin]);
    }
  }
  cu.useRotation[compBegin] = m_bestScanRotationMode;
  int indexMaxSize          = cu.useEscape[compBegin] ? (cu.curPLTSize[compBegin] + 1) : cu.curPLTSize[compBegin];
  if (indexMaxSize <= 1)
  {
    cu.useRotation[compBegin] = false;
  }
  // reconstruct pixel
  PelBuf curPLTIdx = tu.getcurPLTIdx(toChannelType(compBegin));
  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      curPLTIdx.at(x, y) = idxMapping[curPLTIdx.at(x, y)];
      if (curPLTIdx.at(x, y) == cu.curPLTSize[compBegin])
      {
        xCalcPixelPred(cs, partitioner, y, x, compBegin, numComp);
      }
      else
      {
        for (uint32_t compID = compBegin; compID < (compBegin + numComp); compID++)
        {
          CompArea area   = cu.blocks[compID];
          PelBuf   recBuf = cs.getRecoBuf(area);
          uint32_t scaleX = getComponentScaleX((CompID)COMP_Cb, cs.sps->m_chromaFormatIdc);
          uint32_t scaleY = getComponentScaleY((CompID)COMP_Cb, cs.sps->m_chromaFormatIdc);
          if (compBegin != COMP_Y || compID == COMP_Y)
          {
            recBuf.at(x, y) = cu.curPLT[compID][curPLTIdx.at(x, y)];
          }
          else if (compBegin == COMP_Y && compID != COMP_Y && y % (1 << scaleY) == 0 && x % (1 << scaleX) == 0)
          {
            recBuf.at(x >> scaleX, y >> scaleY) = cu.curPLT[compID][curPLTIdx.at(x, y)];
          }
        }
      }
    }
  }

  cs.getPredBuf().fill(0);
  cs.getResiBuf().fill(0);
  cs.getOrgResiBuf().fill(0);

  cs.fracBits           = MAX_UINT;
  cs.cost               = MAX_DOUBLE;
  Distortion distortion = 0;
  for (uint32_t comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    const CompID compID = CompID(comp);
    CPelBuf      reco   = cs.getRecoBuf(compID);
    CPelBuf      org    = cs.getOrgBuf(compID);
#if WCG_EXT
    if (m_encCfg->m_lumaLevelToDeltaQPMapping.isEnabled() ||
        (m_encCfg->m_lmcsEnabled && (cs.slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag)))
    {
      const CPelBuf orgLuma = cs.getOrgBuf(cs.area.blocks[COMP_Y]);

      if (compID == COMP_Y && !(m_encCfg->m_lumaLevelToDeltaQPMapping.isEnabled()))
      {
        const CompArea &areaY = cu.Y();

        CompArea tmpArea1(COMP_Y, areaY.chromaFormat, Position(0, 0), areaY.size());
        PelBuf   tmpRecLuma = m_tmpStorageCtu.getBuf(tmpArea1);
        tmpRecLuma.copyFrom(reco);
        tmpRecLuma.rspSignal(m_pcReshape->m_invLUT);
        distortion += m_pcRdCost->getDistPart(org, tmpRecLuma, cs.sps->m_bitDepths[toChannelType(compID)], compID,
                                              DFunc::SSE_WTD, &orgLuma);
      }
      else
      {
        distortion += m_pcRdCost->getDistPart(org, reco, cs.sps->m_bitDepths[toChannelType(compID)], compID,
                                              DFunc::SSE_WTD, &orgLuma);
      }
    }
    else
#endif
    {
      distortion += m_pcRdCost->getDistPart(org, reco, cs.sps->m_bitDepths[toChannelType(compID)], compID, DFunc::SSE);
    }
  }

  cs.dist += distortion;
  const CompArea &area = cu.blocks[compBegin];
  cs.setDecomp(area);
  cs.picture->getRecoBuf(area).copyFrom(cs.getRecoBuf(area));
}

void IntraSearch::xCalcPixelPredRD(CodingStructure &cs, Partitioner &partitioner, Pel *orgBuf, Pel *paPixelValue,
                                   Pel *paRecoValue, CompID compBegin, uint32_t numComp)
{
  CodingUnit    &cu = *cs.getCU(partitioner.chType);
  TransformUnit &tu = *cs.getTU(partitioner.chType);

  int qp[3];
  int qpRem[3];
  int qpPer[3];
  int quantiserScale[3];
  int quantiserRightShift[3];
  int rightShiftOffset[3];
  int invquantiserRightShift[3];
  int add[3];

  for (uint32_t ch = compBegin; ch < (compBegin + numComp); ch++)
  {
    QpParam cQP(tu, CompID(ch));
    qp[ch]                     = cQP.Qp(true);
    qpRem[ch]                  = qp[ch] % 6;
    qpPer[ch]                  = qp[ch] / 6;
    quantiserScale[ch]         = g_quantScales[0][qpRem[ch]];
    quantiserRightShift[ch]    = QUANT_SHIFT + qpPer[ch];
    rightShiftOffset[ch]       = 1 << (quantiserRightShift[ch] - 1);
    invquantiserRightShift[ch] = IQUANT_SHIFT;
    add[ch]                    = 1 << (invquantiserRightShift[ch] - 1);
  }

  for (uint32_t ch = compBegin; ch < (compBegin + numComp); ch++)
  {
    const int channelBitDepth = cu.cs->sps->m_bitDepths[toChannelType((CompID)ch)];
    paPixelValue[ch] =
      Pel(std::max<int>(0, ((orgBuf[ch] * quantiserScale[ch] + rightShiftOffset[ch]) >> quantiserRightShift[ch])));
    assert(paPixelValue[ch] < (1 << (channelBitDepth + 1)));
    paRecoValue[ch] =
      (((paPixelValue[ch] * g_invQuantScales[0][qpRem[ch]]) << qpPer[ch]) + add[ch]) >> invquantiserRightShift[ch];
    paRecoValue[ch] = Pel(ClipBD<int>(paRecoValue[ch], channelBitDepth));   // to be checked
  }
}

void IntraSearch::xPreCalcPLTIndexRD(CodingStructure &cs, Partitioner &partitioner, CompID compBegin, uint32_t numComp)
{
  CodingUnit &cu       = *cs.getCU(partitioner.chType);
  uint32_t    height   = cu.block(compBegin).height;
  uint32_t    width    = cu.block(compBegin).width;
  bool        lossless = (m_encCfg->m_costMode == COST_LOSSLESS_CODING && cs.slice->m_isLossless);

  CPelBuf orgBuf[3];
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    CompArea area = cu.blocks[comp];
    if (m_encCfg->m_lmcsEnabled && (cs.slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag))
    {
      orgBuf[comp] = cs.getPredBuf(area);
    }
    else
    {
      orgBuf[comp] = cs.getOrgBuf(area);
    }
  }

  int      rasPos;
  uint32_t scaleX = getComponentScaleX(COMP_Cb, cs.sps->m_chromaFormatIdc);
  uint32_t scaleY = getComponentScaleY(COMP_Cb, cs.sps->m_chromaFormatIdc);
  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      rasPos = y * width + x;
      ;
      // chroma discard
      bool discardChroma = (compBegin == COMP_Y) && (y & scaleY || x & scaleX);
      Pel  curPel[3];
      for (int comp = compBegin; comp < (compBegin + numComp); comp++)
      {
        uint32_t pX1 = (comp > 0 && compBegin == COMP_Y) ? (x >> scaleX) : x;
        uint32_t pY1 = (comp > 0 && compBegin == COMP_Y) ? (y >> scaleY) : y;
        curPel[comp] = orgBuf[comp].at(pX1, pY1);
      }

      uint8_t pltIdx   = 0;
      double  minError = MAX_DOUBLE;
      uint8_t bestIdx  = 0;
      for (uint8_t z = 0; z < cu.curPLTSize[compBegin]; z++)
      {
        m_indexError[z][rasPos] = minError;
      }
      while (pltIdx < cu.curPLTSize[compBegin])
      {
        uint64_t sqrtError = 0;
        if (lossless)
        {
          for (int comp = compBegin; comp < (discardChroma ? 1 : (compBegin + numComp)); comp++)
          {
            sqrtError += int64_t(abs(curPel[comp] - cu.curPLT[comp][pltIdx]));
          }
          if (sqrtError == 0)
          {
            m_indexError[pltIdx][rasPos] = (double)sqrtError;
            minError                     = (double)sqrtError;
            bestIdx                      = pltIdx;
            break;
          }
        }
        else
        {
          for (int comp = compBegin; comp < (discardChroma ? 1 : (compBegin + numComp)); comp++)
          {
            int64_t tmpErr = int64_t(curPel[comp] - cu.curPLT[comp][pltIdx]);
            if (isChroma((CompID)comp))
            {
              sqrtError += uint64_t(tmpErr * tmpErr * ENC_CHROMA_WEIGHTING);
            }
            else
            {
              sqrtError += tmpErr * tmpErr;
            }
          }
          m_indexError[pltIdx][rasPos] = (double)sqrtError;
          if (sqrtError < minError)
          {
            minError = (double)sqrtError;
            bestIdx  = pltIdx;
          }
        }
        pltIdx++;
      }

      Pel paPixelValue[3], paRecoValue[3];
      if (!lossless)
      {
        xCalcPixelPredRD(cs, partitioner, curPel, paPixelValue, paRecoValue, compBegin, numComp);
      }
      uint64_t error = 0, rate = 0;
      for (int comp = compBegin; comp < (discardChroma ? 1 : (compBegin + numComp)); comp++)
      {
        if (lossless)
        {
          rate += getEpExGolombNumBins(curPel[comp], 5);
        }
        else
        {
          int64_t tmpErr = int64_t(curPel[comp] - paRecoValue[comp]);
          if (isChroma((CompID)comp))
          {
            error += uint64_t(tmpErr * tmpErr * ENC_CHROMA_WEIGHTING);
          }
          else
          {
            error += tmpErr * tmpErr;
          }
          rate += getEpExGolombNumBins(paPixelValue[comp], 5);   // encode quantized escape color
        }
      }
      double rdCost                                  = (double)error + m_pcRdCost->getLambda() * (double)rate;
      m_indexError[cu.curPLTSize[compBegin]][rasPos] = rdCost;
      if (rdCost < minError)
      {
        minError = rdCost;
        bestIdx  = (uint8_t)cu.curPLTSize[compBegin];
      }
      m_minErrorIndexMap[rasPos] = bestIdx;   // save the optimal index of the current pixel
    }
  }
}

void IntraSearch::xDeriveIndexMap(CodingStructure &cs, Partitioner &partitioner, CompID compBegin, uint32_t numComp,
                                  PLTScanMode pltScanMode, double &minCost, bool *idxExist)
{
  CodingUnit    &cu     = *cs.getCU(partitioner.chType);
  TransformUnit &tu     = *cs.getTU(partitioner.chType);
  uint32_t       height = cu.block(compBegin).height;
  uint32_t       width  = cu.block(compBegin).width;

  int   total    = height * width;
  Pel  *runIndex = tu.getPLTIndex(toChannelType(compBegin));
  bool *runType  = tu.getRunTypes(toChannelType(compBegin));
  m_scanOrder    = g_scanOrder[SCAN_UNGROUPED][pltScanMode ? CoeffScanType::TRAV_VER : CoeffScanType::TRAV_HOR]
                           [gp_sizeIdxInfo->idxFrom(width)][gp_sizeIdxInfo->idxFrom(height)];
  // Trellis initialization
  for (int i = 0; i < 2; i++)
  {
    memset(m_prevRunTypeRDOQ[i], 0, sizeof(Pel) * NUM_TRELLIS_STATE);
    memset(m_prevRunPosRDOQ[i], 0, sizeof(int) * NUM_TRELLIS_STATE);
    memset(m_stateCostRDOQ[i], 0, sizeof(double) * NUM_TRELLIS_STATE);
  }
  for (int state = 0; state < NUM_TRELLIS_STATE; state++)
  {
    m_statePtRDOQ[state][0] = 0;
  }
  // Context modeling
  const FracBitsAccess &fracBits = m_CABACEstimator->getCtx().getFracBitsAcess();
  BinFracBits           fracBitsPltCopyFlagIndex[RUN_IDX_THRE + 1];
  for (int dist = 0; dist <= RUN_IDX_THRE; dist++)
  {
    const unsigned ctxId           = DeriveCtx::CtxPltCopyFlag(PLT_RUN_INDEX, dist);
    fracBitsPltCopyFlagIndex[dist] = fracBits.getFracBitsArray(Ctx::IdxRunModel(ctxId));
  }
  BinFracBits fracBitsPltCopyFlagAbove[RUN_IDX_THRE + 1];
  for (int dist = 0; dist <= RUN_IDX_THRE; dist++)
  {
    const unsigned ctxId           = DeriveCtx::CtxPltCopyFlag(PLT_RUN_COPY, dist);
    fracBitsPltCopyFlagAbove[dist] = fracBits.getFracBitsArray(Ctx::CopyRunModel(ctxId));
  }
  const BinFracBits fracBitsPltRunType = fracBits.getFracBitsArray(Ctx::RunTypeFlag());

  // Trellis RDO per CG
  bool contTrellisRD = true;
  for (int subSetId = 0; (subSetId <= (total - 1) >> LOG2_PALETTE_CG_SIZE) && contTrellisRD; subSetId++)
  {
    int minSubPos = subSetId << LOG2_PALETTE_CG_SIZE;
    int maxSubPos = minSubPos + (1 << LOG2_PALETTE_CG_SIZE);
    maxSubPos     = (maxSubPos > total) ? total : maxSubPos;   // if last position is out of the current CU size
    contTrellisRD =
      xDeriveSubblockIndexMap(cs, partitioner, compBegin, pltScanMode, minSubPos, maxSubPos, fracBitsPltRunType,
                              fracBitsPltCopyFlagIndex, fracBitsPltCopyFlagAbove, minCost, (bool)pltScanMode);
  }
  if (!contTrellisRD)
  {
    return;
  }

  // best state at the last scan position
  double  sumRdCost = MAX_DOUBLE;
  uint8_t bestState = 0;
  for (uint8_t state = 0; state < NUM_TRELLIS_STATE; state++)
  {
    if (m_stateCostRDOQ[0][state] < sumRdCost)
    {
      sumRdCost = m_stateCostRDOQ[0][state];
      bestState = state;
    }
  }

  bool    checkRunTable[MAX_CU_BLKSIZE_PLT * MAX_CU_BLKSIZE_PLT];
  uint8_t checkIndexTable[MAX_CU_BLKSIZE_PLT * MAX_CU_BLKSIZE_PLT];
  uint8_t bestStateTable[MAX_CU_BLKSIZE_PLT * MAX_CU_BLKSIZE_PLT];
  uint8_t nextState = bestState;
  // best trellis path
  for (int i = (width * height - 1); i >= 0; i--)
  {
    bestStateTable[i] = nextState;
    int rasterPos     = m_scanOrder[i].idx;
    nextState         = m_statePtRDOQ[nextState][rasterPos];
  }
  // reconstruct index and runs based on the state pointers
  for (int i = 0; i < (width * height); i++)
  {
    int rasterPos = m_scanOrder[i].idx;
    int abovePos  = (pltScanMode == PLT_SCAN_HORTRAV) ? m_scanOrder[i].idx - width : m_scanOrder[i].idx - 1;
    nextState     = bestStateTable[i];
    if (nextState == 0)   // same as the previous
    {
      checkRunTable[rasterPos] = checkRunTable[m_scanOrder[i - 1].idx];
      if (checkRunTable[rasterPos] == PLT_RUN_INDEX)
      {
        checkIndexTable[rasterPos] = checkIndexTable[m_scanOrder[i - 1].idx];
      }
      else
      {
        checkIndexTable[rasterPos] = checkIndexTable[abovePos];
      }
    }
    else if (nextState == 1)   // CopyAbove mode
    {
      checkRunTable[rasterPos]   = PLT_RUN_COPY;
      checkIndexTable[rasterPos] = checkIndexTable[abovePos];
    }
    else if (nextState == 2)   // Index mode
    {
      checkRunTable[rasterPos]   = PLT_RUN_INDEX;
      checkIndexTable[rasterPos] = m_minErrorIndexMap[rasterPos];
    }
  }

  // Escape flag
  m_bestEscape = false;
  for (int pos = 0; pos < (width * height); pos++)
  {
    uint8_t index = checkIndexTable[pos];
    if (index == cu.curPLTSize[compBegin])
    {
      m_bestEscape = true;
      break;
    }
  }

  // Horizontal scan v.s vertical scan
  if (sumRdCost < minCost)
  {
    cu.useEscape[compBegin] = m_bestEscape;
    m_bestScanRotationMode  = pltScanMode;
    memset(idxExist, false, sizeof(bool) * (MAXPLTSIZE + 1));
    for (int pos = 0; pos < (width * height); pos++)
    {
      runIndex[pos]                  = checkIndexTable[pos];
      runType[pos]                   = checkRunTable[pos];
      idxExist[checkIndexTable[pos]] = true;
    }
    minCost = sumRdCost;
  }
}

bool IntraSearch::xDeriveSubblockIndexMap(CodingStructure &cs, Partitioner &partitioner, CompID compBegin,
                                          PLTScanMode pltScanMode, int minSubPos, int maxSubPos,
                                          const BinFracBits &fracBitsPltRunType,
                                          const BinFracBits *fracBitsPltIndexINDEX,
                                          const BinFracBits *fracBitsPltIndexCOPY, const double minCost, bool useRotate)
{
  CodingUnit &cu            = *cs.getCU(partitioner.chType);
  uint32_t    height        = cu.block(compBegin).height;
  uint32_t    width         = cu.block(compBegin).width;
  int         indexMaxValue = cu.curPLTSize[compBegin];

  int refId = 0;
  int currRasterPos, currScanPos, prevScanPos, aboveScanPos, roffset;
  int log2Width  = (pltScanMode == PLT_SCAN_HORTRAV) ? floorLog2(width) : floorLog2(height);
  int buffersize = (pltScanMode == PLT_SCAN_HORTRAV) ? 2 * width : 2 * height;
  for (int curPos = minSubPos; curPos < maxSubPos; curPos++)
  {
    currRasterPos = m_scanOrder[curPos].idx;
    prevScanPos   = (curPos == 0) ? 0 : (curPos - 1) % buffersize;
    roffset       = (curPos >> log2Width) << log2Width;
    aboveScanPos  = roffset - (curPos - roffset + 1);
    aboveScanPos %= buffersize;
    currScanPos = curPos % buffersize;
    if ((pltScanMode == PLT_SCAN_HORTRAV && curPos < width) || (pltScanMode == PLT_SCAN_VERTRAV && curPos < height))
    {
      aboveScanPos = -1;   // first column/row: above row is not valid
    }

    // Trellis stats:
    // 1st state: same as previous scanned sample
    // 2nd state: Copy_Above mode
    // 3rd state: Index mode
    // Loop of current state
    for (int curState = 0; curState < NUM_TRELLIS_STATE; curState++)
    {
      double  minRdCost         = MAX_DOUBLE;
      int     minState          = 0;   // best prevState
      uint8_t bestRunIndex      = 0;
      bool    bestRunType       = 0;
      bool    bestPrevCodedType = 0;
      int     bestPrevCodedPos  = 0;
      if ((curState == 0 && curPos == 0) || (curState == 1 && aboveScanPos < 0))   // state not available
      {
        m_stateCostRDOQ[1 - refId][curState] = MAX_DOUBLE;
        continue;
      }

      bool    runType  = 0;
      uint8_t runIndex = 0;
      if (curState == 1)   // 2nd state: Copy_Above mode
      {
        runType = PLT_RUN_COPY;
      }
      else if (curState == 2)   // 3rd state: Index mode
      {
        runType  = PLT_RUN_INDEX;
        runIndex = m_minErrorIndexMap[currRasterPos];
      }

      // Loop of previous state
      for (int stateID = 0; stateID < NUM_TRELLIS_STATE; stateID++)
      {
        if (m_stateCostRDOQ[refId][stateID] == MAX_DOUBLE)
        {
          continue;
        }
        if (curState == 0)   // 1st state: same as previous scanned sample
        {
          runType  = m_runMapRDOQ[refId][stateID][prevScanPos];
          runIndex = (runType == PLT_RUN_INDEX) ? m_indexMapRDOQ[refId][stateID][prevScanPos]
                                                : m_indexMapRDOQ[refId][stateID][aboveScanPos];
        }
        else if (curState == 1)   // 2nd state: Copy_Above mode
        {
          runIndex = m_indexMapRDOQ[refId][stateID][aboveScanPos];
        }
        bool    prevRunType   = m_runMapRDOQ[refId][stateID][prevScanPos];
        uint8_t prevRunIndex  = m_indexMapRDOQ[refId][stateID][prevScanPos];
        uint8_t aboveRunIndex = (aboveScanPos >= 0) ? m_indexMapRDOQ[refId][stateID][aboveScanPos] : 0;
        int     dist          = curPos - m_prevRunPosRDOQ[refId][stateID] - 1;
        double  rdCost        = m_stateCostRDOQ[refId][stateID];
        if (rdCost >= minRdCost)
        {
          continue;
        }

        // Calculate Rd cost
        bool               prevCodedRunType = m_prevRunTypeRDOQ[refId][stateID];
        int                prevCodedPos     = m_prevRunPosRDOQ[refId][stateID];
        const BinFracBits *fracBitsPt =
          (m_prevRunTypeRDOQ[refId][stateID] == PLT_RUN_INDEX) ? fracBitsPltIndexINDEX : fracBitsPltIndexCOPY;
        rdCost += xRateDistOptPLT(runType, runIndex, prevRunType, prevRunIndex, aboveRunIndex, prevCodedRunType,
                                  prevCodedPos, curPos, (pltScanMode == PLT_SCAN_HORTRAV) ? width : height, dist,
                                  indexMaxValue, fracBitsPt, fracBitsPltRunType);
        if (rdCost < minRdCost)   // update minState ( minRdCost )
        {
          minRdCost         = rdCost;
          minState          = stateID;
          bestRunType       = runType;
          bestRunIndex      = runIndex;
          bestPrevCodedType = prevCodedRunType;
          bestPrevCodedPos  = prevCodedPos;
        }
      }
      // Update trellis info of current state
      m_stateCostRDOQ[1 - refId][curState]   = minRdCost;
      m_prevRunTypeRDOQ[1 - refId][curState] = bestPrevCodedType;
      m_prevRunPosRDOQ[1 - refId][curState]  = bestPrevCodedPos;
      m_statePtRDOQ[curState][currRasterPos] = minState;
      int buffer2update                      = std::min(buffersize, curPos);
      memcpy(m_indexMapRDOQ[1 - refId][curState], m_indexMapRDOQ[refId][minState], sizeof(uint8_t) * buffer2update);
      memcpy(m_runMapRDOQ[1 - refId][curState], m_runMapRDOQ[refId][minState], sizeof(bool) * buffer2update);
      m_indexMapRDOQ[1 - refId][curState][currScanPos] = bestRunIndex;
      m_runMapRDOQ[1 - refId][curState][currScanPos]   = bestRunType;
    }

    if (useRotate)   // early terminate: Rd cost >= min cost in horizontal scan
    {
      if ((m_stateCostRDOQ[1 - refId][0] >= minCost) && (m_stateCostRDOQ[1 - refId][1] >= minCost) &&
          (m_stateCostRDOQ[1 - refId][2] >= minCost))
      {
        return 0;
      }
    }
    refId = 1 - refId;
  }
  return 1;
}

double IntraSearch::xRateDistOptPLT(bool runType, uint8_t runIndex, bool prevRunType, uint8_t prevRunIndex,
                                    uint8_t aboveRunIndex, bool &prevCodedRunType, int &prevCodedPos, int scanPos,
                                    uint32_t width, int dist, int indexMaxValue, const BinFracBits *IndexfracBits,
                                    const BinFracBits &TypefracBits)
{
  double rdCost       = 0.0;
  bool   identityFlag = !((runType != prevRunType) || ((runType == PLT_RUN_INDEX) && (runIndex != prevRunIndex)));

  if ((!identityFlag && runType == PLT_RUN_INDEX) || scanPos == 0)   // encode index value
  {
    uint8_t refIndex = (prevRunType == PLT_RUN_INDEX) ? prevRunIndex : aboveRunIndex;
    refIndex         = (scanPos == 0) ? (indexMaxValue + 1) : refIndex;
    if (runIndex == refIndex)
    {
      rdCost = MAX_DOUBLE;
      return rdCost;
    }
    rdCost += m_pcRdCost->getLambda() *
      (xGetTruncBinBits((runIndex > refIndex) ? runIndex - 1 : runIndex,
                        (scanPos == 0) ? (indexMaxValue + 1) : indexMaxValue)
       << SCALE_BITS);
  }
  rdCost += m_indexError[runIndex][m_scanOrder[scanPos].idx] * (1 << SCALE_BITS);
  if (scanPos > 0)
  {
    rdCost += m_pcRdCost->getLambda() *
      (identityFlag ? (IndexfracBits[(dist < RUN_IDX_THRE) ? dist : RUN_IDX_THRE].intBits[1])
                    : (IndexfracBits[(dist < RUN_IDX_THRE) ? dist : RUN_IDX_THRE].intBits[0]));
  }
  if (!identityFlag && scanPos >= width && prevRunType != PLT_RUN_COPY)
  {
    rdCost += m_pcRdCost->getLambda() * TypefracBits.intBits[runType];
  }
  if (!identityFlag || scanPos == 0)
  {
    prevCodedRunType = runType;
    prevCodedPos     = scanPos;
  }
  return rdCost;
}

uint32_t IntraSearch::getEpExGolombNumBins(uint32_t symbol, uint32_t count)
{
  uint32_t numBins = 0;
  while (symbol >= (uint32_t)(1 << count))
  {
    numBins++;
    symbol -= 1 << count;
    count++;
  }
  numBins++;
  numBins += count;
  assert(numBins <= 32);
  return numBins;
}

uint32_t IntraSearch::xGetTruncBinBits(const uint32_t symbol, const uint32_t numSymbols)
{
  CHECKD(symbol >= numSymbols, "symbol must be less than numSymbols");

  const uint32_t thresh = floorLog2(numSymbols);

  const uint32_t val = 1 << thresh;

  const uint32_t b = numSymbols - val;

  return symbol < val - b ? thresh : thresh + 1;
}

void IntraSearch::xCalcPixelPred(CodingStructure &cs, Partitioner &partitioner, uint32_t yPos, uint32_t xPos,
                                 CompID compBegin, uint32_t numComp)
{
  CodingUnit    &cu       = *cs.getCU(partitioner.chType);
  TransformUnit &tu       = *cs.getTU(partitioner.chType);
  bool           lossless = (m_encCfg->m_costMode == COST_LOSSLESS_CODING && cs.slice->m_isLossless);

  CPelBuf orgBuf[3];
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    CompArea area = cu.blocks[comp];
    if (m_encCfg->m_lmcsEnabled && (cs.slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag))
    {
      orgBuf[comp] = cs.getPredBuf(area);
    }
    else
    {
      orgBuf[comp] = cs.getOrgBuf(area);
    }
  }

  int qp[3];
  int qpRem[3];
  int qpPer[3];
  int quantiserScale[3];
  int quantiserRightShift[3];
  int rightShiftOffset[3];
  int invquantiserRightShift[3];
  int add[3];
  if (!lossless)
  {
    for (uint32_t ch = compBegin; ch < (compBegin + numComp); ch++)
    {
      QpParam cQP(tu, CompID(ch));
      qp[ch]                     = cQP.Qp(true);
      qpRem[ch]                  = qp[ch] % 6;
      qpPer[ch]                  = qp[ch] / 6;
      quantiserScale[ch]         = g_quantScales[0][qpRem[ch]];
      quantiserRightShift[ch]    = QUANT_SHIFT + qpPer[ch];
      rightShiftOffset[ch]       = 1 << (quantiserRightShift[ch] - 1);
      invquantiserRightShift[ch] = IQUANT_SHIFT;
      add[ch]                    = 1 << (invquantiserRightShift[ch] - 1);
    }
  }

  uint32_t scaleX = getComponentScaleX(COMP_Cb, cs.sps->m_chromaFormatIdc);
  uint32_t scaleY = getComponentScaleY(COMP_Cb, cs.sps->m_chromaFormatIdc);
  for (uint32_t ch = compBegin; ch < (compBegin + numComp); ch++)
  {
    const int    channelBitDepth = cu.cs->sps->m_bitDepths[toChannelType((CompID)ch)];
    CompArea     area            = cu.blocks[ch];
    PelBuf       recBuf          = cs.getRecoBuf(area);
    PLTescapeBuf escapeValue     = tu.getescapeValue((CompID)ch);
    if (compBegin != COMP_Y || ch == 0)
    {
      if (lossless)
      {
        escapeValue.at(xPos, yPos) = orgBuf[ch].at(xPos, yPos);
        recBuf.at(xPos, yPos)      = orgBuf[ch].at(xPos, yPos);
      }
      else
      {
        escapeValue.at(xPos, yPos) = std::max<TCoeff>(
          0, ((orgBuf[ch].at(xPos, yPos) * quantiserScale[ch] + rightShiftOffset[ch]) >> quantiserRightShift[ch]));
        assert(escapeValue.at(xPos, yPos) < (TCoeff(1) << (channelBitDepth + 1)));
        TCoeff value = (((escapeValue.at(xPos, yPos) * g_invQuantScales[0][qpRem[ch]]) << qpPer[ch]) + add[ch]) >>
          invquantiserRightShift[ch];
        recBuf.at(xPos, yPos) = Pel(ClipBD<TCoeff>(value, channelBitDepth));   // to be checked
      }
    }
    else if (compBegin == COMP_Y && ch > 0 && yPos % (1 << scaleY) == 0 && xPos % (1 << scaleX) == 0)
    {
      uint32_t yPosC = yPos >> scaleY;
      uint32_t xPosC = xPos >> scaleX;
      if (lossless)
      {
        escapeValue.at(xPosC, yPosC) = orgBuf[ch].at(xPosC, yPosC);
        recBuf.at(xPosC, yPosC)      = orgBuf[ch].at(xPosC, yPosC);
      }
      else
      {
        escapeValue.at(xPosC, yPosC) = std::max<TCoeff>(
          0, ((orgBuf[ch].at(xPosC, yPosC) * quantiserScale[ch] + rightShiftOffset[ch]) >> quantiserRightShift[ch]));
        assert(escapeValue.at(xPosC, yPosC) < (TCoeff(1) << (channelBitDepth + 1)));
        TCoeff value = (((escapeValue.at(xPosC, yPosC) * g_invQuantScales[0][qpRem[ch]]) << qpPer[ch]) + add[ch]) >>
          invquantiserRightShift[ch];
        recBuf.at(xPosC, yPosC) = Pel(ClipBD<TCoeff>(value, channelBitDepth));   // to be checked
      }
    }
  }
}

void IntraSearch::xDerivePLTLossy(CodingStructure &cs, Partitioner &partitioner, CompID compBegin, uint32_t numComp)
{
  CodingUnit &cu               = *cs.getCU(partitioner.chType);
  const int   channelBitDepthL = cs.sps->m_bitDepths[ChannelType::LUMA];
  const int   channelBitDepthC = cs.sps->m_bitDepths[ChannelType::CHROMA];
  bool        lossless         = (m_encCfg->m_costMode == COST_LOSSLESS_CODING && cs.slice->m_isLossless);
  int         pcmShiftRightL   = (channelBitDepthL - PLT_ENCBITDEPTH);
  int         pcmShiftRightC   = (channelBitDepthC - PLT_ENCBITDEPTH);
  if (lossless)
  {
    pcmShiftRightL = 0;
    pcmShiftRightC = 0;
  }

  int maxPltSize = CS::isDualITree(cs) ? MAXPLTSIZE_DUALTREE : MAXPLTSIZE;

  uint32_t height = cu.block(compBegin).height;
  uint32_t width  = cu.block(compBegin).width;

  CPelBuf orgBuf[3];
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    CompArea area = cu.blocks[comp];
    if (m_encCfg->m_lmcsEnabled && (cs.slice->m_lmcsEnabledFlag && m_pcReshape->m_ctuFlag))
    {
      orgBuf[comp] = cs.getPredBuf(area);
    }
    else
    {
      orgBuf[comp] = cs.getOrgBuf(area);
    }
  }

  TransformUnit &tu = *cs.getTU(partitioner.chType);
  QpParam        cQP(tu, compBegin);
  int            qp = cQP.Qp(true) - 6 * (channelBitDepthL - 8);
  qp                = (qp < 0) ? 0 : ((qp > 56) ? 56 : qp);
  int errorLimit    = g_paletteQuant[qp];

  if (lossless)
  {
    errorLimit = 0;
  }
  uint32_t        totalSize = height * width;
  SortingElement *pelList   = new SortingElement[totalSize];
  SortingElement  element;
  SortingElement *pelListSort = new SortingElement[MAXPLTSIZE + 1];
  uint32_t        dictMaxSize = maxPltSize;
  uint32_t        idx         = 0;
  int             last        = -1;

  uint32_t scaleX = getComponentScaleX(COMP_Cb, cs.sps->m_chromaFormatIdc);
  uint32_t scaleY = getComponentScaleY(COMP_Cb, cs.sps->m_chromaFormatIdc);
  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      uint32_t org[3], pX, pY;
      for (int comp = compBegin; comp < (compBegin + numComp); comp++)
      {
        pX        = (comp > 0 && compBegin == COMP_Y) ? (x >> scaleX) : x;
        pY        = (comp > 0 && compBegin == COMP_Y) ? (y >> scaleY) : y;
        org[comp] = orgBuf[comp].at(pX, pY);
      }
      element.setAll(org, compBegin, numComp);

      CompID tmpCompBegin = compBegin;
      int    tmpNumComp   = numComp;
      if (cs.sps->m_chromaFormatIdc != ChromaFormat::_444 && numComp == 3 &&
          (x != ((x >> scaleX) << scaleX) || (y != ((y >> scaleY) << scaleY))))
      {
        tmpCompBegin = COMP_Y;
        tmpNumComp   = 1;
      }
      int besti   = last,
          bestSAD = (last == -1)
        ? MAX_UINT
        : pelList[last].getSAD(element, cs.sps->m_bitDepths, tmpCompBegin, tmpNumComp, lossless);
      if (lossless)
      {
        if (bestSAD)
        {
          for (int i = idx - 1; i >= 0; i--)
          {
            uint32_t sad = pelList[i].getSAD(element, cs.sps->m_bitDepths, tmpCompBegin, tmpNumComp, lossless);
            if (sad == 0)
            {
              bestSAD = sad;
              besti   = i;
              break;
            }
          }
        }
      }
      else
      {
        if (bestSAD)
        {
          for (int i = idx - 1; i >= 0; i--)
          {
            uint32_t sad = pelList[i].getSAD(element, cs.sps->m_bitDepths, tmpCompBegin, tmpNumComp, lossless);
            if (sad < bestSAD)
            {
              bestSAD = sad;
              besti   = i;
              if (!sad)
              {
                break;
              }
            }
          }
        }
      }
      if (besti >= 0 &&
          pelList[besti].almostEqualData(element, errorLimit, cs.sps->m_bitDepths, tmpCompBegin, tmpNumComp, lossless))
      {
        pelList[besti].addElement(element, tmpCompBegin, tmpNumComp);
        last = besti;
      }
      else
      {
        pelList[idx].copyDataFrom(element, tmpCompBegin, tmpNumComp);
        for (int comp = tmpCompBegin; comp < (tmpCompBegin + tmpNumComp); comp++)
        {
          pelList[idx].setCnt(1, comp);
        }
        last = idx;
        idx++;
      }
    }
  }

  if (cs.sps->m_chromaFormatIdc != ChromaFormat::_444 && numComp == 3)
  {
    for (int i = 0; i < idx; i++)
    {
      pelList[i].setCnt(pelList[i].getCnt(COMP_Y) + (pelList[i].getCnt(COMP_Cb) >> 2), MAX_NUM_COMP);
    }
  }
  else
  {
    if (compBegin == 0)
    {
      for (int i = 0; i < idx; i++)
      {
        pelList[i].setCnt(pelList[i].getCnt(COMP_Y), COMP_Cb);
        pelList[i].setCnt(pelList[i].getCnt(COMP_Y), COMP_Cr);
        pelList[i].setCnt(pelList[i].getCnt(COMP_Y), MAX_NUM_COMP);
      }
    }
    else
    {
      for (int i = 0; i < idx; i++)
      {
        pelList[i].setCnt(pelList[i].getCnt(COMP_Cb), COMP_Y);
        pelList[i].setCnt(pelList[i].getCnt(COMP_Cb), MAX_NUM_COMP);
      }
    }
  }

  for (int i = 0; i < dictMaxSize; i++)
  {
    pelListSort[i].setCnt(0, COMP_Y);
    pelListSort[i].setCnt(0, COMP_Cb);
    pelListSort[i].setCnt(0, COMP_Cr);
    pelListSort[i].setCnt(0, MAX_NUM_COMP);
    pelListSort[i].resetAll(compBegin, numComp);
  }

  // bubble sorting
  dictMaxSize = 1;
  for (int i = 0; i < idx; i++)
  {
    if (pelList[i].getCnt(MAX_NUM_COMP) > pelListSort[dictMaxSize - 1].getCnt(MAX_NUM_COMP))
    {
      int j;
      for (j = dictMaxSize; j > 0; j--)
      {
        if (pelList[i].getCnt(MAX_NUM_COMP) > pelListSort[j - 1].getCnt(MAX_NUM_COMP))
        {
          pelListSort[j].copyAllFrom(pelListSort[j - 1], compBegin, numComp);
          dictMaxSize = std::min(dictMaxSize + 1, (uint32_t)maxPltSize);
        }
        else
        {
          break;
        }
      }
      pelListSort[j].copyAllFrom(pelList[i], compBegin, numComp);
    }
  }

  uint32_t paletteSize  = 0;
  uint64_t numColorBits = 0;
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    numColorBits += (comp > 0) ? channelBitDepthC : channelBitDepthL;
  }
  const int pltLambdaShift            = (compBegin > 0) ? pcmShiftRightC : pcmShiftRightL;
  double    bitCost                   = m_pcRdCost->getLambda() / (double)(1 << (2 * pltLambdaShift)) * numColorBits;
  bool      reuseflag[MAXPLTPREDSIZE] = { false };
  int       run;
  double    reuseflagCost;
  for (int i = 0; i < maxPltSize; i++)
  {
    if (pelListSort[i].getCnt(MAX_NUM_COMP))
    {
      CompID tmpCompBegin = compBegin;
      int    tmpNumComp   = numComp;
      if (cs.sps->m_chromaFormatIdc != ChromaFormat::_444 && numComp == 3 && pelListSort[i].getCnt(COMP_Cb) == 0)
      {
        tmpCompBegin = COMP_Y;
        tmpNumComp   = 1;
      }

      for (int comp = tmpCompBegin; comp < (tmpCompBegin + tmpNumComp); comp++)
      {
        int half                     = pelListSort[i].getCnt(comp) >> 1;
        cu.curPLT[comp][paletteSize] = (pelListSort[i].getSumData(comp) + half) / pelListSort[i].getCnt(comp);
      }

      int best = -1;
      if (errorLimit)
      {
        double pal[MAX_NUM_COMP], err = 0.0, bestCost = 0.0;
        for (int comp = tmpCompBegin; comp < (tmpCompBegin + tmpNumComp); comp++)
        {
          pal[comp] = pelListSort[i].getSumData(comp) / (double)pelListSort[i].getCnt(comp);
          err       = pal[comp] - cu.curPLT[comp][paletteSize];
          if (isChroma((CompID)comp))
          {
            bestCost += (err * err * PLT_CHROMA_WEIGHTING) / (1 << (2 * pcmShiftRightC)) * pelListSort[i].getCnt(comp);
          }
          else
          {
            bestCost += (err * err) / (1 << (2 * pcmShiftRightL)) * pelListSort[i].getCnt(comp);
          }
        }
        bestCost += bitCost;

        for (int t = 0; t < cs.prevPLT.curPLTSize[compBegin]; t++)
        {
          double cost = 0.0;
          for (int comp = tmpCompBegin; comp < (tmpCompBegin + tmpNumComp); comp++)
          {
            err = pal[comp] - cs.prevPLT.curPLT[comp][t];
            if (isChroma((CompID)comp))
            {
              cost += (err * err * PLT_CHROMA_WEIGHTING) / (1 << (2 * pcmShiftRightC)) * pelListSort[i].getCnt(comp);
            }
            else
            {
              cost += (err * err) / (1 << (2 * pcmShiftRightL)) * pelListSort[i].getCnt(comp);
            }
          }
          run = 0;
          for (int t2 = t; t2 >= 0; t2--)
          {
            if (!reuseflag[t2])
            {
              run++;
            }
            else
            {
              break;
            }
          }
          reuseflagCost = m_pcRdCost->getLambda() / (double)(1 << (2 * pltLambdaShift)) *
            getEpExGolombNumBins(run ? run + 1 : run, 0);
          cost += reuseflagCost;

          if (cost < bestCost)
          {
            best     = t;
            bestCost = cost;
          }
        }
        if (best != -1)
        {
          for (int comp = tmpCompBegin; comp < (tmpCompBegin + tmpNumComp); comp++)
          {
            cu.curPLT[comp][paletteSize] = cs.prevPLT.curPLT[comp][best];
          }
          reuseflag[best] = true;
        }
      }

      bool duplicate = false;
      if (pelListSort[i].getCnt(MAX_NUM_COMP) == 1 && best == -1)
      {
        duplicate = true;
      }
      else
      {
        for (int t = 0; t < paletteSize; t++)
        {
          bool duplicateTmp = true;
          for (int comp = tmpCompBegin; comp < (tmpCompBegin + tmpNumComp); comp++)
          {
            duplicateTmp = duplicateTmp && (cu.curPLT[comp][paletteSize] == cu.curPLT[comp][t]);
          }
          if (duplicateTmp)
          {
            duplicate = true;
            break;
          }
        }
      }
      if (!duplicate)
      {
        if (cs.sps->m_chromaFormatIdc != ChromaFormat::_444 && numComp == 3 && pelListSort[i].getCnt(COMP_Cb) == 0)
        {
          if (best != -1)
          {
            cu.curPLT[COMP_Cb][paletteSize] = cs.prevPLT.curPLT[COMP_Cb][best];
            cu.curPLT[COMP_Cr][paletteSize] = cs.prevPLT.curPLT[COMP_Cr][best];
          }
          else
          {
            cu.curPLT[COMP_Cb][paletteSize] = 1 << (channelBitDepthC - 1);
            cu.curPLT[COMP_Cr][paletteSize] = 1 << (channelBitDepthC - 1);
          }
        }
        paletteSize++;
      }
    }
    else
    {
      break;
    }
  }
  cu.curPLTSize[compBegin] = paletteSize;

  delete[] pelList;
  delete[] pelListSort;
}
// -------------------------------------------------------------------------------------------------------------------
// Intra search
// -------------------------------------------------------------------------------------------------------------------

void IntraSearch::xEncIntraHeader(CodingStructure &cs, Partitioner &partitioner, const bool &hasLuma,
                                  const bool &hasChroma, const CUCtxIntra *cuCtxIntra)
{
  CodingUnit &cu = *cs.getCU(partitioner.chType);

  if (hasLuma)
  {
    bool isFirst = partitioner.currArea().lumaPos() == cs.area.lumaPos();

    // CU header
    if (isFirst)
    {
      if ((!cs.slice->isIntra() || cs.slice->m_ibcFlag || cs.slice->m_sps->m_PLTMode) && cu.Y().valid())
      {
        m_CABACEstimator->cu_skip_flag(cu);
        m_CABACEstimator->pred_mode(cu);
      }
      if (CU::isPLT(cu))
      {
        return;
      }
    }

    CodingUnit &cu = *cs.getCU(partitioner.currArea().lumaPos(), partitioner.chType);

    // luma prediction mode
    if (isFirst)
    {
      if (!cu.Y().valid())
      {
        m_CABACEstimator->pred_mode(cu);
      }
      m_CABACEstimator->bdpcm_mode(cu, COMP_Y);
      m_CABACEstimator->intra_luma_pred_mode(cu, *cuCtxIntra);
    }
  }

  if (hasChroma)
  {
    bool isFirst = partitioner.currArea().Cb().valid() && partitioner.currArea().chromaPos() == cs.area.chromaPos();

    CodingUnit &cu = *cs.getCU(partitioner.currArea().chromaPos(), ChannelType::CHROMA);

    if (isFirst)
    {
      m_CABACEstimator->bdpcm_mode(cu, CompID(ChannelType::CHROMA));
      m_CABACEstimator->intra_chroma_pred_mode(cu);
    }
  }
}

void IntraSearch::xEncSubdivCbfQT(CodingStructure &cs, Partitioner &partitioner, const bool &hasLuma,
                                  const bool &hasChroma)
{
  const UnitArea &currArea  = partitioner.currArea();
  TransformUnit  &currTU    = *cs.getTU(currArea.block(partitioner.chType), partitioner.chType);
  CodingUnit     &currCU    = *currTU.cu;
  uint32_t        currDepth = partitioner.currTrDepth;

  const bool subdiv = currTU.depth > currDepth;
  CompID     compID = isLuma(partitioner.chType) ? COMP_Y : COMP_Cb;

  if (partitioner.canSplit(TU_MAX_TR_SPLIT, cs))
  {
    CHECK(!subdiv, "TU split implied");
  }
  else
  {
    CHECK(subdiv && isLuma(compID), "No TU subdivision is allowed with QTBT");
  }

  if (hasChroma)
  {
    const uint32_t numberValidComponents = getNumberValidComponents(currArea.chromaFormat);

    for (uint32_t ch = COMP_Cb; ch < numberValidComponents; ch++)
    {
      const CompID compID = CompID(ch);

      if (currDepth == 0 || TU::getCbfAtDepth(currTU, compID, currDepth - 1))
      {
        const bool prevCbf = (compID == COMP_Cr ? TU::getCbfAtDepth(currTU, COMP_Cb, currDepth) : false);
        m_CABACEstimator->cbf_comp(TU::getCbfAtDepth(currTU, compID, currDepth), currArea.blocks[compID], currDepth,
                                   prevCbf, currCU.getBdpcmMode(compID));
      }
    }
  }

  if (subdiv)
  {
    if (partitioner.canSplit(TU_MAX_TR_SPLIT, cs))
    {
      partitioner.splitCurrArea(TU_MAX_TR_SPLIT, cs);
    }
    else
    {
      THROW("Cannot perform an implicit split!");
    }

    do
    {
      xEncSubdivCbfQT(cs, partitioner, hasLuma, hasChroma);
    } while (partitioner.nextPart(cs));

    partitioner.exitCurrSplit();
  }
  else
  {
    //===== Cbfs =====
    if (hasLuma)
    {
      m_CABACEstimator->cbf_comp(TU::getCbfAtDepth(currTU, COMP_Y, currDepth), currTU.Y(), currTU.depth, false,
                                 currCU.getBdpcmMode(COMP_Y));
    }
  }
}

void IntraSearch::xEncCoeffQT(CodingStructure &cs, Partitioner &partitioner, const CompID compID, CUCtx *cuCtx)
{
  const UnitArea &currArea = partitioner.currArea();

  TransformUnit &currTU    = *cs.getTU(currArea.block(partitioner.chType), partitioner.chType);
  uint32_t       currDepth = partitioner.currTrDepth;
  const bool     subdiv    = currTU.depth > currDepth;

  if (subdiv)
  {
    if (partitioner.canSplit(TU_MAX_TR_SPLIT, cs))
    {
      partitioner.splitCurrArea(TU_MAX_TR_SPLIT, cs);
    }
    else
    {
      THROW("Implicit TU split not available!");
    }

    do
    {
      xEncCoeffQT(cs, partitioner, compID, cuCtx);
    } while (partitioner.nextPart(cs));

    partitioner.exitCurrSplit();
  }
  else
  {
    if (currArea.blocks[compID].valid())
    {
      if (compID == COMP_Cr)
      {
        const int cbfMask =
          (TU::getCbf(currTU, COMP_Cb) ? CBF_MASK_CB : 0) + (TU::getCbf(currTU, COMP_Cr) ? CBF_MASK_CR : 0);
        m_CABACEstimator->joint_cb_cr(currTU, cbfMask);
      }
      if (TU::getCbf(currTU, compID))
      {
        if (isLuma(compID))
        {
          m_CABACEstimator->residual_coding_last(currTU, compID, cuCtx);
          m_CABACEstimator->residual_coding_coef(currTU, compID, cuCtx);
          m_CABACEstimator->residual_coding_sign(currTU, compID);
          m_CABACEstimator->nst_idx(currTU, *cuCtx);
          m_CABACEstimator->mts_idx(currTU, *cuCtx);
        }
        else
        {
          m_CABACEstimator->residual_coding_last(currTU, compID);
          m_CABACEstimator->residual_coding_coef(currTU, compID);
          m_CABACEstimator->residual_coding_sign(currTU, compID);
        }
      }
    }
  }
}

uint64_t IntraSearch::xGetIntraFracBitsQT(CodingStructure &cs, Partitioner &partitioner, const bool &hasLuma,
                                          const bool &hasChroma, CUCtx *cuCtx, const CUCtxIntra *cuCtxIntra)
{
  PROFILER_SCOPE(0, g_timeProfiler, P_INTRA_FRAC_BITS);
  m_CABACEstimator->resetBits();

  xEncIntraHeader(cs, partitioner, hasLuma, hasChroma, cuCtxIntra);
  xEncSubdivCbfQT(cs, partitioner, hasLuma, hasChroma);

  if (hasLuma)
  {
    xEncCoeffQT(cs, partitioner, COMP_Y, cuCtx);
  }
  if (hasChroma)
  {
    xEncCoeffQT(cs, partitioner, COMP_Cb);
    xEncCoeffQT(cs, partitioner, COMP_Cr);
  }

  uint64_t fracBits = m_CABACEstimator->getEstFracBits();
  return fracBits;
}

uint64_t IntraSearch::xGetIntraFracBitsQTChroma(TransformUnit &currTU, const CompID &compID, CUCtx *cuCtx)
{
  m_CABACEstimator->resetBits();
  // Include Cbf and jointCbCr flags here as we make decisions across components

  if (currTU.jointCbCr)
  {
    const bool cbfMaskCb = TU::getCbf(currTU, COMP_Cb);
    const bool cbfMaskCr = TU::getCbf(currTU, COMP_Cr);
    const int  cbfMask   = (cbfMaskCb ? CBF_MASK_CB : 0) + (cbfMaskCr ? CBF_MASK_CR : 0);

    m_CABACEstimator->cbf_comp(cbfMaskCb, currTU.blocks[COMP_Cb], currTU.depth, false,
                               currTU.cu->getBdpcmMode(COMP_Cb));
    m_CABACEstimator->cbf_comp(cbfMaskCr, currTU.blocks[COMP_Cr], currTU.depth, cbfMaskCb,
                               currTU.cu->getBdpcmMode(COMP_Cr));

    if (cbfMask != 0)
    {
      m_CABACEstimator->joint_cb_cr(currTU, cbfMask);
    }
    if (cbfMaskCb)
    {
      m_CABACEstimator->residual_coding_last(currTU, COMP_Cb, cuCtx);
      m_CABACEstimator->residual_coding_coef(currTU, COMP_Cb, cuCtx);
      m_CABACEstimator->residual_coding_sign(currTU, COMP_Cb);
    }
    if (cbfMaskCr)
    {
      m_CABACEstimator->residual_coding_last(currTU, COMP_Cr, cuCtx);
      m_CABACEstimator->residual_coding_coef(currTU, COMP_Cr, cuCtx);
      m_CABACEstimator->residual_coding_sign(currTU, COMP_Cr);
    }
  }
  else
  {
    if (compID == COMP_Cb)
    {
      m_CABACEstimator->cbf_comp(TU::getCbf(currTU, compID), currTU.blocks[compID], currTU.depth, false,
                                 currTU.cu->getBdpcmMode(compID));
    }
    else
    {
      const bool cbCbf   = TU::getCbf(currTU, COMP_Cb);
      const bool crCbf   = TU::getCbf(currTU, compID);
      const int  cbfMask = (cbCbf ? CBF_MASK_CB : 0) + (crCbf ? CBF_MASK_CR : 0);
      m_CABACEstimator->cbf_comp(crCbf, currTU.blocks[compID], currTU.depth, cbCbf, currTU.cu->getBdpcmMode(compID));
      m_CABACEstimator->joint_cb_cr(currTU, cbfMask);
    }
  }

  if (!currTU.jointCbCr && TU::getCbf(currTU, compID))
  {
    m_CABACEstimator->residual_coding_last(currTU, compID, cuCtx);
    m_CABACEstimator->residual_coding_coef(currTU, compID, cuCtx);
    m_CABACEstimator->residual_coding_sign(currTU, compID);
  }

  uint64_t fracBits = m_CABACEstimator->getEstFracBits();
  return fracBits;
}

void IntraSearch::xPredTuLuma(TransformUnit &tu, PelBuf &pred)
{
  const CodingUnit &cu   = *tu.cu;
  const CompArea   &area = tu.blocks[COMP_Y];
  if (!PU::isDIMD(cu) && !PU::isTIMD(cu) && !PU::isOBIC(cu))
  {
    initIntraPatternChType(cu, area);
  }
  if (PU::isMIP(cu, ChannelType::LUMA))
  {
    initIntraMip(cu, area);
    predIntraMip(COMP_Y, pred, cu);
  }
  else if (PU::isDIMD(cu))
  {
    CodingUnit &cuNonConst = *tu.cu;
    predIntraDimd(pred, cuNonConst, area);
  }
  else if (PU::isTIMD(cu))
  {
    CodingUnit &cuNonConst = *tu.cu;
    const auto  timdMode   = (cu.timdSadFlag) ? IntraPrediction::TimdMode::SAD : IntraPrediction::TimdMode::Normal;
    predIntraTimd(pred, cuNonConst, area, true, timdMode, false);
  }
  else if (PU::isOBIC(cu))
  {
    CodingUnit &cuNonConst = *tu.cu;
    predIntraObic(pred, cuNonConst, area, true);
  }
  else if (PU::isSgpm(cu))
  {
    CodingUnit &cuNonConst = *tu.cu;
    predIntraSGPM(pred, cuNonConst, area, true);
  }
  else
  {
    predIntraAng(COMP_Y, pred, cu, true, false);
  }
}

void IntraSearch::xPredTuChroma(TransformUnit &tu, PelBuf &predCb, PelBuf &predCr, IModeTrCandChroma &modeCand)
{
  CodingUnit     &cu     = *tu.cu;
  const CompArea &areaCb = tu.blocks[COMP_Cb];
  const CompArea &areaCr = tu.blocks[COMP_Cr];
  const int       prdMode =
    cu.getBdpcmMode(COMP_Cb) != BdpcmMode::NONE ? BDPCM_IDX : PU::getFinalIntraMode(cu, ChannelType::CHROMA);

  initIntraPatternChType(cu, areaCb);
  initIntraPatternChType(cu, areaCr);

  if (PU::isLMCMode(prdMode))
  {
    if (cu.idxNonLocalCCP && cu.ccMergeFusionIdx)
    {
      setAndPredCcMergeFusionCand(cu, modeCand.chromaMode.ccModels, modeCand.chromaMode.ccModels1, predCb, predCr);
    }
    else if (cu.decDerivedCcpMode)
    {
      setAndPredCcMergeFusionCand(cu, modeCand.chromaMode.ccModels, modeCand.chromaMode.ccModels1, predCb, predCr);
    }
    else if (cu.cccmFlag)
    {
      predIntraCCCM(cu, predCb, predCr, false);
    }
    else
    {
      predIntraChromaLM(COMP_Cb, predCb, cu, areaCb, prdMode, false);
      predIntraChromaLM(COMP_Cr, predCr, cu, areaCr, prdMode, false);
    }
  }
  else if (PU::isMIP(cu, ChannelType::CHROMA))
  {
    initIntraMip(cu, areaCb);
    predIntraMip(COMP_Cb, predCb, cu);

    initIntraMip(cu, areaCr);
    predIntraMip(COMP_Cr, predCr, cu);
  }
  else
  {
    predIntraAng(COMP_Cb, predCb, cu, true, false);
    predIntraAng(COMP_Cr, predCr, cu, true, false);
  }
}

bool IntraSearch::xIntraCodingTUBlockLuma(TransformUnit &tu, Distortion &dist, PreTrListLuma &ptList, TCoeff &absSum)
{
  CodingStructure &cs       = *tu.cs;
  const CompArea  &area     = tu.blocks[COMP_Y];
  const SPS       &sps      = *cs.sps;
  const PPS       &pps      = *cs.pps;
  const Slice     &slice    = *cs.slice;
  const int        bitDepth = sps.m_bitDepths[ChannelType::LUMA];
  PelBuf           org      = cs.getOrgBuf(area);
  PelBuf           rec      = cs.getRecoBuf(area);
  PelBuf           prd(ptList.prd, area);

  //===== copy pre-calculated data signal =====
  cs.getPredBuf(area).copyFrom(prd);
  if (KEEP_PRED_AND_RESI_SIGNALS)
  {
    cs.getOrgResiBuf(area).copyFrom(PelBuf(ptList.res, area));
  }

  DTRACE(g_trace_ctx, D_PRED, "@(%4d,%4d) [%2dx%2d] IMode=%d\n", tu.lx(), tu.ly(), tu.lwidth(), tu.lheight(),
         PU::getFinalIntraMode(*tu.cu, ChannelType::LUMA));

  //===== set lambda =====
  m_pcRdCost->setChromaFormat(cs.sps->m_chromaFormatIdc);
#if RDOQ_CHROMA_LAMBDA
  m_pcTrQuant->selectLambda(COMP_Y);
#endif

  //===== quantization =====
  const QpParam qp(tu, COMP_Y);
  m_pcTrQuant->quantNxN(tu, COMP_Y, qp, absSum, m_CABACEstimator->getCtx(), ptList.trCoeffs);
  DTRACE(g_trace_ctx, D_TU_ABS_SUM, "%d: comp=%d, abssum=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_TU_ABS_SUM), COMP_Y,
         absSum);

  //===== check MTS =====
  if (isMTS(tu.mtsIdx[COMP_Y]) || isNST(tu.mtsIdx[COMP_Y]))
  {
    auto tclevels = tu.getCoeffs(COMP_Y);
    if (isMTS(tu.mtsIdx[COMP_Y]))
    {
      int nCands = (absSum > MTS_TH_COEFF[1]) ? MTS_NCANDS[2]
        : (absSum > MTS_TH_COEFF[0])          ? MTS_NCANDS[1]
                                              : MTS_NCANDS[0];
      if ((tu.mtsIdx[COMP_Y] - MtsType::MTS_1) >= nCands)
      {
        return false;
      }
    }
    TCoeff sumAbsAC = absSum - abs(tclevels.buf[0]);
    if (sumAbsAC == 0)
    {
      return false;
    }
  }

  ClpRng clpRng = cs.slice->setNewClipRange(true, &m_pcReshape->m_fwdLUT, COMP_Y);

  //===== reconstruction (incl. inverse transform) =====
  if (absSum > 0)
  {
    bool hasSignPred = m_pcTrQuant->prdCoeffSigns(tu, COMP_Y);

    PelBuf res = cs.getResiBuf(area);
    m_pcTrQuant->invTransformNxN(tu, COMP_Y, res, qp, hasSignPred);
    rec.reconstruct(prd, res, clpRng);
  }
  else
  {
    if (KEEP_PRED_AND_RESI_SIGNALS)
    {
      cs.getResiBuf(area).fill(0);
    }
    rec.copyClip(prd, clpRng);
  }

  //===== Bif parameters =====
  CompArea tmpArea1(COMP_Y, area.chromaFormat, Position(0, 0), area.size());
  PelBuf   tmpRecLuma;
  tmpRecLuma = m_tmpStorageCtu.getBuf(tmpArea1);
  tmpRecLuma.copyFrom(rec);

  //===== update distortion =====
#if WCG_EXT
  if (m_encCfg->m_lumaLevelToDeltaQPMapping.isEnabled() ||
      (m_encCfg->m_lmcsEnabled && slice.m_lmcsEnabledFlag && (m_pcReshape->m_ctuFlag)))
  {
    const CPelBuf orgLuma = cs.getOrgBuf(cs.area.blocks[COMP_Y]);
    {
      if (!(m_encCfg->m_lumaLevelToDeltaQPMapping.isEnabled()))
      {
        tmpRecLuma.rspSignal(m_pcReshape->m_invLUT);
      }

      if (pps.m_BIF && m_bilateralFilter->getApplyBIF(tu, COMP_Y))
      {
        CompArea compArea    = tu.blocks[COMP_Y];
        PelBuf   recIPredBuf = cs.slice->m_pic->getRecoBuf(compArea);
        CPelBuf  reco        = cs.getRecoBuf(COMP_Y);
        if (!(m_encCfg->m_lumaLevelToDeltaQPMapping.isEnabled()))
        {
          m_bilateralFilter->bilateralFilterRDOdiamond5x5(COMP_Y, tmpRecLuma, tmpRecLuma, tmpRecLuma, tu.cu->qp,
                                                          recIPredBuf, reco, cs.slice->clpRng(COMP_Y), tu, true, true,
                                                          &m_pcReshape->m_invLUT);
        }
        else
        {
          m_bilateralFilter->bilateralFilterRDOdiamond5x5(COMP_Y, tmpRecLuma, tmpRecLuma, tmpRecLuma, tu.cu->qp,
                                                          recIPredBuf, reco, cs.slice->clpRng(COMP_Y), tu, true);
        }
      }

      dist += m_pcRdCost->getDistPart(org, tmpRecLuma, sps.m_bitDepths[toChannelType(COMP_Y)], COMP_Y, DFunc::SSE_WTD,
                                      &orgLuma);
    }
  }
  else
#endif
  {
    {
      if (pps.m_BIF && m_bilateralFilter->getApplyBIF(tu, COMP_Y))
      {
        CompArea compArea    = tu.blocks[COMP_Y];
        PelBuf   recIPredBuf = cs.slice->m_pic->getRecoBuf(compArea);
        CPelBuf  reco        = cs.getRecoBuf(COMP_Y);
        m_bilateralFilter->bilateralFilterRDOdiamond5x5(COMP_Y, tmpRecLuma, tmpRecLuma, tmpRecLuma, tu.cu->qp,
                                                        recIPredBuf, reco, cs.slice->clpRng(COMP_Y), tu, true);
      }

      dist += m_pcRdCost->getDistPart(org, tmpRecLuma, bitDepth, COMP_Y, DFunc::SSE);
    }
  }
  return true;
}

void IntraSearch::xIntraCodingTUBlockChroma(TransformUnit &tu, const CompID compID, Distortion &dist,
                                            PreTrListChroma &ptList)
{
  if (!tu.blocks[compID].valid())
  {
    return;
  }
  CHECK(tu.jointCbCr && compID != COMP_Cb, "wrong combination of compID and jointCbCr");

  CodingStructure  &cs        = *tu.cs;
  const CompArea   &area      = tu.blocks[compID];
  const SPS        &sps       = *cs.sps;
  const PPS        &pps       = *cs.pps;
  const Slice      &slice     = *cs.slice;
  const ChannelType chType    = toChannelType(compID);
  const int         bitDepth  = sps.m_bitDepths[chType];
  const bool        keepResi  = cs.sps->m_LMChroma || KEEP_PRED_AND_RESI_SIGNALS;
  const int         chromaID  = tu.jointCbCr ? tu.jointCbCr + 1 : compID - COMP_Cb;   // array index for ptlist
  const CompID      codedComp = tu.jointCbCr ? ((tu.jointCbCr & CBF_MASK_CB) != 0 ? COMP_Cb : COMP_Cr) : compID;
  const CompID      otherComp = codedComp == COMP_Cr ? COMP_Cb : COMP_Cr;
  PelBuf            org       = cs.getOrgBuf(area);
  PelBuf            rec       = cs.getRecoBuf(area);
  PelBuf            prd(ptList.prd[compID - COMP_Cb], area);

  //===== copy pre-calculated data signal =====
  if (tu.jointCbCr)
  {
    const CompArea &areaCr = tu.blocks[COMP_Cr];
    cs.getPredBuf(area).copyFrom(PelBuf(ptList.prd[0], area));
    cs.getPredBuf(areaCr).copyFrom(PelBuf(ptList.prd[1], areaCr));
  }
  else
  {
    cs.getPredBuf(area).copyFrom(prd);
  }
  if (KEEP_PRED_AND_RESI_SIGNALS)
  {
    if (tu.jointCbCr)
    {
      const CompArea &areaCC = tu.blocks[codedComp];
      const CompArea &areaOC = tu.blocks[otherComp];
      cs.getOrgResiBuf(areaCC).copyFrom(PelBuf(ptList.res[chromaID], areaCC));
      cs.getOrgResiBuf(areaOC).fill(Pel(0));
    }
    else
    {
      cs.getOrgResiBuf(area).copyFrom(PelBuf(ptList.res[compID - COMP_Cb], area));
    }
  }

  DTRACE(g_trace_ctx, D_PRED, "@(%4d,%4d) [%2dx%2d] IMode=%d\n", tu.lx(), tu.ly(), tu.lwidth(), tu.lheight(),
         PU::getFinalIntraMode(*tu.cu, chType));

  //===== set lambda =====
  m_pcRdCost->setChromaFormat(cs.sps->m_chromaFormatIdc);
#if RDOQ_CHROMA_LAMBDA
  m_pcTrQuant->selectLambda(compID);
#endif
  if (tu.getChromaAdj())
  {
    double cResScale = (double)(1 << CSCALE_FP_PREC) / (double)tu.getChromaAdj();
    m_pcTrQuant->setLambda(m_pcTrQuant->getLambda() / (cResScale * cResScale));
  }
  if (tu.jointCbCr)
  {
    // Lambda is loosened for the joint mode with respect to single modes as the same residual is used for both chroma
    // blocks
    const int    absIct = abs(TU::getICTMode(tu));
    const double lfact  = (absIct == 1 || absIct == 3 ? 0.8 : 0.5);
    m_pcTrQuant->setLambda(lfact * m_pcTrQuant->getLambda());
  }
  if (sps.m_jointCbCrEnabledFlag && (tu.cu->cs->slice->m_iSliceQp > 18))
  {
    m_pcTrQuant->setLambda(1.3 * m_pcTrQuant->getLambda());
  }

  //===== quantization =====
  const QpParam qp(tu, codedComp);
  TCoeff        absSum = 0;

  m_pcTrQuant->quantNxN(tu, codedComp, qp, absSum, m_CABACEstimator->getCtx(), ptList.trCoeffs[chromaID]);

  DTRACE(g_trace_ctx, D_TU_ABS_SUM, "%d: comp=%d, abssum=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_TU_ABS_SUM),
         codedComp, absSum);

  //===== reconstruction (incl. inverse transform) =====
  if (absSum > 0)
  {
    bool hasSignPred = m_pcTrQuant->prdCoeffSigns(tu, compID);

    if (tu.jointCbCr)
    {
      tu.getCoeffs(otherComp).fill(0);
      TU::setCbfAtDepth(tu, otherComp, tu.depth, tu.jointCbCr == 3);

      CompArea &areaCr = tu.blocks[COMP_Cr];
      PelBuf    res    = cs.getResiBuf(area);
      PelBuf    resCr  = cs.getResiBuf(areaCr);
      PelBuf    recCr  = cs.getRecoBuf(areaCr);
      PelBuf    prdCr(ptList.prd[1], areaCr);
      m_pcTrQuant->invTransformNxN(tu, codedComp, (codedComp == COMP_Cb ? res : resCr), qp, hasSignPred);
      m_pcTrQuant->invTransformICT(tu, res, resCr);
      if (tu.getChromaAdj())
      {
        res.scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(COMP_Cb));
        resCr.scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(COMP_Cr));
      }
      rec.reconstruct(prd, res, cs.slice->clpRng(COMP_Cb));
      recCr.reconstruct(prdCr, resCr, cs.slice->clpRng(COMP_Cr));
    }
    else
    {
      PelBuf res = cs.getResiBuf(area);
      m_pcTrQuant->invTransformNxN(tu, compID, res, qp, hasSignPred);
      if (tu.getChromaAdj())
      {
        res.scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(compID));
      }
      rec.reconstruct(prd, res, cs.slice->clpRng(compID));
    }
  }
  else if (tu.jointCbCr)
  {
    dist = std::numeric_limits<Distortion>::max();
    return;
  }
  else
  {
    if (keepResi)
    {
      cs.getResiBuf(area).fill(0);
    }
    rec.copyFrom(prd);
  }

  CompArea tmpArea2(compID, area.chromaFormat, Position(0, 0), area.size());
  PelBuf   tmpRecChroma;
  if (isChroma(compID))
  {
    tmpRecChroma = m_tmpStorageCtu.getBuf(tmpArea2);
    tmpRecChroma.copyFrom(rec);
  }

  //===== update distortion =====
#if WCG_EXT
  if (m_encCfg->m_lumaLevelToDeltaQPMapping.isEnabled() ||
      (m_encCfg->m_lmcsEnabled && slice.m_lmcsEnabledFlag && (m_pcReshape->m_ctuFlag || m_encCfg->m_intraCMD)))
  {
    const CPelBuf orgLuma = cs.getOrgBuf(cs.area.blocks[COMP_Y]);
    if (pps.m_chromaBIF && isChroma(compID) && m_bilateralFilter->getApplyBIF(tu, compID))
    {
      CompArea compArea    = tu.blocks[compID];
      PelBuf   recIPredBuf = cs.slice->m_pic->getRecoBuf(compArea);
      CPelBuf  reco        = cs.getRecoBuf(compID);
      m_bilateralFilter->bilateralFilterRDOdiamond5x5(compID, tmpRecChroma, tmpRecChroma, tmpRecChroma, tu.cu->qp,
                                                      recIPredBuf, reco, cs.slice->clpRng(compID), tu, true);
    }
    dist += m_pcRdCost->getDistPart(org, tmpRecChroma, bitDepth, compID, DFunc::SSE_WTD, &orgLuma);
    if (tu.jointCbCr)
    {
      const CPelBuf orgCr = cs.getOrgBuf(tu.blocks[COMP_Cr]);
      const CPelBuf recCr = cs.getRecoBuf(tu.blocks[COMP_Cr]);
      if (compID == COMP_Cr)
      {
        dist += m_pcRdCost->getDistPart(orgCr, tmpRecChroma, bitDepth, COMP_Cr, DFunc::SSE_WTD, &orgLuma);
      }
      else
      {
        dist += m_pcRdCost->getDistPart(orgCr, recCr, bitDepth, COMP_Cr, DFunc::SSE_WTD, &orgLuma);
      }
    }
  }
  else
#endif
  {
    if (pps.m_chromaBIF && isChroma(compID) && m_bilateralFilter->getApplyBIF(tu, compID))
    {
      CompArea compArea    = tu.blocks[compID];
      PelBuf   recIPredBuf = cs.slice->m_pic->getRecoBuf(compArea);
      CPelBuf  reco        = cs.getRecoBuf(compID);
      m_bilateralFilter->bilateralFilterRDOdiamond5x5(compID, tmpRecChroma, tmpRecChroma, tmpRecChroma, tu.cu->qp,
                                                      recIPredBuf, reco, cs.slice->clpRng(compID), tu, true);
    }
    dist += m_pcRdCost->getDistPart(org, tmpRecChroma, bitDepth, compID, DFunc::SSE);
    if (tu.jointCbCr)
    {
      const CPelBuf orgCr = cs.getOrgBuf(tu.blocks[COMP_Cr]);
      const CPelBuf recCr = cs.getRecoBuf(tu.blocks[COMP_Cr]);
      if (compID == COMP_Cr)
      {
        dist += m_pcRdCost->getDistPart(orgCr, tmpRecChroma, bitDepth, COMP_Cr, DFunc::SSE);
      }
      else
      {
        dist += m_pcRdCost->getDistPart(orgCr, recCr, bitDepth, COMP_Cr, DFunc::SSE);
      }
    }
  }
}

bool IntraSearch::xRecurIntraCodingLumaQT(CodingStructure &cs, Partitioner &partitioner, PreTrListLuma &ptList,
                                          CUCtxIntra &cuCtxIntra)
{
  const UnitArea  &currArea   = partitioner.currArea();
  uint32_t         currDepth  = partitioner.currTrDepth;
  const bool       checkSplit = partitioner.canSplit(TU_MAX_TR_SPLIT, cs);
  const bool       checkFull  = !checkSplit;
  CodingStructure *csSplit    = (checkSplit ? &cs : nullptr);
  CodingStructure *csFull     = (checkFull ? &cs : nullptr);

  double     singleCost     = MAX_DOUBLE;
  Distortion singleDistLuma = 0;
  uint64_t   singleFracBits = 0;

  const TempCtx ctxStart(m_ctxPool, m_CABACEstimator->getCtx());
  TempCtx       ctxBest(m_ctxPool);
  CUCtx         cuCtx;
  cuCtx.isDQPCoded         = true;
  cuCtx.isChromaQpAdjCoded = true;

  if (checkFull)
  {
    PROFILER_SCOPE(1, g_timeProfiler, P_TRAFO_QUANT);

    TransformUnit &tu = csFull->addTU(CS::getArea(*csFull, currArea, partitioner.chType), partitioner.chType);
    tu.depth          = currDepth;
    csFull->cost      = 0.0;

    CHECK(!tu.Y().valid(), "Invalid TU");

    if (!ptList.valid)
    {
      xPreCalcPrdTransLuma(tu, ptList);
      xSelectTrCandLuma(tu, ptList);
      ptList.valid = false;   // reset for next TU inside CU
    }
    CHECK(ptList.trTypes.empty(), "Empty transform list");

    TransformUnit   *bestTU = nullptr;
    CodingStructure &saveCS = *m_pSaveCS[0];

    Distortion singleDistTmpLuma = 0;
    uint64_t   singleTmpFracBits = 0;
    double     singleCostTmp     = 0;
    bool       transTested       = false;
    bool       bestCbf           = true;
    bool       isLstBest         = true;
    MtsType    maxMtsType        = MtsType::MTS_6;
    int        numMTSInvalid     = 0;
    size_t     numTrTypes        = ptList.trTypes.size();

    if (numTrTypes > 1)
    {
      saveCS.pcv     = cs.pcv;
      saveCS.picture = cs.picture;
      saveCS.compactResize(cs.area);
      saveCS.clearTUs();
      bestTU = &saveCS.addTU(currArea, partitioner.chType);
    }

    for (size_t trId = 0; trId < numTrTypes; trId++)
    {
      const bool    is1stTest = (trId == 0);
      const bool    isLstTest = (trId == numTrTypes - 1);
      const MtsType trType    = ptList.trTypes[trId];

      if (transTested)
      {
        if (!bestCbf)
        {
          continue;
        }
        if (isMTS(trType) && (trType > maxMtsType || numMTSInvalid > 1))
        {
          continue;
        }
        if (m_encCfg->m_useTransformSkipFast && bestTU->mtsIdx[COMP_Y] == MtsType::SKIP && trType > MtsType::SKIP)
        {
          continue;
        }
      }

      isLstBest         = false;
      singleDistTmpLuma = 0;

      if (!is1stTest)
      {
        m_CABACEstimator->getCtx() = ctxStart;
      }

      tu.mtsIdx[COMP_Y]       = trType;
      tu.derivedIntraDirsLuma = ptList.derivedIntraDirs;
      TCoeff absSum           = 0;
      bool   valid            = xIntraCodingTUBlockLuma(tu, singleDistTmpLuma, ptList, absSum);
      bool   modeCbf          = TU::getCbfAtDepth(tu, COMP_Y, currDepth);
      transTested |= ((trType != MtsType::SKIP) && valid);
      if (trType == MtsType::DCT2_DCT2)
      {
        int nCands = (absSum + 4 > MTS_TH_COEFF[1]) ? MTS_NCANDS[2]
          : (absSum + 2 > MTS_TH_COEFF[0])          ? MTS_NCANDS[1]
                                                    : MTS_NCANDS[0];
        maxMtsType = MtsType(MtsType::MTS_1 + (nCands - 1));
      }
      else if (!valid && isMTS(trType))
      {
        numMTSInvalid++;
      }

      cuCtx.mtsLastScanPos                              = false;
      cuCtx.violatesMtsCoeffConstraint                  = false;
      cuCtx.lfnstLastScanPos                            = false;
      cuCtx.violatesLfnstConstrained[ChannelType::LUMA] = false;
      cuCtx.mtsCoeffAbsSum                              = 0;

      //----- determine rate and r-d cost -----
      if ((!is1stTest && !modeCbf) || !valid)
      {
        singleCostTmp = MAX_DOUBLE;
      }
      else
      {
        singleTmpFracBits = xGetIntraFracBitsQT(*csFull, partitioner, true, false, &cuCtx, &cuCtxIntra);
        const bool invalidMode =
          ((isMTS(trType) && (!cuCtx.mtsLastScanPos || cuCtx.violatesMtsCoeffConstraint)) ||
           (isNST(trType) && (!cuCtx.lfnstLastScanPos || cuCtx.violatesLfnstConstrained[ChannelType::LUMA])));
        if (invalidMode)
        {
          singleCostTmp = MAX_DOUBLE;
        }
        else
        {
          singleCostTmp = m_pcRdCost->calcRdCost(singleTmpFracBits, singleDistTmpLuma);
        }
      }

      if (singleCostTmp < singleCost)
      {
        isLstBest      = true;
        singleCost     = singleCostTmp;
        singleDistLuma = singleDistTmpLuma;
        singleFracBits = singleTmpFracBits;
        bestCbf        = modeCbf;

        if (!isLstTest)
        {
          saveCS.getRecoBuf(tu.Y()).copyFrom(csFull->getRecoBuf(tu.Y()));
          if (KEEP_PRED_AND_RESI_SIGNALS || ENABLE_NNLF)
          {
            saveCS.getPredBuf(tu.Y()).copyFrom(csFull->getPredBuf(tu.Y()));
          }
          if (KEEP_PRED_AND_RESI_SIGNALS)
          {
            saveCS.getResiBuf(tu.Y()).copyFrom(csFull->getResiBuf(tu.Y()));
            saveCS.getOrgResiBuf(tu.Y()).copyFrom(csFull->getOrgResiBuf(tu.Y()));
          }

          bestTU->copyComponentFrom(tu, COMP_Y);
          ctxBest = m_CABACEstimator->getCtx();
        }
      }
    }

    if (!isLstBest)
    {
      csFull->getRecoBuf(tu.Y()).copyFrom(saveCS.getRecoBuf(tu.Y()));
      if (KEEP_PRED_AND_RESI_SIGNALS || ENABLE_NNLF)
      {
        csFull->getPredBuf(tu.Y()).copyFrom(saveCS.getPredBuf(tu.Y()));
      }
      if (KEEP_PRED_AND_RESI_SIGNALS)
      {
        csFull->getResiBuf(tu.Y()).copyFrom(saveCS.getResiBuf(tu.Y()));
        csFull->getOrgResiBuf(tu.Y()).copyFrom(saveCS.getOrgResiBuf(tu.Y()));
      }

      tu.copyComponentFrom(*bestTU, COMP_Y);
      if (!checkSplit)
      {
        m_CABACEstimator->getCtx() = ctxBest;
      }
    }
    else if (checkSplit)
    {
      ctxBest = m_CABACEstimator->getCtx();
    }

    csFull->cost += singleCost;
    csFull->dist += singleDistLuma;
    csFull->fracBits += singleFracBits;
  }

  if (checkSplit)
  {
    //----- store full entropy coding status, load original entropy coding status -----
    if (checkFull)
    {
      m_CABACEstimator->getCtx() = ctxStart;
    }
    //----- code splitted block -----
    csSplit->cost = 0;

    bool splitCbfLuma    = false;
    bool splitIsSelected = true;
    if (partitioner.canSplit(TU_MAX_TR_SPLIT, cs))
    {
      partitioner.splitCurrArea(TU_MAX_TR_SPLIT, cs);
    }

    do
    {
      xRecurIntraCodingLumaQT(*csSplit, partitioner, ptList, cuCtxIntra);
      csSplit->setDecomp(partitioner.currArea().Y());

      splitCbfLuma |= TU::getCbfAtDepth(*csSplit->getTU(partitioner.currArea().lumaPos(), partitioner.chType), COMP_Y,
                                        partitioner.currTrDepth);

    } while (partitioner.nextPart(*csSplit));

    partitioner.exitCurrSplit();

    if (splitIsSelected)
    {
      for (auto &ptu: csSplit->tus)
      {
        if (currArea.Y().contains(ptu->Y()))
        {
          TU::setCbfAtDepth(*ptu, COMP_Y, currDepth, splitCbfLuma ? 1 : 0);
        }
      }

      //----- restore context states -----
      m_CABACEstimator->getCtx() = ctxStart;

      cuCtx.violatesLfnstConstrained.fill(false);
      cuCtx.lfnstLastScanPos           = false;
      cuCtx.violatesMtsCoeffConstraint = false;
      cuCtx.mtsLastScanPos             = false;
      cuCtx.mtsCoeffAbsSum             = 0;

      //----- determine rate and r-d cost -----
      csSplit->fracBits = xGetIntraFracBitsQT(*csSplit, partitioner, true, false, &cuCtx, &cuCtxIntra);
      //--- update cost ---
      csSplit->cost     = m_pcRdCost->calcRdCost(csSplit->fracBits, csSplit->dist);
    }
  }

  bool retVal = false;
  if (csFull || csSplit)
  {
    // otherwise this would've happened in useSubStructure
    cs.picture->getRecoBuf(currArea.Y()).copyFrom(cs.getRecoBuf(currArea.Y()));
    if (KEEP_PRED_AND_RESI_SIGNALS || ENABLE_NNLF)
    {
      cs.picture->getPredBuf(currArea.Y()).copyFrom(cs.getPredBuf(currArea.Y()));
    }
    if (KEEP_PRED_AND_RESI_SIGNALS)
    {
      cs.picture->getResiBuf(currArea.Y()).copyFrom(cs.getResiBuf(currArea.Y()));
    }
    cs.cost = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);
    retVal  = true;
  }
  return retVal;
}

ChromaCbfs IntraSearch::xRecurIntraCodingChromaQT(CodingStructure &cs, Partitioner &partitioner,
                                                  IModeTrCandChroma &ptList)
{
  UnitArea   currArea = partitioner.currArea();
  const bool keepResi = cs.sps->m_LMChroma || KEEP_PRED_AND_RESI_SIGNALS;
  ChromaCbfs cbfs(false);

  if (!currArea.Cb().valid() || !currArea.Cr().valid())
  {
    return cbfs;
  }

  TransformUnit &tu        = *cs.getTU(currArea.chromaPos(), ChannelType::CHROMA);
  const uint32_t currDepth = partitioner.currTrDepth;

  if (currDepth == tu.depth)
  {
    if (!ptList.valid)
    {
      xPreCalcPrdTransChroma(tu, ptList);
      xSelectTrCandChroma(tu, ptList);
      ptList.valid = false;   // reset for next TU inside CU
    }

    CodingStructure &saveCS = *m_pSaveCS[1];
    saveCS.pcv              = cs.pcv;
    saveCS.picture          = cs.picture;
    saveCS.initStructData(MAX_INT, true, &cs.area);

    TransformUnit &bestTU = saveCS.addTU(currArea, partitioner.chType);
    cs.setDecomp(currArea.Cb(), true);   // set in advance (required for Cb2/Cr2 in 4:2:2 video)

    CompArea  &cbArea       = tu.blocks[COMP_Cb];
    CompArea  &crArea       = tu.blocks[COMP_Cr];
    double     bestCostCbCr = 0;
    Distortion bestDistCbCr = 0;

    TempCtx ctxBest(m_ctxPool);
    TempCtx ctxStart(m_ctxPool);
    TempCtx ctxStartTU(m_ctxPool, m_CABACEstimator->getCtx());

    //--------------------------------------------
    //  [1]  Check separate transforms
    //--------------------------------------------
    tu.jointCbCr         = 0;
    bool didTransTest[3] = { false, false, false };
    for (CompID compID = COMP_Cb; compID <= COMP_Cr; compID = CompID(compID + 1))
    {
      const CompArea &area          = tu.blocks[compID];
      double          bestCostComp  = MAX_DOUBLE;
      Distortion      bestDistComp  = 0;
      double          singleCostTmp = 0;
      Distortion      singleDistTmp = 0;
      bool            bestCbf       = true;
      bool            isLstBest     = true;
      size_t          numTrTypes    = ptList.trTypesSep.size();

      if (numTrTypes > 1)
      {
        ctxStart = m_CABACEstimator->getCtx();
      }

      for (size_t trId = 0; trId < numTrTypes; trId++)
      {
        if (didTransTest[compID] && !bestCbf)
        {
          continue;
        }

        const bool    is1stTest = (trId == 0);
        const MtsType trType    = ptList.trTypesSep[trId];
        didTransTest[compID] |= (trType != MtsType::SKIP);

        if (!is1stTest)
        {
          m_CABACEstimator->getCtx() = ctxStart;
        }

        isLstBest                = false;
        singleDistTmp            = 0;
        tu.mtsIdx[compID]        = trType;
        tu.derivedIntraDirChroma = ptList.derivedIntraDirs[tu.jointCbCr];
        xIntraCodingTUBlockChroma(tu, compID, singleDistTmp, ptList);
        const bool modeCbf = TU::getCbf(tu, compID);

        //----- determine rate and r-d cost -----
        if (!is1stTest && !modeCbf)
        {
          singleCostTmp = MAX_DOUBLE;
        }
        else
        {
          singleCostTmp = m_pcRdCost->calcRdCost(xGetIntraFracBitsQTChroma(tu, compID), singleDistTmp);
        }

        if (singleCostTmp < bestCostComp)
        {
          isLstBest    = true;
          bestCostComp = singleCostTmp;
          bestDistComp = singleDistTmp;
          bestCbf      = modeCbf;

#if KEEP_PRED_AND_RESI_SIGNALS
          saveCS.getOrgResiBuf(area).copyFrom(cs.getOrgResiBuf(area));
          saveCS.getPredBuf(area).copyFrom(cs.getPredBuf(area));
#endif
          if (keepResi)
          {
            saveCS.getResiBuf(area).copyFrom(cs.getResiBuf(area));
          }
          saveCS.getRecoBuf(area).copyFrom(cs.getRecoBuf(area));

          bestTU.copyComponentFrom(tu, compID);
          ctxBest = m_CABACEstimator->getCtx();
        }
      }
      if (!isLstBest)
      {
        m_CABACEstimator->getCtx() = ctxBest;
        tu.copyComponentFrom(bestTU, compID);
      }
      bestCostCbCr += bestCostComp;
      bestDistCbCr += bestDistComp;
    }
    if (bestCostCbCr == 0)
    {
      bestCostCbCr = MAX_DOUBLE;   // nothing tested
    }

    //--------------------------------------------
    //  [2]  Check joint transforms
    //--------------------------------------------
    const bool testedTrans = didTransTest[COMP_Cb] && didTransTest[COMP_Cr];
    if (!ptList.trTypesJnt.empty())
    {
      const CompArea &areaCb        = tu.blocks[COMP_Cb];
      const CompArea &areaCr        = tu.blocks[COMP_Cr];
      double          singleCostTmp = 0;
      Distortion      singleDistTmp = 0;
      bool            isLstBest     = true;

      for (const auto trType: ptList.trTypesJnt)
      {
        m_CABACEstimator->getCtx() = ctxStartTU;

        isLstBest                = false;
        singleDistTmp            = 0;
        tu.mtsIdx[COMP_Cb]       = trType;
        tu.mtsIdx[COMP_Cr]       = trType;
        tu.derivedIntraDirChroma = ptList.derivedIntraDirs[tu.jointCbCr];
        xIntraCodingTUBlockChroma(tu, COMP_Cb, singleDistTmp, ptList);
        xIntraCodingTUBlockChroma(tu, COMP_Cr, singleDistTmp, ptList);

        //----- determine rate and r-d cost -----
        CUCtx cuCtx;
        cuCtx.lfnstLastScanPos                              = false;
        cuCtx.violatesLfnstConstrained[ChannelType::CHROMA] = false;

        if (!TU::getCbf(tu, COMP_Cb) && !TU::getCbf(tu, COMP_Cr) && testedTrans)
        {
          singleCostTmp = MAX_DOUBLE;
        }
        else
        {
          uint64_t fracBitsTmp = 0;
          fracBitsTmp += xGetIntraFracBitsQTChroma(tu, COMP_Cb, &cuCtx);
          fracBitsTmp += xGetIntraFracBitsQTChroma(tu, COMP_Cr, &cuCtx);
          const bool invalidMode =
            (isNST(trType) && (!cuCtx.lfnstLastScanPos || cuCtx.violatesLfnstConstrained[ChannelType::CHROMA]));
          if (invalidMode)
          {
            singleCostTmp = MAX_DOUBLE;
          }
          else
          {
            singleCostTmp = m_pcRdCost->calcRdCost(fracBitsTmp, singleDistTmp);
          }
        }

        if (singleCostTmp < bestCostCbCr)
        {
          isLstBest    = true;
          bestCostCbCr = singleCostTmp;
          bestDistCbCr = singleDistTmp;

#if KEEP_PRED_AND_RESI_SIGNALS
          saveCS.getOrgResiBuf(areaCb).copyFrom(cs.getOrgResiBuf(areaCb));
          saveCS.getOrgResiBuf(areaCr).copyFrom(cs.getOrgResiBuf(areaCr));
          saveCS.getPredBuf(areaCb).copyFrom(cs.getPredBuf(areaCb));
          saveCS.getPredBuf(areaCr).copyFrom(cs.getPredBuf(areaCr));
#endif
          if (keepResi)
          {
            saveCS.getResiBuf(areaCb).copyFrom(cs.getResiBuf(areaCb));
            saveCS.getResiBuf(areaCr).copyFrom(cs.getResiBuf(areaCr));
          }
          saveCS.getRecoBuf(areaCb).copyFrom(cs.getRecoBuf(areaCb));
          saveCS.getRecoBuf(areaCr).copyFrom(cs.getRecoBuf(areaCr));

          bestTU.copyComponentFrom(tu, COMP_Cb);
          bestTU.copyComponentFrom(tu, COMP_Cr);
          ctxBest = m_CABACEstimator->getCtx();
        }
      }
      if (!isLstBest)
      {
        tu.copyComponentFrom(bestTU, COMP_Cb);
        tu.copyComponentFrom(bestTU, COMP_Cr);
        m_CABACEstimator->getCtx() = ctxBest;
      }
    }

    //--------------------------------------------
    //  [3]  Check joint chroma coding
    //--------------------------------------------
    if (tu.cs->sps->m_jointCbCrEnabledFlag)
    {
      xPreCalcPrdTransJCCR(tu, ptList);

      bool isLstBest = true;
      for (const auto cbfMask: ptList.cbfMaskJCCR)
      {
        for (const auto trType: ptList.trTypesJCCR)
        {
          m_CABACEstimator->getCtx() = ctxStartTU;

          isLstBest                = false;
          tu.jointCbCr             = (uint8_t)cbfMask;
          tu.derivedIntraDirChroma = ptList.derivedIntraDirs[tu.jointCbCr];
          tu.mtsIdx[COMP_Cb]       = trType;
          tu.mtsIdx[COMP_Cr]       = trType;
          Distortion distTmp       = 0;
          xIntraCodingTUBlockChroma(tu, COMP_Cb, distTmp, ptList);

          double costTmp = MAX_DOUBLE;
          if (distTmp < std::numeric_limits<Distortion>::max())
          {
            CUCtx cuCtx;
            cuCtx.lfnstLastScanPos                              = false;
            cuCtx.violatesLfnstConstrained[ChannelType::CHROMA] = false;

            const uint64_t fracBitsTmp = xGetIntraFracBitsQTChroma(tu, COMP_Cb, &cuCtx);
            const bool     invalidMode =
              (isNST(trType) && (!cuCtx.lfnstLastScanPos || cuCtx.violatesLfnstConstrained[ChannelType::CHROMA]));
            if (!invalidMode)
            {
              costTmp = m_pcRdCost->calcRdCost(fracBitsTmp, distTmp);
            }
          }

          if (costTmp < bestCostCbCr)
          {
            isLstBest    = true;
            bestCostCbCr = costTmp;
            bestDistCbCr = distTmp;

#if KEEP_PRED_AND_RESI_SIGNALS
            saveCS.getOrgResiBuf(cbArea).copyFrom(cs.getOrgResiBuf(cbArea));
            saveCS.getOrgResiBuf(crArea).copyFrom(cs.getOrgResiBuf(crArea));
            saveCS.getPredBuf(cbArea).copyFrom(cs.getPredBuf(cbArea));
            saveCS.getPredBuf(crArea).copyFrom(cs.getPredBuf(crArea));
#endif
            if (keepResi)
            {
              saveCS.getResiBuf(cbArea).copyFrom(cs.getResiBuf(cbArea));
              saveCS.getResiBuf(crArea).copyFrom(cs.getResiBuf(crArea));
            }
            saveCS.getRecoBuf(cbArea).copyFrom(cs.getRecoBuf(cbArea));
            saveCS.getRecoBuf(crArea).copyFrom(cs.getRecoBuf(crArea));

            bestTU.copyComponentFrom(tu, COMP_Cb);
            bestTU.copyComponentFrom(tu, COMP_Cr);
            ctxBest = m_CABACEstimator->getCtx();
          }
        }
      }
      if (!isLstBest)
      {
        tu.copyComponentFrom(bestTU, COMP_Cb);
        tu.copyComponentFrom(bestTU, COMP_Cr);
        m_CABACEstimator->getCtx() = ctxBest;
      }
    }

    //=====  copy back  =====
#if KEEP_PRED_AND_RESI_SIGNALS
    cs.getOrgResiBuf(cbArea).copyFrom(saveCS.getOrgResiBuf(cbArea));
    cs.getOrgResiBuf(crArea).copyFrom(saveCS.getOrgResiBuf(crArea));
    cs.getPredBuf(cbArea).copyFrom(saveCS.getPredBuf(cbArea));
    cs.getPredBuf(crArea).copyFrom(saveCS.getPredBuf(crArea));
#endif
    if (keepResi)
    {
      cs.getResiBuf(cbArea).copyFrom(saveCS.getResiBuf(cbArea));
      cs.getResiBuf(crArea).copyFrom(saveCS.getResiBuf(crArea));
    }
    cs.getRecoBuf(cbArea).copyFrom(saveCS.getRecoBuf(cbArea));
    cs.getRecoBuf(crArea).copyFrom(saveCS.getRecoBuf(crArea));

    // Copy results to the picture structures
    cs.picture->getRecoBuf(cbArea).copyFrom(cs.getRecoBuf(cbArea));
    cs.picture->getRecoBuf(crArea).copyFrom(cs.getRecoBuf(crArea));
#if KEEP_PRED_AND_RESI_SIGNALS
    cs.picture->getPredBuf(cbArea).copyFrom(cs.getPredBuf(cbArea));
    cs.picture->getPredBuf(crArea).copyFrom(cs.getPredBuf(crArea));
    cs.picture->getResiBuf(cbArea).copyFrom(cs.getResiBuf(cbArea));
    cs.picture->getResiBuf(crArea).copyFrom(cs.getResiBuf(crArea));
#endif

    cbfs.cbf(COMP_Cb) = TU::getCbf(tu, COMP_Cb);
    cbfs.cbf(COMP_Cr) = TU::getCbf(tu, COMP_Cr);

    cs.dist += bestDistCbCr;
  }
  else
  {
    unsigned   numValidTBlocks = ::getNumberValidTBlocks(*cs.pcv);
    ChromaCbfs splitCbfs(false);

    if (partitioner.canSplit(TU_MAX_TR_SPLIT, cs))
    {
      partitioner.splitCurrArea(TU_MAX_TR_SPLIT, cs);
    }
    else
    {
      THROW("Implicit TU split not available");
    }

    do
    {
      ChromaCbfs subCbfs = xRecurIntraCodingChromaQT(cs, partitioner, ptList);

      for (uint32_t ch = COMP_Cb; ch < numValidTBlocks; ch++)
      {
        const CompID compID = CompID(ch);
        splitCbfs.cbf(compID) |= subCbfs.cbf(compID);
      }
    } while (partitioner.nextPart(cs));

    partitioner.exitCurrSplit();

    cbfs.Cb |= splitCbfs.Cb;
    cbfs.Cr |= splitCbfs.Cr;

    for (auto &ptu: cs.tus)
    {
      if (currArea.Cb().contains(ptu->Cb()) || (!ptu->Cb().valid() && currArea.Y().contains(ptu->Y())))
      {
        TU::setCbfAtDepth(*ptu, COMP_Cb, currDepth, splitCbfs.Cb);
        TU::setCbfAtDepth(*ptu, COMP_Cr, currDepth, splitCbfs.Cr);
      }
    }
  }

  return cbfs;
}

uint64_t IntraSearch::xFracModeBitsIntra(CodingUnit &cu, const uint32_t &mode, const ChannelType &chType,
                                         const CUCtxIntra &cuCtxIntra)
{
  const bool upd     = m_CABACEstimator->countWithUpdate(false);
  uint32_t   orgMode = mode;

  if (!cu.ciipFlag && !cu.gpmIntraFlag)
  {
    std::swap(orgMode, cu.intraDir[chType]);
  }

  m_CABACEstimator->resetBits();

  if (isLuma(chType))
  {
    if (!cu.ciipFlag && !cu.gpmIntraFlag)
    {
      m_CABACEstimator->intra_luma_pred_mode(cu, cuCtxIntra);
    }
  }
  else
  {
    m_CABACEstimator->intra_chroma_pred_mode(cu);
  }

  if (!cu.ciipFlag && !cu.gpmIntraFlag)
  {
    std::swap(orgMode, cu.intraDir[chType]);
  }
  m_CABACEstimator->countWithUpdate(upd);
  return m_CABACEstimator->getEstFracBits();
}

void IntraSearch::xSortRdModeListFirstColorSpace(ModeInfo mode, double cost, const BdpcmMode bdpcmMode,
                                                 ModeInfo *rdModeList, double *rdCostList, BdpcmMode *bdpcmModeList,
                                                 int &candNum)
{
  if (candNum == 0)
  {
    rdModeList[0]    = mode;
    rdCostList[0]    = cost;
    bdpcmModeList[0] = bdpcmMode;
    candNum++;
    return;
  }

  int insertPos = -1;
  for (int pos = candNum - 1; pos >= 0; pos--)
  {
    if (cost < rdCostList[pos])
    {
      insertPos = pos;
    }
  }

  if (insertPos >= 0)
  {
    for (int i = candNum - 1; i >= insertPos; i--)
    {
      rdModeList[i + 1]    = rdModeList[i];
      rdCostList[i + 1]    = rdCostList[i];
      bdpcmModeList[i + 1] = bdpcmModeList[i];
    }
    rdModeList[insertPos]    = mode;
    rdCostList[insertPos]    = cost;
    bdpcmModeList[insertPos] = bdpcmMode;
    candNum++;
  }
  else
  {
    rdModeList[candNum]    = mode;
    rdCostList[candNum]    = cost;
    bdpcmModeList[candNum] = bdpcmMode;
    candNum++;
  }

  CHECK(candNum > FAST_UDI_MAX_RDMODE_NUM, "exceed intra mode candidate list capacity");

  return;
}

template<typename T, size_t N>
void IntraSearch::xReduceHadCandList(static_vector<T, N> &candModeList, static_vector<double, N> &candCostList,
                                     SortedPelUnitBufs &sortedPelBuffer, int &numModesForFullRD,
                                     const double thresholdHadCost, const double *mipHadCost, const CodingUnit &cu,
                                     const bool fastMip)
{
  const int                                        maxCandPerType = numModesForFullRD >> 1;
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> tempRdModeList;
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM>   tempCandCostList;
  const double                                     minCost    = candCostList[0];
  bool                                             keepOneMip = candModeList.size() > numModesForFullRD;

  int numConv = 0;
  int numMip  = 0;
  for (int idx = 0; idx < candModeList.size() - (keepOneMip ? 0 : 1); idx++)
  {
    bool            addMode = false;
    const ModeInfo &orgMode = candModeList[idx];

    if (!orgMode.mipFlg)
    {
      addMode = (numConv < 3);
      numConv += addMode ? 1 : 0;
    }
    else
    {
      addMode    = (numMip < maxCandPerType || (candCostList[idx] < thresholdHadCost * minCost) || keepOneMip);
      keepOneMip = false;
      numMip += addMode ? 1 : 0;
    }
    if (addMode)
    {
      tempRdModeList.push_back(orgMode);
      tempCandCostList.push_back(candCostList[idx]);
    }
  }

  if ((cu.lwidth() > 8 && cu.lheight() > 8))
  {
    // Sort MIP candidates by Hadamard cost
    const int transpOff = MatrixIntraPrediction::getNumModesMip(cu.Y());

    static_vector<uint8_t, FAST_UDI_MAX_RDMODE_NUM> sortedMipModes(0);
    static_vector<double, FAST_UDI_MAX_RDMODE_NUM>  sortedMipCost(0);
    for (uint8_t mode: { 0, 1, 2 })
    {
      uint8_t candMode = mode + uint8_t((mipHadCost[mode + transpOff] < mipHadCost[mode]) ? transpOff : 0);
      updateCandList(candMode, mipHadCost[candMode], sortedMipModes, sortedMipCost, 3);
    }

    // Append MIP mode to RD mode list
    const int modeListSize = int(tempRdModeList.size());
    for (int idx = 0; idx < 3; idx++)
    {
      const bool     isTransposed = (sortedMipModes[idx] >= transpOff ? true : false);
      const uint32_t mipIdx       = (isTransposed ? sortedMipModes[idx] - transpOff : sortedMipModes[idx]);
      const ModeInfo mipMode(true, isTransposed, 0, mipIdx, -1);
      bool           alreadyIncluded = false;
      for (int modeListIdx = 0; modeListIdx < modeListSize; modeListIdx++)
      {
        if (tempRdModeList[modeListIdx] == mipMode)
        {
          alreadyIncluded = true;
          break;
        }
      }

      if (!alreadyIncluded)
      {
        updateCandList(mipMode, 0, tempRdModeList, tempCandCostList, tempCandCostList.size() + 1);
        if (fastMip)
        {
          break;
        }
      }
    }
  }

  candModeList      = tempRdModeList;
  candCostList      = tempCandCostList;
  numModesForFullRD = int(candModeList.size());
}
