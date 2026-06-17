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

#include "EncNNFilterUnified.h"
#include "UnitTools.h"
#include "CodingStructure.h"
#include "CABACWriter.h"
#include <sadl/model.h>
using namespace std;

#if ENABLE_NNLF

static uint64_t xCalcSSD(const CPelUnitBuf &refBuf, const CPelUnitBuf &cmpBuf)
{
  uint64_t ssd = 0;

  for (int comp = 0; comp < refBuf.bufs.size(); ++comp)
  {
    const CompID    c         = CompID(comp);
    const int       width     = refBuf.get(c).width;
    const int       height    = refBuf.get(c).height;
    const ptrdiff_t orgStride = refBuf.get(c).stride;
    const ptrdiff_t cmpStride = cmpBuf.get(c).stride;
    const Pel      *pOrg      = refBuf.get(c).buf;
    const Pel      *pCmp      = cmpBuf.get(c).buf;

    for (int y = 0; y < height; ++y)
    {
      for (int x = 0; x < width; ++x)
      {
        int temp = pOrg[x] - pCmp[x];
        ssd += (temp * temp);
      }
      pOrg += orgStride;
      pCmp += cmpStride;
    }
  }
  return ssd;
}

void EncNNFilterUnified::initCabac(CABACEncoder *cabacEncoder, CtxPool *ctxPool, Slice &slice)
{
  m_CABACEstimator = cabacEncoder->getCABACEstimator(slice.m_sps);
  m_CtxPool        = ctxPool;
  m_CABACEstimator->initCtxModels(slice);
  m_CABACEstimator->resetBits();
  for (int c = 0; c < getNumberValidComponents(slice.m_sps->m_chromaFormatIdc); ++c)
  {
    m_lambda[c] = slice.getLambdas()[c];
  }
}

double EncNNFilterUnified::getSignalingCost(Picture &pic)
{
  // calculate RD cost
  const TempCtx ctxStart(m_CtxPool, SubCtx(Ctx::nnlfUnifiedParams, m_CABACEstimator->getCtx()));
  m_CABACEstimator->getCtx() = SubCtx(Ctx::nnlfUnifiedParams, ctxStart);
  m_CABACEstimator->resetBits();
  m_CABACEstimator->writeNnlfUnifiedParameters(*pic.m_cs);
  double rate = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
  return rate;
}

void EncNNFilterUnified::chooseParameters(Picture &pic)
{
  // find the best scale and qp offset
  const CodingStructure &cs  = *pic.m_cs;
  const PreCalcValues   &pcv = *cs.pcv;

  NnlfFilterParameters &picprms = getPicprms();

  bool applyMultiplier = cs.slice->m_eSliceType != I_SLICE;
  for (int prmId = 0; prmId < picprms.prmNum; prmId++)
  {
    fill(picprms.prmId.begin(), picprms.prmId.end(), prmId);
    filter(pic, applyMultiplier, false);

    for (int scaleId = 1; scaleId < 3; scaleId++)
    {
      getScaledBuf(scaleId, prmId).copyFrom(getScaledBuf(0, prmId));
    }
  }

  double           minCost = MAX_DOUBLE;
  std::vector<int> bestPrmId;

  // find best parameters without or with scaling
  for (int scaleId = -1; scaleId < 3; scaleId++)
  {
    scalePicture(pic, scaleId);
    parameterSearch(pic, minCost, bestPrmId, scaleId);
  }
  picprms.prmId = bestPrmId;

  if (picprms.sprm.mode < picprms.prmNum)
  {
    fill(picprms.prmId.begin(), picprms.prmId.end(), picprms.sprm.mode);
  }

  int cpt = 0;
  for (int y = 0; y < picprms.nb_blocks_height; ++y)
  {
    for (int x = 0; x < picprms.nb_blocks_width; ++x, ++cpt)
    {
      int xPos   = x * picprms.block_size;
      int yPos   = y * picprms.block_size;
      int width  = (xPos + picprms.block_size > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : picprms.block_size;
      int height = (yPos + picprms.block_size > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : picprms.block_size;

      const UnitArea block(cs.area.chromaFormat, Area(xPos, yPos, width, height));

      if (picprms.prmId[cpt] == -1)
      {
        continue;
      }

      if (picprms.sprm.scaleId != -1)
      {
        int scaleId = picprms.sprm.scaleId;
        pic.getRecoBuf(block).copyFrom(getScaledBuf(scaleId, picprms.prmId[cpt], block));
      }
      else
      {
        pic.getRecoBuf(block).copyFrom(getFilteredBuf(picprms.prmId[cpt], block));
      }
    }
  }
}

void EncNNFilterUnified::parameterSearch(Picture &pic, double &minCost, std::vector<int> &bestPrmId, int scaleId)
{
  CodingStructure     &cs  = *pic.m_cs;
  const PreCalcValues &pcv = *cs.pcv;

  NnlfFilterParameters &picprms = getPicprms();
  const int             prmNum  = picprms.prmNum;

  std::vector<uint64_t> dist(prmNum + 2, 0LL);
  uint64_t *pDist = &dist[1]; // pDist[-1]: slice off distortion, pDist[picprms.prmNum]: block level adaptation
                              // distortion, pDist[i]: slice on distortion

  std::vector<double>   ssdRec(picprms.nb_blocks_height * picprms.nb_blocks_width, 0LL);
  std::map<int, double> euclideanDist;

  int cpt = 0;
  for (int y = 0; y < picprms.nb_blocks_height; ++y)
  {
    for (int x = 0; x < picprms.nb_blocks_width; ++x, ++cpt)
    {
      int xPos   = x * picprms.block_size;
      int yPos   = y * picprms.block_size;
      int width  = (xPos + picprms.block_size > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : picprms.block_size;
      int height = (yPos + picprms.block_size > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : picprms.block_size;

      const UnitArea block(cs.area.chromaFormat, Area(xPos, yPos, width, height));

      uint64_t distBest  = MAX_UINT64;
      int      prmIdBest = -1;

      for (int prmId = -1; prmId < prmNum; prmId++)
      {
        const CPelUnitBuf tempBuf  = prmId == -1
           ? pic.getRecoBuf(block)
           : (scaleId == -1 ? getFilteredBuf(prmId, block) : getScaledBuf(scaleId, prmId, block));
        uint64_t          distTemp = xCalcSSD(pic.getOrigBuf(block), tempBuf);
        if (prmId == -1)
        {
          ssdRec[cpt] = (double)distTemp;
        }
        if (distTemp < distBest)
        {
          distBest  = distTemp;
          prmIdBest = prmId;
        }
        pDist[prmId] += distTemp;
      }
      pDist[prmNum] += distBest;
      picprms.prmId[cpt] = prmIdBest;
      if ((double)distBest - ssdRec[cpt] < 0)
      {
        euclideanDist[cpt] = ssdRec[cpt] - (double)distBest;
      }
    }
  }

  int tempMode      = picprms.sprm.mode;
  picprms.sprm.mode = prmNum;
  double rateAdpt   = getSignalingCost(pic); // rate of adaptation
  double rateScale =
    getNumberValidComponents(cs.sps->m_chromaFormatIdc) * (log2ResidueScale + 1); // rate of signalling scales
  picprms.sprm.mode    = tempMode;
  const bool hasChroma = isChromaEnabled(cs.sps->m_chromaFormatIdc);

  // compare R-D cost
  for (int mode = -1; mode < prmNum + 1; mode++)
  {
    double cost;
    if (mode == -1)
    {
      cost = (double)pDist[mode];
    }
    else if (mode < prmNum)
    {
      int i          = (scaleId == -1) ? 3 : scaleId;
      int rateOffset = 0;
      rateOffset += (picprms.sprm.offset[CompID::COMP_Y][mode][i] == 0) ? 1 : 2;
      if (hasChroma)
      {
        rateOffset += (picprms.sprm.offset[CompID::COMP_Cb][mode][i] == 0) ? 1 : 2;
        rateOffset += (picprms.sprm.offset[CompID::COMP_Cr][mode][i] == 0) ? 1 : 2;
      }
      cost = (double)pDist[mode] + m_lambda[CompID::COMP_Y] * (scaleId == 0 ? (rateScale + rateOffset) : (rateOffset));
    }
    else
    {
      int i          = (scaleId == -1) ? 3 : scaleId;
      int rateOffset = 0;
      for (int mode = 0; mode < prmNum; mode++)
      {
        rateOffset += (picprms.sprm.offset[CompID::COMP_Y][mode][i] == 0) ? 1 : 2;
        if (hasChroma)
        {
          rateOffset += (picprms.sprm.offset[CompID::COMP_Cb][mode][i] == 0) ? 1 : 2;
          rateOffset += (picprms.sprm.offset[CompID::COMP_Cr][mode][i] == 0) ? 1 : 2;
        }
      }
      cost = (double)pDist[mode] +
        m_lambda[CompID::COMP_Y] * (rateAdpt + (scaleId == 0 ? (prmNum * rateScale + rateOffset) : (rateOffset)));
    }

    if (cost < minCost)
    {
      minCost              = cost;
      picprms.sprm.mode    = mode;
      picprms.sprm.scaleId = scaleId;
      bestPrmId            = picprms.prmId;
    }
  }
  if (useNnlfDecOpt && picprms.sprm.mode == 2 &&
      (scaleId == -1 || scaleId == 2)) // This optimzation is used only for block-level mode
  {
    std::vector<pair<int, double>> vtEuclideanDist;
    double                         ratio  = 0.01;
    int                            offNum = 0;
    for (auto it = euclideanDist.begin(); it != euclideanDist.end(); it++)
    {
      vtEuclideanDist.push_back(make_pair(it->first, it->second));
    }
    sort(vtEuclideanDist.begin(), vtEuclideanDist.end(),
         [](const pair<int, double> &x, const pair<int, double> &y) -> int { return x.second < y.second; });

    for (auto it = vtEuclideanDist.begin(); it != vtEuclideanDist.end(); it++)
    {
      if ((double)(it->second / ssdRec[it->first]) > ratio)
      {
        break;
      }
      offNum++;
    }

    for (int i = 0; i < offNum; i++)
    {
      bestPrmId[vtEuclideanDist[i].first] = -1;
    }
  }
}

void EncNNFilterUnified::scaleFactorDerivation(Picture &pic, NnlfFilterParameters &prms, int prmId, int scaleId)
{
  CodingStructure &cs      = *pic.m_cs;
  Slice           *pcSlice = cs.slice;

  const CPelUnitBuf recoBuf             = pic.getRecoBuf();
  const CPelUnitBuf origBuf             = pic.getOrigBuf();
  const CPelUnitBuf filteredBuf         = getScaledBuf(0, prmId);
  const CPelUnitBuf recoBeforeDbfBuf    = pic.getRecBeforeDbfBuf();
  const CPelUnitBuf filteredNoScaledBuf = getFilteredBuf(prmId);

  const int   inputBitDepth    = pcSlice->clpRng(CompID::COMP_Y).bd;   // internal bitdepth
  const int   shift            = log2OutputScale - inputBitDepth;
  const float stablizingFactor = (0.1f * (1 << shift));
  const int   nc               = getNumberValidComponents(cs.sps->m_chromaFormatIdc);

  int area = recoBuf.get(CompID::COMP_Y).width * recoBuf.get(CompID::COMP_Y).height;
  for (int compIdx = 0; compIdx < nc; compIdx++)
  {
    CompID        compID = CompID(compIdx);
    const CPelBuf orgBuf = origBuf.get(compID);
    const CPelBuf recBuf = recoBuf.get(compID);
    const CPelBuf cnnBuf = filteredBuf.get(compID);

    int height = recBuf.height;
    int width  = recBuf.width;

    const CPelBuf recBeforeDbfBuf = recoBeforeDbfBuf.get(compID);
    const CPelBuf cnnNoScaledBuf  = filteredNoScaledBuf.get(compID);

    if (scaleId == -1)
    {
      uint64_t costMin       = MAX_UINT64;
      int      bestRoaOffset = 0;

      for (int roaOffset = 0; roaOffset <= 2; roaOffset++)
      {
        uint64_t ssd = 0LL;
        for (int y = 0; y < height; y++)
        {
          for (int x = 0; x < width; x++)
          {
            // positive-, negative+
            int curP = 0;
            if ((cnnBuf.at(x, y) - (recBeforeDbfBuf.at(x, y) << shift)) >= (roaOffset << shift))
            {
              curP = cnnNoScaledBuf.at(x, y) - roaOffset;
            }
            else if ((cnnBuf.at(x, y) - (recBeforeDbfBuf.at(x, y) << shift)) <= (-roaOffset << shift))
            {
              curP = cnnNoScaledBuf.at(x, y) + roaOffset;
            }
            else
            {
              curP = cnnNoScaledBuf.at(x, y);
            }

            Pel clipP = Pel(Clip3<int>(0, (1 << inputBitDepth) - 1, curP));
            ssd += (orgBuf.at(x, y) - clipP) * (orgBuf.at(x, y) - clipP);
          }
        }
        if (ssd < costMin)
        {
          costMin       = ssd;
          bestRoaOffset = roaOffset;
        }
      }
      prms.sprm.offset[compID][prmId][3] = bestRoaOffset;

      continue;
    }

    double selfMulti  = 0.;
    double crossMulti = 0.;
    double sumOrgResi = 0.;
    double sumCnnResi = 0.;

    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        int orgResi = (orgBuf.at(x, y) - recBuf.at(x, y)) << shift;
        int cnnResi = cnnBuf.at(x, y) - (recBuf.at(x, y) << shift);
        selfMulti += cnnResi * cnnResi;
        crossMulti += cnnResi * orgResi;
        sumOrgResi += orgResi;
        sumCnnResi += cnnResi;
      }
    }

    int up  = int(nnResidueScaleDerivationUpBound * (1 << log2ResidueScale));
    int bot = int(nnResidueScaleDerivationLowBound * (1 << log2ResidueScale));

    int areaComp = compID == 0 ? area : (area / 4);

#if ENABLE_POST_CFE_CHANGES
    int scale =
      int(((areaComp * crossMulti - sumOrgResi * sumCnnResi + areaComp * (double)areaComp * stablizingFactor) /
           (areaComp * selfMulti - sumCnnResi * sumCnnResi + areaComp * (double)areaComp * stablizingFactor)) *
            (1 << log2ResidueScale) +
          0.5);
#else
    int scale = int(((areaComp * crossMulti - sumOrgResi * sumCnnResi + areaComp * areaComp * stablizingFactor) /
                     (areaComp * selfMulti - sumCnnResi * sumCnnResi + areaComp * areaComp * stablizingFactor)) *
                      (1 << log2ResidueScale) +
                    0.5);
#endif

    scale                          = Clip3(bot, up, scale);
    prms.sprm.scale[compID][prmId] = scale;

    const int shift2 = shift + log2ResidueScale;
    const int offset = (1 << shift2) / 2;

    for (int i = 0; i < 3; i++)
    {
      int      scale         = i == 0 ? prms.sprm.scale[compID][prmId] : scale_candidates[i];
      uint64_t costMin       = MAX_UINT64;
      int      bestRoaOffset = 0;

      for (int roaOffset = 0; roaOffset <= 2; roaOffset++)
      {
        uint64_t ssd = 0LL;
        for (int y = 0; y < height; y++)
        {
          for (int x = 0; x < width; x++)
          {
            // positive-, negative+
            int curP = 0;
            if ((cnnBuf.at(x, y) - (recBeforeDbfBuf.at(x, y) << shift)) >= (roaOffset << shift))
            {
              curP = (((int)recBuf.at(x, y) << shift2) +
                      (cnnBuf.at(x, y) - (recBuf.at(x, y) << shift) - (roaOffset << shift)) * scale + offset) >>
                shift2;
            }
            else if ((cnnBuf.at(x, y) - (recBeforeDbfBuf.at(x, y) << shift)) <= (-roaOffset << shift))
            {
              curP = (((int)recBuf.at(x, y) << shift2) +
                      (cnnBuf.at(x, y) - (recBuf.at(x, y) << shift) + (roaOffset << shift)) * scale + offset) >>
                shift2;
            }
            else
            {
              curP =
                (((int)recBuf.at(x, y) << shift2) + (cnnBuf.at(x, y) - (recBuf.at(x, y) << shift)) * scale + offset) >>
                shift2;
            }

            Pel clipP = Pel(Clip3<int>(0, (1 << inputBitDepth) - 1, curP));
            ssd += (orgBuf.at(x, y) - clipP) * (orgBuf.at(x, y) - clipP);
          }
        }
        if (ssd < costMin)
        {
          costMin       = ssd;
          bestRoaOffset = roaOffset;
        }
      }
      prms.sprm.offset[compID][prmId][i] = bestRoaOffset;
    }
  }
}

void EncNNFilterUnified::scalePicture(Picture &pic, int scaleId)
{
  if (scaleId > 2)
  {
    return;
  }

  CodingStructure      &cs                 = *pic.m_cs;
  const PreCalcValues  &pcv                = *cs.pcv;
  NnlfFilterParameters &prms               = getPicprms();
  const int             numValidComponents = getNumberValidComponents(cs.area.chromaFormat);
  for (int prmId = 0; prmId < prms.prmNum; prmId++)
  {
    if (scaleId == 0 || scaleId == -1)
    {
      scaleFactorDerivation(pic, prms, prmId, scaleId);
    }
    for (int comp = 0; comp < numValidComponents; comp++)
    {
      const CompID c = CompID(comp);

      for (int y = 0; y < prms.nb_blocks_height; ++y)
      {
        for (int x = 0; x < prms.nb_blocks_width; ++x)
        {
          int            xPos   = x * prms.block_size;
          int            yPos   = y * prms.block_size;
          int            width  = (xPos + prms.block_size > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : prms.block_size;
          int            height = (yPos + prms.block_size > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : prms.block_size;
          const UnitArea inferAreaNoExt(cs.area.chromaFormat, Area(xPos, yPos, width, height));

          if (scaleId == -1)
          {
            PelUnitBuf scaledBuf   = getScaledBuf(0, prmId, inferAreaNoExt);
            PelUnitBuf noScaledBuf = getFilteredBuf(prmId, inferAreaNoExt);
            scaleResidualBlock(pic, c, inferAreaNoExt, scaledBuf.get(c), noScaledBuf.get(c), 0,
                               prms.sprm.offset[c][prmId][3]);
          }
          else
          {
            PelUnitBuf scaledBuf = getScaledBuf(scaleId, prmId, inferAreaNoExt);
            scaleId == 0 ? scaleResidualBlock(pic, c, inferAreaNoExt, scaledBuf.get(c), scaledBuf.get(c),
                                              prms.sprm.scale[c][prmId], prms.sprm.offset[c][prmId][scaleId])
                         : scaleResidualBlock(pic, c, inferAreaNoExt, scaledBuf.get(c), scaledBuf.get(c),
                                              scale_candidates[scaleId], prms.sprm.offset[c][prmId][scaleId]);
          }
        }
      }
    }
  }
}

#endif
