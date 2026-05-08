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

/** \file     AlfHuffmanCode.cpp
    \brief    ALF Huffman Code class
*/

#include "AlfHuffmanCode.h"
#include "CommonDef.h"

#include <queue>

// clang-format off
std::map<AlfCoeffRestriction::InputParam, AlfCoeffRestriction::Param> AlfCoeffRestriction::m_cache;
std::map<AlfHuffmanCode::InputParam, AlfHuffmanCode::Param> AlfHuffmanCode::m_cache;
std::map<AlfHuffmanCode::InputParam, std::vector<int>> AlfHuffmanCode::m_stats = {
  { AlfHuffmanCode::InputParam(false, 7, 1, 0), { 0,0,8,138,814,1997,2063,1861,1977,1895,1923,1952,1439,776,147,1,0, } },
  { AlfHuffmanCode::InputParam(false, 7, 1, 1), { 3,1,6,26,50,41,24,18,19,15,110,240,707,2153,5862,7068,648, } },
  { AlfHuffmanCode::InputParam(true, 5, 1, 0), { 3,4,44,451,1625,1738,4212,1739,2151,861,153,10,2, } },
  { AlfHuffmanCode::InputParam(true, 5, 1, 1), { 39,129,415,812,1026,1270,1963,957,2291,1919,1440,647,88, } },
  { AlfHuffmanCode::InputParam(true, 5, 2, 0), { 1,2,4,14,52,167,551,856,1826,2761,7151,2873,2050,1138,986,382,130,31,10,2,2, } },
  { AlfHuffmanCode::InputParam(true, 5, 2, 1), { 35,59,125,245,415,729,553,964,938,1342,3738,2245,1981,1228,1603,1450,1434,1056,600,216,36, } },
  { AlfHuffmanCode::InputParam(true, 6, 1, 0), { 4,6,33,350,1609,2023,1745,2205,1685,1985,2108,1046,181,10,2, } },
  { AlfHuffmanCode::InputParam(true, 6, 1, 1), { 43,121,455,964,1290,751,880,994,896,1753,1964,2266,1802,745,70, } },
  { AlfHuffmanCode::InputParam(true, 6, 2, 0), { 4,3,4,13,51,132,551,1031,1805,1707,1987,2674,2645,2610,2028,1870,2097,1593,1344,568,227,33,8,2,3, } },
  { AlfHuffmanCode::InputParam(true, 6, 2, 1), { 39,31,123,269,551,771,1164,1258,876,904,956,962,1837,1739,1767,1573,1696,1497,2360,1491,1489,871,571,158,35, } },
  { AlfHuffmanCode::InputParam(true, 7, 1, 0), { 1,2,13,206,1133,2103,1981,1233,2279,1571,1788,2215,1618,743,101,3,1, } },
  { AlfHuffmanCode::InputParam(true, 7, 1, 1), { 55,165,454,932,1054,903,706,422,875,419,961,1332,2295,2544,2350,1407,120, } },
  { AlfHuffmanCode::InputParam(true, 7, 2, 0), { 2,2,3,6,25,77,356,791,1705,1688,1986,1591,1733,1913,2382,1816,1761,1734,2138,2020,2209,1332,1075,459,162,15,4,2,1, } },
  { AlfHuffmanCode::InputParam(true, 7, 2, 1), { 39,33,125,247,493,718,946,1040,1060,966,863,406,733,890,1071,857,988,955,1784,1685,1924,1946,2327,2077,2016,1441,1046,273,38, } },
  { AlfHuffmanCode::InputParam(true, 8, 1, 0), { 4,5,31,304,1205,1990,1603,1321,1196,2261,1318,1288,1887,2184,1575,713,97,6,3, } },
  { AlfHuffmanCode::InputParam(true, 8, 1, 1), { 74,164,555,949,1185,855,674,536,292,524,401,458,932,1727,2139,2774,2856,1760,137, } },
  { AlfHuffmanCode::InputParam(true, 8, 2, 0), { 0,0,2,3,17,72,351,647,1362,1359,1919,1386,1841,1220,1662,1899,2206,1959,1656,1235,1863,1850,2284,1921,1956,1119,805,285,92,8,2,0,0, } },
  { AlfHuffmanCode::InputParam(true, 8, 2, 1), { 29,33,115,208,417,500,787,888,948,904,846,535,463,440,473,713,1033,505,597,588,1112,1094,1718,1626,1979,2127,2705,2682,2862,2199,1496,326,38, } },
};
// clang-format on

void AlfCoeffRestriction::build()
{
  CHECK(m_inputParam.mantissa < 1 || m_inputParam.mantissa > 2,
        "ALF supports only mantissa bit precision in range [1, 2]");
  m_param->maxValue = 1 << m_inputParam.bitWidth;
  m_param->minValue = -m_param->maxValue;
  m_param->idxToCoeff.resize(0);
  std::vector<int> pos;
  int              cand = 1;
  while (cand <= m_param->maxValue)
  {
    pos.push_back(cand);
    if (m_inputParam.mantissa == 2)
    {
      int cand2 = (cand + (cand >> 1));
      if (cand2 != cand && cand2 <= m_param->maxValue)
      {
        pos.push_back(cand2);
      }
    }
    cand <<= 1;
  }
  for (auto it = pos.rbegin(); it != pos.rend(); it++)
  {
    m_param->idxToCoeff.push_back(-(*it));
  }
  m_param->idxToCoeff.push_back(0);
  for (auto it = pos.begin(); it != pos.end(); it++)
  {
    m_param->idxToCoeff.push_back(*it);
  }

  m_param->coeffToIdx.assign(m_param->maxValue - m_param->minValue + 1, -1);
  for (int i = 0; i < (int)m_param->idxToCoeff.size(); i++)
  {
    m_param->coeffToIdx[m_param->idxToCoeff[i] - m_param->minValue] = i;
  }

  for (int i = 1, lastIdx = m_param->coeffToIdx[-m_param->minValue]; i <= m_param->maxValue; i++)
  {
    int j = i - m_param->minValue;
    if (m_param->coeffToIdx[j] == -1)
    {
      m_param->coeffToIdx[j] = lastIdx;
    }
    else
    {
      lastIdx = m_param->coeffToIdx[j];
    }
  }
  for (int i = -1, lastIdx = m_param->coeffToIdx[-m_param->minValue]; i >= m_param->minValue; i--)
  {
    int j = i - m_param->minValue;
    if (m_param->coeffToIdx[j] == -1)
    {
      m_param->coeffToIdx[j] = lastIdx;
    }
    else
    {
      lastIdx = m_param->coeffToIdx[j];
    }
  }
}

AlfCoeffRestriction::AlfCoeffRestriction(int bitWidth, int mantissa)
  : m_inputParam(bitWidth, mantissa)
  , m_param(nullptr)
{}

void AlfCoeffRestriction::init()
{
  m_param = &m_cache[m_inputParam];
  if (m_param->idxToCoeff.empty())
  {
    build();
  }
}

void AlfHuffmanCode::build()
{
  buildHuffmanTree();
  buildCodeTable();
  for (int i = 0; i < (int)m_param->codeTableAndLength.size(); i++)
  {
    CHECK(m_param->codeTableAndLength[i].second > 32, "The length of Huffman code is greater than 32!")
  }
}

void AlfHuffmanCode::buildHuffmanTree()
{
  std::vector<int> &stats = m_stats[m_inputParam];
  CHECK(stats.size() != m_coeff.getParam().idxToCoeff.size(),
        "The size of alphabet in statistics is different from the actual alphabet size");

  std::priority_queue<std::pair<std::pair<int, int>, Node *>> q;
  int                                                         nodeId = 0;
  for (int i = 0; i < (int)m_coeff.getParam().idxToCoeff.size(); i++)
  {
    Node *node     = new Node();
    node->coeffIdx = i;
    node->zero     = nullptr;
    node->one      = nullptr;
    q.emplace(std::make_pair(-stats[i], nodeId++), node);
  }

  while (q.size() > 1)
  {
    std::pair<std::pair<int, int>, Node *> zero = q.top();
    q.pop();
    std::pair<std::pair<int, int>, Node *> one = q.top();
    q.pop();
    Node *top     = new Node();
    top->coeffIdx = -1;
    top->zero     = zero.second;
    top->one      = one.second;
    q.emplace(std::make_pair(zero.first.first + one.first.first, nodeId++), top);
  }

  m_param->root = q.top().second;
}

void AlfHuffmanCode::buildCodeTable()
{
  m_param->codeTableAndLength.resize(m_coeff.getParam().idxToCoeff.size());
  buildCodeTable(m_param->root, 0, 0);
}

void AlfHuffmanCode::buildCodeTable(Node *node, uint32_t code, int level)
{
  if (node->coeffIdx >= 0)
  {
    m_param->codeTableAndLength[node->coeffIdx] = std::make_pair((uint32_t)code, (int8_t)level);
  }
  else
  {
    buildCodeTable(node->zero, code << 1, level + 1);
    buildCodeTable(node->one, (code << 1) | (uint32_t)1, level + 1);
  }
}

AlfHuffmanCode::AlfHuffmanCode(bool isLuma, int bitWidth, int mantissa, int coeffGroup)
  : m_inputParam(isLuma, bitWidth, mantissa, coeffGroup)
  , m_coeff(bitWidth, mantissa)
  , m_param(nullptr)
{}

void AlfHuffmanCode::init()
{
  m_coeff.init();
  m_param = &m_cache[m_inputParam];
  if (!m_param->root)
  {
    build();
  }
}

void AlfHuffmanCode::setGroup(int coeffGroup)
{
  if (m_inputParam.coeffGroup != coeffGroup)
  {
    m_inputParam.coeffGroup = coeffGroup;
    if (m_param)
    {
      init();
    }
  }
}

bool AlfHuffmanCode::encodeCoeff(int16_t coeff, uint32_t &symbol, int &length)
{
  CHECK(coeff < m_coeff.getParam().minValue, "ALF coefficient is less than the minimal allowed value");
  CHECK(coeff > m_coeff.getParam().maxValue, "ALF coefficient is more than the maximal allowed value");
  int16_t idx = m_coeff.getParam().coeffToIdx[coeff - m_coeff.getParam().minValue];
  CHECK(coeff != m_coeff.getParam().idxToCoeff[idx], "ALF coefficient is outside of the allowed set of values");
  symbol = m_param->codeTableAndLength[idx].first;
  length = m_param->codeTableAndLength[idx].second;
  return true;
}

bool AlfHuffmanCode::decodeBit(int8_t bit, Node *&node)
{
  if (node == nullptr)
  {
    node = m_param->root;
  }
  node = bit ? node->one : node->zero;
  return node->coeffIdx >= 0;
}

int16_t AlfHuffmanCode::getCoeff(Node *node) { return m_coeff.getParam().idxToCoeff[node->coeffIdx]; }
