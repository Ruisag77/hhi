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

/** \file     AlfHuffmanCode.h
    \brief    ALF Huffman Code class (header)
*/

#ifndef __ALFHUFFMANCODE__
#define __ALFHUFFMANCODE__

#include <cinttypes>
#include <vector>
#include <map>

class AlfCoeffRestriction
{
public:
  struct InputParam
  {
    int8_t bitWidth;
    int8_t mantissa;

    InputParam(int8_t bitWidth, int8_t mantissa) : bitWidth(bitWidth), mantissa(mantissa) {}

    bool operator<(const InputParam &right) const
    {
      if (bitWidth != right.bitWidth)
      {
        return bitWidth < right.bitWidth;
      }
      return mantissa < right.mantissa;
    }
  };

  struct Param
  {
    int16_t              minValue, maxValue;
    std::vector<int16_t> idxToCoeff;
    std::vector<int16_t> coeffToIdx;
  };

private:
  static std::map<InputParam, Param> m_cache;

  InputParam m_inputParam;
  Param     *m_param;

  void build();

public:
  AlfCoeffRestriction(int bitWidth, int mantissa);
  void init();

  const InputParam &getInputParam() { return m_inputParam; }
  const Param      &getParam() { return *m_param; }
};

class AlfHuffmanCode
{
public:
  struct InputParam
  {
    bool   isLuma;
    int8_t bitWidth;
    int8_t mantissa;
    int8_t coeffGroup;

    InputParam(bool isLuma, int8_t bitWidth, int8_t mantissa, int8_t coeffGroup)
      : isLuma(isLuma)
      , bitWidth(bitWidth)
      , mantissa(mantissa)
      , coeffGroup(coeffGroup)
    {}

    bool operator<(const InputParam &right) const
    {
      if (isLuma != right.isLuma)
      {
        return isLuma < right.isLuma;
      }
      if (bitWidth != right.bitWidth)
      {
        return bitWidth < right.bitWidth;
      }
      if (mantissa != right.mantissa)
      {
        return mantissa < right.mantissa;
      }
      return coeffGroup < right.coeffGroup;
    }
  };

  struct Node
  {
    int16_t coeffIdx;
    Node   *zero;
    Node   *one;
  };

  struct Param
  {
    Node                                    *root;
    std::vector<std::pair<uint32_t, int8_t>> codeTableAndLength;

    Param() : root(nullptr) {}

    void dfsDelete(Node *node)
    {
      if (node)
      {
        dfsDelete(node->zero);
        dfsDelete(node->one);
        delete node;
      }
    }

    ~Param() { dfsDelete(root); }
  };

private:
  static std::map<InputParam, Param>            m_cache;
  static std::map<InputParam, std::vector<int>> m_stats;

  InputParam          m_inputParam;
  AlfCoeffRestriction m_coeff;
  Param              *m_param;

  void build();
  void buildHuffmanTree();
  void buildCodeTable();
  void buildCodeTable(Node *node, uint32_t code, int level);

public:
  AlfHuffmanCode(bool isLuma, int bitWidth, int mantissa, int coeffGroup);
  void init();

  const InputParam &getInputParam() { return m_inputParam; }
  const Param      &getParam() { return *m_param; }

  void setGroup(int coeffGroup);

  bool    encodeCoeff(int16_t coeff, uint32_t &symbol, int &length);
  bool    decodeBit(int8_t bit, Node *&node);
  int16_t getCoeff(Node *node);
};

#endif // __ALFHUFFMANCODE__
