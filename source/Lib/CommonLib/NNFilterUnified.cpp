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

#include "NNFilterUnified.h"
#include "NNInference.h"
#include <fstream>
#include <sstream>

#include "NNFilterDefaultModel.inc"

using namespace std;

#if ENABLE_NNLF

// some constants
static constexpr int log2InputQpScale  = 6;
static constexpr int log2InputIbpScale = 0;
static constexpr int defaultInputSize  = 128;   // block size
static constexpr int defaultBlockExt   = 8;

struct Input
{
  enum
  {
    Rec = 0,
    Pred,
    BS,
    QPbase,
    QPBlock,
    IPB,
    MultiplierParam,
    TargetMultiplierParam,
    nbInputs
  };
};
struct InputTemporal
{
  enum
  {
    Rec = 0,
    Pred,
    List0,
    List1,
    QPbase,
    nbInputsTemporal
  };
};

void NNFilterUnified::init(const std::string &filename, int picWidth, int picHeight, ChromaFormat format, int prmNum,
                           bool reloadModel /* false */, bool useInputMultiplierSwitch /* false */)
{
  bool modelChange = !m_model || reloadModel;

  if (modelChange)
  {
    bool useBuiltIn = filename.empty();

    ifstream     file;
    stringstream defModel;

    if (!useBuiltIn)
    {
      file.open(filename, ios::in | ios::binary);

      if (!file)
      {
        THROW("Unable to open NNFilter model " << filename);
      }
    }
    else
    {
      defModel.str(std::string(std::begin(defaultNnlfModel), std::end(defaultNnlfModel)));
    }

    m_model.reset(new sadl::Model<TypeSadlLFUnified>());

    if (useBuiltIn ? !m_model->load(defModel) : !m_model->load(file))
    {
      if (useBuiltIn)
      {
        THROW("Issue loading default NNFilter model");
      }
      else
      {
        THROW("Issue loading model NNFilter " << filename);
      }
    }
  }

  // prepare inputs
  int numInputs = useInputMultiplierSwitch ? Input::nbInputs : Input::nbInputs - 2;
  m_inputs.resize(numInputs);
  resizeInputs(defaultInputSize + defaultBlockExt * 2, defaultInputSize + defaultBlockExt * 2, modelChange,
               useInputMultiplierSwitch);

  if (m_filtered.size() > 0 || m_scaled[0].size() > 0)
  {
    return;
  }

  m_filtered.resize(prmNum);
  for (int i = 0; i < prmNum; i++)
  {
    m_filtered[i].create(format, Area(0, 0, picWidth, picHeight));
  }
  for (int j = 0; j < 3; j++)
  {
    m_scaled[j].resize(prmNum);
    for (int i = 0; i < prmNum; i++)
    {
      m_scaled[j][i].create(format, Area(0, 0, picWidth, picHeight));
    }
  }
}
void NNFilterUnified::initTemporal(const std::string &filename)
{
  ifstream file(filename, ios::binary);
  if (!file)
  {
    std::cerr << "[ERROR] unable to open NNFilter temporal model " << filename << std::endl;
    exit(-1);
  }
  if (!m_modelTemporal)
  {
    m_modelTemporal.reset(new sadl::Model<TypeSadlLFUnified>());
    if (!m_modelTemporal->load(file))
    {
      cerr << "[ERROR] issue loading temporal model NNFilter " << filename << endl;
      exit(-1);
    }
  }
  // prepare inputs
  m_inputsTemporal.resize(InputTemporal::nbInputsTemporal);
  resizeInputsTemporal(defaultInputSize + defaultBlockExt * 2, defaultInputSize + defaultBlockExt * 2);
}
void NNFilterUnified::destroy()
{
  for (int i = 0; i < (int)m_filtered.size(); i++)
  {
    m_filtered[i].destroy();
  }
  for (int j = 0; j < 3; j++)
  {
    for (int i = 0; i < (int)m_scaled[j].size(); i++)
    {
      m_scaled[j][i].destroy();
    }
  }
}

// default is square block + extension
void NNFilterUnified::resizeInputs(int width, int height, bool modelChange /* false */,
                                   bool useInputMultiplierSwitch /* false */)
{
  int sizeW = width;
  int sizeH = height;
  if ((sizeH == m_blocksize[0] && sizeW == m_blocksize[1]) && !modelChange)
  {
    return;
  }
  m_blocksize[0] = sizeH;
  m_blocksize[1] = sizeW;
  if (m_nnlfTransInput)
  {
    constexpr int dctSizeW = 2;
    constexpr int dctSizeH = 2;
    constexpr int dctCh    = dctSizeW * dctSizeH;
    assert(sizeH % dctSizeH == 0);
    assert(sizeW % dctSizeW == 0);
    m_inputs[Input::Rec].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, dctCh + dctCh / 2 }));
    m_inputs[Input::Pred].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, dctCh + dctCh / 2 }));
    m_inputs[Input::BS].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, 3 }));
    m_inputs[Input::QPbase].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, 1 }));
    m_inputs[Input::QPBlock].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, 1 }));
    m_inputs[Input::IPB].resize(sadl::Dimensions({ 1, sizeH / dctSizeH, sizeW / dctSizeW, 1 }));
  }
  else
  {
    // note: later QP inputs can be optimized to avoid some duplicate computation with a 1x1 input
    m_inputs[Input::Rec].resize(sadl::Dimensions({ 1, sizeH, sizeW, 3 }));
    m_inputs[Input::Pred].resize(sadl::Dimensions({ 1, sizeH, sizeW, 3 }));
    m_inputs[Input::BS].resize(sadl::Dimensions({ 1, sizeH, sizeW, 3 }));
    m_inputs[Input::QPbase].resize(sadl::Dimensions({ 1, sizeH, sizeW, 1 }));
    m_inputs[Input::QPBlock].resize(sadl::Dimensions({ 1, sizeH, sizeW, 1 }));
    m_inputs[Input::IPB].resize(sadl::Dimensions({ 1, sizeH, sizeW, 1 }));
  }

  if (useInputMultiplierSwitch)
  {
    m_inputs[Input::MultiplierParam].resize(sadl::Dimensions { 1, 1 });
    m_inputs[Input::TargetMultiplierParam].resize(sadl::Dimensions { 1, 1 });
  }

  if (!m_model->init(m_inputs))
  {
    cerr << "[ERROR] issue init model NNFilterUnified " << endl;
    exit(-1);
  }
  if (useInputMultiplierSwitch)
  {
    assert(nb_inputs + 2 == m_inputs.size());
  }
  else
  {
    assert(nb_inputs == m_inputs.size());
  }
  m_input_quantizer.resize(m_inputs.size());
  for (int i = 0; i < nb_inputs; ++i)
  {
    m_input_quantizer[i] = m_model->getInputsTemplate()[i].quantizer;
  }
}
// default is square block + extension
void NNFilterUnified::resizeInputsTemporal(int width, int height)   // assume using same quantizer as the primary model
{
  int sizeW = width;
  int sizeH = height;
  if (sizeH == m_blocksizeTemporal[0] && sizeW == m_blocksizeTemporal[1])
  {
    return;
  }
  m_blocksizeTemporal[0] = sizeH;
  m_blocksizeTemporal[1] = sizeW;
  // note: later QP inputs can be optimized to avoid some duplicate computation with a 1x1 input
  m_inputsTemporal[InputTemporal::Rec].resize(sadl::Dimensions({ 1, sizeH, sizeW, 1 }));
  m_inputsTemporal[InputTemporal::Pred].resize(sadl::Dimensions({ 1, sizeH, sizeW, 1 }));
  m_inputsTemporal[InputTemporal::List0].resize(sadl::Dimensions({ 1, sizeH, sizeW, 1 }));
  m_inputsTemporal[InputTemporal::List1].resize(sadl::Dimensions({ 1, sizeH, sizeW, 1 }));
  m_inputsTemporal[InputTemporal::QPbase].resize(sadl::Dimensions({ 1, sizeH, sizeW, 1 }));

  if (!m_modelTemporal->init(m_inputsTemporal))
  {
    cerr << "[ERROR] issue init temporal model NNFilterHOP " << endl;
    exit(-1);
  }
  m_input_quantizer_temporal =
    m_modelTemporal->getInputsTemplate()[0].quantizer;   // assume all image inputs have same quantizer
}
void roundToOutputBitdepth(const PelUnitBuf &src, PelUnitBuf &dst, const ClpRngs &clpRngs, ChromaFormat chromaId)
{
  const int nc = getNumberValidComponents(chromaId);
  for (int c = 0; c < nc; c++)
  {
    const int shift  = NNFilterUnified::log2OutputScale - clpRngs.comp[CompID(c)].bd;
    const int offset = 1 << (shift - 1);
    assert(shift >= 0);
    const PelBuf &bufSrc = src.get(CompID(c));
    PelBuf       &bufDst = dst.get(CompID(c));
    const int     width  = bufSrc.width;
    const int     height = bufSrc.height;
    for (int y = 0; y < height; ++y)
    {
      for (int x = 0; x < width; ++x)
      {
        bufDst.at(x, y) = Pel(Clip3<int>(0, 1023, (bufSrc.at(x, y) + offset) >> shift));
      }
    }
  }
}

// bufDst: temporary buffer to store results
// inferArea: area used for inference (include extension)
template<typename T> static void extractOutputs(const Picture &pic, sadl::Model<T> &m, PelUnitBuf &bufDst,
                                                UnitArea inferArea, int extLeft, int extRight, int extTop,
                                                int extBottom)
{
  const int log2InputBitdepth = pic.m_cs->slice->clpRng(CompID::COMP_Y).bd;   // internal bitdepth
  auto      output            = m.result(0);
  const int qOutputSadl       = output.quantizer;
  const int shiftInput        = NNFilterUnified::log2OutputScale - log2InputBitdepth;
  const int shiftOutput       = NNFilterUnified::log2OutputScale - qOutputSadl;
  assert(shiftInput >= 0);
  assert(shiftOutput >= 0);

  const int width   = bufDst.Y().width;
  const int height  = bufDst.Y().height;
  PelBuf   &bufDstY = bufDst.get(CompID::COMP_Y);
  CPelBuf   bufRecY = pic.getRecBeforeDbfBuf(inferArea).get(CompID::COMP_Y);

  for (int c = 0; c < 4; ++c)   // unshuffle on C++ side
  {
    for (int y = 0; y < height / 2; ++y)
    {
      for (int x = 0; x < width / 2; ++x)
      {
        int yy = (y * 2) + c / 2;
        int xx = (x * 2) + c % 2;
        if (xx < extLeft || yy < extTop || xx >= width - extRight || yy >= height - extBottom)
        {
          continue;
        }
        int out;
        if constexpr (std::is_same<TypeSadlLFUnified, float>::value)
        {
          out = round((output(0, y, x, c) * (1 << shiftOutput) + (float)bufRecY.at(xx, yy) * (1 << shiftInput)));
        }
        else
        {
          out = ((output(0, y, x, c) << shiftOutput) + (bufRecY.at(xx, yy) << shiftInput));
        }

        bufDstY.at(xx, yy) = Pel(Clip3<int>(0, (1 << NNFilterUnified::log2OutputScale) - 1, out));
      }
    }
  }

  if (isChromaEnabled(pic.m_cs->slice->m_sps->m_chromaFormatIdc))
  {
    PelBuf &bufDstCb = bufDst.get(CompID::COMP_Cb);
    PelBuf &bufDstCr = bufDst.get(CompID::COMP_Cr);
    CPelBuf bufRecCb = pic.getRecBeforeDbfBuf(inferArea).get(CompID::COMP_Cb);
    CPelBuf bufRecCr = pic.getRecBeforeDbfBuf(inferArea).get(CompID::COMP_Cr);

    for (int y = 0; y < height / 2; ++y)
    {
      for (int x = 0; x < width / 2; ++x)
      {
        if (x < extLeft / 2 || y < extTop / 2 || x >= width / 2 - extRight / 2 || y >= height / 2 - extBottom / 2)
        {
          continue;
        }

        int outCb;
        int outCr;
        if constexpr (std::is_same<TypeSadlLFUnified, float>::value)
        {
          outCb = round(output(0, y, x, 4) * (1 << shiftOutput) + (float)bufRecCb.at(x, y) * (1 << shiftInput));
          outCr = round(output(0, y, x, 5) * (1 << shiftOutput) + (float)bufRecCr.at(x, y) * (1 << shiftInput));
        }
        else
        {
          outCb = ((output(0, y, x, 4) << shiftOutput) + (bufRecCb.at(x, y) << shiftInput));
          outCr = ((output(0, y, x, 5) << shiftOutput) + (bufRecCr.at(x, y) << shiftInput));
        }

        bufDstCb.at(x, y) = Pel(Clip3<int>(0, (1 << NNFilterUnified::log2OutputScale) - 1, outCb));
        bufDstCr.at(x, y) = Pel(Clip3<int>(0, (1 << NNFilterUnified::log2OutputScale) - 1, outCr));
      }
    }
  }
}
template<typename T> static void extractOutputsDCT(const Picture &pic, sadl::Model<T> &m, PelUnitBuf &bufDst,
                                                   UnitArea inferArea, int extLeft, int extRight, int extTop,
                                                   int extBottom)
{
  const int log2InputBitdepth = pic.m_cs->slice->clpRng(CompID::COMP_Y).bd;   // internal bitdepth
  auto      output            = m.result(0);
  const int qOutputSadl       = output.quantizer;
  const int shiftInput        = NNFilterUnified::log2OutputScale - log2InputBitdepth;
  const int shiftOutput       = NNFilterUnified::log2OutputScale - qOutputSadl;
  assert(shiftInput >= 0);
  assert(shiftOutput >= 0);

  const int width   = bufDst.Y().width;
  const int height  = bufDst.Y().height;
  PelBuf   &bufDstY = bufDst.get(CompID::COMP_Y);
  CPelBuf   bufRecY = pic.getRecBeforeDbfBuf(inferArea).get(CompID::COMP_Y);

  constexpr int dctSizeW                       = 2;
  constexpr int dctSizeH                       = 2;
  TCoeff        block[dctSizeW * dctSizeH]     = {};
  TCoeff        tempCoeff[dctSizeW * dctSizeH] = {};
  TCoeff        tmp[dctSizeW * dctSizeH]       = {};

  const bool earlyCropping = (inferArea.Y().width / 2 > output.dims()[2]);
  const int  border        = extLeft / dctSizeH;

  for (int y = 0; y < height / dctSizeH; y++)
  {
    for (int x = 0; x < width / dctSizeW; x++)
    {
      int c1 = x % 2 + (y % 2) * 2;   // take every 4th channel before pixel shuffle
      if (earlyCropping)
      {
        if (y < border || y >= ((height / dctSizeH) - border) || x < border || x >= ((width / dctSizeW) - border))
        {
          continue;
        }
      }
      if constexpr (std::is_same<TypeSadlLFUnified, float>::value)
      {
        for (int cc = c1; cc < dctSizeW * dctSizeH * 4; cc += 4)
        {
          if (earlyCropping)
          {
            tempCoeff[cc / 4] = output(0, (y - border) / 2, (x - border) / 2, cc) * (1 << shiftOutput);
          }
          else
          {
            tempCoeff[cc / 4] = output(0, y / 2, x / 2, cc) * (1 << shiftOutput);
          }
        }
      }
      else
      {
        for (int cc = c1; cc < dctSizeW * dctSizeH * 4; cc += 4)
        {
          if (earlyCropping)
          {
            tempCoeff[cc / 4] = output(0, (y - border) / 2, (x - border) / 2, cc) << shiftOutput;
          }
          else
          {
            tempCoeff[cc / 4] = output(0, y / 2, x / 2, cc) << shiftOutput;
          }
        }
      }
      tmp[0]   = tempCoeff[0] + tempCoeff[1];
      tmp[1]   = tempCoeff[2] + tempCoeff[3];
      tmp[2]   = tempCoeff[0] - tempCoeff[1];
      tmp[3]   = tempCoeff[2] - tempCoeff[3];
      block[0] = tmp[0] + tmp[1];
      block[1] = tmp[2] + tmp[3];
      block[2] = tmp[0] - tmp[1];
      block[3] = tmp[2] - tmp[3];

      for (int y1 = 0; y1 < dctSizeH; y1++)
      {
        for (int x1 = 0; x1 < dctSizeW; x1++)
        {
          int xx = x * dctSizeW + x1;
          int yy = y * dctSizeH + y1;

          if (xx < extLeft || yy < extTop || xx >= width - extRight || yy >= height - extBottom)
          {
            continue;
          }
          int out;
          if constexpr (std::is_same<TypeSadlLFUnified, float>::value)
          {
            out = round((block[(y1 * dctSizeW) + x1] + (float)bufRecY.at(xx, yy) * (1 << shiftInput)));
          }
          else
          {
            out = ((block[(y1 * dctSizeW) + x1]) + (bufRecY.at(xx, yy) << shiftInput));
          }

          bufDstY.at(xx, yy) = Pel(Clip3<int>(0, (1 << NNFilterUnified::log2OutputScale) - 1, out));
        }
      }
    }
  }

  if (isChromaEnabled(pic.m_cs->slice->m_sps->m_chromaFormatIdc))
  {
    PelBuf &bufDstCb                         = bufDst.get(CompID::COMP_Cb);
    PelBuf &bufDstCr                         = bufDst.get(CompID::COMP_Cr);
    CPelBuf bufRecCb                         = pic.getRecBeforeDbfBuf(inferArea).get(CompID::COMP_Cb);
    CPelBuf bufRecCr                         = pic.getRecBeforeDbfBuf(inferArea).get(CompID::COMP_Cr);
    TCoeff  blockCb[dctSizeW * dctSizeH]     = {};
    TCoeff  blockCr[dctSizeW * dctSizeH]     = {};
    TCoeff  tempCoeffCb[dctSizeW * dctSizeH] = {};
    TCoeff  tempCoeffCr[dctSizeW * dctSizeH] = {};
    TCoeff  tmpCb[dctSizeW * dctSizeH]       = {};
    TCoeff  tmpCr[dctSizeW * dctSizeH]       = {};

    for (int y = 0; y < height / 2 / dctSizeH; y++)
    {
      for (int x = 0; x < width / 2 / dctSizeW; x++)
      {
        if (earlyCropping)
        {
          if (y < 2 || y >= ((height / 2 / dctSizeH) - 2) || x < 2 || x >= ((width / 2 / dctSizeW) - 2))
          {
            continue;
          }
        }
        if constexpr (std::is_same<TypeSadlLFUnified, float>::value)
        {
          for (int cc = dctSizeW * dctSizeH * 4; cc < dctSizeW * dctSizeH * 5; cc++)
          {
            if (earlyCropping)
            {
              tempCoeffCb[cc - dctSizeW * dctSizeH * 4] = output(0, y - 2, x - 2, cc) * (1 << shiftOutput);
              tempCoeffCr[cc - dctSizeW * dctSizeH * 4] =
                output(0, y - 2, x - 2, cc + dctSizeW * dctSizeH) * (1 << shiftOutput);
            }
            else
            {
              tempCoeffCb[cc - dctSizeW * dctSizeH * 4] = output(0, y, x, cc) * (1 << shiftOutput);
              tempCoeffCr[cc - dctSizeW * dctSizeH * 4] =
                output(0, y, x, cc + dctSizeW * dctSizeH) * (1 << shiftOutput);
            }
          }
        }
        else
        {
          for (int cc = dctSizeW * dctSizeH * 4; cc < dctSizeW * dctSizeH * 5; cc++)
          {
            if (earlyCropping)
            {
              tempCoeffCb[cc - dctSizeW * dctSizeH * 4] = output(0, y - 2, x - 2, cc) << shiftOutput;
              tempCoeffCr[cc - dctSizeW * dctSizeH * 4] = output(0, y - 2, x - 2, cc + dctSizeW * dctSizeH)
                << shiftOutput;
            }
            else
            {
              tempCoeffCb[cc - dctSizeW * dctSizeH * 4] = output(0, y, x, cc) << shiftOutput;
              tempCoeffCr[cc - dctSizeW * dctSizeH * 4] = output(0, y, x, cc + dctSizeW * dctSizeH) << shiftOutput;
            }
          }
        }
        tmpCb[0]   = tempCoeffCb[0] + tempCoeffCb[1];
        tmpCb[1]   = tempCoeffCb[2] + tempCoeffCb[3];
        tmpCb[2]   = tempCoeffCb[0] - tempCoeffCb[1];
        tmpCb[3]   = tempCoeffCb[2] - tempCoeffCb[3];
        blockCb[0] = tmpCb[0] + tmpCb[1];
        blockCb[1] = tmpCb[2] + tmpCb[3];
        blockCb[2] = tmpCb[0] - tmpCb[1];
        blockCb[3] = tmpCb[2] - tmpCb[3];
        tmpCr[0]   = tempCoeffCr[0] + tempCoeffCr[1];
        tmpCr[1]   = tempCoeffCr[2] + tempCoeffCr[3];
        tmpCr[2]   = tempCoeffCr[0] - tempCoeffCr[1];
        tmpCr[3]   = tempCoeffCr[2] - tempCoeffCr[3];
        blockCr[0] = tmpCr[0] + tmpCr[1];
        blockCr[1] = tmpCr[2] + tmpCr[3];
        blockCr[2] = tmpCr[0] - tmpCr[1];
        blockCr[3] = tmpCr[2] - tmpCr[3];

        for (int y1 = 0; y1 < dctSizeH; y1++)
        {
          for (int x1 = 0; x1 < dctSizeW; x1++)
          {
            int xx = x * dctSizeW + x1;
            int yy = y * dctSizeH + y1;
            if (xx < extLeft / 2 || yy < extTop / 2 || xx >= width / 2 - extRight / 2 ||
                yy >= height / 2 - extBottom / 2)
            {
              continue;
            }
            int outCb;
            int outCr;
            if constexpr (std::is_same<TypeSadlLFUnified, float>::value)
            {
              outCb = round((blockCb[(y1 * dctSizeW) + x1] + (float)bufRecCb.at(xx, yy) * (1 << shiftInput)));
              outCr = round((blockCr[(y1 * dctSizeW) + x1] + (float)bufRecCr.at(xx, yy) * (1 << shiftInput)));
            }
            else
            {
              outCb = ((blockCb[(y1 * dctSizeW) + x1]) + (bufRecCb.at(xx, yy) << shiftInput));
              outCr = ((blockCr[(y1 * dctSizeW) + x1]) + (bufRecCr.at(xx, yy) << shiftInput));
            }
            bufDstCb.at(xx, yy) = Pel(Clip3<int>(0, (1 << NNFilterUnified::log2OutputScale) - 1, outCb));
            bufDstCr.at(xx, yy) = Pel(Clip3<int>(0, (1 << NNFilterUnified::log2OutputScale) - 1, outCr));
          }
        }
      }
    }
  }
}

// bufDst: temporary buffer to store results
// inferArea: area used for inference (include extension)
template<typename T> static void extractOutputsTemporal(const Picture &pic, sadl::Model<T> &m, PelUnitBuf &bufDst,
                                                        UnitArea inferArea, int extLeft, int extRight, int extTop,
                                                        int extBottom)
{
  const int log2InputBitdepth = pic.m_cs->slice->clpRng(CompID::COMP_Y).bd;   // internal bitdepth
  auto      output            = m.result(0);
  const int qOutputSadl       = output.quantizer;
  const int shiftInput        = NNFilterUnified::log2OutputScale - log2InputBitdepth;
  const int shiftOutput       = NNFilterUnified::log2OutputScale - qOutputSadl;
  assert(shiftInput >= 0);
  assert(shiftOutput >= 0);

  const int width   = bufDst.Y().width;
  const int height  = bufDst.Y().height;
  PelBuf   &bufDstY = bufDst.get(CompID::COMP_Y);
  CPelBuf   bufRecY = pic.getRecBeforeDbfBuf(inferArea).get(CompID::COMP_Y);

  for (int c = 0; c < 4; ++c)   // unshuffle on C++ side
  {
    for (int y = 0; y < height / 2; ++y)
    {
      for (int x = 0; x < width / 2; ++x)
      {
        int yy = (y * 2) + c / 2;
        int xx = (x * 2) + c % 2;
        if (xx < extLeft || yy < extTop || xx >= width - extRight || yy >= height - extBottom)
        {
          continue;
        }
        int out;
        if constexpr (std::is_same<TypeSadlLFUnified, float>::value)
        {
          out = round((output(0, y, x, c) * (1 << shiftOutput) + (float)bufRecY.at(xx, yy) * (1 << shiftInput)));
        }
        else
        {
          out = ((output(0, y, x, c) << shiftOutput) + (bufRecY.at(xx, yy) << shiftInput));
        }

        bufDstY.at(xx, yy) = Pel(Clip3<int>(0, (1 << NNFilterUnified::log2OutputScale) - 1, out));
      }
    }
  }

  if (isChromaEnabled(pic.m_cs->slice->m_sps->m_chromaFormatIdc))
  {
    PelBuf &bufDstCb = bufDst.get(CompID::COMP_Cb);
    PelBuf &bufDstCr = bufDst.get(CompID::COMP_Cr);
    CPelBuf bufRecCb = pic.getRecoBuf(inferArea).get(CompID::COMP_Cb);
    CPelBuf bufRecCr = pic.getRecoBuf(inferArea).get(CompID::COMP_Cr);

    for (int y = 0; y < height / 2; ++y)
    {
      for (int x = 0; x < width / 2; ++x)
      {
        if (x < extLeft / 2 || y < extTop / 2 || x >= width / 2 - extRight / 2 || y >= height / 2 - extBottom / 2)
        {
          continue;
        }

        int outCb = (bufRecCb.at(x, y) << shiftInput);
        int outCr = (bufRecCr.at(x, y) << shiftInput);

        bufDstCb.at(x, y) = Pel(Clip3<int>(0, (1 << NNFilterUnified::log2OutputScale) - 1, outCb));
        bufDstCr.at(x, y) = Pel(Clip3<int>(0, (1 << NNFilterUnified::log2OutputScale) - 1, outCr));
      }
    }
  }
}
void NNFilterUnified::filterBlock(Picture &pic, UnitArea inferArea, int extLeft, int extRight, int extTop,
                                  int extBottom, int prmId, bool applyMultiplier)
{
  // get model
  auto      &model     = *m_model;
  bool       inter     = pic.m_slices[0]->m_eSliceType != I_SLICE ? true : false;
  int        qpOffset  = (inter ? prmId * 5 : prmId * 2) * (pic.m_slices[0]->m_uiTLayer >= 4 ? 1 : -1);
  int        seqQp     = pic.m_slices[0]->m_pps->m_picInitQPMinus26 + 26 + qpOffset;
  int        sliceQp   = pic.m_slices[0]->m_iSliceQp;
  const bool hasChroma = isChromaEnabled(pic.m_cs->slice->m_sps->m_chromaFormatIdc);
  resizeInputs(inferArea.Y().width, inferArea.Y().height);

#if ENABLE_TRACING
  {
    const Picture &_pic = pic;
    if (inferArea.lx() < 0 && inferArea.ly() < 0)
    {
      DTRACE(g_trace_ctx, D_NNLF, "\nSTART new pic: poc = %d\n\n", _pic.m_poc);
    }
    CPelUnitBuf _buf = _pic.getBsMapBuf(inferArea);
    DTRACE(g_trace_ctx, D_NNLF, "getBsMapBuf LUMA: poc = %d, area = (%d,%d) - (%d,%d) (size = %dx%d)\n\n", _pic.m_poc,
           inferArea.lx(), inferArea.ly(), inferArea.lx() + inferArea.Y().width, inferArea.ly() + inferArea.Y().height,
           inferArea.Y().width, inferArea.Y().height);
    DTRACE_BLOCK(g_trace_ctx, D_NNLF, _buf.get(CompID::COMP_Y).buf, _buf.get(CompID::COMP_Y).stride,
                 inferArea.Y().width, inferArea.Y().height);
    if (hasChroma)
    {
      DTRACE(g_trace_ctx, D_NNLF, "getBsMapBuf CB: poc = %d, area = (%d,%d) - (%d,%d) (size = %dx%d)\n\n", _pic.m_poc,
             inferArea.Cb().x, inferArea.Cb().y, inferArea.Cb().x + inferArea.Cb().width,
             inferArea.Cb().y + inferArea.Cb().height, inferArea.Cb().width, inferArea.Cb().height);
      DTRACE_BLOCK(g_trace_ctx, D_NNLF, _buf.get(CompID::COMP_Cb).buf, _buf.get(CompID::COMP_Cb).stride,
                   inferArea.Cb().width, inferArea.Cb().height);
      DTRACE(g_trace_ctx, D_NNLF, "getBsMapBuf CR: poc = %d, area = (%d,%d) - (%d,%d) (size = %dx%d)\n\n", _pic.m_poc,
             inferArea.Cr().x, inferArea.Cr().y, inferArea.Cr().x + inferArea.Cr().width,
             inferArea.Cr().y + inferArea.Cr().height, inferArea.Cr().width, inferArea.Cr().height);
      DTRACE_BLOCK(g_trace_ctx, D_NNLF, _buf.get(CompID::COMP_Cr).buf, _buf.get(CompID::COMP_Cr).stride,
                   inferArea.Cr().width, inferArea.Cr().height);
    }

    _buf = _pic.getBlockPredModeBuf(inferArea);
    DTRACE(g_trace_ctx, D_NNLF, "getBlockPredModeBuf LUMA: poc = %d, area = (%d,%d) - (%d,%d) (size = %dx%d)\n\n",
           _pic.m_poc, inferArea.lx(), inferArea.ly(), inferArea.lx() + inferArea.Y().width,
           inferArea.ly() + inferArea.Y().height, inferArea.Y().width, inferArea.Y().height);
    DTRACE_BLOCK(g_trace_ctx, D_NNLF, _buf.get(CompID::COMP_Y).buf, _buf.get(CompID::COMP_Y).stride,
                 inferArea.Y().width, inferArea.Y().height);
    if (hasChroma)
    {
      DTRACE(g_trace_ctx, D_NNLF, "getBlockPredModeBuf CB: poc = %d, area = (%d,%d) - (%d,%d) (size = %dx%d)\n\n",
             _pic.m_poc, inferArea.Cb().x, inferArea.Cb().y, inferArea.Cb().x + inferArea.Cb().width,
             inferArea.Cb().y + inferArea.Cb().height, inferArea.Cb().width, inferArea.Cb().height);
      DTRACE_BLOCK(g_trace_ctx, D_NNLF, _buf.get(CompID::COMP_Cb).buf, _buf.get(CompID::COMP_Cb).stride,
                   inferArea.Cb().width, inferArea.Cb().height);
      DTRACE(g_trace_ctx, D_NNLF, "getBlockPredModeBuf CR: poc = %d, area = (%d,%d) - (%d,%d) (size = %dx%d)\n\n",
             _pic.m_poc, inferArea.Cr().x, inferArea.Cr().y, inferArea.Cr().x + inferArea.Cr().width,
             inferArea.Cr().y + inferArea.Cr().height, inferArea.Cr().width, inferArea.Cr().height);
      DTRACE_BLOCK(g_trace_ctx, D_NNLF, _buf.get(CompID::COMP_Cr).buf, _buf.get(CompID::COMP_Cr).stride,
                   inferArea.Cr().width, inferArea.Cr().height);
    }

    _buf = _pic.getRecBeforeDbfBuf(inferArea);
    DTRACE(g_trace_ctx, D_NNLF, "getRecBeforeDbfBuf LUMA: poc = %d, area = (%d,%d) - (%d,%d) (size = %dx%d)\n\n",
           _pic.m_poc, inferArea.lx(), inferArea.ly(), inferArea.lx() + inferArea.Y().width,
           inferArea.ly() + inferArea.Y().height, inferArea.Y().width, inferArea.Y().height);
    DTRACE_BLOCK(g_trace_ctx, D_NNLF, _buf.get(CompID::COMP_Y).buf, _buf.get(CompID::COMP_Y).stride,
                 inferArea.Y().width, inferArea.Y().height);
    if (hasChroma)
    {
      DTRACE(g_trace_ctx, D_NNLF, "getRecBeforeDbfBuf CB: poc = %d, area = (%d,%d) - (%d,%d) (size = %dx%d)\n\n",
             _pic.m_poc, inferArea.Cb().x, inferArea.Cb().y, inferArea.Cb().x + inferArea.Cb().width,
             inferArea.Cb().y + inferArea.Cb().height, inferArea.Cb().width, inferArea.Cb().height);
      DTRACE_BLOCK(g_trace_ctx, D_NNLF, _buf.get(CompID::COMP_Cb).buf, _buf.get(CompID::COMP_Cb).stride,
                   inferArea.Cb().width, inferArea.Cb().height);
      DTRACE(g_trace_ctx, D_NNLF, "getRecBeforeDbfBuf CR: poc = %d, area = (%d,%d) - (%d,%d) (size = %dx%d)\n\n",
             _pic.m_poc, inferArea.Cr().x, inferArea.Cr().y, inferArea.Cr().x + inferArea.Cr().width,
             inferArea.Cr().y + inferArea.Cr().height, inferArea.Cr().width, inferArea.Cr().height);
      DTRACE_BLOCK(g_trace_ctx, D_NNLF, _buf.get(CompID::COMP_Cr).buf, _buf.get(CompID::COMP_Cr).stride,
                   inferArea.Cr().width, inferArea.Cr().height);
    }

    _buf = _pic.getPredBufCustom(inferArea);
    DTRACE(g_trace_ctx, D_NNLF, "getPredBufCustom LUMA: poc = %d, area = (%d,%d) - (%d,%d) (size = %dx%d)\n\n",
           _pic.m_poc, inferArea.lx(), inferArea.ly(), inferArea.lx() + inferArea.Y().width,
           inferArea.ly() + inferArea.Y().height, inferArea.Y().width, inferArea.Y().height);
    DTRACE_BLOCK(g_trace_ctx, D_NNLF, _buf.get(CompID::COMP_Y).buf, _buf.get(CompID::COMP_Y).stride,
                 inferArea.Y().width, inferArea.Y().height);
    if (hasChroma)
    {
      DTRACE(g_trace_ctx, D_NNLF, "getPredBufCustom CB: poc = %d, area = (%d,%d) - (%d,%d) (size = %dx%d)\n\n",
             _pic.m_poc, inferArea.Cb().x, inferArea.Cb().y, inferArea.Cb().x + inferArea.Cb().width,
             inferArea.Cb().y + inferArea.Cb().height, inferArea.Cb().width, inferArea.Cb().height);
      DTRACE_BLOCK(g_trace_ctx, D_NNLF, _buf.get(CompID::COMP_Cb).buf, _buf.get(CompID::COMP_Cb).stride,
                   inferArea.Cb().width, inferArea.Cb().height);
      DTRACE(g_trace_ctx, D_NNLF, "getPredBufCustom CR: poc = %d, area = (%d,%d) - (%d,%d) (size = %dx%d)\n\n",
             _pic.m_poc, inferArea.Cr().x, inferArea.Cr().y, inferArea.Cr().x + inferArea.Cr().width,
             inferArea.Cr().y + inferArea.Cr().height, inferArea.Cr().width, inferArea.Cr().height);
      DTRACE_BLOCK(g_trace_ctx, D_NNLF, _buf.get(CompID::COMP_Cr).buf, _buf.get(CompID::COMP_Cr).stride,
                   inferArea.Cr().width, inferArea.Cr().height);
    }

    _buf = _pic.getBlockQpBuf(inferArea);
    DTRACE(g_trace_ctx, D_NNLF, "getBlockQpBuf LUMA: poc = %d, area = (%d,%d) - (%d,%d) (size = %dx%d)\n\n", _pic.m_poc,
           inferArea.lx(), inferArea.ly(), inferArea.lx() + inferArea.Y().width, inferArea.ly() + inferArea.Y().height,
           inferArea.Y().width, inferArea.Y().height);
    DTRACE_BLOCK(g_trace_ctx, D_NNLF, _buf.get(CompID::COMP_Y).buf, _buf.get(CompID::COMP_Y).stride,
                 inferArea.Y().width, inferArea.Y().height);
    if (hasChroma)
    {
      DTRACE(g_trace_ctx, D_NNLF, "getBlockQpBuf CB: poc = %d, area = (%d,%d) - (%d,%d) (size = %dx%d)\n\n", _pic.m_poc,
             inferArea.Cb().x, inferArea.Cb().y, inferArea.Cb().x + inferArea.Cb().width,
             inferArea.Cb().y + inferArea.Cb().height, inferArea.Cb().width, inferArea.Cb().height);
      DTRACE_BLOCK(g_trace_ctx, D_NNLF, _buf.get(CompID::COMP_Cb).buf, _buf.get(CompID::COMP_Cb).stride,
                   inferArea.Cb().width, inferArea.Cb().height);
      DTRACE(g_trace_ctx, D_NNLF, "getBlockQpBuf CR: poc = %d, area = (%d,%d) - (%d,%d) (size = %dx%d)\n\n", _pic.m_poc,
             inferArea.Cr().x, inferArea.Cr().y, inferArea.Cr().x + inferArea.Cr().width,
             inferArea.Cr().y + inferArea.Cr().height, inferArea.Cr().width, inferArea.Cr().height);
      DTRACE_BLOCK(g_trace_ctx, D_NNLF, _buf.get(CompID::COMP_Cr).buf, _buf.get(CompID::COMP_Cr).stride,
                   inferArea.Cr().width, inferArea.Cr().height);
    }
  }
#endif

  const int    log2InputBitdepth = pic.m_cs->slice->clpRng(CompID::COMP_Y).bd;   // internal bitdepth
  const double inputScalePred    = (1 << log2InputBitdepth);
  const double inputScaleQp      = (1 << log2InputQpScale);
  const double inputScaleIpb     = (1 << log2InputIbpScale);

  std::vector<InputData> listInputData;
  listInputData.push_back(
    { NN_INPUT_REC, 0, inputScalePred, m_input_quantizer[0] - log2InputBitdepth, true, hasChroma });
  listInputData.push_back(
    { NN_INPUT_PRED, 1, inputScalePred, m_input_quantizer[1] - log2InputBitdepth, true, hasChroma });
  listInputData.push_back(
    { NN_INPUT_BS, 2, inputScalePred, m_input_quantizer[2] - log2InputBitdepth, true, hasChroma });
  listInputData.push_back(
    { NN_INPUT_GLOBAL_QP, 3, inputScaleQp, m_input_quantizer[3] - log2InputQpScale, true, false });
  listInputData.push_back(
    { NN_INPUT_LOCAL_QP_BLOCK, 4, inputScaleQp, m_input_quantizer[4] - log2InputQpScale, true, false });
  if (m_forceIntraType)
  {
    listInputData.push_back({ NN_INPUT_ZERO, 5, inputScaleIpb, m_input_quantizer[5] - log2InputIbpScale, true, false });
  }
  else
  {
    listInputData.push_back({ NN_INPUT_IPB, 5, inputScaleIpb, m_input_quantizer[5] - log2InputIbpScale, true, false });
  }
  if (m_nnlfTransInput)
  {
    NNInference::prepareTransInputs<TypeSadlLFUnified>(&pic, inferArea, m_inputs, seqQp, sliceQp, -1 /* sliceType */,
                                                       listInputData);
  }
  else
  {
    NNInference::prepareInputs<TypeSadlLFUnified>(&pic, inferArea, m_inputs, seqQp, sliceQp, -1 /* sliceType */,
                                                  listInputData);
  }

  if (m_inputs.size() == Input::nbInputs)
  {
    m_inputs[6](0, 0) = applyMultiplier ? 1 : 0;
    m_inputs[7](0, 0) = 1;
  }

  NNInference::infer<TypeSadlLFUnified>(model, m_inputs);

  UnitArea   inferAreaNoExt(inferArea.chromaFormat,
                            Area(inferArea.lx() + extLeft, inferArea.ly() + extTop,
                                 inferArea.lwidth() - extLeft - extRight, inferArea.lheight() - extTop - extBottom));
  UnitArea   inferAreaExt(inferArea.chromaFormat, Area(-extLeft, -extTop, inferArea.lwidth(), inferArea.lheight()));
  PelUnitBuf bufDst = m_scaled[0][prmId].getBuf(inferAreaNoExt).subBuf(inferAreaExt);

  if (m_nnlfTransInput)
  {
    extractOutputsDCT(pic, model, bufDst, inferArea, extLeft, extRight, extTop, extBottom);
  }
  else
  {
    extractOutputs(pic, model, bufDst, inferArea, extLeft, extRight, extTop, extBottom);
  }
}

void NNFilterUnified::filterBlockTemporal(Picture &pic, UnitArea inferArea, int extLeft, int extRight, int extTop,
                                          int extBottom, int prmId)
{
  // get model
  auto &model    = *m_modelTemporal;
  bool  inter    = pic.m_slices[0]->m_eSliceType != I_SLICE ? true : false;
  int   qpOffset = (inter ? prmId * 5 : prmId * 2) * (pic.m_slices[0]->m_uiTLayer >= 4 ? 1 : -1);
  int   seqQp    = pic.m_slices[0]->m_pps->m_picInitQPMinus26 + 26 + qpOffset;
  resizeInputsTemporal(inferArea.Y().width, inferArea.Y().height);

  const int    log2InputBitdepth = pic.m_cs->slice->clpRng(CompID::COMP_Y).bd;   // internal bitdepth
  const double inputScalePred    = (1 << log2InputBitdepth);
  const double inputScaleQp      = (1 << log2InputQpScale);

  std::vector<InputData> listInputData;
  listInputData.push_back(
    { NN_INPUT_REC, 0, inputScalePred, m_input_quantizer_temporal - log2InputBitdepth, true, false });
  listInputData.push_back(
    { NN_INPUT_PRED, 1, inputScalePred, m_input_quantizer_temporal - log2InputBitdepth, true, false });
  listInputData.push_back(
    { NN_INPUT_REF_LIST_0, 2, inputScalePred, m_input_quantizer_temporal - log2InputBitdepth, true, false });
  listInputData.push_back(
    { NN_INPUT_REF_LIST_1, 3, inputScalePred, m_input_quantizer_temporal - log2InputBitdepth, true, false });
  listInputData.push_back(
    { NN_INPUT_GLOBAL_QP, 4, inputScaleQp, m_input_quantizer_temporal - log2InputQpScale, true, false });

  NNInference::prepareInputs<TypeSadlLFUnified>(&pic, inferArea, m_inputsTemporal, seqQp, -1 /* localQp */,
                                                -1 /* sliceType */, listInputData);

  NNInference::infer<TypeSadlLFUnified>(model, m_inputsTemporal);

  PelUnitBuf bufDst = m_scaled[0][prmId].getBuf(inferArea);

  extractOutputsTemporal(pic, model, bufDst, inferArea, extLeft, extRight, extTop, extBottom);
}

void NNFilterUnified::filter(Picture &pic, bool applyMultiplier, const bool isDec /* true */)
{
  const CodingStructure &cs  = *pic.m_cs;
  const PreCalcValues   &pcv = *cs.pcv;
  const int              nc  = getNumberValidComponents(cs.sps->m_chromaFormatIdc);
  int                    cpt = 0;
  for (int y = 0; y < m_picprm->nb_blocks_height; ++y)
  {
    for (int x = 0; x < m_picprm->nb_blocks_width; ++x, ++cpt)
    {
      int prmId = m_picprm->prmId[cpt];

      if (prmId == -1)
      {
        continue;
      }

      int xPos   = x * m_picprm->block_size;
      int yPos   = y * m_picprm->block_size;
      int width  = (xPos + m_picprm->block_size > (int)pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : m_picprm->block_size;
      int height = (yPos + m_picprm->block_size > (int)pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : m_picprm->block_size;

      int extLeft   = m_picprm->extension;
      int extRight  = m_picprm->extension;
      int extTop    = m_picprm->extension;
      int extBottom = m_picprm->extension;

      int            extXPos   = xPos - extLeft;
      int            extYPos   = yPos - extTop;
      int            extWidth  = width + extLeft + extRight;
      int            extHeight = height + extTop + extBottom;
      const UnitArea inferArea(cs.area.chromaFormat, Area(extXPos, extYPos, extWidth, extHeight));
      if (m_picprm->temporal)
      {
        filterBlockTemporal(pic, inferArea, extLeft, extRight, extTop, extBottom, prmId);
      }
      else
      {
        filterBlock(pic, inferArea, extLeft, extRight, extTop, extBottom, prmId, applyMultiplier);
      }

      const UnitArea inferAreaNoExt(cs.area.chromaFormat, Area(xPos, yPos, width, height));
      PelUnitBuf     filteredBuf = getFilteredBuf(prmId, inferAreaNoExt);
      PelUnitBuf     scaledBuf   = getScaledBuf(0, prmId, inferAreaNoExt);
      PelUnitBuf     recBuf      = pic.getRecoBuf(inferAreaNoExt);

      roundToOutputBitdepth(scaledBuf, filteredBuf, cs.slice->m_clpRngs, cs.slice->m_sps->m_chromaFormatIdc);

      if (!isDec)
      {
        continue;
      }
      if (m_picprm->sprm.scaleId != -1)
      {
        int scaleId = m_picprm->sprm.scaleId;
        for (int compIdx = 0; compIdx < nc; compIdx++)
        {
          CompID compID = CompID(compIdx);
          scaleId == 0
            ? scaleResidualBlock(pic, compID, inferAreaNoExt, scaledBuf.get(compID), recBuf.get(compID),
                                 m_picprm->sprm.scale[compID][prmId], m_picprm->sprm.offset[compID][prmId][scaleId])
            : scaleResidualBlock(pic, compID, inferAreaNoExt, scaledBuf.get(compID), recBuf.get(compID),
                                 scale_candidates[scaleId], m_picprm->sprm.offset[compID][prmId][scaleId]);
        }
      }
      else
      {
        for (int compIdx = 0; compIdx < nc; compIdx++)
        {
          CompID compID = CompID(compIdx);
          scaleResidualBlock(pic, compID, inferAreaNoExt, scaledBuf.get(compID), filteredBuf.get(compID), 0,
                             m_picprm->sprm.offset[compID][prmId][3]);
        }
        recBuf.copyFrom(filteredBuf);
      }
    }
  }
}

void NNFilterUnified::scaleResidualBlock(Picture &pic, CompID compID, UnitArea inferAreaNoExt, CPelBuf src, PelBuf tgt,
                                         int scale, int roa_offset) const
{
  const Slice &slice         = *pic.m_cs->slice;
  const int    inputBitdepth = slice.clpRng(CompID::COMP_Y).bd;   // internal bitdepth
  const int    shift         = log2OutputScale - inputBitdepth;
  const int    shift2        = shift + log2ResidueScale;
  const int    offset        = (1 << shift2) / 2;
  CPelBuf      rec           = pic.getRecoBuf(inferAreaNoExt).get(compID);
  int          width         = inferAreaNoExt.lwidth();
  int          height        = inferAreaNoExt.lheight();
  CPelBuf      recBeforeDbf  = pic.getRecBeforeDbfBuf(inferAreaNoExt).get(compID);

  if (compID)
  {
    width  = width / 2;
    height = height / 2;
  }

  for (int y = 0; y < height; ++y)
  {
    for (int x = 0; x < width; ++x)
    {
      if (scale > 0)
      {
        // positive-, negative+
        int v = 0;
        if ((src.at(x, y) - (recBeforeDbf.at(x, y) << shift)) >= (roa_offset << shift))
        {
          v = (((int)rec.at(x, y) << shift2) +
               (src.at(x, y) - (rec.at(x, y) << shift) - (roa_offset << shift)) * scale + offset) >>
            shift2;
        }
        else if ((src.at(x, y) - (recBeforeDbf.at(x, y) << shift)) <= (-roa_offset << shift))
        {
          v = (((int)rec.at(x, y) << shift2) +
               (src.at(x, y) - (rec.at(x, y) << shift) + (roa_offset << shift)) * scale + offset) >>
            shift2;
        }
        else
        {
          v = (((int)rec.at(x, y) << shift2) + (src.at(x, y) - (rec.at(x, y) << shift)) * scale + offset) >> shift2;
        }
        tgt.at(x, y) = Pel(Clip3<int>(0, (1 << inputBitdepth) - 1, v));
      }
      else
      {
        if ((src.at(x, y) - (recBeforeDbf.at(x, y) << shift)) >= (roa_offset << shift))
        {
          tgt.at(x, y) = Pel(Clip3<int>(0, (1 << inputBitdepth) - 1, tgt.at(x, y) - roa_offset));
        }
        else if ((src.at(x, y) - (recBeforeDbf.at(x, y) << shift)) <= (-roa_offset << shift))
        {
          tgt.at(x, y) = Pel(Clip3<int>(0, (1 << inputBitdepth) - 1, tgt.at(x, y) + roa_offset));
        }
        else
        {
          tgt.at(x, y) = Pel(Clip3<int>(0, (1 << inputBitdepth) - 1, tgt.at(x, y)));
        }
      }
    }
  }
}

#endif
