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

/** \file     TAppDecLib.h
    \brief    Decoder application class (header)
*/

#ifndef __DECAPP__
#define __DECAPP__

#pragma once

#include "Utilities/VideoIOYuv.h"
#include "CommonLib/Picture.h"
#include "DecoderLib/DecLib.h"

//! \ingroup DecoderApp
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// decoder application class
class DecApp
{
private:
  // stuff read from config but modified by the decoding process
  int              m_iMaxTemporalLayer;
  std::vector<int> m_targetOutputLayerIdSet {};      // set of LayerIds to be outputted
  std::vector<int>
    m_targetDecLayerIdSet {};      // set of LayerIds to be included in the sub-bitstream extraction process.

  // class interface
  DecLib                              m_cDecLib;                     ///< decoder class
  std::unordered_map<int, VideoIOYuv> m_cVideoIOYuvReconFile;        ///< reconstruction YUV class
  std::unordered_map<int, VideoIOYuv> m_videoIOYuvSEIFGSFile;       ///< reconstruction YUV with FGS class
  std::unordered_map<int, VideoIOYuv> m_cVideoIOYuvSEICTIFile;       ///< reconstruction YUV with CTI class

#if JVET_Z0120_SII_SEI_PROCESSING
  bool       m_ShutterFilterEnable;          ///< enable Post-processing with Shutter Interval SEI
  VideoIOYuv m_cTVideoIOYuvSIIPostFile;      ///< post-filtered YUV class
  int        m_SII_BlendingRatio;

  struct IdrSiiInfo
  {
    SEIShutterIntervalInfo m_siiInfo;
    uint32_t               m_picPoc;
    bool                   m_isValidSii;
  };

  std::map<uint32_t, IdrSiiInfo> m_activeSiiInfo;

#endif

  // for output control
  int           m_iPOCLastDisplay;              ///< last POC in display order
  std::ofstream m_seiMessageFileStream;         ///< Used for outputing SEI messages.

  std::ofstream m_oplFileStream;                ///< Used to output log file for confomance testing

  bool m_newCLVS[MAX_NUM_LAYER_IDS];   ///< used to record a new CLVSS

  SEIAnnotatedRegions::AnnotatedRegionHeader                     m_arHeader; ///< AR header
  std::map<uint32_t, SEIAnnotatedRegions::AnnotatedRegionObject> m_arObjects; ///< AR object pool
  std::map<uint32_t, std::string>                                m_arLabels; ///< AR label pool

#if ENABLE_NNLF
  const std::unordered_map<int, std::string> m_nnlfBufTypeMap = {
    { PIC_BS_MAP, "_bs" },
    { PIC_PREDICTION_CUSTOM, "_pred" },
    { PIC_REC_BEFORE_DBF, "_rec_before_dbf" },
    { PIC_BLOCK_PRED_MODE, "_bpm" },
    { PIC_BLOCK_QP, "_qp" },
  };
  std::ofstream m_nnlfJsonFile;
  VideoIOYuv   *m_nnlfDumpIoFiles[NUM_PIC_TYPES];
  int           m_nnlfDumpCount;

  void xNnlfDumpPicData(Picture &pic);
#endif

private:
  bool xIsNaluWithinTargetDecLayerIdSet(
    const InputNALUnit *nalu) const; ///< check whether given Nalu is within targetDecLayerIdSet
  bool xIsNaluWithinTargetOutputLayerIdSet(
    const InputNALUnit *nalu) const; ///< check whether given Nalu is within targetOutputLayerIdSet

public:
  DecApp();
  virtual ~DecApp();

  bool parseCfg(int argc, char *argv[]);

  uint32_t decode(); ///< main decoding function
#if JVET_Z0120_SII_SEI_PROCESSING
  int  getBlendingRatio() const { return m_SII_BlendingRatio; }
  void setBlendingRatio(int value) { m_SII_BlendingRatio = value; }
#endif

private:
  void xCreateDecLib(); ///< create internal classes
  void xDestroyDecLib(); ///< destroy internal classes
  void xWriteOutput(PicList *picList, uint32_t tId); ///< write YUV to file
  void xFlushOutput(PicList  *picList,
                    const int layerId = NOT_VALID); ///< flush all remaining decoded pictures to file

  // check if next NAL unit will be the first NAL unit from a new picture
  bool isNewPicture(std::ifstream *bitstreamFile, class InputByteStream *bytestream);

  // check if next NAL unit will be the first NAL unit from a new access unit
  bool isNewAccessUnit(bool newPicture, std::ifstream *bitstreamFile, class InputByteStream *bytestream);

  void writeLineToOutputLog(Picture *pic);
  void xOutputAnnotatedRegions(PicList *picList);
};

//! \}

#endif // __DECAPP__
