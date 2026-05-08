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

/** \file     DecAppCfg.cpp
    \brief    Decoder configuration class
*/

#include <cstdio>
#include <cstring>
#include <string>
#include "DecAppCfg.h"
#include "Utilities/program_options_lite.h"
#include "Utilities/VideoIOYuv.h"
#include "CommonLib/CodingStatistics.h"
#include "CommonLib/ChromaFormat.h"
#include "CommonLib/dtrace_next.h"

namespace po = df::program_options_lite;

#if ENABLE_CABAC_DUMP
namespace CabacRetrain
{
extern void init(const std::string &fn, bool activate);
}
#endif

//! \ingroup DecoderApp
//! \{

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/** \param argc number of arguments
    \param argv array of arguments
 */

bool DecAppCfg::parseCfg(int argc, char *argv[], DecCfg *decCfg)
{
  bool        do_help                     = false;
  std::string cfg_TargetDecLayerIdSetFile = "";
  std::string outputColourSpaceConvert    = "";
  int         warnUnknowParameter         = 0;
#if ENABLE_TRACING
  std::string sTracingRule         = "";
  std::string sTracingFile         = "";
  bool        bTracingChannelsList = false;
#endif
#if ENABLE_SIMD_OPT
  std::string ignore = "";
#endif
  po::Options opts;

  // clang-format off
  opts.addOptions()
  ("help",                          do_help,                                 "this help text")
  ("BitstreamFile,b",               decCfg->m_bitstreamFileName,             "bitstream input file name")
  ("ReconFile,o",                   decCfg->m_reconFileName,                 "reconstructed YUV output file name\n")
  ("OplFile,-opl",                  decCfg->m_oplFilename,                   "opl-file name without extension for conformance testing\n")
#if ENABLE_SIMD_OPT                                                      
  ("SIMD",                          ignore,                                  "SIMD extension to use (SCALAR, SSE41, SSE42, AVX, AVX2, AVX512), default: the highest supported extension\n")
#endif
#if ENABLE_CABAC_DUMP
  ("ActivateCABACDumping",          decCfg->m_activateDump,                  "If true dump cabac bins in file")
#endif
  ("WarnUnknowParameter,w",         warnUnknowParameter,                     "warn for unknown configuration parameters instead of failing")
  ("SkipFrames,s",                  decCfg->m_iSkipFrame,                            "number of frames to skip before random access")
  ("OutputBitDepth,d",              decCfg->m_outputBitDepth[ChannelType::LUMA],     "bit depth of YUV output luma component (default: use 0 for native depth)")
  ("OutputBitDepthC,d",             decCfg->m_outputBitDepth[ChannelType::CHROMA],   "bit depth of YUV output chroma component (default: use luma output bit-depth)")
  ("OutputColourSpaceConvert",      outputColourSpaceConvert,                "Colour space conversion to apply to input 444 video. Permitted values are (empty string=UNCHANGED) " + getListOfColourSpaceConverts(false))
  ("MaxTemporalLayer,t",            decCfg->m_iMaxTemporalLayer,                     "Maximum Temporal Layer to be decoded. -1 to decode all layers")
  ("TargetOutputLayerSet,p",        decCfg->m_targetOlsIdx,                          "Target output layer set index")
#if JVET_Z0120_SII_SEI_PROCESSING
  ("SEIShutterIntervalPostFilename,-sii", decCfg->m_shutterIntervalPostFileName, "Post Filtering with Shutter Interval SEI. If empty, no filtering is applied (ignore SEI message)\n")
#endif
  ("SEIDecodedPictureHash,-dph",    decCfg->m_decodedPictureHashSEIEnabled,         "Control handling of decoded picture hash SEI messages\n"
                                                                         "\t2-4: generate hash info - no comparison"
                                                                         "\t1: check hash in SEI messages if available in the bitstream\n"
                                                                         "\t0: ignore SEI message")
  ("SEINoDisplay",                  decCfg->m_decodedNoDisplaySEIEnabled,            "Control handling of decoded no display SEI messages")
  ("TarDecLayerIdSetFile,l",        cfg_TargetDecLayerIdSetFile,             "targetDecLayerIdSet file name. The file should include white space separated LayerId values to be decoded. Omitting the option or a value of -1 in the file decodes all layers.")
  ("SEIColourRemappingInfoFilename", decCfg->m_colourRemapSEIFileName,           "Colour Remapping YUV output file name. If empty, no remapping is applied (ignore SEI message)\n")
  ("SEICTIFilename",                decCfg->m_SEICTIFileName,                        "CTI YUV output file name. If empty, no Colour Transform is applied (ignore SEI message)\n")
  ("SEIFGSFilename",                decCfg->m_SEIFGSFileName,                        "FGS YUV output file name. If empty, no film grain is applied (ignore SEI message)\n")
  ("SEIAnnotatedRegionsInfoFilename", decCfg->m_annotatedRegionsSEIFileName,     "Annotated regions output file name. If empty, no object information will be saved (ignore SEI message)\n")
  ("OutputDecodedSEIMessagesFilename", decCfg->m_outputDecodedSEIMessagesFilename, "When non empty, output decoded SEI messages to the indicated file. If file is '-', then output to stdout\n")
#if JVET_S0257_DUMP_360SEI_MESSAGE
  ("360DumpFile",                   decCfg->m_outputDecoded360SEIMessagesFilename,   "When non empty, output decoded 360 SEI messages to the indicated file.\n")
#endif
  ("ClipOutputVideoToRec709Range",  decCfg->m_clipOutputVideoToRec709Range,       "If true then clip output video to the Rec. 709 Range on saving")
  ("PYUV",                          decCfg->m_packedYUVMode,                         "If true then output 10-bit and 12-bit YUV data as 5-byte and 3-byte (respectively) packed YUV data. Ignored for interlaced output.")
#if ENABLE_TRACING
  ("TraceChannelsList",             bTracingChannelsList,                    "List all available tracing channels")
  ("TraceRule",                     sTracingRule,                            "Tracing rule (ex: \"D_CABAC:poc==8\" or \"D_REC_CB_LUMA:poc==8\")")
  ("TraceFile",                     sTracingFile,                            "Tracing file")
#endif
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  ("CacheCfg",                      m_cacheCfgFile,                          "CacheCfg File")
#endif
#if RExt__DECODER_DEBUG_STATISTICS
  ("Stats",                         decCfg->m_statMode,                      "Control decoder debugging statistic output mode\n"
                                                                         "\t0: disable statistic\n"
                                                                         "\t1: enable bit statistic\n"
                                                                         "\t2: enable tool statistic\n"
                                                                         "\t3: enable bit and tool statistic\n")
#endif
  ("MCTSCheck",                     decCfg->m_mctsCheck,                             "If enabled, the decoder checks for violations of mc_exact_sample_value_match_flag in Temporal MCTS ")
  ("targetSubPicIdx",               decCfg->m_targetSubPicIdx,                       "Specify which subpicture shall be written to output, using subpic index, 0: disabled, subpicIdx=m_targetSubPicIdx-1 \n" )
  ( "UpscaledOutput",               decCfg->m_upscaledOutput,                        "Upscaled output for RPR" )
  ("UpscaleFilterForDisplay",       decCfg->m_upscaleFilterForDisplay,               "Filters used for upscaling reconstruction to full resolution (2: ECM 12 - tap luma and 6 - tap chroma MC filters, 1 : Alternative 12 - tap luma and 6 - tap chroma filters, 0 : VVC 8 - tap luma and 4 - tap chroma MC filters)")
#if ENABLE_NNLF
  ("NnlfModelName",                 decCfg->m_nnlfModelName,                 "NNLF model file name (leave empty for default model)")
  ("NnlfDebugOption",               decCfg->m_nnlfDebugOption,               "NNLF debug option: 0: default, 1: apply only on I slice, 2: apply on all slices using I type as input")
  ("NnlfDumpBasename",              decCfg->m_nnlfDumpBasename,              "Basename for NNLF data dumping\n")
#endif
    ;
  // clang-format on

  po::ErrorReporter              err;
  const std::list<const char *> &argv_unhandled = po::scanArgv(opts, argc, (const char **)argv, err);

  for (std::list<const char *>::const_iterator it = argv_unhandled.begin(); it != argv_unhandled.end(); it++)
  {
    msg(ERROR, "Unhandled argument ignored: `%s'\n", *it);
  }

  if (argc == 1 || do_help)
  {
    po::doHelp(std::cout, opts);
    return false;
  }

  if (err.is_errored)
  {
    if (!warnUnknowParameter)
    {
      /* errors have already been reported to stderr */
      return false;
    }
  }

#if ENABLE_TRACING
  g_trace_ctx = tracing_init(sTracingFile, sTracingRule);
  if (bTracingChannelsList && g_trace_ctx)
  {
    std::string sChannelsList;
    g_trace_ctx->getChannelsList(sChannelsList);
    msg(INFO, "\nAvailable tracing channels:\n\n%s\n", sChannelsList.c_str());
  }
#endif

  g_mctsDecCheckEnabled = decCfg->m_mctsCheck;
  // Chroma output bit-depth
  if (decCfg->m_outputBitDepth[ChannelType::LUMA] != 0 && decCfg->m_outputBitDepth[ChannelType::CHROMA] == 0)
  {
    decCfg->m_outputBitDepth[ChannelType::CHROMA] = decCfg->m_outputBitDepth[ChannelType::LUMA];
  }

  decCfg->m_outputColourSpaceConvert = stringToInputColourSpaceConvert(outputColourSpaceConvert, false);
  if (decCfg->m_outputColourSpaceConvert >= NUMBER_INPUT_COLOUR_SPACE_CONVERSIONS)
  {
    msg(ERROR, "Bad output colour space conversion string\n");
    return false;
  }

  if (decCfg->m_bitstreamFileName.empty())
  {
    msg(ERROR, "No input file specified, aborting\n");
    return false;
  }

  if (!cfg_TargetDecLayerIdSetFile.empty())
  {
    FILE *targetDecLayerIdSetFile = fopen(cfg_TargetDecLayerIdSetFile.c_str(), "r");
    if (targetDecLayerIdSetFile)
    {
      bool isLayerIdZeroIncluded = false;
      while (!feof(targetDecLayerIdSetFile))
      {
        int layerIdParsed = 0;
        if (fscanf(targetDecLayerIdSetFile, "%d ", &layerIdParsed) != 1)
        {
          if (decCfg->m_targetDecLayerIdSet.size() == 0)
          {
            msg(ERROR, "No LayerId could be parsed in file %s. Decoding all LayerIds as default.\n",
                cfg_TargetDecLayerIdSetFile.c_str());
          }
          break;
        }
        if (layerIdParsed == -1) // The file includes a -1, which means all LayerIds are to be decoded.
        {
          decCfg->m_targetDecLayerIdSet.clear(); // Empty set means decoding all layers.
          break;
        }
        if (layerIdParsed < 0 || layerIdParsed >= MAX_NUM_LAYER_IDS)
        {
          msg(ERROR, "Warning! Parsed LayerId %d is not within allowed range [0,%d]. Ignoring this value.\n",
              layerIdParsed, MAX_NUM_LAYER_IDS - 1);
        }
        else
        {
          isLayerIdZeroIncluded = layerIdParsed == 0 ? true : isLayerIdZeroIncluded;
          decCfg->m_targetDecLayerIdSet.push_back(layerIdParsed);
        }
      }
      fclose(targetDecLayerIdSetFile);
      if (decCfg->m_targetDecLayerIdSet.size() > 0 && !isLayerIdZeroIncluded)
      {
        msg(ERROR, "TargetDecLayerIdSet must contain LayerId=0, aborting");
        return false;
      }
    }
    else
    {
      msg(ERROR, "File %s could not be opened. Using all LayerIds as default.\n", cfg_TargetDecLayerIdSetFile.c_str());
    }
  }
  if (decCfg->m_iMaxTemporalLayer != 500)
  {
    decCfg->m_mTidExternalSet = true;
  }
  else
  {
    decCfg->m_iMaxTemporalLayer = -1;
  }
  if (decCfg->m_targetOlsIdx != 500)
  {
    decCfg->m_tOlsIdxTidExternalSet = true;
  }
  else
  {
    decCfg->m_targetOlsIdx = -1;
  }

#if ENABLE_CABAC_DUMP
  CabacRetrain::init(decCfg->m_bitstreamFileName, decCfg->m_activateDump);
#endif

  return true;
}

DecAppCfg::DecAppCfg() {}

DecAppCfg::~DecAppCfg() {}

//! \}
