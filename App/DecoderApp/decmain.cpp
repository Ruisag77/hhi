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

/** \file     decmain.cpp
    \brief    Decoder application main
*/

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "DecApp.h"
#include "program_options_lite.h"

//! \ingroup DecoderApp
//! \{

#if JVET_Z0150_MEMORY_USAGE_PRINT
#ifdef __linux
#include <cstdlib>
#include <cstdio>
#include <cstring>

int getProcStatusValue(const char *key)
{
  FILE *file   = fopen("/proc/self/status", "r");
  int   result = -1;
  char  line[128];

  size_t len = strlen(key);
  while (fgets(line, 128, file) != nullptr)
  {
    if (strncmp(line, key, len) == 0)
    {
      result = atoi(line + len);
      break;
    }
  }
  fclose(file);
  return result;
}
#endif
#endif

// ====================================================================================================================
// Main function
// ====================================================================================================================

#if defined(_DEBUG)
#define ENABLE_TRY_CATCH 0
#else
#define ENABLE_TRY_CATCH 1
#endif

int main(int argc, char *argv[])
{
  int returnCode = EXIT_SUCCESS;

  // print information
  fprintf(stdout, "\n");
  fprintf(stdout, "NextSoftware2: Decoder Version %s ", NX2_VERSION);
  fprintf(stdout, NVM_ONOS);
  fprintf(stdout, NVM_COMPILEDBY);
  fprintf(stdout, NVM_BITS);
#if ENABLE_SIMD_OPT
  std::string                       SIMD = "";
  df::program_options_lite::Options optsSimd;
  optsSimd.addOptions()("SIMD", SIMD, "");
  df::program_options_lite::SilentReporter err;
  df::program_options_lite::scanArgv(optsSimd, argc, (const char **)argv, err);
  fprintf(stdout, "[SIMD=%s] ", read_x86_extension(SIMD));
#endif
#if ENABLE_TRACING
  fprintf(stdout, "[ENABLE_TRACING] ");
#endif
  fprintf(stdout, "\n");

  DecApp *pcDecApp = new DecApp;
  // parse configuration
  if (!pcDecApp->parseCfg(argc, argv))
  {
    returnCode = EXIT_FAILURE;

    delete pcDecApp;

    return returnCode;
  }

  // starting time
  double  dResult;
  clock_t lBefore = clock();

  // call decoding function
#if ENABLE_TRY_CATCH
  try
  {
#endif
    if (0 != pcDecApp->decode())
    {
      printf("\n\n***ERROR*** A decoding mismatch occured: signalled md5sum does not match\n");
      returnCode = EXIT_FAILURE;
    }
#if ENABLE_TRY_CATCH
  }
  catch (Exception &e)
  {
    std::cerr << e.what() << std::endl;
    returnCode = EXIT_FAILURE;
  }
  catch (const std::bad_alloc &e)
  {
    std::cout << "Memory allocation failed: " << e.what() << std::endl;
    returnCode = EXIT_FAILURE;
  }
#endif

#if JVET_Z0150_MEMORY_USAGE_PRINT
#ifdef __linux
  int vm = getProcStatusValue("VmPeak:");
  int rm = getProcStatusValue("VmHWM:");
  printf("\nMemory Usage: VmPeak= %d KB ( %.1f GiB ),  VmHWM= %d KB ( %.1f GiB )\n", vm, (double)vm / (1024 * 1024), rm,
         (double)rm / (1024 * 1024));
#endif
#endif

  // ending time
  dResult = (double)(clock() - lBefore) / CLOCKS_PER_SEC;
  printf("\n Total Time: %12.3f sec.\n", dResult);

  delete pcDecApp;

  return returnCode;
}

//! \}
