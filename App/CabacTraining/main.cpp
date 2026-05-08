#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>

#include "program_options_lite.h"
#include "Contexts.h"
#include "io.h"
#include "optimization.h"
#include "simulator.h"

using namespace std;
namespace po             = df::program_options_lite;
static constexpr int CNU = 35;

int main(int argc, char **argv)
{
  // parsing
  bool        doHelp       = false;
  bool        keepModel    = true;
  std::string outfile      = "cabac_prm.txt";
  std::string inputFile    = "";
  std::string sliceToTrain = "all";

  po::Options               opts;
  // clang-format off
  opts.addOptions()
    ( "help", doHelp, "optimize cabac contexts. Usage:\ncat *.cabac_xxx | cabacTrain --output fileout.txt" )
    ( "keepModel", keepModel, "if no bin, keep old parameters instead of putting default parameters" )
    ( "sliceToTrain", sliceToTrain, "all or B/P/I/L")
    ( "output", outfile, "filename of the resulting parameters" )
    ( "input", inputFile, "filename of the input bins" )
    ;
  // clang-format on
  po::ErrorReporter         err;
  const list<const char *> &argvUnhandled = po::scanArgv(opts, argc, (const char **)argv, err);

  for (list<const char *>::const_iterator it = argvUnhandled.begin(); it != argvUnhandled.end(); it++)
  {
    msg(ERROR, "Unhandled argument ignored: `%s'\n", *it);
  }

  if (doHelp)
  {
    po::doHelp(cout, opts);
    return -1;
  }

  if (err.is_errored)
  {
    return -1;
  }

  SliceType toTrain = NUMBER_OF_SLICE_TYPES;

  if (sliceToTrain == "B")
  {
    toTrain = B_SLICE;
  }
  else if (sliceToTrain == "P")
  {
    toTrain = P_SLICE;
  }
  else if (sliceToTrain == "I")
  {
    toTrain = I_SLICE;
  }
  else if (sliceToTrain == "all")
  {
    toTrain = NUMBER_OF_SLICE_TYPES;
  }
  else if (sliceToTrain == "L")
  {
    toTrain = L_SLICE;
  }
  else
  {
    std::cerr << "[ERROR] unknown slice to train" << endl;
  }

  DataDb db;

  if (inputFile != "")
  {
    ifstream in(inputFile.c_str());
    db = loadDataFrame(in);
  }
  else
  {
    db = loadDataFrame();
  }

  std::vector<SliceType>      toOpt           = { B_SLICE, P_SLICE, I_SLICE, L_SLICE };
  std::array<bool, kNbModels> sliceActivation = {};

  if (db.ctxidx < 0)
  {
    std::cout << "[ERROR] no context to train found" << std::endl;
    return -1;
  }
  std::cout << "Ctx " << db.ctxidx << std::endl;
  bool hasBin = false;
  for (int k = 0; k < (int)toOpt.size(); ++k)
  {
    SliceType st       = toOpt[k];
    sliceActivation[k] = hasBin1Db(db.v, st);
    hasBin             = true;
  }

  Models modelsgreedy = db.modelsBPI;

  std::cout << "Before:\n";
  print(db.modelsBPI);
  std::cout << std::endl;
  if (hasBin)
  {
    for (int k = 0; k < (int)toOpt.size(); ++k)
    {
      SliceType st = toOpt[k];
      if (sliceActivation[k])
      {
        if (toTrain == NUMBER_OF_SLICE_TYPES || k == toTrain)
        {
          std::cout << "Optimized parameter slice type " << k << std::endl;
          modelsgreedy[(int)st] = getBestGreedy(db, st);
          db.modelsBPI[(int)st] = modelsgreedy[(int)st];

          std::cout << "Optimized rate offsets " << std::endl;
          auto drate = getBestGreedyDrate(db, st);

          db.modelsBPI[k].rateoffset[0] = drate[0];
          db.modelsBPI[k].rateoffset[1] = drate[1];
          modelsgreedy[k].rateoffset[0] = drate[0];
          modelsgreedy[k].rateoffset[1] = drate[1];
        }
      }
      else
      {
        if (keepModel)
        {
          // do not change the prm
        }
        else
        {
          std::cout << "Default model (no bin) for slice type " << k << std::endl;
          modelsgreedy[k].initId = db.modelsBPI[(int)st].initId = CNU;
          modelsgreedy[k].log2windowsize = db.modelsBPI[(int)st].log2windowsize = DWS;
          modelsgreedy[k].adaptweight = db.modelsBPI[(int)st].adaptweight = DWE;
          modelsgreedy[k].rateoffset[0]                                   = DWO;
          modelsgreedy[k].rateoffset[1]                                   = DWO;
        }
      }
    }
  }
  else
  {
    if (keepModel)
    {
      std::cout << "Skipped (no bin)" << std::endl;
    }
    else
    {
      std::cout << "Default model (no bin)" << std::endl;
      for (int k = 0; k < kNbModels; ++k)
      {
        modelsgreedy[k].initId         = CNU;
        modelsgreedy[k].log2windowsize = DWS;
        modelsgreedy[k].adaptweight    = DWE;
        modelsgreedy[k].rateoffset[0]  = DWO;
        modelsgreedy[k].rateoffset[1]  = DWO;
      }
    }
  }
  std::cout << "\nAfter:\n";
  print(modelsgreedy);

  if (!outfile.empty())
  {
    ofstream file(outfile, ios::binary);
    file << db.ctxidx << " A ";
    for (int i = 0; i < kNbModels; ++i)
    {
      file << modelsgreedy[i].initId << ' ';
    }
    for (int i = 0; i < kNbModels; ++i)
    {
      file << modelsgreedy[i].log2windowsize << ' ';
    }
    for (int i = 0; i < kNbModels; ++i)
    {
      file << modelsgreedy[i].adaptweight << ' ';
    }
    const int nbRateOffset = kNbModels;
    for (int i = 0; i < nbRateOffset; ++i)
    {
      file << modelsgreedy[i].rateoffset[0] << ' ' << modelsgreedy[i].rateoffset[1] << ' ';
    }
    file << std::endl;
  }
}
