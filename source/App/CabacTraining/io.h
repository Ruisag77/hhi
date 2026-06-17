#pragma once
#include <iostream>
#include <vector>
#include "Contexts.h"
#include "data.h"
using namespace std;

extern int verbose;

static Models readModelParameters(std::istream &in)
{
  constexpr int nb_model_to_read = kNbModels;
  Models        prms;
  assert(nb_model_to_read <= (int)prms.size());
  for (int k = 0; k < nb_model_to_read; ++k)
  {
    in >> prms[k].initId;
  }
  for (int k = 0; k < nb_model_to_read; ++k)
  {
    in >> prms[k].log2windowsize;
  }
  for (int k = 0; k < nb_model_to_read; ++k)
  {
    in >> prms[k].adaptweight;
  }
  in >> prms[0].rateoffset[0] >> prms[0].rateoffset[1];
  in >> prms[1].rateoffset[0] >> prms[1].rateoffset[1];
  in >> prms[2].rateoffset[0] >> prms[2].rateoffset[1];
  in >> prms[3].rateoffset[0] >> prms[3].rateoffset[1];
  in.ignore(1024, '\n');
  return prms;
}

static DataDb addDataFrame(istream &in, DataDb &db)
{
  char c = (char)in.peek();

  while (in && c == 'c')
  {
    // loop seq
    std::string ctx;
    in >> ctx;
    if (ctx != "ctx")
    {
      std::cerr << "[ERROR] invalid cabac file" << std::endl;
      exit(-1);
    }
    int ctxidx;
    in >> ctxidx;

    if (db.ctxidx == -1)
    {
      db.ctxidx = ctxidx;
      in.ignore(1024, '\n');
      db.modelsBPI = readModelParameters(in);
    }
    else if (ctxidx != db.ctxidx)
    {
      std::cerr << "[ERROR] mixinf different ctx idx" << std::endl;
      exit(-1);
    }
    else
    {
      in.ignore(1024, '\n');
      readModelParameters(in);
    }

    DataSequence v;
    // in.ignore(1024, '\n');
    c = (char)in.peek();

    while (c == '#')
    {
      in >> c;
      std::string ss;
      in >> ss;
      if (ss == "SIZE")
      {
        in >> v.filesize;
      }
      in.ignore(1024, '\n');
      c = (char)in.peek();
    }

    DataFrame d;
    char      slicetype;
    char      reportslice;
    while (c != 'c' &&
           in >> d.poc >> slicetype >> d.qp >> d.switchBp >> d.tempCABAC >> reportslice >> d.p0 >> d.p1 >> d.rate >>
             d.weight >> d.drate0 >> d.drate1)
    {
      // loop frames
      switch (slicetype)
      {
      case 'B':
        d.type = B_SLICE;
        break;
      case 'P':
        d.type = P_SLICE;
        break;
      case 'I':
        d.type = I_SLICE;
        break;
      case 'L':
        d.type = L_SLICE;
        break;
      default:
        std::cerr << "[ERROR] unkwon slice type" << std::endl;
        exit(-1);
      }

      switch (reportslice)
      {
      case 'B':
        d.reportslice = B_SLICE;
        break;
      case 'P':
        d.reportslice = P_SLICE;
        break;
      case 'I':
        d.reportslice = I_SLICE;
        break;
      case 'L':
        d.reportslice = L_SLICE;
        break;
      default:
        std::cerr << "[ERROR] unkwon slice type" << std::endl;
        exit(-1);
      }
      c = (char)in.get(); // get space
      c = (char)in.peek();

      if (c != '\n')
      {
        std::string s;
        in >> s;
        d.bins.resize(s.size());
        for (int k = 0; k < (int)s.size(); ++k)
        {
          d.bins[k] = (s[k] == '1');
        }
        v.v.push_back(std::move(d));
      }
      in.ignore(1024, '\n');
      c = (char)in.peek();
    }

    if (!v.v.empty())
    {
      db.v.push_back(std::move(v));
    }
  }
  return db;
}

static DataDb loadDataFrame(std::ifstream &file)
{
  DataDb db;
  return addDataFrame(file, db);
}

static DataDb loadDataFrame()
{
  DataDb db;
  return addDataFrame(std::cin, db);
}

static void print(const Models &m)
{
  auto printField = [](const Models &m, int ModelParameters::*pi)
  {
    for (int i = 0; i < (int)m.size(); ++i)
    {
      std::cout << m[i].*pi << '\n';
    }
  };
  printField(m, &ModelParameters::initId);
  printField(m, &ModelParameters::log2windowsize);
  printField(m, &ModelParameters::adaptweight);
  const int nbRateOffset = (int)m.size();
  for (int i = 0; i < nbRateOffset; ++i)
  {
    std::cout << m[i].rateoffset[0] << '\n';
    std::cout << m[i].rateoffset[1] << '\n';
  }
}
