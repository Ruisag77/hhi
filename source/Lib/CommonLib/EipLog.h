#pragma once

#include <string>

namespace EipLog
{
void initEncoder(const std::string &inputFile, const std::string &bitstreamFile, int qp);
void initDecoder(const std::string &bitstreamFile);
void finishEncoder();
void finishDecoder();

void recordEncoderSelectedEip();
void recordDecoderFinalEip(bool hasResidual);
void recordImplicitMts(bool encoder, int poc, int x, int y, int width, int height, unsigned cand,
                       unsigned inferredMode, int oldHor, int oldVer, int newHor, int newVer);
}
