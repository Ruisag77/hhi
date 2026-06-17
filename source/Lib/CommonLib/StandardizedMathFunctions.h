/** \file     StandardizedMathFunctions.h
    \brief    Defines math functions related to section "Conventions" sub-section "Mathematical functions" of the
   Standard
*/

#ifndef __STANDARDIZEDMATHFUNCTIONS__
#define __STANDARDIZEDMATHFUNCTIONS__

#include <cstdint>
#include <tuple>

//! \ingroup CommonLib
//! \{

template<typename T> inline constexpr int sgn(const T val)
{
  return (T(0) < val ? 1 : 0) - (val < T(0) ? 1 : 0);
}  // return -1 for val < 0, 0 for val == 0, and 1 for val > 0
template<typename T> inline constexpr int sgn2(const T val)
{
  return val >= T(0) ? 1 : -1;
}  // return -1 for val < 0, and 1 for val >= 0
template<typename T> inline constexpr bool isPowerOf2(const T val) { return (val & (val - 1)) == 0; }
template<typename T> inline constexpr T    leftShift(const T value, const int shift)
{
  return (shift >= 0) ? (value << shift) : (value >> -shift);
}
template<typename T> inline constexpr T rightShift(const T value, const int shift)
{
  return (shift >= 0) ? (value >> shift) : (value << -shift);
}
template<typename T> inline constexpr T leftShift_round(const T value, const int shift)
{
  return (shift >= 0) ? (value << shift) : ((value + (T(1) << (-shift - 1))) >> -shift);
}
template<typename T> inline constexpr T rightShift_round(const T value, const int shift)
{
  return (shift > 0) ? ((value + (T(1) << (shift - 1))) >> shift) : (value << -shift);
}

static inline int floorLog2(uint32_t x)
{
  if (x == 0)
  {
    // note: ceilLog2() expects -1 as return value
    return -1;
  }
#ifdef __GNUC__
  return 31 - __builtin_clz(x);
#else
#ifdef _MSC_VER
  unsigned long r = 0;
  _BitScanReverse(&r, x);
  return r;
#else
  int result = 0;
  if (x & 0xffff0000)
  {
    x >>= 16;
    result += 16;
  }
  if (x & 0xff00)
  {
    x >>= 8;
    result += 8;
  }
  if (x & 0xf0)
  {
    x >>= 4;
    result += 4;
  }
  if (x & 0xc)
  {
    x >>= 2;
    result += 2;
  }
  if (x & 0x2)
  {
    x >>= 1;
    result += 1;
  }
  return result;
#endif
#endif
}

static inline int floorLog2Uint64(uint64_t x)
{
  if (x == 0)
  {
    // note: ceilLog2() expects -1 as return value
    return -1;
  }
  int result = 0;
  if (x & 0xffffffff00000000)
  {
    x >>= 32;
    result += 32;
  }
  if (x & 0xffff0000)
  {
    x >>= 16;
    result += 16;
  }
  if (x & 0xff00)
  {
    x >>= 8;
    result += 8;
  }
  if (x & 0xf0)
  {
    x >>= 4;
    result += 4;
  }
  if (x & 0xc)
  {
    x >>= 2;
    result += 2;
  }
  if (x & 0x2)
  {
    x >>= 1;
    result += 1;
  }
  return result;
}

static inline int ceilLog2(uint32_t x) { return (x == 0) ? -1 : floorLog2(x - 1) + 1; }

static inline int getMSB(unsigned x)
{
#ifdef __GNUC__
  return (sizeof(int) << 3) - __builtin_clz(x | 1);
#else
#ifdef _MSC_VER
  unsigned long r = 0;
  _BitScanReverse(&r, x | 1);
  int msb = r + 1;
  return msb;
#else
  int msb = 0, bits = (sizeof(int) << 3), y = 1;
  while (x > 1u)
  {
    bits >>= 1;
    y = x >> bits;
    if (y)
    {
      x = y;
      msb += bits;
    }
  }
  msb += y;
  return msb;
#endif
#endif
}

inline std::tuple<int, int, int>
  lutDivideGetScaleRoundShift(const int64_t denom, const int decimalBits) // Note: assumes positive denominator
{
  static const int DIV_PREC_BITS_POW2 = 8;
  static const int DIV_SLOT_BITS      = 3;
  static const int DIV_PREC_BITS      = 14;
  static const int DIV_INTR_BITS      = (DIV_PREC_BITS - DIV_SLOT_BITS);

  int              scale, round, shift;
  static const int pow2W[8] = { 214, 153, 113, 86, 67, 53, 43, 35 }; // DIV_PREC_BITS_POW2
  static const int pow2O[8] = { 4822, 5952, 6624, 6792, 6408, 5424, 3792, 1466 }; // DIV_PREC_BITS
  static const int pow2B[8] = { 12784, 12054, 11670, 11583, 11764, 12195, 12870, 13782 }; // DIV_PREC_BITS

  shift         = floorLog2Uint64(denom);
  round         = 1 << shift >> 1;
  int normDiff  = (((denom << DIV_PREC_BITS) + round) >> shift) & ((1 << DIV_PREC_BITS) - 1);
  int diffFull  = normDiff >> DIV_INTR_BITS;
  int normDiff2 = normDiff - pow2O[diffFull];

  scale = ((pow2W[diffFull] * ((normDiff2 * normDiff2) >> DIV_PREC_BITS)) >> DIV_PREC_BITS_POW2) - (normDiff2 >> 1) +
    pow2B[diffFull];
  scale <<= decimalBits - DIV_PREC_BITS;

  return { scale, round, shift };
}

// Non-fixed point integer division using look-up tables
inline int lutDivideInteger(const int64_t num, const int64_t denom)
{
  const auto [scale, round, shift] = lutDivideGetScaleRoundShift(denom, 16);
  if (num < 0)
  {
    return -int((-num * scale + round) >> (shift + 16));
  }
  else
  {
    return int((num * scale + round) >> (shift + 16));
  }
}

//! \}

#endif //__STANDARDIZEDMATHFUNCTIONS__
