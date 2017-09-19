#pragma once

#include "Defines.h"

#if FIXED_SIZE == 8
  #define FixedPoint int8_t
  #define FixedPointLarge int16_t
  #define SHIFT_VAL 4
  #define SQRT_SHIFT_VAL 2
  #define FP_MAX 0x7F
  #define FP_2PI 100
  #define NUM_DECIMAL_PLACES 1  // Floor(Log10(1<<SHIFT_VAL))
#elif FIXED_SIZE == 16
  #define FixedPoint int16_t
  #define FixedPointLarge int32_t
  #define SHIFT_VAL 8
  #define SQRT_SHIFT_VAL 4
  #define FP_MAX 0x7FFF
  #define FP_2PI 1608
  #define NUM_DECIMAL_PLACES 2
#elif FIXED_SIZE == 32
  #define FixedPoint int32_t
  #define FixedPointLarge int64_t
  // Multiplication by 65536
  #define SHIFT_VAL 16
  #define SQRT_SHIFT_VAL 8
  #define FP_MAX 0x7FFFFFFF
  #define FP_2PI 411774
  #define NUM_DECIMAL_PLACES 4
#elif FIXED_SIZE == 64
  #define FixedPoint int64_t
  #define FixedPointLarge int64_t
  #define SHIFT_VAL 32
  #define SQRT_SHIFT_VAL 16
  #define FP_MAX 0x7FFFFFFFFFFFFFFF
  #define FP_2PI 26986075409
  #define NUM_DECIMAL_PLACES 9
#else
  #define FixedPoint int16_t
  #define FixedPointLarge int32_t
  #define SHIFT_VAL 8
  #define SQRT_SHIFT_VAL 4
  #define FP_MAX 0x7FFF
  #define FP_2PI 1608
  #define NUM_DECIMAL_PLACES 2
#endif






//#include "Arduino.h"
#include <limits.h>

#ifdef WIN32
#include <intrin.h>
#include <stdint.h>
#endif

inline unsigned int msb(unsigned int value) {
#ifdef WIN32
  unsigned long index;
  unsigned char zero = _BitScanReverse(&index, value);
  return zero ? index : zero;
#else
  return 15 - __builtin_clz(value);
#endif
}

inline FixedPoint multiply(FixedPoint m1, FixedPoint m2);
inline FixedPoint divide(FixedPointLarge num, FixedPoint denom);


#define MULT(a,b) multiply(a, b)
#define DIV(a,b) divide(a, b)
#define ABS(a) (a > 0 ? a : a * -1)
#define FROM_INT(a) (((FixedPoint)a) * ((FixedPoint)1 << SHIFT_VAL))
#define FROM_FLOAT(a) ((FixedPoint)(a * ((FixedPoint)1 << SHIFT_VAL)))
#define TO_INT(a) ((int)((a < 0) ? ((a * -1) >> SHIFT_VAL)*-1 : (a >> SHIFT_VAL)))
//#define TO_FLOAT(a) (a / ((float)(1UL << SHIFT_VAL)))
#define FRACTION_PART(a) (a - ((a >> SHIFT_VAL) << SHIFT_VAL))
#define TO_STRING(a) (fpToString(a))


const FixedPoint ONE = FROM_INT(1);
const FixedPoint ZERO = 0;
const FixedPoint POINT_FIVE = DIV(ONE, FROM_INT(2));
const FixedPoint EPSILON = DIV(ONE, FROM_INT(1000));
const FixedPoint G = DIV(FROM_INT(-981), FROM_INT(100));
#define FP_PI (FP_2PI >> 1)

String fpToString(FixedPoint fp) {
  bool negative = fp < 0;
  if (negative) {
    fp *= -1;
  }
  String retval = negative ? "-" : "";
  retval += (fp >> SHIFT_VAL);
  String decimal = ".";
  FixedPoint remainder = FRACTION_PART(fp);
  FixedPoint place = ONE;
  
  for(int i = 0; i < NUM_DECIMAL_PLACES; i++) {  
    place = DIV(place, FROM_INT(10));
    int digit = TO_INT(DIV(remainder, place));
    decimal += digit;    
    remainder = remainder - MULT(place, FROM_INT(digit));
  }
  return retval + decimal;  
}


inline FixedPoint multiply(FixedPoint m1, FixedPoint m2) {
  bool negative = false;
  if (m1 < 0) {
    m1 *= -1;
    negative = true;
  }
  if (m2 < 0) {
    m2 *= -1;
    negative = !negative;
  }

  /*if (msb(m1) + msb(m2) >= SHIFT_VAL - 2) {
    FixedPointLarge m1b = (FixedPointLarge)m1;
    FixedPointLarge m2b = (FixedPointLarge)m2;
    FixedPointLarge resultb = m1b * m2b;
    resultb = resultb >> SHIFT_VAL;
    return (FixedPoint)(negative ? resultb * -1 : resultb);
    }*/

  FixedPointLarge result;
  result = (FixedPointLarge)m1 * m2;
  result = (FixedPoint)(result >> SHIFT_VAL);

  return (FixedPoint)(negative ? result * -1 : result);
}

inline FixedPoint divide(FixedPointLarge num, FixedPoint denom) {
  bool negative = false;
  if (num < 0) {
    num *= -1;
    negative = true;
  }
  if (denom < 0) {
    denom *= -1;
    negative = !negative;
  }
  num <<= SHIFT_VAL;
  return (FixedPoint)(negative ? (num / denom) * -1 : (num / denom));
}

FixedPoint sRoot(FixedPoint num) {
  if (TO_INT(num) < 0) {
    Serial.print("Negative square root error");
    for (;;);
  }
  int16_t shift = 2;
  FixedPoint numShifted = num >> shift;
  while (numShifted != 0) {
    shift += 2;
    numShifted = num >> shift;
    //Serial.println(numShifted);
  }
  shift -= 2;
  //Serial.print("Shift:");
  //Serial.println(shift);

  FixedPoint result = 0;
  while (shift >= 0) {
    //Serial.print("Shift:");
    //Serial.println(shift);
    result = result << 1;
    FixedPoint candidateResult = result + 1;
    if (candidateResult * candidateResult <= (num >> shift)) {
      result = candidateResult;
    }
    shift -= 2;
  }

  return result << (SHIFT_VAL - SQRT_SHIFT_VAL);
}

