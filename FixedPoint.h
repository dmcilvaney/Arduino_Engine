#pragma once

#include "Defines.h"
#include <avr/pgmspace.h>

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

inline FixedPoint fp_multiply(FixedPoint m1, FixedPoint m2);
inline FixedPoint fp_divide(FixedPointLarge num, FixedPoint denom);


#define MULT(a,b) fp_multiply(a, b)
#define DIV(a,b) fp_divide(a, b)
#define ABS(a) (a > 0 ? a : a * -1)
#define FROM_INT(a) (((FixedPoint)a) * ((FixedPoint)1 << SHIFT_VAL))
#define FROM_INT_SHIFT(a,decimalShift) (fp_fromIntShift(a,decimalShift))
#define FROM_FLOAT(a) ((FixedPoint)(a * ((FixedPoint)1 << SHIFT_VAL)))
#define TO_INT(a) ((int)((a < 0) ? ((a * -1) >> SHIFT_VAL)*-1 : (a >> SHIFT_VAL)))
//#define TO_FLOAT(a) (a / ((float)(1UL << SHIFT_VAL)))
#define FRACTION_PART(a) (a - ((a >> SHIFT_VAL) << SHIFT_VAL))
#define TO_STRING(a) (fp_ToString(a))


const FixedPoint ONE = FROM_INT(1);
const FixedPoint ZERO = 0;
const FixedPoint POINT_FIVE = DIV(ONE, FROM_INT(2));
const FixedPoint EPSILON = DIV(ONE, FROM_INT(1000));
const FixedPoint G = DIV(FROM_INT(981), FROM_INT(100));
#define FP_PI (FP_2PI >> 1)

String fp_ToString(FixedPoint fp) {
  bool negative = fp < 0;
  if (negative) {
    fp *= -1;
  }
  String retval = negative ? "-" : "";
  retval += (int)(fp >> SHIFT_VAL);
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

String format64(int64_t val) {
  char buffer[25];
  char*p = &buffer[24];
  *p = '\0';
  bool negative = val < 0;
  if (negative) {
    val *= -1;
  }
  do {
    *(--p) = (char)('0' + (int)(val % 10));
    val /= 10;
  }while(val != 0);
  if(negative) {
    *(--p) = '-';
  }
  return String(p);
}

inline FixedPoint fp_multiply(FixedPoint m1, FixedPoint m2) {
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

inline FixedPoint fp_divide( FixedPointLarge num, FixedPoint denom) {
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

inline FixedPoint fp_fromIntShift(FixedPoint a, FixedPoint decimalShift) {
  int shift = 1;
  for(int i = 0; i < decimalShift; i++) {
    shift *= 10;
  }
  return DIV(FROM_INT(a),FROM_INT(shift));
}

FixedPoint fp_sRoot(FixedPoint num) {
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

const int64_t arctan_lookup_table[] PROGMEM = {0,21474657,42948241,64419678,85887895,107351821,128810385,150262518,171707153,193143226,214569675,235985440,257389466,278780699,300158091,321520597,342867177,364196796,385508422,406801031,428073602,449325123,470554584,491760985,512943331,534100634,555231914,576336198,597412519,618459921,639477455,660464179,681419162,702341479,723230217,744084471,764903345,785685953,806431419,827138878,847807473,868436360,889024704,909571681,930076477,950538292,970956333,991329821,1011657988,1031940078,1052175346,1072363058,1092502492,1112592940,1132633704,1152624099,1172563451,1192451099,1212286396,1232068704,1251797400,1271471872,1291091521,1310655761,1330164018,1349615729,1369010347,1388347335,1407626168,1426846335,1446007338,1465108690,1484149917,1503130558,1522050162,1540908295,1559704531,1578438458,1597109675,1615717797,1634262445,1652743257,1671159882,1689511978,1707799219,1726021287,1744177879,1762268700,1780293470,1798251917,1816143783,1833968820,1851726791,1869417470,1887040642,1904596102,1922083657,1939503124,1956854330,1974137111,1991351317,2008496804,2025573440,2042581102,2059519676,2076389061,2093189160,2109919889,2126581173,2143172945,2159695145,2176147727,2192530647,2208843875,2225087386,2241261165,2257365204,2273399503,2289364070,2305258921,2321084079,2336839575,2352525445,2368141736,2383688498,2399165790,2414573679,2429912235,2445181536,2460381669,2475512722,2490574794,2505567986,2520492407,2535348172,2550135400,2564854216,2579504750,2594087138,2608601521,2623048043,2637426856,2651738113,2665981974,2680158604,2694268170,2708310844,2722286804,2736196229,2750039304,2763816217,2777527159,2791172326,2804751917,2818266134,2831715181,2845099268,2858418605,2871673407,2884863892,2897990280,2911052792,2924051655,2936987095,2949859344,2962668632,2975415196,2988099272,3000721098,3013280915,3025778966,3038215495,3050590748,3062904973,3075158420,3087351339,3099483983,3111556605,3123569460,3135522805,3147416896,3159251992,3171028353,3182746238,3194405910,3206007631,3217551662,3229038269,3240467715,3251840265,3263156184,3274415740,3285619198,3296766824,3307858888,3318895654,3329877393,3340804371,3351676857,3362495119,3373259426,};

// (x / (PI/4)) * 200

//Only works from 0 to pi/4.
FixedPoint fp_arctan_lookup(FixedPoint x) {
  //Serial.println(TO_STRING(x));
  if(x > ONE || x < 0) {
    Serial.println("Arctan out of bounds!");
    for(;;);
  }
  
  const FixedPoint lookupX = MULT(FROM_INT(200), x);
  
  int i = TO_INT(lookupX);
  
  //Serial.println(TO_STRING(lookupX));
  //Serial.println(i);
  int64_t n1;
  memcpy_P (&n1, &arctan_lookup_table[i], sizeof(int64_t));
  int i2 = i + 1;
  if (i2 > 200) {
    return (FixedPoint)(n1 >> (32 - SHIFT_VAL));
  }
  int64_t n2;
  memcpy_P (&n2, &arctan_lookup_table[i+1], sizeof(int64_t));

  FixedPoint n1Fp = (FixedPoint)(n1 >> (32 - SHIFT_VAL));
  FixedPoint n2Fp = (FixedPoint)(n2 >> (32 - SHIFT_VAL));
  //Serial.println(TO_STRING(n1Fp));
  //Serial.println(TO_STRING(n2Fp));
  
  FixedPoint diffFromTableEntry = lookupX - FROM_INT(i);
  //Serial.print("Diff:");
  //Serial.println(TO_STRING(diffFromTableEntry));

  return n1Fp + MULT(n2Fp - n1Fp, diffFromTableEntry);
}

FixedPoint fp_arctangent2(FixedPoint x, FixedPoint y) {
  FixedPoint temp;
  FixedPoint offset = 0;

  if(y < 0) {
    //Serial.println("Flipping above y=0");
    x *= -1;
    y *= -1;
    offset += 4;    
    //Serial.print('x');Serial.println(TO_STRING(x));
    //Serial.print('y');Serial.println(TO_STRING(y));
    //Serial.print('o');Serial.println(offset);
  }
  if(x<=0) {
    //Serial.println("Flipping to the right");
    temp = x;
    x = y;
    y = -temp;
    offset += 2;
    //Serial.print('x');Serial.println(TO_STRING(x));
    //Serial.print('y');Serial.println(TO_STRING(y));
    //Serial.print('o');Serial.println(offset);
  }
  if(x <= y) {
    //Serial.println("Move to lower half.");
    temp = y - x;
    x = x+y;
    y = temp;
    offset += 1;
    //Serial.print('x');Serial.println(TO_STRING(x));
    //Serial.print('y');Serial.println(TO_STRING(y));
    //Serial.print('o');Serial.println(offset);
  }
  //Serial.println("atan2");
  //Serial.println(TO_STRING(x));
  //Serial.println(TO_STRING(y));
  //Serial.println(offset);
  //Serial.println(TO_STRING((FP_PI>>2)*offset));
  //Serial.println(TO_STRING(DIV(y,x)));
  //Serial.println();
  FixedPoint at = fp_arctan_lookup(DIV(y,x));
  //Serial.println(TO_STRING(at));
  //Serial.println();
  
  return at + (FP_PI>>2)*offset;
}

