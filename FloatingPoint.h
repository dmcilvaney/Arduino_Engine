#pragma once

#include <avr/pgmspace.h>
#include <math.h>
#include <float.h>




//#include "Arduino.h"
#include <limits.h>

typedef float FixedPoint;

#define FP_MAX FLT_MAX


#define MULT(a,b) (a*b)
#define DIV(a,b) (a/b)
#define ABS(a) (a > 0 ? a : a * -1)
#define FROM_INT(a) ((float)a)
#define FROM_INT_SHIFT(a,decimalShift) (fp_fromIntShift(a,decimalShift))
#define FROM_FLOAT(a) (a)
#define TO_INT(a) ((int)a)
//#define TO_FLOAT(a) (a / ((float)(1UL << SHIFT_VAL)))
#define FRACTION_PART(a) (a - ((a >> SHIFT_VAL) << SHIFT_VAL))
#define TO_STRING(a) (fp_ToString(a))


const FixedPoint ONE = FROM_INT(1);
const FixedPoint ZERO = 0;
const FixedPoint POINT_FIVE = DIV(ONE, FROM_INT(2));
const FixedPoint EPSILON = DIV(ONE, FROM_INT(1000));
const FixedPoint G = DIV(FROM_INT(981), FROM_INT(100));
#define FP_PI PI

String fp_ToString(FixedPoint fp) {
  return String(fp); 
}

String format64(float val) {
  /*
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
  */
  return String(val);
}

inline FixedPoint fp_fromIntShift(FixedPoint a, FixedPoint decimalShift) {
  int shift = 1;
  for(int i = 0; i < decimalShift; i++) {
    shift *= 10;
  }
  return DIV(FROM_INT(a),FROM_INT(shift));
}

FixedPoint fp_sRoot(FixedPoint num) {
  return sqrt(num);
}

const int64_t arctan_lookup_table[] PROGMEM = {0,21474657,42948241,64419678,85887895,107351821,128810385,150262518,171707153,193143226,214569675,235985440,257389466,278780699,300158091,321520597,342867177,364196796,385508422,406801031,428073602,449325123,470554584,491760985,512943331,534100634,555231914,576336198,597412519,618459921,639477455,660464179,681419162,702341479,723230217,744084471,764903345,785685953,806431419,827138878,847807473,868436360,889024704,909571681,930076477,950538292,970956333,991329821,1011657988,1031940078,1052175346,1072363058,1092502492,1112592940,1132633704,1152624099,1172563451,1192451099,1212286396,1232068704,1251797400,1271471872,1291091521,1310655761,1330164018,1349615729,1369010347,1388347335,1407626168,1426846335,1446007338,1465108690,1484149917,1503130558,1522050162,1540908295,1559704531,1578438458,1597109675,1615717797,1634262445,1652743257,1671159882,1689511978,1707799219,1726021287,1744177879,1762268700,1780293470,1798251917,1816143783,1833968820,1851726791,1869417470,1887040642,1904596102,1922083657,1939503124,1956854330,1974137111,1991351317,2008496804,2025573440,2042581102,2059519676,2076389061,2093189160,2109919889,2126581173,2143172945,2159695145,2176147727,2192530647,2208843875,2225087386,2241261165,2257365204,2273399503,2289364070,2305258921,2321084079,2336839575,2352525445,2368141736,2383688498,2399165790,2414573679,2429912235,2445181536,2460381669,2475512722,2490574794,2505567986,2520492407,2535348172,2550135400,2564854216,2579504750,2594087138,2608601521,2623048043,2637426856,2651738113,2665981974,2680158604,2694268170,2708310844,2722286804,2736196229,2750039304,2763816217,2777527159,2791172326,2804751917,2818266134,2831715181,2845099268,2858418605,2871673407,2884863892,2897990280,2911052792,2924051655,2936987095,2949859344,2962668632,2975415196,2988099272,3000721098,3013280915,3025778966,3038215495,3050590748,3062904973,3075158420,3087351339,3099483983,3111556605,3123569460,3135522805,3147416896,3159251992,3171028353,3182746238,3194405910,3206007631,3217551662,3229038269,3240467715,3251840265,3263156184,3274415740,3285619198,3296766824,3307858888,3318895654,3329877393,3340804371,3351676857,3362495119,3373259426,};

// (x / (PI/4)) * 200

//Only works from 0 to pi/4.
FixedPoint fp_arctan_lookup(FixedPoint x) {
  return atan(x);
}

//Returns number of radians counter clockwise from x positive axis.
FixedPoint fp_arctangent2(FixedPoint x, FixedPoint y) {
  float val = atan2(y,x);
  return val < 0 ? (2 * FP_PI) + val : val;
}

