#pragma once
#include "FixedPoint.h"
#include "Vector.h"
#include "Defines.h"



/*
 * For analogue readings < 850 formula is:
 * 
 * To convert analogue readings to grams force:
 * x / (-1.30315751*10^-5 * x^2 + 1.063940729*10^-2 * x + 2.30339406)
 * 
 * Curve fitting done with http://www.xuru.org/rt/NLR.asp
 * 
 * Data for curve fitting:
  Reading, Weight(g), predicted value
  0 50      0
  415 100   92.8
  530 150   123.8
  640 200   169.5
  705 250   211.9
  755 300   259.6
  790 350   306.7
  824 400   370.8
  855 450   456.3
  870 500   512.9
  880 550   558.9
  895 600   645.3
  900 700   680.1
  906 800   727.2
  920 900   866.5
  940 1100  1190.3
  950 1500  1461.9
 */

 //Fixed point representations in 64 bit with.
 const int64_t c1 = 0xDAA2; //1.30315751*10^-5
 const int64_t c2 = 0x2B943A2; //1.063940729*10^-2
 const int64_t c3 = 0x24DAB3BAE; //2.30339406
 
 FixedPoint FSRCalc(int analogueIn) {
  //Serial.println(analogueIn);

  //Avoid shifting x since we would just have to unshift it during multiplication
  int64_t x = (int64_t)analogueIn;
  //Serial.print("x:");
  //Serial.println(format64(x));  
  
  int64_t a = (x) * c1;  
  //Serial.print("a:");
  //Serial.println(format64(a));
  a = (a * x);
  //Serial.print("a2:");
  //Serial.println(format64(a));

  int64_t b = (x * c2);
  //Serial.print("b:");
  //Serial.println(format64(b));

  int64_t denom = b + c3 - a;
  //Serial.print("denom:");
  //Serial.println(format64(denom));

  //x < 1024, shift as far as possible (considering negative bit) before division, then shift the rest of the way.
  //Serial.print("x shift:");
  //Serial.println(format64(x << 53));
  int64_t result = ((x << 53) / denom) << 11;
  //Serial.print("result:");
  //Serial.println(format64(result));
  //convert from grams to newtons.
  return DIV((FixedPoint)(result >> (32 - SHIFT_VAL)),FROM_INT(102));
 }

 void FSRTest() {
  int vals[] = {  0,  415,  530,  640,  705,  755,  790,  824,  855,  870,  880,  895,  900,  906,  920,  940,  950  };
  for(int i = 0; i < 17; i++) {
    Serial.print(vals[i]);
    Serial.print(": ");
    Serial.println(TO_STRING(FSRCalc(vals[i])));
  }
 }

