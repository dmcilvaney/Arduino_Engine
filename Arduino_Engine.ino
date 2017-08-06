//#include "Simulation.h"
#include "Vector.h"


#define TESTNUM  10000

void setup() {
  Serial.begin(115200);

/*
  Serial.print("20000:");
  FixedPoint temp = FROM_INT(20000);
  Serial.println(temp);
  Serial.print("0.99:");
  FixedPoint mult = FROM_FLOAT(0.9999999);
  Serial.println(mult);
  mult = FROM_FLOAT(20000);
  Serial.println(mult);
  Serial.println(TO_FLOAT(mult));

  Serial.println(TO_FLOAT(sRoot(FROM_INT(4))));
  Serial.println(TO_FLOAT(sRoot(FROM_INT(5))));
  Serial.println(TO_FLOAT(sRoot(FROM_INT(13))));
  Serial.println(TO_FLOAT(sRoot(FROM_FLOAT(20000))));


  uint32_t startTime = millis();

  for (uint32_t i = 0; i < TESTNUM; i++) {
    break;
    //Serial.print("-");
    //Serial.println(temp);
    temp = MULT(temp, mult);
    //Serial.println(temp);
    if (temp < FROM_INT(1)) {
      //Serial.print(temp);
      //Serial.print(" +5 -> ");
      temp = temp + FROM_INT(5);
      //Serial.println(temp);
      }
    //temp = DIV(temp, FROM_INT(2));
    //temp = temp + FROM_INT(2);
  }
  uint32_t endTime = millis();

  Serial.print("Time with my code=");
  Serial.println(((endTime - startTime) / (float)TESTNUM));
  Serial.println(TO_FLOAT(temp));
  Serial.println(temp);

  startTime = millis();
  float temp2 = 2000000;
  for (uint32_t j = 0; j < TESTNUM; j++) {
    break;
    //temp2 = temp2 / 2.0;
    //temp2 = temp2 + 2.0;
    temp2 = temp2 * 0.9999999;
    //if (temp2 < 1) {
    //  temp2 += 5;
    //}

  }
  endTime = millis();
  Serial.print("Time with float=");
  Serial.println(((endTime - startTime) / (float)TESTNUM));

  Serial.println(temp2);
  Serial.println("Done Setup");
  Serial.println();
  Serial.println();
  */

  //Object c1 = buildCircle(Vector3D(FROM_INT(10UL), FROM_INT(10UL), 0), FROM_INT(10), ONE, Vector3D(FROM_INT(10), 0, 0),FROM_INT(10),0,DIV(ONE,FROM_INT(8)));
  //Object c2 = buildCircle(Vector3D(FROM_INT(1UL), FROM_INT(9UL), 0), ONE + ONE, ONE, Vector3D(FROM_INT(-5), -2, 0),FROM_INT(-1),0,DIV(ONE,FROM_INT(8)));
  //addObject(c1);
  //addObject(c2);

  Vector3D v1(FROM_INT(1),FROM_INT(2),FROM_INT(3));
  Vector3D v2(FROM_INT(3),FROM_INT(2),FROM_INT(1));

  v1.print();
  v2.print();

  (v1+v2).print();
  
  Serial.println();
  Vector3D v3 = v1;
  v3 += v3;
  v3.print();

  Serial.println();
  (v3 - v1).print();
  v3 -= v3;
  v3.print();

  Serial.println();
  Serial.println(TO_FLOAT(v1.magnitude()));
  v3 = v1;
  v3.normalize();
  Serial.println(TO_FLOAT(v3.magnitude()));
  Serial.println("CP:");
  (v1%v2).print();
  Serial.println("DP");
  Serial.print(TO_FLOAT(v1*v2));

  Serial.println();
  v1.print();
  v2.print();
  v3.print();
  Serial.println();
  makeOrthonormalBasis(&v1, &v2, &v3);
  v1.print();
  v2.print();
  v3.print();


  Serial.println();
  uint32_t startTime = millis();

  for (uint32_t i = 0; i < TESTNUM; i++) {
    v3 = v1;
    v3.normalize();
  }
  uint32_t endTime = millis();

  Serial.print("ms per op with my code=");
  Serial.println(((endTime - startTime) / (float)TESTNUM));
}

void loop() {
  /*
  //Serial.println("Loop");
  unsigned long currentTime = millis();
  stepSim();
  unsigned long endTime = millis();
  Serial.print("Time:");
  Serial.print(endTime - currentTime);
  printBall();
  delay(20);
  */
}
