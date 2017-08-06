#include "Simulation.h"
#include "Vector.h"

//#define TESTMODE
#define TESTNUM  10000

void setup() {
  Serial.begin(115200);

#ifdef TESTMODE
  test();
#else
  initSimulationEngine();
  Object* a = simulationGetFreeObject();
  a->m_inUse = true;
  a->m_objectType = PARTICLE;
  a->m_position = Vector3D(FROM_INT(5),FROM_INT(5),FROM_INT(5));

  Object* b = simulationGetFreeObject();
  b->m_inUse = true;
  b->m_objectType = PARTICLE;
  b->m_position = Vector3D(FROM_INT(7),FROM_INT(5),FROM_INT(5));
#endif
}

void loop() {
#ifndef TESTMODE
  stepSim();
  delay(20);
#endif
}


void test() {
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

