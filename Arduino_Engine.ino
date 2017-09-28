
#include "Vector.h"
#include "debug.h"
#include "FixedPoint.h"

#include <Servo.h>

Servo leverServo;

//#define TESTMODE
#define TESTNUM  10000

#ifndef TESTMODE
#include "Simulation.h"
Object* p1;
Object* p2;
Object* p3;
Object* c;
Vector3D v;
#endif

void setup() {
  Serial.begin(250000);
#ifdef TESTMODE
  test();
#else
  leverServo.attach(7);
  initSimulationEngine();
  PROFILE_INIT();
  PROFILE_ON(PROFILE_COLLISION);
  delay(200);
  PROFILE_OFF(PROFILE_COLLISION);
  
  p1 = simulationGetFreeObject();
  p1->m_inUse = true;
  p1->m_objectType = PARTICLE;
  p1->m_position = Vector3D(FROM_INT(5),FROM_INT(12),FROM_INT(0));
  p1->m_velocity = Vector3D(0,0,0);
  p1->m_invMass = 0;//DIV(FROM_INT(1),FROM_INT(1));
  p1->m_particleData.m_radius = ONE;

  p2 = simulationGetFreeObject();
  //p2->m_inUse = true;
  p2->m_objectType = PARTICLE;
  p2->m_position = Vector3D(FROM_INT(6),FROM_INT(15),FROM_INT(0));
  p2->m_velocity = Vector3D(0,FROM_INT(-6),0);
  p2->m_invMass = ONE;
  p2->m_particleData.m_radius = ONE;

  p3 = simulationGetFreeObject();
  p3->m_inUse = true;
  p3->m_objectType = PARTICLE;
  p3->m_position = Vector3D(FROM_INT(2),FROM_INT(16),FROM_INT(0));
  p3->m_velocity = Vector3D(0,0,0);
  p3->m_invMass = DIV(FROM_INT(1),FROM_INT(1));
  p3->m_particleData.m_radius = ONE;

  buildGravityForce(simulationGetFreeForce(), p1);
  //buildGravityForce(simulationGetFreeForce(), p2);
  buildGravityForce(simulationGetFreeForce(), p3);
  
  v = Vector3D(FROM_INT(5),FROM_INT(0),FROM_INT(0));
  
  //buildSpringForce(simulationGetFreeForce(), p3, &(p2->m_position), FROM_INT(2), FROM_INT(4), false);
  //buildSpringForce(simulationGetFreeForce(), p2, &(p3->m_position), FROM_INT(2), FROM_INT(4), false);
  
  buildRodConstraint(simulationGetFreeConstraint(), p1, p2, FROM_INT(5) - EPSILON);
  //buildRodConstraint(simulationGetFreeConstraint(), p1, p3, FROM_INT(5));
  //buildRodConstraint(simulationGetFreeConstraint(), p2, p3, FROM_INT(5));

  /*buildSpringForce(simulationGetFreeForce(), p1, &(p2->m_position), FROM_INT(5), FROM_INT(20), false);
  buildSpringForce(simulationGetFreeForce(), p1, &(p3->m_position), FROM_INT(5), FROM_INT(20), false);
  buildSpringForce(simulationGetFreeForce(), p2, &(p3->m_position), FROM_INT(5), FROM_INT(20), false);
  buildSpringForce(simulationGetFreeForce(), p2, &(p1->m_position), FROM_INT(5), FROM_INT(20), false);
  buildSpringForce(simulationGetFreeForce(), p3, &(p1->m_position), FROM_INT(5), FROM_INT(20), false);
  buildSpringForce(simulationGetFreeForce(), p3, &(p2->m_position), FROM_INT(5), FROM_INT(20), false);*/
  
  //buildSpringForce(simulationGetFreeForce(), p1, &v, FROM_INT(5), FROM_INT(20), true);

  buildAnalogForce(simulationGetFreeForce(), p2);

  c = simulationGetFreeObject();
  c->m_inUse = true;
  c->m_objectType = PARTICLE;
  c->m_position = v;
  c->m_invMass = 0;
  c->m_particleData.m_radius = FROM_INT(4);

  /*Object* e = simulationGetFreeObject();
  e->m_inUse = true;
  e->m_objectType = PARTICLE;
  e->m_position = v + Vector3D(FROM_INT(2),0,0);
  e->m_invMass = 0;
  e->m_particleData.m_radius = ONE;*/

  /*Object* d = simulationGetFreeObject();
  d->m_inUse = true;
  d->m_objectType = PARTICLE;
  d->m_position = v + Vector3D(ONE,FROM_INT(6),0);
  d->m_invMass = DIV(FROM_INT(1),FROM_INT(2));
  d->m_particleData.m_radius = ONE;
  buildGravityForce(simulationGetFreeForce(), d, POINT_FIVE);
  */

  //buildSpringForce(simulationGetFreeForce(), a, &(c->m_position), ONE, FROM_INT(15), false);
#endif
}



void loop() {
#ifndef TESTMODE
  stepSim();
  Vector3D leverAngleVector = p2->m_position - p1->m_position;
  leverAngleVector.normalize();
  FixedPoint angle = arctangent2(leverAngleVector.m_x, leverAngleVector.m_y);
  angle = DIV(MULT(angle, FROM_INT(180)), FP_PI);
  leverServo.write(TO_INT(angle) - 180);
  //Serial.print("\t(");
  //Serial.print(TO_STRING(leverAngleVector.m_x));
  //Serial.print(',');
  //Serial.print(TO_STRING(leverAngleVector.m_y));
  //Serial.print(") = ");
  //Serial.println(TO_STRING(angle));
  
#endif
}


void test() {
  Serial.println("TEST MODE!");
  
  Serial.println("PI:");
  Serial.println(TO_STRING(FP_PI));
  
  FixedPoint three = FROM_INT(3);
  Serial.println((int32_t)three);
  Serial.println((int32_t)(three * -1));
  Vector3D vNeg(FROM_INT(3),FROM_INT(3),FROM_INT(3));
  Vector3D vTwo(FROM_INT(2),FROM_INT(2),FROM_INT(2));
  Serial.print("Neg test:");
  vNeg.print();
  vTwo.print();
  vNeg = vNeg + (vTwo * -ONE);
  vNeg.print();

  Serial.println();
  
 
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
  Serial.println(TO_STRING(v1.magnitude()));
  v3 = v1;
  v3.normalize();
  Serial.println(TO_STRING(v3.magnitude()));
  Serial.println("CP:");
  (v1%v2).print();
  Serial.println("DP");
  Serial.print(TO_STRING(v1*v2));

  Serial.println();
  v1.print();
  v2.print();
  v3.print();
  Serial.println();
  makeOrthonormalBasis(&v1, &v2, &v3);
  v1.print();
  v2.print();
  v3.print();


  Serial.println("Testing arctan PI/4   ");
  FixedPoint at0 = arctan_lookup((FP_PI>>2) - (EPSILON+ EPSILON + EPSILON));
  Serial.print("atan=");
  Serial.println(TO_STRING(at0));
  
  Serial.println("Testing arctan 0.505   ");
  FixedPoint at1 = arctan_lookup(FROM_FLOAT(0.507));
  Serial.print("atan=");
  Serial.println(TO_STRING(at1));
  Serial.println("Testing arctan 1  ");
  FixedPoint at2 = arctan_lookup(ONE);
  Serial.println(TO_STRING(at2));
  Serial.print("Angle of vector (1,1)");
  Vector3D atanVect(ONE,-ONE-ONE,ONE);
  atanVect.normalize();
  atanVect.print();
  Serial.println(TO_STRING(arctangent2(atanVect.m_x,atanVect.m_y)));

  Serial.println();
  uint32_t startTime = millis();


  for (uint32_t i = 0; i < TESTNUM; i++) {
    v3 = v1;
    v3.normalize();
  }
  uint32_t endTime = millis();

  Serial.print("ms per op with my code=");
  FixedPoint timeDelta = FROM_INT(endTime - startTime);
  Serial.println(TO_STRING(DIV(timeDelta, FROM_INT(TESTNUM))));


}

