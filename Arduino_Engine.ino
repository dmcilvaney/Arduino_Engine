#include "Simulation.h"
#include "Vector.h"
#include "debug.h"

//#define TESTMODE
#define TESTNUM  10000


Object* p1;
Object* p2;
Object* p3;
Object* c;
Vector3D v;

void setup() {
  Serial.begin(250000);
#ifdef TESTMODE
  test();
#else
  initSimulationEngine();
  
  p1 = simulationGetFreeObject();
  p1->m_inUse = true;
  p1->m_objectType = PARTICLE;
  p1->m_position = Vector3D(FROM_INT(0),FROM_INT(10),FROM_INT(0));
  p1->m_velocity = Vector3D(0,0,0);
  p1->m_invMass = DIV(FROM_INT(1),FROM_INT(1));
  p1->m_particleData.m_radius = ONE;

  p2 = simulationGetFreeObject();
  p2->m_inUse = true;
  p2->m_objectType = PARTICLE;
  p2->m_position = Vector3D(FROM_INT(0),FROM_INT(15),FROM_INT(0));
  p2->m_velocity = Vector3D(0,FROM_INT(-6),0);
  p2->m_invMass = ONE;
  p2->m_particleData.m_radius = ONE;

  p3 = simulationGetFreeObject();
  //p3->m_inUse = true;
  p3->m_objectType = PARTICLE;
  p3->m_position = Vector3D(FROM_INT(2),FROM_INT(0),FROM_INT(0));
  p3->m_velocity = Vector3D(0,0,0);
  p3->m_invMass = DIV(FROM_INT(1),FROM_INT(1));
  p3->m_particleData.m_radius = ONE;

  //buildGravityForce(simulationGetFreeForce(), p1);
  //buildGravityForce(simulationGetFreeForce(), p2);
  //buildGravityForce(simulationGetFreeForce(), p3);
  
  v = Vector3D(FROM_INT(0),FROM_INT(0),FROM_INT(0));
  
  //buildSpringForce(simulationGetFreeForce(), b, &(a->m_position), ONE, FROM_INT(10), false);
  //buildSpringForce(simulationGetFreeForce(), a, &(b->m_position), ONE, FROM_INT(10), false);
  
  buildRodConstraint(simulationGetFreeConstraint(), p1, p2, FROM_INT(5));
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
  stepSim();
  stepSim();
  stepSim();
  stepSim();
  stepSim();
  stepSim();
  stepSim();
  //for(;;);
#endif
}


void test() {
  Serial.println("TEST MODE!");
  
  Serial.println("PI:");
  Serial.println(TO_STRING(FP_PI));
  
  FixedPoint three = FROM_INT(3);
  Serial.println(three);
  Serial.println(three * -1);
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

