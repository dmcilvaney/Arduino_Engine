
#include "Vector.h"
#include "debug.h"
#include "FixedPoint.h"

#include <Servo.h>

Servo leverServo;

#define A_UP 0
#define A_DOWN 1
#define A_SPEED 2
#define A_SWING 3

//#define TESTMODE
#define TESTNUM  10000

#ifdef TESTMODE
#include "FSR.h"
#else
#include "Simulation.h"
Object* pivotObj;
Object* rodEnd;


Vector3D analogueForce;
#endif

void initServo() {
  for(int i = 0; i < 5000; i++) {
    leverServo.attach(7);
    leverServo.writeMicroseconds(1700);
    delay(5);
    leverServo.detach();
    delay(1);
  }
  leverServo.attach(7);
}

void setup() {
  Serial.begin(250000);
#ifdef TESTMODE
  test();
#else
  leverServo.attach(7);
  //initServo();
  initSimulationEngine();
  PROFILE_INIT();
  PROFILE_ON(PROFILE_COLLISION);
  delay(200);
  PROFILE_OFF(PROFILE_COLLISION);
  
  pivotObj = simulationGetFreeObject();
  pivotObj->m_inUse = true;
  pivotObj->m_objectType = PARTICLE;
  //Location: .3m, 0.15m, 0m
  pivotObj->m_position = Vector3D(FROM_INT_SHIFT(3,1),FROM_INT_SHIFT(15,2),FROM_INT(0));
  pivotObj->m_velocity = Vector3D(0,0,0);
  pivotObj->m_invMass = 0;//DIV(FROM_INT(1),FROM_INT(1));
  pivotObj->m_particleData.m_radius = ZERO;

  rodEnd = simulationGetFreeObject();
  rodEnd->m_inUse = true;
  rodEnd->m_objectType = PARTICLE;
  rodEnd->m_position = Vector3D(FROM_INT_SHIFT(1,1),FROM_INT_SHIFT(15,2),FROM_INT(0));
  rodEnd->m_velocity = Vector3D(0,0,0);
  //0.5 kg
  rodEnd->m_invMass = DIV(FROM_INT(1),FROM_INT_SHIFT(5,1));
  rodEnd->m_particleData.m_radius = FROM_INT_SHIFT(5,2);
  
  //buildGravityForce(simulationGetFreeForce(), rodEnd);
  
  //buildSpringForce(simulationGetFreeForce(), p3, &(p2->m_position), FROM_INT(2), FROM_INT(4), false);
  //buildSpringForce(simulationGetFreeForce(), p2, &(p3->m_position), FROM_INT(2), FROM_INT(4), false);
  
  buildRodConstraint(simulationGetFreeConstraint(), pivotObj, rodEnd, FROM_INT_SHIFT(25,2));

  //buildAnalogForce(simulationGetFreeForce(), rodEnd, &analogueForce);
  
  //buildSpringForce(simulationGetFreeForce(), a, &(c->m_position), ONE, FROM_INT(15), false);
#endif
}


Vector3D calculateLeverVector() {
  return rodEnd->m_position - pivotObj->m_position;
}

Vector3D calculateLeverForceVector(Vector3D leverVector) {
  Vector3D leverForceVector = leverVector;
  leverForceVector.normalize();
  leverForceVector = leverForceVector % Vector3D(0,0,ONE);
  return leverForceVector;
}

FixedPoint calculateLeverAngle(Vector3D leverVector) {
  Vector3D leverAngleVector = leverVector;
  FixedPoint angle = fp_arctangent2(leverAngleVector.m_x, leverAngleVector.m_y);
  angle = DIV(MULT(angle, FROM_INT(180)), FP_PI);
  angle = FROM_INT(270) - angle;
  return angle - FROM_INT(45);
}

int servoMicroseconds = 1500;
int moveServo(FixedPoint angle) {
  FixedPoint uSec = MULT(DIV(angle,FROM_INT(65)), FROM_INT(550));  
  int microseconds = 1500 +  TO_INT(uSec);
  servoMicroseconds = ((servoMicroseconds) >> 1) + (microseconds >> 1);
  //servoMicroseconds = microseconds;
  servoMicroseconds = max(1500, min(2050, servoMicroseconds));
  leverServo.writeMicroseconds(servoMicroseconds);
}




void loop() {
#ifndef TESTMODE

  for(FixedPoint a = FROM_INT_SHIFT(5,1); a < FROM_INT_SHIFT(30,0); a+=FROM_INT_SHIFT(3,1)) { 

    Vector3D lever = calculateLeverVector();
    
    rodEnd->m_acceleration = calculateLeverForceVector(calculateLeverVector())*a;
    while(calculateLeverAngle(lever) < FROM_INT(65)) {
      
      stepSim(false);
      
      lever = calculateLeverVector();
      rodEnd->m_acceleration = calculateLeverForceVector(lever)*a;
      FixedPoint angle = calculateLeverAngle(lever);
      moveServo(angle);
      
      Serial.print(TO_STRING(rodEnd->m_velocity.magnitude() ));
      Serial.print(',');
      Serial.print(TO_STRING(rodEnd->m_acceleration.magnitude()));
      Serial.print(',');
      Serial.print(TO_STRING(FSRCalc(analogRead(A_UP))));
      Serial.print(',');
      Serial.println(TO_STRING(FSRCalc(analogRead(A_DOWN))));
    }
    rodEnd->m_velocity = Vector3D(0,0,0);
    rodEnd->m_acceleration = Vector3D(0,0,0);
    for(int i = 0; i < 500; i++) {
      delay(1);
      stepSim(false);
    } 
    
    rodEnd->m_acceleration = calculateLeverForceVector(calculateLeverVector())*a;
    while(calculateLeverAngle(lever) > FROM_INT(15)) {    
      
      stepSim(false);

      lever = calculateLeverVector();
      Vector3D lever = calculateLeverVector();
      rodEnd->m_acceleration = calculateLeverForceVector(lever)*(a*-1);
      FixedPoint angle = calculateLeverAngle(lever);
      moveServo(angle);
      
      Serial.print(TO_STRING(rodEnd->m_velocity.magnitude() ));
      Serial.print(',');
      Serial.print(TO_STRING(rodEnd->m_acceleration.magnitude()));
      Serial.print(',');
      Serial.print(TO_STRING(FSRCalc(analogRead(A_UP))));
      Serial.print(',');
      Serial.println(TO_STRING(FSRCalc(analogRead(A_DOWN))));
    }     
    rodEnd->m_velocity = Vector3D(0,0,0);
    rodEnd->m_acceleration = Vector3D(0,0,0);
    for(int i = 0; i < 500; i++) {
      delay(1);
      stepSim(false);
    }
  }
  for(;;);


  Vector3D lever = calculateLeverVector();
  //Newtons at 4 cm.
  FixedPoint torque = FSRCalc(A_UP) - FSRCalc(A_DOWN);

  analogueForce = calculateLeverForceVector(lever) * torque;
  //Counter force applied by movement.
  analogueForce -= rodEnd->m_acceleration * FROM_INT_SHIFT(7,2);

  FixedPoint mechAdvantage = DIV(FROM_INT_SHIFT(4,2),lever.magnitude());

  analogueForce *= mechAdvantage;

  moveServo(calculateLeverAngle(lever));
  
  stepSim();
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
  Serial.println("Cross product");
  (Vector3D(FROM_INT(3),FROM_INT(-3),FROM_INT(1)) % Vector3D(FROM_INT(4),FROM_INT(9),FROM_INT(2))).print();


  Serial.println("Testing arctan PI/4   ");
  FixedPoint at0 = fp_arctan_lookup((FP_PI>>2) - (EPSILON+ EPSILON + EPSILON));
  Serial.print("atan=");
  Serial.println(TO_STRING(at0));
  
  Serial.println("Testing arctan 0.505   ");
  FixedPoint at1 = fp_arctan_lookup(FROM_FLOAT(0.507));
  Serial.print("atan=");
  Serial.println(TO_STRING(at1));
  Serial.println("Testing arctan 1  ");
  FixedPoint at2 = fp_arctan_lookup(ONE);
  Serial.println(TO_STRING(at2));
  Serial.print("Angle of vector (1,1)");
  Vector3D atanVect(ONE,-ONE-ONE,ONE);
  atanVect.normalize();
  atanVect.print();
  Serial.println(TO_STRING(fp_arctangent2(atanVect.m_x,atanVect.m_y)));

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

  FSRTest();
  while(true) {
    //Serial.print("Analog reading:");
    //Serial.println(analogReading);
    FixedPoint force = FSRCalc(analogRead(A_UP));
    Serial.print(TO_STRING(force));
    force = FSRCalc(analogRead(A_DOWN));
    Serial.print(',');
    Serial.println(TO_STRING(force));
    delay(5);
  }
}

