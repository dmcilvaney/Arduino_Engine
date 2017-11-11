
#include "Vector.h"
#include "debug.h"
#include "FixedPoint.h"
#include "profile.h"

#include <Servo.h>

Servo leverServo;

#define A_UP 0
#define A_DOWN 1
#define A_SPRING_STRENGTH 2
#define A_LEVER_LENGTH 3

//#define TESTMODE
#define TESTNUM  10000

#ifdef TESTMODE
#include "FSR.h"
#else
#include "Simulation.h"
Object* pivotObj;
Object* rodEnd;
Object* weightObj;
ConstraintObject* leverRodConstraint;

ForceObject* springForce1;
ForceObject* springForce2;


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
  pivotObj->m_position = Vector3D(FROM_INT_SHIFT(38,2),FROM_INT_SHIFT(15,2),FROM_INT(0));
  pivotObj->m_velocity = Vector3D(0,0,0);
  pivotObj->m_invMass = 0;//DIV(FROM_INT(1),FROM_INT(1));
  pivotObj->m_particleData.m_radius = ZERO;

  rodEnd = simulationGetFreeObject();
  rodEnd->m_inUse = true;
  rodEnd->m_objectType = PARTICLE;
  rodEnd->m_position = Vector3D(FROM_INT_SHIFT(1,1),FROM_INT_SHIFT(0,2),FROM_INT(0));
  rodEnd->m_velocity = Vector3D(0,0,0);
  //0.1 kg
  rodEnd->m_invMass = DIV(FROM_INT(1),FROM_INT_SHIFT(5,2));
  rodEnd->m_particleData.m_radius = FROM_INT_SHIFT(3,2);

  weightObj = simulationGetFreeObject();
  weightObj->m_inUse = true;
  weightObj->m_objectType = PARTICLE;
  weightObj->m_position = Vector3D(FROM_INT_SHIFT(2,1),FROM_INT_SHIFT(2,2),FROM_INT(0));
  weightObj->m_velocity = Vector3D(0,0,0);
  //0.1 kg
  weightObj->m_invMass = DIV(FROM_INT(1),FROM_INT_SHIFT(5,2));
  weightObj->m_particleData.m_radius = FROM_INT_SHIFT(3,2);
  
  buildGravityForce(simulationGetFreeForce(), rodEnd);
  buildGravityForce(simulationGetFreeForce(), weightObj);

  springForce1 = simulationGetFreeForce();  
  buildSpringForce(springForce1, weightObj, &(rodEnd->m_position), FROM_INT_SHIFT(11,2), FROM_INT(15), false);
  springForce2 = simulationGetFreeForce();
  buildSpringForce(springForce2, rodEnd, &(weightObj->m_position), FROM_INT_SHIFT(11,2), FROM_INT(15), false);
  
  leverRodConstraint = simulationGetFreeConstraint();
  buildRodConstraint(leverRodConstraint, pivotObj, rodEnd, FROM_INT_SHIFT(15,2));

  buildCustomForce(simulationGetFreeForce(), rodEnd, &analogueForce);
  
  //buildSpringForce(simulationGetFreeForce(), a, &(c->m_position), ONE, FROM_INT(15), false);
#endif
}


Vector3D calculateLeverVector() {
  return rodEnd->m_position - pivotObj->m_position;
}

Vector3D calculateLeverForceVector(Vector3D leverVector) {
  Vector3D leverForceVector = leverVector;
  leverForceVector.normalize();
  //Cross product will give perpendicular vector.
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

#define SERVO_MEASUREMENTS 1
int servoMicroseconds[SERVO_MEASUREMENTS];
int currentServo = SERVO_MEASUREMENTS+1;
int moveServo(FixedPoint angle) {
  
  FixedPoint uSec = MULT(DIV(angle,FROM_INT(65)), FROM_INT(550));  
  int microseconds = 1500 +  TO_INT(uSec);
  microseconds = max(1500, min(2050, microseconds));

  if(currentServo > SERVO_MEASUREMENTS) {
    for(int i = 0;i < SERVO_MEASUREMENTS; i++) {
      servoMicroseconds[i] = microseconds;
    }
  }
  servoMicroseconds[(currentServo++)%SERVO_MEASUREMENTS] = microseconds;
  
  int32_t avgMicroseconds = 0;
  for(int i = 0;i < SERVO_MEASUREMENTS; i++) {
    avgMicroseconds += servoMicroseconds[i];
  }
  leverServo.writeMicroseconds(avgMicroseconds/SERVO_MEASUREMENTS);
}

FixedPoint calculateLeverLength() {
  int a = analogRead(A_LEVER_LENGTH);
  //Vary length between 5cm and 50cm
  return FROM_INT_SHIFT(map(a,1023,0,230,380),3);
}

FixedPoint calculateSpringStrength() {
  int a = analogRead(A_SPRING_STRENGTH);
  //Vary length between 5cm and 50cm
  return FROM_INT_SHIFT(map(a,1023,0,100,300),1);
}


#define FSR_MEASUREMENTS 2
FixedPoint fsrReadings[FSR_MEASUREMENTS];
int currentFsrReading = FSR_MEASUREMENTS+1;
void loop() {
#ifndef TESTMODE

  Vector3D lever = calculateLeverVector();
  //Newtons at 4 cm.

  fsrReadings[(currentFsrReading++)%FSR_MEASUREMENTS] = FSRCalc(analogRead(A_DOWN)) - FSRCalc(analogRead(A_UP));
  FixedPoint averageFsrReading = 0;
  for(int i = 0; i < FSR_MEASUREMENTS; i++) {
    averageFsrReading += fsrReadings[i];
  }
  FixedPoint torque = DIV(averageFsrReading,FROM_INT(FSR_MEASUREMENTS));

//  Serial.print("T=");
//  Serial.println(TO_STRING(torque));

  analogueForce = calculateLeverForceVector(lever) * torque;
  //analogueForce.print();
  //Counter force applied by movement.
  //analogueForce -= (rodEnd->m_acceleration * FROM_INT_SHIFT(3,0));
  
  //analogueForce -= (rodEnd->m_velocity * FROM_INT_SHIFT(1,0));
  //analogueForce.print();

  FixedPoint mechAdvantage = DIV(FROM_INT_SHIFT(4,2),lever.magnitude());
  //Serial.print("ma=");
  //Serial.println(TO_STRING(mechAdvantage));

  analogueForce *= mechAdvantage;

  FixedPoint leverAngle = calculateLeverAngle(lever);
  //Serial.println(TO_STRING(leverAngle));
  if((leverAngle > FROM_INT(60) && rodEnd->m_velocity.m_y > 0) || (leverAngle < FROM_INT(0) && rodEnd->m_velocity.m_y < 0)) {
    rodEnd->m_velocity*= FROM_INT_SHIFT(-5,1);
  }

  moveServo(leverAngle);

  leverRodConstraint->m_rodData.m_length = calculateLeverLength();
  springForce1->m_springData.m_springConstant = calculateSpringStrength();
  springForce2->m_springData.m_springConstant = calculateSpringStrength();

  Vector3D accelDirection = rodEnd->m_velocity;
  accelDirection.normalize();
  FixedPoint coincidentAcceleration = analogueForce * accelDirection;
  FixedPoint threshold = FROM_INT_SHIFT(5,1);
  if(coincidentAcceleration < -threshold) {
    coincidentAcceleration *= -1;
    if(coincidentAcceleration < threshold) {
      coincidentAcceleration = threshold;
    }
    FixedPoint dampingMultiplier = DIV(threshold,coincidentAcceleration);
    rodEnd->m_velocity *= dampingMultiplier;
  }
  
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
  FixedPoint at0 = fp_arctan_lookup((DIV(FP_PI,FROM_INT(4))) - (EPSILON+ EPSILON + EPSILON));
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

