#include "Objects.h"
#include "FixedPoint.h"
#include "Vector.h"


#pragma once

#define TIME_SCALE 1

FixedPoint sim_g = 321344;
Vector3D sim_globalForce;

uint16_t sim_objectSet = 0;
Object sim_objectList[16];


void addGlobalForce(Vector3D v) {
  sim_globalForce =  ADD_VECT(sim_globalForce, v);
}

void setG(FixedPoint g) {
  FixedPoint oneG = 321344;
  sim_g = g * oneG;
}

void resetForces() {
  for (uint8_t i; i < 16; i++) {
    if (sim_objectSet & (1 << i)) {
      sim_objectList[i].m_force = Vector3D();
    }
  }
}

void applyGlobalForces() {
  for (uint8_t i = 0; i < 16; i++) {
    if (sim_objectSet & (1 << i)) {
      sim_objectList[i].m_force = ADD_VECT(sim_objectList[i].m_force, sim_globalForce);
      FixedPoint fGrav = MULT(sim_g, sim_objectList[i].m_mass);
      sim_objectList[i].m_force.m_y -= fGrav;
    }
  }
}

void calculateCollissions(FixedPoint dt) {
  for (uint8_t i = 0; i < 16; i++) {
    if (sim_objectSet & (1 << i)) {
      for (uint8_t j = 0; j < 16; j++) {
        if (sim_objectSet & (1 << j) && i != j) {
          if (isColliding(sim_objectList[i], sim_objectList[j])) {
            Serial.print("BOOM!");
            resolveColissionCircleCircle(sim_objectList[i], sim_objectList[j], dt);
          }
        }
      }
    }
  }
}

unsigned long int lastTime;
bool sim_init = false;


const FixedPoint minDt = DIV(ONE, FROM_INT(200));
void resolveForces() {
  //Serial.println("Resolving forces");
  if (!sim_init) {
    lastTime = millis();
    sim_init = true;
  }
  unsigned long int currentTime = millis();
  FixedPoint timeElapsed = FROM_INT(currentTime - lastTime);
  FixedPoint dT = DIV(timeElapsed, FROM_INT(1000));
  dT = DIV(dT, FROM_INT(TIME_SCALE));
  if (dT > minDt) {
    //Serial.println(TO_FLOAT(timeElapsed));
    //Serial.println(TO_FLOAT(dT));
    lastTime = millis();
    for (uint8_t i = 0; i < 16; i++) {
      if (sim_objectSet & (1 << i)) {
        Object &o = sim_objectList[i];
        Vector3D acceleration = MULT_VECT_SCAL(o.m_force, o.m_invMass);
        o.m_velocity = ADD_VECT(o.m_velocity, MULT_VECT_SCAL(acceleration, dT));
        sim_objectList[i].m_location = ADD_VECT(o.m_location, MULT_VECT_SCAL(o.m_velocity, dT));
        o.m_force.m_x = 0;
        o.m_force.m_y = 0;

        FixedPoint angularAcceleration = DIV(o.m_torque, o.m_moment_of_inertia);
        o.m_angular_velocity += MULT(angularAcceleration, dT);
        o.m_orientation = (o.m_orientation + (MULT(o.m_angular_velocity, dT))) % FP_2PI;
      }
    }
    calculateCollissions(dT);
  }
}

uint8_t addObject(Object o) {
  Serial.println("Adding object");
  for (uint8_t i = 0; i < 16; i++) {
    if (!(sim_objectSet & (1 << i))) {
      sim_objectSet |= (1 << i);
      sim_objectList[i] = o;
      sim_objectList[i].m_force = Vector3D();

      Serial.print("X");
      Serial.println(TO_FLOAT(sim_objectList[i].m_location.m_x));
      Serial.println(TO_FLOAT(o.m_location.m_x));
      Serial.print("Y");
      Serial.println(TO_FLOAT(sim_objectList[i].m_location.m_y));
      Serial.println(TO_FLOAT(o.m_location.m_y));
      Serial.print("MASS");
      Serial.println(TO_FLOAT(sim_objectList[i].m_mass));
      Serial.println(TO_FLOAT(o.m_mass));



      return i + 1;
    }
  }
  return 0;
}

void printObjects() {
  for (uint8_t i = 0; i < 16; i++) {
    if (sim_objectSet & (1 << i)) {
      Serial.print("O");
      Serial.print(i);
      sim_objectList[i].m_location.print();
      Serial.println();
    }
  }
}


const FixedPoint MAX_DEPTH = DIV(ONE, FROM_INT(3));
const FixedPoint factor = DIV(FROM_INT(-80), FROM_INT(100));
void applyBounce() {
  for (uint8_t i = 0; i < 16; i++) {
    if (sim_objectSet & (1 << i)) {
      if (sim_objectList[i].m_location.m_y < 0) {
        sim_objectList[i].m_location.m_y = 0;
        if ( sim_objectList[i].m_velocity.m_y < 0) {
          sim_objectList[i].m_velocity.m_y = MULT(sim_objectList[i].m_velocity.m_y, factor);
          if (sim_objectList[i].m_force.m_y < 0) {
            sim_objectList[i].m_force.m_y = 0;
          }
        }
      }
      if (sim_objectList[i].m_location.m_x < 0 ) {
        sim_objectList[i].m_location.m_x = 0;
        if (sim_objectList[i].m_velocity.m_x < 0) {
          sim_objectList[i].m_velocity.m_x = MULT(sim_objectList[i].m_velocity.m_x, factor);
          if (sim_objectList[i].m_force.m_x < 0) {
            sim_objectList[i].m_force.m_x = 0;
          }
        }
      }
      if (sim_objectList[i].m_location.m_x > FROM_INT(10) ) {
        sim_objectList[i].m_location.m_x = FROM_INT(10);
        if ( sim_objectList[i].m_velocity.m_x > 0) {
          sim_objectList[i].m_velocity.m_x = MULT(sim_objectList[i].m_velocity.m_x, factor);
          if (sim_objectList[i].m_force.m_x > 0) {
            sim_objectList[i].m_force.m_x = 0;
          }
        }
      }
    }
  }
}
//50,20
void printBall() {
  for (int i = 0; i < 50; i++) {
    Serial.print('\n');
  }
  uint64_t positionMaskY = 0;
  uint64_t positionMaskX = 0;
  int maxX = 0;
  for (uint8_t i = 0; i < 16; i++) {
    if (sim_objectSet & (1 << i)) {
      int x = TO_INT(sim_objectList[i].m_location.m_x * 4);
      //Serial.println(x);
      //Serial.println(TO_INT(sim_objectList[i].m_location.m_y * 2));
      positionMaskX |= (((uint64_t)1) << x);
      positionMaskY |= (((uint64_t)1) << TO_INT(sim_objectList[i].m_location.m_y * 2));
      if (x > maxX) {
        maxX = x;
      }
    }
  }
  //Serial.print((unsigned long  int)positionMaskX);
  //Serial.print(',');
  //Serial.print((unsigned long int)positionMaskY);
  int ball_num[45] = {0};
  for (int y = 20; y >= 0; y--) {
    if (positionMaskY & (1 << y)) {
      //Serial.print('*');
      for (int x = 0; x <= maxX; x++) {
        bool ballHere = false;
        ball_num[x] = -1;
        if (positionMaskX & (((uint64_t)1) << x)) {
          //Serial.print('*');
          for (uint8_t i = 0; i < 16; i++) {
            //Serial.print(sim_objectSet);
            if (sim_objectSet & (((uint64_t)1) << i)) {
              int ballX = TO_INT(sim_objectList[i].m_location.m_x * 4);
              int ballY = TO_INT(sim_objectList[i].m_location.m_y * 2);
              if ( ballX == x && ballY == y) {
                ball_num[x] = i;
              }
            }
          }
        }
      }

      for (int x = 0; x <= maxX; x++) {
        if (ball_num[x] != -1) {
          //Serial.print("orient: ");
          //Serial.println(TO_FLOAT(sim_objectList[ball_num[x]].m_orientation));
          int orientation = (16+TO_INT(DIV(sim_objectList[ball_num[x]].m_orientation, FP_PI >> 2))) % 4;
          switch (orientation) {
            case (0):
              Serial.print('l');
              break;
            case (1):
              Serial.print('/');
              break;
            case (2):
              Serial.print('-');
              break;
            case (3):
              Serial.print('\\');
              break;
          }
        } else {
          Serial.print(' ');
        }
      }
    }
    Serial.print('\n');
  }
  for (int i = 0; i <= 40; i++) {
    Serial.print('T');
  }
  Serial.println();
}

void stepSim() {
  //Serial.println("Step");
  applyGlobalForces();
  resolveForces();
  //calculateCollissions();
  applyBounce();
  //printObjects();
}

