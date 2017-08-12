#pragma once

#include "Object.h"
#include "SimulationObjects.h"
#include "ForceGenerators.h"
#include "Renderer.h"
#include "Defines.h"
#include "Collisions.h"
#include "debug.h"

#define MIN_TIME_STEP 10

Simulation sim;

static void initSimulationEngine() {
  for (int i = 0; i < sim.m_numObjects; i++) {
    sim.m_worldObjects[i].m_inUse = false;
#ifdef DEBUG
    Serial.print("Init object ");
    Serial.println(i);
#endif
  }
  sim.m_lastStepTime = millis();
}

Object* simulationGetFreeObject() {
  for(int i = 0; i < sim.m_numObjects; i++) {
    if (! sim.m_worldObjects[i].m_inUse) {
#ifdef DEBUG
      Serial.print("Returning object number ");
      Serial.println(i);
#endif
      return &(sim.m_worldObjects[i]);
    }
  }
  return NULL;
}

//(Object* obj, void (*generator)(Object&, const FixedPoint&)) {
ForceObject* simulationGetFreeForce() {
  for(int i = 0; i < sim.m_numForces; i++) {
    if(sim.m_worldForces[i].m_generator == NULL) {
#ifdef DEBUG
      Serial.print("Returning force number ");
      Serial.println(i);
#endif      
      return &(sim.m_worldForces[i]);
    }
  }
  return NULL;
}

int stepTime = 0;

static void stepSim() {
  unsigned long currentTime = millis();
  int timeDelta = (int)(currentTime - sim.m_lastStepTime);

  if(timeDelta < MIN_TIME_STEP) {
    render(sim);
    return;
  }

  int stepDelta = min(stepTime + 2, timeDelta);
  
#ifdef DEBUG    
  Serial.println("Step Sim");
  Serial.print("Time:");
  Serial.println(timeDelta);
#endif
  sim.m_lastStepTime += stepDelta;
  FixedPoint sec = DIV(FROM_INT(stepDelta), FROM_INT(1000));
  for(int i = 0; i < sim.m_numForces; i++) {
#ifdef DEBUG
    Serial.print("Checking force ");
    Serial.println(i);
    Serial.println((int)&sim.m_worldForces[i]);
#endif
    if(sim.m_worldForces[i].m_generator != NULL) {
      sim.m_worldForces[i].m_generator(sim.m_worldForces[i], sec);
    }
  }
  
  for(int i = 0; i < sim.m_numObjects; i++) {
    Object& obj = sim.m_worldObjects[i];
    if (obj.m_inUse) {
      integrateObject(obj, sec);
    }          
  }

  int totalCollisions = 0;
  while(totalCollisions < 20) {
    int collisionNum = 0;
    int worstCollision = -1;
    FixedPoint worstCollisionVelocity = FP_MAX;
    for(int i = 0; i < sim.m_numObjects; i++) {
      for(int j = i+1; j < sim.m_numObjects; j++) {
        if(sim.m_worldObjects[i].m_inUse && sim.m_worldObjects[j].m_inUse) {
          //Serial.print("Collision:");
         // Serial.print(i);
          //Serial.print(',');
          //Serial.println(j);
          if( checkIfCollision(&(sim.m_worldContacts[collisionNum]), &(sim.m_worldObjects[i]), &(sim.m_worldObjects[j]))) {
            calcSeperatingVelocity(sim.m_worldContacts[collisionNum]);
            debug("Coll: SepV:", DEBUG_SIM);
            debug(TO_FLOAT(sim.m_worldContacts[collisionNum].m_seperatingVelocity), DEBUG_SIM);
            debug(" Distance:", DEBUG_SIM);
            debug(TO_FLOAT(sim.m_worldContacts[collisionNum].m_penetration), DEBUG_SIM);
            debugln(DEBUG_SIM);
            if(sim.m_worldContacts[collisionNum].m_seperatingVelocity < 0 && sim.m_worldContacts[collisionNum].m_seperatingVelocity < worstCollisionVelocity) {
              debugln("Adding collision", DEBUG_SIM);
              worstCollisionVelocity = sim.m_worldContacts[collisionNum].m_seperatingVelocity;
              worstCollision = collisionNum;              
              collisionNum++;
            }            
          }
        }
      }
    }
    if(collisionNum == 0) {
      break;
    }
    resolveContact(sim.m_worldContacts[worstCollision], sec);
    totalCollisions++;
    debugln("Worst collision:" + String(worstCollision), DEBUG_SIM);
  }
  stepTime = millis() - currentTime;
}


