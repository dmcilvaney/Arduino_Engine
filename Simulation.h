#pragma once

#include "Object.h"
#include "SimulationObjects.h"
#include "ForceGenerators.h"
#include "Renderer.h"
#include "Defines.h"

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

static int loopNum = 0;
static void stepSim() {
  unsigned long currentTime = millis();
  int timeDelta = (int)(currentTime - sim.m_lastStepTime);
  //int timeDelta = 20;
  if(timeDelta >= MIN_TIME_STEP) {
#ifdef DEBUG    
    Serial.println("Step Sim");
    Serial.print("Time:");
    Serial.println(timeDelta);
#endif
    sim.m_lastStepTime = currentTime;
    //if(loopNum++ > 10) {
      render(sim);
    //  loopNum = 0;
    //}
    FixedPoint sec = DIV(FROM_INT(timeDelta), FROM_INT(1000));
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
  }
}


