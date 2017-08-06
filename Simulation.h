#pragma once

#include "Object.h"
#include "Renderer.h"

#define MIN_TIME_STEP 30

Simulation sim;

static void initSimulationEngine() {
  sim.m_numObjects = NUM_OBJECTS;
  for (int i = 0; i < NUM_OBJECTS; i++) {
    sim.m_worldObjects[i].m_inUse = false;
    sim.m_lastStepTime = millis();
  }
}

Object* simulationGetFreeObject() {
  for(int i = 0; i < NUM_OBJECTS; i++) {
    if (! sim.m_worldObjects[i].m_inUse) {
      Serial.print("Returning object number ");
      Serial.println(i);
      return &(sim.m_worldObjects[i]);
    }
  }
  return NULL;
}

static void stepSim() {
    int timeDelta = (int)(millis() - sim.m_lastStepTime);
    if(timeDelta >= MIN_TIME_STEP) {
      render(sim);
    }
}


