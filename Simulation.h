#pragma once

#include "Object.h"
#include "SimulationObjects.h"
#include "ForceGenerators.h"
#include "Renderer.h"
#include "Defines.h"
#include "Collisions.h"
#include "Constraints.h"
#include "debug.h"

#define MIN_TIME_STEP 10

Simulation sim;

static void initSimulationEngine() {
  for (int i = 0; i < sim.m_numObjects; i++) {
    sim.m_worldObjects[i].m_inUse = false;
  }
  for (int i = 0; i < sim.m_numForces; i++) {
    sim.m_worldForces[i].m_generator = NULL;
  }
  for (int i = 0; i < sim.m_numConstraints; i++) {
    sim.m_worldConstraints[i].m_generator = NULL;
  }
  sim.m_simulationTime = millis();
}

Object* simulationGetFreeObject() {
  for(int i = 0; i < sim.m_numObjects; i++) {
    if (! sim.m_worldObjects[i].m_inUse) {
      return &(sim.m_worldObjects[i]);
    }
  }
  return NULL;
}

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
  Serial.println("No more forces!");
  for(;;);
  return NULL;
}

ConstraintObject* simulationGetFreeConstraint() {
  for(int i = 0; i < sim.m_numConstraints; i++) {
    if(sim.m_worldConstraints[i].m_generator == NULL) {
#ifdef DEBUG
      Serial.print("Returning constraint number ");
      Serial.println(i);
#endif      
      return &(sim.m_worldConstraints[i]);
    }
  }
  Serial.println("No more constraints!");
  for(;;);
  return NULL;
}

ContactObject* simulationGetFreeContact() {
  if (sim.m_activeContacts < sim.m_numContacts) {
    ContactObject* co = &(sim.m_worldContacts[sim.m_activeContacts]);
    sim.m_activeContacts++;
    return co;
  }
  Serial.println("No more contacts!");
  for(;;);
  return NULL;
}

//When did the last frame start.
unsigned long lastFrameTime = 0;
int lastFrameDelta = MIN_TIME_STEP;

static void stepSim() {
  unsigned long currentTime = millis();
  
  //Time difference between the real time and the simulation time.
  unsigned long timeDelta = currentTime - sim.m_simulationTime;

  if(timeDelta <= (lastFrameDelta >> 5) + 1) {
    //Simulation has caught up to real time (-1 frame back), render another frame
    render(sim);
    return;
  }

  Serial.print(timeDelta);
  Serial.print('-');
  Serial.println((currentTime - lastFrameTime));
  //lastFrameDelta += ((int)(currentTime - lastFrameTime) << 4  - (lastFrameDelta >> 4));
  lastFrameDelta += (currentTime - lastFrameTime);
  lastFrameTime = currentTime;

  // Otherwise we probbaly want to try to catch up to the current time.
  unsigned long stepDelta = max((lastFrameDelta) + 10, MIN_TIME_STEP);
  // Make sure we don't overshoot real time.
  stepDelta = min(stepDelta, timeDelta);
  //Serial.print("step:");
  Serial.println(stepDelta);
  
  sim.m_simulationTime += stepDelta;
  FixedPoint sec = DIV(FROM_INT(stepDelta), FROM_INT(1000));
  for(int i = 0; i < sim.m_numForces; i++) {
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
  sim.m_activeContacts = 0;

  //Add constraint derived contacts
  for(int i = 0; i < sim.m_numConstraints; i++) {
    if(sim.m_worldConstraints[i].m_generator != NULL) {
      debug("Checking constraint collision ", DEBUG_SIM);
      debugln(i, DEBUG_SIM);
      sim.m_worldConstraints[i].m_generator(sim.m_worldConstraints[i]);        
    }
  }

  debugln("Contact start",DEBUG_SIM);
  for(int i = 0; i < sim.m_numObjects; i++) {
    sim.m_worldObjects[i].m_penetrationAdjustment = Vector3D(0,0,0);
    for(int j = i+1; j < sim.m_numObjects; j++) {
      if(sim.m_worldObjects[i].m_inUse && sim.m_worldObjects[j].m_inUse) {
        ContactObject tentativeContact;
        if( checkIfCollision(&tentativeContact, &(sim.m_worldObjects[i]), &(sim.m_worldObjects[j]))) {
          *simulationGetFreeContact() = tentativeContact;
#ifdef DEBUG_SIM_ENABLED
          calcSeperatingVelocity(tentativeContact);
#endif
          debug("Coll: SepV:", DEBUG_SIM);
          debug(TO_STRING(tentativeContact.m_seperatingVelocity), DEBUG_SIM);
          debug(" Penetration:", DEBUG_SIM);
          debug(TO_STRING(tentativeContact.m_penetration), DEBUG_SIM);
          debugln(DEBUG_SIM);                               
        }
      }
    }
  }

  debugln(String(sim.m_activeContacts) + " active contacts", DEBUG_SIM);
  while(totalCollisions < 10) {
    int worstCollision = -1;
    FixedPoint worstCollisionVelocity = FP_MAX;

    for(int i = 0; i < sim.m_activeContacts; i++) {

      calcSeperatingVelocity(sim.m_worldContacts[i]);
      
      debug("Contact ", DEBUG_SIM);
      debug(i, DEBUG_SIM);
      debug(" has sep velocity ", DEBUG_SIM);
      debug(TO_STRING(sim.m_worldContacts[i].m_seperatingVelocity), DEBUG_SIM);
      debug(" and penetration ", DEBUG_SIM);
      debugln(TO_STRING(calculateCurrentPenetration(sim.m_worldContacts[i], false)), DEBUG_SIM);
      
      if((sim.m_worldContacts[i].m_seperatingVelocity < -EPSILON || calculateCurrentPenetration(sim.m_worldContacts[i]) > EPSILON) && sim.m_worldContacts[i].m_seperatingVelocity < worstCollisionVelocity) { 
        debug("Adding colision:", DEBUG_SIM);
        debug(TO_STRING(sim.m_worldContacts[i].m_seperatingVelocity), DEBUG_SIM);
        debug(",", DEBUG_SIM);
        debugln(TO_STRING(calculateCurrentPenetration(sim.m_worldContacts[i],false)), DEBUG_SIM);
        //Only bother adding contacts which are worse than the current worst, we will only calculate the worst in
        // each iteration.
        
        worstCollisionVelocity = sim.m_worldContacts[i].m_seperatingVelocity;
        worstCollision = i;
      }
    }
    
    if(sim.m_activeContacts == 0 || worstCollision == -1) {
      debugln("No Collisions", DEBUG_SIM);
      break;
    }
    debugln("Worst collision:" + String(worstCollision) + ", resolving", DEBUG_SIM);
    resolveContact(sim.m_worldContacts[worstCollision], sec);
    totalCollisions++;
  }

}


