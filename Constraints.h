#pragma once

#include "Particle.h"
#include "Defines.h"
#include "SimulationObjects.h"
#include "debug.h"

extern ContactObject* simulationGetFreeContact();

void rodConstraint(ConstraintObject& co) {
  //Serial.print("constraint");
  
  Vector3D posDelta =  co.m_obj1->m_position - co.m_obj2->m_position;
  FixedPoint currentLength = posDelta.magnitude();
  debug("Running RodConstraint of length ",DEBUG_CONSTRAINT);
  debug(TO_STRING(co.m_rodData.m_length), DEBUG_CONSTRAINT);
  debug(" and current length ",DEBUG_CONSTRAINT);
  debugln(TO_STRING(currentLength), DEBUG_CONSTRAINT);
  //co.m_obj1->print();
  //co.m_obj2->print();
  
  posDelta.normalize();
  
  //if( currentLength < co.m_rodData.m_length - EPSILON ) {
    //debug("Rod too short by ",DEBUG_CONSTRAINT);
    
    FixedPoint delta = co.m_rodData.m_length - currentLength;
    
    debugln(TO_STRING(delta), DEBUG_CONSTRAINT);
    
    ContactObject* contact = simulationGetFreeContact();
    contact->m_c1 = co.m_obj1;
    contact->m_c2 = co.m_obj2;
    contact->m_restitution = 0;
    contact->m_contactNormal = posDelta;
    contact->m_penetration = co.m_rodData.m_length - currentLength;
  //}

  //else if( currentLength > co.m_rodData.m_length + EPSILON) {
    //debugln("Rod too long by ",DEBUG_CONSTRAINT);
    //FixedPoint delta = currentLength - co.m_rodData.m_length;
    delta = currentLength - co.m_rodData.m_length;
    debugln(TO_STRING(delta), DEBUG_CONSTRAINT);
    //ContactObject* contact = simulationGetFreeContact();
    contact = simulationGetFreeContact();
    contact->m_c1 = co.m_obj1;
    contact->m_c2 = co.m_obj2;
    contact->m_restitution = 0;
    contact->m_contactNormal = posDelta*-ONE;
    contact->m_penetration = currentLength - co.m_rodData.m_length;
  //}

  //else {
  //  debugln("Rod fine",DEBUG_CONSTRAINT);
  //}
}

void buildRodConstraint(ConstraintObject *co, Object* obj1, Object* obj2, FixedPoint rodLength) {
  Serial.println("New constraint:");
  obj1->print();
  obj2->print();
  co->m_obj1 = obj1;
  co->m_obj2 = obj2;
  co->m_rodData.m_length = rodLength;
  co->m_generator = rodConstraint;
}
