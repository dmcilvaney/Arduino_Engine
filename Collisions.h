#pragma once

#include "Particle.h"
#include "Defines.h"
#include "SimulationObjects.h"
#include "debug.h"

bool checkIfCollision(ContactObject* newContact, Object* o1, Object* o2) {
    if(o1->m_invMass == 0) {
      if(o2->m_invMass > 0) {
        Object* temp = o1;
        o1 = o2;
        o2 = temp;
      } else {
        return false;
      }      
    }
    switch (o1->m_objectType) {
      case PARTICLE:
        return particleCheckIfCollision(newContact, o1, o2);
        break;
      default:
        break;
    }
}

void calcSeperatingVelocity(ContactObject& contact) {
  Vector3D velocity = contact.m_c1->m_velocity;
  if( contact.m_c2 != NULL ) {
    velocity -= contact.m_c2->m_velocity;
  }
  contact.m_seperatingVelocity = velocity * contact.m_contactNormal;
}

FixedPoint calculateCurrentPenetration(ContactObject& contact, bool debugMode = false) {
  if(debugMode) { 
    debug("Penetration adjustment 1: ", DEBUG_COLLISION);
    debug(contact.m_c1->m_penetrationAdjustment.toString(), DEBUG_COLLISION);
    debug("Penetration adjustment 2: ", DEBUG_COLLISION);
    debugln(contact.m_c2->m_penetrationAdjustment.toString(), DEBUG_COLLISION);
  }
  return contact.m_penetration - (contact.m_c1->m_penetrationAdjustment *  contact.m_contactNormal) +  (contact.m_c2->m_penetrationAdjustment * contact.m_contactNormal);
}

void resolveContact(ContactObject& contact, const FixedPoint& timeDelta) {
  //Serial.println("Reslove");
  
  //Fix velocity
  const FixedPoint& seperatingVelocity = contact.m_seperatingVelocity;
  debug("SepVelocity:",DEBUG_COLLISION);
  debugln(TO_STRING(seperatingVelocity),DEBUG_COLLISION);

  FixedPoint totalMass = contact.m_c1->m_invMass + (contact.m_c2 != NULL ? contact.m_c2->m_invMass : 0);
  if (totalMass > 0) {    
    if(seperatingVelocity < 0) {
      debug("\tCollision-> Restitution:",DEBUG_COLLISION);
      debugln(TO_STRING(contact.m_restitution),DEBUG_COLLISION);
      FixedPoint newVelocity = MULT(-seperatingVelocity, contact.m_restitution);
      debug("\tCollision-> New V:",DEBUG_COLLISION);
      debugln(TO_STRING(newVelocity),DEBUG_COLLISION);
      FixedPoint delta = newVelocity - seperatingVelocity;    
  
      debug("\tCollision-> Total mass:",DEBUG_COLLISION);
      debugln(TO_STRING(totalMass),DEBUG_COLLISION);    
      debug("\tCollision-> Delta:",DEBUG_COLLISION);
      debugln(TO_STRING(delta),DEBUG_COLLISION);      
  
      FixedPoint impulse = DIV(delta, totalMass);
      Vector3D impulseVector = contact.m_contactNormal * impulse;

      contact.m_c1->m_velocity += impulseVector * contact.m_c1->m_invMass;
      debug("\tCollision-> C1 impulse:" + (impulseVector * contact.m_c1->m_invMass).toString(), DEBUG_COLLISION);
      if( contact.m_c2 != NULL ) {
        contact.m_c2->m_velocity += impulseVector * -(contact.m_c2->m_invMass);
        debug("  C2 impulse:" + (impulseVector * -contact.m_c2->m_invMass).toString(), DEBUG_COLLISION);
      }
      debugln(DEBUG_COLLISION);
    }  
    
    //Fix Penetration
    debug("\tCollision-> Penetration:", DEBUG_COLLISION);
    debugln(TO_STRING(contact.m_penetration), DEBUG_COLLISION);
    FixedPoint currentPenetration = calculateCurrentPenetration(contact);
    debug(" Collision-> Adjusted Penetration:", DEBUG_COLLISION);
    debugln(TO_STRING(currentPenetration), DEBUG_COLLISION);
    if(currentPenetration > 0) {
      Vector3D movementVectorPerMass = contact.m_contactNormal * DIV(currentPenetration, totalMass);
      
      Vector3D c1Movement = (movementVectorPerMass * contact.m_c1->m_invMass);
      
      debug("\tCollision-> C1 position update:" + (contact.m_c1->m_position).toString() + " by " + c1Movement.toString(), DEBUG_COLLISION);
      contact.m_c1->m_position += c1Movement;
      contact.m_c1->m_penetrationAdjustment += c1Movement;
      debugln(String(" to ") + (contact.m_c1->m_position).toString(), DEBUG_COLLISION);
      
      
      if( contact.m_c2 != NULL ) {
        Vector3D c2Movement = (movementVectorPerMass * -contact.m_c2->m_invMass);
        debug("\tCollision-> C2 position update:" + (contact.m_c2->m_position).toString() + " by " + c2Movement.toString(), DEBUG_COLLISION);
        contact.m_c2->m_position += c2Movement;
        contact.m_c2->m_penetrationAdjustment += c2Movement;
        debugln(String(" to ") + (contact.m_c2->m_position).toString(), DEBUG_COLLISION);
      }
      debugln(DEBUG_COLLISION);
    }
  }  
}

