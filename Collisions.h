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

void resolveContact(const ContactObject& contact, const FixedPoint& timeDelta) {
  //Serial.println("Reslove");
  
  //Fix velocity
  const FixedPoint& seperatingVelocity = contact.m_seperatingVelocity;
  debug("SepVelocity:",DEBUG_COLLISION);
  debugln(TO_FLOAT(seperatingVelocity),DEBUG_COLLISION);

  FixedPoint totalMass = contact.m_c1->m_invMass + (contact.m_c2 != NULL ? contact.m_c2->m_invMass : 0);
  if (totalMass > 0) {    
    if(seperatingVelocity < 0) {
      debug("Restitution:",DEBUG_COLLISION);
      debugln(TO_FLOAT(contact.m_restitution),DEBUG_COLLISION);
      FixedPoint newVelocity = MULT(-seperatingVelocity, contact.m_restitution);
      FixedPoint delta = newVelocity - seperatingVelocity;    
  
      debug("Total mass:",DEBUG_COLLISION);
      debugln(TO_FLOAT(totalMass),DEBUG_COLLISION);    
      debug("Delta:",DEBUG_COLLISION);
      debugln(TO_FLOAT(delta),DEBUG_COLLISION);      
  
      FixedPoint impulse = DIV(delta, totalMass);
      Vector3D impulseVector = contact.m_contactNormal * impulse;

      contact.m_c1->m_velocity += impulseVector * contact.m_c1->m_invMass;
      debug("C1 impulse:" + (impulseVector * contact.m_c1->m_invMass).toString(), DEBUG_COLLISION);
      if( contact.m_c2 != NULL ) {
        contact.m_c2->m_velocity += impulseVector * -(contact.m_c2->m_invMass);
        debug("C2 impulse:" + (impulseVector * contact.m_c2->m_invMass).toString(), DEBUG_COLLISION);
      }
      debugln(DEBUG_COLLISION);
    }  
    //Fix Penetration
    debug("Pen:", DEBUG_COLLISION);
    debugln(TO_FLOAT(contact.m_penetration), DEBUG_COLLISION);
    if(contact.m_penetration > 0) {
      Vector3D movementVectorPerMass = contact.m_contactNormal * DIV(contact.m_penetration, totalMass);
      
      contact.m_c1->m_position += movementVectorPerMass * contact.m_c1->m_invMass;
      debug("C1 position update:" + (movementVectorPerMass * contact.m_c1->m_invMass).toString(), DEBUG_COLLISION);
      
      if( contact.m_c2 != NULL ) {
        Vector3D c2Movement = (movementVectorPerMass * contact.m_c2->m_invMass) * -ONE;
        debug(c2Movement.toString(), DEBUG_COLLISION);
        contact.m_c2->m_position += c2Movement;
        debug("C2 position update:" + c2Movement.toString(), DEBUG_COLLISION);
      }
      debugln(DEBUG_COLLISION);
    }
  }  
}

