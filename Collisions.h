#pragma once

#include "Particle.h"
#include "Defines.h"

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

FixedPoint calcSeperatingVelocity(const ContactObject& contact) {
  Vector3D velocity = contact.m_c1->m_velocity;
  if( contact.m_c2 != NULL ) {
    velocity -= contact.m_c2->m_velocity;
  }
  return velocity * contact.m_contactNormal;
}

void resolveContact(const ContactObject& contact, const FixedPoint& timeDelta) {
  //Serial.println("Reslove");
  //Fix velocity
  FixedPoint seperatingVelocity = calcSeperatingVelocity(contact);
  //Serial.print("SepVelocity:");
  //Serial.println(TO_FLOAT(seperatingVelocity));
  if(seperatingVelocity < 0) {
    FixedPoint newVelocity = MULT(-seperatingVelocity, contact.m_restitution);
    FixedPoint delta = newVelocity - seperatingVelocity;
    FixedPoint totalMass = contact.m_c1->m_invMass + (contact.m_c2 != NULL ? contact.m_c2->m_invMass : 0);

    //Serial.print("Total mass:");
    //Serial.println(TO_FLOAT(totalMass));
    
    //Serial.print("Delta:");
    //Serial.println(TO_FLOAT(delta));
    

    if (totalMass > 0) {
      FixedPoint impulse = DIV(delta, totalMass);
      Vector3D impulseVector = contact.m_contactNormal * impulse;

      contact.m_c1->m_velocity += impulseVector * contact.m_c1->m_invMass;
      //Serial.print("C1 impulse:");
      //(impulseVector * contact.m_c1->m_invMass).print();
      if( contact.m_c2 != NULL ) {
        contact.m_c2->m_velocity += impulseVector * -(contact.m_c1->m_invMass);
        //Serial.print("C1 impulse:");
        //(impulseVector * contact.m_c1->m_invMass).print();
      }
      //Serial.println();
    }    
  }  
}

