#pragma once

#include "SimulationObjects.h"
#include "FixedPoint.h"
#include "Vector.h"
#include "Defines.h"

void particleIntegrate(Object& obj, const FixedPoint& timeDelta) {
  Serial.println("PARTICLE INTEGRATE");
  Serial.print("Time:");
  Serial.println(TO_STRING(timeDelta));
  obj.m_position.print();
  obj.m_velocity.print();
  obj.m_acceleration.print();
  obj.m_force.print();
  Serial.println();
  
  obj.m_position.addScaledVector(obj.m_velocity, timeDelta);

  Vector3D forceAcceleration = obj.m_acceleration;
  forceAcceleration.addScaledVector(obj.m_force, obj.m_invMass);
  
  obj.m_velocity.addScaledVector(forceAcceleration, timeDelta);
  obj.m_velocity *= obj.m_damping;
  
  obj.m_position.print();
  obj.m_velocity.print();
  forceAcceleration.print();
  obj.m_force.print();
  Serial.println();

  //Serial.println("/OBJECT");  
}

void particleGravityForce(const ForceObject& fo, const FixedPoint& timeDelta, const FixedPoint gravMult) {
  if(fo.m_obj->m_invMass > 0) {
    FixedPoint force = MULT(G, DIV(ONE, fo.m_obj->m_invMass));
    fo.m_obj->m_force -= Vector3D(0,force,0)*gravMult;
  }
}

void particleSpringForce(const ForceObject& fo, const FixedPoint& timeDelta, const void* endpoint, const FixedPoint& springConstant, const FixedPoint& restLength) {
  //Serial.println("Particle spring");
  Vector3D displacement = fo.m_obj->m_position - (*(Vector3D*)endpoint);
  FixedPoint forceMagnitude = MULT(ABS(displacement.magnitude() - restLength), springConstant);
  //displacement.print();
  //Serial.println(TO_STRING(forceMagnitude));
  Vector3D force = displacement;
  force.normalize();
  force *= -forceMagnitude;
#ifdef DEBUG
  Serial.print("Displacement:");
  displacement.print();  
  Serial.print("Magnitude:");
  Serial.println(TO_STRING(forceMagnitude));
  Serial.print("Force:");
  force.print();
  Serial.println();
#endif
  
  fo.m_obj->m_force += force;


  displacement.normalize();
  Vector3D dampingForce = displacement * (fo.m_obj->m_velocity * displacement) * FROM_INT(5);
  fo.m_obj->m_force -= dampingForce;
  
}

bool particleCheckIfCollision(ContactObject* newContact, Object* o1, Object* o2) {
  bool isCollision = false;
  switch (o2->m_objectType) {
    case PARTICLE: {
      Vector3D seperation = (o1->m_position - o2->m_position);
      FixedPoint sumRadius = o1->m_particleData.m_radius + o2->m_particleData.m_radius;
      
      if (seperation.magnitude2() < MULT(sumRadius,sumRadius)) {
        isCollision = true;
        newContact->m_penetration = sumRadius - seperation.magnitude();
        seperation.normalize();
        newContact->m_contactNormal = seperation;        
      }
      break;
    }
    default:
      break;
  }
  if(isCollision) {
    newContact->m_c1 = o1;
    newContact->m_c2 = o2;
    newContact->m_restitution = POINT_FIVE;
  }
  return isCollision;
}
