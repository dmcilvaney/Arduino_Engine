#pragma once

#include "SimulationObjects.h"
#include "FixedPoint.h"
#include "Vector.h"
#include "Defines.h"

void particleIntegrate(Object& obj, const FixedPoint& timeDelta) {
#ifdef DEBUG
  Serial.print("Time:");
  Serial.println(TO_FLOAT(timeDelta));
  obj.m_position.print();
  obj.m_velocity.print();
  obj.m_acceleration.print();
  obj.m_force.print();
  Serial.println();
#endif
  obj.m_position.addScaledVector(obj.m_velocity, timeDelta);

  Vector3D forceAcceleration = obj.m_acceleration;
  forceAcceleration.addScaledVector(obj.m_force, obj.m_invMass);
  
  obj.m_velocity.addScaledVector(forceAcceleration, timeDelta);
  obj.m_velocity *= obj.m_damping;
#ifdef DEBUG
  obj.m_position.print();
  obj.m_velocity.print();
  forceAcceleration.print();
  obj.m_force.print();
#endif
}

void particleGravityForce(const ForceObject& fo, const FixedPoint& timeDelta, const FixedPoint gravMult) {
  if(fo.m_obj->m_invMass > 0) {
    FixedPoint force = MULT(G, DIV(ONE, fo.m_obj->m_invMass));
    fo.m_obj->m_force += Vector3D(0,force,0)*gravMult;
  }
}

void particleSpringForce(const ForceObject& fo, const FixedPoint& timeDelta, const void* endpoint, const FixedPoint& springConstant, const FixedPoint& restLength) {
  Vector3D displacement = fo.m_obj->m_position - *(Vector3D*)endpoint;
  FixedPoint forceMagnitude = MULT(ABS(displacement.magnitude() - restLength), springConstant);
  Vector3D force = displacement;
  force.normalize();
  force *= -forceMagnitude;
#ifdef DEBUG
  Serial.print("Displacement:");
  displacement.print();  
  Serial.print("Magnitude:");
  Serial.println(TO_FLOAT(forceMagnitude));
  Serial.print("Force:");
  force.print();
  Serial.println();
#endif
  
  fo.m_obj->m_force += force;
}

bool particleCheckIfCollision(ContactObject* newContact, Object* o1, Object* o2) {
  bool isCollision = false;
  switch (o2->m_objectType) {
    case PARTICLE: {
      Vector3D seperation = (o1->m_position - o2->m_position);
 //     seperation.print();
 //     Serial.print(TO_FLOAT(seperation.magnitude2()));
 //     Serial.print(',');
 //     Serial.println(TO_FLOAT((MULT(o1->m_particleData.m_radius,o1->m_particleData.m_radius) + MULT(o2->m_particleData.m_radius,o2->m_particleData.m_radius))));
      if (seperation.magnitude2() < (MULT(o1->m_particleData.m_radius,o1->m_particleData.m_radius) + MULT(o2->m_particleData.m_radius,o2->m_particleData.m_radius))) {
        isCollision = true;
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

  ObjectType m_type1;
  Object* m_c1;
  ObjectType m_type2;
  Object* m_c2;

  FixedPoint m_restitution;
  Vector3D m_contactNormal;
  FixedPoint m_penetration;
