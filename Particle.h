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

