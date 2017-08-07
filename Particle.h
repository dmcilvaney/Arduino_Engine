#pragma once

#include "SimulationObjects.h"
#include "FixedPoint.h"
#include "Vector.h"

void particleIntegrate(Object& obj, const FixedPoint& timeDelta) {
  obj.m_position.addScaledVector(obj.m_velocity, timeDelta);

  Vector3D forceAcceleration = obj.m_acceleration;
  forceAcceleration.addScaledVector(obj.m_force, obj.m_invMass);
  
  obj.m_velocity.addScaledVector(forceAcceleration, timeDelta);
}

void particleGravityForce(const ForceObject& fo, const FixedPoint& timeDelta, const FixedPoint gravMult) {
  if(fo.m_obj->m_invMass > 0) {
    FixedPoint force = MULT(G, DIV(ONE, fo.m_obj->m_invMass));
    fo.m_obj->m_force += Vector3D(0,force,0)*gravMult;
  }
}

