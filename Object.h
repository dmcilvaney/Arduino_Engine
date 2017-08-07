#pragma once

#include "FixedPoint.h"
#include "Vector.h"
#include "SimulationObjects.h"

#include "Particle.h"

void integrateObject(Object &obj, const FixedPoint& timeDelta) {
  if(obj.m_invMass <= 0) {
    return;
  }
  
  switch (obj.m_objectType) {
    case PARTICLE:
      particleIntegrate(obj, timeDelta);
      break;
    default:
      break;
  }

  //Clear all forces.
  obj.m_force = Vector3D(0,0,0);
}

