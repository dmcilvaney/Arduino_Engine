#pragma once

#include "SimulationObjects.h"
#include "Particle.h"



void gravityForce(ForceObject& fo, const FixedPoint& timeDelta) {
  if(fo.m_obj->m_invMass > 0) {
    switch (fo.m_obj->m_objectType) {
      case PARTICLE:
        particleGravityForce(fo, timeDelta, fo.m_gravData.m_gravMult);
        break;
      default:
        break;
    }
  }
}

void buildGravityForce(ForceObject *fo, Object* obj, FixedPoint gravMult = ONE) {
  fo->m_obj = obj;
  fo->m_generator = gravityForce;
  fo->m_gravData.m_gravMult = gravMult;
}

void springForce(ForceObject& fo, const FixedPoint& timeDelta) {
  if(fo.m_obj->m_invMass > 0) {
    switch (fo.m_obj->m_objectType) {
      case PARTICLE:
        particleSpringForce(fo, timeDelta, fo.m_springData.m_endpoint, fo.m_springData.m_springConstant, fo.m_springData.m_restLength);
        break;
      default:
        break;
    }
  }
}

void buildSpringForce(ForceObject *fo, Object* obj, Vector3D* endPoint, FixedPoint springLength, FixedPoint springConstant) {
  fo->m_obj = obj;
  fo->m_generator = springForce;
  fo->m_springData.m_endpoint = (void*)endPoint;
  fo->m_springData.m_springConstant = springConstant;
  fo->m_springData.m_restLength = springLength;
}

