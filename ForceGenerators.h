#pragma once

#include "SimulationObjects.h"
#include "Particle.h"
#include "Defines.h"



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

void fixedSpringForce(ForceObject& fo, const FixedPoint& timeDelta) {
  if(fo.m_obj->m_invMass > 0) {
    Vector3D vector(fo.m_springData.m_fixedEndpoint.m_fixedEndpointX, fo.m_springData.m_fixedEndpoint.m_fixedEndpointY, fo.m_springData.m_fixedEndpoint.m_fixedEndpointZ);
#ifdef DEBUG
    Serial.print("Fixed Spring end:");
    vector.print();
    Serial.println();
#endif
    switch (fo.m_obj->m_objectType) {
      case PARTICLE:
        particleSpringForce(fo, timeDelta, &vector, fo.m_springData.m_springConstant, fo.m_springData.m_restLength);
        break;
      default:
        break;
    }
  }
}

void buildSpringForce(ForceObject *fo, Object* obj, Vector3D* endPoint, FixedPoint springLength, FixedPoint springConstant, bool isFixedPoint) {
#ifdef DEBUG
  Serial.println("BuildSpringForce");
  endPoint->print();
  Serial.println();
#endif
  fo->m_obj = obj;
  
  if(isFixedPoint) {
#ifdef DEBUG
    Serial.print("FP");
    endPoint->print();
    Serial.println();
#endif
    fo->m_springData.m_fixedEndpoint.m_fixedEndpointX = endPoint->m_x;
    fo->m_springData.m_fixedEndpoint.m_fixedEndpointY = endPoint->m_y;
    fo->m_springData.m_fixedEndpoint.m_fixedEndpointZ = endPoint->m_z;
    #ifdef DEBUG
    Serial.println(TO_FLOAT(fo->m_springData.m_fixedEndpoint.m_fixedEndpointX));
    Serial.println(TO_FLOAT(fo->m_springData.m_fixedEndpoint.m_fixedEndpointY));
    Serial.println(TO_FLOAT(fo->m_springData.m_fixedEndpoint.m_fixedEndpointZ));
    #endif
    fo->m_generator = fixedSpringForce;
  } else {
    fo->m_springData.m_endpoint = (void*)endPoint;
    fo->m_generator = springForce;
  }
  fo->m_springData.m_springConstant = springConstant;
  fo->m_springData.m_restLength = springLength;
}

