#pragma once

#include "SimulationObjects.h"
#include "Particle.h"
#include "Defines.h"
#include "debug.h"

void analogForce(ForceObject& fo, const FixedPoint& timeDelta) {
  int analogReading = analogRead(0);
  //Serial.print("Analog reading:");
  //Serial.println(analogReading);
  int forceInput = 1023 - analogReading;
  FixedPoint force = DIV(FROM_INT(forceInput), FROM_INT(1023));
  force = MULT(force, FROM_INT(100));
  Vector3D forceVector(0, force, 0);
  fo.m_obj->m_force += forceVector;
}

void buildAnalogForce(ForceObject *fo, Object* obj) {
  fo->m_obj = obj;
  fo->m_generator = analogForce;
}

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

    Serial.print("Fixed Spring end:");
    vector.print();
    Serial.println();

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
  debugln("BuildSpringForce", DEBUG_FORCE);
  debugln(endPoint->toString(), DEBUG_FORCE);
  fo->m_obj = obj;
  
  if(isFixedPoint) {
    debug("FixedPoint Spring", DEBUG_FORCE);
    debugln(endPoint->toString(), DEBUG_FORCE);
    
    fo->m_springData.m_fixedEndpoint.m_fixedEndpointX = endPoint->m_x;
    fo->m_springData.m_fixedEndpoint.m_fixedEndpointY = endPoint->m_y;
    fo->m_springData.m_fixedEndpoint.m_fixedEndpointZ = endPoint->m_z;

    debugln(TO_FLOAT(fo->m_springData.m_fixedEndpoint.m_fixedEndpointX), DEBUG_FORCE);
    debugln(TO_FLOAT(fo->m_springData.m_fixedEndpoint.m_fixedEndpointY), DEBUG_FORCE);
    debugln(TO_FLOAT(fo->m_springData.m_fixedEndpoint.m_fixedEndpointZ), DEBUG_FORCE);

    fo->m_generator = fixedSpringForce;
  } else {
    fo->m_springData.m_endpoint = (void*)endPoint;
    fo->m_generator = springForce;
  }
  fo->m_springData.m_springConstant = springConstant;
  fo->m_springData.m_restLength = springLength;
}

