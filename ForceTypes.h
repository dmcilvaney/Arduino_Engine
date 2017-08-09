#pragma once
#include "FixedPoint.h"
#include "Vector.h"
#include "Defines.h"

struct GravityForceData {
  FixedPoint m_gravMult;
};

struct Vect{
  FixedPoint m_fixedEndpointX;
  FixedPoint m_fixedEndpointY;
  FixedPoint m_fixedEndpointZ;
};
struct SpringForceData {
  union {
    void* m_endpoint;
    Vect m_fixedEndpoint;
  };
  FixedPoint m_springConstant;
  FixedPoint m_restLength;
};

