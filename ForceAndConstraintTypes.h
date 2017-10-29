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
struct CustomForceData {
  union {
    void *m_dynamicForceVector;
    Vect *m_fixedForceVector;
  };
};
struct RodConstraintData {
  FixedPoint m_length;
};
struct LeverConstraintData {
  FixedPoint m_leverLength;
  FixedPoint m_leverPivotPosition;
  union {
    void* m_pivotPoint;
    Vect m_pivotFixedPoint;
  };
};

