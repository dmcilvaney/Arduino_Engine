#pragma once
#include "FixedPoint.h"

struct GravityForceData {
  FixedPoint m_gravMult;
};

struct SpringForceData {
  void* m_endpoint;
  FixedPoint m_springConstant;
  FixedPoint m_restLength;
};

