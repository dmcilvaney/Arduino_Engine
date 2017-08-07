#pragma once

struct GravityForceData {
  FixedPoint m_gravMult;
};

struct SpringForceData {
  void* m_endpoint;
};

