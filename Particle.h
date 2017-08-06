#pragma once

#include "SimulationObjects.h"
#include "FixedPoint.h"
#include "Vector.h"

void particleIntegrate(Object *obj, const FixedPoint& timeDelta) {  
  obj->m_position.addScaledVector(obj->m_velocity, timeDelta);
  obj->m_velocity.addScaledVector(obj->m_acceleration, timeDelta);  
}

