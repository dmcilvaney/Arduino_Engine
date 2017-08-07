#pragma once

#include "ForceTypes.h"

enum ObjectType {PARTICLE};

#define NUM_OBJECTS 5
#define NUM_FORCES 50

struct Particle {
  int i;
};


struct Object {
  bool m_inUse;
  
  //Motion variables
  Vector3D m_position;
  Vector3D m_velocity;
  Vector3D m_acceleration;
  
  Vector3D m_force;

  //Physical properties
  FixedPoint m_invMass;
  

  FixedPoint m_damping;

  ObjectType m_objectType;
  
  union {
    struct Particle m_particleData;
  } m_objectData;
};

struct ForceObject {
  void (*m_generator)(ForceObject&, const FixedPoint&);
  Object* m_obj = NULL;
  union {
    GravityForceData m_gravData;
    SpringForceData m_springData;
  };
};

struct Simulation {
  int m_numObjects = NUM_OBJECTS;
  int m_numForces = NUM_FORCES;
  Object m_worldObjects[NUM_OBJECTS];
  ForceObject m_worldForces[NUM_FORCES];
  unsigned long m_lastStepTime = 0;
};
