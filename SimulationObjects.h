#pragma once

enum ObjectType {PARTICLE};

#define NUM_OBJECTS 20

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
  };
};

struct Simulation {
  int m_numObjects;
  Object m_worldObjects[NUM_OBJECTS];
  unsigned long m_lastStepTime = 0;
};
