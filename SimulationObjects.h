#pragma once

#include "ForceTypes.h"
#include "Defines.h"

enum ObjectType {NONE, PARTICLE};

#define NUM_OBJECTS 5
#define NUM_FORCES 5
#define NUM_COLLISIONS 3

struct Particle {
  FixedPoint m_radius;
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
  

  FixedPoint m_damping = ONE - EPSILON;

  ObjectType m_objectType;
  
  union {
    struct Particle m_particleData;
  };
};

struct ForceObject {
  void (*m_generator)(ForceObject&, const FixedPoint&);
  Object* m_obj = NULL;
  union {
    GravityForceData m_gravData;
    SpringForceData m_springData;
  };
};

struct ContactObject {
  Object* m_c1;
  Object* m_c2;

  FixedPoint m_restitution;
  Vector3D m_contactNormal;
  FixedPoint m_penetration;
  FixedPoint m_seperatingVelocity;
};

struct Simulation {
  int m_numObjects = NUM_OBJECTS;
  int m_numForces = NUM_FORCES;
  int m_numContacts = NUM_COLLISIONS;
  Object m_worldObjects[NUM_OBJECTS];
  ForceObject m_worldForces[NUM_FORCES];
  ContactObject m_worldContacts[NUM_COLLISIONS];
  unsigned long m_lastStepTime = 0;
};
