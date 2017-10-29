#pragma once

#include "ForceAndConstraintTypes.h"
#include "Defines.h"

enum ObjectType {NONE, PARTICLE};

#define NUM_OBJECTS 4
#define NUM_FORCES 7
#define NUM_COLLISIONS 7
#define NUM_CONSTRAINTS 1

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
  

  FixedPoint m_damping = FROM_INT_SHIFT(99,2);

  Vector3D m_penetrationAdjustment;

  ObjectType m_objectType;
  
  union {
    struct Particle m_particleData;
  };

  void print() {
    Serial.print("Object at ");
    m_position.print();
    Serial.print(" moving at ");
    m_velocity.print();
    Serial.println();
  }
};

struct ForceObject {
  void (*m_generator)(ForceObject&, const FixedPoint&);
  Object* m_obj = NULL;
  union {
    GravityForceData m_gravData;
    SpringForceData m_springData;
    CustomForceData m_analogForceData;
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

struct ConstraintObject {
  void (*m_generator)(ConstraintObject&);
  Object* m_obj1 = NULL;
  Object* m_obj2 = NULL;  
  union {
    RodConstraintData m_rodData;
  };
};

struct Simulation {
  int m_numObjects = NUM_OBJECTS;
  int m_numForces = NUM_FORCES;
  int m_numContacts = NUM_COLLISIONS;
  int m_numConstraints = NUM_CONSTRAINTS;
  
  Object m_worldObjects[NUM_OBJECTS];
  ForceObject m_worldForces[NUM_FORCES];
  int m_activeContacts = 0;
  ContactObject m_worldContacts[NUM_COLLISIONS];
  ConstraintObject m_worldConstraints[NUM_CONSTRAINTS];
  unsigned long m_simulationTime  = 0;
};
