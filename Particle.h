#include "Object.h"

struct Particle : Object
{
  public:
    Object()
      : m_location(Vector3D()),
        m_mass(ONE),
        m_invMass(ONE),
        m_restitution(ONE),
        m_velocity(Vector3D()),
        m_force(Vector3D()),
        m_type(POINT)
    {}
    Object(Vector3D location, FixedPoint mass, FixedPoint restitution, Vector3D velocity, FixedPoint angular_velocity,
           FixedPoint orientation, FixedPoint moment_of_inertia, ObjectType type)
      : m_location(location),
        m_mass(mass),
        m_invMass(DIV(ONE, mass)),
        m_restitution(restitution),
        m_velocity(velocity),
        m_force(Vector3D()),
        m_torque(0),
        m_angular_velocity(angular_velocity),
        m_orientation(orientation),
        m_moment_of_inertia(moment_of_inertia),
        m_type(type) {
    }

    FixedPoint m_restitution;
    Vector3D m_location;
    FixedPoint m_mass;
    FixedPoint m_invMass;
    Vector3D m_velocity;
    Vector3D m_force;

    FixedPoint m_torque;
    FixedPoint m_angular_velocity;
    FixedPoint m_orientation;
    FixedPoint m_moment_of_inertia;

    ObjectType m_type;
    ObjectData m_data;
};
