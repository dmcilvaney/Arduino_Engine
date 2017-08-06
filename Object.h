#pragma once

#include "FixedPoint.h"
#include "Vector.h"

struct Object
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

Object buildPoint(Vector3D location, FixedPoint mass, FixedPoint restitution, Vector3D velocity) {
  Object newObject(location, mass, restitution, velocity, 0, 0, FP_MAX, POINT);
  return newObject;
}

Object buildCircle(Vector3D location, FixedPoint mass, FixedPoint restitution, Vector3D velocity, FixedPoint angular_velocity,
                   FixedPoint orientation, FixedPoint radius) {
  FixedPoint moment_of_inertia = MULT(mass, MULT(radius, radius))>>1;
  Object newObject(location, mass, restitution, velocity, angular_velocity, orientation, moment_of_inertia, CIRCLE);
  newObject.m_data.m_circleData.radius = radius;
  return newObject;
}

Object buildBox(Vector3D location, FixedPoint mass, FixedPoint restitution, Vector3D velocity, FixedPoint angular_velocity,
                   FixedPoint orientation, FixedPoint h, FixedPoint w) {
  FixedPoint moment_of_inertia = DIV(MULT(mass,MULT(h,h) + MULT(w,w)), FROM_INT(12));
  Object newObject(location, mass, restitution, velocity, angular_velocity, orientation, moment_of_inertia, BOX);
  newObject.m_data.m_boxData.height = h;
  newObject.m_data.m_boxData.width = w;
  return newObject;
}

struct Manifold
{
  Object * a;
  Object * b;
  FixedPoint penetration;
  Vector3D normal;
};

bool isCollidingBox(Object a, Object b) {
  FixedPoint maxX1 = a.m_location.m_x + (a.m_data.m_boxData.width >> 2);
  FixedPoint maxX2 = b.m_location.m_x + (b.m_data.m_boxData.width >> 2);
  FixedPoint minX1 = a.m_location.m_x - (a.m_data.m_boxData.width >> 2);
  FixedPoint minX2 = b.m_location.m_x - (b.m_data.m_boxData.width >> 2);

  FixedPoint maxY1 = a.m_location.m_y + (a.m_data.m_boxData.height >> 2);
  FixedPoint maxY2 = b.m_location.m_y + (b.m_data.m_boxData.height >> 2);
  FixedPoint minY1 = a.m_location.m_y - (a.m_data.m_boxData.height >> 2);
  FixedPoint minY2 = b.m_location.m_y - (b.m_data.m_boxData.height >> 2);

  //Implement seperating axis theorem for bounding boxes.
  if (maxX1 < minX2 || minX1 > maxX2) return false;
  if (maxY1 < minY2 || minY1 > minY2) return false;

  return true;
}

//Does circle A collide with circle B?
bool isCollidingCircle(Object a, Object b) {
  FixedPoint r = a.m_data.m_circleData.radius + b.m_data.m_circleData.radius;
  r = MULT(r , r);
  FixedPoint dist1 = (a.m_location.m_x - b.m_location.m_x);
  dist1 = MULT(dist1, dist1);
  FixedPoint dist2 = (a.m_location.m_y - b.m_location.m_y);
  dist2 = MULT(dist2, dist2);
  FixedPoint dist = dist1 + dist2;

  return r > dist;
}

void colDebug(Object a, Object b) {
  FixedPoint r = a.m_data.m_circleData.radius + b.m_data.m_circleData.radius;
  r = MULT(r , r);
  FixedPoint dist1 = (a.m_location.m_x - b.m_location.m_x);
  dist1 = MULT(dist1, dist1);
  FixedPoint dist2 = (a.m_location.m_y - b.m_location.m_y);
  dist2 = MULT(dist2, dist2);
  FixedPoint dist = dist1 + dist2;
  Serial.print("r^2=");
  Serial.print(TO_FLOAT(r));
  Serial.print(", (d1+d2)^2=");
  Serial.print(TO_FLOAT(dist));
}

//newVelX1 = (firstBall.speed.x * (firstBall.mass – secondBall.mass) + (2 * secondBall.mass * secondBall.speed.x)) / (firstBall.mass + secondBall.mass);
//newVelY1 = (firstBall.speed.y * (firstBall.mass – secondBall.mass) + (2 * secondBall.mass * secondBall.speed.y)) / (firstBall.mass + secondBall.mass);
//newVelX2 = (secondBall.speed.x * (secondBall.mass – firstBall.mass) + (2 * firstBall.mass * firstBall.speed.x)) / (firstBall.mass + secondBall.mass);
//newVelY2 = (secondBall.speed.y * (secondBall.mass – firstBall.mass) + (2 * firstBall.mass * firstBall.speed.y)) / (firstBall.mass + secondBall.mass);


void resolveColissionCircleCircle(Object &a, Object &b, FixedPoint dt) {
  colDebug(a, b);
  Serial.print("LocA:");
  a.m_location.print();
  //Serial.print(a.m_location.m_x);
  //Serial.print(',');
  //Serial.println(a.m_location.m_y);
  Serial.print("LocB:");
  b.m_location.print();

  //Serial.print(a.m_location.m_x);
  //Serial.print(',');
  //Serial.print(a.m_location.m_y);
  //Serial.println();
  Serial.print("VelocityA:");
  a.m_velocity.print();
  Serial.print("VelocityB:");
  b.m_velocity.print();
  Serial.println();

  FixedPoint aVX = DIV(MULT(a.m_velocity.m_x, (a.m_mass - b.m_mass)) + MULT(FROM_INT(2), MULT(b.m_mass, b.m_velocity.m_x)), a.m_mass + b.m_mass);
  FixedPoint aVY = DIV(MULT(a.m_velocity.m_y, (a.m_mass - b.m_mass)) + MULT(FROM_INT(2), MULT(b.m_mass, b.m_velocity.m_y)), a.m_mass + b.m_mass);
  FixedPoint bVX = DIV(MULT(b.m_velocity.m_x, (b.m_mass - a.m_mass)) + MULT(FROM_INT(2), MULT(a.m_mass, a.m_velocity.m_x)), b.m_mass + a.m_mass);
  FixedPoint bVY = DIV(MULT(b.m_velocity.m_y, (b.m_mass - a.m_mass)) + MULT(FROM_INT(2), MULT(a.m_mass, a.m_velocity.m_y)), b.m_mass + a.m_mass);

  /*Serial.print(aVX);
    Serial.print(',');
    Serial.print(aVY);
    Serial.print(',');
    Serial.print(bVX);
    Serial.print(',');
    Serial.print(bVY);
    Serial.println();
  */

  a.m_velocity.m_x = aVX;
  a.m_velocity.m_y = aVY;
  b.m_velocity.m_x = bVX;
  b.m_velocity.m_y = bVY;

  Serial.print("VelocityA:");
  a.m_velocity.print();
  Serial.print("VelocityB:");
  b.m_velocity.print();
  Serial.println();

  Vector3D disp1;
  disp1.m_x = MULT(dt, a.m_velocity.m_x);
  disp1.m_y = MULT(dt, a.m_velocity.m_y);

  Vector3D disp2;
  disp2.m_x = MULT(dt, b.m_velocity.m_x);
  disp2.m_y = MULT(dt, b.m_velocity.m_y);

  Serial.print("d1=");
  disp1.print();
  Serial.print("d2=");
  disp2.print();

  a.m_location.m_x += MULT(dt, a.m_velocity.m_x);
  a.m_location.m_y += MULT(dt, a.m_velocity.m_y);
  b.m_location.m_x += MULT(dt, b.m_velocity.m_x);
  b.m_location.m_y += MULT(dt, b.m_velocity.m_y);

  Serial.print("LocA:");
  a.m_location.print();
  Serial.print("LocB:");
  b.m_location.print();
  colDebug(a, b);
  //for(;;);
}

bool isColliding(Object a, Object b) {
  switch (a.m_type) {
    case (BOX) :
      switch (b.m_type) {
        case BOX:
          return isCollidingBox(a, b);
        case CIRCLE:
          break;
        default:
          return false;
      }
      break;
    case (CIRCLE) :
      switch (b.m_type) {
        case BOX:
          break;
        case CIRCLE:
          return isCollidingCircle(a, b);
        default:
          return false;
      }
      break;
    default:
      return false;
  }
  return false;
}
