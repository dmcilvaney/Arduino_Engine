#pragma once

#include "FixedPoint.h"
#include "Defines.h"

#define ADD_VECT(a,b) Vector3D(a.m_x + b.m_x, a.m_y+b.m_y, a.m_z + b.m_z)
#define MULT_VECT_SCAL(a,b) Vector3D(MULT(a.m_x, b), MULT(a.m_y, b), MULT(a.m_z, b))

struct Vector3D
{  
  FixedPoint m_x = 0;
  FixedPoint m_y = 0;
  FixedPoint m_z = 0;

  Vector3D()
    : m_x(0),
      m_y(0),
      m_z(0) {
  }

  Vector3D(FixedPoint x, FixedPoint y, FixedPoint z)
    : m_x(x),
      m_y(y),
      m_z(z) {
  }

  FixedPoint magnitude() const {
    return sRoot(MULT(m_x, m_x) + MULT(m_y, m_y)  + MULT(m_z, m_z) );
  }

  FixedPoint magnitude2() const {
    return MULT(m_x, m_x) + MULT(m_y, m_y)  + MULT(m_z, m_z);
  }

  void normalize() {
    FixedPoint mag = magnitude();
    if (mag != 0) {
      m_x = DIV(m_x, mag);
      m_y = DIV(m_y, mag);
      m_z = DIV(m_z, mag);
    }    
  }

  void invert() {
    m_x = -m_x;
    m_y = -m_y;
    m_z = -m_z;
  }
  
  void print() const {
    Serial.print("(");
    Serial.print(TO_FLOAT(m_x));
    Serial.print(",");
    Serial.print(TO_FLOAT(m_y));
    Serial.print(",");
    Serial.print(TO_FLOAT(m_z));
    Serial.print(")");
  }


  //  ***(scalar)
  void operator*=(const FixedPoint scal) {
      m_x = MULT(m_x, scal);
      m_y = MULT(m_y, scal);
      m_z = MULT(m_z, scal);
  }
  Vector3D operator*(const FixedPoint scal) const {
    return Vector3D(MULT(m_x, scal), MULT(m_y, scal), MULT(m_z, scal));
  }
  //  ***(vector)
  Vector3D componentProduct(const Vector3D& vector) const {
    return Vector3D(MULT(m_x, vector.m_x), MULT(m_y, vector.m_y), MULT(m_z, vector.m_z));
  }
  FixedPoint operator*(const Vector3D &vector) const {
    return MULT(m_x, vector.m_x) + MULT(m_y, vector.m_y) + MULT(m_z, vector.m_z);
  }

  //  %%%
  Vector3D operator%(const Vector3D& vector) const {
    return Vector3D( MULT(m_y, vector.m_z) - MULT(m_z, vector.m_y), MULT(m_z, vector.m_x) - MULT(m_x, vector.m_z), MULT(m_x, vector.m_y) - MULT(m_y, vector.m_x));
  }
  void operator%=(const Vector3D& vector) {
    *this = (*this)%vector;
  }

  //  +++
  void operator+=(const Vector3D& vector) {
    m_x += vector.m_x;
    m_y += vector.m_y;
    m_z += vector.m_z;
  }
  Vector3D operator+(const Vector3D& vector) const {
    return Vector3D(m_x + vector.m_x, m_y + vector.m_y, m_z + vector.m_z);
  }
  void addScaledVector(const Vector3D& vector, FixedPoint scal) {
    m_x += MULT(vector.m_x, scal);
    m_y += MULT(vector.m_y, scal);
    m_z += MULT(vector.m_z, scal);
  }

  //  ---
  void operator-=(const Vector3D& vector) {
    m_x -= vector.m_x;
    m_y -= vector.m_y;
    m_z -= vector.m_z;
  }
  Vector3D operator-(const Vector3D& vector) const {
    return Vector3D(m_x - vector.m_x, m_y - vector.m_y, m_z - vector.m_z);
  }
};


// Misc util classes
//===============================================
void makeOrthonormalBasis(Vector3D *a, Vector3D *b, Vector3D *c) {
  a->normalize();
  //Set c to be orthoganal to both a and b.
  (*c) = (*a)%(*b);
  //If c is 0 then a,b were parallel.
  if(c->magnitude2() != 0) {
    c->normalize();
    (*b) = (*c) % (*a);
  }
}

