#pragma once

#include "Object.h"
#include "Vector.h"
#include "FixedPoint.h"
#include "Simulation.h"
#include "Defines.h"

#define SCREEN_X 10
#define SCREEN_Y 10
#define SCREEN_SIZE(x,y) (((x*y) + 7) / 8)
#define SCREEN_SCALE FROM_INT(100)

#define SCREEN_BIT_NUMBER(i) (i & 0x7)
#define SCREEN_BIT(i) (1 << SCREEN_BIT_NUMBER(i))
#define SCREEN_INT_POSITION(i) (i>>3)
#define SCREEN_LOC(x, y) y*SCREEN_X + x

#define SCALE_X 1
#define SCALE_Y 1

#define FRAME_DELAY 50

int8_t screen[SCREEN_SIZE(SCREEN_X,SCREEN_Y)];

inline void setPixel(int x, int y) {
  if (x < 0 || y < 0 || x >= SCREEN_X || y >= SCREEN_X) {
    return;
  }
  int index = SCREEN_LOC(x,y);
  screen[SCREEN_INT_POSITION(index)] |= SCREEN_BIT(index);
  /*
  Serial.print('(');
  Serial.print(x);
  Serial.print(',');
  Serial.print(y);
  Serial.print(") -> (W:");
  Serial.print(SCREEN_INT_POSITION(index));
  Serial.print(",B:");
  Serial.print(SCREEN_BIT_NUMBER(index));
  Serial.print(')');
  */
}

inline void clearPixel(int x, int y) {
  if (x < 0 || y < 0 || x >= SCREEN_X || y >= SCREEN_X) {
    return;
  }
  int index = SCREEN_LOC(x,y);
  screen[SCREEN_INT_POSITION(index)] &= ~SCREEN_BIT(index);
}

inline void clearScreen() {
  
  for(int i = 0; i < SCREEN_SIZE(SCREEN_X,SCREEN_Y); i++) {
    screen[i] = 0;
  }
}

inline bool queryScreen(int x, int y) {
  int index = SCREEN_LOC(x,y);
  return screen[SCREEN_INT_POSITION(index)] & SCREEN_BIT(index);
}

void drawCircle(FixedPoint xFP, FixedPoint yFP, FixedPoint rFP) {

  //Midpoint circle algorithm
  int x0 = TO_INT(xFP);
  int y0 = TO_INT(yFP);
  int radius = TO_INT(rFP);
  int x = 0;
  int y = radius;
  int f = 1 - radius;
  int ddf_x = 1;
  int ddf_y = -2 * radius;

  /*Serial.println();
  Serial.print("Draw circle at (");
  Serial.print(x0);
  Serial.print(',');
  Serial.print(y0);
  Serial.print(") with radius ");
  Serial.println(radius);*/

  setPixel(x0, y0 + radius);
  setPixel(x0, y0 - radius);
  setPixel(x0 + radius, y0);
  setPixel(x0 - radius, y0);

  while(x < y) {
    if (f >= 0) {
      y--;
      ddf_y += 2;
      f += ddf_y;
    }
    x++;
    ddf_x += 2;
    f += ddf_x;
    
    setPixel(x0 + x, y0 + y);
    setPixel(x0 - x, y0 + y);
    setPixel(x0 + x, y0 - y);
    setPixel(x0 - x, y0 - y);
    setPixel(x0 + y, y0 + x);
    setPixel(x0 - y, y0 + x);
    setPixel(x0 + y, y0 - x);
    setPixel(x0 - y, y0 - x);
  }
}

void particleDraw(const Object &obj) {
  FixedPoint x = obj.m_position.m_x;
  FixedPoint y = obj.m_position.m_y;
#ifdef DEBUG
  Serial.print("FP X:");
  Serial.println(TO_INT(x));
  Serial.print("FP Y:");
  Serial.println(TO_INT(y));
#endif
  int screenX = TO_INT(x) ;
  int screenY = TO_INT(y) * SCALE_Y;
#ifdef DEBUG
  Serial.print(screenX);
  Serial.print(",");
  Serial.println(screenY);
#endif

  FixedPoint radius = obj.m_particleData.m_radius;
  drawCircle(x,y,radius);
  setPixel(TO_INT(x),TO_INT(y));
}

void objectDraw(const Object &obj) {
  switch (obj.m_objectType) {
    case PARTICLE:
      particleDraw(obj);
      break;
    default:
      break;
  }  
}

void drawScreen() {
  Serial.print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
  for(int x = 0; x < SCREEN_X+2; x++) {
    Serial.print('=');
  }
  Serial.println();
  for (int y = SCREEN_Y - 1; y>= 0; y--) {
    Serial.print('|');
    for (int x = 0; x < SCREEN_X; x++) {
      //Serial.println(arrayLoc);
      if(queryScreen(x,y)) {
        Serial.print('*');
      } else {
        Serial.print(' ');
      }
    }  
    Serial.println('|');
  }
  
  for(int x = 0; x < SCREEN_X+2; x++) {
    Serial.print('=');
  }
  Serial.println();
}

unsigned long lastUpdate = 0;
void render(const Simulation& sim) {

  if(millis() - lastUpdate < FRAME_DELAY) {
    return;
  }
  
  clearScreen();
  for (int i = 0; i < NUM_OBJECTS; i++) {
#ifdef DEBUG
    Serial.print("Checking object ");
    Serial.print(i);
#endif
    if (sim.m_worldObjects[i].m_inUse) {
      objectDraw(sim.m_worldObjects[i]);
    }
  }
  Serial.println();
  drawScreen();
  lastUpdate = millis();
}


