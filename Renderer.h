#pragma once

#include "Object.h"
#include "Vector.h"
#include "FixedPoint.h"
#include "Simulation.h"
#include "Defines.h"

#define SCREEN_X 20
#define SCREEN_Y 20
#define SCREEN_SIZE(x,y) (((x*y) + 7) / 8)
#define SCREEN_SCALE FROM_INT(50)

#define SCREEN_BIT_NUMBER(i) (i & 0x7)
#define SCREEN_BIT(i) (1 << SCREEN_BIT_NUMBER(i))
#define SCREEN_INT_POSITION(i) (i>>3)
#define SCREEN_LOC(x, y) y*SCREEN_X + x

#define FRAME_DELAY 70

int8_t screen1[SCREEN_SIZE(SCREEN_X,SCREEN_Y)];
int8_t screen2[SCREEN_SIZE(SCREEN_X,SCREEN_Y)];

inline void setPixel(int x, int y, int screenNum = 1) {
  if (x < 0 || y < 0 || x >= SCREEN_X || y >= SCREEN_X) {
    return;
  }
  int index = SCREEN_LOC(x,y);
  switch(screenNum) {
    case 1:
      screen1[SCREEN_INT_POSITION(index)] |= SCREEN_BIT(index);
      break;
    case 2:
      screen2[SCREEN_INT_POSITION(index)] |= SCREEN_BIT(index);
      break;
    default:
      return;
  }
}

inline void clearPixel(int x, int y, int screenNum) {
  if (x < 0 || y < 0 || x >= SCREEN_X || y >= SCREEN_X) {
    return;
  }
  int index = SCREEN_LOC(x,y);
  switch(screenNum) {
    case 1:
      screen1[SCREEN_INT_POSITION(index)] &= ~SCREEN_BIT(index);
      break;
    case 2:
      screen2[SCREEN_INT_POSITION(index)] &= ~SCREEN_BIT(index);
      break;
    default:
      return;
  }
}

inline void clearScreen() {
  
  for(int i = 0; i < SCREEN_SIZE(SCREEN_X,SCREEN_Y); i++) {
    screen1[i] = 0;
    screen2[i] = 0;
  }
}

inline bool queryScreen(int x, int y, int screenNum) {
  int index = SCREEN_LOC(x,y);
  switch(screenNum) {
    case 1:
      return screen1[SCREEN_INT_POSITION(index)] & SCREEN_BIT(index);
      break;
    case 2:
      return screen2[SCREEN_INT_POSITION(index)] & SCREEN_BIT(index);
      break;
    default:
      return false;
  }  
}

void drawLine(FixedPoint x1FP, FixedPoint y1FP, FixedPoint x2FP, FixedPoint y2FP) {
  x1FP = MULT(x1FP, SCREEN_SCALE);
  y1FP = MULT(y1FP, SCREEN_SCALE);
  x2FP = MULT(x2FP, SCREEN_SCALE);
  y2FP = MULT(y2FP, SCREEN_SCALE);
  
  FixedPoint deltaX = x2FP - x1FP;
  FixedPoint deltaY = y2FP - y1FP;

  FixedPoint xSteps = ABS(deltaX);
  FixedPoint ySteps = ABS(deltaY);

  FixedPoint dX, dY;
  FixedPoint x = x1FP;
  FixedPoint y = y1FP;

  FixedPoint steps;
  if(xSteps > ySteps) {
    steps = xSteps;
  } else {
    steps = ySteps;
  }
  dX = DIV(deltaX,steps);
  dY = DIV(deltaY,steps);
  for(int i = 0; i < TO_INT(steps); i++) {
    setPixel(x,y,2);
    x+=dX;
    y+=dY;
  }
}

void drawCircle(FixedPoint xFP, FixedPoint yFP, FixedPoint rFP) {
  xFP = MULT(xFP, SCREEN_SCALE);
  yFP = MULT(yFP, SCREEN_SCALE);
  rFP = MULT(rFP, SCREEN_SCALE);
  
  //Midpoint circle algorithm
  int x0 = TO_INT(xFP);
  int y0 = TO_INT(yFP);
  int radius = TO_INT(rFP);
  int x = 0;
  int y = radius;
  int f = 1 - radius;
  int ddf_x = 1;
  int ddf_y = -2 * radius;
  
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
  int screenY = TO_INT(y);
#ifdef DEBUG
  Serial.print(screenX);
  Serial.print(",");
  Serial.println(screenY);
#endif

  FixedPoint radius = obj.m_particleData.m_radius;
  drawCircle(x,y,radius);
  setPixel(TO_INT(MULT(x, SCREEN_SCALE)),TO_INT(MULT(y, SCREEN_SCALE)));
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
    //Serial.print('=');
    Serial.print('=');
  }
  Serial.println();
  for (int y = SCREEN_Y - 1; y>= 0; y--) {
    //Serial.print('|');
    Serial.print('|');
    for (int x = 0; x < SCREEN_X; x++) {
      //Serial.println(arrayLoc);
      if(queryScreen(x,y,2)) {
        Serial.print('0');        
      } else if (queryScreen(x,y,1)) {
        Serial.print('*');
      } else {
        //Serial.print(' ');
        Serial.print(' ');
      }
    }
    //Serial.print('|');
    Serial.println('|');    
  }
  
  for(int x = 0; x < SCREEN_X+2; x++) {
    //Serial.print('=');
    Serial.print('=');
  }
  Serial.println();
}

unsigned long lastUpdate = 0;
void render(const Simulation& sim) {
  PROFILE_ON(PROFILE_RENDER);
  if(millis() - lastUpdate < FRAME_DELAY) {
    PROFILE_OFF(PROFILE_RENDER);
    return;
  }  
  lastUpdate = millis();
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
  for (int i = 0; i < NUM_CONSTRAINTS; i++) {
    if (sim.m_worldConstraints[i].m_generator != NULL) {
      drawLine(sim.m_worldConstraints[i].m_obj1->m_position.m_x, sim.m_worldConstraints[i].m_obj1->m_position.m_y, sim.m_worldConstraints[i].m_obj2->m_position.m_x, sim.m_worldConstraints[i].m_obj2->m_position.m_y);
    }
  }
  Serial.println();
  drawScreen();
  PROFILE_OFF(PROFILE_RENDER);
}


