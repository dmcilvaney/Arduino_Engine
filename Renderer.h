#pragma once

#include "Object.h"
#include "Vector.h"
#include "FixedPoint.h"
#include "Simulation.h"
#include "Defines.h"

#define SCREEN_X 20
#define SCREEN_Y 20
#define SCREEN_SCALE FROM_INT(100)
#define SCREEN_LOC(x, y) y*SCREEN_X + x
#define SCALE_X 1
#define SCALE_Y 1

#define FRAME_DELAY 100

inline void particleDraw(const Object &obj, bool screen[]) {
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
  for(int screenX = (TO_INT(x) - TO_INT(radius))* SCALE_X; screenX <= (TO_INT(x) + TO_INT(radius))* SCALE_X; screenX++) {
    for(int screenY = (TO_INT(y) - TO_INT(radius))* SCALE_Y; screenY <= (TO_INT(y) + TO_INT(radius))* SCALE_Y; screenY++) {
      if(screenX >= 0 && screenX < SCREEN_X && screenY >=0 && screenY < SCREEN_Y) {
        screen[SCREEN_LOC(screenX, screenY)] = true;
      }
    }
  }
}

void objectDraw(const Object &obj, bool screen[]) {
  switch (obj.m_objectType) {
    case PARTICLE:
      particleDraw(obj, screen);
      break;
    default:
      break;
  }  
}

bool screen[SCREEN_X*SCREEN_Y];

void drawScreen() {
  Serial.print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
  for(int x = 0; x < SCREEN_X+2; x++) {
    Serial.print('=');
  }
  Serial.println();
  for (int y = SCREEN_Y - 1; y>= 0; y--) {
    Serial.print('|');
    for (int x = 0; x < SCREEN_X; x++) {
      int arrayLoc = SCREEN_LOC(x, y);
      //Serial.println(arrayLoc);
      if(screen[arrayLoc]) {
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
  
  for(int i = 0; i < SCREEN_X*SCREEN_Y; i++) {
    screen[i] = false;
  }
  for (int i = 0; i < NUM_OBJECTS; i++) {
#ifdef DEBUG
    Serial.print("Checking object ");
    Serial.print(i);
#endif
    if (sim.m_worldObjects[i].m_inUse) {
      objectDraw(sim.m_worldObjects[i], screen);
    }
  }
  Serial.println();
  drawScreen();
  lastUpdate = millis();
}


