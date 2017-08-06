#pragma once

#include "Object.h"
#include "Vector.h"
#include "FixedPoint.h"
#include "Simulation.h"

#define SCREEN_X 40
#define SCREEN_Y 20
#define SCREEN_SCALE FROM_INT(100)
#define SCREEN_LOC(x, y) y*SCREEN_X + x
#define SCALE_X 2
#define SCALE_Y 1

struct Screen {
  bool Screen[SCREEN_X*SCREEN_Y];
};

inline void particleDraw(const Object &obj, Screen& screen) {
    FixedPoint x = obj.m_position.m_x;
    FixedPoint y = obj.m_position.m_y;

    Serial.print("FP X:");
    Serial.println(TO_INT(x));
    Serial.print("FP Y:");
    Serial.println(TO_INT(y));

    int screenX = TO_INT(x) * SCALE_X;
    int screenY = TO_INT(y) * SCALE_Y;

    Serial.print(screenX);
    Serial.print(",");
    Serial.print(screenY);

    if(screenX >= 0 && screenX < SCREEN_X && screenY >=0 && screenY < SCREEN_Y) {
      screen.Screen[SCREEN_LOC(screenX, screenY)] = true;
    }
}

void objectDraw(const Object &obj, Screen& screen) {
  switch (obj.m_objectType) {
    case PARTICLE:
      particleDraw(obj, screen);
      break;
    default:
      break;
  }  
}

void drawScreen(const Screen& screen) {
  Serial.print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
  for(int x = 0; x < SCREEN_X+2; x++) {
    Serial.print('=');
  }
  Serial.println();
  for (int y = SCREEN_Y-1; y >= 0; y--) {
    Serial.print('|');
    for (int x = 0; x < SCREEN_X; x++) {
      int arrayLoc = SCREEN_LOC(x, y);
      //Serial.println(arrayLoc);
      if(screen.Screen[arrayLoc]) {
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

void render(const Simulation& sim) {
  Screen screen;
  for(int i = 0; i < SCREEN_X*SCREEN_Y; i++) {
    screen.Screen[i] = false;
  }
  for (int i = 0; i < NUM_OBJECTS; i++) {
    Serial.print("Checking object ");
    Serial.print(i);
    if (sim.m_worldObjects[i].m_inUse) {
      Serial.print("*");
      objectDraw(sim.m_worldObjects[i], screen);
    }
    Serial.println();
  }
  drawScreen(screen);
}


