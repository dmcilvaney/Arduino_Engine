#pragma once

//#define DEBUG_GENERAL_ENABLED
//#define DEBUG_CONSTRAINT_ENABLED
//#define DEBUG_COLLISION_ENABLED
//#define DEBUG_SIM_ENABLED
//#define DEBUG_FORCE_ENABLED

enum DebugType {DEBUG_GENERAL, DEBUG_COLLISION, DEBUG_CONSTRAINT, DEBUG_FIXEDPOINT, DEBUG_FORCE, DEBUG_PARTICLE, DEBUG_RENDERER, DEBUG_SIM, DEBUG_VECTOR};

enum ProfileType {PROFILE_SIM=2, PROFILE_RENDER=3, PROFILE_COLLISION=4};

#define DEBUG_ON (defined( DEBUG_GENERAL_ENABLED) || defined( DEBUG_COLLISION_ENABLED)  || defined( DEBUG_FIXEDPOINT_ENABLED)  || defined( DEBUG_FORCE_ENABLED)  || defined( DEBUG_RENDERER_ENABLED)  || defined( DEBUG_SIM_ENABLED) ||  defined( DEBUG_VECTOR_ENABLED) ||  defined( DEBUG_CONSTRAINT_ENABLED))
#if DEBUG_ON
#define debug(s, c) (debug_internal(s, c))
#define debugln(s, c) (debugln_internal(s, c))
#else
#define debug(s, c)
#define debugln(s, c)
#endif

inline void PROFILE_INIT() {
  pinMode(PROFILE_SIM, OUTPUT);
  digitalWrite(PROFILE_SIM, LOW);
  pinMode(PROFILE_RENDER, OUTPUT);
  digitalWrite(PROFILE_RENDER, LOW);
  pinMode(PROFILE_COLLISION, OUTPUT);
  digitalWrite(PROFILE_COLLISION, LOW);
}

inline void PROFILE_ON(ProfileType type) {
      digitalWrite(type,HIGH);
}
inline void PROFILE_OFF(ProfileType type) {
      digitalWrite(type,LOW);
}

inline void debug_internal(const String &string, const DebugType &debugType, const bool newLine = false) {
#if DEBUG_ON
  switch (debugType) {
#ifdef DEBUG_GENERAL_ENABLED
      case(DEBUG_GENERAL):
        Serial.print(string);
        if (newLine) {
          Serial.println();
        }
        break;
#endif
#ifdef DEBUG_COLLISION_ENABLED
      case(DEBUG_COLLISION):
        Serial.print(string);
        if (newLine) {
          Serial.println();
        }
        break;
#endif
#ifdef DEBUG_FIXEDPOINT_ENABLED
      case(DEBUG_FIXEDPOINT):
        Serial.print(string);
        if (newLine) {
          Serial.println();
        }
        break;
#endif
#ifdef DEBUG_FORCE_ENABLED
      case(DEBUG_FORCE):
        Serial.print(string);
        if (newLine) {
          Serial.println();
        }
        break;
#endif
#ifdef DEBUG_RENDERER_ENABLED
      case(DEBUG_RENDERER):
        Serial.print(string);
        if (newLine) {
          Serial.println();
        }
        break;
#endif
#ifdef DEBUG_SIM_ENABLED
      case(DEBUG_SIM):
        Serial.print(string);
        if (newLine) {
          Serial.println();
        }
        break;
#endif
#ifdef DEBUG_VECTOR_ENABLED
      case(DEBUG_VECTOR):
        Serial.print(string);
        if (newLine) {
          Serial.println();
        }
        break;
#endif
#ifdef DEBUG_CONSTRAINT_ENABLED
      case(DEBUG_CONSTRAINT):
        Serial.print(string);
        if (newLine) {
          Serial.println();
        }
        break;
#endif
      default:
        break;
  }
#endif
}

#ifndef FLOATING_POINT_MODE
inline void debug_internal(const float &floatVal, const DebugType &debugType) {
  debug_internal(String(floatVal), debugType);
}
#endif

inline void debug_internal(const FixedPoint &fpVal, const DebugType &debugType) {
#if FIXED_SIZE == 64
  debug_internal(format64(fpVal), debugType);
#else
  debug_internal(String(fpVal), debugType);
#endif
}

#if FIXED_SIZE != 16
inline void debug_internal(const int &intVal, const DebugType &debugType) {
  debug_internal(String(intVal), debugType);
}
#endif

inline void debugln_internal(const String &string, const DebugType &debugType) {
  debug_internal(string, debugType, true);
}
#ifndef FLOATING_POINT_MODE
inline void debugln_internal(const float &floatVal, const DebugType &debugType) {
  debug_internal(String(floatVal), debugType, true);
}
#endif
inline void debugln_internal(const FixedPoint &fpVal, const DebugType &debugType) {
#if FIXED_SIZE == 64
  debug_internal(format64(fpVal), debugType, true);
#else
  debug_internal(String(fpVal), debugType, true);
#endif
}
#if FIXED_SIZE != 16
inline void debugln_internal(const int &intVal, const DebugType &debugType) {
  debug_internal(String(intVal), debugType, true);
}
#endif
inline void debugln_internal(const DebugType debugType) {
  debug_internal("", debugType, true);
}

