#pragma once

//#define DEBUG_GENERAL_ENABLED
//#define DEBUG_CONSTRAINT_ENABLED
//#define DEBUG_COLLISION_ENABLED
//#define DEBUG_SIM_ENABLED
//#define DEBUG_FORCE_ENABLED

enum DebugType {DEBUG_GENERAL, DEBUG_COLLISION, DEBUG_CONSTRAINT, DEBUG_FIXEDPOINT, DEBUG_FORCE, DEBUG_PARTICLE, DEBUG_RENDERER, DEBUG_SIM, DEBUG_VECTOR};

#define DEBUG_ON (defined( DEBUG_GENERAL_ENABLED) || defined( DEBUG_COLLISION_ENABLED)  || defined( DEBUG_FIXEDPOINT_ENABLED)  || defined( DEBUG_FORCE_ENABLED)  || defined( DEBUG_RENDERER_ENABLED)  || defined( DEBUG_SIM_ENABLED) ||  defined( DEBUG_VECTOR_ENABLED) ||  defined( DEBUG_CONSTRAINT_ENABLED))

inline void debug(const String string, const DebugType debugType, const bool newLine = false) {
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
}

inline void debug(const float floatVal, const DebugType debugType) {
#if DEBUG_ON
  debug(String(floatVal), debugType);
#endif
}
inline void debug(const FixedPoint fpVal, const DebugType debugType) {
#if DEBUG_ON
  debug(String(fpVal), debugType);
#endif
}
inline void debug(const int intVal, const DebugType debugType) {
#if DEBUG_ON
  debug(String(intVal), debugType);
#endif
}

inline void debugln(const String string, const DebugType debugType) {
#if DEBUG_ON
  debug(string, debugType, true);
#endif
}
inline void debugln(const float floatVal, const DebugType debugType) {
#if DEBUG_ON
  debug(String(floatVal), debugType, true);
#endif
}
inline void debugln(const FixedPoint fpVal, const DebugType debugType) {
#if DEBUG_ON
  debug(String(fpVal), debugType, true);
#endif
}
inline void debugln(const int intVal, const DebugType debugType) {
#if DEBUG_ON
  debug(String(intVal), debugType, true);
#endif
}
inline void debugln(const DebugType debugType) {
#if DEBUG_ON
  debug("", debugType, true);
#endif
}

