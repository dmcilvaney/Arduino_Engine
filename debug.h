#pragma once

#define DEBUG_GENERAL_ENABLED

enum DebugType {DEBUG_GENERAL, DEBUG_COLLISION, DEBUG_FIXEDPOINT, DEBUG_FORCE, DEBUG_PARTICLE, DEBUG_RENDERER, DEBUG_SIM, DEBUG_VECTOR};

void inline print(const String string, const bool newLine) {
  if (newLine) {
    Serial.println(string);
  } else {
    Serial.print(string);
  }
}

inline void debug(const String string, const DebugType debugType, const bool newLine = false) {
  switch (debugType) {
#ifdef DEBUG_GENERAL_ENABLED
      case(DEBUG_GENERAL):
        print(string, newLine);
        break;
#endif
#ifdef DEBUG_COLLISION_ENABLED
      case(DEBUG_COLLISION):
        print(string, newLine);
        break;
#endif
#ifdef DEBUG_FIXEDPOINT_ENABLED
      case(DEBUG_FIXEDPOINT):
        print(string, newLine);
        break;
#endif
#ifdef DEBUG_FORCE_ENABLED
      case(DEBUG_FORCE):
        print(string, newLine);
        break;
#endif
#ifdef DEBUG_RENDERER_ENABLED
      case(DEBUG_RENDERER):
        print(string, newLine);
        break;
#endif
#ifdef DEBUG_SIM_ENABLED
      case(DEBUG_SIM):
        print(string, newLine);
        break;
#endif
#ifdef DEBUG_VECTOR_ENABLED
      case(DEBUG_VECTOR):
        print(string, newLine);
        break;
#endif
      default:
        break;
  }
}

inline void debug(const float floatVal, const DebugType debugType) {
  debug(String(floatVal), debugType);
}
inline void debug(const FixedPoint fpVal, const DebugType debugType) {
  debug(String(fpVal), debugType);
}
inline void debug(const int intVal, const DebugType debugType) {
  debug(String(intVal), debugType);
}

inline void debugln(const String string, const DebugType debugType) {
  debug(string, debugType, true);
}
inline void debugln(const float floatVal, const DebugType debugType) {
  debug(String(floatVal), debugType, true);
}
inline void debugln(const FixedPoint fpVal, const DebugType debugType) {
  debug(String(fpVal), debugType, true);
}
inline void debugln(const int intVal, const DebugType debugType) {
  debug(String(intVal), debugType, true);
}
inline void debugln(const DebugType debugType) {
  debug("", debugType, true);
}

