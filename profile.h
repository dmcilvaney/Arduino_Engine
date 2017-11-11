#pragma once

enum ProfileType {PROFILE_SIM=2, PROFILE_RENDER=3, PROFILE_COLLISION=4, PROFILE_SQRT=5, PROFILE_OPERATOR=6};

inline void PROFILE_INIT() {
  pinMode(PROFILE_SIM, OUTPUT);
  digitalWrite(PROFILE_SIM, LOW);
  pinMode(PROFILE_RENDER, OUTPUT);
  digitalWrite(PROFILE_RENDER, LOW);
  pinMode(PROFILE_COLLISION, OUTPUT);
  digitalWrite(PROFILE_COLLISION, LOW);
  pinMode(PROFILE_SQRT, OUTPUT);
  digitalWrite(PROFILE_SQRT, LOW);
  pinMode(PROFILE_OPERATOR, OUTPUT);
  digitalWrite(PROFILE_OPERATOR, LOW);
}

inline void PROFILE_ON(ProfileType type) {
      digitalWrite(type,HIGH);
}
inline void PROFILE_OFF(ProfileType type) {
      digitalWrite(type,LOW);
}
