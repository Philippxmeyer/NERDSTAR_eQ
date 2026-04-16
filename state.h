#pragma once

#include <Arduino.h>

struct SystemState {
  bool trackingActive;
  bool gotoActive;
  int64_t raGotoTarget;
  int64_t decGotoTarget;
};

extern SystemState systemState;
