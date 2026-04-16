#pragma once

#include <Arduino.h>

struct SystemState {
  bool trackingActive;
  bool gotoActive;
  int64_t azGotoTarget;
  int64_t altGotoTarget;
};

extern SystemState systemState;
