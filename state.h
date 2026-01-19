#pragma once

#include <Arduino.h>

enum class MenuMode {
  Status,
  PolarAlign,
  Setup,
  Catalog,
  Goto
};

struct SystemState {
  MenuMode menuMode;
  bool polarAligned;
  bool trackingActive;
  bool gotoActive;
  int selectedCatalogIndex;
  int selectedCatalogTypeIndex;
  int64_t azGotoTarget;
  int64_t altGotoTarget;
  bool joystickActive;
  float joystickX;
  float joystickY;
  bool joystickButtonPressed;
  bool mountLinkReady;
  bool manualCommandOk;
  bool telescopeFlipped;
  bool flipInProgress;
};

extern SystemState systemState;
