#pragma once

#include <Arduino.h>

struct AxisCalibration {
  double stepsPerDegreeAz;
  double stepsPerDegreeAlt;
  int64_t azHomeOffset;
  int64_t altHomeOffset;
};

struct BacklashConfig {
  int32_t azSteps;
  int32_t altSteps;
};

struct GotoProfile {
  float maxSpeedDegPerSec;
  float accelerationDegPerSec2;
  float decelerationDegPerSec2;
};

// Observer location pushed in by the host (Stellarmate / INDI) via the
// LX200 :Sg / :St / :SG commands.
//
// Longitude is stored east-positive (ISO convention) even though the Meade
// :Sg command transmits it west-positive.  The lx200_link module performs
// the conversion at parse time.
struct SiteLocation {
  double latitudeDeg;        // north-positive, -90..+90
  double longitudeDeg;       // east-positive,  -180..+180
  int32_t utcOffsetMinutes;  // local - UTC, in minutes
  uint8_t valid;             // 0 = never set by the host, 1 = populated
};

