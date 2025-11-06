#pragma once

#include <Arduino.h>

struct PlanetPosition {
  float raHours;
  float decDegrees;
  float distanceAu;
};

enum class PlanetId {
  Mercury,
  Venus,
  Earth,
  Mars,
  Jupiter,
  Saturn,
  Uranus,
  Neptune,
};

namespace planets {

bool computePlanet(PlanetId id, float julianDay, PlanetPosition& out);
bool planetFromString(const String& name, PlanetId& id);
float julianDay(int year, int month, int day, float hourFraction);

}  // namespace planets

