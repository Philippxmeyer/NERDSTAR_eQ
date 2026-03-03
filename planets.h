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
bool computeMoon(float julianDay, PlanetPosition& out);
bool planetFromString(const String& name, PlanetId& id);
bool moonFromString(const String& name);
double julianDay(int year, int month, int day, double hourFraction);

}  // namespace planets
