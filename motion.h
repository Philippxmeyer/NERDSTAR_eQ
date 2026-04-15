#pragma once

#include <Arduino.h>

#include "calibration.h"
#include "config.h"

enum class Axis {
  Az,
  Alt
};

namespace motion {

void init();
void setManualRate(Axis axis, float rpm);
void setManualStepsPerSecond(Axis axis, double stepsPerSecond);
void setGotoStepsPerSecond(Axis axis, double stepsPerSecond);
void clearGotoRates();
void stopAll();
void setTrackingEnabled(bool enabled);
void setTrackingRates(double azDegPerSec, double altDegPerSec);
bool isManualMotionActive();
int64_t getStepCount(Axis axis);
void setStepCount(Axis axis, int64_t value);
double stepsToAzDegrees(int64_t steps);
double stepsToAltDegrees(int64_t steps);
int64_t azDegreesToSteps(double degrees);
int64_t altDegreesToSteps(double degrees);
double getMinAltitudeDegrees();
double getMaxAltitudeDegrees();
void applyCalibration(const AxisCalibration& calibration);
void setBacklash(const BacklashConfig& backlash);
void setBacklashTakeupRateStepsPerSecond(int32_t stepsPerSecond);
int32_t getBacklashSteps(Axis axis);
int32_t getBacklashTakeupRateStepsPerSecond();
int8_t getLastDirection(Axis axis);
void setAltitudeLimitsEnabled(bool enabled);
bool setMotorInversion(bool invertAz, bool invertAlt);
void servicePendingOperations();

#if defined(DEVICE_ROLE_MAIN)
void motorTaskLoop();
#endif

} // namespace motion
