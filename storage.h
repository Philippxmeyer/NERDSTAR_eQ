#pragma once

#include <Arduino.h>

#include "calibration.h"

struct SystemConfig {
  uint32_t magic;
  AxisCalibration axisCalibration;
  BacklashConfig backlash;
  GotoProfile gotoProfile;
  GotoProfile panningProfile;
  uint8_t motorInvertAz;
  uint8_t motorInvertAlt;
  double orientationAzBiasDeg;
  double orientationAltBiasDeg;
  double orientationSampleWeight;
  uint16_t configVersion;
  int32_t backlashTakeupRateStepsPerSecond;
};

namespace storage {

bool init();
const SystemConfig& getConfig();
void setAxisCalibration(const AxisCalibration& calibration);
void setBacklash(const BacklashConfig& backlash);
void setBacklashTakeupRateStepsPerSecond(int32_t stepsPerSecond);
void setGotoProfile(const GotoProfile& profile);
void setPanningProfile(const GotoProfile& profile);
void setMotorInversion(bool invertAz, bool invertAlt);
void setOrientationModel(double azBiasDeg, double altBiasDeg, double sampleWeight);
void clearOrientationModel();
void save();

}  // namespace storage
