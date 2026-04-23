#include "storage.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>

#include "config.h"

namespace {

constexpr uint32_t kConfigMagic = 0x4E455244;  // "NERD"
constexpr uint16_t kConfigVersion = 2;
constexpr size_t kConfigStorageSize = 256;
bool eepromReady = false;

constexpr SiteLocation kDefaultSite{0.0, 0.0, 0, 0};

SystemConfig systemConfig{kConfigMagic,
                          {config::DEFAULT_AXIS_STEPS_PER_DEG,
                           config::DEFAULT_AXIS_STEPS_PER_DEG,
                           config::DEFAULT_AZ_HOME_OFFSET,
                           config::DEFAULT_ALT_HOME_OFFSET},
                          {config::DEFAULT_BACKLASH_AZ_STEPS,
                           config::DEFAULT_BACKLASH_ALT_STEPS},
                          {config::DEFAULT_GOTO_MAX_SPEED_DEG_PER_SEC,
                           config::DEFAULT_GOTO_ACCEL_DEG_PER_SEC2,
                           config::DEFAULT_GOTO_DECEL_DEG_PER_SEC2},
                          {config::DEFAULT_PAN_MAX_SPEED_DEG_PER_SEC,
                           config::DEFAULT_PAN_ACCEL_DEG_PER_SEC2,
                           config::DEFAULT_PAN_DECEL_DEG_PER_SEC2},
                          config::DEFAULT_MOTOR_INVERT_AZ,
                          config::DEFAULT_MOTOR_INVERT_ALT,
                          config::DEFAULT_ORIENTATION_AZ_BIAS_DEG,
                          config::DEFAULT_ORIENTATION_ALT_BIAS_DEG,
                          config::DEFAULT_ORIENTATION_SAMPLE_WEIGHT,
                          kConfigVersion,
                          config::DEFAULT_BACKLASH_TAKEUP_STEPS_PER_SEC,
                          kDefaultSite};

static_assert(sizeof(SystemConfig) <= kConfigStorageSize,
              "SystemConfig too large for config storage");

void applyDefaults() {
  constexpr double stepsPerMotorRev = config::FULLSTEPS_PER_REV * config::MICROSTEPS;
  constexpr double stepsPerAxisRev = stepsPerMotorRev * config::GEAR_RATIO;

  systemConfig.magic = kConfigMagic;
  systemConfig.axisCalibration.stepsPerDegreeAz = stepsPerAxisRev / 360.0;
  systemConfig.axisCalibration.stepsPerDegreeAlt = stepsPerAxisRev / 360.0;
  systemConfig.axisCalibration.azHomeOffset = config::DEFAULT_AZ_HOME_OFFSET;
  systemConfig.axisCalibration.altHomeOffset = config::DEFAULT_ALT_HOME_OFFSET;
  systemConfig.backlash = {config::DEFAULT_BACKLASH_AZ_STEPS,
                           config::DEFAULT_BACKLASH_ALT_STEPS};
  systemConfig.gotoProfile = {config::DEFAULT_GOTO_MAX_SPEED_DEG_PER_SEC,
                              config::DEFAULT_GOTO_ACCEL_DEG_PER_SEC2,
                              config::DEFAULT_GOTO_DECEL_DEG_PER_SEC2};
  systemConfig.panningProfile = {config::DEFAULT_PAN_MAX_SPEED_DEG_PER_SEC,
                                 config::DEFAULT_PAN_ACCEL_DEG_PER_SEC2,
                                 config::DEFAULT_PAN_DECEL_DEG_PER_SEC2};
  systemConfig.motorInvertAz = config::DEFAULT_MOTOR_INVERT_AZ;
  systemConfig.motorInvertAlt = config::DEFAULT_MOTOR_INVERT_ALT;
  systemConfig.orientationAzBiasDeg = config::DEFAULT_ORIENTATION_AZ_BIAS_DEG;
  systemConfig.orientationAltBiasDeg = config::DEFAULT_ORIENTATION_ALT_BIAS_DEG;
  systemConfig.orientationSampleWeight = config::DEFAULT_ORIENTATION_SAMPLE_WEIGHT;
  systemConfig.configVersion = kConfigVersion;
  systemConfig.backlashTakeupRateStepsPerSecond =
      config::DEFAULT_BACKLASH_TAKEUP_STEPS_PER_SEC;
  systemConfig.site = kDefaultSite;
}

bool siteIsInvalid(const SiteLocation& site) {
  if (!isfinite(site.latitudeDeg) || !isfinite(site.longitudeDeg)) return true;
  if (site.latitudeDeg < -90.0 || site.latitudeDeg > 90.0) return true;
  if (site.longitudeDeg < -180.0 || site.longitudeDeg > 180.0) return true;
  if (site.utcOffsetMinutes < -14 * 60 || site.utcOffsetMinutes > 14 * 60) return true;
  if (site.valid > 1) return true;
  return false;
}

bool profileIsInvalid(const GotoProfile& profile) {
  return !isfinite(profile.maxSpeedDegPerSec) || profile.maxSpeedDegPerSec <= 0.0f ||
         !isfinite(profile.accelerationDegPerSec2) || profile.accelerationDegPerSec2 <= 0.0f ||
         !isfinite(profile.decelerationDegPerSec2) || profile.decelerationDegPerSec2 <= 0.0f;
}

void saveConfigInternal() {
  if (!eepromReady) {
    return;
  }
  EEPROM.put(0, systemConfig);
  EEPROM.commit();
}

}  // namespace

namespace storage {

bool init() {
  eepromReady = EEPROM.begin(kConfigStorageSize);
  if (!eepromReady) {
    applyDefaults();
    return false;
  }

  EEPROM.get(0, systemConfig);
  bool needsSave = false;
  if (systemConfig.magic != kConfigMagic || systemConfig.axisCalibration.stepsPerDegreeAz <= 0.0 ||
      systemConfig.axisCalibration.stepsPerDegreeAlt <= 0.0) {
    applyDefaults();
    saveConfigInternal();
    return true;
  }

  if (profileIsInvalid(systemConfig.gotoProfile)) {
    systemConfig.gotoProfile = {config::DEFAULT_GOTO_MAX_SPEED_DEG_PER_SEC,
                                config::DEFAULT_GOTO_ACCEL_DEG_PER_SEC2,
                                config::DEFAULT_GOTO_DECEL_DEG_PER_SEC2};
    needsSave = true;
  }
  if (profileIsInvalid(systemConfig.panningProfile)) {
    systemConfig.panningProfile = {config::DEFAULT_PAN_MAX_SPEED_DEG_PER_SEC,
                                   config::DEFAULT_PAN_ACCEL_DEG_PER_SEC2,
                                   config::DEFAULT_PAN_DECEL_DEG_PER_SEC2};
    needsSave = true;
  }
  if (systemConfig.backlash.azSteps < 0) systemConfig.backlash.azSteps = 0;
  if (systemConfig.backlash.altSteps < 0) systemConfig.backlash.altSteps = 0;
  if (systemConfig.backlashTakeupRateStepsPerSecond <= 0) {
    systemConfig.backlashTakeupRateStepsPerSecond =
        config::DEFAULT_BACKLASH_TAKEUP_STEPS_PER_SEC;
    needsSave = true;
  }
  if (systemConfig.motorInvertAz > 1) {
    systemConfig.motorInvertAz = config::DEFAULT_MOTOR_INVERT_AZ;
    needsSave = true;
  }
  if (systemConfig.motorInvertAlt > 1) {
    systemConfig.motorInvertAlt = config::DEFAULT_MOTOR_INVERT_ALT;
    needsSave = true;
  }
  if (!isfinite(systemConfig.orientationAzBiasDeg)) {
    systemConfig.orientationAzBiasDeg = config::DEFAULT_ORIENTATION_AZ_BIAS_DEG;
    needsSave = true;
  }
  if (!isfinite(systemConfig.orientationAltBiasDeg)) {
    systemConfig.orientationAltBiasDeg = config::DEFAULT_ORIENTATION_ALT_BIAS_DEG;
    needsSave = true;
  }
  if (!isfinite(systemConfig.orientationSampleWeight) || systemConfig.orientationSampleWeight < 0.0) {
    systemConfig.orientationSampleWeight = config::DEFAULT_ORIENTATION_SAMPLE_WEIGHT;
    needsSave = true;
  }
  if (systemConfig.configVersion < 2) {
    // Site location was added in v2; any bytes read from EEPROM for older
    // configs are undefined, so reset to the neutral default.
    systemConfig.site = kDefaultSite;
    needsSave = true;
  }
  if (siteIsInvalid(systemConfig.site)) {
    systemConfig.site = kDefaultSite;
    needsSave = true;
  }
  if (systemConfig.configVersion != kConfigVersion) {
    systemConfig.configVersion = kConfigVersion;
    needsSave = true;
  }
  if (needsSave) {
    saveConfigInternal();
  }
  return true;
}

const SystemConfig& getConfig() { return systemConfig; }

void setAxisCalibration(const AxisCalibration& calibration) {
  systemConfig.axisCalibration = calibration;
  saveConfigInternal();
}

void setBacklash(const BacklashConfig& backlash) {
  systemConfig.backlash = backlash;
  saveConfigInternal();
}

void setBacklashTakeupRateStepsPerSecond(int32_t stepsPerSecond) {
  if (stepsPerSecond <= 0) {
    stepsPerSecond = config::DEFAULT_BACKLASH_TAKEUP_STEPS_PER_SEC;
  }
  systemConfig.backlashTakeupRateStepsPerSecond = stepsPerSecond;
  saveConfigInternal();
}

void setGotoProfile(const GotoProfile& profile) {
  systemConfig.gotoProfile = profile;
  saveConfigInternal();
}

void setPanningProfile(const GotoProfile& profile) {
  systemConfig.panningProfile = profile;
  saveConfigInternal();
}

void setMotorInversion(bool invertAz, bool invertAlt) {
  systemConfig.motorInvertAz = invertAz ? 1 : 0;
  systemConfig.motorInvertAlt = invertAlt ? 1 : 0;
  saveConfigInternal();
}

void setOrientationModel(double azBiasDeg, double altBiasDeg, double sampleWeight) {
  systemConfig.orientationAzBiasDeg = azBiasDeg;
  systemConfig.orientationAltBiasDeg = altBiasDeg;
  systemConfig.orientationSampleWeight = sampleWeight;
  saveConfigInternal();
}

void clearOrientationModel() {
  setOrientationModel(0.0, 0.0, 0.0);
}

void setSiteLatitude(double latitudeDeg) {
  if (!isfinite(latitudeDeg)) return;
  if (latitudeDeg < -90.0) latitudeDeg = -90.0;
  if (latitudeDeg > 90.0) latitudeDeg = 90.0;
  systemConfig.site.latitudeDeg = latitudeDeg;
  systemConfig.site.valid = 1;
  saveConfigInternal();
}

void setSiteLongitude(double longitudeDeg) {
  if (!isfinite(longitudeDeg)) return;
  if (longitudeDeg < -180.0) longitudeDeg = -180.0;
  if (longitudeDeg > 180.0) longitudeDeg = 180.0;
  systemConfig.site.longitudeDeg = longitudeDeg;
  systemConfig.site.valid = 1;
  saveConfigInternal();
}

void setUtcOffsetMinutes(int32_t utcOffsetMinutes) {
  if (utcOffsetMinutes < -14 * 60) utcOffsetMinutes = -14 * 60;
  if (utcOffsetMinutes > 14 * 60) utcOffsetMinutes = 14 * 60;
  systemConfig.site.utcOffsetMinutes = utcOffsetMinutes;
  systemConfig.site.valid = 1;
  saveConfigInternal();
}

void save() { saveConfigInternal(); }

}  // namespace storage
