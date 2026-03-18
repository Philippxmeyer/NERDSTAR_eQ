#include "storage.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include <string.h>

#include "config.h"

namespace {

constexpr uint32_t kConfigMagic = 0x4E455244;  // "NERD"
constexpr uint16_t kConfigVersion = 4;
constexpr size_t kConfigStorageSize = 256;
bool eepromReady = false;

SystemConfig systemConfig{kConfigMagic,
                          {config::DEFAULT_JOYSTICK_CENTER,
                           config::DEFAULT_JOYSTICK_CENTER},
                          {config::DEFAULT_AXIS_STEPS_PER_DEG,
                           config::DEFAULT_AXIS_STEPS_PER_DEG,
                           config::DEFAULT_AZ_HOME_OFFSET,
                           config::DEFAULT_ALT_HOME_OFFSET},
                          {config::DEFAULT_BACKLASH_AZ_STEPS,
                           config::DEFAULT_BACKLASH_ALT_STEPS},
                          {config::DEFAULT_GOTO_MAX_SPEED_DEG_PER_SEC,
                           config::DEFAULT_GOTO_ACCEL_DEG_PER_SEC2,
                           config::DEFAULT_GOTO_DECEL_DEG_PER_SEC2},
                          config::OBSERVER_LATITUDE_DEG,
                          config::OBSERVER_LONGITUDE_DEG,
                          config::DEFAULT_TIMEZONE_OFFSET_MINUTES,
                          DstMode::Auto,
                          config::DEFAULT_JOYSTICK_CALIBRATED,
                          config::DEFAULT_AXIS_CALIBRATED,
                          config::DEFAULT_POLAR_ALIGNED,
                          config::DEFAULT_LAST_RTC_EPOCH,
                          {config::DEFAULT_PAN_MAX_SPEED_DEG_PER_SEC,
                           config::DEFAULT_PAN_ACCEL_DEG_PER_SEC2,
                           config::DEFAULT_PAN_DECEL_DEG_PER_SEC2},
                          config::DEFAULT_JOYSTICK_SWAP_AXES,
                          config::DEFAULT_JOYSTICK_INVERT_AZ,
                          config::DEFAULT_JOYSTICK_INVERT_ALT,
                          config::DEFAULT_MOTOR_INVERT_AZ,
                          config::DEFAULT_MOTOR_INVERT_ALT,
                          config::DEFAULT_ORIENTATION_AZ_BIAS_DEG,
                          config::DEFAULT_ORIENTATION_ALT_BIAS_DEG,
                          config::DEFAULT_ORIENTATION_SAMPLE_WEIGHT,
                          kConfigVersion,
                          config::DEFAULT_DISPLAY_CONTRAST,
                          config::DEFAULT_BACKLASH_TAKEUP_STEPS_PER_SEC};

static_assert(sizeof(SystemConfig) <= kConfigStorageSize, "SystemConfig too large for config storage");

static_assert(sizeof(storage::CatalogEntry) == 12, "CatalogEntry packing mismatch");

// `catalog_data.inc` is a generated binary blob that contains the default catalog data.
// It is produced from `data/catalog.xml` via `tools/build_catalog.py` and embedded into
// the firmware image so we can seed the emulated EEPROM on first boot.
#include "catalog_data.inc"

void applyDefaults() {
  constexpr double stepsPerMotorRev = config::FULLSTEPS_PER_REV * config::MICROSTEPS;
  constexpr double stepsPerAxisRev = stepsPerMotorRev * config::GEAR_RATIO;
  systemConfig.magic = kConfigMagic;
  systemConfig.joystickCalibration = {config::DEFAULT_JOYSTICK_CENTER,
                                      config::DEFAULT_JOYSTICK_CENTER};
  systemConfig.axisCalibration.stepsPerDegreeAz = stepsPerAxisRev / 360.0;
  systemConfig.axisCalibration.stepsPerDegreeAlt = stepsPerAxisRev / 360.0;
  systemConfig.axisCalibration.azHomeOffset = config::DEFAULT_AZ_HOME_OFFSET;
  systemConfig.axisCalibration.altHomeOffset = config::DEFAULT_ALT_HOME_OFFSET;
  systemConfig.backlash = {config::DEFAULT_BACKLASH_AZ_STEPS,
                           config::DEFAULT_BACKLASH_ALT_STEPS};
  systemConfig.gotoProfile.maxSpeedDegPerSec = config::DEFAULT_GOTO_MAX_SPEED_DEG_PER_SEC;
  systemConfig.gotoProfile.accelerationDegPerSec2 = config::DEFAULT_GOTO_ACCEL_DEG_PER_SEC2;
  systemConfig.gotoProfile.decelerationDegPerSec2 = config::DEFAULT_GOTO_DECEL_DEG_PER_SEC2;
  systemConfig.observerLatitudeDeg = config::OBSERVER_LATITUDE_DEG;
  systemConfig.observerLongitudeDeg = config::OBSERVER_LONGITUDE_DEG;
  systemConfig.timezoneOffsetMinutes = config::DEFAULT_TIMEZONE_OFFSET_MINUTES;
  systemConfig.dstMode = DstMode::Auto;
  systemConfig.joystickCalibrated = config::DEFAULT_JOYSTICK_CALIBRATED;
  systemConfig.axisCalibrated = config::DEFAULT_AXIS_CALIBRATED;
  systemConfig.polarAligned = config::DEFAULT_POLAR_ALIGNED;
  systemConfig.lastRtcEpoch = config::DEFAULT_LAST_RTC_EPOCH;
  systemConfig.panningProfile.maxSpeedDegPerSec = config::DEFAULT_PAN_MAX_SPEED_DEG_PER_SEC;
  systemConfig.panningProfile.accelerationDegPerSec2 = config::DEFAULT_PAN_ACCEL_DEG_PER_SEC2;
  systemConfig.panningProfile.decelerationDegPerSec2 = config::DEFAULT_PAN_DECEL_DEG_PER_SEC2;
  systemConfig.joystickSwapAxes = config::DEFAULT_JOYSTICK_SWAP_AXES;
  systemConfig.joystickInvertAz = config::DEFAULT_JOYSTICK_INVERT_AZ;
  systemConfig.joystickInvertAlt = config::DEFAULT_JOYSTICK_INVERT_ALT;
  systemConfig.motorInvertAz = config::DEFAULT_MOTOR_INVERT_AZ;
  systemConfig.motorInvertAlt = config::DEFAULT_MOTOR_INVERT_ALT;
  systemConfig.orientationAzBiasDeg = config::DEFAULT_ORIENTATION_AZ_BIAS_DEG;
  systemConfig.orientationAltBiasDeg = config::DEFAULT_ORIENTATION_ALT_BIAS_DEG;
  systemConfig.orientationSampleWeight = config::DEFAULT_ORIENTATION_SAMPLE_WEIGHT;
  systemConfig.configVersion = kConfigVersion;
  systemConfig.displayContrast = config::DEFAULT_DISPLAY_CONTRAST;
  systemConfig.backlashTakeupRateStepsPerSecond =
      config::DEFAULT_BACKLASH_TAKEUP_STEPS_PER_SEC;
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
  } else {
    if (profileIsInvalid(systemConfig.gotoProfile)) {
      systemConfig.gotoProfile.maxSpeedDegPerSec = config::DEFAULT_GOTO_MAX_SPEED_DEG_PER_SEC;
      systemConfig.gotoProfile.accelerationDegPerSec2 = config::DEFAULT_GOTO_ACCEL_DEG_PER_SEC2;
      systemConfig.gotoProfile.decelerationDegPerSec2 = config::DEFAULT_GOTO_DECEL_DEG_PER_SEC2;
      needsSave = true;
    }
    if (profileIsInvalid(systemConfig.panningProfile)) {
      systemConfig.panningProfile.maxSpeedDegPerSec = config::DEFAULT_PAN_MAX_SPEED_DEG_PER_SEC;
      systemConfig.panningProfile.accelerationDegPerSec2 = config::DEFAULT_PAN_ACCEL_DEG_PER_SEC2;
      systemConfig.panningProfile.decelerationDegPerSec2 = config::DEFAULT_PAN_DECEL_DEG_PER_SEC2;
      needsSave = true;
    }
    if (systemConfig.backlash.azSteps < 0) systemConfig.backlash.azSteps = 0;
    if (systemConfig.backlash.altSteps < 0) systemConfig.backlash.altSteps = 0;
    if (systemConfig.backlashTakeupRateStepsPerSecond <= 0) {
      systemConfig.backlashTakeupRateStepsPerSecond =
          config::DEFAULT_BACKLASH_TAKEUP_STEPS_PER_SEC;
      needsSave = true;
    }
    if (!isfinite(systemConfig.observerLatitudeDeg) || systemConfig.observerLatitudeDeg < -90.0 ||
        systemConfig.observerLatitudeDeg > 90.0) {
      systemConfig.observerLatitudeDeg = config::OBSERVER_LATITUDE_DEG;
      needsSave = true;
    }
    if (!isfinite(systemConfig.observerLongitudeDeg) || systemConfig.observerLongitudeDeg < -180.0 ||
        systemConfig.observerLongitudeDeg > 180.0) {
      systemConfig.observerLongitudeDeg = config::OBSERVER_LONGITUDE_DEG;
      needsSave = true;
    }
    if (systemConfig.timezoneOffsetMinutes < -720 || systemConfig.timezoneOffsetMinutes > 840) {
      systemConfig.timezoneOffsetMinutes = config::DEFAULT_TIMEZONE_OFFSET_MINUTES;
      needsSave = true;
    }
    if (static_cast<uint8_t>(systemConfig.dstMode) > static_cast<uint8_t>(DstMode::Auto)) {
      systemConfig.dstMode = DstMode::Auto;
      needsSave = true;
    }
    auto sanitizeFlag = [&](uint8_t& flag) {
      if (flag > 1) {
        flag = 0;
        needsSave = true;
      }
    };
    sanitizeFlag(systemConfig.joystickSwapAxes);
    sanitizeFlag(systemConfig.joystickInvertAz);
    sanitizeFlag(systemConfig.joystickInvertAlt);
    sanitizeFlag(systemConfig.motorInvertAz);
    sanitizeFlag(systemConfig.motorInvertAlt);
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
      systemConfig.joystickSwapAxes = config::DEFAULT_JOYSTICK_SWAP_AXES;
      systemConfig.joystickInvertAz = config::DEFAULT_JOYSTICK_INVERT_AZ;
      systemConfig.joystickInvertAlt = config::DEFAULT_JOYSTICK_INVERT_ALT;
      systemConfig.motorInvertAz = config::DEFAULT_MOTOR_INVERT_AZ;
      systemConfig.motorInvertAlt = config::DEFAULT_MOTOR_INVERT_ALT;
      systemConfig.orientationAzBiasDeg = config::DEFAULT_ORIENTATION_AZ_BIAS_DEG;
      systemConfig.orientationAltBiasDeg = config::DEFAULT_ORIENTATION_ALT_BIAS_DEG;
      systemConfig.orientationSampleWeight = config::DEFAULT_ORIENTATION_SAMPLE_WEIGHT;
      needsSave = true;
    }
    if (systemConfig.configVersion < 3) {
      systemConfig.displayContrast = config::DEFAULT_DISPLAY_CONTRAST;
      needsSave = true;
    }
    if (systemConfig.configVersion < 4) {
      systemConfig.backlashTakeupRateStepsPerSecond =
          config::DEFAULT_BACKLASH_TAKEUP_STEPS_PER_SEC;
      needsSave = true;
    }
    if (systemConfig.configVersion != kConfigVersion) {
      systemConfig.configVersion = kConfigVersion;
      needsSave = true;
    }
    if (needsSave) {
      saveConfigInternal();
    }
  }
  return true;
}

const SystemConfig& getConfig() { return systemConfig; }

void setJoystickCalibration(const JoystickCalibration& calibration) {
  systemConfig.joystickCalibration = calibration;
  systemConfig.joystickCalibrated = true;
  saveConfigInternal();
}

void setAxisCalibration(const AxisCalibration& calibration) {
  systemConfig.axisCalibration = calibration;
  systemConfig.axisCalibrated = true;
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

void setPolarAligned(bool aligned) {
  systemConfig.polarAligned = aligned;
  saveConfigInternal();
}

void setRtcEpoch(uint32_t epoch) {
  systemConfig.lastRtcEpoch = epoch;
  saveConfigInternal();
}

void setObserverLocation(double latitudeDeg, double longitudeDeg, int32_t timezoneMinutes) {
  systemConfig.observerLatitudeDeg = latitudeDeg;
  systemConfig.observerLongitudeDeg = longitudeDeg;
  systemConfig.timezoneOffsetMinutes = timezoneMinutes;
  saveConfigInternal();
}

void setDstMode(DstMode mode) {
  if (mode == systemConfig.dstMode) {
    return;
  }
  systemConfig.dstMode = mode;
  saveConfigInternal();
}

void setJoystickOrientation(bool swapAxes, bool invertAz, bool invertAlt) {
  uint8_t swapValue = swapAxes ? 1 : 0;
  uint8_t invertAzValue = invertAz ? 1 : 0;
  uint8_t invertAltValue = invertAlt ? 1 : 0;
  if (systemConfig.joystickSwapAxes == swapValue &&
      systemConfig.joystickInvertAz == invertAzValue &&
      systemConfig.joystickInvertAlt == invertAltValue) {
    return;
  }
  systemConfig.joystickSwapAxes = swapValue;
  systemConfig.joystickInvertAz = invertAzValue;
  systemConfig.joystickInvertAlt = invertAltValue;
  systemConfig.configVersion = kConfigVersion;
  saveConfigInternal();
}

void setMotorInversion(bool invertAz, bool invertAlt) {
  uint8_t invertAzValue = invertAz ? 1 : 0;
  uint8_t invertAltValue = invertAlt ? 1 : 0;
  if (systemConfig.motorInvertAz == invertAzValue &&
      systemConfig.motorInvertAlt == invertAltValue) {
    return;
  }
  systemConfig.motorInvertAz = invertAzValue;
  systemConfig.motorInvertAlt = invertAltValue;
  systemConfig.configVersion = kConfigVersion;
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

void setDisplayContrast(uint8_t contrast) {
  if (systemConfig.displayContrast == contrast) {
    return;
  }
  systemConfig.displayContrast = contrast;
  saveConfigInternal();
}

void save() { saveConfigInternal(); }

size_t getCatalogEntryCount() { return kCatalogEntryCount; }

bool readCatalogEntry(size_t index, CatalogEntry& entry) {
  if (index >= kCatalogEntryCount) {
    return false;
  }
  entry = kCatalogEntries[index];
  return true;
}

bool readCatalogString(uint16_t offset, uint8_t length, char* buffer, size_t bufferSize) {
  if (!buffer || bufferSize == 0) {
    return false;
  }
  if (static_cast<size_t>(offset) + length > kCatalogStringTableSize) {
    return false;
  }
  if (bufferSize <= length) {
    return false;
  }
  memcpy(buffer, &kCatalogStrings[offset], length);
  buffer[length] = '\0';
  return true;
}

}  // namespace storage
