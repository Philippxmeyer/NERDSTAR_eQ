#include "motion.h"

#if defined(DEVICE_ROLE_MAIN)

#include <algorithm>
#include <cmath>
#include <limits>

#include <esp32-hal-gpio.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>

#include "config.h"
#include "storage.h"

namespace {

constexpr double kStepsPerAxisRev =
    config::FULLSTEPS_PER_REV * config::MICROSTEPS * config::GEAR_RATIO;
constexpr double kMinActiveStepsPerSecond = 0.1;
constexpr uint32_t kStepPulseWidthUs = 3;
constexpr double kMinDeclinationDegrees = -180.0;
constexpr double kMaxDeclinationDegrees = 180.0;

struct AxisState {
  uint8_t enPin;
  uint8_t dirPin;
  uint8_t stepPin;
  portMUX_TYPE mux;
  volatile int64_t stepCounter;
  double userStepsPerSecond;
  double gotoStepsPerSecond;
  double trackingStepsPerSecond;
  int8_t lastDirection;
  int8_t commandDirection;
  int32_t backlashStepsRemaining;
  bool backlashActive;
  int8_t backlashDirection;
  uint64_t nextStepDueUs;
};

struct ManualAxisControl {
  double currentStepsPerSecond;
  uint64_t lastUpdateUs;
};

AxisState axisAz{config::EN_RA,
                 config::DIR_RA,
                 config::STEP_RA,
                 portMUX_INITIALIZER_UNLOCKED,
                 0,
                 0.0,
                 0.0,
                 0.0,
                 1,
                 0,
                 0,
                 false,
                 0,
                 0};

AxisState axisAlt{config::EN_DEC,
                  config::DIR_DEC,
                  config::STEP_DEC,
                  portMUX_INITIALIZER_UNLOCKED,
                  0,
                  0.0,
                  0.0,
                  0.0,
                  1,
                  0,
                  0,
                  false,
                  0,
                  0};

ManualAxisControl manualAzControl{0.0, 0};
ManualAxisControl manualAltControl{0.0, 0};

volatile int8_t motorDirectionSignAz = 1;
volatile int8_t motorDirectionSignAlt = 1;

AxisCalibration calibration{
    (config::FULLSTEPS_PER_REV * config::MICROSTEPS * config::GEAR_RATIO) / 360.0,
    (config::FULLSTEPS_PER_REV * config::MICROSTEPS * config::GEAR_RATIO) / 360.0,
    0,
    0};

BacklashConfig backlash{0, 0};
int32_t backlashTakeupRateStepsPerSecond = config::DEFAULT_BACKLASH_TAKEUP_STEPS_PER_SEC;

portMUX_TYPE trackingMux = portMUX_INITIALIZER_UNLOCKED;
bool trackingEnabled = false;
bool altitudeLimitsEnabled = false;

AxisState& getAxisState(Axis axis) {
  return (axis == Axis::Ra) ? axisAz : axisAlt;
}

ManualAxisControl& getManualControl(Axis axis) {
  return (axis == Axis::Ra) ? manualAzControl : manualAltControl;
}

double getAxisStepsPerDegree(Axis axis) {
  return (axis == Axis::Ra) ? calibration.stepsPerDegreeAz
                            : calibration.stepsPerDegreeAlt;
}

int32_t getAxisBacklashSteps(const AxisState& axis) {
  return (&axis == &axisAz) ? backlash.azSteps : backlash.altSteps;
}

bool isTrackingEnabled() {
  portENTER_CRITICAL(&trackingMux);
  bool enabled = trackingEnabled;
  portEXIT_CRITICAL(&trackingMux);
  return enabled;
}

void updateNextStep(AxisState& axis, uint64_t next) {
  portENTER_CRITICAL(&axis.mux);
  axis.nextStepDueUs = next;
  portEXIT_CRITICAL(&axis.mux);
}

void setAxisCounter(AxisState& axis, int64_t value) {
  portENTER_CRITICAL(&axis.mux);
  axis.stepCounter = value;
  portEXIT_CRITICAL(&axis.mux);
}

int64_t getAxisCounter(const AxisState& axis) {
  portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&axis.mux));
  int64_t value = axis.stepCounter;
  portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&axis.mux));
  return value;
}

double getAxisUserContribution(const AxisState& axis) {
  portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&axis.mux));
  double value = axis.userStepsPerSecond;
  portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&axis.mux));
  return value;
}

double getAxisGotoContribution(const AxisState& axis) {
  portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&axis.mux));
  double value = axis.gotoStepsPerSecond;
  portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&axis.mux));
  return value;
}

double getAxisTrackingContribution(const AxisState& axis) {
  portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&axis.mux));
  double value = axis.trackingStepsPerSecond;
  portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&axis.mux));
  return value;
}

void setAxisUserContribution(AxisState& axis, double value) {
  portENTER_CRITICAL(&axis.mux);
  axis.userStepsPerSecond = value;
  portEXIT_CRITICAL(&axis.mux);
}

void setAxisGotoContribution(AxisState& axis, double value) {
  portENTER_CRITICAL(&axis.mux);
  axis.gotoStepsPerSecond = value;
  portEXIT_CRITICAL(&axis.mux);
}

void setAxisTrackingContribution(AxisState& axis, double value) {
  portENTER_CRITICAL(&axis.mux);
  axis.trackingStepsPerSecond = value;
  portEXIT_CRITICAL(&axis.mux);
}

double stepsToDeclinationDegreesRaw(int64_t steps) {
  if (calibration.stepsPerDegreeAlt <= 0.0) {
    return 0.0;
  }
  double adjusted = static_cast<double>(steps - calibration.altHomeOffset);
  return adjusted / calibration.stepsPerDegreeAlt;
}

void applyStep(AxisState& axis, int8_t direction, uint64_t nextDue) {
  bool allowStep = true;
  bool applyingBacklash = false;

  portENTER_CRITICAL(&axis.mux);

  if (axis.backlashActive && axis.backlashStepsRemaining > 0) {
    applyingBacklash = true;
  }

  int64_t candidate = axis.stepCounter + direction;
  if (allowStep) {
    if (!applyingBacklash) {
      axis.stepCounter = candidate;
      axis.lastDirection = direction;
    }
    axis.nextStepDueUs = nextDue;
  }

  if (allowStep && applyingBacklash) {
    axis.backlashStepsRemaining--;
    if (axis.backlashStepsRemaining == 0) {
      axis.backlashActive = false;
      axis.lastDirection = direction;
    }
  }

  portEXIT_CRITICAL(&axis.mux);

  if (!allowStep) {
    return;
  }

  int8_t hardwareDirection = direction;
  if (&axis == &axisAz) {
    hardwareDirection *= motorDirectionSignAz;
  } else {
    hardwareDirection *= motorDirectionSignAlt;
  }

  digitalWrite(axis.dirPin, hardwareDirection > 0 ? HIGH : LOW);
  digitalWrite(axis.stepPin, HIGH);
  esp_rom_delay_us(kStepPulseWidthUs);
  digitalWrite(axis.stepPin, LOW);
}

uint64_t updateAxis(AxisState& axis, uint64_t nowUs) {
  double total = 0.0;
  int8_t direction = 0;
  bool backlashMode = false;

  int32_t configuredBacklash = getAxisBacklashSteps(axis);
  double user = getAxisUserContribution(axis);
  double gotoRate = getAxisGotoContribution(axis);
  double trackingRate = isTrackingEnabled() ? getAxisTrackingContribution(axis)
                                            : 0.0;
  double commanded = user + gotoRate + trackingRate;

  if (std::fabs(commanded) >= kMinActiveStepsPerSecond) {
    direction = (commanded >= 0.0) ? 1 : -1;
    total = std::fabs(commanded);
  }

  portENTER_CRITICAL(&axis.mux);
  if (direction != 0 && axis.commandDirection != direction) {
    axis.commandDirection = direction;
    if (axis.lastDirection != 0 && axis.lastDirection != direction &&
        configuredBacklash > 0) {
      axis.backlashStepsRemaining = configuredBacklash;
      axis.backlashActive = true;
      axis.backlashDirection = direction;
    } else {
      axis.backlashStepsRemaining = 0;
      axis.backlashActive = false;
    }
  }

  if (axis.backlashActive && axis.backlashStepsRemaining > 0) {
    backlashMode = true;
    direction = axis.backlashDirection;
    total = static_cast<double>(backlashTakeupRateStepsPerSecond);
  }
  portEXIT_CRITICAL(&axis.mux);

  if (!backlashMode && std::fabs(total) < kMinActiveStepsPerSecond) {
    updateNextStep(axis, 0);
    digitalWrite(axis.stepPin, LOW);
    return std::numeric_limits<uint64_t>::max();
  }

  if (total < kMinActiveStepsPerSecond) {
    total = kMinActiveStepsPerSecond;
  }

  uint64_t nextDue = nowUs;
  {
    portENTER_CRITICAL(&axis.mux);
    if (axis.nextStepDueUs != 0) {
      nextDue = axis.nextStepDueUs;
    }
    portEXIT_CRITICAL(&axis.mux);
  }

  if (nowUs + 1 >= nextDue) {
    double intervalUs = 1000000.0 / (total * 1.0);
    if (intervalUs < 50.0) {
      intervalUs = 50.0;
    }
    uint64_t scheduled = nowUs + static_cast<uint64_t>(intervalUs);
    applyStep(axis, direction, scheduled);
    return scheduled;
  }

  return nextDue;
}

}  // namespace

namespace motion {

void init() {
  pinMode(axisAz.enPin, OUTPUT);
  pinMode(axisAz.dirPin, OUTPUT);
  pinMode(axisAz.stepPin, OUTPUT);
  pinMode(axisAlt.enPin, OUTPUT);
  pinMode(axisAlt.dirPin, OUTPUT);
  pinMode(axisAlt.stepPin, OUTPUT);

  digitalWrite(axisAz.enPin, HIGH);
  digitalWrite(axisAlt.enPin, HIGH);
  digitalWrite(axisAz.stepPin, LOW);
  digitalWrite(axisAlt.stepPin, LOW);

  digitalWrite(axisAz.enPin, LOW);
  digitalWrite(axisAlt.enPin, LOW);

  updateNextStep(axisAz, 0);
  updateNextStep(axisAlt, 0);
}

void motorTaskLoop() {
  while (true) {
    uint64_t now = esp_timer_get_time();
    uint64_t nextAz = updateAxis(axisAz, now);
    uint64_t nextAlt = updateAxis(axisAlt, now);
    uint64_t nextWake = std::min(nextAz, nextAlt);

    if (nextWake == std::numeric_limits<uint64_t>::max()) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    uint64_t nowAfter = esp_timer_get_time();
    if (nextWake <= nowAfter) {
      taskYIELD();
      continue;
    }

    uint64_t delta = nextWake - nowAfter;
    if (delta > 2000) {
      vTaskDelay(pdMS_TO_TICKS(delta / 1000));
    } else {
      esp_rom_delay_us(delta);
    }
  }
}

void setManualRate(Axis axis, float rpm) {
  double stepsPerSecond = (static_cast<double>(rpm) * kStepsPerAxisRev) / 60.0;
  setManualStepsPerSecond(axis, stepsPerSecond);
}

void setManualStepsPerSecond(Axis axis, double stepsPerSecond) {
  if (!std::isfinite(stepsPerSecond)) {
    stepsPerSecond = 0.0;
  }

  AxisState& axisState = getAxisState(axis);
  ManualAxisControl& control = getManualControl(axis);
  const auto& profile = storage::getConfig().panningProfile;
  double stepsPerDegree = getAxisStepsPerDegree(axis);

  double maxSpeed = profile.maxSpeedDegPerSec * stepsPerDegree;
  if (maxSpeed <= 0.0) {
    maxSpeed = std::numeric_limits<double>::infinity();
  }
  double acceleration = profile.accelerationDegPerSec2 * stepsPerDegree;
  if (acceleration <= 0.0) {
    acceleration = 1.0;
  }
  double deceleration = profile.decelerationDegPerSec2 * stepsPerDegree;
  if (deceleration <= 0.0) {
    deceleration = 1.0;
  }

  double target = stepsPerSecond;
  if (std::isfinite(maxSpeed)) {
    target = std::clamp(target, -maxSpeed, maxSpeed);
  }

  uint64_t nowUs = esp_timer_get_time();
  double dt = 0.0;
  if (control.lastUpdateUs != 0 && nowUs >= control.lastUpdateUs) {
    dt = static_cast<double>(nowUs - control.lastUpdateUs) / 1000000.0;
  }
  control.lastUpdateUs = nowUs;

  double current = control.currentStepsPerSecond;
  if (dt <= 0.0) {
    current = target;
  } else {
    double delta = target - current;
    if (delta != 0.0) {
      double limit = ((delta > 0.0) ? acceleration : deceleration) * dt;
      if (limit <= 0.0 || std::fabs(delta) <= limit) {
        current = target;
      } else {
        current += (delta > 0.0) ? limit : -limit;
      }
    }
  }

  if (std::fabs(current) < 1e-6 && target == 0.0) {
    current = 0.0;
  }

  control.currentStepsPerSecond = current;
  setAxisUserContribution(axisState, current);
}

void setGotoStepsPerSecond(Axis axis, double stepsPerSecond) {
  setAxisGotoContribution(getAxisState(axis), stepsPerSecond);
}

void clearGotoRates() {
  setAxisGotoContribution(axisAz, 0.0);
  setAxisGotoContribution(axisAlt, 0.0);
}

void stopAll() {
  manualAzControl.currentStepsPerSecond = 0.0;
  manualAzControl.lastUpdateUs = esp_timer_get_time();
  manualAltControl.currentStepsPerSecond = 0.0;
  manualAltControl.lastUpdateUs = manualAzControl.lastUpdateUs;

  setAxisUserContribution(axisAz, 0.0);
  setAxisUserContribution(axisAlt, 0.0);
  setAxisGotoContribution(axisAz, 0.0);
  setAxisGotoContribution(axisAlt, 0.0);
  setAxisTrackingContribution(axisAz, 0.0);
  setAxisTrackingContribution(axisAlt, 0.0);
  portENTER_CRITICAL(&axisAz.mux);
  axisAz.backlashStepsRemaining = 0;
  axisAz.backlashActive = false;
  portEXIT_CRITICAL(&axisAz.mux);
  portENTER_CRITICAL(&axisAlt.mux);
  axisAlt.backlashStepsRemaining = 0;
  axisAlt.backlashActive = false;
  portEXIT_CRITICAL(&axisAlt.mux);
  portENTER_CRITICAL(&trackingMux);
  trackingEnabled = false;
  portEXIT_CRITICAL(&trackingMux);
}

void setTrackingEnabled(bool enabled) {
  portENTER_CRITICAL(&trackingMux);
  trackingEnabled = enabled;
  portEXIT_CRITICAL(&trackingMux);
}

void setTrackingRates(double raDegPerSec, double decDegPerSec) {
  double raSteps = raDegPerSec * calibration.stepsPerDegreeAz;
  double decSteps = decDegPerSec * calibration.stepsPerDegreeAlt;
  setAxisTrackingContribution(axisAz, raSteps);
  setAxisTrackingContribution(axisAlt, decSteps);
}

bool isManualMotionActive() {
  return (std::fabs(getAxisUserContribution(axisAz)) > kMinActiveStepsPerSecond ||
          std::fabs(getAxisUserContribution(axisAlt)) > kMinActiveStepsPerSecond);
}

int64_t getStepCount(Axis axis) { return getAxisCounter(getAxisState(axis)); }

void setStepCount(Axis axis, int64_t value) {
  AxisState& axisState = getAxisState(axis);
  portENTER_CRITICAL(&axisState.mux);
  axisState.stepCounter = value;
  axisState.backlashStepsRemaining = 0;
  axisState.backlashActive = false;
  portEXIT_CRITICAL(&axisState.mux);
}

double stepsToRaDegrees(int64_t steps) {
  double adjusted = static_cast<double>(steps - calibration.azHomeOffset);
  double degrees = adjusted / calibration.stepsPerDegreeAz;
  degrees = fmod(degrees, 360.0);
  if (degrees < 0.0) {
    degrees += 360.0;
  }
  return degrees;
}

double stepsToDecDegrees(int64_t steps) {
  if (calibration.stepsPerDegreeAlt <= 0.0) {
    return 0.0;
  }
  double degrees = stepsToDeclinationDegreesRaw(steps);
  if (degrees > 180.0 || degrees < -180.0) {
    degrees = fmod(degrees, 360.0);
    if (degrees > 180.0) {
      degrees -= 360.0;
    } else if (degrees < -180.0) {
      degrees += 360.0;
    }
  }
  return degrees;
}

int64_t raDegreesToSteps(double degrees) {
  double wrapped = fmod(degrees, 360.0);
  if (wrapped < 0.0) {
    wrapped += 360.0;
  }
  double steps = wrapped * calibration.stepsPerDegreeAz + calibration.azHomeOffset;
  return static_cast<int64_t>(llround(steps));
}

int64_t decDegreesToSteps(double degrees) {
  if (calibration.stepsPerDegreeAlt <= 0.0) {
    return 0;
  }
  double steps = degrees * calibration.stepsPerDegreeAlt + calibration.altHomeOffset;
  return static_cast<int64_t>(llround(steps));
}

double getMinDeclinationDegrees() { return kMinDeclinationDegrees; }

double getMaxDeclinationDegrees() { return kMaxDeclinationDegrees; }

double stepsToAzDegrees(int64_t steps) { return stepsToRaDegrees(steps); }

double stepsToAltDegrees(int64_t steps) { return stepsToDecDegrees(steps); }

int64_t azDegreesToSteps(double degrees) { return raDegreesToSteps(degrees); }

int64_t altDegreesToSteps(double degrees) { return decDegreesToSteps(degrees); }

double getMinAltitudeDegrees() { return getMinDeclinationDegrees(); }

double getMaxAltitudeDegrees() { return getMaxDeclinationDegrees(); }

void applyCalibration(const AxisCalibration& newCalibration) {
  calibration = newCalibration;
}

void setBacklash(const BacklashConfig& newBacklash) { backlash = newBacklash; }

void setBacklashTakeupRateStepsPerSecond(int32_t stepsPerSecond) {
  backlashTakeupRateStepsPerSecond =
      (stepsPerSecond > 0) ? stepsPerSecond : config::DEFAULT_BACKLASH_TAKEUP_STEPS_PER_SEC;
}

void setAltitudeLimitsEnabled(bool enabled) { altitudeLimitsEnabled = enabled; }

int32_t getBacklashSteps(Axis axis) {
  return (axis == Axis::Ra) ? backlash.azSteps : backlash.altSteps;
}

int32_t getBacklashTakeupRateStepsPerSecond() {
  return backlashTakeupRateStepsPerSecond;
}

int8_t getLastDirection(Axis axis) {
  AxisState& state = getAxisState(axis);
  portENTER_CRITICAL(&state.mux);
  int8_t direction = state.lastDirection;
  portEXIT_CRITICAL(&state.mux);
  return direction;
}

bool setMotorInversion(bool invertAz, bool invertAlt) {
  motorDirectionSignAz = invertAz ? -1 : 1;
  motorDirectionSignAlt = invertAlt ? -1 : 1;
  return true;
}

void servicePendingOperations() {}

}  // namespace motion

#endif  // DEVICE_ROLE_MAIN
