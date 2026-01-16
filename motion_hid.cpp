#include "motion.h"

#if defined(DEVICE_ROLE_HID)

#include <cmath>
#include <cstdlib>
#include <initializer_list>
#include <limits>
#include <vector>

#include "comm.h"
#include "state.h"

namespace {

const char* axisToString(Axis axis) {
  return (axis == Axis::Az) ? "AZ" : "ALT";
}

String formatFloat(double value) { return String(value, 6); }

int64_t parseInt64(const String& value) {
  return static_cast<int64_t>(strtoll(value.c_str(), nullptr, 10));
}

double parseDouble(const String& value) {
  return strtod(value.c_str(), nullptr);
}

size_t axisIndex(Axis axis) { return (axis == Axis::Az) ? 0 : 1; }

bool callAndUpdate(const char* command, std::initializer_list<String> params,
                   std::vector<String>* payload = nullptr) {
  String error;
  bool success = comm::call(command, params, payload, &error);
  systemState.manualCommandOk = success;
  if (!success) {
    if (Serial) {
      Serial.printf("[MOTION] %s failed: %s\n", command,
                    error.isEmpty() ? "<unknown>" : error.c_str());
    }
  }
  return success;
}

float lastManualRpm[2] = {std::numeric_limits<float>::quiet_NaN(),
                          std::numeric_limits<float>::quiet_NaN()};
uint32_t lastManualSendMs[2] = {0, 0};

constexpr float kManualRpmDelta = 0.02f;
constexpr uint32_t kManualRefreshIntervalMs = 250;

bool desiredMotorInvertAz = false;
bool desiredMotorInvertAlt = false;
bool pendingMotorOrientation = false;
uint32_t lastMotorOrientationAttemptMs = 0;
constexpr uint32_t kMotorOrientationRetryIntervalMs = 500;

void invalidateManualCache() {
  for (size_t i = 0; i < 2; ++i) {
    lastManualRpm[i] = std::numeric_limits<float>::quiet_NaN();
    lastManualSendMs[i] = 0;
  }
}

}  // namespace

namespace motion {

void init() {
  // Ensure the link is initialised; the HID controller waits for the main
  // controller to announce readiness in the Arduino sketch before invoking
  // motion functions.
}

void setManualRate(Axis axis, float rpm) {
  size_t index = axisIndex(axis);
  float previous = lastManualRpm[index];
  uint32_t now = millis();

  bool shouldSend = false;
  if (!std::isfinite(previous)) {
    shouldSend = true;
  } else if (fabsf(previous - rpm) > kManualRpmDelta) {
    shouldSend = true;
  } else if (lastManualSendMs[index] == 0 ||
             (now - lastManualSendMs[index]) >= kManualRefreshIntervalMs) {
    shouldSend = true;
  }

  if (!shouldSend) {
    return;
  }

  if (callAndUpdate("SET_MANUAL_RPM",
                    {String(axisToString(axis)), formatFloat(rpm)})) {
    lastManualRpm[index] = rpm;
    lastManualSendMs[index] = now;
  }
}

void setManualStepsPerSecond(Axis axis, double stepsPerSecond) {
  if (callAndUpdate("SET_MANUAL_SPS",
                    {String(axisToString(axis)), formatFloat(stepsPerSecond)})) {
    lastManualRpm[axisIndex(axis)] = std::numeric_limits<float>::quiet_NaN();
    lastManualSendMs[axisIndex(axis)] = 0;
  }
}

void setGotoStepsPerSecond(Axis axis, double stepsPerSecond) {
  callAndUpdate("SET_GOTO_SPS",
                {String(axisToString(axis)), formatFloat(stepsPerSecond)});
}

void clearGotoRates() { callAndUpdate("CLEAR_GOTO", {}); }

void stopAll() {
  if (callAndUpdate("STOP_ALL", {})) {
    invalidateManualCache();
  }
}

void setTrackingEnabled(bool enabled) {
  callAndUpdate("SET_TRACKING_ENABLED", {enabled ? "1" : "0"});
}

void setTrackingRates(double azDegPerSec, double altDegPerSec) {
  callAndUpdate("SET_TRACKING_RATES",
                {formatFloat(azDegPerSec), formatFloat(altDegPerSec)});
}

bool isManualMotionActive() {
  return systemState.joystickActive;
}

int64_t getStepCount(Axis axis) {
  std::vector<String> payload;
  if (!callAndUpdate("GET_STEP_COUNT", {String(axisToString(axis))}, &payload)) {
    return 0;
  }
  if (payload.empty()) {
    return 0;
  }
  return parseInt64(payload.front());
}

void setStepCount(Axis axis, int64_t value) {
  if (callAndUpdate("SET_STEP_COUNT",
                    {String(axisToString(axis)), String(value)})) {
    lastManualRpm[axisIndex(axis)] = std::numeric_limits<float>::quiet_NaN();
    lastManualSendMs[axisIndex(axis)] = 0;
  }
}

double stepsToAzDegrees(int64_t steps) {
  std::vector<String> payload;
  if (!callAndUpdate("STEPS_TO_AZ", {String(steps)}, &payload)) {
    return 0.0;
  }
  if (payload.empty()) {
    return 0.0;
  }
  return parseDouble(payload.front());
}

double stepsToAltDegrees(int64_t steps) {
  std::vector<String> payload;
  if (!callAndUpdate("STEPS_TO_ALT", {String(steps)}, &payload)) {
    return 0.0;
  }
  if (payload.empty()) {
    return 0.0;
  }
  return parseDouble(payload.front());
}

int64_t azDegreesToSteps(double degrees) {
  std::vector<String> payload;
  if (!callAndUpdate("AZ_TO_STEPS", {formatFloat(degrees)}, &payload)) {
    return 0;
  }
  if (payload.empty()) {
    return 0;
  }
  return parseInt64(payload.front());
}

int64_t altDegreesToSteps(double degrees) {
  std::vector<String> payload;
  if (!callAndUpdate("ALT_TO_STEPS", {formatFloat(degrees)}, &payload)) {
    return 0;
  }
  if (payload.empty()) {
    return 0;
  }
  return parseInt64(payload.front());
}

double getMinAltitudeDegrees() { return -5.0; }

double getMaxAltitudeDegrees() { return 90.0; }

void applyCalibration(const AxisCalibration& calibration) {
  callAndUpdate("APPLY_CALIBRATION",
                {formatFloat(calibration.stepsPerDegreeAz),
                 formatFloat(calibration.stepsPerDegreeAlt),
                 String(calibration.azHomeOffset),
                 String(calibration.altHomeOffset)});
}

void setBacklash(const BacklashConfig& backlash) {
  callAndUpdate("SET_BACKLASH",
                {String(backlash.azSteps), String(backlash.altSteps)});
}

void setAltitudeLimitsEnabled(bool enabled) {
  callAndUpdate("SET_ALT_LIMITS_ENABLED", {enabled ? "1" : "0"});
}

bool setMotorInversion(bool invertAz, bool invertAlt) {
  desiredMotorInvertAz = invertAz;
  desiredMotorInvertAlt = invertAlt;
  bool success = callAndUpdate("SET_MOTOR_ORIENTATION",
                               {invertAz ? "1" : "0", invertAlt ? "1" : "0"});
  if (success) {
    pendingMotorOrientation = false;
  } else {
    pendingMotorOrientation = true;
    lastMotorOrientationAttemptMs = millis();
  }
  return success;
}

void servicePendingOperations() {
  if (!pendingMotorOrientation) {
    return;
  }
  if (!comm::isLinkActive()) {
    return;
  }
  uint32_t now = millis();
  if (lastMotorOrientationAttemptMs != 0 &&
      (now - lastMotorOrientationAttemptMs) < kMotorOrientationRetryIntervalMs) {
    return;
  }
  bool success = callAndUpdate("SET_MOTOR_ORIENTATION",
                               {desiredMotorInvertAz ? "1" : "0",
                                desiredMotorInvertAlt ? "1" : "0"});
  lastMotorOrientationAttemptMs = now;
  if (success) {
    pendingMotorOrientation = false;
  }
}

int32_t getBacklashSteps(Axis axis) {
  std::vector<String> payload;
  if (!callAndUpdate("GET_BACKLASH", {String(axisToString(axis))}, &payload)) {
    return 0;
  }
  if (payload.empty()) {
    return 0;
  }
  return static_cast<int32_t>(payload.front().toInt());
}

int8_t getLastDirection(Axis axis) {
  std::vector<String> payload;
  if (!callAndUpdate("GET_LAST_DIR", {String(axisToString(axis))}, &payload)) {
    return 0;
  }
  if (payload.empty()) {
    return 0;
  }
  return static_cast<int8_t>(payload.front().toInt());
}

}  // namespace motion

#endif  // DEVICE_ROLE_HID
