#include "role_config.h"

#include "wifi_ota.h"

#if defined(DEVICE_ROLE_HID)

#include <math.h>

#include "catalog.h"
#include "comm.h"
#include "config.h"
#include "debug.h"
#include "display_menu.h"
#include "input.h"
#include "motion.h"
#include "state.h"
#include "storage.h"

namespace {
bool g_mountLinkReady = false;
uint32_t g_linkInactiveSinceMs = 0;
uint32_t g_linkActiveSinceMs = 0;
constexpr uint32_t kLinkReadyConfirmMs = 250;
constexpr uint32_t kLinkOfflineConfirmMs = 750;
constexpr float kDegreesPerSecondPerRpm = 360.0f / 60.0f;

float shapeJoystickInput(float value) {
  float magnitude = fabsf(value);
  if (magnitude == 0.0f) {
    return 0.0f;
  }

  // Blend a small linear component with a squared response so that
  // tiny joystick motions yield very fine manual rates while full
  // deflection still reaches the maximum speed. This gives short,
  // quick adjustments a more incremental feel.
  constexpr float kLinearBlend = 0.25f;
  float shapedMagnitude =
      kLinearBlend * magnitude + (1.0f - kLinearBlend) * magnitude * magnitude;
  return (value < 0.0f) ? -shapedMagnitude : shapedMagnitude;
}

void initDebugSerial() {
  Serial.begin(config::USB_DEBUG_BAUD);
  delay(50);
  debug::init();
  Serial.println("[HID] Boot");
  debug::printStartupSummary();
  debug::recordEvent("boot");
}
}

void setup() {
  initDebugSerial();
  debug::recordEvent("setup_start");
  wifi_ota::init();
  debug::recordEvent("wifi_ota_init");
  comm::initLink();
  debug::recordEvent("comm_init");

  display_menu::init();
  display_menu::showBootMessage();

  storage::init();
  debug::recordEvent("storage_init");
  systemState.polarAligned = storage::getConfig().polarAligned;
  systemState.selectedCatalogIndex = -1;
  systemState.selectedCatalogTypeIndex = -1;
  systemState.mountLinkReady = false;
  systemState.manualCommandOk = false;

  input::init();
  debug::recordEvent("input_init");
  if (storage::getConfig().joystickCalibrated) {
    input::setJoystickCalibration(storage::getConfig().joystickCalibration);
  } else {
    display_menu::showCalibrationStart();
    JoystickCalibration calibration = input::calibrateJoystick();
    input::setJoystickCalibration(calibration);
    storage::setJoystickCalibration(calibration);
    display_menu::showCalibrationResult(calibration.centerX, calibration.centerY);
  }

  motion::init();
  debug::recordEvent("motion_init");
  motion::applyCalibration(storage::getConfig().axisCalibration);
  motion::setBacklash(storage::getConfig().backlash);
  motion::setMotorInversion(storage::getConfig().motorInvertAz != 0,
                            storage::getConfig().motorInvertAlt != 0);
  display_menu::prepareStartupLockPrompt(systemState.polarAligned);
  display_menu::startTask();

  debug::recordEvent("display_ready");

  comm::waitForReady(5000);
  debug::recordEvent("wait_for_ready_done");
  display_menu::stopBootAnimation();
  display_menu::showReady();
  g_mountLinkReady = comm::isLinkActive();
  if (Serial) {
    Serial.println(g_mountLinkReady ? "[HID] Mount link ready"
                                    : "[HID] Mount link offline");
  }
  if (!g_mountLinkReady) {
    display_menu::showInfo("Mount link offline", 2000);
  }
  systemState.mountLinkReady = g_mountLinkReady;
  display_menu::setOrientationKnown(systemState.polarAligned);
  if (g_mountLinkReady) {
    motion::setMotorInversion(storage::getConfig().motorInvertAz != 0,
                              storage::getConfig().motorInvertAlt != 0);
  }

  if (!catalog::init()) {
    display_menu::showInfo("Catalog missing", 2000);
  }
}

void loop() {
  uint32_t loopStart = millis();
  debug::recordLoop(loopStart);
  debug::recordEvent("loop_start", loopStart);
  comm::updateLink();
  motion::servicePendingOperations();
  debug::recordEvent("loop_after_comm");
  display_menu::update();
  display_menu::handleInput();
  debug::recordEvent("loop_after_ui");

  float rawX = input::getJoystickNormalizedX();
  float rawY = input::getJoystickNormalizedY();
  const SystemConfig& systemConfig = storage::getConfig();
  float azInput = systemConfig.joystickSwapAxes ? rawY : rawX;
  float altInput = systemConfig.joystickSwapAxes ? rawX : rawY;
  if (systemConfig.joystickInvertAz) {
    azInput = -azInput;
  }
  if (systemConfig.joystickInvertAlt) {
    altInput = -altInput;
  }
  systemState.joystickX = azInput;
  systemState.joystickY = altInput;
  systemState.joystickButtonPressed = input::isJoystickButtonPressed();
  systemState.joystickActive = (fabs(azInput) > config::JOYSTICK_DEADZONE ||
                                fabs(altInput) > config::JOYSTICK_DEADZONE);
  float manualMaxRpm = config::MAX_RPM_MANUAL;
  float panningMaxSpeed = systemConfig.panningProfile.maxSpeedDegPerSec;
  if (panningMaxSpeed > 0.0f) {
    manualMaxRpm = panningMaxSpeed / kDegreesPerSecondPerRpm;
  }
  float shapedAzInput = shapeJoystickInput(azInput);
  float shapedAltInput = shapeJoystickInput(altInput);
  motion::setManualRate(Axis::Az, shapedAzInput * manualMaxRpm);
  motion::setManualRate(Axis::Alt, shapedAltInput * manualMaxRpm);
  debug::recordEvent("loop_after_manual");

  if (systemState.gotoActive) {
    if (input::consumeJoystickPress()) {
      systemState.gotoActive = false;
      motion::clearGotoRates();
      display_menu::showInfo("Goto aborted", 2000);
    }
  }

  if (input::consumeJoystickPress() && !systemState.gotoActive) {
    motion::stopAll();
    display_menu::stopTracking();
    display_menu::showInfo("Motion stopped", 2000);
  }

  bool linkActive = comm::isLinkActive();
  if (!linkActive) {
    if (comm::waitForReady(1)) {
      linkActive = comm::isLinkActive();
    }
  }
  debug::recordEvent("loop_after_link_poll");
  uint32_t nowMs = millis();
  if (linkActive) {
    g_linkInactiveSinceMs = 0;
    if (!g_mountLinkReady) {
      if (g_linkActiveSinceMs == 0) {
        g_linkActiveSinceMs = nowMs;
      } else if ((nowMs - g_linkActiveSinceMs) >= kLinkReadyConfirmMs) {
        g_mountLinkReady = true;
        g_linkActiveSinceMs = 0;
        systemState.manualCommandOk = true;
        display_menu::showInfo("Mount link ready", 2000);
        display_menu::setOrientationKnown(systemState.polarAligned);
        motion::setMotorInversion(storage::getConfig().motorInvertAz != 0,
                                  storage::getConfig().motorInvertAlt != 0);
        if (Serial) {
          Serial.println("[HID] Mount link re-established");
        }
        debug::recordEvent("link_reestablished");
      }
    } else {
      g_linkActiveSinceMs = 0;
    }
  } else {
    g_linkActiveSinceMs = 0;
    if (g_mountLinkReady) {
      if (g_linkInactiveSinceMs == 0) {
        g_linkInactiveSinceMs = nowMs;
      } else if ((nowMs - g_linkInactiveSinceMs) >= kLinkOfflineConfirmMs) {
        g_mountLinkReady = false;
        g_linkInactiveSinceMs = 0;
        systemState.manualCommandOk = false;
        display_menu::showInfo("Mount link offline", 2000);
        if (Serial) {
          Serial.println("[HID] Mount link lost");
        }
        debug::recordEvent("link_lost");
      }
    }
  }
  systemState.mountLinkReady = g_mountLinkReady;

  wifi_ota::update();
  debug::recordEvent("loop_before_delay");
  delay(20);
}

#elif defined(DEVICE_ROLE_MAIN)

#include <Arduino.h>
#include <cmath>
#include <cstdlib>

#include "comm.h"
#include "config.h"
#include "motion.h"
#include "state.h"
#include "storage.h"

namespace {

TaskHandle_t motorTaskHandle = nullptr;
TaskHandle_t commandTaskHandle = nullptr;

void initDebugSerial() {
  Serial.begin(config::USB_DEBUG_BAUD);
  delay(50);
  Serial.println("[MAIN] Boot");
}

bool parseAxis(const String& value, Axis& outAxis) {
  if (value.equalsIgnoreCase("AZ")) {
    outAxis = Axis::Az;
    return true;
  }
  if (value.equalsIgnoreCase("ALT")) {
    outAxis = Axis::Alt;
    return true;
  }
  return false;
}

String formatDouble(double value) { return String(value, 6); }

void handleRequest(const comm::Request& request) {
  const String& cmd = request.command;
  auto paramCount = request.params.size();

  auto requireParams = [&](size_t expected) {
    if (paramCount < expected) {
      comm::sendError(request.id, "Missing params");
      return false;
    }
    return true;
  };

  auto parseAxisParam = [&](size_t index, Axis& axis) {
    if (!parseAxis(request.params[index], axis)) {
      comm::sendError(request.id, "Invalid axis");
      return false;
    }
    return true;
  };

  if (cmd == "SET_MANUAL_RPM") {
    if (!requireParams(2)) return;
    Axis axis;
    if (!parseAxisParam(0, axis)) return;
    float rpm = request.params[1].toFloat();
    motion::setManualRate(axis, rpm);
    comm::sendOk(request.id, {});
  } else if (cmd == "SET_MANUAL_SPS") {
    if (!requireParams(2)) return;
    Axis axis;
    if (!parseAxisParam(0, axis)) return;
    double sps = strtod(request.params[1].c_str(), nullptr);
    motion::setManualStepsPerSecond(axis, sps);
    comm::sendOk(request.id, {});
  } else if (cmd == "SET_GOTO_SPS") {
    if (!requireParams(2)) return;
    Axis axis;
    if (!parseAxisParam(0, axis)) return;
    double sps = strtod(request.params[1].c_str(), nullptr);
    motion::setGotoStepsPerSecond(axis, sps);
    comm::sendOk(request.id, {});
  } else if (cmd == "CLEAR_GOTO") {
    motion::clearGotoRates();
    comm::sendOk(request.id, {});
  } else if (cmd == "STOP_ALL") {
    motion::stopAll();
    comm::sendOk(request.id, {});
  } else if (cmd == "SET_TRACKING_ENABLED") {
    if (!requireParams(1)) return;
    bool enabled = request.params[0] == "1";
    motion::setTrackingEnabled(enabled);
    comm::sendOk(request.id, {});
  } else if (cmd == "SET_TRACKING_RATES") {
    if (!requireParams(2)) return;
    double az = strtod(request.params[0].c_str(), nullptr);
    double alt = strtod(request.params[1].c_str(), nullptr);
    motion::setTrackingRates(az, alt);
    comm::sendOk(request.id, {});
  } else if (cmd == "SET_WIFI_ENABLED") {
    if (!requireParams(1)) return;
    bool enabled = request.params[0] == "1";
    wifi_ota::setEnabled(enabled);
    comm::sendOk(request.id, {});
  } else if (cmd == "GET_STEP_COUNT") {
    if (!requireParams(1)) return;
    Axis axis;
    if (!parseAxisParam(0, axis)) return;
    int64_t count = motion::getStepCount(axis);
    comm::sendOk(request.id, {String(count)});
  } else if (cmd == "SET_STEP_COUNT") {
    if (!requireParams(2)) return;
    Axis axis;
    if (!parseAxisParam(0, axis)) return;
    int64_t value = strtoll(request.params[1].c_str(), nullptr, 10);
    motion::setStepCount(axis, value);
    comm::sendOk(request.id, {});
  } else if (cmd == "STEPS_TO_AZ") {
    if (!requireParams(1)) return;
    int64_t steps = strtoll(request.params[0].c_str(), nullptr, 10);
    double degrees = motion::stepsToAzDegrees(steps);
    comm::sendOk(request.id, {formatDouble(degrees)});
  } else if (cmd == "STEPS_TO_ALT") {
    if (!requireParams(1)) return;
    int64_t steps = strtoll(request.params[0].c_str(), nullptr, 10);
    double degrees = motion::stepsToAltDegrees(steps);
    comm::sendOk(request.id, {formatDouble(degrees)});
  } else if (cmd == "AZ_TO_STEPS") {
    if (!requireParams(1)) return;
    double degrees = strtod(request.params[0].c_str(), nullptr);
    int64_t steps = motion::azDegreesToSteps(degrees);
    comm::sendOk(request.id, {String(steps)});
  } else if (cmd == "ALT_TO_STEPS") {
    if (!requireParams(1)) return;
    double degrees = strtod(request.params[0].c_str(), nullptr);
    int64_t steps = motion::altDegreesToSteps(degrees);
    comm::sendOk(request.id, {String(steps)});
  } else if (cmd == "APPLY_CALIBRATION") {
    if (!requireParams(4)) return;
    AxisCalibration calib{};
    calib.stepsPerDegreeAz = strtod(request.params[0].c_str(), nullptr);
    calib.stepsPerDegreeAlt = strtod(request.params[1].c_str(), nullptr);
    calib.azHomeOffset = strtoll(request.params[2].c_str(), nullptr, 10);
    calib.altHomeOffset = strtoll(request.params[3].c_str(), nullptr, 10);
    motion::applyCalibration(calib);
    comm::sendOk(request.id, {});
  } else if (cmd == "SET_BACKLASH") {
    if (!requireParams(2)) return;
    BacklashConfig config{};
    config.azSteps = static_cast<int32_t>(request.params[0].toInt());
    config.altSteps = static_cast<int32_t>(request.params[1].toInt());
    motion::setBacklash(config);
    comm::sendOk(request.id, {});
  } else if (cmd == "SET_MOTOR_ORIENTATION") {
    if (!requireParams(2)) return;
    bool invertAz = request.params[0] == "1";
    bool invertAlt = request.params[1] == "1";
    motion::setMotorInversion(invertAz, invertAlt);
    storage::setMotorInversion(invertAz, invertAlt);
    comm::sendOk(request.id, {});
  } else if (cmd == "SET_ALT_LIMITS_ENABLED") {
    if (!requireParams(1)) return;
    bool enabled = request.params[0] == "1";
    motion::setAltitudeLimitsEnabled(enabled);
    comm::sendOk(request.id, {});
  } else if (cmd == "GET_BACKLASH") {
    if (!requireParams(1)) return;
    Axis axis;
    if (!parseAxisParam(0, axis)) return;
    comm::sendOk(request.id, {String(motion::getBacklashSteps(axis))});
  } else if (cmd == "GET_LAST_DIR") {
    if (!requireParams(1)) return;
    Axis axis;
    if (!parseAxisParam(0, axis)) return;
    comm::sendOk(request.id, {String(motion::getLastDirection(axis))});
  } else {
    comm::sendError(request.id, "Unknown command");
  }
}

void commandTask(void*) {
  constexpr uint32_t kReadyIntervalMs = 500;
  uint32_t lastReadyMs = 0;
  comm::announceReady();
  lastReadyMs = millis();
  while (true) {
    comm::updateLink();
    comm::Request request;
    if (comm::readRequest(request, 100)) {
      handleRequest(request);
      lastReadyMs = millis();
    } else {
      uint32_t now = millis();
      if ((now - lastReadyMs) >= kReadyIntervalMs) {
        comm::announceReady();
        lastReadyMs = now;
      }
    }
  }
}

void motorTask(void*) {
  motion::motorTaskLoop();
}

}  // namespace

void setup() {
  initDebugSerial();
  wifi_ota::init();
  comm::initLink();
  storage::init();
  motion::init();
  motion::applyCalibration(storage::getConfig().axisCalibration);
  motion::setBacklash(storage::getConfig().backlash);
  motion::setMotorInversion(storage::getConfig().motorInvertAz != 0,
                            storage::getConfig().motorInvertAlt != 0);

  xTaskCreatePinnedToCore(motorTask, "motor", 4096, nullptr, 2, &motorTaskHandle,
                          1);
  xTaskCreatePinnedToCore(commandTask, "cmd", 8192, nullptr, 2,
                          &commandTaskHandle, 0);
  if (Serial) {
    Serial.println("[MAIN] Tasks started");
  }
}

void loop() {
  wifi_ota::update();
  vTaskDelay(pdMS_TO_TICKS(20));
}

#endif

