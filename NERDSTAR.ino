#include "role_config.h"

#include <Arduino.h>
#include <cmath>
#include <cstdlib>

#include "comm.h"
#include "config.h"
#include "motion.h"
#include "state.h"
#include "storage.h"
#include "stellarium_link.h"
#include "time_utils.h"
#include "wifi_ota.h"

namespace {

TaskHandle_t motorTaskHandle = nullptr;
TaskHandle_t commandTaskHandle = nullptr;

void initSerial() {
  Serial.begin(config::USB_DEBUG_BAUD);
  delay(50);
  Serial.println("[MAIN] Boot");
}

bool parseAxis(const String& value, Axis& outAxis) {
  if (value.equalsIgnoreCase("RA")) {
    outAxis = Axis::Ra;
    return true;
  }
  if (value.equalsIgnoreCase("DEC")) {
    outAxis = Axis::Dec;
    return true;
  }
  if (value.equalsIgnoreCase("AZ")) {
    outAxis = Axis::Ra;
    return true;
  }
  if (value.equalsIgnoreCase("ALT")) {
    outAxis = Axis::Dec;
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

  if (cmd == "PING") {
    comm::sendResponse(request.id, {"OK"});
    return;
  }

  if (cmd == "STOP_ALL") {
    motion::stopAll();
    comm::sendResponse(request.id, {"OK"});
    return;
  }

  if (cmd == "SET_MANUAL_RPM") {
    if (!requireParams(2)) return;
    Axis axis;
    if (!parseAxisParam(0, axis)) return;
    float rpm = static_cast<float>(request.params[1].toFloat());
    motion::setManualRate(axis, rpm);
    comm::sendResponse(request.id, {"OK"});
    return;
  }

  if (cmd == "SET_MANUAL_SPS") {
    if (!requireParams(2)) return;
    Axis axis;
    if (!parseAxisParam(0, axis)) return;
    double stepsPerSecond = strtod(request.params[1].c_str(), nullptr);
    motion::setManualStepsPerSecond(axis, stepsPerSecond);
    comm::sendResponse(request.id, {"OK"});
    return;
  }

  if (cmd == "SET_GOTO_SPS") {
    if (!requireParams(2)) return;
    Axis axis;
    if (!parseAxisParam(0, axis)) return;
    double stepsPerSecond = strtod(request.params[1].c_str(), nullptr);
    motion::setGotoStepsPerSecond(axis, stepsPerSecond);
    comm::sendResponse(request.id, {"OK"});
    return;
  }

  if (cmd == "CLEAR_GOTO") {
    motion::clearGotoRates();
    comm::sendResponse(request.id, {"OK"});
    return;
  }

  if (cmd == "SET_TRACKING_ENABLED") {
    if (!requireParams(1)) return;
    bool enabled = request.params[0].toInt() != 0;
    motion::setTrackingEnabled(enabled);
    comm::sendResponse(request.id, {"OK"});
    return;
  }

  if (cmd == "SET_TRACKING_RATES") {
    if (!requireParams(2)) return;
    double raDegPerSec = strtod(request.params[0].c_str(), nullptr);
    double decDegPerSec = strtod(request.params[1].c_str(), nullptr);
    motion::setTrackingRates(raDegPerSec, decDegPerSec);
    comm::sendResponse(request.id, {"OK"});
    return;
  }

  if (cmd == "GET_STEP_COUNT") {
    if (!requireParams(1)) return;
    Axis axis;
    if (!parseAxisParam(0, axis)) return;
    comm::sendResponse(request.id, {String(motion::getStepCount(axis))});
    return;
  }

  if (cmd == "SET_STEP_COUNT") {
    if (!requireParams(2)) return;
    Axis axis;
    if (!parseAxisParam(0, axis)) return;
    int64_t value = static_cast<int64_t>(strtoll(request.params[1].c_str(), nullptr, 10));
    motion::setStepCount(axis, value);
    comm::sendResponse(request.id, {"OK"});
    return;
  }

  if (cmd == "STEPS_TO_RA" || cmd == "STEPS_TO_AZ") {
    if (!requireParams(1)) return;
    int64_t steps = static_cast<int64_t>(strtoll(request.params[0].c_str(), nullptr, 10));
    comm::sendResponse(request.id, {formatDouble(motion::stepsToRaDegrees(steps))});
    return;
  }

  if (cmd == "STEPS_TO_DEC" || cmd == "STEPS_TO_ALT") {
    if (!requireParams(1)) return;
    int64_t steps = static_cast<int64_t>(strtoll(request.params[0].c_str(), nullptr, 10));
    comm::sendResponse(request.id, {formatDouble(motion::stepsToDecDegrees(steps))});
    return;
  }

  if (cmd == "RA_TO_STEPS" || cmd == "AZ_TO_STEPS") {
    if (!requireParams(1)) return;
    double raDeg = strtod(request.params[0].c_str(), nullptr);
    comm::sendResponse(request.id, {String(motion::raDegreesToSteps(raDeg))});
    return;
  }

  if (cmd == "DEC_TO_STEPS" || cmd == "ALT_TO_STEPS") {
    if (!requireParams(1)) return;
    double decDeg = strtod(request.params[0].c_str(), nullptr);
    comm::sendResponse(request.id, {String(motion::decDegreesToSteps(decDeg))});
    return;
  }

  if (cmd == "APPLY_CALIBRATION") {
    if (!requireParams(4)) return;
    AxisCalibration calibration{};
    calibration.stepsPerDegreeAz = strtod(request.params[0].c_str(), nullptr);
    calibration.stepsPerDegreeAlt = strtod(request.params[1].c_str(), nullptr);
    calibration.azHomeOffset = static_cast<int64_t>(strtoll(request.params[2].c_str(), nullptr, 10));
    calibration.altHomeOffset = static_cast<int64_t>(strtoll(request.params[3].c_str(), nullptr, 10));
    motion::applyCalibration(calibration);
    comm::sendResponse(request.id, {"OK"});
    return;
  }

  if (cmd == "SET_BACKLASH") {
    if (!requireParams(2)) return;
    BacklashConfig backlash{};
    backlash.azSteps = request.params[0].toInt();
    backlash.altSteps = request.params[1].toInt();
    motion::setBacklash(backlash);
    comm::sendResponse(request.id, {"OK"});
    return;
  }

  if (cmd == "SET_BACKLASH_TAKEUP_RATE") {
    if (!requireParams(1)) return;
    int32_t rate = request.params[0].toInt();
    motion::setBacklashTakeupRateStepsPerSecond(rate);
    comm::sendResponse(request.id, {"OK"});
    return;
  }

  if (cmd == "GET_BACKLASH") {
    comm::sendResponse(
        request.id,
        {String(motion::getBacklashSteps(Axis::Ra)), String(motion::getBacklashSteps(Axis::Dec))});
    return;
  }

  if (cmd == "GET_BACKLASH_TAKEUP_RATE") {
    comm::sendResponse(request.id, {String(motion::getBacklashTakeupRateStepsPerSecond())});
    return;
  }

  if (cmd == "GET_LAST_DIRECTION") {
    if (!requireParams(1)) return;
    Axis axis;
    if (!parseAxisParam(0, axis)) return;
    comm::sendResponse(request.id, {String(motion::getLastDirection(axis))});
    return;
  }

  if (cmd == "SET_ALTITUDE_LIMITS") {
    if (!requireParams(1)) return;
    bool enabled = request.params[0].toInt() != 0;
    motion::setAltitudeLimitsEnabled(enabled);
    comm::sendResponse(request.id, {"OK"});
    return;
  }

  if (cmd == "SET_MOTOR_INVERT") {
    if (!requireParams(2)) return;
    bool invertAz = request.params[0].toInt() != 0;
    bool invertAlt = request.params[1].toInt() != 0;
    bool ok = motion::setMotorInversion(invertAz, invertAlt);
    comm::sendResponse(request.id, {ok ? "1" : "0"});
    return;
  }

  if (cmd == "GET_MOTOR_INVERT") {
    const SystemConfig& cfg = storage::getConfig();
    comm::sendResponse(request.id,
                       {String(cfg.motorInvertAz ? 1 : 0), String(cfg.motorInvertAlt ? 1 : 0)});
    return;
  }

  comm::sendError(request.id, "Unknown command");
}

void commandTask(void*) {
  for (;;) {
    comm::updateLink();
    while (comm::hasRequest()) {
      handleRequest(comm::nextRequest());
    }
    motion::servicePendingOperations();
    vTaskDelay(1);
  }
}

void motorTask(void*) {
  motion::motorTaskLoop();
}

}  // namespace

void setup() {
  initSerial();
  storage::init();
  motion::init();
  motion::applyCalibration(storage::getConfig().axisCalibration);
  motion::setBacklash(storage::getConfig().backlash);
  motion::setBacklashTakeupRateStepsPerSecond(
      storage::getConfig().backlashTakeupRateStepsPerSecond);
  motion::setMotorInversion(storage::getConfig().motorInvertAz != 0,
                            storage::getConfig().motorInvertAlt != 0);
  motion::setAltitudeLimitsEnabled(true);
  time_utils::initRtc();
  comm::initLink();
  wifi_ota::init();
  stellarium_link::init();

  xTaskCreatePinnedToCore(motorTask, "motor", 4096, nullptr, 2, &motorTaskHandle, 1);
  xTaskCreatePinnedToCore(commandTask, "command", 6144, nullptr, 1, &commandTaskHandle, 0);

  Serial.println("[MAIN] Ready");
}

void loop() {
  wifi_ota::update();
  stellarium_link::update();
  delay(10);
}
