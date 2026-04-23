#pragma once

#include <Arduino.h>
#include "driver/uart.h"

#include "role_config.h"

namespace config {

// Pin mappings ----------------------------------------------------------------
constexpr uint8_t EN_RA = 27;
constexpr uint8_t DIR_RA = 26;
constexpr uint8_t STEP_RA = 25;

constexpr uint8_t EN_DEC = 14;
constexpr uint8_t DIR_DEC = 13;
constexpr uint8_t STEP_DEC = 12;

constexpr uart_port_t COMM_UART_NUM = UART_NUM_2;
constexpr uint8_t COMM_TX_PIN = 17;
constexpr uint8_t COMM_RX_PIN = 16;

// RTC I2C pins (DS3231)
constexpr uint8_t RTC_SDA_PIN = 21;
constexpr uint8_t RTC_SCL_PIN = 22;

// Communication
constexpr uint32_t COMM_BAUD = 57600;
constexpr uint32_t COMM_HEARTBEAT_INTERVAL_MS = 200;
constexpr uint32_t COMM_HEARTBEAT_TIMEOUT_MS = 2000;
constexpr uint32_t COMM_RESPONSE_TIMEOUT_MS = 200;

// USB serial is the LX200 command channel used by Stellarmate / INDI.
// 9600 baud matches the default speed of Meade-compatible LX200 drivers.
constexpr uint32_t USB_LX200_BAUD = 9600;

// Motion configuration
constexpr double FULLSTEPS_PER_REV = 200 * 50.0;
constexpr double MICROSTEPS = 16.0;
constexpr float GEAR_RATIO = 4.0f;
constexpr float MAX_RPM_MANUAL = 3.0f;

constexpr float DEFAULT_GOTO_MAX_SPEED_DEG_PER_SEC = 5.0f;
constexpr float DEFAULT_GOTO_ACCEL_DEG_PER_SEC2 = 2.0f;
constexpr float DEFAULT_GOTO_DECEL_DEG_PER_SEC2 = 2.0f;
constexpr float DEFAULT_PAN_MAX_SPEED_DEG_PER_SEC = 5.0f;
constexpr float DEFAULT_PAN_ACCEL_DEG_PER_SEC2 = 2.0f;
constexpr float DEFAULT_PAN_DECEL_DEG_PER_SEC2 = 2.0f;

constexpr double DEFAULT_AXIS_STEPS_PER_DEG = 0.0;
constexpr int64_t DEFAULT_AZ_HOME_OFFSET = 0;
constexpr int64_t DEFAULT_ALT_HOME_OFFSET = 0;
constexpr int32_t DEFAULT_BACKLASH_AZ_STEPS = 259;
constexpr int32_t DEFAULT_BACKLASH_ALT_STEPS = 259;
constexpr int32_t DEFAULT_BACKLASH_TAKEUP_STEPS_PER_SEC = 1036;
constexpr uint8_t DEFAULT_MOTOR_INVERT_AZ = 1;
constexpr uint8_t DEFAULT_MOTOR_INVERT_ALT = 1;

constexpr double DEFAULT_ORIENTATION_AZ_BIAS_DEG = 0.0;
constexpr double DEFAULT_ORIENTATION_ALT_BIAS_DEG = 0.0;
constexpr double DEFAULT_ORIENTATION_SAMPLE_WEIGHT = 0.0;

// LX200 control
constexpr double LX200_GOTO_SPEED_DEG_PER_SEC = 2.0;
constexpr double LX200_MANUAL_SPEED_DEG_PER_SEC = 0.5;
constexpr double LX200_GOTO_TOLERANCE_DEG = 0.05;
constexpr const char* LX200_FIRMWARE_VERSION = "1.0";

// Astronomy constants
constexpr double SIDEREAL_DAY_SECONDS = 86164.0905;

} // namespace config
