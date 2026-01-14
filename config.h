#pragma once

#include <Arduino.h>
#include "driver/uart.h"

#include "role_config.h"

// Display driver selection ---------------------------------------------------
//
// Define CONFIG_DISPLAY_DRIVER to either DISPLAY_DRIVER_SSD1306 or
// DISPLAY_DRIVER_SH1106 before including this header if you need to override
// the default. This allows using either display without editing the menu code.
// #define DISPLAY_DRIVER_SSD1306 1
#define DISPLAY_DRIVER_SH1106 2

#ifndef CONFIG_DISPLAY_DRIVER
#define CONFIG_DISPLAY_DRIVER DISPLAY_DRIVER_SH1106
#endif

namespace config {

// Role-specific pin mappings -------------------------------------------------

#if defined(DEVICE_ROLE_MAIN)

// Stepper driver pins for Azimuth axis
constexpr uint8_t EN_RA = 27;
constexpr uint8_t DIR_RA = 26;
constexpr uint8_t STEP_RA = 25;

// Stepper driver pins for Altitude axis
constexpr uint8_t EN_DEC = 14;
constexpr uint8_t DIR_DEC = 13;
constexpr uint8_t STEP_DEC = 12;

// Inter-board communication (dedicated UART freed from the TMC drivers)
constexpr uart_port_t COMM_UART_NUM = UART_NUM_2;
constexpr uint8_t COMM_TX_PIN = 17;
constexpr uint8_t COMM_RX_PIN = 16;

#elif defined(DEVICE_ROLE_HID)

// Joystick KY-023 pins (ESP32-WROOM DevKit)
constexpr uint8_t JOY_X = 34;  // ADC1_CH6, input only
constexpr uint8_t JOY_Y = 35;  // ADC1_CH7, input only
constexpr uint8_t JOY_BTN = 27;  // LOW active, internal pull-up available

// Rotary encoder pins (ESP32-WROOM DevKit)
constexpr uint8_t ROT_A = 18;
constexpr uint8_t ROT_B = 19;
constexpr uint8_t ROT_BTN = 23;

// OLED + RTC I2C pins (ESP32-WROOM default)
constexpr uint8_t SDA_PIN = 22;
constexpr uint8_t SCL_PIN = 21;

// Inter-board communication (UART2 on ESP32-WROOM)
constexpr uart_port_t COMM_UART_NUM = UART_NUM_2;
constexpr uint8_t COMM_TX_PIN = 17;
constexpr uint8_t COMM_RX_PIN = 16;

#else
#error "Unsupported device role"
#endif

constexpr uint32_t COMM_BAUD = 57600;
constexpr uint32_t COMM_HEARTBEAT_INTERVAL_MS = 200;
constexpr uint32_t COMM_HEARTBEAT_TIMEOUT_MS = 2000;
constexpr uint32_t COMM_RESPONSE_TIMEOUT_MS = 200;
constexpr uint32_t USB_DEBUG_BAUD = 115200;

// Motion configuration
constexpr double FULLSTEPS_PER_REV = 200 * 50.0; // 10.000
constexpr double MICROSTEPS = 16.0;
// Fallback manual slewing limit (RPM) used if no panning profile cap is set
constexpr float MAX_RPM_MANUAL = 3.0f;
constexpr float GEAR_RATIO = 4.0f; // 1:4 reduction motor:telescope
constexpr float JOYSTICK_DEADZONE = 0.03f;
constexpr int JOYSTICK_X_DIRECTION = 1;  // Use -1 to invert the azimuth axis
constexpr int JOYSTICK_Y_DIRECTION = 1;  // Use -1 to invert the altitude axis
constexpr float DEFAULT_GOTO_MAX_SPEED_DEG_PER_SEC = 5.0f;
constexpr float DEFAULT_GOTO_ACCEL_DEG_PER_SEC2 = 2.0f;
constexpr float DEFAULT_GOTO_DECEL_DEG_PER_SEC2 = 2.0f;
constexpr float DEFAULT_PAN_MAX_SPEED_DEG_PER_SEC = 5.0f;
constexpr float DEFAULT_PAN_ACCEL_DEG_PER_SEC2 = 2.0f;
constexpr float DEFAULT_PAN_DECEL_DEG_PER_SEC2 = 2.0f;
constexpr int32_t DEFAULT_TIMEZONE_OFFSET_MINUTES = 60;
constexpr uint8_t DEFAULT_DISPLAY_CONTRAST = 0x7F;
constexpr double DEFAULT_ORIENTATION_AZ_BIAS_DEG = 0.0;
constexpr double DEFAULT_ORIENTATION_ALT_BIAS_DEG = 0.0;
constexpr double DEFAULT_ORIENTATION_SAMPLE_WEIGHT = 0.0;
constexpr int DEFAULT_JOYSTICK_CENTER = 2048;
constexpr double DEFAULT_AXIS_STEPS_PER_DEG = 0.0;
constexpr int64_t DEFAULT_AZ_HOME_OFFSET = 0;
constexpr int64_t DEFAULT_ALT_HOME_OFFSET = 0;
constexpr int32_t DEFAULT_BACKLASH_AZ_STEPS = 0;
constexpr int32_t DEFAULT_BACKLASH_ALT_STEPS = 0;
constexpr bool DEFAULT_JOYSTICK_CALIBRATED = false;
constexpr bool DEFAULT_AXIS_CALIBRATED = false;
constexpr bool DEFAULT_POLAR_ALIGNED = false;
constexpr uint32_t DEFAULT_LAST_RTC_EPOCH = 0;
constexpr uint8_t DEFAULT_JOYSTICK_SWAP_AXES = 0;
constexpr uint8_t DEFAULT_JOYSTICK_INVERT_AZ = 0;
constexpr uint8_t DEFAULT_JOYSTICK_INVERT_ALT = 0;
constexpr uint8_t DEFAULT_MOTOR_INVERT_AZ = 0;
constexpr uint8_t DEFAULT_MOTOR_INVERT_ALT = 0;

// Display configuration
constexpr uint8_t OLED_WIDTH = 128;
constexpr uint8_t OLED_HEIGHT = 64;

// WiFi / OTA configuration
constexpr const char* WIFI_HOSTNAME_PREFIX = "nerdstar";
constexpr const char* WIFI_STA_SSID = "Waldmeyer";       // Set to home network SSID
constexpr const char* WIFI_STA_PASSWORD = "94650411894394693580";   // Set to home network password
constexpr const char* WIFI_AP_SSID = "NERDSTAR";
constexpr const char* WIFI_AP_PASSWORD = "stardust42";
constexpr uint8_t WIFI_AP_CHANNEL = 6;

// Stellarium / networking configuration
constexpr uint16_t STELLARIUM_TCP_PORT = 10001;

// Astronomy constants
constexpr double SIDEREAL_DAY_SECONDS = 86164.0905;
constexpr double POLARIS_RA_HOURS = 2.530301;     // 02h 31m 49s
constexpr double POLARIS_DEC_DEGREES = 89.2641;   // +89° 15' 50"
constexpr double OBSERVER_LATITUDE_DEG = 50.7354;  // Default: Daaden
constexpr double OBSERVER_LONGITUDE_DEG = 7.9671; // West positive

} // namespace config
