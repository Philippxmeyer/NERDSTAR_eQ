#pragma once

#include <Arduino.h>
#include "driver/uart.h"

#include "role_config.h"

namespace config {

// Role-specific pin mappings -------------------------------------------------

#if defined(DEVICE_ROLE_MAIN)

// Stepper driver pins for Azimuth axis
constexpr uint8_t EN_RA = 27;
constexpr uint8_t DIR_RA = 26;
constexpr uint8_t STEP_RA = 25;

// Stepper driver pins for Altitude axis
constexpr uint8_t EN_DEC = 14;
constexpr uint8_t DIR_DEC = 12;
constexpr uint8_t STEP_DEC = 13;

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
constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;

// Inter-board communication (UART2 on ESP32-WROOM)
constexpr uart_port_t COMM_UART_NUM = UART_NUM_2;
constexpr uint8_t COMM_TX_PIN = 17;
constexpr uint8_t COMM_RX_PIN = 16;

#else
#error "Unsupported device role"
#endif

constexpr uint32_t COMM_BAUD = 115200;
constexpr uint32_t COMM_RESPONSE_TIMEOUT_MS = 200;
constexpr uint32_t USB_DEBUG_BAUD = 115200;

// Motion configuration
constexpr double FULLSTEPS_PER_REV = 32.0 * 64.0; // 2048
constexpr double MICROSTEPS = 16.0;
// Fallback manual slewing limit (RPM) used if no panning profile cap is set
constexpr float MAX_RPM_MANUAL = 3.0f;
constexpr float GEAR_RATIO = 4.0f; // 1:4 reduction motor:telescope
constexpr float JOYSTICK_DEADZONE = 0.03f;
constexpr int JOYSTICK_X_DIRECTION = 1;  // Use -1 to invert the azimuth axis
constexpr int JOYSTICK_Y_DIRECTION = 1;  // Use -1 to invert the altitude axis

// Display configuration
constexpr uint8_t OLED_WIDTH = 128;
constexpr uint8_t OLED_HEIGHT = 64;

// WiFi / OTA configuration
constexpr const char* WIFI_HOSTNAME_PREFIX = "nerdstar";
constexpr const char* WIFI_STA_SSID = "";       // Set to home network SSID
constexpr const char* WIFI_STA_PASSWORD = "";   // Set to home network password

// Astronomy constants
constexpr double SIDEREAL_DAY_SECONDS = 86164.0905;
constexpr double POLARIS_RA_HOURS = 2.530301;     // 02h 31m 49s
constexpr double POLARIS_DEC_DEGREES = 89.2641;   // +89° 15' 50"
constexpr double OBSERVER_LATITUDE_DEG = 52.5200;  // Default: Berlin
constexpr double OBSERVER_LONGITUDE_DEG = 13.4050; // East positive

} // namespace config

