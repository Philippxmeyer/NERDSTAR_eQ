#include "debug.h"

#if defined(DEVICE_ROLE_HID)

#include <Arduino.h>
#include <esp_attr.h>
#include <esp_system.h>

#include <cstring>

namespace {
constexpr uint32_t kMagic = 0x48524442;  // 'HDRB'
constexpr size_t kMaxEventLength = 64;
constexpr size_t kMaxCommandLength = 32;
constexpr size_t kMaxErrorLength = 80;

struct RtcDebugInfo {
  uint32_t magic;
  uint32_t bootCount;
  uint32_t lastResetReason;
  uint32_t lastLoopMs;
  uint32_t lastEventMs;
  uint32_t lastCommAttemptMs;
  uint32_t lastCommSuccessMs;
  uint32_t lastCommFailureMs;
  char lastEvent[kMaxEventLength];
  char lastCommCommand[kMaxCommandLength];
  char lastCommError[kMaxErrorLength];
};

RTC_DATA_ATTR RtcDebugInfo g_debugInfo = {};

void copyString(char* dest, size_t capacity, const char* source) {
  if (capacity == 0) {
    return;
  }
  if (source == nullptr) {
    dest[0] = '\0';
    return;
  }
  std::strncpy(dest, source, capacity - 1);
  dest[capacity - 1] = '\0';
}

const char* resetReasonToString(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_UNKNOWN:
      return "Unknown";
    case ESP_RST_POWERON:
      return "Power-on";
    case ESP_RST_EXT:
      return "External";
    case ESP_RST_SW:
      return "Software";
    case ESP_RST_PANIC:
      return "Panic";
    case ESP_RST_INT_WDT:
      return "Interrupt WDT";
    case ESP_RST_TASK_WDT:
      return "Task WDT";
    case ESP_RST_WDT:
      return "Other WDT";
    case ESP_RST_DEEPSLEEP:
      return "Deep sleep";
    case ESP_RST_BROWNOUT:
      return "Brownout";
    case ESP_RST_SDIO:
      return "SDIO";
    case ESP_RST_USB:
      return "USB";
    case ESP_RST_JTAG:
      return "JTAG";
    default:
      return "Reserved";
  }
}

void printCommSummary() {
  if (!Serial) {
    return;
  }
  if (g_debugInfo.lastCommCommand[0] == '\0') {
    return;
  }
  Serial.printf("[DEBUG] Last RPC command: %s\n", g_debugInfo.lastCommCommand);
  if (g_debugInfo.lastCommAttemptMs != 0) {
    Serial.printf("[DEBUG]   Last attempt at t=%lums\n",
                  static_cast<unsigned long>(g_debugInfo.lastCommAttemptMs));
  }
  if (g_debugInfo.lastCommSuccessMs != 0) {
    Serial.printf("[DEBUG]   Last success at t=%lums\n",
                  static_cast<unsigned long>(g_debugInfo.lastCommSuccessMs));
  }
  if (g_debugInfo.lastCommFailureMs != 0) {
    Serial.printf("[DEBUG]   Last failure at t=%lums, error=%s\n",
                  static_cast<unsigned long>(g_debugInfo.lastCommFailureMs),
                  g_debugInfo.lastCommError[0] != '\0' ? g_debugInfo.lastCommError
                                                        : "<none>");
  }
}

}  // namespace

namespace debug {

void init() {
  if (g_debugInfo.magic != kMagic) {
    std::memset(&g_debugInfo, 0, sizeof(g_debugInfo));
    g_debugInfo.magic = kMagic;
  }
  g_debugInfo.bootCount += 1;
  g_debugInfo.lastResetReason = static_cast<uint32_t>(esp_reset_reason());
}

void printStartupSummary() {
  if (!Serial) {
    return;
  }
  Serial.printf("[DEBUG] Boot count: %lu\n",
                static_cast<unsigned long>(g_debugInfo.bootCount));
  Serial.printf("[DEBUG] Reset reason: %s (%lu)\n",
                resetReasonToString(
                    static_cast<esp_reset_reason_t>(g_debugInfo.lastResetReason)),
                static_cast<unsigned long>(g_debugInfo.lastResetReason));
  if (g_debugInfo.lastLoopMs != 0) {
    Serial.printf("[DEBUG] Last loop heartbeat at t=%lums\n",
                  static_cast<unsigned long>(g_debugInfo.lastLoopMs));
  }
  if (g_debugInfo.lastEvent[0] != '\0') {
    Serial.printf("[DEBUG] Last event: %s (t=%lums)\n", g_debugInfo.lastEvent,
                  static_cast<unsigned long>(g_debugInfo.lastEventMs));
  }
  printCommSummary();
}

void recordEvent(const char* label) {
  recordEvent(label, millis());
}

void recordEvent(const char* label, uint32_t timestampMs) {
  copyString(g_debugInfo.lastEvent, sizeof(g_debugInfo.lastEvent), label);
  g_debugInfo.lastEventMs = timestampMs;
}

void recordLoop(uint32_t timestampMs) {
  g_debugInfo.lastLoopMs = timestampMs;
}

void recordCommAttempt(const char* command) {
  copyString(g_debugInfo.lastCommCommand, sizeof(g_debugInfo.lastCommCommand),
             command);
  g_debugInfo.lastCommAttemptMs = millis();
  g_debugInfo.lastCommError[0] = '\0';
  g_debugInfo.lastCommFailureMs = 0;
}

void recordCommSuccess(const char* command) {
  copyString(g_debugInfo.lastCommCommand, sizeof(g_debugInfo.lastCommCommand),
             command);
  g_debugInfo.lastCommSuccessMs = millis();
  g_debugInfo.lastCommError[0] = '\0';
  g_debugInfo.lastCommFailureMs = 0;
}

void recordCommFailure(const char* command, const char* errorMessage) {
  copyString(g_debugInfo.lastCommCommand, sizeof(g_debugInfo.lastCommCommand),
             command);
  if (errorMessage) {
    copyString(g_debugInfo.lastCommError, sizeof(g_debugInfo.lastCommError),
               errorMessage);
  } else {
    g_debugInfo.lastCommError[0] = '\0';
  }
  g_debugInfo.lastCommFailureMs = millis();
}

}  // namespace debug

#endif  // defined(DEVICE_ROLE_HID)

