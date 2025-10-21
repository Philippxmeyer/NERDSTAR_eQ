#pragma once

#include <Arduino.h>

#include "role_config.h"

namespace debug {

#if defined(DEVICE_ROLE_HID)
void init();
void printStartupSummary();
void recordEvent(const char* label);
void recordEvent(const char* label, uint32_t timestampMs);
void recordLoop(uint32_t timestampMs);
void recordCommAttempt(const char* command);
void recordCommSuccess(const char* command);
void recordCommFailure(const char* command, const char* errorMessage);
#else
inline void init() {}
inline void printStartupSummary() {}
inline void recordEvent(const char*, uint32_t = 0) {}
inline void recordLoop(uint32_t) {}
inline void recordCommAttempt(const char*) {}
inline void recordCommSuccess(const char*) {}
inline void recordCommFailure(const char*, const char*) {}
#endif

}  // namespace debug

