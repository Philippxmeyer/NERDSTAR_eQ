#pragma once

#include <Arduino.h>
#include <time.h>

// Software clock for the ESP32. The mount has no battery-backed RTC anymore;
// the host (Stellarmate / SmartScope) feeds an absolute UTC reference through
// the LX200 channel (`:SC` + `:SL`) and the firmware advances it locally with
// `millis()` until the host pushes a fresh sync.
namespace time_utils {

void init();

// Has a host-supplied absolute time ever been received?
bool hasValidTime();

// Current UTC epoch derived from the last sync plus elapsed millis().
// Returns 0 until the first :SC/:SL pair arrives.
time_t currentUtcEpoch();

// Anchor the software clock to an absolute UTC epoch.
void setUtcEpoch(time_t utcEpoch);

// Seconds since the last setUtcEpoch() call. 0 if no sync yet.
uint32_t secondsSinceLastSync();

}  // namespace time_utils
