#pragma once

#include <Arduino.h>
#include <RTClib.h>
#include <time.h>

namespace time_utils {

bool initRtc();
bool rtcAvailable();
time_t currentUtcEpoch();
bool setUtcEpoch(time_t utcEpoch);
DateTime applyTimezone(time_t utcEpoch, int32_t timezoneMinutes);

}  // namespace time_utils
