#include "time_utils.h"

#include <Wire.h>

#include "config.h"

namespace {
RTC_DS3231 g_rtc;
bool g_rtcAvailable = false;
uint32_t g_fallbackStartMs = 0;
time_t g_fallbackStartEpoch = 0;

}  // namespace

namespace time_utils {

bool initRtc() {
  Wire.begin(config::RTC_SDA_PIN, config::RTC_SCL_PIN);
  g_rtcAvailable = g_rtc.begin();
  g_fallbackStartMs = millis();
  g_fallbackStartEpoch = 0;

  if (g_rtcAvailable && g_rtc.lostPower()) {
    g_rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  return g_rtcAvailable;
}

bool rtcAvailable() { return g_rtcAvailable; }

time_t currentUtcEpoch() {
  if (g_rtcAvailable) {
    return g_rtc.now().unixtime();
  }

  uint32_t elapsedSeconds = (millis() - g_fallbackStartMs) / 1000;
  return g_fallbackStartEpoch + elapsedSeconds;
}

bool setUtcEpoch(time_t utcEpoch) {
  if (g_rtcAvailable) {
    g_rtc.adjust(DateTime(utcEpoch));
    return true;
  }
  g_fallbackStartEpoch = utcEpoch;
  g_fallbackStartMs = millis();
  return false;
}

DateTime applyTimezone(time_t utcEpoch, int32_t timezoneMinutes) {
  time_t localEpoch = utcEpoch + static_cast<time_t>(timezoneMinutes) * 60;
  return DateTime(localEpoch);
}

}  // namespace time_utils
