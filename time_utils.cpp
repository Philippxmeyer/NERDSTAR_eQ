#include "time_utils.h"

namespace {

uint32_t g_lastSyncMs = 0;
time_t g_lastSyncEpoch = 0;
bool g_hasValidTime = false;

}  // namespace

namespace time_utils {

void init() {
  g_lastSyncMs = millis();
  g_lastSyncEpoch = 0;
  g_hasValidTime = false;
}

bool hasValidTime() { return g_hasValidTime; }

time_t currentUtcEpoch() {
  if (!g_hasValidTime) {
    return 0;
  }
  // millis() rolls over after ~49 days; the unsigned subtraction handles the
  // wrap correctly as long as we never go more than ~49 days without a sync.
  uint32_t elapsedMs = millis() - g_lastSyncMs;
  return g_lastSyncEpoch + static_cast<time_t>(elapsedMs / 1000);
}

void setUtcEpoch(time_t utcEpoch) {
  g_lastSyncEpoch = utcEpoch;
  g_lastSyncMs = millis();
  g_hasValidTime = true;
}

uint32_t secondsSinceLastSync() {
  if (!g_hasValidTime) {
    return 0;
  }
  return (millis() - g_lastSyncMs) / 1000;
}

}  // namespace time_utils
