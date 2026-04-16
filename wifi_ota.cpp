#include "wifi_ota.h"

#include <ESPmDNS.h>
#include <Hash.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <time.h>

#include "config.h"
#include "time_utils.h"

namespace wifi_ota {
namespace {

bool wifiEnabled = false;
bool wifiConnected = false;
bool otaActive = false;
bool ntpSynced = false;
String otaHostname;

enum class NtpStatus { None, Success, Failure };
NtpStatus lastReportedNtpStatus = NtpStatus::None;

constexpr uint32_t kReconnectIntervalMs = 10000;
constexpr uint32_t kNtpResyncIntervalMs = 6UL * 60UL * 60UL * 1000UL;
constexpr uint32_t kNtpRetryIntervalMs = 60000;

uint32_t lastReconnectAttemptMs = 0;
uint32_t lastNtpSyncMs = 0;
uint32_t lastNtpAttemptMs = 0;

const char* roleSuffix() { return "MAIN"; }

void ensureIdentity() {
  if (otaHostname.isEmpty()) {
    otaHostname = String(config::WIFI_HOSTNAME_PREFIX) + "-" + roleSuffix();
  }
}

void stopWifi() {
  if (otaActive) {
    ArduinoOTA.end();
    otaActive = false;
  }
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  wifiEnabled = false;
  wifiConnected = false;
  ntpSynced = false;
  lastReportedNtpStatus = NtpStatus::None;
  lastReconnectAttemptMs = 0;
  lastNtpSyncMs = 0;
  lastNtpAttemptMs = 0;
}

bool credentialsAvailable() {
  return config::WIFI_STA_SSID && config::WIFI_STA_SSID[0] != '\0';
}

void beginWifi() {
  ensureIdentity();
  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect(true, true);
  WiFi.setHostname(otaHostname.c_str());
  WiFi.setAutoReconnect(true);
  ArduinoOTA.setHostname(otaHostname.c_str());
  WiFi.begin(config::WIFI_STA_SSID, config::WIFI_STA_PASSWORD);
  wifiEnabled = true;
  wifiConnected = false;
  otaActive = false;
  ntpSynced = false;
  lastReportedNtpStatus = NtpStatus::None;
  lastReconnectAttemptMs = millis();
  lastNtpSyncMs = 0;
  lastNtpAttemptMs = 0;
}

bool syncTimeWithNtp() {
  static const char* servers[] = {"pool.ntp.org", "time.nist.gov", "time.cloudflare.com"};
  configTime(0, 0, servers[0], servers[1], servers[2]);
  struct tm timeinfo{};
  if (!getLocalTime(&timeinfo, 10000)) {
    return false;
  }
  time_t utcEpoch = mktime(&timeinfo);
  time_utils::setUtcEpoch(utcEpoch);
  return true;
}

void notifyNtpStatus(bool success) {
  (void)success;
}

void handleConnectedState() {
  uint32_t now = millis();
  if (!wifiConnected) {
    wifiConnected = true;
    if (!otaActive) {
      ArduinoOTA.begin();
      otaActive = true;
    }
    ntpSynced = false;
    lastNtpSyncMs = 0;
    bool success = syncTimeWithNtp();
    uint32_t attemptTime = millis();
    ntpSynced = success;
    lastNtpAttemptMs = attemptTime;
    if (success) {
      lastNtpSyncMs = attemptTime;
    }
    notifyNtpStatus(success);
    return;
  }

  if (otaActive) {
    ArduinoOTA.handle();
  }

  bool shouldSync = false;
  if (!ntpSynced) {
    if (now - lastNtpAttemptMs >= kNtpRetryIntervalMs) {
      shouldSync = true;
    }
  } else if (now - lastNtpSyncMs >= kNtpResyncIntervalMs &&
             now - lastNtpAttemptMs >= kNtpRetryIntervalMs) {
    shouldSync = true;
  }

  if (shouldSync) {
    bool success = syncTimeWithNtp();
    uint32_t attemptTime = millis();
    ntpSynced = success;
    if (success) {
      lastNtpSyncMs = attemptTime;
    }
    lastNtpAttemptMs = attemptTime;
    notifyNtpStatus(success);
  }
}

void handleDisconnectedState() {
  if (wifiConnected || otaActive) {
    if (otaActive) {
      ArduinoOTA.end();
      otaActive = false;
    }
    wifiConnected = false;
    lastReportedNtpStatus = NtpStatus::None;
  }
  uint32_t now = millis();
  if (wifiEnabled && credentialsAvailable() &&
      (lastReconnectAttemptMs == 0 || now - lastReconnectAttemptMs >= kReconnectIntervalMs)) {
    WiFi.disconnect(true, true);
    WiFi.begin(config::WIFI_STA_SSID, config::WIFI_STA_PASSWORD);
    lastReconnectAttemptMs = now;
  }
}

}  // namespace

void init() {
  ensureIdentity();
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true, true);
  wifiEnabled = false;
  wifiConnected = false;
  otaActive = false;
  ntpSynced = false;
  lastReportedNtpStatus = NtpStatus::None;
  lastReconnectAttemptMs = 0;
  lastNtpSyncMs = 0;
  lastNtpAttemptMs = 0;
}

void setEnabled(bool enabled) {
  if (!enabled) {
    if (wifiEnabled || wifiConnected) {
      stopWifi();
    }
    return;
  }

  if (wifiEnabled) {
    return;
  }

  if (!credentialsAvailable()) {
    wifiEnabled = false;
    return;
  }

  beginWifi();
}

bool isEnabled() { return wifiEnabled; }

const char* hostname() { return otaHostname.c_str(); }

bool credentialsConfigured() { return credentialsAvailable(); }

bool isConnected() { return wifiConnected && WiFi.status() == WL_CONNECTED; }

const char* ssid() { return config::WIFI_STA_SSID; }

void update() {
  if (!wifiEnabled) {
    return;
  }

  wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    handleConnectedState();
  } else {
    handleDisconnectedState();
  }
}

}  // namespace wifi_ota
