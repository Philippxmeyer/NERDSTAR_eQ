#include "stellarium_link.h"

#if defined(DEVICE_ROLE_HID)

#include <WiFi.h>
#include <algorithm>
#include <math.h>
#include <utility>

#include "config.h"
#include "display_menu.h"
#include "motion.h"
#include "storage.h"
#include "time_utils.h"
#include "wifi_ota.h"

namespace stellarium_link {
namespace {

WiFiServer g_server(config::STELLARIUM_TCP_PORT);
WiFiClient g_client;
bool g_accessPointActive = false;
IPAddress g_accessPointIp;
String g_commandBuffer;
bool g_clientConnected = false;
double g_pendingRaHours = 0.0;
double g_pendingDecDegrees = 0.0;
bool g_pendingRaValid = false;
bool g_pendingDecValid = false;
uint32_t g_lastClientActivityMs = 0;
constexpr uint32_t kClientIdleTimeoutMs = 5UL * 60UL * 1000UL;

void resetPendingTarget() {
  g_pendingRaValid = false;
  g_pendingDecValid = false;
  g_pendingRaHours = 0.0;
  g_pendingDecDegrees = 0.0;
}

void persistObserverLocation(double latitudeDeg, double longitudeDeg,
                             int32_t timezoneMinutes) {
  storage::setObserverLocation(latitudeDeg, longitudeDeg, timezoneMinutes);
}

void persistNetworkTime(const DateTime& localTime) {
  time_t utcEpoch = time_utils::toUtcEpoch(localTime);
  display_menu::applyNetworkTime(utcEpoch);
}

void clearClientState(bool notify) {
  if (g_client) {
    g_client.stop();
  }
  g_client = WiFiClient();
  bool wasConnected = g_clientConnected;
  g_clientConnected = false;
  g_commandBuffer = "";
  resetPendingTarget();
  g_lastClientActivityMs = 0;
  display_menu::setStellariumStatus(false, 0.0, 0.0);
  if (notify && wasConnected) {
    display_menu::showInfo("Stellarium getrennt", 2000);
  }
}

String formatRa(double hours) {
  double normalized = fmod(hours, 24.0);
  if (normalized < 0.0) normalized += 24.0;
  int h = static_cast<int>(normalized);
  double minutesFloat = (normalized - h) * 60.0;
  int m = static_cast<int>(minutesFloat);
  int s = static_cast<int>((minutesFloat - m) * 60.0 + 0.5);
  if (s >= 60) {
    s -= 60;
    m += 1;
  }
  if (m >= 60) {
    m -= 60;
    h = (h + 1) % 24;
  }
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d", h, m, s);
  return String(buffer);
}

String formatDec(double degrees) {
  char sign = degrees >= 0.0 ? '+' : '-';
  double absVal = fabs(degrees);
  if (absVal > 90.0) absVal = 90.0;
  int d = static_cast<int>(absVal);
  double minutesFloat = (absVal - d) * 60.0;
  int m = static_cast<int>(minutesFloat);
  int s = static_cast<int>((minutesFloat - m) * 60.0 + 0.5);
  if (s >= 60) {
    s -= 60;
    m += 1;
  }
  if (m >= 60) {
    m -= 60;
    d += 1;
  }
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%c%02d*%02d:%02d", sign, d, m, s);
  return String(buffer);
}

String formatLatitude(double degrees) {
  double clamped = std::clamp(degrees, -90.0, 90.0);
  char sign = clamped >= 0.0 ? '+' : '-';
  double absVal = fabs(clamped);
  int d = static_cast<int>(absVal);
  double minutesFloat = (absVal - d) * 60.0;
  int m = static_cast<int>(minutesFloat + 0.5);
  if (m >= 60) {
    m = 0;
    d = std::min(d + 1, 90);
  }
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%c%02d*%02d", sign, d, m);
  return String(buffer);
}

String formatLongitude(double degrees) {
  double clamped = std::clamp(degrees, -180.0, 180.0);
  char sign = clamped >= 0.0 ? '+' : '-';
  double absVal = fabs(clamped);
  int d = static_cast<int>(absVal);
  double minutesFloat = (absVal - d) * 60.0;
  int m = static_cast<int>(minutesFloat + 0.5);
  if (m >= 60) {
    m = 0;
    d = std::min(d + 1, 180);
  }
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%c%03d*%02d", sign, d, m);
  return String(buffer);
}

bool parseRaCommand(const String& payload, double& hours) {
  int firstColon = payload.indexOf(':');
  int secondColon = payload.indexOf(':', firstColon + 1);
  if (firstColon < 0) {
    return false;
  }
  int h = payload.substring(0, firstColon).toInt();
  int m = 0;
  int s = 0;
  if (secondColon < 0) {
    m = payload.substring(firstColon + 1).toInt();
  } else {
    m = payload.substring(firstColon + 1, secondColon).toInt();
    s = payload.substring(secondColon + 1).toInt();
  }
  if (h < 0 || h > 23 || m < 0 || m > 59 || s < 0 || s > 59) {
    return false;
  }
  hours = h + m / 60.0 + s / 3600.0;
  return true;
}

bool parseDecCommand(const String& payload, double& degrees) {
  if (payload.length() < 4) {
    return false;
  }
  char signChar = payload[0];
  bool negative = false;
  int start = 0;
  if (signChar == '+' || signChar == '-') {
    negative = signChar == '-';
    start = 1;
  }
  int starIndex = payload.indexOf('*', start);
  int colonIndex = payload.indexOf(':', starIndex + 1);
  if (starIndex < 0) {
    return false;
  }
  int d = payload.substring(start, starIndex).toInt();
  int m = 0;
  int s = 0;
  if (colonIndex < 0) {
    m = payload.substring(starIndex + 1).toInt();
  } else {
    m = payload.substring(starIndex + 1, colonIndex).toInt();
    s = payload.substring(colonIndex + 1).toInt();
  }
  if (d < 0 || d > 90 || m < 0 || m > 59 || s < 0 || s > 59) {
    return false;
  }
  double value = d + m / 60.0 + s / 3600.0;
  degrees = negative ? -value : value;
  return true;
}

bool parseLatitudeCommand(const String& payload, double& degrees) {
  if (payload.length() < 4) {
    return false;
  }
  char signChar = payload[0];
  bool negative = false;
  int start = 0;
  if (signChar == '+' || signChar == '-') {
    negative = signChar == '-';
    start = 1;
  }
  int starIndex = payload.indexOf('*', start);
  if (starIndex < 0) {
    return false;
  }
  int colonIndex = payload.indexOf(':', starIndex + 1);
  int d = payload.substring(start, starIndex).toInt();
  int m = 0;
  int s = 0;
  if (colonIndex < 0) {
    m = payload.substring(starIndex + 1).toInt();
  } else {
    m = payload.substring(starIndex + 1, colonIndex).toInt();
    s = payload.substring(colonIndex + 1).toInt();
  }
  if (d < 0 || d > 90 || m < 0 || m > 59 || s < 0 || s > 59) {
    return false;
  }
  double value = d + m / 60.0 + s / 3600.0;
  degrees = negative ? -value : value;
  return true;
}

bool parseLongitudeCommand(const String& payload, double& degrees) {
  if (payload.length() < 5) {
    return false;
  }
  char signChar = payload[0];
  bool negative = false;
  int start = 0;
  if (signChar == '+' || signChar == '-') {
    negative = signChar == '-';
    start = 1;
  }
  int starIndex = payload.indexOf('*', start);
  if (starIndex < 0) {
    return false;
  }
  int colonIndex = payload.indexOf(':', starIndex + 1);
  int d = payload.substring(start, starIndex).toInt();
  int m = 0;
  int s = 0;
  if (colonIndex < 0) {
    m = payload.substring(starIndex + 1).toInt();
  } else {
    m = payload.substring(starIndex + 1, colonIndex).toInt();
    s = payload.substring(colonIndex + 1).toInt();
  }
  if (d < 0 || d > 180 || m < 0 || m > 59 || s < 0 || s > 59) {
    return false;
  }
  double value = d + m / 60.0 + s / 3600.0;
  degrees = negative ? -value : value;
  return true;
}

DateTime currentLocalTime() {
  const SystemConfig& config = storage::getConfig();
  if (config.lastRtcEpoch != 0) {
    return time_utils::applyTimezone(static_cast<time_t>(config.lastRtcEpoch));
  }
  return DateTime(2024, 1, 1, 0, 0, 0);
}

String formatTime(const DateTime& local) {
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d", local.hour(), local.minute(), local.second());
  return String(buffer);
}

String formatDate(const DateTime& local) {
  char buffer[16];
  int twoDigitYear = local.year() % 100;
  snprintf(buffer, sizeof(buffer), "%02d/%02d/%02d", local.month(), local.day(), twoDigitYear);
  return String(buffer);
}

String handleCommand(const String& command) {
  String normalized = command;
  normalized.trim();
  normalized.toUpperCase();
  if (normalized.length() < 2 || normalized[0] != ':') {
    return "";
  }
  if (normalized.startsWith(":GR")) {
    double ra = 0.0;
    double dec = 0.0;
    if (!display_menu::computeCurrentEquatorial(ra, dec)) {
      ra = 0.0;
    }
    return formatRa(ra);
  }
  if (normalized.startsWith(":GD")) {
    double ra = 0.0;
    double dec = 0.0;
    if (!display_menu::computeCurrentEquatorial(ra, dec)) {
      dec = 0.0;
    }
    return formatDec(dec);
  }
  if (normalized.startsWith(":SR")) {
    String payload = normalized.substring(3);
    double ra = 0.0;
    if (!parseRaCommand(payload, ra)) {
      g_pendingRaValid = false;
      return "0";
    }
    g_pendingRaHours = ra;
    g_pendingRaValid = true;
    return "1";
  }
  if (normalized.startsWith(":SD")) {
    String payload = normalized.substring(3);
    double dec = 0.0;
    if (!parseDecCommand(payload, dec)) {
      g_pendingDecValid = false;
      return "0";
    }
    g_pendingDecDegrees = dec;
    g_pendingDecValid = true;
    return "1";
  }
  if (normalized.startsWith(":MS")) {
    if (!g_pendingRaValid || !g_pendingDecValid) {
      return "1";
    }
    bool ok = display_menu::requestGotoFromNetwork(g_pendingRaHours, g_pendingDecDegrees,
                                                   "Stellarium");
    if (ok) {
      resetPendingTarget();
    }
    return ok ? "0" : "1";
  }
  if (normalized.startsWith(":Q")) {
    display_menu::abortGotoFromNetwork();
    display_menu::stopTracking();
    motion::stopAll();
    return "";
  }
  if (normalized.startsWith(":GVP")) {
    return String("NERDSTAR");
  }
  if (normalized.startsWith(":GVN")) {
    return String("1.0");
  }
  if (normalized.startsWith(":GVD")) {
    return String("2024-01-01");
  }
  if (normalized.startsWith(":GG")) {
    int32_t tzMinutes = storage::getConfig().timezoneOffsetMinutes;
    int tzHours = static_cast<int>(round(tzMinutes / 60.0));
    char buffer[8];
    snprintf(buffer, sizeof(buffer), "%+03d", tzHours);
    return String(buffer);
  }
  if (normalized.startsWith(":GL")) {
    return formatTime(currentLocalTime());
  }
  if (normalized.startsWith(":GC")) {
    return formatDate(currentLocalTime());
  }
  if (normalized.startsWith(":Gg")) {
    return formatLongitude(storage::getConfig().observerLongitudeDeg);
  }
  if (normalized.startsWith(":Gt")) {
    return formatLatitude(storage::getConfig().observerLatitudeDeg);
  }
  if (normalized.startsWith(":D")) {
    return String("0");
  }
  if (normalized.startsWith(":SG")) {
    String payload = normalized.substring(3);
    double hours = payload.toFloat();
    if (!isfinite(hours) || fabs(hours) > 14.0) {
      return "0";
    }
    int32_t minutes = static_cast<int32_t>(round(hours * 60.0));
    minutes = std::clamp<int32_t>(minutes, -720, 840);
    const auto& config = storage::getConfig();
    persistObserverLocation(config.observerLatitudeDeg, config.observerLongitudeDeg, minutes);
    return "1";
  }
  if (normalized.startsWith(":SL")) {
    String payload = normalized.substring(3);
    int firstColon = payload.indexOf(':');
    int secondColon = payload.indexOf(':', firstColon + 1);
    if (firstColon < 0 || secondColon < 0) {
      return "0";
    }
    int h = payload.substring(0, firstColon).toInt();
    int m = payload.substring(firstColon + 1, secondColon).toInt();
    int s = payload.substring(secondColon + 1).toInt();
    if (h < 0 || h > 23 || m < 0 || m > 59 || s < 0 || s > 59) {
      return "0";
    }
    DateTime local = currentLocalTime();
    DateTime updated(local.year(), local.month(), local.day(), h, m, s);
    persistNetworkTime(updated);
    return "1";
  }
  if (normalized.startsWith(":SC")) {
    String payload = normalized.substring(3);
    int firstSlash = payload.indexOf('/');
    int secondSlash = payload.indexOf('/', firstSlash + 1);
    if (firstSlash < 0 || secondSlash < 0) {
      return "0";
    }
    int month = payload.substring(0, firstSlash).toInt();
    int day = payload.substring(firstSlash + 1, secondSlash).toInt();
    int year = payload.substring(secondSlash + 1).toInt();
    if (month < 1 || month > 12 || day < 1 || day > 31) {
      return "0";
    }
    int fullYear = 2000 + (year % 100);
    DateTime local = currentLocalTime();
    DateTime updated(fullYear, month, day, local.hour(), local.minute(), local.second());
    persistNetworkTime(updated);
    return "1";
  }
  if (normalized.startsWith(":Sg")) {
    String payload = normalized.substring(3);
    double longitude = 0.0;
    if (!parseLongitudeCommand(payload, longitude)) {
      return "0";
    }
    const auto& config = storage::getConfig();
    persistObserverLocation(config.observerLatitudeDeg, longitude, config.timezoneOffsetMinutes);
    return "1";
  }
  if (normalized.startsWith(":St")) {
    String payload = normalized.substring(3);
    double latitude = 0.0;
    if (!parseLatitudeCommand(payload, latitude)) {
      return "0";
    }
    const auto& config = storage::getConfig();
    persistObserverLocation(latitude, config.observerLongitudeDeg, config.timezoneOffsetMinutes);
    return "1";
  }
  return "";
}

void sendResponse(const String& response) {
  if (!g_client || response.isEmpty()) {
    return;
  }
  g_client.print(response);
  g_client.print('#');
}

void handleClientInput() {
  constexpr size_t kMaxBytesPerUpdate = 128;
  constexpr uint32_t kMaxHandleDurationMs = 10;
  uint32_t startMs = millis();
  size_t processed = 0;

  while (g_client && g_client.connected() && g_client.available() > 0) {
    int raw = g_client.read();
    if (raw < 0) {
      break;
    }
    g_lastClientActivityMs = millis();

    // LX200 handshake / tracking query (used by Stellarium Mobile)
    if (raw == 0x06) {
      // 'L' = tracking off, 'P' = tracking on. Report tracking enabled.
      g_client.write('P');
      continue;
    }

    char c = static_cast<char>(raw);
    if (c == '\r' || c == '\n') {
      continue;
    }
    if (c == ':') {
      g_commandBuffer = ":";
      continue;
    }
    if (g_commandBuffer.isEmpty()) {
      continue;
    }
    if (c == '#') {
      String command = g_commandBuffer;
      g_commandBuffer = "";
      String response = handleCommand(command);
      sendResponse(response);
    } else {
      if (g_commandBuffer.length() >= 64) {
        g_commandBuffer = "";
      } else {
        g_commandBuffer += c;
      }
    }

    ++processed;
    if (processed >= kMaxBytesPerUpdate ||
        (millis() - startMs) >= kMaxHandleDurationMs) {
      break;
    }
  }
}

void acceptNewClient() {
  if (!g_server.hasClient()) {
    return;
  }
  WiFiClient candidate = g_server.available();
  if (!candidate) {
    return;
  }
  if (g_clientConnected) {
    clearClientState(false);
  }
  g_client = std::move(candidate);
  g_client.setNoDelay(true);
  g_clientConnected = true;
  g_commandBuffer = "";
  resetPendingTarget();
  g_lastClientActivityMs = millis();
  display_menu::showInfo("Stellarium verbunden", 2000);
}

void updateDisplayStatus() {
  if (!g_clientConnected) {
    display_menu::setStellariumStatus(false, 0.0, 0.0);
    return;
  }
  double ra = 0.0;
  double dec = 0.0;
  if (!display_menu::computeCurrentEquatorial(ra, dec)) {
    ra = 0.0;
    dec = 0.0;
  }
  display_menu::setStellariumStatus(true, ra, dec);
}

}  // namespace

void init() {
  g_server.setNoDelay(true);
  g_accessPointActive = false;
  g_clientConnected = false;
  g_commandBuffer = "";
  resetPendingTarget();
  display_menu::setStellariumStatus(false, 0.0, 0.0);
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true, true);
  g_accessPointIp = IPAddress();
}

bool enableAccessPoint() {
  if (g_accessPointActive) {
    return true;
  }
  wifi_ota::setEnabled(false);
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_MODE_AP);
  bool ok = WiFi.softAP(config::WIFI_AP_SSID, config::WIFI_AP_PASSWORD, config::WIFI_AP_CHANNEL,
                        false, 1);
  if (!ok) {
    WiFi.mode(WIFI_OFF);
    g_accessPointIp = IPAddress();
    return false;
  }
  g_accessPointIp = WiFi.softAPIP();
  g_server.begin();
  g_accessPointActive = true;
  return true;
}

void disableAccessPoint() {
  if (!g_accessPointActive) {
    clearClientState(false);
    return;
  }
  clearClientState(false);
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  g_accessPointActive = false;
  g_accessPointIp = IPAddress();
}

bool accessPointActive() { return g_accessPointActive; }

bool clientConnected() { return g_clientConnected; }

const char* accessPointSsid() { return config::WIFI_AP_SSID; }

String accessPointIp() {
  if (!g_accessPointActive) {
    return String();
  }
  return g_accessPointIp.toString();
}

void forceDisconnectClient() { clearClientState(true); }

void update() {
  if (!g_accessPointActive) {
    if (g_clientConnected) {
      clearClientState(false);
    }
    return;
  }

  if (g_clientConnected && (!g_client || !g_client.connected())) {
    clearClientState(true);
    return;
  }

  acceptNewClient();
  if (g_clientConnected) {
    handleClientInput();
    if (kClientIdleTimeoutMs > 0 && g_lastClientActivityMs != 0) {
      uint32_t now = millis();
      if (now - g_lastClientActivityMs >= kClientIdleTimeoutMs) {
        clearClientState(true);
        return;
      }
    }
  }
  updateDisplayStatus();
}

}  // namespace stellarium_link

#endif  // DEVICE_ROLE_HID
