#include "stellarium_link.h"

#if defined(DEVICE_ROLE_HID)

#include <WiFi.h>
#include <math.h>
#include <utility>

#include "config.h"
#include "display_menu.h"
#include "motion.h"
#include "wifi_ota.h"

namespace stellarium_link {
namespace {

WiFiServer g_server(config::STELLARIUM_TCP_PORT);
WiFiClient g_client;
bool g_accessPointActive = false;
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

bool parseRaCommand(const String& payload, double& hours) {
  int firstColon = payload.indexOf(':');
  int secondColon = payload.indexOf(':', firstColon + 1);
  if (firstColon < 0 || secondColon < 0) {
    return false;
  }
  int h = payload.substring(0, firstColon).toInt();
  int m = payload.substring(firstColon + 1, secondColon).toInt();
  int s = payload.substring(secondColon + 1).toInt();
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
  if (starIndex < 0 || colonIndex < 0) {
    return false;
  }
  int d = payload.substring(start, starIndex).toInt();
  int m = payload.substring(starIndex + 1, colonIndex).toInt();
  int s = payload.substring(colonIndex + 1).toInt();
  if (d < 0 || d > 90 || m < 0 || m > 59 || s < 0 || s > 59) {
    return false;
  }
  double value = d + m / 60.0 + s / 3600.0;
  degrees = negative ? -value : value;
  return true;
}

String handleCommand(const String& command) {
  if (command.length() < 2 || command[0] != ':') {
    return "";
  }
  if (command.startsWith(":GR")) {
    double ra = 0.0;
    double dec = 0.0;
    if (!display_menu::computeCurrentEquatorial(ra, dec)) {
      ra = 0.0;
    }
    return formatRa(ra);
  }
  if (command.startsWith(":GD")) {
    double ra = 0.0;
    double dec = 0.0;
    if (!display_menu::computeCurrentEquatorial(ra, dec)) {
      dec = 0.0;
    }
    return formatDec(dec);
  }
  if (command.startsWith(":Sr")) {
    String payload = command.substring(3);
    double ra = 0.0;
    if (!parseRaCommand(payload, ra)) {
      g_pendingRaValid = false;
      return "0";
    }
    g_pendingRaHours = ra;
    g_pendingRaValid = true;
    return "1";
  }
  if (command.startsWith(":Sd")) {
    String payload = command.substring(3);
    double dec = 0.0;
    if (!parseDecCommand(payload, dec)) {
      g_pendingDecValid = false;
      return "0";
    }
    g_pendingDecDegrees = dec;
    g_pendingDecValid = true;
    return "1";
  }
  if (command.startsWith(":MS")) {
    if (!g_pendingRaValid || !g_pendingDecValid) {
      return "0";
    }
    bool ok = display_menu::requestGotoFromNetwork(g_pendingRaHours, g_pendingDecDegrees,
                                                   "Stellarium");
    if (ok) {
      resetPendingTarget();
    }
    return ok ? "1" : "0";
  }
  if (command.startsWith(":Q")) {
    display_menu::abortGotoFromNetwork();
    display_menu::stopTracking();
    motion::stopAll();
    return "1";
  }
  if (command.startsWith(":GVP")) {
    return String("NERDSTAR");
  }
  if (command.startsWith(":GVN")) {
    return String("1.0");
  }
  return "";
}

void sendResponse(const String& response) {
  if (!g_client) {
    return;
  }
  g_client.print(response);
  g_client.print('#');
}

void handleClientInput() {
  while (g_client && g_client.connected() && g_client.available() > 0) {
    int raw = g_client.read();
    if (raw < 0) {
      break;
    }
    g_lastClientActivityMs = millis();
    char c = static_cast<char>(raw);
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
    return false;
  }
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
}

bool accessPointActive() { return g_accessPointActive; }

bool clientConnected() { return g_clientConnected; }

const char* accessPointSsid() { return config::WIFI_AP_SSID; }

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
