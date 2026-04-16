#include "stellarium_link.h"

#include <WiFi.h>

#include "config.h"
#include "motion.h"
#include "time_utils.h"
#include "wifi_ota.h"

namespace stellarium_link {
namespace {

WiFiServer g_server(config::STELLARIUM_TCP_PORT);
WiFiClient g_client;
bool g_accessPointActive = false;
IPAddress g_accessPointIp;
String g_commandBuffer;

String formatHMS(uint8_t h, uint8_t m, uint8_t s) {
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%02u:%02u:%02u", h, m, s);
  return String(buffer);
}

String formatDMS(int16_t d, uint8_t m, uint8_t s) {
  char sign = d < 0 ? '-' : '+';
  int16_t absD = d < 0 ? -d : d;
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%c%02d*%02u:%02u", sign, absD, m, s);
  return String(buffer);
}

void writeReply(const String& payload) {
  if (!g_client || !g_client.connected()) {
    return;
  }
  g_client.print(payload);
  g_client.print('#');
}

void handleCommand(const String& cmd) {
  if (cmd == ":Q") {
    motion::stopAll();
    writeReply("1");
    return;
  }

  if (cmd == ":GR") {
    time_t utc = time_utils::currentUtcEpoch();
    DateTime now(utc);
    writeReply(formatHMS(now.hour(), now.minute(), now.second()));
    return;
  }

  if (cmd == ":GD") {
    writeReply(formatDMS(0, 0, 0));
    return;
  }

  if (cmd.startsWith(":SC")) {
    // Date command accepted for compatibility.
    writeReply("1");
    return;
  }

  if (cmd.startsWith(":SL")) {
    // Time command accepted for compatibility.
    writeReply("1");
    return;
  }

  writeReply("0");
}

void processClient() {
  while (g_client && g_client.connected() && g_client.available()) {
    char c = static_cast<char>(g_client.read());
    if (c == '#') {
      handleCommand(g_commandBuffer);
      g_commandBuffer = "";
    } else if (isPrintable(static_cast<unsigned char>(c))) {
      if (g_commandBuffer.length() < 96) {
        g_commandBuffer += c;
      }
    }
  }
}

}  // namespace

void init() {
  g_server.begin();
  g_server.setNoDelay(true);
}

void update() {
  if (!g_client || !g_client.connected()) {
    WiFiClient candidate = g_server.available();
    if (candidate) {
      g_client.stop();
      g_client = candidate;
      g_commandBuffer = "";
    }
  }
  processClient();
}

bool enableAccessPoint() {
  if (g_accessPointActive) {
    return true;
  }

  if (wifi_ota::isEnabled()) {
    wifi_ota::setEnabled(false);
  }

  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(config::WIFI_AP_SSID, config::WIFI_AP_PASSWORD,
                        config::WIFI_AP_CHANNEL);
  if (!ok) {
    return false;
  }
  g_accessPointActive = true;
  g_accessPointIp = WiFi.softAPIP();
  return true;
}

void disableAccessPoint() {
  if (!g_accessPointActive) {
    return;
  }
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  g_accessPointActive = false;
  g_accessPointIp = IPAddress();
}

bool accessPointActive() { return g_accessPointActive; }

bool clientConnected() { return g_client && g_client.connected(); }

const char* accessPointSsid() { return config::WIFI_AP_SSID; }

String accessPointIp() {
  if (!g_accessPointActive) {
    return String();
  }
  return g_accessPointIp.toString();
}

void forceDisconnectClient() {
  if (g_client) {
    g_client.stop();
  }
  g_client = WiFiClient();
  g_commandBuffer = "";
}

}  // namespace stellarium_link
