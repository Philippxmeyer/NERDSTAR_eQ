#pragma once

#include <Arduino.h>

namespace stellarium_link {

void init();
void update();
bool enableAccessPoint();
void disableAccessPoint();
bool accessPointActive();
bool clientConnected();
const char* accessPointSsid();
String accessPointIp();
void forceDisconnectClient();

}  // namespace stellarium_link
