#pragma once

#include "role_config.h"

#if defined(DEVICE_ROLE_HID)

#include <Arduino.h>

namespace stellarium_link {

void init();
void update();
bool enableAccessPoint();
void disableAccessPoint();
bool accessPointActive();
bool clientConnected();
const char* accessPointSsid();
void forceDisconnectClient();

}  // namespace stellarium_link

#endif  // DEVICE_ROLE_HID
