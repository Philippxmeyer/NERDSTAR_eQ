#pragma once

#include <Arduino.h>

// LX200 command dispatcher over the USB serial port.
//
// Stellarmate / INDI connects directly to the ESP32 via USB CDC and speaks
// the Meade LX200 protocol.  This module is responsible for:
//   * parsing inbound commands,
//   * driving the motion layer for manual / goto slews,
//   * persisting site location (:Sg / :St / :SG) and
//   * seeding the software clock from host date + time (:SC / :SL); the
//     firmware no longer carries a hardware RTC, so the host is expected
//     to push a fresh time periodically.
namespace lx200_link {

void init();
void update();

}  // namespace lx200_link
