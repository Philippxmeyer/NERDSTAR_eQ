#pragma once

// Single-device firmware build.
// The project now builds only the main controller role.
#ifndef DEVICE_ROLE_MAIN
#define DEVICE_ROLE_MAIN
#endif

#ifdef DEVICE_ROLE_HID
#error "DEVICE_ROLE_HID is no longer supported. Use DEVICE_ROLE_MAIN only."
#endif
