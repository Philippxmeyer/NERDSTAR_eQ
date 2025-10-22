#pragma once

// Device role selection.
// By default the firmware builds the HID/controller variant. To build the main
// controller firmware define DEVICE_ROLE_MAIN (for example via compiler
// options). Exactly one role must be selected.
#ifndef DEVICE_ROLE_MAIN
#ifndef DEVICE_ROLE_HID
 #define DEVICE_ROLE_HID
// #define DEVICE_ROLE_MAIN
#endif
#endif

#if defined(DEVICE_ROLE_MAIN) && defined(DEVICE_ROLE_HID)
#error "Both DEVICE_ROLE_MAIN and DEVICE_ROLE_HID defined. Select exactly one role."
#endif

#if !defined(DEVICE_ROLE_MAIN) && !defined(DEVICE_ROLE_HID)
#error "No device role selected. Define DEVICE_ROLE_MAIN or DEVICE_ROLE_HID."
#endif

