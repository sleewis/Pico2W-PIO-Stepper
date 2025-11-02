#pragma once
#include <Arduino.h>

// Call after Serial.begin(...):
//   bootBanner::print_boot_banner(true, "v0.3.0");
//
// Notes:
//  - ANSI colors may not render in Arduino IDE Serial Monitor.
//  - Pass useAnsi=false to print a plain monochrome banner.

namespace bootBanner {

#ifndef USE_ANSI_COLOR_DEFAULT
#define USE_ANSI_COLOR_DEFAULT 1
#endif

void print_boot_banner(bool useAnsi = (USE_ANSI_COLOR_DEFAULT != 0),
                       const char* fwVersion = "v0.0.0");

} // namespace BootBanner
