#pragma once
#include <Arduino.h>
#include "SerialDevice.h"

enum Axis { AXIS_X=0, AXIS_Y=1, AXIS_Z=2 };

enum UnitMode { UNITS_STEPS = 0, UNITS_MM = 1, UNITS_DEG = 2 };

struct LineMove {
  int32_t  sx, sy, sz;
  uint32_t d_start_us, d_cruise_us, d_end_us;
  float    accel_frac, decel_frac;
  LineMove() : sx(0), sy(0), sz(0), d_start_us(2000), d_cruise_us(1000),
               d_end_us(2000), accel_frac(0.35f), decel_frac(0.35f) {}
};

struct UnitLimit { bool enabled; float min_v; float max_v; };

struct AxisLimits {
  UnitLimit steps;  // voor UNITS_STEPS
  UnitLimit mm;     // voor UNITS_MM
  UnitLimit deg;    // voor UNITS_DEG
};
