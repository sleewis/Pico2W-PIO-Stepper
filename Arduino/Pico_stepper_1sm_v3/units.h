#pragma once
#include <Arduino.h>
#include "types.h"

extern volatile UnitMode g_units;

// Conversies steps <-> units
float steps_to_units_X(int32_t s);
float steps_to_units_Y(int32_t s);
float steps_to_units_Z(int32_t s);

int32_t units_to_steps_X(float u);
int32_t units_to_steps_Y(float u);
int32_t units_to_steps_Z(float u);

// Feed helper: steps per unit voor F in huidige units
float steps_per_unit_X();
float steps_per_unit_Y();
float steps_per_unit_Z();

// Caps per as/unit
float apply_caps_X(float v_units);
float apply_caps_Y(float v_units);
float apply_caps_Z(float v_units);

// UI helpers
const char* units_label();
String toStringUnits(float v);
void serial_print_units(float v);
