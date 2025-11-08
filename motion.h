#pragma once
#include <Arduino.h>
#include "types.h"
#include <math.h>

// ===== Motion profile selector (runtime) =====
enum MotionProfile : uint8_t {
  PROF_TRAPEZOID_PERIOD = 0,   // linear ramp of period (simple trapezoid)
  PROF_CONSTACC_SPEED   = 1,   // true constant acceleration in speed (steps/s^2)
  PROF_S_CURVE          = 2    // quintic smoothstep
};

void pio_stepper_init();
void init_dma();
void start_next_move();

void stop_motion_soft();
void stop_motion_force();
void emergency_stop();  // alleen declaratie

void update_realtime_positions_from_dma();

extern "C" void __isr dma_complete_handler();

// Zodat andere files geen TrapS hoeven te kennen
uint32_t planner_total_steps_generated();

void set_motion_profile(MotionProfile p);
MotionProfile get_motion_profile();
const char* motion_profile_name(MotionProfile p);

