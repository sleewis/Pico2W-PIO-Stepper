#pragma once
#include <Arduino.h>
#include "types.h"

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
