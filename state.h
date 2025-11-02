#pragma once
#include <Arduino.h>
#include "types.h"
#include "config.h"

// Globale PIO/SM
extern PIO  pio;
extern int  sm;
extern uint prog_off;

// Realtime posities
extern volatile int32_t realtime_X, realtime_Y, realtime_Z;

// Motion flags
extern volatile bool motion_active, motion_complete, move_done_irq_flag;

// DMA boekhouding
extern volatile uint32_t dma_move_sent_prev;
extern volatile uint32_t dma_active_len;
extern volatile uint32_t dma_processed_in_buf;
extern int dma_channel;

// Command buffer
extern uint32_t command_buffer[BUFFER_SIZE];

static inline void read_positions_atomic(int32_t& x,int32_t& y,int32_t& z){
  noInterrupts(); x=realtime_X; y=realtime_Y; z=realtime_Z; interrupts();
}
