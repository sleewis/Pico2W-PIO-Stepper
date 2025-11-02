#include "state.h"
#include "hardware/pio.h"
#include "config.h"

PIO  pio = pio0;
int  sm  = -1;
uint prog_off = 0;

volatile int32_t realtime_X = 0;
volatile int32_t realtime_Y = 0;
volatile int32_t realtime_Z = 0;

volatile bool motion_active = false;
volatile bool motion_complete = false;
volatile bool move_done_irq_flag = false;

volatile uint32_t dma_move_sent_prev   = 0;
volatile uint32_t dma_active_len       = 0;
volatile uint32_t dma_processed_in_buf = 0;

int dma_channel = -1;

uint32_t command_buffer[BUFFER_SIZE] = {0};
