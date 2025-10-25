// Pico2W_Simple_Stepper3 - TRUE Real-time Position Tracking
// Uses DMA transfer count for exact step counting

#include <Arduino.h>
extern "C" {
  #include "hardware/pio.h"
  #include "hardware/clocks.h"
  #include "hardware/dma.h"
  #include "hardware/regs/dma.h"      // DMA_CH0_CTRL_TRIG_EN_BITS
  #include "hardware/structs/dma.h"   // dma_hw
  #include "hardware/irq.h"
}

#include "stepper_1sm.pio.h"

enum Axis { AXIS_X=0, AXIS_Y=1, AXIS_Z=2 };

struct LineMove {
  int32_t  sx, sy, sz;
  uint32_t d_start_us;
  uint32_t d_cruise_us;
  uint32_t d_end_us;
  float    accel_frac;
  float    decel_frac;
  
  LineMove() : sx(0), sy(0), sz(0), d_start_us(2000), d_cruise_us(1000), 
               d_end_us(2000), accel_frac(0.35f), decel_frac(0.35f) {}
};

// =============================
// Global PIO/SM settings
// =============================
static PIO  pio = pio0;
static int  sm  = -1;
static uint prog_off = 0;

static constexpr uint OUT_BASE  = 0;
static constexpr uint OUT_COUNT = 6;
static constexpr uint SET_BASE  = 3;
static constexpr uint SET_COUNT = 3;

#define DIR_INV_MASK  (0u)

static inline float clkdiv_1MHz() {
  return (float)clock_get_hz(clk_sys) / 1'000'000.0f;
}

static inline uint32_t make_cmd(bool dx,bool dy,bool dz,
                                bool sx,bool sy,bool sz,
                                uint32_t delay_extra_us) {
  uint32_t dir  = ((dx?1u:0u)<<0) | ((dy?1u:0u)<<1) | ((dz?1u:0u)<<2);
  dir ^= DIR_INV_MASK;
  uint32_t step = ((sx?1u:0u)<<3) | ((sy?1u:0u)<<4) | ((sz?1u:0u)<<5);
  if (delay_extra_us > 0x3FFFFFFu) delay_extra_us = 0x3FFFFFFu;
  return (delay_extra_us << 6) | dir | step;
}

void pio_stepper_init() {
  prog_off = pio_add_program(pio, &stepper_1sm_program);
  sm = pio_claim_unused_sm(pio, true);

  pio_sm_config c = stepper_1sm_program_get_default_config(prog_off);
  sm_config_set_out_pins(&c, OUT_BASE, OUT_COUNT);
  sm_config_set_set_pins(&c, SET_BASE, SET_COUNT);
  sm_config_set_out_shift(&c, true, false, 32);
  sm_config_set_clkdiv(&c, clkdiv_1MHz());
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

  for (uint i=0;i<OUT_COUNT;i++) pio_gpio_init(pio, OUT_BASE+i);
  pio_sm_set_consecutive_pindirs(pio, sm, OUT_BASE, OUT_COUNT, true);

  pio_sm_init(pio, sm, prog_off, &c);
  pio_sm_set_enabled(pio, sm, true);
}

// =============================
// TRUE REAL-TIME POSITION TRACKING
// =============================

static volatile bool move_done_irq_flag = false;  // NEW

// Global position counters
static volatile int32_t realtime_X = 0;
static volatile int32_t realtime_Y = 0; 
static volatile int32_t realtime_Z = 0;

// Track the actual commands processed vs generated
static uint32_t total_commands_generated = 0;
static uint32_t total_commands_processed = 0;

// Per-move / per-buffer DMA accounting
static volatile uint32_t dma_move_sent_prev = 0;     // words fully sent in previous buffers (this move)
static volatile uint32_t dma_active_len     = 0;     // words in the currently active DMA transfer
static volatile uint32_t dma_processed_in_buf = 0;   // words we've already parsed in the current buffer

// Helper
static inline void read_positions_atomic(int32_t& x, int32_t& y, int32_t& z) {
  noInterrupts();
  x = realtime_X; y = realtime_Y; z = realtime_Z;
  interrupts();
}

// Current move tracking
static struct {
  bool dx, dy, dz;  // Direction flags for current move
  uint32_t total_commands;  // Total commands in current move
} current_move = {false, false, false, 0};

// Function to parse step command and update positions
// FIXED: Process step command without double-counting
void process_step_command(uint32_t command) {
  // Extract direction and step bits from command
  uint32_t dir_bits = command & 0x7;
  uint32_t step_bits = (command >> 3) & 0x7;
  
  // Update positions based on which steps are active
  if (step_bits & (1u << 0)) { // X step
    if (dir_bits & (1u << 0)) {
      realtime_X++;
    } else {
      realtime_X--;
    }
  }
  
  if (step_bits & (1u << 1)) { // Y step
    if (dir_bits & (1u << 1)) {
      realtime_Y++;
    } else {
      realtime_Y--;
    }
  }
  
  if (step_bits & (1u << 2)) { // Z step
    if (dir_bits & (1u << 2)) {
      realtime_Z++;
    } else {
      realtime_Z--;
    }
  }
  
  total_commands_processed++;
}


// =============================
// Motion planning
// =============================

// =============================
// FIXED Motion planning - Exact Step Counting
// =============================

// === Jerk-gelimiteerde S-curve planner (drop-in) ============================
#include <math.h>
#include <algorithm>
using std::max;

struct TrapS {
  uint32_t Nx, Ny, Nz;
  uint32_t Nmax;
  Axis     master;
  bool     dx, dy, dz;

  uint32_t steps_acc, steps_cruise, steps_decel;
  int32_t  d_start, d_cruise, d_end;
  int32_t  d_cur;
  enum Phase { ACCEL, CRUISE, DECEL, DONE } phase;
  uint32_t i_in_phase;

  // Bresenham error-accu's voor de slave-assen
  int32_t ex, ey, ez;

  // Debug/telemetry
  uint32_t total_steps_generated;

  static inline float smooth5(float t) {
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    // 6t^5 - 15t^4 + 10t^3  -> C2-continu (jerk=0 op de randen)
    return t*t*t*(10.0f + t*(-15.0f + 6.0f*t));
  }

  void init(const LineMove& m, int32_t cur_x, int32_t cur_y, int32_t cur_z) {
    int32_t dxs = m.sx - cur_x;
    int32_t dys = m.sy - cur_y;
    int32_t dzs = m.sz - cur_z;

    Nx = (uint32_t)abs(dxs);
    Ny = (uint32_t)abs(dys);
    Nz = (uint32_t)abs(dzs);
    Nmax = max(Nx, max(Ny, Nz));
    if (!Nmax) { phase = DONE; return; }

    dx = (dxs >= 0); dy = (dys >= 0); dz = (dzs >= 0);
    master = (Nmax==Nx)?AXIS_X:((Nmax==Ny)?AXIS_Y:AXIS_Z);

    // Zelfde segmentverdeling als je oude code
    steps_acc   = (uint32_t)roundf(m.accel_frac * Nmax);
    steps_decel = (uint32_t)roundf(m.decel_frac * Nmax);
    if (steps_acc + steps_decel > Nmax) {
      steps_acc = Nmax/2;
      steps_decel = Nmax - steps_acc;
    }
    steps_cruise = Nmax - steps_acc - steps_decel;

    d_start  = (int32_t)m.d_start_us;   // langzame start (grote periode)
    d_cruise = (int32_t)m.d_cruise_us;  // doel-snelheid
    d_end    = (int32_t)m.d_end_us;     // langzame eindperiode

    phase = (steps_acc?ACCEL:(steps_cruise?CRUISE:(steps_decel?DECEL:DONE)));
    i_in_phase = 0;
    d_cur = d_start;

    ex = ey = ez = -(int32_t)Nmax/2;
    total_steps_generated = 0;
  }

  size_t fill(uint32_t* out, size_t cap) {
    size_t n = 0;
    if (phase==DONE) return 0;

    while (n < cap && phase != DONE) {
      // --- Bepaal welke as(sen) stappen (Bresenham) ---
      bool sxp=false, syp=false, szp=false;
      switch (master) {
        case AXIS_X:
          sxp = true;
          ey += (int32_t)Ny; if (ey >= 0) { ey -= (int32_t)Nmax; syp = true; }
          ez += (int32_t)Nz; if (ez >= 0) { ez -= (int32_t)Nmax; szp = true; }
          break;
        case AXIS_Y:
          syp = true;
          ex += (int32_t)Nx; if (ex >= 0) { ex -= (int32_t)Nmax; sxp = true; }
          ez += (int32_t)Nz; if (ez >= 0) { ez -= (int32_t)Nmax; szp = true; }
          break;
        case AXIS_Z:
          szp = true;
          ex += (int32_t)Nx; if (ex >= 0) { ex -= (int32_t)Nmax; sxp = true; }
          ey += (int32_t)Ny; if (ey >= 0) { ey -= (int32_t)Nmax; syp = true; }
          break;
      }

      // --- S-curve tijdsprofiel: bepaal d_cur per fase via quintic smoothstep ---
      switch (phase) {
        case ACCEL: {
          if (steps_acc == 0) { d_cur = d_cruise; }
          else {
            float t = (float)i_in_phase / (float)steps_acc;
            float s = smooth5(t);
            // Van langzame periode -> snelle periode (monotoon dalend)
            d_cur = (int32_t)lrintf((1.0f - s) * (float)d_start + s * (float)d_cruise);
          }
        } break;

        case CRUISE:
          d_cur = d_cruise;
          break;

        case DECEL: {
          if (steps_decel == 0) { d_cur = d_end; }
          else {
            float t = (float)i_in_phase / (float)steps_decel;
            float s = smooth5(t);
            // Van cruise periode -> langzame eindperiode (monotoon stijgend)
            d_cur = (int32_t)lrintf((1.0f - s) * (float)d_cruise + s * (float)d_end);
          }
        } break;

        default: break;
      }

      if (d_cur < 0) d_cur = 0;
      const uint32_t PULSE_US = 20; // jouw vaste pulswijdte
      uint32_t extra = ( (uint32_t)d_cur > PULSE_US ) ? ( (uint32_t)d_cur - PULSE_US ) : 0u;

      out[n++] = make_cmd(dx, dy, dz, sxp, syp, szp, extra);
      total_steps_generated++;

      // --- Faseboekhouding ---
      i_in_phase++;
      switch (phase) {
        case ACCEL:
          if (i_in_phase >= steps_acc) {
            phase = (steps_cruise?CRUISE:(steps_decel?DECEL:DONE));
            i_in_phase = 0;
          }
          break;
        case CRUISE:
          if (i_in_phase >= steps_cruise) {
            phase = (steps_decel?DECEL:DONE);
            i_in_phase = 0;
          }
          break;
        case DECEL:
          if (i_in_phase >= steps_decel) {
            phase = DONE;
          }
          break;
        default: break;
      }
    }
    return n;
  }
};


// =============================
// Move Queue Implementation
// =============================
static constexpr size_t MOVE_QUEUE_SIZE = 32;
static LineMove moveQueue[MOVE_QUEUE_SIZE];
static volatile uint8_t queueHead = 0;
static volatile uint8_t queueTail = 0;
static volatile uint8_t queueCount = 0;

bool is_queue_empty() {
  return queueCount == 0;
}

bool is_queue_full() {
  return queueCount >= MOVE_QUEUE_SIZE;
}

bool queue_move(const LineMove& move) {
  if (is_queue_full()) {
    return false;
  }
  
  moveQueue[queueTail] = move;
  queueTail = (queueTail + 1) % MOVE_QUEUE_SIZE;
  queueCount++;
  return true;
}

bool dequeue_move(LineMove* move) {
  if (is_queue_empty()) {
    return false;
  }
  
  *move = moveQueue[queueHead];
  queueHead = (queueHead + 1) % MOVE_QUEUE_SIZE;
  queueCount--;
  return true;
}

void clear_queue() {
  queueHead = 0;
  queueTail = 0;
  queueCount = 0;
}

uint8_t get_queue_count() {
  return queueCount;
}

// =============================
// Homing Emergency Stop System
// =============================

const uint HOME_X_PIN = 20;
const uint HOME_Y_PIN = 21;
//const uint HOME_Z_PIN = 8;

volatile bool homing_triggered = false;
volatile uint triggered_sensor = 0;
volatile bool homing_enabled = false;

// =============================
// DMA with TRUE REAL-TIME POSITION TRACKING
// =============================
static constexpr size_t BUFFER_SIZE = 512;  // Smaller buffer for more frequent updates
static uint32_t command_buffer[BUFFER_SIZE];
static int dma_channel = -1;
static volatile bool motion_active = false;
static volatile bool motion_complete = false;

static TrapS g_gen;

void emergency_stop() {
  stop_motion_force();
  clear_queue();
  Serial.println("EMERGENCY STOP executed.");
}

// Soft: netjes uit laten lopen (geen abort, SM blijft aan)
// Gebruik dit NIET middenin een actieve move; bedoeld voor "wachten tot idle"
void stop_motion_soft() {
  if (dma_channel < 0) return;

  // Wacht tot de lopende burst klaar is
  while (dma_channel_is_busy(dma_channel)) {
    update_realtime_positions_from_dma();   // realtime tellen wat al verstuurd is
    tight_loop_contents();
  }
  // Verwerk rest uit huidige buffer
  update_realtime_positions_from_dma();

  // Wacht tot de PIO TX FIFO leeg is (zou snel 0 zijn)
  while (pio_sm_get_tx_fifo_level(pio, sm) != 0) {
    tight_loop_contents();
  }

  // Pending IRQ (van de net afgeronde burst) opruimen
  dma_channel_acknowledge_irq0(dma_channel);

  motion_active   = false;
  // Laat motion_complete door je IRQ of start_next_move() bepalen
}

// Force: RP2350-veilige abort middenin een move (E-stop)
// - IRQ uit
// - EN-bit clear (E5)
// - abort
// - spurious IRQ weg
// - SM/FIFO reset
void stop_motion_force() {
  if (dma_channel < 0) {
    // PIO naar schone staat
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    motion_active = false;
    motion_complete = false;
    return;
  }

  // Verwerk wat al verstuurd is vóór we hard stoppen
  update_realtime_positions_from_dma();

  // 1) IRQ tijdelijk uit + pending clear
  dma_channel_set_irq0_enabled(dma_channel, false);
  dma_hw->ints0 = (1u << dma_channel);

  // 2) RP2350-E5: EN-bit van het kanaal eerst uit
  dma_hw->ch[dma_channel].ctrl_trig &= ~DMA_CH0_CTRL_TRIG_EN_BITS;
  __dmb();  // geheugenbarrière

  // 3) Abort; wacht tot bus/in-flight klaar is
  dma_channel_abort(dma_channel);

  // 4) Spurious 'done' IRQ (E13/E5) wegklikken
  dma_hw->ints0 = (1u << dma_channel);

  // 5) PIO in schone staat
  pio_sm_set_enabled(pio, sm, false);
  pio_sm_clear_fifos(pio, sm);
  pio_sm_restart(pio, sm);
  // (optioneel) pinnen hard laag forceren met pio_sm_exec(...)

  // 6) IRQ weer aan voor volgende moves
  dma_channel_set_irq0_enabled(dma_channel, true);

  motion_active   = false;
  motion_complete = false;
}

void homing_isr(uint gpio, uint32_t events) {
  if (homing_enabled && (events & GPIO_IRQ_LEVEL_LOW)) {
    triggered_sensor = gpio;
    homing_triggered = true;
    
    emergency_stop();
    
    homing_enabled = false;
    gpio_set_irq_enabled(HOME_X_PIN, GPIO_IRQ_LEVEL_LOW, false);
    gpio_set_irq_enabled(HOME_Y_PIN, GPIO_IRQ_LEVEL_LOW, false);
    //gpio_set_irq_enabled(HOME_Z_PIN, GPIO_IRQ_EDGE_FALL, false);
  }
}

void setup_homing_sensors() {
  gpio_init(HOME_X_PIN);
  gpio_set_dir(HOME_X_PIN, GPIO_IN);
  gpio_pull_up(HOME_X_PIN);
  
  gpio_init(HOME_Y_PIN);
  gpio_set_dir(HOME_Y_PIN, GPIO_IN);
  gpio_pull_up(HOME_Y_PIN);
  
  //gpio_init(HOME_Z_PIN);
  //gpio_set_dir(HOME_Z_PIN, GPIO_IN);
  //gpio_pull_up(HOME_Z_PIN);
  
  gpio_set_irq_enabled_with_callback(HOME_X_PIN, GPIO_IRQ_LEVEL_LOW, false, &homing_isr);
  gpio_set_irq_enabled(HOME_Y_PIN, GPIO_IRQ_EDGE_FALL, false);
  //gpio_set_irq_enabled(HOME_Z_PIN, GPIO_IRQ_EDGE_FALL, false);
  
  homing_enabled = false;
}

void enable_homing_sensors() {
  homing_enabled = true;
  gpio_set_irq_enabled(HOME_X_PIN, GPIO_IRQ_LEVEL_LOW, true);
  gpio_set_irq_enabled(HOME_Y_PIN, GPIO_IRQ_LEVEL_LOW, true);
  //gpio_set_irq_enabled(HOME_Z_PIN, GPIO_IRQ_EDGE_FALL, true);
  Serial.println("Homing sensors ENABLED");
}

void disable_homing_sensors() {
  homing_enabled = false;
  gpio_set_irq_enabled(HOME_X_PIN, GPIO_IRQ_LEVEL_LOW, false);
  gpio_set_irq_enabled(HOME_Y_PIN, GPIO_IRQ_LEVEL_LOW, false);
  //gpio_set_irq_enabled(HOME_Z_PIN, GPIO_IRQ_EDGE_FALL, false);
  Serial.println("Homing sensors DISABLED");
}

void check_homing_safety() {
  if (homing_triggered) {
    homing_triggered = false;
    
    Serial.println();
    Serial.println("!!! EMERGENCY STOP - Homing Sensor Triggered !!!");
    Serial.print("Sensor on GPIO: "); Serial.println(triggered_sensor);
    Serial.print("Stopped at X:"); Serial.print(realtime_X);
    Serial.print(" Y:"); Serial.print(realtime_Y);
    Serial.print(" Z:"); Serial.println(realtime_Z);
    Serial.println("All motion stopped and queue cleared");
    Serial.println("Use M121 to disable sensors, then resume with G-code");
  }
}

// Get the number of commands actually transferred by DMA
uint32_t get_dma_transferred_count_cumulative() {
  if (!motion_active || dma_channel < 0) return dma_move_sent_prev + dma_processed_in_buf;
  uint32_t remaining = dma_channel_hw_addr(dma_channel)->transfer_count;
  uint32_t sent_in_buf = (dma_active_len > remaining) ? (dma_active_len - remaining) : 0;
  return dma_move_sent_prev + sent_in_buf;
}

void update_realtime_positions_from_dma() {
  if (!motion_active || dma_channel < 0 || dma_active_len == 0) return;
  uint32_t remaining = dma_channel_hw_addr(dma_channel)->transfer_count;
  uint32_t sent_in_buf = (dma_active_len > remaining) ? (dma_active_len - remaining) : 0;
  for (uint32_t i = dma_processed_in_buf; i < sent_in_buf; ++i) {
    process_step_command(command_buffer[i]);
  }
  dma_processed_in_buf = sent_in_buf;
}

// DMA completion handler
void __isr dma_complete_handler() {
  if (dma_channel_get_irq0_status(dma_channel)) {
    dma_channel_acknowledge_irq0(dma_channel);
    
    // Process any remaining commands in the buffer
    update_realtime_positions_from_dma();
    
    // We've finished the current buffer: account for it
    dma_move_sent_prev   += dma_active_len;
    dma_processed_in_buf  = 0;
    dma_active_len        = 0;    // NEW: mark no active burst while we refill

    if (g_gen.phase != TrapS::DONE) {
      // Fill next buffer
      size_t n = g_gen.fill(command_buffer, BUFFER_SIZE);
      if (n > 0) {
        dma_active_len = n;
        dma_channel_set_read_addr(dma_channel, command_buffer, false);
        dma_channel_set_trans_count(dma_channel, n, true);
      } else {
        motion_complete = true;
        motion_active = false;
      }
    } else {
      motion_complete = true;
      motion_active = false;
      move_done_irq_flag = true;   // NEW: signal main loop to print
    }
  }
}

// Initialize DMA
void init_dma() {
  if (dma_channel >= 0) {
    dma_channel_unclaim(dma_channel);
  }
  
  dma_channel = dma_claim_unused_channel(true);
  uint dreq = pio_get_dreq(pio, sm, true);
  
  dma_channel_config config = dma_channel_get_default_config(dma_channel);
  channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
  channel_config_set_read_increment(&config, true);
  channel_config_set_write_increment(&config, false);
  channel_config_set_dreq(&config, dreq);
  
  dma_channel_configure(dma_channel, &config, 
                       &pio->txf[sm],
                       command_buffer,
                       0,
                       false);
  
  dma_channel_set_irq0_enabled(dma_channel, true);
  irq_set_exclusive_handler(DMA_IRQ_0, dma_complete_handler);
  irq_set_enabled(DMA_IRQ_0, true);
}

// Stop motion

// Start motion from queue
void start_next_move() {
  if (motion_active || is_queue_empty()) {
    return;
  }
  
  LineMove next_move;
  if (dequeue_move(&next_move)) {
    // stop_motion();
    
    // Initialize generator with current REAL positions
    g_gen.init(next_move, realtime_X, realtime_Y, realtime_Z);
    
    if (g_gen.phase == TrapS::DONE) {
      start_next_move();
      return;
    }
    
    size_t num_commands = g_gen.fill(command_buffer, BUFFER_SIZE);

    // Reset per-move DMA accounting
    dma_move_sent_prev   = 0;
    dma_active_len       = num_commands;
    dma_processed_in_buf = 0;

    // Also reset (since we dropped the per-move guard anyway, this keeps logs sane)
    total_commands_processed = 0;
    total_commands_generated = num_commands; // optional (used only for prints now)
    
    if (num_commands == 0) {
      start_next_move();
      return;
    }
    
    dma_channel_set_read_addr(dma_channel, command_buffer, false);
    dma_channel_set_trans_count(dma_channel, num_commands, false);
    
    dma_channel_acknowledge_irq0(dma_channel);
    
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    pio_sm_set_enabled(pio, sm, true);
    
    // In start_next_move(), after initializing the move:
    Serial.print("Move requested: X");
    Serial.print(next_move.sx);
    Serial.print(" from ");
    Serial.println(realtime_X);

    motion_active = true;
    motion_complete = false;
    dma_channel_start(dma_channel);
    
    Serial.print("Started move with ");
    Serial.print(num_commands);
    Serial.println(" commands");
  }
}

// =============================
// G-code Parser
// =============================
static float lastFeed_steps_s = 2000.0f;

static void handleLine(const String& line) {
  String s = line; s.trim(); s.toUpperCase(); 
  if (!s.length()) return;

  auto clamp = [](int32_t v){ return constrain(v, 0, 80000); };

  if (s.startsWith("G0") || s.startsWith("G1")) {
    int idx; 
    int32_t tx=realtime_X, ty=realtime_Y, tz=realtime_Z; 
    float f=lastFeed_steps_s;
    
    if ((idx=s.indexOf('X'))>=0) tx = clamp(s.substring(idx+1).toInt());
    if ((idx=s.indexOf('Y'))>=0) ty = clamp(s.substring(idx+1).toInt());
    if ((idx=s.indexOf('Z'))>=0) tz = clamp(s.substring(idx+1).toInt());
    if ((idx=s.indexOf('F'))>=0) f  = max(1.0f, s.substring(idx+1).toFloat());

    LineMove m; 
    m.sx=tx; m.sy=ty; m.sz=tz;
    
    if (s.startsWith("G0")) {
      m.d_start_us=1500; m.d_cruise_us=600; m.d_end_us=1500;
    } else {
      m.d_cruise_us = (uint32_t)roundf(1e6f / f);
      m.d_start_us = (uint32_t)roundf(m.d_cruise_us * 3.5f); //2.0f
      m.d_end_us   = m.d_start_us;
      lastFeed_steps_s = f;
    }
    
    m.accel_frac = 0.60f; // 0.35
    m.decel_frac = 0.60f;
    
    if (queue_move(m)) {
      Serial.print("Queued move to X:");
      Serial.print(tx);
      Serial.print(" Y:");
      Serial.print(ty);
      Serial.print(" Z:");
      Serial.print(tz);
      Serial.print(" | Queue: ");
      Serial.println(get_queue_count());
      
      if (!motion_active) {
        start_next_move();
      }
    } else {
      Serial.println("ERROR: Move queue full!");
    }
    return;
  }

  if (s.startsWith("G28")) {
    LineMove m;
    m.sx=0; m.sy=0; m.sz=0;
    m.d_start_us=2000; m.d_cruise_us=1000; m.d_end_us=2000;
    m.accel_frac=0.35f; m.decel_frac=0.35f;
    
    if (queue_move(m)) {
      Serial.println("Queued homing move to origin");
      if (!motion_active) {
        start_next_move();
      }
    } else {
      Serial.println("ERROR: Queue full for homing move");
    }
    return;
  }

  if (s.startsWith("G92")) {
    // Set current position using REAL-TIME positions
    stop_motion_soft(); // Optioneel
    int idx;
    if ((idx=s.indexOf('X'))>=0) realtime_X = s.substring(idx+1).toInt();
    if ((idx=s.indexOf('Y'))>=0) realtime_Y = s.substring(idx+1).toInt();
    if ((idx=s.indexOf('Z'))>=0) realtime_Z = s.substring(idx+1).toInt();
    
    Serial.print("Position set to X:");
    Serial.print(realtime_X);
    Serial.print(" Y:");
    Serial.print(realtime_Y);
    Serial.print(" Z:");
    Serial.println(realtime_Z);
    return;
  }

  if (s.startsWith("M114")) {
    int32_t x,y,z; read_positions_atomic(x,y,z);
    Serial.print("POS: X:"); Serial.print(x);
    Serial.print(" Y:"); Serial.print(y);
    Serial.print(" Z:"); Serial.print(z);
    Serial.print(" | Moving: ");
    Serial.print(motion_active ? "YES" : "NO");
    Serial.print(" | Queue: ");
    Serial.println(get_queue_count());
    return;
  }

  if (s.startsWith("M0")) {
    stop_motion_force();
    clear_queue();
    Serial.println("Stopped motion and cleared queue");
    return;
  }

  if (s.startsWith("M2")) {
    stop_motion_force();
    Serial.println("Stopped motion (queue preserved)");
    return;
  }

  if (s.startsWith("M110")) {
    clear_queue();
    Serial.println("Queue cleared");
    return;
  }

  if (s.startsWith("M111")) {
    Serial.print("Queue: ");
    Serial.print(get_queue_count());
    Serial.print("/");
    Serial.print(MOVE_QUEUE_SIZE);
    Serial.println(" moves");
    return;
  }

  if (s.startsWith("M120")) {
    enable_homing_sensors();
    return;
  }

  if (s.startsWith("M121")) {
    disable_homing_sensors();
    return;
  }
}

// =============================
// TRUE REAL-TIME DISPLAY
// =============================
void update_realtime_display() {
  static uint32_t last_display = 0;
  static String previous_line = "";
  
  if (millis() - last_display < 200) return; // 30Hz updates
  last_display = millis();
  
  // Update positions from DMA progress BEFORE displaying
  update_realtime_positions_from_dma();
  
  int32_t x,y,z; read_positions_atomic(x,y,z);
  String current_line = "POS: " + String(x) + ", " + String(y) + ", " + String(z)+ 
                       (motion_active ? " [MOVING]" : " [IDLE]") +
                       " Q:" + String(get_queue_count()) +
                       " H:" + (homing_enabled ? "ON" : "OFF");
  
  if (previous_line != current_line) {
    Serial.println(current_line);
  }
  
  Serial.flush();
  previous_line = current_line;
}

// =============================
// Setup and Loop
// =============================
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Pico Stepper Controller - TRUE Real-time DMA Position Tracking");
  
  pio_stepper_init();
  init_dma();
  setup_homing_sensors();
  
  // Initialize real-time positions
  realtime_X = 0;
  realtime_Y = 0;
  realtime_Z = 0;
  
  Serial.println("System ready. TRUE real-time DMA position tracking active!");
  Serial.println("Commands:");
  Serial.println("G0/G1 X.. Y.. Z.. F..  - Queue move");
  Serial.println("G28                     - Home to origin");
  Serial.println("G92 X.. Y.. Z..         - Set current position");
  Serial.println("M114                    - Report position/queue");
  Serial.println("M0                      - Stop and clear queue");
  Serial.println("M2                      - Stop (keep queue)");
  Serial.println("M110                    - Clear queue");
  Serial.println("M111                    - Queue status");
  Serial.println("M120                    - Enable homing sensors");
  Serial.println("M121                    - Disable homing sensors");
  Serial.println();
}

void loop() {
  static String buf;

  // Check for homing sensor triggers
  check_homing_safety();
  
  // Handle motion completion
  if (motion_complete) {
    motion_complete = false;
    
    // Final position update
    update_realtime_positions_from_dma();
    
    if (move_done_irq_flag) {
      move_done_irq_flag = false;
      Serial.print("Move completed. Total steps generated: ");
      Serial.println(g_gen.total_steps_generated);
    }

    start_next_move();
  }

  // TRUE REAL-TIME display - queries DMA hardware register directly
  update_realtime_display();
  
  // Handle serial commands
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length() > 0) {
        Serial.println();
        handleLine(buf);
        buf = "";
      }
    } else if (c >= 32 && c < 127) {
      buf += c;
      if (buf.length() > 100) buf.remove(0, 1);
    }
  }
  
  delay(1);
}
