/*
G28             ; Home to origin
M114            ; Check position (should be 0,0,0)
G1 X1000 F1000  ; Move 1000 steps on X
M114            ; Check position (should be 1000,0,0)
G1 Y500 F500    ; Move 500 steps on Y
M114            ; Check position

PINS: Dir+ XYZ [0..2] Pulse+ [3..5]
*/

// Pico2W_stepper_1sm_V3 – Simplified DMA approach - With Move Queue
// More reliable DMA initialization and operation

#include <Arduino.h>
extern "C" {
  #include "hardware/pio.h"
  #include "hardware/clocks.h"
  #include "hardware/dma.h"
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
// Motion planning
// =============================

struct TrapDDA {
  uint32_t Nx, Ny, Nz;
  uint32_t Nmax;
  Axis     master;
  bool     dx, dy, dz;

  uint32_t steps_acc, steps_cruise, steps_decel;
  int32_t  d_start, d_cruise, d_end;
  int32_t  d_cur;
  enum Phase { ACCEL, CRUISE, DECEL, DONE } phase;
  uint32_t i_in_phase;

  uint32_t ex, ey, ez;
  int32_t curX=0, curY=0, curZ=0;

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

    steps_acc   = (uint32_t)roundf(m.accel_frac * Nmax);
    steps_decel = (uint32_t)roundf(m.decel_frac * Nmax);
    if (steps_acc + steps_decel > Nmax) { 
      steps_acc = Nmax/2; 
      steps_decel = Nmax - steps_acc; 
    }
    steps_cruise = Nmax - steps_acc - steps_decel;

    d_start  = (int32_t)m.d_start_us;
    d_cruise = (int32_t)m.d_cruise_us;
    d_end    = (int32_t)m.d_end_us;

    phase = (steps_acc?ACCEL:(steps_cruise?CRUISE:(steps_decel?DECEL:DONE)));
    i_in_phase = 0; 
    d_cur = d_start;

    ex = ey = ez = 0;
    curX = cur_x; curY = cur_y; curZ = cur_z;
  }

  size_t fill(uint32_t* out, size_t cap) {
    size_t n = 0; 
    if (phase==DONE) return 0;
    
    while (n < cap && phase != DONE) {
      bool sxp=false, syp=false, szp=false;
      switch (master) {
        case AXIS_X: 
          sxp = true; 
          ey += Ny; if (ey >= Nmax){ ey -= Nmax; syp = (Ny!=0);} 
          ez += Nz; if (ez>=Nmax){ ez -= Nmax; szp=(Nz!=0);} 
          break;
        case AXIS_Y: 
          syp = true; 
          ex += Nx; if (ex >= Nmax){ ex -= Nmax; sxp = (Nx!=0);} 
          ez += Nz; if (ez>=Nmax){ ez -= Nmax; szp=(Nz!=0);} 
          break;
        case AXIS_Z: 
          szp = true; 
          ex += Nx; if (ex >= Nmax){ ex -= Nmax; sxp = (Nx!=0);} 
          ey += Ny; if (ey>=Nmax){ ey -= Nmax; syp=(Ny!=0);} 
          break;
      }

      uint32_t dt_total = (d_cur < 0) ? 0u : (uint32_t)d_cur;
      const uint32_t PULSE_US = 20;
      uint32_t extra = (dt_total > PULSE_US) ? (dt_total - PULSE_US) : 0u;
      out[n++] = make_cmd(dx,dy,dz, sxp,syp,szp, extra);

      if (sxp) curX += dx?+1:-1; 
      if (syp) curY += dy?+1:-1; 
      if (szp) curZ += dz?+1:-1;

      i_in_phase++;
      switch (phase) {
        case ACCEL:
          if (i_in_phase >= steps_acc) { 
            phase = (steps_cruise?CRUISE:(steps_decel?DECEL:DONE)); 
            i_in_phase = 0; 
            d_cur = d_cruise; 
          } else {
            float t = (float)i_in_phase / (float)steps_acc;
            float eased_t = t * t;
            d_cur = (int32_t)(d_start + (d_cruise - d_start) * eased_t);
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
          } else {
            float t = (float)i_in_phase / (float)steps_decel;
            float eased_t = 1.0f - (1.0f - t) * (1.0f - t);
            d_cur = (int32_t)(d_cruise + (d_end - d_cruise) * eased_t);
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

// Check if queue is empty
bool is_queue_empty() {
  return queueCount == 0;
}

// Check if queue is full
bool is_queue_full() {
  return queueCount >= MOVE_QUEUE_SIZE;
}

// Add move to queue
bool queue_move(const LineMove& move) {
  if (is_queue_full()) {
    return false;
  }
  
  moveQueue[queueTail] = move;
  queueTail = (queueTail + 1) % MOVE_QUEUE_SIZE;
  queueCount++;
  return true;
}

// Get next move from queue
bool dequeue_move(LineMove* move) {
  if (is_queue_empty()) {
    return false;
  }
  
  *move = moveQueue[queueHead];
  queueHead = (queueHead + 1) % MOVE_QUEUE_SIZE;
  queueCount--;
  return true;
}

// Clear all moves from queue
void clear_queue() {
  queueHead = 0;
  queueTail = 0;
  queueCount = 0;
}

// Get queue status
uint8_t get_queue_count() {
  return queueCount;
}

// =============================
// DMA and Motion Control
// =============================
static constexpr size_t BUFFER_SIZE = 512;
static uint32_t command_buffer[BUFFER_SIZE];
static int dma_channel = -1;
static volatile bool motion_active = false;
static volatile bool motion_complete = false;

static TrapDDA g_gen;
static int32_t gCurX=0, gCurY=0, gCurZ=0;

// DMA completion handler
void __isr dma_complete_handler() {
  if (dma_channel_get_irq0_status(dma_channel)) {
    dma_channel_acknowledge_irq0(dma_channel);
    
    if (g_gen.phase != TrapDDA::DONE) {
      size_t n = g_gen.fill(command_buffer, BUFFER_SIZE);
      if (n > 0) {
        dma_channel_set_read_addr(dma_channel, command_buffer, false);
        dma_channel_set_trans_count(dma_channel, n, true);
      } else {
        motion_complete = true;
        motion_active = false;
      }
    } else {
      motion_complete = true;
      motion_active = false;
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
void stop_motion() {
  if (dma_channel >= 0) {
    dma_channel_abort(dma_channel);
  }
  
  pio_sm_set_enabled(pio, sm, false);
  pio_sm_clear_fifos(pio, sm);
  
  for(uint i = SET_BASE; i < SET_BASE + SET_COUNT; i++) {
    gpio_put(i, 0);
  }
  
  motion_active = false;
  motion_complete = false;
  
  if (g_gen.phase == TrapDDA::DONE) {
    gCurX = g_gen.curX;
    gCurY = g_gen.curY;
    gCurZ = g_gen.curZ;
  }
}

// Start motion from queue
void start_next_move() {
  if (motion_active || is_queue_empty()) {
    return;
  }
  
  LineMove next_move;
  if (dequeue_move(&next_move)) {
    stop_motion();
    
    g_gen.init(next_move, gCurX, gCurY, gCurZ);
    
    if (g_gen.phase == TrapDDA::DONE) {
      // Zero-length move, try next one
      start_next_move();
      return;
    }
    
    size_t num_commands = g_gen.fill(command_buffer, BUFFER_SIZE);
    
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
    
    motion_active = true;
    motion_complete = false;
    dma_channel_start(dma_channel);
    
    Serial.print("Started move. Queue: ");
    Serial.println(get_queue_count());
  }
}

// =============================
// G-code Parser with Queue Support
// =============================
static float lastFeed_steps_s = 2000.0f;

static void handleLine(const String& line) {
  String s = line; s.trim(); s.toUpperCase(); 
  if (!s.length()) return;

  auto clamp = [](int32_t v){ return constrain(v, -24000, 24000); };

  if (s.startsWith("G0") || s.startsWith("G1")) {
    int idx; 
    int32_t tx=gCurX, ty=gCurY, tz=gCurZ; 
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
      m.d_start_us = (uint32_t)roundf(m.d_cruise_us * 2.0f);
      m.d_end_us   = m.d_start_us;
      lastFeed_steps_s = f;
    }
    
    m.accel_frac = 0.35f; 
    m.decel_frac = 0.35f;
    
    if (queue_move(m)) {
      Serial.print("Queued move to X:");
      Serial.print(tx);
      Serial.print(" Y:");
      Serial.print(ty);
      Serial.print(" Z:");
      Serial.print(tz);
      Serial.print(" | Queue: ");
      Serial.println(get_queue_count());
      
      // Start motion if not already running
      if (!motion_active) {
        start_next_move();
      }
    } else {
      Serial.println("ERROR: Move queue full!");
    }
    return;
  }

  if (s.startsWith("M114")) {
    Serial.print("POS: X:");
    Serial.print(gCurX);
    Serial.print(" Y:");
    Serial.print(gCurY);
    Serial.print(" Z:");
    Serial.print(gCurZ);
    Serial.print(" | Moving: ");
    Serial.print(motion_active ? "YES" : "NO");
    Serial.print(" | Queue: ");
    Serial.println(get_queue_count());
    return;
  }

  if (s.startsWith("M0") || s.startsWith("M1")) {
    stop_motion();
    clear_queue();
    Serial.println("Stopped motion and cleared queue");
    return;
  }

  if (s.startsWith("M2")) {
    // Stop motion but keep queue
    stop_motion();
    Serial.println("Stopped motion (queue preserved)");
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

  if (s.startsWith("M110")) {
    // Clear queue command
    clear_queue();
    Serial.println("Queue cleared");
    return;
  }

  if (s.startsWith("M111")) {
    // Queue status
    Serial.print("Queue: ");
    Serial.print(get_queue_count());
    Serial.print("/");
    Serial.print(MOVE_QUEUE_SIZE);
    Serial.println(" moves");
    return;
  }
}

// =============================
// Real-time Display
// =============================
void update_realtime_display() {
  static uint32_t last_display = 0;
  static String previous_line = "";
  
  if (millis() - last_display < 100) return;
  last_display = millis();
  
  String current_line = "POS: " + String(g_gen.curX) + ", " + 
                       String(g_gen.curY) + ", " + 
                       String(g_gen.curZ) + 
                       (motion_active ? " [MOVING]" : " [IDLE]") +
                       " Q:" + String(get_queue_count());
  
  Serial.print('\n');
  Serial.print(current_line);
  
  if (current_line.length() < previous_line.length()) {
    for (int i = current_line.length(); i < previous_line.length(); i++) {
      Serial.print(' ');
    }
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
  Serial.println("Pico Stepper Controller - With Move Queue");
  
  pio_stepper_init();
  init_dma();
  
  Serial.println("System ready. Commands:");
  Serial.println("G0/G1 X.. Y.. Z.. F..  - Queue move");
  Serial.println("M114                    - Report position/queue");
  Serial.println("M0                      - Stop and clear queue");
  Serial.println("M2                      - Stop (keep queue)");
  Serial.println("M110                    - Clear queue");
  Serial.println("M111                    - Queue status");
  Serial.println("G28                     - Home to origin");
  Serial.println();
}

void loop() {
  static String buf;

  // Handle motion completion
  if (motion_complete) {
    motion_complete = false;
    
    // Update global positions
    gCurX = g_gen.curX;
    gCurY = g_gen.curY;
    gCurZ = g_gen.curZ;
    
    Serial.println();
    Serial.print("Move completed. Starting next... Queue: ");
    Serial.println(get_queue_count());
    
    // Start next move in queue
    start_next_move();
  }

  // Update real-time display
  update_realtime_display();
  
  // Handle serial commands
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length() > 0) {
        Serial.println(); // New line for command output
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