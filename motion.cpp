#include <Arduino.h>
extern "C" {
  #include "hardware/pio.h"
  #include "hardware/clocks.h"
  #include "hardware/dma.h"
  #include "hardware/regs/dma.h"
  #include "hardware/structs/dma.h"
  #include "hardware/irq.h"
}
#include "stepper_1sm.pio.h"

#include "config.h"
#include "state.h"
#include "queue.h"   // voor clear_queue()
#include "motion.h"

// ---- planner (S-curve, Bresenham) ----
struct TrapS {
  // Geometry / DDA
  uint32_t Nx, Ny, Nz, Nmax;
  Axis     master;
  bool     dx, dy, dz;

  // Trapezoid segmentation
  uint32_t steps_acc, steps_cruise, steps_decel;

  // Periods (µs) and running state
  int32_t  d_start, d_cruise, d_end, d_cur;
  enum Phase { ACCEL, CRUISE, DECEL, DONE } phase;
  uint32_t i_in_phase;

  // Bresenham accumulators
  int32_t  ex, ey, ez;

  // Telemetry
  uint32_t total_steps_generated = 0;

  // --- Added for runtime motion profiles ---
  // Trapezoid (linear period)
  int32_t slope_acc = 0;  // Δ(period µs)/step during ACCEL
  int32_t slope_dec = 0;  // Δ(period µs)/step during DECEL

  // Constant-acceleration in speed space (steps/s)
  double v_start = 0.0, v_cruise_s = 0.0, v_end_s = 0.0; // boundary speeds
  double a_acc   = 0.0, a_dec     = 0.0;                 // per-step constant accel
  double v_cur   = 0.0;                                   // running speed at phase

  // Quintic smoothstep for S-curve
  static inline float smooth5(float t) {
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t*t*t*(10.0f + t*(-15.0f + 6.0f*t));
  }

  // API
  void   init(const LineMove& m, int32_t cur_x, int32_t cur_y, int32_t cur_z);
  size_t fill(uint32_t* out, size_t cap);
};


// --- helpers ---
static inline float clkdiv_1MHz(){
  return (float)clock_get_hz(clk_sys) / 1'000'000.0f;
}
static inline uint32_t make_cmd(bool dx,bool dy,bool dz,bool sx,bool sy,bool sz,uint32_t delay_extra_us){
  uint32_t dir  = ((dx?1u:0u)<<0) | ((dy?1u:0u)<<1) | ((dz?1u:0u)<<2);
  dir ^= DIR_INV_MASK;
  uint32_t step = ((sx?1u:0u)<<3) | ((sy?1u:0u)<<4) | ((sz?1u:0u)<<5);
  if(delay_extra_us>0x3FFFFFFu) delay_extra_us=0x3FFFFFFu;
  return (delay_extra_us<<6) | dir | step;
}

void pio_stepper_init(){
  prog_off = pio_add_program(pio, &stepper_1sm_program);
  sm = pio_claim_unused_sm(pio, true);

  pio_sm_config c = stepper_1sm_program_get_default_config(prog_off);
  sm_config_set_out_pins(&c, 0, 6);
  sm_config_set_set_pins(&c, 3, 3);
  sm_config_set_out_shift(&c, true, false, 32);
  sm_config_set_clkdiv(&c, clkdiv_1MHz());
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

  for(uint i=0;i<6;i++) pio_gpio_init(pio, 0+i);
  pio_sm_set_consecutive_pindirs(pio, sm, 0, 6, true);

  pio_sm_init(pio, sm, prog_off, &c);
  pio_sm_set_enabled(pio, sm, true);
}

// ---- realtime step accounting ----
static void process_step_command(uint32_t cmd){
  uint32_t dir  =  cmd & 0x7;          // (al ge-xor'd in PIO cmd)
  uint32_t step = (cmd>>3) & 0x7;
  if(step & 1u){ realtime_X += (dir & 1u) ? +1 : -1; }
  if(step & 2u){ realtime_Y += (dir & 2u) ? +1 : -1; }
  if(step & 4u){ realtime_Z += (dir & 4u) ? +1 : -1; }
}

void update_realtime_positions_from_dma(){
  if(!motion_active || dma_channel<0 || dma_active_len==0) return;
  uint32_t remaining = dma_channel_hw_addr(dma_channel)->transfer_count;
  uint32_t sent_in_buf = (dma_active_len > remaining) ? (dma_active_len - remaining) : 0;
  for(uint32_t i=dma_processed_in_buf;i<sent_in_buf;++i) process_step_command(command_buffer[i]);
  dma_processed_in_buf = sent_in_buf;
}

// Runtime motion-profile (default = CONST-ACC in speed)
static volatile MotionProfile s_profile = PROF_CONSTACC_SPEED;

void set_motion_profile(MotionProfile p) { s_profile = p; }
MotionProfile get_motion_profile() { return s_profile; }
const char* motion_profile_name(MotionProfile p) {
  switch (p) {
    case PROF_TRAPEZOID_PERIOD: return "TRAPEZOID (linear period)";
    case PROF_CONSTACC_SPEED:   return "CONST-ACC (speed)";
    case PROF_S_CURVE:          return "S-CURVE (quintic)";
    default: return "UNKNOWN";
  }
}


// ---- planner (S-curve, Bresenham) ----
void TrapS::init(const LineMove& m, int32_t cur_x, int32_t cur_y, int32_t cur_z) {
  // Deltas
  int32_t dxs = m.sx - cur_x;
  int32_t dys = m.sy - cur_y;
  int32_t dzs = m.sz - cur_z;

  Nx = (uint32_t)abs(dxs);
  Ny = (uint32_t)abs(dys);
  Nz = (uint32_t)abs(dzs);
  Nmax = max(Nx, max(Ny, Nz));
  if (!Nmax) { phase = DONE; return; }

  dx = (dxs >= 0); dy = (dys >= 0); dz = (dzs >= 0);
  master = (Nmax == Nx) ? AXIS_X : ((Nmax == Ny) ? AXIS_Y : AXIS_Z);

  // Segment fractions
  steps_acc   = (uint32_t)lrintf(m.accel_frac * Nmax);
  steps_decel = (uint32_t)lrintf(m.decel_frac * Nmax);
  if (steps_acc + steps_decel > Nmax) {
    steps_acc = Nmax/2;
    steps_decel = Nmax - steps_acc;
  }
  steps_cruise = Nmax - steps_acc - steps_decel;

  // Periods (µs)
  d_start  = (int32_t)m.d_start_us;
  d_cruise = (int32_t)m.d_cruise_us;
  d_end    = (int32_t)m.d_end_us;

  // Initial phase state
  phase = steps_acc ? ACCEL : (steps_cruise ? CRUISE : (steps_decel ? DECEL : DONE));
  i_in_phase = 0;
  d_cur = d_start;

  // DDA errors (centered)
  ex = ey = ez = -(int32_t)Nmax/2;

  total_steps_generated = 0;

  // --- Precompute for profiles ---
  // Trapezoid in period-space
  slope_acc = (steps_acc   ? (int32_t)((int64_t)d_cruise - d_start)  / (int32_t)steps_acc   : 0);
  slope_dec = (steps_decel ? (int32_t)((int64_t)d_end    - d_cruise) / (int32_t)steps_decel : 0);

  // Constant-acc in speed-space (steps/s)
  v_start   = (d_start  > 0) ? (1e6 / (double)d_start)  : 1e9;
  v_cruise_s= (d_cruise > 0) ? (1e6 / (double)d_cruise) : v_start;
  v_end_s   = (d_end    > 0) ? (1e6 / (double)d_end)    : v_cruise_s;

  a_acc = (steps_acc   ? ( (v_cruise_s*v_cruise_s - v_start*v_start) / (2.0 * (double)steps_acc) ) : 0.0);
  a_dec = (steps_decel ? ( (v_end_s*v_end_s      - v_cruise_s*v_cruise_s) / (2.0 * (double)steps_decel) ) : 0.0);

  switch (phase) {
    case ACCEL:  v_cur = v_start;     break;
    case CRUISE: v_cur = v_cruise_s;  break;
    case DECEL:  v_cur = v_cruise_s;  break;
    default:     v_cur = v_end_s;     break;
  }
}

size_t TrapS::fill(uint32_t* out, size_t cap) {
  size_t n = 0; 
  if (phase == DONE) return 0;

  while (n < cap && phase != DONE) {
    // --- Decide which axes step (Bresenham) ---
    bool sxp=false, syp=false, szp=false;
    switch (master) {
      case AXIS_X:
        sxp = true;
        ey += (int32_t)Ny; if (ey >= 0) { ey -= (int32_t)Nmax; if (Ny) syp = true; }
        ez += (int32_t)Nz; if (ez >= 0) { ez -= (int32_t)Nmax; if (Nz) szp = true; }
        break;
      case AXIS_Y:
        syp = true;
        ex += (int32_t)Nx; if (ex >= 0) { ex -= (int32_t)Nmax; if (Nx) sxp = true; }
        ez += (int32_t)Nz; if (ez >= 0) { ez -= (int32_t)Nmax; if (Nz) szp = true; }
        break;
      case AXIS_Z:
        szp = true;
        ex += (int32_t)Nx; if (ex >= 0) { ex -= (int32_t)Nmax; if (Nx) sxp = true; }
        ey += (int32_t)Ny; if (ey >= 0) { ey -= (int32_t)Nmax; if (Ny) syp = true; }
        break;
    }

    // ---- Per-step timing (three profiles) ----
    auto step_interval_constacc = [](double &v, double a)->double {
      if (a == 0.0) return (v > 1e-9) ? (1.0 / v) : 0.0;  // cruise
      double v2_next = v*v + 2.0*a; if (v2_next < 0.0) v2_next = 0.0;
      double v_next = sqrt(v2_next);
      double dt = (v_next - v) / a; if (dt < 0.0) dt = 0.0;
      v = v_next;
      return dt;
    };

    double dt_sec = 0.0;
    // Read current profile once for this step
    const MotionProfile prof = get_motion_profile();

    switch (phase) {
      case ACCEL:
        if (prof == PROF_S_CURVE) {
          if (steps_acc == 0) { d_cur = d_cruise; }
          else {
            float t = (float)i_in_phase / (float)steps_acc;
            float s = smooth5(t);
            d_cur = (int32_t)lrintf((1.0f - s) * (float)d_start + s * (float)d_cruise);
          }
          dt_sec = d_cur / 1e6;
        } else if (prof == PROF_TRAPEZOID_PERIOD) {
          d_cur = steps_acc ? (d_start + (int32_t)((int64_t)slope_acc * (int64_t)i_in_phase)) : d_cruise;
          dt_sec = d_cur / 1e6;
        } else { // PROF_CONSTACC_SPEED
          dt_sec = step_interval_constacc(v_cur, a_acc);
        }
        break;

      case CRUISE:
        if (prof == PROF_CONSTACC_SPEED) {
          dt_sec = (v_cruise_s > 1e-9) ? (1.0 / v_cruise_s) : 0.0;
        } else {
          d_cur = d_cruise;
          dt_sec = d_cur / 1e6;
        }
        break;

      case DECEL:
        if (prof == PROF_S_CURVE) {
          if (steps_decel == 0) { d_cur = d_end; }
          else {
            float t = (float)i_in_phase / (float)steps_decel;
            float s = smooth5(t);
            d_cur = (int32_t)lrintf((1.0f - s) * (float)d_cruise + s * (float)d_end);
          }
          dt_sec = d_cur / 1e6;
        } else if (prof == PROF_TRAPEZOID_PERIOD) {
          d_cur = steps_decel ? (d_cruise + (int32_t)((int64_t)slope_dec * (int64_t)i_in_phase)) : d_end;
          dt_sec = d_cur / 1e6;
        } else { // PROF_CONSTACC_SPEED
          dt_sec = step_interval_constacc(v_cur, a_dec);
        }
        break;

      default: break;
    }

    // Convert to µs and pack
    if (d_cur < 0) d_cur = 0; // relevant for non-const-acc paths
    const uint32_t PULSE_US = 20;
    uint32_t dt_us = (prof == PROF_CONSTACC_SPEED) 
                      ? (uint32_t)llround(dt_sec * 1e6)
                      : (uint32_t)d_cur;
    uint32_t extra = (dt_us > PULSE_US) ? (dt_us - PULSE_US) : 0u;

    out[n++] = make_cmd(dx, dy, dz, sxp, syp, szp, extra);
    total_steps_generated++;

    // Advance phase
    i_in_phase++;

    switch (phase) {
      case ACCEL:
        if (i_in_phase >= steps_acc) {
          phase = (steps_cruise ? CRUISE : (steps_decel ? DECEL : DONE));
          i_in_phase = 0;
          if (prof == PROF_CONSTACC_SPEED) v_cur = v_cruise_s; // snap to boundary
          d_cur = d_cruise;
        }
        break;
      case CRUISE:
        if (i_in_phase >= steps_cruise) {
          phase = (steps_decel ? DECEL : DONE);
          i_in_phase = 0;
          if (prof == PROF_CONSTACC_SPEED) v_cur = v_cruise_s;
          d_cur = d_cruise;
        }
        break;
      case DECEL:
        if (i_in_phase >= steps_decel) {
          phase = DONE;
          if (prof == PROF_CONSTACC_SPEED) v_cur = v_end_s; // arrive at end speed
          d_cur = d_end;
        }
        break;
      default: break;
    }
  }
  return n;
}


TrapS g_gen;  // definitie hier, waar TrapS bekend is

// ---- DMA/IRQ ----
extern "C" void __isr dma_complete_handler(){
  if(dma_channel_get_irq0_status(dma_channel)){
    dma_channel_acknowledge_irq0(dma_channel);
    update_realtime_positions_from_dma();
    dma_move_sent_prev += dma_active_len;
    dma_processed_in_buf = 0;
    dma_active_len = 0;

    if(g_gen.phase != TrapS::DONE){
      size_t n = g_gen.fill(command_buffer, BUFFER_SIZE);
      if(n>0){
        dma_active_len = n;
        dma_channel_set_read_addr(dma_channel, command_buffer, false);
        dma_channel_set_trans_count(dma_channel, n, true);
      }else{
        motion_active=false; motion_complete=true;
      }
    }else{
      motion_active=false; motion_complete=true; move_done_irq_flag=true;
    }
  }
}

void init_dma(){
  if(dma_channel>=0) dma_channel_unclaim(dma_channel);
  dma_channel = dma_claim_unused_channel(true);
  uint dreq = pio_get_dreq(pio, sm, true);

  dma_channel_config cfg = dma_channel_get_default_config(dma_channel);
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
  channel_config_set_read_increment(&cfg, true);
  channel_config_set_write_increment(&cfg, false);
  channel_config_set_dreq(&cfg, dreq);

  dma_channel_configure(dma_channel, &cfg, &pio->txf[sm], command_buffer, 0, false);
  dma_channel_set_irq0_enabled(dma_channel, true);
  irq_set_exclusive_handler(DMA_IRQ_0, dma_complete_handler);
  irq_set_enabled(DMA_IRQ_0, true);
}

// ---- stop / start ----
void stop_motion_soft(){
  if(dma_channel<0) return;
  while(dma_channel_is_busy(dma_channel)){ update_realtime_positions_from_dma(); tight_loop_contents(); }
  update_realtime_positions_from_dma();
  while(pio_sm_get_tx_fifo_level(pio, sm)!=0) tight_loop_contents();
  dma_channel_acknowledge_irq0(dma_channel);
  motion_active=false;
}

void stop_motion_force(){
  if(dma_channel<0){
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);
    motion_active=false; motion_complete=false;
    return;
  }
  update_realtime_positions_from_dma();
  dma_channel_set_irq0_enabled(dma_channel, false);
  dma_hw->ints0 = (1u<<dma_channel);
  dma_hw->ch[dma_channel].ctrl_trig &= ~DMA_CH0_CTRL_TRIG_EN_BITS;
  __dmb();
  dma_channel_abort(dma_channel);
  dma_hw->ints0 = (1u<<dma_channel);

  pio_sm_set_enabled(pio, sm, false);
  pio_sm_clear_fifos(pio, sm);
  pio_sm_restart(pio, sm);

  dma_channel_set_irq0_enabled(dma_channel, true);
  motion_active=false; motion_complete=false;
}

void start_next_move(){
  if(motion_active || is_queue_empty()) return;
  LineMove next_move;
  if(!dequeue_move(&next_move)) return;

  g_gen.init(next_move, realtime_X, realtime_Y, realtime_Z);
  if(g_gen.phase == TrapS::DONE){ start_next_move(); return; }

  size_t n = g_gen.fill(command_buffer, BUFFER_SIZE);
  dma_move_sent_prev=0; dma_processed_in_buf=0; dma_active_len=n;

  if(n==0){ start_next_move(); return; }

  dma_channel_set_read_addr(dma_channel, command_buffer, false);
  dma_channel_set_trans_count(dma_channel, n, false);
  dma_channel_acknowledge_irq0(dma_channel);

  if (_debug){
    SD.print("Move requested: X"); SD.print(next_move.sx);
    SD.print(" from "); SD.println(realtime_X);
    SD.print("Started move with "); SD.print(n); SD.println(" commands");
  }
  pio_sm_set_enabled(pio, sm, true);
  motion_active=true; motion_complete=false;
  dma_channel_start(dma_channel);
}

void emergency_stop(){
  stop_motion_force();
  clear_queue();
}

uint32_t planner_total_steps_generated(){
  return g_gen.total_steps_generated;
}