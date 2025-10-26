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

// ---- planner (S-curve, Bresenham) ----
struct TrapS {
  uint32_t Nx,Ny,Nz,Nmax; Axis master; bool dx,dy,dz;
  uint32_t steps_acc,steps_cruise,steps_decel;
  int32_t d_start,d_cruise,d_end,d_cur; enum Phase{ACCEL,CRUISE,DECEL,DONE} phase; uint32_t i_in_phase;
  int32_t ex,ey,ez; uint32_t total_steps_generated=0;

  static inline float smooth5(float t){
    if(t<=0) return 0; if(t>=1) return 1;
    return t*t*t*(10.0f + t*(-15.0f + 6.0f*t));
  }

  void init(const LineMove& m,int32_t cx,int32_t cy,int32_t cz){
    int32_t dxs=m.sx-cx, dys=m.sy-cy, dzs=m.sz-cz;
    Nx=abs(dxs); Ny=abs(dys); Nz=abs(dzs); Nmax = max(Nx, max(Ny,Nz));
    if(!Nmax){ phase=DONE; return; }
    dx=(dxs>=0); dy=(dys>=0); dz=(dzs>=0);
    master = (Nmax==Nx)?AXIS_X:((Nmax==Ny)?AXIS_Y:AXIS_Z);

    steps_acc = (uint32_t)roundf(m.accel_frac*Nmax);
    steps_decel = (uint32_t)roundf(m.decel_frac*Nmax);
    if(steps_acc+steps_decel > Nmax){ steps_acc=Nmax/2; steps_decel=Nmax-steps_acc; }
    steps_cruise = Nmax - steps_acc - steps_decel;

    d_start=(int32_t)m.d_start_us; d_cruise=(int32_t)m.d_cruise_us; d_end=(int32_t)m.d_end_us;
    phase = steps_acc?ACCEL:(steps_cruise?CRUISE:(steps_decel?DECEL:DONE));
    i_in_phase=0; d_cur=d_start; ex=ey=ez=-(int32_t)Nmax/2; total_steps_generated=0;
  }

  size_t fill(uint32_t* out, size_t cap){
    size_t n=0; if(phase==DONE) return 0;
    while(n<cap && phase!=DONE){
      bool sxp=false,syp=false,szp=false;
      switch(master){
        case AXIS_X: sxp=true; ey+=Ny; if(ey>=0){ey-=Nmax; syp=true;} ez+=Nz; if(ez>=0){ez-=Nmax; szp=true;} break;
        case AXIS_Y: syp=true; ex+=Nx; if(ex>=0){ex-=Nmax; sxp=true;} ez+=Nz; if(ez>=0){ez-=Nmax; szp=true;} break;
        case AXIS_Z: szp=true; ex+=Nx; if(ex>=0){ex-=Nmax; sxp=true;} ey+=Ny; if(ey>=0){ey-=Nmax; syp=true;} break;
      }
      switch(phase){
        case ACCEL:{ float t=(steps_acc? (float)i_in_phase/(float)steps_acc : 1.0f);
                     float s=smooth5(t); d_cur=(int32_t)lrintf((1.0f-s)*d_start + s*d_cruise); } break;
        case CRUISE: d_cur=d_cruise; break;
        case DECEL:{ float t=(steps_decel? (float)i_in_phase/(float)steps_decel : 1.0f);
                     float s=smooth5(t); d_cur=(int32_t)lrintf((1.0f-s)*d_cruise + s*d_end); } break;
        default: break;
      }
      if(d_cur<0) d_cur=0;
      const uint32_t PULSE_US=20;
      uint32_t extra = ( (uint32_t)d_cur>PULSE_US ) ? ((uint32_t)d_cur-PULSE_US) : 0u;
      out[n++] = make_cmd(dx,dy,dz, sxp,syp,szp, extra);
      total_steps_generated++;

      i_in_phase++;
      switch(phase){
        case ACCEL:  if(i_in_phase>=steps_acc) { phase=(steps_cruise?CRUISE:(steps_decel?DECEL:DONE)); i_in_phase=0; } break;
        case CRUISE: if(i_in_phase>=steps_cruise){ phase=(steps_decel?DECEL:DONE); i_in_phase=0; } break;
        case DECEL:  if(i_in_phase>=steps_decel) { phase=DONE; } break;
        default: break;
      }
    }
    return n;
  }
};

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

#if DEBUG
  Serial.print("Move requested: X"); Serial.print(next_move.sx);
  Serial.print(" from "); Serial.println(realtime_X);
  Serial.print("Started move with "); Serial.print(n); Serial.println(" commands");
#endif

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

