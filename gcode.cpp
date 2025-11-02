#include "gcode.h"
#include "config.h"
#include "types.h"
#include "state.h"
#include "units.h"
#include "queue.h"
#include "motion.h"
#include "homing.h"

static float lastFeed_units_s = 2000.0f;

static inline int32_t clamp_steps(int32_t s){ return constrain(s, -80000, 80000); }

void handleLine(const String& line){
  String s=line; s.trim(); s.toUpperCase(); if(!s.length()) return;

  if(s.startsWith("BAUD") || s.startsWith("BAUDRATE")){
    int idx;
    uint32_t b = 0;
    if((idx=s.indexOf(' '))>=0) b=s.substring(idx+1).toInt();
    if (Serial1 && b >= 9600 && b <= 115200) {
      uart_set_baudrate(uart0, b);
      if (_debug) SD.println(b);
    }
    return;
  }

  if(s.startsWith("DEBUG")){
    _debug = !_debug;
    SD.println(_debug ? "DEBUG ON" : "DEBUG OFF");
    return;
  }

  if(s.startsWith("G101")||s.startsWith("G102")||s.startsWith("G103")){
    if(s.startsWith("G101")) g_units=UNITS_STEPS;
    if(s.startsWith("G102")) g_units=UNITS_MM;
    if(s.startsWith("G103")) g_units=UNITS_DEG;
    SD.print("Units are now in "); SD.println(units_label());
    return;
  }

  if(s.startsWith("G0") || s.startsWith("G1") || s.startsWith("W")){
    int idx;
    float ux=steps_to_units_X(realtime_X);
    float uy=steps_to_units_Y(realtime_Y);
    float uz=steps_to_units_Z(realtime_Z);
    float F_units_s=lastFeed_units_s;

    if((idx=s.indexOf('X'))>=0) ux=s.substring(idx+1).toFloat();
    if((idx=s.indexOf('Y'))>=0) uy=s.substring(idx+1).toFloat();
    if((idx=s.indexOf('Z'))>=0) uz=s.substring(idx+1).toFloat();
    if((idx=s.indexOf('F'))>=0) F_units_s=max(1.0f, s.substring(idx+1).toFloat());

    // W is a GS232 command
    if((idx=s.indexOf('W'))>=0){
      ux=s.substring(idx+1).toFloat();
      if((idx=s.indexOf(' '))>=0) uy=s.substring(idx+1).toFloat();
    }

    // clamp per as/unit
    float ux_req=ux, uy_req=uy, uz_req=uz;
    ux=apply_caps_X(ux); uy=apply_caps_Y(uy); uz=apply_caps_Z(uz);
    if (_debug){
      if(ux!=ux_req){ SD.print("X capped to "); SD.print(ux); SD.print(" "); SD.println(units_label()); }
      if(uy!=uy_req){ SD.print("Y capped to "); SD.print(uy); SD.print(" "); SD.println(units_label()); }
      if(uz!=uz_req){ SD.print("Z capped to "); SD.print(uz); SD.print(" "); SD.println(units_label()); }
    }

    int32_t tx = clamp_steps(units_to_steps_X(ux));
    int32_t ty = clamp_steps(units_to_steps_Y(uy));
    int32_t tz = clamp_steps(units_to_steps_Z(uz));

    int32_t dxs=abs(tx-realtime_X), dys=abs(ty-realtime_Y), dzs=abs(tz-realtime_Z);
    float kx=steps_per_unit_X(), ky=steps_per_unit_Y(), kz=steps_per_unit_Z();
    float steps_per_unit_for_master = (dxs>=dys && dxs>=dzs)?kx:((dys>=dzs)?ky:kz);
    float f_steps_s = F_units_s * steps_per_unit_for_master;

    LineMove m; m.sx=tx; m.sy=ty; m.sz=tz;
    if(s.startsWith("G0") || s.startsWith("W")){ m.d_start_us=10'000; m.d_cruise_us=3000; m.d_end_us=10'000; }
    else { m.d_cruise_us=(uint32_t)roundf(1e6f/f_steps_s); m.d_start_us=(uint32_t)roundf(m.d_cruise_us*3.5f); m.d_end_us=m.d_start_us; lastFeed_units_s=F_units_s; }
    m.accel_frac=0.60f; m.decel_frac=0.60f;

    if(queue_move(m)){
      if (_debug){
        SD.print("Queued move to X:"); SD.print((g_units==UNITS_STEPS)?(double)tx:(double)ux);
        SD.print(" Y:"); SD.print((g_units==UNITS_STEPS)?(double)ty:(double)uy);
        SD.print(" Z:"); SD.print((g_units==UNITS_STEPS)?(double)tz:(double)uz);
        SD.print(" | Units: "); SD.println(units_label());
      }
      if(!motion_active) start_next_move();
    } else {
      if (_debug) SD.println("ERROR: Move queue full!");
    }
    return;
  }
  if(s.startsWith("G28")){
    LineMove m; m.sx=0; m.sy=0; m.sz=0;
    m.d_start_us=2000; m.d_cruise_us=1000; m.d_end_us=2000; m.accel_frac=0.35f; m.decel_frac=0.35f;
    if(queue_move(m)){
      if (_debug) SD.println("Queued homing move to origin (0,0,0 steps)");
      if(!motion_active) start_next_move();
    } else {
      if (_debug) SD.println("ERROR: Queue full for homing move");
    }
    return;
  }

  if(s.startsWith("G92")){
    stop_motion_soft();
    int idx; float ux=steps_to_units_X(realtime_X), uy=steps_to_units_Y(realtime_Y), uz=steps_to_units_Z(realtime_Z);
    if((idx=s.indexOf('X'))>=0) ux=s.substring(idx+1).toFloat();
    if((idx=s.indexOf('Y'))>=0) uy=s.substring(idx+1).toFloat();
    if((idx=s.indexOf('Z'))>=0) uz=s.substring(idx+1).toFloat();
    ux=apply_caps_X(ux); uy=apply_caps_Y(uy); uz=apply_caps_Z(uz);
    realtime_X=clamp_steps(units_to_steps_X(ux));
    realtime_Y=clamp_steps(units_to_steps_Y(uy));
    realtime_Z=clamp_steps(units_to_steps_Z(uz));
    if (_debug){
      SD.print("Position set (capped) to X:"); SD.print(ux);
      SD.print(" Y:"); SD.print(uy); SD.print(" Z:"); SD.println(uz);
    }
    return;
  }

  // C2 is a GS232 command
  if(s.startsWith("C2")){
    int32_t sx,sy,sz; read_positions_atomic(sx,sy,sz);
    g_units=UNITS_DEG;
    float ux=steps_to_units_X(sx), uy=steps_to_units_Y(sy), uz=steps_to_units_Z(sz);
    char buffer[48] = { };
    sprintf(buffer,"+%04d+%04d\r\n",(int32_t)lrintf(ux), (int32_t)lrintf(uy));   
    SD.print(buffer);
    return;
  }

  if(s.startsWith("M114")){
    int32_t sx,sy,sz; read_positions_atomic(sx,sy,sz);
    float ux=steps_to_units_X(sx), uy=steps_to_units_Y(sy), uz=steps_to_units_Z(sz);
    if (_debug){
      SD.print("POS: X:"); serial_print_units(ux); 
      SD.print(" Y:"); serial_print_units(uy);
      SD.print(" Z:"); serial_print_units(uz);
      SD.print(" | Units: "); SD.print(units_label());
      SD.print(" | Moving: "); SD.print(motion_active?"YES":"NO");
      SD.print(" | Queue: "); SD.println(get_queue_count());
    }
    return;
  }


  if(s.startsWith("M0")){
    stop_motion_force(); clear_queue();
    if (_debug) SD.println("Stopped motion and cleared queue");
    return;
  }
  
  if(s.startsWith("M2")){
    stop_motion_force();
    if (_debug) SD.println("Stopped motion (queue preserved)");
    return;
  }
  
  if(s.startsWith("M110")){
    clear_queue();
    if (_debug) SD.println("Queue cleared");
    return;
  }
  
  if(s.startsWith("M111")){
    if (_debug){ SD.print("Queue: "); SD.print(get_queue_count()); SD.println("/32 moves"); }
    return;
  }
  
  if(s.startsWith("M120")){ enable_homing_sensors(); return; }
  if(s.startsWith("M121")){ disable_homing_sensors(); return; }
}

void update_realtime_display(){
  static uint32_t last_display=0; static String prev="";
  if(millis()-last_display < 200) return; last_display=millis();
  update_realtime_positions_from_dma();

  int32_t x,y,z; read_positions_atomic(x,y,z);
  String line = String("POS: ") + toStringUnits(steps_to_units_X(x)) + ", " +
                toStringUnits(steps_to_units_Y(y)) + ", " +
                toStringUnits(steps_to_units_Z(z)) +
                (motion_active?" [MOVING]":" [IDLE]") +
                " Q:" + String(get_queue_count()) +
                " U:" + units_label();
  if (_debug){
    if(line!=prev) SD.println(line);
    SD.flush();
  }
  prev=line;
}
