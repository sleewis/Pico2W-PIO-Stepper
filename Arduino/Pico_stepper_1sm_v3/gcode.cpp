#include "gcode.h"
#include "config.h"
#include "types.h"
#include "state.h"
#include "units.h"
#include "queue.h"
#include "motion.h"
#include "homing.h"

static float lastFeed_units_s = 2000.0f;

static inline int32_t clamp_steps(int32_t s){ return constrain(s, 0, 80000); }

void handleLine(const String& line){
  String s=line; s.trim(); s.toUpperCase(); if(!s.length()) return;

  if(s.startsWith("G101")||s.startsWith("G102")||s.startsWith("G103")){
    if(s.startsWith("G101")) g_units=UNITS_STEPS;
    if(s.startsWith("G102")) g_units=UNITS_MM;
    if(s.startsWith("G103")) g_units=UNITS_DEG;
#if DEBUG
    Serial.print("Units are now in "); Serial.println(units_label());
#endif
    return;
  }

  if(s.startsWith("G0") || s.startsWith("G1")){
    int idx;
    float ux=steps_to_units_X(realtime_X);
    float uy=steps_to_units_Y(realtime_Y);
    float uz=steps_to_units_Z(realtime_Z);
    float F_units_s=lastFeed_units_s;

    if((idx=s.indexOf('X'))>=0) ux=s.substring(idx+1).toFloat();
    if((idx=s.indexOf('Y'))>=0) uy=s.substring(idx+1).toFloat();
    if((idx=s.indexOf('Z'))>=0) uz=s.substring(idx+1).toFloat();
    if((idx=s.indexOf('F'))>=0) F_units_s=max(1.0f, s.substring(idx+1).toFloat());

    // clamp per as/unit
    float ux_req=ux, uy_req=uy, uz_req=uz;
    ux=apply_caps_X(ux); uy=apply_caps_Y(uy); uz=apply_caps_Z(uz);
#if DEBUG
    if(ux!=ux_req){ Serial.print("X capped to "); Serial.print(ux); Serial.print(" "); Serial.println(units_label()); }
    if(uy!=uy_req){ Serial.print("Y capped to "); Serial.print(uy); Serial.print(" "); Serial.println(units_label()); }
    if(uz!=uz_req){ Serial.print("Z capped to "); Serial.print(uz); Serial.print(" "); Serial.println(units_label()); }
#endif

    int32_t tx = clamp_steps(units_to_steps_X(ux));
    int32_t ty = clamp_steps(units_to_steps_Y(uy));
    int32_t tz = clamp_steps(units_to_steps_Z(uz));

    int32_t dxs=abs(tx-realtime_X), dys=abs(ty-realtime_Y), dzs=abs(tz-realtime_Z);
    float kx=steps_per_unit_X(), ky=steps_per_unit_Y(), kz=steps_per_unit_Z();
    float steps_per_unit_for_master = (dxs>=dys && dxs>=dzs)?kx:((dys>=dzs)?ky:kz);
    float f_steps_s = F_units_s * steps_per_unit_for_master;

    LineMove m; m.sx=tx; m.sy=ty; m.sz=tz;
    if(s.startsWith("G0")){ m.d_start_us=1500; m.d_cruise_us=600; m.d_end_us=1500; }
    else { m.d_cruise_us=(uint32_t)roundf(1e6f/f_steps_s); m.d_start_us=(uint32_t)roundf(m.d_cruise_us*3.5f); m.d_end_us=m.d_start_us; lastFeed_units_s=F_units_s; }
    m.accel_frac=0.60f; m.decel_frac=0.60f;

    if(queue_move(m)){
#if DEBUG
      Serial.print("Queued move to X:"); Serial.print((g_units==UNITS_STEPS)?(double)tx:(double)ux);
      Serial.print(" Y:"); Serial.print((g_units==UNITS_STEPS)?(double)ty:(double)uy);
      Serial.print(" Z:"); Serial.print((g_units==UNITS_STEPS)?(double)tz:(double)uz);
      Serial.print(" | Units: "); Serial.println(units_label());
#endif
      if(!motion_active) start_next_move();
    } else {
#if DEBUG
      Serial.println("ERROR: Move queue full!");
#endif
    }
    return;
  }

  if(s.startsWith("G28")){
    LineMove m; m.sx=0; m.sy=0; m.sz=0;
    m.d_start_us=2000; m.d_cruise_us=1000; m.d_end_us=2000; m.accel_frac=0.35f; m.decel_frac=0.35f;
    if(queue_move(m)){
#if DEBUG
      Serial.println("Queued homing move to origin (0,0,0 steps)");
#endif
      if(!motion_active) start_next_move();
    } else {
#if DEBUG
      Serial.println("ERROR: Queue full for homing move");
#endif
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
#if DEBUG
    Serial.print("Position set (capped) to X:"); Serial.print(ux);
    Serial.print(" Y:"); Serial.print(uy); Serial.print(" Z:"); Serial.println(uz);
#endif
    return;
  }

  if(s.startsWith("M114")){
    int32_t sx,sy,sz; read_positions_atomic(sx,sy,sz);
    float ux=steps_to_units_X(sx), uy=steps_to_units_Y(sy), uz=steps_to_units_Z(sz);
#if DEBUG
    Serial.print("POS: X:"); serial_print_units(ux); Serial.print(" Y:"); serial_print_units(uy);
    Serial.print(" Z:"); serial_print_units(uz);
    Serial.print(" | Units: "); Serial.print(units_label());
    Serial.print(" | Moving: "); Serial.print(motion_active?"YES":"NO");
    Serial.print(" | Queue: "); Serial.println(get_queue_count());
#endif
    return;
  }

  if(s.startsWith("M0")){
    stop_motion_force(); clear_queue();
  #if DEBUG
    Serial.println("Stopped motion and cleared queue");
  #endif
    return;
  }
  
  if(s.startsWith("M2")){
    stop_motion_force();
  #if DEBUG
    Serial.println("Stopped motion (queue preserved)");
  #endif
    return;
  }
  
  if(s.startsWith("M110")){
    clear_queue();
  #if DEBUG
    Serial.println("Queue cleared");
  #endif
    return;
  }
  
  if(s.startsWith("M111")){
  #if DEBUG
    Serial.print("Queue: "); Serial.print(get_queue_count()); Serial.println("/32 moves");
  #endif
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
#if DEBUG
  if(line!=prev) Serial.println(line);
  Serial.flush();
#endif
  prev=line;
}
