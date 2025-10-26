#include <Arduino.h>
#include "config.h"
#include "state.h"
#include "units.h"
#include "motion.h"
#include "homing.h"
#include "gcode.h"

void setup(){
  Serial.begin();
  uint32_t t0=millis(); while(!Serial && millis()-t0<5000) delay(10);

#if DEBUG
  Serial.println("Pico Stepper Controller - TRUE Real-time DMA Position Tracking");
#endif

  pio_stepper_init();
  init_dma();
  setup_homing_sensors();

  realtime_X=realtime_Y=realtime_Z=0;

#if DEBUG
  Serial.println("System ready. Commands:");
  Serial.println("G0/G1 X.. Y.. Z.. F..  | G28 | G92 X.. Y.. Z.. | M114 | M0|M2|M110|M111 | M120|M121");
  Serial.println("G101 steps | G102 mm | G103 deg");
#endif
}

void loop(){
  static String buf;

  check_homing_safety();

  if(motion_complete){
    motion_complete=false;
    update_realtime_positions_from_dma();
    if(move_done_irq_flag){
      move_done_irq_flag=false;
#if DEBUG
      Serial.print("Move completed. Total steps generated: ");
      Serial.println(planner_total_steps_generated());
#endif
    }
    start_next_move();
  }

  update_realtime_display();

#if DEBUG
  while(Serial.available()){
    char c=(char)Serial.read();
    if(c=='\n'||c=='\r'){
      if(buf.length()>0){ Serial.println(); handleLine(buf); buf=""; }
    }else if(c>=32 && c<127){
      buf+=c; if(buf.length()>100) buf.remove(0,1);
    }
  }
#else
  while(Serial1.available()){
    char c=(char)Serial1.read();
    if(c=='\n'||c=='\r'){
      if(buf.length()>0){ handleLine(buf); buf=""; }
    }else if(c>=32 && c<127){
      buf+=c; if(buf.length()>100) buf.remove(0,1);
    }
  }
#endif

  delay(1);
}
