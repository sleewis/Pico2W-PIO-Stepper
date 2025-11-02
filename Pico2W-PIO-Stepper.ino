#include <Arduino.h>
#include "config.h"
#include "state.h"
#include "units.h"
#include "motion.h"
#include "homing.h"
#include "gcode.h"

void setup(){
  _debug = false; _usb = false;
  Serial.begin();
  uint32_t t0=millis(); while(!Serial && millis()-t0<5000) delay(10);

  gpio_set_function(16, UART_FUNCSEL_NUM(uart0, 16)); // tx
  gpio_set_function(17, UART_FUNCSEL_NUM(uart0, 17)); // rx
  uart_set_hw_flow (uart0, false, false);
  Serial1.begin(115200);
  t0 = millis(); while (!Serial1 && millis() - t0 < 5000) delay(10);

  SD.setOutput(_usb ? static_cast<Stream&>(Serial) : static_cast<Stream&>(Serial1));
  //g_units=UNITS_STEPS;
  //g_units=UNITS_MM;
  g_units=UNITS_DEG;

  pio_stepper_init();
  init_dma();
  setup_homing_sensors();

  realtime_X=realtime_Y=realtime_Z=0;

  // Kleur aan (true) of zonder kleur (false)
  bootBanner::print_boot_banner(true, "v0.3.0");
}

void loop(){
  static String buf;

  check_homing_safety();

  if(motion_complete){
    motion_complete=false;
    update_realtime_positions_from_dma();
    if(move_done_irq_flag){
      move_done_irq_flag=false;
      if (_debug) {
        SD.print("Move completed. Total steps generated: ");
        SD.println(planner_total_steps_generated());
      }
    }
    start_next_move();
  }

  update_realtime_display();

  while(Serial.available() || Serial1.available()){
    char c;
    bool usb = Serial.available();
    if (usb != _usb){ _usb = usb; SD.setOutput(_usb ? static_cast<Stream&>(Serial) : static_cast<Stream&>(Serial1));}

    if (usb) c=(char)Serial.read(); else c=(char)Serial1.read();
    if(c=='\n'||c=='\r'){
      if(buf.length()>0){ Serial.println(buf); handleLine(buf); buf=""; }
    }else if(c>=32 && c<127){
      buf+=c; if(buf.length()>100) buf.remove(0,1);
    }
  }

  delay(1);
}
