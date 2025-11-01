#include <Arduino.h>
extern "C" {
  #include "hardware/gpio.h"
}
#include "config.h"
#include "state.h"
#include "motion.h"
#include "queue.h"

volatile bool homing_triggered=false;
volatile uint triggered_sensor=0;
volatile bool homing_enabled=false;

static void homing_isr_cb(uint gpio, uint32_t events){
  if(homing_enabled && (events & GPIO_IRQ_LEVEL_LOW)){
    triggered_sensor=gpio; homing_triggered=true;
    emergency_stop();
    homing_enabled=false;
    gpio_set_irq_enabled(HOME_X_PIN, GPIO_IRQ_LEVEL_LOW, false);
    gpio_set_irq_enabled(HOME_Y_PIN, GPIO_IRQ_LEVEL_LOW, false);
    //gpio_set_irq_enabled(HOME_Z_PIN, GPIO_IRQ_LEVEL_LOW, false);
  }
}

void setup_homing_sensors(){
  gpio_init(HOME_X_PIN); gpio_set_dir(HOME_X_PIN, GPIO_IN); gpio_pull_up(HOME_X_PIN);
  gpio_init(HOME_Y_PIN); gpio_set_dir(HOME_Y_PIN, GPIO_IN); gpio_pull_up(HOME_Y_PIN);
  //gpio_init(HOME_Z_PIN); gpio_set_dir(HOME_Z_PIN, GPIO_IN); gpio_pull_up(HOME_Z_PIN);
  gpio_set_irq_enabled_with_callback(HOME_X_PIN, GPIO_IRQ_LEVEL_LOW, false, &homing_isr_cb);
  gpio_set_irq_enabled(HOME_Y_PIN, GPIO_IRQ_LEVEL_LOW, false);
  //gpio_set_irq_enabled(HOME_Z_PIN, GPIO_IRQ_LEVEL_LOW, false);
  homing_enabled=false;
}

void enable_homing_sensors(){
  homing_enabled=true;
  gpio_set_irq_enabled(HOME_X_PIN, GPIO_IRQ_LEVEL_LOW, true);
  gpio_set_irq_enabled(HOME_Y_PIN, GPIO_IRQ_LEVEL_LOW, true);
  //gpio_set_irq_enabled(HOME_Z_PIN, GPIO_IRQ_LEVEL_LOW, true);
  SD.println("Homing sensors ENABLED");
}

void disable_homing_sensors(){
  homing_enabled=false;
  gpio_set_irq_enabled(HOME_X_PIN, GPIO_IRQ_LEVEL_LOW, false);
  gpio_set_irq_enabled(HOME_Y_PIN, GPIO_IRQ_LEVEL_LOW, false);
  //gpio_set_irq_enabled(HOME_Z_PIN, GPIO_IRQ_LEVEL_LOW, false);
  SD.println("Homing sensors DISABLED");
}

void check_homing_safety(){
  if(homing_triggered){
    homing_triggered=false;
    if (_debug){
      SD.println("\n!!! EMERGENCY STOP - Homing Sensor Triggered !!!");
      SD.print("Sensor on GPIO: "); SD.println(triggered_sensor);
      SD.print("Stopped at X:"); SD.print(realtime_X);
      SD.print(" Y:"); SD.print(realtime_Y);
      SD.print(" Z:"); SD.println(realtime_Z);
      SD.println("All motion stopped and queue cleared");
    }
  }
}
