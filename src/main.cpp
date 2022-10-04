/*
 * GPIO Wiring Table
 * 
 * Connect SBus:
 * SBus - GPIO25
 * and put 1kOhm between GPIO25 and GPIO26
 * It uses UART1
 * 
 * Servo Pins:
 * aileron_l - GPIO27
 * aileron_r - GPIO14
 * motor - GPIO12
 * elevator - GPIO13
 * rudder - GPIO15
 * flap_l - GPIO2
 * flap_r - GPIO4
 *
 * Default I2C lines for ESP32 are:
 * SCL - GPIO22
 * SDA - GPIO21
 * Those can be changed to any other pins via TwoWire Object
 * 
 * Connect GPS (Subject to change):
 * RX - GPIO35
 * TX - GPIO34
 * It uses UART2
 */

#include <Arduino.h>
#include "autopilot.h"

Autopilot autopilot;

void setup() {
    autopilot.setup();
}

void loop() {
    autopilot.loop();
}