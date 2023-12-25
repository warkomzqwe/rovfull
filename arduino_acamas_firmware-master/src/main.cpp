/**
 * @file main.cpp
 * @brief Main routine.
 */

#include <Arduino.h>
#include "api_modbus.h"
#include "api_lights.h"
#include "api_grabber.h"


/**
 * \def MODBUS_MAP_PERIOD
 * Time between modbus registers updates with sensors, actuators, etc. in [ms]
 */
#define MODBUS_MAP_PERIOD 10


auto current_time = 0UL;
auto modbus_map_timestamp = 0UL;


void setup() {
    modbus_init();
    lights_init();
    grabber_init();
}


void loop() {
    modbus_update();

    current_time = millis();

    if (current_time - modbus_map_timestamp > MODBUS_MAP_PERIOD) {
        modbus_map_inputs();
        modbus_map_outputs();
        modbus_map_timestamp = current_time;
    }
}