/**
 * @file api_lights.cpp
 * @brief api_lights implementation
 */

#include "api_lights.h"
#include "Arduino.h"


#define PIN_LIGHTS_ENABLE 2
#define PIN_LIGHT_1 4
#define PIN_LIGHT_2 5
#define PIN_LIGHT_3 6


void lights_init() {
    pinMode(PIN_LIGHTS_ENABLE, OUTPUT);
    pinMode(PIN_LIGHT_1, OUTPUT);
    pinMode(PIN_LIGHT_2, OUTPUT);
    pinMode(PIN_LIGHT_3, OUTPUT);

    lights_off();
}


void lights_on() {
    digitalWrite(PIN_LIGHTS_ENABLE, HIGH);
    digitalWrite(PIN_LIGHT_1, HIGH);
    digitalWrite(PIN_LIGHT_2, HIGH);
    digitalWrite(PIN_LIGHT_3, HIGH);
}


void lights_off() {
    digitalWrite(PIN_LIGHTS_ENABLE, LOW);
    digitalWrite(PIN_LIGHT_1, LOW);
    digitalWrite(PIN_LIGHT_2, LOW);
    digitalWrite(PIN_LIGHT_3, LOW);
}
