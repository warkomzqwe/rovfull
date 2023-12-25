//
// Created by nhasbun on 09-11-2020.
//

#ifndef AQUAROV_LED_STRIP_CONTROLLER_MAIN_H
#define AQUAROV_LED_STRIP_CONTROLLER_MAIN_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "LedCalculator.h"


void on_intensity_interrupt();
void on_angle_interrupt();
/**
 * Limit pwm value between specified ranges
 * @param bottom bottom value, ex 0us
 * @param top top value, 800us
 * @param raw_pwm raw pwm value to be limited
 * @return limited pwm value
 */
float limit_pwm(float bottom, float top, float raw_pwm);

#endif //AQUAROV_LED_STRIP_CONTROLLER_MAIN_H
