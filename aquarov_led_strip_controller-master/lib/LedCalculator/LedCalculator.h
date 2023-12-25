//
// Created by nhasbun on 06-11-2020.
//

/*
 * Module builds a focus light area over a led strip and deliver intensity
 * required for independent leds.
 *
 * Calculation is done using a spacial representation of leds over microseconds
 * windows (suitable for RC/ROVs communications) and a slicing intensity window.
 * Graphical representation could be useful, search docs for
 * **led_array_control_schematic.png** file.
 *
 */

#ifndef LED_INTENSITY_CALCULATOR_LEDCALCULATOR_H
#define LED_INTENSITY_CALCULATOR_LEDCALCULATOR_H

#include "Arduino.h"

typedef struct LedProperties {
    uint8_t id = 0;
    uint8_t intensity = 0;
    uint16_t start_position = 0;
    uint16_t end_position = 0;
} LedProperties;


class LedCalculator {

private:
    LedProperties *ledProperties_p;
    uint8_t num_leds = 0;
    uint16_t total_window = 0;
    uint16_t led_window_size = 0;
    uint16_t intensity_diameter = 0;
    uint16_t intensity_position = 0;
    uint16_t intensity_start_pos = 0;
    uint16_t intensity_end_pos = 0;
    uint8_t input_intensity = 0;

public:
    LedCalculator();
    uint8_t get_num_leds() const;
    LedProperties* get_led_properties();

    void set_intensity_position(uint16_t pos);
    void set_intensity_diameter(uint16_t diameter);
    void set_input_intensity(uint8_t intensity);
    void calculate();
};

#endif //LED_INTENSITY_CALCULATOR_LEDCALCULATOR_H
