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

#include "LedCalculator.h"

/// Constructor class.
LedCalculator::LedCalculator() {

    num_leds = 5;
    total_window = 800;
    led_window_size = 160;
    intensity_diameter = 80;
    input_intensity = 0;
    intensity_start_pos = 0;
    intensity_end_pos = 0;
    intensity_position = intensity_start_pos + intensity_diameter;

    ledProperties_p = (LedProperties*)calloc(num_leds, sizeof(LedProperties));

    for (uint8_t i = 0; i < num_leds; i++) {

        (ledProperties_p + i)->id = i;
        (ledProperties_p + i)->start_position = i * led_window_size;
        (ledProperties_p + i)->end_position = (i + 1) * led_window_size;
    }
}


uint8_t LedCalculator::get_num_leds() const {
    return num_leds;
}


LedProperties *LedCalculator::get_led_properties() {
    return ledProperties_p;
}


void LedCalculator::set_intensity_position(uint16_t pos) {

    // Limiting intensity window position
    if (pos < intensity_diameter) {
        pos = intensity_diameter;
    }

    else if (pos > total_window - intensity_diameter) {
        pos = total_window - intensity_diameter;
    }

    else;

    intensity_position = pos;
    intensity_start_pos = pos - intensity_diameter;
    intensity_end_pos = pos + intensity_diameter;
}


void LedCalculator::set_intensity_diameter(uint16_t diameter) {
    intensity_diameter = diameter;
    intensity_position = intensity_start_pos + intensity_diameter;

    set_intensity_position(intensity_position);
}


void LedCalculator::set_input_intensity(uint8_t intensity) {

    // Limiting intensity
    if (intensity > 100) {
        intensity = 100;
    }

    input_intensity = intensity;
}


void LedCalculator::calculate() {

    for (uint8_t i = 0; i < num_leds; i++) {

        LedProperties* ledProperties = ledProperties_p + i;

        // Borderline cases intensity width doesn't overlap led window
        if (intensity_start_pos >= ledProperties->end_position) {

            ledProperties->intensity = 0;
            continue;
        }

        if (intensity_end_pos <= ledProperties->start_position) {

            ledProperties->intensity = 0;
            continue;
        }

        // At this point overlap is happening

        // Case 1
        // Intensity mark start before led window and stop after led window
        if (intensity_start_pos < ledProperties->start_position &&
            intensity_end_pos > ledProperties->end_position) {

            ledProperties->intensity = input_intensity;
            continue;
        }

        // Case 2
        // Intensity mark overlaps and starts after led window
        if (intensity_start_pos >= ledProperties->start_position) {

            float led_usage =
                    (float)(ledProperties->end_position - intensity_start_pos) *
                            (float)input_intensity / (float)led_window_size;
            ledProperties->intensity = ceil((double)led_usage);
            continue;
        }

        // Case 3
        // Intensity mark overlaps and starts before led window
        if (intensity_end_pos > ledProperties->start_position) {

            float led_usage =
                    float(intensity_end_pos - ledProperties->start_position) *
                    (float)input_intensity / (float)led_window_size;
            ledProperties->intensity = ceil((double)led_usage);
            continue;
        }
    }
}
