//
// Created by nhasbun on 06-11-2020.
//

/**
 * Testing continuous led strip calculation module.
 *
 * Leds should be turned on in a slice mode using a intensity window. More info
 * look for ***led_array_control_schematic.png*** in docs.
 */

#include <Arduino.h>
#include <unity.h>
#include "LedCalculator.h"

LedCalculator ledCalculator;


void setUp() {
    ledCalculator = LedCalculator();
}


void test_basic() {
    const uint8_t num_leds = 5;
    const uint16_t led_window = 160;

    // Test num leds
    TEST_ASSERT_EQUAL(num_leds, ledCalculator.get_num_leds());

    // Test correct ids and start positions
    LedProperties* ledProperties = ledCalculator.get_led_properties();
    // LedProperties* ledProperties_start = ledProperties;

    for (uint8_t i = 0; i < num_leds; i++) {

        TEST_ASSERT_EQUAL(i, ledProperties->id);
        TEST_ASSERT_EQUAL(i*led_window, ledProperties->start_position);
        ledProperties++;
    }
}


void test_led_calculation() {

    LedProperties* ledProperties = ledCalculator.get_led_properties();

    // Testing some intensity inputs and windows position
    ledCalculator.set_input_intensity(100);
    ledCalculator.set_intensity_position(80);
    ledCalculator.calculate();

    TEST_ASSERT_EQUAL(100, (ledProperties[0].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[1].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[2].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[3].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[4].intensity));

    ledCalculator.set_input_intensity(50);
    ledCalculator.set_intensity_position(80);
    ledCalculator.calculate();

    TEST_ASSERT_EQUAL(50, (ledProperties[0].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[1].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[2].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[3].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[4].intensity));

    ledCalculator.set_input_intensity(100);
    ledCalculator.set_intensity_position(240);
    ledCalculator.calculate();

    TEST_ASSERT_EQUAL(0, (ledProperties[0].intensity));
    TEST_ASSERT_EQUAL(100, (ledProperties[1].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[2].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[3].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[4].intensity));

    ledCalculator.set_input_intensity(100);
    ledCalculator.set_intensity_position(160);
    ledCalculator.calculate();

    TEST_ASSERT_EQUAL(50, (ledProperties[0].intensity));
    TEST_ASSERT_EQUAL(50, (ledProperties[1].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[2].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[3].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[4].intensity));

    ledCalculator.set_input_intensity(100);
    ledCalculator.set_intensity_position(640);
    ledCalculator.calculate();

    TEST_ASSERT_EQUAL(0, (ledProperties[0].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[1].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[2].intensity));
    TEST_ASSERT_EQUAL(50, (ledProperties[3].intensity));
    TEST_ASSERT_EQUAL(50, (ledProperties[4].intensity));

    // Test handling values outside range
    ledCalculator.set_input_intensity(100);
    ledCalculator.set_intensity_position(0);
    ledCalculator.calculate();

    TEST_ASSERT_EQUAL(100, (ledProperties[0].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[1].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[2].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[3].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[4].intensity));

    ledCalculator.set_input_intensity(100);
    ledCalculator.set_intensity_position(800);
    ledCalculator.calculate();

    TEST_ASSERT_EQUAL(0, (ledProperties[0].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[1].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[2].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[3].intensity));
    TEST_ASSERT_EQUAL(100, (ledProperties[4].intensity));

    ledCalculator.set_input_intensity(100);
    ledCalculator.set_intensity_position(0);
    ledCalculator.calculate();

    TEST_ASSERT_EQUAL(100, (ledProperties[0].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[1].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[2].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[3].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[4].intensity));

    // Changing default intensity window
    ledCalculator.set_intensity_diameter(160);

    // Intensity window now covers 2 leds
    ledCalculator.set_input_intensity(100);
    ledCalculator.set_intensity_position(160);
    ledCalculator.calculate();

    TEST_ASSERT_EQUAL(100, (ledProperties[0].intensity));
    TEST_ASSERT_EQUAL(100, (ledProperties[1].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[2].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[3].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[4].intensity));

    ledCalculator.set_input_intensity(100);
    ledCalculator.set_intensity_position(640);
    ledCalculator.calculate();

    TEST_ASSERT_EQUAL(0, (ledProperties[0].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[1].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[2].intensity));
    TEST_ASSERT_EQUAL(100, (ledProperties[3].intensity));
    TEST_ASSERT_EQUAL(100, (ledProperties[4].intensity));

    // Changing default intensity window
    ledCalculator.set_intensity_diameter(120);

    // Intensity window now covers 1.5 leds
    ledCalculator.set_input_intensity(100);
    ledCalculator.set_intensity_position(120);
    ledCalculator.calculate();

    TEST_ASSERT_EQUAL(100, (ledProperties[0].intensity));
    TEST_ASSERT_EQUAL(50, (ledProperties[1].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[2].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[3].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[4].intensity));

    ledCalculator.set_input_intensity(100);
    ledCalculator.set_intensity_position(680);
    ledCalculator.calculate();

    TEST_ASSERT_EQUAL(0, (ledProperties[0].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[1].intensity));
    TEST_ASSERT_EQUAL(0, (ledProperties[2].intensity));
    TEST_ASSERT_EQUAL(50, (ledProperties[3].intensity));
    TEST_ASSERT_EQUAL(100, (ledProperties[4].intensity));
}


void setup() {
    delay(2000);

    UNITY_BEGIN();
    RUN_TEST(test_basic);
    RUN_TEST(test_led_calculation);
    UNITY_END();
}

void loop() {
    delay(10e6);
}