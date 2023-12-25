/**
 * Module for control of a Led Strip focusing light on some areas. Light
 * intensity is controlled using pwm duty cycle via pca9685 module.
 *
 * We have desired led intensity and focus/angle information incoming as rc pwm.
 *
 */

#include "main.h"
#define fastDigitalRead(p_inputRegister, bitMask) ((*p_inputRegister & bitMask) ? HIGH : LOW)

/*
 * Parameters
 */
const uint8_t intensity_pin = 2;
const uint8_t angle_pin = 3;
const float alpha = 0.1f;
const float intensity_pwm_base = 1100.f;
const float intensity_pwm_top = 1900.f;
const float intensity_pwm_diff = intensity_pwm_top - intensity_pwm_base;
const float angle_pwm_base = 1100.f;
const float angle_pwm_top = 1900.f;
const float angle_pwm_diff = angle_pwm_top - angle_pwm_base;
const uint8_t action_period = 1;
const uint8_t pca_addr = 0x40;
const uint32_t pca_clock = 26290000;
const float pca_pwm_freq = 1600.f;
const uint16_t pca_pwm_register_max = 4095;
const uint8_t num_leds = 5;

boolean serial_com = false;
const uint32_t serial_baudrate = 115200;


/*
 * Global declarations
 */
// Fast digital read
byte intensity_pin_bitMask = 0;
byte angle_pin_bitMask = 0;
volatile byte* intensity_pin_register = nullptr;
volatile byte* angle_pin_register = nullptr;
volatile boolean intensity_falling_edge = false;
volatile boolean angle_falling_edge = false;

// PWM Measurement
// All measurements in microseconds
// Captured values are filtered using a first order low pass filter
volatile unsigned long intensity_start_pulse = 0UL;
volatile unsigned long intensity_end_pulse = 0UL;
volatile unsigned long angle_start_pulse = 0UL;
volatile unsigned long angle_end_pulse = 0UL;
uint16_t falling_count = 0;
float intensity_pwm_lpf = 0.0f;
float angle_pwm_lpf = 0.0f;

// PCA9685 Control
Adafruit_PWMServoDriver pwm;

// Led Calculator
LedCalculator led_calculator;


void setup() {

    // Activating Serial communication
    if (serial_com)
        Serial.begin(serial_baudrate);

    // Enable pin interrupts
    pinMode(intensity_pin, INPUT_PULLUP);
    pinMode(angle_pin, INPUT_PULLUP);
    interrupts();
    attachInterrupt(digitalPinToInterrupt(intensity_pin),
                    on_intensity_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(angle_pin),
                    on_angle_interrupt, CHANGE);

    // Setup fast digital read
    intensity_pin_bitMask = digitalPinToBitMask(intensity_pin);
    angle_pin_bitMask = digitalPinToBitMask(angle_pin);
    intensity_pin_register = portInputRegister(digitalPinToPort(intensity_pin));
    angle_pin_register = portInputRegister(digitalPinToPort(angle_pin));

    // Setup pca9685 control
    pwm = Adafruit_PWMServoDriver(pca_addr, Wire);
    pwm.begin();
    // Calibration needed for every pca9685 module.
    // Range for clock is in range 23Mhz to 27Mhz.
    // More info check on,
    // https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/blob/master/examples/pwmtest/pwmtest.ino
    pwm.setOscillatorFrequency(pca_clock);
    //  Use 50hz freq for calibration
    pwm.setPWMFreq(pca_pwm_freq);

    // Setup led calculator
    led_calculator = LedCalculator();
}

void loop() {

    if (intensity_falling_edge) {
        intensity_falling_edge = false;
        falling_count++;
        intensity_pwm_lpf =
                (float)intensity_end_pulse * alpha +
                intensity_pwm_lpf * (1 - alpha);
    }

    if (angle_falling_edge) {
        angle_falling_edge = false;
        angle_pwm_lpf =
                (float)angle_end_pulse * alpha +
                angle_pwm_lpf * (1 - alpha);
    }

    // Applying pwm captured info to leds
    if (falling_count >= action_period) {

        falling_count = 0;

        if (serial_com) {
            Serial.print("Input pwm: ");
            Serial.println(intensity_pwm_lpf);
            Serial.print("Angle pwm: ");
            Serial.println(angle_pwm_lpf);
        }

        // Centering pwm amplitude in range 0-800us
        float intensity_pwm = intensity_pwm_lpf - intensity_pwm_base;
        float angle_pwm = angle_pwm_lpf - angle_pwm_base;

        intensity_pwm = limit_pwm(0,
                                  intensity_pwm_diff,
                                  intensity_pwm);

        angle_pwm = limit_pwm(0,
                              angle_pwm_diff,
                              angle_pwm);

        uint8_t intensity_pct = (uint8_t)ceil(double(intensity_pwm * 100.f / intensity_pwm_diff));

        // Calculating leds strip intensity to be used
        if (serial_com) {
            Serial.print("Intensity pwm centered: ");
            Serial.println(intensity_pwm);
            Serial.print("Intensity pwm pct: ");
            Serial.println(intensity_pct);
            Serial.print("Angle pwm centered: ");
            Serial.println(angle_pwm);
        }

        led_calculator.set_input_intensity(intensity_pct);
        led_calculator.set_intensity_position(angle_pwm);
        led_calculator.calculate();

        LedProperties* led_properties = led_calculator.get_led_properties();

        // Delivering it to pca9685
        for (uint8_t i = 0; i < num_leds; i++) {

            float led_intensity = (float) led_properties[i].intensity / 100.f;
            uint16_t pwm_reg_value = (uint16_t) ceil(double(led_intensity * pca_pwm_register_max));
            pwm.setPWM(i, 0, pwm_reg_value);

            if (serial_com) {
                Serial.print("LED PWM Register Value ");
                Serial.print(i);
                Serial.print(": ");
                Serial.println(pwm_reg_value);
            }
        }
    }
}

void on_intensity_interrupt() {
    boolean pin_value = fastDigitalRead(intensity_pin_register,
                                        intensity_pin_bitMask);

    if (pin_value) {
        intensity_start_pulse = micros();
    }

    else {
        intensity_falling_edge = true;
        intensity_end_pulse = micros() - intensity_start_pulse;
    }
}

void on_angle_interrupt() {
    boolean pin_value = fastDigitalRead(angle_pin_register, angle_pin_bitMask);

    if (pin_value) {
        angle_start_pulse = micros();
    }

    else {
        angle_falling_edge = true;
        angle_end_pulse = micros() - angle_start_pulse;
    }
}


float limit_pwm(float bottom, float top, float raw_pwm) {
    if (raw_pwm < bottom)
        return bottom;

    else if (raw_pwm > top)
        return top;

    else
        return raw_pwm;
}


