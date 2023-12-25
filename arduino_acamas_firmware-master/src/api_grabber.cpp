/**
 * @file api_grabber.cpp
 * @brief api_grabber implementation
 */

#include "api_grabber.h"
#include "Arduino.h"
#include "Servo.h"

#define PIN_GRABBER 9
#define PIN_GRABBER_ENABLE 3
#define IDLE_US 0
#define OPEN_US 2000
#define CLOSE_US 1000

static Servo servo;
static grabber_state_t current_state = IDLE;


// File scope functions
static void idle_entry();
static void opening_entry();
static void closing_entry();


void grabber_init() {
    pinMode(PIN_GRABBER_ENABLE, OUTPUT);
    servo.attach(PIN_GRABBER);

    digitalWrite(PIN_GRABBER_ENABLE, 0);
    servo.writeMicroseconds(IDLE_US);
}


void grabber_set_state(grabber_state_t grabber_state) {

    switch (current_state) {

        case IDLE:
            if (grabber_state == OPENING) {
                current_state = OPENING;
                opening_entry();

            } else if (grabber_state == CLOSING) {
                current_state = CLOSING;
                closing_entry();
            }
            break;

        case CLOSING:
        case OPENING:
            if (grabber_state == IDLE) {
                current_state = IDLE;
                idle_entry();
            }
            break;
    }
}

/*******************************************************************************
 Private Functions
 ******************************************************************************/
static void idle_entry() {
    digitalWrite(PIN_GRABBER_ENABLE, 0);
    servo.writeMicroseconds(IDLE_US);
}

static void opening_entry() {
    digitalWrite(PIN_GRABBER_ENABLE, 1);
    servo.writeMicroseconds(OPEN_US);
}

static void closing_entry() {
    digitalWrite(PIN_GRABBER_ENABLE, 1);
    servo.writeMicroseconds(CLOSE_US);
}
