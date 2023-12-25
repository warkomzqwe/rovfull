/**
 * @file api_grabber.h
 * @brief ROV grabber control
 */

#pragma once


typedef enum {
    IDLE,
    OPENING,
    CLOSING,
} grabber_state_t;


void grabber_init();
void grabber_set_state(grabber_state_t grabber_state);
