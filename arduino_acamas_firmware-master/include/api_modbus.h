/**
 * @file api_modbus.h
 * @brief Module defines internal modbus protocol used for device.
 */

#pragma once

/**
 * Modbus start including input and holding registers setup.
 */
void modbus_init();

/**
 * Modbus polling function to handle serial communication.
 *
 * It's expected to be polled with high frecuency / high priority.
 */
void modbus_update();

/**
 * Reading all system inputs and map values to modbus registers.
 */
void modbus_map_inputs();

/**
 * Updating all mapped modbus holding registers to outputs.
 *
 * Including pwm, etc.
 */
void modbus_map_outputs();
