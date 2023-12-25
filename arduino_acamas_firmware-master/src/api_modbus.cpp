/**
 * @file api_modbus.cpp
 * @brief Modbus api implementation.
 */


#include <ArduinoModbus.h>
#include <ArduinoRS485.h>
#include "api_modbus.h"
#include "api_lights.h"
#include "api_grabber.h"


/**
 * \def INPUT_REGISTER_START_ADD
 * Start address for input registers
 *
 * \def HOLDING_REGISTERS_START_ADD
 * Start address for holding registers
 */

#define MODBUS ModbusRTUServer

#define DEVICE_SLAVE_ADD 0x01
#define MODBUS_BAUDRATE 115200
#define NUM_INPUT_REGISTERS 3
#define INPUT_REGISTER_START_ADD 0x0
#define NUM_HOLDING_REGISTERS 4
#define HOLDING_REGISTERS_START_ADD 0x100


/**
 * Every register either input or holding have an id or index and an initial value
 */
typedef struct
{
    uint8_t id;
    uint16_t initial_value;
} modbus_16bits_register_t;


/**
 * List of all declared input registers.
 */
enum InputId {
    HWID_L,
    HWID_H,
    VERSION,
};

// Initial values
const modbus_16bits_register_t input_registers[NUM_INPUT_REGISTERS] = {
    {HWID_L, 0xf4f4},
    {HWID_H, 0xf121},
    {VERSION, 0x1},
};


/**
 * List of all declared holding registers
 */
enum HoldingId {
    LIGHTS_ENABLE,
    LIGHTS_ON,
    GRABBER_STATE,
    AUXILIARY_ENABLE,
};

/**
 * Initial values.
 *
 * For GRABBER_STATE we have,
 *      0 --> IDLE
 *      1 --> OPENING
 *      2 --> CLOSING
 */
const modbus_16bits_register_t holding_registers[NUM_HOLDING_REGISTERS] = {
    {LIGHTS_ENABLE, 0x0},
    {LIGHTS_ON, 0x0},
    {GRABBER_STATE, 0x0},
    {AUXILIARY_ENABLE, 0x0},
};


// File scope functions
static uint8_t read_holding_reg(HoldingId reg);

void modbus_init() {

    MODBUS.begin(DEVICE_SLAVE_ADD, MODBUS_BAUDRATE);

    /* Disabling DE pin from RS485 library (used by modbus lib under the hood).
    Assigned pin was 2 (Arduino mega reference) and it was in conflict with PWM 1. */
    RS485.setPins(RS485_DEFAULT_TX_PIN, -1, -1);

    // ==> Configuring registers to meet standard

    // Input registers (Init default to zero)
    // Only different registers are HWID and VERSION
    MODBUS.configureInputRegisters(INPUT_REGISTER_START_ADD, NUM_INPUT_REGISTERS);

    for (uint8_t i = 0; i < NUM_INPUT_REGISTERS; i++) {
        MODBUS.inputRegisterWrite(INPUT_REGISTER_START_ADD + i, input_registers[i].initial_value);
    }

    // Holding registers
    MODBUS.configureHoldingRegisters(HOLDING_REGISTERS_START_ADD, NUM_HOLDING_REGISTERS);

    for (uint8_t i = 0; i < NUM_HOLDING_REGISTERS; i++) {
        MODBUS.holdingRegisterWrite(HOLDING_REGISTERS_START_ADD + i, holding_registers[i].initial_value);
    }
}


void modbus_update() {
    // modbus protocol handler
    MODBUS.poll();
}


void modbus_map_inputs() {
}


void modbus_map_outputs() {

    uint8_t lights_turn_on = read_holding_reg(LIGHTS_ON) && read_holding_reg(LIGHTS_ENABLE);
    auto grabber_state = (grabber_state_t) read_holding_reg(GRABBER_STATE);

    lights_turn_on ? lights_on() : lights_off();
    grabber_set_state(grabber_state);
}


/*******************************************************************************
 Private Functions
 ******************************************************************************/
static uint8_t read_holding_reg(HoldingId reg) {
    return (uint8_t) MODBUS.holdingRegisterRead(reg + HOLDING_REGISTERS_START_ADD);
}




