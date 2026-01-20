#ifndef CONFIG_H
#define CONFIG_H
#include <stdint.h>
#include <mcp2515.h>
#include <stdlib.h>

// CAN Pins
const uint8_t CAN_CS_MOTOR = PIN_PB2;  // PB2 (Arduino pin 10)
const uint8_t CAN_CS_BMS = PIN_PB1;     // PB1 (Arduino pin 9)
const uint8_t CAN_CS_DEBUG = PIN_PB0;   // PB0 (Arduino pin 8)

// CAN Config
const long CAN_BAUDRATE = CAN_500KBPS;  // Standard automotive bitrate
const uint32_t BMS_READY_ID = 0x186040F3;  // BMS readiness CAN ID
const uint8_t BMS_READY_BYTE = 6;          // Check data[6]
const uint8_t BMS_READY_VALUE = 0x50;      // Expected value for readiness

const uint32_t MOTOR_TORQUE_ID = 0x201;    // Motor command ID

// Custom Debug CAN IDs
const uint32_t DEBUG_PEDALS_ID = 0x700;    // Message for pedal readings
const uint32_t DEBUG_STATE_ID = 0x701;     // Message for car state
const uint32_t DEBUG_FAULT_ID = 0x702;     // Message for faults (send only on fault)

#endif