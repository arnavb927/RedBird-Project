/**
 * @file config.h
 * @author Arnav
 * @brief Configuration header for VCU project. Defines pins, CAN settings, thresholds, and constants.
 * @version 1.0
 * @date 2026-01-22
 * @note Values based on project specs. Adjust for hardware calibration.
 *       CAN bitrate fixed at 500kbps. No dynamic values.
 * @see main.cpp for usage.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <mcp2515.h>
#include <stdlib.h>

// Pin Definitions
/** @brief Analog pin for 5V APPS pedal */
#define APPS_5V_PIN PIN_PC0     // PC0 analog
/** @brief Analog pin for 3.3V APPS pedal */
#define APPS_3V3_PIN PIN_PC1    // PC1 analog
/** @brief Analog pin for brake pedal */
#define BRAKE_PIN PIN_PC3       // PC3 analog
/** @brief Digital input pin for start button (active low) */
#define START_BUTTON_PIN PIN_PC4 // PC4 digital input
/** @brief Digital output pin for brake light*/
#define BRAKE_LIGHT_PIN PIN_PD2 // PD2 digital output
/** @brief Digital output pin for buzzer */
#define BUZZER_PIN PIN_PD4      // PD4 digital output
/** @brief Digital Output pin for drive mode LED */
#define DRIVE_LED_PIN PIN_PD3   // PD3 digital output

// CAN Pins
/** @brief Chip select pin for motor CAN */
const uint8_t CAN_CS_MOTOR = PIN_PB2;  // PB2 (Arduino pin 10)
/** @brief Chip select pin for BMS CAN */
const uint8_t CAN_CS_BMS = PIN_PB1;     // PB1 (Arduino pin 9)
/** @brief Chip select pin for debug CAN */
const uint8_t CAN_CS_DEBUG = PIN_PB0;   // PB0 (Arduino pin 8)

// CAN Config
/** @brief CAN Baudrate */
const long CAN_BAUDRATE = CAN_500KBPS;  // Standard automotive bitrate
/** @brief CAN ID for BMS ready message */
const uint32_t BMS_READY_ID = 0x186040F3;  // BMS readiness CAN ID
/** @brief Byte indix in BMS message for ready value */
const uint8_t BMS_READY_BYTE = 6;          // Check data[6]
/** @brief Expected value for BMS ready */
const uint8_t BMS_READY_VALUE = 0x50;      // Expected value for readiness
/** @brief Expected value for BMS ready */
const uint32_t MOTOR_TORQUE_ID = 0x201;    // Motor command ID

// Custom Debug CAN IDs
/** @brief Debug CAN ID for pedal readings */
const uint32_t DEBUG_PEDALS_ID = 0x700;    // Message for pedal readings
/** @brief Debug CAN ID for state */
const uint32_t DEBUG_STATE_ID = 0x701;     // Message for car state
/** @brief Debug CAN ID for faults */
const uint32_t DEBUG_FAULT_ID = 0x702;     // Message for faults (send only on fault)

// Thresholds and Constants
/** @brief Brake threshold for depressed state */
const unsigned int BRAKE_DEPRESSED_THRESHOLD = 512;
/** @brief APPS fault difference threshold (10% difference) */
const float APPS_FAULT_THRESHOLD = 0.10;
/** @brief Hold time for STARTIN state (ms) */
const uint32_t STARTIN_HOLD_TIME = 2000;       // 2 seconds
/** @brief Buzzer on time in BUZZIN state */
const uint32_t BUZZIN_TIME = 2000;             // 2 seconds
/** @brief APPS fault duration before reset to INIT (ms) */
const uint32_t APPS_FAULT_TIMEOUT = 100;       // 100ms fault duration
/** @brief Compile-Time flag to flip motor torque direction */
const bool FLIP_MOTOR_DIRECTION = false; // Compile-time flip for torque sign
/** @brief Minimum torque value */
const int16_t TORQUE_MIN = -32768;
/** @brief Maximum torque value */
const int16_t TORQUE_MAX = 32767;

// Pedal input ranges (ADC 0-1023)
/** @brief Minimum pedal ADC value */
const unsigned int PEDAL_MIN = 0;
/** @brief MAx pedal ADC value */
const unsigned int PEDAL_MAX = 1023;

const uint8_t NUM_POINTS = 5;

const uint16_t PEDAL_POINTS[NUM_POINTS] = {0, 100, 200, 300, 1023};

const int16_t TORQUE_POINTS[NUM_POINTS] = {0, 500, 1500, 3000, 32767};
#endif