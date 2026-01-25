/**
 * @file main.cpp
 * @author Arnav
 * @brief Main program for the Red Bird Racing EVRT VCU
 *        Handles pedal inputs, state transitions, fault detection, and CAN communications
 *        for BMS, motor torque, and debug messages. Implements a state machine for vehicle
 *        states: INIT, STARTIN, BUZZIN, DRIVE
 * @date 2026-01-22
 * @note This code is develop for ATMega328P using arduino framework and MCP2515 library for CAN
 *       Calibrate pedal ranges and thresholds as needed.
 * @see config.h for pin and CAN configurations
 */

#include <Arduino.h>
#include <config.h>

// External MCP2515 instances for CAN buses
/** @brief CAN interface for motor commands */
MCP2515 canMotor(CAN_CS_MOTOR);
/** @brief CAN interface for BMS communications */
MCP2515 canBMS(CAN_CS_BMS);
/** @brief CAN interface for debug messages */
MCP2515 canDebug(CAN_CS_DEBUG);

/**
 * @brief Enum representing the vehicle states as per project spec
 * @note States transition: INIT -> STARTIN (button + brake) -> BUZZIN (BMS ready) -> DRIVE
 *       Faults revert to INIT
 */
enum State
{
  INIT,   /**<Initial state, no torque, waiting for start conditions */
  STARTIN, /**<Starting state, hold button and brake for 2s */
  BUZZIN, /**< Buzzing state, buzzer on for 2s before drive */
  DRIVE   /**< Drive state, computer and send torque, monitor faults */
};

/**
 * @brief Current Vehicle state
 */
State currentState = INIT;

/**
 * @brief Timestamp when the current state started (ms)
 */
uint32_t stateStartTime = 0;

/**
 * @brief Timestamp when an APPS fault was first detected (ms)
 */
 uint32_t faultStartTime = 0;

 /**
  * @brief Flag indicating if an APPS fault is active 
  */
bool isFaulty = false;

/**
 * @brief Calculated torque value to send to motor (-32768 to 32767)
 */
int16_t calculatedTorque = 0;

/**
 * @brief Raw ADC reading from 5V APPS pedal
 */
uint16_t apps5V = 0;

/**
 * @brief Raw ADC reading from 3.3V APPS pedal
 */
uint16_t apps3V3 = 0;

/**
 * @brief RAW ADC reading from brake pedal
 */
uint16_t brake = 0;

/**
 * @brief Flage if start button is pressed (active low)
 */
bool startButtonPressed = false;

/**
 * @brief Arduino setup function. Initializes pins and CAN interfaces
 * @note Called once at startup. Sets pin modes, initalizes output to low,
 *       resets and configures MCP2515 for 500kbps normal mode
 */
void setup() {
  // Initializes input pins
  pinMode(START_BUTTON_PIN, INPUT);

  //Initializes output pins
  pinMode(BRAKE_LIGHT_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(DRIVE_LED_PIN, OUTPUT);

  // Set inital output states
  digitalWrite(BRAKE_LIGHT_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(DRIVE_LED_PIN, LOW);

  // Record start time for state timing
  stateStartTime = millis();

  // Initializes CAN interfaces
  canMotor.reset();
  canMotor.setBitrate(CAN_500KBPS, MCP_20MHZ);
  canMotor.setNormalMode();

  canBMS.reset();
  canBMS.setBitrate(CAN_500KBPS, MCP_20MHZ);
  canBMS.setNormalMode();

  canDebug.reset();
  canDebug.setBitrate(CAN_500KBPS, MCP_20MHZ);
  canDebug.setNormalMode();
}

/**
 * @brief Arduino main loop. Reads inputs, handles state logic computes torque,
 *        checks faults, and sends CAN messages.
 * @note Reads pedals and button, updates outputs, mangages state transtions
 *        and sends debug/motor CAN frames
 */
void loop() {
  //Read sensor inputs 
  apps5V = analogRead(APPS_5V_PIN);
  apps3V3 = analogRead(APPS_3V3_PIN);
  brake = analogRead(BRAKE_PIN);
  startButtonPressed = (digitalRead(START_BUTTON_PIN) == LOW);

  // Update brake light based on threshold
  digitalWrite(BRAKE_LIGHT_PIN, (brake > BRAKE_DEPRESSED_THRESHOLD) ? HIGH : LOW);

  // State machine handling 
  switch (currentState) {
    case INIT:
      // No torque, LEDS/buzzer off
      calculatedTorque = 0;
      digitalWrite(DRIVE_LED_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);

      // transition to STARTIN if button pressed and brake depressed
      if (startButtonPressed && brake > BRAKE_DEPRESSED_THRESHOLD)
      {
        currentState = STARTIN;
        stateStartTime = millis();
      }
      break;

  case STARTIN:
    // No torque, buzzer/LED off
    calculatedTorque = 0;
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(DRIVE_LED_PIN, LOW);
    
    // Check hold time and BMS ready message
    if (!startButtonPressed || brake <= BRAKE_DEPRESSED_THRESHOLD) {
      currentState = INIT;
      break;
    }

    //Check hold time and BMS ready message
    if (millis() - stateStartTime >= STARTIN_HOLD_TIME) {
      can_frame msg;
      if (canBMS.readMessage(&msg) == MCP2515::ERROR_OK &&
          msg.can_id == BMS_READY_ID && msg.data[BMS_READY_BYTE] == BMS_READY_VALUE) {
          currentState = BUZZIN;
          stateStartTime = millis();
      }
    }
    break;

  case BUZZIN:
    // No torque, buzzer on, LED off
    calculatedTorque = 0;
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(DRIVE_LED_PIN, LOW);

    // Transition to DRIVE after buzz time
    if (millis() - stateStartTime >= BUZZIN_TIME) {
      currentState = DRIVE;
      stateStartTime = millis();
      digitalWrite(BUZZER_PIN, LOW);
    }
    break;

  case DRIVE:
    // LED on
    digitalWrite(DRIVE_LED_PIN, HIGH);

    // Check APPS fault
    int32_t left = (int32_t)apps5V * 33;
    int32_t right = (int32_t)apps3V3 *50;
    int32_t abs_diff_scaled = labs(left-right);
    int32_t range = PEDAL_MAX - PEDAL_MIN;
    if (10 * abs_diff_scaled > range * 33) {
      if (!isFaulty) {
        faultStartTime = millis();
        isFaulty = true;
      }
      if (millis() - faultStartTime > APPS_FAULT_TIMEOUT) {
        currentState = INIT;
        calculatedTorque = 0;
        isFaulty = false;
        break;
      }
    }
    else {
      isFaulty = false;
    }

    //Calculate Torque
    int64_t num = static_cast<int64_t>(left) + right - static_cast<int64_t>(PEDAL_MIN) *66;
    int64_t den = 66LL * static_cast<int64_t>(PEDAL_MAX-PEDAL_MIN);
    int64_t clamped_num = (num<0) ? 0LL : (num>den)?den:num;
    uint64_t pedal_pos = static_cast<uint16_t>((clamped_num * range) / den) + PEDAL_MIN;
    uint16_t interpolated_torque = 0;
    for(int i = 0; i < NUM_POINTS - 1; ++i) {
      if (pedal_pos >= PEDAL_POINTS[i] && pedal_pos <= PEDAL_POINTS[i + 1]) {
        uint16_t delta_pedal = PEDAL_POINTS[i+1] - PEDAL_POINTS[i];
        uint16_t pos_in_segment = pedal_pos - PEDAL_POINTS[i];

        int32_t delta_torque = static_cast<int32_t>(TORQUE_POINTS[i+1]) - TORQUE_POINTS[i];

        int64_t temp = static_cast<int64_t>(pos_in_segment) * delta_torque;
        interpolated_torque = TORQUE_POINTS[i] + static_cast<int16_t>(temp/delta_pedal);
        break;
      }
    }

    if (interpolated_torque < TORQUE_MIN) interpolated_torque = TORQUE_MIN;
    if (interpolated_torque > TORQUE_MAX) interpolated_torque = TORQUE_MAX;

    if (FLIP_MOTOR_DIRECTION) {
      interpolated_torque = -interpolated_torque;
    }

    calculatedTorque = interpolated_torque;

    // Send motor torque
    can_frame msg;
    msg.can_id = MOTOR_TORQUE_ID;
    msg.can_dlc = 3;
    msg.data[0] = 0x90;
    msg.data[1] = calculatedTorque & 0xFF;
    msg.data[2] = (calculatedTorque >> 8) & 0xFF;
    canMotor.sendMessage(&msg);
    break;
  }

  // Send debug pedal readings CAN message
  can_frame pedalMsg;
  pedalMsg.can_id = DEBUG_PEDALS_ID;
  pedalMsg.can_dlc = 6;
  pedalMsg.data[0] = (uint8_t)(apps5V & 0xFF);
  pedalMsg.data[1] = (uint8_t)(apps5V >> 8);
  pedalMsg.data[2] = (uint8_t)(apps3V3 & 0xFF);
  pedalMsg.data[3] = (uint8_t)(apps3V3 >> 8);
  pedalMsg.data[4] = (uint8_t)(brake & 0xFF);
  pedalMsg.data[5] = (uint8_t)(brake >> 8);
  canDebug.sendMessage(&pedalMsg);

  // Send debug fault message if faulty
  can_frame stateMsg;
  stateMsg.can_id = DEBUG_STATE_ID;
  stateMsg.can_dlc = 1;
  stateMsg.data[0] = (uint8_t)currentState;
  canDebug.sendMessage(&stateMsg);

  if (isFaulty) {
    can_frame faultMsg;
    faultMsg.can_id = DEBUG_FAULT_ID;
    faultMsg.can_dlc = 8; 
    int32_t left = (int32_t)apps5V * 33;
    int32_t right = (int32_t)apps3V3 * 50;
    int32_t diff_scaled = labs(left-right);
    uint16_t diff = static_cast<int16_t>((diff_scaled + 16) / 33);
    faultMsg.data[0] = (uint8_t)(diff & 0xFF);
    faultMsg.data[1] = (uint8_t)(diff >> 8);
    canDebug.sendMessage(&faultMsg);
  }

}