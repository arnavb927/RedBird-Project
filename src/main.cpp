#include <Arduino.h>
#include <SPI.h>
#include <config.h>
// Pin Definitions
#define APPS_5V_PIN PIN_PC0     // PC0 analog
#define APPS_3V3_PIN PIN_PC1    // PC1 analog
#define BRAKE_PIN PIN_PC3       // PC3 analog
#define START_BUTTON_PIN PIN_PC4 // PC4 digital input

#define BRAKE_LIGHT_PIN PIN_PD2 // PD2 digital output
#define BUZZER_PIN PIN_PD4      // PD4 digital output
#define DRIVE_LED_PIN PIN_PD3   // PD3 digital output

MCP2515 canMotor(CAN_CS_MOTOR);
MCP2515 canBMS(CAN_CS_BMS);
MCP2515 canDebug(CAN_CS_DEBUG);


// Thresholds and Constants
const unsigned int BRAKE_DEPRESSED_THRESHOLD = 512;
const float APPS_FAULT_THRESHOLD = 0.10; // 10% difference
const uint32_t STARTIN_HOLD_TIME = 2000;       // 2 seconds
const uint32_t BUZZIN_TIME = 2000;             // 2 seconds
const uint32_t APPS_FAULT_TIMEOUT = 100;       // 100ms fault duration
const bool FLIP_MOTOR_DIRECTION = false; // Compile-time flip for torque sign

const int16_t TORQUE_MIN = -32768;
const int16_t TORQUE_MAX = 32767;

// Pedal input ranges (assume 0-1023 from analogRead; calibrate if needed)
const unsigned int PEDAL_MIN = 0;
const unsigned int PEDAL_MAX = 1023;

enum State
{
  INIT,
  STARTIN,
  BUZZIN,
  DRIVE
};

State currentState = INIT;
uint32_t stateStartTime = 0;
uint32_t faultStartTime = 0;
bool isFaulty = false;
int16_t calculatedTorque = 0;
uint16_t apps5V = 0;
uint16_t apps3V3 = 0;
uint16_t brake = 0;
bool startButtonPressed = false;

void setup()
{
  
  pinMode(START_BUTTON_PIN, INPUT);
  pinMode(BRAKE_LIGHT_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(DRIVE_LED_PIN, OUTPUT);

  digitalWrite(BRAKE_LIGHT_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(DRIVE_LED_PIN, LOW);

  stateStartTime = millis();


  canMotor.reset();
  if (canMotor.setBitrate(CAN_500KBPS, MCP_20MHZ) != MCP2515::ERROR_OK) {
    while (1);
  }
  canMotor.setNormalMode();

  canBMS.reset();
  if (canBMS.setBitrate(CAN_500KBPS, MCP_20MHZ) != MCP2515::ERROR_OK) {
    while(1);
  }
  canBMS.setNormalMode(MCP_NORMAL);

  canDebug.reset();
  if (canDebug.setBitrate(CAN_500KBPS, MCP_20MHZ) != MCP2515::ERROR_OK) {
    while (1);
  }
  canDebug.setNormalMode();

}

void loop()
{
  apps5V = analogRead(APPS_5V_PIN);
  apps3V3 = analogRead(APPS_3V3_PIN);
  brake = analogRead(BRAKE_PIN);
  startButtonPressed = (digitalRead(START_BUTTON_PIN) == LOW);

  digitalWrite(BRAKE_LIGHT_PIN, (brake > BRAKE_DEPRESSED_THRESHOLD) ? HIGH : LOW);

  switch (currentState)
  {
  case INIT:
    calculatedTorque = 0;
    digitalWrite(DRIVE_LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);

    if (startButtonPressed && brake > BRAKE_DEPRESSED_THRESHOLD)
    {
      currentState = STARTIN;
      stateStartTime = millis();
    }
    break;

  case STARTIN:
    calculatedTorque = 0;
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(DRIVE_LED_PIN, LOW);

    if (!startButtonPressed || brake <= BRAKE_DEPRESSED_THRESHOLD) {
      currentState = INIT;
      break;
    }

    if (millis() - stateStartTime >= STARTIN_HOLD_TIME) {
      can_message_t msg;

      if (canBMS.readMessage(&msg) == MCP2515::ERROR_OK &&
          msg.can_id == BMS_READY_ID && msg.data[BMS_READY_BYTE] == BMS_READY_VALUE) {
          currentState = BUZZIN;
          stateStartTime = millis();
      }
    }
    break;

  case BUZZIN:
    calculatedTorque = 0;
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(DRIVE_LED_PIN, LOW);

    if (millis() - stateStartTime >= BUZZIN_TIME)
    {
      currentState = DRIVE;
      stateStartTime = millis();
      digitalWrite(BUZZER_PIN, LOW);
    }
    break;

  case DRIVE:
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
    int16_t torque_scale = FLIP_MOTOR_DIRECTION ? -32768 : 32767;
    calculatedTorque = static_cast<int16_t>((clamped_num * static_cast<int64_t>(torque_scale))/den);

    // float scaledApps3V3 = static_cast<float>(apps3V3) * (5.0 / 3.3);
    // float diffPercent = abs(apps5V - scaledApps3V3) / static_cast<float>(PEDAL_MAX-PEDAL_MIN);

    // if (diffPercent > APPS_FAULT_THRESHOLD) {
    //   if (!isFaulty) {
    //     faultStartTime = millis();
    //     isFaulty = true;
    //   }

    //   if (millis() - faultStartTime > APPS_FAULT_TIMEOUT) {
    //     currentState = INIT;
    //     calculatedTorque = 0;
    //     isFaulty = false;
    //     break;
    //   }
    // }
    // else {
    //   isFaulty = false;
    // }

    // // Calculate Torque
    // float avgPedal = (static_cast<float>(apps5V) + scaledApps3V3) / 2.0f;
    // float pedalPercent = static_cast<float>(avgPedal - PEDAL_MIN) / (PEDAL_MAX - PEDAL_MIN);
    // pedalPercent = max(0.0f, min(1.0f, pedalPercent));
    // int16_t torqueScale = FLIP_MOTOR_DIRECTION ? TORQUE_MIN : TORQUE_MAX;
    // calculatedTorque = static_cast<int16_t>(pedalPercent * static_cast<float>(torqueScale));

    // if (FLIP_MOTOR_DIRECTION)
    // {
    //   calculatedTorque *= -1;
    // }
    
    can_message_t msg;
    msg.can_id = MOTOR_TORQUE_ID;
    msg.can_dlc = 8;
    msg.data[0] = 0x00;
    msg.data[1] = 0x01;
    msg.data[2] = 0x02;
    msg.data[3] = 0x90;
    msg.data[4] = static_cast<uint8_t>(calculatedTorque & 0xFF);
    msg.data[5] = static_cast<uint8_t>((calculatedTorque >> 8) & 0xFF);
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;

    canMotor.sendMessage(&msg);
    break;
  }

  can_message_t pedalMsg;
  pedalMsg.can_id = DEBUG_PEDALS_ID;
  pedalMsg.can_dlc = 8;
  pedalMsg.data[0] = (uint8_t)(apps5V & 0xFF);
  pedalMsg.data[1] = (uint8_t)(apps5V >> 8);
  pedalMsg.data[2] = (uint8_t)(apps3V3 & 0xFF);
  pedalMsg.data[3] = (uint8_t)(apps3V3 >> 8);
  pedalMsg.data[4] = (uint8_t)(brake & 0xFF);
  pedalMsg.data[5] = (uint8_t)(brake >> 8);
  pedalMsg.data[6] = 0;
  pedalMsg.data[7] = 0;
  canDebug.sendMessage(&pedalMsg);

  can_message_t stateMsg;
  stateMsg.can_id = DEBUG_STATE_ID;
  stateMsg.can_dlc = 8;
  stateMsg.data[0] = (uint8_t)currentState;
  stateMsg.data[1] = 0;
  stateMsg.data[2] = 0;
  stateMsg.data[3] = 0;
  stateMsg.data[4] = 0;
  stateMsg.data[5] = 0;
  stateMsg.data[6] = 0;
  stateMsg.data[7] = 0;
  canDebug.sendMessage(&stateMsg);

  if (isFaulty) {
    can_message_t faultMsg;
    faultMsg.can_id = DEBUG_FAULT_ID;
    faultMsg.can_dlc = 8;
    float scaledApps3V3 = static_cast<float>(apps3V3) * (5.0/3.3);
    uint16_t diff = static_cast<uint16_t>(abs(static_cast<float>(apps5V) - scaledApps3V3));
    faultMsg.data[0] = (uint8_t)(diff & 0xFF);
    faultMsg.data[1] = (uint8_t)(diff>>8);
    faultMsg.data[2] = 0;
    faultMsg.data[3] = 0;
    faultMsg.data[4] = 0;
    faultMsg.data[5] = 0;
    faultMsg.data[6] = 0;
    faultMsg.data[7] = 0;
    canDebug.sendMessage(&faultMsg);

  }

}

// put function definitions here:
