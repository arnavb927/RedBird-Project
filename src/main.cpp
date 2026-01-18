#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <config.h>
// Pin Definitions
#define APPS_5V_PIN A0     // PC0 analog
#define APPS_3V3_PIN A1    // PC1 analog
#define BRAKE_PIN A3       // PC3 analog
#define START_BUTTON_PIN 4 // PC4 digital input

#define BRAKE_LIGHT_PIN 2 // PD2 digital output
#define BUZZER_PIN 4      // PD4 digital output
#define DRIVE_LED_PIN 3   // PD3 digital output

MCP_CAN canMotor(CAN_CS_MOTOR);
MCP_CAN canBMS(CAN_CS_BMS);
MCP_CAN canDebug(CAN_CS_BMS);


// Thresholds and Constants
const unsigned int BRAKE_DEPRESSED_THRESHOLD = 512;
const float APPS_FAULT_THRESHOLD = 0.10; // 10% difference
uint32_t STARTIN_HOLD_TIME = 2000;       // 2 seconds
uint32_t BUZZIN_TIME = 2000;             // 2 seconds
uint32_t APPS_FAULT_TIMEOUT = 100;       // 100ms fault duration
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

  SPI.begin(); //init SPI bus
  if (canMotor.begin(MCP_ANY, CAN_BAUDRATE, MCP_20MHZ) != CAN_OK) {
    while (1);
  }
  canMotor.setMode(MCP_NORMAL);

  if (canBMS.begin(MCP_ANY, CAN_BAUDRATE, MCP_20MHZ) != CAN_OK) {
    while(1);
  }
  canBMS.setMode(MCP_NORMAL);

  if (canDebug.begin(MCP_ANY, CAN_BAUDRATE, MCP_20MHZ) != CAN_OK) {
    while (1);
  }
  canDebug.setMode(MCP_NORMAL);
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
      unsigned char len = 0;
      unsigned char rxBuf[8];
      uint32_t rxId;
      if (canBMS.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) {
        if (rxId == BMS_READY_ID && rxBuf[BMS_READY_BYTE] == BMS_READY_VALUE) {
          currentState = BUZZIN;
          stateStartTime = millis();
        }
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
    float scaledApps3V3 = static_cast<float>(apps3V3) * (5.0 / 3.3);
    float diffPercent = abs(apps5V - scaledApps3V3) / static_cast<float>(PEDAL_MAX-PEDAL_MIN);

    if (diffPercent > APPS_FAULT_THRESHOLD) {
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

    // Calculate Torque
    float avgPedal = (static_cast<float>(apps5V) + scaledApps3V3) / 2.0f;
    float pedalPercent = static_cast<float>(avgPedal - PEDAL_MIN) / (PEDAL_MAX - PEDAL_MIN);
    pedalPercent = max(0.0f, min(1.0f, pedalPercent));
    int16_t torqueScale = FLIP_MOTOR_DIRECTION ? TORQUE_MIN : TORQUE_MAX;
    calculatedTorque = static_cast<int16_t>(pedalPercent * static_cast<float>(torqueScale));

    if (FLIP_MOTOR_DIRECTION)
    {
      calculatedTorque *= -1;
    }
    
    //send torque to motor via CAN
    uint8_t motorData[8] = {0x00, 0x01, 0x02, 0x90, 0x00, 0x00, 0x00};

    //pack torque in little-endian
    motorData[4] = static_cast<uint8_t>(calculatedTorque & 0xFF);
    motorData[5] = static_cast<uint8_t>((calculatedTorque >> 8) & 0xFF);

    canMotor.sendMsgBuf(MOTOR_TORQUE_ID, 0, 8, motorData)
    break;
  }

  uint8_t pedalData[8] = {0};

  pedalData[0] = (uint8_t)(apps5V & 0xFF); pedalData[1] = (uint8_t)(apps5V >> 8);
  pedalData[2] = (uint8_t)(apps3V3 & 0xFF); pedalData[3] = (uint8_t)(apps3V3 >> 8);
  pedalData[4] = (uint8_t)(brake & 0xFF); pedalData[5] = (uint8_t)(brake >> 8);
  canDebug.sendMsgBuf(DEBUG_PEDALS_ID, 0, 8, pedalData);

  uint8_t stateData[8] = {0};
  stateData[0] =(uint8_t)currentState;
  canDebug.sendMsgBuf(DEBUG_STATE_ID, 0, 8, stateData);

  if (isFaulty) {
    uint8_t faultData[8] = {0};
    float scaledApps3V3 = static_cast<float>(apps3V3) * (5.0/3.3);
    uint16_t diff = static_cast<uint16_t>(abs(static_cast<float>(apps5V)-scaledApps3V3));
    faultData[0] = (uint8_t)(diff & 0xFF);
    faultData[1] = (uint8_t)(diff >> 8);
    canDebug.sendMsgBuf(DEBUG_FAULT_ID, 0, 8, faultData);
  }

}

// put function definitions here:
