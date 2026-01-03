#include <Arduino.h>

// Pin Definitions
#define APPS_5V_PIN A0     // PC0 analog
#define APPS_3V3_PIN A1    // PC1 analog
#define BRAKE_PIN A3       // PC3 analog
#define START_BUTTON_PIN 4 // PC4 digital input

#define BRAKE_LIGHT_PIN 2 // PD2 digital output
#define BUZZER_PIN 4      // PD4 digital output
#define DRIVE_LED_PIN 3   // PD3 digital output

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

    if (!startButtonPressed || brake <= BRAKE_DEPRESSED_THRESHOLD)
    {
      currentState = INIT;
      break;
    }

    if (millis() - stateStartTime >= STARTIN_HOLD_TIME)
    {
      currentState = BUZZIN;
      stateStartTime = millis();
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
    float scaledApps3V3 = (static_cast<float>(apps3V3) / 1023.0) * (5.0 / 3.3) * 1023.0;
    float diffPercent = abs(apps5V - scaledApps3V3) / static_cast<float>(PEDAL_MAX);

    if (diffPercent > APPS_FAULT_THRESHOLD)
    {
      if (!isFaulty)
      {
        faultStartTime = millis();
        isFaulty = true;
      }

      if (millis() - faultStartTime > APPS_FAULT_TIMEOUT)
      {
        currentState = INIT;
        calculatedTorque = 0;
        isFaulty = false;
        break;
      }
    }
    else
    {
      isFaulty = false;
    }

    // Calculate Torque
    int avgPedal = (apps5V + static_cast<int>(scaledApps3V3)) / 2;
    float pedalPercent = static_cast<float>(avgPedal - PEDAL_MIN) / (PEDAL_MAX - PEDAL_MIN);
    calculatedTorque = static_cast<int16_t>(pedalPercent * (TORQUE_MAX - TORQUE_MIN) + TORQUE_MIN);

    if (FLIP_MOTOR_DIRECTION)
    {
      calculatedTorque *= -1;
    }

    break;
  }

  delay(10);
}

// put function definitions here:
