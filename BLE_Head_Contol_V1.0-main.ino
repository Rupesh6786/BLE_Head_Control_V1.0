/*
- =====================================================================================
-  Project       : ESP32 Robotic Head Firmware (Direct PWM / Classic Bluetooth)
-  Author        : Rupesh Thakur (https://github.com/Rupesh6786)
-  Repository    : https://github.com/Rupesh6786/BLE_Head_Control_V1.0.git
-  Target MCU    : ESP32 (WROOM-32 / ESP32-DevKit v1)
-  Language      : C++ / Arduino Framework
- =====================================================================================
- 
-  [OVERVIEW]
-  Firmware for a 3-DOF Robotic Head (Neck, Eyes, Mouth) driven directly via ESP32 PWM 
-  pins without external hardware servo drivers. Features smooth interpolation curves, 
-  virtual pin command parser matching app protocols, and low-power optimization.
- 
-  [ROBOT TOPOLOGY & PINOUT & LIMITS]
- Neck (V0)  : GPIO 2  (Base Rotation) -> Limits: 40° to 140° (Neutral: 90°)
- Eyes (V1)  : GPIO 4  (Pan Tracking)  -> Limits: 50° to 115° (Neutral: 90°)
- Mouth (V2) : GPIO 5  (Jaw Mechanism) -> Limits: 0° to 60°   (Closed: 0°, Open: 60°)
- =====================================================================================
 */

#include "BluetoothSerial.h"
#include <ESP32Servo.h>

BluetoothSerial SerialBT;

// ---------- Servos & Pin Configuration ----------
static const int numberOfServos = 3;                 
static const int servoPins[numberOfServos] = {2, 4, 5};  
/*
  servoPins[0] = 2  -> Neck (V0)
  servoPins[1] = 4  -> Eyes (V1)
  servoPins[2] = 5  -> Mouth (V2)
*/

Servo servos[numberOfServos];

// ---------- Servo Min / Max & Neutral Limits ----------
// Neck Limits
const int NECK_MIN     = 40;  // Right
const int NECK_MAX     = 140; // Left
const int NECK_NEUTRAL = 90;

// Eyes Limits
const int EYES_MIN     = 50;  // Right
const int EYES_MAX     = 115; // Left
const int EYES_NEUTRAL = 90;

// Mouth Limits
const int MOUTH_CLOSED = 0;
const int MOUTH_OPEN   = 60;

// Initial positions {Neck, Eyes, Mouth}
int servoPos[numberOfServos] = { NECK_NEUTRAL, EYES_NEUTRAL, MOUTH_CLOSED }; 
const int servoPrgPeriod = 20; // 20 ms interpolation period

#include "animations.h" 

// ========== Helper: Run a sequence smoothly with interpolation ==========
void runHeadPrg(const int sequence[][HEAD_ACE], int steps) {
  for (int i = 0; i < steps; i++) {
    int target[numberOfServos];
    for (int s = 0; s < numberOfServos; s++) {
      target[s] = pgm_read_dword(&(sequence[i][s]));
    }
    int totalTime = pgm_read_dword(&(sequence[i][HEAD_SERVOS]));
    if (totalTime < servoPrgPeriod) totalTime = servoPrgPeriod;

    int segments = totalTime / servoPrgPeriod;
    if (segments < 1) segments = 1;

    for (int step = 1; step <= segments; step++) {
      for (int s = 0; s < numberOfServos; s++) {
        // Interpolation calculation with fixed multiplication operator (*)
        long nextAngle = servoPos[s] + ((long)(target[s] - servoPos[s]) * step) / segments;
        
        // Constrain each servo according to its specific safe hardware limits
        int cmd = nextAngle;
        if (s == 0) cmd = constrain(nextAngle, NECK_MIN, NECK_MAX);
        else if (s == 1) cmd = constrain(nextAngle, EYES_MIN, EYES_MAX);
        else if (s == 2) cmd = constrain(nextAngle, MOUTH_CLOSED, MOUTH_OPEN);

        servos[s].write(cmd);
      }
      delay(servoPrgPeriod);
    }

    for (int s = 0; s < numberOfServos; s++) {
      servoPos[s] = target[s];
    }
  }
}

// ========== Parse Incoming Bluetooth Data ==========
void parseBluetoothCommand(String data) {
  data.trim();
  if (data.length() == 0) return;

  Serial.println("Received BT Data: " + data);

  // 1. Check if it's a slider / pin format (e.g., "V0:120" or "V2:1")
  if (data.startsWith("V") && data.indexOf(':') != -1) {
    int colonIdx = data.indexOf(':');
    String pinStr = data.substring(1, colonIdx); // Extracts '0', '1', or '2'
    int val = data.substring(colonIdx + 1).toInt();

    int pinNum = -1;
    if (pinStr.equals("0")) pinNum = 0;      // V0 -> Neck
    else if (pinStr.equals("1")) pinNum = 1; // V1 -> Eyes
    else if (pinStr.equals("2")) pinNum = 2; // V2 -> Mouth

    if (pinNum >= 0 && pinNum < numberOfServos) {
      // Apply specific min/max limit mappings based on the servo pin
      if (pinNum == 0) {
        // Neck: map toggle or constrain within 40 to 140
        if (val == 1) val = NECK_MAX;
        else if (val == 0) val = NECK_NEUTRAL;
        servoPos[pinNum] = constrain(val, NECK_MIN, NECK_MAX);
      } 
      else if (pinNum == 1) {
        // Eyes: map toggle or constrain within 50 to 115
        if (val == 1) val = EYES_MAX;
        else if (val == 0) val = EYES_NEUTRAL;
        servoPos[pinNum] = constrain(val, EYES_MIN, EYES_MAX);
      } 
      else if (pinNum == 2) {
        // Mouth: toggle 1/0 maps to Open (60°) / Closed (0°)
        if (val == 1 || val == 180) val = MOUTH_OPEN;
        else val = MOUTH_CLOSED;
        servoPos[pinNum] = constrain(val, MOUTH_CLOSED, MOUTH_OPEN);
      }

      servos[pinNum].write(servoPos[pinNum]);
      Serial.printf("Head Servo %d set to %d°\n", pinNum, servoPos[pinNum]);
    }
  } 
  // 2. Otherwise, treat it as an animation preset command
  else {
    int command = data.toInt();
    switch (command) {
      case 0:
        Serial.println("Executing: Neutral Position");
        runHeadPrg(headPrg00, headPrg00step);
        break;
      case 1:
        Serial.println("Executing: Scan Motion");
        runHeadPrg(headPrg01, headPrg01step);
        break;
      case 2:
        Serial.println("Executing: Talk/Expression");
        runHeadPrg(headPrg02, headPrg02step);
        break;
      default:
        Serial.println("⚠️ Invalid head command!");
        break;
    }
  }
}

// ========== Setup ==========
void setup() {
  Serial.begin(115200);
  delay(300);

  // Start Classic Bluetooth Serial matching app connection name
  SerialBT.begin("ESP32-BT-ROBOTIC-HEAD");
  Serial.println("\nBluetooth device started: ESP32-BT-ROBOTIC-HEAD");

  // Attach servos and move to initial positions safely
  for (int i = 0; i < numberOfServos; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(servoPos[i]);
  }

  delay(300);
  Serial.println("Moving to Neutral Setup...");
  runHeadPrg(headPrg00, headPrg00step);
}

// ========== Loop ==========
void loop() {
  if (SerialBT.available()) {
    String dataIn = SerialBT.readStringUntil('\n');
    parseBluetoothCommand(dataIn);
  }
}
