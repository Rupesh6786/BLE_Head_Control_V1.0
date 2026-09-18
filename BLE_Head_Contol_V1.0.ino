/*
 * =====================================================================================
 *  Project       : ESP32 Robotic Head Firmware (Direct PWM / Classic Bluetooth)
 *  Author        : Rupesh Thakur (https://github.com/Rupesh6786)
 *  Repository    : https://github.com/Rupesh6786/esp_quadruped_robot
 *  Target MCU    : ESP32 (WROOM-32 / ESP32-DevKit v1)
 *  Language      : C++ / Arduino Framework
 * =====================================================================================
 * 
 *  [OVERVIEW]
 *  Firmware for a 3-DOF Robotic Head (Neck, Eyes, Mouth) driven directly via ESP32 PWM 
 *  pins without external hardware servo drivers. Features smooth interpolation curves, 
 *  virtual pin command parser matching app protocols, and low-power optimization.
 * 
 *  [ROBOT TOPOLOGY & PINOUT]
 *   - Neck (V0)  : GPIO 2  (Base Rotation)
 *   - Eyes (V1)  : GPIO 4  (Pan Tracking / Gaze)
 *   - Mouth (V2) : GPIO 5  (Jaw Mechanism)
 * =====================================================================================
 */

#include "BluetoothSerial.h"
#include <ESP32Servo.h>
#include "animations.h"

BluetoothSerial SerialBT;

// ---------- Servos ----------
static const int numberOfServos = 3;                 
static const int servoPins[numberOfServos] = {2, 4, 5};  
/*
  servoPins[0] = 2  -> Neck (V0)
  servoPins[1] = 4  -> Eyes (V1)
  servoPins[2] = 5  -> Mouth (V2)
*/

Servo servos[numberOfServos];
int servoPos[numberOfServos] = { 90, 90, 0 }; // Initial positions
const int servoPrgPeriod = 20;                // 20 ms interpolation period

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
        long nextAngle = servoPos[s] + ( (long)(target[s] - servoPos[s]) * step ) / segments;
        int cmd = constrain(nextAngle, 0, 180);
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
    if (pinStr.equals("0")) pinNum = 0; // V0 -> Neck
    else if (pinStr.equals("1")) pinNum = 1; // V1 -> Eyes
    else if (pinStr.equals("2")) pinNum = 2; // V2 -> Mouth

    if (pinNum >= 0 && pinNum < numberOfServos) {
      // If it's the mouth switch (V2), treat value 1/0 as 180/0 degrees (or direct val if toggled)
      if (pinNum == 2 && (val == 0 || val == 1)) {
        val = (val == 1) ? 180 : 0; 
      }
      
      servoPos[pinNum] = constrain(val, 0, 180);
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

  // Attach servos and move to initial positions
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