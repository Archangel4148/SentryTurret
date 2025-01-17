#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 servo driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo channel definitions
const uint8_t SERVO1_CHANNEL = 0;
const uint8_t SERVO2_CHANNEL = 1;

// Servo angle limits (in microseconds for PCA9685 PWM)
const uint16_t SERVO_MIN = 500;  // Minimum pulse length (microseconds)
const uint16_t SERVO_MAX = 2500; // Maximum pulse length (microseconds)

// Function to map floating-point angle to pulse width
uint16_t angleToPulse(float angle) {
  return SERVO_MIN + (uint16_t)((angle / 180.0) * (SERVO_MAX - SERVO_MIN));
}

void setup() {
  Serial.begin(115200); // Set serial communication to high baud rate for speed
  pwm.begin();
  pwm.setPWMFreq(50); // Standard servo frequency
}

void loop() {
  static float angle1 = 90.0, angle2 = 90.0; // Default angles (90 degrees)
  static char buffer[32];
  static uint8_t bufferIndex = 0;

  // Read serial data if available
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { // End of input
      buffer[bufferIndex] = '\0'; // Null-terminate the string
      bufferIndex = 0;

      // Parse angles from the input buffer
      float newAngle1, newAngle2;
      if (sscanf(buffer, "%f %f", &newAngle1, &newAngle2) == 2) {
        angle1 = constrain(newAngle1, 0.0, 180.0);
        angle2 = constrain(newAngle2, 0.0, 180.0);

        // Update servo positions immediately
        pwm.setPWM(SERVO1_CHANNEL, 0, angleToPulse(angle1));
        pwm.setPWM(SERVO2_CHANNEL, 0, angleToPulse(angle2));
      }
    } else if (bufferIndex < sizeof(buffer) - 1) {
      buffer[bufferIndex++] = c; // Add character to buffer
    }
  }
}
