#include <Servo.h>

// Servo test sketch for troubleshooting.
// It sweeps one servo at a time so you can watch each joint by itself.

const byte JOINT1_PIN = 9;
const byte JOINT2_PIN = 10;
const byte JOINT3_PIN = 11;

// Use a small range first to avoid hitting hard stops.
const int SWEEP_MIN = 70;
const int SWEEP_MAX = 110;
const int CENTER_ANGLE = 90;

const int STEP_DELAY_MS = 25;
const int HOLD_DELAY_MS = 700;

Servo joint1Servo;
Servo joint2Servo;
Servo joint3Servo;

void detachAllServos() {
  joint1Servo.detach();
  joint2Servo.detach();
  joint3Servo.detach();
}

void sweepServo(Servo &servoToTest, byte pin, const __FlashStringHelper *name) {
  Serial.print(F("Testing "));
  Serial.println(name);

  detachAllServos();
  servoToTest.attach(pin);
  servoToTest.write(CENTER_ANGLE);
  delay(HOLD_DELAY_MS);

  for (int angle = SWEEP_MIN; angle <= SWEEP_MAX; angle++) {
    servoToTest.write(angle);
    delay(STEP_DELAY_MS);
  }

  delay(HOLD_DELAY_MS);

  for (int angle = SWEEP_MAX; angle >= SWEEP_MIN; angle--) {
    servoToTest.write(angle);
    delay(STEP_DELAY_MS);
  }

  delay(HOLD_DELAY_MS);

  servoToTest.write(CENTER_ANGLE);
  delay(HOLD_DELAY_MS);
  servoToTest.detach();
}

void setup() {
  Serial.begin(115200);

  detachAllServos();
  delay(1000);

  Serial.println();
  Serial.println(F("SCARA Servo Sweep Test"));
  Serial.println(F("Each servo moves by itself."));
  Serial.println(F("Watch for twitching, buzzing, or missed movement."));
  Serial.println();
}

void loop() {
  sweepServo(joint1Servo, JOINT1_PIN, F("joint 1 on pin 9"));
  delay(1000);

  sweepServo(joint2Servo, JOINT2_PIN, F("joint 2 on pin 10"));
  delay(1000);

  sweepServo(joint3Servo, JOINT3_PIN, F("joint 3 on pin 11"));
  delay(1000);
}
