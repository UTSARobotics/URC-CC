#include <Servo.h>

// SCARA workshop sketch
// This version is designed for beginners.
// It controls three servos directly with simple keyboard commands.

// ---------------- PINS ----------------
const byte JOINT1_PIN = 9;
const byte JOINT2_PIN = 10;
const byte LIFT_PIN = 11;

// ---------------- SAFE LIMITS ----------------
// Change these if your robot needs smaller ranges.
const int JOINT1_MIN = 0;
const int JOINT1_MAX = 180;
const int JOINT2_MIN = 0;
const int JOINT2_MAX = 180;
const int LIFT_MIN = 0;
const int LIFT_MAX = 180;

// ---------------- HOME POSITION ----------------
// These are the startup angles.
int joint1Angle = 90;
int joint2Angle = 90;
int liftAngle = 90;

// ---------------- MOVEMENT ----------------
// How many degrees to move each time a key is pressed.
const int STEP_SIZE = 5;

Servo joint1Servo;
Servo joint2Servo;
Servo liftServo;

int clampAngle(int angleValue, int minAngle, int maxAngle) {
  if (angleValue < minAngle) {
    return minAngle;
  }

  if (angleValue > maxAngle) {
    return maxAngle;
  }

  return angleValue;
}

void printHelp() {
  Serial.println();
  Serial.println(F("SCARA Workshop Control"));
  Serial.println(F("----------------------"));
  Serial.println(F("q = joint 1 up"));
  Serial.println(F("a = joint 1 down"));
  Serial.println(F("w = joint 2 up"));
  Serial.println(F("s = joint 2 down"));
  Serial.println(F("e = lift up"));
  Serial.println(F("d = lift down"));
  Serial.println(F("r = reset to home position"));
  Serial.println(F("p = print current angles"));
  Serial.println(F("h = show this help again"));
  Serial.println();
}

void printAngles() {
  Serial.print(F("Joint 1: "));
  Serial.print(joint1Angle);
  Serial.print(F("  Joint 2: "));
  Serial.print(joint2Angle);
  Serial.print(F("  Lift: "));
  Serial.println(liftAngle);
}

void setup() {
  Serial.begin(115200);

  joint1Servo.attach(JOINT1_PIN);
  joint2Servo.attach(JOINT2_PIN);
  liftServo.attach(LIFT_PIN);

  joint1Servo.write(joint1Angle);
  joint2Servo.write(joint2Angle);
  liftServo.write(liftAngle);

  delay(500);

  printHelp();
  printAngles();
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'q') {
      joint1Angle = joint1Angle + STEP_SIZE;
    }

    if (command == 'a') {
      joint1Angle = joint1Angle - STEP_SIZE;
    }

    if (command == 'w') {
      joint2Angle = joint2Angle + STEP_SIZE;
    }

    if (command == 's') {
      joint2Angle = joint2Angle - STEP_SIZE;
    }

    if (command == 'e') {
      liftAngle = liftAngle + STEP_SIZE;
    }

    if (command == 'd') {
      liftAngle = liftAngle - STEP_SIZE;
    }

    if (command == 'r') {
      joint1Angle = 90;
      joint2Angle = 90;
      liftAngle = 90;
      Serial.println(F("Back to home position."));
    }

    if (command == 'h') {
      printHelp();
    }

    if (command == 'p') {
      printAngles();
    }

    joint1Angle = clampAngle(joint1Angle, JOINT1_MIN, JOINT1_MAX);
    joint2Angle = clampAngle(joint2Angle, JOINT2_MIN, JOINT2_MAX);
    liftAngle = clampAngle(liftAngle, LIFT_MIN, LIFT_MAX);

    joint1Servo.write(joint1Angle);
    joint2Servo.write(joint2Angle);
    liftServo.write(liftAngle);

    if (command == 'q' || command == 'a' ||
        command == 'w' || command == 's' ||
        command == 'e' || command == 'd' ||
        command == 'r') {
      printAngles();
    }
  }
}
