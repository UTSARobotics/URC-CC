#include <Servo.h>
#include <math.h>

// ---------------- PINS ----------------
const byte JOINT1_PIN = 9;
const byte JOINT2_PIN = 10;
const byte LIFT_PIN = 11;

// ---------------- ARM LENGTHS ----------------
const float LINK1 = 100.0;
const float LINK2 = 100.0;

// ---------------- POSITION ----------------
float posX = 100;
float posY = 0;

// ---------------- SAFE LIMITS ----------------
const int JOINT1_MIN = 0;
const int JOINT1_MAX = 180;
const int JOINT2_MIN = 0;
const int JOINT2_MAX = 180;
const int LIFT_MIN = 0;
const int LIFT_MAX = 180;

// ---------------- HOME ----------------
int joint1Angle = 90;
int joint2Angle = 90;
int liftAngle = 90;

// movement step in mm
const float STEP_SIZE = 10;

Servo joint1Servo;
Servo joint2Servo;
Servo liftServo;

// ---------------- CLAMP ----------------
int clampAngle(int angleValue, int minAngle, int maxAngle) {
  if (angleValue < minAngle) return minAngle;
  if (angleValue > maxAngle) return maxAngle;
  return angleValue;
}

// ---------------- IK FUNCTION ----------------
bool computeIK(float x, float y, int &theta1, int &theta2) {
  float distSq = x * x + y * y;
  float cosTheta2 = (distSq - LINK1 * LINK1 - LINK2 * LINK2) / (2 * LINK1 * LINK2);

  // unreachable position
  if (cosTheta2 < -1 || cosTheta2 > 1) {
    return false;
  }

  float theta2Rad = acos(cosTheta2);
  float theta1Rad = atan2(y, x) - atan2(LINK2 * sin(theta2Rad), LINK1 + LINK2 * cos(theta2Rad));

  theta1 = degrees(theta1Rad);
  theta2 = degrees(theta2Rad);

  return true;
}

// ---------------- PRINT ----------------
void printStatus() {
  Serial.print(F("X: "));
  Serial.print(posX);
  Serial.print(F(" Y: "));
  Serial.print(posY);
  Serial.print(F(" | J1: "));
  Serial.print(joint1Angle);
  Serial.print(F(" J2: "));
  Serial.print(joint2Angle);
  Serial.print(F(" Lift: "));
  Serial.println(liftAngle);
}

void printHelp() {
  Serial.println();
  Serial.println(F("SCARA IK CONTROL"));
  Serial.println(F("----------------"));
  Serial.println(F("w/s = Y up/down"));
  Serial.println(F("a/d = X left/right"));
  Serial.println(F("e/d = lift up/down"));
  Serial.println(F("r = home"));
  Serial.println(F("p = print status"));
  Serial.println();
}

// ---------------- SETUP ----------------
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
  printStatus();
}

// ---------------- LOOP ----------------
void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    // Move in XY space
    if (command == 'w') posY += STEP_SIZE;
    if (command == 's') posY -= STEP_SIZE;
    if (command == 'a') posX -= STEP_SIZE;
    if (command == 'd') posX += STEP_SIZE;

    // Lift
    if (command == 'e') liftAngle += 5;
    if (command == 'c') liftAngle -= 5;

    // Reset
    if (command == 'r') {
      posX = 100;
      posY = 0;
      liftAngle = 90;
    }

    // Compute IK
    int t1, t2;
    if (computeIK(posX, posY, t1, t2)) {
      joint1Angle = clampAngle(t1, JOINT1_MIN, JOINT1_MAX);
      joint2Angle = clampAngle(t2, JOINT2_MIN, JOINT2_MAX);

      joint1Servo.write(joint1Angle);
      joint2Servo.write(joint2Angle);
    } else {
      Serial.println(F("Target out of reach!"));
    }

    liftAngle = clampAngle(liftAngle, LIFT_MIN, LIFT_MAX);
    liftServo.write(liftAngle);

    if (command == 'p' || command == 'w' || command == 'a' ||
        command == 's' || command == 'd' || command == 'r') {
      printStatus();
    }
  }
}