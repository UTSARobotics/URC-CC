#include <Servo.h>
#include <math.h>

// ---------------- PINS ----------------
const byte JOINT1_PIN = 9;
const byte JOINT2_PIN = 10;
const byte LIFT_PIN   = 11;

// ---------------- ARM LENGTHS (mm) ----------------
const float LINK1 = 35.0;
const float LINK2 = 42.0;

// ---------------- POSITION ----------------
float posX = 50;
float posY = 20;

// ---------------- JOINT STATE ----------------
int joint1Angle = 90;
int joint2Angle = 90;
int liftAngle   = 90;

// ---------------- LIMITS ----------------
const int JOINT1_MIN = 0;
const int JOINT1_MAX = 180;
const int JOINT2_MIN = 0;
const int JOINT2_MAX = 180;
const int LIFT_MIN   = 0;
const int LIFT_MAX   = 180;

// ---------------- MOTION SETTINGS ----------------
const float STEP_SIZE = 10;
const int INTERP_STEPS = 50;
const int DELAY_MS = 40;

// ---------------- SERVOS ----------------
Servo joint1Servo;
Servo joint2Servo;
Servo liftServo;

// ---------------- UTILS ----------------
int clampAngle(int v, int minA, int maxA) {
  if (v < minA) return minA;
  if (v > maxA) return maxA;
  return v;
}

int angleDiff(int a, int b) {
  int d = abs(a - b);
  return min(d, 360 - d);
}

// ---------------- SMART IK ----------------
bool computeIKSmart(float x, float y, int &theta1, int &theta2) {
  float distSq = x * x + y * y;
  float cosTheta2 = (distSq - LINK1 * LINK1 - LINK2 * LINK2) / (2 * LINK1 * LINK2);

  if (cosTheta2 < -1 || cosTheta2 > 1) return false;

  float t2a = acos(cosTheta2);
  float t2b = -acos(cosTheta2);

  float t1a = atan2(y, x) - atan2(LINK2 * sin(t2a), LINK1 + LINK2 * cos(t2a));
  float t1b = atan2(y, x) - atan2(LINK2 * sin(t2b), LINK1 + LINK2 * cos(t2b));

  int t1a_deg = degrees(t1a);
  int t2a_deg = degrees(t2a);

  int t1b_deg = degrees(t1b);
  int t2b_deg = degrees(t2b);

  int costA = angleDiff(t1a_deg, joint1Angle) + angleDiff(t2a_deg, joint2Angle);
  int costB = angleDiff(t1b_deg, joint1Angle) + angleDiff(t2b_deg, joint2Angle);

  if (costA <= costB) {
    theta1 = t1a_deg;
    theta2 = t2a_deg;
  } else {
    theta1 = t1b_deg;
    theta2 = t2b_deg;
  }

  return true;
}

// ---------------- MOVE SINGLE POINT ----------------
bool moveToXY(float x, float y) {
  int t1, t2;

  if (!computeIKSmart(x, y, t1, t2)) {
    Serial.println(F("Target out of reach!"));
    return false;
  }

  joint1Angle = clampAngle(t1, JOINT1_MIN, JOINT1_MAX);
  joint2Angle = clampAngle(t2, JOINT2_MIN, JOINT2_MAX);

  joint1Servo.write(joint1Angle);
  joint2Servo.write(joint2Angle);

  posX = x;
  posY = y;

  return true;
}

// ---------------- LINEAR INTERPOLATION ----------------
void moveLinearXY(float targetX, float targetY) {
  float startX = posX;
  float startY = posY;

  for (int i = 1; i <= INTERP_STEPS; i++) {
    float t = (float)i / INTERP_STEPS;

    float xi = startX + t * (targetX - startX);
    float yi = startY + t * (targetY - startY);

    moveToXY(xi, yi);
    delay(DELAY_MS);
  }
}

// ---------------- DRAW SQUARE ----------------
void drawSquare(float size) {
  float startX = posX;
  float startY = posY;

  moveLinearXY(startX + size, startY);
  moveLinearXY(startX + size, startY + size);
  moveLinearXY(startX, startY + size);
  moveLinearXY(startX, startY);
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

  printStatus();
}

// ---------------- LOOP ----------------
void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    // -------- LINEAR MOVE --------
    if (command == 'g') {
      while (Serial.available() == 0);
      float x = Serial.parseFloat();

      while (Serial.available() == 0);
      float y = Serial.parseFloat();

      Serial.print(F("Linear move to: "));
      Serial.print(x);
      Serial.print(F(", "));
      Serial.println(y);

      moveLinearXY(x, y);
    }

    // -------- MANUAL STEP --------
    if (command == 'w') moveLinearXY(posX, posY + STEP_SIZE);
    if (command == 's') moveLinearXY(posX, posY - STEP_SIZE);
    if (command == 'a') moveLinearXY(posX - STEP_SIZE, posY);
    if (command == 'd') moveLinearXY(posX + STEP_SIZE, posY);

    // -------- SQUARE --------
    if (command == 'q') drawSquare(20);

    // -------- LIFT --------
    if (command == 'e') liftAngle += 5;
    if (command == 'c') liftAngle -= 5;

    // -------- RESET --------
    if (command == 'r') {
      moveLinearXY(50, 20);
      liftAngle = 90;
    }

    liftAngle = clampAngle(liftAngle, LIFT_MIN, LIFT_MAX);
    liftServo.write(liftAngle);

    printStatus();
  }
}