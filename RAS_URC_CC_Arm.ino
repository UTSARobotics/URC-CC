#include <Servo.h>
#include <math.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// ---------------- PINS ----------------
const byte JOINT1_PIN = 9;
const byte JOINT2_PIN = 10;
const byte LIFT_PIN = 11;

// ---------------- GEOMETRY ----------------
const float LINK1 = 35.0f;
const float LINK2 = 42.0f;

// ---------------- LIMITS ----------------
const int SERVO_MIN = 0;
const int SERVO_MAX = 180;
const float JOINT_MIN_DEG = 0.0f;
const float JOINT_MAX_DEG = 180.0f;
const float MIN_RADIUS_MM = fabs(LINK1 - LINK2);
const float MAX_RADIUS_MM = LINK1 + LINK2;
const float START_JOINT1 = 37.0f;
const float START_JOINT2 = 27.5f;
const float STARTUP_SQUARE_SIZE_MM = 20.0f;
const bool RUN_STARTUP_SQUARE = false;

// ---------------- SOLVER / MOTION ----------------
const int LINE_SUBPOINTS = 100;
const int SUBPOINT_DELAY_MS = 5;
const int GN_MAX_ITERS = 30;
const float GN_TOLERANCE_MM = 0.01f;
const float GN_SUCCESS_MM = 0.25f;
const float GN_DAMPING = 0.001f;
const int DEFAULT_CIRCLE_SEGMENTS = 24;
const int MIN_CIRCLE_SEGMENTS = 8;

// ---------------- SERIAL ----------------
const unsigned long SERIAL_BAUD = 115200;
const size_t INPUT_BUFFER_SIZE = 256;

struct Calibration {
  int joint1Offset;
  int joint2Offset;
  bool joint1Invert;
  bool joint2Invert;
  int penUp;
  int penDown;
};

struct MotionState {
  float originAbsX;
  float originAbsY;
  float currentRelX;
  float currentRelY;
  float previousRelX;
  float previousRelY;
  float joint1Estimate;
  float joint2Estimate;
  int penAngle;
  bool penIsDown;
};

Calibration calibration = {
  90,
  90,
  false,
  false,
  110,
  70
};

MotionState state = {
  0.0f,
  0.0f,
  0.0f,
  0.0f,
  0.0f,
  0.0f,
  START_JOINT1,
  START_JOINT2,
  70,
  false
};

Servo joint1Servo;
Servo joint2Servo;
Servo liftServo;

char inputBuffer[INPUT_BUFFER_SIZE];
size_t inputLength = 0;

int clampValue(int value, int minValue, int maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

void trimLine(char *line) {
  size_t len = strlen(line);
  while (len > 0 && isspace(static_cast<unsigned char>(line[len - 1]))) {
    line[--len] = '\0';
  }

  size_t start = 0;
  while (line[start] != '\0' && isspace(static_cast<unsigned char>(line[start]))) {
    start++;
  }

  if (start > 0) {
    memmove(line, line + start, strlen(line + start) + 1);
  }
}

void uppercaseLine(char *line) {
  for (size_t i = 0; line[i] != '\0'; i++) {
    line[i] = static_cast<char>(toupper(static_cast<unsigned char>(line[i])));
  }
}

char *nextToken(char *&cursor) {
  while (*cursor != '\0' && isspace(static_cast<unsigned char>(*cursor))) {
    cursor++;
  }

  if (*cursor == '\0') {
    return NULL;
  }

  char *token = cursor;
  while (*cursor != '\0' && !isspace(static_cast<unsigned char>(*cursor))) {
    cursor++;
  }

  if (*cursor != '\0') {
    *cursor = '\0';
    cursor++;
  }

  return token;
}

bool parseFloatArg(char *token, float &value) {
  if (token == NULL) return false;
  char *endPtr = NULL;
  value = static_cast<float>(strtod(token, &endPtr));
  return endPtr != token && *endPtr == '\0';
}

bool parseIntArg(char *token, int &value) {
  if (token == NULL) return false;
  char *endPtr = NULL;
  long parsed = strtol(token, &endPtr, 10);
  if (endPtr == token || *endPtr != '\0') return false;
  value = static_cast<int>(parsed);
  return true;
}

void printOk(const __FlashStringHelper *message) {
  Serial.print(F("OK "));
  Serial.println(message);
}

void printError(const __FlashStringHelper *message) {
  Serial.print(F("ERR "));
  Serial.println(message);
}

int calibrateJointAngle(float logicalAngle, int offset, bool invert) {
  int rounded = static_cast<int>(round(logicalAngle));
  int angle = invert ? (180 - rounded) : rounded;
  return clampValue(angle + offset - 90, SERVO_MIN, SERVO_MAX);
}

void writeCurrentServoState() {
  joint1Servo.write(calibrateJointAngle(state.joint1Estimate, calibration.joint1Offset, calibration.joint1Invert));
  joint2Servo.write(calibrateJointAngle(state.joint2Estimate, calibration.joint2Offset, calibration.joint2Invert));
  liftServo.write(clampValue(state.penAngle, SERVO_MIN, SERVO_MAX));
}

void forwardKinematics(float joint1Deg, float joint2Deg, float &x, float &y) {
  float q1 = (joint1Deg - 90.0f) * DEG_TO_RAD;
  float q2 = (joint2Deg - 90.0f) * DEG_TO_RAD;
  x = (LINK1 * cos(q1)) + (LINK2 * cos(q1 + q2));
  y = (LINK1 * sin(q1)) + (LINK2 * sin(q1 + q2));
}

bool isAbsolutePointReachable(float absX, float absY) {
  float radius = sqrt((absX * absX) + (absY * absY));
  return radius >= (MIN_RADIUS_MM - 0.001f) && radius <= (MAX_RADIUS_MM + 0.001f);
}

bool solveIKAnalytic(float targetAbsX, float targetAbsY, float currentJoint1, float currentJoint2,
                     float &seedJoint1, float &seedJoint2) {
  float distSq = (targetAbsX * targetAbsX) + (targetAbsY * targetAbsY);
  float cosQ2 = (distSq - (LINK1 * LINK1) - (LINK2 * LINK2)) / (2.0f * LINK1 * LINK2);
  if (cosQ2 < -1.0f || cosQ2 > 1.0f) {
    return false;
  }

  float q2Options[2] = {acos(cosQ2), -acos(cosQ2)};
  bool found = false;
  float bestCost = 0.0f;

  for (int i = 0; i < 2; i++) {
    float q2 = q2Options[i];
    float q1 = atan2(targetAbsY, targetAbsX) - atan2(LINK2 * sin(q2), LINK1 + (LINK2 * cos(q2)));
    float joint1 = (q1 * RAD_TO_DEG) + 90.0f;
    float joint2 = (q2 * RAD_TO_DEG) + 90.0f;

    if (joint1 < JOINT_MIN_DEG || joint1 > JOINT_MAX_DEG) continue;
    if (joint2 < JOINT_MIN_DEG || joint2 > JOINT_MAX_DEG) continue;

    float cost = fabs(joint1 - currentJoint1) + fabs(joint2 - currentJoint2);
    if (!found || cost < bestCost) {
      found = true;
      bestCost = cost;
      seedJoint1 = clampFloat(joint1, JOINT_MIN_DEG, JOINT_MAX_DEG);
      seedJoint2 = clampFloat(joint2, JOINT_MIN_DEG, JOINT_MAX_DEG);
    }
  }

  return found;
}

void absoluteToRelative(float absX, float absY, float &relX, float &relY) {
  float dx = absX - state.originAbsX;
  float dy = absY - state.originAbsY;
  // Rotate internal Cartesian axes so the physical 90/90 "up" direction is +Y for the user.
  relX = -dy;
  relY = dx;
}

void relativeToAbsolute(float relX, float relY, float &absX, float &absY) {
  // Inverse rotation of the user-facing frame back into the solver's internal frame.
  absX = state.originAbsX + relY;
  absY = state.originAbsY - relX;
}

bool solveIKGaussNewton(float targetAbsX, float targetAbsY, float &joint1Deg, float &joint2Deg) {
  if (!isAbsolutePointReachable(targetAbsX, targetAbsY)) {
    return false;
  }

  float seedJoint1;
  float seedJoint2;
  if (solveIKAnalytic(targetAbsX, targetAbsY, joint1Deg, joint2Deg, seedJoint1, seedJoint2)) {
    joint1Deg = seedJoint1;
    joint2Deg = seedJoint2;
  }

  float q1 = (joint1Deg - 90.0f) * DEG_TO_RAD;
  float q2 = (joint2Deg - 90.0f) * DEG_TO_RAD;

  for (int iter = 0; iter < GN_MAX_ITERS; iter++) {
    float c1 = cos(q1);
    float s1 = sin(q1);
    float c12 = cos(q1 + q2);
    float s12 = sin(q1 + q2);

    float x = (LINK1 * c1) + (LINK2 * c12);
    float y = (LINK1 * s1) + (LINK2 * s12);
    float ex = targetAbsX - x;
    float ey = targetAbsY - y;
    float errorNorm = sqrt((ex * ex) + (ey * ey));

    if (errorNorm < GN_TOLERANCE_MM) {
      joint1Deg = clampFloat((q1 * RAD_TO_DEG) + 90.0f, JOINT_MIN_DEG, JOINT_MAX_DEG);
      joint2Deg = clampFloat((q2 * RAD_TO_DEG) + 90.0f, JOINT_MIN_DEG, JOINT_MAX_DEG);
      return true;
    }

    float j11 = (-LINK1 * s1) - (LINK2 * s12);
    float j12 = -LINK2 * s12;
    float j21 = (LINK1 * c1) + (LINK2 * c12);
    float j22 = LINK2 * c12;

    float a11 = (j11 * j11) + (j21 * j21) + GN_DAMPING;
    float a12 = (j11 * j12) + (j21 * j22);
    float a22 = (j12 * j12) + (j22 * j22) + GN_DAMPING;
    float b1 = (j11 * ex) + (j21 * ey);
    float b2 = (j12 * ex) + (j22 * ey);
    float det = (a11 * a22) - (a12 * a12);

    if (fabs(det) < 0.000001f) {
      return false;
    }

    q1 += ((a22 * b1) - (a12 * b2)) / det;
    q2 += ((a11 * b2) - (a12 * b1)) / det;

    q1 = (clampFloat((q1 * RAD_TO_DEG) + 90.0f, JOINT_MIN_DEG, JOINT_MAX_DEG) - 90.0f) * DEG_TO_RAD;
    q2 = (clampFloat((q2 * RAD_TO_DEG) + 90.0f, JOINT_MIN_DEG, JOINT_MAX_DEG) - 90.0f) * DEG_TO_RAD;
  }

  float finalX;
  float finalY;
  forwardKinematics((q1 * RAD_TO_DEG) + 90.0f, (q2 * RAD_TO_DEG) + 90.0f, finalX, finalY);
  float ex = targetAbsX - finalX;
  float ey = targetAbsY - finalY;
  if (sqrt((ex * ex) + (ey * ey)) > GN_SUCCESS_MM) {
    return false;
  }

  joint1Deg = clampFloat((q1 * RAD_TO_DEG) + 90.0f, JOINT_MIN_DEG, JOINT_MAX_DEG);
  joint2Deg = clampFloat((q2 * RAD_TO_DEG) + 90.0f, JOINT_MIN_DEG, JOINT_MAX_DEG);
  return true;
}

bool linePathIsReachable(float startRelX, float startRelY, float targetRelX, float targetRelY) {
  float testJoint1 = state.joint1Estimate;
  float testJoint2 = state.joint2Estimate;

  for (int i = 1; i <= LINE_SUBPOINTS; i++) {
    float t = static_cast<float>(i) / static_cast<float>(LINE_SUBPOINTS);
    float relX = startRelX + ((targetRelX - startRelX) * t);
    float relY = startRelY + ((targetRelY - startRelY) * t);
    float absX;
    float absY;
    relativeToAbsolute(relX, relY, absX, absY);

    if (!solveIKGaussNewton(absX, absY, testJoint1, testJoint2)) {
      return false;
    }
  }
  return true;
}

bool moveToAbsolute(float absX, float absY) {
  float nextJoint1 = state.joint1Estimate;
  float nextJoint2 = state.joint2Estimate;

  if (!solveIKGaussNewton(absX, absY, nextJoint1, nextJoint2)) {
    printError(F("TARGET OUT OF REACH"));
    return false;
  }

  state.joint1Estimate = nextJoint1;
  state.joint2Estimate = nextJoint2;
  absoluteToRelative(absX, absY, state.currentRelX, state.currentRelY);
  writeCurrentServoState();
  return true;
}

bool executeLinearG(float targetRelX, float targetRelY) {
  float startRelX = state.currentRelX;
  float startRelY = state.currentRelY;

  if (!linePathIsReachable(startRelX, startRelY, targetRelX, targetRelY)) {
    printError(F("TARGET OUT OF REACH"));
    return false;
  }

  state.previousRelX = startRelX;
  state.previousRelY = startRelY;

  for (int i = 1; i <= LINE_SUBPOINTS; i++) {
    float t = static_cast<float>(i) / static_cast<float>(LINE_SUBPOINTS);
    float stepRelX = startRelX + ((targetRelX - startRelX) * t);
    float stepRelY = startRelY + ((targetRelY - startRelY) * t);
    float stepAbsX;
    float stepAbsY;

    relativeToAbsolute(stepRelX, stepRelY, stepAbsX, stepAbsY);
    if (!moveToAbsolute(stepAbsX, stepAbsY)) {
      return false;
    }

    delay(SUBPOINT_DELAY_MS);
  }

  return true;
}

bool drawStartupSquare() {
  const float halfSize = STARTUP_SQUARE_SIZE_MM * 0.5f;

  if (!executeLinearG(-halfSize, -halfSize)) return false;
  setPenState(true);
  if (!executeLinearG(halfSize, -halfSize)) return false;
  if (!executeLinearG(halfSize, halfSize)) return false;
  if (!executeLinearG(-halfSize, halfSize)) return false;
  if (!executeLinearG(-halfSize, -halfSize)) return false;
  setPenState(false);
  if (!executeLinearG(0.0f, 0.0f)) return false;
  return true;
}

bool drawCircleRelative(float centerX, float centerY, float radius, int segments) {
  if (radius <= 0.0f) {
    printError(F("C REQUIRES POSITIVE RADIUS"));
    return false;
  }

  if (segments < MIN_CIRCLE_SEGMENTS) {
    printError(F("C SEGMENTS TOO LOW"));
    return false;
  }

  if (!executeLinearG(centerX + radius, centerY)) return false;
  setPenState(true);

  for (int i = 1; i <= segments; i++) {
    float angle = (TWO_PI * i) / segments;
    float x = centerX + (radius * cos(angle));
    float y = centerY + (radius * sin(angle));
    if (!executeLinearG(x, y)) {
      setPenState(false);
      return false;
    }
  }

  setPenState(false);
  return true;
}

void setPenState(bool penDown) {
  state.penIsDown = penDown;
  state.penAngle = penDown ? calibration.penDown : calibration.penUp;
  state.penAngle = clampValue(state.penAngle, SERVO_MIN, SERVO_MAX);
  writeCurrentServoState();
}

void setRawPenAngle(int angle) {
  state.penAngle = clampValue(angle, SERVO_MIN, SERVO_MAX);
  state.penIsDown = (abs(state.penAngle - calibration.penDown) <= abs(state.penAngle - calibration.penUp));
  writeCurrentServoState();
}

void printStatus() {
  float absX;
  float absY;
  forwardKinematics(state.joint1Estimate, state.joint2Estimate, absX, absY);

  Serial.print(F("STATUS X="));
  Serial.print(state.currentRelX, 2);
  Serial.print(F(" Y="));
  Serial.print(state.currentRelY, 2);
  Serial.print(F(" ABS_X="));
  Serial.print(absX, 2);
  Serial.print(F(" ABS_Y="));
  Serial.print(absY, 2);
  Serial.print(F(" PREV_X="));
  Serial.print(state.previousRelX, 2);
  Serial.print(F(" PREV_Y="));
  Serial.print(state.previousRelY, 2);
  Serial.print(F(" J1="));
  Serial.print(state.joint1Estimate, 2);
  Serial.print(F(" J2="));
  Serial.print(state.joint2Estimate, 2);
  Serial.print(F(" PEN="));
  Serial.print(state.penAngle);
  Serial.print(F(" STATE="));
  Serial.println(state.penIsDown ? F("DOWN") : F("UP"));
}

void printCalibration() {
  Serial.print(F("CAL J1_OFFSET="));
  Serial.print(calibration.joint1Offset);
  Serial.print(F(" J2_OFFSET="));
  Serial.print(calibration.joint2Offset);
  Serial.print(F(" J1_INVERT="));
  Serial.print(calibration.joint1Invert ? 1 : 0);
  Serial.print(F(" J2_INVERT="));
  Serial.print(calibration.joint2Invert ? 1 : 0);
  Serial.print(F(" PEN_UP="));
  Serial.print(calibration.penUp);
  Serial.print(F(" PEN_DOWN="));
  Serial.println(calibration.penDown);
}

void printHelp() {
  Serial.println(F("COMMANDS: G x y, A a1 a2, HOME, STATUS, PEN UP, PEN DOWN, Z angle, CAL GET"));
  Serial.println(F("C centerX centerY radius [segments] DRAWS A CIRCLE IN USER COORDINATES"));
  Serial.println(F("CAL SET J1_OFFSET n, CAL SET J2_OFFSET n, CAL SET J1_INVERT 0|1"));
  Serial.println(F("CAL SET J2_INVERT 0|1, CAL SET PEN_UP n, CAL SET PEN_DOWN n"));
  Serial.println(F("HOME IS THE CENTERED START POSE; G 0 0 GOES TO THAT CENTER"));
  Serial.println(F("USER FRAME IS ROTATED TO MATCH THE ARM: A 90 90 IS 'UP'"));
  Serial.println(F("G +X IS FORWARD/RIGHT, G -X IS BACK/LEFT, G +Y IS UP, G -Y IS DOWN"));
  Serial.println(F("A a1 a2 MOVES TO REQUESTED ANGLES AND PRINTS CARTESIAN POSITION"));
  Serial.println(F("SET RUN_STARTUP_SQUARE=true TO TRACE A 20 MM SQUARE SELF-TEST"));
  Serial.print(F("WORKSPACE RADIUS MM: "));
  Serial.print(MIN_RADIUS_MM, 2);
  Serial.print(F(" TO "));
  Serial.println(MAX_RADIUS_MM, 2);
}

void printAngleMovePosition() {
  float absX;
  float absY;
  forwardKinematics(state.joint1Estimate, state.joint2Estimate, absX, absY);
  absoluteToRelative(absX, absY, state.currentRelX, state.currentRelY);

  Serial.print(F("POS ABS_X="));
  Serial.print(absX, 2);
  Serial.print(F(" ABS_Y="));
  Serial.print(absY, 2);
  Serial.print(F(" X="));
  Serial.print(state.currentRelX, 2);
  Serial.print(F(" Y="));
  Serial.println(state.currentRelY, 2);
}

void printCurrentAngles() {
  Serial.print(F("ANGLES J1="));
  Serial.print(state.joint1Estimate, 2);
  Serial.print(F(" J2="));
  Serial.println(state.joint2Estimate, 2);
}

bool handleCalSet(char *field, char *valueToken) {
  if (field == NULL || valueToken == NULL) {
    printError(F("CAL SET REQUIRES FIELD AND VALUE"));
    return false;
  }

  int intValue;
  if (!parseIntArg(valueToken, intValue)) {
    printError(F("INVALID CAL VALUE"));
    return false;
  }

  if (strcmp(field, "J1_OFFSET") == 0) calibration.joint1Offset = clampValue(intValue, SERVO_MIN, SERVO_MAX);
  else if (strcmp(field, "J2_OFFSET") == 0) calibration.joint2Offset = clampValue(intValue, SERVO_MIN, SERVO_MAX);
  else if (strcmp(field, "J1_INVERT") == 0) calibration.joint1Invert = (intValue != 0);
  else if (strcmp(field, "J2_INVERT") == 0) calibration.joint2Invert = (intValue != 0);
  else if (strcmp(field, "PEN_UP") == 0) calibration.penUp = clampValue(intValue, SERVO_MIN, SERVO_MAX);
  else if (strcmp(field, "PEN_DOWN") == 0) calibration.penDown = clampValue(intValue, SERVO_MIN, SERVO_MAX);
  else {
    printError(F("UNKNOWN CAL FIELD"));
    return false;
  }

  state.penAngle = state.penIsDown ? calibration.penDown : calibration.penUp;
  state.penAngle = clampValue(state.penAngle, SERVO_MIN, SERVO_MAX);
  writeCurrentServoState();
  printOk(F("CAL UPDATED"));
  return true;
}

void handleCommand(char *line) {
  trimLine(line);
  if (line[0] == '\0') return;

  uppercaseLine(line);
  char *cursor = line;

  while (true) {
    char *command = nextToken(cursor);
    if (command == NULL) return;

    if (strcmp(command, "STATUS") == 0) {
      printStatus();
      continue;
    }

    if (strcmp(command, "HELP") == 0) {
      printHelp();
      continue;
    }

    if (strcmp(command, "HOME") == 0) {
      state.previousRelX = state.currentRelX;
      state.previousRelY = state.currentRelY;
      state.currentRelX = 0.0f;
      state.currentRelY = 0.0f;
      state.joint1Estimate = START_JOINT1;
      state.joint2Estimate = START_JOINT2;
      writeCurrentServoState();
      printOk(F("HOME"));
      continue;
    }

    if (strcmp(command, "G") == 0) {
      char *arg1 = nextToken(cursor);
      char *arg2 = nextToken(cursor);
      float targetRelX;
      float targetRelY;
      if (!parseFloatArg(arg1, targetRelX) || !parseFloatArg(arg2, targetRelY)) {
        printError(F("G REQUIRES X Y"));
        return;
      }

      if (!executeLinearG(targetRelX, targetRelY)) return;
      printCurrentAngles();
      printOk(F("G COMPLETE"));
      continue;
    }

    if (strcmp(command, "C") == 0) {
      char *arg1 = nextToken(cursor);
      char *arg2 = nextToken(cursor);
      char *arg3 = nextToken(cursor);
      char *arg4 = nextToken(cursor);
      float centerX;
      float centerY;
      float radius;
      int segments = DEFAULT_CIRCLE_SEGMENTS;

      if (!parseFloatArg(arg1, centerX) || !parseFloatArg(arg2, centerY) || !parseFloatArg(arg3, radius)) {
        printError(F("C REQUIRES CENTERX CENTERY RADIUS [SEGMENTS]"));
        return;
      }

      if (arg4 != NULL && !parseIntArg(arg4, segments)) {
        printError(F("C SEGMENTS MUST BE INTEGER"));
        return;
      }

      if (!drawCircleRelative(centerX, centerY, radius, segments)) return;
      printOk(F("CIRCLE COMPLETE"));
      continue;
    }

    if (strcmp(command, "A") == 0) {
      char *arg1 = nextToken(cursor);
      char *arg2 = nextToken(cursor);
      float angle1;
      float angle2;
      if (!parseFloatArg(arg1, angle1) || !parseFloatArg(arg2, angle2)) {
        printError(F("A REQUIRES ANGLE1 ANGLE2"));
        return;
      }

      state.previousRelX = state.currentRelX;
      state.previousRelY = state.currentRelY;
      state.joint1Estimate = clampFloat(angle1, JOINT_MIN_DEG, JOINT_MAX_DEG);
      state.joint2Estimate = clampFloat(angle2, JOINT_MIN_DEG, JOINT_MAX_DEG);
      writeCurrentServoState();
      printAngleMovePosition();
      printOk(F("ANGLE MOVE COMPLETE"));
      continue;
    }

    if (strcmp(command, "PEN") == 0) {
      char *arg1 = nextToken(cursor);
      if (arg1 == NULL) {
        printError(F("PEN REQUIRES UP OR DOWN"));
        return;
      }

      if (strcmp(arg1, "UP") == 0) {
        setPenState(false);
        printOk(F("PEN UP"));
        continue;
      }

      if (strcmp(arg1, "DOWN") == 0) {
        setPenState(true);
        printOk(F("PEN DOWN"));
        continue;
      }

      printError(F("UNKNOWN PEN COMMAND"));
      return;
    }

    if (strcmp(command, "Z") == 0) {
      char *arg1 = nextToken(cursor);
      int angle;
      if (!parseIntArg(arg1, angle)) {
        printError(F("Z REQUIRES ANGLE"));
        return;
      }
      setRawPenAngle(angle);
      printOk(F("Z SET"));
      continue;
    }

    if (strcmp(command, "CAL") == 0) {
      char *arg1 = nextToken(cursor);
      char *arg2 = nextToken(cursor);
      char *arg3 = nextToken(cursor);

      if (arg1 == NULL) {
        printError(F("CAL REQUIRES GET OR SET"));
        return;
      }

      if (strcmp(arg1, "GET") == 0) {
        printCalibration();
        continue;
      }

      if (strcmp(arg1, "SET") == 0) {
        handleCalSet(arg2, arg3);
        continue;
      }

      printError(F("UNKNOWN CAL COMMAND"));
      return;
    }

    printError(F("UNKNOWN COMMAND"));
    return;
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());

    if (c == '\r') continue;

    if (c == '\n') {
      inputBuffer[inputLength] = '\0';
      handleCommand(inputBuffer);
      inputLength = 0;
      continue;
    }

    if (inputLength < INPUT_BUFFER_SIZE - 1) {
      inputBuffer[inputLength++] = c;
    } else {
      inputLength = 0;
      printError(F("INPUT TOO LONG"));
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  joint1Servo.attach(JOINT1_PIN);
  joint2Servo.attach(JOINT2_PIN);
  liftServo.attach(LIFT_PIN);

  forwardKinematics(START_JOINT1, START_JOINT2, state.originAbsX, state.originAbsY);
  state.currentRelX = 0.0f;
  state.currentRelY = 0.0f;
  state.previousRelX = 0.0f;
  state.previousRelY = 0.0f;
  state.joint1Estimate = START_JOINT1;
  state.joint2Estimate = START_JOINT2;
  state.penIsDown = false;
  state.penAngle = calibration.penUp;

  writeCurrentServoState();
  delay(300);

  printHelp();
  printCalibration();
  printStatus();

  if (RUN_STARTUP_SQUARE) {
    if (drawStartupSquare()) {
      printOk(F("STARTUP SQUARE COMPLETE"));
    } else {
      printError(F("STARTUP SQUARE FAILED"));
    }
  }
}

void loop() {
  readSerialCommands();
}
