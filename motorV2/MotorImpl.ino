DualVNH5019MotorShield md;

// PID setup.
const float kp = 12.3, ki = 5.4, kd = 0.01;
const float Hz = 200;
const unsigned SampleTime = 5;
unsigned long lastTime = millis();
FastPID ShortTurnPID(kp, ki, kd, Hz, /*bits=*/16, /*sign=*/true);

// Encoder Pin declarations.
const int LEFT_PULSE = 11;
const int RIGHT_PULSE = 3;

// Speed config.
const int MOVE_FAST_SPEED = 380;
const int MOVE_SLOW_SPEED = 300;  // 310
const int TURN_SLOW_SPEED = 100;
const int TURN_NORMAL_SPEED = 280;
const int TURN_FAST_SPEED = 300;

// Interrupt driven tick counts.
volatile int rightTick = 0;
volatile int leftTick = 0;

// Ticks lookup table.
const int TICKS[15] = {273,  609,  904,  1183, 1480, 1790, 2090, 2390,
                       2705, 2980, 3275, 3590, 3855, 4130, 4430};  // 282
const int TICKS_FAST[15] = {282,  599,  890,  1163, 1455, 1790, 2080, 2390,
                            2715, 2970, 3265, 3580, 3845, 4160, 4420};  // 280

const int TURN_TICKS_L_90 = 380;
const int TURN_TICKS_L_10 = 43;
const int TURN_TICKS_L_45 = 181;
const int TURN_TICKS_L_1 = 4;

const int TURN_TICKS_R_90 = 383;
const int TURN_TICKS_R_10 = 43;
const int TURN_TICKS_R_45 = 181;
const int TURN_TICKS_R_1 = 4;

void goForward(int cm) {
  int totalTicks = cmToTicks(TICKS, cm);
  goForwardTicks(totalTicks, MOVE_SLOW_SPEED, ShortTurnPID);
}

void goForwardFast(int cm) {
  int totalTicks = cmToTicks(TICKS_FAST, cm);
  goForwardTicks(totalTicks, MOVE_FAST_SPEED, ShortTurnPID);
}

void goBackward(int cm) {
  int totalTicks = cmToTicks(TICKS, cm);
  goBackwardTicks(totalTicks, MOVE_SLOW_SPEED, ShortTurnPID);
}

void goBackwardFast(int cm) {
  int totalTicks = cmToTicks(TICKS_FAST, cm);
  goBackwardTicks(totalTicks, MOVE_FAST_SPEED, ShortTurnPID);
}

void turnLeft(int angle) {
  if (angle >= 90 && angle % 90 == 0)
    turnLeftTicks(angle, /*stepSize=*/90, TURN_TICKS_L_90, TURN_FAST_SPEED,
                  ShortTurnPID);
  else if (angle % 45 == 0)
    turnLeftTicks(angle, /*stepSize=*/45, TURN_TICKS_L_45, TURN_NORMAL_SPEED,
                  ShortTurnPID);
  else if (angle % 10 == 0)
    turnLeftTicks(angle, /*stepSize=*/10, TURN_TICKS_L_10, TURN_NORMAL_SPEED,
                  ShortTurnPID);
  else
    turnLeftTicks(angle, /*stepSize=*/1, TURN_TICKS_L_1, TURN_SLOW_SPEED,
                  ShortTurnPID);
}

void turnRight(int angle) {
  if (angle >= 90 && angle % 90 == 0)
    turnRightTicks(angle, /*stepSize=*/90, TURN_TICKS_R_90, TURN_FAST_SPEED,
                   ShortTurnPID);
  else if (angle % 45 == 0)
    turnRightTicks(angle, /*stepSize=*/45, TURN_TICKS_R_45, TURN_NORMAL_SPEED,
                   ShortTurnPID);
  else if (angle % 10 == 0)
    turnRightTicks(angle, /*stepSize=*/10, TURN_TICKS_R_10, TURN_NORMAL_SPEED,
                   ShortTurnPID);
  else
    turnRightTicks(angle, /*stepSize=*/1, TURN_TICKS_R_1, TURN_SLOW_SPEED,
                   ShortTurnPID);
}

// Ramp down version of turnLeft. Used for checklist.
void turnLeftRamp(int angle) { turnRamp(angle, turnLeft); }

// Ramp down version of turnRight. Used for checklist.
void turnRightRamp(int angle) { turnRamp(angle, turnRight); }

void turnRamp(int angle, void (*turnFunc)(int)) {
  int q = 0;
  while (angle > 0) {
    if ((angle / 90) > 0) {
      q = angle / 90;
      turnFunc(90 * q);
      angle = angle % 90;
    } else if ((angle / 45) > 0) {
      q = angle / 45;
      turnFunc(45 * q);
      angle = angle % 45;
    } else if ((angle / 10) > 0) {
      q = angle / 10;
      turnFunc(10 * q);
      angle = angle % 10;
    } else {
      turnFunc(angle);
    }
  }
}

// Move forward for the number of totalTicks.
void goForwardTicks(const int totalTicks, const int baseSpeed, FastPID& pid) {
  setTicks();
  startMotor();
  int currentSpeed = baseSpeed;
  int last_tick_R = 0;
  double offset = 0;

  while (rightTick <= totalTicks || leftTick <= totalTicks) {
    unsigned long now = millis();
    if (totalTicks - rightTick < 150) currentSpeed = 100;

    if ((rightTick - last_tick_R) >= 10 || rightTick == 0 ||
        rightTick == last_tick_R) {
      last_tick_R = rightTick;
      offset += 0.1;
    }

    if (now - lastTime >= SampleTime) {
      // rightTick as setpoint, leftTick as feedback.
      int tickOffset = pid.step(rightTick, leftTick);
      if (offset >= 1) {
        md.setSpeeds(currentSpeed + tickOffset, currentSpeed - tickOffset);
      } else {
        md.setSpeeds(offset * (currentSpeed - tickOffset),
                     offset * (currentSpeed + tickOffset));
      }
      lastTime = now;
    }
  }
  endMotor();
}

// Move backward for the number of totalTicks.
void goBackwardTicks(const int totalTicks, const int baseSpeed, FastPID& pid) {
  setTicks();
  startMotor();
  int currentSpeed = baseSpeed;

  while (rightTick <= totalTicks || leftTick <= totalTicks) {
    unsigned long now = millis();
    if (now - lastTime >= SampleTime) {
      // rightTick as setpoint, leftTick as feedback.
      int tickOffset = pid.step(rightTick, leftTick);
      md.setSpeeds(-(currentSpeed + tickOffset), -(currentSpeed - tickOffset));
      lastTime = now;
    }
  }
  endMotor();
}

// Turn left for totalAngle in a step-by-step manner.
// stepSize is angles to turn for each step, and turnTicks is
// the number of ticks for the specified stepSize.
void turnLeftTicks(const int totalAngle, const int stepSize,
                   const int turnTicks, const int currentSpeed, FastPID& pid) {
  for (int i = 0; i < totalAngle; i += stepSize) {
    setTicks();
    startMotor();
    while (rightTick <= turnTicks || leftTick <= turnTicks) {
      unsigned long now = millis();
      if (now - lastTime >= SampleTime) {
        // rightTick as setpoint, leftTick as feedback.
        int tickOffset = pid.step(rightTick, leftTick);
        md.setSpeeds(-(currentSpeed + tickOffset), currentSpeed - tickOffset);
        lastTime = now;
      }
    }
  }
  endMotor();
}

void turnRightTicks(const int totalAngle, const int stepSize,
                    const int turnTicks, const int currentSpeed, FastPID& pid) {
  for (int i = 0; i < totalAngle; i += stepSize) {
    setTicks();
    startMotor();
    while (rightTick <= turnTicks || leftTick <= turnTicks) {
      unsigned long now = millis();
      if (now - lastTime >= SampleTime) {
        // rightTick as setpoint, leftTick as feedback.
        int tickOffset = pid.step(rightTick, leftTick);
        md.setSpeeds((currentSpeed + tickOffset), -(currentSpeed - tickOffset));
        lastTime = now;
      }
    }
  }
  endMotor();
}

int cmToTicks(const int* ticks, int cm) {
  int dist = (cm / 10) - 1;
  if (dist < 15) return ticks[dist];
  return 0;
}

void setupPID() {
  ShortTurnPID.setOutputRange(-400, 400);
  //  LongPID.setOutputRange(-400, 400);
}

void startEncoder() {
  md.init();
  pinMode(LEFT_PULSE, INPUT);
  pinMode(RIGHT_PULSE, INPUT);
  enableInterrupt(LEFT_PULSE, leftMotorTicks, RISING);
  enableInterrupt(RIGHT_PULSE, rightMotorTicks, RISING);
}

void stopEncoder() {
  disableInterrupt(LEFT_PULSE);
  disableInterrupt(RIGHT_PULSE);
}

void leftMotorTicks() { leftTick++; }

void rightMotorTicks() { rightTick++; }

void setTicks() {
  ShortTurnPID.clear();
  rightTick = 0;
  leftTick = 0;
}

void startMotor() {
  md.setSpeeds(0, 0);
  md.setBrakes(0, 0);
}

void endMotor() {
  md.setSpeeds(0, 0);
  md.setBrakes(400, 400);
  delay(50);
}
