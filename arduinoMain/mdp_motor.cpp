#include "mdp_motor.h"

#include <Arduino.h>
#include <DualVNH5019MotorShield.h>
#include <EnableInterrupt.h>
#include <FastPID.h>

namespace {
// Motor Driver shield.
DualVNH5019MotorShield md;

// PID setup.
const unsigned SampleTime = 5;
unsigned long lastTime = millis();
int moveCount = 0;
FastPID ShortTurnPID(/*kp=*/12.3, /*ki=*/6.1, /*kd=*/0.0036,
                     /*hz=*/200, /*bits=*/16, /*sign=*/true);

// Interrupt driven tick counts.
volatile int rightTick = 0;
volatile int leftTick = 0;

void _incLeftMotorTicks() { leftTick++; }

void _incRightMotorTicks() { rightTick++; }

void _setTicks() {
  rightTick = 0;
  leftTick = 0;
}

// Move forward for the number of totalTicks. Start slow and gradually increase
// speed until baseSpeed, and reduce speed when approaching totalTicks.
void _goForwardRamp(int totalTicks, int baseSpeed, FastPID& pid) {
  if (moveCount >= MAX_MOVE_PID_RESET) {
    moveCount = 0;
    pid.clear();
  }
  _setTicks();
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
        md.setSpeeds(offset * (currentSpeed + tickOffset),
                     offset * (currentSpeed - tickOffset));
      }
      lastTime = now;
    }
  }
  endMotor();
  moveCount++;
}

void _goForwardTicks(int totalTicks, int baseSpeed, FastPID& pid,
                     bool reverse = false) {
  _setTicks();
  startMotor();
  int currentSpeed = baseSpeed;

  while (rightTick <= totalTicks || leftTick <= totalTicks) {
    unsigned long now = millis();
    if (now - lastTime >= SampleTime) {
      // rightTick as setpoint, leftTick as feedback.
      int tickOffset = pid.step(rightTick, leftTick);
      int leftSpeed =
          reverse ? -(currentSpeed + tickOffset) : (currentSpeed + tickOffset);
      int rightSpeed =
          reverse ? -(currentSpeed - tickOffset) : (currentSpeed - tickOffset);
      md.setSpeeds(leftSpeed, rightSpeed);
      lastTime = now;
    }
  }
  endMotor();
}

// Turn left for totalAngle in a step-by-step manner.
// stepSize is angles to turn for each step, and turnTicks is
// the number of ticks for the specified stepSize.
void _turnLeftTicks(int totalAngle, int stepSize, int turnTicks,
                    int currentSpeed, FastPID& pid, bool reverse = false) {
  for (int i = 0; i < totalAngle; i += stepSize) {
    _setTicks();
    startMotor();
    while (rightTick <= turnTicks || leftTick <= turnTicks) {
      unsigned long now = millis();
      if (now - lastTime >= SampleTime) {
        // rightTick as setpoint, leftTick as feedback.
        int tickOffset = pid.step(rightTick, leftTick);
        int leftSpeed = reverse ? (currentSpeed + tickOffset)
                                : -(currentSpeed + tickOffset);
        int rightSpeed = reverse ? -(currentSpeed - tickOffset)
                                 : (currentSpeed - tickOffset);
        md.setSpeeds(leftSpeed, rightSpeed);
        lastTime = now;
      }
    }
    delay(5);
  }
  endMotor();
}

int _cmToTicks(const int* ticks, int cm) {
  int dist = (cm / 10) - 1;
  if (dist < 15) return ticks[dist];
  return 0;
}

void _turnRamp(int angle, void (*turnFunc)(int)) {
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
      break;
    }
    delay(100);
  }
}
}  // namespace

void goForward(int cm) {
  int totalTicks = _cmToTicks(TICKS, cm);
  _goForwardRamp(totalTicks, MOVE_SLOW_SPEED, ShortTurnPID);
}

void goForwardFast(int cm) {
  int totalTicks = _cmToTicks(TICKS_FAST, cm);
  _goForwardRamp(totalTicks, MOVE_FAST_SPEED, ShortTurnPID);
}

void goForwardTicks(int ticks) {
  _goForwardTicks(ticks, MOVE_TICK_SPEED, ShortTurnPID);
}

void goBackward(int cm) {
  int totalTicks = _cmToTicks(TICKS, cm);
  _goForwardTicks(totalTicks, MOVE_SLOW_SPEED, ShortTurnPID, /*reverse=*/true);
}

void goBackwardFast(int cm) {
  int totalTicks = _cmToTicks(TICKS_FAST, cm);
  _goForwardTicks(totalTicks, MOVE_FAST_SPEED, ShortTurnPID, /*reverse=*/true);
}

void goBackwardTicks(int ticks) {
  _goForwardTicks(ticks, MOVE_TICK_SPEED, ShortTurnPID, /*reverse=*/true);
}

void turnLeft(int angle) {
  if (angle >= 90 && angle % 90 == 0)
    _turnLeftTicks(angle, /*stepSize=*/90, TURN_TICKS_L_90, TURN_FAST_SPEED,
                   ShortTurnPID);
  else if (angle % 45 == 0)
    _turnLeftTicks(angle, /*stepSize=*/45, TURN_TICKS_L_45, TURN_NORMAL_SPEED,
                   ShortTurnPID);
  else if (angle % 10 == 0)
    _turnLeftTicks(angle, /*stepSize=*/10, TURN_TICKS_L_10, TURN_NORMAL_SPEED,
                   ShortTurnPID);
  else
    _turnLeftTicks(angle, /*stepSize=*/1, TURN_TICKS_L_1, TURN_SLOW_SPEED,
                   ShortTurnPID);
}

void turnRight(int angle) {
  if (angle >= 90 && angle % 90 == 0)
    _turnLeftTicks(angle, /*stepSize=*/90, TURN_TICKS_R_90, TURN_FAST_SPEED,
                   ShortTurnPID, /*reverse=*/true);
  else if (angle % 45 == 0)
    _turnLeftTicks(angle, /*stepSize=*/45, TURN_TICKS_R_45, TURN_NORMAL_SPEED,
                   ShortTurnPID, /*reverse=*/true);
  else if (angle % 10 == 0)
    _turnLeftTicks(angle, /*stepSize=*/10, TURN_TICKS_R_10, TURN_NORMAL_SPEED,
                   ShortTurnPID, /*reverse=*/true);
  else
    _turnLeftTicks(angle, /*stepSize=*/1, TURN_TICKS_R_1, TURN_SLOW_SPEED,
                   ShortTurnPID, /*reverse=*/true);
}

// Ramp down version of turnLeft. Used for checklist.
void turnLeftRamp(int angle) { _turnRamp(angle, turnLeft); }

// Ramp down version of turnRight. Used for checklist.
void turnRightRamp(int angle) { _turnRamp(angle, turnRight); }

void setLeftSpeed(int speed) { md.setM1Speed(speed); }

void setRightSpeed(int speed) { md.setM2Speed(speed); }

void setupPID() { ShortTurnPID.setOutputRange(-400, 400); }

void startEncoder() {
  md.init();
  pinMode(LEFT_PULSE, INPUT);
  pinMode(RIGHT_PULSE, INPUT);
  enableInterrupt(LEFT_PULSE, _incLeftMotorTicks, RISING);
  enableInterrupt(RIGHT_PULSE, _incRightMotorTicks, RISING);
}

void stopEncoder() {
  disableInterrupt(LEFT_PULSE);
  disableInterrupt(RIGHT_PULSE);
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
