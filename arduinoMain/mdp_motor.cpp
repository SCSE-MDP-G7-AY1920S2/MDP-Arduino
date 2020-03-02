#include "mdp_motor.h"

#include <Arduino.h>
#include <DualVNH5019MotorShield.h>
#include <EnableInterrupt.h>
#include <FastPID.h>

#define LEFT_PULSE 11
#define RIGHT_PULSE 3

namespace {
// Speed config.
constexpr int kMoveFastSpeed = 380;
constexpr int kMoveSlowSpeed = 320;
constexpr int kMoveTickSpeed = 100;
constexpr int kTurnFastSpeed = 300;
constexpr int kTurnNormalSpeed = 280;
constexpr int kTurnSlowSpeed = 100;

// Ticks.
const int kTicksFast[15] = {310,  605,  905,  1207, 1494, 1791, 2086, 2380,
                            2688, 2978, 3275, 3575, 3872, 4173, 4472};
constexpr int kMoveTicks10 = 305;

constexpr int kTurnTicksL90 = 382;
constexpr int kTurnTicksL45 = 180;
constexpr int kTurnTicksL10 = 28;
constexpr int kTurnTicksL1 = 2;

constexpr int kTurnTicksR90 = 381;
constexpr int kTurnTicksR45 = 186;
constexpr int kTurnTicksR10 = 28;
constexpr int kTurnTicksR1 = 2;

// Motor Driver shield.
DualVNH5019MotorShield md;

// PID setup.
const unsigned SampleTime = 5;
unsigned long lastTime = millis();
bool shouldResetPID = false;
FastPID ShortTurnPID(/*kp=*/17.2, /*ki=*/8.2, /*kd=*/0,
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
  if (shouldResetPID) {
    pid.clear();
    shouldResetPID = false;
  }
  _setTicks();
  startMotor();
  int currentSpeed = baseSpeed;
  int last_tick_R = 0;
  double startRate = 0;

  while (rightTick <= totalTicks || leftTick <= totalTicks) {
    unsigned long now = millis();
    if (totalTicks - rightTick < 150) currentSpeed = 100;

    // Start slow and accelerate.
    if (startRate < 1 && ((rightTick - last_tick_R) >= 10 || rightTick == 0 ||
                          rightTick == last_tick_R)) {
      last_tick_R = rightTick;
      startRate += 0.03;
    }
    if (now - lastTime >= SampleTime) {
      // rightTick as setpoint, leftTick as feedback.
      int tickOffset = pid.step(rightTick, leftTick);
      if (startRate >= 1) {
        md.setSpeeds(currentSpeed + tickOffset, currentSpeed - tickOffset);
      } else {
        md.setM1Speed(startRate * (currentSpeed + tickOffset));
        md.setM2Speed(startRate * (currentSpeed - tickOffset));
      }
      lastTime = now;
    }
  }
  endMotor();
}

void _goForwardTicks(int totalTicks, int baseSpeed, FastPID& pid,
                     bool reverse = false) {
  if (shouldResetPID) {
    pid.clear();
    shouldResetPID = false;
  }
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
  pid.clear();
  shouldResetPID = true;
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

void goForward() {
  _goForwardRamp(kMoveTicks10, kMoveSlowSpeed, ShortTurnPID);
}

void goForwardFast(int cm) {
  int totalTicks = _cmToTicks(kTicksFast, cm);
  _goForwardRamp(totalTicks, kMoveFastSpeed, ShortTurnPID);
}

void goForwardTicks(int ticks) {
  _goForwardTicks(ticks, kMoveTickSpeed, ShortTurnPID);
}

void goBackward() {
  _goForwardTicks(kMoveTicks10, kMoveSlowSpeed, ShortTurnPID, /*reverse=*/true);
}

void goBackwardFast(int cm) {
  int totalTicks = _cmToTicks(kTicksFast, cm);
  _goForwardTicks(totalTicks, kMoveFastSpeed, ShortTurnPID, /*reverse=*/true);
}

void goBackwardTicks(int ticks) {
  _goForwardTicks(ticks, kMoveTickSpeed, ShortTurnPID, /*reverse=*/true);
}

void turnLeft(int angle) {
  if (angle >= 90 && angle % 90 == 0)
    _turnLeftTicks(angle, /*stepSize=*/90, kTurnTicksL90, kTurnFastSpeed,
                   ShortTurnPID);
  else if (angle % 45 == 0)
    _turnLeftTicks(angle, /*stepSize=*/45, kTurnTicksL45, kTurnNormalSpeed,
                   ShortTurnPID);
  else if (angle % 10 == 0)
    _turnLeftTicks(angle, /*stepSize=*/10, kTurnTicksL10, kTurnNormalSpeed,
                   ShortTurnPID);
  else
    _turnLeftTicks(angle, /*stepSize=*/1, kTurnTicksL1, kTurnSlowSpeed,
                   ShortTurnPID);
}

void turnRight(int angle) {
  if (angle >= 90 && angle % 90 == 0)
    _turnLeftTicks(angle, /*stepSize=*/90, kTurnTicksR90, kTurnFastSpeed,
                   ShortTurnPID, /*reverse=*/true);
  else if (angle % 45 == 0)
    _turnLeftTicks(angle, /*stepSize=*/45, kTurnTicksR45, kTurnNormalSpeed,
                   ShortTurnPID, /*reverse=*/true);
  else if (angle % 10 == 0)
    _turnLeftTicks(angle, /*stepSize=*/10, kTurnTicksR10, kTurnNormalSpeed,
                   ShortTurnPID, /*reverse=*/true);
  else
    _turnLeftTicks(angle, /*stepSize=*/1, kTurnTicksR1, kTurnSlowSpeed,
                   ShortTurnPID, /*reverse=*/true);
}

// Ramp down version of turnLeft. Used for checklist.
void turnLeftRamp(int angle) { _turnRamp(angle, turnLeft); }

// Ramp down version of turnRight. Used for checklist.
void turnRightRamp(int angle) { _turnRamp(angle, turnRight); }

void setLeftSpeed(int speed) { md.setM1Speed(speed); }

void setRightSpeed(int speed) { md.setM2Speed(speed); }

void setupPID() { 
  ShortTurnPID.setOutputRange(-400, 400);
}

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
