#ifndef MDP_MOTOR_H_
#define MDP_MOTOR_H_

void goForward();
void goForwardHalf();
void goForwardFast(int cm);
void goForwardTicks(int ticks);
void goBackward();
void goBackwardFast(int cm);
void goBackwardTicks(int ticks);
void turnLeft(int angle);
void turnLeftRamp(int angle);
void turnLeftTicks(int ticks);
void adjutstTurnLeftTicks(int delta);
void turnRight(int angle);
void turnRightRamp(int angle);
void turnRightTicks(int ticks);
void adjutstTurnRightTicks(int delta);
void setLeftSpeed(int speed);
void setRightSpeed(int speed);
void setupPID();
void startEncoder();
void stopEncoder();
void startMotor();
void endMotor();

#endif  // MDP_MOTOR_H_
