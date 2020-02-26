#ifndef MDP_MOTOR_H_
#define MDP_MOTOR_H_

// Encoder Pin declarations.
const int LEFT_PULSE = 11;
const int RIGHT_PULSE = 3;

// Speed config.
const int MOVE_FAST_SPEED = 380;
const int MOVE_SLOW_SPEED = 300;  // 310
const int MOVE_TICK_SPEED = 100;
const int TURN_FAST_SPEED = 300;
const int TURN_NORMAL_SPEED = 280;
const int TURN_SLOW_SPEED = 100;

// Ticks lookup table.
const int TICKS[15] = {305,  596,  891,  1191, 1487, 1790, 2090, 2390,
                       2705, 2980, 3275, 3590, 3855, 4130, 4430};
const int TICKS_FAST[15] = {299,  600,  893,  1188, 1484, 1789, 2085, 2387,
                            2688, 2980, 3275, 3575, 3870, 4173, 4480};

const int TURN_TICKS_L_90 = 391;
const int TURN_TICKS_L_45 = 180;
const int TURN_TICKS_L_10 = 28;
const int TURN_TICKS_L_1 = 4;

const int TURN_TICKS_R_90 = 395;
const int TURN_TICKS_R_45 = 186;
const int TURN_TICKS_R_10 = 28;
const int TURN_TICKS_R_1 = 4;

void goForward(int cm);
void goForwardFast(int cm);
void goForwardTicks(int ticks);
void goBackward(int cm);
void goBackwardFast(int cm);
void goBackwardTicks(int ticks);
void turnLeft(int angle);
void turnLeftRamp(int angle);
void turnRight(int angle);
void turnRightRamp(int angle);
void setLeftSpeed(int speed);
void setRightSpeed(int speed);
void setupPID();
void startEncoder();
void stopEncoder();
void startMotor();
void endMotor();

#endif  // MDP_MOTOR_H_
