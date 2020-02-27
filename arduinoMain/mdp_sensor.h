#ifndef MDP_SENSOR_H_
#define MDP_SENSOR_H_

// Model Number for IR sensors.
#define SRmodel 1080
#define LRmodel 20150

// Sensor pins.
#define s0 A0  // front right
#define s1 A1  // front middle
#define s2 A2  // front left
#define s3 A3  // left long range
#define s4 A4  // right bottom
#define s5 A5  // right top

// Sensor Curve-Fitting Parameters.
#define P1 5.770230285862897e+03
#define Q1 3.057232146852901
#define SR_VMIN 145
#define LR_VMIN 170

// Maximum distance for Grid 1 and 2.
#define FRONT_G2 27
#define FRONT_G1 17

#define RIGHT_G2 26
#define RIGHT_G1 16

#define LR_MAX 70
#define LR_OFFSET 15

int getFrontLeft();
int getFrontLeftRaw();
int getFrontMiddle();
int getFrontMiddleRaw();
int getFrontRight();
int getFrontRightRaw();
int getLeft();
int getLeftRaw();
int getRightBack();
int getRightBackRaw();
int getRightFront();
int getRightFrontRaw();
void calibrateSensors();
void setupSensorsCalibration();

#endif  // MDP_SENSOR_H_
