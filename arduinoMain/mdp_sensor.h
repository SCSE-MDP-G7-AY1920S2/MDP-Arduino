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

// Maximum distance available.
#define SR_MAX 30
#define LR_MAX 60

int getFrontLeft();
int getFrontLeftRaw();
int getFrontMiddle();
int getFrontMiddleRaw();
int getFrontRight();
int getFrontRightRaw();
int getLeft();
int getRightBack();
int getRightBackRaw();
int getRightFront();
int getRightFrontRaw();
void calibrateSensors();
void setupSensorsCalibration();

#endif  // MDP_SENSOR_H_
