#ifndef MDP_SENSOR_H_
#define MDP_SENSOR_H_

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
