#include <DualVNH5019MotorShield.h>
#include <EnableInterrupt.h>
#include <FastPID.h>

void setup() {
  // Set up Serial connection.
  Serial.begin(115200);
  Serial.setTimeout(50);

  startEncoder();
  setupPID();

  delay(1000);
  // calibrateStart();

  turnRightRamp(180);


}

void loop() { delay(1000); }
