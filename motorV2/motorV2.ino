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

  goForward(10);
}

void loop() { delay(1000); }
