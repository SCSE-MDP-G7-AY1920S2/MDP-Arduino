#include <FastPID.h>
#include <DualVNH5019MotorShield.h>
#include <EnableInterrupt.h>

void setup() {
  // Set up Serial connection.
  Serial.begin(115200);
  Serial.setTimeout(50);
  
  startEncoder();
  setupPID();
  
  delay(1000);
  //calibrateStart();

  goForwardTicks(3800);
}

void loop() {
  
  delay(1000);
}
