bool testTurn(void (*turnFunc)(int)) {
  calibrateFront(/*isBlock=*/false);
  sendSensor("T");
  for (int i = 0; i < 8; i++) {
    turnFunc(90);
    delay(50);
  }
  sendSensor("T");
  int fl = getFrontLeftRaw();
  int fr = getFrontRightRaw();
  return abs(fl - fr) < 3;
}

// Assumes calibrateStart is performed and test right alignment 
// and right distance.
bool assertStartPosition() {
  int rfOffset = 5;
  int rf = getRightFrontRaw() + rfOffset;
  int rb = getRightBackRaw();
  int rightDist = (rf + rb) / 2;
  return abs(rf - rb) < 3 && 
    rightDist >= 100 && rightDist <= 120;
}

// Assumes that a block is positioned at the expected location,
// and tests whether the robot performs straight and accurate movement.
bool assertEndPosition() {
  int rfOffset = 5;
  int rf = getRightFrontRaw() + rfOffset;
  int rb = getRightBackRaw();
  int rightDist = (rf + rb) / 2;
  int fl = getFrontLeftRaw();
  int fm = getFrontMiddleRaw();
  int fr = getFrontRightRaw();
  int frontDist = (fl + fm + fr) / 3;
  return abs(rf - rb) < 3 && abs(fl - fr) < 3 &&
    rightDist >= 100 && rightDist <= 120 &&
    frontDist >= 110 && frontDist <= 130;
}

bool testGoForwardSlow() {
  calibrateStart();
  if (!assertStartPosition())
    return false;
  sendSensor("T");
  for (int i = 0; i < 8; i++) {
    goForward();
    sendSensor("T");
  }
  return assertEndPosition();
}

bool testGoForwardFast() {
  calibrateStart();
  if (!assertStartPosition())
    return false;
  Serial.println("Enter distance to move:");
  if (Serial.available() > 0) {
    int dist = Serial.parseInt();
    sendSensor("T");
    goForwardFast(dist);
    sendSensor("T");
    return assertEndPosition();
  }
  return false;
}

void runTests(int nTest) {
  static int nextTest = 0;
  if (nTest > 0)
    nextTest = nTest;
  bool testPassed = false;
  if (nextTest == 0) {
    Serial.println("Starting system tests...");
    Serial.println("See https://github.com/zhangks98/MDP-Arduino/wiki/Robot-System-Check-Routines for detailed instructions.");
    Serial.println("Press T to start Test 1");
    nextTest++;
    return;
  }

  Serial.println("Test " + nextTest);
  switch(nextTest) {
    case 1:
      testPassed = testTurn(turnLeft);
      break;
    case 2:
      testPassed = testTurn(turnRight);
      break;
    
  }
  if (testPassed) {
    nextTest++;
    Serial.println("PASSED!");
    Serial.println();
    Serial.println("Press T to start Test " + nextTest);
  } else {
    Serial.println("FAILED!");
    Serial.println();
    Serial.println("Press T to re-run Test " + nextTest);
  }
}
