## Motor Implementation Updates
Apart from general refactoring of the code (e.g. remove unused variables, combine duplicate code), 
this file outlines the major changes to the implementation.
- Use [FastPID](https://github.com/mike-matera/FastPID) instead of the [PID_v1](https://github.com/br3ttb/Arduino-PID-Library) library.
  - Faster computation due to fixed-point arithmatic.
  - `setpoint` and `feedback` are both integers by design, no need to convert to double.
  - Sample Time is no longer managed by the library.

- `goForward` will not have a delay when `offset` is less than 1.
  - Sample time is 5ms, which is already quite small.
  - The initial change in error should be managed by the derivative (D) controller.
  - [ ] This needs further verification.

- `turnLeftFast` and `turnRightFast` are deprecated.
  - These functions sets faster motor speed only when turning 90 degrees, the rest is identical to `turnLeft` and `turnRight`.
  - It doesn't make a lot of sense to have a "faster" and "slower" version just for turning 90 degrees.
  - It does make sense to turn faster when turning for a larger degree, so the case is merged with `turnLeft` and `turnRight`.

- `turnRamp` function for completing the checklist.
  - `turnLeft` and `turnRight` only turns faster when the angle is the *exact* multiple of 90, 45 and 10, which are typically seen in the arena. 
  - `turnRamp` is suitable for any angle, it first checks if the angle is big enough for a larger turning speed, and ramps down the speed based on the remaining angle. 
