#!/bin/bash
# Sync workspace.
arduino-cli compile --fqbn arduino:avr:uno arduinoMain
rsync -avz --del arduinoMain rpi:Arduino/
# Compile and upload the sketch.
ssh rpi "source .profile && cd Arduino && arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno arduinoMain"

