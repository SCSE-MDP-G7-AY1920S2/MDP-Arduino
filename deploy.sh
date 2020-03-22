#!/bin/bash
# Compile the sketch locally.
arduino-cli compile --fqbn arduino:avr:uno arduinoMain

# Sync workspace.
if [[ "$OSTYPE" == "msys" ]]; then
scp -r arduinoMain rpi:Arduino/
else
rsync -avz arduinoMain rpi:Arduino/
fi

# Upload the sketch to the first available serial port.
ssh rpi "source .profile && cd Arduino && arduino-cli upload -p /dev/ttyACM* --fqbn arduino:avr:uno arduinoMain"
