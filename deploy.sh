#!/bin/bash

# Sync workspace.
rsync -avz --del arduinoMain rpi:Arduino

# Compile and upload the sketch.
ssh rpi "cd Arduino && make upload"

