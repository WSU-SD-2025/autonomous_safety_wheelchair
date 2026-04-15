#!/bin/bash

# Usage: ./set_fan_speed.sh <speed>
# Min speed: 0, Max speed: 255

SPEED=${1:-0}

echo "Fan speed set to $SPEED"
sudo bash -c "echo $SPEED > /sys/devices/pwm-fan/target_pwm"