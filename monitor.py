#! /usr/bin/env python2

import serial

s = serial.Serial(port="/dev/ttyACM0")

while True :
    print s.readline().strip()
