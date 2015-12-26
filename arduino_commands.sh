#! /usr/bin/env bash

# push rx.ino to Arduino Mega @ ttyUSB0
arduino -v --upload --board arduino:avr:mega:cpu=atmega1280 --port /dev/ttyUSB0 rx/rx.ino

# push rx.ino to Arduino Uno @ ttyACM0
arduino -v --upload --board arduino:avr:uno --port /dev/ttyACM0 rx/rx.ino

# push tx.ino to Arduino Uno @ ttyACM1
arduino -v --upload --board arduino:avr:uno --port /dev/ttyACM1 tx/tx.ino
