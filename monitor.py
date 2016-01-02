#! /usr/bin/env python2

import serial
import time
import numpy as np


s = serial.Serial(port="/dev/ttyUSB0", baudrate=9600)

send_receive_delay = 0.01
message_delay = 0.02     # delay between start of consecutive sends
delay = message_delay - send_receive_delay

# TODO refactor USC mapping to arduino (rx.ino)
N_servos = 9
USC_addrs = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 27, 28, 29, 30, 31, 32], dtype=np.int)
Ps = np.array([900, 1500, 2100], dtype=np.int)


def send_and_receive(msg) :
#    while s.inWaiting() > 0 :
#        print "WARNING: unchecked response! Flushing..."
#        s.readline().strip()

    print "TX:" + msg

    s.write(msg.strip() + "\n\r")
    time.sleep(send_receive_delay)
    while s.inWaiting() > 0 :
        print "RX:" + s.readline().strip()
        while s.inWaiting() > 0 :
            s.read()


def move_servo(ID=0, P=1500, T=500, wait=True) :
    cmd = "0,S,#" 
    cmd += str(ID + 1)
    cmd += "P"
    cmd += str(P)
    cmd += "T"
    cmd += str(T)
    send_and_receive(cmd)
    if wait :
        time.sleep(T/1000.)


def home_all() :
    for ID in USC_addrs - 1 :
        move_servo(ID=ID, P=1500, T=1000, wait=True)


def wave_all() :
    for P in Ps :
        for ID in USC_addrs - 1 :
            move_servo(ID=ID, P=P, T=1000, wait=True)


while True :
    send_and_receive("0,A")
    time.sleep(delay)
    send_and_receive("0,R")
    time.sleep(delay)
#    wave_all()
