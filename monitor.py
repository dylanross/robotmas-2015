#! /usr/bin/env python2

import serial
import time
import numpy as np

#port = "/dev/ttyACM0"              # connect to "base station" arduino to send RF commands
port = "/dev/ttyUSB0"               # connect directly to the robot

baud = 9600                         # set data transfer rate (bits per sec)


s = serial.Serial(port=port, baudrate=baud)

send_receive_delay = 0.1
message_delay = 0.1     # delay between start of consecutive sends
delay = message_delay - send_receive_delay

# TODO refactor USC mapping to arduino (rx.ino)
N_servos = 9
USC_addrs = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 27, 28, 29, 30, 31, 32], dtype=np.int)
Ps = np.array([900, 1500, 2100], dtype=np.int)


def send_and_receive(msg) :
    # TODO add error checking; return message for evaluation rather than
    # printing directly; could add verbose mode
    print "TX:" + msg

    s.write(msg.strip() + "\n\r")
    time.sleep(send_receive_delay)

    char = ""
    msg = ""
    while s.inWaiting() > 0 and char != "\r" :
        char = s.read()
        msg += char

    if len(msg.strip("\n\r")) > 0 :
        print "RX:" + msg.strip("\n\r")
    else :
        print "RX:WARNING -- NO RESPONSE!"


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
#    send_and_receive("0,A")
#    time.sleep(delay)
#    send_and_receive("0,R")
#    time.sleep(delay)
    send_and_receive("0,U")
    time.sleep(delay)
#    send_and_receive("0,P")
#    time.sleep(delay)
#    wave_all()
