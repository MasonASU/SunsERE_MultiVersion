# -*- coding: utf-8 -*-
"""
Created on Mon Sep 27 15:48:20 2021

@author: mason
"""
import serial
import time
arduino= serial.Serial(port='COM3',baudrate=9600,timeout=0.1)
time.sleep(0.05)
arduino_reading=""
while(True):
    arduino.write(bytes("\n",'utf-8'))
    arduino_reading=arduino.readline()
    time.sleep(0.05)
    if(len(arduino_reading)>5):
        print(float(arduino_reading))
