#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Leonardo La Rocca
"""

from melopero_lsm9ds1 import LSM9DS1
import RPi.GPIO as GPIO
import time

flag = True

def func(val):
    global flag
    flag = False


dev = LSM9DS1()
dev.use_spi()
dev.reset_settings()

time.sleep(0.01)

dev.set_gyro_odr(LSM9DS1.ODR_50Hz)

channel = 11

GPIO.setmode(GPIO.BOARD)

GPIO.setup(channel, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(channel, GPIO.FALLING, callback= func)

dev.set_gyro_interrupt(x_threshold = 100, y_threshold= 100, z_threshold=100, hardware_interrupt=True)

while flag:
    dps = dev.get_gyro()
    print("dps: x:{:.2f} y:{:.2f} z:{:.2f}".format(dps[0], dps[1], dps[2]))
    
    #print("mag : {}".format(mag))
    time.sleep(0.2)

print("interrupt occurred")
       
dev.close()