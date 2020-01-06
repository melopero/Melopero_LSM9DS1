#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Leonardo La Rocca
"""

#First you need to import the module
import melopero_lsm9ds1 as mp
import RPi.GPIO as gpio

import time

#Then you create and initialize the sensor object
sensor = mp.LSM9DS1()

#You can set the sensor to use i2c or spi
sensor.use_i2c() #if no parameters are given the default i2c addresses will be used

#next you can set the output data rate you like
#note that a higher ODR will increase the responsiveness of the interrupt handler
sensor.set_acc_odr(mp.LSM9DS1.ODR_119Hz)

#Set the sensor to launch an hardware interrupt if a measurment exceeds .5g on 
#the x axis for 10 consecutive measurements. Software interrupts are also 
#supported! You can check the interrupt status of the accelerometer by calling : 
#sensor.get_acc_interrupt() this will return a dictionary with information about 
#the accelerometer interrupt status.
sensor.set_acc_interrupt(x_threshold = .5, x_detect_high = True, 
                         samples_to_recognize = 10, hardware_interrupt = True)

#Set the interrupt pin
int_pin = 2
interrupt_occurred = False

#interrupt callback 
def interrupt_callback():
    global interrupt_occurred
    interrupt_occurred = True
    
#Setup the hardware interrupt to call our callback function
gpio.add_event_detect(int_pin, gpio.RISING, callback = interrupt_callback)

#now you can read the measurements!
#Print out 300 measurments at a .25s interval
for _ in range(100):
    #Check if interrupt has occurred
    if interrupt_occurred:
        #Reset interrupt flag
        interrupt_occurred = False
        #Do Stuff
        print("*"*10, "interrupt occurred", "*" * 10)
    
    #print the accelerometer interrupt status
    interrupt_status_dictionary = sensor.get_acc_interrupt()
    for key, value in interrupt_status_dictionary.items():
        print("{} : {}".format(key, value))
    
    #read the data from the sensor
    acc_measurements = sensor.get_acc()
    print("[Accelerometer]: {:.2f} g {:.2f} g {:.2f} g".format(*acc_measurements))

    time.sleep(.25)
    
#Close the device correctly!
sensor.close()