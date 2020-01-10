#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Leonardo La Rocca
"""

#First you need to import the module
import melopero_lsm9ds1 as mp
import RPi.GPIO as GPIO
import time

#Then you create and initialize the sensor object
sensor = mp.LSM9DS1()

#Gyro and Mag chip select pins
#Note: the module uses GPIO.BOARD pin numbers
gyro_cs = 24 #Chip enable 0 pin on raspberry pi
mag_cs = 26 #Chip enable 1 pin on raspberry pi  

#Setup the device to use SPI communication protocol
sensor.use_spi(gyro_cs, mag_cs)

sensor.reset_settings() 

#next you can set the output data rate you like
#note that a higher ODR will increase the responsiveness of the interrupt handler
sensor.set_mag_range(mp.LSM9DS1.MAGNETIC_4G_RANGE)
sensor.set_mag_odr(mp.LSM9DS1.MAG_ODR_80Hz)

#Set the sensor to launch an hardware interrupt if a measurment exceeds 1G on 
#the x axis.Software interrupts are also supported! You can check the interrupt
#status of the accelerometer by calling : sensor.get_mag_interrupt() this will 
#return a dictionary with information about the magnetomter interrupt status.
sensor.set_mag_interrupt(threshold = 1, x_detect = True, latch_interrupt = True,
                         interrupt_active_high = True, hardware_interrupt = True)

#Set the interrupt pin
#This pin has to be connected to the INT_M pin on the sensor.
int_pin = 7
GPIO.setmode(GPIO.BOARD)
GPIO.setup(int_pin, GPIO.IN)
interrupt_occurred = False

#interrupt callback 
def interrupt_callback(arg):
    global interrupt_occurred
    interrupt_occurred = True
    
#Setup the hardware interrupt to call our callback function
GPIO.add_event_detect(int_pin, GPIO.RISING, callback = interrupt_callback)

#now you can read the measurements!
#Print out 300 measurments at a .25s interval
for _ in range(100):
    #Check if interrupt has occurred
    if interrupt_occurred:
        #Reset interrupt flag
        interrupt_occurred = False
        #Do Stuff
        print("*"*10, "interrupt occurred", "*" * 10)
    
    #print the magnetometer interrupt status
    interrupt_status_dictionary = sensor.get_mag_interrupt()
    for key, value in interrupt_status_dictionary.items():
        print("{} : {}".format(key, value))
    
    #read the data from the sensor
    mag_measurements = sensor.get_mag()
    print("[Magnetometer]: {:.2f} G {:.2f} G {:.2f} G".format(*mag_measurements))

    time.sleep(.25)
    
#Close the device correctly!
sensor.close()