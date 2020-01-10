#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Leonardo La Rocca
"""

#First you need to import the module
import melopero_lsm9ds1 as mp
import time

#Then you create and initialize the sensor object
sensor = mp.LSM9DS1()

#You can set the sensor to use i2c or spi
sensor.use_i2c() #if no parameters are given the default i2c addresses will be used
#sensor.use_spi(gyro_cs, mag_cs) to use spi

#next you can set the output data rate you like
sensor.set_gyro_odr(mp.LSM9DS1.ODR_119Hz)
sensor.set_acc_odr(mp.LSM9DS1.ODR_119Hz)
sensor.set_mag_odr(mp.LSM9DS1.MAG_ODR_20Hz)

#now you can read the measurements!
#Print out 100 measurments at a .5s interval

for _ in range(100):
    #read the data from the sensor
    gyro_measurements = sensor.get_gyro()
    acc_measurements = sensor.get_acc()
    mag_measurements = sensor.get_mag()

    print("*" * 20)
    print("[Gyro]: {:.2f} dps {:.2f} dps {:.2f} dps".format(*gyro_measurements))
    print("[Accelerometer]: {:.2f} g {:.2f} g {:.2f} g".format(*acc_measurements))
    print("[Magnetometer]: {:.2f} G {:.2f} G {:.2f} G".format(*mag_measurements))
    print("*" * 20)

    time.sleep(.5)
    
#Close the device correctly!
sensor.close()