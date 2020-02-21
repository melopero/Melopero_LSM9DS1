# Melopero LSM9DS1 breakout board
![melopero logo](images/sensor.jpg?raw=true)

## Getting Started
### Prerequisites
You will need:
- Python3 installed, you can download it here: [download python3](https://www.python.org/downloads/)
- the Melopero LSM9DS1 breakout: [buy here](https://www.melopero.com/en/shop/sensori/melopero-amg8833-grid-eye-ir-array-breakout/)


### Installing
You can install the melopero-lsm9ds1 module by typing this line in a terminal window: 
```python
sudo pip3 install melopero-lsm9ds1
```

### Usage
Import the melopero_lsm9ds1 and time modules in your Python3 script: 
```python
import melopero_lsm9ds1 as mp
import time
```
Create and initialize the sensor object
```python 
sensor = mp.LSM9DS1()
```
Select I2C or SPI as communication bus. 
<br>To select I2C (if no parameters are given the default i2c addresses will be used):
```python
sensor.use_i2c()
#sensor.use_spi(gyro_cs, mag_cs) to use spi

```
If you want to use SPI bus, first you must initialize gyro_cs (CSAG pin on the sensor) and mag_cs (CSM on the sensor)
<br>Note: the module uses GPIO.BOARD pin numbers:
```python
gyro_cs = 24 #Chip Enable 0 (CE_0) pin on the Raspberry Pi
mag_cs = 26 #Chip Enable 1 (CE_1) pin on the Raspberry Pi  
```
Now select SPI bus
```python
sensor.use_spi(gyro_cs, mag_cs)

```

Set the output data rate
```python 
sensor.set_gyro_odr(mp.LSM9DS1.ODR_119Hz)
sensor.set_acc_odr(mp.LSM9DS1.ODR_119Hz)
sensor.set_mag_odr(mp.LSM9DS1.MAG_ODR_20Hz)

```

Print out 100 measurments at a .5s interval
```python 
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
    
```
Close the connection after using the sensor
```python 
sensor.close()
```

## Complete example using I2C bus
The following example will Print out 100 measurments of the accelerometer, gyroscope and magnetometer at a .5s interval.
```python 
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
```

## Complete example using SPI bus
The following example will Print out 100 measurments of the accelerometer, gyroscope and magnetometer at a .5s interval.
```python 
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

#Gyro and Mag chip select pins
#Note: the module uses GPIO.BOARD pin numbers
gyro_cs = 24 #Chip enable 0 pin on raspberry pi
mag_cs = 26 #Chip enable 1 pin on raspberry pi  

#Setup the device to use SPI communication protocol
sensor.use_spi(gyro_cs, mag_cs) 

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
```

### Attention:

The module is written in python3 and by now supports only python3, remember to use always `pip3` when you install the module and `python3` when you run the code. 


