# Melopero LSM9DS1 Python3 library
A library for interfacing the Melopero LSM9DS1 9-DOF breakout board with the Rasberry Pi.
<br> If you were looking for the Arduino library click [HERE](https://github.com/melopero/Melopero_LSM9DS1_Arduino_Library)

# Melopero LSM9DS1 breakout board
![melopero logo](images/melopero-lsm9ds1-main.jpg?raw=true)

# Pinouts

<table style="width:100%">
  <tr>
    <th>Melopero LSM9DS1</th>
    <th>Description</th> 
  </tr>
  <tr>
    <td>3V3</td>
    <td>Input power pin. Apply 3.3V to this pin</td> 
  </tr>
  <tr>
    <td>SCL</td>
    <td>I2C or SPI Serial Clock pin</td> 
  </tr>
  <tr>
    <td>SDA</td>
    <td>I2C SDA pin or SPI MOSI pin</td> 
  </tr>
  <tr>
    <td>GND</td>
    <td>Ground pin</td>
  </tr>
  <tr>
    <td>CSAG</td>
    <td>Accelerometer+Gyro SPI Chip Select</td>
  </tr>
  <tr>
    <td>CSM</td>
    <td>Magnetometer SPI Chip Select</td>
  </tr>
  <tr>
    <td>SDOAG</td>
    <td>Accelerometer+Gyro SPI MISO pin</td>
  </tr>
  <tr>
    <td>SDOM</td>
    <td>Magnetometer SPI MISO pin</td>
  </tr>
  <tr>
    <td>INT1</td>
    <td>Accelerometer+Gyro Interrupt pin</td>
  </tr>
   <tr>
    <td>INTM</td>
    <td>Magnetometer Interrupt pin</td>
  </tr>
  <tr>
    <td>INT2</td>
    <td>Another Interrupt pin for the accelerometer+gyro. <br>This pin is not supported in our library</td>
  </tr>
  <tr>
    <td>DEN</td>
    <td>Gyroscope data enable pin. <br>This pin is not supported in our library</td>
  </tr>
  <tr>
    <td>DRDY</td>
    <td>Magnetometer data ready pin. <br>This pin is not supported in our library</td>
  </tr>
</table>

## Getting Started
### Prerequisites
You will need:
- Python3 installed, you can download it here: [download python3](https://www.python.org/downloads/)
- the Melopero LSM9DS1 breakout: [buy here](https://www.melopero.com/shop/sensori/imu/melopero-lsm9ds1-breakout/)



### Installing
You can install the melopero-lsm9ds1 module by typing this line in a terminal window: 
```python
sudo pip3 install melopero-lsm9ds1
```

### Connect the sensor to the Raspberry Pi <br>(use only 3.3V power and logic, do not connect this sensor board directly to 5V)
You can find a description of the GPIO connector of the Raspberry Pi [HERE](https://www.raspberrypi.org/documentation/usage/gpio/)
<br>This sensor communicates over I2C or SPI.
<br><b>I2C connections</b>:
<table style="width:100%">
  <tr>
    <th>Melopero LSM9DS1</th>
    <th>Raspberry Pi</th> 
  </tr>
  <tr>
    <td>3V3</td>
    <td>3.3V</td> 
  </tr>
  <tr>
    <td>SCL</td>
    <td>SCL</td> 
  </tr>
  <tr>
    <td>SDA</td>
    <td>SDA</td> 
  </tr>
  <tr>
    <td>GND</td>
    <td>GND</td> 
  </tr>
</table>
<br><b>SPI connections</b>:
<table style="width:100%">
  <tr>
    <th>Melopero LSM9DS1</th>
    <th>Raspberry Pi</th> 
  </tr>
  <tr>
    <td>3V3</td>
    <td>3.3V</td> 
  </tr>
  <tr>
    <td>SCL</td>
    <td>SCLK</td> 
  </tr>
  <tr>
    <td>SDA</td>
    <td>MOSI</td> 
  </tr>
  <tr>
    <td>SDOAG, SDOM</td>
    <td>MISO</td> 
  </tr>
  <tr>
    <td>CSAG</td>
    <td>CE_0 (BOARD pin 24, GPIO 8)</td> 
  </tr>
  <tr>
    <td>CSM</td>
    <td>CE_1 (BOARD pin 26, GPIO 7)</td> 
  </tr>
</table>
<br><b>Optional interrupt pins</b>:
<table style="width:100%">
  <tr>
    <th>Melopero LSM9DS1</th>
    <th>Raspberry Pi</th> 
  </tr>
  <tr>
    <td>INT1 <br>(Interrupt pin for Accelerometer and Gyroscope)</td>
    <td>Any pin that supports interrupts</td> 
  </tr>
  <tr>
    <td>INTM <br>(interrupt pin for Magnetometer)</td>
    <td>Any pin that supports interrupts</td> 
  </tr>
  
</table>

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

## Example using I2C bus
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

## Example using SPI bus
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


