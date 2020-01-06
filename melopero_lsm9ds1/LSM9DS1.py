#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Leonardo La Rocca
"""

i2c_available = True
spi_available = True

try:
    import smbus2
except ModuleNotFoundError:
    print("WARNING: You need smbus2 in order to use the i2c protocol.")
    i2c_available = False
except :
    print("WARNING: Something went wrong while importing smbus2.")
    i2c_available = False    
    
try:
    import RPi.GPIO as GPIO
    import spidev
except ModuleNotFoundError:
    print("WARNING: You need spidev and RPi.GPIO in order to use the SPI protocol.")
    spi_available = False
except :
    print("WARNING: Something went wrong while importing spidev and/or RPi.GPIO... (try to run program with superuser privileges).")
    spi_available = False


class LSM9DS1():
    
    WHO_AM_I_REG = 0x0F
    GYRO_ID = 0b01101000
    MAG_ID = 0b00111101
    
    SPI_READ_FLAG = 0b10000000
    
    CONTROL_REG_8 = 0x22
    
    GYRO_CONTROL_REG_1 = 0x10
    
    ACC_CONTROL_REG_5 = 0x1F
    ACC_CONTROL_REG_6 = 0x20
    
    MAG_CONTROL_REG_1 = 0x20
    MAG_CONTROL_REG_2 = 0x21
    MAG_CONTROL_REG_3 = 0x22

    TEMPERATURE_REG = 0x15
    
    GYRO_X_REG = 0x18
    GYRO_Y_REG = 0x1A
    GYRO_Z_REG = 0x1C
    
    ACC_X_REG = 0x28
    ACC_Y_REG = 0x2A
    ACC_Z_REG = 0x2C
    
    MAG_X_REG = 0x28
    MAG_Y_REG = 0x2A
    MAG_Z_REG = 0x2C
    
    ANGULAR_RATE_245DPS_RANGE = 8.75
    ANGULAR_RATE_500DPS_RANGE = 17.50
    ANGULAR_RATE_2000DPS_RANGE = 70
    
    LIN_ACC_2G_RANGE = 0.061
    LIN_ACC_4G_RANGE = 0.122
    LIN_ACC_8G_RANGE = 0.244
    LIN_ACC_16G_RANGE = 0.732
    
    MAGNETIC_4G_RANGE = 0.14
    MAGNETIC_8G_RANGE = 0.29
    MAGNETIC_12G_RANGE = 0.43
    MAGNETIC_16G_RANGE = 0.58
    
    ODR_POWER_DOWN = 0b000
    ODR_10Hz = 0b001
    ODR_50Hz = 0b010
    ODR_119Hz = 0b011
    ODR_238Hz = 0b100
    ODR_476Hz = 0b101
    ODR_952Hz = 0b110
    
    MAG_ODR_0_625Hz = 0b000
    MAG_ODR_1_25Hz = 0b001
    MAG_ODR_2_5Hz = 0b010
    MAG_ODR_5Hz = 0b011
    MAG_ODR_10Hz = 0b100
    MAG_ODR_20Hz = 0b101
    MAG_ODR_40Hz = 0b110
    MAG_ODR_80Hz = 0b111
    
    MAG_CONTINUOUS_MODE = 0b00
    MAG_SINGLE_MODE = 0b01
    MAG_POWER_DOWN = 0b10
    
    INT1_CTRL_REG = 0x0C
    INT2_CTRL_REG = 0x0D
    
    GYRO_INT_CFG_REG = 0x30
    GYRO_X_INT_THR_REG = 0x31
    GYRO_Y_INT_THR_REG = 0x33
    GYRO_Z_INT_THR_REG = 0x35
    GYRO_INT_DUR_REG = 0x37
    GYRO_INT_SRC_REG = 0x14
    
    ACC_INT_CFG_REG = 0x06
    ACC_X_INT_THR_REG = 0x07
    ACC_Y_INT_THR_REG = 0x08
    ACC_Z_INT_THR_REG = 0x09
    ACC_INT_DUR_REG = 0x0A
    ACC_INT_SRC_REG = 0x26
    
    MAG_INT_CFG_REG = 0x30
    MAG_INT_THR_REG = 0x32
    MAG_INT_SRC_REG = 0x31

    def __init__(self):
        self.gyro_scale = LSM9DS1.ANGULAR_RATE_245DPS_RANGE
        self.acc_scale = LSM9DS1.LIN_ACC_2G_RANGE
        self.mag_scale = LSM9DS1.MAGNETIC_4G_RANGE
        self.i2c_enabled = False
        self.spi_enabled = False        
    
    def use_i2c(self, acc_gyro_i2c_addr = 0x6b, mag_i2c_addr = 0x1e, bus = 1):
        if not i2c_available:
            raise Exception("smbus2 module is not present -> i2c protocol not available") 
            
        self.i2c_enabled = True
        self.spi_enabled = False
        
        self.i2c = smbus2.SMBus(bus)
        self.acc_gyro_i2c_address = acc_gyro_i2c_addr
        self.mag_i2c_address = mag_i2c_addr
        #enable i2c in control registers mag control 3 ...
    
    
    def use_spi(self, gyro_cs = 8, mag_cs = 7, bus = 0, max_speed_hz = 10 ** 7):
        if not spi_available:
            raise Exception("spidev and/or RPi.GPIO are not present -> SPI protocol not available")
            
        self.spi_enabled = True
        self.i2c_enabled = False
        
        self.spi = spidev.SpiDev()
        self.spi.cshigh = False
        self.spi.mode = 0b10
        self.spi_bus = bus
        
        #GPIO setup
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(gyro_cs, GPIO.OUT, initial = GPIO.HIGH)
        GPIO.setup(mag_cs, GPIO.OUT, initial = GPIO.HIGH)

        GPIO.output(gyro_cs, GPIO.LOW)
        self.spi.open(bus, 0)
        self.spi.max_speed_hz = max_speed_hz
        self.spi_max_speed_hz = max_speed_hz
        
        whoami = self.spi.xfer2([LSM9DS1.WHO_AM_I_REG | LSM9DS1.SPI_READ_FLAG, 0x00])
        print(whoami)
        if whoami[1] == LSM9DS1.GYRO_ID:
            self.spi_gyro_dev = 0
            self.spi_mag_dev = 1
            self.spi_gyro_cs = gyro_cs
            self.spi_mag_cs = mag_cs
        elif whoami[1] == LSM9DS1.MAG_ID :
            self.spi_mag_dev = 0
            self.spi_gyro_dev = 1
            self.spi_gyro_cs = mag_cs
            self.spi_mag_cs = gyro_cs
        else :
            self.spi_enabled = False
            self.spi.close()
            GPIO.output(gyro_cs, GPIO.HIGH)
            raise ValueError("""Pin numbers {}, {} are wrong or the device is not 
                             responding correctly.""".format(gyro_cs, mag_cs))
        
        self.spi.close()
        GPIO.output(gyro_cs, GPIO.HIGH)
        
        #enable spi read operations
        self.write_byte_mag(LSM9DS1.MAG_CONTROL_REG_3, 0x00)
        #TODO: disable i2c ?
    
    def read_bytes_gyro(self, register_addr, length):
        if self.i2c_enabled :
            return self.i2c.read_i2c_block_data(self.acc_gyro_i2c_address, register_addr, length)
        elif self.spi_enabled:
            GPIO.output(self.spi_gyro_cs, GPIO.LOW)
            self.spi.open(self.spi_bus, self.spi_gyro_dev)
            self.spi.max_speed_hz = self.spi_max_speed_hz
            extend = [0x00] * length
            data_to_send = [register_addr | LSM9DS1.SPI_READ_FLAG]
            data_to_send.extend(extend)
            result = self.spi.xfer2(data_to_send)            
            self.spi.close()
            GPIO.output(self.spi_gyro_cs, GPIO.HIGH)
            return result[1:]
        else :
            raise Exception("You must select a communication protocol before")
    
    def write_byte_gyro(self, register_addr, data):
        if self.i2c_enabled:
            self.i2c.write_byte_data(self.acc_gyro_i2c_address, register_addr, data)  
        elif self.spi_enabled:
            GPIO.output(self.spi_gyro_cs, GPIO.LOW)
            self.spi.open(self.spi_bus, self.spi_gyro_dev)
            self.spi.max_speed_hz = self.spi_max_speed_hz
            self.spi.xfer2([register_addr, data])
            self.spi.close()
            GPIO.output(self.spi_gyro_cs, GPIO.HIGH)
        else :
            raise Exception("You must select a communication protocol before")
            
    def write_flag_gyro(self, register_addr, flag, start, length):
        """Writes the flag bits in the spcified register at the specified position.\n
        start : start of the flag bits, the MSB has an index of 8 and the lsb has an index of 1.\n
        length : how many bits of the flag to set. \n
        Example: 
            register value = 0b11110000 \n
            flag value = 0b0011 \n
            start = 6 \n
            length = 4 \n
            result = 0b11001100
        """
        end = start - length
        reg_content = self.read_bytes_gyro(register_addr, 1)[0]
        
        msb = reg_content >> start << start
        flag_bits = flag << end
        lsb = reg_content % (2 ** end)
        self.write_byte_gyro(register_addr, msb | flag_bits | lsb)
        
            
    def read_bytes_mag(self, register_addr, length):
        if self.i2c_enabled :
            return self.i2c.read_i2c_block_data(self.mag_i2c_address, register_addr, length)
        elif self.spi_enabled:
            GPIO.output(self.spi_mag_cs, GPIO.LOW)
            self.spi.open(self.spi_bus, self.spi_mag_dev)
            self.spi.max_speed_hz = self.spi_max_speed_hz
            extend = [0x00] * length
            data_to_send = [register_addr | LSM9DS1.SPI_READ_FLAG]
            data_to_send.extend(extend)
            result = self.spi.xfer2(data_to_send)
            self.spi.close()
            GPIO.output(self.spi_mag_cs, GPIO.HIGH)
            return result[1:]
        else :
            raise Exception("You must select a communication protocol before")
    
    def write_byte_mag(self, register_addr, data):
        if self.i2c_enabled:
            self.i2c.write_byte_data(self.mag_i2c_address, register_addr, data)  
        elif self.spi_enabled:
            GPIO.output(self.spi_mag_cs, GPIO.LOW)
            self.spi.open(self.spi_bus, self.spi_mag_dev)
            self.spi.max_speed_hz = self.spi_max_speed_hz
            self.spi.xfer2([register_addr, data])
            self.spi.close()
            GPIO.output(self.spi_mag_cs, GPIO.HIGH)
        else :
            raise Exception("You must select a communication protocol before")
    
    def write_flag_mag(self, register_addr, flag, start, length):
        """Writes the flag bits in the spcified register at the specified position.\n
        start : start of the flag bits, the MSB has an index of 8 and the lsb has an index of 1.\n
        length : how many bits of the flag to set. \n
        Example: 
            register value = 0b11110000 \n
            flag value = 0b0011 \n
            start = 6 \n
            length = 4 \n
            result = 0b11001100
        """
        end = start - length
        reg_content = self.read_bytes_mag(register_addr, 1)[0]
        
        msb = reg_content >> start << start
        flag_bits = flag << end
        lsb = reg_content % (2 ** end)
        self.write_byte_mag(register_addr, msb | flag_bits | lsb)
        
    
    def close(self):
        if self.i2c_enabled:
            self.i2c.close()
        elif self.spi_enabled:
            GPIO.cleanup()
        else :
            return
        
    def reset_settings(self):
        self.write_flag_gyro(LSM9DS1.CONTROL_REG_8, 0b101, 3, 3)
        self.write_flag_mag(LSM9DS1.MAG_CONTROL_REG_2, 0b11, 4, 2)  
        self.set_gyro_interrupt()
        self.set_acc_interrupt()
        self.set_mag_interrupt()
        
    def set_gyro_odr(self, odr):
        """Sets the accelerometer output-data-rate."""
        self.write_flag_gyro(LSM9DS1.GYRO_CONTROL_REG_1, odr, 8, 3)
    
    def set_acc_odr(self, odr):
        """Sets the accelerometer output-data-rate."""
        self.write_flag_gyro(LSM9DS1.ACC_CONTROL_REG_6, odr, 8, 3)
    
    def set_mag_odr(self, odr, mode = MAG_CONTINUOUS_MODE):
        """Sets the magnetometer output-data-rate and operating mode 
        (continuous, single conversion, power down)."""
        self.write_flag_mag(LSM9DS1.MAG_CONTROL_REG_3, mode, 2, 2)
        self.write_flag_mag(LSM9DS1.MAG_CONTROL_REG_1, odr, 5, 3)
    
    def set_gyro_range(self, gyro_range):
        modes = {LSM9DS1.ANGULAR_RATE_245DPS_RANGE : 0b00, 
                 LSM9DS1.ANGULAR_RATE_500DPS_RANGE : 0b01, 
                 LSM9DS1.ANGULAR_RATE_2000DPS_RANGE : 0b11}
        
        try :
            flag = modes[gyro_range]
        except :
            raise ValueError("Invalid gyro range, select a valid range.")
        
        self.gyro_scale = gyro_range
        self.write_flag_gyro(LSM9DS1.GYRO_CONTROL_REG_1, flag, 5, 2)
        
        
    def set_acc_range(self, acc_range):
        modes = {LSM9DS1.LIN_ACC_2G_RANGE : 0b00, LSM9DS1.LIN_ACC_4G_RANGE : 0b10, 
                 LSM9DS1.LIN_ACC_8G_RANGE : 0b11, LSM9DS1.LIN_ACC_16G_RANGE : 0b01}
        try:
            flag = modes[acc_range]
        except :
            raise ValueError("Invalid acceleration range, select a valid range.")
       
        self.acc_scale = acc_range
        self.write_flag_gyro(LSM9DS1.ACC_CONTROL_REG_6, flag, 5, 2)
        

    def set_mag_range(self, mag_range):
        modes = {LSM9DS1.MAGNETIC_4G_RANGE : 0b00, LSM9DS1.MAGNETIC_8G_RANGE : 0b01,
                 LSM9DS1.MAGNETIC_12G_RANGE : 0b10, LSM9DS1.MAGNETIC_16G_RANGE : 0b11}
        
        try :
            flag = modes[mag_range]
        except :
            raise ValueError("Invalid magnetic range, select a valid range.")
        
        self.mag_range = mag_range
        self.write_flag_mag(LSM9DS1.MAG_CONTROL_REG_2, flag, 7, 2)
        
        
    def set_acc_interrupt(self, x_threshold = None, x_detect_high = True,
                          y_threshold = None, y_detect_high = True,
                          z_threshold = None, z_detect_high = True,
                          and_event_combination = False, samples_to_recognize = 0, 
                          wait_before_exiting_interrupt = False, hardware_interrupt = False):
        """Calling this method will set the accelerometer interrupt. Calling it without
        specifying any parameters will reset the interrupt settings. \n
        * **x/y/z_threshold** When a measure exceeds (or is lower) than the threshold 
          value it triggers an interrupt. This values are expressed in g's (gravity = 9.8 ms^-2)
        * **x/y/z_detect_high** if True detects samples that are over the threshold.
        * **and_event_combination** if True all specified interrupt conditions must be
          triggered to generate an interrupt 
        * **samples_to_recognize** the number of samples that trigger the interrupt to measure
          before actually triggering the interrupt
        * **wait_before_exiting_interrupt** the number of samples to measure before exiting the interrupt
        * **hardware_interrupt** if True generates an hardware interrupt on the int1 pin"""
        def check_value(value):
            if abs(value) > 0xFF:
                raise ValueError("Threshold limit for acceleration interrupt is a one byte integer!")
        
        def get_range(sens):
            if sens == LSM9DS1.LIN_ACC_2G_RANGE:
                return 2
            elif sens == LSM9DS1.LIN_ACC_4G_RANGE:
                return 4
            elif sens == LSM9DS1.LIN_ACC_8G_RANGE:
                return 8
            elif sens == LSM9DS1.LIN_ACC_16G_RANGE:
                return 16
            else :
                raise ValueError("Invalid acceleration range / scale! Set a valid one with set_acc_range!")
        
        #reset settings
        self.write_byte_gyro(LSM9DS1.ACC_INT_CFG_REG, 0x80 if and_event_combination else 0x00)
        
        acc_range = get_range(self.acc_scale)
        
        if x_threshold:
            x_threshold = int(x_threshold * 255 / acc_range)
            check_value(x_threshold)
            self.write_byte_gyro(LSM9DS1.ACC_X_INT_THR_REG, x_threshold)
            self.write_flag_gyro(LSM9DS1.ACC_INT_CFG_REG, 0b10 if x_detect_high else 0b01, 2, 2)
        
        if y_threshold:
            y_threshold = int(y_threshold * 255 / acc_range)
            check_value(y_threshold)
            self.write_byte_gyro(LSM9DS1.ACC_Y_INT_THR_REG, y_threshold)
            self.write_flag_gyro(LSM9DS1.ACC_INT_CFG_REG, 0b10 if y_detect_high else 0b01, 4, 2)
        
        if z_threshold:
            z_threshold = int(z_threshold * 255 / acc_range)
            check_value(z_threshold)
            self.write_byte_gyro(LSM9DS1.ACC_Z_INT_THR_REG, z_threshold)
            self.write_flag_gyro(LSM9DS1.ACC_INT_CFG_REG, 0b10 if z_detect_high else 0b01, 6, 2)
        
        if samples_to_recognize > 0x7F: 
            raise ValueError("The maximum number of samples to take before launching an interrupt is {}".format(0x7F))
        self.write_byte_gyro(LSM9DS1.ACC_INT_DUR_REG, samples_to_recognize | (0x80 if wait_before_exiting_interrupt else 0x00))

        self.write_flag_gyro(LSM9DS1.INT1_CTRL_REG, hardware_interrupt, 7, 1)
        
    
    def set_gyro_interrupt(self, x_threshold = None, x_detect_high = True,
                           y_threshold = None, y_detect_high = True, 
                           z_threshold = None, z_detect_high = True, 
                           and_event_combination = False, samples_to_recognize = 0,
                           wait_before_exiting_interrupt = False, decrement_counter = False,
                           hardware_interrupt = False):
        """Calling this method will set the gyroscope interrupt. Calling it without
        specifying any parameters will reset the interrupt settings. \n
        * **x/y/z_threshold** the interrupt threshold value expressed in dps (degrees per second)
        * **x/y/z_detect_high** if True detects samples that are over the threshold
        * **and_event_combination** if True all specified interrupt conditions must be
          triggered to generate an interrupt 
        * **samples_to_recognize** the number of samples that trigger the interrupt to measure
          before actually triggering the interrupt
        * **wait_before_exiting_interrupt** the number of samples to measure before exiting the interrupt
        * **hardware_interrupt** if True generates an hardware interrupt on the int1 pin"""
        def to_15_bit_word(value):
            max_value = 16383
            min_value = -16384
            if value < min_value or value > max_value:
                raise ValueError("Threshold out of range.")
            
            abs_value = [b for b in value.to_bytes(length = 2, byteorder = 'big', signed = False)]
            if value < 0:
                neg_value = [not byte for byte in abs_value]
                neg_value[1] += 1
                neg_value[1] &= 0xFF
                neg_value[0] += 1 if neg_value[1] == 0 else 0
                neg_value[0] &= 0x7F 
                return neg_value
            else :
                return abs_value
        
        #reset interrupt
        self.write_byte_gyro(LSM9DS1.GYRO_INT_CFG_REG, 0x80 if and_event_combination else 0x00)
            
        if x_threshold:
            x_threshold = int(x_threshold * 1000 / self.gyro_scale)
            x_threshold_bytes = to_15_bit_word(x_threshold)
            x_threshold_bytes[0] |= ( decrement_counter << 7 )
            self.write_byte_gyro(LSM9DS1.GYRO_X_INT_THR_REG, x_threshold_bytes[0])
            self.write_byte_gyro(LSM9DS1.GYRO_X_INT_THR_REG + 1, x_threshold_bytes[1])
            self.write_flag_gyro(LSM9DS1.GYRO_INT_CFG_REG, 0b10 if x_detect_high else 0b01, 2, 2)
        
        if y_threshold:
            y_threshold = int(y_threshold * 1000 / self.gyro_scale)
            y_threshold_bytes = to_15_bit_word(y_threshold)
            self.write_byte_gyro(LSM9DS1.GYRO_Y_INT_THR_REG, y_threshold_bytes[0])
            self.write_byte_gyro(LSM9DS1.GYRO_Y_INT_THR_REG + 1, y_threshold_bytes[1])
            self.write_flag_gyro(LSM9DS1.GYRO_INT_CFG_REG, 0b10 if y_detect_high else 0b01, 4, 2)
        
        if z_threshold:
            z_threshold = int(z_threshold * 1000 / self.gyro_scale)
            z_threshold_bytes = to_15_bit_word(z_threshold)
            self.write_byte_gyro(LSM9DS1.GYRO_Z_INT_THR_REG, z_threshold_bytes[0])
            self.write_byte_gyro(LSM9DS1.GYRO_Z_INT_THR_REG + 1, z_threshold_bytes[1])
            self.write_flag_gyro(LSM9DS1.GYRO_INT_CFG_REG, 0b10 if z_detect_high else 0b01, 6, 2)

        if samples_to_recognize > 0x7F:
            raise ValueError("The maximum number of samples to take before launching an interrupt is {}".format(0x7F))
        self.write_byte_gyro(LSM9DS1.GYRO_INT_DUR_REG, samples_to_recognize | (0x80 if wait_before_exiting_interrupt else 0x00))
        
        self.write_flag_gyro(LSM9DS1.INT1_CTRL_REG, hardware_interrupt, 8, 1)

    
    def set_mag_interrupt(self, threshold = None, x_detect = False, y_detect = False,
                          z_detect = False, interrupt_active_high = False, 
                          latch_interrupt = False, hardware_interrupt = False):
        """Calling this method will set the gyroscope interrupt. Calling it without
        specifying any parameters will reset the interrupt settings.\n
        * **threshold** If measurement exceeds on positively (or negatively) the 
          threshold, an interrupt is generated, This value is expressed in gauss.
        * **x/y/z_detect** if True detects samples from the specified axes
        * **interrupt_active_high** if True the interrupt signal on the INT_M pin is HIGH
        * **latch_interrupt** Once latched, the INT_M pin remains in the same 
          state until get_mag_interrupt is called.
        * **hardware_interrupt** if True generates an hardware interrupt on the INT_M pin
        """
        flag = 0x00
        flag |= x_detect << 7
        flag |= y_detect << 6
        flag |= z_detect << 5
        flag |= interrupt_active_high << 2
        flag |= latch_interrupt << 1
        flag |= hardware_interrupt
        self.write_byte_mag(LSM9DS1.MAG_INT_CFG_REG, flag)
        
        if threshold:
            threshold = int(threshold * 1000 / self.mag_scale)
            if threshold > 0x7FFF or threshold < 0:
                raise ValueError("The threshold is out of range.")
            thr_bytes = [b for b in threshold.to_bytes(length = 2, byteorder = 'little', signed = False)]
            self.write_byte_mag(LSM9DS1.MAG_INT_THR_REG, thr_bytes[0])
            self.write_byte_mag(LSM9DS1.MAG_INT_THR_REG + 1, thr_bytes[1] & 0x7F)
    
    
    def get_acc_interrupt(self):
        """Returns a dictionary containing the information of the accelerometer's interrupt status"""
        flag = self.read_bytes_gyro(LSM9DS1.ACC_INT_SRC_REG, 1)[0]
        return {"interrupt occurred": bool(flag & 0x40),
                "z_high" : bool(flag & 0x20), "z_low" : bool(flag & 0x10), 
                "y_high" : bool(flag & 0x08), "y_low" : bool(flag & 0x04),
                "x_high" : bool(flag & 0x02), "x_low" : bool(flag & 0x01)}
        
    def get_gyro_interrupt(self):
        """Returns a dictionary containing the information of the gyroscope's interrupt status"""
        flag = self.read_bytes_gyro(LSM9DS1.GYRO_INT_SRC_REG, 1)[0]
        return {"interrupt occurred": bool(flag & 0x40),
                "z_high" : bool(flag & 0x20), "z_low" : bool(flag & 0x10), 
                "y_high" : bool(flag & 0x08), "y_low" : bool(flag & 0x04),
                "x_high" : bool(flag & 0x02), "x_low" : bool(flag & 0x01)}
        
    def get_mag_interrupt(self):
        """Returns a dictionary containing the information of the magnetometer's interrupt status"""
        flag = self.read_bytes_mag(LSM9DS1.MAG_INT_SRC_REG, 1)[0]
        return {"x_pos" : bool(flag & 0x80), "y_pos": bool(flag & 0x40),
                "z_pos" : bool(flag & 0x20), "x_neg" : bool(flag & 0x10), 
                "y_neg" : bool(flag & 0x08), "z_neg" : bool(flag & 0x04),
                "internal overflow" : bool(flag & 0x02), "interrupt occurred" : bool(flag & 0x01)}
    
    def get_temperature(self):
        temp_bytes = self.read_bytes_gyro(LSM9DS1.TEMPERATURE_REG, 2)
        temp_bytes[1] &= 0x0F
        return int.from_bytes(temp_bytes, byteorder = 'little', signed = True)
    
    def _multiple_read_and_convert(self, read_func, start_reg, length, value_length = 2):
        res = []
        val = read_func(start_reg, length)
        for i in range(0, length, value_length):
            res.append(int.from_bytes(val[i:i+value_length], byteorder = 'little', signed = True))
        return res
    
    def get_gyro(self):
        """Returns a triplet (list) containing the angular rate values on the 
        three axes in dps (degrees per second)."""
        raw = self._multiple_read_and_convert(self.read_bytes_gyro, LSM9DS1.GYRO_X_REG, 6)
        return list(map(lambda x : x / 1000 * self.gyro_scale, raw))
        
    def get_acc(self):
        """Returns a triplet (list) containing the linear acceleration values 
        on the three axes in g (gravity, 1 g = 9.80665 m/s2)."""
        raw = self._multiple_read_and_convert(self.read_bytes_gyro, LSM9DS1.ACC_X_REG, 6)
        return list(map(lambda x : x / 1000 * self.acc_scale, raw))
    
    def get_mag(self):
        """Returns a triplet (list) containing the magnetic field values 
        on the three axes in gauss."""
        raw = self._multiple_read_and_convert(self.read_bytes_mag, LSM9DS1.MAG_X_REG, 6)
        return list(map(lambda x : x / 1000 * self.mag_scale, raw))