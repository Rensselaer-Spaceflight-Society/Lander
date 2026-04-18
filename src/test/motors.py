import time

import smbus
import math
import pigpio

import gpiozero
from gpiozero.pins.pigpio import PiGPIOFactory

gpiozero.Device.pin_factory = PiGPIOFactory('127.0.0.1')

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
	high = bus.read_byte_data(Device_Address, addr)
	low = bus.read_byte_data(Device_Address, addr+1)

	#concatenate higher and lower value
	value = ((high << 8) | low)
	
	#to get signed value from mpu6050
	if(value > 32768):
		value = value - 65536
	return value


# Calibration procedure for ESC
def calibrate():   #This is the auto calibration procedure of a normal ESC
    pwm.set_servo_pulsewidth(ESC, 0)
    print("Disconnect the battery and press Enter")
    inp = input()
    if inp == '':
        pwm.set_servo_pulsewidth(ESC, max_value)
        print("Connect the battery now, after two beeps press Enter")
        inp = input()
        if inp == '':
            pwm.set_servo_pulsewidth(ESC, min_value)
            time.sleep(12)
            pwm.set_servo_pulsewidth(ESC, 0)
            time.sleep(2)
            pwm.set_servo_pulsewidth(ESC, min_value)
            time.sleep(1)
            print('Motor Calibrated')
            # control()

# Stop all servos and ESC
def stop(): #This will stop every action your Pi is performing for ESC ofcourse.
    pwm.set_PWM_dutycycle( servoPIN_w, 0 )
    pwm.set_PWM_frequency( servoPIN_w, 0 )

    pwm.set_PWM_dutycycle( servoPIN_e, 0 )
    pwm.set_PWM_frequency( servoPIN_e, 0 )

    pwm.set_PWM_dutycycle( servoPIN_n, 0 )
    pwm.set_PWM_frequency( servoPIN_n, 0 )

    pwm.set_PWM_dutycycle( servoPIN_s, 0 )
    pwm.set_PWM_frequency( servoPIN_s, 0 )
    
    pwm.set_servo_pulsewidth(ESC, 0)
    pwm.stop()


bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

# servo s not working
servoPIN_n = 17
servoPIN_e = 23
servoPIN_s = 22
servoPIN_w = 27
ESC = 4

pwm = pigpio.pi()
pwm.set_servo_pulsewidth(servoPIN_w, 0)
pwm.set_mode(servoPIN_w, pigpio.OUTPUT)
pwm.set_PWM_frequency( servoPIN_w, 50 )

pwm.set_servo_pulsewidth(servoPIN_e, 0)
pwm.set_mode(servoPIN_e, pigpio.OUTPUT)
pwm.set_PWM_frequency( servoPIN_e, 50 )

pwm.set_servo_pulsewidth(servoPIN_n, 0)
pwm.set_mode(servoPIN_n, pigpio.OUTPUT)
pwm.set_PWM_frequency( servoPIN_n, 50 )

pwm.set_servo_pulsewidth(servoPIN_s, 0)
pwm.set_mode(servoPIN_s, pigpio.OUTPUT)
pwm.set_PWM_frequency( servoPIN_s, 50 )

pwm.set_servo_pulsewidth(ESC, 0)
pwm.set_mode(ESC, pigpio.OUTPUT)

Gx_offset = -0.28100763358778624
Gy_offset = 0.0648854961832061
Gz_offset = 0.09798473282442748
Ax_moffset = -0.99430131
Ax_boffset = 0.02041231
Ay_moffset = -0.99869671
Ay_boffset = 0.02664353
Az_moffset = -0.99757678
Az_boffset = 0.0469103

# Comparison Coefficients
# Gx_offset = 0
# Gy_offset = 0
# Gz_offset = 0
# Ax_moffset = 1
# Ax_boffset = 0
# Ay_moffset = 1
# Ay_boffset = 0
# Az_moffset = 1
# Az_boffset = 0

max_value = 2000 #change this if your ESC's max value is different or leave it be
min_value = 700  #change this if your ESC's min value is different or leave it be


print (" Reading Data of Gyroscope and Accelerometer")

try:	
    kpy = 10
    kiy = 0.3
    kdy = 0.01
    kpx = 10
    kix = 0.3
    kdx = 0.01
	
    iteration_time = 0.1
    XZ_integral = 0
    XZ_error_prior = 0

    YZ_integral = 0
    YZ_error_prior = 0

    calibrate()

    pwm.set_servo_pulsewidth(ESC, 770)

    while True:
        
        #Read Accelerometer raw value
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)

        #Read Gyroscope raw value
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)

        #Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ax = acc_x/16384.0
        Ay = acc_y/16384.0
        Az = acc_z/16384.0

        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0

        Ax = (Ax * Ax_moffset) + Ax_boffset
        Ay = (Ay * Ay_moffset) + Ay_boffset
        Az = (Az * Az_moffset) + Az_boffset

        Gx -= Gx_offset
        Gy -= Gy_offset
        Gz -= Gz_offset

        XZ_angle = math.atan2(Ax,Az)
        YZ_angle = math.atan2(Ay,Az)
        

        # Print Data of Gyroscope and Accelerometer
        # print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.8f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 	
        # print ("Ax=%.8f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 	
        
        time.sleep(.1)

        YZ_error = -0.067 - (-YZ_angle)
        YZ_integral += YZ_error * iteration_time
        YZ_derivative = (YZ_error - YZ_error_prior) / iteration_time
        YZ_output = kpy*YZ_error + kiy*YZ_integral + kdy*YZ_derivative + 0
        YZ_error_prior = YZ_error
        # print(kpy*YZ_error, kiy*YZ_integral, kdy*YZ_derivative, YZ_output)
        # print((math.degrees(-YZ_angle)+360), YZ_output, 0.5*(math.degrees(YZ_output)+360), 1500+YZ_output*2000/8, 1500-YZ_output*2000/8)
        pwm.set_servo_pulsewidth( servoPIN_w, min(max(1500+YZ_output*2000/8, 1000), 2000))
        pwm.set_servo_pulsewidth( servoPIN_e, min(max(1500-YZ_output*2000/8, 1000), 2000))
        
        XZ_error = -0.067 - (-XZ_angle)
        XZ_integral += XZ_error * iteration_time
        XZ_derivative = (XZ_error - XZ_error_prior) / iteration_time
        XZ_output = kpx*XZ_error + kix*XZ_integral + kdx*XZ_derivative + 0
        XZ_error_prior = XZ_error
        # print(kpx*XZ_error, kix*XZ_integral, kdx*XZ_derivative, XZ_output)
        # print((math.degrees(-XZ_angle)+360), XZ_output, 0.5*(math.degrees(XZ_output)+360), 1500+XZ_output*2000/8, 1500-XZ_output*2000/8)
        pwm.set_servo_pulsewidth( servoPIN_n, min(max(1500+XZ_output*2000/8, 1000), 2000))
        pwm.set_servo_pulsewidth( servoPIN_s, min(max(1500-XZ_output*2000/8, 1000), 2000))
        

        #TODO:
        # fix servos to test if we want kd or not
        # Update pid code for other servos and clean up formatting
        # Fix git repo

        # PID no longer works?
        # west servo PID works but offset breaks east servo
        # south servo is broke
        # now, none are working???

        # print(-YZ_angle)
        # servoS.angle = math.degrees(output)
		
except KeyboardInterrupt:
    pwm.set_PWM_dutycycle( servoPIN_w, 0 )
    pwm.set_PWM_frequency( servoPIN_w, 0 )

    pwm.set_PWM_dutycycle( servoPIN_e, 0 )
    pwm.set_PWM_frequency( servoPIN_e, 0 )

    pwm.set_PWM_dutycycle( servoPIN_n, 0 )
    pwm.set_PWM_frequency( servoPIN_n, 0 )

    pwm.set_PWM_dutycycle( servoPIN_s, 0 )
    pwm.set_PWM_frequency( servoPIN_s, 0 )
    stop()
    print("stop")