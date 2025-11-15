import time

import RPi.GPIO as GPIO
import smbus					#import SMBus module of I2C
from time import sleep          #import time

# sudo pigpiod
# sudo killall pigpiod

# from gpiozero import Servo
import gpiozero
from gpiozero.pins.pigpio import PiGPIOFactory

gpiozero.Device.pin_factory = PiGPIOFactory('127.0.0.1')
from gpiozero import AngularServo

# GPIO.setmode(GPIO.BCM)

# servoPIN_n = 17
# GPIO.setup(servoPIN_n, GPIO.OUT)

# p1 = GPIO.PWM(servoPIN_n, 50) # GPIO 17 for PWM with 50Hz
# p1.start(2.5) # Initialization

# servoPIN_e = 27
# GPIO.setup(servoPIN_e, GPIO.OUT)

# p2 = GPIO.PWM(servoPIN_e, 50) # GPIO 27 for PWM with 50Hz
# p2.start(2.5) # Initialization

# servoPIN_s = 22
# GPIO.setup(servoPIN_s, GPIO.OUT)

# p3 = GPIO.PWM(servoPIN_s, 50) # GPIO 22 for PWM with 50Hz
# p3.start(2.5) # Initialization

# servoPIN_w = 23
# GPIO.setup(servoPIN_w, GPIO.OUT)

# p4 = GPIO.PWM(servoPIN_w, 50) # GPIO 23 for PWM with 50Hz
# p4.start(2.5) # Initialization

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


bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()
servo_min = []
servo_max = [12]
# Todo tune with gpiozero angularservo libl
# min_angle=-135, max_angle=135,

servoPIN_n = 17
servoPIN_e = 23
servoPIN_s = 22
servoPIN_w = 27


# angles = [[0, 30], [0, 30], [?]]
servoN = AngularServo(servoPIN_n, initial_angle=0, min_angle=0, max_angle=180, min_pulse_width=0.0004, max_pulse_width=0.0026)
servoE = AngularServo(servoPIN_e, initial_angle=0, min_angle=0, max_angle=180, min_pulse_width=0.0004, max_pulse_width=0.0026)
servoS = AngularServo(servoPIN_s, initial_angle=0, min_angle=0, max_angle=180, min_pulse_width=0.0004, max_pulse_width=0.0026)
servoW = AngularServo(servoPIN_w, initial_angle=0, min_angle=0, max_angle=180, min_pulse_width=0.0004, max_pulse_width=0.0026)
print (" Reading Data of Gyroscope and Accelerometer")

try:
	x_thres = -0.2
	posx_thres = 0.2
	posy_thres = 0.2
	y_thres = -0.2
	dummy = 0.2
	neg_dummy = -0.2
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

		# if Ax > 0.15:
		# 	p1.ChangeDutyCycle(10)
		# 	p3.ChangeDutyCycle(5)
		# 	time.sleep(0.1)
		# if Ay > 0.15:	
		# 	p4.ChangeDutyCycle(10)
		# 	p2.ChangeDutyCycle(5)
		# 	time.sleep(0.1)
		
		# if Ax < -0.15:
		# 	print("Two")
		# 	p1.ChangeDutyCycle(5)
		# 	p3.ChangeDutyCycle(10)
		# 	time.sleep(0.1)
		
		# if Ay < -0.15:
		# 	p4.ChangeDutyCycle(5)
		# 	p2.ChangeDutyCycle(10)
		# 	time.sleep(0.1)


		print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 	
		sleep(.1)
		# servoS.angle = 0
		# servoE.angle = 30
		# servoN.angle = 0
		# servoW.angle = 0
		
		# if Ax > dummy and posx_thres == dummy:
		# 	servoS.angle = 20
		# 	posx_thres = 0.1
		# 	sleep(.1)
		# elif posx_thres == 0.1 and Ax < posx_thres:
		# 	servoS.angle = 0
		# 	posx_thres = dummy
		# 	sleep(.1)
		# if Ay > dummy and posy_thres == dummy:
		# 	servoE.angle = 50
		# 	posy_thres = 0.1
		# 	sleep(.1)
		# elif posy_thres == .1 and Ay < posy_thres:
		# 	servoE.angle = 30
		# 	posy_thres = dummy
		# 	sleep(.1)

		# if Ax < neg_dummy and x_thres == neg_dummy:
		# 	servoN.angle = 20
		# 	x_thres = -0.1
		# 	sleep(.1)
		# elif x_thres == -0.1 and Ax > x_thres:
		# 	servoN.angle = 0
		# 	x_thres = neg_dummy
		# 	sleep(.1)

		# if Ay < neg_dummy and y_thres == neg_dummy:
		# 	servoW.angle = 20
		# 	y_thres = -0.1
		# 	sleep(.1)
		# elif y_thres == -0.1 and Ay > y_thres:
		# 	servoW.angle = 0
		# 	y_thres = neg_dummy
		# 	sleep(.1)

except KeyboardInterrupt:
	print("stop")