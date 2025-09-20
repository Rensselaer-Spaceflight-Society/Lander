import time

import RPi.GPIO as GPIO
import smbus					#import SMBus module of I2C
from time import sleep          #import time
import math

# sudo pigpiod
# sudo killall pigpiod

# from gpiozero import Servo
import gpiozero
from gpiozero.pins.pigpio import PiGPIOFactory

gpiozero.Device.pin_factory = PiGPIOFactory('127.0.0.1')
from gpiozero import AngularServo

# GPIO.setmode(GPIO.BCM)

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
prevN = 0.0 #+y
prevS = 0.0 #-y
prevE = 0.0	#+x
prevW = 0.0	#-x


currN = 0.0
currS = 0.0
currE = 0.0
currW = 0.0



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

	# kp = 0.005
	# ki = 0.0003
	# kd = 0.0002
	kp = 10
	ki = 0.3
	kd = 0.01
	iS = 0.0
	iN = 0.0
	iW = 0.0
	iE = 0.0
	intgralS = 0.0
	intgralN = 0.0
	intgralW = 0.0
	intgralE = 0.0

	# MOVE THESE UP
	iteration_time = 0.1
	integral_prior = 0
	error_prior = 0

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
		# if abs(Ax) < 0.1:
		# 	Ax = 0.0
		# if abs(Ay) < 0.1:
		# 	Ay = 0.0
		# if abs(Az) < 0.08:
		# 	Az = 0.0

		Gx = gyro_x/131.0
		Gy = gyro_y/131.0
		Gz = gyro_z/131.0

		XZ_angle = math.atan(Ax/Az)
		YZ_angle = math.atan(Ay/Az)

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


		# print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 	
		sleep(.1)
		# servoS.angle = 0
		# servoE.angle = 30
		# servoN.angle = 0
		# servoW.angle = 0
		# porS = -YZ_angle*kp
		# derS = -kd*(currS - prevS) / 0.1
		# iS += currS * 0.1
		# intgralS -= iS
		# prevS = currS
		# currS = porS + derS + (ki * intgralS)



		# porS = -YZ_angle*kp
		# derS = kd*(currS - prevS) / 0.1
		# iS += -YZ_angle*0.1 #currS * 0.1 TOD: WE LEFT OFF HERE
		# intgralS += iS
		# prevS = currS
		# currS = 0.5 + porS + (ki * intgralS) #+ derS
		# print(currS, porS, intgralS, derS, -YZ_angle)

		# while(1) {
			# error = desired_value – actual_value
			# integral = integral_prior + error * iteration_time
			# derivative = (error – error_prior) / iteration_time
			# output = KP*error + KI*integral + KD*derivative + bias
			# error_prior = error
			# integral_prior = integral
			# sleep(iteration_time)
		# }



		error = -0.067 - (-YZ_angle)
		integral = integral_prior + error * iteration_time
		derivative = (error - error_prior) / iteration_time
		output = kp*error + ki*integral + kd*derivative + 0
		error_prior = error
		integral_prior = integral
		# print(kp*error, ki*integral, kd*derivative, output)
		print(output, 0.5*(math.degrees(output)+360))

		#TODO:
		# fix servos to test if we want kd or not
		# Update pid code for other servos and clean up formatting
		# Fix git repo

		# print(-YZ_angle)
		# servoS.angle = math.degrees(output)


		# porN = Ax*kp
		# derN = kd*(currN - prevN)/0.1
		# iN -= Ax
		# intgralN -= iN*0.1
		# servoN.angle = porN + derN + (ki * intgralN)

		# porE = Ay * kp
		# derE = kd*(currE - prevE) / 0.1
		# iE -= Ay
		# intgralE -= iE * 0.1
		# servoE.angle = porE + derE + (ki * intgralE)

		# porW = -Ay * kp
		# derW = kd*(currW - prevW) / 0.1
		# iW += Ay
		# intgralW += iW * 0.1
		# servoW.angle = porW + derW + (ki * intgralW)




		"""
		
		if Ax > dummy and posx_thres == dummy:
			servoS.angle = 20
			posx_thres = 0.1
			sleep(.1)
		elif posx_thres == 0.1 and Ax < posx_thres:
			servoS.angle = 0
			posx_thres = dummy
			sleep(.1)
		if Ay > dummy and posy_thres == dummy:
			servoE.angle = 50
			posy_thres = 0.1
			sleep(.1)
		elif posy_thres == .1 and Ay < posy_thres:
			servoE.angle = 30
			posy_thres = dummy
			sleep(.1)

		if Ax < neg_dummy and x_thres == neg_dummy:
			servoN.angle = 20
			x_thres = -0.1
			sleep(.1)
		elif x_thres == -0.1 and Ax > x_thres:
			servoN.angle = 0
			x_thres = neg_dummy
			sleep(.1)

		if Ay < neg_dummy and y_thres == neg_dummy:
			servoW.angle = 20
			y_thres = -0.1
			sleep(.1)
		elif y_thres == -0.1 and Ay > y_thres:
			servoW.angle = 0
			y_thres = neg_dummy
			sleep(.1)
        """
except KeyboardInterrupt:
	print("stop")