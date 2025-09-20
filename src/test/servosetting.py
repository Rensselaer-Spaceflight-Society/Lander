import time
# sudo pigpiod
# sudo killall pigpiod

# from gpiozero import Servo
import gpiozero
from gpiozero.pins.pigpio import PiGPIOFactory

gpiozero.Device.pin_factory = PiGPIOFactory('127.0.0.1')
from gpiozero import AngularServo
import RPi.GPIO as GPIO
import smbus					#import SMBus module of I2C
from time import sleep          #import time

servo_min = []
servo_max = [12]
# Todo tune with gpiozero angularservo libl
#min_angle=-135, max_angle=135,

servoPIN_n = 17
servoPIN_e = 23
servoPIN_s = 22
servoPIN_w = 27

# angles = [[0, 30], [0, 30], [?]]
servoN = AngularServo(servoPIN_n, initial_angle=0, min_angle=0, max_angle=180, min_pulse_width=0.0004, max_pulse_width=0.0026)
servoE = AngularServo(servoPIN_e, initial_angle=0, min_angle=0, max_angle=180, min_pulse_width=0.0004, max_pulse_width=0.0026)
servoS = AngularServo(servoPIN_s, initial_angle=0, min_angle=0, max_angle=180, min_pulse_width=0.0004, max_pulse_width=0.0026)
servoW = AngularServo(servoPIN_w, initial_angle=0, min_angle=0, max_angle=180, min_pulse_width=0.0004, max_pulse_width=0.0026)


while True:
    servoS.angle = 10
    sleep(2)
    servoS.angle = 100
    sleep(2)
    # servoE.angle = 30
    # sleep(.13)
    # servoE.angle = 60
    # sleep(.13)
    # servoN.angle = 0
    # sleep(.13)
    # servoN.angle = 30
    # sleep(.13)
    # servoW.angle = 0
    # sleep(.13)
    # servoW.angle = 30
    # sleep(.13)


# # p1 = GPIO.PWM(servoPIN_n, 50) # GPIO 17 for PWM with 50Hz
# # p1.start(2.5) # Initialization
# p1 = Servo(17)

# GPIO.setup(servoPIN_e, GPIO.OUT)

# p2 = GPIO.PWM(servoPIN_e, 50) # GPIO 27 for PWM with 50Hz
# p2.start(2.5) # Initialization
                                        
# GPIO.setup(servoPIN_s, GPIO.OUT)

# p3 = GPIO.PWM(servoPIN_s, 50) # GPIO 22 for PWM with 50Hz
# p3.start(2.5) # Initialization

# GPIO.setup(servoPIN_w, GPIO.OUT)

# p4 = GPIO.PWM(servoPIN_w, 50) # GPIO 23 for PWM with 50Hz
# p4.start(2.5) # Initialization

# time.sleep(2)

# # p1.ChangeDutyCycle(8)
# p1.min()

# time.sleep(5)
# p1.max()

# time.sleep(1)
# # p1.stop()
# p2.stop()
# p3.stop()
# p4.stop()


GPIO.cleanup()
		# servoS.angle = 0
		# servoE.angle = 30
		# servoN.angle = 0
		# servoW.angle = 0
		
		# if Ax > 0.3 and x_thres == 0.3:
		# 	servoS.angle = 30
		# 	sleep(.1)
		# if y > 0.3:
		# 	servoE.angle = 60
		# 	sleep(.1)
		# if Ax < -0.3:
		# 	servoN.angle = 30
		# 	sleep(.1)
		# if Ay < -0.3:
		# 	servoW.angle = 30
		# 	sleep(.1)