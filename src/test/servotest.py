import gpiozero
from gpiozero.pins.pigpio import PiGPIOFactory
gpiozero.Device.pin_factory = PiGPIOFactory('127.0.0.1')
from gpiozero import AngularServo
import time

servoPIN_n = 17
servoPIN_e = 23
servoPIN_s = 22
servoPIN_w = 27

servoN = AngularServo(servoPIN_n, min_pulse_width=0.0004, max_pulse_width=0.0026)
servoE = AngularServo(servoPIN_e, initial_angle=0, min_angle=0, max_angle=180, min_pulse_width=0.0004, max_pulse_width=0.0026)
servoS = AngularServo(servoPIN_s, initial_angle=0, min_angle=0, max_angle=180, min_pulse_width=0.0004, max_pulse_width=0.0026)

# servo_n = gpiozero.Servo(servoPIN_n)

# pwm.set_mode(servoPIN_w, pigpio.OUTPUT)
# pwm.set_PWM_frequency( servoPIN_w, 50 )
# pwm.set_mode(servoPIN_e, pigpio.OUTPUT)
# pwm.set_PWM_frequency( servoPIN_e, 50 )

try:
	while True:
		print('angle: 90')
		servoN.angle = 90
		time.sleep(1)

		print('angle: 0')
		servoN.angle = 0
		time.sleep(1)

		print('angle: -90')
		servoN.angle = -90
		time.sleep(1)

except KeyboardInterrupt:
	# # pwm.set_PWM_dutycycle( servoPIN_w, 0 )
	# pwm.set_servo_pulsewidth( servoPIN_w, 0 )
	servoN.angle = 0

	# # pwm.set_PWM_dutycycle( servoPIN_e, 0 )
	# pwm.set_servo_pulsewidth( servoPIN_e, 0 )
	# pwm.stop()
	print("stop")