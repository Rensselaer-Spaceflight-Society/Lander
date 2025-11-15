import pigpio
import time

servo = 27
servo2 = 23


# more info at http://abyz.me.uk/rpi/pigpio/python.html#set_servo_pulsewidth

pwm = pigpio.pi()
pwm.set_mode(servo, pigpio.OUTPUT)
pwm.set_mode(servo2, pigpio.OUTPUT)


pwm.set_PWM_frequency( servo, 50 )
pwm.set_PWM_frequency( servo2, 50 )


print( "45 deg" )
pwm.set_servo_pulsewidth( servo2, 1000 )
time.sleep( 3 )

print( "90 deg" )
pwm.set_servo_pulsewidth( servo2, 1500 )
time.sleep( 3 )

print( "135 deg" )
pwm.set_servo_pulsewidth(servo2, 2000)
time.sleep( 3 )


print( "0 deg" )
pwm.set_servo_pulsewidth( servo, 500 )
time.sleep( 3 )

print( "90 deg" )
pwm.set_servo_pulsewidth( servo, 1500 )
time.sleep( 3 )

print( "180 deg" )
pwm.set_servo_pulsewidth(servo, 2500)
time.sleep( 3 )

# turning off servo
pwm.set_PWM_dutycycle( servo, 0 )
pwm.set_PWM_frequency( servo, 0 )

pwm.set_PWM_dutycycle( servo2, 0 )
pwm.set_PWM_frequency( servo2, 0 )