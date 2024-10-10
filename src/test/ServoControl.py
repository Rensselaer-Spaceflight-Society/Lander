# We imports the GPIO module
import RPi.GPIO as GPIO
# We import the command sleep from time
from time import sleep

current_heading = 0

# Stops all warnings from appearing
GPIO.setwarnings(False)

# We name all the pins on BOARD mode
GPIO.setmode(GPIO.BOARD)
# Set an output for the PWM Signal
GPIO.setup(16, GPIO.OUT)

# Set up the  PWM on pin #16 at 50Hz
pwm = GPIO.PWM(16, 50)
pwm.start(0)  # Start the servo with 0 duty cycle ( at 0 deg position )

def change_servo_heading(degrees):
    duty_cycle = 11 / 180 * degrees - current_heading

    current_heading += duty_cycle
    pwm.ChangeDutyCycle(duty_cycle)

    pwm.stop(0) 

if __name__ == "__main__":
    # Reference:
    # https://www.kjell.com/globalassets/mediaassets/701916_87897_datasheet_en.pdf?ref=4287817A7A

    # Set up the  PWM on pin #16 at 50Hz
    pwm = GPIO.PWM(16, 50)
    pwm.start(0)  # Start the servo with 0 duty cycle ( at 0 deg position )

    # 11 maps 180 degrees
    pwm.ChangeDutyCycle(11)

# for i in range(100):
#     pwm.ChangeDutyCycle(i / 10)
#     sleep(0.5)

# pwm.ChangeDutyCycle(5)
# pwm.ChangeDutyCycle(5)  # Tells the servo to turn to the left ( -90 deg position )
# sleep(0.5)  # Tells the servo to Delay for 5sec
# pwm.ChangeDutyCycle(7.5)  # Tells the servo to turn to the neutral position ( at 0 deg position )
# sleep(0.5)  # Tells the servo to Delay for 5sec
# pwm.ChangeDutyCycle(10)  # Tells the servo to turn to the right ( +90 deg position )
# sleep(0.5)  # Tells the servo to Delay for 5sec
    pwm.stop(0)  # Stop the servo with 0 duty cycle ( at 0 deg position )   
GPIO.cleanup()  # Clean up all the ports we've used.