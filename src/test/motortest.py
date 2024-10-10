from Adafruit_PCA9685 import PCA9685
from time import sleep

pwm = PCA9685()
pwm.set_pwm_freq(60)  # Set frequency to 60Hz

channel = 3
min_pulse = 150  # Min pulse length out of 4096
max_pulse = 600  # Max pulse length out of 4096

while True:
    pwm.set_pwm(channel, 0, min_pulse)
    sleep(1)
    pwm.set_pwm(channel, 0, max_pulse)
    sleep(1)